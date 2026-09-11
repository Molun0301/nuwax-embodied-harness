// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Point-LIO + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into the Point-LIO core: SDK callbacks feed
// CustomMsg/Imu to the IESKF estimator, which performs LiDAR-inertial SLAM.
// Sensor-frame point clouds and odometry are published on LCM.
//
// Usage:
//   ./pointlio_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --filter_size_surf 0.2 --ivox_grid_resolution 2.0 ... \   # tuning as plain CLI args
//       --host_ip 192.168.1.5 --lidar_ip 192.168.1.155

#include <lcm/lcm-cpp.hpp>

#include <algorithm>
#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"
#include "imu_frame_transform.hpp"
#include "scan_imu_buffer.hpp"
#include "native_input_subscription.hpp"
#include "native_lio_arguments.hpp"

#include "dimos_native_module.hpp"

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// Point-LIO (header-only core, compiled sources linked via CMake)
#include "pointlio.hpp"
#include "pointlio_debug.hpp"

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_IMU;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_typed_input_failed{false};
static lcm::LCM* g_lcm = nullptr;
static PointLio* g_point_lio = nullptr;

// Parse a comma-separated list of doubles (CLI vector args); empty on bad input.
static std::vector<double> parse_doubles(const std::string& csv) {
    std::vector<double> out;
    size_t i = 0;
    while (i < csv.size()) {
        size_t j = csv.find(',', i);
        if (j == std::string::npos) { j = csv.size(); }
        try {
            out.push_back(std::stod(csv.substr(i, j - i)));
        } catch (...) {
            return {};
        }
        i = j + 1;
    }
    return out;
}

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_frame_id;          // required via --frame_id
static std::string g_child_frame_id;     // required via --child_frame_id
static std::string g_sensor_frame_id;    // required via --sensor_frame_id
static std::string g_input_lidar_topic;
static std::string g_input_imu_topic;
static bool g_external_input = false;
static std::array<double, 9> g_body_from_sensor_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
static float g_frequency = 10.0f;

// Frame accumulator (Livox SDK raw → CustomMsg)
static std::mutex g_pc_mutex;
// Serializes all Point-LIO EKF access. The SDK delivers IMU on its own callback
// thread (on_imu_data → feed_imu) while the main loop runs feed_lidar/process/
// get_* — Point-LIO's estimator is not thread-safe, so without this the two
// threads race on the EKF state and occasionally emit a corrupt 2nd trajectory.
// Distinct from g_pc_mutex (which only guards the point accumulator) so incoming
// point packets can still accumulate while the EKF is processing.
static std::mutex g_lio_mutex;
static std::vector<custom_messages::CustomPoint> g_accumulated_points;
static uint64_t g_frame_start_ns = 0;
static bool g_frame_has_timestamp = false;
static uint64_t g_last_typed_cloud_stamp_ns = 0;
static bool g_has_last_typed_cloud_stamp = false;
static uint64_t g_last_typed_imu_stamp_ns = 0;
static bool g_has_last_typed_imu_stamp = false;
struct TypedImuSample {
    uint64_t timestamp_ns;
    boost::shared_ptr<custom_messages::Imu> message;
};
static std::vector<TypedImuSample> g_typed_imu_samples;
static std::optional<uint64_t> g_last_fed_typed_imu_stamp_ns;

static constexpr uint64_t NANOS_PER_SECOND = 1000000000ULL;

static uint64_t get_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

static bool next_source_timestamp(
    const custom_messages::Odometry& odometry,
    uint64_t last_output_source_timestamp_ns,
    bool has_last_output_source_timestamp,
    uint64_t& output_timestamp_ns
) {
    if (odometry.header.frame_id.empty()) { return false; }
    const uint64_t seconds = static_cast<uint64_t>(odometry.header.stamp.sec);
    const uint64_t nanoseconds = static_cast<uint64_t>(odometry.header.stamp.nsecs);
    if (nanoseconds >= NANOS_PER_SECOND ||
        seconds > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    const uint64_t timestamp_ns = seconds * NANOS_PER_SECOND + nanoseconds;
    if (has_last_output_source_timestamp &&
        timestamp_ns <= last_output_source_timestamp_ns) {
        return false;
    }
    output_timestamp_ns = timestamp_ns;
    return true;
}

using dimos::make_header;

static std_msgs::Header make_source_header(
    const std::string& frame_id,
    uint64_t timestamp_ns
) {
    auto header = make_header(frame_id, 0.0);
    header.stamp.sec = static_cast<int32_t>(timestamp_ns / NANOS_PER_SECOND);
    header.stamp.nsec = static_cast<int32_t>(timestamp_ns % NANOS_PER_SECOND);
    return header;
}

// 密集点已按扫描内估计轨迹补偿到同源位姿对应的传感器坐标。
// 保留完整输出点数；世界注册仍由Geometry按这一对位姿与点云执行。
static void publish_lidar(PointCloudXYZI::Ptr cloud, uint64_t timestamp_ns, const std::string& topic = "") {
    const std::string& chan = topic.empty() ? g_lidar_topic : topic;
    if (!g_lcm || !cloud || cloud->empty() || chan.empty()) { return; }

    int num_points = static_cast<int>(cloud->size());

    sensor_msgs::PointCloud2 pc;
    pc.header = make_source_header(g_sensor_frame_id, timestamp_ns);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields_length = 4;
    pc.fields.resize(4);

    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;
        return field;
    };

    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);

    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;

    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int point_idx = 0; point_idx < num_points; ++point_idx) {
        const size_t byte_offset = static_cast<size_t>(point_idx) * 16;
        const float x = cloud->points[point_idx].x;
        const float y = cloud->points[point_idx].y;
        const float z = cloud->points[point_idx].z;
        const float intensity = cloud->points[point_idx].intensity;
        std::memcpy(pc.data.data() + byte_offset, &x, sizeof(float));
        std::memcpy(pc.data.data() + byte_offset + 4, &y, sizeof(float));
        std::memcpy(pc.data.data() + byte_offset + 8, &z, sizeof(float));
        std::memcpy(pc.data.data() + byte_offset + 12, &intensity, sizeof(float));
    }

    g_lcm->publish(chan, &pc);
}

static nav_msgs::Odometry make_odometry_message(
    const custom_messages::Odometry& odom,
    uint64_t timestamp_ns
) {
    nav_msgs::Odometry msg;
    msg.header = make_source_header(g_frame_id, timestamp_ns);
    msg.child_frame_id = g_child_frame_id;

    // Pose in the SLAM/sensor frame.
    msg.pose.pose.position.x = odom.pose.pose.position.x;
    msg.pose.pose.position.y = odom.pose.pose.position.y;
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
    msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
    msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;

    // The pinned Point-LIO core does not populate pose covariance.
    std::memset(msg.pose.covariance, 0, sizeof(msg.pose.covariance));

    // Velocity from Point-LIO's IESKF state (its key output over FAST-LIO).
    msg.twist.twist.linear.x = odom.twist.twist.linear.x;
    msg.twist.twist.linear.y = odom.twist.twist.linear.y;
    msg.twist.twist.linear.z = odom.twist.twist.linear.z;
    msg.twist.twist.angular.x = odom.twist.twist.angular.x;
    msg.twist.twist.angular.y = odom.twist.twist.angular.y;
    msg.twist.twist.angular.z = odom.twist.twist.angular.z;
    std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

    return msg;
}

static void publish_odometry(const nav_msgs::Odometry& msg) {
    if (!g_lcm) { return; }
    g_lcm->publish(g_odometry_topic, &msg);
}


// Canonical typed sensor ingress. Supplying both input channels disables
// direct Livox callbacks while preserving private output channels.
static const sensor_msgs::PointField* find_typed_field(
    const sensor_msgs::PointCloud2* msg, const std::string& name
) {
    const sensor_msgs::PointField* result = nullptr;
    for (const auto& field : msg->fields) {
        if (field.name != name) { continue; }
        if (result != nullptr) { return nullptr; }
        result = &field;
    }
    return result;
}

static bool read_float32_field(
    const sensor_msgs::PointCloud2* msg, const sensor_msgs::PointField* field,
    size_t base, float& value
) {
    if (!msg || !field || field->datatype != 7 || field->count != 1 ||
        field->offset < 0 || static_cast<size_t>(field->offset) > msg->point_step ||
        base + static_cast<size_t>(field->offset) + sizeof(float) > msg->data.size()) {
        return false;
    }
    std::memcpy(&value, msg->data.data() + base + static_cast<size_t>(field->offset), sizeof(float));
    return std::isfinite(value);
}

static bool read_offset_ns(
    const sensor_msgs::PointCloud2* msg, const sensor_msgs::PointField* field,
    size_t base, uint64_t& value
) {
    if (!msg || !field || field->datatype != 6 || field->count != 1 ||
        field->offset < 0 || static_cast<size_t>(field->offset) > msg->point_step ||
        base + static_cast<size_t>(field->offset) + sizeof(uint32_t) > msg->data.size()) {
        return false;
    }
    uint32_t raw = 0;
    std::memcpy(&raw, msg->data.data() + base + static_cast<size_t>(field->offset), sizeof(raw));
    value = raw;
    return true;
}

static bool typed_field_fits(const sensor_msgs::PointField* field, int point_step, int datatype_size) {
    return field != nullptr && field->count == 1 && field->offset >= 0 &&
           field->offset <= point_step && datatype_size <= point_step - field->offset;
}

static bool typed_imu_covers_scan(uint64_t scan_start_ns, uint64_t scan_end_ns) {
    std::lock_guard<std::mutex> lock(g_lio_mutex);
    return !g_typed_imu_samples.empty() &&
           g_typed_imu_samples.front().timestamp_ns <= scan_start_ns &&
           g_typed_imu_samples.back().timestamp_ns >= scan_end_ns;
}

static void on_typed_cloud(const lcm::ReceiveBuffer*, const std::string&, const sensor_msgs::PointCloud2* msg, void*) {
    if (!g_running.load() || !g_external_input || !msg) { return; }
    if (msg->header.frame_id.empty() || msg->header.frame_id != g_sensor_frame_id ||
        msg->header.stamp.sec < 0 || msg->header.stamp.nsec < 0 ||
        msg->header.stamp.nsec >= static_cast<int32_t>(NANOS_PER_SECOND) || msg->is_bigendian) {
        fprintf(stderr, "[pointlio] reject typed cloud: invalid frame, timestamp, or endianness\\n");
        return;
    }
    if (msg->width <= 0 || msg->height <= 0 || msg->point_step < 12 || msg->row_step < 0) {
        fprintf(stderr, "[pointlio] reject typed cloud: invalid dimensions\\n");
        return;
    }
    const auto* x_field = find_typed_field(msg, "x");
    const auto* y_field = find_typed_field(msg, "y");
    const auto* z_field = find_typed_field(msg, "z");
    const auto* time_field = find_typed_field(msg, "offset_ns");
    if (!typed_field_fits(x_field, msg->point_step, 4) || x_field->datatype != 7 ||
        !typed_field_fits(y_field, msg->point_step, 4) || y_field->datatype != 7 ||
        !typed_field_fits(z_field, msg->point_step, 4) || z_field->datatype != 7 ||
        !typed_field_fits(time_field, msg->point_step, 4) || time_field->datatype != 6) {
        fprintf(stderr, "[pointlio] reject typed cloud: require float32 xyz and uint32 offset_ns\\n");
        return;
    }
    const size_t width = static_cast<size_t>(msg->width);
    const size_t height = static_cast<size_t>(msg->height);
    const size_t point_step = static_cast<size_t>(msg->point_step);
    const size_t row_step = static_cast<size_t>(msg->row_step);
    if (width > SIZE_MAX / point_step || row_step < width * point_step ||
        height > SIZE_MAX / row_step || msg->data.size() < height * row_step ||
        height > SIZE_MAX / width || width * height > 2'000'000) {
        fprintf(stderr, "[pointlio] reject typed cloud: row/data bounds invalid\\n");
        return;
    }
    const uint64_t stamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * NANOS_PER_SECOND +
        static_cast<uint64_t>(msg->header.stamp.nsec);
    std::vector<custom_messages::CustomPoint> accepted_points;
    accepted_points.reserve(width * height);
    uint64_t max_offset_ns = 0;
    for (size_t row = 0; row < height; ++row) {
        for (size_t column = 0; column < width; ++column) {
            const size_t base = row * row_step + column * point_step;
            float x, y, z;
            uint64_t offset_ns;
            if (!read_float32_field(msg, x_field, base, x) || !read_float32_field(msg, y_field, base, y) ||
                !read_float32_field(msg, z_field, base, z) || !read_offset_ns(msg, time_field, base, offset_ns)) {
                fprintf(stderr, "[pointlio] reject typed cloud: non-finite or malformed point\\n");
                return;
            }
            max_offset_ns = std::max(max_offset_ns, offset_ns);
            custom_messages::CustomPoint point;
            point.x = x; point.y = y; point.z = z;
            point.reflectivity = 0; point.tag = 0; point.line = 0;
            point.offset_time = static_cast<uli>(offset_ns);
            accepted_points.push_back(point);
        }
    }
    if (stamp_ns > UINT64_MAX - max_offset_ns ||
        !typed_imu_covers_scan(stamp_ns, stamp_ns + max_offset_ns)) {
        fprintf(stderr, "[pointlio] reject typed cloud: IMU does not cover scan window\n");
        return;
    }
    // 原载荷保留设备顺序；内核计算副本按真实时间排列，整点移动以保持坐标对应。
    // 同一时刻的点合法且保持原相对顺序，不改时间、不丢点。
    const auto earlier_point = [](const auto& left, const auto& right) {
        return left.offset_time < right.offset_time;
    };
    if (!std::is_sorted(accepted_points.begin(), accepted_points.end(), earlier_point)) {
        std::stable_sort(accepted_points.begin(), accepted_points.end(), earlier_point);
    }
    // A PointCloud2 message is one scan. Replace the pending frame atomically;
    // never append a later scan to the prior scan's timebase.
    std::lock_guard<std::mutex> lock(g_pc_mutex);
    if (g_has_last_typed_cloud_stamp && stamp_ns <= g_last_typed_cloud_stamp_ns) {
        fprintf(stderr, "[pointlio] reject typed cloud: source timestamp must increase\n");
        return;
    }
    g_last_typed_cloud_stamp_ns = stamp_ns;
    g_has_last_typed_cloud_stamp = true;
    g_frame_start_ns = stamp_ns;
    g_frame_has_timestamp = true;
    g_accumulated_points = std::move(accepted_points);
}
static void on_typed_imu(const lcm::ReceiveBuffer*, const std::string&, const sensor_msgs::Imu* msg, void*) {
    if (!g_running.load() || !g_external_input || !msg || !g_point_lio) return;
    const auto& stamp = msg->header.stamp;
    const auto finite_array = [](const auto& values) {
        for (const auto value : values) {
            if (!std::isfinite(value)) return false;
        }
        return true;
    };
    if (msg->header.frame_id.empty() || msg->header.frame_id != g_sensor_frame_id ||
        stamp.sec < 0 || stamp.nsec < 0 || stamp.nsec >= static_cast<int32_t>(NANOS_PER_SECOND) ||
        !std::isfinite(msg->orientation.x) || !std::isfinite(msg->orientation.y) ||
        !std::isfinite(msg->orientation.z) || !std::isfinite(msg->orientation.w) ||
        !std::isfinite(msg->angular_velocity.x) || !std::isfinite(msg->angular_velocity.y) ||
        !std::isfinite(msg->angular_velocity.z) || !std::isfinite(msg->linear_acceleration.x) ||
        !std::isfinite(msg->linear_acceleration.y) || !std::isfinite(msg->linear_acceleration.z) ||
        !finite_array(msg->orientation_covariance) || !finite_array(msg->angular_velocity_covariance) ||
        !finite_array(msg->linear_acceleration_covariance)) {
        fprintf(stderr, "[pointlio] reject typed IMU: invalid frame, timestamp, or non-finite value\\n");
        return;
    }
    const uint64_t timestamp_ns = static_cast<uint64_t>(stamp.sec) * NANOS_PER_SECOND +
        static_cast<uint64_t>(stamp.nsec);
    {
        std::lock_guard<std::mutex> lock(g_lio_mutex);
        if (g_has_last_typed_imu_stamp && timestamp_ns <= g_last_typed_imu_stamp_ns) {
            fprintf(stderr, "[%s] reject typed IMU: source timestamp must increase\n", "pointlio");
            return;
        }
        if (g_has_last_typed_imu_stamp && nuwax_lio::imu_source_gap_exceeded(
                g_last_typed_imu_stamp_ns, timestamp_ns)) {
            // 缺测已破坏本代估计器连续性；锁定失败并交由原拥有方退出，不能推进水位后续算。
            g_typed_input_failed.store(true);
            g_running.store(false);
            fprintf(stderr, "[pointlio] native_input_discontinuity: IMU source gap=%llu ns exceeds %llu ns; restart owner required\n",
                static_cast<unsigned long long>(timestamp_ns - g_last_typed_imu_stamp_ns),
                static_cast<unsigned long long>(nuwax_lio::kMaxImuSourceGapNs));
            return;
        }
        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        imu_msg->header.seq = msg->header.seq;
        imu_msg->header.stamp.sec = msg->header.stamp.sec;
        imu_msg->header.stamp.nsecs = msg->header.stamp.nsec;
        imu_msg->orientation.x = msg->orientation.x; imu_msg->orientation.y = msg->orientation.y;
        imu_msg->orientation.z = msg->orientation.z; imu_msg->orientation.w = msg->orientation.w;
        const auto gyro = nuwax_lio::rotate_imu_to_body(g_body_from_sensor_rotation,
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        const auto accel = nuwax_lio::rotate_imu_to_body(g_body_from_sensor_rotation,
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        imu_msg->header.frame_id = g_child_frame_id;
        imu_msg->angular_velocity.x = gyro[0];
        imu_msg->angular_velocity.y = gyro[1]; imu_msg->angular_velocity.z = gyro[2];
        // 规范输入为米/秒²；PointLIO 内核按重力倍数执行缩放和饱和检查。
        // 使用内核自身的重力常量逆转换，避免再次乘重力并把静止输入判为饱和。
        imu_msg->linear_acceleration.x = accel[0] / G_m_s2;
        imu_msg->linear_acceleration.y = accel[1] / G_m_s2;
        imu_msg->linear_acceleration.z = accel[2] / G_m_s2;
        g_typed_imu_samples.push_back({timestamp_ns, std::move(imu_msg)});
        g_last_typed_imu_stamp_ns = timestamp_ns;
        g_has_last_typed_imu_stamp = true;
    }
}

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) { return; }

    uint64_t ts_ns = get_timestamp_ns(data);
    uint16_t dot_num = data->dot_num;

    // Per-point intra-packet offset (matches livox_ros_driver2). Without it all
    // points share one timestamp and per-point deskew is lost. time_interval
    // unit is 0.1us, so *100 → ns.
    const uint64_t point_interval_ns = dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_start_ns = ts_ns;
        g_frame_has_timestamp = true;
    }

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[point_idx].x) / 1000.0;   // mm → m
            cp.y = static_cast<double>(pts[point_idx].y) / 1000.0;
            cp.z = static_cast<double>(pts[point_idx].z) / 1000.0;
            cp.reflectivity = pts[point_idx].reflectivity;
            cp.tag = pts[point_idx].tag;
            cp.line = 0;  // Mid-360: single line
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + point_idx * point_interval_ns);
            g_accumulated_points.push_back(cp);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[point_idx].x) / 100.0;   // cm → m
            cp.y = static_cast<double>(pts[point_idx].y) / 100.0;
            cp.z = static_cast<double>(pts[point_idx].z) / 100.0;
            cp.reflectivity = pts[point_idx].reflectivity;
            cp.tag = pts[point_idx].tag;
            cp.line = 0;
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + point_idx * point_interval_ns);
            g_accumulated_points.push_back(cp);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_point_lio) { return; }

    uint64_t pkt_ts_ns = get_timestamp_ns(data);
    double ts = static_cast<double>(pkt_ts_ns) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    // Serialize EKF access against the main loop (run_main_iter). Held across the
    // whole packet so its samples feed atomically.
    std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
    for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
        imu_msg->header.seq = 0;
        imu_msg->header.frame_id = "livox_frame";

        imu_msg->orientation.x = 0.0;
        imu_msg->orientation.y = 0.0;
        imu_msg->orientation.z = 0.0;
        imu_msg->orientation.w = 1.0;
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->orientation_covariance[cov_idx] = 0.0;
        }

        imu_msg->angular_velocity.x = static_cast<double>(imu_pts[point_idx].gyro_x);
        imu_msg->angular_velocity.y = static_cast<double>(imu_pts[point_idx].gyro_y);
        imu_msg->angular_velocity.z = static_cast<double>(imu_pts[point_idx].gyro_z);
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->angular_velocity_covariance[cov_idx] = 0.0;
        }

        // Point-LIO expects accel in g (EKF does its own scaling). SDK already
        // reports g, so feed raw — scaling by GRAVITY_MS2 would double-scale and
        // trip the satu_acc check at rest.
        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[point_idx].acc_x);
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[point_idx].acc_y);
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[point_idx].acc_z);
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->linear_acceleration_covariance[cov_idx] = 0.0;
        }

        g_point_lio->feed_imu(imu_msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
    if (info == nullptr) { return; }

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    if (pointlio_debug) {
        printf("[pointlio] Device connected: handle=%u type=%u sn=%s ip=%s\n", handle, info->dev_type, sn, ip);
    }

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

int main(int argc, char** argv) {
    // 身份查询在通信和算法构造前结束；允许部署路径或文件名独立于后端名称。
    if (argc == 2 && std::strcmp(argv[1], "--nuwax-lio-contract") == 0) {
        std::puts("nuwax-native-lio-v1:pointlio");
        return 0;
    }
    dimos::NativeModule mod(argc, argv);

    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";

    if (g_lidar_topic.empty() && g_odometry_topic.empty()) {
        fprintf(stderr, "Error: at least one of --lidar or --odometry is required\n");
        return 1;
    }

    // Point-LIO tuning, passed as CLI args by the dimos module (no YAML).
    PointLioParams params;
    // common
    params.con_frame = mod.arg_bool("con_frame", params.con_frame);
    params.con_frame_num = mod.arg_int("con_frame_num", params.con_frame_num);
    params.cut_frame = mod.arg_bool("cut_frame", params.cut_frame);
    params.cut_frame_time_interval = mod.arg_float("cut_frame_time_interval", params.cut_frame_time_interval);
    params.time_lag_imu_to_lidar = mod.arg_float("time_lag_imu_to_lidar", params.time_lag_imu_to_lidar);
    // preprocess
    params.scan_line = mod.arg_int("scan_line", params.scan_line);
    params.scan_rate = mod.arg_int("scan_rate", params.scan_rate);
    params.blind = mod.arg_float("blind", params.blind);
    params.point_filter_num = mod.arg_int("point_filter_num", params.point_filter_num);
    std::string lidar_type = mod.arg("lidar_type", "avia");
    params.lidar_type = lidar_type == "velodyne" ? 2 : lidar_type == "ouster" ? 3 :
                        lidar_type == "hesai" ? 4 : lidar_type == "unilidar" ? 5 : 1;
    std::string ts_unit = mod.arg("timestamp_unit", "nanosecond");
    params.timestamp_unit = ts_unit == "second" ? 0 : ts_unit == "millisecond" ? 1 :
                            ts_unit == "microsecond" ? 2 : 3;
    // mapping
    params.use_imu_as_input = mod.arg_bool("use_imu_as_input", params.use_imu_as_input);
    params.prop_at_freq_of_imu = mod.arg_bool("prop_at_freq_of_imu", params.prop_at_freq_of_imu);
    params.check_satu = mod.arg_bool("check_satu", params.check_satu);
    params.init_map_size = mod.arg_int("init_map_size", params.init_map_size);
    params.space_down_sample = mod.arg_bool("space_down_sample", params.space_down_sample);
    params.satu_acc = mod.arg_float("satu_acc", params.satu_acc);
    params.satu_gyro = mod.arg_float("satu_gyro", params.satu_gyro);
    params.acc_norm = mod.arg_float("acc_norm", params.acc_norm);
    params.plane_thr = mod.arg_float("plane_thr", params.plane_thr);
    // 对齐锁定 Dimos 模块的默认值，避免过稀扫描配上过小地图邻域而丢失激光约束。
    int initial_map_frames = 1;
    try {
        if (mod.has("initial_map_frames")) {
            initial_map_frames = nuwax_lio::parse_initial_map_frames(mod.arg("initial_map_frames", ""));
        }
        params.filter_size_surf = mod.has("filter_size_surf")
            ? nuwax_lio::parse_native_voxel_size(mod.arg("filter_size_surf", ""), "filter_size_surf")
            : mod.arg_float("filter_size_surf", 0.2f);
        params.filter_size_map = mod.has("filter_size_map")
            ? nuwax_lio::parse_native_voxel_size(mod.arg("filter_size_map", ""), "filter_size_map")
            : mod.arg_float("filter_size_map", params.filter_size_map);
    } catch (const std::invalid_argument& error) {
        fprintf(stderr, "[pointlio] 配置参数无效：%s\n", error.what());
        return 2;
    }
    params.ivox_grid_resolution = mod.arg_float("ivox_grid_resolution", 2.0f);
    std::string ivox_nearby = mod.arg("ivox_nearby_type", "nearby6");
    params.ivox_nearby_type = ivox_nearby == "center" ? 0 : ivox_nearby == "nearby18" ? 18 :
                              ivox_nearby == "nearby26" ? 26 : 6;
    params.cube_side_length = mod.arg_float("cube_side_length", params.cube_side_length);
    params.det_range = mod.arg_float("det_range", params.det_range);
    params.fov_degree = mod.arg_float("fov_degree", params.fov_degree);
    params.imu_en = mod.arg_bool("imu_en", params.imu_en);
    params.start_in_aggressive_motion = mod.arg_bool("start_in_aggressive_motion", params.start_in_aggressive_motion);
    params.extrinsic_est_en = mod.arg_bool("extrinsic_est_en", params.extrinsic_est_en);
    params.imu_time_inte = mod.arg_float("imu_time_inte", params.imu_time_inte);
    params.lidar_meas_cov = mod.arg_float("lidar_meas_cov", params.lidar_meas_cov);
    params.acc_cov_input = mod.arg_float("acc_cov_input", params.acc_cov_input);
    params.vel_cov = mod.arg_float("vel_cov", params.vel_cov);
    params.gyr_cov_input = mod.arg_float("gyr_cov_input", params.gyr_cov_input);
    params.gyr_cov_output = mod.arg_float("gyr_cov_output", params.gyr_cov_output);
    params.acc_cov_output = mod.arg_float("acc_cov_output", params.acc_cov_output);
    params.b_gyr_cov = mod.arg_float("b_gyr_cov", params.b_gyr_cov);
    params.b_acc_cov = mod.arg_float("b_acc_cov", params.b_acc_cov);
    params.imu_meas_acc_cov = mod.arg_float("imu_meas_acc_cov", params.imu_meas_acc_cov);
    params.imu_meas_omg_cov = mod.arg_float("imu_meas_omg_cov", params.imu_meas_omg_cov);
    params.match_s = mod.arg_float("match_s", params.match_s);
    params.gravity_align = mod.arg_bool("gravity_align", params.gravity_align);
    if (auto g = parse_doubles(mod.arg("gravity", "")); !g.empty()) params.gravity = g;
    if (auto gi = parse_doubles(mod.arg("gravity_init", "")); !gi.empty()) params.gravity_init = gi;
    if (auto et = parse_doubles(mod.arg("extrinsic_t", "")); !et.empty()) params.extrinsic_T = et;
    if (auto er = parse_doubles(mod.arg("extrinsic_r", "")); !er.empty()) params.extrinsic_R = er;
    // odometry
    params.publish_odometry_without_downsample =
        mod.arg_bool("publish_odometry_without_downsample", params.publish_odometry_without_downsample);
    params.odom_only = mod.arg_bool("odom_only", params.odom_only);

    // Point-LIO internal processing rates
    double msr_freq = mod.arg_float("msr_freq", 50.0f);
    double main_freq = mod.arg_float("main_freq", 5000.0f);

    // Livox hardware config
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg_required("frame_id");
    g_child_frame_id = mod.arg_required("child_frame_id");
    g_sensor_frame_id = mod.arg_required("sensor_frame_id");
    g_input_lidar_topic = mod.arg("input_lidar", "");
    g_input_imu_topic = mod.arg("input_imu", "");
    g_external_input = !g_input_lidar_topic.empty() || !g_input_imu_topic.empty();
    if (g_external_input) {
        if (params.extrinsic_R.size() != g_body_from_sensor_rotation.size()) {
            fprintf(stderr, "Error: typed IMU requires a 3x3 body-from-sensor rotation\n");
            return 1;
        }
        std::copy(params.extrinsic_R.begin(), params.extrinsic_R.end(), g_body_from_sensor_rotation.begin());
    }
    if (g_external_input && (g_input_lidar_topic.empty() || g_input_imu_topic.empty())) {
        fprintf(stderr, "Error: --input_lidar and --input_imu must be supplied together\\n");
        return 1;
    }
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);

    // Propagates to the Point-LIO core via the `pointlio_debug` global.
    bool debug = mod.arg_bool("debug", false);
    pointlio_debug = debug;

    // SDK network ports (defaults from SdkPorts struct in livox_sdk_config.hpp)
    livox_common::SdkPorts ports;
    const livox_common::SdkPorts port_defaults;
    ports.cmd_data        = mod.arg_int("cmd_data_port", port_defaults.cmd_data);
    ports.push_msg        = mod.arg_int("push_msg_port", port_defaults.push_msg);
    ports.point_data      = mod.arg_int("point_data_port", port_defaults.point_data);
    ports.imu_data        = mod.arg_int("imu_data_port", port_defaults.imu_data);
    ports.log_data        = mod.arg_int("log_data_port", port_defaults.log_data);
    ports.host_cmd_data   = mod.arg_int("host_cmd_data_port", port_defaults.host_cmd_data);
    ports.host_push_msg   = mod.arg_int("host_push_msg_port", port_defaults.host_push_msg);
    ports.host_point_data = mod.arg_int("host_point_data_port", port_defaults.host_point_data);
    ports.host_imu_data   = mod.arg_int("host_imu_data_port", port_defaults.host_imu_data);
    ports.host_log_data   = mod.arg_int("host_log_data_port", port_defaults.host_log_data);

    if (debug) {
        printf("[pointlio] Starting Point-LIO + Livox Mid-360 native module\n");
        printf("[pointlio] lidar topic: %s\n", g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
        printf("[pointlio] odometry topic: %s\n", g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
        printf("[pointlio] tuning: filter_size_surf=%.3f ivox_res=%.3f lidar_type=%d\n", params.filter_size_surf, params.ivox_grid_resolution, params.lidar_type);
        printf("[pointlio] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n", host_ip.c_str(), lidar_ip.c_str(), g_frequency);
        printf("[pointlio] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n", pointcloud_freq, odom_freq);
    }

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    if (debug) { printf("[pointlio] Initializing Point-LIO...\n"); }
    PointLio point_lio(params, msr_freq, main_freq, 10.0, initial_map_frames);
    g_point_lio = &point_lio;
    if (debug) { printf("[pointlio] Point-LIO initialized.\n"); }

    // Main-loop state. Body lives in `run_main_iter`, driven by the wall-paced
    // main thread.
    auto frame_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / g_frequency));
    std::optional<std::chrono::steady_clock::time_point> last_emit;
    const double process_period_ms = 1000.0 / main_freq;

    auto pc_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / odom_freq));
    std::optional<std::chrono::steady_clock::time_point> last_pc_publish;
    std::optional<std::chrono::steady_clock::time_point> last_odom_publish;
    uint64_t last_output_source_timestamp_ns = 0;
    bool has_last_output_source_timestamp = false;


    auto run_main_iter = [&](std::chrono::steady_clock::time_point now) {
        // Lazy-seed the frame drain bookmark on the first iteration.
        if (!last_emit.has_value()) {
            last_emit = now;
        }

        // 外部规范点云已经是完整扫描，下一轮直接消费，避免被裸包聚合节拍覆盖。
        // SDK裸包仍按帧率聚合；检查与交换共用点云锁，保持帧和时间基准一致。
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            std::lock_guard<std::mutex> lock(g_pc_mutex);
            if (g_external_input || now - *last_emit >= frame_interval) {
                if (!g_accumulated_points.empty()) {
                    points.swap(g_accumulated_points);
                    frame_start = g_frame_start_ns;
                    g_frame_has_timestamp = false;
                }
                last_emit = now;
            }
        }
        const bool first_output = !has_last_output_source_timestamp;
        // 规范输入已经是一帧完整扫描；每个新解直接交付，不再经过另一层墙钟限频。
        // 原有源时间递增检查仍会拒绝空转及重复解，SDK裸包模式保留配置节拍。
        bool cloud_due = !g_lidar_topic.empty() &&
                         (g_external_input || first_output || !last_pc_publish.has_value() ||
                          now - *last_pc_publish >= pc_interval);
        const bool odometry_due = !g_odometry_topic.empty() &&
                                  (g_external_input || first_output || !last_odom_publish.has_value() ||
                                   now - *last_odom_publish >= odom_interval);
        nav_msgs::Odometry odometry;
        PointCloudXYZI::Ptr body_cloud;
        uint64_t source_timestamp_ns = 0;
        {
            // Snapshot estimator output under the EKF lock, then release it
            // before LCM serialization/publish can block the IMU callback.
            std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
            if (g_external_input && !points.empty()) {
                const uint64_t scan_end_ns = frame_start + static_cast<uint64_t>(points.back().offset_time);
                nuwax_lio::feed_scan_imu_and_retain_boundary(
                    g_typed_imu_samples, scan_end_ns, g_last_fed_typed_imu_stamp_ns,
                    [&point_lio](const auto& sample) { point_lio.feed_imu(sample); });
            }

            if (!points.empty()) {
                const size_t num_points = points.size();
                auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
                lidar_msg->header.seq = 0;
                lidar_msg->header.stamp = custom_messages::Time().fromSec( static_cast<double>(frame_start) / 1e9);
                lidar_msg->header.frame_id = "livox_frame";
                lidar_msg->timebase = frame_start;
                lidar_msg->lidar_id = 0;
                for (int idx = 0; idx < 3; idx++) { lidar_msg->rsvd[idx] = 0; }
                lidar_msg->point_num = static_cast<uli>(num_points);
                lidar_msg->points = std::move(points);
                if (pointlio_debug) {
                    fprintf(stderr, "[pointlio] feed_lidar frame: %zu points\n", num_points);
                }
                point_lio.feed_lidar(lidar_msg);
            }

            // A successful step updates the odometry header with the
            // estimator's Livox-derived solution time.
            point_lio.process();
            if (!cloud_due && !odometry_due) { return; }

            const auto& estimator_odometry = point_lio.get_odometry();
            if (!next_source_timestamp(
                    estimator_odometry,
                    last_output_source_timestamp_ns,
                    has_last_output_source_timestamp,
                    source_timestamp_ns)) {
                return;
            }
            odometry = make_odometry_message(estimator_odometry, source_timestamp_ns);
            if (cloud_due) {
                body_cloud = point_lio.get_body_cloud();
                cloud_due = body_cloud && !body_cloud->empty();
            }
        }
        if (!cloud_due && !odometry_due) { return; }

        last_output_source_timestamp_ns = source_timestamp_ns;
        has_last_output_source_timestamp = true;

        // Publish odometry first whenever a cloud is emitted so the two private
        // channels carry an exact, source-timestamped pair for bounded reordering.
        if (!g_odometry_topic.empty() && (odometry_due || cloud_due)) {
            publish_odometry(odometry);
            last_odom_publish = now;
            if (pointlio_debug) {
                const auto& position = odometry.pose.pose.position;
                fprintf(stderr, "[pointlio] publish odom: pose=(%.3f, %.3f, %.3f)\n", position.x, position.y, position.z);
            }
        }
        if (cloud_due) {
            publish_lidar(body_cloud, source_timestamp_ns);
            last_pc_publish = now;
            if (pointlio_debug) {
                const auto& position = odometry.pose.pose.position;
                fprintf(stderr, "[pointlio] publish lidar: %zu points  pose=(%.3f, %.3f, %.3f)\n", body_cloud->size(), position.x, position.y, position.z);
            }
        }
    };

    bool sdk_started = false;
    if (g_external_input) {
        auto* lidar_subscription = lcm.subscribeFunction(
            g_input_lidar_topic, on_typed_cloud, static_cast<void*>(nullptr));
        if (!nuwax_lio::configure_native_input_subscription(
                lidar_subscription, nuwax_lio::kNativeInputCloudQueueCapacity,
                "pointlio", g_input_lidar_topic)) return 1;
        auto* imu_subscription = lcm.subscribeFunction(
            g_input_imu_topic, on_typed_imu, static_cast<void*>(nullptr));
        if (!nuwax_lio::configure_native_input_subscription(
                imu_subscription, nuwax_lio::kNativeInputImuQueueCapacity,
                "pointlio", g_input_imu_topic)) return 1;
        if (debug) { printf("[pointlio] typed input: %s / %s\\n", g_input_lidar_topic.c_str(), g_input_imu_topic.c_str()); }
    } else {
        if (!livox_common::init_livox_sdk(host_ip, lidar_ip, ports, debug)) return 1;
        SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
        SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
        SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);
        if (!LivoxLidarSdkStart()) {
            fprintf(stderr, "Error: LivoxLidarSdkStart failed\\n");
            LivoxLidarSdkUninit();
            return 1;
        }
        sdk_started = true;
        if (debug) { printf("[pointlio] SDK started, waiting for device...\\n"); }
    }

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        if (g_external_input) {
            const int received = nuwax_lio::dispatch_native_input_until_scan(lcm, [] {
                std::lock_guard<std::mutex> lock(g_pc_mutex);
                return g_frame_has_timestamp;
            }, [] { return g_running.load(); });
            if (received < 0) {
                fprintf(stderr, "[pointlio] 规范输入通信读取失败\n");
                g_typed_input_failed.store(true);
                g_running.store(false);
            }
            if (!g_running.load()) break;
        }
        run_main_iter(std::chrono::steady_clock::now());

        if (!g_external_input) lcm.handleTimeout(0);

        // Rate control (~main_freq, 5kHz default).
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds( static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    if (debug) { printf("[pointlio] Shutting down...\n"); }
    if (sdk_started) LivoxLidarSdkUninit();
    g_point_lio = nullptr;
    g_lcm = nullptr;

    if (debug) { printf("[pointlio] Done.\n"); }
    return g_typed_input_failed.load() ? 2 : 0;
}
