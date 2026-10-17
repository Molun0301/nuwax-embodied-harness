// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into FAST-LIO-NON-ROS: SDK callbacks feed
// CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM.  Sensor/body-frame
// point clouds and odometry are published on LCM (consumers register the cloud
// via the odometry pose).
//
// Usage:
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --acc_cov 1.0 --filter_size_surf 0.1 ... \   # tuning as plain CLI args
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
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"
#include "imu_frame_transform.hpp"
#include "scan_imu_buffer.hpp"
#include "native_input_subscription.hpp"
#include "native_lio_arguments.hpp"

#include "dimos_native_module.hpp"

// dimos LCM message headers
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// FAST-LIO (header-only core, compiled sources linked via CMake)
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_IMU;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

// Global state

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_typed_input_failed{false};
static lcm::LCM* g_lcm = nullptr;
static FastLio* g_fastlio = nullptr;

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_frame_id;  // required via --frame_id
static std::string g_child_frame_id;  // required via --child_frame_id
static std::string g_sensor_frame_id;  // required via --sensor_frame_id
static std::string g_input_lidar_topic;
static std::string g_input_imu_topic;
static bool g_external_input = false;
static std::array<double, 9> g_body_from_sensor_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
static float g_frequency = 10.0f;

// Frame accumulator (Livox SDK raw → CustomMsg)
static std::mutex g_pc_mutex;
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

// Helpers

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

// Publish a lidar point cloud, stamped with `frame_id`.

static void publish_lidar(PointCloudXYZI::Ptr cloud, uint64_t timestamp_ns, const std::string& frame_id, const std::string& topic = "") {
    const std::string& chan = topic.empty() ? g_lidar_topic : topic;
    if (!g_lcm || !cloud || cloud->empty() || chan.empty()) { return; }

    int num_points = static_cast<int>(cloud->size());

    sensor_msgs::PointCloud2 pc;
    pc.header = make_source_header(frame_id, timestamp_ns);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    // Fields: x, y, z, intensity (float32 each)
    pc.fields_length = 4;
    pc.fields.resize(4);

    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        return f;
    };

    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);

    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;

    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int i = 0; i < num_points; ++i) {
        const size_t byte_offset = static_cast<size_t>(i) * 16;
        const float x = cloud->points[i].x;
        const float y = cloud->points[i].y;
        const float z = cloud->points[i].z;
        const float intensity = cloud->points[i].intensity;
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

    msg.pose.pose.position.x = odom.pose.pose.position.x;
    msg.pose.pose.position.y = odom.pose.pose.position.y;
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
    msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
    msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;

    // Covariance (fixed-size double[36])
    for (int i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = odom.pose.covariance[i];
    }

    // Twist (zero — FAST-LIO doesn't output velocity directly)
    msg.twist.twist.linear.x = 0;
    msg.twist.twist.linear.y = 0;
    msg.twist.twist.linear.z = 0;
    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = 0;
    std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

    return msg;
}

static void publish_odometry(const nav_msgs::Odometry& msg) {
    if (!g_lcm) { return; }
    g_lcm->publish(g_odometry_topic, &msg);
}


// Canonical typed sensor ingress. Supplying both input channels disables all
// Livox callbacks; outputs remain the private --lidar/--odometry channels.
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
        fprintf(stderr, "[fastlio2] reject typed cloud: invalid frame, timestamp, or endianness\\n");
        return;
    }
    if (msg->width <= 0 || msg->height <= 0 || msg->point_step < 12 || msg->row_step < 0) {
        fprintf(stderr, "[fastlio2] reject typed cloud: invalid dimensions\\n");
        return;
    }
    const auto* x_field = find_typed_field(msg, "x");
    const auto* y_field = find_typed_field(msg, "y");
    const auto* z_field = find_typed_field(msg, "z");
    const auto* time_field = find_typed_field(msg, "offset_ns");
    const auto* self_field = find_typed_field(msg, "nuwax_self_mask");
    if (self_field && (!typed_field_fits(self_field, msg->point_step, 1) || self_field->datatype != 2)) {
        fprintf(stderr, "[fastlio2] reject typed cloud: invalid self mask field\n");
        return;
    }
    if (!typed_field_fits(x_field, msg->point_step, 4) || x_field->datatype != 7 ||
        !typed_field_fits(y_field, msg->point_step, 4) || y_field->datatype != 7 ||
        !typed_field_fits(z_field, msg->point_step, 4) || z_field->datatype != 7 ||
        !typed_field_fits(time_field, msg->point_step, 4) || time_field->datatype != 6) {
        fprintf(stderr, "[fastlio2] reject typed cloud: require float32 xyz and uint32 offset_ns\\n");
        return;
    }
    const size_t width = static_cast<size_t>(msg->width);
    const size_t height = static_cast<size_t>(msg->height);
    const size_t point_step = static_cast<size_t>(msg->point_step);
    const size_t row_step = static_cast<size_t>(msg->row_step);
    if (width > SIZE_MAX / point_step || row_step < width * point_step ||
        height > SIZE_MAX / row_step || msg->data.size() < height * row_step ||
        height > SIZE_MAX / width || width * height > 2'000'000) {
        fprintf(stderr, "[fastlio2] reject typed cloud: row/data bounds invalid\\n");
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
                fprintf(stderr, "[fastlio2] reject typed cloud: non-finite or malformed point\\n");
                return;
            }
            max_offset_ns = std::max(max_offset_ns, offset_ns);
            custom_messages::CustomPoint point;
            point.x = x; point.y = y; point.z = z;
            // 自体仅沿上游无效回波语义排除匹配，原坐标、采样数和末端时间保持。
            const uint8_t self_mask = self_field ? static_cast<uint8_t>(msg->data[base + self_field->offset]) : 0;
            if (self_mask > 1) {
                fprintf(stderr, "[fastlio2] reject typed cloud: invalid self mask value\n");
                return;
            }
            point.reflectivity = 0; point.tag = self_mask ? 0x20 : 0; point.line = 0;
            point.offset_time = static_cast<uli>(offset_ns);
            accepted_points.push_back(point);
        }
    }
    if (stamp_ns > UINT64_MAX - max_offset_ns ||
        !typed_imu_covers_scan(stamp_ns, stamp_ns + max_offset_ns)) {
        fprintf(stderr, "[%s] reject typed cloud: IMU does not cover scan window\n", "fastlio2");
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
        fprintf(stderr, "[%s] reject typed cloud: source timestamp must increase\n", "fastlio2");
        return;
    }
    g_last_typed_cloud_stamp_ns = stamp_ns;
    g_has_last_typed_cloud_stamp = true;
    g_frame_start_ns = stamp_ns;
    g_frame_has_timestamp = true;
    g_accumulated_points = std::move(accepted_points);
}

static void on_typed_imu(const lcm::ReceiveBuffer*, const std::string&, const sensor_msgs::Imu* msg, void*) {
    if (!g_running.load() || !g_external_input || !msg || !g_fastlio) return;
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
        fprintf(stderr, "[fastlio2] reject typed IMU: invalid frame, timestamp, or non-finite value\\n");
        return;
    }
    const uint64_t timestamp_ns = static_cast<uint64_t>(stamp.sec) * NANOS_PER_SECOND +
        static_cast<uint64_t>(stamp.nsec);
    {
        std::lock_guard<std::mutex> lock(g_lio_mutex);
        if (g_has_last_typed_imu_stamp && timestamp_ns <= g_last_typed_imu_stamp_ns) {
            fprintf(stderr, "[%s] reject typed IMU: source timestamp must increase\n", "fastlio2");
            return;
        }
        if (g_has_last_typed_imu_stamp && nuwax_lio::imu_source_gap_exceeded(
                g_last_typed_imu_stamp_ns, timestamp_ns)) {
            // 缺测已破坏本代估计器连续性；锁定失败并交由原拥有方退出，不能推进水位后续算。
            g_typed_input_failed.store(true);
            g_running.store(false);
            fprintf(stderr, "[fastlio2] native_input_discontinuity: IMU source gap=%llu ns exceeds %llu ns; restart owner required\n",
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
        imu_msg->linear_acceleration.x = accel[0];
        imu_msg->linear_acceleration.y = accel[1]; imu_msg->linear_acceleration.z = accel[2];
        g_typed_imu_samples.push_back({timestamp_ns, std::move(imu_msg)});
        g_last_typed_imu_stamp_ns = timestamp_ns;
        g_has_last_typed_imu_stamp = true;
    }
}

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) { return; }

    uint64_t ts_ns = get_timestamp_ns(data);
    uint16_t dot_num = data->dot_num;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_start_ns = ts_ns;
        g_frame_has_timestamp = true;
    }

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[i].x) / 1000.0;  // mm → m
            cp.y = static_cast<double>(pts[i].y) / 1000.0;
            cp.z = static_cast<double>(pts[i].z) / 1000.0;
            cp.reflectivity = pts[i].reflectivity;
            cp.tag = pts[i].tag;
            cp.line = 0;  // Mid-360: non-repetitive, single "line"
            cp.offset_time = static_cast<uli>(ts_ns - g_frame_start_ns);
            g_accumulated_points.push_back(cp);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[i].x) / 100.0;  // cm → m
            cp.y = static_cast<double>(pts[i].y) / 100.0;
            cp.z = static_cast<double>(pts[i].z) / 100.0;
            cp.reflectivity = pts[i].reflectivity;
            cp.tag = pts[i].tag;
            cp.line = 0;
            cp.offset_time = static_cast<uli>(ts_ns - g_frame_start_ns);
            g_accumulated_points.push_back(cp);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_fastlio) { return; }

    uint64_t pkt_ts_ns = get_timestamp_ns(data);
    double ts = static_cast<double>(pkt_ts_ns) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
    for (uint16_t i = 0; i < dot_num; ++i) {
        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
        imu_msg->header.seq = 0;
        imu_msg->header.frame_id = "livox_frame";

        imu_msg->orientation.x = 0.0;
        imu_msg->orientation.y = 0.0;
        imu_msg->orientation.z = 0.0;
        imu_msg->orientation.w = 1.0;
        for (int j = 0; j < 9; ++j) { imu_msg->orientation_covariance[j] = 0.0; }

        imu_msg->angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
        imu_msg->angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
        imu_msg->angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);
        for (int j = 0; j < 9; ++j) { imu_msg->angular_velocity_covariance[j] = 0.0; }

        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;
        for (int j = 0; j < 9; ++j) { imu_msg->linear_acceleration_covariance[j] = 0.0; }

        g_fastlio->feed_imu(imu_msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
    if (info == nullptr) { return; }

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    if (fastlio_debug) {
        printf("[fastlio2] Device connected: handle=%u type=%u sn=%s ip=%s\n", handle, info->dev_type, sn, ip);
    }

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

// Signal handling

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// Main

// One iteration of the main loop: drain accumulated points into a CustomMsg,
// run a FAST-LIO step, and publish results (rate-limited by the bookmarks).
static void run_main_iter(
    std::chrono::steady_clock::time_point now,
    FastLio& fast_lio,
    std::chrono::steady_clock::time_point& last_emit,
    std::chrono::steady_clock::time_point& last_pc_publish,
    std::chrono::steady_clock::time_point& last_odom_publish,
    std::chrono::microseconds frame_interval,
    std::chrono::microseconds pc_interval,
    std::chrono::microseconds odom_interval,
    uint64_t& last_output_source_timestamp_ns,
    bool& has_last_output_source_timestamp,
    bool scan_publish_en,
    bool dense_publish_en
) {
    // 外部规范点云已经是完整扫描，下一轮直接消费，避免被裸包聚合节拍覆盖。
    // SDK裸包仍按帧率聚合；检查与交换共用点云锁，保持帧和时间基准一致。
    std::vector<custom_messages::CustomPoint> points;
    uint64_t frame_start = 0;
    {
        std::lock_guard<std::mutex> lock(g_pc_mutex);
        if (g_external_input || now - last_emit >= frame_interval) {
            if (!g_accumulated_points.empty()) {
                points.swap(g_accumulated_points);
                frame_start = g_frame_start_ns;
                g_frame_has_timestamp = false;
            }
            last_emit = now;
        }
    }
    const bool first_output = !has_last_output_source_timestamp;
    // 完整规范扫描按真实新解发布；仅SDK裸包模式继续使用配置的输出节拍。
    bool cloud_due = scan_publish_en && !g_lidar_topic.empty() &&
                     (g_external_input || first_output || now - last_pc_publish >= pc_interval);
    const bool odometry_due = !g_odometry_topic.empty() &&
                              (g_external_input || first_output || now - last_odom_publish >= odom_interval);
    nav_msgs::Odometry odometry;
    PointCloudXYZI::Ptr cloud;
    uint64_t source_timestamp_ns = 0;
    {
        // SDK IMU callbacks and the main loop share the estimator. Snapshot
        // odometry and cloud under one lock, then release it before LCM I/O.
        std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
        if (g_external_input && !points.empty()) {
            const uint64_t scan_end_ns = frame_start + static_cast<uint64_t>(points.back().offset_time);
            nuwax_lio::feed_scan_imu_and_retain_boundary(
                g_typed_imu_samples, scan_end_ns, g_last_fed_typed_imu_stamp_ns,
                [&fast_lio](const auto& sample) { fast_lio.feed_imu(sample); });
        }

        if (!points.empty()) {
            auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
            lidar_msg->header.seq = 0;
            // 原始整数源时间不能先经绝对浮点秒舍入，末端同步仍需精确起点。
            lidar_msg->header.stamp.sec = frame_start / NANOS_PER_SECOND;
            lidar_msg->header.stamp.nsecs = frame_start % NANOS_PER_SECOND;
            lidar_msg->header.frame_id = "livox_frame";
            lidar_msg->timebase = frame_start;
            lidar_msg->lidar_id = 0;
            for (int i = 0; i < 3; i++) { lidar_msg->rsvd[i] = 0; }
            lidar_msg->point_num = static_cast<uli>(points.size());
            lidar_msg->points = std::move(points);
            fast_lio.feed_lidar(lidar_msg);
        }

        // Cheap when the input queues are empty; a successful step updates the
        // odometry header with the estimator's Livox-derived solution time.
        fast_lio.process();
        if (!cloud_due && !odometry_due) { return; }

        const auto& estimator_odometry = fast_lio.get_odometry();
        if (!next_source_timestamp(
                estimator_odometry,
                last_output_source_timestamp_ns,
                has_last_output_source_timestamp,
                source_timestamp_ns)) {
            return;
        }
        odometry = make_odometry_message(estimator_odometry, source_timestamp_ns);
        if (cloud_due) {
            // Sensor-frame cloud; register downstream via the odom pose.
            // dense_publish_en false selects the IESKF-downsampled scan.
            cloud = dense_publish_en ? fast_lio.get_body_cloud() : fast_lio.get_body_cloud_down();
            cloud_due = cloud && !cloud->empty();
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
    }
    if (cloud_due) {
        publish_lidar(cloud, source_timestamp_ns, g_sensor_frame_id);
        last_pc_publish = now;
    }
}

int main(int argc, char** argv) {
    // 身份查询在通信和算法构造前结束；允许部署路径或文件名独立于后端名称。
    if (argc == 2 && std::strcmp(argv[1], "--nuwax-lio-contract") == 0) {
        std::puts("nuwax-native-lio-v1:fastlio2");
        return 0;
    }
    dimos::NativeModule mod(argc, argv);

    // Required: LCM topics for output ports
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";

    if (g_lidar_topic.empty() && g_odometry_topic.empty()) {
        fprintf(stderr, "Error: at least one of --lidar or --odometry is required\n");
        return 1;
    }

    // FAST-LIO tuning, passed as CLI args by the dimos module (no YAML).
    FastLioParams params;
    params.acc_cov = mod.arg_float("acc_cov", params.acc_cov);
    params.gyr_cov = mod.arg_float("gyr_cov", params.gyr_cov);
    params.b_acc_cov = mod.arg_float("b_acc_cov", params.b_acc_cov);
    params.b_gyr_cov = mod.arg_float("b_gyr_cov", params.b_gyr_cov);
    params.det_range = mod.arg_float("det_range", params.det_range);
    params.cube_side_length = mod.arg_float("cube_side_length", params.cube_side_length);
    params.blind = mod.arg_float("blind", params.blind);
    params.time_offset_lidar_to_imu = mod.arg_float("time_offset_lidar_to_imu", params.time_offset_lidar_to_imu);
    params.fov_degree = mod.arg_int("fov_degree", params.fov_degree);
    params.scan_line = mod.arg_int("scan_line", params.scan_line);
    params.scan_rate = mod.arg_int("scan_rate", params.scan_rate);
    params.time_sync_en = mod.arg_bool("time_sync_en", params.time_sync_en);
    params.extrinsic_est_en = mod.arg_bool("extrinsic_est_en", params.extrinsic_est_en);
    std::string lidar_type = mod.arg("lidar_type", "livox");
    params.lidar_type = lidar_type == "velodyne" ? 2 : lidar_type == "ouster" ? 3 : 1;
    std::string ts_unit = mod.arg("timestamp_unit", "microsecond");
    params.timestamp_unit = ts_unit == "second" ? 0 : ts_unit == "millisecond" ? 1 : ts_unit == "nanosecond" ? 3 : 2;
    if (auto et = parse_doubles(mod.arg("extrinsic_t", "")); !et.empty()) { params.extrinsic_T = et; }
    if (auto er = parse_doubles(mod.arg("extrinsic_r", "")); !er.empty()) { params.extrinsic_R = er; }
    try {
        // 新增配置只接纳完整、有效的原生值；缺省继续保留原 arg_float 的单精度舍入。
        params.filter_size_surf = mod.has("filter_size_surf")
            ? nuwax_lio::parse_native_voxel_size(mod.arg("filter_size_surf", ""), "filter_size_surf")
            : mod.arg_float("filter_size_surf", params.filter_size_surf);
        params.filter_size_map = mod.has("filter_size_map")
            ? nuwax_lio::parse_native_voxel_size(mod.arg("filter_size_map", ""), "filter_size_map")
            : mod.arg_float("filter_size_map", params.filter_size_map);
        if (mod.has("initial_map_frames")) {
            params.initial_map_frames = nuwax_lio::parse_initial_map_frames(mod.arg("initial_map_frames", ""));
        }
        params.validate_sampling_parameters();
        params.validate_local_map_geometry();
    } catch (const std::invalid_argument& error) {
        fprintf(stderr, "[fastlio2] %s\n", error.what());
        return 2;
    }

    // FAST-LIO internal processing rates
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

    // Cloud-publish behaviour: scan_publish_en gates the lidar output;
    // dense_publish_en false voxel-downsamples the published cloud.
    bool scan_publish_en = mod.arg_bool("scan_publish_en", true);
    bool dense_publish_en = mod.arg_bool("dense_publish_en", true);

    // Verbose logging — propagates to the FAST-LIO C++ core via the
    // `fastlio_debug` global. Default false → only real errors print.
    bool debug = mod.arg_bool("debug", false);
    fastlio_debug = debug;

    // SDK network ports (defaults from SdkPorts struct in livox_sdk_config.hpp)
    livox_common::SdkPorts ports;
    const livox_common::SdkPorts port_defaults;
    ports.cmd_data = mod.arg_int("cmd_data_port", port_defaults.cmd_data);
    ports.push_msg = mod.arg_int("push_msg_port", port_defaults.push_msg);
    ports.point_data = mod.arg_int("point_data_port", port_defaults.point_data);
    ports.imu_data = mod.arg_int("imu_data_port", port_defaults.imu_data);
    ports.log_data = mod.arg_int("log_data_port", port_defaults.log_data);
    ports.host_cmd_data = mod.arg_int("host_cmd_data_port", port_defaults.host_cmd_data);
    ports.host_push_msg = mod.arg_int("host_push_msg_port", port_defaults.host_push_msg);
    ports.host_point_data = mod.arg_int("host_point_data_port", port_defaults.host_point_data);
    ports.host_imu_data = mod.arg_int("host_imu_data_port", port_defaults.host_imu_data);
    ports.host_log_data = mod.arg_int("host_log_data_port", port_defaults.host_log_data);

    if (debug) {
        printf("[fastlio2] Starting FAST-LIO2 + Livox Mid-360 native module\n");
        printf("[fastlio2] lidar topic: %s\n", g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
        printf("[fastlio2] odometry topic: %s\n", g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
        printf("[fastlio2] acc_cov: %.3f  filter_size_surf: %.3f\n", params.acc_cov, params.filter_size_surf);
        printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n", host_ip.c_str(), lidar_ip.c_str(), g_frequency);
        printf("[fastlio2] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n", pointcloud_freq, odom_freq);
    }

    // Signal handlers
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // Init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // Init FAST-LIO with config
    if (debug) { printf("[fastlio2] Initializing FAST-LIO...\n"); }
    FastLio fast_lio(params, msr_freq, main_freq);
    g_fastlio = &fast_lio;
    if (debug) { printf("[fastlio2] FAST-LIO initialized.\n"); }

    // Main-loop rate-limit state (consumed by the loop below).
    auto frame_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / g_frequency));
    const double process_period_ms = 1000.0 / main_freq;

    auto pc_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / odom_freq));

    // Rate-limit bookmarks, seeded to now so they don't all fire on iteration 1.
    auto last_emit = std::chrono::steady_clock::now();
    auto last_pc_publish = last_emit;
    auto last_odom_publish = last_emit;
    uint64_t last_output_source_timestamp_ns = 0;
    bool has_last_output_source_timestamp = false;

    bool sdk_started = false;
    if (g_external_input) {
        auto* lidar_subscription = lcm.subscribeFunction(
            g_input_lidar_topic, on_typed_cloud, static_cast<void*>(nullptr));
        if (!nuwax_lio::configure_native_input_subscription(
                lidar_subscription, nuwax_lio::kNativeInputCloudQueueCapacity,
                "fastlio2", g_input_lidar_topic)) return 1;
        auto* imu_subscription = lcm.subscribeFunction(
            g_input_imu_topic, on_typed_imu, static_cast<void*>(nullptr));
        if (!nuwax_lio::configure_native_input_subscription(
                imu_subscription, nuwax_lio::kNativeInputImuQueueCapacity,
                "fastlio2", g_input_imu_topic)) return 1;
        if (debug) { printf("[fastlio2] typed input: %s / %s\\n", g_input_lidar_topic.c_str(), g_input_imu_topic.c_str()); }
    } else {
        // Direct Livox mode is retained for device deployments.
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
        if (debug) { printf("[fastlio2] SDK started, waiting for device...\\n"); }
    }

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        if (g_external_input) {
            const int received = nuwax_lio::dispatch_native_input_until_scan(lcm, [] {
                std::lock_guard<std::mutex> lock(g_pc_mutex);
                return g_frame_has_timestamp;
            }, [] { return g_running.load(); });
            if (received < 0) {
                fprintf(stderr, "[fastlio2] 规范输入通信读取失败\n");
                g_typed_input_failed.store(true);
                g_running.store(false);
            }
            if (!g_running.load()) break;
        }
        auto now = std::chrono::steady_clock::now();
        run_main_iter(
            now,
            fast_lio,
            last_emit,
            last_pc_publish,
            last_odom_publish,
            frame_interval,
            pc_interval,
            odom_interval,
            last_output_source_timestamp_ns,
            has_last_output_source_timestamp,
            scan_publish_en,
            dense_publish_en
        );

        if (!g_external_input) lcm.handleTimeout(0);

        // Rate control (~main_freq, 5kHz default).
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    // Cleanup. Uninit the SDK (stops + joins its callback threads) BEFORE
    // clearing the globals the callbacks read, so a late on_point/on_imu can't
    // race the assignment and dereference a null g_fastlio / g_lcm.
    if (debug) { printf("[fastlio2] Shutting down...\n"); }
    if (sdk_started) LivoxLidarSdkUninit();
    g_fastlio = nullptr;
    g_lcm = nullptr;

    if (debug) { printf("[fastlio2] Done.\n"); }
    return g_typed_input_failed.load() ? 2 : 0;
}
