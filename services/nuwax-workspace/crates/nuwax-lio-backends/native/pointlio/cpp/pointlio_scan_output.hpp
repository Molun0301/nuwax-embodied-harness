#pragma once

#include <algorithm>
#include <Eigen/Geometry>
#include <utility>
#include <vector>

namespace nuwax_lio {

// 逐点估计的扫描内轨迹只用于对外密集点云，不参与滤波或内核地图更新。
class PointLioScanOutput {
public:
    struct Pose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp = 0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Quaterniond body_from_sensor_rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d body_from_sensor_translation = Eigen::Vector3d::Zero();
    };

    void begin_scan() { trajectory_.clear(); }

    void record(const Pose& pose) {
        if (!trajectory_.empty() && trajectory_.back().timestamp == pose.timestamp) {
            trajectory_.back() = pose;
        } else {
            trajectory_.push_back(pose);
        }
    }

    bool finish_scan(const PointCloudXYZI::Ptr& raw, double scan_start,
                     const Eigen::Isometry3d& world_from_reference_sensor) {
        if (!raw || raw->empty() || trajectory_.empty()) return false;
        auto cloud = PointCloudXYZI::Ptr(new PointCloudXYZI(*raw));
        const Eigen::Isometry3d reference_from_world = world_from_reference_sensor.inverse();
        for (auto& point : cloud->points) {
            // curvature沿内核原约定为真实扫描内毫秒偏移；保留所有点及其他字段。
            const double timestamp = scan_start + static_cast<double>(point.curvature) / 1000.0;
            const Eigen::Vector3d value(point.x, point.y, point.z);
            const Eigen::Vector3d compensated = reference_from_world
                * sensor_pose_at(timestamp) * value;
            point.x = static_cast<float>(compensated.x());
            point.y = static_cast<float>(compensated.y());
            point.z = static_cast<float>(compensated.z());
        }
        sensor_cloud_ = std::move(cloud);
        world_from_reference_sensor_ = world_from_reference_sensor;
        return true;
    }

    PointCloudXYZI::Ptr sensor_cloud() const {
        return sensor_cloud_ ? PointCloudXYZI::Ptr(new PointCloudXYZI(*sensor_cloud_)) : nullptr;
    }

    PointCloudXYZI::Ptr world_cloud() const {
        auto cloud = sensor_cloud();
        if (!cloud) return nullptr;
        for (auto& point : cloud->points) {
            const Eigen::Vector3d world = world_from_reference_sensor_
                * Eigen::Vector3d(point.x, point.y, point.z);
            point.x = static_cast<float>(world.x());
            point.y = static_cast<float>(world.y());
            point.z = static_cast<float>(world.z());
        }
        return cloud;
    }

private:
    static Eigen::Isometry3d sensor_pose(const Pose& pose, double dt) {
        Eigen::Quaterniond rotation = pose.rotation;
        const double speed = pose.omega.norm();
        if (speed > 0 && dt != 0) {
            rotation = rotation * Eigen::Quaterniond(Eigen::AngleAxisd(
                speed * dt, pose.omega / speed));
        }
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = (rotation * pose.body_from_sensor_rotation).toRotationMatrix();
        result.translation() = pose.position + pose.velocity * dt
            + .5 * pose.acceleration * dt * dt
            + rotation * pose.body_from_sensor_translation;
        return result;
    }

    Eigen::Isometry3d sensor_pose_at(double timestamp) const {
        const auto upper = std::upper_bound(trajectory_.begin(), trajectory_.end(), timestamp,
            [](double value, const Pose& pose) { return value < pose.timestamp; });
        // 原始密集点可能早于首个或晚于末个体素代表点，只延拓该帧边缘的估计运动。
        if (upper == trajectory_.begin()) {
            return sensor_pose(*upper, timestamp - upper->timestamp);
        }
        const auto& lower = *(upper - 1);
        if (upper == trajectory_.end()) {
            return sensor_pose(lower, timestamp - lower.timestamp);
        }
        const double alpha = (timestamp - lower.timestamp) / (upper->timestamp - lower.timestamp);
        Pose pose;
        pose.position = (1 - alpha) * lower.position + alpha * upper->position;
        pose.rotation = lower.rotation.slerp(alpha, upper->rotation);
        pose.body_from_sensor_translation = (1 - alpha) * lower.body_from_sensor_translation
            + alpha * upper->body_from_sensor_translation;
        pose.body_from_sensor_rotation = lower.body_from_sensor_rotation.slerp(
            alpha, upper->body_from_sensor_rotation);
        return sensor_pose(pose, 0);
    }

    std::vector<Pose, Eigen::aligned_allocator<Pose>> trajectory_;
    PointCloudXYZI::Ptr sensor_cloud_;
    Eigen::Isometry3d world_from_reference_sensor_ = Eigen::Isometry3d::Identity();
};

}  // 命名空间nuwax_lio
