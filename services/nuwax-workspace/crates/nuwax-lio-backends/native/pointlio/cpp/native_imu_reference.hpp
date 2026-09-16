#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nuwax_lio {

// 算法状态位于惯性测量点，轴向保持与机体平行；发布时转换回机体原点。
class NativeImuReference {
public:
    void configure(const std::string& origin_in_body) {
        std::istringstream input(origin_in_body);
        for (int axis = 0; axis < 3; ++axis) {
            double value;
            if (!(input >> value) || !std::isfinite(value)) {
                throw std::invalid_argument("imu_origin_in_body 必须包含三个有限米值");
            }
            origin_in_body_[axis] = value;
            if (axis < 2) {
                char separator;
                if (!(input >> separator) || separator != ',') {
                    throw std::invalid_argument("imu_origin_in_body 必须使用逗号分隔三个数值");
                }
            }
        }
        input >> std::ws;
        if (!input.eof()) {
            throw std::invalid_argument("imu_origin_in_body 不允许尾随内容");
        }
    }

    std::vector<double> lidar_translation_at_imu(const std::vector<double>& body_from_sensor) const {
        if (body_from_sensor.size() != 3) {
            throw std::invalid_argument("extrinsic_t 必须包含三个数值");
        }
        std::vector<double> result(3);
        for (int axis = 0; axis < 3; ++axis) {
            result[axis] = body_from_sensor[axis] - origin_in_body_[axis];
            if (!std::isfinite(result[axis])) {
                throw std::invalid_argument("雷达到惯性原点的平移必须有限");
            }
        }
        return result;
    }

    template <typename Pose>
    void shift_pose_to_body(Pose& pose) const {
        const auto& q = pose.orientation;
        const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        const Eigen::Vector3d lever_world = rotation * origin_in_body_;
        pose.position.x -= lever_world.x();
        pose.position.y -= lever_world.y();
        pose.position.z -= lever_world.z();
    }

    template <typename Pose, typename Twist>
    void shift_world_velocity_to_body(const Pose& pose, Twist& twist) const {
        const auto& q = pose.orientation;
        const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        const Eigen::Vector3d omega_body(twist.angular.x, twist.angular.y, twist.angular.z);
        const Eigen::Vector3d lever_velocity = rotation * omega_body.cross(origin_in_body_);
        twist.linear.x -= lever_velocity.x();
        twist.linear.y -= lever_velocity.y();
        twist.linear.z -= lever_velocity.z();
    }


private:
    Eigen::Vector3d origin_in_body_ = Eigen::Vector3d::Zero();
};

}  // 命名空间 nuwax_lio
