#ifndef GO2_NAVIGATION2__PCD_INITIAL_POSE_UTILS_HPP_
#define GO2_NAVIGATION2__PCD_INITIAL_POSE_UTILS_HPP_

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace go2_navigation2 {

inline geometry_msgs::msg::PoseWithCovarianceStamped makeInitialPoseMessage(
    const Eigen::Matrix4f &map_from_base,
    const std::string &map_frame,
    const rclcpp::Time &stamp,
    const double xy_covariance,
    const double yaw_covariance) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = map_frame;
  pose.header.stamp = stamp;
  pose.pose.pose.position.x = static_cast<double>(map_from_base(0, 3));
  pose.pose.pose.position.y = static_cast<double>(map_from_base(1, 3));
  pose.pose.pose.position.z = 0.0;

  const double yaw = std::atan2(
      static_cast<double>(map_from_base(1, 0)),
      static_cast<double>(map_from_base(0, 0)));
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  pose.pose.pose.orientation = tf2::toMsg(quaternion);

  pose.pose.covariance[0] = xy_covariance;
  pose.pose.covariance[7] = xy_covariance;
  pose.pose.covariance[35] = yaw_covariance;
  return pose;
}

}  // namespace go2_navigation2

#endif  // GO2_NAVIGATION2__PCD_INITIAL_POSE_UTILS_HPP_
