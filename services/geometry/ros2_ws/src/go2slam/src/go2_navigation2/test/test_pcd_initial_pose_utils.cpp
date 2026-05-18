#include <cmath>

#include <gtest/gtest.h>

#include "go2_navigation2/pcd_initial_pose_utils.hpp"

namespace {

TEST(PcdInitialPoseUtils, BuildsInitialPoseFromIcpTransform) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  const double yaw = 1.2;
  transform(0, 0) = static_cast<float>(std::cos(yaw));
  transform(0, 1) = static_cast<float>(-std::sin(yaw));
  transform(1, 0) = static_cast<float>(std::sin(yaw));
  transform(1, 1) = static_cast<float>(std::cos(yaw));
  transform(0, 3) = 2.5f;
  transform(1, 3) = -0.75f;
  transform(2, 3) = 0.42f;

  const auto pose = go2_navigation2::makeInitialPoseMessage(
      transform, "map", rclcpp::Time(123, 456), 0.04, 0.16);

  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_EQ(pose.header.stamp.sec, 123);
  EXPECT_EQ(pose.header.stamp.nanosec, 456u);
  EXPECT_NEAR(pose.pose.pose.position.x, 2.5, 1e-5);
  EXPECT_NEAR(pose.pose.pose.position.y, -0.75, 1e-5);
  EXPECT_NEAR(pose.pose.pose.position.z, 0.0, 1e-5);

  const auto &orientation = pose.pose.pose.orientation;
  const double recovered_yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  EXPECT_NEAR(recovered_yaw, yaw, 1e-5);
  EXPECT_NEAR(pose.pose.covariance[0], 0.04, 1e-9);
  EXPECT_NEAR(pose.pose.covariance[7], 0.04, 1e-9);
  EXPECT_NEAR(pose.pose.covariance[35], 0.16, 1e-9);
}

}  // namespace
