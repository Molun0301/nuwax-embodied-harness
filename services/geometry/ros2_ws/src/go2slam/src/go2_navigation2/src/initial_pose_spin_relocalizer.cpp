#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class InitialPoseSpinRelocalizer : public rclcpp::Node
{
public:
  InitialPoseSpinRelocalizer()
  : Node("initial_pose_spin_relocalizer")
  {
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    rotations_ = declare_parameter<double>("rotations", 3.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    if (rotations_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "rotations <= 0; spin relocalizer is disabled.");
      return;
    }

    if (std::abs(angular_speed_) < 0.05) {
      RCLCPP_WARN(get_logger(), "angular_speed is too small; using 0.5 rad/s.");
      angular_speed_ = 0.5;
    }

    spin_duration_sec_ = rotations_ * 2.0 * kPi / std::abs(angular_speed_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&InitialPoseSpinRelocalizer::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Initial-pose spin relocalizer ready. topic=%s cmd_vel=%s rotations=%.2f speed=%.2f rad/s",
      initial_pose_topic_.c_str(), cmd_vel_topic_.c_str(), rotations_, angular_speed_);
  }

private:
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
  {
    if (triggered_) {
      return;
    }

    triggered_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Initial pose received; starting %.2f in-place rotations after %.2f s.",
      rotations_, start_delay_sec_);

    start_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.0, start_delay_sec_))),
      [this]() {
        if (start_timer_) {
          start_timer_->cancel();
        }
        startSpin();
      });
  }

  void startSpin()
  {
    spin_start_time_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)));
    spin_timer_ = create_wall_timer(period, std::bind(&InitialPoseSpinRelocalizer::publishSpin, this));
    publishSpin();
  }

  void publishSpin()
  {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - spin_start_time_).count();
    if (elapsed >= spin_duration_sec_) {
      publishStop();
      if (spin_timer_) {
        spin_timer_->cancel();
      }
      RCLCPP_INFO(get_logger(), "Initial-pose spin relocalization completed.");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = angular_speed_;
    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(stop);
    }
  }

  std::string initial_pose_topic_;
  std::string cmd_vel_topic_;
  double rotations_{3.0};
  double angular_speed_{0.5};
  double start_delay_sec_{1.0};
  double publish_rate_hz_{20.0};
  double spin_duration_sec_{0.0};
  bool triggered_{false};
  std::chrono::steady_clock::time_point spin_start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPoseSpinRelocalizer>());
  rclcpp::shutdown();
  return 0;
}
