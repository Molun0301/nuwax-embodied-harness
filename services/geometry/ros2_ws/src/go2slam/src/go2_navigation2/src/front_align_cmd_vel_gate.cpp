#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace
{
double clampMagnitude(double value, double min_abs, double max_abs)
{
  const double magnitude = std::clamp(std::abs(value), min_abs, max_abs);
  return std::copysign(magnitude, value);
}
}  // namespace

class FrontAlignCmdVelGate : public rclcpp::Node
{
public:
  FrontAlignCmdVelGate()
  : Node("front_align_cmd_vel_gate"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(*tf_buffer_)
  {
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.6);
    align_yaw_tolerance_ = declare_parameter<double>("align_yaw_tolerance", 0.45);
    min_linear_speed_to_gate_ = declare_parameter<double>("min_linear_speed_to_gate", 0.03);
    angular_kp_ = declare_parameter<double>("angular_kp", 1.6);
    min_align_angular_speed_ = declare_parameter<double>("min_align_angular_speed", 0.35);
    max_align_angular_speed_ = declare_parameter<double>("max_align_angular_speed", 1.0);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 2.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_plan_ = msg;
        latest_plan_time_ = now();
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10,
      std::bind(&FrontAlignCmdVelGate::cmdVelCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double linear_speed = std::hypot(msg->linear.x, msg->linear.y);
    if (linear_speed < min_linear_speed_to_gate_) {
      cmd_pub_->publish(*msg);
      return;
    }

    double target_angle = 0.0;
    if (!getFrontTargetAngle(&target_angle)) {
      cmd_pub_->publish(*msg);
      return;
    }

    if (std::abs(target_angle) <= align_yaw_tolerance_) {
      cmd_pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::Twist aligned_cmd;
    aligned_cmd.angular.z = clampMagnitude(
      angular_kp_ * target_angle, min_align_angular_speed_, max_align_angular_speed_);
    cmd_pub_->publish(aligned_cmd);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Holding linear cmd_vel until front target is aligned: yaw_error=%.3f rad",
      target_angle);
  }

  bool getFrontTargetAngle(double * target_angle)
  {
    if (!latest_plan_ || latest_plan_->poses.empty()) {
      return false;
    }

    if ((now() - latest_plan_time_).seconds() > plan_timeout_) {
      return false;
    }

    const std::string plan_frame = latest_plan_->header.frame_id;
    if (plan_frame.empty()) {
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        robot_base_frame_, plan_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot transform plan to %s: %s", robot_base_frame_.c_str(), ex.what());
      return false;
    }

    bool found_target = false;
    double fallback_distance = std::numeric_limits<double>::max();
    double fallback_angle = 0.0;

    for (const auto & pose : latest_plan_->poses) {
      geometry_msgs::msg::PoseStamped base_pose;
      tf2::doTransform(pose, base_pose, transform);

      const double x = base_pose.pose.position.x;
      const double y = base_pose.pose.position.y;
      const double distance = std::hypot(x, y);

      if (x <= 0.05) {
        continue;
      }

      const double angle = std::atan2(y, x);
      if (distance >= lookahead_distance_) {
        *target_angle = angle;
        return true;
      }

      if (lookahead_distance_ - distance < fallback_distance) {
        fallback_distance = lookahead_distance_ - distance;
        fallback_angle = angle;
        found_target = true;
      }
    }

    if (found_target) {
      *target_angle = fallback_angle;
      return true;
    }

    return false;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  nav_msgs::msg::Path::SharedPtr latest_plan_;
  rclcpp::Time latest_plan_time_{0, 0, RCL_ROS_TIME};

  std::string robot_base_frame_;
  double lookahead_distance_;
  double align_yaw_tolerance_;
  double min_linear_speed_to_gate_;
  double angular_kp_;
  double min_align_angular_speed_;
  double max_align_angular_speed_;
  double plan_timeout_;
  double tf_timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontAlignCmdVelGate>());
  rclcpp::shutdown();
  return 0;
}
