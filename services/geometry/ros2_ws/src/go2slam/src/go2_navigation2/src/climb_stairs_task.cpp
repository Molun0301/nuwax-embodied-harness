#include "go2_navigation2/climb_stairs_task.hpp"
#include <chrono>
#include <thread>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

namespace go2_navigation2
{

// 静态变量跟踪代价地图状态
static bool costmap_disabled = false;

ClimbStairsTask::ClimbStairsTask()
: climb_distance_(2.0),
  wait_duration_(0.5),
  disable_costmap_(true),
  enabled_(true),
  mode_("toggle")
{
}

void ClimbStairsTask::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  node_ = parent;
  auto node = node_.lock();
  plugin_name_ = plugin_name;

  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".climb_distance", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wait_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mode", rclcpp::ParameterValue("toggle"));

  node->get_parameter(plugin_name_ + ".enabled", enabled_);
  node->get_parameter(plugin_name_ + ".climb_distance", climb_distance_);
  node->get_parameter(plugin_name_ + ".wait_duration", wait_duration_);
  node->get_parameter(plugin_name_ + ".mode", mode_);

  local_costmap_enable_client_ = node->create_client<std_srvs::srv::SetBool>(
    "/local_costmap/obstacle_layer/toggle");
  global_costmap_enable_client_ = node->create_client<std_srvs::srv::SetBool>(
    "/global_costmap/obstacle_layer/toggle");

  RCLCPP_INFO(node->get_logger(),
    "ClimbStairsTask initialized: distance=%.2fm, mode=%s",
    climb_distance_, mode_.c_str());
}

bool ClimbStairsTask::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose,
  const int & curr_waypoint_index)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("ClimbStairsTask"), "Node expired!");
    return false;
  }

  if (!enabled_) {
    RCLCPP_INFO(node->get_logger(),
      "ClimbStairsTask disabled, skipping waypoint %d", curr_waypoint_index);
    return true;
  }

  RCLCPP_INFO(node->get_logger(),
    "Waypoint %d at (%.2f, %.2f), mode=%s, costmap_disabled=%s",
    curr_waypoint_index, curr_pose.pose.position.x, curr_pose.pose.position.y,
    mode_.c_str(), costmap_disabled ? "true" : "false");

  bool target_state = false;
  
  if (mode_ == "disable") {
    target_state = false;  // 禁用
  } else if (mode_ == "enable") {
    target_state = true;   // 启用
  } else if (mode_ == "toggle") {
    target_state = !costmap_disabled;  // 切换
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = target_state;

  RCLCPP_INFO(node->get_logger(), "%s costmap obstacle layers...",
    target_state ? "Enabling" : "Disabling");

  if (local_costmap_enable_client_->wait_for_service(std::chrono::seconds(2))) {
    auto future = local_costmap_enable_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Local costmap obstacle layer %s",
        target_state ? "enabled" : "disabled");
      costmap_disabled = !target_state;
    }
  }

  if (global_costmap_enable_client_->wait_for_service(std::chrono::seconds(2))) {
    auto future = global_costmap_enable_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Global costmap obstacle layer %s",
        target_state ? "enabled" : "disabled");
    }
  }

  RCLCPP_INFO(node->get_logger(), "Waiting %.2fs...", wait_duration_);
  std::this_thread::sleep_for(std::chrono::duration<double>(wait_duration_));

  RCLCPP_INFO(node->get_logger(),
    "Waypoint %d processed, Nav2 continues", curr_waypoint_index);

  return true;
}

}  // namespace go2_navigation2

PLUGINLIB_EXPORT_CLASS(go2_navigation2::ClimbStairsTask, nav2_core::WaypointTaskExecutor)
