#ifndef GO2_NAVIGATION2__CLIMB_STAIRS_TASK_HPP_
#define GO2_NAVIGATION2__CLIMB_STAIRS_TASK_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace go2_navigation2
{

class ClimbStairsTask : public nav2_core::WaypointTaskExecutor
{
public:
  ClimbStairsTask();
  ~ClimbStairsTask() = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  
  // 服务客户端
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr local_costmap_enable_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr global_costmap_enable_client_;
  
  // 参数
  double climb_distance_;      // 爬楼距离（米）
  double wait_duration_;       // 等待时间（秒）
  bool disable_costmap_;       // 是否禁用代价地图
  bool enabled_;               // 是否启用
  std::string mode_;           // 模式: "disable" 或 "enable"
};

}  // namespace go2_navigation2

#endif  // GO2_NAVIGATION2__CLIMB_STAIRS_TASK_HPP_
