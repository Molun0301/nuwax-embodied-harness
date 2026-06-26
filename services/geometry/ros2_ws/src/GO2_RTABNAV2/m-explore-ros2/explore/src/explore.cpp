/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

inline static bool goals_near(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two,
                              double tolerance)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  return std::hypot(dx, dy) < tolerance;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<float>("robot_progress_timeout", 12.0);
  this->declare_parameter<float>("robot_progress_radius", 0.12);
  this->declare_parameter<float>("cmd_vel_stale_timeout", 8.0);
  this->declare_parameter<float>("cmd_vel_linear_threshold", 0.02);
  this->declare_parameter<float>("cmd_vel_angular_threshold", 0.05);
  this->declare_parameter<std::string>("progress_odom_frame", "go2_odom");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<float>("goal_tolerance", 0.35);
  this->declare_parameter<float>("min_goal_interval", 3.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("robot_progress_timeout", robot_progress_timeout_);
  this->get_parameter("robot_progress_radius", robot_progress_radius_);
  this->get_parameter("cmd_vel_stale_timeout", cmd_vel_stale_timeout_);
  this->get_parameter("cmd_vel_linear_threshold", cmd_vel_linear_threshold_);
  this->get_parameter("cmd_vel_angular_threshold", cmd_vel_angular_threshold_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("min_goal_interval", min_goal_interval_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("progress_odom_frame", progress_odom_frame_);

  progress_timeout_ = timeout;
  last_robot_movement_time_ = this->now();
  last_cmd_vel_time_ = this->now();
  goal_start_time_ = this->now();
  last_progress_ = this->now();
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&Explore::cmdVelCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "等待连接 Nav2 navigate_to_pose 服务...");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "已连接 Nav2 navigate_to_pose 服务");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "获取机器人初始位姿...");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "无法查询 TF %s -> %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED, 0, 0, false);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

bool Explore::lookupOdomPosition(geometry_msgs::msg::Point& out) const
{
  try {
    const auto tf_stamped = tf_buffer_.lookupTransform(
        progress_odom_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.5));
    out.x = tf_stamped.transform.translation.x;
    out.y = tf_stamped.transform.translation.y;
    out.z = tf_stamped.transform.translation.z;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(logger_, "odom progress TF failed (%s->%s): %s",
                 progress_odom_frame_.c_str(), robot_base_frame_.c_str(),
                 ex.what());
    return false;
  }
}

bool Explore::updateRobotProgress()
{
  geometry_msgs::msg::Point odom_pos;
  if (!lookupOdomPosition(odom_pos)) {
    return false;
  }

  if (!last_robot_pose_valid_) {
    last_robot_pose_ = odom_pos;
    last_robot_pose_valid_ = true;
    last_robot_movement_time_ = this->now();
    last_progress_ = this->now();
    return true;
  }

  const double dx = odom_pos.x - last_robot_pose_.x;
  const double dy = odom_pos.y - last_robot_pose_.y;
  if (std::hypot(dx, dy) >= robot_progress_radius_) {
    last_robot_pose_ = odom_pos;
    last_robot_movement_time_ = this->now();
    last_progress_ = this->now();
  }
  return true;
}

bool Explore::robotStuck() const
{
  if (!pursuit_active_ || !last_robot_pose_valid_) {
    return false;
  }
  return (this->now() - last_robot_movement_time_) >
         tf2::durationFromSec(robot_progress_timeout_);
}

void Explore::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const bool moving =
      std::abs(msg->linear.x) > cmd_vel_linear_threshold_ ||
      std::abs(msg->linear.y) > cmd_vel_linear_threshold_ ||
      std::abs(msg->angular.z) > cmd_vel_angular_threshold_;
  if (moving) {
    saw_cmd_vel_ = true;
    last_cmd_vel_time_ = this->now();
  }
}

bool Explore::cmdVelStale() const
{
  if (!pursuit_active_) {
    return false;
  }
  const auto stale_for = [&](const rclcpp::Time& since) {
    return (this->now() - since) > tf2::durationFromSec(cmd_vel_stale_timeout_);
  };
  if (saw_cmd_vel_) {
    return stale_for(last_cmd_vel_time_);
  }
  return stale_for(goal_start_time_);
}

size_t Explore::countAvailableFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers) const
{
  return static_cast<size_t>(std::count_if(
      frontiers.begin(), frontiers.end(),
      [this](const frontier_exploration::Frontier& f) {
        return !goalOnBlacklist(f.centroid);
      }));
}

void Explore::publishExploreStatus(const std::string& status, size_t frontiers_found,
                                   size_t frontiers_available,
                                   bool use_cached_frontiers)
{
  last_explore_status_ = status;
  if (!use_cached_frontiers) {
    last_frontiers_found_ = frontiers_found;
    last_frontiers_available_ = frontiers_available;
  }
  explore_lite_msgs::msg::ExploreStatus msg;
  msg.status = status;
  msg.blacklist_count = static_cast<uint32_t>(frontier_blacklist_.size());
  msg.frontiers_found = static_cast<uint32_t>(last_frontiers_found_);
  msg.frontiers_available = static_cast<uint32_t>(last_frontiers_available_);
  status_pub_->publish(msg);
  last_status_publish_time_ = this->now();
}

void Explore::cancelStuckGoal(const std::string& reason)
{
  frontier_blacklist_.push_back(prev_goal_);
  RCLCPP_WARN(
      logger_,
      "%s — 已拉黑 (%.2f, %.2f)，累计拉黑 %zu 点，可用前沿 %zu/%zu",
      reason.c_str(), prev_goal_.x, prev_goal_.y, frontier_blacklist_.size(),
      last_frontiers_available_, last_frontiers_found_);
  goal_active_ = false;
  pursuit_active_ = false;
  move_base_client_->async_cancel_all_goals();
  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS);
}

void Explore::makePlan()
{
  // find frontiers (map frame for frontier search)
  auto pose = costmap_client_.getRobotPose();
  // Progress uses odom frame — immune to RTAB-Map map->odom loop jumps.
  updateRobotProgress();

  if (pursuit_active_ && !move_base_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Nav2 navigate_to_pose 未就绪，暂停追击（不拉黑，等待 Nav2 respawn 恢复）");
    goal_active_ = false;
    pursuit_active_ = false;
    move_base_client_->async_cancel_all_goals();
    return;
  }

  if (pursuit_active_ && cmdVelStale()) {
    cancelStuckGoal(
        std::to_string(static_cast<int>(cmd_vel_stale_timeout_)) +
        "s 无 cmd_vel，追击中卡住");
    return;
  }

  if (pursuit_active_ && robotStuck()) {
    cancelStuckGoal(
        "odom " + std::to_string(static_cast<int>(robot_progress_timeout_)) +
        "s 无位移，追击中卡住");
    return;
  }

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    // SLAM maps (e.g. GO2 + RTAB-Map) often have no frontiers until the robot
    // moves and the occupancy grid updates. Keep polling instead of stopping.
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "未找到前沿，等待地图更新...");
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  last_frontiers_found_ = frontiers.size();
  last_frontiers_available_ = countAvailableFrontiers(frontiers);

  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_,
                "所有前沿已尝试完毕（拉黑 %zu 点），探索结束。",
                frontier_blacklist_.size());
    publishExploreStatus(
        explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE,
        last_frontiers_found_, 0, false);
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  const bool same_goal =
      goals_near(prev_goal_, target_position, goal_tolerance_);

  // Blacklist if the robot itself has not moved in odom (not Nav2 feedback).
  if (pursuit_active_ &&
      (this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) &&
      !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_WARN(
        logger_,
        "odom %.0fs 无位移 — 已拉黑 (%.2f, %.2f)，累计拉黑 %zu 点，可用前沿 %zu/%zu",
        progress_timeout_, target_position.x, target_position.y,
        frontier_blacklist_.size(), last_frontiers_available_ - 1,
        last_frontiers_found_);
    goal_active_ = false;
    pursuit_active_ = false;
    prev_goal_ = target_position;
    move_base_client_->async_cancel_all_goals();
    last_frontiers_available_ =
        last_frontiers_available_ > 0 ? last_frontiers_available_ - 1 : 0;
    publishExploreStatus(
        explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS);
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  if (pursuit_active_) {
    if (same_goal) {
      if ((this->now() - last_status_publish_time_) >
          tf2::durationFromSec(2.0)) {
        publishExploreStatus(
            explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS);
        last_status_publish_time_ = this->now();
      }
      RCLCPP_INFO_THROTTLE(
          logger_, *this->get_clock(), 5000,
          "追击中 (%.2f, %.2f) | 已拉黑 %zu 点 | 可用前沿 %zu/%zu",
          prev_goal_.x, prev_goal_.y, frontier_blacklist_.size(),
          last_frontiers_available_, last_frontiers_found_);
      return;
    }
    if ((this->now() - last_goal_sent_time_) <
        tf2::durationFromSec(min_goal_interval_)) {
      RCLCPP_DEBUG_THROTTLE(
          logger_, *this->get_clock(), 5000,
          "抑制重发目标 %.1fs（前沿地图抖动）",
          min_goal_interval_);
      return;
    }
  }

  RCLCPP_INFO(
      logger_,
      "新前沿目标 (%.2f, %.2f) | 已拉黑 %zu 点 | 可用前沿 %zu/%zu",
      target_position.x, target_position.y, frontier_blacklist_.size(),
      last_frontiers_available_, last_frontiers_found_);
  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  prev_goal_ = target_position;
  const bool new_pursuit = !pursuit_active_;
  goal_active_ = true;
  pursuit_active_ = true;
  last_goal_sent_time_ = this->now();
  if (new_pursuit) {
    goal_start_time_ = this->now();
    geometry_msgs::msg::Point odom_pos;
    if (lookupOdomPosition(odom_pos)) {
      last_robot_pose_ = odom_pos;
      last_robot_pose_valid_ = true;
      last_robot_movement_time_ = this->now();
      last_progress_ = this->now();
    }
  }

  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS,
      last_frontiers_found_, last_frontiers_available_, false);

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

#ifdef EXPLORE_ROS_FOXY
  send_goal_options.goal_response_callback =
      [this](std::shared_future<NavigationGoalHandle::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Nav2 拒绝了导航目标");
          goal_active_ = false;
          pursuit_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };
#else
  send_goal_options.goal_response_callback =
      [this](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Nav2 拒绝了导航目标");
          goal_active_ = false;
          pursuit_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };
#endif

  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "返回探索起点...");
  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          publishExploreStatus(
              explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN);
          RCLCPP_INFO(logger_, "已返回探索起点。");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal) const
{
  constexpr static size_t tolerace = 5;
  const nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      pursuit_active_ = false;
      last_progress_ = this->now();
      prev_distance_ = 0;
      break;
    case rclcpp_action::ResultCode::ABORTED:
#ifdef NAV2_RESULT_HAS_ERROR_CODE
      if (result.result && result.result->error_code != 0) {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
        frontier_blacklist_.push_back(frontier_goal);
      } else {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=0 — likely a preemption, not blacklisting");
      }
#else
      // Foxy: retry on next planner timer tick (avoid cancel/recovery race).
      RCLCPP_DEBUG(logger_, "Goal aborted on Foxy — will retry on next timer");
#endif
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      return;
    default:
      RCLCPP_WARN(logger_, "Nav2 返回未知结果码");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "探索已开始。");
  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED, 0, 0, false);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "探索已停止。");

  stopped_ = true;
  goal_active_ = false;
  pursuit_active_ = false;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    publishExploreStatus(
        explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  stopped_ = false;
  resuming_ = true;
  RCLCPP_INFO(logger_, "探索已恢复。");
  publishExploreStatus(
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
