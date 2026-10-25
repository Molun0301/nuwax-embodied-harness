#pragma once
#include <author_host.h>
#include <scan_native.h>
namespace scan_planner {
// Algorithm defaults from author A advanced_param.xml. Physical limits and
// calibrated body coverage come from the unchanged platform request.
inline ros::NodeHandle authorParameters(const ScanConfig& c) {
 ros::NodeHandle n;
 n.setParam("fsm/navi_mode",3);
 n.setParam("fsm/thresh_replan",1.0); n.setParam("fsm/thresh_no_replan",.1);
 n.setParam("fsm/planning_horizon",c.planning_horizon);
 n.setParam("fsm/emergency_time_",1.0); n.setParam("fsm/fail_safe",true);
 n.setParam("fsm/max_replan_fail_count",1000);
 n.setParam("manager/max_vel",c.max_velocity);n.setParam("manager/max_acc",c.max_acceleration);
 n.setParam("manager/max_jerk",4.0);n.setParam("manager/control_points_distance",.2);
 n.setParam("manager/feasibility_tolerance",.5);n.setParam("manager/planning_horizon",c.planning_horizon);
 n.setParam("optimization/lambda_smooth",1.0);n.setParam("optimization/lambda_collision",1.0);
 n.setParam("optimization/lambda_feasibility",.1);n.setParam("optimization/lambda_fitness",1.0);
 n.setParam("optimization/dist0",.2);n.setParam("optimization/max_vel",c.max_velocity);
 n.setParam("optimization/max_acc",c.max_acceleration);n.setParam("optimization/order",3);
 n.setParam("optimization/vel_tolerance",1.0);n.setParam("optimization/acc_tolerance",1.0);
 n.setParam("grid_map/resolution",c.resolution);
 n.setParam("grid_map/sliding_map_size_x",10.0);n.setParam("grid_map/sliding_map_size_y",10.0);n.setParam("grid_map/sliding_map_size_z",5.0);
 n.setParam("grid_map/map_sliding_thresh",.2);n.setParam("grid_map/map_sliding_en",true);
 // Original two-cylinder covering approximation with unchanged actual dimensions.
 n.setParam("grid_map/double_cylinder_offset",c.body_length/4.0);
 n.setParam("grid_map/double_cylinder_radius",std::hypot(c.body_width/2.0,c.body_length/4.0));
 n.setParam("grid_map/body_length",c.body_length);
 n.setParam("grid_map/body_width",c.body_width);
 n.setParam("grid_map/body_center_offset_x",c.body_center_x);
 n.setParam("grid_map/body_center_offset_y",c.body_center_y);
 // Author A IV-B uses a step-aware collision band, not a solid block down
 // through contacting feet. Keep the full standing envelope unchanged for the
 // independent physical judge; retain every observed support/ceiling point.
 // 支撑面位于其占据格内，格顶最多高出真实支撑一个分辨率。越阶能力为零或不足一格时，
 // 碰撞带下沿取到支撑格格顶，支撑面自身的格不算机身障碍；更高的占据格仍按原规则判碰撞。
 const double collision_low=std::max(c.max_step_up,c.resolution)-c.body_to_ground;
 const double collision_high=c.body_z_max;
 n.setParam("grid_map/body_center_offset_z",(collision_low+collision_high)/2.0);
 n.setParam("grid_map/obstacles_inflation_z_up",(collision_high-collision_low)/2.0);
 n.setParam("grid_map/obstacles_inflation_z_down",(collision_high-collision_low)/2.0);
 n.setParam("grid_map/body_height",c.body_to_ground);
 n.setParam("grid_map/frame_id",std::string("map"));
 n.setParam("grid_map/sensor_type",std::string("lidar"));n.setParam("grid_map/cloud_is_world",true);n.setParam("grid_map/need_extrinsic",false);
 n.setParam("grid_map/p_hit",.85);n.setParam("grid_map/p_miss",.30);n.setParam("grid_map/p_min",.12);n.setParam("grid_map/p_max",.98);n.setParam("grid_map/p_occ",.80);n.setParam("grid_map/max_ray_length",5.0);
 return n;
}
}
