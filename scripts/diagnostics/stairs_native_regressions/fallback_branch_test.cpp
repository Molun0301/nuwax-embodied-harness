#include <author_parameters.h>
#include <plan_manage/scan_replan_fsm.h>
#include <cassert>
#include <iostream>
int main(){ScanConfig c{};c.resolution=double(float(.05));c.body_length=.5794;c.body_width=.328;c.body_to_ground=.4;c.body_center_x=.0503;c.body_z_min=-.448;c.body_z_max=.057;c.max_step_up=.15;c.max_step_down=.15;c.max_slope=1.;c.max_velocity=1.;c.max_acceleration=1.;c.max_yaw_rate=.8;c.max_yaw_acceleration=2.;c.sample_dt=.05;c.planning_horizon=3.5;c.max_reverse_velocity=1.;c.max_lateral_velocity=1.;auto n=scan_planner::authorParameters(c);scan_planner::SCANReplanFSM f;f.init(n);auto odom=std::make_shared<nav_msgs::Odometry>();odom->pose.pose.position.z=.4;odom->pose.pose.orientation.w=1.;f.onOdometry(odom);auto path=std::make_shared<nav_msgs::Path>();for(int i=0;i<=60;++i){geometry_msgs::PoseStamped p;p.pose.position.x=i*.05;p.pose.position.z=i<20?0.:(i<40?.1:.2);path->poses.push_back(p);}f.onReferencePath(path);
 auto& m=f.manager();ros::host_time_seconds=1.;
 const Eigen::Vector3d p(0,0,.4),v=Eigen::Vector3d::Zero(),target(2.5,0,.4);
 m.reboundReplan(p,v,v,target,v,true,false); // first initialization: unchanged
 m.reboundReplan(p,v,v,target,v,false,false); // historical branch: unchanged
 m.reboundReplan(p,v,v,target,v,true,false); // explicit noninitial deterministic seed
 m.reboundReplan(p,v,v,target,v,true,true); // random branch: unchanged
 f.onReferencePath(path);
 m.reboundReplan(p,v,v,target,v,true,false); // active-reference bypass: same explicit flags
 f.retireExecution();f.onOdometry(odom);f.onReferencePath(path);
 assert(m.flag_first_call);
 m.reboundReplan(p,v,v,target,v,true,false); // normal new path after reset: unchanged
 std::cout<<"branch calls completed; inspect trace for exact activation flags\n";
}
