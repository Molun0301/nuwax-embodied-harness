#include <author_parameters.h>
#include <plan_manage/scan_replan_fsm.h>
#include <cassert>
#include <memory>
#include <iostream>
std::shared_ptr<nav_msgs::Path> route(double floor){auto path=std::make_shared<nav_msgs::Path>();for(int i=0;i<=60;++i){geometry_msgs::PoseStamped p;p.pose.position.x=i*.05;p.pose.position.z=floor+(i<20?0.:(i<40?.1:.2));path->poses.push_back(p);}return path;}
std::shared_ptr<nav_msgs::Odometry> odom(double z){auto p=std::make_shared<nav_msgs::Odometry>();p->pose.pose.position.z=z;p->pose.pose.orientation.w=1.;return p;}
int main(){
 ScanConfig c{};c.resolution=double(float(.05));c.body_length=.5794;c.body_width=.328;c.body_to_ground=.4;c.body_center_x=.0503;c.body_z_min=-.448;c.body_z_max=.057;c.max_step_up=.15;c.max_step_down=.15;c.max_slope=1.;c.max_velocity=1.;c.max_acceleration=1.;c.max_yaw_rate=.8;c.max_yaw_acceleration=2.;c.sample_dt=.05;c.planning_horizon=3.5;c.max_reverse_velocity=1.;c.max_lateral_velocity=1.;
 auto n=scan_planner::authorParameters(c);scan_planner::SCANReplanFSM f;f.init(n);f.onOdometry(odom(.4));f.onReferencePath(route(0));
 auto& m=f.manager();auto& h=m.route_height_;auto* map=m.grid_map_.get();auto* opt=m.bspline_optimizer_rebound_.get();auto* search=opt->a_star_.get();
 assert(h.ready());assert(h.heightAtArc(0)==.4);assert(h.points.size()==61);
 std::weak_ptr<int> old_task;
 {auto token=std::make_shared<int>(7);old_task=token;opt->bind_route_segment_=[token](int,int){};search->route_height_query_=[token](double){return 999.;};}
 assert(!old_task.expired());assert(opt->bind_route_segment_);assert(search->route_height_query_);
 f.retireExecution();
 assert(old_task.expired());assert(!opt->bind_route_segment_);assert(!search->route_height_query_);assert(!h.ready());assert(h.points.empty()&&h.times.empty()&&h.waypoint_arcs.empty());
 assert(m.grid_map_.get()==map&&m.bspline_optimizer_rebound_.get()==opt&&opt->a_star_.get()==search);
 std::cout<<"retire_releases_both_old_task_callbacks=1 clears_height_and_time=1 preserves_map_optimizer_search_identity=1\n";
 f.onOdometry(odom(2.4));f.onReferencePath(route(2.));
 assert(h.ready());assert(h.points.size()==61);assert(h.heightAtArc(0)==2.+.4);assert(!opt->bind_route_segment_&&!search->route_height_query_);assert(h.heightAtArc(h.arcs.back())==2.+.2+.4);
 std::cout<<"new_path_same_xy_different_level_uses_new_height=1 no_retired_callback_reintroduced=1\n";
 f.retireExecution();f.retireExecution();assert(!h.ready()&&!opt->bind_route_segment_&&!search->route_height_query_);
 std::cout<<"repeated_retirement_idempotent=1\n";
}
