#include <plan_manage/scan_replan_fsm.h>
#include <author_controller.h>
#include <author_command_check.h>
#include <author_diagnostics.h>
#include <author_parameters.h>
#include <business_constraint_mask.h>
#include <scan_native.h>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>
using Eigen::Vector3d;
namespace {
struct RegisteredFrame {
 std::vector<Vector3d> points;
 Vector3d origin=Vector3d::Zero(),body=Vector3d::Zero();
 Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
 int64_t timestamp=0;
};
struct AuthorSession {
 ScanConfig config{};
 std::unique_ptr<scan_planner::SCANReplanFSM> fsm;
 scan_planner::AuthorController controller;
 geometry_msgs::Twist raw_command;
 std::vector<double> reference;
 RegisteredFrame frame;bool has_frame=false,static_fixture=false,static_imported=false;
 bool local_enabled=true,cycle_complete=false,execution_failed=false,safety_dirty=false;
 bool command_paused=false;
 Vector3d command_target=Vector3d::Zero();double command_target_yaw=0.;
 int64_t last_fsm=std::numeric_limits<int64_t>::min(),last_safety=std::numeric_limits<int64_t>::min(),last_control=std::numeric_limits<int64_t>::min();
 Vector3d last_position=Vector3d::Zero();double last_yaw=0;
 uint64_t plans=0;
 Eigen::Vector2d previous_control=Eigen::Vector2d::Zero(),sample_control=Eigen::Vector2d::Zero();
 double previous_w=0,sample_w=0;bool has_control=false,has_sample=false;int64_t sample_time=0;
 void retireExecution() {
  if(fsm)fsm->retireExecution();
  controller.receive_traj=false;controller.have_odom=false;controller.traj.clear();
  controller.traj_duration=0.;controller.traj_id=0;controller.exec_time=0.;controller.last_update_time=ros::Time{};
  controller.odom_pos.setZero();controller.odom_yaw=0.;
  raw_command={};reference.clear();cycle_complete=false;execution_failed=false;safety_dirty=false;command_paused=false;
  command_target.setZero();command_target_yaw=0.;
  last_fsm=last_safety=last_control=std::numeric_limits<int64_t>::min();
  last_position.setZero();last_yaw=0.;previous_control.setZero();sample_control.setZero();
  previous_w=sample_w=0.;has_control=has_sample=false;sample_time=0;
 }
 bool hasExecution()const{return !reference.empty()||controller.receive_traj;}
 bool sameMap(const ScanConfig& c)const {
  return config.resolution==c.resolution&&config.body_length==c.body_length&&config.body_width==c.body_width
   &&config.body_height==c.body_height&&config.body_to_ground==c.body_to_ground
   &&config.body_center_x==c.body_center_x&&config.body_center_y==c.body_center_y
   &&config.body_z_min==c.body_z_min&&config.body_z_max==c.body_z_max
   &&config.max_step_up==c.max_step_up&&config.max_step_down==c.max_step_down&&config.max_slope==c.max_slope;
 }
 bool samePlanning(const ScanConfig& c)const {
  return config.max_velocity==c.max_velocity&&config.max_acceleration==c.max_acceleration
   &&config.max_yaw_rate==c.max_yaw_rate&&config.planning_horizon==c.planning_horizon
   &&config.sample_dt==c.sample_dt&&config.max_yaw_acceleration==c.max_yaw_acceleration
   &&config.max_reverse_velocity==c.max_reverse_velocity&&config.max_lateral_velocity==c.max_lateral_velocity;
 }
 // Returns true only when an existing map was invalidated. No stored scan is replayed here.
 bool configure(const ScanConfig& c) {
  if(!fsm){initialize(c);return false;}
  if(sameMap(c)&&samePlanning(c))return false;
  if(hasExecution())throw std::runtime_error("Author A configuration changed with an unretired trajectory; stop execution before reconfiguration");
  if(!sameMap(c)){
   retireExecution();fsm.reset();frame=RegisteredFrame{};has_frame=false;static_imported=false;
   initialize(c);return true;
  }
  auto params=scan_planner::authorParameters(c);fsm->refreshParameters(params);
  controller.configure(std::max(c.max_velocity,c.max_reverse_velocity),c.max_lateral_velocity,c.max_yaw_rate);
  config=c;return false;
 }
 void initialize(const ScanConfig& c) {
  config=c;auto params=scan_planner::authorParameters(c);
  fsm=std::make_unique<scan_planner::SCANReplanFSM>();fsm->init(params);
  controller=scan_planner::AuthorController{};
  controller.configure(std::max(c.max_velocity,c.max_reverse_velocity),c.max_lateral_velocity,c.max_yaw_rate);
  controller.cmd_vel_pub.sink=[this](const std::any& value){raw_command=std::any_cast<geometry_msgs::Twist>(value);};
  controller.execution_frozen_pub.sink=[this](const std::any& value){fsm->onFrozen(std::any_cast<std_msgs::Bool>(value).data);};
  fsm->onSpline([this](const scan_planner::Bspline& spline){++plans;controller.bsplineCallback(std::make_shared<const scan_planner::Bspline>(spline));last_control=std::numeric_limits<int64_t>::min();});
 }
};
void copy(double* to,const Vector3d& v){for(int i=0;i<3;++i)to[i]=v[i];}
double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
}
#define SCAN_ARGUMENTS const ScanConfig *config,const double *obstacles,size_t obstacle_count,const double *ground,size_t ground_count,\
 const double *reference,size_t reference_count,const double start_position[3],const double start_velocity[3],const double start_acceleration[3],\
 double start_yaw,double start_yaw_rate,const double *boundary,size_t boundary_count,const int32_t *forbidden_columns,size_t forbidden_count,\
 double goal_yaw,int64_t odometry_timestamp_ns,ScanOutput *output,char *error,size_t error_capacity
#define SCAN_FORWARD config,obstacles,obstacle_count,ground,ground_count,reference,reference_count,start_position,start_velocity,start_acceleration,\
 start_yaw,start_yaw_rate,boundary,boundary_count,forbidden_columns,forbidden_count,goal_yaw,odometry_timestamp_ns,output,error,error_capacity
extern "C" int nuwax_scan_session_plan(void* raw,SCAN_ARGUMENTS) {
 AuthorSession ephemeral;auto& s=raw?*static_cast<AuthorSession*>(raw):ephemeral;
 *output={};scan_native_diagnostic.clear();
 try {
  if(!s.local_enabled)throw std::runtime_error("Author A mode 3 requires local planning; disabled-local remains a retired comparison, not author execution");
  ros::host_time_seconds=double(odometry_timestamp_ns)*1e-9;
  s.configure(*config);
  const auto& c=*config;const Vector3d position=Eigen::Map<const Vector3d>(start_position);
  auto map=s.fsm->manager().grid_map_;
  if(s.static_fixture) {
   std::vector<Vector3d> occupied;occupied.reserve(obstacle_count+ground_count);
   for(size_t i=0;i<obstacle_count;++i)occupied.emplace_back(Eigen::Map<const Vector3d>(obstacles+3*i));
   for(size_t i=0;i<ground_count;++i)occupied.emplace_back(Eigen::Map<const Vector3d>(ground+3*i));
   // Static occupancy fixture is explicit, never called a LiDAR observation.
   if(!s.static_imported){map->importStaticOccupied(occupied,position);s.static_imported=true;}
   else map->repopulateStaticOccupied(occupied,position);
  } else {
   if(!s.has_frame)throw std::runtime_error("Author A map requires a registered source frame with its paired sensor origin; fused surface snapshot is not a raycast input");
   map->updateOccupancyCallback({});
  }
  std::vector<Eigen::Vector2d> polygon;polygon.reserve(boundary_count);
  for(size_t i=0;i<boundary_count;++i)polygon.emplace_back(boundary[2*i],boundary[2*i+1]);
  std::vector<Eigen::Vector2i> forbidden;forbidden.reserve(forbidden_count);
  for(size_t i=0;i<forbidden_count;++i)forbidden.emplace_back(forbidden_columns[2*i],forbidden_columns[2*i+1]);
  auto business=std::make_shared<BusinessConstraintMask>(c.resolution,c.body_length,c.body_width,c.body_center_x,c.body_center_y,std::move(polygon),forbidden);
  map->setBusinessConstraintQuery([business](const Vector3d& p,double yaw){return business->rejects(p,yaw);});
  // 起点必须使用当前实测机体查询；已有曲线的计划时刻不能代替实际位置。
  if(map->getInflateOccupancy(position,start_yaw)!=0)
   throw std::runtime_error(business->rejects(position,start_yaw)
    ? "Current body pose is outside the explicit user boundary or forbidden mask"
    : "Current body pose is occupied or outside the local map");
  if(scan_planner::authorTraceEnabled()) {
   auto out=scan_planner::authorTraceStream();
   out << "{\"event\":\"map_probes\",\"stamp_ns\":" << odometry_timestamp_ns
       << ",\"frame_stamp_ns\":" << s.frame.timestamp << ",\"actual\":";
   scan_planner::traceMapProbe(out,*map,position,start_yaw);
   out << ",\"forward_half_metre\":";
   scan_planner::traceMapProbe(out,*map,position+Vector3d(.5*std::cos(start_yaw),.5*std::sin(start_yaw),0.),start_yaw);
   out << "}\n";
  }
  auto odom=std::make_shared<nav_msgs::Odometry>();odom->pose.pose.position.x=position.x();odom->pose.pose.position.y=position.y();odom->pose.pose.position.z=position.z();
  odom->pose.pose.orientation.w=std::cos(start_yaw/2.);odom->pose.pose.orientation.z=std::sin(start_yaw/2.);
  if(start_velocity){odom->twist.twist.linear.x=start_velocity[0];odom->twist.twist.linear.y=start_velocity[1];odom->twist.twist.linear.z=start_velocity[2];}
  s.fsm->onOdometry(odom,start_velocity!=nullptr);s.controller.odomCallback(odom);
  if(!s.has_sample||odometry_timestamp_ns>s.sample_time){s.sample_time=odometry_timestamp_ns;s.sample_control=s.previous_control;s.sample_w=s.previous_w;s.has_sample=true;}
  const Eigen::Vector2d actual_v=start_velocity?Eigen::Map<const Vector3d>(start_velocity).head<2>().eval():s.sample_control;
  const double actual_w=std::isfinite(start_yaw_rate)?start_yaw_rate:s.sample_w;
  const uint64_t previous_plans=s.plans;
  const int state_before=s.fsm->state();
  double control_time_before=s.controller.exec_time;
  bool control_called=false;
  if(reference_count>=2) {
   std::vector<double> incoming(reference,reference+reference_count*3);
   if(incoming!=s.reference){
    auto path=std::make_shared<nav_msgs::Path>();
    for(size_t i=0;i<reference_count;++i){geometry_msgs::PoseStamped p;p.pose.position.x=reference[3*i];p.pose.position.y=reference[3*i+1];
     // Incoming PCT reference is body-reference XYZ. A callback adds the exact
     // same calibrated constant; this reversible transport conversion is not a terrain projection.
     p.pose.position.z=reference[3*i+2]-c.body_to_ground;path->poses.push_back(p);}
    if(scan_planner::authorTraceEnabled()){auto out=scan_planner::authorTraceStream();out<<"{\"event\":\"reference_event\",\"stamp_ns\":"<<odometry_timestamp_ns<<",\"points\":[";for(size_t i=0;i<incoming.size();++i){if(i)out<<',';out<<incoming[i];}out<<"]}\n";}
    s.fsm->onReferencePath(path);if(!s.fsm->hasTarget())throw std::runtime_error(scan_native_diagnostic.empty()?"Author A reference path was not accepted":scan_native_diagnostic);s.reference=std::move(incoming);s.cycle_complete=false;s.execution_failed=false;
   }
   // Current host cadence is the real caller cadence. No fabricated catch-up
   // timer ticks using a single physical odometry sample.
   if(odometry_timestamp_ns>s.last_fsm){
    s.fsm->onFrozen(s.command_paused);
    const bool was_executing=s.fsm->executingTrajectory();
    const bool was_emergency=s.fsm->emergencyStopping();
    s.fsm->tick();
    if(was_executing&&s.fsm->waitingForTarget()&&!s.fsm->hasTarget()&&s.plans>0)s.cycle_complete=true;
    if(was_emergency&&s.fsm->waitingForTarget()&&!s.fsm->hasTarget())s.execution_failed=true;
    s.last_fsm=odometry_timestamp_ns;
   }
   if(s.safety_dirty||odometry_timestamp_ns>s.last_safety){s.fsm->safetyTick();s.last_safety=odometry_timestamp_ns;s.safety_dirty=false;}
   if(odometry_timestamp_ns!=s.last_control||position!=s.last_position||start_yaw!=s.last_yaw){
    control_time_before=s.controller.exec_time;control_called=true;
    s.command_target=position;s.command_target_yaw=start_yaw;
    if(s.controller.receive_traj) {
     const auto before=s.controller.traj[0].evaluateDeBoorT(control_time_before);
     s.command_target_yaw=s.controller.estimateDesiredYaw(control_time_before,before);
     s.command_target=s.controller.traj[0].evaluateDeBoorT(std::min(s.controller.traj_duration,control_time_before+s.controller.time_forward));
    }
    s.controller.cmdCallback({});s.last_control=odometry_timestamp_ns;s.last_position=position;s.last_yaw=start_yaw;
   }
  } else if(std::isfinite(goal_yaw)) {
   s.command_target=position;s.command_target_yaw=goal_yaw;
   // Preserve existing explicit user polygon/forbidden policy for this outer operation.
   if(business->rejects(position,start_yaw))throw std::runtime_error("Terminal orientation is outside the explicit user boundary or forbidden mask");
   // Explicit terminal orientation is an outer NEH business operation, not a
   // replacement goal in author mode 3. Keep the existing calibrated braking law.
   const double remaining=angle(goal_yaw-start_yaw),step=c.max_yaw_acceleration*c.sample_dt;
   s.raw_command={};s.raw_command.angular.z=std::copysign(std::min(c.max_yaw_rate,std::sqrt(step*step+2*c.max_yaw_acceleration*std::abs(remaining))-step),remaining);
  } else s.raw_command={};
  const double cs=std::cos(start_yaw),sn=std::sin(start_yaw);
  const double vx=std::clamp(s.raw_command.linear.x,-c.max_reverse_velocity,c.max_velocity);
  const double vy=std::clamp(s.raw_command.linear.y,-c.max_lateral_velocity,c.max_lateral_velocity);
  Eigen::Vector2d target(cs*vx-sn*vy,sn*vx+cs*vy),delta=target-actual_v;
  const double step=c.max_acceleration*c.sample_dt;if(delta.norm()>step)delta*=step/delta.norm();
  const Eigen::Vector2d chosen=actual_v+delta;
  // 变回机体系后兑现实际通道边界；坐标旋转舍入不能生成被声明为零的横移或倒退。
  output->command_forward=std::clamp(cs*chosen.x()+sn*chosen.y(),-c.max_reverse_velocity,c.max_velocity);
  output->command_lateral=std::clamp(-sn*chosen.x()+cs*chosen.y(),-c.max_lateral_velocity,c.max_lateral_velocity);
  output->command_yaw_rate=std::clamp(s.raw_command.angular.z,actual_w-c.max_yaw_acceleration*c.sample_dt,actual_w+c.max_yaw_acceleration*c.sample_dt);
  output->command_duration=c.sample_dt;output->replanned=s.plans!=previous_plans;
  output->execution_cycle_complete=s.cycle_complete&&reference_count>=2;
  output->execution_failed=s.execution_failed&&reference_count>=2;
  // Author safetyTick can enter EMERGENCY after this tick's FSM callback.
  // Retire only the exported old motion here; the original FSM still owns its
  // next-tick emergency trajectory, recovery branch and retry budget.
  const bool execution_retired=output->execution_cycle_complete||output->execution_failed||(reference_count>=2&&s.fsm->emergencyStopping());
  if(execution_retired){output->command_forward=0.;output->command_lateral=0.;output->command_yaw_rate=0.;}
  bool command_waiting=false;
  if(!execution_retired) {
   const auto checked=scan_planner::checkAuthorCommand(*map,c,position,start_yaw,actual_v,actual_w,
       Eigen::Vector2d(output->command_forward,output->command_lateral),output->command_yaw_rate,
       s.command_target,s.command_target_yaw,reference_count>=2&&s.controller.receive_traj);
   // 几何修正动作尚未执行原轨迹，曲线暂停；实测位置仍可真实移动。
   s.command_paused=(!checked.found&&checked.exhausted)||checked.limited;
   if(checked.found) {
    output->command_forward=checked.linear.x();output->command_lateral=checked.linear.y();
    output->command_yaw_rate=checked.angular;
   } else {
    // 计算不足先归零等待，不把资源不足写成几何不可达。
    output->command_forward=output->command_lateral=output->command_yaw_rate=0.;
    command_waiting=checked.exhausted;
   }
   if(s.controller.receive_traj&&reference_count>=2) {
    if(s.command_paused&&control_called)s.controller.exec_time=control_time_before;
    // 同一拥有者同步原曲线时钟；真实里程计时间与位置始终不改。
    s.fsm->manager().local_data_.start_time_=ros::Time(ros::Time::now().toSec()-s.controller.exec_time);
   }
   s.fsm->onFrozen(s.command_paused);
   if(scan_planner::authorTraceEnabled()&&(checked.limited||command_waiting)) {
    auto out=scan_planner::authorTraceStream();
    out<<"{\"event\":\"command_geometry\",\"stamp_ns\":"<<odometry_timestamp_ns
       <<",\"paused\":"<<s.command_paused<<",\"waiting\":"<<command_waiting
       <<",\"braking_translation\":"<<checked.hold_translation
       <<",\"command\":["<<output->command_forward<<','<<output->command_lateral<<','<<output->command_yaw_rate<<"]}\n";
   }
   if(!checked.found&&!checked.exhausted)
    throw std::runtime_error("Current motion and braking sweep intersect the body collision map");
  } else {s.command_paused=false;s.fsm->onFrozen(false);}
  s.previous_control=Eigen::Vector2d(cs*output->command_forward-sn*output->command_lateral,
                                  sn*output->command_forward+cs*output->command_lateral);
  s.previous_w=output->command_yaw_rate;s.has_control=true;
  output->motion_boundary_sources[0]=s.fsm->startVelocitySource();output->motion_boundary_sources[1]=s.fsm->startAccelerationSource();output->motion_boundary_sources[2]=std::isfinite(start_yaw_rate)?0u:3u;
  auto& manager=s.fsm->manager();
  if(!s.reference.empty()){output->reference_progress=manager.global_data_.last_progress_time_;output->reference_duration=manager.global_data_.global_duration_;copy(output->local_target,s.controller.receive_traj?s.fsm->localTarget():s.fsm->originalGoal());}
  if(s.controller.receive_traj&&reference_count>=2&&!execution_retired&&!s.command_paused&&!command_waiting){
   const double begin=s.controller.exec_time,end=s.controller.traj_duration;output->duration=std::max(0.,end-begin);
   output->sample_count=static_cast<size_t>(std::ceil(output->duration/c.sample_dt))+1;
   output->samples=static_cast<ScanSample*>(std::calloc(output->sample_count,sizeof(ScanSample)));if(!output->samples)throw std::bad_alloc();
   for(size_t i=0;i<output->sample_count;++i){auto& sample=output->samples[i];sample.time=std::min(output->duration,i*c.sample_dt);const double t=begin+sample.time;
    copy(sample.position,s.controller.traj[0].evaluateDeBoorT(t));copy(sample.velocity,s.controller.traj[1].evaluateDeBoorT(t));copy(sample.acceleration,s.controller.traj[2].evaluateDeBoorT(t));sample.yaw=s.controller.estimateDesiredYaw(t,Eigen::Map<const Vector3d>(sample.position));sample.yaw_rate=0.;}
  }
  if(scan_planner::authorTraceEnabled()&&(state_before!=s.fsm->state()||previous_plans!=s.plans)) {
   auto out=scan_planner::authorTraceStream();auto& local=manager.local_data_;
   out<<"{\"event\":\"session_transition\",\"stamp_ns\":"<<odometry_timestamp_ns<<",\"from\":"<<state_before<<",\"to\":"<<s.fsm->state()<<",\"has_target\":"<<s.fsm->hasTarget()<<",\"published_curves\":"<<s.plans<<",\"trajectory_id\":"<<local.traj_id_<<",\"curve_start\":"<<(s.controller.receive_traj?local.start_time_.toSec():0.)<<",\"curve_duration\":"<<(s.controller.receive_traj?local.duration_:0.)<<",\"global_duration\":"<<manager.global_data_.global_duration_<<",\"global_progress_t\":"<<manager.global_data_.last_progress_time_<<",\"actual_position\":";scan_planner::traceMatrix(out,position);
   out<<",\"original_goal\":";scan_planner::traceMatrix(out,s.fsm->originalGoal());
   if(s.controller.receive_traj){out<<",\"curve_first\":";scan_planner::traceMatrix(out,local.position_traj_.evaluateDeBoorT(0.));out<<",\"curve_last\":";scan_planner::traceMatrix(out,local.position_traj_.evaluateDeBoorT(local.duration_));out<<",\"curve_end_velocity\":";scan_planner::traceMatrix(out,local.velocity_traj_.evaluateDeBoorT(local.duration_));out<<",\"curve_end_acceleration\":";scan_planner::traceMatrix(out,local.acceleration_traj_.evaluateDeBoorT(local.duration_));out<<",\"controller_exec_time\":"<<s.controller.exec_time;}
   out<<"}\n";
  }
  (void)start_acceleration;return 0;
 } catch(const std::exception& e){if(error_capacity){std::strncpy(error,e.what(),error_capacity-1);error[error_capacity-1]=0;}nuwax_scan_free(output);return 1;}
}
extern "C" int nuwax_scan_plan(SCAN_ARGUMENTS){return nuwax_scan_session_plan(nullptr,SCAN_FORWARD);}
extern "C" void* nuwax_scan_session_create(){return new(std::nothrow) AuthorSession;}
extern "C" void nuwax_scan_session_destroy(void* p){delete static_cast<AuthorSession*>(p);}
extern "C" void nuwax_scan_session_reset(void* p){if(p){auto* s=static_cast<AuthorSession*>(p);const bool fixture=s->static_fixture;const bool enabled=s->local_enabled;const auto plans=s->plans;*s=AuthorSession{};s->static_fixture=fixture;s->local_enabled=enabled;s->plans=plans;}}
extern "C" void nuwax_scan_session_notify_source_changed(void* p){if(p)static_cast<AuthorSession*>(p)->safety_dirty=true;}
extern "C" void nuwax_scan_session_reset_execution(void* p){if(p)static_cast<AuthorSession*>(p)->retireExecution();}
extern "C" int nuwax_scan_session_configure(void* p,const ScanConfig* config,int* map_rebuilt,char* error,size_t capacity){
 if(!p||!config||!map_rebuilt)return 1;
 try{*map_rebuilt=static_cast<AuthorSession*>(p)->configure(*config)?1:0;return 0;}
 catch(const std::exception& e){if(capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}return 1;}
}
extern "C" void nuwax_scan_session_set_local_planning(void* p,int enabled){if(p)static_cast<AuthorSession*>(p)->local_enabled=enabled!=0;}
extern "C" uint64_t nuwax_scan_session_plan_count(const void* p){return p?static_cast<const AuthorSession*>(p)->plans:0;}
extern "C" void nuwax_scan_session_set_static_fixture(void* p,int enabled){if(p)static_cast<AuthorSession*>(p)->static_fixture=enabled!=0;}
extern "C" void nuwax_scan_free(ScanOutput* p){std::free(p->samples);p->samples=nullptr;p->sample_count=0;}

extern "C" int nuwax_scan_session_ingest_registered_frame(void* p,const ScanConfig* config,const double* points,size_t count,
 const double origin[3],const double body_position[3],int64_t timestamp) {
 if(!p||!config||!origin||!body_position||(count&&!points))return 1;
 auto& s=*static_cast<AuthorSession*>(p);
 RegisteredFrame f;f.origin=Eigen::Map<const Vector3d>(origin);f.body=Eigen::Map<const Vector3d>(body_position);
 // All points already use map axes. Identity denotes that coordinate basis,
 // not an invented sensor attitude; A never rotates world-mode points.
 f.orientation=Eigen::Quaterniond::Identity();
 if(!f.origin.allFinite()||!f.body.allFinite())return 1;
 f.timestamp=timestamp;f.points.reserve(count);
 for(size_t i=0;i<count;++i)f.points.emplace_back(Eigen::Map<const Vector3d>(points+i*3));
 try {
  s.configure(*config);
  if(!s.fsm->manager().grid_map_->ingestLidarFrame(f.points,f.origin,f.orientation,f.body))return 1;
  s.frame=std::move(f);s.has_frame=true;s.safety_dirty=true;return 0;
 } catch(const std::exception&) {return 1;}
}
