#include <plan_manage/scan_replan_fsm.h>
#include <author_controller.h>
#include <author_command_check.h>
#include <author_overlap_escape.h>
#include <author_terminal.h>
#include <author_diagnostics.h>
#include <author_parameters.h>
#include <trajectory_heading.h>
#include <business_constraint_mask.h>
#include <surface_obstacle_layer.h>
#include <scan_native.h>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
using Eigen::Vector3d;
namespace {
constexpr double MINIMUM_DEVIATION_M = 0.20;
constexpr double REPLAN_REMAINING_SECONDS = 2.5;
using Snapshot = std::shared_ptr<const GridMap::CollisionSnapshot>;
struct MapOwner {
  ScanConfig config;
  GridMap map;
  std::shared_ptr<const SurfaceObstacleLayer> surface;
  std::shared_ptr<const BusinessConstraintMask> business;
  explicit MapOwner(const ScanConfig& c):config(c) { auto p=scan_planner::authorParameters(c);map.initMap(p); }
};
struct Trajectory {
  scan_planner::Bspline spline;
  scan_planner::UniformBspline position,velocity,acceleration;
  uint32_t sources[3]{2,2,3};
  Vector3d local_target=Vector3d::Zero();double reference_progress=0.,reference_duration=0.;
  scan_planner::HeadingPlan heading;
  bool reaches_reference_end=false;
  double headingAt(double time) {
    const double total=duration();
    return heading.at(scan_planner::trajectoryHeading(velocity,total,time,.01,heading.yaw0),time,total);
  }
  explicit Trajectory(const scan_planner::Bspline& s):spline(s) {
    Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
    for(size_t i=0;i<s.pos_pts.size();++i)points.col(i)=Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
    for(size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
    position=scan_planner::UniformBspline(points,s.order,0.1);position.setKnot(knots);
    velocity=position.getDerivative();acceleration=velocity.getDerivative();
  }
  double duration() {return position.getTimeSum();}
  // 跟踪只沿曲线自身时间连续搜索，不受全局路线提示跳段影响。
  std::pair<double,double> project(const Vector3d& p,double previous,double period) {
    const double left_bound=std::max(0.,previous-period),right_bound=std::min(duration(),previous+1.);
    const double span=right_bound-left_bound;const int count=std::max(1,int(std::ceil(span/.01)));
    double best=std::numeric_limits<double>::infinity(),tau=left_bound;
    for(int i=0;i<=count;++i) {
      const double t=left_bound+span*i/count;
      const double distance=(position.evaluateDeBoorT(t)-p).squaredNorm();
      if(distance<best) {best=distance;tau=t;}
    }
    // 只细化已选样条位置，不修改结点、控制点或规划器的数值实现。
    const double left=std::max(left_bound,tau-span/count),right=std::min(right_bound,tau+span/count);
    for(int i=0;i<16;++i) {
      const Vector3d delta=position.evaluateDeBoorT(tau)-p,v=velocity.evaluateDeBoorT(tau),a=acceleration.evaluateDeBoorT(tau);
      const double denominator=v.squaredNorm()+delta.dot(a);
      if(denominator<=0.)break;
      const double next=std::clamp(tau-delta.dot(v)/denominator,left,right);
      if(next==tau)break;
      tau=next;
    }
    return {tau,(position.evaluateDeBoorT(tau)-p).norm()};
  }
  std::pair<double,double> projectStart(const Vector3d& p,double maximum_deviation) {
    const double total=duration();const int count=std::max(1,int(std::ceil(total/.01)));
    double best=std::numeric_limits<double>::infinity(),tau=0.;
    // 新曲线由实测状态起步；沿时间从起点寻找首个匹配的局部最近点。
    // 安装不能用只增不减的全局路线进度跳过曲线开头。
    for(int i=0;i<=count;++i) {
      const double t=total*i/count,distance=(position.evaluateDeBoorT(t)-p).squaredNorm();
      if(distance>best && best<=maximum_deviation*maximum_deviation)break;
      if(distance<best){best=distance;tau=t;}
    }
    const double left=std::max(0.,tau-total/count),right=std::min(total,tau+total/count);
    for(int i=0;i<16;++i) {
      const Vector3d delta=position.evaluateDeBoorT(tau)-p,v=velocity.evaluateDeBoorT(tau),a=acceleration.evaluateDeBoorT(tau);
      const double denominator=v.squaredNorm()+delta.dot(a);
      if(denominator<=0.)break;
      const double next=std::clamp(tau-delta.dot(v)/denominator,left,right);
      if(next==tau)break;
      tau=next;
    }
    return {tau,(position.evaluateDeBoorT(tau)-p).norm()};
  }
};
using Curve=std::shared_ptr<Trajectory>;
struct PlannerSession {
  std::shared_ptr<GridMap> map=std::make_shared<GridMap>();
  std::unique_ptr<scan_planner::SCANReplanFSM> fsm;
  std::vector<double> reference;
  Curve produced;
  ScanConfig config{};
  void initialize(const ScanConfig& c) {
    auto p=scan_planner::authorParameters(c);fsm=std::make_unique<scan_planner::SCANReplanFSM>();fsm->init(p,map);
    fsm->onSpline([this](const scan_planner::Bspline& s){produced=std::make_shared<Trajectory>(s);produced->heading=fsm->manager().local_data_.heading;});
    config=c;reference.clear();
  }
};
struct TrackerSession {
  GridMap map;
  scan_planner::AuthorController controller;
  scan_planner::AuthorOverlapEscape escape;
  Curve curve,pending;
  geometry_msgs::Twist raw;
  double phase=0.,reference_time=0.;
  std::optional<int64_t> reference_stamp;
  bool install_pending=false;
  TrackerSession() {controller.cmd_vel_pub.sink=[this](const std::any& v){raw=std::any_cast<geometry_msgs::Twist>(v);};}
};
void errorText(char* out,size_t capacity,const std::exception& e) {if(capacity){std::strncpy(out,e.what(),capacity-1);out[capacity-1]=0;}}
nav_msgs::OdometryConstPtr odometry(const ScanState& state) {
  auto p=std::make_shared<nav_msgs::Odometry>();p->pose.pose.position.x=state.position[0];p->pose.pose.position.y=state.position[1];p->pose.pose.position.z=state.position[2];
  p->pose.pose.orientation.w=std::cos(state.yaw/2.);p->pose.pose.orientation.z=std::sin(state.yaw/2.);
  p->twist.twist.linear.x=state.velocity[0];p->twist.twist.linear.y=state.velocity[1];p->twist.twist.linear.z=state.velocity[2];return p;
}
}
extern "C" void* nuwax_scan_map_create(const ScanConfig* c){try{return new MapOwner(*c);}catch(...){return nullptr;}}
extern "C" void nuwax_scan_map_destroy(void* p){delete static_cast<MapOwner*>(p);}
extern "C" int nuwax_scan_map_surface(void* p,const double* obstacles,size_t count,const double* ground,size_t ground_count,
    const double* boundary,size_t boundary_count,const int32_t* forbidden,size_t forbidden_count) {
  try {
    auto& o=*static_cast<MapOwner*>(p);auto surface=std::make_shared<SurfaceObstacleLayer>(o.config,obstacles,count,ground,ground_count);
    o.surface=surface;
    o.map.setSurfaceCollisionQuery([surface](const Vector3d& x,double yaw,double pad,std::vector<GridMap::BodyCell>* hits,const std::set<GridMap::BodyCell>* initial,bool planar){return surface->contacts(x,yaw,pad,hits,initial,planar);});
    std::vector<Eigen::Vector2d> polygon;for(size_t i=0;i<boundary_count;++i)polygon.emplace_back(boundary[2*i],boundary[2*i+1]);
    std::vector<Eigen::Vector2i> columns;for(size_t i=0;i<forbidden_count;++i)columns.emplace_back(forbidden[2*i],forbidden[2*i+1]);
    const auto& c=o.config;auto mask=std::make_shared<BusinessConstraintMask>(c.resolution,c.body_length,c.body_width,c.body_center_x,c.body_center_y,std::move(polygon),columns);
    o.business=mask;
    o.map.setBusinessConstraintQuery([mask](const Vector3d& x,double yaw){return mask->rejects(x,yaw);});
    o.map.setBusinessEnvelopeQuery([mask](const Vector3d& x,double yaw,double pad){return mask->rejectsEnvelope(x,yaw,pad);});return 0;
  }catch(...){return 1;}
}
extern "C" int nuwax_scan_map_ingest(void* p,const double* points,size_t count,const uint32_t* ground,size_t ground_count,const double origin[3],const double body[3]) {
  try {
    auto& map=static_cast<MapOwner*>(p)->map;std::vector<Vector3d> values;values.reserve(count);for(size_t i=0;i<count;++i)values.emplace_back(Eigen::Map<const Vector3d>(points+3*i));
    std::vector<uint32_t> members;if(ground_count)members.assign(ground,ground+ground_count);
    if(!map.ingestLidarFrame(values,Eigen::Map<const Vector3d>(origin),Eigen::Quaterniond::Identity(),Eigen::Map<const Vector3d>(body),members))return 1;
    map.updateOccupancyCallback({});return 0;
  }catch(...){return 1;}
}
extern "C" void nuwax_scan_map_seed_occupied(void* p,const double* points,size_t count,const double body[3]) {
  std::vector<Vector3d> values;for(size_t i=0;i<count;++i)values.emplace_back(Eigen::Map<const Vector3d>(points+3*i));
  static_cast<MapOwner*>(p)->map.repopulateStaticOccupied(values,Eigen::Map<const Vector3d>(body));
}
extern "C" void* nuwax_scan_map_snapshot(const void* p){
  const auto& owner=*static_cast<const MapOwner*>(p);
  return new Snapshot(owner.map.collisionSnapshot(owner.config,owner.surface.get(),owner.business.get()));
}
extern "C" int nuwax_scan_snapshot_affects(const void* current,const void* previous,const void* trajectory,double from) {
  GridMap now,before;now.bindCollisionSnapshot(*static_cast<const Snapshot*>(current));before.bindCollisionSnapshot(*static_cast<const Snapshot*>(previous));
  auto curve=*static_cast<const Curve*>(trajectory);const double total=curve->duration(),dt=std::min(.01,now.getResolution()/2.);
  const int count=std::max(1,int(std::ceil((total-from)/dt)));double yaw=0.;
  for(int i=0;i<=count;++i) {
    const double t=from+(total-from)*i/count;const Vector3d p=curve->position.evaluateDeBoorT(t),v=curve->velocity.evaluateDeBoorT(t);
    yaw=curve->headingAt(t);
    const double pad=2.*now.getResolution();
    if(now.getInflateOccupancy(p,yaw)!=before.getInflateOccupancy(p,yaw)
        || now.bodyContacts(p,yaw,pad)!=before.bodyContacts(p,yaw,pad))return 1;
  }
  return 0;
}
extern "C" void nuwax_scan_snapshot_destroy(void* p){delete static_cast<Snapshot*>(p);}
extern "C" void nuwax_scan_terminal_points(const void* snapshot,const ScanConfig* config,const double* goal,
    double yaw,double tolerance,ScanTerminalPoints* output) {
  GridMap map;map.bindCollisionSnapshot(*static_cast<const Snapshot*>(snapshot));
  *output=scan_planner::terminalPoints(map,*config,Eigen::Map<const Vector3d>(goal),yaw,tolerance);
}
extern "C" void nuwax_scan_pose_clearance(const void* snapshot,const double* position,double yaw,ScanPoseClearance* output) {
  GridMap map;map.bindCollisionSnapshot(*static_cast<const Snapshot*>(snapshot));
  const Vector3d point=Eigen::Map<const Vector3d>(position);
  output->field_distance=map.fieldDistance(point.x(),point.y());
  output->body_clearance=map.bodyFieldClearance(point,yaw);output->occupied=map.getInflateOccupancy(point,yaw);
}
extern "C" void* nuwax_scan_planner_create(){return new PlannerSession;}
extern "C" void nuwax_scan_planner_destroy(void* p){delete static_cast<PlannerSession*>(p);}
extern "C" void* nuwax_scan_planner_plan(void* p,const ScanConfig* c,const void* snapshot,const double* reference,size_t count,
    const ScanState* state,const void* active,double phase,double goal_yaw,char* error,size_t capacity) {
  try {
    auto& s=*static_cast<PlannerSession*>(p);s.map->bindCollisionSnapshot(*static_cast<const Snapshot*>(snapshot));
    ros::host_time_seconds=state->timestamp_ns*1e-9;
    if(!s.fsm || std::memcmp(&s.config,c,sizeof(*c))!=0)s.initialize(*c);
    auto& manager=s.fsm->manager();
    manager.heading_={state->yaw,std::nullopt,c->max_reverse_velocity,c->max_lateral_velocity,c->max_yaw_rate/2.};
    manager.goal_yaw_=std::isfinite(goal_yaw)?std::optional<double>(goal_yaw):std::nullopt;
    scan_native_diagnostic.clear();s.fsm->onOdometry(odometry(*state),state->measured_velocity,Eigen::Map<const Vector3d>(state->planning_velocity));
    std::vector<double> incoming(reference,reference+count*3);
    if(incoming!=s.reference) {
      auto path=std::make_shared<nav_msgs::Path>();for(size_t i=0;i<count;++i){geometry_msgs::PoseStamped point;point.pose.position.x=reference[3*i];point.pose.position.y=reference[3*i+1];point.pose.position.z=reference[3*i+2]-c->body_to_ground;path->poses.push_back(point);}
      s.fsm->onReferencePath(path);s.reference=std::move(incoming);
    }
    if(!s.fsm->hasTarget())throw std::runtime_error(scan_native_diagnostic.empty()?"原参考路线未被接纳":scan_native_diagnostic);
    if(active) {
      auto curve=*static_cast<const Curve*>(active);auto& local=s.fsm->manager().local_data_;
      local.position_traj_=curve->position;local.velocity_traj_=curve->velocity;local.acceleration_traj_=curve->acceleration;
      local.heading=curve->heading;
      local.duration_=curve->duration();local.start_time_=ros::Time(ros::Time::now().toSec()-phase);
      local.start_pos_=curve->position.evaluateDeBoorT(0.);
    }
    if(!active) {auto& local=s.fsm->manager().local_data_;local.duration_=0.;local.start_time_=ros::Time{};}
    s.produced.reset();s.fsm->requestPlan(active!=nullptr);s.fsm->tick();
    if(!s.produced)throw std::runtime_error(scan_native_diagnostic.empty()?"SCAN本次尚无可执行轨迹":scan_native_diagnostic);
    s.produced->local_target=s.fsm->localTarget();s.produced->reference_progress=s.fsm->manager().global_data_.last_progress_time_;s.produced->reference_duration=s.fsm->manager().global_data_.global_duration_;
    const auto end=s.produced->position.evaluateDeBoorT(s.produced->duration());
    s.produced->reaches_reference_end=std::hypot(end.x()-reference[3*(count-1)],
        end.y()-reference[3*(count-1)+1])<=c->resolution;
    s.produced->sources[0]=s.fsm->startVelocitySource();s.produced->sources[1]=s.fsm->startAccelerationSource();s.produced->sources[2]=state->measured_yaw_rate?0:3;
    return new Curve(s.produced);
  }catch(const std::exception& e){errorText(error,capacity,e);return nullptr;}
}
extern "C" void nuwax_scan_trajectory_destroy(void* p){delete static_cast<Curve*>(p);}
extern "C" void nuwax_scan_trajectory_samples(const void* p,double from,double dt,ScanOutput* out) {
  *out={};auto curve=*static_cast<const Curve*>(p);
  std::copy(curve->sources,curve->sources+3,out->motion_boundary_sources);out->duration=std::max(0.,curve->duration()-from);
  out->reference_progress=curve->reference_progress;out->reference_duration=curve->reference_duration;
  const auto end=curve->local_target;for(int i=0;i<3;++i)out->local_target[i]=end[i];
  out->sample_count=std::ceil(out->duration/dt)+1;out->samples=static_cast<ScanSample*>(std::calloc(out->sample_count,sizeof(ScanSample)));
  double yaw=0.;for(size_t i=0;i<out->sample_count;++i){auto& x=out->samples[i];x.time=std::min(out->duration,i*dt);const double t=std::min(curve->duration(),from+x.time);const auto pos=curve->position.evaluateDeBoorT(t),v=curve->velocity.evaluateDeBoorT(t),a=curve->acceleration.evaluateDeBoorT(t);yaw=curve->headingAt(t);for(int j=0;j<3;++j){x.position[j]=pos[j];x.velocity[j]=v[j];x.acceleration[j]=a[j];}x.yaw=yaw;}
}
extern "C" void* nuwax_scan_tracker_create(){return new TrackerSession;}
extern "C" int nuwax_scan_motion_stopped(const ScanState* state) {
  return state->measured_velocity&&state->measured_yaw_rate
      &&std::hypot(state->velocity[0],state->velocity[1])<.02&&std::abs(state->yaw_rate)<.02;
}
extern "C" void nuwax_scan_tracker_destroy(void* p){delete static_cast<TrackerSession*>(p);}
extern "C" void nuwax_scan_tracker_clear_trajectory(void* p){auto& s=*static_cast<TrackerSession*>(p);s.curve.reset();s.pending.reset();s.controller.receive_traj=false;s.phase=0.;s.reference_time=0.;s.reference_stamp.reset();}
extern "C" void nuwax_scan_tracker_clear(void* p){nuwax_scan_tracker_clear_trajectory(p);static_cast<TrackerSession*>(p)->escape=scan_planner::AuthorOverlapEscape{};}
extern "C" void nuwax_scan_tracker_install(void* p,const void* curve){auto& s=*static_cast<TrackerSession*>(p);s.pending=*static_cast<const Curve*>(curve);}
extern "C" void nuwax_scan_tracker_track(void* p,const ScanConfig* c,const void* snapshot,const ScanState* state,
    const double previous[3],double terminal_yaw,const double* terminal_position,ScanTrackOutput* out,ScanOutput* recovery,char* error,size_t capacity) {
  *out={};*recovery={};scan_native_diagnostic.clear();try {
    auto& s=*static_cast<TrackerSession*>(p);s.map.bindCollisionSnapshot(*static_cast<const Snapshot*>(snapshot));ros::host_time_seconds=state->timestamp_ns*1e-9;
    const Vector3d position=Eigen::Map<const Vector3d>(state->position);const double yaw=state->yaw,cs=std::cos(yaw),sn=std::sin(yaw);
    std::pair<double,double> installed_projection;
    if(s.pending){
      const auto projected=s.pending->projectStart(position,std::max(MINIMUM_DEVIATION_M,2.*c->resolution));
      if(projected.second<=std::max(MINIMUM_DEVIATION_M,2.*c->resolution)){
        s.curve=std::move(s.pending);s.install_pending=true;out->installed=1;installed_projection=projected;
      }else{s.pending.reset();out->reason=SCAN_TRACK_REJECTED_TRAJECTORY;}
    }
    const Eigen::Vector2d command_world(cs*previous[0]-sn*previous[1],sn*previous[0]+cs*previous[1]);
    const Eigen::Vector2d actual=state->measured_velocity?Eigen::Map<const Vector3d>(state->velocity).head<2>().eval():command_world;
    const double angular=state->measured_yaw_rate?state->yaw_rate:previous[2];
    if(s.escape.hasDecision(state->timestamp_ns)) {
      s.escape.plan(s.map,*c,position,yaw,state->measured_velocity?state->velocity:nullptr,
          state->measured_yaw_rate?state->yaw_rate:std::numeric_limits<double>::quiet_NaN(),state->timestamp_ns,previous,*recovery);
      s.curve.reset();out->reason=SCAN_TRACK_NO_TRAJECTORY;
      out->command[0]=recovery->command_forward;out->command[1]=recovery->command_lateral;out->command[2]=recovery->command_yaw_rate;return;
    }
    const int occupied=s.map.getInflateOccupancy(position,yaw);
    if(occupied!=0) {
      s.escape.observeOverlap();
      if(occupied==1 && !(static_cast<const Snapshot*>(snapshot)->get()->business && static_cast<const Snapshot*>(snapshot)->get()->business(position,yaw))) {
        s.escape.plan(s.map,*c,position,yaw,state->measured_velocity?state->velocity:nullptr,state->measured_yaw_rate?state->yaw_rate:std::numeric_limits<double>::quiet_NaN(),state->timestamp_ns,previous,*recovery);
        s.curve.reset();out->reason=SCAN_TRACK_NO_TRAJECTORY;
        out->command[0]=recovery->command_forward;out->command[1]=recovery->command_lateral;out->command[2]=recovery->command_yaw_rate;return;
      }
      throw std::runtime_error("Current body pose is occupied or outside the explicit user boundary or forbidden mask or surface collision map");
    }
    s.escape.observeClear(position,state->timestamp_ns);
    ScanConfig terminal_config;
    if(std::isfinite(terminal_yaw)&&terminal_position&&c->max_reverse_velocity==0.) {
      terminal_config=*c;terminal_config.max_reverse_velocity=std::max(.04,c->min_executable_linear_mps);c=&terminal_config;
    }
    Vector3d target=position;double desired_yaw=yaw;
    if(std::isfinite(terminal_yaw)) {
      s.controller.configure(std::max(c->max_velocity,c->max_reverse_velocity),c->max_lateral_velocity,c->max_yaw_rate);
      const double diff=std::atan2(std::sin(terminal_yaw-yaw),std::cos(terminal_yaw-yaw));
      s.raw={};s.raw.angular.z=std::clamp(s.controller.kp_yaw*diff,-c->max_yaw_rate,c->max_yaw_rate);desired_yaw=terminal_yaw;
      if(terminal_position) {
        target=Eigen::Map<const Vector3d>(terminal_position);
        Eigen::Vector2d velocity=(target-position).head<2>();
        const double distance=velocity.norm();
        const double cap=std::min(.10,scan_planner::deviceSpeedMagnitude(*c));
        if(distance>.02)velocity*=std::min(cap,std::clamp(s.controller.kp_pos*distance,.03,.10))/distance;
        else velocity.setZero();
        s.raw.linear.x=cs*velocity.x()+sn*velocity.y();s.raw.linear.y=-sn*velocity.x()+cs*velocity.y();
      }
    }else {
      if(!s.curve){out->reason|=SCAN_TRACK_NO_TRAJECTORY;return;}
      else {
      const auto projection=out->installed?installed_projection:s.curve->project(position,s.phase,c->sample_dt);s.phase=projection.first;out->deviation=projection.second;
      if(out->deviation>std::max(MINIMUM_DEVIATION_M,2.*c->resolution)){out->reason|=SCAN_TRACK_DEVIATED;s.curve.reset();return;}
      out->remaining=std::max(0.,s.curve->duration()-s.phase);out->phase=s.phase;
      if(out->remaining<=0.) {
        out->reason|=SCAN_TRACK_FINISHED;s.curve.reset();
        // FINISHED只表示几何曲线走完；仍在运动时按实测边界制动，由运行时统一确认停稳。
        if(!nuwax_scan_motion_stopped(state)) {
          const auto brake=scan_planner::checkAuthorCommand(s.map,*c,position,yaw,actual,angular,
              command_world,Eigen::Vector2d::Zero(),0.,position,yaw,false);
          if(!brake.found){out->reason|=SCAN_TRACK_NO_CANDIDATE;return;}
          out->command[0]=brake.linear.x();out->command[1]=brake.linear.y();out->command[2]=brake.angular;
        }
        return;
      }
      if(s.install_pending){s.controller.bsplineCallback(std::make_shared<const scan_planner::Bspline>(s.curve->spline));s.reference_time=s.phase;s.reference_stamp.reset();s.install_pending=false;}
      // 实际投影用于进度；期望参考按控制周期推进，超出原偏离距离时停表。
      const double next_reference=std::min(s.curve->duration(),std::max(s.phase,s.reference_time+c->sample_dt));
      double reference_time=std::max(s.reference_time,s.phase);
      // 同一来源采样的重复计算仍只算一拍，保留既有控制时钟语义。
      if(s.reference_stamp!=state->timestamp_ns) {
        if((s.curve->position.evaluateDeBoorT(next_reference)-position).norm()<=std::max(MINIMUM_DEVIATION_M,2.*c->resolution))
          reference_time=next_reference;
        s.reference_stamp=state->timestamp_ns;
      }
      s.controller.configure(std::max(c->max_velocity,c->max_reverse_velocity),c->max_lateral_velocity,c->max_yaw_rate);
      // 完整参考末段继续位置反馈；局部窗口仍保留原末端减速，不把期望走完当作实际到达。
      s.controller.finish_dist=0.;
      // 本拍已推进并检查参考，回调不再额外推进一拍。
      s.controller.odomCallback(odometry(*state));
      const bool turning=s.curve->heading.turn_first
          &&std::abs(scan_planner::headingDifference(s.curve->headingAt(s.reference_time),yaw))>.3;
      if(!turning)s.reference_time=reference_time;
      desired_yaw=s.curve->headingAt(s.reference_time);
      s.controller.exec_time=s.reference_time;s.controller.last_update_time=ros::Time::now();
      target=s.curve->position.evaluateDeBoorT(std::min(s.curve->duration(),s.reference_time+s.controller.time_forward));s.controller.cmdCallback({});
      if(turning)s.raw.linear={};
      s.raw.angular.z=std::clamp(s.controller.kp_yaw*scan_planner::headingDifference(desired_yaw,yaw),-c->max_yaw_rate,c->max_yaw_rate);
      if(out->remaining<=REPLAN_REMAINING_SECONDS&&!s.curve->reaches_reference_end)out->reason|=SCAN_TRACK_REPLAN_EARLY;
      }
    }
    // 归档271次停车的滑行P95：≥0.5米/秒为0.105米，0.2—0.3为0.037米，<0.1为0.017米，另需留约0.03米定位抖动。
    const double clearance=s.map.bodyFieldClearance(position,yaw);
    double speed_cap=scan_planner::clearanceSpeedCap(*c,clearance);
    if(s.curve&&!std::isfinite(terminal_yaw)) {
      const double horizon=std::max(0.,(scan_planner::deviceSpeedMagnitude(*c)-.10)/c->max_acceleration);
      const double remaining=std::max(0.,s.curve->duration()-s.reference_time),period=2.*c->sample_dt;
      const int count=int(std::ceil(std::min(horizon,remaining)/period));
      double tangent=yaw;
      for(int k=1;k<=count;++k) {
        const double t=std::min(k*period,remaining),at=s.reference_time+t;
        const auto point=s.curve->position.evaluateDeBoorT(at),velocity=s.curve->velocity.evaluateDeBoorT(at);
        tangent=s.curve->headingAt(at);
        const double available=scan_planner::clearanceSpeedCap(*c,s.map.bodyFieldClearance(point,tangent));
        speed_cap=std::min(speed_cap,available+c->max_acceleration*t);
      }
    }
    const double radius=std::hypot(c->body_length/2.+std::abs(c->body_center_x),c->body_width/2.+std::abs(c->body_center_y));
    double vx=std::clamp(s.raw.linear.x,-c->max_reverse_velocity,c->max_velocity),vy=std::clamp(s.raw.linear.y,-c->max_lateral_velocity,c->max_lateral_velocity);
    const double requested_speed=std::hypot(vx,vy);
    if(requested_speed>speed_cap){vx*=speed_cap/requested_speed;vy*=speed_cap/requested_speed;}
    const double requested_w=std::clamp(s.raw.angular.z,-speed_cap/radius,speed_cap/radius);
    Eigen::Vector2d preferred_world(cs*vx-sn*vy,sn*vx+cs*vy),delta=preferred_world-command_world;const double step=c->max_acceleration*c->sample_dt;
    if(delta.norm()>step)delta*=step/delta.norm();const auto chosen=(command_world+delta).eval();
    const Eigen::Vector2d preferred(std::clamp(cs*chosen.x()+sn*chosen.y(),-c->max_reverse_velocity,c->max_velocity),std::clamp(-sn*chosen.x()+cs*chosen.y(),-c->max_lateral_velocity,c->max_lateral_velocity));
    const double w=std::clamp(requested_w,angular-c->max_yaw_acceleration*c->sample_dt,angular+c->max_yaw_acceleration*c->sample_dt);
    const auto checked=scan_planner::checkAuthorCommand(s.map,*c,position,yaw,actual,angular,command_world,preferred,w,target,desired_yaw,bool(s.curve)&&!std::isfinite(terminal_yaw));
    out->limited=checked.limited;if(!checked.found){out->reason=SCAN_TRACK_NO_CANDIDATE;s.curve.reset();return;}
    out->command[0]=checked.linear.x();out->command[1]=checked.linear.y();out->command[2]=checked.angular;
    if(scan_planner::authorTraceEnabled()){auto trace=scan_planner::authorTraceStream();trace<<"{\"event\":\"track\",\"phase\":"<<out->phase<<",\"remaining\":"<<out->remaining<<",\"reason\":"<<out->reason<<"}\n";}
  }catch(const std::exception& e){errorText(error,capacity,e);out->reason=std::string(e.what()).find("BODY_OVERLAP_RECOVERY_FAILED")!=std::string::npos?SCAN_TRACK_RECOVERY_FAILED:SCAN_TRACK_NO_CANDIDATE;}
}
extern "C" void nuwax_scan_free(ScanOutput* out){std::free(out->samples);out->samples=nullptr;out->sample_count=0;}
