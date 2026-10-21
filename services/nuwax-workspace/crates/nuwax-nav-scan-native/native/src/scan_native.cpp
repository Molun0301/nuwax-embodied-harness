#include <scan_native.h>
#include <planner_manager.h>
#include <closed_loop_controller.h>
#include <new>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <limits>
using Eigen::Vector3d;

namespace {
double angle(double value) {return std::atan2(std::sin(value),std::cos(value));}
void writeVector(double* dst,const Vector3d& p) { for(int i=0;i<3;++i) dst[i]=p[i]; }
// 实际命令按差动运动积分，纯转向同样查询真实机身扫掠。
bool commandFree(const GridMap& map,const Vector3d& start,double yaw,double speed,double yaw_rate,double dt,const ScanConfig& c,double initial_speed=std::numeric_limits<double>::quiet_NaN(),double initial_yaw_rate=std::numeric_limits<double>::quiet_NaN()) {
  // 与现有执行器反馈模型一致：起始已应用速度按本拍能力过渡，查询平均速度的实际弧。
  if(std::isfinite(initial_speed)) {
    const double next=initial_speed+std::clamp(speed-initial_speed,-c.max_acceleration*dt,c.max_acceleration*dt);
    speed=(initial_speed+next)/2.0;
  }
  if(std::isfinite(initial_yaw_rate)) {
    const double next=initial_yaw_rate+std::clamp(yaw_rate-initial_yaw_rate,-c.max_yaw_acceleration*dt,c.max_yaw_acceleration*dt);
    yaw_rate=(initial_yaw_rate+next)/2.0;
  }
  const double distance=(std::abs(speed)+std::abs(yaw_rate)*std::hypot(c.body_length/2.0+std::abs(c.body_center_x),c.body_width/2.0+std::abs(c.body_center_y)))*dt;
  const int steps=std::max(1,static_cast<int>(std::ceil(distance/(c.resolution/3.0))));
  Vector3d previous=start;
  double initial_ground=0.0;
  const bool measured_start=map.measuredGround(start,initial_ground);
  for(int i=0;i<=steps;++i) {
    const double t=dt*double(i)/steps;
    Vector3d p=start;
    if(std::abs(yaw_rate)>1e-9) {
      p.x()+=speed/yaw_rate*(std::sin(yaw+yaw_rate*t)-std::sin(yaw));
      p.y()+=speed/yaw_rate*(std::cos(yaw)-std::cos(yaw+yaw_rate*t));
    } else {
      p.x()+=speed*t*std::cos(yaw);p.y()+=speed*t*std::sin(yaw);
    }
    double current_ground=0.0;
    if(i>0 && measured_start && map.measuredGround(p,current_ground)) p.z()=start.z()+current_ground-initial_ground;
    if(!map.traversable(previous,p)||map.getInflateOccupancy(p,yaw+yaw_rate*t)) return false;
    previous=p;
  }
  return true;
}

struct LocalMetadata {Vector3d target;double progress=0.0,reference_duration=0.0;};
struct ScanSession {
  scan_planner::ClosedLoopController controller;
  ScanConfig config{};
  Vector3d planned_start=Vector3d::Zero();
  LocalMetadata local;
  uint64_t planning_requests=0;
  // 实验台对照开关；关闭时直接跟随全局参考，控制器与实际机身检查不变。
  bool local_planning=true;
};
Vector3d followMeasuredHeight(const GridMap& map,const Vector3d& actual,Vector3d point) {
  double a,b;if(map.measuredGround(actual,a)&&map.measuredGround(point,b)) point.z()=actual.z()+b-a;
  return point;
}
bool trajectoryFree(scan_planner::UniformBspline spline,const GridMap& map,const Vector3d& actual,double begin,double end,double fallback_yaw) {
  auto velocity=spline.getDerivative();
  const double speed=velocity.getControlPoint().colwise().norm().maxCoeff();
  const double dt=std::min(0.05,map.getResolution()/(2.0*std::max(speed,1e-6)));
  const int samples=std::max(1,static_cast<int>(std::ceil((end-begin)/dt)));
  for(int i=0;i<=samples;++i) {
    const double t=begin+(end-begin)*i/samples;
    const Vector3d p=followMeasuredHeight(map,actual,spline.evaluateDeBoorT(t));
    const Vector3d v=velocity.evaluateDeBoorT(t);
    const double yaw=v.head<2>().norm()>1e-8?std::atan2(v.y(),v.x()):fallback_yaw;
    if(map.getInflateOccupancy(p,yaw)) return false;
  }
  return true;
}
// 输出仅包含上游实际优化检查的前2/3；尾段在下一窗口重新规划，不伪装成可执行轨迹。
bool emitPlan(scan_planner::UniformBspline spline,const LocalMetadata& metadata,const GridMap& map,
    const ScanConfig& c,const Vector3d& start,const Vector3d& velocity,double start_yaw,double start_yaw_rate,
    int64_t timestamp,scan_planner::ClosedLoopController* controller,bool allow_turn_fallback,ScanOutput* output,std::string& error) {
  auto derivative=spline.getDerivative();auto second=derivative.getDerivative();
  const double valid_end=spline.getTimeSum()*2.0/3.0;
  scan_planner::ClosedLoopController::Reference controlled{Eigen::Vector2d::Zero(),0.0,0.0,false};
  if(controller) controlled=controller->cmdCallback(start,start_yaw,timestamp,c.sample_dt,c.max_velocity,c.max_yaw_rate);
  const double begin=controller?controller->time():0.0;
  const double duration=std::max(0.0,valid_end-begin);
  const size_t count=static_cast<size_t>(std::ceil(duration/c.sample_dt))+1;
  output->samples=static_cast<ScanSample*>(std::calloc(count,sizeof(ScanSample)));
  if(!output->samples) throw std::bad_alloc();
  output->sample_count=count;output->duration=duration;
  output->reference_progress=metadata.progress;output->reference_duration=metadata.reference_duration;
  writeVector(output->local_target,metadata.target);
  double previous_yaw=start_yaw;size_t nearest=0;double nearest_distance=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<count;++i) {
    ScanSample& sample=output->samples[i];sample.time=std::min(duration,i*c.sample_dt);
    const double t=begin+sample.time;
    const Vector3d p=spline.evaluateDeBoorT(t),v=derivative.evaluateDeBoorT(t),a=second.evaluateDeBoorT(t);
    writeVector(sample.position,p);writeVector(sample.velocity,v);writeVector(sample.acceleration,a);
    const double yaw=v.head<2>().norm()>1e-8?std::atan2(v.y(),v.x()):previous_yaw;
    const double dt=i?sample.time-output->samples[i-1].time:0.0;
    sample.yaw_rate=i&&dt>0?std::clamp(angle(yaw-previous_yaw)/dt,-c.max_yaw_rate,c.max_yaw_rate):start_yaw_rate;
    sample.yaw=i?angle(previous_yaw+sample.yaw_rate*dt):start_yaw;previous_yaw=sample.yaw;
    const double distance=(p-start).head<2>().squaredNorm();if(distance<nearest_distance){nearest_distance=distance;nearest=i;}
  }
  if(count<2 || nearest+1>=count){error="SCAN local reference prefix has been consumed";return false;}
  const double current_forward=velocity.x()*std::cos(start_yaw)+velocity.y()*std::sin(start_yaw);
  const double lookahead=std::max(c.resolution,std::abs(current_forward)*c.sample_dt);
  Vector3d aim=metadata.target;double desired_speed=0.0;
  for(size_t i=nearest+1;i<count;++i) {
    aim=Eigen::Map<const Vector3d>(output->samples[i].position);
    desired_speed=Eigen::Map<const Vector3d>(output->samples[i].velocity).head<2>().norm();
    if((aim-start).head<2>().norm()>=lookahead) break;
  }
  const double steering_error=angle(std::atan2(aim.y()-start.y(),aim.x()-start.x())-start_yaw);
  const double distance=(aim-start).head<2>().norm();
  const double curvature=distance>1e-9?2.0*std::sin(steering_error)/distance:0.0;
  double along=desired_speed*std::max(0.0,std::cos(steering_error));
  if(controller) {
    // 原世界系速度前馈+位置反馈投影到已声明的前向差动通道，不虚构横移或倒退。
    along=controlled.frozen?0.0:std::max(0.0,controlled.velocity_world.x()*std::cos(start_yaw)+controlled.velocity_world.y()*std::sin(start_yaw));
  }
  const double desired_yaw_rate=controller&&controlled.frozen?controlled.yaw_command:
      std::clamp(std::cos(steering_error)>0.0?along*curvature:steering_error/c.sample_dt,-c.max_yaw_rate,c.max_yaw_rate);
  output->command_yaw_rate=std::clamp(desired_yaw_rate,start_yaw_rate-c.max_yaw_acceleration*c.sample_dt,start_yaw_rate+c.max_yaw_acceleration*c.sample_dt);
  double achievable=along;
  if(std::abs(curvature)>1e-9 && !(controller&&controlled.frozen)) achievable=std::min(along,std::max(0.0,output->command_yaw_rate/curvature));
  const double min_forward=std::max(0.0,current_forward-c.max_acceleration*c.sample_dt);
  const double max_forward=std::max(min_forward,std::min(c.max_velocity,std::max(0.0,current_forward)+c.max_acceleration*c.sample_dt));
  output->command_forward=std::clamp(achievable,min_forward,max_forward);output->command_duration=c.sample_dt;
  if(!commandFree(map,start,start_yaw,output->command_forward,output->command_yaw_rate,c.sample_dt,c,current_forward,start_yaw_rate)) {
    if(allow_turn_fallback&&min_forward==0.0&&std::abs(output->command_yaw_rate)>1e-9&&
        commandFree(map,start,start_yaw,0.0,output->command_yaw_rate,c.sample_dt,c,current_forward,start_yaw_rate)) output->command_forward=0.0;
    else {error="SCAN measured obstacle intersects the actual differential motion";return false;}
  }
  return true;
}
}
#define SCAN_ARGUMENTS const ScanConfig *config,const double *obstacles,size_t obstacle_count,const double *ground,size_t ground_count,\
    const double *reference,size_t reference_count,const double start_position[3],const double start_velocity[3],const double start_acceleration[3],\
    double start_yaw,double start_yaw_rate,const double *boundary,size_t boundary_count,const int32_t *forbidden_columns,size_t forbidden_count,\
    double goal_yaw,int64_t odometry_timestamp_ns,ScanOutput *output,char *error,size_t error_capacity
#define SCAN_FORWARD config,obstacles,obstacle_count,ground,ground_count,reference,reference_count,start_position,start_velocity,start_acceleration,\
    start_yaw,start_yaw_rate,boundary,boundary_count,forbidden_columns,forbidden_count,goal_yaw,odometry_timestamp_ns,output,error,error_capacity
extern "C" int nuwax_scan_session_plan(void* raw_session,SCAN_ARGUMENTS) {
  auto* session=static_cast<ScanSession*>(raw_session);
  try {
    *output={};
    scan_native_diagnostic.clear();
    const ScanConfig& c=*config;
    const Vector3d start=Eigen::Map<const Vector3d>(start_position);
    const Vector3d velocity=Eigen::Map<const Vector3d>(start_velocity);
    const Vector3d acceleration=Eigen::Map<const Vector3d>(start_acceleration);
    auto map=std::make_shared<GridMap>(c,obstacles,obstacle_count,ground,ground_count,boundary,boundary_count,forbidden_columns,forbidden_count);
    map->setBodyReference(start,start_yaw);
    if(reference_count<2 && std::isfinite(goal_yaw)) {
      if(session) session->controller.reset();
      const Vector3d start=Eigen::Map<const Vector3d>(start_position);
      const double remaining_yaw=angle(goal_yaw-start_yaw);
      const double angular_step=c.max_yaw_acceleration*c.sample_dt;
      const double braking_rate=std::sqrt(angular_step*angular_step+2.0*c.max_yaw_acceleration*std::abs(remaining_yaw))-angular_step;
      const double desired_yaw_rate=std::copysign(std::min(c.max_yaw_rate,braking_rate),remaining_yaw);
      const double yaw_rate=std::clamp(desired_yaw_rate,start_yaw_rate-c.max_yaw_acceleration*c.sample_dt,start_yaw_rate+c.max_yaw_acceleration*c.sample_dt);
      const Vector3d measured_velocity=Eigen::Map<const Vector3d>(start_velocity);
      const double actual_forward=measured_velocity.x()*std::cos(start_yaw)+measured_velocity.y()*std::sin(start_yaw);
      const double forward=std::max(0.0,actual_forward-c.max_acceleration*c.sample_dt);
      if(!commandFree(*map,start,start_yaw,forward,yaw_rate,c.sample_dt,c,actual_forward,start_yaw_rate))
        throw std::runtime_error("SCAN measured obstacle intersects the actual turning body");
      output->samples=static_cast<ScanSample*>(std::calloc(2,sizeof(ScanSample)));
      if(!output->samples) throw std::bad_alloc();
      output->sample_count=2;output->duration=c.sample_dt;
      output->command_forward=forward;output->command_yaw_rate=yaw_rate;output->command_duration=c.sample_dt;
      writeVector(output->local_target,start);
      for(int i=0;i<2;++i) {
        const double time=i*c.sample_dt;
        Vector3d position=start;
        if(std::abs(yaw_rate)>1e-9) {
          position.x()+=forward/yaw_rate*(std::sin(start_yaw+yaw_rate*time)-std::sin(start_yaw));
          position.y()+=forward/yaw_rate*(std::cos(start_yaw)-std::cos(start_yaw+yaw_rate*time));
        } else {position.x()+=forward*time*std::cos(start_yaw);position.y()+=forward*time*std::sin(start_yaw);}
        output->samples[i].time=time;writeVector(output->samples[i].position,position);
        output->samples[i].yaw=start_yaw+yaw_rate*time;output->samples[i].yaw_rate=yaw_rate;
        writeVector(output->samples[i].velocity,Vector3d(forward*std::cos(output->samples[i].yaw),forward*std::sin(output->samples[i].yaw),0.0));
      }
      return 0;
    }

    std::string reason;
    if(session&&session->controller.ready()&&std::memcmp(&session->config,&c,sizeof(c))==0&&
       (start-session->planned_start).norm()<1.0&&
       !session->controller.exhausted(odometry_timestamp_ns,c.sample_dt)&&
       trajectoryFree(session->controller.position(),*map,start,session->controller.time(),session->controller.duration(),start_yaw)) {
      auto controller=session->controller;
      if(emitPlan(controller.position(),session->local,*map,c,start,velocity,start_yaw,start_yaw_rate,odometry_timestamp_ns,&controller,false,output,reason)) {
        session->controller=std::move(controller);output->replanned=0;return 0;
      }
      nuwax_scan_free(output);*output={};
    }
    if(session){session->controller.reset();if(session->local_planning)++session->planning_requests;}
    scan_planner::SCANPlannerManager planner(c,map);
    std::vector<Vector3d> path;path.reserve(reference_count);
    for(size_t i=0;i<reference_count;++i) path.emplace_back(reference[i*3],reference[i*3+1],reference[i*3+2]);
    if(!planner.planGlobalTrajWaypoints(path)) throw std::runtime_error(planner.error);
    planner.getLocalTarget(start);
    if(session&&!session->local_planning) {
      // 没有避障优化保证，执行前用同一实际机身检查整段有效轨迹；被挡时交原链停车并重新全局规划。
      if(!planner.referenceReplan(start,velocity,acceleration,planner.local_target_pt_,planner.local_target_vel_)) throw std::runtime_error(planner.error);
      const double valid_end=planner.trajectory_.getTimeSum()*2.0/3.0;
      if(!trajectoryFree(planner.trajectory_,*map,start,0.0,valid_end,start_yaw))
        throw std::runtime_error("SCAN direct global reference (local planning disabled) intersects measured obstacles");
      LocalMetadata metadata{planner.local_target_pt_,planner.global_data_.last_progress_time_,planner.global_data_.global_duration_};
      scan_planner::ClosedLoopController controller;
      controller.setTrajectory(planner.trajectory_,valid_end,odometry_timestamp_ns);
      if(!emitPlan(planner.trajectory_,metadata,*map,c,start,velocity,start_yaw,start_yaw_rate,odometry_timestamp_ns,
          &controller,true,output,reason)) throw std::runtime_error(reason);
      session->controller=std::move(controller);session->config=c;session->planned_start=start;session->local=metadata;
      output->replanned=1;return 0;
    }
    const bool resting=velocity.head<2>().norm()<1e-9&&acceleration.head<2>().norm()<1e-9;
    for(int initialization=0;initialization<(resting?4:2);++initialization) {
      scan_native_diagnostic.clear();
      using Initialization=scan_planner::SCANPlannerManager::Initialization;
      const Initialization seed=initialization%2==0?Initialization::Automatic:
          (planner.usedAstarInitialization()?Initialization::Polynomial:Initialization::Astar);
      Vector3d initial_acceleration=acceleration;
      if(initialization>=2) {
        // 静止时零导数没有航向约束；额外候选沿实际航向使用已有加速度能力起步。
        // 这是待执行轨迹的加速度边界，原实测速度及非零加速度保持不变。
        initial_acceleration.x()=c.max_acceleration*std::cos(start_yaw);
        initial_acceleration.y()=c.max_acceleration*std::sin(start_yaw);
      }
      if(!planner.reboundReplan(start,velocity,initial_acceleration,planner.local_target_pt_,planner.local_target_vel_,seed)) {reason=planner.error;continue;}
      LocalMetadata metadata{planner.local_target_pt_,planner.global_data_.last_progress_time_,planner.global_data_.global_duration_};
      scan_planner::ClosedLoopController controller;
      if(session) controller.setTrajectory(planner.trajectory_,planner.trajectory_.getTimeSum()*2.0/3.0,odometry_timestamp_ns);
      if(!emitPlan(planner.trajectory_,metadata,*map,c,start,velocity,start_yaw,start_yaw_rate,odometry_timestamp_ns,
          session?&controller:nullptr,initialization>0,output,reason)) {
        nuwax_scan_free(output);*output={};continue;
      }
      if(session){session->controller=std::move(controller);session->config=c;session->planned_start=start;session->local=metadata;}
      output->replanned=1;return 0;
    }
    throw std::runtime_error(reason);
  } catch(const std::exception& ex) {
    if(session) session->controller.reset();
    std::string diagnostic=ex.what();if(!scan_native_diagnostic.empty()) diagnostic+="; "+scan_native_diagnostic;
    if(error_capacity){std::strncpy(error,diagnostic.c_str(),error_capacity-1);error[error_capacity-1]='\0';}
    nuwax_scan_free(output);return 1;
  }
}
extern "C" int nuwax_scan_plan(SCAN_ARGUMENTS) {return nuwax_scan_session_plan(nullptr,SCAN_FORWARD);}
extern "C" void* nuwax_scan_session_create(void){return new(std::nothrow) ScanSession;}
extern "C" void nuwax_scan_session_destroy(void* session){delete static_cast<ScanSession*>(session);}
extern "C" void nuwax_scan_session_reset(void* session){if(session) static_cast<ScanSession*>(session)->controller.reset();}
extern "C" void nuwax_scan_session_set_local_planning(void* raw_session,int enabled) {
  if(auto* session=static_cast<ScanSession*>(raw_session)){session->local_planning=enabled!=0;session->controller.reset();}
}
extern "C" uint64_t nuwax_scan_session_plan_count(const void* session){return session?static_cast<const ScanSession*>(session)->planning_requests:0;}
extern "C" void nuwax_scan_free(ScanOutput* output){std::free(output->samples);output->samples=nullptr;output->sample_count=0;}
