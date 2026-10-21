#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace scan_planner {
// 从上游 ClosedLoopController 提取执行时钟、前视航向、速度前馈和位置反馈。
// 时间只由新的里程计采样推进；重复计算同一采样不推进时钟。
class ClosedLoopController {
 public:
  struct Reference {Eigen::Vector2d velocity_world;double yaw_command;double yaw_error;bool frozen;};
  bool ready() const{return !traj_.empty();}
  void reset(){traj_.clear();exec_time_=0.0;}
  void setTrajectory(const UniformBspline& position,double valid_duration,int64_t timestamp_ns) {
    traj_={position};traj_.push_back(traj_[0].getDerivative());traj_.push_back(traj_[1].getDerivative());
    traj_duration_=valid_duration;exec_time_=0.0;last_update_timestamp_=timestamp_ns;
  }
  double time() const{return exec_time_;}
  double duration() const{return traj_duration_;}
  bool exhausted(int64_t timestamp_ns,double command_period) const {
    const double dt=timestamp_ns>last_update_timestamp_?(timestamp_ns-last_update_timestamp_)*1e-9:0.0;
    return exec_time_+dt+command_period>=traj_duration_;
  }
  const UniformBspline& position() const{return traj_[0];}
  Eigen::Vector3d acceleration() const{return traj_[2].evaluateDeBoorT(exec_time_);}
  Reference cmdCallback(const Eigen::Vector3d& odom_pos,double odom_yaw,int64_t timestamp_ns,
      double command_period,double max_vx,double max_vyaw) {
    const double dt=timestamp_ns>last_update_timestamp_?(timestamp_ns-last_update_timestamp_)*1e-9:0.0;
    last_update_timestamp_=std::max(last_update_timestamp_,timestamp_ns);
    const double t_eval=std::min(exec_time_,traj_duration_);
    Eigen::Vector3d pos_des=traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error=normalizeAngle(estimateDesiredYaw(t_eval,pos_des,odom_yaw)-odom_yaw);
    const double yaw_command=std::clamp(kp_yaw_*yaw_error,-max_vyaw,max_vyaw);
    if(std::abs(yaw_error)>heading_error_threshold_) return {Eigen::Vector2d::Zero(),yaw_command,yaw_error,true};
    exec_time_=std::min(traj_duration_,exec_time_+dt);
    // 指令针对下一控制周期；这只是前馈求值，不推进采样时钟或实际位置。
    const double next_time=std::min(traj_duration_,exec_time_+command_period);
    pos_des=traj_[0].evaluateDeBoorT(next_time);
    const Eigen::Vector3d vel_des=traj_[1].evaluateDeBoorT(next_time);
    const Eigen::Vector2d pos_error=pos_des.head<2>()-odom_pos.head<2>();
    const Eigen::Vector2d vel_world=clampNorm(vel_des.head<2>()+kp_pos_*pos_error,max_vx);
    return {vel_world,yaw_command,yaw_error,false};
  }
 private:
  static double normalizeAngle(double a){return std::atan2(std::sin(a),std::cos(a));}
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d& value,double max_norm) {
    const double norm=value.norm();return norm<=max_norm||norm<1e-6?value:value/norm*max_norm;
  }
  double estimateDesiredYaw(double t_cur,const Eigen::Vector3d& pos_des,double odom_yaw) const {
    const double t_look=std::min(traj_duration_,t_cur+time_forward_);
    Eigen::Vector3d direction=traj_[0].evaluateDeBoorT(t_look)-pos_des;
    if(direction.head<2>().squaredNorm()<1e-4) direction=traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm()<1e-4?odom_yaw:std::atan2(direction.y(),direction.x());
  }
  std::vector<UniformBspline> traj_;
  double traj_duration_=0.0,exec_time_=0.0;
  int64_t last_update_timestamp_=0;
  // 上游 controllers.yaml 中的控制参数，机身和运动上限由真实请求提供。
  double time_forward_=0.8,heading_error_threshold_=0.8,kp_pos_=0.8,kp_yaw_=1.5;
};
}
