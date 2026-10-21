#pragma once
#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/reference_path_utils.h>
#include <plan_manage/polyline_trajectory_utils.h>
#include <scan_native.h>
#include <string>

namespace scan_planner {
struct NativeReference {
  PolynomialTraj global_traj_;
  double global_duration_=0.0,last_progress_time_=0.0,projection_time_=0.0;
  Eigen::Vector3d getPosition(double t) {return global_traj_.evaluate(t);}
  Eigen::Vector3d getVelocity(double t) {return global_traj_.evaluateVel(t);}
};
class SCANPlannerManager {
 public:
  enum class Initialization {Automatic,Polynomial,Astar};
  bool usedAstarInitialization() const {return last_initialization_used_astar_;}
  SCANPlannerManager(const ScanConfig& config, GridMap::Ptr map);
  bool planGlobalTrajWaypoints(const std::vector<Eigen::Vector3d>& points);
  void getLocalTarget(const Eigen::Vector3d& start);
  bool reboundReplan(Eigen::Vector3d start_pt,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
                     Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel,Initialization initialization=Initialization::Automatic);
  // 实验台对照：沿全局参考剩余段直接生成轨迹，不做局部A*与避障优化。
  bool referenceReplan(Eigen::Vector3d start_pt,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
                       Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel);
  bool refineTrajAlgo(UniformBspline &traj,std::vector<Eigen::Vector3d>& derivatives,double ratio,double& ts,Eigen::MatrixXd& controls);
  void reparamBspline(UniformBspline& spline,std::vector<Eigen::Vector3d>& derivatives,double ratio,Eigen::MatrixXd& controls,double& dt,double& increment);
  UniformBspline trajectory_;
  NativeReference global_data_;
  Eigen::Vector3d local_target_pt_,local_target_vel_;
  std::string error;
 private:
  void fixBoundary(Eigen::MatrixXd& controls,double ts,const Eigen::Vector3d& start,const Eigen::Vector3d& end,const std::vector<Eigen::Vector3d>& derivatives);
  bool last_initialization_used_astar_=false;
  ScanConfig config_;
  struct Parameters {double max_vel_,max_acc_,ctrl_pt_dist,planning_horizon_,feasibility_tolerance_;} pp_;
  GridMap::Ptr grid_map_;
  BsplineOptimizer::Ptr bspline_optimizer_rebound_;
  Eigen::Vector3d start_pt_,end_pt_,odom_pos_;
  double planning_horizon_,reference_min_z_=0.0,reference_max_z_=0.0;
};
}
