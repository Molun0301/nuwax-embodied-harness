// 基于上游 planner_manager.cpp 与 scan_replan_fsm.cpp 的无 ROS 提取。
#include <planner_manager.h>
#include <cmath>
#include <algorithm>
namespace scan_planner {
SCANPlannerManager::SCANPlannerManager(const ScanConfig& config,GridMap::Ptr map):config_(config),grid_map_(map) {
  pp_={config.max_velocity,config.max_acceleration,config.resolution,config.planning_horizon,0.0};
  planning_horizon_=config.planning_horizon;
  bspline_optimizer_rebound_.reset(new BsplineOptimizer);
  bspline_optimizer_rebound_->setParam(config.max_velocity,config.max_acceleration,config.resolution);
  bspline_optimizer_rebound_->setEnvironment(grid_map_);
  bspline_optimizer_rebound_->a_star_.reset(new AStar);

}
bool SCANPlannerManager::planGlobalTrajWaypoints(const std::vector<Eigen::Vector3d>& input) {
  std::vector<Eigen::Vector3d> points;
  // 对齐上游组合桥接的参考降采样与一个源格的折线简化。
  if(!prepareReferenceWaypoints(input,0.0,config_.resolution*1.5,points,&error,config_.resolution)) return false;
  if(!makePiecewiseLinearTrajectory(points,pp_.max_vel_,global_data_.global_traj_,&error)) return false;
  global_data_.global_duration_=global_data_.global_traj_.getTimeSum();
  end_pt_=points.back();
  return true;
}
  void SCANPlannerManager::getLocalTarget(const Eigen::Vector3d& start)
  {
    start_pt_ = odom_pos_ = start;
    const double max_vel = pp_.max_vel_;
    const double max_acc = pp_.max_acc_;
    const double duration = global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }
    global_data_.projection_time_ = t_proj;

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    reference_min_z_=reference_max_z_=start.z();
    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = global_data_.getPosition(t);
      reference_min_z_=std::min(reference_min_z_,pos_t.z());
      reference_max_z_=std::max(reference_max_z_,pos_t.z());
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return grid_map_->getInflateOccupancy(pt, std::atan2(pt.y()-odom_pos_.y(), pt.x()-odom_pos_.x()));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= duration)
        {
          Eigen::Vector3d pt = global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        target_t = adjusted_t;
      }
      else
      {
      }
    }

    local_target_pt_.z()=grid_map_->bodyHeight(local_target_pt_);
    global_data_.last_progress_time_ = target_t;
    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((pp_.max_vel_*pp_.max_vel_)/(2*pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

}

namespace scan_planner {
void SCANPlannerManager::fixBoundary(Eigen::MatrixXd& q,double ts,const Eigen::Vector3d& start,const Eigen::Vector3d& end,const std::vector<Eigen::Vector3d>& d) {
  // 原最小二乘参数化允许端点微移；实际反馈的起点、速度、加速度需精确接续。
  q.col(0)=start-ts*d[0]+ts*ts*d[2]/3.0;
  q.col(1)=start-ts*ts*d[2]/6.0;
  q.col(2)=start+ts*d[0]+ts*ts*d[2]/3.0;
  // 窗口尾部保持原最小二乘端点约束，避免时间拉长强制尾部穿入障碍。
  (void)end;
}

bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt,Eigen::Vector3d start_vel,
    Eigen::Vector3d start_acc,Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel,Initialization initialization) {
  bool use_astar=initialization==Initialization::Astar;
  // 原 reboundReplan 的多项式初始化分支，每次使用当前真实运动反馈。
  double ts=pp_.ctrl_pt_dist/pp_.max_vel_*1.2;
  const double dist=(start_pt-local_target_pt).norm();
  const double time=std::pow(pp_.max_vel_,2)/pp_.max_acc_>dist
      ? 2.0*std::sqrt(dist/pp_.max_acc_)
      : (dist-std::pow(pp_.max_vel_,2)/pp_.max_acc_)/pp_.max_vel_+2*pp_.max_vel_/pp_.max_acc_;
  if(dist<1e-8) {error="local reference target coincides with the measured start";return false;}
  PolynomialTraj gl_traj=PolynomialTraj::one_segment_traj_gen(start_pt,start_vel,start_acc,
      local_target_pt,local_target_vel,Eigen::Vector3d::Zero(),time);
  std::vector<Eigen::Vector3d> point_set,start_end_derivatives;
  double t;
  bool flag_too_far;
  ts*=1.5;
  do {
    ts/=1.5;
    point_set.clear();
    flag_too_far=false;
    Eigen::Vector3d last_pt=gl_traj.evaluate(0);
    for(t=0;t<time;t+=ts) {
      Eigen::Vector3d pt=gl_traj.evaluate(t);
      if((last_pt-pt).norm()>pp_.ctrl_pt_dist*1.5) {flag_too_far=true;break;}
      last_pt=pt;
      point_set.push_back(pt);
    }
  } while(flag_too_far||point_set.size()<7);
  point_set.push_back(local_target_pt);
  start_end_derivatives={start_vel,local_target_vel,start_acc,Eigen::Vector3d::Zero()};
  // 原 applyLinearZReference 沿水平弧长连续插值；实测地面供碰撞与实际机身扫掠使用。
  std::vector<double> accumulated(point_set.size(),0.0);
  for(size_t i=1;i<point_set.size();++i)
    accumulated[i]=accumulated[i-1]+(point_set[i].head<2>()-point_set[i-1].head<2>()).norm();
  for(size_t i=0;i<point_set.size();++i) {
    const double ratio=accumulated.back()>1e-6?accumulated[i]/accumulated.back():double(i)/(point_set.size()-1);
    point_set[i].z()=start_pt.z()+ratio*(local_target_pt.z()-start_pt.z());
  }
  point_set.front()=start_pt;point_set.back()=local_target_pt;
  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
  fixBoundary(ctrl_pts,ts,start_pt,local_target_pt,start_end_derivatives);
  const double search_resolution=grid_map_->getSearchResolution();
  const int xy=2*static_cast<int>(std::ceil((config_.planning_horizon+config_.body_length)/search_resolution))+9;
  const double center_z=(start_pt.z()+local_target_pt.z())/2.0;
  const double z_extent=std::max({std::abs(reference_min_z_-center_z),std::abs(reference_max_z_-center_z),std::abs(local_target_pt.z()-start_pt.z())/2.0});
  const double terrain_extent=std::max(config_.max_step_up,config_.max_step_down)+std::tan(config_.max_slope)*config_.resolution;
  const int z=2*static_cast<int>(std::ceil((z_extent+terrain_extent)/search_resolution))+9;
  bspline_optimizer_rebound_->a_star_.reset(new AStar);
  bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_,Eigen::Vector3i(xy,xy,z));
  // 已知碰撞的整段初始化先使用原 A*，避免末段障碍直到多轮反弹才进入搜索。
  if(initialization==Initialization::Automatic) {
    for(size_t i=1;i<point_set.size();++i) {
      const Eigen::Vector3d delta=point_set[i]-point_set[i-1];
      if(grid_map_->getInflateOccupancy(point_set[i],std::atan2(delta.y(),delta.x()))) {use_astar=true;break;}
    }
  }
  last_initialization_used_astar_=use_astar;
  if(use_astar) {
    // 原障碍段 A* 也用于退化初始化；沿原管理器的弧长重采样生成控制点。
    if(bspline_optimizer_rebound_->a_star_->AstarSearch(search_resolution,start_pt,local_target_pt,grid_map_->initialYaw())!=SUCCESS) {
      error="SCAN local A-star found no route through measured terrain";return false;
    }
    const auto path=bspline_optimizer_rebound_->a_star_->getPath();
    std::vector<Eigen::Vector3d> segment{start_pt};
    for(size_t i=1;i+1<path.size();++i) if((path[i]-segment.back()).norm()>1e-8) segment.push_back(path[i]);
    if((segment.back()-local_target_pt).norm()>1e-8) segment.push_back(local_target_pt);
    std::vector<double> length{0.0};
    for(size_t i=1;i<segment.size();++i) length.push_back(length.back()+(segment[i]-segment[i-1]).norm());
    const int intervals=std::max(8,static_cast<int>(std::ceil(length.back()/pp_.ctrl_pt_dist)));
    ts=length.back()/intervals/pp_.max_vel_*1.2;
    point_set.clear();
    size_t id=0;
    for(int sample=0;sample<=intervals;++sample) {
      const double arc=length.back()*sample/intervals;
      while(id+2<length.size()&&length[id+1]<arc) ++id;
      const double fraction=(arc-length[id])/(length[id+1]-length[id]);
      point_set.push_back(segment[id]*(1.0-fraction)+segment[id+1]*fraction);
    }
    UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
    fixBoundary(ctrl_pts,ts,start_pt,local_target_pt,start_end_derivatives);
  }
  // 先用原时间重分配满足运动限制，避免逐轴速度惩罚扭曲本来可通过窄门的斜直线。
  for(int iteration=0;!use_astar&&local_target_vel.norm()<1e-9&&iteration<5;++iteration) {
    UniformBspline initial(ctrl_pts,3,ts);
    initial.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,0.0);
    double ratio;
    if(initial.checkFeasibility(ratio,false)) break;
    double increment;
    reparamBspline(initial,start_end_derivatives,ratio*1.02,ctrl_pts,ts,increment);
  }
  bspline_optimizer_rebound_->initControlPoints(ctrl_pts,true);
  if(!bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts,ts)) {
    error="SCAN rebound optimization found no collision-free local trajectory";return false;
  }
  UniformBspline pos(ctrl_pts,3,ts);
  // 原时间重分配与 refine 优化，使用调用方真实速度/加速度上限。
  for(int iteration=0;iteration<5;++iteration) {
    pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,0.0);
    double ratio;
    if(pos.checkFeasibility(ratio,false)) break;
    Eigen::MatrixXd controls;
    if(!refineTrajAlgo(pos,start_end_derivatives,ratio*1.02,ts,controls)) {
      // 细化保留实测边界导数后可能挪动内部控制点，再调用原反弹优化恢复绕障。
      bspline_optimizer_rebound_->initControlPoints(controls,true);
      if(!bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(controls,ts)) {
        error="SCAN time refinement and rebound found no collision-free trajectory";return false;
      }
    }
    pos=UniformBspline(controls,3,ts);
  }
  trajectory_=pos;
  return true;
}

bool SCANPlannerManager::referenceReplan(Eigen::Vector3d start_pt,Eigen::Vector3d start_vel,
    Eigen::Vector3d start_acc,Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel) {
  // 实测起点接到全局参考投影点，再沿参考取到原局部目标；不移动任何路线点。
  std::vector<Eigen::Vector3d> segment{start_pt};
  const double step=std::max(0.01,pp_.ctrl_pt_dist/(2.0*pp_.max_vel_));
  for(double t=global_data_.projection_time_;t<global_data_.last_progress_time_;t+=step) {
    const Eigen::Vector3d p=global_data_.getPosition(t);
    if((p-segment.back()).norm()>1e-8) segment.push_back(p);
  }
  if((local_target_pt-segment.back()).norm()>1e-8) segment.push_back(local_target_pt);
  // 与原 reboundReplan 相同，竖向沿水平弧长从实测起点线性接到局部目标；实测地面仍由实际机身检查处理。
  std::vector<double> horizontal(segment.size(),0.0);
  for(size_t i=1;i<segment.size();++i) horizontal[i]=horizontal[i-1]+(segment[i].head<2>()-segment[i-1].head<2>()).norm();
  for(size_t i=0;i<segment.size();++i) {
    const double ratio=horizontal.back()>1e-6?horizontal[i]/horizontal.back():double(i)/std::max<size_t>(1,segment.size()-1);
    segment[i].z()=start_pt.z()+ratio*(local_target_pt.z()-start_pt.z());
  }
  // 实测起点与投影点同XY时竖向插值后重合，去掉零长段。
  segment.erase(std::unique(segment.begin(),segment.end(),
      [](const Eigen::Vector3d& a,const Eigen::Vector3d& b){return (a-b).norm()<=1e-8;}),segment.end());
  std::vector<double> length{0.0};
  for(size_t i=1;i<segment.size();++i) length.push_back(length.back()+(segment[i]-segment[i-1]).norm());
  const double total=length.back();
  if(total<1e-8) {error="local reference target coincides with the measured start";return false;}
  // 与原 reboundReplan 多项式初始化相同的时长、首末速度和首末加速度，只把起终点直线弧长换成参考弧长。
  double time=std::pow(pp_.max_vel_,2)/pp_.max_acc_>total
      ? 2.0*std::sqrt(total/pp_.max_acc_)
      : (total-std::pow(pp_.max_vel_,2)/pp_.max_acc_)/pp_.max_vel_+2*pp_.max_vel_/pp_.max_acc_;
  const Eigen::Vector3d direction=(segment[1]-segment[0]).normalized();
  auto profile=[&](double duration) {
    return PolynomialTraj::one_segment_traj_gen(Eigen::Vector3d::Zero(),Eigen::Vector3d(start_vel.norm(),0.0,0.0),
        Eigen::Vector3d(start_acc.dot(direction),0.0,0.0),Eigen::Vector3d(total,0.0,0.0),
        Eigen::Vector3d(local_target_vel.norm(),0.0,0.0),Eigen::Vector3d::Zero(),duration);
  };
  // 原时长按静止起停估计。没有避障优化再调整，按实际首末速度复核弧长速度/加速度上限，
  // 超限时延长并重新求解同一边界；不对拟合后的样条重分配，避免固定起点速度使时长反复放大。
  auto limit_ratio=[&](PolynomialTraj& candidate,double duration,bool& reverses) {
    double ratio=0.0;reverses=false;
    for(int i=0;i<=100;++i) {
      const double t=duration*i/100.0,v=candidate.evaluateVel(t).x();
      reverses=reverses||v<-1e-6;
      ratio=std::max({ratio,std::abs(v)/pp_.max_vel_,std::sqrt(std::abs(candidate.evaluateAcc(t).x())/pp_.max_acc_)});
    }
    return ratio;
  };
  PolynomialTraj arc=profile(time);
  for(int iteration=0;iteration<5;++iteration) {
    bool reverses;
    const double ratio=limit_ratio(arc,time,reverses);
    if(ratio<=1.0) break;
    PolynomialTraj longer=profile(time*ratio*1.02);
    limit_ratio(longer,time*ratio*1.02,reverses);
    if(reverses) break;
    arc=longer;time*=ratio*1.02;
  }
  auto at=[&](double s) {
    s=std::clamp(s,0.0,total);
    const size_t id=std::min<size_t>(std::upper_bound(length.begin(),length.end(),s)-length.begin(),length.size()-1)-1;
    const double fraction=std::clamp((s-length[id])/(length[id+1]-length[id]),0.0,1.0);
    return Eigen::Vector3d(segment[id]*(1.0-fraction)+segment[id+1]*fraction);
  };
  // 原控制点间距与采样规则。
  double ts=pp_.ctrl_pt_dist/pp_.max_vel_*1.2*1.5;
  std::vector<Eigen::Vector3d> point_set;
  bool flag_too_far;
  do {
    ts/=1.5;
    point_set.clear();
    flag_too_far=false;
    Eigen::Vector3d last_pt=at(0.0);
    for(double t=0;t<time;t+=ts) {
      const Eigen::Vector3d pt=at(arc.evaluate(t).x());
      if((last_pt-pt).norm()>pp_.ctrl_pt_dist*1.5) {flag_too_far=true;break;}
      last_pt=pt;
      point_set.push_back(pt);
    }
  } while(flag_too_far||point_set.size()<7);
  point_set.push_back(local_target_pt);
  std::vector<Eigen::Vector3d> start_end_derivatives{start_vel,local_target_vel,start_acc,Eigen::Vector3d::Zero()};
  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
  fixBoundary(ctrl_pts,ts,start_pt,local_target_pt,start_end_derivatives);
  trajectory_=UniformBspline(ctrl_pts,3,ts);
  return true;
}

bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj,std::vector<Eigen::Vector3d>& derivatives,
    double ratio,double& ts,Eigen::MatrixXd& optimal_control_points) {
  double t_inc;
  Eigen::MatrixXd ctrl_pts;
  reparamBspline(traj,derivatives,ratio,ctrl_pts,ts,t_inc);
  traj=UniformBspline(ctrl_pts,3,ts);
  const double t_step=traj.getTimeSum()/(ctrl_pts.cols()-3);
  bspline_optimizer_rebound_->ref_pts_.clear();
  for(double t=0;t<traj.getTimeSum()+1e-4;t+=t_step)
    bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
  return bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts,ts,optimal_control_points);
}

void SCANPlannerManager::reparamBspline(UniformBspline& bspline,std::vector<Eigen::Vector3d>& derivatives,
    double ratio,Eigen::MatrixXd& ctrl_pts,double& dt,double& time_inc) {
  const double time_origin=bspline.getTimeSum();
  const int seg_num=bspline.getControlPoint().cols()-3;
  const Eigen::Vector3d start=bspline.evaluateDeBoorT(0.0),end=bspline.evaluateDeBoorT(time_origin);
  bspline.lengthenTime(ratio);
  const double duration=bspline.getTimeSum();
  dt=duration/double(seg_num);
  time_inc=duration-time_origin;
  std::vector<Eigen::Vector3d> point_set;
  for(double t=0.0;t<=duration+1e-4;t+=dt) point_set.push_back(bspline.evaluateDeBoorT(t));
  UniformBspline::parameterizeToBspline(dt,point_set,derivatives,ctrl_pts);
  fixBoundary(ctrl_pts,dt,start,end,derivatives);
}
}
