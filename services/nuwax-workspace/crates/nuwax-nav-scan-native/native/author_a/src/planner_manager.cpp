#include <author_host.h>
#include <author_diagnostics.h>
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace scan_planner
{
  namespace
  {
    bool buildReferenceControls(const RouteHeightReference& route, const Eigen::Vector3d& start,
        const Eigen::Vector3d& target, int count, double resolution, double max_velocity, double max_acceleration,
        Eigen::MatrixXd& controls, std::vector<double>& control_arcs)
    {
      struct Anchor { Eigen::Vector3d point; double arc; };
      std::vector<Anchor> input{{start, route.begin_arc}};
      for (std::size_t i = 0; i < route.points.size(); ++i)
        if (route.arcs[i] > route.begin_arc && route.arcs[i] < route.end_arc)
          input.push_back({route.points[i], route.arcs[i]});
      input.push_back({target, route.end_arc});
      // 仅简化初始化参考，不修改原点云或完整参考，硬碰撞仍按原实体判定。
      std::vector<char> keep(input.size(), 0);
      keep.front() = keep.back() = 1;
      std::vector<std::pair<std::size_t, std::size_t>> pending{{0, input.size() - 1}};
      while (!pending.empty()) {
        const auto [first, last] = pending.back(); pending.pop_back();
        const Eigen::Vector3d delta = input[last].point - input[first].point;
        const double length2 = delta.squaredNorm();
        double maximum = 0.75 * resolution * resolution;
        std::size_t farthest = first;
        for (std::size_t i = first + 1; i < last; ++i) {
          const double fraction = length2 > 0.
              ? std::clamp((input[i].point - input[first].point).dot(delta) / length2, 0., 1.) : 0.;
          const double distance = (input[i].point - input[first].point - fraction * delta).squaredNorm();
          if (distance > maximum) { maximum = distance; farthest = i; }
        }
        if (farthest != first) {
          keep[farthest] = 1;
          pending.emplace_back(first, farthest); pending.emplace_back(farthest, last);
        }
      }
      std::vector<Anchor> anchors;
      for (std::size_t i = 0; i < input.size(); ++i) if (keep[i]) anchors.push_back(input[i]);
      if (3 * anchors.size() > static_cast<std::size_t>(count)) return false;
      std::vector<int> interior(anchors.size() - 1, 0);
      std::vector<double> lengths(interior.size()), times(interior.size()), peaks(interior.size());
      for (std::size_t i = 0; i < lengths.size(); ++i) {
        lengths[i] = (anchors[i + 1].point - anchors[i].point).norm();
        peaks[i] = std::min(max_velocity, std::sqrt(lengths[i] * max_acceleration));
        times[i] = lengths[i] > 0. ? lengths[i] / peaks[i] + peaks[i] / max_acceleration : 0.;
      }
      // 沿用原容量，按原能力下各段的最短静止到静止时间分配槽位。
      for (int left = count - static_cast<int>(3 * anchors.size()); left > 0; --left) {
        std::size_t selected = 0;
        for (std::size_t i = 1; i < lengths.size(); ++i)
          if (times[i] / (interior[i] + 1) > times[selected] / (interior[selected] + 1)) selected = i;
        ++interior[selected];
      }
      controls.resize(3, count); control_arcs.clear();
      int column = 0;
      for (std::size_t i = 0; i < anchors.size(); ++i) {
        for (int repeat = 0; repeat < 3; ++repeat) {
          controls.col(column++) = anchors[i].point; control_arcs.push_back(anchors[i].arc);
        }
        if (i + 1 == anchors.size()) break;
        for (int j = 1; j <= interior[i]; ++j) {
          const double t = times[i] * j / (interior[i] + 1);
          const double ramp = peaks[i] / max_acceleration;
          // 直段内先加速再制动，避免拐角重复点与匀速段之间的导数跳变。
          const double distance = t < ramp ? .5 * max_acceleration * t * t
              : t > times[i] - ramp ? lengths[i] - .5 * max_acceleration * std::pow(times[i] - t, 2)
              : peaks[i] * (t - .5 * ramp);
          const double fraction = lengths[i] > 0. ? distance / lengths[i] : 0.;
          const double arc = anchors[i].arc + fraction * (anchors[i + 1].arc - anchors[i].arc);
          Eigen::Vector3d point = anchors[i].point + fraction * (anchors[i + 1].point - anchors[i].point);
          point.z() = route.heightAtArc(arc);
          controls.col(column++) = point; control_arcs.push_back(arc);
        }
      }
      return true;
    }

    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cerr << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    refreshParameters(nh);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // Host lifecycle only: reuse original parameter reads without reallocating map or search pool.
  void SCANPlannerManager::refreshParameters(ros::NodeHandle &nh)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("optimization/vel_tolerance", pp_.vel_tolerance_, 1.0);
    nh.param("optimization/acc_tolerance", pp_.acc_tolerance_, 1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizon_, 5.0);

    if (bspline_optimizer_rebound_) bspline_optimizer_rebound_->setParam(nh);
  }

  void SCANPlannerManager::retireExecution()
  {
    route_height_.clear();
    if(bspline_optimizer_rebound_) {
      bspline_optimizer_rebound_->bind_route_segment_={};
      if(bspline_optimizer_rebound_->a_star_)bspline_optimizer_rebound_->a_star_->route_height_query_={};
    }
    local_data_ = LocalTrajData{};
    local_data_.start_pos_.setZero();
    global_data_.global_traj_ = PolynomialTraj{};
    global_data_.local_traj_.clear();
    global_data_.global_duration_ = 0.;
    global_data_.global_start_time_ = ros::Time{};
    global_data_.local_start_time_ = global_data_.local_end_time_ = -1.;
    global_data_.time_increase_ = global_data_.last_time_inc_ = global_data_.last_progress_time_ = 0.;
    continuous_failures_count_ = 0;
    flag_first_call = true;
    flag_force_polynomial = false;
    // Deliberately keep the original A* pool and rounds; each original search advances its round.
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    // 首试保留原多项式；第一次失败后在原重试槽使用完整参考，其余保留原历史/随机流程。
    const bool route_time_seed = !flag_first_call && flag_polyInit && local_data_.traj_id_ == 0
        && continuous_failures_count_ == 1 && route_height_.ready()
        && route_height_.end_time > route_height_.begin_time;
    const bool ordered_xyz_fallback = !route_time_seed && !flag_first_call && flag_polyInit && !flag_randomPolyTraj;
    // ROS node-local lifetime becomes native session lifetime.
    std::cerr << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    std::cerr.precision(3);
    std::cerr << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"rebound_input\",\"attempt\":"<<count-1<<",\"time\":"<<ros::Time::now().toSec()<<",\"poly_init\":"<<flag_polyInit<<",\"random_init\":"<<flag_randomPolyTraj<<",\"ordered_xyz_fallback\":"<<(ordered_xyz_fallback && route_height_.ready());
      out<<",\"route_time_seed\":"<<route_time_seed;
      out<<",\"start_p\":";traceMatrix(out,start_pt);out<<",\"start_v\":";traceMatrix(out,start_vel);out<<",\"start_a\":";traceMatrix(out,start_acc);
      out<<",\"target_p\":";traceMatrix(out,local_target_pt);out<<",\"target_v\":";traceMatrix(out,local_target_vel);out<<",\"route_height_ready\":"<<route_height_.ready()<<",\"route_begin_arc\":"<<route_height_.begin_arc<<",\"route_end_arc\":"<<route_height_.end_arc<<"}\n";
    }
    // 到达容差由原业务目标负责；非零短程仍须生成受约束轨迹，不能固定截断在20厘米。
    if ((start_pt - local_target_pt).squaredNorm() == 0.)
    {
      std::cerr << "Zero translation to local target" << endl;
      continuous_failures_count_++;
      return false;
    }

    ros::WallTime t_start = ros::WallTime::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    std::vector<double> timed_seed_arcs;
    // Initialization flags are members of this persistent manager.
    bool flag_regenerate = false;
    if (route_time_seed)
    {
      // 使用既有完整参考的时标，不把拐角重试再次压成起终点直线。
      const double span = route_height_.end_time - route_height_.begin_time;
      const int capacity = std::max(7, static_cast<int>(std::floor(pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3)));
      const double detail_dt = std::min(ts, grid_map_->getResolution() / pp_.max_vel_);
      const int intervals = std::clamp(static_cast<int>(std::ceil(span / detail_dt)), 6, capacity - 1);
      ts = span / intervals;
      for (int i = 0; i <= intervals; ++i) {
        const double arc = route_height_.arcAtTime(route_height_.begin_time + i * ts);
        timed_seed_arcs.push_back(arc);
        point_set.push_back(route_height_.positionAtArc(arc));
      }
      point_set.front() = start_pt;
      point_set.back() = local_target_pt;
      start_end_derivatives = {start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero()};
    }
    else do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    std::vector<double> height_sample_arcs;
    if(route_height_.ready()) {
      if (route_time_seed) {
        height_sample_arcs = std::move(timed_seed_arcs);
      } else {
      height_sample_arcs.assign(point_set.size(),0.);
      for(size_t i=1;i<point_set.size();++i)
        height_sample_arcs[i]=height_sample_arcs[i-1]+(point_set[i].head<2>()-point_set[i-1].head<2>()).norm();
      const double length=height_sample_arcs.back();
      for(size_t i=0;i<point_set.size();++i){
        const double fraction=length>0.?height_sample_arcs[i]/length:double(i)/double(point_set.size()-1);
        height_sample_arcs[i]=route_height_.begin_arc+fraction*(route_height_.end_arc-route_height_.begin_arc);
        if(ordered_xyz_fallback && i>0 && i+1<point_set.size())
          point_set[i]=route_height_.positionAtArc(height_sample_arcs[i]);
        else
          point_set[i].z()=route_height_.heightAtArc(height_sample_arcs[i]);
      }
      }
      // Retain requested boundary positions and the original LS derivatives.
      point_set.front().z()=start_pt.z();point_set.back().z()=local_target_pt.z();
    } else {
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
      bspline_optimizer_rebound_->a_star_->route_height_query_={};
      bspline_optimizer_rebound_->bind_route_segment_={};
    }

    Eigen::MatrixXd ctrl_pts;
    std::vector<double> seed_control_arcs;
    if (route_time_seed) {
      // 控制多边形保持原折线的单调段，避免插值在窄通道拐角产生反向摆动。
      const int size = static_cast<int>(point_set.size()) + 2;
      if (!buildReferenceControls(route_height_, start_pt, local_target_pt, size,
          grid_map_->getResolution(), pp_.max_vel_, pp_.max_acc_, ctrl_pts, seed_control_arcs)) {
        ROS_WARN("Reference corners exceed the existing control-point capacity.");
        ++continuous_failures_count_;
        return false;
      }
      const double square_dt = ts * ts;
      // 三次样条端点的解析位置/速度/加速度约束，使用原实际运动边界。
      ctrl_pts.col(0) = start_pt - start_vel * ts + start_acc * (square_dt / 3.);
      ctrl_pts.col(1) = start_pt - start_acc * (square_dt / 6.);
      ctrl_pts.col(2) = start_pt + start_vel * ts + start_acc * (square_dt / 3.);
      ctrl_pts.col(size - 3) = local_target_pt - local_target_vel * ts;
      ctrl_pts.col(size - 2) = local_target_pt;
      ctrl_pts.col(size - 1) = local_target_pt + local_target_vel * ts;
    } else {
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    }
    if(!height_sample_arcs.empty()) {
      // Cubic Greville index j corresponds to sample j-1; boundary extensions
      // keep the endpoint arc. This mapping stays fixed throughout the solve.
      std::vector<double> cp_arcs = seed_control_arcs;
      if (cp_arcs.empty()) {
        cp_arcs.resize(ctrl_pts.cols());
        for(int j=0;j<ctrl_pts.cols();++j)
          cp_arcs[j]=height_sample_arcs[std::clamp(j-1,0,int(height_sample_arcs.size())-1)];
      }
      bspline_optimizer_rebound_->bind_route_segment_=[this,cp_arcs](int first,int last){
        const double a=cp_arcs.at(first),b=cp_arcs.at(last);
        bspline_optimizer_rebound_->a_star_->route_height_query_=[this,a,b](double ratio){
          return route_height_.heightAtArc(a+ratio*(b-a));
        };
      };
    }


    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"initial_control_points\",\"attempt\":"<<count-1<<",\"ts\":"<<ts<<",\"points\":";traceMatrix(out,ctrl_pts);out<<"}\n";
    }
    vector<vector<Eigen::Vector3d>> a_star_paths;
    const Eigen::MatrixXd reference_controls = route_time_seed ? ctrl_pts : Eigen::MatrixXd{};
    const double reference_dt = ts;
    bool reference_free = false;
    if (route_time_seed) {
      UniformBspline reference_curve(reference_controls, 3, reference_dt);
      reference_free = trajectoryCollisionFree(reference_curve);
    }
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::WallTime::now() - t_start;

    // Visualization counter has session scope.
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = ros::WallTime::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"rebound_result\",\"attempt\":"<<count-1<<",\"success\":"<<flag_step_1_success<<",\"ts\":"<<ts<<",\"points\":";traceMatrix(out,ctrl_pts);out<<",\"search_paths\":[";
      for(size_t i=0;i<a_star_paths.size();++i){if(i)out<<',';out<<'[';for(size_t j=0;j<a_star_paths[i].size();++j){if(j)out<<',';traceMatrix(out,a_star_paths[i][j]);}out<<']';}out<<"]}\n";
    }
    std::cerr << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success && !reference_free)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    if (!flag_step_1_success) ctrl_pts = reference_controls;
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::WallTime::now() - t_start;
    t_start = ros::WallTime::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      std::cerr << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    bool accepted = flag_step_2_success && checkDynamicFeasibility(pos)
        && (!route_time_seed || (trajectoryCollisionFree(pos) && strictDynamicScale(pos) <= 1.));
    if (!accepted && reference_free && start_vel.isZero(0.) && start_acc.isZero(0.)
        && local_target_vel.isZero(0.)) {
      // 零边界下只缩放时间，位置几何、端点和零导数保持；上界来自原控制多边形。
      double scale = 1.;
      for (int i = 0; i + 1 < reference_controls.cols(); ++i)
        scale = std::max(scale, (reference_controls.col(i + 1) - reference_controls.col(i)).norm()
            / (reference_dt * pp_.max_vel_));
      for (int i = 0; i + 2 < reference_controls.cols(); ++i)
        scale = std::max(scale, std::sqrt((reference_controls.col(i + 2) - 2. * reference_controls.col(i + 1)
            + reference_controls.col(i)).norm() / (reference_dt * reference_dt * pp_.max_acc_)));
      // 向保守方向覆盖浮点差分舍入，不放宽最终速度/加速度上限比较。
      scale = std::nextafter(scale * (1. + std::numeric_limits<double>::epsilon()
          * (reference_controls.cols() + 4)), std::numeric_limits<double>::infinity());
      UniformBspline retained(reference_controls, 3, reference_dt * scale);
      if (trajectoryCollisionFree(retained) && checkDynamicFeasibility(retained)
          && strictDynamicScale(retained) <= 1.) {
        pos = std::move(retained);
        accepted = true;
        if (authorTraceEnabled()) {
          auto out = authorTraceStream();
          out << "{\"event\":\"retained_reference_seed\",\"scale\":" << scale
              << ",\"duration\":" << pos.getTimeSum() << "}\n";
        }
      }
    }
    if (!accepted)
    {
      std::fprintf(stderr,"\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = ros::WallTime::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    std::cerr << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    // The original FSM passes the first waypoint separately as start_pos.
    const std::vector<double>& point_arcs=route_height_.waypoint_arcs;
    const bool have_height=point_arcs.size()==waypoints.size()+1;

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    std::vector<double> inter_arcs;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      if(have_height)inter_arcs.push_back(point_arcs[i]);
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
          if(have_height)inter_arcs.push_back(point_arcs[i]+double(j)/id_num*(point_arcs[i+1]-point_arcs[i]));
        }
      }
    }

    inter_points.push_back(points.back());
    if(have_height)inter_arcs.push_back(point_arcs.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   std::cerr << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    if(have_height){
      route_height_.times.assign(1,0.);route_height_.timed_arcs=inter_arcs;
      for(int i=0;i<time.size();++i)route_height_.times.push_back(route_height_.times.back()+time(i));
    }
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cerr << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"published_spline\",\"attempt\":"<<count-1<<",\"time\":"<<time_now.toSec()<<",\"duration\":"<<local_data_.duration_<<",\"control_points\":";
      traceMatrix(out,local_data_.position_traj_.getControlPoint());out<<",\"knots\":";traceMatrix(out,local_data_.position_traj_.getKnot());out<<"}\n";
    }
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << tc << ", |v|=" << vel.norm() << " > " << vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << tc << ", |a|=" << acc.norm() << " > " << acc_limit);
        return false;
      }
    }

    return true;
  }

  bool SCANPlannerManager::trajectoryCollisionFree(UniformBspline& trajectory)
  {
    const double duration = trajectory.getTimeSum();
    const double dt = std::min(.01, grid_map_->getResolution() / (4. * pp_.max_vel_));
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 0; i <= samples; ++i) {
      const double time = duration * i / samples;
      const Eigen::Vector3d point = trajectory.evaluateDeBoorT(time);
      const Eigen::Vector3d next = trajectory.evaluateDeBoorT(std::min(duration, time + dt));
      const Eigen::Vector2d direction = (next - point).head<2>();
      const double yaw = direction.squaredNorm() < 1e-8 ? 0. : std::atan2(direction.y(), direction.x());
      if (grid_map_->getInflateOccupancy(point, yaw) != 0) return false;
    }
    return true;
  }

  double SCANPlannerManager::strictDynamicScale(UniformBspline& trajectory)
  {
    auto velocity = trajectory.getDerivative();
    auto acceleration = velocity.getDerivative();
    // 导数样条位于其控制多边形凸包内，这些上界覆盖整个时间段。
    return std::max(velocity.getControlPoint().colwise().norm().maxCoeff() / pp_.max_vel_,
        std::sqrt(acceleration.getControlPoint().colwise().norm().maxCoeff() / pp_.max_acc_));
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner

