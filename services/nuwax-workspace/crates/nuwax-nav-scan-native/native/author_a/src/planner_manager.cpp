#include <author_host.h>
#include <author_diagnostics.h>
#include <trajectory_heading.h>
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace scan_planner
{
  namespace
  {
    // 与正时距域求交：常数项非负，负二次项只有一个非负出口。
    bool intersectTimeBound(double a, double b, double c, double& lower, double& upper)
    {
      if (a == 0.) {
        if (b < 0.) lower = std::max(lower, -c / b);
        else if (b > 0.) upper = std::min(upper, -c / b);
        else if (c > 0.) return false;
      } else {
        const double discriminant = b * b - 4. * a * c;
        if (discriminant < 0.) return false;
        const double root = std::sqrt(discriminant);
        if (a < 0.) lower = std::max(lower, b < 0. ? 2. * c / (root - b) : (b + root) / (-2. * a));
        else {
          const double right = (-b + root) / (2. * a);
          if (!(right > 0.)) return false;
          lower = std::max(lower, c / (a * right));
          upper = std::min(upper, right);
        }
      }
      return upper > 0. && lower <= upper;
    }

    bool retimeReferenceSegment(Eigen::MatrixXd& controls, const std::vector<double>& arcs, double& dt,
        const Eigen::Vector3d& start, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& target, const Eigen::Vector3d& target_velocity,
        double max_velocity, double max_acceleration, double time_origin)
    {
      const int count = controls.cols();
      Eigen::MatrixXd positions = controls;
      Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3, count);
      Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(3, count);
      // 时距变化时重建真实边界；直接整体放慢会篡改非零实测导数。
      for (int i = 0; i < 3; ++i) {
        positions.col(i) = start;
        positions.col(count - 3 + i) = target;
      }
      velocities.col(0) = -velocity; velocities.col(2) = velocity;
      velocities.col(count - 3) = -target_velocity; velocities.col(count - 1) = target_velocity;
      accelerations.col(0) = accelerations.col(2) = acceleration / 3.;
      accelerations.col(1) = -acceleration / 6.;
      double first_corner = arcs.back(), last_corner = arcs.front();
      for (int i = 3; i + 2 < count - 3; ++i) {
        if (arcs[i] == arcs[i + 1] && arcs[i] == arcs[i + 2]) {
          first_corner = std::min(first_corner, arcs[i]);
          last_corner = std::max(last_corner, arcs[i]);
        }
      }
      // 非零边界沿首末原直段衰减；不能把前进中的端点骤接到静止初值而产生回头。
      for (int i = 3; i < count - 3; ++i) {
        const double from = first_corner > arcs.front()
            ? std::clamp((first_corner - arcs[i]) / (first_corner - arcs.front()), 0., 1.) : 0.;
        const double to = arcs.back() > last_corner
            ? std::clamp((arcs[i] - last_corner) / (arcs.back() - last_corner), 0., 1.) : 0.;
        velocities.col(i) = from * velocity - to * target_velocity;
        accelerations.col(i) = from * acceleration / 3.;
      }
      double lower = dt, upper = std::numeric_limits<double>::infinity();
      for (int i = 0; i + 1 < count; ++i) {
        const Eigen::Vector3d p = positions.col(i + 1) - positions.col(i);
        const Eigen::Vector3d v = velocities.col(i + 1) - velocities.col(i);
        const Eigen::Vector3d a = accelerations.col(i + 1) - accelerations.col(i);
        // 无加速度项时精确求速度范数；其余用三角不等式给出保守充分界。
        const bool feasible = a.isZero(0.)
            ? intersectTimeBound(v.squaredNorm() - max_velocity * max_velocity,
                2. * p.dot(v), p.squaredNorm(), lower, upper)
            : intersectTimeBound(a.norm(), v.norm() - max_velocity, p.norm(), lower, upper);
        if (!feasible) return false;
      }
      for (int i = 0; i + 2 < count; ++i) {
        const Eigen::Vector3d p = positions.col(i + 2) - 2. * positions.col(i + 1) + positions.col(i);
        const Eigen::Vector3d v = velocities.col(i + 2) - 2. * velocities.col(i + 1) + velocities.col(i);
        const Eigen::Vector3d a = accelerations.col(i + 2) - 2. * accelerations.col(i + 1) + accelerations.col(i);
        if (!intersectTimeBound(a.norm() - max_acceleration, v.norm(), p.norm(), lower, upper)) return false;
      }
      // 导数分母要从累计结点时间相减；按累计时间/本段时距计入这四次差分的舍入量。
      // 最终仍按原严格上限核验，不放宽检查，也不增加求解重试。
      dt = std::nextafter(lower * (1. + std::numeric_limits<double>::epsilon()
          * (count + 4 + 4. * time_origin / lower)),
          std::numeric_limits<double>::infinity());
      if (!std::isfinite(dt) || dt > upper) return false;
      controls = positions + velocities * dt + accelerations * (dt * dt);
      for (int i = 0; i + 1 < count; ++i) {
        const Eigen::Vector3d direction = positions.col(i + 1) - positions.col(i);
        if ((controls.col(i + 1) - controls.col(i)).dot(direction) < 0.) return false;
      }
      return controls.allFinite();
    }

    bool retimeReferenceControls(Eigen::MatrixXd& controls, const std::vector<double>& arcs,
        Eigen::VectorXd& knots, const Eigen::Vector3d& start, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& target,
        const Eigen::Vector3d& target_velocity, double max_velocity, double max_acceleration)
    {
      const int count = controls.cols();
      std::vector<int> stops{0};
      for (int i = 3; i < count - 3; ++i)
        if (arcs[i] == arcs[i + 1] && arcs[i] == arcs[i + 2]) stops.push_back(i);
      stops.push_back(count - 3);
      knots = Eigen::VectorXd::Zero(count + 4);
      double first_dt = 0., last_dt = 0.;
      for (std::size_t segment = 1; segment < stops.size(); ++segment) {
        const int first = stops[segment - 1], last = stops[segment];
        Eigen::MatrixXd part = controls.middleCols(first, last - first + 3);
        std::vector<double> part_arcs(arcs.begin() + first, arcs.begin() + last + 3);
        double dt = 0.;
        if (!retimeReferenceSegment(part, part_arcs, dt,
            segment == 1 ? start : Eigen::Vector3d(part.col(1)),
            segment == 1 ? velocity : Eigen::Vector3d::Zero().eval(),
            segment == 1 ? acceleration : Eigen::Vector3d::Zero().eval(),
            segment + 1 == stops.size() ? target : Eigen::Vector3d(part.col(part.cols() - 2)),
            segment + 1 == stops.size() ? target_velocity : Eigen::Vector3d::Zero().eval(),
            max_velocity, max_acceleration, knots[first + 3])) return false;
        controls.middleCols(first, part.cols()) = part;
        for (int i = first + 3; i < last + 3; ++i) knots[i + 1] = knots[i] + dt;
        if (segment == 1) first_dt = dt;
        last_dt = dt;
        if (authorTraceEnabled()) {
          auto out = authorTraceStream();
          out << "{\"event\":\"reference_segment_timing\",\"first\":" << first
              << ",\"last\":" << last << ",\"dt\":" << dt
              << ",\"duration\":" << (last - first) * dt << "}\n";
        }
      }
      for (int i = 0; i < 3; ++i) knots[i] = (i - 3) * first_dt;
      for (int i = count + 1; i < count + 4; ++i) knots[i] = knots[count] + (i - count) * last_dt;
      return true;
    }

    void buildReferenceControls(RouteHeightReference& route, const Eigen::Vector3d& start,
        Eigen::Vector3d& target, int count, double resolution, double max_velocity, double max_acceleration,
        GridMap& map, Eigen::MatrixXd& controls, std::vector<double>& control_arcs)
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
      std::vector<std::pair<std::size_t, std::size_t>> pending{{0,input.size()-1}};
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
        // 亚体素转折也可能提供必要净空。简化捷径若与当前实体相交，保留最偏离
        // 捷径的原点再分段；空旷段仍沿原容量简化。这里只选初值，完整曲线与命令
        // 继续经过原碰撞/动态/连续制动门禁，采样未命中不构成运动许可。
        if (farthest == first && last > first+1) {
          const double yaw=std::atan2(delta.y(),delta.x());
          const int steps=std::max(1,static_cast<int>(std::ceil(delta.norm()/(resolution*.25))));
          const double margin=std::max(.05,resolution);
          double reference_clearance=margin;
          // 宽处保留目标余量；已知窄段沿原硬可行规则简化，由最小净距软代价选择可留余量。
          for(int j=0;j<=steps;++j) {
            const double arc=input[first].arc+double(j)/steps*(input[last].arc-input[first].arc);
            const Eigen::Vector3d before=route.positionAtArc(std::max(input[first].arc,arc-resolution*.25));
            const Eigen::Vector3d after=route.positionAtArc(std::min(input[last].arc,arc+resolution*.25));
            const double heading=std::atan2(after.y()-before.y(),after.x()-before.x());
            if(map.bodyClearance(route.positionAtArc(arc),heading,margin)<margin) {
              reference_clearance=0.;
              break;
            }
          }
          for (int j=0;j<=steps;++j) {
            const double fraction=double(j)/steps;
            Eigen::Vector3d point=input[first].point+fraction*delta;
            const double arc=input[first].arc+fraction*(input[last].arc-input[first].arc);
            point.z()=route.heightAtArc(arc)
                +(1.-fraction)*(input[first].point.z()-route.heightAtArc(input[first].arc))
                +fraction*(input[last].point.z()-route.heightAtArc(input[last].arc));
            if (map.getInflateOccupancy(point,yaw)==0
                && map.bodyClearance(point,yaw,margin)>=reference_clearance)continue;
            double deviation=-1.;
            for (std::size_t i=first+1;i<last;++i) {
              const double along=length2>0.?std::clamp((input[i].point-input[first].point).dot(delta)/length2,0.,1.):0.;
              const double distance=(input[i].point-input[first].point-along*delta).squaredNorm();
              if (distance>deviation){deviation=distance;farthest=i;}
            }
            break;
          }
        }
        if (farthest != first) {
          keep[farthest] = 1;
          pending.emplace_back(first, farthest); pending.emplace_back(farthest, last);
        }
      }
      std::vector<Anchor> anchors;
      for (std::size_t i = 0; i < input.size(); ++i) if (keep[i]) anchors.push_back(input[i]);
      if (3 * anchors.size() > static_cast<std::size_t>(count)) {
        // 容量只覆盖本次连续参考前缀；保留必要转折，不跨障碍删点。
        anchors.resize(count / 3);
        // 末端落在最后一段内部，避免拐角处的出段速度成为入段边界。
        anchors.back().arc = (anchors[anchors.size() - 2].arc + anchors.back().arc) / 2.;
        anchors.back().point = route.positionAtArc(anchors.back().arc);
        target = anchors.back().point;
        route.end_arc = anchors.back().arc;
        for (std::size_t i = 1; i < route.timed_arcs.size(); ++i) {
          if (route.end_arc > route.timed_arcs[i]) continue;
          const double fraction = (route.end_arc - route.timed_arcs[i - 1])
              / (route.timed_arcs[i] - route.timed_arcs[i - 1]);
          route.end_time = route.times[i - 1] + fraction * (route.times[i] - route.times[i - 1]);
          break;
        }
      }
      std::vector<int> repeats(anchors.size(), 3);
      std::vector<double> lengths(anchors.size() - 1), speeds(anchors.size(), 0.);
      for (std::size_t i = 0; i < lengths.size(); ++i)
        lengths[i] = (anchors[i + 1].point - anchors[i].point).norm();
      for (std::size_t i = 1; i + 1 < anchors.size(); ++i) {
        const Eigen::Vector3d incoming = (anchors[i].point - anchors[i - 1].point).normalized();
        const Eigen::Vector3d outgoing = (anchors[i + 1].point - anchors[i].point).normalized();
        // 在半个相邻段、且不超过制动距离的邻域内完成速度转向：
        // |Δv|=2v sin(θ/2)，可用时间=邻域长度/v，要求 |Δv|/Δt≤a。
        const double blend = std::min({lengths[i - 1] / 2., lengths[i] / 2.,
            max_velocity * max_velocity / (2. * max_acceleration)});
        const double sine_limit = max_acceleration * blend / (2. * max_velocity * max_velocity);
        if ((incoming - outgoing).norm() / 2. < sine_limit) {
          repeats[i] = 1;
          speeds[i] = max_velocity;
        }
      }
      // 连续通过点的速度同时受前后停车距离约束。
      for (std::size_t i = 1; i < speeds.size(); ++i)
        speeds[i] = std::min(speeds[i], std::sqrt(speeds[i - 1] * speeds[i - 1] + 2. * max_acceleration * lengths[i - 1]));
      for (std::size_t i = speeds.size() - 1; i > 0; --i)
        speeds[i - 1] = std::min(speeds[i - 1], std::sqrt(speeds[i] * speeds[i] + 2. * max_acceleration * lengths[i - 1]));
      std::vector<int> interior(lengths.size(), 0);
      std::vector<double> times(lengths.size()), peaks(lengths.size());
      for (std::size_t i = 0; i < lengths.size(); ++i) {
        peaks[i] = std::min(max_velocity, std::sqrt(lengths[i] * max_acceleration
            + (speeds[i] * speeds[i] + speeds[i + 1] * speeds[i + 1]) / 2.));
        times[i] = lengths[i] > 0. ? lengths[i] / peaks[i]
            + ((peaks[i] - speeds[i]) * (peaks[i] - speeds[i])
               + (peaks[i] - speeds[i + 1]) * (peaks[i] - speeds[i + 1])) / (2. * max_acceleration * peaks[i]) : 0.;
      }
      // 容量缩短仍按每锚点三槽计算；省下的小转角槽用于各段内部采样。
      const int anchor_slots = std::accumulate(repeats.begin(), repeats.end(), 0);
      for (int left = count - anchor_slots; left > 0; --left) {
        std::size_t selected = 0;
        for (std::size_t i = 1; i < lengths.size(); ++i)
          if (times[i] / (interior[i] + 1) > times[selected] / (interior[selected] + 1)) selected = i;
        ++interior[selected];
      }
      controls.resize(3, count); control_arcs.clear();
      int column = 0;
      for (std::size_t i = 0; i < anchors.size(); ++i) {
        for (int repeat = 0; repeat < repeats[i]; ++repeat) {
          controls.col(column++) = anchors[i].point; control_arcs.push_back(anchors[i].arc);
        }
        if (i + 1 == anchors.size()) break;
        for (int j = 1; j <= interior[i]; ++j) {
          const double t = times[i] * j / (interior[i] + 1);
          const double ramp = (peaks[i] - speeds[i]) / max_acceleration;
          const double brake = (peaks[i] - speeds[i + 1]) / max_acceleration;
          const double remaining = times[i] - t;
          const double distance = t < ramp ? speeds[i] * t + .5 * max_acceleration * t * t
              : remaining < brake ? lengths[i] - speeds[i + 1] * remaining - .5 * max_acceleration * remaining * remaining
              : speeds[i] * ramp + .5 * max_acceleration * ramp * ramp + peaks[i] * (t - ramp);
          const double fraction = lengths[i] > 0. ? distance / lengths[i] : 0.;
          const double arc = anchors[i].arc + fraction * (anchors[i + 1].arc - anchors[i].arc);
          Eigen::Vector3d point = anchors[i].point + fraction * (anchors[i + 1].point - anchors[i].point);
          // 实测端点可能偏离参考高程；连续吸收这一边界差，不能在首个内部点突然跳回旧高度。
          const double from_offset = anchors[i].point.z() - route.heightAtArc(anchors[i].arc);
          const double to_offset = anchors[i + 1].point.z() - route.heightAtArc(anchors[i + 1].arc);
          point.z() = route.heightAtArc(arc) + (1. - fraction) * from_offset + fraction * to_offset;
          controls.col(column++) = point; control_arcs.push_back(arc);
        }
      }
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

  void SCANPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis, GridMap::Ptr map)
  {
    refreshParameters(nh);

    local_data_.traj_id_ = 0;
    grid_map_ = std::move(map);

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
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                        double start_yaw)
  {

    const double resolution = grid_map_->getResolution();
    bspline_optimizer_rebound_->setInitialClearance(grid_map_->bodyClearance(
        start_pt,start_yaw,2.*std::max(.05,resolution)));
    bool reference_detour = false;
    if (route_height_.ready()) {
      const Eigen::Vector2d chord = (local_target_pt - start_pt).head<2>();
      for (size_t i = 0; i < route_height_.points.size(); ++i) {
        if (route_height_.arcs[i] <= route_height_.begin_arc || route_height_.arcs[i] >= route_height_.end_arc) continue;
        const Eigen::Vector2d offset = (route_height_.points[i] - start_pt).head<2>();
        const double fraction = chord.squaredNorm() > 0.
            ? std::clamp(offset.dot(chord) / chord.squaredNorm(), 0., 1.) : 0.;
        if ((offset - fraction * chord).squaredNorm() > resolution * resolution) {
          reference_detour = true;
          break;
        }
      }
    }
    bool reanchor = false;
    if (local_data_.traj_id_ > 0) {
      const double t = std::clamp((ros::Time::now() - local_data_.start_time_).toSec(), 0., local_data_.duration_);
      reanchor = (local_data_.position_traj_.evaluateDeBoorT(t) - start_pt).norm() > resolution;
    }
    // 可分辨的绕行不能退化为首尾初值；机体落后时也不能继续旧计划位置。
    // 普通直线及亚体素过渡仍保留原初始化，仅改变当前窗口确需重建的几何来源。
    const bool original_retry_seed = !flag_first_call && flag_polyInit && local_data_.traj_id_ == 0
        && continuous_failures_count_ == 1;
    const bool route_time_seed = (reference_detour || reanchor || original_retry_seed) && route_height_.ready()
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
    Eigen::MatrixXd ctrl_pts;
    std::vector<double> seed_control_arcs;
    // Initialization flags are members of this persistent manager.
    bool flag_regenerate = false;
    bool stationary_short_seed = false;
    if (route_time_seed)
    {
      flag_first_call = false;
      flag_force_polynomial = false;
      // 使用既有完整参考的时标，不把拐角重试再次压成起终点直线。
      double span = route_height_.end_time - route_height_.begin_time;
      const int capacity = std::max(7, static_cast<int>(std::floor(pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3)));
      const double detail_dt = std::min(ts, grid_map_->getResolution() / pp_.max_vel_);
      const int intervals = std::clamp(static_cast<int>(std::ceil(span / detail_dt)), 6, capacity - 1);
      const double original_end_arc = route_height_.end_arc;
      buildReferenceControls(route_height_, start_pt, local_target_pt, intervals + 3,
          resolution, pp_.max_vel_, pp_.max_acc_, *grid_map_, ctrl_pts, seed_control_arcs);
      if (route_height_.end_arc < original_end_arc) {
        span = route_height_.end_time - route_height_.begin_time;
        global_data_.last_progress_time_ = route_height_.end_time;
        // 与原局部目标相同的制动距离及速度边界，位置和时标取缩短后的参考。
        local_target_vel = (route_height_.points.back() - local_target_pt).norm()
                < pp_.max_vel_ * pp_.max_vel_ / (2. * pp_.max_acc_)
            ? Eigen::Vector3d::Zero().eval() : route_height_.velocityAtTime(route_height_.end_time);
        if (local_target_vel.norm() > pp_.max_vel_)
          local_target_vel = local_target_vel.normalized() * pp_.max_vel_;
      }
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
        stationary_short_seed = dist < pp_.max_vel_ * pp_.max_vel_ / pp_.max_acc_
            && start_vel.isZero(0.) && start_acc.isZero(0.) && local_target_vel.isZero(0.)
            && !flag_randomPolyTraj;
        // 先生成原五次曲线的几何采样；静止短程在拟合后按实际三次样条重新定时。
        const double short_time = std::max(5. * dist / pp_.max_vel_, sqrt(20. * dist / pp_.max_acc_));
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? short_time : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

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
        // 实际机体未到目标时，旧曲线仍可能已经到期；空历史必须按原分支重建初值。
        if (!std::isfinite(t_cur) || t_cur < 0. || !(t_cur < local_data_.duration_)) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
        }
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

        if (pseudo_arc_length.size() < 2 || !std::isfinite(pseudo_arc_length.back())
            || !(pseudo_arc_length.back() > 0.)) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
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
            // 沿用原历史种子容量；在扩张向量前判定，避免坏历史耗尽内存或阻塞退出。
            if (point_set.size() >= pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) {
              flag_force_polynomial = flag_regenerate = true;
              break;
            }
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          if (flag_regenerate) break;
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        if (flag_regenerate) continue;

        // 点距为满足原最少点数而缩小时，时距同步缩小，保留原计划速度而非拉长短尾段。
        ts *= cps_dist / pp_.ctrl_pt_dist;
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
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
        const double from_offset = start_pt.z() - route_height_.heightAtArc(route_height_.begin_arc);
        const double to_offset = local_target_pt.z() - route_height_.heightAtArc(route_height_.end_arc);
        point_set[i].z() += (1. - fraction) * from_offset + fraction * to_offset;
      }
      }
      // Retain requested boundary positions and the original LS derivatives.
      point_set.front().z()=start_pt.z();point_set.back().z()=local_target_pt.z();
    } else {
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
      bspline_optimizer_rebound_->a_star_->route_height_query_={};
      bspline_optimizer_rebound_->bind_route_segment_={};
    }

    if (!(ts > 0.) || !std::isfinite(ts)
        || !std::all_of(point_set.begin(), point_set.end(), [](const auto& p){return p.allFinite();})
        || !std::all_of(start_end_derivatives.begin(), start_end_derivatives.end(), [](const auto& p){return p.allFinite();})) {
      ROS_WARN("局部轨迹初值包含无效数值，等待重新规划。");
      ++continuous_failures_count_;
      return false;
    }
    if (route_time_seed) {
      // 控制多边形保持原折线的单调段，避免插值在窄通道拐角产生反向摆动。
      const int size = static_cast<int>(point_set.size()) + 2;
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
      if (stationary_short_seed) {
        UniformBspline seed(ctrl_pts, 3, ts);
        // 固定拟合后的控制点，以导数控制点三维范数求共同最小时距。
        // 一阶导数随时距反比、二阶导数随平方反比；覆盖连续时间及原逐轴/采样检查。
        ts *= strictDynamicScale(seed)
            * (1. + std::numeric_limits<double>::epsilon() * (ctrl_pts.cols() + 4));
      }
    }

    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"initial_control_points\",\"attempt\":"<<count-1<<",\"ts\":"<<ts<<",\"points\":";traceMatrix(out,ctrl_pts);out<<"}\n";
    }
    vector<vector<Eigen::Vector3d>> a_star_paths;
    Eigen::MatrixXd reference_controls = route_time_seed ? ctrl_pts : Eigen::MatrixXd{};
    const double reference_dt = ts;
    Eigen::VectorXd reference_knots;
    bool reference_free = false;
    if (route_time_seed) {
      UniformBspline reference_curve(reference_controls, 3, reference_dt);
      reference_free = trajectoryCollisionFree(reference_curve);
    }
    // 原参考本身已经可行时先交付其严格证书，避免优化器不断切掉必要拐角后
    // 才回退到同一参考，导致计算超过原命令交付窗。实测边界和所有门禁仍相同。
    const bool history_executable = local_data_.traj_id_ > 0
        && (ros::Time::now() - local_data_.start_time_).toSec() < local_data_.duration_;
    const bool retain_boundary = !history_executable
        || (start_vel.isZero(0.) && start_acc.isZero(0.) && local_target_vel.isZero(0.));
    if (reference_free && retain_boundary) {
      Eigen::MatrixXd retained_controls = reference_controls;
      Eigen::VectorXd retained_knots;
      if (retimeReferenceControls(retained_controls, seed_control_arcs, retained_knots, start_pt, start_vel, start_acc,
          local_target_pt, local_target_vel, pp_.max_vel_, pp_.max_acc_)) {
        UniformBspline retained(retained_controls, 3, reference_dt);
        retained.setKnot(retained_knots);
        if (trajectoryCollisionFree(retained) && checkDynamicFeasibility(retained)
            && strictDynamicScale(retained) <= 1.) {
          if (trajectoryHasPreferredClearance(retained)) {
            if (authorTraceEnabled()) {
              auto out = authorTraceStream();
              out << "{\"event\":\"retained_reference_seed\",\"scale\":" << retained.getTimeSum() / ((retained_controls.cols() - 3) * reference_dt)
                  << ",\"duration\":" << retained.getTimeSum() << "}\n";
            }
            updateTrajInfo(retained, ros::Time::now());
            continuous_failures_count_ = 0;
            return true;
          }
          // 优化器仍沿用原均匀工作时距；初值的分段结点随几何一同保留到接纳阶段。
          // 工作时距只用于原优化器的代价计算，不作为发布轨迹的时间表。
          ctrl_pts = retained_controls;
          reference_controls = retained_controls;
          reference_knots = retained_knots;
          ts = std::max(reference_dt, (retained_knots.tail(retained_knots.size() - 1)
              - retained_knots.head(retained_knots.size() - 1)).maxCoeff());
          ctrl_pts.col(0) = start_pt - start_vel * ts + start_acc * (ts * ts / 3.);
          ctrl_pts.col(1) = start_pt - start_acc * (ts * ts / 6.);
          ctrl_pts.col(2) = start_pt + start_vel * ts + start_acc * (ts * ts / 3.);
          ctrl_pts.col(ctrl_pts.cols() - 3) = local_target_pt - local_target_vel * ts;
          ctrl_pts.col(ctrl_pts.cols() - 2) = local_target_pt;
          ctrl_pts.col(ctrl_pts.cols() - 1) = local_target_pt + local_target_vel * ts;
        }
      }
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
    if (reference_knots.size()) {
      const double first_dt = reference_knots[4] - reference_knots[3];
      const double last_dt = reference_knots[ctrl_pts.cols()] - reference_knots[ctrl_pts.cols() - 1];
      // 同一真实运动边界换回分段时间表，不能因优化器的工作时距改变实测导数。
      ctrl_pts.col(0) = start_pt - start_vel * first_dt + start_acc * (first_dt * first_dt / 3.);
      ctrl_pts.col(1) = start_pt - start_acc * (first_dt * first_dt / 6.);
      ctrl_pts.col(2) = start_pt + start_vel * first_dt + start_acc * (first_dt * first_dt / 3.);
      ctrl_pts.col(ctrl_pts.cols() - 3) = local_target_pt - local_target_vel * last_dt;
      ctrl_pts.col(ctrl_pts.cols() - 2) = local_target_pt;
      ctrl_pts.col(ctrl_pts.cols() - 1) = local_target_pt + local_target_vel * last_dt;
    }
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    if (reference_knots.size()) pos.setKnot(reference_knots);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    const bool needs_reallocation=route_time_seed
        ? ((ratio=strictDynamicScale(pos))>1.) : !pos.checkFeasibility(ratio,false);
    bool flag_step_2_success = true;
    if (needs_reallocation)
    {
      std::cerr << "Need to reallocate time." << endl;
      if(route_time_seed) {
        // 已优化轨迹只调整时标；静止边界整体缩放，移动边界沿原节点保留首末导数。
        const double scale=ratio*(1.+std::numeric_limits<double>::epsilon()*(ctrl_pts.cols()+4));
        if(start_vel.isZero(0.)&&start_acc.isZero(0.)&&local_target_vel.isZero(0.))
          pos.setKnot((pos.getKnot()*scale).eval());
        else
          pos.lengthenTime(scale);
      } else {
        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }
    }

    bool accepted = flag_step_2_success && checkDynamicFeasibility(pos)
        && (!route_time_seed || (trajectoryCollisionFree(pos) && strictDynamicScale(pos) <= 1.));
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
    auto velocity=trajectory.getDerivative();
    const double dt = std::min(.01, grid_map_->getResolution() / (4. * pp_.max_vel_));
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 0; i <= samples; ++i) {
      const double time = duration * i / samples;
      const Eigen::Vector3d point = trajectory.evaluateDeBoorT(time);
      const double yaw = trajectoryHeading(velocity,duration,time,dt,0.);
      if (grid_map_->getInflateOccupancy(point, yaw) != 0) return false;
    }
    return true;
  }

  // 已有余量或确实窄到无法留足余量时可直交参考，其余情况进入原样条优化。
  bool SCANPlannerManager::trajectoryHasPreferredClearance(UniformBspline& trajectory)
  {
    const double duration=trajectory.getTimeSum();
    auto velocity=trajectory.getDerivative();
    const double dt=std::min(.01,grid_map_->getResolution()/(4.*pp_.max_vel_));
    const int samples=std::max(1,static_cast<int>(std::ceil(duration/dt)));
    for(int i=0;i<=samples;++i) {
      const double time=duration*i/samples;
      const double yaw=trajectoryHeading(velocity,duration,time,dt,0.);
      if(!grid_map_->bodyHasPreferredClearance(trajectory.evaluateDeBoorT(time),yaw))return false;
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

