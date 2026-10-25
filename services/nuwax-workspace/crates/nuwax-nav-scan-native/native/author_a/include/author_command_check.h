#pragma once
#include <plan_env/grid_map.h>
#include <scan_native.h>
#include <chrono>
#include <array>
#include <limits>
#include <vector>

namespace scan_planner {
struct CheckedCommand {
  Eigen::Vector2d linear = Eigen::Vector2d::Zero();
  double angular = 0.;
  bool found = false, limited = false, hold_translation = false, exhausted = false;
};

namespace command_check_detail {
using Clock = std::chrono::steady_clock;

inline Eigen::Vector2d toward(const Eigen::Vector2d& current, const Eigen::Vector2d& target, double step) {
  const Eigen::Vector2d delta = target - current;
  const double distance = std::hypot(delta.x(), delta.y());
  return distance <= step ? target : (current + delta * (step / distance)).eval();
}

inline Eigen::Vector2d world(const Eigen::Vector2d& body, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * body.x() - s * body.y(), s * body.x() + c * body.y()};
}
inline Eigen::Vector2d body(const Eigen::Vector2d& value, double yaw) { return world(value, -yaw); }

struct State {
  Eigen::Vector3d position;
  double yaw;
  Eigen::Vector2d linear;
  double angular;
};

inline CheckedCommand bounded(const Eigen::Vector2d& requested, double angular,
    const State& actual, const ScanConfig& c, bool stop) {
  Eigen::Vector2d target(std::clamp(requested.x(), -c.max_reverse_velocity, c.max_velocity),
      std::clamp(requested.y(), -c.max_lateral_velocity, c.max_lateral_velocity));
  auto linear = toward(actual.linear, target, c.max_acceleration * c.sample_dt);
  linear.x() = std::clamp(linear.x(), -c.max_reverse_velocity, c.max_velocity);
  linear.y() = std::clamp(linear.y(), -c.max_lateral_velocity, c.max_lateral_velocity);
  angular = std::clamp(angular, actual.angular - c.max_yaw_acceleration * c.sample_dt,
      actual.angular + c.max_yaw_acceleration * c.sample_dt);
  angular = std::clamp(angular, -c.max_yaw_rate, c.max_yaw_rate);
  return {linear, angular, true, true, stop, false};
}

inline bool advance(GridMap& map, State& actual, const Eigen::Vector2d& target,
    double target_angular, const ScanConfig& c, double radius, Clock::time_point deadline, bool& exhausted) {
  // 预测执行器响应，保留当前实测速度，不把归零请求当成立刻静止。
  const auto next = toward(actual.linear, target, c.max_acceleration * c.sample_dt);
  const double next_angular = std::clamp(target_angular,
      actual.angular - c.max_yaw_acceleration * c.sample_dt,
      actual.angular + c.max_yaw_acceleration * c.sample_dt);
  const Eigen::Vector2d mean = actual.linear * .5 + next * .5;
  const double mean_angular = actual.angular * .5 + next_angular * .5;
  const double travel = (std::hypot(mean.x(), mean.y()) + radius * std::abs(mean_angular)) * c.sample_dt;
  const double fraction_step = travel > 0. ? std::min(1., c.resolution / (4. * travel)) : 1.;
  Eigen::Vector3d position = actual.position;
  double yaw = actual.yaw;
  for (double fraction = 0.; fraction < 1.;) {
    if (Clock::now() >= deadline) { exhausted = true; return false; }
    fraction = std::min(1., fraction + fraction_step);
    const double time = c.sample_dt * fraction, rotation = mean_angular * time;
    Eigen::Vector2d displacement;
    if (std::abs(mean_angular) < 1e-9) displacement = mean * time;
    else {
      const double a = std::sin(rotation) / mean_angular, b = (1. - std::cos(rotation)) / mean_angular;
      displacement = {a * mean.x() - b * mean.y(), b * mean.x() + a * mean.y()};
    }
    position = actual.position;
    position.head<2>() += world(displacement, actual.yaw);
    yaw = actual.yaw + rotation;
    // 平面命令按当前实际高度检查；原三维轨迹查询继续独立执行。
    if (map.getInflateOccupancy(position, yaw) != 0) return false;
  }
  actual = {position, yaw, next, next_angular};
  return true;
}

inline bool collisionFree(GridMap& map, State actual, const CheckedCommand& command,
    const ScanConfig& c, double radius, Clock::time_point deadline, bool& exhausted, State& stopped) {
  if (!advance(map, actual, command.linear, command.angular, c, radius, deadline, exhausted)) return false;
  while (!actual.linear.isZero(0.) || actual.angular != 0.)
    if (!advance(map, actual, Eigen::Vector2d::Zero(), 0., c, radius, deadline, exhausted)) return false;
  stopped = actual;
  return true;
}
}

inline CheckedCommand checkAuthorCommand(GridMap& map, const ScanConfig& c,
    const Eigen::Vector3d& position, double yaw, const Eigen::Vector2d& actual_world_velocity,
    double actual_angular, const Eigen::Vector2d& preferred, double preferred_angular,
    const Eigen::Vector3d& reference_position, double desired_yaw, bool translation_goal) {
  using namespace command_check_detail;
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.sample_dt));
  const State actual{position, yaw, body(actual_world_velocity, yaw), actual_angular};
  const double radius = std::hypot(c.body_length / 2. + std::abs(c.body_center_x),
      c.body_width / 2. + std::abs(c.body_center_y));
  bool exhausted = false;
  State stopped = actual;
  CheckedCommand original{preferred, preferred_angular, true, false, false, false};
  if (collisionFree(map, actual, original, c, radius, deadline, exhausted, stopped)) return original;
  CheckedCommand best;
  if (exhausted) { best.exhausted = true; return best; }
  const auto angle_error = [&](double value) {
    return std::abs(std::atan2(std::sin(desired_yaw - value), std::cos(desired_yaw - value)));
  };
  const double initial_distance = (reference_position - actual.position).head<2>().norm();
  const double initial_angle = angle_error(actual.yaw);
  const auto progress = [&](const State& end) {
    return initial_distance - (reference_position - end.position).head<2>().norm()
        + radius * (initial_angle - angle_error(end.yaw));
  };
  const auto deviation = [&](const CheckedCommand& value) {
    return (value.linear - preferred).squaredNorm()
        + radius * radius * std::pow(value.angular - preferred_angular, 2);
  };
  std::vector<CheckedCommand> candidates;
  const auto add = [&](const Eigen::Vector2d& target, double angular, bool stop) {
    auto candidate = bounded(target, angular, actual, c, stop);
    if ((original.linear.array() == candidate.linear.array()).all() && original.angular == candidate.angular) return;
    for (const auto& old : candidates)
      if ((old.linear.array() == candidate.linear.array()).all() && old.angular == candidate.angular) return;
    candidates.push_back(candidate);
  };
  const auto brake = bounded(Eigen::Vector2d::Zero(), 0., actual, c, true);
  double best_progress = -std::numeric_limits<double>::infinity();
  if (collisionFree(map, actual, brake, c, radius, deadline, exhausted, stopped)) {
    best = brake; best_progress = progress(stopped);
  }
  const std::array<double, 5> fractions{-1., -.5, 0., .5, 1.};
  std::vector<double> angular_targets{0., preferred_angular};
  const double yaw_error = std::atan2(std::sin(desired_yaw-yaw), std::cos(desired_yaw-yaw));
  const double angular_step = c.max_yaw_acceleration * c.sample_dt;
  angular_targets.push_back(std::copysign(std::min(c.max_yaw_rate,
      std::sqrt(angular_step*angular_step+2.*c.max_yaw_acceleration*std::abs(yaw_error))-angular_step), yaw_error));
  for (double fraction : fractions) {
    const double value = actual.angular + fraction * angular_step;
    if (value * yaw_error >= 0.) angular_targets.push_back(value);
  }
  for (double angular : angular_targets) {
    add(Eigen::Vector2d::Zero(), angular, true);
    add(preferred, angular, preferred.isZero(0.));
    if (translation_goal) {
      for (double x : fractions) for (double y : fractions)
        add(actual.linear + Eigen::Vector2d(x, y) * (c.max_acceleration * c.sample_dt), angular, false);
    }
  }
  // 排序只确定检查顺序，最终选择必须通过同一实际积分及制动查询。
  const auto estimate = [&](const CheckedCommand& value) {
    State end = actual;
    const auto next = toward(actual.linear, value.linear, c.max_acceleration * c.sample_dt);
    const double angular = std::clamp(value.angular, actual.angular-angular_step, actual.angular+angular_step);
    const Eigen::Vector2d displacement = (actual.linear*.5+next*.5)*c.sample_dt
        + next * (std::hypot(next.x(),next.y())/(2.*c.max_acceleration));
    end.position.head<2>() += world(displacement, yaw);
    end.yaw += (actual.angular*.5+angular*.5)*c.sample_dt
        + angular*std::abs(angular)/(2.*c.max_yaw_acceleration);
    return progress(end);
  };
  std::stable_sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b) {
    const double left=estimate(a), right=estimate(b);
    return left==right ? deviation(a)<deviation(b) : left>right;
  });
  for (const auto& candidate : candidates) {
    if (exhausted) break;
    if (collisionFree(map, actual, candidate, c, radius, deadline, exhausted, stopped)) {
      const double gain = progress(stopped);
      if (!best.found || gain > best_progress || (gain == best_progress && deviation(candidate) < deviation(best))) {
        best=candidate;best_progress=gain;
        if (gain>0.) break;
      }
    }
  }
  best.exhausted = exhausted;
  return best;
}
}
