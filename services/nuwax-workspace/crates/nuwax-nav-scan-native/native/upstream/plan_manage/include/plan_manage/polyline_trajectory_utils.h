#ifndef PLAN_MANAGE_POLYLINE_TRAJECTORY_UTILS_H
#define PLAN_MANAGE_POLYLINE_TRAJECTORY_UTILS_H

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <traj_utils/polynomial_traj.h>

namespace scan_planner
{

inline bool makePiecewiseLinearTrajectory(
    const std::vector<Eigen::Vector3d> &input_points,
    double speed,
    PolynomialTraj &trajectory,
    std::string *error = nullptr)
{
  trajectory.reset();
  if (!std::isfinite(speed) || speed <= 0.0)
  {
    if (error) *error = "polyline speed must be finite and positive";
    return false;
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(input_points.size());
  for (const auto &point : input_points)
  {
    if (!point.allFinite())
    {
      if (error) *error = "polyline contains a non-finite point";
      return false;
    }
    if (points.empty() || (point - points.back()).norm() > 1e-6)
      points.push_back(point);
  }
  if (points.size() < 2)
  {
    if (error) *error = "polyline requires at least two distinct points";
    return false;
  }

  for (size_t index = 0; index + 1 < points.size(); ++index)
  {
    const Eigen::Vector3d delta = points[index + 1] - points[index];
    const double length = delta.norm();
    const double duration = length / speed;
    const Eigen::Vector3d velocity = delta / duration;

    // PolynomialTraj stores each axis in descending power order.  A linear
    // segment is therefore [0*t^5, 0*t^4, 0*t^3, 0*t^2, v*t, p0].
    std::vector<double> cx{0.0, 0.0, 0.0, 0.0, velocity.x(), points[index].x()};
    std::vector<double> cy{0.0, 0.0, 0.0, 0.0, velocity.y(), points[index].y()};
    std::vector<double> cz{0.0, 0.0, 0.0, 0.0, velocity.z(), points[index].z()};
    trajectory.addSegment(cx, cy, cz, duration);
  }
  trajectory.init();
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_POLYLINE_TRAJECTORY_UTILS_H
