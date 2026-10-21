// 本地无 ROS 提取与真实感知/机身适配；改动范围和上游归属见 native/NOTICE。
#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>


namespace scan_planner
{

inline std::vector<Eigen::Vector3d> simplifyReferenceWaypoints(
    const std::vector<Eigen::Vector3d> &points,
    double tolerance)
{
  if (points.size() <= 2 || tolerance <= 0.0)
    return points;

  std::vector<bool> keep(points.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<size_t, size_t>> ranges{{0, points.size() - 1}};

  while (!ranges.empty())
  {
    const auto [first, last] = ranges.back();
    ranges.pop_back();
    if (last <= first + 1)
      continue;

    const Eigen::Vector3d segment = points[last] - points[first];
    const double squared_length = segment.squaredNorm();
    double max_distance = -1.0;
    size_t max_index = first;

    for (size_t index = first + 1; index < last; ++index)
    {
      double distance;
      if (squared_length <= 1e-12)
      {
        distance = (points[index] - points[first]).norm();
      }
      else
      {
        const double ratio = std::clamp(
            (points[index] - points[first]).dot(segment) / squared_length,
            0.0, 1.0);
        const Eigen::Vector3d projection = points[first] + ratio * segment;
        distance = (points[index] - projection).norm();
      }

      if (distance > max_distance)
      {
        max_distance = distance;
        max_index = index;
      }
    }

    if (max_distance > tolerance)
    {
      keep[max_index] = true;
      ranges.emplace_back(first, max_index);
      ranges.emplace_back(max_index, last);
    }
  }

  std::vector<Eigen::Vector3d> simplified;
  simplified.reserve(points.size());
  for (size_t index = 0; index < points.size(); ++index)
  {
    if (keep[index])
      simplified.push_back(points[index]);
  }
  return simplified;
}

inline bool prepareReferenceWaypoints(
    const std::vector<Eigen::Vector3d> &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr,
    double simplify_tolerance = 0.0)
{
  waypoints.clear();
  if (path.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0 ||
      !std::isfinite(simplify_tolerance) || simplify_tolerance < 0.0)
  {
    if (error)
      *error = "body height, minimum distance, and simplify tolerance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.size());
  Eigen::Vector3d final_waypoint;
  Eigen::Vector3d last_waypoint;
  bool first = true;

  for (const auto &position : path)
  {
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z()))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    Eigen::Vector3d waypoint(position.x(), position.y(), position.z() + body_height);
    final_waypoint = waypoint;
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }

  waypoints = simplifyReferenceWaypoints(waypoints, simplify_tolerance);
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
