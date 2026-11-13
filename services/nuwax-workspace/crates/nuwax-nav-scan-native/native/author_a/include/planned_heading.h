#pragma once
#include <algorithm>
#include <cmath>
#include <optional>

namespace scan_planner {
inline double headingDifference(double angle,double center) {
  return std::atan2(std::sin(angle-center),std::cos(angle-center));
}
// 后面的能力和起点可达约束优先；返回以起点为锚的连续角表示。
inline double plannedHeading(double tangent,double time,double duration,double yaw0,
    std::optional<double> goal,double reverse,double lateral,double omega,bool use_start=true) {
  double yaw=tangent;
  if(goal)yaw=*goal+std::clamp(headingDifference(yaw,*goal),-omega*(duration-time),omega*(duration-time));
  if(reverse==0.)yaw=tangent+std::clamp(headingDifference(yaw,tangent),-M_PI/2.,M_PI/2.);
  if(lateral==0.)yaw=tangent;
  const double difference=headingDifference(yaw,yaw0);
  return yaw0+(use_start?std::clamp(difference,-omega*time,omega*time):difference);
}
struct HeadingPlan {
  double yaw0=0.;
  std::optional<double> goal;
  double reverse=0.,lateral=0.,omega=0.;
  bool turn_first=false;
  double at(double tangent,double time,double duration) const {
    return plannedHeading(tangent,time,duration,yaw0,goal,reverse,lateral,omega,!turn_first);
  }
  double duration(double original) const {
    return goal?std::max(original,std::abs(headingDifference(*goal,yaw0))/omega):original;
  }
};
}
