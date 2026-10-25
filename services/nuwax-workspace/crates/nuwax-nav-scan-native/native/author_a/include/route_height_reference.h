#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <limits>
#include <vector>

namespace scan_planner {
// Height guidance from the actual ordered PCT route, not a new ground map.
// A local window is selected by the original global-reference time interval.
// Queries are pure and never advance route progress during optimization/search.
class RouteHeightReference {
 public:
  std::vector<Eigen::Vector3d> points;
  std::vector<double> arcs, waypoint_arcs, times, timed_arcs;
  double begin_arc=0., end_arc=0.;
  double begin_time=0., end_time=0.;
  void clear(){points.clear();arcs.clear();waypoint_arcs.clear();times.clear();timed_arcs.clear();begin_arc=end_arc=begin_time=end_time=0.;}
  void set(const std::vector<Eigen::Vector3d>& route) {
    clear();points=route;arcs.resize(points.size(),0.);
    for(size_t i=1;i<points.size();++i)arcs[i]=arcs[i-1]+(points[i]-points[i-1]).norm();
    if(!arcs.empty())end_arc=arcs.back();
  }
  double arcAtTime(double t) const {
    if(times.empty())return 0.;
    if(t<=times.front())return timed_arcs.front();
    for(size_t i=1;i<times.size();++i)if(t<=times[i]) {
      const double duration=times[i]-times[i-1];
      const double f=duration>0.?(t-times[i-1])/duration:1.;
      return timed_arcs[i-1]+f*(timed_arcs[i]-timed_arcs[i-1]);
    }
    return timed_arcs.back();
  }
  void window(double from,double to){begin_time=std::min(from,to);end_time=std::max(from,to);begin_arc=arcAtTime(begin_time);end_arc=arcAtTime(end_time);}
  bool ready()const{return points.size()>1&&!times.empty();}
  // The ordered parameter, not nearest XY, disambiguates repeated coordinates
  // and remains continuous at corners and vertical route segments.
  Eigen::Vector3d positionAtArc(double arc) const {
    if(points.empty())return Eigen::Vector3d::Zero();
    if(arc<=arcs.front())return points.front();
    for(size_t i=1;i<points.size();++i)if(arc<=arcs[i]) {
      const double length=arcs[i]-arcs[i-1];
      const double f=length>0.?(arc-arcs[i-1])/length:1.;
      return points[i-1]+f*(points[i]-points[i-1]);
    }
    return points.back();
  }
  double heightAtArc(double arc)const {
    if(points.empty())return 0.;
    if(arc<=arcs.front())return points.front().z();
    for(size_t i=1;i<points.size();++i)if(arc<=arcs[i]) {
      const double length=arcs[i]-arcs[i-1];
      const double f=length>0.?(arc-arcs[i-1])/length:1.;
      return points[i-1].z()+f*(points[i].z()-points[i-1].z());
    }
    return points.back().z();
  }
};
}
