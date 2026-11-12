#pragma once
#include <plan_env/grid_map.h>
#include <route_height_reference.h>
#include <scan_native.h>

namespace scan_planner {
// 停点按目标朝向留净空；回溯候选按设备能力平移后，再固定目标朝向直线进停点。
inline ScanTerminalPoints terminalPoints(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& goal,
    double yaw,double tolerance,const std::vector<Eigen::Vector3d>& reference) {
  Eigen::Vector3d stop=goal;
  const double spacing=c.resolution/2.;
  if(map.getInflateOccupancy(goal,yaw)!=0 || map.bodyFieldClearance(goal,yaw)<.10) {
    const double range=tolerance/2.;
    const int count=int(std::floor(range/spacing));
    double best_distance=INFINITY,best_clearance=-INFINITY;
    bool preferred=false;
    for(int x=-count;x<=count;++x)for(int y=-count;y<=count;++y) {
      const Eigen::Vector3d point=goal+Eigen::Vector3d(x*spacing,y*spacing,0.);
      const double distance=(point-goal).head<2>().norm();
      if(distance>range || map.getInflateOccupancy(point,yaw)!=0)continue;
      const double clearance=map.bodyFieldClearance(point,yaw);
      const bool wide=clearance>=.10;
      if(wide?(!preferred||distance<best_distance):(!preferred&&(clearance>best_clearance
          ||(clearance==best_clearance&&distance<best_distance)))) {
        stop=point;preferred=wide;best_distance=distance;best_clearance=clearance;
      }
    }
  }
  RouteHeightReference route;route.set(reference);
  ScanTerminalPoints result{};
  std::copy_n(stop.data(),3,result.stop);std::copy_n(stop.data(),3,result.turn);
  std::copy_n(stop.data(),3,result.route_point);
  result.route_arc=route.end_arc;
  const double radius=std::hypot(c.body_length/2.+std::abs(c.body_center_x),c.body_width/2.+std::abs(c.body_center_y));
  if(map.fieldDistance(stop.x(),stop.y())>=radius+.05)return result;
  const int count=int(std::floor(std::min(1.5,route.end_arc)/spacing));
  for(int k=0;!route.points.empty()&&k<=count;++k) {
    const double arc=route.end_arc-k*spacing;
    const auto route_point=route.positionAtArc(arc);
    Eigen::Vector3d point=route_point;
    const double dx=stop.x()-point.x(),dy=stop.y()-point.y();
    const double forward=std::cos(yaw)*dx+std::sin(yaw)*dy;
    const double lateral=-std::sin(yaw)*dx+std::cos(yaw)*dy;
    // 沿不可用通道平移候选点，去掉末段平移所需的倒退或横移分量。
    if(c.max_reverse_velocity==0.&&forward<0.)point+=forward*Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0.);
    if(c.max_lateral_velocity==0.&&lateral!=0.)point+=lateral*Eigen::Vector3d(-std::sin(yaw),std::cos(yaw),0.);
    if(map.fieldDistance(point.x(),point.y())<radius+.05)continue;
    const auto delta=(stop-point).eval();
    const double distance=delta.norm();
    const int samples=int(std::ceil(distance/spacing));
    bool free=true;
    for(int j=0;j<=samples;++j) {
      const auto sample=(point+delta*(std::min(j*spacing,distance)/distance)).eval();
      if(map.getInflateOccupancy(sample,yaw)!=0){free=false;break;}
    }
    if(free) {
      std::copy_n(point.data(),3,result.turn);std::copy_n(route_point.data(),3,result.route_point);
      result.route_arc=arc;return result;
    }
  }
  // 无可用转向点时整套选点退回原目标，保留原参考路线。
  std::copy_n(goal.data(),3,result.stop);std::copy_n(goal.data(),3,result.turn);
  std::copy_n(goal.data(),3,result.route_point);
  return result;
}
}
