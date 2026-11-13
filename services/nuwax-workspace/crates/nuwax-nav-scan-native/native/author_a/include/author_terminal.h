#pragma once
#include <plan_env/grid_map.h>
#include <scan_native.h>

namespace scan_planner {
// 停点按目标朝向在原位置容差内选择首选净空。
inline ScanTerminalPoints terminalPoints(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& goal,
    double yaw,double tolerance) {
  Eigen::Vector3d stop=goal;
  const double spacing=c.resolution/2.;
  if(map.getInflateOccupancy(goal,yaw)!=0 || map.bodyFieldClearance(goal,yaw)<map.preferredClearance()) {
    const double range=tolerance-.03;
    const int count=int(std::floor(range/spacing));
    double best_distance=INFINITY,best_clearance=-INFINITY;
    bool preferred=false;
    for(int x=-count;x<=count;++x)for(int y=-count;y<=count;++y) {
      const Eigen::Vector3d point=goal+Eigen::Vector3d(x*spacing,y*spacing,0.);
      const double distance=(point-goal).head<2>().norm();
      if(distance>range || map.getInflateOccupancy(point,yaw)!=0)continue;
      const double clearance=map.bodyFieldClearance(point,yaw);
      const bool wide=clearance>=map.preferredClearance();
      if(wide?(!preferred||distance<best_distance):(!preferred&&(clearance>best_clearance
          ||(clearance==best_clearance&&distance<best_distance)))) {
        stop=point;preferred=wide;best_distance=distance;best_clearance=clearance;
      }
    }
  }
  ScanTerminalPoints result{};
  std::copy_n(stop.data(),3,result.stop);
  return result;
}
}
