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
    const auto consider=[&](const Eigen::Vector3d& point,double distance) {
      if(distance>range || map.getInflateOccupancy(point,yaw)!=0)return;
      const double clearance=map.bodyFieldClearance(point,yaw);
      const bool wide=clearance>=map.preferredClearance();
      if(wide?(!preferred||distance<best_distance):(!preferred&&(clearance>best_clearance
          ||(clearance==best_clearance&&distance<best_distance)))) {
        stop=point;preferred=wide;best_distance=distance;best_clearance=clearance;
      }
    };
    for(int x=-count;x<=count;++x)for(int y=-count;y<=count;++y) {
      const Eigen::Vector3d point=goal+Eigen::Vector3d(x*spacing,y*spacing,0.);
      consider(point,(point-goal).head<2>().norm());
    }
    // 两个方向的每条网格线都加入合法圆交点；同圆交点的距离就是原半径。
    for(int line=-count;line<=count;++line) {
      const double offset=line*spacing,other=std::sqrt(range*range-offset*offset);
      for(double sign:{-1.,1.}) {
        consider(goal+Eigen::Vector3d(offset,sign*other,0.),range);
        consider(goal+Eigen::Vector3d(sign*other,offset,0.),range);
      }
    }
  }
  ScanTerminalPoints result{};
  std::copy_n(stop.data(),3,result.stop);
  return result;
}
}
