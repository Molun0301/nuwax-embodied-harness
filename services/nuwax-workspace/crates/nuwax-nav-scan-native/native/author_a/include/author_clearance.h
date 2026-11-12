#pragma once
#include <algorithm>
#include <cmath>
#include <scan_native.h>

namespace scan_planner {
// 局部曲线的唯一首选净空；窄处仍由原硬检查决定能否通过。
inline double preferredClearance(double resolution) {
  return std::max(.15,resolution);
}

inline double deviceSpeedMagnitude(const ScanConfig& config) {
  return std::hypot(std::max(config.max_velocity,config.max_reverse_velocity),config.max_lateral_velocity);
}

inline double clearanceSpeedCap(const ScanConfig& config,double clearance) {
  const double device=deviceSpeedMagnitude(config);
  const double near=std::min(.10,device);
  if(clearance>=.20)return device;
  if(clearance<=.05)return near;
  return near+(device-near)*(clearance-.05)/.15;
}
}
