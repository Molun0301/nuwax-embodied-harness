#pragma once
#include <algorithm>

namespace scan_planner {
// 局部曲线的唯一首选净空；窄处仍由原硬检查决定能否通过。
inline double preferredClearance(double resolution) {
  return std::max(.15,resolution);
}
}
