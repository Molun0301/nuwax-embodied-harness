#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <planned_heading.h>

namespace scan_planner {
// 只等比放大结点间隔，控制点及曲线几何保持不变。
inline void arrangeHeadingTime(UniformBspline& trajectory,HeadingPlan& heading) {
  const double duration=trajectory.getTimeSum(),required=heading.duration(duration);
  if(required>duration) {
    auto knots=trajectory.getKnot();
    double begin,end;trajectory.getTimeSpan(begin,end);
    knots=((knots.array()-begin)*(required/duration)+begin).matrix();
    trajectory.setKnot(knots);
    // 保留转好后到曲线末端的时间；增加的起步时间同时移到转好时刻之前。
    if(heading.settle_time) *heading.settle_time+=required-duration;
  }
}
// 未来曲线减速时，微小但非零位移仍决定该处朝向，不能替换为当前机体或世界零角。
// 终点使用进入方向；完全静止的一段才沿用调用者的静止朝向。
inline double trajectoryHeading(UniformBspline& velocity,double duration,double time,double step,double stationary_yaw) {
  // 直接求导避免两个大世界坐标相减的消减误差，也不跨过静止拐角混合两侧方向。
  Eigen::Vector2d direction=velocity.evaluateDeBoorT(time).head<2>();
  if(direction.isZero(0.))
    direction=velocity.evaluateDeBoorT(std::min(duration,time+step)).head<2>();
  if(direction.isZero(0.))
    direction=velocity.evaluateDeBoorT(std::max(0.,time-step)).head<2>();
  return direction.isZero(0.)?stationary_yaw:std::atan2(direction.y(),direction.x());
}
}
