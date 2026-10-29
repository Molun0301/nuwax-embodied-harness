#pragma once
#include <tuple>

// 雷达格与融合高度带的身份分开；同列新增或扩大的高度带不能继承旧接触豁免。
struct BodyCollisionCell {
  int x, y, z;
  bool surface = false;
  double low = 0., high = 0.;
  int operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
  bool operator<(const BodyCollisionCell& other) const {
    return std::tie(surface,x,y,z,low,high)
        < std::tie(other.surface,other.x,other.y,other.z,other.low,other.high);
  }
};
