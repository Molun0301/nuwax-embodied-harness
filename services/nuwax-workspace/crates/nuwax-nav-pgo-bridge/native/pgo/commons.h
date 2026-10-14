#pragma once

#include <cstdint>

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct PoseWithTime {
    Vec3 t;
    double yaw{0.0};
    int32_t sec{0};
    uint32_t nsec{0};
    double second{0.0};

    void setTime(int32_t seconds, uint32_t nanoseconds);
};

double wrapAngle(double angle);
