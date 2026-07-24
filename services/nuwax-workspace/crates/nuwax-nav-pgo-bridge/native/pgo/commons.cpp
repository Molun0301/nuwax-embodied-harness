#include "commons.h"

#include <cmath>
#include <limits>

void PoseWithTime::setTime(int32_t seconds, uint32_t nanoseconds) {
    sec = seconds;
    nsec = nanoseconds;
    second = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
}

double wrapAngle(double angle) {
    constexpr double pi = 3.14159265358979323846;
    constexpr double two_pi = 2.0 * pi;
    if (!std::isfinite(angle)) return std::numeric_limits<double>::quiet_NaN();
    return std::remainder(angle, two_pi);
}
