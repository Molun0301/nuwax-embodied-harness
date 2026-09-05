#pragma once

#include <array>

namespace nuwax_lio {

// IMU 线协议使用传感器坐标，惯导估计器使用机体坐标；向量只旋转，不加平移。
inline std::array<double, 3> rotate_imu_to_body(
    const std::array<double, 9>& body_from_sensor,
    double x, double y, double z) {
    return {
        body_from_sensor[0] * x + body_from_sensor[1] * y + body_from_sensor[2] * z,
        body_from_sensor[3] * x + body_from_sensor[4] * y + body_from_sensor[5] * z,
        body_from_sensor[6] * x + body_from_sensor[7] * y + body_from_sensor[8] * z,
    };
}

}  // 命名空间 nuwax_lio
