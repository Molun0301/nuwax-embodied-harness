#pragma once

// PointLIO输入参数的边界校验；此头由原生入口直接包含。
#include <cmath>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>
#include "dimos_native_module.hpp"

namespace nuwax_lio {

template<class Params>
void validate_pointlio_options(const Params& p, const dimos::NativeModule& args) {
    // 不再让未实现的兼容字段或错误消息单位悄悄回退。
    for (const char* name : {"cube_side_length", "fov_degree", "msr_freq", "odom_only",
            "scan_line", "scan_rate", "con_frame", "con_frame_num", "cut_frame",
            "cut_frame_time_interval", "time_lag_imu_to_lidar"}) {
        if (args.has(name)) throw std::invalid_argument(std::string("当前Point输入链不支持参数：") + name);
    }
    if (args.has("lidar_type") && args.arg("lidar_type") != "avia")
        throw std::invalid_argument("规范点云入口不通过lidar_type切换格式");
    if (args.has("timestamp_unit") && args.arg("timestamp_unit") != "nanosecond")
        throw std::invalid_argument("规范offset_ns固定为纳秒");
    const std::string nearby = args.arg("ivox_nearby_type", "nearby6");
    if (nearby != "center" && nearby != "nearby6" && nearby != "nearby18" && nearby != "nearby26")
        throw std::invalid_argument("ivox_nearby_type必须为center/nearby6/nearby18/nearby26");
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    if (p.point_filter_num < 1 || p.init_map_size < 1 || !std::isfinite(p.blind) || p.blind < 0
            || !positive(p.det_range) || p.blind >= p.det_range
            || !positive(p.acc_norm) || !positive(p.satu_acc) || !positive(p.satu_gyro)
            || !positive(p.plane_thr) || !positive(p.match_s) || !positive(p.ivox_grid_resolution)
            || !positive(p.imu_time_inte) || !positive(p.lidar_meas_cov)
            || !positive(p.imu_meas_acc_cov) || !positive(p.imu_meas_omg_cov))
        throw std::invalid_argument("Point采样、距离、量纲或观测噪声超出有效范围");
    for (double v : {p.acc_cov_input,p.gyr_cov_input,p.acc_cov_output,p.gyr_cov_output,
            p.vel_cov,p.b_acc_cov,p.b_gyr_cov})
        if (!std::isfinite(v) || v < 0) throw std::invalid_argument("Point过程噪声必须非负且有限");
    if (p.use_imu_as_input && !p.imu_en)
        throw std::invalid_argument("惯性输入模式必须启用惯性");
    for (const auto* v : {&p.gravity, &p.gravity_init, &p.extrinsic_T}) {
        if (v->size() != 3) throw std::invalid_argument("Point三维向量长度错误");
        for (double n : *v) if (!std::isfinite(n)) throw std::invalid_argument("Point向量必须有限");
    }
    for (const auto* v : {&p.gravity, &p.gravity_init}) {
        const double norm = (*v)[0]*(*v)[0]+(*v)[1]*(*v)[1]+(*v)[2]*(*v)[2];
        if (!std::isfinite(norm) || norm <= 1e-12)
            throw std::invalid_argument("重力向量不能为零");
    }
    if (p.extrinsic_R.size() != 9) throw std::invalid_argument("外参旋转必须为九个数值");
    const Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> rotation(p.extrinsic_R.data());
    if (!rotation.allFinite() || (rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-6
            || std::abs(rotation.determinant()-1.0) > 1e-6)
        throw std::invalid_argument("外参必须为合法旋转矩阵");
    for (const char* name : {"gravity", "gravity_init", "extrinsic_t", "extrinsic_r"})
        if (args.has(name) && args.arg(name).empty()) throw std::invalid_argument("显式向量参数不能为空");
    if (p.use_imu_as_input) {
        for (const char* name : {"acc_cov_output", "gyr_cov_output", "vel_cov", "imu_meas_acc_cov", "imu_meas_omg_cov", "check_satu", "satu_acc", "satu_gyro"})
            if (args.has(name)) std::fprintf(stderr, "[pointlio] 参数%s只用于惯性观测模式，当前输入模式不使用它\n", name);
    } else {
        for (const char* name : {"acc_cov_input", "gyr_cov_input"})
            if (args.has(name)) std::fprintf(stderr, "[pointlio] 参数%s只用于惯性输入模式，当前观测模式不使用它\n", name);
    }
}

}  // 命名空间nuwax_lio
