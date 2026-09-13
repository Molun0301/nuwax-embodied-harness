#pragma once

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace nuwax_lio {

// 在局部中心坐标检查支撑；正常邻域保留原有最小二乘结果。
// 共线/近共线支撑不能确定平面，不能让任意法线进入激光观测雅可比。
template <typename Scalar, int Count, typename Points>
bool stable_plane_fit(Eigen::Matrix<Scalar, 4, 1>& plane, const Points& points,
                      Scalar threshold, std::size_t count) {
    static_assert(Count >= 3);
    plane.setZero();
    if (count != Count || points.size() < count || !std::isfinite(threshold) || threshold <= 0)
        return false;
    const Eigen::Vector3d origin(points[0].x, points[0].y, points[0].z);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < count; ++i) {
        const Eigen::Vector3d point(points[i].x, points[i].y, points[i].z);
        if (!point.allFinite()) return false;
        mean += point - origin;
    }
    mean /= static_cast<double>(count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < count; ++i) {
        const Eigen::Vector3d delta = Eigen::Vector3d(points[i].x, points[i].y, points[i].z)
            - origin - mean;
        covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<double>(count);
    // 二阶主子式之和等于特征值两两乘积；常规路径无需特征分解。
    const double spread = covariance.trace();
    const double area = covariance(0, 0) * covariance(1, 1)
        + covariance(0, 0) * covariance(2, 2) + covariance(1, 1) * covariance(2, 2)
        - covariance(0, 1) * covariance(0, 1) - covariance(0, 2) * covariance(0, 2)
        - covariance(1, 2) * covariance(1, 2);
    if (!std::isfinite(spread) || !std::isfinite(area) || spread <= 1e-12
        || area < spread * spread * 1e-4) return false;
    Eigen::Matrix<Scalar, Count, 3> design;
    for (int i = 0; i < Count; ++i) {
        design(i, 0) = points[i].x;
        design(i, 1) = points[i].y;
        design(i, 2) = points[i].z;
    }
    const Eigen::Matrix<Scalar, Count, 1> rhs =
        -Eigen::Matrix<Scalar, Count, 1>::Ones();
    const auto decomposition = design.colPivHouseholderQr();
    const Eigen::Matrix<Scalar, 3, 1> coefficients = decomposition.solve(rhs);
    const Scalar length = coefficients.norm();
    // 单精度固定截距在坐标偏移远大于邻域尺度时病态，交给中心坐标求解。
    const bool offset_well_conditioned = (origin + mean).squaredNorm() <= spread * 1e6;
    if (offset_well_conditioned && coefficients.allFinite() && std::isfinite(length) && length > 1e-12) {
        Eigen::Matrix<Scalar, 4, 1> original;
        original.template head<3>() = coefficients / length;
        original[3] = Scalar(1) / length;
        bool valid = original.allFinite();
        for (int i = 0; valid && i < Count; ++i)
            valid = std::abs(original[0] * points[i].x + original[1] * points[i].y
                + original[2] * points[i].z + original[3]) <= threshold;
        if (valid) { plane = original; return true; }
    }
    // 普通非平面邻域仍按原距离门拒绝；仅奇异或病态截距需要替代求解。
    if (offset_well_conditioned && decomposition.rank() == 3) return false;
    // 固定截距形式在穿过原点时奇异；有效面积支撑下改用中心坐标平面。
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen;
    eigen.computeDirect(covariance);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()) return false;
    Eigen::Vector3d normal = eigen.eigenvectors().col(0);
    double offset = -normal.dot(origin + mean);
    // 固定符号只影响可重复性，观测方程仍使用同一个完整平面。
    Eigen::Index major;
    normal.cwiseAbs().maxCoeff(&major);
    if (normal[major] < 0) { normal = -normal; offset = -offset; }
    for (std::size_t i = 0; i < count; ++i) {
        const Eigen::Vector3d delta = Eigen::Vector3d(points[i].x, points[i].y, points[i].z)
            - origin - mean;
        if (std::abs(normal.dot(delta)) > static_cast<double>(threshold)) return false;
    }
    const Eigen::Matrix<Scalar, 4, 1> fitted =
        (Eigen::Vector4d() << normal.x(), normal.y(), normal.z(), offset).finished().template cast<Scalar>();
    if (!fitted.allFinite()) return false;
    plane = fitted;
    return true;
}

} // namespace nuwax_lio
