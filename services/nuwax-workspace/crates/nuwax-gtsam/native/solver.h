#pragma once
#include <cstddef>
#include <cstdint>

// 所有数组的顺序在语言边界固定：位姿为平移+XYZW，信息矩阵为平移在前的行主序。
struct NuwaxGraphPose { double translation[3]; double rotation_xyzw[4]; };
struct NuwaxGraphEdge {
    std::uint64_t from;
    std::uint64_t to;
    NuwaxGraphPose measurement;
    double information[36];
};
struct NuwaxGraphResult { double initial_cost; double final_cost; std::uint32_t iterations; std::uint8_t converged; };
// sigma为零表示该节点不提供重力观测；有效方向在机体坐标中且长度为一。
struct NuwaxGravityPrior { double direction_body[3]; double sigma; };
using NuwaxCancel = int (*)(void*);
extern "C" int nuwax_gtsam_optimize_with_gravity(
    const NuwaxGraphPose* initial, const std::uint8_t* fixed, std::size_t node_count,
    const NuwaxGraphEdge* edges, std::size_t edge_count, const NuwaxGravityPrior* gravity,
    std::uint32_t max_iterations, double gradient_tolerance, double initial_lambda, double lambda_scale,
    NuwaxCancel cancelled, void* cancel_context, NuwaxGraphPose* output,
    NuwaxGraphResult* result, char* error, std::size_t error_capacity) noexcept;
extern "C" int nuwax_gtsam_optimize(
    const NuwaxGraphPose* initial, const std::uint8_t* fixed, std::size_t node_count,
    const NuwaxGraphEdge* edges, std::size_t edge_count, std::uint32_t max_iterations,
    double gradient_tolerance, double initial_lambda, double lambda_scale,
    NuwaxCancel cancelled, void* cancel_context, NuwaxGraphPose* output,
    NuwaxGraphResult* result, char* error, std::size_t error_capacity) noexcept;

struct NuwaxRegistrationResult {
    NuwaxGraphPose transformation;
    double mse;
    std::uint32_t iterations;
    std::uint8_t converged;
};
extern "C" int nuwax_pcl_point_to_plane(const double* source, std::size_t source_count,
    const double* target, std::size_t target_count, double max_distance, std::uint32_t iterations,
    NuwaxRegistrationResult* result) noexcept;
