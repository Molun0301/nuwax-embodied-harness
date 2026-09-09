#include "solver.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {
gtsam::Pose3 pose(const NuwaxGraphPose& value) {
    const auto& q = value.rotation_xyzw;
    for (double v : value.translation) if (!std::isfinite(v)) throw std::invalid_argument("non-finite translation");
    double norm = 0;
    for (double v : q) { if (!std::isfinite(v)) throw std::invalid_argument("non-finite rotation"); norm += v*v; }
    if (std::abs(norm-1.0) > 1e-6) throw std::invalid_argument("rotation is not unit length");
    return {gtsam::Rot3::Quaternion(q[3], q[0], q[1], q[2]),
        gtsam::Point3(value.translation[0], value.translation[1], value.translation[2])};
}
NuwaxGraphPose wire(const gtsam::Pose3& value) {
    const auto t=value.translation(); const auto q=value.rotation().toQuaternion();
    return {{t.x(),t.y(),t.z()}, {q.x(),q.y(),q.z(),q.w()}};
}
}
extern "C" int nuwax_gtsam_optimize(
    const NuwaxGraphPose* initial, const std::uint8_t* fixed, std::size_t node_count,
    const NuwaxGraphEdge* edges, std::size_t edge_count, std::uint32_t max_iterations,
    double gradient_tolerance, double initial_lambda, double lambda_scale,
    NuwaxCancel cancelled, void* cancel_context, NuwaxGraphPose* output,
    NuwaxGraphResult* result, char* error, std::size_t error_capacity) noexcept {
    try {
        if (!initial || !fixed || !output || !result || node_count == 0 || node_count > 2000
            || edge_count > 8000 || (edge_count && !edges) || max_iterations > 300)
            throw std::invalid_argument("invalid graph bounds");
        const auto is_cancelled = [&] { return cancelled && cancelled(cancel_context); };
        if (is_cancelled()) return 2;
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values values;
        bool anchored=false;
        for (std::size_t i=0;i<node_count;++i) {
            values.insert(i,pose(initial[i]));
            if (fixed[i]) {
                graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(i,pose(initial[i]),
                    gtsam::noiseModel::Isotropic::Variance(6,1e-12));
                anchored=true;
            }
        }
        if (!anchored) throw std::invalid_argument("graph has no fixed anchor");
        for (std::size_t i=0;i<edge_count;++i) {
            if (is_cancelled()) return 2;
            const auto& edge=edges[i];
            if (edge.from>=node_count || edge.to>=node_count || edge.from==edge.to)
                throw std::invalid_argument("invalid edge endpoints");
            gtsam::Matrix6 info;
            // GTSAM的切空间是旋转在前；同时置换行和列，不能只换残差或对角线。
            for (int r=0;r<6;++r) for (int c=0;c<6;++c) {
                const double v=edge.information[((r+3)%6)*6+(c+3)%6];
                if (!std::isfinite(v)) throw std::invalid_argument("non-finite information");
                info(r,c)=v;
            }
            if (!info.isApprox(info.transpose(),1e-10)
                || Eigen::LLT<gtsam::Matrix6>(info).info()!=Eigen::Success)
                throw std::invalid_argument("information must be positive definite and symmetric");
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                edge.from,edge.to,pose(edge.measurement),gtsam::noiseModel::Gaussian::Information(info));
        }
        gtsam::LevenbergMarquardtParams params;
        if (!std::isfinite(gradient_tolerance) || gradient_tolerance<0
            || !std::isfinite(initial_lambda) || initial_lambda<=0
            || !std::isfinite(lambda_scale) || lambda_scale<=1)
            throw std::invalid_argument("invalid solver parameters");
        params.maxIterations=max_iterations;
        params.lambdaInitial=initial_lambda;
        params.lambdaFactor=lambda_scale;
        params.relativeErrorTol=1e-10;
        params.absoluteErrorTol=1e-12;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph,values,params);
        const double initial_cost=graph.error(values);
        bool converged=max_iterations==0;
        for (std::uint32_t i=0;i<max_iterations;++i) {
            if (is_cancelled()) return 2;
            if (graph.linearize(optimizer.values())->gradientAtZero().norm() <= gradient_tolerance) { converged=true; break; }
            const double before=optimizer.error();
            optimizer.iterate();
            if (!std::isfinite(optimizer.error())) throw std::runtime_error("non-finite optimized cost");
            if (std::abs(before-optimizer.error()) <= 1e-12+1e-10*std::abs(before)) { converged=true; break; }
        }
        auto optimized=optimizer.values();
        for (std::size_t i=0;i<node_count;++i) if (fixed[i]) optimized.update(i,pose(initial[i]));
        const double final_cost=graph.error(optimized);
        if (!std::isfinite(final_cost) || final_cost > initial_cost + 1e-9*(1+initial_cost))
            throw std::runtime_error("optimization did not preserve cost");
        if (is_cancelled()) return 2;
        // 所有结果校验完成后一次复制，异常或取消不泄漏部分位姿。
        std::vector<NuwaxGraphPose> prepared;
        prepared.reserve(node_count);
        for (std::size_t i=0;i<node_count;++i) prepared.push_back(fixed[i] ? initial[i] : wire(optimized.at<gtsam::Pose3>(i)));
        std::copy(prepared.begin(),prepared.end(),output);
        *result={initial_cost,final_cost,static_cast<std::uint32_t>(optimizer.iterations()),static_cast<std::uint8_t>(converged)};
        return 0;
    } catch (const std::exception& e) {
        if (error && error_capacity) {
            const auto length=std::min(error_capacity-1,std::strlen(e.what()));
            std::memcpy(error,e.what(),length); error[length]='\0';
        }
        return 1;
    } catch (...) {
        if (error && error_capacity) error[0]='\0';
        return 1;
    }
}
