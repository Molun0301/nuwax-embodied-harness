#include "simple_pgo.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
double squared(double value) { return value * value; }
double distance(const Vec3& a, const Vec3& b) {
    return std::sqrt(squared(a.x - b.x) + squared(a.y - b.y) + squared(a.z - b.z));
}
bool isFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
bool isFinite(const PoseWithTime& pose) {
    return std::isfinite(pose.second) && isFinite(pose.t) && std::isfinite(pose.yaw);
}
bool isValid(const Config& config) {
    constexpr int max_optimizer_iterations = 1'000'000;
    return std::isfinite(config.key_pose_delta_deg) && config.key_pose_delta_deg >= 0.0 &&
           std::isfinite(config.key_pose_delta_trans) && config.key_pose_delta_trans >= 0.0 &&
           config.optimizer_iterations >= 0 &&
           config.optimizer_iterations <= max_optimizer_iterations &&
           std::isfinite(config.optimizer_step) && config.optimizer_step > 0.0 &&
           config.optimizer_step <= 1.0;
}
}

SimplePGO::SimplePGO(const Config& config) : config_(config) {
    if (!isValid(config_)) throw std::invalid_argument("invalid SimplePGO configuration");
}

bool SimplePGO::isKeyPose(const PoseWithTime& pose) const {
    if (!isFinite(pose)) return false;
    if (key_poses_.empty()) return true;
    const auto& last = key_poses_.back().local;
    constexpr double radians_to_degrees = 57.29577951308232;
    const double yaw_delta = wrapAngle(pose.yaw) - wrapAngle(last.yaw);
    return distance(pose.t, last.t) > config_.key_pose_delta_trans ||
           std::abs(wrapAngle(yaw_delta)) * radians_to_degrees >
               config_.key_pose_delta_deg;
}

bool SimplePGO::addKeyPose(const PoseWithTime& pose) {
    if (!isFinite(pose)) return false;
    if (!isKeyPose(pose)) return false;
    PoseWithTime global = pose;
    global.t.x += offset_t_.x;
    global.t.y += offset_t_.y;
    global.t.z += offset_t_.z;
    global.yaw = wrapAngle(wrapAngle(global.yaw) + offset_yaw_);
    if (!isFinite(global)) return false;
    if (!key_poses_.empty()) {
        const auto& previous = key_poses_.back().local;
        const Vec3 delta{pose.t.x - previous.t.x, pose.t.y - previous.t.y,
                         pose.t.z - previous.t.z};
        const double delta_yaw = wrapAngle(pose.yaw) - wrapAngle(previous.yaw);
        if (!isFinite(delta) || !std::isfinite(delta_yaw)) return false;
        constraints_.push_back({key_poses_.size() - 1, key_poses_.size(),
                                delta, wrapAngle(delta_yaw), 1.0, false});
    }
    key_poses_.push_back({pose, global});
    return true;
}

bool SimplePGO::addLoopPair(std::size_t source_id, std::size_t target_id, const Vec3& delta,
                            double delta_yaw, double weight) {
    if (source_id >= key_poses_.size() || target_id >= key_poses_.size() ||
        source_id == target_id || !isFinite(delta) || !std::isfinite(delta_yaw) ||
        !std::isfinite(weight) || !(weight > 0.0)) {
        return false;
    }
    constraints_.push_back(
        {source_id, target_id, delta, wrapAngle(delta_yaw), weight, true});
    return true;
}

bool SimplePGO::cost(double& result) const {
    long double accumulated = 0.0;
    for (const auto& edge : constraints_) {
        const auto& source = key_poses_[edge.source_id].global;
        const auto& target = key_poses_[edge.target_id].global;
        const long double dx = static_cast<long double>(target.t.x) - source.t.x - edge.delta.x;
        const long double dy = static_cast<long double>(target.t.y) - source.t.y - edge.delta.y;
        const long double dz = static_cast<long double>(target.t.z) - source.t.z - edge.delta.z;
        const long double dyaw = wrapAngle((target.yaw - source.yaw) - edge.delta_yaw);
        const long double squared_error = dx * dx + dy * dy + dz * dz + dyaw * dyaw;
        const long double contribution = static_cast<long double>(edge.weight) * squared_error;
        if (!std::isfinite(squared_error) || !std::isfinite(contribution)) {
            return false;
        }
        accumulated += contribution;
        if (!std::isfinite(accumulated) ||
            accumulated > static_cast<long double>(std::numeric_limits<double>::max())) {
            return false;
        }
    }
    result = static_cast<double>(accumulated);
    return std::isfinite(result);
}

OptimizationSummary SimplePGO::smoothAndUpdate() {
    OptimizationSummary summary;
    if (!cost(summary.cost_before)) return summary;
    summary.cost_after = summary.cost_before;
    summary.success = true;
    if (key_poses_.size() < 2) {
        return summary;
    }
    const auto original_poses = key_poses_;
    const auto original_offset_t = offset_t_;
    const double original_offset_yaw = offset_yaw_;
    double current_cost = summary.cost_before;
    for (int iteration = 0; iteration < config_.optimizer_iterations; ++iteration) {
        std::vector<Vec3> translation_gradient(key_poses_.size());
        std::vector<double> yaw_gradient(key_poses_.size(), 0.0);
        std::vector<double> weights(key_poses_.size(), 0.0);
        bool finite_iteration = true;
        for (const auto& edge : constraints_) {
            const auto& source = key_poses_[edge.source_id].global;
            const auto& target = key_poses_[edge.target_id].global;
            Vec3 residual{(target.t.x - source.t.x) - edge.delta.x,
                          (target.t.y - source.t.y) - edge.delta.y,
                          (target.t.z - source.t.z) - edge.delta.z};
            const double yaw_residual =
                wrapAngle((target.yaw - source.yaw) - edge.delta_yaw);
            if (edge.source_id != 0) {
                translation_gradient[edge.source_id].x -= edge.weight * residual.x;
                translation_gradient[edge.source_id].y -= edge.weight * residual.y;
                translation_gradient[edge.source_id].z -= edge.weight * residual.z;
                yaw_gradient[edge.source_id] -= edge.weight * yaw_residual;
                weights[edge.source_id] += edge.weight;
            }
            translation_gradient[edge.target_id].x += edge.weight * residual.x;
            translation_gradient[edge.target_id].y += edge.weight * residual.y;
            translation_gradient[edge.target_id].z += edge.weight * residual.z;
            yaw_gradient[edge.target_id] += edge.weight * yaw_residual;
            weights[edge.target_id] += edge.weight;
            finite_iteration = finite_iteration && isFinite(translation_gradient[edge.target_id]) &&
                               std::isfinite(yaw_gradient[edge.target_id]) &&
                               std::isfinite(weights[edge.target_id]);
            if (edge.source_id != 0) {
                finite_iteration = finite_iteration &&
                                   isFinite(translation_gradient[edge.source_id]) &&
                                   std::isfinite(yaw_gradient[edge.source_id]) &&
                                   std::isfinite(weights[edge.source_id]);
            }
        }
        if (!finite_iteration) {
            key_poses_ = original_poses;
            offset_t_ = original_offset_t;
            offset_yaw_ = original_offset_yaw;
            summary.success = false;
            summary.cost_after = summary.cost_before;
            return summary;
        }
        auto candidate_poses = key_poses_;
        for (std::size_t index = 1; index < key_poses_.size(); ++index) {
            const double scale = config_.optimizer_step / std::max(1.0, weights[index]);
            auto& pose = candidate_poses[index].global;
            pose.t.x -= scale * translation_gradient[index].x;
            pose.t.y -= scale * translation_gradient[index].y;
            pose.t.z -= scale * translation_gradient[index].z;
            pose.yaw = wrapAngle(pose.yaw - scale * yaw_gradient[index]);
            finite_iteration = finite_iteration && isFinite(pose);
        }
        if (!finite_iteration) {
            key_poses_ = original_poses;
            offset_t_ = original_offset_t;
            offset_yaw_ = original_offset_yaw;
            summary.success = false;
            summary.cost_after = summary.cost_before;
            return summary;
        }
        key_poses_.swap(candidate_poses);
        double candidate_cost = 0.0;
        if (!cost(candidate_cost)) {
            key_poses_ = original_poses;
            offset_t_ = original_offset_t;
            offset_yaw_ = original_offset_yaw;
            summary.success = false;
            summary.cost_after = summary.cost_before;
            return summary;
        }
        if (!(candidate_cost < current_cost)) {
            key_poses_.swap(candidate_poses);
            break;
        }
        current_cost = candidate_cost;
    }
    summary.cost_after = current_cost;
    const auto& last = key_poses_.back();
    const Vec3 next_offset{last.global.t.x - last.local.t.x,
                           last.global.t.y - last.local.t.y,
                           last.global.t.z - last.local.t.z};
    const double next_offset_yaw = wrapAngle(last.global.yaw) - wrapAngle(last.local.yaw);
    if (isFinite(next_offset) && std::isfinite(next_offset_yaw)) {
        offset_t_ = next_offset;
        offset_yaw_ = wrapAngle(next_offset_yaw);
    } else {
        key_poses_ = original_poses;
        offset_t_ = original_offset_t;
        offset_yaw_ = original_offset_yaw;
        summary.success = false;
        summary.cost_after = summary.cost_before;
    }
    return summary;
}
