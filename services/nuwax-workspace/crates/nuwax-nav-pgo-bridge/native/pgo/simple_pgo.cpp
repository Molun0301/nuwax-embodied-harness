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

double quaternionNormSquared(const Quaternion& quaternion) {
    return squared(quaternion.x) + squared(quaternion.y) + squared(quaternion.z) +
           squared(quaternion.w);
}

bool isFinite(const Quaternion& quaternion) {
    return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
           std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

bool normalizeQuaternion(Quaternion& quaternion) {
    if (!isFinite(quaternion)) return false;
    const double norm_squared = quaternionNormSquared(quaternion);
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-18) return false;
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    quaternion.x *= inverse_norm;
    quaternion.y *= inverse_norm;
    quaternion.z *= inverse_norm;
    quaternion.w *= inverse_norm;
    if (quaternion.w < 0.0) {
        quaternion.x = -quaternion.x;
        quaternion.y = -quaternion.y;
        quaternion.z = -quaternion.z;
        quaternion.w = -quaternion.w;
    }
    return isFinite(quaternion);
}

double quaternionDot(const Quaternion& left, const Quaternion& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
}

Quaternion quaternionConjugate(const Quaternion& quaternion) {
    return {-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w};
}

Quaternion quaternionMultiply(const Quaternion& left, const Quaternion& right) {
    return {
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    };
}

Vec3 rotateVector(const Quaternion& quaternion, const Vec3& value) {
    const Vec3 twice_cross{
        2.0 * (quaternion.y * value.z - quaternion.z * value.y),
        2.0 * (quaternion.z * value.x - quaternion.x * value.z),
        2.0 * (quaternion.x * value.y - quaternion.y * value.x),
    };
    return {
        value.x + quaternion.w * twice_cross.x +
            (quaternion.y * twice_cross.z - quaternion.z * twice_cross.y),
        value.y + quaternion.w * twice_cross.y +
            (quaternion.z * twice_cross.x - quaternion.x * twice_cross.z),
        value.z + quaternion.w * twice_cross.z +
            (quaternion.x * twice_cross.y - quaternion.y * twice_cross.x),
    };
}

RigidPose composePose(const RigidPose& left, const RigidPose& right) {
    const Vec3 rotated = rotateVector(left.q, right.t);
    RigidPose result{{left.t.x + rotated.x, left.t.y + rotated.y, left.t.z + rotated.z},
                     quaternionMultiply(left.q, right.q)};
    normalizeQuaternion(result.q);
    return result;
}

RigidPose inversePose(const RigidPose& pose) {
    const Quaternion inverse_rotation = quaternionConjugate(pose.q);
    const Vec3 inverse_translation =
        rotateVector(inverse_rotation, {-pose.t.x, -pose.t.y, -pose.t.z});
    return {inverse_translation, inverse_rotation};
}

RigidPose betweenPoses(const RigidPose& source, const RigidPose& target) {
    return composePose(inversePose(source), target);
}

bool isFinite(const RigidPose& pose) {
    return isFinite(pose.t) && isFinite(pose.q) &&
           std::abs(quaternionNormSquared(pose.q) - 1.0) <= 1.0e-6;
}

double quaternionAngle(const Quaternion& quaternion) {
    Quaternion normalized = quaternion;
    if (!normalizeQuaternion(normalized)) return std::numeric_limits<double>::quiet_NaN();
    return 2.0 * std::acos(std::clamp(std::abs(normalized.w), 0.0, 1.0));
}

Quaternion slerpQuaternion(const Quaternion& start, const Quaternion& finish, double amount) {
    Quaternion target = finish;
    double cosine = quaternionDot(start, target);
    if (cosine < 0.0) {
        target.x = -target.x;
        target.y = -target.y;
        target.z = -target.z;
        target.w = -target.w;
        cosine = -cosine;
    }
    cosine = std::clamp(cosine, -1.0, 1.0);
    Quaternion result;
    if (cosine > 0.9995) {
        result = {
            start.x + amount * (target.x - start.x),
            start.y + amount * (target.y - start.y),
            start.z + amount * (target.z - start.z),
            start.w + amount * (target.w - start.w),
        };
    } else {
        const double angle = std::acos(cosine);
        const double sine = std::sin(angle);
        const double start_scale = std::sin((1.0 - amount) * angle) / sine;
        const double target_scale = std::sin(amount * angle) / sine;
        result = {
            start_scale * start.x + target_scale * target.x,
            start_scale * start.y + target_scale * target.y,
            start_scale * start.z + target_scale * target.z,
            start_scale * start.w + target_scale * target.w,
        };
    }
    normalizeQuaternion(result);
    return result;
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

void PoseWithTimeSe3::setTime(int32_t seconds, uint32_t nanoseconds) {
    sec = seconds;
    nsec = nanoseconds;
    second = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
}

bool SimplePGO::isKeyPoseSe3(const PoseWithTimeSe3& pose) const {
    RigidPose normalized_pose = pose.pose;
    if (!std::isfinite(pose.second) || !normalizeQuaternion(normalized_pose.q) ||
        !isFinite(normalized_pose)) {
        return false;
    }
    if (key_poses_se3_.empty()) return true;
    const auto& last = key_poses_se3_.back().local.pose;
    constexpr double radians_to_degrees = 57.29577951308232;
    const double angular_delta = quaternionAngle(quaternionMultiply(
        quaternionConjugate(last.q), normalized_pose.q));
    return distance(normalized_pose.t, last.t) > config_.key_pose_delta_trans ||
           angular_delta * radians_to_degrees > config_.key_pose_delta_deg;
}

bool SimplePGO::addKeyPoseSe3(const PoseWithTimeSe3& pose) {
    PoseWithTimeSe3 normalized = pose;
    if (!normalizeQuaternion(normalized.pose.q) || !isFinite(normalized.pose) ||
        !std::isfinite(normalized.second) || !isKeyPoseSe3(normalized)) {
        return false;
    }
    const RigidPose global = composePose(offset_se3_, normalized.pose);
    if (!isFinite(global)) return false;
    if (!key_poses_se3_.empty()) {
        const RigidPose relative =
            betweenPoses(key_poses_se3_.back().local.pose, normalized.pose);
        if (!isFinite(relative)) return false;
        constraints_se3_.push_back({key_poses_se3_.size() - 1,
                                    key_poses_se3_.size(), relative, 1.0, false});
    }
    key_poses_se3_.push_back({normalized, global});
    return true;
}

bool SimplePGO::addLoopPairSe3(std::size_t source_id, std::size_t target_id,
                               const RigidPose& target_from_source, double weight) {
    RigidPose normalized = target_from_source;
    if (source_id >= key_poses_se3_.size() || target_id >= key_poses_se3_.size() ||
        source_id == target_id || !normalizeQuaternion(normalized.q) || !isFinite(normalized) ||
        !std::isfinite(weight) || !(weight > 0.0)) {
        return false;
    }
    constraints_se3_.push_back({source_id, target_id, normalized, weight, true});
    return true;
}

std::size_t SimplePGO::loopConstraintCountSe3() const {
    return static_cast<std::size_t>(
        std::count_if(constraints_se3_.begin(), constraints_se3_.end(),
                      [](const PoseConstraintSe3& constraint) { return constraint.loop; }));
}

bool SimplePGO::costSe3(double& result) const {
    long double accumulated = 0.0;
    for (const auto& edge : constraints_se3_) {
        if (edge.source_id >= key_poses_se3_.size() ||
            edge.target_id >= key_poses_se3_.size()) {
            return false;
        }
        const RigidPose predicted = betweenPoses(key_poses_se3_[edge.source_id].global,
                                                key_poses_se3_[edge.target_id].global);
        const RigidPose error = composePose(inversePose(edge.target_from_source), predicted);
        const double angle = quaternionAngle(error.q);
        if (!isFinite(error) || !std::isfinite(angle)) return false;
        const long double squared_error =
            static_cast<long double>(error.t.x) * error.t.x +
            static_cast<long double>(error.t.y) * error.t.y +
            static_cast<long double>(error.t.z) * error.t.z +
            static_cast<long double>(angle) * angle;
        const long double contribution = static_cast<long double>(edge.weight) * squared_error;
        if (!std::isfinite(squared_error) || !std::isfinite(contribution)) return false;
        accumulated += contribution;
        if (!std::isfinite(accumulated) ||
            accumulated > static_cast<long double>(std::numeric_limits<double>::max())) {
            return false;
        }
    }
    result = static_cast<double>(accumulated);
    return std::isfinite(result);
}

Se3OptimizationSummary SimplePGO::smoothAndUpdateSe3(
    const std::atomic<bool>& cancelled) {
    Se3OptimizationSummary summary;
    if (cancelled.load(std::memory_order_acquire)) return summary;
    if (!costSe3(summary.cost_before)) return summary;
    summary.cost_after = summary.cost_before;
    summary.success = true;
    if (key_poses_se3_.size() < 2 || config_.optimizer_iterations == 0) {
        summary.converged = true;
        return summary;
    }

    struct Accumulator {
        Vec3 translation;
        Quaternion quaternion{0.0, 0.0, 0.0, 0.0};
        double weight{0.0};
    };

    const auto original_poses = key_poses_se3_;
    const RigidPose original_offset = offset_se3_;
    const auto rollback_cancelled = [&] {
        key_poses_se3_ = original_poses;
        offset_se3_ = original_offset;
        summary.success = false;
        summary.converged = false;
        summary.cost_after = summary.cost_before;
    };
    double current_cost = summary.cost_before;
    for (int iteration = 0; iteration < config_.optimizer_iterations; ++iteration) {
        if (cancelled.load(std::memory_order_acquire)) {
            rollback_cancelled();
            return summary;
        }
        std::vector<Accumulator> accumulators(key_poses_se3_.size());
        const auto accumulate = [&](std::size_t index, const RigidPose& proposal, double weight) {
            auto& accumulator = accumulators[index];
            accumulator.translation.x += weight * proposal.t.x;
            accumulator.translation.y += weight * proposal.t.y;
            accumulator.translation.z += weight * proposal.t.z;
            Quaternion aligned = proposal.q;
            if (quaternionDot(aligned, key_poses_se3_[index].global.q) < 0.0) {
                aligned.x = -aligned.x;
                aligned.y = -aligned.y;
                aligned.z = -aligned.z;
                aligned.w = -aligned.w;
            }
            accumulator.quaternion.x += weight * aligned.x;
            accumulator.quaternion.y += weight * aligned.y;
            accumulator.quaternion.z += weight * aligned.z;
            accumulator.quaternion.w += weight * aligned.w;
            accumulator.weight += weight;
        };

        for (const auto& edge : constraints_se3_) {
            if (cancelled.load(std::memory_order_acquire)) {
                rollback_cancelled();
                return summary;
            }
            const RigidPose target_proposal =
                composePose(key_poses_se3_[edge.source_id].global, edge.target_from_source);
            accumulate(edge.target_id, target_proposal, edge.weight);
            if (edge.source_id != 0) {
                const RigidPose source_proposal = composePose(
                    key_poses_se3_[edge.target_id].global, inversePose(edge.target_from_source));
                accumulate(edge.source_id, source_proposal, edge.weight);
            }
        }

        bool accepted = false;
        double step = config_.optimizer_step;
        double candidate_cost = current_cost;
        for (int line_search = 0; line_search < 10; ++line_search) {
            if (cancelled.load(std::memory_order_acquire)) {
                rollback_cancelled();
                return summary;
            }
            auto candidate = key_poses_se3_;
            bool finite_candidate = true;
            for (std::size_t index = 1; index < candidate.size(); ++index) {
                if (cancelled.load(std::memory_order_acquire)) {
                    rollback_cancelled();
                    return summary;
                }
                const auto& accumulator = accumulators[index];
                if (!(accumulator.weight > 0.0) || !std::isfinite(accumulator.weight)) continue;
                RigidPose average{
                    {accumulator.translation.x / accumulator.weight,
                     accumulator.translation.y / accumulator.weight,
                     accumulator.translation.z / accumulator.weight},
                    accumulator.quaternion,
                };
                finite_candidate = finite_candidate && normalizeQuaternion(average.q) &&
                                   isFinite(average);
                auto& updated = candidate[index].global;
                updated.t.x += step * (average.t.x - updated.t.x);
                updated.t.y += step * (average.t.y - updated.t.y);
                updated.t.z += step * (average.t.z - updated.t.z);
                updated.q = slerpQuaternion(updated.q, average.q, step);
                finite_candidate = finite_candidate && isFinite(updated);
            }
            if (!finite_candidate) break;
            const auto previous = key_poses_se3_;
            key_poses_se3_.swap(candidate);
            if (cancelled.load(std::memory_order_acquire)) {
                rollback_cancelled();
                return summary;
            }
            const bool finite_cost = costSe3(candidate_cost);
            if (finite_cost && candidate_cost < current_cost) {
                accepted = true;
                break;
            }
            key_poses_se3_ = previous;
            step *= 0.5;
        }

        if (!accepted) {
            summary.converged = true;
            break;
        }
        ++summary.iterations;
        const double improvement = current_cost - candidate_cost;
        current_cost = candidate_cost;
        if (improvement <= 1.0e-12 * std::max(1.0, current_cost)) {
            summary.converged = true;
            break;
        }
    }

    summary.cost_after = current_cost;
    if (!summary.converged && summary.iterations < config_.optimizer_iterations) {
        summary.converged = true;
    }
    if (cancelled.load(std::memory_order_acquire)) {
        rollback_cancelled();
        return summary;
    }
    const auto& last = key_poses_se3_.back();
    const RigidPose next_offset = composePose(last.global, inversePose(last.local.pose));
    if (!isFinite(next_offset) || !std::isfinite(summary.cost_after) ||
        summary.cost_after > summary.cost_before) {
        key_poses_se3_ = original_poses;
        offset_se3_ = original_offset;
        summary.success = false;
        summary.converged = false;
        summary.cost_after = summary.cost_before;
        return summary;
    }
    offset_se3_ = next_offset;
    return summary;
}
