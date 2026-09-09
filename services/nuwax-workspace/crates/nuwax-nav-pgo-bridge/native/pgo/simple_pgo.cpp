#include "simple_pgo.h"
#include "solver.h"

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

namespace {
NuwaxGraphPose graphPose(const RigidPose& p) {
    return {{p.t.x,p.t.y,p.t.z},{p.q.x,p.q.y,p.q.z,p.q.w}};
}
void graphInputs(const std::vector<KeyPoseSe3>& poses, const std::vector<PoseConstraintSe3>& constraints,
                 std::vector<NuwaxGraphPose>& nodes, std::vector<NuwaxGraphEdge>& edges) {
    for (const auto& pose:poses) nodes.push_back(graphPose(pose.global));
    for (const auto& constraint:constraints) {
        NuwaxGraphEdge edge{};
        edge.from=constraint.source_id; edge.to=constraint.target_id;
        edge.measurement=graphPose(constraint.target_from_source);
        const double weights[6]={1e4,1e4,1e6,1e6,1e6,1e6};
        for (int i=0;i<6;++i) edge.information[i*6+i]=constraint.loop
            ? (i<3 ? std::min(constraint.weight,1e4) : 20.0) : weights[i];
        edges.push_back(edge);
    }
}
int cancelledGraph(void* context) {
    return static_cast<const std::atomic<bool>*>(context)->load(std::memory_order_acquire) ? 1 : 0;
}
}

bool SimplePGO::costSe3(double& result) const {
    if (key_poses_se3_.empty()) { result=0; return constraints_se3_.empty(); }
    std::vector<NuwaxGraphPose> nodes;
    std::vector<NuwaxGraphEdge> edges;
    graphInputs(key_poses_se3_,constraints_se3_,nodes,edges);
    auto output=nodes;
    std::vector<std::uint8_t> fixed(nodes.size(),0); fixed[0]=1;
    NuwaxGraphResult summary{}; char error[512]{};
    const int status=nuwax_gtsam_optimize(nodes.data(),fixed.data(),nodes.size(),edges.data(),edges.size(),
        0,1e-6,1e-4,10.0,nullptr,nullptr,output.data(),&summary,error,sizeof(error));
    result=summary.initial_cost;
    return status==0 && std::isfinite(result);
}

Se3OptimizationSummary SimplePGO::smoothAndUpdateSe3(const std::atomic<bool>& cancelled) {
    Se3OptimizationSummary summary;
    if (cancelled.load(std::memory_order_acquire)) return summary;
    if (key_poses_se3_.empty()) { summary.success=true; summary.converged=true; return summary; }
    std::vector<NuwaxGraphPose> nodes;
    std::vector<NuwaxGraphEdge> edges;
    graphInputs(key_poses_se3_,constraints_se3_,nodes,edges);
    auto output=nodes;
    std::vector<std::uint8_t> fixed(nodes.size(),0); fixed[0]=1;
    NuwaxGraphResult result{}; char error[512]{};
    const int status=nuwax_gtsam_optimize(nodes.data(),fixed.data(),nodes.size(),edges.data(),edges.size(),
        static_cast<std::uint32_t>(config_.optimizer_iterations),1e-6,1e-4,10.0,
        cancelledGraph,const_cast<std::atomic<bool>*>(&cancelled),output.data(),&result,error,sizeof(error));
    summary.cost_before=result.initial_cost; summary.cost_after=result.final_cost;
    summary.iterations=static_cast<int>(result.iterations); summary.converged=result.converged!=0;
    if (status!=0 || !summary.converged || cancelled.load(std::memory_order_acquire)) return summary;
    auto prepared=key_poses_se3_;
    for (std::size_t i=0;i<output.size();++i) {
        const auto& p=output[i];
        prepared[i].global={{p.translation[0],p.translation[1],p.translation[2]},
            {p.rotation_xyzw[0],p.rotation_xyzw[1],p.rotation_xyzw[2],p.rotation_xyzw[3]}};
        if (!isFinite(prepared[i].global)) return summary;
    }
    const auto& last=prepared.back();
    const RigidPose correction=composePose(last.global,inversePose(last.local.pose));
    if (!isFinite(correction)) return summary;
    key_poses_se3_=std::move(prepared);
    offset_se3_=correction; summary.success=true;
    return summary;
}
