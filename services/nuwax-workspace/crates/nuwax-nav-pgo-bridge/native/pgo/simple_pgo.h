#pragma once

#include "commons.h"

#include <atomic>
#include <cstddef>
#include <vector>

struct Config {
    double key_pose_delta_deg{10.0};
    double key_pose_delta_trans{0.5};
    int optimizer_iterations{300};
    double optimizer_step{0.2};
};

struct KeyPose {
    PoseWithTime local;
    PoseWithTime global;
};

struct PoseConstraint {
    std::size_t source_id;
    std::size_t target_id;
    Vec3 delta;
    double delta_yaw;
    double weight;
    bool loop;
};

struct OptimizationSummary {
    double cost_before{0.0};
    double cost_after{0.0};
    bool success{false};
};

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct RigidPose {
    Vec3 t;
    Quaternion q;
};

struct PoseWithTimeSe3 {
    RigidPose pose;
    int32_t sec{0};
    uint32_t nsec{0};
    double second{0.0};

    void setTime(int32_t seconds, uint32_t nanoseconds);
};

struct KeyPoseSe3 {
    PoseWithTimeSe3 local;
    RigidPose global;
};

struct PoseConstraintSe3 {
    std::size_t source_id;
    std::size_t target_id;
    RigidPose target_from_source;
    double weight;
    bool loop;
};

struct Se3OptimizationSummary {
    double cost_before{0.0};
    double cost_after{0.0};
    int iterations{0};
    bool converged{false};
    bool success{false};
};

class SimplePGO {
public:
    explicit SimplePGO(const Config& config);

    bool isKeyPose(const PoseWithTime& pose) const;
    bool addKeyPose(const PoseWithTime& pose);
    bool addLoopPair(std::size_t source_id, std::size_t target_id, const Vec3& delta,
                     double delta_yaw, double weight);
    OptimizationSummary smoothAndUpdate();

    const std::vector<KeyPose>& keyPoses() const { return key_poses_; }
    Vec3 offsetT() const { return offset_t_; }
    double offsetYaw() const { return offset_yaw_; }

    bool isKeyPoseSe3(const PoseWithTimeSe3& pose) const;
    bool addKeyPoseSe3(const PoseWithTimeSe3& pose);
    bool addLoopPairSe3(std::size_t source_id, std::size_t target_id,
                        const RigidPose& target_from_source, double weight);
    Se3OptimizationSummary smoothAndUpdateSe3(const std::atomic<bool>& cancelled);

    const std::vector<KeyPoseSe3>& keyPosesSe3() const { return key_poses_se3_; }
    std::size_t constraintCountSe3() const { return constraints_se3_.size(); }
    std::size_t loopConstraintCountSe3() const;
    RigidPose offsetSe3() const { return offset_se3_; }

private:
    bool cost(double& result) const;
    bool costSe3(double& result) const;
    Config config_;
    std::vector<KeyPose> key_poses_;
    std::vector<PoseConstraint> constraints_;
    Vec3 offset_t_;
    double offset_yaw_{0.0};
    std::vector<KeyPoseSe3> key_poses_se3_;
    std::vector<PoseConstraintSe3> constraints_se3_;
    RigidPose offset_se3_;
};
