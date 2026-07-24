#pragma once

#include "commons.h"

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

private:
    bool cost(double& result) const;
    Config config_;
    std::vector<KeyPose> key_poses_;
    std::vector<PoseConstraint> constraints_;
    Vec3 offset_t_;
    double offset_yaw_{0.0};
};
