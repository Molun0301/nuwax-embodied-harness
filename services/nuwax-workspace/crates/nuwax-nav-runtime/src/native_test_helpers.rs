//! 原生执行回归使用的真实地图与运动标定。
use crate::{SurfaceFollowerConfig, SurfaceSnapshot};
use nuwax_nav_types_messages::{MotionLimits, MotionModel};
use std::sync::Arc;

pub(crate) fn fixture() -> (SurfaceFollowerConfig, Arc<SurfaceSnapshot>) {
    let source = Arc::new(
        crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap(),
    );
    let mut config = SurfaceFollowerConfig::new(0.4);
    config.controller.simulation = false;
    config.controller.control_frequency = 20.;
    config.motion_limits = Some(MotionLimits {
        model: MotionModel::Differential,
        max_forward_mps: 1.,
        max_reverse_mps: 0.,
        max_lateral_mps: 0.,
        max_angular_rps: 0.8,
        linear_acceleration_mps2: 0.5,
        guaranteed_braking_mps2: 0.5,
        angular_acceleration_rps2: 1.,
        guaranteed_angular_braking_rps2: 1.,
        reaction_time_sec: 0.1,
        minimum_turning_radius_m: 0.,
        can_rotate_in_place: true,
    });
    (config, source)
}
