//! 原生执行回归使用的真实地图与运动标定。
use crate::{SurfaceFollowerConfig, SurfaceSnapshot};
use nuwax_nav_types_messages::{MotionLimits, MotionModel};
use std::sync::Arc;

/// 显式声明本测试只提供静态地图，不冒充生产注册点云原帧。
pub(crate) fn author_static_map_follower(config: SurfaceFollowerConfig) -> crate::SurfaceFollower {
    crate::SurfaceFollower::with_map_initializer(config,Some(static_map)).unwrap()
}

/// 末段到达现在需要先用静态地图检查平移；等待真实地图工作线程交付。
pub(crate) fn prepare_terminal_map(follower: &mut crate::SurfaceFollower) {
    use crate::NavPlanner;
    let until = std::time::Instant::now() + std::time::Duration::from_secs(1);
    while follower.terminal_phase() == crate::TerminalPhase::Travel {
        follower.step();
        assert!(std::time::Instant::now() < until, "静态地图未进入末段：{:?}", follower.execution_wait_reason());
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

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
    let mut config = SurfaceFollowerConfig::new(0.4, 0.15);
    config.controller.simulation = false;
    config.controller.control_frequency = 20.;
    config.motion_limits = Some(MotionLimits {
        model: MotionModel::Differential,
        max_forward_mps: 1.,
        max_reverse_mps: 0.,
        max_lateral_mps: 0.,
        max_angular_rps: 0.8, min_executable_linear_mps: 0., min_executable_angular_rps: 0.,
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

fn static_map(map:&mut nuwax_nav_scan_native::MapOwner,obstacles:&[[f64;3]],ground:&[[f64;3]],boundary:&[[f64;2]],forbidden:&[[i32;2]],body:[f64;3])->Result<(),String>{
    let points:Vec<_>=obstacles.iter().chain(ground).copied().collect();map.seed_occupied(&points,body);
    map.update_surface(&[],&[],boundary,forbidden)
}
