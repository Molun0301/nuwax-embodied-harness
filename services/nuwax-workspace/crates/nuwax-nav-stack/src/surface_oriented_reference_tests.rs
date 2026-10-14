use super::*;
use crate::surface_config::SurfaceNavigationConfig;

fn scene(wall: bool) -> (SurfaceSnapshot, nuwax_nav_pct::PctPlanner, MotionLimits) {
    let mut config =
        SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.15, 18.0, 0.2).unwrap();
    config
        .apply_calibration(
            Some(nuwax_nav_types_messages::RobotGeometry {
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_min_m: -0.05,
                z_max_m: 0.2,
                source: "mission reference fixture".into(),
            }),
            None,
        )
        .unwrap();
    // Valid remote observations; neither requested endpoint has measured ground.
    let mut points = vec![
        (3.05_f32, 3.05_f32, 0.05_f32),
        (3.15, 3.05, 0.05),
        (3.05, 3.15, 0.05),
    ];
    if wall {
        for y in -3..=3 {
            for z in 1..=7 {
                points.push((0.55, y as f32 * 0.1 + 0.05, z as f32 * 0.1 + 0.05));
            }
        }
    }
    let map = SurfaceSnapshot::from_points(&points, config.geometry, 0.1)
        .unwrap()
        .with_body_envelope(config.body_envelope.unwrap())
        .unwrap();
    let motion = MotionLimits {
        model: MotionModel::Differential,
        max_forward_mps: 0.5,
        max_reverse_mps: 0.0,
        max_lateral_mps: 0.0,
        max_angular_rps: 1.0,
        linear_acceleration_mps2: 1.0,
        guaranteed_braking_mps2: 1.0,
        angular_acceleration_rps2: 2.0,
        guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.05,
        minimum_turning_radius_m: 0.0,
        can_rotate_in_place: true,
    };
    (map, nuwax_nav_pct::PctPlanner::default(), motion)
}
fn request() -> (BodyPose, BodyPose, GoalOptions) {
    (
        ([0.0, 0.0, 0.45], [0.0, 0.0, 0.0, 1.0]),
        (
            [1.0, 0.0, 0.45],
            [
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        ),
        GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.1,
        },
    )
}

#[test]
fn mission_reference_crosses_unknown_with_real_body_endpoint_and_requested_yaw() {
    let (map, planner, motion) = scene(false);
    let (start, goal, options) = request();
    assert!(map.locate_support_from_body(start.0, start.1, 0.4).is_err());
    assert!(map.locate_support_from_body(goal.0, goal.1, 0.4).is_err());
    let path = plan_reference_cancellable(
        &map,
        &planner,
        start,
        goal,
        options,
        Some(&motion),
        &[],
        30_000,
        None,
    )
    .unwrap();
    let end = *path.last().unwrap();
    assert!(position_distance(end, goal) <= options.position_tolerance_m);
    assert!(angle(planar(end).yaw - planar(goal).yaw).abs() <= options.yaw_tolerance_rad);
    assert!(path
        .windows(2)
        .all(|pair| map.validate_body_motion(pair[0], pair[1], &[]).is_ok()));
    assert!(
        path.len() > 2,
        "actual translation and rotation samples must replace the two-point reference"
    );
}

#[test]
fn mission_reference_keeps_known_obstacle_boundary_and_height_constraints() {
    let (map, planner, motion) = scene(true);
    let (start, goal, options) = request();
    let boundary = [[-0.2, -0.15], [1.2, -0.15], [1.2, 0.15], [-0.2, 0.15]];
    assert!(plan_reference_cancellable(
        &map,
        &planner,
        start,
        goal,
        options,
        Some(&motion),
        &boundary,
        30_000,
        None
    )
    .is_err());
    let (map, planner, motion) = scene(false);
    let other_floor = ([goal.0[0], goal.0[1], 3.0], goal.1);
    assert!(plan_reference_cancellable(
        &map,
        &planner,
        start,
        other_floor,
        options,
        Some(&motion),
        &[],
        30_000,
        None
    )
    .is_err());
}

#[test]
fn mission_reference_cannot_assign_yaw_to_a_nonrotating_platform() {
    let (map, planner, mut motion) = scene(false);
    let (start, goal, options) = request();
    motion.can_rotate_in_place = false;
    motion.minimum_turning_radius_m = 0.5;
    let rotate_only = (start.0, goal.1);
    let error = plan_reference_cancellable(
        &map,
        &planner,
        start,
        rotate_only,
        options,
        Some(&motion),
        &[],
        30_000,
        None,
    )
    .unwrap_err();
    assert!(
        error.contains("ResourceLimited"),
        "unrefined reference must remain inconclusive: {error}"
    );
}

#[test]
fn mission_reference_refines_fifty_two_meter_shortcut_without_raising_action_cap() {
    let (base, planner, motion) = scene(false);
    let map = SurfaceSnapshot::from_points(&[(3.05, 3.05, 0.05)], base.config().clone(), 0.05)
        .unwrap()
        .with_body_envelope(base.body_envelope().unwrap().clone())
        .unwrap();
    let (start, mut goal, options) = request();
    goal.0[0] = 52.0;
    let path = plan_reference_cancellable(
        &map,
        &planner,
        start,
        goal,
        options,
        Some(&motion),
        &[],
        30_000,
        None,
    )
    .unwrap();
    assert!(path.len() > MAX_ACTION_SAMPLES);
    assert!(path
        .windows(2)
        .all(|p| map.validate_body_motion(p[0], p[1], &[]).is_ok()));
    let end = *path.last().unwrap();
    assert!(position_distance(end, goal) <= options.position_tolerance_m);
    assert!(angle(planar(end).yaw - planar(goal).yaw).abs() <= options.yaw_tolerance_rad);
}
