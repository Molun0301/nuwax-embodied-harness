//! Current-state view ownership regressions and isolated release attribution.
use super::*;
use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
use nuwax_nav_types_messages::{MotionModel, NavFrameId, Pose3, Timestamp};

fn recheck_pose(x: f64) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(1, 0),
        pose: Pose3 {
            position: Point3::new(x, 0.55, 0.4),
            orientation: Quaternion::IDENTITY,
        },
    }
}

fn current_view_fixture(
    side: i32,
    fallback: bool,
) -> (SurfaceFollower, Arc<SurfaceSnapshot>, Twist2) {
    let mut points: Vec<_> = (0..side)
        .flat_map(|x| {
            (0..side / 2).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
        })
        .collect();
    if fallback {
        points.push((0.55, 0.55, 0.45));
    }
    let source = Arc::new(
        SurfaceSnapshot::from_points(
            &points,
            super::super::test_snapshot(0.0, false).config().clone(),
            0.1,
        )
        .unwrap()
        .with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.15, -0.15], [0.15, -0.15], [0.15, 0.15], [-0.15, 0.15]],
            z_bounds_m: [-0.05, 0.2],
            body_to_support_z_m: 0.3,
            max_step_up_m: 0.15,
            max_step_down_m: 0.15,
            max_cross_slope_deg: 15.0,
            support_uncertainty_m: 0.0,
        })
        .unwrap(),
    );
    let mut config = SurfaceFollowerConfig::new(0.3);
    config.controller.control_frequency = 20.0;
    config.controller.speed = 0.5;
    config.motion_limits = Some(MotionLimits {
        model: MotionModel::Differential,
        max_forward_mps: 0.5,
        max_reverse_mps: 0.0,
        max_lateral_mps: 0.0,
        max_angular_rps: 0.8,
        linear_acceleration_mps2: 1.0,
        guaranteed_braking_mps2: 1.0,
        angular_acceleration_rps2: 2.0,
        guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.05,
        minimum_turning_radius_m: 0.0,
        can_rotate_in_place: true,
    });
    let mut follower = SurfaceFollower::new(config).unwrap();
    follower
        .update_execution_map(ExecutionMap::Surface(source.clone()))
        .unwrap();
    follower.update_odometry(recheck_pose(0.55));
    follower.refresh_current_body_view();
    assert_eq!(
        !follower.map.as_ref().unwrap().same_body_query_view(&source),
        fallback
    );
    follower.control_target = Some(PlanarPose {
        x: 0.85,
        y: 0.55,
        yaw: 0.0,
    });
    let proposed = Twist2 {
        linear: Point2::new(0.05, 0.0),
        angular_z: 0.0,
    };
    let selected = follower.bounded_body_command(proposed, None);
    let PlannerStep::Command(command) = selected else {
        panic!("missing selection: {selected:?}");
    };
    assert_ne!(command, Twist2::ZERO);
    follower.observe_applied_command(command);
    (follower, source, command)
}

#[test]
#[ignore = "isolated release current-view recheck attribution; not a control deadline guarantee"]
fn current_pose_recheck_then_view_refresh_cost() {
    use crate::control_profile::{begin, end, Stage};
    const ROUNDS: usize = 30;
    for side in [40, 160] {
        for fallback in [false, true] {
            let (mut follower, source, command) = current_view_fixture(side, fallback);
            let mut samples = Vec::new();
            for round in 0..ROUNDS {
                follower.update_odometry(recheck_pose(0.55 + (round % 2) as f64 * 0.001));
                let started = std::time::Instant::now();
                begin();
                let checked = follower.recheck_selected_body_command_at_current_state(command);
                let record = end();
                let recheck_ns = started.elapsed().as_nanos();
                assert!(
                    matches!(checked, Some(PlannerStep::Command(c)) if c == command),
                    "{checked:?}"
                );
                let dirty_after = follower.body_view_dirty;
                let started = std::time::Instant::now();
                follower.refresh_current_body_view();
                let followup_ns = started.elapsed().as_nanos();
                follower
                    .map
                    .as_ref()
                    .unwrap()
                    .validate_body_pose(
                        body(follower.odometry.as_ref().unwrap()).0,
                        body(follower.odometry.as_ref().unwrap()).1,
                    )
                    .unwrap();
                samples.push((
                    recheck_ns,
                    record.wall_nanos[Stage::CurrentViewRefresh as usize],
                    followup_ns,
                ));
                println!("CURRENT_VIEW_RECHECK side={side} fallback={fallback} round={round} recheck_ns={recheck_ns} rebuild_ns={} followup_ns={followup_ns} dirty_after={dirty_after}",
                    record.wall_nanos[Stage::CurrentViewRefresh as usize]);
            }
            assert_eq!(samples.len(), ROUNDS);
            assert_eq!(
                source
                    .validate_body_pose([0.55, 0.55, 0.4], [0.0, 0.0, 0.0, 1.0])
                    .is_err(),
                fallback
            );
        }
    }
}

#[test]
fn current_recheck_keeps_actual_view_but_old_pose_replay_restores_owner_view() {
    let (mut follower, source, command) = current_view_fixture(40, true);
    let original_view = follower.map.clone().unwrap();
    follower.update_odometry(recheck_pose(0.551));
    assert!(follower.body_view_dirty);
    assert!(
        matches!(follower.recheck_selected_body_command_at_current_state(command),
        Some(PlannerStep::Command(c)) if c == command)
    );
    assert!(!follower.body_view_dirty);
    let checked_view = follower.map.clone().unwrap();
    assert!(!Arc::ptr_eq(&original_view, &checked_view));
    follower.refresh_current_body_view();
    assert!(Arc::ptr_eq(&checked_view, follower.map.as_ref().unwrap()));
    assert!(Arc::ptr_eq(&source, follower.source_map.as_ref().unwrap()));
    assert!(source
        .validate_body_pose(body(&recheck_pose(0.55)).0, body(&recheck_pose(0.55)).1)
        .is_err());
    original_view
        .validate_body_pose(body(&recheck_pose(0.55)).0, body(&recheck_pose(0.55)).1)
        .unwrap();

    // Old selection replay is allowed to query the selected pose temporarily,
    // but must restore the latest owner's independently derived view afterward.
    let latest = recheck_pose(1.55);
    follower.update_odometry(latest.clone());
    follower.refresh_current_body_view();
    let latest_view = follower.map.clone().unwrap();
    assert!(latest_view.same_body_query_view(&source));
    assert!(matches!(follower.recheck_selected_body_command(command),
        Some(PlannerStep::Command(c)) if c == command));
    assert!(Arc::ptr_eq(&latest_view, follower.map.as_ref().unwrap()));
    assert_eq!(follower.odometry.as_ref(), Some(&latest));
    assert!(!follower.body_view_dirty);
    assert!(
        follower
            .map
            .as_ref()
            .unwrap()
            .validate_body_pose(body(&recheck_pose(0.55)).0, body(&recheck_pose(0.55)).1)
            .is_err(),
        "old inferred footprint cannot accumulate"
    );
    assert!(
        matches!(follower.recheck_selected_body_command_at_current_state(command),
        Some(PlannerStep::Command(c)) if c == command)
    );
    assert_eq!(
        follower.selected_body_command.as_ref().unwrap().pose,
        latest
    );
}

#[test]
fn current_recheck_failure_keeps_latest_recent_rejection_and_drops_selection() {
    let (mut follower, source, command) = current_view_fixture(40, true);
    let old_view = follower.map.clone().unwrap();
    let blocked = Arc::new(
        source
            .with_recent_obstacle_points(1, 1_000_000_000, 0, &[], [(0, [0.56, 0.55, 0.4])])
            .unwrap(),
    );
    follower
        .update_execution_map(ExecutionMap::Surface(blocked.clone()))
        .unwrap();
    follower.update_odometry(recheck_pose(0.551));
    assert!(matches!(
        follower.recheck_selected_body_command_at_current_state(command),
        Some(PlannerStep::ReplanRequired(_))
    ));
    assert!(follower.selected_body_command.is_none());
    assert_eq!(follower.last_command, Twist2::ZERO);
    assert!(follower.last_command_dispatch_bound.is_none());
    assert!(!follower.body_view_dirty);
    assert!(Arc::ptr_eq(&blocked, follower.source_map.as_ref().unwrap()));
    let latest_view = follower.map.clone().unwrap();
    assert!(latest_view
        .validate_body_pose(body(&recheck_pose(0.551)).0, body(&recheck_pose(0.551)).1)
        .is_err());
    follower.refresh_current_body_view();
    assert!(Arc::ptr_eq(&latest_view, follower.map.as_ref().unwrap()));
    old_view
        .validate_body_pose(body(&recheck_pose(0.55)).0, body(&recheck_pose(0.55)).1)
        .unwrap();
    follower
        .update_execution_map(ExecutionMap::Surface(source))
        .unwrap();
    assert!(follower.body_view_dirty);
    assert!(follower
        .recheck_selected_body_command_at_current_state(command)
        .is_none());
    follower.refresh_current_body_view();
    assert!(!Arc::ptr_eq(&latest_view, follower.map.as_ref().unwrap()));
    follower
        .map
        .as_ref()
        .unwrap()
        .validate_body_pose(body(&recheck_pose(0.551)).0, body(&recheck_pose(0.551)).1)
        .unwrap();
}

#[test]
fn current_recheck_without_map_cannot_restore_an_old_view() {
    let (mut follower, _, command) = current_view_fixture(40, true);
    follower.invalidate_execution_map();
    assert!(follower.map.is_none() && follower.source_map.is_none());
    assert!(!matches!(
        follower.recheck_selected_body_command_at_current_state(command),
        Some(PlannerStep::Command(_))
    ));
    assert!(follower.map.is_none() && follower.source_map.is_none());
    assert!(follower.selected_body_command.is_none());
    assert_eq!(follower.last_command, Twist2::ZERO);
}

#[test]
fn current_recheck_still_forces_refresh_for_signed_zero_pose_change() {
    let (mut follower, _, command) = current_view_fixture(40, false);
    let old_view = follower.map.clone().unwrap();
    let mut latest = recheck_pose(0.55);
    latest.pose.orientation.x = -0.0;
    follower.update_odometry(latest.clone());
    assert!(
        !follower.body_view_dirty,
        "numeric pose equality ignores signed zero"
    );
    assert!(
        matches!(follower.recheck_selected_body_command_at_current_state(command),
        Some(PlannerStep::Command(c)) if c == command)
    );
    assert!(!Arc::ptr_eq(&old_view, follower.map.as_ref().unwrap()));
    assert_eq!(
        follower
            .selected_body_command
            .as_ref()
            .unwrap()
            .pose
            .pose
            .orientation
            .x
            .to_bits(),
        (-0.0_f64).to_bits()
    );
    assert_eq!(
        follower
            .odometry
            .as_ref()
            .unwrap()
            .pose
            .orientation
            .x
            .to_bits(),
        (-0.0_f64).to_bits()
    );
    assert!(!follower.body_view_dirty);
}
