// Included inside surface_follower.rs tests; original prepared planar fixture.
#[test]
fn zero_yaw_legacy_surface_preserves_exact_acceptance_and_positive_control_floor() {
    let map = test_snapshot(0.0, false);
    assert!(map.body_envelope().is_none());
    for (raw_yaw, actual_yaw, succeeds) in [
        (0.0, 0.0, true),
        (0.0, 1e-7, false),
        (1e-8, 1e-7, true),
    ] {
        let config = follower_config();
        assert!(!config.controller.simulation);
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(prepared_test_surface(map.clone())).unwrap();
        let goal = body_pose([0.55, 1.55, 0.5], Quaternion::IDENTITY, 0);
        let actual = body_pose([0.55, 1.55, 0.5], Quaternion::from_yaw(actual_yaw), 0);
        crate::validate_map_pose(&actual, "odometry").unwrap();
        let actual_error = angle_diff(actual.pose.orientation.yaw(), goal.pose.orientation.yaw()).abs();
        if actual_yaw != 0.0 { assert!(actual_error > 0.0 && actual_error < 1e-6); }
        let options = GoalOptions {
            position_tolerance_m: 0.0, yaw_tolerance_rad: raw_yaw, allow_reverse: false,
        };
        follower.configure_goal(&goal, options).unwrap();
        assert_eq!(follower.config.controller.orientation_tolerance, 1e-6,
            "the controller floor is retained; only raw-zero success is qualified exactly");
        follower.update_odometry(actual);
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![goal.clone(), goal.clone()],
        }), None, goal).unwrap();
        let step = follower.step(0.3);
        if succeeds { assert_eq!(step, PlannerStep::Succeeded); }
        else { assert!(matches!(step, PlannerStep::Command(_) | PlannerStep::ReplanRequired(_)),
            "nonzero yaw cannot become a zero-goal success: {step:?}"); }
    }
}
