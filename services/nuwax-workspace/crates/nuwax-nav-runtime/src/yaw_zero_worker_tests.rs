// DRAFT ONLY: include inside planner_worker.rs tests. Real OFF RouteFollower,
// existing publication gate and panic-safe harness; no injected terminal result.
#[test]
fn zero_yaw_real_arrival_requalification_accepts_unchanged_yaw_after_translation() {
    let (config, clear) = crate::native_test_helpers::fixture();
    assert!(!config.controller.simulation);
    let (computed, receive) = std::sync::mpsc::channel();
    let (release, wait) = std::sync::mpsc::channel();
    let worker = PlannerWorker::new(GatedRouteFollower {
        inner: crate::native_test_helpers::author_static_map_follower(config),
        computed, release: wait, map: None, boundary: Vec::new(), odometry: None,
        goal_updates: 0, path_updates: 0,
    }).unwrap();
    let mut harness = RouteWorkerHarness { worker, computed: receive, release, clear, observation_tick: 0 };
    let goal = route_pose(0.5, 0);
    let options = GoalOptions {
        position_tolerance_m: 0.1, yaw_tolerance_rad: 0.0, ..Default::default()
    };
    options.validate().unwrap();
    assert!(options.requires_orientation());
    harness.worker.configure_goal(&goal, options).unwrap();
    harness.worker.update_execution_map(ExecutionMap::Surface(harness.clear.clone())).unwrap();
    harness.worker.update_odometry(goal.clone());
    harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D {
        poses: vec![goal.clone(), goal.clone()],
    }), None, goal.clone()).unwrap();
    assert!(matches!(harness.worker.step(0.3), PlannerStep::Pending(_)));
    let actual_computation = harness.next_computation();
    assert_eq!(actual_computation.outcome, PlannerStep::Succeeded);
    assert_eq!(actual_computation.odometry.pose, goal.pose);
    let mut latest = goal.clone();
    latest.stamp.nanos += 1;
    latest.pose.position.x = 0.51;
    validate_map_pose(&latest, "odometry").unwrap();
    assert!(!same_pose(&latest, &goal));
    let distance = latest.pose.position.distance(goal.pose.position);
    assert!(distance > 0.0 && distance < options.position_tolerance_m);
    assert_eq!(latest.pose.orientation, goal.pose.orientation);
    let yaw_error = nuwax_nav_astar::controllers::angle_diff(
        goal.pose.orientation.yaw(), latest.pose.orientation.yaw()).abs();
    assert_eq!(yaw_error, 0.0);
    let p = latest.pose.position;
    let q = latest.pose.orientation;
    harness.clear.validate_body_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]).unwrap();
    harness.worker.update_odometry(latest);
    // No intermediate step: another requested computation could replace the
    // old successful result and mask the exact requalification branch.
    harness.release.send(()).unwrap();
    wait_for_source_result(&harness.worker);
    let observed = {
        let input = harness.worker.shared.0.lock().unwrap();
        (matches!(input.result.as_ref(), Some((_, PlannerStep::Succeeded))),
         input.result_pose_version, input.pose_version,
         input.result_map_version, input.map_version,
         input.result_boundary_version, input.boundary_version,
         input.result_applied_version, input.applied_version)
    };
    // Never assert while holding the stop/Drop mutex.
    assert!(observed.0);
    assert_ne!(observed.1, observed.2);
    assert_eq!(observed.3, observed.4);
    assert_eq!(observed.5, observed.6);
    assert_eq!(observed.7, observed.8);
    eprintln!("yaw-zero worker: distance={distance:.17e} yaw_error={yaw_error:.17e} versions={observed:?}");
    let delivered = harness.worker.step(0.3);
    eprintln!("yaw-zero worker delivered={delivered:?}");
    assert_eq!(delivered, PlannerStep::Succeeded,
        "a real geometric update within the original position region keeps exact original yaw");
    // Existing Drop sends Stop and releases any queued computation even on RED.
}
