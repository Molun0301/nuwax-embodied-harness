// DRAFT ONLY: include inside planner_worker.rs tests. The existing publication
// gate returns an actual RouteFollower/SurfaceFollower result, not fake success.
#[test]
fn zero_position_real_arrival_requalification_preserves_exact_xyz_only() {
    for nearby in [false, true] {
        let (config, clear) = crate::native_test_helpers::fixture();
        assert!(!config.controller.simulation);
        let (computed, receive) = std::sync::mpsc::channel();
        let (release, wait) = std::sync::mpsc::channel();
        let worker = PlannerWorker::new(GatedRouteFollower {
            inner: crate::RouteFollower::new(config).unwrap(),
            computed, release: wait, map: None, boundary: Vec::new(), odometry: None,
            goal_updates: 0, path_updates: 0,
        }).unwrap();
        // Reuse the panic-safe Drop which stops the owner and releases its gate.
        let mut harness = RouteWorkerHarness { worker, computed: receive, release, clear };
        let goal = route_pose(0.5, 0);
        let options = GoalOptions {
            position_tolerance_m: 0.0, yaw_tolerance_rad: 0.2, ..Default::default()
        };
        harness.worker.configure_goal(&goal, options).unwrap();
        harness.worker.update_execution_map(ExecutionMap::Surface(harness.clear.clone())).unwrap();
        harness.worker.update_odometry(goal.clone());
        harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![goal.clone(), goal.clone()],
        }), None, goal.clone()).unwrap();
        assert!(matches!(harness.worker.step(0.3), PlannerStep::Pending(_)));
        let actual_computation = harness.next_computation();
        assert_eq!(actual_computation.outcome, PlannerStep::Succeeded);
        assert_eq!(actual_computation.odometry.pose.position, goal.pose.position);
        let mut latest = goal.clone();
        latest.stamp.nanos += 1;
        // A real geometric change, not merely a timestamp. Positive yaw remains
        // strictly within the original user's unchanged yaw tolerance.
        latest.pose.orientation = Quaternion::from_yaw(0.01);
        if nearby { latest.pose.position.x += f64::EPSILON; }
        let yaw_error = nuwax_nav_astar::controllers::angle_diff(
            goal.pose.orientation.yaw(), latest.pose.orientation.yaw()).abs();
        assert!(yaw_error > 0.0 && yaw_error < options.yaw_tolerance_rad);
        validate_map_pose(&latest, "odometry").unwrap();
        assert!(!same_pose(&latest, &goal));
        if nearby { assert_ne!(latest.pose.position, goal.pose.position); }
        else { assert_eq!(latest.pose.position, goal.pose.position); }
        harness.worker.update_odometry(latest);
        // Do not call step before this publication. It would request another
        // computation and allow a fresh result to hide the requalification gate.
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
        // Assertions run after unlock: a premise failure must not poison the
        // mutex used by panic-safe Drop/stop cleanup.
        assert!(observed.0);
        assert_ne!(observed.1, observed.2,
            "the next real step must evaluate arrival_event_still_in_goal");
        assert_eq!(observed.3, observed.4);
        assert_eq!(observed.5, observed.6);
        assert_eq!(observed.7, observed.8);
        assert_eq!(harness.worker.step(0.3), if nearby {
            PlannerStep::Pending(Vec::new())
        } else { PlannerStep::Succeeded });
        // Any queued negative-case computation remains gated. Drop handles stop
        // and release even if an assertion above panics; no extra fake result.
    }
}
