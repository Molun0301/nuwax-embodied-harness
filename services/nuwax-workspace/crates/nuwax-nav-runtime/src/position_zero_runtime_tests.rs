// DRAFT ONLY: include inside lib.rs tests. Real SurfaceFollower and scheduler;
// only the existing in-memory motion-port proof fixture stands in for hardware.
// No planner success is injected, no simulation/deadband/TTL override is used.
fn zero_position_surface_parts() -> (SurfaceFollowerConfig, Arc<SurfaceSnapshot>, PoseStamped) {
    let (config, map) = crate::native_test_helpers::fixture();
    assert!(!config.controller.simulation);
    assert!(config.motion_limits.is_some());
    let mut goal = map_pose_at(1);
    goal.pose.position = nuwax_nav_types_messages::Point3::new(0.5, 1.0, 0.5);
    map.validate_body_pose([0.5, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0]).unwrap();
    (config, map, goal)
}
fn zero_position_options() -> nuwax_nav_types_messages::GoalOptions {
    nuwax_nav_types_messages::GoalOptions {
        position_tolerance_m: 0.0, yaw_tolerance_rad: 0.2, allow_reverse: false,
    }
}
async fn zero_position_surface_scheduler() -> (
    NavScheduler<FakeMotion, SurfaceFollower>, watch::Sender<SchedulerStatus>,
    Arc<Mutex<FakeMotionState>>, PoseStamped,
) {
    let (config, map, goal) = zero_position_surface_parts();
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let follower = crate::native_test_helpers::author_static_map_follower(config);
    let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), follower,
        SchedulerConfig::default()).unwrap();
    let (status, _) = watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status, goal.clone(), None).unwrap();
    scheduler.observe_costmap(&status, CostmapObservation {
        costmap: ExecutionMap::Surface(map), source_timestamp_ns: goal.stamp.nanos,
    }, Instant::now()).unwrap();
    let mut request = session();
    request.goal = ExecutionGoal::Body(goal.clone());
    request.options = zero_position_options();
    scheduler.start_session(&status, request).await.unwrap();
    let mut route = path();
    route.path = ExecutionPath::Surface(Path3D { poses: vec![goal.clone(), goal.clone()] });
    scheduler.submit_path(&status, route).await.unwrap();
    assert!(scheduler.stale_input_reason(Instant::now()).is_none());
    assert!(scheduler.obstacle_source_available);
    (scheduler, status, motion, goal)
}
#[tokio::test]
async fn zero_position_real_surface_arrival_finishes_only_after_fence_and_fresh_pose() {
    let (mut scheduler, status, motion, goal) = zero_position_surface_scheduler().await;
    scheduler.control_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none());
    assert!(!scheduler.lease_active);
    let pending = scheduler.arrival_confirmation.as_ref().expect("real follower arrival event");
    let confirmed_at = pending.confirmed_at;
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|c| c.starts_with("fence:")).count(), 1);
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none(), "pre-stop pose cannot confirm success");
    let mut after = goal.clone();
    after.stamp.nanos += 1;
    scheduler.observe_odometry(&status, after.clone(), None).unwrap();
    assert!(scheduler.pose_acquired_at.unwrap() > confirmed_at);
    assert_eq!(after.pose.position, goal.pose.position);
    scheduler.session_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
    assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
    assert!(scheduler.status.last_zero_ack_id.is_some());
    assert!(scheduler.session.is_none());
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|c| c.starts_with("acquire:")).count(), 1);
}
#[tokio::test]
async fn zero_position_real_surface_rejects_nearby_pose_before_arrival_event() {
    let (mut scheduler, status, _motion, goal) = zero_position_surface_scheduler().await;
    let mut nearby = goal.clone();
    nearby.pose.position.x += f64::EPSILON;
    nearby.stamp.nanos += 1;
    assert_ne!(nearby.pose.position.x, goal.pose.position.x);
    assert!(nearby.pose.position.distance(goal.pose.position) < 1e-6);
    scheduler.observe_odometry(&status, nearby, None).unwrap();
    scheduler.control_tick(&status).await.unwrap();
    assert!(scheduler.arrival_confirmation.is_none(), "control deadband cannot issue a zero-goal arrival event");
    assert!(scheduler.status.last_terminal.is_none());
}
#[tokio::test]
async fn zero_position_stopped_real_surface_arrival_rejects_nonzero_drift() {
    let (mut scheduler, status, _motion, goal) = zero_position_surface_scheduler().await;
    scheduler.control_tick(&status).await.unwrap();
    let confirmed_at = scheduler.arrival_confirmation.as_ref().unwrap().confirmed_at;
    let mut nearby = goal.clone();
    nearby.pose.position.x += f64::EPSILON;
    nearby.stamp.nanos += 1;
    assert_ne!(nearby.pose.position.x, goal.pose.position.x);
    scheduler.observe_odometry(&status, nearby, None).unwrap();
    assert!(scheduler.pose_acquired_at.unwrap() > confirmed_at);
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none());
    assert!(scheduler.arrival_resume_requested());
    assert!(!scheduler.lease_active, "do not reacquire or move merely to assert rejection");
}
#[test]
fn zero_position_terminal_group_distinguishes_original_goal_from_wrong_height() {
    for wrong_height in [false, true] {
        let (config, map, start) = zero_position_surface_parts();
        let mut endpoint = start.clone();
        endpoint.pose.position.x = 0.75;
        let mut goal = endpoint.clone();
        if wrong_height { goal.pose.position.z += f64::EPSILON; }
        let mut actual = endpoint.clone();
        if !wrong_height { actual.pose.position.x -= 5e-7; }
        actual.stamp.nanos += 1;
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        follower.configure_goal(&goal, zero_position_options()).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(start.clone());
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![start, endpoint],
        }), None, goal).unwrap();
        follower.update_odometry(actual);
        let step = follower.step(0.3);
        assert_ne!(step, PlannerStep::Succeeded);
        if wrong_height {
            assert!(matches!(step, PlannerStep::ReplanRequired(_)));
            assert_eq!(follower.progress_m(), 0.25,
                "same-XY wrong-height truncation is consumable, then original-goal replan");
        } else {
            assert!(follower.progress_m() < 0.25,
                "actual nonzero offset may not consume the true terminal goal group");
        }
    }
}
