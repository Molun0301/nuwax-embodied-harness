// 零容差到达使用当拍实际位姿；FakeMotion 仅记录租约请求，不证明实测停稳。
fn yaw_zero_pi_goal() -> nuwax_nav_types_messages::Quaternion {
    nuwax_nav_types_messages::Quaternion { x: 0.0, y: 0.0, z: 1.0, w: 0.0 }
}
fn yaw_zero_antipode(q: nuwax_nav_types_messages::Quaternion)
    -> nuwax_nav_types_messages::Quaternion
{
    nuwax_nav_types_messages::Quaternion { x: -q.x, y: -q.y, z: -q.z, w: -q.w }
}
fn yaw_zero_record(label: &str, goal: nuwax_nav_types_messages::Quaternion,
    actual: nuwax_nav_types_messages::Quaternion)
{
    let g = [goal.x, goal.y, goal.z, goal.w];
    let a = [actual.x, actual.y, actual.z, actual.w];
    let delta = actual.yaw() - goal.yaw();
    let iterative = nuwax_nav_astar::controllers::angle_diff(actual.yaw(), goal.yaw());
    let trigonometric = delta.sin().atan2(delta.cos());
    eprintln!("yaw-zero {label}: goal_xyzw={g:?} goal_bits={:?} actual_xyzw={a:?} actual_bits={:?} goal_yaw={:.17e} actual_yaw={:.17e} delta={delta:.17e} iterative={iterative:.17e} trigonometric={trigonometric:.17e}",
        g.map(f64::to_bits), a.map(f64::to_bits), goal.yaw(), actual.yaw());
}
async fn yaw_zero_surface_scheduler(
    goal_orientation: nuwax_nav_types_messages::Quaternion,
    actual_orientation: nuwax_nav_types_messages::Quaternion,
) -> (NavScheduler<FakeMotion, SurfaceFollower>, watch::Sender<SchedulerStatus>,
    Arc<Mutex<FakeMotionState>>, PoseStamped)
{
    // Reuse the committed position-zero body/map/config fixture unchanged.
    let (config, map, mut goal) = zero_position_surface_parts();
    goal.pose.orientation = goal_orientation;
    let mut actual = goal.clone();
    actual.pose.orientation = actual_orientation;
    validate_map_pose(&actual, "odometry").unwrap();
    for pose in [&goal, &actual] {
        let p = pose.pose.position;
        let q = pose.pose.orientation;
        map.validate_body_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]).unwrap();
    }
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let follower = crate::native_test_helpers::author_static_map_follower(config);
    let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), follower,
        SchedulerConfig::default()).unwrap();
    let (status, _) = watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status, actual.clone(), None).unwrap();
    scheduler.observe_costmap(&status, CostmapObservation {
        costmap: ExecutionMap::Surface(map), source_timestamp_ns: actual.stamp.nanos,
    }, Instant::now()).unwrap();
    let mut request = session();
    request.goal = ExecutionGoal::Body(goal.clone());
    request.options = nuwax_nav_types_messages::GoalOptions {
        position_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, allow_reverse: false,
    };
    request.options.validate().unwrap();
    assert!(request.options.requires_orientation());
    scheduler.start_session(&status, request).await.unwrap();
    let mut route = path();
    route.path = ExecutionPath::Surface(Path3D { poses: vec![goal.clone(), goal] });
    scheduler.submit_path(&status, route).await.unwrap();
    scheduler.sample_input_status();
    assert!(scheduler.status.planner_inputs_fresh);
    assert!(scheduler.obstacle_source_available);
    (scheduler, status, motion, actual)
}
async fn yaw_zero_observe_release(
    scheduler: &mut NavScheduler<FakeMotion, SurfaceFollower>,
    status: &watch::Sender<SchedulerStatus>, motion: &Arc<Mutex<FakeMotionState>>,
    _actual: PoseStamped,
) {
    scheduler.session_tick(status).await.unwrap();
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
    assert!(!scheduler.lease_active);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|c| c.starts_with("release:")).count(), 1);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|c| c.starts_with("acquire:")).count(), 1);
}
#[tokio::test]
async fn zero_yaw_antipode_real_surface_arrival_release_control() {
    let goal = yaw_zero_pi_goal();
    let actual = yaw_zero_antipode(goal);
    yaw_zero_record("strict antipode control", goal, actual);
    assert_ne!([goal.x, goal.y, goal.z, goal.w].map(f64::to_bits),
        [actual.x, actual.y, actual.z, actual.w].map(f64::to_bits));
    assert_eq!(goal.yaw(), std::f64::consts::PI);
    assert_eq!(actual.yaw(), goal.yaw());
    let (mut scheduler, status, motion, pose) = yaw_zero_surface_scheduler(goal, actual).await;
    yaw_zero_observe_release(&mut scheduler, &status, &motion, pose).await;
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
    assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));

    assert!(scheduler.session.is_none());
}
#[tokio::test]
async fn zero_yaw_real_surface_rejects_nonzero_residual_before_arrival_event() {
    let goal = nuwax_nav_types_messages::Quaternion::IDENTITY;
    let actual = nuwax_nav_types_messages::Quaternion::from_yaw(1e-7);
    yaw_zero_record("nonzero residual cannot arrive", goal, actual);
    let error = nuwax_nav_astar::controllers::angle_diff(actual.yaw(), goal.yaw()).abs();
    assert!(error > 0.0 && error < 1e-6);
    let delta = actual.yaw() - goal.yaw();
    assert!(delta.sin().atan2(delta.cos()).abs() > 0.0);
    let (mut scheduler, status, _motion, _pose) = yaw_zero_surface_scheduler(goal, actual).await;
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none());
}

#[tokio::test]
async fn zero_yaw_signed_pi_equivalent_real_surface_arrival_release_must_complete() {
    let goal = yaw_zero_pi_goal();
    // Exactly unit quaternion. Numerically equal to -goal; signed-zero choices
    // make both numerator terms negative zero in the real yaw extraction.
    let actual = nuwax_nav_types_messages::Quaternion { x: -0.0, y: 0.0, z: -1.0, w: 0.0 };
    yaw_zero_record("signed-zero equivalent antipode boundary", goal, actual);
    assert_eq!(actual, yaw_zero_antipode(goal));
    assert_eq!(goal.yaw(), std::f64::consts::PI);
    assert_eq!(actual.yaw(), -std::f64::consts::PI);
    assert_eq!(nuwax_nav_astar::controllers::angle_diff(actual.yaw(), goal.yaw()), 0.0);
    // This is an implementation fact to be measured by the original Rust binary,
    // not an epsilon and not the proposed zero-yaw arrival oracle.
    let delta = actual.yaw() - goal.yaw();
    assert_ne!(delta.sin().atan2(delta.cos()), 0.0);
    let (mut scheduler, status, motion, pose) = yaw_zero_surface_scheduler(goal, actual).await;
    yaw_zero_observe_release(&mut scheduler, &status, &motion, pose).await;
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded),
        "the +/-PI representations of the same yaw must agree when the lease is released");
}
