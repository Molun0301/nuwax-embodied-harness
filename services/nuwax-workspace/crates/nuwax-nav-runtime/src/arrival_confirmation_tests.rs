async fn publish_pose_after_confirmed_stop(handle: &NavSchedulerHandle, mut pose: PoseStamped) {
    // 测试设备继续提供停车后的真实状态；单个规划成功返回不代替该观察。
    let mut status = handle.subscribe_status();
    let source = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let snapshot = status.borrow().clone();
            assert_ne!(snapshot.last_terminal, Some(TerminalOutcome::Succeeded),
                "停车后的设备位姿尚未发送，不能提前成功");
            if snapshot.phase == SchedulerPhase::Fencing && snapshot.message.starts_with("已确认停稳") {
                assert!(snapshot.last_zero_ack_id.is_none(), "复核完成前不公开终态停车票据");
                break snapshot.pose_source_timestamp_ns;
            }
            status.changed().await.unwrap();
        }
    }).await.expect("等待测试设备实际停车");
    pose.stamp.nanos = source.checked_add(1).unwrap();
    handle.update_odometry(pose).await.expect("停车后的设备位姿");
}

#[tokio::test]
async fn arrival_is_rechecked_after_stopping_and_drift_resumes_the_same_goal() {
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let planner = FakePlanner::commanding();
    let planning = planner.state.clone();
    let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), planner,
        SchedulerConfig::default()).unwrap();
    let (status, _) = watch::channel(scheduler.status.clone());
    let mut inside = map_pose_at(1);
    inside.pose.position.x = 1.86;
    scheduler.observe_odometry(&status, inside, None).unwrap();
    scheduler.observe_costmap(&status, costmap_at(1_000_000_000), Instant::now()).unwrap();
    let mut goal = session();
    goal.options.position_tolerance_m = 0.15;
    scheduler.start_session(&status, goal.clone()).await.unwrap();
    scheduler.submit_path(&status, path()).await.unwrap();
    planning.lock().unwrap().step = PlannerStep::Succeeded;
    scheduler.control_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none(),
        "首次规划入界和停车证明不能替代停车后的目标位置");
    assert_eq!(scheduler.session.as_ref(), Some(&goal));
    assert!(!scheduler.lease_active);

    let mut outside = map_pose_at(2);
    outside.pose.position.x = 1.83;
    scheduler.observe_odometry(&status, outside, None).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    planning.lock().unwrap().step = PlannerStep::Command(Twist2 {
        linear: Point2::new(0.1, 0.0), angular_z: 0.0,
    });
    scheduler.session_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.phase, SchedulerPhase::Controlling);
    assert_eq!(scheduler.session.as_ref(), Some(&goal));
    assert_eq!(scheduler.current_path_revision, 1);
    assert!(scheduler.status.last_terminal.is_none());
    assert!(scheduler.status.last_zero_ack_id.is_none(), "旧停车证明不能描述已重新取得的租约");
    scheduler.control_tick(&status).await.unwrap();
    assert!(scheduler.status.last_applied_twist.unwrap().linear.x > 0.0);

    let mut second = map_pose_at(3);
    second.pose.position.x = 1.92;
    scheduler.observe_odometry(&status, second, None).unwrap();
    planning.lock().unwrap().step = PlannerStep::Succeeded;
    scheduler.control_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none());
    let mut stopped = map_pose_at(4);
    stopped.pose.position.x = 1.93;
    scheduler.observe_odometry(&status, stopped, None).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
    assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
    assert!(scheduler.session.is_none());
    let calls = &motion.lock().unwrap().calls;
    assert_eq!(calls.iter().filter(|call| call.starts_with("acquire:")).count(), 2);
    assert_eq!(calls.iter().filter(|call| call.starts_with("fence:")).count(), 2);
}

async fn stopped_arrival_fixture() -> (NavScheduler<FakeMotion, FakePlanner>,
    watch::Sender<SchedulerStatus>, Arc<Mutex<FakeMotionState>>) {
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let planner = FakePlanner::commanding();
    planner.state.lock().unwrap().step = PlannerStep::Succeeded;
    let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), planner,
        SchedulerConfig::default()).unwrap();
    let (status, _) = watch::channel(scheduler.status.clone());
    let mut pose = map_pose_at(1);
    pose.pose.position.x = 1.9;
    scheduler.observe_odometry(&status, pose, None).unwrap();
    scheduler.observe_costmap(&status, costmap_at(1_000_000_000), Instant::now()).unwrap();
    let mut goal = session();
    goal.options.position_tolerance_m = 0.15;
    scheduler.start_session(&status, goal).await.unwrap();
    scheduler.submit_path(&status, path()).await.unwrap();
    scheduler.control_tick(&status).await.unwrap();
    (scheduler, status, motion)
}

#[tokio::test]
async fn arrival_new_sequence_cannot_relabel_a_pre_stop_sample_as_confirmation() {
    let (mut scheduler, status, motion) = stopped_arrival_fixture().await;
    let confirmation = scheduler.arrival_confirmation.as_ref().unwrap().confirmed_at;
    let mut delayed = map_pose_at(2);
    delayed.pose.position.x = 2.0;
    scheduler.observe_odometry_at(&status, delayed, Some(Instant::now() + Duration::from_secs(1)),
        confirmation - Duration::from_millis(1)).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none(), "较新源序号仍可能采集于停车前");
    assert!(!scheduler.lease_active);
    let mut current = map_pose_at(3);
    current.pose.position.x = 2.0;
    scheduler.observe_odometry_at(&status, current, Some(Instant::now() + Duration::from_secs(1)),
        Instant::now()).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 1);
}

#[tokio::test]
async fn arrival_recheck_uses_original_height_and_required_orientation() {
    for kind in ["height", "required_yaw", "optional_yaw"] {
        let (mut scheduler, status, _) = stopped_arrival_fixture().await;
        let mut current = map_pose_at(2);
        current.pose.position.x = 2.0;
        if kind == "height" { current.pose.position.z = 0.16; }
        else {
            current.pose.orientation = nuwax_nav_types_messages::Quaternion {
                x: 0.0, y: 0.0, z: 0.25_f64.sin(), w: 0.25_f64.cos(),
            };
            scheduler.session.as_mut().unwrap().options.yaw_tolerance_rad =
                if kind == "required_yaw" { 0.1 } else { std::f64::consts::PI };
        }
        scheduler.observe_odometry(&status, current, None).unwrap();
        scheduler.session_tick(&status).await.unwrap();
        if kind == "optional_yaw" {
            assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
        } else {
            assert!(scheduler.status.last_terminal.is_none(), "{kind}不能只凭平面位置发布成功");
            assert_eq!(scheduler.status.phase, SchedulerPhase::Acquiring);
            assert!(scheduler.arrival_resume_requested());
        }
    }
}

#[tokio::test]
async fn arrival_confirmation_cancellation_invalidation_and_shutdown_use_the_existing_fence() {
    for action in ["cancel", "invalidate", "shutdown"] {
        let (mut scheduler, status, motion) = stopped_arrival_fixture().await;
        if action == "cancel" {
            scheduler.cancel(&status, "goal-1").await.unwrap();
        } else {
            let (reply, response) = oneshot::channel();
            let command = if action == "invalidate" {
                UrgentCommand::InvalidateMap { reason: "来源已撤销".into(), invalidated_at: Instant::now(),
                    kind: MapInvalidationKind::Invalidated, reply }
            } else { UrgentCommand::Shutdown { reply } };
            let result = scheduler.handle_urgent(&status, command).await;
            if let Some(result) = result { result.unwrap(); }
            response.await.unwrap().unwrap();
        }
        let expected = match action { "cancel" => TerminalOutcome::Cancelled,
            "invalidate" => TerminalOutcome::SafetyFailed, _ => TerminalOutcome::Shutdown };
        assert_eq!(scheduler.status.last_terminal, Some(expected));
        assert_eq!(scheduler.status.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert!(scheduler.session.is_none() && !scheduler.lease_active);
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("fence:")).count(), 1);
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 1);
    }
}

#[tokio::test]
async fn arrival_wait_preserves_the_original_wait_deadline() {
    let (mut scheduler, status, motion) = stopped_arrival_fixture().await;
    scheduler.config.path_wait_timeout = Duration::from_secs(1);
    scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(2));
    scheduler.session_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
    assert!(scheduler.status.last_zero_ack_id.is_some());
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("fence:")).count(), 1);
}

#[tokio::test]
async fn arrival_source_pause_keeps_confirmed_stop_and_resumes_only_after_fresh_input() {
    let (mut scheduler, status, motion) = stopped_arrival_fixture().await;
    scheduler.obstacle_source_available = false;
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.status.last_terminal.is_none());
    assert_eq!(scheduler.arrival_confirmation.as_ref().unwrap().proof.zero_ack_id, "zero-ack");
    assert!(scheduler.session.is_some() && !scheduler.lease_active);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 1);
    scheduler.obstacle_source_available = true;
    let mut outside = map_pose_at(2); outside.pose.position.x = 1.7;
    scheduler.observe_odometry(&status, outside.clone(), None).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.arrival_resume_requested());
    scheduler.pose_valid_until = Some(Instant::now() - Duration::from_millis(1));
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.arrival_confirmation.is_some() && !scheduler.lease_active);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 1);
    outside.stamp.nanos += 1;
    scheduler.observe_odometry(&status, outside, Some(Instant::now() + Duration::from_millis(500))).unwrap();
    scheduler.session_tick(&status).await.unwrap();
    assert!(scheduler.lease_active && scheduler.arrival_confirmation.is_none());
    assert_eq!(scheduler.current_path_revision, 1);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 2);
    scheduler.cancel(&status, "goal-1").await.unwrap();
}

struct ArrivalAdmissionMotion {
    inner: FakeMotion,
    acquisitions: usize,
    failure: Option<&'static str>,
    entered: Option<Arc<tokio::sync::Notify>>,
    release: Option<Arc<tokio::sync::Notify>>,
}

#[async_trait]
impl NavMotionPort for ArrivalAdmissionMotion {
    async fn acquire(&mut self, operation: &str, admission: &NavMotionAdmission)
        -> Result<LeaseProof, MotionPortError> {
        self.acquisitions += 1;
        let mut proof = self.inner.acquire(operation, admission).await?;
        proof.lease_id = self.acquisitions as u64;
        if self.acquisitions > 1 {
            if let Some(entered) = &self.entered { entered.notify_one(); }
            if let Some(release) = &self.release { release.notified().await; }
            match self.failure {
                Some("denied") => return Err(MotionPortError::definite_no_lease("原准入已被全局停止撤销")),
                Some("uncertain") => return Err(MotionPortError::new("新租约响应丢失")),
                _ => {}
            }
        }
        Ok(proof)
    }
    async fn renew(&mut self, operation: &str) -> Result<(), MotionPortError> {
        self.inner.renew(operation).await
    }
    async fn submit_twist(&mut self, operation: &str, twist: Twist2) -> Result<AppliedProof, MotionPortError> {
        self.inner.submit_twist(operation, twist).await
    }
    async fn revoke_and_zero(&mut self, operation: &str) -> Result<FenceProof, MotionPortError> {
        let mut proof = self.inner.revoke_and_zero(operation).await?;
        let count = self.inner.0.lock().unwrap().calls.iter().filter(|v| v.starts_with("fence:")).count();
        proof.zero_ack_id = format!("arrival-zero-{count}");
        proof.applied_monotonic_ns = 100 + count as u64;
        Ok(proof)
    }
}

#[tokio::test]
async fn arrival_readmission_failure_never_reuses_old_fence_for_a_possible_new_lease() {
    for failure in ["denied", "uncertain"] {
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        planner.state.lock().unwrap().step = PlannerStep::Succeeded;
        let port = ArrivalAdmissionMotion { inner: FakeMotion(motion.clone()), acquisitions: 0,
            failure: Some(failure), entered: None, release: None };
        let mut scheduler = NavScheduler::new(port, planner, SchedulerConfig::default()).unwrap();
        let (status, _) = watch::channel(scheduler.status.clone());
        let mut inside = map_pose_at(1); inside.pose.position.x = 1.9;
        scheduler.observe_odometry(&status, inside, None).unwrap();
        scheduler.observe_costmap(&status, costmap_at(1_000_000_000), Instant::now()).unwrap();
        let mut goal = session(); goal.options.position_tolerance_m = 0.15;
        scheduler.start_session(&status, goal).await.unwrap();
        scheduler.submit_path(&status, path()).await.unwrap();
        scheduler.control_tick(&status).await.unwrap();
        let mut outside = map_pose_at(2); outside.pose.position.x = 1.8;
        scheduler.observe_odometry(&status, outside, None).unwrap();
        scheduler.session_tick(&status).await.unwrap();
        assert!(scheduler.session_tick(&status).await.is_err());
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert!(!scheduler.lease_active && scheduler.session.is_none());
        let expected = if failure == "denied" { "arrival-zero-1" } else { "arrival-zero-2" };
        assert_eq!(scheduler.status.last_zero_ack_id.as_deref(), Some(expected));
        assert!(!motion.lock().unwrap().calls.iter().any(|v| v.starts_with("twist:")));
    }
}

#[tokio::test]
async fn arrival_cancel_during_readmission_fences_the_new_grant_before_any_command() {
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let entered = Arc::new(tokio::sync::Notify::new());
    let release = Arc::new(tokio::sync::Notify::new());
    let planner = FakePlanner::commanding();
    let planning = planner.state.clone();
    planning.lock().unwrap().step = PlannerStep::Succeeded;
    let port = ArrivalAdmissionMotion { inner: FakeMotion(motion.clone()), acquisitions: 0,
        failure: None, entered: Some(entered.clone()), release: Some(release.clone()) };
    let (handle, task) = spawn_scheduler(NavScheduler::new(port, planner, SchedulerConfig {
        tick_interval: Duration::from_millis(2), ..Default::default()
    }).unwrap());
    let mut inside = map_pose_at(1); inside.pose.position.x = 1.9;
    handle.update_odometry(inside).await.unwrap();
    handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
    let mut goal = session(); goal.options.position_tolerance_m = 0.15;
    handle.start_session(goal).await.unwrap();
    handle.submit_path(path()).await.unwrap();
    let mut state = handle.subscribe_status();
    tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            if state.borrow().message.starts_with("已确认停稳") { break; }
            state.changed().await.unwrap();
        }
    }).await.unwrap();
    let mut outside = map_pose_at(2); outside.pose.position.x = 1.8;
    handle.update_odometry(outside).await.unwrap();
    tokio::time::timeout(Duration::from_secs(1), entered.notified()).await.unwrap();
    planning.lock().unwrap().step = PlannerStep::Command(Twist2 {
        linear: Point2::new(0.2, 0.0), angular_z: 0.0,
    });
    let cancellation = handle.clone();
    let cancel = tokio::spawn(async move { cancellation.cancel("goal-1").await });
    tokio::time::sleep(Duration::from_millis(10)).await;
    assert!(!cancel.is_finished(), "未知取租约结果不能被旧停车证明覆盖");
    assert!(!motion.lock().unwrap().calls.iter().any(|v| v.starts_with("twist:")));
    release.notify_one();
    tokio::time::timeout(Duration::from_secs(1), cancel).await.unwrap().unwrap().unwrap();
    assert_eq!(handle.status().last_terminal, Some(TerminalOutcome::Cancelled));
    assert_eq!(handle.status().last_zero_ack_id.as_deref(), Some("arrival-zero-2"));
    assert!(!motion.lock().unwrap().calls.iter().any(|v| v.starts_with("twist:")));
    handle.shutdown().await.unwrap();
    task.await.unwrap().unwrap();
}

#[tokio::test]
async fn arrival_surface_worker_replays_current_map_and_goal_after_stopping_drift() {
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let mut config = SurfaceFollowerConfig::new(0.4);
    config.controller.speed = 0.2;
    let planner = planner_worker::PlannerWorker::new(crate::native_test_helpers::author_static_map_follower(config)).unwrap();
    let (handle, task) = spawn_scheduler(NavScheduler::new(FakeMotion(motion.clone()), planner,
        SchedulerConfig { tick_interval: Duration::from_millis(5), ..Default::default() }).unwrap());
    let mut start = nuwax_nav_astar::helpers::make_pose_stamped("map", 4.1, 1.55, 0.0);
    start.pose.position.z = 0.5;
    start.stamp = Timestamp::from_unix(1, 0);
    let mut goal_pose = start.clone(); goal_pose.pose.position.x = 4.55;
    let mut inside = start.clone(); inside.pose.position.x = 4.45;
    handle.update_odometry(inside).await.unwrap();
    handle.update_costmap(CostmapObservation {
        costmap: surface_follower::prepared_test_surface(surface_follower::test_snapshot(0.0, false)),
        source_timestamp_ns: 1_000_000_000,
    }).await.unwrap();
    let mut goal = session(); goal.goal = ExecutionGoal::Body(goal_pose.clone()); goal.options.position_tolerance_m = 0.15;
    handle.start_session(goal).await.unwrap();
    let mut route = path();
    route.path = ExecutionPath::Surface(Path3D { poses: vec![start, goal_pose] });
    handle.submit_path(route).await.unwrap();
    let mut stopped = handle.subscribe_status();
    tokio::time::timeout(Duration::from_secs(2), async {
        while !stopped.borrow().message.starts_with("已确认停稳") {
            stopped.changed().await.unwrap();
        }
    }).await.unwrap();
    let mut outside = nuwax_nav_astar::helpers::make_pose_stamped("map", 4.35, 1.55, 0.0);
    outside.pose.position.z = 0.5; outside.stamp = Timestamp::from_unix(2, 0);
    handle.update_odometry(outside.clone()).await.unwrap();
    let observation_clock = Instant::now();
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let status = stopped.borrow().clone();
            assert!(status.last_terminal.is_none());
            if status.last_applied_twist.is_some_and(|command| command.linear.x > 0.0) { break; }
            tokio::time::sleep(Duration::from_millis(50)).await;
            // 停车漂移后设备仍持续采样；同一旧位姿不能推进作者A状态机。
            outside.stamp.nanos = 2_000_000_000 + observation_clock.elapsed().as_nanos() as i64;
            handle.update_odometry(outside.clone()).await.unwrap();
        }
    }).await.expect("实际表面工作线程在重新取租约后应继续原目标");
    assert_eq!(handle.status().path_revision, 1);
    let mut final_pose = nuwax_nav_astar::helpers::make_pose_stamped("map", 4.49, 1.55, 0.0);
    final_pose.pose.position.z = 0.5; final_pose.stamp = Timestamp::from_unix(3, 0);
    handle.update_odometry(final_pose.clone()).await.unwrap();
    publish_pose_after_confirmed_stop(&handle, final_pose).await;
    wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("acquire:")).count(), 2);
    assert_eq!(motion.lock().unwrap().calls.iter().filter(|v| v.starts_with("fence:")).count(), 2);
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
}
