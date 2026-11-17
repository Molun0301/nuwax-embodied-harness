// 进程内假守卫只实现冻结协议，导航端使用实际客户端、调度器与状态通道。
mod guard_protocol {
    use super::*;
    use nuwax_motion_safety_guard::{wire::*, MotionGuardClient};
    use prost::Message;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[derive(Clone, Debug)]
    enum Event {
        Acquire(u64),
        AcquireAttempt(u64),
        Velocity(u64, f64),
        Release(u64),
    }
    struct GuardState {
        generation: u64,
        lease: u64,
        next_lease: u64,
        velocity: f64,
        events: Vec<Event>,
        acquire_error: Option<(ErrorCode, String)>,
        disconnect_release: bool,
        connections: usize,
        feedback: Option<MeasuredMotionFeedback>,
    }
    struct FakeGuard {
        socket: std::path::PathBuf,
        state: Arc<Mutex<GuardState>>,
        task: tokio::task::JoinHandle<()>,
    }
    impl FakeGuard {
        async fn spawn(generation: u64) -> (Self, MotionGuardClient) {
            static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let socket = std::env::temp_dir().join(format!(
                "b3-guard-{}-{}.sock", std::process::id(),
                NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            ));
            let state = Arc::new(Mutex::new(GuardState {
                generation, lease: 0, next_lease: 0, velocity: 0., events: Vec::new(),
                acquire_error: None, disconnect_release: false, connections: 0, feedback: None,
            }));
            let task = Self::serve(&socket, state.clone());
            let client = MotionGuardClient::connect(&socket).await.unwrap();
            (Self { socket, state, task }, client)
        }
        fn serve(socket: &std::path::Path, serving: Arc<Mutex<GuardState>>) -> tokio::task::JoinHandle<()> {
            let listener = tokio::net::UnixListener::bind(socket).unwrap();
            tokio::spawn(async move {
                loop {
                    let (mut stream, _) = listener.accept().await.unwrap();
                    serving.lock().unwrap().connections += 1;
                    loop {
                        let Ok(size) = stream.read_u32().await else { break; };
                        let mut body = vec![0; size as usize];
                        if stream.read_exact(&mut body).await.is_err() { break; }
                        let request = GuardRequest::decode(body.as_slice()).unwrap();
                        let response = {
                            let mut state = serving.lock().unwrap();
                            if matches!(request.op.as_ref(), Some(guard_request::Op::Release(_)))
                                && std::mem::take(&mut state.disconnect_release) { break; }
                            let result = match request.op.unwrap() {
                                guard_request::Op::Acquire(request) => {
                                    assert_eq!(request.source, Source::Nav as i32);
                                    state.events.push(Event::AcquireAttempt(request.generation));
                                    if let Some((code, _)) = &state.acquire_error { Err(*code) }
                                    else if request.generation != state.generation { Err(ErrorCode::StaleGeneration) }
                                    else {
                                        state.next_lease += 1; state.lease = state.next_lease;
                                        let lease = state.lease; state.events.push(Event::Acquire(lease));
                                        Ok(guard_response::Result::Acquire(AcquireResponse { lease_id: lease }))
                                    }
                                }
                                guard_request::Op::Velocity(request) => {
                                    if request.lease_id != state.lease || state.lease == 0 { Err(ErrorCode::LeaseRevoked) }
                                    else {
                                        state.velocity = request.vx_mps;
                                        state.events.push(Event::Velocity(request.lease_id, request.vx_mps));
                                        Ok(guard_response::Result::Velocity(VelocityResponse { limited: false }))
                                    }
                                }
                                guard_request::Op::Release(request) => {
                                    if request.lease_id != state.lease || state.lease == 0 { Err(ErrorCode::LeaseRevoked) }
                                    else {
                                        state.events.push(Event::Release(request.lease_id));
                                        state.lease = 0; state.velocity = 0.;
                                        Ok(guard_response::Result::Release(ReleaseResponse {}))
                                    }
                                }
                                guard_request::Op::Status(_) => Ok(guard_response::Result::Status(StatusResponse {
                                    generation: state.generation, lease_id: state.lease, ready: true, ..Default::default()
                                })),
                                other => panic!("导航调用了协议范围外的操作：{other:?}"),
                            };
                            match result {
                                Ok(result) => GuardResponse { result: Some(result), measured_motion: state.feedback.clone(), ..Default::default() },
                                Err(code) => GuardResponse {
                                    error: Some(Error { code: code as i32, message: state.acquire_error.as_ref()
                                        .filter(|(expected, _)| *expected == code).map_or_else(|| "测试守卫拒绝".into(), |(_, reason)| reason.clone()) }),
                                    measured_motion: state.feedback.clone(), ..Default::default()
                                },
                            }
                        };
                        let body = response.encode_to_vec();
                        if stream.write_u32(body.len() as u32).await.is_err() { break; }
                        if stream.write_all(&body).await.is_err() { break; }
                    }
                }
            })
        }
        async fn restart(&mut self, generation: u64) {
            self.task.abort();
            let _ = (&mut self.task).await;
            std::fs::remove_file(&self.socket).unwrap();
            {
                let mut state = self.state.lock().unwrap();
                state.generation = generation; state.lease = 0; state.next_lease = 0;
                state.velocity = 0.; state.feedback = None;
            }
            self.task = Self::serve(&self.socket, self.state.clone());
        }
        fn stop(&self) {
            let mut state = self.state.lock().unwrap();
            state.generation += 1; state.lease = 0; state.velocity = 0.;
        }
        async fn moving(&self) {
            tokio::time::timeout(Duration::from_secs(2), async {
                while self.state.lock().unwrap().velocity == 0. { tokio::task::yield_now().await; }
            }).await.expect("导航未发送非零速度");
        }
    }
    impl Drop for FakeGuard {
        fn drop(&mut self) {
            self.task.abort();
            std::fs::remove_file(&self.socket).unwrap();
        }
    }
    async fn harness(generation: u64) -> (FakeGuard, GuardedLoopHarness) {
        let (guard, client) = FakeGuard::spawn(generation).await;
        let harness = GuardedLoopHarness::spawn_with_motion(
            false,
            "base_link",
            client,
            Arc::new(Mutex::new(Vec::new())),
        )
        .await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(1);
        harness.wait_for_costmap(1).await;
        (guard, harness)
    }
    fn goal(id: &str, generation: u64) -> schema::NavigationGoal {
        schema::NavigationGoal {
            session_id: id.into(),
            motion_generation: generation,
            ..sealed_goal(id)
        }
    }
    async fn state(
        harness: &mut GuardedLoopHarness,
        id: &str,
        wanted: i32,
    ) -> schema::NavigationStatus {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_status_body(&frame.payload).unwrap();
                if state.session_id == id && state.state == wanted {
                    return state;
                }
            }
        })
        .await
        .expect("原状态通道没有回报目标终态")
    }
    async fn execute(harness: &GuardedLoopHarness, goal: &schema::NavigationGoal) {
        harness.send_goal_and_wait_dequeued(goal).await;
        harness.wait_for_session(&goal.session_id).await;
        let mut path = sealed_path(goal);
        path.planner_runtime_generation = saturating_i64(runtime_generation());
        harness.send_path(&path);
    }
    #[tokio::test]
    async fn b3_detected_guard_disconnect_ends_goal_and_next_goal_reconnects() {
        let (mut guard, mut h) = harness(12).await;
        execute(&h, &goal("重启前目标", 12)).await;
        guard.moving().await;
        guard.restart(12).await;
        let failed = state(&mut h, "重启前目标", 4).await;
        assert_eq!(failed.goal_status, schema::NAVIGATION_GOAL_STATUS_FAILED);
        assert!(failed.reason.contains("传输失败"), "{}", failed.reason);
        assert!(h.scheduler.status().session_id.is_none());
        assert!(!h.loop_task.is_finished() && !h.scheduler_task.is_finished());
        let boundary = guard.state.lock().unwrap().events.len();
        execute(&h, &goal("重启后新目标", 12)).await;
        guard.moving().await;
        {
            let state = guard.state.lock().unwrap();
            assert_eq!(state.connections, 2);
            assert!(matches!(state.events.get(boundary), Some(Event::AcquireAttempt(12))));
            assert_eq!(state.events[boundary..].iter().filter(|event| matches!(event, Event::AcquireAttempt(_))).count(), 1);
        }
        h.shutdown().await;
    }

    #[tokio::test]
    async fn b3_idle_disconnect_fails_that_acquire_and_following_goal_recovers() {
        let (mut guard, mut h) = harness(13).await;
        guard.restart(13).await;
        h.send_goal_and_wait_dequeued(&goal("检测空闲断线的目标", 13)).await;
        let failed = state(&mut h, "检测空闲断线的目标", 4).await;
        assert_eq!(failed.goal_status, schema::NAVIGATION_GOAL_STATUS_FAILED);
        assert!(failed.reason.contains("传输失败"), "{}", failed.reason);
        assert!(h.scheduler.status().session_id.is_none());
        assert!(guard.state.lock().unwrap().events.is_empty(), "失败的Acquire不能自动重发");
        execute(&h, &goal("断线之后新目标", 13)).await;
        guard.moving().await;
        h.shutdown().await;
    }

    #[tokio::test]
    async fn b3_acquire_not_ready_and_invalid_argument_keep_guard_reason_as_failed() {
        for (code, reason) in [(ErrorCode::NotReady, "执行器未连接"), (ErrorCode::InvalidArgument, "执行参数非法")] {
            let (guard, mut h) = harness(14).await;
            guard.state.lock().unwrap().acquire_error = Some((code, reason.into()));
            h.send_goal_and_wait_dequeued(&goal("守卫拒绝目标", 14)).await;
            let failed = state(&mut h, "守卫拒绝目标", 4).await;
            assert_eq!(failed.goal_status, schema::NAVIGATION_GOAL_STATUS_FAILED);
            assert!(failed.reason.contains(reason), "{}", failed.reason);
            assert_eq!(h.scheduler.status().last_terminal, Some(nuwax_nav_runtime::TerminalOutcome::SafetyFailed));
            guard.state.lock().unwrap().acquire_error = None;
            execute(&h, &goal("恢复后的新目标", 14)).await;
            guard.moving().await;
            h.shutdown().await;
        }
    }

    #[tokio::test]
    async fn b3_release_disconnect_fails_current_goal_without_claiming_zero_or_exiting() {
        let (guard, mut h) = harness(15).await;
        let running = goal("释放断线目标", 15);
        execute(&h, &running).await;
        guard.moving().await;
        tokio::time::timeout(Duration::from_secs(2), async {
            while !h.scheduler.status().last_applied_twist.is_some_and(|twist| twist.linear.x != 0.) {
                tokio::task::yield_now().await;
            }
        }).await.unwrap();
        guard.state.lock().unwrap().disconnect_release = true;
        h.send_cancel(&running);
        let failed = state(&mut h, &running.session_id, 4).await;
        assert_eq!(failed.goal_status, schema::NAVIGATION_GOAL_STATUS_FAILED);
        assert!(failed.reason.contains("释放租约失败"), "{}", failed.reason);
        assert!(h.scheduler.status().session_id.is_none());
        assert!(h.scheduler.status().last_applied_twist.is_some_and(|twist| twist.linear.x != 0.), "释放失败不能虚构零速采用");
        execute(&h, &goal("释放断线后的目标", 15)).await;
        guard.moving().await;
        h.shutdown().await;
    }

    #[tokio::test]
    async fn b3_reconnect_does_not_replay_requests_or_keep_prior_guard_measurement() {
        fn feedback(stamp: u64) -> MeasuredMotionFeedback {
            let now = nuwax_motion_safety_guard::measured_motion::monotonic_ns();
            MeasuredMotionFeedback {
                source_id: "同一设备".into(), source_timestamp_ns: stamp,
                received_monotonic_ns: now, valid_until_monotonic_ns: now + 2_000_000_000,
                frame_id: "base_link".into(), body_linear_xyz: vec![0.; 3], body_angular_xyz: vec![0.; 3],
                orientation_wxyz: Vec::new(),
            }
        }
        let (mut guard, mut client) = FakeGuard::spawn(16).await;
        guard.state.lock().unwrap().feedback = Some(feedback(100));
        client.acquire(Source::Nav, 16).await.unwrap();
        assert_eq!(client.measured_motion().unwrap().source_timestamp_ns, 100);
        guard.restart(16).await;
        guard.state.lock().unwrap().feedback = Some(feedback(50));
        let before = guard.state.lock().unwrap().events.len();
        assert!(client.velocity(0.2, 0., 0.).await.is_err());
        assert!(client.release().await.is_err());
        assert_eq!(guard.state.lock().unwrap().events.len(), before, "旧速度与释放不能在新连接重发");
        client.acquire(Source::Nav, 16).await.unwrap();
        assert_eq!(client.measured_motion().unwrap().source_timestamp_ns, 50, "新守卫反馈不能受旧样本水位阻挡");
        let state = guard.state.lock().unwrap();
        assert_eq!(state.events[before..].iter().filter(|event| matches!(event, Event::AcquireAttempt(16))).count(), 1);
        assert!(matches!(state.events.get(before), Some(Event::AcquireAttempt(16))));
    }

    #[tokio::test]
    async fn b3_stale_generation_reports_stopped_and_next_goal_still_runs() {
        let (guard, mut h) = harness(u64::MAX).await;
        h.send_goal_and_wait_dequeued(&goal("过期目标", u64::MAX - 1))
            .await;
        let stopped = state(&mut h, "过期目标", 4).await;
        assert_eq!(stopped.goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);
        assert!(!stopped.reason.is_empty());
        assert_eq!(
            h.scheduler.status().last_terminal,
            Some(nuwax_nav_runtime::TerminalOutcome::Stopped)
        );
        assert!(guard.state.lock().unwrap().events.iter().all(|event| matches!(event, Event::AcquireAttempt(_))));
        execute(&h, &goal("有效目标", u64::MAX)).await;
        guard.moving().await;
        h.shutdown().await;
    }
    #[tokio::test]
    async fn b3_stop_revokes_running_lease_and_reports_stopped() {
        let (guard, mut h) = harness(17).await;
        execute(&h, &goal("执行中目标", 17)).await;
        guard.moving().await;
        guard.stop();
        let stopped = state(&mut h, "执行中目标", 4).await;
        assert_eq!(stopped.goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);
        assert_eq!(stopped.reason, "已被停止");
        assert_eq!(
            h.scheduler.status().last_terminal,
            Some(nuwax_nav_runtime::TerminalOutcome::Stopped)
        );
        let before = guard.state.lock().unwrap().events.len();
        h.send_pose(2);
        h.wait_for_pose(2).await;
        assert_eq!(guard.state.lock().unwrap().events.len(), before);
        assert_eq!(guard.state.lock().unwrap().velocity, 0.);
        h.shutdown().await;
    }
    #[tokio::test]
    async fn b3_replacement_releases_old_lease_before_new_motion() {
        let (guard, h) = harness(17).await;
        execute(&h, &goal("旧目标", 17)).await;
        guard.moving().await;
        let old = guard.state.lock().unwrap().lease;
        guard.state.lock().unwrap().generation = 17;
        execute(&h, &goal("新目标", 17)).await;
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let ready = {
                    let state = guard.state.lock().unwrap();
                    state.events.iter().any(
                        |event| matches!(event,Event::Velocity(lease,x) if *lease!=old && *x!=0.),
                    )
                };
                if ready {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap();
        let state = guard.state.lock().unwrap();
        let new = state.lease;
        assert_ne!(old, new);
        let old_motion = state
            .events
            .iter()
            .position(|event| matches!(event,Event::Velocity(lease,x) if *lease==old && *x!=0.))
            .unwrap();
        let new_motion = state
            .events
            .iter()
            .position(|event| matches!(event,Event::Velocity(lease,x) if *lease==new && *x!=0.))
            .unwrap();
        assert!(state.events[old_motion + 1..=new_motion]
            .iter()
            .any(|event| matches!(event, Event::Release(lease) if *lease==old)));
        assert_eq!(
            state
                .events
                .iter()
                .filter(|event| matches!(event, Event::Acquire(_)))
                .count(),
            2
        );
        drop(state);
        h.shutdown().await;
    }
    #[tokio::test]
    async fn b3_cancel_all_releases_current_goal_and_cancels_the_waiting_goal() {
        let (guard, mut h) = harness(17).await;
        execute(&h, &goal("待取消目标", 17)).await;
        guard.moving().await;
        let lease = guard.state.lock().unwrap().lease;
        send_guarded_test_body(
            &h.cancels,
            "nuwax_msgs.CancelNavigation",
            schema::encode_cancel_navigation_body(&schema::CancelNavigation::default()).unwrap(),
        );
        let cancelled = state(&mut h, "待取消目标", 0).await;
        assert_eq!(cancelled.goal_status, schema::NAVIGATION_GOAL_STATUS_CANCELLED);
        let state = guard.state.lock().unwrap();
        assert_eq!(state.lease, 0);
        assert_eq!(state.velocity, 0.);
        assert_eq!(
            state
                .events
                .iter()
                .filter(|event| matches!(event,Event::Release(id) if *id==lease))
                .count(),
            1
        );
        drop(state);
        h.shutdown().await;
        let (guard, mut h) = harness(19).await;
        // 撤销规划输入，使目标只进入待执行槽；启动取消后再给输入也不能执行它。
        h.scheduler.invalidate_map("测试输入暂缺").await.unwrap();
        h.send_goal_and_wait_dequeued(&goal("等待目标", 19)).await;
        send_guarded_test_body(
            &h.cancels,
            "nuwax_msgs.CancelNavigation",
            schema::encode_cancel_navigation_body(&schema::CancelNavigation::default()).unwrap(),
        );
        let cancelled = self::state(&mut h, "等待目标", 0).await;
        assert_eq!(cancelled.goal_status, schema::NAVIGATION_GOAL_STATUS_CANCELLED);
        assert!(guard.state.lock().unwrap().events.is_empty());
        h.send_pose(2);
        h.wait_for_pose(2).await;
        h.send_costmap(2);
        h.wait_for_costmap(2).await;
        assert!(h.scheduler.status().session_id.is_none());
        h.shutdown().await;
    }
}
