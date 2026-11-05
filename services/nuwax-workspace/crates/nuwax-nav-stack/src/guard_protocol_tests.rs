// 进程内假守卫只实现冻结协议，导航端使用实际客户端、调度器与状态通道。
mod guard_protocol {
    use super::*;
    use nuwax_motion_safety_guard::{wire::*, MotionGuardClient};
    use prost::Message;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[derive(Clone, Debug)]
    enum Event {
        Acquire(u64),
        Velocity(u64, f64),
        Release(u64),
    }
    struct GuardState {
        generation: u64,
        lease: u64,
        next_lease: u64,
        velocity: f64,
        events: Vec<Event>,
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
                "b3-guard-{}-{}.sock",
                std::process::id(),
                NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            ));
            let listener = tokio::net::UnixListener::bind(&socket).unwrap();
            let state = Arc::new(Mutex::new(GuardState {
                generation,
                lease: 0,
                next_lease: 0,
                velocity: 0.,
                events: Vec::new(),
            }));
            let serving = state.clone();
            let task = tokio::spawn(async move {
                let (mut stream, _) = listener.accept().await.unwrap();
                loop {
                    let Ok(size) = stream.read_u32().await else {
                        break;
                    };
                    let mut body = vec![0; size as usize];
                    stream.read_exact(&mut body).await.unwrap();
                    let request = GuardRequest::decode(body.as_slice()).unwrap();
                    let response = {
                        let mut state = serving.lock().unwrap();
                        let result = match request.op.unwrap() {
                            guard_request::Op::Acquire(request) => {
                                assert_eq!(request.source, Source::Nav as i32);
                                if request.generation != state.generation {
                                    Err(ErrorCode::StaleGeneration)
                                } else {
                                    state.next_lease += 1;
                                    state.lease = state.next_lease;
                                    let lease = state.lease;
                                    state.events.push(Event::Acquire(lease));
                                    Ok(guard_response::Result::Acquire(AcquireResponse {
                                        lease_id: lease,
                                    }))
                                }
                            }
                            guard_request::Op::Velocity(request) => {
                                if request.lease_id != state.lease || state.lease == 0 {
                                    Err(ErrorCode::LeaseRevoked)
                                } else {
                                    state.velocity = request.vx_mps;
                                    state
                                        .events
                                        .push(Event::Velocity(request.lease_id, request.vx_mps));
                                    Ok(guard_response::Result::Velocity(VelocityResponse {
                                        limited: false,
                                    }))
                                }
                            }
                            guard_request::Op::Release(request) => {
                                if request.lease_id != state.lease || state.lease == 0 {
                                    Err(ErrorCode::LeaseRevoked)
                                } else {
                                    state.events.push(Event::Release(request.lease_id));
                                    state.lease = 0;
                                    state.velocity = 0.;
                                    Ok(guard_response::Result::Release(ReleaseResponse {}))
                                }
                            }
                            guard_request::Op::Status(_) => {
                                Ok(guard_response::Result::Status(StatusResponse {
                                    generation: state.generation,
                                    lease_id: state.lease,
                                    ready: true,
                                    ..Default::default()
                                }))
                            }
                            other => panic!("导航调用了协议范围外的操作：{other:?}"),
                        };
                        match result {
                            Ok(result) => GuardResponse {
                                result: Some(result),
                                ..Default::default()
                            },
                            Err(code) => GuardResponse {
                                error: Some(Error {
                                    code: code as i32,
                                    message: "测试守卫拒绝".into(),
                                }),
                                ..Default::default()
                            },
                        }
                    };
                    let body = response.encode_to_vec();
                    stream.write_u32(body.len() as u32).await.unwrap();
                    stream.write_all(&body).await.unwrap();
                }
            });
            let client = MotionGuardClient::connect(&socket).await.unwrap();
            (
                Self {
                    socket,
                    state,
                    task,
                },
                client,
            )
        }
        fn stop(&self) {
            let mut state = self.state.lock().unwrap();
            state.generation += 1;
            state.lease = 0;
            state.velocity = 0.;
        }
        async fn moving(&self) {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    if self.state.lock().unwrap().velocity != 0. {
                        break;
                    }
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("导航未发送非零速度");
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
    fn goal(id: &str, generation: u64) -> schema::GoalEnvelope {
        schema::GoalEnvelope {
            session_id: id.into(),
            motion_generation: generation,
            ..sealed_goal(id)
        }
    }
    async fn state(
        harness: &mut GuardedLoopHarness,
        id: &str,
        wanted: i32,
    ) -> schema::NavigationState {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_state_body(&frame.payload).unwrap();
                if state.session_id == id && state.state == wanted {
                    return state;
                }
            }
        })
        .await
        .expect("原状态通道没有回报目标终态")
    }
    async fn execute(harness: &GuardedLoopHarness, goal: &schema::GoalEnvelope) {
        harness.send_goal_and_wait_dequeued(goal).await;
        harness.wait_for_session(&goal.session_id).await;
        let mut path = sealed_path(goal);
        path.planner_runtime_generation = saturating_i64(runtime_generation());
        harness.send_path(&path);
    }
    #[tokio::test]
    async fn b3_stale_generation_reports_stopped_and_next_goal_still_runs() {
        let (guard, mut h) = harness(u64::MAX).await;
        h.send_goal_and_wait_dequeued(&goal("过期目标", u64::MAX - 1))
            .await;
        let stopped = state(&mut h, "过期目标", 4).await;
        assert!(stopped.message.starts_with("LEASE_NOT_GRANTED:"));
        assert_eq!(
            h.scheduler.status().last_terminal,
            Some(nuwax_nav_runtime::TerminalOutcome::Stopped)
        );
        assert!(guard.state.lock().unwrap().events.is_empty());
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
        assert!(stopped.message.starts_with("STOPPED:"));
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
    async fn b3_replacement_uses_new_lease_without_an_intermediate_stop() {
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
            .all(|event| !matches!(event, Event::Release(_) | Event::Velocity(_, 0.))));
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
    async fn b3_cancel_all_releases_current_goal_and_clears_buffered_goal() {
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
        assert!(cancelled.message.starts_with("CANCELLED:"));
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
        h.send_goal_and_wait_dequeued(&goal("缓冲目标", 19)).await;
        send_guarded_test_body(
            &h.cancels,
            "nuwax_msgs.CancelNavigation",
            schema::encode_cancel_navigation_body(&schema::CancelNavigation::default()).unwrap(),
        );
        let cancelled = self::state(&mut h, "缓冲目标", 0).await;
        assert!(cancelled.message.starts_with("CANCELLED:"));
        assert!(guard.state.lock().unwrap().events.is_empty());
        h.send_pose(2);
        h.wait_for_pose(2).await;
        h.send_costmap(2);
        h.wait_for_costmap(2).await;
        assert!(h.scheduler.status().session_id.is_none());
        h.shutdown().await;
    }
}
