    include!("guard_protocol_tests.rs");
    fn planar_test_costmap(
        observation: &nuwax_nav_runtime::CostmapObservation,
    ) -> &nuwax_nav_astar::navigation_map::CostMap2D {
        match &observation.costmap {
            nuwax_nav_runtime::ExecutionMap::Planar(map) => map,
            _ => panic!("expected planar map"),
        }
    }

    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum GuardedNavSource {
        Pose,
        Costmap,
        Status,
        Goal,
        Path,
        Cancel,
    }

    impl GuardedNavSource {
        const ORDER: [Self; 6] = [
            Self::Pose,
            Self::Costmap,
            Self::Status,
            Self::Goal,
            Self::Path,
            Self::Cancel,
        ];

        fn index(self) -> usize {
            Self::ORDER
                .iter()
                .position(|source| *source == self)
                .expect("guarded navigation source must belong to the source ring")
        }

    }

    fn event_source(event: &GuardedNavEvent) -> GuardedNavSource {
        match event {
            GuardedNavEvent::Pose(_) => GuardedNavSource::Pose,
            GuardedNavEvent::Costmap(_) => GuardedNavSource::Costmap,
            GuardedNavEvent::Status(_) => GuardedNavSource::Status,
            GuardedNavEvent::Goal(_) => GuardedNavSource::Goal,
            GuardedNavEvent::Path(_) => GuardedNavSource::Path,
            GuardedNavEvent::Cancel(_) => GuardedNavSource::Cancel,
        }
    }
    use futures::channel::mpsc::{unbounded, UnboundedSender};
    use nuwax_nav_runtime::{NavMotionPort, NavSchedulerHandle, RuntimeError, SchedulerPhase, TerminalOutcome};
    use tokio::task::JoinHandle;

    #[tokio::test]
    async fn status_twist_and_zero_stay_live_under_continuous_inputs() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        harness.send_path(&sealed_path(&goal));
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = harness.scheduler.subscribe_status();
            while status.borrow().path_revision != 1 {
                status.changed().await.expect("active path status");
            }
        })
        .await
        .expect("active path");

        while harness.commands.next().now_or_never().flatten().is_some() {}
        while harness.states.next().now_or_never().flatten().is_some() {}
        while harness.reached.next().now_or_never().flatten().is_some() {}
        let applied_baseline = harness.scheduler.status().applied_commands;
        let expected_twist = Twist2 {
            linear: Point2::new(0.731, 0.0),
            angular_z: -0.417,
        };
        *harness.planner_twist.lock().unwrap() = expected_twist;
        let mut status = harness.scheduler.subscribe_status();
        let feeder =
            GuardedContentionFeeder::spawn(harness.poses.clone(), harness.costmaps.clone(), goal.goal.pose.position);

        tokio::time::timeout(Duration::from_secs(1), async {
            while status.borrow().applied_commands <= applied_baseline {
                status
                    .changed()
                    .await
                    .expect("post-baseline command status");
            }
        })
        .await
        .expect("post-baseline applied command");
        let applied = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .commands
                    .next()
                    .await
                    .expect("command stream open")
                    .expect("command frame");
                let twist =
                    schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                        .expect("real status Twist");
                if twist.linear[0] == expected_twist.linear.x
                    && twist.linear[1] == expected_twist.linear.y
                    && twist.angular[2] == expected_twist.angular_z
                {
                    return twist;
                }
            }
        })
        .await
        .expect("bounded status Twist publication");
        feeder.assert_ready_evidence("Twist");
        assert_eq!(applied.linear[0], expected_twist.linear.x);
        assert_eq!(applied.linear[1], expected_twist.linear.y);
        assert_eq!(applied.angular[2], expected_twist.angular_z);
        assert!(harness.scheduler.status().applied_commands > applied_baseline);

        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while status.borrow().last_terminal != Some(TerminalOutcome::Succeeded) {
                status.changed().await.expect("Succeeded status");
            }
        })
        .await
        .expect("Succeeded terminal status");

        let zero = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .commands
                    .next()
                    .await
                    .expect("command stream open")
                    .expect("command frame");
                let twist =
                    schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                        .expect("real status ZERO");
                if twist.linear == [0.0; 3] && twist.angular == [0.0; 3] {
                    return twist;
                }
            }
        })
        .await
        .expect("bounded status ZERO publication");
        feeder.assert_ready_evidence("ZERO");
        assert_eq!(zero.linear, [0.0; 3]);
        assert_eq!(zero.angular, [0.0; 3]);

        let reached = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .reached
                    .next()
                    .await
                    .expect("reached stream open")
                    .expect("reached frame");
                let reached = schema::decode_bool_body(&frame.payload).expect("Succeeded bool");
                if reached.data {
                    return reached;
                }
            }
        })
        .await
        .expect("bounded Succeeded bool publication");
        feeder.assert_ready_evidence("Succeeded Bool");
        assert!(reached.data);

        let _terminal = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .states
                    .next()
                    .await
                    .expect("state stream open")
                    .expect("state frame");
                let state = schema::decode_navigation_status_body(&frame.payload)
                    .expect("real status NavigationStatus");
                if state.session_id == goal.session_id
                    && state.state == 3
                {
                    return state;
                }
            }
        })
        .await
        .expect("bounded terminal NavigationStatus publication");
        feeder.assert_ready_evidence("terminal NavigationStatus");
        assert_eq!(
            harness.scheduler.status().last_terminal,
            Some(TerminalOutcome::Succeeded)
        );
        assert!(
            !harness.loop_task.is_finished(),
            "single owner loop remains live"
        );
        feeder.stop().await.expect("contention feeder cleanup");
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn input_wait_propagates_required_stream_close() {
        for source in GuardedNavSource::ORDER {
            let (goals_tx, mut goals) = guarded_test_stream();
            let (poses_tx, mut poses) = guarded_test_stream();
            let (paths_tx, mut paths) = guarded_test_stream();
            let (costmaps_tx, mut costmaps) = guarded_test_stream();
            let (cancels_tx, mut cancels) = guarded_test_stream();
            let (status_tx, mut status) =
                tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
            let payload = vec![source.index() as u8 + 1];
            match source {
                GuardedNavSource::Pose => {
                    send_guarded_test_body(&poses_tx, "geometry_msgs.PoseStamped", payload.clone())
                }
                GuardedNavSource::Costmap => {
                    send_guarded_test_body(&costmaps_tx, "nav_msgs.OccupancyGrid", payload.clone())
                }
                GuardedNavSource::Status => status_tx
                    .send(nuwax_nav_runtime::SchedulerStatus {
                        message: "status-payload".into(),
                        ..nuwax_nav_runtime::SchedulerStatus::default()
                    })
                    .expect("status receiver"),
                GuardedNavSource::Goal => {
                    send_guarded_test_body(&goals_tx, "nuwax_msgs.NavigationGoal", payload.clone())
                }
                GuardedNavSource::Path => {
                    send_guarded_test_body(&paths_tx, "nuwax_msgs.PathEnvelope", payload.clone())
                }
                GuardedNavSource::Cancel => send_guarded_test_body(
                    &cancels_tx,
                    "nuwax_msgs.CancelNavigation",
                    payload.clone(),
                ),
            }
            let event = wait_guarded_nav_event(
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .now_or_never().expect("ready source mapping");
            assert_eq!(event_source(&event), source);
            match event {
                GuardedNavEvent::Pose(Some(Ok(frame)))
                | GuardedNavEvent::Goal(Some(Ok(frame)))
                | GuardedNavEvent::Path(Some(Ok(frame)))
                | GuardedNavEvent::Cancel(Some(Ok(frame))) => assert_eq!(frame.payload, payload),
                GuardedNavEvent::Costmap(Some(Ok(GuardedMapFrame::Planar(frame)))) => {
                    assert_eq!(frame.payload, payload)
                }
                GuardedNavEvent::Status(Ok(())) => {
                    assert_eq!(status.borrow().message, "status-payload")
                }
                _ => panic!("unexpected ready mapping for {source:?}"),
            }
        }

        for source in [
            GuardedNavSource::Pose,
            GuardedNavSource::Costmap,
            GuardedNavSource::Goal,
            GuardedNavSource::Path,
            GuardedNavSource::Cancel,
        ] {
            let (goals_tx, mut goals) = guarded_test_stream();
            let (poses_tx, mut poses) = guarded_test_stream();
            let (paths_tx, mut paths) = guarded_test_stream();
            let (costmaps_tx, mut costmaps) = guarded_test_stream();
            let (cancels_tx, mut cancels) = guarded_test_stream();
            let (_status_tx, mut status) =
                tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
            match source {
                GuardedNavSource::Pose => poses_tx.close_channel(),
                GuardedNavSource::Costmap => costmaps_tx.close_channel(),
                GuardedNavSource::Goal => goals_tx.close_channel(),
                GuardedNavSource::Path => paths_tx.close_channel(),
                GuardedNavSource::Cancel => cancels_tx.close_channel(),
                GuardedNavSource::Status => unreachable!("watch is covered separately"),
            }
            let event = wait_guarded_nav_event(
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .now_or_never().expect("closed stream is Ready");
            assert_eq!(event_source(&event), source);
            assert!(matches!(
                event,
                GuardedNavEvent::Pose(None)
                    | GuardedNavEvent::Costmap(None)
                    | GuardedNavEvent::Goal(None)
                    | GuardedNavEvent::Path(None)
                    | GuardedNavEvent::Cancel(None)
            ));
        }

        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (_paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        assert!(
            tokio::time::timeout(
                Duration::from_millis(10),
                wait_guarded_nav_event(
                    &mut goals,
                    &mut poses,
                    &mut paths,
                    &mut costmaps,
                    &mut cancels,
                    &mut status,
                ),
            )
            .await
            .is_err(),
            "all sources must remain pending"
        );
        status_tx
            .send(nuwax_nav_runtime::SchedulerStatus {
                message: "rebuilt-watch".into(),
                ..nuwax_nav_runtime::SchedulerStatus::default()
            })
            .expect("rebuilt watch receiver");
        let rebuilt = wait_guarded_nav_event(
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        )
        .await;
        assert!(matches!(rebuilt, GuardedNavEvent::Status(Ok(()))));
        assert_eq!(status.borrow().message, "rebuilt-watch");
    }

    #[tokio::test]
    async fn input_wait_propagates_status_close() {
        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (_paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        drop(status_tx);
        let event = wait_guarded_nav_event(
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        )
        .now_or_never().expect("closed watch is Ready");
        assert!(matches!(event, GuardedNavEvent::Status(Err(_))));

        for source in [
            GuardedCloseSource::Pose,
            GuardedCloseSource::Costmap,
            GuardedCloseSource::Status,
            GuardedCloseSource::Goal,
            GuardedCloseSource::Path,
            GuardedCloseSource::Cancel,
        ] {
            assert_guarded_close_exits(GuardedLoopHarness::spawn().await, source).await;
        }
    }

    #[test]
    fn input_errors_preserve_reject_and_fail_closed_paths() {
        assert!(matches!(
            runtime_input_error::<()>(
                "guarded arbitration",
                RuntimeError::StaleInput("old pose".into()),
            )
            .expect("stale external input remains rejectable"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(runtime_input_error::<()>(
            "guarded arbitration", RuntimeError::Motion("传输失败".into()),
        ).unwrap(), OwnerInput::Rejected(reason) if reason.contains("传输失败")));
        assert!(matches!(
            decode_guarded_pose_input(&[0xff]).expect("bad pose remains external input"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(
            decode_guarded_costmap_input(&[0xff]).expect("bad costmap remains external input"),
            OwnerInput::Rejected(_)
        ));
    }

    fn active_goal(session_id: &str) -> schema::NavigationGoal {
        schema::NavigationGoal {
session_id: session_id.into(),
..schema::NavigationGoal::default()
}
    }

    fn sealed_goal(session_id: &str) -> schema::NavigationGoal {
        let mut goal = schema::NavigationGoal {
session_id: session_id.into(),
motion_generation: 1,
goal: schema::PoseStamped {
                header: schema::Header {
                    stamp_sec: 1,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
};
        schema::validate_navigation_goal(&goal).expect("seal goal");
        goal
    }

    fn sealed_path(goal: &schema::NavigationGoal) -> schema::PathEnvelope {
        let mut path = schema::PathEnvelope {
route_kind: 0,
schema_version: schema::PATH_ENVELOPE_SCHEMA_VERSION,
session_id: goal.session_id.clone(),
motion_generation: goal.motion_generation as u64,
planner_runtime_generation: saturating_i64(runtime_generation()),
path_revision: 1,
path: schema::Path {
                header: schema::Header {
                    stamp_sec: 2,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                poses: vec![schema::PoseStamped {
                    header: schema::Header {
                        stamp_sec: 2,
                        frame_id: "map".into(),
                        ..schema::Header::default()
                    },
                    pose: goal.goal.pose.clone(),
                }],
            },
..schema::PathEnvelope::default()
};
        schema::validate_path_envelope(&path).expect("seal path");
        path
    }

    fn rollover_goal(
        previous: &schema::NavigationGoal,
        session_id: &str,
        goal_generation: i64,
    ) -> schema::NavigationGoal {
        let mut goal = previous.clone();
        goal.session_id = session_id.into();
        goal.motion_generation = goal_generation as u64;

        schema::validate_navigation_goal(&goal).expect("seal rollover goal");
        goal
    }

    fn guarded_pose_body(stamp_sec: i32) -> Vec<u8> {
        guarded_pose_body_at(stamp_sec, [0.0; 3])
    }

    fn guarded_pose_body_at(stamp_sec: i32, position: [f64; 3]) -> Vec<u8> {
        schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                seq: stamp_sec,
                stamp_sec,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position,
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode guarded test pose")
    }

    fn guarded_costmap_body(revision: i32) -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: revision,
                stamp_sec: revision,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode guarded costmap")
    }

    #[derive(Clone)]
    pub(super) struct RecordingMotion(pub(super) Arc<Mutex<Vec<String>>>, Twist2);

    impl RecordingMotion {
        pub(super) fn new(calls: Arc<Mutex<Vec<String>>>) -> Self {
            Self(calls, Twist2::ZERO)
        }
    }

    struct FirstAcquireMotion {
        inner: RecordingMotion,
        reject_first: bool,
    }

    #[async_trait::async_trait]
    impl nuwax_nav_runtime::NavMotionPort for FirstAcquireMotion {
        fn measured_motion(&self) -> Option<nuwax_nav_runtime::MeasuredMotion> {
            nuwax_nav_runtime::NavMotionPort::measured_motion(&self.inner)
        }
        async fn acquire(&mut self,generation:u64)->Result<u64,nuwax_nav_runtime::MotionPortError> {
            if self.reject_first {
                self.reject_first=false;
                self.inner.0.lock().unwrap().push("rejected_acquire:".into());
                return Err(nuwax_nav_runtime::MotionPortError {message:"代次过期".into(),code:Some(nuwax_motion_safety_guard::wire::ErrorCode::StaleGeneration)});
            }
            self.inner.acquire(generation).await
        }
        async fn velocity(&mut self,twist:Twist2)->Result<(),nuwax_nav_runtime::MotionPortError> { self.inner.velocity(twist).await }
        async fn release(&mut self)->Result<(),nuwax_nav_runtime::MotionPortError> { self.inner.release().await }

}

    #[async_trait::async_trait]
    impl nuwax_nav_runtime::NavMotionPort for RecordingMotion {
        fn measured_motion(&self) -> Option<nuwax_nav_runtime::MeasuredMotion> {
            // 夹具即时采用已接受命令；读取时给出当前执行速度的测量快照。
            let now = nuwax_nav_runtime::motion_feedback::monotonic_ns();
            Some(nuwax_nav_runtime::MeasuredMotion {
                source_id: "test:recording-motion".into(),
                source_timestamp_ns: current_unix_timestamp_ns().unwrap() as u64,
                received_monotonic_ns: now,
                valid_until_monotonic_ns: u64::MAX,
                frame_id: "base_link".into(),
                body_linear_xyz: [self.1.linear.x, self.1.linear.y, 0.],
                body_angular_xyz: [0., 0., self.1.angular_z],
                orientation_wxyz: Some([1., 0., 0., 0.]),
            })
        }
        async fn acquire(&mut self,generation:u64)->Result<u64,nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push(format!("acquire:{generation}")); Ok(generation)
        }
        async fn velocity(&mut self,twist:Twist2)->Result<(),nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push("twist:".into());
            self.1=twist; Ok(())
        }
        async fn release(&mut self)->Result<(),nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push("release:".into()); Ok(())
        }

}

    struct CommandingPlanner {
        succeeds: Arc<std::sync::atomic::AtomicBool>,
        twist: Arc<Mutex<Twist2>>,
    }

    impl nuwax_nav_runtime::NavPlanner for CommandingPlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {}

        fn update_costmap(&mut self, _costmap: nuwax_nav_astar::navigation_map::CostMap2D) {}

        fn step(&mut self) -> nuwax_nav_runtime::PlannerStep {
            if self
                .succeeds
                .swap(false, std::sync::atomic::Ordering::SeqCst)
            {
                return nuwax_nav_runtime::PlannerStep::Succeeded;
            }
            nuwax_nav_runtime::PlannerStep::Command(*self.twist.lock().unwrap())
        }

        fn stop(&mut self) {}
    }

    #[derive(Default)]
    struct GuardedDequeueLedger {
        pending: std::sync::atomic::AtomicUsize,
        dequeued: std::sync::atomic::AtomicUsize,
    }

    type GuardedTestInput = (
        TransportResult<RawLcmFrame>,
        Option<oneshot::Sender<()>>,
        Option<Arc<GuardedDequeueLedger>>,
    );
    type GuardedTestSender = UnboundedSender<GuardedTestInput>;

    struct GuardedContentionFeeder {
        stop: tokio::sync::watch::Sender<bool>,
        tasks: Vec<JoinHandle<()>>,
        pose_ledger: Arc<GuardedDequeueLedger>,
        costmap_ledger: Arc<GuardedDequeueLedger>,
    }

    impl GuardedContentionFeeder {
        fn spawn(poses: GuardedTestSender, costmaps: GuardedTestSender, position: [f64; 3]) -> Self {
            const WINDOW: usize = 8;
            let (stop, stop_rx) = tokio::sync::watch::channel(false);
            let pose_ledger = Arc::new(GuardedDequeueLedger::default());
            let costmap_ledger = Arc::new(GuardedDequeueLedger::default());

            let pose_task = {
                let ledger = Arc::clone(&pose_ledger);
                let mut stop = stop_rx.clone();
                tokio::spawn(async move {
                    let mut stamp = 100_i32;
                    let mut receipts = futures::stream::FuturesUnordered::new();
                    for _ in 0..WINDOW {
                        receipts.push(send_guarded_test_body_observed_with_ledger(
                            &poses,
                            "geometry_msgs.PoseStamped",
                            guarded_pose_body_at(stamp, position),
                            Some(Arc::clone(&ledger)),
                        ));
                        stamp += 1;
                    }
                    loop {
                        if *stop.borrow() {
                            break;
                        }
                        tokio::select! {
                            biased;
                            changed = stop.changed() => {
                                if changed.is_err() || *stop.borrow() {
                                    break;
                                }
                            }
                            receipt = receipts.next() => {
                                receipt.expect("pose receipt stream").expect("pose dequeued");
                                if *stop.borrow() {
                                    break;
                                }
                                receipts.push(send_guarded_test_body_observed_with_ledger(
                                    &poses,
                                    "geometry_msgs.PoseStamped",
                                    guarded_pose_body_at(stamp, position),
                                    Some(Arc::clone(&ledger)),
                                ));
                                stamp += 1;
                            }
                        }
                    }
                })
            };
            let costmap_task = {
                let ledger = Arc::clone(&costmap_ledger);
                let mut stop = stop_rx;
                tokio::spawn(async move {
                    let mut stamp = 200_i32;
                    let mut receipts = futures::stream::FuturesUnordered::new();
                    for _ in 0..WINDOW {
                        receipts.push(send_guarded_test_body_observed_with_ledger(
                            &costmaps,
                            "nav_msgs.OccupancyGrid",
                            guarded_costmap_body(stamp),
                            Some(Arc::clone(&ledger)),
                        ));
                        stamp += 1;
                    }
                    loop {
                        if *stop.borrow() {
                            break;
                        }
                        tokio::select! {
                            biased;
                            changed = stop.changed() => {
                                if changed.is_err() || *stop.borrow() {
                                    break;
                                }
                            }
                            receipt = receipts.next() => {
                                receipt.expect("costmap receipt stream").expect("costmap dequeued");
                                if *stop.borrow() {
                                    break;
                                }
                                receipts.push(send_guarded_test_body_observed_with_ledger(
                                    &costmaps,
                                    "nav_msgs.OccupancyGrid",
                                    guarded_costmap_body(stamp),
                                    Some(Arc::clone(&ledger)),
                                ));
                                stamp += 1;
                            }
                        }
                    }
                })
            };

            Self {
                stop,
                tasks: vec![pose_task, costmap_task],
                pose_ledger,
                costmap_ledger,
            }
        }

        fn assert_ready_evidence(&self, observation: &str) {
            let pose = self
                .pose_ledger
                .pending
                .load(std::sync::atomic::Ordering::SeqCst);
            let costmap = self
                .costmap_ledger
                .pending
                .load(std::sync::atomic::Ordering::SeqCst);
            assert!(
                pose > 0 || costmap > 0,
                "{observation} lacked unresolved Pose/Costmap dequeue evidence"
            );
        }

        async fn stop(self) -> Result<(), String> {
            let stop_error = self
                .stop
                .send(true)
                .err()
                .map(|_| "contention feeder stop receivers closed".to_string());
            let reap_error = reap_guarded_tasks(self.tasks, Duration::from_secs(1))
                .await
                .err();
            match (stop_error, reap_error) {
                (None, None) => Ok(()),
                (Some(error), None) | (None, Some(error)) => Err(error),
                (Some(stop_error), Some(reap_error)) => Err(format!("{stop_error}; {reap_error}")),
            }
        }
    }

    async fn reap_guarded_tasks(
        tasks: Vec<JoinHandle<()>>,
        join_timeout: Duration,
    ) -> Result<(), String> {
        let mut errors = Vec::new();
        for (index, mut task) in tasks.into_iter().enumerate() {
            match tokio::time::timeout(join_timeout, &mut task).await {
                Ok(Ok(())) => {}
                Ok(Err(error)) => errors.push(format!("task {index} join failed: {error}")),
                Err(_) => {
                    task.abort();
                    match task.await {
                        Ok(()) => errors.push(format!(
                            "task {index} timed out after {join_timeout:?} but abort joined cleanly"
                        )),
                        Err(error) => errors.push(format!(
                            "task {index} timed out after {join_timeout:?}; abort join: {error}"
                        )),
                    }
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("; "))
        }
    }

    struct GuardedTaskExitProbe(Arc<std::sync::atomic::AtomicBool>);

    impl Drop for GuardedTaskExitProbe {
        fn drop(&mut self) {
            self.0.store(true, std::sync::atomic::Ordering::SeqCst);
        }
    }

    #[tokio::test]
    async fn contention_feeder_reaps_every_task_before_reporting_failures() {
        let first_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let later_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let panic_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let completed = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let tasks = vec![
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&first_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    std::future::pending::<()>().await;
                })
            },
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&later_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    std::future::pending::<()>().await;
                })
            },
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&panic_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    panic!("intentional feeder join failure");
                })
            },
            {
                let completed = Arc::clone(&completed);
                tokio::spawn(async move {
                    completed.store(true, std::sync::atomic::Ordering::SeqCst);
                })
            },
        ];

        let error = reap_guarded_tasks(tasks, Duration::from_millis(10))
            .await
            .expect_err("timeout and JoinError must be reported after full cleanup");
        assert!(error.contains("task 0 timed out"), "{error}");
        assert!(error.contains("task 1 timed out"), "{error}");
        assert!(error.contains("task 2 join failed"), "{error}");
        assert!(first_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(later_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(panic_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(completed.load(std::sync::atomic::Ordering::SeqCst));

        let normal_a = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let normal_b = Arc::new(std::sync::atomic::AtomicBool::new(false));
        reap_guarded_tasks(
            vec![
                {
                    let completed = Arc::clone(&normal_a);
                    tokio::spawn(async move {
                        completed.store(true, std::sync::atomic::Ordering::SeqCst);
                    })
                },
                {
                    let completed = Arc::clone(&normal_b);
                    tokio::spawn(async move {
                        completed.store(true, std::sync::atomic::Ordering::SeqCst);
                    })
                },
            ],
            Duration::from_secs(1),
        )
        .await
        .expect("normal feeder cleanup");
        assert!(normal_a.load(std::sync::atomic::Ordering::SeqCst));
        assert!(normal_b.load(std::sync::atomic::Ordering::SeqCst));
    }

    struct GuardedLoopHarness {
        goals: GuardedTestSender,
        poses: GuardedTestSender,
        paths: GuardedTestSender,
        costmaps: GuardedTestSender,
        cancels: GuardedTestSender,
        commands: LcmFrameStream,
        states: LcmFrameStream,
        reached: LcmFrameStream,
        shutdown: Option<oneshot::Sender<()>>,
        loop_task: JoinHandle<NavStackResult<()>>,
        scheduler: NavSchedulerHandle,
        scheduler_task: JoinHandle<Result<(), RuntimeError>>,
        motion_calls: Arc<Mutex<Vec<String>>>,
        planner_succeeds: Arc<std::sync::atomic::AtomicBool>,
        planner_twist: Arc<Mutex<Twist2>>,
    }

    impl GuardedLoopHarness {
        async fn spawn() -> Self {Self::spawn_with_planar_mode(false).await}

        async fn spawn_with_planar_mode(planar_mode:bool) -> Self {
            Self::spawn_with_reference(planar_mode, "base_link").await
        }

        async fn spawn_with_reference(planar_mode:bool, body_reference: &'static str) -> Self {
            Self::spawn_with_first_acquire(planar_mode, body_reference, false).await
        }

        async fn spawn_with_first_acquire(planar_mode:bool, body_reference: &'static str, reject_first: bool) -> Self {
            let calls=Arc::new(Mutex::new(Vec::new()));
            Self::spawn_with_motion(planar_mode,body_reference,
                FirstAcquireMotion {inner:RecordingMotion::new(calls.clone()),reject_first},calls).await
        }

        async fn spawn_with_motion<M:nuwax_nav_runtime::NavMotionPort>(planar_mode:bool, body_reference: &'static str, motion:M, motion_calls:Arc<Mutex<Vec<String>>>) -> Self {
            static NEXT_LOOP_ID: std::sync::atomic::AtomicU64 =
                std::sync::atomic::AtomicU64::new(0);
            let loop_id = NEXT_LOOP_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // One real liblcm context is shared by this in-process fixture.
            // Functional owner/codec checks do not require a host multicast route.
            let transport = LcmTransport::new_real("memq://")
                .expect("guarded loop test LCM memq");
            let planner_succeeds = Arc::new(std::sync::atomic::AtomicBool::new(false));
            let planner_twist = Arc::new(Mutex::new(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.1,
            }));
            let scheduler = nuwax_nav_runtime::NavScheduler::new(
                motion,
                CommandingPlanner {
                    succeeds: Arc::clone(&planner_succeeds),
                    twist: Arc::clone(&planner_twist),
                },
                nuwax_nav_runtime::SchedulerConfig {
                    tick_interval: Duration::from_millis(5),

                    path_wait_timeout: Duration::from_secs(5),

                },
            )
            .expect("guarded loop scheduler");
            let (scheduler, scheduler_task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
            tokio::time::timeout(Duration::from_secs(2), async {
                while !scheduler.status().motion_sink_ready {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("scheduler motion readiness");

            let (goals, goal_stream) = guarded_test_stream();
            let (poses, pose_stream) = guarded_test_stream();
            let (paths, path_stream) = guarded_test_stream();
            let (costmaps, costmap_stream) = guarded_test_stream();
            let (cancels, cancel_stream) = guarded_test_stream();
            let (shutdown, shutdown_rx) = oneshot::channel();
            let status = scheduler.subscribe_status();
            let loop_scheduler = scheduler.clone();
            let mut node = TaskChannels { id: format!("guarded-loop-test-{loop_id}").into(), inputs: Vec::new(), outputs: Vec::new() };
            node.outputs = vec![
                format!("/test/guarded/{loop_id}/cmd"),
                format!("/test/guarded/{loop_id}/state"),
                format!("/test/guarded/{loop_id}/reached"),
                format!("/test/guarded/{loop_id}/cancelled"),
            ];
            let commands = typed_stream(&transport, &node.outputs[0], "geometry_msgs.Twist")
                .await
                .expect("guarded command test stream");
            let states = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.NavigationStatus")
                .await
                .expect("guarded state test stream");
            let reached = typed_stream(&transport, &node.outputs[2], "std_msgs.Bool")
                .await
                .expect("guarded reached test stream");
            let loop_transport = transport.clone();
            let loop_task = tokio::spawn(async move {
                run_guarded_nav_loop(&node, &loop_transport, &loop_scheduler, shutdown_rx, goal_stream, pose_stream, path_stream, costmap_stream, cancel_stream, status, None, planar_mode, body_reference, None)
                .await
            });

            Self {
                goals,
                poses,
                paths,
                costmaps,
                cancels,
                commands,
                states,
                reached,
                shutdown: Some(shutdown),
                loop_task,
                scheduler,
                scheduler_task,
                motion_calls,
                planner_succeeds,
                planner_twist,
            }
        }

        fn send_goal(&self, goal: &schema::NavigationGoal) {
            send_guarded_test_body(
                &self.goals,
                "nuwax_msgs.NavigationGoal",
                schema::encode_navigation_goal_body(goal).expect("encode guarded test goal"),
            );
        }

        async fn send_goal_and_wait_dequeued(&self, goal: &schema::NavigationGoal) {
            let dequeued = send_guarded_test_body_observed(
                &self.goals,
                "nuwax_msgs.NavigationGoal",
                schema::encode_navigation_goal_body(goal).expect("encode guarded test goal"),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded goal was not dequeued")
                .expect("guarded goal dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn send_pose(&self, stamp_sec: i32) {
            send_guarded_test_body(
                &self.poses,
                "geometry_msgs.PoseStamped",
                guarded_pose_body(stamp_sec),
            );
        }

        fn send_costmap(&self, revision: i32) {
            send_guarded_test_body(
                &self.costmaps,
                "nav_msgs.OccupancyGrid",
                guarded_costmap_body(revision),
            );
        }

        fn send_path(&self, path: &schema::PathEnvelope) {
            send_guarded_test_body(
                &self.paths,
                "nuwax_msgs.PathEnvelope",
                schema::encode_path_envelope_body(path).expect("encode guarded test path"),
            );
        }

        fn send_cancel(&self, goal: &schema::NavigationGoal) {
            send_guarded_test_body(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal),
            );
        }

        async fn send_cancel_and_wait_dequeued(
            &self,
            goal: &schema::NavigationGoal,
        ) {
            let dequeued = send_guarded_test_body_observed(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded cancellation was not dequeued")
                .expect("guarded cancellation dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn cancel_body(goal: &schema::NavigationGoal) -> Vec<u8> {
            schema::encode_cancel_navigation_body(&schema::CancelNavigation {
                session_id: goal.session_id.clone(),
                reason: 1,
            })
            .expect("encode guarded test cancellation")
        }

        fn motion_call_count(&self, prefix: &str) -> usize {
            self.motion_calls
                .lock()
                .unwrap()
                .iter()
                .filter(|call| call.starts_with(prefix))
                .count()
        }

        async fn wait_for_session(&self, session_id: &str) {
            tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    let snapshot=self.scheduler.status();
                    if snapshot.session_id.as_deref() == Some(session_id) && snapshot.applied_commands>0 {
                        break;
                    }
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap_or_else(|_| {
                panic!(
                    "session {session_id} was not activated; status={:?}",
                    self.scheduler.status()
                )
            });
        }

        async fn wait_for_pose(&self, stamp_sec: i32) {
            let timestamp_ns = i64::from(stamp_sec) * 1_000_000_000;
            tokio::time::timeout(Duration::from_secs(1), async {
                while self.scheduler.status().pose_source_timestamp_ns != timestamp_ns {
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("pose was not consumed");
        }

        async fn wait_for_costmap(&self, stamp_sec: i32) {
            let timestamp_ns = i64::from(stamp_sec) * 1_000_000_000;
            tokio::time::timeout(Duration::from_secs(1), async {
                while self.scheduler.status().costmap_source_timestamp_ns != timestamp_ns {
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("costmap was not consumed");
        }

        async fn wait_for_release(&self) {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    if self
                        .motion_calls
                        .lock()
                        .unwrap()
                        .iter()
                        .any(|call| call.starts_with("release:"))
                    {
                        break;
                    }
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap_or_else(|_| {
                panic!(
                    "未观察到租约释放请求; status={:?} calls={:?}",
                    self.scheduler.status(),
                    self.motion_calls.lock().unwrap().clone()
                )
            });
        }

        async fn wait_for_cancelled_status(
            &mut self,
            goal: &schema::NavigationGoal,
        ) -> schema::NavigationStatus {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let frame = self
                        .states
                        .next()
                        .await
                        .expect("guarded state stream open")
                        .expect("guarded state transport frame");
                    let state = schema::decode_navigation_status_body(&frame.payload)
                        .expect("guarded NavigationStatus body");
                    if state.session_id == goal.session_id
                        && state.motion_generation == goal.motion_generation as i64
                        && state.goal_status == schema::NAVIGATION_GOAL_STATUS_CANCELLED
                    {
                        return state;
                    }
                }
            })
            .await
            .expect("关联目标的取消状态")
        }

        async fn assert_no_session(&self) {
            tokio::time::sleep(Duration::from_millis(30)).await;
            assert!(
                self.scheduler.status().session_id.is_none(),
                "goal activated before all required inputs"
            );
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        async fn shutdown(mut self) {
            if let Some(shutdown) = self.shutdown.take() {
                shutdown.send(()).expect("guarded loop shutdown receiver");
            }
            tokio::time::timeout(Duration::from_secs(1), self.loop_task)
                .await
                .expect("guarded loop shutdown timeout")
                .expect("guarded loop join")
                .expect("guarded loop result");
            self.scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command");
            self.scheduler_task
                .await
                .expect("scheduler join")
                .expect("scheduler result");
        }
    }

    #[derive(Clone, Copy)]
    enum GuardedCloseSource {
        Pose,
        Costmap,
        Status,
        Goal,
        Path,
        Cancel,
    }

    async fn assert_guarded_close_exits(
        mut harness: GuardedLoopHarness,
        source: GuardedCloseSource,
    ) {
        match source {
            GuardedCloseSource::Pose => harness.poses.close_channel(),
            GuardedCloseSource::Costmap => harness.costmaps.close_channel(),
            GuardedCloseSource::Goal => harness.goals.close_channel(),
            GuardedCloseSource::Path => harness.paths.close_channel(),
            GuardedCloseSource::Cancel => harness.cancels.close_channel(),
            GuardedCloseSource::Status => harness
                .scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command"),
        }
        let error = tokio::time::timeout(Duration::from_secs(1), &mut harness.loop_task)
            .await
            .expect("closed source must terminate without busy-loop")
            .expect("guarded loop join")
            .expect_err("closed required source must fail closed");
        assert!(
            error.to_string().contains("closed"),
            "unexpected close error: {error}"
        );
        if !matches!(source, GuardedCloseSource::Status) {
            harness
                .scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command");
        }
        harness
            .scheduler_task
            .await
            .expect("scheduler join")
            .expect("scheduler result");
    }

    fn guarded_test_stream() -> (GuardedTestSender, LcmFrameStream) {
        let (sender, receiver) = unbounded::<GuardedTestInput>();
        let receiver = receiver.map(|(frame, dequeued, ledger)| {
            if let Some(ledger) = ledger {
                ledger
                    .pending
                    .fetch_sub(1, std::sync::atomic::Ordering::SeqCst);
                ledger
                    .dequeued
                    .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            }
            if let Some(dequeued) = dequeued {
                let _ = dequeued.send(());
            }
            frame
        });
        (sender, Box::pin(receiver))
    }

    fn send_guarded_test_body(sender: &GuardedTestSender, type_name: &str, body: Vec<u8>) {
        send_guarded_test_body_with_receipt(sender, type_name, body, None, None);
    }

    fn send_guarded_test_body_observed(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
    ) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed_with_ledger(sender, type_name, body, None)
    }

    fn send_guarded_test_body_observed_with_ledger(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
        ledger: Option<Arc<GuardedDequeueLedger>>,
    ) -> oneshot::Receiver<()> {
        let (dequeued, receipt) = oneshot::channel();
        if let Some(ledger) = ledger.as_ref() {
            ledger
                .pending
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }
        send_guarded_test_body_with_receipt(sender, type_name, body, Some(dequeued), ledger);
        receipt
    }

    fn send_guarded_test_body_with_receipt(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
        dequeued: Option<oneshot::Sender<()>>,
        ledger: Option<Arc<GuardedDequeueLedger>>,
    ) {
        let frame = RawLcmFrameBuilder::new(
            LcmTopic::new("/test/guarded/input").expect("guarded test topic"),
            LcmMessageType::new(type_name).expect("guarded test message type"),
        )
        .payload(body)
        .build();
        sender
            .unbounded_send((Ok(frame), dequeued, ledger))
            .expect("guarded test input stream open");
    }

    #[derive(Clone, Copy)]
    enum ColdInput {
        Goal,
        Pose,
        Costmap,
    }

    #[tokio::test]
    async fn guarded_latest_goal_waits_and_replaces_without_input_gates() {
        let harness = GuardedLoopHarness::spawn().await;
        let mut first = sealed_goal("first");
        first.motion_generation=7;
        let mut latest = sealed_goal("latest");
        // 来源代次/运动代次较小也不再由桥的目标水位拒绝。
        latest.motion_generation = 1;
        harness.send_goal_and_wait_dequeued(&first).await;
        harness.send_goal_and_wait_dequeued(&latest).await;
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.scheduler.status().session_id.as_deref()!=Some("latest"){
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        assert!(harness.scheduler.status().message.contains("等待"));
        assert_eq!(harness.motion_call_count("acquire:"), 0);
        harness.send_pose(2);
        harness.send_costmap(12);
        harness.wait_for_pose(2).await;
        harness.wait_for_costmap(12).await;
        harness.send_path(&sealed_path(&latest));
        tokio::time::timeout(Duration::from_secs(1), async {
            while !harness
                .scheduler
                .status()
                .last_applied_twist
                .is_some_and(|v| v != Twist2::ZERO)
            {
                assert!(!harness.loop_task.is_finished());
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("latest")
        );
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        assert_eq!(harness.motion_call_count("release:"), 0);
        harness.shutdown().await;
    }

    #[test]
    fn goal_options_and_map_frame_are_checked_at_decode() {
        let goal = sealed_goal("decode");
        let body = schema::encode_navigation_goal_body(&goal).unwrap();
        assert_eq!(decode_navigation_goal_input(&body).unwrap(), goal);
        let mut foreign = goal.clone();
        foreign.goal.header.frame_id = "odom".into();
        assert!(decode_navigation_goal_input(
            &schema::encode_navigation_goal_body(&foreign).unwrap()
        )
        .unwrap_err()
        .contains("map"));
        let mut options = goal.clone();
        options.position_tolerance_m = Some(0.31415926535);
        let mut invalid = schema::encode_navigation_goal_body(&options).unwrap();
        let at = invalid
            .windows(8)
            .position(|b| b == 0.31415926535_f64.to_be_bytes())
            .unwrap();
        invalid[at..at + 8].copy_from_slice(&(-1_f64).to_be_bytes());
        assert!(decode_navigation_goal_input(&invalid).is_err());
    }

    #[tokio::test]
    async fn guarded_real_loop_accepts_all_cold_goal_pose_costmap_orders() {
        for order in [
            [ColdInput::Goal, ColdInput::Costmap, ColdInput::Pose],
            [ColdInput::Goal, ColdInput::Pose, ColdInput::Costmap],
            [ColdInput::Pose, ColdInput::Goal, ColdInput::Costmap],
        ] {
            let harness = GuardedLoopHarness::spawn().await;
            let goal = sealed_goal("cold-order-goal");
            for (index, input) in order.into_iter().enumerate() {
                match input {
                    ColdInput::Goal => {
                        harness.send_goal(&goal);
                        tokio::time::sleep(Duration::from_millis(20)).await;
                    }
                    ColdInput::Pose => {
                        harness.send_pose(1);
                        harness.wait_for_pose(1).await;
                    }
                    ColdInput::Costmap => {
                        harness.send_costmap(11);
                        harness.wait_for_costmap(11).await;
                    }
                }
                if index < 2 {
                    assert_eq!(harness.motion_call_count("acquire:"),0,"输入未齐时只保存目标");
                    assert!(harness.scheduler.status().last_terminal.is_none());
                }
            }
            harness.wait_for_session("cold-order-goal").await;
            harness.shutdown().await;
        }
    }

    #[tokio::test]
    async fn planar_observation_clear_fences_active_scheduler_and_rejects_replay() {
        use nuwax_codec::lcm::planar_navigation::{self,PlanarNavigationMap};
        let harness=GuardedLoopHarness::spawn_with_planar_mode(true).await;
        let goal=sealed_goal("planar-goal");
        let timestamp=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as i64-100_000_000;
        let mut grid=schema::decode_occupancy_grid_body(&guarded_costmap_body(1)).unwrap();
        grid.header.stamp_sec=(timestamp/1_000_000_000) as i32;
        grid.header.stamp_nsec=(timestamp%1_000_000_000) as i32;
        let value=PlanarNavigationMap {
            map_frame_generation: 0,
            runtime_generation:7,session_id:"guarded-test-map".into(),
            session_generation:2,map_revision:4,
            map_boot_nonce:[0x42;16],map_commit_sequence:1,
            map_source_timestamp_ns:1,observation_revision:1,source_sequence:0,
            source_timestamp_ns:timestamp,cleared:false,grid,
        };
        let body=planar_navigation::encode_body(&value).unwrap();
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,body.clone());
        harness.send_pose(1);harness.wait_for_pose(1).await;
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.scheduler.status().costmap_source_timestamp_ns!=timestamp {
                assert!(!harness.loop_task.is_finished());
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        harness.send_goal(&goal);harness.wait_for_session("planar-goal").await;
        harness.send_path(&sealed_path(&goal));
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.motion_call_count("twist:")==0 {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.unwrap();
        let cleared=PlanarNavigationMap {cleared:true,observation_revision:2,
            grid:Default::default(),..value};
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,
            planar_navigation::encode_body(&cleared).unwrap());
        harness.wait_for_release().await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,body);
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn rejected_lease_publishes_candidate_failure_and_owner_accepts_next_goal() {
        let mut harness = GuardedLoopHarness::spawn_with_first_acquire(false, "base_link", true).await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        let rejected = sealed_goal("rejected-lease");
        harness.send_goal_and_wait_dequeued(&rejected).await;
        let failure = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_status_body(&frame.payload).unwrap();
                if state.session_id == rejected.session_id && state.state == 4 { break state; }
            }
        }).await.expect("必须实际发布被拒绝候选目标的失败");
        assert_eq!(failure.goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);

        assert_eq!(failure.motion_generation, rejected.motion_generation as i64);

        assert_eq!(failure.path_revision, 0);
        assert_eq!(failure.planner_runtime_generation, 0);
        assert_ne!(failure.sink_ready, 0);
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.assert_no_session().await;
        assert_eq!(harness.motion_call_count("rejected_acquire:"), 1);
        assert_eq!(harness.motion_call_count("acquire:"), 0, "失败目标不能被新输入自动复活");
        assert_eq!(harness.motion_call_count("release:"), 0, "未取得租约不能伪造归零");
        let mut next = sealed_goal("lease-recovered");
        next.motion_generation = rejected.motion_generation + 1;

        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session(&next.session_id).await;
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        harness.send_cancel(&next);
        harness.wait_for_cancelled_status(&next).await;
        harness.shutdown().await;
    }

    async fn activate_map_a(harness: &GuardedLoopHarness) -> schema::NavigationGoal {
        let goal_a = sealed_goal("session-a");
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.send_goal(&goal_a);
        harness.wait_for_session("session-a").await;
        goal_a
    }

    async fn finish_map_a_after_lease_release(
        harness: &GuardedLoopHarness,
        goal_a: &schema::NavigationGoal,
    ) {
        harness.send_path(&sealed_path(goal_a));
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("map-a path did not enter controlling phase");
        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().session_id.is_some()

            {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("租约释放后目标未结束");
    }

    #[tokio::test]
    async fn guarded_terminal_goal_reuses_fresh_inputs_before_costmap_update() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-b")
        );
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_terminal_costmap_then_goal_reuses_fresh_pose() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_new_goal_uses_available_fresh_inputs() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.send_pose(3);
        harness.wait_for_pose(3).await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-b")
        );
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_fresh_pose_before_new_goal_is_reused() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;

        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_changed_goal_with_same_id_replaces_and_accepts_the_next_goal() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let first = activate_map_a(&harness).await;
        let mut changed = first.clone();
        changed.goal.pose.position[0] += 0.5;
        harness.send_goal_and_wait_dequeued(&changed).await;
        tokio::time::timeout(Duration::from_millis(50), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_status_body(&frame.payload).unwrap();
                if state.goal[0] == changed.goal.pose.position[0] {
                    break;
                }
            }
        })
        .await
        .expect("变化后的同号目标应直接替换");
        assert_eq!(harness.motion_call_count("release:"), 1);
        let next = rollover_goal(&first, "session-b", 1);
        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_goal_replacement_releases_previous_before_new_acquire() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        assert_eq!(harness.motion_call_count("release:"),1);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_costmap_update_keeps_the_active_session_and_path() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        let path_a = sealed_path(&goal_a);
        harness.send_path(&path_a);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("map-a path was not accepted");

        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-a")
        );
        assert_eq!(harness.scheduler.status().path_revision, 1);

        assert!(!harness
            .motion_calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("release:")));
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_succeeded_terminal_rejects_late_path_without_losing_release_result() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        let path = sealed_path(&goal);
        harness.send_path(&path);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("path accepted before success");

        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().last_terminal != Some(TerminalOutcome::Succeeded) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("scheduler success terminal");
        let terminal = harness.scheduler.status();
        assert_eq!(terminal.phase, SchedulerPhase::Idle);

        harness.send_path(&path);
        let mut wrong_identity = path.clone();
        wrong_identity.session_id.push_str("-wrong");
        schema::validate_path_envelope(&wrong_identity).expect("seal wrong identity path");
        harness.send_path(&wrong_identity);
        tokio::time::sleep(Duration::from_millis(50)).await;
        let after_late = harness.scheduler.status();
        assert_eq!(after_late.phase, SchedulerPhase::Idle);
        assert_eq!(after_late.last_terminal, Some(TerminalOutcome::Succeeded));

        harness.shutdown().await;
    }

    #[test]
    fn guarded_local_display_survives_heartbeats_and_clears_at_terminal() {
        let telemetry = GuardedNavTelemetry { public_session_id: "display-goal".into(), ..Default::default() };
        let mut status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Controlling,
            session_id: Some("display-goal".into()),
            local_path: vec![[0.0,0.0,0.4],[1.0,0.5,0.6]],
            ..Default::default()
        };
        for observation in [None, Some(false), Some(true)] {
            let wire = build_guarded_navigation_status(&status, &telemetry, observation, Duration::from_secs(1));
            let decoded = schema::decode_navigation_status(&schema::encode_navigation_status(&wire).unwrap()).unwrap();
            assert_eq!(decoded.local_path, status.local_path);
        }
        status.last_terminal = Some(nuwax_nav_runtime::TerminalOutcome::Succeeded);
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(2)).local_path.is_empty());
        status.last_terminal = None;
        status.session_id = Some("old-goal".into());
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(3)).local_path.is_empty());
        status.session_id = Some("display-goal".into());
        status.path_revision = 1;
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(4)).local_path.is_empty());
    }

    #[test]
    fn guarded_navigation_status_receipt_projection_is_read_only() {
        let telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            ..GuardedNavTelemetry::default()
        };
        let baseline_status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-a".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let baseline = build_guarded_navigation_status(
            &baseline_status,
            &telemetry,
            None,
            Duration::from_secs(1),
        );
        assert_eq!(baseline.pose_source_timestamp_ns, 0);
        assert_eq!(baseline.costmap_source_timestamp_ns, 0);

        let status = nuwax_nav_runtime::SchedulerStatus {
            pose_source_timestamp_ns: 11,
            costmap_source_timestamp_ns: 17,
            ..baseline_status
        };
        let projected =
            build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(1));
        assert_eq!(projected.pose_source_timestamp_ns, 11);
        assert_eq!(projected.costmap_source_timestamp_ns, 17);
        assert_eq!(
            build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(1)),
            projected,
            "replaying the same scheduler snapshot must not advance receipt identity",
        );

        let mut without_receipts = projected.clone();
        without_receipts.pose_source_timestamp_ns = 0;
        without_receipts.costmap_source_timestamp_ns = 0;
        assert!(baseline.reason.contains("尚未收到，年龄未知"));
        assert!(projected.reason.contains("来源年龄="));
        assert_eq!(projected.message, projected.reason);
        // 来源时间也用于等待诊断；除回执和诊断文字外，状态投影保持一致。
        without_receipts.reason = baseline.reason.clone();
        without_receipts.message = baseline.message.clone();
        assert_eq!(
            without_receipts, baseline,
            "回执投影只影响来源时间与对应的输入年龄诊断"
        );

        let rejected = guarded_rejection_status(status, "path", "out-of-order candidate");
        assert_eq!(rejected.pose_source_timestamp_ns, 11);
        assert_eq!(rejected.costmap_source_timestamp_ns, 17);
    }

    #[test]
    fn nav_state_limits_ordinary_updates_and_immediately_publishes_transitions() {
        let mut publications = GuardedStatusPublications::default();
        let mut status = nuwax_nav_runtime::SchedulerStatus { phase: SchedulerPhase::Controlling, ..Default::default() };
        let mut telemetry = GuardedNavTelemetry { public_session_id: "goal-a".into(), ..Default::default() };
        let start = Instant::now();
        let mut emitted = 0;
        for ms in 0..1000 {
            status.remaining_distance_m = f64::from(ms);
            emitted += usize::from(publications.state_frame_at("/nav/state", &status, &telemetry, None,
                start + Duration::from_millis(ms as u64)).unwrap().is_some());
        }
        assert_eq!(emitted, 10);
        let now = start + Duration::from_millis(999);
        status.phase = SchedulerPhase::AwaitingPath;
        assert!(publications.state_frame_at("/nav/state", &status, &telemetry, None, now).unwrap().is_some());
        status.last_terminal = Some(TerminalOutcome::Succeeded);
        assert!(publications.state_frame_at("/nav/state", &status, &telemetry, None, now).unwrap().is_some());
        telemetry.public_session_id = "goal-b".into();
        assert!(publications.state_frame_at("/nav/state", &status, &telemetry, None, now).unwrap().is_some());
        status.remaining_distance_m = 1234.;
        let frame = publications.state_frame_at("/nav/state", &status, &telemetry, None,
            now + Duration::from_millis(100)).unwrap().unwrap();
        let decoded = schema::decode_navigation_status_body(&frame.payload).unwrap();
        assert_eq!(decoded.remaining_distance_m, 1234.);
    }

    #[test]
    fn guarded_input_rejection_preserves_the_real_scheduler_phase() {
        let status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-a".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let rejected = guarded_rejection_status(status, "path", "out-of-order candidate");
        assert_eq!(rejected.phase, SchedulerPhase::AwaitingPath);
        assert_eq!(rejected.message, "REJECTED path: out-of-order candidate");

        let telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            ..GuardedNavTelemetry::default()
        };
        let published =
            build_guarded_navigation_status(&rejected, &telemetry, None, Duration::from_secs(1));
        assert_eq!(published.state, 1, "rejection must not fabricate FAILED");
    }

    #[test]
    fn late_path_rejection_keeps_terminal_and_terminal_cause_and_receipts() {
        for (phase,terminal) in [
            (SchedulerPhase::Cancelling,None),(SchedulerPhase::Cancelling,None),
            (SchedulerPhase::Failed,Some(nuwax_nav_runtime::TerminalOutcome::SafetyFailed)),
            (SchedulerPhase::Idle,Some(nuwax_nav_runtime::TerminalOutcome::Succeeded)),
            (SchedulerPhase::Idle,Some(nuwax_nav_runtime::TerminalOutcome::Cancelled)),
        ] {
            let status=nuwax_nav_runtime::SchedulerStatus{phase,last_terminal:terminal,
                message:"原始停车原因及确认结果".into(),session_id:Some("original".into()),
                pose_source_timestamp_ns:17,costmap_source_timestamp_ns:23,..Default::default()};
            assert_eq!(guarded_rejection_status(status.clone(),"path","no active navigation session"),status);
        }
    }

    #[test]
    fn terminal_publication_transitions_to_unowned_guard_heartbeat() {
        let mut active = Some(active_goal("goal-a"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Idle,
            motion_sink_ready: true,

            last_terminal: Some(TerminalOutcome::Cancelled),

            message: "租约已释放".into(),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let mut telemetry = GuardedNavTelemetry {
            runtime_generation: 42,
            public_session_id: "goal-a".into(),
            motion_generation: 1,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            goal: [1.0, 2.0, 0.0],
            current: [0.5, 0.25, 0.0],
            current_z: 0.0,
            goal_z: 0.0,
            surface_navigation: false,
            initial_distance: 3.0,
            progress: 0.5,
            path_revision: 3,
            planner_runtime_generation: 22,
        };

        let terminal_message =
            build_guarded_navigation_status(&terminal, &telemetry, None, Duration::from_secs(1));
        assert_eq!(terminal_message.session_id, "goal-a");
        assert_eq!(terminal_message.planner_runtime_generation, 22);
        assert_eq!(terminal_message.runtime_generation, 42);
        assert_eq!(terminal_message.reason, terminal.message);

        clear_terminal_telemetry_after_publication(&mut telemetry, &terminal);
        assert!(active.is_some());

        let heartbeat = build_guarded_navigation_status(
            &terminal,
            &telemetry,
            Some(true),
            Duration::from_secs(2),
        );
        assert!(heartbeat.session_id.is_empty());
        assert_eq!(heartbeat.state, 0);
        assert_eq!(heartbeat.goal_status, schema::NAVIGATION_GOAL_STATUS_NONE);
        assert!(!heartbeat.input_ready);
        assert!(heartbeat.reason.contains("等待导航输入"));
        assert!(heartbeat.reason.contains("尚未收到，年龄未知"));
        assert_eq!(heartbeat.sink_ready, 1);
        assert_eq!(heartbeat.motion_generation, 0);
        assert_eq!(heartbeat.path_revision, 0);
        assert_eq!(heartbeat.planner_runtime_generation, 0);
        assert_eq!(heartbeat.runtime_generation, 42);
        assert_eq!(heartbeat.planner_map_frame, "map");
        assert_eq!(heartbeat.planner_pose_frame, "map");
        assert_eq!(heartbeat.current, [0.5, 0.25, 0.0]);
    }

    #[test]
    fn path_and_cancel_match_active_goal_identity() {
        let goal = sealed_goal("public-goal-1");
        let path = sealed_path(&goal);
        validate_path_goal_correlation(&path, &goal).expect("correlated path");

        let mut foreign_path = path.clone();
        foreign_path.session_id.push_str("-other");
        assert!(validate_path_goal_correlation(&foreign_path, &goal).is_err());

        let cancel = schema::CancelNavigation {
            session_id: goal.session_id.clone(),
            reason: 1,
        };
        validate_cancel_goal_correlation(&cancel, &goal).expect("correlated cancel");
        let mut foreign_cancel = cancel;
        foreign_cancel.session_id.push_str("-other");
        assert!(validate_cancel_goal_correlation(&foreign_cancel, &goal).is_err());
    }

    #[test]
    fn map_pose_owner_drops_bad_tf_and_processes_following_valid_odometry() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let tf = schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [1.0, 2.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        };
        let tf_body = schema::encode_tf_message_body(&tf).expect("encode TF");
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("valid TF input"),
            OwnerInput::Accepted(None)
        ));
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("重复来源时间的TF按到达顺序接收"),
            OwnerInput::Accepted(None)
        ));
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &[0x01, 0x02]).expect("bad TF is external"),
            OwnerInput::Rejected(_)
        ));

        let odometry = schema::Odometry {
            header: schema::Header {
                stamp_sec: 7,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            pose: schema::Pose {
                position: [3.0, 4.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        };
        let odometry_body = schema::encode_odometry_body(&odometry).expect("encode odometry body");
        let observed = match ingest_map_pose_odometry(&adapter, &odometry_body)
            .expect("valid odometry after bad TF")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("valid odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("valid odometry rejected: {reason}")
            }
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }

    #[test]
    fn queued_pose_uses_only_last_frame_and_preserves_stream_failure() {
        let (sender, mut poses) = guarded_test_stream();
        let make = |stamp| {
            build_body_frame(
                "/test/pose",
                "geometry_msgs.PoseStamped",
                guarded_pose_body(stamp),
            )
            .unwrap()
        };
        for stamp in 3..103 {
            sender.unbounded_send((Ok(make(stamp)), None, None)).unwrap();
        }
        sender.unbounded_send((Ok(make(2)), None, None)).unwrap();
        let mut diagnostics = InputDiagnosticLimiter::default();
        let input = drain_guarded_pose_input(Some(Ok(make(1))), &mut poses, &mut diagnostics)
            .unwrap()
            .unwrap();
        assert_eq!(input.pose.stamp.nanos, 2_000_000_000);
        drop(sender);
        assert!(
            drain_guarded_pose_input(Some(Ok(make(4))), &mut poses, &mut diagnostics)
                .is_err()
        );
    }

    #[test]
    fn queued_pose_defers_source_age_to_the_control_tick() {
        let (sender, mut poses) = guarded_test_stream();
        let current = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap();
        let mut pose = schema::PoseStamped {
            header: schema::Header {
                stamp_sec: current.as_secs() as i32,
                stamp_nsec: current.subsec_nanos() as i32,
                frame_id: "map".into(),
                ..Default::default()
            },
            pose: schema::Pose {
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                ..Default::default()
            },
        };
        let body = |pose: &schema::PoseStamped| {
            build_body_frame(
                "/test/pose",
                "geometry_msgs.PoseStamped",
                schema::encode_pose_stamped_body(pose).unwrap(),
            )
            .unwrap()
        };
        let valid = body(&pose);
        pose.header.stamp_sec += 3600;
        sender
            .unbounded_send((Ok(body(&pose)), None, None))
            .unwrap();
        let input = drain_guarded_pose_input(
            Some(Ok(valid)),
            &mut poses,
            &mut InputDiagnosticLimiter::default(),
        )
        .unwrap().unwrap();
        assert_eq!(input.pose.stamp.nanos / 1_000_000_000, i64::from(pose.header.stamp_sec));
    }

    #[test]
    fn status_publications_reset_per_goal_and_publish_success_once() {
        let mut node = TaskChannels { id: "guard-test".into(), inputs: Vec::new(), outputs: Vec::new() };
        node.outputs = vec![
            "/test/cmd".into(),
            "/test/state".into(),
            "/test/reached".into(),
        ];
        let mut publications = GuardedStatusPublications::default();
        let mut telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            motion_generation: 1,
            ..Default::default()
        };
        let mut status = nuwax_nav_runtime::SchedulerStatus {
            applied_commands: 12,
            last_applied_twist: Some(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.0,
            }),
            ..Default::default()
        };
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            2
        );
        status.last_terminal = Some(TerminalOutcome::Succeeded);
        let terminal = publications
            .frames(&node, &status, &telemetry, Some(true))
            .unwrap();
        assert_eq!(terminal.len(), 3);
        assert_eq!(terminal[1].msg_type.fqn(), "std_msgs.Bool");
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            0
        );
        clear_guarded_session_telemetry(&mut telemetry);
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            1
        );
        telemetry.public_session_id = "goal-b".into();
        telemetry.motion_generation = 2;
        status.last_terminal = None;

        status.applied_commands = 1;
        let next = publications
            .frames(&node, &status, &telemetry, None)
            .unwrap();
        assert_eq!(
            next.len(),
            2,
            "第二个目标首个执行回执不能被上一目标累计计数吞掉"
        );
        assert_eq!(next[0].msg_type.fqn(), "geometry_msgs.Twist");
    }

    #[test]
    fn map_pose_owner_retries_pending_odometry_when_matching_tf_arrives() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let odometry = schema::Odometry {
            header: schema::Header {
                stamp_sec: 7,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            pose: schema::Pose {
                position: [3.0, 4.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        };
        let odometry_body = schema::encode_odometry_body(&odometry).expect("encode odometry body");
        let pending = match ingest_map_pose_odometry(&adapter, &odometry_body)
            .expect("odometry before TF is retryable")
        {
            MapPoseOdometryIngress::AwaitingTransform(odometry, _) => odometry,
            MapPoseOdometryIngress::Accepted(_) => panic!("odometry unexpectedly accepted"),
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("retryable odometry rejected: {reason}")
            }
        };

        let tf_body = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [1.0, 2.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode TF");
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("matching TF"),
            OwnerInput::Accepted(None)
        ));

        let observed = match transform_map_pose_odometry(&adapter, *pending)
            .expect("retry pending odometry")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("aligned odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("aligned odometry rejected: {reason}")
            }
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }



    #[test]
    fn map_pose_owner_accepts_a_coherent_tf_chain_without_odometry_join() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let tf_body = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![
                schema::TransformStamped {
                    header: schema::Header {
                        stamp_sec: 7,
                        frame_id: "map".into(),
                        ..schema::Header::default()
                    },
                    child_frame_id: "odom".into(),
                    transform: schema::Transform {
                        translation: [1.0, 2.0, 0.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
                schema::TransformStamped {
                    header: schema::Header {
                        stamp_sec: 7,
                        frame_id: "odom".into(),
                        ..schema::Header::default()
                    },
                    child_frame_id: "base_link".into(),
                    transform: schema::Transform {
                        translation: [3.0, 4.0, 0.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
            ],
        })
        .expect("encode coherent TF");

        let observed = match ingest_map_pose_tf(&mut adapter, &tf_body).expect("coherent TF input")
        {
            OwnerInput::Accepted(Some(observed)) => observed,
            OwnerInput::Accepted(None) => panic!("coherent TF did not produce map pose"),
            OwnerInput::Rejected(reason) => panic!("coherent TF rejected: {reason}"),
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }

    #[test]
    fn guarded_owner_drops_bad_sensor_frames_and_ignores_costmap_sequence() {
        let mut telemetry = GuardedNavTelemetry {
            map_frame: "map".into(),
            pose_frame: "map".into(),
            current: [9.0, 8.0, 0.0],
            ..GuardedNavTelemetry::default()
        };
        let before_bad_pose = telemetry.clone();
        assert!(matches!(
            decode_guarded_pose_input(&[0x7f]).expect("bad pose is external"),
            OwnerInput::Rejected(_)
        ));
        assert_eq!(telemetry, before_bad_pose);
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid pose");
        let pose_input = match decode_guarded_pose_input(&pose_body).expect("valid pose") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("valid pose rejected: {reason}"),
        };
        commit_guarded_pose_telemetry(&mut telemetry, &pose_input);
        assert_eq!(telemetry.current, [1.0, 2.0, 0.0]);

        let mut non_finite_pose =
            schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, &pose_body))
                .expect("decode pose fixture for mutation");
        non_finite_pose.pose.position[0] = f64::NAN;
        let non_finite_body =
            schema::encode_pose_stamped_body(&non_finite_pose).expect("encode non-finite pose");
        assert!(matches!(
            decode_guarded_pose_input(&non_finite_body).expect("non-finite pose is external"),
            OwnerInput::Rejected(_)
        ));

        let before_bad_costmap = telemetry.clone();
        assert!(matches!(
            decode_guarded_costmap_input(&[0x7f]).expect("bad costmap is external"),
            OwnerInput::Rejected(_)
        ));
        assert_eq!(telemetry, before_bad_costmap);
        let grid_body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 8,
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode valid costmap");
        let input = match decode_guarded_costmap_input(&grid_body).expect("valid costmap") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("valid costmap rejected: {reason}"),
        };
        commit_guarded_costmap_telemetry(&mut telemetry, &input);
        assert_eq!(telemetry.map_frame, "map");
    }

    #[test]
    fn guarded_controller_preserves_unknown_and_stops_for_observed_obstacle() {
        use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerConfig};
        use nuwax_nav_astar::navigation_map::{OCCUPIED, UNKNOWN};

        let width = 60_usize;
        let height = 20_usize;
        let mut data = vec![-1; width * height];
        data[width * height - 1] = 100;
        let grid_body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                stamp_sec: 9,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.1,
                width: width as i32,
                height: height as i32,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data,
        })
        .expect("encode unknown exploration costmap");
        let input = match decode_guarded_costmap_input(&grid_body).expect("decode costmap") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("unknown costmap rejected: {reason}"),
        };
        assert_eq!(planar_test_costmap(&input.observation).get(10, 5), UNKNOWN);
        assert_eq!(
            planar_test_costmap(&input.observation).get(width - 1, height - 1),
            OCCUPIED
        );

        let mut planner = LocalPlanner::with_config(LocalPlannerConfig::default_config(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
        planner.update_costmap(match input.observation.costmap {
            nuwax_nav_runtime::ExecutionMap::Planar(map) => map,
            _ => panic!("expected planar map"),
        });
        planner.handle_odom(PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                orientation: Quaternion::IDENTITY,
                ..Pose3::default()
            },
            NavFrameId::Map,
        ));
        planner.start_planning(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::from_unix(9, 0),
            vec![Point2::new(1.0, 0.5), Point2::new(2.0, 0.5)],
        ));

        let (_, stop) = planner.step();
        assert_eq!(stop, None, "未知格保留未知语义，不能冒充已观测障碍");
        let mut observed=schema::decode_occupancy_grid_body(&grid_body).unwrap();
        observed.data[5*width+15]=100;
        let body=schema::encode_occupancy_grid_body(&observed).unwrap();
        let input=match decode_guarded_costmap_input(&body).unwrap() {
            OwnerInput::Accepted(input)=>input,
            OwnerInput::Rejected(reason)=>panic!("实测障碍拒收：{reason}"),
        };
        if let nuwax_nav_runtime::ExecutionMap::Planar(map)=input.observation.costmap {
            planner.update_costmap(map);
        } else {panic!("二维障碍被转换为其它地图");}
        let (twist,stop)=planner.step();
        assert_eq!(stop,Some(nuwax_nav_astar::local_planner::StopMessage::ObstacleFound));
        assert_eq!(twist,Twist2::ZERO);
    }

    #[test]
    fn scheduler_task_error_wins_when_scheduler_exits_first() {
        let result = guarded_scheduler_result(
            true,
            Err(NavStackError::Internal(
                "guard scheduler status stream closed".into(),
            )),
            Ok(()),
            Err(NavStackError::Internal("motion guard root cause".into())),
        );

        assert!(result
            .expect_err("fatal scheduler task must fail")
            .to_string()
            .contains("motion guard root cause"));
    }

    #[test]
    fn scheduler_loop_error_survives_normal_task_and_shutdown() {
        let result = guarded_scheduler_result(
            true,
            Err(NavStackError::Internal("original loop error".into())),
            Ok(()),
            Ok(()),
        );

        assert!(result
            .expect_err("loop failure must survive")
            .to_string()
            .contains("original loop error"));
    }

    #[test]
    fn scheduler_loop_error_wins_when_loop_exits_first() {
        let result = guarded_scheduler_result(
            false,
            Err(NavStackError::Internal("original loop error".into())),
            Ok(()),
            Err(NavStackError::Internal("later task error".into())),
        );

        assert!(result
            .expect_err("first loop failure must survive")
            .to_string()
            .contains("original loop error"));
    }

    #[tokio::test]
    async fn scheduler_task_completion_is_recorded_before_loop_cleanup() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let (release_loop, loop_released) = oneshot::channel();
        let shutdown_called = Arc::new(AtomicBool::new(false));
        let shutdown_observer = Arc::clone(&shutdown_called);

        let result = coordinate_guarded_scheduler(
            async {
                loop_released.await.expect("task releases loop cleanup");
                Err(NavStackError::Internal(
                    "guard scheduler status stream closed".into(),
                ))
            },
            async {
                release_loop.send(()).expect("loop cleanup receiver alive");
                Err(NavStackError::Internal("motion guard root cause".into()))
            },
            move || async move {
                shutdown_observer.store(true, Ordering::SeqCst);
                Ok(())
            },
        )
        .await;

        assert!(result
            .expect_err("scheduler task failure must win")
            .to_string()
            .contains("motion guard root cause"));
        assert!(!shutdown_called.load(Ordering::SeqCst));
    }

    #[tokio::test]
    async fn scheduler_loop_completion_triggers_shutdown_and_stays_primary() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let (release_task, task_released) = oneshot::channel();
        let shutdown_called = Arc::new(AtomicBool::new(false));
        let shutdown_observer = Arc::clone(&shutdown_called);

        let result = coordinate_guarded_scheduler(
            async {
                release_task.send(()).expect("task receiver alive");
                Err(NavStackError::Internal("original loop error".into()))
            },
            async {
                task_released.await.expect("loop releases task cleanup");
                Err(NavStackError::Internal("later task error".into()))
            },
            move || async move {
                shutdown_observer.store(true, Ordering::SeqCst);
                Ok(())
            },
        )
        .await;

        assert!(result
            .expect_err("first loop failure must survive")
            .to_string()
            .contains("original loop error"));
        assert!(shutdown_called.load(Ordering::SeqCst));
    }

    #[test]
    fn retired_motion_failures_are_recoverable_but_scheduler_closure_is_fatal() {
        use nuwax_nav_runtime::RuntimeError;

        for error in [RuntimeError::SchedulerStopped] {
            assert!(runtime_input_error::<()>("guarded-test", error).is_err());
        }
        assert!(matches!(runtime_input_error::<()>("guarded-test", RuntimeError::Motion("guard unavailable".into())).unwrap(),
            OwnerInput::Rejected(reason) if reason.contains("guard unavailable")));
        assert!(matches!(
            runtime_input_error::<()>("guarded-test", RuntimeError::StaleInput("old pose".into()),)
                .expect("stale external input is recoverable"),
            OwnerInput::Rejected(_)
        ));

        for component in [
            "map_pose_adapter.tf",
            "guarded_nav_scheduler.goal",
            "simple_planner.map_pose",
            "astar_planner.costmap",
        ] {
            assert!(required_frame(None, component).is_err(), "{component}");
        }
    }

    #[test]
    fn external_input_diagnostics_are_rate_limited() {
        let start = Instant::now();
        let mut limiter = InputDiagnosticLimiter::default();
        assert_eq!(limiter.record_at(start), Some(0));
        assert_eq!(limiter.record_at(start + Duration::from_millis(10)), None);
        assert_eq!(limiter.record_at(start + Duration::from_millis(20)), None);
        assert_eq!(
            limiter.record_at(start + INPUT_DIAGNOSTIC_INTERVAL),
            Some(2)
        );
    }
