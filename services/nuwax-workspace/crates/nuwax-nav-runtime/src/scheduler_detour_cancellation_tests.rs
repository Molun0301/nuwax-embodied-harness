// Test-only causal audit. Real scheduler, worker, route and planning kernel;
// the in-process acknowledged motion port is not a physical actuator oracle.
mod scheduler_detour_cancellation {
    use super::*;
    use crate::route_follower::RouteFollower;
    use crate::surface_changes::{SurfaceGeometryChanges, SurfaceMapUpdate};
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{GoalOptions, Path3D, Point3, Quaternion};
    use std::sync::{atomic::AtomicBool, mpsc};
    use std::time::{SystemTime, UNIX_EPOCH};

    fn now_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
            .try_into()
            .unwrap()
    }
    fn pose(x: f64, y: f64, yaw: f64) -> PoseStamped {
        let ns = now_ns();
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix((ns / 1_000_000_000) as i64, (ns % 1_000_000_000) as u32),
            pose: Pose3 {
                position: Point3::new(x, y, 0.5),
                orientation: Quaternion::from_yaw(yaw),
            },
        }
    }
    type Trace = Arc<Mutex<Vec<String>>>;
    fn log(trace: &Trace, kind: &str, value: impl std::fmt::Debug) {
        let mut rows = trace.lock().unwrap();
        assert!(rows.len() < 10_000, "bounded diagnostic trace");
        rows.push(format!("{} {} {:?}", now_ns(), kind, value));
    }
    type Lifecycle = (Option<u64>, u64, Option<u64>, u64, u64);
    type Stops = Arc<Mutex<Vec<(Lifecycle, Lifecycle)>>>;
    struct TraceDump(Trace);
    impl Drop for TraceDump {
        fn drop(&mut self) {
            for event in self.0.lock().unwrap().iter() {
                eprintln!("SCHEDULER_DETOUR {event}");
            }
        }
    }
    struct Admission {
        lifecycle: Lifecycle,
        pose: PoseStamped,
        applied: Twist2,
        path: Path3D,
        map: Arc<SurfaceSnapshot>,
    }
    #[derive(Default)]
    struct AuditState {
        latest: Option<Lifecycle>,
        admissions: Vec<Admission>,
        rejection_after_detour: bool,
        command_results: u64,
    }
    type State = Arc<Mutex<AuditState>>;
    struct Probe {
        inner: RouteFollower,
        trace: Trace,
        stops: Stops,
        state: State,
        pose: Option<PoseStamped>,
        map: Option<Arc<SurfaceSnapshot>>,
        applied: Twist2,
    }
    impl Probe {
        fn record(&self, label: &str, result: &PlannerStep) {
            let lifecycle = self.inner.planning_lifecycle_for_test();
            log(&self.trace, label, (lifecycle, result));
            let mut state = self.state.lock().unwrap();
            if state.latest.is_none_or(|old| lifecycle.4 > old.4) && lifecycle.4 > 0 {
                assert!(state.admissions.len() < 64, "bounded candidate history");
                state.admissions.push(Admission {
                    lifecycle,
                    pose: self.pose.clone().unwrap(),
                    applied: self.applied,
                    path: self.inner.local_path().unwrap().clone(),
                    map: self.map.clone().unwrap(),
                });
            }
            state.latest = Some(lifecycle);
            if matches!(result, PlannerStep::Command(_)) { state.command_results += 1; }
            if lifecycle.4 >= 2 && matches!(result, PlannerStep::Pending(_)) {
                state.rejection_after_detour = true;
            }
        }
    }
    impl NavPlanner for Probe {
        fn configure_goal(&mut self, p: &PoseStamped, o: GoalOptions) -> Result<(), RuntimeError> {
            self.inner.configure_goal(p, o)
        }
        fn configure_boundary(&mut self, b: &[[f64; 2]]) {
            self.inner.configure_boundary(b)
        }
        fn replace_path(&mut self, p: Path2D) {
            self.inner.replace_path(p)
        }
        fn replace_execution_path(
            &mut self,
            p: ExecutionPath,
            y: Option<Vec<f64>>,
            g: PoseStamped,
        ) -> Result<(), RuntimeError> {
            self.inner.replace_execution_path(p, y, g)
        }
        fn update_odometry(&mut self, p: PoseStamped) {
            self.pose = Some(p.clone());
            self.inner.update_odometry(p)
        }
        fn update_costmap(&mut self, m: CostMap2D) {
            self.inner.update_costmap(m)
        }
        fn update_execution_map(&mut self, m: ExecutionMap) -> Result<(), RuntimeError> {
            self.map = match &m {
                ExecutionMap::Surface(s) => Some(s.clone()),
                ExecutionMap::SurfaceUpdate(u) => Some(u.snapshot.clone()),
                _ => None,
            };
            self.inner.update_execution_map(m)
        }
        fn invalidate_execution_map(&mut self) {
            self.inner.invalidate_execution_map()
        }
        fn observe_applied_command(&mut self, c: Twist2) {
            self.applied = c;
            self.inner.observe_applied_command(c)
        }
        fn step(&mut self, w: f64) -> PlannerStep {
            let r = self.inner.step(w);
            self.record("step", &r);
            r
        }
        fn revalidate_selected_command(&mut self, c: Twist2) -> Option<PlannerStep> {
            let r = self.inner.revalidate_selected_command(c);
            if let Some(r) = &r {
                self.record("recheck", r);
            }
            r
        }
        fn revalidate_selected_command_at_current_state(
            &mut self,
            c: Twist2,
        ) -> Option<PlannerStep> {
            let r = self.inner.revalidate_selected_command_at_current_state(c);
            if let Some(r) = &r {
                self.record("state_recheck", r);
            }
            r
        }
        fn take_command_dispatch_bound(&mut self) -> Option<CommandDispatchBound> {
            self.inner.take_command_dispatch_bound()
        }
        fn execution_wait_reason(&self) -> Option<String> {
            self.inner.execution_wait_reason()
        }
        fn local_path_for_display(&self) -> Vec<[f64; 3]> {
            self.inner.local_path_for_display()
        }
        fn stop(&mut self) {
            let before = self.inner.planning_lifecycle_for_test();
            log(&self.trace, "stop_before", before);
            self.inner.stop();
            let after = self.inner.planning_lifecycle_for_test();
            log(&self.trace, "stop_after", after);
            self.stops.lock().unwrap().push((before, after));
        }
    }
    use nuwax_nav_control::{integrate, PlanarPose};
    type WallInstant = std::time::Instant;
    type Grant = Arc<
        Mutex<
            Option<(
                WallInstant,
                crate::command_proof_trace::DeliveredCommandContext,
            )>,
        >,
    >;
    struct DeliveryProbe {
        inner: planner_worker::PlannerWorker,
        grant: Grant,
    }
    impl NavPlanner for DeliveryProbe {
        fn configure_goal(&mut self, p: &PoseStamped, o: GoalOptions) -> Result<(), RuntimeError> {
            self.inner.configure_goal(p, o)
        }
        fn configure_boundary(&mut self, b: &[[f64; 2]]) {
            self.inner.configure_boundary(b)
        }
        fn replace_path(&mut self, p: Path2D) {
            self.inner.replace_path(p)
        }
        fn replace_execution_path(
            &mut self,
            p: ExecutionPath,
            y: Option<Vec<f64>>,
            g: PoseStamped,
        ) -> Result<(), RuntimeError> {
            self.inner.replace_execution_path(p, y, g)
        }
        fn update_odometry(&mut self, p: PoseStamped) {
            self.inner.update_odometry(p)
        }
        fn update_costmap(&mut self, m: CostMap2D) {
            self.inner.update_costmap(m)
        }
        fn update_execution_map(&mut self, m: ExecutionMap) -> Result<(), RuntimeError> {
            self.inner.update_execution_map(m)
        }
        fn invalidate_execution_map(&mut self) {
            self.inner.invalidate_execution_map()
        }
        fn observe_applied_command(&mut self, c: Twist2) {
            self.inner.observe_applied_command(c)
        }
        fn prepare_control_step(&mut self, w: f64, continuing_wait: bool) -> Option<watch::Receiver<()>> {
            self.inner.prepare_control_step(w, continuing_wait)
        }
        fn step(&mut self, w: f64) -> PlannerStep {
            let result = self.inner.step(w);
            *self.grant.lock().unwrap() = if matches!(result,PlannerStep::Command(c) if c!=Twist2::ZERO)
            {
                Some((
                    self.inner
                        .delivered_command_deadline_for_test()
                        .expect("original worker deadline"),
                    self.inner
                        .delivered_command_context_for_test()
                        .expect("exact delivered proof")
                        .clone(),
                ))
            } else {
                None
            };
            result
        }
        fn take_command_dispatch_bound(&mut self) -> Option<CommandDispatchBound> {
            self.inner.take_command_dispatch_bound()
        }
        fn execution_wait_reason(&self) -> Option<String> {
            self.inner.execution_wait_reason()
        }
        fn local_path_for_display(&self) -> Vec<[f64; 3]> {
            self.inner.local_path_for_display()
        }
        fn stop(&mut self) {
            *self.grant.lock().unwrap() = None;
            self.inner.stop()
        }
    }
    struct Receipt {
        operation_id: String,
        map: Arc<SurfaceSnapshot>,
        pose: PlanarPose,
        before: Twist2,
        command: Twist2,
        ns: u64,
        source_ns: u64,
        id: u64,
        proof: crate::command_proof_trace::DeliveredCommandContext,
    }
    struct Edge {
        map: Arc<SurfaceSnapshot>,
        from: PlanarPose,
        to: PlanarPose,
    }
    struct Plant {
        pose: PlanarPose,
        command: Twist2,
        last: WallInstant,
        deadline: WallInstant,
        map: Arc<SurfaceSnapshot>,
        source_ns: u64,
        receipts: Vec<Receipt>,
        edges: Vec<Edge>,
    }
    impl Plant {
        fn advance(&mut self, now: WallInstant) {
            let end = now.min(self.deadline);
            if end > self.last && self.command != Twist2::ZERO {
                let dt = end.duration_since(self.last).as_secs_f64();
                let to = integrate(self.pose, self.command, dt);
                assert!(self.edges.len() < 10_000, "bounded actual-edge history");
                self.edges.push(Edge {
                    map: self.map.clone(),
                    from: self.pose,
                    to,
                });
                self.pose = to;
            }
            self.last = now;
            if now >= self.deadline {
                self.command = Twist2::ZERO;
            }
        }
        fn measured_pose(&mut self) -> PoseStamped {
            self.advance(WallInstant::now());
            pose(self.pose.x, self.pose.y, self.pose.yaw)
        }
    }
    struct Motion {
        inner: FakeMotion,
        trace: Trace,
        plant: Arc<Mutex<Plant>>,
        grant: Grant,
    }
    #[async_trait]
    impl NavMotionPort for Motion {
        async fn acquire(
            &mut self,
            id: &str,
            a: &NavMotionAdmission,
        ) -> Result<LeaseProof, MotionPortError> {
            self.inner.acquire(id, a).await
        }
        async fn renew(&mut self, id: &str) -> Result<(), MotionPortError> {
            self.inner.renew(id).await
        }
        async fn submit_twist(
            &mut self,
            id: &str,
            c: Twist2,
        ) -> Result<AppliedProof, MotionPortError> {
            match self.submit_twist_bounded(id, c, None).await? {
                BoundedTwistOutcome::Applied(proof) => Ok(proof),
                _ => Err(MotionPortError::new("expired unbounded fixture command")),
            }
        }
        async fn submit_twist_bounded(
            &mut self,
            id: &str,
            c: Twist2,
            bound: Option<CommandDispatchBound>,
        ) -> Result<BoundedTwistOutcome, MotionPortError> {
            let now = WallInstant::now();
            if bound.is_some_and(CommandDispatchBound::expired_now) {
                return Ok(BoundedTwistOutcome::ExpiredNotApplied);
            }
            {
                let mut plant = self.plant.lock().unwrap();
                plant.advance(now);
                let mut deadline = now + DEFAULT_TICK_INTERVAL;
                if c != Twist2::ZERO {
                    let (original, proof) = self
                        .grant
                        .lock()
                        .unwrap()
                        .take()
                        .expect("each nonzero delivery has its original unique worker proof");
                    let result_id = proof.proof.input.result_id;
                    assert!(
                        original > now,
                        "no expired worker selection may reach actual application"
                    );
                    deadline = deadline.min(original);
                    if let Some(bound) = bound {
                        deadline = deadline.min(bound.monotonic_deadline().unwrap());
                    }
                    assert!(
                        plant.receipts.len() < 1_200,
                        "bounded actual receipt history"
                    );
                    assert!(
                        plant.receipts.last().is_none_or(|r| result_id > r.id),
                        "one worker result cannot be applied twice"
                    );
                    let receipt = Receipt {
                        operation_id: id.to_owned(),
                        map: plant.map.clone(),
                        pose: plant.pose,
                        before: plant.command,
                        command: c,
                        ns: now_ns(),
                        source_ns: plant.source_ns,
                        id: result_id,
                        proof,
                    };
                    plant.receipts.push(receipt);
                }
                plant.command = c;
                plant.deadline = deadline;
            }
            let r = self.inner.submit_twist(id, c).await?;
            log(&self.trace, "applied", (id, c));
            Ok(BoundedTwistOutcome::Applied(r))
        }
        async fn revoke_and_zero(&mut self, id: &str) -> Result<FenceProof, MotionPortError> {
            {
                let mut plant = self.plant.lock().unwrap();
                plant.advance(WallInstant::now());
                plant.command = Twist2::ZERO;
            }
            let r = self.inner.revoke_and_zero(id).await;
            log(&self.trace, "fenced", (id, r.is_ok()));
            r
        }
    }
    fn body(p: PlanarPose) -> ([f64; 3], [f64; 4]) {
        let q = Quaternion::from_yaw(p.yaw);
        ([p.x, p.y, 0.5], [q.x, q.y, q.z, q.w])
    }
    fn independent_tail(map: &SurfaceSnapshot, pose: PlanarPose, before: Twist2, command: Twist2,
        limits: &nuwax_nav_types_messages::MotionLimits, step: f64) -> (bool, Option<String>) {
        let first_failure = std::cell::RefCell::new(None);
        let check = |kind: &'static str| {
            let first_failure = &first_failure;
            move |a: PlanarPose, b: PlanarPose, elapsed: f64| {
                let mut witness = None;
                let result = map.validate_body_motion_with_witness(body(a), body(b), &[], &mut witness)
                    .and_then(|_| map.body_query_with_current_support(body(a).0, body(a).1)
                        .and_then(|view| view.validate_body_motion_with_witness(body(a), body(b), &[], &mut witness)));
                if let Err(error) = result {
                    if first_failure.borrow().is_none() {
                        *first_failure.borrow_mut() = Some(format!("callback={kind} elapsed={elapsed} from={a:?} to={b:?} error={error:?} witness={witness:?}"));
                    }
                    return false;
                }
                true
            }
        };
        let passed = nuwax_nav_control::check_stopping_trajectory_with_period_check(pose,before,command,limits,
            DEFAULT_TICK_INTERVAL.as_secs_f64(),step,map.body_envelope().unwrap().radius(),check("geometric"),check("period")).is_some();
        (passed, first_failure.into_inner())
    }
    fn validate_history(plant: &Plant, limits: &nuwax_nav_types_messages::MotionLimits, config: &SurfaceFollowerConfig) -> Vec<String> {
        let mut failures = Vec::new();
        for (index, edge) in plant.edges.iter().enumerate() {
            if let Err(e) = edge.map.validate_body_motion(body(edge.from), body(edge.to), &[]) {
                failures.push(format!("actual edge {index}: {e:?}"));
            }
        }
        for (index, r) in plant.receipts.iter().enumerate() {
            assert!(
                r.ns.checked_sub(r.source_ns)
                    .is_some_and(|age| age <= 1_000_000_000),
                "actual receipt {index} needs a genuinely fresh source"
            );
            let actual = independent_tail(&r.map,r.pose,r.before,r.command,limits,0.025);
            if !actual.0 {
                let guard = &r.proof.proof.guard;
                let values = guard.pose.bits.map(f64::from_bits);
                let guard_pose = PoseStamped { frame_id:guard.pose.frame_id.clone(),stamp:guard.pose.stamp,
                    pose:Pose3 { position:Point3::new(values[0],values[1],values[2]),
                        orientation:Quaternion {x:values[3],y:values[4],z:values[5],w:values[6]} } };
                let guard_planar = PlanarPose {x:values[0],y:values[1],yaw:guard_pose.pose.orientation.yaw()};
                let applied = guard.applied_bits.map(f64::from_bits);
                let guard_before = Twist2 {linear:nuwax_nav_types_messages::Point2::new(applied[0],applied[1]),angular_z:applied[2]};
                let historical = independent_tail(&r.map,guard_planar,guard_before,r.command,limits,0.025);
                let exact_step = f64::from(r.map.source_voxel_size_m())*0.5;
                let exact_guard = independent_tail(&r.map,guard_planar,guard_before,r.command,limits,exact_step);
                let exact_actual = independent_tail(&r.map,r.pose,r.before,r.command,limits,exact_step);
                eprintln!("SCHEDULER_EXACT_STEP_REPLAY index={index} step={exact_step} guard={exact_guard:?} actual={exact_actual:?}");
                let mut actual_pose = guard_pose.clone();
                actual_pose.pose.position = Point3::new(r.pose.x,r.pose.y,0.5);
                actual_pose.pose.orientation = Quaternion::from_yaw(r.pose.yaw);
                // The callbacks use geometric pose only. Preserve the captured
                // stamp; this diagnostic replay does not issue motion authority.
                let production_guard = SurfaceFollower::replay_stopping_trajectory_for_test(config,r.map.clone(),guard_pose.clone(),guard_before,r.command,&[]);
                let production_actual = SurfaceFollower::replay_stopping_trajectory_for_test(config,r.map.clone(),actual_pose,r.before,r.command,&[]);
                eprintln!("SCHEDULER_PRODUCTION_TAIL_REPLAY index={index} guard={production_guard:?} actual={production_actual:?}");
                eprintln!("SCHEDULER_TAIL_REPLAY index={index} actual={actual:?} captured_guard={historical:?} guard_pose={guard_pose:?} actual_pose={:?} guard_before={guard_before:?} actual_before={:?}",r.pose,r.before);
                failures.push(format!("actual receipt {index} result={} rejected by complete independent braking-tail oracle pose={:?} before={:?} command={:?}",r.id,r.pose,r.before,r.command));
            }
        }
        failures
    }
    fn obstacle_update(
        previous: &Arc<SurfaceSnapshot>,
        sequence: u64,
        x: f64,
    ) -> (Arc<SurfaceSnapshot>, u64) {
        let ns = now_ns();
        (
            Arc::new(
                previous
                    .with_recent_obstacle_points(
                        sequence,
                        ns,
                        0,
                        &[],
                        (0..5).map(|i| (i, [x, 1.3 + f64::from(i) * 0.1, 0.5])),
                    )
                    .unwrap(),
            ),
            ns,
        )
    }
    async fn publish(
        handle: &NavSchedulerHandle,
        plant: &Arc<Mutex<Plant>>,
        previous: Arc<SurfaceSnapshot>,
        next: Arc<SurfaceSnapshot>,
        ns: u64,
    ) {
        assert!(nuwax_nav_pct::same_source_geometry(&previous, &next));
        {
            let mut p = plant.lock().unwrap();
            p.advance(WallInstant::now());
            p.map = next.clone();
            p.source_ns = ns;
        }
        handle
            .update_obstacle_source(
                Some(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {planar_reference:None,
                    snapshot: next,
                    change_basis: Some(previous),
                    changes: Arc::new(SurfaceGeometryChanges::Empty),
                })),
                true,
                Some(CommandDispatchBound::from_source_timestamp(
                    ns,
                    Duration::from_secs(1),
                )),
            )
            .await
            .unwrap();
    }

    #[tokio::test]
    #[ignore = "real-scheduler persistent-obstacle recovery and actual motion oracle; run in isolation"]
    async fn real_scheduler_preserves_detour_and_reaches_original_goal() {
        run(RecoveryCase::Persistent, false).await;
    }
    #[tokio::test]
    #[ignore = "causal attribution only, stops at second rejection; not goal qualification"]
    async fn real_scheduler_old_admission_turning_room_attribution() {
        run(RecoveryCase::Persistent, true).await;
    }
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum RecoveryCase { Persistent, ClearsWhileHeld, CancelThenNewGoal }
    #[tokio::test]
    #[ignore = "real scheduler moving recovery after genuinely clearing obstacle; run in isolation"]
    async fn real_scheduler_cleared_obstacle_recovers_original_goal() {
        run(RecoveryCase::ClearsWhileHeld, false).await;
    }
    #[tokio::test]
    #[ignore = "real scheduler held-work cancellation and replacement goal arrival; run in isolation"]
    async fn real_scheduler_cancelled_held_detour_cannot_move_replacement_goal() {
        run(RecoveryCase::CancelThenNewGoal, false).await;
    }
    async fn run(case: RecoveryCase, attribution_only: bool) {
        let (config, base, keys) = crate::route_moving_worker_tests::fixture(true, 0.05, 6.0);
        assert!(!config.controller.simulation);
        let limits = config.motion_limits.clone().unwrap();
        let sample_ns = now_ns();
        let mut observations = SurfaceObservations::new(base.source_voxel_size_m(), 32).unwrap();
        let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
        for &key in keys.iter() {
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: sample_ns,
                        hits: 1,
                    },
                );
        }
        for (key, block) in blocks {
            observations.replace_block(key, block).unwrap();
        }
        observations.mark_complete_geometry_source();
        let clear = Arc::new(base.with_observations(Some(Arc::new(observations))));
        let start = pose(0.5, 1.5, 0.0);
        let mut goal = pose(3.0, 1.5, 0.0);
        let original_goal = goal.clone();
        let mut session_id = "goal-1";
        let trace: Trace = Default::default();
        let _dump = TraceDump(trace.clone());
        let stops: Stops = Default::default();
        let grant: Grant = Default::default();
        let state: State = Default::default();
        let clock = WallInstant::now();
        let plant = Arc::new(Mutex::new(Plant {
            pose: PlanarPose {
                x: 0.5,
                y: 1.5,
                yaw: 0.0,
            },
            command: Twist2::ZERO,
            last: clock,
            deadline: clock,
            map: clear.clone(),
            source_ns: sample_ns,
            receipts: Vec::new(),
            edges: Vec::new(),
        }));
        let gate_open = Arc::new(AtomicBool::new(false));
        let gate = gate_open.clone();
        let events = trace.clone();
        let (entered_tx, entered) = mpsc::channel();
        let (release, held) = mpsc::channel();
        let follower = RouteFollower::new_with_planning_hook(
            config.clone(),
            Box::new(move |r| {
                log(&events, "plan_started", (r.generation, r.detour, &r.pose));
                if r.detour && !gate.load(Ordering::Acquire) {
                    entered_tx.send(()).unwrap();
                    let _ = held.recv_timeout(Duration::from_secs(5));
                    log(&events, "plan_hook_released", (r.generation, r.detour));
                }
            }),
        )
        .unwrap();
        let worker = planner_worker::PlannerWorker::new(Probe {
            inner: follower,
            trace: trace.clone(),
            stops: stops.clone(),
            state: state.clone(),
            pose: None,
            map: None,
            applied: Twist2::ZERO,
        })
        .unwrap();
        let motion = Motion {
            inner: FakeMotion(Default::default()),
            trace: trace.clone(),
            plant: plant.clone(),
            grant: grant.clone(),
        };
        let (handle, task) = spawn_scheduler(
            NavScheduler::new(
                motion,
                DeliveryProbe {
                    inner: worker,
                    grant,
                },
                SchedulerConfig::default(),
            )
            .unwrap(),
        );
        handle
            .update_odometry_with_deadline(start.clone(), Instant::now() + Duration::from_secs(1))
            .await
            .unwrap();
        handle
            .update_costmap(CostmapObservation {
                costmap: ExecutionMap::Surface(clear.clone()),
                source_timestamp_ns: sample_ns as i64,
            })
            .await
            .unwrap();
        let mut session = session();
        session.goal_pose = goal.clone();
        let options = session.options;
        handle.start_session(session).await.unwrap();
        let mut route = path();
        route.path = ExecutionPath::Surface(Path3D {
            poses: vec![start.clone(), goal.clone()],
        });
        handle.submit_path(route).await.unwrap();
        let began = WallInstant::now();
        let mut next_pose = began;
        let mut next_source = began;
        let mut map = clear;
        let mut obstacle_x = None;
        let mut sequence = 0;
        let mut hold_started = None;
        let mut released = false;
        let mut started_detour = false;
        let mut stops_at_hold = 0;
        let mut held_sources = 0;
        let mut held_receipts = 0;
        let mut held_nonzero = 0;
        let mut release_position = None;
        let mut pending_reason = false;
        let mut failure = None;
        let mut arrived = false;
        let mut attribution_rejection = false;
        let mut held_command_results = 0;
        let mut held_lifecycle = None;
        let mut obstacle_cleared = false;
        let mut clear_source_ns = None;
        let mut replaced = false;
        let mut replacement_receipt_start = None;
        let mut old_worker_basis = None;
        let mut replacement_admission_start = None;
        loop {
            let now = WallInstant::now();
            if now.duration_since(began) >= Duration::from_secs(60) {
                failure = Some(format!(
                    "original 60-second full-goal window exceeded: {:?}",
                    handle.status()
                ));
                break;
            }
            if now >= next_pose {
                let p = plant.lock().unwrap().measured_pose();
                handle
                    .update_odometry_with_deadline(p, Instant::now() + Duration::from_secs(1))
                    .await
                    .unwrap();
                next_pose = now + Duration::from_millis(25);
            }
            if obstacle_x.is_none()
                && handle
                    .status()
                    .last_applied_twist
                    .is_some_and(|c| c.linear.x >= 0.4)
            {
                let p = plant.lock().unwrap().measured_pose();
                let x = 0.8907157909999999; // exact obstacle from the preserved failed v4 run
                sequence += 1;
                let (blocked, ns) = obstacle_update(&map, sequence, x);
                let b = crate::route_planning::body;
                blocked
                    .validate_body_pose(b(&p).0, b(&p).1)
                    .expect("current stance remains legal");
                assert!(
                    blocked.validate_body_motion(b(&p), b(&goal), &[]).is_err(),
                    "actual forward connection is genuinely blocked"
                );
                let witness = [
                    p.clone(),
                    pose(p.pose.position.x, 1.5, std::f64::consts::FRAC_PI_2),
                    pose(p.pose.position.x, 2.2, std::f64::consts::FRAC_PI_2),
                    pose(p.pose.position.x, 2.2, 0.0),
                    pose(3.0, 2.2, 0.0),
                    pose(3.0, 2.2, -std::f64::consts::FRAC_PI_2),
                    pose(3.0, 1.5, -std::f64::consts::FRAC_PI_2),
                    goal.clone(),
                ];
                for (index, pair) in witness.windows(2).enumerate() {
                    blocked
                        .validate_body_motion(b(&pair[0]), b(&pair[1]), &[])
                        .unwrap_or_else(|e| {
                            panic!("independent full-body detour witness edge {index}: {e:?}")
                        });
                }
                log(&trace, "obstacle_published", (x, ns, &p));
                publish(&handle, &plant, map, blocked.clone(), ns).await;
                map = blocked;
                obstacle_x = Some(x);
                next_source = now + Duration::from_millis(200);
            } else if now >= next_source {
                // Every actual new acquisition retains the same physical obstacle
                // after onset; no clear/static refresh can erase the held problem.
                sequence += 1;
                let (next, ns) = if let Some(x) = obstacle_x.filter(|_| !obstacle_cleared) {
                    obstacle_update(&map, sequence, x)
                } else {
                    let ns = now_ns();
                    (
                        Arc::new(
                            map.with_recent_obstacle_points(
                                sequence,
                                ns,
                                0,
                                &[],
                                std::iter::empty(),
                            )
                            .unwrap(),
                        ),
                        ns,
                    )
                };
                publish(&handle, &plant, map, next.clone(), ns).await;
                map = next;
                next_source = now + Duration::from_millis(200);
                if hold_started.is_some() && !released {
                    held_sources += 1;
                }
            }
            if !started_detour && entered.try_recv().is_ok() {
                started_detour = true;
            }
            let mut status = handle.status();
            if started_detour
                && !released
                && status.last_applied_twist == Some(Twist2::ZERO)
                && status.message.contains("当前轨迹受阻")
            {
                pending_reason = true;
                if hold_started.is_none() {
                    hold_started = Some(now);
                    stops_at_hold = stops.lock().unwrap().len();
                    held_lifecycle = state.lock().unwrap().latest;
                    held_receipts = plant.lock().unwrap().receipts.len();
                    held_command_results = state.lock().unwrap().command_results;
                    log(&trace, "recovery_zero_wait", &status.message);
                }
            }
            if let Some(started) = hold_started {
                if !released {
                    assert_eq!(
                        status.phase,
                        SchedulerPhase::Controlling,
                        "local recovery must not request an external path"
                    );
                    if !replaced {
                        assert_eq!(stops.lock().unwrap().len(), stops_at_hold,
                            "local wait must not cancel the planning owner");
                    }
                    let current = state.lock().unwrap().latest.unwrap();
                    let held = held_lifecycle.unwrap();
                    if !replaced {
                        assert_eq!((current.0, current.1), (held.0, held.1),
                            "source updates cannot retire the held owner or route generation");
                    }
                    if !attribution_only {
                        assert_eq!(state.lock().unwrap().command_results, held_command_results,
                            "withdrawn old admission must not produce Command(ZERO) or nonzero and reset the pending wait budget");
                        assert_eq!(plant.lock().unwrap().receipts.len(), held_receipts,
                            "withdrawn old admission must retain confirmed ZERO until re-admission");
                    }
                    if case == RecoveryCase::ClearsWhileHeld && !obstacle_cleared
                        && now.duration_since(started) >= Duration::from_millis(250) {
                        sequence += 1;
                        let ns = now_ns();
                        let cleared = Arc::new(map.with_recent_obstacle_points(sequence,ns,0,&[],std::iter::empty()).unwrap());
                        let current_pose = plant.lock().unwrap().measured_pose();
                        cleared.validate_body_motion(crate::route_planning::body(&current_pose),
                            crate::route_planning::body(&goal),&[]).expect("genuine clearing acquisition restores the direct full-body route");
                        log(&trace,"obstacle_cleared_while_held",(ns,&current_pose));
                        publish(&handle,&plant,map,cleared.clone(),ns).await;
                        map=cleared;obstacle_cleared=true;clear_source_ns=Some(ns);
                        next_source=now+Duration::from_millis(200);
                        held_sources+=1;
                    }
                    if case == RecoveryCase::CancelThenNewGoal && !replaced
                        && now.duration_since(started) >= Duration::from_millis(300) {
                        let before = plant.lock().unwrap().receipts.last().unwrap().proof.proof.input.clone();
                        handle.cancel(session_id).await.unwrap();
                        let cancelled=handle.status();
                        assert_eq!(cancelled.last_terminal,Some(TerminalOutcome::Cancelled));
                        assert!(cancelled.last_zero_ack_id.is_some(),"cancel must confirm the real fixture fence");
                        assert_eq!(cancelled.last_applied_twist,Some(Twist2::ZERO));
                        let current_pose={let mut p=plant.lock().unwrap();
                            let measured=p.measured_pose();assert_eq!(p.command,Twist2::ZERO);
                            replacement_receipt_start=Some(p.receipts.len());measured};
                        replacement_admission_start=Some(state.lock().unwrap().admissions.len());
                        old_worker_basis=Some((before.epoch,before.goal_version,before.path_version));
                        goal=pose(3.0,2.3,0.0);
                        let witness=[current_pose.clone(),pose(current_pose.pose.position.x,1.5,std::f64::consts::FRAC_PI_2),
                            pose(current_pose.pose.position.x,2.3,std::f64::consts::FRAC_PI_2),
                            pose(current_pose.pose.position.x,2.3,0.0),goal.clone()];
                        for pair in witness.windows(2) {map.validate_body_motion(crate::route_planning::body(&pair[0]),
                            crate::route_planning::body(&pair[1]),&[]).expect("replacement goal has independent full-body feasible route");}
                        session_id="goal-2";
                        let mut next_session=session_for(session_id);next_session.goal_pose=goal.clone();
                        assert_eq!(next_session.options,options,"replacement retains original tolerances");
                        handle.start_session(next_session).await.unwrap();
                        let mut next_route=path_for(session_id,1);
                        next_route.path=ExecutionPath::Surface(Path3D{poses:vec![current_pose.clone(),goal.clone()]});
                        handle.submit_path(next_route).await.unwrap();
                        replaced=true;status=handle.status();
                        log(&trace,"cancel_fenced_then_new_goal_while_held",(old_worker_basis,&current_pose,&goal,&status));
                    }
                    if now.duration_since(started) >= Duration::from_millis(650) {
                        assert!(
                            held_sources >= 3,
                            "hold crosses genuine persistent-obstacle acquisitions"
                        );
                        {
                            let p = plant.lock().unwrap();
                            held_nonzero = p.receipts.len() - held_receipts;
                            release_position = Some(p.pose);
                        }
                        gate_open.store(true, Ordering::Release);
                        release.send(()).unwrap();
                        released = true;
                    }
                }
            }
            if attribution_only && released && state.lock().unwrap().rejection_after_detour {
                attribution_rejection = true;
                break;
            }
            if let Some(outcome) = status.last_terminal {
                arrived = outcome == TerminalOutcome::Succeeded;
                if !arrived {
                    failure = Some(format!("recovery failed: {outcome:?}: {}", status.message));
                }
                break;
            }
            assert_ne!(
                status.phase,
                SchedulerPhase::AwaitingPath,
                "local recovery must retain the original route"
            );
            assert_eq!(
                status.path_revision, 1,
                "no external path replacement was submitted"
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        if !arrived && handle.status().session_id.is_some() {
            handle.cancel(session_id).await.unwrap();
        }
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
        let mut p = plant.lock().unwrap();
        p.advance(WallInstant::now());
        let audit = state.lock().unwrap();
        for (index, admission) in audit.admissions.iter().enumerate() {
            eprintln!(
                "SCHEDULER_ADMISSION index={index} lifecycle={:?} current_pose={:?} applied={:?} path={:?}",
                admission.lifecycle, admission.pose, admission.applied, admission.path
            );
        }
        for (index, r) in p.receipts.iter().enumerate() {
            eprintln!("SCHEDULER_RECEIPT index={index} ns={} source_ns={} id={} operation={} pose={:?} before={:?} command={:?} proof={:?}",r.ns,r.source_ns,r.id,r.operation_id,r.pose,r.before,r.command,r.proof);
        }
        for (index, e) in p.edges.iter().enumerate() {
            eprintln!(
                "SCHEDULER_EDGE index={index} from={:?} to={:?}",
                e.from, e.to
            );
        }
        // Always audit physical receipt state and full braking tails before a
        // goal-window failure is reported. A timeout cannot hide unsafe motion.
        let safety_failures = validate_history(&p, &limits, &config);
        eprintln!("SCHEDULER_SAFETY_AUDIT receipts={} actual_edges={} failures={:?}",
            p.receipts.len(), p.edges.len(), safety_failures);
        assert!(
            pending_reason && released,
            "must exercise the held local recovery"
        );
        assert!(
            audit.admissions.len() >= 2,
            "a new local detour must actually be admitted"
        );
        if attribution_only {
            assert!(
                attribution_rejection,
                "diagnostic must reach a new rejection after detour admission"
            );
            let admission = &audit.admissions[1];
            let first = admission.path.poses.first().unwrap();
            let turn = admission
                .path
                .poses
                .iter()
                .take_while(|point| point.pose.position.distance(first.pose.position) < 1e-9)
                .last()
                .expect("candidate's initial turn");
            let first_body = crate::route_planning::body(first);
            let target_body = crate::route_planning::body(turn);
            let planned = admission
                .map
                .validate_body_motion(first_body, target_body, &[]);
            let actual =
                p.map
                    .validate_body_motion(body(p.pose), (body(p.pose).0, target_body.1), &[]);
            eprintln!("SCHEDULER_TURN_ATTRIBUTION candidate_start={:?} candidate_turn={:?} admission_pose={:?} actual_pose={:?} held_fresh_nonzero={} original_turn={:?} same_turn_at_actual={:?} max_reverse_mps={}",
                first.pose,turn.pose,admission.pose.pose,p.pose,held_nonzero,planned,actual,limits.max_reverse_mps);
            assert!(
                planned.is_ok(),
                "the candidate's captured turn must be independently feasible"
            );
            assert!(
                actual.is_err(),
                "the same turn from actual later pose must demonstrate lost room"
            );
            assert_eq!(limits.max_reverse_mps, 0.0);
            assert!(safety_failures.is_empty(), "independent motion audit failed: {safety_failures:?}");
            return;
        }
        if case == RecoveryCase::ClearsWhileHeld {
            assert!(obstacle_cleared,"clear scenario must publish an actual clearing acquisition while held");
            assert!(p.receipts.iter().filter(|r|r.ns>=clear_source_ns.unwrap()).all(|r|r.source_ns>=clear_source_ns.unwrap()),
                "post-clear receipts must use current genuinely acquired evidence");
        }
        if case == RecoveryCase::CancelThenNewGoal {
            assert!(replaced,"cancellation and new goal must occur before held work releases");
            let old=old_worker_basis.unwrap();
            let after=&p.receipts[replacement_receipt_start.unwrap()..];
            assert!(!after.is_empty(),"new goal must execute real nonzero receipts");
            for receipt in after {
                let proof=&receipt.proof.proof.input;
                assert!(receipt.operation_id.starts_with("nav:goal-2:twist:"),"obsolete session cannot issue a receipt");
                assert!(proof.epoch>old.0 && proof.goal_version>old.1 && proof.path_version>old.2,
                    "old goal/epoch/path result cannot move under replacement identity");
            }
            let held=held_lifecycle.unwrap();
            for admission in &audit.admissions[replacement_admission_start.unwrap()..] {
                assert!(admission.lifecycle.0>held.0 && admission.lifecycle.1>held.1,
                    "released obsolete candidate cannot be admitted under old owner/generation");
                let end=admission.path.poses.last().expect("replacement candidate endpoint").pose.position;
                assert!(end.distance(goal.pose.position)<options.position_tolerance_m,
                    "this short full-horizon replacement candidate must target goal-2, never the released old goal");
            }
            assert!(stops.lock().unwrap().len()>stops_at_hold,"explicit user cancellation still stops the owner");
        }
        assert!(safety_failures.is_empty(), "independent motion audit failed: {safety_failures:?}");
        assert!(failure.is_none(), "{}", failure.unwrap_or_default());
        assert!(arrived, "the scenario's active goal must be completed");
        let at_release = release_position.unwrap();
        assert!(
            (p.pose.x - at_release.x).hypot(p.pose.y - at_release.y) > 1.0,
            "actual post-recovery displacement required"
        );
        assert!(
            (p.pose.x - goal.pose.position.x).hypot(p.pose.y - goal.pose.position.y)
                < options.position_tolerance_m
        );
        assert!(
            nuwax_nav_astar::controllers::angle_diff(p.pose.yaw, goal.pose.orientation.yaw()).abs()
                < options.yaw_tolerance_rad
        );
        eprintln!("SCHEDULER_RECOVERY_RESULT case={case:?} session={session_id} elapsed_s={} source_acquisitions={} held_sources={} held_fresh_nonzero={} receipts={} actual_edges={} final_pose={:?} original_goal={:?} active_goal={:?}",
            began.elapsed().as_secs_f64(),sequence,held_sources,held_nonzero,p.receipts.len(),p.edges.len(),p.pose,original_goal.pose,goal.pose);
    }
}
