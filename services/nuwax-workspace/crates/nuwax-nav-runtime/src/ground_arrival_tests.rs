// D1 D07-D12. Include directly in lib.rs `mod tests` to retain the six exact
// selectors. Qualification history and source snapshots are retained outside the repository.
// Real RouteFollower -> PlanningOwner/Scan -> PlannerWorker -> NavScheduler.
// FakeMotion is only the existing protocol peer. No synthetic planner result,
// relation, winner, terminal event, source identity, or arrival evidence.
pub(crate) mod ground_arrival_qualification {
    use super::*;
    use crate::route_planning::priority_probe as probe;
    use crate::replan_feedback::{GroundIntent, SurfaceReplanInput};
    use crate::surface_follower::ground_owner_tests as fixed;
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    use nuwax_nav_pct::{PctPlanner, SearchLimits};
    use nuwax_nav_pct::ground_route::{GroundRouteStart, GroundRouteGoal, PlannedGroundRoute};
    use nuwax_nav_scan::{GroundCandidateBinding, CandidateEdgeOrigin};
    use nuwax_nav_types_messages::{GoalOptions, Point3, Pose3, Quaternion, Timestamp};
    use std::collections::BTreeMap;
    use std::sync::atomic::{AtomicBool, Ordering};

    pub(super) type Scheduler<M> = NavScheduler<M, planner_worker::PlannerWorker>;
    type Blocks = BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>;
    #[derive(Clone)]
    pub(crate) struct Snapshot {
        pub path: Path3D,
        pub binding: Arc<GroundCandidateBinding>,
        pub cursor: Option<(usize, f64)>,
        pub actual: Option<Arc<ActualExecutionContext>>,
    }
    #[derive(Clone)]
    pub(crate) struct StopSnapshot {
        pub before: Option<(usize, f64)>,
        pub after: Option<(usize, f64)>,
        pub events_before: usize,
    }
    #[derive(Clone, Default)]
    pub(crate) struct Audit {
        pub requests: Vec<(u64, Option<(u64, u64)>, bool)>,
        pub admissions: Vec<probe::Admission>,
        pub final_yaws: Vec<Option<f64>>,
        pub snapshots: Vec<Snapshot>,
        pub events: Vec<Arc<GroundArrivalEvidence>>,
        pub body_successes: usize,
        pub commands: Vec<Twist2>,
        pub stops: Vec<StopSnapshot>,
        pub overflow: bool,
    }
    // Only delegates and reads observations. Even the event is retained only
    // after the sole production owner has emitted it on its real worker thread.
    struct ObservedRoute {
        inner: RouteFollower,
        audit: Arc<Mutex<Audit>>,
        dropped: Arc<AtomicBool>,
        applied: Twist2,
    }
    impl NavPlanner for ObservedRoute {
        fn configure_goal(&mut self, goal: &PoseStamped, options: GoalOptions) -> Result<(), RuntimeError> {
            self.inner.configure_goal(goal, options)
        }
        fn configure_execution_goal(&mut self, goal: &ExecutionGoal, options: GoalOptions,
            initial: Option<GroundInitialCapture>) -> Result<(), RuntimeError> {
            self.inner.configure_execution_goal(goal, options, initial)
        }
        fn consume_initial_actual_for_planning(&mut self) -> Result<(), RuntimeError> {
            self.inner.consume_initial_actual_for_planning()
        }
        fn configure_boundary(&mut self, boundary: &[[f64; 2]]) { self.inner.configure_boundary(boundary) }
        fn set_goal_log_context(&mut self, id: &str) { self.inner.set_goal_log_context(id) }
        fn replace_path(&mut self, path: Path2D) { self.inner.replace_path(path) }
        fn replace_execution_path(&mut self, path: ExecutionPath, yaws: Option<Vec<f64>>, goal: PoseStamped)
            -> Result<(), RuntimeError> { self.inner.replace_execution_path(path, yaws, goal) }
        fn replace_typed_execution_path(&mut self, path: ExecutionPath, yaws: Option<Vec<f64>>, goal: ExecutionGoal)
            -> Result<(), RuntimeError> { self.inner.replace_typed_execution_path(path, yaws, goal) }
        fn update_odometry(&mut self, pose: PoseStamped) { self.inner.update_odometry(pose) }
        fn update_costmap(&mut self, map: CostMap2D) { self.inner.update_costmap(map) }
        fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
            self.inner.update_execution_map(map)
        }
        fn invalidate_execution_map(&mut self) { self.inner.invalidate_execution_map() }
        fn observe_applied_command(&mut self, command: Twist2) {
            self.inner.observe_applied_command(command); self.applied = command;
        }
        fn refresh_execution_context(&mut self, applied: Twist2) -> Option<Arc<ActualExecutionContext>> {
            self.inner.refresh_execution_context(applied)
        }
        fn take_step_execution_context(&mut self, applied: Twist2) -> Option<Arc<ActualExecutionContext>> {
            self.inner.take_step_execution_context(applied)
        }
        fn take_command_dispatch_bound(&mut self) -> Option<CommandDispatchBound> {
            self.inner.take_command_dispatch_bound()
        }
        fn revalidate_selected_command(&mut self, command: Twist2) -> Option<PlannerStep> {
            self.inner.revalidate_selected_command(command)
        }
        fn revalidate_selected_command_at_current_state(&mut self, command: Twist2) -> Option<PlannerStep> {
            self.inner.revalidate_selected_command_at_current_state(command)
        }
        fn execution_wait_reason(&self) -> Option<String> { self.inner.execution_wait_reason() }
        fn local_path_for_display(&self) -> Vec<[f64; 3]> { self.inner.local_path_for_display() }
        fn prepare_control_step(&mut self, width: f64, waiting: bool) -> Option<watch::Receiver<()>> {
            self.inner.prepare_control_step(width, waiting)
        }
        fn step(&mut self, width: f64) -> PlannerStep {
            let dispatch = tracing::Dispatch::new(probe::Subscriber);
            probe::begin_control();
            let result = tracing::dispatcher::with_default(&dispatch, || self.inner.step(width));
            let receipts = probe::end_control();
            let observed = self.inner.local_path().cloned().zip(self.inner.ground_binding_for_test())
                .map(|(path, binding)| Snapshot { path, binding,
                    cursor: self.inner.ground_cursor_for_test(),
                    actual: self.inner.refresh_execution_context(self.applied) });
            let mut audit = self.audit.lock().unwrap();
            for receipt in receipts {
                if audit.admissions.len() < 32 { audit.admissions.push(receipt); } else { audit.overflow = true; }
            }
            if let Some(observed) = observed {
                if audit.snapshots.len() < 128 { audit.snapshots.push(observed); } else { audit.overflow = true; }
            }
            match &result {
                PlannerStep::GroundArrival(event) => {
                    if audit.events.len() < 8 { audit.events.push(event.clone()); } else { audit.overflow = true; }
                }
                PlannerStep::Succeeded => audit.body_successes += 1,
                PlannerStep::Command(command) => {
                    if audit.commands.len() < 128 { audit.commands.push(*command); } else { audit.overflow = true; }
                }
                _ => {}
            }
            result
        }
        fn stop(&mut self) {
            let before = self.inner.ground_cursor_for_test();
            self.inner.stop();
            let after = self.inner.ground_cursor_for_test();
            let mut audit = self.audit.lock().unwrap();
            let events_before = audit.events.len();
            audit.stops.push(StopSnapshot { before, after, events_before });
        }
    }
    impl Drop for ObservedRoute {
        fn drop(&mut self) { self.dropped.store(true, Ordering::Release); }
    }
    // Shared by D01 at the real handle ingress. All mutable planner ownership
    // remains in PlannerWorker; callers receive read-only qualification records.
    pub(crate) fn observed_worker(config: SurfaceFollowerConfig)
        -> (planner_worker::PlannerWorker, Arc<Mutex<Audit>>, Arc<AtomicBool>) {
        let audit = Arc::new(Mutex::new(Audit::default()));
        let requests = audit.clone();
        let follower = RouteFollower::new_with_planning_hook(config, Box::new(move |request| {
            let mut audit = requests.lock().unwrap();
            if audit.requests.len() < 32 {
                audit.requests.push((request.generation, crate::planning_owner::current_request_for_test(),
                request.ground.is_some()));
                audit.final_yaws.push(request.final_yaw);
            } else { audit.overflow = true; }
        })).unwrap();
        let dropped = Arc::new(AtomicBool::new(false));
        let worker = planner_worker::PlannerWorker::new(ObservedRoute {
            inner: follower, audit: audit.clone(), dropped: dropped.clone(), applied: Twist2::ZERO,
        }).unwrap();
        (worker, audit, dropped)
    }
    pub(super) struct Case {
        pub source: Arc<SurfaceSnapshot>,
        pub config: SurfaceFollowerConfig,
        pub actual: PoseStamped,
        pub initial: GroundInitialCapture,
        pub goal: ExecutionGoal,
        pub options: GoalOptions,
        pub route: PlannedGroundRoute,
        pub source_bound: Option<CommandDispatchBound>,
        pub acquired_at: Instant,
    }
    pub(super) fn now_pose(mut pose: PoseStamped) -> PoseStamped {
        let ns = crate::command_dispatch::unix_ns().unwrap();
        pose.stamp = Timestamp::from_unix((ns / 1_000_000_000) as i64, (ns % 1_000_000_000) as u32);
        pose
    }
    pub(super) fn pose(body: ([f64; 3], [f64; 4])) -> PoseStamped { now_pose(fixed::c3_stamped(body)) }
    pub(super) fn make_case(source: Arc<SurfaceSnapshot>, config: SurfaceFollowerConfig,
        actual: PoseStamped, initial_height: Option<f64>, goal_xy: [f64; 2],
        goal_yaw: f64, requested: Option<f64>, acquired_at: Instant,
        source_bound: Option<CommandDispatchBound>) -> Case {
        let xy = [actual.pose.position.x, actual.pose.position.y];
        let prepared = config.pct.prepare_ground_start(&source,
            GroundRouteStart { xy, ground_z: initial_height }, None).unwrap();
        // Capture A before any asynchronous planning. This temporary actual
        // observer uses the original physical query; it does not own navigation.
        let mut observer = SurfaceFollower::new(config.clone()).unwrap();
        observer.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
        observer.update_odometry(actual.clone());
        let context = observer.refresh_execution_context(Twist2::ZERO).unwrap();
        assert!(context.navigation.is_none());
        let initial = GroundInitialCapture::from_actual(prepared.basis().clone(), context).unwrap();
        let options = GoalOptions { position_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, allow_reverse: false };
        let goal = ExecutionGoal::Ground(GroundExecutionGoal {
            frame_id: actual.frame_id.clone(), xy: goal_xy, yaw: goal_yaw, requested_ground_z: requested,
        });
        let route = config.pct.ground_route_from_start(&source, crate::route_planning::body(&actual), &prepared,
            GroundRouteGoal { xy: goal_xy, requested_ground_z: requested }, 0.0, &[], None, None,
            SearchLimits::default(), &[]).expect("original fixed PCT prerequisite");
        assert_eq!(route.reference.original_goal().requested_ground_z, requested);
        Case { source, config, actual, initial, goal, options, route, source_bound, acquired_at }
    }
    pub(super) fn fixed_case(on: bool, actual: ([f64; 3], [f64; 4]), height: Option<f64>,
        goal: [f64; 2], yaw: f64, requested: Option<f64>) -> (Case, Blocks) {
        let (source, blocks) = fixed::c3_direct_source();
        let mut config = fixed::c3_config();
        config.local_planning_enabled = on;
        let acquired = Instant::now();
        (make_case(Arc::new(source), config, pose(actual), height, goal, yaw, requested, acquired, None), blocks)
    }
    pub(super) fn live_moving_case() -> Case {
        let (config, base, keys) = crate::route_moving_worker_tests::fixture(true, 0.05, 6.0);
        assert!(config.local_planning_enabled && !config.controller.simulation);
        assert_eq!(config.max_observation_age_sec, Some(1.0));
        assert_eq!(config.scan.search_expansion_limit, nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT);
        let acquired_at = Instant::now();
        let sample_ns = crate::command_dispatch::unix_ns().unwrap();
        let mut observations = SurfaceObservations::new(base.source_voxel_size_m(), 32).unwrap();
        let mut blocks = Blocks::new();
        for &key in keys.iter() {
            blocks.entry(key.map(|v| v.div_euclid(32))).or_default()
                .insert(key, SurfaceObservation { observed_ns: sample_ns, hits: 1 });
        }
        for (block, values) in blocks { observations.replace_block(block, values).unwrap(); }
        observations.mark_complete_geometry_source();
        for &key in keys.iter() { assert_eq!(observations.get(key).unwrap().observed_ns, sample_ns); }
        let source = Arc::new(base.with_observations(Some(Arc::new(observations))));
        let actual = PoseStamped { frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix((sample_ns / 1_000_000_000) as i64, (sample_ns % 1_000_000_000) as u32),
            pose: Pose3 { position: Point3::new(0.5, 1.0, 0.5), orientation: Quaternion::IDENTITY } };
        let bound = CommandDispatchBound::from_source_timestamp(sample_ns, Duration::from_secs(1));
        make_case(source, config, actual, Some(0.1), [0.6, 1.0], 0.0, Some(0.1), acquired_at, Some(bound))
    }
    pub(super) fn ground_path(route: &PlannedGroundRoute, stamp_like: &PoseStamped, revision: u64) -> VersionedPath {
        let poses = route.guide_body_poses.iter().map(|body| {
            let mut pose = stamp_like.clone();
            pose.pose.position = Point3::new(body.0[0], body.0[1], body.0[2]);
            pose.pose.orientation = Quaternion { x: body.1[0], y: body.1[1], z: body.1[2], w: body.1[3] };
            pose
        }).collect();
        let mut versioned = path_for("goal-1", revision);
        versioned.path = ExecutionPath::Ground(GroundExecutionPath { reference: route.reference.clone(),
            body_guidance: Path3D { poses } });
        versioned
    }
    pub(super) struct Harness<M: NavMotionPort> {
        pub scheduler: Option<Scheduler<M>>,
        pub status: watch::Sender<SchedulerStatus>,
        pub audit: Arc<Mutex<Audit>>,
        pub dropped: Arc<AtomicBool>,
        pub motion: Arc<Mutex<FakeMotionState>>,
        pub case: Case,
        pub request: NavSession,
        pub deadline: Instant,
        next_renew: Instant,
    }
    impl<M: NavMotionPort> Drop for Harness<M> {
        fn drop(&mut self) { if let Some(scheduler) = &mut self.scheduler { scheduler.planner.stop(); } }
    }
    impl<M: NavMotionPort> Harness<M> {
        pub fn s(&mut self) -> &mut Scheduler<M> { self.scheduler.as_mut().unwrap() }
        pub async fn new(case: Case, port: M, motion: Arc<Mutex<FakeMotionState>>) -> Self {
            Self::new_with_mode(case, port, motion, false).await
        }
        pub async fn new_body(case: Case, port: M, motion: Arc<Mutex<FakeMotionState>>) -> Self {
            Self::new_with_mode(case, port, motion, true).await
        }
        async fn new_with_mode(case: Case, port: M, motion: Arc<Mutex<FakeMotionState>>, body: bool) -> Self {
            let (worker, audit, dropped) = observed_worker(case.config.clone());
            let mut scheduler = NavScheduler::new(port, worker, SchedulerConfig::default()).unwrap();
            let (status, _) = watch::channel(scheduler.status.clone());
            let deadline = case.source_bound.and_then(|bound| bound.monotonic_deadline()).map(Instant::from_std)
                .unwrap_or_else(|| Instant::now() + Duration::from_secs(3));
            scheduler.observe_odometry_at(&status, case.actual.clone(),
                case.source_bound.map(|_| case.acquired_at + Duration::from_secs(1)), case.acquired_at).unwrap();
            scheduler.observe_costmap(&status, CostmapObservation { costmap: ExecutionMap::Surface(case.source.clone()),
                source_timestamp_ns: case.actual.stamp.nanos }, Instant::now()).unwrap();
            let (reply, response) = oneshot::channel();
            scheduler.handle_input(&status, InputCommand {
                kind: InputKind::UpdateObstacleSource(None, true, case.source_bound), reply,
            }).await;
            response.await.unwrap().unwrap();
            let mut request = session();
            request.goal = if body { ExecutionGoal::Body(case.actual.clone()) } else { case.goal.clone() };
            request.ground_initial = (!body).then(|| case.initial.clone());
            request.options = case.options;
            scheduler.start_session(&status, request.clone()).await.unwrap();
            let versioned = if body {
                let actual = crate::route_planning::body(&case.actual);
                let route = case.config.pct.body_route(&case.source, actual, actual, case.options, &[], None).unwrap();
                assert_eq!(route, vec![actual]);
                let mut versioned = path();
                versioned.path = ExecutionPath::Surface(Path3D { poses: vec![case.actual.clone()] });
                versioned
            } else { ground_path(&case.route, &case.actual, 1) };
            scheduler.submit_path(&status, versioned).await.unwrap();
            let next_renew = Instant::now() + scheduler.config.renew_interval;
            Self { scheduler: Some(scheduler), status, audit, dropped, motion, case, request, deadline, next_renew }
        }
        pub async fn tick(&mut self) {
            assert!(Instant::now() < self.deadline, "real owner did not publish within original qualification/input lifetime");
            let status = self.status.clone();
            if Instant::now() >= self.next_renew && self.s().should_renew() {
                self.s().renew().await.unwrap();
                self.next_renew = Instant::now() + self.s().config.renew_interval;
            }
            self.s().session_tick(&status).await.unwrap();
        }
        pub async fn wait_admitted(&mut self) -> Snapshot {
            loop {
                if let Some(snapshot) = self.audit.lock().unwrap().snapshots.last().cloned() { return snapshot; }
                assert!(self.s().status.last_terminal.is_none(), "planner failure is not an admission");
                self.tick().await;
                tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
            }
        }
        pub async fn wait_pending(&mut self) {
            while self.s().arrival_confirmation.is_none() {
                assert!(self.s().status.last_terminal.is_none(), "failure is not GroundArrival");
                self.tick().await;
                if self.s().arrival_confirmation.is_none() { tokio::time::sleep(DEFAULT_TICK_INTERVAL).await; }
            }
            let pending = self.s().arrival_confirmation.as_ref().unwrap();
            validate_fence(&pending.proof).unwrap();
            assert_eq!(pending.proof.lease, FenceLeaseState::Idle);
            assert!(matches!(&pending.kind, crate::arrival_confirmation::ArrivalKind::Ground { .. }));
            assert!(!self.s().lease_active);
            assert!(self.s().status.last_terminal.is_none());
            // stop is queued to the real worker; await its consumption rather
            // than reading a zero-valued diagnostic counter as cursor proof.
            while !self.audit.lock().unwrap().stops.iter().any(|stop| stop.events_before > 0) {
                assert!(Instant::now() < self.deadline);
                tokio::task::yield_now().await;
            }
            let stopped = { self.audit.lock().unwrap().stops.iter()
                .find(|stop| stop.events_before > 0).cloned() }.unwrap();
            assert!(stopped.before.is_some(), "real Ground cursor must exist before the arrival stop");
            assert!(stopped.after.is_none(), "the same production stop must clear that cursor");
        }
        pub async fn observe(&mut self, actual: PoseStamped, acquired: Instant) {
            let status = self.status.clone();
            self.s().observe_odometry_at(&status, actual, Some(acquired + Duration::from_secs(1)), acquired).unwrap();
        }
        pub async fn consume_admitted_motion(&mut self, observed: &Snapshot) {
            // Admission alone can still be the original braking ZERO. Keep the
            // fixed input until the real scheduler has applied a nonzero command.
            while self.s().status.last_applied_twist.is_none_or(|twist| twist == Twist2::ZERO) {
                self.tick().await;
                tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
            }
            assert_eq!(observed.path.poses.len(), observed.binding.edges().len() + 1);
            for (edge, pair) in observed.path.poses.windows(2).enumerate() {
                assert_eq!(observed.binding.edges()[edge].motion(),
                    [crate::route_planning::body(&pair[0]), crate::route_planning::body(&pair[1])]);
            }
            for endpoint in observed.path.poses.iter().skip(1) {
                let actual = now_pose(endpoint.clone());
                self.observe(actual.clone(), Instant::now()).await;
                loop {
                    self.tick().await;
                    if self.s().arrival_confirmation.is_some() { break; }
                    let paired = self.audit.lock().unwrap().snapshots.last().and_then(|snapshot| snapshot.actual.as_ref())
                        .is_some_and(|context| crate::replan_feedback::same_pose(&context.odometry, &actual)
                            && context.navigation.is_some());
                    if paired { break; }
                    tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
                }
            }
        }
        pub async fn confirm_fresh(&mut self, expected: TerminalOutcome) {
            let confirmed = self.s().arrival_confirmation.as_ref().unwrap().confirmed_at;
            let previous = self.s().latest_pose.as_ref().unwrap().clone();
            let actual = now_pose(previous.clone());
            let acquired = Instant::now();
            assert!(acquired > confirmed);
            assert!(actual.stamp.nanos > previous.stamp.nanos);
            assert_eq!(actual.pose, previous.pose, "timestamp-only sample must not need a worker pose-version bump");
            self.observe(actual, acquired).await;
            while self.s().status.last_terminal.is_none() {
                self.tick().await;
                if self.s().status.last_terminal.is_none() { tokio::task::yield_now().await; }
            }
            assert_eq!(self.s().status.last_terminal, Some(expected));
            assert!(!self.s().lease_active && self.s().session.is_none());
            assert_eq!(self.s().status.last_applied_twist, Some(Twist2::ZERO));
        }
        pub fn call_count(&self, prefix: &str) -> usize {
            self.motion.lock().unwrap().calls.iter().filter(|call| call.starts_with(prefix)).count()
        }
        pub fn assert_real_owner(&self, require_motion: bool) {
            let audit = { self.audit.lock().unwrap().clone() };
            assert!(!audit.overflow && !audit.events.is_empty());
            assert_eq!(audit.body_successes, 0, "Ground must not erase its evidence into legacy Succeeded");
            for event in &audit.events {
                let actual = event.actual();
                let basis = actual.navigation.as_ref().expect("opaque event retains its consumed actual address");
                assert!(basis.is_terminal_for(event.intent()));
                assert!(event.intent().window().to().is_terminal());
                assert_eq!(basis.walk_state().unwrap().xy(),
                    [actual.odometry.pose.position.x, actual.odometry.pose.position.y]);
                assert_eq!(event.goal(), self.case.goal.as_ground().unwrap());
                assert!(actual.ground_evidence.is_some(), "actual source observation accompanies the event");
            }
            assert!(audit.requests.iter().all(|(_, ticket, ground)| ticket.is_some() && *ground));
            if self.case.config.local_planning_enabled {
                assert!(!audit.requests.is_empty() && !audit.admissions.is_empty());
                for receipt in &audit.admissions {
                    assert!(audit.requests.iter().any(|(_, ticket, _)| ticket.is_some_and(|(id, _)| id == receipt.request_id)));
                }
            }
            if require_motion {
                assert!(audit.commands.iter().any(|command| *command != Twist2::ZERO));
                assert!(audit.snapshots.iter().any(|snapshot| snapshot.cursor.is_some_and(|(edge, fraction)| edge > 0 || fraction > 0.0)));
            }
        }
    }
    pub(super) async fn start(case: Case) -> Harness<FakeMotion> {
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        Harness::new(case, FakeMotion(motion.clone()), motion).await
    }
    pub(super) async fn stopped_fixed() -> (Harness<FakeMotion>, Blocks) {
        let (case, blocks) = fixed_case(true, fixed::c3_start(), Some(0.0), [-1.0, 0.2], 0.0, Some(0.0));
        let mut harness = start(case).await;
        harness.wait_pending().await;
        (harness, blocks)
    }
    pub(super) fn replan(case: &Case, context: Arc<ActualExecutionContext>) -> PlannedGroundRoute {
        let incoming = context.navigation.as_ref().expect("paired real navigation").continuation().unwrap();
        let input = SurfaceReplanInput { source: context.source.clone(), start: context.odometry.clone(),
            goal: case.goal.clone(), options: case.options, boundary: Arc::new(vec![]), actual: Some(context),
            ground_intent: Some(GroundIntent::new(case.route.reference.full_window().unwrap())),
            limits: SearchLimits::default(), offset: 0.4, local_planning_enabled: case.config.local_planning_enabled,
            penalties: Arc::new(vec![]) };
        let route = input.compute_ground_route(&case.config.pct, None, None).unwrap();
        assert_eq!(route.reference.full_window().unwrap().from().walk_state().semantic_key(), incoming.semantic_key());
        assert_eq!(route.reference.full_window().unwrap().from().walk_state().reference_sample_xy(),
            incoming.walk_state().reference_sample_xy());
        route
    }
    pub(super) async fn install_map<M: NavMotionPort>(harness: &mut Harness<M>, map: Arc<SurfaceSnapshot>, obstacle: bool) {
        let status = harness.status.clone();
        let kind = if obstacle { InputKind::UpdateObstacleSource(Some(ExecutionMap::Surface(map)), true, harness.case.source_bound) }
        else { InputKind::UpdateCostmap(CostmapObservation { costmap: ExecutionMap::Surface(map),
            source_timestamp_ns: crate::command_dispatch::unix_ns().unwrap() as i64 }, Instant::now()) };
        let (reply, response) = oneshot::channel();
        harness.s().handle_input(&status, InputCommand { kind, reply }).await;
        response.await.unwrap().unwrap();
    }
    pub(super) fn updated_source(source: &SurfaceSnapshot, blocks: &Blocks, mode: &str) -> Vec<Arc<SurfaceSnapshot>> {
        let mut observations = source.observations().unwrap().as_ref().clone();
        let mut changed = blocks.clone();
        match mode {
            "metadata" => {
                for (&key, block) in &mut changed {
                    for value in block.values_mut() { value.observed_ns = 900; value.hits = 17; }
                    assert!(observations.replace_block_with_geometry_changes(key, block.clone()).unwrap().is_empty());
                }
                observations.mark_complete_geometry_source();
                vec![Arc::new(source.with_observations(Some(Arc::new(observations))))]
            }
            "recent" => vec![Arc::new(source.with_recent_obstacle_points(1, 1_000_000_000, 0, &[],
                std::iter::empty::<(u32, [f64; 3])>()).unwrap())],
            "geometry" => {
                // The existing c1c3_01 changed fine-column input, unchanged.
                let key: [i32; 3] = [0, 2, 4];
                let block_key = key.map(|v| v.div_euclid(32));
                let block = changed.entry(block_key).or_default();
                block.insert(key, SurfaceObservation { observed_ns: 901, hits: 2 });
                assert!(!observations.replace_block_with_geometry_changes(block_key, block.clone()).unwrap().is_empty());
                observations.mark_complete_geometry_source();
                vec![Arc::new(source.with_observations(Some(Arc::new(observations))))]
            }
            "delete_readd" => {
                let key: [i32; 3] = [-10, 2, -1];
                let block_key = key.map(|v| v.div_euclid(32));
                let block = changed.get_mut(&block_key).unwrap();
                let value = block.remove(&key).unwrap();
                assert!(!observations.replace_block_with_geometry_changes(block_key, block.clone()).unwrap().is_empty());
                observations.mark_complete_geometry_source();
                let deleted = Arc::new(source.with_observations(Some(Arc::new(observations.clone()))));
                block.insert(key, value);
                assert!(!observations.replace_block_with_geometry_changes(block_key, block.clone()).unwrap().is_empty());
                observations.mark_complete_geometry_source();
                vec![deleted, Arc::new(source.with_observations(Some(Arc::new(observations))))]
            }
            _ => unreachable!(),
        }
    }
    pub(super) struct Running {
        pub handle: NavSchedulerHandle,
        pub task: Option<tokio::task::JoinHandle<Result<(), RuntimeError>>>,
    }
    impl Drop for Running {
        fn drop(&mut self) { if let Some(task) = &self.task { task.abort(); } }
    }
    impl Running {
        pub async fn finish(mut self) {
            self.handle.shutdown().await.unwrap();
            self.task.take().unwrap().await.unwrap().unwrap();
        }
        pub async fn capture(&self, identity: &PathExecutionIdentity) -> Arc<ActualExecutionContext> {
            tokio::time::timeout(Duration::from_secs(3), async {
                loop {
                    if let Some(context) = self.handle.capture_planning_actual(identity.clone()).await.unwrap() {
                        if context.navigation.is_some() { return context; }
                    }
                    assert!(self.handle.status().last_terminal.is_none());
                    tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
                }
            }).await.expect("real bounded input query never paired current consumed navigation")
        }
    }
    pub(super) fn spawn<M: NavMotionPort>(harness: &mut Harness<M>) -> Running {
        let (handle, task) = spawn_scheduler(harness.scheduler.take().unwrap());
        Running { handle, task: Some(task) }
    }
    pub(super) async fn wait_snapshot(audit: &Arc<Mutex<Audit>>, after: usize,
        route: &PlannedGroundRoute) -> Snapshot {
        let original = route.reference.full_window().unwrap();
        tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let found = audit.lock().unwrap().snapshots.iter().skip(after).rev().find(|snapshot|
                    snapshot.binding.original_window().from()
                        .window_to(original.from()).is_some()).cloned();
                if let Some(snapshot) = found { return snapshot; }
                tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
            }
        }).await.expect("no admitted binding owned by the exact newly submitted PCT route")
    }
    pub(super) fn has_connector(snapshot: &Snapshot) -> bool {
        snapshot.binding.edges().iter().any(|edge| matches!(edge.origin(),
            CandidateEdgeOrigin::AdmissionConnector | CandidateEdgeOrigin::AdmissionTerminal))
    }
}
use ground_arrival_qualification as ga;

#[tokio::test]
async fn d07_real_ground_owner_stops_then_confirms_fresh_arrival() {
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let port = DispatchTestPort { inner: FakeMotion(motion.clone()), renewal_delay: None,
        expire_first: false, fail_bounded: false, bounded_attempts: 0, applied: Vec::new() };
    let mut harness = ga::Harness::new(ga::live_moving_case(), port, motion).await;
    let accepted = harness.s().current_execution_identity.clone().unwrap();
    let admitted = harness.wait_admitted().await;
    assert!(admitted.path.poses.windows(2).any(|pair| pair[0].pose.position != pair[1].pose.position));
    assert!(admitted.binding.planned_join() && admitted.binding.covers_original_goal());
    harness.consume_admitted_motion(&admitted).await;
    harness.wait_pending().await;
    let pending = harness.s().arrival_confirmation.as_ref().unwrap();
    assert!(Arc::ptr_eq(pending.execution_identity.as_ref().unwrap(), &accepted));
    assert_eq!(pending.proof.zero_ack_id, "zero-ack");
    assert_eq!(pending.proof.applied_monotonic_ns, 100);
    assert!(harness.s().pose_acquired_at.unwrap() < harness.s().arrival_confirmation.as_ref().unwrap().confirmed_at);
    harness.tick().await;
    assert!(harness.s().status.last_terminal.is_none());
    harness.confirm_fresh(TerminalOutcome::Succeeded).await;
    harness.assert_real_owner(true);
    assert!(harness.s().motion.bounded_attempts > 0);
    assert!(harness.s().motion.applied.iter().any(|twist| *twist != Twist2::ZERO));
    assert_eq!(harness.call_count("acquire:"), 1);
    assert_eq!(harness.call_count("fence:"), 1);
    let dropped = harness.dropped.clone();
    drop(harness);
    assert!(dropped.load(std::sync::atomic::Ordering::Acquire));
}

#[tokio::test]
async fn d08_pre_stop_sample_cannot_confirm_ground_arrival() {
    let (mut harness, _) = ga::stopped_fixed().await;
    let confirmed = harness.s().arrival_confirmation.as_ref().unwrap().confirmed_at;
    let original_stamp = harness.s().latest_pose.as_ref().unwrap().stamp;
    let delayed = ga::now_pose(harness.s().latest_pose.as_ref().unwrap().clone());
    assert!(delayed.stamp.nanos > original_stamp.nanos);
    harness.observe(delayed, confirmed - Duration::from_millis(1)).await;
    harness.tick().await;
    assert!(harness.s().status.last_terminal.is_none());
    assert!(!harness.s().lease_active);
    assert_eq!(harness.call_count("fence:"), 1);
    harness.confirm_fresh(TerminalOutcome::Succeeded).await;
    assert_eq!(harness.call_count("acquire:"), 1);
    harness.assert_real_owner(false);

    // Original finite pose/source prerequisites and the existing wait deadline
    // remain effective while Ground evidence is pending. No new expiry policy.
    for stale in ["pose", "source", "wait"] {
        let (mut harness, _) = ga::stopped_fixed().await;
        let fence = harness.s().arrival_confirmation.as_ref().unwrap().proof.clone();
        if stale == "pose" {
            let actual = ga::now_pose(harness.s().latest_pose.as_ref().unwrap().clone());
            let acquired = Instant::now();
            let status = harness.status.clone();
            let previous = (harness.s().latest_pose.clone(), harness.s().pose_acquired_at,
                harness.s().pose_valid_until, harness.s().path_wait_started_at,
                harness.s().arrival_confirmation.as_ref().unwrap().confirmed_at);
            assert!(matches!(harness.s().observe_odometry_at(&status, actual, Some(acquired), acquired),
                Err(RuntimeError::StaleInput(_))));
            assert_eq!((harness.s().latest_pose.clone(), harness.s().pose_acquired_at,
                harness.s().pose_valid_until, harness.s().path_wait_started_at,
                harness.s().arrival_confirmation.as_ref().unwrap().confirmed_at), previous);
            assert_eq!(harness.s().arrival_confirmation.as_ref().unwrap().proof, fence);
        } else if stale == "source" {
            let expired = CommandDispatchBound::from_source_timestamp(1, Duration::from_secs(1));
            assert!(expired.expired_now());
            let (reply, response) = oneshot::channel();
            let status = harness.status.clone();
            harness.s().handle_input(&status, InputCommand { kind: InputKind::UpdateObstacleSource(None, true, Some(expired)), reply }).await;
            response.await.unwrap().unwrap();
            let actual = ga::now_pose(harness.s().latest_pose.as_ref().unwrap().clone());
            harness.observe(actual, Instant::now()).await;
        }
        harness.tick().await;
        assert_ne!(harness.s().status.last_terminal, Some(TerminalOutcome::Succeeded));
        let original_wait = harness.s().config.path_wait_timeout;
        harness.s().path_wait_started_at = Some(Instant::now() - original_wait - Duration::from_millis(1));
        harness.tick().await;
        assert_eq!(harness.s().status.last_terminal, Some(TerminalOutcome::PlannerFailed));
        assert_eq!(harness.call_count("fence:"), 1);
        assert_eq!(harness.call_count("acquire:"), 1);
        assert_eq!(harness.s().status.last_zero_ack_id.as_deref(), Some(fence.zero_ack_id.as_str()));
        assert_eq!(harness.s().status.last_fence_monotonic_ns, fence.applied_monotonic_ns);
    }
}

#[tokio::test]
async fn d09_pending_arrival_observes_both_source_update_paths() {
    for obstacle_ingress in [false, true] {
        for mode in ["metadata", "recent", "geometry", "delete_readd"] {
            let (mut harness, blocks) = ga::stopped_fixed().await;
            let original = harness.case.source.clone();
            let sequence = ga::updated_source(&original, &blocks, mode);
            let compatible = mode == "metadata" || mode == "recent";
            for map in sequence {
                assert_eq!(nuwax_nav_pct::same_source_geometry(&original, &map), compatible);
                ga::install_map(&mut harness, map, obstacle_ingress).await;
            }
            let expected = if compatible { TerminalOutcome::Succeeded } else {
                TerminalOutcome::GoalIncomplete { reason: GoalIncompleteReason::TargetLayerUnresolved, xy_reached: true }
            };
            harness.confirm_fresh(expected).await;
            harness.assert_real_owner(false);
            assert_eq!(harness.call_count("fence:"), 1, "{mode} ingress={obstacle_ingress}");
            assert_eq!(harness.call_count("acquire:"), 1, "retirement must not create readmission loop");
        }
        let (mut harness, _) = ga::stopped_fixed().await;
        let status = harness.status.clone();
        if obstacle_ingress {
            let (reply, response) = oneshot::channel();
            harness.s().handle_input(&status, InputCommand {
                kind: InputKind::UpdateObstacleSource(None, false, None), reply,
            }).await;
            response.await.unwrap().unwrap();
            assert!(harness.s().session_tick(&status).await.is_err());
        } else {
            let (reply, response) = oneshot::channel();
            harness.s().handle_urgent(&status, UrgentCommand::InvalidateMap {
                reason: "D09 original source revocation".into(), invalidated_at: Instant::now(), reply,
            }).await;
            response.await.unwrap().unwrap();
        }
        assert_eq!(harness.s().status.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_eq!(harness.call_count("fence:"), 1);
        assert_eq!(harness.call_count("acquire:"), 1);
    }
}

#[tokio::test]
async fn d10_ground_drift_reacquires_and_reconnects_without_old_command() {
    for drift_kind in ["xy", "yaw"] {
        // The old no-calibration premise is retained in execution-14. Reuse
        // the existing complete-stop calibration before the same input capture.
        let (source,_)=crate::surface_follower::ground_owner_tests::c3_direct_source();
        let mut config=crate::surface_follower::ground_owner_tests::c3_config();
        config.motion_limits=Some(crate::surface_follower::ground_owner_tests::c3_limits());
        let acquired=Instant::now();
        let case=ga::make_case(Arc::new(source),config,
            ga::pose(crate::surface_follower::ground_owner_tests::c3_start()),
            Some(0.0),[-0.9,0.2],0.0,Some(0.0),acquired,None);
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let port = ArrivalAdmissionMotion { inner: FakeMotion(motion.clone()), acquisitions: 0,
            failure: None, entered: None, release: None };
        let mut harness = ga::Harness::new(case, port, motion).await;
        let first_path = harness.wait_admitted().await;
        harness.consume_admitted_motion(&first_path).await;
        harness.wait_pending().await;
        harness.assert_real_owner(true);
        let calls_at_stop = harness.motion.lock().unwrap().calls.len();
        let old_identity = harness.s().arrival_confirmation.as_ref().unwrap().execution_identity.clone().unwrap();
        let stopped_actual = harness.s().latest_pose.as_ref().unwrap().clone();
        let stopped_snapshot = { harness.audit.lock().unwrap().snapshots.last().cloned() }.unwrap();
        let old_basis = stopped_snapshot.actual.as_ref().unwrap()
            .navigation.as_ref().unwrap().continuation().unwrap();
        let mut drift = ga::now_pose(stopped_actual.clone());
        if drift_kind == "xy" { drift.pose.position.x -= 0.05; }
        else { drift.pose.orientation = nuwax_nav_types_messages::Quaternion::from_yaw(-0.2); }
        harness.observe(drift.clone(), Instant::now()).await;
        // The real worker pairs pose with its owned ground asynchronously.
        // Keep the original deadline and wait for that precise drift verdict,
        // stopping before the next tick can consume the resume request.
        while !harness.s().arrival_resume_requested() {
            harness.tick().await;
            if !harness.s().arrival_resume_requested() {tokio::task::yield_now().await;}
        }
        assert!(harness.s().arrival_resume_requested());
        assert_eq!(harness.s().status.phase, SchedulerPhase::Acquiring);
        assert!(!harness.s().lease_active);
        assert_eq!(harness.call_count("acquire:"),1);
        assert!(Arc::ptr_eq(harness.s().arrival_confirmation.as_ref().unwrap()
            .execution_identity.as_ref().unwrap(),&old_identity));
        assert!(harness.s().status.last_terminal.is_none());
        assert!(!harness.motion.lock().unwrap().calls[calls_at_stop..].iter().any(|call|call.starts_with("twist:")));
        harness.tick().await;
        assert_eq!(harness.call_count("acquire:"), 2);
        assert_eq!(harness.s().motion.acquisitions, 2, "original port generated a distinct second lease proof");
        assert_eq!(harness.s().local_intent_sequence, 0, "a pre-stop selected command cannot cross the new lease");
        assert!(!harness.motion.lock().unwrap().calls[calls_at_stop..].iter().any(|call| call.starts_with("twist:")));
        assert!(harness.s().status.last_zero_ack_id.is_none());
        let resumed_identity = harness.s().current_execution_identity.clone();
        if let Some(identity) = &resumed_identity { assert!(!Arc::ptr_eq(identity, &old_identity)); }
        assert!(harness.s().status.last_terminal.is_none());
        let running = ga::spawn(&mut harness);
        // Query the real bounded Input ingress, never the worker's private mutex.
        let context = running.capture(&harness.request.execution_identity).await;
        assert!(crate::replan_feedback::same_pose(&context.odometry, &drift));
        let continued = context.navigation.as_ref().unwrap().continuation().unwrap();
        assert_eq!(continued.walk_state().planned_ground_z(), old_basis.walk_state().planned_ground_z());
        assert_ne!(continued.walk_state().planned_ground_z(), Some(drift.pose.position.z));
        let route = ga::replan(&harness.case, context.clone());
        let before = harness.audit.lock().unwrap().snapshots.len();
        running.handle.submit_path(ga::ground_path(&route, &context.odometry, 2)).await.unwrap();
        let admitted = ga::wait_snapshot(&harness.audit, before, &route).await;
        assert!(ga::has_connector(&admitted), "actual readmission must form the real connector+retained suffix");
        assert_eq!(admitted.path.poses[0].pose, drift.pose);
        for actual in admitted.path.poses.iter().skip(1) {
            running.handle.update_odometry(ga::now_pose(actual.clone())).await.unwrap();
            tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
        }
        let mut status = running.handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(3), async {
            while !status.borrow().message.starts_with("已确认停稳") { status.changed().await.unwrap(); }
        }).await.unwrap();
        running.handle.update_odometry(ga::now_pose(stopped_actual)).await.unwrap();
        wait_for_terminal(&running.handle, TerminalOutcome::Succeeded).await;
        assert_eq!(harness.call_count("fence:"), 2);
        assert_eq!(harness.call_count("acquire:"), 2);
        running.finish().await;

    }

    // Same original uncertain-acquire and cancel precedence, using real
    // GroundArrival instead of assigning FakePlanner.step = Succeeded.
    let first_fence_boundary = |motion: &Arc<Mutex<FakeMotionState>>, session_id: &str, label: &str| {
        let (calls, twists) = {
            let state = motion.lock().unwrap();
            (state.calls.clone(), state.twist_probe.as_ref().unwrap().clone())
        };
        let fence = calls.iter().position(|call| call.starts_with("fence:")).unwrap();
        assert_eq!(calls.iter().filter(|call| call.starts_with("fence:")).count(), 1);
        assert_eq!(twists.len(), calls.iter().filter(|call| call.starts_with("twist:")).count());
        for (index, operation, command) in &twists {
            assert!(*index < fence, "no command may follow the verified first fence");
            assert!(operation.starts_with(&format!("nav:{session_id}:twist:")));
            assert_eq!(calls[*index], format!("twist:{operation}"));
            assert_eq!(*command, Twist2::ZERO, "fixed arrival may only submit pre-stop waiting ZERO");
        }
        eprintln!("D10_FIRST_FENCE label={label} index={fence} calls={calls:?} twists={twists:?}");
        calls.len()
    };
    for failure in ["denied", "uncertain", "wrong_generation"] {
        let (case, _) = ga::fixed_case(true, crate::surface_follower::ground_owner_tests::c3_start(),
            Some(0.0), [-1.0, 0.2], 0.0, Some(0.0));
        let motion = Arc::new(Mutex::new(FakeMotionState {
            twist_probe: Some(Vec::new()), ..Default::default()
        }));
        let port = ArrivalAdmissionMotion { inner: FakeMotion(motion.clone()), acquisitions: 0,
            failure: Some(failure), entered: None, release: None };
        let mut harness = ga::Harness::new(case, port, motion).await;
        harness.wait_pending().await;
        let calls_at_stop = first_fence_boundary(&harness.motion, &harness.request.session_id, failure);
        let mut drift = ga::now_pose(harness.s().latest_pose.as_ref().unwrap().clone());
        drift.pose.position.x -= 0.05;
        harness.observe(drift, Instant::now()).await;
        // Match the real asynchronous paired-context handoff used above;
        // do not begin the faulting acquire until drift has actually requested it.
        while !harness.s().arrival_resume_requested() {
            harness.tick().await;
            if !harness.s().arrival_resume_requested() {tokio::task::yield_now().await;}
        }
        assert_eq!(harness.call_count("acquire:"),1);
        assert!(!harness.s().lease_active);
        let status = harness.status.clone();
        assert!(harness.s().session_tick(&status).await.is_err());
        assert_eq!(harness.s().status.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_eq!(harness.s().status.last_zero_ack_id.as_deref(),
            Some(if failure == "denied" { "arrival-zero-1" } else { "arrival-zero-2" }));
        let calls = { harness.motion.lock().unwrap().calls.clone() };
        assert!(!calls[calls_at_stop..].iter().any(|call| call.starts_with("twist:")));
        eprintln!("D10_POST_FENCE label={failure} calls={:?}", &calls[calls_at_stop..]);
    }
    let (case, _) = ga::fixed_case(true, crate::surface_follower::ground_owner_tests::c3_start(),
        Some(0.0), [-1.0, 0.2], 0.0, Some(0.0));
    let motion = Arc::new(Mutex::new(FakeMotionState {
            twist_probe: Some(Vec::new()), ..Default::default()
        }));
    let entered = Arc::new(tokio::sync::Notify::new());
    let release = Arc::new(tokio::sync::Notify::new());
    let port = ArrivalAdmissionMotion { inner: FakeMotion(motion.clone()), acquisitions: 0,
        failure: None, entered: Some(entered.clone()), release: Some(release.clone()) };
    let mut harness = ga::Harness::new(case, port, motion).await;
    harness.wait_pending().await;
    let calls_at_stop = first_fence_boundary(&harness.motion, &harness.request.session_id, "cancel");
    let running = ga::spawn(&mut harness);
    let mut drift = ga::now_pose(harness.case.actual.clone()); drift.pose.position.x -= 0.05;
    running.handle.update_odometry(drift).await.unwrap();
    tokio::time::timeout(Duration::from_secs(3), entered.notified()).await.unwrap();
    let cancellation = running.handle.clone();
    let cancel = tokio::spawn(async move { cancellation.cancel("goal-1").await });
    tokio::time::sleep(Duration::from_millis(10)).await;
    assert!(!cancel.is_finished());
    assert!(!harness.motion.lock().unwrap().calls[calls_at_stop..].iter().any(|call| call.starts_with("twist:")));
    release.notify_one();
    tokio::time::timeout(Duration::from_secs(3), cancel).await.unwrap().unwrap().unwrap();
    assert_eq!(running.handle.status().last_terminal, Some(TerminalOutcome::Cancelled));
    assert_eq!(running.handle.status().last_zero_ack_id.as_deref(), Some("arrival-zero-2"));
    assert!(!harness.motion.lock().unwrap().calls[calls_at_stop..].iter().any(|call| call.starts_with("twist:")));
    eprintln!("D10_POST_FENCE label=cancel calls={:?}", &harness.motion.lock().unwrap().calls[calls_at_stop..]);
    running.finish().await;
}

#[tokio::test]
async fn d11_ground_layer_incomplete_finishes_with_verified_fence() {
    for requested in [None, Some(0.0), Some(0.2), Some(-0.2)] {
        let (case, _) = ga::fixed_case(true, ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]),
            None, [100.0, 100.0], 0.0, requested);
        assert_eq!(case.initial.basis().walk_state().planned_ground_z(), None);
        let mut harness = ga::start(case).await;
        harness.wait_pending().await;
        let expected = if requested.is_some() { TerminalOutcome::GoalIncomplete {
            reason: GoalIncompleteReason::TargetLayerUnresolved, xy_reached: true,
        }} else { TerminalOutcome::Succeeded };
        harness.confirm_fresh(expected).await;
        assert_eq!(harness.call_count("acquire:"), 1);
        assert_eq!(harness.call_count("fence:"), 1);
        harness.assert_real_owner(false);
    }
    use nuwax_nav_mls_planner::surface::BodyGroundSourceRelation;
    let source = Arc::new(crate::surface_follower::ground_owner_tests::c3_bin_shift_source());
    let xy = [0.025, 0.025];
    let actual = ([xy[0], xy[1], 0.705], [0.0, 0.0, 0.0, 1.0]);
    let records: Vec<_> = source.observations().unwrap().ground_records([0, 0]).collect();
    assert_eq!(records.len(), 2);
    assert_eq!(records[0].members().collect::<Vec<_>>(), vec![0]);
    assert_eq!(records[1].members().collect::<Vec<_>>(), vec![11]);
    let heights = [records[0].height_at(xy).unwrap(), records[1].height_at(xy).unwrap()];
    let config = crate::surface_follower::ground_owner_tests::c3_config();
    let (physical, actual_evidence) = source.body_query().start_ground_state_with_evidence(actual).unwrap();
    assert!(actual_evidence.is_direct());
    let mut relations = Vec::new();
    for height in heights {
        let initial = config.pct.prepare_ground_start(&source, nuwax_nav_pct::ground_route::GroundRouteStart {
            xy, ground_z: Some(height),
        }, None).unwrap();
        let walk = initial.basis().walk_state().walk_xy(xy, &[], None).unwrap();
        let query = initial.basis().bind_body_query(&source).unwrap();
        let (_, navigation_evidence) = walk.with_body_navigation(|_, navigation|
            query.validate_motion_with_navigation_and_evidence(actual, actual, &physical, navigation, &[], &mut None))
            .unwrap().expect("both original navigation associations retain real physical permission");
        relations.push(actual_evidence.relation_to(&navigation_evidence));
    }
    assert_eq!(relations.iter().filter(|&&r| r == BodyGroundSourceRelation::Same).count(), 1);
    assert_eq!(relations.iter().filter(|&&r| r == BodyGroundSourceRelation::KnownDifferent).count(), 1);
    let wrong = heights[relations.iter().position(|&r| r == BodyGroundSourceRelation::KnownDifferent).unwrap()];
    for requested in [None, Some(wrong)] {
        let case = ga::make_case(source.clone(), config.clone(), ga::pose(actual), Some(wrong), xy, 0.0,
            requested, Instant::now(), None);
        let mut harness = ga::start(case).await;
        harness.wait_pending().await;
        harness.confirm_fresh(TerminalOutcome::GoalIncomplete {
            reason: GoalIncompleteReason::KnownLayerConflict, xy_reached: true,
        }).await;
        assert_eq!(harness.call_count("acquire:"), 1);
        assert_eq!(harness.call_count("fence:"), 1);
        harness.assert_real_owner(false);
    }
}

#[tokio::test]
async fn d12_initial_and_owned_singletons_complete_ground_arrival() {
    // First zero-motion Ground admission, including the explicit OFF internal
    // control. No earlier nonzero APPLIED may be needed to carry private evidence.
    for on in [true, false] {
        let (case, _) = ga::fixed_case(on, crate::surface_follower::ground_owner_tests::c3_start(),
            Some(0.0), [-1.0, 0.2], 0.0, Some(0.0));
        assert_eq!(case.route.guide_body_poses.len(), 1);
        let mut harness = ga::start(case).await;
        harness.wait_pending().await;
        assert!(harness.audit.lock().unwrap().commands.iter().all(|command| *command == Twist2::ZERO));
        assert!(harness.audit.lock().unwrap().snapshots.iter().all(|snapshot|
            snapshot.path.poses.iter().all(|pose| pose.pose.position == harness.case.actual.pose.position)));
        harness.confirm_fresh(TerminalOutcome::Succeeded).await;
        harness.assert_real_owner(false);
        assert_eq!(harness.call_count("fence:"), 1);
    }
    let (case, _) = ga::fixed_case(true, crate::surface_follower::ground_owner_tests::c3_start(),
        Some(0.0), [-0.9, 0.2], 0.2, Some(0.0));
    let mut harness = ga::start(case).await;
    let admitted = harness.wait_admitted().await;
    // Observe the real translation samples with actual yaw still at its original
    // value. Final orientation is an actual input, never patched into odometry.
    for endpoint in admitted.path.poses.iter().skip(1) {
        let mut actual = ga::now_pose(endpoint.clone()); actual.pose.orientation = nuwax_nav_types_messages::Quaternion::IDENTITY;
        harness.observe(actual, Instant::now()).await;
        harness.tick().await;
    }
    let running = ga::spawn(&mut harness);
    let context = running.capture(&harness.request.execution_identity).await;
    assert_eq!([context.odometry.pose.position.x, context.odometry.pose.position.y], [-0.9, 0.2]);
    assert_eq!(context.odometry.pose.orientation, nuwax_nav_types_messages::Quaternion::IDENTITY);
    assert!(harness.audit.lock().unwrap().events.is_empty(), "unobserved requested yaw is not arrival");
    let singleton = ga::replan(&harness.case, context.clone());
    assert_eq!(singleton.guide_body_poses.len(), 1, "already-owned actual terminal retains a real singleton");
    let before = harness.audit.lock().unwrap().snapshots.len();
    running.handle.submit_path(ga::ground_path(&singleton, &context.odometry, 2)).await.unwrap();
    let yaw_path = ga::wait_snapshot(&harness.audit, before, &singleton).await;
    assert!(ga::has_connector(&yaw_path));
    assert!(yaw_path.path.poses.iter().all(|pose| pose.pose.position == context.odometry.pose.position));
    assert!(yaw_path.path.poses.windows(2).any(|pair| pair[0].pose.orientation != pair[1].pose.orientation));
    assert!(harness.audit.lock().unwrap().events.is_empty());
    for actual in yaw_path.path.poses.iter().skip(1) {
        running.handle.update_odometry(ga::now_pose(actual.clone())).await.unwrap();
        tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
    }
    let mut status = running.handle.subscribe_status();
    let stopped = tokio::time::timeout(Duration::from_secs(3), async {
        while !status.borrow().message.starts_with("已确认停稳") { status.changed().await.unwrap(); }
    }).await;
    if stopped.is_err() {
        let audit = { harness.audit.lock().unwrap().clone() };
        let last = yaw_path.path.poses.last().unwrap();
        eprintln!("D12 control yaw canonical={:?} requests={:?}",
            nuwax_nav_types_messages::Quaternion::from_yaw(harness.case.goal.as_ground().unwrap().yaw).yaw(),
            audit.final_yaws);
        eprintln!("D12 fixed yaw diagnostic goal={:?}/bits={} last_q={:?} extracted={:?}/bits={} status={:?} requests={:?} admissions={} snapshots={} events={} stops={} last_cursor={:?}",
            harness.case.goal.as_ground().unwrap().yaw, harness.case.goal.as_ground().unwrap().yaw.to_bits(),
            last.pose.orientation, last.pose.orientation.yaw(), last.pose.orientation.yaw().to_bits(),
            running.handle.status(), audit.requests, audit.admissions.len(), audit.snapshots.len(), audit.events.len(), audit.stops.len(), audit.snapshots.last().and_then(|s| s.cursor));
    }
    stopped.unwrap();
    running.handle.update_odometry(ga::now_pose(yaw_path.path.poses.last().unwrap().clone())).await.unwrap();
    wait_for_terminal(&running.handle, TerminalOutcome::Succeeded).await;
    assert_eq!(harness.call_count("acquire:"), 1);
    assert_eq!(harness.call_count("fence:"), 1);
    running.finish().await;
    // Independent actual Body singleton control through the same real worker;
    // the nine separately selected parking tests keep their original assertions.
    let (case, _) = ga::fixed_case(true, crate::surface_follower::ground_owner_tests::c3_start(),
        Some(0.0), [-1.0, 0.2], 0.0, None);
    let motion = Arc::new(Mutex::new(FakeMotionState::default()));
    let mut body = ga::Harness::new_body(case, FakeMotion(motion.clone()), motion).await;
    assert!(matches!(body.request.goal, ExecutionGoal::Body(_)) && body.request.ground_initial.is_none());
    while body.s().arrival_confirmation.is_none() {
        assert!(body.s().status.last_terminal.is_none());
        body.tick().await;
        if body.s().arrival_confirmation.is_none() { tokio::time::sleep(DEFAULT_TICK_INTERVAL).await; }
    }
    assert!(matches!(&body.s().arrival_confirmation.as_ref().unwrap().kind, crate::arrival_confirmation::ArrivalKind::Body));
    assert!(body.audit.lock().unwrap().events.is_empty());
    assert!(body.audit.lock().unwrap().body_successes > 0);
    body.confirm_fresh(TerminalOutcome::Succeeded).await;
    assert_eq!(body.call_count("fence:"), 1);
    assert_eq!(body.call_count("acquire:"), 1);
}
