// Included inside guarded_tests::ordinary_ack_cancel_tests with `use super::*`.
// Qualification only: real bridge, encoded LCM ingress, memq publication and the
// real scheduler. GateMotion's fence receipt is a test double, not physical ZERO.
// No latency target or hardware timing claim is made by these causal controls.
use nuwax_nav_runtime::{AppliedProof, FenceProof, LeaseProof, MotionPortError,
    NavMotionAdmission, NavMotionPort, NavPlanner};
use nuwax_nav_runtime::qualification_timing::{TraceData, TraceReport, TraceSession, TraceSink};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AtomicOrder};

const WATCHDOG: Duration = Duration::from_secs(3);
const A: &str = "scheduler.cancel.send.accepted";
const B: &str = "scheduler.cancel.handle.enter";
const CHECKPOINT: &str = "ordinary_ack_test.closed_checkpoint";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Gate { Renew, Submit, Fence, Acquire }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MotionEvent { Enter(Gate), Return(Gate), Dropped(Gate), Release(Gate), FenceOk, FenceUnconfirmed }
struct GateState {
    armed: Vec<Gate>,
    releases: BTreeMap<Gate, oneshot::Sender<()>>,
    events: Vec<MotionEvent>,
}
struct MotionProbe {
    state: Mutex<GateState>,
    changed: tokio::sync::watch::Sender<usize>,
    fail_fence: AtomicBool,
    trace: TraceSink,
}
impl MotionProbe {
    fn new(trace: TraceSink) -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(GateState { armed: Vec::new(), releases: BTreeMap::new(), events: Vec::new() }),
            changed: tokio::sync::watch::channel(0).0,
            fail_fence: AtomicBool::new(false), trace,
        })
    }
    fn record(&self, event: MotionEvent) {
        let count = { let mut s = self.state.lock().unwrap(); s.events.push(event); s.events.len() };
        assert!(count < 256, "bounded motion-event fixture overflow");
        self.changed.send_replace(count);
    }
    fn arm(&self, gate: Gate) {
        let mut s = self.state.lock().unwrap();
        assert!(!s.armed.contains(&gate) && !s.releases.contains_key(&gate));
        s.armed.push(gate);
    }
    fn armed(&self, gate: Gate) -> bool { self.state.lock().unwrap().armed.contains(&gate) }
    fn events(&self) -> Vec<MotionEvent> { self.state.lock().unwrap().events.clone() }
    async fn wait(&self, event: MotionEvent) {
        let mut changed = self.changed.subscribe();
        tokio::time::timeout(WATCHDOG, async {
            loop {
                if self.state.lock().unwrap().events.contains(&event) { return; }
                changed.changed().await.expect("motion event source stays owned");
            }
        }).await.unwrap_or_else(|_| panic!("motion event {event:?} missing: {:?}", self.events()));
    }
    fn release(&self, gate: Gate) {
        let release = self.state.lock().unwrap().releases.remove(&gate);
        if let Some(release) = release {
            self.record(MotionEvent::Release(gate));
            let _ = release.send(());
        }
    }
    fn release_all(&self) {
        self.fail_fence.store(false, AtomicOrder::SeqCst);
        let releases = { let mut s = self.state.lock().unwrap(); s.armed.clear(); std::mem::take(&mut s.releases) };
        for (gate, sender) in releases { self.record(MotionEvent::Release(gate)); let _ = sender.send(()); }
    }
    async fn hold(self: &Arc<Self>, gate: Gate) {
        let receive = {
            let mut s = self.state.lock().unwrap();
            let Some(index) = s.armed.iter().position(|candidate| *candidate == gate) else { return; };
            s.armed.remove(index);
            let (send, receive) = oneshot::channel();
            assert!(s.releases.insert(gate, send).is_none());
            receive
        };
        self.record(MotionEvent::Enter(gate));
        let mut pending = PendingMotion { probe: self.clone(), gate, returned: false };
        receive.await.expect("ordinary gate is explicitly released or interrupted by urgent input");
        pending.returned = true;
        self.record(MotionEvent::Return(gate));
    }
}
struct PendingMotion { probe: Arc<MotionProbe>, gate: Gate, returned: bool }
impl Drop for PendingMotion {
    fn drop(&mut self) { if !self.returned { self.probe.record(MotionEvent::Dropped(self.gate)); } }
}
struct GateMotion { inner: RecordingMotion, probe: Arc<MotionProbe> }
impl NavMotionPort for GateMotion {
    fn renewal_due_at(&self) -> Option<tokio::time::Instant> {
        self.probe.armed(Gate::Renew).then(tokio::time::Instant::now)
    }
    fn acquire<'life0, 'life1, 'life2, 'async_trait>(&'life0 mut self, id: &'life1 str, admission: &'life2 NavMotionAdmission) -> Pin<Box<dyn Future<Output = Result<LeaseProof, MotionPortError>> + Send + 'async_trait>>
    where 'life0: 'async_trait, 'life1: 'async_trait, 'life2: 'async_trait, Self: 'async_trait {
        Box::pin(async move {
        self.probe.hold(Gate::Acquire).await;
        self.inner.acquire(id, admission).await
            })
    }
    fn renew<'life0, 'life1, 'async_trait>(&'life0 mut self, id: &'life1 str) -> Pin<Box<dyn Future<Output = Result<(), MotionPortError>> + Send + 'async_trait>>
    where 'life0: 'async_trait, 'life1: 'async_trait,  Self: 'async_trait {
        Box::pin(async move {
        self.probe.hold(Gate::Renew).await;
        self.inner.renew(id).await
            })
    }
    fn submit_twist<'life0, 'life1, 'async_trait>(&'life0 mut self, id: &'life1 str, twist: Twist2) -> Pin<Box<dyn Future<Output = Result<AppliedProof, MotionPortError>> + Send + 'async_trait>>
    where 'life0: 'async_trait, 'life1: 'async_trait,  Self: 'async_trait {
        Box::pin(async move {
        if twist != Twist2::ZERO { self.probe.hold(Gate::Submit).await; }
        self.inner.submit_twist(id, twist).await
            })
    }
    fn revoke_and_zero<'life0, 'life1, 'async_trait>(&'life0 mut self, id: &'life1 str) -> Pin<Box<dyn Future<Output = Result<FenceProof, MotionPortError>> + Send + 'async_trait>>
    where 'life0: 'async_trait, 'life1: 'async_trait,  Self: 'async_trait {
        Box::pin(async move {
        self.probe.hold(Gate::Fence).await;
        if self.probe.fail_fence.load(AtomicOrder::SeqCst) {
            self.probe.record(MotionEvent::FenceUnconfirmed);
            return Err(MotionPortError { message: "ordinary-ack controlled stop remains unconfirmed".into(),
                lease_may_be_active: true, stop_unconfirmed: true });
        }
        let result = self.inner.revoke_and_zero(id).await;
        if result.is_ok() {
            self.probe.trace.mark("ordinary_ack_test.fence_proof", 0);
            self.probe.record(MotionEvent::FenceOk);
        }
        result
            })
    }
}

// A source-auditing planner double accepts SurfaceUpdate only in the explicit
// map/recent source-fidelity fixture. No geometric/planner correctness is claimed.
struct SourcePlanner(CommandingPlanner);
impl NavPlanner for SourcePlanner {
    fn replace_path(&mut self, p: Path2D) { self.0.replace_path(p); }
    fn update_odometry(&mut self, p: PoseStamped) { self.0.update_odometry(p); }
    fn update_costmap(&mut self, m: nuwax_nav_astar::navigation_map::CostMap2D) { self.0.update_costmap(m); }
    fn update_execution_map(&mut self, map: nuwax_nav_runtime::ExecutionMap) -> Result<(), RuntimeError> {
        if let nuwax_nav_runtime::ExecutionMap::Planar(map) = map { self.0.update_costmap(map); }
        Ok(())
    }
    fn step(&mut self, width: f64) -> nuwax_nav_runtime::PlannerStep { self.0.step(width) }
    fn stop(&mut self) { self.0.stop(); }
}

type MapInput = (TransportResult<GuardedMapFrame>, Option<oneshot::Sender<()>>);
struct Harness {
    goals: GuardedTestSender, poses: GuardedTestSender, paths: GuardedTestSender,
    maps: UnboundedSender<MapInput>, cancels: GuardedTestSender,
    states: LcmFrameStream, cancelled: LcmFrameStream,
    node: ModuleNode, transport: LcmTransport,
    scheduler: NavSchedulerHandle,
    loop_task: Option<JoinHandle<NavStackResult<()>>>,
    scheduler_task: Option<JoinHandle<Result<(), RuntimeError>>>,
    shutdown: Option<oneshot::Sender<()>>,
    probe: Arc<MotionProbe>, calls: Arc<Mutex<Vec<String>>>, trace: Option<TraceSession>,
}
impl Harness {
    async fn new(recent_max_age: Option<Duration>) -> Self {
        let trace = TraceSession::start(8192);
        let probe = MotionProbe::new(trace.sink());
        let calls = Arc::new(Mutex::new(Vec::new()));
        let scheduler = nuwax_nav_runtime::NavScheduler::new(
            GateMotion { inner: RecordingMotion(calls.clone()), probe: probe.clone() },
            SourcePlanner(CommandingPlanner { succeeds: Arc::new(AtomicBool::new(false)),
                twist: Arc::new(Mutex::new(Twist2 { linear: Point2::new(0.2, 0.0), angular_z: 0.1 })) }),
            nuwax_nav_runtime::SchedulerConfig {
                tick_interval: Duration::from_millis(5), renew_interval: Duration::from_millis(100),
                path_wait_timeout: Duration::from_secs(5), robot_width_m: 0.3,
                pose_ttl: Duration::from_secs(5), costmap_ttl: Duration::from_secs(5),
            },
        ).expect("original scheduler").with_qualification_timing(trace.sink());
        let transport = LcmTransport::new_real("memq://").expect("real liblcm memq context");
        let (scheduler, scheduler_task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
        let mut node = ModuleNode::new("ordinary-ack-cancel", "ordinary ACK qualification", crate::graph::ModuleKind::Coordinator);
        node.outputs = vec!["/test/ordinary-ack/cmd".into(), "/test/ordinary-ack/state".into(),
            "/test/ordinary-ack/reached".into(), "/test/ordinary-ack/cancelled".into()];
        let (goals, goal_stream) = guarded_test_stream();
        let (poses, pose_stream) = guarded_test_stream();
        let (paths, path_stream) = guarded_test_stream();
        let (cancels, cancel_stream) = guarded_test_stream();
        let (maps, map_rx) = unbounded::<MapInput>();
        let map_stream = map_rx.map(|(frame, receipt)| { if let Some(receipt) = receipt { let _ = receipt.send(()); } frame });
        let (shutdown, shutdown_rx) = oneshot::channel();
        // Construct the owner before fallible asynchronous subscription/setup, so
        // setup panics are cleaned up by the same run() catch_unwind boundary.
        let mut h = Self { goals, poses, paths, maps, cancels,
            states: Box::pin(futures::stream::pending()), cancelled: Box::pin(futures::stream::pending()),
            node, transport, scheduler, loop_task: None, scheduler_task: Some(scheduler_task),
            shutdown: Some(shutdown), probe, calls, trace: Some(trace) };
        let setup = std::panic::AssertUnwindSafe(async {
            h.states = typed_stream(&h.transport, &h.node.outputs[1], "nuwax_msgs.NavigationState").await.expect("real state stream");
            h.cancelled = typed_stream(&h.transport, &h.node.outputs[3], "std_msgs.Bool").await.expect("real cancel receipt stream");
            h.status_when(|s| s.motion_sink_ready).await;
            let node = h.node.clone(); let transport = h.transport.clone(); let handle = h.scheduler.clone();
            let status = handle.subscribe_status();
            h.loop_task = Some(tokio::spawn(async move {
                run_guarded_nav_loop(&node, &transport, &handle, shutdown_rx, goal_stream, pose_stream,
                    path_stream, map_stream, cancel_stream, status, None, false, "base_link", None, recent_max_age).await
            }));
        }).catch_unwind().await;
        if let Err(panic) = setup {
            let (cleanup, _) = h.cleanup().await;
            eprintln!("ordinary ACK setup cleanup: {cleanup:?}");
            std::panic::resume_unwind(panic);
        }
        h
    }
    fn sink(&self) -> TraceSink { self.trace.as_ref().unwrap().sink() }
    async fn status_when(&self, predicate: impl Fn(&nuwax_nav_runtime::SchedulerStatus) -> bool) {
        let mut status = self.scheduler.subscribe_status();
        tokio::time::timeout(WATCHDOG, async {
            loop {
                if predicate(&status.borrow_and_update()) { return; }
                status.changed().await.expect("original scheduler status open");
            }
        }).await.unwrap_or_else(|_| panic!("scheduler predicate missing: {:?}", self.scheduler.status()));
    }
    fn count(&self, prefix: &str) -> usize { self.calls.lock().unwrap().iter().filter(|s| s.starts_with(prefix)).count() }
    fn pose(&self, stamp: i32) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed(&self.poses, "geometry_msgs.PoseStamped", guarded_pose_body(stamp))
    }
    fn path(&self, path: &schema::PathEnvelope) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed(&self.paths, "nuwax_msgs.PathEnvelope", schema::encode_path_envelope_body(path).unwrap())
    }
    fn goal(&self, goal: &schema::GoalEnvelope) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed(&self.goals, "nuwax_msgs.GoalEnvelope", schema::encode_goal_envelope_body(goal).unwrap())
    }
    fn cancel(&self, goal: &schema::GoalEnvelope, generation: i64) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed(&self.cancels, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(goal, generation))
    }
    fn map_frame(&self, frame: GuardedMapFrame) -> oneshot::Receiver<()> {
        let (send, receive) = oneshot::channel();
        assert!(self.maps.unbounded_send((Ok(frame), Some(send))).is_ok(), "original map input open"); receive
    }
    fn map(&self, stamp: i32) -> oneshot::Receiver<()> {
        self.map_frame(GuardedMapFrame::Planar(build_body_frame("/test/ordinary-ack/map", "nav_msgs.OccupancyGrid", guarded_costmap_body(stamp)).unwrap()))
    }
    async fn activate(&self, id: &str) -> schema::GoalEnvelope {
        received(self.pose(1)).await;
        self.status_when(|s| s.pose_source_timestamp_ns == 1_000_000_000).await;
        received(self.map(11)).await;
        self.status_when(|s| s.costmap_source_timestamp_ns == 11_000_000_000).await;
        let goal = sealed_goal(id);
        received(self.goal(&goal)).await;
        self.status_when(|s| s.session_id.as_deref() == Some(id) && s.phase == SchedulerPhase::AwaitingPath).await;
        goal
    }
    async fn block_renew(&self) {
        self.probe.arm(Gate::Renew);
        // The existing tick wakes the scheduler; success is observed at the
        // exact gate-entry event, never inferred from elapsed wall-clock time.
        self.probe.wait(MotionEvent::Enter(Gate::Renew)).await;
    }
    async fn state_when(&mut self, predicate: impl Fn(&schema::NavigationState) -> bool) -> schema::NavigationState {
        tokio::time::timeout(WATCHDOG, async {
            loop {
                let frame = self.states.next().await.expect("state stream open").expect("real memq state");
                let state = schema::decode_navigation_state_body(&frame.payload).expect("exact state codec");
                if predicate(&state) { return state; }
            }
        }).await.expect("matching real LCM state")
    }
    async fn zero(&mut self, goal: &schema::GoalEnvelope, generation: i64) -> schema::NavigationState {
        let state = self.state_when(|s| s.session_id == goal.session_id && s.cancel_generation == generation
            && s.message.starts_with("CANCELLED:") && !s.zero_ack_id.is_empty()).await;
        assert_eq!(state.owner_kind, goal.owner_kind);
        assert_eq!(state.parent_request_id, goal.parent_request_id);
        assert_eq!(state.goal_generation, goal.goal_generation);
        assert_eq!(state.producer_runtime_generation, goal.producer_runtime_generation);
        assert_eq!(state.zero_applied_monotonic_ns, 100);
        assert_eq!(state.path_revision, 0);
        assert_eq!(state.planner_runtime_generation, 0);
        state
    }
    async fn cleanup(&mut self) -> (Vec<String>, TraceReport) {
        self.probe.release_all();
        if let Some(send) = self.shutdown.take() { let _ = send.send(()); }
        let mut errors = Vec::new();
        if let Some(mut task) = self.loop_task.take() {
            match tokio::time::timeout(WATCHDOG, &mut task).await {
                Ok(Ok(Ok(()))) => {},
                Ok(result) => errors.push(format!("bridge result {result:?}")),
                Err(_) => { errors.push("bridge shutdown timed out; abort is failure cleanup only".into()); task.abort(); let _ = task.await; },
            }
        }
        let shutdown = tokio::time::timeout(WATCHDOG, self.scheduler.shutdown()).await;
        if !matches!(shutdown, Ok(Ok(())) | Ok(Err(RuntimeError::SchedulerStopped))) { errors.push(format!("scheduler shutdown {shutdown:?}")); }
        if let Some(mut task) = self.scheduler_task.take() {
            match tokio::time::timeout(WATCHDOG, &mut task).await {
                Ok(Ok(Ok(()))) => {},
                Ok(result) => errors.push(format!("scheduler result {result:?}")),
                Err(_) => { errors.push("scheduler shutdown timed out; abort is failure cleanup only".into()); task.abort(); let _ = task.await; },
            }
        }
        let report = self.trace.take().unwrap().finish();
        (errors, report)
    }
}
async fn received(receive: oneshot::Receiver<()>) {
    tokio::time::timeout(WATCHDOG, receive).await.expect("exact ingress dequeued").expect("dequeue owner alive");
}
async fn run(body: impl for<'a> FnOnce(&'a mut Harness) -> Pin<Box<dyn Future<Output = ()> + 'a>>) -> TraceReport {
    run_with_recent(None, body).await
}
async fn run_with_recent(recent_max_age: Option<Duration>, body: impl for<'a> FnOnce(&'a mut Harness) -> Pin<Box<dyn Future<Output = ()> + 'a>>) -> TraceReport {
    let mut h = Harness::new(recent_max_age).await;
    let outcome = std::panic::AssertUnwindSafe(async {
        tokio::time::timeout(WATCHDOG, body(&mut h)).await
            .expect("scenario watchdog; success always requires explicit causal evidence");
    }).catch_unwind().await;
    let (cleanup, report) = h.cleanup().await;
    if let Err(panic) = outcome { eprintln!("ordinary ACK panic cleanup: {cleanup:?}; trace={report:?}"); std::panic::resume_unwind(panic); }
    assert!(cleanup.is_empty(), "normal cleanup required: {cleanup:?}");
    assert!(!report.incomplete, "qualification trace incomplete: {report:?}");
    // Print only bounded causal observations after all actors have finished.
    // No elapsed-time result is used as a deadline or hardware latency claim.
    let causal: Vec<_> = report.records.iter().enumerate().filter(|(_, event)| {
        event.phase == A || event.phase == B || event.phase.starts_with("ordinary_ack_test.")
            || event.phase.starts_with("scheduler.input.")
    }).map(|(index, event)| (index, event.phase, event.source)).collect();
    eprintln!("ORDINARY_ACK_CAUSAL {causal:?}");
    report
}
fn first(report: &TraceReport, phase: &str) -> usize {
    report.records.iter().position(|event| event.phase == phase).unwrap_or_else(|| panic!("missing {phase}: {report:?}"))
}
fn matching(report: &TraceReport, phase: &str, source: i64) -> Vec<usize> {
    report.records.iter().enumerate().filter_map(|(index, event)| (event.phase == phase && event.source == source).then_some(index)).collect()
}
fn assert_abc(report: &TraceReport) {
    assert!(first(report, A) < first(report, B));
    assert!(first(report, B) < first(report, "ordinary_ack_test.fence_proof"));
}
fn assert_gate_interrupted(h: &Harness, gate: Gate) {
    let events = h.probe.events();
    assert!(events.contains(&MotionEvent::Dropped(gate)), "original pending motion was interrupted");
    assert!(!events.contains(&MotionEvent::Return(gate)), "ordinary gate must not be released to make Cancel progress");
    assert!(!events.contains(&MotionEvent::Release(gate)), "checkpoint precedes cleanup-only release");
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit real bridge ordinary-ACK cancellation qualification"]
async fn guarded_ordinary_ack_cancel_preempts_pending_renew_pose() {
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-renew-pose").await;
        h.block_renew().await;
        received(h.pose(2)).await;
        assert_eq!(h.scheduler.status().pose_source_timestamp_ns, 1_000_000_000);
        h.sink().mark(CHECKPOINT, 2_000_000_000);
        received(h.cancel(&goal, 1)).await;
        h.zero(&goal, 1).await;
        h.status_when(|s| s.pose_source_timestamp_ns == 2_000_000_000).await;
        assert_gate_interrupted(h, Gate::Renew);
        assert_eq!(h.count("acquire:"), 1);
        assert_eq!(h.count("zero:"), 1);
        assert_eq!(h.scheduler.status().last_terminal, Some(TerminalOutcome::Cancelled));
    })).await;
    assert_abc(&report);
    let accepted = matching(&report, "scheduler.input.send.accepted", 2_000_000_000);
    let replies = matching(&report, "scheduler.input.reply.ok", 2_000_000_000);
    assert_eq!(accepted.len(), 1); assert_eq!(replies.len(), 1);
    assert!(accepted[0] < first(&report, CHECKPOINT));
    assert!(first(&report, CHECKPOINT) < first(&report, A));
    assert!(first(&report, "ordinary_ack_test.fence_proof") < replies[0]);
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit real bridge late path cannot revive a cancelled session"]
async fn guarded_ordinary_ack_cancel_preempts_pending_submit_path() {
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-submit-path").await;
        let path = sealed_path(&goal);
        received(h.path(&path)).await;
        h.status_when(|s| s.phase == SchedulerPhase::Controlling && s.path_revision == 1).await;
        h.probe.arm(Gate::Submit);
        h.probe.wait(MotionEvent::Enter(Gate::Submit)).await;
        let before = h.count("twist:");
        let mut late = path; late.path_revision = 2;
        h.sink().mark("ordinary_ack_test.late_path_begin", 0);
        received(h.path(&late)).await;
        h.sink().mark(CHECKPOINT, 0);
        received(h.cancel(&goal, 1)).await;
        h.zero(&goal, 1).await;
        assert_gate_interrupted(h, Gate::Submit);
        assert_eq!(h.count("twist:"), before, "late path must never resubmit a nonzero command");
        assert_eq!(h.count("acquire:"), 1);
        assert_eq!(h.scheduler.status().path_revision, 0);
        assert_eq!(h.scheduler.status().session_id, None);
    })).await;
    assert_abc(&report);
    let begin = first(&report, "ordinary_ack_test.late_path_begin");
    let accepted: Vec<_> = matching(&report, "scheduler.input.send.accepted", 0).into_iter().filter(|i| *i > begin).collect();
    let refused: Vec<_> = matching(&report, "scheduler.input.reply.error", 0).into_iter().filter(|i| *i > begin).collect();
    assert_eq!(accepted.len(), 1); assert_eq!(refused.len(), 1);
    assert!(accepted[0] < first(&report, A));
    assert!(first(&report, "ordinary_ack_test.fence_proof") < refused[0]);
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit original 32-slot queue send-ownership control"]
async fn guarded_ordinary_ack_cancel_preserves_full_queue_send() {
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-full-queue").await;
        h.block_renew().await;
        let handle = h.scheduler.clone();
        let mut inputs = Vec::new();
        // These are the actual public handle futures. The original bounded
        // channel, not a test queue or scheduler seam, accepts exactly 32.
        for index in 0..32 {
            let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
            pose.stamp = Timestamp::from_unix(100 + index, 0);
            let mut input = Box::pin(handle.update_odometry(pose));
            assert!(futures::poll!(&mut input).is_pending());
            inputs.push(input);
        }
        h.sink().mark("ordinary_ack_test.queue_filled", 0);
        received(h.path(&sealed_path(&goal))).await;
        h.sink().mark(CHECKPOINT, 0);
        received(h.cancel(&goal, 1)).await;
        // Poll every admitted original input to its real reply; no detached
        // filler, dropped future, queue mutation or reconstructed send exists.
        let results = tokio::time::timeout(WATCHDOG, futures::future::join_all(inputs)).await.expect("all original queue fillers settle");
        assert_eq!(results.len(), 32);
        assert!(results.iter().all(Result::is_ok), "real queued replies: {results:?}");
        h.zero(&goal, 1).await;
        assert_gate_interrupted(h, Gate::Renew);
        assert_eq!(h.count("acquire:"), 1);
        assert_eq!(h.count("twist:"), 0);
        assert_eq!(h.count("zero:"), 1);
    })).await;
    assert_abc(&report);
    let filled = first(&report, "ordinary_ack_test.queue_filled");
    for index in 0..32 {
        let source = (100 + index) * 1_000_000_000;
        let accepted = matching(&report, "scheduler.input.send.accepted", source);
        let replies = matching(&report, "scheduler.input.reply.ok", source);
        assert_eq!(accepted.len(), 1); assert_eq!(replies.len(), 1);
        assert!(accepted[0] < filled && filled < replies[0]);
    }
    let accepted: Vec<_> = matching(&report, "scheduler.input.send.accepted", 0).into_iter().filter(|i| *i > filled).collect();
    let refused: Vec<_> = matching(&report, "scheduler.input.reply.error", 0).into_iter().filter(|i| *i > filled).collect();
    assert_eq!(accepted.len(), 1); assert_eq!(refused.len(), 1);
    let begin = matching(&report, "scheduler.input.send.begin", 0).into_iter().find(|i| *i > filled).unwrap();
    assert!(begin < first(&report, A), "same original send starts before Cancel");
    assert!(first(&report, "ordinary_ack_test.fence_proof") < accepted[0], "33rd ordinary send is not accepted until urgent cancellation frees the original queue");
    assert!(accepted[0] < refused[0]);
}

#[cfg(feature = "mls")]
fn source_fixture(goal: &schema::GoalEnvelope, source: i64, received_at: Instant) -> surface_dataflow::PreparedSurfaceMap {
    let config = crate::surface_config::SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.2, 35.0, 0.2).unwrap();
    let points: Vec<_> = (0..40).flat_map(|x| (0..30).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))).collect();
    let snapshot = Arc::new(nuwax_nav_mls_planner::SurfaceSnapshot::from_points(&points, config.geometry, 0.1).unwrap());
    let authority = GuardedMapAuthority {
        committed_map: schema::CommittedMapIdentity {
            session_id: goal.committed_map_session_id.clone(), session_generation: goal.committed_map_session_generation,
            revision: goal.committed_map_revision, commit_boot_nonce: goal.committed_map_boot_nonce,
            commit_sequence: goal.committed_map_commit_sequence, source_timestamp_ns: goal.committed_map_source_timestamp_ns,
            frame_id: goal.committed_map_frame_id.clone(),
        },
        producer_runtime_generation: goal.producer_runtime_generation, map_frame_generation: goal.map_frame_generation,
        cloud_revision: 1, cloud_occupancy_revision: 1, cloud_commit_sequence: 1,
        cloud_source_timestamp_ns: source, commit_boot_nonce: goal.committed_map_boot_nonce,
    };
    surface_dataflow::PreparedSurfaceMap { authority, source_timestamp_ns: source, received_at,
        awaiting_build: false, snapshot: Ok(snapshot), planar_reference: None, change_basis: None,
        geometry_changes: Arc::new(crate::surface_map::SurfaceGeometryChanges::Empty) }
}
#[cfg(feature = "mls")]
fn recent_frame(authority: &GuardedMapAuthority, source: i64, sequence: i64) -> RawLcmFrame {
    use nuwax_codec::lcm::{lio, recent_cloud};
    let cloud = schema::PointCloud2 {
        header: schema::Header { seq: sequence as i32, stamp_sec: (source / 1_000_000_000) as i32,
            stamp_nsec: (source % 1_000_000_000) as i32, frame_id: "map".into() },
        height: 1, width: 1, fields: ["x", "y", "z"].into_iter().enumerate().map(|(i, name)| schema::PointField {
            name: name.into(), offset: (i * 4) as i32, datatype: 7, count: 1,
        }).collect(), is_bigendian: false, point_step: 12, row_step: 12,
        data: [0.5_f32, 1.0, 0.5].into_iter().flat_map(f32::to_le_bytes).collect(), is_dense: true,
    };
    let packet = recent_cloud::RecentCloud3D {
        registration:None,
        producer_runtime_generation: authority.producer_runtime_generation,
        session_id: authority.committed_map.session_id.clone(), session_generation: authority.committed_map.session_generation,
        map_frame_generation: authority.map_frame_generation, commit_boot_nonce: authority.commit_boot_nonce,
        ground_point_indices: Vec::new(), cloud: Some(lio::LioCloudEnvelope {
            schema_version: 1, backend: lio::LIO_BACKEND_DEVICE, runtime_generation: 2, source_sequence: sequence,
            source_timestamp_ns: source, correlated_odometry_source_sequence: sequence,
            correlated_odometry_source_timestamp_ns: source, sensor_frame_id: "sensor".into(), body_frame_id: "body".into(),
            odom_frame_id: "odom".into(), map_frame_id: "map".into(), registration_state: lio::LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count: 1, emitted_point_count: 1, cloud,
        }),
    };
    // The original RecentCloud branch dispatches by GuardedMapFrame, then uses
    // the production recent_cloud decoder on this exact immutable body.
    build_body_frame("/test/ordinary-ack/recent", "nuwax_msgs.RecentCloud3D", recent_cloud::encode_body(&packet).unwrap()).unwrap()
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit map receipt and recent source-bound ownership controls"]
async fn guarded_ordinary_ack_cancel_preserves_map_recent_sources() {
    #[cfg(not(feature = "mls"))]
    panic!("this predeclared source-fidelity selector requires the mls qualification feature");
    #[cfg(feature = "mls")]
    {
        // Fixed subcontrol 1: original prepared-map receipt survives ordinary ACK
        // backpressure and the exact original Cancel. No post-ACK receipt refresh.
        let map_report = run(|h| Box::pin(async move {
            let goal = h.activate("ordinary-map-source").await;
            h.block_renew().await;
            let original_receipt = Instant::now() - Duration::from_millis(300);
            received(h.map_frame(GuardedMapFrame::Surface(source_fixture(&goal, 12_000_000_000, original_receipt)))).await;
            h.sink().mark(CHECKPOINT, 12_000_000_000);
            received(h.cancel(&goal, 1)).await;
            h.zero(&goal, 1).await;
            let status = h.scheduler.status();
            assert_eq!(status.costmap_source_timestamp_ns, 12_000_000_000);
            assert!(status.costmap_receipt_age_ms >= 300, "original receive time must not be rejuvenated at ACK");
            assert_eq!(status.last_terminal, Some(TerminalOutcome::Cancelled));
            assert_eq!(h.count("acquire:"), 1);
            assert_gate_interrupted(h, Gate::Renew);
        })).await;
        assert_abc(&map_report);
        assert_eq!(matching(&map_report, "scheduler.input.send.accepted", 12_000_000_000).len(), 1);
        assert_eq!(matching(&map_report, "scheduler.input.reply.ok", 12_000_000_000).len(), 1);

        // Fixed subcontrol 2: exact encoded recent frames traverse the original
        // bridge RecentSourceState::consume, real send/ACK and local watermark.
        let recent_report = run_with_recent(Some(Duration::from_secs(5)), |h| Box::pin(async move {
            let goal = sealed_goal("ordinary-recent-source");
            received(h.pose(1)).await;
            h.status_when(|s| s.pose_source_timestamp_ns == 1_000_000_000).await;
            let prepared = source_fixture(&goal, 11_000_000_000, Instant::now());
            let authority = prepared.authority.clone();
            received(h.map_frame(GuardedMapFrame::Surface(prepared))).await;
            h.status_when(|s| s.costmap_source_timestamp_ns == 11_000_000_000).await;
            let source = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as i64;
            received(h.map_frame(GuardedMapFrame::RecentCloud(recent_frame(&authority, source, 1)))).await;
            h.status_when(|s| s.planner_inputs_fresh).await;
            received(h.goal(&goal)).await;
            h.status_when(|s| s.session_id.as_deref() == Some(goal.session_id.as_str())).await;
            h.block_renew().await;
            let second = recent_frame(&authority, source + 1, 2);
            let replay = second.clone();
            received(h.map_frame(GuardedMapFrame::RecentCloud(second))).await;
            h.sink().mark(CHECKPOINT, source + 1);
            received(h.cancel(&goal, 1)).await;
            h.zero(&goal, 1).await;
            assert_gate_interrupted(h, Gate::Renew);
            assert_eq!(h.scheduler.status().costmap_source_timestamp_ns, 11_000_000_000,
                "recent evidence must not replace persistent-map source time");
            received(h.map_frame(GuardedMapFrame::RecentCloud(replay))).await;
            // The exact replay is rejected by the state committed after its real
            // ACK. This following real pose is a source-processing barrier.
            received(h.pose(3)).await;
            h.status_when(|s| s.pose_source_timestamp_ns == 3_000_000_000).await;
            assert_eq!(h.count("acquire:"), 1);
            assert_eq!(h.scheduler.status().last_terminal, Some(TerminalOutcome::Cancelled));
        })).await;
        assert_abc(&recent_report);
        let admitted: Vec<_> = recent_report.records.iter().filter(|e| e.phase == "recent.source_admitted").collect();
        assert_eq!(admitted.len(), 2, "two original packets; replay must not construct a new bound or resend");
        assert_eq!(admitted[1].source, admitted[0].source + 1);
        let sent: Vec<_> = recent_report.records.iter().filter_map(|e| match e.data {
            TraceData::Bound { present: true, anchor_unix_ns, latest_issue_unix_ns, remaining_ns }
                if e.phase == "scheduler.input.send.recent_bound" => Some((anchor_unix_ns, latest_issue_unix_ns, remaining_ns)),
            _ => None,
        }).collect();
        let dequeued: Vec<_> = recent_report.records.iter().filter_map(|e| match e.data {
            TraceData::Bound { present: true, anchor_unix_ns, latest_issue_unix_ns, remaining_ns }
                if e.phase == "scheduler.input.recent_bound" => Some((anchor_unix_ns, latest_issue_unix_ns, remaining_ns)),
            _ => None,
        }).collect();
        assert_eq!(sent.len(), 2); assert_eq!(dequeued.len(), 2);
        for (sent, dequeued) in sent.iter().zip(&dequeued) {
            assert_eq!(sent.0, dequeued.0, "same original Unix clock anchor");
            assert_eq!(sent.1, dequeued.1, "same original latest admissible issue time");
            assert!(dequeued.2.unwrap() <= sent.2.unwrap(), "waiting cannot extend the source deadline");
        }
        let source = admitted[1].source;
        assert_eq!(matching(&recent_report, "recent.source_admitted", source).len(), 1);
        assert_eq!(recent_report.records.iter().filter(|e| e.phase == "recent.scheduler_ack" && e.source == source
            && matches!(e.data, TraceData::SpanStart { .. })).count(), 1);
    }
}

fn telemetry_for(goal: &schema::GoalEnvelope) -> GuardedNavTelemetry {
    GuardedNavTelemetry { public_session_id: goal.session_id.clone(), owner_kind: goal.owner_kind,
        parent_request_id: goal.parent_request_id.clone(), goal_generation: goal.goal_generation,
        producer_runtime_generation: goal.producer_runtime_generation, ..Default::default() }
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit rejected-head continuation and original watermark controls"]
async fn guarded_ordinary_ack_rejected_then_valid_cancel_keeps_polling() {
    // Fresh real bridge: malformed and wrong-goal/high-generation heads must
    // both publish their actual rejections, then valid generation 1 progresses.
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-rejected-heads").await;
        h.block_renew().await;
        received(h.pose(2)).await;
        let malformed = send_guarded_test_body_observed(&h.cancels, "nuwax_msgs.CancelNavigation", vec![0xff]);
        let mut wrong = goal.clone(); wrong.parent_request_id.push_str("-wrong");
        let wrong_receipt = h.cancel(&wrong, 99);
        let valid = h.cancel(&goal, 1);
        received(malformed).await;
        h.state_when(|s| s.message.contains("REJECTED cancel:") && s.message.contains("CancelNavigation rejected:")).await;
        received(wrong_receipt).await;
        h.state_when(|s| s.message.contains("REJECTED cancel:") && s.message.contains("identity does not match")).await;
        received(valid).await;
        h.zero(&goal, 1).await;
        assert_gate_interrupted(h, Gate::Renew);
        assert_eq!(h.count("zero:"), 1);
    })).await;
    assert_abc(&report);
    assert_eq!(report.records.iter().filter(|e| e.phase == A).count(), 1);
    assert_eq!(matching(&report, "scheduler.input.reply.ok", 2_000_000_000).len(), 1);

    // Separate helper-only watermark control. Seeded prior generation is stated
    // explicitly; it is not a claim that fresh bridge history created it. The
    // queued ordinary request, cancellation API and fence are still real.
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-stale-watermark").await;
        h.block_renew().await;
        let (sender, mut cancels) = guarded_test_stream();
        let stale = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 1));
        let valid = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 4));
        let mut watermark = Some(GuardedCancelGenerationWatermark { goal: goal.clone(), generation: 3 });
        let mut arbiter = GuardedNavArbiter::default();
        let telemetry = telemetry_for(&goal);
        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        pose.stamp = Timestamp::from_unix(2, 0);
        let result = await_guarded_active_input(h.scheduler.update_odometry(pose), &h.node, &h.transport,
            &h.scheduler, &mut cancels, Some(&goal), &mut watermark, &mut arbiter, &telemetry).await;
        received(stale).await; received(valid).await;
        assert_eq!(result.input, Ok(()));
        let completion = result.cancel.expect("successful original Cancel ingress").expect("valid successor was consumed");
        assert_eq!(completion.result, Ok(()));
        assert_eq!(completion.cancel_generation, 4);
        assert_eq!(watermark.as_ref().unwrap().generation, 4);
        assert!(planner_goal_identity_matches(&watermark.as_ref().unwrap().goal, &goal));
        h.state_when(|s| s.message.contains("stale cancel generation 1; highest observed generation is 3")).await;
        assert_gate_interrupted(h, Gate::Renew);
        assert_eq!(h.count("zero:"), 1);
    })).await;
    assert_abc(&report);
    assert_eq!(report.records.iter().filter(|e| e.phase == A).count(), 1);
}

#[derive(Clone, Copy, Debug)]
enum PairOrder { InputFirst, CancelFirst, BothReady }
async fn pair_control(order: PairOrder, expected_input: Result<(), RuntimeError>, cancel_fails: bool) {
    let report = run(move |h| Box::pin(async move {
        let goal = h.activate("ordinary-result-pair").await;
        h.probe.arm(Gate::Fence);
        h.probe.fail_fence.store(cancel_fails, AtomicOrder::SeqCst);
        let (sender, mut cancels) = guarded_test_stream();
        let receipt = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 1));
        let (release, input_result) = oneshot::channel();
        let started = Arc::new(AtomicUsize::new(0));
        let completed = Arc::new(AtomicUsize::new(0));
        let started_in = started.clone(); let completed_in = completed.clone();
        let input = async move {
            started_in.fetch_add(1, AtomicOrder::SeqCst);
            let result = input_result.await.expect("one controlled original input-result owner");
            completed_in.fetch_add(1, AtomicOrder::SeqCst);
            result
        };
        let mut watermark = None; let mut arbiter = GuardedNavArbiter::default(); let telemetry = telemetry_for(&goal);
        let helper = await_guarded_active_input(input, &h.node, &h.transport, &h.scheduler,
            &mut cancels, Some(&goal), &mut watermark, &mut arbiter, &telemetry);
        tokio::pin!(helper);
        assert!(futures::poll!(&mut helper).is_pending());
        received(receipt).await;
        h.probe.wait(MotionEvent::Enter(Gate::Fence)).await;
        assert_eq!(started.load(AtomicOrder::SeqCst), 1);
        assert_eq!(completed.load(AtomicOrder::SeqCst), 0);
        match order {
            PairOrder::InputFirst => {
                release.send(expected_input.clone()).unwrap();
                assert!(futures::poll!(&mut helper).is_pending(), "input alone cannot discard admitted Cancel");
                assert_eq!(completed.load(AtomicOrder::SeqCst), 1);
                h.probe.release(Gate::Fence);
            }
            PairOrder::CancelFirst => {
                h.probe.release(Gate::Fence);
                h.status_when(|s| if cancel_fails { s.phase == SchedulerPhase::Failed } else { s.last_terminal == Some(TerminalOutcome::Cancelled) }).await;
                assert!(futures::poll!(&mut helper).is_pending(), "Cancel alone cannot discard the controlled input result");
                assert_eq!(completed.load(AtomicOrder::SeqCst), 0);
                release.send(expected_input.clone()).unwrap();
            }
            PairOrder::BothReady => {
                h.probe.release(Gate::Fence);
                h.status_when(|s| if cancel_fails { s.phase == SchedulerPhase::Failed } else { s.last_terminal == Some(TerminalOutcome::Cancelled) }).await;
                // The helper has not been polled since gate entry. The original
                // Cancel result and controlled input are both ready at its next poll.
                release.send(expected_input.clone()).unwrap();
            }
        }
        let settled = tokio::time::timeout(WATCHDOG, &mut helper).await.expect("both exact results settle");
        assert_eq!(settled.input, expected_input);
        let completion = settled.cancel.expect("original Cancel ingress").expect("admitted Cancel retained");
        assert!(planner_goal_identity_matches(&completion.active_goal, &goal));
        assert_eq!(completion.cancel_generation, 1);
        if cancel_fails {
            assert_eq!(completion.result, Err(RuntimeError::StopUnconfirmed("ordinary-ack controlled stop remains unconfirmed".into())));
        } else { assert_eq!(completion.result, Ok(())); }
        assert_eq!(started.load(AtomicOrder::SeqCst), 1);
        assert_eq!(completed.load(AtomicOrder::SeqCst), 1);
        h.probe.fail_fence.store(false, AtomicOrder::SeqCst);
    })).await;
    assert!(first(&report, A) < first(&report, B));
    eprintln!("ORDINARY_ACK_RESULT_PAIR order={order:?} controlled_input=true real_cancel=true cancel_fails={cancel_fails}");
}
#[tokio::test(flavor = "current_thread")]
#[ignore = "three fixed helper result-pair orders, not a scheduler error-injection matrix"]
async fn guarded_ordinary_ack_result_pair_retains_both_orders() {
    // These exact input errors are explicitly controlled result values, not
    // fabricated claims of real scheduler failures or physical stop outcomes.
    pair_control(PairOrder::InputFirst, Ok(()), false).await;
    pair_control(PairOrder::CancelFirst, Err(RuntimeError::StaleInput("controlled ordinary rejection".into())), false).await;
    pair_control(PairOrder::BothReady, Err(RuntimeError::SchedulerStopped), true).await;
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "real bridge replacement remains behind both old results"]
async fn guarded_ordinary_ack_cancel_settlement_precedes_new_goal() {
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-old-session").await;
        h.block_renew().await;
        h.probe.arm(Gate::Fence);
        received(h.path(&sealed_path(&goal))).await;
        let next = rollover_goal(&goal, "ordinary-new-session", 8, 1);
        let mut next_receipt = h.goal(&next);
        let mut late_path = sealed_path(&goal); late_path.path_revision = 2;
        let mut late_receipt = h.path(&late_path);
        received(h.cancel(&goal, 1)).await;
        h.probe.wait(MotionEvent::Enter(Gate::Fence)).await;
        assert!(matches!(next_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
        assert!(matches!(late_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
        assert_eq!(h.count("acquire:"), 1);
        assert_eq!(h.scheduler.status().phase, SchedulerPhase::Cancelling);
        h.sink().mark(CHECKPOINT, 0);
        h.probe.release(Gate::Fence);
        let old_terminal = h.zero(&goal, 1).await;
        assert_eq!(old_terminal.path_revision, 0);
        received(next_receipt).await;
        received(late_receipt).await;
        h.status_when(|s| s.session_id.as_deref() == Some(next.session_id.as_str())).await;
        assert_eq!(h.count("acquire:"), 2);
        assert_eq!(h.count("twist:"), 0);
        assert_eq!(h.scheduler.status().path_revision, 0);
        assert_eq!(h.scheduler.status().last_terminal, None);
        // A real next-session state verifies that stale old-path telemetry did
        // not overwrite the replacement identity after the old terminal batch.
        h.state_when(|s| s.session_id == next.session_id && s.producer_runtime_generation == next.producer_runtime_generation
            && s.goal_generation == next.goal_generation && s.path_revision == 0 && s.cancel_generation == 0).await;
    })).await;
    assert_abc(&report);
    let old_reply = matching(&report, "scheduler.input.reply.error", 0)[0];
    let starts: Vec<_> = report.records.iter().enumerate().filter_map(|(i, e)|
        (e.phase == "scheduler.send.start_session" && matches!(e.data, TraceData::SpanStart { .. })).then_some(i)).collect();
    assert_eq!(starts.len(), 2);
    assert!(first(&report, "ordinary_ack_test.fence_proof") < old_reply && old_reply < starts[1]);
}

async fn coordinate_original_tasks(h: &mut Harness) -> NavStackResult<()> {
    // Keep join handles in Harness while the original coordinator is pending.
    // A failed scenario/watchdog still leaves cleanup able to release and join.
    let loop_task = &mut h.loop_task;
    let scheduler_task = &mut h.scheduler_task;
    let handle = h.scheduler.clone();
    coordinate_guarded_scheduler(
        async {
            let result = loop_task.as_mut().expect("one original bridge task owner").await;
            // Retire a consumed JoinHandle before any later coordinator await.
            // A watchdog cleanup must only poll the still-pending owner.
            loop_task.take();
            result.map_err(|e| NavStackError::Internal(format!("bridge join: {e}")))?
        },
        async {
            let result = scheduler_task.as_mut().expect("one original scheduler task owner").await;
            scheduler_task.take();
            result.map_err(|e| NavStackError::Internal(format!("scheduler join: {e}")))?
                .map_err(|e| NavStackError::Internal(format!("scheduler task: {e}")))
        },
        || async { handle.shutdown().await.map_err(|e| NavStackError::Internal(format!("scheduler shutdown: {e}"))) },
    ).await
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "admitted shutdown settlement and actual closed-Cancel fatal branch"]
async fn guarded_ordinary_ack_shutdown_keeps_admitted_results() {
    // Already admitted Cancel: shutdown is observed only after the bridge has
    // collected its original ordinary ACK and real cancellation result.
    let admitted = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-shutdown-admitted").await;
        h.block_renew().await;
        h.probe.arm(Gate::Fence);
        received(h.pose(2)).await;
        received(h.cancel(&goal, 1)).await;
        h.probe.wait(MotionEvent::Enter(Gate::Fence)).await;
        h.shutdown.take().unwrap().send(()).expect("original bridge shutdown receiver");
        assert!(!h.loop_task.as_ref().unwrap().is_finished());
        assert_eq!(h.scheduler.status().pose_source_timestamp_ns, 1_000_000_000);
        h.sink().mark(CHECKPOINT, 0);
        h.probe.release(Gate::Fence);
        h.zero(&goal, 1).await;
        assert_eq!(h.scheduler.status().pose_source_timestamp_ns, 2_000_000_000);
        coordinate_original_tasks(h).await.expect("original coordinator collects both completed tasks");
        assert_eq!(h.count("zero:"), 1, "outer shutdown cannot fence an already cancelled session again");
        assert_gate_interrupted(h, Gate::Renew);
    })).await;
    assert_abc(&admitted);
    assert_eq!(matching(&admitted, "scheduler.input.reply.ok", 2_000_000_000).len(), 1);

    // Actual source EOF while an ordinary ACK is blocked. This directly
    // exercises the helper's common fatal branch, not an injected output fault.
    for full_queue in [false, true] {
        let fatal = run(move |h| Box::pin(async move {
            let goal = h.activate("ordinary-fatal-stream").await;
            h.block_renew().await;
            let handle = h.scheduler.clone();
            let mut fillers = Vec::new();
            if full_queue {
                for index in 0..32 {
                    let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
                    pose.stamp = Timestamp::from_unix(100 + index, 0);
                    let mut input = Box::pin(handle.update_odometry(pose));
                    assert!(futures::poll!(&mut input).is_pending());
                    fillers.push(input);
                }
            }
            h.sink().mark("ordinary_ack_test.fatal_input_begin", 0);
            if full_queue { received(h.path(&sealed_path(&goal))).await; }
            else { received(h.pose(2)).await; }
            h.sink().mark(CHECKPOINT, 0);
            h.cancels.close_channel();
            let result = coordinate_original_tasks(h).await.expect_err("real required Cancel source EOF survives coordination");
            assert!(result.to_string().contains("cancel LCM stream closed"), "{result}");
            let results = futures::future::join_all(fillers).await;
            assert_eq!(results.len(), if full_queue { 32 } else { 0 });
            assert!(results.iter().all(|result| *result == Err(RuntimeError::SchedulerStopped)),
                "all original filler replies must close when the actual scheduler exits: {results:?}");
            assert_eq!(h.scheduler.status().last_terminal, Some(TerminalOutcome::Shutdown));
            assert_eq!(h.count("zero:"), 1);
            assert_eq!(h.count("acquire:"), 1);
            assert_gate_interrupted(h, Gate::Renew);
        })).await;
        assert!(!fatal.records.iter().any(|e| e.phase == A), "source EOF must never fabricate Cancel acceptance");
        if full_queue {
            let begin = first(&fatal, "ordinary_ack_test.fatal_input_begin");
            assert_eq!(matching(&fatal, "scheduler.input.send.begin", 0).iter().filter(|i| **i > begin).count(), 1);
            assert_eq!(matching(&fatal, "scheduler.input.send.accepted", 0).iter().filter(|i| **i > begin).count(), 0,
                "the exact 33rd original send must fail closed, never enqueue after shutdown");
            assert_eq!(fatal.records.iter().skip(begin).filter(|e| e.phase == "scheduler.send.path"
                && matches!(e.data, TraceData::SpanEnd { .. })).count(), 1);
            // Original send_input has no send.closed mark: failed bounded send
            // returns SchedulerStopped before the reply instrumented branch.
            for index in 0..32 {
                let source = (100 + index) * 1_000_000_000;
                assert_eq!(matching(&fatal, "scheduler.input.send.accepted", source).len(), 1);
                assert_eq!(matching(&fatal, "scheduler.input.reply.closed", source).len(), 1);
            }
        } else {
            assert_eq!(matching(&fatal, "scheduler.input.reply.closed", 2_000_000_000).len(), 1);
        }
    }

    // Same fatal branch with a real scheduler stop-unconfirmed result: helper
    // preserves both its actual SchedulerStopped input and original source error.
    // The scheduler task separately preserves its exact shutdown failure.
    run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-fatal-unconfirmed").await;
        h.block_renew().await;
        h.probe.fail_fence.store(true, AtomicOrder::SeqCst);
        let (sender, mut cancels) = guarded_test_stream(); drop(sender);
        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map); pose.stamp = Timestamp::from_unix(2, 0);
        let mut watermark = None; let mut arbiter = GuardedNavArbiter::default(); let telemetry = telemetry_for(&goal);
        let result = await_guarded_active_input(h.scheduler.update_odometry(pose), &h.node, &h.transport,
            &h.scheduler, &mut cancels, Some(&goal), &mut watermark, &mut arbiter, &telemetry).await;
        assert_eq!(result.input, Err(RuntimeError::SchedulerStopped));
        let error = result.cancel.err().expect("original source error retained");
        assert!(error.to_string().contains("cancel LCM stream closed"));
        let task = h.scheduler_task.as_mut().unwrap().await;
        h.scheduler_task.take();
        let task = task.expect("failed scheduler task joined");
        assert_eq!(task, Err(RuntimeError::StopUnconfirmed("ordinary-ack controlled stop remains unconfirmed".into())));
        assert_eq!(h.scheduler.status().last_terminal, None);
        assert!(h.scheduler.status().last_zero_ack_id.is_none());
        assert_eq!(h.count("zero:"), 0);
        // Observe this independent original bridge's actual source-closure
        // exit before cleanup sends shutdown; no scheduling-dependent outcome.
        let bridge = tokio::time::timeout(WATCHDOG, h.loop_task.as_mut().unwrap()).await
            .expect("original bridge observes closed scheduler status");
        h.loop_task.take();
        let bridge = bridge.expect("original bridge joined");
        assert!(bridge.expect_err("status source closed").to_string().contains("closed"));
        h.probe.fail_fence.store(false, AtomicOrder::SeqCst);
        assert_gate_interrupted(h, Gate::Renew);
    })).await;
    eprintln!("ORDINARY_ACK_FATAL actual_cancel_stream_eof=true rejection_publication_transport_fault_injected=false");
}

#[derive(Clone, Copy, Debug)]
enum Eligibility { ActiveReady, NoActive, Terminal, SessionMismatch }
async fn eligibility_control(case: Eligibility) {
    run(move |h| Box::pin(async move {
        let goal = h.activate("ordinary-no-prefetch").await;
        if matches!(case, Eligibility::Terminal) {
            received(h.cancel(&goal, 1)).await;
            h.zero(&goal, 1).await;
        }
        let (sender, mut cancels) = guarded_test_stream();
        let mut receipt = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 2));
        let mut watermark = None; let mut arbiter = GuardedNavArbiter { cursor: GuardedNavSource::Path };
        let telemetry = telemetry_for(&goal);
        let mut foreign = goal.clone(); foreign.session_id.push_str("-foreign");
        let active = match case { Eligibility::NoActive => None, Eligibility::SessionMismatch => Some(&foreign), _ => Some(&goal) };
        let before = h.count("zero:");
        if matches!(case, Eligibility::ActiveReady) {
            let result = await_guarded_active_input(std::future::ready(17_u32), &h.node, &h.transport,
                &h.scheduler, &mut cancels, active, &mut watermark, &mut arbiter, &telemetry).await;
            assert_eq!(result.input, 17); assert!(matches!(result.cancel, Ok(None)));
        } else {
            let (release, released) = oneshot::channel();
            let helper = await_guarded_active_input(async { released.await.expect("one excluded input owner"); 17_u32 },
                &h.node, &h.transport, &h.scheduler, &mut cancels, active, &mut watermark, &mut arbiter, &telemetry);
            tokio::pin!(helper);
            assert!(futures::poll!(&mut helper).is_pending());
            assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
            release.send(()).unwrap();
            let result = helper.await;
            assert_eq!(result.input, 17); assert!(matches!(result.cancel, Ok(None)));
        }
        assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
        assert!(watermark.is_none()); assert_eq!(arbiter.cursor, GuardedNavSource::Path);
        assert_eq!(h.count("zero:"), before);
        let remaining = cancels.next().now_or_never().flatten().expect("same unconsumed Cancel remains").unwrap();
        assert_eq!(schema::decode_cancel_navigation_body(&remaining.payload).unwrap().cancel_generation, 2);
        received(receipt).await;
    })).await;
}
#[tokio::test(flavor = "current_thread")]
#[ignore = "ready/ineligible no-prefetch plus original serial StartSession control"]
async fn guarded_ordinary_ack_ready_and_inactive_keep_cancel_order() {
    for case in [Eligibility::ActiveReady, Eligibility::NoActive, Eligibility::Terminal, Eligibility::SessionMismatch] {
        eligibility_control(case).await;
    }
    // Original real bridge acquire is a deliberate negative control. Cancel
    // stays in its original ingress stream until StartSession/acquire returns.
    let report = run(|h| Box::pin(async move {
        received(h.pose(1)).await;
        h.status_when(|s| s.pose_source_timestamp_ns == 1_000_000_000).await;
        received(h.map(11)).await;
        h.status_when(|s| s.costmap_source_timestamp_ns == 11_000_000_000).await;
        let goal = sealed_goal("ordinary-start-exclusion");
        h.probe.arm(Gate::Acquire);
        received(h.goal(&goal)).await;
        h.probe.wait(MotionEvent::Enter(Gate::Acquire)).await;
        let mut cancel_receipt = h.cancel(&goal, 1);
        // Explicit original acquire-entry and current-thread checkpoint. No
        // timer-based absence claim, helper wrapper or new StartSession owner.
        assert!(matches!(cancel_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
        assert_eq!(h.count("acquire:"), 0);
        h.sink().mark(CHECKPOINT, 0);
        h.probe.release(Gate::Acquire);
        received(cancel_receipt).await;
        h.zero(&goal, 1).await;
        assert_eq!(h.count("acquire:"), 1);
        assert_eq!(h.count("zero:"), 1);
        assert!(h.probe.events().contains(&MotionEvent::Return(Gate::Acquire)));
    })).await;
    assert!(first(&report, CHECKPOINT) < first(&report, A));
    let start_reply = matching(&report, "scheduler.input.reply.ok", 0)[0];
    assert!(start_reply < first(&report, A), "StartSession ACK is outside the new cancellation window");
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "actual unconfirmed MotionPort stop must not become a cancellation receipt"]
async fn guarded_ordinary_ack_unconfirmed_stop_never_publishes_cancelled() {
    let report = run(|h| Box::pin(async move {
        let goal = h.activate("ordinary-unconfirmed-stop").await;
        h.block_renew().await;
        h.probe.fail_fence.store(true, AtomicOrder::SeqCst);
        received(h.pose(2)).await;
        received(h.cancel(&goal, 1)).await;
        let rejected = h.state_when(|s| s.message.contains("REJECTED cancel:")
            && s.message.contains("navigation stop confirmation pending")).await;
        assert!(rejected.zero_ack_id.is_empty()); assert_eq!(rejected.zero_applied_monotonic_ns, 0);
        assert!(!rejected.message.starts_with("CANCELLED:"));
        h.status_when(|s| s.pose_source_timestamp_ns == 2_000_000_000).await;
        let status = h.scheduler.status();
        assert_eq!(status.last_terminal, None);
        assert_eq!(status.session_id.as_deref(), Some(goal.session_id.as_str()));
        assert!(status.last_zero_ack_id.is_none()); assert_eq!(status.last_fence_monotonic_ns, 0);
        assert_eq!(h.count("zero:"), 0);
        assert_gate_interrupted(h, Gate::Renew);
        // The actual rejection publication is the causal observation boundary.
        // No cancelled Bool or zero-bearing state may have preceded it.
        assert!(h.cancelled.next().now_or_never().is_none());
        while let Some(frame) = h.states.next().now_or_never().flatten() {
            let state = schema::decode_navigation_state_body(&frame.unwrap().payload).unwrap();
            assert!(!state.message.starts_with("CANCELLED:"));
            assert!(state.zero_ack_id.is_empty());
        }
        h.sink().mark("ordinary_ack_test.unconfirmed_checkpoint", 0);
        // Normal cleanup may now obtain the first confirmed fence. It is not
        // credited to the failed Cancel and is intentionally after assertions.
        h.probe.fail_fence.store(false, AtomicOrder::SeqCst);
    })).await;
    assert!(first(&report, A) < first(&report, B));
    assert_eq!(matching(&report, "scheduler.input.reply.ok", 2_000_000_000).len(), 1);
    assert!(first(&report, "ordinary_ack_test.unconfirmed_checkpoint") < first(&report, "ordinary_ack_test.fence_proof"));
}
