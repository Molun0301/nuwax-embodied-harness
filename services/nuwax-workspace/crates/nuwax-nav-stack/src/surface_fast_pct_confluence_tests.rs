// Tests only. PCT, publications, worker contexts and scheduler are real.
// The single synthetic planner action is Command(nonzero) -> ReplanRequired.
// This is feedback-timing qualification, not geometric-refusal qualification.
use super::*;
use nuwax_nav_runtime::{AppliedProof, BoundedTwistOutcome, CommandDispatchBound,
    ExecutionMap, ExecutionPath, FenceProof, LeaseProof, MotionPortError,
    NavMotionAdmission, NavMotionPort, NavPlanner, PlannerStep, RuntimeError};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::GoalOptions;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as Order};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(in crate::lcm_dataflow::surface_dataflow) enum Event {
    Dispatch { complete: bool },
    Computed { call: usize, ok: bool, gate: usize },
    Refusal(usize),
    Gate(usize),
    Reuse { complete: bool },
    Held,
}

pub(in crate::lcm_dataflow::surface_dataflow) struct Probe {
    events: Mutex<Vec<Event>>,
    changed: tokio::sync::watch::Sender<usize>,
    pub(super) normal: AtomicUsize,
    pub(super) quick: AtomicUsize,
    refusals: AtomicUsize,
    pending_refusal_zero: AtomicBool,
    armed: AtomicBool,
    gate: AtomicUsize,
    releases: Mutex<BTreeMap<usize, oneshot::Sender<()>>>,
    permit: std_mpsc::Sender<usize>,
    permits: Mutex<std_mpsc::Receiver<usize>>,
}
impl Probe {
    fn new() -> Arc<Self> {
        let (permit, permits) = std_mpsc::channel();
        Arc::new(Self { events: Mutex::new(Vec::new()),
            changed: tokio::sync::watch::channel(0).0,
            normal: AtomicUsize::new(0), quick: AtomicUsize::new(0),
            refusals: AtomicUsize::new(0), pending_refusal_zero: AtomicBool::new(false), armed: AtomicBool::new(false),
            gate: AtomicUsize::new(0), releases: Mutex::new(BTreeMap::new()),
            permit, permits: Mutex::new(permits) })
    }
    pub(in crate::lcm_dataflow::surface_dataflow) fn record(&self, event: Event) {
        let count = {
            let mut events = self.events.lock().unwrap();
            if events.len() >= 512 { None }
            else { events.push(event); Some(events.len()) }
        };
        // Fail after dropping the mutex: a watchdog must not poison observation.
        let count = count.expect("bounded confluence observation overflow");
        self.changed.send_replace(count);
    }
    async fn wait(&self, predicate: impl Fn(&[Event]) -> bool) {
        let mut changed = self.changed.subscribe();
        // Timeout is a failure watchdog only. Success requires a causal event.
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                if predicate(&self.events.lock().unwrap()) { return; }
                changed.changed().await.expect("probe remains open");
            }
        }).await.unwrap_or_else(|_| {
            let events = self.events.lock().unwrap().clone();
            panic!("confluence event missing: {events:?}");
        });
    }
    fn release(&self, index: usize) {
        if let Some(sender) = self.releases.lock().unwrap().remove(&index) {
            let _ = sender.send(());
        }
    }
    fn release_all(&self) {
        self.armed.store(false, Order::SeqCst);
        for (_, sender) in std::mem::take(&mut *self.releases.lock().unwrap()) {
            let _ = sender.send(());
        }
        // Wake a mistakenly scheduled third computation so normal shutdown can
        // drain it; it still invokes real PCT and returns its original result.
        let _ = self.permit.send(0);
    }
}
struct ReleaseOnDrop(Arc<Probe>);
impl Drop for ReleaseOnDrop { fn drop(&mut self) { self.0.release_all(); } }

pub(super) struct RefuseRealCommand<P> { pub(super) inner: P, pub(super) probe: Arc<Probe> }
impl<P: NavPlanner> NavPlanner for RefuseRealCommand<P> {
    fn refresh_execution_context(&mut self, a: Twist2) -> Option<Arc<ActualExecutionContext>> { self.inner.refresh_execution_context(a) }
    fn take_step_execution_context(&mut self, a: Twist2) -> Option<Arc<ActualExecutionContext>> { self.inner.take_step_execution_context(a) }
    fn take_command_dispatch_bound(&mut self) -> Option<CommandDispatchBound> { self.inner.take_command_dispatch_bound() }
    fn revalidate_selected_command(&mut self, c: Twist2) -> Option<PlannerStep> { self.inner.revalidate_selected_command(c) }
    fn revalidate_selected_command_at_current_state(&mut self, c: Twist2) -> Option<PlannerStep> { self.inner.revalidate_selected_command_at_current_state(c) }
    fn set_goal_log_context(&mut self, id: &str) { self.inner.set_goal_log_context(id); }
    fn configure_boundary(&mut self, b: &[[f64; 2]]) { self.inner.configure_boundary(b); }
    fn configure_goal(&mut self, g: &PoseStamped, o: GoalOptions) -> Result<(), RuntimeError> { self.inner.configure_goal(g, o) }
    fn replace_path(&mut self, p: Path2D) { self.inner.replace_path(p); }
    fn replace_path_with_orientations(&mut self, p: Path2D, y: Option<Vec<f64>>) { self.inner.replace_path_with_orientations(p, y); }
    fn replace_execution_path(&mut self, p: ExecutionPath, y: Option<Vec<f64>>, g: PoseStamped) -> Result<(), RuntimeError> { self.inner.replace_execution_path(p, y, g) }
    fn update_odometry(&mut self, p: PoseStamped) { self.inner.update_odometry(p); }
    fn update_costmap(&mut self, m: CostMap2D) { self.inner.update_costmap(m); }
    fn update_execution_map(&mut self, m: ExecutionMap) -> Result<(), RuntimeError> { self.inner.update_execution_map(m) }
    fn invalidate_execution_map(&mut self) { self.inner.invalidate_execution_map(); }
    fn observe_applied_command(&mut self, c: Twist2) { self.inner.observe_applied_command(c); }
    fn execution_wait_reason(&self) -> Option<String> { self.inner.execution_wait_reason() }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> { self.inner.local_path_for_display() }
    fn prepare_control_step(&mut self, w: f64, continuing: bool) -> Option<tokio::sync::watch::Receiver<()>> { self.inner.prepare_control_step(w, continuing) }
    fn step(&mut self, width: f64) -> PlannerStep {
        let step = self.inner.step(width);
        if matches!(&step, PlannerStep::Command(command) if *command != Twist2::ZERO) {
            let refusal = self.probe.refusals.fetch_add(1, Order::SeqCst) + 1;
            self.probe.pending_refusal_zero.store(true, Order::SeqCst);
            self.probe.record(Event::Refusal(refusal));
            PlannerStep::ReplanRequired("TEST synthetic refusal of a real worker Command".into())
        } else { step }
    }
    fn stop(&mut self) { self.inner.stop(); }
}

pub(super) struct GateMotion { pub(super) inner: RecordingMotion, pub(super) probe: Arc<Probe> }
#[async_trait::async_trait]
impl NavMotionPort for GateMotion {
    fn renewal_due_at(&self) -> Option<tokio::time::Instant> {
        self.probe.armed.load(Order::SeqCst).then(tokio::time::Instant::now)
    }
    async fn acquire(&mut self, id: &str, a: &NavMotionAdmission) -> Result<LeaseProof, MotionPortError> { self.inner.acquire(id, a).await }
    async fn renew(&mut self, id: &str) -> Result<(), MotionPortError> {
        if self.probe.armed.swap(false, Order::SeqCst) {
            let index = self.probe.refusals.load(Order::SeqCst);
            let (send, receive) = oneshot::channel();
            self.probe.releases.lock().unwrap().insert(index, send);
            self.probe.gate.store(index, Order::SeqCst);
            self.probe.record(Event::Gate(index));
            // Gate is already entered before permitting the next real PCT call.
            let _ = self.probe.permit.send(index);
            let _ = receive.await;
            self.probe.gate.store(0, Order::SeqCst);
        }
        self.inner.renew(id).await
    }
    async fn submit_twist(&mut self, id: &str, twist: Twist2) -> Result<AppliedProof, MotionPortError> {
        assert_eq!(twist, Twist2::ZERO, "the synthetic refusal must never reach motion as nonzero");
        let result = self.inner.submit_twist(id, twist).await;
        // The scheduler may renew inside apply_controller_twist, before its
        // original ZERO submission. Arm only after that real receipt returns;
        // the outer renewal branch then gates after refusal status publication.
        if result.is_ok() && self.probe.pending_refusal_zero.swap(false, Order::SeqCst) {
            self.probe.armed.store(true, Order::SeqCst);
        }
        result
    }
    async fn submit_twist_bounded(&mut self, id: &str, twist: Twist2, bound: Option<CommandDispatchBound>) -> Result<BoundedTwistOutcome, MotionPortError> {
        assert!(bound.is_none(), "refusal ZERO must not inherit the candidate command bound");
        self.submit_twist(id, twist).await.map(BoundedTwistOutcome::Applied)
    }
    async fn revoke_and_zero(&mut self, id: &str) -> Result<FenceProof, MotionPortError> { self.inner.revoke_and_zero(id).await }
    async fn priority_revoke_and_zero(&mut self, id: &str) -> Result<FenceProof, MotionPortError> { self.inner.priority_revoke_and_zero(id).await }
}

async fn status_when(h: &Harness, predicate: impl Fn(&nuwax_nav_runtime::SchedulerStatus) -> bool) {
    let mut status = h.scheduler.subscribe_status();
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if predicate(&status.borrow()) { return; }
            status.changed().await.expect("scheduler status remains open");
        }
    }).await.unwrap_or_else(|_| panic!("scheduler condition missing: {:?}", h.scheduler.status()));
}

#[cfg(feature = "surface-recovery-qualification")]
#[tokio::test]
async fn real_fast_pct_publication_before_wait_tick_converges_after_injected_refusal() {
    let probe = Probe::new();
    let observed = probe.clone();
    let normal: SurfacePathComputer = Arc::new(move |surface, pose, goal, configured| {
        // Only the first call can start before a refusal. Subsequent calls wait
        // for the original scheduler's renewal gate; no route is fabricated.
        if observed.normal.load(Order::SeqCst) > 0 {
            observed.permits.lock().unwrap().recv_timeout(Duration::from_secs(5))
                .expect("scheduler must gate before the replacement PCT call");
        }
        let call = observed.normal.fetch_add(1, Order::SeqCst) + 1;
        let result = body_path(surface, pose, goal, configured);
        observed.record(Event::Computed { call, ok: result.is_ok(), gate: observed.gate.load(Order::SeqCst) });
        result
    });
    let observed = probe.clone();
    let quick: SurfacePathComputer = Arc::new(move |surface, pose, goal, configured| {
        observed.quick.fetch_add(1, Order::SeqCst);
        body_path_with_budget(surface, pose, goal, configured, 0)
    });
    // Exact existing flat-map/calibration fixture. No tuning of PCT budgets,
    // body dimensions, terrain, tolerances, scheduler cadence or deadlines.
    let mut configured = config();
    configured.follower.local_planning_enabled = false;
    configured.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        self_collision: None,
        footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m: -0.05, z_max_m: 0.2, source: "OFF soft-refresh owner fixture".into(),
    }), None).unwrap();
    let mut harness = Harness::start_with_components_and_feedback_probe(normal,
        Duration::from_millis(25), configured, true, Some(quick), None,
        nuwax_nav_runtime::SchedulerConfig::default(), Some(probe.clone())).await;
    // Declared after Harness: releases channels before Harness::drop joins owners.
    let cleanup = ReleaseOnDrop(probe.clone());
    status_when(&harness, |status| status.planner_inputs_fresh).await;
    let mut maps = harness.hub.events.subscribe();
    let frozen = {
        let mut feed = harness.feed.lock().unwrap();
        feed.map_enabled = false;
        feed.last_published.clone().expect("ready implies a source publication")
    };
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if harness.hub.latest.lock().unwrap().as_ref()
                .is_some_and(|map| map.authority.cloud_revision == frozen.cloud_revision && map.snapshot.is_ok()) { break; }
            maps.recv().await.expect("map publication remains open");
        }
    }).await.expect("last pre-freeze map must be installed");
    status_when(&harness, |s| s.costmap_source_timestamp_ns == frozen.source_timestamp_ns).await;
    assert!(harness.hub.execution_feedback.borrow().is_none());
    let goal = goal_for(&frozen, 1, [1.55, 1.55, 2.5]);
    harness.goal(&goal).await;
    let first = harness.path_for(&goal).await;
    let mut published = first.clone();
    let mut refusal_index = 1;
    let mut published_calls = 1;
    let mut same_actual = None;
    let stable_calls = loop {
        probe.wait(|events| events.contains(&Event::Gate(refusal_index))).await;
        let state = harness.scheduler.status();
        let feedback = state.surface_replan_feedback.unwrap();
        assert!(feedback.zero_confirmed_rejection);
        assert!(feedback.rejected_context.is_some());
        assert_eq!(feedback.current_context, feedback.rejected_context);
        assert_eq!(feedback.current_context.as_ref().unwrap().applied, Twist2::ZERO);
        if let Some(previous) = &same_actual { assert_eq!(&feedback.current_context, previous); }
        else { same_actual = Some(feedback.current_context.clone()); }
        let completed = published_calls;
        // State and feedback channels may legally race. Missing-context captures
        // can perform extra real work; the first complete capture must converge.
        probe.wait(|events| events.contains(&Event::Reuse { complete: true })
            || events.contains(&Event::Computed { call: completed + 1, ok: true, gate: refusal_index })).await;
        let reused = probe.events.lock().unwrap().contains(&Event::Reuse { complete: true });
        if reused {
            let events = probe.events.lock().unwrap().clone();
            let captures: Vec<_> = events.iter().filter_map(|event| match event {
                Event::Dispatch { complete } => Some(*complete), _ => None,
            }).collect();
            assert_eq!(captures.first(), Some(&false),
                "initial missing context must dispatch real PCT, never gate it");
            assert_eq!(captures.last(), Some(&true));
            assert_eq!(captures.len(), completed);
            assert!(completed >= 2, "the initial incomplete memo cannot hold a refusal");
            // If an earlier complete success existed, extra work beyond it is a
            // regression, not an allowed missing-context fallback.
            assert!(captures[..captures.len()-1].iter().all(|complete| !complete));
            break completed;
        }
        let next = harness.path_for(&goal).await;
        assert_eq!(next.path_revision, published.path_revision + 1);
        assert_eq!(probe.gate.load(Order::SeqCst), refusal_index,
            "real replacement PCT + original publish must finish before the next session_tick");
        assert_eq!(harness.scheduler.status().path_revision, published.path_revision as u64);
        published = next;
        published_calls += 1;
        probe.release(refusal_index);
        refusal_index += 1;
    };
    probe.wait(|events| events.iter().filter(|event| **event == Event::Held).count() >= 2).await;
    assert_eq!(probe.normal.load(Order::SeqCst), stable_calls);
    assert_eq!(probe.quick.load(Order::SeqCst), 0);
    let stopped = harness.scheduler.status();
    assert_eq!(stopped.phase, SchedulerPhase::AwaitingPath);
    assert_eq!(stopped.path_revision, published.path_revision as u64);
    assert_eq!(stopped.last_applied_twist, Some(Twist2::ZERO));
    let feedback = stopped.surface_replan_feedback.unwrap();
    assert!(feedback.zero_confirmed_rejection);
    assert_eq!(feedback.rejected_context, feedback.current_context);
    assert_eq!(Some(feedback.current_context), same_actual);
    eprintln!("FAST_PCT_CONFLUENCE real_normal_calls={stable_calls} real_quick_calls=0 initial_actual=None final_actual=Some held_ticks=2 refusal=synthetic real_worker_command=true");
    // Cancel uses the original urgent branch while renew is still pending.
    harness.cancel(&goal).await;
    status_when(&harness, |s| s.last_terminal == Some(TerminalOutcome::Cancelled)).await;
    drop(cleanup);
    harness.shutdown().await;
    assert_eq!(probe.normal.load(Order::SeqCst), stable_calls);
    assert_eq!(probe.quick.load(Order::SeqCst), 0);
}
