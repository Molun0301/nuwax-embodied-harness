// Included only inside #[cfg(all(test, feature = "guard-runtime"))] guarded_tests,
// under its existing surface-recovery-qualification feature. Review-only candidate.
// NOT executed. Replaces the original test include only after integration approval.
use nuwax_nav_runtime::qualification_timing::{TraceReport, TraceSession, TraceSink};

const ACK_READY: &str = "bridge_ack_test.real_ack_ready";
const GATE_HELD: &str = "bridge_ack_test.gate_held";
const GATE_DISABLED: &str = "bridge_ack_test.gate_disabled";
const ENQUEUED: &str = "bridge_ack_test.cancel_enqueued";
const CANCEL_READY_HELD: &str = "bridge_ack_test.cancel_result_ready_while_held";
const BEFORE_DELEGATE: &str = "bridge_ack_test.before_real_publish_delegate";
const OLD_STATE_RECEIVED: &str = "bridge_ack_test.old_batch_state_received";
const OLD_TWIST_RECEIVED: &str = "bridge_ack_test.old_batch_twist_received";
const DISTINCT_TAIL_TWIST: Twist2 = Twist2 { linear: Point2 { x: 0.37, y: 0.0 }, angular_z: 0.13 };
const CHECKPOINT: &str = "bridge_ack_test.closed_checkpoint";
const RELEASE_NORMAL: &str = "bridge_ack_test.release_normal";
const RELEASE_CLEANUP: &str = "bridge_ack_test.release_cleanup";
const GATE_RESUMED: &str = "bridge_ack_test.gate_resumed";
const DEQUEUED: &str = "bridge_ack_test.observed_stream_dequeued";
const DECODED: &str = "bridge_ack_test.cancel_decoded";
const API_BEGIN: &str = "bridge_ack_test.cancel_api_begin";
const API_OK: &str = "bridge_ack_test.cancel_api_ok";
const ZEROED: &str = "bridge_ack_test.correlated_zero_received";

struct Target {
    session_id: String,
    owner_kind: i32,
    parent_request_id: String,
    goal_generation: i64,
    producer_runtime_generation: i64,
    path_revision: i64,
    cancel_generation: i64,
    pose_source_timestamp_ns: i64,
    costmap_source_timestamp_ns: i64,
    trigger: TailTrigger,
}
impl Target {
    fn new(goal: &schema::GoalEnvelope, path_revision: i64, cancel_generation: i64) -> Self {
        Self {
            session_id: goal.session_id.clone(), owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
            path_revision, cancel_generation, pose_source_timestamp_ns: 2_000_000_000,
            costmap_source_timestamp_ns: 11_000_000_000, trigger: TailTrigger::Pose,
        }
    }
    fn for_trigger(mut self, trigger: TailTrigger) -> Self {
        self.trigger = trigger;
        self.pose_source_timestamp_ns = if trigger == TailTrigger::Pose { 2_000_000_000 } else { 1_000_000_000 };
        self.costmap_source_timestamp_ns = if trigger == TailTrigger::Costmap { 12_000_000_000 } else { 11_000_000_000 };
        self
    }
    fn publication(&self, status: &nuwax_nav_runtime::SchedulerStatus, t: &GuardedNavTelemetry,
        planner_observation: Option<bool>) -> bool {
        status.phase == SchedulerPhase::Controlling && status.last_terminal.is_none()
            && status.session_id.as_deref() == Some(self.session_id.as_str())
            && status.pose_source_timestamp_ns == self.pose_source_timestamp_ns
            && status.costmap_source_timestamp_ns == self.costmap_source_timestamp_ns
            && planner_observation.is_some() == (self.trigger != TailTrigger::Status)
            && i64::try_from(status.path_revision).ok() == Some(self.path_revision)
            && t.public_session_id == self.session_id && t.owner_kind == self.owner_kind
            && t.parent_request_id == self.parent_request_id && t.goal_generation == self.goal_generation
            && t.producer_runtime_generation == self.producer_runtime_generation
            && t.path_revision == self.path_revision && t.cancel_generation == 0
    }
    fn cancel(&self, c: &schema::CancelNavigation) -> bool {
        c.session_id == self.session_id && c.owner_kind == self.owner_kind
            && c.parent_request_id == self.parent_request_id && c.goal_generation == self.goal_generation
            && c.producer_runtime_generation == self.producer_runtime_generation
            && c.cancel_generation == self.cancel_generation
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TailTrigger { Pose, Costmap, Status }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GateSite { AfterAck, BeforeDelegate }
#[derive(Clone, Copy, Debug)]
enum WrapperSnapshotCase { Terminal, SessionMismatch }
impl WrapperSnapshotCase {
    fn matches(self, target: &Target, status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry) -> bool {
        telemetry.public_session_id == target.session_id
            && match self {
                Self::Terminal => status.last_terminal == Some(TerminalOutcome::Cancelled)
                    && status.session_id.as_deref() == Some(target.session_id.as_str()),
                Self::SessionMismatch => status.last_terminal.is_none()
                    && status.session_id.as_deref() == Some("synthetic-other-session"),
            }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReleaseMode { Normal, Cleanup }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BoundaryOutcome { Disabled, Normal, Cleanup, Disconnected }
struct Boundary {
    entered: oneshot::Sender<()>,
    release: Option<oneshot::Receiver<ReleaseMode>>,
    resumed: oneshot::Sender<BoundaryOutcome>,
}
pub(super) struct AckReturnProbe {
    target: Target,
    trace: TraceSink,
    boundary: Option<Boundary>,
    site: GateSite,
    cancel_completed: Option<oneshot::Sender<()>>,
    captured_batch: Option<(Vec<u8>, Vec<u8>)>, // exact nonzero Twist and NavigationState bodies
    observe_all_cancels: bool,
    expected_predelegate_twist: Option<Twist2>,
    wrapper_snapshot: Option<WrapperSnapshotCase>,
}

// These are separate test-only boundaries. BeforeDelegate pauses after frames
// construction, before the one original publish_batch; normal release proceeds
// through that unchanged call, same context, same immutable frame Vec.
pub(super) async fn hold_publication_ack(
    status: &nuwax_nav_runtime::SchedulerStatus, telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
) {
    hold_boundary(GateSite::AfterAck, status, telemetry, planner_observation, None).await;
}

pub(super) async fn hold_publication_before_delegate(
    status: &nuwax_nav_runtime::SchedulerStatus, telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>, frames: &[RawLcmFrame],
) {
    hold_boundary(GateSite::BeforeDelegate, status, telemetry, planner_observation, Some(frames)).await;
}

async fn hold_boundary(
    site: GateSite, status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry, planner_observation: Option<bool>, frames: Option<&[RawLcmFrame]>,
) {
    let selected = PUBLICATION_PROBE.try_with(|shared| {
        let mut shared = shared.lock().unwrap();
        let probe = shared.ack_return_probe.as_mut()?;
        let matches = probe.wrapper_snapshot.map_or_else(
            || probe.target.publication(status, telemetry, planner_observation),
            |case| case.matches(&probe.target, status, telemetry),
        );
        if probe.site != site || !matches { return None; }
        probe.boundary.as_ref()?; // Once consumed, never overwrite captured batch evidence.
        if let Some(frames) = frames.filter(|_| probe.wrapper_snapshot.is_none()) {
            // Do not infer that every ordinary status batch contains Twist:
            // original applied_commands dedup can legitimately omit it.
            let twist = frames.iter().find(|frame| {
                frame.msg_type.fqn() == "geometry_msgs.Twist"
                    && schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                        .is_ok_and(|twist| probe.expected_predelegate_twist.is_some_and(|expected|
                            twist.linear == [expected.linear.x, expected.linear.y, 0.0]
                                && twist.angular == [0.0, 0.0, expected.angular_z]))
            })?;
            let state = frames.iter().find(|frame| frame.msg_type.fqn() == "nuwax_msgs.NavigationState")?;
            probe.captured_batch = Some((twist.payload.clone(), state.payload.clone()));
        }
        probe.boundary.take().map(|boundary| (probe.trace.clone(), boundary))
    }).ok().flatten();
    let Some((trace, boundary)) = selected else { return; };
    // No PublicationProbe mutex or watch borrow is retained over the await.
    trace.mark(if site == GateSite::AfterAck { ACK_READY } else { BEFORE_DELEGATE }, 0);
    let outcome = if let Some(release) = boundary.release {
        trace.mark(GATE_HELD, 0);
        let _ = boundary.entered.send(());
        let outcome = match release.await {
            Ok(ReleaseMode::Normal) => BoundaryOutcome::Normal,
            Ok(ReleaseMode::Cleanup) => BoundaryOutcome::Cleanup,
            Err(_) => BoundaryOutcome::Disconnected,
        };
        if outcome != BoundaryOutcome::Normal { trace.mark(RELEASE_CLEANUP, 0); }
        trace.mark(GATE_RESUMED, 0);
        outcome
    } else {
        trace.mark(GATE_DISABLED, 0);
        let _ = boundary.entered.send(());
        BoundaryOutcome::Disabled
    };
    let _ = boundary.resumed.send(outcome);
}

pub(super) fn record_cancel(phase: &'static str, cancel: &schema::CancelNavigation) {
    let selected = PUBLICATION_PROBE.try_with(|shared| {
        let mut shared = shared.lock().unwrap();
        let probe = shared.ack_return_probe.as_mut().filter(|probe| probe.observe_all_cancels || probe.target.cancel(cancel))?;
        let completed = if phase == API_DONE { probe.cancel_completed.take() } else { None };
        Some((probe.trace.clone(), completed))
    }).ok().flatten();
    if let Some((trace, completed)) = selected {
        trace.mark(phase, 0);
        if let Some(completed) = completed { let _ = completed.send(()); }
    }
}

// Invoked at the original stream's existing observed-receipt send, before
// returning the frame. Goal-replacement receipts are not Cancel dequeue proof.
pub(super) fn record_observed_dequeue(frame: &TransportResult<RawLcmFrame>) {
    if !frame.as_ref().is_ok_and(|frame| frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation") { return; }
    let trace = PUBLICATION_PROBE.try_with(|shared| {
        shared.lock().unwrap().ack_return_probe.as_ref().map(|probe| probe.trace.clone())
    }).ok().flatten();
    if let Some(trace) = trace { trace.mark(DEQUEUED, 0); }
}

#[derive(Default)]
struct ReleaseOwner(Option<oneshot::Sender<ReleaseMode>>);
impl ReleaseOwner {
    fn normal(&mut self, trace: &TraceSink) -> bool {
        let Some(sender) = self.0.take() else { return false; };
        trace.mark(RELEASE_NORMAL, 0);
        sender.send(ReleaseMode::Normal).is_ok()
    }
    fn cleanup(&mut self) {
        if let Some(sender) = self.0.take() { let _ = sender.send(ReleaseMode::Cleanup); }
    }
}
impl Drop for ReleaseOwner { fn drop(&mut self) { self.cleanup(); } }

#[derive(Debug)]
struct Cleanup {
    loop_stopped: bool,
    scheduler_stopped: bool,
    normal: bool,
    details: Vec<String>,
}
async fn cleanup(h: &mut GuardedLoopHarness) -> Cleanup {
    let shutdown_sent = h.shutdown.take().is_some_and(|shutdown| shutdown.send(()).is_ok());
    let loop_result = tokio::time::timeout(Duration::from_secs(1), &mut h.loop_task).await;
    let loop_normal = matches!(&loop_result, Ok(Ok(Ok(()))));
    let mut loop_stopped = loop_result.is_ok();
    let mut details = vec![format!("bridge shutdown_sent={shutdown_sent} join={loop_result:?}")];
    if !loop_stopped {
        h.loop_task.abort();
        let result = tokio::time::timeout(Duration::from_secs(1), &mut h.loop_task).await;
        loop_stopped = result.is_ok();
        details.push(format!("bridge abort join={result:?}"));
    }
    let shutdown = tokio::time::timeout(Duration::from_secs(1), h.scheduler.shutdown()).await;
    let shutdown_ok = matches!(&shutdown, Ok(Ok(())));
    details.push(format!("scheduler shutdown={shutdown:?}"));
    let scheduler_result = tokio::time::timeout(Duration::from_secs(1), &mut h.scheduler_task).await;
    let scheduler_normal = matches!(&scheduler_result, Ok(Ok(Ok(()))));
    let mut scheduler_stopped = scheduler_result.is_ok();
    details.push(format!("scheduler join={scheduler_result:?}"));
    if !scheduler_stopped {
        h.scheduler_task.abort();
        let result = tokio::time::timeout(Duration::from_secs(1), &mut h.scheduler_task).await;
        scheduler_stopped = result.is_ok();
        details.push(format!("scheduler abort join={result:?}"));
    }
    Cleanup {
        loop_stopped, scheduler_stopped,
        normal: shutdown_sent && loop_normal && shutdown_ok && scheduler_normal,
        details,
    }
}

#[derive(Debug)]
struct Scenario {
    boundary: BoundaryOutcome,
    cancelled: bool,
    zero_count_before: usize,
    zero_count_after: usize,
}
async fn scenario(
    h: &mut GuardedLoopHarness, trigger: TailTrigger, site: GateSite, hold: bool,
    queue_replacement: bool, trace: &TraceSink,
    release_owner: &mut ReleaseOwner,
) -> Result<Scenario, String> {
    let goal = activate_map_a(h).await;
    let path = sealed_path(&goal);
    h.send_path(&path);
    let mut status = h.scheduler.subscribe_status();
    tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let snapshot = status.borrow_and_update().clone();
            if snapshot.phase == SchedulerPhase::Controlling && snapshot.path_revision == 1
                && snapshot.last_applied_twist.is_some_and(|twist| twist != Twist2::ZERO) {
                return Ok::<(), String>(());
            }
            status.changed().await.map_err(|error| error.to_string())?;
        }
    }).await.map_err(|_| "active nonzero path prerequisite timed out".to_string())??;
    let zero_count_before = h.motion_call_count("zero:");
    let acquire_count_before = h.motion_call_count("acquire:");
    let mut cancellation_snapshot = None;
    let (entered, installation) = oneshot::channel();
    let (resumed, mut resume_receipt) = oneshot::channel();
    let release = if hold {
        let (sender, receiver) = oneshot::channel();
        release_owner.0 = Some(sender);
        Some(receiver)
    } else { None };
    let (cancel_completed, mut cancel_completion) = oneshot::channel();
    h.publication_probe.lock().unwrap().ack_return_probe = Some(AckReturnProbe {
        target: Target::new(&goal, path.path_revision, 1).for_trigger(trigger), trace: trace.clone(),
        boundary: Some(Boundary { entered, release, resumed }),
        site, cancel_completed: Some(cancel_completed), captured_batch: None, observe_all_cancels: false,
        expected_predelegate_twist: (site == GateSite::BeforeDelegate).then_some(DISTINCT_TAIL_TWIST),
        wrapper_snapshot: None,
    });
    if site == GateSite::BeforeDelegate {
        // The fixture runs current_thread: there is no await between installing
        // the gate and changing the real test planner's next command. All prior
        // commands used the different default value. The gate captures the FIRST
        // actual batch containing this new value, before original publish_batch.
        let mut planner_twist = h.planner_twist.lock().unwrap();
        assert_ne!(*planner_twist, DISTINCT_TAIL_TWIST);
        *planner_twist = DISTINCT_TAIL_TWIST;
    }
    // Ordinary input through the original selector, not a direct helper call.
    // Existing synthetic pose timestamp policy and five-second TTL are unchanged.
    match trigger {
        TailTrigger::Pose => h.send_pose(2),
        TailTrigger::Costmap => h.send_costmap(12),
        TailTrigger::Status => {} // only the real scheduler watch can trigger this tail
    }
    tokio::time::timeout(Duration::from_secs(1), installation).await
        .map_err(|_| "matching real publish ACK boundary did not enter".to_string())?
        .map_err(|_| "ACK boundary installation sender closed".to_string())?;

    let mut dequeue_receipt = send_guarded_test_body_observed(
        &h.cancels, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 1),
    );
    // Both selectors explicitly use current_thread. No actor can run between
    // synchronous test-stream enqueue and this marker; this is not native enqueue.
    trace.mark(ENQUEUED, 0);
    let queued_goal = queue_replacement.then(|| rollover_goal(&goal, "session-queued-after-tail", 8, 1));
    let mut goal_receipt = queued_goal.as_ref().map(|next| send_guarded_test_body_observed(
        &h.goals, "nuwax_msgs.GoalEnvelope", schema::encode_goal_envelope_body(next).expect("queued Goal body")));
    if hold {
        // New contract: the real Cancel future must be observed Ok by the same
        // original loop owner while its original publication future is held.
        tokio::time::timeout(Duration::from_secs(1), &mut cancel_completion).await
            .map_err(|_| "real Cancel result was not observed while publication remained held".to_string())?
            .map_err(|_| "Cancel completion hook closed".to_string())?;
        trace.mark(CANCEL_READY_HELD, 0);
        if !matches!(resume_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)) {
            return Err("publication gate was not still closed at checkpoint".into());
        }
        tokio::time::timeout(Duration::from_secs(1), &mut dequeue_receipt).await
            .map_err(|_| "Cancel was not dequeued while publication was held".to_string())?
            .map_err(|_| "Cancel dequeue receipt closed".to_string())?;
        let snapshot = h.scheduler.status();
        if snapshot.last_terminal != Some(TerminalOutcome::Cancelled)
            || snapshot.last_zero_ack_id.is_none()
            || h.motion_call_count("zero:") != zero_count_before + 1 {
            return Err("real exact-goal fence did not finish at closed checkpoint".into());
        }
        cancellation_snapshot = Some(snapshot);
        if let Some(receipt) = &mut goal_receipt {
            if !matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty))
                || h.motion_call_count("acquire:") != acquire_count_before {
                return Err("queued replacement Goal was processed before old P/Cancel settlement".into());
            }
        }
        // Read real same-context output already delivered. This is only a
        // checkpoint check; no absence-of-arrival deadline is claimed.
        while let Some(frame) = h.states.next().now_or_never().flatten() {
            let frame = frame.map_err(|error| error.to_string())?;
            let state = schema::decode_navigation_state_body(&frame.payload)
                .map_err(|error| error.to_string())?;
            if state.session_id == goal.session_id && state.cancel_generation == 1
                && !state.zero_ack_id.is_empty() {
                return Err("correlated ZEROED overtook the still-owned old publication".into());
            }
        }
        trace.mark(CHECKPOINT, 0);
        if !release_owner.normal(trace) { return Err("normal gate release was not delivered".into()); }
    }
    let boundary = tokio::time::timeout(Duration::from_secs(1), &mut resume_receipt).await
        .map_err(|_| "ACK gate return was not observed".to_string())?
        .map_err(|_| "ACK gate return sender closed".to_string())?;
    if !hold {
        tokio::time::timeout(Duration::from_secs(1), &mut dequeue_receipt).await
            .map_err(|_| "Cancel was not dequeued by the original stream".to_string())?
            .map_err(|_| "Cancel dequeue receipt closed".to_string())?;
    }
    if site == GateSite::BeforeDelegate {
        let (old_twist, old_state) = h.publication_probe.lock().unwrap()
            .ack_return_probe.as_ref().and_then(|probe| probe.captured_batch.clone())
            .ok_or_else(|| "pre-delegate gate did not capture a real nonzero batch".to_string())?;
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut seen_old_twist = false;
            loop {
                let frame = h.commands.next().await.ok_or("command stream closed")?
                    .map_err(|error| error.to_string())?;
                let twist = schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                    .map_err(|error| error.to_string())?;
                if frame.payload == old_twist && !seen_old_twist {
                    seen_old_twist = true;
                    trace.mark(OLD_TWIST_RECEIVED, 0);
                }
                if twist.linear == [0.0; 3] && twist.angular == [0.0; 3] {
                    // Old startup ZERO may still be queued. It is not the
                    // cancellation ZERO and cannot identify the held batch.
                    if seen_old_twist { break; }
                } else if seen_old_twist && frame.payload != old_twist {
                    return Err("unexpected different nonzero Twist after held old batch".to_string());
                }
            }
            loop {
                let frame = h.states.next().await.ok_or("state stream closed")?
                    .map_err(|error| error.to_string())?;
                let state = schema::decode_navigation_state_body(&frame.payload)
                    .map_err(|error| error.to_string())?;
                if frame.payload == old_state {
                    if state.cancel_generation != 0 || !state.zero_ack_id.is_empty() {
                        return Err("captured old batch identity was rewritten".to_string());
                    }
                    trace.mark(OLD_STATE_RECEIVED, 0);
                    return Ok(());
                }
                if state.session_id == goal.session_id && state.cancel_generation == 1
                    && !state.zero_ack_id.is_empty() {
                    return Err("correlated ZEROED overtook exact captured old state".to_string());
                }
            }
        }).await.map_err(|_| "real old batch did not precede terminal output".to_string())??;
    }
    let zero = h.wait_for_correlated_zero(&goal, 1).await;
    let terminal = cancellation_snapshot.unwrap_or_else(|| h.scheduler.status());
    if zero.producer_runtime_generation != goal.producer_runtime_generation
        || !zero.message.starts_with("CANCELLED:")
        || terminal.last_zero_ack_id.as_deref() != Some(zero.zero_ack_id.as_str())
        || i64::try_from(terminal.last_fence_monotonic_ns).ok() != Some(zero.zero_applied_monotonic_ns) {
        return Err("external exact cancellation proof did not match the scheduler fence".into());
    }
    trace.mark(ZEROED, 0);
    if let (Some(next), Some(receipt)) = (queued_goal, goal_receipt) {
        tokio::time::timeout(Duration::from_secs(1), receipt).await
            .map_err(|_| "queued replacement Goal did not re-enter normal rotation".to_string())?
            .map_err(|_| "queued replacement Goal receipt closed".to_string())?;
        h.wait_for_session(&next.session_id).await;
        if h.motion_call_count("acquire:") != acquire_count_before + 1 {
            return Err("replacement Goal did not acquire exactly once after old tail/Cancel".into());
        }
    }
    Ok(Scenario {
        boundary, cancelled: terminal.last_terminal == Some(TerminalOutcome::Cancelled),
        zero_count_before, zero_count_after: h.motion_call_count("zero:"),
    })
}

fn unique(report: &TraceReport, phase: &str) -> u64 {
    let events: Vec<_> = report.records.iter().filter(|event| event.phase == phase).collect();
    assert_eq!(events.len(), 1, "expected exactly one {phase} event");
    events[0].at_ns
}

async fn run(trigger: TailTrigger, site: GateSite, hold: bool, queue_replacement: bool) {
    // Original harness bootstrapping is unchanged. Once it returns, every
    // scenario failure/panic is collected before the owned tasks are cleaned up.
    let mut harness = GuardedLoopHarness::spawn().await;
    let trace = TraceSession::start(256);
    let sink = trace.sink();
    let mut release_owner = ReleaseOwner::default();
    let result = std::panic::AssertUnwindSafe(scenario(&mut harness, trigger, site, hold, queue_replacement, &sink, &mut release_owner))
        .catch_unwind().await;
    release_owner.cleanup(); // Error/drop releases are never normal release.
    let cleanup = cleanup(&mut harness).await;
    drop(harness); // Original subscription/context field teardown after task joins.
    let mut report = trace.finish();
    report.incomplete |= !cleanup.loop_stopped || !cleanup.scheduler_stopped;
    let summary = match &result {
        Ok(result) => format!("{result:?}"), Err(_) => "scenario panic caught before cleanup".into(),
    };
    eprintln!("BRIDGE_ACK_RETURN contract=early_cancel trigger={trigger:?} site={site:?} hold={hold} queue_replacement={queue_replacement} scenario={summary} cleanup={cleanup:?}");
    eprintln!("BRIDGE_ACK_TRACE capacity={} records={} incomplete={} contended={} overflow={} poisoned={} closed_race={} open={} finish_unavailable={}",
        report.capacity, report.records.len(), report.incomplete, report.dropped_contended,
        report.dropped_overflow, report.dropped_poisoned, report.dropped_closed_race,
        report.open_spans_at_finish, report.finish_buffer_unavailable);
    for event in &report.records {
        eprintln!("BRIDGE_ACK_EVENT at_ns={} phase={} thread={:?} data={:?}",
            event.at_ns, event.phase, event.thread, event.data);
    }
    // Assertions follow cleanup/output. No claimed performance deadline is added.
    assert!(cleanup.normal, "original bridge/scheduler cleanup did not complete normally: {:?}", cleanup.details);
    assert!(!report.incomplete, "incomplete trace or task ownership cannot establish causal order");
    let result = match result { Ok(result) => result.expect("bridge ACK scenario failed"), Err(_) => panic!("bridge ACK scenario panicked") };
    assert!(result.cancelled);
    assert_eq!(result.zero_count_after, result.zero_count_before + 1, "exactly one cancellation fence required");
    assert_eq!(result.boundary, if hold { BoundaryOutcome::Normal } else { BoundaryOutcome::Disabled });
    let ack = unique(&report, if site == GateSite::AfterAck { ACK_READY } else { BEFORE_DELEGATE });
    let queued = unique(&report, ENQUEUED);
    let dequeued = unique(&report, DEQUEUED);
    let decoded = unique(&report, DECODED);
    let api_begin = unique(&report, API_BEGIN);
    let api_ok = unique(&report, API_OK);
    let external = unique(&report, ZEROED);
    assert!(ack <= queued && queued <= dequeued && dequeued <= decoded
        && decoded <= api_begin && api_begin <= api_ok && api_ok <= external);
    assert!(!report.records.iter().any(|event| event.phase == RELEASE_CLEANUP));
    if hold {
        let held = unique(&report, GATE_HELD);
        let cancelled = unique(&report, CANCEL_READY_HELD);
        let checkpoint = unique(&report, CHECKPOINT);
        let release = unique(&report, RELEASE_NORMAL);
        let resumed = unique(&report, GATE_RESUMED);
        assert!(ack <= held && held <= queued && queued <= dequeued && api_ok <= cancelled && cancelled <= checkpoint
            && checkpoint <= release && release <= resumed && resumed <= external);
        assert!(!report.records.iter().any(|event| event.phase == GATE_DISABLED));
        if site == GateSite::BeforeDelegate {
            assert!(resumed <= unique(&report, OLD_STATE_RECEIVED));
            assert!(unique(&report, OLD_STATE_RECEIVED) <= external);
            assert!(resumed <= unique(&report, OLD_TWIST_RECEIVED));
        }
    } else {
        assert!(ack <= unique(&report, GATE_DISABLED));
        assert!(!report.records.iter().any(|event| matches!(event.phase,
            GATE_HELD | CANCEL_READY_HELD | CHECKPOINT | RELEASE_NORMAL | GATE_RESUMED)));
    }
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit early Cancel while original bridge ACK-return is held; no latency claim"]
async fn guarded_cancel_ingress_completes_while_observation_ack_return_held() {
    run(TailTrigger::Pose, GateSite::AfterAck, true, false).await;
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit unheld control for the original bridge Cancel ingress and exact fence"]
async fn guarded_cancel_ingress_without_ack_return_gate() { run(TailTrigger::Pose, GateSite::AfterAck, false, false).await; }

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit pre-delegate order control; real same-context publication after early fence"]
async fn guarded_early_cancel_retains_old_nonzero_batch_before_new_zeroed() {
    run(TailTrigger::Status, GateSite::BeforeDelegate, true, false).await;
}

// The following controls exercise the exact private production coordination
// helper. P is a controlled future, NOT a failed native publish_batch. C is the
// original NavSchedulerHandle::cancel against a real spawned scheduler actor.
const API_DONE: &str = "bridge_ack_test.cancel_api_done";
const MATRIX_P_ERROR: &str = "controlled-publication-result-error";

#[derive(Clone, Default)]
struct CapturedWarnings(Arc<Mutex<Vec<std::collections::BTreeMap<String, String>>>>);
struct WarningFields(std::collections::BTreeMap<String, String>);
impl tracing::field::Visit for WarningFields {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        self.0.insert(field.name().into(), format!("{value:?}"));
    }
    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        self.0.insert(field.name().into(), value.into());
    }
    fn record_i64(&mut self, field: &tracing::field::Field, value: i64) {
        self.0.insert(field.name().into(), value.to_string());
    }
    fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
        self.0.insert(field.name().into(), value.to_string());
    }
}
impl tracing::Subscriber for CapturedWarnings {
    fn enabled(&self, metadata: &tracing::Metadata<'_>) -> bool {
        *metadata.level() == tracing::Level::WARN
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        let mut fields = WarningFields(Default::default());
        event.record(&mut fields);
        if fields.0.get("message").is_some_and(|message| message.contains("ordinary publication failed after")) {
            self.0.lock().unwrap().push(fields.0);
        }
    }
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
}

async fn matrix_scheduler(
    goal: &schema::GoalEnvelope, active: bool,
) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>, Arc<Mutex<Vec<String>>>) {
    let calls = Arc::new(Mutex::new(Vec::new()));
    let scheduler = nuwax_nav_runtime::NavScheduler::new(
        RecordingMotion(Arc::clone(&calls)),
        CommandingPlanner {
            succeeds: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            twist: Arc::new(Mutex::new(Twist2 { linear: Point2::new(0.2, 0.0), angular_z: 0.1 })),
        },
        nuwax_nav_runtime::SchedulerConfig {
            tick_interval: Duration::from_millis(5), renew_interval: Duration::from_millis(100),
            path_wait_timeout: Duration::from_secs(5), robot_width_m: 0.3,
            pose_ttl: Duration::from_secs(5), costmap_ttl: Duration::from_secs(5),
        },
    ).expect("matrix real scheduler configuration");
    let (handle, mut task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
    let setup = std::panic::AssertUnwindSafe(async {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !status.borrow_and_update().motion_sink_ready {
                status.changed().await.expect("matrix real scheduler status");
            }
        }).await.expect("matrix real scheduler readiness");
        if active {
            let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
            pose.stamp = Timestamp::from_unix(1, 0);
            handle.update_odometry(pose).await.expect("matrix pose input");
            handle.update_costmap(nuwax_nav_runtime::CostmapObservation {
                costmap: nuwax_nav_runtime::ExecutionMap::Planar(
                    nuwax_nav_astar::navigation_map::CostMap2D::new_all_free("map", 4, 4, 0.1)),
                source_timestamp_ns: 1_000_000_000,
            }).await.expect("matrix map input");
            handle.start_session(nuwax_nav_runtime::NavSession {
                boundary: Vec::new(), options: Default::default(),
                goal: nuwax_nav_runtime::ExecutionGoal::Body(pose_from_schema(&goal.goal).expect("matrix goal pose")),
                ground_initial: None,
                session_id: goal.session_id.clone(),
                execution_identity: guarded_goal_execution_identity(goal),

                motion_admission: nuwax_nav_runtime::NavMotionAdmission {

                    guard_boot_nonce: goal.motion_guard_boot_nonce,
                    motion_stop_generation: u64::try_from(goal.motion_stop_generation).expect("stop generation"),
                },
            }).await.expect("matrix active session");
        }
    }).catch_unwind().await;
    if let Err(panic) = setup {
        // This actor was created here, before the caller's scenario cleanup.
        // Preserve the failed setup prerequisite, but attempt both original
        // shutdown and join; abort/rejoin is failure cleanup, never success.
        let shutdown = tokio::time::timeout(Duration::from_secs(1), handle.shutdown()).await;
        let joined = tokio::time::timeout(Duration::from_secs(1), &mut task).await;
        eprintln!("BRIDGE_TAIL_SETUP_FAILED shutdown={shutdown:?} join={joined:?}");
        if joined.is_err() {
            task.abort();
            let rejoined = tokio::time::timeout(Duration::from_secs(1), &mut task).await;
            eprintln!("BRIDGE_TAIL_SETUP_FAILED abort_join={rejoined:?}");
        }
        std::panic::resume_unwind(panic);
    }
    (handle, task, calls)
}

fn count_zero(calls: &Arc<Mutex<Vec<String>>>) -> usize {
    calls.lock().unwrap().iter().filter(|call| call.starts_with("zero:")).count()
}

fn assert_original_p_error(result: NavStackResult<()>) {
    match result {
        Err(NavStackError::Internal(message)) => assert_eq!(message, MATRIX_P_ERROR),
        Err(other) => panic!("P error type changed: {other}"),
        Ok(()) => panic!("controlled P failure was hidden"),
    }
}

fn assert_accepted_warning(
    warnings: &CapturedWarnings, goal: &schema::GoalEnvelope, cancel_ok: bool,
) {
    let events = warnings.0.lock().unwrap();
    assert_eq!(events.len(), 1, "exactly one real production warn is required: {events:?}");
    let event = &events[0];
    assert_eq!(event["message"], "ordinary publication failed after early active cancellation");
    assert!(event["publication_error"].contains(MATRIX_P_ERROR));
    assert_eq!(event["session_id"], goal.session_id);
    assert_eq!(event["owner_kind"], goal.owner_kind.to_string());
    assert_eq!(event["parent_request_id"], goal.parent_request_id);
    assert_eq!(event["goal_generation"], goal.goal_generation.to_string());
    assert_eq!(event["producer_runtime_generation"], goal.producer_runtime_generation.to_string());
    assert_eq!(event["cancel_generation"], "1");
    if cancel_ok {
        assert_eq!(event["cancel_result"], "Ok(())");
    } else {
        assert!(event["cancel_result"].contains("Err(Busy("), "actual C failure lost: {event:?}");
    }
}

#[derive(Clone, Copy, Debug)]
enum CompletionOrder { PublicationFirst, CancelFirst }

async fn result_pair_control(publication_fails: bool, cancel_ok: bool, order: CompletionOrder) {
    use tracing::instrument::WithSubscriber;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    let goal = sealed_goal("tail-pair-exact");
    let (scheduler, scheduler_task, calls) = matrix_scheduler(&goal, cancel_ok).await;
    let baseline = count_zero(&calls);
    let (cancel_sender, mut cancels) = guarded_test_stream();
    let mut dequeued = send_guarded_test_body_observed(&cancel_sender, "nuwax_msgs.CancelNavigation",
        GuardedLoopHarness::cancel_body(&goal, 1));
    let trace = TraceSession::start(128);
    let (completed, mut completion) = oneshot::channel();
    let probe = Arc::new(Mutex::new(PublicationProbe::default()));
    probe.lock().unwrap().ack_return_probe = Some(AckReturnProbe {
        target: Target::new(&goal, 1, 1), trace: trace.sink(), boundary: None,
        site: GateSite::AfterAck, cancel_completed: Some(completed), captured_batch: None, observe_all_cancels: false,
        expected_predelegate_twist: None, wrapper_snapshot: None,
    });
    let warnings = CapturedWarnings::default();
    let dispatch = tracing::Dispatch::new(warnings.clone());
    let shutdown_count = Arc::new(AtomicUsize::new(0));
    let joined = Arc::new(AtomicBool::new(false));
    let shutdown_count_copy = Arc::clone(&shutdown_count);
    let joined_copy = Arc::clone(&joined);
    let shutdown_scheduler = scheduler.clone();
    let before_cleanup = Arc::new(AtomicUsize::new(usize::MAX));
    let before_cleanup_copy = Arc::clone(&before_cleanup);
    let scenario = async {
        let mut watermark = None;
        let mut arbiter = GuardedNavArbiter { cursor: GuardedNavSource::Goal };
        let initial_cursor = arbiter.cursor;
        let (release, released) = oneshot::channel::<NavStackResult<()>>();
        let publication = async { released.await.expect("controlled P release owned") };
        let result = {
            let helper = await_guarded_tail_publication(publication, &scheduler, &mut cancels,
                Some(&goal), &mut watermark, &mut arbiter);
            let scoped = PUBLICATION_PROBE.scope(probe, helper).with_subscriber(dispatch);
            tokio::pin!(scoped);
            // On current_thread no scheduler task can run between these polls.
            // First poll admits one Cancel and starts its real urgent request.
            assert!(futures::poll!(&mut scoped).is_pending());
            assert!(matches!(dequeued.try_recv(), Ok(())));
            match order {
                CompletionOrder::PublicationFirst => {
                    release.send(if publication_fails { Err(NavStackError::Internal(MATRIX_P_ERROR.into())) } else { Ok(()) })
                        .expect("P receiver still owned");
                    // P is now Ready but C cannot yet have been polled to its
                    // completed response: neither real actor nor test has yielded.
                    assert!(futures::poll!(&mut scoped).is_pending(), "P must not discard unfinished C");
                    assert!(matches!(completion.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
                }
                CompletionOrder::CancelFirst => {
                    tokio::time::timeout(Duration::from_secs(1), async {
                        tokio::select! {
                            biased;
                            _ = &mut scoped => panic!("helper returned while P was still owned and held"),
                            ready = &mut completion => ready.expect("real C result observation"),
                        }
                    }).await.expect("real C result while P held");
                    assert!(futures::poll!(&mut scoped).is_pending(), "C must not discard unfinished P");
                    release.send(if publication_fails { Err(NavStackError::Internal(MATRIX_P_ERROR.into())) } else { Ok(()) })
                        .expect("P receiver still owned after C result");
                }
            }
            tokio::time::timeout(Duration::from_secs(1), scoped).await.expect("owned helper completion")
        };
        assert_eq!(arbiter.cursor, GuardedNavSource::Cancel.successor());
        assert_ne!(arbiter.cursor, initial_cursor);
        let observed = watermark.as_ref().expect("one admitted watermark");
        assert!(planner_goal_identity_matches(&observed.goal, &goal));
        assert_eq!(observed.generation, 1);
        before_cleanup_copy.store(count_zero(&calls), Ordering::SeqCst);
        match result {
            Err(error) => Err(error),
            Ok(Some(OwnerInput::Accepted(completion))) => {
                assert_eq!(completion.active_goal, goal);
                assert_eq!(completion.cancel_generation, 1);
                assert_eq!(completion.result.is_ok(), cancel_ok);
                if !cancel_ok { assert!(matches!(completion.result, Err(RuntimeError::Busy(_)))); }
                Ok(())
            }
            _ => panic!("one exact Cancel must return its actual completion"),
        }
    };
    // A scenario assertion/panic is converted only for cleanup, then must fail
    // the result assertion below. Original scheduler actor is joined in all cases.
    let loop_future = async {
        match std::panic::AssertUnwindSafe(scenario).catch_unwind().await {
            Ok(result) => result,
            Err(_) => Err(NavStackError::Internal("result-pair control panicked".into())),
        }
    };
    let coordinated = coordinate_guarded_scheduler(loop_future, async move {
        let result = scheduler_task.await;
        joined_copy.store(true, Ordering::SeqCst);
        result.map_err(|error| NavStackError::Internal(format!("matrix scheduler join: {error}")))?
            .map_err(|error| NavStackError::Internal(format!("matrix scheduler task: {error}")))
    }, move || async move {
        shutdown_count_copy.fetch_add(1, Ordering::SeqCst);
        shutdown_scheduler.shutdown().await.map_err(|error| NavStackError::Internal(format!("matrix shutdown: {error}")))
    }).await;
    let report = trace.finish();
    assert!(!report.incomplete);
    assert_eq!(shutdown_count.load(Ordering::SeqCst), 1);
    assert!(joined.load(Ordering::SeqCst));
    assert_eq!(before_cleanup.load(Ordering::SeqCst), baseline + usize::from(cancel_ok),
        "one C fence on Ok; no fictitious fence on actual Busy");
    if publication_fails {
        assert_original_p_error(coordinated);
        assert_accepted_warning(&warnings, &goal, cancel_ok);
    } else {
        coordinated.expect("successful P keeps ordinary coordinator cleanup");
        assert!(warnings.0.lock().unwrap().is_empty());
    }
    // Count actual execute entry and completion independently of motion calls.
    assert_eq!(report.records.iter().filter(|event| event.phase == API_BEGIN).count(), 1);
    assert_eq!(report.records.iter().filter(|event| event.phase == API_DONE).count(), 1);
    eprintln!("BRIDGE_TAIL_PAIR publication_fails={publication_fails} cancel_ok={cancel_ok} order={order:?} collected_both=true shutdowns=1 joined=true");
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit controlled-P / real-C result matrix; not native publication faults"]
async fn guarded_tail_result_pair_matrix_collects_both_results() {
    for order in [CompletionOrder::PublicationFirst, CompletionOrder::CancelFirst] {
        for publication_fails in [false, true] {
            for cancel_ok in [false, true] {
                result_pair_control(publication_fails, cancel_ok, order).await;
            }
        }
    }
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit Status and final Costmap tail source controls"]
async fn guarded_early_cancel_covers_status_and_final_costmap_tails() {
    for trigger in [TailTrigger::Status, TailTrigger::Costmap] {
        run(trigger, GateSite::AfterAck, true, false).await;
    }
}

async fn coordinate_control<F: Future<Output = NavStackResult<()>>>(
    scenario: F, scheduler: &NavSchedulerHandle,
    scheduler_task: JoinHandle<Result<(), RuntimeError>>,
) -> NavStackResult<()> {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    let shutdowns = Arc::new(AtomicUsize::new(0));
    let joined = Arc::new(AtomicBool::new(false));
    let on_shutdown = Arc::clone(&shutdowns);
    let on_join = Arc::clone(&joined);
    let result = coordinate_guarded_scheduler(async {
        match std::panic::AssertUnwindSafe(scenario).catch_unwind().await {
            Ok(result) => result,
            Err(_) => Err(NavStackError::Internal("tail control scenario panicked before cleanup".into())),
        }
    }, async move {
        let result = scheduler_task.await;
        on_join.store(true, Ordering::SeqCst);
        result.map_err(|error| NavStackError::Internal(format!("control scheduler join: {error}")))?
            .map_err(|error| NavStackError::Internal(format!("control scheduler task: {error}")))
    }, || async {
        on_shutdown.fetch_add(1, Ordering::SeqCst);
        scheduler.shutdown().await.map_err(|error| NavStackError::Internal(format!("control scheduler shutdown: {error}")))
    }).await;
    assert_eq!(shutdowns.load(Ordering::SeqCst), 1);
    assert!(joined.load(Ordering::SeqCst));
    result
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit P-Ready and excluded-window no-prefetch controls"]
async fn guarded_tail_ready_and_ineligible_do_not_read_cancel() {
    for eligible in [true, false] {
        let goal = sealed_goal("tail-ready-exact");
        let (scheduler, task, calls) = matrix_scheduler(&goal, true).await;
        let baseline = count_zero(&calls);
        let scenario = async {
            let (sender, mut stream) = guarded_test_stream();
            let mut receipt = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation",
                GuardedLoopHarness::cancel_body(&goal, 1));
            let mut watermark = None;
            let mut arbiter = GuardedNavArbiter { cursor: GuardedNavSource::Path };
            if eligible {
                // Both P and the original Cancel stream are synchronously Ready.
                let result = await_guarded_tail_publication(std::future::ready(Ok(())), &scheduler,
                    &mut stream, Some(&goal), &mut watermark, &mut arbiter).await?;
                assert!(result.is_none());
            } else {
                let (release, released) = oneshot::channel();
                let helper = await_guarded_tail_publication(async {
                    released.await.expect("excluded P owner"); Ok(())
                }, &scheduler, &mut stream, None, &mut watermark, &mut arbiter);
                tokio::pin!(helper);
                assert!(futures::poll!(&mut helper).is_pending());
                assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
                release.send(()).expect("excluded P release");
                assert!(helper.await?.is_none());
            }
            assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
            assert!(watermark.is_none());
            assert_eq!(arbiter.cursor, GuardedNavSource::Path);
            assert_eq!(count_zero(&calls), baseline);
            assert!(stream.next().now_or_never().flatten().is_some(), "original queued frame remains available");
            receipt.await.expect("frame receipt only after explicit test dequeue");
            Ok(())
        };
        coordinate_control(scenario, &scheduler, task).await.expect("Ready/no-window control and cleanup");
        eprintln!("BRIDGE_TAIL_READY eligible={eligible} no_prefetch=true");
    }
}

#[derive(Clone, Copy, Debug)]
enum RejectedHead { Session, Owner, Parent, GoalGeneration, Producer, Stale, Nonpositive, Malformed, Closed }
impl RejectedHead {
    fn expected_reason(self) -> &'static str {
        match self {
            Self::Session | Self::Owner | Self::Parent | Self::GoalGeneration | Self::Producer => "identity does not match",
            Self::Stale => "stale cancel generation 1; highest observed generation is 3",
            Self::Nonpositive => "must be positive",
            Self::Malformed => "CancelNavigation rejected:",
            Self::Closed => "cancel LCM stream closed",
        }
    }
}

async fn rejected_head_control(case: RejectedHead, publication_fails: bool) {
    use tracing::instrument::WithSubscriber;
    let goal = sealed_goal("tail-head-exact");
    let (scheduler, task, calls) = matrix_scheduler(&goal, true).await;
    let baseline = count_zero(&calls);
    let warnings = CapturedWarnings::default();
    let trace = TraceSession::start(128);
    let probe = Arc::new(Mutex::new(PublicationProbe::default()));
    probe.lock().unwrap().ack_return_probe = Some(AckReturnProbe {
        target: Target::new(&goal, 1, 1), trace: trace.sink(), boundary: None,
        site: GateSite::AfterAck, cancel_completed: None, captured_batch: None,
        observe_all_cancels: true, expected_predelegate_twist: None, wrapper_snapshot: None,
    });
    let scenario = async {
        let (sender, mut stream) = guarded_test_stream();
        let mut foreign = goal.clone();
        match case {
            RejectedHead::Session => foreign.session_id.push_str("-foreign"),
            RejectedHead::Owner => foreign.owner_kind += 1,
            RejectedHead::Parent => foreign.parent_request_id.push_str("-foreign"),
            RejectedHead::GoalGeneration => foreign.goal_generation += 1,
            RejectedHead::Producer => foreign.producer_runtime_generation += 1,
            _ => {}
        }
        let mut watermark = if matches!(case, RejectedHead::Stale) {
            Some(GuardedCancelGenerationWatermark { goal: goal.clone(), generation: 3 })
        } else { None };
        let mut first_receipt = None;
        let mut second_receipt = None;
        if matches!(case, RejectedHead::Closed) {
            drop(sender); // Actual original RawFrame source EOF; no synthetic backend fault.
        } else {
            let body = if matches!(case, RejectedHead::Malformed) { vec![0xff] }
                else { GuardedLoopHarness::cancel_body(&foreign,
                    if matches!(case, RejectedHead::Nonpositive) { 0 } else { 1 }) };
            first_receipt = Some(send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation", body));
            second_receipt = Some(send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation",
                GuardedLoopHarness::cancel_body(&goal, 3)));
            // Keep sender alive via the queued frames; closing after these two is
            // harmless because the helper is allowed to consume only the head.
            drop(sender);
        }
        let mut arbiter = GuardedNavArbiter { cursor: GuardedNavSource::Goal };
        let (release, released) = oneshot::channel::<NavStackResult<()>>();
        let result = {
            let helper = await_guarded_tail_publication(async { released.await.expect("rejected-head P owned") },
                &scheduler, &mut stream, Some(&goal), &mut watermark, &mut arbiter);
            let scoped = PUBLICATION_PROBE.scope(probe, helper)
                .with_subscriber(tracing::Dispatch::new(warnings.clone()));
            tokio::pin!(scoped);
            assert!(futures::poll!(&mut scoped).is_pending(), "rejection must retain original pending P");
            if let Some(receipt) = &mut first_receipt { assert!(matches!(receipt.try_recv(), Ok(()))); }
            if let Some(receipt) = &mut second_receipt {
                assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)),
                    "one rejected head must not be bypassed to consume the second Cancel");
            }
            assert_eq!(count_zero(&calls), baseline);
            release.send(if publication_fails { Err(NavStackError::Internal(MATRIX_P_ERROR.into())) } else { Ok(()) })
                .expect("rejected-head P still owned");
            tokio::time::timeout(Duration::from_secs(1), scoped).await.expect("owned helper completion")
        };
        assert_eq!(arbiter.cursor, GuardedNavSource::Cancel.successor());
        if matches!(case, RejectedHead::Stale) {
            assert_eq!(watermark.as_ref().expect("old watermark retained").generation, 3);
            assert!(planner_goal_identity_matches(&watermark.as_ref().unwrap().goal, &goal));
        } else { assert!(watermark.is_none(), "rejected input polluted watermark"); }
        assert_eq!(count_zero(&calls), baseline);
        if let Some(receipt) = &mut second_receipt {
            assert!(matches!(receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
            let frame = stream.next().now_or_never().flatten().expect("second frame remains").expect("second frame valid");
            let cancel = schema::decode_cancel_navigation_body(&frame.payload).expect("second exact Cancel");
            validate_cancel_goal_correlation(&cancel, &goal).expect("exact successor was not rewritten");
            assert_eq!(cancel.cancel_generation, 3);
        }
        match result {
            Err(error) => Err(error),
            Ok(Some(OwnerInput::Rejected(reason))) if !publication_fails => {
                assert!(reason.contains(case.expected_reason()), "wrong rejection: {reason}");
                Ok(())
            },
            _ => panic!("rejected head escaped its original classification"),
        }
    };
    let coordinated = coordinate_control(scenario, &scheduler, task).await;
    let report = trace.finish();
    assert!(!report.incomplete);
    assert!(!report.records.iter().any(|event| event.phase == API_BEGIN || event.phase == API_DONE),
        "rejected head must not invoke original scheduler.cancel");
    let events = warnings.0.lock().unwrap();
    if publication_fails {
        assert_original_p_error(coordinated);
        assert_eq!(events.len(), 1, "consumed rejection/error secondary fact must be logged once");
        assert!(events[0]["publication_error"].contains(MATRIX_P_ERROR));
        let (field, message) = if matches!(case, RejectedHead::Closed) {
            ("cancel_input_error", "ordinary publication failed after consuming failed cancellation input")
        } else {
            ("cancel_rejection", "ordinary publication failed after consuming rejected cancellation input")
        };
        assert!(events[0][field].contains(case.expected_reason()), "wrong secondary fact: {:?}", events[0]);
        assert_eq!(events[0]["message"], message);
        assert!(!events[0].contains_key("cancel_result"), "no invented C outcome for a rejected input");
    } else if matches!(case, RejectedHead::Closed) {
        match coordinated {
            Err(error) => assert!(error.to_string().contains("cancel LCM stream closed")),
            Ok(()) => panic!("original required_frame source failure was lost"),
        }
        assert!(events.is_empty());
    } else {
        coordinated.expect("P success retains original rejection result after P");
        assert!(events.is_empty());
    }
    eprintln!("BRIDGE_TAIL_HEAD case={case:?} publication_fails={publication_fails} source_reads=1 frames_consumed={} cancel_calls=0",
        usize::from(!matches!(case, RejectedHead::Closed)));
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit one-head identity/watermark/input-error controls"]
async fn guarded_tail_rejected_head_is_not_bypassed_and_secondary_error_survives() {
    for case in [RejectedHead::Session, RejectedHead::Owner, RejectedHead::Parent,
        RejectedHead::GoalGeneration, RejectedHead::Producer, RejectedHead::Stale,
        RejectedHead::Nonpositive, RejectedHead::Malformed, RejectedHead::Closed] {
        for publication_fails in [false, true] {
            rejected_head_control(case, publication_fails).await;
        }
    }
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit queued replacement remains behind old P and early Cancel settlement"]
async fn guarded_tail_queued_goal_waits_for_old_publication_and_cancel_settlement() {
    run(TailTrigger::Pose, GateSite::AfterAck, true, true).await;
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit real-wrapper eligibility controls with synthetic snapshots; real memq P"]
async fn guarded_tail_wrapper_skips_terminal_and_mismatched_snapshot_windows() {
    for case in [WrapperSnapshotCase::Terminal, WrapperSnapshotCase::SessionMismatch] {
        let goal = sealed_goal("tail-wrapper-exact");
        let (scheduler, task, calls) = matrix_scheduler(&goal, true).await;
        let baseline = count_zero(&calls);
        let transport = LcmTransport::new_real("memq://").expect("wrapper same-context memq");
        let mut node = ModuleNode::new("tail-wrapper-eligibility", "tail wrapper eligibility control",
            crate::graph::ModuleKind::Coordinator);
        node.outputs = vec!["/test/tail-wrapper/cmd".into(), "/test/tail-wrapper/state".into(),
            "/test/tail-wrapper/reached".into(), "/test/tail-wrapper/cancelled".into()];
        let mut states = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.NavigationState")
            .await.expect("wrapper real state stream");
        let trace = TraceSession::start(64);
        let probe = Arc::new(Mutex::new(PublicationProbe::default()));
        let (entered, mut entered_receipt) = oneshot::channel();
        let (release, released) = oneshot::channel();
        let (resumed, mut resumed_receipt) = oneshot::channel();
        probe.lock().unwrap().ack_return_probe = Some(AckReturnProbe {
            target: Target::new(&goal, 1, 1), trace: trace.sink(),
            boundary: Some(Boundary { entered, release: Some(released), resumed }),
            site: GateSite::BeforeDelegate, cancel_completed: None, captured_batch: None,
            observe_all_cancels: true, expected_predelegate_twist: None, wrapper_snapshot: Some(case),
        });
        let scenario = async {
            let mut active = Some(goal.clone());
            let mut telemetry = GuardedNavTelemetry {
                public_session_id: goal.session_id.clone(), owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(), goal_generation: goal.goal_generation,
                producer_runtime_generation: goal.producer_runtime_generation,
                path_revision: 1, ..Default::default()
            };
            // These deliberately synthetic snapshots test the existing wrapper's
            // eligibility and old-snapshot clear; they are not runtime outcomes.
            let snapshot = nuwax_nav_runtime::SchedulerStatus {
                phase: if matches!(case, WrapperSnapshotCase::Terminal) { SchedulerPhase::Idle } else { SchedulerPhase::Controlling },
                session_id: Some(if matches!(case, WrapperSnapshotCase::Terminal) {
                    goal.session_id.clone()
                } else { "synthetic-other-session".into() }),
                last_terminal: matches!(case, WrapperSnapshotCase::Terminal).then_some(TerminalOutcome::Cancelled),
                ..Default::default()
            };
            let (sender, mut cancels) = guarded_test_stream();
            let mut cancel_receipt = send_guarded_test_body_observed(&sender, "nuwax_msgs.CancelNavigation",
                GuardedLoopHarness::cancel_body(&goal, 1));
            let mut publications = GuardedStatusPublications::default();
            let mut goal_buffer = PlannerGoalBuffer::default();
            let mut last_cancel_terminal = None;
            let mut watermark = None;
            let mut arbiter = GuardedNavArbiter { cursor: GuardedNavSource::Goal };
            {
                let future = publish_guarded_tail_observation(&transport, &node, &scheduler, snapshot,
                    &mut telemetry, None, &mut publications, &mut cancels, &mut active,
                    &mut goal_buffer, &mut last_cancel_terminal, &mut watermark, &mut arbiter);
                let scoped = PUBLICATION_PROBE.scope(probe, future);
                tokio::pin!(scoped);
                assert!(futures::poll!(&mut scoped).is_pending());
                assert!(matches!(entered_receipt.try_recv(), Ok(())), "original P did reach pre-delegate gate");
                assert!(matches!(resumed_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
                assert!(matches!(cancel_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
                assert_eq!(count_zero(&calls), baseline);
                release.send(ReleaseMode::Normal).expect("wrapper gate normal release");
                tokio::time::timeout(Duration::from_secs(1), scoped).await.expect("real wrapper P completion")?;
            }
            assert_eq!(resumed_receipt.await.expect("wrapper gate receipt"), BoundaryOutcome::Normal);
            assert!(matches!(cancel_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)));
            assert!(watermark.is_none());
            assert!(last_cancel_terminal.is_none());
            assert_eq!(arbiter.cursor, GuardedNavSource::Goal);
            assert_eq!(count_zero(&calls), baseline);
            match case {
                WrapperSnapshotCase::Terminal => {
                    assert!(active.is_none());
                    assert!(telemetry.public_session_id.is_empty());
                }
                WrapperSnapshotCase::SessionMismatch => {
                    assert_eq!(active.as_ref(), Some(&goal));
                    assert_eq!(telemetry.public_session_id, goal.session_id);
                }
            }
            let frame = tokio::time::timeout(Duration::from_secs(1), states.next()).await
                .expect("real wrapper output").expect("wrapper state stream open")
                .expect("wrapper real frame");
            let state = schema::decode_navigation_state_body(&frame.payload).expect("wrapper real state body");
            assert_eq!(state.session_id, goal.session_id, "publication retains pre-clear identity");
            assert_eq!(state.cancel_generation, 0, "unread queued Cancel cannot change output generation");
            assert!(cancels.next().now_or_never().flatten().is_some(), "exact Cancel remains queued");
            cancel_receipt.await.expect("only test's explicit later dequeue consumed exact Cancel");
            Ok(())
        };
        coordinate_control(scenario, &scheduler, task).await.expect("real wrapper qualification and cleanup");
        let report = trace.finish();
        assert!(!report.incomplete);
        assert!(!report.records.iter().any(|event| event.phase == API_BEGIN || event.phase == API_DONE));
        eprintln!("BRIDGE_TAIL_WRAPPER case={case:?} synthetic_snapshot=true real_publication=true no_prefetch=true");
    }
}
