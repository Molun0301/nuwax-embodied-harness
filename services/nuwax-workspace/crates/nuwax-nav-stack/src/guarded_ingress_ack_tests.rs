// Included only inside #[cfg(all(test, feature = "guard-runtime"))] guarded_tests,
// under its existing surface-recovery-qualification feature. Review-only draft.
use nuwax_nav_runtime::qualification_timing::{TraceReport, TraceSession, TraceSink};

const ACK_READY: &str = "bridge_ack_test.real_ack_ready";
const GATE_HELD: &str = "bridge_ack_test.gate_held";
const GATE_DISABLED: &str = "bridge_ack_test.gate_disabled";
const ENQUEUED: &str = "bridge_ack_test.cancel_enqueued";
const PROGRESSED: &str = "bridge_ack_test.scheduler_progress_while_held";
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
}
impl Target {
    fn new(goal: &schema::GoalEnvelope, path_revision: i64, cancel_generation: i64) -> Self {
        Self {
            session_id: goal.session_id.clone(), owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
            path_revision, cancel_generation, pose_source_timestamp_ns: 2_000_000_000,
        }
    }
    fn publication(&self, status: &nuwax_nav_runtime::SchedulerStatus, t: &GuardedNavTelemetry) -> bool {
        status.phase == SchedulerPhase::Controlling && status.last_terminal.is_none()
            && status.session_id.as_deref() == Some(self.session_id.as_str())
            && status.pose_source_timestamp_ns == self.pose_source_timestamp_ns
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
}

// Called only after the original real publish_batch .await and ? succeeded,
// and after the original publish span closed. Errors bypass this hook unchanged.
// The task-local PublicationProbe binds the gate to one original loop instance.
pub(super) async fn hold_publication_ack(
    status: &nuwax_nav_runtime::SchedulerStatus, telemetry: &GuardedNavTelemetry,
) {
    let selected = PUBLICATION_PROBE.try_with(|shared| {
        let mut shared = shared.lock().unwrap();
        let probe = shared.ack_return_probe.as_mut()?;
        if !probe.target.publication(status, telemetry) { return None; }
        probe.boundary.take().map(|boundary| (probe.trace.clone(), boundary))
    }).ok().flatten();
    let Some((trace, boundary)) = selected else { return; };
    // No PublicationProbe mutex or watch borrow is retained over the await.
    trace.mark(ACK_READY, 0);
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
    let trace = PUBLICATION_PROBE.try_with(|shared| {
        let shared = shared.lock().unwrap();
        shared.ack_return_probe.as_ref().filter(|probe| probe.target.cancel(cancel))
            .map(|probe| probe.trace.clone())
    }).ok().flatten();
    if let Some(trace) = trace { trace.mark(phase, 0); }
}

// Invoked at the original stream's existing observed-receipt send, before it
// returns the frame. After installing this probe these two tests inject exactly
// one observed frame, the Cancel; setup/trigger messages are not observed ones.
pub(super) fn record_observed_dequeue() {
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
    h: &mut GuardedLoopHarness, hold: bool, trace: &TraceSink, release_owner: &mut ReleaseOwner,
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
    let (entered, installation) = oneshot::channel();
    let (resumed, mut resume_receipt) = oneshot::channel();
    let release = if hold {
        let (sender, receiver) = oneshot::channel();
        release_owner.0 = Some(sender);
        Some(receiver)
    } else { None };
    h.publication_probe.lock().unwrap().ack_return_probe = Some(AckReturnProbe {
        target: Target::new(&goal, path.path_revision, 1), trace: trace.clone(),
        boundary: Some(Boundary { entered, release, resumed }),
    });
    // Ordinary input through the original selector, not a direct helper call.
    // Existing synthetic pose timestamp policy and five-second TTL are unchanged.
    h.send_pose(2);
    tokio::time::timeout(Duration::from_secs(1), installation).await
        .map_err(|_| "matching real publish ACK boundary did not enter".to_string())?
        .map_err(|_| "ACK boundary installation sender closed".to_string())?;

    let mut dequeue_receipt = send_guarded_test_body_observed(
        &h.cancels, "nuwax_msgs.CancelNavigation", GuardedLoopHarness::cancel_body(&goal, 1),
    );
    // Both selectors explicitly use current_thread. No actor can run between
    // synchronous test-stream enqueue and this marker; this is not native enqueue.
    trace.mark(ENQUEUED, 0);
    if hold {
        // Wait for actual continued control, rather than checking immediately on
        // a current-thread runtime before any other task could make progress.
        let baseline = h.scheduler.status().applied_commands;
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow_and_update().clone();
                if snapshot.last_terminal.is_some() || snapshot.phase != SchedulerPhase::Controlling {
                    return Err("control stopped while the original bridge was held".to_string());
                }
                if snapshot.applied_commands > baseline
                    && snapshot.last_applied_twist.is_some_and(|twist| twist != Twist2::ZERO) {
                    return Ok(());
                }
                status.changed().await.map_err(|error| error.to_string())?;
            }
        }).await.map_err(|_| "no real scheduler progress while bridge gate was held".to_string())??;
        trace.mark(PROGRESSED, 0);
        if !matches!(resume_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)) {
            return Err("ACK gate was not still closed at checkpoint".into());
        }
        if !matches!(dequeue_receipt.try_recv(), Err(oneshot::error::TryRecvError::Empty)) {
            return Err("Cancel had already left the original stream while the gate was closed".into());
        }
        let snapshot = h.scheduler.status();
        if snapshot.phase != SchedulerPhase::Controlling || snapshot.last_terminal.is_some()
            || snapshot.session_id.as_deref() != Some(goal.session_id.as_str())
            || h.motion_call_count("zero:") != zero_count_before {
            return Err("active exact-goal prerequisite changed during closed checkpoint".into());
        }
        trace.mark(CHECKPOINT, 0);
        if !release_owner.normal(trace) { return Err("normal gate release was not delivered".into()); }
    }
    let boundary = tokio::time::timeout(Duration::from_secs(1), &mut resume_receipt).await
        .map_err(|_| "ACK gate return was not observed".to_string())?
        .map_err(|_| "ACK gate return sender closed".to_string())?;
    tokio::time::timeout(Duration::from_secs(1), &mut dequeue_receipt).await
        .map_err(|_| "Cancel was not dequeued by the original stream".to_string())?
        .map_err(|_| "Cancel dequeue receipt closed".to_string())?;
    let zero = h.wait_for_correlated_zero(&goal, 1).await;
    let terminal = h.scheduler.status();
    if zero.producer_runtime_generation != goal.producer_runtime_generation
        || !zero.message.starts_with("CANCELLED:")
        || terminal.last_zero_ack_id.as_deref() != Some(zero.zero_ack_id.as_str())
        || i64::try_from(terminal.last_fence_monotonic_ns).ok() != Some(zero.zero_applied_monotonic_ns) {
        return Err("external exact cancellation proof did not match the scheduler fence".into());
    }
    trace.mark(ZEROED, 0);
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

async fn run(hold: bool) {
    // Original harness bootstrapping is unchanged. Once it returns, every
    // scenario failure/panic is collected before the owned tasks are cleaned up.
    let mut harness = GuardedLoopHarness::spawn().await;
    let trace = TraceSession::start(256);
    let sink = trace.sink();
    let mut release_owner = ReleaseOwner::default();
    let result = std::panic::AssertUnwindSafe(scenario(&mut harness, hold, &sink, &mut release_owner))
        .catch_unwind().await;
    release_owner.cleanup(); // Error/drop releases are never normal release.
    let cleanup = cleanup(&mut harness).await;
    drop(harness); // Original subscription/context field teardown after task joins.
    let mut report = trace.finish();
    report.incomplete |= !cleanup.loop_stopped || !cleanup.scheduler_stopped;
    let summary = match &result {
        Ok(result) => format!("{result:?}"), Err(_) => "scenario panic caught before cleanup".into(),
    };
    eprintln!("BRIDGE_ACK_RETURN hold={hold} scenario={summary} cleanup={cleanup:?}");
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
    let ack = unique(&report, ACK_READY);
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
        let progressed = unique(&report, PROGRESSED);
        let checkpoint = unique(&report, CHECKPOINT);
        let release = unique(&report, RELEASE_NORMAL);
        let resumed = unique(&report, GATE_RESUMED);
        assert!(ack <= held && held <= queued && queued <= progressed && progressed <= checkpoint
            && checkpoint <= release && release <= resumed && resumed <= dequeued);
        assert!(!report.records.iter().any(|event| event.phase == GATE_DISABLED));
    } else {
        assert!(ack <= unique(&report, GATE_DISABLED));
        assert!(!report.records.iter().any(|event| matches!(event.phase,
            GATE_HELD | PROGRESSED | CHECKPOINT | RELEASE_NORMAL | GATE_RESUMED)));
    }
}

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit original bridge ACK-return dependency; no natural latency attribution"]
async fn guarded_cancel_ingress_waits_for_observation_ack_return() { run(true).await; }

#[tokio::test(flavor = "current_thread")]
#[ignore = "explicit unheld control for the original bridge Cancel ingress and exact fence"]
async fn guarded_cancel_ingress_without_ack_return_gate() { run(false).await; }
