//! Runtime side of the bounded owner. The drain task is independent of tick and
//! feedback waiters and keeps running after logical stop. No backend call runs here.
use super::compute_backend::*;
use super::*;
use crate::explore_compute_owner as exchange;
use futures::FutureExt;
use std::panic::AssertUnwindSafe;
use std::sync::atomic::AtomicBool;
use std::sync::Weak;
use tokio::sync::Notify;

type Owner = exchange::OwnerHandle<ExploreComputeBackend>;

/// Weak reconstruction keeps service supervision from owning its runtime family.
/// A strong handle exists only while a retained control action actually executes.
#[derive(Clone)]
struct WeakRuntimeContext {
    inner: Weak<Mutex<ExploreRuntimeInner>>,
    service: Weak<ComputeService>,
    tick_gate: Weak<Mutex<()>>,
    expected_map_frame: Arc<str>,
    readiness_cache: Weak<SyncMutex<ExplorationReadinessDetail>>,
    snapshot_cache: Weak<RwLock<ExploreRuntimeSnapshot>>,
    navigation_sink: Weak<RwLock<Option<Arc<dyn ExploreNavigationSink>>>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    generation: Weak<AtomicU64>,
    orphan_cancellations: Weak<OrphanCancellationRegistry>,
    #[cfg(test)]
    attribution: Option<Arc<attribution::Attribution>>,
    #[cfg(test)]
    homeward_test_hook: Option<Arc<dyn Fn() + Send + Sync>>,
}
impl WeakRuntimeContext {
    fn capture(runtime: &ExploreRuntimeHandle) -> Self {
        Self {
            inner: Arc::downgrade(&runtime.inner),
            service: Arc::downgrade(&runtime.compute_service),
            tick_gate: Arc::downgrade(&runtime.tick_gate),
            expected_map_frame: runtime.expected_map_frame.clone(),
            readiness_cache: Arc::downgrade(&runtime.readiness_cache),
            snapshot_cache: Arc::downgrade(&runtime.snapshot_cache),
            navigation_sink: Arc::downgrade(&runtime.navigation_sink),
            frontier_tx: runtime.frontier_tx.clone(),
            generation: Arc::downgrade(&runtime.generation),
            orphan_cancellations: Arc::downgrade(&runtime.orphan_cancellations),
            #[cfg(test)]
            attribution: runtime.attribution.clone(),
            #[cfg(test)]
            homeward_test_hook: runtime.homeward_test_hook.clone(),
        }
    }
    fn upgrade(&self) -> Option<ExploreRuntimeHandle> {
        Some(ExploreRuntimeHandle {
            inner: self.inner.upgrade()?,
            compute_service: self.service.upgrade()?,
            tick_gate: self.tick_gate.upgrade()?,
            expected_map_frame: self.expected_map_frame.clone(),
            readiness_cache: self.readiness_cache.upgrade()?,
            snapshot_cache: self.snapshot_cache.upgrade()?,
            navigation_sink: self.navigation_sink.upgrade()?,
            frontier_tx: self.frontier_tx.clone(),
            generation: self.generation.upgrade()?,
            orphan_cancellations: self.orphan_cancellations.upgrade()?,
            #[cfg(test)]
            attribution: self.attribution.clone(),
            #[cfg(test)]
            homeward_test_hook: self.homeward_test_hook.clone(),
        })
    }
    fn retain_stop(
        &self,
        inner: &mut ExploreRuntimeInner,
        reason: ExplorationStopReason,
        message: String,
    ) -> Result<(), String> {
        if inner
            .compute
            .control_stop
            .as_ref()
            .is_some_and(|record| record.desired.serial == ControlSerial::Exhausted)
        {
            return Ok(()); // Exact terminal action already retained; allocate nothing.
        }
        let generation = self
            .generation
            .upgrade()
            .ok_or("exploration runtime generation owner unavailable")?;
        let identifiers = (|| {
            let next = generation
                .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |value| {
                    value.checked_add(1)
                })
                .map_err(|_| "exploration generation exhausted".to_string())?
                + 1;
            Ok::<_, String>((
                next,
                ControlSerial::Counted(inner.compute.next_lifecycle()?),
            ))
        })();
        let (next, serial, reason, message) = match identifiers {
            Ok((next, serial)) => (next, serial, reason, message),
            Err(error) => {
                // No new session can ever be admitted after exhaustion. Retain one
                // distinct terminal token, never wrap or reuse a counted action ID.
                inner.compute.fault.get_or_insert(error.clone());
                inner.logical_active = false;
                inner.compute.start_reservation = None;
                inner.compute.start_abandoned = None;
                if let Some(owner) = &inner.compute.owner {
                    owner.stop_admission();
                }
                if inner
                    .compute
                    .control_stop
                    .as_ref()
                    .is_some_and(|record| record.desired.serial == ControlSerial::Exhausted)
                {
                    return Ok(()); // Join the exact already retained terminal action.
                }
                (
                    generation.load(Ordering::SeqCst),
                    ControlSerial::Exhausted,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!("{error}; {message}"),
                )
            }
        };
        let action = ComputeStopAction {
            serial,
            generation: next,
            parent: inner.last_request_id.clone(),
            session_generation: inner.session_generation,
            child_lease: inner.active_child.as_ref().map(|child| child.lease.clone()),
            reason,
            message,
        };
        inner.generation = next;
        inner.request_compute_stop(reason);
        inner.last_stop = Some(reason);
        inner.last_stop_reason = stop_reason_str(reason).into();
        inner.last_poll = Some(ExplorationPoll::Stop(reason));
        match &mut inner.compute.control_stop {
            Some(record) => record.desired = action,
            None => {
                inner.compute.control_stop = Some(ComputeControlRecord {
                    desired: action,
                    running: None,
                    settled: None,
                })
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ControlSerial {
    Counted(u64),
    Exhausted,
}

#[derive(Clone)]
pub(super) struct ComputeStopAction {
    serial: ControlSerial,
    pub generation: u64,
    pub parent: String,
    pub session_generation: u64,
    pub child_lease: Option<ChildNavigationLease>,
    pub reason: ExplorationStopReason,
    pub message: String,
}
impl ComputeStopAction {
    pub(super) fn matches(&self, inner: &ExploreRuntimeInner) -> bool {
        self.generation == inner.generation
            && self.parent == inner.last_request_id
            && self.session_generation == inner.session_generation
            && match (&self.child_lease, &inner.active_child) {
                (None, None) => true,
                (Some(expected), Some(child)) => expected.matches(&child.lease),
                _ => false,
            }
    }
}
struct ComputeControlRecord {
    desired: ComputeStopAction,
    running: Option<ControlSerial>,
    settled: Option<ControlSerial>,
}
impl ComputeControlRecord {
    fn pending(&self) -> bool {
        self.running.is_some() || self.settled != Some(self.desired.serial)
    }
}

pub(super) struct ComputeService {
    pub wake: Arc<Notify>,
    pub changed: Arc<Notify>,
    driver: OnceLock<Mutex<JoinHandle<()>>>,
    driver_joined: AtomicBool,
    terminal: watch::Sender<Option<Result<(), String>>>,
}
impl ComputeService {
    pub(super) fn new() -> Self {
        let (terminal, _) = watch::channel(None);
        Self {
            wake: Arc::new(Notify::new()),
            changed: Arc::new(Notify::new()),
            driver: OnceLock::new(),
            driver_joined: AtomicBool::new(false),
            terminal,
        }
    }
    pub(super) fn ensure(
        &self,
        runtime: &ExploreRuntimeHandle,
        state: &mut ComputeState,
    ) -> Result<(), String> {
        if state.shutdown {
            return Err("exploration compute service is shut down".into());
        }
        if let Some(error) = &state.fault {
            return Err(error.clone());
        }
        if state.owner.is_some() {
            return Ok(());
        }
        let owner = exchange::ComputeOwner::spawn(ExploreComputeBackend::new(
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront),
        ))
        .map_err(|error| format!("exploration compute owner spawn failed: {error}"))?;
        state.owner = Some(owner.handle());
        let task = tokio::spawn(drain_owner(
            WeakRuntimeContext::capture(runtime),
            Arc::downgrade(&runtime.inner),
            runtime.frontier_tx.clone(),
            runtime.snapshot_cache.clone(),
            self.wake.clone(),
            self.changed.clone(),
            self.terminal.clone(),
            owner,
        ));
        self.driver
            .set(Mutex::new(task))
            .map_err(|_| "exploration compute driver already installed".to_string())?;
        Ok(())
    }
    pub(super) async fn wait_terminal(&self) -> Result<(), String> {
        let mut terminal = self.terminal.subscribe();
        loop {
            if let Some(result) = terminal.borrow().clone() {
                return result;
            }
            terminal
                .changed()
                .await
                .map_err(|_| "exploration compute terminal channel closed".to_string())?;
        }
    }
    pub(super) async fn join(&self) -> Result<(), String> {
        let Some(driver) = self.driver.get() else {
            return Ok(());
        };
        let mut task = driver.lock().await;
        if !self.driver_joined.load(Ordering::Acquire) {
            // Await the retained handle by mutable reference. Cancelling shutdown
            // releases only this guard; it never drops or detaches the handle.
            let result = (&mut *task).await;
            self.driver_joined.store(true, Ordering::Release);
            if let Err(error) = result {
                let message = format!("exploration compute driver join failed: {error}");
                self.terminal.send_replace(Some(Err(message.clone())));
                return Err(message);
            }
        }
        self.terminal.borrow().clone().unwrap_or_else(|| {
            Err("exploration compute driver exited without terminal settlement".into())
        })
    }
}
impl Drop for ComputeService {
    fn drop(&mut self) {
        // The driver owns the native JoinHandle, not this async-drop path. It has
        // only a Weak reference to inner, so final family destruction wakes it to
        // close and join the native worker without blocking the async executor.
        self.wake.notify_one();
    }
}

#[derive(Clone)]
pub(super) struct ComputeBasis {
    generation: u64,
    parent: String,
    history: u64,
    frame: i64,
    committed_map: Option<schema::CommittedMapIdentity>,
    input: ComputeInput,
    return_on_completion: bool,
    mission_present: bool,
    no_frontier_timeout: Duration,
    #[cfg(test)]
    inspection: bool,
}
impl ComputeBasis {
    fn current(&self, inner: &ExploreRuntimeInner) -> bool {
        self.current_at_history(inner, self.history)
    }
    fn current_at_history(&self, inner: &ExploreRuntimeInner, expected_history: u64) -> bool {
        #[cfg(test)]
        if self.inspection {
            return inner.logical_active
                && self.generation == inner.generation
                && self.parent == inner.last_request_id
                && expected_history == inner.compute.history;
        }
        // Tick admission validates the sampled pose. Completion can outlive
        // that sample's receipt TTL; navigation consumers check their live pose.
        // Map identity and the current installed map receipt still qualify here.
        if !inner.logical_active
            || self.generation != inner.generation
            || self.parent != inner.last_request_id
            || expected_history != inner.compute.history
            || self.frame != input_frame(inner)
            || inner.pending_child_cancellation.is_some()
            || inner
                .compute
                .feedback
                .as_ref()
                .is_some_and(|feedback| feedback.result.is_none())
            || inner.costmap_received_monotonic_ns == 0
            || now_monotonic_nanos().saturating_sub(inner.costmap_received_monotonic_ns)
                > inner.costmap_receipt_ttl.as_nanos() as u64
        {
            return false;
        }
        match (&self.input, &inner.surface_mode, &inner.costmap_input) {
            (ComputeInput::Planar(before), None, Some(now)) => {
                self.committed_map == inner.committed_map && Arc::ptr_eq(before, now)
            }
            (ComputeInput::Surface { original, .. }, Some(mode), _) => {
                // A query-equivalent composite receipt may advance without changing
                // the frozen computation. Keep its original identity for GoalIntent;
                // domain changes and conflicting/non-monotonic commits still fail.
                let map_current = self.committed_map == inner.committed_map
                    || self
                        .committed_map
                        .as_ref()
                        .zip(inner.committed_map.as_ref())
                        .is_some_and(|(before, now)| {
                            crate::nav_runtime::same_committed_map_domain(before, now)
                                && before.revision < now.revision
                                && before.commit_sequence < now.commit_sequence
                        });
                mode.accepting
                    && mode.input_ready
                    && map_current
                    && mode
                        .receiver
                        .freeze()
                        .is_ok_and(|now| original.same_planning_query_view(&now))
            }
            _ => false,
        }
    }
}
fn input_frame(inner: &ExploreRuntimeInner) -> i64 {
    inner
        .surface_mode
        .as_ref()
        .and_then(|mode| mode.latest.as_ref())
        .map_or(inner.costmap_frame_generation, |ticket| {
            ticket.map_frame_generation
        })
}

pub(super) struct JobRecord {
    terminal_action: Option<ControlSerial>,
    id: exchange::Id,
    basis: ComputeBasis,
    abandoned: Arc<AtomicBool>,
    prepared: Option<exchange::Prepared<ComputeProjection>>,
    decision: Option<exchange::DecisionAck>,
    settled: Option<exchange::Settled<ComputeFinalized>>,
    result: Option<Result<Arc<ComputeFinalized>, String>>,
    goal_handled: bool,
    released: bool,
    retirement: Option<exchange::Id>,
    output_bookkept: bool,
}
/// Cancellation only changes a flag; it never consumes a prepared/final value.
/// The record and accepted-goal lineage remain in inner until the drain retires it.
pub(super) struct JobTicket {
    pub id: exchange::Id,
    basis: ComputeBasis,
    abandoned: Arc<AtomicBool>,
    wake: Arc<Notify>,
}
impl Drop for JobTicket {
    fn drop(&mut self) {
        self.abandoned.store(true, Ordering::Release);
        self.wake.notify_one();
    }
}
impl JobTicket {
    pub(super) fn current(&self, inner: &ExploreRuntimeInner) -> bool {
        self.basis.current(inner)
    }
    /// Requalify the original sampled advice after its own exact accepted
    /// Cancelled feedback. Only that acknowledgment's one history increment is
    /// accounted for; source, frame, parent, generation and receipt checks remain.
    pub(super) fn current_after_feedback(
        &self,
        inner: &ExploreRuntimeInner,
        ticket: &FeedbackTicket,
    ) -> bool {
        let Some(record) = inner
            .compute
            .feedback
            .as_ref()
            .filter(|record| record.id == ticket.id && record.identity == ticket.identity)
        else {
            return false;
        };
        let FeedbackIdentity::Internal {
            generation,
            lineage,
            goal,
            feedback: ExplorationGoalFeedback::Cancelled,
        } = &record.identity
        else {
            return false;
        };
        if *generation != self.basis.generation
            || lineage != &format!("poll:{}:{}", self.id.incarnation, self.id.sequence)
            || record.history_before != self.basis.history
            || !matches!(record.result, Some(Ok(())))
        {
            return false;
        }
        let Some(history_after) = record.history_after else {
            return false;
        };
        if record.history_before.checked_add(1) != Some(history_after) {
            return false;
        }
        let matching_goal = inner
            .compute
            .job
            .as_ref()
            .filter(|job| job.id == self.id)
            .and_then(|job| job.result.as_ref())
            .and_then(|result| result.as_ref().ok())
            .and_then(|result| selected_goal(&result.projection))
            .is_some_and(|selected| &selected.id == goal);
        matching_goal && self.basis.current_at_history(inner, history_after)
    }
    pub(super) fn finish(&self, inner: &mut ExploreRuntimeInner) {
        if let Some(job) = inner.compute.job.as_mut().filter(|job| job.id == self.id) {
            job.released = true;
            // Every normal return path either leaves a child/pending dispatch,
            // supplies feedback, or contains no selected source goal.
            job.goal_handled = true;
        }
        self.wake.notify_one();
    }
    pub(super) fn feedback_handoff(&self, inner: &mut ExploreRuntimeInner) {
        if let Some(job) = inner.compute.job.as_mut().filter(|job| job.id == self.id) {
            job.goal_handled = true;
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum FeedbackIdentity {
    External {
        child: String,
        terminal: ChildNavigationResult,
    },
    Internal {
        generation: u64,
        lineage: String,
        goal: String,
        feedback: ExplorationGoalFeedback,
    },
}
struct FeedbackRecord {
    id: exchange::Id,
    identity: FeedbackIdentity,
    history_before: u64,
    history_after: Option<u64>,
    result: Option<Result<(), String>>,
    ack: Option<ComputeFeedbackAck>,
    completion: Arc<SyncMutex<Option<Result<(), String>>>>,
    message: Option<&'static str>,
    reached: bool,
}
#[derive(Clone)]
pub(super) struct FeedbackTicket {
    id: exchange::Id,
    identity: FeedbackIdentity,
    completion: Arc<SyncMutex<Option<Result<(), String>>>>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LifecycleKind {
    Install,
    Export,
    Stop,
    Retire,
}
struct LifecycleRecord {
    id: exchange::Id,
    sequence: u64,
    kind: LifecycleKind,
    result: Option<Result<Arc<ComputeLifecycleOutput>, String>>,
    ack: Option<ComputeLifecycleAck>,
    abandoned: Arc<AtomicBool>,
}
pub(super) struct LifecycleTicket {
    sequence: u64,
    abandoned: Arc<AtomicBool>,
    wake: Arc<Notify>,
}
impl Drop for LifecycleTicket {
    fn drop(&mut self) {
        self.abandoned.store(true, Ordering::Release);
        self.wake.notify_one();
    }
}

pub(super) struct ComputeState {
    owner: Option<Owner>,
    job: Option<JobRecord>,
    feedback: Option<FeedbackRecord>,
    lifecycle: Option<LifecycleRecord>,
    desired_stop: Option<(u64, ExplorationStopReason)>,
    control_stop: Option<ComputeControlRecord>,
    fault_action_scheduled: bool,
    stop_settled: u64,
    lifecycle_sequence: u64,
    pub history: u64,
    pub fault: Option<String>,
    pub start_reservation: Option<u64>,
    start_abandoned: Option<Arc<AtomicBool>>,
    pub shutdown: bool,
    #[cfg(test)]
    pub panic_driver: bool,
    #[cfg(test)]
    pub hold_install_ack: bool,
}
impl Default for ComputeState {
    fn default() -> Self {
        Self {
            owner: None,
            job: None,
            feedback: None,
            lifecycle: None,
            desired_stop: None,
            control_stop: None,
            fault_action_scheduled: false,
            stop_settled: 0,
            lifecycle_sequence: 0,
            history: 0,
            fault: None,
            start_reservation: None,
            start_abandoned: None,
            shutdown: false,
            #[cfg(test)]
            panic_driver: false,
            #[cfg(test)]
            hold_install_ack: false,
        }
    }
}
impl ComputeState {
    #[cfg(test)]
    pub(super) fn lifecycle_counter_for_tests(&self) -> u64 {
        self.lifecycle_sequence
    }
    #[cfg(test)]
    pub(super) fn status(&self) -> Option<exchange::Status> {
        self.owner.as_ref().map(Owner::status)
    }
    pub(super) fn control_pending(&self) -> bool {
        self.control_stop
            .as_ref()
            .is_some_and(ComputeControlRecord::pending)
    }
    fn next_lifecycle(&mut self) -> Result<u64, String> {
        self.lifecycle_sequence = self
            .lifecycle_sequence
            .checked_add(1)
            .ok_or_else(|| "exploration lifecycle sequence exhausted".to_string())?;
        Ok(self.lifecycle_sequence)
    }
    fn bump_history(&mut self) -> Result<(), String> {
        self.history = self
            .history
            .checked_add(1)
            .ok_or_else(|| "exploration history revision exhausted".to_string())?;
        Ok(())
    }
}

impl ExploreRuntimeInner {
    pub(super) fn request_compute_stop(&mut self, reason: ExplorationStopReason) {
        self.logical_active = false;
        self.compute.start_reservation = None;
        self.compute.start_abandoned = None;
        if let Some(owner) = &self.compute.owner {
            owner.stop_admission();
        }
        if self.compute.shutdown || self.compute.owner.is_none() {
            return; // Final closure cannot admit a new lifecycle against an exited owner.
        }
        match self.compute.next_lifecycle() {
            Ok(sequence) => self.compute.desired_stop = Some((sequence, reason)),
            Err(error) => self.compute.fault = Some(error),
        }
    }
}

pub(super) struct StartTicket {
    abandoned: Arc<AtomicBool>,
    wake: Arc<Notify>,
}
impl Drop for StartTicket {
    fn drop(&mut self) {
        self.abandoned.store(true, Ordering::Release);
        self.wake.notify_one();
    }
}
impl ExploreRuntimeHandle {
    #[cfg(test)]
    pub(super) async fn inject_compute_control_stop_for_tests(
        &self,
        generation_exhausted: bool,
        lifecycle_exhausted: bool,
    ) -> Result<(), String> {
        let mut inner = self.inner.lock().await;
        if generation_exhausted {
            self.generation.store(u64::MAX, Ordering::SeqCst);
        }
        if lifecycle_exhausted {
            inner.compute.lifecycle_sequence = u64::MAX;
        }
        WeakRuntimeContext::capture(self).retain_stop(
            &mut inner,
            ExplorationStopReason::ConsecutiveFailuresExceeded,
            "injected counter-boundary control stop".into(),
        )?;
        drop(inner);
        self.compute_service.wake.notify_one();
        self.compute_service.changed.notify_waiters();
        Ok(())
    }

    pub(super) fn capture_compute_context(
        &self,
        inner: &ExploreRuntimeInner,
        pose: &ExploreRobotPose,
        now_ns: u64,
    ) -> Result<ComputeContext, String> {
        let input = if let Some(mode) = &inner.surface_mode {
            ComputeInput::Surface {
                original: mode.receiver.freeze().map_err(|error| error.to_string())?,
                config: mode.config.clone()?,
            }
        } else {
            ComputeInput::Planar(inner.costmap_input.clone().ok_or("返航二维图缺失")?)
        };
        Ok(ComputeContext {
            #[cfg(test)]
            attribution: self.attribution.clone(),
            #[cfg(test)]
            homeward_test_hook: self.homeward_test_hook.clone(),
            input,
            robot_pose: PoseStamped {
                frame_id: if inner.surface_mode.is_some() {
                    NavFrameId::from_static(&pose.frame_id)
                } else {
                    NavFrameId::Map
                },
                stamp: Timestamp {
                    nanos: pose.source_timestamp_ns as i64,
                    clock: ClockKind::Unix,
                },
                pose: Pose3 {
                    position: Point3 {
                        x: pose.x,
                        y: pose.y,
                        z: pose.z,
                    },
                    orientation: pose.orientation,
                },
            },
            now: Timestamp {
                nanos: now_ns as i64,
                clock: ClockKind::Unix,
            },
            config: inner.active_config.clone().ok_or("活动探索配置缺失")?,
            calibration: inner.mission_calibration.clone(),
        })
    }
    pub(super) fn reserve_start_locked(
        &self,
        inner: &mut ExploreRuntimeInner,
        generation: u64,
    ) -> StartTicket {
        let abandoned = Arc::new(AtomicBool::new(false));
        inner.compute.start_reservation = Some(generation);
        inner.compute.start_abandoned = Some(abandoned.clone());
        StartTicket {
            abandoned,
            wake: self.compute_service.wake.clone(),
        }
    }

    pub(super) fn admit_compute_locked(
        &self,
        inner: &mut ExploreRuntimeInner,
        context: &ComputeContext,
        request: ComputeRequest,
    ) -> Result<JobTicket, String> {
        if let Some(error) = &inner.compute.fault {
            return Err(error.clone());
        }
        // A released result may be retired synchronously from outer bookkeeping;
        // its payload's destruction occurs outside this lock in the drain task.
        if inner.compute.job.is_some() {
            return Err("exploration compute transaction is still settling".into());
        }
        if !inner.logical_active
            || inner
                .compute
                .feedback
                .as_ref()
                .is_some_and(|record| record.result.is_none())
        {
            return Err("exploration compute admission is waiting for lifecycle/feedback".into());
        }
        let owner = inner
            .compute
            .owner
            .as_ref()
            .ok_or("exploration compute owner missing")?;
        let basis = ComputeBasis {
            generation: inner.generation,
            parent: inner.last_request_id.clone(),
            history: inner.compute.history,
            frame: input_frame(inner),
            committed_map: inner.committed_map.clone(),
            input: context.input.clone(),
            return_on_completion: context.config.return_policy.return_on_completion,
            mission_present: matches!(
                &request,
                ComputeRequest::Selection(SelectionRequest {
                    progress: Some(_),
                    ..
                })
            ),
            no_frontier_timeout: context
                .config
                .stop_conditions
                .no_frontier_timeout_ms
                .map(Duration::from_millis)
                .unwrap_or(DEFAULT_NO_FRONTIER_STABILITY),
            #[cfg(test)]
            inspection: matches!(&request, ComputeRequest::InspectCheckpoint { .. }),
        };
        let id = owner.submit(request).map_err(|error| error.to_string())?;
        let abandoned = Arc::new(AtomicBool::new(false));
        inner.compute.job = Some(JobRecord {
            terminal_action: None,
            id,
            basis: basis.clone(),
            abandoned: abandoned.clone(),
            prepared: None,
            decision: None,
            settled: None,
            result: None,
            goal_handled: false,
            released: false,
            retirement: None,
            output_bookkept: false,
        });
        self.compute_service.wake.notify_one();
        Ok(JobTicket {
            id,
            basis,
            abandoned,
            wake: self.compute_service.wake.clone(),
        })
    }
    pub(super) async fn await_compute(
        &self,
        ticket: &JobTicket,
    ) -> Result<Arc<ComputeFinalized>, String> {
        loop {
            let notified = self.compute_service.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            {
                let guard = self.inner.lock().await;
                if let Some(error) = &guard.compute.fault {
                    return Err(error.clone());
                }
                let job = guard
                    .compute
                    .job
                    .as_ref()
                    .filter(|job| job.id == ticket.id)
                    .ok_or("exploration compute transaction retired")?;
                if let Some(result) = &job.result {
                    let waiting_for_exact_terminal = job.terminal_action.is_some_and(|serial| {
                        guard.compute.control_stop.as_ref().is_some_and(|record| {
                            record.desired.serial == serial
                                && record.desired.generation == guard.generation
                                && record.desired.parent == guard.last_request_id
                                && record.desired.session_generation == guard.session_generation
                                && record.settled != Some(serial)
                        })
                    });
                    if !waiting_for_exact_terminal {
                        return result.clone();
                    }
                }
            }
            notified.await;
        }
    }
    pub(super) fn admit_feedback_locked(
        &self,
        inner: &mut ExploreRuntimeInner,
        identity: FeedbackIdentity,
        goal_id: String,
        feedback: ExplorationGoalFeedback,
        message: Option<&'static str>,
        reached: bool,
    ) -> Result<FeedbackTicket, String> {
        if let Some(record) = &inner.compute.feedback {
            if record.identity == identity {
                return Ok(FeedbackTicket {
                    id: record.id,
                    identity,
                    completion: record.completion.clone(),
                });
            }
            if record.result.is_none() {
                return Err("another exploration feedback is awaiting acknowledgment".into());
            }
        }
        if !inner.logical_active {
            return Err("exploration feedback admission closed".into());
        }
        let owner = inner
            .compute
            .owner
            .as_ref()
            .ok_or("exploration compute owner missing")?;
        // This immutable primitive admission and retained identity are one outer
        // critical section with request_compute_stop. Err is never accepted.
        let id = owner
            .submit_feedback(ComputeFeedback {
                goal_id: goal_id.clone(),
                feedback,
            })
            .map_err(|error| error.to_string())?;
        if let Some(job) = inner.compute.job.as_mut() {
            if job
                .result
                .as_ref()
                .and_then(|result| result.as_ref().ok())
                .is_some_and(|value| {
                    selected_goal(&value.projection).is_some_and(|goal| goal.id == goal_id)
                })
            {
                job.goal_handled = true;
            }
        }
        let completion = Arc::new(SyncMutex::new(None));
        inner.compute.feedback = Some(FeedbackRecord {
            id,
            identity: identity.clone(),
            history_before: inner.compute.history,
            history_after: None,
            result: None,
            ack: None,
            completion: completion.clone(),
            message,
            reached,
        });
        self.compute_service.wake.notify_one();
        Ok(FeedbackTicket {
            id,
            identity,
            completion,
        })
    }
    pub(super) fn existing_external_feedback_locked(
        &self,
        inner: &ExploreRuntimeInner,
        child: &str,
        terminal: &ChildNavigationResult,
    ) -> Result<Option<FeedbackTicket>, String> {
        let Some(record) = &inner.compute.feedback else {
            return Ok(None);
        };
        let FeedbackIdentity::External {
            child: accepted_child,
            terminal: accepted_terminal,
        } = &record.identity
        else {
            return Ok(None);
        };
        if accepted_child != child {
            return Ok(None);
        }
        if inner.pending_child_cancellation.is_some()
            && matches!(
                terminal,
                ChildNavigationResult::Cancelled(_) | ChildNavigationResult::CancelFailed { .. }
            )
        {
            return Ok(None); // Independent, exact physical-ZEROED reconciliation.
        }
        if accepted_terminal != terminal {
            return Err("conflicting duplicate exploration child terminal".into());
        }
        Ok(Some(FeedbackTicket {
            id: record.id,
            identity: record.identity.clone(),
            completion: record.completion.clone(),
        }))
    }
    pub(super) async fn await_feedback(&self, ticket: &FeedbackTicket) -> Result<(), String> {
        loop {
            let notified = self.compute_service.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            {
                let guard = self.inner.lock().await;
                if let Some(error) = &guard.compute.fault {
                    return Err(error.clone());
                }
                if let Some(result) = ticket.completion.lock().clone() {
                    return result;
                }
            }
            notified.await;
        }
    }
    pub(super) async fn await_compute_idle(&self) -> Result<(), String> {
        loop {
            let notified = self.compute_service.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            {
                let guard = self.inner.lock().await;
                if let Some(error) = &guard.compute.fault {
                    return Err(error.clone());
                }
                if guard.compute.job.is_none()
                    && !guard.compute.control_pending()
                    && guard.compute.desired_stop.is_none()
                    && guard
                        .compute
                        .feedback
                        .as_ref()
                        .is_none_or(|record| record.result.is_some())
                    && guard
                        .compute
                        .lifecycle
                        .as_ref()
                        .is_none_or(|record| record.result.is_some())
                {
                    return Ok(());
                }
            }
            notified.await;
        }
    }
    pub(super) fn admit_lifecycle_locked(
        &self,
        inner: &mut ExploreRuntimeInner,
        command: ComputeLifecycle,
    ) -> Result<LifecycleTicket, String> {
        if let Some(error) = &inner.compute.fault {
            return Err(error.clone());
        }
        if inner.compute.job.is_some()
            || inner.compute.desired_stop.is_some()
            || inner
                .compute
                .feedback
                .as_ref()
                .is_some_and(|record| record.result.is_none())
            || inner
                .compute
                .lifecycle
                .as_ref()
                .is_some_and(|record| record.result.is_none())
        {
            return Err("exploration lifecycle is still settling".into());
        }
        let kind = match &command {
            ComputeLifecycle::Install { .. } => LifecycleKind::Install,
            ComputeLifecycle::ExportCheckpoint => LifecycleKind::Export,
            _ => return Err("internal lifecycle command requires retained driver lineage".into()),
        };
        let sequence = inner.compute.next_lifecycle()?;
        let abandoned = Arc::new(AtomicBool::new(false));
        let id = inner
            .compute
            .owner
            .as_ref()
            .ok_or("exploration compute owner missing")?
            .submit_lifecycle(command)
            .map_err(|error| error.to_string())?;
        inner.compute.lifecycle = Some(LifecycleRecord {
            id,
            sequence,
            kind,
            result: None,
            ack: None,
            abandoned: abandoned.clone(),
        });
        self.compute_service.wake.notify_one();
        Ok(LifecycleTicket {
            sequence,
            abandoned,
            wake: self.compute_service.wake.clone(),
        })
    }
    pub(super) async fn await_lifecycle(
        &self,
        ticket: &LifecycleTicket,
    ) -> Result<Arc<ComputeLifecycleOutput>, String> {
        loop {
            let notified = self.compute_service.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            {
                let guard = self.inner.lock().await;
                if let Some(error) = &guard.compute.fault {
                    return Err(error.clone());
                }
                let record = guard
                    .compute
                    .lifecycle
                    .as_ref()
                    .filter(|record| record.sequence == ticket.sequence)
                    .ok_or("exploration lifecycle acknowledgment superseded")?;
                if let Some(result) = &record.result {
                    return result.clone();
                }
            }
            notified.await;
        }
    }
    pub(super) fn open_compute_locked(
        &self,
        inner: &mut ExploreRuntimeInner,
    ) -> Result<(), String> {
        // Consuming the completed lifecycle is safe only while the reservation
        // still owns the exact outer generation. Caller checks it in this lock.
        if inner.compute.control_pending() {
            return Err("exploration control action is still settling".into());
        }
        inner.compute.lifecycle = None;
        inner.compute.feedback = None;
        inner.compute.control_stop = None;
        inner
            .compute
            .owner
            .as_ref()
            .ok_or("exploration compute owner missing")?
            .open_admission()
            .map_err(|error| error.to_string())?;
        inner.logical_active = true;
        inner.compute.start_reservation = None;
        inner.compute.start_abandoned = None;
        Ok(())
    }
    #[cfg(test)]
    pub(crate) async fn inspect_compute_checkpoint(
        &self,
    ) -> Result<Vec<nuwax_exploration::mission::SourceCheckpoint>, String> {
        self.await_compute_idle().await?;
        let mut guard = self.inner.lock().await;
        if !guard.logical_active {
            let ticket =
                self.admit_lifecycle_locked(&mut guard, ComputeLifecycle::ExportCheckpoint)?;
            drop(guard);
            let output = self.await_lifecycle(&ticket).await?;
            return match output.as_ref() {
                ComputeLifecycleOutput::Checkpoint(sources) => Ok(sources.clone()),
                _ => Err("expected checkpoint".into()),
            };
        }
        let now = now_unix_nanos();
        // Inspection never evaluates map/pose input. A source-free fixture may
        // use this inert context; ComputeBasis::inspection validates only session
        // and history, and the backend never forks or queries this grid.
        let context = ComputeContext {
            attribution: None,
            homeward_test_hook: None,
            input: guard
                .costmap_input
                .clone()
                .map(ComputeInput::Planar)
                .unwrap_or_else(|| {
                    ComputeInput::Planar(Arc::new(ExploreCostmapSnapshot {
                        grid: OccupancyGrid2D::new(
                            nuwax_nav_types_messages::geometry::Point2::new(0.0, 0.0),
                            1.0,
                            1,
                            1,
                            vec![OccupancyCell::Unknown],
                        ),
                        frame_id: self.expected_map_frame.to_string(),
                    }))
                }),
            robot_pose: PoseStamped {
                frame_id: NavFrameId::Map,
                stamp: Timestamp {
                    nanos: now as i64,
                    clock: ClockKind::Unix,
                },
                pose: Pose3 {
                    position: Point3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: Quaternion::IDENTITY,
                },
            },
            now: Timestamp {
                nanos: now as i64,
                clock: ClockKind::Unix,
            },
            config: guard
                .active_config
                .clone()
                .ok_or("test configuration missing")?,
            calibration: None,
        };
        let ticket = self.admit_compute_locked(
            &mut guard,
            &context,
            ComputeRequest::InspectCheckpoint {
                context: context.clone(),
            },
        )?;
        drop(guard);
        let output = self.await_compute(&ticket).await?;
        let mut guard = self.inner.lock().await;
        ticket.finish(&mut guard);
        match &output.projection {
            ComputeProjection::Checkpoint(result) => {
                result.clone().map_err(|error| error.to_string())
            }
            _ => Err("expected checkpoint projection".into()),
        }
    }
    #[cfg(test)]
    pub(super) async fn install_compute_controller(
        &self,
        controller: ExplorationController,
    ) -> Result<(), String> {
        let mut guard = self.inner.lock().await;
        self.compute_service.ensure(self, &mut guard.compute)?;
        let generation = guard.generation;
        guard.request_compute_stop(ExplorationStopReason::UserCancelled);
        self.compute_service.wake.notify_one();
        drop(guard);
        self.await_compute_idle().await?;
        let mut guard = self.inner.lock().await;
        if guard.generation != generation {
            return Err("test installation generation changed".into());
        }
        let ticket = self.admit_lifecycle_locked(
            &mut guard,
            ComputeLifecycle::Install {
                controller,
                begin: false,
                restore: None,
            },
        )?;
        drop(guard);
        self.await_lifecycle(&ticket).await?;
        let mut guard = self.inner.lock().await;
        if guard.generation != generation {
            return Err("test installation generation changed".into());
        }
        self.open_compute_locked(&mut guard)
    }
    pub(super) async fn request_compute_shutdown(&self) {
        let ownerless = {
            let mut guard = self.inner.lock().await;
            if !guard.compute.shutdown {
                // Admit the one final Stop before closing the lifecycle gate.
                guard.request_compute_stop(ExplorationStopReason::UserCancelled);
                guard.compute.shutdown = true;
            }
            guard.compute.owner.is_none()
        };
        if ownerless {
            self.compute_service.terminal.send_replace(Some(Ok(())));
        }
        self.compute_service.wake.notify_one();
        self.compute_service.changed.notify_waiters();
    }
}

fn selected_goal(projection: &ComputeProjection) -> Option<&FrontierGoal> {
    match projection {
        ComputeProjection::Selection(SelectionProjection {
            poll: Ok(ExplorationPoll::Goal(goal)),
            ..
        }) => Some(goal),
        _ => None,
    }
}

/// All try_take values first enter retained inner records, before any await. The
/// driver alone consumes the primitive and retains the native owner until joined.
async fn drain_owner(
    context: WeakRuntimeContext,
    inner: Weak<Mutex<ExploreRuntimeInner>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    wake: Arc<Notify>,
    changed: Arc<Notify>,
    terminal: watch::Sender<Option<Result<(), String>>>,
    mut owner: exchange::ComputeOwner<ExploreComputeBackend>,
) {
    let (native_done, native_done_rx) = watch::channel(false);
    let native = async {
        let drained = AssertUnwindSafe(drain_owner_loop(
            &context,
            inner.clone(),
            frontier_tx,
            snapshot_cache.clone(),
            wake.clone(),
            changed.clone(),
            &mut owner,
        ))
        .catch_unwind()
        .await;
        let result = match drained {
            Ok(result) => result,
            Err(_) => {
                let error =
                    "exploration compute driver panicked; owner is permanently faulted".to_string();
                if let Some(inner) = inner.upgrade() {
                    let mut guard = inner.lock().await;
                    guard.compute.fault.get_or_insert(error.clone());
                    if !guard.compute.fault_action_scheduled {
                        guard.compute.fault_action_scheduled = true;
                        if let Err(control_error) = context.retain_stop(
                            &mut guard,
                            ExplorationStopReason::ConsecutiveFailuresExceeded,
                            error.clone(),
                        ) {
                            guard.compute.fault = Some(control_error);
                            guard.request_compute_stop(
                                ExplorationStopReason::ConsecutiveFailuresExceeded,
                            );
                        }
                    }
                    *snapshot_cache.write() = guard.snapshot(STATUS_FAILED, error.clone());
                }
                let handle = owner.handle();
                handle.shutdown();
                changed.notify_waiters();
                loop {
                    let status = handle.status();
                    if status.closed && owner.try_join() == exchange::JoinStatus::Joined {
                        break;
                    }
                    if status.closed {
                        tokio::task::yield_now().await;
                    } else {
                        handle.changed(status.epoch).await;
                    }
                }
                Err(error)
            }
        };
        native_done.send_replace(true);
        changed.notify_waiters();
        result
    };
    // Both coroutines belong to this one retained task. A held native stage does
    // not prevent exact child cancellation, and join cannot outrun that action.
    let control = run_control_actions(&context, wake.clone(), changed.clone(), native_done_rx);
    let (native_result, control_result) = tokio::join!(native, control);
    terminal.send_replace(Some(native_result.and(control_result)));
    changed.notify_waiters();
}

async fn run_control_actions(
    context: &WeakRuntimeContext,
    wake: Arc<Notify>,
    changed: Arc<Notify>,
    mut native_done: watch::Receiver<bool>,
) -> Result<(), String> {
    let mut terminal_error = None;
    loop {
        let notified = changed.notified();
        tokio::pin!(notified);
        notified.as_mut().enable();
        let Some(inner) = context.inner.upgrade() else {
            return terminal_error.map_or(Ok(()), Err);
        };
        let (action, native_finished) = {
            let mut guard = inner.lock().await;
            let action = guard.compute.control_stop.as_mut().and_then(|record| {
                if record.running.is_none() && record.settled != Some(record.desired.serial) {
                    record.running = Some(record.desired.serial);
                    Some(record.desired.clone())
                } else {
                    None
                }
            });
            (action, *native_done.borrow())
        };
        drop(inner);
        if let Some(action) = action {
            changed.notify_waiters();
            let execution = if let Some(runtime) = context.upgrade() {
                let result = AssertUnwindSafe(runtime.finish_compute_action(&action))
                    .catch_unwind()
                    .await;
                if result.is_err() {
                    let error = "exploration retained control action panicked".to_string();
                    if let Some(lease) = &action.child_lease {
                        {
                            let mut guard = runtime.inner.lock().await;
                            if guard
                                .active_child
                                .as_ref()
                                .is_some_and(|child| child.lease.matches(lease))
                                && guard.pending_child_cancellation.is_none()
                            {
                                guard.pending_child_cancellation = Some(PendingChildCancellation {
                                    action: PendingChildAction {
                                        reason: ExplorationStopReason::ConsecutiveFailuresExceeded,
                                        message: error.clone(),
                                    },
                                    ticket: None,
                                    failure: Some(error.clone()),
                                    dispatch_in_flight: false,
                                    early_terminals: Vec::new(),
                                });
                            }
                        }
                        // Preserve the original exact lease and failure barrier;
                        // never retry a partially completed cancellation here.
                        let _ = runtime
                            .record_child_cancel_failure(lease, error.clone(), None)
                            .await;
                    }
                    let mut guard = runtime.inner.lock().await;
                    guard.compute.fault.get_or_insert(error.clone());
                    guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
                    Err(error)
                } else {
                    Ok(())
                }
            } else {
                Err("exploration runtime disappeared before retained control action".into())
            };
            if let Err(error) = execution {
                terminal_error.get_or_insert(error);
            }
            if let Some(inner) = context.inner.upgrade() {
                let mut guard = inner.lock().await;
                if let Some(record) = guard
                    .compute
                    .control_stop
                    .as_mut()
                    .filter(|record| record.running == Some(action.serial))
                {
                    record.running = None;
                    record.settled = Some(action.serial);
                }
            }
            changed.notify_waiters();
            wake.notify_one();
            continue;
        }
        if native_finished {
            return terminal_error.map_or(Ok(()), Err);
        }
        tokio::select! {
            _ = &mut notified => {},
            result = native_done.changed() => if result.is_err() { return Err("exploration native settlement signal closed".into()); },
        }
    }
}

async fn drain_owner_loop(
    context: &WeakRuntimeContext,
    inner: Weak<Mutex<ExploreRuntimeInner>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    wake: Arc<Notify>,
    changed: Arc<Notify>,
    owner: &mut exchange::ComputeOwner<ExploreComputeBackend>,
) -> Result<(), String> {
    let handle = owner.handle();
    let mut terminal_error = None;
    loop {
        let notified = wake.notified();
        tokio::pin!(notified);
        notified.as_mut().enable();
        let epoch = handle.status().epoch;
        let mut retired_job = None;
        let mut should_shutdown = false;
        if let Some(inner) = inner.upgrade() {
            let mut guard = inner.lock().await;
            #[cfg(test)]
            if std::mem::take(&mut guard.compute.panic_driver) {
                panic!("injected exploration driver panic");
            }
            let history_before = guard.compute.history;
            let was_active = guard.logical_active;
            if let Err(error) = drain_locked(&mut guard, &handle, &mut retired_job, context) {
                guard.compute.fault.get_or_insert(error.clone());
                if !guard.compute.fault_action_scheduled {
                    guard.compute.fault_action_scheduled = true;
                    if let Err(control_error) = context.retain_stop(
                        &mut guard,
                        ExplorationStopReason::ConsecutiveFailuresExceeded,
                        error.clone(),
                    ) {
                        guard.compute.fault = Some(control_error);
                        guard.request_compute_stop(
                            ExplorationStopReason::ConsecutiveFailuresExceeded,
                        );
                    }
                }
                handle.stop_admission();
                terminal_error.get_or_insert(error);
                should_shutdown = true;
            }
            if let Some(error) = guard.compute.fault.clone() {
                terminal_error.get_or_insert(error);
                should_shutdown = true;
            }
            if guard.compute.shutdown
                && guard.compute.desired_stop.is_none()
                && guard.compute.job.is_none()
                && guard
                    .compute
                    .feedback
                    .as_ref()
                    .is_none_or(|record| record.result.is_some())
                && guard
                    .compute
                    .lifecycle
                    .as_ref()
                    .is_none_or(|record| record.result.is_some())
            {
                should_shutdown = true;
            }
            let frontier_update = if guard.logical_active && guard.compute.history != history_before
            {
                let frontiers = guard.committed.frontiers.clone();
                guard.update_frontiers(frontiers)
            } else {
                None
            };
            let snapshot = if guard.logical_active && guard.compute.history != history_before {
                guard
                    .compute
                    .feedback
                    .as_ref()
                    .filter(|record| record.result.is_some())
                    .and_then(|record| record.message)
                    .map(|message| guard.snapshot(STATUS_ACTIVE, message))
            } else if was_active && !guard.logical_active {
                let status = if guard.active_child.is_some() {
                    if guard
                        .pending_child_cancellation
                        .as_ref()
                        .is_some_and(|pending| {
                            pending.failure.is_some() && !pending.dispatch_in_flight
                        })
                    {
                        STATUS_FAILED
                    } else {
                        STATUS_STOPPING
                    }
                } else {
                    guard
                        .last_stop
                        .map(status_for_stop_reason)
                        .unwrap_or(STATUS_FAILED)
                };
                let message = guard
                    .compute
                    .fault
                    .clone()
                    .or_else(|| {
                        guard
                            .compute
                            .control_stop
                            .as_ref()
                            .map(|record| record.desired.message.clone())
                    })
                    .unwrap_or_else(|| "exploration compute owner stopped".into());
                Some(guard.snapshot(status, message))
            } else {
                None
            };
            // Publish these committed projections in the same state order as
            // stop. A delayed driver must never overwrite a later stop snapshot.
            if let Some(frontiers) = frontier_update {
                frontier_tx.send_replace(frontiers);
            }
            if let Some(snapshot) = snapshot {
                *snapshot_cache.write() = snapshot;
            }
            drop(guard);
        } else {
            should_shutdown = true;
        }
        // Large projection/input retirement runs outside inner. A final Arc/grid
        // destructor still executes on this Tokio task; this residual is measured
        // separately and is not claimed to be off-executor CPU work.
        drop(retired_job);
        changed.notify_waiters();
        if should_shutdown {
            handle.shutdown();
        }
        let status = handle.status();
        if status.closed {
            terminal_error = terminal_error.or_else(|| {
                status
                    .fault
                    .map(|fault| format!("exploration compute fault: {fault:?}"))
            });
            if owner.try_join() == exchange::JoinStatus::Joined {
                break;
            }
            // closed is published immediately before the native function returns;
            // no timer or blocking join is required while backend work is held.
            tokio::task::yield_now().await;
            continue;
        }
        tokio::select! { _ = handle.changed(epoch) => {}, _ = &mut notified => {} }
    }
    terminal_error.map_or(Ok(()), Err)
}

fn drain_locked(
    inner: &mut ExploreRuntimeInner,
    owner: &Owner,
    retired: &mut Option<JobRecord>,
    context: &WeakRuntimeContext,
) -> Result<(), String> {
    if inner
        .compute
        .start_abandoned
        .as_ref()
        .is_some_and(|abandoned| abandoned.load(Ordering::Acquire))
    {
        inner.request_compute_stop(ExplorationStopReason::UserCancelled);
    }
    if let Some(fault) = owner.status().fault {
        return Err(format!("exploration compute fault: {fault:?}"));
    }
    if let Some(prepared) = owner
        .try_take_prepared()
        .map_err(|error| error.to_string())?
    {
        let job = inner
            .compute
            .job
            .as_mut()
            .filter(|job| job.id == prepared.id)
            .ok_or("unowned exploration prepared projection")?;
        job.prepared = Some(prepared);
    }
    if inner
        .compute
        .job
        .as_ref()
        .is_some_and(|job| job.prepared.is_some() && job.decision.is_none())
    {
        let job = inner.compute.job.as_ref().expect("job checked");
        let cancelled = matches!(
            job.prepared.as_ref().map(|value| &value.projection),
            Some(ComputeProjection::Cancelled)
                | Some(ComputeProjection::Selection(SelectionProjection {
                    cancelled: true,
                    ..
                }))
        );
        let decision =
            if !job.abandoned.load(Ordering::Acquire) && !cancelled && job.basis.current(inner) {
                exchange::Decision::Accept
            } else {
                exchange::Decision::Discard
            };
        let ack = owner
            .resolve(job.id, decision)
            .map_err(|error| error.to_string())?;
        inner
            .compute
            .job
            .as_mut()
            .expect("same critical section")
            .decision = Some(ack);
    }
    if let Some(settled) = owner
        .try_take_settled()
        .map_err(|error| error.to_string())?
    {
        let job = inner
            .compute
            .job
            .as_mut()
            .filter(|job| job.id == settled.acknowledgement.id)
            .ok_or("unowned exploration settlement")?;
        job.settled = Some(settled);
    }
    if let Some(job) = inner.compute.job.as_mut() {
        if let Some(settled) = job.settled.take() {
            job.decision = Some(settled.acknowledgement);
            job.prepared = None;
            job.result = Some(match settled.result {
                exchange::Settlement::Accepted(value) => {
                    inner.committed = value.committed.clone();
                    Ok(Arc::new(value))
                }
                exchange::Settlement::Discarded => {
                    Err("exploration compute input superseded or stopped".into())
                }
            });
        }
    }
    let accepted_output = inner.compute.job.as_mut().and_then(|job| {
        if job.output_bookkept {
            return None;
        }
        let output = job.result.as_ref()?.as_ref().ok()?.clone();
        job.output_bookkept = true;
        Some((job.id, job.basis.clone(), output))
    });
    if let Some((job_id, basis, output)) = accepted_output {
        let previous_control = inner
            .compute
            .control_stop
            .as_ref()
            .map(|record| record.desired.serial);
        // Accept is immutable. A later soft source refresh does not undo source
        // end/history; only a newer outer session/lifecycle supersedes this work.
        if inner.logical_active
            && inner.generation == basis.generation
            && inner.last_request_id == basis.parent
        {
            if let ComputeProjection::Selection(selection) = &output.projection {
                if let Some(Err(error)) = &selection.mission {
                    let reason = if error.is_inconclusive() {
                        ExplorationStopReason::PlanningResourceLimited
                    } else {
                        ExplorationStopReason::ConsecutiveFailuresExceeded
                    };
                    context.retain_stop(
                        inner,
                        reason,
                        format!("返航路线核验失败：{error}"),
                    )?;
                } else {
                    match &selection.poll {
                        Ok(ExplorationPoll::Stop(reason)) => {
                            inner.last_poll = Some(ExplorationPoll::Stop(*reason));
                            if basis.return_on_completion
                                && basis.mission_present
                                && reason.terminal_kind() == ExplorationTerminalKind::Succeeded
                            {
                                inner.return_reason = Some(*reason);
                                if let Some(progress) = inner.mission.as_mut() {
                                    progress.returning = true;
                                }
                            } else {
                                context.retain_stop(
                                    inner,
                                    *reason,
                                    exploration_stop_message(*reason),
                                )?;
                            }
                        }
                        Err(error) => {
                            context.retain_stop(
                                inner,
                                ExplorationStopReason::ConsecutiveFailuresExceeded,
                                format!("exploration controller error: {error}"),
                            )?;
                        }
                        Ok(ExplorationPoll::NoFrontier) => {
                            inner.last_poll = Some(ExplorationPoll::NoFrontier);
                            let first = *inner.no_frontier_since.get_or_insert_with(Instant::now);
                            if first.elapsed() >= basis.no_frontier_timeout {
                                if basis.return_on_completion && basis.mission_present {
                                    inner.return_reason =
                                        Some(ExplorationStopReason::NoFrontierTimeout);
                                    if let Some(progress) = inner.mission.as_mut() {
                                        progress.returning = true;
                                    }
                                } else {
                                    context.retain_stop(
                                        inner,
                                        ExplorationStopReason::NoFrontierTimeout,
                                        "no frontier timeout reached".into(),
                                    )?;
                                }
                            }
                        }
                        Ok(ExplorationPoll::Pending) => {
                            inner.last_poll = Some(ExplorationPoll::Pending);
                            inner.no_frontier_since = None;
                        }
                        Ok(ExplorationPoll::SourceUnavailable(reason)) => {
                            inner.last_poll =
                                Some(ExplorationPoll::SourceUnavailable(reason.clone()));
                            inner.no_frontier_since = None;
                        }
                        Ok(ExplorationPoll::Goal(_)) => {}
                    }
                }
            }
        }
        let current_control = inner
            .compute
            .control_stop
            .as_ref()
            .map(|record| record.desired.serial);
        if current_control != previous_control {
            if let Some(job) = inner.compute.job.as_mut().filter(|job| job.id == job_id) {
                job.terminal_action = current_control;
            }
        }
    }
    let operation_id = inner
        .compute
        .lifecycle
        .as_ref()
        .filter(|record| record.result.is_none())
        .map(|record| record.id)
        .or_else(|| {
            inner
                .compute
                .feedback
                .as_ref()
                .filter(|record| record.result.is_none())
                .map(|record| record.id)
        });
    #[cfg(test)]
    let operation_id = operation_id.filter(|_| {
        !inner.compute.hold_install_ack
            || inner
                .compute
                .lifecycle
                .as_ref()
                .is_none_or(|record| record.kind != LifecycleKind::Install)
    });
    let operation_ack = operation_id
        .map(|id| owner.try_take_operation_ack(id))
        .transpose()
        .map_err(|error| error.to_string())?
        .flatten();
    if let Some(ack) = operation_ack {
        match ack {
            exchange::OperationAck::Feedback { id, result } => {
                let record = inner
                    .compute
                    .feedback
                    .as_mut()
                    .filter(|record| record.id == id)
                    .ok_or("unowned exploration feedback acknowledgment")?;
                record.ack = Some(result);
            }
            exchange::OperationAck::Lifecycle { id, result } => {
                let record = inner
                    .compute
                    .lifecycle
                    .as_mut()
                    .filter(|record| record.id == id)
                    .ok_or("unowned exploration lifecycle acknowledgment")?;
                record.ack = Some(result);
            }
        }
    }
    let feedback_history_after = if inner
        .compute
        .feedback
        .as_ref()
        .is_some_and(|record| record.ack.is_some())
    {
        Some(
            inner
                .compute
                .history
                .checked_add(1)
                .ok_or("exploration history revision exhausted")?,
        )
    } else {
        None
    };
    if let Some(record) = &mut inner.compute.feedback {
        if let Some(ack) = record.ack.take() {
            inner.committed = ack.committed;
            let result = ack.result.map_err(|error| error.to_string());
            if let FeedbackIdentity::External { child, .. } = &record.identity {
                if result.is_ok() && record.reached {
                    inner.reached_goals = inner.reached_goals.saturating_add(1);
                }
                if inner
                    .active_child
                    .as_ref()
                    .is_some_and(|active| &active.request_id == child)
                {
                    if inner.pending_child_cancellation.is_none() {
                        inner.active_child = None;
                    }
                    inner.no_frontier_since = None;
                }
            }
            *record.completion.lock() = Some(result.clone());
            record.result = Some(result);
            record.history_after = feedback_history_after;
            inner.compute.history =
                feedback_history_after.expect("accepted feedback history allocated above");
        }
    }
    let mut lifecycle_completion = None;
    if let Some(record) = &mut inner.compute.lifecycle {
        if let Some(ack) = record.ack.take() {
            inner.committed = ack.committed;
            record.result = Some(ack.result.map(Arc::new).map_err(|error| error.to_string()));
            lifecycle_completion = Some((
                record.kind,
                record.sequence,
                record.result.as_ref().expect("set above").is_err(),
                record.abandoned.load(Ordering::Acquire),
            ));
        }
    }
    if let Some((kind, sequence, failed, abandoned)) = lifecycle_completion {
        match kind {
            LifecycleKind::Stop => {
                inner.compute.stop_settled = sequence;
                if inner
                    .compute
                    .desired_stop
                    .is_some_and(|(desired, _)| desired == sequence)
                {
                    inner.compute.desired_stop = None;
                }
                inner.compute.lifecycle = None;
            }
            LifecycleKind::Retire => {
                if let Some(job) = inner.compute.job.as_mut() {
                    job.goal_handled = true;
                    job.released = true;
                }
                // The retirement Err is terminal, retained as a permanent runtime
                // fault rather than retried against partially mutated source state.
                if failed {
                    return Err("accepted exploration goal retirement failed".into());
                }
                inner.compute.bump_history()?;
                inner.compute.lifecycle = None;
            }
            LifecycleKind::Install if abandoned => {
                inner.compute.lifecycle = None;
                inner.request_compute_stop(ExplorationStopReason::UserCancelled);
            }
            LifecycleKind::Export if abandoned => {
                inner.compute.lifecycle = None;
            }
            _ => {}
        }
        if kind == LifecycleKind::Stop && failed {
            return Err("exploration source stop failed".into());
        }
    }
    // Feedback Err is a terminal acknowledgment, never a retry. Any retained
    // child is cancelled by the same exact safety protocol as an explicit stop.
    if inner.logical_active {
        if let Some(error) = inner
            .compute
            .feedback
            .as_ref()
            .and_then(|record| record.result.as_ref())
            .and_then(|result| result.as_ref().err())
            .cloned()
        {
            context.retain_stop(
                inner,
                ExplorationStopReason::ConsecutiveFailuresExceeded,
                format!("exploration goal feedback failed: {error}"),
            )?;
        }
    }
    let obsolete_goal = inner.compute.job.as_ref().and_then(|job| {
        if job.goal_handled
            || job.retirement.is_some()
            || !(job.abandoned.load(Ordering::Acquire) || !job.basis.current(inner))
        {
            return None;
        }
        let value = job.result.as_ref()?.as_ref().ok()?;
        let goal = selected_goal(&value.projection)?;
        // Dispatch handoff is itself retained before its async sink call. A
        // cancelled tick must not retire a goal already owned by that handoff.
        if inner
            .pending_child_dispatch
            .as_ref()
            .is_some_and(|request| request.frontier_goal_id == goal.id)
            || inner
                .active_child
                .as_ref()
                .is_some_and(|child| child.frontier_goal_id == goal.id)
        {
            return None;
        }
        Some((job.id, goal.id.clone()))
    });
    if let Some((transaction, goal_id)) = obsolete_goal {
        if inner
            .compute
            .lifecycle
            .as_ref()
            .is_none_or(|record| record.result.is_some())
            && inner
                .compute
                .feedback
                .as_ref()
                .is_none_or(|record| record.result.is_some())
        {
            // Closed admission is intentional. This command cannot admit arbitrary
            // external feedback: the exact accepted transaction and finalized goal
            // above are the sole authority for this Cancelled operation.
            owner.stop_admission();
            let sequence = inner.compute.next_lifecycle()?;
            match owner.submit_lifecycle(ComputeLifecycle::Retire {
                goal_id,
                feedback: ExplorationGoalFeedback::Cancelled,
            }) {
                Ok(id) => {
                    inner
                        .compute
                        .job
                        .as_mut()
                        .filter(|job| job.id == transaction)
                        .expect("lineage retained")
                        .retirement = Some(id);
                    inner.compute.lifecycle = Some(LifecycleRecord {
                        id,
                        sequence,
                        kind: LifecycleKind::Retire,
                        result: None,
                        ack: None,
                        abandoned: Arc::new(AtomicBool::new(false)),
                    });
                }
                Err(exchange::OwnerError::Busy) => {}
                Err(error) => return Err(error.to_string()),
            }
        }
    }
    if let Some(job) = inner.compute.job.as_mut() {
        if let Some(goal) = job
            .result
            .as_ref()
            .and_then(|value| value.as_ref().ok())
            .and_then(|value| selected_goal(&value.projection))
        {
            if inner
                .pending_child_dispatch
                .as_ref()
                .is_some_and(|request| request.frontier_goal_id == goal.id)
                || inner
                    .active_child
                    .as_ref()
                    .is_some_and(|child| child.frontier_goal_id == goal.id)
            {
                job.goal_handled = true;
            }
        }
    }
    if inner
        .compute
        .lifecycle
        .as_ref()
        .is_some_and(|record| record.result.is_some() && record.abandoned.load(Ordering::Acquire))
    {
        let kind = inner.compute.lifecycle.as_ref().expect("checked").kind;
        if kind == LifecycleKind::Install {
            inner.compute.lifecycle = None;
            inner.request_compute_stop(ExplorationStopReason::UserCancelled);
        } else if kind == LifecycleKind::Export {
            inner.compute.lifecycle = None;
        }
    }
    let job_done = inner.compute.job.as_ref().is_some_and(|job| {
        job.result.is_some()
            && (job.released || job.abandoned.load(Ordering::Acquire))
            && (job.goal_handled
                || job.result.as_ref().is_some_and(|result| match result {
                    Err(_) => true,
                    Ok(value) => selected_goal(&value.projection).is_none(),
                }))
    });
    if job_done {
        *retired = inner.compute.job.take();
    }
    if inner.compute.job.is_none()
        && inner
            .compute
            .feedback
            .as_ref()
            .is_none_or(|record| record.result.is_some())
        && inner
            .compute
            .lifecycle
            .as_ref()
            .is_none_or(|record| record.result.is_some())
    {
        if let Some((sequence, reason)) = inner.compute.desired_stop {
            match owner.submit_lifecycle(ComputeLifecycle::Stop(reason)) {
                Ok(id) => {
                    inner.compute.lifecycle = Some(LifecycleRecord {
                        id,
                        sequence,
                        kind: LifecycleKind::Stop,
                        result: None,
                        ack: None,
                        abandoned: Arc::new(AtomicBool::new(false)),
                    })
                }
                Err(exchange::OwnerError::Busy) => {}
                Err(error) => return Err(error.to_string()),
            }
        } else if inner.logical_active && !owner.status().admitting {
            owner.open_admission().map_err(|error| error.to_string())?;
        }
    }
    Ok(())
}
