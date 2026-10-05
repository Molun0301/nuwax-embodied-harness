// Included inside explore_runtime::tests so existing exact child/lease fixtures
// and assertions are reused. Gates establish ordering; timeouts detect hangs.
#[derive(Default)]
struct OwnerRuntimeGate {
    entered: std::sync::atomic::AtomicBool,
    state: StdMutex<bool>,
    release: std::sync::Condvar,
    changed: tokio::sync::Notify,
}
impl OwnerRuntimeGate {
    fn hold(&self) {
        self.entered.store(true, Ordering::Release);
        self.changed.notify_waiters();
        let mut released = self.state.lock().unwrap();
        while !*released {
            released = self.release.wait(released).unwrap();
        }
    }
    async fn entered(&self) {
        loop {
            let notified = self.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            if self.entered.load(Ordering::Acquire) {
                return;
            }
            notified.await;
        }
    }
    fn release(&self) {
        *self.state.lock().unwrap() = true;
        self.release.notify_all();
    }
}
// Unwind must release a held native source before Tokio tears down the driver:
// the owner's last-resort join intentionally cannot detach that worker.
struct OwnerRuntimeGateRelease(Arc<OwnerRuntimeGate>);
impl Drop for OwnerRuntimeGateRelease {
    fn drop(&mut self) {
        self.0.release();
    }
}

struct OwnerRuntimeAsyncRelease(Arc<tokio::sync::Notify>);
impl Drop for OwnerRuntimeAsyncRelease {
    fn drop(&mut self) {
        self.0.notify_one();
    }
}
struct OwnerRuntimeSource {
    prepare_gate: Option<Arc<OwnerRuntimeGate>>,
    commit_gate: Option<Arc<OwnerRuntimeGate>>,
    feedback_gate: Option<Arc<OwnerRuntimeGate>>,
    feedback: Arc<StdMutex<Vec<ExplorationGoalFeedback>>>,
    fail_feedback: bool,
    active: bool,
    pending: Option<ExplorationPoll>,
    selected: Option<String>,
}
impl OwnerRuntimeSource {
    fn new(feedback: Arc<StdMutex<Vec<ExplorationGoalFeedback>>>) -> Self {
        Self {
            prepare_gate: None,
            commit_gate: None,
            feedback_gate: None,
            feedback,
            fail_feedback: false,
            active: false,
            pending: None,
            selected: None,
        }
    }
}
impl ExplorationGoalSource for OwnerRuntimeSource {
    fn source_name(&self) -> std::borrow::Cow<'static, str> {
        "wavefront".into()
    }
    fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.active = true;
        Ok(())
    }
    fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.active = false;
        self.selected = None;
        Ok(())
    }
    fn supports_prepared_poll(&self) -> bool {
        true
    }
    fn poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
        self.prepare_poll(state)?;
        self.commit_prepared_poll()
    }
    fn prepare_poll(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> nuwax_exploration::ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
        if let Some(gate) = &self.prepare_gate {
            gate.hold();
        }
        if self.selected.is_some() {
            self.pending = Some(ExplorationPoll::Pending);
            return Ok(nuwax_exploration::PreparedExplorationPoll::new(Ok(
                ExplorationPoll::Pending,
            )));
        }
        let mut pose = state.robot_pose.clone();
        pose.pose.position.x = 2.5;
        let poll = ExplorationPoll::Goal(FrontierGoal::new("owner-goal", pose, 1.0));
        self.pending = Some(poll.clone());
        Ok(nuwax_exploration::PreparedExplorationPoll::new(Ok(poll)))
    }
    fn commit_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
        if let Some(gate) = &self.commit_gate {
            gate.hold();
        }
        let poll = self
            .pending
            .take()
            .ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)?;
        if let ExplorationPoll::Goal(goal) = &poll {
            self.selected = Some(goal.id.clone());
        }
        Ok(poll)
    }
    fn discard_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.pending = None;
        Ok(())
    }
    fn report_goal_feedback(
        &mut self,
        _: &str,
        feedback: ExplorationGoalFeedback,
    ) -> nuwax_exploration::ExploreResult<()> {
        if let Some(gate) = &self.feedback_gate {
            gate.hold();
        }
        self.feedback.lock().unwrap().push(feedback);
        self.selected = None;
        if self.fail_feedback {
            Err(nuwax_exploration::ExploreError::SourceError(
                "held terminal error".into(),
            ))
        } else {
            Ok(())
        }
    }
    fn checkpoint(&self) -> nuwax_exploration::ExploreResult<String> {
        Ok(format!(
            "feedback_count={}",
            self.feedback.lock().unwrap().len()
        ))
    }
    fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
        nuwax_exploration::ExplorationSourceDiagnostics {
            source_name: "wavefront".into(),
            is_active: self.active,
            last_goal_id: self.selected.clone(),
            last_error: None,
        }
    }
}
async fn install_owner_runtime_source(handle: &ExploreRuntimeHandle, source: OwnerRuntimeSource) {
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(source);
    controller.begin_exploration().unwrap();
    handle.install_compute_controller(controller).await.unwrap();
}
async fn owner_watchdog<T>(future: impl std::future::Future<Output = T>) -> T {
    tokio::time::timeout(Duration::from_secs(5), future)
        .await
        .expect("held-owner protocol hung")
}

#[tokio::test]
async fn compute_owner_held_prepare_stop_is_cheap_and_reuses_worker() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.start("owner-held-prepare", "prefer_wavefront").await;
    let feedback = Arc::new(StdMutex::new(Vec::new()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeSource::new(feedback.clone());
    source.prepare_gate = Some(gate.clone());
    install_owner_runtime_source(&handle, source).await;
    let before = handle.inner.lock().await.compute.status().unwrap();
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    let stopped = owner_watchdog(handle.stop("owner-held-prepare")).await;
    assert_eq!(stopped.status, STATUS_CANCELLED);
    assert!(!handle.inner.lock().await.logical_active);
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(feedback.lock().unwrap().is_empty());
    assert!(
        handle.inner.lock().await.compute.status().unwrap().phase
            != crate::explore_compute_owner::Phase::Idle
    );
    gate.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let started = owner_watchdog(handle.start("owner-reused", "prefer_wavefront")).await;
    assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
    let after = handle.inner.lock().await.compute.status().unwrap();
    assert_eq!(before.incarnation, after.incarnation);
    assert_eq!(after.high_water.workers, 1);
    assert_eq!(after.high_water.active, 1);
    assert!(sink.submitted.lock().unwrap().is_empty());
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_cancelled_tick_retires_accepted_undispatched_goal_once() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.start("owner-abandoned", "prefer_wavefront").await;
    let feedback = Arc::new(StdMutex::new(Vec::new()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeSource::new(feedback.clone());
    source.commit_gate = Some(gate.clone());
    install_owner_runtime_source(&handle, source).await;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await; // Immutable Accept already linearized.
    tick.abort();
    assert!(owner_watchdog(tick).await.unwrap_err().is_cancelled());
    gate.release();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert_eq!(
        *feedback.lock().unwrap(),
        vec![ExplorationGoalFeedback::Cancelled]
    );
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(handle.inner.lock().await.logical_active);
    // Controller diagnostics preserve the last poll's cached source projection.
    // Prove the real selection latch is clear instead: this source returns Pending
    // while selected, so a fresh handoff can happen only after the Cancelled ack.
    let fresh = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(
        *feedback.lock().unwrap(),
        vec![ExplorationGoalFeedback::Cancelled]
    );
    stop_owner_runtime_child(&handle, "owner-abandoned").await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_accepted_feedback_survives_stop_and_zeroed_with_duplicate_join() {
    let (handle, _) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle
        .start("owner-feedback-stop", "prefer_wavefront")
        .await;
    let feedback = Arc::new(StdMutex::new(Vec::new()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeSource::new(feedback.clone());
    source.feedback_gate = Some(gate.clone());
    install_owner_runtime_source(&handle, source).await;
    let active = handle.tick((2.0, 2.0)).await;
    let lease = active_child_lease(&handle).await;
    let terminal = ChildNavigationResult::Reached(terminal_proof(&lease));
    let first = tokio::spawn({
        let handle = handle.clone();
        let id = active.child_task_id.clone();
        let terminal = terminal.clone();
        async move { handle.report_navigation_result(&id, terminal).await }
    });
    owner_watchdog(gate.entered()).await;
    let duplicate = tokio::spawn({
        let handle = handle.clone();
        let id = active.child_task_id.clone();
        async move { handle.report_navigation_result(&id, terminal).await }
    });
    assert_eq!(
        owner_watchdog(handle.stop("owner-feedback-stop"))
            .await
            .status,
        STATUS_STOPPING
    );
    owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
    ))
    .await
    .unwrap();
    assert!(handle.inner.lock().await.active_child.is_none());
    gate.release();
    owner_watchdog(first).await.unwrap().unwrap();
    owner_watchdog(duplicate).await.unwrap().unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert_eq!(
        *feedback.lock().unwrap(),
        vec![ExplorationGoalFeedback::Reached]
    );
    assert_eq!(handle.inner.lock().await.reached_goals, 1);
    assert!(!handle.inner.lock().await.logical_active);
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        "feedback_count=1"
    );
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_feedback_error_is_terminal_and_never_replayed() {
    let (handle, _) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle
        .start("owner-feedback-error", "prefer_wavefront")
        .await;
    let feedback = Arc::new(StdMutex::new(Vec::new()));
    let mut source = OwnerRuntimeSource::new(feedback.clone());
    source.fail_feedback = true;
    install_owner_runtime_source(&handle, source).await;
    let active = handle.tick((2.0, 2.0)).await;
    let lease = active_child_lease(&handle).await;
    let terminal = ChildNavigationResult::Reached(terminal_proof(&lease));
    let first = handle
        .report_navigation_result(&active.child_task_id, terminal.clone())
        .await
        .unwrap_err();
    let duplicate = handle
        .report_navigation_result(&active.child_task_id, terminal)
        .await
        .unwrap_err();
    assert_eq!(first, duplicate);
    assert!(first.contains("held terminal error"));
    assert_eq!(
        *feedback.lock().unwrap(),
        vec![ExplorationGoalFeedback::Reached]
    );
    assert_eq!(handle.inner.lock().await.reached_goals, 0);
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_driver_panic_retains_held_native_worker_across_shutdown_timeout() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.start("owner-driver-panic", "prefer_wavefront").await;
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeSource::new(Arc::new(StdMutex::new(Vec::new())));
    source.prepare_gate = Some(gate.clone());
    install_owner_runtime_source(&handle, source).await;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    {
        let mut guard = handle.inner.lock().await;
        guard.compute.panic_driver = true;
    }
    handle.compute_service.wake.notify_one();
    let scheduler =
        handle.spawn_scheduler(Arc::new(MissingPoseProvider), Duration::from_millis(10));
    let pending = owner_watchdog(scheduler.shutdown_with_settlement()).await;
    assert!(!pending.producers_settled());
    assert!(owner_watchdog(handle.diagnostics()).await.active == false);
    assert!(sink.submitted.lock().unwrap().is_empty());
    gate.release();
    let _ = owner_watchdog(tick).await;
    let terminal = owner_watchdog(scheduler.shutdown_with_settlement()).await;
    assert!(terminal.producers_settled());
    assert!(terminal
        .into_result()
        .unwrap_err()
        .contains("driver panicked"));
    assert!(handle
        .checkpoint_progress("owner-driver-panic")
        .await
        .is_err());
    assert_eq!(
        handle
            .start("owner-must-not-reuse", "prefer_wavefront")
            .await
            .status,
        STATUS_REJECTED
    );
}

// A transactional scripted source used only to make semantic mutation, goal-ID
// allocation, and source end observable at the actual runtime boundary. Its
// checkpoint uses the real wavefront history format so restart exercises the
// production restore path rather than a test-only replacement of runtime state.
#[derive(Clone, Debug, Default)]
struct OwnerRuntimeAudit {
    begins: usize,
    ends: usize,
    prepares: usize,
    commits: usize,
    discards: usize,
    allocated_ids: Vec<String>,
    feedback: Vec<(String, ExplorationGoalFeedback)>,
}

struct OwnerRuntimeScript {
    audit: Arc<StdMutex<OwnerRuntimeAudit>>,
    prepare_gate: Option<Arc<OwnerRuntimeGate>>,
    commit_gate: Option<Arc<OwnerRuntimeGate>>,
    feedback_gate: Option<Arc<OwnerRuntimeGate>>,
    stop_reason: Option<ExplorationStopReason>,
    poll_no_frontier: bool,
    candidate_x: f64,
    active: bool,
    pending: Option<ExplorationPoll>,
    selected: Option<FrontierGoal>,
    history: nuwax_explore_wavefront::session::ExplorationSession,
}

impl OwnerRuntimeScript {
    fn new(audit: Arc<StdMutex<OwnerRuntimeAudit>>) -> Self {
        Self {
            audit,
            prepare_gate: None,
            commit_gate: None,
            feedback_gate: None,
            stop_reason: None,
            poll_no_frontier: false,
            candidate_x: 2.5,
            active: false,
            pending: None,
            selected: None,
            history: nuwax_explore_wavefront::session::ExplorationSession::new(),
        }
    }
}

impl ExplorationGoalSource for OwnerRuntimeScript {
    fn source_name(&self) -> std::borrow::Cow<'static, str> {
        "wavefront".into()
    }

    fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.audit.lock().unwrap().begins += 1;
        self.active = true;
        Ok(())
    }

    fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
        assert!(
            self.pending.is_none(),
            "end must follow transaction settlement"
        );
        self.audit.lock().unwrap().ends += 1;
        self.active = false;
        self.selected = None;
        Ok(())
    }

    fn supports_prepared_poll(&self) -> bool {
        true
    }

    fn poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
        self.prepare_poll(state)?;
        self.commit_prepared_poll()
    }

    fn prepare_poll(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> nuwax_exploration::ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
        assert!(self.pending.is_none());
        self.audit.lock().unwrap().prepares += 1;
        if let Some(gate) = &self.prepare_gate {
            gate.hold();
        }
        let mut pose = state.robot_pose.clone();
        pose.pose.position.x = 2.5;
        // An empty preview ID is deliberate: allocation is a commit-side effect.
        pose.pose.position.x = self.candidate_x;
        let poll = if self.poll_no_frontier {
            ExplorationPoll::NoFrontier
        } else {
            self.stop_reason.map_or_else(
                || ExplorationPoll::Goal(FrontierGoal::new("", pose, 1.0)),
                ExplorationPoll::Stop,
            )
        };
        self.pending = Some(poll.clone());
        Ok(nuwax_exploration::PreparedExplorationPoll::new(Ok(poll)))
    }

    fn commit_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
        if let Some(gate) = &self.commit_gate {
            gate.hold();
        }
        let mut poll = self
            .pending
            .take()
            .ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)?;
        let mut audit = self.audit.lock().unwrap();
        audit.commits += 1;
        if let ExplorationPoll::Goal(goal) = &mut poll {
            goal.id = format!("owner-script.{}", audit.allocated_ids.len() + 1);
            audit.allocated_ids.push(goal.id.clone());
            self.selected = Some(goal.clone());
        }
        Ok(poll)
    }

    fn discard_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.pending
            .take()
            .ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)?;
        self.audit.lock().unwrap().discards += 1;
        Ok(())
    }

    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> nuwax_exploration::ExploreResult<()> {
        assert_eq!(
            self.selected.as_ref().map(|goal| goal.id.as_str()),
            Some(goal_id)
        );
        if let Some(gate) = &self.feedback_gate {
            gate.hold();
        }
        let selected = self.selected.take().expect("selected goal checked above");
        if feedback == ExplorationGoalFeedback::Reached {
            self.history.record_goal_reached(Point2::new(
                selected.pose.pose.position.x,
                selected.pose.pose.position.y,
            ));
        }
        self.audit
            .lock()
            .unwrap()
            .feedback
            .push((goal_id.into(), feedback));
        Ok(())
    }

    fn checkpoint(&self) -> nuwax_exploration::ExploreResult<String> {
        self.history
            .validate_checkpoint()
            .map_err(nuwax_exploration::ExploreError::SourceError)?;
        serde_json::to_string(&(1_u32, &self.history))
            .map_err(|error| nuwax_exploration::ExploreError::SourceError(error.to_string()))
    }

    fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
        nuwax_exploration::ExplorationSourceDiagnostics {
            source_name: "wavefront".into(),
            is_active: self.active,
            last_goal_id: self.selected.as_ref().map(|goal| goal.id.clone()),
            last_error: None,
        }
    }
}

async fn install_owner_runtime_script(handle: &ExploreRuntimeHandle, source: OwnerRuntimeScript) {
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(source);
    controller.begin_exploration().unwrap();
    owner_watchdog(handle.install_compute_controller(controller))
        .await
        .unwrap();
}

async fn wait_owner_runtime_state(
    handle: &ExploreRuntimeHandle,
    predicate: impl Fn(&ExploreRuntimeInner) -> bool,
) {
    owner_watchdog(async {
        loop {
            let notified = handle.compute_service.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            if predicate(&*handle.inner.lock().await) {
                return;
            }
            notified.await;
        }
    })
    .await;
}

async fn stop_owner_runtime_child(handle: &ExploreRuntimeHandle, request_id: &str) {
    let child = handle.snapshot().child_task_id;
    let lease = active_child_lease(handle).await;
    assert_eq!(
        owner_watchdog(handle.stop(request_id)).await.status,
        STATUS_STOPPING
    );
    // Fixture acknowledgement verifies protocol matching, not physical stopping.
    owner_watchdog(handle.report_navigation_result(
        &child,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
    ))
    .await
    .unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_same_domain_replacement_discards_history_and_id_then_progresses() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    assert_eq!(
        handle
            .start("owner-map-race", "prefer_wavefront")
            .await
            .status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeScript::new(audit.clone());
    source.prepare_gate = Some(gate.clone());
    install_owner_runtime_script(&handle, source).await;
    let before = handle.inspect_compute_checkpoint().await.unwrap();
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    assert_eq!(audit.lock().unwrap().prepares, 1);
    assert!(owner_watchdog(ingest_committed_costmap_fixture(
        &handle,
        frontier_costmap(2),
        2
    ))
    .await
    .unwrap());
    assert!(handle.inner.lock().await.logical_active);
    assert!(sink.submitted.lock().unwrap().is_empty());
    gate.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (
                audit.begins,
                audit.ends,
                audit.prepares,
                audit.commits,
                audit.discards
            ),
            (1, 0, 1, 0, 1)
        );
        assert!(audit.allocated_ids.is_empty());
        assert!(audit.feedback.is_empty());
    }
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        before[0].payload
    );
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert_eq!(handle.inner.lock().await.child_sequence, 0);
    let fresh = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, "owner-map-race.nav.1");
    {
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(submitted[0].frontier_goal_id, "owner-script.1");
        assert_eq!(submitted[0].intent.committed_map.revision, 2);
    }
    {
        let audit = audit.lock().unwrap();
        assert_eq!((audit.prepares, audit.commits, audit.discards), (2, 1, 1));
        assert_eq!(audit.allocated_ids, vec!["owner-script.1"]);
        assert!(audit.feedback.is_empty());
    }
    let status = handle.inner.lock().await.compute.status().unwrap();
    assert_eq!(status.incarnation, incarnation);
    assert_eq!(status.high_water.workers, 1);
    stop_owner_runtime_child(&handle, "owner-map-race").await;
    assert_eq!(audit.lock().unwrap().ends, 1);
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_held_install_ack_cannot_revive_replaced_map_domain() {
    use crate::explore_compute_owner::{OperationKind, OperationPhase};

    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.inner.lock().await.compute.hold_install_ack = true;
    let start = tokio::spawn({
        let handle = handle.clone();
        async move { handle.start("owner-install-race", "prefer_wavefront").await }
    });
    wait_owner_runtime_state(&handle, |guard| {
        guard.compute.start_reservation.is_some()
            && guard.compute.status().is_some_and(|status| {
                matches!(
                    status.operation,
                    Some((_, OperationKind::Lifecycle, OperationPhase::Acknowledged))
                )
            })
    })
    .await;
    assert!(
        !start.is_finished(),
        "start is held after native installation"
    );
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let mut replacement = committed_map_identity(2, "map");
    replacement.session_id = "owner-replacement-domain".into();
    assert!(owner_watchdog(
        handle.ingest_committed_costmap_snapshot(frontier_costmap(2), replacement.clone())
    )
    .await
    .unwrap());
    {
        let guard = handle.inner.lock().await;
        assert!(!guard.logical_active);
        assert!(guard.compute.start_reservation.is_none());
        assert_eq!(guard.committed_map.as_ref(), Some(&replacement));
        assert_eq!(guard.child_sequence, 0);
    }
    assert!(sink.submitted.lock().unwrap().is_empty());
    handle.inner.lock().await.compute.hold_install_ack = false;
    handle.compute_service.wake.notify_one();
    let superseded = owner_watchdog(start).await.unwrap();
    assert_ne!(superseded.status, STATUS_ACCEPTED);
    assert_eq!(superseded.last_stop_reason, "map_revision_changed");
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert!(!handle.inner.lock().await.logical_active);
    assert!(sink.submitted.lock().unwrap().is_empty());
    let fresh = owner_watchdog(handle.start("owner-install-fresh", "prefer_wavefront")).await;
    assert_eq!(fresh.status, STATUS_ACCEPTED, "{}", fresh.message);
    let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    assert_eq!(active.child_task_id, "owner-install-fresh.nav.1");
    {
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(submitted[0].intent.committed_map, replacement);
    }
    let status = handle.inner.lock().await.compute.status().unwrap();
    assert_eq!(status.incarnation, incarnation);
    assert_eq!(status.high_water.workers, 1);
    stop_owner_runtime_child(&handle, "owner-install-fresh").await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_aborted_feedback_caller_duplicate_stop_and_resume_keep_history_once() {
    let (handle, sink) = configured_handle();
    handle
        .ingest_committed_costmap_snapshot_in_frame(
            frontier_costmap(1),
            committed_map_identity(1, "map"),
            1,
        )
        .await
        .unwrap();
    handle
        .start("owner-feedback-aborted", "prefer_wavefront")
        .await;
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeScript::new(audit.clone());
    source.feedback_gate = Some(gate.clone());
    install_owner_runtime_script(&handle, source).await;
    let empty_history = handle.inspect_compute_checkpoint().await.unwrap()[0]
        .payload
        .clone();
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    let lease = active_child_lease(&handle).await;
    let terminal = ChildNavigationResult::Reached(terminal_proof(&lease));
    let first = tokio::spawn({
        let handle = handle.clone();
        let child = active.child_task_id.clone();
        let terminal = terminal.clone();
        async move { handle.report_navigation_result(&child, terminal).await }
    });
    owner_watchdog(gate.entered()).await;
    first.abort();
    assert!(owner_watchdog(first).await.unwrap_err().is_cancelled());
    let duplicate = handle.report_navigation_result(&active.child_task_id, terminal.clone());
    tokio::pin!(duplicate);
    assert!(matches!(
        futures::poll!(duplicate.as_mut()),
        std::task::Poll::Pending
    ));
    assert_eq!(
        owner_watchdog(handle.stop("owner-feedback-aborted"))
            .await
            .status,
        STATUS_STOPPING
    );
    owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
    ))
    .await
    .unwrap();
    assert!(handle.inner.lock().await.active_child.is_none());
    assert_eq!(handle.inner.lock().await.reached_goals, 0);
    assert!(audit.lock().unwrap().feedback.is_empty());
    assert_eq!(
        audit.lock().unwrap().ends,
        0,
        "source end must wait for accepted feedback"
    );
    let checkpoint = handle.checkpoint_progress("owner-feedback-aborted");
    tokio::pin!(checkpoint);
    assert!(matches!(
        futures::poll!(checkpoint.as_mut()),
        std::task::Poll::Pending
    ));
    gate.release();
    owner_watchdog(duplicate).await.unwrap();
    let checkpoint = owner_watchdog(checkpoint).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let saved = ExplorationCheckpoint::parse(&checkpoint).unwrap();
    assert_eq!(saved.reached_goals, 1);
    assert_ne!(saved.sources[0].payload, empty_history);
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        saved.sources[0].payload
    );
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (
                audit.begins,
                audit.ends,
                audit.prepares,
                audit.commits,
                audit.discards
            ),
            (1, 1, 1, 1, 0)
        );
        assert_eq!(audit.allocated_ids, vec!["owner-script.1"]);
        assert_eq!(
            audit.feedback,
            vec![("owner-script.1".into(), ExplorationGoalFeedback::Reached)]
        );
    }
    // Repeating the exact old terminal remains a deduplicated acknowledgement.
    owner_watchdog(handle.report_navigation_result(&active.child_task_id, terminal))
        .await
        .unwrap();
    assert_eq!(audit.lock().unwrap().feedback.len(), 1);
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
    let mut resume = handle
        .inner
        .lock()
        .await
        .active_config
        .as_deref()
        .cloned()
        .unwrap();
    resume.resume_progress_json = checkpoint;
    let started = owner_watchdog(handle.start_with_config("owner-feedback-resumed", resume)).await;
    assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        saved.sources[0].payload
    );
    assert_eq!(handle.inner.lock().await.reached_goals, 1);
    assert_eq!(
        handle
            .inner
            .lock()
            .await
            .compute
            .status()
            .unwrap()
            .incarnation,
        incarnation
    );
    let fresh = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, "owner-feedback-resumed.nav.1");
    assert_eq!(sink.submitted.lock().unwrap().len(), 2);
    assert_eq!(audit.lock().unwrap().feedback.len(), 1);
    let fresh_lease = active_child_lease(&handle).await;
    assert_eq!(
        owner_watchdog(handle.stop("owner-feedback-resumed"))
            .await
            .status,
        STATUS_STOPPING
    );
    owner_watchdog(handle.report_navigation_result(
        &fresh.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&fresh_lease, 2)),
    ))
    .await
    .unwrap();
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_internal_cancelled_feedback_requalifies_map_before_return_dispatch() {
    let (handle, sink) = configured_handle();
    handle
        .ingest_committed_costmap_snapshot_in_frame(
            frontier_costmap(1),
            committed_map_identity(1, "map"),
            1,
        )
        .await
        .unwrap();
    handle
        .start("owner-return-feedback-race", "prefer_wavefront")
        .await;
    enable_test_mission(&handle, Some(0.01)).await;
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeScript::new(audit.clone());
    source.feedback_gate = Some(gate.clone());
    install_owner_runtime_script(&handle, source).await;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    {
        let audit = audit.lock().unwrap();
        assert_eq!((audit.prepares, audit.commits, audit.discards), (1, 1, 0));
        assert_eq!(audit.allocated_ids, vec!["owner-script.1"]);
        assert!(audit.feedback.is_empty());
    }
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(
        owner_watchdog(handle.ingest_committed_costmap_snapshot_in_frame(
            frontier_costmap(2),
            committed_map_identity(2, "map"),
            1,
        ))
        .await
        .unwrap()
    );
    assert!(handle.inner.lock().await.logical_active);
    gate.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert!(
        sink.submitted.lock().unwrap().is_empty(),
        "old route advice cannot dispatch after feedback acknowledgement"
    );
    {
        let guard = handle.inner.lock().await;
        assert_eq!(guard.child_sequence, 0);
        assert!(guard.return_goal_id.is_none());
        assert!(guard.return_reason.is_none());
        assert!(!guard.mission.as_ref().unwrap().returning);
    }
    assert_eq!(
        audit.lock().unwrap().feedback,
        vec![("owner-script.1".into(), ExplorationGoalFeedback::Cancelled)]
    );
    let fresh = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, "owner-return-feedback-race.nav.1");
    {
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(
            submitted[0].frontier_goal_id,
            "owner-return-feedback-race.return.1"
        );
        assert_eq!(submitted[0].intent.committed_map.revision, 2);
    }
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (
                audit.begins,
                audit.ends,
                audit.prepares,
                audit.commits,
                audit.discards
            ),
            (1, 0, 2, 2, 0)
        );
        assert_eq!(
            audit.allocated_ids,
            vec!["owner-script.1", "owner-script.2"]
        );
        assert_eq!(
            audit.feedback,
            vec![
                ("owner-script.1".into(), ExplorationGoalFeedback::Cancelled),
                ("owner-script.2".into(), ExplorationGoalFeedback::Cancelled),
            ]
        );
    }
    stop_owner_runtime_child(&handle, "owner-return-feedback-race").await;
    assert_eq!(audit.lock().unwrap().ends, 1);
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_abandoned_accepted_stop_is_applied_once_even_after_input_changes() {
    use crate::explore_compute_owner::{Decision, Phase};

    for replace_input in [false, true] {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start("owner-accepted-stop", "prefer_wavefront")
            .await;
        let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
        let gate = Arc::new(OwnerRuntimeGate::default());
        let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
        let mut source = OwnerRuntimeScript::new(audit.clone());
        source.commit_gate = Some(gate.clone());
        source.stop_reason = Some(ExplorationStopReason::NoInformationGain);
        install_owner_runtime_script(&handle, source).await;
        let incarnation = handle
            .inner
            .lock()
            .await
            .compute
            .status()
            .unwrap()
            .incarnation;
        let tick = tokio::spawn({
            let handle = handle.clone();
            async move { handle.tick((2.0, 2.0)).await }
        });
        owner_watchdog(gate.entered()).await;
        let status = handle.inner.lock().await.compute.status().unwrap();
        assert_eq!(status.phase, Phase::Finalizing);
        assert_eq!(status.decision.unwrap().decision, Decision::Accept);
        assert_eq!(audit.lock().unwrap().commits, 0);
        assert_eq!(audit.lock().unwrap().ends, 0);
        tick.abort();
        assert!(owner_watchdog(tick).await.unwrap_err().is_cancelled());
        if replace_input {
            assert!(owner_watchdog(ingest_committed_costmap_fixture(
                &handle,
                frontier_costmap(2),
                2
            ))
            .await
            .unwrap());
            assert!(handle.inner.lock().await.logical_active);
        }
        gate.release();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        let terminal = handle.snapshot();
        assert_eq!(
            terminal.status, STATUS_SUCCEEDED,
            "replace_input={replace_input}: {}",
            terminal.message
        );
        assert_eq!(terminal.last_stop_reason, "no_information_gain");
        {
            let guard = handle.inner.lock().await;
            assert!(!guard.logical_active);
            assert!(!guard.committed.active);
            assert!(guard.active_child.is_none());
            assert_eq!(guard.child_sequence, 0);
            assert!(matches!(
                guard.last_poll,
                Some(ExplorationPoll::Stop(
                    ExplorationStopReason::NoInformationGain
                ))
            ));
        }
        {
            let audit = audit.lock().unwrap();
            assert_eq!(
                (
                    audit.begins,
                    audit.ends,
                    audit.prepares,
                    audit.commits,
                    audit.discards
                ),
                (1, 1, 1, 1, 0)
            );
            assert!(audit.allocated_ids.is_empty());
            assert!(audit.feedback.is_empty());
        }
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert_eq!(
            owner_watchdog(handle.tick((2.0, 2.0))).await.status,
            STATUS_SUCCEEDED
        );
        assert_eq!(audit.lock().unwrap().prepares, 1);
        assert_eq!(audit.lock().unwrap().ends, 1);
        let fresh =
            owner_watchdog(handle.start("owner-after-accepted-stop", "prefer_wavefront")).await;
        assert_eq!(fresh.status, STATUS_ACCEPTED, "{}", fresh.message);
        assert_eq!(
            handle
                .inner
                .lock()
                .await
                .compute
                .status()
                .unwrap()
                .incarnation,
            incarnation
        );
        let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        assert_eq!(active.child_task_id, "owner-after-accepted-stop.nav.1");
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        stop_owner_runtime_child(&handle, "owner-after-accepted-stop").await;
        handle.request_compute_shutdown().await;
        owner_watchdog(handle.compute_service.join()).await.unwrap();
    }
}

async fn owner_runtime_active_homeward_fault_cancels_exact_child(
    panic_driver: bool,
    cancel_fails: bool,
) {
    use crate::explore_compute_owner::Phase;

    let (mut handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.start("owner-active-fault", "prefer_wavefront").await;
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    install_owner_runtime_script(&handle, OwnerRuntimeScript::new(audit.clone())).await;
    let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    let lease = active_child_lease(&handle).await;
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    enable_test_mission(&handle, Some(100.0)).await;
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    handle.homeward_test_hook = Some(Arc::new({
        let gate = gate.clone();
        move || {
            gate.hold();
            assert!(panic_driver, "injected HomewardBudget owner panic");
        }
    }));
    let cancel_release = Arc::new(tokio::sync::Notify::new());
    let _cancel_cleanup = OwnerRuntimeAsyncRelease(cancel_release.clone());
    *sink.cancel_release.lock().unwrap() = Some(cancel_release.clone());
    if cancel_fails {
        *sink.cancel_error.lock().unwrap() = Some("owner fault cancel dispatch failed".into());
        *sink.cancel_recovery.lock().unwrap() = Some(ChildNavigationCancelRecovery {
            ticket: ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: 37,
            },
            failure: Some("CANCEL_DISPATCH_FAILED".into()),
        });
    }
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    {
        let guard = handle.inner.lock().await;
        assert_eq!(guard.compute.status().unwrap().phase, Phase::Computing);
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
        assert!(guard.pending_child_cancellation.is_none());
    }
    assert_eq!(
        audit.lock().unwrap().prepares,
        1,
        "active-child budget work cannot poll the source"
    );
    if panic_driver {
        handle.inner.lock().await.compute.panic_driver = true;
        handle.compute_service.wake.notify_one();
    } else {
        gate.release(); // The native HomewardBudget call now panics.
    }
    owner_watchdog(sink.cancel_entered.notified()).await;
    {
        let guard = handle.inner.lock().await;
        assert!(!guard.logical_active);
        assert!(guard.compute.fault.is_some());
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
        assert!(
            guard
                .pending_child_cancellation
                .as_ref()
                .unwrap()
                .dispatch_in_flight
        );
        if panic_driver {
            // Cancellation must start before the held native worker can drain.
            let status = guard.compute.status().unwrap();
            assert!(!status.closed);
            assert_eq!(status.phase, Phase::Computing);
        }
    }
    assert!(handle.blocks_mapping_mutation().await);
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Reached(terminal_proof(&lease)),
    ))
    .await
    .is_err());
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    cancel_release.notify_one();
    gate.release();
    owner_watchdog(tick).await.unwrap();
    let terminal_error = owner_watchdog(handle.compute_service.join())
        .await
        .unwrap_err();
    if panic_driver {
        assert!(
            terminal_error.contains("driver panicked"),
            "{terminal_error}"
        );
    } else {
        assert!(terminal_error.contains("Panic"), "{terminal_error}");
    }
    assert_eq!(
        *sink.cancelled.lock().unwrap(),
        vec![active.child_task_id.clone()]
    );
    let cancel_generation = if cancel_fails { 37 } else { 1 };
    {
        let guard = handle.inner.lock().await;
        let pending = guard.pending_child_cancellation.as_ref().unwrap();
        assert!(!pending.dispatch_in_flight);
        assert_eq!(pending.failure.is_some(), cancel_fails);
        assert_eq!(
            pending.ticket.as_ref().unwrap().cancel_generation,
            cancel_generation
        );
        assert!(pending.ticket.as_ref().unwrap().lease.matches(&lease));
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
        let status = guard.compute.status().unwrap();
        assert_eq!(status.incarnation, incarnation);
        assert!(status.closed);
        assert_eq!(status.high_water.workers, 1);
    }
    let retained = handle.snapshot();
    assert_eq!(retained.child_task_id, active.child_task_id);
    assert_eq!(
        retained.status,
        if cancel_fails {
            STATUS_FAILED
        } else {
            STATUS_STOPPING
        }
    );
    if cancel_fails {
        assert_eq!(retained.last_stop_reason, "child_cancel_failed");
        assert!(retained.global_stop_reconciliation.eligible);
    }
    assert!(handle.blocks_mapping_mutation().await);
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, cancel_generation + 1)),
    ))
    .await
    .is_err());
    assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
    // This injected acknowledgement checks exact protocol ownership only. It
    // does not establish that any actuator or physical robot actually stopped.
    let released = owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, cancel_generation)),
    ))
    .await
    .unwrap();
    assert_eq!(released.status, STATUS_FAILED);
    assert!(released.child_task_id.is_empty());
    assert!(handle.inner.lock().await.active_child.is_none());
    assert!(handle
        .inner
        .lock()
        .await
        .pending_child_cancellation
        .is_none());
    assert!(!handle.blocks_mapping_mutation().await);
    if cancel_fails {
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        assert!(released.ownership_reconciliation_receipt.is_some());
    } else {
        assert_eq!(released.last_stop_reason, "consecutive_failures_exceeded");
    }
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (
                audit.begins,
                audit.ends,
                audit.prepares,
                audit.commits,
                audit.discards
            ),
            (1, 0, 1, 1, 0)
        );
        assert_eq!(audit.allocated_ids, vec!["owner-script.1"]);
        assert!(audit.feedback.is_empty());
    }
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
    assert!(handle
        .checkpoint_progress("owner-active-fault")
        .await
        .is_err());
    assert_eq!(
        owner_watchdog(handle.start("owner-fault-cannot-restart", "prefer_wavefront"))
            .await
            .status,
        STATUS_REJECTED
    );
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
}

#[tokio::test]
async fn compute_owner_active_homeward_panic_uses_zeroed_protocol_and_retains_cancel_failure() {
    for cancel_fails in [false, true] {
        owner_runtime_active_homeward_fault_cancels_exact_child(false, cancel_fails).await;
    }
}

#[tokio::test]
async fn compute_owner_active_driver_panic_cancels_while_native_homeward_remains_held() {
    for cancel_fails in [false, true] {
        owner_runtime_active_homeward_fault_cancels_exact_child(true, cancel_fails).await;
    }
}

#[tokio::test]
async fn compute_owner_first_no_frontier_interval_starts_at_committed_delivery() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle
        .start("owner-delayed-no-frontier", "prefer_wavefront")
        .await;
    let stability = Duration::from_millis(30);
    {
        let mut guard = handle.inner.lock().await;
        Arc::make_mut(guard.active_config.as_mut().unwrap())
            .stop_conditions
            .no_frontier_timeout_ms = Some(stability.as_millis() as u64);
    }
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeScript::new(audit.clone());
    source.prepare_gate = Some(gate.clone());
    source.poll_no_frontier = true;
    install_owner_runtime_script(&handle, source).await;
    let first = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    // This deliberate gate delay exceeds the configured stability interval.
    // It models a slow native prepare; it is not a CPU-speed assertion.
    tokio::time::sleep(stability * 2).await;
    assert!(!first.is_finished());
    assert!(handle.inner.lock().await.no_frontier_since.is_none());
    assert_eq!(audit.lock().unwrap().commits, 0);
    let released_at = Instant::now();
    gate.release();
    let first = owner_watchdog(first).await.unwrap();
    assert_eq!(first.status, STATUS_PENDING, "{}", first.message);
    {
        let guard = handle.inner.lock().await;
        assert!(guard.logical_active);
        assert!(guard.no_frontier_since.unwrap() >= released_at);
        assert!(matches!(guard.last_poll, Some(ExplorationPoll::NoFrontier)));
    }
    assert_eq!(audit.lock().unwrap().commits, 1);
    assert_eq!(audit.lock().unwrap().ends, 0);
    assert!(sink.submitted.lock().unwrap().is_empty());
    // Only a later committed result can observe an interval that began at the
    // first committed delivery. The wait intentionally establishes that age.
    tokio::time::sleep(stability * 2).await;
    let second = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(second.status, STATUS_SUCCEEDED, "{}", second.message);
    assert_eq!(second.last_stop_reason, "no_frontier_timeout");
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    {
        let audit = audit.lock().unwrap();
        assert_eq!((audit.prepares, audit.commits, audit.discards), (2, 2, 0));
        assert_eq!(audit.ends, 1);
        assert!(audit.allocated_ids.is_empty());
        assert!(audit.feedback.is_empty());
    }
    assert!(!handle.inner.lock().await.logical_active);
    assert!(sink.submitted.lock().unwrap().is_empty());
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_admitted_internal_failed_feedback_survives_aborted_tick_and_stop() {
    const CHILD: &str = "NUWAX_TEST_INTERNAL_FAILED_RESUME_CHILD";
    if std::env::var_os(CHILD).is_none() {
        // Isolated process environment, never process-global mutation in a test
        // suite. Both initial startup and resume capture this explicit calibration
        // through the ordinary production path.
        let mut child = std::process::Command::new(std::env::current_exe().unwrap());
        child.args(["explore_runtime::tests::compute_owner_admitted_internal_failed_feedback_survives_aborted_tick_and_stop", "--exact", "--nocapture", "--test-threads=1"]);
        child.env(CHILD, "1");
        for line in include_str!("../tests/exploration-attribution.env").lines() {
            if let Some(binding) = line.strip_prefix("export ") {
                let (key, value) = binding.split_once('=').unwrap();
                child.env(key, value.trim_matches('\''));
            }
        }
        let output = child.output().expect("run calibrated resume fixture");
        assert!(
            output.status.success(),
            "calibrated child failed:\n{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        eprintln!(
            "CALIBRATED_RESUME_CHILD {}",
            String::from_utf8_lossy(&output.stdout)
        );
        return;
    }
    let (handle, sink) = configured_handle();
    handle
        .ingest_committed_costmap_snapshot_in_frame(
            frontier_costmap(1),
            committed_map_identity(1, "map"),
            1,
        )
        .await
        .unwrap();
    let config = ExploreStartConfig {
        motion_admission: crate::nav_runtime::test_motion_admission(),
        return_policy: ReturnPolicy {
            maximum_distance_m: None,
            reserve_distance_m: 0.0,
            return_on_completion: true,
        },
        ..ExploreStartConfig::default()
    };
    let started = handle
        .start_with_config("owner-internal-failed", config)
        .await;
    assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
    let audit = Arc::new(StdMutex::new(OwnerRuntimeAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeScript::new(audit.clone());
    source.feedback_gate = Some(gate.clone());
    // This is unknown space in the real fixture: the completed candidate query
    // produces internal Failed feedback, rather than budget cancellation.
    source.candidate_x = 5.0;
    install_owner_runtime_script(&handle, source).await;
    let original_checkpoint = handle.inspect_compute_checkpoint().await.unwrap()[0]
        .payload
        .clone();
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await; // The internal Failed transaction is admitted.
    assert!(audit.lock().unwrap().feedback.is_empty());
    assert!(sink.submitted.lock().unwrap().is_empty());
    tick.abort();
    assert!(owner_watchdog(tick).await.unwrap_err().is_cancelled());
    assert_eq!(
        owner_watchdog(handle.stop("owner-internal-failed"))
            .await
            .status,
        STATUS_CANCELLED
    );
    assert_eq!(
        owner_watchdog(handle.stop("owner-internal-failed"))
            .await
            .status,
        STATUS_CANCELLED
    );
    assert_eq!(
        audit.lock().unwrap().ends,
        0,
        "source end waits for admitted feedback"
    );
    let checkpoint = handle.checkpoint_progress("owner-internal-failed");
    tokio::pin!(checkpoint);
    assert!(matches!(
        futures::poll!(checkpoint.as_mut()),
        std::task::Poll::Pending
    ));
    gate.release();
    let checkpoint = owner_watchdog(checkpoint).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let saved = ExplorationCheckpoint::parse(&checkpoint).unwrap();
    assert_eq!(saved.reached_goals, 0);
    assert_eq!(saved.sources[0].payload, original_checkpoint);
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        original_checkpoint
    );
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.prepares, audit.commits, audit.discards, audit.ends),
            (1, 1, 0, 1)
        );
        assert_eq!(audit.allocated_ids, vec!["owner-script.1"]);
        assert_eq!(audit.feedback.len(), 1);
        assert_eq!(audit.feedback[0].0, "owner-script.1");
        assert!(
            matches!(&audit.feedback[0].1, ExplorationGoalFeedback::Failed(message)
            if message.contains("候选往返路线不可通行"))
        );
    }
    assert!(sink.cancelled.lock().unwrap().is_empty());
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert_eq!(handle.inner.lock().await.reached_goals, 0);
    let mut resume = handle
        .inner
        .lock()
        .await
        .active_config
        .as_deref()
        .cloned()
        .unwrap();
    resume.resume_progress_json = checkpoint;
    let started =
        owner_watchdog(handle.start_with_config("owner-internal-failed-resumed", resume)).await;
    assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
    assert_eq!(
        handle
            .inner
            .lock()
            .await
            .compute
            .status()
            .unwrap()
            .incarnation,
        incarnation
    );
    assert_eq!(
        handle.inspect_compute_checkpoint().await.unwrap()[0].payload,
        original_checkpoint
    );
    let fresh = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, "owner-internal-failed-resumed.nav.1");
    assert_eq!(
        audit.lock().unwrap().feedback.len(),
        1,
        "accepted Failed is never replayed as Cancelled"
    );
    stop_owner_runtime_child(&handle, "owner-internal-failed-resumed").await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn scheduler_observes_compute_failure_while_its_blocked_pose_loop_remains_live() {
    use crate::explore_compute_owner::Phase;

    let (mut handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle
        .start("owner-independent-terminal", "prefer_wavefront")
        .await;
    let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    let lease = active_child_lease(&handle).await;
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    enable_test_mission(&handle, Some(100.0)).await;
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _gate_cleanup = OwnerRuntimeGateRelease(gate.clone());
    handle.homeward_test_hook = Some(Arc::new({
        let gate = gate.clone();
        move || gate.hold()
    }));
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    let pose_entered = Arc::new(tokio::sync::Notify::new());
    let scheduler = handle.spawn_scheduler(
        Arc::new(BlockingPoseProvider {
            entered: pose_entered.clone(),
        }),
        Duration::from_millis(5),
    );
    owner_watchdog(pose_entered.notified()).await;
    handle.inner.lock().await.compute.panic_driver = true;
    handle.compute_service.wake.notify_one();
    wait_owner_runtime_state(&handle, |guard| {
        guard
            .pending_child_cancellation
            .as_ref()
            .is_some_and(|pending| !pending.dispatch_in_flight && pending.ticket.is_some())
    })
    .await;
    let terminal = scheduler.wait_terminal();
    tokio::pin!(terminal);
    assert!(matches!(
        futures::poll!(terminal.as_mut()),
        std::task::Poll::Pending
    ));
    {
        let guard = handle.inner.lock().await;
        let status = guard.compute.status().unwrap();
        assert_eq!(status.incarnation, incarnation);
        assert_eq!(status.phase, Phase::Computing);
        assert!(
            !status.closed,
            "the original held native owner is still supervised"
        );
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
    }
    assert!(!scheduler.shutdown_is_requested());
    assert!(!scheduler.inner.worker_abort.is_finished());
    assert!(scheduler.inner.terminal_rx.borrow().is_none());
    gate.release();
    let failure = owner_watchdog(terminal).await.unwrap_err();
    assert!(failure.contains("driver panicked"), "{failure}");
    owner_watchdog(tick).await.unwrap();
    // The failure must come from compute supervision, independently of the
    // scheduler's still-live pose-provider await and its own terminal channel.
    assert!(!scheduler.shutdown_is_requested());
    assert!(!scheduler.inner.worker_abort.is_finished());
    assert!(scheduler.inner.terminal_rx.borrow().is_none());
    assert_eq!(
        *sink.cancelled.lock().unwrap(),
        vec![active.child_task_id.clone()]
    );
    assert!(handle.blocks_mapping_mutation().await);
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 2)),
    ))
    .await
    .is_err());
    // Software acknowledgement proves exact protocol reconciliation only.
    let released = owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
    ))
    .await
    .unwrap();
    assert_eq!(released.status, STATUS_FAILED);
    assert!(released.child_task_id.is_empty());
    let settled = owner_watchdog(scheduler.shutdown_with_settlement()).await;
    assert!(
        settled.producers_settled(),
        "root shutdown must join the original native owner"
    );
    assert!(settled
        .into_result()
        .unwrap_err()
        .contains("driver panicked"));
    let status = handle.inner.lock().await.compute.status().unwrap();
    assert_eq!(status.incarnation, incarnation);
    assert!(status.closed);
    assert_eq!(status.high_water.workers, 1);
    assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
}

async fn owner_runtime_exhausted_control_keeps_exact_child(
    generation_exhausted: bool,
    lifecycle_exhausted: bool,
) {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle
        .start("owner-exhausted-control", "prefer_wavefront")
        .await;
    let active = owner_watchdog(handle.tick((2.0, 2.0))).await;
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    let lease = active_child_lease(&handle).await;
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let original_generation = handle.generation.load(Ordering::SeqCst);
    let cancel_release = Arc::new(tokio::sync::Notify::new());
    let _cancel_cleanup = OwnerRuntimeAsyncRelease(cancel_release.clone());
    *sink.cancel_release.lock().unwrap() = Some(cancel_release.clone());
    owner_watchdog(
        handle.inject_compute_control_stop_for_tests(generation_exhausted, lifecycle_exhausted),
    )
    .await
    .unwrap();
    owner_watchdog(sink.cancel_entered.notified()).await;
    let exhausted_counters = {
        let guard = handle.inner.lock().await;
        assert!(!guard.logical_active);
        assert!(guard.compute.fault.is_some());
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
        assert!(
            guard
                .pending_child_cancellation
                .as_ref()
                .unwrap()
                .dispatch_in_flight
        );
        if generation_exhausted {
            assert_eq!(guard.generation, u64::MAX);
            assert_eq!(handle.generation.load(Ordering::SeqCst), u64::MAX);
        } else {
            assert_eq!(guard.generation, original_generation + 1);
            assert_eq!(
                handle.generation.load(Ordering::SeqCst),
                original_generation + 1
            );
        }
        if lifecycle_exhausted {
            assert_eq!(guard.compute.lifecycle_counter_for_tests(), u64::MAX);
        }
        (
            guard.generation,
            guard.compute.lifecycle_counter_for_tests(),
        )
    };
    assert!(handle.blocks_mapping_mutation().await);
    // Rejoin the retained terminal token while its exact cancellation is held.
    // No second counted action, generation allocation, or cancel is permitted.
    owner_watchdog(handle.inject_compute_control_stop_for_tests(false, false))
        .await
        .unwrap();
    {
        let guard = handle.inner.lock().await;
        assert_eq!(
            (
                guard.generation,
                guard.compute.lifecycle_counter_for_tests()
            ),
            exhausted_counters
        );
        assert_eq!(
            handle.generation.load(Ordering::SeqCst),
            exhausted_counters.0
        );
    }
    assert!(sink.cancelled.lock().unwrap().is_empty());
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Reached(terminal_proof(&lease)),
    ))
    .await
    .is_err());
    cancel_release.notify_one();
    let failure = owner_watchdog(handle.compute_service.join())
        .await
        .unwrap_err();
    let exhausted_name = if generation_exhausted {
        "generation exhausted"
    } else {
        "lifecycle sequence exhausted"
    };
    assert!(failure.contains(exhausted_name), "{failure}");
    {
        let guard = handle.inner.lock().await;
        let pending = guard.pending_child_cancellation.as_ref().unwrap();
        assert!(!pending.dispatch_in_flight);
        assert!(pending.failure.is_none());
        let ticket = pending.ticket.as_ref().unwrap();
        assert!(ticket.lease.matches(&lease));
        assert_eq!(ticket.cancel_generation, 1);
        assert!(guard.active_child.as_ref().unwrap().lease.matches(&lease));
        let status = guard.compute.status().unwrap();
        assert_eq!(status.incarnation, incarnation);
        assert!(status.closed);
        assert_eq!(status.high_water.workers, 1);
    }
    assert_eq!(
        *sink.cancelled.lock().unwrap(),
        vec![active.child_task_id.clone()]
    );
    assert!(
        owner_watchdog(handle.checkpoint_progress("owner-exhausted-control"))
            .await
            .is_err()
    );
    assert_eq!(
        owner_watchdog(handle.start("owner-exhausted-before-zero", "prefer_wavefront"))
            .await
            .status,
        STATUS_REJECTED
    );
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 2)),
    ))
    .await
    .is_err());
    assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
    // This is a software protocol proof, not evidence of physical stopping.
    let released = owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
    ))
    .await
    .unwrap();
    assert_eq!(released.status, STATUS_FAILED);
    assert!(released.child_task_id.is_empty());
    assert!(!handle.blocks_mapping_mutation().await);
    {
        let guard = handle.inner.lock().await;
        assert!(guard.active_child.is_none());
        assert!(guard.pending_child_cancellation.is_none());
        assert!(guard.compute.fault.is_some());
        if generation_exhausted {
            assert_eq!(guard.generation, u64::MAX);
            assert_eq!(handle.generation.load(Ordering::SeqCst), u64::MAX);
        }
        if lifecycle_exhausted {
            assert_eq!(guard.compute.lifecycle_counter_for_tests(), u64::MAX);
        }
    }
    // Repeated terminal observation cannot make a permanently faulted owner
    // reusable or turn counter exhaustion into an unowned cancellation retry.
    owner_watchdog(handle.inject_compute_control_stop_for_tests(false, false))
        .await
        .unwrap();
    assert_eq!(
        owner_watchdog(handle.compute_service.join())
            .await
            .unwrap_err(),
        failure
    );
    assert!(
        owner_watchdog(handle.checkpoint_progress("owner-exhausted-control"))
            .await
            .is_err()
    );
    assert_eq!(
        owner_watchdog(handle.start("owner-exhausted-after-zero", "prefer_wavefront"))
            .await
            .status,
        STATUS_REJECTED
    );
    assert!(owner_watchdog(handle.inspect_compute_checkpoint())
        .await
        .is_err());
    assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(
        handle
            .inner
            .lock()
            .await
            .compute
            .status()
            .unwrap()
            .incarnation,
        incarnation
    );
}

#[tokio::test]
async fn compute_owner_generation_exhaustion_retains_one_exact_child_cancellation() {
    owner_runtime_exhausted_control_keeps_exact_child(true, false).await;
}

#[tokio::test]
async fn compute_owner_lifecycle_exhaustion_retains_one_exact_child_cancellation() {
    owner_runtime_exhausted_control_keeps_exact_child(false, true).await;
}

#[tokio::test]
async fn compute_owner_final_shutdown_is_idempotent_while_held_and_after_closure() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
        .await
        .unwrap();
    handle.start("owner-final-once", "prefer_wavefront").await;
    let feedback = Arc::new(StdMutex::new(Vec::new()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    let mut source = OwnerRuntimeSource::new(feedback);
    source.prepare_gate = Some(gate.clone());
    install_owner_runtime_source(&handle, source).await;
    let tick = tokio::spawn({
        let handle = handle.clone();
        async move { handle.tick((2.0, 2.0)).await }
    });
    owner_watchdog(gate.entered()).await;
    handle.request_compute_shutdown().await;
    let serial = handle
        .inner
        .lock()
        .await
        .compute
        .lifecycle_counter_for_tests();
    for _ in 0..3 {
        handle.request_compute_shutdown().await;
    }
    assert_eq!(
        handle
            .inner
            .lock()
            .await
            .compute
            .lifecycle_counter_for_tests(),
        serial
    );
    gate.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.compute_service.join()).await.unwrap();
    handle.request_compute_shutdown().await;
    assert_eq!(
        handle.stop("owner-final-once").await.status,
        STATUS_CANCELLED
    );
    assert_eq!(
        handle
            .inner
            .lock()
            .await
            .compute
            .lifecycle_counter_for_tests(),
        serial
    );
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let rejected = owner_watchdog(handle.start("owner-cannot-reopen", "prefer_wavefront")).await;
    assert_eq!(rejected.status, STATUS_REJECTED);
    assert!(
        rejected.message.contains("shut down"),
        "{}",
        rejected.message
    );
    assert!(sink.submitted.lock().unwrap().is_empty());
}

#[tokio::test]
async fn compute_owner_ownerless_final_shutdown_never_spawns_a_late_worker() {
    let (handle, _) = configured_handle();
    assert!(handle.inner.lock().await.compute.status().is_none());
    handle.request_compute_shutdown().await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
    let rejected = owner_watchdog(handle.start("ownerless-closed", "prefer_wavefront")).await;
    assert_eq!(rejected.status, STATUS_REJECTED);
    assert!(
        rejected.message.contains("shut down"),
        "{}",
        rejected.message
    );
    let guard = handle.inner.lock().await;
    assert!(guard.compute.status().is_none());
    assert_eq!(guard.compute.lifecycle_counter_for_tests(), 0);
    assert!(!guard.logical_active);
}
