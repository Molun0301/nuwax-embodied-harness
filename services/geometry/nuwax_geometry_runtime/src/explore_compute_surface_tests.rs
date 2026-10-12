// Include inside explore_runtime::tests, alongside explore_compute_runtime_tests.rs.
// Uses the existing async-notified native gate, release-on-drop guard and watchdog.
// Every selection, staged delta, commit, discard and feedback below delegates to
// the real SurfaceGoalSourceAdapter. No LCM, network, hardware or clock rewrites.

#[derive(Clone)]
struct SurfaceOwnerCapture {
    original: nuwax_exploration::FrozenSurfaceInput,
    pose: PoseStamped,
    preview: nuwax_exploration::ExploreResult<ExplorationPoll>,
    checkpoint_before: String,
    checkpoint_after: String,
}

#[derive(Default)]
struct SurfaceOwnerAudit {
    captures: Vec<SurfaceOwnerCapture>,
    commits: Vec<ExplorationPoll>,
    discards: usize,
    feedback: Vec<(String, ExplorationGoalFeedback)>,
    feedback_checkpoints: Vec<(String, String)>,
}

struct GatedRealSurfaceSource {
    source: SurfaceGoalSourceAdapter,
    audit: Arc<StdMutex<SurfaceOwnerAudit>>,
    after_prepare: Option<Arc<OwnerRuntimeGate>>,
    before_commit: Option<Arc<OwnerRuntimeGate>>,
}

impl ExplorationGoalSource for GatedRealSurfaceSource {
    fn source_name(&self) -> std::borrow::Cow<'static, str> {
        self.source.source_name()
    }
    fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.source.begin()
    }
    fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.source.end()
    }
    fn supports_prepared_poll(&self) -> bool {
        self.source.supports_prepared_poll()
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
        let planning = state
            .frozen_surface_input
            .expect("the actual Surface compute owner must supply its frozen planning input");
        let original = planning.original_input().clone();
        assert!(!Arc::ptr_eq(
            original.snapshot().expect("original Surface source"),
            planning
                .query_snapshot()
                .expect("private Surface query view"),
        ));
        let checkpoint_before = self.source.checkpoint()?;
        let prepared = self.source.prepare_poll(state)?;
        let checkpoint_after = self.source.checkpoint()?;
        self.audit
            .lock()
            .unwrap()
            .captures
            .push(SurfaceOwnerCapture {
                original,
                pose: state.robot_pose.clone(),
                preview: prepared.outcome().clone(),
                checkpoint_before,
                checkpoint_after,
            });
        // The real source has finished preparing, but no proposal is visible to
        // the actor yet. Take makes the hold one-shot; later progress is real.
        if let Some(gate) = self.after_prepare.take() {
            gate.hold();
        }
        Ok(prepared)
    }
    fn commit_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
        // Reaching this hold means the actor's immutable Accept has linearized.
        if let Some(gate) = self.before_commit.take() {
            gate.hold();
        }
        let poll = self.source.commit_prepared_poll()?;
        self.audit.lock().unwrap().commits.push(poll.clone());
        Ok(poll)
    }
    fn discard_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<()> {
        self.source.discard_prepared_poll()?;
        self.audit.lock().unwrap().discards += 1;
        Ok(())
    }
    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> nuwax_exploration::ExploreResult<()> {
        let before = self.source.checkpoint()?;
        self.source
            .report_goal_feedback(goal_id, feedback.clone())?;
        let after = self.source.checkpoint()?;
        let mut audit = self.audit.lock().unwrap();
        audit.feedback.push((goal_id.into(), feedback));
        audit.feedback_checkpoints.push((before, after));
        Ok(())
    }
    fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
        self.source.frontier_snapshot()
    }
    fn checkpoint(&self) -> nuwax_exploration::ExploreResult<String> {
        self.source.checkpoint()
    }
    fn restore_checkpoint(&mut self, payload: &str) -> nuwax_exploration::ExploreResult<()> {
        self.source.restore_checkpoint(payload)
    }
    fn restore_checkpoint_for_replan(
        &mut self,
        payload: &str,
    ) -> nuwax_exploration::ExploreResult<()> {
        self.source.restore_checkpoint_for_replan(payload)
    }
    fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
        self.source.diagnostics()
    }
}

async fn install_gated_real_surface_source(
    handle: &ExploreRuntimeHandle,
    audit: Arc<StdMutex<SurfaceOwnerAudit>>,
    after_prepare: Arc<OwnerRuntimeGate>,
    before_commit: Option<Arc<OwnerRuntimeGate>>,
) -> SurfaceInputReceiver {
    let (receiver, config, body_to_support) = {
        let guard = handle.inner.lock().await;
        let mode = guard.surface_mode.as_ref().unwrap();
        (
            mode.receiver.clone(),
            guard.active_config.as_deref().unwrap().clone(),
            mode.config.as_ref().unwrap().follower.body_to_support_z_m,
        )
    };
    let mut source =
        SurfaceGoalSourceAdapter::new("surface", body_to_support, receiver.clone()).unwrap();
    if let Some(model) = nuwax_nav_types_messages::NavigationCapabilities::from_env()
        .unwrap()
        .and_then(|profile| profile.observation)
    {
        source.set_observation_model(model).unwrap();
    }
    source
        .configure_constraints(
            config.boundary,
            config.max_navigation_retries,
            config.blacklist_radius_m,
        )
        .unwrap();
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(GatedRealSurfaceSource {
        source,
        audit,
        after_prepare: Some(after_prepare),
        before_commit,
    });
    controller.begin_exploration().unwrap();
    owner_watchdog(handle.install_compute_controller(controller))
        .await
        .unwrap();
    receiver
}

// A new envelope is an explicit fixture publication, with a real receipt taken
// before begin. The unchanged occupancy identity intentionally isolates Surface
// source qualification from the independent committed-map/domain guards.
async fn publish_surface_owner_receipt(
    handle: &ExploreRuntimeHandle,
    committed: &schema::CommittedMapIdentity,
    revision: i32,
    snapshot: Arc<SurfaceSnapshot>,
) -> SurfaceMapTicket {
    let mut message = surface_message("map", revision);
    message.committed_map = committed.clone();
    // Redeliver the original source receipt; only the notification advances.
    message.cloud_occupancy_revision = committed.revision;
    message.commit_boot_nonce = committed.commit_boot_nonce;
    message.commit_sequence = committed.commit_sequence;
    message.source_timestamp_ns = committed.source_timestamp_ns;
    message.cloud.header.stamp_sec = (committed.source_timestamp_ns / 1_000_000_000) as i32;
    message.cloud.header.stamp_nsec = (committed.source_timestamp_ns % 1_000_000_000) as i32;
    let received = now_monotonic_nanos();
    let ticket = owner_watchdog(handle.begin_surface_map_update(&message, received))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(ticket.received_monotonic_ns, received);
    assert!(owner_watchdog(handle.finish_surface_map_update(ticket.clone(), Ok(snapshot))).await);
    let guard = handle.inner.lock().await;
    assert_eq!(guard.costmap_received_monotonic_ns, received);
    assert_eq!(guard.committed_map.as_ref(), Some(committed));
    assert!(guard.logical_active || guard.active_config.is_none());
    ticket
}

fn surface_owner_goal(poll: &ExplorationPoll) -> &FrontierGoal {
    match poll {
        ExplorationPoll::Goal(goal) => goal,
        other => panic!("expected a real Surface goal, got {other:?}"),
    }
}

fn assert_surface_owner_capture(
    capture: &SurfaceOwnerCapture,
    source: &Arc<SurfaceSnapshot>,
    notification: u64,
    pose: &ExploreRobotPose,
) {
    assert_eq!(capture.original.notification(), notification);
    assert!(Arc::ptr_eq(capture.original.snapshot().unwrap(), source));
    assert_eq!(capture.pose.stamp.nanos, pose.source_timestamp_ns as i64);
    assert_eq!(capture.pose.stamp.clock, ClockKind::Unix);
    assert_eq!(
        capture.pose.pose.position,
        Point3::new(pose.x, pose.y, pose.z)
    );
    assert_eq!(capture.pose.pose.orientation, pose.orientation);
    assert_eq!(capture.pose.frame_id.as_str(), pose.frame_id);
    assert_eq!(capture.checkpoint_before, capture.checkpoint_after);
    assert!(surface_owner_goal(capture.preview.as_ref().unwrap())
        .id
        .ends_with("-1"));
}

async fn finish_surface_owner_success(
    handle: &ExploreRuntimeHandle,
    request_id: &str,
    active: &ExploreRuntimeSnapshot,
    audit: &Arc<StdMutex<SurfaceOwnerAudit>>,
) {
    let lease = active_child_lease(handle).await;
    let terminal = ChildNavigationResult::Reached(terminal_proof(&lease));
    owner_watchdog(handle.report_navigation_result(&active.child_task_id, terminal.clone()))
        .await
        .unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    // Exact duplicate terminal acknowledgement must not count this goal twice.
    owner_watchdog(handle.report_navigation_result(&active.child_task_id, terminal))
        .await
        .unwrap();
    assert_eq!(handle.inner.lock().await.reached_goals, 1);
    assert_eq!(
        audit
            .lock()
            .unwrap()
            .feedback
            .iter()
            .filter(|(_, feedback)| { matches!(feedback, ExplorationGoalFeedback::Reached) })
            .count(),
        1,
    );
    let sources = owner_watchdog(handle.inspect_compute_checkpoint())
        .await
        .unwrap();
    let checkpoint: serde_json::Value = serde_json::from_str(&sources[0].payload).unwrap();
    assert!(checkpoint[1]
        .as_array()
        .unwrap()
        .iter()
        .any(|row| row["reached"] == true));
    assert_eq!(
        owner_watchdog(handle.stop(request_id)).await.status,
        STATUS_CANCELLED
    );
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn compute_owner_surface_repeated_same_root_receipts_preserve_capture_then_progress() {
    let (handle, sink) = configured_handle();
    let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
    let request_id = "owner-surface-receipts";
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    let receiver =
        install_gated_real_surface_source(&handle, audit.clone(), gate.clone(), None).await;
    let original = receiver.freeze().unwrap();
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(gate.entered()).await;
    let captured_receipt = handle.inner.lock().await.costmap_received_monotonic_ns;
    for revision in 2..=5 {
        publish_surface_owner_receipt(&handle, &message.committed_map, revision, snapshot.clone())
            .await;
        let refreshed = receiver.freeze().unwrap();
        assert!(original.same_source_view(&refreshed));
        assert!(Arc::ptr_eq(refreshed.snapshot().unwrap(), &snapshot));
        assert_eq!(
            refreshed.notification(),
            original.notification() + (revision - 1) as u64
        );
        assert!(!tick.is_finished());
        assert!(sink.submitted.lock().unwrap().is_empty());
        let audit = audit.lock().unwrap();
        assert_eq!(audit.captures.len(), 1);
        assert!(audit.commits.is_empty());
        assert_eq!(audit.discards, 0);
        assert!(audit.feedback.is_empty());
        assert_surface_owner_capture(
            &audit.captures[0],
            &snapshot,
            original.notification(),
            &pose,
        );
    }
    let latest = receiver.freeze().unwrap();
    let latest_receipt = handle.inner.lock().await.costmap_received_monotonic_ns;
    assert!(latest_receipt >= captured_receipt);
    // Reads do not create observations, touch the receipt, or rewrite the pose.
    for _ in 0..3 {
        let _ = handle.snapshot();
        let _ = receiver.snapshot().unwrap();
        assert_eq!(
            receiver.freeze().unwrap().notification(),
            latest.notification()
        );
        assert_eq!(
            handle.inner.lock().await.costmap_received_monotonic_ns,
            latest_receipt
        );
    }
    gate.release();
    let active = owner_watchdog(tick).await.unwrap();
    assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
    assert_eq!(active.child_task_id, "owner-surface-receipts.nav.1");
    assert_eq!(active.selected_source, "surface");
    let first_goal = sink.submitted.lock().unwrap()[0].frontier_goal_id.clone();
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (1, 1, 0)
        );
        assert_eq!(surface_owner_goal(&audit.commits[0]).id, first_goal);
        assert_surface_owner_capture(
            &audit.captures[0],
            &snapshot,
            original.notification(),
            &pose,
        );
    }
    let lease = active_child_lease(&handle).await;
    let reason = "SafetyFailed cause=StaleInput: held Surface receipt fixture".to_string();
    let terminal = ChildNavigationResult::Failed {
        proof: terminal_proof(&lease),
        reason: reason.clone(),
    };
    let mut wrong_lease = lease.clone();
    wrong_lease.goal_generation += 1;
    assert!(owner_watchdog(handle.report_navigation_result(
        &active.child_task_id,
        ChildNavigationResult::Failed {
            proof: terminal_proof(&wrong_lease),
            reason: reason.clone()
        },
    ))
    .await
    .is_err());
    assert!(audit.lock().unwrap().feedback.is_empty());
    owner_watchdog(handle.report_navigation_result(&active.child_task_id, terminal.clone()))
        .await
        .unwrap();
    owner_watchdog(handle.report_navigation_result(&active.child_task_id, terminal))
        .await
        .unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            audit.feedback,
            vec![(
                first_goal,
                ExplorationGoalFeedback::InputUnavailable(reason)
            )]
        );
        assert_eq!(
            audit.feedback_checkpoints[0].0, audit.feedback_checkpoints[0].1,
            "input interruption must not blacklist a Surface frontier or service a regional round"
        );
    }
    assert_eq!(handle.inner.lock().await.reached_goals, 0);
    // The only newer observation arrived DURING preparation. A commit which
    // accidentally consumed the live notification would strand this next tick.
    let next =
        owner_watchdog(handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)))
            .await;
    assert_eq!(next.status, STATUS_ACTIVE, "{}", next.message);
    assert_eq!(next.child_task_id, "owner-surface-receipts.nav.2");
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (2, 2, 0)
        );
        assert_eq!(
            audit.captures[1].original.notification(),
            latest.notification()
        );
        assert!(surface_owner_goal(&audit.commits[1]).id.ends_with("-2"));
    }
    assert_eq!(sink.submitted.lock().unwrap().len(), 2);
    assert!(sink.cancelled.lock().unwrap().is_empty());
    // Once that newer receipt has actually been consumed, exact input feedback
    // alone cannot manufacture another observation or another frontier attempt.
    let next_lease = active_child_lease(&handle).await;
    let next_reason = "SafetyFailed cause=StaleInput: no new Surface receipt".to_string();
    owner_watchdog(handle.report_navigation_result(
        &next.child_task_id,
        ChildNavigationResult::Failed {
            proof: terminal_proof(&next_lease),
            reason: next_reason.clone(),
        },
    ))
    .await
    .unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let waiting =
        owner_watchdog(handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)))
            .await;
    assert_eq!(waiting.status, STATUS_PENDING, "{}", waiting.message);
    assert_eq!(sink.submitted.lock().unwrap().len(), 2);
    assert_eq!(
        receiver.freeze().unwrap().notification(),
        latest.notification()
    );
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (3, 3, 0)
        );
        assert!(matches!(&audit.commits[2], ExplorationPoll::Pending));
        assert_eq!(audit.feedback.len(), 2);
        assert_eq!(
            audit.feedback[1],
            (
                surface_owner_goal(&audit.commits[1]).id.clone(),
                ExplorationGoalFeedback::InputUnavailable(next_reason),
            )
        );
        assert_eq!(
            audit.feedback_checkpoints[1].0,
            audit.feedback_checkpoints[1].1
        );
    }
    publish_surface_owner_receipt(&handle, &message.committed_map, 6, snapshot.clone()).await;
    let fresh =
        owner_watchdog(handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)))
            .await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, "owner-surface-receipts.nav.3");
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (4, 4, 0)
        );
        assert_eq!(
            audit.captures[3].original.notification(),
            latest.notification() + 1
        );
        assert!(surface_owner_goal(&audit.commits[3]).id.ends_with("-3"));
    }
    assert_eq!(sink.submitted.lock().unwrap().len(), 3);
    assert!(sink.cancelled.lock().unwrap().is_empty());
    finish_surface_owner_success(&handle, request_id, &fresh, &audit).await;
}

#[tokio::test]
async fn compute_owner_surface_accepted_cancelled_preserves_newer_receipt_for_next_goal() {
    let (handle, sink) = configured_handle();
    let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
    let request_id = "owner-surface-cancelled";
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let commit = Arc::new(OwnerRuntimeGate::default());
    let _prepare_release = OwnerRuntimeGateRelease(prepare.clone());
    let _commit_release = OwnerRuntimeGateRelease(commit.clone());
    let receiver = install_gated_real_surface_source(
        &handle,
        audit.clone(),
        prepare.clone(),
        Some(commit.clone()),
    )
    .await;
    let original = receiver.freeze().unwrap();
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(prepare.entered()).await;
    for revision in 2..=4 {
        publish_surface_owner_receipt(&handle, &message.committed_map, revision, snapshot.clone())
            .await;
        assert!(original.same_source_view(&receiver.freeze().unwrap()));
    }
    let latest = receiver.freeze().unwrap();
    assert_eq!(latest.notification(), original.notification() + 3);
    prepare.release();
    owner_watchdog(commit.entered()).await;
    {
        let audit = audit.lock().unwrap();
        assert_surface_owner_capture(
            &audit.captures[0],
            &snapshot,
            original.notification(),
            &pose,
        );
        assert!(audit.commits.is_empty());
        assert_eq!(audit.discards, 0);
        assert!(audit.feedback.is_empty());
    }
    // Abandon only after Accept, then let the retained owner deliver its exact
    // Cancelled feedback for the undispatched goal. No child terminal is forged.
    tick.abort();
    assert!(owner_watchdog(tick).await.unwrap_err().is_cancelled());
    commit.release();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (1, 1, 0)
        );
        let goal = surface_owner_goal(&audit.commits[0]);
        assert!(goal.id.ends_with("-1"));
        assert_eq!(
            audit.feedback,
            vec![(goal.id.clone(), ExplorationGoalFeedback::Cancelled)]
        );
        assert_eq!(
            audit.feedback_checkpoints[0].0,
            audit.feedback_checkpoints[0].1
        );
    }
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(sink.cancelled.lock().unwrap().is_empty());
    assert!(handle.inner.lock().await.logical_active);
    assert_eq!(handle.inner.lock().await.child_sequence, 0);
    let next =
        owner_watchdog(handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)))
            .await;
    assert_eq!(next.status, STATUS_ACTIVE, "{}", next.message);
    assert_eq!(next.child_task_id, "owner-surface-cancelled.nav.1");
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (2, 2, 0)
        );
        assert_eq!(
            audit.captures[1].original.notification(),
            latest.notification()
        );
        assert!(surface_owner_goal(&audit.commits[1]).id.ends_with("-2"));
        assert_eq!(audit.feedback.len(), 1);
    }
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    finish_surface_owner_success(&handle, request_id, &next, &audit).await;
}

// Complete occupied evidence for this synthetic cloud. This does not claim free
// space, physical motion, or a new sensor observation outside the fixture event.
fn surface_owner_observations(
    message: &schema::NavigationMap3D,
) -> Arc<nuwax_nav_mls_planner::SurfaceObservations> {
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    let mut blocks = std::collections::BTreeMap::<
        [i32; 3],
        std::collections::BTreeMap<[i32; 3], SurfaceObservation>,
    >::new();
    for (x, y, z) in schema::navigation_map_3d_points(message).unwrap() {
        let key = [x, y, z].map(|value| (value / message.source_voxel_size).floor() as i32);
        blocks
            .entry(key.map(|value| value.div_euclid(32)))
            .or_default()
            .insert(
                key,
                SurfaceObservation {
                    observed_ns: message.source_timestamp_ns as u64,
                    hits: 1,
                },
            );
    }
    let mut observations = SurfaceObservations::new(message.source_voxel_size, 32).unwrap();
    for (block, cells) in blocks {
        observations.replace_block(block, cells).unwrap();
    }
    observations.set_source_timestamp_ns(message.source_timestamp_ns as u64);
    observations.mark_complete_geometry_source();
    Arc::new(observations)
}

async fn surface_owner_changed_source_discards_then_progresses(observation_owner_only: bool) {
    let (handle, sink) = configured_handle();
    let (message, _, bare) = install_surface_fixture(&handle, "map").await;
    let snapshot = if observation_owner_only {
        let observed = Arc::new(bare.with_observations(Some(surface_owner_observations(&message))));
        assert!(observed.same_geometry(&bare));
        // Source attachment is published before start, never mutated by a read.
        publish_surface_owner_receipt(&handle, &message.committed_map, 2, observed.clone()).await;
        observed
    } else {
        bare
    };
    let request_id = if observation_owner_only {
        "owner-surface-evidence"
    } else {
        "owner-surface-new-root"
    };
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(prepare.clone());
    let receiver =
        install_gated_real_surface_source(&handle, audit.clone(), prepare.clone(), None).await;
    let before = owner_watchdog(handle.inspect_compute_checkpoint())
        .await
        .unwrap()[0]
        .payload
        .clone();
    let original = receiver.freeze().unwrap();
    let incarnation = handle
        .inner
        .lock()
        .await
        .compute
        .status()
        .unwrap()
        .incarnation;
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(prepare.entered()).await;
    let replacement = if observation_owner_only {
        let observations = snapshot.observations().unwrap();
        let changed =
            Arc::new(snapshot.with_observations(Some(Arc::new(observations.as_ref().clone()))));
        // Exactly equal evidence, a different immutable observation owner. The
        // original geometry Arc is reused; this is not a geometry rebuild.
        assert!(changed.same_geometry(&snapshot));
        assert!(!Arc::ptr_eq(changed.observations().unwrap(), observations));
        changed
    } else {
        let changed = Arc::new(
            SurfaceSnapshot::from_points(
                &schema::navigation_map_3d_points(&message).unwrap(),
                surface_config("map").geometry,
                message.source_voxel_size,
            )
            .unwrap(),
        );
        // Equal numeric points, a genuinely new immutable geometry source.
        assert!(!changed.same_geometry(&snapshot));
        changed
    };
    assert!(!replacement.same_body_query_view(&snapshot));
    publish_surface_owner_receipt(&handle, &message.committed_map, 3, replacement.clone()).await;
    let current = receiver.freeze().unwrap();
    assert!(!original.same_source_view(&current));
    assert_eq!(current.notification(), original.notification() + 1);
    assert!(!tick.is_finished());
    assert!(sink.submitted.lock().unwrap().is_empty());
    {
        let audit = audit.lock().unwrap();
        assert_surface_owner_capture(
            &audit.captures[0],
            &snapshot,
            original.notification(),
            &pose,
        );
        assert!(audit.commits.is_empty());
        assert_eq!(audit.discards, 0);
    }
    prepare.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (1, 0, 1)
        );
        assert!(
            audit.feedback.is_empty(),
            "a discarded proposal never owned a selected goal"
        );
    }
    assert_eq!(
        owner_watchdog(handle.inspect_compute_checkpoint())
            .await
            .unwrap()[0]
            .payload,
        before
    );
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(sink.cancelled.lock().unwrap().is_empty());
    {
        let guard = handle.inner.lock().await;
        assert!(guard.logical_active);
        assert_eq!(guard.child_sequence, 0);
        assert_eq!(guard.reached_goals, 0);
        assert!(guard.active_child.is_none());
        assert!(guard.pending_child_dispatch.is_none());
        assert!(guard.frontiers.is_empty());
    }
    let fresh = owner_watchdog(handle.tick_with_pose(surface_pose(
        &replacement,
        0.0,
        Quaternion::IDENTITY,
    )))
    .await;
    assert_eq!(fresh.status, STATUS_ACTIVE, "{}", fresh.message);
    assert_eq!(fresh.child_task_id, format!("{request_id}.nav.1"));
    {
        let audit = audit.lock().unwrap();
        assert_eq!(
            (audit.captures.len(), audit.commits.len(), audit.discards),
            (2, 1, 1)
        );
        assert_eq!(
            audit.captures[1].original.notification(),
            current.notification()
        );
        assert!(Arc::ptr_eq(
            audit.captures[1].original.snapshot().unwrap(),
            &replacement
        ));
        assert!(
            surface_owner_goal(&audit.commits[0]).id.ends_with("-1"),
            "discard must not consume the Surface ID"
        );
        assert!(audit.feedback.is_empty());
    }
    {
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(submitted[0].intent.committed_map, message.committed_map);
        assert_eq!(
            submitted[0].frontier_goal_id,
            surface_owner_goal(&audit.lock().unwrap().commits[0]).id
        );
    }
    let status = handle.inner.lock().await.compute.status().unwrap();
    assert_eq!(status.incarnation, incarnation);
    assert_eq!(status.high_water.workers, 1);
    assert_eq!(status.high_water.active, 1);
    finish_surface_owner_success(&handle, request_id, &fresh, &audit).await;
}

#[tokio::test]
async fn compute_owner_surface_changed_observation_owner_discards_without_history_or_id() {
    surface_owner_changed_source_discards_then_progresses(true).await;
}

#[tokio::test]
async fn compute_owner_surface_new_immutable_source_discards_without_history_or_id() {
    surface_owner_changed_source_discards_then_progresses(false).await;
}
