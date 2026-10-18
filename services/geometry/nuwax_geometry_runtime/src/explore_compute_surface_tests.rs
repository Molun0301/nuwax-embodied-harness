// Include inside explore_runtime::tests, alongside explore_compute_runtime_tests.rs.
// Uses the existing async-notified native gate, release-on-drop guard and watchdog.
// Every selection, staged delta, commit, discard and feedback below delegates to
// the real SurfaceGoalSourceAdapter. No LCM, network, hardware or clock rewrites.

#[derive(Clone, Default)]
struct SurfaceOwnerFrontierWork(Arc<StdMutex<(usize, usize)>>);

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for SurfaceOwnerFrontierWork {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        if event.metadata().target() != "navigation.surface.frontier.rebuild" {
            return;
        }
        #[derive(Default)]
        struct Cells(Option<usize>);
        impl tracing::field::Visit for Cells {
            fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
                if field.name() == "evaluated_cells" {
                    self.0 = Some(usize::try_from(value).unwrap());
                }
            }
            fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
        }
        let mut cells = Cells::default();
        event.record(&mut cells);
        let mut work = self.0.lock().unwrap();
        work.0 += 1;
        work.1 += cells.0.expect("actual rebuild event must report evaluated_cells");
    }
}

#[derive(Clone, Default)]
struct SurfaceOwnerTomogramWork(Arc<StdMutex<(usize, usize)>>);

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for SurfaceOwnerTomogramWork {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        if event.metadata().target() != "navigation.pct" {
            return;
        }
        #[derive(Default)]
        struct Build { is_map: bool, cells: Option<usize> }
        impl tracing::field::Visit for Build {
            fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
                if field.name() == "event" { self.is_map = value == "navigation.pct.map"; }
            }
            fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
                if field.name() == "cells" { self.cells = Some(usize::try_from(value).unwrap()); }
            }
            fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
        }
        let mut build = Build::default();
        event.record(&mut build);
        if build.is_map {
            let mut work = self.0.lock().unwrap();
            work.0 += 1;
            work.1 += build.cells.expect("actual map construction must report cells");
        }
    }
}

fn capture_surface_owner_work<T>(prepare: impl FnOnce() -> T) -> (T, usize, usize, usize, usize) {
    use tracing_subscriber::prelude::*;
    let work = SurfaceOwnerFrontierWork::default();
    let tomogram = SurfaceOwnerTomogramWork::default();
    // 同步闭包在真实 owner worker 上执行；只观察本次 prepare，无全局 subscriber。
    let result = tracing::subscriber::with_default(
        tracing_subscriber::registry().with(work.clone()).with(tomogram.clone()), prepare);
    let (rebuilds, cells) = *work.0.lock().unwrap();
    let (tomogram_builds, tomogram_cells) = *tomogram.0.lock().unwrap();
    (result, rebuilds, cells, tomogram_builds, tomogram_cells)
}

#[derive(Clone)]
struct SurfaceOwnerCapture {
    original: nuwax_exploration::FrozenSurfaceInput,
    pose: PoseStamped,
    preview: nuwax_exploration::ExploreResult<ExplorationPoll>,
    checkpoint_before: String,
    checkpoint_after: String,
    // Only observe completed work in the real owner; never warm either cache.
    private_frontier_before: Option<nuwax_nav_mls_planner::SurfaceFrontierCacheStats>,
    private_frontier_after: Option<nuwax_nav_mls_planner::SurfaceFrontierCacheStats>,
    source_frontier_before: Option<nuwax_nav_mls_planner::SurfaceFrontierCacheStats>,
    source_frontier_after: Option<nuwax_nav_mls_planner::SurfaceFrontierCacheStats>,
    private_query: std::sync::Weak<SurfaceSnapshot>,
    frontier_rebuilds: usize,
    frontier_evaluated_cells: usize,
    tomogram_builds: usize,
    tomogram_cells: usize,
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
    second_after_prepare: Option<Arc<OwnerRuntimeGate>>,
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
        let query = planning.query_snapshot().expect("private Surface query view");
        let private_frontier_before = query.frontier_cache_stats();
        let source_frontier_before = original.snapshot().unwrap().frontier_cache_stats();
        let checkpoint_before = self.source.checkpoint()?;
        let (prepared, frontier_rebuilds, frontier_evaluated_cells, tomogram_builds, tomogram_cells) =
            capture_surface_owner_work(|| self.source.prepare_poll(state));
        let prepared = prepared?;
        let checkpoint_after = self.source.checkpoint()?;
        let private_frontier_after = query.frontier_cache_stats();
        let source_frontier_after = original.snapshot().unwrap().frontier_cache_stats();
        let private_query = Arc::downgrade(query);
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
                private_frontier_before,
                private_frontier_after,
                source_frontier_before,
                source_frontier_after,
                private_query,
                frontier_rebuilds,
                frontier_evaluated_cells,
                tomogram_builds,
                tomogram_cells,
            });
        // The real source has finished preparing, but no proposal is visible to
        // the actor yet. Each of the at most two test holds is used once.
        if let Some(gate) = self.after_prepare.take() {
            self.after_prepare = self.second_after_prepare.take();
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
    install_gated_real_surface_source_with_second_prepare(
        handle, audit, after_prepare, None, before_commit,
    ).await
}

async fn install_gated_real_surface_source_with_second_prepare(
    handle: &ExploreRuntimeHandle,
    audit: Arc<StdMutex<SurfaceOwnerAudit>>,
    after_prepare: Arc<OwnerRuntimeGate>,
    second_after_prepare: Option<Arc<OwnerRuntimeGate>>,
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
        second_after_prepare,
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
    assert_surface_owner_capture_at_goal(capture, source, notification, pose, 1);
}

fn assert_surface_owner_capture_at_goal(
    capture: &SurfaceOwnerCapture,
    source: &Arc<SurfaceSnapshot>,
    notification: u64,
    pose: &ExploreRobotPose,
    goal_sequence: u64,
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
        .ends_with(&format!("-{goal_sequence}")));
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
    assert!(audit.lock().unwrap().captures.iter()
        .all(|capture| capture.private_query.upgrade().is_none()),
        "stop and owner shutdown must release every retained private query");
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
    let inspection = owner_watchdog(handle.inspect_compute_checkpoint()).await.unwrap();
    assert!(!inspection.is_empty());
    assert!(audit.lock().unwrap().captures[0].private_query.upgrade().is_some(),
        "active checkpoint inspection must not clear the private query");
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
    {
        let audit = audit.lock().unwrap();
        // 首次实际重建必须被观察到；暖统计仅作诊断，不能计成本次工作。
        let source_cells = snapshot.cell_keys().count();
        assert!(source_cells > 0);
        let first_query = &audit.captures[0].private_query;
        let mut rebuilt_cells = 0;
        let tomogram_size = audit.captures[0].tomogram_cells;
        assert!(tomogram_size > 0, "the actual owner must build a nonempty first tomogram");
        let mut built_tomogram_cells = 0;
        for (index, capture) in audit.captures.iter().enumerate() {
            assert_eq!(capture.private_frontier_before.is_none(), index == 0, "capture {index}");
            assert!(capture.private_frontier_after.is_some(), "capture {index}");
            assert!(capture.source_frontier_before.is_none(), "capture {index}");
            assert!(capture.source_frontier_after.is_none(), "capture {index}");
            assert!(std::sync::Weak::ptr_eq(first_query, &capture.private_query));
            assert!(capture.private_query.upgrade().is_some());
            assert_eq!(capture.frontier_rebuilds, usize::from(index == 0), "capture {index}");
            assert_eq!(capture.frontier_evaluated_cells,
                if index == 0 { source_cells } else { 0 }, "capture {index}");
            rebuilt_cells += capture.frontier_evaluated_cells;
            let expected_builds = usize::from(index == 0);
            assert_eq!(capture.tomogram_builds, expected_builds, "capture {index}");
            assert_eq!(capture.tomogram_cells, expected_builds * tomogram_size, "capture {index}");
            built_tomogram_cells += capture.tomogram_cells;
            eprintln!("surface_owner_tomogram_work capture={index} builds={} cells={} first_cells={tomogram_size}",
                capture.tomogram_builds, capture.tomogram_cells);
            match capture.preview.as_ref().unwrap() {
                ExplorationPoll::Goal(_) => assert!(matches!(index, 0 | 1 | 3)),
                ExplorationPoll::Pending => assert_eq!(index, 2),
                other => panic!("unexpected real owner capture {index}: {other:?}"),
            }
            eprintln!("surface_owner_frontier_work capture={index} rebuilds={} evaluated_cells={} source_cells={source_cells}",
                capture.frontier_rebuilds, capture.frontier_evaluated_cells);
        }
        assert_eq!(rebuilt_cells, source_cells);
        assert_eq!(built_tomogram_cells, tomogram_size);
        assert!(snapshot.frontier_cache_stats().is_none());
    }
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
    {
        let audit = audit.lock().unwrap();
        assert_eq!(audit.captures.iter().map(|c| c.frontier_rebuilds).collect::<Vec<_>>(), vec![1, 1]);
        assert_eq!(audit.captures.iter().map(|c| c.tomogram_builds).collect::<Vec<_>>(), vec![1, 1]);
        assert!(audit.captures.iter().all(|c| c.tomogram_cells > 0));
        assert!(audit.captures.iter().all(|c| c.private_frontier_before.is_none()));
        assert!(audit.captures[0].private_query.upgrade().is_none(), "Retire releases the old query");
        assert!(!std::sync::Weak::ptr_eq(&audit.captures[0].private_query, &audit.captures[1].private_query));
    }
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
    observations.mark_complete_geometry_source();
    Arc::new(observations)
}

async fn surface_owner_changed_source_qualifies_query_then_progresses(observation_owner_only: bool) {
    use futures::FutureExt;
    let (handle, sink) = configured_handle();
    let request_id = if observation_owner_only {
        "owner-surface-evidence"
    } else {
        "owner-surface-new-root"
    };
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(prepare.clone());
    let mut ticking: Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>> = None;
    let exercise = std::panic::AssertUnwindSafe(async {
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
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
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
    ticking = Some(tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    }));
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
    assert!(!ticking.as_ref().unwrap().is_finished());
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
    let joined_tick = owner_watchdog(ticking.as_mut().expect("held source-owner tick")).await;
    ticking = None;
    let settled = joined_tick.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    if observation_owner_only {
        // Intentional migration: a new raw owner is still a different source,
        // but the shared derived payload now qualifies this Explore selection.
        assert!(original.same_planning_query_view(&current));
        assert_eq!(settled.status, STATUS_ACTIVE, "{}", settled.message);
        assert_eq!(settled.child_task_id, format!("{request_id}.nav.1"));
        {
            let audit = audit.lock().unwrap();
            assert_eq!((audit.captures.len(), audit.commits.len(), audit.discards), (1, 1, 0));
            assert!(surface_owner_goal(&audit.commits[0]).id.ends_with("-1"));
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
        assert!(sink.cancelled.lock().unwrap().is_empty());
        finish_surface_owner_success(&handle, request_id, &settled, &audit).await;
        return;
    }
    assert!(!original.same_planning_query_view(&current));
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
    {
        let audit = audit.lock().unwrap();
        assert_eq!(audit.captures.iter().map(|c| c.frontier_rebuilds).collect::<Vec<_>>(), vec![1, 1]);
        assert_eq!(audit.captures.iter().map(|c| c.tomogram_builds).collect::<Vec<_>>(), vec![1, 1]);
        assert!(audit.captures.iter().all(|c| c.tomogram_cells > 0));
        for capture in &audit.captures {
            assert!(capture.private_frontier_before.is_none());
            assert_eq!(capture.frontier_evaluated_cells,
                capture.original.snapshot().unwrap().cell_keys().count());
        }
        assert!(audit.captures[0].private_query.upgrade().is_none(), "Discard releases the old query");
        assert!(!std::sync::Weak::ptr_eq(&audit.captures[0].private_query, &audit.captures[1].private_query));
    }
    finish_surface_owner_success(&handle, request_id, &fresh, &audit).await;
    }).catch_unwind().await;
    prepare.release();
    if let Err(panic) = exercise {
        let cleanup = std::panic::AssertUnwindSafe(
            watermark_owner_cleanup(&handle, request_id, &mut ticking)
        ).catch_unwind().await;
        eprintln!("QUERY_EQ migrated source-owner cleanup completed={} business_panicked=true", cleanup.is_ok());
        std::panic::resume_unwind(panic);
    }
}

#[tokio::test]
async fn compute_owner_surface_new_raw_owner_keeps_query_equivalent_selection() {
    surface_owner_changed_source_qualifies_query_then_progresses(true).await;
}

#[tokio::test]
async fn compute_owner_surface_new_immutable_source_discards_without_history_or_id() {
    surface_owner_changed_source_qualifies_query_then_progresses(false).await;
}

// F2 real-input fixtures. Append inside the existing explore_runtime::tests
// inclusion; reuse its actual compute owner, gated Surface source and cleanup.
fn watermark_owner_config() -> SurfaceNavigationConfig {
    let mut config = surface_config("map");
    config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
        z_min_m: -0.05, z_max_m: 0.2, source: "F2 real Writer owner fixture".into(),
    }), None).unwrap();
    assert!(config.body_envelope.is_some());
    config
}

fn watermark_owner_source() -> Arc<nuwax_map_chunks::LayerSource> {
    Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
        sensor_id: "depth-front".into(), clock_id: "camera-clock".into(),
        runtime_generation: 7, session_generation: 1,
    })
}

fn watermark_owner_publication(
    writer: &mut nuwax_map_chunks::Writer,
    previous: Option<&nuwax_map_chunks::Manifest>,
    revision: i32,
    timestamp: u64,
) -> (schema::NavigationMap3D, nuwax_map_chunks::SealedMap) {
    let mut message = surface_message("map", revision);
    message.source_timestamp_ns = timestamp as i64;
    message.committed_map.source_timestamp_ns = timestamp as i64;
    message.cloud.header.stamp_sec = (timestamp / 1_000_000_000) as i32;
    message.cloud.header.stamp_nsec = (timestamp % 1_000_000_000) as i32;
    message.cloud.width = 0;
    message.cloud.row_step = 0;
    message.cloud.data.clear();
    let sealed = writer.seal(schema::navigation_map_3d_domain(&message),
        revision as u64, timestamp).unwrap();
    message.source_voxel_size = sealed.manifest().voxel_size_m;
    message.fine = Some(crate::lcm_runtime::mapping_feed_tests::prepare_fine_transfer_for_watermark_tests(&sealed, previous).unwrap());
    if previous.is_none() {
        assert!(sealed.manifest().blocks.iter().all(|reference|
            message.fine.as_ref().unwrap().included.contains_key(&reference.object)),
            "F2 initial real fixture must fit the original inline transfer budget");
    }
    // Traverse the actual NavigationMap3D wire codec, not a hand-written mock.
    let bytes = schema::encode_navigation_map_3d_body(&message).unwrap();
    let message = schema::decode_navigation_map_3d_body(&bytes).unwrap();
    (message, sealed)
}

async fn watermark_owner_install(
    handle: &ExploreRuntimeHandle,
    cache: &mut nuwax_nav_stack::surface_map::SurfaceMapCache,
    message: &schema::NavigationMap3D,
) -> (Arc<SurfaceSnapshot>, u64) {
    let receipt = now_monotonic_nanos();
    let ticket = owner_watchdog(handle.begin_surface_map_update(message, receipt))
        .await.unwrap().unwrap();
    let snapshot = cache.build(message).unwrap();
    assert!(snapshot.observations().is_some(), "full-body fixture must enable observations");
    assert!(owner_watchdog(handle.finish_surface_map_update(ticket, Ok(snapshot.clone()))).await);
    let guard = handle.inner.lock().await;
    assert_eq!(guard.costmap_received_monotonic_ns, receipt);
    assert_eq!(guard.committed_map.as_ref(), Some(&message.committed_map));
    assert_eq!(guard.surface_mode.as_ref().unwrap().installed_revision,
        Some((message.cloud_revision, message.committed_map.revision)));
    (snapshot, receipt)
}

async fn watermark_owner_cleanup(
    handle: &ExploreRuntimeHandle,
    request_id: &str,
    ticking: &mut Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>>,
) {
    let tick_cleanup = if let Some(mut ticking) = ticking.take() {
        match scheme_a_race_wait("F2 released test waiter", &mut ticking).await {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(error)) => Err(format!("test waiter failed: {error}")),
            Err(error) => {
                // Only the exceptional test waiter is aborted, exactly as the
                // existing Scheme A fixture. Native owner still gets Stop/join.
                ticking.abort();
                let _ = ticking.await;
                Err(error)
            },
        }
    } else { Ok(()) };
    let stopped = scheme_a_race_wait("F2 original Stop", handle.stop(request_id)).await;
    let ticket = scheme_a_race_wait("F2 original cancellation ticket", async {
        handle.inner.lock().await.pending_child_cancellation.as_ref().and_then(|pending| pending.ticket.clone())
    }).await;
    let reported = match ticket.as_ref().ok().and_then(|ticket| ticket.as_ref()) {
        Some(ticket) => Some(scheme_a_race_wait("F2 exact ZEROED", handle.report_navigation_result(
            &ticket.lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&ticket.lease, ticket.cancel_generation)),
        )).await),
        None => None,
    };
    let idle = scheme_a_race_wait("F2 original compute idle", handle.await_compute_idle()).await;
    let shutdown = scheme_a_race_wait("F2 original compute shutdown", handle.request_compute_shutdown()).await;
    let joined = scheme_a_race_wait("F2 original compute join", handle.compute_service.join()).await;
    let retired = scheme_a_race_wait("F2 settled child ownership", async {
        let guard = handle.inner.lock().await;
        (guard.active_child.is_some(), guard.pending_child_cancellation.is_some(), guard.logical_active)
    }).await;
    eprintln!("F2 cleanup: tick={tick_cleanup:?}, stopped={stopped:?}, ticket={ticket:?}, reported={reported:?}, idle={idle:?}, shutdown={shutdown:?}, joined={joined:?}, retired={retired:?}");
    assert!(tick_cleanup.is_ok());
    assert!(stopped.is_ok());
    assert!(ticket.is_ok());
    assert!(matches!(reported, None | Some(Ok(Ok(_)))));
    assert!(matches!(idle, Ok(Ok(()))));
    assert!(shutdown.is_ok());
    assert!(matches!(joined, Ok(Ok(()))));
    assert!(matches!(retired, Ok((false, false, false))));
}

async fn watermark_owner_real_publication_qualifies_prepared_selection(evidence_change: bool) {
    use futures::FutureExt;
    let (handle, sink) = configured_handle();
    let request_id = if evidence_change { "owner-real-writer-evidence" } else { "owner-real-writer-watermark" };
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    let second_gate = evidence_change.then(|| Arc::new(OwnerRuntimeGate::default()));
    let _second_release = second_gate.as_ref().map(|gate| OwnerRuntimeGateRelease(gate.clone()));
    let mut ticking: Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>> = None;
    let exercise = std::panic::AssertUnwindSafe(async {
    let config = watermark_owner_config();
    handle.configure_surface_source(Ok(config.clone()), 37, [7; 16]).await.unwrap();
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository.clone(), 0.1, 4 * 1024 * 1024).unwrap();
    let first_time = now_unix_nanos();
    writer.observe([0, 30].into_iter().flat_map(|z| (0..50).flat_map(move |x|
        (0..30).map(move |y| [x, y, z]))), first_time).unwrap();
    let source = watermark_owner_source();
    writer.record_source_watermark(source.clone(), 90, 0).unwrap();
    let (first_message, first_sealed) = watermark_owner_publication(&mut writer, None, 1, first_time);
    let first_root = first_sealed.manifest().clone();
    let first_bytes: std::collections::BTreeMap<_, _> = first_root.blocks.iter().map(|reference|
        (reference.object, repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
    let (first, first_receipt) = watermark_owner_install(&handle, &mut cache, &first_message).await;
    assert_eq!(handle.start(request_id, "prefer_wavefront").await.status, STATUS_ACCEPTED);
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let receiver = install_gated_real_surface_source_with_second_prepare(
        &handle, audit.clone(), gate.clone(), second_gate.clone(), None,
    ).await;
    let original = receiver.freeze().unwrap();
    let planning = original.fork_for_planning();
    let original_evidence = first.observations().unwrap().get([15, 15, 0]).unwrap();
    let pose = surface_pose(&first, 0.0, Quaternion::IDENTITY);
    ticking = Some(tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    }));
    owner_watchdog(gate.entered()).await;
    let (prepared_at_gate, committed_at_gate, discarded_at_gate) = {
        let audit = audit.lock().unwrap();
        (audit.captures.clone(), audit.commits.len(), audit.discards)
    };
    assert_eq!(prepared_at_gate.len(), 1, "F2 fixture must reach one real prepare");
    eprintln!("F2 before-Accept qualification: preview={:?}, receipt_age_ns={}",
        prepared_at_gate[0].preview, now_monotonic_nanos().saturating_sub(first_receipt));
    let _ = surface_owner_goal(prepared_at_gate[0].preview.as_ref().unwrap());
    assert_eq!((committed_at_gate, discarded_at_gate), (0, 0));
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert!(!ticking.as_ref().unwrap().is_finished());

    let second_time = now_unix_nanos().max(first_time + 1);
    writer.record_source_watermark(source.clone(), 900, 1).unwrap();
    if evidence_change {
        writer.observe([[15, 15, 0]], second_time).unwrap();
    }
    let (second_message, second_sealed) = watermark_owner_publication(&mut writer,
        Some(first_root.as_ref()), 2, second_time);
    let second_root = second_sealed.manifest();
    assert_eq!(first_root.domain, second_root.domain);
    assert_eq!(first_root.voxel_size_m.to_bits(), second_root.voxel_size_m.to_bits());
    assert_eq!(first_root.geometry_revision, second_root.geometry_revision);
    assert_ne!(first_root.source_watermarks, second_root.source_watermarks);
    assert!(second_root.revision > first_root.revision);
    assert!(second_root.source_timestamp_ns > first_root.source_timestamp_ns);
    assert_eq!(first_message.virtual_obstacles, second_message.virtual_obstacles);
    if evidence_change {
        assert_ne!(first_root.blocks, second_root.blocks);
        assert!(!second_message.fine.as_ref().unwrap().included.is_empty());
    } else {
        // Complete BlockRef equality is an input proof for this test only. The
        // production candidate must not add another whole-root comparison.
        assert_eq!(first_root.blocks, second_root.blocks);
        assert!(second_message.fine.as_ref().unwrap().included.is_empty());
        for reference in &second_root.blocks {
            assert_eq!(first_bytes[&reference.object], repository
                .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap());
        }
    }
    let (second, second_receipt) = watermark_owner_install(&handle, &mut cache, &second_message).await;
    let current = receiver.freeze().unwrap();
    assert!(second_receipt >= first_receipt);
    assert_eq!(current.notification(), original.notification() + 1);
    assert!(first.same_geometry(&second));
    assert!(first.observations().unwrap().same_geometry(second.observations().unwrap()));
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    let new_evidence = second.observations().unwrap().get([15, 15, 0]).unwrap();
    if evidence_change {
        assert!(new_evidence.observed_ns > original_evidence.observed_ns);
        assert!(new_evidence.hits > original_evidence.hits);
    } else {
        assert_eq!(original_evidence, new_evidence);
    }
    // Capture red facts, then settle the worker before asserting. A red test
    // must never strand a prepared native owner or leave an accepted child.
    let observations_reused = Arc::ptr_eq(first.observations().unwrap(), second.observations().unwrap());
    let same_view = original.same_source_view(&current);
    let rebound = planning.rebind_original(&current);
    if let Some(rebound) = rebound.as_ref() {
        assert_eq!(rebound.original_input().notification(), current.notification());
        assert!(Arc::ptr_eq(rebound.query_snapshot().unwrap(), planning.query_snapshot().unwrap()));
    }
    gate.release();
    let joined_tick = owner_watchdog(ticking.as_mut().expect("real gated tick")).await;
    ticking = None;
    let result = joined_tick.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let (captures, commits, discards) = {
        let audit = audit.lock().unwrap();
        (audit.captures.clone(), audit.commits.clone(), audit.discards)
    };
    let submitted = sink.submitted.lock().unwrap().clone();
    let cancelled = sink.cancelled.lock().unwrap().clone();
    // Intentional contract migration: truthful raw evidence still changes the
    // source, but no derived query field changed before this actual Accept.
    if evidence_change { assert!(!observations_reused && !same_view); }
    else { assert!(observations_reused && same_view); }
    assert!(original.same_planning_query_view(&current));
    assert!(rebound.is_some());
    assert_eq!((captures.len(), commits.len(), discards), (1, 1, 0));
    assert_eq!(submitted.len(), 1);
    assert!(cancelled.is_empty());
    assert_surface_owner_capture(&captures[0], &first, original.notification(), &pose);
    // Exercise the actual owner's next selection too. InputUnavailable retires
    // the real child without blacklisting the frontier or consuming the newer
    // map notification; this is the same terminal-proof flow as the existing
    // same-root owner reuse tests. The first proposal now owns its real child.
    if result.status == STATUS_ACTIVE && !submitted.is_empty() {
        let lease = active_child_lease(&handle).await;
        owner_watchdog(handle.report_navigation_result(&result.child_task_id,
            ChildNavigationResult::Failed {
                proof: terminal_proof(&lease),
                reason: "SafetyFailed cause=StaleInput: F2 actual owner reuse fixture".into(),
            })).await.unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
    }
    let second_pose = surface_pose(&second, 0.0, Quaternion::IDENTITY);
    let mut renewed_committed_map = None;
    let next_result = if let Some(second_gate) = second_gate.as_ref() {
        ticking = Some(tokio::spawn({
            let handle = handle.clone();
            let pose = second_pose.clone();
            async move { handle.tick_with_pose(pose).await }
        }));
        owner_watchdog(second_gate.entered()).await;
        // Capture completed real work on root2 before publishing root3. Neither
        // the source nor its private query is replaced or warmed by this check.
        let (warm_captures, warm_commits, warm_discards) = {
            let audit = audit.lock().unwrap();
            (audit.captures.clone(), audit.commits.len(), audit.discards)
        };
        {
            assert_eq!((warm_captures.len(), warm_commits, warm_discards), (2, 1, 0));
            let warm = &warm_captures[1];
            assert_surface_owner_capture_at_goal(warm, &second, current.notification(), &second_pose, 2);
            assert!(std::sync::Weak::ptr_eq(&warm_captures[0].private_query, &warm.private_query));
            assert!(warm.private_frontier_before.is_some());
            assert_eq!(second.cell_keys().count(), 3000, "unchanged real Writer fixture size");
            assert_eq!((warm.tomogram_builds, warm.tomogram_cells), (0, 0));
            assert_eq!((warm.frontier_rebuilds, warm.frontier_evaluated_cells), (0, 0));
        }
        assert!(!ticking.as_ref().unwrap().is_finished());
        {
            let submitted_at_gate = sink.submitted.lock().unwrap();
            assert_eq!(submitted_at_gate.len(), 1);
            assert_eq!(submitted_at_gate[0].intent.committed_map, first_message.committed_map);
            assert_eq!(submitted_at_gate[0].frontier_goal_id,
                surface_owner_goal(&commits[0]).id);
        }
        assert!(sink.cancelled.lock().unwrap().is_empty());
        let second_bytes: std::collections::BTreeMap<_, _> = second_root.blocks.iter().map(|reference|
            (reference.object, repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap())).collect();
        let third_time = now_unix_nanos().max(second_time + 1);
        writer.record_source_watermark(source, 901, 2).unwrap();
        let (third_message, third_sealed) = watermark_owner_publication(&mut writer,
            Some(second_root.as_ref()), 3, third_time);
        let third_root = third_sealed.manifest();
        assert_eq!(second_root.domain, third_root.domain);
        assert_eq!(second_root.voxel_size_m.to_bits(), third_root.voxel_size_m.to_bits());
        assert_eq!(second_root.geometry_revision, third_root.geometry_revision);
        assert_eq!(second_root.blocks, third_root.blocks);
        assert_ne!(second_root.source_watermarks, third_root.source_watermarks);
        assert!(third_root.revision > second_root.revision);
        assert!(third_root.source_timestamp_ns > second_root.source_timestamp_ns);
        assert_eq!(second_message.virtual_obstacles, third_message.virtual_obstacles);
        assert!(third_message.fine.as_ref().unwrap().included.is_empty());
        for reference in &third_root.blocks {
            assert_eq!(second_bytes[&reference.object], repository
                .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap());
        }
        // Use the original Writer/transfer/codec/cache/receiver completion path.
        // This is a new real receipt, never a clock or field assignment in test.
        let (third, third_receipt) = watermark_owner_install(&handle, &mut cache, &third_message).await;
        let renewed = receiver.freeze().unwrap();
        assert!(third_receipt > second_receipt);
        assert_eq!(renewed.notification(), current.notification() + 1);
        assert!(second.same_geometry(&third));
        assert!(Arc::ptr_eq(second.observations().unwrap(), third.observations().unwrap()));
        let renewed_evidence = third.observations().unwrap().get([15, 15, 0]).unwrap();
        assert_eq!(new_evidence, renewed_evidence);
        assert!(renewed_evidence.observed_ns > original_evidence.observed_ns);
        assert!(renewed_evidence.hits > original_evidence.hits);
        assert!(current.same_source_view(&renewed));
        assert!(!Arc::ptr_eq(first.observations().unwrap(), third.observations().unwrap()));
        assert!(!original.same_source_view(&renewed));
        assert!(original.same_planning_query_view(&renewed));
        assert!(planning.rebind_original(&renewed).is_some());
        assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
        renewed_committed_map = Some(third_message.committed_map.clone());
        eprintln!("F2 evidence pure-watermark renewal: root2={}, root3={}, original_receipt_ns={second_receipt}, renewed_receipt_ns={third_receipt}, notification={}",
            second_root.revision, third_root.revision, renewed.notification());
        second_gate.release();
        let joined_tick = owner_watchdog(ticking.as_mut().expect("second real gated tick")).await;
        ticking = None;
        joined_tick.unwrap()
    } else {
        owner_watchdog(handle.tick_with_pose(second_pose)).await
    };
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let later_captures = audit.lock().unwrap().captures.clone();
    eprintln!("F2 actual owner: evidence_change={evidence_change}, observations_reused={observations_reused}, same_view={same_view}, rebound={}, first_status={}, captures={}, commits={}, discards={discards}, first_submitted={}, first_cancelled={}, next_status={}, later_captures={}",
        rebound.is_some(), result.status, captures.len(), commits.len(), submitted.len(), cancelled.len(), next_result.status, later_captures.len());
    assert_surface_owner_capture(&captures[0], &first, original.notification(), &pose);
    assert_eq!(next_result.status, STATUS_ACTIVE, "{}", next_result.message);
    assert_eq!(later_captures.len(), 2);
    assert_eq!(later_captures[1].original.notification(), current.notification());
    assert_eq!(later_captures[0].tomogram_builds, 1);
    assert!(later_captures[0].tomogram_cells > 0);
    if evidence_change { assert!(!observations_reused && !same_view); }
    else { assert!(observations_reused && same_view); }
    assert!(rebound.is_some());
    assert!(std::sync::Weak::ptr_eq(&later_captures[0].private_query,
        &later_captures[1].private_query));
    assert_eq!((later_captures[1].tomogram_builds, later_captures[1].tomogram_cells), (0, 0));
    assert_eq!((later_captures[1].frontier_rebuilds, later_captures[1].frontier_evaluated_cells), (0, 0));
    assert_eq!((captures.len(), commits.len(), discards), (1, 1, 0), "{}", result.message);
    assert_eq!(result.status, STATUS_ACTIVE, "{}", result.message);
    assert_eq!(submitted.len(), 1);
    assert!(cancelled.is_empty());
    assert_eq!(submitted[0].intent.committed_map, first_message.committed_map,
        "accepted goal must retain the actual frozen source identity");
    assert_ne!(submitted[0].intent.committed_map, second_message.committed_map);
    if evidence_change {
        {
            let audit = audit.lock().unwrap();
            assert_eq!((audit.commits.len(), audit.discards), (2, 0));
        }
        let final_submitted = sink.submitted.lock().unwrap();
        assert_eq!(final_submitted.len(), 2);
        assert_eq!(final_submitted[1].intent.committed_map, second_message.committed_map);
        if let Some(renewed) = renewed_committed_map.as_ref() {
            assert_ne!(&final_submitted[1].intent.committed_map, renewed);
        }
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }
    }).catch_unwind().await;
    // Release every possible native hold before waiting for the test waiter,
    // then attempt the original Stop/idle/shutdown/join even after a panic.
    gate.release();
    if let Some(gate) = second_gate.as_ref() { gate.release(); }
    let cleanup = std::panic::AssertUnwindSafe(
        watermark_owner_cleanup(&handle, request_id, &mut ticking)
    ).catch_unwind().await;
    eprintln!("QUERY_EQ migrated writer cleanup completed={} business_panicked={}", cleanup.is_ok(), exercise.is_err());
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
    if let Err(panic) = cleanup { std::panic::resume_unwind(panic); }
}

#[tokio::test]
async fn compute_owner_real_writer_watermark_keeps_prepared_source_view() {
    watermark_owner_real_publication_qualifies_prepared_selection(false).await;
}

#[tokio::test]
async fn compute_owner_real_writer_evidence_change_keeps_prepared_query_view() {
    // New Explore-only contract: real evidence changes the raw source but keeps
    // query-equivalent selection and its warm private owner across later receipts.
    watermark_owner_real_publication_qualifies_prepared_selection(true).await;
}

#[tokio::test]
async fn real_writer_same_plane_scatter_change_invalidates_complete_source_view() {
    let config = watermark_owner_config();
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository.clone(), 0.1, 4 * 1024 * 1024).unwrap();
    let source = watermark_owner_source();
    let timestamp = now_unix_nanos();
    writer.observe((0..50).flat_map(|x| (0..30).map(move |y| [x, y, 0])), timestamp).unwrap();
    let layer_key = [15, 15, 0];
    let anchor = nuwax_map_chunks::local_key(layer_key);
    let block_key = nuwax_map_chunks::block_key(layer_key);
    let layers = |spread: f64| std::collections::BTreeMap::from([(anchor,
        nuwax_map_chunks::LayerObservation {
            members: vec![anchor], plane_origin: [1.55, 1.55, 0.025], normal: [0.0, 0.0, 1.0],
            samples: [0.025 - spread, 0.025 + spread].into_iter().enumerate().map(|(index, z)|
                nuwax_map_chunks::LayerSample { source: source.clone(), point: [1.55, 1.55, z],
                    source_sequence: 1, source_timestamp_ns: timestamp, point_index: index as u32 }).collect(),
        })]);
    writer.replace_layer_observations(block_key, layers(0.001)).unwrap();
    writer.record_source_watermark(source.clone(), timestamp, 1).unwrap();
    let (first_message, first_sealed) = watermark_owner_publication(&mut writer, None, 1, timestamp);
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let first = cache.build(&first_message).unwrap();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(first.clone()).unwrap();
    let original = receiver.freeze().unwrap();
    let planning = original.fork_for_planning();
    let old_evidence = first.observations().unwrap().get(layer_key);
    let old_layer = first.observations().unwrap().ground_layers([15, 15]).next().unwrap();
    let old_scatter = old_layer.statistics.moments().sample_scatter().unwrap();
    let old_query = first.query_ground([1.55, 1.55], 0.025);
    // Same occupied members, plane, normal, sample identity and mean. Only the
    // truthful sample spread changes; no observe() call advances voxel evidence.
    writer.replace_layer_observations(block_key, layers(0.01)).unwrap();
    let (second_message, second_sealed) = watermark_owner_publication(&mut writer,
        Some(first_sealed.manifest().as_ref()), 2, timestamp + 1);
    let first_root = first_sealed.manifest();
    let second_root = second_sealed.manifest();
    assert_eq!(first_root.geometry_revision, second_root.geometry_revision);
    assert_eq!(first_root.source_watermarks, second_root.source_watermarks);
    assert_ne!(first_root.blocks, second_root.blocks);
    let before = first_root.blocks.iter().find(|reference| reference.key == block_key).unwrap();
    let after = second_root.blocks.iter().find(|reference| reference.key == block_key).unwrap();
    assert!(after.block_revision > before.block_revision);
    assert_eq!(after.geometry_revision, before.geometry_revision);
    assert_eq!(after.points, before.points);
    let before_bytes = repository.read_object(before.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
    let after_bytes = repository.read_object(after.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
    assert_ne!(before_bytes, after_bytes);
    let second = cache.build(&second_message).unwrap();
    let new_layer = second.observations().unwrap().ground_layers([15, 15]).next().unwrap();
    let new_scatter = new_layer.statistics.moments().sample_scatter().unwrap();
    let new_query = second.query_ground([1.55, 1.55], 0.025);
    assert_eq!(old_evidence, second.observations().unwrap().get(layer_key));
    assert_eq!(old_layer.members, new_layer.members);
    assert_eq!(old_layer.plane_origin, new_layer.plane_origin);
    assert_eq!(old_layer.normal, new_layer.normal);
    assert!((old_layer.statistics.moments().mean()[2] - new_layer.statistics.moments().mean()[2]).abs() < 1e-12);
    assert!(new_scatter[5] > old_scatter[5]);
    assert_eq!(old_query.height, new_query.height);
    assert!(new_query.max_residual_m.unwrap() > old_query.max_residual_m.unwrap());
    assert!(first.same_geometry(&second));
    assert!(first.observations().unwrap().same_geometry(second.observations().unwrap()));
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    assert!(!Arc::ptr_eq(first.observations().unwrap(), second.observations().unwrap()));
    receiver.observe(second).unwrap();
    let current = receiver.freeze().unwrap();
    assert!(!original.same_source_view(&current));
    assert!(!original.same_planning_query_view(&current));
    assert!(planning.rebind_original(&current).is_none());
}

// DRAFT ONLY: append inside the existing explore_runtime::tests inclusion via
// explore_compute_surface_tests.rs. Reuses the real Writer, wire codec, cache,
// compute owner, after_prepare gate and existing Stop/ZEROED/shutdown cleanup.
// This intentionally specifies a NEW Explore query-equivalence contract.
// It is not a replacement explanation of the F2 evidence-change negative test.

#[tokio::test]
async fn compute_owner_repeated_writer_evidence_keeps_query_equivalent_selection() {
    use futures::FutureExt;
    let (handle, sink) = configured_handle();
    let request_id = "owner-query-equivalent-evidence";
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    let mut ticking: Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>> = None;
    let exercise = std::panic::AssertUnwindSafe(async {
        let config = watermark_owner_config();
        handle.configure_surface_source(Ok(config.clone()), 37, [7; 16]).await.unwrap();
        let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
        let repository = nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer = nuwax_map_chunks::Writer::new(repository, 0.1, 4 * 1024 * 1024).unwrap();
        let first_time = now_unix_nanos();
        writer.observe([0, 30].into_iter().flat_map(|z| (0..50).flat_map(move |x|
            (0..30).map(move |y| [x, y, z]))), first_time).unwrap();
        let source = watermark_owner_source();
        writer.record_source_watermark(source.clone(), first_time, 1).unwrap();
        let (first_message, first_sealed) = watermark_owner_publication(&mut writer, None, 1, first_time);
        let (first, mut latest_receipt) = watermark_owner_install(&handle, &mut cache, &first_message).await;
        assert_eq!(handle.start(request_id, "prefer_wavefront").await.status, STATUS_ACCEPTED);
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let receiver = install_gated_real_surface_source_with_second_prepare(
            &handle, audit.clone(), gate.clone(), None, None,
        ).await;
        let original = receiver.freeze().unwrap();
        let planning = original.fork_for_planning();
        let old_evidence = first.observations().unwrap().get([15, 15, 0]).unwrap();
        let pose = surface_pose(&first, 0.0, Quaternion::IDENTITY);
        ticking = Some(tokio::spawn({
            let handle = handle.clone();
            let pose = pose.clone();
            async move { handle.tick_with_pose(pose).await }
        }));
        owner_watchdog(gate.entered()).await;
        {
            let observed = audit.lock().unwrap();
            assert_eq!((observed.captures.len(), observed.commits.len(), observed.discards), (1, 0, 0));
            let _ = surface_owner_goal(observed.captures[0].preview.as_ref().unwrap());
        }
        assert!(sink.submitted.lock().unwrap().is_empty());

        // Several truthful updates land while one REAL prepared selection is held.
        // No geometry/obstacle/plane/config/pose is changed and no TTL is lengthened.
        let mut previous = first_sealed.manifest().clone();
        let mut latest = first.clone();
        let mut newest_message = first_message.clone();
        let mut current = original.clone();
        let mut timestamp = first_time;
        for revision in 2..=4 {
            timestamp = now_unix_nanos().max(timestamp + 1);
            writer.observe([[15, 15, 0]], timestamp).unwrap();
            writer.record_source_watermark(source.clone(), timestamp, revision as u64).unwrap();
            let (message, sealed) = watermark_owner_publication(&mut writer,
                Some(previous.as_ref()), revision, timestamp);
            let root = sealed.manifest();
            assert_eq!(root.domain, previous.domain);
            assert_eq!(root.geometry_revision, previous.geometry_revision);
            assert_ne!(root.blocks, previous.blocks);
            assert!(!message.fine.as_ref().unwrap().included.is_empty());
            let (snapshot, receipt) = watermark_owner_install(&handle, &mut cache, &message).await;
            latest_receipt = receipt;
            let captured = receiver.freeze().unwrap();
            assert_eq!(captured.notification(), original.notification() + (revision - 1) as u64);
            assert!(first.same_geometry(&snapshot));
            assert!(first.observations().unwrap().same_geometry(snapshot.observations().unwrap()));
            assert!(!Arc::ptr_eq(first.observations().unwrap(), snapshot.observations().unwrap()));
            assert!(!original.same_source_view(&captured), "raw source equality stays strict");
            let evidence = snapshot.observations().unwrap().get([15, 15, 0]).unwrap();
            assert_eq!(evidence.observed_ns, timestamp);
            assert_eq!(evidence.hits, old_evidence.hits + (revision - 1) as u16);
            assert_eq!(first.observations().unwrap().get([15, 15, 0]).unwrap(), old_evidence);
            assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
            previous = root.clone();
            latest = snapshot;
            newest_message = message;
            current = captured;
        }
        let rebound = planning.rebind_original(&current);
        gate.release();
        let joined = owner_watchdog(ticking.as_mut().unwrap()).await;
        ticking = None;
        let result = joined.unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        // A business RED requires the original receipt TTL to remain valid.
        // Nothing after the last real installation renews or edits this receipt.
        let (receipt, ttl) = {
            let guard = handle.inner.lock().await;
            (guard.costmap_received_monotonic_ns, guard.costmap_receipt_ttl)
        };
        let receipt_age_ns = now_monotonic_nanos().saturating_sub(receipt);
        eprintln!("QUERY_EQ receipt qualification: receipt={receipt} age_ns={receipt_age_ns} ttl_ns={}", ttl.as_nanos());
        assert_eq!(receipt, latest_receipt);
        assert!(receipt != 0 && u128::from(receipt_age_ns) <= ttl.as_nanos(),
            "fixture receipt expired before business classification");
        let (captures, commits, discards) = {
            let observed = audit.lock().unwrap();
            (observed.captures.clone(), observed.commits.clone(), observed.discards)
        };
        eprintln!("QUERY_EQ actual owner: captures={} commits={} discards={} status={} message={} rebound={}",
            captures.len(), commits.len(), discards, result.status, result.message, rebound.is_some());
        assert_surface_owner_capture(&captures[0], &first, original.notification(), &pose);
        // First NEW-contract business assertion. On current HEAD, the actual
        // settled counts should be (1,0,1), rather than a manufactured Arc RED.
        assert_eq!((captures.len(), commits.len(), discards), (1, 1, 0),
            "query-equivalent raw evidence must not discard a prepared selection: {}", result.message);
        assert_eq!(result.status, STATUS_ACTIVE, "{}", result.message);
        {
            let submitted = sink.submitted.lock().unwrap();
            assert_eq!(submitted.len(), 1);
            assert_eq!(submitted[0].intent.committed_map, first_message.committed_map);
            assert_ne!(submitted[0].intent.committed_map, newest_message.committed_map);
        }
        assert!(sink.cancelled.lock().unwrap().is_empty());
        let rebound = rebound.expect("new contract must preserve qualified private query");
        assert_eq!(rebound.original_input().notification(), current.notification());
        assert!(Arc::ptr_eq(rebound.original_input().snapshot().unwrap(), &latest));
        assert!(Arc::ptr_eq(rebound.query_snapshot().unwrap(), planning.query_snapshot().unwrap()));
        assert_eq!(rebound.query_snapshot().unwrap().observations().unwrap().get([15, 15, 0]), Some(old_evidence),
            "private query keeps its immutable raw source; only query semantics are equivalent");
        assert_eq!(rebound.original_input().snapshot().unwrap().observations().unwrap().get([15, 15, 0]),
            latest.observations().unwrap().get([15, 15, 0]), "original exposes the newest evidence");

        // Reuse must also happen inside the actual backend on its next request,
        // after exact feedback. The production source owns notification/history.
        let lease = active_child_lease(&handle).await;
        owner_watchdog(handle.report_navigation_result(&result.child_task_id,
            ChildNavigationResult::Failed {
                proof: terminal_proof(&lease),
                reason: "SafetyFailed cause=StaleInput: query equivalence fixture".into(),
            })).await.unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        let next_pose = surface_pose(&latest, 0.0, Quaternion::IDENTITY);
        let next = owner_watchdog(handle.tick_with_pose(next_pose.clone())).await;
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        let observed = audit.lock().unwrap();
        assert_eq!(next.status, STATUS_ACTIVE, "{}", next.message);
        assert_eq!(observed.captures.len(), 2);
        assert_surface_owner_capture_at_goal(&observed.captures[1], &latest, current.notification(), &next_pose, 2);
        let actual_private = observed.captures[1].private_query.upgrade().unwrap();
        assert_eq!(actual_private.observations().unwrap().get([15, 15, 0]), Some(old_evidence),
            "the actual backend retains only its old immutable query raw evidence");
        assert_eq!(observed.captures[1].original.snapshot().unwrap().observations().unwrap().get([15, 15, 0]),
            latest.observations().unwrap().get([15, 15, 0]), "actual captured original stays current");
        assert!(std::sync::Weak::ptr_eq(&observed.captures[0].private_query,
            &observed.captures[1].private_query));
        assert_eq!((observed.captures[1].tomogram_builds, observed.captures[1].tomogram_cells), (0, 0));
        assert_eq!((observed.captures[1].frontier_rebuilds, observed.captures[1].frontier_evaluated_cells), (0, 0));
        assert_eq!((observed.commits.len(), observed.discards), (2, 0));
    }).catch_unwind().await;
    gate.release();
    let cleanup = std::panic::AssertUnwindSafe(
        watermark_owner_cleanup(&handle, request_id, &mut ticking),
    ).catch_unwind().await;
    eprintln!("QUERY_EQ cleanup completed={} business_panicked={}", cleanup.is_ok(), exercise.is_err());
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
    if let Err(panic) = cleanup { std::panic::resume_unwind(panic); }
}

#[tokio::test]
async fn real_writer_measured_metadata_keeps_private_query_and_new_raw_evidence() {
    let config = watermark_owner_config();
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository.clone(), 0.1, 4 * 1024 * 1024).unwrap();
    let source = watermark_owner_source();
    let key = [15, 15, 0];
    let block = nuwax_map_chunks::block_key(key);
    let anchor = nuwax_map_chunks::local_key(key);
    let first_time = now_unix_nanos();
    writer.observe((0..50).flat_map(|x| (0..30).map(move |y| [x, y, 0])), first_time).unwrap();
    let layers = |timestamp, sequence| std::collections::BTreeMap::from([(anchor,
        nuwax_map_chunks::LayerObservation {
            members: vec![anchor], plane_origin: [1.55, 1.55, 0.025], normal: [0.0, 0.0, 1.0],
            samples: [0.024, 0.026].into_iter().enumerate().map(|(index, z)|
                nuwax_map_chunks::LayerSample { source: source.clone(), point: [1.55, 1.55, z],
                    source_sequence: sequence, source_timestamp_ns: timestamp, point_index: index as u32 }).collect(),
        })]);
    let original_layers = layers(first_time, 1);
    writer.replace_layer_observations(block, original_layers.clone()).unwrap();
    writer.record_source_watermark(source.clone(), first_time, 1).unwrap();
    let (first_message, first_sealed) = watermark_owner_publication(&mut writer, None, 1, first_time);
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let first = cache.build(&first_message).unwrap();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(first.clone()).unwrap();
    let original = receiver.freeze().unwrap();
    let planning = original.fork_for_planning();
    let old_layer = first.observations().unwrap().ground_layers([15, 15]).next().unwrap();
    let old_query = planning.query_snapshot().unwrap().query_ground([1.55, 1.55], 0.025);
    let old_evidence = first.observations().unwrap().get(key).unwrap();
    let mut previous = first_sealed.manifest().clone();
    let mut timestamp = first_time;
    let mut rebound_results = Vec::new();
    for (revision, renew_layer_samples) in [(2, false), (3, true)] {
        timestamp = now_unix_nanos().max(timestamp + 1);
        writer.observe([key], timestamp).unwrap();
        let expected_layers = if renew_layer_samples {
            let renewed = layers(timestamp, revision as u64);
            writer.replace_layer_observations(block, renewed.clone()).unwrap();
            renewed
        } else { original_layers.clone() };
        writer.record_source_watermark(source.clone(), timestamp, revision as u64).unwrap();
        let (message, sealed) = watermark_owner_publication(&mut writer,
            Some(previous.as_ref()), revision, timestamp);
        assert_eq!(previous.geometry_revision, sealed.manifest().geometry_revision);
        assert_ne!(previous.blocks, sealed.manifest().blocks);
        let reference = sealed.manifest().blocks.iter().find(|reference| reference.key == block).unwrap();
        let bytes = repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
        assert_eq!(nuwax_map_chunks::decode_layer_observations(&bytes).unwrap(), expected_layers,
            "raw layer sample identity/time remains truthful in the actual persisted object");
        let current_snapshot = cache.build(&message).unwrap();
        let current_layer = current_snapshot.observations().unwrap().ground_layers([15, 15]).next().unwrap();
        assert_eq!(old_layer.anchor, current_layer.anchor);
        assert_eq!(old_layer.members, current_layer.members);
        assert_eq!(old_layer.normal, current_layer.normal);
        assert_eq!(old_layer.plane_origin, current_layer.plane_origin);
        assert_eq!(old_layer.statistics, current_layer.statistics);
        assert!(first.same_geometry(&current_snapshot));
        assert!(!Arc::ptr_eq(first.observations().unwrap(), current_snapshot.observations().unwrap()));
        let new_evidence = current_snapshot.observations().unwrap().get(key).unwrap();
        assert_eq!(new_evidence.observed_ns, timestamp);
        assert_eq!(new_evidence.hits, old_evidence.hits + (revision - 1) as u16);
        let new_query = current_snapshot.query_ground([1.55, 1.55], 0.025);
        assert_eq!(old_query.state, new_query.state);
        assert_eq!(old_query.height, new_query.height);
        assert_eq!(old_query.normal, new_query.normal);
        assert_eq!(old_query.sources, new_query.sources);
        assert_eq!(old_query.radius_m, new_query.radius_m);
        assert_eq!(old_query.max_residual_m, new_query.max_residual_m);
        receiver.observe(current_snapshot.clone()).unwrap();
        let current = receiver.freeze().unwrap();
        assert!(!original.same_source_view(&current));
        assert_eq!(current.notification(), original.notification() + (revision - 1) as u64);
        let rebound = planning.rebind_original(&current);
        if let Some(rebound) = rebound.as_ref() {
            assert!(Arc::ptr_eq(rebound.original_input().snapshot().unwrap(), &current_snapshot));
            assert!(Arc::ptr_eq(rebound.query_snapshot().unwrap(), planning.query_snapshot().unwrap()));
            assert_eq!(rebound.original_input().notification(), current.notification());
            assert_eq!(rebound.original_input().snapshot().unwrap().observations().unwrap().get(key), Some(new_evidence));
            assert_eq!(rebound.query_snapshot().unwrap().observations().unwrap().get(key), Some(old_evidence));
        }
        rebound_results.push((renew_layer_samples, rebound.is_some()));
        previous = sealed.manifest().clone();
    }
    assert_eq!(first.observations().unwrap().get(key).unwrap(), old_evidence);
    assert_eq!(rebound_results, vec![(false, true), (true, true)],
        "NEW contract: unchanged complete layer query payload must preserve the private query");
}

// These controls first prove a truthful Writer raw-only transition qualifies,
// then independently revoke the prepared result through an unchanged safety gate.
#[derive(Clone, Copy, Debug)]
enum QueryEquivalentSafetyFence {
    ReceiptExpiry,
    UserStop,
    Session,
    Frame,
    PartialSource,
    RecentInput,
    LayerScatter,
}

async fn query_equivalent_writer_still_obeys_safety_fence(fence: QueryEquivalentSafetyFence) {
    use futures::FutureExt;
    let (handle, sink) = configured_handle();
    let request_id = format!("query-equivalent-safety-{fence:?}");
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    let mut ticking: Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>> = None;
    let exercise = std::panic::AssertUnwindSafe(async {
        let config = watermark_owner_config();
        handle.configure_surface_source(Ok(config.clone()), 37, [7; 16]).await.unwrap();
        let repository = nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer = nuwax_map_chunks::Writer::new(repository, 0.1, 4 * 1024 * 1024).unwrap();
        let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
        let source = watermark_owner_source();
        let first_time = now_unix_nanos();
        writer.observe([0, 30].into_iter().flat_map(|z| (0..50).flat_map(move |x|
            (0..30).map(move |y| [x, y, z]))), first_time).unwrap();
        let key = [15, 15, 0];
        let block = nuwax_map_chunks::block_key(key);
        let anchor = nuwax_map_chunks::local_key(key);
        let layers = |spread: f64| std::collections::BTreeMap::from([(anchor,
            nuwax_map_chunks::LayerObservation {
                members: vec![anchor], plane_origin: [1.55, 1.55, 0.025], normal: [0.0, 0.0, 1.0],
                samples: [0.025 - spread, 0.025 + spread].into_iter().enumerate().map(|(index, z)|
                    nuwax_map_chunks::LayerSample { source: source.clone(), point: [1.55, 1.55, z],
                        source_sequence: 1, source_timestamp_ns: first_time, point_index: index as u32 }).collect(),
            })]);
        if matches!(fence, QueryEquivalentSafetyFence::LayerScatter) {
            writer.replace_layer_observations(block, layers(0.001)).unwrap();
        }
        writer.record_source_watermark(source.clone(), first_time, 1).unwrap();
        let (first_message, first_sealed) = watermark_owner_publication(&mut writer, None, 1, first_time);
        let (first, _) = watermark_owner_install(&handle, &mut cache, &first_message).await;
        assert_eq!(handle.start(&request_id, "prefer_wavefront").await.status, STATUS_ACCEPTED);
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let receiver = install_gated_real_surface_source(&handle, audit.clone(), gate.clone(), None).await;
        let original = receiver.freeze().unwrap();
        let planning = original.fork_for_planning();
        let pose = surface_pose(&first, 0.0, Quaternion::IDENTITY);
        ticking = Some(tokio::spawn({
            let handle = handle.clone();
            async move { handle.tick_with_pose(pose).await }
        }));
        owner_watchdog(gate.entered()).await;
        {
            let audit = audit.lock().unwrap();
            assert_eq!((audit.captures.len(), audit.commits.len(), audit.discards), (1, 0, 0));
            assert_eq!(audit.captures[0].checkpoint_before, audit.captures[0].checkpoint_after);
            // A statistics control may prepare a valid non-Goal projection. It
            // still must not commit any staged state from the changed query.
            assert!(audit.captures[0].preview.is_ok(), "{fence:?}: {:?}", audit.captures[0].preview);
            if !matches!(fence, QueryEquivalentSafetyFence::LayerScatter) {
                let _ = surface_owner_goal(audit.captures[0].preview.as_ref().unwrap());
            }
        }
        let second_time = now_unix_nanos().max(first_time + 1);
        writer.observe([key], second_time).unwrap();
        writer.record_source_watermark(source.clone(), second_time, 2).unwrap();
        let (second_message, second_sealed) = watermark_owner_publication(&mut writer,
            Some(first_sealed.manifest().as_ref()), 2, second_time);
        let (second, receipt) = watermark_owner_install(&handle, &mut cache, &second_message).await;
        let current = receiver.freeze().unwrap();
        assert!(!original.same_source_view(&current));
        assert!(original.same_planning_query_view(&current));
        assert!(planning.rebind_original(&current).is_some());
        assert!(second.observations().unwrap().get(key).unwrap().observed_ns
            > first.observations().unwrap().get(key).unwrap().observed_ns);
        assert!(first.same_geometry(&second));

        match fence {
            QueryEquivalentSafetyFence::ReceiptExpiry => {
                let ttl = handle.inner.lock().await.costmap_receipt_ttl;
                tokio::time::sleep(ttl + Duration::from_millis(1)).await;
                let guard = handle.inner.lock().await;
                assert_eq!(guard.costmap_received_monotonic_ns, receipt);
                assert!(now_monotonic_nanos().saturating_sub(receipt) > ttl.as_nanos() as u64);
                assert!(original.same_planning_query_view(&receiver.freeze().unwrap()));
            }
            QueryEquivalentSafetyFence::UserStop => {
                let stopped = owner_watchdog(handle.stop(&request_id)).await;
                assert_eq!(stopped.status, STATUS_CANCELLED);
                assert!(!handle.inner.lock().await.logical_active);
            }
            QueryEquivalentSafetyFence::Session | QueryEquivalentSafetyFence::Frame => {
                // Seal a truthful new-domain root; do not relabel an old fine
                // manifest or skip the actual wire/admission path.
                let third_time = now_unix_nanos().max(second_time + 1);
                let mut next = surface_message("map", 3);
                if matches!(fence, QueryEquivalentSafetyFence::Session) {
                    next.committed_map.session_id = "query-equivalent-new-session".into();
                    next.committed_map.session_generation += 1;
                } else { next.map_frame_generation += 1; }
                next.source_timestamp_ns = third_time as i64;
                next.committed_map.source_timestamp_ns = third_time as i64;
                next.cloud.header.stamp_sec = (third_time / 1_000_000_000) as i32;
                next.cloud.header.stamp_nsec = (third_time % 1_000_000_000) as i32;
                next.cloud.width = 0; next.cloud.row_step = 0; next.cloud.data.clear();
                let sealed = writer.seal(schema::navigation_map_3d_domain(&next), 3, third_time).unwrap();
                next.source_voxel_size = sealed.manifest().voxel_size_m;
                next.fine = Some(crate::lcm_runtime::mapping_feed_tests::prepare_fine_transfer_for_watermark_tests(
                    &sealed, None).unwrap());
                let bytes = schema::encode_navigation_map_3d_body(&next).unwrap();
                let next = schema::decode_navigation_map_3d_body(&bytes).unwrap();
                watermark_owner_install(&handle, &mut cache, &next).await;
                assert!(!handle.inner.lock().await.logical_active);
                assert!(!original.same_planning_query_view(&receiver.freeze().unwrap()));
            }
            QueryEquivalentSafetyFence::PartialSource => {
                let mut partial = second.observations().unwrap().as_ref().clone();
                let values = [0, 30].into_iter().flat_map(|z| (0..32).flat_map(move |x|
                    (0..30).map(move |y| [x, y, z])))
                    .map(|key| (key, second.observations().unwrap().get(key).unwrap())).collect();
                partial.replace_block([0, 0, 0], values).unwrap();
                assert!(!partial.is_complete_geometry_source());
                let partial = Arc::new(second.with_observations(Some(Arc::new(partial))));
                // Explicit partial-source injection, not a complete Reader result.
                publish_surface_owner_receipt(&handle, &second_message.committed_map, 3, partial).await;
                assert!(!original.same_planning_query_view(&receiver.freeze().unwrap()));
            }
            QueryEquivalentSafetyFence::RecentInput => {
                let changed = Arc::new(second.with_recent_obstacle_points(1,
                    second_time, 0, &[], [(0, [1.55, 1.55, 0.6])]).unwrap());
                // Explicit recent injection; this does not claim current GEO
                // Explore LCM ingress attaches the independent recent stream.
                publish_surface_owner_receipt(&handle, &second_message.committed_map, 3, changed).await;
                assert!(!original.same_planning_query_view(&receiver.freeze().unwrap()));
            }
            QueryEquivalentSafetyFence::LayerScatter => {
                writer.replace_layer_observations(block, layers(0.01)).unwrap();
                let third_time = now_unix_nanos().max(second_time + 1);
                let (third_message, third_sealed) = watermark_owner_publication(&mut writer,
                    Some(second_sealed.manifest().as_ref()), 3, third_time);
                assert_eq!(second_sealed.manifest().geometry_revision, third_sealed.manifest().geometry_revision);
                let (third, _) = watermark_owner_install(&handle, &mut cache, &third_message).await;
                assert!(second.same_geometry(&third));
                assert_eq!(second.observations().unwrap().get(key), third.observations().unwrap().get(key));
                assert!(third.query_ground([1.55, 1.55], 0.025).max_residual_m.unwrap()
                    > second.query_ground([1.55, 1.55], 0.025).max_residual_m.unwrap());
                assert!(!original.same_planning_query_view(&receiver.freeze().unwrap()));
                assert!(planning.rebind_original(&receiver.freeze().unwrap()).is_none());
            }
        }
        gate.release();
        let joined = owner_watchdog(ticking.as_mut().unwrap()).await;
        ticking = None;
        joined.unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        let audit = audit.lock().unwrap();
        assert_eq!((audit.captures.len(), audit.commits.len(), audit.discards), (1, 0, 1), "{fence:?}");
        assert!(audit.feedback.is_empty());
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }).catch_unwind().await;
    gate.release();
    let cleanup = std::panic::AssertUnwindSafe(
        watermark_owner_cleanup(&handle, &request_id, &mut ticking),
    ).catch_unwind().await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
    if let Err(panic) = cleanup { std::panic::resume_unwind(panic); }
}

#[tokio::test]
async fn compute_owner_query_equivalent_writer_still_obeys_receipt_expiry() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::ReceiptExpiry).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_still_obeys_user_stop() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::UserStop).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_still_obeys_session_change() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::Session).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_still_obeys_frame_change() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::Frame).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_rejects_partial_source() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::PartialSource).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_rejects_injected_recent() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::RecentInput).await;
}
#[tokio::test]
async fn compute_owner_query_equivalent_writer_rejects_changed_layer_scatter() {
    query_equivalent_writer_still_obeys_safety_fence(QueryEquivalentSafetyFence::LayerScatter).await;
}
