//! Source-only tests accompanying explore_compute_backend.rs. Run them in the
//! parent's next coherent Geometry qualification window; not qualified yet.

use super::*;
use nuwax_explore_wavefront::{FrontierGoalIdSeed, FrontierGoalIdSource};
use nuwax_nav_types_messages::geometry::Point2;
use std::cell::Cell;

struct RecordedIds(Arc<AtomicU64>);
impl FrontierGoalIdSource for RecordedIds {
    fn next_frontier_goal_id(&mut self, seed: &FrontierGoalIdSeed) -> String {
        let sequence = self.0.fetch_add(1, Ordering::SeqCst);
        format!("owner-test-{}-{sequence}", seed.stamp.nanos)
    }
}

fn controller() -> (ExplorationController, Arc<AtomicU64>) {
    let ids = Arc::new(AtomicU64::new(0));
    let adapter = WavefrontGoalSourceAdapter::new(
        "wavefront",
        WavefrontConfig::default(),
        Box::new(RecordedIds(ids.clone())),
    );
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(adapter);
    controller.begin_exploration().unwrap();
    (controller, ids)
}

fn context() -> ComputeContext {
    let mut grid = OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.5,
        width: 12,
        height: 12,
        cells: vec![OccupancyCell::Unknown; 144],
    };
    for y in 2..=7 {
        for x in 2..=7 {
            grid.set(x, y, OccupancyCell::Free);
        }
    }
    let stamp = Timestamp {
        nanos: 123_000_000,
        clock: ClockKind::Unix,
    };
    ComputeContext {
        homeward_test_hook: None,
        attribution: None,
        input: ComputeInput::Planar(Arc::new(ExploreCostmapSnapshot {
            grid,
            frame_id: "map".into(),
        })),
        robot_pose: PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp {
                nanos: 122_000_000,
                clock: ClockKind::Unix,
            },
            pose: Pose3 {
                position: Point3 {
                    x: 2.25,
                    y: 2.25,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        },
        now: stamp,
        config: Arc::new(ExploreStartConfig::default()),
        calibration: None,
    }
}

fn selection_request(context: &ComputeContext) -> ComputeRequest {
    ComputeRequest::Selection(SelectionRequest {
        context: context.clone(),
        progress: None,
    })
}

fn state(context: &ComputeContext) -> ExplorationStateSnapshot<'_> {
    let mut state = ExplorationStateSnapshot::new(context.robot_pose.clone(), context.now);
    let ComputeInput::Planar(map) = &context.input else {
        panic!("planar fixture")
    };
    state.global_costmap = Some(&map.grid);
    state
}

fn checkpoint_payloads(controller: &ExplorationController) -> Vec<(String, String)> {
    controller
        .checkpoint()
        .unwrap()
        .into_iter()
        .map(|source| (source.name, source.payload))
        .collect()
}

fn finalized(backend: &mut ExploreComputeBackend) -> ComputeFinalized {
    let Settlement::Accepted(value) = backend.resolve(Decision::Accept) else {
        panic!("accepted backend result")
    };
    value
}

fn selection(value: ComputeProjection) -> SelectionProjection {
    let ComputeProjection::Selection(selection) = value else {
        panic!("selection result")
    };
    selection
}

#[test]
fn real_wavefront_discard_reprepare_matches_direct_goal_checkpoint_and_ids() {
    let context = context();
    let (mut reference, reference_ids) = controller();
    let expected = reference
        .poll_next_goal_with_deferred_stop(&state(&context), false)
        .unwrap();
    assert!(matches!(expected, ExplorationPoll::Goal(_)));
    let (controller, ids) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let diagnostics = backend.controller.diagnostics();

    let first = selection(backend.prepare_request(None, selection_request(&context), || false));
    assert!(matches!(first.poll, Ok(ExplorationPoll::Goal(_))));
    assert_eq!(
        ids.load(Ordering::SeqCst),
        0,
        "draft must not consume an ID"
    );
    assert_eq!(backend.controller.diagnostics(), diagnostics);
    assert!(backend.controller.frontier_snapshot().is_empty());
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    assert!(matches!(
        backend.resolve(Decision::Discard),
        Settlement::Discarded
    ));
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    assert_eq!(ids.load(Ordering::SeqCst), 0);

    let second = selection(backend.prepare_request(None, selection_request(&context), || false));
    assert_eq!(
        first.poll, second.poll,
        "same sampled source yields the same draft after discard"
    );
    let accepted = finalized(&mut backend);
    let accepted_poll = selection(accepted.projection).poll.unwrap();
    assert_eq!(accepted_poll, expected);
    assert_eq!(
        checkpoint_payloads(&backend.controller),
        checkpoint_payloads(&reference)
    );
    assert_eq!(ids.load(Ordering::SeqCst), 1);
    assert_eq!(
        ids.load(Ordering::SeqCst),
        reference_ids.load(Ordering::SeqCst)
    );
    let ExplorationPoll::Goal(goal) = accepted_poll else {
        unreachable!()
    };
    assert_eq!(
        goal.pose.stamp, context.now,
        "completion must not reanchor the sampled clock"
    );
    assert!(accepted.committed.active);
    assert_eq!(accepted.committed.source_count, 1);
    assert_eq!(accepted.committed.frontiers, vec![goal]);
}

#[test]
fn real_wavefront_feedback_precedes_stop_and_explicit_checkpoint_export() {
    let context = context();
    let (controller, _) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    backend.prepare_request(None, selection_request(&context), || false);
    let accepted = finalized(&mut backend);
    let ExplorationPoll::Goal(goal) = selection(accepted.projection).poll.unwrap() else {
        panic!("real frontier fixture")
    };
    let before = checkpoint_payloads(&backend.controller);
    let feedback = backend.feedback(ComputeFeedback {
        goal_id: goal.id,
        feedback: ExplorationGoalFeedback::Reached,
    });
    feedback.result.unwrap();
    assert!(feedback.committed.frontiers.is_empty());
    let after = checkpoint_payloads(&backend.controller);
    assert_ne!(
        before, after,
        "confirmed feedback must update real source history"
    );
    let payload: serde_json::Value = serde_json::from_str(&after[0].1).unwrap();
    assert_eq!(payload[1]["explored_goals"].as_array().unwrap().len(), 1);
    assert_eq!(payload[1]["consecutive_failures"], 0);

    let stopped = backend.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled));
    assert!(matches!(
        stopped.result,
        Ok(ComputeLifecycleOutput::Stopped)
    ));
    assert!(!stopped.committed.active);
    let exported = backend.lifecycle(ComputeLifecycle::ExportCheckpoint);
    let ComputeLifecycleOutput::Checkpoint(sources) = exported.result.unwrap() else {
        panic!("explicit checkpoint result")
    };
    assert_eq!(
        sources
            .into_iter()
            .map(|source| (source.name, source.payload))
            .collect::<Vec<_>>(),
        after
    );
}

#[test]
fn route_only_resolution_and_returning_selection_have_no_controller_delta() {
    let context = context();
    let (controller, ids) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let progress = MissionProgress::new(mission_pose_from_stamped(&context.robot_pose)).unwrap();
    let projection = backend.prepare_request(
        None,
        ComputeRequest::HomewardBudget(HomewardBudgetRequest {
            context: context.clone(),
            progress: progress.clone(),
        }),
        || false,
    );
    assert!(
        matches!(projection, ComputeProjection::HomewardBudget(Err(_))),
        "missing captured route calibration stays an error"
    );
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    assert!(matches!(
        finalized(&mut backend).projection,
        ComputeProjection::HomewardBudget(Err(_))
    ));
    assert_eq!(checkpoint_payloads(&backend.controller), before);

    let mut returning = progress;
    returning.returning = true;
    let projection = selection(backend.prepare_request(
        None,
        ComputeRequest::Selection(SelectionRequest {
            context,
            progress: Some(returning),
        }),
        || false,
    ));
    assert_eq!(projection.poll, Ok(ExplorationPoll::Pending));
    assert!(matches!(projection.mission, Some(Err(_))));
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    let _ = finalized(&mut backend);
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    assert_eq!(ids.load(Ordering::SeqCst), 0);
}

#[test]
fn cancellation_after_real_preparation_retains_delta_until_discard() {
    let context = context();
    let (controller, ids) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let checks = Cell::new(0);
    let result = selection(backend.prepare_request(None, selection_request(&context), || {
        checks.set(checks.get() + 1);
        checks.get() >= 3 // initial, post-fork, post-controller-prepare
    }));
    assert!(result.cancelled);
    assert!(matches!(result.poll, Ok(ExplorationPoll::Goal(_))));
    assert!(backend.prepared.as_ref().unwrap().controller_prepared);
    assert!(matches!(
        backend.resolve(Decision::Discard),
        Settlement::Discarded
    ));
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    assert_eq!(ids.load(Ordering::SeqCst), 0);
    backend.prepare_request(None, selection_request(&context), || false);
    assert!(matches!(
        selection(finalized(&mut backend).projection).poll,
        Ok(ExplorationPoll::Goal(_))
    ));
}

#[test]
fn unsupported_source_is_typed_and_cancellation_cannot_launder_its_error() {
    struct Unsupported;
    impl ExplorationGoalSource for Unsupported {
        fn source_name(&self) -> std::borrow::Cow<'static, str> {
            "unsupported".into()
        }
        fn begin(&mut self) -> ExploreResult<()> {
            Ok(())
        }
        fn end(&mut self) -> ExploreResult<()> {
            Ok(())
        }
        fn poll_goal(
            &mut self,
            _: &ExplorationStateSnapshot<'_>,
        ) -> ExploreResult<ExplorationPoll> {
            panic!("owner must never fall back to synchronous custom-source polling")
        }
        fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
            nuwax_exploration::ExplorationSourceDiagnostics {
                source_name: "unsupported".into(),
                is_active: true,
                last_goal_id: None,
                last_error: None,
            }
        }
    }
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    controller.add_source(Unsupported);
    controller.begin_exploration().unwrap();
    let mut backend = ExploreComputeBackend::new(controller);
    let checks = Cell::new(0);
    let result = selection(backend.prepare_request(None, selection_request(&context()), || {
        checks.set(checks.get() + 1);
        checks.get() >= 3
    }));
    assert!(result.cancelled);
    assert_eq!(result.poll, Err(ExploreError::PreparedPollUnsupported));
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    // Normal callers discard cancelled work; even an erroneous acceptance may
    // not change a real preparation error into Pending/success.
    assert_eq!(
        selection(finalized(&mut backend).projection).poll,
        Err(ExploreError::PreparedPollUnsupported)
    );
}

#[test]
fn install_restore_failure_preserves_old_history_and_allows_next_start() {
    let (old, _) = controller();
    let mut backend = ExploreComputeBackend::new(old);
    backend
        .lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled))
        .result
        .unwrap();
    let before = checkpoint_payloads(&backend.controller);
    let candidate = build_controller(
        ExplorationArbitrationPolicy::PreferWavefront,
        &ExploreStartConfig::default(),
    );
    let failed = backend.lifecycle(ComputeLifecycle::Install {
        controller: candidate,
        begin: true,
        restore: Some(ComputeRestore {
            sources: vec![SourceCheckpoint {
                name: "wrong-source".into(),
                payload: "invalid".into(),
            }],
            replan: false,
        }),
    });
    assert!(failed.result.is_err());
    assert!(!failed.committed.active);
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    let candidate = build_controller(
        ExplorationArbitrationPolicy::PreferWavefront,
        &ExploreStartConfig::default(),
    );
    let installed = backend.lifecycle(ComputeLifecycle::Install {
        controller: candidate,
        begin: true,
        restore: None,
    });
    assert!(matches!(
        installed.result,
        Ok(ComputeLifecycleOutput::Installed)
    ));
    assert!(installed.committed.active);
}

fn mission_point(x: f64) -> MissionPose {
    MissionPose {
        position: [x, 0.0, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    }
}

#[test]
fn mission_queries_preserve_homeward_first_and_candidate_error_nesting() {
    let progress = MissionProgress::new(mission_point(0.0)).unwrap();
    let start = mission_point(1.0);
    let candidate = mission_point(2.0);
    let policy = ReturnPolicy {
        maximum_distance_m: Some(10.0),
        ..Default::default()
    };
    for failing_call in [0, 1, 2, 3] {
        let mut calls = Vec::new();
        let result = ordered_mission_query(
            &start,
            &progress,
            &policy,
            Some((&candidate, GoalOptions::default())),
            |from, to, _options| {
                calls.push((from.position[0], to.position[0]));
                if calls.len() == failing_call {
                    Err(format!("route-{failing_call}").into())
                } else {
                    Ok(MissionRoute { length: 1.0, end: to.clone() })
                }
            },
            &|| false,
        );
        let RouteComputation::Complete(result) = result else {
            panic!("not cancelled")
        };
        let expected_calls = [(1.0, 0.0), (1.0, 2.0), (2.0, 0.0)];
        assert_eq!(
            calls,
            expected_calls[..if failing_call == 0 { 3 } else { failing_call }]
        );
        match failing_call {
            0 => assert_eq!(result.unwrap().can_visit, Ok(true)),
            1 => assert_eq!(result, Err("route-1".into())),
            n => {
                let advice = result.unwrap();
                assert_eq!(advice.homeward, 1.0);
                assert_eq!(advice.can_visit, Err(format!("route-{n}").into()));
            }
        }
    }
}

#[derive(Default)]
struct ProtocolCounts {
    prepares: AtomicU64,
    commits: AtomicU64,
    discards: AtomicU64,
}

struct ProtocolSource {
    name: &'static str,
    pending: bool,
    fail_prepare: bool,
    fail_discard: bool,
    change_commit: bool,
    counts: Arc<ProtocolCounts>,
}

impl ProtocolSource {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            pending: false,
            fail_prepare: false,
            fail_discard: false,
            change_commit: false,
            counts: Arc::new(ProtocolCounts::default()),
        }
    }
}

impl ExplorationGoalSource for ProtocolSource {
    fn source_name(&self) -> std::borrow::Cow<'static, str> {
        self.name.into()
    }
    fn begin(&mut self) -> ExploreResult<()> {
        Ok(())
    }
    fn end(&mut self) -> ExploreResult<()> {
        if self.pending {
            Err(ExploreError::PreparedPollBusy)
        } else {
            Ok(())
        }
    }
    fn poll_goal(&mut self, _: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
        panic!("concrete backend must use prepared polling")
    }
    fn supports_prepared_poll(&self) -> bool {
        true
    }
    fn prepare_poll(
        &mut self,
        _: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
        self.counts.prepares.fetch_add(1, Ordering::SeqCst);
        if self.fail_prepare {
            return Err(ExploreError::SourceError(
                "later preparation rejected".into(),
            ));
        }
        assert!(!self.pending);
        self.pending = true;
        Ok(nuwax_exploration::PreparedExplorationPoll::new(Ok(
            ExplorationPoll::Pending,
        )))
    }
    fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
        assert!(self.pending);
        self.pending = false;
        self.counts.commits.fetch_add(1, Ordering::SeqCst);
        Ok(if self.change_commit {
            ExplorationPoll::NoFrontier
        } else {
            ExplorationPoll::Pending
        })
    }
    fn discard_prepared_poll(&mut self) -> ExploreResult<()> {
        assert!(self.pending);
        self.counts.discards.fetch_add(1, Ordering::SeqCst);
        if self.fail_discard {
            return Err(ExploreError::SourceError(
                "rollback discard rejected".into(),
            ));
        }
        self.pending = false;
        Ok(())
    }
    fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
        nuwax_exploration::ExplorationSourceDiagnostics {
            source_name: self.name.into(),
            is_active: true,
            last_goal_id: None,
            last_error: None,
        }
    }
}

fn owner_until(
    handle: &ExplorationComputeClient,
    predicate: impl Fn(&owner::Status) -> bool,
) -> owner::Status {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut status = handle.status();
    while !predicate(&status) {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .expect("concrete owner transition timed out");
        status = handle.wait_changed(status.epoch, remaining);
    }
    status
}

fn assert_owner_fault_blocks_reuse(handle: &ExplorationComputeClient) {
    let status = owner_until(handle, |status| status.fault.is_some());
    assert_eq!(status.fault, Some(owner::Fault::Panic));
    assert!(!status.admitting);
    assert!(!status.prepared_available);
    assert!(!status.settled_available);
    let fault = Err(owner::OwnerError::Faulted(owner::Fault::Panic));
    assert_eq!(handle.open_admission(), fault);
    assert_eq!(
        handle.submit_lifecycle(ComputeLifecycle::ExportCheckpoint),
        Err(owner::OwnerError::Faulted(owner::Fault::Panic))
    );
    assert_eq!(
        handle.submit_lifecycle(ComputeLifecycle::Install {
            controller: build_controller(
                ExplorationArbitrationPolicy::PreferWavefront,
                &ExploreStartConfig::default()
            ),
            begin: true,
            restore: None,
        }),
        Err(owner::OwnerError::Faulted(owner::Fault::Panic))
    );
    assert!(matches!(
        handle.try_take_prepared(),
        Err(owner::OwnerError::Faulted(owner::Fault::Panic))
    ));
    assert!(matches!(
        handle.try_take_settled(),
        Err(owner::OwnerError::Faulted(owner::Fault::Panic))
    ));
}

#[test]
fn concrete_owner_changed_committed_outcome_permanently_blocks_export_and_reinstall() {
    let mut source = ProtocolSource::new("changed-commit");
    source.change_commit = true;
    let counts = source.counts.clone();
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    controller.add_source(source);
    controller.begin_exploration().unwrap();
    let owner = ExplorationComputeOwner::spawn(ExploreComputeBackend::new(controller)).unwrap();
    let handle = owner.handle();
    handle.open_admission().unwrap();
    let id = handle.submit(selection_request(&context())).unwrap();
    owner_until(&handle, |status| status.prepared_available);
    let prepared = handle.try_take_prepared().unwrap().unwrap();
    assert_eq!(prepared.id, id);
    assert_eq!(
        selection(prepared.projection).poll,
        Ok(ExplorationPoll::Pending)
    );
    handle.resolve(id, Decision::Accept).unwrap();
    assert_owner_fault_blocks_reuse(&handle);
    assert_eq!(counts.prepares.load(Ordering::SeqCst), 1);
    assert_eq!(counts.commits.load(Ordering::SeqCst), 1);
    drop(owner); // joins the already-faulted worker, never replaces/detaches it
}

#[test]
fn concrete_owner_later_prepare_and_rollback_discard_failure_permanently_faults() {
    let mut first = ProtocolSource::new("first");
    first.fail_discard = true;
    let first_counts = first.counts.clone();
    let mut second = ProtocolSource::new("later");
    second.fail_prepare = true;
    let second_counts = second.counts.clone();
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::HighestScore);
    controller.add_source(first);
    controller.add_source(second);
    controller.begin_exploration().unwrap();
    let owner = ExplorationComputeOwner::spawn(ExploreComputeBackend::new(controller)).unwrap();
    let handle = owner.handle();
    handle.open_admission().unwrap();
    handle.submit(selection_request(&context())).unwrap();
    assert_owner_fault_blocks_reuse(&handle);
    assert_eq!(first_counts.prepares.load(Ordering::SeqCst), 1);
    assert_eq!(first_counts.discards.load(Ordering::SeqCst), 1);
    assert_eq!(first_counts.commits.load(Ordering::SeqCst), 0);
    assert_eq!(second_counts.prepares.load(Ordering::SeqCst), 1);
    drop(owner);
}

fn surface_fixture(offset: f32) -> Arc<SurfaceSnapshot> {
    let config = nuwax_nav_mls_planner::mls_planner::Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.45,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.1,
        wall_buffer_weight: 0.0,
        step_threshold_m: 0.2,
        max_slope_deg: 25.0,
        min_step_tread_depth_m: 0.3,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    };
    let mut points = Vec::new();
    for level in [0.0_f32, 3.0] {
        for x in 0..20 {
            for y in 0..20 {
                points.push((
                    (x as f32 + 0.5) * 0.1,
                    (y as f32 + 0.5) * 0.1,
                    level + offset + 0.05,
                ));
            }
        }
    }
    Arc::new(SurfaceSnapshot::from_points(&points, config, 0.1).unwrap())
}

fn surface_controller(receiver: SurfaceInputReceiver) -> ExplorationController {
    let mut source = SurfaceGoalSourceAdapter::new("surface", 0.3, receiver).unwrap();
    source.configure_constraints(None, 1, 0.3).unwrap();
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(source);
    controller.begin_exploration().unwrap();
    controller
}

#[test]
fn actual_surface_backend_uses_original_input_and_pins_it_until_resolution() {
    let receiver = SurfaceInputReceiver::new();
    let snapshot = surface_fixture(0.0);
    let original_weak = Arc::downgrade(&snapshot);
    receiver.observe(snapshot.clone()).unwrap();
    let original = receiver.freeze().unwrap();
    let notification = original.notification();
    let mut context = context();
    context.robot_pose.pose.position = Point3::new(1.05, 1.05, 0.4);
    context.input = ComputeInput::Surface {
        original,
        config: SurfaceNavigationConfig::new("map".into(), 0.3, 0.2, 0.45, 0.1, 0.2, 25.0, 0.3)
            .unwrap(),
    };
    let sampled = ExplorationStateSnapshot::new(context.robot_pose.clone(), context.now);
    let expected = {
        let mut reference = surface_controller(receiver.clone());
        reference
            .poll_next_goal_with_deferred_stop(&sampled, false)
            .unwrap()
    };
    assert!(matches!(expected, ExplorationPoll::Goal(_)));
    let mut backend = ExploreComputeBackend::new(surface_controller(receiver.clone()));
    // Replacement has no surface near the sampled body; a live read would differ.
    receiver.observe(surface_fixture(10.0)).unwrap();
    assert!(!matches!(
        surface_controller(receiver.clone())
            .poll_next_goal_with_deferred_stop(&sampled, false)
            .unwrap(),
        ExplorationPoll::Goal(_)
    ));
    drop(snapshot);
    let projection = selection(backend.prepare_request(
        None,
        ComputeRequest::Selection(SelectionRequest {
            context,
            progress: None,
        }),
        || false,
    ));
    assert_eq!(projection.poll.as_ref().unwrap(), &expected);
    // 请求局部引用已释放；prepared anchor 与 owner 私有槽各保留一份原始来源。
    assert_eq!(original_weak.strong_count(), 2);
    let pending_original = backend
        .prepared
        .as_ref()
        .unwrap()
        ._original_surface
        .as_ref()
        .unwrap();
    assert_eq!(pending_original.notification(), notification);
    assert!(!pending_original.same_source_view(&receiver.freeze().unwrap()));
    let accepted = finalized(&mut backend);
    assert_eq!(selection(accepted.projection).poll.unwrap(), expected);
    assert_eq!(
        original_weak.strong_count(),
        1,
        "settlement releases only the transaction pin; the qualified owner slot remains"
    );
    backend.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled)).result.unwrap();
    assert!(original_weak.upgrade().is_none(), "Stop releases the retained source");
}

#[test]
fn actual_calibrated_homeward_budget_query_succeeds_without_controller_delta() {
    let mut context = context();
    let mut grid = OccupancyGrid2D::new(
        Point2::new(0.0, 0.0),
        0.1,
        60,
        60,
        vec![OccupancyCell::Free; 3600],
    );
    for y in 0..45 {
        grid.set(30, y, OccupancyCell::Occupied);
    }
    let map = Arc::new(ExploreCostmapSnapshot {
        grid,
        frame_id: "map".into(),
    });
    context.input = ComputeInput::Planar(map.clone());
    context.robot_pose.pose.position = Point3::new(2.0, 2.0, 0.0);
    context.calibration = Some(crate::explore_mission::tests::calibration());
    let mut home = mission_pose_from_stamped(&context.robot_pose);
    home.position[0] = 4.0;
    let mut progress = MissionProgress::new(home).unwrap();
    progress.travelled_m = 1.0;
    let homeward = context
        .routes(None)
        .unwrap()
        .query(
            &mission_pose_from_stamped(&context.robot_pose),
            &progress.home,
            home_goal_options(),
            None,
        )
        .unwrap().length;
    assert!(homeward > 5.0, "real route must detour around the wall");
    Arc::make_mut(&mut context.config)
        .return_policy
        .maximum_distance_m = Some(homeward);
    let (controller, ids) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let projection = backend.prepare_request(
        None,
        ComputeRequest::HomewardBudget(HomewardBudgetRequest { context, progress }),
        || false,
    );
    assert!(matches!(
        projection,
        ComputeProjection::HomewardBudget(Ok(true))
    ));
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    assert!(matches!(
        finalized(&mut backend).projection,
        ComputeProjection::HomewardBudget(Ok(true))
    ));
    assert_eq!(checkpoint_payloads(&backend.controller), before);
    assert_eq!(ids.load(Ordering::SeqCst), 0);
    assert_eq!(map.grid.get(30, 20), Some(OccupancyCell::Occupied));
}

#[test]
fn coverage_and_narrow_retirement_do_not_add_navigation_outcomes() {
    let context = context();
    let (controller, _) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let projection = backend.prepare_request(
        None,
        ComputeRequest::Coverage {
            context: context.clone(),
        },
        || false,
    );
    assert!(matches!(projection, ComputeProjection::Coverage(Some(value)) if value == 25.0));
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    assert!(
        matches!(finalized(&mut backend).projection, ComputeProjection::Coverage(Some(value)) if value == 25.0)
    );
    assert_eq!(checkpoint_payloads(&backend.controller), before);

    backend.prepare_request(
        None,
        ComputeRequest::InspectCheckpoint {
            context: context.clone(),
        },
        || false,
    );
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    let ComputeProjection::Checkpoint(result) = finalized(&mut backend).projection else {
        panic!("test-only serialized checkpoint inspection")
    };
    assert_eq!(
        result
            .unwrap()
            .into_iter()
            .map(|source| (source.name, source.payload))
            .collect::<Vec<_>>(),
        before
    );

    backend.prepare_request(None, selection_request(&context), || false);
    let ExplorationPoll::Goal(goal) = selection(finalized(&mut backend).projection).poll.unwrap()
    else {
        panic!("real frontier")
    };
    let after_selection = checkpoint_payloads(&backend.controller);
    let rejected = backend.lifecycle(ComputeLifecycle::Retire {
        goal_id: goal.id.clone(),
        feedback: ExplorationGoalFeedback::Reached,
    });
    assert!(rejected.result.is_err());
    assert!(!rejected.committed.frontiers.is_empty());
    let retired = backend.lifecycle(ComputeLifecycle::Retire {
        goal_id: goal.id,
        feedback: ExplorationGoalFeedback::Cancelled,
    });
    assert!(matches!(
        retired.result,
        Ok(ComputeLifecycleOutput::Retired)
    ));
    assert!(retired.committed.frontiers.is_empty());
    assert_eq!(
        checkpoint_payloads(&backend.controller),
        after_selection,
        "retirement cannot add reached/failure/regional history"
    );
}

#[test]
fn install_rejects_unsupported_source_before_begin_and_preserves_old_controller() {
    struct NeverBegin;
    impl ExplorationGoalSource for NeverBegin {
        fn source_name(&self) -> std::borrow::Cow<'static, str> {
            "unsupported-install".into()
        }
        fn begin(&mut self) -> ExploreResult<()> {
            panic!("unsupported source must be rejected before begin")
        }
        fn end(&mut self) -> ExploreResult<()> {
            panic!("unbegun source requires no end call")
        }
        fn poll_goal(
            &mut self,
            _: &ExplorationStateSnapshot<'_>,
        ) -> ExploreResult<ExplorationPoll> {
            panic!("unsupported source must never be polled")
        }
        fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
            nuwax_exploration::ExplorationSourceDiagnostics {
                source_name: "unsupported-install".into(),
                is_active: false,
                last_goal_id: None,
                last_error: None,
            }
        }
    }
    let (old, _) = controller();
    let mut backend = ExploreComputeBackend::new(old);
    backend
        .lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled))
        .result
        .unwrap();
    let before = checkpoint_payloads(&backend.controller);
    let mut candidate = ExplorationController::new(ExplorationArbitrationPolicy::HighestScore);
    candidate.add_source(NeverBegin);
    let ack = backend.lifecycle(ComputeLifecycle::Install {
        controller: candidate,
        begin: true,
        restore: None,
    });
    assert!(matches!(
        ack.result,
        Err(ExploreError::PreparedPollUnsupported)
    ));
    assert_eq!(ack.committed.source_count, 1);
    assert_eq!(
        ack.committed.diagnostics.sources[0].source_name,
        "wavefront"
    );
    assert_eq!(checkpoint_payloads(&backend.controller), before);
}

fn cancellable_surface_context() -> ComputeContext {
    let map = surface_fixture(0.0)
        .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
            z_bounds_m: [-0.05, 0.2],
            body_to_support_z_m: 0.3,
            max_step_up_m: 0.2,
            max_step_down_m: 0.2,
            max_cross_slope_deg: 15.0,
            support_uncertainty_m: 0.0,
        }).unwrap();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(Arc::new(map)).unwrap();
    let mut context = context();
    context.robot_pose.pose.position = Point3::new(1.05, 1.05, 0.4);
    context.input = ComputeInput::Surface {
        original: receiver.freeze().unwrap(),
        config: SurfaceNavigationConfig::new("map".into(), 0.3, 0.2, 0.45, 0.1, 0.2, 25.0, 0.3)
            .unwrap(),
    };
    context
}

#[test]
fn single_surface_query_observes_same_flag_after_backend_prechecks() {
    // Set the flag at the last existing between-stage check, but return false
    // from that test predicate: only the real planner can observe cancellation.
    // Both queries would otherwise succeed on this actual full-body map.
    for returning_selection in [false, true] {
        let context = cancellable_surface_context();
        let start = mission_pose_from_stamped(&context.robot_pose);
        let planning = context.input.fork_for_planning();
        let routes = context.routes(planning.as_ref()).unwrap();
        assert!(routes.query(&start, &start, home_goal_options(), None).unwrap().length.is_finite());
        let mut progress = MissionProgress::new(start).unwrap();
        let request = if returning_selection {
            progress.returning = true;
            ComputeRequest::Selection(SelectionRequest { context, progress: Some(progress) })
        } else {
            ComputeRequest::HomewardBudget(HomewardBudgetRequest { context, progress })
        };
        let (controller, ids) = controller();
        let mut backend = ExploreComputeBackend::new(controller);
        let before = checkpoint_payloads(&backend.controller);
        let cancellation = std::sync::atomic::AtomicBool::new(false);
        let checks = Cell::new(0);
        let last_check = if returning_selection { 4 } else { 3 };
        let projection = backend.prepare_request(Some(&cancellation), request, || {
            checks.set(checks.get() + 1);
            if checks.get() == last_check {
                cancellation.store(true, Ordering::Release);
            }
            false
        });
        assert_eq!(checks.get(), last_check);
        if returning_selection {
            let result = selection(projection);
            assert_eq!(result.mission, Some(Err("三维规划已取消".into())));
            assert!(!result.cancelled, "the real query error retains its existing nesting");
        } else {
            assert!(matches!(projection, ComputeProjection::HomewardBudget(Err(ref error))
                if error.to_string() == "三维规划已取消"));
        }
        assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
        assert!(matches!(backend.resolve(Decision::Discard), Settlement::Discarded));
        assert!(backend.prepared.is_none());
        assert_eq!(checkpoint_payloads(&backend.controller), before);
        assert_eq!(ids.load(Ordering::SeqCst), 0);
    }
}

#[test]
fn surface_motion_limits_error_precedes_cancel_without_changing_route_nesting() {
    use nuwax_nav_types_messages::{MotionLimits, MotionModel};
    let mut context = cancellable_surface_context();
    let limits = MotionLimits {
        model: MotionModel::Differential,
        max_forward_mps: 0.5, max_reverse_mps: 0.3, max_lateral_mps: 0.0,
        max_angular_rps: 0.7, linear_acceleration_mps2: 1.0,
        guaranteed_braking_mps2: 0.0, // Not part of MotionKinematics validation.
        angular_acceleration_rps2: 2.0, guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.1, minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
    };
    let expected = limits.validate().unwrap_err();
    assert!(limits.kinematics().validate().is_ok());
    let ComputeInput::Surface { config, .. } = &mut context.input else { unreachable!() };
    config.follower.motion_limits = Some(limits);
    let planning = context.input.fork_for_planning();
    let invalid = context.routes(planning.as_ref()).unwrap();
    let valid_context = cancellable_surface_context();
    let valid_planning = valid_context.input.fork_for_planning();
    let valid = valid_context.routes(valid_planning.as_ref()).unwrap();
    let start = mission_pose_from_stamped(&context.robot_pose);
    let progress = MissionProgress::new(start.clone()).unwrap();
    for failing_query in 1..=3 {
        let flag = std::sync::atomic::AtomicBool::new(false);
        let mut calls = 0;
        let result = ordered_mission_query(&start, &progress, &ReturnPolicy::default(), Some((&start, GoalOptions::default())),
            |from, to, options| {
                calls += 1;
                if calls == failing_query {
                    flag.store(true, Ordering::Release);
                    invalid.query(from, to, options, Some(&flag))
                } else {
                    valid.query(from, to, options, Some(&flag))
                }
            }, &|| flag.load(Ordering::Acquire));
        assert_eq!(calls, failing_query);
        let RouteComputation::Complete(result) = result else {
            panic!("a real validation error must not be replaced by cancellation")
        };
        if failing_query == 1 {
            assert_eq!(result, Err(expected.clone().into()));
        } else {
            assert_eq!(result.unwrap().can_visit, Err(expected.clone().into()));
        }
    }
}


#[test]
fn mission_frontier_options_match_child_dispatch_and_keep_home_strict() {
    let context = cancellable_surface_context();
    let ComputeInput::Surface { config, .. } = &context.input else { unreachable!() };
    let mut goal = FrontierGoal::new("constraints", context.robot_pose.clone(), 1.0);
    assert_eq!(frontier_goal_options(&goal, true, None), GoalOptions::default());
    for directional in [true, false] {
        goal.metadata.insert("goal_orientation_required".into(), directional.to_string());
        let options = frontier_goal_options(&goal, true, Some(config));
        assert_eq!(options.position_tolerance_m, 0.35);
        assert_eq!(options.yaw_tolerance_rad, if directional { 0.35 } else { std::f64::consts::PI });
        assert_eq!(options.allow_reverse, config.follower.kinematics().is_some_and(|m| m.max_reverse_mps > 0.0));
        assert_eq!(frontier_goal_options(&goal, true, None).yaw_tolerance_rad, options.yaw_tolerance_rad);
        let planar = frontier_goal_options(&goal, false, None);
        assert_eq!(planar, GoalOptions::default());
        // Even orientation-free surface metadata must never relax return goals.
        assert_eq!(home_goal_options(), GoalOptions {
            allow_reverse: false, position_tolerance_m: 0.1, yaw_tolerance_rad: 0.2,
        });
    }
}

#[test]
fn mission_queries_connect_actual_endpoint_and_charge_only_actual_lengths() {
    let start = mission_point(1.0);
    let candidate = mission_point(2.0);
    let end = MissionPose { position: [1.8, 0.1, 0.0], orientation_xyzw: [0.0, 0.0, 0.5, 0.8660254037844386] };
    let mut progress = MissionProgress::new(mission_point(0.0)).unwrap();
    progress.travelled_m = 1.0;
    let policy = ReturnPolicy { maximum_distance_m: Some(6.0), reserve_distance_m: 1.0, ..Default::default() };
    let options = GoalOptions { yaw_tolerance_rad: std::f64::consts::PI, ..Default::default() };
    let mut calls = Vec::new();
    let result = ordered_mission_query(&start, &progress, &policy, Some((&candidate, options)),
        |from, to, options| {
            calls.push((from.clone(), to.clone(), options));
            Ok(MissionRoute { length: if calls.len() == 1 { 99.0 } else { 2.0 },
                end: if calls.len() == 2 { end.clone() } else { to.clone() } })
        }, &|| false);
    let RouteComputation::Complete(Ok(advice)) = result else { panic!("expected completed advice") };
    assert_eq!(advice.homeward, 99.0);
    assert_eq!(advice.can_visit, Ok(true)); // travelled + Q1 + Q2 + reserve = 6; Q0 is separate.
    assert_eq!(calls, vec![
        (start.clone(), progress.home.clone(), home_goal_options()),
        (start, candidate, options),
        (end, progress.home, home_goal_options()),
    ]);
}

#[test]
fn mission_surface_route_carries_real_projected_endpoint_into_home_query() {
    let context = cancellable_surface_context();
    let planning = context.input.fork_for_planning();
    let routes = context.routes(planning.as_ref()).unwrap();
    let start = mission_pose_from_stamped(&context.robot_pose);
    let mut goal = FrontierGoal::new("projected-end", context.robot_pose.clone(), 1.0);
    goal.pose.pose.position.x += 0.3;
    goal.pose.pose.position.z += 0.2;
    goal.pose.pose.orientation = Quaternion::from_yaw(std::f64::consts::PI);
    goal.metadata.insert("goal_orientation_required".into(), "false".into());
    let ComputeInput::Surface { config, .. } = &context.input else { unreachable!() };
    let options = frontier_goal_options(&goal, true, Some(config));
    let candidate = mission_pose_from_stamped(&goal.pose);
    let MissionRoutes::Surface { map, config, boundary } = &routes else { unreachable!() };
    let path = nuwax_nav_stack::surface_oriented::plan_reference_cancellable(map, &config.follower.pct,
        (start.position, start.orientation_xyzw), (candidate.position, candidate.orientation_xyzw),
        options, config.follower.motion_limits.as_ref(), boundary, 30_000, None).unwrap();
    let expected_end = path.last().unwrap();
    let expected_length: f64 = path.windows(2).map(|p|
        p[0].0.iter().zip(p[1].0).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt()).sum();
    let outward = routes.query(&start, &candidate, options, None).unwrap();
    assert_eq!(outward.end.position, expected_end.0);
    assert_eq!(outward.end.orientation_xyzw, expected_end.1);
    assert_eq!(outward.length, expected_length);
    assert_ne!(outward.end.position, candidate.position, "terrain projection must expose a different endpoint");
    assert_ne!(outward.end.orientation_xyzw, candidate.orientation_xyzw, "free yaw must not invent nominal orientation");
    let progress = MissionProgress::new(start.clone()).unwrap();
    let mut calls = 0;
    let result = ordered_mission_query(&start, &progress, &ReturnPolicy::default(), Some((&candidate, options)),
        |from, to, options| {
            calls += 1;
            if calls == 3 {
                assert_eq!(from, &outward.end);
                assert_eq!(to, &progress.home);
                assert_eq!(options, home_goal_options());
            }
            routes.query(from, to, options, None)
        }, &|| false);
    assert_eq!(calls, 3);
    let RouteComputation::Complete(Ok(advice)) = result else { panic!("expected real route advice") };
    assert!(advice.can_visit.is_ok());
}

#[test]
fn mission_zero_length_surface_query_still_requires_complete_body_model() {
    let routes = MissionRoutes::Surface {
        map: surface_fixture(0.0),
        config: SurfaceNavigationConfig::new("map".into(), 0.3, 0.2, 0.45, 0.1, 0.2, 25.0, 0.3).unwrap(),
        boundary: Vec::new(),
    };
    let start = MissionPose { position: [1.05, 1.05, 0.4], orientation_xyzw: [0.0, 0.0, 0.0, 1.0] };
    assert!(routes.query(&start, &start, home_goal_options(), None).is_err());
}

#[test]
fn mission_surface_unknown_query_keeps_requested_home_yaw_and_real_endpoint() {
    let context = cancellable_surface_context();
    let planning = context.input.fork_for_planning();
    let routes = context.routes(planning.as_ref()).unwrap();
    let mut start = mission_pose_from_stamped(&context.robot_pose);
    start.position[0] = -10.0; start.position[1] = -10.0;
    let mut goal = start.clone(); goal.position[0] += 0.5;
    let q = Quaternion::from_yaw(std::f64::consts::FRAC_PI_2);
    goal.orientation_xyzw = [q.x,q.y,q.z,q.w];
    let MissionRoutes::Surface {map,config,..} = &routes else {unreachable!()};
    assert!(map.locate_support_from_body(start.position,start.orientation_xyzw,
        config.follower.body_to_support_z_m).is_err());
    let options = home_goal_options();
    let route = routes.query(&start,&goal,options,None).unwrap();
    assert!(route.length >= 0.4);
    let distance=route.end.position.iter().zip(goal.position).map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
    assert!(distance <= options.position_tolerance_m);
    let [x,y,z,w]=route.end.orientation_xyzw;
    let yaw=(2.0*(x*y+w*z)).atan2(1.0-2.0*(y*y+z*z));
    assert!((yaw-std::f64::consts::FRAC_PI_2).abs() <= options.yaw_tolerance_rad);
    let cancelled = std::sync::atomic::AtomicBool::new(true);
    assert!(routes.query(&start,&goal,options,Some(&cancelled)).err().unwrap().contains("取消"));
}

#[test]
fn mission_resource_error_keeps_typed_homeward_and_candidate_nesting() {
    let context = cancellable_surface_context();
    let start = mission_pose_from_stamped(&context.robot_pose);
    let progress = MissionProgress::new(start.clone()).unwrap();
    for fail_at in 1..=3 {
        let mut calls = 0;
        let result = ordered_mission_query(&start,&progress,&ReturnPolicy::default(),Some((&start,home_goal_options())),
            |_,to,_| {
                calls += 1;
                if calls == fail_at { Err(MissionRouteError::Inconclusive("ResourceLimited: bounded primitive refinement is inconclusive".into())) }
                else { Ok(MissionRoute {length:0.0,end:to.clone()}) }
            }, &||false);
        let RouteComputation::Complete(result) = result else { panic!("resource result is not cancellation") };
        let error = if fail_at == 1 { result.unwrap_err() } else { result.unwrap().can_visit.unwrap_err() };
        assert!(error.is_inconclusive());
        assert_eq!(calls,fail_at);
    }
    let mut context = cancellable_surface_context();
    let ComputeInput::Surface {config,..} = &mut context.input else { unreachable!() };
    config.follower.motion_limits = Some(nuwax_nav_types_messages::MotionLimits {
        model:nuwax_nav_types_messages::MotionModel::Differential,
        max_forward_mps:0.5,max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:1.0,
        linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
        angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,
        reaction_time_sec:0.05,minimum_turning_radius_m:0.5,can_rotate_in_place:false,
    });
    let planning = context.input.fork_for_planning();
    let routes = context.routes(planning.as_ref()).unwrap();
    let mut goal = start.clone();
    let q = Quaternion::from_yaw(std::f64::consts::FRAC_PI_2);
    goal.orientation_xyzw = [q.x,q.y,q.z,q.w];
    assert!(routes.query(&start,&goal,home_goal_options(),None).err().unwrap().is_inconclusive());
}


fn surface_reuse_context(receiver: &SurfaceInputReceiver, config: Arc<ExploreStartConfig>) -> ComputeContext {
    let mut context = context();
    context.robot_pose.pose.position = Point3::new(1.05, 1.05, 0.4);
    context.config = config;
    context.input = ComputeInput::Surface {
        original: receiver.freeze().unwrap(),
        config: SurfaceNavigationConfig::new("map".into(), 0.3, 0.2, 0.45, 0.1, 0.2, 25.0, 0.3).unwrap(),
    };
    context
}

fn surface_reuse_query(backend: &mut ExploreComputeBackend, context: &ComputeContext)
    -> std::sync::Weak<SurfaceSnapshot>
{
    let planning = backend.selection_planning(context).unwrap();
    Arc::downgrade(planning.query_snapshot().unwrap())
}

#[test]
fn surface_planning_slot_rebinds_receipts_but_invalidates_config_boundary_and_absence() {
    let receiver = SurfaceInputReceiver::new();
    let snapshot = surface_fixture(0.0);
    receiver.observe(snapshot.clone()).unwrap();
    let mut context = surface_reuse_context(&receiver, Arc::new(ExploreStartConfig::default()));
    let mut backend = ExploreComputeBackend::new(surface_controller(receiver.clone()));
    let first = surface_reuse_query(&mut backend, &context);
    let notification = backend.surface_planning.as_ref().unwrap().1.original_input().notification();
    receiver.observe(snapshot).unwrap();
    context = surface_reuse_context(&receiver, context.config.clone());
    context.robot_pose.pose.position.x += 0.02;
    let rebound = surface_reuse_query(&mut backend, &context);
    assert!(std::sync::Weak::ptr_eq(&first, &rebound));
    assert_eq!(backend.surface_planning.as_ref().unwrap().1.original_input().notification(), notification + 1);
    // 数值相同但配置拥有方改变，也保守冷失效。
    context.config = Arc::new(context.config.as_ref().clone());
    let changed_config = surface_reuse_query(&mut backend, &context);
    assert!(first.upgrade().is_none());
    assert!(!std::sync::Weak::ptr_eq(&rebound, &changed_config));
    let mut changed = context.config.as_ref().clone();
    changed.boundary = Some(nuwax_exploration::ExplorationBoundary::from_vertices(vec![
        Point2::new(0.0, 0.0), Point2::new(2.0, 0.0), Point2::new(2.0, 2.0), Point2::new(0.0, 2.0),
    ]).unwrap());
    context.config = Arc::new(changed);
    let bounded = surface_reuse_query(&mut backend, &context);
    assert!(changed_config.upgrade().is_none());
    assert!(!std::sync::Weak::ptr_eq(&changed_config, &bounded));
    receiver.clear().unwrap();
    context = surface_reuse_context(&receiver, context.config.clone());
    assert!(backend.selection_planning(&context).unwrap().query_snapshot().is_none());
    assert!(backend.surface_planning.is_none());
    assert!(bounded.upgrade().is_none());
    receiver.observe(surface_fixture(0.0)).unwrap();
    let restored = surface_reuse_context(&receiver, context.config.clone());
    let restored_query = surface_reuse_query(&mut backend, &restored);
    let planar = super::tests::context();
    assert!(backend.selection_planning(&planar).is_none());
    assert!(restored_query.upgrade().is_none());
}

#[test]
fn real_surface_warm_and_cold_queries_match_after_exact_input_feedback() {
    let receiver = SurfaceInputReceiver::new();
    let snapshot = surface_fixture(0.0);
    receiver.observe(snapshot.clone()).unwrap();
    let config = Arc::new(ExploreStartConfig::default());
    let mut warm = ExploreComputeBackend::new(surface_controller(receiver.clone()));
    let mut cold = ExploreComputeBackend::new(surface_controller(receiver.clone()));
    let mut retained = None;
    for round in 0..2 {
        let mut context = surface_reuse_context(&receiver, config.clone());
        if round == 1 {
            context.robot_pose.pose.position.x += 0.02;
            context.robot_pose.pose.orientation = Quaternion {
                x: 0.0, y: 0.0, z: (0.1_f64 * 0.5).sin(), w: (0.1_f64 * 0.5).cos(),
            };
        }
        cold.surface_planning = None;
        let warm_preview = selection(warm.prepare_request(None, selection_request(&context), || false));
        let cold_preview = selection(cold.prepare_request(None, selection_request(&context), || false));
        assert_eq!(warm_preview.poll, cold_preview.poll);
        let warm_poll = selection(finalized(&mut warm).projection).poll.unwrap();
        let cold_poll = selection(finalized(&mut cold).projection).poll.unwrap();
        assert_eq!(warm_poll, cold_poll, "warm and cold outcomes in round {round}");
        let ExplorationPoll::Goal(goal) = warm_poll else { panic!("real Surface goal"); };
        let current = Arc::downgrade(warm.surface_planning.as_ref().unwrap().1.query_snapshot().unwrap());
        if let Some(previous) = &retained { assert!(std::sync::Weak::ptr_eq(previous, &current)); }
        retained = Some(current);
        for backend in [&mut warm, &mut cold] {
            let before = checkpoint_payloads(&backend.controller);
            backend.feedback(ComputeFeedback { goal_id: goal.id.clone(),
                feedback: ExplorationGoalFeedback::InputUnavailable("fixture receipt interruption".into()),
            }).result.unwrap();
            assert_eq!(checkpoint_payloads(&backend.controller), before);
            assert!(backend.surface_planning.is_some(), "ordinary exact feedback keeps immutable queries");
        }
        assert_eq!(checkpoint_payloads(&warm.controller), checkpoint_payloads(&cold.controller));
        receiver.observe(snapshot.clone()).unwrap();
    }
    warm.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled)).result.unwrap();
    assert!(retained.unwrap().upgrade().is_none());
}

#[test]
fn surface_planning_slot_cancel_discard_and_unavailable_release_without_history() {
    let snapshot = surface_fixture(0.0);
    for case in ["pre_cancel", "post_prepare_cancel", "discard", "unavailable"] {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(snapshot.clone()).unwrap();
        let mut context = surface_reuse_context(&receiver, Arc::new(ExploreStartConfig::default()));
        let mut backend = ExploreComputeBackend::new(surface_controller(receiver));
        let retained = surface_reuse_query(&mut backend, &context);
        let before = checkpoint_payloads(&backend.controller);
        if case == "unavailable" { context.robot_pose.frame_id = NavFrameId::from_static("odom"); }
        let checks = Cell::new(0);
        let result = backend.prepare_request(None, selection_request(&context), || {
            checks.set(checks.get() + 1);
            case == "pre_cancel" || case == "post_prepare_cancel" && checks.get() >= 3
        });
        match case {
            "pre_cancel" => {
                assert!(matches!(result, ComputeProjection::Cancelled));
                assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
            }
            "post_prepare_cancel" => {
                let result = selection(result);
                assert!(result.cancelled);
                assert!(matches!(result.poll, Ok(ExplorationPoll::Goal(_))));
                assert!(backend.prepared.as_ref().unwrap().controller_prepared);
            }
            "unavailable" => assert!(matches!(selection(result).poll, Ok(ExplorationPoll::SourceUnavailable(_)))),
            _ => assert!(matches!(selection(result).poll, Ok(ExplorationPoll::Goal(_)))),
        }
        // 查询槽先释放，原事务 anchor 仍在，直到明确 resolve。
        assert!(backend.prepared.as_ref().unwrap()._original_surface.is_some(), "{case}");
        if case != "discard" {
            assert!(backend.surface_planning.is_none(), "{case}");
            assert!(retained.upgrade().is_none(), "{case}");
        }
        assert!(matches!(backend.resolve(Decision::Discard), Settlement::Discarded));
        assert!(backend.prepared.is_none(), "{case}");
        assert!(backend.surface_planning.is_none(), "{case}");
        assert!(retained.upgrade().is_none(), "{case}");
        assert_eq!(checkpoint_payloads(&backend.controller), before, "{case}");
    }
}

#[test]
fn surface_planning_slot_lifecycle_clears_even_failed_install_and_export() {
    let snapshot = surface_fixture(0.0);
    for case in ["stop", "install", "failed_install", "export"] {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(snapshot.clone()).unwrap();
        let context = surface_reuse_context(&receiver, Arc::new(ExploreStartConfig::default()));
        let mut backend = ExploreComputeBackend::new(surface_controller(receiver.clone()));
        if case == "install" {
            // 单独检查 Install 边界，不让之前的 Stop 清槽代替该断言。
            backend.controller.end_exploration().unwrap();
        }
        let retained = surface_reuse_query(&mut backend, &context);
        let operation = match case {
            "stop" => ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled),
            "export" => ComputeLifecycle::ExportCheckpoint,
            _ => ComputeLifecycle::Install {
                controller: surface_controller(receiver), begin: false, restore: None,
            },
        };
        let result = backend.lifecycle(operation).result;
        assert_eq!(result.is_err(), matches!(case, "failed_install" | "export"), "{case}");
        assert!(backend.surface_planning.is_none(), "{case}");
        assert!(retained.upgrade().is_none(), "{case}");
    }
}
