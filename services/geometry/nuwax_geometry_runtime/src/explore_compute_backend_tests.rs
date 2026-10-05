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

    let first = selection(backend.prepare_request(selection_request(&context), || false));
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

    let second = selection(backend.prepare_request(selection_request(&context), || false));
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
    backend.prepare_request(selection_request(&context), || false);
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
    let result = selection(backend.prepare_request(selection_request(&context), || {
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
    backend.prepare_request(selection_request(&context), || false);
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
    let result = selection(backend.prepare_request(selection_request(&context()), || {
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
            Some(&candidate),
            |from, to| {
                calls.push((from.position[0], to.position[0]));
                if calls.len() == failing_call {
                    Err(format!("route-{failing_call}"))
                } else {
                    Ok(1.0)
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
                assert_eq!(advice.can_visit, Err(format!("route-{n}")));
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
        ComputeRequest::Selection(SelectionRequest {
            context,
            progress: None,
        }),
        || false,
    ));
    assert_eq!(projection.poll.as_ref().unwrap(), &expected);
    // All request-local strong roots are gone. The backend pin alone must remain.
    assert_eq!(original_weak.strong_count(), 1);
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
        0,
        "settlement releases the request pin"
    );
    assert!(original_weak.upgrade().is_none());
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
        .distance(
            &mission_pose_from_stamped(&context.robot_pose),
            &progress.home,
        )
        .unwrap();
    assert!(homeward > 5.0, "real route must detour around the wall");
    Arc::make_mut(&mut context.config)
        .return_policy
        .maximum_distance_m = Some(homeward);
    let (controller, ids) = controller();
    let mut backend = ExploreComputeBackend::new(controller);
    let before = checkpoint_payloads(&backend.controller);
    let projection = backend.prepare_request(
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

    backend.prepare_request(selection_request(&context), || false);
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
