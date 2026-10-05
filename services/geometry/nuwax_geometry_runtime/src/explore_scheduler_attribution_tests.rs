// Ignored source-only scheduler attribution harness.
//
// Include as a child of explore_runtime::tests, so the existing recording sink,
// startup helper, lease proof helpers, and per-instance attribution stay shared.
// This is a calibrated software fixture, not a hardware/motion acceptance test.
// It bypasses LCM/scan ingress using committed snapshots in coordinate frame 1.
// The exact benchmark base is 500x500, resolution 0.2 f64, origin (0,0), all Free.
// Frontier border and 8x8 occupied local-edit derivatives are separately labeled.
// Explicit non-default scheduling fixture: 5 ms period, 600 s no-frontier window.
// Copied payload is a lower bound: adapter/inflation/safe-grid and per-distance
// Costmap clones are excluded. Owner stage record pushes occur on the compute thread.
// The workload and request timestamp boundaries match the recorded synchronous baseline.
// No process environment is changed here. Supply the companion environment file
// before launching this one ignored test in its own process (--test-threads=1).

use super::super::attribution::{Attribution, Record, Stage};
use super::*;
use std::sync::atomic::{AtomicBool, AtomicUsize};
use tokio::sync::{oneshot, Semaphore};

const OBSERVER_LIMIT: usize = 2048;
const HEARTBEAT_LIMIT: usize = 256;
const WAIT_LIMIT: Duration = Duration::from_secs(90);
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(2);
const SCHEDULER_PERIOD: Duration = Duration::from_millis(5);
const CALIBRATION_ID: &str = "exploration-attribution-calibration-v1";

#[derive(Clone, Copy, Debug)]
enum Fixture {
    ExactAllFreeBenchmark,
    FrontierBorderDerivative,
}
impl Fixture {
    fn label(self) -> &'static str {
        match self {
            Self::ExactAllFreeBenchmark => "exact_benchmark_all_free_500x500_f64_0_2",
            Self::FrontierBorderDerivative => "frontier_derivative_unknown_20_cell_border",
        }
    }
    fn grid(self, local_edit: bool) -> ExploreCostmapSnapshot {
        let mut grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.2,
            width: 500,
            height: 500,
            cells: vec![OccupancyCell::Free; 250_000],
        };
        if matches!(self, Self::FrontierBorderDerivative) {
            for y in 0..500 {
                for x in 0..500 {
                    if x < 20 || x >= 480 || y < 20 || y >= 480 {
                        grid.set(x, y, OccupancyCell::Unknown);
                    }
                }
            }
        }
        if local_edit {
            // Deliberately away from the pose footprint and initial home (50,50).
            for y in 230..238 {
                for x in 260..268 {
                    grid.set(x, y, OccupancyCell::Occupied);
                }
            }
        }
        ExploreCostmapSnapshot {
            grid,
            frame_id: "map".into(),
        }
    }
}

/// Gate only pose delivery BETWEEN real scheduler ticks, never inside compute.
/// Returning a fresh timestamp models an independently refreshed pose observation.
struct GatedPose {
    permits: Semaphore,
    requested: AtomicUsize,
}
impl GatedPose {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            permits: Semaphore::new(0),
            requested: AtomicUsize::new(0),
        })
    }
    fn release(&self) {
        self.permits.add_permits(1);
    }
    async fn wait_requested(&self, target: usize) {
        tokio::time::timeout(WAIT_LIMIT, async {
            while self.requested.load(Ordering::Acquire) < target {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("scheduler did not request the next pose; inspect its terminal state");
    }
}
#[async_trait]
impl ExplorePoseProvider for GatedPose {
    async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
        self.requested.fetch_add(1, Ordering::Release);
        self.permits
            .acquire()
            .await
            .map_err(|e| e.to_string())?
            .forget();
        Ok(ExploreRobotPose {
            x: 50.0,
            y: 50.0,
            z: 0.0,
            orientation: Quaternion::IDENTITY,
            frame_id: "map".into(),
            source_timestamp_ns: now_unix_nanos(),
            received_monotonic_ns: now_monotonic_nanos(),
        })
    }
}

fn ms(value: Duration) -> f64 {
    value.as_secs_f64() * 1_000.0
}
fn percentile(mut values: Vec<f64>, fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    values[((values.len() - 1) as f64 * fraction).ceil() as usize]
}
fn intersects(a0: Instant, a1: Instant, b0: Instant, b1: Instant) -> bool {
    a0 < b1 && b0 < a1
}
fn overlap_ms(a0: Instant, a1: Instant, records: &[Record], stage: Stage) -> f64 {
    records
        .iter()
        .filter(|r| r.stage == stage)
        .map(|r| {
            let lo = a0.max(r.started);
            let hi = a1.min(r.finished);
            if hi > lo {
                ms(hi.duration_since(lo))
            } else {
                0.0
            }
        })
        .sum()
}

#[derive(Debug)]
struct CallTiming {
    submitted: Instant,
    first_polled: Instant,
    finished: Instant,
    status: String,
}
impl CallTiming {
    fn json(&self, records: &[Record]) -> serde_json::Value {
        serde_json::json!({"queue_delay_ms":ms(self.first_polled.duration_since(self.submitted)),
            "execution_or_lock_wait_ms":ms(self.finished.duration_since(self.first_polled)),
            "end_to_end_ms":ms(self.finished.duration_since(self.submitted)),"status":self.status,
            "selection_overlap_ms":overlap_ms(self.submitted,self.finished,records,Stage::OwnerSelection),
            "route_prepare_overlap_ms":overlap_ms(self.submitted,self.finished,records,Stage::OwnerRoutePreparation),
            "route_query_overlap_ms":overlap_ms(self.submitted,self.finished,records,Stage::OwnerRouteQueries)})
    }
}
struct SnapshotSample {
    requested: Instant,
    finished: Instant,
}
struct Observation {
    snapshots: Vec<SnapshotSample>,
    snapshot_omitted: usize,
    diagnostics: Option<CallTiming>,
}

/// OS sampling is independent of the current-thread Tokio worker. The async
/// diagnostics request is timestamped here BEFORE enqueue on that same worker.
fn observe_phase(
    handle: ExploreRuntimeHandle,
    trace: Arc<Attribution>,
    runtime: tokio::runtime::Handle,
    done: Arc<AtomicBool>,
) -> std::thread::JoinHandle<Observation> {
    std::thread::spawn(move || {
        let deadline = Instant::now() + WAIT_LIMIT;
        let mut snapshots = Vec::with_capacity(OBSERVER_LIMIT);
        let mut omitted = 0;
        let mut diagnostics = None;
        while !done.load(Ordering::Acquire) && Instant::now() < deadline {
            let requested = Instant::now();
            let _snapshot = handle.snapshot();
            let finished = Instant::now();
            if snapshots.len() < OBSERVER_LIMIT {
                snapshots.push(SnapshotSample {
                    requested,
                    finished,
                });
            } else {
                omitted += 1;
            }
            if diagnostics.is_none()
                && trace.active_stage.load(Ordering::Acquire) == Stage::OwnerSelection as usize
            {
                let submitted = Instant::now();
                let h = handle.clone();
                diagnostics = Some(runtime.spawn(async move {
                    let first_polled = Instant::now();
                    let value = h.diagnostics().await;
                    CallTiming {
                        submitted,
                        first_polled,
                        finished: Instant::now(),
                        status: format!("controller_active={}", value.active),
                    }
                }));
            }
            std::thread::sleep(Duration::from_micros(100));
        }
        // Caller does not block its Tokio worker waiting on this thread: it uses
        // spawn_blocking only after real compute and heartbeat sampling finish.
        let diagnostics =
            diagnostics.map(|task| runtime.block_on(task).expect("diagnostics task panicked"));
        Observation {
            snapshots,
            snapshot_omitted: omitted,
            diagnostics,
        }
    })
}

struct Beat {
    intended: Instant,
    actual: Instant,
    previous: Instant,
}
async fn heartbeat(done: Arc<AtomicBool>, ready: oneshot::Sender<()>) -> (Vec<Beat>, usize) {
    let mut values = Vec::with_capacity(HEARTBEAT_LIMIT);
    let mut omitted = 0;
    let mut previous = Instant::now();
    let mut intended = previous + HEARTBEAT_PERIOD;
    let _ = ready.send(());
    loop {
        tokio::time::sleep_until(tokio::time::Instant::from_std(intended)).await;
        let actual = Instant::now();
        if values.len() < HEARTBEAT_LIMIT {
            values.push(Beat {
                intended,
                actual,
                previous,
            });
        } else {
            omitted += 1;
        }
        if done.load(Ordering::Acquire) {
            break;
        }
        previous = actual;
        intended = actual + HEARTBEAT_PERIOD;
    }
    (values, omitted)
}

fn stage_json(records: &[Record]) -> serde_json::Value {
    let mut rows = Vec::new();
    for stage in [
        Stage::RuntimeInputCapture,
        Stage::OwnerSelection,
        Stage::OwnerRoutePreparation,
        Stage::OwnerRouteQueries,
        Stage::OwnerFinalization,
        Stage::OwnerCoverage,
    ] {
        let selected: Vec<_> = records.iter().filter(|r| r.stage == stage).collect();
        rows.push(serde_json::json!({"stage":format!("{stage:?}"),"count":selected.len(),
            "elapsed_total_ms":selected.iter().map(|r|r.elapsed_ms).sum::<f64>(),
            "elapsed_max_ms":selected.iter().map(|r|r.elapsed_ms).fold(0.0,f64::max),
            "explicit_copied_cell_payload_bytes":selected.iter().map(|r|r.copied_cell_bytes).sum::<usize>()}));
    }
    serde_json::Value::Array(rows)
}

async fn audit_calibration(handle: &ExploreRuntimeHandle, enabled: bool) {
    use nuwax_nav_types_messages::{NavigationCapabilities, RobotGeometry};
    let capabilities = NavigationCapabilities::from_env()
        .unwrap()
        .expect("supply attribution-calibration.env before launching test");
    assert_eq!(capabilities.calibration_id, CALIBRATION_ID);
    assert!(
        capabilities.observation.is_some(),
        "do not disable calibrated observation scoring"
    );
    let geometry = RobotGeometry::from_runtime_json(
        &std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").expect("calibration geometry missing"),
    )
    .unwrap();
    let guard = handle.inner.lock().await;
    let captured = guard
        .navigation_configuration
        .as_ref()
        .unwrap()
        .as_ref()
        .unwrap();
    assert_eq!(captured.capabilities, Some(capabilities.clone()));
    assert_eq!(captured.geometry, Some(geometry.clone()));
    assert_eq!(captured.base_frame, "base_link");
    assert_eq!(captured.body_to_support_z_m, Some(0.3));
    assert_eq!(guard.costmap_frame_generation, 1);
    assert_eq!(guard.mission_calibration.is_some(), enabled);
    let mission=guard.mission_calibration.as_ref().map(|calibration| {
        assert_eq!(calibration.planar.motion_limits,Some(capabilities.motion.clone()));
        assert_eq!(calibration.planar.robot_geometry,Some(geometry));
        let expected_speed=capabilities.motion.max_forward_mps.max(capabilities.motion.max_reverse_mps).hypot(capabilities.motion.max_lateral_mps);
        assert_eq!(calibration.maximum_sample_speed_mps,expected_speed);
        let defaults=nuwax_nav_simple_planner::SimplePlannerConfig::default();
        assert_eq!(calibration.planar.inflation_radius,defaults.inflation_radius);
        assert_eq!(calibration.planar.max_expansions,defaults.max_expansions);
        serde_json::json!({"maximum_sample_speed_mps":calibration.maximum_sample_speed_mps,
            "inflation_radius_m":calibration.planar.inflation_radius,"max_expansions":calibration.planar.max_expansions,
            "planar_full_config_debug":format!("{:?}",calibration.planar)})
    });
    eprintln!(
        "EXPLORE_ATTRIBUTION {}",
        serde_json::json!({"kind":"calibration_audit","navigation":captured,"mission":mission,
        "startup_path":"configured_handle + start_with_config; no direct calibration/controller replacement",
        "admission_fixture":"test_motion_admission; recording sink; no physical motion",
        "ingress":"committed snapshot in frame_generation 1; LCM and live scan ingress bypassed"})
    );
}

async fn start_case(
    label: &str,
    fixture: Fixture,
    return_budget: bool,
) -> (
    ExploreRuntimeHandle,
    Arc<RecordingNavSink>,
    Arc<Attribution>,
) {
    let (mut handle, sink) = configured_handle();
    let trace = Attribution::new();
    handle.attribution = Some(trace.clone());
    handle
        .ingest_committed_costmap_snapshot_in_frame(
            fixture.grid(false),
            committed_map_identity(1, "map"),
            1,
        )
        .await
        .unwrap();
    let config = ExploreStartConfig {
        motion_admission: crate::nav_runtime::test_motion_admission(),
        stop_conditions: ExploreStopConditions {
            no_frontier_timeout_ms: Some(600_000),
            ..Default::default()
        },
        return_policy: if return_budget {
            ReturnPolicy {
                maximum_distance_m: Some(100_000.0),
                reserve_distance_m: 1.0,
                return_on_completion: true,
            }
        } else {
            ReturnPolicy::default()
        },
        ..Default::default()
    };
    let started = handle.start_with_config(label, config).await;
    assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
    audit_calibration(&handle, return_budget).await;
    (handle, sink, trace)
}

async fn release_child_without_motion(handle: &ExploreRuntimeHandle) -> bool {
    let reached_before = {
        let guard = handle.inner.lock().await;
        if guard.active_child.is_none() {
            return false;
        }
        guard.reached_goals
    };
    // Explicit owner inspection occurs between measured ticks. It does not put a
    // full checkpoint/history clone into ordinary per-step committed projections.
    let before = handle
        .inspect_compute_checkpoint()
        .await
        .unwrap()
        .into_iter()
        .map(|source| (source.name, source.payload))
        .collect::<Vec<_>>();
    let lease = active_child_lease(handle).await;
    let value = handle.report_navigation_result(&lease.session_id, ChildNavigationResult::Failed {
        proof: terminal_proof(&lease), reason: "SafetyFailed cause=StaleInput: explicitly injected attribution fixture retirement; no motion occurred".into(),
    }).await.unwrap();
    assert_eq!(value.status, STATUS_ACTIVE);
    let after = handle
        .inspect_compute_checkpoint()
        .await
        .unwrap()
        .into_iter()
        .map(|source| (source.name, source.payload))
        .collect::<Vec<_>>();
    assert_eq!(
        before, after,
        "InputUnavailable retirement must not alter source history, blacklist or counters"
    );
    let guard = handle.inner.lock().await;
    assert_eq!(reached_before, guard.reached_goals);
    assert!(guard.active_child.is_none());
    true
}

async fn finish_cancel_and_check_stale_proofs(handle: &ExploreRuntimeHandle) {
    let snapshot = handle.snapshot();
    if snapshot.status != STATUS_STOPPING {
        assert!(snapshot.child_task_id.is_empty());
        return;
    }
    let (lease, cancel_generation) = {
        let guard = handle.inner.lock().await;
        (
            guard.active_child.as_ref().unwrap().lease.clone(),
            guard
                .pending_child_cancellation
                .as_ref()
                .unwrap()
                .ticket
                .as_ref()
                .unwrap()
                .cancel_generation,
        )
    };
    assert!(
        handle.blocks_mapping_mutation().await,
        "STOPPING acknowledgement is not a ZEROED terminal"
    );
    assert!(handle
        .report_navigation_result(
            &lease.session_id,
            ChildNavigationResult::Reached(terminal_proof(&lease))
        )
        .await
        .is_err());
    let mut stale = lease.clone();
    stale.goal_generation += 1;
    assert!(handle
        .report_navigation_result(
            &lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&stale, cancel_generation))
        )
        .await
        .is_err());
    assert!(handle
        .report_navigation_result(
            &lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&lease, cancel_generation + 1))
        )
        .await
        .is_err());
    assert_eq!(handle.snapshot().status, STATUS_STOPPING);
    let began = Instant::now();
    let terminal = handle
        .report_navigation_result(
            &lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&lease, cancel_generation)),
        )
        .await
        .unwrap();
    eprintln!(
        "EXPLORE_ATTRIBUTION {}",
        serde_json::json!({"kind":"injected_matching_zero_terminal","proof_handling_ms":ms(began.elapsed()),"status":terminal.status,
        "physical_zero_latency_measured":false})
    );
    assert!(terminal.child_task_id.is_empty());
    assert!(!handle.blocks_mapping_mutation().await);
}

async fn run_matrix_case(fixture: Fixture, return_budget: bool) {
    let label = format!("attribute-{}-budget-{return_budget}", fixture.label());
    let (handle, sink, trace) = start_case(&label, fixture, return_budget).await;
    let pose = GatedPose::new();
    let scheduler = handle.spawn_scheduler(pose.clone(), SCHEDULER_PERIOD);
    pose.wait_requested(1).await;
    for (phase_index, phase) in ["cold", "unchanged", "local_edit_8x8_occupied"]
        .into_iter()
        .enumerate()
    {
        let retirement = if phase_index > 0 {
            release_child_without_motion(&handle).await
        } else {
            false
        };
        let revision = if phase_index == 2 { 2 } else { 1 };
        let expected_map = fixture.grid(phase_index == 2);
        let changed = handle
            .ingest_committed_costmap_snapshot_in_frame(
                expected_map.clone(),
                committed_map_identity(revision, "map"),
                1,
            )
            .await
            .unwrap();
        assert_eq!(changed, phase_index == 2);
        let (before, omitted_before) = trace.snapshot();
        assert_eq!(omitted_before, 0);
        let done = Arc::new(AtomicBool::new(false));
        let observer = observe_phase(
            handle.clone(),
            trace.clone(),
            tokio::runtime::Handle::current(),
            done.clone(),
        );
        let (ready_tx, ready_rx) = oneshot::channel();
        let pulse = tokio::spawn(heartbeat(done.clone(), ready_tx));
        ready_rx.await.unwrap();
        let started = Instant::now();
        pose.release();
        pose.wait_requested(phase_index + 2).await;
        let wall_ms = ms(started.elapsed());
        // Let a delayed heartbeat record the starvation window before stopping it.
        tokio::time::sleep(HEARTBEAT_PERIOD).await;
        done.store(true, Ordering::Release);
        let (beats, beat_omitted) = pulse.await.unwrap();
        let observation = tokio::task::spawn_blocking(move || observer.join().unwrap())
            .await
            .unwrap();
        let (all, omitted) = trace.snapshot();
        assert_eq!(omitted, 0);
        let records = &all[before.len()..];
        assert_eq!(trace.active_stage.load(Ordering::Acquire), 0);
        assert!(
            records.iter().any(|r| r.stage == Stage::OwnerSelection),
            "real selection stage required"
        );
        if matches!(fixture, Fixture::FrontierBorderDerivative) && return_budget {
            assert!(
                records
                    .iter()
                    .any(|r| r.stage == Stage::OwnerRoutePreparation),
                "frontier budget case must actually prepare routes"
            );
            assert!(
                records.iter().any(|r| r.stage == Stage::OwnerRouteQueries),
                "frontier budget case must actually query routes"
            );
        }
        if !return_budget {
            assert!(!records.iter().any(|r| matches!(
                r.stage,
                Stage::OwnerRoutePreparation | Stage::OwnerRouteQueries
            )));
        }
        let state = handle.snapshot();
        if matches!(fixture, Fixture::ExactAllFreeBenchmark) {
            assert_eq!(state.status, STATUS_PENDING, "{}", state.message);
        }
        if matches!(fixture, Fixture::FrontierBorderDerivative) {
            assert_eq!(
                state.status, STATUS_ACTIVE,
                "fixture failed to reach a real goal: {}",
                state.message
            );
        }
        let goal_metadata = {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.costmap_input.as_deref(), Some(&expected_map));
            assert_eq!(guard.reached_goals, 0);
            guard
                .last_frontier_goal
                .as_ref()
                .map(|goal| goal.metadata.clone())
        };
        let snap_ms: Vec<_> = observation
            .snapshots
            .iter()
            .map(|v| ms(v.finished.duration_since(v.requested)))
            .collect();
        let overlapping_snapshots = observation
            .snapshots
            .iter()
            .filter(|v| {
                records
                    .iter()
                    .any(|r| intersects(v.requested, v.finished, r.started, r.finished))
            })
            .count();
        eprintln!(
            "EXPLORE_ATTRIBUTION {}",
            serde_json::json!({"kind":"scheduler_phase","fixture":fixture.label(),"phase":phase,
            "return_budget":return_budget,"runtime_flavor":"current_thread","runtime_workers":1,"compute_workers":1,"scheduler_period_ms":ms(SCHEDULER_PERIOD),
            "tick_and_next_pose_request_wall_ms":wall_ms,"state":state.status,"frontiers":state.frontier_count,
            "injected_input_unavailable_terminal_before_phase":retirement,"no_motion_or_reached_feedback":true,
            "stage_records":stage_json(records),"stage_records_omitted":omitted,"goal_metadata":goal_metadata,
            "incremental_detector_stats":"not exposed by runtime source diagnostics at this revision",
            "cached_snapshot_count":snap_ms.len(),"cached_snapshot_p50_ms":percentile(snap_ms.clone(),0.5),
            "cached_snapshot_p95_ms":percentile(snap_ms.clone(),0.95),"cached_snapshot_max_ms":percentile(snap_ms,1.0),
            "cached_snapshot_actual_overlap_count":overlapping_snapshots,"cached_snapshot_samples_omitted":observation.snapshot_omitted,
            "diagnostics":observation.diagnostics.as_ref().map(|v|v.json(records)),
            "heartbeat_period_ms":ms(HEARTBEAT_PERIOD),"heartbeat_count":beats.len(),"heartbeat_omitted":beat_omitted,
            "heartbeat_max_actual_gap_ms":beats.iter().map(|b|ms(b.actual.duration_since(b.previous))).fold(0.0,f64::max),
            "heartbeat_max_wake_lateness_ms":beats.iter().map(|b|ms(b.actual.saturating_duration_since(b.intended))).fold(0.0,f64::max),
            "submitted_child_count":sink.submitted.lock().unwrap().len()})
        );
    }
    let stop = handle.stop(&label).await;
    assert!(matches!(stop.status, STATUS_STOPPING | STATUS_CANCELLED));
    finish_cancel_and_check_stale_proofs(&handle).await;
    pose.release();
    scheduler.shutdown().await.unwrap();
}

#[derive(Clone, Copy, Debug)]
enum Action {
    Stop,
    ReplaceMap,
}
#[derive(Clone, Copy, Debug)]
enum ControlLane {
    SharedSchedulerRuntime,
    IndependentControlRuntime,
}

/// A request is timestamped on the observing OS thread before it is queued.
/// Independent control runs on another current-thread executor solely to separate
/// a synchronous Tokio-worker stall from the exploration mutex wait itself.
fn race_action(
    handle: ExploreRuntimeHandle,
    trace: Arc<Attribution>,
    runtime: tokio::runtime::Handle,
    target: Stage,
    action: Action,
    lane: ControlLane,
    label: String,
) -> std::thread::JoinHandle<Result<CallTiming, String>> {
    std::thread::spawn(move || {
        let control_runtime = matches!(lane, ControlLane::IndependentControlRuntime).then(|| {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
        });
        let deadline = Instant::now() + WAIT_LIMIT;
        while trace.active_stage.load(Ordering::Acquire) != target as usize {
            if Instant::now() >= deadline {
                return Err(format!("missed real stage {target:?}; no timing claim"));
            }
            std::thread::sleep(Duration::from_micros(50));
        }
        let submitted = Instant::now();
        let future = async move {
            let first_polled = Instant::now();
            let status = match action {
                Action::Stop => handle.stop(&label).await.status,
                Action::ReplaceMap => {
                    let mut identity = committed_map_identity(2, "map");
                    identity.session_id = "replacement-map-domain".into();
                    identity.session_generation = 2;
                    let changed = handle
                        .ingest_committed_costmap_snapshot_in_frame(
                            Fixture::FrontierBorderDerivative.grid(true),
                            identity,
                            2,
                        )
                        .await
                        .unwrap();
                    assert!(changed);
                    handle.snapshot().status
                }
            };
            CallTiming {
                submitted,
                first_polled,
                finished: Instant::now(),
                status: status.to_string(),
            }
        };
        Ok(match lane {
            ControlLane::SharedSchedulerRuntime => {
                let task = runtime.spawn(future);
                runtime
                    .block_on(task)
                    .expect("shared control request panicked")
            }
            ControlLane::IndependentControlRuntime => {
                control_runtime.as_ref().unwrap().block_on(future)
            }
        })
    })
}

async fn run_interference_case(target: Stage, action: Action, lane: ControlLane) {
    let label = format!("attribute-interference-{target:?}-{action:?}-{lane:?}");
    let (handle, sink, trace) = start_case(&label, Fixture::FrontierBorderDerivative, true).await;
    let pose = GatedPose::new();
    let scheduler = handle.spawn_scheduler(pose.clone(), SCHEDULER_PERIOD);
    pose.wait_requested(1).await;
    let action_task = race_action(
        handle.clone(),
        trace.clone(),
        tokio::runtime::Handle::current(),
        target,
        action,
        lane,
        label.clone(),
    );
    let done = Arc::new(AtomicBool::new(false));
    let (ready_tx, ready_rx) = oneshot::channel();
    let pulse = tokio::spawn(heartbeat(done.clone(), ready_tx));
    ready_rx.await.unwrap();
    pose.release();
    let call = tokio::task::spawn_blocking(move || action_task.join().unwrap())
        .await
        .unwrap()
        .expect(
            "required stage was not observed; rerun with truthful fixture, do not insert a hold",
        );
    // Drain real compute before inspecting final dispatch/cancellation outcomes.
    pose.release();
    scheduler.shutdown().await.unwrap();
    done.store(true, Ordering::Release);
    let (beats, omitted) = pulse.await.unwrap();
    let (records, dropped) = trace.snapshot();
    assert_eq!(dropped, 0);
    let submitted_in_target = records
        .iter()
        .any(|r| r.stage == target && r.started <= call.submitted && call.submitted < r.finished);
    let control_polled_in_target = records.iter().any(|r| {
        r.stage == target && r.started <= call.first_polled && call.first_polled < r.finished
    });
    let control_finished_in_target = records
        .iter()
        .any(|r| r.stage == target && r.started <= call.finished && call.finished < r.finished);
    eprintln!(
        "EXPLORE_ATTRIBUTION {}",
        serde_json::json!({"kind":"concurrent_request","action":format!("{action:?}"),"lane":format!("{lane:?}"),
        "target":format!("{target:?}"),"runtime_flavor":"current_thread","scheduler_workers":1,"compute_workers":1,
        "independent_control_workers":usize::from(matches!(lane,ControlLane::IndependentControlRuntime)),
        "request":call.json(&records),"requested_during_target_stage":submitted_in_target,"first_polled_during_target_stage":control_polled_in_target,
        "finished_during_target_stage":control_finished_in_target,"stage_records":stage_json(&records),
        "heartbeat_max_actual_gap_ms":beats.iter().map(|b|ms(b.actual.duration_since(b.previous))).fold(0.0,f64::max),"heartbeat_omitted":omitted})
    );
    assert!(
        submitted_in_target,
        "stage atomic was only a hint; this run missed actual interval overlap"
    );
    if matches!(target, Stage::OwnerSelection | Stage::OwnerRouteQueries)
        && control_finished_in_target
    {
        assert!(
            sink.submitted.lock().unwrap().is_empty(),
            "stale completed query must not dispatch after stop or map domain replacement"
        );
    }
    if matches!(action, Action::ReplaceMap) {
        let guard = handle.inner.lock().await;
        assert_eq!(
            guard.committed_map.as_ref().unwrap().session_id,
            "replacement-map-domain"
        );
        assert_eq!(guard.costmap_frame_generation, 2);
        assert!(!guard.logical_active);
        assert_eq!(guard.last_stop_reason, "map_revision_changed");
    } else {
        assert_eq!(handle.snapshot().last_stop_reason, "user_cancelled");
    }
    finish_cancel_and_check_stale_proofs(&handle).await;
    assert!(!handle.blocks_mapping_mutation().await);
}

#[test]
#[ignore = "manual calibrated 500x500 attribution; isolated process and companion env required"]
fn exploration_scheduler_stage_attribution_500x500() {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        for fixture in [
            Fixture::ExactAllFreeBenchmark,
            Fixture::FrontierBorderDerivative,
        ] {
            for return_budget in [false, true] {
                run_matrix_case(fixture, return_budget).await;
            }
        }
        run_interference_case(
            Stage::OwnerSelection,
            Action::Stop,
            ControlLane::SharedSchedulerRuntime,
        )
        .await;
        run_interference_case(
            Stage::OwnerSelection,
            Action::Stop,
            ControlLane::IndependentControlRuntime,
        )
        .await;
        run_interference_case(
            Stage::OwnerRouteQueries,
            Action::Stop,
            ControlLane::IndependentControlRuntime,
        )
        .await;
        run_interference_case(
            Stage::OwnerRouteQueries,
            Action::ReplaceMap,
            ControlLane::IndependentControlRuntime,
        )
        .await;
    });
}
