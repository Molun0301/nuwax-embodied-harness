use super::*;

use futures::{FutureExt, StreamExt};
use nalgebra::Point3;
use nuwax_exploration::ExplorationBoundary;
use nuwax_nav_stack::{
    register_compiled_in_process_owners, NavStackBuilder, NavStackPreset, NavStackProfile,
    NavTopicConfig, StackRunner,
};
use nuwax_protocol::RawLcmFrame;
use std::collections::VecDeque;
use std::ffi::OsString;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::sync::Mutex;

static ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

struct EnvRestore(Vec<(&'static str, Option<OsString>)>);

impl EnvRestore {
    fn set(values: &[(&'static str, &str)]) -> Self {
        let saved = values
            .iter()
            .map(|(key, _)| (*key, std::env::var_os(key)))
            .collect();
        for (key, value) in values {
            std::env::set_var(key, value);
        }
        Self(saved)
    }
}

impl Drop for EnvRestore {
    fn drop(&mut self) {
        for (key, value) in self.0.drain(..) {
            match value {
                Some(value) => std::env::set_var(key, value),
                None => std::env::remove_var(key),
            }
        }
    }
}

const GUARD_CHILD: &str = r#"
import os
import signal
import sys
import threading
import uuid
from pathlib import Path

from services.navigation.motion_safety_guard.actor import GuardActor
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.epoch import (
    EpochRecord,
    EpochStore,
    ProvisionRecord,
    atomic_write_record,
)
from services.navigation.motion_safety_guard.server import MotionGuardUnixServer

root = Path(sys.argv[1])
socket_path = root / "guard.sock"
events_path = root / "actuator.events"
digest = "sha256:" + "4" * 64

class RecordingActuator:
    is_configured = True
    actuator_symbols_sha256 = digest

    def _record(self, value):
        with events_path.open("a", encoding="ascii") as stream:
            stream.write(value + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def apply_motion(self, request):
        if request.HasField("twist"):
            twist = request.twist
            self._record(f"TWIST {twist.linear_x_mps} {twist.linear_y_mps} {twist.angular_z_rps}")
        else:
            self._record("MOTION")

    def execute_sport(self, command):
        self._record("SPORT")

    def zero(self):
        self._record("ZERO")

    def switch_mode(self, command):
        self._record("MODE")
        return command.target_mode

provision_path = root / "provision.json"
epoch_path = root / "epoch.json"
atomic_write_record(
    provision_path,
    ProvisionRecord.create(instance_id=str(uuid.uuid4())).to_document(),
    mode=0o400,
)
atomic_write_record(
    epoch_path,
    EpochRecord.initial(actuator_symbols_sha256=digest).to_document(),
    mode=0o600,
)
store = EpochStore(provision_path=provision_path, epoch_path=epoch_path)
epoch = store.advance_boot()
core = MotionGuardCore(
    epoch=epoch,
    epoch_store=store,
    actuator=RecordingActuator(),
)
core.initialize_zero_fence()
server = MotionGuardUnixServer(
    socket_path=socket_path,
    actor=GuardActor(core),
    socket_gid=os.getegid(),
)
stop = threading.Event()
signal.signal(signal.SIGTERM, lambda _signum, _frame: stop.set())
signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())
server.start()
print("READY", flush=True)
try:
    while not stop.wait(0.05):
        pass
finally:
    server.stop()
"#;

struct GuardChild {
    child: Child,
    root: PathBuf,
    socket_path: PathBuf,
    events_path: PathBuf,
}

impl GuardChild {
    async fn spawn(nonce: &str) -> Self {
        let root = std::env::temp_dir().join(format!("nuwax-v2-guard-{nonce}"));
        std::fs::create_dir(&root).expect("create guard canary directory");
        let socket_path = root.join("guard.sock");
        let events_path = root.join("actuator.events");
        let repository_root =
            env!("CARGO_MANIFEST_DIR").replace("/services/geometry/nuwax_geometry_runtime", "");
        let python = PathBuf::from(&repository_root).join(".venv_model_download/bin/python");
        assert!(
            python.is_file(),
            "Guard canary Python environment is missing"
        );
        let child = Command::new(python)
            .args(["-u", "-c", GUARD_CHILD])
            .arg(&root)
            .current_dir(repository_root)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawn production Motion Guard child");
        let mut guard = Self {
            child,
            root,
            socket_path,
            events_path,
        };
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                if guard.socket_path.exists() {
                    break;
                }
                if let Some(status) = guard.child.try_wait().expect("poll Guard child") {
                    use std::io::Read;
                    let mut stderr = String::new();
                    if let Some(mut pipe) = guard.child.stderr.take() {
                        let _ = pipe.read_to_string(&mut stderr);
                    }
                    panic!("Motion Guard child exited before READY: {status}; stderr={stderr}");
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .expect("Motion Guard child READY timeout");
        guard
    }

    fn actuator_events(&self) -> String {
        std::fs::read_to_string(&self.events_path).expect("read recording actuator events")
    }

    async fn shutdown(mut self) {
        let pid = self.child.id().to_string();
        let status = Command::new("kill")
            .args(["-TERM", &pid])
            .status()
            .expect("signal Motion Guard child");
        assert!(status.success(), "SIGTERM Motion Guard child");
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                if let Some(status) = self.child.try_wait().expect("reap Motion Guard child") {
                    assert!(status.success(), "Motion Guard child status: {status}");
                    break;
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .expect("Motion Guard child reap timeout");
        assert!(!self.socket_path.exists(), "Guard socket must be unlinked");
        std::fs::remove_dir_all(&self.root).expect("remove Guard canary state");
        assert!(!self.root.exists(), "Guard canary state must be removed");
    }
}

impl Drop for GuardChild {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
        let _ = std::fs::remove_dir_all(&self.root);
    }
}

#[derive(Default)]
struct FailClosedNavSink {
    submissions: Mutex<Vec<crate::explore_runtime::ChildNavigationRequest>>,
}

#[async_trait]
impl crate::explore_runtime::ExploreNavigationSink for FailClosedNavSink {
    async fn submit(
        &self,
        request: crate::explore_runtime::ChildNavigationRequest,
    ) -> Result<
        crate::explore_runtime::ChildNavigationTicket,
        crate::explore_runtime::ChildNavigationSubmitError,
    > {
        self.submissions.lock().expect("submissions").push(request);
        panic!("out-of-boundary frontier must not reach navigation")
    }

    async fn cancel(
        &self,
        _lease: &crate::explore_runtime::ChildNavigationLease,
    ) -> Result<crate::explore_runtime::ChildNavigationCancelTicket, String> {
        panic!("fail-closed case must not own child navigation")
    }
}

fn typed_frame(topic: &str, message_type: &str, payload: Vec<u8>, seqno: u64) -> RawLcmFrame {
    crate::typed_codec::build_typed_frame(
        LcmTopic::new(topic).expect("canary topic"),
        LcmMessageType::new(message_type).expect("canary message type"),
        payload,
        seqno,
    )
}

fn canary_mapping() -> MappingAdapter {
    let mut adapter = MappingAdapter::new_with_frames(
        MappingAdapterConfig {
            resolution: 0.5,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-5.0, -5.0),
            voxel_size: 0.05,
            initial_safe_radius_meters: 2.0,
        },
        "map",
        "odom",
    );
    adapter
        .start_session("v2-canary-mapping", "v2-canary-map")
        .expect("start V2 canary mapping session");
    adapter
}

fn simple_planner_runner(topics: &NavTopicConfig) -> StackRunner {
    let graph = NavStackBuilder::with_topics(NavStackProfile::Simple2D, topics.clone())
        .preset(NavStackPreset::None)
        .build()
        .expect("build production Simple2D graph");
    assert_eq!(
        graph
            .nodes()
            .iter()
            .map(|node| node.id.as_str())
            .collect::<Vec<_>>(),
        vec![
            "map_pose_adapter",
            "simple_planner",
            "guarded_nav_scheduler"
        ]
    );
    let mut runner = StackRunner::from_graph(graph);
    register_compiled_in_process_owners(&mut runner);
    runner
}

async fn next_typed(
    stream: &mut nuwax_transport::LcmFrameStream,
    timeout: Duration,
) -> RawLcmFrame {
    tokio::time::timeout(timeout, stream.next())
        .await
        .expect("timed out waiting for V2 canary frame")
        .expect("V2 canary subscription closed")
        .expect("V2 canary transport error")
}

#[derive(Clone)]
struct PathLedgerEntry {
    ordinal: u64,
    elapsed_ns: u128,
    path: schema::PathEnvelope,
}

#[derive(Clone)]
struct NavStateLedgerEntry {
    ordinal: u64,
    elapsed_ns: u128,
    state: schema::NavigationState,
    feedback_applied: bool,
}

const NAV_STATE_EVIDENCE_CAPACITY: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NavCriticalEvidenceKind {
    SentinelReceipt,
    FreshReceipt,
    FirstActive,
    PathAccepted,
    PathAcceptedAndTwistApplied,
    UnexpectedPathAccepted,
    UnexpectedPathAcceptedAndTwistApplied,
    TwistApplied,
    Terminal,
    PostTerminalFirstIdle,
    UnexpectedState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NavRoutineEvidenceKind {
    PreGateIdleHeartbeat,
    AwaitingPathRefresh,
    ActiveInputRefresh,
    PostTerminalIdleRepeat,
    FailureDrain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NavEvidenceKind {
    Critical(NavCriticalEvidenceKind),
    Routine(NavRoutineEvidenceKind),
}

struct NavStateEvidenceRun {
    kind: NavEvidenceKind,
    first: NavStateLedgerEntry,
    latest: NavStateLedgerEntry,
    observed_count: u64,
}

struct NavStateEvidenceLedger {
    runs: VecDeque<NavStateEvidenceRun>,
    retained_count: usize,
    compacted_count: u64,
}

impl NavStateEvidenceLedger {
    fn new() -> Self {
        Self {
            runs: VecDeque::with_capacity(NAV_STATE_EVIDENCE_CAPACITY),
            retained_count: 0,
            compacted_count: 0,
        }
    }

    fn record(&mut self, kind: NavEvidenceKind, entry: NavStateLedgerEntry) {
        let extends_routine_run = matches!(kind, NavEvidenceKind::Routine(_))
            && self.runs.back().is_some_and(|run| run.kind == kind);
        if extends_routine_run {
            let retains_latest = self.runs.back().is_some_and(|run| run.observed_count == 1);
            if retains_latest {
                self.reserve_retained(kind, entry.ordinal, 1);
                self.retained_count += 1;
            } else {
                self.compacted_count = self
                    .compacted_count
                    .checked_add(1)
                    .expect("V2 NavigationState compacted count overflow");
            }
            let run = self.runs.back_mut().expect("routine run must exist");
            run.observed_count = run
                .observed_count
                .checked_add(1)
                .expect("V2 NavigationState evidence observation overflow");
            run.latest = entry;
            return;
        }

        self.reserve_retained(kind, entry.ordinal, 1);
        self.retained_count += 1;
        self.runs.push_back(NavStateEvidenceRun {
            kind,
            first: entry.clone(),
            latest: entry,
            observed_count: 1,
        });
    }

    fn reserve_retained(&self, kind: NavEvidenceKind, ordinal: u64, additional: usize) {
        if self.retained_count.saturating_add(additional) > NAV_STATE_EVIDENCE_CAPACITY {
            let all_retained_critical = self
                .runs
                .iter()
                .all(|run| matches!(run.kind, NavEvidenceKind::Critical(_)));
            panic!(
                "V2 NavigationState evidence overflow: capacity={} retained={} compacted={} all_retained_critical={} incoming_kind={kind:?} incoming_ordinal={ordinal}",
                NAV_STATE_EVIDENCE_CAPACITY,
                self.retained_count,
                self.compacted_count,
                all_retained_critical
            );
        }
    }

    fn retained_entries(&self) -> Vec<&NavStateLedgerEntry> {
        let mut retained = Vec::with_capacity(self.retained_count);
        for run in &self.runs {
            retained.push(&run.first);
            if run.observed_count > 1 {
                retained.push(&run.latest);
            }
        }
        retained
    }

    fn dump(&self, prefix: &str) {
        eprintln!(
            "{prefix}_SUMMARY retained={} compacted={} runs={} capacity={}",
            self.retained_count,
            self.compacted_count,
            self.runs.len(),
            NAV_STATE_EVIDENCE_CAPACITY
        );
        for run in &self.runs {
            eprintln!(
                "{prefix} kind={:?} observed={} first_ordinal={} first_elapsed_ns={} latest_ordinal={} latest_elapsed_ns={} first_state={} latest_state={} first_session={:?} latest_session={:?} first_path_revision={} latest_path_revision={} first_current={:?} latest_current={:?} first_message={:?} latest_message={:?} first_zero_ack={} latest_zero_ack={}",
                run.kind,
                run.observed_count,
                run.first.ordinal,
                run.first.elapsed_ns,
                run.latest.ordinal,
                run.latest.elapsed_ns,
                run.first.state.state,
                run.latest.state.state,
                run.first.state.session_id,
                run.latest.state.session_id,
                run.first.state.path_revision,
                run.latest.state.path_revision,
                run.first.state.current,
                run.latest.state.current,
                run.first.state.message,
                run.latest.state.message,
                run.first.state.zero_ack_id,
                run.latest.state.zero_ack_id,
            );
        }
    }
}

#[derive(Clone)]
enum PathAcceptanceOutcome {
    ExpectedAccepted(PathLedgerEntry),
    TerminalBeforeExpectedAcceptance(NavStateLedgerEntry),
    UnexpectedAccepted(NavStateLedgerEntry),
}

#[derive(Clone)]
enum AcceptedPathCandidateMatch {
    Matched(PathLedgerEntry),
    Pending,
    Conflict,
}

fn path_matches_active_goal(path: &schema::PathEnvelope, goal: &schema::GoalEnvelope) -> bool {
    path.session_id == goal.session_id
        && path.owner_kind == goal.owner_kind
        && path.parent_request_id == goal.parent_request_id
        && path.goal_generation == goal.goal_generation
        && path.producer == goal.producer
        && path.producer_epoch == goal.producer_epoch
        && path.producer_runtime_generation == goal.producer_runtime_generation
        && path.committed_map_session_id == goal.committed_map_session_id
        && path.committed_map_session_generation == goal.committed_map_session_generation
        && path.committed_map_revision == goal.committed_map_revision
        && path.committed_map_boot_nonce == goal.committed_map_boot_nonce
        && path.committed_map_commit_sequence == goal.committed_map_commit_sequence
        && path.committed_map_source_timestamp_ns == goal.committed_map_source_timestamp_ns
        && path.committed_map_frame_id == goal.committed_map_frame_id
        && path.boundary_id == goal.boundary_id
        && path.motion_guard_generation == goal.motion_guard_generation
        && path.motion_guard_boot_nonce == goal.motion_guard_boot_nonce
        && path.motion_stop_generation == goal.motion_stop_generation
        && path.producer_runtime_generation == goal.producer_runtime_generation
}

fn is_canonical_path_candidate(entry: &PathLedgerEntry, goal: &schema::GoalEnvelope) -> bool {
    let path = &entry.path;
    path_matches_active_goal(path, goal)
        && path.planner_runtime_generation > 0
        && path.path_revision > 0
        && schema::validate_path_envelope(path).is_ok()
}

fn accepted_state_matches_active_goal(
    state: &schema::NavigationState,
    goal: &schema::GoalEnvelope,
) -> bool {
    state.session_id == goal.session_id
        && state.owner_kind == goal.owner_kind
        && state.parent_request_id == goal.parent_request_id
        && state.goal_generation == goal.goal_generation
        && state.producer_runtime_generation == goal.producer_runtime_generation
}

fn match_accepted_path_candidate(
    state: &schema::NavigationState,
    goal: &schema::GoalEnvelope,
    candidates: &VecDeque<PathLedgerEntry>,
) -> AcceptedPathCandidateMatch {
    if state.state != 2
        || state.path_revision <= 0
        || !accepted_state_matches_active_goal(state, goal)
    {
        return AcceptedPathCandidateMatch::Conflict;
    }

    let mut conflicting_candidate = false;
    for entry in candidates {
        let path = &entry.path;
        let exact = path.session_id == state.session_id
            && path.owner_kind == state.owner_kind
            && path.parent_request_id == state.parent_request_id
            && path.goal_generation == state.goal_generation
            && path.producer_runtime_generation == state.producer_runtime_generation
            && path.path_revision == state.path_revision
            && path.planner_runtime_generation == state.planner_runtime_generation;
        if exact {
            return AcceptedPathCandidateMatch::Matched(entry.clone());
        }
        if path.path_revision == state.path_revision
            || (path.planner_runtime_generation == state.planner_runtime_generation && path.path_revision > state.path_revision)
        {
            conflicting_candidate = true;
        }
    }

    if conflicting_candidate {
        AcceptedPathCandidateMatch::Conflict
    } else {
        AcceptedPathCandidateMatch::Pending
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PendingAcceptedStateUpdate {
    Duplicate,
    Advance,
    Conflict,
}

fn classify_pending_accepted_state_update(
    pending: &schema::NavigationState,
    incoming: &schema::NavigationState,
    goal: &schema::GoalEnvelope,
) -> PendingAcceptedStateUpdate {
    if incoming.state != 2
        || incoming.path_revision <= 0
        || !accepted_state_matches_active_goal(incoming, goal)
        || incoming.path_revision < pending.path_revision
    {
        return PendingAcceptedStateUpdate::Conflict;
    }

    let duplicate = incoming.session_id == pending.session_id
        && incoming.owner_kind == pending.owner_kind
        && incoming.parent_request_id == pending.parent_request_id
        && incoming.goal_generation == pending.goal_generation
        && incoming.producer_runtime_generation == pending.producer_runtime_generation
        && incoming.path_revision == pending.path_revision
        && incoming.planner_runtime_generation == pending.planner_runtime_generation;
    if duplicate {
        PendingAcceptedStateUpdate::Duplicate
    } else if incoming.path_revision == pending.path_revision {
        PendingAcceptedStateUpdate::Conflict
    } else {
        PendingAcceptedStateUpdate::Advance
    }
}

fn terminal_before_expected_path_acceptance(state: &schema::NavigationState) -> bool {
    matches!(state.state, 3 | 4)
}

fn accepted_path_evidence_kind(
    expected: bool,
    state: &schema::NavigationState,
) -> NavCriticalEvidenceKind {
    match (expected, state_reports_twist_applied(state)) {
        (true, true) => NavCriticalEvidenceKind::PathAcceptedAndTwistApplied,
        (true, false) => NavCriticalEvidenceKind::PathAccepted,
        (false, true) => NavCriticalEvidenceKind::UnexpectedPathAcceptedAndTwistApplied,
        (false, false) => NavCriticalEvidenceKind::UnexpectedPathAccepted,
    }
}

fn state_reports_twist_applied(state: &schema::NavigationState) -> bool {
    state.message.to_ascii_lowercase().contains("twist applied")
}

fn is_canonical_idle(state: &schema::NavigationState) -> bool {
    state.state == 0
        && has_empty_navigation_identity(state)
        && state.message == "IDLE: guard client heartbeat"
}

async fn apply_canary_idle_feedback(
    nav: &crate::nav_runtime::NavRuntimeState,
    payload: &[u8],
) -> bool {
    let Ok(state) = schema::decode_navigation_state_body(payload) else {
        return false;
    };
    if !is_canonical_idle(&state) || state.sink_ready != 1 {
        return false;
    }
    let Some(update) = crate::typed_codec::decode_nav_state_payload(payload) else {
        return false;
    };
    apply_typed_navigation_feedback(nav, update).await.is_some()
}

fn canary_tf_payload_for_edges(stamp_ns: u64, edges: &[(&str, &str)]) -> Vec<u8> {
    let edge = |parent: &str, child: &str| schema::TransformStamped {
        header: schema::Header {
            stamp_sec: (stamp_ns / 1_000_000_000) as i32,
            stamp_nsec: (stamp_ns % 1_000_000_000) as i32,
            frame_id: parent.to_string(),
            ..schema::Header::default()
        },
        child_frame_id: child.to_string(),
        transform: schema::Transform {
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            ..schema::Transform::default()
        },
    };
    schema::encode_tf_message_body(&schema::TfMessage {
        transforms: edges
            .iter()
            .map(|(parent, child)| edge(parent, child))
            .collect(),
    })
    .expect("encode canary TF")
}

fn canary_tf_payload(stamp_ns: u64) -> Vec<u8> {
    canary_tf_payload_for_edges(stamp_ns, &[("map", "odom"), ("odom", "base_link")])
}

fn apply_canary_tf_payload(
    nav: &crate::nav_runtime::NavRuntimeState,
    history: &mut PublicTfHistory,
    generation: &mut u64,
    payload: &[u8],
) -> bool {
    let Some(entries) = parse_tf_payload(payload) else {
        return false;
    };
    if !history.merge(entries) {
        return false;
    }
    let observed_entries = history.entries();
    let base_candidates = semantic_base_candidates(&observed_entries, "base_link");
    let map_to_odom = resolve_transform(&observed_entries, "odom", "map").is_some();
    let odom_to_base = base_candidates
        .iter()
        .any(|base| resolve_transform(&observed_entries, base, "odom").is_some());
    let next_generation = generation.saturating_add(1);
    if nav
        .observe_tf_chain(crate::nav_runtime::TfChainObservation {
            generation: next_generation,
            map_frame: normalize_frame("map"),
            odom_frame: normalize_frame("odom"),
            base_frame: normalize_frame("base_link"),
            map_to_odom,
            odom_to_base,
        })
        .is_err()
    {
        return false;
    }
    *generation = next_generation;
    true
}

fn canary_tf_window_complete(history: &mut PublicTfHistory, payload: &[u8]) -> bool {
    let Some(entries) = parse_tf_payload(payload) else {
        return false;
    };
    if !history.merge(entries) {
        return false;
    }
    let entries = history.entries();
    let bases = semantic_base_candidates(&entries, "base_link");
    resolve_transform(&entries, "odom", "map").is_some()
        && bases
            .iter()
            .any(|base| resolve_transform(&entries, base, "odom").is_some())
}

#[derive(Debug, Clone, Copy)]
struct PostQuietTfGate {
    baseline_count: u64,
    baseline_generation: u64,
    fresh_complete_payload_applied: bool,
}

impl PostQuietTfGate {
    fn observe_payload_result(&mut self, applied: bool, proof_window_complete: bool) {
        self.fresh_complete_payload_applied |= applied && proof_window_complete;
    }

    fn allows_dispatch(&self, detail: &crate::nav_runtime::NavigationReadinessDetail) -> bool {
        self.fresh_complete_payload_applied
            && detail.tf_count > self.baseline_count
            && detail.tf_generation > self.baseline_generation
            && detail.ready
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct InitialReadinessDispatchLedger {
    goal: usize,
    path: usize,
    twist: usize,
}

#[derive(Debug)]
struct InitialReadinessGateOutcome {
    detail: crate::nav_runtime::NavigationReadinessDetail,
    dispatches: InitialReadinessDispatchLedger,
    release_count: usize,
}

#[derive(Debug)]
struct InitialReadinessGateFailure {
    reason: String,
    detail: crate::nav_runtime::NavigationReadinessDetail,
    dispatches: InitialReadinessDispatchLedger,
    release_count: usize,
    cleanup_required: bool,
}

impl InitialReadinessGateFailure {
    fn render(&self) -> String {
        format!(
            "{}: dispatches={:?} release_count={} cleanup_required={} detail={:#?}",
            self.reason, self.dispatches, self.release_count, self.cleanup_required, self.detail
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitialReadinessDeadlineCheckpoint {
    OuterBeforeReadiness,
    OuterAfterReadiness,
    NavDrain,
    AfterNavDrain,
    BeforeGoalPoll,
    BeforePathPoll,
    BeforeGuardRead,
    BeforeFinalReadiness,
    AfterFinalReadiness,
}

fn initial_readiness_timeout_failure(
    detail: crate::nav_runtime::NavigationReadinessDetail,
    dispatches: InitialReadinessDispatchLedger,
) -> InitialReadinessGateFailure {
    InitialReadinessGateFailure {
        reason: "initial navigation readiness timeout".to_string(),
        detail,
        dispatches,
        release_count: 0,
        cleanup_required: true,
    }
}

fn initial_readiness_deadline_expired(
    deadline: tokio::time::Instant,
    checkpoint: InitialReadinessDeadlineCheckpoint,
    deadline_checkpoint: &mut dyn FnMut(InitialReadinessDeadlineCheckpoint) -> bool,
) -> bool {
    tokio::time::Instant::now() >= deadline || deadline_checkpoint(checkpoint)
}

async fn await_initial_navigation_readiness(
    readiness_handle: &GeometryRuntimeHandle,
    nav: &crate::nav_runtime::NavRuntimeState,
    nav_state_ledger_rx: &mut tokio::sync::mpsc::Receiver<NavStateLedgerEntry>,
    nav_state_ledger: &mut NavStateEvidenceLedger,
    goal_frames: &mut nuwax_transport::LcmFrameStream,
    path_ledger_rx: &mut tokio::sync::mpsc::Receiver<PathLedgerEntry>,
    guard_events: &mut dyn FnMut() -> String,
    deadline: tokio::time::Instant,
    deadline_checkpoint: &mut dyn FnMut(InitialReadinessDeadlineCheckpoint) -> bool,
) -> Result<InitialReadinessGateOutcome, InitialReadinessGateFailure> {
    let mut dispatches = InitialReadinessDispatchLedger::default();
    loop {
        let deadline_expired_before_readiness = initial_readiness_deadline_expired(
            deadline,
            InitialReadinessDeadlineCheckpoint::OuterBeforeReadiness,
            deadline_checkpoint,
        );
        let detail = crate::nav_runtime::compute_readiness(readiness_handle, nav).await;
        if deadline_expired_before_readiness
            || initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::OuterAfterReadiness,
                deadline_checkpoint,
            )
        {
            return Err(initial_readiness_timeout_failure(detail, dispatches));
        }
        if initial_readiness_deadline_expired(
            deadline,
            InitialReadinessDeadlineCheckpoint::BeforeGuardRead,
            deadline_checkpoint,
        ) {
            return Err(initial_readiness_timeout_failure(detail, dispatches));
        }
        dispatches.twist = guard_events()
            .lines()
            .filter(|line| line.starts_with("TWIST "))
            .count();
        if dispatches.twist != 0 {
            return Err(InitialReadinessGateFailure {
                reason: "motion dispatched before initial readiness".to_string(),
                detail,
                dispatches,
                release_count: 0,
                cleanup_required: true,
            });
        }
        if detail.ready {
            let mut nav_drain_budget = NAV_STATE_EVIDENCE_CAPACITY;
            loop {
                if initial_readiness_deadline_expired(
                    deadline,
                    InitialReadinessDeadlineCheckpoint::NavDrain,
                    deadline_checkpoint,
                ) {
                    return Err(initial_readiness_timeout_failure(detail, dispatches));
                }
                match nav_state_ledger_rx.try_recv() {
                    Ok(entry) => {
                        if !is_canonical_idle(&entry.state) {
                            let reason = format!(
                                "noncanonical NavigationState before initial readiness: ordinal={} state={} session={:?} owner={} parent={:?} goal_generation={} path_revision={} message={:?}",
                                entry.ordinal,
                                entry.state.state,
                                entry.state.session_id,
                                entry.state.owner_kind,
                                entry.state.parent_request_id,
                                entry.state.goal_generation,
                                entry.state.path_revision,
                                entry.state.message,
                            );
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState),
                                entry,
                            );
                            return Err(InitialReadinessGateFailure {
                                reason,
                                detail,
                                dispatches,
                                release_count: 0,
                                cleanup_required: true,
                            });
                        }
                        let kind = if entry.feedback_applied {
                            NavEvidenceKind::Routine(NavRoutineEvidenceKind::PreGateIdleHeartbeat)
                        } else {
                            NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                        };
                        nav_state_ledger.record(kind, entry);
                        if nav_drain_budget == 0 {
                            return Err(InitialReadinessGateFailure {
                                reason: "NavigationState activity flood before initial readiness"
                                    .to_string(),
                                detail,
                                dispatches,
                                release_count: 0,
                                cleanup_required: true,
                            });
                        }
                        nav_drain_budget -= 1;
                    }
                    Err(tokio::sync::mpsc::error::TryRecvError::Empty) => break,
                    Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => {
                        return Err(InitialReadinessGateFailure {
                            reason: "V2 NavigationState ledger closed before initial readiness"
                                .to_string(),
                            detail,
                            dispatches,
                            release_count: 0,
                            cleanup_required: true,
                        });
                    }
                }
            }
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::AfterNavDrain,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::BeforeGoalPoll,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            if let Some(frame) = goal_frames.next().now_or_never() {
                let reason = match frame {
                    Some(Ok(_)) => {
                        dispatches.goal += 1;
                        "goal dispatched before initial readiness"
                    }
                    Some(Err(_)) => "goal transport error before initial readiness",
                    None => "goal subscription closed before initial readiness",
                };
                return Err(InitialReadinessGateFailure {
                    reason: reason.to_string(),
                    detail,
                    dispatches,
                    release_count: 0,
                    cleanup_required: true,
                });
            }
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::BeforePathPoll,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            match path_ledger_rx.try_recv() {
                Ok(_) => {
                    dispatches.path += 1;
                    return Err(InitialReadinessGateFailure {
                        reason: "path dispatched before initial readiness".to_string(),
                        detail,
                        dispatches,
                        release_count: 0,
                        cleanup_required: true,
                    });
                }
                Err(tokio::sync::mpsc::error::TryRecvError::Empty) => {}
                Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => {
                    return Err(InitialReadinessGateFailure {
                        reason: "PathEnvelope ledger closed before initial readiness".to_string(),
                        detail,
                        dispatches,
                        release_count: 0,
                        cleanup_required: true,
                    });
                }
            }
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::BeforeGuardRead,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            dispatches.twist = guard_events()
                .lines()
                .filter(|line| line.starts_with("TWIST "))
                .count();
            if dispatches.twist != 0 {
                return Err(InitialReadinessGateFailure {
                    reason: "motion dispatched before initial readiness".to_string(),
                    detail,
                    dispatches,
                    release_count: 0,
                    cleanup_required: true,
                });
            }
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::BeforeFinalReadiness,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            let linearized_detail =
                crate::nav_runtime::compute_readiness(readiness_handle, nav).await;
            if initial_readiness_deadline_expired(
                deadline,
                InitialReadinessDeadlineCheckpoint::AfterFinalReadiness,
                deadline_checkpoint,
            ) {
                return Err(initial_readiness_timeout_failure(
                    linearized_detail,
                    dispatches,
                ));
            }
            if linearized_detail.ready {
                return Ok(InitialReadinessGateOutcome {
                    detail: linearized_detail,
                    dispatches,
                    release_count: 1,
                });
            }
            continue;
        }
        tokio::select! {
            biased;
            _ = tokio::time::sleep_until(deadline) => {
                return Err(initial_readiness_timeout_failure(detail, dispatches));
            }
            entry = nav_state_ledger_rx.recv() => {
                let Some(entry) = entry else {
                    return Err(InitialReadinessGateFailure {
                        reason: "V2 NavigationState ledger closed before initial readiness".to_string(),
                        detail,
                        dispatches,
                        release_count: 0,
                cleanup_required: true,
                    });
                };
                if !is_canonical_idle(&entry.state) {
                    let reason = format!(
                        "noncanonical NavigationState before initial readiness: ordinal={} state={} session={:?} owner={} parent={:?} goal_generation={} path_revision={} message={:?}",
                        entry.ordinal,
                        entry.state.state,
                        entry.state.session_id,
                        entry.state.owner_kind,
                        entry.state.parent_request_id,
                        entry.state.goal_generation,
                        entry.state.path_revision,
                        entry.state.message,
                    );
                    nav_state_ledger.record(
                        NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState),
                        entry,
                    );
                    return Err(InitialReadinessGateFailure {
                        reason,
                        detail,
                        dispatches,
                        release_count: 0,
                cleanup_required: true,
                    });
                }
                let kind = if entry.feedback_applied {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::PreGateIdleHeartbeat)
                } else {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                };
                nav_state_ledger.record(kind, entry);
            }
            frame = goal_frames.next() => {
                let reason = match frame {
                    Some(Ok(_)) => {
                        dispatches.goal += 1;
                        "goal dispatched before initial readiness"
                    }
                    Some(Err(_)) => "goal transport error before initial readiness",
                    None => "goal subscription closed before initial readiness",
                };
                return Err(InitialReadinessGateFailure {
                    reason: reason.to_string(),
                    detail,
                    dispatches,
                    release_count: 0,
                cleanup_required: true,
                });
            }
            entry = path_ledger_rx.recv() => {
                let reason = if entry.is_some() {
                    dispatches.path += 1;
                    "path dispatched before initial readiness"
                } else {
                    "PathEnvelope ledger closed before initial readiness"
                };
                return Err(InitialReadinessGateFailure {
                    reason: reason.to_string(),
                    detail,
                    dispatches,
                    release_count: 0,
                cleanup_required: true,
                });
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LiveInputPhase {
    Sentinel,
    Fresh,
}

#[derive(Clone)]
struct LiveInputPublishLedgerEntry {
    phase: LiveInputPhase,
    sequence: u64,
    timestamp_ns: u64,
    pose: schema::Pose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LiveInputKind {
    Pose,
    Costmap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LiveInputOutstanding {
    sequence: u64,
    timestamp_ns: u64,
    published_at: tokio::time::Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LiveInputReceiptLedgerEntry {
    kind: LiveInputKind,
    publication: LiveInputOutstanding,
    receipt_ordinal: Option<u64>,
    receipt_at: Option<tokio::time::Instant>,
    superseded_by: Option<u64>,
    next_publication: Option<LiveInputOutstanding>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LiveInputAckWake {
    pose: bool,
    costmap: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LiveInputTrigger {
    BaseTick,
    Deadline,
    Ack(Option<LiveInputKind>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LiveInputPublishPlan {
    pose: bool,
    costmap: bool,
}

const LIVE_INPUT_RECEIPT_LEDGER_CAPACITY: usize = 64;
const LIVE_INPUT_ACK_WAKE_CAPACITY: usize = 4;
const LIVE_POSE_INTERVAL: Duration = Duration::from_millis(100);
const LIVE_COSTMAP_INTERVAL: Duration = Duration::from_millis(250);

#[derive(Default)]
struct LiveInputDeadlineSchedule {
    pose_last_publish_at: Option<tokio::time::Instant>,
    costmap_last_publish_at: Option<tokio::time::Instant>,
}

impl LiveInputDeadlineSchedule {
    fn interval(kind: LiveInputKind) -> Duration {
        match kind {
            LiveInputKind::Pose => LIVE_POSE_INTERVAL,
            LiveInputKind::Costmap => LIVE_COSTMAP_INTERVAL,
        }
    }

    fn record_publish(&mut self, kind: LiveInputKind, published_at: tokio::time::Instant) {
        let slot = match kind {
            LiveInputKind::Pose => &mut self.pose_last_publish_at,
            LiveInputKind::Costmap => &mut self.costmap_last_publish_at,
        };
        if let Some(previous) = *slot {
            assert!(
                published_at >= previous + Self::interval(kind),
                "V2 receipt-driven publication must respect its nominal interval"
            );
        }
        *slot = Some(published_at);
    }

    fn deadline(&self, kind: LiveInputKind) -> Option<tokio::time::Instant> {
        match kind {
            LiveInputKind::Pose => self.pose_last_publish_at,
            LiveInputKind::Costmap => self.costmap_last_publish_at,
        }
        .map(|published_at| published_at + Self::interval(kind))
    }

    fn is_due(&self, kind: LiveInputKind, now: tokio::time::Instant) -> bool {
        match self.deadline(kind) {
            Some(deadline) => now >= deadline,
            None => true,
        }
    }

    fn next_ready_deadline(&self, shaper: &LiveInputReceiptShaper) -> Option<tokio::time::Instant> {
        [LiveInputKind::Pose, LiveInputKind::Costmap]
            .into_iter()
            .filter(|kind| shaper.can_publish(*kind))
            .filter_map(|kind| self.deadline(kind))
            .min()
    }
}

struct LiveInputReceiptShaper {
    pose_outstanding: Option<LiveInputOutstanding>,
    costmap_outstanding: Option<LiveInputOutstanding>,
    ledger: VecDeque<LiveInputReceiptLedgerEntry>,
    completed_count: u64,
    last_timestamp_ns: u64,
    periodic: bool,
}

impl LiveInputReceiptShaper {
    fn new() -> Self {
        Self {
            pose_outstanding: None,
            costmap_outstanding: None,
            ledger: VecDeque::with_capacity(LIVE_INPUT_RECEIPT_LEDGER_CAPACITY),
            completed_count: 0,
            last_timestamp_ns: 0,
            periodic: false,
        }
    }

    fn enable_periodic(&mut self) {
        self.periodic = true;
    }

    fn can_publish(&self, kind: LiveInputKind) -> bool {
        if self.periodic {
            return true;
        }
        match kind {
            LiveInputKind::Pose => self.pose_outstanding.is_none(),
            LiveInputKind::Costmap => self.costmap_outstanding.is_none(),
        }
    }

    fn record_publish(
        &mut self,
        kind: LiveInputKind,
        sequence: u64,
        timestamp_ns: u64,
        published_at: tokio::time::Instant,
    ) {
        assert!(timestamp_ns > 0, "V2 live input timestamp must be positive");
        assert!(
            timestamp_ns > self.last_timestamp_ns,
            "V2 live input timestamp must increase strictly and remain unique"
        );
        assert!(
            self.can_publish(kind),
            "V2 live input may have at most one outstanding publication per kind"
        );
        while self.ledger.len() >= LIVE_INPUT_RECEIPT_LEDGER_CAPACITY {
            let oldest = self
                .ledger
                .front()
                .expect("full V2 receipt ledger must have an oldest entry");
            assert!(
                oldest.receipt_ordinal.is_some() || oldest.superseded_by.is_some(),
                "V2 receipt ledger cannot evict an outstanding publication"
            );
            self.ledger.pop_front();
        }
        let publication = LiveInputOutstanding {
            sequence,
            timestamp_ns,
            published_at,
        };
        if let Some(previous) = self.ledger.iter_mut().rev().find(|entry| {
            entry.kind == kind && entry.receipt_at.is_some() && entry.next_publication.is_none()
        }) {
            previous.next_publication = Some(publication);
        }
        match kind {
            LiveInputKind::Pose => self.pose_outstanding = Some(publication),
            LiveInputKind::Costmap => self.costmap_outstanding = Some(publication),
        }
        self.ledger.push_back(LiveInputReceiptLedgerEntry {
            kind,
            publication,
            receipt_ordinal: None,
            receipt_at: None,
            superseded_by: None,
            next_publication: None,
        });
        self.last_timestamp_ns = timestamp_ns;
    }

    fn observe_receipt(
        &mut self,
        state: &schema::NavigationState,
        receipt_ordinal: u64,
        receipt_at: tokio::time::Instant,
    ) -> LiveInputAckWake {
        LiveInputAckWake {
            pose: self.acknowledge(
                LiveInputKind::Pose,
                state.pose_source_timestamp_ns,
                receipt_ordinal,
                receipt_at,
            ),
            costmap: self.acknowledge(
                LiveInputKind::Costmap,
                state.costmap_source_timestamp_ns,
                receipt_ordinal,
                receipt_at,
            ),
        }
    }

    fn acknowledge(
        &mut self,
        kind: LiveInputKind,
        wire_timestamp_ns: i64,
        receipt_ordinal: u64,
        receipt_at: tokio::time::Instant,
    ) -> bool {
        let Some(wire_timestamp_ns) = u64::try_from(wire_timestamp_ns)
            .ok()
            .filter(|timestamp_ns| *timestamp_ns > 0)
        else {
            return false;
        };
        let outstanding = if self.periodic {
            self.ledger.iter().find(|entry| {
                entry.kind == kind
                    && entry.publication.timestamp_ns == wire_timestamp_ns
                    && entry.receipt_at.is_none()
                    && entry.superseded_by.is_none()
            }).map(|entry| entry.publication)
        } else {
            match kind {
                LiveInputKind::Pose => self.pose_outstanding,
                LiveInputKind::Costmap => self.costmap_outstanding,
            }
        };
        let Some(outstanding) = outstanding else {
            return false;
        };
        if wire_timestamp_ns != outstanding.timestamp_ns {
            return false;
        }
        let ledger_entry = self
            .ledger
            .iter_mut()
            .find(|entry| entry.kind == kind && entry.publication == outstanding)
            .expect("V2 outstanding publication must remain in receipt ledger");
        assert!(
            ledger_entry.receipt_ordinal.is_none(),
            "V2 outstanding publication may be acknowledged only once"
        );
        ledger_entry.receipt_ordinal = Some(receipt_ordinal);
        ledger_entry.receipt_at = Some(receipt_at);
        if self.periodic {
            // 遥测可能合并中间输入。只把精确时间戳记作回执，旧帧单独标记为已被覆盖。
            for entry in &mut self.ledger {
                if entry.kind == kind && entry.publication.timestamp_ns < wire_timestamp_ns
                    && entry.receipt_at.is_none() && entry.superseded_by.is_none()
                {
                    entry.superseded_by = Some(wire_timestamp_ns);
                }
            }
        }
        let remaining = self.ledger.iter().rev().find(|entry| {
            entry.kind == kind && entry.receipt_at.is_none() && entry.superseded_by.is_none()
        }).map(|entry| entry.publication);
        match kind {
            LiveInputKind::Pose => self.pose_outstanding = remaining,
            LiveInputKind::Costmap => self.costmap_outstanding = remaining,
        }
        self.completed_count = self
            .completed_count
            .checked_add(1)
            .expect("V2 receipt completion count overflow");
        true
    }

    fn dump(&self, prefix: &str) {
        eprintln!(
            "{prefix}_SUMMARY retained={} completed={} capacity={} pose_outstanding={:?} costmap_outstanding={:?}",
            self.ledger.len(),
            self.completed_count,
            LIVE_INPUT_RECEIPT_LEDGER_CAPACITY,
            self.pose_outstanding,
            self.costmap_outstanding,
        );
        for entry in &self.ledger {
            let publish_to_receipt_ns = entry.receipt_at.map(|receipt_at| {
                receipt_at
                    .duration_since(entry.publication.published_at)
                    .as_nanos()
            });
            let ack_to_next_publish_ns = entry.receipt_at.zip(entry.next_publication).map(
                |(receipt_at, next_publication)| {
                    next_publication
                        .published_at
                        .duration_since(receipt_at)
                        .as_nanos()
                },
            );
            eprintln!(
                "{prefix} kind={:?} sequence={} timestamp_ns={} receipt_ordinal={:?} superseded_by={:?} publish_to_receipt_ns={:?} next_sequence={:?} next_timestamp_ns={:?} ack_to_next_publish_ns={:?}",
                entry.kind,
                entry.publication.sequence,
                entry.publication.timestamp_ns,
                entry.receipt_ordinal,
                entry.superseded_by,
                publish_to_receipt_ns,
                entry.next_publication.map(|publication| publication.sequence),
                entry.next_publication.map(|publication| publication.timestamp_ns),
                ack_to_next_publish_ns,
            );
        }
    }
}

struct LiveInputReceiptDumpGuard(Arc<Mutex<LiveInputReceiptShaper>>);

impl Drop for LiveInputReceiptDumpGuard {
    fn drop(&mut self) {
        self.0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .dump("V2 LIVE_INPUT_RECEIPT_FINAL");
    }
}

fn send_live_input_ack_wakes(
    ack_wake_tx: &tokio::sync::mpsc::Sender<LiveInputKind>,
    ack_wake: LiveInputAckWake,
) {
    for kind in [
        ack_wake.pose.then_some(LiveInputKind::Pose),
        ack_wake.costmap.then_some(LiveInputKind::Costmap),
    ]
    .into_iter()
    .flatten()
    {
        match ack_wake_tx.try_send(kind) {
            Ok(()) | Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {}
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => return,
        }
    }
}

fn drain_live_input_ack_wakes(ack_wake_rx: &mut tokio::sync::mpsc::Receiver<LiveInputKind>) {
    loop {
        match ack_wake_rx.try_recv() {
            Ok(_) => {}
            Err(tokio::sync::mpsc::error::TryRecvError::Empty) => return,
            Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => {
                panic!("V2 receipt wake channel closed")
            }
        }
    }
}

async fn wait_live_input_trigger(
    base_tick: &mut tokio::time::Interval,
    ack_wake_rx: &mut tokio::sync::mpsc::Receiver<LiveInputKind>,
    next_deadline: Option<tokio::time::Instant>,
) -> LiveInputTrigger {
    if let Some(deadline) = next_deadline {
        tokio::select! {
            _ = base_tick.tick() => LiveInputTrigger::BaseTick,
            wake = ack_wake_rx.recv() => LiveInputTrigger::Ack(wake),
            _ = tokio::time::sleep_until(deadline) => LiveInputTrigger::Deadline,
        }
    } else {
        tokio::select! {
            _ = base_tick.tick() => LiveInputTrigger::BaseTick,
            wake = ack_wake_rx.recv() => LiveInputTrigger::Ack(wake),
        }
    }
}

fn plan_after_live_input_trigger(
    trigger: LiveInputTrigger,
    ack_wake_rx: &mut tokio::sync::mpsc::Receiver<LiveInputKind>,
    deadline_schedule: &LiveInputDeadlineSchedule,
    shaper: &LiveInputReceiptShaper,
    now: tokio::time::Instant,
) -> LiveInputPublishPlan {
    if let LiveInputTrigger::Ack(wake) = trigger {
        assert!(wake.is_some(), "V2 receipt wake channel closed");
    }
    drain_live_input_ack_wakes(ack_wake_rx);
    LiveInputPublishPlan {
        pose: shaper.can_publish(LiveInputKind::Pose)
            && deadline_schedule.is_due(LiveInputKind::Pose, now),
        costmap: shaper.can_publish(LiveInputKind::Costmap)
            && deadline_schedule.is_due(LiveInputKind::Costmap, now),
    }
}

#[derive(Default)]
struct LiveInputCadenceSummary {
    pose_publish_count: u64,
    costmap_publish_count: u64,
    pose_max_interval_ns: u64,
    costmap_max_interval_ns: u64,
    last_pose_timestamp_ns: Option<u64>,
    last_costmap_timestamp_ns: Option<u64>,
}

impl LiveInputCadenceSummary {
    fn record_pose(&mut self, timestamp_ns: u64) {
        if let Some(previous) = self.last_pose_timestamp_ns {
            assert!(
                timestamp_ns > previous,
                "V2 pose publish timestamp must increase strictly"
            );
            self.pose_max_interval_ns = self
                .pose_max_interval_ns
                .max(timestamp_ns.saturating_sub(previous));
        }
        self.last_pose_timestamp_ns = Some(timestamp_ns);
        self.pose_publish_count = self
            .pose_publish_count
            .checked_add(1)
            .expect("V2 pose publish count overflow");
    }

    fn record_costmap(&mut self, timestamp_ns: u64) {
        if let Some(previous) = self.last_costmap_timestamp_ns {
            assert!(
                timestamp_ns > previous,
                "V2 costmap publish timestamp must increase strictly"
            );
            self.costmap_max_interval_ns = self
                .costmap_max_interval_ns
                .max(timestamp_ns.saturating_sub(previous));
        }
        self.last_costmap_timestamp_ns = Some(timestamp_ns);
        self.costmap_publish_count = self
            .costmap_publish_count
            .checked_add(1)
            .expect("V2 costmap publish count overflow");
    }

    fn dump(&self, prefix: &str) {
        eprintln!(
            "{prefix} pose_publish_count={} costmap_publish_count={} pose_max_interval_ns={} costmap_max_interval_ns={} last_pose_timestamp_ns={:?} last_costmap_timestamp_ns={:?}",
            self.pose_publish_count,
            self.costmap_publish_count,
            self.pose_max_interval_ns,
            self.costmap_max_interval_ns,
            self.last_pose_timestamp_ns,
            self.last_costmap_timestamp_ns,
        );
    }
}

struct LiveInputCadenceDumpGuard(Arc<Mutex<LiveInputCadenceSummary>>);

impl Drop for LiveInputCadenceDumpGuard {
    fn drop(&mut self) {
        self.0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .dump("V2 LIVE_INPUT_CADENCE_FINAL");
    }
}

fn has_empty_navigation_identity(state: &schema::NavigationState) -> bool {
    state.session_id.is_empty()
        && state.owner_kind == 0
        && state.parent_request_id.is_empty()
        && state.goal_generation == 0
        && state.path_revision == 0
        && state.producer_runtime_generation == 0
        && state.planner_runtime_generation == 0
}

async fn fail_pre_path_gate(
    reason: &str,
    _live_input_task: &mut tokio::task::JoinHandle<()>,
    publish_ack_rx: &mut tokio::sync::mpsc::Receiver<LiveInputPublishLedgerEntry>,
    publish_ledger: &mut VecDeque<LiveInputPublishLedgerEntry>,
    nav_state_ledger_rx: &mut tokio::sync::mpsc::Receiver<NavStateLedgerEntry>,
    nav_state_ledger: &mut NavStateEvidenceLedger,
) -> ! {
    let drain_deadline = tokio::time::Instant::now() + Duration::from_millis(50);
    let mut publish_open = true;
    let mut state_open = true;
    loop {
        if tokio::time::Instant::now() >= drain_deadline || (!publish_open && !state_open) {
            break;
        }
        tokio::select! {
            entry = publish_ack_rx.recv(), if publish_open && publish_ledger.len() < 64 => {
                match entry {
                    Some(entry) => publish_ledger.push_back(entry),
                    None => publish_open = false,
                }
            }
            entry = nav_state_ledger_rx.recv(), if state_open => {
                match entry {
                    Some(entry) => nav_state_ledger.record(
                        NavEvidenceKind::Routine(NavRoutineEvidenceKind::FailureDrain),
                        entry,
                    ),
                    None => state_open = false,
                }
            }
            _ = tokio::time::sleep_until(drain_deadline) => break,
        }
    }
    eprintln!("V2 PRE_PATH_GATE_FAILURE reason={reason}");
    eprintln!("V2 PRE_PATH_GATE_FAILURE structured_rejected_visibility=UNAVAILABLE stderr_only_boundary=production_scheduler_stderr_not_observable_via_typed_navigation_state");
    for entry in publish_ledger {
        eprintln!(
            "V2 PRE_PATH_PUBLISH_ACK phase={:?} sequence={} timestamp_ns={} pose={:?}",
            entry.phase, entry.sequence, entry.timestamp_ns, entry.pose
        );
    }
    nav_state_ledger.dump("V2 PRE_PATH_STATE");
    panic!("V2 pre-Path receipt gate failed: {reason}");
}

fn spawn_path_ledger(
    mut stream: nuwax_transport::LcmFrameStream,
    started_at: std::time::Instant,
) -> (
    tokio::sync::mpsc::Receiver<PathLedgerEntry>,
    tokio::task::JoinHandle<()>,
) {
    let (tx, rx) = tokio::sync::mpsc::channel(64);
    let task = tokio::spawn(async move {
        let mut ordinal = 0_u64;
        while let Some(frame) = stream.next().await {
            let frame = frame.expect("V2 PathEnvelope ledger transport error");
            let path = schema::decode_path_envelope_body(&frame.payload)
                .expect("decode V2 PathEnvelope ledger frame");
            ordinal += 1;
            let elapsed_ns = started_at.elapsed().as_nanos();
            let first = path.path.poses.first().map(|pose| pose.pose.position);
            let last = path.path.poses.last().map(|pose| pose.pose.position);
            eprintln!(
                "V2 PATH_LEDGER ordinal={} elapsed_ns={} schema={} session={} owner={} parent={} goal_generation={} producer={} producer_epoch={} producer_runtime_generation={} map_session={} map_session_generation={} map_revision={} map_boot_nonce={:02x?} map_commit_sequence={} map_source_timestamp_ns={} map_frame={} boundary_id={} guard_generation={} guard_nonce={:02x?} stop_generation={} planner_generation={} revision={} source_timestamp_ns={} goal_runtime={} planner_runtime={} points={} first={:?} last={:?}",
                ordinal,
                elapsed_ns,
                path.schema_version,
                path.session_id,
                path.owner_kind,
                path.parent_request_id,
                path.goal_generation,
                path.producer,
                path.producer_epoch,
                path.producer_runtime_generation,
                path.committed_map_session_id,
                path.committed_map_session_generation,
                path.committed_map_revision,
                path.committed_map_boot_nonce,
                path.committed_map_commit_sequence,
                path.committed_map_source_timestamp_ns,
                path.committed_map_frame_id,
                path.boundary_id,
                path.motion_guard_generation,
                path.motion_guard_boot_nonce,
                path.motion_stop_generation,
                path.planner_runtime_generation,
                path.path_revision,
                path.source_timestamp_ns,
                path.producer_runtime_generation,
                path.planner_runtime_generation,
                path.path.poses.len(),
                first,
                last
            );
            match tx.try_send(PathLedgerEntry {
                ordinal,
                elapsed_ns,
                path,
            }) {
                Ok(()) => {}
                Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                    panic!("V2 PathEnvelope ledger overflow: capacity=64")
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => break,
            }
        }
    });
    (rx, task)
}

fn spawn_nav_state_ledger(
    mut stream: nuwax_transport::LcmFrameStream,
    nav: crate::nav_runtime::NavRuntimeState,
    started_at: std::time::Instant,
    live_input_receipt_shaper: Arc<Mutex<LiveInputReceiptShaper>>,
    ack_wake_tx: tokio::sync::mpsc::Sender<LiveInputKind>,
) -> (
    tokio::sync::mpsc::Receiver<NavStateLedgerEntry>,
    tokio::task::JoinHandle<()>,
) {
    let (tx, rx) = tokio::sync::mpsc::channel(64);
    let task = tokio::spawn(async move {
        let mut ordinal = 0_u64;
        while let Some(frame) = stream.next().await {
            let frame = frame.expect("V2 NavigationState ledger transport error");
            let state = schema::decode_navigation_state_body(&frame.payload)
                .expect("decode V2 NavigationState ledger frame");
            let feedback_applied = apply_canary_idle_feedback(&nav, &frame.payload).await;
            ordinal += 1;
            let elapsed_ns = started_at.elapsed().as_nanos();
            let receipt_at = tokio::time::Instant::now();
            let ack_wake = live_input_receipt_shaper
                .lock()
                .expect("live input receipt shaper lock poisoned")
                .observe_receipt(&state, ordinal, receipt_at);
            send_live_input_ack_wakes(&ack_wake_tx, ack_wake);
            eprintln!(
                "V2 STATE_LEDGER ordinal={} elapsed_ns={} session={} runtime_generation={} guard_generation={} pose_source_timestamp_ns={} costmap_source_timestamp_ns={} goal_generation={} path_revision={} goal_runtime={} planner_runtime={} state={} message={:?} zero_ack={} zero_ns={}",
                ordinal,
                elapsed_ns,
                state.session_id,
                state.runtime_generation,
                state.guard_generation,
                state.pose_source_timestamp_ns,
                state.costmap_source_timestamp_ns,
                state.goal_generation,
                state.path_revision,
                state.producer_runtime_generation,
                state.planner_runtime_generation,
                state.state,
                state.message,
                state.zero_ack_id,
                state.zero_applied_monotonic_ns
            );
            match tx.try_send(NavStateLedgerEntry {
                ordinal,
                elapsed_ns,
                state,
                feedback_applied,
            }) {
                Ok(()) => {}
                Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                    panic!("V2 NavigationState ledger overflow: capacity=64")
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => break,
            }
        }
    });
    (rx, task)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum V2CanaryExplorePhase {
    PreStartUnregistered,
    Started,
}

struct V2CanaryOwnedResources {
    live_input_task: Option<tokio::task::JoinHandle<()>>,
    path_ledger_task: Option<tokio::task::JoinHandle<()>>,
    nav_state_ledger_task: Option<tokio::task::JoinHandle<()>>,
    runner: Option<StackRunner>,
    guard_client: Option<nuwax_motion_safety_guard::MotionGuardClient>,
    guard: Option<GuardChild>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    explore_phase: V2CanaryExplorePhase,
    mapping: Arc<parking_lot::Mutex<MappingAdapter>>,
    parent_request: String,
}

#[derive(Default)]
struct V2CanaryCleanupReport {
    feeder_panic: Option<Box<dyn std::any::Any + Send + 'static>>,
    errors: Vec<String>,
}

impl V2CanaryCleanupReport {
    fn summary(&self) -> String {
        format!(
            "feeder_panic={} errors={:?}",
            self.feeder_panic.is_some(),
            self.errors
        )
    }
}

fn panic_payload_message(payload: &(dyn std::any::Any + Send + 'static)) -> String {
    if let Some(message) = payload.downcast_ref::<String>() {
        message.clone()
    } else if let Some(message) = payload.downcast_ref::<&'static str>() {
        (*message).to_string()
    } else {
        "non-string panic payload".to_string()
    }
}

fn explore_pre_start_allows_sink_clear(
    snapshot: &crate::explore_runtime::ExploreRuntimeSnapshot,
    blocks_mapping_teardown: bool,
) -> bool {
    !blocks_mapping_teardown
        && snapshot.status == crate::explore_runtime::STATUS_IDLE
        && snapshot.request_id.is_empty()
        && snapshot.exploration_generation == 0
        && snapshot.child_task_id.is_empty()
        && snapshot.child_lease.is_none()
        && !snapshot.global_stop_reconciliation.eligible
        && snapshot.ownership_reconciliation_receipt.is_none()
}

fn explore_stop_allows_sink_clear(
    stopped: &crate::explore_runtime::ExploreRuntimeSnapshot,
    blocks_mapping_teardown: bool,
) -> bool {
    !blocks_mapping_teardown
        && stopped.status == crate::explore_runtime::STATUS_CANCELLED
        && stopped.child_task_id.is_empty()
        && stopped.child_lease.is_none()
}

async fn cleanup_v2_canary(
    resources: &mut V2CanaryOwnedResources,
    path_ledger_rx: &mut tokio::sync::mpsc::Receiver<PathLedgerEntry>,
    path_ledger: &mut VecDeque<PathLedgerEntry>,
    nav_state_ledger_rx: &mut tokio::sync::mpsc::Receiver<NavStateLedgerEntry>,
    nav_state_ledger: &mut NavStateEvidenceLedger,
) -> V2CanaryCleanupReport {
    let mut report = V2CanaryCleanupReport::default();

    if let Some(task) = resources.live_input_task.take() {
        let feeder = std::panic::AssertUnwindSafe(async move {
            task.abort();
            task.await
        })
        .catch_unwind()
        .await;
        match feeder {
            Ok(Err(error)) if error.is_cancelled() => {}
            Ok(Err(error)) if error.is_panic() => {
                report.feeder_panic = Some(error.into_panic());
            }
            Ok(Err(error)) => report
                .errors
                .push(format!("live input feeder join error: {error}")),
            Ok(Ok(())) => report
                .errors
                .push("live input feeder exited before cleanup abort".to_string()),
            Err(payload) => report.errors.push(format!(
                "live input feeder cleanup panicked: {}",
                panic_payload_message(payload.as_ref())
            )),
        }
    }

    let drain = std::panic::AssertUnwindSafe(async {
        let drain_deadline = tokio::time::Instant::now() + Duration::from_millis(50);
        loop {
            tokio::select! {
                entry = path_ledger_rx.recv() => {
                    match entry {
                        Some(entry) if path_ledger.len() < 64 => path_ledger.push_back(entry),
                        Some(entry) => return Err(format!(
                            "PathEnvelope evidence overflow ordinal={} capacity=64",
                            entry.ordinal
                        )),
                        None => break,
                    }
                }
                entry = nav_state_ledger_rx.recv() => {
                    match entry {
                        Some(entry) => {
                            let can_retain = nav_state_ledger.retained_count
                                < NAV_STATE_EVIDENCE_CAPACITY
                                || nav_state_ledger.runs.back().is_some_and(|run| {
                                    run.kind == NavEvidenceKind::Routine(
                                        NavRoutineEvidenceKind::FailureDrain,
                                    ) && run.observed_count > 1
                                });
                            if can_retain {
                                nav_state_ledger.record(
                                    NavEvidenceKind::Routine(
                                        NavRoutineEvidenceKind::FailureDrain,
                                    ),
                                    entry,
                                );
                            } else {
                                return Err(format!(
                                    "NavigationState evidence overflow ordinal={} retained={} compacted={}",
                                    entry.ordinal,
                                    nav_state_ledger.retained_count,
                                    nav_state_ledger.compacted_count
                                ));
                            }
                        }
                        None => break,
                    }
                }
                _ = tokio::time::sleep_until(drain_deadline) => break,
            }
        }
        Ok::<(), String>(())
    })
    .catch_unwind()
    .await;
    match drain {
        Ok(Ok(())) => {}
        Ok(Err(error)) => report.errors.push(format!("bounded drain failed: {error}")),
        Err(payload) => report.errors.push(format!(
            "bounded drain panicked: {}",
            panic_payload_message(payload.as_ref())
        )),
    }

    let dump = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        for entry in path_ledger.iter() {
            eprintln!(
                "V2 CLEANUP_PATH ordinal={} elapsed_ns={} session={} goal_generation={} revision={} planner_runtime={}",
                entry.ordinal,
                entry.elapsed_ns,
                entry.path.session_id,
                entry.path.goal_generation,
                entry.path.path_revision,
                entry.path.planner_runtime_generation
            );
        }
        nav_state_ledger.dump("V2 CLEANUP_STATE");
    }));
    if let Err(payload) = dump {
        report.errors.push(format!(
            "ledger dump panicked: {}",
            panic_payload_message(payload.as_ref())
        ));
    }

    let explore_settlement = std::panic::AssertUnwindSafe(async {
        match resources.explore_phase {
            V2CanaryExplorePhase::PreStartUnregistered => {
                let snapshot = resources.explore.snapshot();
                let blocks_mapping_teardown = resources.explore.blocks_mapping_mutation().await;
                (
                    V2CanaryExplorePhase::PreStartUnregistered,
                    snapshot,
                    blocks_mapping_teardown,
                )
            }
            V2CanaryExplorePhase::Started => {
                let stopped = resources.explore.stop(&resources.parent_request).await;
                let blocks_mapping_teardown = resources.explore.blocks_mapping_mutation().await;
                (
                    V2CanaryExplorePhase::Started,
                    stopped,
                    blocks_mapping_teardown,
                )
            }
        }
    })
    .catch_unwind()
    .await;
    match explore_settlement {
        Ok((V2CanaryExplorePhase::PreStartUnregistered, snapshot, blocks_mapping_teardown))
            if explore_pre_start_allows_sink_clear(&snapshot, blocks_mapping_teardown) =>
        {
            eprintln!(
                "V2 CLEANUP explore_status={:?} explore_message={:?} pre_start_settled=true",
                snapshot.status, snapshot.message
            );
        }
        Ok((V2CanaryExplorePhase::Started, stopped, blocks_mapping_teardown))
            if explore_stop_allows_sink_clear(&stopped, blocks_mapping_teardown) =>
        {
            eprintln!(
                "V2 CLEANUP explore_status={:?} explore_message={:?} exact_terminal=true",
                stopped.status, stopped.message
            );
        }
        Ok((phase, snapshot, blocks_mapping_teardown)) => {
            report.errors.push(format!(
                "explore cleanup did not settle exact lifecycle phase={phase:?}: status={:?} request_id={:?} child_task_id={:?} child_lease={} message={:?} blocks_mapping_teardown={blocks_mapping_teardown}",
                snapshot.status,
                snapshot.request_id,
                snapshot.child_task_id,
                snapshot.child_lease.is_some(),
                snapshot.message
            ));
            return report;
        }
        Err(payload) => {
            report.errors.push(format!(
                "explore cleanup panicked before exact lifecycle settlement: {}",
                panic_payload_message(payload.as_ref())
            ));
            return report;
        }
    }
    if let Err(payload) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        resources.explore.clear_navigation_sink();
    })) {
        report.errors.push(format!(
            "explore clear_navigation_sink panicked: {}",
            panic_payload_message(payload.as_ref())
        ));
    }
    if let Some(runner) = resources.runner.as_mut() {
        match std::panic::AssertUnwindSafe(runner.shutdown())
            .catch_unwind()
            .await
        {
            Ok(Ok(())) => {}
            Ok(Err(error)) => report
                .errors
                .push(format!("runner shutdown failed: {error:?}")),
            Err(payload) => report.errors.push(format!(
                "runner shutdown panicked: {}",
                panic_payload_message(payload.as_ref())
            )),
        }
    }

    if let Some(task) = resources.path_ledger_task.take() {
        let result = std::panic::AssertUnwindSafe(async move {
            task.abort();
            task.await
        })
        .catch_unwind()
        .await;
        match result {
            Ok(Err(error)) if error.is_cancelled() => {}
            Ok(Err(error)) => report
                .errors
                .push(format!("PathEnvelope ledger task join error: {error}")),
            Ok(Ok(())) => {}
            Err(payload) => report.errors.push(format!(
                "PathEnvelope ledger cleanup panicked: {}",
                panic_payload_message(payload.as_ref())
            )),
        }
    }
    if let Some(task) = resources.nav_state_ledger_task.take() {
        let result = std::panic::AssertUnwindSafe(async move {
            task.abort();
            task.await
        })
        .catch_unwind()
        .await;
        match result {
            Ok(Err(error)) if error.is_cancelled() => {}
            Ok(Err(error)) => report
                .errors
                .push(format!("NavigationState ledger task join error: {error}")),
            Ok(Ok(())) => {}
            Err(payload) => report.errors.push(format!(
                "NavigationState ledger cleanup panicked: {}",
                panic_payload_message(payload.as_ref())
            )),
        }
    }

    if let Some(client) = resources.guard_client.take() {
        if let Err(payload) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drop(client);
        })) {
            report.errors.push(format!(
                "Guard client drop panicked: {}",
                panic_payload_message(payload.as_ref())
            ));
        }
    }
    if let Some(guard) = resources.guard.take() {
        let root = guard.root.clone();
        let socket_path = guard.socket_path.clone();
        if let Err(payload) = std::panic::AssertUnwindSafe(guard.shutdown())
            .catch_unwind()
            .await
        {
            report.errors.push(format!(
                "Guard shutdown panicked: {}",
                panic_payload_message(payload.as_ref())
            ));
        }
        if socket_path.exists() {
            if let Err(error) = std::fs::remove_file(&socket_path) {
                report.errors.push(format!(
                    "Guard socket fallback removal failed path={socket_path:?}: {error}"
                ));
            }
        }
        if root.exists() {
            if let Err(error) = std::fs::remove_dir_all(&root) {
                report.errors.push(format!(
                    "Guard root fallback removal failed path={root:?}: {error}"
                ));
            }
        }
    }

    let mapping_stop = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        resources.mapping.lock().stop_session("v2-canary-mapping")
    }));
    match mapping_stop {
        Ok(Ok(stopped)) => eprintln!("V2 CLEANUP mapping_active={}", stopped.active),
        Ok(Err(error)) => report
            .errors
            .push(format!("mapping stop failed: {error:?}")),
        Err(payload) => report.errors.push(format!(
            "mapping stop panicked: {}",
            panic_payload_message(payload.as_ref())
        )),
    }

    report
}

type V2CanaryBodyOutcome = Result<Result<(), String>, Box<dyn std::any::Any + Send + 'static>>;

fn finish_v2_canary(body_outcome: V2CanaryBodyOutcome, cleanup_report: V2CanaryCleanupReport) {
    let cleanup_summary = cleanup_report.summary();
    match body_outcome {
        Err(payload) => {
            eprintln!("V2 CLEANUP_AFTER_BODY_PANIC {cleanup_summary}");
            std::panic::resume_unwind(payload);
        }
        Ok(Err(reason)) => {
            eprintln!("V2 CLEANUP_AFTER_BODY_REASON {cleanup_summary}");
            panic!("V2 canary body failed after cleanup: {reason}");
        }
        Ok(Ok(())) => {
            if let Some(payload) = cleanup_report.feeder_panic {
                eprintln!("V2 CLEANUP_AFTER_FEEDER_PANIC {cleanup_summary}");
                std::panic::resume_unwind(payload);
            }
            assert!(
                cleanup_report.errors.is_empty(),
                "V2 canary cleanup failed after successful body: {cleanup_summary}"
            );
        }
    }
}

fn encode_canary_nonce(pid: u32, unix_ns: u64) -> String {
    const BASE32: &[u8; 32] = b"abcdefghijklmnopqrstuvwxyz234567";

    let mut bytes = [0_u8; 12];
    bytes[..4].copy_from_slice(&pid.to_be_bytes());
    bytes[4..].copy_from_slice(&unix_ns.to_be_bytes());

    let mut nonce = String::with_capacity(20);
    let mut accumulator = 0_u16;
    let mut bits = 0_u8;
    for byte in bytes {
        accumulator = (accumulator << 8) | u16::from(byte);
        bits += 8;
        while bits >= 5 {
            bits -= 5;
            nonce.push(BASE32[usize::from((accumulator >> bits) & 0x1f)] as char);
            accumulator &= (1_u16 << bits) - 1;
        }
    }
    if bits > 0 {
        nonce.push(BASE32[usize::from((accumulator << (5 - bits)) & 0x1f)] as char);
    }
    nonce
}

fn decode_canary_nonce(nonce: &str) -> Option<(u32, u64)> {
    const BASE32: &[u8; 32] = b"abcdefghijklmnopqrstuvwxyz234567";

    if nonce.len() != 20 {
        return None;
    }
    let mut bytes = [0_u8; 12];
    let mut byte_index = 0_usize;
    let mut accumulator = 0_u16;
    let mut bits = 0_u8;
    for encoded in nonce.bytes() {
        let value = BASE32.iter().position(|candidate| *candidate == encoded)? as u16;
        accumulator = (accumulator << 5) | value;
        bits += 5;
        if bits >= 8 {
            bits -= 8;
            bytes[byte_index] = (accumulator >> bits) as u8;
            byte_index += 1;
            accumulator &= (1_u16 << bits) - 1;
        }
    }
    if byte_index != bytes.len() || bits != 4 || accumulator != 0 {
        return None;
    }

    let pid = u32::from_be_bytes(bytes[..4].try_into().ok()?);
    let unix_ns = u64::from_be_bytes(bytes[4..].try_into().ok()?);
    Some((pid, unix_ns))
}

#[test]
fn canary_nonce_is_fixed_width_lossless_unique_and_fits_typed_patterns() {
    for values in [(0, 0), (u32::MAX, u64::MAX)] {
        let nonce = encode_canary_nonce(values.0, values.1);
        assert_eq!(nonce.len(), 20);
        assert!(nonce
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || (b'2'..=b'7').contains(&byte)));
        assert_eq!(decode_canary_nonce(&nonce), Some(values));
    }

    let unique = [
        encode_canary_nonce(0, 0),
        encode_canary_nonce(0, 1),
        encode_canary_nonce(1, 0),
        encode_canary_nonce(u32::MAX, u64::MAX),
    ];
    assert_eq!(
        unique
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        unique.len()
    );

    let nonce = encode_canary_nonce(u32::MAX, u64::MAX);
    let typed_patterns = [
        (
            "occupancy",
            format!("/v2/{nonce}/occ"),
            "nav_msgs.OccupancyGrid",
        ),
        ("goal", "/nav/goal".to_string(), "nuwax_msgs.GoalEnvelope"),
        ("path", "/nav/path".to_string(), "nuwax_msgs.PathEnvelope"),
        (
            "pose",
            format!("/v2/{nonce}/pose"),
            "geometry_msgs.PoseStamped",
        ),
        ("stop", format!("/v2/{nonce}/stop"), "std_msgs.Bool"),
        ("odometry", format!("/v2/{nonce}/odom"), "nav_msgs.Odometry"),
        ("tf", format!("/v2/{nonce}/tf"), "tf2_msgs.TFMessage"),
        (
            "cancel",
            format!("/v2/{nonce}/cancel"),
            "nuwax_msgs.CancelNavigation",
        ),
        (
            "cmd_vel",
            format!("/v2/{nonce}/cmd_vel"),
            "geometry_msgs.Twist",
        ),
        (
            "nav_state",
            format!("/v2/{nonce}/nav_state"),
            "nuwax_msgs.NavigationState",
        ),
        (
            "goal_reached",
            format!("/v2/{nonce}/goal_reached"),
            "std_msgs.Bool",
        ),
    ];
    for (name, channel, message_type) in &typed_patterns {
        let typed_fqn = LcmTopic::new(channel)
            .expect("typed canary topic")
            .fqn(message_type);
        assert!(
            typed_fqn.len() <= 63,
            "{name} typed FQN exceeds LCM's 63-byte pattern budget: {}",
            typed_fqn.len()
        );
    }
    let pattern_len = |name| {
        let (_, channel, message_type) = typed_patterns
            .iter()
            .find(|(candidate, _, _)| *candidate == name)
            .expect("typed canary channel");
        LcmTopic::new(channel)
            .expect("typed canary topic")
            .fqn(message_type)
            .len()
    };
    assert_eq!(pattern_len("nav_state"), 61);
    assert_eq!(pattern_len("cancel"), 59);

    let old_20_bit_pid = format!("{:x}-{:x}", 0xfffff_u32, u64::MAX);
    let old_21_bit_pid = format!("{:x}-{:x}", 0x100000_u32, u64::MAX);
    assert_eq!(old_20_bit_pid.len(), 22);
    assert_eq!(old_21_bit_pid.len(), 23);
    assert_eq!(
        format!("/v2/{old_20_bit_pid}/nav_state").len() + 1 + "nuwax_msgs.NavigationState".len(),
        63
    );
    assert_eq!(
        format!("/v2/{old_21_bit_pid}/nav_state").len() + 1 + "nuwax_msgs.NavigationState".len(),
        64
    );

    for (nonce_len, pattern_len, fits) in [(21, 62, true), (22, 63, true), (23, 64, false)] {
        let boundary_nonce = "a".repeat(nonce_len);
        let actual = format!("/v2/{boundary_nonce}/nav_state").len()
            + 1
            + "nuwax_msgs.NavigationState".len();
        assert_eq!(actual, pattern_len);
        assert_eq!(actual <= 63, fits);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn vertical_v2_mapping_exploration_simple_planner_canary() {
    let _env_lock = ENV_LOCK.lock().await;
    let nonce = encode_canary_nonce(std::process::id(), crate::explore_runtime::now_unix_nanos());
    let provider = test_lcm_url();
    let occupancy_channel = format!("/v2/{nonce}/occ");
    let goal_channel = "/nav/goal".to_string();
    let path_channel = "/nav/path".to_string();
    let pose_channel = format!("/v2/{nonce}/pose");
    let stop_channel = format!("/v2/{nonce}/stop");
    let odometry_channel = format!("/v2/{nonce}/odom");
    let tf_channel = format!("/v2/{nonce}/tf");
    let cancel_channel = format!("/v2/{nonce}/cancel");
    let cmd_vel_channel = format!("/v2/{nonce}/cmd_vel");
    let nav_state_channel = format!("/v2/{nonce}/nav_state");
    let goal_reached_channel = format!("/v2/{nonce}/goal_reached");
    let guard = GuardChild::spawn(&nonce).await;
    let guard_socket = guard.socket_path.to_string_lossy().into_owned();
    let _env = EnvRestore::set(&[
        ("NUWAX_LCM_URL", &provider),
        ("NUWAX_MOTION_GUARD_SOCKET", &guard_socket),
    ]);
    let mut guard_client =
        nuwax_motion_safety_guard::MotionGuardClient::connect(guard.socket_path.clone())
            .await
            .expect("connect canary Motion Guard");
    let guard_readiness = guard_client
        .get_readiness()
        .await
        .expect("read production Motion Guard readiness");
    assert!(guard_readiness.ready);
    assert!(guard_readiness.actuator_configured);
    assert!(guard_readiness.epoch_durable);
    assert!(guard_readiness.startup_zero_confirmed);
    assert!(guard_readiness.generation > 0);
    assert_eq!(guard_readiness.boot_nonce.len(), 16);

    let publisher = LcmTransport::new_real(&provider).expect("real LCM publisher context");
    let observer = LcmTransport::new_real(&provider).expect("independent real LCM observer");
    assert_eq!(publisher.backend_name(), "real-lcm");
    assert_eq!(observer.backend_name(), "real-lcm");

    let occupancy_topic = LcmTopic::new(&occupancy_channel).expect("occupancy topic");
    let occupancy_type = LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type");
    let goal_topic = LcmTopic::new(&goal_channel).expect("goal topic");
    let goal_type = LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("goal type");
    let path_topic = LcmTopic::new(&path_channel).expect("path topic");
    let path_type = LcmMessageType::new("nuwax_msgs.PathEnvelope").expect("path type");
    let nav_state_topic = LcmTopic::new(&nav_state_channel).expect("nav state topic");
    let nav_state_type = LcmMessageType::new("nuwax_msgs.NavigationState").expect("nav state type");
    let tf_topic = LcmTopic::new(&tf_channel).expect("TF topic");
    let tf_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF type");
    let mut occupancy_frames = observer
        .subscribe(occupancy_topic.clone(), Some(occupancy_type.clone()))
        .await;
    let mut goal_frames = observer
        .subscribe(goal_topic.clone(), Some(goal_type.clone()))
        .await;
    let path_frames = observer
        .subscribe(path_topic.clone(), Some(path_type.clone()))
        .await;
    let nav_state_frames = observer
        .subscribe(nav_state_topic, Some(nav_state_type))
        .await;
    let mut tf_frames = observer.subscribe(tf_topic, Some(tf_type)).await;
    let ledger_started_at = std::time::Instant::now();
    let live_input_receipt_shaper = Arc::new(Mutex::new(LiveInputReceiptShaper::new()));
    let _live_input_receipt_dump = LiveInputReceiptDumpGuard(live_input_receipt_shaper.clone());
    let (ack_wake_tx, mut ack_wake_rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    let (mut path_ledger_rx, path_ledger_task) = spawn_path_ledger(path_frames, ledger_started_at);
    let (mut nav_state_ledger_rx, nav_state_ledger_task) = spawn_nav_state_ledger(
        nav_state_frames,
        nav.clone(),
        ledger_started_at,
        live_input_receipt_shaper.clone(),
        ack_wake_tx,
    );
    let mut path_ledger = VecDeque::with_capacity(64);
    let mut path_candidates = VecDeque::with_capacity(64);
    let mut nav_state_ledger = NavStateEvidenceLedger::new();

    let mut topics = NavTopicConfig::geometry_runtime_defaults();
    topics.set("occupancy", &occupancy_channel);
    topics.set("odometry", &odometry_channel);
    topics.set("tf", &tf_channel);
    topics.set("map_pose", &pose_channel);
    topics.set("goal_pose", &goal_channel);
    topics.set("stop_movement", &stop_channel);
    topics.set("goal_path", &path_channel);
    topics.set("nav_cancel", &cancel_channel);
    topics.set("cmd_vel", &cmd_vel_channel);
    topics.set("nav_state", &nav_state_channel);
    topics.set("goal_reached", &goal_reached_channel);
    let mut runner = simple_planner_runner(&topics);
    assert_eq!(runner.unowned_nodes(), Vec::<String>::new());
    runner
        .spawn_all()
        .await
        .expect("start complete production Simple2D graph");
    assert_eq!(
        runner.running_nodes(),
        vec![
            "guarded_nav_scheduler",
            "map_pose_adapter",
            "simple_planner"
        ]
    );

    tokio::time::sleep(Duration::from_millis(75)).await;
    let state = Arc::new(RwLock::new(SharedState::default()));
    let mapping = Arc::new(parking_lot::Mutex::new(canary_mapping()));
    let (map_revision_tx, map_revision_rx) = broadcast::channel(4);
    let (map_revision_latest_tx, mut map_revision_latest_rx) = watch::channel(None);
    let bridge_map_revision_latest_tx = map_revision_latest_tx.clone();
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
    let committed_map_authority = Arc::new(std::sync::Mutex::new(None));
    let explore_projection_gate = Arc::new(tokio::sync::Mutex::new(
        super::ExploreProjectionGate::default(),
    ));
    spawn_map_revision_fanout(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority.clone(),
        explore_projection_gate.clone(),
        publisher.clone(),
        occupancy_topic.clone(),
        occupancy_type.clone(),
        None,
        None,
        Arc::new(RwLock::new(None)),
        explore.clone(),
        mapping.clone(),
        lifecycle_lock.clone(),
    );

    let source_timestamp_ns = crate::explore_runtime::now_unix_nanos();
    let event = {
        let mut adapter = mapping.lock();
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("production mapping pose API");
        // 周向观测让自动栅格覆盖完整机器人轮廓；只有前方三个点时，
        // 自动边界的最小横坐标为0，机器人后侧在图外，不能开始导航。
        let points = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.5, 0.5, 0.0),
            Point3::new(1.5, -0.5, 0.0),
            Point3::new(-1.5, 0.5, 0.0),
            Point3::new(-1.5, -0.5, 0.0),
            Point3::new(0.0, 1.5, 0.0),
            Point3::new(0.0, -1.5, 0.0),
        ];
        let outcome = ingest_mapping_points(
            &mut adapter,
            &points,
            12,
            source_timestamp_ns,
            "odom",
            source_timestamp_ns,
            &state,
            &map_revision_tx,
        )
        .expect("production pointcloud ingest API");
        assert!(outcome.occupancy_committed);
        assert_eq!(adapter.current_revision(), 1);
        crate::lcm_runtime::PreparedMapRevisionEvent::from_map(
            &state.try_read().expect("committed map").map,
            &adapter.snapshot(),
        )
        .seal(adapter.latest_occupancy_commit_receipt().expect("actual occupancy receipt"))
        .expect("committed fixture matches actual receipt")
    };
    assert_eq!(event.revision, 1);
    assert_eq!(event.frame_id, "map");

    map_revision_latest_rx
        .changed()
        .await
        .expect("map fanout revision");
    let fanned_out = map_revision_latest_rx
        .borrow_and_update()
        .clone()
        .expect("committed map fanout event");
    assert_eq!(fanned_out.revision, event.revision);
    assert_eq!(fanned_out.frame_id, "map");
    let occupancy_frame = next_typed(&mut occupancy_frames, Duration::from_secs(3)).await;
    let occupancy = schema::decode_occupancy_grid_body(&occupancy_frame.payload)
        .expect("decode fanout occupancy");
    assert_eq!(occupancy.header.seq, event.revision as i32);
    assert_eq!(occupancy.header.frame_id, "map");
    let grid_resolution = f64::from(occupancy.info.resolution);
    let robot_col = (-occupancy.info.origin.position[0] / grid_resolution).floor() as i32;
    let robot_row = (-occupancy.info.origin.position[1] / grid_resolution).floor() as i32;
    let footprint_radius = (nuwax_explore_wavefront::WavefrontConfig::default()
        .obstacle_inflation_radius_m
        / grid_resolution)
        .ceil() as i32;
    // 核对实际生产发布的栅格，不能由配置里的固定原点/宽高推断地图范围。
    for dy in -footprint_radius..=footprint_radius {
        for dx in -footprint_radius..=footprint_radius {
            if dx * dx + dy * dy > footprint_radius * footprint_radius {
                continue;
            }
            let col = robot_col + dx;
            let row = robot_row + dy;
            assert!(
                col >= 0 && row >= 0 && col < occupancy.info.width && row < occupancy.info.height,
                "机器人轮廓必须位于已发布地图内"
            );
            assert_eq!(
                occupancy.data[(row * occupancy.info.width + col) as usize],
                0,
                "机器人轮廓必须被发布为已知自由区域"
            );
        }
    }
    assert!(occupancy.data.iter().any(|value| *value < 0), "夹具必须保留未观测区域");
    let published_costmap = exploration_costmap_from_revision_in_frame(&fanned_out, "map")
        .expect("由实际发布修订构造探索栅格");
    let mut selector = nuwax_explore_wavefront::selector::WavefrontGoalSelector::new(
        nuwax_explore_wavefront::WavefrontConfig::default(),
    );
    let mut selector_session = nuwax_explore_wavefront::session::ExplorationSession::new();
    let mut selector_ids = nuwax_exploration::wavefront_adapter::CountingIdSource::default();
    let selection = selector
        .select_goal_at(
            Point2 { x: 0.0, y: 0.0 },
            &published_costmap.grid,
            &mut selector_session,
            nuwax_nav_types_messages::geometry::Timestamp::default(),
            &mut selector_ids,
        )
        .expect("发布地图上的前沿检测");
    assert!(
        matches!(selection, nuwax_explore_wavefront::GoalSelection::Goal(_)),
        "已发布地图必须同时提供完整起点轮廓和可达安全前沿：{selection:?}"
    );

    let pose = schema::PoseStamped {
        header: schema::Header {
            seq: 1,
            stamp_sec: (source_timestamp_ns / 1_000_000_000) as i32,
            stamp_nsec: (source_timestamp_ns % 1_000_000_000) as i32,
            frame_id: "map".to_string(),
        },
        pose: schema::Pose {
            position: [0.0, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
    };
    publisher
        .publish(typed_frame(
            &pose_channel,
            "geometry_msgs.PoseStamped",
            schema::encode_pose_stamped_body(&pose).expect("encode map pose"),
            1,
        ))
        .await
        .expect("publish planner map pose");

    {
        let mut shared = state.write().await;
        shared.pose = PoseState {
            x: 0.0,
            y: 0.0,
            qw: 1.0,
            frame_id: "map".to_string(),
            timestamp_ns: source_timestamp_ns,
            ..PoseState::default()
        };
    }
    nav.observe_stack_state(NavState {
        status: "IDLE".to_string(),
        message: "V2 canary Simple2D owner active".to_string(),
        timestamp_ns: crate::explore_runtime::now_unix_nanos(),
        ..NavState::default()
    })
    .await;
    let mut tf_history = PublicTfHistory::default();
    let mut tf_generation = 0_u64;
    let initial_tf_payload = canary_tf_payload(source_timestamp_ns);
    publisher
        .publish(typed_frame(
            &tf_channel,
            "tf2_msgs.TFMessage",
            initial_tf_payload,
            1,
        ))
        .await
        .expect("publish initial canary TF");
    let initial_tf_frame = next_typed(&mut tf_frames, Duration::from_secs(1)).await;
    assert!(apply_canary_tf_payload(
        &nav,
        &mut tf_history,
        &mut tf_generation,
        &initial_tf_frame.payload,
    ));
    let (cloud_map_tx, _) = broadcast::channel(4);
    let (nav_state_tx, _) = broadcast::channel(8);
    let (frontiers_tx, _) = broadcast::channel(4);
    let bridge = Arc::new(ExploreNavigationBridge {
        body_reference_frame_id: Arc::from("base_link"),
        mapping_adapter: None,
        state: state.clone(),
        expected_map_frame: Arc::from("map"),
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx: map_revision_tx.clone(),
        map_revision_latest_tx: bridge_map_revision_latest_tx,
        explore_projection_gate,
        committed_map_authority,
        mapping_lifecycle_lock: lifecycle_lock,
        transport: publisher.clone(),
        transport_backend_name: "real-lcm",
        lcm_backend_available: true,
        cmd_vel_sink_ready: Arc::new(AtomicBool::new(true)),
        nav: nav.clone(),
        exploration_producer_epoch: explore.producer_epoch_projection(),
        runtime_observation_generation: Arc::new(AtomicU64::new(1)),
        orphan_cancellations: explore.orphan_cancellation_registry(),
    });
    let readiness_handle = bridge.facade_handle();
    explore.set_navigation_sink(bridge.clone());
    tokio::time::sleep(Duration::from_millis(75)).await;
    let sentinel_pose = schema::Pose {
        position: [0.125, 0.0, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    let fresh_pose = pose.pose.clone();
    let live_pose = Arc::new(Mutex::new((
        LiveInputPhase::Sentinel,
        sentinel_pose.clone(),
    )));
    let mut live_input_publish_ledger = VecDeque::with_capacity(64);
    let (publish_ack_tx, mut publish_ack_rx) = tokio::sync::mpsc::channel(2);
    let live_input_cadence_summary = Arc::new(Mutex::new(LiveInputCadenceSummary::default()));
    let _live_input_cadence_dump = LiveInputCadenceDumpGuard(live_input_cadence_summary.clone());
    let live_input_publisher = publisher.clone();
    let live_occupancy_channel = occupancy_channel.clone();
    let live_pose_channel = pose_channel.clone();
    let live_occupancy_template = occupancy.clone();
    let periodic_live_pose = live_pose.clone();
    let periodic_cadence_summary = live_input_cadence_summary.clone();
    let periodic_receipt_shaper = live_input_receipt_shaper.clone();
    let mut live_input_task = tokio::spawn(async move {
        let mut sequence = 10_000_u64;
        let mut last_timestamp_ns = 0_u64;
        let mut acknowledged_phase = None;
        let mut deadline_schedule = LiveInputDeadlineSchedule::default();
        let mut base_tick = tokio::time::interval(Duration::from_millis(50));
        base_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            let next_deadline = {
                let shaper = periodic_receipt_shaper
                    .lock()
                    .expect("live input receipt shaper lock poisoned");
                deadline_schedule.next_ready_deadline(&shaper)
            };
            let trigger =
                wait_live_input_trigger(&mut base_tick, &mut ack_wake_rx, next_deadline).await;
            let now = tokio::time::Instant::now();
            let plan = {
                let shaper = periodic_receipt_shaper
                    .lock()
                    .expect("live input receipt shaper lock poisoned");
                plan_after_live_input_trigger(
                    trigger,
                    &mut ack_wake_rx,
                    &deadline_schedule,
                    &shaper,
                    now,
                )
            };
            let publish_pose = plan.pose;
            let publish_costmap = plan.costmap;
            if !publish_pose && !publish_costmap {
                continue;
            }

            let (phase, current_pose) = periodic_live_pose
                .lock()
                .expect("live pose lock poisoned")
                .clone();
            let mut allocate_identity = || {
                let timestamp_ns = crate::explore_runtime::now_unix_nanos()
                    .max(last_timestamp_ns.saturating_add(1))
                    .max(1);
                assert!(
                    timestamp_ns > last_timestamp_ns,
                    "V2 live input timestamp must increase strictly"
                );
                last_timestamp_ns = timestamp_ns;
                let allocated_sequence = sequence;
                sequence = sequence
                    .checked_add(1)
                    .expect("V2 live input sequence overflow");
                (allocated_sequence, timestamp_ns)
            };

            let (occupancy_frame, costmap_timestamp_ns) = if publish_costmap {
                let (costmap_sequence, timestamp_ns) = allocate_identity();
                let published_at = tokio::time::Instant::now();
                deadline_schedule.record_publish(LiveInputKind::Costmap, published_at);
                periodic_receipt_shaper
                    .lock()
                    .expect("live input receipt shaper lock poisoned")
                    .record_publish(
                        LiveInputKind::Costmap,
                        costmap_sequence,
                        timestamp_ns,
                        published_at,
                    );
                let mut live_occupancy = live_occupancy_template.clone();
                live_occupancy.header.seq =
                    i32::try_from(costmap_sequence).expect("periodic costmap sequence");
                live_occupancy.header.stamp_sec = (timestamp_ns / 1_000_000_000) as i32;
                live_occupancy.header.stamp_nsec = (timestamp_ns % 1_000_000_000) as i32;
                (
                    Some(typed_frame(
                        &live_occupancy_channel,
                        "nav_msgs.OccupancyGrid",
                        schema::encode_occupancy_grid_body(&live_occupancy)
                            .expect("encode periodic movement costmap"),
                        costmap_sequence,
                    )),
                    Some(timestamp_ns),
                )
            } else {
                (None, None)
            };

            let (pose_frame, pose_publication) = if publish_pose {
                let (pose_sequence, timestamp_ns) = allocate_identity();
                let published_at = tokio::time::Instant::now();
                deadline_schedule.record_publish(LiveInputKind::Pose, published_at);
                periodic_receipt_shaper
                    .lock()
                    .expect("live input receipt shaper lock poisoned")
                    .record_publish(
                        LiveInputKind::Pose,
                        pose_sequence,
                        timestamp_ns,
                        published_at,
                    );
                let live_pose_stamped = schema::PoseStamped {
                    header: schema::Header {
                        seq: i32::try_from(pose_sequence).expect("periodic pose sequence"),
                        stamp_sec: (timestamp_ns / 1_000_000_000) as i32,
                        stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
                        frame_id: "map".to_string(),
                    },
                    pose: current_pose,
                };
                (
                    Some(typed_frame(
                        &live_pose_channel,
                        "geometry_msgs.PoseStamped",
                        schema::encode_pose_stamped_body(&live_pose_stamped)
                            .expect("encode periodic pose"),
                        pose_sequence,
                    )),
                    Some((pose_sequence, timestamp_ns, live_pose_stamped.pose)),
                )
            } else {
                (None, None)
            };

            match (occupancy_frame, pose_frame) {
                (Some(occupancy_frame), Some(pose_frame)) => {
                    let (occupancy_result, pose_result) = tokio::join!(
                        live_input_publisher.publish(occupancy_frame),
                        live_input_publisher.publish(pose_frame)
                    );
                    occupancy_result.expect("publish periodic movement costmap");
                    pose_result.expect("publish periodic pose");
                }
                (Some(occupancy_frame), None) => live_input_publisher
                    .publish(occupancy_frame)
                    .await
                    .expect("publish periodic movement costmap"),
                (None, Some(pose_frame)) => live_input_publisher
                    .publish(pose_frame)
                    .await
                    .expect("publish periodic pose"),
                (None, None) => unreachable!("V2 live input tick must publish at least one input"),
            }
            {
                let mut summary = periodic_cadence_summary
                    .lock()
                    .expect("live input cadence summary lock poisoned");
                if let Some(timestamp_ns) = costmap_timestamp_ns {
                    summary.record_costmap(timestamp_ns);
                }
                if let Some((_, timestamp_ns, _)) = pose_publication.as_ref() {
                    summary.record_pose(*timestamp_ns);
                }
            }
            if let Some((pose_sequence, timestamp_ns, published_pose)) = pose_publication {
                if acknowledged_phase != Some(phase) {
                    let entry = LiveInputPublishLedgerEntry {
                        phase,
                        sequence: pose_sequence,
                        timestamp_ns,
                        pose: published_pose,
                    };
                    match publish_ack_tx.try_send(entry) {
                        Ok(()) => acknowledged_phase = Some(phase),
                        Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                            panic!("V2 pre-Path publish acknowledgement overflow: capacity=2")
                        }
                        Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                            panic!("V2 pre-Path publish acknowledgement receiver closed")
                        }
                    }
                }
            }
        }
    });

    let parent_request = format!("v2-explore-{nonce}");
    let mut resources = V2CanaryOwnedResources {
        live_input_task: Some(live_input_task),
        path_ledger_task: Some(path_ledger_task),
        nav_state_ledger_task: Some(nav_state_ledger_task),
        runner: Some(runner),
        guard_client: Some(guard_client),
        guard: Some(guard),
        explore: explore.clone(),
        explore_phase: V2CanaryExplorePhase::PreStartUnregistered,
        mapping: mapping.clone(),
        parent_request: parent_request.clone(),
    };
    macro_rules! body_failure {
        ($reason:expr) => {{
            return Err::<(), String>($reason.to_string());
        }};
    }
    let body_outcome = std::panic::AssertUnwindSafe(async {
        let sentinel_ack =
        match tokio::time::timeout(Duration::from_secs(1), publish_ack_rx.recv()).await {
            Ok(Some(entry)) => entry,
            Ok(None) => {
                fail_pre_path_gate(
                    "sentinel publish acknowledgement channel closed",
                    resources
                        .live_input_task
                        .as_mut()
                        .expect("V2 live input owner must remain available"),
                    &mut publish_ack_rx,
                    &mut live_input_publish_ledger,
                    &mut nav_state_ledger_rx,
                    &mut nav_state_ledger,
                )
                .await
            }
            Err(_) => {
                fail_pre_path_gate(
                    "sentinel publish acknowledgement timeout",
                    resources
                        .live_input_task
                        .as_mut()
                        .expect("V2 live input owner must remain available"),
                    &mut publish_ack_rx,
                    &mut live_input_publish_ledger,
                    &mut nav_state_ledger_rx,
                    &mut nav_state_ledger,
                )
                .await
            }
        };
    live_input_publish_ledger.push_back(sentinel_ack.clone());
    if sentinel_ack.phase != LiveInputPhase::Sentinel || sentinel_ack.pose != sentinel_pose {
        fail_pre_path_gate(
            "sentinel publish acknowledgement identity mismatch",
            resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
            &mut publish_ack_rx,
            &mut live_input_publish_ledger,
            &mut nav_state_ledger_rx,
            &mut nav_state_ledger,
        )
        .await;
    }
    let sentinel_receipt = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let Some(entry) = nav_state_ledger_rx.recv().await else {
                return Err("NavigationState ledger closed before sentinel receipt".to_string());
            };
            let sentinel_received =
                is_canonical_idle(&entry.state) && entry.state.current == [0.125, 0.0, 0.0];
            let kind = if sentinel_received {
                NavEvidenceKind::Critical(NavCriticalEvidenceKind::SentinelReceipt)
            } else if is_canonical_idle(&entry.state) {
                NavEvidenceKind::Routine(NavRoutineEvidenceKind::PreGateIdleHeartbeat)
            } else {
                NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
            };
            nav_state_ledger.record(kind, entry.clone());
            if !is_canonical_idle(&entry.state) {
                return Err(format!(
                    "active, terminal, or non-empty identity before sentinel receipt at ordinal {}",
                    entry.ordinal
                ));
            }
            if sentinel_received {
                return Ok(entry.ordinal);
            }
        }
    })
    .await;
    let sentinel_ordinal = match sentinel_receipt {
        Ok(Ok(ordinal)) => ordinal,
        Ok(Err(reason)) => {
            fail_pre_path_gate(
                &reason,
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
        Err(_) => {
            fail_pre_path_gate(
                "sentinel NavigationState.current receipt timeout",
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
    };

    *live_pose.lock().expect("live pose lock poisoned") =
        (LiveInputPhase::Fresh, fresh_pose.clone());
    let fresh_ack = match tokio::time::timeout(Duration::from_secs(1), publish_ack_rx.recv()).await
    {
        Ok(Some(entry)) => entry,
        Ok(None) => {
            fail_pre_path_gate(
                "fresh publish acknowledgement channel closed",
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
        Err(_) => {
            fail_pre_path_gate(
                "fresh publish acknowledgement timeout",
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
    };
    live_input_publish_ledger.push_back(fresh_ack.clone());
    if fresh_ack.phase != LiveInputPhase::Fresh
        || fresh_ack.pose != fresh_pose
        || fresh_ack.sequence <= sentinel_ack.sequence
        || fresh_ack.timestamp_ns <= sentinel_ack.timestamp_ns
    {
        fail_pre_path_gate(
            "fresh publish acknowledgement ordering or identity mismatch",
            resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
            &mut publish_ack_rx,
            &mut live_input_publish_ledger,
            &mut nav_state_ledger_rx,
            &mut nav_state_ledger,
        )
        .await;
    }
    let fresh_receipt = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let Some(entry) = nav_state_ledger_rx.recv().await else {
                return Err("NavigationState ledger closed before fresh receipt".to_string());
            };
            let fresh_received = is_canonical_idle(&entry.state)
                && entry.ordinal > sentinel_ordinal
                && entry.state.current == [0.0, 0.0, 0.0];
            let kind = if fresh_received {
                NavEvidenceKind::Critical(NavCriticalEvidenceKind::FreshReceipt)
            } else if is_canonical_idle(&entry.state) {
                NavEvidenceKind::Routine(NavRoutineEvidenceKind::PreGateIdleHeartbeat)
            } else {
                NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
            };
            nav_state_ledger.record(kind, entry.clone());
            if !is_canonical_idle(&entry.state) {
                return Err(format!(
                    "active, terminal, or non-empty identity before fresh receipt at ordinal {}",
                    entry.ordinal
                ));
            }
            if fresh_received {
                return Ok(entry.ordinal);
            }
        }
    })
    .await;
    match fresh_receipt {
        Ok(Ok(_)) => {
            live_input_receipt_shaper.lock().expect("live receipt shaper")
                .enable_periodic();
        }
        Ok(Err(reason)) => {
            fail_pre_path_gate(
                &reason,
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
        Err(_) => {
            fail_pre_path_gate(
                "fresh NavigationState.current receipt timeout",
                resources
                    .live_input_task
                    .as_mut()
                    .expect("V2 live input owner must remain available"),
                &mut publish_ack_rx,
                &mut live_input_publish_ledger,
                &mut nav_state_ledger_rx,
                &mut nav_state_ledger,
            )
            .await
        }
    }
        let readiness_deadline = tokio::time::Instant::now() + Duration::from_secs(1);
        let mut guard_events = || {
            resources
                .guard
                .as_ref()
                .expect("V2 Guard owner must remain available")
                .actuator_events()
        };
        let mut deadline_checkpoint = |_: InitialReadinessDeadlineCheckpoint| false;
        let readiness = match await_initial_navigation_readiness(
            &readiness_handle,
            &nav,
            &mut nav_state_ledger_rx,
            &mut nav_state_ledger,
            &mut goal_frames,
            &mut path_ledger_rx,
            &mut guard_events,
            readiness_deadline,
            &mut deadline_checkpoint,
        )
        .await
        {
            Ok(outcome) => outcome,
            Err(failure) => body_failure!(failure.render()),
        };
        assert!(readiness.detail.ready, "{}", readiness.detail.reason);
        assert_eq!(readiness.release_count, 1);
        assert_eq!(readiness.dispatches, InitialReadinessDispatchLedger::default());

        let boundary = ExplorationBoundary::from_vertices(vec![
        Point2 { x: -2.0, y: -2.0 },
        Point2 { x: 2.0, y: -2.0 },
        Point2 { x: 2.0, y: 2.0 },
        Point2 { x: -2.0, y: 2.0 },
    ])
    .expect("V2 canary boundary");
    let started = explore
        .start_with_config(
            &parent_request,
            crate::explore_runtime::ExploreStartConfig {
                motion_admission: crate::nav_runtime::MotionAdmissionProof::try_from_wire(
                    guard_readiness.generation,
                    &guard_readiness.boot_nonce,
                    guard_readiness.motion_stop_generation,
                )
                .expect("real Motion Guard admission proof"),
                boundary: Some(boundary),
                ..crate::explore_runtime::ExploreStartConfig::default()
            },
        )
        .await;
    resources.explore_phase = V2CanaryExplorePhase::Started;
    assert_eq!(started.status, "ACCEPTED", "{}", started.message);
    let active = explore.tick((0.0, 0.0)).await;
    assert_eq!(active.status, "ACTIVE", "{}", active.message);
    assert!(active.child_lease.is_some());

    let mut first_active_recorded = false;
    let goal_deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    let goal_frame = loop {
        tokio::select! {
            frame = goal_frames.next() => {
                match frame {
                    Some(Ok(frame)) => break frame,
                    Some(Err(error)) => body_failure!(format!(
                        "child goal transport error: {error}"
                    )),
                    None => body_failure!("child goal subscription closed"),
                }
            }
            entry = nav_state_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!("NavigationState ledger closed before child goal"),
                };
                let kind = if entry.state.state == 1 && !first_active_recorded {
                    first_active_recorded = true;
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::FirstActive)
                } else if matches!(entry.state.state, 0 | 1) {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::AwaitingPathRefresh)
                } else if entry.state.state == 2 {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedPathAccepted)
                } else if matches!(entry.state.state, 3 | 4) {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal)
                } else {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                };
                nav_state_ledger.record(kind, entry);
            }
            _ = tokio::time::sleep_until(goal_deadline) => {
                body_failure!("child goal timeout")
            },
        }
    };
    let goal = schema::decode_goal_envelope_body(&goal_frame.payload).expect("decode child goal");
    let lease = nav.active_goal_lease().await.expect("active child lease");
    assert_eq!(
        goal.owner_kind,
        schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    );
    assert_eq!(goal.parent_request_id, parent_request);
    assert_eq!(goal.session_id, lease.session_id);
    assert_eq!(goal.goal_generation, lease.goal_generation as i64);
    assert_eq!(
        goal.producer_runtime_generation,
        lease.producer_runtime_generation as i64
    );
    assert_eq!(goal.producer_runtime_generation, lease.producer_runtime_generation as i64);
    assert_eq!(goal.boundary_frame_id, "map");
    assert_eq!(goal.boundary_points.len(), 4);
    assert_eq!(goal.goal.header.frame_id, "map");

    let mut phase_deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    let mut pending_accepted: Option<NavStateLedgerEntry> = None;
    let acceptance_outcome = loop {
        tokio::select! {
            entry = path_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!("PathEnvelope ledger closed before path acceptance"),
                };
                if path_ledger.is_empty() {
                    assert_eq!(entry.ordinal, 1);
                    assert!(entry.elapsed_ns > 0);
                    phase_deadline = tokio::time::Instant::now() + Duration::from_secs(3);
                }
                assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                path_ledger.push_back(entry.clone());
                if is_canonical_path_candidate(&entry, &goal) {
                    assert!(
                        path_candidates.len() < 64,
                        "V2 canonical PathEnvelope candidate overflow"
                    );
                    path_candidates.push_back(entry);
                }
                if let Some(accepted_entry) = pending_accepted.as_ref() {
                    match match_accepted_path_candidate(
                        &accepted_entry.state,
                        &goal,
                        &path_candidates,
                    ) {
                        AcceptedPathCandidateMatch::Matched(path_entry) => {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(accepted_path_evidence_kind(
                                    true,
                                    &accepted_entry.state,
                                )),
                                accepted_entry.clone(),
                            );
                            break PathAcceptanceOutcome::ExpectedAccepted(path_entry);
                        }
                        AcceptedPathCandidateMatch::Conflict => {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(accepted_path_evidence_kind(
                                    false,
                                    &accepted_entry.state,
                                )),
                                accepted_entry.clone(),
                            );
                            break PathAcceptanceOutcome::UnexpectedAccepted(
                                accepted_entry.clone(),
                            );
                        }
                        AcceptedPathCandidateMatch::Pending => {}
                    }
                }
            }
            entry = nav_state_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!("NavigationState ledger closed before path acceptance"),
                };
                if terminal_before_expected_path_acceptance(&entry.state) {
                    nav_state_ledger.record(
                        NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal),
                        entry.clone(),
                    );
                    break PathAcceptanceOutcome::TerminalBeforeExpectedAcceptance(entry);
                }
                if entry.state.state == 2 {
                    if let Some(pending_entry) = pending_accepted.as_ref() {
                        match classify_pending_accepted_state_update(
                            &pending_entry.state,
                            &entry.state,
                            &goal,
                        ) {
                            PendingAcceptedStateUpdate::Duplicate => {
                                nav_state_ledger.record(
                                    NavEvidenceKind::Routine(
                                        NavRoutineEvidenceKind::ActiveInputRefresh,
                                    ),
                                    entry,
                                );
                                continue;
                            }
                            PendingAcceptedStateUpdate::Conflict => {
                                nav_state_ledger.record(
                                    NavEvidenceKind::Critical(accepted_path_evidence_kind(
                                        false,
                                        &entry.state,
                                    )),
                                    entry.clone(),
                                );
                                break PathAcceptanceOutcome::UnexpectedAccepted(entry);
                            }
                            PendingAcceptedStateUpdate::Advance => {}
                        }
                    }
                    match match_accepted_path_candidate(&entry.state, &goal, &path_candidates) {
                        AcceptedPathCandidateMatch::Matched(path_entry) => {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(accepted_path_evidence_kind(
                                    true,
                                    &entry.state,
                                )),
                                entry,
                            );
                            break PathAcceptanceOutcome::ExpectedAccepted(path_entry);
                        }
                        AcceptedPathCandidateMatch::Conflict => {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(accepted_path_evidence_kind(
                                    false,
                                    &entry.state,
                                )),
                                entry.clone(),
                            );
                            break PathAcceptanceOutcome::UnexpectedAccepted(entry);
                        }
                        AcceptedPathCandidateMatch::Pending => {
                            if let Some(superseded) = pending_accepted.replace(entry) {
                                nav_state_ledger.record(
                                    NavEvidenceKind::Routine(
                                        NavRoutineEvidenceKind::ActiveInputRefresh,
                                    ),
                                    superseded,
                                );
                            }
                        }
                    }
                    continue;
                }
                let kind = if entry.state.state == 1 && !first_active_recorded {
                    first_active_recorded = true;
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::FirstActive)
                } else if matches!(entry.state.state, 0 | 1) {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::AwaitingPathRefresh)
                } else {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                };
                nav_state_ledger.record(kind, entry);
            }
            _ = tokio::time::sleep_until(phase_deadline) => {
                if let Some(entry) = pending_accepted.take() {
                    nav_state_ledger.record(
                        NavEvidenceKind::Critical(accepted_path_evidence_kind(false, &entry.state)),
                        entry.clone(),
                    );
                    break PathAcceptanceOutcome::UnexpectedAccepted(entry);
                }
                if path_ledger.is_empty() {
                    body_failure!("first PathEnvelope ledger timeout")
                }
                body_failure!("scheduler path acceptance timeout")
            }
        }
    };
    let mut acceptance_failure = None;
    let terminal_entry = match acceptance_outcome {
        PathAcceptanceOutcome::ExpectedAccepted(accepted_path_entry) => {
            let path = accepted_path_entry.path;
            assert!(!path.path.poses.is_empty());
            assert_eq!(path.path.header.frame_id, "map");
            assert!(path.planner_runtime_generation > 0);
            assert!(path.path_revision > 0);
            publisher
                .publish(typed_frame(
                    &stop_channel,
                    "std_msgs.Bool",
                    schema::encode_bool_body(&schema::Bool { data: true }),
                    1,
                ))
                .await
                .expect("stop planner after first canonical path");
            tokio::time::sleep(Duration::from_millis(75)).await;
            let accepted_path_lease = nav
                .observe_path_envelope(&path)
                .await
                .expect("correlated PathEnvelope accepted");
            assert_eq!(accepted_path_lease.path_revision, path.path_revision as u64);

            let mut wrong_identity = path.clone();
            wrong_identity.session_id.push_str("-stale");
            schema::validate_path_envelope(&wrong_identity).expect("reseal wrong-identity path");
            assert!(nav.observe_path_envelope(&wrong_identity).await.is_err());
            assert_eq!(explore.snapshot().status, "ACTIVE");
            resources
                .runner
                .as_mut()
                .expect("V2 runner owner must remain available")
                .ensure_healthy()
                .expect("complete Simple2D graph healthy");

            let actuator_deadline = tokio::time::Instant::now() + Duration::from_secs(3);
            let mut early_terminal = None;
            loop {
                let events = resources
                    .guard
                    .as_ref()
                    .expect("V2 Guard owner must remain available")
                    .actuator_events();
                if events.lines().any(|line| {
                    line.strip_prefix("TWIST ").is_some_and(|values| {
                        values
                            .split_ascii_whitespace()
                            .any(|value| value.parse::<f64>().is_ok_and(|value| value != 0.0))
                    })
                }) {
                    break;
                }
                tokio::select! {
                    entry = path_ledger_rx.recv() => {
                        let entry = match entry {
                            Some(entry) => entry,
                            None => body_failure!("PathEnvelope ledger closed while waiting for actuator"),
                        };
                        assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                        path_ledger.push_back(entry);
                    }
                    entry = nav_state_ledger_rx.recv() => {
                        let entry = match entry {
                            Some(entry) => entry,
                            None => body_failure!("NavigationState ledger closed while waiting for actuator"),
                        };
                        if matches!(entry.state.state, 3 | 4) {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal),
                                entry.clone(),
                            );
                            early_terminal = Some(entry);
                            break;
                        }
                        let kind = if state_reports_twist_applied(&entry.state) {
                            NavEvidenceKind::Critical(NavCriticalEvidenceKind::TwistApplied)
                        } else if entry.state.state == 2 {
                            NavEvidenceKind::Routine(NavRoutineEvidenceKind::ActiveInputRefresh)
                        } else {
                            NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                        };
                        nav_state_ledger.record(kind, entry);
                    }
                    _ = tokio::time::sleep(Duration::from_millis(20)) => {}
                    _ = tokio::time::sleep_until(actuator_deadline) => {
                        body_failure!("first non-zero production actuator command timeout")
                    }
                }
            }

            if let Some(entry) = early_terminal {
                entry
            } else {
                for waypoint in path.path.poses.iter().skip(1) {
                    live_pose.lock().expect("live pose lock poisoned").1 = waypoint.pose.clone();
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }

                let terminal_deadline = tokio::time::Instant::now() + Duration::from_secs(5);
                loop {
                    tokio::select! {
                        entry = path_ledger_rx.recv() => {
                            let entry = match entry {
                                Some(entry) => entry,
                                None => body_failure!("PathEnvelope ledger closed before terminal"),
                            };
                            assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                            path_ledger.push_back(entry);
                        }
                        entry = nav_state_ledger_rx.recv() => {
                            let entry = match entry {
                                Some(entry) => entry,
                                None => body_failure!("NavigationState ledger closed before terminal"),
                            };
                            if matches!(entry.state.state, 3 | 4) {
                                nav_state_ledger.record(
                                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal),
                                    entry.clone(),
                                );
                                break entry;
                            }
                            let kind = if state_reports_twist_applied(&entry.state) {
                                NavEvidenceKind::Critical(NavCriticalEvidenceKind::TwistApplied)
                            } else if entry.state.state == 2 {
                                NavEvidenceKind::Routine(NavRoutineEvidenceKind::ActiveInputRefresh)
                            } else {
                                NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                            };
                            nav_state_ledger.record(kind, entry);
                        }
                        _ = tokio::time::sleep_until(terminal_deadline) => {
                            body_failure!("guarded scheduler terminal timeout")
                        }
                    }
                }
            }
        }
        PathAcceptanceOutcome::TerminalBeforeExpectedAcceptance(entry) => {
            acceptance_failure = Some(format!(
                "terminal before expected Path acceptance: ordinal={} message={:?}",
                entry.ordinal, entry.state.message
            ));
            entry
        }
        PathAcceptanceOutcome::UnexpectedAccepted(entry) => {
            acceptance_failure = Some(format!(
                "unexpected Path accepted: ordinal={} session={:?} goal_generation={} path_revision={} goal_runtime={} planner_runtime={} message={:?}",
                entry.ordinal,
                entry.state.session_id,
                entry.state.goal_generation,
                entry.state.path_revision,
                entry.state.producer_runtime_generation,
                entry.state.planner_runtime_generation,
                entry.state.message
            ));
            let terminal_deadline = tokio::time::Instant::now() + Duration::from_secs(5);
            loop {
                tokio::select! {
                    path_entry = path_ledger_rx.recv() => {
                        let path_entry = match path_entry {
                            Some(entry) => entry,
                            None => body_failure!("PathEnvelope ledger closed after unexpected acceptance"),
                        };
                        assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                        path_ledger.push_back(path_entry);
                    }
                    state_entry = nav_state_ledger_rx.recv() => {
                        let state_entry = match state_entry {
                            Some(entry) => entry,
                            None => body_failure!("NavigationState ledger closed after unexpected acceptance"),
                        };
                        if matches!(state_entry.state.state, 3 | 4) {
                            nav_state_ledger.record(
                                NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal),
                                state_entry.clone(),
                            );
                            break state_entry;
                        }
                        let kind = if state_reports_twist_applied(&state_entry.state) {
                            NavEvidenceKind::Critical(NavCriticalEvidenceKind::TwistApplied)
                        } else if state_entry.state.state == 2 {
                            NavEvidenceKind::Routine(NavRoutineEvidenceKind::ActiveInputRefresh)
                        } else {
                            NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                        };
                        nav_state_ledger.record(kind, state_entry);
                    }
                    _ = tokio::time::sleep_until(terminal_deadline) => {
                        body_failure!("terminal timeout after unexpected Path acceptance")
                    }
                }
            }
        }
    };
    let runner_health_after_terminal = resources
        .runner
        .as_mut()
        .expect("V2 runner owner must remain available")
        .ensure_healthy()
        .map_err(|error| format!("{error:?}"));
    eprintln!(
        "V2 RUNNER_HEALTH_AFTER_TERMINAL {:?}",
        runner_health_after_terminal
    );

    let terminal_ordinal = terminal_entry.ordinal;
    let ledger_deadline = tokio::time::Instant::now() + Duration::from_millis(500);
    let mut quiet_first_idle_ordinal = None;
    let mut quiet_latest_idle_ordinal = None;
    let mut quiet_latest_applied_idle_ordinal = None;
    let mut quiet_failure = None;
    loop {
        tokio::select! {
            entry = path_ledger_rx.recv() => {
                if let Some(entry) = entry {
                    let ordinal = entry.ordinal;
                    assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                    path_ledger.push_back(entry);
                    quiet_failure = Some(format!(
                        "post-terminal PathEnvelope during quiet window: terminal_ordinal={terminal_ordinal} path_ordinal={ordinal}"
                    ));
                    break;
                } else {
                    body_failure!("V2 PathEnvelope ledger consumer stopped during quiet window")
                }
            }
            entry = nav_state_ledger_rx.recv() => {
                if let Some(entry) = entry {
                    if is_canonical_idle(&entry.state) {
                        let kind = if quiet_first_idle_ordinal.is_none() {
                            quiet_first_idle_ordinal = Some(entry.ordinal);
                            NavEvidenceKind::Critical(
                                NavCriticalEvidenceKind::PostTerminalFirstIdle,
                            )
                        } else {
                            NavEvidenceKind::Routine(
                                NavRoutineEvidenceKind::PostTerminalIdleRepeat,
                            )
                        };
                        quiet_latest_idle_ordinal = Some(entry.ordinal);
                        if entry.feedback_applied { quiet_latest_applied_idle_ordinal = Some(entry.ordinal); }
                        nav_state_ledger.record(kind, entry);
                    } else {
                        let kind = if matches!(entry.state.state, 3 | 4) {
                            NavCriticalEvidenceKind::Terminal
                        } else {
                            NavCriticalEvidenceKind::UnexpectedState
                        };
                        quiet_failure = Some(format!(
                            "noncanonical NavigationState during post-terminal quiet window: ordinal={} state={} session={:?} owner={} parent={:?} goal_generation={} path_revision={} goal_runtime={} planner_runtime={} message={:?}",
                            entry.ordinal,
                            entry.state.state,
                            entry.state.session_id,
                            entry.state.owner_kind,
                            entry.state.parent_request_id,
                            entry.state.goal_generation,
                            entry.state.path_revision,
                            entry.state.producer_runtime_generation,
                            entry.state.planner_runtime_generation,
                            entry.state.message
                        ));
                        nav_state_ledger.record(NavEvidenceKind::Critical(kind), entry);
                        break;
                    }
                } else {
                    body_failure!("V2 NavigationState ledger consumer stopped during quiet window")
                }
            }
            _ = tokio::time::sleep_until(ledger_deadline) => break,
        }
    }
    if quiet_failure.is_none() {
        match (quiet_first_idle_ordinal, quiet_latest_idle_ordinal, quiet_latest_applied_idle_ordinal) {
            (Some(first), Some(latest), Some(applied))
                if terminal_ordinal < first && first <= applied && applied <= latest => {}
            (first, latest, applied) => {
                quiet_failure = Some(format!(
                    "post-terminal readiness feedback missing: terminal={terminal_ordinal} first_idle={first:?} latest_idle={latest:?} latest_applied={applied:?}"
                ));
            }
        }
    }
    eprintln!(
        "V2 QUIET_WINDOW terminal_ordinal={} first_idle={:?} latest_idle={:?} failure={:?}",
        terminal_ordinal, quiet_first_idle_ordinal, quiet_latest_idle_ordinal, quiet_failure
    );

    for entry in &path_ledger {
        eprintln!(
            "V2 PATH_LEDGER_FINAL ordinal={} elapsed_ns={} session={} planner_generation={} goal_generation={} revision={} goal_runtime={} planner_runtime={} points={}",
            entry.ordinal,
            entry.elapsed_ns,
            entry.path.session_id,
            entry.path.planner_runtime_generation,
            entry.path.goal_generation,
            entry.path.path_revision,
            entry.path.producer_runtime_generation,
            entry.path.planner_runtime_generation,
            entry.path.path.poses.len()
        );
    }
    nav_state_ledger.dump("V2 STATE_LEDGER_FINAL");
    let retained_nav_states = nav_state_ledger.retained_entries();
    let actuator_events = resources
                    .guard
                    .as_ref()
                    .expect("V2 Guard owner must remain available")
                    .actuator_events();
    eprintln!("V2 ACTUATOR_LEDGER\n{actuator_events}");

    let path_mutation = path_ledger.iter().enumerate().any(|(index, left)| {
        path_ledger.iter().skip(index + 1).any(|right| {
            left.path.session_id == right.path.session_id
                && left.path.goal_generation == right.path.goal_generation
                && left.path.path_revision == right.path.path_revision
                && left.path.producer_runtime_generation == right.path.producer_runtime_generation
                && left.path.planner_runtime_generation == right.path.planner_runtime_generation
                && left.path.path.poses.iter().map(|p| &p.pose).ne(right.path.path.poses.iter().map(|p| &p.pose))
        })
    }) || retained_nav_states.iter().any(|entry| {
        let message = entry.state.message.to_ascii_lowercase();
        message.contains("pathmutation") || message.contains("path mutation")
    });
    let stale_input = retained_nav_states
        .iter()
        .any(|entry| entry.state.message.to_ascii_lowercase().contains("stale"));
    let planner_failed = retained_nav_states.iter().any(|entry| {
        let message = entry.state.message.to_ascii_lowercase();
        message.contains("planner") && (message.contains("failed") || message.contains("failure"))
    });
    let failure_classification = if let Some(reason) = acceptance_failure.as_deref() {
        Some(format!("PathAcceptanceOutcome: {reason}"))
    } else if let Some(reason) = quiet_failure.as_deref() {
        Some(format!("QuietWindow: {reason}"))
    } else if terminal_entry.state.state != 4 {
        None
    } else if path_mutation {
        Some("PathMutation".to_string())
    } else if stale_input {
        Some("stale input".to_string())
    } else if planner_failed {
        Some("PlannerFailed".to_string())
    } else {
        Some(
            "other fail_closed (generic terminal message; source remains unclassified)".to_string(),
        )
    };
    if let Some(classification) = failure_classification {
        eprintln!(
            "V2 FAILURE_CLASSIFICATION class={classification:?} terminal_ordinal={} terminal_message={:?} runner_health={:?}",
            terminal_entry.ordinal,
            terminal_entry.state.message,
            runner_health_after_terminal
        );
        body_failure!(format!(
            "V2 scheduler failed: classification={classification}; terminal={:?}",
            terminal_entry.state.message
        ));
    }

    let terminal_state = terminal_entry.state;
    assert_eq!(terminal_state.session_id, goal.session_id);
    assert_eq!(
        terminal_state.guard_generation,
        guard_readiness.generation as i64
    );
    assert!(!terminal_state.zero_ack_id.is_empty());
    assert!(terminal_state.zero_applied_monotonic_ns > 0);
    assert_eq!(terminal_state.state, 3);
    assert_eq!(terminal_state.message, "Succeeded after verified fence");
    assert!(actuator_events.lines().any(|line| line == "ZERO"));

    let first_child_id = active.child_task_id.clone();
    let first_nav_lease = nav
        .active_goal_lease()
        .await
        .expect("active terminal lease");
    let terminal_update = NavigationUpdate::from_local(
        NavState {
            status: "SUCCEEDED".to_string(),
            message: "V2 canary reached".to_string(),
            goal_id: first_child_id.clone(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..NavState::default()
        },
        &nav,
        Some(first_nav_lease),
    )
    .await;
    let applied_terminal = apply_typed_navigation_feedback(&nav, terminal_update)
        .await
        .expect("apply correlated terminal feedback");
    report_exploration_navigation_terminal(&explore, &applied_terminal).await;
    let child_terminal = explore.snapshot();
    assert_eq!(child_terminal.status, "ACTIVE");
    assert!(child_terminal.child_task_id.is_empty());
    assert!(child_terminal.child_lease.is_none());
    assert!(nav.active_goal_lease().await.is_none());

    let baseline_readiness =
        crate::nav_runtime::compute_readiness(&readiness_handle, &nav).await;
    let mut post_quiet_tf_gate = PostQuietTfGate {
        baseline_count: baseline_readiness.tf_count,
        baseline_generation: baseline_readiness.tf_generation,
        fresh_complete_payload_applied: false,
    };
    let mut post_quiet_tf_proof = PublicTfHistory::default();
    let fresh_tf_payload = canary_tf_payload(crate::explore_runtime::now_unix_nanos());
    publisher
        .publish(typed_frame(&tf_channel, "tf2_msgs.TFMessage", fresh_tf_payload, 2))
        .await
        .expect("publish post-quiet canary TF");
    let readiness_deadline = tokio::time::Instant::now() + Duration::from_secs(1);
    let readiness = loop {
        let detail = crate::nav_runtime::compute_readiness(&readiness_handle, &nav).await;
        if post_quiet_tf_gate.allows_dispatch(&detail) {
            break detail;
        }
        tokio::select! {
            frame = tf_frames.next() => {
                let frame = frame
                    .expect("TF observer closed during readiness gate")
                    .expect("TF observer transport error during readiness gate");
                let proof_complete =
                    canary_tf_window_complete(&mut post_quiet_tf_proof, &frame.payload);
                let applied = apply_canary_tf_payload(
                    &nav, &mut tf_history, &mut tf_generation, &frame.payload,
                );
                post_quiet_tf_gate.observe_payload_result(applied, proof_complete);
            }
            entry = nav_state_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!(
                        "V2 NavigationState ledger consumer stopped during readiness gate"
                    ),
                };
                if !is_canonical_idle(&entry.state) {
                    body_failure!(format!(
                        "noncanonical NavigationState during readiness gate: ordinal={} state={} session={:?} owner={} parent={:?} goal_generation={} path_revision={} message={:?}",
                        entry.ordinal,
                        entry.state.state,
                        entry.state.session_id,
                        entry.state.owner_kind,
                        entry.state.parent_request_id,
                        entry.state.goal_generation,
                        entry.state.path_revision,
                        entry.state.message,
                    ));
                }
                let kind = if entry.feedback_applied {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::PostTerminalIdleRepeat)
                } else {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                };
                nav_state_ledger.record(kind, entry);
            }
            entry = path_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!(
                        "V2 PathEnvelope ledger consumer stopped during readiness gate"
                    ),
                };
                let ordinal = entry.ordinal;
                assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                path_ledger.push_back(entry);
                body_failure!(format!(
                    "post-terminal PathEnvelope during readiness gate: terminal_ordinal={terminal_ordinal} path_ordinal={ordinal}"
                ));
            }
            _ = tokio::time::sleep_until(readiness_deadline) => {
                body_failure!(format!("post-terminal readiness timeout: {detail:#?}"));
            }
        }
    };
    assert!(readiness.ready, "{}", readiness.reason);

    let next_active = explore.tick((0.0, 0.0)).await;
    assert_eq!(next_active.status, "ACTIVE", "{}", next_active.message);
    assert!(!next_active.child_task_id.is_empty());
    assert_ne!(next_active.child_task_id, first_child_id);
    assert!(next_active.child_lease.is_some());

    let second_goal_deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    let mut pending_second_path = None;
    let second_goal_frame = loop {
        tokio::select! {
            frame = goal_frames.next() => {
                match frame {
                    Some(Ok(frame)) => break frame,
                    Some(Err(error)) => body_failure!(format!(
                        "post-terminal goal transport error: {error}"
                    )),
                    None => body_failure!("post-terminal goal subscription closed"),
                }
            }
            entry = path_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!("PathEnvelope ledger closed before post-terminal goal"),
                };
                assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
                path_ledger.push_back(entry.clone());
                pending_second_path = Some(entry);
            }
            entry = nav_state_ledger_rx.recv() => {
                let entry = match entry {
                    Some(entry) => entry,
                    None => body_failure!("NavigationState ledger closed before post-terminal goal"),
                };
                let kind = if matches!(entry.state.state, 0 | 1) {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::AwaitingPathRefresh)
                } else if entry.state.state == 2 {
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::ActiveInputRefresh)
                } else if matches!(entry.state.state, 3 | 4) {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal)
                } else {
                    NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                };
                nav_state_ledger.record(kind, entry);
            }
            _ = tokio::time::sleep_until(second_goal_deadline) => {
                body_failure!("post-terminal child goal timeout")
            }
        }
    };
    let second_goal = schema::decode_goal_envelope_body(&second_goal_frame.payload)
        .expect("decode post-terminal child goal");
    assert_ne!(second_goal.goal_generation, goal.goal_generation);
    assert_eq!(second_goal.parent_request_id, parent_request);

    let second_path_entry = if let Some(entry) = pending_second_path {
        entry
    } else {
        let second_path_deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        loop {
            tokio::select! {
                entry = path_ledger_rx.recv() => {
                    match entry {
                        Some(entry) => break entry,
                        None => body_failure!("PathEnvelope ledger closed before post-terminal path"),
                    }
                }
                entry = nav_state_ledger_rx.recv() => {
                    let entry = match entry {
                        Some(entry) => entry,
                        None => body_failure!("NavigationState ledger closed before post-terminal path"),
                    };
                    let kind = if matches!(entry.state.state, 0 | 1) {
                        NavEvidenceKind::Routine(NavRoutineEvidenceKind::AwaitingPathRefresh)
                    } else if entry.state.state == 2 {
                        NavEvidenceKind::Routine(NavRoutineEvidenceKind::ActiveInputRefresh)
                    } else if matches!(entry.state.state, 3 | 4) {
                        NavEvidenceKind::Critical(NavCriticalEvidenceKind::Terminal)
                    } else {
                        NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                    };
                    nav_state_ledger.record(kind, entry);
                }
                _ = tokio::time::sleep_until(second_path_deadline) => {
                    body_failure!("post-terminal PathEnvelope ledger timeout")
                }
            }
        }
    };
    let second_path = second_path_entry.path.clone();
    if path_ledger
        .back()
        .map_or(true, |entry| entry.ordinal != second_path_entry.ordinal)
    {
        assert!(path_ledger.len() < 64, "V2 PathEnvelope ledger overflow");
        path_ledger.push_back(second_path_entry);
    }
    assert_eq!(second_path.session_id, second_goal.session_id);
    assert_eq!(second_path.goal_generation, second_goal.goal_generation);
    assert_eq!(second_path.producer_runtime_generation, second_goal.producer_runtime_generation);

    let second_child_lease = next_active
        .child_lease
        .expect("post-terminal child cleanup lease");
    let second_terminal = explore
        .report_navigation_result(
            &next_active.child_task_id,
            crate::explore_runtime::ChildNavigationResult::Failed {
                proof: crate::explore_runtime::ChildNavigationTerminalProof {
                    lease: second_child_lease,
                },
                reason: "V2 canary cleanup".to_string(),
            },
        )
        .await
        .expect("release post-terminal child ownership");
        assert!(second_terminal.child_task_id.is_empty());
        Ok::<(), String>(())
    })
    .catch_unwind()
    .await;

    let cleanup_report = cleanup_v2_canary(
        &mut resources,
        &mut path_ledger_rx,
        &mut path_ledger,
        &mut nav_state_ledger_rx,
        &mut nav_state_ledger,
    )
    .await;
    finish_v2_canary(body_outcome, cleanup_report);
}

fn acceptance_test_goal() -> schema::GoalEnvelope {
    schema::GoalEnvelope {
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: "acceptance-test".to_string(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: "acceptance-parent".to_string(),
        goal_generation: 11,
        producer: "acceptance-producer".to_string(),
        producer_epoch: 12,
        producer_runtime_generation: 13,
        committed_map_session_id: "acceptance-map".to_string(),
        committed_map_session_generation: 14,
        committed_map_revision: 15,
        committed_map_boot_nonce: [0x16; 16],
        committed_map_commit_sequence: 16,
        committed_map_source_timestamp_ns: 17,
        committed_map_frame_id: "map".to_string(),
        boundary_id: "acceptance-boundary".to_string(),
        motion_guard_generation: 18,
        motion_guard_boot_nonce: [0x19; 16],
        motion_stop_generation: 20,
        body_reference_frame_id: "base_link".into(),
        ..schema::GoalEnvelope::default()
    }
}

fn acceptance_test_path(
    goal: &schema::GoalEnvelope,
    ordinal: u64,
    revision: i64,
) -> PathLedgerEntry {
    let mut path = schema::PathEnvelope {
        schema_version: goal.schema_version,
        session_id: goal.session_id.clone(),
        owner_kind: goal.owner_kind,
        parent_request_id: goal.parent_request_id.clone(),
        goal_generation: goal.goal_generation,
        producer: goal.producer.clone(),
        producer_epoch: goal.producer_epoch,
        producer_runtime_generation: goal.producer_runtime_generation,
        committed_map_session_id: goal.committed_map_session_id.clone(),
        committed_map_session_generation: goal.committed_map_session_generation,
        committed_map_revision: goal.committed_map_revision,
        committed_map_boot_nonce: goal.committed_map_boot_nonce,
        committed_map_commit_sequence: goal.committed_map_commit_sequence,
        committed_map_source_timestamp_ns: goal.committed_map_source_timestamp_ns,
        committed_map_frame_id: goal.committed_map_frame_id.clone(),
        boundary_id: goal.boundary_id.clone(),
        motion_guard_generation: goal.motion_guard_generation,
        motion_guard_boot_nonce: goal.motion_guard_boot_nonce,
        motion_stop_generation: goal.motion_stop_generation,
        planner_runtime_generation: 22,
        path_revision: revision,
        source_timestamp_ns: 2_000_000_000,

        path: schema::Path {
            header: schema::Header {
                stamp_sec: 2,
                frame_id: "map".to_string(),
                ..schema::Header::default()
            },
            poses: vec![schema::PoseStamped {
                header: schema::Header {
                    stamp_sec: 2,
                    frame_id: "map".to_string(),
                    ..schema::Header::default()
                },
                pose: goal.goal.pose.clone(),
            }],
        },
        ..schema::PathEnvelope::default()
    };
    schema::validate_path_envelope(&path).expect("seal acceptance test path");
    PathLedgerEntry {
        ordinal,
        elapsed_ns: u128::from(ordinal),
        path,
    }
}

fn acceptance_test_state(path: &schema::PathEnvelope) -> schema::NavigationState {
    schema::NavigationState {
        session_id: path.session_id.clone(),
        state: 2,
        owner_kind: path.owner_kind,
        parent_request_id: path.parent_request_id.clone(),
        goal_generation: path.goal_generation,
        path_revision: path.path_revision,
        producer_runtime_generation: path.producer_runtime_generation,
        planner_runtime_generation: path.planner_runtime_generation,
        ..schema::NavigationState::default()
    }
}

#[test]
fn canonical_path_candidate_requires_exact_committed_map_receipt() {
    let goal = acceptance_test_goal();
    let candidate = acceptance_test_path(&goal, 1, 1);
    assert!(is_canonical_path_candidate(&candidate, &goal));
    for field in 0..3 {
        let mut conflict = candidate.clone();
        match field {
            0 => conflict.path.committed_map_boot_nonce[0] ^= 1,
            1 => conflict.path.committed_map_commit_sequence += 1,
            _ => conflict.path.committed_map_source_timestamp_ns += 1,
        }
        // Reusing an assigned path key cannot repair mismatched map provenance.
        assert!(!is_canonical_path_candidate(&conflict, &goal));
    }
}

#[test]
fn accepted_path_candidate_matches_later_canonical_revision() {
    let goal = acceptance_test_goal();
    let revision_one = acceptance_test_path(&goal, 1, 1);
    let revision_two = acceptance_test_path(&goal, 2, 2);
    let state = acceptance_test_state(&revision_two.path);
    let candidates = VecDeque::from([revision_one, revision_two]);

    let AcceptedPathCandidateMatch::Matched(matched) =
        match_accepted_path_candidate(&state, &goal, &candidates)
    else {
        panic!("revision 2 must match instead of binding revision 1")
    };
    assert_eq!(matched.ordinal, 2);
    assert_eq!(matched.path.path_revision, 2);
}

#[test]
fn accepted_path_candidate_waits_for_state_before_path_reordering() {
    let goal = acceptance_test_goal();
    let revision_two = acceptance_test_path(&goal, 2, 2);
    let state = acceptance_test_state(&revision_two.path);
    let mut candidates = VecDeque::new();

    assert!(matches!(
        match_accepted_path_candidate(&state, &goal, &candidates),
        AcceptedPathCandidateMatch::Pending
    ));
    candidates.push_back(revision_two);
    assert!(matches!(
        match_accepted_path_candidate(&state, &goal, &candidates),
        AcceptedPathCandidateMatch::Matched(_)
    ));
}

#[test]
fn accepted_path_candidate_rejects_identity_planner_and_revision_conflicts() {
    let goal = acceptance_test_goal();
    let revision_two = acceptance_test_path(&goal, 2, 2);
    let candidates = VecDeque::from([revision_two.clone()]);

    let mut wrong_identity = acceptance_test_state(&revision_two.path);
    wrong_identity.parent_request_id.push_str("-wrong");
    assert!(matches!(
        match_accepted_path_candidate(&wrong_identity, &goal, &candidates),
        AcceptedPathCandidateMatch::Conflict
    ));

    let mut wrong_planner = acceptance_test_state(&revision_two.path);
    wrong_planner.planner_runtime_generation += 1;
    assert!(matches!(
        match_accepted_path_candidate(&wrong_planner, &goal, &candidates),
        AcceptedPathCandidateMatch::Conflict
    ));

    let mut wrong_revision = acceptance_test_state(&revision_two.path);
    wrong_revision.path_revision += 1;
    assert!(matches!(
        match_accepted_path_candidate(&wrong_revision, &goal, &candidates),
        AcceptedPathCandidateMatch::Conflict
    ));
}

#[test]
fn pending_accepted_state_advances_before_late_old_candidate() {
    let goal = acceptance_test_goal();
    let revision_one = acceptance_test_path(&goal, 1, 1);
    let revision_two = acceptance_test_path(&goal, 2, 2);
    let state_one = acceptance_test_state(&revision_one.path);
    let state_two = acceptance_test_state(&revision_two.path);
    let mut pending = state_one;
    let mut candidates = VecDeque::new();

    assert_eq!(
        classify_pending_accepted_state_update(&pending, &state_two, &goal),
        PendingAcceptedStateUpdate::Advance
    );
    pending = state_two;
    candidates.push_back(revision_one);
    assert!(matches!(
        match_accepted_path_candidate(&pending, &goal, &candidates),
        AcceptedPathCandidateMatch::Pending
    ));
    candidates.push_back(revision_two);
    let AcceptedPathCandidateMatch::Matched(matched) =
        match_accepted_path_candidate(&pending, &goal, &candidates)
    else {
        panic!("pending revision 2 must match only revision 2")
    };
    assert_eq!(matched.path.path_revision, 2);
}

#[test]
fn pending_accepted_state_distinguishes_duplicate_and_revision_rollback() {
    let goal = acceptance_test_goal();
    let revision_one = acceptance_test_path(&goal, 1, 1);
    let revision_two = acceptance_test_path(&goal, 2, 2);
    let state_one = acceptance_test_state(&revision_one.path);
    let state_two = acceptance_test_state(&revision_two.path);

    assert_eq!(
        classify_pending_accepted_state_update(&state_two, &state_two, &goal),
        PendingAcceptedStateUpdate::Duplicate
    );

    let mut same_revision_different_planner = state_two.clone();
    same_revision_different_planner.planner_runtime_generation += 1;
    assert_eq!(
        classify_pending_accepted_state_update(&state_two, &same_revision_different_planner, &goal,),
        PendingAcceptedStateUpdate::Conflict
    );

    let mut wrong_identity = state_two.clone();
    wrong_identity.parent_request_id.push_str("-wrong");
    assert_eq!(
        classify_pending_accepted_state_update(&state_two, &wrong_identity, &goal),
        PendingAcceptedStateUpdate::Conflict
    );

    let mut nonpositive_revision = state_two.clone();
    nonpositive_revision.path_revision = 0;
    assert_eq!(
        classify_pending_accepted_state_update(&state_two, &nonpositive_revision, &goal),
        PendingAcceptedStateUpdate::Conflict
    );

    assert_eq!(
        classify_pending_accepted_state_update(&state_two, &state_one, &goal),
        PendingAcceptedStateUpdate::Conflict
    );
}

#[test]
fn canonical_idle_requires_exact_heartbeat_and_empty_identity() {
    let mut idle = schema::NavigationState {
        message: "IDLE: guard client heartbeat".to_string(),
        ..schema::NavigationState::default()
    };
    assert!(is_canonical_idle(&idle));

    idle.message = "IDLE".to_string();
    assert!(!is_canonical_idle(&idle));
    idle.message = "IDLE: guard client heartbeat".to_string();
    idle.state = 3;
    assert!(!is_canonical_idle(&idle));
    idle.state = 0;
    idle.session_id = "owned-session".to_string();
    assert!(!is_canonical_idle(&idle));
}

#[tokio::test]
async fn canary_canonical_idle_feedback_applies_typed_readiness() {
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    let state = schema::NavigationState {
        stamp_sec: 1,
        message: "IDLE: guard client heartbeat".to_string(),
        runtime_generation: 7,
        sink_ready: 1,
        planner_map_frame: "map".to_string(),
        planner_pose_frame: "map".to_string(),
        ..schema::NavigationState::default()
    };
    let payload = schema::encode_navigation_state_body(&state).expect("encode canonical heartbeat");
    assert!(apply_canary_idle_feedback(&nav, &payload).await);
}

#[tokio::test]
async fn canary_real_tf_observation_is_payload_derived_and_fail_closed() {
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    let heartbeat = schema::NavigationState {
        stamp_sec: 1,
        message: "IDLE: guard client heartbeat".to_string(),
        runtime_generation: 7,
        sink_ready: 1,
        planner_map_frame: "map".to_string(),
        planner_pose_frame: "map".to_string(),
        ..schema::NavigationState::default()
    };
    let heartbeat =
        schema::encode_navigation_state_body(&heartbeat).expect("encode canonical heartbeat");
    assert!(apply_canary_idle_feedback(&nav, &heartbeat).await);

    let stamp_ns = crate::explore_runtime::now_unix_nanos();
    let mut history = PublicTfHistory::default();
    let mut generation = 0_u64;
    let cases = [
        (vec![("map", "odom")], 1_u64),
        (vec![("odom", "base_link")], 2_u64),
    ];
    for (edges, expected_generation) in cases {
        let payload = canary_tf_payload_for_edges(stamp_ns, &edges);
        assert!(apply_canary_tf_payload(
            &nav,
            &mut history,
            &mut generation,
            &payload,
        ));
        assert_eq!(generation, expected_generation);
    }

    for malformed in [Vec::new(), vec![0_u8; 7]] {
        assert!(!apply_canary_tf_payload(
            &nav,
            &mut history,
            &mut generation,
            &malformed,
        ));
        assert_eq!(generation, 2);
    }

    let mut rejected_generation = 0_u64;
    let rejected = canary_tf_payload_for_edges(stamp_ns, &[("map", "odom")]);
    assert!(!apply_canary_tf_payload(
        &nav,
        &mut history,
        &mut rejected_generation,
        &rejected,
    ));
    assert_eq!(rejected_generation, 0);
}

fn canary_readiness_handle(nav: &crate::nav_runtime::NavRuntimeState) -> GeometryRuntimeHandle {
    let transport = LcmTransport::new_real(&test_lcm_url()).expect("test readiness LCM context");
    let (cloud_map_tx, _) = broadcast::channel(1);
    let (nav_state_tx, _) = broadcast::channel(1);
    let (frontiers_tx, _) = broadcast::channel(1);
    let (map_revision_tx, _) = broadcast::channel(1);
    let (map_revision_latest_tx, _) = watch::channel(None);
    ExploreNavigationBridge {
        body_reference_frame_id: Arc::from("base_link"),
        mapping_adapter: None,
        state: Arc::new(RwLock::new(SharedState::default())),
        expected_map_frame: Arc::from("map"),
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx,
        map_revision_latest_tx,
        explore_projection_gate: Arc::new(
            tokio::sync::Mutex::new(ExploreProjectionGate::default()),
        ),
        committed_map_authority: Arc::new(std::sync::Mutex::new(None)),
        mapping_lifecycle_lock: Arc::new(tokio::sync::Mutex::new(())),
        transport,
        transport_backend_name: "real-lcm",
        lcm_backend_available: true,
        cmd_vel_sink_ready: Arc::new(AtomicBool::new(true)),
        nav: nav.clone(),
        exploration_producer_epoch: Arc::new(AtomicU64::new(1)),
        runtime_observation_generation: Arc::new(AtomicU64::new(1)),
        orphan_cancellations: Arc::new(
            crate::explore_runtime::OrphanCancellationRegistry::default(),
        ),
    }
    .facade_handle()
}

async fn observe_canary_ready_inputs(
    nav: &crate::nav_runtime::NavRuntimeState,
    sink_ready: bool,
    map_frame: &str,
    pose_frame: &str,
) {
    nav.observe_stack_state(NavState {
        status: "IDLE".to_string(),
        timestamp_ns: crate::explore_runtime::now_unix_nanos(),
        ..NavState::default()
    })
    .await;
    nav.observe_planner_input(crate::nav_runtime::PlannerInputObservation {
        generation: 1,
        map_revision: 1,
        map_frame: map_frame.to_string(),
        pose_frame: pose_frame.to_string(),
        ready: true,
    })
    .expect("planner readiness observation");
    nav.observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
        generation: 1,
        ready: sink_ready,
    })
    .expect("motion sink readiness observation");
}

#[tokio::test]
async fn post_quiet_tf_window_uses_formal_readiness() {
    use crate::nav_runtime::NavigationReadinessCode;

    let stamp_ns = crate::explore_runtime::now_unix_nanos();

    let canonical_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let canonical_handle = canary_readiness_handle(&canonical_nav);
    let heartbeat = schema::NavigationState {
        stamp_sec: 1,
        message: "IDLE: guard client heartbeat".to_string(),
        runtime_generation: 7,
        sink_ready: 1,
        planner_map_frame: "map".to_string(),
        planner_pose_frame: "map".to_string(),
        ..schema::NavigationState::default()
    };
    let heartbeat = schema::encode_navigation_state_body(&heartbeat).expect("heartbeat payload");
    assert!(apply_canary_idle_feedback(&canonical_nav, &heartbeat).await);
    let canonical = crate::nav_runtime::compute_readiness(&canonical_handle, &canonical_nav).await;
    assert!(!canonical.tf_observed);
    assert!(canonical
        .blockers
        .contains(&NavigationReadinessCode::TfUnobserved));

    for (sink_ready, map_frame, pose_frame, expected) in [
        (true, "map", "map", NavigationReadinessCode::TfIncomplete),
        (
            false,
            "map",
            "map",
            NavigationReadinessCode::MotionSinkNotReady,
        ),
        (
            true,
            "map",
            "odom",
            NavigationReadinessCode::PlannerFrameMismatch,
        ),
    ] {
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        let handle = canary_readiness_handle(&nav);
        observe_canary_ready_inputs(&nav, sink_ready, map_frame, pose_frame).await;
        let mut history = PublicTfHistory::default();
        let mut generation = 0;
        let payload = if expected == NavigationReadinessCode::TfIncomplete {
            canary_tf_payload_for_edges(stamp_ns, &[("map", "odom")])
        } else {
            canary_tf_payload(stamp_ns)
        };
        assert!(apply_canary_tf_payload(
            &nav,
            &mut history,
            &mut generation,
            &payload
        ));
        let detail = crate::nav_runtime::compute_readiness(&handle, &nav).await;
        assert!(!detail.ready);
        assert!(detail.blockers.contains(&expected), "{detail:#?}");
    }

    let malformed_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let malformed_handle = canary_readiness_handle(&malformed_nav);
    observe_canary_ready_inputs(&malformed_nav, true, "map", "map").await;
    let mut malformed_history = PublicTfHistory::default();
    let mut malformed_generation = 0;
    assert!(!apply_canary_tf_payload(
        &malformed_nav,
        &mut malformed_history,
        &mut malformed_generation,
        &[0_u8; 7],
    ));
    let malformed = crate::nav_runtime::compute_readiness(&malformed_handle, &malformed_nav).await;
    assert!(!malformed.ready);
    assert!(malformed
        .blockers
        .contains(&NavigationReadinessCode::TfUnobserved));

    let stale_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let stale_handle = canary_readiness_handle(&stale_nav);
    observe_canary_ready_inputs(&stale_nav, true, "map", "map").await;
    let mut stale_history = PublicTfHistory::default();
    let mut stale_generation = 0;
    assert!(apply_canary_tf_payload(
        &stale_nav,
        &mut stale_history,
        &mut stale_generation,
        &canary_tf_payload(stamp_ns),
    ));
    stale_nav.expire_tf_for_test();
    let stale = crate::nav_runtime::compute_readiness(&stale_handle, &stale_nav).await;
    assert!(!stale.ready);
    assert!(stale.blockers.contains(&NavigationReadinessCode::TfStale));

    let old_ready_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let old_ready_handle = canary_readiness_handle(&old_ready_nav);
    observe_canary_ready_inputs(&old_ready_nav, true, "map", "map").await;
    let mut old_history = PublicTfHistory::default();
    let mut old_generation = 0;
    assert!(apply_canary_tf_payload(
        &old_ready_nav,
        &mut old_history,
        &mut old_generation,
        &canary_tf_payload(stamp_ns),
    ));
    let old_ready = crate::nav_runtime::compute_readiness(&old_ready_handle, &old_ready_nav).await;
    assert!(old_ready.ready);
    let old_gate = PostQuietTfGate {
        baseline_count: old_ready.tf_count,
        baseline_generation: old_ready.tf_generation,
        fresh_complete_payload_applied: false,
    };
    assert_eq!(usize::from(old_gate.allows_dispatch(&old_ready)), 0);

    let mut split_gate = PostQuietTfGate {
        baseline_count: old_ready.tf_count.saturating_sub(1),
        baseline_generation: old_ready.tf_generation.saturating_sub(1),
        fresh_complete_payload_applied: false,
    };
    split_gate.observe_payload_result(true, false);
    split_gate.observe_payload_result(false, true);
    assert_eq!(usize::from(split_gate.allows_dispatch(&old_ready)), 0);

    let rejected_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let rejected_handle = canary_readiness_handle(&rejected_nav);
    observe_canary_ready_inputs(&rejected_nav, true, "map", "map").await;
    let mut rejected_history = PublicTfHistory::default();
    let mut accepted_generation = 1;
    assert!(apply_canary_tf_payload(
        &rejected_nav,
        &mut rejected_history,
        &mut accepted_generation,
        &canary_tf_payload_for_edges(stamp_ns, &[("map", "odom")]),
    ));
    let rejected_baseline =
        crate::nav_runtime::compute_readiness(&rejected_handle, &rejected_nav).await;
    let mut rejected_proof = PublicTfHistory::default();
    let rejected_payload = canary_tf_payload(stamp_ns);
    let rejected_proof_complete = canary_tf_window_complete(&mut rejected_proof, &rejected_payload);
    let mut stale_generation = 0;
    let rejected_applied = apply_canary_tf_payload(
        &rejected_nav,
        &mut rejected_history,
        &mut stale_generation,
        &rejected_payload,
    );
    let mut rejected_gate = PostQuietTfGate {
        baseline_count: rejected_baseline.tf_count.saturating_sub(1),
        baseline_generation: rejected_baseline.tf_generation.saturating_sub(1),
        fresh_complete_payload_applied: false,
    };
    rejected_gate.observe_payload_result(rejected_applied, rejected_proof_complete);
    let rejected = crate::nav_runtime::compute_readiness(&rejected_handle, &rejected_nav).await;
    assert!(!rejected.ready);
    assert!(rejected
        .blockers
        .contains(&NavigationReadinessCode::TfIncomplete));
    assert!(!rejected.reason.is_empty());
    assert_eq!(usize::from(rejected_gate.allows_dispatch(&rejected)), 0);

    let timeout_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let timeout_handle = canary_readiness_handle(&timeout_nav);
    observe_canary_ready_inputs(&timeout_nav, true, "map", "map").await;
    let timeout = crate::nav_runtime::compute_readiness(&timeout_handle, &timeout_nav).await;
    let timeout_gate = PostQuietTfGate {
        baseline_count: timeout.tf_count,
        baseline_generation: timeout.tf_generation,
        fresh_complete_payload_applied: false,
    };
    assert!(!timeout.reason.is_empty());
    assert!(!timeout.blockers.is_empty());
    assert_eq!(usize::from(timeout_gate.allows_dispatch(&timeout)), 0);

    let ready_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let ready_handle = canary_readiness_handle(&ready_nav);
    observe_canary_ready_inputs(&ready_nav, true, "map", "map").await;
    let baseline = crate::nav_runtime::compute_readiness(&ready_handle, &ready_nav).await;
    let mut persistent = PublicTfHistory::default();
    let mut proof = PublicTfHistory::default();
    let mut generation = 0;
    let mut gate = PostQuietTfGate {
        baseline_count: baseline.tf_count,
        baseline_generation: baseline.tf_generation,
        fresh_complete_payload_applied: false,
    };
    for edges in [
        [("map", "odom")].as_slice(),
        [("odom", "base_link")].as_slice(),
    ] {
        let payload = canary_tf_payload_for_edges(stamp_ns, edges);
        let proof_complete = canary_tf_window_complete(&mut proof, &payload);
        let applied =
            apply_canary_tf_payload(&ready_nav, &mut persistent, &mut generation, &payload);
        gate.observe_payload_result(applied, proof_complete);
    }
    let ready = crate::nav_runtime::compute_readiness(&ready_handle, &ready_nav).await;
    assert!(ready.tf_count > baseline.tf_count);
    assert!(ready.tf_generation > baseline.tf_generation);
    assert!(ready.ready, "{ready:#?}");
    assert_eq!(usize::from(gate.allows_dispatch(&ready)), 1);

    let single_nav = crate::nav_runtime::NavRuntimeState::from_env();
    let single_handle = canary_readiness_handle(&single_nav);
    observe_canary_ready_inputs(&single_nav, true, "map", "map").await;
    let single_baseline = crate::nav_runtime::compute_readiness(&single_handle, &single_nav).await;
    let mut single_history = PublicTfHistory::default();
    let mut single_proof = PublicTfHistory::default();
    let mut single_generation = 0;
    let single_payload = canary_tf_payload(stamp_ns);
    let single_complete = canary_tf_window_complete(&mut single_proof, &single_payload);
    let single_applied = apply_canary_tf_payload(
        &single_nav,
        &mut single_history,
        &mut single_generation,
        &single_payload,
    );
    let mut single_gate = PostQuietTfGate {
        baseline_count: single_baseline.tf_count,
        baseline_generation: single_baseline.tf_generation,
        fresh_complete_payload_applied: false,
    };
    single_gate.observe_payload_result(single_applied, single_complete);
    let single_ready = crate::nav_runtime::compute_readiness(&single_handle, &single_nav).await;
    assert!(single_ready.ready, "{single_ready:#?}");
    assert_eq!(usize::from(single_gate.allows_dispatch(&single_ready)), 1);
}

struct OwnedCleanupSpy {
    resource: Option<Box<u8>>,
    reaped: Arc<Mutex<usize>>,
}

impl OwnedCleanupSpy {
    fn new(reaped: Arc<Mutex<usize>>) -> Self {
        Self {
            resource: Some(Box::new(1)),
            reaped,
        }
    }

    fn reap(&mut self, failure: &InitialReadinessGateFailure) {
        assert!(failure.cleanup_required);
        assert!(
            self.resource.take().is_some(),
            "resource must be owned once"
        );
        *self.reaped.lock().expect("cleanup spy lock poisoned") += 1;
    }

    fn resource_count(&self) -> usize {
        usize::from(self.resource.is_some())
    }
}

impl Drop for OwnedCleanupSpy {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            assert!(
                self.resource.is_none(),
                "initial gate failure ignored required cleanup"
            );
        }
    }
}

#[tokio::test]
async fn initial_readiness_uses_typed_feedback_generation() {
    use crate::nav_runtime::NavigationReadinessCode;

    #[derive(Clone, Copy)]
    enum GateCase {
        Ready,
        ReadyCanonicalThenNoncanonical,
        ReadyNavFlood,
        ReadyExpiredNavFlood,
        ReadyDeadlineAfterNavDrain,
        ReadyDeadlineAfterFinalReadiness,
        PendingExpiredNavInput,
        PendingExpiredGoal,
        ReadyGoal,
        ReadyPath,
        ReadyTwist,
        NoHeartbeat,
        PlannerMissing,
        SinkMissing,
        SinkFalse,
        NavLedgerClosed,
        PathLedgerClosed,
    }

    async fn run_case(
        case: GateCase,
        stamp_ns: u64,
    ) -> Result<InitialReadinessGateOutcome, InitialReadinessGateFailure> {
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        let handle = canary_readiness_handle(&nav);
        let heartbeat = |runtime_generation: i64, guard_generation: i64, sink_ready: i8| {
            schema::encode_navigation_state_body(&schema::NavigationState {
                stamp_sec: 1,
                message: "IDLE: guard client heartbeat".to_string(),
                runtime_generation,
                guard_generation,
                sink_ready,
                planner_map_frame: "map".to_string(),
                planner_pose_frame: "map".to_string(),
                ..schema::NavigationState::default()
            })
            .expect("encode typed initial-readiness heartbeat")
        };
        let heartbeat = match case {
            GateCase::Ready
            | GateCase::ReadyCanonicalThenNoncanonical
            | GateCase::ReadyNavFlood
            | GateCase::ReadyExpiredNavFlood
            | GateCase::ReadyDeadlineAfterNavDrain
            | GateCase::ReadyDeadlineAfterFinalReadiness
            | GateCase::ReadyGoal
            | GateCase::ReadyPath
            | GateCase::ReadyTwist => Some(heartbeat(41, 37, 1)),
            GateCase::PlannerMissing => Some(heartbeat(0, 37, 1)),
            GateCase::SinkMissing => Some(heartbeat(41, 0, 1)),
            GateCase::SinkFalse => Some(heartbeat(41, 37, 0)),
            GateCase::NoHeartbeat
            | GateCase::PendingExpiredNavInput
            | GateCase::PendingExpiredGoal
            | GateCase::NavLedgerClosed
            | GateCase::PathLedgerClosed => None,
        };

        let nav_capacity = if matches!(
            case,
            GateCase::ReadyNavFlood | GateCase::ReadyExpiredNavFlood
        ) {
            NAV_STATE_EVIDENCE_CAPACITY + 1
        } else {
            2
        };
        let (nav_tx, mut nav_rx) = tokio::sync::mpsc::channel(nav_capacity);
        if matches!(
            case,
            GateCase::ReadyNavFlood | GateCase::ReadyExpiredNavFlood
        ) {
            for ordinal in 1..=(NAV_STATE_EVIDENCE_CAPACITY as u64 + 1) {
                nav_tx
                    .send(NavStateLedgerEntry {
                        ordinal,
                        elapsed_ns: ordinal as u128,
                        state: schema::NavigationState {
                            message: "IDLE: guard client heartbeat".to_string(),
                            runtime_generation: 41,
                            guard_generation: 37,
                            sink_ready: 1,
                            planner_map_frame: "map".to_string(),
                            planner_pose_frame: "map".to_string(),
                            ..schema::NavigationState::default()
                        },
                        feedback_applied: true,
                    })
                    .await
                    .expect("prefill bounded NavigationState flood");
            }
        }
        if matches!(case, GateCase::PendingExpiredNavInput) {
            nav_tx
                .send(NavStateLedgerEntry {
                    ordinal: 1,
                    elapsed_ns: 1,
                    state: schema::NavigationState {
                        message: "IDLE: guard client heartbeat".to_string(),
                        runtime_generation: 41,
                        guard_generation: 37,
                        sink_ready: 1,
                        planner_map_frame: "map".to_string(),
                        planner_pose_frame: "map".to_string(),
                        ..schema::NavigationState::default()
                    },
                    feedback_applied: true,
                })
                .await
                .expect("queue pending input behind expired deadline");
        }
        if matches!(case, GateCase::ReadyCanonicalThenNoncanonical) {
            nav_tx
                .send(NavStateLedgerEntry {
                    ordinal: 1,
                    elapsed_ns: 1,
                    state: schema::NavigationState {
                        message: "IDLE: guard client heartbeat".to_string(),
                        runtime_generation: 41,
                        guard_generation: 37,
                        sink_ready: 1,
                        planner_map_frame: "map".to_string(),
                        planner_pose_frame: "map".to_string(),
                        ..schema::NavigationState::default()
                    },
                    feedback_applied: true,
                })
                .await
                .expect("queue canonical state before noncanonical state");
        }
        if matches!(case, GateCase::ReadyCanonicalThenNoncanonical) {
            nav_tx
                .send(NavStateLedgerEntry {
                    ordinal: 2,
                    elapsed_ns: 1,
                    state: schema::NavigationState {
                        state: 1,
                        message: "ACTIVE before readiness".to_string(),
                        session_id: "unexpected".to_string(),
                        ..schema::NavigationState::default()
                    },
                    feedback_applied: false,
                })
                .await
                .expect("queue noncanonical state");
        }
        let nav_tx = if matches!(case, GateCase::NavLedgerClosed) {
            drop(nav_tx);
            None
        } else {
            Some(nav_tx)
        };

        let (path_tx, mut path_rx) = tokio::sync::mpsc::channel(1);
        if matches!(case, GateCase::ReadyPath) {
            path_tx
                .send(PathLedgerEntry {
                    ordinal: 1,
                    elapsed_ns: 1,
                    path: schema::PathEnvelope::default(),
                })
                .await
                .expect("queue pre-release PathEnvelope");
        }
        let path_tx = if matches!(case, GateCase::PathLedgerClosed) {
            drop(path_tx);
            None
        } else {
            Some(path_tx)
        };
        let provider = test_lcm_url();
        let transport = LcmTransport::new_real(&provider).expect("gate test LCM context");
        let goal_channel = format!("/v2/gate/{}/goal", crate::explore_runtime::now_unix_nanos());
        let goal_topic = LcmTopic::new(&goal_channel).expect("gate goal topic");
        let goal_type = LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("gate goal type");
        let mut goal_frames = transport.subscribe(goal_topic, Some(goal_type)).await.peekable();
        let goal_publisher = LcmTransport::new_real(&provider)
            .expect("independent gate goal publisher context");
        if matches!(
            case,
            GateCase::ReadyGoal
                | GateCase::PendingExpiredGoal
                | GateCase::ReadyDeadlineAfterNavDrain
        ) {
            goal_publisher
                .publish(typed_frame(
                    &goal_channel,
                    "nuwax_msgs.GoalEnvelope",
                    Vec::new(),
                    1,
                ))
                .await
                .expect("publish pre-release GoalEnvelope");
            let delivered = tokio::time::timeout(
                Duration::from_secs(2),
                std::pin::Pin::new(&mut goal_frames).peek(),
            ).await.expect("真实目标帧必须到达订阅端，不能用固定休眠猜测就绪");
            assert!(matches!(delivered, Some(Ok(_))));
        }
        let mut goal_frames: nuwax_transport::LcmFrameStream = Box::pin(goal_frames);
        if let Some(payload) = heartbeat {
            let update = crate::typed_codec::decode_nav_state_payload(&payload)
                .expect("decode typed initial-readiness heartbeat");
            assert!(apply_typed_navigation_feedback(&nav, update)
                .await
                .is_some());
        }
        if matches!(case, GateCase::SinkMissing) {
            nav.clear_motion_sink_for_test();
        }
        let mut tf_history = PublicTfHistory::default();
        let mut tf_generation = 0;
        assert!(apply_canary_tf_payload(
            &nav,
            &mut tf_history,
            &mut tf_generation,
            &canary_tf_payload(stamp_ns),
        ));

        let mut evidence = NavStateEvidenceLedger::new();
        let mut guard_events = if matches!(case, GateCase::ReadyTwist) {
            "TWIST 0.1 0.0\n".to_string()
        } else {
            String::new()
        };
        let mut read_guard_events = || guard_events.clone();
        let gate_deadline = match case {
            GateCase::Ready
            | GateCase::ReadyCanonicalThenNoncanonical
            | GateCase::ReadyGoal
            | GateCase::ReadyPath
            | GateCase::ReadyTwist
            | GateCase::NavLedgerClosed
            | GateCase::PathLedgerClosed
            | GateCase::ReadyNavFlood
            | GateCase::ReadyDeadlineAfterNavDrain
            | GateCase::ReadyDeadlineAfterFinalReadiness => {
                tokio::time::Instant::now() + Duration::from_secs(2)
            }
            GateCase::ReadyExpiredNavFlood
            | GateCase::PendingExpiredNavInput
            | GateCase::PendingExpiredGoal => tokio::time::Instant::now(),
            _ => tokio::time::Instant::now() + Duration::from_millis(5),
        };
        let expire_at = match case {
            GateCase::ReadyDeadlineAfterNavDrain => {
                Some(InitialReadinessDeadlineCheckpoint::AfterNavDrain)
            }
            GateCase::ReadyDeadlineAfterFinalReadiness => {
                Some(InitialReadinessDeadlineCheckpoint::AfterFinalReadiness)
            }
            _ => None,
        };
        let mut deadline_checkpoint = |checkpoint| expire_at == Some(checkpoint);
        let outcome = await_initial_navigation_readiness(
            &handle,
            &nav,
            &mut nav_rx,
            &mut evidence,
            &mut goal_frames,
            &mut path_rx,
            &mut read_guard_events,
            gate_deadline,
            &mut deadline_checkpoint,
        )
        .await;
        if matches!(case, GateCase::ReadyDeadlineAfterNavDrain) {
            assert!(matches!(
                goal_frames.next().now_or_never(),
                Some(Some(Ok(_)))
            ));
        }
        if matches!(case, GateCase::ReadyDeadlineAfterFinalReadiness) {
            let failure = outcome
                .as_ref()
                .expect_err("post-compute deadline must prevent release");
            assert!(failure.detail.ready, "{:#?}", failure.detail);
            assert_eq!(failure.detail.planner_generation, 41);
            assert_eq!(failure.detail.motion_sink_generation, 37);
        }
        match case {
            GateCase::ReadyCanonicalThenNoncanonical => {
                assert_eq!(evidence.retained_count, 2);
                let critical_runs = evidence
                    .runs
                    .iter()
                    .filter(|run| {
                        run.kind
                            == NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
                    })
                    .collect::<Vec<_>>();
                assert_eq!(critical_runs.len(), 1);
                assert_eq!(critical_runs[0].observed_count, 1);
                assert_eq!(critical_runs[0].first.ordinal, 2);
                assert_eq!(critical_runs[0].latest.ordinal, 2);
            }
            GateCase::ReadyNavFlood => {
                assert_eq!(evidence.retained_count, 2);
                assert_eq!(evidence.runs.len(), 1);
                let flood_run = evidence.runs.front().expect("bounded flood evidence run");
                assert_eq!(
                    flood_run.kind,
                    NavEvidenceKind::Routine(NavRoutineEvidenceKind::PreGateIdleHeartbeat)
                );
                assert_eq!(
                    flood_run.observed_count,
                    NAV_STATE_EVIDENCE_CAPACITY as u64 + 1
                );
            }
            GateCase::ReadyExpiredNavFlood
            | GateCase::ReadyDeadlineAfterNavDrain
            | GateCase::ReadyDeadlineAfterFinalReadiness
            | GateCase::PendingExpiredNavInput
            | GateCase::PendingExpiredGoal => {
                assert_eq!(evidence.retained_count, 0);
                assert!(evidence.runs.is_empty());
            }
            _ => {}
        }
        drop(nav_tx);
        drop(path_tx);
        guard_events.clear();
        outcome
    }

    let stamp_ns = crate::explore_runtime::now_unix_nanos();

    let ready = run_case(GateCase::Ready, stamp_ns)
        .await
        .expect("high-generation typed heartbeat must release initial gate");
    assert!(ready.detail.ready, "{:#?}", ready.detail);
    assert_eq!(ready.detail.planner_generation, 41);
    assert_eq!(ready.detail.motion_sink_generation, 37);
    assert_eq!(ready.release_count, 1);
    assert_eq!(ready.dispatches, InitialReadinessDispatchLedger::default());

    let reaped_multiframe = Arc::new(Mutex::new(0_usize));
    let mut multiframe_cleanup_spy = OwnedCleanupSpy::new(reaped_multiframe.clone());
    let multiframe = run_case(GateCase::ReadyCanonicalThenNoncanonical, stamp_ns)
        .await
        .expect_err("second queued noncanonical state must prevent release");
    assert!(
        multiframe
            .reason
            .contains("noncanonical NavigationState before initial readiness"),
        "{}",
        multiframe.render()
    );
    assert_eq!(multiframe.release_count, 0);
    assert_eq!(
        multiframe.dispatches,
        InitialReadinessDispatchLedger::default()
    );
    assert!(multiframe.cleanup_required);
    multiframe_cleanup_spy.reap(&multiframe);
    assert_eq!(multiframe_cleanup_spy.resource_count(), 0);
    assert_eq!(
        *reaped_multiframe
            .lock()
            .expect("multiframe cleanup spy lock poisoned"),
        1
    );
    drop(multiframe_cleanup_spy);

    let reaped_flood = Arc::new(Mutex::new(0_usize));
    let mut flood_cleanup_spy = OwnedCleanupSpy::new(reaped_flood.clone());
    let flood = tokio::time::timeout(
        Duration::from_secs(3),
        run_case(GateCase::ReadyNavFlood, stamp_ns),
    )
    .await
    .expect("bounded NavigationState flood gate must return")
    .expect_err("NavigationState activity flood must prevent release");
    assert_eq!(
        flood.reason,
        "NavigationState activity flood before initial readiness"
    );
    assert_eq!(flood.release_count, 0);
    assert_eq!(flood.dispatches, InitialReadinessDispatchLedger::default());
    assert!(flood.cleanup_required);
    flood_cleanup_spy.reap(&flood);
    assert_eq!(flood_cleanup_spy.resource_count(), 0);
    assert_eq!(
        *reaped_flood
            .lock()
            .expect("flood cleanup spy lock poisoned"),
        1
    );
    drop(flood_cleanup_spy);

    let expired_flood = tokio::time::timeout(
        Duration::from_secs(3),
        run_case(GateCase::ReadyExpiredNavFlood, stamp_ns),
    )
    .await
    .expect("expired flood deadline must return")
    .expect_err("expired deadline must prevent flood drain release");
    assert_eq!(expired_flood.reason, "initial navigation readiness timeout");
    assert_eq!(expired_flood.release_count, 0);
    assert_eq!(
        expired_flood.dispatches,
        InitialReadinessDispatchLedger::default()
    );
    assert!(expired_flood.cleanup_required);

    for case in [
        GateCase::ReadyDeadlineAfterNavDrain,
        GateCase::ReadyDeadlineAfterFinalReadiness,
    ] {
        let reaped_ready_deadline = Arc::new(Mutex::new(0_usize));
        let mut ready_deadline_cleanup_spy = OwnedCleanupSpy::new(reaped_ready_deadline.clone());
        let ready_deadline =
            tokio::time::timeout(Duration::from_secs(3), run_case(case, stamp_ns))
                .await
                .expect("ready credential deadline checkpoint must return")
                .expect_err("ready credential deadline must prevent release");
        assert_eq!(
            ready_deadline.reason,
            "initial navigation readiness timeout"
        );
        assert_eq!(ready_deadline.release_count, 0);
        assert_eq!(
            ready_deadline.dispatches,
            InitialReadinessDispatchLedger::default()
        );
        assert!(ready_deadline.cleanup_required);
        ready_deadline_cleanup_spy.reap(&ready_deadline);
        assert_eq!(ready_deadline_cleanup_spy.resource_count(), 0);
        assert_eq!(
            *reaped_ready_deadline
                .lock()
                .expect("ready deadline cleanup spy lock poisoned"),
            1
        );
        drop(ready_deadline_cleanup_spy);
    }

    for case in [
        GateCase::PendingExpiredNavInput,
        GateCase::PendingExpiredGoal,
    ] {
        let reaped_expired_pending = Arc::new(Mutex::new(0_usize));
        let mut expired_pending_cleanup_spy = OwnedCleanupSpy::new(reaped_expired_pending.clone());
        let expired_pending =
            tokio::time::timeout(Duration::from_secs(3), run_case(case, stamp_ns))
                .await
                .expect("expired pending input gate must return")
                .expect_err("expired deadline must win over pending input");
        assert_eq!(
            expired_pending.reason,
            "initial navigation readiness timeout"
        );
        assert_eq!(expired_pending.release_count, 0);
        assert_eq!(
            expired_pending.dispatches,
            InitialReadinessDispatchLedger::default()
        );
        assert!(expired_pending.cleanup_required);
        expired_pending_cleanup_spy.reap(&expired_pending);
        assert_eq!(expired_pending_cleanup_spy.resource_count(), 0);
        assert_eq!(
            *reaped_expired_pending
                .lock()
                .expect("expired pending cleanup spy lock poisoned"),
            1
        );
        drop(expired_pending_cleanup_spy);
    }

    for (case, reason, expected_dispatches) in [
        (
            GateCase::ReadyGoal,
            "goal dispatched before initial readiness",
            InitialReadinessDispatchLedger {
                goal: 1,
                path: 0,
                twist: 0,
            },
        ),
        (
            GateCase::ReadyPath,
            "path dispatched before initial readiness",
            InitialReadinessDispatchLedger {
                goal: 0,
                path: 1,
                twist: 0,
            },
        ),
        (
            GateCase::ReadyTwist,
            "motion dispatched before initial readiness",
            InitialReadinessDispatchLedger {
                goal: 0,
                path: 0,
                twist: 1,
            },
        ),
    ] {
        let failure = run_case(case, stamp_ns)
            .await
            .expect_err("queued pre-release activity must prevent release");
        assert!(failure.reason.contains(reason), "{}", failure.render());
        assert_eq!(failure.dispatches, expected_dispatches);
        assert!(failure.cleanup_required);
    }

    for (case, expected) in [
        (
            GateCase::NoHeartbeat,
            NavigationReadinessCode::StackStateUnobserved,
        ),
        (
            GateCase::PlannerMissing,
            NavigationReadinessCode::PlannerInputUnobserved,
        ),
        (
            GateCase::SinkMissing,
            NavigationReadinessCode::MotionSinkUnobserved,
        ),
        (
            GateCase::SinkFalse,
            NavigationReadinessCode::MotionSinkNotReady,
        ),
    ] {
        let failure = run_case(case, stamp_ns)
            .await
            .expect_err("unready initial gate must fail closed at bounded deadline");
        assert_eq!(failure.reason, "initial navigation readiness timeout");
        assert!(
            failure.detail.blockers.contains(&expected),
            "{}",
            failure.render()
        );
        assert_eq!(
            failure.dispatches,
            InitialReadinessDispatchLedger::default()
        );
        assert!(failure.cleanup_required);
    }

    let reaped = Arc::new(Mutex::new(0_usize));
    let mut cleanup_spy = OwnedCleanupSpy::new(reaped.clone());
    let closed = run_case(GateCase::NavLedgerClosed, stamp_ns)
        .await
        .expect_err("closed readiness input must fail closed");
    assert_eq!(
        closed.reason,
        "V2 NavigationState ledger closed before initial readiness"
    );
    assert!(closed.render().contains("detail=NavigationReadinessDetail"));
    assert_eq!(closed.dispatches, InitialReadinessDispatchLedger::default());
    cleanup_spy.reap(&closed);
    assert_eq!(cleanup_spy.resource_count(), 0);
    assert_eq!(*reaped.lock().expect("cleanup spy lock poisoned"), 1);
    drop(cleanup_spy);

    let path_closed = run_case(GateCase::PathLedgerClosed, stamp_ns)
        .await
        .expect_err("closed Path ledger must fail closed");
    assert_eq!(
        path_closed.reason,
        "PathEnvelope ledger closed before initial readiness"
    );
    assert!(path_closed
        .render()
        .contains("detail=NavigationReadinessDetail"));
    assert_eq!(
        path_closed.dispatches,
        InitialReadinessDispatchLedger::default()
    );
    assert!(path_closed.cleanup_required);
}

#[tokio::test]
async fn initial_readiness_noncanonical_fixture_isolated() {
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    let handle = canary_readiness_handle(&nav);
    let stamp_ns = crate::explore_runtime::now_unix_nanos();
    let mut tf_history = PublicTfHistory::default();
    let mut tf_generation = 0;
    assert!(apply_canary_tf_payload(
        &nav,
        &mut tf_history,
        &mut tf_generation,
        &canary_tf_payload(stamp_ns),
    ));

    let (nav_tx, mut nav_rx) = tokio::sync::mpsc::channel(1);
    nav_tx
        .send(NavStateLedgerEntry {
            ordinal: 1,
            elapsed_ns: 1,
            state: schema::NavigationState {
                state: 1,
                message: "ACTIVE before readiness".to_string(),
                session_id: "unexpected".to_string(),
                ..schema::NavigationState::default()
            },
            feedback_applied: false,
        })
        .await
        .expect("queue isolated noncanonical state");
    let (path_tx, mut path_rx) = tokio::sync::mpsc::channel(1);
    let transport =
        LcmTransport::new_real(&test_lcm_url()).expect("isolated noncanonical LCM context");
    let goal_channel = format!("/v2/gate/noncanonical/{stamp_ns}");
    let goal_topic = LcmTopic::new(&goal_channel).expect("isolated noncanonical Goal topic");
    let goal_type =
        LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("isolated noncanonical Goal type");
    let mut goal_frames = transport.subscribe(goal_topic, Some(goal_type)).await;
    let mut evidence = NavStateEvidenceLedger::new();
    let mut guard_events = || String::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(1);
    let mut deadline_checkpoint = |_: InitialReadinessDeadlineCheckpoint| false;

    let reaped = Arc::new(Mutex::new(0_usize));
    let mut cleanup_spy = OwnedCleanupSpy::new(reaped.clone());
    let failure = tokio::time::timeout(
        Duration::from_millis(100),
        await_initial_navigation_readiness(
            &handle,
            &nav,
            &mut nav_rx,
            &mut evidence,
            &mut goal_frames,
            &mut path_rx,
            &mut guard_events,
            deadline,
            &mut deadline_checkpoint,
        ),
    )
    .await
    .expect("isolated noncanonical gate must return")
    .expect_err("noncanonical pre-readiness feedback must fail closed");

    assert!(
        failure
            .reason
            .contains("noncanonical NavigationState before initial readiness"),
        "{}",
        failure.render()
    );
    assert_eq!(
        failure.dispatches,
        InitialReadinessDispatchLedger::default()
    );
    assert_eq!(failure.release_count, 0);
    assert!(failure.cleanup_required);
    assert!(failure
        .render()
        .contains("detail=NavigationReadinessDetail"));
    assert_eq!(evidence.retained_count, 1);
    assert_eq!(evidence.runs.len(), 1);
    assert_eq!(
        evidence
            .runs
            .front()
            .expect("isolated UnexpectedState evidence run")
            .kind,
        NavEvidenceKind::Critical(NavCriticalEvidenceKind::UnexpectedState)
    );
    cleanup_spy.reap(&failure);
    assert_eq!(cleanup_spy.resource_count(), 0);
    assert_eq!(*reaped.lock().expect("cleanup spy lock poisoned"), 1);
    drop(cleanup_spy);
    drop(nav_tx);
    drop(path_tx);
}

#[tokio::test]
async fn canary_noncanonical_or_unready_idle_does_not_apply_feedback() {
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    let mut state = schema::NavigationState {
        stamp_sec: 1,
        message: "IDLE".to_string(),
        runtime_generation: 7,
        sink_ready: 1,
        planner_map_frame: "map".to_string(),
        planner_pose_frame: "map".to_string(),
        ..schema::NavigationState::default()
    };
    let payload =
        schema::encode_navigation_state_body(&state).expect("encode noncanonical heartbeat");
    assert!(!apply_canary_idle_feedback(&nav, &payload).await);
    state.message = "IDLE: guard client heartbeat".to_string();
    state.sink_ready = 0;
    let payload = schema::encode_navigation_state_body(&state).expect("encode unready heartbeat");
    assert!(!apply_canary_idle_feedback(&nav, &payload).await);
}

#[test]
fn terminal_state_remains_terminal_before_candidate_match() {
    let mut terminal = schema::NavigationState {
        state: 3,
        ..schema::NavigationState::default()
    };
    assert!(terminal_before_expected_path_acceptance(&terminal));
    terminal.state = 4;
    assert!(terminal_before_expected_path_acceptance(&terminal));
    terminal.state = 2;
    assert!(!terminal_before_expected_path_acceptance(&terminal));
}

#[test]
fn periodic_live_input_keeps_cadence_without_fabricating_receipts() {
    let mut shaper = LiveInputReceiptShaper::new();
    let started = tokio::time::Instant::now();
    shaper.record_publish(LiveInputKind::Pose, 1, 1000, started);
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    shaper.enable_periodic();
    assert!(shaper.can_publish(LiveInputKind::Pose));
    shaper.record_publish(LiveInputKind::Pose, 2, 1001, started + LIVE_POSE_INTERVAL);
    assert!(shaper.ledger[0].receipt_ordinal.is_none());
    let receipt = schema::NavigationState {
        pose_source_timestamp_ns: 1001,
        ..Default::default()
    };
    assert!(shaper.observe_receipt(&receipt, 7, started + LIVE_POSE_INTERVAL * 2).pose);
    assert_eq!(shaper.completed_count, 1);
    assert!(shaper.ledger[0].receipt_ordinal.is_none());
    assert_eq!(shaper.ledger[0].superseded_by, Some(1001));
    assert_eq!(shaper.ledger[1].receipt_ordinal, Some(7));
    assert!(!shaper.observe_receipt(&schema::NavigationState {
        pose_source_timestamp_ns: 9999,
        ..Default::default()
    }, 8, started + LIVE_POSE_INTERVAL * 3).pose);
    for sequence in 3..=100_u64 {
        let timestamp = 1000 + sequence;
        shaper.record_publish(LiveInputKind::Pose, sequence, timestamp, started + LIVE_POSE_INTERVAL * sequence as u32);
        shaper.observe_receipt(&schema::NavigationState {
            pose_source_timestamp_ns: timestamp as i64,
            ..Default::default()
        }, sequence, started + LIVE_POSE_INTERVAL * sequence as u32);
    }
    assert_eq!(shaper.ledger.len(), LIVE_INPUT_RECEIPT_LEDGER_CAPACITY);
}

#[test]
fn live_input_receipt_shaper_requires_exact_corresponding_wire_timestamp() {
    let mut shaper = LiveInputReceiptShaper::new();
    let published_at = tokio::time::Instant::now();
    shaper.record_publish(LiveInputKind::Pose, 10, 101, published_at);
    shaper.record_publish(LiveInputKind::Costmap, 11, 102, published_at);

    let mut unrelated = schema::NavigationState {
        stamp_sec: 9,
        stamp_nsec: 17,
        runtime_generation: 23,
        message: "watch duplicate".to_string(),
        ..schema::NavigationState::default()
    };
    let wake = shaper.observe_receipt(&unrelated, 1, published_at + Duration::from_millis(10));
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: false,
            costmap: false
        }
    );
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    assert!(!shaper.can_publish(LiveInputKind::Costmap));

    unrelated.pose_source_timestamp_ns = 102;
    unrelated.costmap_source_timestamp_ns = 103;
    let wake = shaper.observe_receipt(&unrelated, 2, published_at + Duration::from_millis(20));
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: false,
            costmap: false
        }
    );

    unrelated.pose_source_timestamp_ns = 101;
    unrelated.costmap_source_timestamp_ns = 0;
    let wake = shaper.observe_receipt(&unrelated, 3, published_at + Duration::from_millis(30));
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: true,
            costmap: false
        }
    );
    assert!(shaper.can_publish(LiveInputKind::Pose));
    assert!(!shaper.can_publish(LiveInputKind::Costmap));

    let duplicate = shaper.observe_receipt(&unrelated, 4, published_at + Duration::from_millis(40));
    assert!(!duplicate.pose);
    unrelated.pose_source_timestamp_ns = 0;
    unrelated.costmap_source_timestamp_ns = 102;
    let wake = shaper.observe_receipt(&unrelated, 5, published_at + Duration::from_millis(50));
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: false,
            costmap: true
        }
    );
    assert!(shaper.can_publish(LiveInputKind::Costmap));
    assert_eq!(shaper.completed_count, 2);
    assert_eq!(shaper.ledger[0].receipt_ordinal, Some(3));
    assert_eq!(shaper.ledger[1].receipt_ordinal, Some(5));
}

#[test]
fn live_input_receipt_shaper_keeps_one_outstanding_and_bounded_evidence() {
    let mut shaper = LiveInputReceiptShaper::new();
    let started_at = tokio::time::Instant::now();
    for index in 0..80_u64 {
        let timestamp_ns = 1_000 + index;
        let published_at = started_at + Duration::from_millis(index * 100);
        assert!(shaper.can_publish(LiveInputKind::Pose));
        shaper.record_publish(LiveInputKind::Pose, index, timestamp_ns, published_at);
        assert!(!shaper.can_publish(LiveInputKind::Pose));
        let state = schema::NavigationState {
            pose_source_timestamp_ns: i64::try_from(timestamp_ns).expect("test timestamp"),
            ..schema::NavigationState::default()
        };
        let wake =
            shaper.observe_receipt(&state, index + 1, published_at + Duration::from_millis(10));
        assert!(wake.pose);
        assert!(shaper.can_publish(LiveInputKind::Pose));
    }
    assert_eq!(shaper.completed_count, 80);
    assert_eq!(shaper.ledger.len(), LIVE_INPUT_RECEIPT_LEDGER_CAPACITY);
    assert!(shaper
        .ledger
        .iter()
        .all(|entry| entry.receipt_ordinal.is_some() && entry.receipt_at.is_some()));
}

#[test]
fn live_input_receipt_shaper_rejects_nonpositive_or_reused_timestamps() {
    let mut shaper = LiveInputReceiptShaper::new();
    let published_at = tokio::time::Instant::now();
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        shaper.record_publish(LiveInputKind::Pose, 1, 0, published_at);
    }))
    .is_err());
    shaper.record_publish(LiveInputKind::Pose, 2, 100, published_at);
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        shaper.record_publish(LiveInputKind::Costmap, 3, 100, published_at);
    }))
    .is_err());
    assert!(shaper.can_publish(LiveInputKind::Costmap));
}

#[test]
fn receipt_deadline_initial_publish_is_immediate() {
    let now = tokio::time::Instant::now();
    let shaper = LiveInputReceiptShaper::new();
    let schedule = LiveInputDeadlineSchedule::default();
    let (_tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);

    assert_eq!(
        plan_after_live_input_trigger(LiveInputTrigger::BaseTick, &mut rx, &schedule, &shaper, now,),
        LiveInputPublishPlan {
            pose: true,
            costmap: true,
        }
    );
}

#[test]
fn receipt_deadline_pre_gate_keeps_independent_kind_deadlines() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);
    schedule.record_publish(LiveInputKind::Costmap, started_at);
    shaper.record_publish(LiveInputKind::Costmap, 2, 102, started_at);
    let (_tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);

    let costmap_receipt = schema::NavigationState {
        costmap_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    let costmap_ack_at = started_at + Duration::from_millis(124);
    let wake = shaper.observe_receipt(&costmap_receipt, 1, costmap_ack_at);
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: false,
            costmap: true,
        }
    );
    assert_eq!(
        plan_after_live_input_trigger(
            LiveInputTrigger::Ack(Some(LiveInputKind::Costmap)),
            &mut rx,
            &schedule,
            &shaper,
            costmap_ack_at,
        ),
        LiveInputPublishPlan {
            pose: false,
            costmap: false,
        }
    );

    let pose_receipt = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        ..schema::NavigationState::default()
    };
    let pose_ack_at = started_at + Duration::from_millis(224);
    let wake = shaper.observe_receipt(&pose_receipt, 2, pose_ack_at);
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: true,
            costmap: false,
        }
    );
    assert_eq!(
        plan_after_live_input_trigger(
            LiveInputTrigger::Ack(Some(LiveInputKind::Pose)),
            &mut rx,
            &schedule,
            &shaper,
            pose_ack_at,
        ),
        LiveInputPublishPlan {
            pose: true,
            costmap: false,
        }
    );
    schedule.record_publish(LiveInputKind::Pose, pose_ack_at);
    shaper.record_publish(LiveInputKind::Pose, 3, 103, pose_ack_at);

    assert_eq!(
        plan_after_live_input_trigger(
            LiveInputTrigger::Deadline,
            &mut rx,
            &schedule,
            &shaper,
            started_at + LIVE_COSTMAP_INTERVAL,
        ),
        LiveInputPublishPlan {
            pose: false,
            costmap: true,
        }
    );
}

#[test]
fn receipt_deadline_phase_change_does_not_bypass_deadline() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);
    schedule.record_publish(LiveInputKind::Costmap, started_at);
    shaper.record_publish(LiveInputKind::Costmap, 2, 102, started_at);
    let receipt = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        costmap_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    let wake = shaper.observe_receipt(&receipt, 1, started_at + Duration::from_millis(20));
    assert!(wake.pose && wake.costmap);
    let (_tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);

    let mut phase = LiveInputPhase::Sentinel;
    assert_eq!(phase, LiveInputPhase::Sentinel);
    phase = LiveInputPhase::Fresh;
    assert_eq!(phase, LiveInputPhase::Fresh);
    assert_eq!(
        plan_after_live_input_trigger(
            LiveInputTrigger::BaseTick,
            &mut rx,
            &schedule,
            &shaper,
            started_at + LIVE_POSE_INTERVAL - Duration::from_millis(1),
        ),
        LiveInputPublishPlan {
            pose: false,
            costmap: false,
        }
    );
    assert_eq!(
        plan_after_live_input_trigger(
            LiveInputTrigger::Deadline,
            &mut rx,
            &schedule,
            &shaper,
            started_at + LIVE_POSE_INTERVAL,
        ),
        LiveInputPublishPlan {
            pose: true,
            costmap: false,
        }
    );
}

#[test]
fn receipt_deadline_schedules_early_ack_at_exact_deadline() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);

    let state = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        ..schema::NavigationState::default()
    };
    let receipt_at = started_at + Duration::from_millis(30);
    assert!(shaper.observe_receipt(&state, 1, receipt_at).pose);
    assert_eq!(
        schedule.next_ready_deadline(&shaper),
        Some(started_at + LIVE_POSE_INTERVAL)
    );
    assert!(!schedule.is_due(
        LiveInputKind::Pose,
        started_at + LIVE_POSE_INTERVAL - Duration::from_nanos(1)
    ));
    let next_published_at = started_at + LIVE_POSE_INTERVAL;
    assert!(schedule.is_due(LiveInputKind::Pose, next_published_at));
    schedule.record_publish(LiveInputKind::Pose, next_published_at);
    shaper.record_publish(LiveInputKind::Pose, 2, 102, next_published_at);
    assert_eq!(shaper.ledger[0].next_publication.unwrap().sequence, 2);
    assert_eq!(
        shaper.ledger[0]
            .next_publication
            .unwrap()
            .published_at
            .duration_since(shaper.ledger[0].receipt_at.unwrap()),
        Duration::from_millis(70)
    );

    let next_state = schema::NavigationState {
        pose_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    let next_receipt_at = next_published_at + Duration::from_millis(10);
    assert!(shaper.observe_receipt(&next_state, 2, next_receipt_at).pose);
    assert_eq!(
        shaper.ledger[1]
            .receipt_at
            .unwrap()
            .duration_since(shaper.ledger[1].publication.published_at),
        Duration::from_millis(10)
    );
}

#[test]
fn receipt_deadline_late_ack_is_immediately_due_once() {
    let started_at = tokio::time::Instant::now();
    let receipt_at = started_at + LIVE_POSE_INTERVAL + Duration::from_millis(50);
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);

    let state = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        ..schema::NavigationState::default()
    };
    let wake = shaper.observe_receipt(&state, 1, receipt_at);
    let (tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);
    send_live_input_ack_wakes(&tx, wake);
    assert_eq!(rx.try_recv().unwrap(), LiveInputKind::Pose);
    assert!(matches!(
        rx.try_recv(),
        Err(tokio::sync::mpsc::error::TryRecvError::Empty)
    ));
    assert!(schedule.is_due(LiveInputKind::Pose, receipt_at));

    schedule.record_publish(LiveInputKind::Pose, receipt_at);
    shaper.record_publish(LiveInputKind::Pose, 2, 102, receipt_at);
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    assert_eq!(
        shaper.ledger[0]
            .next_publication
            .unwrap()
            .published_at
            .duration_since(receipt_at),
        Duration::ZERO
    );
}

#[test]
fn receipt_deadline_keeps_costmap_independent_from_pose() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);
    schedule.record_publish(LiveInputKind::Costmap, started_at);
    shaper.record_publish(LiveInputKind::Costmap, 2, 102, started_at);

    let state = schema::NavigationState {
        costmap_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    let receipt_at = started_at + Duration::from_millis(40);
    let wake = shaper.observe_receipt(&state, 1, receipt_at);
    assert_eq!(
        wake,
        LiveInputAckWake {
            pose: false,
            costmap: true
        }
    );
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    assert!(shaper.can_publish(LiveInputKind::Costmap));
    assert_eq!(
        schedule.next_ready_deadline(&shaper),
        Some(started_at + LIVE_COSTMAP_INTERVAL)
    );

    let costmap_deadline = started_at + LIVE_COSTMAP_INTERVAL;
    assert!(!schedule.is_due(
        LiveInputKind::Costmap,
        costmap_deadline - Duration::from_nanos(1)
    ));
    schedule.record_publish(LiveInputKind::Costmap, costmap_deadline);
    shaper.record_publish(LiveInputKind::Costmap, 3, 103, costmap_deadline);
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    assert!(!shaper.can_publish(LiveInputKind::Costmap));
}

#[test]
fn receipt_wake_ignores_invalid_and_duplicate_ack() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);
    let (tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);

    let invalid = schema::NavigationState {
        pose_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    send_live_input_ack_wakes(
        &tx,
        shaper.observe_receipt(&invalid, 1, started_at + Duration::from_millis(10)),
    );
    assert!(matches!(
        rx.try_recv(),
        Err(tokio::sync::mpsc::error::TryRecvError::Empty)
    ));

    let exact = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        ..schema::NavigationState::default()
    };
    send_live_input_ack_wakes(
        &tx,
        shaper.observe_receipt(&exact, 2, started_at + Duration::from_millis(20)),
    );
    assert_eq!(rx.try_recv().unwrap(), LiveInputKind::Pose);
    send_live_input_ack_wakes(
        &tx,
        shaper.observe_receipt(&exact, 3, started_at + Duration::from_millis(30)),
    );
    assert!(matches!(
        rx.try_recv(),
        Err(tokio::sync::mpsc::error::TryRecvError::Empty)
    ));
}

#[test]
fn receipt_deadline_publishes_simultaneously_ready_kinds_once_each() {
    let ready_at = tokio::time::Instant::now();
    let started_at = ready_at - LIVE_COSTMAP_INTERVAL;
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    for (kind, sequence, timestamp_ns) in [
        (LiveInputKind::Pose, 1, 101),
        (LiveInputKind::Costmap, 2, 102),
    ] {
        schedule.record_publish(kind, started_at);
        shaper.record_publish(kind, sequence, timestamp_ns, started_at);
    }
    let state = schema::NavigationState {
        pose_source_timestamp_ns: 101,
        costmap_source_timestamp_ns: 102,
        ..schema::NavigationState::default()
    };
    let wake = shaper.observe_receipt(&state, 1, ready_at);
    let (tx, mut rx) = tokio::sync::mpsc::channel(LIVE_INPUT_ACK_WAKE_CAPACITY);
    send_live_input_ack_wakes(&tx, wake);
    let plan = plan_after_live_input_trigger(
        LiveInputTrigger::Deadline,
        &mut rx,
        &schedule,
        &shaper,
        ready_at,
    );
    assert!(matches!(
        rx.try_recv(),
        Err(tokio::sync::mpsc::error::TryRecvError::Empty)
    ));
    assert_eq!(
        plan,
        LiveInputPublishPlan {
            pose: true,
            costmap: true
        }
    );

    schedule.record_publish(LiveInputKind::Pose, ready_at);
    shaper.record_publish(LiveInputKind::Pose, 3, 103, ready_at);
    schedule.record_publish(LiveInputKind::Costmap, ready_at);
    shaper.record_publish(LiveInputKind::Costmap, 4, 104, ready_at);
    assert!(!shaper.can_publish(LiveInputKind::Pose));
    assert!(!shaper.can_publish(LiveInputKind::Costmap));
}

#[test]
fn live_input_record_before_publish_reserves_outstanding_identity() {
    let mut shaper = LiveInputReceiptShaper::new();
    let published_at = tokio::time::Instant::now();
    shaper.record_publish(LiveInputKind::Pose, 1, 101, published_at);
    assert_eq!(shaper.ledger.len(), 1);
    assert_eq!(shaper.pose_outstanding.unwrap().sequence, 1);
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        shaper.record_publish(
            LiveInputKind::Pose,
            2,
            102,
            published_at + LIVE_POSE_INTERVAL,
        );
    }))
    .is_err());
}

#[test]
fn receipt_trigger_state_machine_coalesces_across_timer_generations() {
    let started_at = tokio::time::Instant::now();
    let mut shaper = LiveInputReceiptShaper::new();
    let mut schedule = LiveInputDeadlineSchedule::default();
    schedule.record_publish(LiveInputKind::Pose, started_at);
    shaper.record_publish(LiveInputKind::Pose, 1, 101, started_at);
    schedule.record_publish(LiveInputKind::Costmap, started_at);
    shaper.record_publish(LiveInputKind::Costmap, 2, 102, started_at);
    let (tx, mut rx) = tokio::sync::mpsc::channel(1);
    let mut sequence = 3_u64;
    let mut timestamp_ns = 102_u64;

    for generation in 0..4_u64 {
        let now = started_at + Duration::from_millis((generation + 1) * 250);
        let state = schema::NavigationState {
            pose_source_timestamp_ns: i64::try_from(shaper.pose_outstanding.unwrap().timestamp_ns)
                .expect("pose test timestamp"),
            costmap_source_timestamp_ns: i64::try_from(
                shaper.costmap_outstanding.unwrap().timestamp_ns,
            )
            .expect("costmap test timestamp"),
            ..schema::NavigationState::default()
        };
        let wake = shaper.observe_receipt(&state, generation + 1, now);
        assert!(wake.pose && wake.costmap);
        send_live_input_ack_wakes(&tx, wake);
        let trigger = if generation % 2 == 0 {
            LiveInputTrigger::BaseTick
        } else {
            LiveInputTrigger::Deadline
        };
        let plan = plan_after_live_input_trigger(trigger, &mut rx, &schedule, &shaper, now);
        assert!(matches!(
            rx.try_recv(),
            Err(tokio::sync::mpsc::error::TryRecvError::Empty)
        ));
        assert_eq!(
            plan,
            LiveInputPublishPlan {
                pose: true,
                costmap: true
            }
        );

        for kind in [LiveInputKind::Pose, LiveInputKind::Costmap] {
            timestamp_ns = timestamp_ns
                .checked_add(1)
                .expect("test timestamp overflow");
            schedule.record_publish(kind, now);
            shaper.record_publish(kind, sequence, timestamp_ns, now);
            sequence = sequence.checked_add(1).expect("test sequence overflow");
        }
        assert!(!shaper.can_publish(LiveInputKind::Pose));
        assert!(!shaper.can_publish(LiveInputKind::Costmap));
    }

    assert!(shaper
        .ledger
        .iter()
        .map(|entry| entry.publication.timestamp_ns)
        .collect::<Vec<_>>()
        .windows(2)
        .all(|pair| pair[0] < pair[1]));

    let (_open_tx, mut open_rx) = tokio::sync::mpsc::channel(1);
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        plan_after_live_input_trigger(
            LiveInputTrigger::Ack(None),
            &mut open_rx,
            &schedule,
            &shaper,
            started_at,
        );
    }))
    .is_err());

    drop(rx);
    send_live_input_ack_wakes(
        &tx,
        LiveInputAckWake {
            pose: true,
            costmap: true,
        },
    );
}

#[test]
fn live_input_cadence_summary_is_bounded() {
    let mut summary = LiveInputCadenceSummary::default();
    summary.record_pose(100);
    summary.record_pose(200);
    summary.record_pose(350);
    summary.record_costmap(100);
    summary.record_costmap(350);
    assert_eq!(summary.pose_publish_count, 3);
    assert_eq!(summary.costmap_publish_count, 2);
    assert_eq!(summary.pose_max_interval_ns, 150);
    assert_eq!(summary.costmap_max_interval_ns, 250);
}

#[tokio::test]
async fn cleanup_recognizes_real_pre_start_explore_as_settled() {
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let snapshot = explore.snapshot();
    let blocks_mapping_teardown = explore.blocks_mapping_mutation().await;
    assert!(explore_pre_start_allows_sink_clear(
        &snapshot,
        blocks_mapping_teardown
    ));

    let rejected = explore.stop("pre-start-parent").await;
    assert_eq!(rejected.status, crate::explore_runtime::STATUS_REJECTED);
    assert!(!explore_stop_allows_sink_clear(&rejected, false));

    let mut retained = snapshot;
    retained.status = crate::explore_runtime::STATUS_ACTIVE;
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.status = crate::explore_runtime::STATUS_IDLE;

    retained.request_id = "unexpected-owner".to_string();
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.request_id.clear();

    retained.exploration_generation = 1;
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.exploration_generation = 0;

    retained.child_task_id = "retained-child".to_string();
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.child_task_id.clear();

    retained.child_lease = Some(crate::explore_runtime::ChildNavigationLease {
        session_id: "retained-session".to_string(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: "retained-parent".to_string(),
        goal_generation: 1,
        producer_runtime_generation: 1,

    });
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.child_lease = None;

    retained.global_stop_reconciliation.eligible = true;
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.global_stop_reconciliation.eligible = false;

    retained.ownership_reconciliation_receipt = Some(
        crate::explore_runtime::ExplorationOwnershipReconciliationReceipt {
            child_lease: None,
            cause: "test-residual".to_string(),
            recovery_correlation: "test-correlation".to_string(),
            parent_request_id: "retained-parent".to_string(),
            exploration_generation: 1,
            child_lease_fingerprint: "test-child-lease".to_string(),
            cancel_generation: 1,
            commit_disposition: "test-committed".to_string(),
        },
    );
    assert!(!explore_pre_start_allows_sink_clear(&retained, false));
    retained.ownership_reconciliation_receipt = None;

    assert!(!explore_pre_start_allows_sink_clear(&retained, true));
}

#[test]
fn cleanup_requires_exact_explore_terminal_before_sink_clear() {
    let mut stopped = crate::explore_runtime::ExploreRuntimeSnapshot {
        status: crate::explore_runtime::STATUS_CANCELLED,
        ..Default::default()
    };
    assert!(explore_stop_allows_sink_clear(&stopped, false));

    stopped.status = crate::explore_runtime::STATUS_STOPPING;
    assert!(!explore_stop_allows_sink_clear(&stopped, false));
    stopped.status = crate::explore_runtime::STATUS_FAILED;
    assert!(!explore_stop_allows_sink_clear(&stopped, false));
    stopped.status = crate::explore_runtime::STATUS_CANCELLED;
    stopped.child_task_id = "retained-child".to_string();
    assert!(!explore_stop_allows_sink_clear(&stopped, false));
    stopped.child_task_id.clear();
    stopped.child_lease = Some(crate::explore_runtime::ChildNavigationLease {
        session_id: "retained-session".to_string(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: "retained-parent".to_string(),
        goal_generation: 1,
        producer_runtime_generation: 1,

    });
    assert!(!explore_stop_allows_sink_clear(&stopped, false));
    stopped.child_lease = None;
    assert!(!explore_stop_allows_sink_clear(&stopped, true));
}

#[test]
fn finish_v2_canary_preserves_failure_priority() {
    let body_panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        finish_v2_canary(
            Err(Box::new(String::from("body panic"))),
            V2CanaryCleanupReport {
                feeder_panic: Some(Box::new(String::from("feeder panic"))),
                errors: vec!["cleanup error".to_string()],
            },
        );
    }))
    .expect_err("body panic must be restored");
    assert_eq!(panic_payload_message(body_panic.as_ref()), "body panic");

    let body_reason = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        finish_v2_canary(
            Ok(Err("body reason".to_string())),
            V2CanaryCleanupReport {
                feeder_panic: Some(Box::new(String::from("feeder panic"))),
                errors: vec!["cleanup error".to_string()],
            },
        );
    }))
    .expect_err("body reason must outrank cleanup failures");
    assert_eq!(
        panic_payload_message(body_reason.as_ref()),
        "V2 canary body failed after cleanup: body reason"
    );

    let feeder_panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        finish_v2_canary(
            Ok(Ok(())),
            V2CanaryCleanupReport {
                feeder_panic: Some(Box::new(String::from("feeder panic"))),
                errors: vec!["cleanup error".to_string()],
            },
        );
    }))
    .expect_err("feeder panic must outrank cleanup errors after body success");
    assert_eq!(panic_payload_message(feeder_panic.as_ref()), "feeder panic");

    let cleanup_error = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        finish_v2_canary(
            Ok(Ok(())),
            V2CanaryCleanupReport {
                feeder_panic: None,
                errors: vec!["cleanup error".to_string()],
            },
        );
    }))
    .expect_err("cleanup errors must fail an otherwise successful canary");
    assert!(panic_payload_message(cleanup_error.as_ref())
        .contains("V2 canary cleanup failed after successful body"));

    finish_v2_canary(Ok(Ok(())), V2CanaryCleanupReport::default());
}

#[tokio::test]
async fn vertical_v2_rejects_wrong_frame_and_out_of_boundary_goal() {
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let wrong_frame = crate::explore_runtime::ExploreCostmapSnapshot {
        grid: OccupancyGrid2D::filled(Point2 { x: 0.0, y: 0.0 }, 0.5, 2, 2, OccupancyCell::Free),
        frame_id: "odom".to_string(),
    };
    assert!(explore.ingest_costmap_snapshot(wrong_frame).await.is_err());
    assert!(!explore.cached_readiness().costmap_input_ready);

    let sink = Arc::new(FailClosedNavSink::default());
    explore.set_navigation_sink(sink.clone());

    let mut grid = OccupancyGrid2D::filled(
        Point2 { x: 0.0, y: 0.0 },
        0.5,
        20,
        20,
        OccupancyCell::Unknown,
    );
    for y in 0..20 {
        for x in 0..10 {
            grid.set(x, y, OccupancyCell::Free);
        }
    }
    explore
        .ingest_costmap_snapshot(crate::explore_runtime::ExploreCostmapSnapshot {
            grid,
            frame_id: "map".to_string(),
        })
        .await
        .expect("map-frame costmap");
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 0.25, y: 0.0 },
        Point2 { x: 0.25, y: 0.25 },
        Point2 { x: 0.0, y: 0.25 },
    ])
    .expect("small valid boundary");
    let accepted = explore
        .start_with_config(
            "v2-boundary-reject",
            crate::explore_runtime::ExploreStartConfig {
                motion_admission: crate::nav_runtime::test_motion_admission(),
                boundary: Some(boundary),
                ..crate::explore_runtime::ExploreStartConfig::default()
            },
        )
        .await;
    assert_eq!(accepted.status, "ACCEPTED");

    let pending = explore.tick((0.0, 0.0)).await;
    assert_eq!(pending.status, "PENDING", "{}", pending.message);
    assert!(pending.child_task_id.is_empty());
    assert!(pending.child_lease.is_none());
    assert!(sink.submissions.lock().expect("submissions").is_empty());
    let diagnostics = explore.diagnostics().await;
    assert!(diagnostics
        .sources
        .iter()
        .any(|source| { source.last_error.contains("outside exploration boundary") }));

    let stopped = explore.stop("v2-boundary-reject").await;
    assert_eq!(stopped.status, "CANCELLED");
    explore.clear_navigation_sink();
}
