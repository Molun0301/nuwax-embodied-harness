//! Exploration controller, child-navigation handoff, and background scheduling.
//!
//! `ExploreRuntimeHandle` owns the production wavefront session. State reads are
//! pure snapshots; controller progress happens only through `tick*` or a started
//! [`ExploreSchedulerHandle`]. Mapping and navigation wiring are injected through
//! typed interfaces so the module never silently discards a selected goal.

#![allow(dead_code)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
use nuwax_exploration::wavefront_adapter::WavefrontGoalSourceAdapter;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationBoundary, ExplorationController,
    ExplorationGoalFeedback, ExplorationGoalSource, ExplorationPoll, ExplorationStateSnapshot,
    ExplorationStopReason, ExplorationTerminalKind,
};
use nuwax_explore_wavefront::WavefrontConfig;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{
    ClockKind, Point3, Pose3, PoseStamped, Quaternion, Timestamp,
};
use nuwax_nav_types_messages::goal::NavigationGoal;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_nav_types_messages::visual::FrontierGoal;
use parking_lot::{Mutex as SyncMutex, RwLock};
use tokio::sync::{watch, Mutex};
use tokio::task::JoinHandle;

use crate::nav_runtime::MotionAdmissionProof;

pub const STATUS_IDLE: &str = "IDLE";
pub const STATUS_ACCEPTED: &str = "ACCEPTED";
pub const STATUS_REJECTED: &str = "REJECTED";
pub const STATUS_PENDING: &str = "PENDING";
pub const STATUS_STOPPING: &str = "STOPPING";
pub const STATUS_STOPPED: &str = "STOPPED";
pub const STATUS_ACTIVE: &str = "ACTIVE";
pub const STATUS_SUCCEEDED: &str = "SUCCEEDED";
pub const STATUS_CANCELLED: &str = "CANCELLED";
pub const STATUS_FAILED: &str = "FAILED";
pub const STATUS_UNIMPLEMENTED: &str = "UNIMPLEMENTED";

const POLICY_PREFER_WAVEFRONT: &str = "prefer_wavefront";
const POLICY_PREFER_TARE: &str = "prefer_tare";
const POLICY_HIGHEST_SCORE: &str = "highest_score";
const POLICY_FIRST_AVAILABLE: &str = "first_available";
const DEFAULT_SCHEDULER_PERIOD: Duration = Duration::from_millis(200);
const DEFAULT_POSE_RECEIPT_TTL: Duration = Duration::from_secs(1);
const DEFAULT_POSE_UNAVAILABLE_GRACE: Duration = Duration::from_secs(2);

pub fn parse_arbitration_policy(s: &str) -> ExplorationArbitrationPolicy {
    try_parse_arbitration_policy(s).unwrap_or(ExplorationArbitrationPolicy::PreferWavefront)
}

fn try_parse_arbitration_policy(s: &str) -> Result<ExplorationArbitrationPolicy, String> {
    match s.trim().to_ascii_lowercase().as_str() {
        "" | POLICY_PREFER_WAVEFRONT => Ok(ExplorationArbitrationPolicy::PreferWavefront),
        POLICY_PREFER_TARE => Ok(ExplorationArbitrationPolicy::PreferTare),
        POLICY_HIGHEST_SCORE => Ok(ExplorationArbitrationPolicy::HighestScore),
        POLICY_FIRST_AVAILABLE => Ok(ExplorationArbitrationPolicy::FirstAvailable),
        other => Err(format!("unsupported exploration arbitration: {other}")),
    }
}

pub fn arbitration_policy_str(p: ExplorationArbitrationPolicy) -> &'static str {
    match p {
        ExplorationArbitrationPolicy::PreferWavefront => POLICY_PREFER_WAVEFRONT,
        ExplorationArbitrationPolicy::PreferTare => POLICY_PREFER_TARE,
        ExplorationArbitrationPolicy::HighestScore => POLICY_HIGHEST_SCORE,
        ExplorationArbitrationPolicy::FirstAvailable => POLICY_FIRST_AVAILABLE,
    }
}

pub fn stop_reason_str(r: ExplorationStopReason) -> &'static str {
    match r {
        ExplorationStopReason::NoInformationGain => "no_information_gain",
        ExplorationStopReason::NoReachableFreeSpace => "no_reachable_free_space",
        ExplorationStopReason::ConsecutiveFailuresExceeded => "consecutive_failures_exceeded",
        ExplorationStopReason::UserCancelled => "user_cancelled",
        ExplorationStopReason::CoverageReached => "coverage_reached",
        ExplorationStopReason::NoFrontierTimeout => "no_frontier_timeout",
        ExplorationStopReason::DurationExceeded => "duration_exceeded",
        ExplorationStopReason::MaxGoalsReached => "max_goals_reached",
        ExplorationStopReason::MapRevisionChanged => "map_revision_changed",
    }
}

fn status_for_stop_reason(reason: ExplorationStopReason) -> &'static str {
    match reason.terminal_kind() {
        ExplorationTerminalKind::Succeeded => STATUS_SUCCEEDED,
        ExplorationTerminalKind::Cancelled => STATUS_CANCELLED,
        ExplorationTerminalKind::Failed => STATUS_FAILED,
    }
}

pub fn now_unix_nanos() -> u64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|duration| duration.as_nanos() as u64)
        .unwrap_or(0)
}

pub fn now_monotonic_nanos() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START
        .get_or_init(Instant::now)
        .elapsed()
        .as_nanos()
        .min(u64::MAX as u128) as u64
        + 1
}

pub fn pose_stamped_from_robot(x: f64, y: f64, _frame_id: &str) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp {
            nanos: now_unix_nanos() as i64,
            clock: ClockKind::Unix,
        },
        pose: Pose3 {
            position: Point3 { x, y, z: 0.0 },
            orientation: Quaternion::IDENTITY,
        },
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExploreStopConditions {
    pub max_coverage_pct: Option<f64>,
    pub no_frontier_timeout_ms: Option<u64>,
    pub max_duration_ms: Option<u64>,
    pub max_goals: Option<u32>,
    pub stop_on_map_revision_change: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExploreStartConfig {
    pub strategy: String,
    pub arbitration: String,
    pub map_version_id: String,
    pub map_revision: u64,
    pub motion_admission: MotionAdmissionProof,
    pub boundary: Option<ExplorationBoundary>,
    pub stop_conditions: ExploreStopConditions,
    pub max_navigation_retries: u32,
    pub blacklist_radius_m: f64,
}

impl Default for ExploreStartConfig {
    fn default() -> Self {
        Self {
            strategy: "wavefront".to_string(),
            arbitration: POLICY_PREFER_WAVEFRONT.to_string(),
            map_version_id: String::new(),
            map_revision: 0,
            motion_admission: MotionAdmissionProof::default(),
            boundary: None,
            stop_conditions: ExploreStopConditions::default(),
            max_navigation_retries: 1,
            blacklist_radius_m: WavefrontConfig::default().safe_distance_m,
        }
    }
}

impl ExploreStartConfig {
    fn validate(&self) -> Result<ExplorationArbitrationPolicy, String> {
        match self.strategy.trim().to_ascii_lowercase().as_str() {
            "" | "wavefront" | "frontier" => {}
            "tare" | "native" | "exploration-native" => {
                return Err(format!(
                    "exploration source `{}` is unsupported/source unavailable in the qualified production profile",
                    self.strategy
                ));
            }
            other => return Err(format!("unknown exploration strategy: {other}")),
        }
        let policy = try_parse_arbitration_policy(&self.arbitration)?;
        if policy == ExplorationArbitrationPolicy::PreferTare {
            return Err(
                "prefer_tare is unsupported/source unavailable in the qualified production profile"
                    .to_string(),
            );
        }
        if !self.blacklist_radius_m.is_finite() || self.blacklist_radius_m < 0.0 {
            return Err("blacklist_radius_m must be finite and non-negative".to_string());
        }
        self.motion_admission
            .validate()
            .map_err(|reason| format!("motion admission proof invalid: {reason}"))?;
        if self
            .stop_conditions
            .max_coverage_pct
            .is_some_and(|coverage| !coverage.is_finite() || !(0.0..=100.0).contains(&coverage))
        {
            return Err("max_coverage_pct must be in [0, 100]".to_string());
        }
        Ok(policy)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExploreCostmapSnapshot {
    pub grid: OccupancyGrid2D,
    pub frame_id: String,
    pub map_version_id: String,
    pub map_revision: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExploreRobotPose {
    pub x: f64,
    pub y: f64,
    pub frame_id: String,
    pub source_timestamp_ns: u64,
    pub received_monotonic_ns: u64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrontierSnapshot {
    pub frontiers: Vec<FrontierGoal>,
    pub revision: u64,
    pub map_revision: u64,
}

#[derive(Debug, Clone)]
pub struct ChildNavigationRequest {
    pub parent_request_id: String,
    pub request_id: String,
    pub map_version_id: String,
    pub map_revision: u64,
    pub motion_admission: MotionAdmissionProof,
    pub frontier_goal_id: String,
    pub goal: NavigationGoal,
    pub boundary: Option<ExplorationBoundary>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationLease {
    pub session_id: String,
    pub owner_kind: i32,
    pub parent_request_id: String,
    pub goal_generation: u64,
    pub map_version_id: String,
    pub map_revision: u64,
    pub goal_payload_sha256: [u8; 32],
}

impl ChildNavigationLease {
    fn matches(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
            && self.goal_payload_sha256 == other.goal_payload_sha256
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationTicket {
    pub lease: ChildNavigationLease,
}

/// Admission failure returned before a child navigation lease becomes active.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChildNavigationSubmitError {
    /// Navigation is temporarily unable to accept the goal; retry without feedback.
    RetryableReadiness(String),
    /// The request is permanently rejected and must enter goal-failure policy.
    Rejected(String),
}

impl ChildNavigationSubmitError {
    fn reason(&self) -> &str {
        match self {
            Self::RetryableReadiness(reason) | Self::Rejected(reason) => reason,
        }
    }

    fn is_retryable(&self) -> bool {
        matches!(self, Self::RetryableReadiness(_))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationCancelTicket {
    pub lease: ChildNavigationLease,
    pub cancel_generation: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationTerminalProof {
    pub lease: ChildNavigationLease,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationZeroProof {
    pub lease: ChildNavigationLease,
    pub cancel_generation: u64,
    pub zero_ack_id: String,
    pub zero_applied_monotonic_ns: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChildNavigationResult {
    Reached(ChildNavigationTerminalProof),
    Failed {
        proof: ChildNavigationTerminalProof,
        reason: String,
    },
    Cancelled(ChildNavigationZeroProof),
    CancelFailed {
        proof: ChildNavigationTerminalProof,
        cancel_generation: u64,
        reason: String,
    },
}

const MAX_EARLY_CANCEL_TERMINALS: usize = 4;

fn cancellation_result_generation(result: &ChildNavigationResult) -> Option<u64> {
    match result {
        ChildNavigationResult::Cancelled(proof) => Some(proof.cancel_generation),
        ChildNavigationResult::CancelFailed {
            cancel_generation, ..
        } => Some(*cancel_generation),
        ChildNavigationResult::Reached(_) | ChildNavigationResult::Failed { .. } => None,
    }
}

fn retain_early_cancel_terminal(
    pending: &mut PendingChildCancellation,
    result: ChildNavigationResult,
) {
    if pending.early_terminals.contains(&result) {
        return;
    }
    if pending.early_terminals.len() >= MAX_EARLY_CANCEL_TERMINALS {
        pending.early_terminals.remove(0);
    }
    pending.early_terminals.push(result);
}

fn take_early_cancel_terminal(
    pending: &mut PendingChildCancellation,
    generation: u64,
) -> Option<ChildNavigationResult> {
    let position = pending
        .early_terminals
        .iter()
        .position(|result| {
            matches!(result, ChildNavigationResult::Cancelled(_))
                && cancellation_result_generation(result) == Some(generation)
        })
        .or_else(|| {
            pending.early_terminals.iter().position(|result| {
                matches!(result, ChildNavigationResult::CancelFailed { .. })
                    && cancellation_result_generation(result) == Some(generation)
            })
        })?;
    Some(pending.early_terminals.remove(position))
}

#[async_trait]
pub trait ExploreNavigationSink: Send + Sync {
    async fn submit(
        &self,
        request: ChildNavigationRequest,
    ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError>;

    async fn cancel(
        &self,
        lease: &ChildNavigationLease,
    ) -> Result<ChildNavigationCancelTicket, String>;
}

#[async_trait]
pub trait ExplorePoseProvider: Send + Sync {
    async fn current_pose(&self) -> Result<ExploreRobotPose, String>;
}

#[derive(Debug, Clone)]
pub struct ExploreRuntimeSnapshot {
    pub status: &'static str,
    pub message: String,
    pub request_id: String,
    pub selected_source: String,
    pub last_stop_reason: String,
    pub frontier_count: u32,
    pub revision: u64,
    pub child_task_id: String,
    pub map_version_id: String,
    pub map_revision: u64,
}

impl Default for ExploreRuntimeSnapshot {
    fn default() -> Self {
        Self {
            status: STATUS_IDLE,
            message: "exploration idle".to_string(),
            request_id: String::new(),
            selected_source: String::new(),
            last_stop_reason: String::new(),
            frontier_count: 0,
            revision: 0,
            child_task_id: String::new(),
            map_version_id: String::new(),
            map_revision: 0,
        }
    }
}

#[derive(Debug, Clone)]
struct ActiveChildNavigation {
    request_id: String,
    frontier_goal_id: String,
    lease: ChildNavigationLease,
}

#[derive(Debug, Clone)]
struct PendingChildAction {
    reason: ExplorationStopReason,
    message: String,
}

#[derive(Debug, Clone)]
struct PendingChildCancellation {
    action: PendingChildAction,
    ticket: Option<ChildNavigationCancelTicket>,
    failure: Option<String>,
    dispatch_in_flight: bool,
    early_terminals: Vec<ChildNavigationResult>,
}

pub struct ExploreRuntimeInner {
    controller: ExplorationController,
    arbitration: ExplorationArbitrationPolicy,
    last_request_id: String,
    last_selected_source: String,
    last_stop_reason: String,
    last_stop: Option<ExplorationStopReason>,
    last_frontier_goal: Option<FrontierGoal>,
    last_poll: Option<ExplorationPoll>,
    costmap_input: Option<ExploreCostmapSnapshot>,
    active_config: Option<ExploreStartConfig>,
    active_child: Option<ActiveChildNavigation>,
    pending_child_dispatch: Option<ChildNavigationRequest>,
    pending_child_cancellation: Option<PendingChildCancellation>,
    generation: u64,
    child_sequence: u64,
    reached_goals: u32,
    started_at_ns: u64,
    no_frontier_since_ns: Option<u64>,
    frontiers: Vec<FrontierGoal>,
    frontier_revision: u64,
}

impl ExploreRuntimeInner {
    fn new() -> Self {
        Self {
            controller: build_controller(
                ExplorationArbitrationPolicy::PreferWavefront,
                &ExploreStartConfig::default(),
            ),
            arbitration: ExplorationArbitrationPolicy::PreferWavefront,
            last_request_id: String::new(),
            last_selected_source: String::new(),
            last_stop_reason: String::new(),
            last_stop: None,
            last_frontier_goal: None,
            last_poll: None,
            costmap_input: None,
            active_config: None,
            active_child: None,
            pending_child_dispatch: None,
            pending_child_cancellation: None,
            generation: 0,
            child_sequence: 0,
            reached_goals: 0,
            started_at_ns: 0,
            no_frontier_since_ns: None,
            frontiers: Vec::new(),
            frontier_revision: 0,
        }
    }

    pub fn add_source<S: ExplorationGoalSource + 'static>(&mut self, source: S) {
        self.controller.add_source(source);
    }

    fn map_revision(&self) -> u64 {
        self.costmap_input
            .as_ref()
            .map(|snapshot| snapshot.map_revision)
            .unwrap_or(0)
    }

    fn session_in_progress(&self) -> bool {
        self.controller.is_active() || self.pending_child_cancellation.is_some()
    }

    fn map_version_id(&self) -> String {
        self.active_config
            .as_ref()
            .map(|config| config.map_version_id.clone())
            .filter(|value| !value.is_empty())
            .or_else(|| {
                self.costmap_input
                    .as_ref()
                    .map(|snapshot| snapshot.map_version_id.clone())
            })
            .unwrap_or_default()
    }

    fn update_frontiers(&mut self, frontiers: Vec<FrontierGoal>) -> Option<FrontierSnapshot> {
        let map_revision = self.map_revision();
        if self.frontiers == frontiers {
            return None;
        }
        self.frontiers = frontiers;
        self.frontier_revision = self.frontier_revision.saturating_add(1);
        Some(FrontierSnapshot {
            frontiers: self.frontiers.clone(),
            revision: self.frontier_revision,
            map_revision,
        })
    }

    fn snapshot(&self, status: &'static str, message: impl Into<String>) -> ExploreRuntimeSnapshot {
        ExploreRuntimeSnapshot {
            status,
            message: message.into(),
            request_id: self.last_request_id.clone(),
            selected_source: self.last_selected_source.clone(),
            last_stop_reason: self.last_stop_reason.clone(),
            frontier_count: self.frontiers.len() as u32,
            revision: self.frontier_revision,
            child_task_id: self
                .active_child
                .as_ref()
                .map(|child| child.request_id.clone())
                .unwrap_or_default(),
            map_version_id: self.map_version_id(),
            map_revision: self.map_revision(),
        }
    }
}

impl Default for ExploreRuntimeInner {
    fn default() -> Self {
        Self::new()
    }
}

fn build_controller(
    policy: ExplorationArbitrationPolicy,
    config: &ExploreStartConfig,
) -> ExplorationController {
    let mut controller = ExplorationController::new(policy);
    let mut adapter =
        WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
    adapter.configure_constraints(
        config.boundary.clone(),
        config.max_navigation_retries,
        config.blacklist_radius_m,
    );
    controller.add_source(adapter);
    controller
}

#[derive(Clone)]
pub struct ExploreRuntimeHandle {
    inner: Arc<Mutex<ExploreRuntimeInner>>,
    readiness_cache: Arc<SyncMutex<ExplorationReadinessDetail>>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    navigation_sink: Arc<RwLock<Option<Arc<dyn ExploreNavigationSink>>>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    generation: Arc<AtomicU64>,
}

impl ExploreRuntimeHandle {
    pub fn default_with_wavefront() -> Self {
        let (frontier_tx, _) = watch::channel(FrontierSnapshot::default());
        Self {
            inner: Arc::new(Mutex::new(ExploreRuntimeInner::new())),
            readiness_cache: Arc::new(SyncMutex::new(ExplorationReadinessDetail {
                ready: false,
                reason: "exploration controller not started".to_string(),
                backend_available: false,
                controller_active: false,
                source_count: 1,
                frontier_source_ready: false,
                costmap_input_ready: false,
                costmap_input_revision: 0,
                navigation_sink_ready: false,
                active_source: "wavefront".to_string(),
                arbitration: POLICY_PREFER_WAVEFRONT.to_string(),
            })),
            snapshot_cache: Arc::new(RwLock::new(ExploreRuntimeSnapshot::default())),
            navigation_sink: Arc::new(RwLock::new(None)),
            frontier_tx,
            generation: Arc::new(AtomicU64::new(0)),
        }
    }

    pub fn set_navigation_sink(&self, sink: Arc<dyn ExploreNavigationSink>) {
        *self.navigation_sink.write() = Some(sink);
    }

    pub fn clear_navigation_sink(&self) {
        *self.navigation_sink.write() = None;
    }

    pub fn subscribe_frontiers(&self) -> watch::Receiver<FrontierSnapshot> {
        self.frontier_tx.subscribe()
    }

    pub fn frontier_snapshot(&self) -> FrontierSnapshot {
        self.frontier_tx.borrow().clone()
    }

    pub fn snapshot(&self) -> ExploreRuntimeSnapshot {
        self.snapshot_cache.read().clone()
    }

    fn store_snapshot(&self, snapshot: ExploreRuntimeSnapshot) -> ExploreRuntimeSnapshot {
        *self.snapshot_cache.write() = snapshot.clone();
        snapshot
    }

    fn publish_frontiers(&self, snapshot: Option<FrontierSnapshot>) {
        if let Some(snapshot) = snapshot {
            self.frontier_tx.send_replace(snapshot);
        }
    }

    pub async fn arbitration(&self) -> ExplorationArbitrationPolicy {
        self.inner.lock().await.arbitration
    }

    #[cfg(test)]
    pub async fn start(&self, request_id: &str, arbitration: &str) -> ExploreRuntimeSnapshot {
        self.start_with_config(
            request_id,
            ExploreStartConfig {
                arbitration: arbitration.to_string(),
                motion_admission: crate::nav_runtime::test_motion_admission(),
                ..ExploreStartConfig::default()
            },
        )
        .await
    }

    pub async fn start_with_config(
        &self,
        request_id: &str,
        mut config: ExploreStartConfig,
    ) -> ExploreRuntimeSnapshot {
        {
            let guard = self.inner.lock().await;
            if guard.session_in_progress() {
                return ExploreRuntimeSnapshot {
                    status: STATUS_REJECTED,
                    message: format!(
                        "exploration already active (request_id={})",
                        guard.last_request_id
                    ),
                    request_id: request_id.to_string(),
                    selected_source: guard.last_selected_source.clone(),
                    last_stop_reason: guard.last_stop_reason.clone(),
                    frontier_count: guard.frontiers.len() as u32,
                    revision: guard.frontier_revision,
                    child_task_id: String::new(),
                    map_version_id: guard.map_version_id(),
                    map_revision: guard.map_revision(),
                };
            }
        }
        let policy = match config.validate() {
            Ok(policy) => policy,
            Err(error) => {
                let mut snapshot = self.snapshot();
                snapshot.status = STATUS_REJECTED;
                snapshot.message = error;
                snapshot.request_id = request_id.to_string();
                return self.store_snapshot(snapshot);
            }
        };
        if self.navigation_sink.read().is_none() {
            let mut snapshot = self.snapshot();
            snapshot.status = STATUS_REJECTED;
            snapshot.message =
                "navigation sink unavailable; exploration was not started".to_string();
            snapshot.request_id = request_id.to_string();
            return self.store_snapshot(snapshot);
        }

        let mut guard = self.inner.lock().await;
        if guard.session_in_progress() {
            let snapshot = ExploreRuntimeSnapshot {
                status: STATUS_REJECTED,
                message: format!(
                    "exploration already active (request_id={})",
                    guard.last_request_id
                ),
                request_id: request_id.to_string(),
                selected_source: guard.last_selected_source.clone(),
                last_stop_reason: guard.last_stop_reason.clone(),
                frontier_count: guard.frontiers.len() as u32,
                revision: guard.frontier_revision,
                child_task_id: String::new(),
                map_version_id: guard.map_version_id(),
                map_revision: guard.map_revision(),
            };
            drop(guard);
            return snapshot;
        }

        if let Some(costmap) = guard.costmap_input.as_ref() {
            if config.map_version_id.is_empty() {
                config.map_version_id = costmap.map_version_id.clone();
            }
        }

        let mut controller = build_controller(policy, &config);
        if let Err(error) = controller.begin_exploration() {
            let snapshot = ExploreRuntimeSnapshot {
                status: STATUS_REJECTED,
                message: format!("begin_exploration failed: {error}"),
                request_id: request_id.to_string(),
                ..guard.snapshot(STATUS_REJECTED, "begin_exploration failed")
            };
            drop(guard);
            return self.store_snapshot(snapshot);
        }

        let generation = self.generation.fetch_add(1, Ordering::SeqCst) + 1;
        guard.controller = controller;
        guard.arbitration = policy;
        guard.last_request_id = request_id.to_string();
        guard.last_selected_source.clear();
        guard.last_stop_reason.clear();
        guard.last_stop = None;
        guard.last_frontier_goal = None;
        guard.last_poll = None;
        guard.active_config = Some(config.clone());
        guard.active_child = None;
        guard.pending_child_dispatch = None;
        guard.pending_child_cancellation = None;
        guard.generation = generation;
        guard.child_sequence = 0;
        guard.reached_goals = 0;
        guard.started_at_ns = now_unix_nanos();
        guard.no_frontier_since_ns = None;
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_ACCEPTED,
            format!(
                "exploration accepted (source=wavefront, arbitration={}, map_revision={})",
                arbitration_policy_str(policy),
                config.map_revision
            ),
        );
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontier_update);
        self.store_snapshot(snapshot)
    }

    pub async fn stop(&self, request_id: &str) -> ExploreRuntimeSnapshot {
        self.finish_session(
            Some(request_id),
            ExplorationStopReason::UserCancelled,
            "exploration cancelled by caller".to_string(),
        )
        .await
    }

    async fn finish_session(
        &self,
        request_id: Option<&str>,
        reason: ExplorationStopReason,
        message: String,
    ) -> ExploreRuntimeSnapshot {
        self.begin_child_action(request_id, PendingChildAction { reason, message })
            .await
    }

    async fn begin_child_action(
        &self,
        request_id: Option<&str>,
        action: PendingChildAction,
    ) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if let Some(requested_id) = request_id.map(str::trim) {
            if requested_id.is_empty() || requested_id != guard.last_request_id {
                let mut snapshot = guard.snapshot(
                    STATUS_REJECTED,
                    format!(
                        "exploration stop request_id mismatch: active={} received={}",
                        guard.last_request_id, requested_id
                    ),
                );
                snapshot.request_id = requested_id.to_string();
                return snapshot;
            }
        }
        if guard.pending_child_cancellation.is_some() {
            let (failure, dispatch_in_flight) = guard
                .pending_child_cancellation
                .as_ref()
                .map(|pending| (pending.failure.clone(), pending.dispatch_in_flight))
                .expect("pending child cancellation checked above");
            let retry_correlated = request_id.is_some() && failure.is_some() && !dispatch_in_flight;
            if !retry_correlated {
                let status = if failure.is_some() && !dispatch_in_flight {
                    STATUS_FAILED
                } else {
                    STATUS_STOPPING
                };
                let message = if dispatch_in_flight && failure.is_some() {
                    "child navigation cancellation retry is already in flight"
                } else {
                    "child navigation cancellation remains pending ZEROED proof"
                };
                let snapshot = guard.snapshot(status, message);
                let snapshot = self.store_snapshot(snapshot);
                drop(guard);
                return snapshot;
            }
            let Some(child) = guard.active_child.clone() else {
                let snapshot = guard.snapshot(
                    STATUS_FAILED,
                    "pending child cancellation lost its active safety lease",
                );
                drop(guard);
                return self.store_snapshot(snapshot);
            };
            guard
                .pending_child_cancellation
                .as_mut()
                .expect("pending child cancellation checked above")
                .dispatch_in_flight = true;
            let snapshot = guard.snapshot(
                STATUS_STOPPING,
                format!(
                    "CANCEL_RETRY_PENDING: re-dispatching child {} cancellation",
                    child.request_id
                ),
            );
            self.store_snapshot(snapshot);
            drop(guard);
            return self.dispatch_child_cancellation(child, true).await;
        }
        if guard.last_request_id.is_empty() {
            if let Some(request_id) = request_id.filter(|value| !value.trim().is_empty()) {
                guard.last_request_id = request_id.to_string();
            }
        }
        let generation = self.generation.fetch_add(1, Ordering::SeqCst) + 1;
        guard.generation = generation;
        let _ = guard.controller.stop_with_reason(action.reason);
        guard.last_stop = Some(action.reason);
        guard.last_stop_reason = stop_reason_str(action.reason).to_string();
        guard.last_poll = Some(ExplorationPoll::Stop(action.reason));
        guard.pending_child_dispatch = None;

        let Some(child) = guard.active_child.clone() else {
            let status = status_for_stop_reason(action.reason);
            let message = action.message;
            let frontier_update = if matches!(status, STATUS_PENDING | STATUS_ACTIVE) {
                None
            } else {
                guard.update_frontiers(Vec::new())
            };
            let snapshot = guard.snapshot(status, message);
            let backend_available = self.readiness_cache.lock().backend_available;
            let readiness = Self::compute_readiness_locked(
                &guard,
                backend_available,
                self.navigation_sink.read().is_some(),
            );
            drop(guard);
            *self.readiness_cache.lock() = readiness;
            self.publish_frontiers(frontier_update);
            return self.store_snapshot(snapshot);
        };

        guard.pending_child_cancellation = Some(PendingChildCancellation {
            action,
            ticket: None,
            failure: None,
            dispatch_in_flight: true,
            early_terminals: Vec::new(),
        });
        let snapshot = guard.snapshot(
            STATUS_STOPPING,
            format!(
                "CANCEL_PENDING: retaining child {} until actuator ZEROED proof",
                child.request_id
            ),
        );
        self.store_snapshot(snapshot);
        drop(guard);

        self.dispatch_child_cancellation(child, false).await
    }

    async fn dispatch_child_cancellation(
        &self,
        child: ActiveChildNavigation,
        retry: bool,
    ) -> ExploreRuntimeSnapshot {
        let Some(sink) = self.navigation_sink.read().clone() else {
            return self
                .record_child_cancel_failure(
                    &child.lease,
                    "navigation sink unavailable during child cancellation".to_string(),
                )
                .await;
        };
        let ticket = match sink.cancel(&child.lease).await {
            Ok(ticket) if ticket.cancel_generation > 0 && ticket.lease.matches(&child.lease) => {
                ticket
            }
            Ok(_) => {
                return self
                    .record_child_cancel_failure(
                        &child.lease,
                        "navigation sink returned an invalid cancellation ticket".to_string(),
                    )
                    .await;
            }
            Err(error) => {
                return self.record_child_cancel_failure(&child.lease, error).await;
            }
        };

        let mut guard = self.inner.lock().await;
        if guard
            .active_child
            .as_ref()
            .is_none_or(|active| !active.lease.matches(&child.lease))
        {
            return self.snapshot();
        }
        let Some(pending) = guard.pending_child_cancellation.as_mut() else {
            return self.snapshot();
        };
        if !pending.dispatch_in_flight {
            return self.snapshot();
        }
        pending.ticket = Some(ticket.clone());
        pending.dispatch_in_flight = false;
        let early_terminal = take_early_cancel_terminal(pending, ticket.cancel_generation);
        let snapshot = guard.snapshot(
            STATUS_STOPPING,
            format!(
                "{}: child {} cancel_generation={} awaiting ZEROED",
                if retry {
                    "CANCEL_RETRY_PENDING"
                } else {
                    "CANCEL_PENDING"
                },
                child.request_id,
                ticket.cancel_generation
            ),
        );
        let snapshot = self.store_snapshot(snapshot);
        drop(guard);
        if let Some(result) = early_terminal {
            return self
                .report_navigation_result(&child.request_id, result)
                .await
                .unwrap_or_else(|_| self.snapshot());
        }
        snapshot
    }

    async fn record_child_cancel_failure(
        &self,
        child_lease: &ChildNavigationLease,
        error: String,
    ) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if guard
            .active_child
            .as_ref()
            .is_none_or(|child| !child.lease.matches(child_lease))
        {
            return self.snapshot();
        }
        let Some(pending) = guard.pending_child_cancellation.as_mut() else {
            return self.snapshot();
        };
        pending.dispatch_in_flight = false;
        pending.failure = Some(error.clone());
        let _ = guard
            .controller
            .stop_with_reason(ExplorationStopReason::ConsecutiveFailuresExceeded);
        guard.last_stop = Some(ExplorationStopReason::ConsecutiveFailuresExceeded);
        guard.last_stop_reason = "child_cancel_failed".to_string();
        guard.last_poll = Some(ExplorationPoll::Stop(
            ExplorationStopReason::ConsecutiveFailuresExceeded,
        ));
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_FAILED,
            format!("child cancellation failed; safety ownership retained: {error}"),
        );
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        *self.readiness_cache.lock() = readiness;
        let snapshot = self.store_snapshot(snapshot);
        drop(guard);
        self.publish_frontiers(frontier_update);
        snapshot
    }

    pub async fn tick(&self, robot_xy: (f64, f64)) -> ExploreRuntimeSnapshot {
        let now_monotonic_ns = now_monotonic_nanos();
        self.tick_with_pose(ExploreRobotPose {
            x: robot_xy.0,
            y: robot_xy.1,
            frame_id: "map".to_string(),
            source_timestamp_ns: now_unix_nanos(),
            received_monotonic_ns: now_monotonic_ns,
        })
        .await
    }

    pub async fn tick_with_pose(&self, pose: ExploreRobotPose) -> ExploreRuntimeSnapshot {
        let generation = self.generation.load(Ordering::SeqCst);
        self.tick_generation(generation, pose).await
    }

    async fn tick_generation(
        &self,
        generation: u64,
        pose: ExploreRobotPose,
    ) -> ExploreRuntimeSnapshot {
        if normalize_frame(&pose.frame_id) != "map" {
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::MapRevisionChanged,
                    format!(
                        "global exploration requires map-frame pose; received frame={}",
                        pose.frame_id
                    ),
                )
                .await;
        }
        let pose_age_ns = now_monotonic_nanos().saturating_sub(pose.received_monotonic_ns);
        if pose.source_timestamp_ns == 0
            || pose.received_monotonic_ns == 0
            || pose_age_ns > DEFAULT_POSE_RECEIPT_TTL.as_nanos() as u64
        {
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!(
                        "pose source stale (source_timestamp_ns={}, receipt_age_ms={})",
                        pose.source_timestamp_ns,
                        pose_age_ns / 1_000_000
                    ),
                )
                .await;
        }

        let now_ns = now_unix_nanos();
        let mut guard = self.inner.lock().await;
        if generation != guard.generation || !guard.controller.is_active() {
            return self.snapshot();
        }

        if guard.pending_child_cancellation.is_some() {
            let snapshot = guard.snapshot(
                STATUS_STOPPING,
                "child navigation cancellation is awaiting actuator ZEROED proof",
            );
            drop(guard);
            return self.store_snapshot(snapshot);
        }

        if let Some(reason) = evaluate_stop_conditions(&guard, now_ns) {
            drop(guard);
            return self
                .finish_session(
                    None,
                    reason,
                    format!(
                        "exploration stop condition reached: {}",
                        stop_reason_str(reason)
                    ),
                )
                .await;
        }

        if guard.active_child.is_some() {
            let snapshot =
                guard.snapshot(STATUS_ACTIVE, "waiting for child navigation terminal state");
            drop(guard);
            return self.store_snapshot(snapshot);
        }

        let Some(costmap) = guard.costmap_input.clone() else {
            let snapshot = guard.snapshot(STATUS_PENDING, "source unavailable: missing costmap");
            drop(guard);
            return self.store_snapshot(snapshot);
        };
        if normalize_frame(&costmap.frame_id) != "map" {
            drop(guard);
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::MapRevisionChanged,
                    format!(
                        "global exploration requires map-frame costmap; received frame={}",
                        costmap.frame_id
                    ),
                )
                .await;
        }
        if let Some(mut request) = guard.pending_child_dispatch.take() {
            request.map_revision = costmap.map_revision;
            if request.map_version_id.is_empty() {
                request.map_version_id = costmap.map_version_id.clone();
            }
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                format!(
                    "retrying frontier {} after navigation readiness wait",
                    request.frontier_goal_id
                ),
            );
            self.store_snapshot(snapshot);
            return self.dispatch_child(generation, request, guard).await;
        }

        let mut state = ExplorationStateSnapshot::from_xy(
            (pose.x, pose.y),
            Timestamp {
                nanos: now_ns as i64,
                clock: ClockKind::Unix,
            },
        );
        state.global_costmap = Some(&costmap.grid);
        let poll_result = guard.controller.poll_next_goal(&state);
        let diagnostics = guard.controller.diagnostics();
        guard.last_selected_source = diagnostics.selected_source.unwrap_or_default();
        let source_frontiers = guard.controller.frontier_snapshot();
        let frontier_update = guard.update_frontiers(source_frontiers);

        match poll_result {
            Ok(ExplorationPoll::Goal(goal)) => {
                guard.last_frontier_goal = Some(goal.clone());
                guard.last_poll = Some(ExplorationPoll::Goal(goal.clone()));
                guard.no_frontier_since_ns = None;
                guard.child_sequence = guard.child_sequence.saturating_add(1);
                let child_request_id =
                    format!("{}.nav.{}", guard.last_request_id, guard.child_sequence);
                let mut navigation_goal = frontier_goal_to_navigation_goal(&goal);
                navigation_goal.id = child_request_id.clone();
                let config = guard.active_config.clone().unwrap_or_default();
                let request = ChildNavigationRequest {
                    parent_request_id: guard.last_request_id.clone(),
                    request_id: child_request_id,
                    map_version_id: config.map_version_id,
                    map_revision: costmap.map_revision,
                    motion_admission: config.motion_admission,
                    frontier_goal_id: goal.id.clone(),
                    goal: navigation_goal,
                    boundary: config.boundary,
                };
                let dispatching = guard.snapshot(
                    STATUS_PENDING,
                    format!("dispatching frontier {} to child navigation", goal.id),
                );
                self.publish_frontiers(frontier_update);
                self.store_snapshot(dispatching);
                self.dispatch_child(generation, request, guard).await
            }
            Ok(ExplorationPoll::Pending) => {
                let first_no_frontier = *guard.no_frontier_since_ns.get_or_insert(now_ns);
                guard.last_poll = Some(ExplorationPoll::Pending);
                if guard
                    .active_config
                    .as_ref()
                    .and_then(|config| config.stop_conditions.no_frontier_timeout_ms)
                    .is_some_and(|limit_ms| {
                        now_ns.saturating_sub(first_no_frontier)
                            >= limit_ms.saturating_mul(1_000_000)
                    })
                {
                    drop(guard);
                    self.publish_frontiers(frontier_update);
                    return self
                        .finish_session(
                            None,
                            ExplorationStopReason::NoFrontierTimeout,
                            "no frontier timeout reached".to_string(),
                        )
                        .await;
                }
                let snapshot = guard.snapshot(STATUS_PENDING, "no frontier available yet");
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.store_snapshot(snapshot)
            }
            Ok(ExplorationPoll::SourceUnavailable(reason)) => {
                guard.last_poll = Some(ExplorationPoll::SourceUnavailable(reason.clone()));
                let snapshot =
                    guard.snapshot(STATUS_PENDING, format!("source unavailable: {reason}"));
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.store_snapshot(snapshot)
            }
            Ok(ExplorationPoll::Stop(reason)) => {
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.finish_session(
                    None,
                    reason,
                    format!("exploration completed: {}", stop_reason_str(reason)),
                )
                .await
            }
            Err(error) => {
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!("exploration controller error: {error}"),
                )
                .await
            }
        }
    }

    async fn dispatch_child(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        mut guard: tokio::sync::MutexGuard<'_, ExploreRuntimeInner>,
    ) -> ExploreRuntimeSnapshot {
        if generation != guard.generation || !guard.controller.is_active() {
            return self.snapshot();
        }
        let Some(sink) = self.navigation_sink.read().clone() else {
            drop(guard);
            return self
                .record_dispatch_failure(
                    generation,
                    request,
                    ChildNavigationSubmitError::Rejected("navigation sink unavailable".to_string()),
                )
                .await;
        };
        let proposed_request_id = request.request_id.clone();
        let expected_parent_request_id = request.parent_request_id.clone();
        let frontier_goal_id = request.frontier_goal_id.clone();
        let ticket = match sink.submit(request.clone()).await {
            Ok(ticket) => ticket,
            Err(error) => {
                drop(guard);
                return self
                    .record_dispatch_failure(generation, request, error)
                    .await;
            }
        };
        if ticket.lease.session_id != proposed_request_id
            || ticket.lease.parent_request_id != expected_parent_request_id
            || ticket.lease.owner_kind == 0
            || ticket.lease.goal_generation == 0
            || ticket.lease.goal_payload_sha256 == [0; 32]
        {
            drop(guard);
            return self
                .record_dispatch_failure(
                    generation,
                    request,
                    ChildNavigationSubmitError::Rejected(
                        "navigation sink returned a mismatched child goal lease".to_string(),
                    ),
                )
                .await;
        }
        guard.pending_child_dispatch = None;
        let child_request_id = ticket.lease.session_id.clone();
        guard.active_child = Some(ActiveChildNavigation {
            request_id: child_request_id.clone(),
            frontier_goal_id,
            lease: ticket.lease,
        });
        let snapshot = guard.snapshot(
            STATUS_ACTIVE,
            format!("child navigation accepted: {child_request_id}"),
        );
        drop(guard);
        self.store_snapshot(snapshot)
    }

    async fn record_dispatch_failure(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        error: ChildNavigationSubmitError,
    ) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if generation != guard.generation || !guard.controller.is_active() {
            return self.snapshot();
        }
        let reason = error.reason().to_string();
        if error.is_retryable() {
            guard.pending_child_dispatch = Some(request);
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                format!("waiting for child navigation readiness: {reason}"),
            );
            drop(guard);
            return self.store_snapshot(snapshot);
        }
        guard.pending_child_dispatch = None;
        let _ = guard.controller.report_goal_feedback(
            &request.frontier_goal_id,
            ExplorationGoalFeedback::Failed(reason.clone()),
        );
        let frontiers = guard.controller.frontier_snapshot();
        let frontier_update = guard.update_frontiers(frontiers);
        let snapshot = guard.snapshot(
            STATUS_PENDING,
            format!("child navigation dispatch failed: {reason}"),
        );
        drop(guard);
        self.publish_frontiers(frontier_update);
        self.store_snapshot(snapshot)
    }

    pub async fn report_navigation_result(
        &self,
        child_request_id: &str,
        result: ChildNavigationResult,
    ) -> Result<ExploreRuntimeSnapshot, String> {
        let mut guard = self.inner.lock().await;
        let Some(active_child) = guard.active_child.as_ref() else {
            return Err("no active exploration child navigation".to_string());
        };
        if active_child.request_id != child_request_id {
            return Err(format!(
                "stale child navigation result: expected {}, received {}",
                active_child.request_id, child_request_id
            ));
        }
        let received_lease = match &result {
            ChildNavigationResult::Reached(proof) => &proof.lease,
            ChildNavigationResult::Failed { proof, .. }
            | ChildNavigationResult::CancelFailed { proof, .. } => &proof.lease,
            ChildNavigationResult::Cancelled(proof) => &proof.lease,
        };
        if !active_child.lease.matches(received_lease) {
            return Err(
                "child navigation terminal lease does not match active ownership".to_string(),
            );
        }

        if guard.pending_child_cancellation.is_some() {
            match &result {
                ChildNavigationResult::CancelFailed {
                    cancel_generation,
                    reason,
                    ..
                } => {
                    let (expected_generation, dispatch_in_flight) = guard
                        .pending_child_cancellation
                        .as_ref()
                        .map(|pending| {
                            (
                                pending
                                    .ticket
                                    .as_ref()
                                    .map(|ticket| ticket.cancel_generation)
                                    .unwrap_or(0),
                                pending.dispatch_in_flight,
                            )
                        })
                        .expect("pending child cancellation checked above");
                    if *cancel_generation == 0 {
                        return Err("cancel failure generation must be non-zero".to_string());
                    }
                    if *cancel_generation != expected_generation && dispatch_in_flight {
                        let pending = guard
                            .pending_child_cancellation
                            .as_mut()
                            .expect("pending child cancellation checked above");
                        retain_early_cancel_terminal(pending, result.clone());
                        let snapshot = guard.snapshot(
                            STATUS_STOPPING,
                            format!(
                                "CANCEL_EARLY_TERMINAL_BUFFERED: generation={} awaiting ticket",
                                cancel_generation
                            ),
                        );
                        drop(guard);
                        return Ok(self.store_snapshot(snapshot));
                    }
                    if dispatch_in_flight && *cancel_generation == expected_generation {
                        let snapshot = guard.snapshot(
                            STATUS_STOPPING,
                            format!(
                                "CANCEL_RETRY_OLD_FAILURE_IGNORED: generation={cancel_generation}"
                            ),
                        );
                        drop(guard);
                        return Ok(self.store_snapshot(snapshot));
                    }
                    if *cancel_generation != expected_generation {
                        return Err(format!(
                            "cancel failure generation mismatch: expected {expected_generation}, received {cancel_generation}"
                        ));
                    }
                    let child_lease = active_child.lease.clone();
                    let reason = reason.clone();
                    drop(guard);
                    return Ok(self.record_child_cancel_failure(&child_lease, reason).await);
                }
                ChildNavigationResult::Cancelled(proof) => {
                    if proof.cancel_generation == 0
                        || proof.zero_ack_id.trim().is_empty()
                        || proof.zero_applied_monotonic_ns == 0
                    {
                        return Err(
                            "incomplete or mismatched child navigation ZEROED proof".to_string()
                        );
                    }
                    let (expected_generation, dispatch_in_flight) = guard
                        .pending_child_cancellation
                        .as_ref()
                        .map(|pending| {
                            (
                                pending
                                    .ticket
                                    .as_ref()
                                    .map(|ticket| ticket.cancel_generation)
                                    .unwrap_or(0),
                                pending.dispatch_in_flight,
                            )
                        })
                        .expect("pending child cancellation checked above");
                    if proof.cancel_generation != expected_generation && dispatch_in_flight {
                        let pending = guard
                            .pending_child_cancellation
                            .as_mut()
                            .expect("pending child cancellation checked above");
                        retain_early_cancel_terminal(pending, result.clone());
                        let snapshot = guard.snapshot(
                            STATUS_STOPPING,
                            format!(
                                "ZEROED_EARLY_BUFFERED: generation={} awaiting ticket",
                                proof.cancel_generation
                            ),
                        );
                        drop(guard);
                        return Ok(self.store_snapshot(snapshot));
                    }
                    if proof.cancel_generation != expected_generation {
                        return Err(format!(
                            "child navigation ZEROED generation mismatch: expected {expected_generation}, received {}",
                            proof.cancel_generation
                        ));
                    }
                }
                ChildNavigationResult::Reached(_) | ChildNavigationResult::Failed { .. } => {
                    return Err(
                        "child terminal cannot bypass pending actuator ZEROED proof".to_string()
                    );
                }
            }

            let pending = guard
                .pending_child_cancellation
                .take()
                .expect("pending child cancellation checked above");
            guard
                .active_child
                .take()
                .expect("active child checked above");
            guard.no_frontier_since_ns = None;
            let (status, message, frontier_update) = if let Some(error) = pending.failure {
                let _ = guard
                    .controller
                    .stop_with_reason(ExplorationStopReason::ConsecutiveFailuresExceeded);
                guard.last_stop = Some(ExplorationStopReason::ConsecutiveFailuresExceeded);
                guard.last_stop_reason = "child_cancel_failed".to_string();
                (
                    STATUS_FAILED,
                    format!("child cancellation failed before late ZEROED proof: {error}"),
                    guard.update_frontiers(Vec::new()),
                )
            } else {
                (
                    status_for_stop_reason(pending.action.reason),
                    pending.action.message,
                    guard.update_frontiers(Vec::new()),
                )
            };
            let snapshot = guard.snapshot(status, message);
            let backend_available = self.readiness_cache.lock().backend_available;
            let readiness = Self::compute_readiness_locked(
                &guard,
                backend_available,
                self.navigation_sink.read().is_some(),
            );
            drop(guard);
            *self.readiness_cache.lock() = readiness;
            self.publish_frontiers(frontier_update);
            return Ok(self.store_snapshot(snapshot));
        }

        let frontier_goal_id = active_child.frontier_goal_id.clone();
        let feedback = match &result {
            ChildNavigationResult::Reached(_) => ExplorationGoalFeedback::Reached,
            ChildNavigationResult::Failed { reason, .. } => {
                ExplorationGoalFeedback::Failed(reason.clone())
            }
            ChildNavigationResult::Cancelled(_) | ChildNavigationResult::CancelFailed { .. } => {
                return Err("unexpected cancellation terminal without a pending action".to_string())
            }
        };
        guard
            .controller
            .report_goal_feedback(&frontier_goal_id, feedback)
            .map_err(|error| error.to_string())?;
        if matches!(&result, ChildNavigationResult::Reached(_)) {
            guard.reached_goals = guard.reached_goals.saturating_add(1);
        }
        guard.active_child = None;
        guard.no_frontier_since_ns = None;
        let frontiers = guard.controller.frontier_snapshot();
        let frontier_update = guard.update_frontiers(frontiers);
        let message = match result {
            ChildNavigationResult::Reached(_) => {
                "child navigation reached; selecting next frontier"
            }
            ChildNavigationResult::Failed { .. } => {
                "child navigation failed; retry/blacklist policy applied"
            }
            ChildNavigationResult::Cancelled(_) | ChildNavigationResult::CancelFailed { .. } => {
                unreachable!("cancellation variants returned above")
            }
        };
        let snapshot = guard.snapshot(STATUS_ACTIVE, message);
        drop(guard);
        self.publish_frontiers(frontier_update);
        Ok(self.store_snapshot(snapshot))
    }

    pub async fn ingest_costmap(&self, costmap: OccupancyGrid2D) -> bool {
        let next_revision = {
            let guard = self.inner.lock().await;
            guard.map_revision().saturating_add(1).max(1)
        };
        self.ingest_costmap_snapshot(ExploreCostmapSnapshot {
            grid: costmap,
            frame_id: "map".to_string(),
            map_version_id: String::new(),
            map_revision: next_revision,
        })
        .await
        .unwrap_or(false)
    }

    pub async fn ingest_costmap_snapshot(
        &self,
        snapshot: ExploreCostmapSnapshot,
    ) -> Result<bool, String> {
        validate_costmap_snapshot(&snapshot)?;
        let mut guard = self.inner.lock().await;
        if guard.costmap_input.as_ref() == Some(&snapshot) {
            return Ok(false);
        }

        if let Some(config) = guard.active_config.as_mut() {
            if config.map_version_id.is_empty() {
                config.map_version_id = snapshot.map_version_id.clone();
            }
        }
        // 实时地图只刷新下一轮探索输入，不取消当前目标。
        guard.costmap_input = Some(snapshot);
        let current_frontiers = guard.frontiers.clone();
        let frontier_update = guard.update_frontiers(current_frontiers);
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontier_update);

        Ok(true)
    }

    pub fn refresh_readiness_cache_with_backend(&self, backend_available: bool) {
        if let Ok(guard) = self.inner.try_lock() {
            let detail = Self::compute_readiness_locked(
                &guard,
                backend_available,
                self.navigation_sink.read().is_some(),
            );
            *self.readiness_cache.lock() = detail;
        }
    }

    pub async fn is_active(&self) -> bool {
        self.inner.lock().await.session_in_progress()
    }

    /// Mapping must retain its costmap while exploration or child navigation
    /// still owns motion-safety state, including a failed cancellation barrier.
    pub async fn blocks_mapping_mutation(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.controller.is_active()
            || guard.active_child.is_some()
            || guard.pending_child_cancellation.is_some()
    }

    pub async fn readiness(&self, backend_available: bool) -> ExplorationReadinessDetail {
        let guard = self.inner.lock().await;
        let detail = Self::compute_readiness_locked(
            &guard,
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        *self.readiness_cache.lock() = detail.clone();
        detail
    }

    pub fn readiness_sync(&self, backend_available: bool) -> ExplorationReadinessDetail {
        let cached = self.readiness_cache.lock().clone();
        if let Ok(guard) = self.inner.try_lock() {
            let detail = Self::compute_readiness_locked(
                &guard,
                backend_available,
                self.navigation_sink.read().is_some(),
            );
            *self.readiness_cache.lock() = detail.clone();
            detail
        } else {
            cached
        }
    }

    pub fn cached_readiness(&self) -> ExplorationReadinessDetail {
        self.readiness_cache.lock().clone()
    }

    fn compute_readiness_locked(
        guard: &ExploreRuntimeInner,
        backend_available: bool,
        navigation_sink_ready: bool,
    ) -> ExplorationReadinessDetail {
        let controller_active = guard.controller.is_active();
        let source_count = guard.controller.source_count();
        let frontier_source_ready = controller_active && source_count > 0;
        let costmap_input_ready = guard
            .costmap_input
            .as_ref()
            .is_some_and(|snapshot| validate_costmap_snapshot(snapshot).is_ok());
        let ready = backend_available
            && frontier_source_ready
            && costmap_input_ready
            && navigation_sink_ready;
        let reason = if !backend_available {
            "LCM backend unavailable"
        } else if !controller_active {
            "exploration controller not active"
        } else if source_count == 0 {
            "no exploration source registered"
        } else if !costmap_input_ready {
            "valid map-frame costmap not available"
        } else if !navigation_sink_ready {
            "navigation sink unavailable"
        } else {
            "exploration runtime ready"
        };
        ExplorationReadinessDetail {
            ready,
            reason: reason.to_string(),
            backend_available,
            controller_active,
            source_count,
            frontier_source_ready,
            costmap_input_ready,
            costmap_input_revision: guard.map_revision(),
            navigation_sink_ready,
            active_source: guard
                .controller
                .diagnostics()
                .selected_source
                .unwrap_or_else(|| "wavefront".to_string()),
            arbitration: arbitration_policy_str(guard.arbitration).to_string(),
        }
    }

    pub async fn diagnostics(&self) -> ControllerDiagnosticsView {
        let guard = self.inner.lock().await;
        let diagnostics = guard.controller.diagnostics();
        ControllerDiagnosticsView {
            active: diagnostics.active,
            arbitration: arbitration_policy_str(guard.arbitration).to_string(),
            selected_source: diagnostics.selected_source.unwrap_or_default(),
            last_stop_reason: guard.last_stop_reason.clone(),
            sources: diagnostics
                .sources
                .into_iter()
                .map(|source| SourceDiagView {
                    source_name: source.source_name,
                    is_active: source.is_active,
                    last_goal_id: source.last_goal_id.unwrap_or_default(),
                    last_error: source.last_error.unwrap_or_default(),
                })
                .collect(),
            frontier_count: guard.frontiers.len() as u32,
            revision: guard.frontier_revision,
        }
    }

    pub fn spawn_scheduler(
        &self,
        pose_provider: Arc<dyn ExplorePoseProvider>,
        period: Duration,
    ) -> ExploreSchedulerHandle {
        ExploreSchedulerHandle::spawn(self.clone(), pose_provider, period)
    }
}

fn validate_costmap_snapshot(snapshot: &ExploreCostmapSnapshot) -> Result<(), String> {
    if normalize_frame(&snapshot.frame_id) != "map" {
        return Err(format!(
            "global exploration requires map-frame costmap; received frame={}",
            snapshot.frame_id
        ));
    }
    if snapshot.grid.width == 0 || snapshot.grid.height == 0 {
        return Err("costmap dimensions must be non-zero".to_string());
    }
    if snapshot.grid.cells.len() != (snapshot.grid.width as usize) * (snapshot.grid.height as usize)
    {
        return Err("costmap cells length does not match width * height".to_string());
    }
    Ok(())
}

fn normalize_frame(frame_id: &str) -> &str {
    frame_id.trim().trim_start_matches('/')
}

fn evaluate_stop_conditions(
    guard: &ExploreRuntimeInner,
    now_ns: u64,
) -> Option<ExplorationStopReason> {
    let config = guard.active_config.as_ref()?;
    let conditions = &config.stop_conditions;
    if conditions.max_duration_ms.is_some_and(|limit_ms| {
        now_ns.saturating_sub(guard.started_at_ns) >= limit_ms.saturating_mul(1_000_000)
    }) {
        return Some(ExplorationStopReason::DurationExceeded);
    }
    if guard.active_child.is_none()
        && conditions
            .max_goals
            .is_some_and(|max_goals| guard.reached_goals >= max_goals)
    {
        return Some(ExplorationStopReason::MaxGoalsReached);
    }
    if let (Some(max_coverage), Some(costmap)) =
        (conditions.max_coverage_pct, guard.costmap_input.as_ref())
    {
        if coverage_pct(&costmap.grid) >= max_coverage {
            return Some(ExplorationStopReason::CoverageReached);
        }
    }
    None
}

fn coverage_pct(grid: &OccupancyGrid2D) -> f64 {
    if grid.cells.is_empty() {
        return 0.0;
    }
    let known = grid
        .cells
        .iter()
        .filter(|cell| !matches!(cell, OccupancyCell::Unknown))
        .count();
    known as f64 * 100.0 / grid.cells.len() as f64
}

pub struct ExploreSchedulerHandle {
    shutdown_tx: watch::Sender<bool>,
    join: JoinHandle<()>,
}

impl ExploreSchedulerHandle {
    fn spawn(
        runtime: ExploreRuntimeHandle,
        pose_provider: Arc<dyn ExplorePoseProvider>,
        period: Duration,
    ) -> Self {
        Self::spawn_with_pose_grace(
            runtime,
            pose_provider,
            period,
            DEFAULT_POSE_UNAVAILABLE_GRACE,
        )
    }

    fn spawn_with_pose_grace(
        runtime: ExploreRuntimeHandle,
        pose_provider: Arc<dyn ExplorePoseProvider>,
        period: Duration,
        pose_unavailable_grace: Duration,
    ) -> Self {
        let (shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let period = if period.is_zero() {
            DEFAULT_SCHEDULER_PERIOD
        } else {
            period
        };
        let join = tokio::spawn(async move {
            let mut interval = tokio::time::interval(period);
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut pose_unavailable_since = None;
            loop {
                tokio::select! {
                    _ = interval.tick() => {
                        if !runtime.is_active().await {
                            pose_unavailable_since = None;
                            continue;
                        }
                        match pose_provider.current_pose().await {
                            Ok(pose) => {
                                pose_unavailable_since = None;
                                let _ = runtime.tick_with_pose(pose).await;
                            }
                            Err(error) => {
                                if runtime.is_active().await {
                                    let first_unavailable = *pose_unavailable_since
                                        .get_or_insert_with(Instant::now);
                                    let snapshot = runtime.snapshot();
                                    if !snapshot.child_task_id.is_empty()
                                        || first_unavailable.elapsed() >= pose_unavailable_grace
                                    {
                                        let _ = runtime
                                            .finish_session(
                                                None,
                                                ExplorationStopReason::ConsecutiveFailuresExceeded,
                                                format!("pose source unavailable: {error}"),
                                            )
                                            .await;
                                        continue;
                                    }
                                    let mut snapshot = runtime.snapshot();
                                    snapshot.status = STATUS_PENDING;
                                    snapshot.message = format!(
                                        "pose source unavailable within bounded grace: {error}"
                                    );
                                    runtime.store_snapshot(snapshot);
                                }
                            }
                        }
                    }
                    changed = shutdown_rx.changed() => {
                        if changed.is_err() || *shutdown_rx.borrow() {
                            break;
                        }
                    }
                }
            }
        });
        Self { shutdown_tx, join }
    }

    pub async fn shutdown(self) {
        let _ = self.shutdown_tx.send(true);
        let _ = self.join.await;
    }
}

#[derive(Debug, Clone)]
pub struct ExplorationReadinessDetail {
    pub ready: bool,
    pub reason: String,
    pub backend_available: bool,
    pub controller_active: bool,
    pub source_count: usize,
    pub frontier_source_ready: bool,
    pub costmap_input_ready: bool,
    pub costmap_input_revision: u64,
    pub navigation_sink_ready: bool,
    pub active_source: String,
    pub arbitration: String,
}

#[derive(Debug, Clone)]
pub struct SourceDiagView {
    pub source_name: String,
    pub is_active: bool,
    pub last_goal_id: String,
    pub last_error: String,
}

#[derive(Debug, Clone)]
pub struct ControllerDiagnosticsView {
    pub active: bool,
    pub arbitration: String,
    pub selected_source: String,
    pub last_stop_reason: String,
    pub sources: Vec<SourceDiagView>,
    pub frontier_count: u32,
    pub revision: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::borrow::Cow;
    use std::collections::VecDeque;
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct RecordingNavSink {
        submitted: StdMutex<Vec<ChildNavigationRequest>>,
        submit_errors: StdMutex<VecDeque<ChildNavigationSubmitError>>,
        cancelled: StdMutex<Vec<String>>,
        cancel_error: StdMutex<Option<String>>,
        next_goal_generation: AtomicU64,
        next_cancel_generation: AtomicU64,
        submit_entered: tokio::sync::Notify,
        submit_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
        cancel_entered: tokio::sync::Notify,
        cancel_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
    }

    #[async_trait]
    impl ExploreNavigationSink for RecordingNavSink {
        async fn submit(
            &self,
            request: ChildNavigationRequest,
        ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError> {
            let submit_release = { self.submit_release.lock().unwrap().clone() };
            if let Some(release) = submit_release {
                self.submit_entered.notify_one();
                release.notified().await;
            }
            self.submitted.lock().unwrap().push(request.clone());
            if let Some(error) = self.submit_errors.lock().unwrap().pop_front() {
                return Err(error);
            }
            let lease = ChildNavigationLease {
                session_id: request.request_id.clone(),
                owner_kind: 2,
                parent_request_id: request.parent_request_id.clone(),
                goal_generation: self.next_goal_generation.fetch_add(1, Ordering::SeqCst) + 1,
                map_version_id: request.map_version_id.clone(),
                map_revision: request.map_revision,
                goal_payload_sha256: [7; 32],
            };
            Ok(ChildNavigationTicket { lease })
        }

        async fn cancel(
            &self,
            lease: &ChildNavigationLease,
        ) -> Result<ChildNavigationCancelTicket, String> {
            let cancel_release = { self.cancel_release.lock().unwrap().clone() };
            if let Some(release) = cancel_release {
                self.cancel_entered.notify_one();
                release.notified().await;
            }
            self.cancelled
                .lock()
                .unwrap()
                .push(lease.session_id.clone());
            if let Some(error) = self.cancel_error.lock().unwrap().clone() {
                return Err(error);
            }
            Ok(ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: self.next_cancel_generation.fetch_add(1, Ordering::SeqCst) + 1,
            })
        }
    }

    struct StaticPoseProvider;

    #[async_trait]
    impl ExplorePoseProvider for StaticPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            Ok(ExploreRobotPose {
                x: 2.0,
                y: 2.0,
                frame_id: "map".to_string(),
                source_timestamp_ns: now_unix_nanos(),
                received_monotonic_ns: now_monotonic_nanos(),
            })
        }
    }

    struct MissingPoseProvider;

    #[async_trait]
    impl ExplorePoseProvider for MissingPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            Err("test pose stream stale".to_string())
        }
    }

    fn frontier_costmap(revision: u64) -> ExploreCostmapSnapshot {
        let mut grid = OccupancyGrid2D {
            resolution: 0.5,
            origin: nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
            width: 12,
            height: 12,
            cells: vec![OccupancyCell::Unknown; 144],
        };
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        ExploreCostmapSnapshot {
            grid,
            frame_id: "map".to_string(),
            map_version_id: "map-v1".to_string(),
            map_revision: revision,
        }
    }

    fn occupied_costmap(revision: u64) -> ExploreCostmapSnapshot {
        ExploreCostmapSnapshot {
            grid: OccupancyGrid2D {
                resolution: 0.5,
                origin: nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
                width: 8,
                height: 8,
                cells: vec![OccupancyCell::Occupied; 64],
            },
            frame_id: "map".to_string(),
            map_version_id: "map-v1".to_string(),
            map_revision: revision,
        }
    }

    fn configured_handle() -> (ExploreRuntimeHandle, Arc<RecordingNavSink>) {
        let handle = ExploreRuntimeHandle::default_with_wavefront();
        let sink = Arc::new(RecordingNavSink::default());
        handle.set_navigation_sink(sink.clone());
        (handle, sink)
    }

    async fn active_child_lease(handle: &ExploreRuntimeHandle) -> ChildNavigationLease {
        handle
            .inner
            .lock()
            .await
            .active_child
            .as_ref()
            .expect("active child")
            .lease
            .clone()
    }

    fn terminal_proof(lease: &ChildNavigationLease) -> ChildNavigationTerminalProof {
        ChildNavigationTerminalProof {
            lease: lease.clone(),
        }
    }

    fn zero_proof(
        lease: &ChildNavigationLease,
        cancel_generation: u64,
    ) -> ChildNavigationZeroProof {
        ChildNavigationZeroProof {
            lease: lease.clone(),
            cancel_generation,
            zero_ack_id: format!("zero-{cancel_generation}"),
            zero_applied_monotonic_ns: now_monotonic_nanos(),
        }
    }

    #[test]
    fn parse_arbitration_policy_round_trip_and_fallback() {
        for policy in [
            ExplorationArbitrationPolicy::PreferWavefront,
            ExplorationArbitrationPolicy::PreferTare,
            ExplorationArbitrationPolicy::HighestScore,
            ExplorationArbitrationPolicy::FirstAvailable,
        ] {
            assert_eq!(
                parse_arbitration_policy(arbitration_policy_str(policy)),
                policy
            );
        }
        assert_eq!(
            parse_arbitration_policy("unknown"),
            ExplorationArbitrationPolicy::PreferWavefront
        );
    }

    #[test]
    fn stop_reason_statuses_are_not_collapsed() {
        assert_eq!(
            status_for_stop_reason(ExplorationStopReason::NoInformationGain),
            STATUS_SUCCEEDED
        );
        assert_eq!(
            status_for_stop_reason(ExplorationStopReason::DurationExceeded),
            STATUS_SUCCEEDED
        );
        assert_eq!(
            status_for_stop_reason(ExplorationStopReason::UserCancelled),
            STATUS_CANCELLED
        );
        assert_eq!(
            status_for_stop_reason(ExplorationStopReason::ConsecutiveFailuresExceeded),
            STATUS_FAILED
        );
    }

    #[tokio::test]
    async fn start_rejects_without_navigation_sink() {
        let handle = ExploreRuntimeHandle::default_with_wavefront();
        let snapshot = handle.start("req-1", "prefer_wavefront").await;
        assert_eq!(snapshot.status, STATUS_REJECTED);
        assert!(snapshot.message.contains("navigation sink unavailable"));
        assert!(!handle.is_active().await);
    }

    #[tokio::test]
    async fn repeated_start_does_not_replace_active_request() {
        let (handle, _) = configured_handle();
        assert_eq!(
            handle.start("req-1", "prefer_wavefront").await.status,
            STATUS_ACCEPTED
        );
        let rejected = handle.start("req-2", "prefer_wavefront").await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        assert!(rejected.message.contains("req-1"));
        assert_eq!(handle.snapshot().request_id, "req-1");
        assert_eq!(handle.snapshot().status, STATUS_ACCEPTED);
        let diagnostics = handle.diagnostics().await;
        assert!(diagnostics.active);
    }

    #[tokio::test]
    async fn unsupported_tare_source_is_explicitly_rejected() {
        let (handle, _) = configured_handle();
        let snapshot = handle
            .start_with_config(
                "req-tare",
                ExploreStartConfig {
                    strategy: "tare".to_string(),
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        assert_eq!(snapshot.status, STATUS_REJECTED);
        assert!(snapshot.message.contains("unsupported/source unavailable"));
    }

    #[tokio::test]
    async fn explicit_costmap_input_requires_map_frame_but_ignores_sequence() {
        let (handle, _) = configured_handle();
        let mut odom = frontier_costmap(1);
        odom.frame_id = "odom".to_string();
        assert!(handle.ingest_costmap_snapshot(odom).await.is_err());
        assert!(handle
            .ingest_costmap_snapshot(frontier_costmap(2))
            .await
            .unwrap());
        assert!(handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap());
        assert!(!handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap());
    }

    #[tokio::test]
    async fn tick_dispatches_real_child_and_feedback_advances_session() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        assert_eq!(
            handle.start("req-flow", "prefer_wavefront").await.status,
            STATUS_ACCEPTED
        );

        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        assert!(!active.child_task_id.is_empty());
        assert_eq!(active.frontier_count, 1);
        assert!(active.revision > 0);

        let lease = active_child_lease(&handle).await;
        let after_feedback = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Reached(terminal_proof(&lease)),
            )
            .await
            .unwrap();
        assert_eq!(after_feedback.status, STATUS_ACTIVE);
        assert!(after_feedback.child_task_id.is_empty());
        assert_eq!(after_feedback.frontier_count, 0);
        assert!(after_feedback.revision > active.revision);
    }

    #[tokio::test]
    async fn retryable_readiness_wait_does_not_consume_navigation_failures() {
        let (handle, sink) = configured_handle();
        sink.submit_errors.lock().unwrap().extend((0..10).map(|_| {
            ChildNavigationSubmitError::RetryableReadiness(
                "MOTION_SINK_NOT_READY: test navigation chain is recovering".to_string(),
            )
        }));
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-readiness", "prefer_wavefront").await;

        for attempt in 0..10 {
            let waiting = handle.tick((2.0, 2.0)).await;
            assert_eq!(waiting.status, STATUS_PENDING, "attempt={attempt}");
            assert!(waiting
                .message
                .contains("waiting for child navigation readiness"));
            assert!(waiting.child_task_id.is_empty());
            if attempt == 0 {
                handle
                    .ingest_costmap_snapshot(frontier_costmap(2))
                    .await
                    .unwrap();
            }
        }

        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        {
            let submitted = sink.submitted.lock().unwrap();
            assert_eq!(submitted.len(), 11);
            assert!(submitted
                .iter()
                .all(|request| request.request_id == submitted[0].request_id));
            assert_eq!(submitted[0].map_revision, 1);
            assert!(submitted[1..]
                .iter()
                .all(|request| request.map_revision == 2));
        }

        let diagnostics = handle.diagnostics().await;
        assert!(diagnostics.sources[0].last_error.is_empty());
    }

    #[tokio::test]
    async fn permanent_dispatch_rejection_still_applies_goal_failure_policy() {
        let (handle, sink) = configured_handle();
        sink.submit_errors
            .lock()
            .unwrap()
            .push_back(ChildNavigationSubmitError::Rejected(
                "GOAL_BOUNDARY_INVALID: test permanent rejection".to_string(),
            ));
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-rejected", "prefer_wavefront").await;

        let rejected = handle.tick((2.0, 2.0)).await;
        assert_eq!(rejected.status, STATUS_PENDING);
        assert!(rejected
            .message
            .contains("child navigation dispatch failed"));
        assert_eq!(rejected.frontier_count, 0);

        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 2);
        assert_ne!(submitted[0].request_id, submitted[1].request_id);
    }

    #[tokio::test]
    async fn stop_retains_child_until_correlated_zeroed_proof() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-stop", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        let stopped = handle.stop("req-stop").await;
        assert_eq!(stopped.status, STATUS_STOPPING);
        assert_eq!(stopped.last_stop_reason, "user_cancelled");
        assert_eq!(stopped.child_task_id, active.child_task_id);
        assert_eq!(
            sink.cancelled.lock().unwrap().as_slice(),
            &[active.child_task_id.clone()]
        );

        let wrong = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 999)),
            )
            .await;
        assert!(wrong.is_err());
        assert_eq!(handle.snapshot().status, STATUS_STOPPING);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);

        let cancelled = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(cancelled.status, STATUS_CANCELLED);
        assert!(cancelled.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn stop_rejects_a_mismatched_parent_request_before_child_cancel() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-stop-owner", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;

        let rejected = handle.stop("req-stop-other").await;

        assert_eq!(rejected.status, STATUS_REJECTED);
        assert_eq!(rejected.request_id, "req-stop-other");
        assert!(sink.cancelled.lock().unwrap().is_empty());
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
        assert_eq!(handle.snapshot().request_id, "req-stop-owner");
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn rejected_start_does_not_attach_the_active_child_to_the_new_parent() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-active-owner", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        assert!(!active.child_task_id.is_empty());

        let rejected = handle.start("req-rejected-new", "prefer_wavefront").await;

        assert_eq!(rejected.status, STATUS_REJECTED);
        assert_eq!(rejected.request_id, "req-rejected-new");
        assert!(rejected.child_task_id.is_empty());
        let authoritative = handle.snapshot();
        assert_eq!(authoritative.status, STATUS_ACTIVE);
        assert_eq!(authoritative.request_id, "req-active-owner");
        assert_eq!(authoritative.child_task_id, active.child_task_id);
    }

    #[tokio::test]
    async fn zeroed_proof_requires_exact_lease_generation_ack_and_time() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-proof", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(handle.stop("req-proof").await.status, STATUS_STOPPING);

        let mut wrong_leases = Vec::new();
        let mut wrong = lease.clone();
        wrong.session_id = "other-child".to_string();
        wrong_leases.push(wrong);
        let mut wrong = lease.clone();
        wrong.owner_kind += 1;
        wrong_leases.push(wrong);
        let mut wrong = lease.clone();
        wrong.parent_request_id = "other-parent".to_string();
        wrong_leases.push(wrong);
        let mut wrong = lease.clone();
        wrong.goal_generation += 1;
        wrong_leases.push(wrong);
        for wrong in wrong_leases {
            assert!(handle
                .report_navigation_result(
                    &active.child_task_id,
                    ChildNavigationResult::Cancelled(zero_proof(&wrong, 1)),
                )
                .await
                .is_err());
        }

        let mut missing_ack = zero_proof(&lease, 1);
        missing_ack.zero_ack_id.clear();
        assert!(handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(missing_ack),
            )
            .await
            .is_err());
        let mut missing_time = zero_proof(&lease, 1);
        missing_time.zero_applied_monotonic_ns = 0;
        assert!(handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(missing_time),
            )
            .await
            .is_err());
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
    }

    #[tokio::test]
    async fn cancel_dispatch_failure_is_parent_failed_and_retains_safety_ownership() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-cancel-fail", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        *sink.cancel_error.lock().unwrap() = Some("guard cancel dispatch failed".to_string());

        let failed = handle.stop("req-cancel-fail").await;

        assert_eq!(failed.status, STATUS_FAILED);
        assert_eq!(failed.last_stop_reason, "child_cancel_failed");
        assert_eq!(failed.child_task_id, active.child_task_id);
        assert!(handle.is_active().await);
        assert!(handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn failed_cancel_without_ticket_retries_only_for_the_exact_parent_request() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-cancel-retry", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        *sink.cancel_error.lock().unwrap() = Some("guard unavailable".to_string());
        assert_eq!(handle.stop("req-cancel-retry").await.status, STATUS_FAILED);
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);

        let rejected = handle.stop("other-request").await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        assert_eq!(rejected.request_id, "other-request");
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
        assert_eq!(handle.snapshot().request_id, "req-cancel-retry");
        assert_eq!(handle.snapshot().status, STATUS_FAILED);

        *sink.cancel_error.lock().unwrap() = None;
        let retry = handle.stop("req-cancel-retry").await;
        assert_eq!(retry.status, STATUS_STOPPING);
        assert_eq!(retry.last_stop_reason, "child_cancel_failed");
        assert!(retry.message.contains("CANCEL_RETRY_PENDING"));
        assert_eq!(sink.cancelled.lock().unwrap().len(), 2);

        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(released.status, STATUS_FAILED);
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        assert!(released.child_task_id.is_empty());
        assert!(!handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn failed_cancel_with_ticket_retries_with_a_new_generation() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-timeout-retry", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(
            handle.stop("req-timeout-retry").await.status,
            STATUS_STOPPING
        );
        let failed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "CANCEL_ZERO_ACK_TIMEOUT".to_string(),
                },
            )
            .await
            .unwrap();
        assert_eq!(failed.status, STATUS_FAILED);

        let retry = handle.stop("req-timeout-retry").await;
        assert_eq!(retry.status, STATUS_STOPPING);
        assert!(retry.message.contains("cancel_generation=2"));
        assert_eq!(sink.cancelled.lock().unwrap().len(), 2);

        assert!(handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .is_err());
        assert_eq!(handle.snapshot().status, STATUS_STOPPING);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);

        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 2)),
            )
            .await
            .unwrap();
        assert_eq!(released.status, STATUS_FAILED);
        assert!(released.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn duplicate_old_cancel_failure_cannot_supersede_an_inflight_retry() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle
            .start("req-retry-old-failure", "prefer_wavefront")
            .await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(
            handle.stop("req-retry-old-failure").await.status,
            STATUS_STOPPING
        );
        handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "CANCEL_ZERO_ACK_TIMEOUT".to_string(),
                },
            )
            .await
            .unwrap();

        let release = Arc::new(tokio::sync::Notify::new());
        *sink.cancel_release.lock().unwrap() = Some(release.clone());
        let retry_task = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.stop("req-retry-old-failure").await })
        };
        sink.cancel_entered.notified().await;
        let duplicate = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "duplicate old timeout".to_string(),
                },
            )
            .await
            .unwrap();
        assert_eq!(duplicate.status, STATUS_STOPPING);
        assert!(duplicate.message.contains("OLD_FAILURE_IGNORED"));

        release.notify_one();
        let retry = retry_task.await.unwrap();
        assert!(retry.message.contains("cancel_generation=2"));
        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 2)),
            )
            .await
            .unwrap();
        assert_eq!(released.status, STATUS_FAILED);
        assert!(released.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn concurrent_repeated_stop_dispatches_only_one_cancel_retry() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-single-retry", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        *sink.cancel_error.lock().unwrap() = Some("guard unavailable".to_string());
        assert_eq!(handle.stop("req-single-retry").await.status, STATUS_FAILED);

        *sink.cancel_error.lock().unwrap() = None;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.cancel_release.lock().unwrap() = Some(release.clone());
        let retry_task = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.stop("req-single-retry").await })
        };
        sink.cancel_entered.notified().await;

        let concurrent = handle.stop("req-single-retry").await;
        assert_eq!(concurrent.status, STATUS_STOPPING);
        assert!(concurrent.message.contains("already in flight"));

        release.notify_one();
        let retry = retry_task.await.unwrap();
        assert_eq!(retry.status, STATUS_STOPPING);
        assert!(retry.message.contains("cancel_generation=1"));
        assert_eq!(sink.cancelled.lock().unwrap().len(), 2);

        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(released.status, STATUS_FAILED);
        assert!(released.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn zeroed_before_cancel_ticket_is_replayed_after_ticket_commit() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-early-zero", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.cancel_release.lock().unwrap() = Some(release.clone());

        let stop_task = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.stop("req-early-zero").await })
        };
        sink.cancel_entered.notified().await;

        let buffered = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .expect("exact early ZEROED must be retained");
        assert_eq!(buffered.status, STATUS_STOPPING);
        assert!(buffered.message.contains("ZEROED_EARLY_BUFFERED"));
        assert_eq!(buffered.child_task_id, active.child_task_id);

        release.notify_one();
        let terminal = stop_task.await.unwrap();
        assert_eq!(terminal.status, STATUS_CANCELLED);
        assert!(terminal.child_task_id.is_empty());
        assert!(!handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn cancel_zero_ack_timeout_is_parent_failed_and_keeps_child_identity() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-cancel-timeout", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(
            handle.stop("req-cancel-timeout").await.status,
            STATUS_STOPPING
        );

        let failed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "CANCEL_ZERO_ACK_TIMEOUT".to_string(),
                },
            )
            .await
            .unwrap();

        assert_eq!(failed.status, STATUS_FAILED);
        assert_eq!(failed.last_stop_reason, "child_cancel_failed");
        assert_eq!(failed.child_task_id, active.child_task_id);
        assert!(handle.is_active().await);
        assert!(handle.blocks_mapping_mutation().await);

        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();

        assert_eq!(released.status, STATUS_FAILED);
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        assert!(released.child_task_id.is_empty());
        assert!(!handle.is_active().await);
        assert!(!handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn explicit_revision_pin_does_not_stop_on_costmap_sequence_change() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle
            .start_with_config(
                "req-pinned",
                ExploreStartConfig {
                    map_revision: 1,
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    stop_conditions: ExploreStopConditions {
                        stop_on_map_revision_change: true,
                        ..ExploreStopConditions::default()
                    },
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;

        handle
            .ingest_costmap_snapshot(frontier_costmap(2))
            .await
            .unwrap();
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert_eq!(handle.stop("req-pinned").await.status, STATUS_STOPPING);
        let stopped = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert_eq!(stopped.last_stop_reason, "user_cancelled");
        assert!(stopped.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn child_request_carries_complete_map_boundary() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        let boundary = ExplorationBoundary::from_vertices(vec![
            nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
            nuwax_nav_types_messages::geometry::Point2 { x: 5.0, y: 0.0 },
            nuwax_nav_types_messages::geometry::Point2 { x: 5.0, y: 5.0 },
            nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 5.0 },
        ])
        .unwrap();
        handle
            .start_with_config(
                "req-boundary",
                ExploreStartConfig {
                    boundary: Some(boundary.clone()),
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    ..ExploreStartConfig::default()
                },
            )
            .await;

        let active = handle.tick((2.0, 2.0)).await;

        assert_eq!(active.status, STATUS_ACTIVE);
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(submitted[0].boundary.as_ref(), Some(&boundary));
        assert_eq!(
            submitted[0].motion_admission,
            crate::nav_runtime::test_motion_admission()
        );
    }

    #[tokio::test]
    async fn natural_no_reachable_completion_is_succeeded_not_cancelled() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(occupied_costmap(1))
            .await
            .unwrap();
        handle.start("req-natural", "prefer_wavefront").await;

        let completed = handle.tick((1.0, 1.0)).await;

        assert_eq!(completed.status, STATUS_SUCCEEDED);
        assert_eq!(completed.last_stop_reason, "no_reachable_free_space");
    }

    #[tokio::test]
    async fn pure_snapshot_read_does_not_drive_controller() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-read", "prefer_wavefront").await;
        let before = handle.snapshot();
        let after = handle.snapshot();
        assert_eq!(before.status, STATUS_ACCEPTED);
        assert_eq!(before.revision, after.revision);
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn scheduler_progresses_without_state_queries() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-scheduler", "prefer_wavefront").await;
        let scheduler =
            handle.spawn_scheduler(Arc::new(StaticPoseProvider), Duration::from_millis(10));

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if !sink.submitted.lock().unwrap().is_empty() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("scheduler should submit a goal without GET polling");

        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
        scheduler.shutdown().await;
    }

    #[tokio::test]
    async fn stale_pose_with_active_child_enters_cancel_pending_immediately() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-stale-active", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let scheduler = ExploreSchedulerHandle::spawn_with_pose_grace(
            handle.clone(),
            Arc::new(MissingPoseProvider),
            Duration::from_millis(5),
            Duration::from_secs(1),
        );

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.snapshot().status == STATUS_STOPPING {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("active child stale pose must request cancellation");
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
        scheduler.shutdown().await;
    }

    #[tokio::test]
    async fn stale_pose_without_child_has_bounded_grace_then_fails() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-stale-idle", "prefer_wavefront").await;
        let scheduler = ExploreSchedulerHandle::spawn_with_pose_grace(
            handle.clone(),
            Arc::new(MissingPoseProvider),
            Duration::from_millis(5),
            Duration::from_millis(25),
        );

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.snapshot().status == STATUS_FAILED {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("idle stale pose must fail after bounded grace");
        assert!(handle
            .snapshot()
            .message
            .contains("pose source unavailable"));
        assert!(!handle.is_active().await);
        assert!(!handle.blocks_mapping_mutation().await);
        scheduler.shutdown().await;
    }

    #[tokio::test]
    async fn readiness_requires_backend_costmap_controller_and_nav_sink() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-ready", "prefer_wavefront").await;
        let readiness = handle.readiness(true).await;
        assert!(readiness.ready, "{}", readiness.reason);
        assert!(readiness.navigation_sink_ready);
        assert_eq!(readiness.costmap_input_revision, 1);
    }

    #[tokio::test]
    async fn default_unpinned_session_keeps_active_child_across_revision_changes() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-map", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        handle
            .ingest_costmap_snapshot(frontier_costmap(2))
            .await
            .unwrap();
        let snapshot = handle.tick((2.0, 2.0)).await;
        assert_eq!(snapshot.status, STATUS_ACTIVE);
        assert_eq!(snapshot.map_revision, 2);
        assert_eq!(snapshot.child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());
        assert_eq!(active_child_lease(&handle).await.map_revision, 1);

        let completed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Reached(terminal_proof(&lease)),
            )
            .await
            .unwrap();
        assert_eq!(completed.status, STATUS_ACTIVE);
        assert!(completed.child_task_id.is_empty());
        assert!(handle.is_active().await);

        let next = handle.tick((2.0, 2.0)).await;
        assert_eq!(next.status, STATUS_ACTIVE);
        assert_ne!(next.child_task_id, active.child_task_id);
        assert_eq!(active_child_lease(&handle).await.map_revision, 2);
    }

    #[tokio::test]
    async fn max_goals_waits_for_active_child_terminal_before_stopping() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        let accepted = handle
            .start_with_config(
                "req-max-goals",
                ExploreStartConfig {
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    stop_conditions: ExploreStopConditions {
                        max_goals: Some(1),
                        ..ExploreStopConditions::default()
                    },
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        assert_eq!(accepted.status, STATUS_ACCEPTED);

        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);

        let waiting = handle.tick((2.0, 2.0)).await;
        assert_eq!(waiting.status, STATUS_ACTIVE);
        assert_eq!(waiting.child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());

        handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Reached(terminal_proof(&lease)),
            )
            .await
            .unwrap();
        let completed = handle.tick((2.0, 2.0)).await;
        assert_eq!(completed.status, STATUS_SUCCEEDED);
        assert_eq!(completed.last_stop_reason, "max_goals_reached");
        assert!(completed.child_task_id.is_empty());
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn max_goals_does_not_count_failed_child_as_reached() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        let accepted = handle
            .start_with_config(
                "req-max-goals-failed",
                ExploreStartConfig {
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    stop_conditions: ExploreStopConditions {
                        max_goals: Some(1),
                        ..ExploreStopConditions::default()
                    },
                    max_navigation_retries: 0,
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        assert_eq!(accepted.status, STATUS_ACCEPTED);

        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);

        let failed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Failed {
                    proof: terminal_proof(&lease),
                    reason: "planner rejected test goal".to_string(),
                },
            )
            .await
            .unwrap();
        assert_eq!(failed.status, STATUS_ACTIVE);
        assert!(failed.child_task_id.is_empty());

        let guard = handle.inner.lock().await;
        assert_eq!(guard.reached_goals, 0);
        assert_eq!(evaluate_stop_conditions(&guard, now_unix_nanos()), None);
        drop(guard);

        let next = handle.tick((2.0, 2.0)).await;
        assert_ne!(next.last_stop_reason, "max_goals_reached");
        assert_ne!(next.status, STATUS_SUCCEEDED);
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn revision_change_during_submit_keeps_dispatched_child_active() {
        let (handle, sink) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-submit-race", "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.submit_release.lock().unwrap() = Some(release.clone());
        let ticking = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.tick((2.0, 2.0)).await })
        };
        sink.submit_entered.notified().await;
        let ingesting = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.ingest_costmap_snapshot(frontier_costmap(2)).await })
        };
        tokio::task::yield_now().await;
        release.notify_one();

        let active = ticking.await.unwrap();
        ingesting.await.unwrap().unwrap();

        assert_eq!(active.status, STATUS_ACTIVE);
        let current = handle.tick((2.0, 2.0)).await;
        assert_eq!(current.status, STATUS_ACTIVE);
        assert_eq!(current.map_revision, 2);
        assert_eq!(current.child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());
        assert_eq!(active_child_lease(&handle).await.map_revision, 1);
    }

    #[tokio::test]
    async fn user_stop_after_revision_update_cancels_active_child() {
        let (handle, _) = configured_handle();
        handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap();
        handle.start("req-stop-replan", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        handle
            .ingest_costmap_snapshot(frontier_costmap(2))
            .await
            .unwrap();
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);

        let stopping = handle.stop("req-stop-replan").await;
        assert_eq!(stopping.status, STATUS_STOPPING);
        assert_eq!(stopping.last_stop_reason, "user_cancelled");
        let terminal = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(terminal.status, STATUS_CANCELLED);
        assert!(!handle.is_active().await);
    }

    #[test]
    fn wavefront_adapter_source_name_is_wavefront() {
        let adapter =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        assert_eq!(adapter.source_name(), Cow::Borrowed("wavefront"));
    }
}
