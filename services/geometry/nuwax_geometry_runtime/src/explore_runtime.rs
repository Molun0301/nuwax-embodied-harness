//! Exploration controller, child-navigation handoff, and background scheduling.
//!
//! `ExploreRuntimeHandle` owns the production wavefront session. State reads are
//! pure snapshots; controller progress happens only through `tick*` or a started
//! [`ExploreSchedulerHandle`]. Mapping and navigation wiring are injected through
//! typed interfaces so the module never silently discards a selected goal.

#![allow(dead_code)]

use std::collections::{HashMap, VecDeque};
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Write};
#[cfg(unix)]
use std::os::unix::fs::{MetadataExt, OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use futures::stream::{FuturesUnordered, StreamExt};
use nuwax_codec::lcm::schema;
use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
use nuwax_exploration::surface_adapter::{SurfaceGoalSourceAdapter, SurfaceInputReceiver};
use nuwax_exploration::wavefront_adapter::WavefrontGoalSourceAdapter;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationBoundary, ExplorationController,
    ExplorationGoalFeedback, ExplorationGoalSource, ExplorationPoll, ExplorationStateSnapshot,
    ExplorationStopReason, ExplorationTerminalKind,
};
use nuwax_explore_wavefront::WavefrontConfig;
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_stack::surface_config::SurfaceNavigationConfig;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{
    ClockKind, Point3, Pose3, PoseStamped, Quaternion, Timestamp,
};
use nuwax_nav_types_messages::goal::NavigationGoal;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_nav_types_messages::visual::FrontierGoal;
use parking_lot::{Mutex as SyncMutex, RwLock};
use serde::{Deserialize, Serialize};
use tokio::sync::{watch, Mutex};
use tokio::task::{AbortHandle, JoinHandle};

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
const DEFAULT_COSTMAP_RECEIPT_TTL: Duration = Duration::from_secs(2);
const DEFAULT_NO_FRONTIER_STABILITY: Duration = Duration::from_secs(5);
const DEFAULT_POSE_UNAVAILABLE_GRACE: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const SCHEDULER_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const SCHEDULER_SHUTDOWN_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const SCHEDULER_ABORT_JOIN_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(test)]
const SCHEDULER_ABORT_JOIN_TIMEOUT: Duration = Duration::from_millis(100);
#[cfg(not(test))]
const CHILD_CANCEL_DISPATCH_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const CHILD_CANCEL_DISPATCH_TIMEOUT: Duration = Duration::from_millis(100);
const ORPHAN_RECONCILE_BATCH_SIZE: usize = 32;
#[cfg(not(test))]
const ORPHAN_RECONCILE_PASS_TIMEOUT: Duration = Duration::from_millis(100);
#[cfg(test)]
const ORPHAN_RECONCILE_PASS_TIMEOUT: Duration = Duration::from_millis(50);
#[cfg(not(test))]
const ORPHAN_RECONCILE_ENTRY_TIMEOUT: Duration = Duration::from_millis(50);
#[cfg(test)]
const ORPHAN_RECONCILE_ENTRY_TIMEOUT: Duration = Duration::from_millis(10);

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
        _ => "unsupported_stop_reason",
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
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExploreStartConfig {
    pub strategy: String,
    pub arbitration: String,
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
            motion_admission: MotionAdmissionProof::default(),
            boundary: None,
            stop_conditions: ExploreStopConditions::default(),
            max_navigation_retries: 1,
            blacklist_radius_m: WavefrontConfig::default().safe_distance_m,
        }
    }
}

impl ExploreStartConfig {
    fn validate(&self, surface_mode: bool) -> Result<ExplorationArbitrationPolicy, String> {
        match self.strategy.trim().to_ascii_lowercase().as_str() {
            "" | "wavefront" | "frontier" => {}
            "surface" if surface_mode => {}
            "surface" => {
                return Err("surface exploration requires the mls-3d navigation profile".into())
            }
            "tare" | "native" | "exploration-native" => {
                return Err(format!(
                    "exploration source `{}` is unsupported/source unavailable in the qualified production profile",
                    self.strategy
                ));
            }
            other => return Err(format!("unknown exploration strategy: {other}")),
        }
        if surface_mode && self.stop_conditions.max_coverage_pct.is_some() {
            return Err(
                "3D exploration max_coverage_pct requires a known volume and is unsupported".into(),
            );
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
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExploreRobotPose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub orientation: Quaternion,
    pub frame_id: String,
    pub source_timestamp_ns: u64,
    pub received_monotonic_ns: u64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrontierSnapshot {
    pub frontiers: Vec<FrontierGoal>,
    pub revision: u64,
}

#[derive(Debug, Clone)]
pub struct ChildNavigationRequest {
    pub parent_request_id: String,
    pub request_id: String,
    pub expected_map_name: String,
    pub motion_admission: MotionAdmissionProof,
    pub frontier_goal_id: String,
    pub goal: NavigationGoal,
    pub boundary: Option<ExplorationBoundary>,
    pub intent: schema::GoalIntent,
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn build_wavefront_goal_intent(
    producer_epoch: u64,
    cancel_generation: u64,
    parent_request_id: &str,
    request_id: &str,
    motion_admission: &MotionAdmissionProof,
    goal: &NavigationGoal,
    boundary: Option<&ExplorationBoundary>,
    committed_map: schema::CommittedMapIdentity,
) -> Result<schema::GoalIntent, String> {
    if producer_epoch == 0 || producer_epoch > i64::MAX as u64 {
        return Err("wavefront producer epoch is outside the shared intent domain".to_string());
    }
    if cancel_generation > i64::MAX as u64 {
        return Err("wavefront cancel generation is outside the shared intent domain".to_string());
    }
    if !matches!(&goal.target.frame_id, NavFrameId::Map)
        && goal.target.frame_id.as_str() != committed_map.frame_id
    {
        return Err("wavefront goal is not in map-frame semantics".to_string());
    }
    motion_admission
        .validate()
        .map_err(|reason| format!("wavefront motion admission is invalid: {reason}"))?;
    let timestamp_ns = goal.target.stamp.nanos;
    if timestamp_ns <= 0 {
        return Err("wavefront goal timestamp must be positive".to_string());
    }
    let goal_pose = schema::PoseStamped {
        header: schema::Header {
            seq: 0,
            stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000)
                .map_err(|_| "wavefront goal timestamp seconds exceed i32".to_string())?,
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: committed_map.frame_id.clone(),
        },
        pose: schema::Pose {
            position: [
                goal.target.pose.position.x,
                goal.target.pose.position.y,
                goal.target.pose.position.z,
            ],
            orientation_xyzw: [
                goal.target.pose.orientation.x,
                goal.target.pose.orientation.y,
                goal.target.pose.orientation.z,
                goal.target.pose.orientation.w,
            ],
        },
    };
    let goal_sha256 =
        schema::goal_intent_goal_sha256(&goal_pose).map_err(|error| error.to_string())?;
    let (boundary_id, boundary_points, boundary_sha256) = match boundary {
        Some(boundary) => {
            let boundary_id = format!("{parent_request_id}:boundary");
            let points = boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y, 0.0])
                .collect::<Vec<_>>();
            let digest =
                schema::goal_intent_boundary_sha256(&committed_map.frame_id, &boundary_id, &points)
                    .map_err(|error| error.to_string())?;
            (boundary_id, points, digest)
        }
        None => (String::new(), Vec::new(), [0; 32]),
    };
    Ok(schema::GoalIntent {
        producer: "wavefront".to_string(),
        producer_epoch: producer_epoch as i64,
        priority: 100,
        session_id: request_id.to_string(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: parent_request_id.to_string(),
        goal_generation: 0,
        cancel_generation: cancel_generation as i64,
        committed_map,
        boundary_id,
        boundary_sha256,
        boundary_points,
        motion_guard_generation: motion_admission.guard_generation as i64,
        motion_guard_boot_nonce: motion_admission.guard_boot_nonce,
        motion_stop_generation: motion_admission.motion_stop_generation as i64,
        goal_sha256,
        goal: goal_pose,
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationLease {
    pub session_id: String,
    pub owner_kind: i32,
    pub parent_request_id: String,
    pub goal_generation: u64,
    pub producer_runtime_generation: u64,
    pub goal_payload_sha256: [u8; 32],
}

impl ChildNavigationLease {
    pub(crate) fn matches(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
            && self.producer_runtime_generation == other.producer_runtime_generation
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
    /// The exploration session's frozen navigation authority is no longer valid.
    SessionInvalidated(String),
}

impl ChildNavigationSubmitError {
    fn reason(&self) -> &str {
        match self {
            Self::RetryableReadiness(reason)
            | Self::Rejected(reason)
            | Self::SessionInvalidated(reason) => reason,
        }
    }

    fn is_retryable(&self) -> bool {
        matches!(self, Self::RetryableReadiness(_))
    }

    fn is_session_fatal(&self) -> bool {
        matches!(self, Self::SessionInvalidated(_))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationCancelTicket {
    pub lease: ChildNavigationLease,
    pub cancel_generation: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationCancelRecovery {
    pub ticket: ChildNavigationCancelTicket,
    pub failure: Option<String>,
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
    /// Snapshot the authoritative navigation cancel generation for a frozen intent.
    fn current_cancel_generation(&self) -> Result<u64, String> {
        Ok(0)
    }

    /// Capture the active map name when a child goal intent is formed.
    fn current_map_name(&self) -> Result<String, String> {
        Err("active map name unavailable".to_string())
    }

    async fn submit(
        &self,
        request: ChildNavigationRequest,
    ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError>;

    async fn cancel(
        &self,
        lease: &ChildNavigationLease,
    ) -> Result<ChildNavigationCancelTicket, String>;

    /// Recover an exact cancellation receipt when the async response was lost.
    /// A receipt identifies the transaction only; ownership remains held until
    /// a matching `ZEROED` proof is reported.
    fn recover_cancel_receipt(
        &self,
        _lease: &ChildNavigationLease,
    ) -> Option<ChildNavigationCancelRecovery> {
        None
    }

    /// Transfer an exact stale-success lease to a retained cancellation owner.
    fn retain_orphan_cancellation(&self, _lease: ChildNavigationLease) -> Result<(), String> {
        Err("retained orphan cancellation is unavailable".to_string())
    }

    /// 证明精确 orphan lease 已不再持有导航状态。
    async fn orphan_lease_is_terminal(&self, _lease: &ChildNavigationLease) -> bool {
        false
    }
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
    pub exploration_generation: u64,
    pub child_lease: Option<ChildNavigationLease>,
    pub rollover_recovery_eligible: bool,
    pub global_stop_reconciliation: ExplorationGlobalStopReconciliationSnapshot,
    pub ownership_reconciliation_receipt: Option<ExplorationOwnershipReconciliationReceipt>,
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
            exploration_generation: 0,
            child_lease: None,
            rollover_recovery_eligible: false,
            global_stop_reconciliation: ExplorationGlobalStopReconciliationSnapshot::default(),
            ownership_reconciliation_receipt: None,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ExplorationGlobalStopReconciliationSnapshot {
    pub eligible: bool,
    pub cancel_generation: u64,
    pub recovery_correlation: String,
    pub motion_admission: Option<MotionAdmissionProof>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ExplorationOwnershipReconciliationReceipt {
    pub cause: String,
    pub recovery_correlation: String,
    pub parent_request_id: String,
    pub exploration_generation: u64,
    pub child_lease_fingerprint: String,
    pub cancel_generation: u64,
    pub commit_disposition: String,
}

#[derive(Clone, PartialEq, Eq)]
pub struct RecoveryFenceToken(Vec<u8>);

impl RecoveryFenceToken {
    pub fn from_wire(value: Vec<u8>) -> Self {
        Self(value)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    fn is_valid(&self) -> bool {
        self.0.len() == 32 && self.0.iter().any(|byte| *byte != 0)
    }

    #[cfg(test)]
    fn clear(&mut self) {
        self.0.clear();
    }
}

impl fmt::Debug for RecoveryFenceToken {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("RecoveryFenceToken([REDACTED])")
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExplorationRecoveryGuardProof {
    pub stop_operation_id: String,
    pub recovery_correlation: String,
    pub motion_admission: MotionAdmissionProof,
    pub zero_ack_id: String,
    pub zero_applied_monotonic_ns: u64,
    pub recovery_fence_token: RecoveryFenceToken,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExplorationRolloverRecoveryRequest {
    pub request_id: String,
    pub exploration_generation: u64,
    pub child_lease: ChildNavigationLease,
    pub expected_navigation_runtime_generation: u64,
    pub observed_navigation_runtime_generation: u64,
    pub guard_proof: ExplorationRecoveryGuardProof,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExplorationGlobalStopGuardProof {
    pub stop_operation_id: String,
    pub recovery_correlation: String,
    pub original_motion_admission: MotionAdmissionProof,
    pub held_guard_generation: u64,
    pub held_guard_boot_nonce: [u8; 16],
    pub held_motion_stop_generation: u64,
    pub zero_ack_id: String,
    pub zero_applied_monotonic_ns: u64,
    pub recovery_fence_token: RecoveryFenceToken,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExplorationGlobalStopReconciliationRequest {
    pub request_id: String,
    pub exploration_generation: u64,
    pub child_lease: ChildNavigationLease,
    pub cancel_generation: u64,
    pub recovery_correlation: String,
    pub guard_proof: ExplorationGlobalStopGuardProof,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExplorationRecoveryError {
    InvalidArgument(String),
    FailedPrecondition(String),
}

impl ExplorationRecoveryError {
    pub fn message(&self) -> &str {
        match self {
            Self::InvalidArgument(message) | Self::FailedPrecondition(message) => message,
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

#[derive(Debug, Clone)]
struct CompletedRolloverRecovery {
    request: ExplorationRolloverRecoveryRequest,
    snapshot: ExploreRuntimeSnapshot,
}

#[derive(Debug, Clone)]
struct CompletedGlobalStopReconciliation {
    request: ExplorationGlobalStopReconciliationRequest,
    snapshot: ExploreRuntimeSnapshot,
}

#[derive(Debug, Clone)]
struct CompletedExactChildZeroed {
    recovery_correlation: String,
    request_fingerprint: String,
    receipt: ExplorationOwnershipReconciliationReceipt,
}

#[derive(Debug, Clone)]
enum CompletedOwnershipTransition {
    NavigationRollover(CompletedRolloverRecovery),
    GlobalStop(CompletedGlobalStopReconciliation),
    ExactChildZeroed(CompletedExactChildZeroed),
}

const GLOBAL_STOP_RECONCILIATION_CAUSE: &str = "ordinary_global_stop";
const GLOBAL_STOP_COMMIT_DISPOSITION: &str = "global_stop_reconciled_child_ownership";
const EXACT_CHILD_ZEROED_CAUSE: &str = "late_exact_child_zeroed";
const EXACT_CHILD_ZEROED_DISPOSITION: &str = "exact_child_zeroed_released_ownership";
const RECONCILIATION_JOURNAL_SCHEMA_VERSION: u32 = 1;
const RECONCILIATION_JOURNAL_CAPACITY: usize = 32;
const RECONCILIATION_JOURNAL_MAX_BYTES: u64 = 256 * 1024;
const RECONCILIATION_JOURNAL_MAX_ID_BYTES: usize = 512;
const RECONCILIATION_JOURNAL_MAX_MESSAGE_BYTES: usize = 4096;
const PRODUCTION_RECONCILIATION_JOURNAL_PATH: &str =
    "/var/lib/nuwax/geometry-runtime/exploration_ownership_journal.json";
const JOURNAL_TEMP_CREATE_ATTEMPTS: usize = 128;
static NEXT_JOURNAL_TEMP_ID: AtomicU64 = AtomicU64::new(0);
static RECONCILIATION_JOURNAL_IO_LOCK: OnceLock<SyncMutex<()>> = OnceLock::new();
static POISONED_RECONCILIATION_JOURNAL_PATHS: OnceLock<
    SyncMutex<std::collections::HashSet<PathBuf>>,
> = OnceLock::new();

#[cfg(test)]
static TEST_RECONCILIATION_JOURNAL_PATHS: OnceLock<
    SyncMutex<std::collections::HashMap<usize, PathBuf>>,
> = OnceLock::new();
#[cfg(test)]
static TEST_DIRECTORY_SYNC_FAILURES: OnceLock<
    SyncMutex<std::collections::HashMap<PathBuf, usize>>,
> = OnceLock::new();

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
enum DurableOwnershipTransitionKind {
    GlobalStop,
    ExactChildZeroed,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct DurableReconciliationSnapshot {
    status: String,
    message: String,
    request_id: String,
    selected_source: String,
    last_stop_reason: String,
    frontier_count: u32,
    revision: u64,
    exploration_generation: u64,
}

impl DurableReconciliationSnapshot {
    fn into_runtime_snapshot(
        self,
        receipt: ExplorationOwnershipReconciliationReceipt,
    ) -> ExploreRuntimeSnapshot {
        ExploreRuntimeSnapshot {
            status: STATUS_FAILED,
            message: self.message,
            request_id: self.request_id,
            selected_source: self.selected_source,
            last_stop_reason: self.last_stop_reason,
            frontier_count: self.frontier_count,
            revision: self.revision,
            child_task_id: String::new(),
            exploration_generation: self.exploration_generation,
            child_lease: None,
            rollover_recovery_eligible: false,
            global_stop_reconciliation: ExplorationGlobalStopReconciliationSnapshot::default(),
            ownership_reconciliation_receipt: Some(receipt),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct DurableOwnershipTransition {
    kind: DurableOwnershipTransitionKind,
    recovery_correlation: String,
    request_fingerprint: String,
    receipt: ExplorationOwnershipReconciliationReceipt,
    snapshot: DurableReconciliationSnapshot,
    committed_unix_ns: u64,
    acknowledged: bool,
}

impl DurableOwnershipTransition {
    fn runtime_snapshot(&self) -> ExploreRuntimeSnapshot {
        self.snapshot
            .clone()
            .into_runtime_snapshot(self.receipt.clone())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct DurableOwnershipJournal {
    schema_version: u32,
    entries: Vec<DurableOwnershipTransition>,
}

impl Default for DurableOwnershipJournal {
    fn default() -> Self {
        Self {
            schema_version: RECONCILIATION_JOURNAL_SCHEMA_VERSION,
            entries: Vec::new(),
        }
    }
}

enum DurableCommitOutcome {
    Committed,
    ExactReplay(Box<DurableOwnershipTransition>),
}

fn reconciliation_journal_path_from_env() -> PathBuf {
    std::env::var_os("NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH")
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            #[cfg(test)]
            {
                std::env::temp_dir()
                    .join(format!(
                        "nuwax-exploration-ownership-journal-{}",
                        std::process::id()
                    ))
                    .join("journal.json")
            }
            #[cfg(not(test))]
            PathBuf::from(PRODUCTION_RECONCILIATION_JOURNAL_PATH)
        })
}

fn is_bounded_string(value: &str, max_bytes: usize) -> bool {
    !value.trim().is_empty() && value.len() <= max_bytes
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value
            .as_bytes()
            .iter()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn validate_durable_ownership_transition(entry: &DurableOwnershipTransition) -> Result<(), String> {
    let receipt = &entry.receipt;
    let snapshot = &entry.snapshot;
    if !is_bounded_string(
        &entry.recovery_correlation,
        RECONCILIATION_JOURNAL_MAX_ID_BYTES,
    ) || !entry
        .recovery_correlation
        .strip_prefix("exploration-global-stop:")
        .is_some_and(is_sha256_hex)
        || !is_sha256_hex(&entry.request_fingerprint)
        || receipt.recovery_correlation != entry.recovery_correlation
        || !is_sha256_hex(&receipt.child_lease_fingerprint)
        || !is_bounded_string(
            &receipt.parent_request_id,
            RECONCILIATION_JOURNAL_MAX_ID_BYTES,
        )
        || receipt.exploration_generation == 0
        || receipt.cancel_generation == 0
        || snapshot.status != STATUS_FAILED
        || !is_bounded_string(&snapshot.message, RECONCILIATION_JOURNAL_MAX_MESSAGE_BYTES)
        || !is_bounded_string(&snapshot.request_id, RECONCILIATION_JOURNAL_MAX_ID_BYTES)
        || snapshot.selected_source.len() > RECONCILIATION_JOURNAL_MAX_ID_BYTES
        || !is_bounded_string(
            &snapshot.last_stop_reason,
            RECONCILIATION_JOURNAL_MAX_ID_BYTES,
        )
        || snapshot.request_id != receipt.parent_request_id
        || snapshot.exploration_generation != receipt.exploration_generation
        || snapshot.frontier_count != 0
        || entry.committed_unix_ns == 0
    {
        return Err("ownership journal contains an invalid transition".to_string());
    }
    let kind_matches = match entry.kind {
        DurableOwnershipTransitionKind::GlobalStop => {
            receipt.cause == GLOBAL_STOP_RECONCILIATION_CAUSE
                && receipt.commit_disposition == GLOBAL_STOP_COMMIT_DISPOSITION
                && snapshot.last_stop_reason == "global_stop_reconciled_safe"
        }
        DurableOwnershipTransitionKind::ExactChildZeroed => {
            receipt.cause == EXACT_CHILD_ZEROED_CAUSE
                && receipt.commit_disposition == EXACT_CHILD_ZEROED_DISPOSITION
                && snapshot.last_stop_reason == "child_cancel_failed"
        }
    };
    if !kind_matches {
        return Err("ownership journal transition kind is inconsistent".to_string());
    }
    Ok(())
}

fn journal_parent(path: &Path) -> Result<&Path, String> {
    path.parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .ok_or_else(|| "ownership journal path has no usable parent".to_string())
}

fn ensure_secure_journal_directory(parent: &Path) -> Result<(), String> {
    let metadata = match fs::symlink_metadata(parent) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            fs::create_dir_all(parent)
                .map_err(|error| format!("ownership journal directory create failed: {error}"))?;
            #[cfg(unix)]
            fs::set_permissions(parent, fs::Permissions::from_mode(0o700)).map_err(|error| {
                format!("ownership journal directory permission setup failed: {error}")
            })?;
            fs::symlink_metadata(parent)
                .map_err(|error| format!("ownership journal directory metadata failed: {error}"))?
        }
        Err(error) => {
            return Err(format!(
                "ownership journal directory metadata failed: {error}"
            ));
        }
    };
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err("ownership journal parent must be a real directory".to_string());
    }
    #[cfg(unix)]
    if metadata.mode() & 0o777 != 0o700 {
        return Err("ownership journal directory permissions must be 0700".to_string());
    }
    Ok(())
}

fn validate_existing_journal_file(path: &Path) -> Result<(), String> {
    match fs::symlink_metadata(path) {
        Ok(metadata) => {
            if !metadata.file_type().is_file() || metadata.file_type().is_symlink() {
                return Err("ownership journal must be a regular file".to_string());
            }
            #[cfg(unix)]
            if metadata.mode() & 0o777 != 0o600 {
                return Err("ownership journal permissions must be 0600".to_string());
            }
            Ok(())
        }
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("ownership journal metadata failed: {error}")),
    }
}

fn lock_durable_ownership_journal(path: &Path) -> Result<nix::fcntl::Flock<File>, String> {
    let parent = journal_parent(path)?;
    ensure_secure_journal_directory(parent)?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| "ownership journal path has no file name".to_string())?;
    let lock_path = parent.join(format!(".{file_name}.lock"));
    validate_existing_journal_file(&lock_path)?;
    let file = OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .mode(0o600)
        .custom_flags(nix::libc::O_NOFOLLOW)
        .open(&lock_path)
        .map_err(|error| format!("ownership journal lock open failed: {error}"))?;
    let metadata = file
        .metadata()
        .map_err(|error| format!("ownership journal lock metadata failed: {error}"))?;
    if !metadata.is_file() || metadata.mode() & 0o777 != 0o600 {
        return Err("ownership journal lock must be a 0600 regular file".to_string());
    }
    nix::fcntl::Flock::lock(file, nix::fcntl::FlockArg::LockExclusive)
        .map_err(|(_, error)| format!("ownership journal lock acquisition failed: {error}"))
}

fn sync_journal_directory(parent: &Path) -> io::Result<()> {
    #[cfg(test)]
    {
        let mut failures = TEST_DIRECTORY_SYNC_FAILURES
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock();
        if let Some(remaining) = failures.get_mut(parent) {
            if *remaining > 0 {
                *remaining -= 1;
                return Err(io::Error::other("injected journal directory sync failure"));
            }
        }
    }
    File::open(parent)?.sync_all()
}

fn poison_journal_path(path: &Path) {
    POISONED_RECONCILIATION_JOURNAL_PATHS
        .get_or_init(|| SyncMutex::new(std::collections::HashSet::new()))
        .lock()
        .insert(path.to_path_buf());
}

fn recover_poisoned_journal_path(path: &Path) -> Result<(), String> {
    let poisoned = POISONED_RECONCILIATION_JOURNAL_PATHS
        .get_or_init(|| SyncMutex::new(std::collections::HashSet::new()))
        .lock()
        .contains(path);
    if !poisoned {
        return Ok(());
    }
    sync_journal_directory(journal_parent(path)?).map_err(|error| {
        format!("ownership journal remains post-rename durability uncertain: {error}")
    })?;
    POISONED_RECONCILIATION_JOURNAL_PATHS
        .get_or_init(|| SyncMutex::new(std::collections::HashSet::new()))
        .lock()
        .remove(path);
    Ok(())
}

fn read_durable_ownership_journal(path: &Path) -> Result<DurableOwnershipJournal, String> {
    recover_poisoned_journal_path(path)?;
    let parent = journal_parent(path)?;
    if parent.exists() {
        ensure_secure_journal_directory(parent)?;
    }
    validate_existing_journal_file(path)?;
    if path.exists() {
        sync_journal_directory(parent).map_err(|error| {
            format!("ownership journal directory durability verification failed: {error}")
        })?;
    }
    let file = match OpenOptions::new()
        .read(true)
        .custom_flags(nix::libc::O_NOFOLLOW)
        .open(path)
    {
        Ok(file) => file,
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            return Ok(DurableOwnershipJournal::default());
        }
        Err(error) => return Err(format!("ownership journal read failed: {error}")),
    };
    let metadata = file
        .metadata()
        .map_err(|error| format!("ownership journal metadata failed: {error}"))?;
    if !metadata.is_file() || metadata.len() > RECONCILIATION_JOURNAL_MAX_BYTES {
        return Err("ownership journal is not a bounded regular file".to_string());
    }
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    file.take(RECONCILIATION_JOURNAL_MAX_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|error| format!("ownership journal read failed: {error}"))?;
    if bytes.len() as u64 > RECONCILIATION_JOURNAL_MAX_BYTES {
        return Err("ownership journal exceeds maximum byte size".to_string());
    }
    let journal: DurableOwnershipJournal = serde_json::from_slice(&bytes)
        .map_err(|error| format!("ownership journal JSON invalid: {error}"))?;
    if journal.schema_version != RECONCILIATION_JOURNAL_SCHEMA_VERSION {
        return Err(format!(
            "ownership journal schema {} is unsupported",
            journal.schema_version
        ));
    }
    if journal.entries.len() > RECONCILIATION_JOURNAL_CAPACITY {
        return Err("ownership journal exceeds bounded capacity".to_string());
    }
    let mut correlations = std::collections::HashSet::new();
    for entry in &journal.entries {
        validate_durable_ownership_transition(entry)?;
        if !correlations.insert(entry.recovery_correlation.as_str()) {
            return Err("ownership journal contains an invalid transition".to_string());
        }
    }
    Ok(journal)
}

fn persist_durable_ownership_journal(
    path: &Path,
    journal: &DurableOwnershipJournal,
) -> Result<(), String> {
    recover_poisoned_journal_path(path)?;
    let parent = journal_parent(path)?;
    ensure_secure_journal_directory(parent)?;
    validate_existing_journal_file(path)?;
    let bytes = serde_json::to_vec(journal)
        .map_err(|error| format!("ownership journal serialization failed: {error}"))?;
    if bytes.len() as u64 > RECONCILIATION_JOURNAL_MAX_BYTES {
        return Err("ownership journal exceeds maximum byte size".to_string());
    }
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| "ownership journal path has no file name".to_string())?;
    let mut selected = None;
    for _ in 0..JOURNAL_TEMP_CREATE_ATTEMPTS {
        let temp_path = parent.join(format!(
            ".{file_name}.tmp.{}.{}",
            std::process::id(),
            NEXT_JOURNAL_TEMP_ID.fetch_add(1, Ordering::Relaxed)
        ));
        match OpenOptions::new()
            .create_new(true)
            .write(true)
            .mode(0o600)
            .custom_flags(nix::libc::O_NOFOLLOW)
            .open(&temp_path)
        {
            Ok(file) => {
                selected = Some((temp_path, file));
                break;
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => {
                return Err(format!("ownership journal temp create failed: {error}"));
            }
        }
    }
    let (temp_path, mut file) =
        selected.ok_or_else(|| "ownership journal temp file attempts exhausted".to_string())?;
    let mut renamed = false;
    let result = (|| {
        file.write_all(&bytes)
            .and_then(|_| file.write_all(b"\n"))
            .and_then(|_| file.sync_all())
            .map_err(|error| format!("ownership journal temp sync failed: {error}"))?;
        fs::rename(&temp_path, path)
            .map_err(|error| format!("ownership journal atomic rename failed: {error}"))?;
        renamed = true;
        if let Err(error) = sync_journal_directory(parent) {
            poison_journal_path(path);
            return Err(format!(
                "ownership journal post-rename durability uncertain: {error}"
            ));
        }
        Ok(())
    })();
    if result.is_err() && !renamed {
        let _ = fs::remove_file(&temp_path);
    }
    result
}

fn commit_durable_ownership_transition(
    path: &Path,
    transition: DurableOwnershipTransition,
) -> Result<DurableCommitOutcome, String> {
    let _io_guard = RECONCILIATION_JOURNAL_IO_LOCK
        .get_or_init(|| SyncMutex::new(()))
        .lock();
    let _file_guard = lock_durable_ownership_journal(path)?;
    validate_durable_ownership_transition(&transition)?;
    let mut journal = read_durable_ownership_journal(path)?;
    if let Some(existing) = journal
        .entries
        .iter()
        .find(|entry| entry.recovery_correlation == transition.recovery_correlation)
    {
        if existing.kind == transition.kind
            && existing.request_fingerprint == transition.request_fingerprint
        {
            return Ok(DurableCommitOutcome::ExactReplay(Box::new(
                existing.clone(),
            )));
        }
        return Err("ownership transition already has a different first winner".to_string());
    }
    while journal.entries.len() >= RECONCILIATION_JOURNAL_CAPACITY {
        let Some(position) = journal.entries.iter().position(|entry| entry.acknowledged) else {
            return Err("ownership journal is full of unacknowledged transitions".to_string());
        };
        journal.entries.remove(position);
    }
    journal.entries.push(transition);
    persist_durable_ownership_journal(path, &journal)?;
    Ok(DurableCommitOutcome::Committed)
}

fn replay_durable_ownership_transition(
    path: &Path,
    kind: DurableOwnershipTransitionKind,
    recovery_correlation: &str,
    request_fingerprint: &str,
) -> Result<Option<DurableOwnershipTransition>, String> {
    let _io_guard = RECONCILIATION_JOURNAL_IO_LOCK
        .get_or_init(|| SyncMutex::new(()))
        .lock();
    let _file_guard = lock_durable_ownership_journal(path)?;
    let journal = read_durable_ownership_journal(path)?;
    let Some(existing) = journal
        .entries
        .iter()
        .find(|entry| entry.recovery_correlation == recovery_correlation)
    else {
        return Ok(None);
    };
    if existing.kind == kind && existing.request_fingerprint == request_fingerprint {
        return Ok(Some(existing.clone()));
    }
    Err("ownership transition replay proof does not match the durable first winner".to_string())
}

fn acknowledge_durable_ownership_transition(
    path: &Path,
    recovery_correlation: &str,
    child_lease_fingerprint: &str,
) -> Result<bool, String> {
    let _io_guard = RECONCILIATION_JOURNAL_IO_LOCK
        .get_or_init(|| SyncMutex::new(()))
        .lock();
    let _file_guard = lock_durable_ownership_journal(path)?;
    let mut journal = read_durable_ownership_journal(path)?;
    let Some(existing) = journal.entries.iter_mut().find(|entry| {
        entry.recovery_correlation == recovery_correlation
            && entry.receipt.child_lease_fingerprint == child_lease_fingerprint
    }) else {
        return Ok(false);
    };
    if existing.acknowledged {
        return Ok(true);
    }
    existing.acknowledged = true;
    persist_durable_ownership_journal(path, &journal)?;
    Ok(true)
}

fn append_canonical_field(target: &mut Vec<u8>, value: &[u8]) {
    target.extend_from_slice(&(value.len() as u64).to_be_bytes());
    target.extend_from_slice(value);
}

fn append_child_lease_identity(target: &mut Vec<u8>, lease: &ChildNavigationLease) {
    append_canonical_field(target, lease.session_id.as_bytes());
    target.extend_from_slice(&lease.owner_kind.to_be_bytes());
    append_canonical_field(target, lease.parent_request_id.as_bytes());
    target.extend_from_slice(&lease.goal_generation.to_be_bytes());
    target.extend_from_slice(&lease.producer_runtime_generation.to_be_bytes());
    target.extend_from_slice(&lease.goal_payload_sha256);
}

fn child_lease_fingerprint(lease: &ChildNavigationLease) -> String {
    let mut canonical = Vec::with_capacity(160);
    append_canonical_field(&mut canonical, b"exploration-child-lease-v1");
    append_child_lease_identity(&mut canonical, lease);
    crate::map_checksum::sha256_hex(&canonical)
}

fn global_stop_recovery_correlation(
    request_id: &str,
    exploration_generation: u64,
    child_lease: &ChildNavigationLease,
    cancel_generation: u64,
    motion_admission: &MotionAdmissionProof,
) -> String {
    let mut canonical = Vec::with_capacity(256);
    append_canonical_field(&mut canonical, b"exploration-global-stop-reconciliation-v1");
    append_canonical_field(&mut canonical, GLOBAL_STOP_RECONCILIATION_CAUSE.as_bytes());
    append_canonical_field(&mut canonical, request_id.as_bytes());
    canonical.extend_from_slice(&exploration_generation.to_be_bytes());
    append_child_lease_identity(&mut canonical, child_lease);
    canonical.extend_from_slice(&cancel_generation.to_be_bytes());
    canonical.extend_from_slice(&motion_admission.guard_generation.to_be_bytes());
    canonical.extend_from_slice(&motion_admission.guard_boot_nonce);
    canonical.extend_from_slice(&motion_admission.motion_stop_generation.to_be_bytes());
    format!(
        "exploration-global-stop:{}",
        crate::map_checksum::sha256_hex(&canonical)
    )
}

fn global_stop_request_fingerprint(request: &ExplorationGlobalStopReconciliationRequest) -> String {
    let proof = &request.guard_proof;
    let mut canonical = Vec::with_capacity(384);
    append_canonical_field(&mut canonical, b"exploration-global-stop-request-v1");
    append_canonical_field(&mut canonical, request.request_id.as_bytes());
    canonical.extend_from_slice(&request.exploration_generation.to_be_bytes());
    append_child_lease_identity(&mut canonical, &request.child_lease);
    canonical.extend_from_slice(&request.cancel_generation.to_be_bytes());
    append_canonical_field(&mut canonical, request.recovery_correlation.as_bytes());
    append_canonical_field(&mut canonical, proof.stop_operation_id.as_bytes());
    append_canonical_field(&mut canonical, proof.recovery_correlation.as_bytes());
    canonical.extend_from_slice(
        &proof
            .original_motion_admission
            .guard_generation
            .to_be_bytes(),
    );
    canonical.extend_from_slice(&proof.original_motion_admission.guard_boot_nonce);
    canonical.extend_from_slice(
        &proof
            .original_motion_admission
            .motion_stop_generation
            .to_be_bytes(),
    );
    canonical.extend_from_slice(&proof.held_guard_generation.to_be_bytes());
    canonical.extend_from_slice(&proof.held_guard_boot_nonce);
    canonical.extend_from_slice(&proof.held_motion_stop_generation.to_be_bytes());
    append_canonical_field(&mut canonical, proof.zero_ack_id.as_bytes());
    canonical.extend_from_slice(&proof.zero_applied_monotonic_ns.to_be_bytes());
    append_canonical_field(&mut canonical, proof.recovery_fence_token.as_slice());
    crate::map_checksum::sha256_hex(&canonical)
}

fn exact_child_zero_request_fingerprint(proof: &ChildNavigationZeroProof) -> String {
    let mut canonical = Vec::with_capacity(256);
    append_canonical_field(&mut canonical, b"exploration-exact-child-zero-v1");
    append_child_lease_identity(&mut canonical, &proof.lease);
    canonical.extend_from_slice(&proof.cancel_generation.to_be_bytes());
    append_canonical_field(&mut canonical, proof.zero_ack_id.as_bytes());
    canonical.extend_from_slice(&proof.zero_applied_monotonic_ns.to_be_bytes());
    crate::map_checksum::sha256_hex(&canonical)
}

fn durable_snapshot_from_runtime(
    snapshot: &ExploreRuntimeSnapshot,
) -> DurableReconciliationSnapshot {
    DurableReconciliationSnapshot {
        status: snapshot.status.to_string(),
        message: snapshot.message.clone(),
        request_id: snapshot.request_id.clone(),
        selected_source: snapshot.selected_source.clone(),
        last_stop_reason: snapshot.last_stop_reason.clone(),
        frontier_count: snapshot.frontier_count,
        revision: snapshot.revision,
        exploration_generation: snapshot.exploration_generation,
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SurfaceMapTicket {
    pub(crate) committed_map: schema::CommittedMapIdentity,
    producer_runtime_generation: i64,
    map_frame_generation: i64,
    cloud_revision: i64,
    commit_boot_nonce: [u8; 16],
    received_monotonic_ns: u64,
}

impl SurfaceMapTicket {
    fn same_domain(&self, other: &Self) -> bool {
        self.producer_runtime_generation == other.producer_runtime_generation
            && self.commit_boot_nonce == other.commit_boot_nonce
            && self.committed_map.session_id == other.committed_map.session_id
            && self.committed_map.session_generation == other.committed_map.session_generation
            && self.committed_map.frame_id == other.committed_map.frame_id
            && self.map_frame_generation == other.map_frame_generation
    }
}

struct SurfaceExplorationMode {
    config: Result<SurfaceNavigationConfig, String>,
    receiver: SurfaceInputReceiver,
    producer_runtime_generation: i64,
    commit_boot_nonce: [u8; 16],
    latest: Option<SurfaceMapTicket>,
    installed_cloud_revision: Option<i64>,
    minimum_eligible_cloud_revision: i64,
    input_ready: bool,
    input_error: Option<String>,
    accepting: bool,
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
    surface_mode: Option<SurfaceExplorationMode>,
    /// 最近一次有效地图输入的本地接收时刻，不参与地图身份。
    costmap_received_monotonic_ns: u64,
    committed_map: Option<schema::CommittedMapIdentity>,
    active_config: Option<ExploreStartConfig>,
    active_child: Option<ActiveChildNavigation>,
    pending_child_dispatch: Option<ChildNavigationRequest>,
    pending_child_cancellation: Option<PendingChildCancellation>,
    completed_rollover_recovery: Option<CompletedOwnershipTransition>,
    session_generation: u64,
    generation: u64,
    child_sequence: u64,
    reached_goals: u32,
    started_at_ns: u64,
    no_frontier_since: Option<Instant>,
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
            surface_mode: None,
            costmap_received_monotonic_ns: 0,
            committed_map: None,
            active_config: None,
            active_child: None,
            pending_child_dispatch: None,
            pending_child_cancellation: None,
            completed_rollover_recovery: None,
            session_generation: 0,
            generation: 0,
            child_sequence: 0,
            reached_goals: 0,
            started_at_ns: 0,
            no_frontier_since: None,
            frontiers: Vec::new(),
            frontier_revision: 0,
        }
    }

    pub fn add_source<S: ExplorationGoalSource + 'static>(&mut self, source: S) {
        self.controller.add_source(source);
    }

    fn session_in_progress(&self) -> bool {
        self.controller.is_active() || self.pending_child_cancellation.is_some()
    }

    fn update_frontiers(&mut self, frontiers: Vec<FrontierGoal>) -> Option<FrontierSnapshot> {
        if self.frontiers == frontiers {
            return None;
        }
        self.frontiers = frontiers;
        self.frontier_revision = self.frontier_revision.saturating_add(1);
        Some(FrontierSnapshot {
            frontiers: self.frontiers.clone(),
            revision: self.frontier_revision,
        })
    }

    fn snapshot(&self, status: &'static str, message: impl Into<String>) -> ExploreRuntimeSnapshot {
        let child_lease = self.active_child.as_ref().map(|child| child.lease.clone());
        let rollover_recovery_eligible = !self.controller.is_active()
            && child_lease.is_some()
            && self
                .pending_child_cancellation
                .as_ref()
                .is_some_and(|pending| pending.failure.is_some() && !pending.dispatch_in_flight);
        let global_stop_reconciliation = self
            .pending_child_cancellation
            .as_ref()
            .filter(|pending| pending.failure.is_some() && !pending.dispatch_in_flight)
            .and_then(|pending| pending.ticket.as_ref())
            .and_then(|ticket| {
                let child_lease = child_lease.as_ref()?;
                let motion_admission = self.active_config.as_ref()?.motion_admission;
                Some(ExplorationGlobalStopReconciliationSnapshot {
                    eligible: !self.controller.is_active()
                        && ticket.cancel_generation > 0
                        && ticket.lease.matches(child_lease),
                    cancel_generation: ticket.cancel_generation,
                    recovery_correlation: global_stop_recovery_correlation(
                        &self.last_request_id,
                        self.session_generation,
                        child_lease,
                        ticket.cancel_generation,
                        &motion_admission,
                    ),
                    motion_admission: Some(motion_admission),
                })
            })
            .unwrap_or_default();
        let ownership_reconciliation_receipt =
            self.completed_rollover_recovery
                .as_ref()
                .and_then(|completed| match completed {
                    CompletedOwnershipTransition::GlobalStop(completed) => {
                        completed.snapshot.ownership_reconciliation_receipt.clone()
                    }
                    CompletedOwnershipTransition::ExactChildZeroed(completed) => {
                        Some(completed.receipt.clone())
                    }
                    CompletedOwnershipTransition::NavigationRollover(_) => None,
                });
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
            exploration_generation: self.session_generation,
            child_lease,
            rollover_recovery_eligible,
            global_stop_reconciliation,
            ownership_reconciliation_receipt,
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

fn build_profile_controller(
    policy: ExplorationArbitrationPolicy,
    config: &ExploreStartConfig,
    surface: Option<&SurfaceExplorationMode>,
) -> Result<ExplorationController, String> {
    let Some(surface) = surface else {
        return Ok(build_controller(policy, config));
    };
    if !surface.accepting {
        return Err("surface exploration input owner is stopped".into());
    }
    let surface_config = surface.config.as_ref().map_err(Clone::clone)?;
    let mut adapter = SurfaceGoalSourceAdapter::new(
        "surface",
        surface_config.follower.body_to_support_z_m,
        surface.receiver.clone(),
    )
    .map_err(|error| error.to_string())?;
    adapter
        .configure_constraints(
            config.boundary.clone(),
            config.max_navigation_retries,
            config.blacklist_radius_m,
        )
        .map_err(|error| error.to_string())?;
    let mut controller = ExplorationController::new(policy);
    controller.add_source(adapter);
    Ok(controller)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OrphanCancellationPhase {
    OwnerRequired,
    OwnerRegistered,
    CancelPending,
    CancelPendingOwnerRegistered,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct ExactOrphanLeaseKey {
    session_id: String,
    owner_kind: i32,
    parent_request_id: String,
    goal_generation: u64,
    producer_runtime_generation: u64,
    goal_payload_sha256: [u8; 32],
}

impl From<&ChildNavigationLease> for ExactOrphanLeaseKey {
    fn from(lease: &ChildNavigationLease) -> Self {
        Self {
            session_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            producer_runtime_generation: lease.producer_runtime_generation,
            goal_payload_sha256: lease.goal_payload_sha256,
        }
    }
}

#[derive(Clone, Debug)]
struct OrphanCancellationEntry {
    lease: ChildNavigationLease,
    phase: OrphanCancellationPhase,
    queue_token: u64,
}

#[derive(Clone, Debug)]
struct OrphanCancellationQueueItem {
    key: ExactOrphanLeaseKey,
    token: u64,
}

#[derive(Default)]
struct OrphanCancellationState {
    entries: HashMap<ExactOrphanLeaseKey, OrphanCancellationEntry>,
    queue: VecDeque<OrphanCancellationQueueItem>,
    next_queue_token: u64,
}

#[derive(Default)]
pub(crate) struct OrphanCancellationRegistry {
    state: SyncMutex<OrphanCancellationState>,
}

impl OrphanCancellationRegistry {
    pub(crate) fn register(&self, lease: ChildNavigationLease) {
        let key = ExactOrphanLeaseKey::from(&lease);
        let mut state = self.state.lock();
        if state.entries.contains_key(&key) {
            return;
        }
        state.next_queue_token = state
            .next_queue_token
            .checked_add(1)
            .expect("orphan queue token exhausted");
        let token = state.next_queue_token;
        state.entries.insert(
            key.clone(),
            OrphanCancellationEntry {
                lease,
                phase: OrphanCancellationPhase::OwnerRequired,
                queue_token: token,
            },
        );
        state
            .queue
            .push_back(OrphanCancellationQueueItem { key, token });
    }

    pub(crate) fn mark_cancel_pending(&self, lease: &ChildNavigationLease) {
        let key = ExactOrphanLeaseKey::from(lease);
        if let Some(entry) = self.state.lock().entries.get_mut(&key) {
            entry.phase = match entry.phase {
                OrphanCancellationPhase::OwnerRequired | OrphanCancellationPhase::CancelPending => {
                    OrphanCancellationPhase::CancelPending
                }
                OrphanCancellationPhase::OwnerRegistered
                | OrphanCancellationPhase::CancelPendingOwnerRegistered => {
                    OrphanCancellationPhase::CancelPendingOwnerRegistered
                }
            };
        }
    }

    pub(crate) fn claim_owner(&self, lease: &ChildNavigationLease) -> bool {
        let key = ExactOrphanLeaseKey::from(lease);
        let mut state = self.state.lock();
        let Some(entry) = state.entries.get_mut(&key) else {
            return false;
        };
        entry.phase = match entry.phase {
            OrphanCancellationPhase::OwnerRequired => OrphanCancellationPhase::OwnerRegistered,
            OrphanCancellationPhase::CancelPending => {
                OrphanCancellationPhase::CancelPendingOwnerRegistered
            }
            OrphanCancellationPhase::OwnerRegistered
            | OrphanCancellationPhase::CancelPendingOwnerRegistered => return false,
        };
        true
    }

    pub(crate) fn release_owner(&self, lease: &ChildNavigationLease) {
        let key = ExactOrphanLeaseKey::from(lease);
        if let Some(entry) = self.state.lock().entries.get_mut(&key) {
            entry.phase = match entry.phase {
                OrphanCancellationPhase::OwnerRegistered => OrphanCancellationPhase::OwnerRequired,
                OrphanCancellationPhase::CancelPendingOwnerRegistered => {
                    OrphanCancellationPhase::CancelPending
                }
                phase => phase,
            };
        }
    }

    pub(crate) fn remove_terminal(&self, lease: &ChildNavigationLease) {
        let retired_state = {
            let mut state = self.state.lock();
            state.entries.remove(&ExactOrphanLeaseKey::from(lease));
            state
                .entries
                .is_empty()
                .then(|| std::mem::take(&mut *state))
        };
        if let Some(retired_state) = retired_state {
            let _cleanup = tokio::task::spawn_blocking(move || drop(retired_state));
        }
    }

    pub(crate) fn has_entries(&self) -> bool {
        !self.state.lock().entries.is_empty()
    }

    pub(crate) fn next_batch(&self, limit: usize) -> Vec<ChildNavigationLease> {
        let mut state = self.state.lock();
        let probe_limit = state.queue.len().min(limit);
        let mut batch = Vec::with_capacity(probe_limit);
        for _ in 0..probe_limit {
            let Some(item) = state.queue.pop_front() else {
                break;
            };
            let live = state
                .entries
                .get(&item.key)
                .filter(|entry| entry.queue_token == item.token)
                .map(|entry| entry.lease.clone());
            if let Some(lease) = live {
                state.queue.push_back(item);
                batch.push(lease);
            }
        }
        batch
    }

    pub(crate) fn blocks_motion_mutation(&self) -> bool {
        !self.state.lock().entries.is_empty()
    }
}

#[derive(Clone)]
pub struct ExploreRuntimeHandle {
    inner: Arc<Mutex<ExploreRuntimeInner>>,
    expected_map_frame: Arc<str>,
    readiness_cache: Arc<SyncMutex<ExplorationReadinessDetail>>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    navigation_sink: Arc<RwLock<Option<Arc<dyn ExploreNavigationSink>>>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    generation: Arc<AtomicU64>,
    orphan_cancellations: Arc<OrphanCancellationRegistry>,
}

impl ExploreRuntimeHandle {
    /// 初始化时捕获三维配置；配置错误只撤销探索能力，不关闭几何服务。
    pub(crate) async fn configure_surface_source(
        &self,
        mut config: Result<SurfaceNavigationConfig, String>,
        producer_runtime_generation: u64,
        commit_boot_nonce: [u8; 16],
    ) -> Result<(), String> {
        if producer_runtime_generation == 0
            || producer_runtime_generation > i64::MAX as u64
            || commit_boot_nonce == [0; 16]
        {
            config = Err("surface exploration producer identity is unavailable".into());
        }
        if config
            .as_ref()
            .is_ok_and(|config| config.geometry.world_frame != self.expected_map_frame.as_ref())
        {
            config = Err(
                "surface navigation configuration frame differs from Geometry map frame".into(),
            );
        }
        let mut guard = self.inner.lock().await;
        if guard.session_in_progress() {
            return Err("cannot reconfigure an active exploration source".into());
        }
        let mode = SurfaceExplorationMode {
            config,
            receiver: SurfaceInputReceiver::new(),
            producer_runtime_generation: producer_runtime_generation as i64,
            commit_boot_nonce,
            latest: None,
            installed_cloud_revision: None,
            minimum_eligible_cloud_revision: 0,
            input_ready: false,
            input_error: Some("surface map input is unavailable".into()),
            accepting: true,
        };
        guard.controller = build_profile_controller(
            guard.arbitration,
            &ExploreStartConfig::default(),
            Some(&mode),
        )
        .unwrap_or_else(|_| ExplorationController::new(guard.arbitration));
        guard.surface_mode = Some(mode);
        guard.committed_map = None;
        guard.costmap_received_monotonic_ns = 0;
        drop(guard);
        let backend = self.readiness_cache.lock().backend_available;
        self.refresh_readiness_cache_with_backend(backend);
        Ok(())
    }

    /// 接收新消息身份并立即处理清空/域变化；表面计算由外部受管工作执行。
    pub(crate) async fn begin_surface_map_update(
        &self,
        message: &schema::NavigationMap3D,
        received_monotonic_ns: u64,
    ) -> Result<Option<SurfaceMapTicket>, String> {
        schema::validate_navigation_map_3d(message)?;
        if received_monotonic_ns == 0
            || message.committed_map.frame_id != self.expected_map_frame.as_ref()
        {
            return Err("surface map receipt time or frame is invalid".into());
        }
        let ticket = SurfaceMapTicket {
            committed_map: message.committed_map.clone(),
            producer_runtime_generation: message.producer_runtime_generation,
            map_frame_generation: message.map_frame_generation,
            cloud_revision: message.cloud_revision,
            commit_boot_nonce: message.commit_boot_nonce,
            received_monotonic_ns,
        };
        let mut guard = self.inner.lock().await;
        let invalidates = {
            let mode = guard
                .surface_mode
                .as_mut()
                .ok_or_else(|| "surface exploration is disabled".to_string())?;
            if !mode.accepting {
                return Err("surface exploration input owner is stopped".into());
            }
            mode.config.as_ref().map_err(Clone::clone)?;
            if ticket.producer_runtime_generation != mode.producer_runtime_generation
                || ticket.commit_boot_nonce != mode.commit_boot_nonce
            {
                return Err("surface map producer does not belong to this Geometry runtime".into());
            }
            if mode
                .latest
                .as_ref()
                .is_some_and(|previous| ticket.cloud_revision <= previous.cloud_revision)
            {
                return Err("surface cloud revision is duplicate or stale".into());
            }
            let invalidates = message.cleared
                || mode
                    .latest
                    .as_ref()
                    .is_some_and(|previous| !ticket.same_domain(previous));
            mode.latest = Some(ticket.clone());
            if invalidates {
                mode.installed_cloud_revision = None;
                mode.minimum_eligible_cloud_revision =
                    message.cloud_revision + i64::from(message.cleared);
                mode.receiver.clear().map_err(|error| error.to_string())?;
                mode.input_ready = false;
                mode.input_error = Some(
                    if message.cleared {
                        "surface map cleared"
                    } else {
                        "surface map domain changed"
                    }
                    .into(),
                );
            }
            invalidates
        };
        let finish_request = if invalidates {
            guard.committed_map = None;
            guard.costmap_received_monotonic_ns = 0;
            guard.no_frontier_since = None;
            guard
                .controller
                .is_active()
                .then(|| guard.last_request_id.clone())
        } else {
            None
        };
        drop(guard);
        if let Some(request_id) = finish_request {
            self.finish_session(
                Some(&request_id),
                ExplorationStopReason::MapRevisionChanged,
                if message.cleared {
                    "surface map cleared by mapping owner"
                } else {
                    "surface map domain changed"
                }
                .into(),
            )
            .await;
        }
        let backend = self.readiness_cache.lock().backend_available;
        self.refresh_readiness_cache_with_backend(backend);
        Ok((!message.cleared).then_some(ticket))
    }

    /// 同域允许最新完成图先安装；票据保留自身身份和接收时刻，清空/换域仍撤销旧结果。
    pub(crate) async fn finish_surface_map_update(
        &self,
        ticket: SurfaceMapTicket,
        result: Result<Arc<SurfaceSnapshot>, String>,
    ) -> bool {
        let mut guard = self.inner.lock().await;
        let error = {
            let Some(mode) = guard.surface_mode.as_mut() else {
                return false;
            };
            if !mode.accepting
                || mode.latest.as_ref().is_none_or(|latest| {
                    !ticket.same_domain(latest) || ticket.cloud_revision > latest.cloud_revision
                })
                || ticket.cloud_revision < mode.minimum_eligible_cloud_revision
                || mode
                    .installed_cloud_revision
                    .is_some_and(|revision| ticket.cloud_revision <= revision)
            {
                return false;
            }
            match result {
                Ok(snapshot) => match mode.receiver.replace(Some(snapshot)) {
                    Ok(()) => {
                        mode.installed_cloud_revision = Some(ticket.cloud_revision);
                        mode.input_ready = true;
                        mode.input_error = None;
                        None
                    }
                    Err(error) => Some(error.to_string()),
                },
                Err(error) => Some(error),
            }
        };
        let finish_request = if let Some(error) = error.as_ref() {
            let mode = guard
                .surface_mode
                .as_mut()
                .expect("surface mode was checked");
            let _ = mode.receiver.clear();
            mode.input_ready = false;
            mode.input_error = Some(error.clone());
            guard.committed_map = None;
            guard.costmap_received_monotonic_ns = 0;
            guard.no_frontier_since = None;
            guard
                .controller
                .is_active()
                .then(|| guard.last_request_id.clone())
        } else {
            guard.committed_map = Some(ticket.committed_map);
            guard.costmap_received_monotonic_ns = ticket.received_monotonic_ns;
            None
        };
        drop(guard);
        if let Some(request_id) = finish_request {
            self.finish_session(
                Some(&request_id),
                ExplorationStopReason::ConsecutiveFailuresExceeded,
                format!(
                    "surface map computation failed: {}",
                    error.unwrap_or_default()
                ),
            )
            .await;
        }
        let backend = self.readiness_cache.lock().backend_available;
        self.refresh_readiness_cache_with_backend(backend);
        true
    }

    pub(crate) async fn stop_surface_input_owner(&self) {
        let mut guard = self.inner.lock().await;
        if let Some(mode) = guard.surface_mode.as_mut() {
            mode.accepting = false;
            mode.latest = None;
            mode.installed_cloud_revision = None;
            mode.input_ready = false;
            mode.input_error = Some("surface input owner stopped".into());
            let _ = mode.receiver.clear();
            guard.committed_map = None;
            guard.costmap_received_monotonic_ns = 0;
            guard.no_frontier_since = None;
        }
        drop(guard);
        let backend = self.readiness_cache.lock().backend_available;
        self.refresh_readiness_cache_with_backend(backend);
    }

    pub fn default_with_wavefront() -> Self {
        Self::new_with_validated_map_frame(Arc::from("map"))
    }

    pub fn with_wavefront_map_frame(expected_map_frame: impl Into<String>) -> Result<Self, String> {
        let expected_map_frame = validate_expected_map_frame(expected_map_frame.into())?;
        Ok(Self::new_with_validated_map_frame(expected_map_frame))
    }

    fn new_with_validated_map_frame(expected_map_frame: Arc<str>) -> Self {
        let (frontier_tx, _) = watch::channel(FrontierSnapshot::default());
        Self {
            inner: Arc::new(Mutex::new(ExploreRuntimeInner::new())),
            expected_map_frame,
            readiness_cache: Arc::new(SyncMutex::new(ExplorationReadinessDetail {
                ready: false,
                reason: "exploration controller not started".to_string(),
                backend_available: false,
                controller_active: false,
                source_count: 1,
                frontier_source_ready: false,
                costmap_input_ready: false,
                navigation_sink_ready: false,
                active_source: "wavefront".to_string(),
                arbitration: POLICY_PREFER_WAVEFRONT.to_string(),
            })),
            snapshot_cache: Arc::new(RwLock::new(ExploreRuntimeSnapshot::default())),
            navigation_sink: Arc::new(RwLock::new(None)),
            frontier_tx,
            generation: Arc::new(AtomicU64::new(0)),
            orphan_cancellations: Arc::new(OrphanCancellationRegistry::default()),
        }
    }

    pub fn expected_map_frame(&self) -> &str {
        self.expected_map_frame.as_ref()
    }

    pub(crate) fn producer_epoch_projection(&self) -> Arc<AtomicU64> {
        self.generation.clone()
    }

    pub(crate) fn orphan_cancellation_registry(&self) -> Arc<OrphanCancellationRegistry> {
        self.orphan_cancellations.clone()
    }

    async fn reconcile_orphan_cancellation(
        &self,
        sink: &Arc<dyn ExploreNavigationSink>,
        lease: ChildNavigationLease,
    ) {
        if sink.orphan_lease_is_terminal(&lease).await {
            self.orphan_cancellations.remove_terminal(&lease);
            return;
        }
        let recovered = sink
            .recover_cancel_receipt(&lease)
            .is_some_and(|recovery| recovery.failure.is_none());
        if recovered || sink.cancel(&lease).await.is_ok() {
            self.orphan_cancellations.mark_cancel_pending(&lease);
        }
        let _ = sink.retain_orphan_cancellation(lease.clone());
        if sink.orphan_lease_is_terminal(&lease).await {
            self.orphan_cancellations.remove_terminal(&lease);
        }
    }

    pub(crate) async fn reconcile_orphan_cancellations(&self) -> bool {
        let leases = self
            .orphan_cancellations
            .next_batch(ORPHAN_RECONCILE_BATCH_SIZE);
        let Some(sink) = self.navigation_sink.read().clone() else {
            return leases.is_empty();
        };
        let mut pending = FuturesUnordered::new();
        for lease in leases {
            pending.push(tokio::time::timeout(
                ORPHAN_RECONCILE_ENTRY_TIMEOUT,
                self.reconcile_orphan_cancellation(&sink, lease),
            ));
        }
        let _ = tokio::time::timeout(ORPHAN_RECONCILE_PASS_TIMEOUT, async {
            while pending.next().await.is_some() {}
        })
        .await;
        !self.orphan_cancellations.blocks_motion_mutation()
    }

    pub fn set_navigation_sink(&self, sink: Arc<dyn ExploreNavigationSink>) {
        *self.navigation_sink.write() = Some(sink);
    }

    #[cfg(test)]
    pub(crate) fn navigation_sink_for_tests(&self) -> Arc<dyn ExploreNavigationSink> {
        self.navigation_sink
            .read()
            .clone()
            .expect("production exploration navigation sink")
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

    fn reconciliation_journal_path(&self) -> PathBuf {
        #[cfg(test)]
        if let Some(path) = TEST_RECONCILIATION_JOURNAL_PATHS
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock()
            .get(&(Arc::as_ptr(&self.inner) as usize))
            .cloned()
        {
            return path;
        }
        reconciliation_journal_path_from_env()
    }

    #[cfg(test)]
    pub(crate) fn set_reconciliation_journal_path_for_tests(&self, path: PathBuf) {
        TEST_RECONCILIATION_JOURNAL_PATHS
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock()
            .insert(Arc::as_ptr(&self.inner) as usize, path);
    }

    pub fn latest_durable_reconciliation_snapshot(
        &self,
    ) -> Result<Option<ExploreRuntimeSnapshot>, ExplorationRecoveryError> {
        let path = self.reconciliation_journal_path();
        let _io_guard = RECONCILIATION_JOURNAL_IO_LOCK
            .get_or_init(|| SyncMutex::new(()))
            .lock();
        let _file_guard = lock_durable_ownership_journal(&path)
            .map_err(ExplorationRecoveryError::FailedPrecondition)?;
        let journal = read_durable_ownership_journal(&path)
            .map_err(ExplorationRecoveryError::FailedPrecondition)?;
        Ok(journal
            .entries
            .iter()
            .rev()
            .find(|entry| !entry.acknowledged)
            .map(DurableOwnershipTransition::runtime_snapshot))
    }

    pub fn restore_latest_durable_reconciliation_snapshot(
        &self,
    ) -> Result<(), ExplorationRecoveryError> {
        if let Some(snapshot) = self.latest_durable_reconciliation_snapshot()? {
            self.store_snapshot(snapshot);
        }
        Ok(())
    }

    pub fn acknowledge_durable_reconciliation(
        &self,
        recovery_correlation: &str,
        child_lease_fingerprint: &str,
    ) -> Result<bool, ExplorationRecoveryError> {
        if recovery_correlation.trim().is_empty() || child_lease_fingerprint.trim().is_empty() {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "reconciliation acknowledgement identity is incomplete".to_string(),
            ));
        }
        acknowledge_durable_ownership_transition(
            &self.reconciliation_journal_path(),
            recovery_correlation,
            child_lease_fingerprint,
        )
        .map_err(ExplorationRecoveryError::FailedPrecondition)
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
        config: ExploreStartConfig,
    ) -> ExploreRuntimeSnapshot {
        let surface_mode = {
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
                    exploration_generation: guard.session_generation,
                    child_lease: guard.active_child.as_ref().map(|child| child.lease.clone()),
                    rollover_recovery_eligible: false,
                    global_stop_reconciliation:
                        ExplorationGlobalStopReconciliationSnapshot::default(),
                    ownership_reconciliation_receipt: None,
                };
            }
            guard.surface_mode.is_some()
        };
        let policy = match config.validate(surface_mode) {
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
                exploration_generation: guard.session_generation,
                child_lease: guard.active_child.as_ref().map(|child| child.lease.clone()),
                rollover_recovery_eligible: false,
                global_stop_reconciliation: ExplorationGlobalStopReconciliationSnapshot::default(),
                ownership_reconciliation_receipt: None,
            };
            drop(guard);
            return snapshot;
        }

        let mut controller =
            match build_profile_controller(policy, &config, guard.surface_mode.as_ref()) {
                Ok(controller) => controller,
                Err(error) => {
                    let mut snapshot = guard.snapshot(STATUS_REJECTED, error);
                    snapshot.request_id = request_id.to_string();
                    if surface_mode {
                        snapshot.selected_source = "surface".into();
                    }
                    drop(guard);
                    return self.store_snapshot(snapshot);
                }
            };
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
        if surface_mode {
            guard.last_selected_source = "surface".into();
        }
        guard.last_stop_reason.clear();
        guard.last_stop = None;
        guard.last_frontier_goal = None;
        guard.last_poll = None;
        guard.active_config = Some(config.clone());
        guard.active_child = None;
        guard.pending_child_dispatch = None;
        guard.pending_child_cancellation = None;
        guard.completed_rollover_recovery = None;
        guard.session_generation = generation;
        guard.generation = generation;
        guard.child_sequence = 0;
        guard.reached_goals = 0;
        guard.started_at_ns = now_unix_nanos();
        guard.no_frontier_since = None;
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_ACCEPTED,
            format!(
                "exploration accepted (source={}, arbitration={})",
                if surface_mode { "surface" } else { "wavefront" },
                arbitration_policy_str(policy)
            ),
        );
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            self.expected_map_frame.as_ref(),
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

    fn validate_rollover_recovery_request(
        request: &ExplorationRolloverRecoveryRequest,
    ) -> Result<(), ExplorationRecoveryError> {
        let request_id = request.request_id.trim();
        if request_id.is_empty()
            || request.exploration_generation == 0
            || request.child_lease.session_id.trim().is_empty()
            || request.child_lease.parent_request_id.trim() != request_id
            || request.child_lease.goal_generation == 0
            || request.child_lease.producer_runtime_generation == 0
            || request.child_lease.goal_payload_sha256 == [0; 32]
            || request.expected_navigation_runtime_generation == 0
            || request.observed_navigation_runtime_generation
                <= request.expected_navigation_runtime_generation
        {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "incomplete exploration rollover recovery identity".to_string(),
            ));
        }
        if request.guard_proof.stop_operation_id.trim().is_empty()
            || request.guard_proof.recovery_correlation.trim().is_empty()
            || request.guard_proof.motion_admission.validate().is_err()
            || request.guard_proof.motion_admission.motion_stop_generation == 0
            || request.guard_proof.zero_ack_id.trim().is_empty()
            || request.guard_proof.zero_applied_monotonic_ns == 0
            || !request.guard_proof.recovery_fence_token.is_valid()
        {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "incomplete motion guard recovery fence proof".to_string(),
            ));
        }

        Ok(())
    }

    pub async fn replay_completed_rollover_recovery(
        &self,
        request: &ExplorationRolloverRecoveryRequest,
    ) -> Result<Option<ExploreRuntimeSnapshot>, ExplorationRecoveryError> {
        Self::validate_rollover_recovery_request(request)?;

        let guard = self.inner.lock().await;
        let Some(completed) = guard.completed_rollover_recovery.as_ref() else {
            return Ok(None);
        };
        if let CompletedOwnershipTransition::NavigationRollover(completed) = completed {
            if completed.request == *request {
                return Ok(Some(completed.snapshot.clone()));
            }
        }
        Err(ExplorationRecoveryError::FailedPrecondition(
            "rollover recovery proof does not match the completed transition".to_string(),
        ))
    }

    pub async fn recover_after_navigation_rollover(
        &self,
        request: ExplorationRolloverRecoveryRequest,
    ) -> Result<ExploreRuntimeSnapshot, ExplorationRecoveryError> {
        Self::validate_rollover_recovery_request(&request)?;
        let request_id = request.request_id.trim();

        let mut guard = self.inner.lock().await;
        if let Some(completed) = guard.completed_rollover_recovery.as_ref() {
            if let CompletedOwnershipTransition::NavigationRollover(completed) = completed {
                if completed.request == request {
                    return Ok(completed.snapshot.clone());
                }
            }
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "rollover recovery proof does not match the completed transition".to_string(),
            ));
        }
        if guard.last_request_id != request_id {
            return Err(ExplorationRecoveryError::FailedPrecondition(format!(
                "exploration recovery request_id mismatch: active={} received={}",
                guard.last_request_id, request_id
            )));
        }
        if guard.session_generation != request.exploration_generation {
            return Err(ExplorationRecoveryError::FailedPrecondition(format!(
                "exploration recovery generation mismatch: active={} received={}",
                guard.session_generation, request.exploration_generation
            )));
        }
        let active_child = guard.active_child.as_ref().ok_or_else(|| {
            ExplorationRecoveryError::FailedPrecondition(
                "exploration recovery has no retained child ownership".to_string(),
            )
        })?;
        if !active_child.lease.matches(&request.child_lease) {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration recovery child lease does not match retained ownership".to_string(),
            ));
        }
        let cancel_failed = guard
            .pending_child_cancellation
            .as_ref()
            .is_some_and(|pending| pending.failure.is_some() && !pending.dispatch_in_flight);
        if guard.controller.is_active() || !cancel_failed {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration recovery requires a stopped controller and failed child cancellation"
                    .to_string(),
            ));
        }

        let generation = self.generation.fetch_add(1, Ordering::SeqCst) + 1;
        guard.generation = generation;
        guard.pending_child_dispatch = None;
        guard.pending_child_cancellation = None;
        guard.active_child = None;
        guard.no_frontier_since = None;
        guard.last_stop = Some(ExplorationStopReason::ConsecutiveFailuresExceeded);
        guard.last_stop_reason = "runtime_rollover_aborted_safe".to_string();
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_FAILED,
            "RUNTIME_ROLLOVER_ABORTED_SAFE: global stop and durable guard fence released orphaned child ownership",
        );
        guard.completed_rollover_recovery = Some(CompletedOwnershipTransition::NavigationRollover(
            CompletedRolloverRecovery {
                request,
                snapshot: snapshot.clone(),
            },
        ));
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            self.expected_map_frame.as_ref(),
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontier_update);
        Ok(self.store_snapshot(snapshot))
    }

    fn validate_global_stop_reconciliation_request(
        request: &ExplorationGlobalStopReconciliationRequest,
    ) -> Result<(), ExplorationRecoveryError> {
        let request_id = request.request_id.trim();
        if request_id.is_empty()
            || request.exploration_generation == 0
            || request.child_lease.session_id.trim().is_empty()
            || request.child_lease.parent_request_id.trim() != request_id
            || request.child_lease.goal_generation == 0
            || request.child_lease.producer_runtime_generation == 0
            || request.child_lease.goal_payload_sha256 == [0; 32]
            || request.cancel_generation == 0
            || request.recovery_correlation.trim().is_empty()
            || request.guard_proof.recovery_correlation != request.recovery_correlation
        {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "incomplete exploration global-stop reconciliation identity".to_string(),
            ));
        }
        let proof = &request.guard_proof;
        if proof.stop_operation_id.trim().is_empty()
            || proof.original_motion_admission.validate().is_err()
            || proof.held_guard_generation == 0
            || proof.held_guard_boot_nonce.iter().all(|byte| *byte == 0)
            || proof.held_motion_stop_generation == 0
            || proof.zero_ack_id.trim().is_empty()
            || proof.zero_applied_monotonic_ns == 0
            || !proof.recovery_fence_token.is_valid()
        {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "incomplete held motion guard recovery fence proof".to_string(),
            ));
        }
        if proof.held_guard_generation != proof.original_motion_admission.guard_generation
            || proof.held_guard_boot_nonce != proof.original_motion_admission.guard_boot_nonce
            || proof.held_motion_stop_generation
                <= proof.original_motion_admission.motion_stop_generation
        {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "held guard proof does not strictly advance the original motion admission epoch"
                    .to_string(),
            ));
        }
        Ok(())
    }

    pub async fn replay_completed_global_stop_reconciliation(
        &self,
        request: &ExplorationGlobalStopReconciliationRequest,
    ) -> Result<Option<ExploreRuntimeSnapshot>, ExplorationRecoveryError> {
        Self::validate_global_stop_reconciliation_request(request)?;
        let request_fingerprint = global_stop_request_fingerprint(request);
        if let Some(durable) = replay_durable_ownership_transition(
            &self.reconciliation_journal_path(),
            DurableOwnershipTransitionKind::GlobalStop,
            &request.recovery_correlation,
            &request_fingerprint,
        )
        .map_err(ExplorationRecoveryError::FailedPrecondition)?
        {
            let guard = self.inner.lock().await;
            let retains_exact_live_ownership = guard.last_request_id == request.request_id
                && guard.session_generation == request.exploration_generation
                && guard
                    .active_child
                    .as_ref()
                    .is_some_and(|child| child.lease.matches(&request.child_lease))
                && guard
                    .pending_child_cancellation
                    .as_ref()
                    .and_then(|pending| pending.ticket.as_ref())
                    .is_some_and(|ticket| {
                        ticket.cancel_generation == request.cancel_generation
                            && ticket.lease.matches(&request.child_lease)
                    });
            if retains_exact_live_ownership {
                return Ok(None);
            }
            return Ok(Some(durable.runtime_snapshot()));
        }
        let guard = self.inner.lock().await;
        let Some(completed) = guard.completed_rollover_recovery.as_ref() else {
            return Ok(None);
        };
        if let CompletedOwnershipTransition::GlobalStop(completed) = completed {
            if completed.request == *request {
                return Ok(Some(completed.snapshot.clone()));
            }
        }
        Err(ExplorationRecoveryError::FailedPrecondition(
            "ownership reconciliation proof does not match the completed transition".to_string(),
        ))
    }

    pub async fn reconcile_after_global_stop(
        &self,
        request: ExplorationGlobalStopReconciliationRequest,
    ) -> Result<ExploreRuntimeSnapshot, ExplorationRecoveryError> {
        Self::validate_global_stop_reconciliation_request(&request)?;
        let request_fingerprint = global_stop_request_fingerprint(&request);
        let request_id = request.request_id.trim();
        let mut guard = self.inner.lock().await;

        if let Some(completed) = guard.completed_rollover_recovery.as_ref() {
            if let CompletedOwnershipTransition::GlobalStop(completed) = completed {
                if completed.request == request {
                    return Ok(completed.snapshot.clone());
                }
            }
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "ownership reconciliation proof does not match the completed transition"
                    .to_string(),
            ));
        }
        if guard.last_request_id != request_id {
            return Err(ExplorationRecoveryError::FailedPrecondition(format!(
                "exploration reconciliation request_id mismatch: active={} received={}",
                guard.last_request_id, request_id
            )));
        }
        if guard.session_generation != request.exploration_generation {
            return Err(ExplorationRecoveryError::FailedPrecondition(format!(
                "exploration reconciliation generation mismatch: active={} received={}",
                guard.session_generation, request.exploration_generation
            )));
        }
        let active_child = guard.active_child.as_ref().ok_or_else(|| {
            ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation has no retained child ownership".to_string(),
            )
        })?;
        if !active_child.lease.matches(&request.child_lease) {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation child lease does not match retained ownership"
                    .to_string(),
            ));
        }
        let active_motion_admission = guard
            .active_config
            .as_ref()
            .map(|config| config.motion_admission)
            .ok_or_else(|| {
                ExplorationRecoveryError::FailedPrecondition(
                    "exploration reconciliation has no original motion admission".to_string(),
                )
            })?;
        if active_motion_admission != request.guard_proof.original_motion_admission {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation motion admission mismatch".to_string(),
            ));
        }
        let pending = guard.pending_child_cancellation.as_ref().ok_or_else(|| {
            ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation has no retained cancellation".to_string(),
            )
        })?;
        let ticket = pending.ticket.as_ref().ok_or_else(|| {
            ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation requires an exact retained cancel ticket".to_string(),
            )
        })?;
        if guard.controller.is_active()
            || pending.failure.is_none()
            || pending.dispatch_in_flight
            || ticket.cancel_generation != request.cancel_generation
            || !ticket.lease.matches(&request.child_lease)
        {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation requires a stopped controller and exact failed cancellation"
                    .to_string(),
            ));
        }
        let expected_correlation = global_stop_recovery_correlation(
            &guard.last_request_id,
            guard.session_generation,
            &active_child.lease,
            ticket.cancel_generation,
            &active_motion_admission,
        );
        if request.recovery_correlation != expected_correlation {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration reconciliation correlation mismatch".to_string(),
            ));
        }

        let receipt = ExplorationOwnershipReconciliationReceipt {
            cause: GLOBAL_STOP_RECONCILIATION_CAUSE.to_string(),
            recovery_correlation: expected_correlation,
            parent_request_id: guard.last_request_id.clone(),
            exploration_generation: guard.session_generation,
            child_lease_fingerprint: child_lease_fingerprint(&active_child.lease),
            cancel_generation: ticket.cancel_generation,
            commit_disposition: GLOBAL_STOP_COMMIT_DISPOSITION.to_string(),
        };
        let mut snapshot = guard.snapshot(
            STATUS_FAILED,
            "GLOBAL_STOP_RECONCILED_SAFE: held Guard fence reconciled retained child ownership",
        );
        snapshot.last_stop_reason = "global_stop_reconciled_safe".to_string();
        snapshot.frontier_count = 0;
        if !guard.frontiers.is_empty() {
            snapshot.revision = snapshot.revision.saturating_add(1);
        }
        snapshot.child_task_id.clear();
        snapshot.child_lease = None;
        snapshot.rollover_recovery_eligible = false;
        snapshot.global_stop_reconciliation =
            ExplorationGlobalStopReconciliationSnapshot::default();
        snapshot.ownership_reconciliation_receipt = Some(receipt.clone());
        let durable_transition = DurableOwnershipTransition {
            kind: DurableOwnershipTransitionKind::GlobalStop,
            recovery_correlation: request.recovery_correlation.clone(),
            request_fingerprint,
            receipt: receipt.clone(),
            snapshot: durable_snapshot_from_runtime(&snapshot),
            committed_unix_ns: now_unix_nanos(),
            acknowledged: false,
        };
        match commit_durable_ownership_transition(
            &self.reconciliation_journal_path(),
            durable_transition,
        )
        .map_err(ExplorationRecoveryError::FailedPrecondition)?
        {
            DurableCommitOutcome::Committed => {}
            DurableCommitOutcome::ExactReplay(existing) => {
                if existing.receipt != receipt {
                    return Err(ExplorationRecoveryError::FailedPrecondition(
                        "global-stop durable replay receipt mismatch".to_string(),
                    ));
                }
                snapshot = existing.runtime_snapshot();
            }
        }
        guard.pending_child_dispatch = None;
        guard.pending_child_cancellation = None;
        guard.active_child = None;
        guard.no_frontier_since = None;
        guard.last_stop = Some(ExplorationStopReason::ConsecutiveFailuresExceeded);
        guard.last_stop_reason = "global_stop_reconciled_safe".to_string();
        let frontier_update = guard.update_frontiers(Vec::new());
        guard.completed_rollover_recovery = Some(CompletedOwnershipTransition::GlobalStop(
            CompletedGlobalStopReconciliation {
                request,
                snapshot: snapshot.clone(),
            },
        ));
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            self.expected_map_frame.as_ref(),
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontier_update);
        Ok(self.store_snapshot(snapshot))
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
                self.expected_map_frame.as_ref(),
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
                    None,
                )
                .await;
        };
        let (ticket, recovered_after_timeout) =
            match tokio::time::timeout(CHILD_CANCEL_DISPATCH_TIMEOUT, sink.cancel(&child.lease))
                .await
            {
                Err(_) => match sink.recover_cancel_receipt(&child.lease) {
                    Some(recovery)
                        if recovery.ticket.cancel_generation > 0
                            && recovery.ticket.lease.matches(&child.lease) =>
                    {
                        if let Some(failure) = recovery.failure {
                            return self
                                .record_child_cancel_failure(
                                    &child.lease,
                                    failure,
                                    Some(recovery.ticket),
                                )
                                .await;
                        }
                        (recovery.ticket, true)
                    }
                    Some(_) => {
                        return self
                            .record_child_cancel_failure(
                                &child.lease,
                                "navigation sink recovered an invalid cancellation ticket"
                                    .to_string(),
                                None,
                            )
                            .await;
                    }
                    None => {
                        return self
                            .record_child_cancel_failure(
                                &child.lease,
                                "navigation child cancellation dispatch timed out".to_string(),
                                None,
                            )
                            .await;
                    }
                },
                Ok(Ok(ticket))
                    if ticket.cancel_generation > 0 && ticket.lease.matches(&child.lease) =>
                {
                    (ticket, false)
                }
                Ok(Ok(_)) => {
                    return self
                        .record_child_cancel_failure(
                            &child.lease,
                            "navigation sink returned an invalid cancellation ticket".to_string(),
                            None,
                        )
                        .await;
                }
                Ok(Err(error)) => {
                    let ticket = sink
                        .recover_cancel_receipt(&child.lease)
                        .filter(|recovery| {
                            recovery.ticket.cancel_generation > 0
                                && recovery.ticket.lease.matches(&child.lease)
                        })
                        .map(|recovery| recovery.ticket);
                    return self
                        .record_child_cancel_failure(&child.lease, error, ticket)
                        .await;
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
                if recovered_after_timeout {
                    "CANCEL_RECEIPT_RECOVERED"
                } else if retry {
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
        ticket: Option<ChildNavigationCancelTicket>,
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
        let early_terminal = ticket.and_then(|ticket| {
            let cancel_generation = ticket.cancel_generation;
            pending.ticket = Some(ticket);
            take_early_cancel_terminal(pending, cancel_generation)
        });
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
            self.expected_map_frame.as_ref(),
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        *self.readiness_cache.lock() = readiness;
        let snapshot = self.store_snapshot(snapshot);
        drop(guard);
        self.publish_frontiers(frontier_update);
        if let Some(result) = early_terminal {
            return Box::pin(self.report_navigation_result(&child_lease.session_id, result))
                .await
                .unwrap_or(snapshot);
        }
        snapshot
    }

    pub async fn tick(&self, robot_xy: (f64, f64)) -> ExploreRuntimeSnapshot {
        let now_monotonic_ns = now_monotonic_nanos();
        self.tick_with_pose(ExploreRobotPose {
            x: robot_xy.0,
            y: robot_xy.1,
            z: 0.0,
            orientation: Quaternion::IDENTITY,
            frame_id: self.expected_map_frame.to_string(),
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
        if ![pose.x, pose.y, pose.z]
            .iter()
            .all(|value| value.is_finite())
            || !pose.orientation.yaw().is_finite()
            || pose.source_timestamp_ns > i64::MAX as u64
        {
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    "exploration pose is not a finite full body pose".into(),
                )
                .await;
        }
        if normalize_frame(&pose.frame_id) != self.expected_map_frame.as_ref() {
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!(
                        "global exploration requires pose frame={}; received frame={}",
                        self.expected_map_frame, pose.frame_id
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

        if self.orphan_cancellations.has_entries() {
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                "recovering orphan navigation ownership before child dispatch",
            );
            drop(guard);
            self.reconcile_orphan_cancellations().await;
            return self.store_snapshot(snapshot);
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

        let surface_mode = guard.surface_mode.is_some();
        let costmap = if surface_mode {
            if !guard
                .surface_mode
                .as_ref()
                .is_some_and(|mode| mode.input_ready && mode.accepting)
            {
                guard.no_frontier_since = None;
                let reason = guard
                    .surface_mode
                    .as_ref()
                    .and_then(|mode| mode.input_error.clone())
                    .unwrap_or_else(|| "surface map input unavailable".into());
                let snapshot = guard.snapshot(STATUS_PENDING, reason);
                drop(guard);
                return self.store_snapshot(snapshot);
            }
            None
        } else {
            guard.costmap_input.clone()
        };
        if !surface_mode && costmap.is_none() {
            guard.no_frontier_since = None;
            let snapshot = guard.snapshot(STATUS_PENDING, "source unavailable: missing costmap");
            drop(guard);
            return self.store_snapshot(snapshot);
        }
        if costmap.as_ref().is_some_and(|costmap| {
            normalize_frame(&costmap.frame_id) != self.expected_map_frame.as_ref()
        }) {
            drop(guard);
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!(
                        "global exploration requires costmap frame={}; received frame={}",
                        self.expected_map_frame,
                        costmap.as_ref().expect("checked costmap").frame_id
                    ),
                )
                .await;
        }
        if guard.costmap_received_monotonic_ns == 0
            || now_monotonic_nanos().saturating_sub(guard.costmap_received_monotonic_ns)
                > DEFAULT_COSTMAP_RECEIPT_TTL.as_nanos() as u64
        {
            guard.no_frontier_since = None;
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                if surface_mode {
                    "source unavailable: surface map input stale"
                } else {
                    "source unavailable: costmap input stale"
                },
            );
            drop(guard);
            return self.store_snapshot(snapshot);
        }
        if let Some(request) = guard.pending_child_dispatch.take() {
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

        // 先确认提交所需的同步上下文，再允许选择器领取目标。
        // 等待期间既没有导航副作用，也不能计为目标失败或无前沿。
        let navigation_context = (|| {
            let committed_map = guard
                .committed_map
                .clone()
                .ok_or_else(|| "committed map identity unavailable".to_string())?;
            let sink = self
                .navigation_sink
                .read()
                .clone()
                .ok_or_else(|| "navigation sink unavailable".to_string())?;
            Ok::<_, String>((
                committed_map,
                sink.current_cancel_generation()?,
                sink.current_map_name()?,
            ))
        })();
        let (committed_map, cancel_generation, expected_map_name) = match navigation_context {
            Ok(context) => context,
            Err(error) => {
                guard.no_frontier_since = None;
                let snapshot =
                    guard.snapshot(STATUS_PENDING, format!("wavefront goal withheld: {error}"));
                drop(guard);
                return self.store_snapshot(snapshot);
            }
        };
        let mut state = ExplorationStateSnapshot::new(
            PoseStamped {
                frame_id: if surface_mode {
                    NavFrameId::from_static(self.expected_map_frame.as_ref())
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
            Timestamp {
                nanos: now_ns as i64,
                clock: ClockKind::Unix,
            },
        );
        state.global_costmap = costmap.as_ref().map(|costmap| &costmap.grid);
        let poll_result = guard.controller.poll_next_goal(&state);
        let diagnostics = guard.controller.diagnostics();
        guard.last_selected_source = diagnostics.selected_source.unwrap_or_else(|| {
            if surface_mode {
                "surface".into()
            } else {
                String::new()
            }
        });
        let source_frontiers = guard.controller.frontier_snapshot();
        let frontier_update = guard.update_frontiers(source_frontiers);

        match poll_result {
            Ok(ExplorationPoll::Goal(goal)) => {
                guard.last_frontier_goal = Some(goal.clone());
                guard.last_poll = Some(ExplorationPoll::Goal(goal.clone()));
                guard.no_frontier_since = None;
                guard.child_sequence = guard.child_sequence.saturating_add(1);
                let child_request_id =
                    format!("{}.nav.{}", guard.last_request_id, guard.child_sequence);
                let mut navigation_goal = frontier_goal_to_navigation_goal(&goal);
                navigation_goal.id = child_request_id.clone();
                let config = guard.active_config.clone().unwrap_or_default();
                let intent = match build_wavefront_goal_intent(
                    guard.session_generation,
                    cancel_generation,
                    &guard.last_request_id,
                    &child_request_id,
                    &config.motion_admission,
                    &navigation_goal,
                    config.boundary.as_ref(),
                    committed_map,
                ) {
                    Ok(intent) => intent,
                    Err(error) => {
                        drop(guard);
                        self.publish_frontiers(frontier_update);
                        return self
                            .finish_session(
                                None,
                                ExplorationStopReason::ConsecutiveFailuresExceeded,
                                format!("wavefront goal intent invalid: {error}"),
                            )
                            .await;
                    }
                };
                let request = ChildNavigationRequest {
                    parent_request_id: guard.last_request_id.clone(),
                    request_id: child_request_id,
                    expected_map_name,
                    motion_admission: config.motion_admission,
                    frontier_goal_id: goal.id.clone(),
                    goal: navigation_goal,
                    boundary: config.boundary,
                    intent,
                };
                let dispatching = guard.snapshot(
                    STATUS_PENDING,
                    format!("dispatching frontier {} to child navigation", goal.id),
                );
                self.publish_frontiers(frontier_update);
                self.store_snapshot(dispatching);
                self.dispatch_child(generation, request, guard).await
            }
            Ok(ExplorationPoll::NoFrontier) => {
                let first_no_frontier = *guard.no_frontier_since.get_or_insert_with(Instant::now);
                guard.last_poll = Some(ExplorationPoll::NoFrontier);
                let limit_ms = guard
                    .active_config
                    .as_ref()
                    .and_then(|config| config.stop_conditions.no_frontier_timeout_ms)
                    .unwrap_or(DEFAULT_NO_FRONTIER_STABILITY.as_millis() as u64);
                if first_no_frontier.elapsed() >= Duration::from_millis(limit_ms) {
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
            Ok(ExplorationPoll::Pending) => {
                guard.no_frontier_since = None;
                guard.last_poll = Some(ExplorationPoll::Pending);
                let snapshot = guard.snapshot(
                    STATUS_PENDING,
                    "exploration source is awaiting work completion",
                );
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.store_snapshot(snapshot)
            }
            Ok(ExplorationPoll::SourceUnavailable(reason)) => {
                guard.no_frontier_since = None;
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

        // Freeze the exact request before releasing Explore state. Navigation
        // admission is async and must never hold the Explore inner lock.
        guard.pending_child_dispatch = Some(request.clone());
        drop(guard);

        let ticket = match sink.submit(request.clone()).await {
            Ok(ticket) => ticket,
            Err(error) => {
                return self
                    .record_dispatch_failure(generation, request, error)
                    .await;
            }
        };
        if ticket.lease.session_id != proposed_request_id
            || ticket.lease.parent_request_id != expected_parent_request_id
            || ticket.lease.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            || (request.intent.goal_generation > 0
                && u64::try_from(request.intent.goal_generation).ok()
                    != Some(ticket.lease.goal_generation))
            || ticket.lease.goal_generation == 0
            || ticket.lease.producer_runtime_generation == 0
            || ticket.lease.goal_payload_sha256 == [0; 32]
        {
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

        let mut guard = self.inner.lock().await;
        let frozen_request_matches = guard
            .pending_child_dispatch
            .as_ref()
            .is_some_and(|pending| {
                pending.request_id == request.request_id
                    && pending.parent_request_id == request.parent_request_id
                    && pending.intent == request.intent
            });
        if generation != guard.generation
            || !guard.controller.is_active()
            || !frozen_request_matches
        {
            drop(guard);
            // Nav may already own the exact lease. Do not attach it to a newer
            // Explore epoch. A failed immediate cancel must transfer ownership
            // to a retained exact-lease recovery task or remain in Explore's
            // persistent in-process orphan set for scheduler retry.
            self.orphan_cancellations.register(ticket.lease.clone());
            if sink.cancel(&ticket.lease).await.is_ok() {
                self.orphan_cancellations.mark_cancel_pending(&ticket.lease);
            }
            let _ = sink.retain_orphan_cancellation(ticket.lease);
            return self.snapshot();
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
        if error.is_session_fatal() {
            drop(guard);
            return self
                .finish_session(
                    None,
                    ExplorationStopReason::ConsecutiveFailuresExceeded,
                    reason,
                )
                .await;
        }
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
                    return Ok(self
                        .record_child_cancel_failure(&child_lease, reason, None)
                        .await);
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

            if let ChildNavigationResult::Cancelled(proof) = &result {
                let pending = guard
                    .pending_child_cancellation
                    .as_ref()
                    .expect("pending child cancellation checked above");
                if let Some(failure) = pending.failure.as_ref() {
                    let ticket = pending.ticket.as_ref().ok_or_else(|| {
                        "late exact ZEROED has no retained cancellation ticket".to_string()
                    })?;
                    let motion_admission = guard
                        .active_config
                        .as_ref()
                        .map(|config| config.motion_admission)
                        .ok_or_else(|| {
                            "late exact ZEROED has no original motion admission".to_string()
                        })?;
                    let recovery_correlation = global_stop_recovery_correlation(
                        &guard.last_request_id,
                        guard.session_generation,
                        &active_child.lease,
                        ticket.cancel_generation,
                        &motion_admission,
                    );
                    let request_fingerprint = exact_child_zero_request_fingerprint(proof);
                    let receipt = ExplorationOwnershipReconciliationReceipt {
                        cause: EXACT_CHILD_ZEROED_CAUSE.to_string(),
                        recovery_correlation: recovery_correlation.clone(),
                        parent_request_id: guard.last_request_id.clone(),
                        exploration_generation: guard.session_generation,
                        child_lease_fingerprint: child_lease_fingerprint(&active_child.lease),
                        cancel_generation: ticket.cancel_generation,
                        commit_disposition: EXACT_CHILD_ZEROED_DISPOSITION.to_string(),
                    };
                    let mut durable_snapshot = guard.snapshot(
                        STATUS_FAILED,
                        format!("child cancellation failed before late ZEROED proof: {failure}"),
                    );
                    durable_snapshot.frontier_count = 0;
                    if !guard.frontiers.is_empty() {
                        durable_snapshot.revision = durable_snapshot.revision.saturating_add(1);
                    }
                    durable_snapshot.child_task_id.clear();
                    durable_snapshot.child_lease = None;
                    durable_snapshot.rollover_recovery_eligible = false;
                    durable_snapshot.global_stop_reconciliation =
                        ExplorationGlobalStopReconciliationSnapshot::default();
                    durable_snapshot.ownership_reconciliation_receipt = Some(receipt.clone());
                    let transition = DurableOwnershipTransition {
                        kind: DurableOwnershipTransitionKind::ExactChildZeroed,
                        recovery_correlation: recovery_correlation.clone(),
                        request_fingerprint: request_fingerprint.clone(),
                        receipt: receipt.clone(),
                        snapshot: durable_snapshot_from_runtime(&durable_snapshot),
                        committed_unix_ns: now_unix_nanos(),
                        acknowledged: false,
                    };
                    match commit_durable_ownership_transition(
                        &self.reconciliation_journal_path(),
                        transition,
                    )
                    .map_err(|error| {
                        format!("late exact ZEROED ownership journal commit failed: {error}")
                    })? {
                        DurableCommitOutcome::Committed => {}
                        DurableCommitOutcome::ExactReplay(existing) => {
                            if existing.receipt != receipt {
                                return Err(
                                    "late exact ZEROED durable replay receipt mismatch".to_string()
                                );
                            }
                        }
                    }
                    guard.completed_rollover_recovery = Some(
                        CompletedOwnershipTransition::ExactChildZeroed(CompletedExactChildZeroed {
                            recovery_correlation,
                            request_fingerprint,
                            receipt,
                        }),
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
            guard.no_frontier_since = None;
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
                self.expected_map_frame.as_ref(),
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
        guard.no_frontier_since = None;
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
        self.ingest_costmap_snapshot(ExploreCostmapSnapshot {
            grid: costmap,
            frame_id: self.expected_map_frame.to_string(),
        })
        .await
        .unwrap_or(false)
    }

    pub async fn ingest_costmap_snapshot(
        &self,
        snapshot: ExploreCostmapSnapshot,
    ) -> Result<bool, String> {
        self.ingest_costmap_with_identity(snapshot, None).await
    }

    pub async fn ingest_committed_costmap_snapshot(
        &self,
        snapshot: ExploreCostmapSnapshot,
        committed_map: schema::CommittedMapIdentity,
    ) -> Result<bool, String> {
        self.ingest_costmap_with_identity(snapshot, Some(committed_map))
            .await
    }

    async fn ingest_costmap_with_identity(
        &self,
        snapshot: ExploreCostmapSnapshot,
        committed_map: Option<schema::CommittedMapIdentity>,
    ) -> Result<bool, String> {
        validate_costmap_snapshot(&snapshot, self.expected_map_frame.as_ref())?;
        if let Some(identity) = committed_map.as_ref() {
            if identity.session_id.trim().is_empty()
                || identity.session_generation <= 0
                || identity.revision <= 0
                || identity.payload_sha256 == [0; 32]
                || normalize_frame(&identity.frame_id) != self.expected_map_frame.as_ref()
            {
                return Err(
                    "committed costmap identity is incomplete or has wrong frame".to_string(),
                );
            }
        }
        let mut guard = self.inner.lock().await;
        if guard.surface_mode.is_some() {
            // 二维栅格仅保留展示，不覆盖三维输入的权威身份与原始接收时刻。
            let changed = guard.costmap_input.as_ref() != Some(&snapshot);
            guard.costmap_input = Some(snapshot);
            return Ok(changed);
        }
        if guard.costmap_input.as_ref() == Some(&snapshot)
            && guard.committed_map.as_ref() == committed_map.as_ref()
        {
            // 新的输入接收事件可以续期，状态查询或缓存读取不会进入这里。
            guard.costmap_received_monotonic_ns = now_monotonic_nanos();
            return Ok(false);
        }
        if let Some(identity) = committed_map.as_ref() {
            if guard.committed_map.as_ref().is_some_and(|current| {
                let same_session = identity.session_id == current.session_id
                    && identity.session_generation == current.session_generation;
                same_session
                    && (identity.revision < current.revision
                        || (identity.revision == current.revision && identity != current))
            }) {
                return Err("committed costmap identity is stale or conflicting".to_string());
            }
        }
        // 缺少提交身份的输入必须清除旧身份，避免把未提交栅格冒充为已提交快照。
        guard.committed_map = committed_map;
        guard.costmap_received_monotonic_ns = now_monotonic_nanos();

        // 实时地图只刷新下一轮探索输入，不取消当前目标。
        guard.costmap_input = Some(snapshot);
        let current_frontiers = guard.frontiers.clone();
        let frontier_update = guard.update_frontiers(current_frontiers);
        let backend_available = self.readiness_cache.lock().backend_available;
        let readiness = Self::compute_readiness_locked(
            &guard,
            self.expected_map_frame.as_ref(),
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
                self.expected_map_frame.as_ref(),
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
            || self.orphan_cancellations.blocks_motion_mutation()
    }

    pub async fn readiness(&self, backend_available: bool) -> ExplorationReadinessDetail {
        let guard = self.inner.lock().await;
        let detail = Self::compute_readiness_locked(
            &guard,
            self.expected_map_frame.as_ref(),
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
                self.expected_map_frame.as_ref(),
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
        expected_map_frame: &str,
        backend_available: bool,
        navigation_sink_ready: bool,
    ) -> ExplorationReadinessDetail {
        let controller_active = guard.controller.is_active();
        let source_count = guard.controller.source_count();
        let frontier_source_ready = controller_active && source_count > 0;
        let costmap_input_ready = guard.surface_mode.as_ref().map_or_else(
            || {
                guard.costmap_input.as_ref().is_some_and(|snapshot| {
                    validate_costmap_snapshot(snapshot, expected_map_frame).is_ok()
                })
            },
            |mode| {
                mode.config.is_ok()
                    && mode.accepting
                    && mode.input_ready
                    && mode
                        .receiver
                        .snapshot()
                        .is_ok_and(|snapshot| snapshot.is_some())
            },
        );
        let ready = backend_available
            && frontier_source_ready
            && costmap_input_ready
            && navigation_sink_ready;
        let reason = if !backend_available {
            "LCM backend unavailable"
        } else if let Some(error) = guard
            .surface_mode
            .as_ref()
            .and_then(|mode| mode.config.as_ref().err())
        {
            error.as_str()
        } else if !controller_active {
            "exploration controller not active"
        } else if source_count == 0 {
            "no exploration source registered"
        } else if !costmap_input_ready {
            guard
                .surface_mode
                .as_ref()
                .and_then(|mode| mode.input_error.as_deref())
                .unwrap_or("valid map-frame costmap not available")
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
            navigation_sink_ready,
            active_source: guard
                .controller
                .diagnostics()
                .selected_source
                .unwrap_or_else(|| {
                    if guard.surface_mode.is_some() {
                        "surface"
                    } else {
                        "wavefront"
                    }
                    .to_string()
                }),
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

fn validate_costmap_snapshot(
    snapshot: &ExploreCostmapSnapshot,
    expected_map_frame: &str,
) -> Result<(), String> {
    if normalize_frame(&snapshot.frame_id) != expected_map_frame {
        return Err(format!(
            "global exploration requires costmap frame={expected_map_frame}; received frame={}",
            snapshot.frame_id,
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

fn validate_expected_map_frame(frame_id: String) -> Result<Arc<str>, String> {
    let valid = frame_id == frame_id.trim()
        && !frame_id.is_empty()
        && frame_id.len() <= 255
        && frame_id.split('/').all(|token| {
            let mut chars = token.chars();
            chars
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && chars.all(|character| character.is_ascii_alphanumeric() || character == '_')
        });
    if !valid {
        return Err(format!(
            "exploration expected map frame is invalid: {frame_id:?}"
        ));
    }
    Ok(Arc::from(frame_id))
}

fn normalize_frame(frame_id: &str) -> &str {
    frame_id.trim()
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
        if guard.committed_map.is_some()
            && guard.costmap_received_monotonic_ns != 0
            && now_monotonic_nanos().saturating_sub(guard.costmap_received_monotonic_ns)
                <= DEFAULT_COSTMAP_RECEIPT_TTL.as_nanos() as u64
            && coverage_pct(&costmap.grid, config.boundary.as_ref())
                .is_some_and(|coverage| coverage >= max_coverage)
        {
            return Some(ExplorationStopReason::CoverageReached);
        }
    }
    None
}

fn coverage_pct(grid: &OccupancyGrid2D, boundary: Option<&ExplorationBoundary>) -> Option<f64> {
    let mut total = 0usize;
    let mut known = 0usize;
    for y in 0..grid.height {
        for x in 0..grid.width {
            let point = grid.cell_center(x, y)?;
            if boundary.is_some_and(|boundary| !boundary.contains(point)) {
                continue;
            }
            let cell = grid.get(x, y)?;
            total += 1;
            known += usize::from(cell != OccupancyCell::Unknown);
        }
    }
    (total > 0).then(|| known as f64 * 100.0 / total as f64)
}

#[derive(Clone)]
pub struct ExploreSchedulerHandle {
    inner: Arc<ExploreSchedulerInner>,
}

struct ExploreSchedulerInner {
    shutdown_tx: watch::Sender<bool>,
    terminal_tx: watch::Sender<Option<Result<(), String>>>,
    terminal_rx: watch::Receiver<Option<Result<(), String>>>,
    monitor: Mutex<JoinHandle<()>>,
    worker_abort: AbortHandle,
    monitor_state: AtomicU8,
    monitor_error: SyncMutex<Option<String>>,
}

const SCHEDULER_MONITOR_PENDING: u8 = 0;
const SCHEDULER_MONITOR_SETTLED: u8 = 1;
const SCHEDULER_MONITOR_FAILED: u8 = 2;

/// A shutdown deadline that can wake an async waiter even when the Tokio time
/// driver is owned by a synchronously blocked producer.
struct WallClockWake {
    wake: tokio::sync::oneshot::Receiver<()>,
}

struct WallClockDeadlineRequest {
    deadline: Instant,
    wake: tokio::sync::oneshot::Sender<()>,
}

struct WallClockDeadlineService {
    sender: std::sync::mpsc::Sender<WallClockDeadlineRequest>,
    _worker: std::thread::JoinHandle<()>,
}

fn wall_clock_deadline_service() -> Result<&'static WallClockDeadlineService, String> {
    static SERVICE: OnceLock<Result<WallClockDeadlineService, String>> = OnceLock::new();
    match SERVICE.get_or_init(|| {
        let (sender, receiver) = std::sync::mpsc::channel();
        let worker = std::thread::Builder::new()
            .name("explore-scheduler-wall-clock".to_owned())
            .spawn(move || run_wall_clock_deadline_service(receiver))
            .map_err(|error| format!("spawn wall-clock deadline failed: {error}"))?;
        Ok(WallClockDeadlineService {
            sender,
            _worker: worker,
        })
    }) {
        Ok(service) => Ok(service),
        Err(error) => Err(error.clone()),
    }
}

fn run_wall_clock_deadline_service(receiver: std::sync::mpsc::Receiver<WallClockDeadlineRequest>) {
    let mut pending: Vec<WallClockDeadlineRequest> = Vec::new();
    loop {
        let now = Instant::now();
        let mut index = 0;
        while index < pending.len() {
            if pending[index].wake.is_closed() {
                pending.swap_remove(index);
            } else if pending[index].deadline <= now {
                let request = pending.swap_remove(index);
                let _ = request.wake.send(());
            } else {
                index += 1;
            }
        }

        let next_wait = pending
            .iter()
            .map(|request| request.deadline.saturating_duration_since(Instant::now()))
            .min();
        let received = match next_wait {
            Some(wait) => receiver.recv_timeout(wait),
            None => match receiver.recv() {
                Ok(request) => {
                    pending.push(request);
                    continue;
                }
                Err(_) => return,
            },
        };
        match received {
            Ok(request) => pending.push(request),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return,
        }
    }
}

impl WallClockWake {
    fn new(duration: Duration) -> Result<Self, String> {
        let deadline = Instant::now()
            .checked_add(duration)
            .ok_or_else(|| "wall-clock deadline overflow".to_owned())?;
        let (wake_tx, wake) = tokio::sync::oneshot::channel();
        wall_clock_deadline_service()?
            .sender
            .send(WallClockDeadlineRequest {
                deadline,
                wake: wake_tx,
            })
            .map_err(|_| "wall-clock deadline service stopped".to_owned())?;
        Ok(Self { wake })
    }

    async fn wait(&mut self) {
        let _ = (&mut self.wake).await;
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ExploreSchedulerShutdownOutcome {
    result: Result<(), String>,
    producers_settled: bool,
}

impl ExploreSchedulerShutdownOutcome {
    fn new(result: Result<(), String>, producers_settled: bool) -> Self {
        Self {
            result,
            producers_settled,
        }
    }

    pub(crate) fn producers_settled(&self) -> bool {
        self.producers_settled
    }

    pub(crate) fn into_result(self) -> Result<(), String> {
        self.result
    }
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
        #[cfg(test)]
        let topology_control = crate::lcm_runtime::topology_test_control();
        let period = if period.is_zero() {
            DEFAULT_SCHEDULER_PERIOD
        } else {
            period
        };
        let worker = tokio::spawn(async move {
            #[cfg(test)]
            crate::lcm_runtime::wait_for_topology_scheduler_release(topology_control);
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
        let worker_abort = worker.abort_handle();
        let (terminal_tx, terminal_rx) = watch::channel(None);
        let monitor_terminal_tx = terminal_tx.clone();
        let monitor_shutdown_rx = shutdown_tx.subscribe();
        let monitor = tokio::spawn(async move {
            let result = match worker.await {
                Ok(()) => Ok(()),
                Err(error) if error.is_cancelled() && *monitor_shutdown_rx.borrow() => Ok(()),
                Err(error) => Err(format!("exploration scheduler task join failed: {error}")),
            };
            monitor_terminal_tx.send_replace(Some(result));
        });
        Self {
            inner: Arc::new(ExploreSchedulerInner {
                shutdown_tx,
                terminal_tx,
                terminal_rx,
                monitor: Mutex::new(monitor),
                worker_abort,
                monitor_state: AtomicU8::new(SCHEDULER_MONITOR_PENDING),
                monitor_error: SyncMutex::new(None),
            }),
        }
    }

    pub fn shutdown_is_requested(&self) -> bool {
        *self.inner.shutdown_tx.borrow()
    }

    async fn wait_final_result(&self) -> Result<(), String> {
        let mut terminal_rx = self.inner.terminal_rx.clone();
        if let Some(result) = terminal_rx.borrow().clone() {
            return result;
        }
        let result = terminal_rx
            .wait_for(Option::is_some)
            .await
            .map_err(|_| "exploration scheduler terminal channel closed".to_owned())?
            .clone()
            .expect("scheduler terminal wait accepts only Some");
        result
    }

    /// Observe only an unrequested scheduler terminal. A clean terminal after
    /// `shutdown()` remains owned by the root task supervisor, which performs
    /// the final settlement when its persistent-stop signal arrives.
    pub async fn wait_terminal(&self) -> Result<(), String> {
        let result = self.wait_final_result().await;
        if result.is_ok() && self.shutdown_is_requested() {
            std::future::pending::<Result<(), String>>().await
        } else {
            result
        }
    }

    pub async fn shutdown(&self) -> Result<(), String> {
        self.shutdown_with_settlement().await.into_result()
    }

    pub(crate) async fn shutdown_with_settlement(&self) -> ExploreSchedulerShutdownOutcome {
        self.inner.shutdown_tx.send_replace(true);
        match self.inner.monitor_state.load(Ordering::Acquire) {
            SCHEDULER_MONITOR_SETTLED => {
                return ExploreSchedulerShutdownOutcome::new(self.wait_final_result().await, true);
            }
            SCHEDULER_MONITOR_FAILED => {
                let error = self
                    .inner
                    .monitor_error
                    .lock()
                    .clone()
                    .unwrap_or_else(|| "exploration scheduler monitor failed".to_owned());
                return ExploreSchedulerShutdownOutcome::new(Err(error), false);
            }
            SCHEDULER_MONITOR_PENDING => {}
            _ => unreachable!("invalid exploration scheduler monitor state"),
        }

        let mut acquisition_deadline = match WallClockWake::new(SCHEDULER_SHUTDOWN_TIMEOUT) {
            Ok(deadline) => deadline,
            Err(error) => {
                return ExploreSchedulerShutdownOutcome::new(
                    Err(format!(
                        "exploration scheduler monitor acquisition deadline unavailable: {error}"
                    )),
                    false,
                )
            }
        };
        let mut monitor = tokio::select! {
            biased;
            monitor = self.inner.monitor.lock() => monitor,
            _ = acquisition_deadline.wait() => {
                return ExploreSchedulerShutdownOutcome::new(
                    Err("exploration scheduler concurrent shutdown timed out".to_owned()),
                    false,
                )
            }
        };
        match self.inner.monitor_state.load(Ordering::Acquire) {
            SCHEDULER_MONITOR_SETTLED => {
                drop(monitor);
                return ExploreSchedulerShutdownOutcome::new(self.wait_final_result().await, true);
            }
            SCHEDULER_MONITOR_FAILED => {
                let error = self
                    .inner
                    .monitor_error
                    .lock()
                    .clone()
                    .unwrap_or_else(|| "exploration scheduler monitor failed".to_owned());
                drop(monitor);
                return ExploreSchedulerShutdownOutcome::new(Err(error), false);
            }
            SCHEDULER_MONITOR_PENDING => {}
            _ => unreachable!("invalid exploration scheduler monitor state"),
        }
        let mut graceful_deadline = match WallClockWake::new(SCHEDULER_SHUTDOWN_TIMEOUT) {
            Ok(deadline) => deadline,
            Err(error) => {
                return ExploreSchedulerShutdownOutcome::new(
                    Err(format!(
                        "exploration scheduler shutdown deadline unavailable: {error}"
                    )),
                    false,
                )
            }
        };
        tokio::select! {
            biased;
            result = &mut *monitor => match result {
                Ok(()) => {
                    self.inner
                        .monitor_state
                        .store(SCHEDULER_MONITOR_SETTLED, Ordering::Release);
                    drop(monitor);
                    return ExploreSchedulerShutdownOutcome::new(
                        self.wait_final_result().await,
                        true,
                    );
                }
                Err(error) => {
                    let detail = format!("exploration scheduler monitor join failed: {error}");
                    *self.inner.monitor_error.lock() = Some(detail.clone());
                    self.inner
                        .monitor_state
                        .store(SCHEDULER_MONITOR_FAILED, Ordering::Release);
                    drop(monitor);
                    return ExploreSchedulerShutdownOutcome::new(
                        Err(detail),
                        false,
                    );
                }
            },
            _ = graceful_deadline.wait() => {}
        }
        self.inner.worker_abort.abort();
        let mut abort_deadline = match WallClockWake::new(SCHEDULER_ABORT_JOIN_TIMEOUT) {
            Ok(deadline) => deadline,
            Err(error) => {
                return ExploreSchedulerShutdownOutcome::new(
                    Err(format!(
                        "exploration scheduler abort-join deadline unavailable: {error}"
                    )),
                    false,
                )
            }
        };
        tokio::select! {
            biased;
            result = &mut *monitor => match result {
                Ok(()) => {
                    self.inner
                        .monitor_state
                        .store(SCHEDULER_MONITOR_SETTLED, Ordering::Release);
                    drop(monitor);
                    ExploreSchedulerShutdownOutcome::new(self.wait_final_result().await, true)
                }
                Err(error) => {
                    let detail = format!(
                        "exploration scheduler monitor abort join failed: {error}"
                    );
                    *self.inner.monitor_error.lock() = Some(detail.clone());
                    self.inner
                        .monitor_state
                        .store(SCHEDULER_MONITOR_FAILED, Ordering::Release);
                    drop(monitor);
                    ExploreSchedulerShutdownOutcome::new(Err(detail), false)
                }
            },
            _ = abort_deadline.wait() => {
                ExploreSchedulerShutdownOutcome::new(
                    Err("exploration scheduler abort join timed out".to_owned()),
                    false,
                )
            }
        }
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
    use nuwax_nav_types_messages::geometry::Point2;
    use std::borrow::Cow;
    use std::collections::VecDeque;
    use std::sync::Mutex as StdMutex;

    static NEXT_RECONCILIATION_TEST_PATH: AtomicU64 = AtomicU64::new(0);

    fn reconciliation_test_path(label: &str) -> PathBuf {
        std::env::temp_dir()
            .join(format!(
                "nuwax-exploration-reconciliation-{}-{}-{label}",
                std::process::id(),
                NEXT_RECONCILIATION_TEST_PATH.fetch_add(1, Ordering::Relaxed)
            ))
            .join("journal.json")
    }

    #[derive(Default)]
    struct RecordingNavSink {
        submitted: StdMutex<Vec<ChildNavigationRequest>>,
        submit_errors: StdMutex<VecDeque<ChildNavigationSubmitError>>,
        cancel_snapshot: StdMutex<Option<Result<u64, String>>>,
        map_name_error: StdMutex<Option<String>>,
        cancelled: StdMutex<Vec<String>>,
        cancel_error: StdMutex<Option<String>>,
        cancel_recovery: StdMutex<Option<ChildNavigationCancelRecovery>>,
        lease_override: StdMutex<Option<ChildNavigationLease>>,
        next_goal_generation: AtomicU64,
        next_cancel_generation: AtomicU64,
        submit_entered: tokio::sync::Notify,
        submit_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
        cancel_entered: tokio::sync::Notify,
        cancel_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
    }

    #[async_trait]
    impl ExploreNavigationSink for RecordingNavSink {
        fn current_cancel_generation(&self) -> Result<u64, String> {
            self.cancel_snapshot
                .lock()
                .unwrap()
                .clone()
                .unwrap_or(Ok(0))
        }

        fn current_map_name(&self) -> Result<String, String> {
            if let Some(error) = self.map_name_error.lock().unwrap().clone() {
                return Err(error);
            }
            Ok("map".to_string())
        }

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
            let lease = self
                .lease_override
                .lock()
                .unwrap()
                .clone()
                .unwrap_or_else(|| ChildNavigationLease {
                    session_id: request.request_id.clone(),
                    owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                    parent_request_id: request.parent_request_id.clone(),
                    goal_generation: self.next_goal_generation.fetch_add(1, Ordering::SeqCst) + 1,
                    producer_runtime_generation: 11,
                    goal_payload_sha256: [7; 32],
                });
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

        fn recover_cancel_receipt(
            &self,
            lease: &ChildNavigationLease,
        ) -> Option<ChildNavigationCancelRecovery> {
            self.cancel_recovery
                .lock()
                .unwrap()
                .clone()
                .filter(|recovery| recovery.ticket.lease.matches(lease))
        }
    }

    struct StaticPoseProvider;

    #[async_trait]
    impl ExplorePoseProvider for StaticPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            Ok(ExploreRobotPose {
                x: 2.0,
                y: 2.0,
                z: 0.0,
                orientation: nuwax_nav_types_messages::geometry::Quaternion::IDENTITY,
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

    struct BlockingPoseProvider {
        entered: Arc<tokio::sync::Notify>,
    }

    #[async_trait]
    impl ExplorePoseProvider for BlockingPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            self.entered.notify_one();
            std::future::pending().await
        }
    }

    struct PanickingPoseProvider;

    #[async_trait]
    impl ExplorePoseProvider for PanickingPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            panic!("injected exploration scheduler panic")
        }
    }

    struct SynchronousBlockingPoseProvider {
        entered: Arc<std::sync::atomic::AtomicBool>,
        release: Arc<std::sync::atomic::AtomicBool>,
    }

    #[async_trait]
    impl ExplorePoseProvider for SynchronousBlockingPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            self.entered.store(true, Ordering::Release);
            while !self.release.load(Ordering::Acquire) {
                std::thread::yield_now();
            }
            StaticPoseProvider.current_pose().await
        }
    }

    fn frontier_costmap(_fixture: u64) -> ExploreCostmapSnapshot {
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
        }
    }

    fn occupied_costmap(_fixture: u64) -> ExploreCostmapSnapshot {
        ExploreCostmapSnapshot {
            grid: OccupancyGrid2D {
                resolution: 0.5,
                origin: nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
                width: 8,
                height: 8,
                cells: vec![OccupancyCell::Occupied; 64],
            },
            frame_id: "map".to_string(),
        }
    }

    fn committed_map_identity(revision: u64, frame_id: &str) -> schema::CommittedMapIdentity {
        schema::CommittedMapIdentity {
            session_id: "explore-test-map".to_string(),
            session_generation: 1,
            revision: revision as i64,
            payload_sha256: crate::map_checksum::sha256_bytes(&revision.to_le_bytes()),
            frame_id: frame_id.to_string(),
        }
    }

    async fn ingest_committed_costmap_fixture(
        handle: &ExploreRuntimeHandle,
        snapshot: ExploreCostmapSnapshot,
        revision: u64,
    ) -> Result<bool, String> {
        let frame_id = snapshot.frame_id.clone();
        handle
            .ingest_committed_costmap_snapshot(
                snapshot,
                committed_map_identity(revision, &frame_id),
            )
            .await
    }

    fn configured_handle() -> (ExploreRuntimeHandle, Arc<RecordingNavSink>) {
        let handle = ExploreRuntimeHandle::default_with_wavefront();
        handle.set_reconciliation_journal_path_for_tests(reconciliation_test_path("handle"));
        let sink = Arc::new(RecordingNavSink::default());
        handle.set_navigation_sink(sink.clone());
        (handle, sink)
    }

    fn surface_config(frame: &str) -> SurfaceNavigationConfig {
        SurfaceNavigationConfig::new(frame.into(), 0.5, 0.2, 0.6, 0.1, 0.15, 18.0, 0.2).unwrap()
    }

    fn surface_message(frame: &str, revision: i32) -> schema::NavigationMap3D {
        let mut data = Vec::new();
        for z in [0.0_f32, 3.0] {
            for x in 0..50 {
                for y in 0..30 {
                    for value in [(x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z] {
                        data.extend_from_slice(&value.to_le_bytes());
                    }
                }
            }
        }
        let timestamp = now_unix_nanos() as i64;
        schema::NavigationMap3D {
            producer_runtime_generation: 37,
            committed_map: schema::CommittedMapIdentity {
                session_id: "surface-session".into(),
                session_generation: 1,
                revision: i64::from(revision),
                payload_sha256: [4; 32],
                frame_id: frame.into(),
            },
            map_frame_generation: 1,
            cloud_revision: i64::from(revision),
            source_timestamp_ns: timestamp,
            source_voxel_size: 0.1,
            commit_boot_nonce: [7; 16],
            commit_sequence: i64::from(revision),
            cloud: schema::PointCloud2 {
                header: schema::Header {
                    seq: revision,
                    stamp_sec: (timestamp / 1_000_000_000) as i32,
                    stamp_nsec: (timestamp % 1_000_000_000) as i32,
                    frame_id: frame.into(),
                },
                width: (data.len() / 12) as i32,
                height: 1,
                point_step: 12,
                row_step: data.len() as i32,
                fields: ["x", "y", "z"]
                    .iter()
                    .enumerate()
                    .map(|(index, name)| schema::PointField {
                        name: (*name).into(),
                        offset: index as i32 * 4,
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                data,
                is_dense: true,
                ..schema::PointCloud2::default()
            },
            ..schema::NavigationMap3D::default()
        }
    }

    fn surface_pose(
        snapshot: &SurfaceSnapshot,
        height: f64,
        orientation: Quaternion,
    ) -> ExploreRobotPose {
        let support = snapshot.locate_support([1.5, 1.5, height]).unwrap();
        let body = snapshot
            .body_position_from_support(
                support.support_xyz,
                [orientation.x, orientation.y, orientation.z, orientation.w],
                0.5,
            )
            .unwrap();
        ExploreRobotPose {
            x: body[0],
            y: body[1],
            z: body[2],
            orientation,
            frame_id: snapshot.config().world_frame.clone(),
            source_timestamp_ns: now_unix_nanos(),
            received_monotonic_ns: now_monotonic_nanos(),
        }
    }

    async fn install_surface_fixture(
        handle: &ExploreRuntimeHandle,
        frame: &str,
    ) -> (
        schema::NavigationMap3D,
        SurfaceMapTicket,
        Arc<SurfaceSnapshot>,
    ) {
        handle
            .configure_surface_source(Ok(surface_config(frame)), 37, [7; 16])
            .await
            .unwrap();
        let message = surface_message(frame, 1);
        let ticket = handle
            .begin_surface_map_update(&message, now_monotonic_nanos())
            .await
            .unwrap()
            .unwrap();
        let snapshot = Arc::new(
            SurfaceSnapshot::from_points(
                &schema::navigation_map_3d_points(&message).unwrap(),
                surface_config(frame).geometry,
                message.source_voxel_size,
            )
            .unwrap(),
        );
        assert!(
            handle
                .finish_surface_map_update(ticket.clone(), Ok(snapshot.clone()))
                .await
        );
        (message, ticket, snapshot)
    }

    #[tokio::test]
    async fn surface_goals_preserve_real_floor_height_full_orientation_and_source() {
        for (strategy, floor) in [("frontier", 0.0), ("wavefront", 3.0), ("surface", 3.0)] {
            let handle = ExploreRuntimeHandle::with_wavefront_map_frame("map").unwrap();
            let sink = Arc::new(RecordingNavSink::default());
            handle.set_navigation_sink(sink.clone());
            let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
            let orientation = Quaternion {
                x: 0.05_f64.sin(),
                y: 0.0,
                z: 0.0,
                w: 0.05_f64.cos(),
            };
            let started = handle
                .start_with_config(
                    "surface-height",
                    ExploreStartConfig {
                        strategy: strategy.into(),
                        motion_admission: crate::nav_runtime::test_motion_admission(),
                        ..ExploreStartConfig::default()
                    },
                )
                .await;
            assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
            assert_eq!(started.selected_source, "surface");
            let active = handle
                .tick_with_pose(surface_pose(&snapshot, floor, orientation))
                .await;
            assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
            assert_eq!(active.selected_source, "surface");
            let requests = sink.submitted.lock().unwrap();
            let goal = &requests[0].goal.target;
            assert!(goal.pose.position.z > floor + 0.4 && goal.pose.position.z < floor + 0.7);
            assert_eq!(goal.pose.orientation, orientation);
            assert_eq!(
                requests[0].intent.goal.pose.position[2],
                goal.pose.position.z
            );
            assert_eq!(requests[0].intent.goal.header.frame_id, "map");
        }
    }

    #[tokio::test]
    async fn surface_clear_stops_exact_child_and_discards_old_inputs_and_computation() {
        let (handle, sink) = configured_handle();
        let (message, ticket, snapshot) = install_surface_fixture(&handle, "map").await;
        handle.start("surface-clear", "prefer_wavefront").await;
        let active = handle
            .tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY))
            .await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let lease = active_child_lease(&handle).await;
        let mut clear = message.clone();
        clear.cleared = true;
        clear.cloud_revision = 2;
        clear.source_timestamp_ns = 0;
        clear.commit_sequence = 0;
        clear.committed_map.revision = 0;
        clear.committed_map.payload_sha256 = [0; 32];
        clear.cloud = schema::PointCloud2 {
            header: schema::Header {
                seq: 2,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            ..schema::PointCloud2::default()
        };
        assert!(handle
            .begin_surface_map_update(&clear, now_monotonic_nanos())
            .await
            .unwrap()
            .is_none());
        assert_eq!(handle.snapshot().status, STATUS_STOPPING);
        assert_eq!(
            sink.cancelled.lock().unwrap().as_slice(),
            &[active.child_task_id.clone()]
        );
        assert!(!handle.finish_surface_map_update(ticket, Ok(snapshot)).await);
        assert!(handle
            .begin_surface_map_update(&message, now_monotonic_nanos())
            .await
            .is_err());
        let mut wrong_producer = surface_message("map", 3);
        wrong_producer.producer_runtime_generation = 36;
        assert!(handle
            .begin_surface_map_update(&wrong_producer, now_monotonic_nanos())
            .await
            .is_err());
        let stopped = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .unwrap();
        assert_eq!(stopped.status, STATUS_FAILED);
        assert_eq!(stopped.last_stop_reason, "map_revision_changed");
        assert!(stopped.child_task_id.is_empty());
        assert!(handle
            .inner
            .lock()
            .await
            .surface_mode
            .as_ref()
            .unwrap()
            .receiver
            .snapshot()
            .unwrap()
            .is_none());
    }

    #[tokio::test]
    async fn surface_domain_change_and_two_dimensional_display_preserve_input_authority() {
        let (handle, _) = configured_handle();
        let (message, old_ticket, snapshot) = install_surface_fixture(&handle, "map").await;
        let received = handle.inner.lock().await.costmap_received_monotonic_ns;
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 90)
            .await
            .unwrap();
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&message.committed_map));
            assert_eq!(guard.costmap_received_monotonic_ns, received);
        }
        let wrong_frame = surface_message("odom", 2);
        assert!(handle
            .begin_surface_map_update(&wrong_frame, now_monotonic_nanos())
            .await
            .is_err());
        let mut new_map = surface_message("map", 2);
        new_map.committed_map.session_id = "other-map-session".into();
        new_map.committed_map.session_generation = 2;
        handle
            .begin_surface_map_update(&new_map, now_monotonic_nanos())
            .await
            .unwrap()
            .unwrap();
        assert!(
            !handle
                .finish_surface_map_update(old_ticket, Ok(snapshot))
                .await
        );
        let guard = handle.inner.lock().await;
        assert!(guard.committed_map.is_none());
        assert_eq!(guard.costmap_received_monotonic_ns, 0);
        assert!(guard
            .surface_mode
            .as_ref()
            .unwrap()
            .receiver
            .snapshot()
            .unwrap()
            .is_none());
    }

    #[tokio::test]
    async fn surface_configuration_and_coverage_errors_reject_exploration_only() {
        let (handle, _) = configured_handle();
        let surface_request = ExploreStartConfig {
            strategy: "surface".into(),
            motion_admission: crate::nav_runtime::test_motion_admission(),
            ..ExploreStartConfig::default()
        };
        let rejected = handle
            .start_with_config("surface-two-dimensional", surface_request.clone())
            .await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        handle
            .configure_surface_source(Err("missing body/support calibration".into()), 37, [7; 16])
            .await
            .unwrap();
        let rejected = handle
            .start_with_config("surface-bad-config", surface_request.clone())
            .await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        assert!(rejected.message.contains("calibration"));
        assert_eq!(rejected.request_id, "surface-bad-config");
        handle
            .configure_surface_source(Ok(surface_config("map")), 37, [7; 16])
            .await
            .unwrap();
        let mut coverage = surface_request;
        coverage.stop_conditions.max_coverage_pct = Some(80.0);
        let rejected = handle.start_with_config("surface-coverage", coverage).await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        assert!(rejected.message.contains("known volume"));
    }

    #[tokio::test]
    async fn surface_no_frontier_uses_stable_completion_without_two_dimensional_coverage() {
        let (handle, sink) = configured_handle();
        let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 1.0, y: 1.0 },
            Point2 { x: 2.0, y: 1.0 },
            Point2 { x: 2.0, y: 2.0 },
            Point2 { x: 1.0, y: 2.0 },
        ])
        .unwrap();
        handle
            .start_with_config(
                "surface-no-frontier",
                ExploreStartConfig {
                    boundary: Some(boundary),
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        let waiting = handle
            .tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY))
            .await;
        assert_eq!(waiting.status, STATUS_PENDING, "{}", waiting.message);
        assert_eq!(waiting.selected_source, "surface");
        handle.inner.lock().await.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
        let done = handle
            .tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY))
            .await;
        assert_eq!(done.status, STATUS_SUCCEEDED, "{}", done.message);
        assert_eq!(done.last_stop_reason, "no_frontier_timeout");
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn surface_computation_completion_never_refreshes_expired_receive_time() {
        let (handle, _) = configured_handle();
        handle
            .configure_surface_source(Ok(surface_config("map")), 37, [7; 16])
            .await
            .unwrap();
        let message = surface_message("map", 1);
        let received = now_monotonic_nanos();
        let ticket = handle
            .begin_surface_map_update(&message, received)
            .await
            .unwrap()
            .unwrap();
        let snapshot = Arc::new(
            SurfaceSnapshot::from_points(
                &schema::navigation_map_3d_points(&message).unwrap(),
                surface_config("map").geometry,
                message.source_voxel_size,
            )
            .unwrap(),
        );
        tokio::time::sleep(DEFAULT_COSTMAP_RECEIPT_TTL + Duration::from_millis(20)).await;
        assert!(
            handle
                .finish_surface_map_update(ticket, Ok(snapshot.clone()))
                .await
        );
        assert_eq!(
            handle.inner.lock().await.costmap_received_monotonic_ns,
            received
        );
        handle.start("surface-stale", "prefer_wavefront").await;
        let waiting = handle
            .tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY))
            .await;
        assert_eq!(waiting.status, STATUS_PENDING);
        assert!(waiting.message.contains("surface map input stale"));
    }

    #[tokio::test]
    async fn surface_continuous_same_domain_updates_install_progress_without_crossing_clear() {
        let (handle, _) = configured_handle();
        handle
            .configure_surface_source(Ok(surface_config("map")), 37, [7; 16])
            .await
            .unwrap();
        let first = surface_message("map", 1);
        let received = now_monotonic_nanos();
        let first_ticket = handle
            .begin_surface_map_update(&first, received)
            .await
            .unwrap()
            .unwrap();
        let second = surface_message("map", 2);
        let second_ticket = handle
            .begin_surface_map_update(&second, now_monotonic_nanos())
            .await
            .unwrap()
            .unwrap();
        let snapshot = Arc::new(
            SurfaceSnapshot::from_points(
                &schema::navigation_map_3d_points(&first).unwrap(),
                surface_config("map").geometry,
                first.source_voxel_size,
            )
            .unwrap(),
        );
        assert!(
            handle
                .finish_surface_map_update(first_ticket.clone(), Ok(snapshot.clone()))
                .await,
            "同域新输入排队不能饿死正在完成的有效图"
        );
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&first.committed_map));
            assert_eq!(guard.costmap_received_monotonic_ns, received);
            assert!(Arc::ptr_eq(
                &guard
                    .surface_mode
                    .as_ref()
                    .unwrap()
                    .receiver
                    .snapshot()
                    .unwrap()
                    .unwrap(),
                &snapshot,
            ));
        }
        let replacement = Arc::new(
            SurfaceSnapshot::from_points(
                &schema::navigation_map_3d_points(&second).unwrap(),
                surface_config("map").geometry,
                second.source_voxel_size,
            )
            .unwrap(),
        );
        let second_received = second_ticket.received_monotonic_ns;
        assert!(
            handle
                .finish_surface_map_update(second_ticket, Ok(replacement.clone()))
                .await
        );
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&second.committed_map));
            assert_eq!(guard.costmap_received_monotonic_ns, second_received);
            assert!(Arc::ptr_eq(
                &guard
                    .surface_mode
                    .as_ref()
                    .unwrap()
                    .receiver
                    .snapshot()
                    .unwrap()
                    .unwrap(),
                &replacement,
            ));
        }
        assert!(
            !handle
                .finish_surface_map_update(first_ticket, Ok(snapshot.clone()))
                .await
        );
        let third = surface_message("map", 3);
        let third_ticket = handle
            .begin_surface_map_update(&third, now_monotonic_nanos())
            .await
            .unwrap()
            .unwrap();
        let mut clear = surface_message("map", 4);
        clear.cleared = true;
        clear.cloud.width = 0;
        clear.cloud.row_step = 0;
        clear.cloud.data.clear();
        handle
            .begin_surface_map_update(&clear, now_monotonic_nanos())
            .await
            .unwrap();
        let fifth = surface_message("map", 5);
        let fifth_ticket = handle
            .begin_surface_map_update(&fifth, now_monotonic_nanos())
            .await
            .unwrap()
            .unwrap();
        assert!(
            !handle
                .finish_surface_map_update(third_ticket, Ok(snapshot.clone()))
                .await,
            "即使清空后同域新图已排队，清空前的旧图也不能恢复"
        );
        assert!(
            handle
                .finish_surface_map_update(fifth_ticket, Ok(snapshot))
                .await
        );
        assert_eq!(
            handle.inner.lock().await.committed_map.as_ref(),
            Some(&fifth.committed_map)
        );
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

    fn rollover_recovery_request(
        snapshot: &ExploreRuntimeSnapshot,
    ) -> ExplorationRolloverRecoveryRequest {
        ExplorationRolloverRecoveryRequest {
            request_id: snapshot.request_id.clone(),
            exploration_generation: snapshot.exploration_generation,
            child_lease: snapshot.child_lease.clone().expect("retained child lease"),
            expected_navigation_runtime_generation: 41,
            observed_navigation_runtime_generation: 42,
            guard_proof: ExplorationRecoveryGuardProof {
                stop_operation_id: "exploration-rollover-stop:fixture".to_string(),
                recovery_correlation: "exploration-rollover:fixture".to_string(),
                motion_admission: crate::nav_runtime::test_motion_admission(),
                zero_ack_id: "global-zero-fixture".to_string(),
                zero_applied_monotonic_ns: now_monotonic_nanos(),
                recovery_fence_token: RecoveryFenceToken::from_wire(vec![9; 32]),
            },
        }
    }

    fn global_stop_reconciliation_request(
        snapshot: &ExploreRuntimeSnapshot,
    ) -> ExplorationGlobalStopReconciliationRequest {
        let reconciliation = &snapshot.global_stop_reconciliation;
        assert!(reconciliation.eligible, "fixture must be eligible");
        let motion_admission = reconciliation
            .motion_admission
            .expect("original motion admission");
        ExplorationGlobalStopReconciliationRequest {
            request_id: snapshot.request_id.clone(),
            exploration_generation: snapshot.exploration_generation,
            child_lease: snapshot.child_lease.clone().expect("retained child lease"),
            cancel_generation: reconciliation.cancel_generation,
            recovery_correlation: reconciliation.recovery_correlation.clone(),
            guard_proof: ExplorationGlobalStopGuardProof {
                stop_operation_id: "hold:exploration-global-stop-fixture".to_string(),
                recovery_correlation: reconciliation.recovery_correlation.clone(),
                original_motion_admission: motion_admission,
                held_guard_generation: motion_admission.guard_generation,
                held_guard_boot_nonce: motion_admission.guard_boot_nonce,
                held_motion_stop_generation: motion_admission
                    .motion_stop_generation
                    .saturating_add(1),
                zero_ack_id: "zero:exploration-global-stop-fixture".to_string(),
                zero_applied_monotonic_ns: now_monotonic_nanos(),
                recovery_fence_token: RecoveryFenceToken::from_wire(vec![9; 32]),
            },
        }
    }

    async fn global_stop_reconciliation_fixture(
        request_id: &str,
    ) -> (
        ExploreRuntimeHandle,
        String,
        ChildNavigationLease,
        ExploreRuntimeSnapshot,
    ) {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .expect("frontier costmap");
        handle.start(request_id, "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        assert_eq!(handle.stop(request_id).await.status, STATUS_STOPPING);
        let failed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "exact child cancellation failed".to_string(),
                },
            )
            .await
            .expect("exact cancel failure");
        assert!(failed.global_stop_reconciliation.eligible);
        (handle, active.child_task_id, lease, failed)
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
    async fn explicit_costmap_input_requires_map_frame() {
        let (handle, _) = configured_handle();
        let mut odom = frontier_costmap(1);
        odom.frame_id = "odom".to_string();
        assert!(handle.ingest_costmap_snapshot(odom).await.is_err());
        assert!(handle
            .ingest_costmap_snapshot(frontier_costmap(2))
            .await
            .unwrap());
        assert!(!handle
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .unwrap());
    }

    #[tokio::test]
    async fn configured_robot_map_frame_drives_default_costmap_and_pose() {
        let handle = ExploreRuntimeHandle::with_wavefront_map_frame("robot/map")
            .expect("configured map frame");
        let cloned = handle.clone();
        assert_eq!(handle.expected_map_frame(), "robot/map");
        assert_eq!(cloned.expected_map_frame(), "robot/map");
        assert!(Arc::ptr_eq(
            &handle.expected_map_frame,
            &cloned.expected_map_frame
        ));

        let sink = Arc::new(RecordingNavSink::default());
        handle.set_navigation_sink(sink.clone());
        let mut configured_costmap = frontier_costmap(1);
        configured_costmap.frame_id = "robot/map".to_string();
        assert!(
            ingest_committed_costmap_fixture(&handle, configured_costmap, 1)
                .await
                .expect("configured committed costmap")
        );
        assert_eq!(
            handle
                .inner
                .lock()
                .await
                .costmap_input
                .as_ref()
                .expect("configured costmap")
                .frame_id,
            "robot/map"
        );
        assert_eq!(
            handle
                .start("req-robot-map", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let readiness = handle.readiness(true).await;
        assert!(readiness.ready, "{}", readiness.reason);

        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 1);
        assert_eq!(submitted[0].goal.target.frame_id, NavFrameId::Map);
    }

    #[tokio::test]
    async fn configured_robot_map_frame_rejects_literal_map_inputs() {
        for invalid in ["", " robot/map", "/robot/map", "robot//map", "robot/map-1"] {
            assert!(
                ExploreRuntimeHandle::with_wavefront_map_frame(invalid).is_err(),
                "invalid configured frame accepted: {invalid:?}"
            );
        }

        let handle = ExploreRuntimeHandle::with_wavefront_map_frame("robot/map")
            .expect("configured map frame");
        handle.set_navigation_sink(Arc::new(RecordingNavSink::default()));
        let literal_map = frontier_costmap(1);
        let error = handle
            .ingest_costmap_snapshot(literal_map.clone())
            .await
            .expect_err("literal map must not satisfy configured robot/map");
        assert!(error.contains("costmap frame=robot/map"), "{error}");

        handle.inner.lock().await.costmap_input = Some(literal_map);
        let readiness = handle.readiness(true).await;
        assert!(!readiness.costmap_input_ready);
        assert!(!readiness.ready);

        let mut slash_configured_costmap = frontier_costmap(2);
        slash_configured_costmap.frame_id = "/robot/map".to_string();
        let slash_error = handle
            .ingest_costmap_snapshot(slash_configured_costmap)
            .await
            .expect_err("slash-prefixed frame must not satisfy raw-exact robot/map");
        assert!(
            slash_error.contains("costmap frame=robot/map"),
            "{slash_error}"
        );

        let mut configured_costmap = frontier_costmap(2);
        configured_costmap.frame_id = "robot/map".to_string();
        assert!(
            ingest_committed_costmap_fixture(&handle, configured_costmap, 2)
                .await
                .expect("configured costmap")
        );
        assert_eq!(
            handle
                .start("req-wrong-pose", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let failed = handle
            .tick_with_pose(ExploreRobotPose {
                x: 2.0,
                y: 2.0,
                z: 0.0,
                orientation: nuwax_nav_types_messages::geometry::Quaternion::IDENTITY,
                frame_id: "map".to_string(),
                source_timestamp_ns: now_unix_nanos(),
                received_monotonic_ns: now_monotonic_nanos(),
            })
            .await;
        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.message.contains("pose frame=robot/map"));
    }

    #[tokio::test]
    async fn tick_dispatches_real_child_and_feedback_advances_session() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn preflight_readiness_recovers_without_stranding_selected_goal() {
        for missing in ["committed_map", "cancel_generation", "map_name"] {
            let (handle, sink) = configured_handle();
            if missing == "committed_map" {
                handle
                    .ingest_costmap_snapshot(frontier_costmap(1))
                    .await
                    .unwrap();
            } else {
                ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
                    .await
                    .unwrap();
            }
            if missing == "cancel_generation" {
                *sink.cancel_snapshot.lock().unwrap() = Some(Err("取消代次暂不可读".to_string()));
            }
            if missing == "map_name" {
                *sink.map_name_error.lock().unwrap() = Some("地图名称暂不可读".to_string());
            }
            handle.start("req-preflight", "prefer_wavefront").await;
            let waiting = handle.tick((2.0, 2.0)).await;
            assert_eq!(waiting.status, STATUS_PENDING, "{missing}");
            assert!(sink.submitted.lock().unwrap().is_empty());

            ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
                .await
                .unwrap();
            *sink.cancel_snapshot.lock().unwrap() = None;
            *sink.map_name_error.lock().unwrap() = None;
            let recovered = handle.tick((2.0, 2.0)).await;
            assert_eq!(
                recovered.status, STATUS_ACTIVE,
                "{missing}: {}",
                recovered.message
            );
            let submitted = sink.submitted.lock().unwrap();
            assert_eq!(submitted.len(), 1);
            assert_eq!(submitted[0].request_id, "req-preflight.nav.1");
        }
    }

    #[tokio::test]
    async fn invalid_goal_intent_finishes_without_stranded_child() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        *sink.cancel_snapshot.lock().unwrap() = Some(Ok(u64::MAX));
        handle.start("req-invalid-intent", "prefer_wavefront").await;
        let failed = handle.tick((2.0, 2.0)).await;
        assert_eq!(failed.status, STATUS_FAILED, "{}", failed.message);
        assert!(failed.message.contains("cancel generation"));
        assert!(failed.child_task_id.is_empty());
        assert!(!handle.is_active().await);
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn default_no_frontier_stability_window_finishes_with_fresh_map_input() {
        let (handle, sink) = configured_handle();
        let mut costmap = frontier_costmap(1);
        costmap.grid.cells.fill(OccupancyCell::Free);
        ingest_committed_costmap_fixture(&handle, costmap.clone(), 1)
            .await
            .unwrap();
        handle.start("req-no-frontier", "prefer_wavefront").await;
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_PENDING);
        handle.inner.lock().await.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
        ingest_committed_costmap_fixture(&handle, costmap, 2)
            .await
            .unwrap();
        let completed = handle.tick((2.0, 2.0)).await;
        assert_eq!(completed.status, STATUS_SUCCEEDED);
        assert_eq!(completed.last_stop_reason, "no_frontier_timeout");
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn unavailable_or_stale_costmap_never_completes_no_frontier_window() {
        let (handle, sink) = configured_handle();
        handle.start("req-no-input", "prefer_wavefront").await;
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_PENDING);
        assert!(handle.inner.lock().await.no_frontier_since.is_none());

        let mut costmap = frontier_costmap(1);
        costmap.grid.cells.fill(OccupancyCell::Free);
        handle
            .ingest_costmap_snapshot(costmap.clone())
            .await
            .unwrap();
        let uncommitted = handle.tick((2.0, 2.0)).await;
        assert!(uncommitted
            .message
            .contains("committed map identity unavailable"));
        assert!(handle.inner.lock().await.no_frontier_since.is_none());

        ingest_committed_costmap_fixture(&handle, costmap.clone(), 1)
            .await
            .unwrap();
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_PENDING);
        {
            let mut guard = handle.inner.lock().await;
            guard.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
            // 模拟地图输入过期，随后仅查询状态与调度，不接收新地图。
            guard.costmap_received_monotonic_ns = 0;
        }
        let stale = handle.tick((2.0, 2.0)).await;
        assert_eq!(stale.status, STATUS_PENDING);
        assert!(stale.message.contains("costmap input stale"));
        assert!(stale.last_stop_reason.is_empty());
        assert!(handle.inner.lock().await.no_frontier_since.is_none());
        assert!(sink.submitted.lock().unwrap().is_empty());
        // 同一有效快照的实际续收恢复输入，但仍必须重新积累稳定窗口。
        ingest_committed_costmap_fixture(&handle, costmap, 1)
            .await
            .unwrap();
        let resumed = handle.tick((2.0, 2.0)).await;
        assert_eq!(resumed.status, STATUS_PENDING);
        assert!(resumed.last_stop_reason.is_empty());
    }

    #[tokio::test]
    async fn pending_navigation_readiness_does_not_count_as_no_frontier() {
        let (handle, sink) = configured_handle();
        sink.submit_errors.lock().unwrap().extend(
            (0..2).map(|_| {
                ChildNavigationSubmitError::RetryableReadiness("等待运动执行端".to_string())
            }),
        );
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-pending-goal", "prefer_wavefront").await;
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_PENDING);
        let pending = handle.tick((2.0, 2.0)).await;
        assert!(pending
            .message
            .contains("waiting for child navigation readiness"));
        assert!(pending.last_stop_reason.is_empty());
        assert!(handle.inner.lock().await.no_frontier_since.is_none());
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_ACTIVE);
    }

    #[test]
    fn coverage_has_no_value_when_boundary_does_not_overlap_map() {
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 20.0, y: 20.0 },
            Point2 { x: 21.0, y: 20.0 },
            Point2 { x: 21.0, y: 21.0 },
            Point2 { x: 20.0, y: 21.0 },
        ])
        .unwrap();
        assert_eq!(
            coverage_pct(&frontier_costmap(1).grid, Some(&boundary)),
            None
        );
    }

    #[tokio::test]
    async fn coverage_stop_counts_only_cells_inside_requested_boundary() {
        for inside_known in [false, true] {
            let (handle, _) = configured_handle();
            let mut costmap = frontier_costmap(1);
            costmap.grid.cells.fill(if inside_known {
                OccupancyCell::Unknown
            } else {
                OccupancyCell::Free
            });
            for y in 0..4 {
                for x in 0..4 {
                    costmap.grid.set(
                        x,
                        y,
                        if inside_known {
                            OccupancyCell::Free
                        } else {
                            OccupancyCell::Unknown
                        },
                    );
                }
            }
            let boundary = ExplorationBoundary::from_vertices(vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 2.0, y: 0.0 },
                Point2 { x: 2.0, y: 2.0 },
                Point2 { x: 0.0, y: 2.0 },
            ])
            .unwrap();
            ingest_committed_costmap_fixture(&handle, costmap, 1)
                .await
                .unwrap();
            handle
                .start_with_config(
                    "req-coverage",
                    ExploreStartConfig {
                        boundary: Some(boundary),
                        stop_conditions: ExploreStopConditions {
                            max_coverage_pct: Some(50.0),
                            ..ExploreStopConditions::default()
                        },
                        motion_admission: crate::nav_runtime::test_motion_admission(),
                        ..ExploreStartConfig::default()
                    },
                )
                .await;
            let result = handle
                .tick(if inside_known { (1.0, 1.0) } else { (3.0, 3.0) })
                .await;
            assert_eq!(
                result.last_stop_reason == "coverage_reached",
                inside_known,
                "{}",
                result.message
            );
        }
    }

    #[tokio::test]
    async fn retryable_readiness_wait_does_not_consume_navigation_failures() {
        let (handle, sink) = configured_handle();
        sink.submit_errors.lock().unwrap().extend((0..10).map(|_| {
            ChildNavigationSubmitError::RetryableReadiness(
                "MOTION_SINK_NOT_READY: test navigation chain is recovering".to_string(),
            )
        }));
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
                ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
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
            assert!(submitted
                .iter()
                .all(|request| request.parent_request_id == "req-readiness"));
        }

        let diagnostics = handle.diagnostics().await;
        assert!(diagnostics.sources[0].last_error.is_empty());
    }

    #[tokio::test]
    async fn session_invalidated_dispatch_rejection_finishes_immediately() {
        let (handle, sink) = configured_handle();
        sink.submit_errors.lock().unwrap().push_back(
            ChildNavigationSubmitError::SessionInvalidated(
                "active map name changed before goal lease admission".to_string(),
            ),
        );
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-invalidated", "prefer_wavefront").await;

        let failed = handle.tick((2.0, 2.0)).await;
        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.message.contains("active map name changed"));
        assert!(failed.child_task_id.is_empty());
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn child_dispatch_rejects_nonrecoverable_lease_identity() {
        for invalid_lease in [
            ChildNavigationLease {
                session_id: "invalid-owner".to_string(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                parent_request_id: "req-invalid-lease".to_string(),
                goal_generation: 1,
                producer_runtime_generation: 11,
                goal_payload_sha256: [7; 32],
            },
            ChildNavigationLease {
                session_id: "invalid-generation".to_string(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                parent_request_id: "req-invalid-lease".to_string(),
                goal_generation: 1,
                producer_runtime_generation: 0,
                goal_payload_sha256: [7; 32],
            },
        ] {
            let (handle, sink) = configured_handle();
            *sink.lease_override.lock().unwrap() = Some(invalid_lease);
            ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
                .await
                .unwrap();
            handle.start("req-invalid-lease", "prefer_wavefront").await;

            let rejected = handle.tick((2.0, 2.0)).await;
            assert_eq!(rejected.status, STATUS_PENDING);
            assert!(rejected.child_task_id.is_empty());
            assert!(rejected
                .message
                .contains("child navigation dispatch failed"));
        }
    }

    #[tokio::test]
    async fn stop_retains_child_until_correlated_zeroed_proof() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        let mut wrong = lease.clone();
        wrong.producer_runtime_generation += 1;
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn cancel_error_retains_exact_receipt_for_late_zeroed_reconciliation() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start("req-cancel-error-receipt", "prefer_wavefront")
            .await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        *sink.cancel_error.lock().unwrap() = Some("uncertain cancel publish error".to_string());
        *sink.cancel_recovery.lock().unwrap() = Some(ChildNavigationCancelRecovery {
            ticket: ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: 37,
            },
            failure: Some("CANCEL_DISPATCH_FAILED".to_string()),
        });

        let failed = handle.stop("req-cancel-error-receipt").await;
        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.message.contains("uncertain cancel publish error"));
        assert_eq!(failed.child_task_id, active.child_task_id);
        assert!(failed.global_stop_reconciliation.eligible);
        assert!(handle.blocks_mapping_mutation().await);

        let released = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 37)),
            )
            .await
            .expect("late exact ZEROED after uncertain publish error");
        assert_eq!(released.status, STATUS_FAILED);
        assert!(released.child_task_id.is_empty());
        assert!(released.ownership_reconciliation_receipt.is_some());
        assert!(!handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn cancel_dispatch_timeout_is_retryable_and_retains_safety_ownership() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start("req-cancel-timeout-retry", "prefer_wavefront")
            .await;
        let active = handle.tick((2.0, 2.0)).await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.cancel_release.lock().unwrap() = Some(release);

        let failed = handle.stop("req-cancel-timeout-retry").await;

        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.message.contains("dispatch timed out"));
        assert_eq!(failed.child_task_id, active.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);

        *sink.cancel_release.lock().unwrap() = None;
        let retried = handle.stop("req-cancel-timeout-retry").await;
        assert_eq!(retried.status, STATUS_STOPPING);
        assert!(retried.message.contains("CANCEL_RETRY_PENDING"));
        assert_eq!(retried.child_task_id, active.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn nav_runtime_rollover_requires_durable_guard_fence_before_releasing_child() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start("req-rollover-recovery", "prefer_wavefront")
            .await;
        handle.tick((2.0, 2.0)).await;
        *sink.cancel_error.lock().unwrap() = Some("old runtime lost child".to_string());

        let retained = handle.stop("req-rollover-recovery").await;
        assert_eq!(retained.status, STATUS_FAILED);
        assert!(retained.rollover_recovery_eligible);
        assert!(!retained.child_task_id.is_empty());
        assert!(handle.blocks_mapping_mutation().await);

        let mut incomplete = rollover_recovery_request(&retained);
        incomplete.guard_proof.recovery_fence_token.clear();
        assert!(matches!(
            handle.recover_after_navigation_rollover(incomplete).await,
            Err(ExplorationRecoveryError::InvalidArgument(_))
        ));
        assert_eq!(handle.snapshot().child_task_id, retained.child_task_id);

        let request = rollover_recovery_request(&retained);
        let recovered = handle
            .recover_after_navigation_rollover(request.clone())
            .await
            .expect("exact durable recovery proof");
        assert_eq!(recovered.status, STATUS_FAILED);
        assert_eq!(recovered.last_stop_reason, "runtime_rollover_aborted_safe");
        assert!(recovered.child_task_id.is_empty());
        assert!(!recovered.rollover_recovery_eligible);
        assert!(!handle.blocks_mapping_mutation().await);

        let replay = handle
            .replay_completed_rollover_recovery(&request)
            .await
            .expect("exact replay probe is idempotent")
            .expect("completed recovery snapshot");
        assert_eq!(replay.request_id, recovered.request_id);
        assert_eq!(replay.last_stop_reason, recovered.last_stop_reason);

        let mut conflicting = request;
        conflicting
            .guard_proof
            .recovery_correlation
            .push_str(":other");
        assert!(matches!(
            handle
                .replay_completed_rollover_recovery(&conflicting)
                .await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
    }

    #[tokio::test]
    async fn rollover_recovery_rejects_wrong_child_and_nonfailed_cancellation() {
        let (handle, _sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-rollover-wrong", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;

        let stopping = handle.stop("req-rollover-wrong").await;
        assert_eq!(stopping.status, STATUS_STOPPING);
        assert!(!stopping.rollover_recovery_eligible);
        assert!(matches!(
            handle
                .recover_after_navigation_rollover(rollover_recovery_request(&stopping))
                .await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));

        let failed = handle
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: terminal_proof(&lease),
                    cancel_generation: 1,
                    reason: "new runtime has no old child".to_string(),
                },
            )
            .await
            .unwrap();
        assert!(failed.rollover_recovery_eligible);
        let mut wrong = rollover_recovery_request(&failed);
        wrong.child_lease.goal_generation += 1;
        assert!(matches!(
            handle.recover_after_navigation_rollover(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        assert_eq!(handle.snapshot().child_task_id, failed.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn global_stop_reconciliation_rejects_wrong_identity_and_guard_proof() {
        let (handle, _, _, retained) =
            global_stop_reconciliation_fixture("req-global-hostile").await;
        let exact = global_stop_reconciliation_request(&retained);

        let mut wrong = exact.clone();
        wrong.child_lease.goal_generation += 1;
        assert!(matches!(
            handle.reconcile_after_global_stop(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        let mut wrong = exact.clone();
        wrong.exploration_generation += 1;
        assert!(matches!(
            handle.reconcile_after_global_stop(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        let mut wrong = exact.clone();
        wrong.cancel_generation += 1;
        assert!(matches!(
            handle.reconcile_after_global_stop(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        let mut wrong = exact.clone();
        wrong.recovery_correlation.push_str(":wrong");
        wrong.guard_proof.recovery_correlation = wrong.recovery_correlation.clone();
        assert!(matches!(
            handle.reconcile_after_global_stop(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        let mut wrong = exact;
        wrong.guard_proof.held_motion_stop_generation = wrong
            .guard_proof
            .original_motion_admission
            .motion_stop_generation;
        assert!(matches!(
            handle.reconcile_after_global_stop(wrong).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
        assert_eq!(handle.snapshot().child_task_id, retained.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn global_stop_reconciliation_persists_exact_response_loss_replay() {
        let (handle, _, _, retained) =
            global_stop_reconciliation_fixture("req-global-replay").await;
        let exact = global_stop_reconciliation_request(&retained);
        let committed = handle
            .reconcile_after_global_stop(exact.clone())
            .await
            .expect("verified held-fence commit");
        let receipt = committed
            .ownership_reconciliation_receipt
            .as_ref()
            .expect("completed reconciliation receipt");
        assert_eq!(receipt.cause, GLOBAL_STOP_RECONCILIATION_CAUSE);
        assert_eq!(receipt.recovery_correlation, exact.recovery_correlation);
        assert_eq!(receipt.parent_request_id, exact.request_id);
        assert_eq!(receipt.exploration_generation, exact.exploration_generation);
        assert_eq!(receipt.cancel_generation, exact.cancel_generation);
        assert_eq!(
            receipt.child_lease_fingerprint,
            child_lease_fingerprint(&exact.child_lease)
        );
        assert_eq!(receipt.commit_disposition, GLOBAL_STOP_COMMIT_DISPOSITION);

        let replay = handle
            .replay_completed_global_stop_reconciliation(&exact)
            .await
            .expect("response-loss replay probe")
            .expect("completed response must persist");
        assert_eq!(
            replay.ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
        let direct_replay = handle
            .reconcile_after_global_stop(exact.clone())
            .await
            .expect("exact commit replay");
        assert_eq!(
            direct_replay.ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );

        let mut conflicting = exact;
        conflicting
            .guard_proof
            .zero_ack_id
            .push_str(":different-proof");
        assert!(matches!(
            handle
                .replay_completed_global_stop_reconciliation(&conflicting)
                .await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
    }

    #[tokio::test]
    async fn global_stop_response_loss_replays_after_runtime_restart() {
        let (handle, _, _, retained) =
            global_stop_reconciliation_fixture("req-global-restart").await;
        let journal_path = handle.reconciliation_journal_path();
        let mut exact = global_stop_reconciliation_request(&retained);
        exact.guard_proof.recovery_fence_token = RecoveryFenceToken::from_wire(vec![0x5a; 32]);
        let committed = handle
            .reconcile_after_global_stop(exact.clone())
            .await
            .expect("durable global-stop commit");
        let persisted = fs::read_to_string(&journal_path).expect("durable journal JSON");
        assert!(!persisted.contains("90,90,90"));
        assert!(!persisted.contains("recovery_fence_token"));
        drop(handle);

        let restarted = ExploreRuntimeHandle::default_with_wavefront();
        restarted.set_reconciliation_journal_path_for_tests(journal_path);
        restarted
            .restore_latest_durable_reconciliation_snapshot()
            .expect("startup journal recovery");
        assert_eq!(
            restarted.snapshot().ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
        let replay = restarted
            .replay_completed_global_stop_reconciliation(&exact)
            .await
            .expect("restart replay query")
            .expect("durable restart replay");
        assert_eq!(
            replay.ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
        assert_eq!(
            restarted
                .latest_durable_reconciliation_snapshot()
                .expect("startup recovery")
                .expect("unacknowledged durable receipt")
                .ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
    }

    #[tokio::test]
    async fn new_start_does_not_evict_unacknowledged_reconciliation() {
        let (handle, _, _, retained) =
            global_stop_reconciliation_fixture("req-global-unacked").await;
        let exact = global_stop_reconciliation_request(&retained);
        let committed = handle
            .reconcile_after_global_stop(exact.clone())
            .await
            .expect("durable global-stop commit");
        assert_eq!(
            handle
                .start("req-after-unacked", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let replay = handle
            .replay_completed_global_stop_reconciliation(&exact)
            .await
            .expect("old durable replay after new start")
            .expect("unacknowledged receipt must remain");
        assert_eq!(
            replay.ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
        let receipt = committed
            .ownership_reconciliation_receipt
            .as_ref()
            .expect("committed receipt");
        assert!(handle
            .acknowledge_durable_reconciliation(
                &receipt.recovery_correlation,
                &receipt.child_lease_fingerprint,
            )
            .expect("durable acknowledgement"));
        assert!(handle
            .latest_durable_reconciliation_snapshot()
            .expect("post-ack journal read")
            .is_none());
        assert!(handle
            .replay_completed_global_stop_reconciliation(&exact)
            .await
            .expect("acknowledged exact replay remains until bounded eviction")
            .is_some());
    }

    #[tokio::test]
    async fn journal_write_failure_retains_child_for_global_and_exact_zero() {
        for (request_id, global) in [
            ("req-global-write-failure", true),
            ("req-zero-write-failure", false),
        ] {
            let (handle, child_request_id, lease, retained) =
                global_stop_reconciliation_fixture(request_id).await;
            let blocked_parent = reconciliation_test_path("blocked-parent");
            fs::create_dir_all(blocked_parent.parent().unwrap()).unwrap();
            fs::write(&blocked_parent, b"not a directory").unwrap();
            handle.set_reconciliation_journal_path_for_tests(blocked_parent.join("journal.json"));

            if global {
                assert!(matches!(
                    handle
                        .reconcile_after_global_stop(global_stop_reconciliation_request(&retained))
                        .await,
                    Err(ExplorationRecoveryError::FailedPrecondition(_))
                ));
            } else {
                assert!(handle
                    .report_navigation_result(
                        &child_request_id,
                        ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
                    )
                    .await
                    .is_err());
            }
            let unchanged = handle.snapshot();
            assert_eq!(unchanged.child_task_id, retained.child_task_id);
            assert!(unchanged.global_stop_reconciliation.eligible);
            assert!(handle.blocks_mapping_mutation().await);
        }
    }

    #[tokio::test]
    async fn rejected_zero_proof_never_writes_ownership_journal() {
        let (handle, child_request_id, lease, _) =
            global_stop_reconciliation_fixture("req-zero-no-write").await;
        let journal_path = handle.reconciliation_journal_path();
        let mut invalid = zero_proof(&lease, 1);
        invalid.zero_ack_id.clear();
        assert!(handle
            .report_navigation_result(&child_request_id, ChildNavigationResult::Cancelled(invalid),)
            .await
            .is_err());
        assert!(!journal_path.exists());
    }

    #[tokio::test]
    async fn late_exact_zero_persists_first_winner_and_preserves_stop_reason() {
        let (handle, child_request_id, lease, retained) =
            global_stop_reconciliation_fixture("req-zero-ledger").await;
        let global_request = global_stop_reconciliation_request(&retained);
        let released = handle
            .report_navigation_result(
                &child_request_id,
                ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
            )
            .await
            .expect("late exact zero durable commit");
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        let receipt = released
            .ownership_reconciliation_receipt
            .expect("exact zero reconciliation receipt");
        assert_eq!(receipt.cause, EXACT_CHILD_ZEROED_CAUSE);
        assert_eq!(receipt.commit_disposition, EXACT_CHILD_ZEROED_DISPOSITION);
        let journal = read_durable_ownership_journal(&handle.reconciliation_journal_path())
            .expect("durable exact-zero journal");
        assert_eq!(journal.entries.len(), 1);
        assert_eq!(
            journal.entries[0].kind,
            DurableOwnershipTransitionKind::ExactChildZeroed
        );
        assert!(matches!(
            handle.reconcile_after_global_stop(global_request).await,
            Err(ExplorationRecoveryError::FailedPrecondition(_))
        ));
    }

    #[test]
    fn recovery_fence_token_debug_is_redacted() {
        let token = RecoveryFenceToken::from_wire(vec![0x5a; 32]);
        let rendered = format!("{token:?}");
        assert_eq!(rendered, "RecoveryFenceToken([REDACTED])");
        assert!(!rendered.contains("90"));
        assert!(!rendered.contains("5a"));
    }

    #[test]
    fn global_stop_request_debug_redacts_recovery_fence_token() {
        let retained = ExploreRuntimeSnapshot {
            request_id: "debug-parent".to_string(),
            exploration_generation: 1,
            child_lease: Some(ChildNavigationLease {
                session_id: "debug-parent.nav.1".to_string(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                parent_request_id: "debug-parent".to_string(),
                goal_generation: 1,
                producer_runtime_generation: 1,
                goal_payload_sha256: [7; 32],
            }),
            global_stop_reconciliation: ExplorationGlobalStopReconciliationSnapshot {
                eligible: true,
                cancel_generation: 1,
                recovery_correlation:
                    "exploration-global-stop:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
                        .to_string(),
                motion_admission: Some(crate::nav_runtime::test_motion_admission()),
            },
            ..ExploreRuntimeSnapshot::default()
        };
        let mut request = global_stop_reconciliation_request(&retained);
        request.guard_proof.recovery_fence_token = RecoveryFenceToken::from_wire(vec![0x5a; 32]);
        let rendered = format!("{request:?}");
        assert!(rendered.contains("RecoveryFenceToken([REDACTED])"));
        assert!(!rendered.contains("90, 90, 90"));
        assert!(!rendered.contains(&"5a".repeat(32)));
    }

    fn durable_test_transition(index: usize) -> DurableOwnershipTransition {
        let correlation = format!("exploration-global-stop:{index:064x}");
        DurableOwnershipTransition {
            kind: DurableOwnershipTransitionKind::GlobalStop,
            recovery_correlation: correlation.clone(),
            request_fingerprint: format!("{:064x}", index + 1),
            receipt: ExplorationOwnershipReconciliationReceipt {
                cause: GLOBAL_STOP_RECONCILIATION_CAUSE.to_string(),
                recovery_correlation: correlation,
                parent_request_id: format!("parent-{index}"),
                exploration_generation: index as u64 + 1,
                child_lease_fingerprint: format!("{:064x}", index + 2),
                cancel_generation: 1,
                commit_disposition: GLOBAL_STOP_COMMIT_DISPOSITION.to_string(),
            },
            snapshot: DurableReconciliationSnapshot {
                status: STATUS_FAILED.to_string(),
                message: "durable fixture".to_string(),
                request_id: format!("parent-{index}"),
                selected_source: "wavefront".to_string(),
                last_stop_reason: "global_stop_reconciled_safe".to_string(),
                frontier_count: 0,
                revision: 1,
                exploration_generation: index as u64 + 1,
            },
            committed_unix_ns: index as u64 + 1,
            acknowledged: false,
        }
    }

    fn write_raw_test_journal(path: &Path, bytes: &[u8]) {
        let parent = path.parent().unwrap();
        fs::create_dir_all(parent).unwrap();
        fs::set_permissions(parent, fs::Permissions::from_mode(0o700)).unwrap();
        let mut file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .mode(0o600)
            .open(path)
            .unwrap();
        file.write_all(bytes).unwrap();
        file.sync_all().unwrap();
    }

    #[test]
    fn journal_rejects_unknown_fields_oversize_and_inconsistent_semantics() {
        let transition = durable_test_transition(101);
        let journal = DurableOwnershipJournal {
            schema_version: RECONCILIATION_JOURNAL_SCHEMA_VERSION,
            entries: vec![transition.clone()],
        };

        let unknown_path = reconciliation_test_path("unknown-field");
        let mut unknown = serde_json::to_value(&journal).unwrap();
        unknown
            .as_object_mut()
            .unwrap()
            .insert("unexpected".to_string(), serde_json::json!(true));
        write_raw_test_journal(&unknown_path, &serde_json::to_vec(&unknown).unwrap());
        assert!(read_durable_ownership_journal(&unknown_path).is_err());

        let oversized_path = reconciliation_test_path("oversized");
        write_raw_test_journal(
            &oversized_path,
            &vec![b' '; RECONCILIATION_JOURNAL_MAX_BYTES as usize + 1],
        );
        assert!(read_durable_ownership_journal(&oversized_path).is_err());

        for (label, mut invalid) in [
            ("bad-request-hex", transition.clone()),
            ("bad-child-hex", transition.clone()),
            ("bad-cause", transition.clone()),
            ("bad-snapshot-parent", transition.clone()),
            ("bad-correlation", transition),
        ] {
            match label {
                "bad-request-hex" => invalid.request_fingerprint = "g".repeat(64),
                "bad-child-hex" => invalid.receipt.child_lease_fingerprint = "A".repeat(64),
                "bad-cause" => invalid.receipt.cause = EXACT_CHILD_ZEROED_CAUSE.to_string(),
                "bad-snapshot-parent" => invalid.snapshot.request_id.push_str("-other"),
                "bad-correlation" => {
                    invalid.recovery_correlation =
                        format!("exploration-global-stop:{}", "z".repeat(64));
                    invalid.receipt.recovery_correlation = invalid.recovery_correlation.clone();
                }
                _ => unreachable!(),
            }
            let path = reconciliation_test_path(label);
            let bytes = serde_json::to_vec(&DurableOwnershipJournal {
                schema_version: RECONCILIATION_JOURNAL_SCHEMA_VERSION,
                entries: vec![invalid],
            })
            .unwrap();
            write_raw_test_journal(&path, &bytes);
            assert!(read_durable_ownership_journal(&path).is_err(), "{label}");
        }
    }

    #[cfg(unix)]
    #[test]
    fn journal_rejects_symlink_nonregular_and_insecure_metadata() {
        use std::os::unix::fs::symlink;

        let symlink_path = reconciliation_test_path("symlink");
        let target_path = symlink_path.with_file_name("target.json");
        write_raw_test_journal(&target_path, b"{}\n");
        symlink(&target_path, &symlink_path).unwrap();
        assert!(read_durable_ownership_journal(&symlink_path).is_err());

        let directory_path = reconciliation_test_path("nonregular");
        fs::create_dir_all(&directory_path).unwrap();
        assert!(read_durable_ownership_journal(&directory_path).is_err());

        let insecure_path = reconciliation_test_path("insecure-file");
        write_raw_test_journal(&insecure_path, b"{}\n");
        fs::set_permissions(&insecure_path, fs::Permissions::from_mode(0o640)).unwrap();
        assert!(read_durable_ownership_journal(&insecure_path).is_err());
    }

    #[test]
    fn journal_uses_an_exclusive_cross_process_file_lock() {
        let path = reconciliation_test_path("flock");
        let first = lock_durable_ownership_journal(&path).expect("first journal flock");
        let parent = path.parent().unwrap();
        let file_name = path.file_name().unwrap().to_str().unwrap();
        let second_file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(parent.join(format!(".{file_name}.lock")))
            .unwrap();
        assert!(
            nix::fcntl::Flock::lock(second_file, nix::fcntl::FlockArg::LockExclusiveNonblock)
                .is_err()
        );
        drop(first);
    }

    #[tokio::test]
    async fn post_rename_directory_sync_failure_poison_retries_then_survives_restart() {
        let (handle, _, _, retained) =
            global_stop_reconciliation_fixture("req-post-rename-sync").await;
        let path = handle.reconciliation_journal_path();
        let request = global_stop_reconciliation_request(&retained);
        TEST_DIRECTORY_SYNC_FAILURES
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock()
            .insert(path.parent().unwrap().to_path_buf(), 2);

        assert!(matches!(
            handle.reconcile_after_global_stop(request.clone()).await,
            Err(ExplorationRecoveryError::FailedPrecondition(message))
                if message.contains("post-rename durability uncertain")
        ));
        assert_eq!(handle.snapshot().child_task_id, retained.child_task_id);
        assert!(handle.blocks_mapping_mutation().await);

        assert!(matches!(
            handle
                .replay_completed_global_stop_reconciliation(&request)
                .await,
            Err(ExplorationRecoveryError::FailedPrecondition(message))
                if message.contains("post-rename durability uncertain")
        ));
        assert!(handle
            .replay_completed_global_stop_reconciliation(&request)
            .await
            .expect("directory fsync retry")
            .is_none());

        let committed = handle
            .reconcile_after_global_stop(request.clone())
            .await
            .expect("exact durable retry clears live ownership");
        assert!(committed.child_task_id.is_empty());
        assert!(!handle.blocks_mapping_mutation().await);
        drop(handle);

        let restarted = ExploreRuntimeHandle::default_with_wavefront();
        TEST_DIRECTORY_SYNC_FAILURES
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock()
            .insert(path.parent().unwrap().to_path_buf(), 1);
        restarted.set_reconciliation_journal_path_for_tests(path);
        assert!(matches!(
            restarted.restore_latest_durable_reconciliation_snapshot(),
            Err(ExplorationRecoveryError::FailedPrecondition(message))
                if message.contains("directory durability verification failed")
        ));
        restarted
            .restore_latest_durable_reconciliation_snapshot()
            .expect("restart restores directory-synced journal");
        assert_eq!(
            restarted.snapshot().ownership_reconciliation_receipt,
            committed.ownership_reconciliation_receipt
        );
        assert!(restarted
            .replay_completed_global_stop_reconciliation(&request)
            .await
            .expect("restart exact replay")
            .is_some());
    }

    #[test]
    fn journal_evicts_only_acknowledged_entries_at_bounded_capacity() {
        let path = reconciliation_test_path("capacity");
        for index in 0..RECONCILIATION_JOURNAL_CAPACITY {
            assert!(matches!(
                commit_durable_ownership_transition(&path, durable_test_transition(index)),
                Ok(DurableCommitOutcome::Committed)
            ));
        }
        assert!(commit_durable_ownership_transition(
            &path,
            durable_test_transition(RECONCILIATION_JOURNAL_CAPACITY)
        )
        .is_err());
        let first = durable_test_transition(0);
        assert!(acknowledge_durable_ownership_transition(
            &path,
            &first.recovery_correlation,
            &first.receipt.child_lease_fingerprint,
        )
        .unwrap());
        assert!(matches!(
            commit_durable_ownership_transition(
                &path,
                durable_test_transition(RECONCILIATION_JOURNAL_CAPACITY)
            ),
            Ok(DurableCommitOutcome::Committed)
        ));
        let journal = read_durable_ownership_journal(&path).unwrap();
        assert_eq!(journal.entries.len(), RECONCILIATION_JOURNAL_CAPACITY);
        assert!(journal
            .entries
            .iter()
            .all(|entry| entry.recovery_correlation != first.recovery_correlation));
    }

    #[tokio::test]
    async fn late_exact_zero_and_global_reconciliation_have_one_first_winner() {
        let (handle, child_request_id, lease, retained) =
            global_stop_reconciliation_fixture("req-global-race").await;
        let global_request = global_stop_reconciliation_request(&retained);
        let global_handle = handle.clone();
        let zero_handle = handle.clone();
        let zero_child_request_id = child_request_id.clone();
        let global_task = tokio::spawn(async move {
            global_handle
                .reconcile_after_global_stop(global_request)
                .await
        });
        let zero_task = tokio::spawn(async move {
            zero_handle
                .report_navigation_result(
                    &zero_child_request_id,
                    ChildNavigationResult::Cancelled(zero_proof(&lease, 1)),
                )
                .await
        });
        let global_result = global_task.await.expect("global task");
        let zero_result = zero_task.await.expect("late zero task");

        assert_ne!(global_result.is_ok(), zero_result.is_ok());
        let final_snapshot = handle.snapshot();
        assert!(final_snapshot.child_task_id.is_empty());
        assert!(!handle.blocks_mapping_mutation().await);
        if global_result.is_ok() {
            assert!(final_snapshot.ownership_reconciliation_receipt.is_some());
            assert_eq!(
                final_snapshot.last_stop_reason,
                "global_stop_reconciled_safe"
            );
        } else {
            let receipt = final_snapshot
                .ownership_reconciliation_receipt
                .as_ref()
                .expect("exact ZEROED first-winner receipt");
            assert_eq!(receipt.cause, EXACT_CHILD_ZEROED_CAUSE);
            assert_eq!(final_snapshot.last_stop_reason, "child_cancel_failed");
        }
    }

    #[tokio::test]
    async fn failed_cancel_without_ticket_retries_only_for_the_exact_parent_request() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        tokio::time::timeout(Duration::from_secs(1), sink.cancel_entered.notified())
            .await
            .expect("cancel submit did not enter within one second");
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        tokio::time::timeout(Duration::from_secs(1), sink.cancel_entered.notified())
            .await
            .expect("cancel submit did not enter within one second");

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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        tokio::time::timeout(Duration::from_secs(1), sink.cancel_entered.notified())
            .await
            .expect("cancel submit did not enter within one second");

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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn costmap_refresh_does_not_stop_active_exploration() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start_with_config(
                "req-refresh",
                ExploreStartConfig {
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;

        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
            .await
            .unwrap();
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert_eq!(handle.stop("req-refresh").await.status, STATUS_STOPPING);
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn no_reachable_robot_origin_is_failed_not_completed() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, occupied_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-natural", "prefer_wavefront").await;

        let completed = handle.tick((1.0, 1.0)).await;

        assert_eq!(completed.status, STATUS_FAILED);
        assert_eq!(completed.last_stop_reason, "no_reachable_free_space");
    }

    #[tokio::test]
    async fn pure_snapshot_read_does_not_drive_controller() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        scheduler
            .shutdown()
            .await
            .expect("scheduler graceful shutdown");
    }

    #[tokio::test]
    async fn scheduler_shutdown_settles_owner_cancelled_blocked_pose_provider() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-blocked-pose", "prefer_wavefront").await;
        let entered = Arc::new(tokio::sync::Notify::new());
        let scheduler = handle.spawn_scheduler(
            Arc::new(BlockingPoseProvider {
                entered: entered.clone(),
            }),
            Duration::from_millis(5),
        );
        tokio::time::timeout(Duration::from_secs(1), entered.notified())
            .await
            .expect("scheduler must enter the blocking pose provider");

        tokio::time::timeout(Duration::from_secs(1), scheduler.shutdown())
            .await
            .expect("scheduler shutdown must abort a blocked pose provider")
            .expect("owner-requested scheduler cancellation must settle nonfatally");
    }

    #[tokio::test]
    async fn scheduler_shutdown_settles_blocked_submit_and_releases_runtime_lock() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-blocked-submit", "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.submit_release.lock().unwrap() = Some(release);
        let scheduler =
            handle.spawn_scheduler(Arc::new(StaticPoseProvider), Duration::from_millis(5));
        tokio::time::timeout(Duration::from_secs(1), sink.submit_entered.notified())
            .await
            .expect("scheduler must enter child submit");

        tokio::time::timeout(Duration::from_secs(1), scheduler.shutdown())
            .await
            .expect("scheduler shutdown must abort blocked child submit")
            .expect("owner-requested scheduler cancellation must settle nonfatally");
        let stopped =
            tokio::time::timeout(Duration::from_secs(1), handle.stop("req-blocked-submit"))
                .await
                .expect("aborting the scheduler must release ExploreRuntimeInner");
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(!handle.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn unexpected_scheduler_cancellation_remains_fatal() {
        let (handle, _) = configured_handle();
        let scheduler =
            handle.spawn_scheduler(Arc::new(StaticPoseProvider), Duration::from_millis(5));

        scheduler.inner.worker_abort.abort();
        let terminal = tokio::time::timeout(Duration::from_secs(1), scheduler.wait_terminal())
            .await
            .expect("unexpected scheduler cancellation terminal deadline")
            .expect_err("unexpected scheduler cancellation must remain fatal");
        assert!(terminal.contains("scheduler task join failed"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn requested_clean_scheduler_terminal_waits_for_root_owner_stop() {
        let (handle, _) = configured_handle();
        let scheduler =
            handle.spawn_scheduler(Arc::new(StaticPoseProvider), Duration::from_millis(5));
        let observer = scheduler.clone();
        let mut terminal = Box::pin(observer.wait_terminal());

        scheduler
            .shutdown()
            .await
            .expect("requested scheduler shutdown must settle cleanly");
        assert!(
            tokio::time::timeout(Duration::from_millis(50), terminal.as_mut())
                .await
                .is_err(),
            "a requested clean terminal must remain quiescent for the persistent owner"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn scheduler_panic_is_fatal_and_settled() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-panic", "prefer_wavefront").await;
        let scheduler =
            handle.spawn_scheduler(Arc::new(PanickingPoseProvider), Duration::from_millis(5));

        let terminal = tokio::time::timeout(Duration::from_secs(1), scheduler.wait_terminal())
            .await
            .expect("scheduler panic terminal deadline")
            .expect_err("scheduler panic must be visible");
        assert!(terminal.contains("scheduler task join failed"));
        let outcome = scheduler.shutdown_with_settlement().await;
        assert!(outcome.producers_settled());
        assert!(outcome.into_result().is_err());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn scheduler_abort_join_timeout_retains_final_handle_and_reports_unsettled() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-unsettled", "prefer_wavefront").await;
        let entered = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let release = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let scheduler = handle.spawn_scheduler(
            Arc::new(SynchronousBlockingPoseProvider {
                entered: entered.clone(),
                release: release.clone(),
            }),
            Duration::from_millis(5),
        );
        tokio::time::timeout(Duration::from_secs(1), async {
            while !entered.load(Ordering::Acquire) {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("scheduler entered synchronous producer");

        let first = scheduler.shutdown_with_settlement().await;
        assert!(!first.producers_settled());
        assert!(first
            .clone()
            .into_result()
            .expect_err("abort join timeout must be fatal")
            .contains("abort join timed out"));

        release.store(true, Ordering::Release);
        let second =
            tokio::time::timeout(Duration::from_secs(1), scheduler.shutdown_with_settlement())
                .await
                .expect("retained monitor final join deadline");
        assert!(second.producers_settled());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn scheduler_shutdown_cancellation_retains_monitor_for_retry() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle
            .start("req-cancelled-shutdown", "prefer_wavefront")
            .await;
        let entered = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let release = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let scheduler = handle.spawn_scheduler(
            Arc::new(SynchronousBlockingPoseProvider {
                entered: entered.clone(),
                release: release.clone(),
            }),
            Duration::from_millis(5),
        );
        tokio::time::timeout(Duration::from_secs(1), async {
            while !entered.load(Ordering::Acquire) {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("scheduler entered synchronous producer");

        let mut cancellation = WallClockWake::new(Duration::from_millis(25))
            .expect("independent cancellation wake starts");
        let cancelled = tokio::select! {
            biased;
            outcome = scheduler.shutdown_with_settlement() => Some(outcome),
            _ = cancellation.wait() => None,
        };
        assert!(
            cancelled.is_none(),
            "outer shutdown future must be cancelled"
        );
        assert_eq!(
            scheduler.inner.monitor_state.load(Ordering::Acquire),
            SCHEDULER_MONITOR_PENDING
        );

        release.store(true, Ordering::Release);
        let settled =
            tokio::time::timeout(Duration::from_secs(1), scheduler.shutdown_with_settlement())
                .await
                .expect("retry joins retained monitor");
        assert!(settled.producers_settled());
        assert_eq!(
            scheduler.inner.monitor_state.load(Ordering::Acquire),
            SCHEDULER_MONITOR_SETTLED
        );
    }

    #[tokio::test]
    async fn stale_pose_with_active_child_enters_cancel_pending_immediately() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        scheduler
            .shutdown()
            .await
            .expect("scheduler graceful shutdown");
    }

    #[tokio::test]
    async fn stale_pose_without_child_has_bounded_grace_then_fails() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        scheduler
            .shutdown()
            .await
            .expect("scheduler graceful shutdown");
    }

    #[tokio::test]
    async fn readiness_requires_backend_costmap_controller_and_nav_sink() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-ready", "prefer_wavefront").await;
        let readiness = handle.readiness(true).await;
        assert!(readiness.ready, "{}", readiness.reason);
        assert!(readiness.navigation_sink_ready);
        assert!(readiness.costmap_input_ready);
    }

    #[tokio::test]
    async fn costmap_refresh_keeps_active_child_and_next_goal_progresses() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-map", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
            .await
            .unwrap();
        let snapshot = handle.tick((2.0, 2.0)).await;
        assert_eq!(snapshot.status, STATUS_ACTIVE);
        assert_eq!(snapshot.child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());

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
    }

    #[tokio::test]
    async fn max_goals_waits_for_active_child_terminal_before_stopping() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
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
    async fn costmap_refresh_during_submit_keeps_dispatched_child_active() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-submit-race", "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.submit_release.lock().unwrap() = Some(release.clone());
        let ticking = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.tick((2.0, 2.0)).await })
        };
        tokio::time::timeout(Duration::from_secs(1), sink.submit_entered.notified())
            .await
            .expect("navigation submit did not enter within one second");
        let ingesting = {
            let handle = handle.clone();
            tokio::spawn(async move {
                ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2).await
            })
        };
        tokio::task::yield_now().await;
        release.notify_one();

        let active = ticking.await.unwrap();
        ingesting.await.unwrap().unwrap();

        assert_eq!(active.status, STATUS_ACTIVE);
        let current = handle.tick((2.0, 2.0)).await;
        assert_eq!(current.status, STATUS_ACTIVE);
        assert_eq!(current.child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn user_stop_after_costmap_update_cancels_active_child() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1)
            .await
            .unwrap();
        handle.start("req-stop-replan", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
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
