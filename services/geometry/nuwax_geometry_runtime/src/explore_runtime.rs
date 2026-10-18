//! Exploration controller, child-navigation handoff, and background scheduling.
//!
//! `ExploreRuntimeHandle` owns the production wavefront session. State reads are
//! pure snapshots; controller progress happens only through `tick*` or a started
//! [`ExploreSchedulerHandle`]. Mapping and navigation wiring are injected through
//! typed interfaces so the module never silently discards a selected goal.

#![allow(dead_code)]

#[cfg(test)]
#[path = "explore_attribution.rs"]
mod attribution;

#[path = "explore_compute_backend.rs"]
mod compute_backend;
#[path = "explore_compute_service.rs"]
mod explore_compute_service;
use compute_backend::*;
use explore_compute_service::*;

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
use nuwax_exploration::mission::{ReturnPolicy, MissionProgress, MissionPose};
use crate::explore_mission::{MissionCalibration, MissionRoutes, ExplorationCheckpoint, ExplorationConfiguration, NavigationConfiguration, mission_pose, pose3};

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
        ExplorationStopReason::PlanningResourceLimited => "planning_resource_limited",
        ExplorationStopReason::ConsecutiveFailuresExceeded => "consecutive_failures_exceeded",
        ExplorationStopReason::UserCancelled => "user_cancelled",
        ExplorationStopReason::CoverageReached => "coverage_reached",
        ExplorationStopReason::NoFrontierTimeout => "no_frontier_timeout",
        ExplorationStopReason::DurationExceeded => "duration_exceeded",
        ExplorationStopReason::MaxGoalsReached => "max_goals_reached",
        ExplorationStopReason::DistanceBudgetReached => "distance_budget_reached",
        ExplorationStopReason::MapRevisionChanged => "map_revision_changed",
        _ => "unsupported_stop_reason",
    }
}

fn exploration_stop_message(reason: ExplorationStopReason) -> String {
    if reason == ExplorationStopReason::PlanningResourceLimited {
        "ResourceLimited: exploration planning exhausted its bounded resources; reachability remains inconclusive".into()
    } else {
        format!("exploration completed: {}", stop_reason_str(reason))
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

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
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
    pub return_policy: ReturnPolicy,
    pub resume_progress_json: String,
    pub motion_admission: MotionAdmissionProof,
    pub boundary: Option<ExplorationBoundary>,
    pub stop_conditions: ExploreStopConditions,
    pub max_navigation_retries: u32,
    pub blacklist_radius_m: f64,
}

impl Default for ExploreStartConfig {
    fn default() -> Self {
        Self {
            return_policy: ReturnPolicy::default(),
            resume_progress_json: String::new(),
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
        let policy = self.validate_behavior(surface_mode)?;
        if !self.resume_progress_json.is_empty() { ExplorationCheckpoint::parse(&self.resume_progress_json)?; }
        self.motion_admission.validate()
            .map_err(|reason| format!("motion admission proof invalid: {reason}"))?;
        Ok(policy)
    }

    pub(crate) fn validate_behavior(&self, surface_mode: bool) -> Result<ExplorationArbitrationPolicy, String> {
        match self.strategy.trim().to_ascii_lowercase().as_str() {
            "" | "wavefront" | "frontier" => {}
            "surface" if surface_mode => {}
            "surface" => {
                return Err("surface exploration requires the pct-scan-3d navigation profile".into())
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
        self.return_policy.validate()?;
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
    pub intent: schema::GoalIntent,
    pub boundary_owner: Arc<ExploreStartConfig>,
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
    let (boundary_id, boundary_points) = match boundary {
        Some(boundary) => {
            let boundary_id = format!("{parent_request_id}:boundary");
            let points = boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y, 0.0])
                .collect::<Vec<_>>();
            (boundary_id, points)
        }
        None => (String::new(), Vec::new()),
    };
    Ok(schema::GoalIntent {
        map_frame_generation: 0,
                        options: Default::default(),
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
        body_to_support_z_m: None,
        boundary_points,
        motion_guard_generation: motion_admission.guard_generation as i64,
        motion_guard_boot_nonce: motion_admission.guard_boot_nonce,
        motion_stop_generation: motion_admission.motion_stop_generation as i64,
        goal: goal_pose,
    })
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChildNavigationLease {
    pub session_id: String,
    pub owner_kind: i32,
    pub parent_request_id: String,
    pub goal_generation: u64,
    pub producer_runtime_generation: u64,

}

impl ChildNavigationLease {
    pub(crate) fn matches(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
            && self.producer_runtime_generation == other.producer_runtime_generation
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationTicket {
    pub lease: ChildNavigationLease,
}

/// Child navigation submission failure; the variant carries any admission-stage proof.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChildNavigationSubmitError {
    /// 输入修订前进，释放候选等待新观察，不记为目标失败。
    SourceChanged(String),
    /// Navigation is temporarily unable to accept the goal; retry without feedback.
    RetryableReadiness(String),
    /// Permanent rejection without proof that no execution ownership/publication occurred.
    Rejected(String),
    /// This attempt was rejected before installing a navigation execution owner or
    /// publishing its goal. Does not prove absence or physical stop of another owner.
    RejectedBeforePublish(String),
    /// The exploration session's frozen navigation authority is no longer valid.
    SessionInvalidated(String),
}

impl ChildNavigationSubmitError {
    fn reason(&self) -> &str {
        match self {
            Self::RetryableReadiness(reason)
            | Self::SourceChanged(reason)
            | Self::Rejected(reason)
            | Self::RejectedBeforePublish(reason)
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
pub enum ChildNavigationCancelOutcome {
    Pending(ChildNavigationCancelTicket),
    /// Authoritative natural retirement, not a fabricated cancellation proof.
    AlreadyTerminal(ChildNavigationTerminalProof),
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

/// The original tick permit and its complete navigation handoff travel together.
/// A real retained run requires the explicit internal root-owned capability.
pub struct ChildNavigationDispatch {
    runtime: ExploreRuntimeHandle,
    generation: u64,
    request: ChildNavigationRequest,
    sink: Arc<dyn ExploreNavigationSink>,
    tick_permit: tokio::sync::OwnedMutexGuard<()>,
}

impl ChildNavigationDispatch {
    pub(crate) async fn run(
        self,
        owner: Option<&dyn ChildNavigationDispatchOwner>,
    ) -> ExploreRuntimeSnapshot {
        let Self { runtime, generation, request, sink, tick_permit } = self;
        // Keep the ORIGINAL permit through publish, attachment, and any exact
        // orphan settlement. Dropping the observer cannot release this permit.
        let _tick_permit = tick_permit;
        let snapshot = runtime.complete_child_dispatch(generation, request, sink, owner).await;
        if owner.is_some() {
            // Only explicit completion clears this projection. A panic leaves
            // it set and is reported by the original root supervisor.
            runtime.inner.lock().await.retained_child_dispatch = false;
        }
        snapshot
    }
}

/// A response is only an observation; it never transfers navigation ownership.
pub enum ChildNavigationDispatchObserver {
    Retained(tokio::sync::oneshot::Receiver<ExploreRuntimeSnapshot>),
    #[cfg(test)]
    Inline(std::pin::Pin<Box<dyn std::future::Future<Output = ExploreRuntimeSnapshot> + Send>>),
}

impl ChildNavigationDispatchObserver {
    #[cfg(test)]
    pub(crate) fn inline(operation: ChildNavigationDispatch) -> Self {
        Self::Inline(Box::pin(operation.run(None)))
    }

    async fn observe(self) -> Option<ExploreRuntimeSnapshot> {
        match self {
            Self::Retained(receiver) => receiver.await.ok(),
            #[cfg(test)]
            Self::Inline(operation) => Some(operation.await),
        }
    }
}

/// Internal, explicit root-owned capability. Both methods are required; there
/// is no silent pending/default settlement for an incompletely wired sink.
#[async_trait]
pub(crate) trait ChildNavigationDispatchOwner: Send + Sync {
    /// The callback runs once under the original root admission mutex, no await.
    fn with_dispatch_admission(&self, decide: &mut dyn FnMut(bool));
    /// Return only after authority proves the exact orphan terminal.
    async fn settle_retained_dispatch_orphan(&self, lease: ChildNavigationLease);
}

#[async_trait]
pub trait ExploreNavigationSink: Send + Sync {
    /// Register BEFORE polling submit. A failed registration must drop the
    /// never-polled operation; it must not fall back to an unmanaged task.
    fn retain_child_dispatch(
        &self,
        operation: ChildNavigationDispatch,
    ) -> Result<ChildNavigationDispatchObserver, String> {
        drop(operation);
        Err("retained child dispatch owner is unavailable".to_string())
    }

    /// 实际坐标拥有方的当前代际；测试替身可没有该观察能力。
    fn current_map_frame_generation(&self) -> Result<Option<i64>, String> { Ok(None) }
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
    ) -> Result<ChildNavigationCancelOutcome, String>;

    /// Read only a natural terminal that authoritatively retired this exact
    /// execution. An absent active lease alone is not proof of completion.
    async fn terminal_result_for(&self, _lease: &ChildNavigationLease) -> Option<ChildNavigationResult> {
        None
    }

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
    pub mission_progress_json: String,
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
            mission_progress_json: String::new(),
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
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub child_lease_fingerprint: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub child_lease: Option<ChildNavigationLease>,
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
    /// False survives cancellation of the post-attachment receipt read.
    terminal_receipt_checked: bool,
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
    request_identity: AssignedTransitionIdentity,
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
const RECONCILIATION_JOURNAL_SCHEMA_VERSION: u32 = 2;
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
            mission_progress_json: String::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum AssignedTransitionIdentity {
    GlobalStop {
        parent_request_id: String,
        exploration_generation: u64,
        child_lease: ChildNavigationLease,
        cancel_generation: u64,
        stop_operation_id: String,
        original_guard_generation: u64,
        original_guard_boot_nonce: [u8; 16],
        original_motion_stop_generation: u64,
        held_guard_generation: u64,
        held_guard_boot_nonce: [u8; 16],
        held_motion_stop_generation: u64,
        zero_ack_id: String,
        zero_applied_monotonic_ns: u64,
    },
    ExactChildZeroed {
        child_lease: ChildNavigationLease,
        cancel_generation: u64,
        zero_ack_id: String,
        zero_applied_monotonic_ns: u64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct DurableOwnershipTransition {
    kind: DurableOwnershipTransitionKind,
    recovery_correlation: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    request_fingerprint: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    request_identity: Option<AssignedTransitionIdentity>,
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

#[derive(Serialize, Deserialize)]
#[serde(tag = "format", content = "transition", rename_all = "snake_case", deny_unknown_fields)]
enum DurableOwnershipEntryV2 {
    LegacyV1(DurableOwnershipTransition),
    AssignedV2(DurableOwnershipTransition),
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct DurableOwnershipJournalV2 {
    schema_version: u32,
    entries: Vec<DurableOwnershipEntryV2>,
}

fn decode_ownership_journal(bytes: &[u8]) -> Result<DurableOwnershipJournal, String> {
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Header { schema_version: u32, entries: Vec<serde_json::Value> }
    let header: Header = serde_json::from_slice(bytes)
        .map_err(|error| format!("ownership journal JSON invalid: {error}"))?;
    let mut entries = Vec::with_capacity(header.entries.len());
    for value in header.entries {
        let entry = match header.schema_version {
            1 => {
                let entry: DurableOwnershipTransition = serde_json::from_value(value)
                    .map_err(|error| format!("legacy ownership journal JSON invalid: {error}"))?;
                if entry.request_identity.is_some() || entry.receipt.child_lease.is_some() {
                    return Err("legacy journal cannot contain assigned identity".into());
                }
                entry
            }
            2 => match serde_json::from_value::<DurableOwnershipEntryV2>(value)
                .map_err(|error| format!("ownership journal v2 JSON invalid: {error}"))? {
                    DurableOwnershipEntryV2::LegacyV1(entry) if entry.request_identity.is_none()
                        && entry.receipt.child_lease.is_none() => entry,
                    DurableOwnershipEntryV2::AssignedV2(entry) if entry.request_identity.is_some()
                        && entry.receipt.child_lease.is_some() && entry.request_fingerprint.is_empty()
                        && entry.receipt.child_lease_fingerprint.is_empty() => entry,
                    _ => return Err("ownership journal mixes legacy and assigned identity".into()),
                },
            other => return Err(format!("ownership journal schema {other} is unsupported")),
        };
        entries.push(entry);
    }
    if !matches!(header.schema_version, 1 | 2) {
        return Err(format!("ownership journal schema {} is unsupported", header.schema_version));
    }
    Ok(DurableOwnershipJournal { schema_version: header.schema_version, entries })
}

fn encode_ownership_journal(journal: &DurableOwnershipJournal) -> Result<Vec<u8>, serde_json::Error> {
    if journal.schema_version != 2 {
        return serde_json::to_vec(journal);
    }
    serde_json::to_vec(&DurableOwnershipJournalV2 {
        schema_version: 2,
        entries: journal.entries.iter().cloned().map(|entry|
            if entry.request_identity.is_some() { DurableOwnershipEntryV2::AssignedV2(entry) }
            else { DurableOwnershipEntryV2::LegacyV1(entry) }).collect(),
    })
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
    let identity_valid = match (&entry.request_identity, &receipt.child_lease) {
        (None, None) => entry.recovery_correlation.strip_prefix("exploration-global-stop:")
            .is_some_and(is_sha256_hex) && is_sha256_hex(&entry.request_fingerprint)
            && is_sha256_hex(&receipt.child_lease_fingerprint),
        (Some(identity), Some(child)) => {
            let (identity_child, cancel_generation, zero_ack_id, zero_ns, kind_valid) = match identity {
                AssignedTransitionIdentity::GlobalStop { parent_request_id, exploration_generation,
                    child_lease, cancel_generation, stop_operation_id, original_guard_generation,
                    original_guard_boot_nonce, held_guard_generation, held_guard_boot_nonce,
                    zero_ack_id, zero_applied_monotonic_ns, .. } => (child_lease, *cancel_generation,
                        zero_ack_id, *zero_applied_monotonic_ns,
                        entry.kind == DurableOwnershipTransitionKind::GlobalStop
                        && parent_request_id == &receipt.parent_request_id
                        && *exploration_generation == receipt.exploration_generation
                        && is_bounded_string(stop_operation_id, RECONCILIATION_JOURNAL_MAX_ID_BYTES)
                        && *original_guard_generation > 0 && *original_guard_boot_nonce != [0; 16]
                        && *held_guard_generation > 0 && *held_guard_boot_nonce != [0; 16]),
                AssignedTransitionIdentity::ExactChildZeroed { child_lease, cancel_generation,
                    zero_ack_id, zero_applied_monotonic_ns } => (child_lease, *cancel_generation,
                        zero_ack_id, *zero_applied_monotonic_ns,
                        entry.kind == DurableOwnershipTransitionKind::ExactChildZeroed),
            };
            kind_valid && identity_child == child && child.parent_request_id == receipt.parent_request_id
                && valid_child_lease(child) && cancel_generation == receipt.cancel_generation
                && is_bounded_string(zero_ack_id, RECONCILIATION_JOURNAL_MAX_ID_BYTES) && zero_ns > 0
                && entry.request_fingerprint.is_empty() && receipt.child_lease_fingerprint.is_empty()
                && entry.recovery_correlation == assigned_recovery_correlation(child, cancel_generation)
        }
        _ => false,
    };
    if !identity_valid || !is_bounded_string(&entry.recovery_correlation, RECONCILIATION_JOURNAL_MAX_ID_BYTES)
        || receipt.recovery_correlation != entry.recovery_correlation
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
    let journal = decode_ownership_journal(&bytes)?;
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
    let bytes = encode_ownership_journal(journal)
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
            && existing.request_identity == transition.request_identity
            && existing.receipt.child_lease == transition.receipt.child_lease
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
    if transition.request_identity.is_some() { journal.schema_version = 2; }
    journal.entries.push(transition);
    persist_durable_ownership_journal(path, &journal)?;
    Ok(DurableCommitOutcome::Committed)
}

fn replay_durable_ownership_transition(
    path: &Path,
    kind: DurableOwnershipTransitionKind,
    recovery_correlation: &str,
    request_identity: &AssignedTransitionIdentity,
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
    if existing.kind == kind && existing.request_identity.as_ref() == Some(request_identity) {
        return Ok(Some(existing.clone()));
    }
    Err("ownership transition replay proof does not match the durable first winner".to_string())
}

fn acknowledge_durable_ownership_transition(
    path: &Path,
    recovery_correlation: &str,
    child_lease_fingerprint: &str,
    child_lease: Option<&ChildNavigationLease>,
) -> Result<bool, String> {
    let _io_guard = RECONCILIATION_JOURNAL_IO_LOCK
        .get_or_init(|| SyncMutex::new(()))
        .lock();
    let _file_guard = lock_durable_ownership_journal(path)?;
    let mut journal = read_durable_ownership_journal(path)?;
    let Some(existing) = journal.entries.iter_mut().find(|entry| {
        entry.recovery_correlation == recovery_correlation
            && match (entry.receipt.child_lease.as_ref(), child_lease) {
                (Some(stored), Some(child)) => child_lease_fingerprint.is_empty() && stored == child,
                (None, None) => !child_lease_fingerprint.is_empty() && entry.receipt.child_lease_fingerprint == child_lease_fingerprint,
                _ => false,
            }
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

fn valid_child_lease(child: &ChildNavigationLease) -> bool {
    is_bounded_string(&child.session_id, RECONCILIATION_JOURNAL_MAX_ID_BYTES)
        && is_bounded_string(&child.parent_request_id, RECONCILIATION_JOURNAL_MAX_ID_BYTES)
        && child.owner_kind == schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        && (1..=i64::MAX as u64).contains(&child.goal_generation)
        && (1..=i64::MAX as u64).contains(&child.producer_runtime_generation)
}

fn assigned_recovery_correlation(child: &ChildNavigationLease, cancel_generation: u64) -> String {
    format!("exploration-global-stop:v2:{}:{}:{}",
        child.producer_runtime_generation, child.goal_generation, cancel_generation)
}

fn global_stop_request_identity(request: &ExplorationGlobalStopReconciliationRequest) -> AssignedTransitionIdentity {
    let proof = &request.guard_proof;
    AssignedTransitionIdentity::GlobalStop {
        parent_request_id: request.request_id.clone(),
        exploration_generation: request.exploration_generation,
        child_lease: request.child_lease.clone(),
        cancel_generation: request.cancel_generation,
        stop_operation_id: proof.stop_operation_id.clone(),
        original_guard_generation: proof.original_motion_admission.guard_generation,
        original_guard_boot_nonce: proof.original_motion_admission.guard_boot_nonce,
        original_motion_stop_generation: proof.original_motion_admission.motion_stop_generation,
        held_guard_generation: proof.held_guard_generation,
        held_guard_boot_nonce: proof.held_guard_boot_nonce,
        held_motion_stop_generation: proof.held_motion_stop_generation,
        zero_ack_id: proof.zero_ack_id.clone(),
        zero_applied_monotonic_ns: proof.zero_applied_monotonic_ns,
    }
}

fn exact_child_zero_request_identity(proof: &ChildNavigationZeroProof) -> AssignedTransitionIdentity {
    AssignedTransitionIdentity::ExactChildZeroed {
        child_lease: proof.lease.clone(), cancel_generation: proof.cancel_generation,
        zero_ack_id: proof.zero_ack_id.clone(), zero_applied_monotonic_ns: proof.zero_applied_monotonic_ns,
    }
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
    #[cfg(test)]
    pub(crate) fn cloud_revision_for_test(&self) -> i64 { self.cloud_revision }

    // Compare only inside the same domain. A mask commit can advance while the
    // cloud root stays unchanged; neither component is a new synthetic clock.
    fn revision(&self) -> (i64, i64) {
        (self.cloud_revision, self.committed_map.revision)
    }

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
    installed_revision: Option<(i64, i64)>,
    minimum_eligible_cloud_revision: i64,
    input_ready: bool,
    input_error: Option<String>,
    accepting: bool,
}

impl SurfaceExplorationMode {
    // The existing finish contract permits an older in-flight success newer
    // than installed_revision. Do not replace this with "ticket == latest".
    fn accepts_surface_ticket(&self, ticket: &SurfaceMapTicket) -> bool {
        self.accepting
            && self.latest.as_ref().is_some_and(|latest|
                ticket.same_domain(latest) && ticket.revision() <= latest.revision())
            && ticket.cloud_revision >= self.minimum_eligible_cloud_revision
            && self.installed_revision.is_none_or(|revision| ticket.revision() > revision)
    }
}

pub struct ExploreRuntimeInner {
    logical_active: bool,
    committed: CommittedView,
    compute: ComputeState,
    arbitration: ExplorationArbitrationPolicy,
    last_request_id: String,
    last_selected_source: String,
    last_stop_reason: String,
    last_stop: Option<ExplorationStopReason>,
    last_frontier_goal: Option<FrontierGoal>,
    last_poll: Option<ExplorationPoll>,
    costmap_input: Option<Arc<ExploreCostmapSnapshot>>,
    surface_mode: Option<SurfaceExplorationMode>,
    /// 最近一次有效地图输入的本地接收时刻，不参与地图身份。
    costmap_received_monotonic_ns: u64,
    costmap_receipt_ttl: Duration,
    committed_map: Option<schema::CommittedMapIdentity>,
    costmap_frame_generation: i64,
    active_config: Option<Arc<ExploreStartConfig>>,
    // Captured once at start, including errors; saving never relabels a run with later environment values.
    navigation_configuration: Option<Result<NavigationConfiguration, String>>,
    mission: Option<MissionProgress>,
    mission_calibration: Option<MissionCalibration>,
    return_reason: Option<ExplorationStopReason>,
    return_goal_id: Option<String>,
    return_failures: u32,
    next_return_route_check: Option<Instant>,
    return_after_cancellation: bool,
    progress_stopped_ns: Option<u64>,
    progress_map_name: String,
    progress_origin: Option<(String,i64,i64)>,
    active_child: Option<ActiveChildNavigation>,
    pending_child_dispatch: Option<ChildNavigationRequest>,
    // One accepted dispatch owns the original tick permit across all epochs.
    // Neither Stop nor Start clears this; only that operation may settle it.
    retained_child_dispatch: bool,
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
            logical_active: false,
            committed: CommittedView::capture(&build_controller(
                ExplorationArbitrationPolicy::PreferWavefront,
                &ExploreStartConfig::default(),
            )),
            compute: ComputeState::default(),
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
            costmap_receipt_ttl: DEFAULT_COSTMAP_RECEIPT_TTL,
            committed_map: None,
            costmap_frame_generation: 0,
            active_config: None,
            navigation_configuration: None,
            mission: None,
            mission_calibration: None,
            return_reason: None,
            return_goal_id: None,
            return_failures: 0,
            next_return_route_check: None,
            return_after_cancellation: false,
            progress_stopped_ns: None,
            progress_map_name: String::new(),
            progress_origin: None,
            active_child: None,
            pending_child_dispatch: None,
            retained_child_dispatch: false,
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

    fn session_in_progress(&self) -> bool {
        self.logical_active || self.compute.start_reservation.is_some() || self.compute.control_pending() || self.pending_child_cancellation.is_some()
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
        let rollover_recovery_eligible = !self.logical_active
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
                    eligible: !self.logical_active
                        && ticket.cancel_generation > 0
                        && ticket.lease.matches(child_lease),
                    cancel_generation: ticket.cancel_generation,
                    recovery_correlation: assigned_recovery_correlation(child_lease, ticket.cancel_generation),
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
            mission_progress_json: self.mission.as_ref().map(|p|serde_json::to_string(p).expect("有效行程载荷")).unwrap_or_default(),
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
    build_controller_with_geometry(policy, config, None, None).expect("无观察标定的默认控制器")
}

fn build_controller_with_geometry(
    policy: ExplorationArbitrationPolicy,
    config: &ExploreStartConfig,
    geometry: Option<nuwax_nav_types_messages::RobotGeometry>,
    observation: Option<nuwax_nav_types_messages::ObservationModel>,
) -> Result<ExplorationController,String> {
    let mut controller = ExplorationController::new(policy);
    let mut adapter =
        WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
    if let Some(geometry) = geometry {
        adapter.set_robot_geometry(geometry);
    }
    if let Some(model)=observation {adapter.set_observation_model(model).map_err(|error|error.to_string())?;}
    adapter.configure_constraints(
        config.boundary.clone(),
        config.max_navigation_retries,
        config.blacklist_radius_m,
    );
    controller.add_source(adapter);
    Ok(controller)
}

fn build_profile_controller(
    policy: ExplorationArbitrationPolicy,
    config: &ExploreStartConfig,
    surface: Option<&SurfaceExplorationMode>,
) -> Result<ExplorationController, String> {
    let observation=nuwax_nav_types_messages::NavigationCapabilities::from_env()?.and_then(|profile|profile.observation);
    let Some(surface) = surface else {
        let geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON")
            .ok()
            .filter(|value| !value.trim().is_empty())
            .map(|value| nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value))
            .transpose()?;
        return build_controller_with_geometry(policy, config, geometry, observation);
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
    if let Some(model)=observation {adapter.set_observation_model(model).map_err(|error|error.to_string())?;}
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

}

impl From<&ChildNavigationLease> for ExactOrphanLeaseKey {
    fn from(lease: &ChildNavigationLease) -> Self {
        Self {
            session_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            producer_runtime_generation: lease.producer_runtime_generation,

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
    #[cfg(test)]
    attribution: Option<Arc<attribution::Attribution>>,
    #[cfg(test)]
    homeward_test_hook: Option<Arc<dyn Fn() + Send + Sync>>,
    inner: Arc<Mutex<ExploreRuntimeInner>>,
    compute_service: Arc<ComputeService>,
    tick_gate: Arc<Mutex<()>>,
    expected_map_frame: Arc<str>,
    readiness_cache: Arc<SyncMutex<ExplorationReadinessDetail>>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    navigation_sink: Arc<RwLock<Option<Arc<dyn ExploreNavigationSink>>>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    generation: Arc<AtomicU64>,
    orphan_cancellations: Arc<OrphanCancellationRegistry>,
}

impl ExploreRuntimeHandle {
    fn next_exploration_generation(&self, guard: &mut ExploreRuntimeInner) -> u64 {
        match self.generation.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |value| value.checked_add(1)) {
            Ok(previous) => previous + 1,
            Err(current) => {
                guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
                guard.compute.fault = Some("exploration generation exhausted".into());
                guard.compute.shutdown = true;
                self.compute_service.wake.notify_one();
                current
            }
        }
    }

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
            installed_revision: None,
            minimum_eligible_cloud_revision: 0,
            input_ready: false,
            input_error: Some("surface map input is unavailable".into()),
            accepting: true,
        };
        // The actual source/controller is installed by the worker at start.
        guard.committed = CommittedView::capture(&build_profile_controller(
            guard.arbitration, &ExploreStartConfig::default(), Some(&mode),
        ).unwrap_or_else(|_| ExplorationController::new(guard.arbitration)));
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
                .is_some_and(|previous| {
                    ticket.cloud_revision < previous.cloud_revision
                        || (ticket.cloud_revision == previous.cloud_revision
                            && (message.cleared
                                || !ticket.same_domain(previous)
                                || ticket.committed_map.revision <= previous.committed_map.revision
                                || ticket.committed_map.commit_sequence <= previous.committed_map.commit_sequence))
                })
            {
                // Match navigation's composite receipt rule: a same-root mask
                // revision must advance through a newer owner commit. Domain
                // changes and clears still require the original cloud fence.
                return Err("surface map revision is duplicate, stale, or lacks a new commit".into());
            }
            let invalidates = message.cleared
                || mode
                    .latest
                    .as_ref()
                    .is_some_and(|previous| !ticket.same_domain(previous));
            mode.latest = Some(ticket.clone());
            if invalidates {
                mode.installed_revision = None;
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
            (guard.logical_active || guard.compute.start_reservation.is_some())
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

    /// A cache success is not necessarily an owner installation. Qualify the
    /// same already-installed Arc under the original completion fences; this
    /// operation does not update source, notification, ticket or receipt clocks.
    pub(crate) async fn can_defer_incomplete_surface_map(
        &self,
        ticket: &SurfaceMapTicket,
        retained: &Arc<SurfaceSnapshot>,
    ) -> bool {
        let guard = self.inner.lock().await;
        let Some(mode) = guard.surface_mode.as_ref() else { return false; };
        mode.accepts_surface_ticket(ticket)
            && mode.receiver.snapshot().ok().flatten()
                .is_some_and(|installed| Arc::ptr_eq(&installed, retained))
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
            if !mode.accepts_surface_ticket(&ticket) {
                return false;
            }
            match result {
                Ok(snapshot) => match mode.receiver.observe(snapshot) {
                    Ok(()) => {
                        mode.installed_revision = Some(ticket.revision());
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
            (guard.logical_active || guard.compute.start_reservation.is_some())
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
            mode.installed_revision = None;
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
            compute_service: Arc::new(ComputeService::new()),
            tick_gate: Arc::new(Mutex::new(())),
            #[cfg(test)]
            attribution: None,
            #[cfg(test)]
            homeward_test_hook: None,
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
        if recovered {
            self.orphan_cancellations.mark_cancel_pending(&lease);
        } else {
            match sink.cancel(&lease).await {
                Ok(ChildNavigationCancelOutcome::Pending(_)) => {
                    self.orphan_cancellations.mark_cancel_pending(&lease);
                }
                Ok(ChildNavigationCancelOutcome::AlreadyTerminal(proof))
                    if proof.lease.matches(&lease) => {
                    self.orphan_cancellations.remove_terminal(&lease);
                    return;
                }
                Ok(_) | Err(_) => {}
            }
        }
        let _ = sink.retain_orphan_cancellation(lease.clone());
        if sink.orphan_lease_is_terminal(&lease).await {
            self.orphan_cancellations.remove_terminal(&lease);
        }
    }

    pub(crate) async fn reconcile_orphan_cancellations(&self) -> bool {
        // Process Stop shares the original dispatch fence. An accepted dispatch
        // may still be between Goal publication and registering its exact orphan.
        let Ok(_tick_permit) = self.tick_gate.try_lock() else { return false; };
        self.reconcile_orphan_cancellations_with_tick_permit().await
    }

    async fn reconcile_orphan_cancellations_with_tick_permit(&self) -> bool {
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

    #[cfg(test)]
    pub(crate) fn fail_next_reconciliation_directory_sync_for_tests(&self) {
        let path = self.reconciliation_journal_path();
        TEST_DIRECTORY_SYNC_FAILURES
            .get_or_init(|| SyncMutex::new(std::collections::HashMap::new()))
            .lock()
            .insert(path.parent().unwrap().to_path_buf(), 1);
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
        child_lease: Option<&ChildNavigationLease>,
    ) -> Result<bool, ExplorationRecoveryError> {
        if recovery_correlation.trim().is_empty()
            || (child_lease_fingerprint.is_empty() == child_lease.is_none())
            || child_lease.is_some_and(|child| !valid_child_lease(child)) {
            return Err(ExplorationRecoveryError::InvalidArgument(
                "reconciliation acknowledgement identity is incomplete".to_string(),
            ));
        }
        acknowledge_durable_ownership_transition(
            &self.reconciliation_journal_path(),
            recovery_correlation,
            child_lease_fingerprint,
            child_lease,
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
            mission_progress_json: String::new(),
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
            mission_progress_json: String::new(),
            };
            drop(guard);
            return snapshot;
        }

        let navigation_configuration = NavigationConfiguration::capture();
        let mission_calibration = if config.return_policy.enabled() {
            let surface=guard.surface_mode.as_ref().and_then(|m|m.config.as_ref().ok());
            match MissionCalibration::capture(surface) {
                Ok(value)=>Some(value),
                Err(error)=>{let snapshot=guard.snapshot(STATUS_REJECTED,error);drop(guard);return self.store_snapshot(snapshot);}
            }
        } else {None};
        let controller =
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
        let mut legacy_replanned = false;
        let mut restored=if config.resume_progress_json.is_empty(){None}else{
            let result=(|| {
                let mut saved=ExplorationCheckpoint::parse(&config.resume_progress_json)?;
                let map_name=self.navigation_sink.read().as_ref().ok_or("导航地图来源缺失")?.current_map_name()?;
                if saved.map_name!=map_name || saved.map_frame_id!=self.expected_map_frame.as_ref() {
                    return Err("探索进度与当前地图不匹配".into());
                }
                legacy_replanned = saved.resume_requires_replan(
                    &ExplorationConfiguration::capture(&config, surface_mode),
                    navigation_configuration.as_ref().map_err(Clone::clone)?,
                )?;
                if config.return_policy.enabled()!=saved.mission.is_some(){return Err("探索进度缺少行程或请求未启用行程".into());}
                if legacy_replanned {
                    saved.prepare_history_for_replan();
                }
                Ok::<_,String>(saved)
            })();
            match result {Ok(saved)=>Some(saved),Err(error)=>{
                let snapshot=guard.snapshot(STATUS_REJECTED,error);drop(guard);return self.store_snapshot(snapshot);
            }}
        };

        if let Err(error) = self.compute_service.ensure(self, &mut guard.compute) {
            let snapshot = guard.snapshot(STATUS_REJECTED, error); drop(guard); return self.store_snapshot(snapshot);
        }
        let generation = self.next_exploration_generation(&mut guard);
        if let Some(error) = guard.compute.fault.clone() {
            let snapshot = guard.snapshot(STATUS_REJECTED, error); drop(guard); return self.store_snapshot(snapshot);
        }
        guard.generation = generation;
        guard.last_request_id = request_id.to_string();
        let _reservation = self.reserve_start_locked(&mut guard, generation);
        let reserved_domain = guard.committed_map.as_ref().map(|map| (
            map.session_id.clone(), map.session_generation,
            guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation),
        ));
        let reserved_map_name = self.navigation_sink.read().as_ref().and_then(|sink| sink.current_map_name().ok());
        let reservation_matches = |guard: &ExploreRuntimeInner| {
            guard.generation == generation && guard.compute.start_reservation == Some(generation)
                && guard.last_request_id == request_id
                && reserved_domain.as_ref().is_none_or(|(session, session_generation, frame)| {
                    guard.committed_map.as_ref().is_some_and(|map| &map.session_id == session && map.session_generation == *session_generation)
                        && guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation) == *frame
                })
                && reserved_map_name.as_ref().is_none_or(|expected| self.navigation_sink.read().as_ref()
                    .is_some_and(|sink| sink.current_map_name().is_ok_and(|name| &name == expected)))
        };
        let restore = restored.as_mut().map(|saved| ComputeRestore {
            sources: std::mem::take(&mut saved.sources), replan: legacy_replanned,
        });
        drop(guard);
        // Old accepted feedback and end settle before begin/restore/install. This
        // await holds no runtime lock, and the reservation survives cancellation.
        if let Err(error) = self.await_compute_idle().await {
            let mut guard = self.inner.lock().await;
            guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
            let snapshot = guard.snapshot(STATUS_REJECTED, error); drop(guard); return self.store_snapshot(snapshot);
        }
        let mut guard = self.inner.lock().await;
        if !reservation_matches(&guard) {
            if guard.compute.start_reservation == Some(generation) { guard.request_compute_stop(ExplorationStopReason::MapRevisionChanged); self.compute_service.wake.notify_one(); }
            return self.snapshot();
        }
        let install = match self.admit_lifecycle_locked(&mut guard, ComputeLifecycle::Install { controller, begin: true, restore }) {
            Ok(ticket) => ticket,
            Err(error) => { guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
                let snapshot = guard.snapshot(STATUS_REJECTED, error); drop(guard); return self.store_snapshot(snapshot); }
        };
        drop(guard);
        let installed = self.await_lifecycle(&install).await;
        let mut guard = self.inner.lock().await;
        if !reservation_matches(&guard) {
            if guard.compute.start_reservation == Some(generation) { guard.request_compute_stop(ExplorationStopReason::MapRevisionChanged); self.compute_service.wake.notify_one(); }
            return self.snapshot();
        }
        if let Err(error) = installed {
            guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
            self.compute_service.wake.notify_one();
            let snapshot = guard.snapshot(STATUS_REJECTED, format!("begin/restore exploration failed: {error}"));
            drop(guard); return self.store_snapshot(snapshot);
        }
        if let Err(error) = self.open_compute_locked(&mut guard) {
            guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
            let snapshot = guard.snapshot(STATUS_REJECTED, error); drop(guard); return self.store_snapshot(snapshot);
        }

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
        let mut active_config=config.clone();
        // 恢复载荷只在新控制器安装时使用，不随运行期配置克隆反复复制。
        active_config.resume_progress_json.clear();
        guard.active_config = Some(Arc::new(active_config));
        guard.navigation_configuration = Some(navigation_configuration);
        guard.mission = None;
        guard.mission_calibration = mission_calibration;
        guard.return_reason = None;
        guard.next_return_route_check = None;
        guard.return_goal_id = None;
        guard.return_failures = 0;
        guard.return_after_cancellation = false;
        guard.progress_stopped_ns=None;guard.progress_map_name.clear();guard.progress_origin=None;
        guard.active_child = None;
        guard.pending_child_dispatch = None;
        guard.pending_child_cancellation = None;
        guard.completed_rollover_recovery = None;
        guard.session_generation = generation;
        guard.generation = generation;
        guard.child_sequence = 0;
        guard.reached_goals = 0;
        guard.started_at_ns = now_unix_nanos();
        if let Some(saved)=restored {
            guard.mission=saved.mission;guard.return_reason=saved.return_reason;guard.reached_goals=saved.reached_goals;
            guard.started_at_ns=guard.started_at_ns.saturating_sub(saved.elapsed_ns);
        }
        guard.no_frontier_since = None;
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_ACCEPTED,
            format!(
                "exploration accepted (source={}, arbitration={}){}",
                if surface_mode { "surface" } else { "wavefront" },
                arbitration_policy_str(policy),
                if legacy_replanned { "; legacy history resumed under current settings; route and retry decisions replanned" } else { "" }
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
        if guard.logical_active || !cancel_failed {
            return Err(ExplorationRecoveryError::FailedPrecondition(
                "exploration recovery requires a stopped controller and failed child cancellation"
                    .to_string(),
            ));
        }

        let generation = self.next_exploration_generation(&mut guard);
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
        let request_identity = global_stop_request_identity(request);
        if let Some(durable) = replay_durable_ownership_transition(
            &self.reconciliation_journal_path(),
            DurableOwnershipTransitionKind::GlobalStop,
            &request.recovery_correlation,
            &request_identity,
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
        let request_identity = global_stop_request_identity(&request);
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
        if guard.logical_active
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
        let expected_correlation = assigned_recovery_correlation(&active_child.lease, ticket.cancel_generation);
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
            child_lease_fingerprint: String::new(),
            child_lease: Some(active_child.lease.clone()),
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
            request_fingerprint: String::new(),
            request_identity: Some(request_identity),
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
        self.begin_child_action(request_id, PendingChildAction { reason, message }, None)
            .await
    }

    /// Claimed by the retained compute-service control coroutine. Identity and
    /// generation are checked again atomically with existing cancel admission.
    async fn finish_compute_action(&self, action: &ComputeStopAction) -> ExploreRuntimeSnapshot {
        self.begin_child_action(Some(&action.parent), PendingChildAction {
            reason: action.reason, message: action.message.clone(),
        }, Some(action)).await
    }

    async fn cancel_child_for_return(&self,generation:u64,reason:ExplorationStopReason)->ExploreRuntimeSnapshot {
        let mut guard=self.inner.lock().await;
        if generation!=guard.generation || !guard.logical_active || guard.pending_child_cancellation.is_some(){return self.snapshot();}
        let Some(child)=guard.active_child.clone() else{return guard.snapshot(STATUS_ACTIVE,"返航等待派发");};
        guard.return_after_cancellation=true;
        guard.pending_child_dispatch=None;
        guard.pending_child_cancellation=Some(PendingChildCancellation {action:PendingChildAction {reason,message:"行程条件已满足，先停车再返航".into()},ticket:None,failure:None,dispatch_in_flight:true,early_terminals:Vec::new()});
        let snapshot=guard.snapshot(STATUS_STOPPING,"返航等待当前子目标物理停稳");self.store_snapshot(snapshot);drop(guard);
        self.dispatch_child_cancellation(child,false).await
    }

    async fn begin_child_action(
        &self,
        request_id: Option<&str>,
        action: PendingChildAction,
        compute_action: Option<&ComputeStopAction>,
    ) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if compute_action.is_some_and(|action| !action.matches(&guard)) { return self.snapshot(); }
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
        if guard.return_after_cancellation {
            // 用户取消、故障或关闭覆盖返航意图，共用已经在途的停车事务。
            guard.return_after_cancellation=false;
            if compute_action.is_none() { guard.generation=self.next_exploration_generation(&mut guard); }
            guard.request_compute_stop(action.reason);
            self.compute_service.wake.notify_one();
            guard.last_stop=Some(action.reason);guard.last_stop_reason=stop_reason_str(action.reason).into();
            if let Some(pending)=guard.pending_child_cancellation.as_mut(){pending.action=action.clone();}
        }
        if guard.pending_child_cancellation.is_some() {
            let (failure, dispatch_in_flight) = guard
                .pending_child_cancellation
                .as_ref()
                .map(|pending| (pending.failure.clone(), pending.dispatch_in_flight))
                .expect("pending child cancellation checked above");
            let retry_correlated = compute_action.is_none() && request_id.is_some() && failure.is_some() && !dispatch_in_flight;
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
        let generation = if compute_action.is_some() { guard.generation } else { self.next_exploration_generation(&mut guard) };
        self.record_child_action_stop_locked(&mut guard, generation, action.reason);

        let Some(child) = guard.active_child.clone() else {
            let (snapshot, readiness, frontier_update) =
                self.prepare_no_child_stop_locked(&mut guard, action);
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

    fn record_child_action_stop_locked(
        &self,
        guard: &mut ExploreRuntimeInner,
        generation: u64,
        reason: ExplorationStopReason,
    ) {
        guard.generation = generation;
        guard.request_compute_stop(reason);
        self.compute_service.wake.notify_one();
        guard.last_stop = Some(reason);
        guard.last_stop_reason = stop_reason_str(reason).to_string();
        guard.last_poll = Some(ExplorationPoll::Stop(reason));
        guard.pending_child_dispatch = None;
    }

    fn prepare_no_child_stop_locked(
        &self,
        guard: &mut ExploreRuntimeInner,
        action: PendingChildAction,
    ) -> (ExploreRuntimeSnapshot, ExplorationReadinessDetail, Option<FrontierSnapshot>) {
        guard.progress_stopped_ns.get_or_insert_with(now_unix_nanos);
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
            guard,
            self.expected_map_frame.as_ref(),
            backend_available,
            self.navigation_sink.read().is_some(),
        );
        (snapshot, readiness, frontier_update)
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
                Ok(Ok(ChildNavigationCancelOutcome::AlreadyTerminal(proof))) => {
                    return self.finish_already_terminal_child(&child.lease, proof).await;
                }
                Ok(Ok(ChildNavigationCancelOutcome::Pending(ticket)))
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

    async fn finish_already_terminal_child(
        &self,
        lease: &ChildNavigationLease,
        proof: ChildNavigationTerminalProof,
    ) -> ExploreRuntimeSnapshot {
        if !proof.lease.matches(lease) {
            return self.record_child_cancel_failure(lease,
                "navigation sink returned an invalid natural-retirement result".into(), None).await;
        }
        let guard = self.inner.lock().await;
        if guard.active_child.as_ref().is_none_or(|child| !child.lease.matches(lease))
            || guard.pending_child_cancellation.is_none()
        {
            return self.snapshot();
        }
        if guard.pending_child_cancellation.as_ref().is_some_and(|pending| pending.ticket.is_some()) {
            drop(guard);
            return self.record_child_cancel_failure(lease,
                "natural retirement cannot replace an assigned cancellation ticket".into(), None).await;
        }
        // Stop already closed source feedback admission. Complete only its
        // retained logical action; preserve the original Nav natural terminal.
        self.complete_pending_child_action(guard)
    }

    fn complete_pending_child_action(
        &self,
        mut guard: tokio::sync::MutexGuard<'_, ExploreRuntimeInner>,
    ) -> ExploreRuntimeSnapshot {
        let pending = guard
            .pending_child_cancellation
            .take()
            .expect("pending child cancellation checked above");
        guard
            .active_child
            .take()
            .expect("active child checked above");
        guard.no_frontier_since = None;
        if guard.return_after_cancellation && pending.failure.is_none() {
            guard.return_after_cancellation=false;
            let snapshot=guard.snapshot(STATUS_ACTIVE,"当前子目标已物理停稳，等待返航路线与新租约");
            drop(guard);return self.store_snapshot(snapshot);
        }
        guard.return_after_cancellation=false;
        guard.progress_stopped_ns.get_or_insert_with(now_unix_nanos);
        let (status, message, frontier_update) = if let Some(error) = pending.failure {
            guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
            self.compute_service.wake.notify_one();
            guard.last_stop = Some(ExplorationStopReason::ConsecutiveFailuresExceeded);
            guard.last_stop_reason = "child_cancel_failed".to_string();
            (
                STATUS_FAILED,
                format!("child cancellation failed before ownership settled: {error}"),
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
        self.store_snapshot(snapshot)
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
        guard.request_compute_stop(ExplorationStopReason::ConsecutiveFailuresExceeded);
        self.compute_service.wake.notify_one();
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
        let Ok(tick_permit) = self.tick_gate.clone().try_lock_owned() else { return self.snapshot(); };
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

        if let Err(error) = self.await_compute_idle().await {
            return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await;
        }
        let now_ns = now_unix_nanos();
        let mut guard = self.inner.lock().await;
        if generation != guard.generation || !guard.logical_active {
            return self.snapshot();
        }

        if guard.retained_child_dispatch {
            // Normally the original task still owns tick_gate. A panic can drop
            // that permit, but cannot authorize a replacement to clear this bit.
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                "previous retained child dispatch remains unresolved",
            );
            drop(guard);
            return self.store_snapshot(snapshot);
        }

        if self.orphan_cancellations.has_entries() {
            let snapshot = guard.snapshot(
                STATUS_PENDING,
                "recovering orphan navigation ownership before child dispatch",
            );
            drop(guard);
            self.reconcile_orphan_cancellations_with_tick_permit().await;
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

        let mission_policy=guard.active_config.as_ref().map(|c|c.return_policy.clone()).unwrap_or_default();
        if mission_policy.enabled() {
            let frame_generation=guard.surface_mode.as_ref().and_then(|m|m.latest.as_ref()).map_or(guard.costmap_frame_generation,|t|t.map_frame_generation);
            if frame_generation>0 && guard.committed_map.is_some() {
                let current=mission_pose(&pose);
                let speed=guard.mission_calibration.as_ref().expect("预算启动已捕获标定").maximum_sample_speed_mps;
                if let Err(error)=current.validate(){drop(guard);return self.finish_session(None,ExplorationStopReason::ConsecutiveFailuresExceeded,error).await;}
                let progress=guard.mission.get_or_insert_with(||MissionProgress::new(current.clone()).expect("位姿载荷已验证"));
                if let Err(error)=progress.observe(&current,pose.source_timestamp_ns,frame_generation as u64,speed) {
                    drop(guard);return self.finish_session(None,ExplorationStopReason::ConsecutiveFailuresExceeded,error).await;
                }
            }
        }
        let mut return_balance_reached = false;
        if mission_policy.maximum_distance_m.is_some() && guard.active_child.is_some()
            && guard.mission.as_ref().is_some_and(|progress| !progress.returning)
            && guard.next_return_route_check.is_none_or(|deadline| Instant::now() >= deadline)
        {
            let progress = guard.mission.clone().expect("进度已存在");
            let context = match self.capture_compute_context(&guard, &pose, now_ns) {
                Ok(context) => context,
                Err(error) => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded,
                    format!("活动探索返航路线核验失败：{error}")).await; }
            };
            let route_ticket = match self.admit_compute_locked(&mut guard, &context,
                ComputeRequest::HomewardBudget(HomewardBudgetRequest { context: context.clone(), progress })) {
                Ok(ticket) => ticket,
                Err(error) => { let snapshot = guard.snapshot(STATUS_PENDING, error); drop(guard); return self.store_snapshot(snapshot); }
            };
            guard.next_return_route_check = Some(Instant::now() + Duration::from_secs(1));
            drop(guard);
            let result = self.await_compute(&route_ticket).await;
            guard = self.inner.lock().await;
            if !route_ticket.current(&guard) { return self.snapshot(); }
            let query = match result {
                Ok(value) => match &value.projection { ComputeProjection::HomewardBudget(query) => query.clone(), _ => Err("invalid homeward compute projection".into()) },
                Err(_) => { return self.snapshot(); }
            };
            route_ticket.finish(&mut guard);
            drop(guard);
            if let Err(error) = self.await_compute_idle().await {
                return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await;
            }
            guard = self.inner.lock().await;
            if !route_ticket.current(&guard) { return self.snapshot(); }
            return_balance_reached = match query {
                Ok(value) => value,
                Err(error) => {
                    let reason = if error.is_inconclusive() { ExplorationStopReason::PlanningResourceLimited }
                        else { ExplorationStopReason::ConsecutiveFailuresExceeded };
                    drop(guard); return self.finish_session(None, reason,
                        format!("活动探索返航路线核验失败：{error}")).await;
                }
            };
        }
        // Preserve duration -> max-goals -> coverage ordering. Coverage is the
        // same full-grid calculation, evaluated only on this immutable input by
        // the owner, never by the async control thread or under inner.
        let mut condition = if guard.return_reason.is_none() { evaluate_stop_conditions(&guard, now_ns, None) } else { None };
        if condition.is_none() && guard.return_reason.is_none()
            && guard.active_config.as_ref().is_some_and(|config| config.stop_conditions.max_coverage_pct.is_some())
            && guard.costmap_input.is_some() && guard.committed_map.is_some()
            && guard.costmap_received_monotonic_ns != 0
            && now_monotonic_nanos().saturating_sub(guard.costmap_received_monotonic_ns) <= guard.costmap_receipt_ttl.as_nanos() as u64
        {
            let context = match self.capture_compute_context(&guard, &pose, now_ns) { Ok(context) => context, Err(_) => return self.snapshot() };
            let coverage_ticket = match self.admit_compute_locked(&mut guard, &context, ComputeRequest::Coverage { context: context.clone() }) {
                Ok(ticket) => ticket, Err(_) => return self.snapshot(),
            };
            drop(guard);
            let result = self.await_compute(&coverage_ticket).await;
            guard = self.inner.lock().await;
            if !coverage_ticket.current(&guard) { return self.snapshot(); }
            let coverage = match result { Ok(value) => match &value.projection { ComputeProjection::Coverage(value) => *value, _ => None }, Err(_) => None };
            condition = evaluate_stop_conditions(&guard, now_ns, coverage);
            coverage_ticket.finish(&mut guard);
            drop(guard);
            if let Err(error) = self.await_compute_idle().await {
                return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await;
            }
            guard = self.inner.lock().await;
            if !coverage_ticket.current(&guard) { return self.snapshot(); }
        }
        let exhausted=return_balance_reached || guard.mission.as_ref().is_some_and(|p|!p.returning && mission_policy.maximum_distance_m.is_some_and(|limit|p.travelled_m+mission_policy.reserve_distance_m>=limit));
        if exhausted || condition.is_some() {
            let reason=if exhausted {ExplorationStopReason::DistanceBudgetReached}else{condition.expect("条件已存在")};
            let can_return=exhausted || mission_policy.return_on_completion && reason.terminal_kind()==ExplorationTerminalKind::Succeeded;
            if can_return && guard.mission.is_some() {
                guard.return_reason=Some(reason);
                guard.mission.as_mut().expect("已检查进度").returning=true;
                if guard.active_child.is_some() {
                    drop(guard);return self.cancel_child_for_return(generation,reason).await;
                }
                guard.pending_child_dispatch=None;
            } else {
                let message=match guard.last_poll.as_ref() {
                    Some(ExplorationPoll::SourceUnavailable(detail)) if reason==ExplorationStopReason::NoReachableFreeSpace=>format!("exploration inputs unavailable at duration limit: {detail}"),
                    _=>format!("exploration stop condition reached: {}",stop_reason_str(reason)),
                };
                drop(guard);return self.finish_session(None,reason,message).await;
            }
        }

        if let Some(child) = guard.active_child.as_ref() {
            if !child.terminal_receipt_checked {
                let lease = child.lease.clone();
                drop(guard);
                return self.reconcile_child_terminal(&lease).await;
            }
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
            #[cfg(test)]
            let _span = self.attribution.as_ref().map(|p| p.begin(attribution::Stage::RuntimeInputCapture, 0));
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
                > guard.costmap_receipt_ttl.as_nanos() as u64
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
            return self.dispatch_child(generation, request, guard, tick_permit).await;
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
            let input_frame_generation = guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation);
            if sink.current_map_frame_generation()?.is_some_and(|current| current != input_frame_generation) {
                return Err("committed map coordinate generation is waiting for current input".into());
            }
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
        if guard.progress_origin.is_none() {
            let frame=guard.surface_mode.as_ref().and_then(|m|m.latest.as_ref()).map_or(guard.costmap_frame_generation,|t|t.map_frame_generation);
            if frame>0 {guard.progress_origin=Some((committed_map.session_id.clone(),committed_map.session_generation,frame));guard.progress_map_name=expected_map_name.clone();}
        }
        let context = match self.capture_compute_context(&guard, &pose, now_ns) {
            Ok(context) => context,
            Err(error) => { let snapshot = guard.snapshot(STATUS_PENDING, error); drop(guard); return self.store_snapshot(snapshot); }
        };
        let mission_progress = guard.mission.clone();
        let selection_ticket = match self.admit_compute_locked(&mut guard, &context,
            ComputeRequest::Selection(SelectionRequest { context: context.clone(), progress: mission_progress.clone() })) {
            Ok(ticket) => ticket,
            Err(error) => { let snapshot = guard.snapshot(STATUS_PENDING, error); drop(guard); return self.store_snapshot(snapshot); }
        };
        drop(guard);
        let finalized = self.await_compute(&selection_ticket).await;
        guard = self.inner.lock().await;
        if !selection_ticket.current(&guard) { return self.snapshot(); }
        let finalized = match finalized { Ok(value) => value, Err(_) => return self.snapshot() };
        let selection = match &finalized.projection { ComputeProjection::Selection(selection) => selection.clone(), _ => {
            drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, "invalid selection compute projection".into()).await;
        }};
        let mut poll_result = selection.poll;
        let diagnostics = guard.committed.diagnostics.clone();
        guard.last_selected_source = diagnostics.selected_source.unwrap_or_else(|| if surface_mode { "surface".into() } else { String::new() });
        let source_frontiers = guard.committed.frontiers.clone();
        let frontier_update = guard.update_frontiers(source_frontiers);

        if let Some(progress) = mission_progress {
            let natural = matches!(&poll_result, Ok(ExplorationPoll::Stop(reason)) if reason.terminal_kind() == ExplorationTerminalKind::Succeeded);
            let candidate = match &poll_result { Ok(ExplorationPoll::Goal(goal)) => Some(goal.clone()), _ => None };
            let returning = progress.returning || natural && mission_policy.return_on_completion;
            if returning || candidate.is_some() {
                let parent_id = guard.last_request_id.clone();
                let advice = match selection.mission {
                    Some(Ok(advice)) => advice,
                    Some(Err(error)) => {
                        let reason = if error.is_inconclusive() { ExplorationStopReason::PlanningResourceLimited }
                            else { ExplorationStopReason::ConsecutiveFailuresExceeded };
                        drop(guard); self.publish_frontiers(frontier_update);
                        return self.finish_session(None, reason, format!("返航路线核验失败：{error}")).await;
                    }
                    None => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, "missing mission route advice".into()).await; }
                };
                let homeward = advice.homeward;
                let can_visit = match advice.can_visit {
                    Ok(value) => value,
                    Err(error) => {
                        if error.is_inconclusive() {
                            // Resource limits cannot count as failed navigation,
                            // blacklist this candidate, or trigger return success.
                            selection_ticket.finish(&mut guard);
                            drop(guard); self.publish_frontiers(frontier_update);
                            return self.finish_session(None, ExplorationStopReason::PlanningResourceLimited,
                                format!("ResourceLimited: candidate mission route remains inconclusive: {error}")).await;
                        }
                        let goal = candidate.as_ref().expect("仅候选路线查询会失败");
                        let feedback = ExplorationGoalFeedback::Failed(format!("候选往返路线不可通行：{error}"));
                        let ticket = match self.admit_feedback_locked(&mut guard,
                            FeedbackIdentity::Internal { generation, lineage: format!("poll:{}:{}", selection_ticket.id.incarnation, selection_ticket.id.sequence), goal: goal.id.clone(), feedback: feedback.clone() },
                            goal.id.clone(), feedback, None, false) {
                            Ok(ticket) => ticket,
                            Err(feedback_error) => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded,
                                format!("候选路线失败反馈未完成：{feedback_error}")).await; }
                        };
                        selection_ticket.feedback_handoff(&mut guard);
                        drop(guard);
                        let acknowledged = self.await_feedback(&ticket).await;
                        guard = self.inner.lock().await;
                        if generation != guard.generation || !guard.logical_active || guard.last_request_id != parent_id { return self.snapshot(); }
                        if let Err(feedback_error) = acknowledged { drop(guard); self.publish_frontiers(frontier_update);
                            return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded,
                                format!("候选路线失败反馈未完成：{feedback_error}")).await; }
                        guard.last_poll = Some(ExplorationPoll::Pending);
                        guard.no_frontier_since = None;
                        selection_ticket.finish(&mut guard);
                        let snapshot = guard.snapshot(STATUS_PENDING, format!("候选往返路线不可通行，等待其它候选：{error}"));
                        drop(guard); self.publish_frontiers(frontier_update); return self.store_snapshot(snapshot);
                    }
                };
                if returning || !can_visit {
                    if let Some(goal) = candidate.as_ref() {
                        let feedback = ExplorationGoalFeedback::Cancelled;
                        let ticket = match self.admit_feedback_locked(&mut guard,
                            FeedbackIdentity::Internal { generation, lineage: format!("poll:{}:{}", selection_ticket.id.incarnation, selection_ticket.id.sequence), goal: goal.id.clone(), feedback: feedback.clone() },
                            goal.id.clone(), feedback, None, false) {
                            Ok(ticket) => ticket,
                            Err(error) => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
                        };
                        selection_ticket.feedback_handoff(&mut guard);
                        drop(guard);
                        let acknowledged = self.await_feedback(&ticket).await;
                        guard = self.inner.lock().await;
                        if generation != guard.generation || !guard.logical_active || guard.last_request_id != parent_id { return self.snapshot(); }
                        if let Err(error) = acknowledged { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
                        if !selection_ticket.current_after_feedback(&guard, &ticket) {
                            selection_ticket.finish(&mut guard);
                            guard.no_frontier_since = None;
                            let snapshot = guard.snapshot(STATUS_PENDING, "mission source changed during candidate retirement; waiting for fresh route advice");
                            drop(guard);
                            return self.store_snapshot(snapshot);
                        }
                    }
                    let reason = if progress.returning { guard.return_reason.unwrap_or(ExplorationStopReason::DistanceBudgetReached) }
                        else if natural { match poll_result { Ok(ExplorationPoll::Stop(reason)) => reason, _ => unreachable!() } }
                        else { ExplorationStopReason::DistanceBudgetReached };
                    guard.return_reason = Some(reason);
                    guard.mission.as_mut().expect("同一运行进度").returning = true;
                    let mut home = FrontierGoal::new(format!("{}.return.{}", parent_id, guard.child_sequence + 1),
                        PoseStamped { frame_id: context.robot_pose.frame_id.clone(), stamp: context.now, pose: pose3(&progress.home) }, 0.0);
                    home.producer = "mission_return".into(); home.distance_cost = Some(homeward as f32);
                    guard.return_goal_id = Some(home.id.clone());
                    poll_result = Ok(ExplorationPoll::Goal(home));
                }
            }
        }

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
                if guard.return_goal_id.as_deref()==Some(goal.id.as_str()) {
                    navigation_goal.position_tolerance_m=0.1;
                    navigation_goal.yaw_tolerance_rad=0.2;
                }
                let (motion_admission, intent_result) = {
                    let default_config;
                    let config = match guard.active_config.as_deref() {
                        Some(config) => config,
                        None => {
                            default_config = ExploreStartConfig::default();
                            &default_config
                        }
                    };
                    (
                        config.motion_admission,
                        build_wavefront_goal_intent(
                            guard.session_generation,
                            cancel_generation,
                            &guard.last_request_id,
                            &child_request_id,
                            &config.motion_admission,
                            &navigation_goal,
                            config.boundary.as_ref(),
                            committed_map,
                        ),
                    )
                };
                let mut intent = match intent_result {
                    Ok(intent) => intent,
                    Err(error) => {
                selection_ticket.finish(&mut guard);
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
                if let Some(surface)=&guard.surface_mode {
                    // Return endpoints are the captured body pose. Only ordinary surface
                    // frontier coordinates were derived through support -> body.
                    intent.body_to_support_z_m=if guard.return_goal_id.as_deref()==Some(goal.id.as_str()) { None } else {
                        surface.config.as_ref().ok().map(|config|config.follower.body_to_support_z_m)
                    };
                }
                intent.options = if guard.return_goal_id.as_deref() == Some(goal.id.as_str()) {
                    crate::explore_mission::home_goal_options()
                } else {
                    crate::explore_mission::frontier_goal_options(
                        &goal,
                        guard.surface_mode.is_some(),
                        guard.surface_mode.as_ref().and_then(|surface| surface.config.as_ref().ok()),
                    )
                };
                intent.map_frame_generation = guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation);
                let request = ChildNavigationRequest {
                    parent_request_id: guard.last_request_id.clone(),
                    request_id: child_request_id,
                    expected_map_name,
                    motion_admission,
                    frontier_goal_id: goal.id.clone(),
                    goal: navigation_goal,
                    intent,
                    boundary_owner: Arc::clone(guard.active_config.as_ref().expect("active exploration owns its start config")),
                };
                let dispatching = guard.snapshot(
                    STATUS_PENDING,
                    format!("dispatching frontier {} to child navigation", goal.id),
                );
                self.publish_frontiers(frontier_update);
                self.store_snapshot(dispatching);
                guard.pending_child_dispatch = Some(request.clone());
                selection_ticket.finish(&mut guard);
                self.dispatch_child(generation, request, guard, tick_permit).await
            }
            Ok(ExplorationPoll::NoFrontier) => {
                selection_ticket.finish(&mut guard);
                // The retained driver decided the stability timeout exactly once
                // at committed-result delivery, even if this waiter was delayed.
                let message = if guard.return_reason == Some(ExplorationStopReason::NoFrontierTimeout)
                    && guard.mission.as_ref().is_some_and(|progress| progress.returning) {
                    "探索前沿已稳定结束，准备返航"
                } else { "no frontier available yet" };
                let snapshot = guard.snapshot(STATUS_PENDING, message);
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.store_snapshot(snapshot)
            }
            Ok(ExplorationPoll::Pending) => {
                selection_ticket.finish(&mut guard);
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
                selection_ticket.finish(&mut guard);
                guard.no_frontier_since = None;
                guard.last_poll = Some(ExplorationPoll::SourceUnavailable(reason.clone()));
                let snapshot =
                    guard.snapshot(STATUS_PENDING, format!("source unavailable: {reason}"));
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.store_snapshot(snapshot)
            }
            Ok(ExplorationPoll::Stop(reason)) => {
                selection_ticket.finish(&mut guard);
                drop(guard);
                self.publish_frontiers(frontier_update);
                self.finish_session(
                    None,
                    reason,
                    exploration_stop_message(reason),
                )
                .await
            }
            Err(error) => {
                selection_ticket.finish(&mut guard);
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
        tick_permit: tokio::sync::OwnedMutexGuard<()>,
    ) -> ExploreRuntimeSnapshot {
        if generation != guard.generation || !guard.logical_active {
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
        // Freeze the exact request before releasing Explore state. Navigation
        // admission is async and must never hold the Explore inner lock.
        guard.pending_child_dispatch = Some(request.clone());

        let operation = ChildNavigationDispatch {
            runtime: self.clone(),
            generation,
            request,
            sink: sink.clone(),
            tick_permit,
        };
        // Hold original inner while original admission accepts/rejects the
        // future. A started task may run, but Stop/projection cannot observe it
        // before this synchronous accepted-ownership bit becomes visible.
        let observation = sink.retain_child_dispatch(operation);
        if matches!(&observation, Ok(ChildNavigationDispatchObserver::Retained(_))) {
            guard.retained_child_dispatch = true;
        }
        drop(guard);
        match observation {
            Ok(observer) => observer.observe().await.unwrap_or_else(|| self.snapshot()),
            Err(error) => {
                // No submit was polled. Keep the frozen request for the original
                // scheduler cadence; this is not a goal rejection/budget event.
                let guard = self.inner.lock().await;
                if generation != guard.generation || !guard.logical_active {
                    return self.snapshot();
                }
                self.store_snapshot(guard.snapshot(
                    STATUS_PENDING,
                    format!("child dispatch owner unavailable: {error}"),
                ))
            }
        }
    }

    async fn complete_child_dispatch(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        sink: Arc<dyn ExploreNavigationSink>,
        owner: Option<&dyn ChildNavigationDispatchOwner>,
    ) -> ExploreRuntimeSnapshot {
        let proposed_request_id = request.request_id.clone();
        let expected_parent_request_id = request.parent_request_id.clone();
        let frontier_goal_id = request.frontier_goal_id.clone();
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
        let mut attached = false;
        {
            let mut decide = |admission_open: bool| {
                let frozen_request_matches = guard.pending_child_dispatch.as_ref().is_some_and(|pending| {
                    pending.request_id == request.request_id
                        && pending.parent_request_id == request.parent_request_id
                        && pending.intent == request.intent
                });
                if !admission_open || generation != guard.generation
                    || !guard.logical_active || !frozen_request_matches
                {
                    // Synchronous visibility before releasing either state lock.
                    // The original tick permit excludes all batch reconcilers.
                    self.orphan_cancellations.register(ticket.lease.clone());
                    return;
                }
                guard.pending_child_dispatch = None;
                let child_request_id = ticket.lease.session_id.clone();
                guard.active_child = Some(ActiveChildNavigation {
                    request_id: child_request_id.clone(),
                    frontier_goal_id: frontier_goal_id.clone(),
                    lease: ticket.lease.clone(),
                    terminal_receipt_checked: false,
                });
                // Publish identity in the SAME non-cancellable admission segment.
                self.store_snapshot(guard.snapshot(
                    STATUS_ACTIVE,
                    format!("child navigation accepted: {child_request_id}"),
                ));
                attached = true;
            };
            if let Some(owner) = owner {
                owner.with_dispatch_admission(&mut decide);
            } else {
                // Explicit cfg(test) inline sinks retain their original contract.
                decide(true);
            }
        }
        drop(guard);
        if attached {
            return self.reconcile_child_terminal(&ticket.lease).await;
        }
        if let Some(owner) = owner {
            // The task that published also owns the exact orphan continuation.
            // No new root admission is needed, even after root close wins.
            owner.settle_retained_dispatch_orphan(ticket.lease).await;
        } else {
            self.reconcile_orphan_cancellation(&sink, ticket.lease).await;
        }
        self.snapshot()
    }

    async fn reconcile_child_terminal(&self, lease: &ChildNavigationLease) -> ExploreRuntimeSnapshot {
        let Some(sink) = self.navigation_sink.read().clone() else { return self.snapshot(); };
        if let Some(result) = sink.terminal_result_for(lease).await {
            // Admission itself persists before report's first completion await.
            // Concurrent live/recovery delivery shares its original identity.
            if let Err(error) = self.report_navigation_result(&lease.session_id, result).await {
                tracing::debug!(child_request_id = %lease.session_id, %error,
                    "retained child terminal no longer admitted by current exploration action");
            }
        } else {
            let mut guard = self.inner.lock().await;
            if let Some(child) = guard.active_child.as_mut().filter(|child| child.lease.matches(lease)) {
                child.terminal_receipt_checked = true;
            }
        }
        self.snapshot()
    }

    async fn record_dispatch_failure(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        error: ChildNavigationSubmitError,
    ) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if generation != guard.generation || !guard.logical_active {
            return self.snapshot();
        }
        let reason = error.reason().to_string();
        if matches!(error, ChildNavigationSubmitError::RejectedBeforePublish(_)) {
            // The existing frozen pending request is the one-use admission token.
            // Check it before home identity so an old result cannot clear a newer
            // dispatch merely because return_goal_id has already advanced.
            let pending_matches = guard.pending_child_dispatch.as_ref().is_some_and(|pending| {
                pending.request_id == request.request_id
                    && pending.parent_request_id == request.parent_request_id
            });
            if !pending_matches {
                return self.snapshot();
            }
            if guard.return_goal_id.as_deref() == Some(request.frontier_goal_id.as_str()) {
                guard.pending_child_dispatch = None;
                guard.no_frontier_since = None;
                guard.return_failures = guard.return_failures.saturating_add(1);
                if guard.return_failures
                    > guard.active_config.as_ref().map_or(0, |config| config.max_navigation_retries)
                {
                    let action = PendingChildAction {
                        reason: ExplorationStopReason::ConsecutiveFailuresExceeded,
                        message: format!("返航派发在投递前被拒绝，已耗尽重试次数：{reason}"),
                    };
                    let generation = self.next_exploration_generation(&mut guard);
                    self.record_child_action_stop_locked(&mut guard, generation, action.reason);
                    // Current frozen pending dispatch excludes an attached child:
                    // tick gates children/cancellation, and attach consumes pending.
                    let (snapshot, readiness, frontier_update) =
                        self.prepare_no_child_stop_locked(&mut guard, action);
                    // Only this new synchronous rejection path publishes while
                    // holding inner; the public Stop publication order is unchanged.
                    *self.readiness_cache.lock() = readiness;
                    self.publish_frontiers(frontier_update);
                    return self.store_snapshot(snapshot);
                }
                let snapshot = guard.snapshot(
                    STATUS_PENDING,
                    format!("返航派发在投递前被拒绝，等待重新核验路线：{reason}"),
                );
                return self.store_snapshot(snapshot);
            }
        }
        if matches!(error, ChildNavigationSubmitError::SourceChanged(_)) {
            guard.pending_child_dispatch = None;
            guard.no_frontier_since = None;
            let feedback = ExplorationGoalFeedback::Cancelled;
            let ticket = match self.admit_feedback_locked(&mut guard,
                FeedbackIdentity::Internal { generation, lineage: request.request_id.clone(), goal: request.frontier_goal_id.clone(), feedback: feedback.clone() },
                request.frontier_goal_id.clone(), feedback, None, false) {
                Ok(ticket) => ticket, Err(error) => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
            };
            drop(guard);
            let acknowledged = self.await_feedback(&ticket).await;
            guard = self.inner.lock().await;
            if generation != guard.generation || !guard.logical_active { return self.snapshot(); }
            if let Err(error) = acknowledged { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
            let snapshot = guard.snapshot(STATUS_PENDING, format!("waiting for refreshed exploration input: {reason}"));
            drop(guard);
            return self.store_snapshot(snapshot);
        }
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
        let feedback = ExplorationGoalFeedback::Failed(reason.clone());
        let ticket = match self.admit_feedback_locked(&mut guard,
            FeedbackIdentity::Internal { generation, lineage: request.request_id.clone(), goal: request.frontier_goal_id.clone(), feedback: feedback.clone() },
            request.frontier_goal_id.clone(), feedback, None, false) {
            Ok(ticket) => ticket, Err(error) => { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
        };
        drop(guard);
        let acknowledged = self.await_feedback(&ticket).await;
        guard = self.inner.lock().await;
        if generation != guard.generation || !guard.logical_active { return self.snapshot(); }
        if let Err(error) = acknowledged { drop(guard); return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await; }
        let frontiers = guard.committed.frontiers.clone();
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
        if let Some(ticket) = self.existing_external_feedback_locked(&guard, child_request_id, &result)? {
            drop(guard);
            self.await_feedback(&ticket).await?;
            return Ok(self.snapshot());
        }
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
                    let _motion_admission = guard
                        .active_config
                        .as_ref()
                        .map(|config| config.motion_admission)
                        .ok_or_else(|| {
                            "late exact ZEROED has no original motion admission".to_string()
                        })?;
                    let recovery_correlation = assigned_recovery_correlation(&active_child.lease, ticket.cancel_generation);
                    let request_identity = exact_child_zero_request_identity(proof);
                    let receipt = ExplorationOwnershipReconciliationReceipt {
                        cause: EXACT_CHILD_ZEROED_CAUSE.to_string(),
                        recovery_correlation: recovery_correlation.clone(),
                        parent_request_id: guard.last_request_id.clone(),
                        exploration_generation: guard.session_generation,
                        child_lease_fingerprint: String::new(),
            child_lease: Some(active_child.lease.clone()),
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
                        request_fingerprint: String::new(),
                        request_identity: Some(request_identity.clone()),
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
                            request_identity,
                            receipt,
                        }),
                    );
                }
            }

            return Ok(self.complete_pending_child_action(guard));
        }

        let frontier_goal_id = active_child.frontier_goal_id.clone();
        if guard.return_goal_id.as_deref()==Some(frontier_goal_id.as_str()) {
            guard.active_child=None;
            guard.no_frontier_since=None;
            let reason=guard.return_reason.unwrap_or(ExplorationStopReason::DistanceBudgetReached);
            match &result {
                ChildNavigationResult::Reached(_)=>{
                    let distance=guard.mission.as_ref().map_or(0.0,|p|p.travelled_m);
                    drop(guard);return Ok(Box::pin(self.finish_session(None,reason,format!("返航已到达并停稳，累计行程{distance:.3}米"))).await);
                }
                ChildNavigationResult::Failed{reason:failure,..}=>{
                    guard.return_failures=guard.return_failures.saturating_add(1);
                    if guard.return_failures>guard.active_config.as_ref().map_or(0,|c|c.max_navigation_retries){
                        let message=format!("返航失败且已停稳：{failure}");drop(guard);
                        return Ok(Box::pin(self.finish_session(None,ExplorationStopReason::ConsecutiveFailuresExceeded,message)).await);
                    }
                    let snapshot=guard.snapshot(STATUS_ACTIVE,format!("返航已停稳，重新核验路线：{failure}"));drop(guard);return Ok(self.store_snapshot(snapshot));
                }
                _=>return Err("返航收到没有取消事务的取消终态".into()),
            }
        }
        let input_interrupted=matches!(&result,ChildNavigationResult::Failed{reason,..} if reason.starts_with("SafetyFailed cause=StaleInput:"));
        let feedback = match &result {
            ChildNavigationResult::Reached(_) => ExplorationGoalFeedback::Reached,
            ChildNavigationResult::Failed { reason, .. } => {
                if input_interrupted {
                    tracing::warn!(child_request_id,frontier_goal_id,%reason,"子导航输入中断且已停车，等待新鲜输入，不将前沿列入黑名单");
                    ExplorationGoalFeedback::InputUnavailable(reason.clone())
                }else{
                    tracing::warn!(child_request_id, frontier_goal_id, %reason, "探索子导航失败，进入原重试策略");
                    ExplorationGoalFeedback::Failed(reason.clone())
                }
            }
            ChildNavigationResult::Cancelled(_) | ChildNavigationResult::CancelFailed { .. } => {
                return Err("unexpected cancellation terminal without a pending action".to_string())
            }
        };
        let message = match &result {
            ChildNavigationResult::Failed {..} if input_interrupted => "child navigation input interrupted; waiting for fresh inputs",
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
        let reached = matches!(&result, ChildNavigationResult::Reached(_));
        let ticket = self.admit_feedback_locked(&mut guard,
            FeedbackIdentity::External { child: child_request_id.to_string(), terminal: result },
            frontier_goal_id, feedback, Some(message), reached)?;
        // The exact child and terminal stay retained until the persistent driver
        // applies this acknowledgment, even if this caller is cancelled.
        drop(guard);
        self.await_feedback(&ticket).await?;
        Ok(self.snapshot())
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
        self.ingest_costmap_with_identity(snapshot, None, 0, None).await
    }

    pub async fn ingest_committed_costmap_snapshot(
        &self,
        snapshot: ExploreCostmapSnapshot,
        committed_map: schema::CommittedMapIdentity,
    ) -> Result<bool, String> {
        self.ingest_costmap_with_identity(snapshot, Some(committed_map), 0, None)
            .await
    }

    pub(crate) async fn ingest_committed_costmap_snapshot_in_frame(&self, snapshot: ExploreCostmapSnapshot,
        committed_map: schema::CommittedMapIdentity, frame_generation: i64,
    ) -> Result<bool, String> {
        self.ingest_costmap_with_identity(snapshot, Some(committed_map), frame_generation, None).await
    }

    /// 二维探索消费与导航一致的真实扫描派生视图，保持原规范地图身份与源时效。
    pub(crate) async fn ingest_planar_navigation(&self, value: &nuwax_codec::lcm::planar_navigation::PlanarNavigationMap) -> Result<bool, String> {
        use nuwax_nav_types_messages::grid::OccupancyCell;
        value.validate().map_err(|e| e.to_string())?;
        if value.cleared { return Err("撤销消息不能作为二维探索图".into()); }
        if value.source_timestamp_ns as u64 > now_unix_nanos().saturating_add(50_000_000) { return Err("二维派生探索源位于未来".into()); }
        let source_age = now_unix_nanos().saturating_sub(value.source_timestamp_ns as u64);
        let ttl = Duration::from_nanos(nuwax_codec::lcm::planar_navigation::MAX_SOURCE_AGE_NS as u64);
        if source_age > ttl.as_nanos() as u64 { return Err("二维派生探索源已经过期".into()); }
        let snapshot = ExploreCostmapSnapshot { grid: OccupancyGrid2D::new(
            nuwax_nav_types_messages::geometry::Point2::new(value.grid.info.origin.position[0], value.grid.info.origin.position[1]),
            f64::from(value.grid.info.resolution), value.grid.info.width as u32, value.grid.info.height as u32,
            value.grid.data.iter().map(|v| match v { -1 => OccupancyCell::Unknown, 0 => OccupancyCell::Free, _ => OccupancyCell::Occupied }).collect()),
            frame_id: value.grid.header.frame_id.clone() };
        let identity = schema::CommittedMapIdentity { session_id: value.session_id.clone(), session_generation: value.session_generation,
            revision: value.map_revision, commit_boot_nonce: value.map_boot_nonce,
            commit_sequence: value.map_commit_sequence, source_timestamp_ns: value.map_source_timestamp_ns,
            frame_id: value.grid.header.frame_id.clone() };
        self.ingest_costmap_with_identity(snapshot, Some(identity), value.map_frame_generation,
            Some((now_monotonic_nanos().saturating_sub(source_age), ttl))).await
    }

    async fn ingest_costmap_with_identity(
        &self,
        snapshot: ExploreCostmapSnapshot,
        committed_map: Option<schema::CommittedMapIdentity>,
        frame_generation: i64,
        observation: Option<(u64, Duration)>,
    ) -> Result<bool, String> {
        if frame_generation < 0 { return Err("二维探索坐标代际无效".into()); }
        validate_costmap_snapshot(&snapshot, self.expected_map_frame.as_ref())?;
        if let Some(identity) = committed_map.as_ref() {
            if identity.session_id.trim().is_empty()
                || identity.session_generation <= 0
                || identity.revision <= 0
                || identity.commit_boot_nonce == [0; 16]
                || identity.commit_sequence <= 0
                || identity.source_timestamp_ns < 0
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
            let changed = guard.costmap_input.as_deref() != Some(&snapshot);
            guard.costmap_input = Some(Arc::new(snapshot));
            return Ok(changed);
        }
        let (received, ttl) = observation.unwrap_or_else(|| (now_monotonic_nanos(), DEFAULT_COSTMAP_RECEIPT_TTL));
        if guard.costmap_input.as_deref() == Some(&snapshot)
            && guard.committed_map.as_ref() == committed_map.as_ref()
            && guard.costmap_frame_generation == frame_generation
        {
            // 新的输入接收事件可以续期，状态查询或缓存读取不会进入这里。
            guard.costmap_received_monotonic_ns = received;
            guard.costmap_receipt_ttl = ttl;
            return Ok(false);
        }
        if let Some(identity) = committed_map.as_ref() {
            if guard.committed_map.as_ref().is_some_and(|current| {
                let same_session = identity.session_id == current.session_id
                    && identity.session_generation == current.session_generation;
                same_session
                    && (identity.commit_boot_nonce != current.commit_boot_nonce
                        || frame_generation < guard.costmap_frame_generation || identity.revision < current.revision
                        || (identity.revision == current.revision && identity != current))
            }) {
                return Err("committed costmap identity is stale or conflicting".to_string());
            }
        }
        let finish_request = if (guard.logical_active || guard.compute.start_reservation.is_some()) && guard.committed_map.as_ref().zip(committed_map.as_ref()).is_some_and(|(old, new)|
            old.session_id != new.session_id || old.session_generation != new.session_generation || guard.costmap_frame_generation != frame_generation) {
            Some(guard.last_request_id.clone())
        } else { None };
        // 缺少提交身份的输入必须清除旧身份，避免把未提交栅格冒充为已提交快照。
        guard.committed_map = committed_map;
        guard.costmap_frame_generation = frame_generation;
        guard.costmap_received_monotonic_ns = received;
        guard.costmap_receipt_ttl = ttl;

        // 实时地图只刷新下一轮探索输入，不取消当前目标。
        guard.costmap_input = Some(Arc::new(snapshot));
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
        if let Some(request_id) = finish_request {
            self.finish_session(Some(&request_id), ExplorationStopReason::MapRevisionChanged, "二维探索地图域或坐标代际变化".into()).await;
        }

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
        // Stop may clear pending while its accepted dispatch is still running.
        // Pure pose/compute work under tick_gate is NOT a motion-safety blocker.
        guard.retained_child_dispatch
            || guard.logical_active
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
        let controller_active = guard.logical_active;
        let source_count = guard.committed.source_count;
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
            active_source: guard.committed.diagnostics.clone()
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

    pub(crate) async fn checkpoint_progress(&self,request_id:&str)->Result<String,String> {
        let initial = self.inner.lock().await;
        if request_id.is_empty() || request_id != initial.last_request_id { return Err("探索进度请求身份不匹配".into()); }
        if initial.logical_active || initial.compute.start_reservation.is_some() || initial.active_child.is_some()
            || initial.pending_child_cancellation.is_some() || initial.pending_child_dispatch.is_some() || self.orphan_cancellations.has_entries() {
            return Err("探索进度只可在原任务停止且导航拥有权已确认释放后保存".into());
        }
        let stopped_fence = (initial.generation, initial.committed_map.clone(), initial.progress_origin.clone(),
            initial.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(initial.costmap_frame_generation, |ticket| ticket.map_frame_generation));
        drop(initial);
        // Accepted feedback may still be mutating old confirmed history. Capture
        // export fields only after it and old-session end have acknowledged.
        self.await_compute_idle().await?;
        let guard=self.inner.lock().await;
        if guard.generation != stopped_fence.0 || guard.committed_map != stopped_fence.1 || guard.progress_origin != stopped_fence.2
            || guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation) != stopped_fence.3 {
            return Err("探索停止代际或地图域在等待保存时发生变化".into());
        }
        if request_id.is_empty() || request_id!=guard.last_request_id{return Err("探索进度请求身份不匹配".into());}
        if guard.logical_active || guard.active_child.is_some() || guard.pending_child_cancellation.is_some() || guard.pending_child_dispatch.is_some() || self.orphan_cancellations.has_entries(){
            return Err("探索进度只可在原任务停止且导航拥有权已确认释放后保存".into());
        }
        let config=guard.active_config.as_ref().ok_or("探索配置不存在")?;
        let map=guard.committed_map.as_ref().ok_or("探索已提交地图不存在")?;
        let frame=guard.surface_mode.as_ref().and_then(|m|m.latest.as_ref()).map_or(guard.costmap_frame_generation,|t|t.map_frame_generation);
        let origin=guard.progress_origin.as_ref().ok_or("探索尚未消费有效坐标输入")?;
        if origin.0!=map.session_id || origin.1!=map.session_generation || origin.2!=frame{return Err("探索历史的坐标域已变化，不能保存成当前地图进度".into());}
        let generation = guard.generation;
        let history = guard.compute.history;
        let domain = (map.clone(), frame, origin.clone());
        let mut saved=ExplorationCheckpoint {schema_version:2,request_id:request_id.into(),map_name:guard.progress_map_name.clone(),map_frame_id:self.expected_map_frame.to_string(),
            origin_session_id:origin.0.clone(),origin_session_generation:origin.1,origin_frame_generation:origin.2,
            configuration:Some(ExplorationConfiguration::capture(config, guard.surface_mode.is_some())),
            navigation_configuration:Some(guard.navigation_configuration.as_ref().ok_or("探索启动配置不存在")?.as_ref().map_err(Clone::clone)?.clone()),
            configuration_sha256:None,navigation_configuration_sha256:None,
            elapsed_ns:guard.progress_stopped_ns.unwrap_or_else(now_unix_nanos).saturating_sub(guard.started_at_ns),reached_goals:guard.reached_goals,mission:guard.mission.clone(),return_reason:guard.return_reason,
            sources:Vec::new()};
        drop(guard);
        self.await_compute_idle().await?;
        let mut guard = self.inner.lock().await;
        let unchanged = |guard: &ExploreRuntimeInner| !guard.logical_active && guard.generation == generation
            && guard.compute.history == history && guard.last_request_id == request_id
            && guard.active_child.is_none() && guard.pending_child_cancellation.is_none() && guard.pending_child_dispatch.is_none()
            && guard.committed_map.as_ref() == Some(&domain.0)
            && guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation) == domain.1
            && guard.progress_origin.as_ref() == Some(&domain.2);
        if !unchanged(&guard) { return Err("探索停止代际或历史在保存前发生变化".into()); }
        let export = self.admit_lifecycle_locked(&mut guard, ComputeLifecycle::ExportCheckpoint)?;
        drop(guard);
        let output = self.await_lifecycle(&export).await?;
        let guard = self.inner.lock().await;
        if !unchanged(&guard) || self.orphan_cancellations.has_entries() { return Err("探索停止代际或历史在保存时发生变化".into()); }
        saved.sources = match output.as_ref() { ComputeLifecycleOutput::Checkpoint(sources) => sources.clone(), _ => return Err("invalid exploration checkpoint acknowledgment".into()) };
        drop(guard);
        let payload=serde_json::to_string(&saved).map_err(|e|e.to_string())?;
        ExplorationCheckpoint::parse(&payload)?;
        Ok(payload)
    }

    pub async fn diagnostics(&self) -> ControllerDiagnosticsView {
        let guard = self.inner.lock().await;
        let diagnostics = guard.committed.diagnostics.clone();
        ControllerDiagnosticsView {
            active: guard.logical_active,
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
    coverage: Option<f64>,
) -> Option<ExplorationStopReason> {
    let config = guard.active_config.as_ref()?;
    let conditions = &config.stop_conditions;
    if conditions.max_duration_ms.is_some_and(|limit_ms| {
        now_ns.saturating_sub(guard.started_at_ns) >= limit_ms.saturating_mul(1_000_000)
    }) {
        // 一直缺少可用起点、尚未选择目标时，期限耗尽不能伪装成探索成功。
        if guard.child_sequence == 0
            && matches!(
                guard.last_poll.as_ref(),
                Some(ExplorationPoll::SourceUnavailable(_))
            )
        {
            return Some(ExplorationStopReason::NoReachableFreeSpace);
        }
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
                <= guard.costmap_receipt_ttl.as_nanos() as u64
            && coverage.is_some_and(|coverage| coverage >= max_coverage)
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
    compute_runtime: ExploreRuntimeHandle,
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
        let (shutdown_tx, mut shutdown_rx) = watch::channel(false);
        #[cfg(test)]
        let topology_control = crate::lcm_runtime::topology_test_control();
        let period = if period.is_zero() {
            DEFAULT_SCHEDULER_PERIOD
        } else {
            period
        };
        let compute_runtime = runtime.clone();
        let worker = tokio::spawn(async move {
            #[cfg(test)]
            crate::lcm_runtime::wait_for_topology_scheduler_release(topology_control);
            let mut interval = tokio::time::interval(period);
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tokio::select! {
                    _ = interval.tick() => {
                        if !runtime.is_active().await {
                            continue;
                        }
                        match pose_provider.current_pose().await {
                            Ok(pose) => {
                                let _ = runtime.tick_with_pose(pose).await;
                            }
                            Err(error) => {
                                // 决策层允许低频输入。活动子导航由执行守卫按源时效停车，
                                // 探索保留任务并等待其具名终态，不能抢先终止整个父任务。
                                let mut guard = runtime.inner.lock().await;
                                if !guard.logical_active || guard.pending_child_cancellation.is_some() {
                                    continue;
                                }
                                guard.no_frontier_since = None;
                                guard.last_poll = Some(ExplorationPoll::SourceUnavailable(error.clone()));
                                let expired = guard.active_config.as_ref().and_then(|config| config.stop_conditions.max_duration_ms)
                                    .is_some_and(|limit| now_unix_nanos().saturating_sub(guard.started_at_ns) >= limit.saturating_mul(1_000_000));
                                if expired {
                                    drop(guard);
                                    let _ = runtime.finish_session(None, ExplorationStopReason::NoReachableFreeSpace,
                                        format!("任务期限内位姿输入未恢复：{error}")).await;
                                    continue;
                                }
                                let status = if guard.active_child.is_some() { STATUS_ACTIVE } else { STATUS_PENDING };
                                let snapshot = guard.snapshot(status, format!("等待位姿输入恢复，目标决策已保留：{error}"));
                                runtime.store_snapshot(snapshot);
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
                compute_runtime,
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
        let scheduler = async {
            let result = self.wait_final_result().await;
            if result.is_ok() && self.shutdown_is_requested() {
                std::future::pending::<Result<(), String>>().await
            } else { result }
        };
        let compute = async {
            let result = self.inner.compute_runtime.compute_service.wait_terminal().await;
            if result.is_ok() && self.shutdown_is_requested() {
                std::future::pending::<Result<(), String>>().await
            } else { result }
        };
        // Suppress each clean requested terminal independently: one settled
        // producer must not hide an unexpected failure in the other producer.
        tokio::select! { result = scheduler => result, result = compute => result }
    }

    pub async fn shutdown(&self) -> Result<(), String> {
        self.shutdown_with_settlement().await.into_result()
    }

    async fn with_compute_settlement(&self, scheduler_result: Result<(), String>) -> ExploreSchedulerShutdownOutcome {
        let mut deadline = match WallClockWake::new(SCHEDULER_SHUTDOWN_TIMEOUT) {
            Ok(deadline) => deadline,
            Err(error) => return ExploreSchedulerShutdownOutcome::new(Err(error), false),
        };
        tokio::select! {
            biased;
            compute = self.inner.compute_runtime.compute_service.join() => {
                ExploreSchedulerShutdownOutcome::new(scheduler_result.and(compute), true)
            }
            _ = deadline.wait() => ExploreSchedulerShutdownOutcome::new(
                Err("exploration compute owner settlement timed out; original worker and driver remain supervised".into()), false),
        }
    }

    pub(crate) async fn shutdown_with_settlement(&self) -> ExploreSchedulerShutdownOutcome {
        self.inner.shutdown_tx.send_replace(true);
        self.inner.compute_runtime.request_compute_shutdown().await;
        match self.inner.monitor_state.load(Ordering::Acquire) {
            SCHEDULER_MONITOR_SETTLED => {
                return self.with_compute_settlement(self.wait_final_result().await).await;
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
                return self.with_compute_settlement(self.wait_final_result().await).await;
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
                    return self.with_compute_settlement(self.wait_final_result().await).await;
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
                    self.with_compute_settlement(self.wait_final_result().await).await
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
    mod scheduler_attribution { include!("explore_scheduler_attribution_tests.rs"); }

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
        receipt_reads: AtomicU64,
        receipt_entered: tokio::sync::Notify,
        receipt_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
        terminal_result: StdMutex<Option<ChildNavigationResult>>,
        cancel_outcome: StdMutex<Option<ChildNavigationCancelOutcome>>,
        next_goal_generation: AtomicU64,
        next_cancel_generation: AtomicU64,
        submit_entered: tokio::sync::Notify,
        submit_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
        cancel_entered: tokio::sync::Notify,
        cancel_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
    }

    #[async_trait]
    impl ExploreNavigationSink for RecordingNavSink {
        fn retain_child_dispatch(
            &self,
            operation: crate::explore_runtime::ChildNavigationDispatch,
        ) -> Result<crate::explore_runtime::ChildNavigationDispatchObserver, String> {
            // Legacy fake-sink contract; this fixture does not prove root ownership.
            Ok(crate::explore_runtime::ChildNavigationDispatchObserver::inline(operation))
        }

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

                });
            Ok(ChildNavigationTicket { lease })
        }

        async fn terminal_result_for(&self, _lease: &ChildNavigationLease) -> Option<ChildNavigationResult> {
            self.receipt_reads.fetch_add(1, Ordering::SeqCst);
            let release = { self.receipt_release.lock().unwrap().clone() };
            if let Some(release) = release {
                self.receipt_entered.notify_one();
                release.notified().await;
            }
            self.terminal_result.lock().unwrap().clone()
        }

        async fn cancel(
            &self,
            lease: &ChildNavigationLease,
        ) -> Result<ChildNavigationCancelOutcome, String> {
            if let Some(outcome) = self.cancel_outcome.lock().unwrap().clone() { return Ok(outcome); }
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
            Ok(ChildNavigationCancelOutcome::Pending(ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: self.next_cancel_generation.fetch_add(1, Ordering::SeqCst) + 1,
            }))
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

    async fn receipt_test_source_history(handle: &ExploreRuntimeHandle) -> serde_json::Value {
        let sources = handle.inspect_compute_checkpoint().await.expect("owner checkpoint inspection");
        let wavefront = sources.iter().find(|source| source.name == "wavefront").unwrap();
        serde_json::from_str(&wavefront.payload).unwrap()
    }

    fn assert_one_receipt_source_failure(history: &serde_json::Value) {
        let failures = history[1]["failed_goals"].as_array().expect("Wavefront failed-goal history");
        assert_eq!(failures.len(), 1);
        assert_eq!(failures[0]["failures"], serde_json::json!(1));
        assert_eq!(history[1]["blacklisted_goals"], serde_json::json!([]));
    }

    #[tokio::test]
    async fn absent_child_terminal_receipt_is_read_once_per_child() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("receipt-none-once", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        assert!(!active.child_task_id.is_empty());
        assert_eq!(sink.receipt_reads.load(Ordering::SeqCst), 1);
        for _ in 0..3 {
            assert_eq!(handle.tick((2.0, 2.0)).await.child_task_id, active.child_task_id);
        }
        assert_eq!(sink.receipt_reads.load(Ordering::SeqCst), 1);
        let lease = active_child_lease(&handle).await;
        handle.stop("receipt-none-once").await;
        handle.report_navigation_result(&lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&lease, 1))).await.unwrap();
    }

    async fn child_paused_in_receipt_read(
        parent: &str,
    ) -> (ExploreRuntimeHandle, Arc<RecordingNavSink>, tokio::task::JoinHandle<ExploreRuntimeSnapshot>, Arc<tokio::sync::Notify>) {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start(parent, "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.receipt_release.lock().unwrap() = Some(release.clone());
        let tick = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.tick((2.0, 2.0)).await })
        };
        tokio::time::timeout(Duration::from_secs(2), sink.receipt_entered.notified()).await
            .expect("child attached before receipt read");
        assert!(!handle.snapshot().child_task_id.is_empty());
        assert!(handle.inner.lock().await.pending_child_dispatch.is_none());
        (handle, sink, tick, release)
    }

    #[tokio::test]
    async fn cancelled_child_receipt_waiter_resumes_on_next_tick() {
        let parent = "receipt-resume";
        let (handle, sink, tick, _) = child_paused_in_receipt_read(parent).await;
        let lease = active_child_lease(&handle).await;
        *sink.terminal_result.lock().unwrap() = Some(ChildNavigationResult::Failed {
            proof: terminal_proof(&lease), reason: "retained failure".into(),
        });
        tick.abort();
        assert!(tick.await.unwrap_err().is_cancelled());
        assert!(!handle.inner.lock().await.active_child.as_ref().unwrap().terminal_receipt_checked);
        *sink.receipt_release.lock().unwrap() = None;
        let resumed = handle.tick((2.0, 2.0)).await;
        assert!(resumed.child_task_id.is_empty());
        assert_eq!(sink.receipt_reads.load(Ordering::SeqCst), 2);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1, "resume must not resubmit");
        assert!(handle.frontier_snapshot().frontiers.is_empty());
        assert_one_receipt_source_failure(&receipt_test_source_history(&handle).await);
        handle.stop(parent).await;
    }

    #[tokio::test]
    async fn live_and_recovered_child_terminal_share_one_feedback_admission() {
        for reached in [false, true] {
            let parent = if reached { "receipt-live-reached" } else { "receipt-live-failed" };
            let (handle, sink, tick, release) = child_paused_in_receipt_read(parent).await;
            let lease = active_child_lease(&handle).await;
            let result = if reached { ChildNavigationResult::Reached(terminal_proof(&lease)) }
                else { ChildNavigationResult::Failed { proof: terminal_proof(&lease), reason: "one failure".into() } };
            let history = handle.inner.lock().await.compute.history;
            *sink.terminal_result.lock().unwrap() = Some(result.clone());
            handle.report_navigation_result(&lease.session_id, result).await.unwrap();
            release.notify_one();
            let completed = tokio::time::timeout(Duration::from_secs(2), tick).await.unwrap().unwrap();
            assert!(completed.child_task_id.is_empty());
            let inner = handle.inner.lock().await;
            assert_eq!(inner.reached_goals, if reached { 1 } else { 0 });
            assert_eq!(inner.compute.history, history + 1);
            assert!(inner.active_child.is_none());
            drop(inner);
            if !reached {
                assert_one_receipt_source_failure(&receipt_test_source_history(&handle).await);
            }
            handle.stop(parent).await;
        }
    }

    #[tokio::test]
    async fn stop_wins_while_child_receipt_waits_without_reopening_feedback() {
        let parent = "receipt-stop-wins";
        let (handle, sink, tick, release) = child_paused_in_receipt_read(parent).await;
        let lease = active_child_lease(&handle).await;
        let result = ChildNavigationResult::Reached(terminal_proof(&lease));
        *sink.terminal_result.lock().unwrap() = Some(result.clone());
        *sink.cancel_outcome.lock().unwrap() = Some(ChildNavigationCancelOutcome::AlreadyTerminal(terminal_proof(&lease)));
        let history = handle.inner.lock().await.compute.history;
        let stopped = handle.stop(parent).await;
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(stopped.child_task_id.is_empty());
        release.notify_one();
        let settled = tokio::time::timeout(Duration::from_secs(2), tick).await.unwrap().unwrap();
        assert_eq!(settled.status, STATUS_CANCELLED);
        let inner = handle.inner.lock().await;
        assert!(!inner.logical_active);
        assert_eq!(inner.reached_goals, 0, "Stop closed ordinary source admission");
        assert_eq!(inner.compute.history, history, "Stop must not admit ordinary feedback");
    }

    #[tokio::test]
    async fn resumed_child_receipt_preserves_existing_duration_stop_priority() {
        let parent = "receipt-duration-priority";
        let (handle, sink, tick, _) = child_paused_in_receipt_read(parent).await;
        let lease = active_child_lease(&handle).await;
        let result = ChildNavigationResult::Reached(terminal_proof(&lease));
        *sink.terminal_result.lock().unwrap() = Some(result.clone());
        *sink.cancel_outcome.lock().unwrap() = Some(ChildNavigationCancelOutcome::AlreadyTerminal(terminal_proof(&lease)));
        tick.abort();
        assert!(tick.await.unwrap_err().is_cancelled());
        *sink.receipt_release.lock().unwrap() = None;
        {
            let mut inner = handle.inner.lock().await;
            Arc::make_mut(inner.active_config.as_mut().unwrap()).stop_conditions.max_duration_ms = Some(1);
            inner.started_at_ns = now_unix_nanos().saturating_sub(2_000_000);
        }
        let stopped = handle.tick((2.0, 2.0)).await;
        assert_eq!(stopped.last_stop_reason, "duration_exceeded");
        assert!(stopped.child_task_id.is_empty());
        assert_eq!(sink.receipt_reads.load(Ordering::SeqCst), 1,
            "existing stop priority precedes a resumed ordinary receipt read");
        assert_eq!(handle.inner.lock().await.reached_goals, 0);
    }

    #[tokio::test]
    async fn already_terminal_cannot_replace_a_locally_retained_cancel_ticket() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("receipt-local-ticket", "prefer_wavefront").await;
        handle.tick((2.0, 2.0)).await;
        let lease = active_child_lease(&handle).await;
        handle.stop("receipt-local-ticket").await;
        let before = handle.inner.lock().await.pending_child_cancellation.as_ref().unwrap().ticket.clone();
        assert!(before.is_some());
        let result = handle.finish_already_terminal_child(&lease, terminal_proof(&lease)).await;
        assert_eq!(result.status, STATUS_FAILED);
        assert_eq!(result.child_task_id, lease.session_id);
        assert_eq!(handle.inner.lock().await.pending_child_cancellation.as_ref().unwrap().ticket, before);
        assert!(handle.blocks_mapping_mutation().await);
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
        let ticket = before.expect("original retained cancel ticket");
        let settled = handle.report_navigation_result(&lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&lease, ticket.cancel_generation))).await.unwrap();
        assert_eq!(settled.status, STATUS_FAILED, "prior failure remains visible after exact ZERO");
        assert!(settled.child_task_id.is_empty());
        assert!(!handle.blocks_mapping_mutation().await);
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
            commit_boot_nonce: [1; 16],
            commit_sequence: revision as i64,
            source_timestamp_ns: revision as i64,
            frame_id: frame_id.to_string(),
        }
    }

    #[tokio::test]
    async fn planar_costmap_preserves_receipt_time_and_rejects_provenance_conflicts() {
        let (handle, _) = configured_handle();
        let timestamp = now_unix_nanos() as i64;
        let value = nuwax_codec::lcm::planar_navigation::PlanarNavigationMap {
            map_frame_generation: 1, runtime_generation: 1,
            session_id: "planar-session".into(), session_generation: 1,
            map_revision: 2, map_boot_nonce: [3; 16], map_commit_sequence: 4,
            map_source_timestamp_ns: 7, observation_revision: 1,
            source_sequence: 1, source_timestamp_ns: timestamp,
            grid: schema::OccupancyGrid {
                header: schema::Header {
                    stamp_sec: (timestamp / 1_000_000_000) as i32,
                    stamp_nsec: (timestamp % 1_000_000_000) as i32,
                    frame_id: "map".into(), ..Default::default()
                },
                info: schema::MapMetaData {
                    width: 2, height: 2, resolution: 0.5,
                    origin: schema::Pose { position: [0.0; 3], orientation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                    ..Default::default()
                },
                data: vec![0; 4],
            },
            ..Default::default()
        };
        assert!(handle.ingest_planar_navigation(&value).await.unwrap());
        let stored = handle.inner.lock().await.committed_map.clone().unwrap();
        assert_eq!(stored.commit_boot_nonce, [3; 16]);
        assert_eq!(stored.commit_sequence, 4);
        assert_eq!(stored.source_timestamp_ns, 7);
        assert_ne!(stored.source_timestamp_ns, value.source_timestamp_ns);
        for field in 0..3 {
            let mut conflict = value.clone();
            match field {
                0 => { conflict.map_boot_nonce[0] ^= 1; conflict.map_revision += 1; },
                1 => conflict.map_commit_sequence += 1,
                _ => conflict.map_source_timestamp_ns += 1,
            }
            assert!(handle.ingest_planar_navigation(&conflict).await.is_err());
            assert_eq!(handle.inner.lock().await.committed_map.as_ref(), Some(&stored));
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
                commit_boot_nonce: [7; 16],
                commit_sequence: i64::from(revision),
                source_timestamp_ns: timestamp,
                frame_id: frame.into(),
            },
            map_frame_generation: 1,
            cloud_revision: i64::from(revision),
            cloud_occupancy_revision: i64::from(revision),
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
            assert_eq!(requests[0].intent.goal.pose.position,
                [goal.pose.position.x, goal.pose.position.y, goal.pose.position.z]);
            assert_eq!(requests[0].intent.goal.pose.orientation_xyzw,
                [orientation.x, orientation.y, orientation.z, orientation.w]);
            assert_eq!(requests[0].intent.map_frame_generation, 1);
            assert_eq!(requests[0].intent.body_to_support_z_m, Some(surface_config("map").follower.body_to_support_z_m));
            assert_eq!(requests[0].intent.options, nuwax_nav_types_messages::GoalOptions {
                allow_reverse: surface_config("map").follower.kinematics()
                    .is_some_and(|motion| motion.max_reverse_mps > 0.0),
                yaw_tolerance_rad: std::f64::consts::PI,
                ..Default::default()
            });
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
        clear.committed_map.commit_boot_nonce = [0; 16];
        clear.committed_map.commit_sequence = 0;
        clear.committed_map.source_timestamp_ns = 0;
        clear.cloud_occupancy_revision = 0;
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
        assert_eq!(status_for_stop_reason(ExplorationStopReason::PlanningResourceLimited), STATUS_FAILED);
        assert_eq!(stop_reason_str(ExplorationStopReason::PlanningResourceLimited), "planning_resource_limited");
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

        handle.inner.lock().await.costmap_input = Some(Arc::new(literal_map));
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
        {
            let submitted = sink.submitted.lock().unwrap();
            assert_eq!(submitted.len(), 1);
            let request = &submitted[0];
            let expected = build_wavefront_goal_intent(
                active.exploration_generation, 0, "req-flow", "req-flow.nav.1",
                &crate::nav_runtime::test_motion_admission(), &request.goal, None,
                committed_map_identity(1, "map"),
            ).unwrap();
            assert_eq!(request.intent, expected);
            assert!(request.intent.boundary_points.is_empty());
            assert!(request.intent.boundary_id.is_empty());
            assert!(request.intent.boundary_points.is_empty());
            assert_eq!(request.goal.id, request.request_id);
            assert_eq!(request.expected_map_name, "map");
            assert_eq!(request.motion_admission, crate::nav_runtime::test_motion_admission());
        }
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
            assert!(submitted
                .iter()
                .all(|request| request.intent == submitted[0].intent));
            assert_eq!(submitted[0].intent.committed_map, committed_map_identity(1, "map"));
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
    async fn source_revision_change_reselects_without_blacklisting_the_frontier() {
        let (handle, sink) = configured_handle();
        sink.submit_errors.lock().unwrap().push_back(ChildNavigationSubmitError::SourceChanged("普通修订前进".into()));
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("source-refresh", "prefer_wavefront").await;
        let pending = handle.tick((2.0, 2.0)).await;
        assert_eq!(pending.status, STATUS_PENDING);
        assert!(pending.message.contains("refreshed exploration input"));
        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2).await.unwrap();
        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let submitted = sink.submitted.lock().unwrap();
        assert_eq!(submitted.len(), 2);
        assert_eq!(submitted[0].goal.target.pose.position, submitted[1].goal.target.pose.position);
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

            },
            ChildNavigationLease {
                session_id: "invalid-generation".to_string(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                parent_request_id: "req-invalid-lease".to_string(),
                goal_generation: 1,
                producer_runtime_generation: 0,

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
            receipt.child_lease,
            Some(exact.child_lease.clone())
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
                &receipt.child_lease_fingerprint, receipt.child_lease.as_ref(),
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
            request_identity: None,
            kind: DurableOwnershipTransitionKind::GlobalStop,
            recovery_correlation: correlation.clone(),
            request_fingerprint: format!("{:064x}", index + 1),
            receipt: ExplorationOwnershipReconciliationReceipt {
            child_lease: None,
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
            let bytes = encode_ownership_journal(&DurableOwnershipJournal {
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
            &first.receipt.child_lease_fingerprint, first.receipt.child_lease.as_ref(),
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
        assert_eq!(submitted[0].intent.boundary_id, "req-boundary:boundary");
        assert_eq!(submitted[0].intent.goal.header.frame_id, "map");
        assert_eq!(submitted[0].intent.committed_map.frame_id, "map");
        assert_eq!(submitted[0].intent.boundary_points, vec![
            [0.0, 0.0, 0.0], [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0], [0.0, 5.0, 0.0],
        ]);
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
    async fn unobserved_origin_wait_recovers_and_preserves_cancel_and_duration_limit() {
        for scenario in ["recover", "cancel", "duration"] {
            let (handle, sink) = configured_handle();
            let mut incomplete = frontier_costmap(1);
            incomplete.grid.set(4, 4, OccupancyCell::Unknown);
            ingest_committed_costmap_fixture(&handle, incomplete, 1)
                .await
                .unwrap();
            handle
                .start_with_config(
                    "req-initial-observation",
                    ExploreStartConfig {
                        motion_admission: crate::nav_runtime::test_motion_admission(),
                        stop_conditions: ExploreStopConditions {
                            max_duration_ms: Some(1_000),
                            ..ExploreStopConditions::default()
                        },
                        ..ExploreStartConfig::default()
                    },
                )
                .await;
            let waiting = handle.tick((2.0, 2.0)).await;
            assert_eq!(waiting.status, STATUS_PENDING);
            assert!(waiting.message.contains("not fully observed"));
            assert!(sink.submitted.lock().unwrap().is_empty());

            match scenario {
                "recover" => {
                    ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
                        .await
                        .unwrap();
                    let active = handle.tick((2.0, 2.0)).await;
                    assert_eq!(active.status, STATUS_ACTIVE);
                    assert!(!active.child_task_id.is_empty());
                    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
                    let mut guard = handle.inner.lock().await;
                    guard.started_at_ns = now_unix_nanos() - 2_000_000_000;
                    guard.last_poll = Some(ExplorationPoll::SourceUnavailable(
                        "later input wait".into(),
                    ));
                    assert_eq!(
                        evaluate_stop_conditions(&guard, now_unix_nanos(), None),
                        Some(ExplorationStopReason::DurationExceeded),
                        "已选择目标后的原限时规则保持"
                    );
                }
                "cancel" => {
                    let stopped = handle.stop("req-initial-observation").await;
                    assert_eq!(stopped.status, STATUS_CANCELLED);
                    assert!(sink.submitted.lock().unwrap().is_empty());
                }
                _ => {
                    handle.inner.lock().await.started_at_ns = now_unix_nanos() - 2_000_000_000;
                    let stopped = handle.tick((2.0, 2.0)).await;
                    assert_eq!(stopped.last_stop_reason, "no_reachable_free_space");
                    assert_eq!(stopped.status, STATUS_FAILED);
                    assert!(stopped.message.contains("not fully observed"));
                    assert!(sink.submitted.lock().unwrap().is_empty());
                }
            }
        }
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
    async fn stop_during_submit_cancels_late_ticket_and_retains_orphan() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("req-late-ticket", "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.submit_release.lock().unwrap() = Some(release.clone());
        let ticking = {
            let handle = handle.clone();
            tokio::spawn(async move { handle.tick((2.0, 2.0)).await })
        };
        tokio::time::timeout(Duration::from_secs(1), sink.submit_entered.notified())
            .await.expect("submit must wait outside the exploration lock");
        let frozen = handle.inner.lock().await.pending_child_dispatch.clone().unwrap();
        assert_eq!(frozen.request_id, "req-late-ticket.nav.1");
        let stopped = tokio::time::timeout(Duration::from_secs(1), handle.stop("req-late-ticket"))
            .await.expect("stop must not wait for child submission");
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(handle.inner.lock().await.pending_child_dispatch.is_none());
        release.notify_one();
        let settled = tokio::time::timeout(Duration::from_secs(1), ticking)
            .await.expect("late child ticket must settle").unwrap();
        assert_eq!(settled.status, STATUS_CANCELLED);
        assert!(settled.child_task_id.is_empty());
        assert!(handle.inner.lock().await.active_child.is_none());
        assert_eq!(sink.submitted.lock().unwrap()[0].intent, frozen.intent);
        assert_eq!(*sink.cancelled.lock().unwrap(), vec![frozen.request_id]);
        assert!(handle.orphan_cancellations.has_entries());
        assert!(handle.blocks_mapping_mutation().await);
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
    async fn stale_pose_preserves_active_child_until_execution_terminal_or_user_stop() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("req-stale-active", "prefer_wavefront").await;
        let active = handle.tick((2.0, 2.0)).await;
        let scheduler = ExploreSchedulerHandle::spawn(handle.clone(), Arc::new(MissingPoseProvider), Duration::from_millis(5));
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(handle.is_active().await);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert!(sink.cancelled.lock().unwrap().is_empty());
        handle.stop("req-stale-active").await;
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
        assert_eq!(handle.snapshot().status, STATUS_STOPPING);
        scheduler.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn stale_pose_without_child_waits_for_request_deadline_not_two_second_grace() {
        let (handle, _) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("req-stale-idle", "prefer_wavefront").await;
        let scheduler = ExploreSchedulerHandle::spawn(handle.clone(), Arc::new(MissingPoseProvider), Duration::from_millis(5));
        tokio::time::sleep(Duration::from_millis(2100)).await;
        assert_eq!(handle.snapshot().status, STATUS_PENDING);
        assert!(handle.is_active().await);
        handle.stop("req-stale-idle").await;
        assert!(!handle.is_active().await);
        scheduler.shutdown().await.unwrap();
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

    include!("explore_compute_runtime_tests.rs");
    include!("explore_compute_surface_tests.rs");
    include!("explore_composite_map_tests.rs");
    include!("explore_surface_incomplete_tests.rs");

    #[tokio::test]
    async fn compute_owner_surface_slow_prepare_keeps_result_but_rejects_stale_next_tick() {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request_id = "owner-surface-slow-prepare";
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
        let identity = {
            let guard = handle.inner.lock().await;
            (
                guard.generation,
                guard.session_generation,
                guard.last_request_id.clone(),
                guard.compute.history,
                guard.committed_map.clone(),
                guard.compute.status().unwrap().incarnation,
            )
        };
        let tick = tokio::spawn({
            let handle = handle.clone();
            let pose = pose.clone();
            async move { handle.tick_with_pose(pose).await }
        });
        owner_watchdog(gate.entered()).await;
        // Hold the actual Surface source after preparation. The native worker
        // has not published a proposal, so Accept/commit still run normally.
        tokio::time::sleep(Duration::from_millis(1100)).await;
        assert!(pose.received_monotonic_ns != 0);
        assert!(
            now_monotonic_nanos().saturating_sub(pose.received_monotonic_ns)
                > DEFAULT_POSE_RECEIPT_TTL.as_nanos() as u64
        );
        // Only a real same-source map receipt is refreshed. Neither the sampled
        // pose nor its timestamp/receipt is rewritten to make completion pass.
        publish_surface_owner_receipt(&handle, &message.committed_map, 2, snapshot.clone()).await;
        assert!(original.same_source_view(&receiver.freeze().unwrap()));
        {
            let guard = handle.inner.lock().await;
            assert_eq!(
                (
                    guard.generation,
                    guard.session_generation,
                    guard.last_request_id.clone(),
                    guard.compute.history,
                    guard.committed_map.clone(),
                    guard.compute.status().unwrap().incarnation,
                ),
                identity
            );
        }
        assert!(!tick.is_finished());
        assert!(sink.submitted.lock().unwrap().is_empty());
        gate.release();
        let active = owner_watchdog(tick).await.unwrap();
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        assert_eq!(active.child_task_id, "owner-surface-slow-prepare.nav.1");
        {
            let audit = audit.lock().unwrap();
            assert_eq!(
                (audit.captures.len(), audit.commits.len(), audit.discards),
                (1, 1, 0)
            );
            assert_surface_owner_capture(
                &audit.captures[0], &snapshot, original.notification(), &pose,
            );
            let submitted = sink.submitted.lock().unwrap();
            assert_eq!(submitted.len(), 1);
            assert_eq!(
                submitted[0].frontier_goal_id,
                surface_owner_goal(&audit.commits[0]).id
            );
        }
        // This recording sink proves exploration handoff, not live navigation
        // admission or motor safety. Those independent consumers are unchanged.
        let lease = active_child_lease(&handle).await;
        owner_watchdog(handle.report_navigation_result(
            &active.child_task_id,
            ChildNavigationResult::Reached(terminal_proof(&lease)),
        ))
        .await
        .unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        assert!(handle.inner.lock().await.active_child.is_none());
        // The identical now-old receipt remains invalid at the next tick's
        // entrance. Removing completion age must not admit a stale new request.
        let stale = owner_watchdog(handle.tick_with_pose(pose)).await;
        assert_eq!(stale.status, STATUS_FAILED, "{}", stale.message);
        assert!(stale.message.contains("pose source stale"));
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        {
            let audit = audit.lock().unwrap();
            assert_eq!(
                (audit.captures.len(), audit.commits.len(), audit.discards),
                (1, 1, 0)
            );
        }
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        handle.request_compute_shutdown().await;
        owner_watchdog(handle.compute_service.join()).await.unwrap();
    }

    #[tokio::test]
    async fn compute_owner_zero_pose_source_or_receipt_is_rejected_before_prepare() {
        for zero_source in [false, true] {
            let (handle, sink) = configured_handle();
            let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
            assert_eq!(
                handle.start("owner-zero-pose", "prefer_wavefront").await.status,
                STATUS_ACCEPTED
            );
            let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
            let gate = Arc::new(OwnerRuntimeGate::default());
            let _release = OwnerRuntimeGateRelease(gate.clone());
            install_gated_real_surface_source(&handle, audit.clone(), gate, None).await;
            let mut pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
            if zero_source {
                pose.source_timestamp_ns = 0;
            } else {
                pose.received_monotonic_ns = 0;
            }
            let rejected = owner_watchdog(handle.tick_with_pose(pose)).await;
            assert_eq!(rejected.status, STATUS_FAILED, "{}", rejected.message);
            assert!(rejected.message.contains("pose source stale"));
            owner_watchdog(handle.await_compute_idle()).await.unwrap();
            {
                let audit = audit.lock().unwrap();
                assert!(audit.captures.is_empty());
                assert!(audit.commits.is_empty());
                assert_eq!(audit.discards, 0);
            }
            assert!(sink.submitted.lock().unwrap().is_empty());
            handle.request_compute_shutdown().await;
            owner_watchdog(handle.compute_service.join()).await.unwrap();
        }
    }

    #[tokio::test]
    async fn compute_owner_feedback_still_invalidates_an_accepted_selection() {
        let (handle, sink) = configured_handle();
        let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
        assert_eq!(
            handle.start("owner-feedback-basis", "prefer_wavefront").await.status,
            STATUS_ACCEPTED
        );
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let gate = Arc::new(OwnerRuntimeGate::default());
        gate.release();
        install_gated_real_surface_source(&handle, audit.clone(), gate, None).await;
        let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
        let mut guard = handle.inner.lock().await;
        let context = handle.capture_compute_context(&guard, &pose, now_unix_nanos()).unwrap();
        let ticket = handle.admit_compute_locked(
            &mut guard,
            &context,
            ComputeRequest::Selection(SelectionRequest { context: context.clone(), progress: None }),
        ).unwrap();
        drop(guard);
        // Exercise the real native prepare/Accept/commit protocol, not a
        // constructed ComputeBasis or a direct mutation of its history.
        let finalized = owner_watchdog(handle.await_compute(&ticket)).await.unwrap();
        let goal_id = match &finalized.projection {
            ComputeProjection::Selection(selection) => {
                surface_owner_goal(selection.poll.as_ref().unwrap()).id.clone()
            }
            _ => panic!("expected real Surface selection"),
        };
        guard = handle.inner.lock().await;
        assert!(ticket.current(&guard));
        let history_before = guard.compute.history;
        let generation = guard.generation;
        let feedback = ExplorationGoalFeedback::Failed("new source feedback".into());
        let feedback_ticket = handle.admit_feedback_locked(
            &mut guard,
            FeedbackIdentity::Internal {
                generation,
                lineage: format!("poll:{}:{}", ticket.id.incarnation, ticket.id.sequence),
                goal: goal_id.clone(),
                feedback: feedback.clone(),
            },
            goal_id.clone(), feedback.clone(), None, false,
        ).unwrap();
        ticket.feedback_handoff(&mut guard);
        assert!(!ticket.current(&guard), "pending feedback must fence the old result");
        assert!(!ticket.current_after_feedback(&guard, &feedback_ticket));
        drop(guard);
        owner_watchdog(handle.await_feedback(&feedback_ticket)).await.unwrap();
        guard = handle.inner.lock().await;
        assert_eq!(guard.compute.history, history_before + 1);
        assert!(!ticket.current(&guard), "acknowledged feedback must supersede old history");
        assert!(!ticket.current_after_feedback(&guard, &feedback_ticket));
        ticket.finish(&mut guard);
        drop(guard);
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        {
            let audit = audit.lock().unwrap();
            assert_eq!((audit.captures.len(), audit.commits.len(), audit.discards), (1, 1, 0));
            assert_eq!(audit.feedback, vec![(goal_id, feedback)]);
        }
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert_eq!(
            owner_watchdog(handle.stop("owner-feedback-basis")).await.status,
            STATUS_CANCELLED
        );
        handle.request_compute_shutdown().await;
        owner_watchdog(handle.compute_service.join()).await.unwrap();
    }

    // These fixtures never dispatch a goal or compute a legacy identity.
    async fn hashfree_checkpoint_fixture() -> (String, ExploreStartConfig) {
        let (handle, sink) = configured_handle();
        let started = handle.start("settings-origin", "prefer_wavefront").await;
        assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
        let config;
        let restore_sources;
        {
            let mut guard = handle.inner.lock().await;
            config = guard.active_config.as_deref().cloned().unwrap();
            guard.committed_map = Some(schema::CommittedMapIdentity {
                session_id: "legacy-map-alias".into(), session_generation: 1, revision: 1,
                commit_boot_nonce: [0x37; 16], commit_sequence: 1, source_timestamp_ns: 1, frame_id: "map".into(),
            });
            guard.costmap_frame_generation = 1;
            guard.progress_origin = Some(("legacy-map-alias".into(), 1, 1));
            guard.progress_map_name = "map".into();
            guard.reached_goals = 1;
            guard.started_at_ns = now_unix_nanos().saturating_sub(500_000_000);
            let mut session = nuwax_explore_wavefront::session::ExplorationSession::new();
            session.record_goal_reached(Point2::new(12.5, -3.25));
            session.record_goal_failed(Point2::new(7.0, 9.0), 0, 1.5);
            restore_sources = vec![nuwax_exploration::mission::SourceCheckpoint {
                name: "wavefront".into(), payload: serde_json::to_string(&(1_u32, session)).unwrap(),
            }];
        }
        let mut controller = build_profile_controller(ExplorationArbitrationPolicy::PreferWavefront, &config, None).unwrap();
        controller.begin_exploration().unwrap();
        controller.restore_checkpoint(&restore_sources).unwrap();
        handle.install_compute_controller(controller).await.unwrap();
        handle.stop("settings-origin").await;
        let payload = handle.checkpoint_progress("settings-origin").await.unwrap();
        assert!(sink.submitted.lock().unwrap().is_empty());
        if let Ok(path) = std::env::var("NUWAX_TEST_EXPLORATION_CHECKPOINT_OUTPUT") {
            std::fs::write(path, &payload).unwrap();
        }
        (payload, config)
    }

    #[tokio::test]
    async fn checkpoint_v2_resumes_equal_settings_and_reports_changed_field_without_dispatch() {
        let (payload, mut config) = hashfree_checkpoint_fixture().await;
        let saved = ExplorationCheckpoint::parse(&payload).unwrap();
        assert_eq!(saved.schema_version, 2);
        assert!(!payload.contains("sha256"));
        config.resume_progress_json = payload.clone();
        let (resumed, sink) = configured_handle();
        let accepted = resumed.start_with_config("equal-settings", config.clone()).await;
        assert_eq!(accepted.status, STATUS_ACCEPTED, "{}", accepted.message);
        assert!(!accepted.message.contains("legacy history"));
        let sources = resumed.inspect_compute_checkpoint().await.unwrap();
        let guard = resumed.inner.lock().await;
        assert_eq!(guard.reached_goals, 1);
        assert_eq!(sources[0].payload, saved.sources[0].payload);
        assert!(guard.active_child.is_none());
        assert!(sink.submitted.lock().unwrap().is_empty());
        drop(guard);
        config.max_navigation_retries += 1;
        let (changed, sink) = configured_handle();
        let rejected = changed.start_with_config("changed-settings", config).await;
        assert_eq!(rejected.status, STATUS_REJECTED);
        assert!(rejected.message.contains("configuration.max_navigation_retries"));
        assert!(!changed.is_active().await);
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn checkpoint_legacy_changed_settings_preserve_world_history_and_replan_without_dispatch() {
        let (payload, mut config) = hashfree_checkpoint_fixture().await;
        let mut saved = ExplorationCheckpoint::parse(&payload).unwrap();
        saved.schema_version = 1;
        saved.configuration = None;
        saved.navigation_configuration = None;
        saved.configuration_sha256 = Some([0x22; 32]);
        saved.navigation_configuration_sha256 = Some([0x99; 32]);
        let legacy_payload = serde_json::to_string(&saved).unwrap();
        config.resume_progress_json = legacy_payload.clone();
        config.max_navigation_retries = 9;
        config.blacklist_radius_m = 0.25;
        let (resumed, sink) = configured_handle();
        let accepted = resumed.start_with_config("legacy-current-settings", config).await;
        assert_eq!(accepted.status, STATUS_ACCEPTED, "{}", accepted.message);
        assert!(accepted.message.contains("legacy history resumed under current settings"));
        let source = resumed.inspect_compute_checkpoint().await.unwrap();
        let guard = resumed.inner.lock().await;
        assert_eq!(guard.reached_goals, 1);
        assert!(now_unix_nanos().saturating_sub(guard.started_at_ns) >= saved.elapsed_ns);
        let history: serde_json::Value = serde_json::from_str(&source[0].payload).unwrap();
        assert_eq!(history[1]["explored_goals"], serde_json::json!([{ "x": 12.5, "y": -3.25 }]));
        assert_eq!(history[1]["blacklisted_goals"], serde_json::json!([]));
        assert_eq!(history[1]["failed_goals"], serde_json::json!([]));
        assert_eq!(history[1]["consecutive_failures"], serde_json::json!(0));
        assert!(guard.active_child.is_none());
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert_eq!(serde_json::to_string(&ExplorationCheckpoint::parse(&legacy_payload).unwrap()).unwrap(), legacy_payload);
    }

    #[tokio::test]
    async fn checkpoint_save_uses_start_snapshot_without_environment_recapture() {
        let (handle, _) = configured_handle();
        handle.start("frozen-settings", "prefer_wavefront").await;
        let frozen;
        {
            let mut guard = handle.inner.lock().await;
            // A supplied frozen value intentionally differs from this process's environment.
            let mut navigation = guard.navigation_configuration.as_ref().unwrap().as_ref().unwrap().clone();
            navigation.base_frame = "frozen-start-body-frame".into();
            frozen = navigation.clone();
            guard.navigation_configuration = Some(Ok(navigation));
            guard.committed_map = Some(schema::CommittedMapIdentity {
                session_id: "legacy-map-alias".into(), session_generation: 1, revision: 1,
                commit_boot_nonce: [0x37; 16], commit_sequence: 1, source_timestamp_ns: 1, frame_id: "map".into(),
            });
            guard.costmap_frame_generation = 1;
            guard.progress_origin = Some(("legacy-map-alias".into(), 1, 1));
            guard.progress_map_name = "map".into();
        }
        handle.stop("frozen-settings").await;
        let saved = ExplorationCheckpoint::parse(&handle.checkpoint_progress("frozen-settings").await.unwrap()).unwrap();
        assert_eq!(saved.navigation_configuration, Some(frozen));
    }

    #[tokio::test]
    async fn checkpoint_restores_confirmed_source_in_new_generation_and_rejects_wrong_binding() {
        let (handle,_)=configured_handle();
        ingest_committed_costmap_fixture(&handle,frontier_costmap(1),1).await.unwrap();
        handle.inner.lock().await.costmap_frame_generation=1;
        handle.start("checkpoint-origin","prefer_wavefront").await;
        let active=handle.tick((2.0,2.0)).await;
        assert!(handle.checkpoint_progress("checkpoint-origin").await.is_err());
        let old_lease=active_child_lease(&handle).await;
        handle.report_navigation_result(&active.child_task_id,ChildNavigationResult::Reached(terminal_proof(&old_lease))).await.unwrap();
        handle.stop("checkpoint-origin").await;
        let checkpoint=handle.checkpoint_progress("checkpoint-origin").await.unwrap();
        let saved=ExplorationCheckpoint::parse(&checkpoint).unwrap();
        assert_eq!(saved.reached_goals,1);
        assert!(handle.checkpoint_progress("another-parent").await.is_err());
        let (resumed,sink)=configured_handle();
        ingest_committed_costmap_fixture(&resumed,frontier_costmap(2),2).await.unwrap();
        resumed.inner.lock().await.costmap_frame_generation=2;
        let mut config=handle.inner.lock().await.active_config.as_deref().cloned().unwrap();
        config.resume_progress_json=checkpoint.clone();
        let accepted=resumed.start_with_config("checkpoint-new-parent",config.clone()).await;
        assert_eq!(accepted.status,STATUS_ACCEPTED,"{}",accepted.message);
        assert_eq!(resumed.inspect_compute_checkpoint().await.unwrap()[0].payload,saved.sources[0].payload);
        assert!(resumed.report_navigation_result(&active.child_task_id,ChildNavigationResult::Reached(terminal_proof(&old_lease))).await.is_err());
        let next=resumed.tick((2.0,2.0)).await;
        assert_eq!(next.status,STATUS_ACTIVE,"{}",next.message);
        assert_ne!(next.child_task_id,active.child_task_id);
        assert!(resumed.report_navigation_result(&next.child_task_id,ChildNavigationResult::Reached(terminal_proof(&old_lease))).await.is_err());
        assert_eq!(sink.submitted.lock().unwrap().len(),1);
        for variation in ["map","config","source"] {
            let (other,sink)=configured_handle();
            let mut candidate=config.clone();
            if variation=="config"{candidate.max_navigation_retries+=1;}else{
                let mut value:serde_json::Value=serde_json::from_str(&checkpoint).unwrap();
                if variation=="map"{value["map_name"]=serde_json::json!("other-map");}else{value["sources"][0]["payload"]=serde_json::json!("{}");}
                candidate.resume_progress_json=value.to_string();
            }
            let rejected=other.start_with_config("bad-restore",candidate).await;
            assert_eq!(rejected.status,STATUS_REJECTED,"{}",rejected.message);
            assert!(!other.is_active().await);assert!(sink.submitted.lock().unwrap().is_empty());
        }
    }

    #[tokio::test]
    async fn mission_return_two_pre_admission_rejections_exhaust_original_retry_budget() {
        const PARENT: &str = "mission-return-rejected";
        #[derive(Debug)]
        struct AttemptObservation {
            snapshot: ExploreRuntimeSnapshot,
            return_failures: u32,
            logical_active: bool,
            returning: bool,
            return_goal_id: Option<String>,
            frame_generation: i64,
            has_active_child: bool,
            has_pending_dispatch: bool,
            reached_goals: u32,
            submitted_count: usize,
            lease_allocations: u64,
        }

        let (handle, sink) = configured_handle();
        let map = frontier_costmap(1);
        let identity = committed_map_identity(1, "map");
        let mut observed = Vec::with_capacity(2);
        let scenario = async {
            scheme_a_race_wait(
                "install original map receipt",
                handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
            ).await??;
            let started = scheme_a_race_wait("start original owner", handle
                .start_with_config(
                    PARENT,
                    ExploreStartConfig {
                        motion_admission: crate::nav_runtime::test_motion_admission(),
                        max_navigation_retries: 1,
                        ..ExploreStartConfig::default()
                    },
                )).await?;
            if started.status != STATUS_ACCEPTED {
                return Err(format!("fixture start was not accepted: {started:#?}"));
            }
            // Reuse the existing real-owner mission fixture. Its frame assignment
            // stays at the already installed frame 1; no frame change is injected.
            enable_test_mission(&handle, Some(0.01)).await;
            sink.submit_errors.lock().unwrap().extend((0..2).map(|_| {
                // RecordingNavSink returns this before allocating any lease.
                // The explicit variant is the new contract, not a Bridge inference.
                ChildNavigationSubmitError::RejectedBeforePublish(
                    "test permanent pre-admission rejection".to_string(),
                )
            }));

            for _ in 0..2 {
                // A new receipt for the same map/frame keeps the original input Arc
                // and owner identity; do not bypass the production freshness checks.
                scheme_a_race_wait(
                    "refresh original map receipt",
                    handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
                ).await??;
                let snapshot = scheme_a_race_wait("real rejection tick", handle.tick((2.0, 2.0))).await?;
                let submitted_count = sink.submitted.lock().unwrap().len();
                let lease_allocations = sink.next_goal_generation.load(Ordering::SeqCst);
                let guard = handle.inner.lock().await;
                observed.push(AttemptObservation {
                    snapshot,
                    return_failures: guard.return_failures,
                    logical_active: guard.logical_active,
                    returning: guard.mission.as_ref().is_some_and(|mission| mission.returning),
                    return_goal_id: guard.return_goal_id.clone(),
                    frame_generation: guard.costmap_frame_generation,
                    has_active_child: guard.active_child.is_some(),
                    has_pending_dispatch: guard.pending_child_dispatch.is_some(),
                    reached_goals: guard.reached_goals,
                    submitted_count,
                    lease_allocations,
                });
            }
            Ok::<(), String>(())
        }.await;
        let requests = sink.submitted.lock().unwrap().clone();
        let remaining_errors = sink.submit_errors.lock().unwrap().len();
        let cancel_count = sink.cancelled.lock().unwrap().len();
        let receipt_reads = sink.receipt_reads.load(Ordering::SeqCst);
        eprintln!("return rejection observations before cleanup: {observed:#?}; scenario={scenario:#?}");

        // Capture BOTH attempts before testing the budget. Clean the still-active
        // red path with ordinary Stop, without inventing a terminal/zero proof or
        // replacing either observed snapshot with the cleanup result.
        let cleanup = scheme_a_race_wait("ordinary cleanup Stop", async {
            if handle.is_active().await { Some(handle.stop(PARENT).await) } else { None }
        }).await;
        let settled = scheme_a_race_wait("ordinary compute-idle cleanup", handle.await_compute_idle()).await;
        // Idle retires the current job; it does not shut down the native owner.
        // Always attempt both existing lifecycle steps before asserting a failure.
        let shutdown = scheme_a_race_wait("request original compute shutdown", handle.request_compute_shutdown()).await;
        let joined = scheme_a_race_wait("join original compute owner", handle.compute_service.join()).await;
        assert!(shutdown.is_ok(), "compute shutdown request failed: {shutdown:?}; scenario={scenario:#?}");
        assert!(matches!(&joined, Ok(Ok(()))), "compute owner join failed: {joined:?}; scenario={scenario:#?}");
        assert!(matches!(&settled, Ok(Ok(()))), "compute cleanup failed: {settled:?}; scenario={scenario:#?}");
        if let Some(cleanup) = cleanup.expect("ordinary Stop cleanup timed out") {
            assert_eq!(cleanup.status, STATUS_CANCELLED, "{}", cleanup.message);
        }
        assert!(scenario.is_ok(), "fixture/body failed after cleanup: {scenario:#?}; observations={observed:#?}");
        assert!(sink.cancelled.lock().unwrap().is_empty());

        assert_eq!(requests.len(), 2, "both real-owner attempts must reach the sink");
        assert_eq!(remaining_errors, 0, "both pre-admission errors must be consumed");
        assert_eq!(cancel_count, 0);
        assert_eq!(receipt_reads, 0, "there was no accepted lease to reconcile");
        for (index, (attempt, request)) in observed.iter().zip(&requests).enumerate() {
            assert_eq!(attempt.submitted_count, index + 1, "{attempt:#?}");
            assert_eq!(attempt.lease_allocations, 0, "{attempt:#?}");
            assert_eq!(attempt.frame_generation, 1, "{attempt:#?}");
            if index == 0 {
                assert!(attempt.returning, "{attempt:#?}");
            }
            assert!(!attempt.has_active_child, "{attempt:#?}");
            assert!(!attempt.has_pending_dispatch, "{attempt:#?}");
            assert_eq!(attempt.reached_goals, 0, "{attempt:#?}");
            assert!(attempt.snapshot.child_task_id.is_empty(), "{attempt:#?}");
            assert!(attempt.snapshot.child_lease.is_none(), "{attempt:#?}");
            assert_eq!(request.parent_request_id, PARENT);
            assert_eq!(request.request_id, format!("{PARENT}.nav.{}", index + 1));
            assert_eq!(request.frontier_goal_id, format!("{PARENT}.return.{}", index + 1));
            if index == 0 {
                assert_eq!(attempt.return_goal_id.as_deref(), Some(request.frontier_goal_id.as_str()));
            }
            assert_eq!(request.intent.map_frame_generation, 1);
            assert_eq!(request.intent.committed_map, identity);
            assert_eq!(request.intent.goal.pose.position, [2.0, 2.0, 0.0]);
            assert_eq!(request.boundary_owner.max_navigation_retries, 1);
        }
        assert_eq!(
            [observed[0].return_failures, observed[1].return_failures],
            [1, 2],
            "one original retry permits two failures, not an unbounded return loop: {observed:#?}",
        );
        assert!(observed[0].logical_active, "{:#?}", observed[0]);
        assert!(matches!(observed[0].snapshot.status, STATUS_PENDING | STATUS_ACTIVE));
        assert!(!observed[1].logical_active, "{:#?}", observed[1]);
        assert_eq!(observed[1].snapshot.status, STATUS_FAILED, "{:#?}", observed[1]);
        assert_eq!(
            observed[1].snapshot.last_stop_reason,
            "consecutive_failures_exceeded",
            "{:#?}", observed[1],
        );
    }


    #[tokio::test]
    async fn mission_return_generic_rejections_do_not_consume_definite_rejection_budget() {
        const PARENT: &str = "mission-return-rejected";
        #[derive(Debug)]
        struct AttemptObservation {
            snapshot: ExploreRuntimeSnapshot,
            return_failures: u32,
            logical_active: bool,
            returning: bool,
            return_goal_id: Option<String>,
            frame_generation: i64,
            has_active_child: bool,
            has_pending_dispatch: bool,
            reached_goals: u32,
            submitted_count: usize,
            lease_allocations: u64,
        }

        let (handle, sink) = configured_handle();
        let map = frontier_costmap(1);
        let identity = committed_map_identity(1, "map");
        let mut observed = Vec::with_capacity(2);
        let scenario = async {
            scheme_a_race_wait(
                "install original map receipt",
                handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
            ).await??;
            let started = scheme_a_race_wait("start original owner", handle
                .start_with_config(
                    PARENT,
                    ExploreStartConfig {
                        motion_admission: crate::nav_runtime::test_motion_admission(),
                        max_navigation_retries: 1,
                        ..ExploreStartConfig::default()
                    },
                )).await?;
            if started.status != STATUS_ACCEPTED {
                return Err(format!("fixture start was not accepted: {started:#?}"));
            }
            // Reuse the existing real-owner mission fixture. Its frame assignment
            // stays at the already installed frame 1; no frame change is injected.
            enable_test_mission(&handle, Some(0.01)).await;
            sink.submit_errors.lock().unwrap().extend((0..2).map(|_| {
                // Even this no-lease mock cannot upgrade a generic error's contract.
                // Real generic results can mean already/possibly published work.
                ChildNavigationSubmitError::Rejected(
                    "test legacy generic rejection".to_string(),
                )
            }));

            for _ in 0..2 {
                // A new receipt for the same map/frame keeps the original input Arc
                // and owner identity; do not bypass the production freshness checks.
                scheme_a_race_wait(
                    "refresh original map receipt",
                    handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
                ).await??;
                let snapshot = scheme_a_race_wait("real rejection tick", handle.tick((2.0, 2.0))).await?;
                let submitted_count = sink.submitted.lock().unwrap().len();
                let lease_allocations = sink.next_goal_generation.load(Ordering::SeqCst);
                let guard = handle.inner.lock().await;
                observed.push(AttemptObservation {
                    snapshot,
                    return_failures: guard.return_failures,
                    logical_active: guard.logical_active,
                    returning: guard.mission.as_ref().is_some_and(|mission| mission.returning),
                    return_goal_id: guard.return_goal_id.clone(),
                    frame_generation: guard.costmap_frame_generation,
                    has_active_child: guard.active_child.is_some(),
                    has_pending_dispatch: guard.pending_child_dispatch.is_some(),
                    reached_goals: guard.reached_goals,
                    submitted_count,
                    lease_allocations,
                });
            }
            Ok::<(), String>(())
        }.await;
        let requests = sink.submitted.lock().unwrap().clone();
        let remaining_errors = sink.submit_errors.lock().unwrap().len();
        let cancel_count = sink.cancelled.lock().unwrap().len();
        let receipt_reads = sink.receipt_reads.load(Ordering::SeqCst);
        eprintln!("return rejection observations before cleanup: {observed:#?}; scenario={scenario:#?}");

        // Capture BOTH attempts before testing the budget. Clean the still-active
        // still-active path with ordinary Stop, without inventing a terminal/zero proof or
        // replacing either observed snapshot with the cleanup result.
        let cleanup = scheme_a_race_wait("ordinary cleanup Stop", async {
            if handle.is_active().await { Some(handle.stop(PARENT).await) } else { None }
        }).await;
        let settled = scheme_a_race_wait("ordinary compute-idle cleanup", handle.await_compute_idle()).await;
        // Idle retires the current job; it does not shut down the native owner.
        // Always attempt both existing lifecycle steps before asserting a failure.
        let shutdown = scheme_a_race_wait("request original compute shutdown", handle.request_compute_shutdown()).await;
        let joined = scheme_a_race_wait("join original compute owner", handle.compute_service.join()).await;
        assert!(shutdown.is_ok(), "compute shutdown request failed: {shutdown:?}; scenario={scenario:#?}");
        assert!(matches!(&joined, Ok(Ok(()))), "compute owner join failed: {joined:?}; scenario={scenario:#?}");
        assert!(matches!(&settled, Ok(Ok(()))), "compute cleanup failed: {settled:?}; scenario={scenario:#?}");
        if let Some(cleanup) = cleanup.expect("ordinary Stop cleanup timed out") {
            assert_eq!(cleanup.status, STATUS_CANCELLED, "{}", cleanup.message);
        }
        assert!(scenario.is_ok(), "fixture/body failed after cleanup: {scenario:#?}; observations={observed:#?}");
        assert!(sink.cancelled.lock().unwrap().is_empty());

        assert_eq!(requests.len(), 2, "both real-owner attempts must reach the sink");
        assert_eq!(remaining_errors, 0, "both generic errors must be consumed");
        assert_eq!(cancel_count, 0);
        assert_eq!(receipt_reads, 0, "there was no accepted lease to reconcile");
        for (index, (attempt, request)) in observed.iter().zip(&requests).enumerate() {
            assert_eq!(attempt.submitted_count, index + 1, "{attempt:#?}");
            assert_eq!(attempt.lease_allocations, 0, "{attempt:#?}");
            assert_eq!(attempt.frame_generation, 1, "{attempt:#?}");
            if index == 0 {
                assert!(attempt.returning, "{attempt:#?}");
            }
            assert!(!attempt.has_active_child, "{attempt:#?}");
            assert!(!attempt.has_pending_dispatch, "{attempt:#?}");
            assert_eq!(attempt.reached_goals, 0, "{attempt:#?}");
            assert!(attempt.snapshot.child_task_id.is_empty(), "{attempt:#?}");
            assert!(attempt.snapshot.child_lease.is_none(), "{attempt:#?}");
            assert_eq!(request.parent_request_id, PARENT);
            assert_eq!(request.request_id, format!("{PARENT}.nav.{}", index + 1));
            assert_eq!(request.frontier_goal_id, format!("{PARENT}.return.{}", index + 1));
            if index == 0 {
                assert_eq!(attempt.return_goal_id.as_deref(), Some(request.frontier_goal_id.as_str()));
            }
            assert_eq!(request.intent.map_frame_generation, 1);
            assert_eq!(request.intent.committed_map, identity);
            assert_eq!(request.intent.goal.pose.position, [2.0, 2.0, 0.0]);
            assert_eq!(request.boundary_owner.max_navigation_retries, 1);
        }
        assert_eq!(
            [observed[0].return_failures, observed[1].return_failures],
            [0, 0],
            "generic errors must not consume the new definite-rejection budget: {observed:#?}",
        );
        assert!(observed[0].logical_active, "{:#?}", observed[0]);
        assert!(matches!(observed[0].snapshot.status, STATUS_PENDING | STATUS_ACTIVE));
        assert!(observed[1].logical_active, "{:#?}", observed[1]);
        assert!(matches!(observed[1].snapshot.status, STATUS_PENDING | STATUS_ACTIVE));
        assert_ne!(observed[1].snapshot.last_stop_reason, "consecutive_failures_exceeded");
    }


// Review-only excerpt; NOT applied, parsed, formatted, compiled, or executed.
// Baseline: 332237e1. Insert in explore_runtime.rs's existing #[cfg(test)]
// mod tests, beside two-rejections-new-contract-tests.rs after implementation
// approval. This excerpt uses that module's existing imports and fixtures.
//
// Preconditions:
// - Add ChildNavigationSubmitError::RejectedBeforePublish and its reason arm.
// - Final admission identity is the existing generation + logical_active gate,
//   followed by pending.request_id / pending.parent_request_id. Home branching
//   still uses the existing return_goal_id. Payload is NOT dispatch identity.
//   Do NOT derive PartialEq on ChildNavigationRequest or add Arc identity gates.
//   The sink gets a clone; production's result handler gets the request retained
//   by dispatch_child. The direct-call controls below test this private identity
//   contract, not permission for untrusted external callers to forge a result.
// - Keep the original configured_handle, RecordingNavSink, enable_test_mission,
//   Stop, await_compute_idle and compute-owner lifecycle. No production hooks,
//   wrapper projection APIs, alternate owner, or fabricated pending are used.
// - max_navigation_retries=1 means two definite home failures exhaust the budget.
// - Runtime execution, when separately approved, must use an approved temporary
//   directory for configured_handle's existing reconciliation journal fixture.
//
// All state expected by these tests comes from real start/tick operations.
// Only direct duplicate/mismatched RESULT delivery clones a captured request.
// Original submit_entered/submit_release establish order; timeouts are deadlock
// watchdogs only. There is no sleep, yield, polling loop, fake terminal or ZEROED.
// Body results and snapshots are captured before ordinary Stop + compute-idle,
// then original request_compute_shutdown + compute_service.join. Assertions
// follow all cleanup steps even on contract-red paths. A watchdog may
// abort/join a stuck test waiter, but never fabricates retirement or owner proof.
//
// This test-only observation is NOT a new production frozen-request type.
#[derive(Debug, Clone, PartialEq)]
struct SchemeARaceState {
    generation: u64,
    session_generation: u64,
    parent: String,
    logical_active: bool,
    return_failures: u32,
    return_goal_id: Option<String>,
    returning: bool,
    pending: Option<(String, String)>, // (parent_request_id, request_id)
    active_lease: Option<ChildNavigationLease>,
    has_pending_cancellation: bool,
    retry_budget: Option<u32>,
    frame_generation: i64,
    reached_goals: u32,
    compute_history: u64,
    compute_fault: Option<String>,
    status: &'static str,
    message: String,
    snapshot_parent: String,
    snapshot_generation: u64,
    snapshot_revision: u64,
    last_stop_reason: String,
    snapshot_child: String,
    snapshot_lease: Option<ChildNavigationLease>,
}

async fn scheme_a_race_state(handle: &ExploreRuntimeHandle) -> SchemeARaceState {
    let guard = handle.inner.lock().await;
    let snapshot = handle.snapshot();
    SchemeARaceState {
        generation: guard.generation,
        session_generation: guard.session_generation,
        parent: guard.last_request_id.clone(),
        logical_active: guard.logical_active,
        return_failures: guard.return_failures,
        return_goal_id: guard.return_goal_id.clone(),
        returning: guard.mission.as_ref().is_some_and(|mission| mission.returning),
        pending: guard.pending_child_dispatch.as_ref().map(scheme_a_request_ids),
        active_lease: guard.active_child.as_ref().map(|child| child.lease.clone()),
        has_pending_cancellation: guard.pending_child_cancellation.is_some(),
        retry_budget: guard.active_config.as_ref().map(|config| config.max_navigation_retries),
        frame_generation: guard.costmap_frame_generation,
        reached_goals: guard.reached_goals,
        compute_history: guard.compute.history,
        compute_fault: guard.compute.fault.clone(),
        status: snapshot.status,
        message: snapshot.message,
        snapshot_parent: snapshot.request_id,
        snapshot_generation: snapshot.exploration_generation,
        snapshot_revision: snapshot.revision,
        last_stop_reason: snapshot.last_stop_reason,
        snapshot_child: snapshot.child_task_id,
        snapshot_lease: snapshot.child_lease,
    }
}

// Test observations compare only existing discrete identity; no request-wide
// PartialEq, deep bit serialization or new production hash is needed.
fn scheme_a_request_ids(request: &ChildNavigationRequest) -> (String, String) {
    (request.parent_request_id.clone(), request.request_id.clone())
}

fn scheme_a_submitted_ids(case: &SchemeARaceHarness) -> Vec<(String, String)> {
    case.sink.submitted.lock().unwrap().iter().map(scheme_a_request_ids).collect()
}

fn scheme_a_definite_rejection() -> ChildNavigationSubmitError {
    ChildNavigationSubmitError::RejectedBeforePublish(
        "test definite rejection before owner installation and goal publication".into(),
    )
}

fn scheme_a_race_config() -> ExploreStartConfig {
    ExploreStartConfig {
        motion_admission: crate::nav_runtime::test_motion_admission(),
        max_navigation_retries: 1,
        ..ExploreStartConfig::default()
    }
}

async fn scheme_a_race_wait<T>(
    label: &str,
    future: impl std::future::Future<Output = T>,
) -> Result<T, String> {
    tokio::time::timeout(Duration::from_secs(5), future)
        .await
        .map_err(|_| format!("watchdog expired: {label}"))
}

// Owns only test waiters and references to the original fixture. It does not
// write production state, except through the already existing mission helper.
struct SchemeARaceHarness {
    handle: ExploreRuntimeHandle,
    sink: Arc<RecordingNavSink>,
    map: ExploreCostmapSnapshot,
    identity: schema::CommittedMapIdentity,
    parent: &'static str,
    release: Option<Arc<tokio::sync::Notify>>,
    ticking: Option<tokio::task::JoinHandle<ExploreRuntimeSnapshot>>,
}

impl SchemeARaceHarness {
    fn new(parent: &'static str) -> Self {
        let (handle, sink) = configured_handle();
        Self {
            handle,
            sink,
            map: frontier_costmap(1),
            identity: committed_map_identity(1, "map"),
            parent,
            release: None,
            ticking: None,
        }
    }

    async fn refresh(&self) -> Result<(), String> {
        // Same committed map/domain/frame, with a fresh original input receipt.
        scheme_a_race_wait(
            "refresh original map receipt",
            self.handle.ingest_committed_costmap_snapshot_in_frame(
                self.map.clone(), self.identity.clone(), 1,
            ),
        ).await??;
        Ok(())
    }

    async fn prepare(&self) -> Result<(), String> {
        self.refresh().await?;
        let started = scheme_a_race_wait(
            "start original owner",
            self.handle.start_with_config(self.parent, scheme_a_race_config()),
        ).await?;
        if started.status != STATUS_ACCEPTED {
            return Err(format!("fixture start was not accepted: {started:#?}"));
        }
        // Installs no alternate controller or source. Frame stays at the
        // already-ingested 1; the helper enables the original tiny mission.
        enable_test_mission(&self.handle, Some(0.01)).await;
        Ok(())
    }

    async fn completed_rejection(
        &self,
    ) -> Result<(ExploreRuntimeSnapshot, u64, ChildNavigationRequest), String> {
        self.refresh().await?;
        let generation = self.handle.inner.lock().await.generation;
        self.sink.submit_errors.lock().unwrap().push_back(scheme_a_definite_rejection());
        let snapshot = scheme_a_race_wait("real rejecting tick", self.handle.tick((2.0, 2.0))).await?;
        let request = self.sink.submitted.lock().unwrap().last().cloned()
            .ok_or_else(|| format!("real tick never reached sink: {snapshot:#?}"))?;
        Ok((snapshot, generation, request))
    }

    async fn blocked_rejection(&mut self) -> Result<(u64, ChildNavigationRequest), String> {
        if self.ticking.is_some() || self.release.is_some() {
            return Err("test attempted to overlap two gated ticks".into());
        }
        self.refresh().await?;
        self.sink.submit_errors.lock().unwrap().push_back(scheme_a_definite_rejection());
        let release = Arc::new(tokio::sync::Notify::new());
        *self.sink.submit_release.lock().unwrap() = Some(release.clone());
        self.release = Some(release);
        let handle = self.handle.clone();
        self.ticking = Some(tokio::spawn(async move { handle.tick((2.0, 2.0)).await }));
        scheme_a_race_wait("original submit_entered gate", self.sink.submit_entered.notified()).await?;
        // The original sink appends submitted AFTER this gate. Read the REAL
        // frozen pending here; verify the sink's copy only after release.
        let guard = self.handle.inner.lock().await;
        let request = guard.pending_child_dispatch.clone()
            .ok_or_else(|| "submit entered without a real pending request".to_string())?;
        Ok((guard.generation, request))
    }

    fn release_submit(&mut self) {
        *self.sink.submit_release.lock().unwrap() = None;
        if let Some(release) = self.release.take() {
            release.notify_one();
        }
    }

    async fn release_tick(&mut self) -> Result<ExploreRuntimeSnapshot, String> {
        self.release_submit();
        let result = match self.ticking.as_mut() {
            Some(ticking) => scheme_a_race_wait("released real tick", ticking).await,
            None => return Err("no gated tick to release".into()),
        };
        // On watchdog expiry retain the JoinHandle for cleanup, rather than
        // detaching a task still retaining the handle and its tick_gate.
        match result {
            Ok(joined) => {
                self.ticking = None;
                joined.map_err(|error| format!("gated tick failed: {error}"))
            }
            Err(error) => Err(error),
        }
    }

    async fn cleanup(&mut self) -> Result<ExploreRuntimeSnapshot, String> {
        let mut errors = Vec::new();
        self.release_submit();
        if let Some(mut ticking) = self.ticking.take() {
            match scheme_a_race_wait("cleanup released tick", &mut ticking).await {
                Ok(Ok(_)) => {}
                Ok(Err(error)) => errors.push(format!("cleanup tick failed: {error}")),
                Err(error) => {
                    errors.push(error);
                    // Exceptional watchdog recovery only. Original Stop below
                    // remains responsible for the exploration/compute state.
                    ticking.abort();
                    let _ = ticking.await;
                }
            }
        }
        // Read the current real parent because a restart test may have changed
        // it. Never clear pending or synthesize completion as test cleanup.
        let current_parent = self.handle.inner.lock().await.last_request_id.clone();
        let stopped = scheme_a_race_wait("ordinary cleanup Stop", self.handle.stop(&current_parent)).await;
        let idle = scheme_a_race_wait("ordinary compute-idle cleanup", self.handle.await_compute_idle()).await;
        // Stop/idle settle jobs but leave the native compute owner alive. Reuse
        // the existing final shutdown + retained join pattern even when an
        // earlier scenario or cleanup step failed. No new thread controller.
        let shutdown = scheme_a_race_wait("request original compute shutdown", self.handle.request_compute_shutdown()).await;
        let joined = scheme_a_race_wait("join original compute owner", self.handle.compute_service.join()).await;
        if let Err(error) = shutdown { errors.push(format!("compute shutdown request failed: {error}")); }
        match joined {
            Ok(Ok(())) => {}
            Ok(Err(error)) | Err(error) => errors.push(format!("compute owner join failed: {error}")),
        }
        if let Err(error) = &stopped { errors.push(error.clone()); }
        if let Ok(snapshot) = &stopped {
            if snapshot.status != STATUS_CANCELLED {
                errors.push(format!("cleanup Stop did not cancel: {snapshot:#?}"));
            }
        }
        match idle {
            Ok(Ok(())) => {}
            Ok(Err(error)) | Err(error) => errors.push(format!("compute cleanup failed: {error}")),
        }
        if !errors.is_empty() { return Err(errors.join("; ")); }
        stopped
    }
}

// Called AFTER cleanup only. These are mock facts, not evidence about physical
// retirement of a real external Nav owner, especially in the wrong-ticket case.
fn scheme_a_assert_no_lease_side_effects(case: &SchemeARaceHarness, submitted: usize) {
    assert_eq!(case.sink.submitted.lock().unwrap().len(), submitted);
    assert!(case.sink.submit_errors.lock().unwrap().is_empty());
    assert_eq!(case.sink.next_goal_generation.load(Ordering::SeqCst), 0);
    assert_eq!(case.sink.receipt_reads.load(Ordering::SeqCst), 0);
    assert!(case.sink.cancelled.lock().unwrap().is_empty());
    assert!(!case.handle.orphan_cancellations.has_entries());
}

fn scheme_a_assert_home_request(request: &ChildNavigationRequest, parent: &str, sequence: u64) {
    assert_eq!(request.parent_request_id, parent);
    assert_eq!(request.request_id, format!("{parent}.nav.{sequence}"));
    assert_eq!(request.frontier_goal_id, format!("{parent}.return.{sequence}"));
    assert_eq!(request.intent.committed_map, committed_map_identity(1, "map"));
    assert_eq!(request.intent.map_frame_generation, 1);
    assert_eq!(request.intent.goal.pose.position, [2.0, 2.0, 0.0]);
    assert_eq!(request.boundary_owner.max_navigation_retries, 1);
}

#[tokio::test]
async fn scheme_a_duplicate_home_rejection_consumes_budget_once() {
    let mut case = SchemeARaceHarness::new("scheme-a-duplicate");
    let observed = async {
        case.prepare().await?;
        let (first, generation, request) = case.completed_rejection().await?;
        let before = scheme_a_race_state(&case.handle).await;
        let duplicate = scheme_a_race_wait("duplicate rejection", case.handle.record_dispatch_failure(
            generation, request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after = scheme_a_race_state(&case.handle).await;
        let duplicate_again = scheme_a_race_wait("second duplicate rejection", case.handle.record_dispatch_failure(
            generation, request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after_again = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((first, request, before, duplicate, after, duplicate_again, after_again))
    }.await;
    eprintln!("duplicate rejection before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (first, request, before, duplicate, after, duplicate_again, after_again) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&request, case.parent, 1);
    assert_eq!(before.return_failures, 1);
    assert!(before.logical_active);
    assert!(before.pending.is_none());
    assert_eq!(before, after, "duplicate changed owner, cache, pending or compute history");
    assert_eq!(before, after_again);
    assert_eq!(duplicate.status, first.status);
    assert_eq!(duplicate_again.status, first.status);
    scheme_a_assert_no_lease_side_effects(&case, 1);
}

#[tokio::test]
async fn scheme_a_old_home_result_cannot_consume_new_real_pending() {
    let mut case = SchemeARaceHarness::new("scheme-a-old-result");
    let observed = async {
        case.prepare().await?;
        let (first, old_generation, old_request) = case.completed_rejection().await?;
        let first_state = scheme_a_race_state(&case.handle).await;
        let (new_generation, new_request) = case.blocked_rejection().await?;
        let before = scheme_a_race_state(&case.handle).await;
        let duplicate = scheme_a_race_wait("old result against newer pending", case.handle.record_dispatch_failure(
            old_generation, old_request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after = scheme_a_race_state(&case.handle).await;
        let second = case.release_tick().await?;
        let second_state = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((first, first_state, old_generation, old_request, new_generation,
            new_request, before, duplicate, after, second, second_state))
    }.await;
    eprintln!("old result / new pending before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (_, first_state, old_generation, old_request, new_generation, new_request,
        before, duplicate, after, second, second_state) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&old_request, case.parent, 1);
    scheme_a_assert_home_request(&new_request, case.parent, 2);
    assert_eq!(old_generation, new_generation, "this is a same-session stale request");
    assert_eq!(before.pending.as_ref(), Some(&scheme_a_request_ids(&new_request)));
    assert_eq!(before.return_goal_id.as_deref(), Some(new_request.frontier_goal_id.as_str()));
    assert_ne!(old_request.frontier_goal_id, new_request.frontier_goal_id);
    assert_eq!(before, after, "old result must not clear new pending or admit source feedback");
    assert_eq!(duplicate.status, before.status);
    assert_eq!([first_state.return_failures, after.return_failures, second_state.return_failures], [1, 1, 2]);
    assert_eq!(second.status, STATUS_FAILED);
    assert_eq!(second.last_stop_reason, "consecutive_failures_exceeded");
    assert!(!second_state.logical_active);
    assert!(second_state.pending.is_none());
    assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&old_request), scheme_a_request_ids(&new_request)]);
    scheme_a_assert_no_lease_side_effects(&case, 2);
}

#[tokio::test]
async fn scheme_a_stop_before_home_rejection_keeps_cancelled_snapshot() {
    let mut case = SchemeARaceHarness::new("scheme-a-stop-first");
    let observed = async {
        case.prepare().await?;
        let (generation, request) = case.blocked_rejection().await?;
        let before = scheme_a_race_state(&case.handle).await;
        let stopped = scheme_a_race_wait("Stop while submit is blocked", case.handle.stop(case.parent)).await?;
        scheme_a_race_wait("Stop compute settlement", case.handle.await_compute_idle()).await??;
        let after_stop = scheme_a_race_state(&case.handle).await;
        let late = case.release_tick().await?;
        let after_late = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((generation, request, before, stopped, after_stop, late, after_late))
    }.await;
    eprintln!("Stop-before-rejection before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (generation, request, before, stopped, after_stop, late, after_late) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&request, case.parent, 1);
    assert_eq!(before.pending.as_ref(), Some(&scheme_a_request_ids(&request)));
    assert_eq!(before.return_failures, 0);
    assert_eq!(stopped.status, STATUS_CANCELLED);
    assert!(after_stop.generation > generation);
    assert!(!after_stop.logical_active);
    assert!(after_stop.pending.is_none());
    assert_eq!(after_stop.return_failures, 0);
    assert_eq!(late.status, STATUS_CANCELLED);
    assert_eq!(after_stop, after_late, "late pre-publication failure overwrote Stop");
    assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&request)]);
    scheme_a_assert_no_lease_side_effects(&case, 1);
}

#[tokio::test]
async fn scheme_a_exhausted_rejection_before_stop_preserves_newer_cancelled() {
    let mut case = SchemeARaceHarness::new("scheme-a-reject-first");
    let observed = async {
        case.prepare().await?;
        let (_, _, first_request) = case.completed_rejection().await?;
        let first_state = scheme_a_race_state(&case.handle).await;
        let (failed, generation, second_request) = case.completed_rejection().await?;
        let failed_state = scheme_a_race_state(&case.handle).await;
        let stopped = scheme_a_race_wait("Stop after exhausted failure", case.handle.stop(case.parent)).await?;
        scheme_a_race_wait("newer Stop compute settlement", case.handle.await_compute_idle()).await??;
        let after_stop = scheme_a_race_state(&case.handle).await;
        let duplicate = scheme_a_race_wait("old terminal-attempt rejection", case.handle.record_dispatch_failure(
            generation, second_request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after_duplicate = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((first_request, first_state, failed, failed_state, second_request,
            stopped, after_stop, duplicate, after_duplicate))
    }.await;
    eprintln!("rejection-before-Stop before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (first_request, first_state, failed, failed_state, second_request,
        stopped, after_stop, duplicate, after_duplicate) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&first_request, case.parent, 1);
    scheme_a_assert_home_request(&second_request, case.parent, 2);
    assert_eq!([first_state.return_failures, failed_state.return_failures], [1, 2]);
    assert_eq!(failed.status, STATUS_FAILED);
    assert_eq!(failed.last_stop_reason, "consecutive_failures_exceeded");
    assert!(!failed_state.logical_active);
    assert_eq!(stopped.status, STATUS_CANCELLED);
    assert!(after_stop.generation > failed_state.generation);
    assert_eq!(after_stop.return_failures, 2);
    assert_eq!(duplicate.status, STATUS_CANCELLED);
    assert_eq!(after_stop, after_duplicate);
    scheme_a_assert_no_lease_side_effects(&case, 2);
}

async fn scheme_a_check_restart_isolation(new_parent: &'static str) {
    let mut case = SchemeARaceHarness::new("scheme-a-restart-old");
    let observed = async {
        case.prepare().await?;
        let (old_generation, old_request) = case.blocked_rejection().await?;
        let old_state = scheme_a_race_state(&case.handle).await;
        let stopped = scheme_a_race_wait("Stop old session", case.handle.stop(case.parent)).await?;
        scheme_a_race_wait("retire old compute lifecycle", case.handle.await_compute_idle()).await??;
        case.refresh().await?;
        let restarted = scheme_a_race_wait("start while old submit is blocked", case.handle.start_with_config(
            new_parent, scheme_a_race_config(),
        )).await?;
        let before_late = scheme_a_race_state(&case.handle).await;
        // Old tick still owns tick_gate here. The requirement is that start can
        // proceed; this test neither requests nor expects a new concurrent tick.
        let late = case.release_tick().await?;
        let after_late = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((old_generation, old_request, old_state, stopped,
            restarted, before_late, late, after_late))
    }.await;
    eprintln!("restart {new_parent:?} before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (old_generation, old_request, old_state, stopped,
        restarted, before_late, late, after_late) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&old_request, case.parent, 1);
    assert_eq!(old_state.pending.as_ref(), Some(&scheme_a_request_ids(&old_request)));
    assert_eq!(stopped.status, STATUS_CANCELLED);
    assert_eq!(restarted.status, STATUS_ACCEPTED, "{}", restarted.message);
    assert!(before_late.logical_active);
    assert_eq!(before_late.parent, new_parent);
    assert_eq!(before_late.snapshot_parent, new_parent);
    assert!(before_late.generation > old_generation);
    assert_ne!(before_late.session_generation, old_state.session_generation);
    assert_eq!(before_late.session_generation, before_late.generation);
    assert_eq!(before_late.return_failures, 0);
    assert!(before_late.pending.is_none());
    assert_eq!(late.status, STATUS_ACCEPTED);
    assert_eq!(late.request_id, new_parent);
    assert_eq!(before_late, after_late, "old rejection mutated new session with parent {new_parent:?}");
    assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&old_request)]);
    scheme_a_assert_no_lease_side_effects(&case, 1);
}

#[tokio::test]
async fn scheme_a_old_rejection_cannot_mutate_different_parent_restart() {
    scheme_a_check_restart_isolation("scheme-a-restart-new").await;
}

#[tokio::test]
async fn scheme_a_old_rejection_cannot_mutate_same_parent_text_restart() {
    scheme_a_check_restart_isolation("scheme-a-restart-old").await;
}

// Two independent negative tests change exactly one existing discrete ID on
// a RESULT copy. The actual pending request is always produced by the real tick.
async fn scheme_a_check_single_id_mismatch(change_parent_id: bool) {
    let parent = if change_parent_id { "scheme-a-parent-id-only" } else { "scheme-a-request-id-only" };
    let mut case = SchemeARaceHarness::new(parent);
    let observed = async {
        case.prepare().await?;
        let (generation, real_request) = case.blocked_rejection().await?;
        let mut mismatched = real_request.clone();
        if change_parent_id {
            mismatched.parent_request_id.push_str(".wrong-parent");
        } else {
            mismatched.request_id.push_str(".wrong-attempt");
        }
        let before = scheme_a_race_state(&case.handle).await;
        let ignored = scheme_a_race_wait("single-ID mismatched result", case.handle.record_dispatch_failure(
            generation, mismatched.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after = scheme_a_race_state(&case.handle).await;
        let actual = case.release_tick().await?;
        let after_actual = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((real_request, mismatched, before, ignored, after, actual, after_actual))
    }.await;
    eprintln!("single-ID mismatch parent={change_parent_id} before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (real_request, mismatched, before, ignored, after, actual, after_actual) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&real_request, parent, 1);
    if change_parent_id {
        assert_ne!(mismatched.parent_request_id, real_request.parent_request_id);
        assert_eq!(mismatched.request_id, real_request.request_id);
    } else {
        assert_eq!(mismatched.parent_request_id, real_request.parent_request_id);
        assert_ne!(mismatched.request_id, real_request.request_id);
    }
    // Fixture-control checks only: this finite payload was cloned untouched.
    // These comparisons are NOT part of the proposed production identity guard.
    assert_eq!(mismatched.frontier_goal_id, real_request.frontier_goal_id);
    assert_eq!(mismatched.intent, real_request.intent);
    assert_eq!(mismatched.goal, real_request.goal);
    assert_eq!(mismatched.boundary_owner, real_request.boundary_owner);
    assert_eq!(mismatched.expected_map_name, real_request.expected_map_name);
    assert_eq!(mismatched.motion_admission, real_request.motion_admission);
    assert_eq!(before.pending.as_ref(), Some(&scheme_a_request_ids(&real_request)));
    assert_eq!(before.return_failures, 0);
    assert_eq!(before, after, "mismatched ID consumed pending, budget or source feedback");
    assert_eq!(ignored.status, before.status);
    assert_eq!(after_actual.return_failures, 1, "real IDs must remain consumable exactly once");
    assert!(after_actual.logical_active);
    assert!(after_actual.pending.is_none());
    assert!(matches!(actual.status, STATUS_PENDING | STATUS_ACTIVE));
    assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&real_request)]);
    scheme_a_assert_no_lease_side_effects(&case, 1);
}

#[tokio::test]
async fn scheme_a_request_id_only_mismatch_cannot_consume_home_pending() {
    scheme_a_check_single_id_mismatch(false).await;
}

#[tokio::test]
async fn scheme_a_parent_id_only_mismatch_cannot_consume_home_pending() {
    scheme_a_check_single_id_mismatch(true).await;
}

// Positive identity controls: alter ONE f64 in a direct RESULT copy while keeping
// generation, parent/request IDs and the home goal ID unchanged. The real pending
// and original submit payload stay finite and untouched. This does NOT claim a
// real home producer emits NaN, or authorize admitting malformed navigation data.
async fn scheme_a_check_float_payload_is_not_identity(use_nan: bool) {
    let parent = if use_nan { "scheme-a-nan-payload" } else { "scheme-a-signed-zero" };
    let mut case = SchemeARaceHarness::new(parent);
    let observed = async {
        case.prepare().await?;
        let (generation, real_request) = case.blocked_rejection().await?;
        let mut reported_request = real_request.clone();
        reported_request.intent.goal.pose.position[2] = if use_nan { f64::NAN } else { -0.0 };
        let before = scheme_a_race_state(&case.handle).await;
        let reported = scheme_a_race_wait("same-ID result with altered float payload", case.handle.record_dispatch_failure(
            generation, reported_request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after_reported = scheme_a_race_state(&case.handle).await;
        let duplicate = scheme_a_race_wait("duplicate same-ID altered-payload result", case.handle.record_dispatch_failure(
            generation, reported_request.clone(), scheme_a_definite_rejection(),
        )).await?;
        let after_duplicate = scheme_a_race_state(&case.handle).await;
        // Original sink now returns the queued definite rejection with the
        // original retained request. Its discrete token was already consumed.
        let original = case.release_tick().await?;
        let after_original = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((real_request, reported_request, before, reported, after_reported,
            duplicate, after_duplicate, original, after_original))
    }.await;
    eprintln!("float payload identity control NaN={use_nan} before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (real_request, reported_request, before, reported, after_reported,
        duplicate, after_duplicate, original, after_original) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&real_request, parent, 1);
    assert_eq!(scheme_a_request_ids(&reported_request), scheme_a_request_ids(&real_request));
    assert_eq!(reported_request.frontier_goal_id, real_request.frontier_goal_id);
    assert_eq!(real_request.intent.goal.pose.position[2], 0.0);
    assert!(real_request.intent.goal.pose.position[2].is_sign_positive(), "control starts with +0.0");
    let changed = reported_request.intent.goal.pose.position[2];
    if use_nan {
        assert!(changed.is_nan(), "NaN must not prevent consuming the same discrete attempt");
    } else {
        assert_eq!(changed, 0.0);
        assert!(changed.is_sign_negative(), "control changes +0.0 to -0.0");
    }
    assert_eq!(before.pending.as_ref(), Some(&scheme_a_request_ids(&real_request)));
    assert_eq!(before.return_failures, 0);
    assert_eq!(after_reported.generation, before.generation);
    assert_eq!(after_reported.return_failures, 1);
    assert!(after_reported.logical_active);
    assert!(after_reported.pending.is_none());
    assert!(after_reported.active_lease.is_none());
    assert_eq!(after_reported.compute_history, before.compute_history, "no source feedback for definite home rejection");
    assert!(matches!(reported.status, STATUS_PENDING | STATUS_ACTIVE));
    assert_eq!(duplicate.status, reported.status);
    assert_eq!(original.status, reported.status);
    assert_eq!(after_reported, after_duplicate, "duplicate same-ID report consumed twice");
    assert_eq!(after_reported, after_original, "original submit result consumed the same ID again");
    assert_eq!(after_original.return_failures, 1);
    assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&real_request)]);
    let original_sink_z = case.sink.submitted.lock().unwrap()[0].intent.goal.pose.position[2];
    assert_eq!(original_sink_z, 0.0);
    assert!(original_sink_z.is_sign_positive(), "only the direct result copy was altered");
    scheme_a_assert_no_lease_side_effects(&case, 1);
}

#[tokio::test]
async fn scheme_a_nan_payload_preserves_same_id_exactly_once_consumption() {
    scheme_a_check_float_payload_is_not_identity(true).await;
}

#[tokio::test]
async fn scheme_a_signed_zero_payload_preserves_same_id_exactly_once_consumption() {
    scheme_a_check_float_payload_is_not_identity(false).await;
}

#[tokio::test]
async fn scheme_a_success_with_foreign_ticket_stays_generic_for_home_budget() {
    let mut case = SchemeARaceHarness::new("scheme-a-foreign-ticket");
    let observed = async {
        case.prepare().await?;
        let foreign = ChildNavigationLease {
            session_id: "foreign-public-session".into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "foreign-parent".into(),
            goal_generation: 91,
            producer_runtime_generation: 11,
        };
        *case.sink.lease_override.lock().unwrap() = Some(foreign.clone());
        case.refresh().await?;
        let before = scheme_a_race_state(&case.handle).await;
        // No error is enqueued: submit REALLY returns Ok(ticket) through the
        // original lease_override seam; production validation makes it generic.
        let returned = scheme_a_race_wait("real tick with wrong successful ticket", case.handle.tick((2.0, 2.0))).await?;
        let request = case.sink.submitted.lock().unwrap().last().cloned()
            .ok_or_else(|| format!("home tick did not reach original sink: {returned:#?}"))?;
        let after = scheme_a_race_state(&case.handle).await;
        Ok::<_, String>((foreign, before, request, returned, after))
    }.await;
    eprintln!("foreign-ticket generic result before cleanup: {observed:#?}");
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (foreign, before, request, returned, after) = observed.expect("scenario failed");
    scheme_a_assert_home_request(&request, case.parent, 1);
    assert_ne!(foreign.session_id, request.request_id);
    assert_ne!(foreign.parent_request_id, request.parent_request_id);
    assert_ne!(foreign.owner_kind, schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD);
    assert_eq!(before.return_failures, 0);
    assert_eq!(after.return_failures, 0, "wrong successful ticket provides no before-publish proof");
    assert!(after.logical_active);
    assert!(after.returning);
    assert!(after.pending.is_none());
    assert!(after.active_lease.is_none());
    assert!(!after.has_pending_cancellation);
    assert_eq!(returned.status, STATUS_PENDING);
    assert!(returned.message.contains("mismatched child goal lease"));
    assert_ne!(returned.last_stop_reason, "consecutive_failures_exceeded");
    assert!(returned.child_task_id.is_empty());
    assert!(returned.child_lease.is_none());
    scheme_a_assert_no_lease_side_effects(&case, 1);
    // IMPORTANT: RecordingNavSink holds no real external Nav owner. Zero calls
    // to cancel prove only that this mismatched foreign identity was not blindly
    // cancelled here. Ordinary Explore Stop is test-fixture cleanup, NOT proof
    // that a real Some/unknown external owner is retired or physically stopped.
}

// Separate integration plan, intentionally NOT approximated by a new production
// projection wrapper in this excerpt:
// 1. Real Bridge nonaccepted Some(attempted/existing owner), and publication /
//    rollback failures retaining Some, must remain generic after original typed
//    classification; verify real owner identity and original lifecycle effects.
// 2. Real Bridge accepted=true + lease=None must remain generic; a missing lease
//    on that accepted path is not a before-publish proof. Exercise the real producer/bridge branch when a
//    supported existing seam can produce it; otherwise retain as source-audit /
//    integration-test gap rather than fake success with a synthetic Nav result.
// 3. The wrong-ticket test above covers the existing Explore consumer path only.
//    Neither that mock nor no-cancel/no-allocation counters close real Some /
//    accepted-without-lease / unknown-phase ownership recovery or physical-stop safety.

// Review-only excerpt; NOT applied, parsed, formatted, compiled, or executed.
// Baseline: 332237e1. Insert in explore_runtime.rs's existing #[cfg(test)]
// mod tests after implementation approval, alongside scheme-a-race-tests.rs.
// Reuses its scheme_a_race_wait / scheme_a_race_state / SchemeARaceState and
// the original configured_handle, RecordingNavSink and pure assertion helper
// assert_one_receipt_source_failure. It does NOT use the mission race harness.
// Exact selector:
// explore_runtime::tests::scheme_a_frontier_rejection_before_publish_preserves_source_failed_ack
//
// Source-checked prerequisites and evidence (baseline line numbers):
// - R=services/geometry/nuwax_geometry_runtime/src/explore_runtime.rs.
//   R:5864-5906 supplies a real frontier grid and complete committed identity;
//   R:5965-5971 installs the original Wavefront runtime and RecordingNavSink.
//   R:4642-4687 requires compatible identity/frame and renews an unchanged
//   input receipt. This test ingests frame generation 1 BEFORE start and keeps
//   that same identity/frame for the refresh; it never changes the live domain.
//   R:3809-3855 preflights map identity, cancel generation, map name and selects
//   through the original compute owner. No controller/source is replaced.
//   R:5565-5588 records the real request, pops submit_errors, then allocates a
//   lease. Thus this queued error is observed before any mock lease allocation.
//   R:4273-4290 is the ordinary Failed feedback + await_feedback path which
//   Scheme A must retain for a non-home RejectedBeforePublish request.
// - C=services/geometry/nuwax_geometry_runtime/src/explore_compute_service.rs.
//   C:845-911 submits feedback to the owner; C:949-964 waits for its completion;
//   C:1829-1852 installs the ACK's committed view and increments history once.
//   C:1099-1179 inspect_compute_checkpoint reads the SAME owner/source through
//   the existing serialized checkpoint operation. It does not replace it.
// - W=services/nuwax-workspace/crates/nuwax-exploration/src/wavefront_adapter.rs.
//   W:186-192 exports (version, real session). W:399-441 matches the selected
//   goal ID, records Failed at that goal's XY, and clears the selected frontier.
//   nuwax-explore-wavefront/src/session.rs:150-188 stores point/failures and
//   only blacklists after failures exceed max_navigation_retries.
//
// No mission/home helper, fabricated pending/result, terminal receipt, new
// production hook, alternate source, thread control, sleep, polling, or second
// successful navigation is used. All contract assertions follow every cleanup
// attempt, even when the body or an earlier cleanup operation returns an error.
// Runtime execution, if separately approved, needs an approved temporary
// directory for configured_handle's original reconciliation journal fixture.

#[tokio::test]
async fn scheme_a_frontier_rejection_before_publish_preserves_source_failed_ack() {
    #[derive(Debug, Default)]
    struct Observations {
        initial_ingest_changed: Option<bool>,
        refresh_changed: Option<bool>,
        started: Option<ExploreRuntimeSnapshot>,
        before: Option<SchemeARaceState>,
        before_mission_absent: Option<bool>,
        before_source: Option<serde_json::Value>,
        rejected: Option<ExploreRuntimeSnapshot>,
        submitted: Vec<ChildNavigationRequest>,
        after: Option<SchemeARaceState>,
        after_mission_absent: Option<bool>,
        after_source: Option<serde_json::Value>,
        after_checkpoint: Option<SchemeARaceState>,
    }

    // Same original inspection as receipt_test_source_history, but errors are
    // returned to the body instead of panicking before owner shutdown/join.
    async fn source_history(handle: &ExploreRuntimeHandle) -> Result<serde_json::Value, String> {
        let sources = scheme_a_race_wait(
            "inspect original Wavefront checkpoint",
            handle.inspect_compute_checkpoint(),
        ).await??;
        if sources.len() != 1 {
            return Err(format!("expected the one original source, got {sources:#?}"));
        }
        let wavefront = sources.iter().find(|source| source.name == "wavefront")
            .ok_or_else(|| format!("original Wavefront checkpoint missing: {sources:#?}"))?;
        serde_json::from_str(&wavefront.payload)
            .map_err(|error| format!("original Wavefront checkpoint decode failed: {error}"))
    }

    let parent = "scheme-a-frontier-before-publish";
    let (handle, sink) = configured_handle();
    let map = frontier_costmap(1);
    let identity = committed_map_identity(1, "map");
    let mut observed = Observations::default();
    let body = async {
        observed.initial_ingest_changed = Some(scheme_a_race_wait(
            "initial committed map in frame 1",
            handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
        ).await??);
        let started = scheme_a_race_wait(
            "start original owner without mission",
            handle.start_with_config(parent, ExploreStartConfig {
                motion_admission: crate::nav_runtime::test_motion_admission(),
                max_navigation_retries: 1,
                ..ExploreStartConfig::default()
            }),
        ).await?;
        observed.started = Some(started.clone());
        if started.status != STATUS_ACCEPTED {
            return Err(format!("ordinary frontier fixture did not start: {started:#?}"));
        }
        observed.before_source = Some(source_history(&handle).await?);
        observed.before = Some(scheme_a_race_wait(
            "capture original pre-dispatch state", scheme_a_race_state(&handle),
        ).await?);
        observed.before_mission_absent = Some(scheme_a_race_wait(
            "observe mission remains disabled before tick",
            async { handle.inner.lock().await.mission.is_none() },
        ).await?);
        // A real receipt refresh avoids aging out while checkpointing. Neither
        // the grid, committed identity, nor frame generation is changed.
        observed.refresh_changed = Some(scheme_a_race_wait(
            "refresh unchanged original committed map receipt",
            handle.ingest_committed_costmap_snapshot_in_frame(map.clone(), identity.clone(), 1),
        ).await??);
        sink.submit_errors.lock()
            .map_err(|_| "original sink error queue lock poisoned".to_string())?
            .push_back(ChildNavigationSubmitError::RejectedBeforePublish(
                "ordinary frontier rejected before owner installation and goal publication".into(),
            ));
        observed.rejected = Some(scheme_a_race_wait(
            "real ordinary frontier tick and Failed ACK", handle.tick((2.0, 2.0)),
        ).await?);
        observed.submitted = sink.submitted.lock()
            .map_err(|_| "original sink submitted lock poisoned".to_string())?.clone();
        observed.after = Some(scheme_a_race_wait(
            "capture post-ACK state before checkpoint", scheme_a_race_state(&handle),
        ).await?);
        observed.after_mission_absent = Some(scheme_a_race_wait(
            "observe mission remains disabled after rejection",
            async { handle.inner.lock().await.mission.is_none() },
        ).await?);
        observed.after_source = Some(source_history(&handle).await?);
        observed.after_checkpoint = Some(scheme_a_race_wait(
            "capture state after original source inspection", scheme_a_race_state(&handle),
        ).await?);
        Ok::<(), String>(())
    }.await;
    eprintln!("ordinary frontier rejection before cleanup: body={body:#?}; observed={observed:#?}");

    // Deliberately no ?, unwrap, expect, or assertion between these cleanup
    // operations: a failed Stop/idle/shutdown still cannot skip shutdown/join.
    let stopped = scheme_a_race_wait("ordinary frontier cleanup Stop", handle.stop(parent)).await;
    let idle = scheme_a_race_wait("ordinary frontier compute-idle cleanup", handle.await_compute_idle()).await;
    let shutdown = scheme_a_race_wait("request original compute shutdown", handle.request_compute_shutdown()).await;
    let joined = scheme_a_race_wait("join original compute owner", handle.compute_service.join()).await;
    let mut cleanup_errors = Vec::new();
    match &stopped {
        Ok(snapshot) if snapshot.status == STATUS_CANCELLED => {}
        Ok(snapshot) => cleanup_errors.push(format!("cleanup Stop did not cancel: {snapshot:#?}")),
        Err(error) => cleanup_errors.push(format!("cleanup Stop failed: {error}")),
    }
    if let Err(error) = &shutdown {
        cleanup_errors.push(format!("compute shutdown request failed: {error}"));
    }
    match &idle {
        Ok(Ok(())) => {}
        Ok(Err(error)) | Err(error) => cleanup_errors.push(format!("compute idle failed: {error}")),
    }
    match &joined {
        Ok(Ok(())) => {}
        Ok(Err(error)) | Err(error) => cleanup_errors.push(format!("compute owner join failed: {error}")),
    }
    assert!(cleanup_errors.is_empty(), "cleanup={cleanup_errors:#?}; body={body:#?}; observed={observed:#?}");
    assert!(body.is_ok(), "body={body:#?}; observed={observed:#?}");

    let before = observed.before.as_ref().expect("captured initial state");
    let after = observed.after.as_ref().expect("captured post-ACK state");
    let after_checkpoint = observed.after_checkpoint.as_ref().expect("captured post-inspection state");
    let before_source = observed.before_source.as_ref().expect("initial source checkpoint");
    let after_source = observed.after_source.as_ref().expect("post-rejection source checkpoint");
    let rejected = observed.rejected.as_ref().expect("real tick result");
    assert_eq!(observed.initial_ingest_changed, Some(true));
    assert_eq!(observed.refresh_changed, Some(false));
    assert_eq!(observed.before_mission_absent, Some(true));
    assert_eq!(observed.after_mission_absent, Some(true));
    assert_eq!(observed.submitted.len(), 1, "must reach the original sink with one real frontier");
    let request = &observed.submitted[0];
    assert_eq!(request.parent_request_id, parent);
    assert_eq!(request.request_id, format!("{parent}.nav.1"));
    assert!(!request.frontier_goal_id.is_empty());
    assert!(!request.frontier_goal_id.starts_with(&format!("{parent}.return.")));
    assert_eq!(request.goal.id, request.request_id);
    assert_eq!(request.intent.committed_map, identity);
    assert_eq!(request.intent.map_frame_generation, 1);
    assert_eq!(request.intent.producer, "wavefront");
    assert_eq!(request.intent.owner_kind, schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD);
    assert_eq!(request.expected_map_name, "map");
    assert_eq!(request.motion_admission, crate::nav_runtime::test_motion_admission());
    assert_eq!(request.boundary_owner.max_navigation_retries, 1);
    assert!(!request.boundary_owner.return_policy.enabled());
    for state in [before, after, after_checkpoint] {
        assert!(state.logical_active);
        assert!(!state.returning);
        assert!(state.return_goal_id.is_none());
        assert_eq!(state.return_failures, 0, "ordinary frontier must not consume home retry budget");
        assert!(state.pending.is_none());
        assert!(state.active_lease.is_none());
        assert!(!state.has_pending_cancellation);
        assert!(state.compute_fault.is_none());
        assert_eq!(state.frame_generation, 1);
        assert_eq!(state.reached_goals, 0);
        assert_eq!(state.parent, parent);
        assert_eq!(state.generation, before.generation);
        assert_eq!(state.session_generation, before.session_generation);
    }
    assert_eq!(rejected.status, STATUS_PENDING);
    assert_eq!(rejected.selected_source, "wavefront");
    assert!(rejected.child_task_id.is_empty());
    assert!(rejected.child_lease.is_none());
    assert_eq!(rejected.frontier_count, 0, "the actual source must release its selected frontier");
    assert_eq!(before.compute_history.checked_add(1), Some(after.compute_history),
        "ordinary dispatch rejection must await exactly one owner Feedback ACK");
    assert_eq!(after_checkpoint.compute_history, after.compute_history,
        "checkpoint inspection must not manufacture another feedback/history increment");
    assert_eq!(before_source[1]["failed_goals"], serde_json::json!([]));
    assert_eq!(before_source[1]["consecutive_failures"], serde_json::json!(0));
    assert_eq!(before_source[1]["blacklisted_goals"], serde_json::json!([]));
    assert_one_receipt_source_failure(after_source);
    assert_eq!(after_source[1]["consecutive_failures"], serde_json::json!(1));
    assert_eq!(after_source[1]["failed_goals"][0]["point"], serde_json::json!({
        "x": request.goal.target.pose.position.x,
        "y": request.goal.target.pose.position.y,
    }), "failure belongs to the original source's selected frontier position");
    assert_eq!(after_source[1]["explored_goals"], before_source[1]["explored_goals"]);
    assert!(sink.submit_errors.lock().unwrap().is_empty());
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(sink.next_goal_generation.load(Ordering::SeqCst), 0);
    assert_eq!(sink.receipt_reads.load(Ordering::SeqCst), 0);
    assert!(sink.cancelled.lock().unwrap().is_empty());
    assert!(!handle.orphan_cancellations.has_entries());
}

    async fn enable_test_mission(handle:&ExploreRuntimeHandle,limit:Option<f64>) {
        let mut guard=handle.inner.lock().await;
        Arc::make_mut(guard.active_config.as_mut().unwrap()).return_policy=ReturnPolicy {maximum_distance_m:limit,reserve_distance_m:0.0,return_on_completion:true};
        guard.mission_calibration=Some(crate::explore_mission::tests::calibration());
        guard.costmap_frame_generation=1;
    }

    #[tokio::test]
    async fn planning_resource_limit_fails_without_return_on_completion() {
        use nuwax_exploration::{ExplorationGoalSource, ExplorationSourceDiagnostics, ExploreResult};
        struct LimitedSource(Option<ExplorationPoll>);
        impl ExplorationGoalSource for LimitedSource {
            fn source_name(&self) -> std::borrow::Cow<'static, str> { "wavefront".into() }
            fn begin(&mut self) -> ExploreResult<()> { Ok(()) }
            fn end(&mut self) -> ExploreResult<()> { Ok(()) }
            fn poll_goal(&mut self, _: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
                Ok(ExplorationPoll::Stop(ExplorationStopReason::PlanningResourceLimited))
            }
            fn supports_prepared_poll(&self) -> bool { true }
            fn prepare_poll(&mut self, state: &ExplorationStateSnapshot<'_>)
                -> ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
                let outcome=self.poll_goal(state);self.0=outcome.as_ref().ok().cloned();
                Ok(nuwax_exploration::PreparedExplorationPoll::new(outcome))
            }
            fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
                self.0.take().ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)
            }
            fn discard_prepared_poll(&mut self) -> ExploreResult<()> { self.0=None;Ok(()) }
            fn diagnostics(&self) -> ExplorationSourceDiagnostics {
                ExplorationSourceDiagnostics { source_name:"wavefront".into(),is_active:true,
                    last_goal_id:None,last_error:Some("ResourceLimited: route remains inconclusive".into()) }
            }
        }
        for return_on_completion in [false,true] {
            let (handle,sink)=configured_handle();
            ingest_committed_costmap_fixture(&handle,frontier_costmap(1),1).await.unwrap();
            handle.start("resource-limited","prefer_wavefront").await;
            if return_on_completion { enable_test_mission(&handle,None).await; }
            let mut controller=ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
            controller.add_source(LimitedSource(None));controller.begin_exploration().unwrap();
            handle.install_compute_controller(controller).await.unwrap();
            let stopped=handle.tick((2.0,2.0)).await;
            assert_eq!(stopped.status,STATUS_FAILED,"{}",stopped.message);
            assert_eq!(stopped.last_stop_reason,"planning_resource_limited");
            assert!(stopped.message.contains("inconclusive"));
            assert!(sink.submitted.lock().unwrap().is_empty(),"a resource failure must not dispatch return motion");
            let guard=handle.inner.lock().await;
            assert!(guard.return_reason.is_none());
            assert!(guard.mission.as_ref().is_none_or(|mission|!mission.returning));
        }
    }

    #[tokio::test]
    async fn mission_inconclusive_candidate_does_not_blacklist_or_return() {
        assert_mission_inconclusive_route_does_not_blacklist_or_return(false).await;
    }

    #[tokio::test]
    async fn mission_inconclusive_homeward_keeps_resource_reason_through_owner() {
        assert_mission_inconclusive_route_does_not_blacklist_or_return(true).await;
    }

    async fn assert_mission_inconclusive_route_does_not_blacklist_or_return(homeward: bool) {
        use nuwax_exploration::{ExplorationGoalSource, ExplorationSourceDiagnostics, ExploreResult};
        struct CandidateSource(Arc<StdMutex<Vec<ExplorationGoalFeedback>>>, Option<ExplorationPoll>);
        impl ExplorationGoalSource for CandidateSource {
            fn source_name(&self)->std::borrow::Cow<'static,str>{"surface".into()}
            fn begin(&mut self)->ExploreResult<()>{Ok(())}
            fn end(&mut self)->ExploreResult<()>{Ok(())}
            fn poll_goal(&mut self,state:&ExplorationStateSnapshot<'_>)->ExploreResult<ExplorationPoll>{
                let mut pose=state.robot_pose.clone();
                pose.pose.orientation=Quaternion::from_yaw(std::f64::consts::FRAC_PI_2);
                Ok(ExplorationPoll::Goal(FrontierGoal::new("resource-candidate",pose,1.0)))
            }
            fn supports_prepared_poll(&self)->bool{true}
            fn prepare_poll(&mut self,state:&ExplorationStateSnapshot<'_>)->ExploreResult<nuwax_exploration::PreparedExplorationPoll>{
                let result=self.poll_goal(state);self.1=result.as_ref().ok().cloned();
                Ok(nuwax_exploration::PreparedExplorationPoll::new(result))
            }
            fn commit_prepared_poll(&mut self)->ExploreResult<ExplorationPoll>{
                self.1.take().ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)
            }
            fn discard_prepared_poll(&mut self)->ExploreResult<()>{self.1=None;Ok(())}
            fn report_goal_feedback(&mut self,_:&str,feedback:ExplorationGoalFeedback)->ExploreResult<()>{
                self.0.lock().unwrap().push(feedback);Ok(())
            }
            fn diagnostics(&self)->ExplorationSourceDiagnostics{
                ExplorationSourceDiagnostics{source_name:"surface".into(),is_active:true,last_goal_id:None,last_error:None}
            }
        }
        let handle=ExploreRuntimeHandle::with_wavefront_map_frame("map").unwrap();
        let sink=Arc::new(RecordingNavSink::default());handle.set_navigation_sink(sink.clone());
        let mut config=surface_config("map");
        config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_min_m:-0.05,z_max_m:0.2,source:"resource-limited mission fixture".into(),
        }),None).unwrap();
        config.follower.motion_limits=Some(nuwax_nav_types_messages::MotionLimits {
            model:nuwax_nav_types_messages::MotionModel::Differential,
            max_forward_mps:0.5,max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:1.0,
            linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,
            reaction_time_sec:0.05,minimum_turning_radius_m:0.5,can_rotate_in_place:false,
        });
        handle.configure_surface_source(Ok(config.clone()),37,[7;16]).await.unwrap();
        let message=surface_message("map",1);
        let ticket=handle.begin_surface_map_update(&message,now_monotonic_nanos()).await.unwrap().unwrap();
        let snapshot=Arc::new(SurfaceSnapshot::from_points(&schema::navigation_map_3d_points(&message).unwrap(),
            config.geometry.clone(),message.source_voxel_size).unwrap()
            .with_body_envelope(config.body_envelope.clone().unwrap()).unwrap());
        assert!(handle.finish_surface_map_update(ticket,Ok(snapshot.clone())).await);
        let accepted=handle.start_with_config("mission-resource-limit",ExploreStartConfig {
            motion_admission:crate::nav_runtime::test_motion_admission(),..Default::default()
        }).await;
        assert_eq!(accepted.status,STATUS_ACCEPTED,"{}",accepted.message);
        enable_test_mission(&handle,None).await;
        let pose=surface_pose(&snapshot,0.0,Quaternion::IDENTITY);
        if homeward {
            // The real non-rotating route query is inconclusive at the first
            // (homeward) leg, before candidate advice can be produced.
            let mut home=mission_pose(&pose);
            let q=Quaternion::from_yaw(std::f64::consts::FRAC_PI_2);
            home.orientation_xyzw=[q.x,q.y,q.z,q.w];
            handle.inner.lock().await.mission=Some(MissionProgress::new(home).unwrap());
        }
        let feedback=Arc::new(StdMutex::new(Vec::new()));
        let mut controller=ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        controller.add_source(CandidateSource(feedback.clone(),None));controller.begin_exploration().unwrap();
        handle.install_compute_controller(controller).await.unwrap();
        let stopped=handle.tick_with_pose(pose).await;
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        assert_eq!(stopped.status,STATUS_FAILED,"{}",stopped.message);
        assert_eq!(stopped.last_stop_reason,"planning_resource_limited","{}",stopped.message);
        if !homeward {
            assert!(stopped.message.contains("inconclusive"));
        }
        if homeward {
            assert!(matches!(feedback.lock().unwrap().as_slice(),[ExplorationGoalFeedback::Cancelled]),
                "the accepted, undispatched goal must be retired once without failed-goal feedback");
        } else {
            assert!(feedback.lock().unwrap().is_empty(),"inconclusive planning must not poison failure history");
        }
        assert!(sink.submitted.lock().unwrap().is_empty());
        let guard=handle.inner.lock().await;
        assert!(!guard.logical_active);
        assert!(guard.return_reason.is_none());
        assert!(guard.mission.as_ref().is_some_and(|mission|!mission.returning));
    }

    #[tokio::test]
    async fn mission_unreachable_candidate_is_not_budget_exhaustion() {
        use nuwax_exploration::{ExplorationGoalSource, ExplorationSourceDiagnostics, ExploreResult};
        struct CandidateSource(Arc<StdMutex<Vec<ExplorationGoalFeedback>>>, Option<ExplorationPoll>);
        impl ExplorationGoalSource for CandidateSource {
            fn source_name(&self) -> std::borrow::Cow<'static, str> { "wavefront".into() }
            fn begin(&mut self) -> ExploreResult<()> { Ok(()) }
            fn end(&mut self) -> ExploreResult<()> { Ok(()) }
            fn poll_goal(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
                let mut pose = state.robot_pose.clone();
                // 首次给出未知区域目标，失败反馈后提供可通行的替代目标。
                pose.pose.position.x = if self.0.lock().unwrap().is_empty() { 5.0 } else { 2.5 };
                Ok(ExplorationPoll::Goal(FrontierGoal::new("candidate", pose, 1.0)))
            }
            fn supports_prepared_poll(&self) -> bool { true }
            fn prepare_poll(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
                let result = self.poll_goal(state);
                self.1 = result.as_ref().ok().cloned();
                Ok(nuwax_exploration::PreparedExplorationPoll::new(result))
            }
            fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
                self.1.take().ok_or(nuwax_exploration::ExploreError::NoPreparedPoll)
            }
            fn discard_prepared_poll(&mut self) -> ExploreResult<()> { self.1 = None; Ok(()) }
            fn report_goal_feedback(&mut self, _: &str, feedback: ExplorationGoalFeedback) -> ExploreResult<()> {
                self.0.lock().unwrap().push(feedback); Ok(())
            }
            fn diagnostics(&self) -> ExplorationSourceDiagnostics {
                ExplorationSourceDiagnostics { source_name: "wavefront".into(), is_active: true, last_goal_id: None, last_error: None }
            }
        }
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("mission-route-failure", "prefer_wavefront").await;
        enable_test_mission(&handle, None).await;
        let feedback = Arc::new(StdMutex::new(Vec::new()));
        let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        controller.add_source(CandidateSource(feedback.clone(), None));
        controller.begin_exploration().unwrap();
        handle.install_compute_controller(controller).await.unwrap();
        let waiting = handle.tick((2.0, 2.0)).await;
        assert_eq!(waiting.status, STATUS_PENDING, "{}", waiting.message);
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert!(matches!(feedback.lock().unwrap()[0], ExplorationGoalFeedback::Failed(_)));
        assert!(handle.inner.lock().await.return_reason.is_none());
        let active = handle.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        assert_eq!(sink.submitted.lock().unwrap()[0].frontier_goal_id, "candidate");
    }

    #[tokio::test]
    async fn mission_unreachable_home_fails_without_return_dispatch() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("mission-home-failure", "prefer_wavefront").await;
        enable_test_mission(&handle, None).await;
        handle.inner.lock().await.mission = Some(MissionProgress::new(MissionPose {
            position: [5.0, 5.0, 0.0], orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }).unwrap());
        let failed = handle.tick((2.0, 2.0)).await;
        assert_eq!(failed.status, STATUS_FAILED, "{}", failed.message);
        assert!(failed.message.contains("返航路线核验失败"));
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn mission_budget_dispatches_home_and_requires_matching_terminal() {
        let (handle,sink)=configured_handle();
        ingest_committed_costmap_fixture(&handle,frontier_costmap(1),1).await.unwrap();
        handle.start("mission-small-budget","prefer_wavefront").await;
        enable_test_mission(&handle,Some(0.01)).await;
        let active=handle.tick((2.0,2.0)).await;
        assert_eq!(active.status,STATUS_ACTIVE,"{}",active.message);
        let requests=sink.submitted.lock().unwrap().clone();
        assert_eq!(requests.len(),1);
        assert!(requests[0].frontier_goal_id.contains(".return."));
        assert_eq!(requests[0].intent.options,nuwax_nav_types_messages::GoalOptions {
            allow_reverse:false,position_tolerance_m:0.1,yaw_tolerance_rad:0.2,
        });
        assert_eq!(requests[0].intent.goal.pose.position,[2.0,2.0,0.0]);
        assert_eq!(requests[0].intent.body_to_support_z_m, None);
        let lease=active_child_lease(&handle).await;
        assert!(handle.report_navigation_result("old-child",ChildNavigationResult::Reached(terminal_proof(&lease))).await.is_err());
        assert_eq!(handle.snapshot().status,STATUS_ACTIVE);
        let done=handle.report_navigation_result(&active.child_task_id,ChildNavigationResult::Reached(terminal_proof(&lease))).await.unwrap();
        assert_eq!(done.status,STATUS_SUCCEEDED);assert_eq!(done.last_stop_reason,"distance_budget_reached");
        assert_eq!(sink.submitted.lock().unwrap().len(),1);
    }

    #[tokio::test]
    async fn mission_active_goal_reserves_current_return_route_before_limit() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("mission-active-budget", "prefer_wavefront").await;
        handle.tick((2.0, 2.0)).await;
        enable_test_mission(&handle, Some(1.5)).await;
        let mut progress = MissionProgress::new(MissionPose {
            position: [1.3, 2.0, 0.0], orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }).unwrap();
        progress.travelled_m = 1.0;
        handle.inner.lock().await.mission = Some(progress);
        let stopping = handle.tick((2.0, 2.0)).await;
        assert_eq!(stopping.status, STATUS_STOPPING, "{}", stopping.message);
        assert_eq!(sink.cancelled.lock().unwrap().len(), 1);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        assert_eq!(handle.inner.lock().await.return_reason, Some(ExplorationStopReason::DistanceBudgetReached));
    }

    #[tokio::test]
    async fn mission_return_cancel_waits_for_zero_and_user_cancel_wins() {
        for user_cancel in [false,true] {
            let (handle,sink)=configured_handle();
            ingest_committed_costmap_fixture(&handle,frontier_costmap(1),1).await.unwrap();
            handle.start("mission-cancel","prefer_wavefront").await;
            let active=handle.tick((2.0,2.0)).await;
            let lease=active_child_lease(&handle).await;
            enable_test_mission(&handle,Some(2.0)).await;
            {let mut guard=handle.inner.lock().await;let mut p=MissionProgress::new(MissionPose {position:[2.0,2.0,0.0],orientation_xyzw:[0.0,0.0,0.0,1.0]}).unwrap();p.travelled_m=2.1;guard.mission=Some(p);}
            assert_eq!(handle.tick((2.0,2.0)).await.status,STATUS_STOPPING);
            assert_eq!(sink.cancelled.lock().unwrap().len(),1);
            assert!(handle.report_navigation_result(&active.child_task_id,ChildNavigationResult::Reached(terminal_proof(&lease))).await.is_err());
            if user_cancel {assert_eq!(handle.stop("mission-cancel").await.status,STATUS_STOPPING);}
            let proof=zero_proof(&lease,1);
            let after=handle.report_navigation_result(&active.child_task_id,ChildNavigationResult::Cancelled(proof)).await.unwrap();
            if user_cancel {
                assert_eq!(after.status,STATUS_CANCELLED);
                handle.tick((2.0,2.0)).await;
                assert_eq!(sink.submitted.lock().unwrap().len(),1);
            }else{
                assert_eq!(after.status,STATUS_ACTIVE);
                let home=handle.tick((2.0,2.0)).await;
                assert_eq!(home.status,STATUS_ACTIVE,"{}",home.message);
                let requests=sink.submitted.lock().unwrap();assert_eq!(requests.len(),2);assert!(requests[1].frontier_goal_id.contains(".return."));
            }
        }
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
        assert_eq!(evaluate_stop_conditions(&guard, now_unix_nanos(), None), None);
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
    fn assigned_journal_fixture(index: usize) -> DurableOwnershipTransition {
        let mut entry = durable_test_transition(index);
        let child = ChildNavigationLease {
            session_id: format!("child-{index}"), owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: entry.receipt.parent_request_id.clone(), goal_generation: index as u64 + 1,
            producer_runtime_generation: 77,
        };
        entry.kind = DurableOwnershipTransitionKind::ExactChildZeroed;
        entry.recovery_correlation = assigned_recovery_correlation(&child, 1);
        entry.request_fingerprint.clear();
        entry.request_identity = Some(AssignedTransitionIdentity::ExactChildZeroed {
            child_lease: child.clone(), cancel_generation: 1,
            zero_ack_id: format!("zero-{index}"), zero_applied_monotonic_ns: 1,
        });
        entry.receipt.recovery_correlation = entry.recovery_correlation.clone();
        entry.receipt.cause = EXACT_CHILD_ZEROED_CAUSE.into();
        entry.receipt.commit_disposition = EXACT_CHILD_ZEROED_DISPOSITION.into();
        entry.receipt.child_lease_fingerprint.clear();
        entry.receipt.child_lease = Some(child);
        entry.snapshot.last_stop_reason = "child_cancel_failed".into();
        entry
    }

    #[test]
    fn legacy_v1_restore_and_opaque_ack_keep_both_original_receipts() {
        struct NoHash;
        impl Drop for NoHash { fn drop(&mut self) { crate::map_checksum::FORBID_HASH.with(|flag| flag.set(false)); } }
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(true));
        let _no_hash = NoHash;
        // Shape-only constants from the original v1 fixture; no goal/proof body
        // exists here and no legacy digest is calculated or reconstructed.
        let bytes = r#"{
  "schema_version": 1,
  "entries": [
    {
      "kind": "global_stop",
      "recovery_correlation": "exploration-global-stop:0000000000000000000000000000000000000000000000000000000000000065",
      "request_fingerprint": "0000000000000000000000000000000000000000000000000000000000000066",
      "receipt": {
        "cause": "ordinary_global_stop",
        "recovery_correlation": "exploration-global-stop:0000000000000000000000000000000000000000000000000000000000000065",
        "parent_request_id": "parent-101",
        "exploration_generation": 102,
        "child_lease_fingerprint": "0000000000000000000000000000000000000000000000000000000000000067",
        "cancel_generation": 1,
        "commit_disposition": "global_stop_reconciled_child_ownership"
      },
      "snapshot": {
        "status": "FAILED",
        "message": "synthetic legacy receipt fixture; no goal or proof payload",
        "request_id": "parent-101",
        "selected_source": "wavefront",
        "last_stop_reason": "global_stop_reconciled_safe",
        "frontier_count": 0,
        "revision": 1,
        "exploration_generation": 102
      },
      "committed_unix_ns": 102,
      "acknowledged": false
    },
    {
      "kind": "exact_child_zeroed",
      "recovery_correlation": "exploration-global-stop:0000000000000000000000000000000000000000000000000000000000000066",
      "request_fingerprint": "0000000000000000000000000000000000000000000000000000000000000067",
      "receipt": {
        "cause": "late_exact_child_zeroed",
        "recovery_correlation": "exploration-global-stop:0000000000000000000000000000000000000000000000000000000000000066",
        "parent_request_id": "parent-102",
        "exploration_generation": 103,
        "child_lease_fingerprint": "0000000000000000000000000000000000000000000000000000000000000068",
        "cancel_generation": 1,
        "commit_disposition": "exact_child_zeroed_released_ownership"
      },
      "snapshot": {
        "status": "FAILED",
        "message": "synthetic legacy receipt fixture; no goal or proof payload",
        "request_id": "parent-102",
        "selected_source": "wavefront",
        "last_stop_reason": "child_cancel_failed",
        "frontier_count": 0,
        "revision": 1,
        "exploration_generation": 103
      },
      "committed_unix_ns": 103,
      "acknowledged": false
    }
  ]
}
"#;
        let path = reconciliation_test_path("v8-legacy-v1");
        write_raw_test_journal(&path, bytes.as_bytes());
        let original = read_durable_ownership_journal(&path).unwrap();
        assert_eq!(original.schema_version, 1);
        assert_eq!(original.entries.len(), 2);
        let latest = original.entries.last().unwrap();
        assert!(latest.request_identity.is_none());
        let snapshot = latest.runtime_snapshot();
        assert!(snapshot.child_lease.is_none());
        assert_eq!(snapshot.status, STATUS_FAILED);
        assert_eq!(snapshot.ownership_reconciliation_receipt.as_ref(), Some(&latest.receipt));
        assert!(!acknowledge_durable_ownership_transition(&path, &latest.recovery_correlation, "wrong-opaque", None).unwrap());
        for _ in 0..2 {
            assert!(acknowledge_durable_ownership_transition(&path, &latest.recovery_correlation,
                &latest.receipt.child_lease_fingerprint, None).unwrap());
        }
        let restarted = read_durable_ownership_journal(&path).unwrap();
        assert_eq!(restarted.schema_version, 1);
        assert_eq!(restarted.entries[0], original.entries[0]);
        assert_eq!(restarted.entries[1].request_fingerprint, latest.request_fingerprint);
        assert_eq!(restarted.entries[1].receipt, latest.receipt);
        assert!(restarted.entries[1].acknowledged);
        assert_eq!(restarted.entries.iter().rev().find(|entry| !entry.acknowledged).unwrap(), &original.entries[0]);
    }

    #[test]
    fn assigned_journal_v2_keeps_legacy_entries_and_exact_ack_branches() {
        let path = reconciliation_test_path("v8-mixed-journal");
        let legacy = durable_test_transition(1);
        write_raw_test_journal(&path, &encode_ownership_journal(&DurableOwnershipJournal {
            schema_version: 1, entries: vec![legacy.clone()],
        }).unwrap());
        let assigned = assigned_journal_fixture(2);
        assert!(matches!(commit_durable_ownership_transition(&path, assigned.clone()).unwrap(), DurableCommitOutcome::Committed));
        let journal = read_durable_ownership_journal(&path).unwrap();
        assert_eq!(journal.schema_version, 2);
        assert_eq!(journal.entries, vec![legacy.clone(), assigned.clone()]);
        let raw: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(raw["entries"][0]["format"], "legacy_v1");
        assert_eq!(raw["entries"][1]["format"], "assigned_v2");
        assert!(raw["entries"][1]["transition"].get("request_fingerprint").is_none());
        assert!(!acknowledge_durable_ownership_transition(&path, &assigned.recovery_correlation,
            &legacy.receipt.child_lease_fingerprint, assigned.receipt.child_lease.as_ref()).unwrap());
        assert!(acknowledge_durable_ownership_transition(&path, &assigned.recovery_correlation,
            "", assigned.receipt.child_lease.as_ref()).unwrap());
        assert!(acknowledge_durable_ownership_transition(&path, &legacy.recovery_correlation,
            &legacy.receipt.child_lease_fingerprint, None).unwrap());
        assert!(read_durable_ownership_journal(&path).unwrap().entries.iter().all(|entry| entry.acknowledged));
    }

    #[test]
    fn assigned_journal_first_winner_replays_once_and_rejects_real_changes() {
        let path = reconciliation_test_path("v8-assigned-first-winner");
        let entry = assigned_journal_fixture(3);
        assert!(matches!(commit_durable_ownership_transition(&path, entry.clone()).unwrap(), DurableCommitOutcome::Committed));
        assert!(matches!(commit_durable_ownership_transition(&path, entry.clone()).unwrap(), DurableCommitOutcome::ExactReplay(_)));
        let mut changed = entry.clone();
        if let Some(AssignedTransitionIdentity::ExactChildZeroed { zero_ack_id, .. }) = &mut changed.request_identity {
            zero_ack_id.push_str("-changed");
        }
        assert!(commit_durable_ownership_transition(&path, changed).err().unwrap().contains("first winner"));
        let mut mixed = entry.clone();
        mixed.receipt.child_lease_fingerprint = "0".repeat(64);
        assert!(validate_durable_ownership_transition(&mixed).is_err());
        assert_eq!(read_durable_ownership_journal(&path).unwrap().entries, vec![entry]);
    }

    #[test]
    fn assigned_write_cannot_evict_any_unacknowledged_legacy_receipt() {
        let path = reconciliation_test_path("v8-mixed-capacity");
        let entries = (0..RECONCILIATION_JOURNAL_CAPACITY).map(durable_test_transition).collect::<Vec<_>>();
        write_raw_test_journal(&path, &encode_ownership_journal(&DurableOwnershipJournal {
            schema_version: 1, entries: entries.clone(),
        }).unwrap());
        let assigned = assigned_journal_fixture(50);
        assert!(commit_durable_ownership_transition(&path, assigned.clone()).is_err());
        assert_eq!(read_durable_ownership_journal(&path).unwrap().entries, entries);
        assert!(acknowledge_durable_ownership_transition(&path, &entries[0].recovery_correlation,
            &entries[0].receipt.child_lease_fingerprint, None).unwrap());
        assert!(matches!(commit_durable_ownership_transition(&path, assigned.clone()).unwrap(), DurableCommitOutcome::Committed));
        let migrated = read_durable_ownership_journal(&path).unwrap();
        assert_eq!(migrated.schema_version, 2);
        assert_eq!(&migrated.entries[..31], &entries[1..]);
        assert_eq!(migrated.entries.last(), Some(&assigned));
    }

    #[tokio::test]
    async fn surface_return_keeps_captured_body_home_without_support_contract() {
        eprintln!("surface return: constructing complete-body source");
        let handle = ExploreRuntimeHandle::with_wavefront_map_frame("map").unwrap();
        let sink = Arc::new(RecordingNavSink::default());
        handle.set_navigation_sink(sink.clone());
        // Mission route queries require the real complete-body view, unlike
        // the older point-only surface-frontier fixture.
        let mut config = surface_config("map");
        config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_min_m: -0.05, z_max_m: 0.2, source: "surface return body fixture".into(),
        }), None).unwrap();
        handle.configure_surface_source(Ok(config.clone()), 37, [7;16]).await.unwrap();
        let message = surface_message("map", 1);
        let ticket = handle.begin_surface_map_update(&message, now_monotonic_nanos()).await.unwrap().unwrap();
        let snapshot = Arc::new(SurfaceSnapshot::from_points(
            &schema::navigation_map_3d_points(&message).unwrap(), config.geometry.clone(), message.source_voxel_size,
        ).unwrap().with_body_envelope(config.body_envelope.clone().unwrap()).unwrap());
        assert!(handle.finish_surface_map_update(ticket, Ok(snapshot.clone())).await);
        eprintln!("surface return: complete-body source committed");
        let started = handle.start_with_config("surface-return-body", ExploreStartConfig {
            motion_admission: crate::nav_runtime::test_motion_admission(), ..Default::default()
        }).await;
        assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
        enable_test_mission(&handle, Some(0.01)).await;
        let body_pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
        let home = [body_pose.x, body_pose.y, body_pose.z];
        snapshot.validate_body_pose(home, [0.0, 0.0, 0.0, 1.0]).expect("captured home has real support and complete-body clearance");
        eprintln!("surface return: home {:?} validated, d={}", home, config.follower.body_to_support_z_m);
        // A previously captured body home after a completed outward journey.
        // Exhaustion runs the real homeward query, without an unrelated new
        // frontier route search before selecting the return action.
        let mut progress = MissionProgress::new(MissionPose {
            position: home, orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }).unwrap();
        progress.travelled_m = 0.02;
        handle.inner.lock().await.mission = Some(progress);
        eprintln!("surface return: entering original exhausted-budget tick/homeward route");
        let active = handle.tick_with_pose(body_pose).await;
        eprintln!("surface return: tick completed: {} {}", active.status, active.message);
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let requests = sink.submitted.lock().unwrap();
        assert_eq!(requests.len(), 1);
        assert!(requests[0].frontier_goal_id.contains(".return."));
        assert_eq!(requests[0].intent.goal.pose.position, home);
        assert_eq!(requests[0].intent.body_to_support_z_m, None);
    }

}
