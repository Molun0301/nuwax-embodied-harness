//! Exploration controller, child-navigation handoff, and background scheduling.
//!
//! `ExploreRuntimeHandle` owns the production wavefront session. State reads are
//! pure snapshots; controller progress happens only through `tick*` or a started
//! [`ExploreSchedulerHandle`]. Mapping and navigation wiring are injected through
//! typed interfaces so the module never silently discards a selected goal.

#[cfg(test)]
#[path = "explore_attribution.rs"]
mod attribution;

#[path = "explore_compute_backend.rs"]
mod compute_backend;
#[path = "explore_compute_service.rs"]
mod explore_compute_service;
use compute_backend::*;
use explore_compute_service::*;

#[cfg(unix)]
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use nuwax_codec::lcm::schema;
use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
use nuwax_exploration::surface_adapter::{SurfaceGoalSourceAdapter, SurfaceInputReceiver};
use nuwax_exploration::wavefront_adapter::WavefrontGoalSourceAdapter;
#[cfg(test)]
use nuwax_exploration::ExplorationGoalSource;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationBoundary, ExplorationController,
    ExplorationGoalFeedback, ExplorationPoll, ExplorationStateSnapshot,
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
pub const STATUS_ACTIVE: &str = "ACTIVE";
pub const STATUS_SUCCEEDED: &str = "SUCCEEDED";
pub const STATUS_CANCELLED: &str = "CANCELLED";
pub const STATUS_FAILED: &str = "FAILED";

const POLICY_PREFER_WAVEFRONT: &str = "prefer_wavefront";
const POLICY_PREFER_TARE: &str = "prefer_tare";
const POLICY_HIGHEST_SCORE: &str = "highest_score";
const POLICY_FIRST_AVAILABLE: &str = "first_available";
const DEFAULT_SCHEDULER_PERIOD: Duration = Duration::from_millis(200);
const DEFAULT_POSE_RECEIPT_TTL: Duration = Duration::from_secs(1);
const DEFAULT_POSE_RECOVERY_TIMEOUT: Duration = Duration::from_secs(30);
const DEFAULT_SELECTION_PROGRESS_TIMEOUT: Duration = Duration::from_secs(30);
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
#[cfg(test)]
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

fn configured_pose_data_ttl() -> Duration {
    std::env::var("NUWAX_EXPLORE_POSE_TTL_MS").ok().and_then(|v| v.parse().ok())
        .map(Duration::from_millis).unwrap_or(DEFAULT_POSE_RECEIPT_TTL)
}

fn pose_input_is_fresh(pose: &ExploreRobotPose, now_monotonic_ns: u64) -> bool {
    use nuwax_nav_types_messages::source_time;
    pose.received_monotonic_ns != 0
        && source_time::input_is_alive(now_monotonic_ns.saturating_sub(pose.received_monotonic_ns))
        && source_time::data_age_ns_at(pose.source_timestamp_ns, now_unix_nanos())
            .is_ok_and(|age| age <= configured_pose_data_ttl().as_nanos() as u64)
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

#[derive(Debug, Clone, PartialEq)]
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
        boundary_points,

        motion_guard_boot_nonce: motion_admission.guard_boot_nonce,
        motion_stop_generation: i64::try_from(motion_admission.motion_stop_generation).map_err(|_| "motion_stop_generation cannot fit LCM i64".to_string())?,
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

/// Trusted installed-unresolved responsibility. Only the Nav-backed Bridge constructs
/// this carrier; external sinks cannot upgrade an arbitrary Some lease into ownership.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationUnresolved {
    pub(crate) lease: ChildNavigationLease,
    pub(crate) reason: String,
}

/// 桥接器按完整子目标身份保留的终态结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildNavigationResolved {
    pub(crate) result: ChildNavigationResult,
}

impl ChildNavigationResolved {
    fn lease(&self) -> &ChildNavigationLease {
        match &self.result {
            ChildNavigationResult::Reached(proof) | ChildNavigationResult::Failed { proof, .. } => &proof.lease,
            ChildNavigationResult::Cancelled(proof) => &proof.lease,
        }
    }

    fn reason(&self) -> &str {
        match &self.result {
            ChildNavigationResult::Reached(_) => "child navigation already reached",
            ChildNavigationResult::Failed { reason, .. } => reason,
            ChildNavigationResult::Cancelled(_) => "child navigation already cancelled",
        }
    }
}

fn child_lease_matches_request(lease: &ChildNavigationLease, request: &ChildNavigationRequest) -> bool {
    lease.session_id == request.request_id
        && lease.parent_request_id == request.parent_request_id
        && lease.owner_kind == schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        && (request.intent.goal_generation <= 0
            || u64::try_from(request.intent.goal_generation).ok() == Some(lease.goal_generation))
        && lease.goal_generation > 0 && lease.producer_runtime_generation > 0
}

/// Child navigation submission failure; the variant carries any admission-stage proof.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChildNavigationSubmitError {
    /// This invocation installed an exact owner and must settle it before business feedback.
    DispatchUnresolved(ChildNavigationUnresolved),
    /// A terminal receipt already settled the original owner; independent of accepted.
    DispatchResolved(ChildNavigationResolved),
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
            Self::DispatchUnresolved(unresolved) => &unresolved.reason,
            Self::DispatchResolved(resolved) => resolved.reason(),
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
}

/// 子导航失败是否只因输入中断且已围栏停车。导航终态只经状态消息传递，没有独立的类型化原因。
/// 输入时效错误以`StaleInput`失败；自a51326fd起，近期障碍源到期、来源撤销和地图来源暂不可用改为
/// 保持租约等待新鲜输入，持续到原路径等待期限后以PlannerFailed围栏结束，终态消息保留等待原因。
fn child_failure_is_input_interruption(reason: &str) -> bool {
    reason.starts_with("SafetyFailed cause=StaleInput:")
        || (reason.starts_with("planner path timeout after ") && reason.contains("; 等待新鲜导航输入: "))
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
            let guard = {
                let mut guard = runtime.inner.lock().await;
                guard.retained_child_dispatch = false;
                guard
            };
            let mut current = runtime.snapshot_cache.write();
            if current.request_id == guard.last_request_id {
            }
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
    ) -> Result<(), String>;

    /// Read only a natural terminal that authoritatively retired this exact
    /// execution. An absent active lease alone is not proof of completion.
    async fn terminal_result_for(&self, _lease: &ChildNavigationLease) -> Option<ChildNavigationResult> {
        None
    }

}

#[async_trait]
pub trait ExplorePoseProvider: Send + Sync {
    async fn current_pose(&self) -> Result<ExploreRobotPose, String>;
}

#[derive(Debug, Clone)]
pub struct ExploreRuntimeSnapshot {
    /// 真实子取消或在途派发仍待停车，不由业务FAILED推断停稳。
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
            mission_progress_json: String::new(),
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SurfaceMapTicket {
    pub(crate) committed_map: schema::CommittedMapIdentity,
    producer_runtime_generation: i64,
    map_frame_generation: i64,
    cloud_revision: i64,
    commit_boot_nonce: [u8; 16],
    received_monotonic_ns: u64,
    source_timestamp_ns: u64,
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
    costmap_source_timestamp_ns: Option<u64>,
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
    progress_stopped_ns: Option<u64>,
    progress_map_name: String,
    progress_origin: Option<(String,i64,i64)>,
    active_child: Option<ActiveChildNavigation>,
    pending_child_dispatch: Option<ChildNavigationRequest>,
    // One accepted dispatch owns the original tick permit across all epochs.
    // Neither Stop nor Start clears this; only that operation may settle it.
    retained_child_dispatch: bool,
    session_generation: u64,
    generation: u64,
    child_sequence: u64,
    reached_goals: u32,
    started_at_ns: u64,
    // 与守卫回执共用 CLOCK_MONOTONIC，不改变进度和输入新鲜度的原时钟。
    started_guard_clock_ns: u64,
    no_frontier_since: Option<Instant>,
    pose_wait_since: Option<Instant>,
    selection_wait_since: Option<Instant>,
    frontiers: Vec<FrontierGoal>,
    frontier_revision: u64,
}

impl ExploreRuntimeInner {
    /// 同域最新输入按来源钟判龄；已安装身份及计算完成时刻不提供续期。
    fn map_input_is_fresh(&self, now_monotonic_ns: u64) -> bool {
        use nuwax_nav_types_messages::source_time;
        match &self.surface_mode {
            Some(mode) => {
                if !mode.accepting || !mode.input_ready || mode.installed_revision.is_none() {
                    return false;
                }
                mode.latest.as_ref().is_some_and(|ticket| {
                    source_time::input_is_alive(now_monotonic_ns.saturating_sub(ticket.received_monotonic_ns))
                        && ticket.received_monotonic_ns != 0
                        && source_time::data_age_ns_at(ticket.source_timestamp_ns, now_unix_nanos())
                            .is_ok_and(|age| age <= self.costmap_receipt_ttl.as_nanos() as u64)
                })
            }
            None => self.costmap_received_monotonic_ns != 0 && self.costmap_source_timestamp_ns.map_or_else(
                // 旧二维兼容投影没有独立观测戳，仍仅保留原接收期限。
                || now_monotonic_ns.saturating_sub(self.costmap_received_monotonic_ns)
                    <= self.costmap_receipt_ttl.as_nanos() as u64,
                |source| source_time::input_is_alive(now_monotonic_ns.saturating_sub(self.costmap_received_monotonic_ns))
                    && source_time::data_age_ns_at(source, now_unix_nanos())
                        .is_ok_and(|age| age <= self.costmap_receipt_ttl.as_nanos() as u64)),
        }
    }

    fn new() -> Self {
        Self {
            logical_active: false,
            committed: CommittedView::capture(&ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront)),
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
            costmap_source_timestamp_ns: None,
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
            progress_stopped_ns: None,
            progress_map_name: String::new(),
            progress_origin: None,
            active_child: None,
            pending_child_dispatch: None,
            retained_child_dispatch: false,
            session_generation: 0,
            generation: 0,
            child_sequence: 0,
            reached_goals: 0,
            started_at_ns: 0,
            started_guard_clock_ns: 0,
            no_frontier_since: None,
            pose_wait_since: None,
            selection_wait_since: None,
            frontiers: Vec::new(),
            frontier_revision: 0,
        }
    }

    fn session_in_progress(&self) -> bool {
        self.logical_active || self.compute.start_reservation.is_some() || self.compute.control_pending()
    }

    /// 待继续、换图丢弃和短暂失鲜共用同一等待期限，只有实际交付子导航才清除。
    fn selection_wait_failure(&mut self) -> Option<String> {
        if !self.logical_active || self.active_child.is_some()

            || (self.no_frontier_since.is_some()
                && matches!(self.last_poll, Some(ExplorationPoll::NoFrontier))) {
            return None;
        }
        let since = *self.selection_wait_since.get_or_insert_with(Instant::now);
        let limit = self.active_config.as_ref()
            .and_then(|config| config.stop_conditions.no_frontier_timeout_ms)
            .map(Duration::from_millis)
            .unwrap_or(DEFAULT_SELECTION_PROGRESS_TIMEOUT);
        (since.elapsed() >= limit).then(|| {
            let code = if self.map_input_is_fresh(now_monotonic_nanos()) {
                "exploration_selection_stalled"
            } else {
                "exploration_source_stalled"
            };
            format!("ResourceLimited: {code}，连续{:.1}秒未能交付子导航；{}；前沿是否可达尚未确定",
                since.elapsed().as_secs_f64(), self.selection_wait_detail())
        })
    }

    /// 直接记录接纳、安装和计算事实，避免把来源等待统一解释成前沿计算未完成。
    fn selection_wait_detail(&self) -> String {
        let now = now_monotonic_nanos();
        let compute = self.compute.query_pending();
        let dispatch = self.pending_child_dispatch.is_some();
        let poll = match self.last_poll.as_ref() {
            Some(ExplorationPoll::Pending) => "pending",
            Some(ExplorationPoll::SourceUnavailable(_)) => "source_unavailable",
            Some(ExplorationPoll::NoFrontier) => "no_frontier",
            Some(ExplorationPoll::Goal(_)) => "goal",
            Some(ExplorationPoll::Stop(_)) => "stop",
            None => "none",
        };
        if let Some(mode) = self.surface_mode.as_ref() {
            let age = mode.latest.as_ref().map(|ticket|
                now.saturating_sub(ticket.received_monotonic_ns) / 1_000_000);
            let data_age = mode.latest.as_ref().map(|ticket|
                nuwax_nav_types_messages::source_time::data_age_ns_at(ticket.source_timestamp_ns, now_unix_nanos())
                    .map(|age| age / 1_000_000));
            let state = if !mode.accepting { "surface input owner stopped" }
                else if !mode.input_ready { "surface map input unavailable" }
                else if !self.map_input_is_fresh(now) { "surface map input stale" }
                else if mode.latest.as_ref().map(SurfaceMapTicket::revision) != mode.installed_revision {
                    "surface map build pending"
                } else { "surface map input fresh" };
            format!("{state}；已接纳修订={:?}；已安装修订={:?}；接收年龄={age:?}毫秒；来源数据年龄={data_age:?}毫秒；来源就绪={}；来源错误={:?}；计算在途={compute}；待派发={dispatch}；上次选择={poll}",
                mode.latest.as_ref().map(SurfaceMapTicket::revision), mode.installed_revision,
                mode.input_ready, mode.input_error)
        } else {
            let state = if self.map_input_is_fresh(now) { "costmap input fresh" } else { "costmap input stale" };
            format!("{state}；接收年龄={}毫秒；计算在途={compute}；待派发={dispatch}；上次选择={poll}",
                now.saturating_sub(self.costmap_received_monotonic_ns) / 1_000_000)
        }
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

            mission_progress_json: self.mission.as_ref().map(|p|serde_json::to_string(p).expect("有效行程载荷")).unwrap_or_default(),
        }
    }
}

impl Default for ExploreRuntimeInner {
    fn default() -> Self {
        Self::new()
    }
}

fn build_controller_with_geometry(
    policy: ExplorationArbitrationPolicy,
    config: &ExploreStartConfig,
    geometry: nuwax_nav_types_messages::RobotGeometry,
    observation: Option<nuwax_nav_types_messages::ObservationModel>,
) -> Result<ExplorationController,String> {
    let mut controller = ExplorationController::new(policy);
    let mut adapter =
        WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default(), geometry);
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
        let value = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").map_err(|error| error.to_string())?;
        let geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value)?;
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

#[derive(Clone)]
pub struct ExploreRuntimeHandle {
    #[cfg(test)]
    attribution: Option<Arc<attribution::Attribution>>,
    #[cfg(test)]
    homeward_test_hook: Option<Arc<dyn Fn() + Send + Sync>>,
    inner: Arc<Mutex<ExploreRuntimeInner>>,
    compute_service: Arc<ComputeService>,
    tick_gate: Arc<Mutex<()>>,
    // 只串行完整表面安装与所选结果复核，接收/清空/停止仍使用原状态锁。
    surface_install_gate: Arc<Mutex<()>>,
    expected_map_frame: Arc<str>,
    readiness_cache: Arc<SyncMutex<ExplorationReadinessDetail>>,
    snapshot_cache: Arc<RwLock<ExploreRuntimeSnapshot>>,
    navigation_sink: Arc<RwLock<Option<Arc<dyn ExploreNavigationSink>>>>,
    frontier_tx: watch::Sender<FrontierSnapshot>,
    generation: Arc<AtomicU64>,
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

    #[cfg(test)]
    /// 接收新消息身份并立即处理清空/域变化；表面计算由外部受管工作执行。
    pub(crate) async fn begin_surface_map_update(
        &self,
        message: &schema::NavigationMap3D,
        received_monotonic_ns: u64,
    ) -> Result<Option<SurfaceMapTicket>, String> {
        self.begin_surface_map_update_with_retained_source(message, received_monotonic_ns, None)
            .await.map(|(ticket, _)| ticket)
    }

    pub(crate) async fn begin_surface_map_update_with_retained_source(
        &self,
        message: &schema::NavigationMap3D,
        received_monotonic_ns: u64,
        retained: Option<&Arc<SurfaceSnapshot>>,
    ) -> Result<(Option<SurfaceMapTicket>, bool), String> {
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
            source_timestamp_ns: message.source_timestamp_ns as u64,
        };
        let mut guard = self.inner.lock().await;
        let (invalidates, same_installed_source) = {
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
                        schema::navigation_map_clear_reason(message)
                    } else {
                        "surface map domain changed"
                    }
                    .into(),
                );
            }
            // This is only actual installation identity, not mask equality.
            // The shared retry dependency checks actual mask layout/contents.
            let same_installed_source = !invalidates && retained.is_some_and(|retained|
                mode.receiver.snapshot().ok().flatten()
                    .is_some_and(|installed| Arc::ptr_eq(&installed, retained)));
            (invalidates, same_installed_source)
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
                    schema::navigation_map_clear_reason(message)
                } else {
                    "surface map domain changed"
                }
                .into(),
            )
            .await;
        }
        let backend = self.readiness_cache.lock().backend_available;
        self.refresh_readiness_cache_with_backend(backend);
        Ok(((!message.cleared).then_some(ticket), same_installed_source))
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
        let _install = if result.is_ok() { Some(self.surface_install_gate.lock().await) } else { None };
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
        drop(_install);
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

    #[cfg(test)]
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
            surface_install_gate: Arc::new(Mutex::new(())),
            #[cfg(test)]
            attribution: None,
            #[cfg(test)]
            homeward_test_hook: None,
            expected_map_frame,
            readiness_cache: Arc::new(SyncMutex::new(ExplorationReadinessDetail {
                ready: false,
                #[cfg(test)] reason: "exploration controller not started".to_string(),
                backend_available: false,
                controller_active: false,
                source_count: 1,
                frontier_source_ready: false,
                costmap_input_ready: false,
                #[cfg(test)] navigation_sink_ready: false,
                active_source: "wavefront".to_string(),
                arbitration: POLICY_PREFER_WAVEFRONT.to_string(),
            })),
            snapshot_cache: Arc::new(RwLock::new(ExploreRuntimeSnapshot::default())),
            navigation_sink: Arc::new(RwLock::new(None)),
            frontier_tx,
            generation: Arc::new(AtomicU64::new(0)),
        }
    }

    #[cfg(test)]
    pub fn expected_map_frame(&self) -> &str {
        self.expected_map_frame.as_ref()
    }

    pub(crate) fn producer_epoch_projection(&self) -> Arc<AtomicU64> {
        self.generation.clone()
    }

    pub fn set_navigation_sink(&self, sink: Arc<dyn ExploreNavigationSink>) {
        *self.navigation_sink.write() = Some(sink);
    }

    #[cfg(test)]
    pub fn clear_navigation_sink(&self) {
        *self.navigation_sink.write() = None;
    }

    pub fn subscribe_frontiers(&self) -> watch::Receiver<FrontierSnapshot> {
        self.frontier_tx.subscribe()
    }

    #[cfg(test)]
    pub fn frontier_snapshot(&self) -> FrontierSnapshot {
        self.frontier_tx.borrow().clone()
    }

    pub fn snapshot(&self) -> ExploreRuntimeSnapshot {
        self.snapshot_cache.read().clone()
    }

    fn store_snapshot(&self, snapshot: ExploreRuntimeSnapshot) -> ExploreRuntimeSnapshot {
        // 缓存仅保存当前业务投影；原孤儿是否停车由现有拥有方实时提供。
        *self.snapshot_cache.write() = snapshot;
        self.snapshot()
    }

    fn publish_frontiers(&self, snapshot: Option<FrontierSnapshot>) {
        if let Some(snapshot) = snapshot {
            self.frontier_tx.send_replace(snapshot);
        }
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
        self.end_for_navigation_takeover().await;
        let surface_mode = {
            let guard = self.inner.lock().await;

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
        let mut restored=if config.resume_progress_json.is_empty(){None}else{
            let result=(|| {
                let saved=ExplorationCheckpoint::parse(&config.resume_progress_json)?;
                let map_name=self.navigation_sink.read().as_ref().ok_or("导航地图来源缺失")?.current_map_name()?;
                if saved.map_name!=map_name || saved.map_frame_id!=self.expected_map_frame.as_ref() {
                    return Err("探索进度与当前地图不匹配".into());
                }
                saved.validate_resume_configuration(
                    &ExplorationConfiguration::capture(&config, surface_mode),
                    navigation_configuration.as_ref().map_err(Clone::clone)?,
                )?;
                if config.return_policy.enabled()!=saved.mission.is_some(){return Err("探索进度缺少行程或请求未启用行程".into());}
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
            sources: std::mem::take(&mut saved.sources),
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
        guard.progress_stopped_ns=None;guard.progress_map_name.clear();guard.progress_origin=None;
        guard.active_child = None;
        guard.pending_child_dispatch = None;
        guard.session_generation = generation;
        guard.generation = generation;
        guard.child_sequence = 0;
        guard.reached_goals = 0;
        guard.started_at_ns = now_unix_nanos();
        guard.pose_wait_since = None;
        guard.selection_wait_since = None;
        guard.started_guard_clock_ns = nuwax_motion_safety_guard::measured_motion::monotonic_ns();
        if let Some(saved)=restored {
            guard.mission=saved.mission;guard.return_reason=saved.return_reason;guard.reached_goals=saved.reached_goals;
            guard.started_at_ns=guard.started_at_ns.saturating_sub(saved.elapsed_ns);
        }
        guard.no_frontier_since = None;
        let frontier_update = guard.update_frontiers(Vec::new());
        let snapshot = guard.snapshot(
            STATUS_ACCEPTED,
            format!(
                "exploration accepted (source={}, arbitration={})",
                if surface_mode { "surface" } else { "wavefront" },
                arbitration_policy_str(policy),
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

    async fn begin_child_action(&self, request_id: Option<&str>, action: PendingChildAction,
        compute_action: Option<&ComputeStopAction>) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if compute_action.is_some_and(|action| !action.matches(&guard)) { return self.snapshot(); }
        if let Some(requested_id) = request_id.map(str::trim) {
            if requested_id.is_empty() || requested_id != guard.last_request_id {
                let mut snapshot = guard.snapshot(STATUS_REJECTED,
                    format!("exploration stop request_id mismatch: active={} received={}", guard.last_request_id, requested_id));
                snapshot.request_id = requested_id.to_string();
                return snapshot;
            }
        }
        let child = guard.active_child.take();
        let generation = if compute_action.is_some() { guard.generation } else { self.next_exploration_generation(&mut guard) };
        self.record_child_action_stop_locked(&mut guard, generation, action.reason);
        let (snapshot, readiness, frontier_update) = self.prepare_no_child_stop_locked(&mut guard, action);
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontier_update);
        let snapshot = self.store_snapshot(snapshot);
        let sink = self.navigation_sink.read().clone();
        if let (Some(child), Some(sink)) = (child, sink) {
            if let Err(error) = sink.cancel(&child.lease).await {
                return self.store_snapshot(ExploreRuntimeSnapshot { status: STATUS_FAILED, message: error, ..snapshot });
            }
        }
        snapshot
    }

    pub async fn end_for_navigation_takeover(&self) {
        let mut guard = self.inner.lock().await;
        if !guard.session_in_progress() { return; }
        guard.active_child = None;
        let generation = self.next_exploration_generation(&mut guard);
        self.record_child_action_stop_locked(&mut guard, generation, ExplorationStopReason::UserCancelled);
        let (snapshot, readiness, frontiers) = self.prepare_no_child_stop_locked(&mut guard, PendingChildAction {
            reason: ExplorationStopReason::UserCancelled, message: "探索已被新运动命令接管".into(),
        });
        drop(guard);
        *self.readiness_cache.lock() = readiness;
        self.publish_frontiers(frontiers);
        self.store_snapshot(snapshot);
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

    #[cfg(test)]
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
        self.tick_generation(generation, pose, None).await
    }

    async fn tick_generation(
        &self,
        generation: u64,
        pose: ExploreRobotPose,
        pose_provider: Option<&dyn ExplorePoseProvider>,
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
            || !pose_input_is_fresh(&pose, now_monotonic_nanos())
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
        guard.pose_wait_since = None;

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
            && guard.map_input_is_fresh(now_monotonic_nanos())
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
                    guard.active_child = None;
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

        if let Some(error) = guard.selection_wait_failure() {
            drop(guard);
            return self.finish_session(None, ExplorationStopReason::PlanningResourceLimited, error).await;
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
        if !guard.map_input_is_fresh(now_monotonic_nanos()) {
            guard.no_frontier_since = None;
            let snapshot = guard.snapshot(STATUS_PENDING,
                format!("source unavailable: {}", guard.selection_wait_detail()));
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
            return self.dispatch_child_with_fresh_pose(generation, request, guard, tick_permit, pose_provider).await;
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
        let finalized = match self.await_compute(&selection_ticket).await {
            Ok(value) => value,
            Err(_) => return self.snapshot(),
        };
        let install_permit = match selection_ticket.qualify(self).await {
            Ok(permit) => permit,
            Err(error) => return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await,
        };
        guard = self.inner.lock().await;
        if !selection_ticket.current(&guard) { return self.snapshot(); }
        drop(install_permit);
        // 资格比较跨过原连续等待期限时，不能再把迟到结果交给子导航。
        if let Some(error) = guard.selection_wait_failure() {
            drop(guard);
            return self.finish_session(None, ExplorationStopReason::PlanningResourceLimited, error).await;
        }
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
                        let install_permit = match selection_ticket.qualify(self).await {
                            Ok(permit) => permit,
                            Err(error) => return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await,
                        };
                        guard = self.inner.lock().await;
                        drop(install_permit);
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
                self.dispatch_child_with_fresh_pose(generation, request, guard, tick_permit, pose_provider).await
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
                    format!("exploration source is awaiting work completion；{}", guard.selection_wait_detail()),
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

    async fn dispatch_child_with_fresh_pose(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        guard: tokio::sync::MutexGuard<'_, ExploreRuntimeInner>,
        tick_permit: tokio::sync::OwnedMutexGuard<()>,
        pose_provider: Option<&dyn ExplorePoseProvider>,
    ) -> ExploreRuntimeSnapshot {
        let Some(provider) = pose_provider else {
            return self.dispatch_child(generation, request, guard, tick_permit).await;
        };
        // 原待派发请求已拥有本轮结果。等待不重新领取候选，也不释放单次派发许可。
        drop(guard);
        loop {
            match provider.current_pose().await {
                Ok(pose) => {
                    let mut guard = self.inner.lock().await;
                    if generation != guard.generation || !guard.logical_active
                        || guard.pending_child_dispatch.as_ref() != Some(&request) {
                        return self.snapshot();
                    }
                    if ![pose.x, pose.y, pose.z].iter().all(|value| value.is_finite())
                        || !pose.orientation.yaw().is_finite()
                        || normalize_frame(&pose.frame_id) != self.expected_map_frame.as_ref()
                        || pose.source_timestamp_ns == 0 || pose.received_monotonic_ns == 0
                        || !pose_input_is_fresh(&pose, now_monotonic_nanos()) {
                        drop(guard);
                        return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded,
                            "恢复后的探索位姿无效".into()).await;
                    }
                    guard.pose_wait_since = None;
                    if !guard.map_input_is_fresh(now_monotonic_nanos()) {
                        let snapshot = guard.snapshot(STATUS_PENDING, "目标决策已保留，等待当前地图输入恢复");
                        drop(guard);
                        return self.store_snapshot(snapshot);
                    }
                    // 运动准入、当前位姿及地图身份仍由原导航投递拥有方再次核验。
                    return self.dispatch_child(generation, request, guard, tick_permit).await;
                }
                Err(error) => {
                    self.record_pose_wait(error).await;
                    if !self.is_active().await { return self.snapshot(); }
                }
            }
            tokio::time::sleep(DEFAULT_SCHEDULER_PERIOD).await;
        }
    }

    async fn record_pose_wait(&self, error: String) -> ExploreRuntimeSnapshot {
        let mut guard = self.inner.lock().await;
        if !guard.logical_active { return self.snapshot(); }
        let since = *guard.pose_wait_since.get_or_insert_with(Instant::now);
        if let Some(error) = guard.selection_wait_failure() {
            drop(guard);
            return self.finish_session(None, ExplorationStopReason::PlanningResourceLimited, error).await;
        }
        guard.no_frontier_since = None;
        guard.last_poll = Some(ExplorationPoll::SourceUnavailable(error.clone()));
        let parent_expired = guard.active_config.as_ref().and_then(|config| config.stop_conditions.max_duration_ms)
            .is_some_and(|limit| now_unix_nanos().saturating_sub(guard.started_at_ns) >= limit.saturating_mul(1_000_000));
        let input_expired = guard.active_child.is_none() && since.elapsed() >= DEFAULT_POSE_RECOVERY_TIMEOUT;
        if parent_expired || input_expired {
            drop(guard);
            return self.finish_session(None, ExplorationStopReason::NoReachableFreeSpace,
                format!("位姿输入持续未恢复，等待已到期限（{:.1}秒）：{error}", since.elapsed().as_secs_f64())).await;
        }
        let status = if guard.active_child.is_some() { STATUS_ACTIVE } else { STATUS_PENDING };
        let retained = if guard.pending_child_dispatch.is_some() { "目标决策已保留" } else { "等待新鲜输入" };
        let snapshot = guard.snapshot(status, format!("等待位姿输入恢复，{retained}（已等待{:.1}秒）：{error}", since.elapsed().as_secs_f64()));
        drop(guard);
        self.store_snapshot(snapshot)
    }

    async fn refresh_compute_status(&self, elapsed: Duration) {
        // 选择 future 可能已排队取得此锁；监视分支不能再排队后暂停轮询选择，
        // 否则锁许可留给选择、监视却一直等待，原调度任务会自等。
        let Ok(mut guard) = self.inner.try_lock() else { return; };
        if !guard.logical_active { return; }
        if let Some(error) = guard.selection_wait_failure() {
            drop(guard);
            self.finish_session(None, ExplorationStopReason::PlanningResourceLimited, error).await;
            return;
        }
        let expired = guard.active_config.as_ref().and_then(|config| config.stop_conditions.max_duration_ms)
            .is_some_and(|limit| now_unix_nanos().saturating_sub(guard.started_at_ns) >= limit.saturating_mul(1_000_000));
        // 需要返航的任务仍由原tick按同一份路线建议处理期限，状态刷新不能抢先结束父任务。
        let return_on_completion = guard.mission.is_some()
            && guard.active_config.as_ref().is_some_and(|config| config.return_policy.return_on_completion);
        if expired && !return_on_completion {
            drop(guard);
            self.finish_session(None, ExplorationStopReason::DurationExceeded, "探索计算期间已到任务期限".into()).await;
        } else if guard.compute.query_pending() && guard.pose_wait_since.is_none() {
            let status = if guard.active_child.is_some() { STATUS_ACTIVE } else { STATUS_PENDING };
            let snapshot = guard.snapshot(status, format!("正在评估探索前沿（已计算{:.1}秒）", elapsed.as_secs_f64()));
            drop(guard);
            self.store_snapshot(snapshot);
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
        let frontier_goal_id = request.frontier_goal_id.clone();
        let (ticket, mut resolved) = match sink.submit(request.clone()).await {
            Ok(ticket) => (ticket, None),
            Err(ChildNavigationSubmitError::DispatchResolved(resolved)) => {
                if !child_lease_matches_request(resolved.lease(), &request) {
                    return self.record_dispatch_failure(generation, request,
                        ChildNavigationSubmitError::Rejected("navigation sink returned a mismatched resolved child lease".into())).await;
                }
                match &resolved.result {
                    ChildNavigationResult::Reached(_) | ChildNavigationResult::Failed { .. } => {}
                    ChildNavigationResult::Cancelled(_) => {
                        return self.complete_resolved_dispatch_cancellation(generation, request, resolved, owner).await;
                    }
                }
                (ChildNavigationTicket { lease: resolved.lease().clone() }, Some(resolved))
            }
            Err(ChildNavigationSubmitError::DispatchUnresolved(unresolved)) => {
                let _ = sink.cancel(&unresolved.lease).await;
                return self.record_dispatch_failure(generation, request, ChildNavigationSubmitError::DispatchUnresolved(unresolved)).await;
            }
            Err(error) => {
                return self.record_dispatch_failure(generation, request, error).await;
            }
        };
        if !child_lease_matches_request(&ticket.lease, &request) {
            return self.record_dispatch_failure(generation, request,
                ChildNavigationSubmitError::Rejected(
                    "navigation sink returned a mismatched child goal lease".to_string())).await;
        }

        let mut guard = self.inner.lock().await;
        let mut attached = false;
        {
            let mut decide = |admission_open: bool| {
                let frozen_request_matches = if resolved.is_some() {
                    guard.pending_child_dispatch.as_ref() == Some(&request)
                } else {
                    guard.pending_child_dispatch.as_ref().is_some_and(|pending| {
                        pending.request_id == request.request_id
                            && pending.parent_request_id == request.parent_request_id
                            && pending.intent == request.intent
                    })
                };
                if !admission_open || generation != guard.generation
                    || !guard.logical_active || !frozen_request_matches
                {
                    return;
                }
                guard.pending_child_dispatch = None;
                guard.selection_wait_since = None;
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
        let superseded = !attached && guard.session_in_progress();
        drop(guard);
        if attached {
            if let Some(resolved) = resolved.take() {
                // Consume the proof already returned by THIS command. Do not read
                // the bounded Nav receipt cache again or infer success from accepted.
                let result = resolved.result;
                match self.report_navigation_result(&ticket.lease.session_id, result.clone()).await {
                    Ok(snapshot) => return snapshot,
                    Err(error) => {
                        tracing::debug!(%error, "resolved child result lost business admission after attachment");
                        // Stop may win between attach and feedback. The original
                        // natural proof completes only its exact pending logical action.
                        return self.snapshot();
                    }
                }
            }
            return self.reconcile_child_terminal(&ticket.lease).await;
        }
        if resolved.is_some() || superseded { return self.snapshot(); }
        let _ = sink.cancel(&ticket.lease).await;
        self.snapshot()
    }

    async fn complete_resolved_dispatch_cancellation(
        &self,
        generation: u64,
        request: ChildNavigationRequest,
        resolved: ChildNavigationResolved,
        owner: Option<&dyn ChildNavigationDispatchOwner>,
    ) -> ExploreRuntimeSnapshot {
        let ChildNavigationResult::Cancelled(_) = &resolved.result else {
            unreachable!("only proved cancellation uses this branch");
        };
        let mut guard = self.inner.lock().await;
        let mut admission = None;
        {
            let mut decide = |admission_open: bool| {
                if !admission_open || generation != guard.generation || !guard.logical_active
                    || guard.pending_child_dispatch.as_ref() != Some(&request) { return; }
                guard.pending_child_dispatch = None;
                guard.no_frontier_since = None;
                // Reuse original Cancelled feedback inside the SAME original root
                // admission callback. No arrival, unreachable result or new Cancel.
                let feedback = ExplorationGoalFeedback::Cancelled;
                admission = Some(self.admit_feedback_locked(&mut guard,
                    FeedbackIdentity::Internal { generation, lineage: request.request_id.clone(),
                        goal: request.frontier_goal_id.clone(), feedback: feedback.clone() },
                    request.frontier_goal_id.clone(), feedback, None, false));
            };
            if let Some(owner) = owner { owner.with_dispatch_admission(&mut decide); }
            else { decide(true); }
        }
        let ticket = match admission {
            None => return self.snapshot(),
            Some(Ok(ticket)) => ticket,
            Some(Err(error)) => {
                drop(guard);
                return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await;
            }
        };
        drop(guard);
        let acknowledged = self.await_feedback(&ticket).await;
        let guard = self.inner.lock().await;
        if generation != guard.generation || !guard.logical_active { return self.snapshot(); }
        if let Err(error) = acknowledged {
            drop(guard);
            return self.finish_session(None, ExplorationStopReason::ConsecutiveFailuresExceeded, error).await;
        }
        self.store_snapshot(guard.snapshot(STATUS_PENDING,
            "child navigation cancelled before attachment; original exact ZERO proof consumed"))
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
        // Physical settlement happened before this call for installed-unresolved.
        // Stop/new generation/full-request replacement may discard only business feedback.
        if matches!(&error, ChildNavigationSubmitError::DispatchUnresolved(_))
            && guard.pending_child_dispatch.as_ref() != Some(&request) {
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
            ChildNavigationResult::Failed { proof, .. } => &proof.lease,
            ChildNavigationResult::Cancelled(proof) => &proof.lease,
        };
        if !active_child.lease.matches(received_lease) {
            return Err(
                "child navigation terminal lease does not match active ownership".to_string(),
            );
        }

        if let ChildNavigationResult::Failed { reason, .. } = &result {
            if reason.contains("LEASE_NOT_GRANTED") {
                let reason_code = if reason.contains("STALE_STOP_GENERATION") {
                    ExplorationStopReason::UserCancelled
                } else { ExplorationStopReason::ConsecutiveFailuresExceeded };
                let message = reason.clone();
                guard.active_child = None;
                let generation = self.next_exploration_generation(&mut guard);
                self.record_child_action_stop_locked(&mut guard, generation, reason_code);
                let (snapshot, readiness, frontiers) = self.prepare_no_child_stop_locked(&mut guard,
                    PendingChildAction { reason: reason_code, message });
                drop(guard);
                *self.readiness_cache.lock() = readiness;
                self.publish_frontiers(frontiers);
                return Ok(self.store_snapshot(snapshot));
            }
        }
        if matches!(&result, ChildNavigationResult::Cancelled(_)) {
            guard.active_child = None;
            let generation = self.next_exploration_generation(&mut guard);
            self.record_child_action_stop_locked(&mut guard, generation, ExplorationStopReason::UserCancelled);
            let (snapshot, readiness, frontiers) = self.prepare_no_child_stop_locked(&mut guard, PendingChildAction {
                reason: ExplorationStopReason::UserCancelled, message: "探索子导航已取消".into(),
            });
            drop(guard);
            *self.readiness_cache.lock() = readiness;
            self.publish_frontiers(frontiers);
            return Ok(self.store_snapshot(snapshot));
        }
        // 子目标终态已核对完整租约。
        // 机体重叠恢复失败与目标位置无关；换前沿无法修复，直接结束探索。
        if let ChildNavigationResult::Failed { reason, .. } = &result {
            if reason.contains(nuwax_nav_types_messages::BODY_OVERLAP_RECOVERY_FAILED) {
                let action=PendingChildAction {
                    reason:if reason.contains("escape proof computation exhausted") {
                        ExplorationStopReason::PlanningResourceLimited
                    } else {ExplorationStopReason::NoReachableFreeSpace},
                    message:format!("机体重叠恢复失败，原子导航已确认停车，探索结束：{reason}"),
                };
                guard.active_child=None;
                let generation=self.next_exploration_generation(&mut guard);
                self.record_child_action_stop_locked(&mut guard,generation,action.reason);
                let (snapshot,readiness,frontiers)=self.prepare_no_child_stop_locked(&mut guard,action);
                drop(guard);
                *self.readiness_cache.lock()=readiness;
                self.publish_frontiers(frontiers);
                return Ok(self.store_snapshot(snapshot));
            }
        }
        let frontier_goal_id = active_child.frontier_goal_id.clone();
        if guard.return_goal_id.as_deref()==Some(frontier_goal_id.as_str()) {
            guard.active_child=None;
            guard.no_frontier_since=None;
            let reason=guard.return_reason.unwrap_or(ExplorationStopReason::DistanceBudgetReached);
            match &result {
                ChildNavigationResult::Reached(_)=>{
                    let distance=guard.mission.as_ref().map_or(0.0,|p|p.travelled_m);
                    drop(guard);return Ok(Box::pin(self.finish_session(None,reason,format!("返航已到达，累计行程{distance:.3}米"))).await);
                }
                ChildNavigationResult::Failed{reason:failure,..}=>{
                    guard.return_failures=guard.return_failures.saturating_add(1);
                    if guard.return_failures>guard.active_config.as_ref().map_or(0,|c|c.max_navigation_retries){
                        let message=format!("返航失败且已停止：{failure}");drop(guard);
                        return Ok(Box::pin(self.finish_session(None,ExplorationStopReason::ConsecutiveFailuresExceeded,message)).await);
                    }
                    let snapshot=guard.snapshot(STATUS_ACTIVE,format!("返航已停止，重新核验路线：{failure}"));drop(guard);return Ok(self.store_snapshot(snapshot));
                }
                _=>return Err("返航收到没有取消事务的取消终态".into()),
            }
        }
        let input_interrupted=matches!(&result,ChildNavigationResult::Failed{reason,..} if child_failure_is_input_interruption(reason));
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
            ChildNavigationResult::Cancelled(_) => {
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
            ChildNavigationResult::Cancelled(_) => {
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

    #[cfg(test)]
    pub async fn ingest_costmap_snapshot(
        &self,
        snapshot: ExploreCostmapSnapshot,
    ) -> Result<bool, String> {
        self.ingest_costmap_with_identity(snapshot, None, 0, None).await
    }

    #[cfg(test)]
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
        let source_age = nuwax_nav_types_messages::source_time::data_age_ns_at(
            value.source_timestamp_ns as u64, now_unix_nanos())?;
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
            Some((now_monotonic_nanos(), ttl, value.source_timestamp_ns as u64))).await
    }

    async fn ingest_costmap_with_identity(
        &self,
        snapshot: ExploreCostmapSnapshot,
        committed_map: Option<schema::CommittedMapIdentity>,
        frame_generation: i64,
        observation: Option<(u64, Duration, u64)>,
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
        let source = observation.map(|value| value.2);
        let (received, ttl, _) = observation.unwrap_or_else(|| (now_monotonic_nanos(), DEFAULT_COSTMAP_RECEIPT_TTL, 0));
        if guard.costmap_input.as_deref() == Some(&snapshot)
            && guard.committed_map.as_ref() == committed_map.as_ref()
            && guard.costmap_frame_generation == frame_generation
        {
            // 新的输入接收事件可以续期，状态查询或缓存读取不会进入这里。
            guard.costmap_received_monotonic_ns = received;
            guard.costmap_receipt_ttl = ttl;
            guard.costmap_source_timestamp_ns = source;
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
        guard.costmap_source_timestamp_ns = source;

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

    /// 探索业务或已接纳的子目标派发仍在使用地图时，保持地图工作区。
    pub async fn blocks_mapping_mutation(&self) -> bool {
        let guard = self.inner.lock().await;
        // Stop may clear pending while its accepted dispatch is still running.
        // Pure pose/compute work under tick_gate is NOT a motion-safety blocker.
        guard.retained_child_dispatch
            || guard.logical_active
            || guard.active_child.is_some()

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

    #[cfg(test)]
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
        #[cfg(test)]
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
            #[cfg(test)] reason: reason.to_string(),
            backend_available,
            controller_active,
            source_count,
            frontier_source_ready,
            costmap_input_ready,
            #[cfg(test)] navigation_sink_ready,
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
            || initial.pending_child_dispatch.is_some() {
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
        if guard.logical_active || guard.active_child.is_some() || guard.pending_child_dispatch.is_some(){
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
            elapsed_ns:guard.progress_stopped_ns.unwrap_or_else(now_unix_nanos).saturating_sub(guard.started_at_ns),reached_goals:guard.reached_goals,mission:guard.mission.clone(),return_reason:guard.return_reason,
            sources:Vec::new()};
        drop(guard);
        self.await_compute_idle().await?;
        let mut guard = self.inner.lock().await;
        let unchanged = |guard: &ExploreRuntimeInner| !guard.logical_active && guard.generation == generation
            && guard.compute.history == history && guard.last_request_id == request_id
            && guard.active_child.is_none() && guard.pending_child_dispatch.is_none()
            && guard.committed_map.as_ref() == Some(&domain.0)
            && guard.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref()).map_or(guard.costmap_frame_generation, |ticket| ticket.map_frame_generation) == domain.1
            && guard.progress_origin.as_ref() == Some(&domain.2);
        if !unchanged(&guard) { return Err("探索停止代际或历史在保存前发生变化".into()); }
        let export = self.admit_lifecycle_locked(&mut guard, ComputeLifecycle::ExportCheckpoint)?;
        drop(guard);
        let output = self.await_lifecycle(&export).await?;
        let guard = self.inner.lock().await;
        if !unchanged(&guard) { return Err("探索停止代际或历史在保存时发生变化".into()); }
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
    if let (Some(max_coverage), Some(_)) =
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
        let period = if period.is_zero() {
            DEFAULT_SCHEDULER_PERIOD
        } else {
            period
        };
        let compute_runtime = runtime.clone();
        let worker = tokio::spawn(async move {
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
                                let generation = runtime.generation.load(Ordering::SeqCst);
                                let tick = runtime.tick_generation(generation, pose, Some(pose_provider.as_ref()));
                                tokio::pin!(tick);
                                let started = Instant::now();
                                loop {
                                    tokio::select! {
                                        _ = &mut tick => break,
                                        _ = interval.tick() => runtime.refresh_compute_status(started.elapsed()).await,
                                        changed = shutdown_rx.changed() => {
                                            if changed.is_err() || *shutdown_rx.borrow() { return; }
                                        }
                                    }
                                }
                            }
                            Err(error) => {
                                // 活动子导航的停车仍归执行端；无子目标的连续失鲜有界等待。
                                runtime.record_pose_wait(error).await;
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
    #[cfg(test)] pub reason: String,
    pub backend_available: bool,
    pub controller_active: bool,
    pub source_count: usize,
    pub frontier_source_ready: bool,
    pub costmap_input_ready: bool,
    #[cfg(test)] pub navigation_sink_ready: bool,
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

    #[derive(Default)]
    struct RecordingNavSink {
        submitted: StdMutex<Vec<ChildNavigationRequest>>,
        submit_errors: StdMutex<VecDeque<ChildNavigationSubmitError>>,
        cancel_snapshot: StdMutex<Option<Result<u64, String>>>,
        map_name_error: StdMutex<Option<String>>,
        cancelled: StdMutex<Vec<String>>,
        cancel_error: StdMutex<Option<String>>,
        lease_override: StdMutex<Option<ChildNavigationLease>>,
        receipt_reads: AtomicU64,
        receipt_entered: tokio::sync::Notify,
        receipt_release: StdMutex<Option<Arc<tokio::sync::Notify>>>,
        terminal_result: StdMutex<Option<ChildNavigationResult>>,
        next_goal_generation: AtomicU64,
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
        ) -> Result<(), String> {
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
            Ok(())
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
        let stopped = handle.stop("receipt-none-once").await;
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(stopped.child_task_id.is_empty());
    }

    #[tokio::test]
    async fn overlap_recovery_failure_finishes_exploration_without_dispatching_again() {
        for (cause,expected) in [("escape time or distance budget exhausted",ExplorationStopReason::NoReachableFreeSpace),
            ("escape proof computation exhausted",ExplorationStopReason::PlanningResourceLimited)] {
            let (handle,sink)=configured_handle();
            ingest_committed_costmap_fixture(&handle,frontier_costmap(1),1).await.unwrap();
            handle.start("overlap-stop","prefer_wavefront").await;
            let active=handle.tick((2.,2.)).await;
            assert!(!active.child_task_id.is_empty());
            let lease=active_child_lease(&handle).await;
            let reason=format!("PlannerFailed cause=LocalPlanner: {} {cause}",
                nuwax_nav_types_messages::BODY_OVERLAP_RECOVERY_FAILED);
            let mut stale=terminal_proof(&lease);
            stale.lease.goal_generation+=1;
            assert!(handle.report_navigation_result(&lease.session_id,
                ChildNavigationResult::Failed {proof:stale,reason:reason.clone()}).await.is_err());
            let stopped=handle.report_navigation_result(&lease.session_id,
                ChildNavigationResult::Failed {proof:terminal_proof(&lease),reason:reason.clone()}).await.unwrap();
            assert_eq!(stopped.status,STATUS_FAILED);
            assert_eq!(stopped.last_stop_reason,stop_reason_str(expected));
            assert!(stopped.message.contains(&reason));
            assert!(stopped.child_task_id.is_empty());
            for _ in 0..3 {
                assert_eq!(handle.tick((2.,2.)).await.status,STATUS_FAILED);
            }
            assert_eq!(sink.submitted.lock().unwrap().len(),1,
                "已停车的机体重叠失败不能靠不断更换前沿重试");
        }
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
    async fn input_pause_timeout_waits_for_fresh_input_without_failing_the_frontier() {
        // 导航运行时自a51326fd起，近期障碍源到期、来源撤销等输入中断保持租约等待新鲜输入，持续到原
        // 30秒路径等待期限后以PlannerFailed围栏结束。消息取自3.2实测终态；单纯等不到路径的超时作对照。
        let input_pause = "planner path timeout after 30000ms; fencing; 等待新鲜导航输入: \
            recent obstacle source deadline elapsed; PlannerFailed after verified fence";
        let path_only = "planner path timeout after 30000ms; fencing; PlannerFailed after verified fence";
        for (parent, reason, interrupted) in [("input-pause-timeout", input_pause, true),
            ("path-only-timeout", path_only, false)] {
            let (handle, _sink) = configured_handle();
            ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
            handle.start(parent, "prefer_wavefront").await;
            assert!(!handle.tick((2.0, 2.0)).await.child_task_id.is_empty());
            let lease = active_child_lease(&handle).await;
            let after = handle.report_navigation_result(&lease.session_id, ChildNavigationResult::Failed {
                proof: terminal_proof(&lease), reason: reason.into(),
            }).await.unwrap();
            assert!(after.child_task_id.is_empty());
            let history = receipt_test_source_history(&handle).await;
            if interrupted {
                assert_eq!(after.message, "child navigation input interrupted; waiting for fresh inputs");
                assert_eq!(history[1]["failed_goals"], serde_json::json!([]),
                    "输入中断不能计入前沿失败");
                assert_eq!(history[1]["blacklisted_goals"], serde_json::json!([]));
            } else {
                assert_eq!(after.message, "child navigation failed; retry/blacklist policy applied");
                assert_one_receipt_source_failure(&history);
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
        assert_eq!(handle.inner.lock().await.costmap_source_timestamp_ns, Some(value.source_timestamp_ns as u64));
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
        let sink = Arc::new(RecordingNavSink::default());
        handle.set_navigation_sink(sink.clone());
        (handle, sink)
    }

    fn surface_config(frame: &str) -> SurfaceNavigationConfig {
        SurfaceNavigationConfig::new(frame.into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.2)/2., -(0.2)/2.], [(0.2)/2., -(0.2)/2.], [(0.2)/2., (0.2)/2.], [-(0.2)/2., (0.2)/2.]], z_min_m: -(0.5), z_max_m: (0.6)-(0.5), source: "测试外形".into() }, 0.1, 0.15, 18.0, 0.2).unwrap()
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
        let config = surface_config(frame);
        handle
            .configure_surface_source(Ok(config.clone()), 37, [7; 16])
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
                config.geometry.clone(),
                message.source_voxel_size,
            )
            .unwrap().with_body_envelope(config.body_envelope.clone()).unwrap(),
        );
        assert!(
            handle
                .finish_surface_map_update(ticket.clone(), Ok(snapshot.clone()))
                .await
        );
        (message, ticket, snapshot)
    }

    #[tokio::test]
    async fn surface_goals_preserve_real_floor_height_arrival_heading_and_source() {
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
            // 无方向视场的自由朝向目标取PCT路线到达航向，只含偏航，不沿用起点横滚。
            let arrival = goal.pose.orientation;
            assert_eq!((arrival.x, arrival.y), (0.0, 0.0));
            assert!(((arrival.z * arrival.z + arrival.w * arrival.w).sqrt() - 1.0).abs() < 1e-12);
            assert_ne!(arrival, orientation);
            assert_eq!(
                requests[0].intent.goal.pose.position[2],
                goal.pose.position.z
            );
            assert_eq!(requests[0].intent.goal.header.frame_id, "map");
            assert_eq!(requests[0].intent.goal.pose.position,
                [goal.pose.position.x, goal.pose.position.y, goal.pose.position.z]);
            assert_eq!(requests[0].intent.goal.pose.orientation_xyzw,
                [arrival.x, arrival.y, arrival.z, arrival.w]);
            assert_eq!(requests[0].intent.map_frame_generation, 1);
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
        for ground_block_failed in [false, true] {
        let (handle, sink) = configured_handle();
        let (message, ticket, snapshot) = install_surface_fixture(&handle, "map").await;
        handle.start("surface-clear", "prefer_wavefront").await;
        let active = handle
            .tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY))
            .await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let mut clear = message.clone();
        clear.cleared = true;
        clear.ground_block_failed = ground_block_failed;
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
        assert_eq!(handle.snapshot().status, STATUS_FAILED);
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
        let stopped = handle.snapshot();
        assert_eq!(stopped.status, STATUS_FAILED);
        if ground_block_failed { assert!(stopped.message.contains("mapping_ground_block_write_failed")); }
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
    async fn surface_slow_build_uses_live_source_without_relabeling_installed_map() {
        let (handle, sink) = configured_handle();
        handle.configure_surface_source(Ok(surface_config("map")), 37, [7; 16])
            .await.unwrap();
        let first = surface_message("map", 1);
        // 模拟耗时超过接收期限的构建，后续发布继续由真实接纳入口更新。
        let old_receipt = now_monotonic_nanos()
            .saturating_sub((DEFAULT_COSTMAP_RECEIPT_TTL + Duration::from_secs(1)).as_nanos() as u64);
        let ticket = handle.begin_surface_map_update(&first, old_receipt).await.unwrap().unwrap();
        let snapshot = Arc::new(SurfaceSnapshot::from_points(
            &schema::navigation_map_3d_points(&first).unwrap(),
            surface_config("map").geometry, first.source_voxel_size,
        ).unwrap());
        let next = surface_message("map", 2);
        handle.begin_surface_map_update(&next, now_monotonic_nanos()).await.unwrap();
        assert_eq!(handle.start("surface-slow-build", "prefer_wavefront").await.status, STATUS_ACCEPTED);
        let waiting = handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(waiting.status, STATUS_PENDING, "{}", waiting.message);
        assert!(sink.submitted.lock().unwrap().is_empty(), "仅有新发布但没有完整图不能派发");
        assert!(handle.finish_surface_map_update(ticket, Ok(snapshot.clone())).await);
        let active = handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let requests = sink.submitted.lock().unwrap();
        assert_eq!(requests.len(), 1);
        assert_eq!(requests[0].intent.committed_map, first.committed_map);
        drop(requests);
        let guard = handle.inner.lock().await;
        assert_eq!(guard.committed_map.as_ref(), Some(&first.committed_map));
        assert_eq!(guard.costmap_received_monotonic_ns, old_receipt);
    }

    #[tokio::test]
    async fn surface_duplicate_publication_cannot_keep_stopped_source_alive() {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        handle.start("surface-stopped-source", "prefer_wavefront").await;
        {
            let mut guard = handle.inner.lock().await;
            // 来源数据超过原两秒期限；接收心跳、只读查询和重发均不能延长它。
            let old_source = now_unix_nanos().saturating_sub(
                (DEFAULT_COSTMAP_RECEIPT_TTL + Duration::from_secs(1)).as_nanos() as u64);
            guard.surface_mode.as_mut().unwrap().latest.as_mut().unwrap().source_timestamp_ns = old_source;
        }
        assert!(handle.begin_surface_map_update(&message, now_monotonic_nanos()).await.is_err());
        let waiting = handle.tick_with_pose(surface_pose(&snapshot, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(waiting.status, STATUS_PENDING);
        assert!(waiting.message.contains("surface map input stale"));
        assert!(sink.submitted.lock().unwrap().is_empty());
        assert!(handle.inner.lock().await.no_frontier_since.is_none());
    }

    #[tokio::test]
    async fn surface_age_follows_slow_source_and_link_liveness_remains_bounded() {
        const CHILD: &str = "NEH_TEST_SURFACE_DATA_CLOCK";
        let Some(path) = std::env::var_os(CHILD) else {
            let path = std::env::temp_dir().join(format!("neh-surface-data-clock-{}", std::process::id()));
            let status = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", "explore_runtime::tests::surface_age_follows_slow_source_and_link_liveness_remains_bounded"])
                .env(CHILD, &path).env("NUWAX_SOURCE_TIME_FILE", &path).status().unwrap();
            std::fs::remove_file(path).unwrap();
            assert!(status.success());
            return;
        };
        let (handle, _) = configured_handle();
        let (message, _, _) = install_surface_fixture(&handle, "map").await;
        let source = message.source_timestamp_ns as u64;
        let write_clock = |current: u64, live: bool| {
            let mono = if live { nuwax_motion_safety_guard::measured_motion::monotonic_ns() } else { 0 };
            let mut bytes = b"NEHTIME1".to_vec();
            for field in [1_u64, 1, current, 0, mono] { bytes.extend_from_slice(&field.to_le_bytes()); }
            std::fs::write(&path, bytes).unwrap();
        };
        let mut inner = handle.inner.lock().await;
        inner.surface_mode.as_mut().unwrap().latest.as_mut().unwrap().received_monotonic_ns = 1;
        write_clock(source + 1_050_000_000, true);
        assert!(inner.map_input_is_fresh(3_000_000_001), "0.35倍的三秒墙钟只产生1.05秒数据年龄");
        write_clock(source + 2_000_000_001, true);
        assert!(!inner.map_input_is_fresh(2), "新接收事件也不能刷新真实过期数据");
        write_clock(source + 1_050_000_000, true);
        assert!(!inner.map_input_is_fresh(5_000_000_002), "时钟仍活着时，断流输入也必须过期");
        write_clock(source + 1_050_000_000, false);
        assert!(!inner.map_input_is_fresh(2), "来源钟停止也必须被独立判活拒绝");
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

        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
            .await
            .unwrap();
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
        assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
        assert_eq!(handle.stop("req-refresh").await.status, STATUS_CANCELLED);
        let stopped = handle.snapshot();
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
    async fn new_exploration_during_submit_does_not_cancel_old_child() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("old-exploration", "prefer_wavefront").await;
        let release = Arc::new(tokio::sync::Notify::new());
        *sink.submit_release.lock().unwrap() = Some(release.clone());
        let ticking = tokio::spawn({ let handle=handle.clone(); async move { handle.tick((2.0,2.0)).await } });
        sink.submit_entered.notified().await;
        let replacement = handle.start("new-exploration", "prefer_wavefront").await;
        assert_eq!(replacement.status, STATUS_ACCEPTED);
        release.notify_one();
        ticking.await.unwrap();
        assert!(sink.cancelled.lock().unwrap().is_empty());
        assert_eq!(handle.snapshot().request_id, "new-exploration");
        assert!(handle.inner.lock().await.logical_active);
    }

    #[tokio::test]
    async fn stop_during_submit_cancels_late_ticket_once() {
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
        assert!(!handle.blocks_mapping_mutation().await);
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
        assert_eq!(handle.snapshot().status, STATUS_CANCELLED);
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

    include!("explore_surface_incomplete_tests.rs");
    include!("explore_compute_runtime_tests.rs");
    include!("explore_pose_recovery_tests.rs");
    include!("explore_compute_surface_tests.rs");
    include!("explore_continuous_map_tests.rs");
    include!("explore_composite_map_tests.rs");

    #[tokio::test]
    async fn compute_owner_surface_slow_prepare_accepts_live_unbuilt_publication() {
        let (handle, sink) = configured_handle();
        let (first, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request_id = "surface-live-slow-prepare";
        assert_eq!(handle.start(request_id, "prefer_wavefront").await.status, STATUS_ACCEPTED);
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let gate = Arc::new(OwnerRuntimeGate::default());
        let _release = OwnerRuntimeGateRelease(gate.clone());
        install_gated_real_surface_source(&handle, audit.clone(), gate.clone(), None).await;
        let tick = tokio::spawn({
            let handle = handle.clone();
            let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
            async move { handle.tick_with_pose(pose).await }
        });
        owner_watchdog(gate.entered()).await;
        tokio::time::sleep(DEFAULT_COSTMAP_RECEIPT_TTL + Duration::from_millis(20)).await;
        // 新地图仍在计算中；接收不能冒充安装，也不能使已完成前沿永远被退休。
        let next = surface_message("map", 2);
        handle.begin_surface_map_update(&next, now_monotonic_nanos()).await.unwrap();
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&first.committed_map));
            assert!(now_monotonic_nanos().saturating_sub(guard.costmap_received_monotonic_ns)
                > DEFAULT_COSTMAP_RECEIPT_TTL.as_nanos() as u64);
        }
        gate.release();
        let active = owner_watchdog(tick).await.unwrap();
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        {
            let audit = audit.lock().unwrap();
            assert_eq!((audit.captures.len(), audit.commits.len(), audit.discards), (1, 1, 0));
        }
        finish_surface_owner_success(&handle, request_id, &active, &audit).await;
    }

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

// Post-settlement business-fence control only. The original tick supplies its real
// frozen request. Direct delivery below does NOT claim a physical terminal proof;
// Bridge/root integration separately tests the strict settlement continuation.
#[tokio::test]
async fn unresolved_dispatch_feedback_requires_original_generation_and_full_frozen_request() {
    let mut case = SchemeARaceHarness::new("unresolved-full-request-fence");
    let observed = async {
        case.prepare().await?;
        let (generation, request) = case.blocked_rejection().await?;
        let lease = ChildNavigationLease {
            session_id: request.request_id.clone(), owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: request.parent_request_id.clone(), goal_generation: 1,
            producer_runtime_generation: 11,
        };
        let before = scheme_a_race_state(&case.handle).await;
        let mut observations = Vec::new();
        for mutation in 0..7 {
            let mut stale = request.clone();
            let received_generation = if mutation == 0 { generation + 1 } else { generation };
            match mutation {
                0 => {},
                1 => stale.expected_map_name.push_str("-other"),
                2 => stale.frontier_goal_id.push_str("-other"),
                3 => stale.goal.target.pose.position.x += 1.0,
                4 => stale.intent.goal.pose.position[0] += 1.0,
                5 => stale.motion_admission.motion_stop_generation += 1,
                _ => Arc::make_mut(&mut stale.boundary_owner).max_navigation_retries += 1,
            }
            let returned = case.handle.record_dispatch_failure(received_generation, stale,
                ChildNavigationSubmitError::DispatchUnresolved(ChildNavigationUnresolved {
                    lease: lease.clone(), reason: "already-settled submission business result".into(),
                })).await;
            observations.push((mutation, returned, scheme_a_race_state(&case.handle).await));
        }
        let mut identity_checks = Vec::new();
        identity_checks.push(child_lease_matches_request(&lease, &request));
        for mutation in 0..5 {
            let mut wrong = lease.clone();
            match mutation {
                0 => wrong.session_id.push_str("-foreign"),
                1 => wrong.parent_request_id.push_str("-foreign"),
                2 => wrong.owner_kind = schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                3 => wrong.goal_generation = 0,
                _ => wrong.producer_runtime_generation = 0,
            }
            identity_checks.push(child_lease_matches_request(&wrong, &request));
        }
        Ok::<_, String>((before, observations, identity_checks))
    }.await;
    let cleanup = case.cleanup().await;
    assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
    let (before, observations, identity_checks) = observed.expect("scenario failed");
    for (mutation, returned, after) in observations {
        assert_eq!(after, before, "stale settled failure mutated business state: case {mutation}");
        assert_eq!(returned.revision, before.snapshot_revision);
    }
    assert_eq!(identity_checks, vec![true, false, false, false, false, false]);
}

// Consumer contract only: original start/tick freezes each request, then the
// mock sink returns the current command's typed terminal carrier. The companion
// lcm_runtime controls establish each carrier from the actual typed source path.
#[tokio::test]
async fn resolved_dispatch_consumes_original_reached_failed_and_cancelled_without_cache_reread() {
    for (parent, terminal) in [
        ("resolved-dispatch-reached", "SUCCEEDED"),
        ("resolved-dispatch-failed", "FAILED"),
        ("resolved-dispatch-cancelled", "CANCELLED"),
    ] {
        let mut case = SchemeARaceHarness::new(parent);
        let observed = async {
            case.refresh().await?;
            let started = scheme_a_race_wait("start original resolved consumer",
                case.handle.start_with_config(parent, scheme_a_race_config())).await?;
            if started.status != STATUS_ACCEPTED { return Err(format!("start rejected: {started:#?}")); }
            let (_generation, request) = case.blocked_rejection().await?;
            let lease = ChildNavigationLease {
                session_id: request.request_id.clone(), parent_request_id: request.parent_request_id.clone(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                goal_generation: 1, producer_runtime_generation: 11,
            };
            let result = match terminal {
                "SUCCEEDED" => ChildNavigationResult::Reached(terminal_proof(&lease)),
                "FAILED" => ChildNavigationResult::Failed { proof: terminal_proof(&lease),
                    reason: "original measured terminal failure".into() },
                "CANCELLED" => ChildNavigationResult::Cancelled(zero_proof(&lease, 7)),
                _ => unreachable!(),
            };
            {
                let mut queued = case.sink.submit_errors.lock().unwrap();
                queued.clear();
                queued.push_back(ChildNavigationSubmitError::DispatchResolved(ChildNavigationResolved { result }));
            }
            let before = scheme_a_race_state(&case.handle).await;
            let returned = case.release_tick().await?;
            let after = scheme_a_race_state(&case.handle).await;
            let source = receipt_test_source_history(&case.handle).await;
            Ok::<_, String>((request, before, returned, after, source))
        }.await;
        let cleanup = case.cleanup().await;
        assert!(cleanup.is_ok(), "{cleanup:#?}; observations={observed:#?}");
        let (request, before, returned, after, source) = observed.expect("resolved consumer scenario failed");
        assert_eq!(after.reached_goals, before.reached_goals + u32::from(terminal == "SUCCEEDED"));
        assert_eq!(after.compute_history, before.compute_history + 1, "one original source feedback");
        assert!(after.pending.is_none());
        assert!(after.active_lease.is_none());
        assert!(returned.child_task_id.is_empty());
        assert!(returned.child_lease.is_none());
        assert_eq!(scheme_a_submitted_ids(&case), vec![scheme_a_request_ids(&request)]);
        match terminal {
            "SUCCEEDED" => assert!(returned.message.contains("reached; selecting next frontier")),
            "FAILED" => {
                assert!(returned.message.contains("failed; retry/blacklist policy applied"));
                assert_one_receipt_source_failure(&source);
            }
            "CANCELLED" => {
                assert!(returned.message.contains("cancelled before attachment"));
                assert!(source[1]["failed_goals"].as_array().unwrap().is_empty(),
                    "proved cancellation must not become an unreachable frontier");
            }
            _ => unreachable!(),
        }
        scheme_a_assert_no_lease_side_effects(&case, 1);
    }
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
    async fn new_exploration_replaces_old_business_without_cancelling_child() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("old-exploration", "prefer_wavefront").await;
        let old = handle.tick((2., 2.)).await;
        assert!(!old.child_task_id.is_empty());
        let next = handle.start("new-exploration", "prefer_wavefront").await;
        assert_eq!(next.status, STATUS_ACCEPTED, "{}", next.message);
        assert_eq!(next.request_id, "new-exploration");
        assert!(next.child_task_id.is_empty());
        assert!(sink.cancelled.lock().unwrap().is_empty());
        let active = handle.tick((2., 2.)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        assert_ne!(active.child_task_id, old.child_task_id);
        let requests = sink.submitted.lock().unwrap();
        assert_eq!(requests.len(), 2);
        assert_eq!(requests[1].parent_request_id, "new-exploration");
    }

    #[tokio::test]
    async fn cancelled_child_requires_exact_lease_but_no_zero_receipt() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("cancelled-child", "prefer_wavefront").await;
        let active = handle.tick((2., 2.)).await;
        let lease = active_child_lease(&handle).await;
        for field in 0..5 {
            let mut wrong = lease.clone();
            match field { 0 => wrong.session_id = "other".into(), 1 => wrong.owner_kind += 1,
                2 => wrong.parent_request_id = "other".into(), 3 => wrong.goal_generation += 1,
                _ => wrong.producer_runtime_generation += 1 }
            assert!(handle.report_navigation_result(&active.child_task_id, ChildNavigationResult::Cancelled(
                ChildNavigationZeroProof { lease: wrong, cancel_generation: 0, zero_ack_id: String::new(), zero_applied_monotonic_ns: 0 },
            )).await.is_err());
            assert_eq!(handle.snapshot().child_lease, Some(lease.clone()));
        }
        let stopped = handle.report_navigation_result(&active.child_task_id, ChildNavigationResult::Cancelled(
            ChildNavigationZeroProof { lease, cancel_generation: 0, zero_ack_id: String::new(), zero_applied_monotonic_ns: 0 },
        )).await.unwrap();
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(stopped.child_task_id.is_empty());
        assert!(sink.cancelled.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn lease_not_granted_ends_exploration_without_frontier_failure() {
        for stale in [false, true] {
            let (handle, _) = configured_handle();
            ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
            handle.start("lease-rejection", "prefer_wavefront").await;
            let active = handle.tick((2., 2.)).await;
            let lease = active_child_lease(&handle).await;
            let reason = if stale { "LEASE_NOT_GRANTED: STALE_STOP_GENERATION" } else { "LEASE_NOT_GRANTED: MOTION_SINK_NOT_READY" };
            let stopped = handle.report_navigation_result(&active.child_task_id, ChildNavigationResult::Failed {
                proof: terminal_proof(&lease), reason: reason.into(),
            }).await.unwrap();
            assert_eq!(stopped.status, if stale { STATUS_CANCELLED } else { STATUS_FAILED });
            assert!(stopped.message.contains(reason));
            assert!(stopped.child_task_id.is_empty());
            let history = receipt_test_source_history(&handle).await;
            assert_eq!(history[1]["failed_goals"], serde_json::json!([]));
            assert_eq!(history[1]["blacklisted_goals"], serde_json::json!([]));
        }
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
        assert_eq!(stopping.status, STATUS_ACTIVE, "{}", stopping.message);
        assert!(sink.cancelled.lock().unwrap().is_empty());
        assert_eq!(sink.submitted.lock().unwrap().len(), 2);
        assert_eq!(handle.inner.lock().await.return_reason, Some(ExplorationStopReason::DistanceBudgetReached));
    }

    #[tokio::test]
    async fn mission_return_dispatches_directly_and_user_cancel_only_cancels_return_goal() {
        let (handle, sink) = configured_handle();
        ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
        handle.start("mission-direct-return", "prefer_wavefront").await;
        let old = handle.tick((2., 2.)).await;
        let old_lease = active_child_lease(&handle).await;
        enable_test_mission(&handle, Some(2.)).await;
        { let mut guard = handle.inner.lock().await;
            let mut progress = MissionProgress::new(MissionPose { position: [2., 2., 0.], orientation_xyzw: [0., 0., 0., 1.] }).unwrap();
            progress.travelled_m = 2.1; guard.mission = Some(progress);
        }
        let returned = handle.tick((2., 2.)).await;
        assert_eq!(returned.status, STATUS_ACTIVE, "{}", returned.message);
        assert!(sink.cancelled.lock().unwrap().is_empty());
        let requests = sink.submitted.lock().unwrap().clone();
        assert_eq!(requests.len(), 2);
        assert!(requests[1].frontier_goal_id.contains(".return."));
        assert_ne!(old.child_task_id, returned.child_task_id);
        assert!(handle.report_navigation_result(&old.child_task_id, ChildNavigationResult::Reached(terminal_proof(&old_lease))).await.is_err());
        let stopped = handle.stop("mission-direct-return").await;
        assert_eq!(stopped.status, STATUS_CANCELLED);
        assert!(stopped.child_task_id.is_empty());
        assert_eq!(*sink.cancelled.lock().unwrap(), vec![returned.child_task_id]);
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
        ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2)
            .await
            .unwrap();
        assert_eq!(handle.snapshot().status, STATUS_ACTIVE);

        let stopping = handle.stop("req-stop-replan").await;
        assert_eq!(stopping.status, STATUS_CANCELLED);
        assert_eq!(stopping.last_stop_reason, "user_cancelled");
        let terminal = handle.snapshot();
        assert_eq!(terminal.status, STATUS_CANCELLED);
        assert!(!handle.is_active().await);
    }

    #[test]
    fn wavefront_adapter_source_name_is_wavefront() {
        let adapter =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
        assert_eq!(adapter.source_name(), Cow::Borrowed("wavefront"));
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
        config.apply_calibration(nuwax_nav_types_messages::RobotGeometry {
            self_collision: None,
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_min_m: -0.5, z_max_m: 0.2, source: "surface return body fixture".into(),
        }, None).unwrap();
        handle.configure_surface_source(Ok(config.clone()), 37, [7;16]).await.unwrap();
        let message = surface_message("map", 1);
        let ticket = handle.begin_surface_map_update(&message, now_monotonic_nanos()).await.unwrap().unwrap();
        let snapshot = Arc::new(SurfaceSnapshot::from_points(
            &schema::navigation_map_3d_points(&message).unwrap(), config.geometry.clone(), message.source_voxel_size,
        ).unwrap().with_body_envelope(config.body_envelope.clone()).unwrap());
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
    }

}
