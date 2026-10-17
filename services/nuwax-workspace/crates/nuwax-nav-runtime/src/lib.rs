//! Guard-backed, single-owner navigation scheduler core.
//!
//! This crate deliberately stops at the process-neutral scheduling boundary.
//! It owns one mutable motion port, serializes lease renewal and commands, and
//! accepts only lease-correlated paths. Map sequence fields may still be
//! carried by compatibility transports, but they do not participate in motion.

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::future::Future;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nuwax_motion_safety_guard::wire::{LeaseTuple, TwistCommand};
use nuwax_motion_safety_guard::{
    ClientConfig, MotionAdmissionProof, MotionGuardClient, MotionGuardClientError, MAX_FRAME_BYTES,
};
use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerConfig, StopMessage};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{GoalKey, Path2D, Path3D, PoseStamped, Twist2};
use thiserror::Error;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;
use tokio::time::{interval_at, Instant, MissedTickBehavior};

mod arrival_confirmation;
use arrival_confirmation::ArrivalConfirmation;
mod command_dispatch;
pub use command_dispatch::CommandDispatchBound;
/// Explicit elapsed-time observer seam for bounded recovery qualifications.
#[cfg(feature = "surface-recovery-qualification")]
pub mod qualification_timing;
mod planner_worker;
pub mod surface_follower;
pub use nuwax_nav_mls_planner::SurfaceSnapshot;
pub use surface_follower::{SurfaceFollower, SurfaceFollowerConfig};
/// Shared ON/OFF source-change proofs.
pub mod surface_changes;
mod route_follower;
mod planning_owner;
mod route_planning;
/// Explicit, bounded real-input capture for ignored cross-crate qualifications.
#[cfg(feature="local-guidance-qualification")]
pub mod local_guidance_qualification;
#[cfg(test)]
mod route_moving_worker_tests;
#[cfg(test)]
mod control_profile;
#[cfg(test)]
mod command_proof_trace;
pub use route_follower::RouteFollower;

/// 局部执行保留路径维度，三维路径不能通过默认适配静默丢弃高度。
#[derive(Debug, Clone, PartialEq)]
pub enum ExecutionPath {
    /// 现有二维路径，配套朝向沿原可选字段传递。
    Planar(Path2D),
    /// 完整机体参考位置与姿态路径。
    Surface(Path3D),
}

// Resend timestamps do not change the route actually consumed by a follower.
// Preserve all consumed coordinates, orientations, frames and dimensionality.
fn same_execution_path(left: &ExecutionPath, right: &ExecutionPath) -> bool {
    match (left, right) {
        (ExecutionPath::Planar(left), ExecutionPath::Planar(right)) =>
            left.frame_id == right.frame_id && left.points == right.points,
        (ExecutionPath::Surface(left), ExecutionPath::Surface(right)) =>
            left.poses.len() == right.poses.len() && left.poses.iter().zip(&right.poses)
                .all(|(left, right)| left.frame_id == right.frame_id && left.pose == right.pose),
        _ => false,
    }
}

/// 当前局部执行使用的不可变地图输入。
#[derive(Clone)]
pub enum ExecutionMap {
    /// 现有二维占据图。
    Planar(CostMap2D),
    /// 由同一MLS拥有方生成的不可变表面快照。
    Surface(Arc<SurfaceSnapshot>),
    /// Immutable surface plus complete differences from its explicit baseline.
    SurfaceUpdate(surface_changes::SurfaceMapUpdate),
}

impl std::fmt::Debug for ExecutionMap {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Planar(map) => formatter.debug_tuple("Planar").field(map).finish(),
            Self::Surface(_) | Self::SurfaceUpdate(_) => formatter.write_str("Surface(..)"),
        }
    }
}

/// Production controller cadence: 20 Hz.
pub const DEFAULT_TICK_INTERVAL: Duration = Duration::from_millis(50);
/// Maximum scheduler-owned lease renewal interval.
pub const MAX_RENEW_INTERVAL: Duration = Duration::from_millis(250);
/// Qualified upper bound for one production NAV guard exchange.
pub const MAX_NAV_REQUEST_TIMEOUT: Duration = Duration::from_millis(250);
/// 零表示不按固定接收年龄终止导航，来源失效仍由拥有方显式撤销。
pub const DEFAULT_POSE_TTL: Duration = Duration::ZERO;
/// 地图在同一来源内持续有效，直到更新、清空或来源失效。
pub const DEFAULT_COSTMAP_TTL: Duration = Duration::ZERO;
/// 默认连续等待路径或可执行运动的期限；成功执行运动后重新计时。
pub const DEFAULT_PATH_WAIT_TIMEOUT: Duration = Duration::from_secs(30);
/// 局部运动预测长度，与输入过期门槛独立。
pub const DEFAULT_CONTROL_HORIZON: Duration = Duration::from_millis(250);
/// Hard upper bound from cancel admission to a terminal fence attempt.
pub const MAX_CANCEL_FENCE_TIMEOUT: Duration = Duration::from_millis(300);

const PRIORITY_FENCE_ATTEMPT_TIMEOUT: Duration = Duration::from_millis(75);
const PRIORITY_FENCE_ATTEMPTS: usize = 2;

const INPUT_CAPACITY: usize = 32;
const URGENT_CAPACITY: usize = 8;

/// Build the fail-closed client configuration used by the future Geometry owner.
///
/// The generic guard client keeps a broader diagnostic default. Navigation uses
/// this tighter timeout so a delayed request cannot consume the guard's entire
/// early-fence budget.
#[must_use]
pub fn production_client_config(socket_path: impl Into<PathBuf>) -> ClientConfig {
    ClientConfig {
        socket_path: socket_path.into(),
        request_timeout: MAX_NAV_REQUEST_TIMEOUT,
        max_frame_bytes: MAX_FRAME_BYTES,
    }
}

/// Immutable motion admission carried by one accepted navigation request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavMotionAdmission {
    /// Durable guard generation observed by the request admission layer.
    pub guard_generation: u64,
    /// Sixteen-byte boot nonce observed with `guard_generation`.
    pub guard_boot_nonce: [u8; 16],
    /// Global motion-stop generation observed by the same admission check.
    pub motion_stop_generation: u64,
}

/// Complete in-process execution identity correlated from sealed goal and path envelopes.
///
/// This type is deliberately not a wire schema. Goal admission initializes the
/// goal-level fields and leaves the planner/path fields at their neutral values;
/// an accepted path carries the same goal-level fields plus its sealed planner
/// generation and revision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PathExecutionIdentity {
    /// Stable goal/session identifier.
    pub session_id: String,
    /// Lease owner kind.
    pub owner_kind: i32,
    /// Public request that owns the lease.
    pub parent_request_id: String,
    /// Monotonic goal generation.
    pub goal_generation: i64,
    /// Stable goal producer name.
    pub producer: String,
    /// Goal producer epoch.
    pub producer_epoch: i64,
    /// Runtime generation that sealed and owns the goal.
    pub producer_runtime_generation: i64,
    /// Committed mapping session identifier.
    pub committed_map_session_id: String,
    /// Committed mapping-session generation.
    pub committed_map_session_generation: i64,
    /// Committed map revision.
    pub committed_map_revision: i64,
    /// Opaque occupancy commit process domain.
    pub committed_map_boot_nonce: [u8; 16],
    /// Occupancy commit sequence.
    pub committed_map_commit_sequence: i64,
    /// Original occupancy commit timestamp.
    pub committed_map_source_timestamp_ns: i64,
    /// Authoritative committed map frame.
    pub committed_map_frame_id: String,
    /// Navigation boundary identifier.
    pub boundary_id: String,

    /// Admitted Motion Guard generation.
    pub motion_guard_generation: i64,
    /// Admitted Motion Guard boot nonce.
    pub motion_guard_boot_nonce: [u8; 16],
    /// Admitted global stop generation.
    pub motion_stop_generation: i64,
    /// Planner runtime generation that produced the path; zero before path admission.
    pub planner_runtime_generation: i64,
    /// Monotonic path revision; zero before path admission.
    pub path_revision: u64,
}

impl PathExecutionIdentity {
    /// Original Geometry-assigned goal identity.
    pub fn goal_key(&self) -> GoalKey {
        GoalKey { producer_runtime_generation: self.producer_runtime_generation as u64, goal_generation: self.goal_generation as u64 }
    }

    fn has_same_goal_identity(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
            && self.producer == other.producer
            && self.producer_epoch == other.producer_epoch
            && self.producer_runtime_generation == other.producer_runtime_generation
            && self.committed_map_session_id == other.committed_map_session_id
            && self.committed_map_session_generation == other.committed_map_session_generation
            && self.committed_map_revision == other.committed_map_revision
            && self.committed_map_boot_nonce == other.committed_map_boot_nonce
            && self.committed_map_commit_sequence == other.committed_map_commit_sequence
            && self.committed_map_source_timestamp_ns == other.committed_map_source_timestamp_ns
            && self.committed_map_frame_id == other.committed_map_frame_id
            && self.boundary_id == other.boundary_id
            && self.motion_guard_generation == other.motion_guard_generation
            && self.motion_guard_boot_nonce == other.motion_guard_boot_nonce
            && self.motion_stop_generation == other.motion_stop_generation
    }
}

/// One navigation session and its request-level motion admission.
#[derive(Debug, Clone, PartialEq)]
pub struct NavSession {
    /// 原任务的平面边界，局部重规划沿用同一范围。
    pub boundary: Vec<[f64; 2]>,
    /// 随封存目标核对的倒车授权与到达容差。
    pub options: nuwax_nav_types_messages::GoalOptions,
    /// Stable goal/session identifier.
    pub session_id: String,
    /// Complete sealed goal identity expected on every admitted path.
    pub execution_identity: PathExecutionIdentity,
    /// Runtime generation of the unique local-execution owner.
    pub local_owner_runtime_generation: u64,
    /// Request-level motion admission proof transported with the goal.
    pub motion_admission: NavMotionAdmission,
    /// 已封存的用户原目标；三维目标使用机体参考位置。
    pub goal_pose: PoseStamped,
}

/// A planner path with the correlation fields required for safe cutover.
#[derive(Debug, Clone, PartialEq)]
pub struct VersionedPath {
    /// Goal/session identifier copied from the versioned goal envelope.
    pub session_id: String,
    /// Complete identity copied from the sealed goal and path envelopes.
    pub execution_identity: PathExecutionIdentity,
    /// Monotonic path revision within the active navigation session.
    pub path_revision: u64,

    /// Planner output consumed by the local controller.
    pub path: ExecutionPath,
    /// 可选路径朝向；生产入口由已核对身份的目标与路径生成，末项为请求的到达朝向。
    pub path_yaws: Option<Vec<f64>>,
}

/// Scheduler lifecycle. Terminal meaning is carried separately in status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerPhase {
    /// No lease and no active session.
    Idle,
    /// Waiting for the motion guard to grant a NAV lease.
    Acquiring,
    /// 租约已取得，等待首次有效路径或已确认零速后的更新路径。
    AwaitingPath,
    /// Running the 20 Hz controller and waiting for every APPLIED proof.
    Controlling,
    /// Cancel accepted; no more twist is issued while fence proof is pending.
    Cancelling,
    /// 非取消收尾正在等待停车证明，或复核停车后的目标位置。
    Fencing,
    /// Fail-closed state. No twist or renewal is issued.
    Failed,
}

/// Verified terminal outcome from the most recent session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalOutcome {
    /// Cancel completed after a valid resource fence.
    Cancelled,
    /// 规划到达、租约已围栏，停车后采样仍满足原目标。
    Succeeded,
    /// Planner failed and the lease was fenced.
    PlannerFailed,
    /// A versioned-path protocol violation was fenced.
    SafetyFailed,
    /// Scheduler shutdown fenced the active lease.
    Shutdown,
}

/// Observable state published by the scheduler task.
#[derive(Debug, Clone, PartialEq)]
pub struct SchedulerStatus {
    /// Current scheduler phase.
    pub phase: SchedulerPhase,
    /// Active session, if any.
    pub session_id: Option<String>,
    /// Latest accepted path revision.
    pub path_revision: u64,
    /// Commands whose correlated APPLIED proof was validated.
    pub applied_commands: u64,
    /// Last controller command with a correlated APPLIED proof.
    pub last_applied_twist: Option<Twist2>,
    /// Whether the scheduler's motion transport is currently usable.
    pub motion_sink_ready: bool,
    /// Guard process generation from the active or most recent acquire exchange.
    pub guard_generation: u64,
    /// Whether both planner inputs are within their receipt TTL.
    pub planner_inputs_fresh: bool,
    /// Source timestamp from the most recently consumed map pose.
    pub pose_source_timestamp_ns: i64,
    /// Source timestamp from the most recently consumed occupancy snapshot.
    pub costmap_source_timestamp_ns: i64,
    /// Monotonic receipt age of the latest map pose.
    pub pose_receipt_age_ms: u64,
    /// Monotonic receipt age of the latest occupancy snapshot.
    pub costmap_receipt_age_ms: u64,
    /// Most recent verified terminal outcome.
    pub last_terminal: Option<TerminalOutcome>,
    /// Most recent verified guard ZeroAck identifier.
    pub last_zero_ack_id: Option<String>,
    /// Monotonic timestamp carried by the most recent verified fence.
    pub last_fence_monotonic_ns: u64,
    /// Diagnostic message for operators and tests.
    pub message: String,
    /// 当前目标的局部规划轨迹，仅用于显示。
    pub local_path: Vec<[f64; 3]>,
}

impl Default for SchedulerStatus {
    fn default() -> Self {
        Self {
            phase: SchedulerPhase::Idle,
            session_id: None,
            path_revision: 0,
            applied_commands: 0,
            last_applied_twist: None,
            motion_sink_ready: false,
            guard_generation: 0,
            planner_inputs_fresh: false,
            pose_source_timestamp_ns: 0,
            costmap_source_timestamp_ns: 0,
            pose_receipt_age_ms: u64::MAX,
            costmap_receipt_age_ms: u64::MAX,
            last_terminal: None,
            last_zero_ack_id: None,
            last_fence_monotonic_ns: 0,
            message: "idle".into(),
            local_path: Vec::new(),
        }
    }
}

/// Scheduler configuration independent of transport wiring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SchedulerConfig {
    /// Controller tick interval.
    pub tick_interval: Duration,
    /// Idle/awaiting-path lease renewal interval.
    pub renew_interval: Duration,
    /// 等待路径期限；零表示等待至路径就绪或取消。
    pub path_wait_timeout: Duration,
    /// Collision footprint width passed to the local planner.
    pub robot_width_m: f64,
    /// 位姿接收期限；零表示不以固定接收年龄终止。
    pub pose_ttl: Duration,
    /// 地图接收期限；零表示保持至来源显式失效。
    pub costmap_ttl: Duration,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            tick_interval: DEFAULT_TICK_INTERVAL,
            renew_interval: MAX_RENEW_INTERVAL,
            path_wait_timeout: DEFAULT_PATH_WAIT_TIMEOUT,
            robot_width_m: 0.3,
            pose_ttl: DEFAULT_POSE_TTL,
            costmap_ttl: DEFAULT_COSTMAP_TTL,
        }
    }
}

/// Typed scheduler errors.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum RuntimeError {
    /// Caller or configuration input is invalid.
    #[error("invalid navigation input `{field}`: {reason}")]
    InvalidInput {
        /// Invalid field.
        field: &'static str,
        /// Validation reason.
        reason: String,
    },
    /// Another session still owns or may own the motion resource.
    #[error("navigation scheduler is busy: {0}")]
    Busy(String),
    /// A stale or cross-session path was rejected without replacing the active path.
    #[error("versioned path rejected: {reason}")]
    PathRejected {
        /// Correlation failure.
        reason: String,
    },
    /// One immutable path revision arrived with different bytes.
    #[error("versioned path mutation detected: {reason}")]
    PathMutation {
        /// Immutability violation.
        reason: String,
    },
    /// Motion port failure. The result may be ambiguous.
    #[error("motion guard operation failed: {0}")]
    Motion(String),
    /// 停止命令已隔离租约，原操作尚未取得物理停稳确认，可继续查询。
    #[error("navigation stop confirmation pending: {0}")]
    StopUnconfirmed(String),
    /// Planner reported a terminal failure.
    #[error("planner failed: {0}")]
    Planner(String),
    /// A controller input exceeded its receipt TTL or replayed an old source timestamp.
    #[error("stale navigation input: {0}")]
    StaleInput(String),
    /// APPLIED proof did not correlate to the submitted twist.
    #[error("invalid APPLIED proof: {0}")]
    InvalidApplied(String),
    /// Fence proof was incomplete or not IDLE.
    #[error("invalid fence proof: {0}")]
    InvalidFence(String),
    /// The scheduler task is no longer accepting requests.
    #[error("navigation scheduler task is stopped")]
    SchedulerStopped,
    /// Operation-id sequence exhausted.
    #[error("navigation operation sequence exhausted")]
    SequenceExhausted,
}

/// Motion-port error before scheduler policy is applied.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
#[error("{message}")]
pub struct MotionPortError {
    /// Stable diagnostic message.
    pub message: String,
    /// Whether an ambiguous result may have created or retained a lease.
    pub lease_may_be_active: bool,
    /// 已有精确停止票据，允许保留会话等待后续确认。
    pub stop_unconfirmed: bool,
}

impl MotionPortError {
    /// Construct a motion-port error.
    #[must_use]
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            lease_may_be_active: true,
            stop_unconfirmed: false,
        }
    }

    /// Construct an error known to have occurred before lease acquisition.
    #[must_use]
    pub fn definite_no_lease(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            lease_may_be_active: false,
            stop_unconfirmed: false,
        }
    }

    fn stop_unconfirmed(message: impl Into<String>) -> Self {
        Self { message: message.into(), lease_may_be_active: true, stop_unconfirmed: true }
    }
}

impl From<MotionGuardClientError> for MotionPortError {
    fn from(error: MotionGuardClientError) -> Self {
        Self::new(error.to_string())
    }
}

/// Proof that a source-scoped lease was acquired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeaseProof {
    /// Non-zero guard lease identifier.
    pub lease_id: u64,
    /// Guard generation that issued the lease.
    pub guard_generation: u64,
}

/// One planner-consumed occupancy snapshot with immutable source metadata.
#[derive(Debug, Clone)]
pub struct CostmapObservation {
    /// 保留原维度的局部执行地图；三维表面只共享不可变快照。
    pub costmap: ExecutionMap,
    /// Positive source timestamp carried by the occupancy frame.
    pub source_timestamp_ns: i64,
}

/// Correlated proof that one twist reached APPLIED.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppliedProof {
    /// Operation identifier echoed by the port contract.
    pub operation_id: String,
    /// Guard-side monotonic application timestamp.
    pub applied_monotonic_ns: u64,
}

/// Lease state carried by a fence proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceLeaseState {
    /// No active owner remains.
    Idle,
    /// A source still owns the resource.
    Active,
    /// Revocation has not reached a terminal state.
    Revoking,
}

/// Proof required before publishing a navigation terminal state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FenceProof {
    /// Non-empty guard-generated acknowledgement identifier.
    pub zero_ack_id: String,
    /// Guard-side monotonic zero application timestamp.
    pub applied_monotonic_ns: u64,
    /// Terminal lease state.
    pub lease: FenceLeaseState,
}

/// Result of bounded receiving admission; expiry never claims the lease is gone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundedTwistOutcome {
    /// The exact request was acknowledged as applied.
    Applied(AppliedProof),
    /// No new command was applied because its observation issue bound expired.
    ExpiredNotApplied,
}

/// Mutable motion boundary owned by exactly one scheduler task.
#[async_trait]
pub trait NavMotionPort: Send + 'static {
    /// 服务端截止推导出的提前续租时刻；模拟或外部实现可使用配置周期。
    fn renewal_due_at(&self) -> Option<Instant> { None }
    /// Acquire the NAV lease directly from the request's admission proof.
    async fn acquire(
        &mut self,
        operation_id: &str,
        admission: &NavMotionAdmission,
    ) -> Result<LeaseProof, MotionPortError>;
    /// Renew the active lease.
    async fn renew(&mut self, operation_id: &str) -> Result<(), MotionPortError>;
    /// Submit one twist and return only after APPLIED.
    async fn submit_twist(
        &mut self,
        operation_id: &str,
        twist: Twist2,
    ) -> Result<AppliedProof, MotionPortError>;
    /// Submit with the original observation issue bound enforced at receiving admission.
    /// Legacy ports must explicitly implement finite bounds rather than ignoring them.
    async fn submit_twist_bounded(
        &mut self, operation_id: &str, twist: Twist2, bound: Option<CommandDispatchBound>,
    ) -> Result<BoundedTwistOutcome, MotionPortError> {
        if bound.is_some() {
            return Err(MotionPortError::new("motion port does not support observation dispatch deadlines"));
        }
        self.submit_twist(operation_id,twist).await.map(BoundedTwistOutcome::Applied)
    }
    /// Revoke the lease, apply zero, and return terminal fence proof.
    async fn revoke_and_zero(&mut self, operation_id: &str) -> Result<FenceProof, MotionPortError>;
    /// Preempt an ordinary exchange and fence over an independent priority path.
    async fn priority_revoke_and_zero(
        &mut self,
        operation_id: &str,
    ) -> Result<FenceProof, MotionPortError> {
        self.revoke_and_zero(operation_id).await
    }
}

#[async_trait]
impl NavMotionPort for MotionGuardClient {
    fn renewal_due_at(&self) -> Option<Instant> { self.renewal_due_at() }
    async fn acquire(
        &mut self,
        operation_id: &str,
        admission: &NavMotionAdmission,
    ) -> Result<LeaseProof, MotionPortError> {
        let proof = MotionAdmissionProof {
            generation: admission.guard_generation,
            boot_nonce: admission.guard_boot_nonce,
            motion_stop_generation: admission.motion_stop_generation,
        };
        let metadata = match self.acquire_lease(operation_id, &proof).await {
            Ok(metadata) => metadata,
            Err(error) => {
                let definite_no_lease = matches!(
                    &error,
                    MotionGuardClientError::InvalidInput(_) | MotionGuardClientError::Remote { .. }
                );
                let message = error.to_string();
                return Err(if definite_no_lease {
                    MotionPortError::definite_no_lease(message)
                } else {
                    MotionPortError::new(message)
                });
            }
        };
        Ok(LeaseProof {
            lease_id: metadata.lease_tuple.lease_id,
            guard_generation: metadata.lease_tuple.generation,
        })
    }

    async fn renew(&mut self, operation_id: &str) -> Result<(), MotionPortError> {
        self.renew_lease(operation_id).await?;
        Ok(())
    }

    async fn submit_twist(
        &mut self,
        operation_id: &str,
        twist: Twist2,
    ) -> Result<AppliedProof, MotionPortError> {
        let confirmation = self
            .submit_twist(
                operation_id,
                TwistCommand {
                    linear_x_mps: twist.linear.x,
                    linear_y_mps: twist.linear.y,
                    angular_z_rps: twist.angular_z,
                },
            )
            .await?;
        Ok(AppliedProof {
            operation_id: confirmation.operation_id,
            applied_monotonic_ns: confirmation.applied_monotonic_ns,
        })
    }

    async fn submit_twist_bounded(
        &mut self, operation_id:&str, twist:Twist2, bound:Option<CommandDispatchBound>,
    )->Result<BoundedTwistOutcome,MotionPortError> {
        let Some(bound)=bound else {
            return <Self as NavMotionPort>::submit_twist(self,operation_id,twist).await.map(BoundedTwistOutcome::Applied);
        };
        if bound.expired_now() {return Ok(BoundedTwistOutcome::ExpiredNotApplied);}
        let Some(deadline)=bound.monotonic_deadline() else {return Ok(BoundedTwistOutcome::ExpiredNotApplied);};
        let result=self.submit_twist_before(operation_id,TwistCommand {
            linear_x_mps:twist.linear.x,linear_y_mps:twist.linear.y,angular_z_rps:twist.angular_z,
        },deadline).await;
        match result {
            Ok(proof)=>Ok(BoundedTwistOutcome::Applied(AppliedProof {
                operation_id:proof.operation_id,applied_monotonic_ns:proof.applied_monotonic_ns,
            })),
            Err(MotionGuardClientError::DispatchExpired{..})=>Ok(BoundedTwistOutcome::ExpiredNotApplied),
            Err(MotionGuardClientError::Remote{code,..}) if code=="DEADLINE_EXCEEDED"=>Ok(BoundedTwistOutcome::ExpiredNotApplied),
            Err(error)=>Err(error.into()),
        }
    }

    async fn revoke_and_zero(&mut self, operation_id: &str) -> Result<FenceProof, MotionPortError> {
        let confirmation = self.revoke_and_zero(operation_id).await?;
        Ok(FenceProof {
            zero_ack_id: confirmation.zero_ack_id,
            applied_monotonic_ns: confirmation.applied_monotonic_ns,
            lease: FenceLeaseState::Idle,
        })
    }
}

struct ProductionNavMotionPort {
    normal: Option<MotionGuardClient>,
    normal_config: ClientConfig,
    priority_config: ClientConfig,
    lease_tuple: Option<LeaseTuple>,
    pending_fence: Option<(String, nuwax_motion_safety_guard::ZeroConfirmation)>,
}

impl ProductionNavMotionPort {
    async fn connect(config: ClientConfig) -> Result<Self, MotionPortError> {
        let normal = MotionGuardClient::connect_with_config(config.clone())
            .await
            .map_err(|error| MotionPortError::definite_no_lease(error.to_string()))?;
        let mut priority_config = config.clone();
        priority_config.request_timeout = PRIORITY_FENCE_ATTEMPT_TIMEOUT;
        Ok(Self {
            normal: Some(normal),
            normal_config: config,
            priority_config,
            lease_tuple: None,
            pending_fence: None,
        })
    }

    async fn normal_client(&mut self) -> Result<&mut MotionGuardClient, MotionPortError> {
        if self.normal.is_none() {
            let client = MotionGuardClient::connect_with_config(self.normal_config.clone())
                .await
                .map_err(|error| MotionPortError::definite_no_lease(error.to_string()))?;
            self.normal = Some(client);
        }
        Ok(self.normal.as_mut().expect("normal client was restored"))
    }

    fn fence_proof(confirmation: nuwax_motion_safety_guard::ZeroConfirmation) -> FenceProof {
        FenceProof {
            zero_ack_id: confirmation.zero_ack_id,
            applied_monotonic_ns: confirmation.applied_monotonic_ns,
            lease: FenceLeaseState::Idle,
        }
    }

    async fn confirm_pending_fence(&mut self) -> Result<FenceProof, MotionPortError> {
        let (operation, zero) = self.pending_fence.as_ref().expect("已有停车票据").clone();
        let confirmed = MotionGuardClient::confirm_stationary_after_fence(
            self.normal_config.clone(), &operation, zero).await.map_err(|error| {
                if matches!(&error, MotionGuardClientError::Remote { code, .. } if code == "MOTION_STOP_UNCONFIRMED") {
                    MotionPortError::stop_unconfirmed(error.to_string())
                } else { MotionPortError::new(error.to_string()) }
            })?;
        self.pending_fence = None;
        self.lease_tuple = None;
        Ok(Self::fence_proof(confirmed))
    }
}

#[async_trait]
impl NavMotionPort for ProductionNavMotionPort {
    fn renewal_due_at(&self) -> Option<Instant> {
        self.normal.as_ref().and_then(MotionGuardClient::renewal_due_at)
    }
    async fn acquire(
        &mut self,
        operation_id: &str,
        admission: &NavMotionAdmission,
    ) -> Result<LeaseProof, MotionPortError> {
        let client = self.normal_client().await?;
        let proof =
            <MotionGuardClient as NavMotionPort>::acquire(client, operation_id, admission).await?;
        self.lease_tuple = client.active_lease().cloned();
        Ok(proof)
    }

    async fn renew(&mut self, operation_id: &str) -> Result<(), MotionPortError> {
        <MotionGuardClient as NavMotionPort>::renew(self.normal_client().await?, operation_id).await
    }

    async fn submit_twist(
        &mut self,
        operation_id: &str,
        twist: Twist2,
    ) -> Result<AppliedProof, MotionPortError> {
        <MotionGuardClient as NavMotionPort>::submit_twist(
            self.normal_client().await?,
            operation_id,
            twist,
        )
        .await
    }

    async fn submit_twist_bounded(
        &mut self, operation_id:&str, twist:Twist2, bound:Option<CommandDispatchBound>,
    )->Result<BoundedTwistOutcome,MotionPortError> {
        // Preserve the absolute bound across a possible connection await.
        <MotionGuardClient as NavMotionPort>::submit_twist_bounded(
            self.normal_client().await?,operation_id,twist,bound).await
    }

    async fn revoke_and_zero(&mut self, operation_id: &str) -> Result<FenceProof, MotionPortError> {
        self.priority_revoke_and_zero(operation_id).await
    }

    async fn priority_revoke_and_zero(
        &mut self,
        operation_id: &str,
    ) -> Result<FenceProof, MotionPortError> {
        if self.pending_fence.is_some() {
            return self.confirm_pending_fence().await;
        }
        // Closing the ordinary stream cancels a delayed write/read without
        // waiting for its 250 ms request timeout.
        self.normal.take();
        let lease_tuple = self
            .lease_tuple
            .clone()
            .ok_or_else(|| MotionPortError::new("priority fence has no active lease tuple"))?;
        let priority_config = self.priority_config.clone();
        let operation_id = operation_id.to_string();
        let priority_request =
            MotionGuardClient::prepare_priority_revoke_and_zero(lease_tuple, &operation_id)
                .map_err(|error| MotionPortError::new(error.to_string()))?;
        let fence = tokio::time::timeout(MAX_CANCEL_FENCE_TIMEOUT, async {
            let mut last_error = None;
            for _ in 0..PRIORITY_FENCE_ATTEMPTS {
                match MotionGuardClient::execute_priority_revoke_and_zero(
                    priority_config.clone(),
                    &priority_request,
                )
                .await
                {
                    Ok(confirmation) => return Ok(confirmation),
                    Err(error) => last_error = Some(error.to_string()),
                }
            }
            Err(MotionPortError::new(last_error.unwrap_or_else(|| {
                "priority fence exhausted without an attempt".into()
            })))
        })
        .await
        .map_err(|_| MotionPortError::new("priority fence exceeded 300 ms hard timeout"))??;
        // 租约和指令通道已在原硬期限内隔离；此后等待同一归零标识的物理停稳。
        self.pending_fence = Some((operation_id, fence));
        self.confirm_pending_fence().await
    }
}

/// Result of one deterministic local-planner step.
#[derive(Debug, Clone, PartialEq)]
pub enum PlannerStep {
    /// Continue controlling with this twist.
    Command(Twist2),
    /// No admitted control result yet; retain route/goal and the existing wait deadline.
    Pending(Vec<nuwax_nav_scan::LocalPlanError>),
    /// Goal arrival detected.
    Succeeded,
    /// Planner cannot safely continue.
    Failed(String),
    /// 当前路径已失效，先确认零速再有界等待更新的规划修订。
    ReplanRequired(String),
}

#[derive(Debug, Clone, PartialEq)]
struct LocalExecutionCommandIntent {
    identity: Arc<PathExecutionIdentity>,
    sequence: u64,
    owner_runtime_generation: u64,
    payload: LocalExecutionPayload,
    dispatch_bound: Option<CommandDispatchBound>,
}

#[derive(Debug, Clone, PartialEq)]
enum LocalExecutionPayload {
    Command(Twist2),
    Pending(Vec<nuwax_nav_scan::LocalPlanError>),
    Succeeded,
    Failed(String),
    ReplanRequired(String),
}

/// Synchronous deterministic planner boundary.
pub trait NavPlanner: Send + 'static {
    /// Take observation timing metadata belonging only to the immediately returned command.
    /// None means that this result has no finite observation-time dependency.
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {None}
    /// Recheck the previous selected full stopping trajectory against the latest
    /// complete surface update. None keeps the legacy full-computation behavior.
    fn revalidate_selected_command(&mut self,_command:Twist2)->Option<PlannerStep> {None}
    /// Recheck the unissued selected command from latest odometry and actual
    /// applied velocity, without selecting another candidate or renewing its lease.
    fn revalidate_selected_command_at_current_state(&mut self,_command:Twist2)->Option<PlannerStep> {None}
    /// 传递既有导航会话标识用于计算线程日志，不参与运动准入。
    fn set_goal_log_context(&mut self, _goal_id: &str) {}
    /// 接续局部路线时保留已受理任务边界；无局部重规划的后端可沿原全局路径。
    fn configure_boundary(&mut self, _boundary: &[[f64; 2]]) {}
    /// 在获取新目标租约前捕获执行选项；默认后端不推定支持倒车。
    fn configure_goal(&mut self, _goal: &PoseStamped, options: nuwax_nav_types_messages::GoalOptions) -> Result<(), RuntimeError> {
        options.validate().map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})?;
        if options.allow_reverse {return Err(RuntimeError::InvalidInput {field:"allow_reverse",reason:"执行后端未声明倒车能力".into()});}
        Ok(())
    }
    /// Replace the active plan after version validation.
    fn replace_path(&mut self, path: Path2D);
    /// 替换路径与可选朝向；不使用朝向的兼容后端仍沿原接口运行。
    fn replace_path_with_orientations(&mut self, path: Path2D, _yaws: Option<Vec<f64>>) {
        self.replace_path(path);
    }
    /// 接收经调度器核对的完整路径与原用户目标，默认后端不支持三维降级。
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        _goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        match path {
            ExecutionPath::Planar(path) => {
                self.replace_path_with_orientations(path, yaws);
                Ok(())
            }
            ExecutionPath::Surface(_) => Err(RuntimeError::InvalidInput {
                field: "execution_path",
                reason: "当前局部后端不支持三维路径".into(),
            }),
        }
    }
    /// Update the latest odometry sample.
    fn update_odometry(&mut self, odometry: PoseStamped);
    /// Update the latest planner-consumed occupancy costmap.
    fn update_costmap(&mut self, costmap: CostMap2D);
    /// 接收与路径维度匹配的地图，默认后端不能把表面快照当作二维占据图。
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        match map {
            ExecutionMap::Planar(map) => {
                self.update_costmap(map);
                Ok(())
            }
            ExecutionMap::Surface(_) | ExecutionMap::SurfaceUpdate(_) => Err(RuntimeError::InvalidInput {
                field: "execution_map",
                reason: "当前局部后端不支持三维表面地图".into(),
            }),
        }
    }
    /// 来源被明确清空或更换时撤销局部缓存，不能用旧地图重新恢复运动。
    fn invalidate_execution_map(&mut self) {
        self.stop();
    }
    /// 接收唯一运动端已确认的指令；后台计算不能把未执行候选当作当前速度。
    fn observe_applied_command(&mut self, _command: Twist2) {}
    /// 计算未完成或控制许可过期时的等待原因；命令回执不能覆盖该执行阶段。
    fn execution_wait_reason(&self) -> Option<String> { None }
    /// 只读展示当前局部规划，不改变速度或路径许可。
    fn local_path_for_display(&self) -> Vec<[f64; 3]> { Vec::new() }
    /// Prepare a deferred control result without blocking the scheduler task.
    /// A receiver requests a bounded wait for publication, not command authority.
    /// `continuing_wait` keeps an already deferred tick waiting for matching input
    /// versions; it does not broaden which initial steps may defer.
    /// Synchronous backends return None and retain their immediate step behavior.
    /// Delegating adapters must forward this hook with `step` to preserve scheduling.
    fn prepare_control_step(&mut self, _robot_width_m: f64, _continuing_wait: bool) -> Option<watch::Receiver<()>> { None }
    /// Produce one controller step without waiting for a background computation.
    fn step(&mut self, robot_width_m: f64) -> PlannerStep;
    /// Clear planner state after a terminal fence.
    fn stop(&mut self);
}

/// Production-qualified local execution backend selected at scheduler construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalExecutionBackendKind {
    /// Run the deterministic basic local planner inside the scheduler task.
    BasicInProcess,
    /// 显式配置机体到支撑面的标定后，在同一调度任务内跟随三维路径。
    SurfaceInProcess,
}

struct BasicInProcessLocalBackend(LocalPlanner);

impl BasicInProcessLocalBackend {
    fn production() -> Self {
        Self(LocalPlanner::with_config(
            LocalPlannerConfig::default_config(),
        ))
    }

    fn set_navigation_motion_limits(&mut self, physical: nuwax_nav_types_messages::MotionLimits) -> Result<(),RuntimeError> {
        physical.validate().map_err(|reason|RuntimeError::InvalidInput{field:"navigation_capabilities",reason})?;
        self.0.set_motion_limits(surface_follower::forward_navigation_limits(&physical))
            .map_err(|reason|RuntimeError::InvalidInput{field:"navigation_capabilities",reason})
    }

    #[cfg(test)]
    fn kind(&self) -> LocalExecutionBackendKind {
        LocalExecutionBackendKind::BasicInProcess
    }
}

impl NavPlanner for BasicInProcessLocalBackend {
    fn configure_goal(&mut self, goal:&PoseStamped, options:nuwax_nav_types_messages::GoalOptions)->Result<(),RuntimeError> {
        let options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options};
        self.0.configure_goal(goal.clone(),options).map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})
    }

    fn replace_path(&mut self, path: Path2D) {
        self.0.start_planning(path);
    }

    fn replace_path_with_orientations(&mut self, path: Path2D, yaws: Option<Vec<f64>>) {
        self.0.start_planning_with_orientations(path, yaws);
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.0.handle_odom(odometry);
    }

    fn update_costmap(&mut self, costmap: CostMap2D) {
        self.0.update_costmap(costmap);
    }

    fn step(&mut self, robot_width_m: f64) -> PlannerStep {
        let (twist, stop) = self.0.step(robot_width_m);
        match stop {
            None => PlannerStep::Command(twist),
            Some(StopMessage::Arrived) => PlannerStep::Succeeded,
            Some(StopMessage::ObstacleFound) => PlannerStep::ReplanRequired("局部轨迹受阻，停车等待新路径".into()),
            Some(StopMessage::Error) => PlannerStep::Failed("local planner error".into()),
        }
    }

    fn stop(&mut self) {
        self.0.stop_planning();
    }
}

fn production_local_backend(
    kind: LocalExecutionBackendKind,
) -> Result<BasicInProcessLocalBackend, RuntimeError> {
    match kind {
        LocalExecutionBackendKind::BasicInProcess => {
            let mut backend = BasicInProcessLocalBackend::production();
            if let Some(value) = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON")
                .ok()
                .filter(|value| !value.trim().is_empty())
            {
                let geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value)
                    .map_err(|reason| RuntimeError::InvalidInput {
                    field: "robot_geometry",
                    reason,
                })?;
                backend.0.set_robot_geometry(geometry);
            }
            if let Some(capabilities)=nuwax_nav_types_messages::NavigationCapabilities::from_env().map_err(|reason| RuntimeError::InvalidInput {field:"navigation_capabilities",reason})? {
                backend.set_navigation_motion_limits(capabilities.motion)?;
            }
            Ok(backend)
        }
        LocalExecutionBackendKind::SurfaceInProcess => Err(RuntimeError::InvalidInput {
            field: "body_to_support_z_m",
            reason: "三维后端必须通过显式配置构造，缺少机体到支撑面的标定".into(),
        }),
    }
}

enum Interruptible<T> {
    Completed(T),
    Urgent(Option<UrgentCommand>),
}

async fn await_interruptible<T, F>(
    urgent_open: bool,
    urgent_rx: &mut mpsc::Receiver<UrgentCommand>,
    future: F,
) -> Interruptible<T>
where
    F: Future<Output = T>,
{
    if !urgent_open {
        return Interruptible::Completed(future.await);
    }
    tokio::pin!(future);
    tokio::select! {
        biased;
        command = urgent_rx.recv() => Interruptible::Urgent(command),
        result = &mut future => Interruptible::Completed(result),
    }
}

/// Single-task navigation scheduler.
///
/// The motion port is stored by value and is only accessed through `&mut self`,
/// making renew, twist, and fence mutually exclusive by construction.
pub struct NavScheduler<M, P> {
    motion: M,
    planner: P,
    config: SchedulerConfig,
    session: Option<NavSession>,
    lease_active: bool,
    current_path_revision: u64,

    current_path: Option<(ExecutionPath, Option<Vec<f64>>)>,
    current_execution_identity: Option<Arc<PathExecutionIdentity>>,
    local_intent_sequence: u64,
    path_wait_started_at: Option<Instant>,
    arrival_confirmation: Option<ArrivalConfirmation>,
    latest_pose: Option<PoseStamped>,
    pose_acquired_at: Option<Instant>,
    pose_receipt: Option<PlannerInputReceipt>,
    pose_valid_until: Option<Instant>,
    obstacle_source_available: bool,
    obstacle_source_bound: Option<CommandDispatchBound>,
    costmap_receipt: Option<PlannerInputReceipt>,
    map_invalidated_at: Option<Instant>,
    operation_sequence: u64,
    status: SchedulerStatus,
    #[cfg(feature = "surface-recovery-qualification")]
    qualification_timing: Option<qualification_timing::TraceSink>,
}

#[derive(Debug, Clone, Copy)]
struct PlannerInputReceipt {
    source_timestamp_ns: i64,
    received_at: Instant,
}

impl<M, P> NavScheduler<M, P>
where
    M: NavMotionPort,
    P: NavPlanner,
{
    /// Construct a scheduler without starting a task.
    pub fn new(motion: M, planner: P, config: SchedulerConfig) -> Result<Self, RuntimeError> {
        validate_config(config)?;
        let status = SchedulerStatus {
            motion_sink_ready: true,
            message: "motion scheduler ready".into(),
            ..SchedulerStatus::default()
        };
        Ok(Self {
            motion,
            planner,
            config,
            session: None,
            lease_active: false,
            current_path_revision: 0,

            current_path: None,
            current_execution_identity: None,
            local_intent_sequence: 0,
            path_wait_started_at: None,
            arrival_confirmation: None,
            latest_pose: None,
            pose_acquired_at: None,
            pose_receipt: None,
            pose_valid_until: None,
            obstacle_source_available: true,
            obstacle_source_bound: None,
            costmap_receipt: None,
            map_invalidated_at: None,
            operation_sequence: 0,
            status,
            #[cfg(feature = "surface-recovery-qualification")]
            qualification_timing: None,
        })
    }

    /// Attach an explicit qualification-only observer; default construction is silent.
    #[cfg(feature = "surface-recovery-qualification")]
    pub fn with_qualification_timing(mut self, sink: qualification_timing::TraceSink) -> Self {
        self.qualification_timing = Some(sink);
        self
    }

    #[cfg(feature = "surface-recovery-qualification")]
    fn qualification_span(&self, phase: &'static str, source: i64)
        -> Option<qualification_timing::SpanGuard> {
        self.qualification_timing.as_ref().map(|sink| sink.span(phase, source))
    }

    #[cfg(feature = "surface-recovery-qualification")]
    fn qualification_snapshot(&self, phase: &'static str, source: i64,
        stale: Option<bool>, stale_cause_bits: u8) {
        let Some(sink) = self.qualification_timing.as_ref().filter(|sink| sink.enabled()) else { return; };
        // These are copies of admitted fields. Do not call refresh_input_status,
        // stale_input_reason, expired_now, planner methods, or any geometry here.
        let now = Instant::now();
        let recent = self.obstacle_source_bound.map(|bound| bound.qualification_timing_fields());
        sink.snapshot_at(phase, source, now.into_std(), qualification_timing::SchedulerSnapshot {
            phase: self.status.phase, lease_active: self.lease_active,
            session_active: self.session.is_some(), planner_inputs_fresh: self.status.planner_inputs_fresh,
            operation_sequence: self.operation_sequence,
            path_revision: self.current_path_revision,
            pose_source_ns: self.pose_receipt.map(|receipt| receipt.source_timestamp_ns),
            map_source_ns: self.costmap_receipt.map(|receipt| receipt.source_timestamp_ns),
            pose_receipt_age_ns: self.pose_receipt.map(|receipt|
                qualification_timing::nanos(now.saturating_duration_since(receipt.received_at))),
            map_receipt_age_ns: self.costmap_receipt.map(|receipt|
                qualification_timing::nanos(now.saturating_duration_since(receipt.received_at))),
            pose_deadline_remaining_ns: self.pose_valid_until.map(|deadline|
                qualification_timing::remaining_ns(deadline.into_std(), now.into_std())),
            recent_available: self.obstacle_source_available,
            recent_bound_present: recent.is_some(),
            recent_anchor_unix_ns: recent.map(|(anchor, _, _)| anchor),
            recent_latest_issue_unix_ns: recent.map(|(_, latest, _)| latest),
            recent_deadline_remaining_ns: recent.and_then(|(_, _, deadline)| deadline)
                .map(|deadline| qualification_timing::remaining_ns(deadline, now.into_std())),
            last_applied: self.status.last_applied_twist.map(|twist|
                [twist.linear.x, twist.linear.y, twist.angular_z]),
            stale, stale_cause_bits,
        });
    }

    async fn run(
        mut self,
        mut input_rx: mpsc::Receiver<InputCommand>,
        mut urgent_rx: mpsc::Receiver<UrgentCommand>,
        status_tx: watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        let mut tick = interval_at(
            Instant::now() + self.config.tick_interval,
            self.config.tick_interval,
        );
        tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut fallback_renewal = Instant::now() + self.config.renew_interval;
        let mut input_open = true;
        let mut urgent_open = true;
        let mut deferred_control: Option<DeferredControl> = None;

        loop {
            // A deferred tick belongs to one accepted path, never the next goal
            // or a replacement path admitted while computation was outstanding.
            if deferred_control.as_ref().is_some_and(|wait|
                self.status.phase != SchedulerPhase::Controlling
                    || self.arrival_confirmation.is_some()
                    || !self.current_execution_identity.as_ref()
                        .is_some_and(|identity| Arc::ptr_eq(identity, &wait.identity))) {
                deferred_control = None;
            }
            if !input_open && !urgent_open {
                return self
                    .finish_terminal(&status_tx, TerminalOutcome::Shutdown, SchedulerPhase::Idle)
                    .await;
            }

            let renewal_due = self.motion.renewal_due_at().map_or(fallback_renewal, |due| due.min(fallback_renewal));
            let controlling=matches!(self.status.phase,SchedulerPhase::AwaitingPath|SchedulerPhase::Controlling)
                || self.arrival_confirmation.is_some();
            tokio::select! {
                biased;

                command = urgent_rx.recv(), if urgent_open => {
                    match command {
                        Some(command) => {
                            if let Some(result) = self.handle_urgent(&status_tx, command).await {
                                return result;
                            }
                        }
                        None => urgent_open = false,
                    }
                }

                _ = tokio::time::sleep_until(renewal_due), if self.should_renew() => {
                    match await_interruptible(urgent_open, &mut urgent_rx, self.renew()).await {
                        Interruptible::Completed(Ok(())) => {
                            fallback_renewal = Instant::now() + self.config.renew_interval;
                        }
                        Interruptible::Completed(Err(error)) => {
                            let error = self.fail_closed(&status_tx, error).await;
                            // 单个目标被外部停车撤租后，精确围栏已确认便退休目标。
                            // 进程继续拥有输入与下一目标，不能靠重启完成业务取消。
                            if !self.lease_active && self.session.is_none() { continue; }
                            self.status.motion_sink_ready = false;
                            self.publish_status(&status_tx);
                            if matches!(error, RuntimeError::StopUnconfirmed(_)) { continue; }
                            return Err(error);
                        }
                        Interruptible::Urgent(Some(command)) => {
                            if let Some(result) = self.handle_urgent(&status_tx, command).await {
                                return result;
                            }
                        }
                        Interruptible::Urgent(None) => {
                            urgent_open = false;
                            if self.lease_active {
                                return self.finish_terminal(
                                    &status_tx,
                                    TerminalOutcome::Shutdown,
                                    SchedulerPhase::Idle,
                                ).await;
                            }
                        }
                    }
                }

                event = async {
                    if let Some(wait) = deferred_control.as_mut() {
                        match wait.queued_inputs {
                            Some(0) => SchedulerEvent::ControlReady,
                            Some(_) => SchedulerEvent::Input(input_rx.recv().await),
                            None => tokio::select! {
                                biased;
                                finish = wait.ready() => SchedulerEvent::Publication(finish),
                                command = input_rx.recv(), if input_open => SchedulerEvent::Input(command),
                            },
                        }
                    } else {
                        // 普通输入与控制公平轮换；取消和续租仍由外层优先选择。
                        tokio::select! {
                            command=input_rx.recv(), if input_open=>SchedulerEvent::Input(command),
                            _=tick.tick(), if controlling=>SchedulerEvent::Tick,
                            else=>std::future::pending().await,
                        }
                    }
                } => match event {
                    SchedulerEvent::Input(command)=> {
                        if let Some(remaining) = deferred_control.as_mut().and_then(|wait| wait.queued_inputs.as_mut()) {
                            *remaining = remaining.saturating_sub(1);
                        }
                        match command {
                            Some(command)=>self.handle_input(&status_tx,command).await,
                            None=>input_open=false,
                        }
                    }
                    SchedulerEvent::Publication(finish_wait) => {
                        if let Some(wait) = deferred_control.as_mut() {
                            // Admit the finite prefix already queued at readiness.
                            // Versions advance only in handle_input, so consuming a
                            // publication first could otherwise dispatch old sources.
                            // New arrivals do not extend this prefix or its deadline.
                            wait.queued_inputs = Some(input_rx.len());
                            wait.finish_wait = finish_wait;
                        }
                    }
                    SchedulerEvent::Tick | SchedulerEvent::ControlReady => {
                        let previous = deferred_control.take();
                        let deadline = previous.as_ref().map_or_else(
                            || Instant::now() + self.config.tick_interval, |wait| wait.deadline);
                        let finish_wait = previous.as_ref().is_some_and(|wait| wait.finish_wait)
                            || Instant::now() >= deadline;
                        if !finish_wait && self.status.phase == SchedulerPhase::Controlling
                            && self.arrival_confirmation.is_none() {
                            if let Some(identity) = self.current_execution_identity.clone() {
                                let publication = {
                                    #[cfg(feature = "surface-recovery-qualification")]
                                    let _qualification_span = self.qualification_span(
                                        "scheduler.prepare_control_step", self.status.pose_source_timestamp_ns);
                                    self.planner.prepare_control_step(self.config.robot_width_m, previous.is_some())
                                };
                                if let Some(publication) = publication {
                                    deferred_control = Some(DeferredControl {
                                        publication, deadline, identity, queued_inputs: None, finish_wait: false,
                                    });
                                    continue;
                                }
                            }
                        }
                    // 重新取租约期间已经确认静止；与原StartSession一样收敛有界
                    // acquire结果，不能丢弃未知grant后再拿旧停车票据确认新租约。
                    let outcome = if self.arrival_resume_requested() {
                        Interruptible::Completed(self.session_tick(&status_tx).await)
                    } else {
                        await_interruptible(urgent_open, &mut urgent_rx, self.session_tick(&status_tx)).await
                    };
                    match outcome {
                        Interruptible::Completed(Ok(())) => {}
                        Interruptible::Completed(Err(error)) => {
                            if !self.lease_active && self.session.is_none() { continue; }
                            self.status.motion_sink_ready = false;
                            self.publish_status(&status_tx);
                            if matches!(error, RuntimeError::StopUnconfirmed(_)) { continue; }
                            return Err(error);
                        }
                        Interruptible::Urgent(Some(command)) => {
                            if let Some(result) = self.handle_urgent(&status_tx, command).await {
                                return result;
                            }
                        }
                        Interruptible::Urgent(None) => {
                            return self.finish_terminal(
                                &status_tx,
                                TerminalOutcome::Shutdown,
                                SchedulerPhase::Idle,
                            ).await;
                        }
                    }
                    }
                }
            }
        }
    }

    async fn handle_urgent(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        command: UrgentCommand,
    ) -> Option<Result<(), RuntimeError>> {
        match command {
            UrgentCommand::Cancel { session_id, reply } => {
                let result = self.cancel(status_tx, &session_id).await;
                let _ = reply.send(result);
                None
            }
            UrgentCommand::InvalidateMap {
                reason,
                invalidated_at,
                reply,
            } => {
                self.map_invalidated_at = Some(
                    self.map_invalidated_at
                        .map_or(invalidated_at, |previous| previous.max(invalidated_at)),
                );
                self.costmap_receipt = None;
                self.planner.invalidate_execution_map();
                self.refresh_input_status(Instant::now());
                let result = if self.lease_active || self.session.is_some() {
                    let error = self
                        .fail_closed(
                            status_tx,
                            RuntimeError::StaleInput(format!("地图来源失效: {reason}")),
                        )
                        .await;
                    if self.lease_active {
                        Err(error)
                    } else {
                        Ok(())
                    }
                } else {
                    self.status.message = format!("地图来源失效，等待新输入: {reason}");
                    self.publish_status(status_tx);
                    Ok(())
                };
                let _ = reply.send(result);
                None
            }
            UrgentCommand::Shutdown { reply } => {
                let result = self
                    .finish_terminal(status_tx, TerminalOutcome::Shutdown, SchedulerPhase::Idle)
                    .await;
                let _ = reply.send(result.clone());
                Some(result)
            }
        }
    }

    async fn handle_input(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        command: InputCommand,
    ) {
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_input = self.qualification_timing.as_ref().map(|sink| {
            let (kind, source) = match &command.kind {
                InputKind::StartSession(_) => ("scheduler.input.start_session", 0),
                InputKind::SubmitPath(_) => ("scheduler.input.path", 0),
                InputKind::UpdateOdometry(pose, _, _) => ("scheduler.input.pose", pose.stamp.nanos),
                InputKind::UpdateCostmap(map, _) => ("scheduler.input.map", map.source_timestamp_ns),
                // Runtime owns no recent source timestamp; record its actual incoming bound.
                InputKind::UpdateObstacleSource(_, _, _) => ("scheduler.input.recent", 0),
                InputKind::PlanningDiagnostic(_, _, _) => ("scheduler.input.planning_diagnostic", 0),
            };
            sink.mark("scheduler.input.dequeue", source);
            match &command.kind {
                InputKind::SubmitPath(path) => sink.count("scheduler.input.path_revision", source, path.path_revision),
                InputKind::UpdateObstacleSource(_, available, bound) => {
                    sink.count("scheduler.input.recent_available", 0, u64::from(*available));
                    sink.bound("scheduler.input.recent_bound", 0, *bound);
                }
                _ => {}
            }
            (sink.clone(), source, sink.span(kind, source))
        });
        let result = match command.kind {
            InputKind::StartSession(session) => self.start_session(status_tx, session).await,
            InputKind::SubmitPath(path) => self.submit_path(status_tx, path).await,
            InputKind::UpdateOdometry(odometry, valid_until, acquired_at) => {
                self.observe_odometry_at(status_tx, odometry, valid_until, acquired_at)
            }
            InputKind::UpdateCostmap(observation, received_at) => {
                self.observe_costmap(status_tx, observation, received_at)
            }
            InputKind::UpdateObstacleSource(map, available, bound) => (|| {
                if let Some(map)=map {self.planner.update_execution_map(map)?;}
                self.obstacle_source_available=available;
                self.obstacle_source_bound=bound;
                self.refresh_input_status(Instant::now());
                self.publish_status(status_tx);
                Ok(())
            })(),
            InputKind::PlanningDiagnostic(goal_id, key, message) => {
                if self.status.phase == SchedulerPhase::AwaitingPath && self.session.as_ref().is_some_and(|session|
                    session.session_id == goal_id && session.execution_identity.goal_key() == key) {
                    self.status.message = message; self.publish_status(status_tx);
                }
                Ok(())
            }
        };
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_ok = result.is_ok();
        let _ = command.reply.send(result);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = qualification_input {
            sink.mark(if qualification_ok { "scheduler.input.ack.ok" }
                else { "scheduler.input.ack.error" }, source);
        }
    }

    #[cfg(test)]
    fn observe_odometry(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        odometry: PoseStamped,
        valid_until: Option<Instant>,
    ) -> Result<(), RuntimeError> {
        self.observe_odometry_at(status_tx, odometry, valid_until, Instant::now())
    }

    fn observe_odometry_at(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        odometry: PoseStamped,
        valid_until: Option<Instant>,
        acquired_at: Instant,
    ) -> Result<(), RuntimeError> {
        if acquired_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "odometry.acquired_at", reason: "位姿采集时刻或来源期限无效".into(),
            });
        }
        if odometry.frame_id != nuwax_nav_types_messages::NavFrameId::Map {
            return Err(RuntimeError::InvalidInput {
                field: "odometry.frame_id",
                reason: format!("expected map, got {}", odometry.frame_id.as_str()),
            });
        }
        if valid_until.is_some_and(|deadline| deadline <= Instant::now()) {
            return Err(RuntimeError::StaleInput(
                "pose source deadline already elapsed before admission".into(),
            ));
        }
        let source_timestamp_ns = odometry.stamp.nanos;
        validate_source_timestamp(
            "odometry.source_timestamp_ns",
            source_timestamp_ns,
            self.pose_receipt,
        )?;
        self.latest_pose = Some(odometry.clone());
        self.pose_acquired_at = Some(acquired_at);
        self.planner.update_odometry(odometry);
        self.pose_receipt = Some(PlannerInputReceipt {
            source_timestamp_ns,
            received_at: Instant::now(),
        });
        self.pose_valid_until = valid_until;
        self.refresh_input_status(Instant::now());
        if self.status.last_terminal.is_none() && self.session.is_none() {
            self.status.message = "fresh map pose consumed".into();
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn observe_costmap(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        observation: CostmapObservation,
        received_at: Instant,
    ) -> Result<(), RuntimeError> {
        if received_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "costmap.received_at",
                reason: "地图接收时刻不能位于未来".into(),
            });
        }
        if self
            .map_invalidated_at
            .is_some_and(|invalidated_at| received_at < invalidated_at)
        {
            return Err(RuntimeError::StaleInput(
                "地图在来源失效前已经接收，不能迟到恢复旧地图".into(),
            ));
        }
        // A verified surface mask/statistics update can keep the same source
        // stamp. Deliver its geometry, but never renew the original receipt.
        let unchanged_surface_source=matches!(&observation.costmap,ExecutionMap::SurfaceUpdate(_))
            && self.costmap_receipt.is_some_and(|old|old.source_timestamp_ns==observation.source_timestamp_ns);
        if !unchanged_surface_source {
            validate_source_timestamp("costmap.source_timestamp_ns",observation.source_timestamp_ns,self.costmap_receipt)?;
        }
        self.planner.update_execution_map(observation.costmap)?;
        if !unchanged_surface_source {
            self.costmap_receipt = Some(PlannerInputReceipt {
                source_timestamp_ns: observation.source_timestamp_ns, received_at,
            });
        }
        self.refresh_input_status(Instant::now());
        if self.status.last_terminal.is_none() && self.session.is_none() {
            self.status.message = "fresh occupancy costmap consumed".into();
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn refresh_input_status(&mut self, now: Instant) {
        self.status.pose_source_timestamp_ns = self
            .pose_receipt
            .map_or(0, |receipt| receipt.source_timestamp_ns);
        self.status.costmap_source_timestamp_ns = self
            .costmap_receipt
            .map_or(0, |receipt| receipt.source_timestamp_ns);
        self.status.pose_receipt_age_ms = receipt_age_ms(self.pose_receipt, now);
        self.status.costmap_receipt_age_ms = receipt_age_ms(self.costmap_receipt, now);
        self.status.planner_inputs_fresh = self
            .pose_receipt
            .is_some_and(|receipt| self.config.pose_ttl.is_zero() || now.duration_since(receipt.received_at) <= self.config.pose_ttl)
            && self.pose_valid_until.is_none_or(|deadline| now < deadline)
            && self.obstacle_source_available
            && self.obstacle_source_bound.is_none_or(|bound|!bound.expired_now())
            && self.costmap_receipt.is_some_and(|receipt| {
                self.config.costmap_ttl.is_zero() || now.duration_since(receipt.received_at) <= self.config.costmap_ttl
            });
    }

    fn stale_input_reason(&mut self, now: Instant) -> Option<String> {
        self.refresh_input_status(now);
        if !self.obstacle_source_available {return Some("recent obstacle source revoked".into());}
        if self.obstacle_source_bound.is_some_and(CommandDispatchBound::expired_now) {
            return Some("recent obstacle source deadline elapsed".into());
        }
        let pose_reason =
            stale_receipt_reason("pose", self.pose_receipt, now, self.config.pose_ttl).or_else(
                || {
                    self.pose_valid_until
                        .filter(|deadline| now >= *deadline)
                        .map(|_| {
                            format!(
                                "pose source deadline elapsed (source_timestamp_ns={})",
                                self.status.pose_source_timestamp_ns
                            )
                        })
                },
            );
        let costmap_reason = stale_receipt_reason(
            "costmap",
            self.costmap_receipt,
            now,
            self.config.costmap_ttl,
        );
        match (pose_reason, costmap_reason) {
            (None, None) => None,
            (Some(reason), None) | (None, Some(reason)) => Some(reason),
            (Some(pose), Some(costmap)) => Some(format!("{pose}; {costmap}")),
        }
    }

    async fn start_session(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        session: NavSession,
    ) -> Result<(), RuntimeError> {
        validate_session(&session)?;
        if self.lease_active || self.session.is_some() {
            return Err(RuntimeError::Busy(
                "active session must be fenced before replacement".into(),
            ));
        }
        let session_id = session.session_id.clone();
        let motion_admission = session.motion_admission;
        let guard_generation = motion_admission.guard_generation;
        self.planner.stop();
        self.planner.set_goal_log_context(&session.session_id);
        self.planner.configure_goal(&session.goal_pose,session.options)?;
        self.planner.configure_boundary(&session.boundary);
        self.session = Some(session);
        self.current_path_revision = 0;

        self.current_path = None;
        self.current_execution_identity = None;
        self.local_intent_sequence = 0;
        self.path_wait_started_at = None;
        self.status = SchedulerStatus {
            phase: SchedulerPhase::Acquiring,
            session_id: Some(session_id),
            path_revision: 0,
            applied_commands: 0,
            last_applied_twist: None,
            motion_sink_ready: true,
            guard_generation,
            planner_inputs_fresh: self.status.planner_inputs_fresh,
            pose_source_timestamp_ns: self.status.pose_source_timestamp_ns,
            costmap_source_timestamp_ns: self.status.costmap_source_timestamp_ns,
            pose_receipt_age_ms: self.status.pose_receipt_age_ms,
            costmap_receipt_age_ms: self.status.costmap_receipt_age_ms,
            last_terminal: None,
            last_zero_ack_id: None,
            last_fence_monotonic_ns: 0,
            message: "acquiring NAV lease".into(),
            local_path: Vec::new(),
        };
        self.publish_status(status_tx);

        let operation_id = self.next_operation_id("acquire")?;
        match self.motion.acquire(&operation_id, &motion_admission).await {
            Ok(proof)
                if proof.lease_id != 0
                    && proof.guard_generation == motion_admission.guard_generation =>
            {
                self.lease_active = true;
                self.status.motion_sink_ready = true;
                self.status.guard_generation = proof.guard_generation;
                self.path_wait_started_at = Some(Instant::now());
                self.status.phase = SchedulerPhase::AwaitingPath;
                self.status.message = "NAV lease active; awaiting versioned path".into();
                self.publish_status(status_tx);
                Ok(())
            }
            Ok(_) => {
                self.lease_active = true;
                let error = RuntimeError::Motion(
                    "guard returned a lease outside the admitted generation".into(),
                );
                Err(self.fail_closed(status_tx, error).await)
            }
            Err(error) if error.lease_may_be_active => {
                self.lease_active = true;
                let error = RuntimeError::Motion(error.message);
                Err(self.fail_closed(status_tx, error).await)
            }
            Err(error) => {
                self.planner.stop();
                self.session = None;
                self.current_path_revision = 0;

                self.current_path = None;
                self.current_execution_identity = None;
                self.local_intent_sequence = 0;
                self.status.phase = SchedulerPhase::Failed;
                self.status.session_id = None;
                self.status.path_revision = 0;
                self.status.motion_sink_ready = false;
                self.status.message = error.message.clone();
                self.publish_status(status_tx);
                Err(RuntimeError::Motion(error.message))
            }
        }
    }

    async fn submit_path(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        path: VersionedPath,
    ) -> Result<(), RuntimeError> {
        validate_path_shape(&path)?;
        let Some(session) = self.session.as_ref() else {
            return Err(path_rejected("no active navigation session"));
        };
        if !self.lease_active
            || !matches!(
                self.status.phase,
                SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling
            )
        {
            return Err(path_rejected("active session is not accepting paths"));
        }
        if path.session_id != session.session_id
            || !path
                .execution_identity
                .has_same_goal_identity(&session.execution_identity)
        {
            return Err(path_rejected(
                "path execution identity does not match the active navigation session",
            ));
        }
        if path.path_revision < self.current_path_revision {
            return Err(path_rejected("path revision is stale"));
        }
        if path.path_revision == self.current_path_revision && self.current_path_revision != 0 {
            let immutable = self.current_path.as_ref().is_some_and(|(current, yaws)| {
                    same_execution_path(current, &path.path) && yaws == &path.path_yaws
                })
                && self
                    .current_execution_identity
                    .as_ref()
                    .is_some_and(|identity| identity.as_ref() == &path.execution_identity);
            if immutable {
                if self.status.phase == SchedulerPhase::AwaitingPath {
                    return Err(path_rejected(
                        "失效路径需要更高规划修订，不能用同修订重放恢复",
                    ));
                }
                return Ok(());
            }
            let error = RuntimeError::PathMutation {
                reason: format!(
                    "session={} path_revision={} changed identity or payload",
                    path.session_id, path.path_revision
                ),
            };
            let qualified_cause = format!("SafetyFailed cause=PathMutation: {error}");
            self.status.message = qualified_cause.clone();
            self.finish_terminal(
                status_tx,
                TerminalOutcome::SafetyFailed,
                SchedulerPhase::Failed,
            )
            .await?;
            self.status.message = qualified_cause;
            self.publish_status(status_tx);
            return Err(error);
        }

        let VersionedPath {
            path: planner_path,
            path_yaws,
            execution_identity,
            path_revision,

            ..
        } = path;
        self.planner.replace_execution_path(
            planner_path.clone(),
            path_yaws.clone(),
            session.goal_pose.clone(),
        )?;
        self.current_path_revision = path_revision;

        self.current_path = Some((planner_path, path_yaws));
        self.current_execution_identity = Some(Arc::new(execution_identity));
        // 等待预算在真正执行有效控制步骤后清除，不能靠反复投递无效新路径续期。
        self.status.phase = SchedulerPhase::Controlling;
        self.status.path_revision = self.current_path_revision;
        self.status.message = "versioned path accepted".into();
        self.publish_status(status_tx);
        Ok(())
    }

    async fn cancel(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        session_id: &str,
    ) -> Result<(), RuntimeError> {
        let Some(session) = self.session.as_ref() else {
            return Err(RuntimeError::Busy("no active session to cancel".into()));
        };
        if session_id.trim() != session.session_id {
            return Err(RuntimeError::Busy(format!(
                "cancel session {} does not match active {}",
                session_id.trim(),
                session.session_id
            )));
        }
        self.status.phase = SchedulerPhase::Cancelling;
        self.status.message = "cancel accepted; waiting for guard fence".into();
        self.publish_status(status_tx);
        self.finish_terminal(status_tx, TerminalOutcome::Cancelled, SchedulerPhase::Idle)
            .await
    }

    async fn control_tick(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.control_tick", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.control_tick.enter", self.status.pose_source_timestamp_ns, None, 0);
        if let Some(reason) = self.stale_input_reason(Instant::now()) {
            #[cfg(feature = "surface-recovery-qualification")]
            if self.qualification_timing.as_ref().is_some_and(|sink| sink.enabled()) {
                // Classify only the existing result. Never evaluate staleness again.
                let mut bits = 0;
                if reason.contains("pose") { bits |= 1; }
                if reason.contains("costmap") { bits |= 2; }
                if reason.contains("recent") { bits |= 4; }
                if bits == 0 { bits = 8; }
                self.qualification_snapshot("scheduler.control_tick.stale", self.status.pose_source_timestamp_ns, Some(true), bits);
            }
            let qualified_cause = format!("SafetyFailed cause=StaleInput: {reason}");
            self.status.message = qualified_cause.clone();
            self.publish_status(status_tx);
            self.finish_terminal(
                status_tx,
                TerminalOutcome::SafetyFailed,
                SchedulerPhase::Failed,
            )
            .await?;
            self.status.message = qualified_cause;
            self.publish_status(status_tx);
            return Ok(());
        }
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.control_tick.fresh", self.status.pose_source_timestamp_ns, Some(false), 0);
        let step = {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.planner.step", self.status.pose_source_timestamp_ns);
            self.planner.step(self.config.robot_width_m)
        };
        let intent = self.wrap_local_intent(step)?;
        if !self.local_intent_is_current(&intent) {
            self.status.message = "stale local execution intent rejected".into();
            self.publish_status(status_tx);
            return Ok(());
        }
        match intent.payload {
            LocalExecutionPayload::Command(twist) => {
                let wait_reason = self.planner.execution_wait_reason();
                if !self.apply_controller_twist_with_bound(status_tx, twist, intent.dispatch_bound).await? {
                    self.path_wait_started_at.get_or_insert_with(Instant::now);
                    return self.check_path_wait_timeout(status_tx).await;
                }
                if twist == Twist2::ZERO {
                    // 零速已确认不等于规划恢复；同一轮持续等待不能被每拍归零续期。
                    // 正常制动后会继续运动或到达；缺少原因也不能无限清除等待期限。
                    self.path_wait_started_at.get_or_insert_with(Instant::now);
                    self.status.message = wait_reason.filter(|reason| !reason.trim().is_empty())
                        .unwrap_or_else(|| "控制器持续零速，等待可执行运动".into());
                    self.publish_status(status_tx);
                    return self.check_path_wait_timeout(status_tx).await;
                }
                self.path_wait_started_at = None;
                Ok(())
            }
            LocalExecutionPayload::Pending(failures) => {
                self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
                self.path_wait_started_at.get_or_insert_with(Instant::now);
                // Rebuild from this tick's evidence; never append to the previous status.
                self.status.message = self.planner.execution_wait_reason()
                    .filter(|reason| !reason.trim().is_empty())
                    .unwrap_or_else(|| "局部轨迹待定，保持零速等待".into());
                if !failures.is_empty() {
                    self.status.message = format!("{}: {failures:?}", self.status.message);
                }
                self.publish_status(status_tx);
                self.check_path_wait_timeout(status_tx).await
            }
            LocalExecutionPayload::ReplanRequired(reason) => {
                self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
                self.planner.stop();
                self.path_wait_started_at.get_or_insert_with(Instant::now);
                self.status.phase = SchedulerPhase::AwaitingPath;
                self.status.message = format!("路径失效，零速已执行，等待更新路径: {reason}");
                self.publish_status(status_tx);
                Ok(())
            }
            LocalExecutionPayload::Succeeded => {
                self.finish_terminal(status_tx, TerminalOutcome::Succeeded, SchedulerPhase::Idle)
                    .await
            }
            LocalExecutionPayload::Failed(reason) => {
                let qualified_cause = format!("PlannerFailed cause=LocalPlanner: {reason}");
                self.status.message = qualified_cause.clone();
                self.publish_status(status_tx);
                self.finish_terminal(
                    status_tx,
                    TerminalOutcome::PlannerFailed,
                    SchedulerPhase::Failed,
                )
                .await?;
                self.status.message = qualified_cause;
                self.publish_status(status_tx);
                Ok(())
            }
        }
    }

    /// 运动和等待重规划的零速共用同一提交与回执校验路径。
    async fn apply_controller_twist(
        &mut self, status_tx:&watch::Sender<SchedulerStatus>, twist:Twist2,
    )->Result<(),RuntimeError> {
        self.apply_controller_twist_with_bound(status_tx,twist,None).await.map(|_|())
    }

    // False means the selected command expired and an ordinary zero was confirmed.
    async fn apply_controller_twist_with_bound(
        &mut self, status_tx:&watch::Sender<SchedulerStatus>, twist:Twist2,
        mut bound:Option<CommandDispatchBound>,
    )->Result<bool,RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.apply_controller_twist", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some(sink) = &self.qualification_timing {
            sink.motion("scheduler.apply.requested", self.status.pose_source_timestamp_ns,
                [twist.linear.x, twist.linear.y, twist.angular_z]);
        }
        if twist!=Twist2::ZERO {
            if let Some(source)=self.obstacle_source_bound {
                bound=Some(bound.map_or(source,|candidate|candidate.earliest(source)));
            }
        }
        if let Err(error)=validate_twist(twist) {return Err(self.fail_closed(status_tx,error).await);}
        if self.motion.renewal_due_at().is_some_and(|due|Instant::now()>=due) {
            if let Err(error)=self.renew().await {return Err(self.fail_closed(status_tx,error).await);}
        }
        let mut applied_twist=twist;
        let mut expired=false;
        loop {
            // This check is after renewal. The concrete port also checks after
            // any connection await and caps the receiver's existing deadline.
            if bound.is_some_and(CommandDispatchBound::expired_now) {
                applied_twist=Twist2::ZERO;bound=None;expired=true;
            }
            let operation_id=self.next_operation_id("twist")?;
            #[cfg(feature = "surface-recovery-qualification")]
            self.qualification_snapshot("scheduler.motion.submit.enter", self.status.pose_source_timestamp_ns, None, 0);
            let outcome = {
                #[cfg(feature = "surface-recovery-qualification")]
                let _qualification_span = self.qualification_span("scheduler.motion.submit", self.status.pose_source_timestamp_ns);
                #[cfg(feature = "surface-recovery-qualification")]
                if let Some(sink) = &self.qualification_timing {
                    sink.motion("scheduler.motion.submit.command", self.status.pose_source_timestamp_ns,
                        [applied_twist.linear.x, applied_twist.linear.y, applied_twist.angular_z]);
                    sink.bound("scheduler.motion.submit.bound", self.status.pose_source_timestamp_ns, bound);
                }
                self.motion.submit_twist_bounded(&operation_id,applied_twist,bound).await
            };
            #[cfg(feature = "surface-recovery-qualification")]
            if let Some(sink) = &self.qualification_timing {
                sink.mark(match &outcome {
                    Ok(BoundedTwistOutcome::Applied(_)) => "scheduler.motion.submit.return.applied",
                    Ok(BoundedTwistOutcome::ExpiredNotApplied) => "scheduler.motion.submit.return.expired",
                    Err(_) => "scheduler.motion.submit.return.error",
                }, self.status.pose_source_timestamp_ns);
            }
            let proof=match outcome {
                Ok(BoundedTwistOutcome::Applied(proof))=>proof,
                Ok(BoundedTwistOutcome::ExpiredNotApplied) if bound.is_some()=>{
                    applied_twist=Twist2::ZERO;bound=None;expired=true;continue;
                },
                Ok(BoundedTwistOutcome::ExpiredNotApplied)=>{
                    return Err(self.fail_closed(status_tx,RuntimeError::Motion("unbounded zero submission reported observation expiry".into())).await);
                },
                Err(error)=>return Err(self.fail_closed(status_tx,RuntimeError::Motion(error.message)).await),
            };
            if proof.operation_id!=operation_id || proof.applied_monotonic_ns==0 {
                let reason=format!("expected operation_id={operation_id}, got {} at {}",proof.operation_id,proof.applied_monotonic_ns);
                return Err(self.fail_closed(status_tx,RuntimeError::InvalidApplied(reason)).await);
            }
            self.status.applied_commands=self.status.applied_commands.saturating_add(1);
            self.status.last_applied_twist=Some(applied_twist);
            self.planner.observe_applied_command(applied_twist);
            self.status.message=if expired {"执行来源凭据在下发前已过期，零速已执行，等待重新计算".into()}else{"twist APPLIED".into()};
            self.publish_status(status_tx);
            return Ok(!expired);
        }
    }

    fn wrap_local_intent(
        &mut self,
        step: PlannerStep,
    ) -> Result<LocalExecutionCommandIntent, RuntimeError> {
        self.local_intent_sequence = self
            .local_intent_sequence
            .checked_add(1)
            .ok_or(RuntimeError::SequenceExhausted)?;
        let identity = Arc::clone(self.current_execution_identity.as_ref().ok_or_else(|| {
            RuntimeError::Planner("local planner produced an intent without an active path".into())
        })?);
        let owner_runtime_generation = self
            .session
            .as_ref()
            .map(|session| session.local_owner_runtime_generation)
            .ok_or_else(|| {
                RuntimeError::Planner(
                    "local planner produced an intent without an active session".into(),
                )
            })?;
        let candidate_bound=self.planner.take_command_dispatch_bound();
        let dispatch_bound=if matches!(&step,PlannerStep::Command(v) if *v!=Twist2::ZERO) {candidate_bound}else{None};
        let payload = match step {
            PlannerStep::Command(twist) => LocalExecutionPayload::Command(twist),
            PlannerStep::Pending(failures) => LocalExecutionPayload::Pending(failures),
            PlannerStep::Succeeded => LocalExecutionPayload::Succeeded,
            PlannerStep::Failed(reason) => LocalExecutionPayload::Failed(reason),
            PlannerStep::ReplanRequired(reason) => LocalExecutionPayload::ReplanRequired(reason),
        };
        Ok(LocalExecutionCommandIntent {
            identity,
            sequence: self.local_intent_sequence,
            owner_runtime_generation,
            payload,
            dispatch_bound,
        })
    }

    fn local_intent_is_current(&self, intent: &LocalExecutionCommandIntent) -> bool {
        self.lease_active
            && self.status.phase == SchedulerPhase::Controlling
            && intent.sequence != 0
            && intent.sequence == self.local_intent_sequence
            && self
                .current_execution_identity
                .as_ref()
                .is_some_and(|identity| Arc::ptr_eq(identity, &intent.identity))
            && self.session.as_ref().is_some_and(|session| {
                session.local_owner_runtime_generation == intent.owner_runtime_generation
                    && intent
                        .identity
                        .has_same_goal_identity(&session.execution_identity)
            })
    }

    async fn session_tick(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        if self.arrival_confirmation.is_some() {
            return self.confirm_stopped_arrival(status_tx).await;
        }
        if self.status.phase == SchedulerPhase::Controlling {
            return self.control_tick(status_tx).await;
        }
        if self.status.phase != SchedulerPhase::AwaitingPath {
            return Ok(());
        }
        if self.current_path_revision > 0 {
            if let Some(reason) = self.stale_input_reason(Instant::now()) {
                self.status.message = format!("SafetyFailed cause=StaleInput: {reason}");
                self.finish_terminal(
                    status_tx,
                    TerminalOutcome::SafetyFailed,
                    SchedulerPhase::Failed,
                )
                .await?;
                self.status.message = format!("SafetyFailed cause=StaleInput: {reason}");
                self.publish_status(status_tx);
                return Ok(());
            }
        }
        self.check_path_wait_timeout(status_tx).await
    }

    async fn check_path_wait_timeout(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        let Some(started_at) = self.path_wait_started_at else {
            let error = RuntimeError::Planner("awaiting path without a deadline".into());
            return Err(self.fail_closed(status_tx, error).await);
        };
        if self.config.path_wait_timeout.is_zero()
            || Instant::now().duration_since(started_at) < self.config.path_wait_timeout {
            return Ok(());
        }
        let timeout_message = format!(
            "planner path timeout after {}ms; fencing",
            self.config.path_wait_timeout.as_millis(),
        );
        self.status.message = if self.status.message.trim().is_empty() {
            timeout_message
        } else {
            format!("{timeout_message}; {}", self.status.message)
        };
        self.publish_status(status_tx);
        self.finish_terminal(
            status_tx,
            TerminalOutcome::PlannerFailed,
            SchedulerPhase::Failed,
        )
        .await
    }

    async fn renew(&mut self) -> Result<(), RuntimeError> {
        let operation_id = self.next_operation_id("renew")?;
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.renew.enter", self.status.pose_source_timestamp_ns, None, 0);
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.renew", self.status.pose_source_timestamp_ns);
        let result = self.motion
            .renew(&operation_id)
            .await
            .map_err(|error| RuntimeError::Motion(error.message));
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot(if result.is_ok() { "scheduler.renew.ok" }
            else { "scheduler.renew.error" }, self.status.pose_source_timestamp_ns, None, 0);
        result
    }

    async fn fail_closed(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        original: RuntimeError,
    ) -> RuntimeError {
        let cause_kind = match &original {
            RuntimeError::InvalidInput { .. } => "InvalidInput",
            RuntimeError::Busy(_) => "Busy",
            RuntimeError::PathRejected { .. } => "PathRejected",
            RuntimeError::PathMutation { .. } => "PathMutation",
            RuntimeError::Motion(_) => "Motion",
            RuntimeError::StopUnconfirmed(_) => "StopUnconfirmed",
            RuntimeError::Planner(_) => "Planner",
            RuntimeError::StaleInput(_) => "StaleInput",
            RuntimeError::InvalidApplied(_) => "InvalidApplied",
            RuntimeError::InvalidFence(_) => "InvalidFence",
            RuntimeError::SchedulerStopped => "SchedulerStopped",
            RuntimeError::SequenceExhausted => "SequenceExhausted",
        };
        let qualified_cause = format!("SafetyFailed cause={cause_kind}: {original}");
        self.status.message = qualified_cause.clone();
        match self
            .finish_terminal(
                status_tx,
                TerminalOutcome::SafetyFailed,
                SchedulerPhase::Failed,
            )
            .await
        {
            Ok(()) => {
                self.status.message = qualified_cause;
                self.publish_status(status_tx);
                original
            }
            Err(RuntimeError::StopUnconfirmed(reason)) => RuntimeError::StopUnconfirmed(format!("{original}; {reason}")),
            Err(fence_error) => RuntimeError::Motion(format!(
                "{original}; explicit fence also failed: {fence_error}"
            )),
        }
    }

    async fn finish_terminal(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        terminal: TerminalOutcome,
        phase_after: SchedulerPhase,
    ) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.finish_terminal", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.finish_terminal.enter", self.status.pose_source_timestamp_ns, None, 0);
        // 上层可能只消费首次终态；先保留真实原因，不能等围栏完成后再补发。
        let failure_reason = matches!(terminal, TerminalOutcome::PlannerFailed | TerminalOutcome::SafetyFailed)
            .then(|| self.status.message.clone());
        let arrived_identity = self.current_execution_identity.take();
        self.local_intent_sequence = 0;
        {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.finish_terminal.planner_stop", self.status.pose_source_timestamp_ns);
            self.planner.stop();
        }
        let had_lease = self.lease_active;
        if terminal == TerminalOutcome::Succeeded && had_lease {
            self.path_wait_started_at.get_or_insert_with(Instant::now);
        }
        let mut verified_fence = self.arrival_confirmation.take().map(|pending| pending.proof);
        if had_lease {
            if self.status.phase != SchedulerPhase::Cancelling {
                self.status.phase = SchedulerPhase::Fencing;
                self.status.message = format!("waiting for {terminal:?} fence proof");
                self.publish_status(status_tx);
            }
            let operation_id = self.next_operation_id("fence")?;
            #[cfg(feature = "surface-recovery-qualification")]
            self.qualification_snapshot("scheduler.fence.enter", self.status.pose_source_timestamp_ns, None, 0);
            let outcome = {
                #[cfg(feature = "surface-recovery-qualification")]
                let _qualification_span = self.qualification_span("scheduler.priority_revoke_and_zero", self.status.pose_source_timestamp_ns);
                self.motion.priority_revoke_and_zero(&operation_id).await
            };
            #[cfg(feature = "surface-recovery-qualification")]
            self.qualification_snapshot(if outcome.is_ok() { "scheduler.fence.return.ok" }
                else { "scheduler.fence.return.error" }, self.status.pose_source_timestamp_ns, None, 0);
            let proof = match outcome {
                Ok(proof) => proof,
                Err(error) => {
                    self.fail_status(status_tx, &error.message);
                    if error.stop_unconfirmed {
                        return Err(RuntimeError::StopUnconfirmed(error.message));
                    }
                    return Err(RuntimeError::Motion(error.message));
                }
            };
            if let Err(error) = validate_fence(&proof) {
                self.fail_status(status_tx, &error.to_string());
                return Err(error);
            }
            self.lease_active = false;
            verified_fence = Some(proof);
        } else if terminal == TerminalOutcome::Shutdown && self.status.last_terminal.is_some() {
            self.status.phase = SchedulerPhase::Idle;
            self.status.message = "scheduler stopped; prior verified terminal retained".into();
            self.publish_status(status_tx);
            return Ok(());
        }

        if terminal == TerminalOutcome::Succeeded && had_lease {
            self.arrival_confirmation = Some(ArrivalConfirmation {
                proof: verified_fence.take().expect("已确认真实停车"),
                confirmed_at: Instant::now(), resume_requested: false,
                execution_identity: arrived_identity,
            });
            self.status.phase = SchedulerPhase::Fencing;
            self.status.last_applied_twist = Some(Twist2::ZERO);
            self.status.message = "已确认停稳，等待停车后采样的有效位姿复核目标".into();
            self.publish_status(status_tx);
            return Ok(());
        }

        {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.finish_terminal.planner_stop", self.status.pose_source_timestamp_ns);
            self.planner.stop();
        }
        self.session = None;
        self.current_path_revision = 0;

        self.current_path = None;
        self.current_execution_identity = None;
        self.local_intent_sequence = 0;
        self.path_wait_started_at = None;
        self.status.phase = phase_after;
        self.status.session_id = None;
        self.status.path_revision = 0;
        self.status.last_terminal = Some(terminal);
        if let Some(proof) = verified_fence {
            self.status.motion_sink_ready = terminal != TerminalOutcome::Shutdown;
            self.status.last_zero_ack_id = Some(proof.zero_ack_id);
            self.status.last_fence_monotonic_ns = proof.applied_monotonic_ns;
            self.status.last_applied_twist = Some(Twist2::ZERO);
            self.status.message = format!("{terminal:?} after verified fence");
        } else {
            self.status.message = format!("{terminal:?} with no active lease");
        }
        if let Some(reason) = failure_reason {
            self.status.message = format!("{reason}; {}", self.status.message);
        }
        if terminal == TerminalOutcome::Shutdown {
            self.status.motion_sink_ready = false;
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn should_renew(&self) -> bool {
        self.lease_active
            && matches!(
                self.status.phase,
                SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling
            )
    }

    fn next_operation_id(&mut self, operation: &str) -> Result<String, RuntimeError> {
        self.operation_sequence = self
            .operation_sequence
            .checked_add(1)
            .ok_or(RuntimeError::SequenceExhausted)?;
        let session_id = self
            .session
            .as_ref()
            .map(|session| session.session_id.as_str())
            .unwrap_or("idle");
        Ok(format!(
            "nav:{session_id}:{operation}:{}",
            self.operation_sequence
        ))
    }

    fn fail_status(&mut self, status_tx: &watch::Sender<SchedulerStatus>, reason: &str) {
        self.status.phase = SchedulerPhase::Failed;
        self.status.motion_sink_ready = false;
        self.status.last_terminal = None;
        self.status.message = reason.to_string();
        self.publish_status(status_tx);
    }

    fn publish_status(&self, status_tx: &watch::Sender<SchedulerStatus>) {
        let mut status = self.status.clone();
        if status.phase == SchedulerPhase::Controlling && status.last_terminal.is_none() {
            status.local_path = self.planner.local_path_for_display();
        } else {
            status.local_path.clear();
        }
        status_tx.send_replace(status);
    }
}

/// Cloneable command handle for a spawned scheduler.
#[derive(Clone)]
pub struct NavSchedulerHandle {
    input_tx: mpsc::Sender<InputCommand>,
    urgent_tx: mpsc::Sender<UrgentCommand>,
    status_rx: watch::Receiver<SchedulerStatus>,
    #[cfg(feature = "surface-recovery-qualification")]
    qualification_timing: Option<qualification_timing::TraceSink>,
}

impl NavSchedulerHandle {
    /// Observer explicitly attached before spawn; None is the silent default.
    #[cfg(feature = "surface-recovery-qualification")]
    pub fn qualification_timing(&self) -> Option<&qualification_timing::TraceSink> {
        self.qualification_timing.as_ref()
    }

    /// Acquire a lease for a new session.
    pub async fn start_session(&self, session: NavSession) -> Result<(), RuntimeError> {
        self.send_input(InputKind::StartSession(session)).await
    }

    /// Submit a correlated, immutable path revision.
    pub async fn submit_path(&self, path: VersionedPath) -> Result<(), RuntimeError> {
        self.send_input(InputKind::SubmitPath(path)).await
    }

    /// 同进程当前观察入口；调用时刻保留至队列消费，传输来源须用采集时刻入口。
    pub async fn update_odometry(&self, odometry: PoseStamped) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, None, Instant::now()))
            .await
    }

    /// 保留来源期限的同进程当前观察入口；传输来源须另提供实际采集时刻。
    pub async fn update_odometry_with_deadline(
        &self,
        odometry: PoseStamped,
        valid_until: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, Some(valid_until), Instant::now()))
            .await
    }

    /// 生产位姿保留实际采集单调时刻，停车复核不能把队列消费当新采样。
    pub async fn update_odometry_with_acquisition(
        &self, odometry: PoseStamped, acquired_at: Instant, valid_until: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, Some(valid_until), acquired_at)).await
    }

    /// Update the planner costmap with its real revision and source timestamp.
    pub async fn update_costmap(
        &self,
        observation: CostmapObservation,
    ) -> Result<(), RuntimeError> {
        self.update_costmap_received(observation, Instant::now())
            .await
    }

    /// Refresh only current obstacle evidence and its admitted source deadline.
    /// Persistent terrain receipts and planning geometry versions are unchanged.
    pub async fn update_obstacle_source(&self,map:Option<ExecutionMap>,available:bool,
        bound:Option<CommandDispatchBound>)->Result<(),RuntimeError> {
        self.send_input(InputKind::UpdateObstacleSource(map,available,bound)).await
    }

    /// 保留实际地图接收时刻，表面构建完成或队列等待不能延长输入有效期。
    pub async fn update_costmap_received(
        &self,
        observation: CostmapObservation,
        received_at: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateCostmap(observation, received_at))
            .await
    }

    /// Cancel the active session and wait for a verified fence.
    pub async fn cancel(&self, session_id: &str) -> Result<(), RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::Cancel {
                session_id: session_id.trim().to_string(),
                reply,
            })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// 明确撤销地图来源；活跃会话等待失败停车围栏，空闲状态不申请运动租约。
    pub async fn invalidate_map(&self, reason: impl Into<String>) -> Result<(), RuntimeError> {
        self.invalidate_map_received(reason, Instant::now()).await
    }

    /// 使用来源失效事件的原接收时刻，不把排队处理时刻当作来源水位。
    pub async fn invalidate_map_received(
        &self,
        reason: impl Into<String>,
        received_at: Instant,
    ) -> Result<(), RuntimeError> {
        if received_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "map.invalidated_at",
                reason: "地图失效事件的接收时刻不能位于未来".into(),
            });
        }
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::InvalidateMap {
                reason: reason.into(),
                invalidated_at: received_at,
                reply,
            })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// Stop the scheduler and fence any active lease.
    pub async fn shutdown(&self) -> Result<(), RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::Shutdown { reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// 规划诊断按目标摘要关联，仅更新等待路径阶段，不能覆盖执行或终态。
    pub async fn report_planning_diagnostic(&self,goal_id:String,key:GoalKey,message:String)->Result<(),RuntimeError> {
        self.send_input(InputKind::PlanningDiagnostic(goal_id,key,message)).await
    }

    /// Read the latest scheduler status.
    #[must_use]
    pub fn status(&self) -> SchedulerStatus {
        self.status_rx.borrow().clone()
    }

    /// Subscribe to scheduler status changes.
    #[must_use]
    pub fn subscribe_status(&self) -> watch::Receiver<SchedulerStatus> {
        self.status_rx.clone()
    }

    async fn send_input(&self, kind: InputKind) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_input = self.qualification_timing.as_ref().map(|sink| {
            let (phase, source) = match &kind {
                InputKind::StartSession(_) => ("scheduler.send.start_session", 0),
                InputKind::SubmitPath(_) => ("scheduler.send.path", 0),
                InputKind::UpdateOdometry(pose, _, _) => ("scheduler.send.pose", pose.stamp.nanos),
                InputKind::UpdateCostmap(map, _) => ("scheduler.send.map", map.source_timestamp_ns),
                InputKind::UpdateObstacleSource(_, _, _) => ("scheduler.send.recent", 0),
                InputKind::PlanningDiagnostic(_, _, _) => ("scheduler.send.planning_diagnostic", 0),
            };
            if let InputKind::UpdateObstacleSource(_, _, bound) = &kind {
                sink.bound("scheduler.input.send.recent_bound", 0, *bound);
            }
            sink.mark("scheduler.input.send.begin", source);
            (sink.clone(), source, sink.span(phase, source))
        });
        let (reply, result) = oneshot::channel();
        self.input_tx
            .send(InputCommand { kind, reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = &qualification_input {
            // Successful send means queued; the actor may already have dequeued it.
            sink.mark("scheduler.input.send.accepted", *source);
        }
        let reply = result.await.map_err(|_| RuntimeError::SchedulerStopped);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = &qualification_input {
            sink.mark(match &reply {
                Ok(Ok(())) => "scheduler.input.reply.ok",
                Ok(Err(_)) => "scheduler.input.reply.error",
                Err(_) => "scheduler.input.reply.closed",
            }, *source);
        }
        reply?
    }
}

/// Spawn one scheduler task and return its command handle and join handle.
#[must_use]
pub fn spawn_scheduler<M, P>(
    scheduler: NavScheduler<M, P>,
) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>)
where
    M: NavMotionPort,
    P: NavPlanner,
{
    #[cfg(feature = "surface-recovery-qualification")]
    let qualification_timing = scheduler.qualification_timing.clone();
    let (input_tx, input_rx) = mpsc::channel(INPUT_CAPACITY);
    let (urgent_tx, urgent_rx) = mpsc::channel(URGENT_CAPACITY);
    let (status_tx, status_rx) = watch::channel(scheduler.status.clone());
    let handle = NavSchedulerHandle {
        input_tx,
        urgent_tx,
        status_rx,
        #[cfg(feature = "surface-recovery-qualification")]
        qualification_timing,
    };
    let task = tokio::spawn(scheduler.run(input_rx, urgent_rx, status_tx));
    (handle, task)
}

/// Connect the production private-UDS guard client and spawn the qualified
/// LocalPlanner scheduler. A successful transport connection makes the motion
/// sink ready; individual sessions still require an admitted guard proof.
pub async fn spawn_production_scheduler(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    spawn_production_scheduler_with_backend(
        socket_path,
        config,
        LocalExecutionBackendKind::BasicInProcess,
    )
    .await
}

/// Connect the production guard and freeze one local execution backend into
/// the scheduler-owned task for its complete lifetime.
pub async fn spawn_production_scheduler_with_backend(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
    backend_kind: LocalExecutionBackendKind,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    validate_config(config)?;
    let backend = production_local_backend(backend_kind)?;
    let motion = ProductionNavMotionPort::connect(production_client_config(socket_path))
        .await
        .map_err(|error| RuntimeError::Motion(error.message))?;
    let scheduler = NavScheduler::new(motion, backend, config)?;
    Ok(spawn_scheduler(scheduler))
}

/// 显式构造三维局部后端，标定非法时先拒绝，不能为失败配置连接运动端。
pub async fn spawn_production_surface_scheduler(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
    surface_config: SurfaceFollowerConfig,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    validate_config(config)?;
    let backend = production_surface_backend(surface_config)?;
    let motion = ProductionNavMotionPort::connect(production_client_config(socket_path))
        .await
        .map_err(|error| RuntimeError::Motion(error.message))?;
    Ok(spawn_scheduler(NavScheduler::new(motion, backend, config)?))
}

fn production_surface_backend(config: SurfaceFollowerConfig) -> Result<planner_worker::PlannerWorker, RuntimeError> {
    planner_worker::PlannerWorker::new(RouteFollower::new(config)?)
}

/// Construct the unchanged production backend for opt-in cross-crate qualification.
#[cfg(feature = "surface-recovery-qualification")]
pub fn qualification_surface_backend(
    config: SurfaceFollowerConfig,
) -> Result<impl NavPlanner, RuntimeError> {
    production_surface_backend(config)
}

struct DeferredControl {
    publication: watch::Receiver<()>,
    deadline: Instant,
    identity: Arc<PathExecutionIdentity>,
    queued_inputs: Option<usize>,
    finish_wait: bool,
}

impl DeferredControl {
    // True ends the wait even if the worker still has no current result. Channel
    // closure must resolve once, not repeatedly subscribe to a closed notifier.
    async fn ready(&mut self) -> bool {
        tokio::select! {
            biased;
            _ = tokio::time::sleep_until(self.deadline) => true,
            changed = self.publication.changed() => changed.is_err(),
        }
    }
}

enum SchedulerEvent { Input(Option<InputCommand>), Tick, Publication(bool), ControlReady }

struct InputCommand {
    kind: InputKind,
    reply: oneshot::Sender<Result<(), RuntimeError>>,
}

enum InputKind {
    StartSession(NavSession),
    SubmitPath(VersionedPath),
    UpdateOdometry(PoseStamped, Option<Instant>, Instant),
    UpdateCostmap(CostmapObservation, Instant),
    UpdateObstacleSource(Option<ExecutionMap>,bool,Option<CommandDispatchBound>),
    PlanningDiagnostic(String, GoalKey, String),
}

enum UrgentCommand {
    Cancel {
        session_id: String,
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
    InvalidateMap {
        reason: String,
        invalidated_at: Instant,
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
    Shutdown {
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
}

fn validate_config(config: SchedulerConfig) -> Result<(), RuntimeError> {
    if config.tick_interval.is_zero() {
        return Err(RuntimeError::InvalidInput {
            field: "tick_interval",
            reason: "must be positive".into(),
        });
    }
    if config.renew_interval.is_zero() || config.renew_interval > MAX_RENEW_INTERVAL {
        return Err(RuntimeError::InvalidInput {
            field: "renew_interval",
            reason: format!("must be in 1..={}ms", MAX_RENEW_INTERVAL.as_millis()),
        });
    }
    if !config.robot_width_m.is_finite() || config.robot_width_m <= 0.0 {
        return Err(RuntimeError::InvalidInput {
            field: "robot_width_m",
            reason: "must be finite and positive".into(),
        });
    }
    Ok(())
}

fn validate_source_timestamp(
    field: &'static str,
    source_timestamp_ns: i64,
    previous: Option<PlannerInputReceipt>,
) -> Result<(), RuntimeError> {
    if source_timestamp_ns <= 0 {
        return Err(RuntimeError::InvalidInput {
            field,
            reason: "must be positive".into(),
        });
    }
    if previous.is_some_and(|receipt| source_timestamp_ns <= receipt.source_timestamp_ns) {
        return Err(RuntimeError::StaleInput(format!(
            "{field}={source_timestamp_ns} did not advance beyond {}",
            previous.map_or(0, |receipt| receipt.source_timestamp_ns),
        )));
    }
    Ok(())
}

fn receipt_age_ms(receipt: Option<PlannerInputReceipt>, now: Instant) -> u64 {
    receipt.map_or(u64::MAX, |receipt| {
        now.duration_since(receipt.received_at)
            .as_millis()
            .min(u128::from(u64::MAX)) as u64
    })
}

fn stale_receipt_reason(
    label: &str,
    receipt: Option<PlannerInputReceipt>,
    now: Instant,
    ttl: Duration,
) -> Option<String> {
    let Some(receipt) = receipt else {
        return Some(format!("{label} has no consumed observation"));
    };
    let age = now.duration_since(receipt.received_at);
    (!ttl.is_zero() && age > ttl).then(|| {
        format!(
            "{label} receipt age={}ms exceeds TTL={}ms (source_timestamp_ns={})",
            age.as_millis(),
            ttl.as_millis(),
            receipt.source_timestamp_ns,
        )
    })
}

fn validate_session(session: &NavSession) -> Result<(), RuntimeError> {
    session.options.validate().map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})?;
    validate_map_pose(&session.goal_pose, "goal_pose")?;
    if session.session_id.trim().is_empty() {
        return Err(RuntimeError::InvalidInput {
            field: "session_id",
            reason: "must not be empty".into(),
        });
    }
    if session.session_id != session.execution_identity.session_id {
        return Err(RuntimeError::InvalidInput {
            field: "execution_identity.session_id",
            reason: "must equal session_id".into(),
        });
    }
    if session.local_owner_runtime_generation == 0 {
        return Err(RuntimeError::InvalidInput {
            field: "local_owner_runtime_generation",
            reason: "must be non-zero".into(),
        });
    }
    if session.motion_admission.guard_generation == 0 {
        return Err(RuntimeError::InvalidInput {
            field: "motion_admission.guard_generation",
            reason: "must be non-zero".into(),
        });
    }
    if session
        .motion_admission
        .guard_boot_nonce
        .iter()
        .all(|byte| *byte == 0)
    {
        return Err(RuntimeError::InvalidInput {
            field: "motion_admission.guard_boot_nonce",
            reason: "must not be all zero".into(),
        });
    }
    if session.execution_identity.motion_guard_generation
        != i64::try_from(session.motion_admission.guard_generation).unwrap_or(i64::MAX)
        || session.execution_identity.motion_guard_boot_nonce
            != session.motion_admission.guard_boot_nonce
        || session.execution_identity.motion_stop_generation
            != i64::try_from(session.motion_admission.motion_stop_generation).unwrap_or(i64::MAX)
    {
        return Err(RuntimeError::InvalidInput {
            field: "execution_identity.motion_admission",
            reason: "must equal the request-level motion admission".into(),
        });
    }
    Ok(())
}

fn validate_path_shape(path: &VersionedPath) -> Result<(), RuntimeError> {
    match &path.path {
        ExecutionPath::Planar(planar) => {
            if planar.frame_id != nuwax_nav_types_messages::NavFrameId::Map
                || planar.points.is_empty()
                || planar
                    .points
                    .iter()
                    .any(|point| !point.x.is_finite() || !point.y.is_finite())
            {
                return Err(RuntimeError::InvalidInput {
                    field: "path.payload",
                    reason: "二维路径必须在map坐标系包含有限坐标".into(),
                });
            }
            if path.path_yaws.as_ref().is_some_and(|yaws| {
                yaws.len() != planar.points.len() || yaws.iter().any(|yaw| !yaw.is_finite())
            }) {
                return Err(RuntimeError::InvalidInput {
                    field: "path.path_yaws",
                    reason: "路径朝向数量必须等于路径点数，且所有朝向必须是有限值".into(),
                });
            }
        }
        ExecutionPath::Surface(surface) => {
            if surface.poses.is_empty() || path.path_yaws.is_some() {
                return Err(RuntimeError::InvalidInput {
                    field: "path.payload",
                    reason: "三维路径必须包含完整姿态，不能叠加二维朝向来源".into(),
                });
            }
            for pose in &surface.poses {
                validate_map_pose(pose, "path.payload")?;
            }
        }
    }
    if path.session_id.trim().is_empty() {
        return Err(RuntimeError::InvalidInput {
            field: "path.session_id",
            reason: "must not be empty".into(),
        });
    }
    if path.session_id != path.execution_identity.session_id {
        return Err(RuntimeError::InvalidInput {
            field: "path.execution_identity.session_id",
            reason: "must equal path.session_id".into(),
        });
    }
    if path.path_revision == 0 || path.path_revision != path.execution_identity.path_revision {
        return Err(RuntimeError::InvalidInput {
            field: "path.revision",
            reason: "must be non-zero and equal the execution identity revision".into(),
        });
    }
    if path.execution_identity.planner_runtime_generation <= 0 {
        return Err(RuntimeError::InvalidInput {
            field: "path.execution_identity.planner_runtime_generation",
            reason: "must be positive".into(),
        });
    }
    Ok(())
}

fn validate_map_pose(pose: &PoseStamped, field: &'static str) -> Result<(), RuntimeError> {
    if pose.frame_id != nuwax_nav_types_messages::NavFrameId::Map
        || ![
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ]
        .into_iter()
        .all(f64::is_finite)
        || !pose.pose.orientation.yaw().is_finite()
    {
        return Err(RuntimeError::InvalidInput {
            field,
            reason: "位姿必须在map坐标系具有有限位置和有效完整四元数".into(),
        });
    }
    Ok(())
}

fn validate_twist(twist: Twist2) -> Result<(), RuntimeError> {
    if !twist.linear.x.is_finite() || !twist.linear.y.is_finite() || !twist.angular_z.is_finite() {
        return Err(RuntimeError::InvalidInput {
            field: "planner.twist",
            reason: "all twist components must be finite".into(),
        });
    }
    Ok(())
}

fn validate_fence(proof: &FenceProof) -> Result<(), RuntimeError> {
    if proof.lease != FenceLeaseState::Idle {
        return Err(RuntimeError::InvalidFence(format!(
            "expected IDLE, got {:?}",
            proof.lease
        )));
    }
    if proof.zero_ack_id.trim().is_empty() || proof.applied_monotonic_ns == 0 {
        return Err(RuntimeError::InvalidFence(
            "terminal ZeroAck is incomplete".into(),
        ));
    }
    Ok(())
}

fn path_rejected(reason: impl Into<String>) -> RuntimeError {
    RuntimeError::PathRejected {
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    include!("scheduler_delivery_gap_tests.rs");
    include!("arrival_confirmation_tests.rs");
    include!("recent_source_health_tests.rs");
    include!("scheduler_detour_cancellation_tests.rs");
    use std::collections::HashMap;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::{Arc, Mutex};

    use nuwax_motion_safety_guard::wire::{
        motion_guard_request, motion_guard_response, AckCode, AcquireLeaseResponse, GuardResult,
        LeaseState, LeaseTuple, MotionGuardRequest, MotionGuardResponse, RenewLeaseResponse,
        RevokeAndZeroResponse, SubmitMotionCommandResponse, ZeroAck, API_VERSION,
    };
    use nuwax_nav_astar::navigation_map::CostMap2D;
    use nuwax_nav_types_messages::{NavFrameId, Point2, Pose3, Timestamp};
    use prost::Message;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::{UnixListener, UnixStream};
    use tokio::sync::mpsc as tokio_mpsc;

    use super::*;

    #[test]
    fn production_surface_worker_accepts_legacy_reverse_authorization_as_an_upper_bound() {
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.basic_kinematics = Some(nuwax_nav_types_messages::MotionKinematics {
            model:nuwax_nav_types_messages::MotionModel::Omnidirectional,
            max_forward_mps:0.35,max_reverse_mps:0.35,max_lateral_mps:0.35,
            max_angular_rps:0.8,minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        });
        assert!(config.motion_limits.is_none());
        let mut backend = production_surface_backend(config).unwrap();
        let goal = PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),pose:Pose3::default()};
        backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {allow_reverse:true,..Default::default()}).unwrap();
    }

    fn physical_omni_limits() -> nuwax_nav_types_messages::MotionLimits {
        nuwax_nav_types_messages::MotionLimits {
            model:nuwax_nav_types_messages::MotionModel::Omnidirectional,
            max_forward_mps:1.0,max_reverse_mps:1.0,max_lateral_mps:1.0,max_angular_rps:0.8,
            linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,minimum_turning_radius_m:0.0,
            can_rotate_in_place:true,
        }
    }

    #[test]
    fn production_basic_old_oriented_paths_cannot_emit_sideways_or_reverse() {
        for (x,y) in [(2.0,3.0),(1.0,2.0)] {
            let mut backend=BasicInProcessLocalBackend::production();
            backend.set_navigation_motion_limits(physical_omni_limits()).unwrap();
            backend.update_costmap(CostMap2D::new_all_free("map",80,80,0.1));
            let mut pose=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0,2.0,0.0);
            let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",x,y,0.0);
            backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {
                allow_reverse:true,position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,
            }).unwrap();
            backend.replace_path_with_orientations(Path2D::stamped(NavFrameId::Map,Timestamp::default(),
                vec![Point2::new(2.0,2.0),Point2::new(x,y)]),Some(vec![0.0,0.0]));
            for tick in 0..20 {
                pose.stamp=Timestamp::from_unix(1+tick/10,(tick%10)as u32*100_000_000);
                backend.update_odometry(pose.clone());
                match backend.step(0.3) {
                    PlannerStep::Command(command)=>{
                        assert!(command.linear.x>=0.0,"生产二维后端不能按旧授权倒退：{command:?}");
                        assert_eq!(command.linear.y,0.0,"生产二维后端不能沿旧全向路径横移：{command:?}");
                    },
                    PlannerStep::ReplanRequired(_)=>break,
                    other=>panic!("旧全向路径应保持前向或请求新路径：{other:?}"),
                }
            }
        }
    }

    #[test]
    fn production_basic_forward_paths_turn_and_reach_side_and_rear_goals() {
        for heading in [std::f64::consts::FRAC_PI_2,std::f64::consts::PI] {
            let mut backend=BasicInProcessLocalBackend::production();
            let physical=physical_omni_limits();
            backend.set_navigation_motion_limits(physical.clone()).unwrap();
            let grid=CostMap2D::new_all_free("map",80,80,0.1);
            backend.update_costmap(grid.clone());
            let mut pose=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0,2.0,0.0);
            let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0+heading.cos(),2.0+heading.sin(),heading);
            backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {
                allow_reverse:true,position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,
            }).unwrap();
            backend.replace_path_with_orientations(Path2D::stamped(NavFrameId::Map,Timestamp::default(),
                vec![Point2::new(2.0,2.0),Point2::new(2.0,2.0),goal.pose.position.to_2d()]),
                Some(vec![0.0,heading,heading]));
            let dt=1.0/LocalPlannerConfig::default_config().control_frequency;
            let mut applied=Twist2::ZERO;let mut reached=false;let mut done=false;
            let mut first_motion=None;
            for tick in 0..500 {
                pose.stamp=Timestamp::from_unix(1,0);pose.stamp.nanos+=(tick as f64*dt*1e9)as i64;
                backend.update_odometry(pose.clone());
                if tick%10==0 {backend.update_costmap(grid.clone());}
                let command=if reached {Twist2::ZERO} else {
                    match backend.step(0.3) {
                        PlannerStep::Command(command)=>command,
                        PlannerStep::Succeeded=>{reached=true;Twist2::ZERO},
                        other=>panic!("二维前向路径必须连续闭环：方向{heading}，拍{tick}，{other:?}"),
                    }
                };
                assert!(command.linear.x>=0.0,"生产二维命令不能倒退：{command:?}");
                assert_eq!(command.linear.y,0.0,"生产二维命令不能横移：{command:?}");
                if first_motion.is_none() && command.linear.x.hypot(command.linear.y)+command.angular_z.abs()>1e-9 {
                    first_motion=Some(command);assert_eq!(command.linear.x,0.0,"侧后方目标必须先原地转向");
                }
                let next=Twist2 {
                    linear:Point2::new(command.linear.x.clamp(applied.linear.x-physical.linear_acceleration_mps2*dt,
                        applied.linear.x+physical.linear_acceleration_mps2*dt),0.0),
                    angular_z:command.angular_z.clamp(applied.angular_z-physical.angular_acceleration_rps2*dt,
                        applied.angular_z+physical.angular_acceleration_rps2*dt),
                };
                let v=(applied.linear.x+next.linear.x)*0.5;let w=(applied.angular_z+next.angular_z)*0.5;
                let yaw=pose.pose.orientation.yaw();let end_yaw=yaw+w*dt;
                if w.abs()<1e-12 {pose.pose.position.x+=v*yaw.cos()*dt;pose.pose.position.y+=v*yaw.sin()*dt;}
                else {pose.pose.position.x+=v*(end_yaw.sin()-yaw.sin())/w;pose.pose.position.y+=v*(yaw.cos()-end_yaw.cos())/w;}
                pose.pose.orientation=nuwax_nav_types_messages::Quaternion::from_yaw(end_yaw);
                applied=next;backend.observe_applied_command(applied);
                assert!(applied.linear.x>=0.0);assert_eq!(applied.linear.y,0.0);
                if reached && applied.linear.x<1e-12 && applied.angular_z.abs()<1e-12 {
                    done=true;eprintln!("生产二维前向闭环：方向{heading:.3}，{}秒，零横移/倒退",(tick+1)as f64*dt);break;
                }
            }
            assert!(done && first_motion.is_some());
            assert!(pose.pose.position.distance(goal.pose.position)<0.12);
            assert_eq!(physical.max_lateral_mps,1.0,"物理手动运动声明不改写");
        }
    }

    static NEXT_SOCKET_ID: AtomicU64 = AtomicU64::new(1);

    struct GuardFixture {
        path: PathBuf,
        task: Option<JoinHandle<Vec<&'static str>>>,
    }

    impl GuardFixture {
        async fn finish(mut self) -> Vec<&'static str> {
            let operations = self
                .task
                .take()
                .expect("guard fixture task handle")
                .await
                .expect("guard fixture task");
            let _ = std::fs::remove_file(&self.path);
            operations
        }
    }

    impl Drop for GuardFixture {
        fn drop(&mut self) {
            if let Some(task) = self.task.as_ref() {
                task.abort();
            }
            let _ = std::fs::remove_file(&self.path);
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum DelayedOperation {
        Submit,
        Renew,
        FenceResponse,
    }

    struct PriorityGuardFixture {
        path: PathBuf,
        task: JoinHandle<()>,
        delayed_rx: tokio_mpsc::UnboundedReceiver<&'static str>,
    }

    impl Drop for PriorityGuardFixture {
        fn drop(&mut self) {
            self.task.abort();
            let _ = std::fs::remove_file(&self.path);
        }
    }

    fn socket_path(label: &str) -> PathBuf {
        let id = NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "nuwax-nav-runtime-{label}-{}-{id}.sock",
            std::process::id()
        ))
    }

    async fn spawn_scripted_guard(label: &str) -> GuardFixture {
        let path = socket_path(label);
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path).expect("bind guard fixture UDS");
        let task = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.expect("accept guard client");
            let mut operations = Vec::new();
            for _ in 0..3 {
                let request = read_guard_request(&mut stream).await;
                let (operation, response) = scripted_guard_response(&request);
                operations.push(operation);
                write_guard_response(&mut stream, response).await;
            }
            operations
        });
        GuardFixture {
            path,
            task: Some(task),
        }
    }

    async fn spawn_priority_guard(label: &str, delayed: DelayedOperation) -> PriorityGuardFixture {
        let path = socket_path(label);
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path).expect("bind priority guard fixture UDS");
        let (delayed_tx, delayed_rx) = tokio_mpsc::unbounded_channel();
        let seen_fences = Arc::new(Mutex::new(HashMap::new()));
        let task = tokio::spawn(async move {
            loop {
                let Ok((stream, _)) = listener.accept().await else {
                    return;
                };
                let delayed_tx = delayed_tx.clone();
                let seen_fences = Arc::clone(&seen_fences);
                tokio::spawn(handle_priority_guard_connection(
                    stream,
                    delayed,
                    delayed_tx,
                    seen_fences,
                ));
            }
        });
        PriorityGuardFixture {
            path,
            task,
            delayed_rx,
        }
    }

    async fn handle_priority_guard_connection(
        mut stream: UnixStream,
        delayed: DelayedOperation,
        delayed_tx: tokio_mpsc::UnboundedSender<&'static str>,
        seen_fences: Arc<Mutex<HashMap<String, Vec<u8>>>>,
    ) {
        while let Some(request) = try_read_guard_request(&mut stream).await {
            let (operation, response) = scripted_guard_response(&request);
            let must_delay = matches!(
                (delayed, operation),
                (DelayedOperation::Submit, "applied") | (DelayedOperation::Renew, "renew")
            );
            if must_delay {
                let _ = delayed_tx.send(operation);
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
            if delayed == DelayedOperation::FenceResponse && operation == "fence" {
                let operation_id = match request.operation.as_ref() {
                    Some(motion_guard_request::Operation::RevokeAndZero(body)) => body
                        .envelope
                        .as_ref()
                        .map(|envelope| envelope.operation_id.clone())
                        .unwrap_or_default(),
                    _ => String::new(),
                };
                let request_bytes = request.encode_to_vec();
                let first_attempt = {
                    let mut seen = seen_fences.lock().expect("seen fence operations");
                    if let Some(first_bytes) = seen.get(&operation_id) {
                        assert_eq!(
                            first_bytes, &request_bytes,
                            "priority fence retry must replay identical sealed bytes"
                        );
                        false
                    } else {
                        seen.insert(operation_id, request_bytes);
                        true
                    }
                };
                if first_attempt {
                    let _ = delayed_tx.send("fence");
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    return;
                }
            }
            if !try_write_guard_response(&mut stream, response).await {
                return;
            }
        }
    }

    async fn try_read_guard_request(stream: &mut UnixStream) -> Option<MotionGuardRequest> {
        let mut prefix = [0_u8; 4];
        stream.read_exact(&mut prefix).await.ok()?;
        let length = u32::from_be_bytes(prefix) as usize;
        if length == 0 || length > MAX_FRAME_BYTES {
            return None;
        }
        let mut payload = vec![0_u8; length];
        stream.read_exact(&mut payload).await.ok()?;
        MotionGuardRequest::decode(payload.as_slice()).ok()
    }

    async fn try_write_guard_response(
        stream: &mut UnixStream,
        response: MotionGuardResponse,
    ) -> bool {
        let payload = response.encode_to_vec();
        stream
            .write_all(&(payload.len() as u32).to_be_bytes())
            .await
            .is_ok()
            && stream.write_all(&payload).await.is_ok()
            && stream.flush().await.is_ok()
    }

    async fn read_guard_request(stream: &mut UnixStream) -> MotionGuardRequest {
        let mut prefix = [0_u8; 4];
        stream
            .read_exact(&mut prefix)
            .await
            .expect("read guard request prefix");
        let length = u32::from_be_bytes(prefix) as usize;
        assert!(length > 0 && length <= MAX_FRAME_BYTES);
        let mut payload = vec![0_u8; length];
        stream
            .read_exact(&mut payload)
            .await
            .expect("read guard request body");
        MotionGuardRequest::decode(payload.as_slice()).expect("decode guard request")
    }

    async fn write_guard_response(stream: &mut UnixStream, response: MotionGuardResponse) {
        let payload = response.encode_to_vec();
        stream
            .write_all(&(payload.len() as u32).to_be_bytes())
            .await
            .expect("write guard response prefix");
        stream
            .write_all(&payload)
            .await
            .expect("write guard response body");
        stream.flush().await.expect("flush guard response");
    }

    fn guard_result(ack: AckCode, lease: LeaseState) -> GuardResult {
        GuardResult {
            ack: ack as i32,
            lease: lease as i32,
            error: None,
        }
    }

    fn scripted_guard_response(
        request: &MotionGuardRequest,
    ) -> (&'static str, MotionGuardResponse) {
        let (operation, outcome) = match request.operation.as_ref().expect("request operation") {
            motion_guard_request::Operation::AcquireLease(body) => {
                let envelope = body.envelope.as_ref().expect("acquire envelope");
                assert_eq!(envelope.motion_stop_generation, 1);
                let admitted = envelope.tuple.as_ref().expect("acquire admission tuple");
                (
                    "acquire",
                    motion_guard_response::Outcome::AcquireLease(AcquireLeaseResponse {
                        result: Some(guard_result(AckCode::Received, LeaseState::Active)),
                        lease_tuple: Some(LeaseTuple {
                            generation: admitted.generation,
                            boot_nonce: admitted.boot_nonce.clone(),
                            lease_id: 41,
                        }),
                        lease_deadline_monotonic_ns: u64::MAX,
                        owner_deadline_monotonic_ns: u64::MAX - 1,
                    }),
                )
            }
            motion_guard_request::Operation::RenewLease(_) => (
                "renew",
                motion_guard_response::Outcome::RenewLease(RenewLeaseResponse {
                    result: Some(guard_result(AckCode::Received, LeaseState::Active)),
                    lease_deadline_monotonic_ns: u64::MAX,
                    owner_deadline_monotonic_ns: u64::MAX - 1,
                }),
            ),
            motion_guard_request::Operation::SubmitMotionCommand(_) => (
                "applied",
                motion_guard_response::Outcome::SubmitMotionCommand(SubmitMotionCommandResponse {
                    result: Some(guard_result(AckCode::Applied, LeaseState::Active)),
                    applied_monotonic_ns: 101,
                    zero_ack: None,
                }),
            ),
            motion_guard_request::Operation::RevokeAndZero(_) => (
                "fence",
                motion_guard_response::Outcome::RevokeAndZero(RevokeAndZeroResponse {
                    result: Some(guard_result(AckCode::ResourceFenced, LeaseState::Idle)),
                    zero_ack: Some(ZeroAck {
                        zero_ack_id: "zero-uds-1".into(),
                        applied_monotonic_ns: 102,
                        ..Default::default()
                    }),
                }),
            ),
            operation => panic!("unexpected guard operation: {operation:?}"),
        };
        (
            operation,
            MotionGuardResponse {
                api_version: API_VERSION,
                request_id: request.request_id.clone(),
                outcome: Some(outcome),
            },
        )
    }

    #[derive(Debug, Default)]
    struct FakeMotionState {
        calls: Vec<String>,
        fail_twist: bool,
        unconfirmed_fences: usize,
    }

    #[derive(Clone)]
    struct FakeMotion(Arc<Mutex<FakeMotionState>>);

    #[async_trait]
    impl NavMotionPort for FakeMotion {
        async fn acquire(
            &mut self,
            operation_id: &str,
            admission: &NavMotionAdmission,
        ) -> Result<LeaseProof, MotionPortError> {
            self.0
                .lock()
                .expect("fake motion state")
                .calls
                .push(format!("acquire:{operation_id}"));
            Ok(LeaseProof {
                lease_id: 11,
                guard_generation: admission.guard_generation,
            })
        }

        async fn renew(&mut self, operation_id: &str) -> Result<(), MotionPortError> {
            self.0
                .lock()
                .expect("fake motion state")
                .calls
                .push(format!("renew:{operation_id}"));
            Ok(())
        }

        async fn submit_twist(
            &mut self,
            operation_id: &str,
            _twist: Twist2,
        ) -> Result<AppliedProof, MotionPortError> {
            let mut state = self.0.lock().expect("fake motion state");
            state.calls.push(format!("twist:{operation_id}"));
            if state.fail_twist {
                return Err(MotionPortError::new("injected APPLIED timeout"));
            }
            Ok(AppliedProof {
                operation_id: operation_id.into(),
                applied_monotonic_ns: 99,
            })
        }

        async fn revoke_and_zero(
            &mut self,
            operation_id: &str,
        ) -> Result<FenceProof, MotionPortError> {
            let mut state = self.0.lock().expect("fake motion state");
            state.calls.push(format!("fence:{operation_id}"));
            if state.unconfirmed_fences > 0 {
                state.unconfirmed_fences -= 1;
                return Err(MotionPortError::stop_unconfirmed("等待实际停稳"));
            }
            Ok(FenceProof {
                zero_ack_id: "zero-ack".into(),
                applied_monotonic_ns: 100,
                lease: FenceLeaseState::Idle,
            })
        }
    }

    #[derive(Clone)]
    struct FakePlanner {
        state: Arc<Mutex<FakePlannerState>>,
    }

    #[derive(Debug)]
    struct FakePlannerState {
        step: PlannerStep,
        wait_reason: Option<String>,
        costmap_updates: usize,
        odometry_updates: usize,
        stops: usize,
    }

    impl FakePlanner {
        fn commanding() -> Self {
            Self {
                state: Arc::new(Mutex::new(FakePlannerState {
                    step: PlannerStep::Command(Twist2 {
                        linear: Point2::new(0.2, 0.0),
                        angular_z: 0.1,
                    }),
                    wait_reason: None,
                    costmap_updates: 0,
                    odometry_updates: 0,
                    stops: 0,
                })),
            }
        }
    }

    impl NavPlanner for FakePlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {
            self.state
                .lock()
                .expect("fake planner state")
                .odometry_updates += 1;
        }

        fn update_costmap(&mut self, _costmap: CostMap2D) {
            self.state
                .lock()
                .expect("fake planner state")
                .costmap_updates += 1;
        }

        fn step(&mut self, _robot_width_m: f64) -> PlannerStep {
            self.state.lock().expect("fake planner state").step.clone()
        }

        fn execution_wait_reason(&self) -> Option<String> {
            self.state.lock().expect("fake planner state").wait_reason.clone()
        }

        fn stop(&mut self) {
            self.state.lock().expect("fake planner state").stops += 1;
        }
    }

    fn test_scheduler(
        motion: FakeMotion,
        planner: FakePlanner,
    ) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>) {
        test_scheduler_with_ttls(
            motion,
            planner,
            Duration::from_secs(1),
            Duration::from_secs(1),
        )
    }

    fn test_scheduler_with_ttls(
        motion: FakeMotion,
        planner: FakePlanner,
        pose_ttl: Duration,
        costmap_ttl: Duration,
    ) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>) {
        let scheduler = NavScheduler::new(
            motion,
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                renew_interval: Duration::from_millis(20),
                path_wait_timeout: Duration::from_secs(1),
                robot_width_m: 0.3,
                pose_ttl,
                costmap_ttl,
            },
        )
        .expect("test scheduler");
        spawn_scheduler(scheduler)
    }

    #[tokio::test]
    async fn planning_reason_survives_pose_updates_and_rejects_old_goal_reports() {
        let motion=FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let (handle,task)=spawn_scheduler(NavScheduler::new(motion,FakePlanner::commanding(),SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),execution_identity_for("goal-1",0).goal_key(),"等待路径：起点支撑不足".into()).await.unwrap();
        handle.update_odometry(map_pose_at(2)).await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),GoalKey {producer_runtime_generation:9,goal_generation:1},"旧目标的错误".into()).await.unwrap();
        assert_eq!(handle.status().message,"等待路径：起点支撑不足");
        handle.cancel("goal-1").await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),execution_identity_for("goal-1",0).goal_key(),"晚到报告".into()).await.unwrap();
        assert_eq!(handle.status().last_terminal,Some(TerminalOutcome::Cancelled));
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn slow_geometry_keeps_renewing_and_cancellation_does_not_wait_for_compute() {
        struct Slow {entered:Arc<std::sync::atomic::AtomicBool>}
        impl NavPlanner for Slow {
            fn replace_path(&mut self,_:Path2D) {}
            fn update_odometry(&mut self,_:PoseStamped) {}
            fn update_costmap(&mut self,_:CostMap2D) {}
            fn step(&mut self,_:f64)->PlannerStep {
                self.entered.store(true,Ordering::SeqCst);
                std::thread::sleep(Duration::from_millis(700));
                PlannerStep::Command(Twist2 {linear:Point2::new(0.2,0.0),angular_z:0.0})
            }
            fn stop(&mut self) {}
        }
        let entered=Arc::new(std::sync::atomic::AtomicBool::new(false));
        let motion=FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let calls=motion.0.clone();
        let worker=planner_worker::PlannerWorker::new(Slow {entered:entered.clone()}).unwrap();
        let (handle,task)=spawn_scheduler(NavScheduler::new(motion,worker,SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();
        handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1),async {
            while !entered.load(Ordering::SeqCst) {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.unwrap();
        tokio::time::sleep(Duration::from_millis(520)).await;
        assert!(calls.lock().unwrap().calls.iter().filter(|call|call.starts_with("renew:")).count()>=2);
        assert!(handle.status().message.contains("局部轨迹正在计算"),
            "零速执行回执不能掩盖仍在等待局部计算的阶段");
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        tokio::time::timeout(Duration::from_millis(100),handle.cancel("goal-1")).await
            .expect("取消不能等待700毫秒局部计算").unwrap();
        assert_eq!(handle.status().last_terminal,Some(TerminalOutcome::Cancelled));
        assert_ne!(handle.status().last_applied_twist.map(|t|t.linear.x),Some(0.2));
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn slow_local_blockage_reaches_scheduler_replanning_state() {
        struct Blocked;
        impl NavPlanner for Blocked {
            fn replace_path(&mut self, _:Path2D) {}
            fn update_odometry(&mut self, _:PoseStamped) {}
            fn update_costmap(&mut self, _:CostMap2D) {}
            fn step(&mut self, _:f64) -> PlannerStep {
                std::thread::sleep(DEFAULT_CONTROL_HORIZON + Duration::from_millis(50));
                PlannerStep::ReplanRequired("完整机体扫掠受阻".into())
            }
            fn stop(&mut self) {}
        }
        let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let worker = planner_worker::PlannerWorker::new(Blocked).unwrap();
        let (handle, task) = spawn_scheduler(NavScheduler::new(motion, worker, SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();
        handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while handle.status().phase != SchedulerPhase::AwaitingPath
                || !handle.status().message.contains("完整机体扫掠受阻")
            {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("慢失败必须沿生产调度器触发停车重规划，不能无限发零速");
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        handle.cancel("goal-1").await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    fn execution_identity_for(session_id: &str, path_revision: u64) -> PathExecutionIdentity {
        PathExecutionIdentity {
            session_id: session_id.into(),
            owner_kind: 1,
            parent_request_id: format!("request-{session_id}"),
            goal_generation: 1,
            producer: "test-producer".into(),
            producer_epoch: 1,
            producer_runtime_generation: 1,
            committed_map_session_id: "map-session".into(),
            committed_map_session_generation: 1,
            committed_map_revision: 1,
            committed_map_boot_nonce: [3; 16],
            committed_map_commit_sequence: 1,
            committed_map_source_timestamp_ns: 1,
            committed_map_frame_id: "map".into(),
            boundary_id: String::new(),

            motion_guard_generation: 7,
            motion_guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
            planner_runtime_generation: i64::from(path_revision != 0),
            path_revision,

        }
    }

    fn session_for(session_id: &str) -> NavSession {
        NavSession {
            boundary: Vec::new(),
            options: Default::default(),
            session_id: session_id.into(),
            execution_identity: execution_identity_for(session_id, 0),
            local_owner_runtime_generation: 1,
            motion_admission: NavMotionAdmission {
                guard_generation: 7,
                guard_boot_nonce: [7; 16],
                motion_stop_generation: 1,
            },
            goal_pose: nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0),
        }
    }

    fn session() -> NavSession {
        session_for("goal-1")
    }

    fn path_for(session_id: &str, path_revision: u64) -> VersionedPath {
        VersionedPath {
            session_id: session_id.into(),
            execution_identity: execution_identity_for(session_id, path_revision),
            path_revision,

            path_yaws: None,
            path: ExecutionPath::Planar(Path2D::stamped(
                NavFrameId::Map,
                Timestamp::default(),
                vec![Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)],
            )),
        }
    }

    fn path() -> VersionedPath {
        path_for("goal-1", 1)
    }

    fn map_pose_at(seconds: i64) -> PoseStamped {
        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        // 测试输入表示真实有效姿态，不能用全零四元数冒充单位旋转。
        pose.pose.orientation = nuwax_nav_types_messages::Quaternion::IDENTITY;
        pose.stamp = Timestamp::from_unix(seconds, 0);
        pose
    }

    fn map_pose() -> PoseStamped {
        map_pose_at(1)
    }

    fn costmap_at(source_timestamp_ns: i64) -> CostmapObservation {
        CostmapObservation {
            costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 40, 40, 0.1)),
            source_timestamp_ns,
        }
    }

    async fn wait_for_sink(handle: &NavSchedulerHandle) -> SchedulerStatus {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.motion_sink_ready {
                    break snapshot;
                }
                status.changed().await.expect("scheduler status open");
            }
        })
        .await
        .expect("motion sink readiness timeout")
    }

    async fn wait_for_terminal(
        handle: &NavSchedulerHandle,
        expected: TerminalOutcome,
    ) -> SchedulerStatus {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.last_terminal == Some(expected) {
                    break snapshot;
                }
                status.changed().await.expect("scheduler status open");
            }
        })
        .await
        .expect("terminal status timeout")
    }

    #[tokio::test]
    async fn server_deadline_renewal_keeps_legal_slow_commands_alive() {
        let socket = socket_path("renew-slow-commands");
        let listener = UnixListener::bind(&socket).expect("测试守卫套接字");
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        let task = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.expect("接入客户端");
            let mut operations = Vec::new();
            let mut fence_at = Instant::now() + Duration::from_secs(1);
            let mut applied = 0;
            loop {
                let request = tokio::time::timeout_at(fence_at, read_guard_request(&mut stream))
                    .await.expect("必须在325毫秒看门狗前续租");
                let (operation, mut response) = scripted_guard_response(&request);
                operations.push(operation);
                let now = Instant::now();
                match response.outcome.as_mut().expect("响应") {
                    motion_guard_response::Outcome::AcquireLease(body) => {
                        let Some(motion_guard_request::Operation::AcquireLease(request)) = request.operation.as_ref() else { unreachable!() };
                        let issued = request.envelope.as_ref().expect("信封").issued_monotonic_ns;
                        body.owner_deadline_monotonic_ns = issued + 750_000_000;
                        fence_at = now + Duration::from_millis(325);
                        // 响应传输耗时必须扣入余量，不得重新从接收时刻计算许可。
                        tokio::time::sleep(Duration::from_millis(80)).await;
                    }
                    motion_guard_response::Outcome::RenewLease(body) => {
                        let Some(motion_guard_request::Operation::RenewLease(request)) = request.operation.as_ref() else { unreachable!() };
                        let issued = request.envelope.as_ref().expect("信封").issued_monotonic_ns;
                        body.owner_deadline_monotonic_ns = issued + 750_000_000;
                        fence_at = now + Duration::from_millis(325);
                    }
                    motion_guard_response::Outcome::SubmitMotionCommand(_) => {
                        tokio::time::sleep(Duration::from_millis(200)).await;
                        assert!(Instant::now() < fence_at, "合法慢SDK不能触发失联停车");
                        applied += 1;
                        if applied == 3 {
                            planner_state.lock().expect("规划器").step = PlannerStep::Succeeded;
                        }
                    }
                    _ => {}
                }
                write_guard_response(&mut stream, response).await;
                if operation == "fence" { break; }
            }
            operations
        });
        let fixture = GuardFixture { path: socket.clone(), task: Some(task) };
        let client = MotionGuardClient::connect_with_config(production_client_config(socket)).await.expect("客户端");
        let scheduler = NavScheduler::new(client, planner, SchedulerConfig {
            tick_interval: Duration::from_millis(150), ..SchedulerConfig::default()
        }).expect("调度器");
        let (handle, task) = spawn_scheduler(scheduler);
        handle.start_session(session()).await.expect("会话");
        tokio::time::sleep(Duration::from_millis(400)).await;
        assert_eq!(handle.status().phase, SchedulerPhase::AwaitingPath);
        handle.update_costmap(CostmapObservation {
            costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
            source_timestamp_ns: 1_000_000_000,
        }).await.expect("地图");
        handle.update_odometry(map_pose()).await.expect("位姿");
        handle.submit_path(path()).await.expect("路径");
        publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(3), async {
            while status.borrow().last_terminal != Some(TerminalOutcome::Succeeded) {
                status.changed().await.expect("状态流");
            }
        }).await.expect("三次慢动作完成后真实停车");
        handle.shutdown().await.expect("关闭");
        task.await.expect("调度拥有方").expect("调度结果");
        let operations = fixture.finish().await;
        assert_eq!(operations.iter().filter(|operation| **operation == "applied").count(), 3);
        assert!(operations.iter().filter(|operation| **operation == "renew").count() >= 4);
    }

    #[tokio::test]
    async fn scheduler_acquires_directly_from_request_admission_without_readiness_query() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) =
            test_scheduler(FakeMotion(motion_state.clone()), FakePlanner::commanding());

        let ready = wait_for_sink(&handle).await;
        assert!(ready.motion_sink_ready);
        assert_eq!(ready.guard_generation, 0);

        handle
            .start_session(session())
            .await
            .expect("admitted request accepts session");
        let calls = motion_state.lock().expect("motion state").calls.clone();
        assert_eq!(calls.len(), 1);
        assert!(calls[0].starts_with("acquire:"));
        assert_eq!(handle.status().guard_generation, 7);
        handle
            .cancel("goal-1")
            .await
            .expect("verified cancel fence");
        handle.shutdown().await.expect("scheduler shutdown");
        task.await
            .expect("scheduler task join")
            .expect("scheduler task result");
    }

    #[tokio::test]
    async fn controller_consumes_map_inputs_and_only_reports_applied_motion() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        let (handle, task) = test_scheduler(FakeMotion(motion_state.clone()), planner);
        wait_for_sink(&handle).await;

        handle.start_session(session()).await.expect("session");
        handle
            .update_costmap(CostmapObservation {
                costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("costmap");
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle.submit_path(path()).await.expect("versioned path");

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.status().applied_commands > 0 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("controller APPLIED timeout");
        let status = handle.status();
        assert!(status.last_applied_twist.is_some());
        assert_eq!(planner_state.lock().expect("planner").costmap_updates, 1);
        assert_eq!(planner_state.lock().expect("planner").odometry_updates, 1);

        handle.cancel("goal-1").await.expect("cancel waits ZEROED");
        let terminal = handle.status();
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(terminal.session_id, None);
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(planner_state.lock().expect("planner").stops, 3);
        let calls = motion_state.lock().expect("motion").calls.clone();
        assert!(calls.iter().any(|call| call.starts_with("twist:")));
        assert!(calls.iter().any(|call| call.starts_with("fence:")));

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn basic_in_process_backend_is_scheduler_owned_and_intent_gated() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let backend = production_local_backend(LocalExecutionBackendKind::BasicInProcess).unwrap();
        assert_eq!(backend.kind(), LocalExecutionBackendKind::BasicInProcess);
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            backend,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                renew_interval: Duration::from_millis(200),
                path_wait_timeout: Duration::from_secs(1),
                robot_width_m: 0.3,
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
            },
        )
        .expect("basic in-process scheduler");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;

        handle.update_odometry(map_pose()).await.expect("map pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        tokio::time::sleep(Duration::from_millis(15)).await;
        assert!(
            motion_state
                .lock()
                .expect("motion")
                .calls
                .iter()
                .all(|call| !call.starts_with("twist:")),
            "the backend cannot bypass scheduler session and path admission"
        );

        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("qualified path");
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.status().applied_commands > 0 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("actual basic planner APPLIED timeout");
        let applied = handle
            .status()
            .last_applied_twist
            .expect("actual basic planner twist");
        assert_ne!(applied, Twist2::ZERO);

        handle.cancel("goal-1").await.expect("verified cancel");
        let terminal = handle.status();
        assert_eq!(terminal.phase, SchedulerPhase::Idle);
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(terminal.session_id, None);
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        let twist_count_after_cancel = motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .filter(|call| call.starts_with("twist:"))
            .count();
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert_eq!(
            motion_state
                .lock()
                .expect("motion")
                .calls
                .iter()
                .filter(|call| call.starts_with("twist:"))
                .count(),
            twist_count_after_cancel,
            "cancelled backend must remain stopped beyond two controller ticks"
        );
        let late = handle
            .submit_path(path())
            .await
            .expect_err("terminal scheduler must reject late path replay");
        assert!(matches!(late, RuntimeError::PathRejected { .. }));

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
        assert!(matches!(
            handle.submit_path(path()).await,
            Err(RuntimeError::SchedulerStopped)
        ));
    }

    #[tokio::test]
    async fn production_backend_waits_for_goal_yaw_and_fences_orientation_mutation() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            production_local_backend(LocalExecutionBackendKind::BasicInProcess).unwrap(),
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .expect("生产局部后端");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        let goal_yaw = std::f64::consts::FRAC_PI_2;
        let mut endpoint = nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0);
        endpoint.stamp = Timestamp::from_unix(1, 0);
        handle.update_odometry(endpoint).await.expect("终点位姿");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("覆盖路径的地图");
        let mut requested=session();
        requested.goal_pose.pose.orientation=nuwax_nav_types_messages::Quaternion::from_yaw(goal_yaw);
        handle.start_session(requested).await.expect("会话");

        let mut oriented_path = path();
        for invalid_yaws in [vec![0.0], vec![0.0, f64::NAN]] {
            oriented_path.path_yaws = Some(invalid_yaws);
            assert!(matches!(
                handle.submit_path(oriented_path.clone()).await,
                Err(RuntimeError::InvalidInput {
                    field: "path.path_yaws",
                    ..
                })
            ));
        }
        oriented_path.path_yaws = Some(vec![0.0, goal_yaw]);
        handle.submit_path(oriented_path).await.expect("带朝向路径");
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = handle.subscribe_status();
            while status.borrow().applied_commands == 0 {
                status.changed().await.expect("控制状态");
            }
        })
        .await
        .expect("实际旋转指令");
        let rotating = handle.status();
        assert_eq!(rotating.last_terminal, None);
        let twist = rotating.last_applied_twist.expect("旋转已执行");
        assert_eq!(twist.linear.x, 0.0);
        assert!(twist.angular_z > 0.0);

        let mut aligned = nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, goal_yaw);
        aligned.stamp = Timestamp::from_unix(2, 0);
        handle.update_odometry(aligned.clone()).await.expect("朝向已到");
        publish_pose_after_confirmed_stop(&handle, aligned).await;
        let terminal = wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(terminal.last_zero_ack_id.is_some());

        handle
            .start_session(session_for("goal-2"))
            .await
            .expect("第二次会话");
        let mut second_path = path_for("goal-2", 1);
        second_path.path_yaws = Some(vec![0.0, 0.0]);
        handle
            .submit_path(second_path.clone())
            .await
            .expect("第二次旋转路径");
        second_path.path_yaws = Some(vec![0.0, goal_yaw]);
        assert!(matches!(
            handle.submit_path(second_path).await,
            Err(RuntimeError::PathMutation { .. })
        ));
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::SafetyFailed)
        );
        handle.shutdown().await.expect("关闭");
        task.await.expect("等待调度器").expect("调度器已退出");
    }

    #[tokio::test]
    async fn same_revision_replay_is_idempotent_but_mutation_fences_motion() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = Arc::clone(&planner.state);
        let (handle, task) = test_scheduler(FakeMotion(Arc::clone(&motion_state)), planner);
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");

        let accepted = path();
        handle
            .submit_path(accepted.clone())
            .await
            .expect("initial path");
        handle
            .submit_path(accepted.clone())
            .await
            .expect("exact same-revision replay is idempotent");
        assert_eq!(handle.status().message, "versioned path accepted");
        let mut resent = accepted.clone();
        if let ExecutionPath::Planar(path) = &mut resent.path {
            path.stamp = Timestamp::from_unix(9, 0);
            path.points[0].y = -0.0;
        }
        handle.submit_path(resent).await.expect("same route with a new send time is idempotent");

        let mut wrong_identity = accepted.clone();
        wrong_identity.session_id = "wrong-session".into();
        wrong_identity.execution_identity.session_id = "wrong-session".into();
        let rejected = handle
            .submit_path(wrong_identity)
            .await
            .expect_err("wrong identity must be rejected without fencing");
        assert!(matches!(rejected, RuntimeError::PathRejected { .. }));
        assert_eq!(handle.status().phase, SchedulerPhase::Controlling);

        let replacement = path_for("goal-1", 2);
        handle
            .submit_path(replacement.clone())
            .await
            .expect("higher revision replacement is accepted");
        assert_eq!(handle.status().path_revision, 2);

        let mut mutation = replacement;
        mutation.path = ExecutionPath::Planar(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(3.0, 0.0), Point2::new(4.0, 0.0)],
        ));
        let error = handle
            .submit_path(mutation)
            .await
            .expect_err("same-revision mutation must fail closed");
        assert!(matches!(error, RuntimeError::PathMutation { .. }));

        let terminal = handle.status();
        assert_eq!(terminal.phase, SchedulerPhase::Failed);
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_eq!(terminal.session_id, None);
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(terminal
            .message
            .starts_with("SafetyFailed cause=PathMutation:"));
        assert!(terminal.message.contains("path_revision=2"));
        let terminal_cause = terminal.message.clone();
        assert_eq!(planner_state.lock().expect("planner").stops, 3);

        let twist_count_after_fence = motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .filter(|call| call.starts_with("twist:"))
            .count();
        tokio::time::sleep(Duration::from_millis(25)).await;
        {
            let motion = motion_state.lock().expect("motion");
            assert_eq!(
                motion
                    .calls
                    .iter()
                    .filter(|call| call.starts_with("twist:"))
                    .count(),
                twist_count_after_fence,
                "the old path must not submit motion after its mutation fence"
            );
            assert!(motion.calls.iter().any(|call| call.starts_with("fence:")));
        }

        let late = handle
            .submit_path(path())
            .await
            .expect_err("terminal session must reject a late path replay");
        assert!(matches!(late, RuntimeError::PathRejected { .. }));
        assert_eq!(handle.status().message, terminal_cause);
        assert_eq!(
            handle.status().last_zero_ack_id.as_deref(),
            Some("zero-ack")
        );

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn post_terminal_receipts_preserve_qualified_cause() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("initial pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("initial costmap");
        handle.start_session(session()).await.expect("session");

        let accepted = path_for("goal-1", 1);
        handle
            .submit_path(accepted.clone())
            .await
            .expect("initial path");
        let mut mutation = accepted;
        mutation.path = ExecutionPath::Planar(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(3.0, 0.0), Point2::new(4.0, 0.0)],
        ));
        let error = handle
            .submit_path(mutation)
            .await
            .expect_err("same-revision mutation must fail closed");
        assert!(matches!(error, RuntimeError::PathMutation { .. }));

        let terminal = handle.status();
        assert_eq!(terminal.phase, SchedulerPhase::Failed);
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert!(terminal
            .message
            .starts_with("SafetyFailed cause=PathMutation:"));
        let qualified_cause = terminal.message;

        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("post-terminal pose receipt");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("post-terminal costmap receipt");
        let after_receipts = handle.status();
        assert_eq!(after_receipts.phase, SchedulerPhase::Failed);
        assert_eq!(
            after_receipts.last_terminal,
            Some(TerminalOutcome::SafetyFailed)
        );
        assert_eq!(after_receipts.message, qualified_cause);
        assert_eq!(after_receipts.pose_source_timestamp_ns, 2_000_000_000);
        assert_eq!(after_receipts.costmap_source_timestamp_ns, 2_000_000_000);
        assert!(after_receipts.planner_inputs_fresh);

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn natural_terminal_allows_continuous_goal_a_then_b() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        planner.state.lock().expect("planner").step = PlannerStep::Succeeded;
        let (handle, task) = test_scheduler(FakeMotion(motion_state.clone()), planner);
        wait_for_sink(&handle).await;

        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose A");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap A");
        handle
            .start_session(session_for("goal-a"))
            .await
            .expect("goal A");
        handle
            .submit_path(path_for("goal-a", 1))
            .await
            .expect("path A");
        publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        assert!(!task.is_finished(), "natural terminal must not stop owner");

        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("pose B");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("costmap B");
        handle
            .start_session(session_for("goal-b"))
            .await
            .expect("goal B must not see Busy");
        handle
            .submit_path(path_for("goal-b", 1))
            .await
            .expect("path B");
        publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;

        let fence_count = motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .filter(|call| call.starts_with("fence:"))
            .count();
        assert_eq!(fence_count, 2);
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn planner_failure_can_retry_without_restarting_scheduler_owner() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        planner_state.lock().expect("planner").step = PlannerStep::Failed("blocked".into());
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose 1");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap 1");
        handle
            .start_session(session_for("retry-1"))
            .await
            .expect("first session");
        handle
            .submit_path(path_for("retry-1", 1))
            .await
            .expect("first path");
        let terminal = wait_for_terminal(&handle, TerminalOutcome::PlannerFailed).await;
        assert_eq!(
            terminal.message,
            "PlannerFailed cause=LocalPlanner: blocked"
        );
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(!task.is_finished(), "planner failure must not stop owner");

        planner_state.lock().expect("planner").step = PlannerStep::Succeeded;
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("pose 2");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("costmap 2");
        handle
            .start_session(session_for("retry-2"))
            .await
            .expect("retry session");
        handle
            .submit_path(path_for("retry-2", 1))
            .await
            .expect("retry path");
        publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn replan_confirms_zero_requires_new_revision_and_remains_cancellable() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = Arc::clone(&planner.state);
        planner_state.lock().unwrap().step = PlannerStep::ReplanRequired("连接阻断".into());
        let (handle, task) = test_scheduler(FakeMotion(Arc::clone(&motion_state)), planner);
        handle.update_odometry(map_pose()).await.unwrap();
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        let held = handle.status();
        assert_eq!(held.last_applied_twist, Some(Twist2::ZERO));
        assert!(held.applied_commands > 0);
        assert_eq!(held.last_terminal, None);
        assert!(matches!(
            handle.submit_path(path()).await,
            Err(RuntimeError::PathRejected { .. })
        ));

        planner_state.lock().unwrap().step = PlannerStep::Command(Twist2 {
            linear: Point2::new(0.2, 0.0),
            angular_z: 0.0,
        });
        handle.submit_path(path_for("goal-1", 2)).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().last_applied_twist == Some(Twist2::ZERO) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .expect("更高路径修订恢复执行");
        planner_state.lock().unwrap().step = PlannerStep::ReplanRequired("再次阻断".into());
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        handle.cancel("goal-1").await.unwrap();
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::Cancelled)
        );
        assert!(handle.status().last_zero_ack_id.is_some());
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    async fn wait_for_phase(handle: &NavSchedulerHandle, expected: SchedulerPhase) {
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = handle.subscribe_status();
            while status.borrow().phase != expected {
                status.changed().await.unwrap();
            }
        })
        .await
        .expect("调度阶段等待超时");
    }

    #[tokio::test]
    async fn worker_pending_keeps_original_wait_deadline_goal_and_route_then_times_out() {
        struct BlockedPending {
            release: Option<std::sync::mpsc::Receiver<()>>,
        }
        impl NavPlanner for BlockedPending {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self, _: f64) -> PlannerStep {
                if let Some(release) = self.release.take() { release.recv().unwrap(); }
                PlannerStep::Pending(vec![nuwax_nav_scan::LocalPlanError { body_witness: None,
                    kind: nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
                    reason: "finite search budget".into(), closest_pose: None, blocked: None,
                }])
            }
            fn execution_wait_reason(&self) -> Option<String> { Some("typed search pending".into()) }
            fn stop(&mut self) {}
        }
        let (release, receive) = std::sync::mpsc::channel();
        let worker = crate::planner_worker::PlannerWorker::new(BlockedPending { release: Some(receive) }).unwrap();
        let mut scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))), worker,
            SchedulerConfig { path_wait_timeout: Duration::from_secs(2), ..SchedulerConfig::default() }).unwrap();
        let (sender, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
        scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
        scheduler.start_session(&sender, session()).await.unwrap();
        let original_deadline = scheduler.path_wait_started_at;
        scheduler.submit_path(&sender, path()).await.unwrap();
        let identity = scheduler.current_execution_identity.clone();
        let route = scheduler.current_path.clone();
        // Worker is still computing: this must not be Command(ZERO), which clears the timer.
        scheduler.control_tick(&sender).await.unwrap();
        // Release before assertions so a failed assertion cannot strand the worker on drop.
        release.send(()).unwrap();
        assert_eq!(scheduler.path_wait_started_at, original_deadline);
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            scheduler.control_tick(&sender).await.unwrap();
            assert_eq!(scheduler.path_wait_started_at, original_deadline);
            assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
            assert_eq!(scheduler.current_execution_identity, identity);
            assert_eq!(scheduler.current_path, route);
            assert!(scheduler.session.is_some());
            assert_eq!(scheduler.status.phase, SchedulerPhase::Controlling);
            if scheduler.status.message.contains("typed search pending") {
                assert!(scheduler.status.message.contains("BudgetExhausted"));
                assert!(scheduler.status.message.contains("finite search budget"));
                break;
            }
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(3));
        scheduler.control_tick(&sender).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
        assert!(scheduler.status.last_zero_ack_id.is_some());
    }

    #[tokio::test]
    async fn scheduler_message_pending_retains_reason_and_failures_without_growth() {
        for wait_reason in [None, Some(""), Some("  "), Some("typed search pending")] {
            for has_failures in [false, true] {
                let planner = FakePlanner::commanding();
                let failures = if has_failures {
                    vec![nuwax_nav_scan::LocalPlanError { body_witness: None,
                        kind: nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
                        reason: "finite search budget; observation expired; required clearance".into(),
                        closest_pose: None,
                        blocked: None,
                    }]
                } else { vec![] };
                {
                    let mut state = planner.state.lock().unwrap();
                    state.step = PlannerStep::Pending(failures);
                    state.wait_reason = wait_reason.map(str::to_owned);
                }
                let mut scheduler = NavScheduler::new(
                    FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                    planner, SchedulerConfig::default()).unwrap();
                let (sender, _) = watch::channel(scheduler.status.clone());
                scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
                scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
                scheduler.start_session(&sender, session()).await.unwrap();
                scheduler.submit_path(&sender, path()).await.unwrap();
                let original_deadline = scheduler.path_wait_started_at;
                let identity = scheduler.current_execution_identity.clone();
                scheduler.control_tick(&sender).await.unwrap();
                let message = scheduler.status.message.clone();
                assert!(message.starts_with(wait_reason.filter(|r| !r.trim().is_empty())
                    .unwrap_or("局部轨迹待定，保持零速等待")));
                if has_failures {
                    assert!(message.contains("BudgetExhausted"));
                    assert!(message.contains("finite search budget; observation expired; required clearance"));
                } else {
                    assert!(!message.contains("[]"));
                    assert!(!message.ends_with(':'));
                }
                for _ in 0..3 {
                    scheduler.control_tick(&sender).await.unwrap();
                    assert_eq!(scheduler.status.message, message, "repeated ticks must not accumulate text");
                    assert_eq!(scheduler.path_wait_started_at, original_deadline);
                    assert_eq!(scheduler.current_execution_identity, identity);
                    assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
                    assert_eq!(scheduler.status.last_terminal, None);
                }
            }
        }
    }

    #[tokio::test]
    async fn default_wait_budget_terminates_all_stationary_waits_and_allows_recovery() {
        for reason in [None, Some(""), Some("当前几何尚无可执行运动")] {
        for pending in [false, true] {
            for recover in [false, true] {
                let planner = FakePlanner::commanding();
                {
                    let mut state = planner.state.lock().unwrap();
                    state.step = if pending { PlannerStep::Pending(vec![]) }
                        else { PlannerStep::Command(Twist2::ZERO) };
                    state.wait_reason = reason.map(str::to_owned);
                }
                let mut scheduler = NavScheduler::new(
                    FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                    planner, SchedulerConfig::default()).unwrap();
                let (sender, _) = watch::channel(scheduler.status.clone());
                scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
                scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
                scheduler.start_session(&sender, session()).await.unwrap();
                scheduler.submit_path(&sender, path()).await.unwrap();
                let started = scheduler.path_wait_started_at;
                scheduler.control_tick(&sender).await.unwrap();
                assert_eq!(scheduler.path_wait_started_at, started, "持续零速不得清除等待期限");
                if recover {
                    let mut state = scheduler.planner.state.lock().unwrap();
                    state.step = PlannerStep::Command(Twist2 { linear: Point2::new(0.2, 0.0), angular_z: 0.0 });
                    state.wait_reason = None;
                }
                scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(31));
                scheduler.control_tick(&sender).await.unwrap();
                assert!(scheduler.path_wait_started_at.is_none());
                if recover {
                    assert_eq!(scheduler.status.last_terminal, None);
                    assert_ne!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
                } else {
                    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
                    assert!(scheduler.status.last_zero_ack_id.is_some());
                    if let Some(reason) = reason.filter(|s|!s.is_empty()) {
                        assert!(scheduler.status.message.contains(reason));
                    }
                }
            }
        }
        }
        let mut scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (sender, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
        scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
        scheduler.start_session(&sender, session()).await.unwrap();
        scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(31));
        scheduler.check_path_wait_timeout(&sender).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
        assert!(scheduler.status.last_zero_ack_id.is_some());
    }

    #[tokio::test]
    async fn scheduler_message_timeout_first_terminal_retains_cause_and_fence() {
        for cause in ["BudgetExhausted: finite search budget", "NearFieldObservation: cell=[1,2,3] expired",
            "surface does not satisfy the required clearance", "", "  "] {
            let motion = Arc::new(Mutex::new(FakeMotionState::default()));
            let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()),
                FakePlanner::commanding(), SchedulerConfig {
                path_wait_timeout: Duration::from_secs(1), ..SchedulerConfig::default()
            }).unwrap();
            scheduler.session = Some(session());
            scheduler.lease_active = true;
            scheduler.path_wait_started_at = Some(Instant::now() - scheduler.config.path_wait_timeout);
            scheduler.status.message = cause.into();
            let (sender, observed) = watch::channel(scheduler.status.clone());
            scheduler.check_path_wait_timeout(&sender).await.unwrap();
            let status = observed.borrow();
            assert_eq!(status.last_terminal, Some(TerminalOutcome::PlannerFailed));
            assert_eq!(status.phase, SchedulerPhase::Failed);
            assert!(status.message.starts_with("planner path timeout after "));
            assert!(status.message.contains("ms; fencing"));
            if !cause.trim().is_empty() { assert!(status.message.contains(cause)); }
            else { assert!(!status.message.contains("; ;")); }
            assert!(status.message.ends_with("PlannerFailed after verified fence"));
            assert_eq!(status.last_zero_ack_id.as_deref(), Some("zero-ack"));
            assert_eq!(status.last_applied_twist, Some(Twist2::ZERO));
            assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("fence:")).count(), 1);
            assert!(scheduler.path_wait_started_at.is_none());
        }
    }

    #[tokio::test]
    async fn scheduler_message_timeout_stop_failure_remains_primary() {
        let motion = Arc::new(Mutex::new(FakeMotionState {
            unconfirmed_fences: 1, ..Default::default()
        }));
        let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()),
            FakePlanner::commanding(), SchedulerConfig {
                path_wait_timeout: Duration::from_secs(1), ..SchedulerConfig::default()
            }).unwrap();
        scheduler.session = Some(session());
        scheduler.lease_active = true;
        scheduler.path_wait_started_at = Some(Instant::now() - scheduler.config.path_wait_timeout);
        scheduler.status.message = "BudgetExhausted: finite search budget".into();
        let (sender, observed) = watch::channel(scheduler.status.clone());
        let error = scheduler.check_path_wait_timeout(&sender).await.unwrap_err();
        assert!(matches!(error, RuntimeError::StopUnconfirmed(ref reason) if reason == "等待实际停稳"));
        let status = observed.borrow();
        assert_eq!(status.message, "等待实际停稳");
        assert_eq!(status.phase, SchedulerPhase::Failed);
        assert_eq!(status.last_terminal, None);
        assert!(status.last_zero_ack_id.is_none());
        assert!(!status.motion_sink_ready);
        assert!(scheduler.lease_active);
        assert!(scheduler.session.is_some());
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("fence:")).count(), 1);
    }

    #[tokio::test]
    async fn unusable_replan_revisions_cannot_extend_the_original_wait_budget() {
        let planner = FakePlanner::commanding();
        planner.state.lock().unwrap().step = PlannerStep::ReplanRequired("无有效连接".into());
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(2),
                renew_interval: Duration::from_millis(20),
                path_wait_timeout: Duration::from_millis(60),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        handle.update_odometry(map_pose()).await.unwrap();
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_millis(200), async {
            let mut revision = 1;
            while handle.status().last_terminal.is_none() {
                if handle.status().phase == SchedulerPhase::AwaitingPath {
                    revision += 1;
                    let _ = handle.submit_path(path_for("goal-1", revision)).await;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("无效修订不能不断续期等待时间");
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::PlannerFailed)
        );
        assert!(handle.status().last_zero_ack_id.is_some());
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn source_pose_deadline_fences_while_receipt_is_still_fresh() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle
            .update_odometry_with_deadline(map_pose(), Instant::now() + Duration::from_millis(120))
            .await
            .unwrap();
        assert!(matches!(
            handle
                .update_odometry_with_deadline(
                    map_pose_at(2),
                    Instant::now() - Duration::from_millis(1)
                )
                .await,
            Err(RuntimeError::StaleInput(_))
        ));
        assert_eq!(
            handle.status().pose_source_timestamp_ns,
            1_000_000_000,
            "过期输入不能覆盖已接纳位姿"
        );
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        let terminal = wait_for_terminal(&handle, TerminalOutcome::SafetyFailed).await;
        assert!(
            terminal.applied_commands > 0,
            "验证运动开始后按来源期限实际停车"
        );
        assert!(
            terminal.pose_receipt_age_ms < 1_000,
            "接收期限仍有效，来源期限已先到"
        );
        assert!(terminal.message.contains("pose source deadline"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(terminal.last_zero_ack_id.is_some());
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn supplied_map_receipt_age_survives_construction_and_rejects_the_future() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle.update_odometry(map_pose()).await.unwrap();
        assert!(matches!(
            handle
                .update_costmap_received(
                    costmap_at(1_000_000_000),
                    Instant::now() + Duration::from_secs(1)
                )
                .await,
            Err(RuntimeError::InvalidInput {
                field: "costmap.received_at",
                ..
            })
        ));
        handle
            .update_costmap_received(
                costmap_at(1_000_000_000),
                Instant::now() - Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert!(!handle.status().planner_inputs_fresh);
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        wait_for_terminal(&handle, TerminalOutcome::SafetyFailed).await;
        assert!(motion_state
            .lock()
            .unwrap()
            .calls
            .iter()
            .all(|call| !call.starts_with("twist:")));
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn actual_surface_backend_stops_for_changed_map_and_resumes_only_new_path() {
        let mut local_config = SurfaceFollowerConfig::new(0.4);
        local_config.controller.speed = 0.2;
        local_config.controller.goal_tolerance = 0.12;
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            planner_worker::PlannerWorker::new(SurfaceFollower::new(local_config).unwrap()).unwrap(),
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        let mut start = nuwax_nav_astar::helpers::make_pose_stamped("map", 0.55, 1.55, 0.0);
        start.pose.position.z = 0.5;
        start.stamp = Timestamp::from_unix(1, 0);
        let mut goal = start.clone();
        goal.pose.position.x = 4.55;
        let surface_path = Path3D {
            poses: vec![start.clone(), goal.clone()],
        };
        handle.update_odometry(start.clone()).await.unwrap();
        // Fixture: continuously observe the unchanged synthetic pose
        // with timestamps advanced by actual elapsed monotonic time; keep real TTL.
        let observation_clock = Instant::now();
        let observation_handle = handle.clone();
        let observation_pose = start.clone();
        let observation_task = tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_millis(10)).await;
                let elapsed = observation_clock.elapsed();
                let mut pose = observation_pose.clone();
                pose.stamp = Timestamp::from_unix(1 + elapsed.as_secs() as i64, elapsed.subsec_nanos());
                if observation_handle.update_odometry(pose).await.is_err() { break; }
            }
        });

        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(surface_follower::test_snapshot(0.0, false)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .unwrap();
        let mut surface_session = session();
        surface_session.goal_pose = goal;
        handle.start_session(surface_session).await.unwrap();
        let mut first_path = path();
        first_path.path = ExecutionPath::Surface(surface_path.clone());
        handle.submit_path(first_path).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !handle.status().last_applied_twist.is_some_and(|command|command.linear.x>0.0) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .unwrap();
        assert!(handle.status().last_applied_twist.unwrap().linear.x > 0.0);
        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(Arc::new(surface_follower::test_snapshot(0.0, true)
                    .with_recent_obstacle_points(1,1,0,&[],[(0,[0.8,1.55,0.3])]).unwrap())),
                source_timestamp_ns: 2_000_000_000,
            })
            .await
            .unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        let stopped_count = handle.status().applied_commands;
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(handle.status().last_terminal, None);
        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(surface_follower::test_snapshot(0.0, false)),
                source_timestamp_ns: 3_000_000_000,
            })
            .await
            .unwrap();
        let mut replacement = path_for("goal-1", 2);
        replacement.path = ExecutionPath::Surface(surface_path);
        handle.submit_path(replacement).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().applied_commands <= stopped_count || !handle.status().last_applied_twist.is_some_and(|command|command.linear.x>0.0) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .unwrap();
        assert!(handle.status().last_applied_twist.unwrap().linear.x > 0.0);
        handle.cancel("goal-1").await.unwrap();

        // 原目标仍在上层，收到仅包含下层当前位置的路径不能报告已到达。
        let mut upper_goal_session = session_for("upper-goal");
        upper_goal_session.goal_pose = start.clone();
        upper_goal_session.goal_pose.pose.position.z += 3.0;
        handle.start_session(upper_goal_session).await.unwrap();
        let mut partial = path_for("upper-goal", 1);
        partial.path = ExecutionPath::Surface(Path3D { poses: vec![start] });
        handle.submit_path(partial).await.unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        assert_eq!(handle.status().last_terminal, None);
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        handle.cancel("upper-goal").await.unwrap();
        observation_task.abort();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn explicit_map_invalidation_preserves_owner_and_fences_only_active_motion() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle.update_odometry(map_pose()).await.unwrap();
        let old_receipt = Instant::now();
        assert!(matches!(
            handle
                .invalidate_map_received("未来失效事件", old_receipt + Duration::from_secs(1))
                .await,
            Err(RuntimeError::InvalidInput {
                field: "map.invalidated_at",
                ..
            })
        ));
        handle
            .update_costmap_received(costmap_at(1_000_000_000), old_receipt)
            .await
            .unwrap();
        let invalidation_receipt = Instant::now();
        handle
            .invalidate_map_received("空闲时切换地图", invalidation_receipt)
            .await
            .unwrap();
        assert_eq!(handle.status().phase, SchedulerPhase::Idle);
        assert!(!handle.status().planner_inputs_fresh);
        assert!(
            motion_state.lock().unwrap().calls.is_empty(),
            "空闲失效不能申请或围栏租约"
        );
        assert!(matches!(
            handle
                .update_costmap_received(costmap_at(2_000_000_000), old_receipt)
                .await,
            Err(RuntimeError::StaleInput(_))
        ));

        // 同一来源事件可同时表示新地图域；相同原接收时刻的有效新图可立即接入。
        handle
            .update_costmap_received(costmap_at(2_000_000_000), invalidation_receipt)
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        handle.invalidate_map("活动地图被清空").await.unwrap();
        let stopped = handle.status();
        assert_eq!(stopped.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_ne!(stopped.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(stopped.last_applied_twist, Some(Twist2::ZERO));
        assert!(stopped.last_zero_ack_id.is_some());
        assert!(!stopped.planner_inputs_fresh);
        assert!(!task.is_finished());

        handle.update_odometry(map_pose_at(3)).await.unwrap();
        handle
            .update_costmap(costmap_at(3_000_000_000))
            .await
            .unwrap();
        handle
            .start_session(session_for("new-map-goal"))
            .await
            .unwrap();
        handle
            .submit_path(path_for("new-map-goal", 1))
            .await
            .unwrap();
        handle.cancel("new-map-goal").await.unwrap();
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::Cancelled)
        );
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn surface_backend_without_calibration_is_rejected_before_connecting_motion() {
        let result = spawn_production_scheduler_with_backend(
            "/not-a-motion-socket",
            SchedulerConfig::default(),
            LocalExecutionBackendKind::SurfaceInProcess,
        )
        .await;
        assert!(matches!(
            result,
            Err(RuntimeError::InvalidInput {
                field: "body_to_support_z_m",
                ..
            })
        ));
        let mut basic =
            production_local_backend(LocalExecutionBackendKind::BasicInProcess).unwrap();
        let surface_path = Path3D {
            poses: vec![map_pose()],
        };
        assert!(basic
            .replace_execution_path(ExecutionPath::Surface(surface_path), None, map_pose())
            .is_err());
    }

    #[tokio::test]
    async fn default_session_waits_beyond_five_seconds_then_controls_and_cancels() {
        let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let scheduler = NavScheduler::new(motion, FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.unwrap();
        handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
        handle.start_session(session_for("persistent-path")).await.unwrap();
        tokio::time::sleep(Duration::from_millis(5100)).await;
        assert_eq!(handle.status().phase, SchedulerPhase::AwaitingPath);
        assert!(handle.status().last_terminal.is_none());
        handle.submit_path(path_for("persistent-path", 1)).await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(handle.status().phase, SchedulerPhase::Controlling);
        assert!(handle.status().last_applied_twist.is_some_and(|twist|twist != Twist2::ZERO));
        handle.cancel("persistent-path").await.unwrap();
        wait_for_terminal(&handle, TerminalOutcome::Cancelled).await;
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn missing_first_path_fences_and_allows_the_next_session() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                renew_interval: Duration::from_millis(20),
                path_wait_timeout: Duration::from_millis(30),
                robot_width_m: 0.3,
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
            },
        )
        .expect("scheduler with path deadline");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;

        handle
            .start_session(session_for("missing-path"))
            .await
            .expect("first session");
        let terminal = wait_for_terminal(&handle, TerminalOutcome::PlannerFailed).await;
        assert_eq!(terminal.phase, SchedulerPhase::Failed);
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .any(|call| call.starts_with("fence:")));
        assert!(!task.is_finished(), "path timeout must not stop the owner");

        planner_state.lock().expect("planner").step = PlannerStep::Succeeded;
        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("retry pose");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("retry costmap");
        handle
            .start_session(session_for("after-timeout"))
            .await
            .expect("session after path timeout");
        handle
            .submit_path(path_for("after-timeout", 1))
            .await
            .expect("path after timeout");
        publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn exploration_second_frontier_runs_after_first_child_terminal() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        planner.state.lock().expect("planner").step = PlannerStep::Succeeded;
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose 1");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("map 1");
        for (index, child) in ["explore:frontier-1", "explore:frontier-2"]
            .into_iter()
            .enumerate()
        {
            handle
                .start_session(session_for(child))
                .await
                .expect("frontier child accepted");
            handle
                .submit_path(path_for(child, 1))
                .await
                .expect("frontier path accepted");
            publish_pose_after_confirmed_stop(&handle, session().goal_pose.clone()).await;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
            if index == 0 {
                handle
                    .update_odometry(map_pose_at(2))
                    .await
                    .expect("pose 2");
                handle
                    .update_costmap(costmap_at(2_000_000_000))
                    .await
                    .expect("map 2");
            }
        }
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn default_map_receipt_keeps_observed_map_without_age_expiry() {
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            FakePlanner::commanding(),
            SchedulerConfig::default(),
        )
        .expect("默认生产调度器配置");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        let mut latest_map_source = 0;
        for tick in 0_i64..=20 {
            handle.update_odometry(map_pose_at(tick + 1)).await.unwrap();
            if tick % 5 == 0 {
                latest_map_source = 77_000_000_000 + tick * 50_000_000;
                handle
                    .update_costmap(costmap_at(latest_map_source))
                    .await
                    .unwrap();
            }
            assert!(
                handle.status().planner_inputs_fresh,
                "250毫秒实际地图续收不能周期性失效"
            );
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        for tick in 21_i64..=33 {
            handle.update_odometry(map_pose_at(tick + 1)).await.unwrap();
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        let expired = handle.status();
        assert!(expired.planner_inputs_fresh);
        assert_eq!(expired.costmap_source_timestamp_ns, latest_map_source);
        assert!(expired.costmap_receipt_age_ms > 500);
        assert!(
            matches!(
                handle.update_costmap(costmap_at(latest_map_source)).await,
                Err(RuntimeError::StaleInput(_))
            ),
            "旧缓存不能通过相同源时间重放续期"
        );
        assert!(handle.status().planner_inputs_fresh);
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn pose_stream_loss_fences_without_applied_heartbeat_refresh() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state.clone()),
            FakePlanner::commanding(),
            Duration::from_millis(30),
            Duration::from_millis(200),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        let loss_started = std::time::Instant::now();
        let terminal = wait_for_terminal(&handle, TerminalOutcome::SafetyFailed).await;
        assert!(
            loss_started.elapsed() <= Duration::from_millis(100),
            "pose loss must park within TTL + one tick + fence"
        );
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert!(terminal
            .message
            .starts_with("SafetyFailed cause=StaleInput:"));
        assert!(terminal.message.contains("pose receipt age"));
        assert!(!terminal.planner_inputs_fresh);
        assert!(motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .any(|call| call.starts_with("fence:")));
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn first_failure_terminal_keeps_the_reason_after_fencing() {
        for terminal in [TerminalOutcome::PlannerFailed,TerminalOutcome::SafetyFailed] {
            let mut scheduler = NavScheduler::new(
                FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                FakePlanner::commanding(),SchedulerConfig::default()).unwrap();
            scheduler.session=Some(session());
            scheduler.lease_active=true;
            scheduler.status.message="原始失败原因：目标前方没有可行支撑".into();
            let (sender, observed)=watch::channel(scheduler.status.clone());
            scheduler.finish_terminal(&sender,terminal,SchedulerPhase::Failed).await.unwrap();
            let status=observed.borrow();
            assert_eq!(status.last_terminal,Some(terminal));
            assert!(status.last_zero_ack_id.is_some());
            assert!(status.message.contains("目标前方没有可行支撑"),"首次终态丢失原因: {}",status.message);
        }
    }

    #[tokio::test]
    async fn stop_unconfirmed_keeps_scheduler_and_goal_until_cancel_retries() {
        let motion = Arc::new(Mutex::new(FakeMotionState {
            unconfirmed_fences: 1, ..Default::default()
        }));
        let planner = FakePlanner::commanding();
        planner.state.lock().unwrap().step = PlannerStep::Succeeded;
        let (handle, task) = test_scheduler(FakeMotion(motion), planner);
        wait_for_sink(&handle).await;
        handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
        handle.update_odometry(map_pose()).await.unwrap();
        let original = session();
        handle.start_session(original.clone()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().phase != SchedulerPhase::Failed {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        assert!(!task.is_finished(), "暂时缺少停稳确认不能结束调度器");
        assert_eq!(handle.status().session_id.as_deref(), Some(original.session_id.as_str()));
        assert!(handle.status().last_terminal.is_none());
        handle.cancel(&original.session_id).await.unwrap();
        assert_eq!(handle.status().last_terminal, Some(TerminalOutcome::Cancelled));
        assert!(handle.status().motion_sink_ready);
        handle.start_session(session()).await.unwrap();
        handle.cancel(&original.session_id).await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn costmap_stream_loss_fences_within_configured_deadline() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state),
            FakePlanner::commanding(),
            Duration::from_millis(200),
            Duration::from_millis(30),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        let loss_started = std::time::Instant::now();
        let terminal = wait_for_terminal(&handle, TerminalOutcome::SafetyFailed).await;
        assert!(loss_started.elapsed() <= Duration::from_millis(100));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(!terminal.planner_inputs_fresh);
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn replayed_pose_source_timestamp_does_not_extend_ttl() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state),
            FakePlanner::commanding(),
            Duration::from_millis(40),
            Duration::from_millis(200),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        tokio::time::sleep(Duration::from_millis(20)).await;
        let replay = handle
            .update_odometry(map_pose_at(1))
            .await
            .expect_err("same source stamp must be rejected");
        assert!(matches!(replay, RuntimeError::StaleInput(_)));
        wait_for_terminal(&handle, TerminalOutcome::SafetyFailed).await;
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn ambiguous_applied_failure_revokes_and_zeroes_before_task_failure() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState {
            fail_twist: true,
            ..FakeMotionState::default()
        }));
        let (handle, task) =
            test_scheduler(FakeMotion(motion_state.clone()), FakePlanner::commanding());
        wait_for_sink(&handle).await;
        handle.start_session(session()).await.expect("session");
        handle
            .update_costmap(CostmapObservation {
                costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("costmap");
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle.submit_path(path()).await.expect("path");

        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().last_terminal != Some(TerminalOutcome::SafetyFailed) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("失败必须先确认归零终态");
        assert!(!task.is_finished(), "已确认停车的目标失败不能结束调度拥有方");
        let calls = motion_state.lock().expect("motion").calls.clone();
        let twist = calls
            .iter()
            .position(|call| call.starts_with("twist:"))
            .expect("twist call");
        let fence = calls
            .iter()
            .position(|call| call.starts_with("fence:"))
            .expect("fence call");
        assert!(
            fence > twist,
            "ambiguous command failure must be fenced afterwards"
        );
        assert!(handle.status().motion_sink_ready);
        let terminal = handle.status();
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(
            terminal.message,
            "SafetyFailed cause=Motion: motion guard operation failed: injected APPLIED timeout"
        );
        motion_state.lock().unwrap().fail_twist = false;
        handle.start_session(session()).await.expect("停车后仍可接纳后续目标");
        handle.cancel(&session().session_id).await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn real_uds_adapter_requires_correlated_applied_and_fence_proofs() {
        let guard = spawn_scripted_guard("proofs").await;
        let mut client =
            MotionGuardClient::connect_with_config(production_client_config(guard.path.clone()))
                .await
                .expect("connect real guard fixture UDS");

        let admission = NavMotionAdmission {
            guard_generation: 9,
            guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
        };
        let lease = NavMotionPort::acquire(&mut client, "nav:uds:acquire", &admission)
            .await
            .expect("real lease proof");
        assert_eq!(lease.lease_id, 41);
        assert_eq!(lease.guard_generation, 9);
        let applied = NavMotionPort::submit_twist(
            &mut client,
            "nav:uds:twist",
            Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.1,
            },
        )
        .await
        .expect("correlated APPLIED proof");
        assert_eq!(applied.operation_id, "nav:uds:twist");
        assert_eq!(applied.applied_monotonic_ns, 101);
        let fence = NavMotionPort::revoke_and_zero(&mut client, "nav:uds:fence")
            .await
            .expect("correlated resource fence");
        assert_eq!(fence.zero_ack_id, "zero-uds-1");
        assert_eq!(fence.applied_monotonic_ns, 102);
        assert_eq!(fence.lease, FenceLeaseState::Idle);

        assert_eq!(guard.finish().await, vec!["acquire", "applied", "fence"]);
    }

    async fn collect_real_uds_cancel_latencies(delayed: DelayedOperation) -> Vec<Duration> {
        const SAMPLES: usize = 20;

        let label = match delayed {
            DelayedOperation::Submit => "delayed-submit",
            DelayedOperation::Renew => "delayed-renew",
            DelayedOperation::FenceResponse => "lost-first-fence-response",
        };
        let mut guard = spawn_priority_guard(label, delayed).await;
        let (handle, task) = spawn_production_scheduler(
            guard.path.clone(),
            SchedulerConfig {
                tick_interval: if delayed == DelayedOperation::Submit {
                    Duration::from_millis(5)
                } else {
                    Duration::from_secs(1)
                },
                renew_interval: if delayed == DelayedOperation::Renew {
                    Duration::from_millis(5)
                } else {
                    Duration::from_millis(250)
                },
                path_wait_timeout: Duration::from_secs(5),
                robot_width_m: 0.3,
                pose_ttl: Duration::from_secs(5),
                costmap_ttl: Duration::from_secs(5),
            },
        )
        .await
        .expect("connect priority guard fixture");
        wait_for_sink(&handle).await;

        let mut latencies = Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let session_id = format!("latency-{label}-{sample}");
            if delayed == DelayedOperation::Submit {
                let seconds = i64::try_from(sample + 1).expect("sample seconds");
                handle
                    .update_odometry(map_pose_at(seconds))
                    .await
                    .expect("fresh map pose");
                handle
                    .update_costmap(costmap_at(seconds * 1_000_000_000))
                    .await
                    .expect("fresh costmap");
            }
            handle
                .start_session(session_for(&session_id))
                .await
                .expect("latency session");
            if delayed == DelayedOperation::Submit {
                handle
                    .submit_path(path_for(&session_id, 1))
                    .await
                    .expect("latency path");
            }
            let expected_observation = match delayed {
                DelayedOperation::Submit => "applied",
                DelayedOperation::Renew => "renew",
                DelayedOperation::FenceResponse => "fence",
            };
            if delayed != DelayedOperation::FenceResponse {
                let observed =
                    match tokio::time::timeout(Duration::from_secs(1), guard.delayed_rx.recv())
                        .await
                    {
                        Ok(Some(observed)) => observed,
                        Ok(None) => panic!("priority guard observation channel closed"),
                        Err(_) => panic!(
                            "delayed operation observation timeout; scheduler status={:?}",
                            handle.status()
                        ),
                    };
                assert_eq!(observed, expected_observation);
            }

            let started = std::time::Instant::now();
            handle.cancel(&session_id).await.expect("priority fence");
            let elapsed = started.elapsed();
            if delayed == DelayedOperation::FenceResponse {
                let observed =
                    tokio::time::timeout(Duration::from_secs(1), guard.delayed_rx.recv())
                        .await
                        .expect("first fence attempt observation timeout")
                        .expect("priority guard observation channel");
                assert_eq!(observed, expected_observation);
            }
            assert!(
                elapsed <= MAX_CANCEL_FENCE_TIMEOUT,
                "cancel hard max exceeded: {elapsed:?}"
            );
            latencies.push(elapsed);
        }

        handle.shutdown().await.expect("scheduler shutdown");
        task.await
            .expect("scheduler task join")
            .expect("scheduler task result");
        latencies
    }

    fn assert_cancel_latency_distribution(label: &str, mut latencies: Vec<Duration>) {
        latencies.sort_unstable();
        let p95_index = (latencies.len() * 95).div_ceil(100).saturating_sub(1);
        let p95 = latencies[p95_index];
        let hard_max = *latencies.last().expect("latency samples");
        eprintln!(
            "cancel latency kind={label} samples={} p95_us={} hard_max_us={}",
            latencies.len(),
            p95.as_micros(),
            hard_max.as_micros(),
        );
        assert!(
            p95 <= Duration::from_millis(150),
            "cancel p95 exceeded 150ms: {p95:?}"
        );
        assert!(
            hard_max <= MAX_CANCEL_FENCE_TIMEOUT,
            "cancel hard max exceeded 300ms: {hard_max:?}"
        );
    }

    #[tokio::test]
    async fn cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency() {
        assert_cancel_latency_distribution(
            "delayed-submit",
            collect_real_uds_cancel_latencies(DelayedOperation::Submit).await,
        );
    }

    #[tokio::test]
    async fn cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency() {
        assert_cancel_latency_distribution(
            "delayed-renew",
            collect_real_uds_cancel_latencies(DelayedOperation::Renew).await,
        );
    }

    #[tokio::test]
    async fn cancel_replays_identical_fence_after_first_response_timeout() {
        assert_cancel_latency_distribution(
            "lost-first-fence-response",
            collect_real_uds_cancel_latencies(DelayedOperation::FenceResponse).await,
        );
    }
    include!("dispatch_handoff_tests.rs");

}
