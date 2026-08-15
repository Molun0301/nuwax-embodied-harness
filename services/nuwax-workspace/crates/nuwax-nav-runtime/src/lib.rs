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
use std::time::Duration;

use async_trait::async_trait;
use nuwax_motion_safety_guard::wire::{LeaseTuple, TwistCommand};
use nuwax_motion_safety_guard::{
    ClientConfig, MotionAdmissionProof, MotionGuardClient, MotionGuardClientError, MAX_FRAME_BYTES,
};
use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerConfig, StopMessage};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{Path2D, PoseStamped, Twist2};
use thiserror::Error;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;
use tokio::time::{interval_at, Instant, MissedTickBehavior};

/// Production controller cadence: 20 Hz.
pub const DEFAULT_TICK_INTERVAL: Duration = Duration::from_millis(50);
/// Maximum scheduler-owned lease renewal interval.
pub const MAX_RENEW_INTERVAL: Duration = Duration::from_millis(250);
/// Qualified upper bound for one production NAV guard exchange.
pub const MAX_NAV_REQUEST_TIMEOUT: Duration = Duration::from_millis(250);
/// Default maximum receipt age for a map-frame pose used by the controller.
pub const DEFAULT_POSE_TTL: Duration = Duration::from_millis(200);
/// Default maximum receipt age for the planner-consumed occupancy costmap.
pub const DEFAULT_COSTMAP_TTL: Duration = Duration::from_millis(500);
/// Maximum time a leased session may wait for its first correlated path.
pub const DEFAULT_PATH_WAIT_TIMEOUT: Duration = Duration::from_secs(5);
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
/// generation, revision, and payload digest.
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
    /// Committed map payload digest.
    pub committed_map_payload_sha256: [u8; 32],
    /// Authoritative committed map frame.
    pub committed_map_frame_id: String,
    /// Navigation boundary identifier.
    pub boundary_id: String,
    /// Navigation boundary digest.
    pub boundary_sha256: [u8; 32],
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
    /// Canonical goal payload digest.
    pub goal_payload_sha256: [u8; 32],
    /// Canonical path payload digest; zero before path admission.
    pub path_payload_sha256: [u8; 32],
}

impl PathExecutionIdentity {
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
            && self.committed_map_payload_sha256 == other.committed_map_payload_sha256
            && self.committed_map_frame_id == other.committed_map_frame_id
            && self.boundary_id == other.boundary_id
            && self.boundary_sha256 == other.boundary_sha256
            && self.motion_guard_generation == other.motion_guard_generation
            && self.motion_guard_boot_nonce == other.motion_guard_boot_nonce
            && self.motion_stop_generation == other.motion_stop_generation
            && self.goal_payload_sha256 == other.goal_payload_sha256
    }
}

/// One navigation session and its request-level motion admission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NavSession {
    /// Stable goal/session identifier.
    pub session_id: String,
    /// Complete sealed goal identity expected on every admitted path.
    pub execution_identity: PathExecutionIdentity,
    /// Runtime generation of the unique local-execution owner.
    pub local_owner_runtime_generation: u64,
    /// Request-level motion admission proof transported with the goal.
    pub motion_admission: NavMotionAdmission,
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
    /// Canonical payload digest supplied by the versioned codec.
    pub payload_hash: [u8; 32],
    /// Planner output consumed by the local controller.
    pub path: Path2D,
}

/// Scheduler lifecycle. Terminal meaning is carried separately in status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerPhase {
    /// No lease and no active session.
    Idle,
    /// Waiting for the motion guard to grant a NAV lease.
    Acquiring,
    /// Lease acquired; waiting for the first correlated path.
    AwaitingPath,
    /// Running the 20 Hz controller and waiting for every APPLIED proof.
    Controlling,
    /// Cancel accepted; no more twist is issued while fence proof is pending.
    Cancelling,
    /// A non-cancel terminal path is waiting for fence proof.
    Fencing,
    /// Fail-closed state. No twist or renewal is issued.
    Failed,
}

/// Verified terminal outcome from the most recent session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalOutcome {
    /// Cancel completed after a valid resource fence.
    Cancelled,
    /// Planner reported arrival and the lease was fenced.
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
    /// Maximum time from lease acquisition to the first correlated path.
    pub path_wait_timeout: Duration,
    /// Collision footprint width passed to the local planner.
    pub robot_width_m: f64,
    /// Maximum monotonic receipt age for map-frame pose input.
    pub pose_ttl: Duration,
    /// Maximum monotonic receipt age for occupancy costmap input.
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
}

impl MotionPortError {
    /// Construct a motion-port error.
    #[must_use]
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            lease_may_be_active: true,
        }
    }

    /// Construct an error known to have occurred before lease acquisition.
    #[must_use]
    pub fn definite_no_lease(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            lease_may_be_active: false,
        }
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
    /// Decoded controller costmap.
    pub costmap: CostMap2D,
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

/// Mutable motion boundary owned by exactly one scheduler task.
#[async_trait]
pub trait NavMotionPort: Send + 'static {
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
}

#[async_trait]
impl NavMotionPort for ProductionNavMotionPort {
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

    async fn revoke_and_zero(&mut self, operation_id: &str) -> Result<FenceProof, MotionPortError> {
        self.priority_revoke_and_zero(operation_id).await
    }

    async fn priority_revoke_and_zero(
        &mut self,
        operation_id: &str,
    ) -> Result<FenceProof, MotionPortError> {
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
        self.lease_tuple = None;
        Ok(Self::fence_proof(fence))
    }
}

/// Result of one deterministic local-planner step.
#[derive(Debug, Clone, PartialEq)]
pub enum PlannerStep {
    /// Continue controlling with this twist.
    Command(Twist2),
    /// Goal arrival detected.
    Succeeded,
    /// Planner cannot safely continue.
    Failed(String),
}

#[derive(Debug, Clone, PartialEq)]
struct LocalExecutionCommandIntent {
    identity: PathExecutionIdentity,
    sequence: u64,
    owner_runtime_generation: u64,
    payload: LocalExecutionPayload,
}

#[derive(Debug, Clone, PartialEq)]
enum LocalExecutionPayload {
    Command(Twist2),
    Succeeded,
    Failed(String),
}

/// Synchronous deterministic planner boundary.
pub trait NavPlanner: Send + 'static {
    /// Replace the active plan after version validation.
    fn replace_path(&mut self, path: Path2D);
    /// Update the latest odometry sample.
    fn update_odometry(&mut self, odometry: PoseStamped);
    /// Update the latest planner-consumed occupancy costmap.
    fn update_costmap(&mut self, costmap: CostMap2D);
    /// Produce one controller step.
    fn step(&mut self, robot_width_m: f64) -> PlannerStep;
    /// Clear planner state after a terminal fence.
    fn stop(&mut self);
}

/// Production-qualified local execution backend selected at scheduler construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalExecutionBackendKind {
    /// Run the deterministic basic local planner inside the scheduler task.
    BasicInProcess,
}

struct BasicInProcessLocalBackend(LocalPlanner);

impl BasicInProcessLocalBackend {
    fn production() -> Self {
        Self(LocalPlanner::with_config(
            LocalPlannerConfig::default_config(),
        ))
    }

    #[cfg(test)]
    fn kind(&self) -> LocalExecutionBackendKind {
        LocalExecutionBackendKind::BasicInProcess
    }
}

impl NavPlanner for BasicInProcessLocalBackend {
    fn replace_path(&mut self, path: Path2D) {
        self.0.start_planning(path);
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
            Some(StopMessage::ObstacleFound) => PlannerStep::Failed("obstacle found".into()),
            Some(StopMessage::Error) => PlannerStep::Failed("local planner error".into()),
        }
    }

    fn stop(&mut self) {
        self.0.stop_planning();
    }
}

fn production_local_backend(kind: LocalExecutionBackendKind) -> BasicInProcessLocalBackend {
    match kind {
        LocalExecutionBackendKind::BasicInProcess => BasicInProcessLocalBackend::production(),
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
    current_payload_hash: Option<[u8; 32]>,
    current_path: Option<Path2D>,
    current_execution_identity: Option<PathExecutionIdentity>,
    local_intent_sequence: u64,
    path_wait_started_at: Option<Instant>,
    pose_receipt: Option<PlannerInputReceipt>,
    costmap_receipt: Option<PlannerInputReceipt>,
    operation_sequence: u64,
    status: SchedulerStatus,
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
            current_payload_hash: None,
            current_path: None,
            current_execution_identity: None,
            local_intent_sequence: 0,
            path_wait_started_at: None,
            pose_receipt: None,
            costmap_receipt: None,
            operation_sequence: 0,
            status,
        })
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
        let mut renew = interval_at(
            Instant::now() + self.config.renew_interval,
            self.config.renew_interval,
        );
        renew.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut input_open = true;
        let mut urgent_open = true;

        loop {
            if !input_open && !urgent_open {
                return self
                    .finish_terminal(&status_tx, TerminalOutcome::Shutdown, SchedulerPhase::Idle)
                    .await;
            }

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

                _ = renew.tick(), if self.should_renew() => {
                    match await_interruptible(urgent_open, &mut urgent_rx, self.renew()).await {
                        Interruptible::Completed(Ok(())) => {}
                        Interruptible::Completed(Err(error)) => {
                            let error = self.fail_closed(&status_tx, error).await;
                            self.status.motion_sink_ready = false;
                            self.publish_status(&status_tx);
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

                command = input_rx.recv(), if input_open => {
                    match command {
                        Some(command) => self.handle_input(&status_tx, command).await,
                        None => input_open = false,
                    }
                }

                _ = tick.tick(), if matches!(
                    self.status.phase,
                    SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling
                ) => {
                    match await_interruptible(urgent_open, &mut urgent_rx, self.session_tick(&status_tx)).await {
                        Interruptible::Completed(Ok(())) => {}
                        Interruptible::Completed(Err(error)) => {
                            self.status.motion_sink_ready = false;
                            self.publish_status(&status_tx);
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
        let result = match command.kind {
            InputKind::StartSession(session) => self.start_session(status_tx, session).await,
            InputKind::SubmitPath(path) => self.submit_path(status_tx, path).await,
            InputKind::UpdateOdometry(odometry) => self.observe_odometry(status_tx, odometry),
            InputKind::UpdateCostmap(observation) => self.observe_costmap(status_tx, observation),
        };
        let _ = command.reply.send(result);
    }

    fn observe_odometry(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        odometry: PoseStamped,
    ) -> Result<(), RuntimeError> {
        if odometry.frame_id != nuwax_nav_types_messages::NavFrameId::Map {
            return Err(RuntimeError::InvalidInput {
                field: "odometry.frame_id",
                reason: format!("expected map, got {}", odometry.frame_id.as_str()),
            });
        }
        let source_timestamp_ns = odometry.stamp.nanos;
        validate_source_timestamp(
            "odometry.source_timestamp_ns",
            source_timestamp_ns,
            self.pose_receipt,
        )?;
        self.planner.update_odometry(odometry);
        self.pose_receipt = Some(PlannerInputReceipt {
            source_timestamp_ns,
            received_at: Instant::now(),
        });
        self.refresh_input_status(Instant::now());
        self.status.message = "fresh map pose consumed".into();
        self.publish_status(status_tx);
        Ok(())
    }

    fn observe_costmap(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        observation: CostmapObservation,
    ) -> Result<(), RuntimeError> {
        validate_source_timestamp(
            "costmap.source_timestamp_ns",
            observation.source_timestamp_ns,
            self.costmap_receipt,
        )?;
        self.planner.update_costmap(observation.costmap);
        self.costmap_receipt = Some(PlannerInputReceipt {
            source_timestamp_ns: observation.source_timestamp_ns,
            received_at: Instant::now(),
        });
        self.refresh_input_status(Instant::now());
        self.status.message = "fresh occupancy costmap consumed".into();
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
            .is_some_and(|receipt| now.duration_since(receipt.received_at) <= self.config.pose_ttl)
            && self.costmap_receipt.is_some_and(|receipt| {
                now.duration_since(receipt.received_at) <= self.config.costmap_ttl
            });
    }

    fn stale_input_reason(&mut self, now: Instant) -> Option<String> {
        self.refresh_input_status(now);
        let pose_reason =
            stale_receipt_reason("pose", self.pose_receipt, now, self.config.pose_ttl);
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
        self.planner.stop();
        self.session = Some(session.clone());
        self.current_path_revision = 0;
        self.current_payload_hash = None;
        self.current_path = None;
        self.current_execution_identity = None;
        self.local_intent_sequence = 0;
        self.path_wait_started_at = None;
        self.status = SchedulerStatus {
            phase: SchedulerPhase::Acquiring,
            session_id: Some(session.session_id.clone()),
            path_revision: 0,
            applied_commands: 0,
            last_applied_twist: None,
            motion_sink_ready: true,
            guard_generation: session.motion_admission.guard_generation,
            planner_inputs_fresh: self.status.planner_inputs_fresh,
            pose_source_timestamp_ns: self.status.pose_source_timestamp_ns,
            costmap_source_timestamp_ns: self.status.costmap_source_timestamp_ns,
            pose_receipt_age_ms: self.status.pose_receipt_age_ms,
            costmap_receipt_age_ms: self.status.costmap_receipt_age_ms,
            last_terminal: None,
            last_zero_ack_id: None,
            last_fence_monotonic_ns: 0,
            message: "acquiring NAV lease".into(),
        };
        self.publish_status(status_tx);

        let operation_id = self.next_operation_id("acquire")?;
        match self
            .motion
            .acquire(&operation_id, &session.motion_admission)
            .await
        {
            Ok(proof)
                if proof.lease_id != 0
                    && proof.guard_generation == session.motion_admission.guard_generation =>
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
                self.current_payload_hash = None;
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
            let immutable = self.current_payload_hash == Some(path.payload_hash)
                && self.current_path.as_ref() == Some(&path.path)
                && self.current_execution_identity.as_ref() == Some(&path.execution_identity);
            if immutable {
                return Ok(());
            }
            let error = RuntimeError::PathMutation {
                reason: format!(
                    "session={} path_revision={} changed identity or payload",
                    path.session_id, path.path_revision
                ),
            };
            let qualified_cause = format!("SafetyFailed cause=PathMutation: {error}");
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

        self.planner.replace_path(path.path.clone());
        self.current_path_revision = path.path_revision;
        self.current_payload_hash = Some(path.payload_hash);
        self.current_path = Some(path.path);
        self.current_execution_identity = Some(path.execution_identity);
        self.path_wait_started_at = None;
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
        if let Some(reason) = self.stale_input_reason(Instant::now()) {
            let qualified_cause = format!("SafetyFailed cause=StaleInput: {reason}");
            self.status.message = format!("planner input stale: {reason}; fencing");
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
        let step = self.planner.step(self.config.robot_width_m);
        let intent = self.wrap_local_intent(step)?;
        if !self.local_intent_is_current(&intent) {
            self.status.message = "stale local execution intent rejected".into();
            self.publish_status(status_tx);
            return Ok(());
        }
        match intent.payload {
            LocalExecutionPayload::Command(twist) => {
                if let Err(error) = validate_twist(twist) {
                    return Err(self.fail_closed(status_tx, error).await);
                }
                let operation_id = self.next_operation_id("twist")?;
                let proof = match self.motion.submit_twist(&operation_id, twist).await {
                    Ok(proof) => proof,
                    Err(error) => {
                        let error = RuntimeError::Motion(error.message);
                        return Err(self.fail_closed(status_tx, error).await);
                    }
                };
                if proof.operation_id != operation_id || proof.applied_monotonic_ns == 0 {
                    let reason = format!(
                        "expected operation_id={operation_id}, got {} at {}",
                        proof.operation_id, proof.applied_monotonic_ns
                    );
                    let error = RuntimeError::InvalidApplied(reason);
                    return Err(self.fail_closed(status_tx, error).await);
                }
                self.status.applied_commands = self.status.applied_commands.saturating_add(1);
                self.status.last_applied_twist = Some(twist);
                self.status.message = "twist APPLIED".into();
                self.publish_status(status_tx);
                Ok(())
            }
            LocalExecutionPayload::Succeeded => {
                self.finish_terminal(status_tx, TerminalOutcome::Succeeded, SchedulerPhase::Idle)
                    .await
            }
            LocalExecutionPayload::Failed(reason) => {
                let qualified_cause = format!("PlannerFailed cause=LocalPlanner: {reason}");
                self.status.message = format!("planner failed: {reason}; fencing");
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

    fn wrap_local_intent(
        &mut self,
        step: PlannerStep,
    ) -> Result<LocalExecutionCommandIntent, RuntimeError> {
        self.local_intent_sequence = self
            .local_intent_sequence
            .checked_add(1)
            .ok_or(RuntimeError::SequenceExhausted)?;
        let identity = self.current_execution_identity.clone().ok_or_else(|| {
            RuntimeError::Planner("local planner produced an intent without an active path".into())
        })?;
        let owner_runtime_generation = self
            .session
            .as_ref()
            .map(|session| session.local_owner_runtime_generation)
            .ok_or_else(|| {
                RuntimeError::Planner(
                    "local planner produced an intent without an active session".into(),
                )
            })?;
        let payload = match step {
            PlannerStep::Command(twist) => LocalExecutionPayload::Command(twist),
            PlannerStep::Succeeded => LocalExecutionPayload::Succeeded,
            PlannerStep::Failed(reason) => LocalExecutionPayload::Failed(reason),
        };
        Ok(LocalExecutionCommandIntent {
            identity,
            sequence: self.local_intent_sequence,
            owner_runtime_generation,
            payload,
        })
    }

    fn local_intent_is_current(&self, intent: &LocalExecutionCommandIntent) -> bool {
        self.lease_active
            && self.status.phase == SchedulerPhase::Controlling
            && intent.sequence != 0
            && intent.sequence == self.local_intent_sequence
            && self.current_execution_identity.as_ref() == Some(&intent.identity)
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
        if self.status.phase == SchedulerPhase::Controlling {
            return self.control_tick(status_tx).await;
        }
        if self.status.phase != SchedulerPhase::AwaitingPath {
            return Ok(());
        }
        let Some(started_at) = self.path_wait_started_at else {
            let error = RuntimeError::Planner("awaiting path without a deadline".into());
            return Err(self.fail_closed(status_tx, error).await);
        };
        if Instant::now().duration_since(started_at) < self.config.path_wait_timeout {
            return Ok(());
        }
        self.status.message = format!(
            "planner path timeout after {}ms; fencing",
            self.config.path_wait_timeout.as_millis(),
        );
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
        self.motion
            .renew(&operation_id)
            .await
            .map_err(|error| RuntimeError::Motion(error.message))
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
            RuntimeError::Planner(_) => "Planner",
            RuntimeError::StaleInput(_) => "StaleInput",
            RuntimeError::InvalidApplied(_) => "InvalidApplied",
            RuntimeError::InvalidFence(_) => "InvalidFence",
            RuntimeError::SchedulerStopped => "SchedulerStopped",
            RuntimeError::SequenceExhausted => "SequenceExhausted",
        };
        let qualified_cause = format!("SafetyFailed cause={cause_kind}: {original}");
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
        self.current_execution_identity = None;
        self.local_intent_sequence = 0;
        self.planner.stop();
        let had_lease = self.lease_active;
        let mut verified_fence = None;
        if had_lease {
            if self.status.phase != SchedulerPhase::Cancelling {
                self.status.phase = SchedulerPhase::Fencing;
                self.status.message = format!("waiting for {terminal:?} fence proof");
                self.publish_status(status_tx);
            }
            let operation_id = self.next_operation_id("fence")?;
            let proof = match self.motion.priority_revoke_and_zero(&operation_id).await {
                Ok(proof) => proof,
                Err(error) => {
                    self.fail_status(status_tx, &error.message);
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

        self.planner.stop();
        self.session = None;
        self.current_path_revision = 0;
        self.current_payload_hash = None;
        self.current_path = None;
        self.current_execution_identity = None;
        self.local_intent_sequence = 0;
        self.path_wait_started_at = None;
        self.status.phase = phase_after;
        self.status.session_id = None;
        self.status.path_revision = 0;
        self.status.last_terminal = Some(terminal);
        if let Some(proof) = verified_fence {
            self.status.last_zero_ack_id = Some(proof.zero_ack_id);
            self.status.last_fence_monotonic_ns = proof.applied_monotonic_ns;
            self.status.last_applied_twist = Some(Twist2::ZERO);
            self.status.message = format!("{terminal:?} after verified fence");
        } else {
            self.status.message = format!("{terminal:?} with no active lease");
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
        status_tx.send_replace(self.status.clone());
    }
}

/// Cloneable command handle for a spawned scheduler.
#[derive(Clone)]
pub struct NavSchedulerHandle {
    input_tx: mpsc::Sender<InputCommand>,
    urgent_tx: mpsc::Sender<UrgentCommand>,
    status_rx: watch::Receiver<SchedulerStatus>,
}

impl NavSchedulerHandle {
    /// Acquire a lease for a new session.
    pub async fn start_session(&self, session: NavSession) -> Result<(), RuntimeError> {
        self.send_input(InputKind::StartSession(session)).await
    }

    /// Submit a correlated, immutable path revision.
    pub async fn submit_path(&self, path: VersionedPath) -> Result<(), RuntimeError> {
        self.send_input(InputKind::SubmitPath(path)).await
    }

    /// Update planner odometry through the scheduler task.
    pub async fn update_odometry(&self, odometry: PoseStamped) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry)).await
    }

    /// Update the planner costmap with its real revision and source timestamp.
    pub async fn update_costmap(
        &self,
        observation: CostmapObservation,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateCostmap(observation)).await
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

    /// Stop the scheduler and fence any active lease.
    pub async fn shutdown(&self) -> Result<(), RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::Shutdown { reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
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
        let (reply, result) = oneshot::channel();
        self.input_tx
            .send(InputCommand { kind, reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
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
    let (input_tx, input_rx) = mpsc::channel(INPUT_CAPACITY);
    let (urgent_tx, urgent_rx) = mpsc::channel(URGENT_CAPACITY);
    let (status_tx, status_rx) = watch::channel(scheduler.status.clone());
    let handle = NavSchedulerHandle {
        input_tx,
        urgent_tx,
        status_rx,
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
    let motion = ProductionNavMotionPort::connect(production_client_config(socket_path))
        .await
        .map_err(|error| RuntimeError::Motion(error.message))?;
    let backend = production_local_backend(backend_kind);
    let scheduler = NavScheduler::new(motion, backend, config)?;
    Ok(spawn_scheduler(scheduler))
}

struct InputCommand {
    kind: InputKind,
    reply: oneshot::Sender<Result<(), RuntimeError>>,
}

enum InputKind {
    StartSession(NavSession),
    SubmitPath(VersionedPath),
    UpdateOdometry(PoseStamped),
    UpdateCostmap(CostmapObservation),
}

enum UrgentCommand {
    Cancel {
        session_id: String,
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
    if config.path_wait_timeout.is_zero() {
        return Err(RuntimeError::InvalidInput {
            field: "path_wait_timeout",
            reason: "must be positive".into(),
        });
    }
    if !config.robot_width_m.is_finite() || config.robot_width_m <= 0.0 {
        return Err(RuntimeError::InvalidInput {
            field: "robot_width_m",
            reason: "must be finite and positive".into(),
        });
    }
    if config.pose_ttl.is_zero() || config.costmap_ttl.is_zero() {
        return Err(RuntimeError::InvalidInput {
            field: "planner_input_ttl",
            reason: "pose and costmap TTL must be positive".into(),
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
    (age > ttl).then(|| {
        format!(
            "{label} receipt age={}ms exceeds TTL={}ms (source_timestamp_ns={})",
            age.as_millis(),
            ttl.as_millis(),
            receipt.source_timestamp_ns,
        )
    })
}

fn validate_session(session: &NavSession) -> Result<(), RuntimeError> {
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
    if path.payload_hash == [0; 32]
        || path.payload_hash != path.execution_identity.path_payload_sha256
    {
        return Err(RuntimeError::InvalidInput {
            field: "path.payload_hash",
            reason: "must be non-zero and equal the execution identity digest".into(),
        });
    }
    if path.execution_identity.planner_runtime_generation <= 0 {
        return Err(RuntimeError::InvalidInput {
            field: "path.execution_identity.planner_runtime_generation",
            reason: "must be positive".into(),
        });
    }
    if path.path.is_empty() {
        return Err(RuntimeError::InvalidInput {
            field: "path.payload",
            reason: "must contain at least one point".into(),
        });
    }
    if path
        .path
        .points
        .iter()
        .any(|point| !point.x.is_finite() || !point.y.is_finite())
    {
        return Err(RuntimeError::InvalidInput {
            field: "path.payload",
            reason: "all path coordinates must be finite".into(),
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
            self.0
                .lock()
                .expect("fake motion state")
                .calls
                .push(format!("fence:{operation_id}"));
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
            committed_map_payload_sha256: [3; 32],
            committed_map_frame_id: "map".into(),
            boundary_id: String::new(),
            boundary_sha256: [0; 32],
            motion_guard_generation: 7,
            motion_guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
            planner_runtime_generation: i64::from(path_revision != 0),
            path_revision,
            goal_payload_sha256: [4; 32],
            path_payload_sha256: if path_revision == 0 {
                [0; 32]
            } else {
                [u8::try_from(path_revision).unwrap_or(u8::MAX); 32]
            },
        }
    }

    fn session_for(session_id: &str) -> NavSession {
        NavSession {
            session_id: session_id.into(),
            execution_identity: execution_identity_for(session_id, 0),
            local_owner_runtime_generation: 1,
            motion_admission: NavMotionAdmission {
                guard_generation: 7,
                guard_boot_nonce: [7; 16],
                motion_stop_generation: 1,
            },
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
            payload_hash: [u8::try_from(path_revision).unwrap_or(u8::MAX); 32],
            path: Path2D::stamped(
                NavFrameId::Map,
                Timestamp::default(),
                vec![Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)],
            ),
        }
    }

    fn path() -> VersionedPath {
        path_for("goal-1", 1)
    }

    fn map_pose_at(seconds: i64) -> PoseStamped {
        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        pose.stamp = Timestamp::from_unix(seconds, 0);
        pose
    }

    fn map_pose() -> PoseStamped {
        map_pose_at(1)
    }

    fn costmap_at(source_timestamp_ns: i64) -> CostmapObservation {
        CostmapObservation {
            costmap: CostMap2D::new_all_free("map", 4, 4, 0.1),
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
                costmap: CostMap2D::new_all_free("map", 4, 4, 0.1),
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
        let backend = production_local_backend(LocalExecutionBackendKind::BasicInProcess);
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
        mutation.path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(3.0, 0.0), Point2::new(4.0, 0.0)],
        );
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
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
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
                costmap: CostMap2D::new_all_free("map", 4, 4, 0.1),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("costmap");
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle.submit_path(path()).await.expect("path");

        let result = tokio::time::timeout(Duration::from_secs(1), task)
            .await
            .expect("scheduler failure timeout")
            .expect("join");
        assert!(result.is_err());
        let calls = &motion_state.lock().expect("motion").calls;
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
        assert!(!handle.status().motion_sink_ready);
        let terminal = handle.status();
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert_eq!(terminal.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(
            terminal.message,
            "SafetyFailed cause=Motion: motion guard operation failed: injected APPLIED timeout"
        );
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
}
