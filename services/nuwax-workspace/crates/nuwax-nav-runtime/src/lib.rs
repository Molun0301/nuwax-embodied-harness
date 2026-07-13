//! Guard-backed, single-owner navigation scheduler core.
//!
//! This crate deliberately stops at the process-neutral scheduling boundary.
//! It owns one mutable motion port, serializes lease renewal and commands, and
//! accepts only versioned paths. Production LCM wiring remains disabled until
//! the goal/path envelope can carry the same session and revision fields.

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::path::PathBuf;
use std::time::Duration;

use async_trait::async_trait;
use nuwax_motion_safety_guard::wire::TwistCommand;
use nuwax_motion_safety_guard::{
    ClientConfig, MotionGuardClient, MotionGuardClientError, MAX_FRAME_BYTES,
};
use nuwax_nav_astar::local_planner::{LocalPlanner, StopMessage};
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

/// One navigation session and the oldest map revision it may consume.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NavSession {
    /// Stable goal/session identifier.
    pub session_id: String,
    /// Map revision observed when the goal was accepted.
    pub map_revision: u64,
}

/// A planner path with the correlation fields required for safe cutover.
#[derive(Debug, Clone, PartialEq)]
pub struct VersionedPath {
    /// Goal/session identifier copied from the versioned goal envelope.
    pub session_id: String,
    /// Map revision used to produce this path.
    pub map_revision: u64,
    /// Monotonic path revision within a map revision.
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
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchedulerStatus {
    /// Current scheduler phase.
    pub phase: SchedulerPhase,
    /// Active session, if any.
    pub session_id: Option<String>,
    /// Latest accepted map revision.
    pub map_revision: u64,
    /// Latest accepted path revision.
    pub path_revision: u64,
    /// Commands whose correlated APPLIED proof was validated.
    pub applied_commands: u64,
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
            map_revision: 0,
            path_revision: 0,
            applied_commands: 0,
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
    /// Collision footprint width passed to the local planner.
    pub robot_width_m: f64,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            tick_interval: DEFAULT_TICK_INTERVAL,
            renew_interval: MAX_RENEW_INTERVAL,
            robot_width_m: 0.3,
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
    /// APPLIED proof did not correlate to the submitted twist.
    #[error("invalid APPLIED proof: {0}")]
    InvalidApplied(String),
    /// Fence proof was incomplete or not ARMED + IDLE.
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
}

/// Correlated proof that one twist reached APPLIED.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppliedProof {
    /// Operation identifier echoed by the port contract.
    pub operation_id: String,
    /// Guard-side monotonic application timestamp.
    pub applied_monotonic_ns: u64,
}

/// Safety-gate state carried by a fence proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceGate {
    /// Guard accepts ordinary leased operation.
    Armed,
    /// Guard is disarmed.
    Disarmed,
    /// Guard requires an explicit reset.
    ResetRequired,
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
    /// Terminal safety gate.
    pub gate: FenceGate,
    /// Terminal lease state.
    pub lease: FenceLeaseState,
}

/// Mutable motion boundary owned by exactly one scheduler task.
#[async_trait]
pub trait NavMotionPort: Send + 'static {
    /// Acquire the NAV lease for one session.
    async fn acquire(&mut self, operation_id: &str) -> Result<LeaseProof, MotionPortError>;
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
}

#[async_trait]
impl NavMotionPort for MotionGuardClient {
    async fn acquire(&mut self, operation_id: &str) -> Result<LeaseProof, MotionPortError> {
        let metadata = match self.acquire_lease(operation_id).await {
            Ok(metadata) => metadata,
            Err(error) => {
                let definite_no_lease = matches!(
                    &error,
                    MotionGuardClientError::InvalidInput(_)
                        | MotionGuardClientError::Remote { .. }
                        | MotionGuardClientError::NotReady { .. }
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
            gate: FenceGate::Armed,
            lease: FenceLeaseState::Idle,
        })
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

/// Synchronous deterministic planner boundary.
pub trait NavPlanner: Send + 'static {
    /// Replace the active plan after version validation.
    fn replace_path(&mut self, path: Path2D);
    /// Update the latest odometry sample.
    fn update_odometry(&mut self, odometry: PoseStamped);
    /// Produce one controller step.
    fn step(&mut self, robot_width_m: f64) -> PlannerStep;
    /// Clear planner state after a terminal fence.
    fn stop(&mut self);
}

impl NavPlanner for LocalPlanner {
    fn replace_path(&mut self, path: Path2D) {
        self.start_planning(path);
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.handle_odom(odometry);
    }

    fn step(&mut self, robot_width_m: f64) -> PlannerStep {
        let (twist, stop) = LocalPlanner::step(self, robot_width_m);
        match stop {
            None => PlannerStep::Command(twist),
            Some(StopMessage::Arrived) => PlannerStep::Succeeded,
            Some(StopMessage::ObstacleFound) => PlannerStep::Failed("obstacle found".into()),
            Some(StopMessage::Error) => PlannerStep::Failed("local planner error".into()),
        }
    }

    fn stop(&mut self) {
        self.stop_planning();
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
    current_map_revision: u64,
    current_path_revision: u64,
    current_payload_hash: Option<[u8; 32]>,
    current_path: Option<Path2D>,
    operation_sequence: u64,
    status: SchedulerStatus,
}

impl<M, P> NavScheduler<M, P>
where
    M: NavMotionPort,
    P: NavPlanner,
{
    /// Construct a scheduler without starting a task.
    pub fn new(motion: M, planner: P, config: SchedulerConfig) -> Result<Self, RuntimeError> {
        validate_config(config)?;
        Ok(Self {
            motion,
            planner,
            config,
            session: None,
            lease_active: false,
            current_map_revision: 0,
            current_path_revision: 0,
            current_payload_hash: None,
            current_path: None,
            operation_sequence: 0,
            status: SchedulerStatus::default(),
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
                    if let Err(error) = self.renew().await {
                        let error = self.fail_closed(&status_tx, error).await;
                        return Err(error);
                    }
                }

                command = input_rx.recv(), if input_open => {
                    match command {
                        Some(command) => self.handle_input(&status_tx, command).await,
                        None => input_open = false,
                    }
                }

                _ = tick.tick(), if self.status.phase == SchedulerPhase::Controlling => {
                    if let Err(error) = self.renew().await {
                        let error = self.fail_closed(&status_tx, error).await;
                        return Err(error);
                    }
                    match urgent_rx.try_recv() {
                        Ok(command) => {
                            if let Some(result) = self.handle_urgent(&status_tx, command).await {
                                return result;
                            }
                            continue;
                        }
                        Err(mpsc::error::TryRecvError::Disconnected) => urgent_open = false,
                        Err(mpsc::error::TryRecvError::Empty) => {}
                    }
                    self.control_tick(&status_tx).await?;
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
            InputKind::UpdateOdometry(odometry) => {
                self.planner.update_odometry(odometry);
                Ok(())
            }
        };
        let _ = command.reply.send(result);
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
        self.current_map_revision = session.map_revision;
        self.current_path_revision = 0;
        self.current_payload_hash = None;
        self.current_path = None;
        self.status = SchedulerStatus {
            phase: SchedulerPhase::Acquiring,
            session_id: Some(session.session_id.clone()),
            map_revision: session.map_revision,
            path_revision: 0,
            applied_commands: 0,
            last_terminal: None,
            last_zero_ack_id: None,
            last_fence_monotonic_ns: 0,
            message: "acquiring NAV lease".into(),
        };
        self.publish_status(status_tx);

        let operation_id = self.next_operation_id("acquire")?;
        match self.motion.acquire(&operation_id).await {
            Ok(proof) if proof.lease_id != 0 => {
                self.lease_active = true;
                self.status.phase = SchedulerPhase::AwaitingPath;
                self.status.message = "NAV lease active; awaiting versioned path".into();
                self.publish_status(status_tx);
                Ok(())
            }
            Ok(_) => {
                self.lease_active = true;
                let error = RuntimeError::Motion("guard returned lease_id=0".into());
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
                self.current_map_revision = 0;
                self.current_path_revision = 0;
                self.current_payload_hash = None;
                self.current_path = None;
                self.status.phase = SchedulerPhase::Failed;
                self.status.session_id = None;
                self.status.map_revision = 0;
                self.status.path_revision = 0;
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
        if path.session_id != session.session_id {
            return Err(path_rejected(format!(
                "session {} does not match active {}",
                path.session_id, session.session_id
            )));
        }
        if path.map_revision < session.map_revision || path.map_revision < self.current_map_revision
        {
            return Err(path_rejected("map revision is stale"));
        }

        if path.map_revision == self.current_map_revision {
            if path.path_revision < self.current_path_revision {
                return Err(path_rejected("path revision is stale"));
            }
            if path.path_revision == self.current_path_revision && self.current_path_revision != 0 {
                let immutable = self.current_payload_hash == Some(path.payload_hash)
                    && self.current_path.as_ref() == Some(&path.path);
                if immutable {
                    return Ok(());
                }
                let error = RuntimeError::PathMutation {
                    reason: format!(
                        "session={} map_revision={} path_revision={} changed payload",
                        path.session_id, path.map_revision, path.path_revision
                    ),
                };
                self.finish_terminal(
                    status_tx,
                    TerminalOutcome::SafetyFailed,
                    SchedulerPhase::Failed,
                )
                .await?;
                return Err(error);
            }
        }

        self.planner.replace_path(path.path.clone());
        self.current_map_revision = path.map_revision;
        self.current_path_revision = path.path_revision;
        self.current_payload_hash = Some(path.payload_hash);
        self.current_path = Some(path.path);
        self.status.phase = SchedulerPhase::Controlling;
        self.status.map_revision = self.current_map_revision;
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
        match self.planner.step(self.config.robot_width_m) {
            PlannerStep::Command(twist) => {
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
                self.status.message = "twist APPLIED".into();
                self.publish_status(status_tx);
                Ok(())
            }
            PlannerStep::Succeeded => {
                self.finish_terminal(status_tx, TerminalOutcome::Succeeded, SchedulerPhase::Idle)
                    .await
            }
            PlannerStep::Failed(reason) => {
                self.status.message = format!("planner failed: {reason}; fencing");
                self.publish_status(status_tx);
                self.finish_terminal(
                    status_tx,
                    TerminalOutcome::PlannerFailed,
                    SchedulerPhase::Failed,
                )
                .await
            }
        }
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
        match self
            .finish_terminal(
                status_tx,
                TerminalOutcome::SafetyFailed,
                SchedulerPhase::Failed,
            )
            .await
        {
            Ok(()) => original,
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
        let had_lease = self.lease_active;
        let mut verified_fence = None;
        if had_lease {
            if self.status.phase != SchedulerPhase::Cancelling {
                self.status.phase = SchedulerPhase::Fencing;
                self.status.message = format!("waiting for {terminal:?} fence proof");
                self.publish_status(status_tx);
            }
            let operation_id = self.next_operation_id("fence")?;
            let proof = match self.motion.revoke_and_zero(&operation_id).await {
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
        self.current_map_revision = 0;
        self.current_path_revision = 0;
        self.current_payload_hash = None;
        self.current_path = None;
        self.status.phase = phase_after;
        self.status.session_id = None;
        self.status.map_revision = 0;
        self.status.path_revision = 0;
        self.status.last_terminal = Some(terminal);
        if let Some(proof) = verified_fence {
            self.status.last_zero_ack_id = Some(proof.zero_ack_id);
            self.status.last_fence_monotonic_ns = proof.applied_monotonic_ns;
            self.status.message = format!("{terminal:?} after verified fence");
        } else {
            self.status.message = format!("{terminal:?} with no active lease");
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

struct InputCommand {
    kind: InputKind,
    reply: oneshot::Sender<Result<(), RuntimeError>>,
}

enum InputKind {
    StartSession(NavSession),
    SubmitPath(VersionedPath),
    UpdateOdometry(PoseStamped),
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
    if !config.robot_width_m.is_finite() || config.robot_width_m <= 0.0 {
        return Err(RuntimeError::InvalidInput {
            field: "robot_width_m",
            reason: "must be finite and positive".into(),
        });
    }
    Ok(())
}

fn validate_session(session: &NavSession) -> Result<(), RuntimeError> {
    if session.session_id.trim().is_empty() {
        return Err(RuntimeError::InvalidInput {
            field: "session_id",
            reason: "must not be empty".into(),
        });
    }
    if session.map_revision == 0 {
        return Err(RuntimeError::InvalidInput {
            field: "map_revision",
            reason: "must be non-zero".into(),
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
    if path.map_revision == 0 || path.path_revision == 0 {
        return Err(RuntimeError::InvalidInput {
            field: "path.revision",
            reason: "map and path revisions must be non-zero".into(),
        });
    }
    if path.payload_hash == [0; 32] {
        return Err(RuntimeError::InvalidInput {
            field: "path.payload_hash",
            reason: "must not be all zero".into(),
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
    if proof.gate != FenceGate::Armed || proof.lease != FenceLeaseState::Idle {
        return Err(RuntimeError::InvalidFence(format!(
            "expected ARMED+IDLE, got {:?}+{:?}",
            proof.gate, proof.lease
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
