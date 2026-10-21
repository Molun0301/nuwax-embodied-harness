//! Persistent fail-closed client for the private motion-guard Unix socket.
//!
//! The client owns one persistent connection and one NAV-scoped lease. It does
//! not reconnect after an ambiguous transport or protocol failure and it does
//! not spawn a renewal task; the NAV runtime remains the only scheduler owner.

use std::fmt;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use prost::Message;
use rustix::time::{clock_gettime, ClockId};
use thiserror::Error;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;
use tokio::time::timeout;
use uuid::Uuid;

use crate::constants::{
    COMMAND_DEADLINE_MS, DEFAULT_REQUEST_TIMEOUT_MS, DEFAULT_UDS_PATH, MAX_FRAME_BYTES,
    RENEW_INTERVAL_MS,
};
use crate::crypto::{constant_time_eq, sha256};
use crate::wire::{
    motion_guard_request, motion_guard_response, submit_motion_command_request, AckCode,
    AcquireLeaseRequest, AcquireLeaseResponse, GetGuardStateRequest, GetGuardStateResponse,
    GetOperationResultRequest, GetOperationResultResponse, GetReadinessRequest, GlobalStopEnvelope,
    GlobalStopMode, GuardAction, GuardResult, GuardSource, LeaseState, LeaseTuple,
    MotionGuardRequest, MotionGuardResponse, OperationNamespace, RenewLeaseRequest,
    RenewLeaseResponse, RequestEnvelope, RevokeAndZeroRequest, StopAllMotionRequest, StopCommand,
    SubmitMotionCommandRequest, TwistCommand, ZeroAck, API_VERSION,
};

const BOOT_NONCE_BYTES: usize = 16;
const NANOS_PER_SECOND: u64 = 1_000_000_000;
const NANOS_PER_MILLISECOND: u64 = 1_000_000;

/// Connection and framing settings for [`MotionGuardClient`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClientConfig {
    /// Private motion-guard Unix socket path.
    pub socket_path: PathBuf,
    /// Maximum duration of one write plus its correlated response read.
    pub request_timeout: Duration,
    /// Maximum protobuf body size, excluding the four-byte prefix.
    pub max_frame_bytes: usize,
}

impl Default for ClientConfig {
    fn default() -> Self {
        Self {
            socket_path: PathBuf::from(DEFAULT_UDS_PATH),
            request_timeout: Duration::from_millis(DEFAULT_REQUEST_TIMEOUT_MS),
            max_frame_bytes: MAX_FRAME_BYTES,
        }
    }
}

/// Typed NAV readiness returned by the guard.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadinessStatus {
    /// Whether a NAV lease may be acquired now.
    pub ready: bool,
    /// Whether the production actuator was configured.
    pub actuator_configured: bool,
    /// Whether this boot epoch is durable.
    pub epoch_durable: bool,
    /// Guard-provided diagnostic reason.
    pub reason: String,
    /// Durable boot generation.
    pub generation: u64,
    /// Sixteen-byte boot nonce.
    pub boot_nonce: Vec<u8>,
    /// Whether the automatic startup zero handshake completed.
    pub startup_zero_confirmed: bool,
    /// Guard-owned global-stop barrier sampled with this readiness result.
    pub motion_stop_generation: u64,
}

/// Immutable request-level proof authorizing one direct lease acquisition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionAdmissionProof {
    /// Durable guard generation observed by the request admission layer.
    pub generation: u64,
    /// Sixteen-byte guard boot nonce observed with `generation`.
    pub boot_nonce: [u8; BOOT_NONCE_BYTES],
    /// Guard-owned global-stop barrier observed by request admission.
    pub motion_stop_generation: u64,
}

/// Lease acquisition result and the required renewal cadence.
#[derive(Debug, Clone, PartialEq)]
pub struct LeaseMetadata {
    /// Authoritative epoch and lease tuple.
    pub lease_tuple: LeaseTuple,
    /// Absolute lease deadline on Linux `CLOCK_MONOTONIC`.
    pub lease_deadline_monotonic_ns: u64,
    /// Current owner watchdog deadline on Linux `CLOCK_MONOTONIC`.
    pub owner_deadline_monotonic_ns: u64,
    /// Frozen interval at which the NAV scheduler must call `renew_lease`.
    pub renew_interval: Duration,
}

/// Metadata returned after one correlated lease renewal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RenewalMetadata {
    /// 服务端本次续租确认的租约窗口截止。
    pub lease_deadline_monotonic_ns: u64,
    /// Updated owner watchdog deadline.
    pub owner_deadline_monotonic_ns: u64,
    /// Frozen interval for the next scheduler-owned renewal.
    pub renew_interval: Duration,
}

/// Correlated terminal confirmation for an applied motion command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommandConfirmation {
    /// Caller-provided idempotency key.
    pub operation_id: String,
    /// Guard-side monotonic time sampled after actuator application.
    pub applied_monotonic_ns: u64,
}

/// Correlated proof that the actuator accepted a zero command.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ZeroConfirmation {
    /// Non-empty guard-generated zero acknowledgement id.
    pub zero_ack_id: String,
    /// Guard-side monotonic time sampled after zero application.
    pub applied_monotonic_ns: u64,
    /// 是否取得实际速度或保证制动的停稳证明。
    pub stationary_confirmed: bool,
    /// 命令确认、待确认或最终证明种类。
    pub confirmation_kind: String,
    /// 实测停稳反馈的来源时间，未使用反馈时为零。
    pub feedback_source_timestamp_ns: u64,
    /// 同机单调时钟下的确认截止，零表示没有在途确认。
    pub stationary_deadline_monotonic_ns: u64,
}

/// Result of an ordinary global stop operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GlobalStopConfirmation {
    /// Correlated proof that the actuator accepted zero.
    pub zero: ZeroConfirmation,
    /// Whether an active lease was revoked by this stop.
    pub lease_revoked: bool,
}

/// Exact proof that one durable recovery fence is currently held by the guard.
#[derive(Clone, PartialEq, Eq)]
pub struct HeldRecoveryFenceProof {
    /// Idempotency key of the completed hold operation.
    pub operation_id: String,
    /// Stable correlation shared by hold, Geometry recovery, and release.
    pub recovery_correlation: String,
    /// Guard boot generation returned with the durable state proof.
    pub generation: u64,
    /// Exact sixteen-byte guard boot nonce.
    pub boot_nonce: [u8; BOOT_NONCE_BYTES],
    /// Guard-owned global-stop generation produced by the hold operation.
    pub motion_stop_generation: u64,
    /// Correlated actuator zero acknowledgement produced by the hold operation.
    pub zero: ZeroConfirmation,
    /// Secret 256-bit recovery-fence token returned only by the hold operation.
    pub recovery_fence_token: [u8; 32],
}

impl fmt::Debug for HeldRecoveryFenceProof {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("HeldRecoveryFenceProof")
            .field("operation_id", &self.operation_id)
            .field("recovery_correlation", &self.recovery_correlation)
            .field("generation", &self.generation)
            .field("boot_nonce", &self.boot_nonce)
            .field("motion_stop_generation", &self.motion_stop_generation)
            .field("zero", &self.zero)
            .field("recovery_fence_token", &"[REDACTED]")
            .finish()
    }
}

/// One sealed priority fence request that may be replayed byte-for-byte.
#[derive(Debug, Clone)]
pub struct PriorityFenceRequest {
    request: MotionGuardRequest,
}

/// Observable client failures.
#[derive(Debug, Error)]
pub enum MotionGuardClientError {
    /// Local configuration or caller input violates the wire contract.
    #[error("invalid motion-guard client input: {0}")]
    InvalidInput(String),
    /// 观测绑定的分发截止已到；本次请求确定尚未发送。
    #[error("motion dispatch expired before sending `{operation_id}`")]
    DispatchExpired {
        /// 尚未发送的运动操作幂等键。
        operation_id: String,
    },
    /// Initial connection or an established stream failed.
    #[error("motion-guard transport failed: {0}")]
    Transport(String),
    /// A request exceeded its configured write/read deadline.
    #[error("motion-guard request timed out; outcome for `{operation_id}` is unknown")]
    Timeout {
        /// Mutation operation id, or an empty string for readiness.
        operation_id: String,
    },
    /// The peer returned bytes that violate correlation or protobuf semantics.
    #[error("motion-guard protocol violation: {0}")]
    Protocol(String),
    /// The guard returned a typed application error.
    #[error("motion guard rejected `{operation_id}` with {code}: {message}")]
    Remote {
        /// Stable guard error code.
        code: String,
        /// Guard diagnostic message.
        message: String,
        /// Whether the guard classifies the failure as retryable.
        retryable: bool,
        /// Mutation operation id, or an empty string for readiness.
        operation_id: String,
    },
    /// A prior timeout, EOF, framing error, or protocol mismatch poisoned the stream.
    #[error("motion-guard client is poisoned after: {reason}")]
    Poisoned {
        /// First failure that made the connection unusable.
        reason: String,
    },
}

/// Persistent NAV-scoped private-UDS client.
///
/// All methods take `&mut self`, so requests, renewals, commands, and fencing
/// have one strict order on the single connection. A timeout or protocol
/// mismatch closes the connection and permanently poisons this value.
pub struct MotionGuardClient {
    stream: Option<UnixStream>,
    config: ClientConfig,
    lease_tuple: Option<LeaseTuple>,
    sequence: u64,
    poison_reason: Option<String>,
    require_stationary: bool,
    measured_motion: crate::measured_motion::MeasurementCache,
    renewal_due_at: Option<tokio::time::Instant>,
}

impl MotionGuardClient {
    /// Connects one persistent NAV client to `socket_path`.
    pub async fn connect(socket_path: impl AsRef<Path>) -> Result<Self, MotionGuardClientError> {
        let config = ClientConfig {
            socket_path: socket_path.as_ref().to_path_buf(),
            ..ClientConfig::default()
        };
        Self::connect_with_config(config).await
    }

    /// Connects one persistent NAV client using explicit bounds.
    pub async fn connect_with_config(config: ClientConfig) -> Result<Self, MotionGuardClientError> {
        validate_config(&config)?;
        let stream = timeout(
            config.request_timeout,
            UnixStream::connect(&config.socket_path),
        )
        .await
        .map_err(|_| MotionGuardClientError::Transport("Unix socket connect timed out".into()))?
        .map_err(|error| MotionGuardClientError::Transport(error.to_string()))?;
        Ok(Self {
            stream: Some(stream),
            config,
            lease_tuple: None,
            sequence: 0,
            renewal_due_at: None,
            measured_motion: Default::default(),
            poison_reason: None,
            require_stationary: std::env::var("NUWAX_NAV_CAPABILITIES_JSON").ok().is_some_and(|v| !v.trim().is_empty()),
        })
    }

    /// Fence one known NAV lease over a fresh, independent UDS connection.
    ///
    /// This is the priority path used when the scheduler drops an in-flight
    /// ordinary exchange. `u64::MAX` is reserved for this terminal operation,
    /// so any delayed ordinary request is either ordered before the fence or
    /// rejected as stale after it. The guard actor remains the single actuator
    /// serialization owner.
    pub async fn priority_revoke_and_zero_with_lease(
        config: ClientConfig,
        lease_tuple: LeaseTuple,
        operation_id: &str,
    ) -> Result<ZeroConfirmation, MotionGuardClientError> {
        let request = Self::prepare_priority_revoke_and_zero(lease_tuple, operation_id)?;
        Self::execute_priority_revoke_and_zero(config, &request).await
    }

    /// Seal one terminal request for byte-identical retries on fresh streams.
    pub fn prepare_priority_revoke_and_zero(
        lease_tuple: LeaseTuple,
        operation_id: &str,
    ) -> Result<PriorityFenceRequest, MotionGuardClientError> {
        validate_epoch(lease_tuple.generation, &lease_tuple.boot_nonce)
            .map_err(MotionGuardClientError::InvalidInput)?;
        if lease_tuple.lease_id == 0 {
            return Err(MotionGuardClientError::InvalidInput(
                "priority fence requires a non-zero lease id".into(),
            ));
        }
        let operation_id = validate_operation_id(operation_id)?;
        let issued = monotonic_ns()?;
        let deadline = issued
            .checked_add(u64::from(COMMAND_DEADLINE_MS) * NANOS_PER_MILLISECOND)
            .ok_or_else(|| {
                MotionGuardClientError::InvalidInput("monotonic deadline overflow".into())
            })?;
        let envelope = RequestEnvelope {
            tuple: Some(lease_tuple),
            source: GuardSource::Nav as i32,
            action: GuardAction::Stop as i32,
            operation_id,
            payload_sha256: Vec::new(),
            sequence: u64::MAX,
            issued_monotonic_ns: issued,
            deadline_monotonic_ns: deadline,
            motion_stop_generation: 0,
        };
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::RevokeAndZero(
                RevokeAndZeroRequest {
                    envelope: Some(envelope),
                },
            )),
        };
        seal_request(&mut request)?;
        Ok(PriorityFenceRequest { request })
    }

    /// Execute a previously sealed priority fence over one fresh connection.
    pub async fn execute_priority_revoke_and_zero(
        config: ClientConfig,
        request: &PriorityFenceRequest,
    ) -> Result<ZeroConfirmation, MotionGuardClientError> {
        let mut client = Self::connect_with_config(config).await?;
        let outcome = client
            .exchange(
                request.request.clone(),
                OutcomeKind::RevokeAndZero,
                AckCode::ResourceFenced,
            )
            .await?;
        let motion_guard_response::Outcome::RevokeAndZero(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        response
            .zero_ack
            .ok_or_else(|| "RESOURCE_FENCED response has no ZeroAck".to_string())
            .and_then(validate_zero_ack)
            .or_else(|message| client.protocol_error(message))
    }

    /// 紧急命令围栏完成后，在独立连接查询原操作的停稳终态；不占用300毫秒命令预算。
    pub async fn confirm_stationary_after_fence(config:ClientConfig,operation_id:&str,zero:ZeroConfirmation)->Result<ZeroConfirmation,MotionGuardClientError> {
        if zero.stationary_deadline_monotonic_ns==0 {
            if !zero.stationary_confirmed && std::env::var("NUWAX_NAV_CAPABILITIES_JSON").ok().is_some_and(|v|!v.trim().is_empty()) {
                return Err(MotionGuardClientError::Protocol("已标定运动链缺少实际停稳或保证制动证明".into()));
            }
            return Ok(zero);
        }
        let mut client=Self::connect_with_config(config).await?;
        client.wait_stationary(operation_id,GuardSource::Nav,OperationNamespace::Lease,zero).await
    }

    async fn wait_stationary(&mut self,operation_id:&str,source:GuardSource,namespace:OperationNamespace,zero:ZeroConfirmation)->Result<ZeroConfirmation,MotionGuardClientError> {
        let deadline=zero.stationary_deadline_monotonic_ns;
        if deadline==0 {
            if self.require_stationary && !zero.stationary_confirmed {
                return self.protocol_error("已标定运动链缺少实际停稳或保证制动证明");
            }
            return Ok(zero);
        }
        if deadline>monotonic_ns()?.saturating_add(32_000_000_000) {return self.protocol_error("停稳确认截止超出有界预算");}
        loop {
            // 原等待期限结束后仍查询一次：同一停车操作可能已在后台取得迟到确认。
            let request=MotionGuardRequest {api_version:API_VERSION,request_id:new_request_id(),operation:Some(motion_guard_request::Operation::GetOperationResult(GetOperationResultRequest {
                source:source as i32,operation_id:operation_id.into(),operation_namespace:namespace as i32,
            }))};
            let outcome=self.exchange_allowing_acks(request,OutcomeKind::GetOperationResult,&[AckCode::Zeroed,AckCode::ResourceFenced]).await?;
            let motion_guard_response::Outcome::GetOperationResult(value)=outcome else {unreachable!("已检查响应种类")};
            if !value.resolved || value.target_operation_id!=operation_id {return self.protocol_error("停稳查询与原停止操作不一致");}
            let observed=value.zero_ack.ok_or_else(||"停稳查询缺少原归零标识".to_owned()).and_then(validate_zero_ack).or_else(|e|self.protocol_error(e))?;
            if observed.zero_ack_id!=zero.zero_ack_id {return self.protocol_error("停稳查询更换了原归零标识");}
            if observed.stationary_confirmed {return Ok(observed);}
            if observed.confirmation_kind=="unconfirmed" {break;}
            if monotonic_ns()?>deadline {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        Err(MotionGuardClientError::Remote{code:"MOTION_STOP_UNCONFIRMED".into(),message:"执行器未在保证制动期限内确认停稳".into(),retryable:false,operation_id:operation_id.into()})
    }

    /// Returns the source identity fixed for every request from this client.
    #[must_use]
    pub const fn source(&self) -> GuardSource {
        GuardSource::Nav
    }

    /// Returns the frozen scheduler-owned renewal interval (250 ms).
    #[must_use]
    pub const fn renew_interval(&self) -> Duration {
        Duration::from_millis(RENEW_INTERVAL_MS)
    }

    /// 已验证服务端截止对应的续租时刻；普通动作不会顺延该时刻。
    #[must_use]
    pub fn renewal_due_at(&self) -> Option<tokio::time::Instant> {
        self.lease_tuple.as_ref().and(self.renewal_due_at)
    }

    fn schedule_renewal(&mut self, issued_ns: u64, owner_deadline_ns: u64) -> Result<(), MotionGuardClientError> {
        // 预留完整通信上限和守卫调度片，命令执行期间仍由同一拥有方串行续租。
        let reserve = 425_000_000 + 25_000_000
            + self.config.request_timeout.min(Duration::from_millis(COMMAND_DEADLINE_MS as u64)).as_nanos() as u64;
        let due_ns = issued_ns.saturating_add(RENEW_INTERVAL_MS * NANOS_PER_MILLISECOND)
            .min(owner_deadline_ns.saturating_sub(reserve));
        let sampled_at = tokio::time::Instant::now();
        let remaining = due_ns.saturating_sub(monotonic_ns()?);
        self.renewal_due_at = Some(sampled_at + Duration::from_nanos(remaining));
        Ok(())
    }

    /// Returns whether this connection was permanently poisoned.
    #[must_use]
    pub fn is_poisoned(&self) -> bool {
        self.poison_reason.is_some()
    }

    /// 取得原期限内的真实运动快照，与 APPLIED 命令证明互不替代。
    pub fn measured_motion(&self) -> Option<crate::MeasuredMotion> {
        if self.is_poisoned() { None } else { self.measured_motion.current() }
    }

    /// Returns the active lease tuple, if acquisition succeeded and fencing has not.
    #[must_use]
    pub fn active_lease(&self) -> Option<&LeaseTuple> {
        self.lease_tuple.as_ref()
    }

    /// Queries and strictly validates NAV readiness on the persistent stream.
    pub async fn get_readiness(&mut self) -> Result<ReadinessStatus, MotionGuardClientError> {
        let request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::GetReadiness(
                GetReadinessRequest {
                    source: GuardSource::Nav as i32,
                },
            )),
        };
        let outcome = self
            .exchange(request, OutcomeKind::GetReadiness, AckCode::Received)
            .await?;
        let motion_guard_response::Outcome::GetReadiness(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        validate_epoch(response.generation, &response.boot_nonce)
            .or_else(|message| self.protocol_error(message))?;
        Ok(ReadinessStatus {
            ready: response.ready,
            actuator_configured: response.actuator_configured,
            epoch_durable: response.epoch_durable,
            reason: response.reason,
            generation: response.generation,
            boot_nonce: response.boot_nonce,
            startup_zero_confirmed: response.startup_zero_confirmed,
            motion_stop_generation: response.motion_stop_generation,
        })
    }

    /// Verifies that a correlated global-stop operation still owns the durable recovery fence.
    ///
    /// The operation journal is checked first and the current guard state second, so a concurrent
    /// release or a guard reboot makes verification fail closed before Geometry mutates ownership.
    pub async fn verify_held_recovery_fence(
        &mut self,
        proof: &HeldRecoveryFenceProof,
    ) -> Result<(), MotionGuardClientError> {
        validate_held_recovery_fence_proof(proof).map_err(MotionGuardClientError::InvalidInput)?;

        let operation_request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::GetOperationResult(
                GetOperationResultRequest {
                    source: GuardSource::GatewayManual as i32,
                    operation_id: proof.operation_id.trim().to_string(),
                    operation_namespace: OperationNamespace::GlobalStop as i32,
                },
            )),
        };
        let operation_outcome = self
            .exchange_allowing_acks(
                operation_request,
                OutcomeKind::GetOperationResult,
                &[AckCode::Zeroed, AckCode::ResourceFenced],
            )
            .await?;
        let motion_guard_response::Outcome::GetOperationResult(operation) = operation_outcome
        else {
            unreachable!("exchange already correlated the outcome")
        };

        if operation.zero_ack.as_ref().is_some_and(|zero|zero.stationary_deadline_monotonic_ns>0 || zero.confirmation_kind=="unconfirmed" || (self.require_stationary && !zero.stationary_confirmed)) {
            return self.protocol_error("恢复围栏的原停止操作尚未取得停稳证明");
        }

        let state_request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::GetGuardState(
                GetGuardStateRequest {
                    source: GuardSource::GatewayManual as i32,
                },
            )),
        };
        let state_outcome = self
            .exchange(state_request, OutcomeKind::GetGuardState, AckCode::Received)
            .await?;
        let motion_guard_response::Outcome::GetGuardState(state) = state_outcome else {
            unreachable!("exchange already correlated the outcome")
        };

        validate_held_recovery_fence_responses(proof, &operation, &state)
            .or_else(|message| self.protocol_error(message))
    }

    /// Applies an ordinary global stop without changing future admission policy.
    pub async fn stop_all_motion(
        &mut self,
        operation_id: &str,
        reason: &str,
    ) -> Result<GlobalStopConfirmation, MotionGuardClientError> {
        self.ensure_usable()?;
        let operation_id = validate_operation_id(operation_id)?;
        let reason = reason.trim();
        if reason.is_empty() {
            return Err(MotionGuardClientError::InvalidInput(
                "global stop reason must not be empty".into(),
            ));
        }
        let issued = monotonic_ns()?;
        let deadline = issued
            .checked_add(u64::from(COMMAND_DEADLINE_MS) * NANOS_PER_MILLISECOND)
            .ok_or_else(|| {
                MotionGuardClientError::InvalidInput("monotonic deadline overflow".into())
            })?;
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::StopAllMotion(
                StopAllMotionRequest {
                    envelope: Some(GlobalStopEnvelope {
                        source: GuardSource::GatewayManual as i32,
                        operation_id: operation_id.clone(),
                        payload_sha256: Vec::new(),
                        issued_monotonic_ns: issued,
                        deadline_monotonic_ns: deadline,
                    }),
                    reason: reason.to_string(),
                    mode: GlobalStopMode::Ordinary as i32,
                    recovery_correlation: String::new(),
                    recovery_fence_token: Vec::new(),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange_allowing_acks(
                request,
                OutcomeKind::StopAllMotion,
                &[AckCode::Zeroed, AckCode::ResourceFenced],
            )
            .await?;
        let motion_guard_response::Outcome::StopAllMotion(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        let zero = response
            .zero_ack
            .ok_or_else(|| "global stop response has no ZeroAck".to_string())
            .and_then(validate_zero_ack)
            .or_else(|message| self.protocol_error(message))?;
        self.lease_tuple = None;
        self.sequence = 0;
        let zero=self.wait_stationary(&operation_id,GuardSource::GatewayManual,OperationNamespace::GlobalStop,zero).await?;
        Ok(GlobalStopConfirmation {
            zero,
            lease_revoked: response.lease_revoked,
        })
    }

    /// Acquires a source-scoped lease directly from a request-level admission proof.
    pub async fn acquire_lease(
        &mut self,
        operation_id: &str,
        proof: &MotionAdmissionProof,
    ) -> Result<LeaseMetadata, MotionGuardClientError> {
        self.ensure_usable()?;
        if self.lease_tuple.is_some() {
            return Err(MotionGuardClientError::InvalidInput(
                "a NAV lease is already active".into(),
            ));
        }
        let operation_id = validate_operation_id(operation_id)?;
        validate_epoch(proof.generation, &proof.boot_nonce)
            .map_err(MotionGuardClientError::InvalidInput)?;
        let epoch_tuple = LeaseTuple {
            generation: proof.generation,
            boot_nonce: proof.boot_nonce.to_vec(),
            lease_id: 0,
        };
        let mut envelope =
            self.new_envelope(epoch_tuple, GuardAction::Unspecified, &operation_id, 0)?;
        envelope.motion_stop_generation = proof.motion_stop_generation;
        let issued_ns = envelope.issued_monotonic_ns;
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::AcquireLease(
                AcquireLeaseRequest {
                    envelope: Some(envelope),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange(request, OutcomeKind::AcquireLease, AckCode::Received)
            .await?;
        let motion_guard_response::Outcome::AcquireLease(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        let metadata = self.validate_acquire(response, proof)?;
        self.sequence = 0;
        self.lease_tuple = Some(metadata.lease_tuple.clone());
        self.schedule_renewal(issued_ns, metadata.owner_deadline_monotonic_ns)?;
        Ok(metadata)
    }

    /// Renews the active NAV lease using the next strict sequence number.
    pub async fn renew_lease(
        &mut self,
        operation_id: &str,
    ) -> Result<RenewalMetadata, MotionGuardClientError> {
        let operation_id = validate_operation_id(operation_id)?;
        let envelope = self.next_envelope(GuardAction::Unspecified, &operation_id)?;
        let issued_ns = envelope.issued_monotonic_ns;
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::RenewLease(
                RenewLeaseRequest {
                    envelope: Some(envelope),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange(request, OutcomeKind::RenewLease, AckCode::Received)
            .await?;
        let motion_guard_response::Outcome::RenewLease(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        let metadata = self.validate_renewal(response)?;
        self.schedule_renewal(issued_ns, metadata.owner_deadline_monotonic_ns)?;
        Ok(metadata)
    }

    /// Submits a typed twist and waits for a correlated `APPLIED` acknowledgement.
    pub async fn submit_twist(
        &mut self,
        operation_id: &str,
        twist: TwistCommand,
    ) -> Result<CommandConfirmation, MotionGuardClientError> {
        self.submit_twist_with_dispatch_deadline(operation_id, twist, None)
            .await
    }

    /// 使用固定的观测截止约束接收端准入，仍等待相关联的 `APPLIED` 确认。
    ///
    /// 截止只限制接收端准入，不缩短传输超时或变更独立的速度租约。
    /// 本地已到截止时返回 [`MotionGuardClientError::DispatchExpired`]，不发送请求。
    pub async fn submit_twist_before(
        &mut self,
        operation_id: &str,
        twist: TwistCommand,
        dispatch_before: Instant,
    ) -> Result<CommandConfirmation, MotionGuardClientError> {
        self.submit_twist_with_dispatch_deadline(operation_id, twist, Some(dispatch_before))
            .await
    }

    async fn submit_twist_with_dispatch_deadline(
        &mut self,
        operation_id: &str,
        twist: TwistCommand,
        dispatch_before: Option<Instant>,
    ) -> Result<CommandConfirmation, MotionGuardClientError> {
        validate_twist(&twist)?;
        let operation_id = validate_operation_id(operation_id)?;
        let mut envelope = self.next_envelope(GuardAction::Twist, &operation_id)?;
        if let Some(dispatch_before) = dispatch_before {
            // next_envelope 已先读取原始 CLOCK_MONOTONIC，再采样 Instant；
            // 两次读取的间隔只会提前截止，不能把观测证明延长到固定截止之后。
            let sampled_now = Instant::now();
            envelope.deadline_monotonic_ns = command_deadline_monotonic_ns(
                envelope.issued_monotonic_ns,
                Some((dispatch_before, sampled_now)),
                &operation_id,
            )?;
        }
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::SubmitMotionCommand(
                SubmitMotionCommandRequest {
                    envelope: Some(envelope),
                    command: Some(submit_motion_command_request::Command::Twist(twist)),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange(request, OutcomeKind::SubmitMotionCommand, AckCode::Applied)
            .await?;
        let motion_guard_response::Outcome::SubmitMotionCommand(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        if response.applied_monotonic_ns == 0 {
            return self.protocol_error("APPLIED response has no application timestamp");
        }
        if response.zero_ack.is_some() {
            return self.protocol_error("twist APPLIED response unexpectedly contains ZeroAck");
        }
        Ok(CommandConfirmation {
            operation_id,
            applied_monotonic_ns: response.applied_monotonic_ns,
        })
    }

    /// Submits a typed stop and waits for its non-empty `ZEROED` acknowledgement.
    pub async fn submit_stop(
        &mut self,
        operation_id: &str,
    ) -> Result<ZeroConfirmation, MotionGuardClientError> {
        let operation_id = validate_operation_id(operation_id)?;
        let envelope = self.next_envelope(GuardAction::Stop, &operation_id)?;
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::SubmitMotionCommand(
                SubmitMotionCommandRequest {
                    envelope: Some(envelope),
                    command: Some(submit_motion_command_request::Command::Stop(StopCommand {})),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange(request, OutcomeKind::SubmitMotionCommand, AckCode::Zeroed)
            .await?;
        let motion_guard_response::Outcome::SubmitMotionCommand(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        let zero = response
            .zero_ack
            .ok_or_else(|| "ZEROED stop response has no ZeroAck".to_string())
            .and_then(validate_zero_ack)
            .or_else(|message| self.protocol_error(message))?;
        if response.applied_monotonic_ns == 0 {
            return self.protocol_error("ZEROED stop response has no application timestamp");
        }
        self.wait_stationary(&operation_id,GuardSource::Nav,OperationNamespace::Lease,zero).await
    }

    /// Atomically revokes the active lease, applies zero, and waits for fencing proof.
    ///
    /// The lease is cleared only after `RESOURCE_FENCED` and a non-empty `ZeroAck`
    /// correlate on the same request. Any timeout or mismatch poisons the client.
    pub async fn revoke_and_zero(
        &mut self,
        operation_id: &str,
    ) -> Result<ZeroConfirmation, MotionGuardClientError> {
        let operation_id = validate_operation_id(operation_id)?;
        let envelope = self.next_envelope(GuardAction::Stop, &operation_id)?;
        let mut request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: new_request_id(),
            operation: Some(motion_guard_request::Operation::RevokeAndZero(
                RevokeAndZeroRequest {
                    envelope: Some(envelope),
                },
            )),
        };
        seal_request(&mut request)?;
        let outcome = self
            .exchange(request, OutcomeKind::RevokeAndZero, AckCode::ResourceFenced)
            .await?;
        let motion_guard_response::Outcome::RevokeAndZero(response) = outcome else {
            unreachable!("exchange already correlated the outcome")
        };
        let zero = response
            .zero_ack
            .ok_or_else(|| "RESOURCE_FENCED response has no ZeroAck".to_string())
            .and_then(validate_zero_ack)
            .or_else(|message| self.protocol_error(message))?;
        self.lease_tuple = None;
        self.sequence = 0;
        self.wait_stationary(&operation_id,GuardSource::Nav,OperationNamespace::Lease,zero).await
    }

    fn validate_acquire(
        &mut self,
        response: AcquireLeaseResponse,
        proof: &MotionAdmissionProof,
    ) -> Result<LeaseMetadata, MotionGuardClientError> {
        let tuple = response
            .lease_tuple
            .ok_or_else(|| "AcquireLease response has no lease tuple".to_string())
            .or_else(|message| self.protocol_error(message))?;
        if tuple.generation != proof.generation
            || tuple.boot_nonce.as_slice() != proof.boot_nonce
            || tuple.lease_id == 0
        {
            return self.protocol_error("AcquireLease response has an invalid lease tuple");
        }
        validate_deadlines(
            response.lease_deadline_monotonic_ns,
            response.owner_deadline_monotonic_ns,
        )
        .or_else(|message| self.protocol_error(message))?;
        Ok(LeaseMetadata {
            lease_tuple: tuple,
            lease_deadline_monotonic_ns: response.lease_deadline_monotonic_ns,
            owner_deadline_monotonic_ns: response.owner_deadline_monotonic_ns,
            renew_interval: self.renew_interval(),
        })
    }

    fn validate_renewal(
        &mut self,
        response: RenewLeaseResponse,
    ) -> Result<RenewalMetadata, MotionGuardClientError> {
        validate_deadlines(
            response.lease_deadline_monotonic_ns,
            response.owner_deadline_monotonic_ns,
        )
        .or_else(|message| self.protocol_error(message))?;
        Ok(RenewalMetadata {
            lease_deadline_monotonic_ns: response.lease_deadline_monotonic_ns,
            owner_deadline_monotonic_ns: response.owner_deadline_monotonic_ns,
            renew_interval: self.renew_interval(),
        })
    }

    fn next_envelope(
        &mut self,
        action: GuardAction,
        operation_id: &str,
    ) -> Result<RequestEnvelope, MotionGuardClientError> {
        self.ensure_usable()?;
        let tuple = self
            .lease_tuple
            .clone()
            .ok_or_else(|| MotionGuardClientError::InvalidInput("no active NAV lease".into()))?;
        self.sequence = self.sequence.checked_add(1).ok_or_else(|| {
            MotionGuardClientError::InvalidInput("lease sequence exhausted u64".into())
        })?;
        self.new_envelope(tuple, action, operation_id, self.sequence)
    }

    fn new_envelope(
        &self,
        tuple: LeaseTuple,
        action: GuardAction,
        operation_id: &str,
        sequence: u64,
    ) -> Result<RequestEnvelope, MotionGuardClientError> {
        let issued = monotonic_ns()?;
        let deadline = command_deadline_monotonic_ns(issued, None, operation_id)?;
        Ok(RequestEnvelope {
            tuple: Some(tuple),
            source: GuardSource::Nav as i32,
            action: action as i32,
            operation_id: operation_id.to_string(),
            payload_sha256: Vec::new(),
            sequence,
            issued_monotonic_ns: issued,
            deadline_monotonic_ns: deadline,
            motion_stop_generation: 0,
        })
    }

    async fn exchange(
        &mut self,
        request: MotionGuardRequest,
        expected_outcome: OutcomeKind,
        expected_ack: AckCode,
    ) -> Result<motion_guard_response::Outcome, MotionGuardClientError> {
        self.exchange_allowing_acks(request, expected_outcome, &[expected_ack])
            .await
    }

    async fn exchange_allowing_acks(
        &mut self,
        request: MotionGuardRequest,
        expected_outcome: OutcomeKind,
        expected_acks: &[AckCode],
    ) -> Result<motion_guard_response::Outcome, MotionGuardClientError> {
        self.ensure_usable()?;
        if request.api_version != API_VERSION || request.request_id.is_empty() {
            return Err(MotionGuardClientError::InvalidInput(
                "request api_version and request_id must be set".into(),
            ));
        }
        if OutcomeKind::from_request(&request) != Some(expected_outcome) {
            return Err(MotionGuardClientError::InvalidInput(
                "request operation does not match expected outcome".into(),
            ));
        }
        let operation_id = request_operation_id(&request).unwrap_or_default();
        let request_id = request.request_id.clone();
        let payload = request.encode_to_vec();
        if payload.is_empty() || payload.len() > self.config.max_frame_bytes {
            return Err(MotionGuardClientError::InvalidInput(format!(
                "request protobuf size {} is outside frame bounds",
                payload.len()
            )));
        }

        let exchange_result = {
            let stream = self
                .stream
                .as_mut()
                .expect("ensure_usable verified the stream");
            timeout(
                self.config.request_timeout,
                exchange_frame(stream, &payload, self.config.max_frame_bytes),
            )
            .await
        };
        let response_payload = match exchange_result {
            Err(_) => {
                let error = MotionGuardClientError::Timeout {
                    operation_id: operation_id.clone(),
                };
                self.poison(error.to_string());
                return Err(error);
            }
            Ok(Err(message)) => {
                let error = MotionGuardClientError::Transport(message);
                self.poison(error.to_string());
                return Err(error);
            }
            Ok(Ok(payload)) => payload,
        };

        let response =
            MotionGuardResponse::decode(response_payload.as_slice()).map_err(|error| {
                let message = format!("invalid response protobuf: {error}");
                self.poison(message.clone());
                MotionGuardClientError::Protocol(message)
            })?;
        if response.api_version != API_VERSION {
            return self.protocol_error(format!(
                "response api_version {} does not match {}",
                response.api_version, API_VERSION
            ));
        }
        if response.request_id != request_id {
            return self.protocol_error("response request_id does not correlate to request");
        }
        let outcome = response
            .outcome
            .ok_or_else(|| "response outcome is missing".to_string())
            .or_else(|message| self.protocol_error(message))?;
        if OutcomeKind::from_response(&outcome) != Some(expected_outcome) {
            return self.protocol_error("response outcome does not correlate to request operation");
        }
        let result = outcome_result(&outcome)
            .ok_or_else(|| "response outcome has no GuardResult".to_string())
            .or_else(|message| self.protocol_error(message))?;
        let ack = AckCode::try_from(result.ack).map_err(|_| {
            let message = format!("response contains unknown ack value {}", result.ack);
            self.poison(message.clone());
            MotionGuardClientError::Protocol(message)
        })?;
        // 拒绝运动也可能携带新的真实观测；采样并不表示发生于 APPLIED 之后。
        self.measured_motion.update(response.measured_motion_feedback, monotonic_ns()?);
        if let Some(remote) = result.error.as_ref() {
            if remote.code.trim().is_empty() {
                return self.protocol_error("GuardResult error has an empty code");
            }
            return Err(MotionGuardClientError::Remote {
                code: remote.code.clone(),
                message: remote.message.clone(),
                retryable: remote.retryable,
                operation_id,
            });
        }
        if !expected_acks.contains(&ack) {
            return self.protocol_error(format!(
                "response ack {ack:?} is not one of {expected_acks:?}"
            ));
        }
        validate_success_state(expected_outcome, result)
            .or_else(|message| self.protocol_error(message))?;
        Ok(outcome)
    }

    fn ensure_usable(&self) -> Result<(), MotionGuardClientError> {
        if let Some(reason) = self.poison_reason.as_ref() {
            return Err(MotionGuardClientError::Poisoned {
                reason: reason.clone(),
            });
        }
        if self.stream.is_none() {
            return Err(MotionGuardClientError::Poisoned {
                reason: "connection is closed".into(),
            });
        }
        Ok(())
    }

    fn protocol_error<T>(
        &mut self,
        message: impl Into<String>,
    ) -> Result<T, MotionGuardClientError> {
        let message = message.into();
        self.poison(message.clone());
        Err(MotionGuardClientError::Protocol(message))
    }

    fn poison(&mut self, reason: String) {
        if self.poison_reason.is_none() {
            self.poison_reason = Some(reason);
        }
        self.stream.take();
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OutcomeKind {
    AcquireLease,
    RenewLease,
    SubmitMotionCommand,
    RevokeAndZero,
    StopAllMotion,
    GetReadiness,
    GetOperationResult,
    GetGuardState,
}

impl OutcomeKind {
    fn from_request(request: &MotionGuardRequest) -> Option<Self> {
        match request.operation.as_ref()? {
            motion_guard_request::Operation::AcquireLease(_) => Some(Self::AcquireLease),
            motion_guard_request::Operation::RenewLease(_) => Some(Self::RenewLease),
            motion_guard_request::Operation::SubmitMotionCommand(_) => {
                Some(Self::SubmitMotionCommand)
            }
            motion_guard_request::Operation::RevokeAndZero(_) => Some(Self::RevokeAndZero),
            motion_guard_request::Operation::StopAllMotion(_) => Some(Self::StopAllMotion),
            motion_guard_request::Operation::GetReadiness(_) => Some(Self::GetReadiness),
            motion_guard_request::Operation::GetOperationResult(_) => {
                Some(Self::GetOperationResult)
            }
            motion_guard_request::Operation::GetGuardState(_) => Some(Self::GetGuardState),
            motion_guard_request::Operation::ExecuteSportCommand(_)
            | motion_guard_request::Operation::RevokeZeroAndSwitchMode(_) => None,
        }
    }

    fn from_response(outcome: &motion_guard_response::Outcome) -> Option<Self> {
        match outcome {
            motion_guard_response::Outcome::AcquireLease(_) => Some(Self::AcquireLease),
            motion_guard_response::Outcome::RenewLease(_) => Some(Self::RenewLease),
            motion_guard_response::Outcome::SubmitMotionCommand(_) => {
                Some(Self::SubmitMotionCommand)
            }
            motion_guard_response::Outcome::RevokeAndZero(_) => Some(Self::RevokeAndZero),
            motion_guard_response::Outcome::StopAllMotion(_) => Some(Self::StopAllMotion),
            motion_guard_response::Outcome::GetReadiness(_) => Some(Self::GetReadiness),
            motion_guard_response::Outcome::GetOperationResult(_) => Some(Self::GetOperationResult),
            motion_guard_response::Outcome::GetGuardState(_) => Some(Self::GetGuardState),
            motion_guard_response::Outcome::ExecuteSportCommand(_)
            | motion_guard_response::Outcome::RevokeZeroAndSwitchMode(_) => None,
        }
    }
}

fn validate_config(config: &ClientConfig) -> Result<(), MotionGuardClientError> {
    if config.socket_path.as_os_str().is_empty() {
        return Err(MotionGuardClientError::InvalidInput(
            "socket_path must not be empty".into(),
        ));
    }
    if config.request_timeout.is_zero() {
        return Err(MotionGuardClientError::InvalidInput(
            "request_timeout must be positive".into(),
        ));
    }
    if config.max_frame_bytes == 0 || config.max_frame_bytes > u32::MAX as usize {
        return Err(MotionGuardClientError::InvalidInput(
            "max_frame_bytes must fit a non-zero u32".into(),
        ));
    }
    Ok(())
}

fn validate_operation_id(operation_id: &str) -> Result<String, MotionGuardClientError> {
    let operation_id = operation_id.trim();
    if operation_id.is_empty() {
        return Err(MotionGuardClientError::InvalidInput(
            "operation_id must not be empty".into(),
        ));
    }
    Ok(operation_id.to_string())
}

fn validate_epoch(generation: u64, boot_nonce: &[u8]) -> Result<(), String> {
    if generation == 0
        || boot_nonce.len() != BOOT_NONCE_BYTES
        || boot_nonce.iter().all(|byte| *byte == 0)
    {
        return Err("motion admission proof contains an invalid boot epoch".into());
    }
    Ok(())
}

fn validate_deadlines(lease_deadline: u64, owner_deadline: u64) -> Result<(), String> {
    let now = monotonic_ns().map_err(|error| error.to_string())?;
    if owner_deadline <= now || lease_deadline < owner_deadline {
        return Err("response contains invalid lease deadlines".into());
    }
    Ok(())
}

fn validate_twist(twist: &TwistCommand) -> Result<(), MotionGuardClientError> {
    if !twist.linear_x_mps.is_finite()
        || !twist.linear_y_mps.is_finite()
        || !twist.angular_z_rps.is_finite()
    {
        return Err(MotionGuardClientError::InvalidInput(
            "twist components must be finite".into(),
        ));
    }
    Ok(())
}

fn validate_zero_ack(zero: ZeroAck) -> Result<ZeroConfirmation, String> {
    if zero.zero_ack_id.trim().is_empty() || zero.applied_monotonic_ns == 0 {
        return Err("terminal ZeroAck is incomplete".into());
    }
    Ok(ZeroConfirmation {
        zero_ack_id: zero.zero_ack_id,
        applied_monotonic_ns: zero.applied_monotonic_ns,
        stationary_confirmed: zero.stationary_confirmed,
        confirmation_kind: zero.confirmation_kind,
        feedback_source_timestamp_ns: zero.feedback_source_timestamp_ns,
        stationary_deadline_monotonic_ns: zero.stationary_deadline_monotonic_ns,
    })
}

fn validate_held_recovery_fence_proof(proof: &HeldRecoveryFenceProof) -> Result<(), String> {
    validate_operation_id(&proof.operation_id).map_err(|error| error.to_string())?;
    if proof.recovery_correlation.trim().is_empty() {
        return Err("recovery fence correlation must not be empty".into());
    }
    validate_epoch(proof.generation, &proof.boot_nonce)?;
    if proof.motion_stop_generation == 0
        || proof.zero.zero_ack_id.trim().is_empty()
        || proof.zero.applied_monotonic_ns == 0
        || proof.recovery_fence_token.iter().all(|byte| *byte == 0)
    {
        return Err("recovery fence proof is incomplete".into());
    }
    Ok(())
}

fn validate_held_recovery_fence_responses(
    proof: &HeldRecoveryFenceProof,
    operation: &GetOperationResultResponse,
    state: &GetGuardStateResponse,
) -> Result<(), String> {
    if !operation.resolved || operation.target_operation_id != proof.operation_id.trim() {
        return Err("recovery fence hold operation is not resolved".into());
    }
    let target_ack = AckCode::try_from(operation.target_ack)
        .map_err(|_| "recovery fence hold has an unknown terminal ack".to_string())?;
    if !matches!(target_ack, AckCode::Zeroed | AckCode::ResourceFenced) {
        return Err("recovery fence hold has no terminal zero ack".into());
    }
    let operation_zero = operation
        .zero_ack
        .as_ref()
        .ok_or_else(|| "recovery fence hold operation has no ZeroAck".to_string())?;
    if operation_zero.zero_ack_id != proof.zero.zero_ack_id
        || operation_zero.applied_monotonic_ns != proof.zero.applied_monotonic_ns
        || operation.motion_stop_generation != proof.motion_stop_generation
        || !operation.recovery_fence_held
        || !constant_time_eq(&operation.recovery_fence_token, &proof.recovery_fence_token)
    {
        return Err("recovery fence hold operation does not match the supplied proof".into());
    }

    validate_epoch(state.generation, &state.boot_nonce)?;
    let state_lease = state
        .result
        .as_ref()
        .and_then(|result| LeaseState::try_from(result.lease).ok());
    if state.generation != proof.generation
        || state.boot_nonce.as_slice() != proof.boot_nonce
        || state.motion_stop_generation != proof.motion_stop_generation
        || state.last_zero_ack_id != proof.zero.zero_ack_id
        || !state.fenced
        || state.active_lease_id != 0
        || state_lease != Some(LeaseState::Idle)
        || !state.recovery_fence_held
    {
        return Err("current guard state does not hold the supplied recovery fence".into());
    }
    let token_sha256 = sha256(&proof.recovery_fence_token);
    let correlation_sha256 = sha256(proof.recovery_correlation.trim().as_bytes());
    if !constant_time_eq(&state.recovery_fence_token_sha256, &token_sha256)
        || !constant_time_eq(
            &state.recovery_fence_correlation_sha256,
            &correlation_sha256,
        )
    {
        return Err("current guard state hashes do not match the recovery fence".into());
    }
    Ok(())
}

fn validate_success_state(kind: OutcomeKind, result: &GuardResult) -> Result<(), String> {
    let lease = LeaseState::try_from(result.lease)
        .map_err(|_| format!("response contains unknown lease state {}", result.lease))?;
    let expected = match kind {
        OutcomeKind::AcquireLease | OutcomeKind::RenewLease | OutcomeKind::SubmitMotionCommand => {
            Some(LeaseState::Active)
        }
        OutcomeKind::RevokeAndZero | OutcomeKind::StopAllMotion => Some(LeaseState::Idle),
        OutcomeKind::GetReadiness
        | OutcomeKind::GetOperationResult
        | OutcomeKind::GetGuardState => None,
    };
    if let Some(expected_lease) = expected {
        if lease != expected_lease {
            return Err(format!(
                "response lease state {lease:?} does not match {expected_lease:?}"
            ));
        }
    }
    Ok(())
}

fn monotonic_ns() -> Result<u64, MotionGuardClientError> {
    let now = clock_gettime(ClockId::Monotonic);
    let seconds = u64::try_from(now.tv_sec).map_err(|_| {
        MotionGuardClientError::InvalidInput("CLOCK_MONOTONIC returned negative seconds".into())
    })?;
    let nanoseconds = u64::try_from(now.tv_nsec).map_err(|_| {
        MotionGuardClientError::InvalidInput("CLOCK_MONOTONIC returned negative nanos".into())
    })?;
    seconds
        .checked_mul(NANOS_PER_SECOND)
        .and_then(|value| value.checked_add(nanoseconds))
        .ok_or_else(|| MotionGuardClientError::InvalidInput("monotonic clock overflow".into()))
}

/// 将固定 Instant 截止保守地映射到先采样的原始单调时钟；不跨 await 传递剩余预算。
fn command_deadline_monotonic_ns(
    issued: u64,
    dispatch_sample: Option<(Instant, Instant)>,
    operation_id: &str,
) -> Result<u64, MotionGuardClientError> {
    let ordinary_window = Duration::from_millis(u64::from(COMMAND_DEADLINE_MS));
    let ordinary_deadline = issued
        .checked_add(u64::from(COMMAND_DEADLINE_MS) * NANOS_PER_MILLISECOND)
        .ok_or_else(|| {
            MotionGuardClientError::InvalidInput("monotonic deadline overflow".into())
        })?;
    let Some((dispatch_before, sampled_now)) = dispatch_sample else {
        return Ok(ordinary_deadline);
    };
    let remaining = dispatch_before
        .checked_duration_since(sampled_now)
        .filter(|remaining| !remaining.is_zero())
        .ok_or_else(|| MotionGuardClientError::DispatchExpired {
            operation_id: operation_id.to_string(),
        })?;
    // 先取较小窗口再作整数转换，避免远期截止造成无关溢出；绝不补一纳秒。
    let remaining_ns = u64::try_from(remaining.min(ordinary_window).as_nanos()).map_err(|_| {
        MotionGuardClientError::InvalidInput("dispatch deadline duration overflow".into())
    })?;
    let dispatch_deadline = issued.checked_add(remaining_ns).ok_or_else(|| {
        MotionGuardClientError::InvalidInput("dispatch monotonic deadline overflow".into())
    })?;
    Ok(ordinary_deadline.min(dispatch_deadline))
}

fn new_request_id() -> String {
    format!("request:{}", Uuid::new_v4())
}

async fn exchange_frame(
    stream: &mut UnixStream,
    request: &[u8],
    max_frame_bytes: usize,
) -> Result<Vec<u8>, String> {
    let length = u32::try_from(request.len())
        .map_err(|_| "request frame length does not fit u32".to_string())?;
    stream
        .write_all(&length.to_be_bytes())
        .await
        .map_err(|error| format!("failed to write frame prefix: {error}"))?;
    stream
        .write_all(request)
        .await
        .map_err(|error| format!("failed to write frame body: {error}"))?;
    stream
        .flush()
        .await
        .map_err(|error| format!("failed to flush frame: {error}"))?;

    let mut prefix = [0_u8; 4];
    stream
        .read_exact(&mut prefix)
        .await
        .map_err(|error| format!("failed to read response prefix: {error}"))?;
    let response_length = u32::from_be_bytes(prefix) as usize;
    if response_length == 0 {
        return Err("zero-length protobuf response frame".into());
    }
    if response_length > max_frame_bytes {
        return Err(format!(
            "response frame length {response_length} exceeds {max_frame_bytes}"
        ));
    }
    let mut response = vec![0_u8; response_length];
    stream
        .read_exact(&mut response)
        .await
        .map_err(|error| format!("failed to read response body: {error}"))?;
    Ok(response)
}

fn request_operation_id(request: &MotionGuardRequest) -> Option<String> {
    match request.operation.as_ref()? {
        motion_guard_request::Operation::AcquireLease(body) => body
            .envelope
            .as_ref()
            .map(|value| value.operation_id.clone()),
        motion_guard_request::Operation::RenewLease(body) => body
            .envelope
            .as_ref()
            .map(|value| value.operation_id.clone()),
        motion_guard_request::Operation::SubmitMotionCommand(body) => body
            .envelope
            .as_ref()
            .map(|value| value.operation_id.clone()),
        motion_guard_request::Operation::RevokeAndZero(body) => body
            .envelope
            .as_ref()
            .map(|value| value.operation_id.clone()),
        motion_guard_request::Operation::StopAllMotion(body) => body
            .envelope
            .as_ref()
            .map(|value| value.operation_id.clone()),
        motion_guard_request::Operation::GetOperationResult(body) => {
            Some(body.operation_id.clone())
        }
        _ => None,
    }
}

fn outcome_result(outcome: &motion_guard_response::Outcome) -> Option<&GuardResult> {
    match outcome {
        motion_guard_response::Outcome::AcquireLease(body) => body.result.as_ref(),
        motion_guard_response::Outcome::RenewLease(body) => body.result.as_ref(),
        motion_guard_response::Outcome::SubmitMotionCommand(body) => body.result.as_ref(),
        motion_guard_response::Outcome::RevokeAndZero(body) => body.result.as_ref(),
        motion_guard_response::Outcome::StopAllMotion(body) => body.result.as_ref(),
        motion_guard_response::Outcome::GetReadiness(body) => body.result.as_ref(),
        motion_guard_response::Outcome::GetOperationResult(body) => body.result.as_ref(),
        motion_guard_response::Outcome::GetGuardState(body) => body.result.as_ref(),
        motion_guard_response::Outcome::ExecuteSportCommand(_)
        | motion_guard_response::Outcome::RevokeZeroAndSwitchMode(_) => None,
    }
}

fn seal_request(request: &mut MotionGuardRequest) -> Result<(), MotionGuardClientError> {
    let payload = canonical_payload(request)?;
    let digest = sha256(&payload).to_vec();
    if let Some(motion_guard_request::Operation::StopAllMotion(body)) = request.operation.as_mut() {
        let envelope = body.envelope.as_mut().ok_or_else(|| {
            MotionGuardClientError::InvalidInput("global stop request has no envelope".into())
        })?;
        envelope.payload_sha256 = digest;
    } else {
        let envelope = operation_envelope_mut(request).ok_or_else(|| {
            MotionGuardClientError::InvalidInput("mutation request has no envelope".into())
        })?;
        envelope.payload_sha256 = digest;
    }
    Ok(())
}

fn canonical_payload(request: &MotionGuardRequest) -> Result<Vec<u8>, MotionGuardClientError> {
    match request.operation.as_ref() {
        Some(motion_guard_request::Operation::AcquireLease(body)) => {
            Ok(encode_without_envelope(body.clone()))
        }
        Some(motion_guard_request::Operation::RenewLease(body)) => {
            Ok(encode_without_envelope(body.clone()))
        }
        Some(motion_guard_request::Operation::SubmitMotionCommand(body)) => {
            Ok(encode_without_envelope(body.clone()))
        }
        Some(motion_guard_request::Operation::RevokeAndZero(body)) => {
            Ok(encode_without_envelope(body.clone()))
        }
        Some(motion_guard_request::Operation::StopAllMotion(body)) => {
            Ok(encode_without_envelope(body.clone()))
        }
        _ => Err(MotionGuardClientError::InvalidInput(
            "operation has no canonical mutation payload".into(),
        )),
    }
}

trait EnvelopeMessage: Message {
    fn clear_envelope(&mut self);
}

impl EnvelopeMessage for AcquireLeaseRequest {
    fn clear_envelope(&mut self) {
        self.envelope = None;
    }
}

impl EnvelopeMessage for RenewLeaseRequest {
    fn clear_envelope(&mut self) {
        self.envelope = None;
    }
}

impl EnvelopeMessage for SubmitMotionCommandRequest {
    fn clear_envelope(&mut self) {
        self.envelope = None;
    }
}

impl EnvelopeMessage for RevokeAndZeroRequest {
    fn clear_envelope(&mut self) {
        self.envelope = None;
    }
}

impl EnvelopeMessage for StopAllMotionRequest {
    fn clear_envelope(&mut self) {
        self.envelope = None;
    }
}

fn encode_without_envelope<T: EnvelopeMessage>(mut message: T) -> Vec<u8> {
    message.clear_envelope();
    message.encode_to_vec()
}

fn operation_envelope_mut(request: &mut MotionGuardRequest) -> Option<&mut RequestEnvelope> {
    match request.operation.as_mut()? {
        motion_guard_request::Operation::AcquireLease(body) => body.envelope.as_mut(),
        motion_guard_request::Operation::RenewLease(body) => body.envelope.as_mut(),
        motion_guard_request::Operation::SubmitMotionCommand(body) => body.envelope.as_mut(),
        motion_guard_request::Operation::RevokeAndZero(body) => body.envelope.as_mut(),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dispatch_deadline_rejects_at_and_after_fixed_bound_without_one_ns_extension() {
        let deadline = Instant::now();
        let one_ns = Duration::from_nanos(1);
        let before = deadline.checked_sub(one_ns).unwrap();
        let after = deadline.checked_add(one_ns).unwrap();
        assert_eq!(
            command_deadline_monotonic_ns(1_000, Some((deadline, before)), "twist:bound")
                .unwrap(),
            1_001
        );
        for sample in [deadline, after] {
            assert!(matches!(
                command_deadline_monotonic_ns(1_000, Some((deadline, sample)), "twist:bound"),
                Err(MotionGuardClientError::DispatchExpired { operation_id })
                    if operation_id == "twist:bound"
            ));
        }
    }

    #[test]
    fn dispatch_deadline_conversion_uses_later_sample_without_refreshing_fixed_bound() {
        let raw_sample_instant = Instant::now();
        let deadline = raw_sample_instant + Duration::from_nanos(100);
        let delayed_sample = raw_sample_instant + Duration::from_nanos(7);
        let converted = command_deadline_monotonic_ns(
            1_000,
            Some((deadline, delayed_sample)),
            "twist:bound",
        )
        .unwrap();
        // 原始时钟先读，Instant 后读：7ns 的读取间隔只缩短准入窗口。
        assert_eq!(converted, 1_093);
        assert!(converted < 1_100);
        let later_sample = delayed_sample + Duration::from_nanos(10);
        assert_eq!(
            command_deadline_monotonic_ns(1_010, Some((deadline, later_sample)), "twist:bound")
                .unwrap(),
            converted
        );
    }

    #[test]
    fn dispatch_deadline_never_extends_the_ordinary_300_ms_window() {
        let sampled_now = Instant::now();
        let ordinary_window = Duration::from_millis(300);
        assert_eq!(COMMAND_DEADLINE_MS, 300);
        assert_eq!(command_deadline_monotonic_ns(123, None, "ordinary").unwrap(), 300_000_123);
        for remaining in [ordinary_window, ordinary_window + Duration::from_nanos(1)] {
            assert_eq!(
                command_deadline_monotonic_ns(
                    123,
                    Some((sampled_now + remaining, sampled_now)),
                    "twist:bound",
                )
                .unwrap(),
                300_000_123
            );
        }
        assert_eq!(
            command_deadline_monotonic_ns(
                123,
                Some((sampled_now + ordinary_window - Duration::from_nanos(1), sampled_now)),
                "twist:bound",
            )
            .unwrap(),
            300_000_122
        );
    }

    #[test]
    fn dispatch_deadline_caps_duration_before_u64_conversion_and_checks_addition() {
        let sampled_now = Instant::now();
        let beyond_u64_nanos = Duration::from_secs(u64::MAX / NANOS_PER_SECOND + 1);
        assert!(beyond_u64_nanos.as_nanos() > u128::from(u64::MAX));
        let far_deadline = sampled_now.checked_add(beyond_u64_nanos).unwrap();
        assert_eq!(
            command_deadline_monotonic_ns(
                123,
                Some((far_deadline, sampled_now)),
                "twist:far",
            )
            .unwrap(),
            300_000_123
        );
        let last_valid_issue = u64::MAX - 300_000_000;
        assert_eq!(
            command_deadline_monotonic_ns(last_valid_issue, None, "ordinary").unwrap(),
            u64::MAX
        );
        for sample in [None, Some((far_deadline, sampled_now))] {
            assert!(matches!(
                command_deadline_monotonic_ns(last_valid_issue + 1, sample, "overflow"),
                Err(MotionGuardClientError::InvalidInput(_))
            ));
        }
    }

    #[tokio::test]
    #[ignore = "本云端 Unix socket 操作受环境限制；在支持本地 UDS 的环境单列运行"]
    async fn real_uds_measurement_is_independent_of_applied_and_updates_on_rejection() {
        let (stream,mut peer)=UnixStream::pair().unwrap();
        let mut client=MotionGuardClient {stream:Some(stream),config:ClientConfig::default(),
            lease_tuple:Some(LeaseTuple{generation:9,boot_nonce:vec![7;BOOT_NONCE_BYTES],lease_id:41}),
            sequence:0,poison_reason:None,require_stationary:false,renewal_due_at:None,
            measured_motion:Default::default()};
        let server=tokio::spawn(async move {
            for index in 0..3 {
                let size=peer.read_u32().await.unwrap();let mut bytes=vec![0;size as usize];
                peer.read_exact(&mut bytes).await.unwrap();
                let request=MotionGuardRequest::decode(bytes.as_slice()).unwrap();
                let now=super::monotonic_ns().unwrap();
                let motion=(index<2).then(||crate::wire::MeasuredMotionFeedback {
                    source_id:"test:actual-device".into(),source_timestamp_ns:100+index,
                    received_monotonic_ns:now,valid_until_monotonic_ns:now+1_000_000_000,
                    frame_id:"base_link".into(),body_linear_xyz:vec![index as f64*0.1,0.,0.],
                    body_angular_xyz:vec![0.,0.,0.],orientation_wxyz:vec![1.,0.,0.,0.],
                });
                let result=GuardResult {ack:if index==0 {AckCode::Applied as i32}else{AckCode::Received as i32},
                    lease:crate::wire::LeaseState::Active as i32,
                    error:(index==1).then(||crate::wire::ErrorDetail{code:"TEST_REJECT".into(),message:"rejected but observed".into(),retryable:false})};
                let outcome=if index==0 {
                    motion_guard_response::Outcome::SubmitMotionCommand(crate::wire::SubmitMotionCommandResponse {
                        result:Some(result),applied_monotonic_ns:now,zero_ack:None})
                } else {motion_guard_response::Outcome::GetReadiness(crate::wire::GetReadinessResponse {
                    result:Some(result),generation:9,boot_nonce:vec![7;BOOT_NONCE_BYTES],..Default::default()})};
                let reply=MotionGuardResponse {api_version:API_VERSION,request_id:request.request_id,
                    measured_motion_feedback:motion,outcome:Some(outcome)}.encode_to_vec();
                peer.write_u32(reply.len() as u32).await.unwrap();peer.write_all(&reply).await.unwrap();
            }
        });
        client.submit_twist("measured-vs-command",TwistCommand{linear_x_mps:0.8,linear_y_mps:0.,angular_z_rps:0.3}).await.unwrap();
        assert_eq!(client.measured_motion().unwrap().body_linear_xyz,[0.;3]);
        assert!(matches!(client.get_readiness().await,Err(MotionGuardClientError::Remote{code,..}) if code=="TEST_REJECT"));
        assert_eq!(client.measured_motion().unwrap().body_linear_xyz,[0.1,0.,0.]);
        client.get_readiness().await.unwrap();assert!(client.measured_motion().is_none());
        server.await.unwrap();
    }

    #[tokio::test]
    async fn preexpired_dispatch_sends_no_bytes_and_does_not_poison_connection() {
        let (stream, peer) = UnixStream::pair().unwrap();
        let config = ClientConfig::default();
        let request_timeout = config.request_timeout;
        let mut client = MotionGuardClient {
            stream: Some(stream),
            config,
            lease_tuple: Some(LeaseTuple {
                generation: 9,
                boot_nonce: vec![7; BOOT_NONCE_BYTES],
                lease_id: 41,
            }),
            sequence: 0,
            poison_reason: None,
            require_stationary: false,
            renewal_due_at: None,
            measured_motion: Default::default(),
        };
        let error = client
            .submit_twist_before(
                "twist:expired",
                TwistCommand {
                    linear_x_mps: 0.2,
                    linear_y_mps: 0.0,
                    angular_z_rps: 0.0,
                },
                Instant::now().checked_sub(Duration::from_nanos(1)).unwrap(),
            )
            .await
            .unwrap_err();
        assert!(matches!(
            error,
            MotionGuardClientError::DispatchExpired { operation_id }
                if operation_id == "twist:expired"
        ));
        let mut byte = [0_u8; 1];
        assert_eq!(peer.try_read(&mut byte).unwrap_err().kind(), std::io::ErrorKind::WouldBlock);
        assert_eq!(client.config.request_timeout, request_timeout);
        assert!(client.lease_tuple.is_some());
        assert!(client.poison_reason.is_none());
        client.ensure_usable().unwrap();
    }

    fn held_proof() -> HeldRecoveryFenceProof {
        HeldRecoveryFenceProof {
            operation_id: "hold:exploration-17".into(),
            recovery_correlation: "exploration-rollover:17".into(),
            generation: 9,
            boot_nonce: [7; BOOT_NONCE_BYTES],
            motion_stop_generation: 11,
            zero: ZeroConfirmation {
                zero_ack_id: "zero-hold-17".into(),
                applied_monotonic_ns: 13,
                ..Default::default()
            },
            recovery_fence_token: [5; 32],
        }
    }

    fn held_responses(
        proof: &HeldRecoveryFenceProof,
    ) -> (GetOperationResultResponse, GetGuardStateResponse) {
        let operation = GetOperationResultResponse {
            result: Some(GuardResult {
                ack: AckCode::Zeroed as i32,
                lease: LeaseState::Idle as i32,
                error: None,
            }),
            target_operation_id: proof.operation_id.clone(),
            target_ack: AckCode::Zeroed as i32,
            resolved: true,
            zero_ack: Some(ZeroAck {
                zero_ack_id: proof.zero.zero_ack_id.clone(),
                applied_monotonic_ns: proof.zero.applied_monotonic_ns,
                ..Default::default()
            }),
            motion_stop_generation: proof.motion_stop_generation,
            recovery_fence_token: proof.recovery_fence_token.to_vec(),
            recovery_fence_held: true,
        };
        let state = GetGuardStateResponse {
            last_zero_confirmation: None,
            result: Some(GuardResult {
                ack: AckCode::Received as i32,
                lease: LeaseState::Idle as i32,
                error: None,
            }),
            generation: proof.generation,
            boot_nonce: proof.boot_nonce.to_vec(),
            last_zero_ack_id: proof.zero.zero_ack_id.clone(),
            fenced: true,
            active_lease_id: 0,
            motion_stop_generation: proof.motion_stop_generation,
            recovery_fence_held: true,
            recovery_fence_token_sha256: sha256(&proof.recovery_fence_token).to_vec(),
            recovery_fence_correlation_sha256: sha256(proof.recovery_correlation.as_bytes())
                .to_vec(),
        };
        (operation, state)
    }

    #[test]
    fn held_recovery_fence_requires_exact_operation_and_live_state() {
        let proof = held_proof();
        let (operation, state) = held_responses(&proof);
        validate_held_recovery_fence_responses(&proof, &operation, &state).unwrap();

        let mut wrong_operation = operation.clone();
        wrong_operation.recovery_fence_token[0] ^= 1;
        assert!(validate_held_recovery_fence_responses(&proof, &wrong_operation, &state).is_err());

        let mut released_state = state.clone();
        released_state.recovery_fence_held = false;
        assert!(
            validate_held_recovery_fence_responses(&proof, &operation, &released_state).is_err()
        );

        let mut wrong_correlation = state;
        wrong_correlation.recovery_fence_correlation_sha256 = sha256(b"other").to_vec();
        assert!(
            validate_held_recovery_fence_responses(&proof, &operation, &wrong_correlation).is_err()
        );
    }

    #[test]
    fn held_recovery_fence_debug_redacts_secret_token() {
        let proof = held_proof();
        let rendered = format!("{proof:?}");
        assert!(rendered.contains("recovery_fence_token: \"[REDACTED]\""));
        assert!(!rendered.contains("5, 5, 5"));
        assert!(!rendered.contains(&"05".repeat(32)));
    }
}
