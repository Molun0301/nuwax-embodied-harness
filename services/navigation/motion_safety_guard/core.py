from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import hmac
import secrets
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple
import uuid

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import (
    API_VERSION,
    DEDUP_CAPACITY_DEFAULT,
    LEASE_RENEWAL_WINDOW_NS,
    MAX_COMMAND_DEADLINE_NS,
    OWNER_TIMEOUT_NS,
    WATCHDOG_FENCE_LEAD_NS,
    ZERO_RETRY_INITIAL_NS,
    ZERO_RETRY_MAX_NS,
)
from .epoch import EpochError, EpochRecord, EpochStore, MAX_GENERATION
from .go2_actuator import ActuatorBackend, ActuatorError
from .peer_auth import PeerCredentials
from .stop_confirmation import StopConfirmation, StopEvidence, StopProgress
from .wire import (
    WireContractError,
    operation_envelope,
    request_source_name,
    verify_payload_sha256,
)


class ProtocolViolation(ValueError):
    pass


@dataclass
class GuardFailure(Exception):
    code: str
    message: str
    retryable: bool = False


@dataclass(frozen=True)
class OperationRecord:
    key: Tuple[object, ...]
    operation_id: str
    payload_sha256: bytes
    source: str
    lease_id: int
    response_bytes: bytes
    ack: int
    zero_ack_id: str
    zero_applied_ns: int
    motion_stop_generation: int
    namespace: str
    recovery_fence_token: bytes = b""
    recovery_fence_held: bool = False
    acquire: bool = False
    stationary_zero_ack_bytes: bytes = b""


_LEASE_TO_EPOCH = {
    pb.LEASE_STATE_IDLE: "Idle",
    pb.LEASE_STATE_ACTIVE: "Active",
    pb.LEASE_STATE_PAUSING: "Pausing",
    pb.LEASE_STATE_REVOKING: "Revoking",
}

_SOURCE_ACTION_POLICY = {
    "NAV": frozenset(
        {
            pb.GUARD_ACTION_TWIST,
            pb.GUARD_ACTION_STOP,
            pb.GUARD_ACTION_MODE_SWITCH,
        }
    ),
    "GATEWAY_MANUAL": frozenset(
        {
            pb.GUARD_ACTION_TWIST,
            pb.GUARD_ACTION_STOP,
            pb.GUARD_ACTION_BODY_POSE,
            pb.GUARD_ACTION_SPEED_LEVEL,
            pb.GUARD_ACTION_OBSTACLE_POLICY,
        }
    ),
    "GATEWAY_FOLLOW": frozenset(
        {pb.GUARD_ACTION_TWIST, pb.GUARD_ACTION_STOP}
    ),
    "GATEWAY_RELATIVE": frozenset(
        {pb.GUARD_ACTION_TWIST, pb.GUARD_ACTION_STOP}
    ),
    "GATEWAY_SPORT": frozenset(
        {
            pb.GUARD_ACTION_TWIST,
            pb.GUARD_ACTION_STOP,
            pb.GUARD_ACTION_BODY_POSE,
            pb.GUARD_ACTION_SPEED_LEVEL,
            pb.GUARD_ACTION_OBSTACLE_POLICY,
            pb.GUARD_ACTION_MODE_SWITCH,
            pb.GUARD_ACTION_SPORT_ACTION,
        }
    ),
}

_QUERY_NAMESPACES = {
    pb.OPERATION_NAMESPACE_ACQUIRE: "acquire",
    pb.OPERATION_NAMESPACE_LEASE: "lease",
    pb.OPERATION_NAMESPACE_GLOBAL_STOP: "global_stop",
}


class MotionGuardCore:
    """Single-threaded arbitration state for the private motion-guard RPCs."""

    def __init__(
        self,
        *,
        epoch: EpochRecord,
        epoch_store: EpochStore,
        actuator: ActuatorBackend,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        lease_id_factory: Optional[Callable[[], int]] = None,
        dedup_capacity: int = DEDUP_CAPACITY_DEFAULT,
        stop_confirmation: Optional[StopConfirmation] = None,
    ) -> None:
        if dedup_capacity < 4:
            raise ValueError("dedup_capacity must reserve normal and priority entries")
        self.epoch = epoch
        self._epoch_store = epoch_store
        self._actuator = actuator
        self._stop_confirmation = stop_confirmation or StopConfirmation.from_environment()
        self._pending_stop = None
        self._last_zero_confirmation = None
        self._pending_zero_records = []
        self._execution_feedback_failure = ""
        self._monotonic_ns = monotonic_ns
        self._lease_id_factory = lease_id_factory or (
            lambda: secrets.randbits(64) or 1
        )
        self._epoch_durable = True
        self._actuator_configured = bool(actuator.is_configured) and (
            actuator.actuator_symbols_sha256 == epoch.actuator_symbols_sha256
        )
        self._actuator_reason = (
            "OK"
            if self._actuator_configured
            else "ACTUATOR_MANIFEST_MISMATCH_OR_UNCONFIGURED"
        )
        self.lease_state = pb.LEASE_STATE_IDLE
        self._startup_zero_confirmed = bool(epoch.startup_zero_confirmed)
        self._active_lease_id = 0
        self._active_source = ""
        self._last_sequence = 0
        self._lease_deadline_ns = 0
        self._owner_deadline_ns = 0
        self._watchdog_deadline_ns = 0
        self._motion_stop_generation = 0
        self._zero_retry_not_before_ns = 0
        self._zero_retry_delay_ns = ZERO_RETRY_INITIAL_NS
        self._state_lock = threading.RLock()
        self._ordinary_capacity = dedup_capacity - max(1, dedup_capacity // 4)
        self._priority_capacity = dedup_capacity - self._ordinary_capacity
        self._ordinary_records: Dict[Tuple[object, ...], OperationRecord] = {}
        self._priority_records: Dict[Tuple[object, ...], OperationRecord] = {}
        # 全局停车可先于原运动拥有方到达；保留被撤租者的精确归零票据供其收尾。
        self._externally_fenced_leases: Dict[Tuple[int, str], bytes] = {}
        self._pending_retired_lease: Optional[Tuple[int, str]] = None
        self._operation_index: Dict[str, List[OperationRecord]] = {}

    @property
    def active_lease_id(self) -> int:
        return self._active_lease_id

    @property
    def motion_stop_generation(self) -> int:
        return self._motion_stop_generation

    @property
    def startup_configured(self) -> bool:
        return self._epoch_durable and self._actuator_configured

    @property
    def startup_reason(self) -> str:
        if not self._epoch_durable:
            return "EPOCH_NOT_DURABLE"
        return self._actuator_reason

    def handle(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
    ) -> pb.MotionGuardResponse:
        operation = request.WhichOneof("operation")
        if operation is None:
            raise ProtocolViolation("MotionGuardRequest.operation is missing")
        if not request.request_id:
            return self._error_response(
                request,
                "INVALID_ARGUMENT",
                "request_id is required",
            )
        try:
            source = request_source_name(request)
            if source not in _SOURCE_ACTION_POLICY:
                raise GuardFailure(
                    "PERMISSION_DENIED",
                    "claimed source is unspecified or unsupported",
                )
            if request.api_version != API_VERSION:
                raise GuardFailure(
                    "UNSUPPORTED_API_VERSION",
                    f"api_version {request.api_version} is unsupported",
                )
            handler = getattr(self, f"_handle_{operation}")
            return handler(request, peer, source)
        except (WireContractError, ProtocolViolation) as exc:
            return self._error_response(request, "INVALID_ARGUMENT", str(exc))
        except GuardFailure as exc:
            return self._error_response(
                request,
                exc.code,
                exc.message,
                retryable=exc.retryable,
            )
        except EpochError as exc:
            self._clear_lease()
            self.lease_state = pb.LEASE_STATE_REVOKING
            self._startup_zero_confirmed = False
            return self._error_response(
                request,
                "EPOCH_DURABILITY_LOST",
                str(exc),
            )

    def tick(self, now_ns: Optional[int] = None) -> Optional[pb.ZeroAck]:
        with self._state_lock:
            now = self._monotonic_ns() if now_ns is None else now_ns
            completed = None
            if self._pending_stop is not None:
                try:
                    evidence = self._stop_confirmation.poll(self._actuator, self._pending_stop)
                    if evidence is None:
                        return None
                    completed = self._finish_stop_confirmation(evidence)
                    self._commit_epoch(startup_zero_confirmed=True)
                except ActuatorError as exc:
                    self._pending_stop = None
                    self._last_zero_confirmation.confirmation_kind = "unconfirmed"
                    self._last_zero_confirmation.stationary_deadline_monotonic_ns = 0
                    self._store_stop_confirmation()
                    self._mark_zero_unconfirmed(exc, now_ns=now)
                    return None
                except EpochError:
                    self.lease_state = pb.LEASE_STATE_REVOKING
                    self._startup_zero_confirmed = False
                    self._schedule_zero_retry(now)
                    return None
            if (
                self.lease_state == pb.LEASE_STATE_REVOKING
                and not self._startup_zero_confirmed
            ):
                if now < self._zero_retry_not_before_ns:
                    return None
                return self._retry_zero_fence(now)
            if self.lease_state != pb.LEASE_STATE_ACTIVE:
                return completed
            if now < min(self._watchdog_deadline_ns, self._lease_deadline_ns):
                if self._active_source == "NAV":
                    try:
                        self._check_execution_mode()
                    except ActuatorError as exc:
                        self._execution_feedback_failure = exc.code + ": " + exc.message
                        try:
                            self._zero()
                            self._commit_epoch(startup_zero_confirmed=True)
                        except ActuatorError as zero_error:
                            self._mark_zero_unconfirmed(zero_error, now_ns=now)
                        except EpochError:
                            self._startup_zero_confirmed = False
                return completed
            self.lease_state = pb.LEASE_STATE_REVOKING
            try:
                zero_ack = self._zero()
                self._clear_lease()
                self._commit_epoch(
                    lease=pb.LEASE_STATE_IDLE,
                    fenced=True,
                    last_zero_ack_id=zero_ack.zero_ack_id,
                    startup_zero_confirmed=True,
                )
                self._reset_zero_retry()
                return zero_ack
            except ActuatorError as exc:
                self._mark_zero_unconfirmed(exc, now_ns=now)
                return None
            except EpochError:
                self.lease_state = pb.LEASE_STATE_REVOKING
                self._startup_zero_confirmed = False
                self._schedule_zero_retry(now)
                return None

    def initialize_zero_fence(self, *, prepare_mode: bool = True, cancelled=lambda: False) -> pb.ZeroAck:
        """Zero all owned paths and make this boot eligible for new leases."""
        with self._state_lock:
            self.lease_state = pb.LEASE_STATE_REVOKING
            self._startup_zero_confirmed = False
            precommit_error: Optional[EpochError] = None
            try:
                self._commit_epoch(
                    lease=pb.LEASE_STATE_REVOKING,
                    fenced=True,
                    startup_zero_confirmed=False,
                )
            except EpochError as exc:
                precommit_error = exc
            try:
                zero_ack = self._zero(wait=True)
                requirement = self._stop_confirmation.locomotion
                if requirement is not None and prepare_mode and precommit_error is None:
                    try:
                        changed = requirement.prepare(self._actuator, self._read_execution_feedback, self._stop_confirmation.note_motion, cancelled)
                        if changed:
                            zero_ack = self._zero(wait=True)
                        requirement.check(self._read_execution_feedback)
                    except ActuatorError as mode_error:
                        # 模式请求可能部分生效，失败后仍经原停止路径确认机体。
                        try:
                            self._zero(wait=True)
                        except ActuatorError as stop_error:
                            raise ActuatorError("MOTION_STOP_UNCONFIRMED", mode_error.message + "; 停止清理也失败：" + stop_error.message) from mode_error
                        raise
            except ActuatorError as exc:
                self._mark_zero_unconfirmed(exc)
                raise
            self._clear_lease()
            if precommit_error is not None:
                raise precommit_error
            self._commit_epoch(
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                startup_zero_confirmed=True,
            )
            self._reset_zero_retry()
            return zero_ack

    def _handle_get_readiness(
        self,
        request: pb.MotionGuardRequest,
        _peer: PeerCredentials,
        _source: str,
    ) -> pb.MotionGuardResponse:
        with self._state_lock:
            recovery_fenced = bool(self.epoch.recovery_fence_token_hex)
            operational = (
                self._epoch_durable
                and self._actuator_configured
                and self._startup_zero_confirmed
                and not recovery_fenced
            )
            if not self._epoch_durable:
                reason = "EPOCH_NOT_DURABLE"
            elif not self._actuator_configured:
                reason = self._actuator_reason
            elif self._pending_stop is not None:
                reason = "MOTION_STOP_PENDING"
            elif not self._startup_zero_confirmed:
                reason = self._execution_feedback_failure or "STARTUP_ZERO_UNCONFIRMED"
            elif recovery_fenced:
                reason = "RECOVERY_FENCED"
            else:
                reason = "OK"
            if operational and _source == "NAV":
                try:
                    self._check_execution_mode()
                except ActuatorError as exc:
                    operational = False
                    reason = exc.code + ": " + exc.message
            response, outcome = self._response(request)
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
            outcome.ready = operational
            outcome.actuator_configured = self._actuator_configured
            outcome.epoch_durable = self._epoch_durable
            outcome.reason = reason
            outcome.generation = self.epoch.generation
            outcome.boot_nonce = self.epoch.boot_nonce_bytes
            outcome.startup_zero_confirmed = self._startup_zero_confirmed
            outcome.motion_stop_generation = self._motion_stop_generation
            outcome.recovery_fence_held = recovery_fenced
            return response

    def _handle_get_guard_state(
        self,
        request: pb.MotionGuardRequest,
        _peer: PeerCredentials,
        _source: str,
    ) -> pb.MotionGuardResponse:
        with self._state_lock:
            response, outcome = self._response(request)
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
            outcome.generation = self.epoch.generation
            outcome.boot_nonce = self.epoch.boot_nonce_bytes
            outcome.last_zero_ack_id = self.epoch.last_zero_ack_id
            outcome.fenced = self.epoch.fenced
            outcome.active_lease_id = self._active_lease_id
            outcome.motion_stop_generation = self._motion_stop_generation
            if self._last_zero_confirmation is not None:
                outcome.last_zero_confirmation.CopyFrom(self._last_zero_confirmation)
            token = bytes.fromhex(self.epoch.recovery_fence_token_hex)
            outcome.recovery_fence_held = bool(token)
            if token:
                outcome.recovery_fence_token_sha256 = hashlib.sha256(token).digest()
                outcome.recovery_fence_correlation_sha256 = bytes.fromhex(
                    self.epoch.recovery_fence_correlation_sha256
                )
            return response

    def _handle_acquire_lease(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        with self._state_lock:
            if self.epoch.recovery_fence_token_hex:
                raise GuardFailure(
                    "RECOVERY_FENCED",
                    "motion recovery fence is held",
                )
        cached = self._prepare_mutation(
            request,
            peer,
            source,
            namespace="acquire",
        )
        if cached is not None:
            record = self._find_record(request, peer, source, "acquire")
            if record is not None and record.lease_id != self._active_lease_id:
                return self._error_response(
                    request,
                    "OUTCOME_UNKNOWN",
                    "acquire operation belongs to a closed lease",
                )
            return cached
        envelope = request.acquire_lease.envelope
        with self._state_lock:
            if self.epoch.recovery_fence_token_hex:
                raise GuardFailure(
                    "RECOVERY_FENCED",
                    "motion recovery fence is held",
                )
            if not self._actuator_configured or not self._epoch_durable:
                raise GuardFailure("MOTION_SINK_NOT_READY", self._actuator_reason)
            if not self._startup_zero_confirmed:
                raise GuardFailure(
                    "MOTION_SINK_NOT_READY",
                    "startup zero has not been confirmed",
                    True,
                )
            if envelope.motion_stop_generation != self._motion_stop_generation:
                raise GuardFailure(
                    "STALE_STOP_GENERATION",
                    "motion stop generation is stale",
                )
            if self.lease_state != pb.LEASE_STATE_IDLE:
                raise GuardFailure(
                    "RESOURCE_BUSY",
                    "motion resource already has an owner",
                    True,
                )
            if envelope.sequence != 0:
                raise GuardFailure("INVALID_ARGUMENT", "AcquireLease sequence must be zero")
            if source == "NAV":
                try:
                    self._check_execution_mode()
                except ActuatorError as exc:
                    raise GuardFailure(exc.code, exc.message, True) from exc
            now = self._monotonic_ns()
            lease_id = int(self._lease_id_factory())
            if lease_id <= 0 or lease_id > MAX_GENERATION:
                raise GuardFailure("INTERNAL", "lease_id factory returned an invalid u64")
            self._stop_confirmation.note_motion()
            self._active_lease_id = lease_id
            self._active_source = source
            self._last_sequence = 0
            self._lease_deadline_ns = now + LEASE_RENEWAL_WINDOW_NS
            self._owner_deadline_ns = min(now + OWNER_TIMEOUT_NS, self._lease_deadline_ns)
            self._watchdog_deadline_ns = max(
                now,
                self._owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS,
            )
            self._commit_epoch(
                lease=pb.LEASE_STATE_ACTIVE,
                fenced=False,
            )
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.lease_tuple.CopyFrom(self._current_tuple())
        outcome.lease_deadline_monotonic_ns = self._lease_deadline_ns
        outcome.owner_deadline_monotonic_ns = self._owner_deadline_ns
        self._remember(
            request,
            peer,
            source,
            response,
            namespace="acquire",
            acquire=True,
        )
        return response

    def _handle_renew_lease(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        cached = self._prepare_mutation(request, peer, source, namespace="lease")
        if cached is not None:
            return cached
        envelope = request.renew_lease.envelope
        with self._state_lock:
            now = self._validate_active(envelope, peer, source)
            self._accept_sequence(envelope.sequence)
            if source == "NAV":
                try:
                    self._check_execution_mode()
                except ActuatorError as exc:
                    self._zero()
                    self._commit_epoch(startup_zero_confirmed=True)
                    raise GuardFailure(exc.code, exc.message) from exc
            # 持续任务按有效续租接续，不能在首次取得租约十分钟后强制结束。
            # 身份、序列、存活和执行模式已核对；命令和重复操作不会延长此窗口。
            self._lease_deadline_ns = now + LEASE_RENEWAL_WINDOW_NS
            self._owner_deadline_ns = min(now + OWNER_TIMEOUT_NS, self._lease_deadline_ns)
            self._watchdog_deadline_ns = max(
                now,
                self._owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS,
            )
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.lease_deadline_monotonic_ns = self._lease_deadline_ns
        outcome.owner_deadline_monotonic_ns = self._owner_deadline_ns
        self._remember(request, peer, source, response)
        return response

    def _handle_submit_motion_command(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        cached = self._prepare_mutation(request, peer, source, namespace="lease")
        if cached is not None:
            return cached
        body = request.submit_motion_command
        command = body.WhichOneof("command")
        expected_action = {
            "twist": pb.GUARD_ACTION_TWIST,
            "stop": pb.GUARD_ACTION_STOP,
            "body_pose": pb.GUARD_ACTION_BODY_POSE,
            "speed_level": pb.GUARD_ACTION_SPEED_LEVEL,
            "obstacle_policy": pb.GUARD_ACTION_OBSTACLE_POLICY,
        }.get(command)
        if expected_action is None or body.envelope.action != expected_action:
            raise GuardFailure("INVALID_ARGUMENT", "command oneof/action mismatch")
        self._authorize_action(source, body.envelope.action)
        self._begin_lease_operation(body.envelope, peer, source)
        try:
            if command == "stop":
                zero_ack = self._zero()
            else:
                if source == "NAV":
                    try:
                        self._check_execution_mode()
                    except ActuatorError:
                        self._zero()
                        self._commit_epoch(startup_zero_confirmed=True)
                        raise
                if self._pending_stop is not None:
                    raise ActuatorError("MOTION_STOP_PENDING", "等待实际停稳，不能提交新的运动。")
                self._stop_confirmation.note_motion()
                self._actuator.apply_motion(body)
        except ActuatorError as exc:
            if command == "stop":
                self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        self._ensure_lease_still_active(body.envelope, peer, source)
        response, outcome = self._response(request)
        if command == "stop":
            applied_ns = zero_ack.applied_monotonic_ns
            self._commit_epoch(last_zero_ack_id=zero_ack.zero_ack_id)
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_ZEROED))
            outcome.zero_ack.CopyFrom(zero_ack)
        else:
            applied_ns = self._monotonic_ns()
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_APPLIED))
        outcome.applied_monotonic_ns = applied_ns
        self._remember(request, peer, source, response)
        return response

    def _handle_execute_sport_command(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        cached = self._prepare_mutation(request, peer, source, namespace="lease")
        if cached is not None:
            return cached
        body = request.execute_sport_command
        if body.envelope.action != pb.GUARD_ACTION_SPORT_ACTION:
            raise GuardFailure("INVALID_ARGUMENT", "sport command/action mismatch")
        self._authorize_action(source, body.envelope.action)
        self._begin_lease_operation(body.envelope, peer, source)
        try:
            if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
                zero_ack = self._zero()
            else:
                if self._pending_stop is not None:
                    raise ActuatorError("MOTION_STOP_PENDING", "等待实际停稳，不能提交新的机体动作。")
                self._stop_confirmation.note_motion()
                self._actuator.execute_sport(body.command)
        except ActuatorError as exc:
            if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
                self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        self._ensure_lease_still_active(body.envelope, peer, source)
        response, outcome = self._response(request)
        if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
            applied_ns = zero_ack.applied_monotonic_ns
            self._commit_epoch(last_zero_ack_id=zero_ack.zero_ack_id)
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_ZEROED))
            outcome.zero_ack.CopyFrom(zero_ack)
        else:
            applied_ns = self._monotonic_ns()
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_APPLIED))
        outcome.applied_monotonic_ns = applied_ns
        self._remember(request, peer, source, response)
        return response

    def _handle_revoke_and_zero(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        cached = self._prepare_mutation(
            request,
            peer,
            source,
            namespace="lease",
            priority=True,
        )
        if cached is not None:
            return cached
        envelope = request.revoke_and_zero.envelope
        retired = self._externally_fenced_leases.get((envelope.tuple.lease_id, source))
        if retired is not None:
            response, outcome = self._response(request)
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
            outcome.result.lease = pb.LEASE_STATE_IDLE
            outcome.zero_ack.ParseFromString(retired)
            # 这里只交付旧租约的历史停车证明，不能归零或撤销后来者的运动。
            self._remember(request, peer, source, response, priority=True)
            return response
        self._begin_lease_operation(envelope, peer, source)
        return self._revoke_response(request, peer, source)

    def _handle_revoke_zero_and_switch_mode(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        cached = self._prepare_mutation(
            request,
            peer,
            source,
            namespace="lease",
            priority=True,
        )
        if cached is not None:
            return cached
        body = request.revoke_zero_and_switch_mode
        if body.envelope.action != pb.GUARD_ACTION_MODE_SWITCH:
            raise GuardFailure("INVALID_ARGUMENT", "mode switch action mismatch")
        self._authorize_action(source, body.envelope.action)
        self._begin_lease_operation(body.envelope, peer, source)
        self.lease_state = pb.LEASE_STATE_PAUSING
        try:
            zero_ack = self._zero(wait=True)
            self._stop_confirmation.note_motion()
            active_mode = self._actuator.switch_mode(body.command)
            self._clear_lease()
            self._commit_epoch(
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                startup_zero_confirmed=True,
            )
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
        outcome.zero_ack.CopyFrom(zero_ack)
        outcome.active_mode = active_mode
        self._remember(request, peer, source, response, priority=True)
        return response

    def _handle_stop_all_motion(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        if source != "GATEWAY_MANUAL":
            raise GuardFailure(
                "PERMISSION_DENIED",
                "StopAllMotion requires the trusted gateway manual source",
            )
        body = request.stop_all_motion
        cached = self._prepare_global_stop(
            request,
            peer,
            source,
        )
        if cached is not None:
            return cached
        if not body.reason.strip():
            raise GuardFailure("INVALID_ARGUMENT", "stop reason is required")
        if body.mode not in {
            pb.GLOBAL_STOP_MODE_ORDINARY,
            pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE,
            pb.GLOBAL_STOP_MODE_RELEASE_RECOVERY_FENCE,
        }:
            raise GuardFailure("INVALID_ARGUMENT", "unknown global stop mode")
        correlation = body.recovery_correlation.strip()
        if body.mode != pb.GLOBAL_STOP_MODE_ORDINARY and not correlation:
            raise GuardFailure(
                "INVALID_ARGUMENT",
                "recovery correlation is required for recovery fence operations",
            )
        correlation_sha256 = hashlib.sha256(correlation.encode("utf-8")).hexdigest()
        with self._state_lock:
            held_token_hex = self.epoch.recovery_fence_token_hex
            held_correlation_sha256 = (
                self.epoch.recovery_fence_correlation_sha256
            )
            if body.mode == pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE:
                if held_token_hex and not hmac.compare_digest(
                    held_correlation_sha256, correlation_sha256
                ):
                    raise GuardFailure(
                        "RESOURCE_BUSY",
                        "a recovery fence is already held by another correlation",
                    )
            elif body.mode == pb.GLOBAL_STOP_MODE_RELEASE_RECOVERY_FENCE:
                supplied_token = bytes(body.recovery_fence_token)
                if not held_token_hex:
                    raise GuardFailure(
                        "OUTCOME_UNKNOWN",
                        "no recovery fence is currently held",
                    )
                if len(supplied_token) != 32 or not hmac.compare_digest(
                    supplied_token, bytes.fromhex(held_token_hex)
                ) or not hmac.compare_digest(
                    correlation_sha256, held_correlation_sha256
                ):
                    raise GuardFailure(
                        "PERMISSION_DENIED",
                        "recovery fence token or correlation does not match",
                    )
            if self._motion_stop_generation == MAX_GENERATION:
                raise GuardFailure(
                    "RESOURCE_EXHAUSTED",
                    "motion stop generation overflow",
                )
            self._motion_stop_generation += 1
            lease_revoked = self.lease_state == pb.LEASE_STATE_ACTIVE
            self.lease_state = pb.LEASE_STATE_REVOKING
            try:
                zero_ack = self._zero()
                self._clear_lease()
                recovery_updates: Dict[str, object] = {}
                if body.mode == pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE:
                    recovery_updates = {
                        "recovery_fence_token_hex": (
                            held_token_hex or secrets.token_hex(32)
                        ),
                        "recovery_fence_correlation_sha256": correlation_sha256,
                    }
                elif body.mode in {
                    pb.GLOBAL_STOP_MODE_ORDINARY,
                    pb.GLOBAL_STOP_MODE_RELEASE_RECOVERY_FENCE,
                }:
                    # 新的显式全局停车以新停止代次撤销全部旧准入；取得归零
                    # 回执后也结束旧业务恢复围栏，避免原拥有方退出后永久锁死。
                    # 归零或持久化失败不会提交解除，停稳等待仍由原确认器负责。
                    recovery_updates = {
                        "recovery_fence_token_hex": "",
                        "recovery_fence_correlation_sha256": "",
                    }
                self._commit_epoch(
                    lease=pb.LEASE_STATE_IDLE,
                    fenced=True,
                    last_zero_ack_id=zero_ack.zero_ack_id,
                    startup_zero_confirmed=True,
                    **recovery_updates,
                )
            except ActuatorError as exc:
                self._mark_zero_unconfirmed(exc)
                raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(
            self._result(
                pb.ACK_CODE_RESOURCE_FENCED
                if lease_revoked
                else pb.ACK_CODE_ZEROED
            )
        )
        outcome.zero_ack.CopyFrom(zero_ack)
        outcome.lease_revoked = lease_revoked
        outcome.motion_stop_generation = self._motion_stop_generation
        outcome.recovery_fence_held = bool(self.epoch.recovery_fence_token_hex)
        if body.mode == pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE:
            outcome.recovery_fence_token = bytes.fromhex(
                self.epoch.recovery_fence_token_hex
            )
        self._remember(
            request,
            peer,
            source,
            response,
            namespace="global_stop",
            priority=True,
        )
        return response

    def _handle_get_operation_result(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        operation_id = request.get_operation_result.operation_id
        namespace = _QUERY_NAMESPACES.get(
            request.get_operation_result.operation_namespace
        )
        if namespace is None:
            raise GuardFailure(
                "INVALID_ARGUMENT",
                "operation_namespace must identify acquire, lease, or global stop",
            )
        response, outcome = self._response(request)
        outcome.target_operation_id = operation_id
        records = self._operation_index.get(operation_id, ())
        record = next(
            (
                candidate
                for candidate in records
                if candidate.source == source and candidate.namespace == namespace
            ),
            None,
        )
        if record is None:
            outcome.result.CopyFrom(
                self._result(
                    pb.ACK_CODE_UNSPECIFIED,
                    code="OUTCOME_UNKNOWN",
                    message="operation result is not available in this boot epoch",
                    retryable=False,
                )
            )
            outcome.resolved = False
            return response
        outcome.result.CopyFrom(self._result(record.ack))
        outcome.target_ack = record.ack
        outcome.resolved = True
        if record.zero_ack_id:
            if record.stationary_zero_ack_bytes:
                outcome.zero_ack.ParseFromString(record.stationary_zero_ack_bytes)
            else:
                saved = pb.MotionGuardResponse.FromString(record.response_bytes)
                outcome.zero_ack.CopyFrom(getattr(saved, saved.WhichOneof("outcome")).zero_ack)
        outcome.motion_stop_generation = record.motion_stop_generation
        outcome.recovery_fence_token = record.recovery_fence_token
        outcome.recovery_fence_held = record.recovery_fence_held
        return response

    def _prepare_mutation(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        *,
        namespace: str,
        priority: bool = False,
    ) -> Optional[pb.MotionGuardResponse]:
        verify_payload_sha256(request)
        envelope = operation_envelope(request)
        if envelope is None:
            raise ProtocolViolation("mutation request has no envelope")
        if not envelope.operation_id:
            raise GuardFailure("INVALID_ARGUMENT", "operation_id is required")
        self._validate_epoch_tuple(envelope, namespace=namespace)
        key = self._dedup_key(envelope, peer, source, namespace)
        record = self._ordinary_records.get(key) or self._priority_records.get(key)
        if record is not None:
            if record.payload_sha256 != bytes(envelope.payload_sha256):
                return self._error_response(
                    request,
                    "REQUEST_ID_CONFLICT",
                    "operation_id was reused with a different typed payload",
                )
            response = pb.MotionGuardResponse()
            response.ParseFromString(record.response_bytes)
            response.request_id = request.request_id
            return response
        self._validate_deadline(envelope)
        target = self._priority_records if priority else self._ordinary_records
        capacity = self._priority_capacity if priority else self._ordinary_capacity
        if len(target) >= capacity:
            self._cache_exhausted()
            raise GuardFailure("RESOURCE_EXHAUSTED", "dedup cache is exhausted")
        return None

    def _prepare_global_stop(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> Optional[pb.MotionGuardResponse]:
        verify_payload_sha256(request)
        envelope = request.stop_all_motion.envelope
        if not envelope.operation_id:
            raise GuardFailure("INVALID_ARGUMENT", "operation_id is required")
        key = self._dedup_key(envelope, peer, source, "global_stop")
        record = self._ordinary_records.get(key) or self._priority_records.get(key)
        if record is not None:
            if record.payload_sha256 != bytes(envelope.payload_sha256):
                return self._error_response(
                    request,
                    "REQUEST_ID_CONFLICT",
                    "operation_id was reused with a different typed payload",
                )
            response = pb.MotionGuardResponse()
            response.ParseFromString(record.response_bytes)
            response.request_id = request.request_id
            return response
        self._validate_deadline(envelope)
        if len(self._priority_records) >= self._priority_capacity:
            self._cache_exhausted()
            raise GuardFailure("RESOURCE_EXHAUSTED", "priority dedup cache is exhausted")
        return None

    def _validate_epoch_tuple(self, envelope: pb.RequestEnvelope, *, namespace: str) -> None:
        token = envelope.tuple
        if token.generation != self.epoch.generation or bytes(token.boot_nonce) != self.epoch.boot_nonce_bytes:
            raise GuardFailure("STALE_EPOCH", "generation or boot nonce is stale")
        if namespace == "acquire" and token.lease_id != 0:
            raise GuardFailure("INVALID_ARGUMENT", f"{namespace} operation requires lease_id=0")
        if namespace == "lease" and token.lease_id == 0:
            raise GuardFailure("OUTCOME_UNKNOWN", "lease-bound operation has no lease_id")

    def _validate_deadline(self, envelope: pb.RequestEnvelope) -> int:
        now = self._monotonic_ns()
        issued = envelope.issued_monotonic_ns
        deadline = envelope.deadline_monotonic_ns
        if issued == 0 or deadline <= issued:
            raise GuardFailure("INVALID_ARGUMENT", "invalid monotonic deadline interval")
        if deadline - issued > MAX_COMMAND_DEADLINE_NS:
            raise GuardFailure("INVALID_ARGUMENT", "deadline exceeds 300ms bound")
        if issued > now:
            raise GuardFailure("INVALID_ARGUMENT", "issued_monotonic_ns is in the future")
        if now > deadline:
            raise GuardFailure("DEADLINE_EXCEEDED", "request monotonic deadline expired", True)
        return now

    def _validate_active(
        self,
        envelope: pb.RequestEnvelope,
        peer: PeerCredentials,
        source: str,
    ) -> int:
        now = self._monotonic_ns()
        token = envelope.tuple
        if (
            self.lease_state != pb.LEASE_STATE_ACTIVE
            or token.lease_id != self._active_lease_id
        ):
            raise GuardFailure("OUTCOME_UNKNOWN", "lease is inactive or already closed")
        if source != self._active_source:
            raise GuardFailure("PERMISSION_DENIED", "source does not own the lease")
        if now >= self._owner_deadline_ns or now >= self._lease_deadline_ns:
            self.tick(now)
            raise GuardFailure("OUTCOME_UNKNOWN", "lease liveness deadline expired")
        return now

    def _begin_lease_operation(
        self,
        envelope: pb.RequestEnvelope,
        peer: PeerCredentials,
        source: str,
    ) -> int:
        with self._state_lock:
            now = self._validate_active(envelope, peer, source)
            self._accept_sequence(envelope.sequence)
            return now

    def _ensure_lease_still_active(
        self,
        envelope: pb.RequestEnvelope,
        peer: PeerCredentials,
        source: str,
    ) -> None:
        with self._state_lock:
            if (
                self.lease_state != pb.LEASE_STATE_ACTIVE
                or envelope.tuple.lease_id != self._active_lease_id
                or source != self._active_source
            ):
                raise GuardFailure(
                    "OUTCOME_UNKNOWN",
                    "actuator completion arrived after the lease was fenced",
                )

    def _accept_sequence(self, sequence: int) -> None:
        if sequence <= self._last_sequence:
            raise GuardFailure("STALE_SEQUENCE", "sequence is not strictly increasing")
        self._last_sequence = sequence

    @staticmethod
    def _authorize_action(source: str, action: int) -> None:
        if action not in _SOURCE_ACTION_POLICY.get(source, frozenset()):
            action_name = pb.GuardAction.Name(action)
            raise GuardFailure(
                "ACTION_NOT_ALLOWED",
                f"source {source} is not authorized for {action_name}",
            )

    def _revoke_response(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                startup_zero_confirmed=True,
            )
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
        outcome.zero_ack.CopyFrom(zero_ack)
        self._remember(request, peer, source, response, priority=True)
        return response

    def _mark_zero_unconfirmed(
        self,
        error: ActuatorError,
        *,
        now_ns: Optional[int] = None,
    ) -> None:
        if error.code.startswith(("MOTION_", "LOCOMOTION_")):
            self._execution_feedback_failure = error.code + ": " + error.message
        self.lease_state = pb.LEASE_STATE_REVOKING
        self._startup_zero_confirmed = False
        try:
            self._commit_epoch(
                lease=pb.LEASE_STATE_REVOKING,
                fenced=True,
                startup_zero_confirmed=False,
            )
        except EpochError:
            pass
        self._schedule_zero_retry(
            self._monotonic_ns() if now_ns is None else now_ns
        )

    def _retry_zero_fence(self, now_ns: int) -> Optional[pb.ZeroAck]:
        try:
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                startup_zero_confirmed=True,
            )
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc, now_ns=now_ns)
            return None
        except EpochError:
            self.lease_state = pb.LEASE_STATE_REVOKING
            self._startup_zero_confirmed = False
            self._schedule_zero_retry(now_ns)
            return None
        self._reset_zero_retry()
        return zero_ack

    def _schedule_zero_retry(self, now_ns: int) -> None:
        self._zero_retry_not_before_ns = now_ns + self._zero_retry_delay_ns
        self._zero_retry_delay_ns = min(
            self._zero_retry_delay_ns * 2,
            ZERO_RETRY_MAX_NS,
        )

    def _reset_zero_retry(self) -> None:
        self._zero_retry_not_before_ns = 0
        self._zero_retry_delay_ns = ZERO_RETRY_INITIAL_NS

    def _read_execution_feedback(self):
        return self._stop_confirmation.feedback(self._actuator)

    def _check_execution_mode(self) -> None:
        requirement = self._stop_confirmation.locomotion
        if requirement is not None:
            requirement.check(self._read_execution_feedback)

    @property
    def stop_confirmation_timeout_seconds(self) -> float:
        return self._stop_confirmation.maximum_wait_seconds

    def _zero(self, *, wait: bool = False) -> pb.ZeroAck:
        self._actuator.zero()
        started = self._stop_confirmation.begin()
        if self._pending_stop is None:
            retained_zero_id = (self._last_zero_confirmation.zero_ack_id
                                if self._pending_zero_records and self._last_zero_confirmation is not None else None)
            progress = self._stop_confirmation.start(self._actuator, started)
            if isinstance(progress, StopEvidence):
                if progress.stationary_confirmed and self._last_zero_confirmation is not None:
                    return pb.ZeroAck.FromString(self._last_zero_confirmation.SerializeToString())
                self._last_zero_confirmation = self._new_zero_ack(self._monotonic_ns(), progress)
            else:
                self._pending_stop = progress
                self._startup_zero_confirmed = False
                self._last_zero_confirmation = self._new_zero_ack(
                    self._monotonic_ns(), StopEvidence(False, "awaiting_feedback" if progress.has_feedback else "awaiting_braking_bound"))
                if retained_zero_id:
                    # 停稳超时后的自动重试仍属于同一次未完成停车，保留原操作关联。
                    self._last_zero_confirmation.zero_ack_id = retained_zero_id
                self._last_zero_confirmation.stationary_deadline_monotonic_ns = self._monotonic_ns() + int(self._stop_confirmation.remaining_seconds(progress) * 1_000_000_000)
        if wait and self._pending_stop is not None:
            try:
                evidence = self._stop_confirmation.wait(self._actuator, self._pending_stop)
                self._finish_stop_confirmation(evidence)
            except ActuatorError:
                self._pending_stop = None
                self._last_zero_confirmation.confirmation_kind = "unconfirmed"
                self._last_zero_confirmation.stationary_deadline_monotonic_ns = 0
                self._store_stop_confirmation()
                raise
        return pb.ZeroAck.FromString(self._last_zero_confirmation.SerializeToString())

    def _finish_stop_confirmation(self, evidence: StopEvidence) -> pb.ZeroAck:
        zero = self._new_zero_ack(self._monotonic_ns(), evidence)
        zero.zero_ack_id = self._last_zero_confirmation.zero_ack_id
        self._last_zero_confirmation = zero
        self._execution_feedback_failure = ""
        self._pending_stop = None
        self._store_stop_confirmation()
        return zero

    def _store_stop_confirmation(self) -> None:
        # 补充原操作账本的物理终态，不改写最初命令回执的幂等响应字节。
        encoded = self._last_zero_confirmation.SerializeToString(deterministic=True)
        for key, saved in self._externally_fenced_leases.items():
            if pb.ZeroAck.FromString(saved).zero_ack_id == self._last_zero_confirmation.zero_ack_id:
                self._externally_fenced_leases[key] = encoded
        for priority, key, operation_id in self._pending_zero_records:
            target = self._priority_records if priority else self._ordinary_records
            record = target.get(key)
            if record is None or record.zero_ack_id != self._last_zero_confirmation.zero_ack_id:
                continue
            updated = replace(record, stationary_zero_ack_bytes=encoded)
            target[key] = updated
            self._operation_index[operation_id] = [updated if item is record else item for item in self._operation_index.get(operation_id, ())]
        if self._last_zero_confirmation.stationary_confirmed:
            self._pending_zero_records = []

    @staticmethod
    def _new_zero_ack(now_ns: int, evidence: Optional[StopEvidence] = None) -> pb.ZeroAck:
        evidence = evidence or StopEvidence(False, "actuator_command_ack")
        return pb.ZeroAck(
            zero_ack_id=str(uuid.uuid4()),
            applied_monotonic_ns=now_ns,
            stationary_confirmed=evidence.stationary_confirmed,
            confirmation_kind=evidence.confirmation_kind,
            feedback_source_timestamp_ns=evidence.feedback_source_timestamp_ns,
        )

    def _cache_exhausted(self) -> None:
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                startup_zero_confirmed=True,
            )
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
        except EpochError:
            self._startup_zero_confirmed = False

    def _commit_epoch(
        self,
        *,
        lease: Optional[int] = None,
        fenced: Optional[bool] = None,
        last_zero_ack_id: Optional[str] = None,
        startup_zero_confirmed: Optional[bool] = None,
        recovery_fence_token_hex: Optional[str] = None,
        recovery_fence_correlation_sha256: Optional[str] = None,
    ) -> None:
        next_lease = self.lease_state if lease is None else lease
        updates: Dict[str, object] = {
            "lease_state": _LEASE_TO_EPOCH[next_lease],
        }
        if fenced is not None:
            updates["fenced"] = fenced
        if last_zero_ack_id is not None:
            updates["last_zero_ack_id"] = last_zero_ack_id
        if startup_zero_confirmed is not None:
            updates["startup_zero_confirmed"] = startup_zero_confirmed and self._pending_stop is None
        if recovery_fence_token_hex is not None:
            updates["recovery_fence_token_hex"] = recovery_fence_token_hex
        if recovery_fence_correlation_sha256 is not None:
            updates["recovery_fence_correlation_sha256"] = (
                recovery_fence_correlation_sha256
            )
        next_epoch = self.epoch.with_updates(**updates)
        try:
            self._epoch_store.persist(next_epoch)
        except (OSError, EpochError) as exc:
            self._epoch_durable = False
            self._startup_zero_confirmed = False
            raise EpochError(f"epoch durable commit failed: {exc}") from exc
        self._epoch_durable = True
        self.epoch = next_epoch
        self.lease_state = next_lease
        self._startup_zero_confirmed = next_epoch.startup_zero_confirmed
        # 到期、主动撤租、全局停车和失败重试共用同一持久提交边界。
        # 只保存已经实际取得且与本次持久回执匹配的停车证明。
        if (self._pending_retired_lease is not None and next_lease == pb.LEASE_STATE_IDLE
                and last_zero_ack_id and last_zero_ack_id == self._last_zero_confirmation.zero_ack_id
                and self._last_zero_confirmation.applied_monotonic_ns > 0):
            self._externally_fenced_leases[self._pending_retired_lease] = self._last_zero_confirmation.SerializeToString()
            self._pending_retired_lease = None
            while len(self._externally_fenced_leases) > self._priority_capacity:
                del self._externally_fenced_leases[next(iter(self._externally_fenced_leases))]

    def _clear_lease(self) -> None:
        if self._active_lease_id:
            self._pending_retired_lease = (self._active_lease_id, self._active_source)
        self._active_lease_id = 0
        self._active_source = ""
        self._last_sequence = 0
        self._lease_deadline_ns = 0
        self._owner_deadline_ns = 0
        self._watchdog_deadline_ns = 0
        self.lease_state = pb.LEASE_STATE_IDLE

    def _current_tuple(self) -> pb.LeaseTuple:
        return pb.LeaseTuple(
            generation=self.epoch.generation,
            boot_nonce=self.epoch.boot_nonce_bytes,
            lease_id=self._active_lease_id,
        )

    def _dedup_key(
        self,
        envelope,
        peer: PeerCredentials,
        source: str,
        namespace: str,
    ) -> Tuple[object, ...]:
        if namespace == "global_stop":
            return (
                namespace,
                source,
                envelope.operation_id,
            )
        if namespace == "acquire":
            return (
                namespace,
                bytes(envelope.tuple.boot_nonce),
                source,
                envelope.operation_id,
            )
        return (
            namespace,
            source,
            envelope.tuple.generation,
            bytes(envelope.tuple.boot_nonce),
            envelope.tuple.lease_id,
            envelope.operation_id,
        )

    def _find_record(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        namespace: str,
    ) -> Optional[OperationRecord]:
        envelope = operation_envelope(request)
        if envelope is None:
            return None
        key = self._dedup_key(envelope, peer, source, namespace)
        return self._ordinary_records.get(key) or self._priority_records.get(key)

    def _remember(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        response: pb.MotionGuardResponse,
        *,
        namespace: str = "lease",
        priority: bool = False,
        acquire: bool = False,
    ) -> None:
        envelope = operation_envelope(request)
        if envelope is None:
            return
        key = self._dedup_key(envelope, peer, source, namespace)
        outcome_name = response.WhichOneof("outcome")
        outcome = getattr(response, outcome_name)
        result = outcome.result
        zero_ack = getattr(outcome, "zero_ack", None)
        record = OperationRecord(
            key=key,
            operation_id=envelope.operation_id,
            payload_sha256=bytes(envelope.payload_sha256),
            source=source,
            lease_id=(
                outcome.lease_tuple.lease_id
                if acquire
                else getattr(getattr(envelope, "tuple", None), "lease_id", 0)
            ),
            response_bytes=response.SerializeToString(deterministic=True),
            ack=result.ack,
            zero_ack_id=(zero_ack.zero_ack_id if zero_ack is not None else ""),
            zero_applied_ns=(
                zero_ack.applied_monotonic_ns if zero_ack is not None else 0
            ),
            motion_stop_generation=getattr(outcome, "motion_stop_generation", 0),
            namespace=namespace,
            recovery_fence_token=bytes(
                getattr(outcome, "recovery_fence_token", b"")
            ),
            recovery_fence_held=bool(
                getattr(outcome, "recovery_fence_held", False)
            ),
            acquire=acquire,
        )
        target = self._priority_records if priority else self._ordinary_records
        target[key] = record
        self._operation_index.setdefault(envelope.operation_id, []).append(record)
        if self._pending_stop is not None and record.zero_ack_id == self._last_zero_confirmation.zero_ack_id:
            self._pending_zero_records.append((priority, key, envelope.operation_id))

    def _response(self, request: pb.MotionGuardRequest):
        operation = request.WhichOneof("operation")
        if operation is None:
            raise ProtocolViolation("request operation is missing")
        response = pb.MotionGuardResponse(
            api_version=API_VERSION,
            request_id=request.request_id,
        )
        outcome = getattr(response, operation)
        outcome.SetInParent()
        return response, outcome

    def _error_response(
        self,
        request: pb.MotionGuardRequest,
        code: str,
        message: str,
        *,
        retryable: bool = False,
    ) -> pb.MotionGuardResponse:
        response, outcome = self._response(request)
        outcome.result.CopyFrom(
            self._result(
                pb.ACK_CODE_UNSPECIFIED,
                code=code,
                message=message,
                retryable=retryable,
            )
        )
        return response

    def _result(
        self,
        ack: int,
        *,
        code: str = "",
        message: str = "",
        retryable: bool = False,
    ) -> pb.GuardResult:
        result = pb.GuardResult(ack=ack, lease=self.lease_state)
        if code:
            result.error.code = code
            result.error.message = message
            result.error.retryable = retryable
        return result
