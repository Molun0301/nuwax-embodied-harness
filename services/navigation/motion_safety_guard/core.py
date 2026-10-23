from __future__ import annotations

from dataclasses import dataclass, replace
import logging
import math
import secrets
import time
from typing import Callable, Dict, List, Optional, Tuple
import uuid

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import (
    API_VERSION,
    MAX_UINT64,
    OPERATION_RECORD_CAPACITY,
    RETRY_HORIZON_NS,
    MAX_COMMAND_DEADLINE_NS,
    OWNER_TIMEOUT_NS,
    WATCHDOG_FENCE_LEAD_NS,
    ZERO_RETRY_INITIAL_NS,
    ZERO_RETRY_MAX_NS,
)
from .go2_actuator import ActuatorBackend, ActuatorError
from .peer_auth import PeerCredentials
from .stop_confirmation import StopConfirmation, StopEvidence
from .wire import (
    WireContractError,
    operation_envelope,
    request_source_name,
    canonical_payload,
)


_LOGGER = logging.getLogger(__name__)


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
    operation: str
    payload: bytes
    source: str
    lease_id: int
    response_bytes: bytes
    ack: int
    zero_ack_id: str
    zero_applied_ns: int
    motion_stop_generation: int
    namespace: str
    recorded_monotonic_ns: int
    acquire: bool = False
    stationary_zero_ack_bytes: bytes = b""


_QUERY_NAMESPACES = {
    pb.OPERATION_NAMESPACE_ACQUIRE: "acquire",
    pb.OPERATION_NAMESPACE_LEASE: "lease",
    pb.OPERATION_NAMESPACE_GLOBAL_STOP: "global_stop",
}


class MotionGuardCore:
    """Single-threaded arbitration state for the private motion-guard RPCs."""

    def __init__(self, *, actuator: ActuatorBackend,
                 monotonic_ns: Callable[[], int] = time.monotonic_ns,
                 lease_id_factory: Optional[Callable[[], int]] = None,
                 stop_confirmation: Optional[StopConfirmation] = None) -> None:
        self._boot_nonce = secrets.token_bytes(16)
        self._actuator = actuator
        self._stop_confirmation = stop_confirmation or StopConfirmation.from_environment()
        self._pending_stop = None
        self._last_zero_confirmation = None
        self._pending_zero_records = []
        self._execution_feedback_failure = ""
        self._monotonic_ns = monotonic_ns
        self._lease_id_factory = lease_id_factory or (lambda: secrets.randbits(64) or 1)
        self._actuator_configured = bool(actuator.is_configured)
        self._actuator_reason = "OK" if self._actuator_configured else "ACTUATOR_UNCONFIGURED"
        self.lease_state = pb.LEASE_STATE_IDLE
        self._startup_zero_confirmed = False
        self._active_lease_id = 0
        self._active_source = ""
        self._owner_deadline_ns = 0
        self._watchdog_deadline_ns = 0
        self._motion_stop_generation = 0
        self._zero_retry_not_before_ns = 0
        self._zero_retry_delay_ns = ZERO_RETRY_INITIAL_NS
        self._ordinary_records: Dict[Tuple[object, ...], OperationRecord] = {}
        self._priority_records: Dict[Tuple[object, ...], OperationRecord] = {}
        self._operation_index: Dict[str, List[OperationRecord]] = {}

    @property
    def boot_nonce(self) -> bytes:
        return self._boot_nonce

    @property
    def active_lease_id(self) -> int:
        return self._active_lease_id

    @property
    def motion_stop_generation(self) -> int:
        return self._motion_stop_generation

    @property
    def startup_configured(self) -> bool:
        return self._actuator_configured

    @property
    def startup_reason(self) -> str:
        return self._actuator_reason

    def handle(self, request: pb.MotionGuardRequest, peer: PeerCredentials) -> pb.MotionGuardResponse:
        operation = request.WhichOneof("operation")
        if operation is None:
            raise ProtocolViolation("MotionGuardRequest.operation is missing")
        if not request.request_id:
            return self._error_response(request, "INVALID_ARGUMENT", "request_id is required")
        try:
            if request.api_version != API_VERSION:
                raise GuardFailure("UNSUPPORTED_API_VERSION", f"api_version {request.api_version} is unsupported")
            source = request_source_name(request)
            if source == "UNSPECIFIED":
                raise GuardFailure("PERMISSION_DENIED", "claimed source is unspecified")
            return getattr(self, f"_handle_{operation}")(request, peer, source)
        except (WireContractError, ProtocolViolation) as exc:
            return self._error_response(request, "INVALID_ARGUMENT", str(exc))
        except GuardFailure as exc:
            return self._error_response(request, exc.code, exc.message, retryable=exc.retryable)

    def tick(self, now_ns: Optional[int] = None) -> Optional[pb.ZeroAck]:
        now = self._monotonic_ns() if now_ns is None else now_ns
        completed = None
        if self._pending_stop is not None:
            try:
                evidence = self._stop_confirmation.poll(self._actuator, self._pending_stop)
                if evidence is None:
                    return None
                completed = self._finish_stop_confirmation(evidence)
                self._startup_zero_confirmed = True
            except ActuatorError as exc:
                self._pending_stop = None
                self._last_zero_confirmation.confirmation_kind = "unconfirmed"
                self._last_zero_confirmation.stationary_deadline_monotonic_ns = 0
                self._store_stop_confirmation()
                self._mark_zero_unconfirmed(exc, now_ns=now)
                return None
        if self.lease_state == pb.LEASE_STATE_REVOKING and not self._startup_zero_confirmed:
            return None if now < self._zero_retry_not_before_ns else self._retry_zero_fence(now)
        if self.lease_state != pb.LEASE_STATE_ACTIVE:
            return completed
        if now < self._watchdog_deadline_ns:
            if self._active_source == "NAV":
                try:
                    self._check_execution_mode()
                except ActuatorError as exc:
                    self._execution_feedback_failure = exc.code + ": " + exc.message
                    try:
                        zero_ack = self._zero()
                        self._complete_lease_zero()
                        self._reset_zero_retry()
                    except ActuatorError as zero_error:
                        self._mark_zero_unconfirmed(zero_error, now_ns=now)
            return completed
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            zero_ack = self._zero()
            self._complete_lease_zero()
            self._reset_zero_retry()
            return zero_ack
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc, now_ns=now)
            return None

    def initialize_zero_fence(self, *, prepare_mode: bool = True, cancelled=lambda: False) -> pb.ZeroAck:
        """启动或关闭时归零全部实际路径，停稳后再开放租约。"""
        self.lease_state = pb.LEASE_STATE_REVOKING
        self._startup_zero_confirmed = False
        try:
            zero_ack = self._zero(wait=True)
            requirement = self._stop_confirmation.locomotion
            if requirement is not None and prepare_mode:
                try:
                    changed = requirement.prepare(self._actuator, self._read_execution_feedback, self._stop_confirmation.note_motion, cancelled)
                    if changed:
                        zero_ack = self._zero(wait=True)
                    requirement.check(self._read_execution_feedback)
                except ActuatorError as mode_error:
                    try:
                        self._zero(wait=True)
                    except ActuatorError as stop_error:
                        raise ActuatorError("MOTION_STOP_UNCONFIRMED", mode_error.message + "; 停止清理也失败：" + stop_error.message) from mode_error
                    raise
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
            raise
        self._complete_lease_zero()
        self._reset_zero_retry()
        return zero_ack

    def _handle_get_readiness(self, request, _peer, _source):
        ready = self._actuator_configured and self._startup_zero_confirmed and not self._execution_feedback_failure
        if not self._actuator_configured:
            reason = self._actuator_reason
        elif self._pending_stop is not None:
            reason = "MOTION_STOP_PENDING"
        elif self._execution_feedback_failure:
            reason = self._execution_feedback_failure
        elif not self._startup_zero_confirmed:
            reason = "STARTUP_ZERO_UNCONFIRMED"
        else:
            reason = "OK"
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.ready = bool(ready)
        outcome.actuator_configured = self._actuator_configured
        outcome.reason = reason
        outcome.boot_nonce = self._boot_nonce
        outcome.startup_zero_confirmed = self._startup_zero_confirmed
        outcome.motion_stop_generation = self._motion_stop_generation
        return response

    def _handle_get_guard_state(self, request, _peer, _source):
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.boot_nonce = self._boot_nonce
        outcome.active_lease_id = self._active_lease_id
        outcome.motion_stop_generation = self._motion_stop_generation
        if self._last_zero_confirmation is not None:
            outcome.last_zero_confirmation.CopyFrom(self._last_zero_confirmation)
        return response

    def _handle_acquire_lease(self, request, peer, source):
        cached = self._prepare_mutation(request, peer, source, namespace="acquire")
        if cached is not None:
            record = self._find_record(request, peer, source, "acquire")
            if record is not None and record.lease_id != self._active_lease_id:
                return self._error_response(request, "OUTCOME_UNKNOWN", "acquire operation belongs to a closed lease")
            return cached
        envelope = request.acquire_lease.envelope
        if not self._actuator_configured:
            raise GuardFailure("MOTION_SINK_NOT_READY", self._actuator_reason)
        if not self._startup_zero_confirmed:
            raise GuardFailure("MOTION_SINK_NOT_READY", "startup zero has not been confirmed", True)
        if envelope.motion_stop_generation != self._motion_stop_generation:
            raise GuardFailure("STALE_STOP_GENERATION", "motion stop generation is stale")
        if self.lease_state != pb.LEASE_STATE_IDLE:
            raise GuardFailure("RESOURCE_BUSY", "motion resource already has an owner", True)
        if source == "NAV":
            try:
                self._check_execution_mode()
            except ActuatorError as exc:
                self._execution_feedback_failure = exc.code + ": " + exc.message
                raise GuardFailure(exc.code, exc.message, True) from exc
        now = self._monotonic_ns()
        lease_id = int(self._lease_id_factory())
        if lease_id <= 0 or lease_id > MAX_UINT64:
            raise GuardFailure("INTERNAL", "lease_id factory returned an invalid u64")
        self._stop_confirmation.note_motion()
        self._active_lease_id = lease_id
        self._active_source = source
        self._owner_deadline_ns = now + OWNER_TIMEOUT_NS
        self._watchdog_deadline_ns = max(now, self._owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS)
        self.lease_state = pb.LEASE_STATE_ACTIVE
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.lease_tuple.CopyFrom(self._current_tuple())
        outcome.owner_deadline_monotonic_ns = self._owner_deadline_ns
        self._remember(request, peer, source, response, namespace="acquire", acquire=True)
        return response

    def _handle_renew_lease(self, request, peer, source):
        cached = self._prepare_mutation(request, peer, source, namespace="lease")
        if cached is not None:
            return cached
        now = self._validate_active(request.renew_lease.envelope, peer, source)
        self._owner_deadline_ns = now + OWNER_TIMEOUT_NS
        self._watchdog_deadline_ns = max(now, self._owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS)
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.owner_deadline_monotonic_ns = self._owner_deadline_ns
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
        self._validate_active(body.envelope, peer, source)
        limited = False
        try:
            if command == "stop":
                zero_ack = self._zero()
            else:
                if self._pending_stop is not None:
                    raise ActuatorError("MOTION_STOP_PENDING", "等待实际停稳，不能提交新的运动。")
                self._stop_confirmation.note_motion()
                limited = self._actuator.apply_motion(body)
        except ActuatorError as exc:
            if command == "stop":
                self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        if command == "stop":
            applied_ns = zero_ack.applied_monotonic_ns
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_ZEROED))
            outcome.zero_ack.CopyFrom(zero_ack)
        else:
            applied_ns = self._monotonic_ns()
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_APPLIED))
        outcome.applied_monotonic_ns = applied_ns
        outcome.limited = limited
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
        self._validate_active(body.envelope, peer, source)
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
        response, outcome = self._response(request)
        if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
            applied_ns = zero_ack.applied_monotonic_ns
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
        )
        if cached is not None:
            return cached
        body = request.revoke_zero_and_switch_mode
        if body.envelope.action != pb.GUARD_ACTION_MODE_SWITCH:
            raise GuardFailure("INVALID_ARGUMENT", "mode switch action mismatch")
        self._validate_active(body.envelope, peer, source)
        self.lease_state = pb.LEASE_STATE_PAUSING
        try:
            zero_ack = self._zero(wait=True)
            self._stop_confirmation.note_motion()
            active_mode = self._actuator.switch_mode(body.command)
            self._complete_lease_zero()
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
        outcome.zero_ack.CopyFrom(zero_ack)
        outcome.active_mode = active_mode
        self._remember(request, peer, source, response, priority=True)
        return response

    def _handle_stop_all_motion(self, request, peer, source):
        return self._revoke_response(request, peer, source, global_stop=True)

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
        self._prune_operation_records()
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
        saved = pb.MotionGuardResponse.FromString(record.response_bytes)
        saved_outcome = getattr(saved, saved.WhichOneof("outcome"))
        outcome.result.CopyFrom(self._result(record.ack))
        if saved_outcome.result.HasField("error"):
            outcome.result.error.CopyFrom(saved_outcome.result.error)
        outcome.target_ack = record.ack
        outcome.resolved = True
        if record.zero_ack_id:
            if record.stationary_zero_ack_bytes:
                outcome.zero_ack.ParseFromString(record.stationary_zero_ack_bytes)
            else:
                outcome.zero_ack.CopyFrom(saved_outcome.zero_ack)
        outcome.motion_stop_generation = record.motion_stop_generation
        return response

    def _prepare_mutation(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        *,
        namespace: str,
    ) -> Optional[pb.MotionGuardResponse]:
        envelope = operation_envelope(request)
        if envelope is None:
            raise ProtocolViolation("mutation request has no envelope")
        if not envelope.operation_id:
            raise GuardFailure("INVALID_ARGUMENT", "operation_id is required")
        self._validate_boot_tuple(envelope, namespace=namespace)
        self._prune_operation_records()
        key = self._dedup_key(envelope, peer, source, namespace)
        record = self._ordinary_records.get(key) or self._priority_records.get(key)
        if record is not None:
            if record.operation != request.WhichOneof("operation") or record.payload != canonical_payload(request):
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
        return None

    def _validate_boot_tuple(self, envelope: pb.RequestEnvelope, *, namespace: str) -> None:
        token = envelope.tuple
        if bytes(token.boot_nonce) != self._boot_nonce:
            raise GuardFailure("STALE_EPOCH", "boot nonce is stale")
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
        if now >= self._owner_deadline_ns:
            self.tick(now)
            raise GuardFailure("OUTCOME_UNKNOWN", "lease liveness deadline expired")
        return now





    def _revoke_response(self, request, peer, source, *, global_stop=False):
        # 显式停车优先隔离当前拥有方并发零；标识、截止和旧租约均不阻止这一步。
        lease_revoked = self.lease_state == pb.LEASE_STATE_ACTIVE
        self._clear_lease()
        self.lease_state = pb.LEASE_STATE_REVOKING
        self._startup_zero_confirmed = False
        failure = None
        try:
            zero_ack = self._zero()
            self._complete_lease_zero()
            self._reset_zero_retry()
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc)
            failure = exc
        finally:
            if global_stop:
                # 协议代次按uint64递增；即使归零失败，也不沿用停车前的准入。
                self._motion_stop_generation = (self._motion_stop_generation + 1) & MAX_UINT64
        response, outcome = self._response(request)
        if failure is None:
            ack = pb.ACK_CODE_RESOURCE_FENCED if lease_revoked or not global_stop else pb.ACK_CODE_ZEROED
            outcome.result.CopyFrom(self._result(ack))
            outcome.zero_ack.CopyFrom(zero_ack)
        else:
            outcome.result.CopyFrom(self._result(pb.ACK_CODE_UNSPECIFIED, code=failure.code, message=failure.message))
        if global_stop:
            outcome.lease_revoked = lease_revoked
            outcome.motion_stop_generation = self._motion_stop_generation
        body = getattr(request, request.WhichOneof("operation"))
        reason = (body.reason.strip() or "全局停车") if global_stop else "撤销租约并归零"
        completed_ns = zero_ack.applied_monotonic_ns if failure is None else self._monotonic_ns()
        _LOGGER.info("停车请求 operation=%s operation_id=%s reason=%s expired=%s ack=%s error=%s",
                     request.WhichOneof("operation"), body.envelope.operation_id, reason,
                     completed_ns > body.envelope.deadline_monotonic_ns,
                     outcome.result.ack, failure.code if failure is not None else "")
        self._remember(request, peer, source, response,
                       namespace="global_stop" if global_stop else "lease", priority=True)
        return response

    def _mark_zero_unconfirmed(self, error: ActuatorError, *, now_ns: Optional[int] = None) -> None:
        if error.code.startswith(("MOTION_", "LOCOMOTION_")):
            self._execution_feedback_failure = error.code + ": " + error.message
        self.lease_state = pb.LEASE_STATE_REVOKING
        self._startup_zero_confirmed = False
        self._schedule_zero_retry(self._monotonic_ns() if now_ns is None else now_ns)

    def _retry_zero_fence(self, now_ns: int) -> Optional[pb.ZeroAck]:
        try:
            zero_ack = self._zero(retry=True)
            self._complete_lease_zero()
        except ActuatorError as exc:
            self._mark_zero_unconfirmed(exc, now_ns=now_ns)
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
        self._execution_feedback_failure = ""

    @property
    def stop_confirmation_timeout_seconds(self) -> float:
        return self._stop_confirmation.maximum_wait_seconds

    def _zero(self, *, wait: bool = False, retry: bool = False) -> pb.ZeroAck:
        retained = self._last_zero_confirmation if retry and self._last_zero_confirmation is not None and (
            self._pending_zero_records or self._last_zero_confirmation.confirmation_kind == "unconfirmed") else None
        self._actuator.zero()
        applied_ns = self._monotonic_ns()
        started = self._stop_confirmation.begin()
        progress = self._pending_stop
        if progress is None:
            progress = self._stop_confirmation.start(self._actuator, started)
        if isinstance(progress, StopEvidence):
            zero = self._new_zero_ack(applied_ns, progress)
        else:
            # 并发停车共享同一实际停稳窗口，但每次真实零速有独立应用票据。
            self._pending_stop = progress
            self._startup_zero_confirmed = False
            zero = self._new_zero_ack(applied_ns,
                StopEvidence(False, "awaiting_feedback" if progress.has_feedback else "awaiting_braking_bound"))
            zero.stationary_deadline_monotonic_ns = self._monotonic_ns() + int(self._stop_confirmation.remaining_seconds(progress) * 1_000_000_000)
        if retained is not None:
            # 自动补发仍属于原未完成操作，不能把后续重试时刻写成原应用时刻。
            zero.zero_ack_id = retained.zero_ack_id
            zero.applied_monotonic_ns = retained.applied_monotonic_ns
        self._last_zero_confirmation = zero
        if zero.stationary_confirmed:
            self._store_stop_confirmation()
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
        zero = self._last_zero_confirmation
        zero.stationary_confirmed = evidence.stationary_confirmed
        zero.confirmation_kind = evidence.confirmation_kind
        zero.feedback_source_timestamp_ns = evidence.feedback_source_timestamp_ns
        zero.stationary_deadline_monotonic_ns = 0
        self._execution_feedback_failure = ""
        self._pending_stop = None
        self._store_stop_confirmation()
        return pb.ZeroAck.FromString(zero.SerializeToString())

    def _store_stop_confirmation(self) -> None:
        # 多个原操作可等待同一停稳窗口；分别保留其零速身份、应用时刻和初始响应。
        current = self._last_zero_confirmation
        def supplement(zero):
            zero.stationary_confirmed = current.stationary_confirmed
            zero.confirmation_kind = current.confirmation_kind
            zero.feedback_source_timestamp_ns = current.feedback_source_timestamp_ns
            zero.stationary_deadline_monotonic_ns = current.stationary_deadline_monotonic_ns
            return zero.SerializeToString(deterministic=True)
        for priority, key, operation_id in self._pending_zero_records:
            target = self._priority_records if priority else self._ordinary_records
            record = target.get(key)
            if record is None:
                continue
            saved = pb.MotionGuardResponse.FromString(record.response_bytes)
            zero = getattr(saved, saved.WhichOneof("outcome")).zero_ack
            updated = replace(record, stationary_zero_ack_bytes=supplement(zero),
                recorded_monotonic_ns=self._monotonic_ns())
            target[key] = updated
            self._operation_index[operation_id] = [updated if item is record else item
                for item in self._operation_index.get(operation_id, ())]
        if current.stationary_confirmed:
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

    def _complete_lease_zero(self) -> None:
        """撤销租约，异步停稳完成前保持新运动未就绪。"""
        self._clear_lease()
        self._startup_zero_confirmed = self._pending_stop is None

    def _clear_lease(self) -> None:
        self._active_lease_id = 0
        self._active_source = ""
        self._owner_deadline_ns = 0
        self._watchdog_deadline_ns = 0
        self.lease_state = pb.LEASE_STATE_IDLE

    def _current_tuple(self) -> pb.LeaseTuple:
        return pb.LeaseTuple(
            boot_nonce=self._boot_nonce,
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
        self._prune_operation_records()
        operation = request.WhichOneof("operation")
        if operation == "submit_motion_command":
            if request.submit_motion_command.WhichOneof("command") != "stop":
                return
        elif operation not in {
            "acquire_lease", "revoke_and_zero", "revoke_zero_and_switch_mode", "stop_all_motion",
        }:
            return
        envelope = operation_envelope(request)
        if envelope is None or not envelope.operation_id:
            return
        if operation in {"stop_all_motion", "revoke_and_zero"}:
            # 每次显式停车都实际执行；该查询标识只保留最新结果及其停稳关联。
            for previous in tuple(self._operation_index.get(envelope.operation_id, ())):
                if previous.source == source and previous.namespace == namespace:
                    owner = (self._priority_records if self._priority_records.get(previous.key) is previous
                             else self._ordinary_records)
                    self._forget_operation_record(owner, previous.key, previous)
        key = self._dedup_key(envelope, peer, source, namespace)
        outcome_name = response.WhichOneof("outcome")
        outcome = getattr(response, outcome_name)
        result = outcome.result
        zero_ack = getattr(outcome, "zero_ack", None)
        record = OperationRecord(
            key=key,
            operation_id=envelope.operation_id,
            operation=operation,
            payload=canonical_payload(request),
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
            recorded_monotonic_ns=self._monotonic_ns(),
            acquire=acquire,
        )
        target = self._priority_records if priority else self._ordinary_records
        target[key] = record
        self._operation_index.setdefault(envelope.operation_id, []).append(record)
        if self._pending_stop is not None and record.zero_ack_id == self._last_zero_confirmation.zero_ack_id:
            self._pending_zero_records.append((priority, key, envelope.operation_id))
        self._prune_operation_records()

    def _prune_operation_records(self) -> None:
        """只淘汰已完成的历史结果，停车待确认记录留给原操作查询。"""
        pending = {(priority, key) for priority, key, _ in self._pending_zero_records}
        cutoff = self._monotonic_ns() - RETRY_HORIZON_NS
        completed = []
        for priority, target in ((False, self._ordinary_records), (True, self._priority_records)):
            for key, record in tuple(target.items()):
                if (priority, key) in pending:
                    continue
                if record.recorded_monotonic_ns < cutoff:
                    self._forget_operation_record(target, key, record)
                else:
                    completed.append((record.recorded_monotonic_ns, priority, key, record))
        excess = len(self._ordinary_records) + len(self._priority_records) - OPERATION_RECORD_CAPACITY
        if excess > 0:
            completed.sort(key=lambda entry: entry[0])
            for _, priority, key, record in completed[:excess]:
                target = self._priority_records if priority else self._ordinary_records
                self._forget_operation_record(target, key, record)

    def _forget_operation_record(self, target, key, record: OperationRecord) -> None:
        del target[key]
        retained = [item for item in self._operation_index[record.operation_id] if item is not record]
        priority = target is self._priority_records
        self._pending_zero_records = [entry for entry in self._pending_zero_records
                                      if entry[:2] != (priority, key)]
        if retained:
            self._operation_index[record.operation_id] = retained
        else:
            del self._operation_index[record.operation_id]

    def _measured_motion_feedback(self) -> Optional[pb.MeasuredMotionFeedback]:
        """附带已有真实样本，不代表当前命令后的观测，也不影响命令结果。"""
        try:
            feedback = self._read_execution_feedback()
            if feedback is None or not feedback.source_id or not feedback.source_id.strip():
                return None
            linear = (feedback.linear_x_mps, feedback.linear_y_mps, feedback.linear_z_mps)
            angular = (feedback.angular_x_rps, feedback.angular_y_rps, feedback.angular_z_rps)
            if any(value is None or not math.isfinite(value) for value in linear + angular):
                return None
            orientation = feedback.orientation_wxyz
            if orientation is not None and not 0.9 <= math.sqrt(sum(v * v for v in orientation)) <= 1.1:
                return None
            deadline = self._stop_confirmation.feedback_valid_until_monotonic_ns(feedback)
            if deadline is None or self._monotonic_ns() > deadline:
                return None
            return pb.MeasuredMotionFeedback(
                source_id=feedback.source_id,
                source_timestamp_ns=feedback.source_timestamp_ns,
                received_monotonic_ns=feedback.received_monotonic_ns,
                valid_until_monotonic_ns=deadline,
                frame_id=feedback.frame_id,
                body_linear_xyz=linear,
                body_angular_xyz=angular,
                orientation_wxyz=orientation or (),
            )
        except Exception:
            # 旁路测量不可用仅表示未知，不能阻塞租约、指令、停车或错误回执。
            return None

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
        measured = self._measured_motion_feedback()
        if measured is not None:
            response.measured_motion_feedback.CopyFrom(measured)
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
