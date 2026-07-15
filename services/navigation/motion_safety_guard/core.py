from __future__ import annotations

from dataclasses import dataclass
import secrets
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple
import uuid

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import (
    API_VERSION,
    DEDUP_CAPACITY_DEFAULT,
    LEASE_ABSOLUTE_NS,
    MAX_COMMAND_DEADLINE_NS,
    OWNER_TIMEOUT_NS,
    WATCHDOG_FENCE_LEAD_NS,
)
from .epoch import EpochError, EpochRecord, EpochStore, MAX_GENERATION
from .go2_actuator import ActuatorBackend, ActuatorError
from .peer_auth import AuthorizationError, PeerAuthorizer, PeerCredentials
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
    peer_uid: int
    source: str
    lease_id: int
    response_bytes: bytes
    ack: int
    zero_ack_id: str
    zero_applied_ns: int
    acquire: bool = False


_GATE_TO_EPOCH = {
    pb.SAFETY_GATE_RESET_REQUIRED: "ResetRequired",
    pb.SAFETY_GATE_ARMED: "Armed",
}
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
    "ADMIN": frozenset(),
}


class MotionGuardCore:
    """Single-threaded arbitration state for all eleven motion-guard RPCs."""

    def __init__(
        self,
        *,
        epoch: EpochRecord,
        epoch_store: EpochStore,
        actuator: ActuatorBackend,
        authorizer: PeerAuthorizer,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        lease_id_factory: Optional[Callable[[], int]] = None,
        dedup_capacity: int = DEDUP_CAPACITY_DEFAULT,
    ) -> None:
        if dedup_capacity < 4:
            raise ValueError("dedup_capacity must reserve normal and emergency entries")
        self.epoch = epoch
        self._epoch_store = epoch_store
        self._actuator = actuator
        self._authorizer = authorizer
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
        self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
        self.lease_state = pb.LEASE_STATE_IDLE
        self._active_lease_id = 0
        self._active_peer_uid = -1
        self._active_source = ""
        self._last_sequence = 0
        self._lease_deadline_ns = 0
        self._owner_deadline_ns = 0
        self._watchdog_deadline_ns = 0
        self._state_lock = threading.RLock()
        self._ordinary_capacity = dedup_capacity - max(1, dedup_capacity // 4)
        self._emergency_capacity = dedup_capacity - self._ordinary_capacity
        self._ordinary_records: Dict[Tuple[object, ...], OperationRecord] = {}
        self._emergency_records: Dict[Tuple[object, ...], OperationRecord] = {}
        self._operation_index: Dict[str, List[OperationRecord]] = {}

    @property
    def active_lease_id(self) -> int:
        return self._active_lease_id

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
            self._authorizer.authorize(peer.uid, source)
            if request.api_version != API_VERSION:
                raise GuardFailure(
                    "UNSUPPORTED_API_VERSION",
                    f"api_version {request.api_version} is unsupported",
                )
            handler = getattr(self, f"_handle_{operation}")
            return handler(request, peer, source)
        except AuthorizationError as exc:
            return self._error_response(request, "PERMISSION_DENIED", str(exc))
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
            self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
            self.lease_state = pb.LEASE_STATE_IDLE
            return self._error_response(
                request,
                "EPOCH_DURABILITY_LOST",
                str(exc),
            )

    def tick(self, now_ns: Optional[int] = None) -> Optional[pb.ZeroAck]:
        with self._state_lock:
            if self.lease_state != pb.LEASE_STATE_ACTIVE:
                return None
            now = self._monotonic_ns() if now_ns is None else now_ns
            if now < min(self._watchdog_deadline_ns, self._lease_deadline_ns):
                return None
            self.lease_state = pb.LEASE_STATE_REVOKING
            try:
                zero_ack = self._zero()
                self._clear_lease()
                self._commit_epoch(
                    gate=pb.SAFETY_GATE_ARMED,
                    lease=pb.LEASE_STATE_IDLE,
                    fenced=True,
                    last_zero_ack_id=zero_ack.zero_ack_id,
                )
                return zero_ack
            except (ActuatorError, EpochError):
                self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
                self.lease_state = pb.LEASE_STATE_REVOKING
                try:
                    self._commit_epoch(
                        gate=pb.SAFETY_GATE_RESET_REQUIRED,
                        lease=pb.LEASE_STATE_REVOKING,
                        fenced=True,
                    )
                except EpochError:
                    pass
                return None

    def force_safe_fence(self) -> pb.ZeroAck:
        """Zero all owned paths and durably return to RESET_REQUIRED/IDLE."""
        with self._state_lock:
            self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
            self.lease_state = pb.LEASE_STATE_REVOKING
            precommit_error: Optional[EpochError] = None
            try:
                self._commit_epoch(
                    gate=pb.SAFETY_GATE_RESET_REQUIRED,
                    lease=pb.LEASE_STATE_REVOKING,
                    fenced=True,
                )
            except EpochError as exc:
                precommit_error = exc
            try:
                zero_ack = self._zero()
            except ActuatorError as exc:
                self._fail_zero_or_switch(exc)
                raise
            self._clear_lease()
            self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
            if precommit_error is not None:
                raise precommit_error
            self._commit_epoch(
                gate=pb.SAFETY_GATE_RESET_REQUIRED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
            )
            return zero_ack

    def _handle_get_readiness(
        self,
        request: pb.MotionGuardRequest,
        _peer: PeerCredentials,
        _source: str,
    ) -> pb.MotionGuardResponse:
        operational = (
            self._epoch_durable
            and self._actuator_configured
            and self.safety_gate == pb.SAFETY_GATE_ARMED
        )
        if not self._epoch_durable:
            reason = "EPOCH_NOT_DURABLE"
        elif not self._actuator_configured:
            reason = self._actuator_reason
        elif self.safety_gate != pb.SAFETY_GATE_ARMED:
            reason = "RESET_REQUIRED"
        else:
            reason = "OK"
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.ready = operational
        outcome.actuator_configured = self._actuator_configured
        outcome.epoch_durable = self._epoch_durable
        outcome.reason = reason
        outcome.generation = self.epoch.generation
        outcome.boot_nonce = self.epoch.boot_nonce_bytes
        return response

    def _handle_get_safety_state(
        self,
        request: pb.MotionGuardRequest,
        _peer: PeerCredentials,
        _source: str,
    ) -> pb.MotionGuardResponse:
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.generation = self.epoch.generation
        outcome.boot_nonce = self.epoch.boot_nonce_bytes
        outcome.reset_generation = self.epoch.reset_generation
        outcome.last_zero_ack_id = self.epoch.last_zero_ack_id
        outcome.fenced = self.epoch.fenced
        return response

    def _handle_reset_safe_stop(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        body = request.reset_safe_stop
        cached = self._prepare_mutation(
            request,
            peer,
            source,
            namespace="admin",
            emergency=True,
        )
        if cached is not None:
            return cached
        if source != "ADMIN":
            raise GuardFailure("PERMISSION_DENIED", "ResetSafeStop requires ADMIN source")
        if not body.confirm_physical_safe or not body.reason.strip() or not body.operator.strip():
            raise GuardFailure(
                "INVALID_ARGUMENT",
                "physical confirmation, reason, and operator are required",
            )
        if not self._actuator_configured:
            raise GuardFailure("MOTION_SINK_NOT_READY", self._actuator_reason)
        if self.epoch.reset_generation == MAX_GENERATION:
            raise GuardFailure("RESOURCE_EXHAUSTED", "reset_generation overflow")
        try:
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                gate=pb.SAFETY_GATE_ARMED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
                reset_generation=self.epoch.reset_generation + 1,
            )
        except ActuatorError as exc:
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_ZEROED))
        outcome.zero_ack.CopyFrom(zero_ack)
        outcome.reset_generation = self.epoch.reset_generation
        self._remember(request, peer, source, response, emergency=True)
        return response

    def _handle_acquire_lease(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
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
        if not self._actuator_configured or not self._epoch_durable:
            raise GuardFailure("MOTION_SINK_NOT_READY", self._actuator_reason)
        if self.safety_gate != pb.SAFETY_GATE_ARMED:
            raise GuardFailure("SAFETY_STOP_ACTIVE", "physical reset is required")
        if self.lease_state != pb.LEASE_STATE_IDLE:
            raise GuardFailure("RESOURCE_BUSY", "motion resource already has an owner", True)
        envelope = request.acquire_lease.envelope
        if envelope.sequence != 0:
            raise GuardFailure("INVALID_ARGUMENT", "AcquireLease sequence must be zero")
        now = self._monotonic_ns()
        lease_id = int(self._lease_id_factory())
        if lease_id <= 0 or lease_id > MAX_GENERATION:
            raise GuardFailure("INTERNAL", "lease_id factory returned an invalid u64")
        self._active_lease_id = lease_id
        self._active_peer_uid = peer.uid
        self._active_source = source
        self._last_sequence = 0
        self._lease_deadline_ns = now + LEASE_ABSOLUTE_NS
        self._owner_deadline_ns = min(now + OWNER_TIMEOUT_NS, self._lease_deadline_ns)
        self._watchdog_deadline_ns = max(
            now,
            self._owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS,
        )
        self._commit_epoch(
            gate=pb.SAFETY_GATE_ARMED,
            lease=pb.LEASE_STATE_ACTIVE,
            fenced=False,
        )
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RECEIVED))
        outcome.lease_tuple.CopyFrom(self._current_tuple())
        outcome.lease_deadline_monotonic_ns = self._lease_deadline_ns
        outcome.owner_deadline_monotonic_ns = self._owner_deadline_ns
        self._remember(request, peer, source, response, acquire=True)
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
            self._actuator.apply_motion(body)
        except ActuatorError as exc:
            if command == "stop":
                self._fail_zero_or_switch(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        self._ensure_lease_still_active(body.envelope, peer, source)
        response, outcome = self._response(request)
        if command == "stop":
            zero_ack = self._new_zero_ack(self._monotonic_ns())
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
            self._actuator.execute_sport(body.command)
        except ActuatorError as exc:
            if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
                self._fail_zero_or_switch(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        self._ensure_lease_still_active(body.envelope, peer, source)
        response, outcome = self._response(request)
        if body.command.action == pb.SPORT_ACTION_STOP_MOVE:
            zero_ack = self._new_zero_ack(self._monotonic_ns())
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
            emergency=True,
        )
        if cached is not None:
            return cached
        envelope = request.revoke_and_zero.envelope
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
            emergency=True,
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
            zero_ack = self._zero()
            active_mode = self._actuator.switch_mode(body.command)
            self._clear_lease()
            self._commit_epoch(
                gate=pb.SAFETY_GATE_ARMED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
            )
        except ActuatorError as exc:
            self._fail_zero_or_switch(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
        outcome.zero_ack.CopyFrom(zero_ack)
        outcome.active_mode = active_mode
        self._remember(request, peer, source, response, emergency=True)
        return response

    def _handle_latch_safety_stop(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        body = request.latch_safety_stop
        manual_global_latch = (
            source == "GATEWAY_MANUAL" and body.envelope.tuple.lease_id == 0
        )
        if manual_global_latch and body.envelope.action != pb.GUARD_ACTION_STOP:
            raise GuardFailure(
                "ACTION_NOT_ALLOWED",
                "GATEWAY_MANUAL global latch requires STOP action",
            )
        namespace = (
            "admin"
            if source == "ADMIN"
            else "emergency"
            if manual_global_latch
            else "lease"
        )
        cached = self._prepare_mutation(
            request,
            peer,
            source,
            namespace=namespace,
            emergency=True,
        )
        if cached is not None:
            return cached
        if not body.reason.strip():
            raise GuardFailure("INVALID_ARGUMENT", "safe-stop reason is required")
        if source != "ADMIN" and not manual_global_latch:
            self._begin_lease_operation(body.envelope, peer, source)
        self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            self._commit_epoch(
                gate=pb.SAFETY_GATE_RESET_REQUIRED,
                lease=pb.LEASE_STATE_REVOKING,
                fenced=True,
            )
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                gate=pb.SAFETY_GATE_RESET_REQUIRED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
            )
        except ActuatorError as exc:
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_SAFETY_LATCHED))
        outcome.zero_ack.CopyFrom(zero_ack)
        self._remember(request, peer, source, response, emergency=True)
        return response

    def _handle_get_operation_result(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
    ) -> pb.MotionGuardResponse:
        operation_id = request.get_operation_result.operation_id
        response, outcome = self._response(request)
        outcome.target_operation_id = operation_id
        records = self._operation_index.get(operation_id, ())
        record = next(
            (
                candidate
                for candidate in records
                if source == "ADMIN"
                or (candidate.peer_uid == peer.uid and candidate.source == source)
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
            outcome.zero_ack.zero_ack_id = record.zero_ack_id
            outcome.zero_ack.applied_monotonic_ns = record.zero_applied_ns
        return response

    def _prepare_mutation(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        *,
        namespace: str,
        emergency: bool = False,
    ) -> Optional[pb.MotionGuardResponse]:
        verify_payload_sha256(request)
        envelope = operation_envelope(request)
        if envelope is None:
            raise ProtocolViolation("mutation request has no envelope")
        if not envelope.operation_id:
            raise GuardFailure("INVALID_ARGUMENT", "operation_id is required")
        self._validate_epoch_tuple(envelope, namespace=namespace)
        key = self._dedup_key(envelope, peer, source, namespace)
        record = self._ordinary_records.get(key) or self._emergency_records.get(key)
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
        target = self._emergency_records if emergency else self._ordinary_records
        capacity = self._emergency_capacity if emergency else self._ordinary_capacity
        if len(target) >= capacity:
            self._cache_exhausted()
            raise GuardFailure("RESOURCE_EXHAUSTED", "dedup cache is exhausted")
        return None

    def _validate_epoch_tuple(self, envelope: pb.RequestEnvelope, *, namespace: str) -> None:
        token = envelope.tuple
        if token.generation != self.epoch.generation or bytes(token.boot_nonce) != self.epoch.boot_nonce_bytes:
            raise GuardFailure("STALE_EPOCH", "generation or boot nonce is stale")
        if namespace in {"acquire", "admin", "emergency"} and token.lease_id != 0:
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
        if peer.uid != self._active_peer_uid or source != self._active_source:
            raise GuardFailure("PERMISSION_DENIED", "peer/source does not own the lease")
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
                or peer.uid != self._active_peer_uid
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
                gate=pb.SAFETY_GATE_ARMED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
            )
        except ActuatorError as exc:
            self._fail_zero_or_switch(exc)
            raise GuardFailure(exc.code, exc.message) from exc
        response, outcome = self._response(request)
        outcome.result.CopyFrom(self._result(pb.ACK_CODE_RESOURCE_FENCED))
        outcome.zero_ack.CopyFrom(zero_ack)
        self._remember(request, peer, source, response, emergency=True)
        return response

    def _fail_zero_or_switch(self, error: ActuatorError) -> None:
        del error
        self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            self._commit_epoch(
                gate=pb.SAFETY_GATE_RESET_REQUIRED,
                lease=pb.LEASE_STATE_REVOKING,
                fenced=True,
            )
        except EpochError:
            pass

    def _zero(self) -> pb.ZeroAck:
        self._actuator.zero()
        return self._new_zero_ack(self._monotonic_ns())

    @staticmethod
    def _new_zero_ack(now_ns: int) -> pb.ZeroAck:
        return pb.ZeroAck(
            zero_ack_id=str(uuid.uuid4()),
            applied_monotonic_ns=now_ns,
        )

    def _cache_exhausted(self) -> None:
        self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
        self.lease_state = pb.LEASE_STATE_REVOKING
        try:
            zero_ack = self._zero()
            self._clear_lease()
            self._commit_epoch(
                gate=pb.SAFETY_GATE_RESET_REQUIRED,
                lease=pb.LEASE_STATE_IDLE,
                fenced=True,
                last_zero_ack_id=zero_ack.zero_ack_id,
            )
        except (ActuatorError, EpochError):
            try:
                self._commit_epoch(
                    gate=pb.SAFETY_GATE_RESET_REQUIRED,
                    lease=pb.LEASE_STATE_REVOKING,
                    fenced=True,
                )
            except EpochError:
                pass

    def _commit_epoch(
        self,
        *,
        gate: Optional[int] = None,
        lease: Optional[int] = None,
        fenced: Optional[bool] = None,
        last_zero_ack_id: Optional[str] = None,
        reset_generation: Optional[int] = None,
    ) -> None:
        next_gate = self.safety_gate if gate is None else gate
        next_lease = self.lease_state if lease is None else lease
        updates: Dict[str, object] = {
            "safety_gate": _GATE_TO_EPOCH[next_gate],
            "lease_state": _LEASE_TO_EPOCH[next_lease],
        }
        if fenced is not None:
            updates["fenced"] = fenced
        if last_zero_ack_id is not None:
            updates["last_zero_ack_id"] = last_zero_ack_id
        if reset_generation is not None:
            updates["reset_generation"] = reset_generation
        next_epoch = self.epoch.with_updates(**updates)
        try:
            self._epoch_store.persist(next_epoch)
        except (OSError, EpochError) as exc:
            self._epoch_durable = False
            self.safety_gate = pb.SAFETY_GATE_RESET_REQUIRED
            raise EpochError(f"epoch durable commit failed: {exc}") from exc
        self.epoch = next_epoch
        self.safety_gate = next_gate
        self.lease_state = next_lease

    def _clear_lease(self) -> None:
        self._active_lease_id = 0
        self._active_peer_uid = -1
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
        envelope: pb.RequestEnvelope,
        peer: PeerCredentials,
        source: str,
        namespace: str,
    ) -> Tuple[object, ...]:
        if namespace == "acquire":
            return (
                namespace,
                bytes(envelope.tuple.boot_nonce),
                peer.uid,
                source,
                envelope.operation_id,
            )
        if namespace in {"admin", "emergency"}:
            return (
                namespace,
                peer.uid,
                source,
                envelope.tuple.generation,
                bytes(envelope.tuple.boot_nonce),
                envelope.operation_id,
            )
        return (
            namespace,
            peer.uid,
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
        return self._ordinary_records.get(key) or self._emergency_records.get(key)

    def _remember(
        self,
        request: pb.MotionGuardRequest,
        peer: PeerCredentials,
        source: str,
        response: pb.MotionGuardResponse,
        *,
        emergency: bool = False,
        acquire: bool = False,
    ) -> None:
        envelope = operation_envelope(request)
        if envelope is None:
            return
        namespace = "acquire" if acquire else (
            "admin"
            if request.WhichOneof("operation") in {"reset_safe_stop"}
            or (
                request.WhichOneof("operation") == "latch_safety_stop"
                and source == "ADMIN"
            )
            else "emergency"
            if (
                request.WhichOneof("operation") == "latch_safety_stop"
                and source == "GATEWAY_MANUAL"
                and envelope.tuple.lease_id == 0
            )
            else "lease"
        )
        key = self._dedup_key(envelope, peer, source, namespace)
        outcome_name = response.WhichOneof("outcome")
        outcome = getattr(response, outcome_name)
        result = outcome.result
        zero_ack = getattr(outcome, "zero_ack", None)
        record = OperationRecord(
            key=key,
            operation_id=envelope.operation_id,
            payload_sha256=bytes(envelope.payload_sha256),
            peer_uid=peer.uid,
            source=source,
            lease_id=(
                outcome.lease_tuple.lease_id
                if acquire
                else envelope.tuple.lease_id
            ),
            response_bytes=response.SerializeToString(deterministic=True),
            ack=result.ack,
            zero_ack_id=(zero_ack.zero_ack_id if zero_ack is not None else ""),
            zero_applied_ns=(
                zero_ack.applied_monotonic_ns if zero_ack is not None else 0
            ),
            acquire=acquire,
        )
        target = self._emergency_records if emergency else self._ordinary_records
        target[key] = record
        self._operation_index.setdefault(envelope.operation_id, []).append(record)

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
        result = pb.GuardResult(
            ack=ack,
            gate=self.safety_gate,
            lease=self.lease_state,
        )
        if code:
            result.error.code = code
            result.error.message = message
            result.error.retryable = retryable
        return result
