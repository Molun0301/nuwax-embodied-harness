from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import os
import socket
import threading
import time
from typing import Callable, Collection, Dict, Optional, Set, Tuple, Type, Union
import uuid

from google.protobuf.message import DecodeError, Message

from services.navigation.motion_safety_guard.constants import (
    ACTOR_POLL_INTERVAL_SECONDS,
    API_VERSION,
    DEFAULT_SOCKET_PATH,
    MAX_COMMAND_DEADLINE_NS,
    MAX_FRAME_BYTES,
    MAX_PRODUCTION_SDK_TIMEOUT_SECONDS,
    RENEW_INTERVAL_NS,
    WATCHDOG_FENCE_LEAD_NS,
)
from services.navigation.motion_safety_guard.framing import (
    FramingError,
    recv_frame,
    send_frame,
)
from services.navigation.motion_safety_guard.wire import seal_request
from services.navigation.proto_gen import motion_guard_pb2 as pb


DEFAULT_TIMEOUT_SECONDS = 1.0
_BOOT_NONCE_BYTES = 16
_RENEW_EXECUTION_RESERVE_NS = int(
    (MAX_PRODUCTION_SDK_TIMEOUT_SECONDS + ACTOR_POLL_INTERVAL_SECONDS) * 1_000_000_000
)


class MotionGuardError(RuntimeError):
    """Base class for observable motion-guard client failures."""


class MotionGuardUsageError(MotionGuardError, ValueError):
    """The caller supplied a value that cannot satisfy the wire contract."""


class MotionGuardProtocolError(MotionGuardError):
    """The peer returned a response that violates the protobuf contract."""


class MotionGuardTransportError(MotionGuardError):
    """A request may not have reached a correlated terminal outcome."""

    def __init__(
        self,
        message: str,
        *,
        operation_id: str = "",
        outcome_unknown: bool = False,
    ) -> None:
        super().__init__(message)
        self.operation_id = operation_id
        self.outcome_unknown = outcome_unknown


class MotionGuardTimeoutError(MotionGuardTransportError, TimeoutError):
    pass


@dataclass(frozen=True)
class GuardResultSnapshot:
    ack: int
    lease: int
    error_code: str = ""
    error_message: str = ""
    retryable: bool = False


class MotionGuardRemoteError(MotionGuardError):
    """A typed application error returned by the motion guard."""

    def __init__(
        self,
        *,
        operation: str,
        operation_id: str,
        result: GuardResultSnapshot,
    ) -> None:
        message = result.error_message or result.error_code or "motion guard rejected request"
        super().__init__(f"{result.error_code}: {message}")
        self.operation = operation
        self.operation_id = operation_id
        self.code = result.error_code
        self.retryable = result.retryable
        self.result = result


class InvalidArgumentError(MotionGuardRemoteError):
    pass


class PermissionDeniedError(MotionGuardRemoteError):
    pass


class ActionNotAllowedError(MotionGuardRemoteError):
    pass


class UnsupportedApiVersionError(MotionGuardRemoteError):
    pass


class MotionSinkNotReadyError(MotionGuardRemoteError):
    pass


class RecoveryFencedError(MotionGuardRemoteError):
    pass


class ResourceBusyError(MotionGuardRemoteError):
    pass


class ResourceExhaustedError(MotionGuardRemoteError):
    pass


class DeadlineExceededError(MotionGuardRemoteError):
    pass


class StaleEpochError(MotionGuardRemoteError):
    pass


class StaleSequenceError(MotionGuardRemoteError):
    pass


class StaleStopGenerationError(MotionGuardRemoteError):
    pass


class OutcomeUnknownError(MotionGuardRemoteError):
    pass


class RequestIdConflictError(MotionGuardRemoteError):
    pass


class EpochDurabilityLostError(MotionGuardRemoteError):
    pass


class MotionGuardInternalError(MotionGuardRemoteError):
    pass


_REMOTE_ERRORS: Dict[str, Type[MotionGuardRemoteError]] = {
    "INVALID_ARGUMENT": InvalidArgumentError,
    "PERMISSION_DENIED": PermissionDeniedError,
    "ACTION_NOT_ALLOWED": ActionNotAllowedError,
    "UNSUPPORTED_API_VERSION": UnsupportedApiVersionError,
    "MOTION_SINK_NOT_READY": MotionSinkNotReadyError,
    "RECOVERY_FENCED": RecoveryFencedError,
    "RESOURCE_BUSY": ResourceBusyError,
    "RESOURCE_EXHAUSTED": ResourceExhaustedError,
    "DEADLINE_EXCEEDED": DeadlineExceededError,
    "STALE_EPOCH": StaleEpochError,
    "STALE_SEQUENCE": StaleSequenceError,
    "STALE_STOP_GENERATION": StaleStopGenerationError,
    "OUTCOME_UNKNOWN": OutcomeUnknownError,
    "REQUEST_ID_CONFLICT": RequestIdConflictError,
    "EPOCH_DURABILITY_LOST": EpochDurabilityLostError,
    "INTERNAL": MotionGuardInternalError,
}


@dataclass(frozen=True)
class MotionAdmissionProof:
    generation: int
    boot_nonce: bytes
    motion_stop_generation: int = 0


@dataclass(frozen=True)
class ReadinessStatus:
    result: GuardResultSnapshot
    ready: bool
    actuator_configured: bool
    epoch_durable: bool
    startup_zero_confirmed: bool
    reason: str
    generation: int
    boot_nonce: bytes
    motion_stop_generation: int
    recovery_fence_held: bool = False

    def require_admission_proof(self) -> MotionAdmissionProof:
        if not self.ready:
            raise MotionGuardNotReadyError(self)
        return MotionAdmissionProof(
            generation=self.generation,
            boot_nonce=bytes(self.boot_nonce),
            motion_stop_generation=self.motion_stop_generation,
        )


@dataclass(frozen=True)
class GuardState:
    result: GuardResultSnapshot
    generation: int
    boot_nonce: bytes
    last_zero_ack_id: str
    fenced: bool
    active_lease_id: int
    motion_stop_generation: int
    recovery_fence_held: bool = False
    recovery_fence_token_sha256: bytes = b""
    recovery_fence_correlation_sha256: bytes = b""


@dataclass(frozen=True)
class ZeroConfirmation:
    zero_ack_id: str
    applied_monotonic_ns: int


@dataclass(frozen=True)
class CommandConfirmation:
    operation_id: str
    result: GuardResultSnapshot
    applied_monotonic_ns: int
    zero: Optional[ZeroConfirmation] = None


@dataclass(frozen=True)
class ModeSwitchConfirmation:
    operation_id: str
    result: GuardResultSnapshot
    zero: ZeroConfirmation
    active_mode: int


@dataclass(frozen=True)
class StopAllMotionConfirmation:
    operation_id: str
    result: GuardResultSnapshot
    zero: ZeroConfirmation
    lease_revoked: bool
    motion_stop_generation: int
    recovery_fence_token: bytes = b""
    recovery_fence_held: bool = False
    generation: int = 0
    boot_nonce: bytes = b""


@dataclass(frozen=True)
class OperationStatus:
    result: GuardResultSnapshot
    operation_id: str
    resolved: bool
    target_ack: int
    zero: Optional[ZeroConfirmation]
    motion_stop_generation: int
    recovery_fence_token: bytes = b""
    recovery_fence_held: bool = False


class MotionGuardNotReadyError(MotionGuardError):
    def __init__(self, readiness: ReadinessStatus) -> None:
        super().__init__(f"motion guard is not ready: {readiness.reason}")
        self.readiness = readiness


class MotionGuardSessionClosedError(MotionGuardError):
    pass


class MotionGuardFenceError(MotionGuardError):
    """The caller did not receive a correlated durable zero acknowledgement."""

    def __init__(
        self,
        message: str,
        *,
        cause: Optional[BaseException] = None,
        zero: Optional[ZeroConfirmation] = None,
        prior_failure: Optional[BaseException] = None,
    ) -> None:
        super().__init__(message)
        self.cause = cause
        self.zero = zero
        self.prior_failure = prior_failure


def normalize_source(source: Union[int, str]) -> int:
    if isinstance(source, bool):
        raise MotionGuardUsageError("GuardSource must not be a boolean")
    if isinstance(source, str):
        name = source.strip().upper()
        if not name.startswith("GUARD_SOURCE_"):
            name = f"GUARD_SOURCE_{name}"
        try:
            source = pb.GuardSource.Value(name)
        except ValueError as exc:
            raise MotionGuardUsageError(f"unknown GuardSource {source!r}") from exc
    if not isinstance(source, int):
        raise MotionGuardUsageError("GuardSource must be an int or enum name")
    try:
        pb.GuardSource.Name(source)
    except ValueError as exc:
        raise MotionGuardUsageError(f"unknown GuardSource value {source}") from exc
    if source == pb.GUARD_SOURCE_UNSPECIFIED:
        raise MotionGuardUsageError("GUARD_SOURCE_UNSPECIFIED is not usable")
    return source


class MotionGuardClient:
    """Thread-safe, persistent AF_UNIX client for the authoritative guard proto."""

    def __init__(
        self,
        socket_path: Union[str, os.PathLike[str]] = DEFAULT_SOCKET_PATH,
        *,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        max_frame_bytes: int = MAX_FRAME_BYTES,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        id_factory: Optional[Callable[[], str]] = None,
    ) -> None:
        if timeout_seconds <= 0:
            raise MotionGuardUsageError("timeout_seconds must be positive")
        if max_frame_bytes <= 0:
            raise MotionGuardUsageError("max_frame_bytes must be positive")
        self.socket_path = os.fspath(socket_path)
        self.timeout_seconds = float(timeout_seconds)
        self.max_frame_bytes = int(max_frame_bytes)
        self._monotonic_ns = monotonic_ns
        self._id_factory = id_factory or (lambda: str(uuid.uuid4()))
        self._io_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._sessions: Set[MotionGuardSession] = set()
        self._closed = False
        self._closing = False
        self._failure: Optional[BaseException] = None
        self._socket = self._connect()

    def _connect(self) -> socket.socket:
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        connection.settimeout(self.timeout_seconds)
        try:
            connection.connect(self.socket_path)
        except (OSError, TimeoutError) as exc:
            connection.close()
            raise MotionGuardTransportError(
                f"failed to connect to motion guard socket {self.socket_path}: {exc}"
            ) from exc
        return connection

    @property
    def failed(self) -> bool:
        return self._failure is not None

    def __enter__(self) -> MotionGuardClient:
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        del exc_type, traceback
        try:
            self.close()
        except MotionGuardError as close_error:
            if exc is not None:
                close_error.add_note(f"client context body also raised: {exc!r}")
                raise close_error from exc
            raise
        return False

    def close(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closing = True
            sessions = tuple(self._sessions)
        first_error: Optional[MotionGuardError] = None
        for session in sessions:
            try:
                session.close()
            except MotionGuardError as exc:
                if first_error is None:
                    first_error = exc
        with self._io_lock:
            with self._lifecycle_lock:
                self._close_socket()
                self._closed = True
                self._closing = False
        if first_error is not None:
            raise first_error

    def _close_socket(self) -> None:
        connection, self._socket = self._socket, None
        if connection is None:
            return
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()

    def _register_session(self, session: MotionGuardSession) -> None:
        with self._lifecycle_lock:
            if self._closed or self._closing:
                raise MotionGuardSessionClosedError("motion guard client is closing")
            self._sessions.add(session)
            session._registered = True

    def _unregister_session(self, session: MotionGuardSession) -> None:
        with self._lifecycle_lock:
            self._sessions.discard(session)

    def _new_id(self, label: str) -> str:
        value = str(self._id_factory()).strip()
        if not value:
            raise MotionGuardUsageError("id_factory returned an empty identifier")
        return f"{label}:{value}"

    def _now(self) -> int:
        now = int(self._monotonic_ns())
        if now <= 0:
            raise MotionGuardUsageError("monotonic clock must return a positive value")
        return now

    def _new_envelope(
        self,
        *,
        lease_tuple: pb.LeaseTuple,
        source: int,
        action: int,
        operation_id: str,
        sequence: int,
        motion_stop_generation: int = 0,
    ) -> pb.RequestEnvelope:
        now = self._now()
        deadline = now + MAX_COMMAND_DEADLINE_NS
        if deadline >= 1 << 64:
            raise MotionGuardUsageError("monotonic deadline exceeds uint64")
        return pb.RequestEnvelope(
            tuple=pb.LeaseTuple(
                generation=lease_tuple.generation,
                boot_nonce=bytes(lease_tuple.boot_nonce),
                lease_id=lease_tuple.lease_id,
            ),
            source=source,
            action=action,
            operation_id=operation_id,
            sequence=sequence,
            issued_monotonic_ns=now,
            deadline_monotonic_ns=deadline,
            motion_stop_generation=motion_stop_generation,
        )

    def _new_global_stop_envelope(self, operation_id: str) -> pb.GlobalStopEnvelope:
        now = self._now()
        deadline = now + MAX_COMMAND_DEADLINE_NS
        if deadline >= 1 << 64:
            raise MotionGuardUsageError("monotonic deadline exceeds uint64")
        return pb.GlobalStopEnvelope(
            source=pb.GUARD_SOURCE_GATEWAY_MANUAL,
            operation_id=operation_id,
            issued_monotonic_ns=now,
            deadline_monotonic_ns=deadline,
        )

    def _exchange(
        self,
        request: pb.MotionGuardRequest,
        *,
        expected_outcome: str,
        expected_acks: Optional[Collection[int]],
        allow_remote_error: bool = False,
    ) -> Tuple[Message, GuardResultSnapshot]:
        operation = request.WhichOneof("operation")
        if operation != expected_outcome:
            raise MotionGuardUsageError(
                f"request operation {operation!r} does not match {expected_outcome!r}"
            )
        if request.api_version != API_VERSION:
            raise MotionGuardUsageError("client request uses an unsupported api_version")
        if not request.request_id:
            raise MotionGuardUsageError("request_id is required")
        operation_id = self._operation_id(request)
        payload = request.SerializeToString(deterministic=True)
        if not payload or len(payload) > self.max_frame_bytes:
            raise MotionGuardProtocolError(
                f"serialized request size {len(payload)} is outside frame bounds"
            )

        with self._io_lock:
            self._raise_if_unusable()
            connection = self._socket
            assert connection is not None
            try:
                send_frame(
                    connection,
                    payload,
                    max_frame_bytes=self.max_frame_bytes,
                )
                response_payload = recv_frame(
                    connection,
                    max_frame_bytes=self.max_frame_bytes,
                )
            except socket.timeout as exc:
                error = MotionGuardTimeoutError(
                    "motion guard request timed out; outcome is unknown",
                    operation_id=operation_id,
                    outcome_unknown=bool(operation_id),
                )
                self._poison(error)
                raise error from exc
            except (FramingError, OSError, TimeoutError) as exc:
                error = MotionGuardTransportError(
                    f"motion guard transport failed: {exc}",
                    operation_id=operation_id,
                    outcome_unknown=bool(operation_id),
                )
                self._poison(error)
                raise error from exc

            response = pb.MotionGuardResponse()
            try:
                response.ParseFromString(response_payload)
            except DecodeError as exc:
                error = MotionGuardProtocolError("motion guard returned invalid protobuf")
                self._poison(error)
                raise error from exc
            if response.api_version != API_VERSION:
                self._raise_protocol(
                    f"response api_version {response.api_version} does not match {API_VERSION}"
                )
            if response.request_id != request.request_id:
                self._raise_protocol("response request_id does not correlate to request")
            outcome_name = response.WhichOneof("outcome")
            if outcome_name != expected_outcome:
                self._raise_protocol(
                    f"response outcome {outcome_name!r} does not match {expected_outcome!r}"
                )
            outcome = getattr(response, outcome_name)
            if "result" not in outcome.DESCRIPTOR.fields_by_name:
                self._raise_protocol("response outcome has no GuardResult")
            result = self._snapshot(outcome.result)
            if result.error_code:
                if not allow_remote_error:
                    error_type = _REMOTE_ERRORS.get(
                        result.error_code,
                        MotionGuardRemoteError,
                    )
                    raise error_type(
                        operation=operation,
                        operation_id=operation_id,
                        result=result,
                    )
            elif result.error_message or result.retryable:
                self._raise_protocol("GuardResult has error metadata without an error code")
            if expected_acks is not None and result.ack not in frozenset(expected_acks):
                self._raise_protocol(
                    f"unexpected ack {result.ack} for outcome {expected_outcome}"
                )
            return outcome, result

    @staticmethod
    def _operation_id(request: pb.MotionGuardRequest) -> str:
        operation = request.WhichOneof("operation")
        if operation is None:
            return ""
        body = getattr(request, operation)
        if "envelope" not in body.DESCRIPTOR.fields_by_name:
            return ""
        return body.envelope.operation_id

    @staticmethod
    def _snapshot(result: pb.GuardResult) -> GuardResultSnapshot:
        return GuardResultSnapshot(
            ack=result.ack,
            lease=result.lease,
            error_code=result.error.code,
            error_message=result.error.message,
            retryable=result.error.retryable,
        )

    def _raise_if_unusable(self) -> None:
        if self._failure is not None:
            raise MotionGuardTransportError(
                "motion guard client is fail-closed after a prior transport/protocol failure"
            ) from self._failure
        if self._closed or self._socket is None:
            raise MotionGuardTransportError("motion guard client is closed")

    def _raise_protocol(self, message: str) -> None:
        error = MotionGuardProtocolError(message)
        self._poison(error)
        raise error

    def _poison(self, error: BaseException) -> None:
        if self._failure is None:
            self._failure = error
        self._close_socket()

    def get_readiness(self, source: Union[int, str]) -> ReadinessStatus:
        source_value = normalize_source(source)
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._new_id("request"),
            get_readiness=pb.GetReadinessRequest(source=source_value),
        )
        outcome, result = self._exchange(
            request,
            expected_outcome="get_readiness",
            expected_acks={pb.ACK_CODE_RECEIVED},
        )
        boot_nonce = bytes(outcome.boot_nonce)
        self._validate_epoch(outcome.generation, boot_nonce)
        return ReadinessStatus(
            result=result,
            ready=outcome.ready,
            actuator_configured=outcome.actuator_configured,
            epoch_durable=outcome.epoch_durable,
            startup_zero_confirmed=outcome.startup_zero_confirmed,
            reason=outcome.reason,
            generation=outcome.generation,
            boot_nonce=boot_nonce,
            motion_stop_generation=outcome.motion_stop_generation,
            recovery_fence_held=outcome.recovery_fence_held,
        )

    def get_guard_state(self, source: Union[int, str]) -> GuardState:
        source_value = normalize_source(source)
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._new_id("request"),
            get_guard_state=pb.GetGuardStateRequest(source=source_value),
        )
        outcome, result = self._exchange(
            request,
            expected_outcome="get_guard_state",
            expected_acks={pb.ACK_CODE_RECEIVED},
        )
        boot_nonce = bytes(outcome.boot_nonce)
        self._validate_epoch(outcome.generation, boot_nonce)
        return GuardState(
            result=result,
            generation=outcome.generation,
            boot_nonce=boot_nonce,
            last_zero_ack_id=outcome.last_zero_ack_id,
            fenced=outcome.fenced,
            active_lease_id=outcome.active_lease_id,
            motion_stop_generation=outcome.motion_stop_generation,
            recovery_fence_held=outcome.recovery_fence_held,
            recovery_fence_token_sha256=bytes(
                outcome.recovery_fence_token_sha256
            ),
            recovery_fence_correlation_sha256=bytes(
                outcome.recovery_fence_correlation_sha256
            ),
        )

    def get_operation_result(
        self,
        source: Union[int, str],
        operation_id: str,
        *,
        operation_namespace: int,
    ) -> OperationStatus:
        source_value = normalize_source(source)
        target = self._require_text(operation_id, "operation_id")
        if (
            isinstance(operation_namespace, bool)
            or not isinstance(operation_namespace, int)
            or operation_namespace
            not in {
                pb.OPERATION_NAMESPACE_ACQUIRE,
                pb.OPERATION_NAMESPACE_LEASE,
                pb.OPERATION_NAMESPACE_GLOBAL_STOP,
            }
        ):
            raise MotionGuardUsageError(
                "operation_namespace must identify acquire, lease, or global stop"
            )
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._new_id("request"),
            get_operation_result=pb.GetOperationResultRequest(
                source=source_value,
                operation_id=target,
                operation_namespace=operation_namespace,
            ),
        )
        outcome, result = self._exchange(
            request,
            expected_outcome="get_operation_result",
            expected_acks=None,
            allow_remote_error=True,
        )
        if outcome.target_operation_id != target:
            self._raise_protocol("operation result target does not correlate to query")
        if outcome.resolved:
            if result.error_code or outcome.target_ack == pb.ACK_CODE_UNSPECIFIED:
                self._raise_protocol("resolved operation result has no terminal ack")
            if result.ack != outcome.target_ack:
                self._raise_protocol("operation result ack fields disagree")
            if operation_namespace == pb.OPERATION_NAMESPACE_GLOBAL_STOP and (
                outcome.target_ack
                not in {pb.ACK_CODE_ZEROED, pb.ACK_CODE_RESOURCE_FENCED}
                or not outcome.HasField("zero_ack")
                or outcome.motion_stop_generation == 0
            ):
                self._raise_protocol(
                    "resolved global stop has no terminal zero and generation proof"
                )
        elif (
            result.error_code != "OUTCOME_UNKNOWN"
            or outcome.target_ack != pb.ACK_CODE_UNSPECIFIED
        ):
            self._raise_protocol("unresolved operation result is not OUTCOME_UNKNOWN")
        zero = None
        if outcome.HasField("zero_ack"):
            zero = self._zero_confirmation(outcome.zero_ack)
        return OperationStatus(
            result=result,
            operation_id=target,
            resolved=outcome.resolved,
            target_ack=outcome.target_ack,
            zero=zero,
            motion_stop_generation=outcome.motion_stop_generation,
            recovery_fence_token=bytes(outcome.recovery_fence_token),
            recovery_fence_held=outcome.recovery_fence_held,
        )

    def acquire_session(
        self,
        source: Union[int, str],
        proof: MotionAdmissionProof,
        *,
        operation_id: Optional[str] = None,
        renew_interval_seconds: float = RENEW_INTERVAL_NS / 1_000_000_000,
    ) -> MotionGuardSession:
        source_value = normalize_source(source)
        generation, boot_nonce, stop_generation = self._validate_admission_proof(proof)
        with self._lifecycle_lock:
            if self._closed or self._closing:
                raise MotionGuardSessionClosedError("motion guard client is closing")
        operation = self._operation_id_value(operation_id, "acquire")
        epoch_tuple = pb.LeaseTuple(
            generation=generation,
            boot_nonce=boot_nonce,
            lease_id=0,
        )
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._new_id("request"),
            acquire_lease=pb.AcquireLeaseRequest(
                envelope=self._new_envelope(
                    lease_tuple=epoch_tuple,
                    source=source_value,
                    action=pb.GUARD_ACTION_UNSPECIFIED,
                    operation_id=operation,
                    sequence=0,
                    motion_stop_generation=stop_generation,
                )
            ),
        )
        seal_request(request)
        outcome, _result = self._exchange(
            request,
            expected_outcome="acquire_lease",
            expected_acks={pb.ACK_CODE_RECEIVED},
        )
        lease_tuple = pb.LeaseTuple()
        lease_tuple.CopyFrom(outcome.lease_tuple)
        if (
            lease_tuple.generation != generation
            or bytes(lease_tuple.boot_nonce) != boot_nonce
            or lease_tuple.lease_id == 0
        ):
            self._raise_protocol("AcquireLease returned an invalid lease tuple")
        session = MotionGuardSession(
            client=self,
            source=source_value,
            lease_tuple=lease_tuple,
            renew_interval_seconds=renew_interval_seconds,
            owner_deadline_monotonic_ns=outcome.owner_deadline_monotonic_ns,
        )
        try:
            self._register_session(session)
        except BaseException as registration_error:
            try:
                session._fence_without_registration()
            except MotionGuardError as fence_error:
                fence_error.add_note(
                    f"session registration also failed: {registration_error!r}"
                )
                raise fence_error from registration_error
            raise
        session._start_renewal()
        return session

    session = acquire_session

    def stop_all_motion(
        self,
        reason: str,
        *,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        return self._global_stop(
            reason,
            mode=pb.GLOBAL_STOP_MODE_ORDINARY,
            operation_id=operation_id,
        )

    def hold_recovery_fence(
        self,
        reason: str,
        recovery_correlation: str,
        *,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        return self._global_stop(
            reason,
            mode=pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE,
            recovery_correlation=recovery_correlation,
            operation_id=operation_id,
        )

    def release_recovery_fence(
        self,
        reason: str,
        recovery_correlation: str,
        recovery_fence_token: bytes,
        *,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        if not isinstance(recovery_fence_token, bytes) or len(
            recovery_fence_token
        ) != 32:
            raise MotionGuardUsageError(
                "recovery_fence_token must be exactly 32 bytes"
            )
        return self._global_stop(
            reason,
            mode=pb.GLOBAL_STOP_MODE_RELEASE_RECOVERY_FENCE,
            recovery_correlation=recovery_correlation,
            recovery_fence_token=recovery_fence_token,
            operation_id=operation_id,
        )

    def _global_stop(
        self,
        reason: str,
        *,
        mode: int,
        recovery_correlation: str = "",
        recovery_fence_token: bytes = b"",
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        reason_value = self._require_text(reason, "reason")
        correlation_value = (
            self._require_text(recovery_correlation, "recovery_correlation")
            if mode != pb.GLOBAL_STOP_MODE_ORDINARY
            else ""
        )
        operation = self._operation_id_value(operation_id, "stop-all-motion")
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._new_id("request"),
            stop_all_motion=pb.StopAllMotionRequest(
                envelope=self._new_global_stop_envelope(operation),
                reason=reason_value,
                mode=mode,
                recovery_correlation=correlation_value,
                recovery_fence_token=recovery_fence_token,
            ),
        )
        seal_request(request)
        outcome, result = self._exchange(
            request,
            expected_outcome="stop_all_motion",
            expected_acks={pb.ACK_CODE_ZEROED, pb.ACK_CODE_RESOURCE_FENCED},
        )
        lease_revoked = bool(outcome.lease_revoked)
        expected_ack = (
            pb.ACK_CODE_RESOURCE_FENCED if lease_revoked else pb.ACK_CODE_ZEROED
        )
        if result.ack != expected_ack:
            self._raise_protocol(
                "StopAllMotion ack does not match lease_revoked correlation"
            )
        if outcome.motion_stop_generation == 0:
            self._raise_protocol("StopAllMotion returned no stop generation proof")
        token = bytes(outcome.recovery_fence_token)
        held = bool(outcome.recovery_fence_held)
        if mode == pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE and (
            not held or len(token) != 32
        ):
            self._raise_protocol("recovery fence hold returned no durable token")
        if mode == pb.GLOBAL_STOP_MODE_RELEASE_RECOVERY_FENCE and (held or token):
            self._raise_protocol("recovery fence release did not clear the fence")
        try:
            state = self.get_guard_state(pb.GUARD_SOURCE_GATEWAY_MANUAL)
        except MotionGuardTransportError as exc:
            raise MotionGuardTransportError(
                "global stop completed but guard-state proof is uncertain",
                operation_id=operation,
                outcome_unknown=True,
            ) from exc
        if state.motion_stop_generation != outcome.motion_stop_generation:
            self._raise_protocol(
                "global stop and guard-state stop generations do not correlate"
            )
        if state.recovery_fence_held != held:
            self._raise_protocol(
                "global stop and guard-state recovery fence state disagree"
            )
        if mode == pb.GLOBAL_STOP_MODE_HOLD_RECOVERY_FENCE:
            if not hmac.compare_digest(
                state.recovery_fence_token_sha256,
                hashlib.sha256(token).digest(),
            ) or not hmac.compare_digest(
                state.recovery_fence_correlation_sha256,
                hashlib.sha256(correlation_value.encode("utf-8")).digest(),
            ):
                self._raise_protocol(
                    "recovery fence token or correlation lacks durable state proof"
                )
        return StopAllMotionConfirmation(
            operation_id=operation,
            result=result,
            zero=self._zero_confirmation(outcome.zero_ack),
            lease_revoked=lease_revoked,
            motion_stop_generation=outcome.motion_stop_generation,
            recovery_fence_token=token,
            recovery_fence_held=held,
            generation=state.generation,
            boot_nonce=state.boot_nonce,
        )

    def _operation_id_value(self, value: Optional[str], label: str) -> str:
        if value is None:
            return self._new_id(label)
        return self._require_text(value, "operation_id")

    @staticmethod
    def _require_text(value: str, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise MotionGuardUsageError(f"{name} must be non-empty")
        return value.strip()

    def _validate_epoch(self, generation: int, boot_nonce: bytes) -> None:
        if generation <= 0 or len(boot_nonce) != _BOOT_NONCE_BYTES:
            self._raise_protocol("response contains an invalid boot epoch tuple")

    @staticmethod
    def _validate_admission_proof(
        proof: MotionAdmissionProof,
    ) -> Tuple[int, bytes, int]:
        if not isinstance(proof, MotionAdmissionProof):
            raise MotionGuardUsageError("proof must be a MotionAdmissionProof")
        if isinstance(proof.generation, bool) or not isinstance(proof.generation, int):
            raise MotionGuardUsageError("proof generation must be a positive integer")
        if proof.generation <= 0:
            raise MotionGuardUsageError("proof generation must be positive")
        if not isinstance(proof.boot_nonce, bytes):
            raise MotionGuardUsageError("proof boot_nonce must be bytes")
        boot_nonce = bytes(proof.boot_nonce)
        if len(boot_nonce) != _BOOT_NONCE_BYTES:
            raise MotionGuardUsageError("proof boot_nonce must be exactly 16 bytes")
        stop_generation = proof.motion_stop_generation
        if (
            isinstance(stop_generation, bool)
            or not isinstance(stop_generation, int)
            or stop_generation < 0
            or stop_generation >= 1 << 64
        ):
            raise MotionGuardUsageError(
                "proof motion_stop_generation must be a non-negative uint64"
            )
        return proof.generation, boot_nonce, stop_generation

    def _zero_confirmation(self, zero_ack: pb.ZeroAck) -> ZeroConfirmation:
        if not zero_ack.zero_ack_id or zero_ack.applied_monotonic_ns <= 0:
            self._raise_protocol("terminal zero acknowledgement is incomplete")
        return ZeroConfirmation(
            zero_ack_id=zero_ack.zero_ack_id,
            applied_monotonic_ns=zero_ack.applied_monotonic_ns,
        )


class MotionGuardSession:
    """One source-owned lease with serialized commands and automatic renewal."""

    def __init__(
        self,
        *,
        client: MotionGuardClient,
        source: int,
        lease_tuple: pb.LeaseTuple,
        renew_interval_seconds: float,
        owner_deadline_monotonic_ns: Optional[int] = None,
    ) -> None:
        if renew_interval_seconds <= 0:
            raise MotionGuardUsageError("renew_interval_seconds must be positive")
        self._client = client
        self.source = source
        self.lease_tuple = pb.LeaseTuple()
        self.lease_tuple.CopyFrom(lease_tuple)
        self.renew_interval_seconds = float(renew_interval_seconds)
        self._lock = threading.RLock()
        self._stop_renewal = threading.Event()
        self._renew_thread: Optional[threading.Thread] = None
        self._sequence = 0
        self._active = True
        self._fenced = False
        self._registered = False
        self._failure: Optional[BaseException] = None
        self._fence_error: Optional[MotionGuardFenceError] = None
        self._zero: Optional[ZeroConfirmation] = None
        self._next_renewal_ns = self._client._now()
        if owner_deadline_monotonic_ns is not None:
            self._schedule_next_renewal(owner_deadline_monotonic_ns, self._next_renewal_ns)

    @property
    def failure(self) -> Optional[BaseException]:
        with self._lock:
            return self._failure

    @property
    def fenced(self) -> bool:
        with self._lock:
            return self._fenced

    def __enter__(self) -> MotionGuardSession:
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        del exc_type, traceback
        try:
            self.close()
        except MotionGuardError as close_error:
            if exc is not None:
                close_error.add_note(f"session context body also raised: {exc!r}")
                raise close_error from exc
            raise
        return False

    def _start_renewal(self) -> None:
        with self._lock:
            if not self._active or self._fenced or self._stop_renewal.is_set():
                return
            thread = threading.Thread(
                target=self._renew_loop,
                name=f"motion-guard-renew-{self.lease_tuple.lease_id}",
                daemon=True,
            )
            self._renew_thread = thread
            thread.start()

    def _renew_loop(self) -> None:
        while not self._stop_renewal.is_set():
            with self._lock:
                delay = max(0, self._next_renewal_ns - self._client._now()) / 1_000_000_000
            if self._stop_renewal.wait(delay):
                return
            with self._lock:
                if not self._active or self._fenced:
                    return
                try:
                    self._renew_if_due_locked()
                except Exception as exc:
                    self._failure = exc
                    self._stop_renewal.set()
                    return

    def _schedule_next_renewal(self, owner_deadline_ns: int, issued_ns: int) -> None:
        if owner_deadline_ns <= 0:
            self._client._raise_protocol("lease response contains no owner deadline")
        # 围栏开始前预留一次既有SDK最坏占用和守卫调度周期；不延长服务端期限。
        latest_safe_renewal = owner_deadline_ns - WATCHDOG_FENCE_LEAD_NS - _RENEW_EXECUTION_RESERVE_NS
        requested_renewal = issued_ns + int(self.renew_interval_seconds * 1_000_000_000)
        self._next_renewal_ns = min(requested_renewal, latest_safe_renewal)

    def _renew_if_due_locked(self) -> None:
        """命令与续租共用序列锁；到期续租在下一次非零动作之前优先提交。"""
        if self._stop_renewal.is_set() or self._client._now() < self._next_renewal_ns:
            return
        self._assert_operational()
        envelope, _operation = self._next_envelope(action=pb.GUARD_ACTION_UNSPECIFIED, label="renew")
        request = pb.MotionGuardRequest(
            api_version=API_VERSION,
            request_id=self._client._new_id("request"),
            renew_lease=pb.RenewLeaseRequest(envelope=envelope),
        )
        seal_request(request)
        try:
            outcome, _result = self._client._exchange(
                request, expected_outcome="renew_lease", expected_acks={pb.ACK_CODE_RECEIVED},
            )
            self._schedule_next_renewal(outcome.owner_deadline_monotonic_ns, envelope.issued_monotonic_ns)
        except Exception as exc:
            self._failure = exc
            self._stop_renewal.set()
            raise

    def _next_envelope(
        self,
        *,
        action: int,
        label: str,
        operation_id: Optional[str] = None,
    ) -> Tuple[pb.RequestEnvelope, str]:
        self._sequence += 1
        operation = self._client._operation_id_value(operation_id, label)
        return (
            self._client._new_envelope(
                lease_tuple=self.lease_tuple,
                source=self.source,
                action=action,
                operation_id=operation,
                sequence=self._sequence,
            ),
            operation,
        )

    def _assert_operational(self) -> None:
        if self._failure is not None:
            raise MotionGuardSessionClosedError(
                "motion guard session is fail-closed after a renewal/command failure"
            ) from self._failure
        if not self._active or self._fenced:
            raise MotionGuardSessionClosedError("motion guard session is closed")

    def _submit_motion(
        self,
        *,
        action: int,
        field: str,
        command: Message,
        expected_acks: Collection[int],
        operation_id: Optional[str],
    ) -> CommandConfirmation:
        with self._lock:
            self._assert_operational()
            if action != pb.GUARD_ACTION_STOP:
                self._renew_if_due_locked()
            envelope, operation = self._next_envelope(
                action=action,
                label=field,
                operation_id=operation_id,
            )
            body = pb.SubmitMotionCommandRequest(envelope=envelope)
            getattr(body, field).CopyFrom(command)
            request = pb.MotionGuardRequest(
                api_version=API_VERSION,
                request_id=self._client._new_id("request"),
                submit_motion_command=body,
            )
            seal_request(request)
            try:
                outcome, result = self._client._exchange(
                    request,
                    expected_outcome="submit_motion_command",
                    expected_acks=expected_acks,
                )
            except (MotionGuardTransportError, MotionGuardProtocolError) as exc:
                self._failure = exc
                raise
            zero = None
            if pb.ACK_CODE_ZEROED in expected_acks:
                zero = self._client._zero_confirmation(outcome.zero_ack)
            return CommandConfirmation(
                operation_id=operation,
                result=result,
                applied_monotonic_ns=outcome.applied_monotonic_ns,
                zero=zero,
            )

    def twist(
        self,
        linear_x_mps: float,
        linear_y_mps: float = 0.0,
        angular_z_rps: float = 0.0,
        *,
        operation_id: Optional[str] = None,
    ) -> CommandConfirmation:
        return self._submit_motion(
            action=pb.GUARD_ACTION_TWIST,
            field="twist",
            command=pb.TwistCommand(
                linear_x_mps=float(linear_x_mps),
                linear_y_mps=float(linear_y_mps),
                angular_z_rps=float(angular_z_rps),
            ),
            expected_acks={pb.ACK_CODE_APPLIED},
            operation_id=operation_id,
        )

    def stop(self, *, operation_id: Optional[str] = None) -> CommandConfirmation:
        return self._submit_motion(
            action=pb.GUARD_ACTION_STOP,
            field="stop",
            command=pb.StopCommand(),
            expected_acks={pb.ACK_CODE_ZEROED},
            operation_id=operation_id,
        )

    def set_body_pose(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        *,
        operation_id: Optional[str] = None,
    ) -> CommandConfirmation:
        return self._submit_motion(
            action=pb.GUARD_ACTION_BODY_POSE,
            field="body_pose",
            command=pb.BodyPoseCommand(
                roll_rad=float(roll_rad),
                pitch_rad=float(pitch_rad),
                yaw_rad=float(yaw_rad),
            ),
            expected_acks={pb.ACK_CODE_APPLIED},
            operation_id=operation_id,
        )

    body_pose = set_body_pose

    def set_speed_level(
        self,
        level: int,
        *,
        operation_id: Optional[str] = None,
    ) -> CommandConfirmation:
        if isinstance(level, bool) or not isinstance(level, int) or not 1 <= level <= 3:
            raise MotionGuardUsageError("speed level must be an integer in [1, 3]")
        return self._submit_motion(
            action=pb.GUARD_ACTION_SPEED_LEVEL,
            field="speed_level",
            command=pb.SpeedLevelCommand(level=level),
            expected_acks={pb.ACK_CODE_APPLIED},
            operation_id=operation_id,
        )

    speed_level = set_speed_level

    def set_obstacle_policy(
        self,
        enabled: bool,
        *,
        operation_id: Optional[str] = None,
    ) -> CommandConfirmation:
        if not isinstance(enabled, bool):
            raise MotionGuardUsageError("obstacle policy enabled must be a boolean")
        return self._submit_motion(
            action=pb.GUARD_ACTION_OBSTACLE_POLICY,
            field="obstacle_policy",
            command=pb.ObstaclePolicyCommand(enabled=enabled),
            expected_acks={pb.ACK_CODE_APPLIED},
            operation_id=operation_id,
        )

    obstacle_policy = set_obstacle_policy

    def execute_sport(
        self,
        action: int,
        *,
        enabled: bool = True,
        operation_id: Optional[str] = None,
    ) -> CommandConfirmation:
        if isinstance(action, bool) or not isinstance(action, int):
            raise MotionGuardUsageError("SportAction must be an integer enum value")
        try:
            pb.SportAction.Name(action)
        except ValueError as exc:
            raise MotionGuardUsageError(f"unknown SportAction value {action}") from exc
        if action == pb.SPORT_ACTION_UNSPECIFIED:
            raise MotionGuardUsageError("SPORT_ACTION_UNSPECIFIED is not usable")
        if not isinstance(enabled, bool):
            raise MotionGuardUsageError("sport action enabled must be a boolean")
        with self._lock:
            self._assert_operational()
            if action != pb.SPORT_ACTION_STOP_MOVE:
                self._renew_if_due_locked()
            envelope, operation = self._next_envelope(
                action=pb.GUARD_ACTION_SPORT_ACTION,
                label="sport",
                operation_id=operation_id,
            )
            request = pb.MotionGuardRequest(
                api_version=API_VERSION,
                request_id=self._client._new_id("request"),
                execute_sport_command=pb.ExecuteSportCommandRequest(
                    envelope=envelope,
                    command=pb.SportActionCommand(action=action, enabled=enabled),
                ),
            )
            seal_request(request)
            expected = (
                {pb.ACK_CODE_ZEROED}
                if action == pb.SPORT_ACTION_STOP_MOVE
                else {pb.ACK_CODE_APPLIED}
            )
            try:
                outcome, result = self._client._exchange(
                    request,
                    expected_outcome="execute_sport_command",
                    expected_acks=expected,
                )
            except (MotionGuardTransportError, MotionGuardProtocolError) as exc:
                self._failure = exc
                raise
            zero = (
                self._client._zero_confirmation(outcome.zero_ack)
                if action == pb.SPORT_ACTION_STOP_MOVE
                else None
            )
            return CommandConfirmation(
                operation_id=operation,
                result=result,
                applied_monotonic_ns=outcome.applied_monotonic_ns,
                zero=zero,
            )

    sport = execute_sport

    def switch_mode(
        self,
        target_mode: int,
        *,
        operation_id: Optional[str] = None,
    ) -> ModeSwitchConfirmation:
        if isinstance(target_mode, bool) or not isinstance(target_mode, int):
            raise MotionGuardUsageError("ControlMode must be an integer enum value")
        try:
            pb.ControlMode.Name(target_mode)
        except ValueError as exc:
            raise MotionGuardUsageError(f"unknown ControlMode value {target_mode}") from exc
        if target_mode == pb.CONTROL_MODE_UNSPECIFIED:
            raise MotionGuardUsageError("CONTROL_MODE_UNSPECIFIED is not usable")
        self._stop_and_join_renewal()
        with self._lock:
            self._assert_operational()
            envelope, operation = self._next_envelope(
                action=pb.GUARD_ACTION_MODE_SWITCH,
                label="mode-switch",
                operation_id=operation_id,
            )
            request = pb.MotionGuardRequest(
                api_version=API_VERSION,
                request_id=self._client._new_id("request"),
                revoke_zero_and_switch_mode=pb.RevokeZeroAndSwitchModeRequest(
                    envelope=envelope,
                    command=pb.ModeSwitchCommand(target_mode=target_mode),
                ),
            )
            seal_request(request)
            try:
                outcome, result = self._client._exchange(
                    request,
                    expected_outcome="revoke_zero_and_switch_mode",
                    expected_acks={pb.ACK_CODE_RESOURCE_FENCED},
                )
                zero = self._client._zero_confirmation(outcome.zero_ack)
            except MotionGuardError as exc:
                self._failure = exc
                raise MotionGuardFenceError(
                    "mode switch did not return a verified zero/fence acknowledgement",
                    cause=exc,
                ) from exc
            if outcome.active_mode != target_mode:
                error = MotionGuardProtocolError("mode switch response returned another mode")
                self._failure = error
                raise MotionGuardFenceError(
                    "mode switch response failed correlation",
                    cause=error,
                    zero=zero,
                ) from error
            self._mark_fenced(zero)
            return ModeSwitchConfirmation(
                operation_id=operation,
                result=result,
                zero=zero,
                active_mode=outcome.active_mode,
            )

    mode = switch_mode

    def revoke_and_zero(
        self,
        *,
        operation_id: Optional[str] = None,
    ) -> ZeroConfirmation:
        self._stop_and_join_renewal()
        with self._lock:
            if self._fenced and self._zero is not None:
                return self._zero
            if self._fence_error is not None:
                raise self._fence_error
            prior_failure = self._failure
            if not self._active:
                raise MotionGuardSessionClosedError(
                    "session closed without a verified zero acknowledgement"
                ) from prior_failure
            envelope, _operation = self._next_envelope(
                action=pb.GUARD_ACTION_STOP,
                label="revoke-zero",
                operation_id=operation_id,
            )
            request = pb.MotionGuardRequest(
                api_version=API_VERSION,
                request_id=self._client._new_id("request"),
                revoke_and_zero=pb.RevokeAndZeroRequest(envelope=envelope),
            )
            seal_request(request)
            try:
                outcome, _result = self._client._exchange(
                    request,
                    expected_outcome="revoke_and_zero",
                    expected_acks={pb.ACK_CODE_RESOURCE_FENCED},
                )
                zero = self._client._zero_confirmation(outcome.zero_ack)
            except MotionGuardError as exc:
                self._active = False
                fence_error = MotionGuardFenceError(
                    "RevokeAndZero did not return a verified zero/fence acknowledgement",
                    cause=exc,
                    prior_failure=prior_failure,
                )
                self._fence_error = fence_error
                self._failure = prior_failure or exc
                self._unregister()
                raise fence_error from exc
            self._mark_fenced(zero)
            if prior_failure is not None:
                error = MotionGuardFenceError(
                    "session fenced, but an earlier renewal/command failure remains observable",
                    cause=prior_failure,
                    zero=zero,
                )
                self._failure = error
                raise error from prior_failure
            return zero

    def close(self) -> ZeroConfirmation:
        return self.revoke_and_zero()

    def _stop_and_join_renewal(self) -> None:
        self._stop_renewal.set()
        thread = self._renew_thread
        if thread is None or thread is threading.current_thread():
            return
        thread.join(timeout=self._client.timeout_seconds + 1.0)
        if thread.is_alive():
            error = MotionGuardTransportError(
                "lease renewal thread did not stop before fencing"
            )
            self._client._poison(error)
            thread.join(timeout=self._client.timeout_seconds + 1.0)
            fence_error = MotionGuardFenceError(
                "cannot fence while lease renewal ordering is unresolved",
                cause=error,
            )
            self._fence_error = fence_error
            raise fence_error from error

    def _mark_fenced(self, zero: ZeroConfirmation) -> None:
        self._zero = zero
        self._fenced = True
        self._active = False
        self._unregister()

    def _unregister(self) -> None:
        if self._registered:
            self._registered = False
            self._client._unregister_session(self)

    def _fence_without_registration(self) -> None:
        self.revoke_and_zero()


__all__ = [
    "ActionNotAllowedError",
    "CommandConfirmation",
    "DeadlineExceededError",
    "EpochDurabilityLostError",
    "GuardState",
    "GuardResultSnapshot",
    "InvalidArgumentError",
    "ModeSwitchConfirmation",
    "MotionAdmissionProof",
    "MotionGuardClient",
    "MotionGuardError",
    "MotionGuardFenceError",
    "MotionGuardInternalError",
    "MotionGuardNotReadyError",
    "MotionGuardProtocolError",
    "MotionGuardRemoteError",
    "MotionGuardSession",
    "MotionGuardSessionClosedError",
    "MotionGuardTimeoutError",
    "MotionGuardTransportError",
    "MotionGuardUsageError",
    "MotionSinkNotReadyError",
    "OperationStatus",
    "OutcomeUnknownError",
    "PermissionDeniedError",
    "ReadinessStatus",
    "RecoveryFencedError",
    "RequestIdConflictError",
    "ResourceBusyError",
    "ResourceExhaustedError",
    "StopAllMotionConfirmation",
    "StaleEpochError",
    "StaleSequenceError",
    "StaleStopGenerationError",
    "UnsupportedApiVersionError",
    "ZeroConfirmation",
    "normalize_source",
]
