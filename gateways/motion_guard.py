from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import os
import threading
import uuid
from typing import Callable, Dict, Optional, Union

from core.errors import MotionGuardFacadeError
from services.navigation.motion_guard_client import (
    CommandConfirmation,
    GuardState,
    ModeSwitchConfirmation,
    MotionAdmissionProof,
    MotionGuardClient,
    MotionGuardError,
    MotionGuardSession,
    MotionGuardTransportError,
    ReadinessStatus,
    StopAllMotionConfirmation,
    ZeroConfirmation,
)
from services.navigation.motion_safety_guard.constants import DEFAULT_SOCKET_PATH
from services.navigation.proto_gen import motion_guard_pb2 as pb


class GatewayMotionGuardError(MotionGuardFacadeError):
    pass


class GatewayMotionGuardUnavailable(GatewayMotionGuardError):
    pass


@dataclass(frozen=True)
class GatewayCommandConfirmation:
    command: CommandConfirmation
    zero: ZeroConfirmation


@dataclass(frozen=True)
class MotionAdmission:
    """Request-scoped proof created by the sole public readiness check."""

    request_id: str
    capability_name: str
    generation: int
    boot_nonce: bytes
    motion_stop_generation: int
    admitted_monotonic_ns: int
    proof: MotionAdmissionProof
    guard_motion_stop_generation: Optional[int] = None

    def __post_init__(self) -> None:
        if not self.request_id or not self.capability_name:
            raise ValueError("motion admission identity must be non-empty")
        if self.generation <= 0 or len(self.boot_nonce) != 16:
            raise ValueError("motion admission epoch is invalid")
        if self.motion_stop_generation < 0 or self.admitted_monotonic_ns <= 0:
            raise ValueError("motion admission local generation is invalid")
        guard_stop_generation = self.guard_motion_stop_generation
        if guard_stop_generation is None:
            guard_stop_generation = self.proof.motion_stop_generation
            object.__setattr__(
                self,
                "guard_motion_stop_generation",
                guard_stop_generation,
            )
        if (
            guard_stop_generation < 0
            or guard_stop_generation != self.proof.motion_stop_generation
        ):
            raise ValueError("motion admission guard generation is invalid")
        if (
            self.proof.generation != self.generation
            or bytes(self.proof.boot_nonce) != bytes(self.boot_nonce)
        ):
            raise ValueError("motion admission proof does not match its epoch")

    def as_metadata(self) -> Dict[str, object]:
        guard_stop_generation = self.guard_motion_stop_generation
        assert guard_stop_generation is not None
        return {
            "request_id": self.request_id,
            "capability_name": self.capability_name,
            "generation": self.generation,
            "boot_nonce_hex": bytes(self.boot_nonce).hex(),
            "motion_stop_generation": guard_stop_generation,
            "admitted_monotonic_ns": self.admitted_monotonic_ns,
        }


class GatewayMotionGuard:
    """Gateway-owned facade over one persistent, fail-closed guard client."""

    def __init__(
        self,
        *,
        socket_path: Union[str, os.PathLike[str]] = DEFAULT_SOCKET_PATH,
        client_factory: Callable[..., MotionGuardClient] = MotionGuardClient,
    ) -> None:
        self.socket_path = os.fspath(socket_path)
        self._client_factory = client_factory
        self._lock = threading.RLock()
        self._client: Optional[MotionGuardClient] = None
        self._last_error: Optional[BaseException] = None

    @property
    def started(self) -> bool:
        with self._lock:
            return self._client is not None and not self._client.failed

    @property
    def last_error(self) -> Optional[BaseException]:
        with self._lock:
            return self._last_error

    def start(self) -> None:
        """Open the persistent transport without querying motion readiness."""

        with self._lock:
            current = self._client
            if current is not None and not current.failed:
                return
            self._client = None
            if current is not None:
                try:
                    current.close()
                except MotionGuardError:
                    pass
            client: Optional[MotionGuardClient] = None
            try:
                client = self._client_factory(self.socket_path)
            except MotionGuardError as exc:
                if client is not None:
                    try:
                        client.close()
                    except MotionGuardError:
                        pass
                self._last_error = exc
                raise GatewayMotionGuardUnavailable(
                    f"motion guard is unavailable at {self.socket_path}: {exc}"
                ) from exc
            self._client = client
            self._last_error = None

    connect = start

    def close(self) -> None:
        with self._lock:
            client, self._client = self._client, None
        if client is None:
            return
        try:
            client.close()
        except MotionGuardError as exc:
            with self._lock:
                self._last_error = exc
            raise GatewayMotionGuardError(
                "motion guard client closed without a verified fence"
            ) from exc

    def readiness(self, source: int) -> ReadinessStatus:
        client = self._require_client()
        try:
            return client.get_readiness(source)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardUnavailable(
                f"motion guard readiness failed: {exc}"
            ) from exc

    def guard_state(self) -> GuardState:
        """Return the durable Gateway-manual guard state for recovery routing."""

        client = self._require_client()
        try:
            return client.get_guard_state(pb.GUARD_SOURCE_GATEWAY_MANUAL)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardUnavailable(
                f"motion guard state query failed: {exc}"
            ) from exc

    def acquire_follow_session(
        self,
        proof: MotionAdmissionProof,
    ) -> MotionGuardSession:
        return self._acquire(pb.GUARD_SOURCE_GATEWAY_FOLLOW, proof)

    def acquire_relative_session(
        self,
        proof: MotionAdmissionProof,
    ) -> MotionGuardSession:
        return self._acquire(pb.GUARD_SOURCE_GATEWAY_RELATIVE, proof)

    def stop_and_fence(self, session: MotionGuardSession) -> ZeroConfirmation:
        stop_error: Optional[Exception] = None
        try:
            session.stop()
        except Exception as exc:
            stop_error = exc
        zero = session.close()
        if stop_error is not None:
            raise stop_error
        return zero

    def execute_sport(
        self,
        action: int,
        *,
        proof: MotionAdmissionProof,
        enabled: bool = True,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_SPORT,
            proof,
            lambda session: session.execute_sport(action, enabled=enabled),
        )

    def set_body_pose(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        *,
        proof: MotionAdmissionProof,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            proof,
            lambda session: session.set_body_pose(roll_rad, pitch_rad, yaw_rad),
        )

    def set_speed_level(
        self,
        level: int,
        *,
        proof: MotionAdmissionProof,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            proof,
            lambda session: session.set_speed_level(level),
        )

    def set_obstacle_policy(
        self,
        enabled: bool,
        *,
        proof: MotionAdmissionProof,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            proof,
            lambda session: session.set_obstacle_policy(enabled),
        )

    def switch_mode(
        self,
        target_mode: int,
        *,
        proof: MotionAdmissionProof,
    ) -> ModeSwitchConfirmation:
        session = self._acquire(pb.GUARD_SOURCE_GATEWAY_SPORT, proof)
        try:
            try:
                return session.switch_mode(target_mode)
            finally:
                if not session.fenced:
                    session.close()
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardError(
                "motion guard mode switch failed without a verified fence"
            ) from exc

    def stop_all_motion(
        self,
        reason: str,
        *,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        return self._global_stop_with_recovery(
            "stop_all_motion",
            reason,
            operation_id=operation_id,
        )

    def hold_recovery_fence(
        self,
        *,
        reason: str,
        correlation_id: str,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        return self._global_stop_with_recovery(
            "hold_recovery_fence",
            reason,
            correlation_id,
            operation_id=operation_id,
        )

    def release_recovery_fence(
        self,
        *,
        reason: str,
        correlation_id: str,
        recovery_fence_token: bytes,
        operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        return self._global_stop_with_recovery(
            "release_recovery_fence",
            reason,
            correlation_id,
            recovery_fence_token,
            operation_id=operation_id,
        )

    def _global_stop_with_recovery(
        self,
        method_name: str,
        *args: object,
        operation_id: Optional[str],
    ) -> StopAllMotionConfirmation:
        operation = (
            operation_id.strip()
            if isinstance(operation_id, str) and operation_id.strip()
            else f"{method_name.replace('_', '-')}:{uuid.uuid4()}"
        )
        client = self._require_client()
        try:
            method = getattr(client, method_name)
            return method(*args, operation_id=operation)
        except MotionGuardTransportError as exc:
            self._record_error(exc)
            if not exc.outcome_unknown or (
                exc.operation_id and exc.operation_id != operation
            ):
                raise GatewayMotionGuardError(
                    "global motion stop did not return a verified zero acknowledgement"
                ) from exc
            try:
                self.start()
                recovery_client = self._require_client()
                status = recovery_client.get_operation_result(
                    pb.GUARD_SOURCE_GATEWAY_MANUAL,
                    operation,
                    operation_namespace=pb.OPERATION_NAMESPACE_GLOBAL_STOP,
                )
                if not status.resolved:
                    recovery_method = getattr(recovery_client, method_name)
                    return recovery_method(*args, operation_id=operation)
                if status.zero is None or status.target_ack not in {
                    pb.ACK_CODE_ZEROED,
                    pb.ACK_CODE_RESOURCE_FENCED,
                }:
                    raise MotionGuardError(
                        "resolved global stop has no valid terminal zero result"
                    )
                state = recovery_client.get_guard_state(
                    pb.GUARD_SOURCE_GATEWAY_MANUAL
                )
                if state.motion_stop_generation != status.motion_stop_generation:
                    raise MotionGuardError(
                        "resolved global stop does not match durable guard state"
                    )
                token = status.recovery_fence_token
                held = status.recovery_fence_held
                if state.recovery_fence_held != held:
                    raise MotionGuardError(
                        "resolved global stop recovery state is not durable"
                    )
                if method_name == "hold_recovery_fence":
                    correlation_id = str(args[1]).strip()
                    if (
                        not held
                        or len(token) != 32
                        or not hmac.compare_digest(
                            state.recovery_fence_token_sha256,
                            hashlib.sha256(token).digest(),
                        )
                        or not hmac.compare_digest(
                            state.recovery_fence_correlation_sha256,
                            hashlib.sha256(
                                correlation_id.encode("utf-8")
                            ).digest(),
                        )
                    ):
                        raise MotionGuardError(
                            "resolved recovery fence hold lacks durable state proof"
                        )
                elif method_name == "release_recovery_fence" and (
                    held or state.recovery_fence_held
                ):
                    raise MotionGuardError(
                        "resolved recovery fence release did not clear durable state"
                    )
                return StopAllMotionConfirmation(
                    operation_id=operation,
                    result=status.result,
                    zero=status.zero,
                    lease_revoked=status.target_ack
                    == pb.ACK_CODE_RESOURCE_FENCED,
                    motion_stop_generation=status.motion_stop_generation,
                    recovery_fence_token=status.recovery_fence_token,
                    recovery_fence_held=status.recovery_fence_held,
                    generation=state.generation,
                    boot_nonce=state.boot_nonce,
                )
            except MotionGuardError as recovery_error:
                self._record_error(recovery_error)
                raise GatewayMotionGuardError(
                    "global motion stop outcome could not be recovered"
                ) from recovery_error
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardError(
                "global motion stop did not return a verified zero acknowledgement"
            ) from exc

    def _one_shot(
        self,
        source: int,
        proof: MotionAdmissionProof,
        operation: Callable[[MotionGuardSession], CommandConfirmation],
    ) -> GatewayCommandConfirmation:
        session = self._acquire(source, proof)
        try:
            try:
                command = operation(session)
            finally:
                zero = self.stop_and_fence(session)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardError(
                "motion guard one-shot command failed without a clean fence"
            ) from exc
        return GatewayCommandConfirmation(command=command, zero=zero)

    def _acquire(
        self,
        source: int,
        proof: MotionAdmissionProof,
    ) -> MotionGuardSession:
        client = self._require_client()
        try:
            return client.acquire_session(source, proof)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardUnavailable(
                f"motion guard lease acquisition failed: {exc}"
            ) from exc

    def _require_client(self) -> MotionGuardClient:
        self.start()
        with self._lock:
            client = self._client
        if client is None or client.failed:
            raise GatewayMotionGuardUnavailable(
                "motion guard client is not started or is fail-closed"
            )
        return client

    def _record_error(self, error: BaseException) -> None:
        with self._lock:
            self._last_error = error


__all__ = [
    "GatewayCommandConfirmation",
    "GatewayMotionGuard",
    "GatewayMotionGuardError",
    "GatewayMotionGuardUnavailable",
    "MotionAdmission",
]
