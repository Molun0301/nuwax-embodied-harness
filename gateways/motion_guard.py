from __future__ import annotations

from dataclasses import dataclass
import os
import threading
from typing import Callable, Optional, Union

from core.errors import MotionGuardFacadeError
from services.navigation.motion_guard_client import (
    CommandConfirmation,
    ModeSwitchConfirmation,
    MotionGuardClient,
    MotionGuardError,
    MotionGuardSession,
    ReadinessStatus,
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
            return self._client is not None

    @property
    def last_error(self) -> Optional[BaseException]:
        with self._lock:
            return self._last_error

    def start(self) -> ReadinessStatus:
        with self._lock:
            if self._client is not None:
                return self._client.get_readiness(pb.GUARD_SOURCE_GATEWAY_MANUAL)
            client: Optional[MotionGuardClient] = None
            try:
                client = self._client_factory(self.socket_path)
                readiness = client.get_readiness(pb.GUARD_SOURCE_GATEWAY_MANUAL)
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
            return readiness

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

    def is_ready(self, source: int) -> bool:
        try:
            return self.readiness(source).ready
        except GatewayMotionGuardError:
            return False

    def is_reachable(self, source: int) -> bool:
        try:
            self.readiness(source)
            return True
        except GatewayMotionGuardError:
            return False

    def acquire_follow_session(self) -> MotionGuardSession:
        return self._acquire(pb.GUARD_SOURCE_GATEWAY_FOLLOW)

    def acquire_relative_session(self) -> MotionGuardSession:
        return self._acquire(pb.GUARD_SOURCE_GATEWAY_RELATIVE)

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
        enabled: bool = True,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_SPORT,
            lambda session: session.execute_sport(action, enabled=enabled),
        )

    def set_body_pose(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
    ) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            lambda session: session.set_body_pose(roll_rad, pitch_rad, yaw_rad),
        )

    def set_speed_level(self, level: int) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            lambda session: session.set_speed_level(level),
        )

    def set_obstacle_policy(self, enabled: bool) -> GatewayCommandConfirmation:
        return self._one_shot(
            pb.GUARD_SOURCE_GATEWAY_MANUAL,
            lambda session: session.set_obstacle_policy(enabled),
        )

    def switch_mode(self, target_mode: int) -> ModeSwitchConfirmation:
        session = self._acquire(pb.GUARD_SOURCE_GATEWAY_SPORT)
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

    def latch_safety_stop(self, reason: str) -> ZeroConfirmation:
        client = self._require_client()
        try:
            return client.latch_gateway_safety_stop(reason)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardError(
                "gateway safety latch did not return a verified zero acknowledgement"
            ) from exc

    def _one_shot(
        self,
        source: int,
        operation: Callable[[MotionGuardSession], CommandConfirmation],
    ) -> GatewayCommandConfirmation:
        session = self._acquire(source)
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

    def _acquire(self, source: int) -> MotionGuardSession:
        client = self._require_client()
        try:
            return client.acquire_session(source)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardUnavailable(
                f"motion guard lease acquisition failed: {exc}"
            ) from exc

    def _require_client(self) -> MotionGuardClient:
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
]
