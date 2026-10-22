from __future__ import annotations

from dataclasses import dataclass
import os
import threading
import uuid
from typing import Callable, Dict, Optional, Union

from core.errors import MotionGuardFacadeError
from core.config_values import runtime_value
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
    """受理时从同一守卫快照捕获、随请求透传的凭据。"""

    request_id: str
    capability_name: str
    boot_nonce: bytes
    motion_stop_generation: int

    @property
    def proof(self) -> MotionAdmissionProof:
        return MotionAdmissionProof(boot_nonce=self.boot_nonce,
                                    motion_stop_generation=self.motion_stop_generation)

    def as_metadata(self) -> Dict[str, object]:
        return {"request_id": self.request_id, "capability_name": self.capability_name,
                "boot_nonce_hex": self.boot_nonce.hex(),
                "motion_stop_generation": self.motion_stop_generation}


class GatewayMotionGuard:
    """Gateway-owned facade over one persistent, fail-closed guard client."""

    def __init__(
        self,
        *,
        socket_path: Optional[Union[str, os.PathLike[str]]] = None,
        client_factory: Callable[..., MotionGuardClient] = MotionGuardClient,
    ) -> None:
        # 与守卫和导航进程共用配置；显式注入仍优先，地址随本实例固定。
        self.socket_path = os.fspath(
            socket_path if socket_path is not None else runtime_value(
                "NUWAX_MOTION_GUARD_SOCKET", DEFAULT_SOCKET_PATH,
            )
        )
        self._client_factory = client_factory
        self._lock = threading.RLock()
        self._client: Optional[MotionGuardClient] = None
        self._last_error: Optional[BaseException] = None
        self._has_connected = False

    @property
    def has_connected(self) -> bool:
        """本实例是否曾建立运动连接；断连不能抹掉已有运动责任。"""

        with self._lock:
            return self._has_connected

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
            self._has_connected = True
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
        """读取守卫当前状态，不据此预判运动是否可执行。"""

        client = self._require_client()
        try:
            return client.get_guard_state(pb.GUARD_SOURCE_GATEWAY_MANUAL)
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardUnavailable(
                f"motion guard state query failed: {exc}"
            ) from exc

    def admission_snapshot(self) -> MotionAdmissionProof:
        """从同一次状态回包捕获启动身份和唯一停止代次。"""
        state = self.guard_state()
        return MotionAdmissionProof(boot_nonce=state.boot_nonce,
                                    motion_stop_generation=state.motion_stop_generation)

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
        # 终结会话直接使用原子撤租归零；先Stop并等待会持锁阻断续租，
        # 还会在租约已过期后重复停车。close自身核对物理停止回执。
        return session.close()

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
        self, reason: str, *, operation_id: Optional[str] = None,
    ) -> StopAllMotionConfirmation:
        operation = (operation_id.strip() if isinstance(operation_id, str) and operation_id.strip()
                     else f"stop-all-motion:{uuid.uuid4()}")
        client = self._require_client()
        try:
            return client.stop_all_motion(reason, operation_id=operation)
        except MotionGuardTransportError as exc:
            self._record_error(exc)
            if not exc.outcome_unknown or (exc.operation_id and exc.operation_id != operation):
                raise GatewayMotionGuardError("全局停车未取得归零回执。") from exc
            try:
                self.start()
                recovery_client = self._require_client()
                status = recovery_client.get_operation_result(
                    pb.GUARD_SOURCE_GATEWAY_MANUAL, operation,
                    operation_namespace=pb.OPERATION_NAMESPACE_GLOBAL_STOP,
                )
                if not status.resolved:
                    return recovery_client.stop_all_motion(reason, operation_id=operation)
                zero = recovery_client.wait_for_stationary(
                    pb.GUARD_SOURCE_GATEWAY_MANUAL, operation, status.zero,
                    operation_namespace=pb.OPERATION_NAMESPACE_GLOBAL_STOP,
                )
                return StopAllMotionConfirmation(
                    operation_id=operation, result=status.result, zero=zero,
                    lease_revoked=status.target_ack == pb.ACK_CODE_RESOURCE_FENCED,
                    motion_stop_generation=status.motion_stop_generation,
                )
            except MotionGuardError as recovery_error:
                self._record_error(recovery_error)
                raise GatewayMotionGuardError("全局停车结果恢复失败。") from recovery_error
        except MotionGuardError as exc:
            self._record_error(exc)
            raise GatewayMotionGuardError("全局停车未取得归零回执。") from exc




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
