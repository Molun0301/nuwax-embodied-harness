"""运动守卫的逐操作客户端。"""
from __future__ import annotations

from dataclasses import dataclass
import enum
import socket
import threading
from typing import Optional

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError

from services.navigation.proto_gen import motion_guard_pb2 as pb
from services.navigation.motion_safety_guard.framing import FramingError, recv_frame, send_frame


class MotionErrorCode(str, enum.Enum):
    STALE_GENERATION = "stale_generation"
    LEASE_REVOKED = "lease_revoked"
    NOT_READY = "not_ready"
    ACTUATOR_FAILED = "actuator_failed"
    INVALID_ARGUMENT = "invalid_argument"
    UNAVAILABLE = "unavailable"


class MotionError(RuntimeError):
    def __init__(self, code: MotionErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class MotionSource(str, enum.Enum):
    NAV = "nav"
    GATEWAY_FOLLOW = "gateway_follow"
    GATEWAY_RELATIVE = "gateway_relative"


@dataclass(frozen=True)
class MeasuredMotion:
    source_id: str
    source_timestamp_ns: int
    received_monotonic_ns: int
    valid_until_monotonic_ns: int
    frame_id: str
    linear_xyz: tuple[float, float, float]
    angular_xyz: tuple[float, float, float]
    orientation_wxyz: Optional[tuple[float, float, float, float]]


@dataclass(frozen=True)
class GuardStatus:
    generation: int
    lease_id: int
    lease_source: Optional[MotionSource]
    zeroed: bool
    ready: bool
    reason: str
    command_limits: Optional[dict]


_SOURCES = {MotionSource.NAV: pb.SOURCE_NAV, MotionSource.GATEWAY_FOLLOW: pb.SOURCE_GATEWAY_FOLLOW,
            MotionSource.GATEWAY_RELATIVE: pb.SOURCE_GATEWAY_RELATIVE}
_ERRORS = {pb.ERROR_CODE_STALE_GENERATION: MotionErrorCode.STALE_GENERATION,
           pb.ERROR_CODE_LEASE_REVOKED: MotionErrorCode.LEASE_REVOKED,
           pb.ERROR_CODE_NOT_READY: MotionErrorCode.NOT_READY,
           pb.ERROR_CODE_ACTUATOR_FAILED: MotionErrorCode.ACTUATOR_FAILED,
           pb.ERROR_CODE_INVALID_ARGUMENT: MotionErrorCode.INVALID_ARGUMENT}


class MotionClient:
    def __init__(self, socket_path: str) -> None:
        self._socket_path = socket_path
        self._socket = None
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            if self._socket is not None:
                self._socket.close()
                self._socket = None

    def _exchange(self, connection, request):
        send_frame(connection, request.SerializeToString())
        response = pb.GuardResponse.FromString(recv_frame(connection))
        if response.HasField("error"):
            raise MotionError(_ERRORS.get(response.error.code, MotionErrorCode.UNAVAILABLE), response.error.message)
        if response.WhichOneof("result") != request.WhichOneof("op"):
            raise MotionError(MotionErrorCode.UNAVAILABLE, "运动守卫应答操作不匹配")
        return response

    def _request(self, request, *, independent=False):
        if independent:
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                    connection.connect(self._socket_path)
                    return self._exchange(connection, request)
            except (OSError, FramingError, DecodeError) as exc:
                raise MotionError(MotionErrorCode.UNAVAILABLE, str(exc)) from exc
        with self._lock:
            try:
                if self._socket is None:
                    self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    self._socket.connect(self._socket_path)
                return self._exchange(self._socket, request)
            except (OSError, FramingError, DecodeError) as exc:
                self._socket.close()
                self._socket = None
                raise MotionError(MotionErrorCode.UNAVAILABLE, str(exc)) from exc

    def begin(self) -> int:
        return self._request(pb.GuardRequest(begin=pb.BeginRequest())).begin.generation

    def acquire(self, source: MotionSource, generation: int) -> int:
        return self._request(pb.GuardRequest(acquire=pb.AcquireRequest(source=_SOURCES[source], generation=generation))).acquire.lease_id

    def velocity(self, lease_id: int, vx: float, vy: float, wz: float) -> tuple[bool, Optional[MeasuredMotion]]:
        response = self._request(pb.GuardRequest(velocity=pb.VelocityRequest(lease_id=lease_id, vx_mps=vx, vy_mps=vy, wz_rps=wz)))
        measured = None
        if response.HasField("measured_motion"):
            value = response.measured_motion
            if len(value.body_linear_xyz) != 3 or len(value.body_angular_xyz) != 3 or len(value.orientation_wxyz) not in {0, 4}:
                raise MotionError(MotionErrorCode.UNAVAILABLE, "运动反馈向量长度无效")
            measured = MeasuredMotion(value.source_id, value.source_timestamp_ns, value.received_monotonic_ns,
                value.valid_until_monotonic_ns, value.frame_id, tuple(value.body_linear_xyz), tuple(value.body_angular_xyz),
                tuple(value.orientation_wxyz) if value.orientation_wxyz else None)
        return response.velocity.limited, measured

    def release(self, lease_id: int) -> None:
        try:
            self._request(pb.GuardRequest(release=pb.ReleaseRequest(lease_id=lease_id)))
        except MotionError as exc:
            if exc.code != MotionErrorCode.LEASE_REVOKED:
                raise

    def stop(self, reason: str) -> int:
        return self._request(pb.GuardRequest(stop=pb.StopRequest(reason=reason)), independent=True).stop.generation

    def sport(self, *, action: Optional[str] = None, enabled: bool = True,
              body_pose: Optional[tuple[float, float, float]] = None, control_mode: Optional[str] = None) -> int:
        request = pb.SportRequest()
        if action is not None:
            try:
                name = {"dance1": "DANCE_1", "dance2": "DANCE_2"}.get(action, action.upper())
                value = pb.SportAction.Value("SPORT_ACTION_" + name)
            except ValueError as exc:
                raise MotionError(MotionErrorCode.INVALID_ARGUMENT, str(exc)) from exc
            request.action.CopyFrom(pb.SportActionCommand(action=value, enabled=enabled))
        elif body_pose is not None:
            request.body_pose.CopyFrom(pb.BodyPoseCommand(roll_rad=body_pose[0], pitch_rad=body_pose[1], yaw_rad=body_pose[2]))
        elif control_mode is not None:
            request.control_mode = {"high": pb.CONTROL_MODE_HIGH_LEVEL, "low": pb.CONTROL_MODE_LOW_LEVEL}.get(control_mode, pb.CONTROL_MODE_UNSPECIFIED)
        return self._request(pb.GuardRequest(sport=request)).sport.generation

    def setting(self, *, speed_level: Optional[int] = None, obstacle_avoidance: Optional[bool] = None) -> None:
        request = pb.SettingRequest()
        if speed_level is not None:
            request.speed_level = speed_level
        if obstacle_avoidance is not None:
            request.obstacle_avoidance = obstacle_avoidance
        self._request(pb.GuardRequest(setting=request))

    def status(self) -> GuardStatus:
        value = self._request(pb.GuardRequest(status=pb.StatusRequest())).status
        limits = MessageToDict(value.command_limits, preserving_proto_field_name=True,
                               always_print_fields_with_no_presence=True) if value.HasField("command_limits") else None
        return GuardStatus(value.generation, value.lease_id, {v: k for k, v in _SOURCES.items()}.get(value.lease_source),
                           value.zeroed, value.ready, value.reason, limits)

    def lease(self, source: MotionSource, generation: int) -> "MotionLease":
        return MotionLease(self, self.acquire(source, generation))


class MotionLease:
    def __init__(self, client: MotionClient, lease_id: int) -> None:
        self._client = client
        self.lease_id = lease_id

    def velocity(self, vx: float, vy: float, wz: float) -> tuple[bool, Optional[MeasuredMotion]]:
        return self._client.velocity(self.lease_id, vx, vy, wz)

    def __enter__(self) -> "MotionLease":
        return self

    def __exit__(self, *exc) -> None:
        self._client.release(self.lease_id)
