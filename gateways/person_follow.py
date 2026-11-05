from __future__ import annotations

from core.config_values import runtime_value

from dataclasses import dataclass, replace
import math
import logging
import queue
import struct
import threading
from typing import Any, Callable, Optional, Tuple

from services.navigation.motion_client import MotionLease


_LOGGER = logging.getLogger(__name__)

PERSON_FOLLOW_TARGET_FINGERPRINT = bytes.fromhex("49b0abee638b1fdb")
PERSON_FOLLOW_COMMAND_FINGERPRINT = bytes.fromhex("76ad78cfd90347e8")

PERSON_FOLLOW_TRACKED = 0
PERSON_FOLLOW_LOST = 1
PERSON_FOLLOW_STOP = 2

DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL = "/person_follow_target"
DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL = "/gateway/person_follow_cmd_vel"

_TARGET_TYPE = "nuwax_msgs.PersonFollowTarget"
_COMMAND_TYPE = "nuwax_msgs.PersonFollowCommand"
_FORBIDDEN_COMMAND_CHANNELS = frozenset({"/cmd_vel", "/nav_cmd_vel", "/nav/cmd_vel"})
PERSON_FOLLOW_WAITING_DATA = 0
PERSON_FOLLOW_TRACKING = 1
PERSON_FOLLOW_COMMAND_LOST = 2
PERSON_FOLLOW_STOPPED = 3
PERSON_FOLLOW_FAILED = 4

_MAX_IDENTIFIER_BYTES = 255
_MAX_QUERY_BYTES = 255
_MAX_REASON_BYTES = 1024


class PersonFollowBridgeError(RuntimeError):
    pass


class PersonFollowBridgeUnavailable(PersonFollowBridgeError):
    pass


@dataclass(frozen=True)
class PersonFollowTarget:
    session_id: str
    query: str
    target_seq: int
    required_cloud_revision: int
    stamp_sec: int
    stamp_nsec: int
    camera_frame: str
    camera_parent_frame: str
    camera_translation_x: float
    camera_translation_y: float
    camera_translation_z: float
    camera_rotation_x: float
    camera_rotation_y: float
    camera_rotation_z: float
    camera_rotation_w: float
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    bbox_x1: float
    bbox_y1: float
    bbox_x2: float
    bbox_y2: float
    tracking_state: int

    def lifecycle(self, tracking_state: int) -> "PersonFollowTarget":
        return replace(self, tracking_state=tracking_state)


@dataclass(frozen=True)
class PersonFollowCommand:
    session_id: str
    target_seq: int
    target_stamp_sec: int
    target_stamp_nsec: int
    command_seq: int
    cloud_revision: int
    state: int
    reason: str
    linear_x: float
    linear_y: float
    angular_z: float

    @property
    def is_exact_zero(self) -> bool:
        return self.linear_x == self.linear_y == self.angular_z == 0.0


@dataclass
class _ControlRequest:
    operation: Callable[[], Any]
    completed: threading.Event
    result: Any = None
    error: Optional[BaseException] = None


def _typed_channel(channel: str, message_type: str) -> str:
    value = str(channel).strip()
    if not value or not value.startswith("/") or "#" in value or any(ch.isspace() for ch in value):
        raise PersonFollowBridgeError(f"invalid LCM base channel {channel!r}")
    return f"{value}#{message_type}"


def _encode_string(
    value: str,
    field: str,
    *,
    max_bytes: int,
    allow_empty: bool = False,
) -> bytes:
    if not isinstance(value, str) or (not value and not allow_empty):
        qualifier = "a string" if allow_empty else "a non-empty string"
        raise PersonFollowBridgeError(f"{field} must be {qualifier}")
    try:
        encoded = value.encode("utf-8", errors="strict")
    except UnicodeError as exc:
        raise PersonFollowBridgeError(f"{field} must be valid UTF-8") from exc
    if len(encoded) > max_bytes or b"\0" in encoded:
        raise PersonFollowBridgeError(f"{field} exceeds its bounded UTF-8 contract")
    length = len(encoded) + 1
    return struct.pack(">I", length) + encoded + b"\0"


class _DecodeCursor:
    def __init__(self, encoded: bytes) -> None:
        self.encoded = encoded
        self.offset = 0

    def read(self, length: int, field: str) -> bytes:
        end = self.offset + length
        if length < 0 or end > len(self.encoded):
            raise PersonFollowBridgeError(f"truncated PersonFollowCommand {field}")
        value = self.encoded[self.offset : end]
        self.offset = end
        return value

    def unpack(self, format_string: str, field: str) -> Tuple[Any, ...]:
        size = struct.calcsize(format_string)
        return struct.unpack(format_string, self.read(size, field))

    def string(
        self,
        field: str,
        *,
        max_bytes: int,
        allow_empty: bool = False,
    ) -> str:
        (length,) = self.unpack(">I", f"{field} length")
        if length < 1 or length > max_bytes + 1:
            raise PersonFollowBridgeError(f"{field} exceeds its bounded UTF-8 contract")
        encoded = self.read(length, field)
        if encoded[-1:] != b"\0" or b"\0" in encoded[:-1]:
            raise PersonFollowBridgeError(f"{field} is not a canonical LCM string")
        try:
            value = encoded[:-1].decode("utf-8", errors="strict")
        except UnicodeError as exc:
            raise PersonFollowBridgeError(f"{field} must be valid UTF-8") from exc
        if not value and not allow_empty:
            raise PersonFollowBridgeError(f"{field} must be non-empty")
        return value


def _validate_target(target: PersonFollowTarget) -> None:
    _encode_string(target.session_id, "session_id", max_bytes=_MAX_IDENTIFIER_BYTES)
    _encode_string(target.query, "query", max_bytes=_MAX_QUERY_BYTES)
    _encode_string(target.camera_frame, "camera_frame", max_bytes=_MAX_IDENTIFIER_BYTES)
    _encode_string(
        target.camera_parent_frame,
        "camera_parent_frame",
        max_bytes=_MAX_IDENTIFIER_BYTES,
    )
    if (
        isinstance(target.target_seq, bool)
        or not isinstance(target.target_seq, int)
        or not 0 < target.target_seq < (1 << 63)
    ):
        raise PersonFollowBridgeError("target_seq must be a positive int64")
    if (
        isinstance(target.required_cloud_revision, bool)
        or not isinstance(target.required_cloud_revision, int)
        or not 0 < target.required_cloud_revision < (1 << 63)
    ):
        raise PersonFollowBridgeError("required_cloud_revision must be a positive int64")
    if not -(1 << 63) <= target.stamp_sec < (1 << 63):
        raise PersonFollowBridgeError("stamp_sec is outside int64")
    if not 0 <= target.stamp_nsec < 1_000_000_000:
        raise PersonFollowBridgeError("stamp_nsec must be in [0, 1e9)")
    if not 0 < target.image_width < (1 << 31) or not 0 < target.image_height < (1 << 31):
        raise PersonFollowBridgeError("image dimensions must be positive int32 values")
    if target.tracking_state not in {
        PERSON_FOLLOW_TRACKED,
        PERSON_FOLLOW_LOST,
        PERSON_FOLLOW_STOP,
    }:
        raise PersonFollowBridgeError("tracking_state is not TRACKED, LOST, or STOP")
    values = (
        target.camera_translation_x,
        target.camera_translation_y,
        target.camera_translation_z,
        target.camera_rotation_x,
        target.camera_rotation_y,
        target.camera_rotation_z,
        target.camera_rotation_w,
        target.fx,
        target.fy,
        target.cx,
        target.cy,
        target.bbox_x1,
        target.bbox_y1,
        target.bbox_x2,
        target.bbox_y2,
    )
    if not all(math.isfinite(value) for value in values):
        raise PersonFollowBridgeError("camera transform, intrinsics, and bbox must be finite")
    if target.fx <= 0.0 or target.fy <= 0.0:
        raise PersonFollowBridgeError("camera focal lengths must be positive")
    if target.tracking_state == PERSON_FOLLOW_TRACKED:
        if not (
            0.0 <= target.bbox_x1 < target.bbox_x2 <= float(target.image_width)
            and 0.0 <= target.bbox_y1 < target.bbox_y2 <= float(target.image_height)
        ):
            raise PersonFollowBridgeError("tracked bbox must be valid xyxy image coordinates")


def encode_person_follow_target(target: PersonFollowTarget) -> bytes:
    _validate_target(target)
    output = bytearray(PERSON_FOLLOW_TARGET_FINGERPRINT)
    output.extend(_encode_string(target.session_id, "session_id", max_bytes=_MAX_IDENTIFIER_BYTES))
    output.extend(_encode_string(target.query, "query", max_bytes=_MAX_QUERY_BYTES))
    output.extend(
        struct.pack(
            ">qqqi",
            target.target_seq,
            target.required_cloud_revision,
            target.stamp_sec,
            target.stamp_nsec,
        )
    )
    output.extend(_encode_string(target.camera_frame, "camera_frame", max_bytes=_MAX_IDENTIFIER_BYTES))
    output.extend(
        _encode_string(
            target.camera_parent_frame,
            "camera_parent_frame",
            max_bytes=_MAX_IDENTIFIER_BYTES,
        )
    )
    output.extend(
        struct.pack(
            ">7d",
            target.camera_translation_x,
            target.camera_translation_y,
            target.camera_translation_z,
            target.camera_rotation_x,
            target.camera_rotation_y,
            target.camera_rotation_z,
            target.camera_rotation_w,
        )
    )
    output.extend(struct.pack(">ii", target.image_width, target.image_height))
    output.extend(
        struct.pack(
            ">8di",
            target.fx,
            target.fy,
            target.cx,
            target.cy,
            target.bbox_x1,
            target.bbox_y1,
            target.bbox_x2,
            target.bbox_y2,
            target.tracking_state,
        )
    )
    return bytes(output)


def decode_person_follow_command(encoded: bytes) -> PersonFollowCommand:
    if not isinstance(encoded, bytes) or len(encoded) < 8:
        raise PersonFollowBridgeError("PersonFollowCommand is truncated")
    cursor = _DecodeCursor(encoded)
    if cursor.read(8, "fingerprint") != PERSON_FOLLOW_COMMAND_FINGERPRINT:
        raise PersonFollowBridgeError("PersonFollowCommand fingerprint mismatch")
    session_id = cursor.string("session_id", max_bytes=_MAX_IDENTIFIER_BYTES)
    (
        target_seq,
        target_stamp_sec,
        target_stamp_nsec,
        command_seq,
        cloud_revision,
        state,
    ) = cursor.unpack(">qqiqqi", "correlation")
    reason = cursor.string("reason", max_bytes=_MAX_REASON_BYTES, allow_empty=True)
    linear_x, linear_y, angular_z = cursor.unpack(">3d", "twist")
    if cursor.offset != len(encoded):
        raise PersonFollowBridgeError("PersonFollowCommand has trailing bytes")
    if not 0 < target_seq < (1 << 63):
        raise PersonFollowBridgeError("target_seq must be a positive int64")
    if not -(1 << 63) <= target_stamp_sec < (1 << 63):
        raise PersonFollowBridgeError("target_stamp_sec is outside int64")
    if not 0 <= target_stamp_nsec < 1_000_000_000:
        raise PersonFollowBridgeError("target_stamp_nsec must be in [0, 1e9)")
    if not 0 < command_seq < (1 << 63):
        raise PersonFollowBridgeError("command_seq must be a positive int64")
    if state not in {
        PERSON_FOLLOW_WAITING_DATA,
        PERSON_FOLLOW_TRACKING,
        PERSON_FOLLOW_COMMAND_LOST,
        PERSON_FOLLOW_STOPPED,
        PERSON_FOLLOW_FAILED,
    }:
        raise PersonFollowBridgeError("PersonFollowCommand state is invalid")
    if not 0 <= cloud_revision < (1 << 63):
        raise PersonFollowBridgeError("cloud_revision must be a non-negative int64")
    if (
        state in {PERSON_FOLLOW_TRACKING, PERSON_FOLLOW_COMMAND_LOST, PERSON_FOLLOW_STOPPED}
        and cloud_revision == 0
    ):
        raise PersonFollowBridgeError("active cloud_revision must be a positive int64")
    if not all(math.isfinite(value) for value in (linear_x, linear_y, angular_z)):
        raise PersonFollowBridgeError("PersonFollowCommand twist must be finite")
    command = PersonFollowCommand(
        session_id=session_id,
        target_seq=target_seq,
        target_stamp_sec=target_stamp_sec,
        target_stamp_nsec=target_stamp_nsec,
        command_seq=command_seq,
        cloud_revision=cloud_revision,
        state=state,
        reason=reason,
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
    )
    if (
        state
        in {
            PERSON_FOLLOW_WAITING_DATA,
            PERSON_FOLLOW_COMMAND_LOST,
            PERSON_FOLLOW_STOPPED,
            PERSON_FOLLOW_FAILED,
        }
        and not command.is_exact_zero
    ):
        raise PersonFollowBridgeError("non-tracking PersonFollowCommand twist must be exactly zero")
    return command


def _source_timestamp_ns(perception_context: Any) -> int:
    for metadata in (
        getattr(perception_context, "metadata", None),
        getattr(getattr(perception_context, "observation", None), "metadata", None),
    ):
        if not isinstance(metadata, dict) or "source_timestamp_ns" not in metadata:
            continue
        value = metadata["source_timestamp_ns"]
        if isinstance(value, bool):
            break
        try:
            timestamp_ns = int(value)
        except (TypeError, ValueError):
            break
        if timestamp_ns > 0:
            return timestamp_ns
        break
    raise PersonFollowBridgeError("perception context is missing an exact positive source_timestamp_ns")


def build_person_follow_target(
    *,
    session_id: str,
    query: str,
    target_seq: int,
    required_cloud_revision: int,
    camera_parent_frame: str,
    camera_translation: Tuple[float, float, float],
    camera_rotation: Tuple[float, float, float, float],
    perception_context: Any,
    detection: Optional[Any],
    tracking_state: int,
) -> PersonFollowTarget:
    camera_info = getattr(perception_context, "camera_info", None)
    if camera_info is None:
        raise PersonFollowBridgeError("person follow requires source CameraInfo")
    source_timestamp_ns = _source_timestamp_ns(perception_context)
    bbox = getattr(detection, "bbox", None) if detection is not None else None
    if tracking_state == PERSON_FOLLOW_TRACKED and bbox is None:
        raise PersonFollowBridgeError("TRACKED person follow target requires a bbox")
    if bbox is None:
        bbox_x1 = bbox_y1 = bbox_x2 = bbox_y2 = 0.0
    else:
        bbox_x1 = float(bbox.x_px)
        bbox_y1 = float(bbox.y_px)
        bbox_x2 = bbox_x1 + float(bbox.width_px)
        bbox_y2 = bbox_y1 + float(bbox.height_px)
    target = PersonFollowTarget(
        session_id=str(session_id).strip(),
        query=str(query).strip(),
        target_seq=int(target_seq),
        required_cloud_revision=int(required_cloud_revision),
        stamp_sec=source_timestamp_ns // 1_000_000_000,
        stamp_nsec=source_timestamp_ns % 1_000_000_000,
        camera_frame=str(camera_info.frame_id),
        camera_parent_frame=str(camera_parent_frame).strip(),
        camera_translation_x=float(camera_translation[0]),
        camera_translation_y=float(camera_translation[1]),
        camera_translation_z=float(camera_translation[2]),
        camera_rotation_x=float(camera_rotation[0]),
        camera_rotation_y=float(camera_rotation[1]),
        camera_rotation_z=float(camera_rotation[2]),
        camera_rotation_w=float(camera_rotation[3]),
        image_width=int(camera_info.width_px),
        image_height=int(camera_info.height_px),
        fx=float(camera_info.fx),
        fy=float(camera_info.fy),
        cx=float(camera_info.cx),
        cy=float(camera_info.cy),
        bbox_x1=bbox_x1,
        bbox_y1=bbox_y1,
        bbox_x2=bbox_x2,
        bbox_y2=bbox_y2,
        tracking_state=int(tracking_state),
    )
    _validate_target(target)
    return target


def _default_lcm_factory(provider: str):
    try:
        import lcm
    except ImportError as exc:
        raise PersonFollowBridgeUnavailable("python3-lcm is required for the PersonFollow gateway bridge") from exc
    return lcm.LCM(provider)


@dataclass
class _Binding:
    session_id: str
    lease: MotionLease
    target: Optional[PersonFollowTarget] = None
    command: Optional[PersonFollowCommand] = None
    error: Optional[BaseException] = None


class GatewayPersonFollowBridge:
    """一个线程拥有 LCM 连接，将原生跟随速度直接写入当前运动租约。"""

    def __init__(self, *, lcm_url: Optional[str] = None, target_channel: Optional[str] = None,
                 command_channel: Optional[str] = None, lcm_factory=_default_lcm_factory,
                 poll_timeout_ms: int = 20) -> None:
        self.lcm_url = lcm_url or runtime_value("NUWAX_LCM_URL") or DEFAULT_LCM_URL
        self.target_channel = target_channel or runtime_value("NUWAX_PERSON_FOLLOW_TARGET_CHANNEL") or DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL
        self.command_channel = command_channel or runtime_value("NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL") or DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL
        if self.command_channel in _FORBIDDEN_COMMAND_CHANNELS:
            raise PersonFollowBridgeError("跟随速度必须使用独立 LCM 通道。")
        self._target_channel = _typed_channel(self.target_channel, _TARGET_TYPE)
        self._command_channel = _typed_channel(self.command_channel, _COMMAND_TYPE)
        self._lcm_factory = lcm_factory
        self._poll_timeout_ms = poll_timeout_ms
        self._requests = queue.Queue()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._closed = False
        self._started = False
        self._failure = None
        self._lcm = None
        self._binding = None

    @property
    def started(self) -> bool:
        with self._lock:
            return self._started

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise PersonFollowBridgeUnavailable("跟随通信桥已关闭。")
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="gateway-person-follow-lcm", daemon=True)
                self._thread.start()
        self._ready.wait()
        with self._lock:
            if self._failure is not None:
                raise self._failure

    def _call(self, operation):
        request = _ControlRequest(operation=operation, completed=threading.Event())
        with self._lock:
            if not self._started or self._closed:
                raise self._failure or PersonFollowBridgeUnavailable("跟随通信桥未运行。")
            self._requests.put(request)
        request.completed.wait()
        if request.error is not None:
            raise request.error
        return request.result

    def bind(self, session_id: str, lease: MotionLease, *, check: Callable[[], None]) -> None:
        def operation():
            # 检查实际排队执行点，避免旧线程暂停后覆盖新任务的绑定。
            check()
            self._bind(session_id, lease)
        self._call(operation)

    def _bind(self, session_id, lease) -> None:
        self._end_current()
        self._binding = _Binding(session_id, lease)

    def publish(self, target: PersonFollowTarget, *, check: Callable[[], None]) -> None:
        def operation():
            check()
            self._publish(target)
        self._call(operation)

    def _publish(self, target) -> None:
        binding = self._matching(target.session_id)
        payload = encode_person_follow_target(target)
        binding.target = target
        self._lcm.publish(self._target_channel, payload)

    def _matching(self, session_id):
        binding = self._binding
        if binding is None or binding.session_id != session_id:
            raise PersonFollowBridgeError("跟随任务已被更新的任务替换。")
        if binding.error is not None:
            raise binding.error
        return binding

    def latest_command(self, session_id: str) -> Optional[PersonFollowCommand]:
        return self._call(lambda: self._matching(session_id).command)

    def end(self, session_id: str) -> None:
        def operation():
            if self._binding is not None and self._binding.session_id == session_id:
                self._end_current()
        self._call(operation)

    def _end_current(self) -> None:
        binding, self._binding = self._binding, None
        if binding is not None and binding.target is not None:
            # 原协议允许 TRACKED→STOP 复用最后一次真实图像时间，不伪造新采样。
            target = replace(binding.target, target_seq=binding.target.target_seq + 1, tracking_state=PERSON_FOLLOW_STOP)
            self._lcm.publish(self._target_channel, encode_person_follow_target(target))

    def _on_command(self, _channel, payload) -> None:
        binding = self._binding
        if binding is None or binding.target is None or binding.error is not None:
            return
        try:
            command = decode_person_follow_command(payload)
            target = binding.target
            if command.session_id != binding.session_id or command.target_seq < target.target_seq:
                return
            if binding.command is not None and command.command_seq <= binding.command.command_seq:
                return
            if (command.target_seq != target.target_seq or command.target_stamp_sec != target.stamp_sec
                    or command.target_stamp_nsec != target.stamp_nsec
                    or (command.cloud_revision < target.required_cloud_revision
                        and command.state not in {PERSON_FOLLOW_WAITING_DATA, PERSON_FOLLOW_FAILED})):
                raise PersonFollowBridgeError("跟随速度与当前图像目标不匹配。")
            if command.state == PERSON_FOLLOW_FAILED:
                raise PersonFollowBridgeError(command.reason)
            expected = {
                PERSON_FOLLOW_TRACKED: {PERSON_FOLLOW_TRACKING, PERSON_FOLLOW_COMMAND_LOST, PERSON_FOLLOW_WAITING_DATA},
                PERSON_FOLLOW_LOST: {PERSON_FOLLOW_COMMAND_LOST},
                PERSON_FOLLOW_STOP: {PERSON_FOLLOW_STOPPED},
            }[target.tracking_state]
            if command.state not in expected:
                raise PersonFollowBridgeError("跟随速度状态与当前目标不匹配。")
            binding.lease.velocity(command.linear_x, command.linear_y, command.angular_z)
            binding.command = command
        except BaseException as error:
            binding.error = error
            try:
                binding.lease.velocity(0.0, 0.0, 0.0)
            except Exception:
                _LOGGER.exception("跟随错误后的零速指令失败。")

    def _run(self) -> None:
        subscription = None
        try:
            self._lcm = self._lcm_factory(self.lcm_url)
            subscription = self._lcm.subscribe(self._command_channel, self._on_command)
            with self._lock:
                self._started = True
            self._ready.set()
            while not self._stop.is_set():
                while True:
                    try:
                        request = self._requests.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        request.result = request.operation()
                    except BaseException as error:
                        request.error = error
                    finally:
                        request.completed.set()
                self._lcm.handle_timeout(self._poll_timeout_ms)
        except BaseException as error:
            with self._lock:
                self._failure = PersonFollowBridgeUnavailable(f"跟随 LCM 连接失败：{error}")
        finally:
            self._ready.set()
            with self._lock:
                self._started = False
            try:
                self._end_current()
            except Exception:
                _LOGGER.exception("跟随通信关闭时发布 STOP 失败。")
            while True:
                try:
                    request = self._requests.get_nowait()
                except queue.Empty:
                    break
                request.error = self._failure or PersonFollowBridgeUnavailable("跟随通信桥已停止。")
                request.completed.set()
            if subscription is not None:
                self._lcm.unsubscribe(subscription)
            self._lcm = None

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._stop.set()
            thread = self._thread
        if thread is not None:
            thread.join()
