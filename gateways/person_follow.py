from __future__ import annotations

from core.config_values import runtime_value

from dataclasses import dataclass, replace
import math
import os
from pathlib import Path
import queue
import re
import struct
import threading
import time
from typing import Any, Callable, Mapping, Optional, Tuple


PERSON_FOLLOW_TARGET_FINGERPRINT = bytes.fromhex("49b0abee638b1fdb")
PERSON_FOLLOW_COMMAND_FINGERPRINT = bytes.fromhex("76ad78cfd90347e8")

PERSON_FOLLOW_TRACKED = 0
PERSON_FOLLOW_LOST = 1
PERSON_FOLLOW_STOP = 2

DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=1"
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
_READY_KEYS = frozenset({"pid", "boot_id", "starttime", "subscriptions_ready"})
_BOOT_ID_PATTERN = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)
_FALSE_VALUES = frozenset({"0", "false", "no", "off"})
_TRUE_VALUES = frozenset({"1", "true", "yes", "on"})


class PersonFollowBridgeError(RuntimeError):
    pass


class PersonFollowBridgeUnavailable(PersonFollowBridgeError):
    pass


class PersonFollowBridgeTimeout(PersonFollowBridgeError):
    pass


@dataclass(frozen=True)
class PersonFollowReadiness:
    enabled: bool
    ready: bool
    ready_file: str
    reason: str
    pid: Optional[int] = None
    boot_id: Optional[str] = None
    starttime: Optional[str] = None
    subscriptions_ready: bool = False


def _parse_enabled(value: Optional[str]) -> Optional[bool]:
    normalized = str(value or "true").strip().lower()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    return None


def _resolve_person_follow_ready_file(
    environ: Mapping[str, str],
    repo_root: Optional[Path],
) -> Path:
    configured = str(environ.get("NUWAX_PERSON_FOLLOW_READY_FILE") or "").strip()
    if configured:
        return Path(configured)
    managed = _parse_enabled(environ.get("NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD", "false"))
    if managed:
        return Path("/run/nuwax/person-follow/nuwax_person_follow.ready")
    root = repo_root or Path(__file__).resolve().parent.parent
    return root / "runtime_data/person-follow/nuwax_person_follow.ready"


def _read_process_starttime(proc_root: Path, pid: int) -> Optional[str]:
    try:
        stat_line = (proc_root / str(pid) / "stat").read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return None
    if ") " not in stat_line:
        return None
    fields = stat_line.rsplit(") ", 1)[1].split()
    if len(fields) <= 19:
        return None
    starttime = fields[19]
    if not starttime.isdigit() or int(starttime) <= 0:
        return None
    return starttime


def read_person_follow_readiness(
    *,
    environ: Optional[Mapping[str, str]] = None,
    repo_root: Optional[Path] = None,
    proc_root: Path = Path("/proc"),
) -> PersonFollowReadiness:
    """严格验证 person-follow ready marker 与当前进程身份。"""

    source = os.environ if environ is None else environ
    enabled = _parse_enabled(source.get("NUWAX_PERSON_FOLLOW_AUTOSTART"))
    ready_file = _resolve_person_follow_ready_file(source, repo_root)
    if enabled is False:
        return PersonFollowReadiness(
            enabled=False,
            ready=False,
            ready_file=str(ready_file),
            reason="disabled",
        )
    if enabled is None:
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="invalid_autostart_config",
        )
    if ready_file.is_symlink() or not ready_file.is_file():
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="ready_file_missing",
        )
    try:
        lines = ready_file.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError):
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="ready_file_unreadable",
        )
    values = {}
    for line in lines:
        if "=" not in line:
            values = {}
            break
        key, value = line.split("=", 1)
        if key not in _READY_KEYS or key in values:
            values = {}
            break
        values[key] = value
    if set(values) != set(_READY_KEYS):
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="invalid_ready_marker",
        )
    pid_text = values["pid"]
    starttime = values["starttime"]
    boot_id = values["boot_id"].lower()
    if (
        not pid_text.isdigit()
        or int(pid_text) <= 0
        or not starttime.isdigit()
        or int(starttime) <= 0
        or values["subscriptions_ready"] != "1"
        or _BOOT_ID_PATTERN.fullmatch(boot_id) is None
    ):
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="invalid_ready_marker",
        )
    pid = int(pid_text)
    try:
        current_boot_id = (proc_root / "sys/kernel/random/boot_id").read_text(encoding="utf-8").strip().lower()
    except (OSError, UnicodeError):
        current_boot_id = ""
    current_starttime = _read_process_starttime(proc_root, pid)
    if boot_id != current_boot_id or starttime != current_starttime:
        return PersonFollowReadiness(
            enabled=True,
            ready=False,
            ready_file=str(ready_file),
            reason="stale_process_identity",
            pid=pid,
            boot_id=boot_id,
            starttime=starttime,
            subscriptions_ready=True,
        )
    return PersonFollowReadiness(
        enabled=True,
        ready=True,
        ready_file=str(ready_file),
        reason="ready",
        pid=pid,
        boot_id=boot_id,
        starttime=starttime,
        subscriptions_ready=True,
    )


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


@dataclass
class _Preflight:
    completed: threading.Event
    target: PersonFollowTarget
    phase: str = "tracked"
    last_command_seq: int = 0
    error: Optional[PersonFollowBridgeError] = None


@dataclass(frozen=True)
class _PreparedSession:
    session_id: str
    last_target_seq: int
    cloud_revision: int
    source_stamp: Tuple[int, int]
    last_command_seq: int


@dataclass
class _SessionBinding:
    session_id: str
    session: Any
    fence_session: Callable[[Any], Any]
    target_timeout_sec: float
    last_target_at: float
    next_target_seq: int
    cloud_revision: int
    last_source_stamp: Tuple[int, int]
    last_command_seq: int
    allow_preflight_stamp_once: bool = True
    current_target: Optional[PersonFollowTarget] = None
    last_accepted_target: Optional[PersonFollowTarget] = None


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


class GatewayPersonFollowBridge:
    """Single-threaded LCM bridge into one source-scoped motion-guard lease."""

    def __init__(
        self,
        *,
        lcm_url: Optional[str] = None,
        target_channel: Optional[str] = None,
        command_channel: Optional[str] = None,
        lcm_factory: Callable[[str], Any] = _default_lcm_factory,
        monotonic: Callable[[], float] = time.monotonic,
        operation_timeout_sec: float = 1.0,
        barrier_timeout_sec: float = 1.0,
        poll_timeout_ms: int = 20,
    ) -> None:
        self.lcm_url = str(lcm_url or runtime_value("NUWAX_LCM_URL") or DEFAULT_LCM_URL)
        self.target_channel = str(
            target_channel or runtime_value("NUWAX_PERSON_FOLLOW_TARGET_CHANNEL") or DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL
        )
        self.command_channel = str(
            command_channel or runtime_value("NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL") or DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL
        )
        if self.command_channel in _FORBIDDEN_COMMAND_CHANNELS:
            raise PersonFollowBridgeError(
                "PersonFollow command output requires a dedicated channel; "
                "/cmd_vel, /nav_cmd_vel, and /nav/cmd_vel are forbidden"
            )
        self._typed_target_channel = _typed_channel(self.target_channel, _TARGET_TYPE)
        self._typed_command_channel = _typed_channel(self.command_channel, _COMMAND_TYPE)
        if self._typed_target_channel == self._typed_command_channel:
            raise PersonFollowBridgeError("target and command channels must be distinct")
        if not math.isfinite(operation_timeout_sec) or operation_timeout_sec <= 0.0:
            raise PersonFollowBridgeError("operation_timeout_sec must be positive")
        if not math.isfinite(barrier_timeout_sec) or barrier_timeout_sec <= 0.0:
            raise PersonFollowBridgeError("barrier_timeout_sec must be positive")
        if isinstance(poll_timeout_ms, bool) or not 1 <= int(poll_timeout_ms) <= 1000:
            raise PersonFollowBridgeError("poll_timeout_ms must be in [1, 1000]")
        self._lcm_factory = lcm_factory
        self._monotonic = monotonic
        self._operation_timeout_sec = float(operation_timeout_sec)
        self._barrier_timeout_sec = float(barrier_timeout_sec)
        self._poll_timeout_ms = int(poll_timeout_ms)
        self._requests: queue.Queue[_ControlRequest] = queue.Queue()
        self._ready = threading.Event()
        self._stop_requested = threading.Event()
        self._state_lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._owner_thread_id: Optional[int] = None
        self._started = False
        self._closing = False
        self._fatal_error: Optional[PersonFollowBridgeError] = None
        self._lcm = None
        self._subscription = None
        self._preflight: Optional[_Preflight] = None
        self._prepared: Optional[_PreparedSession] = None
        self._binding: Optional[_SessionBinding] = None
        self._session_error: Optional[Tuple[str, PersonFollowBridgeError]] = None

    @property
    def started(self) -> bool:
        with self._state_lock:
            return self._started

    def start(self) -> None:
        with self._state_lock:
            if self._started:
                return
            if self._thread is not None and self._thread.is_alive():
                raise PersonFollowBridgeUnavailable("PersonFollow bridge is still starting")
            self._ready.clear()
            self._stop_requested.clear()
            self._closing = False
            self._fatal_error = None
            thread = threading.Thread(
                target=self._run,
                name="gateway-person-follow-lcm",
                daemon=True,
            )
            self._thread = thread
            thread.start()
        if not self._ready.wait(self._operation_timeout_sec):
            self._stop_requested.set()
            thread.join(timeout=self._operation_timeout_sec)
            raise PersonFollowBridgeTimeout("PersonFollow LCM bridge startup timed out")
        with self._state_lock:
            error = self._fatal_error
            started = self._started
        if error is not None:
            raise error
        if not started:
            raise PersonFollowBridgeUnavailable("PersonFollow LCM bridge did not start")

    def close(self) -> None:
        with self._state_lock:
            thread = self._thread
            started = self._started
            if thread is None:
                return
            self._closing = True
        close_error: Optional[BaseException] = None
        if started and thread.is_alive():
            try:
                self._call(self._shutdown, allow_closing=True)
            except BaseException as exc:
                close_error = exc
                self._stop_requested.set()
        else:
            self._stop_requested.set()
        thread.join(timeout=self._operation_timeout_sec + 1.0)
        if thread.is_alive():
            raise PersonFollowBridgeTimeout("PersonFollow LCM bridge thread did not stop")
        with self._state_lock:
            self._thread = None
            self._closing = False
        if close_error is not None:
            raise PersonFollowBridgeError("PersonFollow bridge closed without a verified motion fence") from close_error

    def prepare_session(
        self,
        tracked_target: PersonFollowTarget,
        *,
        timeout_sec: Optional[float] = None,
    ) -> PersonFollowTarget:
        if tracked_target.tracking_state != PERSON_FOLLOW_TRACKED:
            raise PersonFollowBridgeError("preflight requires a TRACKED target")
        payload = encode_person_follow_target(tracked_target)
        timeout = self._barrier_timeout_sec if timeout_sec is None else float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise PersonFollowBridgeError("preflight timeout must be positive")
        preflight = self._call(lambda: self._begin_preflight(tracked_target, payload))
        self._wait_preflight(preflight, timeout, "TRACKING")
        stop_target = replace(
            tracked_target,
            target_seq=tracked_target.target_seq + 1,
            tracking_state=PERSON_FOLLOW_STOP,
        )
        stop_payload = encode_person_follow_target(stop_target)
        self._call(lambda: self._advance_preflight(preflight, stop_target, stop_payload))
        self._wait_preflight(preflight, timeout, "STOPPED")
        return self._call(lambda: self._complete_preflight(preflight))

    def bind_session(
        self,
        session_id: str,
        session: Any,
        *,
        fence_session: Callable[[Any], Any],
        target_timeout_sec: float,
    ) -> None:
        if not math.isfinite(target_timeout_sec) or target_timeout_sec <= 0.0:
            raise PersonFollowBridgeError("target_timeout_sec must be positive")
        self._call(
            lambda: self._bind_session(
                str(session_id).strip(),
                session,
                fence_session,
                float(target_timeout_sec),
            )
        )

    def publish_tracked(self, target: PersonFollowTarget) -> PersonFollowTarget:
        if target.tracking_state != PERSON_FOLLOW_TRACKED:
            raise PersonFollowBridgeError("publish_tracked requires a TRACKED target")
        return self._call(lambda: self._publish_tracked(target))

    def publish_lost(self, target: PersonFollowTarget) -> PersonFollowTarget:
        if target.tracking_state != PERSON_FOLLOW_LOST:
            raise PersonFollowBridgeError("publish_lost requires a LOST target")
        return self._call(lambda: self._publish_lost(target))

    def end_session(self, target: PersonFollowTarget, *, reason: str) -> Optional[PersonFollowTarget]:
        if target.tracking_state not in {PERSON_FOLLOW_LOST, PERSON_FOLLOW_STOP}:
            raise PersonFollowBridgeError("end_session requires LOST or STOP")
        return self._call(lambda: self._end_session(target, str(reason)))

    def fence_session(self, session_id: str, *, reason: str) -> None:
        self._call(
            lambda: self._fence_matching_session(
                str(session_id).strip(),
                str(reason),
            )
        )

    def assert_session_healthy(self, session_id: str) -> None:
        self._call(lambda: self._assert_session_healthy(str(session_id).strip()))

    def has_active_session(self, session_id: str) -> bool:
        return bool(
            self._call(lambda: self._binding is not None and self._binding.session_id == str(session_id).strip())
        )

    def _call(self, operation: Callable[[], Any], *, allow_closing: bool = False) -> Any:
        if threading.get_ident() == self._owner_thread_id:
            return operation()
        with self._state_lock:
            error = self._fatal_error
            thread = self._thread
            started = self._started
            closing = self._closing
        if error is not None:
            raise error
        if not started or thread is None or not thread.is_alive():
            raise PersonFollowBridgeUnavailable("PersonFollow LCM bridge is not running")
        if closing and not allow_closing:
            raise PersonFollowBridgeUnavailable("PersonFollow LCM bridge is closing")
        request = _ControlRequest(operation=operation, completed=threading.Event())
        self._requests.put(request)
        if not request.completed.wait(self._operation_timeout_sec):
            self._stop_requested.set()
            raise PersonFollowBridgeTimeout("PersonFollow LCM owner thread did not respond")
        if request.error is not None:
            if isinstance(request.error, PersonFollowBridgeError):
                raise request.error
            raise PersonFollowBridgeError("PersonFollow LCM operation failed") from request.error
        return request.result

    def _run(self) -> None:
        self._owner_thread_id = threading.get_ident()
        try:
            lcm_context = self._lcm_factory(self.lcm_url)
            subscription = lcm_context.subscribe(
                self._typed_command_channel,
                self._on_command,
            )
            self._lcm = lcm_context
            self._subscription = subscription
            with self._state_lock:
                self._started = True
            self._ready.set()
            while not self._stop_requested.is_set():
                self._drain_requests()
                if self._stop_requested.is_set():
                    break
                lcm_context.handle_timeout(self._poll_timeout_ms)
                self._check_target_timeout()
        except BaseException as exc:
            self._set_fatal(exc)
        finally:
            self._ready.set()
            try:
                self._fence_active("PersonFollow LCM owner stopped")
            except BaseException as exc:
                self._set_fatal(exc)
            self._fail_pending_requests()
            if self._lcm is not None and self._subscription is not None:
                try:
                    self._lcm.unsubscribe(self._subscription)
                except BaseException as exc:
                    self._set_fatal(exc)
            self._subscription = None
            self._lcm = None
            self._preflight = None
            self._prepared = None
            self._owner_thread_id = None
            with self._state_lock:
                self._started = False

    def _drain_requests(self) -> None:
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            try:
                request.result = request.operation()
            except BaseException as exc:
                request.error = exc
            finally:
                request.completed.set()

    def _fail_pending_requests(self) -> None:
        with self._state_lock:
            error = self._fatal_error or PersonFollowBridgeUnavailable("PersonFollow LCM owner thread stopped")
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            request.error = error
            request.completed.set()

    def _set_fatal(self, error: BaseException) -> None:
        if isinstance(error, PersonFollowBridgeError):
            bridge_error = error
        else:
            bridge_error = PersonFollowBridgeUnavailable(f"PersonFollow LCM owner failed: {error}")
            bridge_error.__cause__ = error
        with self._state_lock:
            if self._fatal_error is None:
                self._fatal_error = bridge_error
        self._stop_requested.set()

    def _wait_preflight(
        self,
        preflight: _Preflight,
        timeout: float,
        expected_state: str,
    ) -> None:
        if not preflight.completed.wait(timeout):
            try:
                self._call(lambda: self._cancel_preflight(preflight))
            except PersonFollowBridgeError:
                pass
            raise PersonFollowBridgeTimeout(f"PersonFollow preflight timed out waiting for {expected_state}")
        if preflight.error is not None:
            self._call(lambda: self._cancel_preflight(preflight))
            raise preflight.error

    def _begin_preflight(
        self,
        target: PersonFollowTarget,
        payload: bytes,
    ) -> _Preflight:
        if self._binding is not None:
            raise PersonFollowBridgeError("cannot prepare while a follow session is active")
        if self._preflight is not None:
            raise PersonFollowBridgeError("a PersonFollow preflight is already pending")
        while self._lcm.handle_timeout(0):
            pass
        preflight = _Preflight(completed=threading.Event(), target=target)
        self._preflight = preflight
        self._prepared = None
        self._session_error = None
        try:
            self._lcm.publish(self._typed_target_channel, payload)
        except BaseException:
            self._preflight = None
            raise
        return preflight

    def _advance_preflight(
        self,
        preflight: _Preflight,
        target: PersonFollowTarget,
        payload: bytes,
    ) -> None:
        if self._preflight is not preflight or preflight.error is not None:
            raise preflight.error or PersonFollowBridgeError("preflight is no longer active")
        if preflight.phase != "tracked":
            raise PersonFollowBridgeError("preflight TRACKED phase is incomplete")
        preflight.phase = "stop"
        preflight.target = target
        preflight.completed.clear()
        try:
            self._lcm.publish(self._typed_target_channel, payload)
        except BaseException:
            self._preflight = None
            raise

    def _complete_preflight(self, preflight: _Preflight) -> PersonFollowTarget:
        if self._preflight is not preflight or preflight.error is not None:
            raise preflight.error or PersonFollowBridgeError("preflight is no longer active")
        if preflight.phase != "stop" or not preflight.completed.is_set():
            raise PersonFollowBridgeError("preflight STOP phase is incomplete")
        target = preflight.target
        self._prepared = _PreparedSession(
            session_id=target.session_id,
            last_target_seq=target.target_seq,
            cloud_revision=target.required_cloud_revision,
            source_stamp=(target.stamp_sec, target.stamp_nsec),
            last_command_seq=preflight.last_command_seq,
        )
        self._preflight = None
        return target

    def _cancel_preflight(self, preflight: _Preflight) -> None:
        if self._preflight is preflight:
            self._preflight = None

    def _bind_session(
        self,
        session_id: str,
        session: Any,
        fence_session: Callable[[Any], Any],
        target_timeout_sec: float,
    ) -> None:
        if not session_id:
            raise PersonFollowBridgeError("session_id must be non-empty")
        if self._preflight is not None:
            raise PersonFollowBridgeError("cannot bind while preflight is pending")
        if self._binding is not None:
            raise PersonFollowBridgeError("another PersonFollow guard session is active")
        prepared = self._prepared
        if prepared is None or prepared.session_id != session_id:
            raise PersonFollowBridgeError("cannot bind before matching preflight succeeds")
        if getattr(session, "fenced", False):
            raise PersonFollowBridgeError("cannot bind an already fenced motion session")
        self._binding = _SessionBinding(
            session_id=session_id,
            session=session,
            fence_session=fence_session,
            target_timeout_sec=target_timeout_sec,
            last_target_at=self._monotonic(),
            next_target_seq=prepared.last_target_seq + 1,
            cloud_revision=prepared.cloud_revision,
            last_source_stamp=prepared.source_stamp,
            last_command_seq=prepared.last_command_seq,
        )
        self._prepared = None
        self._session_error = None

    def _publish_tracked(self, target: PersonFollowTarget) -> PersonFollowTarget:
        self._assert_session_healthy(target.session_id)
        binding = self._binding
        if binding is None or binding.session_id != target.session_id:
            raise PersonFollowBridgeError("TRACKED target has no matching guard session")
        source_stamp = (target.stamp_sec, target.stamp_nsec)
        if source_stamp == binding.last_source_stamp and binding.allow_preflight_stamp_once:
            binding.allow_preflight_stamp_once = False
        elif source_stamp <= binding.last_source_stamp:
            self._fail_active_session(
                PersonFollowBridgeError("PersonFollow target source timestamps must strictly increase")
            )
            raise self._session_error[1]
        else:
            binding.allow_preflight_stamp_once = False
        correlated = replace(
            target,
            target_seq=binding.next_target_seq,
            required_cloud_revision=binding.cloud_revision,
        )
        payload = encode_person_follow_target(correlated)
        self._lcm.publish(self._typed_target_channel, payload)
        binding.current_target = correlated
        binding.next_target_seq += 1
        binding.last_source_stamp = source_stamp
        binding.last_target_at = self._monotonic()
        return correlated

    def _publish_lost(self, target: PersonFollowTarget) -> PersonFollowTarget:
        self._assert_session_healthy(target.session_id)
        binding = self._binding
        if binding is None or binding.session_id != target.session_id:
            raise PersonFollowBridgeError("LOST target has no matching guard session")
        source_stamp = (target.stamp_sec, target.stamp_nsec)
        if source_stamp <= binding.last_source_stamp:
            self._fail_active_session(
                PersonFollowBridgeError("PersonFollow target source timestamps must strictly increase")
            )
            raise self._session_error[1]
        correlated = replace(
            target,
            target_seq=binding.next_target_seq,
            required_cloud_revision=binding.cloud_revision,
        )
        payload = encode_person_follow_target(correlated)
        self._lcm.publish(self._typed_target_channel, payload)
        binding.current_target = correlated
        binding.next_target_seq += 1
        binding.last_source_stamp = source_stamp
        binding.allow_preflight_stamp_once = False
        binding.last_target_at = self._monotonic()
        return correlated

    def _end_session(
        self,
        target: PersonFollowTarget,
        reason: str,
    ) -> Optional[PersonFollowTarget]:
        self._assert_session_healthy(target.session_id)
        binding = self._binding
        if binding is None or binding.session_id != target.session_id:
            raise PersonFollowBridgeError("lifecycle target has no matching guard session")
        correlated: Optional[PersonFollowTarget] = None
        publish_error: Optional[BaseException] = None
        if (target.stamp_sec, target.stamp_nsec) > binding.last_source_stamp:
            correlated = replace(
                target,
                target_seq=binding.next_target_seq,
                required_cloud_revision=binding.cloud_revision,
            )
            payload = encode_person_follow_target(correlated)
            try:
                self._lcm.publish(self._typed_target_channel, payload)
            except BaseException as exc:
                publish_error = exc
        fence_error: Optional[BaseException] = None
        try:
            self._fence_active(reason)
        except BaseException as exc:
            fence_error = exc
        if fence_error is not None:
            raise PersonFollowBridgeError(
                "PersonFollow lifecycle ended without a verified motion fence"
            ) from fence_error
        if publish_error is not None:
            raise PersonFollowBridgeError("PersonFollow lifecycle target could not be published") from publish_error
        self._session_error = None
        return correlated

    def _assert_session_healthy(self, session_id: str) -> None:
        if self._session_error is not None and self._session_error[0] == session_id:
            raise self._session_error[1]

    def _fence_matching_session(self, session_id: str, reason: str) -> None:
        self._assert_session_healthy(session_id)
        binding = self._binding
        if binding is None or binding.session_id != session_id:
            raise PersonFollowBridgeError("no matching PersonFollow guard session")
        self._fence_active(reason)
        self._session_error = None

    def _on_command(self, _channel: str, encoded: bytes) -> None:
        try:
            command = decode_person_follow_command(encoded)
        except PersonFollowBridgeError as exc:
            if self._preflight is not None:
                self._abort_preflight(exc)
            elif self._binding is not None:
                self._fail_active_session(exc)
            return
        preflight = self._preflight
        if preflight is not None:
            error = self._validate_command(
                command,
                preflight.target,
                preflight.last_command_seq,
                preflight=True,
            )
            if error is not None:
                self._abort_preflight(error)
                return
            preflight.last_command_seq = command.command_seq
            if command.state == PERSON_FOLLOW_WAITING_DATA:
                return
            preflight.completed.set()
            return
        binding = self._binding
        if binding is None:
            return
        target = binding.current_target
        if target is None:
            self._fail_active_session(PersonFollowBridgeError("PersonFollow command has no active target"))
            return
        error = self._validate_command(
            command,
            target,
            binding.last_command_seq,
            preflight=False,
        )
        if error is not None:
            self._fail_active_session(error)
            return
        try:
            if command.state in {PERSON_FOLLOW_COMMAND_LOST, PERSON_FOLLOW_STOPPED}:
                binding.session.stop()
            else:
                binding.session.twist(
                    command.linear_x,
                    command.linear_y,
                    command.angular_z,
                )
            binding.last_command_seq = command.command_seq
            binding.last_accepted_target = target
        except BaseException as exc:
            self._fail_active_session(
                PersonFollowBridgeError("motion guard rejected a PersonFollow command"),
                cause=exc,
            )

    def _validate_command(
        self,
        command: PersonFollowCommand,
        target: PersonFollowTarget,
        last_command_seq: int,
        *,
        preflight: bool,
    ) -> Optional[PersonFollowBridgeError]:
        if (
            command.session_id != target.session_id
            or command.target_seq != target.target_seq
            or command.target_stamp_sec != target.stamp_sec
            or command.target_stamp_nsec != target.stamp_nsec
        ):
            return PersonFollowBridgeError("PersonFollow command correlation mismatch")
        if command.command_seq <= last_command_seq:
            return PersonFollowBridgeError("PersonFollow command_seq is not strictly increasing")
        if command.state == PERSON_FOLLOW_WAITING_DATA:
            if preflight:
                return None
            return PersonFollowBridgeError("WAITING_DATA is only valid during preflight")
        if command.state == PERSON_FOLLOW_FAILED:
            return PersonFollowBridgeError(f"PersonFollow owner reported FAILED: {command.reason}")
        if command.cloud_revision < target.required_cloud_revision:
            return PersonFollowBridgeError("PersonFollow command correlation mismatch")
        expected = {
            PERSON_FOLLOW_TRACKED: PERSON_FOLLOW_TRACKING,
            PERSON_FOLLOW_LOST: PERSON_FOLLOW_COMMAND_LOST,
            PERSON_FOLLOW_STOP: PERSON_FOLLOW_STOPPED,
        }[target.tracking_state]
        if command.state != expected:
            return PersonFollowBridgeError("PersonFollow command state does not match target")
        return None

    def _abort_preflight(self, error: PersonFollowBridgeError) -> None:
        preflight = self._preflight
        if preflight is None:
            return
        self._preflight = None
        preflight.error = error
        preflight.completed.set()

    def _check_target_timeout(self) -> None:
        binding = self._binding
        if binding is None:
            return
        if self._monotonic() - binding.last_target_at <= binding.target_timeout_sec:
            return
        self._fail_active_session(
            PersonFollowBridgeTimeout("PersonFollow target updates timed out; motion session was fenced")
        )

    def _fail_active_session(
        self,
        error: PersonFollowBridgeError,
        *,
        cause: Optional[BaseException] = None,
    ) -> None:
        binding = self._binding
        if binding is None:
            return
        self._binding = None
        if cause is not None:
            error.__cause__ = cause
        try:
            binding.fence_session(binding.session)
        except BaseException as fence_error:
            combined = PersonFollowBridgeError("PersonFollow command failed and the motion fence was not acknowledged")
            combined.__cause__ = fence_error
            error = combined
        self._session_error = (binding.session_id, error)

    def _fence_active(self, _reason: str) -> None:
        binding = self._binding
        if binding is None:
            return
        self._binding = None
        binding.fence_session(binding.session)

    def _shutdown(self) -> None:
        self._fence_active("PersonFollow bridge shutdown")
        self._stop_requested.set()


__all__ = [
    "DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL",
    "DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL",
    "GatewayPersonFollowBridge",
    "PERSON_FOLLOW_COMMAND_FINGERPRINT",
    "PERSON_FOLLOW_COMMAND_LOST",
    "PERSON_FOLLOW_FAILED",
    "PERSON_FOLLOW_LOST",
    "PERSON_FOLLOW_STOP",
    "PERSON_FOLLOW_STOPPED",
    "PERSON_FOLLOW_TRACKED",
    "PERSON_FOLLOW_TRACKING",
    "PERSON_FOLLOW_WAITING_DATA",
    "PersonFollowBridgeError",
    "PersonFollowBridgeTimeout",
    "PersonFollowBridgeUnavailable",
    "PersonFollowReadiness",
    "PersonFollowCommand",
    "PersonFollowTarget",
    "build_person_follow_target",
    "decode_person_follow_command",
    "encode_person_follow_target",
    "read_person_follow_readiness",
]
