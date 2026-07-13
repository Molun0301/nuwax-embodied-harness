from __future__ import annotations

from dataclasses import dataclass, replace
import math
import os
import queue
import struct
import threading
import time
from typing import Any, Callable


PERSON_FOLLOW_TARGET_FINGERPRINT = bytes.fromhex("e627c3082965e18d")
TWIST_FINGERPRINT = bytes.fromhex("2e7c07d7cdf7e027")

PERSON_FOLLOW_TRACKED = 0
PERSON_FOLLOW_LOST = 1
PERSON_FOLLOW_STOP = 2

DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=1"
DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL = "/person_follow_target"
DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL = "/gateway/person_follow_cmd_vel"

_TARGET_TYPE = "nuwax_msgs.PersonFollowTarget"
_TWIST_TYPE = "geometry_msgs.Twist"
_FORBIDDEN_COMMAND_CHANNELS = frozenset({"/cmd_vel", "/nav_cmd_vel"})
_TWIST_SIZE = 8 + 6 * 8
_ZERO_EPSILON = 1e-12


class PersonFollowBridgeError(RuntimeError):
    pass


class PersonFollowBridgeUnavailable(PersonFollowBridgeError):
    pass


class PersonFollowBridgeTimeout(PersonFollowBridgeError):
    pass


@dataclass(frozen=True)
class PersonFollowTarget:
    session_id: str
    query: str
    stamp_sec: int
    stamp_nsec: int
    camera_frame: str
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
class TwistCommand:
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]

    @property
    def is_zero(self) -> bool:
        return all(abs(value) <= _ZERO_EPSILON for value in (*self.linear, *self.angular))


@dataclass
class _ControlRequest:
    operation: Callable[[], Any]
    completed: threading.Event
    result: Any = None
    error: BaseException | None = None


@dataclass
class _ZeroBarrier:
    completed: threading.Event


@dataclass
class _SessionBinding:
    session_id: str
    session: Any
    fence_session: Callable[[Any], Any]
    target_timeout_sec: float
    last_target_at: float
    last_target: PersonFollowTarget | None = None


def _typed_channel(channel: str, message_type: str) -> str:
    value = str(channel).strip()
    if not value or not value.startswith("/") or "#" in value or any(ch.isspace() for ch in value):
        raise PersonFollowBridgeError(f"invalid LCM base channel {channel!r}")
    return f"{value}#{message_type}"


def _encode_string(value: str, field: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise PersonFollowBridgeError(f"{field} must be non-empty")
    encoded = value.encode("utf-8")
    length = len(encoded) + 1
    if length > 0xFFFF_FFFF:
        raise PersonFollowBridgeError(f"{field} exceeds the LCM string limit")
    return struct.pack(">I", length) + encoded + b"\0"


def _validate_target(target: PersonFollowTarget) -> None:
    _encode_string(target.session_id, "session_id")
    _encode_string(target.query, "query")
    _encode_string(target.camera_frame, "camera_frame")
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
        raise PersonFollowBridgeError("camera intrinsics and bbox must be finite")
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
    output.extend(_encode_string(target.session_id, "session_id"))
    output.extend(_encode_string(target.query, "query"))
    output.extend(struct.pack(">qi", target.stamp_sec, target.stamp_nsec))
    output.extend(_encode_string(target.camera_frame, "camera_frame"))
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


def decode_twist_command(encoded: bytes) -> TwistCommand:
    if len(encoded) != _TWIST_SIZE:
        raise PersonFollowBridgeError(
            f"geometry_msgs.Twist must be exactly {_TWIST_SIZE} bytes"
        )
    if encoded[:8] != TWIST_FINGERPRINT:
        raise PersonFollowBridgeError("geometry_msgs.Twist fingerprint mismatch")
    values = struct.unpack(">6d", encoded[8:])
    if not all(math.isfinite(value) for value in values):
        raise PersonFollowBridgeError("geometry_msgs.Twist values must be finite")
    if any(abs(value) > _ZERO_EPSILON for value in (values[2], values[3], values[4])):
        raise PersonFollowBridgeError("geometry_msgs.Twist uses unsupported axes")
    return TwistCommand(
        linear=(values[0], values[1], values[2]),
        angular=(values[3], values[4], values[5]),
    )


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
    raise PersonFollowBridgeError(
        "perception context is missing an exact positive source_timestamp_ns"
    )


def build_person_follow_target(
    *,
    session_id: str,
    query: str,
    perception_context: Any,
    detection: Any | None,
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
        stamp_sec=source_timestamp_ns // 1_000_000_000,
        stamp_nsec=source_timestamp_ns % 1_000_000_000,
        camera_frame=str(camera_info.frame_id),
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
        raise PersonFollowBridgeUnavailable(
            "python3-lcm is required for the PersonFollow gateway bridge"
        ) from exc
    return lcm.LCM(provider)


class GatewayPersonFollowBridge:
    """Single-threaded LCM bridge into one source-scoped motion-guard lease."""

    def __init__(
        self,
        *,
        lcm_url: str | None = None,
        target_channel: str | None = None,
        command_channel: str | None = None,
        lcm_factory: Callable[[str], Any] = _default_lcm_factory,
        monotonic: Callable[[], float] = time.monotonic,
        operation_timeout_sec: float = 1.0,
        barrier_timeout_sec: float = 1.0,
        poll_timeout_ms: int = 20,
    ) -> None:
        self.lcm_url = str(lcm_url or os.getenv("NUWAX_LCM_URL") or DEFAULT_LCM_URL)
        self.target_channel = str(
            target_channel
            or os.getenv("NUWAX_PERSON_FOLLOW_TARGET_CHANNEL")
            or DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL
        )
        self.command_channel = str(
            command_channel
            or os.getenv("NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL")
            or DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL
        )
        if self.command_channel in _FORBIDDEN_COMMAND_CHANNELS:
            raise PersonFollowBridgeError(
                "PersonFollow command output must not use /cmd_vel or /nav_cmd_vel"
            )
        self._typed_target_channel = _typed_channel(self.target_channel, _TARGET_TYPE)
        self._typed_command_channel = _typed_channel(self.command_channel, _TWIST_TYPE)
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
        self._thread: threading.Thread | None = None
        self._owner_thread_id: int | None = None
        self._started = False
        self._closing = False
        self._fatal_error: PersonFollowBridgeError | None = None
        self._lcm = None
        self._subscription = None
        self._barrier: _ZeroBarrier | None = None
        self._binding: _SessionBinding | None = None
        self._session_error: tuple[str, PersonFollowBridgeError] | None = None

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
        close_error: BaseException | None = None
        if started and thread.is_alive():
            try:
                self._call(self._shutdown)
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
        if close_error is not None and self._binding is not None:
            raise PersonFollowBridgeError(
                "PersonFollow bridge closed without a verified motion fence"
            ) from close_error

    def prepare_session(
        self,
        stop_target: PersonFollowTarget,
        *,
        timeout_sec: float | None = None,
    ) -> None:
        if stop_target.tracking_state != PERSON_FOLLOW_STOP:
            raise PersonFollowBridgeError("session barrier requires a STOP target")
        payload = encode_person_follow_target(stop_target)
        barrier = self._call(lambda: self._begin_zero_barrier(payload))
        timeout = self._barrier_timeout_sec if timeout_sec is None else float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise PersonFollowBridgeError("barrier timeout must be positive")
        if not barrier.completed.wait(timeout):
            try:
                self._call(lambda: self._cancel_zero_barrier(barrier))
            except PersonFollowBridgeError:
                pass
            raise PersonFollowBridgeTimeout(
                "PersonFollow owner did not acknowledge STOP with a zero command"
            )

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

    def publish_tracked(self, target: PersonFollowTarget) -> None:
        if target.tracking_state != PERSON_FOLLOW_TRACKED:
            raise PersonFollowBridgeError("publish_tracked requires a TRACKED target")
        payload = encode_person_follow_target(target)
        self._call(lambda: self._publish_tracked(target, payload))

    def end_session(self, target: PersonFollowTarget, *, reason: str) -> None:
        if target.tracking_state not in {PERSON_FOLLOW_LOST, PERSON_FOLLOW_STOP}:
            raise PersonFollowBridgeError("end_session requires LOST or STOP")
        payload = encode_person_follow_target(target)
        self._call(lambda: self._end_session(target, payload, str(reason)))

    def assert_session_healthy(self, session_id: str) -> None:
        self._call(lambda: self._assert_session_healthy(str(session_id).strip()))

    def has_active_session(self, session_id: str) -> bool:
        return bool(
            self._call(
                lambda: self._binding is not None
                and self._binding.session_id == str(session_id).strip()
            )
        )

    def _call(self, operation: Callable[[], Any]) -> Any:
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
        if closing:
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
            self._barrier = None
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
            error = self._fatal_error or PersonFollowBridgeUnavailable(
                "PersonFollow LCM owner thread stopped"
            )
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
            bridge_error = PersonFollowBridgeUnavailable(
                f"PersonFollow LCM owner failed: {error}"
            )
            bridge_error.__cause__ = error
        with self._state_lock:
            if self._fatal_error is None:
                self._fatal_error = bridge_error
        self._stop_requested.set()

    def _begin_zero_barrier(self, payload: bytes) -> _ZeroBarrier:
        if self._binding is not None:
            raise PersonFollowBridgeError("cannot prepare while a follow session is active")
        if self._barrier is not None:
            raise PersonFollowBridgeError("a PersonFollow zero barrier is already pending")
        while self._lcm.handle_timeout(0):
            pass
        barrier = _ZeroBarrier(completed=threading.Event())
        self._barrier = barrier
        self._session_error = None
        try:
            self._lcm.publish(self._typed_target_channel, payload)
        except BaseException:
            self._barrier = None
            raise
        return barrier

    def _cancel_zero_barrier(self, barrier: _ZeroBarrier) -> None:
        if self._barrier is barrier:
            self._barrier = None

    def _bind_session(
        self,
        session_id: str,
        session: Any,
        fence_session: Callable[[Any], Any],
        target_timeout_sec: float,
    ) -> None:
        if not session_id:
            raise PersonFollowBridgeError("session_id must be non-empty")
        if self._barrier is not None:
            raise PersonFollowBridgeError("cannot bind before the STOP zero barrier")
        if self._binding is not None:
            raise PersonFollowBridgeError("another PersonFollow guard session is active")
        if getattr(session, "fenced", False):
            raise PersonFollowBridgeError("cannot bind an already fenced motion session")
        self._binding = _SessionBinding(
            session_id=session_id,
            session=session,
            fence_session=fence_session,
            target_timeout_sec=target_timeout_sec,
            last_target_at=self._monotonic(),
        )
        self._session_error = None

    def _publish_tracked(self, target: PersonFollowTarget, payload: bytes) -> None:
        self._assert_session_healthy(target.session_id)
        binding = self._binding
        if binding is None or binding.session_id != target.session_id:
            raise PersonFollowBridgeError("TRACKED target has no matching guard session")
        self._lcm.publish(self._typed_target_channel, payload)
        binding.last_target = target
        binding.last_target_at = self._monotonic()

    def _end_session(
        self,
        target: PersonFollowTarget,
        payload: bytes,
        reason: str,
    ) -> None:
        publish_error: BaseException | None = None
        try:
            self._lcm.publish(self._typed_target_channel, payload)
        except BaseException as exc:
            publish_error = exc
        fence_error: BaseException | None = None
        try:
            self._fence_active(reason)
        except BaseException as exc:
            fence_error = exc
        if fence_error is not None:
            raise PersonFollowBridgeError(
                "PersonFollow lifecycle ended without a verified motion fence"
            ) from fence_error
        if publish_error is not None:
            raise PersonFollowBridgeError(
                "PersonFollow lifecycle target could not be published"
            ) from publish_error
        self._session_error = None

    def _assert_session_healthy(self, session_id: str) -> None:
        if self._session_error is not None and self._session_error[0] == session_id:
            raise self._session_error[1]

    def _on_command(self, _channel: str, encoded: bytes) -> None:
        command = decode_twist_command(encoded)
        barrier = self._barrier
        if barrier is not None:
            if command.is_zero:
                self._barrier = None
                barrier.completed.set()
            return
        binding = self._binding
        if binding is None:
            return
        try:
            if command.is_zero:
                binding.session.stop()
            else:
                binding.session.twist(
                    command.linear[0],
                    command.linear[1],
                    command.angular[2],
                )
        except BaseException as exc:
            self._fail_active_session(
                PersonFollowBridgeError("motion guard rejected a PersonFollow command"),
                cause=exc,
            )

    def _check_target_timeout(self) -> None:
        binding = self._binding
        if binding is None:
            return
        if self._monotonic() - binding.last_target_at <= binding.target_timeout_sec:
            return
        self._fail_active_session(
            PersonFollowBridgeTimeout(
                "PersonFollow target updates timed out; motion session was fenced"
            )
        )

    def _fail_active_session(
        self,
        error: PersonFollowBridgeError,
        *,
        cause: BaseException | None = None,
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
            combined = PersonFollowBridgeError(
                "PersonFollow command failed and the motion fence was not acknowledged"
            )
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
        binding = self._binding
        publish_error: BaseException | None = None
        if binding is not None and binding.last_target is not None:
            try:
                self._lcm.publish(
                    self._typed_target_channel,
                    encode_person_follow_target(
                        binding.last_target.lifecycle(PERSON_FOLLOW_STOP)
                    ),
                )
            except BaseException as exc:
                publish_error = exc
        self._fence_active("PersonFollow bridge shutdown")
        self._stop_requested.set()
        if publish_error is not None:
            raise PersonFollowBridgeError(
                "PersonFollow STOP could not be published during shutdown"
            ) from publish_error


__all__ = [
    "DEFAULT_PERSON_FOLLOW_COMMAND_CHANNEL",
    "DEFAULT_PERSON_FOLLOW_TARGET_CHANNEL",
    "GatewayPersonFollowBridge",
    "PERSON_FOLLOW_LOST",
    "PERSON_FOLLOW_STOP",
    "PERSON_FOLLOW_TRACKED",
    "PersonFollowBridgeError",
    "PersonFollowBridgeTimeout",
    "PersonFollowBridgeUnavailable",
    "PersonFollowTarget",
    "TwistCommand",
    "build_person_follow_target",
    "decode_twist_command",
    "encode_person_follow_target",
]
