from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
import uuid
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Protocol, Set, Tuple, TYPE_CHECKING, runtime_checkable
from urllib.parse import quote

from contracts.geometry import Pose
from core.logging_utils import StreamBusLogHandler
from contracts.map_workspace import MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType, MapVersion
from contracts.navigation import (
    ExplorationState,
    ExplorationStatus,
    ExploreAreaRequest,
    NavigationGoal,
    NavigationStatus,
)
from contracts.runtime_views import MapSnapshot
from providers.data_plane import (
    ExplorationDataPlane,
    LocalizationDataPlane,
    MappingDataPlane,
    NavigationDataPlane,
    RobotAssemblyProtocol,
)
from providers.exploration import ExplorationProvider
from providers.localization import LocalizationProvider
from providers.maps import MapProvider
from providers.motion import MotionControl
from providers.navigation import NavigationProvider
from providers.pointcloud import PointCloudProvider
from services.geometry.config import PlatformGeometryConfig
from services.geometry.external_debug_observer import ExternalGeometryDebugObserver
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
from services.geometry.loop_closure_state import read_loop_closure_from_db

from contracts.naming import build_safe_dir_segment
from services.localization.frame_tree_ops import resolve_frame_transform

# 与 core.settings._normalize_lcm_backend 保持一致，只允许生产后端。
_CANONICAL_LCM_BACKENDS: FrozenSet[str] = frozenset({"real-lcm"})

if TYPE_CHECKING:
    from core.event_bus import EventBus

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime")

_LIVE_MAPPING_DIAGNOSTICS_TTL_SEC = 0.5
_EXTERNAL_EXPLORATION_LOG_SCAN_MIN_INTERVAL_SEC = 2.0


_GEOMETRY_SIDE_CAR_RESTART_LIMIT = 2
_GEOMETRY_MONITOR_INTERVAL_SEC = 1.0
_GEOMETRY_STREAMING_WINDOW_SEC = 3.0
# 平台几何运行时由 Rust nuwax-geometry-runtime 子进程、gRPC 和 LCM 驱动。
_GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"
_MAIN_LOG_APPEND_LOCK = threading.Lock()
_GO2_SH_MAIN_LOG_LOGGER = "nuwax_robot_bridge.geometry.go2.sh"
_SUBPROCESS_LOG_LEVEL_NAMES = frozenset(
    {"DEBUG", "INFO", "WARN", "WARNING", "ERROR", "FATAL", "CRITICAL"}
)
_ROS2_NODE_LOG_RE = re.compile(
    r"^\[(?P<proc>[^\]]+)\]\s+"
    r"\[(?P<level>DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]\s+"
    r"\[(?P<stamp>[\d.]+)\]\s+"
    r"(?P<rest>.+)$",
    re.IGNORECASE,
)
_LAUNCH_LOG_RE = re.compile(
    r"^\[(?P<level>DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]\s+"
    r"(?P<rest>.+)$",
    re.IGNORECASE,
)


_RemoteOperationToken = Tuple[object, Optional[int]]
_ANY_DELIVERY_OWNER = object()


@dataclass(eq=False)
class _SubscriptionDeliveryTicket:
    """One runtime callback fence shared by all subscription kinds."""

    ticket_id: int
    kind: str
    subscription_id: Optional[int]
    callback: Callable[..., object]
    provider: Optional[object]
    provider_generation: Optional[int]
    published: bool = False
    invalidated: bool = False
    active_count: int = 0
    active_threads: Dict[int, int] = field(default_factory=dict)


def _remote_is_connected(remote: object) -> bool:
    try:
        connected = getattr(remote, "is_connected", False)
        if callable(connected):
            connected = connected()
        return bool(connected)
    except Exception:
        return False


def _remote_connection_generation(remote: object) -> Optional[int]:
    try:
        generation = getattr(remote, "connection_generation", None)
        if callable(generation):
            generation = generation()
        return int(generation) if generation is not None else None
    except (TypeError, ValueError):
        return None


def _cancel_remote_stream_handle(handle, *, stream_name: str) -> bool:
    if handle is None:
        return True
    cancel = getattr(handle, "cancel", None)
    if not callable(cancel):
        return True
    try:
        result = cancel()
        return result is not False
    except Exception:
        LOGGER.exception("取消远端 %s stream handle 失败。", stream_name)
        return False


def _capture_remote_operation_token_locked(
    runtime: object,
    remote: object,
) -> Optional[_RemoteOperationToken]:
    if hasattr(runtime, "_remote_provider"):
        if getattr(runtime, "_remote_provider", None) is not remote:
            return None
    if not _remote_is_connected(remote):
        return None
    return remote, _remote_connection_generation(remote)


def _remote_operation_token_is_current_locked(
    runtime: object,
    token: Optional[_RemoteOperationToken],
) -> bool:
    if token is None:
        return False
    remote, generation = token
    if hasattr(runtime, "_remote_provider"):
        if getattr(runtime, "_remote_provider", None) is not remote:
            return False
    return bool(
        _remote_is_connected(remote)
        and _remote_connection_generation(remote) == generation
    )


def _exploration_state_request_id(state: object) -> str:
    return str(getattr(state, "current_request_id", "") or "").strip()


def _is_complete_exploration_terminal(
    state: object,
    *,
    request_id: str,
    safe_stop_only: bool = False,
) -> bool:
    status = getattr(state, "status", None)
    terminal_statuses = {
        ExplorationStatus.SUCCEEDED,
        ExplorationStatus.FAILED,
        ExplorationStatus.CANCELLED,
    }
    if safe_stop_only:
        terminal_statuses.discard(ExplorationStatus.FAILED)
    metadata = dict(getattr(state, "metadata", {}) or {})
    return bool(
        _exploration_state_request_id(state) == str(request_id or "").strip()
        and status in terminal_statuses
        and metadata.get("exploration_terminal") is True
    )


def _restore_mapping_session_mode_locked(runtime: object) -> None:
    mapping_context = None
    context_known = hasattr(runtime, "_mapping_session_context")
    reader = getattr(runtime, "get_mapping_session_context", None)
    if callable(reader):
        context_known = True
        try:
            mapping_context = reader()
        except Exception:
            LOGGER.exception(
                "读取 mapping session context 失败；保留当前 session mode。"
            )
            return
    elif context_known:
        mapping_context = getattr(runtime, "_mapping_session_context", None)
    if context_known:
        setattr(
            runtime,
            "_external_geometry_session_mode",
            "mapping"
            if isinstance(mapping_context, dict) and mapping_context.get("active")
            else None,
        )


def _commit_remote_exploration_state_locked(
    runtime: object,
    state: ExplorationState,
    *,
    request_id: str,
    request: Optional[ExploreAreaRequest] = None,
    map_version_id: Optional[str] = None,
) -> bool:
    expected_id = str(request_id or "").strip()
    if not expected_id or _exploration_state_request_id(state) != expected_id:
        return False
    status = state.status
    terminal = status in {
        ExplorationStatus.SUCCEEDED,
        ExplorationStatus.FAILED,
        ExplorationStatus.CANCELLED,
    }
    metadata = dict(state.metadata or {})
    if terminal and metadata.get("exploration_terminal") is not True:
        return False

    current = getattr(runtime, "_external_exploration_request", None)
    current_id = str(getattr(current, "request_id", "") or "").strip()
    if request is None:
        if current_id != expected_id:
            return False
    elif current_id and current_id != expected_id:
        return False

    state_copy = state.model_copy(deep=True)
    if terminal:
        setattr(runtime, "_external_exploration_request", None)
        setattr(runtime, "_external_exploration_terminal_state", state_copy)
        _restore_mapping_session_mode_locked(runtime)
    else:
        if request is not None:
            active_request = (
                request.model_copy(deep=True)
                if hasattr(request, "model_copy")
                else request
            )
            setattr(runtime, "_external_exploration_request", active_request)
        setattr(runtime, "_external_exploration_terminal_state", None)
        setattr(runtime, "_external_geometry_session_mode", "explore")
        if map_version_id is not None:
            setattr(
                runtime,
                "_external_exploration_version_id",
                str(map_version_id or "").strip() or None,
            )

    return True


def _canonical_main_log_level(level: str) -> str:
    upper = str(level or "INFO").strip().upper()
    if upper in {"WARN", "WARNING"}:
        return "WARNING"
    if upper in {"FATAL", "CRITICAL"}:
        return "ERROR"
    if upper in _SUBPROCESS_LOG_LEVEL_NAMES:
        return upper
    return "INFO"


def _main_log_level_to_logging(level: str) -> int:
    canonical = _canonical_main_log_level(level)
    return {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
    }.get(canonical, logging.INFO)


def _normalize_subprocess_log_line(message: str) -> Tuple[str, str]:
    """解析 go2.sh / ros2 launch 子进程行，提取真实等级并去掉重复前缀。"""

    text = str(message or "").strip()
    if not text:
        return "INFO", text

    node_match = _ROS2_NODE_LOG_RE.match(text)
    if node_match:
        level = _canonical_main_log_level(node_match.group("level"))
        proc = node_match.group("proc")
        rest = node_match.group("rest")
        return level, "[%s] %s" % (proc, rest)

    launch_match = _LAUNCH_LOG_RE.match(text)
    if launch_match:
        level = _canonical_main_log_level(launch_match.group("level"))
        return level, launch_match.group("rest").strip()

    lowered = text.lower()
    if (
        "deprecationwarning" in lowered
        or ": warning:" in lowered
        or lowered.startswith("warning:")
        or lowered.startswith("go2.sh: warning:")
    ):
        return "WARNING", text

    return "INFO", text


def _project_log_file() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    log_dir = Path(os.environ.get("NUWAX_LOG_DIR", str(repo_root))).expanduser()
    configured = str(os.environ.get("NUWAX_LOG_FILE", "") or "").strip()
    if configured:
        return Path(configured).expanduser()
    return log_dir / "nuwax_robot_bridge.log"


def _format_main_log_line(logger_name: str, message: str, *, level: str = "INFO") -> str:
    now = datetime.now()
    asctime = now.strftime("%Y-%m-%d %H:%M:%S") + ",%03d" % (now.microsecond // 1000)
    return "%s [%s] [%s] %s\n" % (asctime, level, logger_name, message)


def _publish_main_log_stream(logger_name: str, message: str, *, level: str = "INFO") -> None:
    """将 go2.sh 子进程行推送到 StreamBus logs.entry（不经 file handler，避免重复写盘）。"""

    record = logging.LogRecord(
        name=str(logger_name or _GO2_SH_MAIN_LOG_LOGGER),
        level=_main_log_level_to_logging(level),
        pathname=__file__,
        lineno=0,
        msg=str(message or ""),
        args=(),
        exc_info=None,
    )
    for handler in logging.getLogger().handlers:
        if not isinstance(handler, StreamBusLogHandler):
            continue
        try:
            if handler.filter(record):
                handler.emit(record)
        except Exception:
            LOGGER.debug("go2.sh 日志推送到 StreamBus 失败。", exc_info=True)
        return


def _append_main_log_line(logger_name: str, message: str) -> None:
    text = str(message or "").rstrip("\r\n")
    if not text:
        return
    level, normalized = _normalize_subprocess_log_line(text)
    path = _project_log_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    line = _format_main_log_line(logger_name, normalized, level=level)
    with _MAIN_LOG_APPEND_LOCK:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(line)
    _publish_main_log_stream(logger_name, normalized, level=level)


@dataclass
class _ManagedProcess:
    name: str
    command: List[str]
    log_path: Path
    cwd: Path
    process_title: Optional[str] = None
    display_name: Optional[str] = None
    mirror_main_log: bool = False
    process: Optional[subprocess.Popen] = None
    started_at: Optional[float] = None
    started_at_iso: Optional[str] = None
    exit_code: Optional[int] = None
    last_error: Optional[str] = None
    restart_count: int = 0
    _output_thread: Optional[threading.Thread] = field(default=None, repr=False)

    def start(self, *, restart: bool = False) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        if restart:
            self.restart_count += 1
        banner = "\n[%s] start %s\n" % (_utc_now_iso(), self.command_display())
        if self.mirror_main_log:
            with self.log_path.open("ab") as log_file:
                log_file.write(banner.encode("utf-8"))
            _append_main_log_line(self._main_log_logger_name(), banner.strip())
            self.process = subprocess.Popen(
                self.launch_command(),
                cwd=str(self.cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                bufsize=0,
            )
            self._output_thread = threading.Thread(
                target=self._pump_output_to_logs,
                name="nuwax-managed-process-%s" % self.name,
                daemon=True,
            )
            self._output_thread.start()
        else:
            with self.log_path.open("ab") as log_file:
                log_file.write(banner.encode("utf-8"))
                self.process = subprocess.Popen(
                    self.launch_command(),
                    cwd=str(self.cwd),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
        self.started_at = time.monotonic()
        self.started_at_iso = _utc_now_iso()
        self.exit_code = None
        self.last_error = None

    def _main_log_logger_name(self) -> str:
        if self.name in {"nuwax-geometry-runtime", "nuwax_geometry_runtime"}:
            return _GO2_SH_MAIN_LOG_LOGGER
        return "nuwax_robot_bridge.geometry.%s" % self.name

    def _pump_output_to_logs(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        logger_name = self._main_log_logger_name()
        try:
            with self.log_path.open("ab") as sidecar:
                while True:
                    chunk = process.stdout.readline()
                    if not chunk:
                        break
                    sidecar.write(chunk)
                    sidecar.flush()
                    text = chunk.decode("utf-8", errors="replace").rstrip("\r\n")
                    if text:
                        _append_main_log_line(logger_name, text)
        except Exception:
            LOGGER.debug("托管进程日志同步到主日志失败 process=%s", self.name, exc_info=True)

    def poll(self) -> Optional[int]:
        if self.process is None:
            return self.exit_code
        code = self.process.poll()
        if code is not None:
            self.exit_code = int(code)
            if code != 0:
                self.last_error = _read_recent_log(self.log_path)
        return self.exit_code

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(
        self,
        *,
        timeout_sec: float = 5.0,
        graceful_launch_shutdown: bool = False,
        graceful_timeout_sec: float = 20.0,
    ) -> None:
        process = self.process
        if process is None or process.poll() is not None:
            self.poll()
            return

        def _signal_process_group(sig: signal.Signals) -> None:
            # 先验证进程是否仍在运行，避免 PID 回收竞态：
            # 旧进程退出后新进程恰好分配相同 PID，os.killpg 会误杀新进程。
            if process.poll() is not None:
                return
            try:
                os.killpg(os.getpgid(process.pid), sig)
            except Exception:
                try:
                    # 再次确认进程仍在运行后再发信号
                    if process.poll() is not None:
                        return
                    if sig == signal.SIGINT:
                        process.send_signal(signal.SIGINT)
                    elif sig == signal.SIGTERM:
                        process.terminate()
                    else:
                        process.kill()
                except Exception:
                    pass

        if graceful_launch_shutdown:
            _signal_process_group(signal.SIGINT)
            try:
                process.wait(timeout=max(1.0, float(graceful_timeout_sec)))
                self.poll()
                return
            except subprocess.TimeoutExpired:
                pass

        _signal_process_group(signal.SIGTERM)
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            _signal_process_group(signal.SIGKILL)
            try:
                process.wait(timeout=1.0)
            except Exception:
                pass
        self.poll()

    def status(self) -> Dict[str, object]:
        self.poll()
        return {
            "name": self.name,
            "process_title": self.process_title,
            "display_name": self.display_name,
            "running": self.is_running(),
            "pid": self.process.pid if self.process is not None and self.is_running() else None,
            "command": self.command,
            "command_display": self.command_display(),
            "log_path": str(self.log_path),
            "started_at": self.started_at_iso,
            "exit_code": self.exit_code,
            "restart_count": self.restart_count,
            "last_error": self.last_error,
        }

    def command_display(self) -> str:
        return " ".join(shlex.quote(str(part)) for part in self.command)

    def launch_command(self) -> List[str]:
        title = str(self.process_title or "").strip()
        if not title:
            return list(self.command)
        return ["bash", "-c", 'exec -a "$0" "$@"', title, *self.command]


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_recent_log(path: Path, *, max_bytes: int = 4096) -> str:
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes), os.SEEK_SET)
            data = handle.read().decode("utf-8", errors="replace")
    except Exception as exc:
        return "读取日志失败：%s" % exc
    lines = [line.strip() for line in data.splitlines() if line.strip()]
    return "\n".join(lines[-20:])


def _read_log_from_offset(path: Path, offset: int, *, max_bytes: int = 8 * 1024 * 1024) -> str:
    """Read log bytes from offset (used to scan the current exploration session)."""
    try:
        safe_offset = max(0, int(offset))
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            if safe_offset >= size:
                return ""
            handle.seek(safe_offset, os.SEEK_SET)
            data = handle.read(max(1, min(max_bytes, size - safe_offset))).decode("utf-8", errors="replace")
    except Exception as exc:
        return "读取日志失败：%s" % exc
    return data


def _capture_log_file_offset(path: Path) -> int:
    try:
        return int(path.stat().st_size)
    except Exception:
        return 0


def _parse_iso_timestamp(value: object) -> Optional[datetime]:
    if not value:
        return None
    if isinstance(value, datetime):
        parsed = value
    else:
        try:
            parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        except ValueError:
            return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


@runtime_checkable
class GeometryBackend(
    LocalizationDataPlane,
    MappingDataPlane,
    NavigationDataPlane,
    ExplorationDataPlane,
    Protocol,
):
    """平台几何运行时背后的统一后端契约。"""


class PlatformGeometryRuntime:
    """平台几何引擎入口。

    当前阶段的平台几何运行时除了承接 provider owner 语义，还负责：
    1. 约束平台地图版本对应的几何资产目录；
    2. 管理当前激活的平台地图版本几何上下文；
    3. 在历史地图激活时，把平台版本快照重新组织为平台几何输出。
    """

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        catalog_root: str,
        robot: RobotAssemblyProtocol,
        event_bus: Optional["EventBus"] = None,
    ) -> None:
        self.config = config
        self.robot = robot
        self._event_bus = event_bus
        self._catalog_root = Path(catalog_root).expanduser()
        self._catalog_root.mkdir(parents=True, exist_ok=True)
        self._active_map_version: Optional[MapVersion] = None
        self._last_activation_result: Dict[str, object] = {}
        # 以下4个 Python provider 已由 Rust bridge (GeometryRuntimeProvider) 替代，保留 None 占位以兼容现有引用
        self._live_map_provider = None
        self._tf_localization_provider = None
        self._odometry_publisher = None
        self._point_cloud_publisher = None
        self._live_geometry_start_lock = threading.RLock()
        self._live_geometry_start_attempted = False
        self._pending_localization_callbacks = []
        self._pending_map_callbacks = []
        self._localization_update_subscribers: Dict[int, Callable[..., None]] = {}
        self._map_update_subscribers: Dict[int, Callable[..., None]] = {}
        self._remote_localization_subscription_handles: Dict[int, object] = {}
        self._remote_map_subscription_handles: Dict[int, object] = {}
        self._next_localization_subscription_id = 1
        self._next_map_subscription_id = 1
        self._navigation_update_subscribers: Dict[int, Callable[[object], None]] = {}
        self._remote_navigation_subscription_ids: Dict[int, int] = {}
        self._next_navigation_subscription_id = 1
        self._runtime_state_lock = threading.RLock()
        self._mapping_lifecycle_lock = threading.RLock()
        self._mapping_session_context: Optional[Dict[str, object]] = None
        self._runtime_phase = "disabled" if not self._is_geometry_profile_enabled() else "preflight"
        self._runtime_message = "平台几何运行时尚未启动。"
        self._runtime_blocked = False
        self._runtime_started = False
        self._provider_attach_lock = threading.RLock()
        self._provider_attach_epoch = 0
        self._provider_attach_stopping = False
        self._subscription_delivery_condition = threading.Condition()
        self._subscription_delivery_tickets: Dict[
            int,
            _SubscriptionDeliveryTicket,
        ] = {}
        self._next_subscription_delivery_ticket_id = 1
        self._runtime_stop_event = threading.Event()
        self._runtime_monitor_thread: Optional[threading.Thread] = None
        self._managed_processes: Dict[str, _ManagedProcess] = {}
        self._runtime_diagnostics: List[Dict[str, object]] = []
        self._map_stream_tracker: Dict[str, object] = {}
        self._external_live_pose: Optional[Pose] = None
        self._external_live_pose_at_monotonic = 0.0
        self._external_live_pose_lock = threading.Lock()
        self._debug_initial_pose_hint: Optional[Pose] = None
        self._debug_initial_pose_anchor: Optional[Pose] = None
        self._debug_initial_pose_hint_until = 0.0
        self._debug_initial_pose_hint_stable_count = 0
        # 正式导航后端由 `_remote_provider`（GeometryRuntimeProvider）持有。
        self._cmd_vel_bridge = None
        self._geometry_runtime_subprocess: Optional[GeometryRuntimeSubprocess] = None
        self._remote_provider: Optional[GeometryRuntimeProvider] = None
        self._rosout_bridge = None
        self._external_debug_observer: Optional[ExternalGeometryDebugObserver] = None
        self._external_debug_frontier_payload: Optional[Dict[str, object]] = None
        self._exploration_loop_lock = threading.RLock()
        self._exploration_delivery_context = threading.local()
        self._exploration_update_subscribers: Dict[int, Callable[[ExplorationState], None]] = {}
        self._remote_exploration_subscription_id: Optional[int] = None
        self._next_exploration_subscription_id = 1
        self._external_exploration_request: Optional[ExploreAreaRequest] = None
        self._external_exploration_started_at_monotonic: Optional[float] = None
        self._external_exploration_started_at_iso: Optional[str] = None
        self._external_exploration_deadline_monotonic: Optional[float] = None
        self._external_exploration_database_path: Optional[str] = None
        self._external_exploration_version_id: Optional[str] = None
        self._external_exploration_terminal_state: Optional[ExplorationState] = None
        self._external_exploration_log_offset: Optional[int] = None
        self._external_exploration_last_log_scan_at = 0.0
        self._live_mapping_diagnostics_cache: Optional[Dict[str, object]] = None
        self._live_mapping_diagnostics_cache_at = 0.0
        self._external_geometry_session_mode: Optional[str] = None
        self.providers = PlatformGeometryProviderBundle(self)

    def _ensure_subscription_delivery_state(self) -> threading.Condition:
        """Lazily support tests and compatibility shims built via object.__new__."""

        condition = getattr(self, "_subscription_delivery_condition", None)
        if condition is not None:
            return condition
        bootstrap_lock = getattr(
            self,
            "_provider_attach_lock",
            getattr(self, "_runtime_state_lock", threading.RLock()),
        )
        with bootstrap_lock:
            condition = getattr(
                self,
                "_subscription_delivery_condition",
                None,
            )
            if condition is None:
                condition = threading.Condition()
                self._subscription_delivery_condition = condition
                self._subscription_delivery_tickets = {}
                self._next_subscription_delivery_ticket_id = 1
        return condition

    def _new_subscription_delivery_ticket(
        self,
        *,
        kind: str,
        subscription_id: Optional[int],
        callback: Callable[..., object],
        provider: Optional[object],
        provider_generation: Optional[int] = None,
    ) -> _SubscriptionDeliveryTicket:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            ticket_id = int(self._next_subscription_delivery_ticket_id)
            self._next_subscription_delivery_ticket_id = ticket_id + 1
            ticket = _SubscriptionDeliveryTicket(
                ticket_id=ticket_id,
                kind=str(kind),
                subscription_id=(
                    int(subscription_id)
                    if subscription_id is not None
                    else None
                ),
                callback=callback,
                provider=provider,
                provider_generation=(
                    provider_generation
                    if provider_generation is not None
                    else (
                        _remote_connection_generation(provider)
                        if provider is not None
                        else None
                    )
                ),
            )
            self._subscription_delivery_tickets[ticket_id] = ticket
            return ticket

    def _publish_subscription_delivery_ticket(
        self,
        ticket: _SubscriptionDeliveryTicket,
        *,
        subscription_id: Optional[int] = None,
    ) -> bool:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            if (
                ticket.invalidated
                or self._subscription_delivery_tickets.get(ticket.ticket_id)
                is not ticket
            ):
                return False
            if subscription_id is not None:
                ticket.subscription_id = int(subscription_id)
            ticket.published = True
            condition.notify_all()
            return True

    def _invalidate_subscription_delivery_tickets_locked(
        self,
        *,
        kind: Optional[str] = None,
        subscription_id: Optional[int] = None,
        provider: object = _ANY_DELIVERY_OWNER,
    ) -> Tuple[_SubscriptionDeliveryTicket, ...]:
        """Invalidate matching tickets while the caller owns runtime locks."""

        condition = self._ensure_subscription_delivery_state()
        invalidated = []
        with condition:
            for ticket_id, ticket in tuple(
                self._subscription_delivery_tickets.items()
            ):
                if kind is not None and ticket.kind != kind:
                    continue
                if (
                    subscription_id is not None
                    and ticket.subscription_id != int(subscription_id)
                ):
                    continue
                if provider is not _ANY_DELIVERY_OWNER and ticket.provider is not provider:
                    continue
                ticket.invalidated = True
                ticket.published = False
                self._subscription_delivery_tickets.pop(ticket_id, None)
                invalidated.append(ticket)
            condition.notify_all()
        return tuple(invalidated)

    def _subscription_delivery_identity_is_current_locked(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> bool:
        if bool(getattr(self, "_provider_attach_stopping", False)):
            return False
        if (
            ticket.provider is not None
            and getattr(self, "_remote_provider", None) is not ticket.provider
        ):
            return False
        if (
            ticket.provider is not None
            and _remote_connection_generation(ticket.provider)
            != ticket.provider_generation
        ):
            return False
        subscription_id = ticket.subscription_id
        if ticket.kind == "navigation":
            if subscription_id is None:
                return True
            with self._runtime_state_lock:
                return bool(
                    self._navigation_update_subscribers.get(subscription_id)
                    is ticket.callback
                )
        if ticket.kind == "exploration":
            if subscription_id is None:
                return True
            with self._exploration_loop_lock:
                return bool(
                    self._exploration_update_subscribers.get(subscription_id)
                    is ticket.callback
                )
        if ticket.kind == "exploration_source":
            if subscription_id is None:
                return False
            with self._exploration_loop_lock:
                return bool(
                    self._remote_exploration_subscription_id
                    == subscription_id
                    and self._exploration_update_subscribers
                )
        if subscription_id is None:
            return True
        if ticket.kind == "localization":
            return bool(
                self._localization_update_subscribers.get(subscription_id)
                is ticket.callback
            )
        if ticket.kind == "map":
            return bool(
                self._map_update_subscribers.get(subscription_id)
                is ticket.callback
            )
        return False

    def _begin_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        condition = self._ensure_subscription_delivery_state()
        with provider_attach_lock:
            if not self._subscription_delivery_identity_is_current_locked(
                ticket
            ):
                return False
            with condition:
                if (
                    ticket.invalidated
                    or not ticket.published
                    or self._subscription_delivery_tickets.get(
                        ticket.ticket_id
                    )
                    is not ticket
                ):
                    return False
                thread_id = threading.get_ident()
                ticket.active_count += 1
                ticket.active_threads[thread_id] = (
                    ticket.active_threads.get(thread_id, 0) + 1
                )
                return True

    def _finish_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> None:
        condition = self._ensure_subscription_delivery_state()
        thread_id = threading.get_ident()
        with condition:
            ticket.active_count = max(0, ticket.active_count - 1)
            thread_count = ticket.active_threads.get(thread_id, 0)
            if thread_count <= 1:
                ticket.active_threads.pop(thread_id, None)
            else:
                ticket.active_threads[thread_id] = thread_count - 1
            condition.notify_all()

    def _retire_subscription_delivery_ticket(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> None:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            ticket.invalidated = True
            ticket.published = False
            if (
                self._subscription_delivery_tickets.get(ticket.ticket_id)
                is ticket
            ):
                self._subscription_delivery_tickets.pop(ticket.ticket_id, None)
            condition.notify_all()

    def _invoke_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
        *args,
        retire: bool = False,
        **kwargs,
    ):
        started = self._begin_subscription_delivery(ticket)
        if not started:
            if retire:
                self._retire_subscription_delivery_ticket(ticket)
            return None
        try:
            return ticket.callback(*args, **kwargs)
        finally:
            self._finish_subscription_delivery(ticket)
            if retire:
                self._retire_subscription_delivery_ticket(ticket)

    def _wait_for_subscription_deliveries(
        self,
        tickets: Tuple[_SubscriptionDeliveryTicket, ...],
    ) -> None:
        if not tickets:
            return
        condition = self._ensure_subscription_delivery_state()
        current_thread_id = threading.get_ident()
        with condition:
            while any(
                ticket.active_count
                > ticket.active_threads.get(current_thread_id, 0)
                for ticket in tickets
            ):
                condition.wait()

    def start(self) -> None:
        """启动平台几何运行时。"""

        with self._runtime_state_lock:
            if self._runtime_started:
                return
            self._provider_attach_epoch += 1
            self._provider_attach_stopping = False
            self._runtime_started = True
            self._runtime_stop_event.clear()
            self._runtime_diagnostics = []
            self._managed_processes = {}
            self._map_stream_tracker = {}
        if not self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时或几何 profile 已禁用。")
            return
        try:
            self._set_runtime_state("preflight", "正在检查 nuwax-geometry-runtime 二进制、gRPC 端口与 LCM 通道配置。")
            preflight_errors = self._run_preflight_checks()
            if preflight_errors:
                message = "平台几何运行时预检失败：" + "；".join(preflight_errors)
                self._set_runtime_state("failed", message, blocked=True)
                # Phase 3 目标态：不再 raise，平台几何运行时失败交由 monitor 与 web 仪表呈现。
                return
            self._set_runtime_state("starting_external", "正在挂接 canonical nuwax-geometry-runtime 并建立 gRPC 连接。")
            self._start_geometry_runtime_subprocess()
            self._ensure_external_geometry_observers_started()
            self._set_runtime_state(
                "starting_external",
                "已挂接 canonical nuwax-geometry-runtime，等待 gRPC HealthCheck/READY 与 LCM 通道。",
            )
            self._start_monitor_thread()
        except Exception as exc:
            self._record_runtime_diagnostic("geometry_start", exc)
            self._set_runtime_state("failed", "平台几何运行时启动失败：%s" % exc, blocked=True)
            self._stop_managed_processes()
            self._live_geometry_start_attempted = False
            with self._runtime_state_lock:
                self._runtime_started = False
            # Phase 3 目标态：不再 raise，统一交给上层显式查询 /api/geometry/status。

    def _get_motion_provider(self) -> Optional[MotionControl]:
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, MotionControl):
            return providers
        return None

    def get_geometry_runtime_provider(self) -> Optional["GeometryRuntimeProvider"]:
        """返回当前 Rust gRPC 远程 provider，未连/未启时返回 None。

        生命周期说明：仅当 ``start()`` 成功完成 ``_start_geometry_runtime_subprocess`` 后
        才会有非 None 实例；调用方（gateway runtime）必须按 §13.3.5 顺序在
        ``PlatformGeometryRuntime.start()`` 之后再使用。
        """
        with self._runtime_state_lock:
            provider = self._remote_provider
        if provider is None:
            return None
        connected = getattr(provider, "is_connected", None)
        if callable(connected):
            connected = connected()
        if not bool(connected):
            return None
        return provider

    def notify_cmd_vel_sink_ready(
        self,
        ready: bool,
        generation: Optional[int] = None,
    ) -> None:
        """Rework-17:把 sink ready 状态跨进程写回 Rust handle（best-effort）。

        内部通过当前 `_remote_provider` 调 `SetCmdVelSinkReady` gRPC RPC。
        provider 未连接 / RPC 失败均吞掉异常记 diagnostic,允许 caller
        （gateway runtime `_notify_geometry_cmd_vel_sink_ready`）继续走
        fallback 路径,不抛错。
        """
        provider = self.get_geometry_runtime_provider()
        if provider is None:
            return
        generation_value = int(generation or 0)
        if ready and generation_value <= 0:
            self._record_runtime_diagnostic(
                "geometry_cmd_vel_sink_ready",
                ValueError("ready=true requires a positive motion sink generation"),
            )
            return
        if generation_value <= 0:
            generation_value = 1
        try:
            provider.notify_cmd_vel_sink_ready(
                bool(ready),
                generation=generation_value,
            )
        except Exception as exc:  # noqa: BLE001
            self._record_runtime_diagnostic("geometry_cmd_vel_sink_ready", exc)

    def stop(self) -> None:
        """停止平台几何运行时。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        with provider_attach_lock:
            with self._runtime_state_lock:
                self._provider_attach_epoch += 1
                self._provider_attach_stopping = True
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked()
                )
        self._set_runtime_state("stopping", "正在停止平台几何侧车进程。")
        self._runtime_stop_event.set()
        # Phase 3 目标态：cmd_vel 桥已下线，平台内建速度桥不再启动/停止
        if self._external_debug_observer is not None:
            try:
                self._external_debug_observer.stop()
            except Exception:
                pass
            self._external_debug_observer = None
        try:
            self.stop_platform_exploration()
        except Exception:
            LOGGER.exception("停止 native exploration 失败；继续关闭几何运行时。")
        thread = self._runtime_monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._stop_managed_processes()
        self._wait_for_subscription_deliveries(delivery_tickets)
        self._live_geometry_start_attempted = False
        with self._runtime_state_lock:
            self._runtime_started = False
            self._runtime_monitor_thread = None
        if self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时已停止。")
        else:
            self._set_runtime_state("disabled", "平台几何运行时或几何 profile 已禁用。")

    def _set_runtime_state(self, phase: str, message: str, *, blocked: bool = False) -> None:
        with self._runtime_state_lock:
            self._runtime_phase = phase
            self._runtime_message = message
            self._runtime_blocked = bool(blocked)

    def _record_runtime_diagnostic(self, code: str, error: object) -> None:
        payload = {
            "timestamp": _utc_now_iso(),
            "code": str(code),
            "message": str(error),
        }
        with self._runtime_state_lock:
            self._runtime_diagnostics.append(payload)
            self._runtime_diagnostics = self._runtime_diagnostics[-20:]

    def _repo_root(self) -> Path:
        return Path(__file__).resolve().parents[2]

    def _geometry_script(self, name: str) -> Path:
        return Path(__file__).resolve().parent / name

    def _geometry_log_dir(self) -> Path:
        return self._repo_root() / "runtime_data" / "geometry" / "logs"

    def _geometry_stack(self) -> str:
        # 与 contracts/geometry_runtime.py 的 GEOMETRY_STACK_NAME 对齐。
        return _GEOMETRY_STACK_NAME

    def _uses_external_geometry_stack(self) -> bool:
        # 平台几何运行时由 nuwax-geometry-runtime 子进程承担。手动
        # mapping/explore/localization 仍用
        # `start_external_mapping` / `_start_external_script_exploration` / `_launch_external_localization`
        # 走外部脚本，但它们不再受此标志门控（启动路径已显式使用 go2.sh 命令构造）。
        # 保留方法名是为了兼容历史 caller 的导入和诊断信息，逻辑上始终返回 False。
        return False

    def _removed_internal_stack_message(self) -> str:
        return "当前运行时使用 nuwax-geometry-runtime + gRPC + LCM 链路。"

    def _external_process_name(self) -> str:
        name = str(getattr(self.config.external, "process_name", "") or "").strip()
        return name or "nuwax-geometry-runtime"

    def _external_log_name(self) -> str:
        log_name = str(getattr(self.config.external, "log_name", "") or "").strip()
        return log_name or "nuwax_geometry_runtime.log"

    def _managed_process_identity(self, name: str) -> Tuple[str, str]:
        if name == self._external_process_name():
            return "nuwax-geometry-external", "统一几何服务"
        if name == "external_odom_tf_bridge":
            return "nuwax-odom-tf-bridge", "里程计 TF 桥"
        safe = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "-" for ch in str(name or "geometry")).strip("-_")
        return f"nuwax-{safe or 'geometry'}", "项目几何进程"

    def _external_script_path(self) -> Path:
        # R-N13 修复:从 `script_path`(ExternalGeometryStackConfig 中不存在)
        # 改为 `binary_path`(实存在,settings.py:887),9 处调用面全获正确路径
        # 与 `_launch_runtime_binary()` 行为对齐。
        return Path(str(getattr(self.config.external, "binary_path", "") or "")).expanduser()

    def _external_args(self) -> List[str]:
        raw_args = getattr(self.config.external, "args", ())
        if isinstance(raw_args, str):
            return shlex.split(raw_args)
        return [str(item) for item in raw_args if str(item or "").strip()]

    # go2.sh mode 标志集合
    _GO2_MODE_FLAGS: frozenset = frozenset({"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"})

    def _build_external_geometry_command(self) -> List[str]:
        script_path = self._external_script_path()
        args = self._external_args()
        # 不再默认添加 --mapping：mapping 是手动建图专用，平台自动场景不启动。
        # 如果 go2.sh 且未配置 mode，让 go2.sh 自己报错退出(code=2)。
        return [str(script_path), *args]

    def _build_external_exploration_command(self, request: Optional[ExploreAreaRequest] = None) -> List[str]:
        mode_flags = {"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"}
        passthrough_args = [arg for arg in self._external_args() if arg not in mode_flags]
        db_path = self._external_exploration_database_path_for_request(request)
        return [str(self._external_script_path()), "--explore", *passthrough_args, str(db_path)]

    def _build_external_mapping_command(self, *, map_name: Optional[str] = None) -> List[str]:
        mode_flags = {"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"}
        passthrough_args = [arg for arg in self._external_args() if arg not in mode_flags]
        request = ExploreAreaRequest(
            request_id="manual_mapping",
            map_name=str(map_name or "").strip() or None,
            strategy="manual",
        )
        db_path = self._external_exploration_database_path_for_request(request)
        command: List[str] = [str(self._external_script_path()), "--mapping", *passthrough_args, str(db_path)]
        # 第 5.3 task：boundary JSON 写入后由 start_external_mapping 拼接 --boundary
        boundary_path = self._maybe_persist_boundary_for_external_command(
            command=command,
            session="mapping",
        )
        if boundary_path is not None:
            command = [*command, "--boundary", str(boundary_path)]
        return command

    def _external_exploration_database_path_for_request(self, request: Optional[ExploreAreaRequest]) -> Path:
        map_name = str(getattr(request, "map_name", "") or "").strip() if request is not None else ""
        if map_name:
            version_id = self._next_exploration_map_version_id(map_name)
            path = self._version_mapping_database_path(map_name, version_id)
            path.parent.mkdir(parents=True, exist_ok=True)
            with self._exploration_loop_lock:
                self._external_exploration_version_id = version_id
            return path
        return self._default_external_exploration_database_path()

    def plan_external_exploration(self, request: ExploreAreaRequest) -> Dict[str, object]:
        """返回无副作用的 geometry-runtime gRPC 探索计划。"""

        stop_conditions = dict(request.stop_conditions or {})
        request_metadata = dict(request.metadata or {})
        active_version = getattr(self, "_active_map_version", None)
        version_id = str(
            request_metadata.get("map_version_id")
            or (active_version.version_id if active_version is not None else "")
        ).strip() or None
        return {
            "exploration_launch_mode": "nuwax_geometry_runtime",
            "exploration_transport": "grpc",
            "exploration_script": None,
            "exploration_command": None,
            "exploration_database_path": None,
            "exploration_version_id": version_id,
            "request_id": request.request_id,
            "strategy": str(request.strategy or "wavefront"),
            "arbitration": str(
                request_metadata.get("arbitration")
                or stop_conditions.get("arbitration")
                or "prefer_wavefront"
            ),
        }

    def get_exploration_pinned_version_id(self) -> Optional[str]:
        """返回当前探索会话绑定的 mapver_*，保存版本时应写入同一路径。"""

        context = getattr(self, "_mapping_session_context", None)
        if isinstance(context, dict):
            version_id = str(context.get("map_version_id") or "").strip()
            if version_id:
                return version_id
        with self._exploration_loop_lock:
            return str(self._external_exploration_version_id or "").strip() or None

    def get_mapping_session_context(self) -> Optional[Dict[str, object]]:
        lock = getattr(self, "_mapping_lifecycle_lock", None)
        if lock is None:
            return None
        with lock:
            context = getattr(self, "_mapping_session_context", None)
            return self._clone_metadata(context) if isinstance(context, dict) else None

    def clear_exploration_pinned_version_context(self) -> None:
        with self._exploration_loop_lock:
            self._external_exploration_version_id = None
            self._external_exploration_database_path = None

    def clear_external_exploration_state(self, *, reason: str = "") -> None:
        del reason
        with self._exploration_loop_lock:
            self._external_exploration_request = None
            self._external_exploration_started_at_monotonic = None
            self._external_exploration_started_at_iso = None
            self._external_exploration_deadline_monotonic = None
            self._external_exploration_database_path = None
            self._external_exploration_version_id = None
            self._external_exploration_terminal_state = None
            self._external_exploration_log_offset = None
            self._external_debug_frontier_payload = None
            self._external_geometry_session_mode = None

    def _next_exploration_map_version_id(self, map_name: str) -> str:
        versions_root = self._safe_map_dir(map_name) / "versions"
        latest_revision = 0
        if versions_root.exists():
            for version_dir in versions_root.iterdir():
                if not version_dir.is_dir() or not (version_dir / "version.json").exists():
                    continue
                name = version_dir.name
                if not name.startswith("mapver_"):
                    continue
                try:
                    latest_revision = max(latest_revision, int(name.split("_", 1)[1]))
                except (IndexError, ValueError):
                    continue
        return "mapver_%06d" % (latest_revision + 1)

    def _default_external_exploration_database_path(self) -> Path:
        raw_path = (os.environ.get("NUWAX_MAPPING_DATABASE_PATH") or "").strip()
        if raw_path:
            path = Path(raw_path).expanduser()
        else:
            path = Path(self.config.mapping.database_root).expanduser() / "live_mapping.db"
        if not path.is_absolute():
            path = self._repo_root() / path
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    @staticmethod
    def _env_truthy(*names: str, default: str = "false") -> bool:
        value = default
        for name in names:
            candidate = os.environ.get(name)
            if candidate is not None:
                value = candidate
                break
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def _external_localization_uses_static_map_yaml(self) -> bool:
        # GO2 几何栈 localize_only/restart_map/slam_mode 模式已下线；legacy GO2_NAV_* keys are deprecated.
        return self._env_truthy("NUWAX_NAV2_USE_STATIC_MAP_YAML")

    @staticmethod
    def _command_uses_external_exploration(command: List[str]) -> bool:
        return any(str(part) in {"-e", "--explore", "--exploration"} for part in command)

    def _external_geometry_process_is_exploration(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_exploration(process.command)
        )

    @staticmethod
    def _command_uses_external_mapping(command: List[str]) -> bool:
        return any(str(part) in {"-m", "--new", "--mapping"} for part in command)

    def _external_geometry_process_is_mapping(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_mapping(process.command)
        )

    def _external_geometry_stack_is_running(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(process is not None and process.is_running())

    @staticmethod
    def _command_uses_external_localization(command: List[str]) -> bool:
        return any(str(part) in {"-l", "--loc", "--localization"} for part in command)

    def _command_has_mode_flag(self, command: List[str]) -> bool:
        """检查命令是否包含任何 go2.sh mode 标志（explore/localization/mapping）。"""
        return any(str(part) in self._GO2_MODE_FLAGS for part in command)

    def _external_geometry_process_is_localization(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_localization(process.command)
        )

    def _external_input_bridge_enabled(self) -> bool:
        return bool(getattr(self.config.external, "input_bridge_enabled", False))

    def _external_point_cloud_bridge_enabled(self) -> bool:
        return self._external_input_bridge_enabled() or bool(
            getattr(self.config.external, "point_cloud_bridge_enabled", False)
        )

    def _platform_odom_tf_bridge_enabled(self) -> bool:
        return self._external_input_bridge_enabled() or bool(
            getattr(self.config.external, "platform_odom_tf_bridge_enabled", False)
        )

    def _platform_cmd_vel_bridge_enabled(self) -> bool:
        return bool(getattr(self.config.external, "platform_cmd_vel_bridge_enabled", False))

    def _run_preflight_checks(self) -> List[str]:
        # 目标态：主预检检查 nuwax-geometry-runtime 子进程可达性；
        # 旧 ROS2 launch / odom_tf_bridge.sh / ros2 命令探测已全部下线。
        return self._run_geometry_runtime_preflight_checks()

    def _run_geometry_runtime_preflight_checks(self) -> List[str]:
        """平台几何运行时预检：nuwax-geometry-runtime 二进制 + gRPC 端口 + LCM 通道配置。

        返回错误列表；空列表表示预检通过。
        """
        import socket

        errors: List[str] = []
        runtime_binary = str(
            getattr(self.config, "runtime_binary", "")
            or getattr(self.config.external, "binary_path", "")
            or ""
        ).strip()
        if runtime_binary:
            binary_path = Path(runtime_binary).expanduser()
            if not binary_path.exists():
                errors.append("nuwax-geometry-runtime 二进制不存在 %s" % binary_path)
            elif not binary_path.is_file():
                errors.append("nuwax-geometry-runtime 二进制不是文件 %s" % binary_path)
            elif not os.access(binary_path, os.X_OK):
                errors.append("nuwax-geometry-runtime 二进制不可执行 %s" % binary_path)
        else:
            LOGGER.info(
                "nuwax-geometry-runtime binary 未配置（NUWAX_GEOMETRY_RUNTIME_BINARY）；"
                "Python 只挂接由 scripts/systemd/start_geometry_runtime.sh 管理的进程。"
            )
        runtime_port = int(
            getattr(self.config, "runtime_port", 0)
            or getattr(self.config.external, "port", 0)
            or 0
        )
        # 仅当显式配置了非零端口时探测；Python 后续只 attach，不拥有端口进程。
        if runtime_port > 0:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(0.5)
                    connect_result = sock.connect_ex(("127.0.0.1", runtime_port))
                    if connect_result == 0:
                        LOGGER.info(
                            "nuwax-geometry-runtime gRPC 端口 %s 已监听，"
                            "将 attach canonical runtime；attach 失败时 fail-closed。",
                            runtime_port,
                        )
            except OSError as exc:
                errors.append(
                    "探测 nuwax-geometry-runtime gRPC 端口 %s 失败：%s" % (runtime_port, exc)
                )
        ready_file = str(
            getattr(self.config, "runtime_ready_file", "")
            or getattr(self.config.external, "ready_file", "")
            or ""
        ).strip()
        if not ready_file:
            errors.append("缺少 nuwax-geometry-runtime ready_file 配置 NUWAX_GEOMETRY_RUNTIME_READY_FILE")
        lcm_url = str(getattr(self.config, "lcm_url", "") or "").strip()
        lcm_backend = str(getattr(self.config, "lcm_backend", "") or "").strip()
        if not lcm_url:
            errors.append("缺少 LCM url 配置 NUWAX_LCM_URL（nuwax-geometry-runtime 启动依赖）")
        if lcm_backend and lcm_backend not in _CANONICAL_LCM_BACKENDS:
            errors.append(
                "LCM backend 配置 %s 非法；只允许 real-lcm。" % lcm_backend
            )
        required_frames = {
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
        }
        for name, value in required_frames.items():
            if not str(value or "").strip():
                errors.append("缺少 TF frame 配置 %s" % name)
        return errors

    def _start_required_processes(self) -> None:
        # 目标态：平台几何运行时启动链路已切到 `_start_geometry_runtime_subprocess()` +
        # `_ensure_external_geometry_observers_started()`；旧 ROS2 odom_tf_bridge / cmd_vel_bridge /
        # 外部 SLAM/Nav2 脚本兼容启动入口已下线，保留本方法为 no-op shim 以兼容历史 caller 的引用。
        LOGGER.debug(
            "_start_required_processes 已下线为目标态 no-op；请改用 _start_geometry_runtime_subprocess。"
        )
        return None

    def _start_monitor_thread(self) -> None:
        if self._runtime_monitor_thread is not None and self._runtime_monitor_thread.is_alive():
            return
        self._runtime_monitor_thread = threading.Thread(
            target=self._monitor_runtime_loop,
            name="nuwax_geometry_runtime_monitor",
            daemon=True,
        )
        self._runtime_monitor_thread.start()

    def _build_launch_prefix(self) -> List[str]:
        # 目标态：ROS2 bash 前置脚本已下线（共享 nuwax-geometry-runtime 子进程内置 LCM 通道，无需环境探测）。
        # 保留方法为 no-op shim 以兼容历史 caller；platform 启动走 _start_geometry_runtime_subprocess。
        return []

    def _build_odom_tf_bridge_command(self) -> List[str]:
        # 目标态：odom_tf_bridge.py + odom/cmd_vel 兼容桥已下线（外部 SLAM/Nav2 不再
        # 通过独立 Python 进程转译 ROS2 话题；nuwax-geometry-runtime 子进程内置 LCM channel
        # 处理）。保留方法为 no-op shim 以兼容历史 caller。
        return []

    def _start_managed_process(self, name: str, command: List[str], log_name: str, *, restart: bool = False) -> None:
        current = self._managed_processes.get(name)
        if name == self._external_process_name():
            with self._exploration_loop_lock:
                exploration_active = (
                    self._external_exploration_request is not None
                    and self._external_exploration_terminal_state is None
                )
            if (
                exploration_active
                and current is not None
                and current.is_running()
                and self._command_uses_external_exploration(current.command)
                and not self._command_uses_external_exploration(command)
            ):
                LOGGER.info(
                    "外部探索运行中，忽略非探索模式进程切换请求。current=%s incoming=%s",
                    current.command,
                    list(command),
                )
                return
        if current is not None and current.is_running():
            if current.command == list(command):
                return
            current.stop(
                timeout_sec=12.0,
                graceful_launch_shutdown=(name == self._external_process_name()),
                graceful_timeout_sec=15.0,
            )
            time.sleep(3.0)
            restart = True
        mirror_main_log = name == self._external_process_name()
        process = current or _ManagedProcess(
            name=name,
            command=list(command),
            log_path=self._geometry_log_dir() / log_name,
            cwd=self._repo_root(),
            mirror_main_log=mirror_main_log,
        )
        process_title, display_name = self._managed_process_identity(name)
        process.command = list(command)
        process.process_title = process_title
        process.display_name = display_name
        process.mirror_main_log = mirror_main_log
        process.start(restart=restart)
        if mirror_main_log:
            LOGGER.info(
                "外部几何脚本日志已同步到主日志：process=%s main_log=%s sidecar=%s",
                name,
                _project_log_file(),
                process.log_path,
            )
        self._managed_processes[name] = process

    def _stop_managed_processes(self) -> None:
        for process in list(self._managed_processes.values()):
            process.stop()
        if self._external_debug_observer is not None:
            try:
                self._external_debug_observer.stop()
            except Exception:
                pass
            self._external_debug_observer = None
        if self._geometry_runtime_subprocess is not None:
            self._geometry_runtime_subprocess.stop()
            self._geometry_runtime_subprocess = None
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        navigation_subscription_ids = ()
        exploration_subscription_id = None
        with provider_attach_lock:
            with self._exploration_loop_lock:
                remote_provider = self._remote_provider
                if remote_provider is not None:
                    delivery_tickets = (
                        self._invalidate_subscription_delivery_tickets_locked(
                            provider=remote_provider,
                        )
                    )
                self._remote_provider = None
                exploration_subscription_id = (
                    self._remote_exploration_subscription_id
                )
                self._remote_exploration_subscription_id = None
            with self._runtime_state_lock:
                navigation_subscription_ids = tuple(
                    self._remote_navigation_subscription_ids.values()
                )
                self._remote_navigation_subscription_ids = {}
            stream_handles = tuple(
                self._remote_localization_subscription_handles.values()
            ) + tuple(self._remote_map_subscription_handles.values())
            self._remote_localization_subscription_handles = {}
            self._remote_map_subscription_handles = {}
        if remote_provider is not None:
            if exploration_subscription_id is not None:
                try:
                    remote_provider.unsubscribe_exploration_updates(
                        exploration_subscription_id
                    )
                except Exception:
                    LOGGER.exception(
                        "停止 geometry exploration subscription 失败。"
                    )
            for subscription_id in reversed(navigation_subscription_ids):
                try:
                    remote_provider.unsubscribe_navigation_updates(
                        subscription_id
                    )
                except Exception:
                    LOGGER.exception(
                        "停止 geometry navigation subscription=%s 失败。",
                        subscription_id,
                    )
        for handle in stream_handles:
            _cancel_remote_stream_handle(
                handle,
                stream_name="geometry data",
            )
        self._wait_for_subscription_deliveries(delivery_tickets)
        if remote_provider is not None:
            remote_provider.disconnect()

    def _start_geometry_runtime_subprocess(self) -> None:
        """挂接 canonical Rust geometry runtime 并建立 gRPC 连接。

        兼容旧方法名，但本方法不创建进程。进程唯一 owner 是
        ``scripts/systemd/start_geometry_runtime.sh``；失联时清理本地 handle 并
        fail-closed，等待外部 owner 恢复。
        """
        if self._geometry_runtime_subprocess is not None:
            if self._geometry_runtime_subprocess.is_running:
                current_provider = self.get_geometry_runtime_provider()
                if current_provider is not None:
                    return
                if self._attach_geometry_runtime_provider(
                    self._geometry_runtime_subprocess
                ):
                    return
                LOGGER.warning("已有 geometry sidecar 存活，但 gRPC provider 重新连接失败。")
                return
            LOGGER.warning("canonical geometry runtime 已失联，清理 Python attach handle")
            try:
                self._geometry_runtime_subprocess.stop()
            except Exception as exc:
                LOGGER.warning("zombie bridge.stop 失败（不影响主流程）: %s", exc)
            self._geometry_runtime_subprocess = None
        bridge = GeometryRuntimeSubprocess(
            binary_path=str(getattr(self.config, "runtime_binary", "") or "") or None,
            port=max(1, int(getattr(self.config, "runtime_port", 50051) or 50051)),
        )
        if not bridge.start():
            LOGGER.warning("canonical geometry runtime 挂接失败: %s", bridge.last_error)
            self._record_runtime_diagnostic("geometry_runtime_attach", bridge.last_error or "挂接失败")
            if bridge.is_running is False and getattr(bridge, "_attached", False):
                LOGGER.warning("bridge.attach 残留状态异常，主动清空本地 handle")
                try:
                    bridge.disconnect()
                except Exception:  # pragma: no cover
                    pass
            return
        self._geometry_runtime_subprocess = bridge
        self._attach_geometry_runtime_provider(bridge)

    def _rollback_geometry_runtime_provider_attach(
        self,
        provider,
        *,
        navigation_subscription_ids=(),
        exploration_subscription_id=None,
        stream_handles=(),
    ) -> None:
        """Best-effort rollback for a prepared or partially committed provider."""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        try:
            with provider_attach_lock:
                with self._exploration_loop_lock:
                    delivery_tickets = (
                        self._invalidate_subscription_delivery_tickets_locked(
                            provider=provider,
                        )
                    )
                    if self._remote_provider is provider:
                        self._remote_provider = None
                        self._remote_exploration_subscription_id = None
                        self._remote_localization_subscription_handles = {}
                        self._remote_map_subscription_handles = {}
                        with self._runtime_state_lock:
                            self._remote_navigation_subscription_ids = {}
        except Exception:
            LOGGER.exception("回滚 geometry provider owner 引用失败，继续强制断连。")

        if exploration_subscription_id is not None:
            try:
                unsubscribe = getattr(
                    provider,
                    "unsubscribe_exploration_updates",
                    None,
                )
                if callable(unsubscribe):
                    unsubscribe(exploration_subscription_id)
            except Exception:
                LOGGER.exception("回滚 geometry exploration subscription 失败。")
        for subscription_id in reversed(tuple(navigation_subscription_ids or ())):
            try:
                unsubscribe = getattr(
                    provider,
                    "unsubscribe_navigation_updates",
                    None,
                )
                if callable(unsubscribe):
                    unsubscribe(subscription_id)
            except Exception:
                LOGGER.exception(
                    "回滚 geometry navigation subscription=%s 失败。",
                    subscription_id,
                )
        for handle in reversed(tuple(stream_handles or ())):
            _cancel_remote_stream_handle(
                handle,
                stream_name="geometry attach rollback",
            )
        self._wait_for_subscription_deliveries(delivery_tickets)

        try:
            provider.disconnect()
            return
        except Exception:
            LOGGER.exception("geometry provider disconnect 回滚失败，强制清理 channel。")

        channel = getattr(provider, "_channel", None)
        if channel is not None:
            try:
                channel.close()
            except Exception:
                LOGGER.exception("强制关闭 geometry provider channel 失败。")
        for name, value in (
            ("_channel", None),
            ("_stub", None),
            ("_connected", False),
            ("_stopping", True),
        ):
            try:
                setattr(provider, name, value)
            except Exception:
                LOGGER.exception("强制清理 geometry provider.%s 失败。", name)

    def _attach_geometry_runtime_provider(
        self,
        bridge: GeometryRuntimeSubprocess,
    ) -> bool:
        """为已由 canonical owner 启动的 sidecar 建立 provider 与流订阅。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._runtime_state_lock:
                if (
                    bool(getattr(self, "_provider_attach_stopping", False))
                    or (
                        hasattr(self, "_runtime_started")
                        and not self._runtime_started
                    )
                ):
                    return False
                attach_epoch = int(
                    getattr(self, "_provider_attach_epoch", 0)
                ) + 1
                self._provider_attach_epoch = attach_epoch
                navigation_subscribers = tuple(
                    self._navigation_update_subscribers.items()
                )
                previous_navigation_ids = tuple(
                    self._remote_navigation_subscription_ids.values()
                )
                previous_stream_handles = tuple(
                    getattr(
                        self,
                        "_remote_localization_subscription_handles",
                        {},
                    ).values()
                ) + tuple(
                    getattr(
                        self,
                        "_remote_map_subscription_handles",
                        {},
                    ).values()
                )
            with self._exploration_loop_lock:
                previous_provider = self._remote_provider
                previous_exploration_id = (
                    self._remote_exploration_subscription_id
                )
                needs_exploration_subscription = bool(
                    self._exploration_update_subscribers
                )
            localization_registry = getattr(
                self,
                "_localization_update_subscribers",
                {},
            )
            map_registry = getattr(self, "_map_update_subscribers", {})
            localization_subscribers = (
                tuple(localization_registry.items())
                if localization_registry
                else tuple(
                    (None, callback)
                    for callback in self._pending_localization_callbacks
                )
            )
            map_subscribers = (
                tuple(map_registry.items())
                if map_registry
                else tuple(
                    (None, callback)
                    for callback in self._pending_map_callbacks
                )
            )

        provider = None
        committed = False
        staged_navigation_ids = {}
        staged_exploration_id = None
        staged_localization_handles = {}
        staged_map_handles = {}
        staged_stream_handles = []

        def attach_is_current() -> bool:
            with self._runtime_state_lock:
                return bool(
                    int(getattr(self, "_provider_attach_epoch", 0))
                    == attach_epoch
                    and not bool(
                        getattr(self, "_provider_attach_stopping", False)
                    )
                    and (
                        not hasattr(self, "_runtime_started")
                        or self._runtime_started
                    )
                )

        def only_while_published(
            callback,
            *,
            kind=None,
            subscription_id=None,
        ):
            ticket = self._new_subscription_delivery_ticket(
                kind=str(kind or ""),
                subscription_id=subscription_id,
                callback=callback,
                provider=provider,
            )

            def guarded(*args, **kwargs):
                return self._invoke_subscription_delivery(
                    ticket,
                    *args,
                    **kwargs,
                )

            return guarded, ticket

        try:
            provider = GeometryRuntimeProvider(bridge)
            if not provider.connect():
                LOGGER.warning(
                    "Rust geometry runtime gRPC 连接失败，等待 canonical owner 恢复"
                )
                if (
                    self._geometry_runtime_subprocess is not None
                    and not self._geometry_runtime_subprocess.is_running
                ):
                    LOGGER.warning("bridge 已被 disconnect 清空，下一次入口重新 attach")
                    self._geometry_runtime_subprocess = None
                return False

            with provider_attach_lock:
                if not attach_is_current():
                    return False

            self._inject_static_extrinsics(provider)
            for subscription_id, callback in localization_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="localization",
                    subscription_id=subscription_id,
                )
                handle = provider.subscribe_pose(
                    guarded
                )
                if handle is None:
                    raise RuntimeError("pose subscription preparation failed")
                self._publish_subscription_delivery_ticket(ticket)
                staged_stream_handles.append(handle)
                if subscription_id is not None:
                    staged_localization_handles[subscription_id] = handle
            for subscription_id, callback in map_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="map",
                    subscription_id=subscription_id,
                )
                handle = provider.subscribe_map_updates(
                    guarded
                )
                if handle is None:
                    raise RuntimeError("map subscription preparation failed")
                self._publish_subscription_delivery_ticket(ticket)
                staged_stream_handles.append(handle)
                if subscription_id is not None:
                    staged_map_handles[subscription_id] = handle
            for subscription_id, callback in navigation_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="navigation",
                    subscription_id=subscription_id,
                )
                remote_subscription_id = provider.subscribe_navigation_updates(
                    guarded
                )
                if remote_subscription_id is None:
                    raise RuntimeError(
                        "navigation subscription preparation failed"
                    )
                self._publish_subscription_delivery_ticket(ticket)
                staged_navigation_ids[subscription_id] = int(
                    remote_subscription_id
                )
            if needs_exploration_subscription:
                remote_token = (
                    provider,
                    _remote_connection_generation(provider),
                )
                def deliver_exploration_source(
                    payload,
                    token=remote_token,
                ):
                    return self._handle_remote_exploration_update(
                        payload,
                        remote_token=token,
                    )

                source_ticket = self._new_subscription_delivery_ticket(
                    kind="exploration_source",
                    subscription_id=None,
                    callback=deliver_exploration_source,
                    provider=provider,
                )
                staged_exploration_id = provider.subscribe_exploration_updates(
                    lambda payload: self._invoke_subscription_delivery(
                        source_ticket,
                        payload,
                    )
                )
                if staged_exploration_id is None:
                    raise RuntimeError(
                        "exploration subscription preparation failed"
                    )
                staged_exploration_id = int(staged_exploration_id)
                self._publish_subscription_delivery_ticket(
                    source_ticket,
                    subscription_id=staged_exploration_id,
                )

            with provider_attach_lock:
                with self._runtime_state_lock:
                    subscriptions_unchanged = tuple(
                        self._navigation_update_subscribers.items()
                    ) == navigation_subscribers
                with self._exploration_loop_lock:
                    owner_unchanged = self._remote_provider is previous_provider
                    exploration_need_unchanged = bool(
                        self._exploration_update_subscribers
                    ) == needs_exploration_subscription
                current_localization_registry = getattr(
                    self,
                    "_localization_update_subscribers",
                    {},
                )
                current_map_registry = getattr(
                    self,
                    "_map_update_subscribers",
                    {},
                )
                callbacks_unchanged = bool(
                    (
                        tuple(current_localization_registry.items())
                        == localization_subscribers
                        if localization_registry
                        else tuple(self._pending_localization_callbacks)
                        == tuple(
                            callback
                            for _, callback in localization_subscribers
                        )
                    )
                    and (
                        tuple(current_map_registry.items()) == map_subscribers
                        if map_registry
                        else tuple(self._pending_map_callbacks)
                        == tuple(callback for _, callback in map_subscribers)
                    )
                )
                if not (
                    attach_is_current()
                    and _remote_is_connected(provider)
                    and owner_unchanged
                    and subscriptions_unchanged
                    and exploration_need_unchanged
                    and callbacks_unchanged
                ):
                    return False

                with self._exploration_loop_lock:
                    self._remote_provider = provider
                    self._remote_exploration_subscription_id = (
                        staged_exploration_id
                    )
                    with self._runtime_state_lock:
                        self._remote_navigation_subscription_ids = dict(
                            staged_navigation_ids
                        )
                    self._remote_localization_subscription_handles = dict(
                        staged_localization_handles
                    )
                    self._remote_map_subscription_handles = dict(
                        staged_map_handles
                    )
                committed = True

            if previous_provider is not None:
                self._rollback_geometry_runtime_provider_attach(
                    previous_provider,
                    navigation_subscription_ids=previous_navigation_ids,
                    exploration_subscription_id=previous_exploration_id,
                    stream_handles=previous_stream_handles,
                )
            LOGGER.info("Rust ros-bridge gRPC 连接已建立")
            return True
        except Exception:
            LOGGER.exception("geometry provider attach prepare/commit 失败。")
            return False
        finally:
            if provider is not None and not committed:
                try:
                    self._rollback_geometry_runtime_provider_attach(
                        provider,
                        navigation_subscription_ids=tuple(
                            staged_navigation_ids.values()
                        ),
                        exploration_subscription_id=staged_exploration_id,
                        stream_handles=tuple(staged_stream_handles),
                    )
                except Exception:
                    LOGGER.exception("geometry provider attach rollback helper 异常。")
                    try:
                        with provider_attach_lock:
                            with self._exploration_loop_lock:
                                if self._remote_provider is provider:
                                    self._remote_provider = None
                                    self._remote_exploration_subscription_id = None
                                    self._remote_localization_subscription_handles = {}
                                    self._remote_map_subscription_handles = {}
                                    with self._runtime_state_lock:
                                        self._remote_navigation_subscription_ids = {}
                    except Exception:
                        LOGGER.exception("强制清除半提交 geometry provider owner 失败。")
                    channel = getattr(provider, "_channel", None)
                    if channel is not None:
                        try:
                            channel.close()
                        except Exception:
                            pass
                    for name, value in (
                        ("_channel", None),
                        ("_stub", None),
                        ("_connected", False),
                    ):
                        try:
                            setattr(provider, name, value)
                        except Exception:
                            pass

    def _inject_static_extrinsics(self, provider: "GeometryRuntimeProvider") -> None:
        """装配期：从机器人 CalibrationProvider 取静态外参拓扑，缓存进 GeometryRuntimeProvider。

        平台侧零 import driver：经 self.robot.providers 鸭子类型取 get_sensor_topology，
        将各 SensorPair 转为 Transform(parent=base, child=camera_optical) 静态边，由
        provider 每帧合并进重建的 frame_tree。外参为静态值，此处只取一次。

        无 CalibrationProvider / 无拓扑 / 转换失败时记诊断并跳过（不兜底、不投错链），
        投影侧将因 camera→map 不可达而报错，符合“能用就用、不能用报错”契约。
        """
        from contracts.geometry import Transform

        providers = getattr(self.robot, "providers", None)
        topo_getter = getattr(providers, "get_sensor_topology", None)
        if not callable(topo_getter):
            LOGGER.info("机器人未提供 CalibrationProvider.get_sensor_topology，跳过静态外参注入")
            return
        try:
            topology = topo_getter()
        except Exception as exc:
            self._record_runtime_diagnostic("static_extrinsics_topology", exc)
            return
        if topology is None or not getattr(topology, "pairs", None):
            LOGGER.info("CalibrationProvider 返回空标定拓扑，跳过静态外参注入")
            return

        edges = []
        for pair in topology.pairs:
            try:
                edges.append(
                    Transform(
                        parent_frame_id=pair.parent_frame_id,
                        child_frame_id=pair.child_frame_id,
                        translation=pair.initial_translation,
                        rotation=pair.initial_rotation,
                    )
                )
            except Exception as exc:  # pragma: no cover - 非法 frame_id 等
                self._record_runtime_diagnostic("static_extrinsics_pair", exc)
                continue
        if not edges:
            LOGGER.warning("CalibrationProvider 拓扑未产出有效静态外参边")
            return
        provider.set_static_extrinsics(edges)
        LOGGER.info("已装配 %d 条静态外参边（robot_id=%s）", len(edges), getattr(topology, "robot_id", "?"))

    def _monitor_runtime_loop(self) -> None:
        while not self._runtime_stop_event.wait(_GEOMETRY_MONITOR_INTERVAL_SEC):
            try:
                self._monitor_processes_once()
                self._advance_runtime_state_from_inputs()
            except Exception as exc:
                self._record_runtime_diagnostic("monitor_loop", exc)
                self._set_runtime_state("failed", "平台几何运行时监控失败：%s" % exc, blocked=True)
                # Phase 3 目标态：监控循环遇错仅置 failed，不中断轮询；web 仪表可见即可。

    def _should_suppress_external_process_autorestart(self) -> bool:
        """探索/建图会话或探索完成后保留几何时，禁止 monitor 自动重启 go2.sh。"""

        with self._exploration_loop_lock:
            if self._external_exploration_request is not None:
                return True
            if self._external_exploration_terminal_state is not None:
                return True
            if self._external_geometry_session_mode in {"mapping", "explore"}:
                return True
            if self._external_exploration_database_path:
                return True
        return False

    def _monitor_processes_once(self) -> None:
        for name, process in list(self._managed_processes.items()):
            exit_code = process.poll()
            if exit_code is None or process.is_running():
                continue
            if name == self._external_process_name() and self._should_suppress_external_process_autorestart():
                continue
            if process.restart_count < _GEOMETRY_SIDE_CAR_RESTART_LIMIT:
                self._set_runtime_state("recovering", "%s 进程异常退出，正在尝试恢复。" % name)
                self._record_runtime_diagnostic("process_exit", "%s exit_code=%s\n%s" % (name, exit_code, process.last_error or ""))
                process.start(restart=True)
                continue
            self._set_runtime_state(
                "failed",
                "%s 进程异常退出且恢复次数耗尽，exit_code=%s。" % (name, exit_code),
                blocked=True,
            )
            self._record_runtime_diagnostic("process_failed", "%s exit_code=%s\n%s" % (name, exit_code, process.last_error or ""))

    def _advance_runtime_state_from_inputs(self) -> None:
        # 目标态：runtime ready 由 nuwax-geometry-runtime 子进程存活 + gRPC HealthCheck/READY +
        # LCM 通道订阅状态驱动，不再围绕外部 SLAM/Nav2 脚本、`/map_obstacles`、Nav2 action 推进。
        if not self._runtime_started or not self._is_geometry_profile_enabled():
            return
        bridge_alive = bool(
            self._geometry_runtime_subprocess is not None
            and self._geometry_runtime_subprocess.is_running
        )
        if not bridge_alive:
            self._set_runtime_state(
                "waiting_bridge",
                "等待 nuwax-geometry-runtime 子进程启动。",
            )
            return
        provider_connected = bool(
            self._remote_provider is not None
            and getattr(self._remote_provider, "is_connected", lambda: False)()
        )
        if not provider_connected:
            self._set_runtime_state(
                "waiting_bridge",
                "nuwax-geometry-runtime 子进程已启动，等待 gRPC 连接。",
            )
            return
        health_ok, health_message = self._probe_geometry_runtime_health()
        if not health_ok:
            self._set_runtime_state(
                "waiting_bridge",
                "等待 nuwax-geometry-runtime gRPC HealthCheck.healthy=True：%s"
                % (health_message or "health check pending"),
            )
            return
        live_map_status = self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        self._update_stream_tracker(live_map_status)
        readiness = self._build_mapping_readiness(
            live_map_status=live_map_status,
            tf_localization_status=tf_localization_status,
        )
        if not bool(readiness.get("first_frame_ready")):
            self._set_runtime_state(
                "waiting_map",
                str(readiness.get("message") or "等待 nuwax-geometry-runtime 推送首帧占据栅格。"),
            )
            return
        if bool(readiness.get("streaming_required")) and not bool(readiness.get("streaming_ready")):
            self._set_runtime_state(
                "first_frame",
                str(readiness.get("message") or "nuwax-geometry-runtime 已发布首帧，正在确认连续流式更新。"),
            )
            return
        exploration_state = self.get_platform_exploration_state()
        if getattr(exploration_state.status, "value", exploration_state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }:
            explore_message = exploration_state.message or "平台探索任务运行中（外部脚本由 start_external_mapping 拉起）。"
            self._set_runtime_state("exploring", explore_message)
            return
        if not bool(readiness.get("localization_ready")):
            self._set_runtime_state(
                "streaming",
                str(readiness.get("message") or "地图首帧已就绪，等待 nuwax-geometry-runtime 定位与 TF。"),
            )
            return
        self._set_runtime_state(
            "ready",
            "nuwax-geometry-runtime gRPC HealthCheck、地图 streaming 与 TF 链均已就绪。",
        )

    def _probe_geometry_runtime_health(self) -> Tuple[bool, str]:
        """探测 nuwax-geometry-runtime gRPC HealthCheck.healthy 与 GetStatus 状态。

        返回 (ok, message)：
        - ok=True: bridge 可用，message 为附加诊断（可能为空）
        - ok=False: bridge 不可用，message 给出原因
        """
        provider = self._remote_provider
        if provider is None:
            return False, "GeometryRuntimeProvider 未连接"
        try:
            status = provider.get_status()
        except Exception as exc:  # pragma: no cover - gRPC 健康探测容错
            return False, "GetStatus RPC 失败：%s" % exc
        if not isinstance(status, dict):
            return False, "GetStatus 返回非 dict：%r" % (status,)
        phase = str(status.get("phase") or "").strip()
        # target runtime state machine: BOOTING/STARTING/READY/FAILED
        if phase == "FAILED":
            message = str(status.get("message") or "nuwax-geometry-runtime 上报 FAILED")
            return False, message
        if phase and phase not in {"READY", "RUNNING", "STARTING", "BOOTING", ""}:
            return False, "nuwax-geometry-runtime 未知状态 phase=%s" % phase
        return True, str(status.get("message") or "")

    def _blocking_process_failure(self) -> Optional[str]:
        for name, process in self._managed_processes.items():
            if process.is_running():
                continue
            if name == self._external_process_name() and self._should_suppress_external_process_autorestart():
                continue
            exit_code = process.poll()
            if exit_code is None:
                continue
            if process.restart_count >= _GEOMETRY_SIDE_CAR_RESTART_LIMIT:
                return "%s 进程已退出，exit_code=%s。" % (name, exit_code)
        return None

    def _build_mapping_readiness(
        self,
        *,
        live_map_status: Optional[Dict[str, object]] = None,
        tf_localization_status: Optional[Dict[str, object]] = None,
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        live_map_status = live_map_status or self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = tf_localization_status or self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        if self._uses_external_geometry_stack():
            return self._build_external_geometry_readiness(
                live_map_status=live_map_status,
                tf_localization_status=tf_localization_status,
                odometry_status=odometry_status,
                point_cloud_status=point_cloud_status,
                point_cloud_source=point_cloud_source,
                data_plane_status=data_plane_status,
            )
        return self._build_geometry_runtime_readiness(
            live_map_status=live_map_status,
            tf_localization_status=tf_localization_status,
            odometry_status=odometry_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )

    def _build_geometry_runtime_readiness(
        self,
        *,
        live_map_status: Dict[str, object],
        tf_localization_status: Dict[str, object],
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """目标态 readiness 聚合：nuwax-geometry-runtime gRPC + 观察器 + LCM 通道。

        与 _build_external_geometry_readiness 的差异：
        - 使用 live_map_status.occupancy_grid_ready（观察器首帧） +
          self._map_stream_tracker.streaming（连续流式更新） +
          tf_chain.ready（map→odom→base TF 链）+ bridge alive + HealthCheck.healthy。
        """
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        first_frame_ready = bool(
            live_map_status.get("occupancy_grid_ready")
            and int(live_map_status.get("occupancy_message_count") or 0) > 0
            and live_map_status.get("occupancy_has_publisher") is not False
        )
        localization_process = bool(
            self._external_geometry_process_is_localization()
            or str((self._last_activation_result or {}).get("backend_mode") or "") == "mapping_localization"
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        streaming_required = not localization_process
        map_ready = bool(first_frame_ready and (streaming_ready or not streaming_required))
        localization_ready = bool(map_ready and tf_chain.get("ready"))
        bridge_alive, health_message = self._probe_geometry_runtime_health()
        if not bridge_alive:
            message = "等待 nuwax-geometry-runtime gRPC HealthCheck：%s" % (health_message or "pending")
            waiting_for = "geometry_runtime_health"
        elif not first_frame_ready:
            message = "等待 nuwax-geometry-runtime 推送首帧占据栅格。"
            waiting_for = "geometry_runtime_first_frame"
        elif streaming_required and not streaming_ready:
            message = "nuwax-geometry-runtime 已发布首帧，正在确认连续流式更新。"
            waiting_for = "geometry_runtime_streaming"
        elif not tf_chain.get("ready"):
            message = "等待 nuwax-geometry-runtime 推送 map→odom→base TF 链。"
            waiting_for = "geometry_runtime_tf"
        else:
            message = "nuwax-geometry-runtime gRPC HealthCheck、地图 streaming 与 TF 链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": bridge_alive,
            "input_bridge_ready": bridge_alive,
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "streaming_required": streaming_required,
            "map_ready": map_ready,
            "localization_ready": localization_ready,
            "relocalization_ready": False,
            "waiting_for": waiting_for,
            "message": message,
            "removed_internal_stack": False,
            "geometry_runtime_health": {
                "alive": bridge_alive,
                "message": health_message,
            },
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "odom_tf_bridge_process": None,
        }

    def _build_external_geometry_readiness(
        self,
        *,
        live_map_status: Dict[str, object],
        tf_localization_status: Dict[str, object],
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        process_name = self._external_process_name()
        external_process = self._managed_processes.get(process_name)
        odom_tf_bridge = self._managed_processes.get("external_odom_tf_bridge")
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        process_running = bool(external_process is not None and external_process.is_running())
        input_bridge_enabled = self._external_input_bridge_enabled()
        point_cloud_bridge_enabled = self._external_point_cloud_bridge_enabled()
        input_bridge_ready = True
        if input_bridge_enabled:
            input_bridge_ready = bool(
                odom_tf_bridge is not None
                and odom_tf_bridge.is_running()
                and int(odometry_status.get("published_count") or 0) > 0
                and int(point_cloud_status.get("published_count") or 0) > 0
                and (point_cloud_source.get("timestamp") or data_plane_status.get("started") is not False)
            )
        elif point_cloud_bridge_enabled:
            input_bridge_ready = bool(
                int(point_cloud_status.get("published_count") or 0) > 0
                and (point_cloud_source.get("timestamp") or data_plane_status.get("started") is not False)
            )
        first_frame_ready = bool(
            live_map_status.get("occupancy_grid_ready")
            and int(live_map_status.get("occupancy_message_count") or 0) > 0
            and live_map_status.get("occupancy_has_publisher") is not False
        )
        localization_process = bool(
            self._external_geometry_process_is_localization()
            or str((self._last_activation_result or {}).get("backend_mode") or "") == "mapping_localization"
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        streaming_required = not localization_process
        map_ready = bool(first_frame_ready and (streaming_ready or not streaming_required))
        localization_ready = bool(process_running and input_bridge_ready and map_ready and tf_chain.get("ready"))
        if not process_running:
            message = "外部几何脚本尚未运行。"
            waiting_for = "external_process"
        elif not input_bridge_ready:
            message = (
                "外部几何点云输入桥未就绪；需要平台发布点云输入。"
                if point_cloud_bridge_enabled and not input_bridge_enabled
                else "外部几何输入桥未就绪；需要平台发布 /odom、点云输入和坐标变换。"
            )
            waiting_for = "external_input_bridge"
        elif not first_frame_ready:
            message = "等待外部几何后端发布首帧占据栅格。"
            waiting_for = "external_map"
        elif streaming_required and not streaming_ready:
            message = "外部几何后端已发布首帧，正在确认连续 streaming。"
            waiting_for = "external_map_streaming"
        elif not tf_chain.get("ready"):
            if localization_process:
                message = "外部定位模式已收到首帧地图，等待坐标变换链就绪。"
            else:
                message = "外部占据栅格已 streaming，等待坐标变换链就绪。"
            waiting_for = "external_tf"
        else:
            if localization_process:
                message = "外部定位地图首帧与坐标变换链均已就绪。"
            else:
                message = "外部占据栅格 streaming 与坐标变换链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": bool(process_running and input_bridge_ready),
            "input_bridge_ready": bool(input_bridge_ready),
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "streaming_required": streaming_required,
            "map_ready": map_ready,
            "external_launch_mode": "localization" if localization_process else "mapping_or_exploration",
            "localization_ready": localization_ready,
            "relocalization_ready": localization_ready,
            "waiting_for": waiting_for,
            "message": message,
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "odom_tf_bridge_process": odom_tf_bridge.status() if odom_tf_bridge is not None else None,
            "external_geometry": {
                "stack": self.config.stack,
                "process_name": process_name,
                "script_path": str(self._external_script_path()),
                "args": tuple(self._external_args()),
                "provides_map": bool(getattr(self.config.external, "provides_map", True)),
                "provides_navigation": bool(self.config.external.provides_navigation),
                "provides_tf": bool(getattr(self.config.external, "provides_tf", True)),
                "input_bridge_enabled": input_bridge_enabled,
                "point_cloud_bridge_enabled": point_cloud_bridge_enabled,
                "platform_odom_tf_bridge_enabled": self._platform_odom_tf_bridge_enabled(),
                "platform_cmd_vel_bridge_enabled": self._platform_cmd_vel_bridge_enabled(),
            },
            "external_process": external_process.status() if external_process is not None else None,
        }

    def _build_tf_chain_status(self, *, tf_localization_status: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        tf_localization_status = tf_localization_status or self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        removed_internal_stack = not self._uses_external_geometry_stack()
        if self._uses_external_geometry_stack():
            required_processes = [self._external_process_name()]
            if self._platform_odom_tf_bridge_enabled():
                required_processes.append("external_odom_tf_bridge")
        else:
            required_processes = []
        required_process_names = tuple(required_processes)
        process_status = {
            name: self._managed_processes[name].status()
            for name in required_process_names
            if name in self._managed_processes
        }
        missing = [name for name in required_process_names if name not in self._managed_processes]
        stopped = [name for name, status in process_status.items() if not bool(status.get("running"))]
        pose_ready = bool(tf_localization_status.get("pose_ready") or tf_localization_status.get("pose_available"))
        map_to_base_ready = bool(tf_localization_status.get("map_to_base_ready"))
        chain_ready = bool(not removed_internal_stack and not missing and not stopped and pose_ready and map_to_base_ready)
        return {
            "ready": chain_ready,
            "process_ready": bool(not removed_internal_stack and not missing and not stopped),
            "pose_ready": pose_ready,
            "map_to_base_ready": map_to_base_ready,
            "map_to_odom_ready": bool(tf_localization_status.get("map_to_odom_ready")),
            "odom_to_base_ready": bool(tf_localization_status.get("odom_to_base_ready")),
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "imu_frame": getattr(self.config.frames, "imu_frame", None),
            "required_processes": required_process_names,
            "missing_processes": missing,
            "stopped_processes": stopped,
            "processes": process_status,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "removed_internal_stack": removed_internal_stack,
            "validation": (
                "geometry_runtime_tf_provider_must_resolve_map_to_base_pose"
                if self._uses_external_geometry_stack()
                else "geometry_runtime_tf_chain"
            ),
        }

    def _update_stream_tracker(self, live_map_status: Dict[str, object]) -> None:
        count = int(live_map_status.get("occupancy_message_count") or 0)
        timestamp = _parse_iso_timestamp(live_map_status.get("latest_occupancy_timestamp"))
        received_at = _parse_iso_timestamp(live_map_status.get("latest_occupancy_received_at"))
        metrics = live_map_status.get("occupancy_metrics") if isinstance(live_map_status.get("occupancy_metrics"), dict) else {}
        cost_metrics = live_map_status.get("cost_map_metrics") if isinstance(live_map_status.get("cost_map_metrics"), dict) else {}
        content_hash = str(metrics.get("content_hash") or "")
        costmap_hash = str(cost_metrics.get("content_hash") or "")
        known_count = int(metrics.get("known_cell_count") or 0)
        costmap_known_count = int(cost_metrics.get("known_cell_count") or 0)
        now = datetime.now(timezone.utc)
        previous_count = int(self._map_stream_tracker.get("last_count") or 0)
        previous_costmap_count = int(self._map_stream_tracker.get("last_costmap_count") or 0)
        previous_hash = str(self._map_stream_tracker.get("last_content_hash") or "")
        previous_costmap_hash = str(self._map_stream_tracker.get("last_costmap_hash") or "")
        previous_known_count = int(self._map_stream_tracker.get("last_known_cell_count") or 0)
        previous_costmap_known_count = int(self._map_stream_tracker.get("last_costmap_known_cell_count") or 0)
        first_count = int(self._map_stream_tracker.get("first_count") or count)
        if count > previous_count:
            self._map_stream_tracker["last_count"] = count
            self._map_stream_tracker["last_growth_at"] = now.isoformat()
            if "first_growth_at" not in self._map_stream_tracker:
                self._map_stream_tracker["first_growth_at"] = now.isoformat()
                self._map_stream_tracker["first_count"] = count
        content_changed = bool(content_hash and content_hash != previous_hash)
        known_changed = known_count != previous_known_count
        known_cell_delta = known_count - previous_known_count if previous_hash else known_count
        costmap_delta = costmap_known_count - previous_costmap_known_count if previous_costmap_hash else costmap_known_count
        if count > previous_count and (content_changed or known_changed):
            self._map_stream_tracker["last_content_change_at"] = now.isoformat()
            self._map_stream_tracker["content_revision"] = int(self._map_stream_tracker.get("content_revision") or 0) + 1
        if costmap_hash and (costmap_hash != previous_costmap_hash or costmap_delta != 0):
            self._map_stream_tracker["last_costmap_change_at"] = now.isoformat()
            self._map_stream_tracker["costmap_revision"] = int(self._map_stream_tracker.get("costmap_revision") or 0) + 1
        if content_hash:
            self._map_stream_tracker["last_content_hash"] = content_hash
        if costmap_hash:
            self._map_stream_tracker["last_costmap_hash"] = costmap_hash
        self._map_stream_tracker["last_known_cell_count"] = known_count
        self._map_stream_tracker["last_costmap_known_cell_count"] = costmap_known_count
        self._map_stream_tracker["last_costmap_count"] = int(live_map_status.get("cost_map_message_count") or 0)
        latest_age_sec = None
        if timestamp is not None:
            latest_age_sec = max(0.0, (now - timestamp).total_seconds())
        received_age_sec = None
        if received_at is not None:
            received_age_sec = max(0.0, (now - received_at).total_seconds())
        first_growth_at = _parse_iso_timestamp(self._map_stream_tracker.get("first_growth_at"))
        last_growth_at = _parse_iso_timestamp(self._map_stream_tracker.get("last_growth_at"))
        last_content_change_at = _parse_iso_timestamp(self._map_stream_tracker.get("last_content_change_at"))
        map_fresh_window_sec = max(10.0, float(self.config.mapping.timestamp_quality.max_tf_age_sec) * 20.0, _GEOMETRY_STREAMING_WINDOW_SEC * 3.0)
        source_fresh = latest_age_sec is not None and latest_age_sec <= map_fresh_window_sec
        count_recent = last_growth_at is not None and (now - last_growth_at).total_seconds() <= map_fresh_window_sec
        content_recent = last_content_change_at is not None and (now - last_content_change_at).total_seconds() <= _GEOMETRY_STREAMING_WINDOW_SEC * 2.0
        growth_window = (now - first_growth_at).total_seconds() if first_growth_at is not None else 0.0
        streaming = bool(
            count >= max(first_count + 1, 2)
            and source_fresh
            and count_recent
            and (content_recent or known_count > 0)
            and growth_window <= map_fresh_window_sec * max(1, count - first_count + 1)
        )
        self._map_stream_tracker["latest_age_sec"] = latest_age_sec
        self._map_stream_tracker["received_age_sec"] = received_age_sec
        self._map_stream_tracker["source_timestamp_fresh"] = source_fresh
        self._map_stream_tracker["message_count_recent"] = count_recent
        self._map_stream_tracker["content_recent"] = content_recent
        self._map_stream_tracker["message_count_delta"] = count - previous_count
        self._map_stream_tracker["content_hash"] = content_hash or None
        self._map_stream_tracker["known_cell_count"] = known_count
        self._map_stream_tracker["known_cell_delta"] = known_cell_delta
        self._map_stream_tracker["costmap_message_count_delta"] = int(live_map_status.get("cost_map_message_count") or 0) - previous_costmap_count
        self._map_stream_tracker["costmap_hash"] = costmap_hash or None
        self._map_stream_tracker["costmap_known_cell_count"] = costmap_known_count
        self._map_stream_tracker["costmap_delta"] = costmap_delta
        self._map_stream_tracker["last_message_at"] = received_at.isoformat() if received_at is not None else None
        self._map_stream_tracker["stream_age_ms"] = received_age_sec * 1000.0 if received_age_sec is not None else None
        self._map_stream_tracker["streaming"] = streaming

    def _navigation_backend_policy(self) -> Dict[str, object]:
        """返回当前平台导航后端选择策略。"""
        backend = self._resolve_navigation_backend()
        backend_name = getattr(backend, "provider_name", None)
        is_runtime_provider = bool(backend) and isinstance(backend, GeometryRuntimeProvider)
        selection = "nuwax_geometry_runtime" if is_runtime_provider else "unavailable"
        return {
            "selection": selection,
            "execution_chain": "nuwax_geometry_runtime_grpc" if is_runtime_provider else "nuwax_navigation_only",
            "single_execution_chain": True,
            "selected_provider_name": backend_name,
            "active_backend_provider": backend_name,
            "active_navigation_backend": backend_name,
            "active_backend_kind": "geometry_runtime_provider" if is_runtime_provider else None,
            "message": (
                "当前正式导航执行链为 nuwax-geometry-runtime (Rust gRPC facade)。"
                if is_runtime_provider
                else "当前未绑定 GeometryRuntimeProvider。"
            ),
        }

    def _with_navigation_backend_policy(self, status: Dict[str, object]) -> Dict[str, object]:
        policy = self._navigation_backend_policy()
        return {
            **dict(status or {}),
            "navigation_backend_policy": policy,
            "active_navigation_backend": policy.get("active_backend_provider"),
        }

    def _build_navigation_status(self) -> Dict[str, object]:
        """返回当前导航后端的诊断状态。"""
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and getattr(remote, "is_connected", False):
            try:
                readiness = remote.get_navigation_readiness()
            except Exception as exc:  # pragma: no cover - 防御性兜底
                readiness = {"ready": False, "message": "gRPC 调用失败：%s" % exc}
            status = {
                "phase": "geometry_runtime",
                "message": str(readiness.get("message") or "nuwax-geometry-runtime gRPC 已连接。"),
                "action_name": "/navigate_to_pose",
                "global_costmap_topic": "/global_costmap/costmap",
                "local_costmap_topic": "/local_costmap/costmap",
                "costmap_source": "nuwax_geometry_runtime",
                "action_client_connected": bool(remote.is_available()),
                "lifecycle_ready": bool(readiness.get("ready")),
                "tf_ready": bool(readiness.get("tf_ready")),
                "action_server_ready": bool(readiness.get("action_server_ready")),
                "waiting_for": readiness.get("waiting_for"),
                "ready": bool(readiness.get("ready")),
                "launch_attempted": False,
            }
            return self._with_navigation_backend_policy(status)
        # 未绑定 GeometryRuntimeProvider:返回 nuwax navigation 未就绪占位
        status = {
            "phase": "disabled",
            "message": "nuwax-geometry-runtime 尚未连接；navigation 未就绪。",
            "action_name": "/navigate_to_pose",
            "global_costmap_topic": "/global_costmap/costmap",
            "local_costmap_topic": "/local_costmap/costmap",
            "costmap_source": "nuwax_geometry_runtime",
            "action_client_connected": False,
            "lifecycle_ready": False,
            "launch_attempted": False,
            "ready": False,
            "waiting_for": ["nuwax_geometry_runtime_subprocess"],
        }
        return self._with_navigation_backend_policy(status)

    def get_runtime_status(self) -> Dict[str, object]:
        with self._runtime_state_lock:
            phase = self._runtime_phase
            message = self._runtime_message
            blocked = self._runtime_blocked
            diagnostics = list(self._runtime_diagnostics)
            started = self._runtime_started
        return {
            "phase": phase,
            "state": phase,
            "message": message,
            "blocked": blocked,
            "started": started,
            "live_geometry_start_attempted": self._live_geometry_start_attempted,
            "processes": {name: process.status() for name, process in self._managed_processes.items()},
            "navigation_backend_policy": self._navigation_backend_policy(),
            "map_stream": dict(self._map_stream_tracker),
            "mapping_readiness": self._build_mapping_readiness(),
            "navigation": self._build_navigation_status(),
            "external_geometry": self.config.build_metadata().get("external_geometry"),
            "external_debug_observer": self._external_debug_observer_status(),
            "tf_localization_provider": self._geometry_runtime_status_stub("tf_localization_provider"),
            "diagnostics": diagnostics,
            "mapping_stack": {
                "script": str(
                    self._repo_root()
                    / "scripts"
                    / "systemd"
                    / "start_geometry_runtime.sh"
                ),
                "starts_external_geometry_by_default": False,
                "message": "GO2 几何栈由共享 nuwax-geometry-runtime 承载。",
            },
        }

    def _external_debug_observer_status(self) -> Dict[str, object]:
        observer = self._external_debug_observer
        with self._exploration_loop_lock:
            frontier_payload = self._clone_metadata(self._external_debug_frontier_payload or {})
        return {
            "running": bool(observer is not None and observer.is_running),
            "last_error": observer.last_error if observer is not None else None,
            "frontier_count": frontier_payload.get("frontier_count"),
            "frontier_updated_at": frontier_payload.get("updated_at"),
        }

    def _ensure_live_geometry_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        if not self._uses_external_geometry_stack():
            raise RuntimeError(self._removed_internal_stack_message())
        self._ensure_external_geometry_observers_started()

    def start_external_debug_observers(self) -> None:
        """Start read-only observers for externally launched GO2 map/frontier streams."""

        if not self._is_geometry_profile_enabled() or not self._uses_external_geometry_stack():
            return
        self._ensure_external_geometry_observers_started()

    def _resolve_external_tf_localization_status(
        self,
        *,
        base_stub: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """合成 GO2 几何栈外部 TF 就绪状态。

        Rust bridge GetStatus 不含 pose_ready/map_to_base_ready；探索/建图时 nuwax-geometry-runtime
        通过 /go2/pose_map 与 /tf 提供 map 系位姿，须合并 debug observer 与 gRPC 信号。
        """
        status = dict(base_stub or {})
        map_frame = str(self.config.frames.map_frame or "map").strip().lstrip("/")
        odom_frame = str(self.config.frames.odom_frame or "odom").strip().lstrip("/")
        base_frame = str(self.config.frames.base_frame or "base_link").strip().lstrip("/")
        base_candidates = tuple(
            dict.fromkeys(
                candidate
                for candidate in (
                    base_frame,
                    "base_footprint",
                    "base_link",
                )
                if candidate
            )
        )
        pose_ready = bool(status.get("pose_ready") or status.get("pose_available"))
        map_to_base_ready = bool(status.get("map_to_base_ready"))
        map_to_odom_ready = bool(status.get("map_to_odom_ready"))
        odom_to_base_ready = bool(status.get("odom_to_base_ready"))
        source_kind = status.get("source_kind")

        live_pose = self.get_external_live_pose(max_age_sec=3.0)
        if live_pose is not None:
            frame_id = str(live_pose.frame_id or "").strip().lstrip("/")
            if not frame_id or frame_id == map_frame:
                pose_ready = True
                map_to_base_ready = True
                source_kind = source_kind or "external_pose_topic"

        if not map_to_base_ready and self._remote_provider is not None and self._remote_provider.is_connected:
            bridge_pose = self._remote_provider.get_current_pose()
            if bridge_pose is not None:
                frame_id = str(bridge_pose.frame_id or "").strip().lstrip("/")
                if frame_id == map_frame:
                    pose_ready = True
                    map_to_base_ready = True
                    source_kind = source_kind or "geometry_runtime_pose"
            if not map_to_base_ready:
                frame_tree = self._remote_provider.get_frame_tree()
                if frame_tree is not None:
                    present_frames = {
                        frame_id
                        for transform in frame_tree.transforms
                        for frame_id in (transform.parent_frame_id, transform.child_frame_id)
                    }
                    map_to_odom_ready = map_to_odom_ready or (
                        map_frame in present_frames and odom_frame in present_frames
                    )
                    odom_to_base_ready = odom_to_base_ready or any(
                        candidate in present_frames for candidate in base_candidates
                    )
                    for candidate in base_candidates:
                        if resolve_frame_transform(
                            frame_tree,
                            source_frame_id=map_frame,
                            target_frame_id=candidate,
                        ) is not None:
                            pose_ready = True
                            map_to_base_ready = True
                            source_kind = source_kind or "geometry_runtime_tf_chain"
                            break

        status.update(
            {
                "pose_ready": pose_ready,
                "pose_available": pose_ready,
                "map_to_base_ready": map_to_base_ready,
                "map_to_odom_ready": map_to_odom_ready,
                "odom_to_base_ready": odom_to_base_ready,
                "source_kind": source_kind,
            }
        )
        return status

    def _geometry_runtime_status_stub(self, provider_key: str) -> Dict[str, object]:
        """返回 Rust bridge 替代 Python provider 后的状态占位 dict，保持调用方 key 兼容。"""
        remote_status = {}
        if self._remote_provider is not None:
            try:
                remote_status = self._remote_provider.get_status() or {}
            except Exception:
                remote_status = {}
        stub = {
            "provider": "geometry_runtime",
            "provider_key": provider_key,
            "status": "active" if self._geometry_runtime_handles_input_bridge() else "inactive",
            "geometry_runtime_connected": self._geometry_runtime_handles_input_bridge(),
            **remote_status,
        }
        # live map readiness 读取 Rust mapping diagnostics；observer 不再承载地图真值。
        if provider_key == "live_map_provider":
            mapping_diagnostics = None
            if self._remote_provider is not None:
                try:
                    mapping_diagnostics = self._remote_provider.get_mapping_diagnostics()
                except Exception:
                    mapping_diagnostics = None
            if isinstance(mapping_diagnostics, dict):
                revision = int(
                    mapping_diagnostics.get("last_occupancy_revision")
                    or mapping_diagnostics.get("occupancy_revision")
                    or 0
                )
                message_count = int(mapping_diagnostics.get("pointcloud_count") or 0)
                stub.update({
                    "occupancy_grid_ready": revision > 0,
                    "occupancy_has_publisher": revision > 0,
                    "occupancy_revision": revision,
                    "occupancy_sequence": revision,
                    "occupancy_message_count": message_count,
                    "streaming": bool(mapping_diagnostics.get("ready")),
                    "mapping_diagnostics": dict(mapping_diagnostics),
                })
        if provider_key == "tf_localization_provider":
            stub = self._resolve_external_tf_localization_status(base_stub=stub)
        return stub

    def _geometry_runtime_handles_input_bridge(self) -> bool:
        """Rust bridge 直接订阅 Go2 ROS2 话题时，Python publishers 无需启动。"""
        return (
            self._geometry_runtime_subprocess is not None
            and self._geometry_runtime_subprocess.is_running
            and self._remote_provider is not None
            and self._remote_provider.is_connected
        )

    def _ensure_external_geometry_observers_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        with self._live_geometry_start_lock:
            self._ensure_external_debug_observer_started_locked()
            input_bridge_enabled = self._external_input_bridge_enabled()
            point_cloud_bridge_enabled = self._external_point_cloud_bridge_enabled()
            geometry_runtime_active = self._geometry_runtime_handles_input_bridge()
            if geometry_runtime_active:
                LOGGER.debug("Rust bridge 活跃，跳过 Python odom/pointcloud publishers")
            input_bridge_ready = True
            if geometry_runtime_active:
                input_bridge_ready = True  # Rust bridge 已处理 odom/pointcloud 输入
            elif input_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            elif point_cloud_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            if geometry_runtime_active and input_bridge_ready:
                return
            self._live_geometry_start_attempted = True
            try:
                pass  # Python providers 已移除，由 Rust bridge (GeometryRuntimeProvider) 替代
            except Exception:
                self._live_geometry_start_attempted = False
                raise

    def _ensure_external_debug_observer_started_locked(self) -> None:
        observer = self._external_debug_observer
        if observer is None:
            observer = ExternalGeometryDebugObserver(
                config=self.config,
                frontier_callback=self._handle_external_debug_frontier_update,
                pose_callback=self._handle_external_debug_pose_update,
            )
            self._external_debug_observer = observer
        if not observer.is_running:
            observer.start()
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
        observer.set_session_mode(session_mode)
        # commit 2 §2.2：localization 模式默认启动后必须立即 emit_initial_heartbeat，
        # 顶掉前端 SSE 滞留的 cleared 信号，避免复现 Symptom 2（地图变空白视觉表现）。
        if session_mode == "localization":
            try:
                self._map_stream.emit_initial_heartbeat()
            except Exception:
                LOGGER.debug("emit_initial_heartbeat 失败，忽略（localization 模式启动路径）。", exc_info=True)

    def _on_external_geometry_session_mode_changed(self, mode: Optional[str]) -> None:
        """同步 debug observer 会话模式，并在非 explore 时清空前沿缓存。

        commit 2 §2.1 扩展：接受 ``"localization"`` 入参；仅触发 observer
        ``set_session_mode("localization")``，不再清空 ``_external_debug_frontier_payload``
        （避免与 commit 1 set_session_mode 双清；该清空责任在 set_session_mode 内）。
        """
        observer = self._external_debug_observer
        if observer is not None:
            observer.set_session_mode(mode)
        if mode != "explore":
            with self._exploration_loop_lock:
                self._external_debug_frontier_payload = None
            try:
                self._notify_exploration_update(self.get_platform_exploration_state())
            except Exception:
                LOGGER.debug("清空非 explore 前沿缓存后通知失败。", exc_info=True)

    def get_external_live_pose(self, *, max_age_sec: float = 2.0) -> Optional[Pose]:
        """返回 GO2 debug observer /go2/pose_map 缓存的 map 系位姿，供 Nav2 下发目标。"""

        with self._external_live_pose_lock:
            pose = self._external_live_pose
            updated_at = self._external_live_pose_at_monotonic
        if pose is None:
            return None
        if max_age_sec > 0.0 and (time.monotonic() - updated_at) > max_age_sec:
            return None
        return self._resolve_pose_after_initial_hint(pose.model_copy(deep=True))

    def invalidate_external_live_pose_cache(self) -> None:
        """清空 /go2/pose_map 缓存，便于 2D Pose 后尽快接受新 TF。"""

        with self._external_live_pose_lock:
            self._external_live_pose = None
            self._external_live_pose_at_monotonic = 0.0

    def clear_debug_initial_pose_hint(self) -> None:
        """清除 debug 2D Pose 的 UI hint，避免与真实 TF 打架。"""

        with self._external_live_pose_lock:
            self._debug_initial_pose_hint = None
            self._debug_initial_pose_anchor = None
            self._debug_initial_pose_hint_until = 0.0
            self._debug_initial_pose_hint_stable_count = 0

    def _set_debug_initial_pose_hint(self, target_pose: Pose, *, ttl_sec: float = 12.0) -> None:
        with self._external_live_pose_lock:
            self._debug_initial_pose_hint = target_pose.model_copy(deep=True)
            self._debug_initial_pose_anchor = (
                self._external_live_pose.model_copy(deep=True)
                if self._external_live_pose is not None
                else None
            )
            self._debug_initial_pose_hint_until = time.monotonic() + max(1.0, float(ttl_sec))
            self._debug_initial_pose_hint_stable_count = 0

    def _resolve_pose_after_initial_hint(self, pose: Pose) -> Pose:
        with self._external_live_pose_lock:
            hint = self._debug_initial_pose_hint
            anchor = self._debug_initial_pose_anchor
            expires_at = self._debug_initial_pose_hint_until
        if hint is None or time.monotonic() > expires_at:
            with self._external_live_pose_lock:
                self._debug_initial_pose_hint = None
                self._debug_initial_pose_anchor = None
                self._debug_initial_pose_hint_until = 0.0
            return pose
        try:
            hx = float(hint.position.x)
            hy = float(hint.position.y)
            nx = float(pose.position.x)
            ny = float(pose.position.y)
            dist_to_hint = math.hypot(nx - hx, ny - hy)
            if anchor is not None:
                ax = float(anchor.position.x)
                ay = float(anchor.position.y)
                dist_anchor_to_hint = math.hypot(ax - hx, ay - hy)
                if dist_to_hint + 0.35 >= dist_anchor_to_hint:
                    return hint.model_copy(deep=True)
            if dist_to_hint <= 0.45:
                if anchor is not None:
                    ax = float(anchor.position.x)
                    ay = float(anchor.position.y)
                    if math.hypot(nx - ax, ny - ay) <= 0.35:
                        with self._external_live_pose_lock:
                            self._debug_initial_pose_hint_stable_count = 0
                        return hint.model_copy(deep=True)
                with self._external_live_pose_lock:
                    self._debug_initial_pose_hint_stable_count += 1
                    if self._debug_initial_pose_hint_stable_count >= 3:
                        self._debug_initial_pose_hint = None
                        self._debug_initial_pose_anchor = None
                        self._debug_initial_pose_hint_until = 0.0
                        self._debug_initial_pose_hint_stable_count = 0
                        return pose
                return hint.model_copy(deep=True)
            with self._external_live_pose_lock:
                self._debug_initial_pose_hint_stable_count = 0
            return hint.model_copy(deep=True)
        except (TypeError, ValueError):
            return pose

    def publish_debug_initial_pose(self, target_pose: Pose) -> Dict[str, object]:
        """RViz 2D Pose Estimate 入口。

        调用方可通过 debug 面板/外部 debug observer 路径注入初始位姿 hint；
        本入口仅保留协议签名（HTTP 503 语义：明确告知功能下线），不再默默失败。
        """
        # 清掉旧的 hint，让下一次 _resolve_pose_after_initial_hint 走默认路径。
        self.clear_debug_initial_pose_hint()
        return {
            "published": False,
            "topic": "/initialpose",
            "message": (
                "RViz 2D Pose Estimate 当前不可用："
                "当前 geometry runtime proto 未定义 relocalize/initial-pose RPC，"
                "因此平台不会伪造已提交结果。"
            ),
            "deprecated": True,
            "unsupported": True,
        }

    def _handle_external_debug_pose_update(
        self,
        pose: Any,
        frame_tree: Any = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> None:
        resolved_pose = pose
        if pose is not None and hasattr(pose, "model_copy"):
            resolved_pose = self._resolve_pose_after_initial_hint(pose.model_copy(deep=True))
            with self._external_live_pose_lock:
                self._external_live_pose = resolved_pose.model_copy(deep=True)
                self._external_live_pose_at_monotonic = time.monotonic()
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            deliveries = []
            for subscription_id, callback in tuple(
                self._localization_update_subscribers.items()
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="localization",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=None,
                )
                self._publish_subscription_delivery_ticket(ticket)
                deliveries.append(ticket)
        meta = dict(metadata or {})
        for ticket in deliveries:
            try:
                self._invoke_subscription_delivery(
                    ticket,
                    resolved_pose,
                    frame_tree,
                    meta,
                    retire=True,
                )
            except Exception:
                LOGGER.debug("投递外部 GO2 位姿 debug update 失败。", exc_info=True)

    def _handle_external_debug_frontier_update(self, payload: Dict[str, object]) -> None:
        """缓存调试前沿；探索任务状态只接受 native runtime 快照。"""

        with self._exploration_loop_lock:
            if self._external_geometry_session_mode != "explore":
                return
            self._external_debug_frontier_payload = self._clone_metadata(payload)

    def is_available(self) -> bool:
        """返回是否存在可供平台消费的几何后端。"""

        if not self._is_geometry_profile_enabled():
            return False
        if not self._uses_external_geometry_stack():
            return False
        navigation_backend = self._resolve_navigation_backend()
        exploration_backend = self._resolve_exploration_backend()
        navigation_available = False
        if navigation_backend is not None:
            checker = getattr(navigation_backend, "is_navigation_available", None)
            navigation_available = bool(checker()) if callable(checker) else bool(navigation_backend.is_available())
        exploration_available = False
        if exploration_backend is not None:
            checker = getattr(exploration_backend, "is_exploration_available", None)
            exploration_available = bool(checker()) if callable(checker) else bool(exploration_backend.is_available())
        return self.get_map_snapshot() is not None or any(
            available
            for available in (
                self._resolve_localization_backend() is not None,
                self._resolve_mapping_backend() is not None,
                navigation_available,
                exploration_available,
            )
        )

    def describe_backend(self) -> dict:
        """返回当前几何后端来源说明。"""

        if not self._is_geometry_profile_enabled():
            return {
                "source_kind": "disabled",
                "backend_name": None,
                "runtime_status": self.get_runtime_status(),
                "navigation_backend_policy": self._navigation_backend_policy(),
                "active_map_name": self._active_map_version.map_name if self._active_map_version is not None else None,
                "active_map_version_id": self._active_map_version.version_id if self._active_map_version is not None else None,
                **self.config.build_metadata(),
            }
        backend = (
            self._resolve_navigation_backend()
            or self._resolve_mapping_backend()
            or self._resolve_localization_backend()
            or self._resolve_exploration_backend()
        )
        source_kind = "unavailable"
        backend_name = None
        if backend is not None:
            if backend is getattr(self.robot, "data_plane_runtime", None):
                source_kind = "platform_compat_runtime"
            elif backend is getattr(self.robot, "data_plane", None):
                source_kind = "platform_device_bridge"
            elif backend is getattr(self.robot, "providers", None):
                source_kind = "platform_compat_provider"
            else:
                source_kind = "platform_backend"
            backend_name = getattr(backend, "provider_name", None)
        return {
            "source_kind": source_kind,
            "backend_name": backend_name,
            "runtime_status": self.get_runtime_status(),
            "live_map_provider": self._geometry_runtime_status_stub("live_map_provider"),
            "tf_localization_provider": self._geometry_runtime_status_stub("tf_localization_provider"),
            "odometry_publisher": self._geometry_runtime_status_stub("odometry_publisher"),
            "point_cloud_publisher": self._geometry_runtime_status_stub("point_cloud_publisher"),
            "navigation_backend_policy": self._navigation_backend_policy(),
            "active_map_name": self._active_map_version.map_name if self._active_map_version is not None else None,
            "active_map_version_id": self._active_map_version.version_id if self._active_map_version is not None else None,
            **self.config.build_metadata(),
        }

    def get_live_mapping_diagnostics(self, *, force_refresh: bool = False) -> Dict[str, object]:
        """返回探索建图输入链路的可行动诊断。"""

        now = time.monotonic()
        if (
            not force_refresh
            and self._live_mapping_diagnostics_cache is not None
            and now - self._live_mapping_diagnostics_cache_at < _LIVE_MAPPING_DIAGNOSTICS_TTL_SEC
        ):
            return self._clone_metadata(self._live_mapping_diagnostics_cache)

        diagnostics = self._build_live_mapping_diagnostics()
        self._live_mapping_diagnostics_cache = diagnostics
        self._live_mapping_diagnostics_cache_at = now
        return self._clone_metadata(diagnostics)

    def _build_live_mapping_diagnostics(self) -> Dict[str, object]:
        live_map_status = self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        odometry_status = self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = self._build_point_cloud_source_status()
        data_plane_status = self._build_data_plane_status()
        waiting_for, message = self._classify_live_mapping_wait(
            live_map_status=live_map_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )
        return {
            "ready": waiting_for is None,
            "waiting_for": waiting_for,
            "message": message,
            "live_geometry_start_attempted": self._live_geometry_start_attempted,
            "geometry_enabled": self.config.enabled,
            "mapping_profile_enabled": self.config.mapping.profile_enabled,
            "runtime_status": self.get_runtime_status(),
            "mapping_readiness": self._build_mapping_readiness(
                live_map_status=live_map_status,
                tf_localization_status=tf_localization_status,
                odometry_status=odometry_status,
                point_cloud_status=point_cloud_status,
                point_cloud_source=point_cloud_source,
                data_plane_status=data_plane_status,
            ),
            "live_map_provider": live_map_status,
            "tf_localization_provider": tf_localization_status,
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "config": self._build_live_mapping_config_summary(),
        }

    def _build_live_mapping_config_summary(self) -> Dict[str, object]:
        direct_dds = getattr(getattr(self.robot, "data_plane_runtime", None), "config", None)
        direct_dds = getattr(direct_dds, "direct_dds", None)
        return {
            "geometry_stack": self.config.stack,
            "odom_topic": self.config.topics.odom_topic,
            "point_cloud_topic": self.config.topics.point_cloud_topic,
            "occupancy_topic": self.config.topics.occupancy_topic,
            "cost_map_topic": self.config.topics.cost_map_topic,
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "geometry_profile_name": self.config.mapping.profile_name,
            "go2_direct_dds_point_cloud_topic": getattr(direct_dds, "point_cloud_topic", None),
            "go2_direct_dds_enabled": getattr(direct_dds, "enabled", None),
        }

    def _build_point_cloud_source_status(self) -> Dict[str, object]:
        provider = self._resolve_point_cloud_provider()
        data_plane_status = self._build_data_plane_status()
        status: Dict[str, object] = {
            "provider_name": getattr(provider, "provider_name", None),
            "provider_version": getattr(provider, "provider_version", None),
            "provider_available": provider is not None,
            "point_cloud_ready": bool(data_plane_status.get("dds_cloud_ready")),
            "point_count": int(data_plane_status.get("latest_point_cloud_points") or 0),
            "source_topic": data_plane_status.get("point_cloud_source_topic"),
            "frame_id": None,
            "timestamp": None,
            "last_error": None,
        }
        if provider is None:
            status["last_error"] = "未找到平台点云 provider。"
            return status
        availability_checker = getattr(provider, "is_available", None)
        if callable(availability_checker):
            try:
                status["provider_available"] = bool(availability_checker())
            except Exception as exc:
                status["provider_available"] = False
                status["last_error"] = str(exc)
        subscriber = getattr(provider, "subscribe_point_cloud", None)
        if not callable(subscriber):
            status["last_error"] = "点云 provider 未实现 subscribe_point_cloud。"
        return status

    def _build_data_plane_status(self) -> Dict[str, object]:
        runtime = getattr(self.robot, "data_plane_runtime", None)
        data_plane = getattr(self.robot, "data_plane", None)
        config = getattr(runtime, "config", None)
        if config is None:
            config = getattr(getattr(data_plane, "runtime", None), "config", None)
        status_getter = getattr(runtime, "get_status", None) or getattr(data_plane, "get_status", None)
        raw_status: Dict[str, object] = {}
        if callable(status_getter):
            try:
                raw_status = self._clone_metadata(status_getter())
            except Exception as exc:
                raw_status = {"last_error": str(exc)}
        direct_dds = getattr(config, "direct_dds", None)
        bridge_status = raw_status.get("bridge") if isinstance(raw_status.get("bridge"), dict) else {}
        return {
            "present": runtime is not None or data_plane is not None,
            "enabled": getattr(config, "enabled", None),
            "started": raw_status.get("started"),
            "direct_dds_started": bridge_status.get("direct_dds_started"),
            "dds_pose_ready": bridge_status.get("dds_pose_ready"),
            "dds_cloud_ready": bridge_status.get("dds_cloud_ready"),
            "pose_source": bridge_status.get("pose_source"),
            "point_cloud_source_topic": bridge_status.get("point_cloud_source_topic"),
            "latest_point_cloud_points": bridge_status.get("latest_point_cloud_points"),
            "direct_dds_enabled": getattr(direct_dds, "enabled", None),
            "direct_dds_pose_topic": getattr(direct_dds, "pose_topic", None),
            "direct_dds_point_cloud_topic": getattr(direct_dds, "point_cloud_topic", None),
            "dds_iface": getattr(config, "dds_iface", None),
            "last_error": raw_status.get("last_error"),
        }

    def _classify_live_mapping_wait(
        self,
        *,
        live_map_status: Dict[str, object],
        point_cloud_status: Dict[str, object],
        point_cloud_source: Dict[str, object],
        data_plane_status: Dict[str, object],
    ) -> Tuple[Optional[str], str]:
        live_map_error = str(live_map_status.get("last_error") or "").strip()
        if live_map_error:
            return "live_map_provider_error", "实时地图订阅错误：%s" % live_map_error
        occupancy_grid_ready = bool(live_map_status.get("occupancy_grid_ready"))
        if self._uses_external_geometry_stack() and not self._external_point_cloud_bridge_enabled():
            occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
            if not occupancy_grid_ready:
                return (
                    "external_occupancy_grid",
                    "等待外部几何后端在 %s 发布首帧占据栅格。"
                    % occupancy_topic,
                )
            return None, "外部建图链路已收到 %s 首帧占据栅格。" % occupancy_topic
        point_cloud_error = str(point_cloud_status.get("last_error") or "").strip()
        if point_cloud_error:
            return "point_cloud_bridge_error", "点云桥错误：%s" % point_cloud_error
        source_error = str(point_cloud_source.get("last_error") or "").strip()
        if source_error and not occupancy_grid_ready:
            return "point_cloud_input", source_error
        published_count = int(point_cloud_status.get("published_count") or 0)
        point_cloud_topic = str(point_cloud_status.get("topic") or point_cloud_status.get("point_cloud_topic") or self.config.topics.point_cloud_topic)
        source_topic = str(
            point_cloud_source.get("source_topic")
            or data_plane_status.get("point_cloud_source_topic")
            or data_plane_status.get("direct_dds_point_cloud_topic")
            or self.config.topics.point_cloud_topic
            or ""
        )
        if published_count <= 0:
            if data_plane_status.get("enabled") is False:
                return (
                    "point_cloud_input",
                    "Go2 data plane 未启用，点云桥尚未向 %s 发布首帧；请启用 GO2_DATA_PLANE_ENABLED 并确认 %s 有输入。"
                    % (point_cloud_topic, source_topic),
                )
            return (
                "point_cloud_input",
                "点云桥尚未向 %s 发布首帧，等待 Go2 data plane 从 %s 输入点云。"
                % (point_cloud_topic, source_topic),
            )
        if not bool(live_map_status.get("occupancy_grid_ready")):
            occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
            return (
                "external_occupancy_grid",
                "点云桥已向 %s 发布 %d 帧，等待外部几何后端在 %s 发布首帧占据栅格。"
                % (point_cloud_topic, published_count, occupancy_topic),
            )
        occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
        return None, "外部建图链路已收到 %s 首帧占据栅格。" % occupancy_topic

    def _attach_live_mapping_diagnostics_to_request(self, request):
        metadata = dict(getattr(request, "metadata", {}) or {})
        metadata["initial_mapping_chain_diagnostics"] = self.get_live_mapping_diagnostics()
        copier = getattr(request, "model_copy", None)
        if callable(copier):
            return request.model_copy(update={"metadata": metadata}, deep=True)
        request.metadata = metadata
        return request

    @staticmethod
    def _resolve_explorer_status_text(frontier_meta: Dict[str, object]) -> str:
        status_text = str((frontier_meta or {}).get("status_text") or "").strip()
        if not status_text:
            return ""
        upper = status_text.upper()
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        plan_fresh = bool(observer_payload.get("plan_fresh"))
        plan_points = int(observer_payload.get("plan_point_count") or 0)
        if plan_fresh and plan_points > 0 and any(
            term in upper for term in ("WARMING UP", "WAITING (NO REACHABLE")
        ):
            return "[explorer] NAVIGATING"
        return status_text

    @staticmethod
    def _external_explorer_headline_message(
        frontier_meta: Dict[str, object],
        *,
        frontier_count: Optional[int] = None,
    ) -> Optional[str]:
        status_text = PlatformGeometryRuntime._resolve_explorer_status_text(frontier_meta)
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        plan_fresh = bool(observer_payload.get("plan_fresh"))
        plan_points = int(observer_payload.get("plan_point_count") or 0)
        if not status_text:
            if frontier_count is not None and frontier_count > 0:
                return "正在探索：%d 个前沿点" % int(frontier_count)
            if plan_fresh and plan_points > 0:
                return "正在探索：已规划路径（%d 点）" % plan_points
            return None
        cleaned = re.sub(r"^\[explorer\]\s*", "", status_text, flags=re.IGNORECASE).strip()
        if not cleaned:
            if frontier_count is not None and frontier_count > 0:
                return "正在探索：%d 个前沿点" % int(frontier_count)
            return None
        upper = cleaned.upper()
        if "DONE (EXPLORATION FINISHED)" in upper:
            return "GO2 几何栈原生探索已完成：%s" % cleaned
        if upper.startswith("STRATEGY:"):
            return "探索换策略中：%s" % cleaned
        if frontier_count is not None and frontier_count > 0:
            return "正在探索（%s）：%d 个前沿点" % (cleaned, int(frontier_count))
        return "正在探索：%s" % cleaned

    def _augment_exploration_state_with_mapping_diagnostics(self, state):
        diagnostics = self.get_live_mapping_diagnostics()
        metadata = dict(getattr(state, "metadata", {}) or {})
        current_waiting_for = str(metadata.get("waiting_for") or "").strip() or None
        diagnostic_waiting_for = diagnostics.get("waiting_for")
        active = getattr(state.status, "value", state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }
        # scheme(b): 直接读外部 frontier observer 数据（不依赖 state 上尚未合并的字段）。
        # _external_debug_frontier_metadata 持 RLock，可重入；返回 (metadata, frontier_count)。
        frontier_meta, frontier_count = self._external_debug_frontier_metadata()
        frontier_status_text = str((frontier_meta or {}).get("status_text") or "").upper()
        explorer_message = self._external_explorer_headline_message(
            frontier_meta,
            frontier_count=frontier_count,
        )
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        explorer_actually_running = bool(
            active
            and "DONE (EXPLORATION FINISHED)" not in frontier_status_text
            and (
                explorer_message is not None
                or (frontier_count is not None and frontier_count > 0)
                or bool(observer_payload.get("plan_fresh"))
            )
        )
        # 安全条款：_error 结尾的诊断必须始终覆写 headline（真实错误不被遮蔽）。
        is_error_diagnostic = bool(
            diagnostic_waiting_for and str(diagnostic_waiting_for).endswith("_error")
        )
        # 覆写条件：错误诊断恒覆写；非错误诊断仅在 explorer 尚未实际运行时覆写。
        override_waiting = bool(
            active
            and diagnostic_waiting_for
            and (
                is_error_diagnostic
                or (
                    not explorer_actually_running
                    and current_waiting_for in {
                        None, "occupancy_grid", "mapping_ready",
                        "point_cloud_input", "external_occupancy_grid",
                    }
                )
            )
        )
        metadata["mapping_chain_diagnostics"] = diagnostics  # 始终保留诊断
        updates = {"metadata": metadata}
        if override_waiting:
            if diagnostic_waiting_for:
                metadata["waiting_for"] = diagnostic_waiting_for
            metadata["waiting_detail"] = diagnostics.get("message")
            if diagnostics.get("message"):
                updates["message"] = diagnostics.get("message")
        elif explorer_actually_running and not is_error_diagnostic:
            updates["message"] = explorer_message or updates.get("message")
            metadata["waiting_for"] = None
            metadata.pop("waiting_detail", None)
            status_text = str((frontier_meta or {}).get("status_text") or "").strip()
            if status_text:
                metadata["status_text"] = status_text
        with self._exploration_loop_lock:
            metadata["geometry_session_mode"] = self._external_geometry_session_mode
        updates["metadata"] = metadata
        return state.model_copy(update=updates, deep=True)

    def clear_active_map_version(self, *, reason: str = "") -> None:
        """清空当前激活的平台地图版本上下文。"""

        self._active_map_version = None
        self._last_activation_result = {"cleared": True, "reason": str(reason or "").strip() or None}

    def prepare_map_version_assets(
        self,
        version: MapVersion,
        *,
        skip_pcd_export: bool = False,
    ) -> MapVersion:
        """为平台地图版本补齐并落盘几何资产描述。"""

        previous_asset = version.geometry_asset
        parameter_summary = self._build_parameter_summary()
        profile_summary = self._clone_metadata(parameter_summary.get("profile") or {})
        mapping_database_path = self._version_mapping_database_path(version.map_name, version.version_id)
        pcd_path = self._version_pcd_asset_path(version.map_name, version.version_id)
        built_at = previous_asset.built_at if previous_asset is not None else version.timestamp
        backend_run = self._build_mapping_run_metadata()
        database_metadata = self._materialize_backend_state(mapping_database_path)
        # 导航地图归档来自 version 快照，不依赖 PCD 导出；PCD 失败不应阻塞 yaml/geometry_asset 落盘。
        nav2_map_metadata = self._materialize_nav2_map_assets(version)
        if skip_pcd_export:
            pcd_metadata = {
                "pcd_present": self._is_usable_pcd_file(pcd_path),
                "pcd_source_kind": "skipped_for_activation",
                "pcd_target_path": str(pcd_path.resolve(strict=False)),
            }
        else:
            pcd_metadata = self._materialize_pcd_asset(pcd_path)
        quality_metrics = self._build_geometry_quality_metrics(
            database_metadata=database_metadata,
            pcd_metadata=pcd_metadata,
            mapping_run=backend_run,
            nav2_map_metadata=nav2_map_metadata,
        )
        database_quality = dict(quality_metrics.get("database") or {})
        database_required = bool(database_quality.get("required", True))
        database_present = bool(database_quality.get("present", False))
        database_asset_type = MapGeometryAssetType.MAPPING_DATABASE
        asset_refs = []
        if database_required or database_present:
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=database_asset_type,
                    path=str(mapping_database_path),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=database_quality,
                    metadata={"required": database_required},
                )
            )
        nav2_map_quality = dict(quality_metrics.get("navigation_map") or {})
        if bool(nav2_map_quality.get("present")):
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.NAVIGATION_MAP,
                    path=str(nav2_map_quality.get("yaml_path") or nav2_map_metadata.get("yaml_path")),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=nav2_map_quality,
                    metadata={"required": True, "state_kind": "navigation_map_yaml"},
                )
            )
        asset_refs.append(
            MapGeometryAssetRef(
                asset_type=MapGeometryAssetType.PCD,
                path=str(pcd_path),
                frame_id=version.frame_id,
                profile_summary=profile_summary,
                built_at=built_at,
                quality_metrics=dict(quality_metrics.get("pcd") or {}),
                metadata={"required": False},
            )
        )
        # Phase 3: 追加 nuwax_mapping 资产引用。
        # 默认路径：当 mapping_adapter 已经有点云数据(revision>0)时,
        # 把 occupancy grid / costmap / cloud_map 三类资产写到 version 资产
        # 清单。映射路径分别为:
        # - occupancy: NUWAX_OCCUPANCY_SHM_PATH(默认 /tmp/nuwax_occupancy.mmap)
        # - costmap:    同 occupancy SHM(由 server::get_occupancy_grid_shm 写入,
        #               同一份 SHM 内包含 costmap 等价派生;此处仅引用同一文件)
        # - cloud_map:  NUWAX_CLOUDMAP_SHM_PATH(默认 /tmp/nuwax_cloudmap.mmap,
        #               既有 Phase 0 cloud_map SHM)
        # mapping_adapter 未启用时不追加这些 asset refs。
        if self._is_nuwax_mapping_active():
            occupancy_shm_path = os.environ.get(
                "NUWAX_OCCUPANCY_SHM_PATH", "/tmp/nuwax_occupancy.mmap"
            )
            cloud_map_shm_path = os.environ.get(
                "NUWAX_CLOUDMAP_SHM_PATH", "/tmp/nuwax_cloudmap.mmap"
            )
            nuwax_mapping_meta = {
                "backend_mode": "nuwax_mapping",
                "lcm_backend": self._mapping_lcm_backend_name(),
                "shm_layout_version": 1,
                "required": False,
            }
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.NUWAX_MAPPING_OCCUPANCY_GRID,
                    path=str(occupancy_shm_path),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=dict(quality_metrics.get("nuwax_mapping") or {}),
                    metadata=dict(nuwax_mapping_meta),
                )
            )
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.NUWAX_MAPPING_COSTMAP,
                    path=str(occupancy_shm_path),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics={},
                    metadata={**dict(nuwax_mapping_meta), "costmap_source": "nuwax_mapping_cost_mapper"},
                )
            )
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.NUWAX_MAPPING_CLOUD_MAP,
                    path=str(cloud_map_shm_path),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics={},
                    metadata={**dict(nuwax_mapping_meta), "voxel_size": 0.05},
                )
            )
        geometry_asset = MapGeometryAssetBinding(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_stack=self.config.stack,
            frame_id=version.frame_id,
            mapping_database_path=str(mapping_database_path),
            pcd_path=str(pcd_path),
            occupancy_projection=self.config.mapping.occupancy_projection,
            costmap_source=str(self.config.navigation.costmap_source),
            parameter_summary=parameter_summary,
            profile_summary=profile_summary,
            quality_metrics=quality_metrics,
            assets=list(asset_refs),
            built_at=built_at,
            metadata={
                **(dict(previous_asset.metadata) if previous_asset is not None else {}),
                "geometry_asset_path": str(self._geometry_asset_path(version.map_name, version.version_id)),
                "geometry_asset_root": str(self._version_geometry_root(version.map_name, version.version_id)),
                "catalog_root": str(self._catalog_root),
                "profile": profile_summary,
                "input_contract": parameter_summary.get("input_contract"),
                "output_contract": parameter_summary.get("output_contract"),
                "preprocessing": parameter_summary.get("preprocessing"),
                "navigable_layer": self.config.mapping.occupancy_projection,
                "traversability_layer": self.config.mapping.outputs.traversability_layer,
                "localization_status_contract": self.config.mapping.outputs.localization_status,
                "relocalization_diagnostics_contract": self.config.mapping.outputs.relocalization_diagnostics,
                "source_mode": quality_metrics.get("source_mode"),
                "streaming_readiness": self._clone_metadata(quality_metrics.get("streaming_readiness")),
                "backend_run": backend_run,
                "mapping_run": backend_run,
                "session_id": backend_run.get("session_id"),
                "run_id": backend_run.get("run_id"),
                "database": database_metadata,
                "pcd": pcd_metadata,
                "navigation_map": nav2_map_metadata,
                "diagnostics": quality_metrics.get("diagnostics", []),
            },
        )
        self._geometry_asset_path(version.map_name, version.version_id).parent.mkdir(parents=True, exist_ok=True)
        self._geometry_asset_path(version.map_name, version.version_id).write_text(
            geometry_asset.model_dump_json(indent=2),
            encoding="utf-8",
        )
        return version.model_copy(
            update={
                "geometry_asset": geometry_asset,
                "metadata": {
                    **dict(version.metadata),
                    "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset),
                    "backend_session_id": backend_run.get("session_id"),
                    "backend_run_id": backend_run.get("run_id"),
                    "mapping_database_path": database_metadata.get("database_source_path") or database_metadata.get("database_target_path"),
                    "backend_streaming_ready": quality_metrics.get("live_streaming_ready"),
                    "mapping_session_id": backend_run.get("session_id"),
                    "mapping_run_id": backend_run.get("run_id"),
                    "mapping_live_streaming_ready": quality_metrics.get("live_streaming_ready"),
                    "map_version_source_mode": quality_metrics.get("source_mode"),
                    "checkpoint_ready": quality_metrics.get("checkpoint_ready"),
                    "navigation_map_yaml_path": nav2_map_metadata.get("yaml_path"),
                    "navigation_map_ready": nav2_map_metadata.get("present"),
                },
            },
            deep=True,
        )

    def _version_db_ready_for_activation(self, version: MapVersion) -> bool:
        db_path = self._resolve_existing_mapping_db_for_version(version)
        return db_path is not None and self._is_usable_mapping_db_file(db_path)

    def activate_map_version(
        self,
        version: MapVersion,
        *,
        require_backend_switch: bool = True,
    ) -> MapSnapshot:
        """激活平台地图版本对应的几何资产。

        载入已有 .db 时不重复跑 PCD 导出（activate_map 需快速返回）。
        """

        if self._version_db_ready_for_activation(version) and version.geometry_asset is not None:
            prepared_version = version
        elif self._version_db_ready_for_activation(version):
            prepared_version = self.prepare_map_version_assets(version, skip_pcd_export=True)
        else:
            prepared_version = self.prepare_map_version_assets(version)
        geometry_asset = prepared_version.geometry_asset
        assert geometry_asset is not None
        activation_result = self._activate_backend_map_version(
            prepared_version,
            require_backend_switch=require_backend_switch,
        )
        self._active_map_version = prepared_version.model_copy(deep=True)
        self._last_activation_result = activation_result

        ready_snapshot = self._wait_for_geometry_snapshot(timeout_sec=0.2)
        if ready_snapshot is not None:
            return ready_snapshot
        fallback_snapshot = self.get_map_snapshot()
        if fallback_snapshot is None:
            raise RuntimeError(
                "平台地图版本 %s 已激活，但平台几何运行时未能提供地图快照。" % prepared_version.version_id
            )
        return fallback_snapshot

    def subscribe_map_updates(self, callback) -> int:
        if not callable(callback):
            raise ValueError("map update callback must be callable")

        def wrapped_callback(payload):
            return callback(self._attach_active_map_version_to_live_map_payload(payload))

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            subscription_id = self._next_map_subscription_id
            self._next_map_subscription_id += 1
            self._map_update_subscribers[subscription_id] = wrapped_callback
            self._pending_map_callbacks.append(wrapped_callback)
            provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="map",
                    subscription_id=subscription_id,
                    callback=wrapped_callback,
                    provider=provider,
                )

                def remote_callback(payload):
                    return self._invoke_subscription_delivery(
                        ticket,
                        payload,
                    )

                try:
                    handle = provider.subscribe_map_updates(remote_callback)
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._map_update_subscribers.get(subscription_id)
                            is wrapped_callback
                        ):
                            self._map_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_map_subscription_handles.pop(
                                subscription_id,
                                None,
                            )
                            for index in range(
                                len(self._pending_map_callbacks) - 1,
                                -1,
                                -1,
                            ):
                                if (
                                    self._pending_map_callbacks[index]
                                    is wrapped_callback
                                ):
                                    del self._pending_map_callbacks[index]
                                    break
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if handle is not None:
                    self._remote_map_subscription_handles[
                        subscription_id
                    ] = handle
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def subscribe_navigation_updates(self, callback) -> int:
        """注册导航 push；sidecar 尚未连接时保留并在 attach 后重绑。"""

        if not callable(callback):
            raise ValueError("navigation update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._runtime_state_lock:
                subscription_id = self._next_navigation_subscription_id
                self._next_navigation_subscription_id += 1
                self._navigation_update_subscribers[subscription_id] = callback
                provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="navigation",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=provider,
                )

                def remote_callback(state):
                    return self._invoke_subscription_delivery(
                        ticket,
                        state,
                    )

                try:
                    remote_subscription_id = (
                        provider.subscribe_navigation_updates(remote_callback)
                    )
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._navigation_update_subscribers.get(
                                subscription_id
                            )
                            is callback
                        ):
                            self._navigation_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_navigation_subscription_ids.pop(
                                subscription_id,
                                None,
                            )
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if remote_subscription_id is not None:
                    with self._runtime_state_lock:
                        self._remote_navigation_subscription_ids[subscription_id] = int(
                            remote_subscription_id
                        )
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        with provider_attach_lock:
            with self._runtime_state_lock:
                removed = self._navigation_update_subscribers.pop(
                    int(subscription_id),
                    None,
                )
                remote_subscription_id = self._remote_navigation_subscription_ids.pop(
                    int(subscription_id),
                    None,
                )
                provider = self._remote_provider
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked(
                        kind="navigation",
                        subscription_id=int(subscription_id),
                    )
                )
        if (
            remote_subscription_id is not None
            and provider is not None
            and bool(getattr(provider, "is_connected", False))
        ):
            try:
                provider.unsubscribe_navigation_updates(remote_subscription_id)
            except Exception:
                LOGGER.exception(
                    "取消远端 navigation subscription=%s 失败。",
                    remote_subscription_id,
                )
        self._wait_for_subscription_deliveries(delivery_tickets)
        return removed is not None

    def unsubscribe_map_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        handle = None
        delivery_tickets = ()
        with provider_attach_lock:
            handle = self._remote_map_subscription_handles.pop(
                int(subscription_id),
                None,
            )
            callback = self._map_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if callback is not None:
                try:
                    self._pending_map_callbacks.remove(callback)
                except ValueError:
                    pass
            delivery_tickets = (
                self._invalidate_subscription_delivery_tickets_locked(
                    kind="map",
                    subscription_id=int(subscription_id),
                )
            )
        _cancel_remote_stream_handle(handle, stream_name="map update")
        self._wait_for_subscription_deliveries(delivery_tickets)
        return callback is not None

    def _attach_active_map_version_to_live_map_payload(self, payload):
        if not isinstance(payload, dict):
            return payload
        version = self._active_map_version
        if version is None:
            return payload
        metadata = self._clone_metadata(payload.get("metadata") or {})
        metadata.update(self._build_map_snapshot_geometry_metadata(
            version=version,
            geometry_asset=version.geometry_asset,
            live_available=True,
            map_snapshot_kind="live_streaming",
            source_kind="platform_live_map_provider",
        ))
        mapping_context = getattr(self, "_mapping_session_context", None)
        mapping_active = isinstance(mapping_context, dict) and bool(
            mapping_context.get("active")
        )
        exploration_active = getattr(self, "_external_exploration_request", None) is not None
        if exploration_active or mapping_active:
            pass  # 探索模式下不抑制 live map（新建地图从小到大增长是正常的）
        elif (
            self._external_geometry_process_is_localization()
            or str((self._last_activation_result or {}).get("backend_mode") or "") == "mapping_localization"
        ) and version.snapshot.occupancy_grid is not None:
            # 定位模式 Debug 固定展示已加载的平台 checkpoint；live /map_obstacles
            # （含 2D Pose 后零星重发）与保存版本栅格不一致，不得替换显示层。
            metadata.update({
                "event_type": "heartbeat",
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": "localization_mode_pin_checkpoint_display",
                "pin_checkpoint_display": True,
                "map_snapshot_kind": "offline_checkpoint",
                "source_kind": "platform_map_version",
                "display_map_snapshot_kind": "offline_checkpoint",
                "platform_map_version_id": version.version_id,
                "platform_map_version_revision": version.revision,
                "active_map_name": version.map_name,
                "map_name": version.map_name,
                "checkpoint_ready": True,
                "geometry_ready": True,
                "backend_mode": "mapping_localization",
                "waiting_for_live_stream": False,
            })
            return {
                **payload,
                "occupancy_grid": None,
                "cost_map": None,
                "semantic_map": self._clone_contract(version.snapshot.semantic_map),
                "metadata": metadata,
            }
        elif self._live_map_payload_under_covers_seed(payload, metadata, version):
            live_known_cells = self._metadata_known_cell_count(metadata)
            if live_known_cells is None:
                live_known_cells = self._occupancy_known_cell_count(payload.get("occupancy_grid"))
            seed_known_cells = self._occupancy_known_cell_count(version.snapshot.occupancy_grid)
            metadata.update({
                "event_type": "heartbeat",
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": "live_occupancy_known_cells_below_seed_map",
                "live_known_cell_count": live_known_cells,
                "seed_known_cell_count": seed_known_cells,
            })
            self._publish_snapshot_suppressed_event(
                version=version,
                metadata=metadata,
                live_known_cells=live_known_cells,
                seed_known_cells=seed_known_cells,
            )
            return {
                **payload,
                "occupancy_grid": None,
                "cost_map": None,
                "semantic_map": None,
                "metadata": metadata,
            }
        return {
            **payload,
            "semantic_map": payload.get("semantic_map") or self._clone_contract(version.snapshot.semantic_map),
            "metadata": metadata,
        }

    def _publish_snapshot_suppressed_event(
        self,
        *,
        version: MapVersion,
        metadata: Dict[str, object],
        live_known_cells,
        seed_known_cells,
    ) -> None:
        """把 live<seed 抑制语义通过 EventBus 暴露给消费方，避免静默卡图。"""

        event_bus = getattr(self.robot, "event_bus", None) or getattr(self, "_event_bus", None)
        if event_bus is None:
            return
        try:
            from contracts.events import RuntimeEventCategory
            payload = {
                "map_name": version.map_name,
                "map_version_id": version.version_id,
                "suppressed_live_map_reason": metadata.get("suppressed_live_map_reason"),
                "live_known_cell_count": live_known_cells,
                "seed_known_cell_count": seed_known_cells,
                "source_topic": str(self.config.topics.occupancy_topic or ""),
                "frame_id": metadata.get("frame_id"),
                "update_sequence": metadata.get("update_sequence"),
            }
            event_bus.publish(
                event_bus.build_event(
                    "mapping.snapshot_suppressed",
                    category=RuntimeEventCategory.ROBOT,
                    source="platform_geometry_runtime",
                    message="live_map known_cell_count 低于种子，已抑制本帧。",
                    payload=payload,
                )
            )
        except Exception:
            LOGGER.debug("发布 mapping.snapshot_suppressed 失败。", exc_info=True)

    def _live_map_payload_under_covers_seed(self, payload, metadata: Dict[str, object], version: MapVersion) -> bool:
        if str(metadata.get("event_type") or "content_update") != "content_update":
            return False
        if version.snapshot.occupancy_grid is None:
            return False
        live_known_cells = self._metadata_known_cell_count(metadata)
        if live_known_cells is None:
            live_known_cells = self._occupancy_known_cell_count(payload.get("occupancy_grid"))
        seed_known_cells = self._occupancy_known_cell_count(version.snapshot.occupancy_grid)
        return seed_known_cells is not None and (live_known_cells is None or live_known_cells < seed_known_cells)

    def _metadata_known_cell_count(self, metadata: Dict[str, object]) -> Optional[int]:
        occupancy_metrics = metadata.get("occupancy_metrics")
        if not isinstance(occupancy_metrics, dict) or "known_cell_count" not in occupancy_metrics:
            return None
        try:
            return int(occupancy_metrics.get("known_cell_count") or 0)
        except (TypeError, ValueError):
            return None

    def _occupancy_known_cell_count(self, occupancy_grid) -> Optional[int]:
        data = getattr(occupancy_grid, "data", None)
        if data is None:
            return None
        return sum(1 for value in data if int(value) >= 0)

    def get_map_snapshot(self) -> Optional[MapSnapshot]:
        """返回当前平台几何运行时对外暴露的地图快照。"""

        version = self._active_map_version
        live_occupancy = self._read_live_map_layer("occupancy_grid")
        live_cost_map = self._read_live_map_layer("cost_map")
        live_available = live_occupancy is not None or live_cost_map is not None
        if live_available:
            return self._build_live_map_snapshot(
                live_occupancy=live_occupancy,
                live_cost_map=live_cost_map,
                seed_version=version,
            )
        if version is None:
            return None
        occupancy_grid = self._clone_contract(version.snapshot.occupancy_grid)
        cost_map = self._clone_contract(version.snapshot.cost_map)
        semantic_map = self._clone_contract(version.snapshot.semantic_map)
        if occupancy_grid is None and cost_map is None and semantic_map is None:
            return None
        geometry_asset = version.geometry_asset
        geometry_metadata = self._build_map_snapshot_geometry_metadata(
            version=version,
            geometry_asset=geometry_asset,
            live_available=False,
            map_snapshot_kind="offline_checkpoint",
            source_kind="platform_map_version",
        )
        return MapSnapshot(
            source_name=self.providers.provider_name,
            version_id=version.version_id,
            revision=version.revision,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata={
                **dict(version.snapshot.metadata),
                **geometry_metadata,
            },
        )

    def _build_live_map_snapshot(self, *, live_occupancy, live_cost_map, seed_version: Optional[MapVersion]) -> MapSnapshot:
        live_status = self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        update_sequence = live_status.get("update_sequence")
        occupancy_sequence = int(live_status.get("occupancy_sequence") or live_status.get("occupancy_message_count") or 0)
        cost_map_sequence = int(live_status.get("cost_map_sequence") or live_status.get("cost_map_message_count") or 0)
        message_count = max(int(live_status.get("message_count") or 0), occupancy_sequence + cost_map_sequence)
        occupancy_content_revision = int(live_status.get("occupancy_revision") or 0)
        cost_map_content_revision = int(live_status.get("cost_map_revision") or 0)
        content_revision = occupancy_content_revision + cost_map_content_revision
        try:
            live_revision = max(1, int(content_revision or 1))
        except (TypeError, ValueError):
            live_revision = 1
        occupancy_metrics = self._clone_metadata(live_status.get("occupancy_metrics") or {})
        cost_map_metrics = self._clone_metadata(live_status.get("cost_map_metrics") or {})
        occupancy_content_hash = occupancy_metrics.get("content_hash")
        cost_map_content_hash = cost_map_metrics.get("content_hash")
        seed = "|".join(str(part or "") for part in (
            live_status.get("provider_name"),
            live_status.get("occupancy_topic"),
            live_status.get("cost_map_topic"),
            live_status.get("started"),
        ))
        digest = hashlib.sha256(seed.encode("utf-8")).hexdigest()[:16] if seed.strip("|") else "current"
        metadata = {
            "event_type": "content_update",
            "source_kind": "platform_live_map_provider",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "mapping_live_streaming_ready": True,
            "source_name": live_status.get("provider_name") or "geometry_runtime",
            "update_sequence": update_sequence,
            "occupancy_sequence": occupancy_sequence,
            "cost_map_sequence": cost_map_sequence,
            "message_count": message_count,
            "occupancy_content_hash": occupancy_content_hash,
            "cost_map_content_hash": cost_map_content_hash,
            "content_hash": occupancy_content_hash,
            "occupancy_content_revision": occupancy_content_revision,
            "cost_map_content_revision": cost_map_content_revision,
            "content_revision": content_revision,
            "occupancy_metrics": occupancy_metrics,
            "cost_map_metrics": cost_map_metrics,
            "live_map_provider": live_status,
            "live_mapping_diagnostics": self.get_live_mapping_diagnostics(),
        }
        if seed_version is not None:
            metadata.update({
                "map_name": seed_version.map_name,
                "seed_map_version_id": seed_version.version_id,
                "seed_map_version_revision": seed_version.revision,
                "seed_source_snapshot_version_id": seed_version.source_version_id,
            })
            metadata.update(self._build_map_snapshot_geometry_metadata(
                version=seed_version,
                geometry_asset=seed_version.geometry_asset,
                live_available=True,
                map_snapshot_kind="live_streaming",
                source_kind="platform_live_map_provider",
            ))
        else:
            metadata.update({
                "geometry_stack": self.config.stack,
                "occupancy_projection": self.config.mapping.occupancy_projection,
                "costmap_source": str(self.config.navigation.costmap_source),
                "parameter_summary": self._build_parameter_summary(),
                "profile": self._clone_metadata(self.config.build_metadata().get("profile")),
                "input_contract": self._clone_metadata(self.config.build_metadata().get("input_contract")),
                "output_contract": self._clone_metadata(self.config.build_metadata().get("output_contract")),
                "preprocessing": self._clone_metadata(self.config.build_metadata().get("preprocessing")),
                "geometry_backend": self.describe_backend(),
            })
        return MapSnapshot(
            source_name=str(live_status.get("provider_name") or "geometry_runtime"),
            version_id="live_%s_%s" % (digest, live_revision),
            revision=live_revision,
            occupancy_grid=self._clone_contract(live_occupancy),
            cost_map=self._clone_contract(live_cost_map),
            semantic_map=self._clone_contract(seed_version.snapshot.semantic_map) if seed_version is not None else None,
            metadata=metadata,
        )

    def _build_map_snapshot_geometry_metadata(
        self,
        *,
        version: MapVersion,
        geometry_asset: Optional[MapGeometryAssetBinding],
        live_available: bool,
        map_snapshot_kind: str,
        source_kind: str,
    ) -> Dict[str, object]:
        version_metadata = {"map_name": version.map_name}
        if map_snapshot_kind == "live_streaming":
            version_metadata.update({
                "seed_map_version_id": version.version_id,
                "seed_map_version_revision": version.revision,
                "seed_source_snapshot_version_id": version.source_version_id,
            })
        else:
            version_metadata.update({
                "platform_map_version_id": version.version_id,
                "platform_map_version_revision": version.revision,
                "source_snapshot_version_id": version.source_version_id,
            })
        metadata = {
            "source_kind": source_kind,
            "map_snapshot_kind": map_snapshot_kind,
            "mapping_live_streaming_ready": bool(live_available),
            **version_metadata,
            "geometry_stack": geometry_asset.geometry_stack if geometry_asset is not None else self.config.stack,
            "mapping_database_path": geometry_asset.mapping_database_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "occupancy_projection": geometry_asset.occupancy_projection if geometry_asset is not None else self.config.mapping.occupancy_projection,
            "costmap_source": geometry_asset.costmap_source if geometry_asset is not None else str(self.config.navigation.costmap_source),
            "parameter_summary": self._clone_metadata(
                geometry_asset.parameter_summary if geometry_asset is not None else self._build_parameter_summary()
            ),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary if geometry_asset is not None else {}),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics if geometry_asset is not None else {}),
            "geometry_assets": self._clone_metadata(geometry_asset.assets if geometry_asset is not None else []),
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
            "built_at": geometry_asset.built_at.isoformat() if geometry_asset is not None else None,
            "geometry_ready": bool(
                live_available
                or (
                    geometry_asset is not None
                    and geometry_asset.quality_metrics.get("status") == "ready"
                    and (
                        geometry_asset.quality_metrics.get("database", {}).get("present", False)
                        or not geometry_asset.quality_metrics.get("database", {}).get("required", True)
                    )
                )
            ),
            "geometry_diagnostics": self._clone_metadata(
                geometry_asset.quality_metrics.get("diagnostics", []) if geometry_asset is not None else []
            ),
            "geometry_activation": self._clone_metadata(self._last_activation_result),
            "profile": self._clone_metadata(
                geometry_asset.parameter_summary.get("profile") if geometry_asset is not None else self.config.build_metadata().get("profile")
            ),
            "input_contract": self._clone_metadata(
                geometry_asset.parameter_summary.get("input_contract") if geometry_asset is not None else self.config.build_metadata().get("input_contract")
            ),
            "output_contract": self._clone_metadata(
                geometry_asset.parameter_summary.get("output_contract") if geometry_asset is not None else self.config.build_metadata().get("output_contract")
            ),
            "preprocessing": self._clone_metadata(
                geometry_asset.parameter_summary.get("preprocessing") if geometry_asset is not None else self.config.build_metadata().get("preprocessing")
            ),
            "geometry_backend": self.describe_backend(),
            "live_mapping_diagnostics": self.get_live_mapping_diagnostics(),
        }
        if geometry_asset is not None:
            metadata.update(dict(geometry_asset.metadata))
        metadata["source_kind"] = source_kind
        metadata["map_snapshot_kind"] = map_snapshot_kind
        metadata["mapping_live_streaming_ready"] = bool(live_available)
        metadata.update(version_metadata)
        return metadata

    def _is_geometry_profile_enabled(self) -> bool:
        return bool(self.config.enabled and self.config.mapping.profile_enabled)

    def _resolve_backend(self, data_plane_protocol, provider_protocol):
        if not self._is_geometry_profile_enabled():
            return None
        data_plane_runtime = getattr(self.robot, "data_plane_runtime", None)
        if data_plane_runtime is not None and isinstance(data_plane_runtime, data_plane_protocol):
            return data_plane_runtime
        data_plane = getattr(self.robot, "data_plane", None)
        if data_plane is not None and isinstance(data_plane, data_plane_protocol) and data_plane_protocol not in {
            LocalizationDataPlane,
            MappingDataPlane,
            NavigationDataPlane,
            ExplorationDataPlane,
            GeometryBackend,
        }:
            return data_plane
        if data_plane_runtime is None and data_plane is not None and isinstance(data_plane, data_plane_protocol):
            return data_plane
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, provider_protocol):
            return providers
        return None

    def _resolve_geometry_backend(self) -> Optional[GeometryBackend]:
        backend = self._resolve_backend(GeometryBackend, NavigationProvider)
        if backend is not None and isinstance(backend, GeometryBackend):
            return backend
        return None

    def _resolve_raw_localization_backend(self):
        return self._resolve_backend(LocalizationDataPlane, LocalizationProvider)

    def _resolve_localization_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        if self._remote_provider is not None and self._remote_provider.is_connected:
            return self._remote_provider
        return self._resolve_raw_localization_backend()

    def _resolve_mapping_backend(self):
        if self._remote_provider is not None and self._remote_provider.is_connected:
            return self._remote_provider
        return self._resolve_backend(MappingDataPlane, MapProvider)

    def _resolve_point_cloud_provider(self):
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, PointCloudProvider):
            return providers
        return None

    def _resolve_navigation_backend(self):
        """解析平台正式导航后端；不可用时返回 None。"""
        if not self._is_geometry_profile_enabled():
            return None
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and getattr(remote, "is_connected", False):
            return remote
        return None

    def _resolve_exploration_backend(self):
        """解析唯一生产探索后端：连通的 GeometryRuntimeProvider。"""

        if not self._is_geometry_profile_enabled():
            return None
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and getattr(remote, "is_connected", False):
            return remote
        return None

    def _safe_map_dir(self, map_name: str) -> Path:
        primary = self._catalog_root / build_safe_dir_segment(map_name)
        if not primary.exists():
            legacy = self._catalog_root / quote(str(map_name or "").strip(), safe="")
            if legacy.exists():
                return legacy
        return primary

    def _version_dir(self, map_name: str, version_id: str) -> Path:
        versions_root = self._safe_map_dir(map_name) / "versions"
        primary = versions_root / build_safe_dir_segment(version_id)
        if not primary.exists():
            legacy = versions_root / quote(str(version_id or "").strip(), safe="")
            if legacy.exists():
                return legacy
        return primary

    def _version_geometry_root(self, map_name: str, version_id: str) -> Path:
        return self._version_dir(map_name, version_id) / "geometry" / self.config.stack

    def _geometry_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "geometry_asset.json"

    def _map_named_backend_db_filename(self, map_name: str) -> str:
        """探索/定位使用的 .db 文件名与地图名一致。"""

        safe_name = build_safe_dir_segment(map_name) or "map"
        return "%s.db" % safe_name

    def _version_mapping_database_path(self, map_name: str, version_id: str) -> Path:
        """返回该地图版本的建图数据库路径。"""

        return self._version_geometry_root(map_name, version_id) / "mapping" / self._map_named_backend_db_filename(map_name)

    def _candidate_version_mapping_database_paths(self, map_name: str, version_id: str) -> List[Path]:
        """返回当前地图版本的建图数据库候选。"""

        mapping_dir = self._version_geometry_root(map_name, version_id) / "mapping"
        return [mapping_dir / self._map_named_backend_db_filename(map_name)]

    def _external_mapping_state_asset_name(self) -> str:
        configured_name = str(getattr(self.config.mapping.outputs, "database_asset_name", "") or "").strip()
        return configured_name or "nuwax_mapping_state.db"

    def _version_pcd_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "pcd" / self.config.mapping.outputs.pcd_asset_name

    def _version_nav2_map_yaml_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "navigation" / "map.yaml"

    def _version_nav2_map_pgm_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "navigation" / "map.pgm"

    def _materialize_nav2_map_assets(self, version: MapVersion) -> Dict[str, object]:
        occupancy_grid = version.snapshot.occupancy_grid
        yaml_path = self._version_nav2_map_yaml_path(version.map_name, version.version_id)
        pgm_path = self._version_nav2_map_pgm_path(version.map_name, version.version_id)
        if occupancy_grid is None:
            return {
                "present": False,
                "source_kind": "unavailable",
                "yaml_path": str(yaml_path.resolve(strict=False)),
                "pgm_path": str(pgm_path.resolve(strict=False)),
                "reason": "occupancy_grid_missing",
            }
        width = int(occupancy_grid.width)
        height = int(occupancy_grid.height)
        data = list(occupancy_grid.data)
        if len(data) != width * height:
            return {
                "present": False,
                "source_kind": "platform_snapshot",
                "yaml_path": str(yaml_path.resolve(strict=False)),
                "pgm_path": str(pgm_path.resolve(strict=False)),
                "reason": "occupancy_grid_size_mismatch",
                "width": width,
                "height": height,
                "data_length": len(data),
            }
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with pgm_path.open("wb") as handle:
            handle.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
            for row in range(height - 1, -1, -1):
                start = row * width
                for value in data[start:start + width]:
                    if value < 0:
                        pixel = 205
                    elif value >= 65:
                        pixel = 0
                    else:
                        pixel = 254
                    handle.write(bytes((pixel,)))
        origin = occupancy_grid.origin
        yaw = self._yaw_from_pose(origin)
        yaml_path.write_text(
            "image: %s\nmode: trinary\nresolution: %.12g\norigin: [%.12g, %.12g, %.12g]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            % (pgm_path.name, float(occupancy_grid.resolution_m), float(origin.position.x), float(origin.position.y), yaw),
            encoding="utf-8",
        )
        return {
            "present": True,
            "source_kind": "platform_snapshot",
            "yaml_path": str(yaml_path.resolve(strict=False)),
            "pgm_path": str(pgm_path.resolve(strict=False)),
            "yaml": self._file_metadata(yaml_path),
            "pgm": self._file_metadata(pgm_path),
            "frame_id": occupancy_grid.frame_id,
            "width": width,
            "height": height,
            "resolution_m": float(occupancy_grid.resolution_m),
        }

    def _yaw_from_pose(self, pose: Pose) -> float:
        orientation = pose.orientation
        siny_cosp = 2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y))
        cosy_cosp = 1.0 - 2.0 * (float(orientation.y) * float(orientation.y) + float(orientation.z) * float(orientation.z))
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_parameter_summary(self) -> Dict[str, object]:
        return self._clone_metadata(self.config.build_metadata())

    def _materialize_mapping_database(self, target_path: Path) -> Dict[str, object]:
        """Deprecated alias —— 转向 _materialize_backend_state。"""

        return self._materialize_backend_state(target_path)

    def _materialize_backend_state(self, target_path: Path) -> Dict[str, object]:
        """持久化 RTAB-Map .db 状态到版本目录（sqlite3 .backup 安全拷贝）。"""

        return self._materialize_mapping_db(target_path)

    def _is_usable_mapping_db_file(self, path: Path) -> bool:
        resolved = path.expanduser().resolve(strict=False)
        if not resolved.is_file():
            return False
        try:
            return int(resolved.stat().st_size) > 4096
        except OSError:
            return False

    def _locate_runtime_backend_state_for_materialize(self, target_path: Path) -> Optional[Path]:
        """定位可拷贝到版本目录的运行中 .db，避免误用其它地图版本或 active 版本路径。"""

        configured_root = Path(str(self.config.mapping.database_root or "").strip()).expanduser()
        candidates: List[Path] = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "live_mapping.db",
                    configured_root / self._external_mapping_state_asset_name(),
                )
            )
        for candidate in candidates:
            if (
                self._is_usable_mapping_db_file(candidate)
                and candidate.resolve(strict=False) != resolved_target
            ):
                return candidate
        return None

    def _locate_active_exploration_backend_state(self) -> Optional[Path]:
        """定位当前探索会话写入的 .db（即使与本次 save 目标 version 路径不同）。"""

        with self._exploration_loop_lock:
            raw_path = self._external_exploration_database_path
        if not raw_path:
            return None
        candidate = Path(str(raw_path)).expanduser()
        if not candidate.is_absolute():
            candidate = self._repo_root() / candidate
        if candidate.suffix == ".db" and self._is_usable_mapping_db_file(candidate):
            return candidate
        return None

    def _db_file_size_bytes(self, path: Path) -> int:
        try:
            return int(path.expanduser().resolve(strict=False).stat().st_size)
        except OSError:
            return 0

    def _resolve_db_materialization_source(self, target_path: Path) -> Tuple[Optional[Path], str]:
        """按优先级选择 .db 落盘来源，探索产物优先于共享 live_rtabmap.db。"""

        exploration_source = self._locate_external_exploration_backend_state_for_target(target_path)
        if exploration_source is not None:
            return exploration_source, "exploration_target"

        active_exploration_source = self._locate_active_exploration_backend_state()
        resolved_target = target_path.expanduser().resolve(strict=False)
        if active_exploration_source is not None:
            resolved_active = active_exploration_source.expanduser().resolve(strict=False)
            if resolved_active != resolved_target:
                return active_exploration_source, "exploration_runtime"
            return active_exploration_source, "exploration_target"

        if self._is_usable_mapping_db_file(resolved_target):
            return resolved_target, "persisted_target"

        runtime_source = self._locate_runtime_backend_state_for_materialize(resolved_target)
        if runtime_source is not None:
            if active_exploration_source is not None and self._db_file_size_bytes(
                active_exploration_source
            ) > self._db_file_size_bytes(runtime_source):
                return active_exploration_source, "exploration_runtime"
            return runtime_source, "live_runtime"
        if active_exploration_source is not None:
            return active_exploration_source, "exploration_runtime"
        return None, "unavailable"

    def _materialize_mapping_db(self, target_path: Path) -> Dict[str, object]:
        """持久化 RTAB-Map .db 到版本目录，优先使用 sqlite3 .backup 保证写入安全。"""

        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path, source_kind_hint = self._resolve_db_materialization_source(target_path)
        if source_path is None:
            if target_path.exists() and target_path.is_file():
                return {
                    "database_present": True,
                    "database_source_kind": "persisted_asset",
                    "database_source_path": str(target_path.resolve()),
                    "database_target_path": str(target_path.resolve()),
                    "database_target": self._file_metadata(target_path),
                }
            return {
                "database_present": False,
                "database_source_kind": "unavailable",
                "database_source_path": None,
                "database_target_path": str(target_path.resolve()),
            }
        resolved_source = source_path.resolve()
        resolved_target = target_path.resolve()
        if resolved_source == resolved_target:
            return {
                "database_present": True,
                "database_source_kind": source_kind_hint,
                "database_source_path": str(resolved_source),
                "database_target_path": str(resolved_target),
                "database_target": self._file_metadata(resolved_target),
            }
        if (
            source_kind_hint == "live_runtime"
            and self._is_usable_mapping_db_file(resolved_target)
            and self._db_file_size_bytes(resolved_target)
            > max(self._db_file_size_bytes(resolved_source), 4096) * 12 // 10
        ):
            return {
                "database_present": True,
                "database_source_kind": "persisted_target",
                "database_source_path": str(resolved_target),
                "database_target_path": str(resolved_target),
                "database_target": self._file_metadata(resolved_target),
                "skipped_live_runtime_overwrite": True,
            }
        source_before = self._file_metadata(resolved_source)
        backup_ok = False
        try:
            import subprocess
            result = subprocess.run(
                ["sqlite3", str(resolved_source), ".backup '%s'" % str(resolved_target)],
                capture_output=True, timeout=30,
            )
            backup_ok = result.returncode == 0 and resolved_target.exists()
        except Exception:
            pass
        if not backup_ok:
            shutil.copy2(resolved_source, resolved_target)
        source_after = self._file_metadata(resolved_source)
        return {
            "database_present": target_path.exists(),
            "database_source_kind": "sqlite3_backup" if backup_ok else source_kind_hint,
            "database_source_path": str(resolved_source),
            "database_target_path": str(resolved_target),
            "database_source_before": source_before,
            "database_source_after": source_after,
            "database_target": self._file_metadata(resolved_target),
            "live_db_size_changed_during_copy": (
                source_before.get("size_bytes") != source_after.get("size_bytes")
                or source_before.get("mtime_ns") != source_after.get("mtime_ns")
            ),
        }

    def _locate_external_exploration_backend_state_for_target(self, target_path: Path) -> Optional[Path]:
        with self._exploration_loop_lock:
            raw_path = self._external_exploration_database_path
        if not raw_path:
            return None
        candidate = Path(str(raw_path)).expanduser()
        if not candidate.is_absolute():
            candidate = self._repo_root() / candidate
        try:
            if candidate.resolve(strict=False) != target_path.resolve(strict=False):
                return None
        except Exception:
            return None
        if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
            return candidate
        return None

    def _pcd_min_points(self) -> int:
        from services.geometry.cloud_map_fetcher import default_min_points

        return default_min_points()

    def _is_usable_pcd_file(self, path: Path) -> bool:
        from services.geometry.pointcloud_map import read_pcd_point_count

        resolved = path.expanduser().resolve(strict=False)
        if not resolved.is_file():
            return False
        try:
            if int(resolved.stat().st_size) < 1024:
                return False
            return int(read_pcd_point_count(resolved)) >= self._pcd_min_points()
        except Exception:
            return False

    def _materialize_pcd_asset(
        self,
        target_path: Path,
    ) -> Dict[str, object]:
        """校验当前运行时已落盘的版本 PCD。"""

        target_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_target = target_path.expanduser().resolve()
        if self._is_usable_pcd_file(resolved_target):
            return {
                "pcd_present": True,
                "pcd_source_kind": "persisted_asset",
                "pcd_source_path": str(resolved_target),
                "pcd_target_path": str(resolved_target),
                "pcd_target": self._file_metadata(resolved_target),
            }

        if resolved_target.exists() and not self._is_usable_pcd_file(resolved_target):
            try:
                resolved_target.unlink()
            except Exception:
                LOGGER.debug("删除无效版本 PCD 失败：%s", resolved_target, exc_info=True)

        return {
            "pcd_present": False,
            "pcd_source_kind": "unavailable",
            "pcd_source_path": None,
            "pcd_target_path": str(resolved_target),
        }

    def _build_geometry_quality_metrics(
        self,
        *,
        database_metadata: Dict[str, object],
        pcd_metadata: Dict[str, object],
        mapping_run: Dict[str, object],
        nav2_map_metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        diagnostics = []
        nav2_map_metadata = dict(nav2_map_metadata or {})
        database_present = bool(database_metadata.get("database_present", False))
        external_stack = self._uses_external_geometry_stack()
        nav2_map_present = bool(nav2_map_metadata.get("present", False))
        database_required = not external_stack
        pcd_present = bool(pcd_metadata.get("pcd_present", False))
        readiness = self._build_mapping_readiness()
        live_source = database_metadata.get("database_source_kind") == "live_runtime"
        live_db_stat = database_metadata.get("database_source_after")
        live_db_mtime_age_sec = None
        if isinstance(live_db_stat, dict) and live_db_stat.get("mtime_epoch") is not None:
            live_db_mtime_age_sec = max(0.0, time.time() - float(live_db_stat.get("mtime_epoch") or 0.0))
        live_db_updating = bool(
            live_source
            and live_db_mtime_age_sec is not None
            and live_db_mtime_age_sec <= max(5.0, _GEOMETRY_STREAMING_WINDOW_SEC * 2.0)
            and int(live_db_stat.get("size_bytes") or 0) > 0
        ) if isinstance(live_db_stat, dict) else False
        live_streaming_ready = bool(readiness.get("streaming_ready") and live_db_updating)
        if not database_present and database_required:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "mapping_database_missing",
                    "message": "地图版本缺少建图数据库资产。",
                }
            )
        if external_stack and not nav2_map_present:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "navigation_map_asset_missing",
                    "message": "地图版本缺少导航地图资产，无法加载定位状态。",
                }
            )
        if not database_present and not database_required:
            diagnostics.append(
                {
                    "level": "info" if nav2_map_present else "warning",
                    "code": "external_geometry_state_not_materialized",
                    "message": "未找到建图状态数据库；当前版本使用导航地图作为定位检查点。",
                }
            )
        if database_present and not live_streaming_ready:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "external_geometry_state_checkpoint_only",
                    "message": "建图状态数据库已落盘为离线 checkpoint，但未证明外部几何状态正在随地图更新。",
                }
            )
        pcd_target_path = str(pcd_metadata.get("pcd_target_path") or "").strip()
        if pcd_target_path:
            target_pcd = Path(pcd_target_path).expanduser()
            if target_pcd.is_file() and not self._is_usable_pcd_file(target_pcd):
                pcd_present = False
                diagnostics.append(
                    {
                        "level": "error",
                        "code": "pcd_asset_stale_or_too_small",
                        "message": "地图版本 PCD 点云无效（点数过少或为模板占位文件），请重新保存。",
                    }
                )
        if not pcd_present:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "pcd_asset_missing",
                    "message": "地图版本缺少 PCD 点云资产，ICP 质量指标为 unknown。",
                }
            )
        if not mapping_run.get("run_id"):
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "mapping_run_id_missing",
                    "message": "未能从外部几何进程状态解析真实 run_id。",
                }
            )
        # status="ready" 必须 backend checkpoint 与导航地图同时就位。
        if external_stack:
            backend_checkpoint_ready = bool(database_present)
            ready_for_load = backend_checkpoint_ready and nav2_map_present
            if ready_for_load:
                status = "ready"
            elif nav2_map_present:
                status = "snapshot_only"
            else:
                status = "snapshot_only"
        else:
            backend_checkpoint_ready = bool(database_present)
            status = "ready" if backend_checkpoint_ready else "failed"
        return {
            "status": status,
            "live_streaming_ready": live_streaming_ready,
            "checkpoint_ready": backend_checkpoint_ready,
            "backend_checkpoint_ready": backend_checkpoint_ready,
            "navigation_map_present": nav2_map_present,
            "platform_snapshot_only": bool(external_stack and not backend_checkpoint_ready),
            "backend_switch_supported": True,
            "source_mode": (
                "live_streaming_checkpoint" if live_streaming_ready
                else "mapping_database_checkpoint" if external_stack and backend_checkpoint_ready
                else "navigation_map_checkpoint" if external_stack and nav2_map_present
                else "platform_snapshot_only" if external_stack
                else "offline_checkpoint"
            ),
            "streaming_readiness": readiness,
            "mapping_run": mapping_run,
            "database": {
                "present": database_present,
                "required": database_required,
                "source_kind": database_metadata.get("database_source_kind"),
                "source_path": database_metadata.get("database_source_path"),
                "target_path": database_metadata.get("database_target_path"),
                "mtime_age_sec": live_db_mtime_age_sec,
                "live_updating": live_db_updating,
                "quality": "live_updating" if live_db_updating else "checkpoint_or_unknown",
            },
            "pcd": {
                "present": pcd_present,
                "source_kind": pcd_metadata.get("pcd_source_kind"),
                "source_path": pcd_metadata.get("pcd_source_path"),
                "target_path": pcd_metadata.get("pcd_target_path"),
                "quality": "present" if pcd_present else "unknown",
            },
            "navigation_map": {
                **self._clone_metadata(nav2_map_metadata),
                "required": bool(external_stack),
                "quality": "loadable" if nav2_map_present else "missing",
            },
            "diagnostics": diagnostics,
        }

    def _summarize_geometry_asset(self, geometry_asset: MapGeometryAssetBinding) -> Dict[str, object]:
        return {
            "geometry_stack": geometry_asset.geometry_stack,
            "frame_id": geometry_asset.frame_id,
            "mapping_database_path": geometry_asset.mapping_database_path,
            "pcd_path": geometry_asset.pcd_path,
            "asset_types": [item.asset_type.value for item in geometry_asset.assets],
            "assets": self._clone_metadata(geometry_asset.assets),
            "built_at": geometry_asset.built_at.isoformat(),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary),
            "source_mode": geometry_asset.metadata.get("source_mode"),
            "session_id": geometry_asset.metadata.get("session_id"),
            "run_id": geometry_asset.metadata.get("run_id"),
        }

    def _file_metadata(self, path: Path) -> Dict[str, object]:
        try:
            stat = path.stat()
        except OSError as exc:
            return {"present": False, "error": str(exc), "path": str(path)}
        return {
            "present": True,
            "path": str(path),
            "size_bytes": int(stat.st_size),
            "mtime_epoch": float(stat.st_mtime),
            "mtime_ns": int(stat.st_mtime_ns),
            "mtime_iso": datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc).isoformat(),
        }

    # ========================================================================
    # Phase 3: nuwax_mapping 资产写入相关 helper
    # ========================================================================

    def _is_nuwax_mapping_active(self) -> bool:
        """Phase 3: 检查 nuwax_mapping backend 是否就绪。

        返回 True 当且仅当:
        - `GeometryRuntimeProvider` 已在 self._runtime_provider
        - 远程 runtime HealthCheck 返回 healthy=True
        - remote 端 mapping_ready_poll() 返回 True(详见 feedback_phase3.md)

        返回 False 时，`prepare_map_version_assets` 走平台建图路径，
        不追加 nuwax_mapping_* asset refs。
        """
        provider = getattr(self, "_runtime_provider", None)
        if provider is None:
            return False
        try:
            return bool(provider.is_mapping_ready())
        except Exception as exc:  # noqa: BLE001
            logger.debug(
                "_is_nuwax_mapping_active: provider.is_mapping_ready 失败,降级为 False: %s",
                exc,
            )
            return False

    def _mapping_lcm_backend_name(self) -> str:
        """Phase 3: 取 LCM backend canonical 名(给 nuwax_mapping asset metadata)。"""
        provider = getattr(self, "_runtime_provider", None)
        if provider is None:
            return "<unknown>"
        try:
            return str(provider.get_lcm_backend_name())
        except Exception as exc:  # noqa: BLE001
            logger.debug(
                "_mapping_lcm_backend_name: provider.get_lcm_backend_name 失败,降级为 <unknown>: %s",
                exc,
            )
            return "<unknown>"

    def _build_mapping_run_metadata(self) -> Dict[str, object]:
        process_name = self._external_process_name()
        process = self._managed_processes.get(process_name)
        db_path = self._locate_live_mapping_database()
        started_at = process.started_at_iso if process is not None else None
        pid = process.process.pid if process is not None and process.process is not None and process.is_running() else None
        source = str(db_path.resolve()) if db_path is not None else None
        seed = "|".join(str(part or "") for part in (source, started_at, pid))
        digest = hashlib.sha256(seed.encode("utf-8")).hexdigest()[:16] if seed.strip("|") else ""
        run_id = "nuwax_geometry_runtime_%s" % digest if digest else None
        return {
            "session_id": run_id,
            "run_id": run_id,
            "database_path": source,
            "started_at": started_at,
            "pid": pid,
            "node_name": process_name,
            "source_stack": self.config.stack,
            "log_path": str(process.log_path) if process is not None else None,
            "process_running": process.is_running() if process is not None else False,
        }

    def get_live_pointcloud_map_asset_summary(self) -> Dict[str, object]:
        """返回当前累计点云地图 PCD 资产候选，不读取扫描帧。"""

        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            geometry_asset = self._active_map_version.geometry_asset
            current_path = geometry_asset.pcd_path
            if current_path and self._is_usable_pcd_file(Path(current_path).expanduser()):
                return self._pointcloud_map_asset_summary(
                    Path(current_path).expanduser(),
                    source_kind="map_version_pcd",
                    priority=1,
                    map_name=self._active_map_version.map_name,
                    map_version_id=self._active_map_version.version_id,
                    frame_id=geometry_asset.frame_id or self._active_map_version.frame_id,
                    geometry_stack=geometry_asset.geometry_stack,
                    mapping_database_path=geometry_asset.mapping_database_path,
                )

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_pcd_map_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    return self._pointcloud_map_asset_summary(
                        Path(candidate).expanduser(),
                        source_kind="mapping_backend_pcd",
                        priority=2,
                        frame_id=self.config.frames.map_frame,
                        geometry_stack=self.config.stack,
                    )

        candidates = self._configured_pcd_asset_candidates()
        for candidate in candidates:
            if self._is_usable_pcd_file(candidate):
                return self._pointcloud_map_asset_summary(
                    candidate,
                    source_kind="configured_pcd_root",
                    priority=3,
                    frame_id=self.config.frames.map_frame,
                    geometry_stack=self.config.stack,
                    expected_candidates=candidates,
                )

        primary = candidates[0] if candidates else None
        if primary is not None:
            return self._pointcloud_map_asset_summary(
                primary,
                source_kind="configured_pcd_root",
                priority=3,
                frame_id=self.config.frames.map_frame,
                geometry_stack=self.config.stack,
                expected_candidates=candidates,
            )
        return {
            "available": False,
            "status": "missing",
            "source_kind": "configured_pcd_root",
            "reason": "configured_pcd_root_empty",
            "pcd_path": None,
            "expected_candidates": [],
            "frame_id": self.config.frames.map_frame,
            "geometry_stack": self.config.stack,
            "not_fallback_to_sensor_scan": True,
        }

    def _configured_pcd_asset_candidates(self) -> List[Path]:
        configured_value = str(self.config.mapping.pcd_root or "").strip()
        if not configured_value:
            return []
        configured_root = Path(configured_value).expanduser()
        if configured_root.suffix == ".pcd":
            return [configured_root]
        return [
            configured_root / self.config.mapping.outputs.pcd_asset_name,
            configured_root / "cloud_map.pcd",
            configured_root / "map.pcd",
            configured_root / "current.pcd",
        ]

    def _pointcloud_map_asset_summary(
        self,
        path: Path,
        *,
        source_kind: str,
        priority: int,
        frame_id: Optional[str] = None,
        geometry_stack: Optional[str] = None,
        map_name: Optional[str] = None,
        map_version_id: Optional[str] = None,
        mapping_database_path: Optional[str] = None,
        expected_candidates: Optional[List[Path]] = None,
    ) -> Dict[str, object]:
        resolved = path.expanduser().resolve(strict=False)
        available = resolved.exists() and resolved.is_file()
        summary: Dict[str, object] = {
            "available": available,
            "status": "available" if available else "missing",
            "source_kind": source_kind,
            "priority": priority,
            "pcd_path": str(resolved),
            "path": str(resolved),
            "map_name": map_name,
            "map_version_id": map_version_id,
            "frame_id": frame_id or self.config.frames.map_frame,
            "geometry_stack": geometry_stack or self.config.stack,
            "mapping_database_path": mapping_database_path,
            "not_fallback_to_sensor_scan": True,
            "expected_candidates": [str(item.expanduser().resolve(strict=False)) for item in (expected_candidates or [path])],
        }
        if available:
            try:
                summary["file"] = self._file_metadata(resolved)
            except Exception as exc:
                summary["file"] = {"present": True, "path": str(resolved), "error": str(exc)}
            return summary
        summary["reason"] = "%s_missing" % source_kind
        return summary

    def _locate_live_pcd_asset(self) -> Optional[Path]:
        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            current_path = self._active_map_version.geometry_asset.pcd_path
            if current_path:
                current_pcd = Path(current_path).expanduser()
                if self._is_usable_pcd_file(current_pcd):
                    return current_pcd

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_pcd_map_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if self._is_usable_pcd_file(candidate_path):
                        return candidate_path

        for candidate in self._configured_pcd_asset_candidates():
            if self._is_usable_pcd_file(candidate):
                return candidate
        return None

    def _locate_live_mapping_database(self) -> Optional[Path]:
        """定位当前活跃的建图数据库。"""

        return self._locate_live_backend_state()

    def _locate_live_backend_state(self) -> Optional[Path]:
        """定位当前活跃的建图数据库文件。"""

        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            asset = self._active_map_version.geometry_asset
            current_path = Path(asset.mapping_database_path or "").expanduser()
            if current_path.suffix == ".db" and current_path.exists() and current_path.is_file():
                return current_path

        configured_root = Path(str(self.config.mapping.database_root or "").strip()).expanduser()
        candidates: List[Path] = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "live_mapping.db",
                    configured_root / self._external_mapping_state_asset_name(),
                    configured_root / "current.db",
                    configured_root / "active.db",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _resolve_existing_mapping_db_for_version(self, version: MapVersion) -> Optional[Path]:
        """Find the .db file for a specific map version."""
        geometry_asset = version.geometry_asset
        candidates: List[Path] = []
        if geometry_asset is not None:
            if geometry_asset.mapping_database_path:
                candidates.append(Path(geometry_asset.mapping_database_path).expanduser())
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type in (
                    MapGeometryAssetType.MAPPING_DATABASE,
                    MapGeometryAssetType.NAVIGATION_MAP,
                ):
                    candidates.append(Path(asset_ref.path).expanduser())
        candidates.append(self._version_mapping_database_path(version.map_name, version.version_id))
        for candidate in self._candidate_version_mapping_database_paths(version.map_name, version.version_id):
            if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
                return candidate
        for candidate in candidates:
            if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _activate_backend_map_version(
        self,
        version: MapVersion,
        *,
        require_backend_switch: bool = True,
    ) -> Dict[str, object]:
        """切换后端地图版本。

        require_backend_switch=False 时（activate_map / restore_active_workspace 默认路径）
        实时栈下不重启 SLAM 进程，仅返回 backend_switch_applied=False 但允许调用方继续 ingest 种子。
        """

        geometry_asset = version.geometry_asset
        backend_state = geometry_asset.mapping_database_path if geometry_asset is not None else None
        activation_payload = {
            "map_name": version.map_name,
            "version_id": version.version_id,
            "geometry_stack": self.config.stack,
            "mapping_database_path": backend_state,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "frame_id": version.frame_id,
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
            "require_backend_switch": bool(require_backend_switch),
        }
        if not require_backend_switch:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "require_backend_switch_false",
                "backend_mode": "live_streaming_only",
            }
        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None and mapping_backend is not self._remote_provider:
            activator = getattr(mapping_backend, "activate_platform_map_version", None)
            if callable(activator):
                result = activator(
                    map_name=version.map_name,
                    version_id=version.version_id,
                    geometry_asset=geometry_asset.model_dump(mode="json") if geometry_asset is not None else None,
                    metadata={
                        "source_name": version.source_name,
                        "source_version_id": version.source_version_id,
                    },
                )
                if isinstance(result, dict):
                    cloned_result = self._clone_metadata(result)
                    backend_switch_applied = bool(cloned_result.get("backend_switch_applied", True))
                    return {
                        **activation_payload,
                        **cloned_result,
                        "backend_switch_applied": backend_switch_applied,
                    }
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_result": result,
                }
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "backend_missing_activate_platform_map_version",
                "backend": type(mapping_backend).__name__,
            }
        if self._uses_external_geometry_stack():
            return self._activate_external_nav2_map_version(version, activation_payload)
        if mapping_backend is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "mapping_backend_unavailable",
            }
        return {
            **activation_payload,
            "backend_switch_applied": False,
            "reason": "backend_missing_activate_platform_map_version",
        }

    def _activate_external_nav2_map_version(
        self,
        version: MapVersion,
        activation_payload: Dict[str, object],
    ) -> Dict[str, object]:
        # RTAB-Map activation: restart the managed process with the version's .db
        # Use mapping mode for empty/new DBs, localization mode for populated DBs.
        db_path = self._resolve_existing_mapping_db_for_version(version)
        nav2_map_metadata = self._resolve_nav2_map_metadata(version)
        nav2_yaml_path = self._resolve_existing_nav2_map_yaml_path(version, nav2_map_metadata)
        if nav2_yaml_path is None:
            materialized_metadata = self._materialize_nav2_map_assets(version)
            nav2_map_metadata = {**nav2_map_metadata, **self._clone_metadata(materialized_metadata)}
            nav2_yaml_path = self._resolve_existing_nav2_map_yaml_path(version, nav2_map_metadata)
        if nav2_yaml_path is not None:
            nav2_yaml_path = nav2_yaml_path.expanduser().resolve(strict=False)
        if db_path is not None and db_path.exists():
            self._ensure_external_geometry_observers_started()
            process_name = self._external_process_name()
            current_process = self._managed_processes.get(process_name)
            resolved_db = db_path.expanduser().resolve()
            db_has_map_data = db_path.stat().st_size > 4096
            backend_mode = "mapping_localization" if db_has_map_data else "mapping"
            use_static_map_yaml = self._external_localization_uses_static_map_yaml()
            static_map_yaml_path = (
                nav2_yaml_path
                if db_has_map_data and nav2_yaml_path is not None and use_static_map_yaml
                else None
            )
            static_map_requested = static_map_yaml_path is not None
            if db_has_map_data:
                self.clear_external_exploration_state(reason="activate_mapping_localization")
            # 空数据库时不启动 --mapping：mapping 是手动建图专用。
            # 新地图（空数据库）激活时只需停止当前外部进程，等待 explore 时再启动 --explore。
            if not db_has_map_data:
                if current_process is not None and current_process.is_running():
                    current_process.stop(
                        timeout_sec=12.0,
                        graceful_launch_shutdown=True,
                        graceful_timeout_sec=15.0,
                    )
                    time.sleep(3.0)
                    self._managed_processes.pop(process_name, None)
                self._map_stream_tracker = {}
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_map_loaded": False,
                    "backend_mode": "unavailable",
                    "active_backend_version_id": version.version_id,
                    "reason": "mapping_database_empty_no_auto_mapping",
                    "mapping": {
                        "switch_method": "stop_previous_no_start",
                        "database_path": str(resolved_db),
                        "note": "mapping is manual-only; use explore_area to start --explore",
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": False,
                    },
                }
            # 有数据的数据库：走 --localization
            if (
                current_process is not None
                and current_process.is_running()
                and self._external_process_serves_database(current_process.command, resolved_db, localization=True)
                and (
                    not static_map_requested
                    or self._external_process_uses_map_yaml(current_process.command, static_map_yaml_path)
                )
            ):
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_map_loaded": True,
                    "backend_mode": backend_mode,
                    "active_backend_version_id": version.version_id,
                    "reason": "mapping_localization_already_active",
                    "mapping": {
                        "switch_method": "reuse_running_localization",
                        "database_path": str(resolved_db),
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": static_map_requested,
                    },
                }
            restart = current_process is not None
            if current_process is not None and current_process.is_running():
                current_process.stop(
                    timeout_sec=12.0,
                    graceful_launch_shutdown=True,
                    graceful_timeout_sec=15.0,
                )
                time.sleep(3.0)
            self._map_stream_tracker = {}
            command = [str(self._external_script_path()), "--localization", str(db_path)]
            if static_map_yaml_path is not None:
                command.extend(["--map-yaml", str(static_map_yaml_path)])
            # GO2 几何栈 localization reads the passed .db directly (localize_only:=true).
            # Static map_server is opt-in via NUWAX_NAV2_USE_STATIC_MAP_YAML=true (legacy GO2_NAV_* deprecated).
            self._start_managed_process(process_name, command, self._external_log_name(), restart=restart)
            process = self._managed_processes.get(process_name)
            process_running = bool(process is not None and process.is_running())
            if process_running:
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_map_loaded": True,
                    "backend_mode": backend_mode,
                    "active_backend_version_id": version.version_id,
                    "reason": f"{backend_mode}_started",
                    "mapping": {
                        "switch_method": "process_restart_localization",
                        "database_path": str(db_path),
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": static_map_requested,
                    },
                }
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "mapping_localization_start_failed",
                "backend_mode": "unavailable",
                "mapping": {"database_path": str(db_path)},
                "navigation_map": {
                    **nav2_map_metadata,
                    "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                    "used_for_static_map": False,
                },
            }

        # Fallback: try the external localization process with map.yaml.
        if nav2_yaml_path is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "navigation_map_and_mapping_database_missing",
                "backend_mode": "unavailable",
                "navigation_map": nav2_map_metadata,
            }
        pcd_path = self._resolve_existing_version_pcd_path(version)
        command = self._build_nav2_map_activation_command(
            map_yaml_path=nav2_yaml_path,
            pcd_path=pcd_path,
        )
        self._ensure_external_geometry_observers_started()
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        restart = current_process is not None
        if current_process is not None:
            current_process.stop()
        self._map_stream_tracker = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=restart)
        process = self._managed_processes.get(process_name)
        process_status = process.status() if process is not None else None
        process_running = bool(process is not None and process.is_running())
        if not process_running:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "external_localization_start_failed",
                "backend_mode": "amcl_localization_only",
                "navigation_map": nav2_map_metadata,
                "navigation": {
                    "switch_method": "managed_localization_process",
                    "map_yaml_path": str(nav2_yaml_path),
                    "pcd_path": str(pcd_path) if pcd_path is not None else None,
                    "pcd_present": bool(pcd_path is not None),
                    "command": command,
                    "process": process_status,
                },
            }
        return {
            **activation_payload,
            "backend_switch_applied": True,
            "backend_map_loaded": True,
            "backend_mode": "amcl_localization_only",
            "active_backend_version_id": version.version_id,
            "reason": "external_localization_started",
            "navigation": {
                "switch_method": "managed_localization_process",
                "map_yaml_path": str(nav2_yaml_path),
                "pcd_path": str(pcd_path) if pcd_path is not None else None,
                "pcd_present": bool(pcd_path is not None),
                "command": command,
                "process": process_status,
            },
        }

    def _resolve_nav2_map_metadata(self, version: MapVersion) -> Dict[str, object]:
        geometry_asset = version.geometry_asset
        metadata: Dict[str, object] = {}
        if geometry_asset is not None:
            metadata.update(self._clone_metadata(geometry_asset.quality_metrics.get("navigation_map") or {}))
            metadata.update(self._clone_metadata(geometry_asset.metadata.get("navigation_map") or {}))
        version_metadata = getattr(version, "metadata", None)
        if isinstance(version_metadata, dict):
            yaml_path = version_metadata.get("navigation_map_yaml_path")
            if yaml_path and not metadata.get("yaml_path"):
                metadata["yaml_path"] = yaml_path
            ready = version_metadata.get("navigation_map_ready")
            if ready is not None and "present" not in metadata:
                metadata["present"] = bool(ready)
        return metadata

    def _resolve_existing_nav2_map_yaml_path(
        self,
        version: MapVersion,
        nav2_map_metadata: Dict[str, object],
    ) -> Optional[Path]:
        candidates: List[object] = []
        geometry_asset = version.geometry_asset
        if geometry_asset is not None:
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type != MapGeometryAssetType.NAVIGATION_MAP:
                    continue
                asset_metadata = dict(asset_ref.metadata or {})
                if asset_metadata.get("state_kind") == "navigation_map_yaml":
                    candidates.append(asset_ref.path)
                candidates.append(asset_ref.quality_metrics.get("yaml_path"))
        candidates.append(nav2_map_metadata.get("yaml_path"))
        candidates.append(self._version_nav2_map_yaml_path(version.map_name, version.version_id))
        for candidate in candidates:
            if not candidate:
                continue
            path = Path(str(candidate)).expanduser()
            if path.exists() and path.is_file():
                return path
        return None

    def _resolve_existing_version_pcd_path(self, version: MapVersion) -> Optional[Path]:
        geometry_asset = version.geometry_asset
        candidates: List[object] = []
        if geometry_asset is not None:
            candidates.append(geometry_asset.pcd_path)
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type == MapGeometryAssetType.PCD:
                    candidates.append(asset_ref.path)
        candidates.append(self._version_pcd_asset_path(version.map_name, version.version_id))
        for candidate in candidates:
            if not candidate:
                continue
            path = Path(str(candidate)).expanduser()
            if path.exists() and path.is_file():
                return path.resolve(strict=False)
        return None

    def _build_nav2_map_activation_command(self, *, map_yaml_path: Path, pcd_path: Optional[Path]) -> List[str]:
        # 直接复用外部 SLAM/Nav2 脚本；平台只连接外部 Nav2 action。
        return self._build_external_geometry_command()

    def _ensure_amcl_initialized(self) -> None:
        """等待 AMCL 发布 map→odom TF；超时后发布 identity initialpose 作为 fallback。"""
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                pose = self._remote_provider.get_current_pose() if self._remote_provider is not None else None
            except Exception:
                pose = None
            if pose is not None:
                LOGGER.info("AMCL 已初始化，map→odom TF 可用。")
                return
            time.sleep(0.5)
        LOGGER.warning("AMCL 15 秒内未发布 TF，发布 identity initialpose 作为 fallback。")
        self._publish_initialpose_fallback()
        deadline2 = time.monotonic() + 5.0
        while time.monotonic() < deadline2:
            try:
                pose = self._remote_provider.get_current_pose() if self._remote_provider is not None else None
            except Exception:
                pose = None
            if pose is not None:
                LOGGER.info("AMCL 在 initialpose fallback 后成功初始化。")
                return
            time.sleep(0.5)
        LOGGER.warning("AMCL 在 initialpose fallback 后仍未发布 TF，定位栈可能未就绪。")

    def _publish_initialpose_fallback(self) -> None:
        """显式报告 relocalize 未接入，禁止把无调用方的 hasattr 分支当成功。"""

        message = (
            "geometry runtime proto 当前没有 Relocalize/PublishInitialPose RPC；"
            "Python 不会用 PublishOdom 冒充初始位姿。"
        )
        self._record_runtime_diagnostic("relocalize_unsupported", message)
        LOGGER.warning(message)

    def _wait_for_geometry_snapshot(self, *, timeout_sec: float) -> Optional[MapSnapshot]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        snapshot = self.get_map_snapshot()
        while snapshot is None and time.monotonic() < deadline:
            time.sleep(0.05)
            snapshot = self.get_map_snapshot()
        return snapshot

    def _read_live_map_layer(self, layer_name: str):
        backend = self._resolve_mapping_backend()
        if backend is None:
            return None
        getter = getattr(backend, f"get_{layer_name}", None)
        if not callable(getter):
            return None
        return getter()

    def _has_live_or_seed_map(self) -> bool:
        return bool(
            (self._remote_provider is not None and self._remote_provider.is_connected)
            or self._active_map_version is not None
        )

    def subscribe_localization_updates(self, callback) -> Optional[int]:
        """订阅平台 TF 定位输出。"""

        if not self._is_geometry_profile_enabled():
            return None
        if not callable(callback):
            raise ValueError("localization update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            subscription_id = self._next_localization_subscription_id
            self._next_localization_subscription_id += 1
            self._localization_update_subscribers[subscription_id] = callback
            self._pending_localization_callbacks.append(callback)
            # External observer owns delivery for the external stack.
            if self._uses_external_geometry_stack():
                return subscription_id
            provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="localization",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=provider,
                )

                def remote_callback(*args, **kwargs):
                    return self._invoke_subscription_delivery(
                        ticket,
                        *args,
                        **kwargs,
                    )

                try:
                    handle = provider.subscribe_pose(remote_callback)
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._localization_update_subscribers.get(
                                subscription_id
                            )
                            is callback
                        ):
                            self._localization_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_localization_subscription_handles.pop(
                                subscription_id,
                                None,
                            )
                            for index in range(
                                len(self._pending_localization_callbacks) - 1,
                                -1,
                                -1,
                            ):
                                if (
                                    self._pending_localization_callbacks[index]
                                    is callback
                                ):
                                    del self._pending_localization_callbacks[
                                        index
                                    ]
                                    break
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if handle is not None:
                    self._remote_localization_subscription_handles[
                        subscription_id
                    ] = handle
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def unsubscribe_localization_updates(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        handle = None
        delivery_tickets = ()
        with provider_attach_lock:
            handle = self._remote_localization_subscription_handles.pop(
                int(subscription_id),
                None,
            )
            callback = self._localization_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if callback is not None:
                try:
                    self._pending_localization_callbacks.remove(callback)
                except ValueError:
                    pass
            delivery_tickets = (
                self._invalidate_subscription_delivery_tickets_locked(
                    kind="localization",
                    subscription_id=int(subscription_id),
                )
            )
        _cancel_remote_stream_handle(handle, stream_name="localization update")
        self._wait_for_subscription_deliveries(delivery_tickets)

    def subscribe_exploration_updates(self, callback: Callable[[ExplorationState], None]) -> int:
        """订阅探索状态；sidecar 尚未连接时保留并在 attach 后重绑。"""

        if not callable(callback):
            raise ValueError("exploration update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        remote_subscription_id = None
        provider = None
        with provider_attach_lock:
            with self._exploration_loop_lock:
                subscription_id = self._next_exploration_subscription_id
                self._next_exploration_subscription_id += 1
                self._exploration_update_subscribers[subscription_id] = callback
                provider = self._remote_provider
                remote_subscription_id_before = (
                    self._remote_exploration_subscription_id
                )
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                self._bind_remote_exploration_updates(
                    provider,
                    rollback_local=(
                        subscription_id,
                        callback,
                        remote_subscription_id_before,
                    ),
                )
            return subscription_id

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                removed = self._exploration_update_subscribers.pop(
                    int(subscription_id),
                    None,
                )
                if removed is None:
                    return False
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked(
                        kind="exploration",
                        subscription_id=int(subscription_id),
                    )
                )
                if self._exploration_update_subscribers:
                    remote_subscription_id = None
                    provider = None
                else:
                    remote_subscription_id = (
                        self._remote_exploration_subscription_id
                    )
                    self._remote_exploration_subscription_id = None
                    provider = self._remote_provider
                    delivery_tickets += (
                        self._invalidate_subscription_delivery_tickets_locked(
                            kind="exploration_source",
                            provider=provider,
                        )
                    )
        if remote_subscription_id is not None and provider is not None:
            try:
                provider.unsubscribe_exploration_updates(remote_subscription_id)
            except Exception:
                LOGGER.exception(
                    "取消远端 exploration subscription=%s 失败。",
                    remote_subscription_id,
                )
        self._wait_for_subscription_deliveries(delivery_tickets)
        return True

    def _bind_remote_exploration_updates(
        self,
        provider,
        *,
        rollback_local=None,
    ) -> bool:
        """把本地 fanout 绑定到 provider 的唯一后台状态源。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                if (
                    provider is not self._remote_provider
                    or not self._exploration_update_subscribers
                ):
                    return False
                if self._remote_exploration_subscription_id is not None:
                    return True
                remote_token = _capture_remote_operation_token_locked(
                    self,
                    provider,
                )
                if remote_token is None:
                    return False
                def deliver_exploration_source(
                    payload,
                    token=remote_token,
                ):
                    return self._handle_remote_exploration_update(
                        payload,
                        remote_token=token,
                    )

                source_ticket = self._new_subscription_delivery_ticket(
                    kind="exploration_source",
                    subscription_id=None,
                    callback=deliver_exploration_source,
                    provider=provider,
                )

                def remote_callback(payload):
                    return self._invoke_subscription_delivery(
                        source_ticket,
                        payload,
                    )

                try:
                    remote_subscription_id = (
                        provider.subscribe_exploration_updates(remote_callback)
                    )
                except Exception:
                    if rollback_local is not None:
                        (
                            rollback_subscription_id,
                            rollback_callback,
                            remote_subscription_id_before,
                        ) = rollback_local
                        if (
                            self._exploration_update_subscribers.get(
                                rollback_subscription_id
                            )
                            is rollback_callback
                        ):
                            self._exploration_update_subscribers.pop(
                                rollback_subscription_id,
                                None,
                            )
                        if (
                            self._remote_exploration_subscription_id
                            != remote_subscription_id_before
                        ):
                            self._remote_exploration_subscription_id = (
                                remote_subscription_id_before
                            )
                    self._retire_subscription_delivery_ticket(source_ticket)
                    raise
                if remote_subscription_id is None:
                    self._retire_subscription_delivery_ticket(source_ticket)
                    return False
                self._remote_exploration_subscription_id = int(
                    remote_subscription_id
                )
                self._publish_subscription_delivery_ticket(
                    source_ticket,
                    subscription_id=int(remote_subscription_id),
                )
                return True

    def _handle_remote_exploration_update(
        self,
        payload,
        *,
        remote_token: Optional[_RemoteOperationToken] = None,
    ) -> None:
        if not isinstance(payload, dict):
            LOGGER.warning("忽略非 dict 的 geometry exploration 状态更新。")
            return
        try:
            state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(payload)
        except Exception:
            LOGGER.exception("geometry exploration 状态更新无法转换为平台契约。")
            return
        request_id = str(payload.get("request_id") or "").strip()
        committed = False
        with self._exploration_loop_lock:
            if (
                remote_token is not None
                and not _remote_operation_token_is_current_locked(
                    self,
                    remote_token,
                )
            ):
                LOGGER.info("忽略旧 geometry remote 代际的 exploration update。")
                return
            current = self._external_exploration_request
            current_id = str(getattr(current, "request_id", "") or "").strip()
            if not request_id:
                LOGGER.info("忽略缺少 request_id 的 geometry exploration update。")
                return
            recovered_request = None
            if not current_id and state.status in {
                ExplorationStatus.ACCEPTED,
                ExplorationStatus.RUNNING,
            }:
                recovered_request = ExploreAreaRequest(
                    request_id=request_id,
                    strategy="wavefront",
                    metadata={
                        "map_version_id": state.metadata.get("map_version_id"),
                        "map_revision": state.metadata.get("map_revision"),
                        "recovered_from_native_state": True,
                    },
                )
                current = recovered_request
                current_id = request_id
            if current_id != request_id:
                LOGGER.info(
                    "忽略跨 exploration request 的迟到状态 update_request_id=%s current_request_id=%s",
                    request_id or None,
                    current_id or None,
                )
                return
            frontier_metadata, _ = self._external_debug_frontier_metadata()
            metadata = {**frontier_metadata, **dict(state.metadata)}
            strategy = state.strategy or str(
                getattr(current, "strategy", "") or ""
            ).strip() or None
            state = state.model_copy(
                update={
                    "strategy": strategy,
                    "metadata": metadata,
                },
                deep=True,
            )
            committed = _commit_remote_exploration_state_locked(
                self,
                state,
                request_id=request_id,
                request=recovered_request,
                map_version_id=(
                    str(state.metadata.get("map_version_id") or "").strip()
                    or None
                ),
            )
            if not committed:
                LOGGER.info("忽略无法原子提交的 geometry exploration update。")
        if committed:
            if remote_token is not None:
                self._notify_remote_exploration_update(
                    state,
                    remote_token=remote_token,
                )
            else:
                self._notify_exploration_update(state)

    def start_platform_exploration(self, request) -> bool:
        """兼容平台入口；实际启动统一委托 provider bundle。"""

        return bool(self.providers.start_exploration(request))

    def stop_platform_exploration(self) -> bool:
        """兼容平台入口；实际停止走 correlated native RPC。"""

        return bool(self.providers.stop_exploration())

    # ------------------------------------------------------------------
    # 导航入口（capability navigation.navigate_to_pose 调用层）
    # ------------------------------------------------------------------

    def _dispatch_navigate(
        self,
        pose: Optional[object] = None,
        *,
        goal: Optional[object] = None,
    ) -> bool:
        """经当前 GeometryRuntimeProvider gRPC backend 提交导航目标。"""

        from contracts.navigation import NavigationGoal

        if not self._is_geometry_profile_enabled():
            return False
        backend = self._resolve_navigation_backend()
        if backend is None:
            LOGGER.warning(
                "_dispatch_navigate backend_unavailable pose=%s",
                getattr(pose, "frame_id", None),
            )
            return False

        target_pose = None
        if goal is not None and hasattr(goal, "target_pose"):
            target_pose = getattr(goal, "target_pose", None)
        elif pose is not None and hasattr(pose, "position"):
            target_pose = pose

        goal_id = ""
        if goal is not None:
            goal_id = str(getattr(goal, "goal_id", "") or "")
        if not goal_id:
            goal_id = "dispatch_navigate:%d" % int(time.time() * 1000)

        navigation_goal = NavigationGoal(
            goal_id=goal_id,
            target_pose=target_pose,
            metadata={
                "dispatch_source": "_dispatch_navigate",
                "transport": "geometry_runtime_grpc",
            },
        )
        try:
            return bool(backend.set_goal(navigation_goal))
        except Exception:
            LOGGER.warning(
                "_dispatch_navigate backend.set_goal 异常 goal_id=%s",
                goal_id,
                exc_info=True,
            )
            return False

    def sync_platform_exploration_after_task(self) -> bool:
        """兼容平台入口；只同步 native runtime 已给出的终态。"""

        return bool(self.providers.sync_exploration_after_task())

    def report_platform_exploration_hit_stop(self, reason: str):
        """请求停止当前探索，但绝不合成后端尚未给出的终态。"""

        if not self._is_geometry_profile_enabled():
            return None
        with self._exploration_loop_lock:
            remote = getattr(self, "_remote_provider", None)
            remote_token = (
                _capture_remote_operation_token_locked(self, remote)
                if remote is not None
                else None
            )
            current = getattr(
                self,
                "_external_exploration_request",
                None,
            )
        if remote_token is not None:
            request_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if not request_id:
                raise RuntimeError(
                    "hit-stop 缺少当前探索 request_id，拒绝非关联停止。"
                )
            result = remote.stop_exploration(request_id=request_id)
            if not isinstance(result, dict):
                raise RuntimeError("hit-stop 未收到 geometry runtime 停止确认。")
            state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(result)
            terminal_id = str(state.current_request_id or "").strip()
            with self._exploration_loop_lock:
                if not _remote_operation_token_is_current_locked(
                    self,
                    remote_token,
                ):
                    raise RuntimeError(
                        "hit-stop 返回期间 geometry remote 代际已切换。"
                    )
                current = getattr(
                    self,
                    "_external_exploration_request",
                    None,
                )
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if current_id != request_id:
                    existing_terminal = getattr(
                        self,
                        "_external_exploration_terminal_state",
                        None,
                    )
                    existing_id = str(
                        getattr(existing_terminal, "current_request_id", "")
                        or ""
                    ).strip()
                    existing_metadata = dict(
                        getattr(existing_terminal, "metadata", {}) or {}
                    )
                    if (
                        not current_id
                        and _is_complete_exploration_terminal(
                            existing_terminal,
                            request_id=request_id,
                            safe_stop_only=True,
                        )
                    ):
                        return existing_terminal.model_copy(deep=True)
                    raise RuntimeError(
                        "hit-stop 返回期间 exploration request 已切换。"
                    )
                if terminal_id != request_id:
                    raise RuntimeError(
                        "hit-stop 返回的 request_id 与活动探索不匹配。"
                    )
                metadata = dict(state.metadata)
                metadata.update({
                    "hit_stop_requested": True,
                    "hit_stop_reason": str(reason or "").strip()
                    or "探索命中停机。",
                })
                state = state.model_copy(
                    update={"metadata": metadata},
                    deep=True,
                )
                if not _commit_remote_exploration_state_locked(
                    self,
                    state,
                    request_id=request_id,
                ):
                    raise RuntimeError(
                        "hit-stop 返回状态无法按 exact request 原子提交。"
                    )
            self._notify_exploration_update(state)
            return state
        raise RuntimeError(
            "native geometry remote 已失联，hit-stop 拒绝回退 legacy 终态。"
        )

    @staticmethod
    def _remove_mapping_database_files(db_path: Path) -> None:
        resolved = db_path.expanduser()
        for candidate in (
            resolved,
            Path(str(resolved) + "-journal"),
            Path(str(resolved) + "-wal"),
            Path(str(resolved) + "-shm"),
        ):
            try:
                if candidate.is_file():
                    candidate.unlink()
            except Exception:
                LOGGER.debug("删除建图数据库文件失败：%s", candidate, exc_info=True)

    def _start_external_script_exploration(self, request: ExploreAreaRequest) -> bool:
        """【Phase 6 legacy/manual】外部脚本探索启动入口。

        平台默认探索路径已切到 `_remote_provider` (GeometryRuntimeProvider) →
        gRPC `StartExploration` → Rust `nuwax-exploration` controller。本方法
        仅在显式 legacy/manual 入口（如历史验收脚本、外部硬件桥接）时被
        `_handle_explore_area` 走 legacy 路径时调用。不再作为默认路径。
        """
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        command = self._build_external_exploration_command(request)
        db_path = self._extract_external_exploration_database_path(command)
        if db_path:
            self._remove_mapping_database_files(Path(str(db_path)))
        duration_limit = self._external_exploration_duration_limit_sec(request)
        now = time.monotonic()
        with self._exploration_loop_lock:
            self._external_exploration_request = request.model_copy(deep=True)
            self._external_exploration_started_at_monotonic = now
            self._external_exploration_started_at_iso = _utc_now_iso()
            self._external_exploration_deadline_monotonic = None if duration_limit is None else now + duration_limit
            self._external_exploration_database_path = db_path
            self._external_exploration_terminal_state = None
            self._external_geometry_session_mode = "explore"
        # 第 8 条判据 reset 同步（spec exploration-completion-criteria "reset 同步"）：
        # 探索会话切换时必须 reset loop_closure_state + explore_status_done +
        # external_exploration_terminal_state，避免上一会话缓存串味。
        self._reset_exploration_completion_state()
        # commit 2 §2.4：幂等校验——若 observer 当前 session_mode 已为 "explore"，
        # 跳过显式 set_session_mode("explore") 副作用（避免重复 reset loop_closure_state）；
        # 第二次双调按原规约保留，确保 caller 端语义稳定。
        _observer_for_idempotent_explore = self._external_debug_observer
        if _observer_for_idempotent_explore is None or _observer_for_idempotent_explore.get_current_session_mode() != "explore":
            self._on_external_geometry_session_mode_changed("explore")
        self._on_external_geometry_session_mode_changed("explore")
        if current_process is not None:
            self._managed_processes.pop(process_name, None)
            current_process.stop()
            time.sleep(3.0)
        self._map_stream_tracker = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=True)
        process = self._managed_processes.get(process_name)
        if process is not None:
            with self._exploration_loop_lock:
                self._external_exploration_log_offset = _capture_log_file_offset(process.log_path)
        if process is None or not process.is_running():
            self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_explore_start_failed",
                message="GO2 几何栈原生探索脚本启动失败。",
                notify_compat=False,
                process_status=process.status() if process is not None else None,
                kill_geometry_process=True,
            )
            return False
        self._record_runtime_diagnostic(
            "external_go2_exploration_started",
            "explore_area 已委托 GO2 几何栈脚本执行：%s" % process.command_display(),
        )
        return True

    def _stop_external_script_exploration(self, *, cancel_navigation: bool, notify_compat: bool) -> bool:
        with self._exploration_loop_lock:
            has_request = self._external_exploration_request is not None
            terminal_state = self._external_exploration_terminal_state
        if not has_request and terminal_state is None and not self._external_geometry_process_is_exploration():
            return False
        if terminal_state is not None:
            return True
        if cancel_navigation:
            self._cancel_platform_exploration_navigation()
        self._finish_external_script_exploration(
            status=ExplorationStatus.CANCELLED,
            reason="manual_cancel",
            message="GO2 几何栈原生探索任务已取消；共享 nuwax-geometry-runtime 仍保持运行，请使用 stop_mapping 结束几何栈。",
            notify_compat=notify_compat,
            kill_geometry_process=False,
        )
        return True

    def _kill_external_geometry_process(self) -> Optional[Dict[str, object]]:
        process_name = self._external_process_name()
        process = self._managed_processes.pop(process_name, None)
        if process is None:
            return None
        process_status = process.status()
        if process.is_running():
            process.stop(
                timeout_sec=15.0,
                graceful_launch_shutdown=True,
                graceful_timeout_sec=25.0,
            )
            time.sleep(3.0)
            process_status = process.status()
        with self._exploration_loop_lock:
            self._external_geometry_session_mode = None
        return process_status

    def _finish_external_script_exploration(
        self,
        *,
        status: ExplorationStatus,
        reason: str,
        message: str,
        notify_compat: bool,
        process_status: Optional[Dict[str, object]] = None,
        kill_geometry_process: bool = False,
        completion_reasons: Optional[List[str]] = None,
    ) -> ExplorationState:
        process_name = self._external_process_name()
        process = self._managed_processes.get(process_name)
        if kill_geometry_process:
            process_status = self._kill_external_geometry_process() or process_status
            process = None
        elif process_status is None and process is not None:
            process_status = process.status()
        with self._exploration_loop_lock:
            request = self._external_exploration_request
        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=None,
            reason=reason,
            terminal=True,
        )
        frontier_metadata, _ = self._external_debug_frontier_metadata()
        if frontier_metadata:
            metadata.update(frontier_metadata)
        metadata["exploration_terminal"] = True
        metadata["terminal_status"] = str(getattr(status, "value", status) or "succeeded")
        metadata["status_text"] = self._terminal_explorer_status_text(str(reason or ""), message)
        # 第 8 条判据新增：reasons 命名空间前缀（spec exploration-completion-criteria）
        # 仅当显式传入 completion_reasons 时覆盖；旧调用点不传则留空 list，保留兼容。
        if completion_reasons is not None:
            metadata["reasons"] = list(completion_reasons)
        else:
            metadata.setdefault("reasons", [])
        metadata.pop("waiting_for", None)
        metadata.pop("waiting_detail", None)
        if not kill_geometry_process and self._external_geometry_stack_is_running():
            metadata["geometry_stack_kept_alive"] = True
            metadata["geometry_stack_stop_via"] = "stop_mapping"
            metadata["geometry_stack_running"] = True
        else:
            metadata["geometry_stack_running"] = False
        state = ExplorationState(
            current_request_id=request.request_id if request is not None else None,
            status=status,
            strategy=request.strategy if request is not None else "frontier",
            message=message,
            metadata=metadata,
        )
        with self._exploration_loop_lock:
            self._external_exploration_terminal_state = state
            self._external_exploration_request = None
            self._external_exploration_deadline_monotonic = None
            self._external_exploration_log_offset = None
            if kill_geometry_process:
                self._external_geometry_session_mode = None
        if notify_compat:
            self._notify_compat_exploration_stopped()
        return state

    def _ensure_mapping_lifecycle_fields(self) -> None:
        if not hasattr(self, "_mapping_lifecycle_lock"):
            self._mapping_lifecycle_lock = threading.RLock()
        if not hasattr(self, "_mapping_session_context"):
            self._mapping_session_context = None

    def start_mapping(
        self,
        *,
        map_name: Optional[str] = None,
        map_version_id: Optional[str] = None,
    ) -> Dict[str, object]:
        """Start or idempotently confirm one native Rust mapping session."""

        if not self._is_geometry_profile_enabled():
            raise RuntimeError("平台几何运行时或 mapping profile 已禁用。")
        if not self._runtime_started:
            self.start()
        provider = self._remote_provider
        if provider is None or not bool(getattr(provider, "is_connected", False)):
            self._start_geometry_runtime_subprocess()
            provider = self._remote_provider
        if provider is None or not bool(getattr(provider, "is_connected", False)):
            raise RuntimeError("nuwax-geometry-runtime gRPC 未连接，无法启动建图。")

        active_version = getattr(self, "_active_map_version", None)
        resolved_map_name = str(
            map_name or (active_version.map_name if active_version is not None else "")
        ).strip()
        if not resolved_map_name:
            raise RuntimeError("启动建图必须提供 map_name。")

        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            current = self._mapping_session_context
            new_session_started = not (
                isinstance(current, dict) and bool(current.get("active"))
            )
            requested_version = str(map_version_id or "").strip()
            if isinstance(current, dict) and bool(current.get("active")):
                if str(current.get("map_name") or "") != resolved_map_name:
                    raise RuntimeError(
                        "已有其它地图的建图会话处于 ACTIVE，拒绝隐式切换。"
                    )
                current_version = str(current.get("map_version_id") or "")
                if requested_version and requested_version != current_version:
                    raise RuntimeError(
                        "探索/建图请求的 map_version_id 与 ACTIVE 建图会话不一致。"
                    )
                session_id = str(current.get("session_id") or "")
                provisional_version = current_version
            else:
                session_id = "mapping:%s" % uuid.uuid4().hex
                provisional_version = requested_version or self._next_exploration_map_version_id(
                    resolved_map_name
                )
                if isinstance(current, dict) and not requested_version:
                    previous_version = str(current.get("map_version_id") or "")
                    if provisional_version == previous_version and provisional_version.startswith(
                        "mapver_"
                    ):
                        try:
                            provisional_version = "mapver_%06d" % (
                                int(provisional_version.split("_", 1)[1]) + 1
                            )
                        except (IndexError, ValueError):
                            pass

            start_rpc = getattr(provider, "start_mapping", None)
            if not callable(start_rpc):
                raise RuntimeError("GeometryRuntimeProvider 未实现 StartMapping RPC。")
            response = start_rpc(
                session_id=session_id,
                map_name=resolved_map_name,
                map_version_id=provisional_version,
            )
            if (
                not isinstance(response, dict)
                or str(response.get("phase") or "") != "ACTIVE"
                or not bool(response.get("active"))
                or str(response.get("session_id") or "") != session_id
                or str(response.get("map_version_id") or "") != provisional_version
            ):
                raise RuntimeError("StartMapping 未确认匹配的 ACTIVE Rust 会话。")
            context = {
                **dict(response),
                "session_id": session_id,
                "map_name": resolved_map_name,
                "map_version_id": provisional_version,
                "active": True,
            }
            # Local state is committed only after Rust confirms ACTIVE.
            self._mapping_session_context = context

        with self._exploration_loop_lock:
            if new_session_started:
                self._external_geometry_session_mode = "mapping"
                self._external_exploration_terminal_state = None
                self._external_exploration_request = None
            self._external_exploration_version_id = provisional_version
        if new_session_started:
            self._map_stream_tracker = {}
        diagnostics_reader = getattr(provider, "get_mapping_diagnostics", None)
        diagnostics = diagnostics_reader() if callable(diagnostics_reader) else None
        return {
            "mapping_launch_mode": "nuwax_geometry_runtime",
            "mapping_transport": "grpc_lcm",
            "mapping_script": None,
            "mapping_command": None,
            "mapping_database_path": None,
            "mapping_version_id": provisional_version,
            "mapping_session_id": session_id,
            "session_generation": int(context.get("session_generation") or 0),
            "revision": int(context.get("revision") or 0),
            "mapping_active": True,
            "map_name": resolved_map_name,
            "geometry_stack_running": True,
            "sidecar_kept_alive": True,
            "mapping_diagnostics": diagnostics,
        }

    def start_external_mapping(self, *, map_name: Optional[str] = None) -> Dict[str, object]:
        """兼容旧 capability 名，实际委托 nuwax-geometry-runtime 建图链。"""

        return self.start_mapping(map_name=map_name)

    def get_geometry_stack_status(self) -> Dict[str, object]:
        """返回 go2.sh 几何栈运行态，供 Agent 区分工作区 mapping 与真实 SLAM 进程。"""

        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        provider = getattr(self, "_remote_provider", None)
        sidecar_running = bool(
            provider is not None and getattr(provider, "is_connected", False)
        )
        running = sidecar_running or bool(process is not None and process.is_running())
        command = list(process.command) if process is not None else []
        launch_mode = None
        if self._command_uses_external_exploration(command):
            launch_mode = "explore"
        elif self._command_uses_external_mapping(command):
            launch_mode = "mapping"
        elif self._command_uses_external_localization(command):
            launch_mode = "localization"
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
            terminal_state = self._external_exploration_terminal_state
            has_active_exploration = self._external_exploration_request is not None
            mapping_database_path = self._external_exploration_database_path
            mapping_version_id = self._external_exploration_version_id
        mapping_context = self.get_mapping_session_context()
        mapping_active = bool(mapping_context and mapping_context.get("active"))
        exploration_active = bool(
            has_active_exploration or session_mode == "explore"
        )
        if exploration_active:
            launch_mode = "explore"
            session_mode = "explore"
        elif mapping_active:
            launch_mode = "mapping"
            session_mode = "mapping"
            mapping_version_id = str(
                mapping_context.get("map_version_id") or ""
            ) or mapping_version_id
        guidance = None
        if not running:
            guidance = (
                "几何栈未运行。遥控建图请调用 start_mapping(map_name)；"
                "自主探索请调用 explore_area(map_name)。"
                "activate_map 仅切换工作区，不会启动 go2.sh。"
            )
        elif launch_mode == "mapping":
            guidance = "遥控建图进行中：用户可遥控移动，完成后 save_active_map_version，再 stop_mapping。"
        elif launch_mode == "explore" and terminal_state is not None:
            guidance = (
                "自主探索已结束，几何与导航运行时仍在运行。"
                "可继续手动导航/遥控，完成后 save_active_map_version，再 stop_mapping。"
            )
        elif launch_mode == "explore" and exploration_active:
            guidance = (
                "自主探索进行中；用 get_task_status 查看 frontier_count 与 message，"
                "不要 relocalize_active_map，也不要因 localization_ready=false 误判卡住。"
                "用户要求结束探索时请调用 stop_exploration。"
            )
        return {
            "geometry_stack_running": running,
            "launch_mode": launch_mode,
            "session_mode": session_mode,
            "process_running": bool(process is not None and process.is_running()),
            "sidecar_running": sidecar_running,
            "mapping_active": mapping_active,
            "mapping_session_id": (
                mapping_context.get("session_id") if mapping_context else None
            ),
            "mapping_session_generation": (
                mapping_context.get("session_generation") if mapping_context else None
            ),
            "process_status": process_status,
            "mapping_database_path": mapping_database_path,
            "mapping_version_id": mapping_version_id,
            "exploration_active": exploration_active,
            "exploration_terminal": terminal_state is not None,
            "teleop_mapping_api": "start_mapping",
            "autonomous_explore_api": "explore_area",
            "stop_exploration_api": "stop_exploration",
            "stop_geometry_stack_api": "stop_mapping",
            "agent_guidance": guidance,
        }

    def stop_mapping(self, *, reason: str = "stop_mapping") -> Dict[str, object]:
        """Stop the exact native mapping session without stopping the sidecar."""

        provider = self._remote_provider
        if provider is None or not bool(getattr(provider, "is_connected", False)):
            raise RuntimeError("nuwax-geometry-runtime gRPC 未连接，无法停止建图。")
        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            context = self._mapping_session_context
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                state_reader = getattr(provider, "get_mapping_state", None)
                context = state_reader() if callable(state_reader) else None
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                raise RuntimeError("当前没有可关联的 Rust mapping session。")
            session_id = str(context.get("session_id") or "")

            exploration_reader = getattr(provider, "get_exploration_state", None)
            if not callable(exploration_reader):
                raise RuntimeError(
                    "停止建图前无法读取 ExplorationState，拒绝让 child navigation 失去 costmap。"
                )
            exploration_state = exploration_reader()
            if not isinstance(exploration_state, dict):
                raise RuntimeError(
                    "停止建图前未获得 ExplorationState，mapping adapter 保持 ACTIVE。"
                )
            exploration_status = str(
                exploration_state.get("status") or ""
            ).strip().upper()
            safe_terminal_statuses = {
                "IDLE",
                "STOPPED",
                "SUCCEEDED",
                "CANCELLED",
            }
            parked_exploration = None
            if exploration_status not in safe_terminal_statuses:
                request_id = str(
                    exploration_state.get("request_id") or ""
                ).strip()
                if not request_id:
                    raise RuntimeError(
                        "active exploration 缺少 exact request_id，拒绝停止 mapping。"
                    )
                exploration_stopper = getattr(provider, "stop_exploration", None)
                if not callable(exploration_stopper):
                    raise RuntimeError(
                        "GeometryRuntimeProvider 未实现 correlated StopExploration。"
                    )
                parked_exploration = exploration_stopper(request_id=request_id)
                if not isinstance(parked_exploration, dict):
                    raise RuntimeError(
                        "StopExploration 未返回 exact terminal，mapping adapter 保持 ACTIVE。"
                    )
                parked_status = str(
                    parked_exploration.get("status") or ""
                ).strip().upper()
                parked_request_id = str(
                    parked_exploration.get("request_id") or ""
                ).strip()
                if (
                    parked_request_id != request_id
                    or parked_status not in safe_terminal_statuses - {"IDLE"}
                ):
                    raise RuntimeError(
                        "StopExploration 未返回 exact terminal，mapping adapter 保持 ACTIVE。"
                    )

            stop_rpc = getattr(provider, "stop_mapping", None)
            if not callable(stop_rpc):
                raise RuntimeError("GeometryRuntimeProvider 未实现 StopMapping RPC。")
            response = stop_rpc(
                session_id=session_id,
                reason=str(reason or "stop_mapping"),
            )
            if (
                not isinstance(response, dict)
                or str(response.get("phase") or "") != "STOPPED"
                or bool(response.get("active"))
                or str(response.get("session_id") or "") != session_id
            ):
                raise RuntimeError("StopMapping 未确认匹配的 STOPPED Rust 会话。")
            self._mapping_session_context = {
                **dict(context),
                **dict(response),
                "active": False,
            }

        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
            mapping_database_path = self._external_exploration_database_path
            self._external_geometry_session_mode = None
        self._record_runtime_diagnostic(
            "native_mapping_stopped",
            "StopMapping 已关闭 Rust adapter ingestion reason=%s session_id=%s"
            % (reason, session_id),
        )
        return {
            "stopped": True,
            "reason": str(reason or "stop_mapping"),
            "session_mode": session_mode,
            "mapping_session_id": session_id,
            "mapping_active": False,
            "idempotent_replay": bool(response.get("idempotent_replay")),
            "geometry_stack_running": True,
            "sidecar_kept_alive": True,
            "exploration_state": None,
            "parked_exploration": parked_exploration,
            "mapping_version_id": str(response.get("map_version_id") or "") or None,
            "mapping_database_path": mapping_database_path,
        }

    def _external_exploration_duration_limit_sec(self, request: ExploreAreaRequest) -> Optional[float]:
        limits: List[float] = []
        if request.max_duration_sec is not None:
            limits.append(float(request.max_duration_sec))
        stop_conditions = dict(getattr(request, "stop_conditions", {}) or {})
        value = stop_conditions.get("max_duration_sec")
        if value is not None and not isinstance(value, bool):
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                parsed = 0.0
            if math.isfinite(parsed) and parsed > 0.0:
                limits.append(parsed)
        return min(limits) if limits else None

    def _external_exploration_elapsed_sec(self) -> float:
        with self._exploration_loop_lock:
            started = self._external_exploration_started_at_monotonic
        if started is None:
            return 0.0
        return max(0.0, time.monotonic() - started)

    @staticmethod
    def _external_process_serves_database(
        command: List[str],
        database_path: Path,
        *,
        localization: bool = False,
    ) -> bool:
        if not command:
            return False
        if localization and not PlatformGeometryRuntime._command_uses_external_localization(command):
            return False
        extracted = PlatformGeometryRuntime._extract_external_exploration_database_path(command)
        if extracted is None and command:
            extracted = str(command[-1])
        if not extracted:
            return False
        try:
            return Path(str(extracted)).expanduser().resolve() == database_path.expanduser().resolve()
        except OSError:
            return False

    @staticmethod
    def _external_process_uses_map_yaml(command: List[str], map_yaml_path: Optional[Path]) -> bool:
        if map_yaml_path is None or not command:
            return False
        idx = 0
        extracted = None
        while idx < len(command):
            item = str(command[idx])
            if item in {"--map-yaml", "--map-yaml-path"} and idx + 1 < len(command):
                extracted = str(command[idx + 1])
                break
            idx += 1
        if not extracted:
            return False
        try:
            return Path(str(extracted)).expanduser().resolve() == map_yaml_path.expanduser().resolve()
        except OSError:
            return False

    @staticmethod
    def _extract_external_exploration_database_path(command: List[str]) -> Optional[str]:
        if not command:
            return None
        idx = 0
        result = None
        while idx < len(command):
            item = str(command[idx])
            if item == "--database-path" and idx + 1 < len(command):
                result = str(command[idx + 1])
                idx += 2
                continue
            if item in {"--map-yaml", "--map-yaml-path"} and idx + 1 < len(command):
                idx += 2
                continue
            if idx > 0 and not item.startswith("-") and ":=" not in item:
                result = item
            idx += 1
        return result

    def _build_external_exploration_metadata(
        self,
        *,
        request: Optional[ExploreAreaRequest],
        process_status: Optional[Dict[str, object]],
        waiting_for: Optional[str],
        reason: Optional[str] = None,
        terminal: bool = False,
    ) -> Dict[str, object]:
        duration_limit = self._external_exploration_duration_limit_sec(request) if request is not None else None
        elapsed = self._external_exploration_elapsed_sec()
        with self._exploration_loop_lock:
            started_at = self._external_exploration_started_at_iso
            deadline = self._external_exploration_deadline_monotonic
            database_path = self._external_exploration_database_path
            exploration_version_id = self._external_exploration_version_id
        remaining = None
        if deadline is not None:
            remaining = max(0.0, deadline - time.monotonic())
        metadata: Dict[str, object] = {
            "exploration_mode": "go2_rtabnav2_script",
            "exploration_runtime": "external_go2_rtabnav2",
            "exploration_script": str(self._external_script_path()),
            "exploration_process_name": self._external_process_name(),
            "exploration_database_path": database_path,
            "exploration_version_id": exploration_version_id,
            "exploration_launch_mode": "explore",
            "started_at": started_at,
            "elapsed_time_sec": elapsed,
            "max_duration_sec": duration_limit,
            "remaining_time_sec": remaining,
            "terminal": bool(terminal),
            "exploration_terminal": bool(terminal),
            "terminal_reason": reason,
            "waiting_for": waiting_for,
            "external_process": process_status,
            "platform_frontier_runtime_bypassed": True,
            "native_go2_explore_lite": True,
            "native_go2_frontier_explorer": False,
        }
        if request is not None:
            metadata.update(
                {
                    "map_name": request.map_name,
                    "center_pose": request.center_pose.model_dump(mode="json") if request.center_pose is not None else None,
                    "radius_m": request.radius_m,
                    "target_name": request.target_name,
                    "stop_conditions": dict(request.stop_conditions),
                }
            )
        if process_status is not None:
            metadata["exploration_command"] = process_status.get("command")
            metadata["exploration_log_path"] = process_status.get("log_path")
        return metadata

    def _notify_remote_exploration_update(
        self,
        state: ExplorationState,
        *,
        remote_token: _RemoteOperationToken,
    ) -> None:
        context = getattr(self, "_exploration_delivery_context", None)
        if context is None:
            context = threading.local()
            self._exploration_delivery_context = context
        missing = object()
        previous = getattr(context, "remote_token", missing)
        context.remote_token = remote_token
        try:
            self._notify_exploration_update(state)
        finally:
            if previous is missing:
                try:
                    del context.remote_token
                except AttributeError:
                    pass
            else:
                context.remote_token = previous

    def _notify_exploration_update(self, state: ExplorationState) -> None:
        context = getattr(self, "_exploration_delivery_context", None)
        remote_token = (
            getattr(context, "remote_token", None)
            if context is not None
            else None
        )
        if context is not None and remote_token is not None:
            try:
                del context.remote_token
            except AttributeError:
                pass
        delivery_provider = None
        delivery_generation = None
        if remote_token is not None:
            delivery_provider, delivery_generation = remote_token
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                deliveries = []
                for subscription_id, callback in tuple(
                    self._exploration_update_subscribers.items()
                ):
                    ticket = self._new_subscription_delivery_ticket(
                        kind="exploration",
                        subscription_id=subscription_id,
                        callback=callback,
                        provider=delivery_provider,
                        provider_generation=delivery_generation,
                    )
                    self._publish_subscription_delivery_ticket(ticket)
                    deliveries.append(ticket)
        for ticket in deliveries:
            try:
                self._invoke_subscription_delivery(
                    ticket,
                    state.model_copy(deep=True),
                    retire=True,
                )
            except Exception:
                pass

    def get_platform_exploration_state(self) -> ExplorationState:
        """读取 native exploration runtime 的权威快照。

        该入口只做 gRPC snapshot -> 平台契约转换，不推进任务、不触发 fanout。
        状态变化由 GeometryRuntimeProvider 的唯一轮询 worker 推送。
        """

        def unavailable(message: str, reason: str) -> ExplorationState:
            return ExplorationState(
                status=ExplorationStatus.IDLE,
                message=message,
                metadata={
                    "provider": "platform_geometry_runtime",
                    "state_source": "geometry_runtime_grpc",
                    "runtime_ready": False,
                    "unavailable_reason": reason,
                },
            )

        if not self._is_geometry_profile_enabled():
            return unavailable(
                "platform_geometry profile 已禁用。",
                "profile_disabled",
            )

        backend = self._resolve_exploration_backend()
        if backend is None:
            return unavailable(
                "GeometryRuntimeProvider 未连接，无法读取探索状态。",
                "provider_unavailable",
            )

        with self._exploration_loop_lock:
            remote_token = _capture_remote_operation_token_locked(self, backend)
        if remote_token is None:
            return unavailable(
                "GeometryRuntimeProvider 连接已失效，无法读取探索状态。",
                "provider_generation_unavailable",
            )

        try:
            payload = backend.get_exploration_state()
        except Exception:
            LOGGER.exception("读取 native exploration 状态失败。")
            return unavailable(
                "读取 native exploration 状态失败。",
                "state_rpc_failed",
            )
        if not isinstance(payload, dict):
            return unavailable(
                "native exploration runtime 未返回有效状态。",
                "state_payload_unavailable",
            )

        state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(payload)
        with self._exploration_loop_lock:
            if not _remote_operation_token_is_current_locked(self, remote_token):
                return unavailable(
                    "读取探索状态期间 GeometryRuntimeProvider 连接代际已切换。",
                    "provider_generation_changed",
                )
            current_request = self._external_exploration_request

        metadata = dict(state.metadata)
        if hasattr(self, "_external_debug_frontier_payload"):
            frontier_metadata, _ = self._external_debug_frontier_metadata()
            metadata = {**frontier_metadata, **metadata}

        current_request_id = str(
            getattr(current_request, "request_id", "") or ""
        ).strip()
        state_request_id = str(state.current_request_id or "").strip()
        if current_request_id and current_request_id == state_request_id:
            metadata["request_context_match"] = True
            strategy = state.strategy or str(
                getattr(current_request, "strategy", "") or ""
            ).strip() or None
        else:
            metadata["request_context_match"] = not bool(
                current_request_id or state_request_id
            )
            if current_request_id or state_request_id:
                metadata["cached_request_id"] = current_request_id or None
            strategy = state.strategy

        return state.model_copy(
            update={
                "strategy": strategy,
                "metadata": metadata,
            },
            deep=True,
        )

    def _external_debug_frontier_metadata(self) -> Tuple[Dict[str, object], Optional[int]]:
        with self._exploration_loop_lock:
            if getattr(self, "_external_geometry_session_mode", None) != "explore":
                return {}, None
            payload = self._clone_metadata(
                getattr(self, "_external_debug_frontier_payload", None) or {}
            )
        if not payload:
            return {}, None
        latest_frontiers = payload.get("latest_frontiers") if isinstance(payload.get("latest_frontiers"), list) else []
        current_frontier = payload.get("current_frontier") if isinstance(payload.get("current_frontier"), dict) else None
        try:
            frontier_count = int(payload.get("frontier_count")) if payload.get("frontier_count") is not None else len(latest_frontiers)
        except (TypeError, ValueError):
            frontier_count = len(latest_frontiers)
        payload_metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        raw_status_text = str(payload_metadata.get("status_text") or "").strip()
        provisional_meta = {
            "status_text": raw_status_text,
            "external_frontier_debug_observer": {
                "metadata": payload_metadata,
            },
        }
        resolved_status_text = self._resolve_explorer_status_text(provisional_meta) or raw_status_text
        metadata = {
            "latest_frontiers": latest_frontiers,
            "current_frontier": current_frontier,
            "frontier_source": payload.get("source") or "go2_rtabnav2_debug_observer",
            "native_go2_explore_lite": True,
            "native_go2_frontier_explorer": False,
            "status_text": resolved_status_text,
            "exploration_terminal": bool(payload_metadata.get("exploration_terminal")),
            "terminal_status": payload_metadata.get("terminal_status"),
            "planned_path": payload.get("planned_path") if isinstance(payload.get("planned_path"), list) else payload_metadata.get("planned_path"),
            "plan_point_count": payload_metadata.get("plan_point_count"),
            "plan_fresh": payload_metadata.get("plan_fresh"),
            # current_goal 已在源头解析为 NavigationGoal|None，直接透传，不做 dict 类型过滤。
            "current_goal": payload.get("current_goal") if isinstance(payload.get("current_goal"), NavigationGoal) else None,
            "external_frontier_debug_observer": {
                "source_name": payload.get("source_name"),
                "updated_at": payload.get("updated_at"),
                "sequence": payload.get("sequence"),
                "frontier_count": frontier_count,
                "metadata": payload_metadata,
            },
        }
        return metadata, frontier_count

    def _metadata_indicates_exploration_done(
        self, metadata: Dict[str, object]
    ) -> Tuple[bool, List[str]]:
        """判断探索任务是否已结束，并返回 reasons 命名空间前缀列表。

        原 7 条判据路径（marker DONE / explore.status exploration_complete /
        max_duration_sec / external_process_completed / external_process_exited /
        日志扫描 / stop_mapping / hit_stop / manual_cancel） MUST NOT 破坏；
        新增第 8 条：地图已形成闭环（主路径闭环状态 + 辅助信号
        `explore/status exploration_complete`，任一触发即 stop）。

        返回：
        - (False, []) —— 未结束；
        - (True, [reasons...]) —— 已结束，reasons 命名空间前缀（见 spec
          exploration-completion-criteria "reasons 命名空间前缀"）。
        """

        # 原有 7 条判据路径（保持原状，reasons 用旧命名以保持向后兼容）
        if bool(metadata.get("exploration_terminal")) or bool(metadata.get("terminal")):
            return True, ["explorer:exploration_terminal"]
        status_text = str(metadata.get("status_text") or "").upper()
        if "DONE (EXPLORATION FINISHED)" in status_text:
            return True, ["explorer:exploration_finished"]
        observer_meta = metadata.get("external_frontier_debug_observer")
        if isinstance(observer_meta, dict):
            observer_payload = observer_meta.get("metadata") if isinstance(observer_meta.get("metadata"), dict) else {}
            if bool(observer_payload.get("exploration_terminal")):
                return True, ["explorer:exploration_terminal"]
            observer_status = str(observer_payload.get("status_text") or "").upper()
            if "DONE (EXPLORATION FINISHED)" in observer_status:
                return True, ["explorer:exploration_finished"]

        # 第 8 条判据：地图已形成闭环（双信号源，任一触发即 stop）
        # 主路径：observer._loop_closure_state.loop_closure_count >= threshold
        # 辅助信号：observer._explore_status_done（explore/status == "exploration_complete"）
        return self._check_loop_closure_completion()

    def _check_loop_closure_completion(self) -> Tuple[bool, List[str]]:
        """第 8 条判据：双信号源检查（spec exploration-completion-criteria）。"""

        reasons: List[str] = []
        observer = self._external_debug_observer
        # 主路径：/rtabmap/info topic；缺失时降级到 DB SQLite fallback
        triggered_by_topic = False
        if observer is not None:
            state = observer.loop_closure_state
            threshold = self._resolve_loop_closure_threshold()
            if state.loop_closure_count >= threshold and state.source in {"topic", "db_sqlite"}:
                triggered_by_topic = True
                if "mapping:loop_closed" not in reasons:
                    reasons.append("mapping:loop_closed")
        else:
            # 无 observer 时尝试 DB fallback
            count = self._read_loop_closure_count_from_db_with_fallback()
            threshold = self._resolve_loop_closure_threshold()
            if count >= threshold and count > 0:
                triggered_by_topic = True
                if "mapping:loop_closed" not in reasons:
                    reasons.append("mapping:loop_closed")

        # 辅助信号：explore/status == "exploration_complete"
        if observer is not None and observer.explore_status_done:
            if "explorer:no_frontiers" not in reasons:
                reasons.append("explorer:no_frontiers")

        return (bool(reasons), reasons)

    @staticmethod
    def _resolve_loop_closure_threshold() -> int:
        """从 core.settings 解析 threshold；缺失或异常时 fallback 3。"""

        try:
            from core.settings import (
                NAV_LOOP_CLOSURE_THRESHOLD,  # type: ignore
            )
            value = int(NAV_LOOP_CLOSURE_THRESHOLD)
        except Exception:
            value = 3
        if value <= 0:
            return 3
        return value

    def _read_loop_closure_count_from_db_with_fallback(self) -> int:
        """DB SQLite fallback：HIL 第 1 步验证 /rtabmap/info 不存在时启用。"""

        db_path = self._resolve_loop_closure_db_path()
        if db_path is None:
            return 0
        try:
            return int(
                read_loop_closure_from_db(
                    db_path,
                    threshold=self._resolve_loop_closure_threshold(),
                )
            )
        except Exception:
            LOGGER.debug("loop_closure DB fallback 失败。", exc_info=True)
            return 0

    def _resolve_loop_closure_db_path(self) -> Optional[Path]:
        """解析当前探索会话的 RTAB-Map DB 路径。"""

        with self._exploration_loop_lock:
            db_path = self._external_exploration_database_path
        if not db_path:
            return None
        try:
            candidate = Path(str(db_path)).expanduser()
        except Exception:
            return None
        return candidate

    def _reset_exploration_completion_state(self) -> None:
        """重置第 8 条判据相关状态（spec exploration-completion-criteria "reset 同步"）。

        触发时机：`_start_external_script_exploration` 启动新会话时。
        同时清零：observer._loop_closure_state、observer._explore_status_done、
        `external_exploration_terminal_state`。
        """

        observer = self._external_debug_observer
        if observer is not None and hasattr(observer, "reset_loop_closure_state"):
            try:
                observer.reset_loop_closure_state()
            except Exception:
                LOGGER.debug("observer reset_loop_closure_state 失败。", exc_info=True)
        # _external_exploration_terminal_state 已在 _start_external_script_exploration
        # 内部 `_exploration_loop_lock` 段同步置 None；这里做防御性二次清零。

    # ------------------------------------------------------------------
    # 手划探索范围 boundary JSON 写入（spec exploration-boundary §go2.sh --boundary）
    # ------------------------------------------------------------------

    _BOUNDARY_JSON_PATH = Path("/tmp/go2_boundary.json")

    def _maybe_persist_boundary_for_external_command(
        self,
        *,
        command: List[str],
        session: str,
    ) -> Optional[Path]:
        """调用 observer 的 `_compute_boundary_polygon`，把结果序列化到 `/tmp/go2_boundary.json`。

        返回：
        - `Path`: 写入成功且非空 polygon 时返回 `/tmp/go2_boundary.json`；
        - `None`: 无 observer / 无 boundary 缓存 / 异常 / 空 polygon 时返回 None。

        行为约定：
        - 复用 observer 单订阅者约束（不另起订阅 `/map_obstacles`）；
        - 原子写：先写 `.tmp` 再 rename，避免 go2.sh 读到半截文件；
        - 写入失败仅记 warning 日志，不抛异常（避免 mapping 启动被 boundary 失败连累）。
        """

        observer = self._external_debug_observer
        if observer is None:
            LOGGER.debug(
                "boundary_persist_skipped reason=observer_not_started session=%s", session
            )
            return None
        try:
            polygon = observer.latest_boundary_polygon
        except Exception:
            LOGGER.debug("读取 observer.latest_boundary_polygon 失败。", exc_info=True)
            polygon = []
        if not polygon:
            LOGGER.debug(
                "boundary_persist_skipped reason=empty_polygon session=%s", session
            )
            return None
        payload = {
            "polygon": [{"x": float(x), "y": float(y)} for (x, y) in polygon],
            "session": str(session),
            "command_preview": list(command)[:8],
            "generated_at": _utc_now_iso(),
        }
        target = self._BOUNDARY_JSON_PATH
        tmp_path = Path(str(target) + ".tmp")
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            with open(tmp_path, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=2)
                fp.flush()
                try:
                    os.fsync(fp.fileno())
                except Exception:
                    pass
            tmp_path.replace(target)
        except Exception:
            LOGGER.warning(
                "boundary JSON 写入失败 path=%s session=%s",
                target,
                session,
                exc_info=True,
            )
            try:
                if tmp_path.exists():
                    tmp_path.unlink()
            except Exception:
                pass
            return None
        LOGGER.info(
            "boundary JSON 已写入 path=%s vertices=%d session=%s",
            target,
            len(payload["polygon"]),
            session,
        )
        return target

    def _external_exploration_terminal_from_frontier_observer(self) -> Optional[ExplorationState]:
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            payload = self._clone_metadata(self._external_debug_frontier_payload or {})
        if request is None or not payload:
            return None
        payload_metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        frontier_metadata, _ = self._external_debug_frontier_metadata()
        done, reasons = self._metadata_indicates_exploration_done({**frontier_metadata, **payload_metadata})
        if not done:
            return None
        terminal_status = str(payload_metadata.get("terminal_status") or "succeeded").lower()
        status = ExplorationStatus.SUCCEEDED if terminal_status == "succeeded" else ExplorationStatus.FAILED
        status_text = str(payload_metadata.get("status_text") or "").strip()
        message = "GO2 几何栈原生探索已完成。"
        if reasons:
            message = "GO2 几何栈原生探索已完成：%s" % ", ".join(reasons)
        elif status_text:
            message = "GO2 几何栈原生探索已完成：%s" % status_text
        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        return self._finish_external_script_exploration(
            status=status,
            reason="external_explorer_frontier_done",
            message=message,
            notify_compat=False,
            process_status=process_status,
            kill_geometry_process=False,
            completion_reasons=reasons,
        )

    def _should_scan_external_exploration_log(self) -> bool:
        now = time.monotonic()
        if now - self._external_exploration_last_log_scan_at < _EXTERNAL_EXPLORATION_LOG_SCAN_MIN_INTERVAL_SEC:
            return False
        self._external_exploration_last_log_scan_at = now
        return True

    def _build_fast_external_exploration_state_from_frontier(self) -> ExplorationState:
        """从已缓存的 frontier observer payload 构建轻量探索态，供 ROS 热路径推送。"""

        with self._exploration_loop_lock:
            request = self._external_exploration_request
            terminal_state = self._external_exploration_terminal_state
        if terminal_state is not None:
            return terminal_state.model_copy(deep=True)
        if request is None:
            return ExplorationState(status=ExplorationStatus.IDLE)

        frontier_metadata, frontier_count = self._external_debug_frontier_metadata()
        explorer_message = self._external_explorer_headline_message(
            frontier_metadata,
            frontier_count=frontier_count,
        )
        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        waiting_for = None
        message = explorer_message or "GO2 几何栈原生 frontier 探索运行中。"
        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=waiting_for,
            reason=None,
            terminal=False,
        )
        if frontier_metadata:
            metadata.update(frontier_metadata)
        return ExplorationState(
            current_request_id=request.request_id,
            status=ExplorationStatus.RUNNING,
            strategy=request.strategy,
            frontier_count=frontier_count,
            message=message,
            metadata=metadata,
        )

    def _get_external_script_exploration_state(self) -> ExplorationState:
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            terminal_state = self._external_exploration_terminal_state
            deadline = self._external_exploration_deadline_monotonic
        if terminal_state is not None:
            return self._augment_exploration_state_with_mapping_diagnostics(terminal_state)
        if request is None:
            return ExplorationState(status=ExplorationStatus.IDLE)

        frontier_terminal_state = self._external_exploration_terminal_from_frontier_observer()
        if frontier_terminal_state is not None:
            return self._augment_exploration_state_with_mapping_diagnostics(frontier_terminal_state)

        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        process_running = bool(process is not None and process.is_running())
        elapsed = self._external_exploration_elapsed_sec()
        if deadline is not None and time.monotonic() >= deadline:
            state = self._finish_external_script_exploration(
                status=ExplorationStatus.SUCCEEDED,
                reason="max_duration_sec",
                message="GO2 几何栈原生探索已达到最大时长，任务已结束。",
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        if not process_running:
            exit_code = process_status.get("exit_code") if isinstance(process_status, dict) else None
            if exit_code == 0:
                state = self._finish_external_script_exploration(
                    status=ExplorationStatus.SUCCEEDED,
                    reason="external_process_completed",
                    message="GO2 几何栈原生探索脚本已正常退出。",
                    notify_compat=False,
                    process_status=process_status,
                    kill_geometry_process=False,
                )
                return self._augment_exploration_state_with_mapping_diagnostics(state)
            state = self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_process_exited",
                message="GO2 几何栈原生探索脚本已退出，exit_code=%s。" % exit_code,
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        log_terminal = None
        if self._should_scan_external_exploration_log():
            log_terminal = self._external_exploration_terminal_from_process_log(process)
        if log_terminal is not None:
            state = self._finish_external_script_exploration(
                status=log_terminal["status"],
                reason=str(log_terminal["reason"]),
                message=str(log_terminal["message"]),
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        frontier_metadata, frontier_count = self._external_debug_frontier_metadata()
        explorer_message = self._external_explorer_headline_message(
            frontier_metadata,
            frontier_count=frontier_count,
        )

        diagnostics = self.get_live_mapping_diagnostics()
        runtime_status = diagnostics.get("runtime_status") if isinstance(diagnostics.get("runtime_status"), dict) else {}
        mapping_readiness = diagnostics.get("mapping_readiness")
        if not isinstance(mapping_readiness, dict):
            mapping_readiness = {}
        navigation_status = (
            runtime_status.get("navigation")
            if isinstance(runtime_status.get("navigation"), dict)
            else self._build_navigation_status()
        )
        waiting_for = None
        message = "GO2 几何栈原生 frontier 探索运行中。"
        exploration_signals_active = bool(
            explorer_message is not None
            or (frontier_count is not None and frontier_count > 0)
            or bool((frontier_metadata or {}).get("plan_fresh"))
        )
        if exploration_signals_active:
            message = explorer_message or message
        elif diagnostics.get("waiting_for"):
            waiting_for = str(diagnostics.get("waiting_for"))
            message = str(diagnostics.get("message") or "等待 GO2 几何栈探索输入链路 ready。")
        elif not bool(mapping_readiness.get("localization_ready")):
            waiting_for = str(mapping_readiness.get("waiting_for") or "geometry_runtime_localization")
            message = str(mapping_readiness.get("message") or "等待 GO2 几何栈 SLAM/TF ready。")
        elif not bool(navigation_status.get("ready")):
            waiting_for = self._normalize_exploration_waiting_for(
                navigation_status.get("waiting_for") or "nuwax_navigation_runtime"
            )
            message = str(navigation_status.get("message") or "等待 nuwax-geometry-runtime nav_runtime ready。")
        elif explorer_message:
            message = explorer_message

        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=waiting_for,
            reason=None,
            terminal=False,
        )
        metadata["mapping_chain_diagnostics"] = diagnostics
        metadata["mapping_readiness"] = self._clone_metadata(mapping_readiness)
        metadata["navigation_readiness"] = self._clone_metadata(navigation_status)
        metadata["elapsed_time_sec"] = elapsed
        if waiting_for:
            metadata["waiting_detail"] = message
        if frontier_metadata:
            metadata.update(frontier_metadata)
        return self._augment_exploration_state_with_mapping_diagnostics(
            ExplorationState(
                current_request_id=request.request_id,
                status=ExplorationStatus.RUNNING,
                strategy=request.strategy,
                frontier_count=frontier_count,
                message=message,
                metadata=metadata,
            )
        )

    def _read_external_exploration_log_texts(self, process: Optional[_ManagedProcess]) -> List[str]:
        if process is None:
            return []
        texts: List[str] = []
        with self._exploration_loop_lock:
            log_offset = self._external_exploration_log_offset
        try:
            if log_offset is not None:
                texts.append(_read_log_from_offset(process.log_path, log_offset))
            texts.append(_read_recent_log(process.log_path, max_bytes=65536))
        except Exception:
            return texts
        deduped: List[str] = []
        seen: Set[str] = set()
        for text in texts:
            normalized = str(text or "")
            if not normalized or normalized in seen:
                continue
            seen.add(normalized)
            deduped.append(normalized)
        for text in list(deduped):
            deduped.extend(self._read_external_frontier_ros_logs(text))
        return deduped

    def _external_exploration_terminal_from_process_log(self, process: Optional[_ManagedProcess]) -> Optional[Dict[str, object]]:
        best: Optional[Dict[str, object]] = None
        for candidate in self._read_external_exploration_log_texts(process):
            terminal = self._external_exploration_terminal_from_log_text(candidate)
            if terminal is None:
                continue
            if terminal.get("reason") == "external_explorer_log_complete":
                return terminal
            best = terminal
        return best

    @staticmethod
    def _terminal_explorer_status_text(reason: str, message: str) -> str:
        normalized_message = str(message or "").lower()
        if "done" in normalized_message or "exploration finished" in normalized_message:
            return "DONE (exploration finished)"
        if reason in {
            "external_explorer_frontier_done",
            "external_explorer_log_complete",
            "external_explorer_terminal_status",
        }:
            return "DONE (exploration finished)"
        if reason == "external_explorer_log_navigation_failures_terminal":
            return "DONE (exploration finished)"
        return "DONE (exploration finished)"

    def _read_external_frontier_ros_logs(self, launch_log_text: str) -> List[str]:
        pids: List[str] = []
        pid_patterns = (
            r"\[frontier_explorer\.py-\d+\]: process started with pid \[(\d+)\]",
            r"\[explore-\d+\]: process started with pid \[(\d+)\]",
            r"\[explore_node-\d+\]: process started with pid \[(\d+)\]",
        )
        for pattern in pid_patterns:
            for match in re.finditer(pattern, launch_log_text or ""):
                pids.append(match.group(1))
        if not pids:
            return []
        ros_log_root = Path.home() / ".ros" / "log"
        texts: List[str] = []
        for pid in dict.fromkeys(pids):
            for path in sorted(ros_log_root.glob(f"python3_{pid}_*.log"), key=lambda item: item.stat().st_mtime if item.exists() else 0.0, reverse=True)[:2]:
                try:
                    texts.append(_read_recent_log(path, max_bytes=65536))
                except Exception:
                    continue
        return texts

    def _external_exploration_terminal_from_log_text(self, text: str) -> Optional[Dict[str, object]]:
        if not text:
            return None
        latest_start = text.rfind("] start ")
        if latest_start >= 0:
            text = text[latest_start:]
        normalized = text.lower()
        complete_markers = (
            "exploration complete",
            "exploration_complete",
            "exploration reached a terminal state",
            "done (exploration finished)",
            "no frontiers found. exploration complete",
            "所有前沿已尝试完毕",
            "frontier_explorer is idle",
            "shutting down frontier_explorer node",
        )
        complete_pos = max((normalized.rfind(marker) for marker in complete_markers), default=-1)
        if complete_pos >= 0:
            return {
                "status": ExplorationStatus.SUCCEEDED,
                "reason": "external_explorer_log_complete",
                "message": "GO2 几何栈原生探索已完成。",
            }
        return None

    def _read_exploration_pose(self):
        navigation_backend = self._resolve_navigation_backend()
        reader = getattr(navigation_backend, "_read_current_pose", None) if navigation_backend is not None else None
        if callable(reader):
            pose = reader()
            if pose is not None:
                return pose
        localization_backend = self._resolve_localization_backend()
        if localization_backend is None:
            return None
        try:
            return localization_backend.get_current_pose()
        except Exception:
            return None

    def _normalize_exploration_waiting_for(self, value: object) -> str:
        normalized = str(value or "").strip() or "navigation_backend"
        return {
            "lifecycle": "nav2_lifecycle",
            "action": "nav2_action",
            "ros2_action": "nav2_action",
            "tf": "tf",
            "costmap": "costmap",
            "backend": "navigation_backend",
            "navigation": "navigation_backend",
        }.get(normalized, normalized)

    def _notify_compat_exploration_stopped(self) -> None:
        backend = self._resolve_backend(ExplorationDataPlane, ExplorationProvider)
        if backend is None:
            return
        stopper = getattr(backend, "stop_exploration", None)
        if not callable(stopper):
            return
        try:
            stopper()
        except Exception:
            pass

    def _notify_compat_navigation_cancelled(self) -> None:
        backend = self._resolve_backend(NavigationDataPlane, NavigationProvider)
        if backend is None:
            return
        canceller = getattr(backend, "cancel_goal", None)
        if not callable(canceller):
            return
        try:
            state = backend.get_navigation_state()
            goal_id = str(getattr(state, "current_goal_id", "") or "").strip()
            if goal_id:
                canceller(request_id=goal_id)
        except Exception:
            pass

    def _cancel_platform_exploration_navigation(self, *, current_goal=None) -> None:
        goal = current_goal
        if goal is None:
            return
        navigation_backend = self._resolve_navigation_backend()
        if navigation_backend is None:
            return
        navigation_state = navigation_backend.get_navigation_state()
        if navigation_state.current_goal_id == goal.goal_id and navigation_state.status not in {
            NavigationStatus.IDLE,
            NavigationStatus.SUCCEEDED,
            NavigationStatus.FAILED,
            NavigationStatus.CANCELLED,
        }:
            navigation_backend.cancel_goal(request_id=goal.goal_id)

    @staticmethod
    def _clone_contract(value):
        if value is None:
            return None
        copier = getattr(value, "model_copy", None)
        if callable(copier):
            return copier(deep=True)
        return value

    @classmethod
    def _clone_metadata(cls, value: Any):
        if hasattr(value, "model_dump"):
            return value.model_dump(mode="json")
        if isinstance(value, dict):
            return {key: cls._clone_metadata(inner) for key, inner in value.items()}
        if isinstance(value, list):
            return [cls._clone_metadata(item) for item in value]
        return value


@dataclass
class PlatformGeometryProviderBundle(
    LocalizationProvider,
    MapProvider,
    NavigationProvider,
    ExplorationProvider,
):
    """平台几何 provider bundle。

    所有 provider 接口的平台级代理。数据源来自平台几何运行时管理的后端
    （外部几何侧车和平台数据面），但定位状态由平台状态机管理，
    不直接暴露后端原始状态给上层。
    """

    runtime: PlatformGeometryRuntime
    provider_name: str = "platform_geometry_runtime"
    provider_version: str = "0.3.0"

    def is_available(self) -> bool:
        return self.runtime.is_available()

    def _is_geometry_profile_enabled(self) -> bool:
        return self.runtime._is_geometry_profile_enabled()

    def is_localization_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return False
        if isinstance(backend, LocalizationDataPlane):
            return bool(backend.is_localization_available())
        return bool(backend.is_available())

    def is_localization_active(self) -> bool:
        """对外暴露"是否处于定位模式"判定，供感知/记忆门控使用。

        双源口径（与 runtime.py:1394~1395 / 2713~2714 的本地判定完全一致）：
        1. 外部几何进程在跑且为定位模式
        2. 最近一次 activate_* 报告的 backend_mode == "mapping_localization"
        任一为 True 即视为定位模式。
        """

        if not self._is_geometry_profile_enabled():
            return False
        try:
            if bool(self.runtime._external_geometry_process_is_localization()):
                return True
        except Exception:
            pass
        activation = getattr(self.runtime, "_last_activation_result", None)
        if isinstance(activation, dict):
            return str(activation.get("backend_mode") or "") == "mapping_localization"
        return False

    def is_map_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return True
        backend = self.runtime._resolve_mapping_backend()
        if backend is None:
            return False
        availability_checker = getattr(backend, "is_map_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return bool(backend.is_available())

    def is_navigation_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        if isinstance(backend, NavigationDataPlane):
            return bool(backend.is_navigation_available())
        return bool(backend.is_available())

    def is_exploration_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_exploration_backend()
        if backend is None:
            return False
        if isinstance(backend, ExplorationDataPlane):
            return bool(backend.is_exploration_available())
        return bool(backend.is_available())

    def get_current_pose(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_current_pose()

    def get_exploration_pose(self):
        """探索/建图阶段的实时位姿：优先导航后端 map→base 实时 TF，定位会话后端仅兜底。

        定位后端在重定位就绪前 get_current_pose() 恒返回 None，而探索建图时机器人
        持续移动/旋转，需要实时位姿驱动关键帧判定，故复用 runtime._read_exploration_pose()
        的优先级（导航后端实时 TF > 定位后端兜底）。
        """

        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime._read_exploration_pose()

    def get_frame_tree(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_frame_tree()

    def subscribe_localization(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_localization_updates(callback)

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        self.runtime.unsubscribe_localization_updates(subscription_id)

    def get_localization_state(self) -> dict:
        """返回后端原始定位状态，供平台定位状态机消费。"""

        if not self._is_geometry_profile_enabled():
            return {
                "tracking_state": "unavailable",
                "map_loaded": False,
                "quality_score": 0.0,
                "last_error": "平台几何运行时或几何 profile 已禁用。",
            }
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return {
                "tracking_state": "unavailable",
                "map_loaded": False,
                "quality_score": 0.0,
                "last_error": "平台几何运行时未绑定定位后端。",
            }
        get_state = getattr(backend, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
                if isinstance(state, dict):
                    return {
                        **state,
                        "backend_name": getattr(backend, "provider_name", None),
                        "raw_odom_input": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("odometry_publisher")),
                        "tf_localization_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("tf_localization_provider")),
                        "map_loaded": bool(state.get("map_loaded") or self.runtime._has_live_or_seed_map()),
                    }
            except Exception:
                pass
        # 回退到基于位姿可用性的简单判断
        pose = backend.get_current_pose()
        return {
            "tracking_state": "tracking" if pose is not None else "lost",
            "backend_name": getattr(backend, "provider_name", None),
            "map_loaded": self.runtime._has_live_or_seed_map(),
            "quality_score": 1.0 if pose is not None else 0.0,
            "last_error": None,
        }

    def get_status(self) -> dict:
        if not self._is_geometry_profile_enabled():
            return {"provider_status": "disabled"}
        navigation_backend_policy = self.runtime._navigation_backend_policy()
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "localization_backend": getattr(self.runtime._resolve_localization_backend(), "provider_name", None),
            "navigation_backend_policy": self.runtime._clone_metadata(navigation_backend_policy),
            "active_navigation_backend": navigation_backend_policy.get("active_backend_provider"),
            "tf_localization_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("tf_localization_provider")),
            "live_map_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("live_map_provider")),
        }

    def get_occupancy_grid(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.occupancy_grid
        # runtime.get_map_snapshot 已通过远端标准 getter 执行可靠 pull；None 表示
        # Rust 当前确实没有 occupancy revision，不能用空层伪造可用地图。
        return None

    def get_cost_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.cost_map
        # Rust facade 暂无独立 cost-map RPC；只返回 snapshot 中实际存在的层。
        return None

    def get_semantic_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.semantic_map
        # semantic layer 仅来自已激活平台版本，不合成远端空层。
        return None

    def set_goal(self, goal):
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        target_pose = getattr(goal, "target_pose", None)
        if target_pose is not None:
            LOGGER.debug(
                "几何 ProviderBundle 转发导航目标 backend=%s goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                getattr(goal, "goal_id", None),
                float(target_pose.position.x),
                float(target_pose.position.y),
                float(target_pose.position.z),
                target_pose.frame_id,
            )
        return self._call_navigation_backend(backend, "set_goal", goal)

    def cancel_goal(self, request_id: Optional[str] = None) -> bool:
        exact_goal_id = str(request_id or "").strip()
        if not exact_goal_id:
            LOGGER.warning("拒绝缺少 exact goal_id 的导航取消请求。")
            return False
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        state_getter = getattr(backend, "get_navigation_state", None)
        if not callable(state_getter):
            return False
        try:
            state = state_getter()
        except Exception:
            LOGGER.exception("导航取消前读取 backend exact goal 失败。")
            return False
        active_goal_id = str(
            getattr(state, "current_goal_id", "") or ""
        ).strip()
        raw_status = getattr(state, "status", None)
        active_status = str(
            getattr(raw_status, "value", raw_status) or ""
        ).strip().lower()
        observed_goal_id = str(
            dict(getattr(state, "metadata", {}) or {}).get("observed_goal_id") or ""
        ).strip()
        if (
            active_goal_id != exact_goal_id
            or (observed_goal_id and observed_goal_id != exact_goal_id)
            or active_status
            not in {
                NavigationStatus.ACCEPTED.value,
                NavigationStatus.PLANNING.value,
                NavigationStatus.RUNNING.value,
            }
        ):
            LOGGER.warning(
                "拒绝导航定向取消：requested_goal_id=%s active_goal_id=%s observed_goal_id=%s status=%s",
                exact_goal_id,
                active_goal_id or None,
                observed_goal_id or None,
                active_status or None,
            )
            return False
        return self._call_navigation_backend(
            backend,
            "cancel_goal",
            request_id=exact_goal_id,
        )

    def cancel_navigation(self, *, request_id: Optional[str] = None) -> bool:
        """扩展的导航取消入口（5.5 task）。

        `request_id` 必须是当前 ACTIVE 的 exact goal 标识；缺失或与后端状态不匹配时
        fail-closed，不发出 CancelNavigation RPC。

        返回：是否成功发出取消请求。
        """

        if not self._is_geometry_profile_enabled():
            return False
        return self.cancel_goal(request_id=str(request_id or "").strip() or None)

    def navigate(self, goal, timeout_sec: Optional[int] = None) -> str:
        """V2 非阻塞提交；成功返回 request/goal id，拒绝返回空串。"""

        del timeout_sec
        return str(getattr(goal, "goal_id", "") or "") if self.set_goal(goal) else ""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        """V2 定向取消。"""

        return self.cancel_goal(request_id=request_id)

    def get_state(self, request_id: Optional[str] = None):
        """V2 状态读取；当前 Rust facade 仅维护单 active goal。"""

        state = self.get_navigation_state()
        if request_id and state.current_goal_id not in {None, request_id}:
            metadata = dict(state.metadata)
            metadata.update({
                "requested_goal_id": request_id,
                "goal_id_mismatch": True,
            })
            return state.model_copy(update={"metadata": metadata}, deep=True)
        return state

    def subscribe_navigation_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_navigation_updates(callback)

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.unsubscribe_navigation_updates(subscription_id)

    def get_navigation_state(self):
        """扩展的导航状态读取入口（5.5 task）。

        - 优先走 backend.get_navigation_state()；
        - 在 metadata 中标记实际的 geometry runtime gRPC transport。
        """

        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            from contracts.navigation import NavigationState, NavigationStatus

            policy = self.runtime._navigation_backend_policy()
            message = "平台几何运行时未绑定 GeometryRuntimeProvider。"
            return NavigationState(
                status=NavigationStatus.IDLE,
                message=message,
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "navigation_transport": "geometry_runtime_grpc",
                },
            )
        getter = getattr(backend, "get_navigation_state", None)
        if not callable(getter):
            from contracts.navigation import NavigationState, NavigationStatus

            policy = self.runtime._navigation_backend_policy()
            return NavigationState(
                status=NavigationStatus.IDLE,
                message="当前导航后端 %s 未实现 get_navigation_state 接口。" % (
                    getattr(backend, "provider_name", None) or backend.__class__.__name__
                ),
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "navigation_transport": "geometry_runtime_grpc",
                    "unimplemented": True,
                },
            )
        state = getter()
        # 复用现有 metadata，补充实际 transport 标记；不破坏 schema。
        metadata = dict(getattr(state, "metadata", {}) or {})
        metadata.setdefault("navigation_transport", "geometry_runtime_grpc")
        return state.model_copy(update={"metadata": metadata}, deep=True)

    def is_goal_reached(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        return self._call_navigation_backend(backend, "is_goal_reached")

    def _call_navigation_backend(self, backend, method_name: str, *args, **kwargs):
        """统一调用 backend 的协议方法。

        当前正式 backend 是 GeometryRuntimeProvider（gRPC facade），
        其方法集（set_navigation_goal/cancel_navigation/get_navigation_readiness）与
        NavigationProvider 协议方法（set_goal/cancel_goal/get_navigation_state/is_goal_reached）
        名字不一致。Python 侧这里做"方法存在即转发，不存在则返回失败占位"，
        避免调用方收到 AttributeError 或被默默假装成功。
        """

        method = getattr(backend, method_name, None)
        if not callable(method):
            LOGGER.warning(
                "导航 backend=%s 未实现 %s；返回失败占位（不假装成功）。",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                method_name,
            )
            if method_name in {"set_goal", "cancel_goal", "is_goal_reached"}:
                return False
            if method_name in {"subscribe_navigation_updates"}:
                return None
            return None
        try:
            return method(*args, **kwargs)
        except Exception:
            LOGGER.exception(
                "导航 backend=%s.%s 调用异常。",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                method_name,
            )
            if method_name in {"set_goal", "cancel_goal", "is_goal_reached"}:
                return False
            if method_name in {"subscribe_navigation_updates"}:
                return None
            return None

    # ------------------------------------------------------------------
    # 探索入口：唯一生产链路为 GeometryRuntimeProvider（gRPC facade）
    # ------------------------------------------------------------------
    def _resolve_remote_exploration_provider(self):
        """解析连通的 gRPC facade provider；未连通返回 None。

        `PlatformGeometryRuntime` 在 `_start_geometry_runtime_subprocess` 时挂 gRPC provider
        于 `_remote_provider`；该对象是探索控制与状态的唯一生产 provider。
        """
        remote = getattr(self.runtime, "_remote_provider", None)
        if remote is None:
            return None
        try:
            if not bool(remote.is_connected):
                return None
        except Exception:
            return None
        return remote

    def _start_exploration_via_remote(self, request, remote) -> bool:
        """把 request 翻译成 gRPC facade 的 start_exploration 调用。

        request 是 platform 侧的 `ExploreAreaRequest` 契约；gRPC facade 接收完整
        map pin、边界、停止条件、重试和 blacklist 参数。translation 或 RPC
        失败时返回 False，不切换到第二套执行器。
        """
        request_id = ""
        strategy = "wavefront"
        arbitration = "prefer_wavefront"
        map_version_id = ""
        map_revision = 0
        boundary_line_segments = []
        stop_conditions = {}
        max_navigation_retries = 0
        blacklist_radius_m = 0.0
        if request is not None:
            request_id = str(getattr(request, "request_id", "") or "")
            if not request_id:
                request_id = "platform_explore:%d" % int(time.time() * 1000)
            strategy = str(getattr(request, "strategy", "") or "wavefront").strip()
            stop_conditions = dict(getattr(request, "stop_conditions", {}) or {})
            request_metadata = dict(getattr(request, "metadata", {}) or {})
            arbitration = str(
                request_metadata.get("arbitration")
                or stop_conditions.get("arbitration")
                or "prefer_wavefront"
            ).strip()
            map_version_id = str(
                request_metadata.get("map_version_id")
                or stop_conditions.get("map_version_id")
                or ""
            ).strip()
            map_revision = int(
                getattr(request, "map_revision", None)
                or request_metadata.get("map_revision")
                or 0
            )
            boundary_line_segments = list(
                getattr(request, "boundary_line_segments", ()) or ()
            )
            max_duration_sec = getattr(request, "max_duration_sec", None)
            if (
                max_duration_sec is not None
                and "max_duration_ms" not in stop_conditions
            ):
                stop_conditions["max_duration_ms"] = max(
                    0, int(round(float(max_duration_sec) * 1000.0))
                )
            max_navigation_retries = int(
                getattr(request, "max_navigation_retries", None)
                or request_metadata.get("max_navigation_retries")
                or 0
            )
            blacklist_radius_m = float(
                getattr(request, "blacklist_radius_m", None)
                or request_metadata.get("blacklist_radius_m")
                or 0.0
            )
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        with lock:
            remote_token = _capture_remote_operation_token_locked(
                self.runtime,
                remote,
            )
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if remote_token is None or (
                current_id and current_id != request_id
            ):
                return False
        try:
            mapping_state = self.runtime.start_mapping(
                map_name=(
                    str(getattr(request, "map_name", "") or "").strip() or None
                    if request is not None
                    else None
                ),
                map_version_id=map_version_id or None,
            )
            map_version_id = str(
                mapping_state.get("mapping_version_id") or ""
            ).strip()
            if not map_version_id:
                raise RuntimeError("探索启动前未获得 ACTIVE mapping version。")
            if map_revision <= 0:
                map_revision = int(mapping_state.get("revision") or 0)
            result = remote.start_exploration(
                request_id=request_id,
                strategy=strategy,
                arbitration=arbitration,
                map_version_id=map_version_id,
                map_revision=map_revision,
                boundary_line_segments=boundary_line_segments,
                stop_conditions=stop_conditions,
                max_navigation_retries=max_navigation_retries,
                blacklist_radius_m=blacklist_radius_m,
            )
        except Exception:
            LOGGER.exception("Bundle.start_exploration gRPC 失败。")
            return False
        if not isinstance(result, dict):
            return False
        state = self._wrap_remote_exploration_state(result)
        raw_status = str(result.get("status") or "").strip().lower()
        accepted = raw_status not in {
            "",
            "idle",
            "rejected",
            "failed",
            "failure",
            "unimplemented",
        }
        notify_state = False
        with lock:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            terminal = _is_complete_exploration_terminal(
                state,
                request_id=request_id,
            )
            if accepted or terminal:
                if not _commit_remote_exploration_state_locked(
                    self.runtime,
                    state,
                    request_id=request_id,
                    request=request,
                    map_version_id=map_version_id,
                ):
                    return False
            notify_state = True
        if notify_state:
            notifier = getattr(
                self.runtime,
                "_notify_exploration_update",
                None,
            )
            if callable(notifier):
                notifier(state)
        return accepted

    def start_exploration(self, request) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        remote = self._resolve_remote_exploration_provider()
        if remote is None:
            LOGGER.error("Bundle.start_exploration: GeometryRuntimeProvider 未连接。")
            return False
        return self._start_exploration_via_remote(request, remote)

    def stop_exploration(self, request_id: Optional[str] = None) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        requested_id = str(request_id or "").strip()
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        committed = False
        with lock:
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            correlated_id = requested_id or expected_id
            existing_terminal = getattr(
                self.runtime,
                "_external_exploration_terminal_state",
                None,
            )
            if not correlated_id:
                return False
            if not expected_id:
                if _is_complete_exploration_terminal(
                    existing_terminal,
                    request_id=correlated_id,
                    safe_stop_only=True,
                ):
                    return True
                return False
            if requested_id and requested_id != expected_id:
                return False
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(
                    self.runtime,
                    remote,
                )
                if remote is not None
                else None
            )
        if remote_token is None:
            LOGGER.error("native remote 不可用，拒绝 correlated StopExploration。")
            return False
        try:
            result = remote.stop_exploration(request_id=correlated_id)
        except Exception:
            LOGGER.exception(
                "Bundle.stop_exploration gRPC 异常；correlated stop 失败。"
            )
            return False
        if not isinstance(result, dict):
            return False
        state = self._wrap_remote_exploration_state(result)
        raw_status = str(result.get("status") or "").strip().lower()
        accepted = raw_status not in {
            "",
            "rejected",
            "failed",
            "failure",
            "unimplemented",
        }
        with lock:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            if not accepted or _exploration_state_request_id(state) != correlated_id:
                return False
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if current_id != expected_id:
                existing_terminal = getattr(
                    self.runtime,
                    "_external_exploration_terminal_state",
                    None,
                )
                return bool(
                    not current_id
                    and _is_complete_exploration_terminal(
                        existing_terminal,
                        request_id=correlated_id,
                        safe_stop_only=True,
                    )
                )
            committed = _commit_remote_exploration_state_locked(
                self.runtime,
                state,
                request_id=correlated_id,
            )
        if committed:
            self.runtime._notify_exploration_update(state)
        return committed

    def sync_exploration_after_task(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        committed = False
        terminal = False
        with lock:
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            existing_terminal = getattr(
                self.runtime,
                "_external_exploration_terminal_state",
                None,
            )
            if not expected_id and existing_terminal is not None:
                return _is_complete_exploration_terminal(
                    existing_terminal,
                    request_id=_exploration_state_request_id(existing_terminal),
                )
            if not expected_id:
                return False
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(
                    self.runtime,
                    remote,
                )
                if remote is not None
                else None
            )
        if remote_token is None:
            return False
        try:
            remote_state = remote.get_exploration_state()
        except Exception:
            LOGGER.exception(
                "Bundle.sync_exploration_after_task gRPC 异常。"
            )
            return False
        if not isinstance(remote_state, dict):
            return False
        state = self._wrap_remote_exploration_state(remote_state)
        with lock:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if current_id != expected_id:
                return False
            committed = _commit_remote_exploration_state_locked(
                self.runtime,
                state,
                request_id=expected_id,
            )
            if not committed:
                return False
            terminal = state.status in {
                ExplorationStatus.SUCCEEDED,
                ExplorationStatus.FAILED,
                ExplorationStatus.CANCELLED,
            }
        self.runtime._notify_exploration_update(state)
        return terminal

    def subscribe_exploration_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_exploration_updates(callback)

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.unsubscribe_exploration_updates(subscription_id)

    def get_exploration_state(self):
        """通过 runtime 的统一公开入口读取 native exploration 快照。"""

        return self.runtime.get_platform_exploration_state()

    @staticmethod
    def _wrap_remote_exploration_state(state: Dict[str, object]):
        """把 gRPC dict ExplorationState 装成 `contracts.navigation.ExplorationState`。

        Phase 5 收口：契约字段对齐 dimos `nuwax_exploration::ExplorationState`
        (status / message / request_id / selected_source / last_stop_reason /
        frontier_count / revision / timestamp_ns)。
        """
        from contracts.navigation import ExplorationState, ExplorationStatus

        raw_status = str(state.get("status") or "").strip().lower()
        stop_reason = str(state.get("last_stop_reason") or "").strip().lower()
        status_enum = {
            "idle": ExplorationStatus.IDLE,
            "accepted": ExplorationStatus.ACCEPTED,
            "queued": ExplorationStatus.ACCEPTED,
            "pending": ExplorationStatus.ACCEPTED,
            "active": ExplorationStatus.RUNNING,
            "running": ExplorationStatus.RUNNING,
            "in_progress": ExplorationStatus.RUNNING,
            "stopping": ExplorationStatus.RUNNING,
            "cancel_pending": ExplorationStatus.RUNNING,
            "succeeded": ExplorationStatus.SUCCEEDED,
            "success": ExplorationStatus.SUCCEEDED,
            "completed": ExplorationStatus.SUCCEEDED,
            "failed": ExplorationStatus.FAILED,
            "failure": ExplorationStatus.FAILED,
            "cancelled": ExplorationStatus.CANCELLED,
            "canceled": ExplorationStatus.CANCELLED,
            "rejected": ExplorationStatus.FAILED,
            "unimplemented": ExplorationStatus.IDLE,
        }.get(raw_status, ExplorationStatus.IDLE)
        if raw_status == "stopped":
            if stop_reason in {"no_information_gain", "no_reachable_free_space", "hit_stop"}:
                status_enum = ExplorationStatus.SUCCEEDED
            elif stop_reason == "consecutive_failures_exceeded":
                status_enum = ExplorationStatus.FAILED
            elif stop_reason == "user_cancelled":
                status_enum = ExplorationStatus.CANCELLED
            else:
                status_enum = ExplorationStatus.FAILED
        try:
            frontier_count = state.get("frontier_count")
            frontier_count = int(frontier_count) if frontier_count is not None else 0
        except (TypeError, ValueError):
            frontier_count = 0
        try:
            revision = state.get("revision")
            revision = int(revision) if revision is not None else 0
        except (TypeError, ValueError):
            revision = 0
        try:
            timestamp_ns = state.get("timestamp_ns")
            timestamp_ns = int(timestamp_ns) if timestamp_ns is not None else 0
        except (TypeError, ValueError):
            timestamp_ns = 0
        try:
            map_revision = state.get("map_revision")
            map_revision = int(map_revision) if map_revision is not None else 0
        except (TypeError, ValueError):
            map_revision = 0
        message = str(state.get("message") or "") or None
        terminal = status_enum in {
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        }
        metadata = {
            "provider": "geometry_runtime",
            "gRPC_facade": True,
            "state_source": "geometry_runtime_grpc",
            "runtime_ready": True,
            "exploration_mode": "nuwax_geometry_runtime",
            "geometry_session_mode": "explore" if raw_status != "idle" else "idle",
            "status_text": message or raw_status.upper(),
            "exploration_terminal": terminal,
            "terminal_status": status_enum.value if terminal else None,
            "selected_source": state.get("selected_source") or None,
            "last_stop_reason": stop_reason or None,
            "request_id": state.get("request_id") or None,
            "child_task_id": state.get("child_task_id") or None,
            "map_version_id": state.get("map_version_id") or None,
            "map_revision": map_revision,
            "revision": revision,
            "timestamp_ns": timestamp_ns,
        }
        timestamp = (
            datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, tz=timezone.utc)
            if timestamp_ns > 0
            else datetime.now(timezone.utc)
        )
        return ExplorationState(
            timestamp=timestamp,
            updated_at=timestamp,
            current_request_id=str(state.get("request_id") or "") or None,
            status=status_enum,
            strategy=str(state.get("strategy") or "") or None,
            message=message,
            frontier_count=frontier_count,
            metadata=metadata,
        )
