from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
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
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple, TYPE_CHECKING, runtime_checkable
from urllib.parse import quote

from contracts.geometry import Pose, Vector3
from core.logging_utils import StreamBusLogHandler
from contracts.map_workspace import MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType, MapVersion
from contracts.navigation import (
    ExplorationState,
    ExplorationStatus,
    ExploreAreaRequest,
    NavigationGoal,
    NavigationState,
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
from services.geometry.cmd_vel_bridge import Ros2CmdVelMotionBridge
from services.geometry.config import PlatformGeometryConfig
from services.geometry.external_debug_observer import ExternalGeometryDebugObserver
from services.geometry.nav2_action_provider import Nav2ActionNavigationProvider
from services.geometry.ros_bridge_client import RosBridgeSubprocess
from services.geometry.remote_provider import RosBridgeProvider
from services.geometry.rosout_bridge import RosoutBridge
from contracts.naming import build_safe_dir_segment
from services.localization.frame_tree_ops import resolve_frame_transform
from services.navigation.config import ExplorationConfig, NavigationBackendConfig
from services.navigation.exploration_runtime import PlatformExplorationRuntime
from services.navigation.frontier import FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner

if TYPE_CHECKING:
    from core.event_bus import EventBus

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime")

_LIVE_MAPPING_DIAGNOSTICS_TTL_SEC = 0.5
_EXTERNAL_EXPLORATION_LOG_SCAN_MIN_INTERVAL_SEC = 2.0


_GEOMETRY_SIDE_CAR_RESTART_LIMIT = 2
_GEOMETRY_MONITOR_INTERVAL_SEC = 1.0
_GEOMETRY_STREAMING_WINDOW_SEC = 3.0
_EXTERNAL_GEOMETRY_STACKS = {"external_slam_nav2"}
_EXTERNAL_SLAM_NAV2_STATE_ASSET_NAME = "external_slam_nav2_state.db"
_MAIN_LOG_APPEND_LOCK = threading.Lock()
_GO2_SH_MAIN_LOG_LOGGER = "nuwax_robot_bridge.geometry.go2.sh"


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


def _publish_main_log_stream(logger_name: str, message: str) -> None:
    """将 go2.sh 子进程行推送到 StreamBus logs.entry（不经 file handler，避免重复写盘）。"""

    record = logging.LogRecord(
        name=str(logger_name or _GO2_SH_MAIN_LOG_LOGGER),
        level=logging.INFO,
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
    path = _project_log_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    line = _format_main_log_line(logger_name, text)
    with _MAIN_LOG_APPEND_LOCK:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(line)
    _publish_main_log_stream(logger_name, text)


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
        if self.name == "external_slam_nav2":
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
    1. 约束平台地图版本对应的外部 SLAM/Nav2 资产目录；
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
        # 以下4个 Python provider 已由 Rust bridge (RosBridgeProvider) 替代，保留 None 占位以兼容现有引用
        self._live_map_provider = None
        self._tf_localization_provider = None
        self._odometry_publisher = None
        self._point_cloud_publisher = None
        self._live_geometry_start_lock = threading.RLock()
        self._live_geometry_start_attempted = False
        self._pending_localization_callbacks = []
        self._pending_map_callbacks = []
        self._runtime_state_lock = threading.RLock()
        self._runtime_phase = "disabled" if not self._is_geometry_profile_enabled() else "preflight"
        self._runtime_message = "平台几何运行时尚未启动。"
        self._runtime_blocked = False
        self._runtime_started = False
        self._runtime_stop_event = threading.Event()
        self._runtime_monitor_thread: Optional[threading.Thread] = None
        self._managed_processes: Dict[str, _ManagedProcess] = {}
        self._runtime_diagnostics: List[Dict[str, object]] = []
        self._map_stream_tracker: Dict[str, object] = {}
        # 外部 /map 观察器（ExternalGeometryDebugObserver）才是 /map 占据栅格的真实来源；
        # Rust bridge 的 get_status 只返回 {phase,message,rss_kb}，不含占据状态。
        # 此处缓存观察器最近一帧的就绪元数据，供 _ros_bridge_status_stub("live_map_provider")
        # 合并，从而让 _build_rtabmap_readiness / _classify_live_mapping_wait /
        # _update_stream_tracker 读到真实的 occupancy 状态。
        self._observer_occupancy_snapshot: Dict[str, object] = {}
        self._observer_occupancy_lock = threading.Lock()
        self._external_live_pose: Optional[Pose] = None
        self._external_live_pose_at_monotonic = 0.0
        self._external_live_pose_lock = threading.Lock()
        self._debug_initial_pose_hint: Optional[Pose] = None
        self._debug_initial_pose_anchor: Optional[Pose] = None
        self._debug_initial_pose_hint_until = 0.0
        self._debug_initial_pose_hint_stable_count = 0
        self._nav2_launch_attempted = False
        self._nav2_action_provider = Nav2ActionNavigationProvider(self)
        self._cmd_vel_bridge: Optional[Ros2CmdVelMotionBridge] = None
        self._ros_bridge_subprocess: Optional[RosBridgeSubprocess] = None
        self._remote_provider: Optional[RosBridgeProvider] = None
        self._rosout_bridge = RosoutBridge()
        self._external_debug_observer: Optional[ExternalGeometryDebugObserver] = None
        self._external_debug_frontier_payload: Optional[Dict[str, object]] = None
        self._platform_exploration_config = ExplorationConfig(enabled=True)
        self._platform_exploration_runtime = PlatformExplorationRuntime(
            frontier_explorer=FrontierExplorer(self._platform_exploration_config),
            navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
            config=self._platform_exploration_config,
        )
        self._exploration_loop_lock = threading.RLock()
        self._exploration_loop_stop_event = threading.Event()
        self._exploration_loop_thread: Optional[threading.Thread] = None
        self._exploration_loop_generation = 0
        self._exploration_loop_thread_generation = 0
        self._exploration_tick_count = 0
        self._exploration_last_tick_at_iso: Optional[str] = None
        self._exploration_last_tick_monotonic: Optional[float] = None
        self._exploration_last_tick_source: Optional[str] = None
        self._exploration_update_subscribers: Dict[int, Callable[[ExplorationState], None]] = {}
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

    def start(self) -> None:
        """启动平台几何运行时。"""

        with self._runtime_state_lock:
            if self._runtime_started:
                return
            self._runtime_started = True
            self._runtime_stop_event.clear()
            self._runtime_diagnostics = []
            self._managed_processes = {}
            self._nav2_launch_attempted = False
            self._map_stream_tracker = {}
            with self._observer_occupancy_lock:
                self._observer_occupancy_snapshot = {}
        if not self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时或几何 profile 已禁用。")
            return
        try:
            self._set_runtime_state("preflight", "正在检查外部 SLAM/Nav2 脚本、话题与 TF 配置。")
            preflight_errors = self._run_preflight_checks()
            if preflight_errors:
                message = "平台几何运行时预检失败：" + "；".join(preflight_errors)
                self._set_runtime_state("failed", message, blocked=True)
                if self.config.require_ros2:
                    raise RuntimeError(message)
                return
            if not self._uses_external_geometry_stack():
                raise RuntimeError(self._removed_internal_stack_message())
            self._set_runtime_state("starting_external", "正在启动外部 SLAM/Nav2 几何脚本。")
            self._start_ros_bridge_subprocess()
            self._ensure_external_geometry_observers_started()
            self._start_required_processes()
            self._set_runtime_state("starting_external", "外部 SLAM/Nav2 几何脚本已启动，等待 /map_obstacles、TF 与 Nav2 action。")
            self._rosout_bridge.start()
            self._start_monitor_thread()
        except Exception as exc:
            self._record_runtime_diagnostic("geometry_start", exc)
            self._set_runtime_state("failed", "平台几何运行时启动失败：%s" % exc, blocked=True)
            self._stop_managed_processes()
            self._live_geometry_start_attempted = False
            with self._runtime_state_lock:
                self._runtime_started = False
            if self.config.require_ros2:
                raise

    def _get_motion_provider(self) -> Optional[MotionControl]:
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, MotionControl):
            return providers
        return None

    def stop(self) -> None:
        """停止平台几何运行时。"""

        self._set_runtime_state("stopping", "正在停止平台几何侧车进程。")
        self._runtime_stop_event.set()
        if self._cmd_vel_bridge is not None:
            try:
                self._cmd_vel_bridge.stop()
            except Exception:
                pass
            self._cmd_vel_bridge = None
        if self._external_debug_observer is not None:
            try:
                self._external_debug_observer.stop()
            except Exception:
                pass
            self._external_debug_observer = None
        self._stop_platform_exploration_loop(cancel_navigation=True, notify=True)
        thread = self._runtime_monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._stop_managed_processes()
        self._rosout_bridge.stop()
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
        return "external_slam_nav2"

    def _uses_external_geometry_stack(self) -> bool:
        return True

    def _removed_internal_stack_message(self) -> str:
        return "当前运行时固定使用 external_slam_nav2（RTAB-Map + Nav2 action）单链路。"

    def _external_process_name(self) -> str:
        name = str(getattr(self.config.external, "process_name", "") or "").strip()
        return name or "external_slam_nav2"

    def _external_log_name(self) -> str:
        log_name = str(getattr(self.config.external, "log_name", "") or "").strip()
        return log_name or "external_slam_nav2.log"

    def _managed_process_identity(self, name: str) -> Tuple[str, str]:
        if name == self._external_process_name():
            return "nuwax-geometry-external", "统一几何服务"
        if name == "external_odom_tf_bridge":
            return "nuwax-odom-tf-bridge", "里程计 TF 桥"
        safe = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "-" for ch in str(name or "geometry")).strip("-_")
        return f"nuwax-{safe or 'geometry'}", "项目几何进程"

    def _external_script_path(self) -> Path:
        return Path(str(getattr(self.config.external, "script_path", "") or "")).expanduser()

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
        return [str(self._external_script_path()), "--mapping", *passthrough_args, str(db_path)]

    def _external_exploration_database_path_for_request(self, request: Optional[ExploreAreaRequest]) -> Path:
        map_name = str(getattr(request, "map_name", "") or "").strip() if request is not None else ""
        if map_name:
            version_id = self._next_exploration_map_version_id(map_name)
            path = self._version_rtabmap_database_path(map_name, version_id)
            path.parent.mkdir(parents=True, exist_ok=True)
            with self._exploration_loop_lock:
                self._external_exploration_version_id = version_id
            return path
        return self._default_external_exploration_database_path()

    def plan_external_exploration(self, request: ExploreAreaRequest) -> Dict[str, object]:
        """返回即将执行的 launch_rtabmap.sh --explore 命令与版本目录下的 .db 路径。"""

        command = self._build_external_exploration_command(request)
        db_path = self._extract_external_exploration_database_path(command)
        version_id = self.get_exploration_pinned_version_id()
        return {
            "exploration_launch_mode": "explore",
            "exploration_script": str(self._external_script_path()),
            "exploration_command": command,
            "exploration_database_path": db_path,
            "exploration_version_id": version_id,
        }

    def get_exploration_pinned_version_id(self) -> Optional[str]:
        """返回当前探索会话绑定的 mapver_*，保存版本时应写入同一路径。"""

        with self._exploration_loop_lock:
            return str(self._external_exploration_version_id or "").strip() or None

    def clear_exploration_pinned_version_context(self) -> None:
        with self._exploration_loop_lock:
            self._external_exploration_version_id = None
            self._external_exploration_database_path = None

    def clear_external_exploration_state(self, *, reason: str = "") -> None:
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
            self._exploration_tick_count = 0
            self._exploration_last_tick_at_iso = None
            self._exploration_last_tick_monotonic = None
            self._exploration_last_tick_source = str(reason or "").strip() or "clear_external_exploration_state"

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
        raw_path = (os.environ.get("NUWAX_RTABMAP_DATABASE_PATH") or "").strip()
        if raw_path:
            path = Path(raw_path).expanduser()
        else:
            root = os.environ.get("NUWAX_RTABMAP_DATABASE_ROOT") or self.config.rtabmap.database_root
            path = Path(str(root)).expanduser() / "live_rtabmap.db"
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
        # GO2_RTABNAV2 uses localize_only/restart_map/slam_mode; legacy GO2_NAV_* keys are deprecated.
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
        if self._uses_external_geometry_stack():
            return self._run_external_geometry_preflight_checks()
        return [self._removed_internal_stack_message()]

    def _run_external_geometry_preflight_checks(self) -> List[str]:
        errors: List[str] = []
        if not bool(getattr(self.config.external, "enabled", True)):
            errors.append("external_slam_nav2 单链路需要启用外部 SLAM/Nav2 脚本。")
        raw_script_path = str(getattr(self.config.external, "script_path", "") or "").strip()
        script_path = self._external_script_path()
        if not raw_script_path:
            errors.append("缺少外部 SLAM/Nav2 script 配置 NUWAX_GEOMETRY_EXTERNAL_SCRIPT")
        elif not script_path.exists():
            errors.append("外部 SLAM/Nav2 script 不存在 %s" % script_path)
        elif not script_path.is_file():
            errors.append("外部 SLAM/Nav2 script 不是文件 %s" % script_path)
        elif not os.access(script_path, os.X_OK):
            errors.append("外部 SLAM/Nav2 script 不可执行 %s" % script_path)
        if self._external_input_bridge_enabled() or self._platform_odom_tf_bridge_enabled():
            launch_script = self._geometry_script("launch_ros2_stack.sh")
            if not launch_script.exists():
                errors.append("缺少 ROS2 环境启动脚本 %s，无法启动可选 odom/TF 桥。" % launch_script)
            odom_bridge_script = self._geometry_script("odom_tf_bridge.py")
            if not odom_bridge_script.exists():
                errors.append("缺少 odom TF bridge 脚本 %s，无法启动可选 odom/TF 桥。" % odom_bridge_script)
            setup_script = Path(str(self.config.setup_script or "")).expanduser()
            ros_setup_available = setup_script.exists() or any(path.exists() for path in self._ros_setup_candidates())
            if self.config.require_ros2 and not setup_script.exists():
                errors.append("缺少 geometry setup script %s" % setup_script)
            if shutil.which("ros2") is None and not ros_setup_available:
                errors.append("未找到 ros2 命令、geometry setup script 或系统 ROS setup，无法启动可选 odom/TF 桥。")
        required_topics = {
            "occupancy_topic": self.config.topics.occupancy_topic,
            "cost_map_topic": self.config.topics.cost_map_topic,
            "tf_topic": self.config.topics.tf_topic,
            "tf_static_topic": self.config.topics.tf_static_topic,
        }
        if self._external_point_cloud_bridge_enabled():
            required_topics["point_cloud_topic"] = self.config.topics.point_cloud_topic
        if self._external_input_bridge_enabled():
            required_topics["odom_topic"] = self.config.topics.odom_topic
        for name, value in required_topics.items():
            if not str(value or "").strip():
                errors.append("缺少外部 SLAM/Nav2 topic 配置 %s" % name)
        required_frames = {
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
        }
        for name, value in required_frames.items():
            if not str(value or "").strip():
                errors.append("缺少外部 SLAM/Nav2 TF frame 配置 %s" % name)
        return errors

    def _ros_setup_candidates(self) -> List[Path]:
        return [
            Path("/opt/ros/humble/setup.bash"),
            Path("/opt/ros/jazzy/setup.bash"),
            Path("/opt/ros/iron/setup.bash"),
            Path("/opt/ros/foxy/setup.bash"),
        ]

    def _start_required_processes(self) -> None:
        if not self._uses_external_geometry_stack():
            message = self._removed_internal_stack_message()
            self._record_runtime_diagnostic("removed_internal_rtabmap_stack", message)
            raise RuntimeError(message)
        platform_odom_tf_bridge_enabled = self._platform_odom_tf_bridge_enabled()
        platform_cmd_vel_bridge_enabled = self._platform_cmd_vel_bridge_enabled()
        if platform_odom_tf_bridge_enabled:
            self._start_managed_process(
                "external_odom_tf_bridge",
                self._build_odom_tf_bridge_command(),
                "external_odom_tf_bridge.log",
            )
        external_command = self._build_external_geometry_command()
        # 如果外部脚本未配置 mode 标志（如 go2.sh 无 --explore/--localization），
        # 不自动启动外部几何进程，避免默认进入 manual mapping 模式。
        if not self._command_has_mode_flag(external_command):
            self._record_runtime_diagnostic(
                "external_slam_nav2_skipped",
                "外部 SLAM/Nav2 脚本未配置 mode 标志，平台启动时不自动启动 go2.sh（mapping 为手动建图专用）。",
            )
        else:
            self._start_managed_process(
                self._external_process_name(),
                external_command,
                self._external_log_name(),
            )
        if platform_cmd_vel_bridge_enabled:
            try:
                bridge = Ros2CmdVelMotionBridge(
                    config=self.config,
                    motion_provider_resolver=self._get_motion_provider,
                )
                bridge.start()
                self._cmd_vel_bridge = bridge
                LOGGER.info("ROS2 cmd_vel 运动桥已启动，订阅 %s", bridge.topic)
            except Exception as exc:
                LOGGER.warning("ROS2 cmd_vel 运动桥启动失败（机器人不会响应 Nav2 /cmd_vel）：%s", exc)
        if self._external_input_bridge_enabled():
            self._start_ros_bridge_subprocess()
            bridge_message = "外部 SLAM/Nav2 脚本托管启动；平台启动 ROS2 odom/点云输入桥，不启动内置 RTAB-Map/Nav2。"
        elif self._external_point_cloud_bridge_enabled():
            bridge_message = "外部 SLAM/Nav2 脚本托管启动；平台只启动 ROS2 点云输入桥，跳过 odom/odom_tf_bridge。"
        else:
            bridge_message = "外部 SLAM/Nav2 脚本托管启动；平台跳过 odom_tf_bridge/cmd_vel_bridge 与内置 RTAB-Map/Nav2。"
        if platform_odom_tf_bridge_enabled or platform_cmd_vel_bridge_enabled:
            bridge_message += " 平台兼容桥启用：odom_tf=%s, cmd_vel=%s。" % (
                platform_odom_tf_bridge_enabled,
                platform_cmd_vel_bridge_enabled,
            )
        self._record_runtime_diagnostic(
            "external_slam_nav2",
            bridge_message,
        )

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
        return ["bash", str(self._geometry_script("launch_ros2_stack.sh"))]

    def _build_odom_tf_bridge_command(self) -> List[str]:
        # 平台内部 odom 源由 odometry_publisher 写入固定 frame，使用「锁定值」语义；
        # 外部 SLAM/Nav2 切换命名时由配置驱动，不靠 bridge 静默兜底。
        return self._build_launch_prefix() + [
            "python3",
            str(self._geometry_script("odom_tf_bridge.py")),
            "--odom-topic",
            str(self.config.topics.odom_topic),
            "--odom-frame",
            str(self.config.frames.odom_frame),
            "--default-odom-frame",
            str(self.config.frames.odom_frame),
            "--default-child-frame",
            str(self.config.frames.base_frame),
            "--child-frame",
            str(self.config.frames.base_frame),
        ]

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
        if self._ros_bridge_subprocess is not None:
            self._ros_bridge_subprocess.stop()
            self._ros_bridge_subprocess = None
        if self._remote_provider is not None:
            self._remote_provider.disconnect()
            self._remote_provider = None

    def _start_ros_bridge_subprocess(self) -> None:
        """启动 Rust nuwax-ros-bridge 子进程并建立 gRPC 连接。"""
        if self._ros_bridge_subprocess is not None and self._ros_bridge_subprocess.is_running:
            return
        bridge = RosBridgeSubprocess()
        if not bridge.start():
            LOGGER.warning("Rust ros-bridge 子进程启动失败: %s", bridge.last_error)
            self._record_runtime_diagnostic("ros_bridge_start", bridge.last_error or "启动失败")
            return
        self._ros_bridge_subprocess = bridge
        provider = RosBridgeProvider(bridge)
        if provider.connect():
            self._remote_provider = provider
            LOGGER.info("Rust ros-bridge gRPC 连接已建立")
            self._inject_static_extrinsics(provider)
            for cb in self._pending_localization_callbacks:
                self._remote_provider.subscribe_pose(cb)
            for cb in self._pending_map_callbacks:
                self._remote_provider.subscribe_map_updates(cb)
        else:
            LOGGER.warning("Rust ros-bridge gRPC 连接失败，子进程仍在运行")

    def _inject_static_extrinsics(self, provider: "RosBridgeProvider") -> None:
        """装配期：从机器人 CalibrationProvider 取静态外参拓扑，缓存进 RosBridgeProvider。

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
                if self.config.require_ros2:
                    return

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
        if not self._runtime_started or not self._is_geometry_profile_enabled():
            return
        if not self._uses_external_geometry_stack():
            self._set_runtime_state("failed", self._removed_internal_stack_message(), blocked=True)
            return
        process_failure = self._blocking_process_failure()
        if process_failure:
            self._set_runtime_state("failed", process_failure, blocked=True)
            return
        live_map_status = self._clone_metadata(self._ros_bridge_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._ros_bridge_status_stub("tf_localization_provider"))
        odometry_status = self._clone_metadata(self._ros_bridge_status_stub("odometry_publisher"))
        point_cloud_status = self._clone_metadata(self._ros_bridge_status_stub("point_cloud_publisher"))
        point_cloud_source = self._build_point_cloud_source_status()
        data_plane_status = self._build_data_plane_status()
        self._update_stream_tracker(live_map_status)
        readiness = self._build_rtabmap_readiness(
            live_map_status=live_map_status,
            tf_localization_status=tf_localization_status,
            odometry_status=odometry_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )
        if not bool(readiness.get("go2_odom_ready")):
            self._set_runtime_state("waiting_input_bridge", str(readiness.get("message") or "等待外部 SLAM/Nav2 输入桥。"))
            return
        if not bool(readiness.get("first_frame_ready")):
            self._set_runtime_state("waiting_map", str(readiness.get("message") or "等待外部 SLAM/Nav2 /map_obstacles 首帧。"))
            return
        if bool(readiness.get("streaming_required")) and not bool(readiness.get("streaming_ready")):
            self._set_runtime_state("first_frame", str(readiness.get("message") or "外部 SLAM/Nav2 已发布 /map_obstacles 首帧，正在确认连续流式更新。"))
            return
        exploration_state = self.get_platform_exploration_state()
        if getattr(exploration_state.status, "value", exploration_state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }:
            explore_message = exploration_state.message or "GO2_RTABNAV2 原生探索任务运行中。"
            self._set_runtime_state("exploring", explore_message)
            return
        if not bool(readiness.get("localization_ready")):
            self._set_runtime_state("streaming", str(readiness.get("message") or "地图首帧已就绪，等待外部 SLAM/Nav2 定位与 TF。"))
            return
        nav2_status = self._build_nav2_status()
        nav2_phase = str(nav2_status.get("phase") or "")
        if nav2_phase == "starting":
            self._set_runtime_state("starting_nav2", "正在等待外部 Nav2 action 可用。")
            return
        if nav2_phase == "failed":
            self._set_runtime_state("failed", str(nav2_status.get("message") or "外部 Nav2 action 失败。"), blocked=True)
            return
        if nav2_phase != "ready":
            self._set_runtime_state("streaming", "外部 SLAM/Nav2 地图 streaming 与 TF 已就绪；Nav2 状态：%s。" % nav2_status.get("phase"))
            return
        self._set_runtime_state("ready", "外部 SLAM/Nav2 地图 streaming、TF 与 Nav2 action 均已就绪。")

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

    def _build_rtabmap_readiness(
        self,
        *,
        live_map_status: Optional[Dict[str, object]] = None,
        tf_localization_status: Optional[Dict[str, object]] = None,
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        live_map_status = live_map_status or self._clone_metadata(self._ros_bridge_status_stub("live_map_provider"))
        tf_localization_status = tf_localization_status or self._clone_metadata(self._ros_bridge_status_stub("tf_localization_provider"))
        odometry_status = odometry_status or self._clone_metadata(self._ros_bridge_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._ros_bridge_status_stub("point_cloud_publisher"))
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
        message = self._removed_internal_stack_message()
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        return {
            "ready": False,
            "go2_odom_ready": False,
            "input_bridge_ready": False,
            "first_frame_ready": False,
            "streaming_ready": False,
            "localization_ready": False,
            "relocalization_ready": False,
            "waiting_for": "external_slam_nav2_stack",
            "message": message,
            "removed_internal_stack": True,
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "legacy_rtabmap": {
                "info_topic": "/rtabmap/info",
                "info_not_required": True,
                "process": None,
                "removed": True,
            },
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
        odometry_status = odometry_status or self._clone_metadata(self._ros_bridge_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._ros_bridge_status_stub("point_cloud_publisher"))
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
            or str((self._last_activation_result or {}).get("backend_mode") or "") == "rtabmap_localization"
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        streaming_required = not localization_process
        map_ready = bool(first_frame_ready and (streaming_ready or not streaming_required))
        localization_ready = bool(process_running and input_bridge_ready and map_ready and tf_chain.get("ready"))
        if not process_running:
            message = "外部 SLAM/Nav2 脚本尚未运行。"
            waiting_for = "external_process"
        elif not input_bridge_ready:
            message = (
                "外部 SLAM/Nav2 点云输入桥未就绪；需要平台发布点云输入。"
                if point_cloud_bridge_enabled and not input_bridge_enabled
                else "外部 SLAM/Nav2 输入桥未就绪；需要平台发布 /odom、点云输入，并运行 odom -> base TF bridge。"
            )
            waiting_for = "external_input_bridge"
        elif not first_frame_ready:
            message = "等待外部 SLAM/Nav2 发布 /map_obstacles 首帧。"
            waiting_for = "external_map"
        elif streaming_required and not streaming_ready:
            message = "外部 SLAM/Nav2 已发布 /map_obstacles 首帧，正在确认连续 streaming。"
            waiting_for = "external_map_streaming"
        elif not tf_chain.get("ready"):
            if localization_process:
                message = "外部 SLAM/Nav2 定位模式已收到 /map_obstacles 首帧，等待 map -> odom -> base TF 链就绪。"
            else:
                message = "外部 SLAM/Nav2 /map_obstacles 已 streaming，等待 map -> odom -> base TF 链就绪。"
            waiting_for = "external_tf"
        else:
            if localization_process:
                message = "外部 SLAM/Nav2 定位地图首帧与 TF 链均已就绪。"
            else:
                message = "外部 SLAM/Nav2 /map_obstacles streaming 与 TF 链均已就绪。"
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
            "legacy_rtabmap": {
                "info_topic": "/rtabmap/info",
                "info_not_required": True,
                "process": None,
                "removed": True,
            },
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
                "provides_nav2": bool(getattr(self.config.external, "provides_nav2", True)),
                "provides_tf": bool(getattr(self.config.external, "provides_tf", True)),
                "input_bridge_enabled": input_bridge_enabled,
                "point_cloud_bridge_enabled": point_cloud_bridge_enabled,
                "platform_odom_tf_bridge_enabled": self._platform_odom_tf_bridge_enabled(),
                "platform_cmd_vel_bridge_enabled": self._platform_cmd_vel_bridge_enabled(),
            },
            "external_process": external_process.status() if external_process is not None else None,
        }

    def _build_tf_chain_status(self, *, tf_localization_status: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        tf_localization_status = tf_localization_status or self._clone_metadata(self._ros_bridge_status_stub("tf_localization_provider"))
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
                "external_slam_nav2_tf_provider_must_resolve_map_to_base_pose"
                if self._uses_external_geometry_stack()
                else "removed_internal_rtabmap_stack"
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
        map_fresh_window_sec = max(10.0, float(self.config.rtabmap.timestamp_quality.max_tf_age_sec) * 20.0, _GEOMETRY_STREAMING_WINDOW_SEC * 3.0)
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
        nav2_provider = getattr(self, "_nav2_action_provider", None)
        nav2_provider_name = getattr(nav2_provider, "provider_name", "nav2_action_navigation")
        return {
            "selection": "nav2_action",
            "execution_chain": "nav2_action_only",
            "single_execution_chain": True,
            "nav2_action_provider_name": nav2_provider_name,
            "nav2_action_active": nav2_provider is not None,
            "selected_provider_name": nav2_provider_name if nav2_provider is not None else None,
            "active_backend_provider": nav2_provider_name if nav2_provider is not None else None,
            "active_navigation_backend": nav2_provider_name if nav2_provider is not None else None,
            "active_backend_kind": "nav2_action" if nav2_provider is not None else None,
            "message": "当前正式导航执行链为 Nav2 action。",
        }

    def _with_navigation_backend_policy(self, status: Dict[str, object]) -> Dict[str, object]:
        policy = self._navigation_backend_policy()
        return {
            **dict(status or {}),
            "navigation_backend_policy": policy,
            "active_navigation_backend": policy.get("active_backend_provider"),
            "nav2_action_active": bool(policy.get("nav2_action_active")),
        }

    def _build_nav2_status(self) -> Dict[str, object]:
        process = self._managed_processes.get("nav2")
        provider = getattr(self, "_nav2_action_provider", None)
        if provider is not None:
            status = dict(provider.get_nav2_status())
            status.pop("launch_package", None)
            status.pop("launch_file", None)
            return self._with_navigation_backend_policy(status)
        status = {
            "phase": "external",
            "message": "Nav2 action provider 尚未初始化。",
            "action_name": self.config.nav2.action_name,
            "global_costmap_topic": self.config.nav2.global_costmap_topic,
            "local_costmap_topic": self.config.nav2.local_costmap_topic,
            "costmap_source": self.config.nav2.costmap_source,
            "action_client_connected": False,
            "lifecycle_ready": False,
            "launch_attempted": self._nav2_launch_attempted,
        }
        if process is not None:
            status["process"] = process.status()
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
            "rtabmap_readiness": self._build_rtabmap_readiness(),
            "nav2": self._build_nav2_status(),
            "external_geometry": self.config.build_metadata().get("external_geometry"),
            "external_debug_observer": self._external_debug_observer_status(),
            "tf_localization_provider": self._ros_bridge_status_stub("tf_localization_provider"),
            "diagnostics": diagnostics,
            "launch_ros2_stack": {
                "script": str(self._geometry_script("launch_ros2_stack.sh")),
                "starts_rtabmap_or_nav2_by_default": False,
                "message": "launch_ros2_stack.sh 只用于可选 ROS2 桥环境；旧 RTAB-Map/Nav2 内置启动链路已删除。",
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
        """合成 GO2_RTABNAV2 外部栈 TF 就绪状态。

        Rust bridge GetStatus 不含 pose_ready/map_to_base_ready；探索/建图时 RTAB-Map
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
                    source_kind = source_kind or "ros_bridge_pose"
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
                            source_kind = source_kind or "ros_bridge_tf_chain"
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

    def _ros_bridge_status_stub(self, provider_key: str) -> Dict[str, object]:
        """返回 Rust bridge 替代 Python provider 后的状态占位 dict，保持调用方 key 兼容。"""
        remote_status = {}
        if self._remote_provider is not None:
            try:
                remote_status = self._remote_provider.get_status() or {}
            except Exception:
                remote_status = {}
        stub = {
            "provider": "ros_bridge",
            "provider_key": provider_key,
            "status": "active" if self._ros_bridge_handles_input_bridge() else "inactive",
            "ros_bridge_connected": self._ros_bridge_handles_input_bridge(),
            **remote_status,
        }
        # /map 占据栅格的真实状态由外部 observer 提供（Rust get_status 不含占据字段），
        # 仅对 live_map_provider 合并 observer 快照，使其 occupancy 字段生效。
        if provider_key == "live_map_provider":
            with self._observer_occupancy_lock:
                observer_snapshot = dict(self._observer_occupancy_snapshot)
            if observer_snapshot:
                stub.update(observer_snapshot)
        if provider_key == "tf_localization_provider":
            stub = self._resolve_external_tf_localization_status(base_stub=stub)
        return stub

    def _ros_bridge_handles_input_bridge(self) -> bool:
        """Rust bridge 直接订阅 Go2 ROS2 话题时，Python publishers 无需启动。"""
        return (
            self._ros_bridge_subprocess is not None
            and self._ros_bridge_subprocess.is_running
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
            ros_bridge_active = self._ros_bridge_handles_input_bridge()
            if ros_bridge_active:
                LOGGER.debug("Rust bridge 活跃，跳过 Python odom/pointcloud publishers")
            input_bridge_ready = True
            if ros_bridge_active:
                input_bridge_ready = True  # Rust bridge 已处理 odom/pointcloud 输入
            elif input_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            elif point_cloud_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            if ros_bridge_active and input_bridge_ready:
                return
            self._live_geometry_start_attempted = True
            try:
                pass  # Python providers 已移除，由 Rust bridge (RosBridgeProvider) 替代
            except Exception:
                self._live_geometry_start_attempted = False
                raise

    def _ensure_external_debug_observer_started_locked(self) -> None:
        observer = self._external_debug_observer
        if observer is None:
            observer = ExternalGeometryDebugObserver(
                config=self.config,
                map_callback=self._handle_external_debug_map_update,
                frontier_callback=self._handle_external_debug_frontier_update,
                pose_callback=self._handle_external_debug_pose_update,
            )
            self._external_debug_observer = observer
        if not observer.is_running:
            observer.start()
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
        observer.set_session_mode(session_mode)

    def _on_external_geometry_session_mode_changed(self, mode: Optional[str]) -> None:
        """同步 debug observer 会话模式，并在非 explore 时清空前沿缓存。"""
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

    def _handle_external_debug_map_update(self, payload: Dict[str, object]) -> None:
        self._cache_observer_occupancy_snapshot(payload)
        callbacks = list(self._pending_map_callbacks)
        for callback in callbacks:
            try:
                callback(payload)
            except Exception:
                LOGGER.debug("投递外部 GO2 /map debug update 失败。", exc_info=True)

    def _cache_observer_occupancy_snapshot(self, payload: Dict[str, object]) -> None:
        """从外部 /map 观察器 payload 提取就绪元数据并缓存。

        观察器 metadata 不含 `occupancy_grid_ready` / `occupancy_has_publisher`，
        这里据 occupancy_message_count 显式合成，供 readiness 链路读取。
        """
        metadata = payload.get("metadata") if isinstance(payload, dict) else None
        if not isinstance(metadata, dict):
            return
        message_count = int(metadata.get("occupancy_message_count") or metadata.get("message_count") or 0)
        snapshot: Dict[str, object] = {
            "occupancy_grid_ready": message_count > 0,
            "occupancy_has_publisher": True,
            "occupancy_message_count": message_count,
            "occupancy_sequence": int(metadata.get("occupancy_sequence") or message_count),
            "cost_map_message_count": int(metadata.get("cost_map_message_count") or 0),
            "occupancy_topic": str(metadata.get("source_topic") or self.config.topics.occupancy_topic or "/map_obstacles"),
            "streaming": bool(metadata.get("streaming")),
            "known_cell_count": int(metadata.get("known_cell_count") or 0),
        }
        for key in (
            "latest_occupancy_timestamp",
            "latest_occupancy_received_at",
            "latest_occupancy_source_timestamp",
        ):
            value = metadata.get(key)
            if value is not None:
                snapshot[key] = value
        occupancy_metrics = metadata.get("occupancy_metrics")
        if isinstance(occupancy_metrics, dict):
            snapshot["occupancy_metrics"] = dict(occupancy_metrics)
        cost_map_metrics = metadata.get("cost_map_metrics")
        if isinstance(cost_map_metrics, dict):
            snapshot["cost_map_metrics"] = dict(cost_map_metrics)
        with self._observer_occupancy_lock:
            self._observer_occupancy_snapshot = snapshot

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
        """RViz 2D Pose Estimate：向 RTAB-Map 发布 /initialpose 触发重定位。"""

        from contracts.geometry_runtime import DEFAULT_MAP_FRAME
        from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

        frame_id = str(target_pose.frame_id or DEFAULT_MAP_FRAME).strip().lstrip("/") or DEFAULT_MAP_FRAME
        topics = ("/initialpose",)
        publish_attempts = 2
        wait_for_subscriber_sec = 3.0
        publish_interval_sec = 0.2
        node = None
        rclpy = None
        has_context = False
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.duration import Duration
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

            rclpy = acquire_ros2_context()
            has_context = True
            node = rclpy.create_node("nuwax_debug_initialpose_publisher")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            publishers = {
                topic: node.create_publisher(PoseWithCovarianceStamped, topic, qos)
                for topic in topics
            }
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = (node.get_clock().now() - Duration(seconds=0.5)).to_msg()
            msg.pose.pose.position.x = float(target_pose.position.x)
            msg.pose.pose.position.y = float(target_pose.position.y)
            msg.pose.pose.position.z = float(target_pose.position.z)
            msg.pose.pose.orientation.x = float(target_pose.orientation.x)
            msg.pose.pose.orientation.y = float(target_pose.orientation.y)
            msg.pose.pose.orientation.z = float(target_pose.orientation.z)
            msg.pose.pose.orientation.w = float(target_pose.orientation.w)
            msg.pose.covariance = [0.25 if index in (0, 7, 35) else 0.0 for index in range(36)]

            deadline = time.monotonic() + wait_for_subscriber_sec
            subscriber_counts: Dict[str, int] = {topic: 0 for topic in topics}
            while time.monotonic() < deadline:
                for topic, publisher in publishers.items():
                    subscriber_counts[topic] = int(publisher.get_subscription_count())
                if any(count > 0 for count in subscriber_counts.values()):
                    break
                time.sleep(0.1)

            self._set_debug_initial_pose_hint(target_pose)
            for attempt in range(publish_attempts):
                for publisher in publishers.values():
                    publisher.publish(msg)
                if attempt + 1 < publish_attempts:
                    time.sleep(publish_interval_sec)

            self.invalidate_external_live_pose_cache()
            hint_pose = target_pose.model_copy(deep=True)
            self._handle_external_debug_pose_update(
                hint_pose,
                metadata={"source_kind": "debug_initial_pose_hint", "source_name": "nuwax_debug_initialpose_publisher"},
            )
            total_subscribers = sum(subscriber_counts.values())
            mapping_hint = (
                "当前为建图模式时 /initialpose 主要用于重定位 hint，效果弱于定位模式。"
                if total_subscribers == 0
                else None
            )
            log_line = (
                "debug 2D Pose 已发布 /initialpose pose=(%.3f, %.3f, %.3f) frame=%s "
                "subscribers=%s attempts=%d"
                % (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    frame_id,
                    subscriber_counts,
                    publish_attempts,
                )
            )
            LOGGER.info(log_line)
            _append_main_log_line(_GO2_SH_MAIN_LOG_LOGGER, log_line)
            message = "已发布 /initialpose（RViz 2D Pose Estimate 等价）。"
            if total_subscribers == 0:
                message = (
                    "已发布 /initialpose，但当前无 RTAB-Map 订阅者；"
                    "请确认 go2.sh 已启动且 rtabmap 节点在运行。"
                )
            return {
                "published": True,
                "topic": "/initialpose",
                "topics": list(topics),
                "frame_id": frame_id,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "subscriber_counts": subscriber_counts,
                "publish_attempts": publish_attempts,
                "mapping_hint": mapping_hint,
                "message": message,
            }
        except Exception as exc:
            LOGGER.info("debug 发布 /initialpose 失败：%s", exc)
            return {
                "published": False,
                "topic": "/initialpose",
                "topics": list(topics),
                "message": "发布 /initialpose 失败：%s" % exc,
            }
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if has_context:
                release_ros2_context()

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
        callbacks = list(self._pending_localization_callbacks)
        meta = dict(metadata or {})
        for callback in callbacks:
            try:
                callback(resolved_pose, frame_tree, meta)
            except Exception:
                LOGGER.debug("投递外部 GO2 位姿 debug update 失败。", exc_info=True)

    def _handle_external_debug_frontier_update(self, payload: Dict[str, object]) -> None:
        with self._exploration_loop_lock:
            if self._external_geometry_session_mode != "explore":
                return
        terminal_state = None
        with self._exploration_loop_lock:
            self._external_debug_frontier_payload = self._clone_metadata(payload)
            has_external_request = self._external_exploration_request is not None
        metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        if has_external_request and self._metadata_indicates_exploration_done(metadata):
            terminal_status = str(metadata.get("terminal_status") or "succeeded").lower()
            status = ExplorationStatus.SUCCEEDED if terminal_status == "succeeded" else ExplorationStatus.FAILED
            message = "GO2_RTABNAV2 原生探索已完成。"
            status_text = str(metadata.get("status_text") or "").strip()
            if status_text:
                message = "GO2_RTABNAV2 原生探索已完成：%s" % status_text
            terminal_state = self._finish_external_script_exploration(
                status=status,
                reason="external_explorer_terminal_status",
                message=message,
                notify_compat=False,
                kill_geometry_process=False,
            )
        try:
            if terminal_state is not None:
                self._notify_exploration_update(terminal_state)
            else:
                # 热路径：避免每次 frontier 回调扫描外部日志与完整 runtime 诊断，
                # 否则会阻塞 ROS observer 单线程，导致 debug 面板地图/路径滞后数秒。
                self._notify_exploration_update(self._build_fast_external_exploration_state_from_frontier())
        except Exception:
            LOGGER.debug("投递外部 GO2 frontier debug update 失败。", exc_info=True)

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
            if backend is getattr(self, "_platform_exploration_runtime", None):
                source_kind = "platform_runtime"
            elif backend is getattr(self.robot, "data_plane_runtime", None):
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
            "live_map_provider": self._ros_bridge_status_stub("live_map_provider"),
            "tf_localization_provider": self._ros_bridge_status_stub("tf_localization_provider"),
            "odometry_publisher": self._ros_bridge_status_stub("odometry_publisher"),
            "point_cloud_publisher": self._ros_bridge_status_stub("point_cloud_publisher"),
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
        live_map_status = self._clone_metadata(self._ros_bridge_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._ros_bridge_status_stub("tf_localization_provider"))
        odometry_status = self._clone_metadata(self._ros_bridge_status_stub("odometry_publisher"))
        point_cloud_status = self._clone_metadata(self._ros_bridge_status_stub("point_cloud_publisher"))
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
            "rtabmap_profile_enabled": self.config.rtabmap.profile_enabled,
            "runtime_status": self.get_runtime_status(),
            "rtabmap_readiness": self._build_rtabmap_readiness(
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
            "require_ros2": self.config.require_ros2,
            "odom_topic": self.config.topics.odom_topic,
            "point_cloud_topic": self.config.topics.point_cloud_topic,
            "occupancy_topic": self.config.topics.occupancy_topic,
            "cost_map_topic": self.config.topics.cost_map_topic,
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "geometry_profile_name": self.config.rtabmap.profile_name,
            "rtabmap_launch_removed": True,
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
    ) -> tuple[Optional[str], str]:
        live_map_error = str(live_map_status.get("last_error") or "").strip()
        if live_map_error:
            return "live_map_provider_error", "ROS2 实时地图订阅错误：%s" % live_map_error
        occupancy_grid_ready = bool(live_map_status.get("occupancy_grid_ready"))
        if self._uses_external_geometry_stack() and not self._external_point_cloud_bridge_enabled():
            occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
            if not occupancy_grid_ready:
                return (
                    "external_occupancy_grid",
                    "等待外部 SLAM/Nav2 在 %s 发布首帧占据栅格。"
                    % occupancy_topic,
                )
            return None, "外部 SLAM/Nav2 建图链路已收到 %s 首帧占据栅格。" % occupancy_topic
        point_cloud_error = str(point_cloud_status.get("last_error") or "").strip()
        if point_cloud_error:
            return "point_cloud_bridge_error", "ROS2 点云桥错误：%s" % point_cloud_error
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
                "点云桥已向 %s 发布 %d 帧，等待外部 SLAM/Nav2 在 %s 发布首帧占据栅格。"
                % (point_cloud_topic, published_count, occupancy_topic),
            )
        occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
        return None, "外部 SLAM/Nav2 建图链路已收到 %s 首帧占据栅格。" % occupancy_topic

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
            return "GO2_RTABNAV2 原生探索已完成：%s" % cleaned
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
        rtabmap_database_path = self._version_rtabmap_database_path(version.map_name, version.version_id)
        pcd_path = self._version_pcd_asset_path(version.map_name, version.version_id)
        built_at = previous_asset.built_at if previous_asset is not None else version.timestamp
        backend_run = self._build_rtabmap_run_metadata()
        database_metadata = self._materialize_backend_state(rtabmap_database_path)
        database_for_pcd = rtabmap_database_path if rtabmap_database_path.exists() and rtabmap_database_path.is_file() else None
        # Nav2 归档来自 version 快照，不依赖 PCD 导出；PCD 失败不应阻塞 yaml/geometry_asset 落盘。
        nav2_map_metadata = self._materialize_nav2_map_assets(version)
        if skip_pcd_export:
            pcd_metadata = {
                "pcd_present": self._is_usable_pcd_file(pcd_path),
                "pcd_source_kind": "skipped_for_activation",
                "pcd_target_path": str(pcd_path.resolve(strict=False)),
            }
        else:
            pcd_metadata = self._materialize_pcd_asset(pcd_path, database_path=database_for_pcd)
        quality_metrics = self._build_geometry_quality_metrics(
            database_metadata=database_metadata,
            pcd_metadata=pcd_metadata,
            rtabmap_run=backend_run,
            nav2_map_metadata=nav2_map_metadata,
        )
        database_quality = dict(quality_metrics.get("database") or {})
        database_required = bool(database_quality.get("required", True))
        database_present = bool(database_quality.get("present", False))
        # RTAB-Map .db 是唯一 SLAM 后端资产类型。
        database_asset_type = MapGeometryAssetType.RTABMAP_DATABASE
        asset_refs = []
        if database_required or database_present:
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=database_asset_type,
                    path=str(rtabmap_database_path),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=database_quality,
                    metadata={"required": database_required},
                )
            )
        nav2_map_quality = dict(quality_metrics.get("nav2_map") or {})
        if self._uses_external_geometry_stack() and bool(nav2_map_quality.get("present")):
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.EXTERNAL_SLAM_NAV2_STATE,
                    path=str(nav2_map_quality.get("yaml_path") or nav2_map_metadata.get("yaml_path")),
                    frame_id=version.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=nav2_map_quality,
                    metadata={"required": True, "state_kind": "nav2_map_yaml"},
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
        geometry_asset = MapGeometryAssetBinding(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_stack=self.config.stack,
            frame_id=version.frame_id,
            rtabmap_database_path=str(rtabmap_database_path),
            # 老 version.json 兼容：rtabmap_db_path 与 rtabmap_database_path 同步写。
            rtabmap_db_path=str(rtabmap_database_path),
            pcd_path=str(pcd_path),
            occupancy_projection=self.config.rtabmap.occupancy_projection,
            costmap_source=self.config.nav2.costmap_source,
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
                "navigable_layer": self.config.rtabmap.occupancy_projection,
                "traversability_layer": self.config.rtabmap.outputs.traversability_layer,
                "localization_status_contract": self.config.rtabmap.outputs.localization_status,
                "relocalization_diagnostics_contract": self.config.rtabmap.outputs.relocalization_diagnostics,
                "source_mode": quality_metrics.get("source_mode"),
                "streaming_readiness": self._clone_metadata(quality_metrics.get("streaming_readiness")),
                "backend_run": backend_run,
                # 兼容老消费方继续读 rtabmap_run。
                "rtabmap_run": backend_run,
                "session_id": backend_run.get("session_id"),
                "run_id": backend_run.get("run_id"),
                "database": database_metadata,
                "pcd": pcd_metadata,
                "nav2_map": nav2_map_metadata,
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
                    "rtabmap_database_path": database_metadata.get("database_source_path") or database_metadata.get("database_target_path"),
                    "backend_streaming_ready": quality_metrics.get("live_streaming_ready"),
                    # 老消费方读老 metadata key 时仍能解析（迁移期保留 alias）。
                    "rtabmap_session_id": backend_run.get("session_id"),
                    "rtabmap_run_id": backend_run.get("run_id"),
                    "rtabmap_live_streaming_ready": quality_metrics.get("live_streaming_ready"),
                    "map_version_source_mode": quality_metrics.get("source_mode"),
                    "checkpoint_ready": quality_metrics.get("checkpoint_ready"),
                    "nav2_map_yaml_path": nav2_map_metadata.get("yaml_path"),
                    "nav2_map_ready": nav2_map_metadata.get("present"),
                },
            },
            deep=True,
        )

    def _version_db_ready_for_activation(self, version: MapVersion) -> bool:
        db_path = self._resolve_existing_rtabmap_db_for_version(version)
        return db_path is not None and self._is_usable_rtabmap_db_file(db_path)

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
        def wrapped_callback(payload):
            return callback(self._attach_active_map_version_to_live_map_payload(payload))

        self._pending_map_callbacks.append(wrapped_callback)
        if self._remote_provider is not None and self._remote_provider.is_connected:
            self._remote_provider.subscribe_map_updates(wrapped_callback)
        return 0

    def unsubscribe_map_updates(self, subscription_id: int) -> bool:
        return True  # Rust bridge 管理订阅生命周期，无需手动注销

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
        if self._platform_exploration_runtime.is_running:
            pass  # 探索模式下不抑制 live map（新建地图从小到大增长是正常的）
        elif self._external_geometry_process_is_localization() or str(
            (self._last_activation_result or {}).get("backend_mode") or ""
        ) == "rtabmap_localization":
            pass  # 定位模式以 RTAB-Map .db 为准，允许 live /map 从空到完整逐步刷新
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
        live_status = self._clone_metadata(self._ros_bridge_status_stub("live_map_provider"))
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
            "rtabmap_live_streaming_ready": True,
            "source_name": live_status.get("provider_name") or "ros_bridge",
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
                "occupancy_projection": self.config.rtabmap.occupancy_projection,
                "costmap_source": self.config.nav2.costmap_source,
                "parameter_summary": self._build_parameter_summary(),
                "profile": self._clone_metadata(self.config.build_metadata().get("profile")),
                "input_contract": self._clone_metadata(self.config.build_metadata().get("input_contract")),
                "output_contract": self._clone_metadata(self.config.build_metadata().get("output_contract")),
                "preprocessing": self._clone_metadata(self.config.build_metadata().get("preprocessing")),
                "geometry_backend": self.describe_backend(),
            })
        return MapSnapshot(
            source_name=str(live_status.get("provider_name") or "ros_bridge"),
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
            "rtabmap_live_streaming_ready": bool(live_available),
            **version_metadata,
            "geometry_stack": geometry_asset.geometry_stack if geometry_asset is not None else self.config.stack,
            "rtabmap_db_path": geometry_asset.rtabmap_db_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "occupancy_projection": geometry_asset.occupancy_projection if geometry_asset is not None else self.config.rtabmap.occupancy_projection,
            "costmap_source": geometry_asset.costmap_source if geometry_asset is not None else self.config.nav2.costmap_source,
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
        metadata["rtabmap_live_streaming_ready"] = bool(live_available)
        metadata.update(version_metadata)
        return metadata

    def _is_geometry_profile_enabled(self) -> bool:
        return bool(self.config.enabled and self.config.rtabmap.profile_enabled)

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
        if not self._is_geometry_profile_enabled():
            return None
        return getattr(self, "_nav2_action_provider", None)

    def _resolve_exploration_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = getattr(self, "_platform_exploration_runtime", None)
        if backend is not None:
            return backend
        return self._resolve_backend(ExplorationDataPlane, ExplorationProvider)

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
        """探索/定位使用的 .db 文件名与地图名一致，避免多图共用 live_rtabmap.db。"""

        safe_name = build_safe_dir_segment(map_name) or "map"
        return "%s.db" % safe_name

    def _version_rtabmap_database_path(self, map_name: str, version_id: str) -> Path:
        """返回该地图版本的 RTAB-Map .db 状态文件路径。"""

        return self._version_geometry_root(map_name, version_id) / "rtabmap" / self._map_named_backend_db_filename(map_name)

    def _candidate_version_rtabmap_database_paths(self, map_name: str, version_id: str) -> List[Path]:
        """主路径为 <map_name>.db，兼容旧版 external_slam_nav2_state.db。"""

        rtabmap_dir = self._version_geometry_root(map_name, version_id) / "rtabmap"
        candidates = [
            rtabmap_dir / self._map_named_backend_db_filename(map_name),
            rtabmap_dir / self._external_slam_nav2_state_asset_name(),
            rtabmap_dir / _EXTERNAL_SLAM_NAV2_STATE_ASSET_NAME,
        ]
        seen: set[str] = set()
        ordered: List[Path] = []
        for candidate in candidates:
            key = str(candidate)
            if key in seen:
                continue
            seen.add(key)
            ordered.append(candidate)
        return ordered

    def _external_slam_nav2_state_asset_name(self) -> str:
        configured_name = str(getattr(self.config.rtabmap.outputs, "database_asset_name", "") or "").strip()
        if not configured_name or configured_name == "rtabmap.db":
            return _EXTERNAL_SLAM_NAV2_STATE_ASSET_NAME
        return configured_name

    def _version_pcd_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "pcd" / self.config.rtabmap.outputs.pcd_asset_name

    def _version_nav2_map_yaml_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "nav2" / "map.yaml"

    def _version_nav2_map_pgm_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "nav2" / "map.pgm"

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

    def _materialize_rtabmap_database(self, target_path: Path) -> Dict[str, object]:
        """Deprecated alias —— 转向 _materialize_backend_state。"""

        return self._materialize_backend_state(target_path)

    def _materialize_backend_state(self, target_path: Path) -> Dict[str, object]:
        """持久化 RTAB-Map .db 状态到版本目录（sqlite3 .backup 安全拷贝）。"""

        return self._materialize_rtabmap_db(target_path)

    def _is_usable_rtabmap_db_file(self, path: Path) -> bool:
        resolved = path.expanduser().resolve(strict=False)
        if not resolved.is_file():
            return False
        try:
            return int(resolved.stat().st_size) > 4096
        except OSError:
            return False

    def _locate_runtime_backend_state_for_materialize(self, target_path: Path) -> Optional[Path]:
        """定位可拷贝到版本目录的运行中 .db，避免误用其它地图版本或 active 版本路径。"""

        resolved_target = target_path.expanduser().resolve(strict=False)
        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_rtabmap_database_path", None)
            if callable(getter):
                candidate = Path(str(getter() or "").strip()).expanduser()
                if (
                    candidate.suffix == ".db"
                    and self._is_usable_rtabmap_db_file(candidate)
                    and candidate.resolve(strict=False) != resolved_target
                ):
                    return candidate

        configured_root = Path(str(self.config.rtabmap.database_root or "").strip()).expanduser()
        candidates: List[Path] = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "live_rtabmap.db",
                    configured_root / "rtabmap.db",
                )
            )
        for candidate in candidates:
            if (
                self._is_usable_rtabmap_db_file(candidate)
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
        if candidate.suffix == ".db" and self._is_usable_rtabmap_db_file(candidate):
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

        if self._is_usable_rtabmap_db_file(resolved_target):
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

    def _materialize_rtabmap_db(self, target_path: Path) -> Dict[str, object]:
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
            and self._is_usable_rtabmap_db_file(resolved_target)
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
        *,
        database_path: Optional[Path] = None,
    ) -> Dict[str, object]:
        """将 RTAB-Map 累计点云落盘为版本 PCD（优先实时 /cloud_map，其次 .db 导出）。"""

        from services.geometry.cloud_map_fetcher import (
            database_export_enabled,
            default_db_export_timeout_sec,
            export_cloud_map_pcd_from_database,
            fetch_cloud_map_xyz,
        )
        from services.geometry.pointcloud_map import PointCloudMapError, write_pcd_binary_xyz

        target_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_target = target_path.expanduser().resolve()
        min_points = self._pcd_min_points()
        cloud_topic = str(getattr(self.config.topics, "cloud_map_topic", "") or "/cloud_map").strip() or "/cloud_map"
        map_frame = self.config.frames.map_frame

        if self._is_usable_pcd_file(resolved_target):
            return {
                "pcd_present": True,
                "pcd_source_kind": "persisted_asset",
                "pcd_source_path": str(resolved_target),
                "pcd_target_path": str(resolved_target),
                "pcd_target": self._file_metadata(resolved_target),
            }

        fetched = fetch_cloud_map_xyz(
            cloud_topic,
            map_frame=map_frame,
            min_points=min_points,
        )
        if fetched is not None:
            xyz, frame_id, source_topic = fetched
            write_pcd_binary_xyz(resolved_target, xyz)
            return {
                "pcd_present": True,
                "pcd_source_kind": "ros_cloud_map",
                "pcd_source_path": source_topic,
                "pcd_target_path": str(resolved_target),
                "pcd_point_count": int(xyz.shape[0]),
                "pcd_frame_id": frame_id,
                "pcd_target": self._file_metadata(resolved_target),
            }

        db_candidate = database_path if database_path is not None else self._locate_live_backend_state()
        if db_candidate is not None and database_export_enabled():
            resolved_db = Path(db_candidate).expanduser()
            if resolved_db.suffix == ".db" and resolved_db.is_file():
                try:
                    point_count = export_cloud_map_pcd_from_database(
                        resolved_db,
                        resolved_target,
                        min_points=min_points,
                        timeout_sec=default_db_export_timeout_sec(),
                    )
                    return {
                        "pcd_present": True,
                        "pcd_source_kind": "rtabmap_export_scan",
                        "pcd_source_path": str(resolved_db.resolve(strict=False)),
                        "pcd_target_path": str(resolved_target),
                        "pcd_point_count": int(point_count),
                        "pcd_target": self._file_metadata(resolved_target),
                    }
                except (PointCloudMapError, subprocess.TimeoutExpired) as exc:
                    LOGGER.warning("从 RTAB-Map 数据库导出累计点云失败：%s", exc)

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
        rtabmap_run: Dict[str, object],
        nav2_map_metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        diagnostics = []
        nav2_map_metadata = dict(nav2_map_metadata or {})
        database_present = bool(database_metadata.get("database_present", False))
        external_stack = self._uses_external_geometry_stack()
        nav2_map_present = bool(nav2_map_metadata.get("present", False))
        database_required = not external_stack
        pcd_present = bool(pcd_metadata.get("pcd_present", False))
        readiness = self._build_rtabmap_readiness()
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
                    "code": "rtabmap_database_missing",
                    "message": "地图版本缺少 RTAB-Map database 资产。",
                }
            )
        if external_stack and not nav2_map_present:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "nav2_map_asset_missing",
                    "message": "地图版本缺少 Nav2 map.yaml/PGM 资产，无法加载到底层 SLAM/Nav2 定位状态。",
                }
            )
        if not database_present and not database_required:
            diagnostics.append(
                {
                    "level": "info" if nav2_map_present else "warning",
                    "code": "external_geometry_state_not_materialized",
                    "message": "未找到 external_slam_nav2 状态数据库；当前 external_slam_nav2 版本恢复以 Nav2 map.yaml/PGM + AMCL 为后端检查点。",
                }
            )
        if database_present and not live_streaming_ready:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "external_geometry_state_checkpoint_only",
                    "message": "external_slam_nav2 状态数据库已落盘为离线 checkpoint，但未证明外部几何状态正在随 /map streaming 更新。",
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
                        "message": "地图版本 PCD 点云无效（点数过少或为模板占位文件），请重新保存或从 RTAB-Map 数据库导出。",
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
        if not rtabmap_run.get("run_id"):
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "rtabmap_run_id_missing",
                    "message": "未能从外部 SLAM/Nav2 进程状态解析真实 run_id。",
                }
            )
        # external_slam_nav2 后端使用 RTAB-Map：backend_checkpoint_ready 证明 .db 已落盘。
        # status="ready" 必须 backend checkpoint 与 nav2 map 同时就位，否则只是 snapshot_only。
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
            "nav2_map_present": nav2_map_present,
            "platform_snapshot_only": bool(external_stack and not backend_checkpoint_ready),
            "backend_switch_supported": True,
            "source_mode": (
                "live_streaming_checkpoint" if live_streaming_ready
                else "rtabmap_db_checkpoint" if external_stack and backend_checkpoint_ready
                else "nav2_map_checkpoint" if external_stack and nav2_map_present
                else "platform_snapshot_only" if external_stack
                else "offline_checkpoint"
            ),
            "streaming_readiness": readiness,
            "rtabmap_run": rtabmap_run,
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
            "nav2_map": {
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
            "rtabmap_db_path": geometry_asset.rtabmap_db_path,
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

    def _build_rtabmap_run_metadata(self) -> Dict[str, object]:
        process_name = self._external_process_name()
        process = self._managed_processes.get(process_name)
        db_path = self._locate_live_rtabmap_database()
        started_at = process.started_at_iso if process is not None else None
        pid = process.process.pid if process is not None and process.process is not None and process.is_running() else None
        source = str(db_path.resolve()) if db_path is not None else None
        seed = "|".join(str(part or "") for part in (source, started_at, pid))
        digest = hashlib.sha256(seed.encode("utf-8")).hexdigest()[:16] if seed.strip("|") else ""
        run_id = "external_slam_nav2_%s" % digest if digest else None
        return {
            "session_id": run_id,
            "run_id": run_id,
            "database_path": source,
            "started_at": started_at,
            "pid": pid,
            "node_name": process_name,
            "source_stack": self.config.stack,
            "rtabmap_launch_removed": True,
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
                    rtabmap_db_path=geometry_asset.rtabmap_db_path,
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
        configured_value = str(self.config.rtabmap.pcd_root or "").strip()
        if not configured_value:
            return []
        configured_root = Path(configured_value).expanduser()
        if configured_root.suffix == ".pcd":
            return [configured_root]
        return [
            configured_root / self.config.rtabmap.outputs.pcd_asset_name,
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
        rtabmap_db_path: Optional[str] = None,
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
            "rtabmap_db_path": rtabmap_db_path,
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

    def _locate_live_rtabmap_database(self) -> Optional[Path]:
        """Deprecated alias —— 转向 _locate_live_backend_state。"""

        return self._locate_live_backend_state()

    def _locate_live_backend_state(self) -> Optional[Path]:
        """定位当前活跃的 RTAB-Map .db 状态文件。"""

        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            asset = self._active_map_version.geometry_asset
            current_path = Path(asset.rtabmap_database_path or asset.rtabmap_db_path or "").expanduser()
            if current_path.suffix == ".db" and current_path.exists() and current_path.is_file():
                return current_path

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_rtabmap_database_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if candidate_path.suffix == ".db" and candidate_path.exists() and candidate_path.is_file():
                        return candidate_path

        configured_root = Path(str(self.config.rtabmap.database_root or "").strip()).expanduser()
        candidates: List[Path] = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "live_rtabmap.db",
                    configured_root / "rtabmap.db",
                    configured_root / "current.db",
                    configured_root / "active.db",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _resolve_existing_rtabmap_db_for_version(self, version: MapVersion) -> Optional[Path]:
        """Find the .db file for a specific map version."""
        geometry_asset = version.geometry_asset
        candidates: List[Path] = []
        if geometry_asset is not None:
            for raw in (geometry_asset.rtabmap_database_path, geometry_asset.rtabmap_db_path):
                if raw:
                    candidates.append(Path(str(raw)).expanduser())
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type in (
                    MapGeometryAssetType.RTABMAP_DATABASE,
                    MapGeometryAssetType.EXTERNAL_SLAM_NAV2_STATE,
                ):
                    candidates.append(Path(asset_ref.path).expanduser())
        candidates.append(self._version_rtabmap_database_path(version.map_name, version.version_id))
        for candidate in self._candidate_version_rtabmap_database_paths(version.map_name, version.version_id):
            if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
                return candidate
        for candidate in candidates:
            if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _invoke_ros2_service_call(
        self,
        *,
        service_name: str,
        service_type: str,
        request: str,
        timeout_sec: float = 8.0,
    ) -> Dict[str, object]:
        """对当前 ROS2 环境发一次 `ros2 service call` 同步请求。"""

        command = self._build_launch_prefix() + [
            "ros2",
            "service",
            "call",
            service_name,
            service_type,
            request,
        ]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(2.0, float(timeout_sec)),
            )
        except subprocess.TimeoutExpired as exc:
            return {
                "ok": False,
                "service": service_name,
                "service_type": service_type,
                "request": request,
                "error": "timeout",
                "timeout_sec": exc.timeout,
                "command": command,
            }
        except FileNotFoundError as exc:
            return {
                "ok": False,
                "service": service_name,
                "service_type": service_type,
                "request": request,
                "error": "ros2_cli_unavailable",
                "message": str(exc),
                "command": command,
            }
        return {
            "ok": completed.returncode == 0,
            "service": service_name,
            "service_type": service_type,
            "request": request,
            "returncode": completed.returncode,
            "stdout_tail": (completed.stdout or "").strip().splitlines()[-5:],
            "stderr_tail": (completed.stderr or "").strip().splitlines()[-5:],
            "command": command,
        }

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
        backend_state = (
            geometry_asset.rtabmap_database_path or geometry_asset.rtabmap_db_path
            if geometry_asset is not None
            else None
        )
        activation_payload = {
            "map_name": version.map_name,
            "version_id": version.version_id,
            "geometry_stack": self.config.stack,
            "rtabmap_database_path": backend_state,
            # 老消费方兼容：保持 rtabmap_db_path 字段读得到。
            "rtabmap_db_path": backend_state,
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
        db_path = self._resolve_existing_rtabmap_db_for_version(version)
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
            backend_mode = "rtabmap_localization" if db_has_map_data else "rtabmap_mapping"
            use_static_map_yaml = self._external_localization_uses_static_map_yaml()
            static_map_yaml_path = (
                nav2_yaml_path
                if db_has_map_data and nav2_yaml_path is not None and use_static_map_yaml
                else None
            )
            static_map_requested = static_map_yaml_path is not None
            if db_has_map_data:
                self.clear_external_exploration_state(reason="activate_rtabmap_localization")
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
                with self._observer_occupancy_lock:
                    self._observer_occupancy_snapshot = {}
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_map_loaded": False,
                    "backend_mode": "unavailable",
                    "active_backend_version_id": version.version_id,
                    "reason": "rtabmap_empty_db_no_auto_mapping",
                    "rtabmap": {
                        "switch_method": "stop_previous_no_start",
                        "database_path": str(resolved_db),
                        "note": "mapping is manual-only; use explore_area to start --explore",
                    },
                    "nav2_map": {
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
                    "reason": "rtabmap_localization_already_active",
                    "rtabmap": {
                        "switch_method": "reuse_running_localization",
                        "database_path": str(resolved_db),
                    },
                    "nav2_map": {
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
            with self._observer_occupancy_lock:
                self._observer_occupancy_snapshot = {}
            command = [str(self._external_script_path()), "--localization", str(db_path)]
            if static_map_yaml_path is not None:
                command.extend(["--map-yaml", str(static_map_yaml_path)])
            # GO2_RTABNAV2 localization reads the passed .db directly (localize_only:=true).
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
                    "rtabmap": {
                        "switch_method": "process_restart_localization",
                        "database_path": str(db_path),
                    },
                    "nav2_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": static_map_requested,
                    },
                }
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "rtabmap_localization_start_failed",
                "backend_mode": "unavailable",
                "rtabmap": {"database_path": str(db_path)},
                "nav2_map": {
                    **nav2_map_metadata,
                    "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                    "used_for_static_map": False,
                },
            }

        # Fallback: try Nav2 AMCL with map.yaml
        if nav2_yaml_path is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "nav2_map_yaml_missing_and_rtabmap_db_missing",
                "backend_mode": "unavailable",
                "nav2_map": nav2_map_metadata,
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
        with self._observer_occupancy_lock:
            self._observer_occupancy_snapshot = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=restart)
        process = self._managed_processes.get(process_name)
        process_status = process.status() if process is not None else None
        process_running = bool(process is not None and process.is_running())
        if not process_running:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "external_nav2_amcl_start_failed",
                "backend_mode": "amcl_localization_only",
                "nav2_map": nav2_map_metadata,
                "nav2": {
                    "switch_method": "managed_ros2_launch_amcl",
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
            "reason": "external_nav2_amcl_started",
            "nav2": {
                "switch_method": "managed_ros2_launch_amcl",
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
            metadata.update(self._clone_metadata(geometry_asset.quality_metrics.get("nav2_map") or {}))
            metadata.update(self._clone_metadata(geometry_asset.metadata.get("nav2_map") or {}))
        version_metadata = getattr(version, "metadata", None)
        if isinstance(version_metadata, dict):
            yaml_path = version_metadata.get("nav2_map_yaml_path")
            if yaml_path and not metadata.get("yaml_path"):
                metadata["yaml_path"] = yaml_path
            ready = version_metadata.get("nav2_map_ready")
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
                if asset_ref.asset_type != MapGeometryAssetType.EXTERNAL_SLAM_NAV2_STATE:
                    continue
                asset_metadata = dict(asset_ref.metadata or {})
                if asset_metadata.get("state_kind") == "nav2_map_yaml":
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
        """向 /initialpose 发布 identity PoseWithCovarianceStamped，触发 AMCL 初始化。"""
        from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context
        node = None
        rclpy = None
        has_context = False
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            rclpy = acquire_ros2_context()
            has_context = True
            node = rclpy.create_node("nuwax_amcl_initialpose_publisher")
            pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            msg = PoseWithCovarianceStamped()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = self.config.frames.map_frame
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891945200942
            # 等待订阅者就绪后发布
            time.sleep(0.5)
            pub.publish(msg)
            LOGGER.debug("已发布 identity initialpose 到 /initialpose。")
        except Exception as exc:
            LOGGER.warning("发布 initialpose fallback 失败：%s", exc)
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if has_context:
                release_ros2_context()

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
        self._pending_localization_callbacks.append(callback)
        # 外部栈位姿由 ExternalGeometryDebugObserver 在 robot.start() 之后启动
        # （见 GatewayRuntime._start_data_frame_stream_sources），此处不能提前启动 observer：
        # 它会设置 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp，导致 Unitree SDK ChannelFactory 初始化失败。
        if self._uses_external_geometry_stack():
            return 0
        if self._remote_provider is not None and self._remote_provider.is_connected:
            self._remote_provider.subscribe_pose(callback)
        return 0

    def unsubscribe_localization_updates(self, subscription_id: Optional[int]) -> None:
        pass  # Rust bridge 管理订阅生命周期，无需手动注销

    def subscribe_exploration_updates(self, callback: Callable[[ExplorationState], None]) -> int:
        """订阅平台探索后台 tick 产生的状态快照。"""

        if not callable(callback):
            raise ValueError("exploration update callback must be callable")
        with self._exploration_loop_lock:
            subscription_id = self._next_exploration_subscription_id
            self._next_exploration_subscription_id += 1
            self._exploration_update_subscribers[subscription_id] = callback
            return subscription_id

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        with self._exploration_loop_lock:
            return self._exploration_update_subscribers.pop(int(subscription_id), None) is not None

    def start_platform_exploration(self, request) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        if not self._uses_external_geometry_stack():
            self._record_runtime_diagnostic("removed_internal_exploration_stack", self._removed_internal_stack_message())
            return False
        self._ensure_external_geometry_observers_started()
        request = self._attach_live_mapping_diagnostics_to_request(request)
        old_thread = None
        with self._exploration_loop_lock:
            self._exploration_loop_stop_event.set()
            self._exploration_loop_generation += 1
            old_thread = self._exploration_loop_thread
        if old_thread is not None and old_thread.is_alive() and old_thread is not threading.current_thread():
            old_thread.join(timeout=max(0.2, self._exploration_loop_interval_sec() * 2.0))
        self._cancel_platform_exploration_navigation()
        accepted = self._start_external_script_exploration(request)
        if not accepted:
            return False
        self._notify_exploration_update(self.get_platform_exploration_state())
        return True

    def stop_platform_exploration(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self._stop_platform_exploration_loop(cancel_navigation=True, notify=True)

    def sync_platform_exploration_after_task(self) -> bool:
        """探索任务收尾：自然完成时仅同步状态，不取消 Nav2、不杀 go2.sh。"""

        if not self._is_geometry_profile_enabled():
            return False
        with self._exploration_loop_lock:
            if self._external_exploration_terminal_state is not None:
                self._notify_exploration_update(self.get_platform_exploration_state())
                return True
        return self._stop_platform_exploration_loop(cancel_navigation=False, notify=True)

    def report_platform_exploration_hit_stop(self, reason: str) -> None:
        if not self._is_geometry_profile_enabled():
            return
        self._finish_external_script_exploration(
            status=ExplorationStatus.SUCCEEDED,
            reason="hit_stop",
            message=str(reason or "").strip() or "探索命中停机。",
            notify_compat=True,
            kill_geometry_process=False,
        )
        self._notify_exploration_update(self.get_platform_exploration_state())

    def _stop_platform_exploration_loop(self, *, cancel_navigation: bool, notify: bool) -> bool:
        thread = None
        with self._exploration_loop_lock:
            if self._external_exploration_terminal_state is not None:
                if notify:
                    self._notify_exploration_update(self.get_platform_exploration_state())
                return True
            self._exploration_loop_stop_event.set()
            self._exploration_loop_generation += 1
            thread = self._exploration_loop_thread
        stopped = self._stop_external_script_exploration(
            cancel_navigation=cancel_navigation,
            notify_compat=cancel_navigation,
        )
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.2, self._exploration_loop_interval_sec() * 2.0))
        if notify:
            self._notify_exploration_update(self.get_platform_exploration_state())
        return stopped

    @staticmethod
    def _remove_rtabmap_database_files(db_path: Path) -> None:
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
                LOGGER.debug("删除旧 RTAB-Map 数据库文件失败：%s", candidate, exc_info=True)

    def _start_external_script_exploration(self, request: ExploreAreaRequest) -> bool:
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        command = self._build_external_exploration_command(request)
        db_path = self._extract_external_exploration_database_path(command)
        if db_path:
            self._remove_rtabmap_database_files(Path(str(db_path)))
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
            self._exploration_tick_count = 0
            self._exploration_last_tick_at_iso = None
            self._exploration_last_tick_monotonic = None
            self._exploration_last_tick_source = "external_script_start"
        self._on_external_geometry_session_mode_changed("explore")
        if current_process is not None:
            self._managed_processes.pop(process_name, None)
            current_process.stop()
            time.sleep(3.0)
        self._map_stream_tracker = {}
        with self._observer_occupancy_lock:
            self._observer_occupancy_snapshot = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=True)
        process = self._managed_processes.get(process_name)
        if process is not None:
            with self._exploration_loop_lock:
                self._external_exploration_log_offset = _capture_log_file_offset(process.log_path)
        if process is None or not process.is_running():
            self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_explore_start_failed",
                message="GO2_RTABNAV2 原生探索脚本启动失败。",
                notify_compat=False,
                process_status=process.status() if process is not None else None,
                kill_geometry_process=True,
            )
            return False
        self._record_runtime_diagnostic(
            "external_go2_exploration_started",
            "explore_area 已委托 GO2_RTABNAV2 脚本执行：%s" % process.command_display(),
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
            message="GO2_RTABNAV2 原生探索任务已取消；RTAB-Map/Nav2 仍保持运行，请使用 stop_mapping 结束几何栈。",
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

    def start_external_mapping(self, *, map_name: Optional[str] = None) -> Dict[str, object]:
        """启动 go2.sh --mapping，供遥控/手动建图。"""

        if not self._is_geometry_profile_enabled() or not self._uses_external_geometry_stack():
            raise RuntimeError(self._removed_internal_stack_message())
        self._ensure_external_geometry_observers_started()
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        command = self._build_external_mapping_command(map_name=map_name)
        db_path = self._extract_external_exploration_database_path(command)
        if db_path:
            self._remove_rtabmap_database_files(Path(str(db_path)))
        with self._exploration_loop_lock:
            self._external_geometry_session_mode = "mapping"
            self._external_exploration_terminal_state = None
            self._external_exploration_request = None
            self._external_exploration_started_at_monotonic = time.monotonic()
            self._external_exploration_started_at_iso = _utc_now_iso()
            self._external_exploration_deadline_monotonic = None
            self._external_exploration_database_path = db_path
        self._on_external_geometry_session_mode_changed("mapping")
        if current_process is not None:
            self._managed_processes.pop(process_name, None)
            current_process.stop(
                timeout_sec=12.0,
                graceful_launch_shutdown=True,
                graceful_timeout_sec=15.0,
            )
            time.sleep(3.0)
        self._map_stream_tracker = {}
        with self._observer_occupancy_lock:
            self._observer_occupancy_snapshot = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=True)
        process = self._managed_processes.get(process_name)
        if process is None or not process.is_running():
            with self._exploration_loop_lock:
                self._external_geometry_session_mode = None
            raise RuntimeError("GO2_RTABNAV2 手动建图脚本启动失败。")
        self._record_runtime_diagnostic(
            "external_go2_mapping_started",
            "start_mapping 已委托 GO2_RTABNAV2 脚本执行：%s" % process.command_display(),
        )
        return {
            "mapping_launch_mode": "mapping",
            "mapping_script": str(self._external_script_path()),
            "mapping_command": command,
            "mapping_database_path": db_path,
            "mapping_version_id": self.get_exploration_pinned_version_id(),
            "geometry_stack_running": True,
        }

    def get_geometry_stack_status(self) -> Dict[str, object]:
        """返回 go2.sh 几何栈运行态，供 Agent 区分工作区 mapping 与真实 SLAM 进程。"""

        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        running = bool(process is not None and process.is_running())
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
                "自主探索已结束，RTAB-Map/Nav2 仍在运行。"
                "可继续手动导航/遥控，完成后 save_active_map_version，再 stop_mapping。"
            )
        elif launch_mode == "explore" and has_active_exploration:
            guidance = (
                "自主探索进行中；用 get_task_status 查看 frontier_count 与 message，"
                "不要 relocalize_active_map，也不要因 localization_ready=false 误判卡住。"
                "用户要求结束探索时请调用 stop_exploration。"
            )
        return {
            "geometry_stack_running": running,
            "launch_mode": launch_mode,
            "session_mode": session_mode,
            "process_running": running,
            "process_status": process_status,
            "mapping_database_path": mapping_database_path,
            "mapping_version_id": mapping_version_id,
            "exploration_active": has_active_exploration,
            "exploration_terminal": terminal_state is not None,
            "teleop_mapping_api": "start_mapping",
            "autonomous_explore_api": "explore_area",
            "stop_exploration_api": "stop_exploration",
            "stop_geometry_stack_api": "stop_mapping",
            "agent_guidance": guidance,
        }

    def stop_mapping(self, *, reason: str = "stop_mapping") -> Dict[str, object]:
        """统一停止 go2.sh 几何栈（探索/遥控建图/定位均由该入口结束）。"""

        with self._exploration_loop_lock:
            has_request = self._external_exploration_request is not None
            terminal_state = self._external_exploration_terminal_state
            session_mode = self._external_geometry_session_mode
        process_was_running = self._external_geometry_stack_is_running()
        exploration_state: Optional[ExplorationState] = None
        if has_request:
            self._cancel_platform_exploration_navigation()
            exploration_state = self._finish_external_script_exploration(
                status=ExplorationStatus.CANCELLED,
                reason=str(reason or "stop_mapping"),
                message="GO2_RTABNAV2 几何栈已通过 stop_mapping 停止。",
                notify_compat=True,
                kill_geometry_process=True,
            )
        else:
            process_status = self._kill_external_geometry_process()
            if terminal_state is not None and not process_was_running:
                exploration_state = terminal_state.model_copy(deep=True)
            elif terminal_state is not None:
                metadata = dict(terminal_state.metadata or {})
                metadata["geometry_stack_running"] = False
                metadata["geometry_stack_stopped_via"] = str(reason or "stop_mapping")
                exploration_state = terminal_state.model_copy(
                    update={"metadata": metadata},
                    deep=True,
                )
                with self._exploration_loop_lock:
                    self._external_exploration_terminal_state = exploration_state
            if exploration_state is not None:
                self._notify_exploration_update(exploration_state)
        stopped = not self._external_geometry_stack_is_running()
        if not stopped and process_was_running:
            raise RuntimeError("GO2_RTABNAV2 几何栈停止失败，外部进程仍在运行。")
        self._on_external_geometry_session_mode_changed(None)
        self._record_runtime_diagnostic(
            "external_go2_mapping_stopped",
            "stop_mapping 已停止 GO2_RTABNAV2 几何栈 reason=%s session_mode=%s" % (reason, session_mode),
        )
        with self._exploration_loop_lock:
            mapping_database_path = self._external_exploration_database_path
        return {
            "stopped": stopped,
            "reason": str(reason or "stop_mapping"),
            "session_mode": session_mode,
            "geometry_stack_running": False,
            "exploration_state": exploration_state.model_dump(mode="json") if exploration_state is not None else None,
            "mapping_version_id": self.get_exploration_pinned_version_id(),
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

    def _exploration_loop_interval_sec(self) -> float:
        return max(0.02, float(getattr(self._platform_exploration_config, "status_poll_interval_sec", 0.2) or 0.2))

    def _ensure_platform_exploration_loop_locked(self) -> None:
        if (
            self._exploration_loop_thread is not None
            and self._exploration_loop_thread.is_alive()
            and self._exploration_loop_thread_generation == self._exploration_loop_generation
        ):
            return
        if not self._platform_exploration_runtime.is_running:
            return
        generation = self._exploration_loop_generation
        self._exploration_loop_thread_generation = generation
        self._exploration_loop_thread = threading.Thread(
            target=self._platform_exploration_loop,
            args=(generation,),
            name="nuwax_platform_exploration_loop",
            daemon=True,
        )
        self._exploration_loop_thread.start()

    def _platform_exploration_loop(self, generation: int) -> None:
        while not self._exploration_loop_stop_event.is_set():
            state_snapshot = None
            terminal = False
            try:
                with self._exploration_loop_lock:
                    if generation != self._exploration_loop_generation:
                        break
                    if not self._platform_exploration_runtime.is_running:
                        break
                    self._record_exploration_tick_locked(tick_source="exploration_loop")
                self.sync_platform_exploration(tick_source="exploration_loop")
                state_snapshot = self.get_platform_exploration_state()
                terminal = getattr(state_snapshot.status, "value", state_snapshot.status) in {
                    ExplorationStatus.SUCCEEDED.value,
                    ExplorationStatus.FAILED.value,
                    ExplorationStatus.CANCELLED.value,
                }
            except Exception as exc:
                self._record_runtime_diagnostic("exploration_loop", exc)
                try:
                    self._platform_exploration_runtime.fail_current(
                        reason="平台探索后台循环失败：%s" % exc,
                        failure_code="exploration_loop_error",
                        waiting_for="exploration_loop",
                        metadata=self._exploration_loop_metadata(tick_source="exploration_loop_error"),
                    )
                    state_snapshot = self.get_platform_exploration_state()
                except Exception:
                    state_snapshot = None
                terminal = True
            if state_snapshot is not None:
                self._notify_exploration_update(state_snapshot)
            if terminal:
                self._exploration_loop_stop_event.set()
                break
            if self._exploration_loop_stop_event.wait(self._exploration_loop_interval_sec()):
                break
        with self._exploration_loop_lock:
            if (
                self._exploration_loop_thread is threading.current_thread()
                and generation == self._exploration_loop_thread_generation
            ):
                self._exploration_loop_thread = None

    def _record_exploration_tick_locked(self, *, tick_source: str) -> None:
        self._exploration_tick_count += 1
        self._exploration_last_tick_at_iso = _utc_now_iso()
        self._exploration_last_tick_monotonic = time.monotonic()
        self._exploration_last_tick_source = tick_source

    def _exploration_loop_metadata(self, *, tick_source: str) -> Dict[str, object]:
        with self._exploration_loop_lock:
            thread = self._exploration_loop_thread
            thread_alive = bool(thread is not None and thread.is_alive())
            last_tick_source = self._exploration_last_tick_source or tick_source
            return {
                "exploration_tick_source": tick_source,
                "exploration_last_tick_source": last_tick_source,
                "exploration_tick_count": self._exploration_tick_count,
                "exploration_last_tick_at": self._exploration_last_tick_at_iso,
                "exploration_last_tick_monotonic": self._exploration_last_tick_monotonic,
                "exploration_loop_interval_sec": self._exploration_loop_interval_sec(),
                "exploration_loop_generation": self._exploration_loop_generation,
                "exploration_loop": {
                    "mode": "autonomous",
                    "thread_alive": thread_alive,
                    "stop_requested": self._exploration_loop_stop_event.is_set(),
                    "generation": self._exploration_loop_generation,
                    "tick_count": self._exploration_tick_count,
                    "last_tick_at": self._exploration_last_tick_at_iso,
                    "last_tick_source": last_tick_source,
                    "interval_sec": self._exploration_loop_interval_sec(),
                },
            }

    def _notify_exploration_update(self, state: ExplorationState) -> None:
        with self._exploration_loop_lock:
            subscribers = tuple(self._exploration_update_subscribers.values())
        for callback in subscribers:
            try:
                callback(state.model_copy(deep=True))
            except Exception:
                pass

    def sync_platform_exploration(self, *, tick_source: str = "manual") -> None:
        if not self._is_geometry_profile_enabled():
            return
        if not self._platform_exploration_runtime.is_running:
            return
        current_pose = self._read_exploration_pose()
        snapshot = self.get_map_snapshot()
        occupancy_grid = self._clone_contract(snapshot.occupancy_grid) if snapshot is not None else None
        cost_map = self._clone_contract(snapshot.cost_map) if snapshot is not None else None
        if occupancy_grid is None:
            occupancy_grid = self._read_live_map_layer("occupancy_grid")
        if cost_map is None:
            cost_map = self._read_live_map_layer("cost_map")
        runtime_metadata = self._build_exploration_runtime_metadata(
            snapshot=snapshot,
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        navigation_backend = self._resolve_navigation_backend()
        navigation_backend_name = getattr(navigation_backend, "provider_name", None) if navigation_backend is not None else None
        runtime_metadata.update({
            **self._exploration_loop_metadata(tick_source=tick_source),
            "navigation_backend_name": navigation_backend_name,
        })
        waiting_for = runtime_metadata.get("waiting_for")
        current_goal_before_tick = self._platform_exploration_runtime.get_current_navigation_goal()
        if waiting_for:
            preserve_current_goal = False
            navigation_state = None
            navigation_state_unavailable = False
            if current_goal_before_tick is not None and navigation_backend is not None:
                try:
                    navigation_state = navigation_backend.get_navigation_state()
                except Exception:
                    navigation_state_unavailable = True
            if navigation_state_unavailable:
                preserve_current_goal = True
            if (
                navigation_state is not None
                and current_goal_before_tick is not None
                and navigation_state.current_goal_id == current_goal_before_tick.goal_id
            ):
                if bool(navigation_state.goal_reached) or navigation_state.status == NavigationStatus.SUCCEEDED:
                    self._platform_exploration_runtime.mark_navigation_arrived()
                    return
                if navigation_state.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
                    self._platform_exploration_runtime.reject_current_target(
                        navigation_state.message or "探索前沿导航失败。"
                    )
                    return
                preserve_current_goal = navigation_state.status in {
                    NavigationStatus.ACCEPTED,
                    NavigationStatus.PLANNING,
                    NavigationStatus.RUNNING,
                }
            if preserve_current_goal:
                transient_waiting_detail = runtime_metadata.get("waiting_detail")
                transient_failure_code = runtime_metadata.get("failure_code")
                runtime_metadata = {
                    key: value
                    for key, value in runtime_metadata.items()
                    if key not in {"waiting_for", "waiting_detail", "failure_code"}
                }
                runtime_metadata["transient_waiting_for"] = str(waiting_for)
                if transient_waiting_detail:
                    runtime_metadata["transient_waiting_detail"] = str(transient_waiting_detail)
                if transient_failure_code:
                    runtime_metadata["transient_failure_code"] = str(transient_failure_code)
                frontier_freshness = runtime_metadata.get("frontier_freshness")
                if isinstance(frontier_freshness, dict):
                    sanitized_freshness = {
                        key: value
                        for key, value in dict(frontier_freshness).items()
                        if key != "stale_reason"
                    }
                    runtime_metadata["frontier_freshness"] = {
                        **sanitized_freshness,
                        "fresh": True,
                        "transient_waiting_for": str(waiting_for),
                    }
            else:
                try:
                    self._cancel_platform_exploration_navigation(current_goal=current_goal_before_tick)
                except Exception:
                    return
                self._platform_exploration_runtime.block_on_prerequisite(
                    waiting_for=str(waiting_for),
                    message=str(runtime_metadata.get("waiting_detail") or "探索前置条件未就绪。"),
                    failure_code=str(runtime_metadata.get("failure_code") or "exploration_prerequisite_unavailable"),
                    metadata=runtime_metadata,
                )
                return
        if current_goal_before_tick is not None and self._frontier_goal_is_stale(current_goal_before_tick, runtime_metadata):
            try:
                self._cancel_platform_exploration_navigation(current_goal=current_goal_before_tick)
            except Exception:
                return
            self._platform_exploration_runtime.mark_current_frontier_stale(reason="live_map_costmap_or_pose_revision_changed")
            return
        self._platform_exploration_runtime.tick(
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            runtime_metadata=runtime_metadata,
        )
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        if current_goal is None:
            return
        if navigation_backend is None:
            self._platform_exploration_runtime.block_on_prerequisite(
                waiting_for="navigation_backend",
                message="探索 frontier 需要导航后端执行，当前未绑定可用导航后端。",
                failure_code="navigation_backend_unavailable",
                metadata=runtime_metadata,
            )
            return
        availability_checker = getattr(navigation_backend, "is_navigation_available", None)
        try:
            navigation_available = bool(availability_checker()) if callable(availability_checker) else bool(navigation_backend.is_available())
        except Exception:
            navigation_available = False
        if not navigation_available:
            try:
                state = navigation_backend.get_navigation_state()
            except Exception:
                return
            if state.current_goal_id == current_goal.goal_id:
                if bool(state.goal_reached) or state.status == NavigationStatus.SUCCEEDED:
                    self._platform_exploration_runtime.mark_navigation_arrived()
                    return
                if state.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
                    self._platform_exploration_runtime.reject_current_target(
                        state.message or "探索前沿导航失败。"
                    )
                    return
                if state.status in {
                    NavigationStatus.ACCEPTED,
                    NavigationStatus.PLANNING,
                    NavigationStatus.RUNNING,
                }:
                    return
            metadata = dict(getattr(state, "metadata", {}) or {})
            readiness = metadata.get("nav2_readiness") if isinstance(metadata.get("nav2_readiness"), dict) else {}
            waiting = self._normalize_exploration_waiting_for(
                metadata.get("waiting_for") or readiness.get("waiting_for") or "navigation_backend"
            )
            self._platform_exploration_runtime.block_on_prerequisite(
                waiting_for=waiting,
                message=state.message or "探索 frontier 导航后端暂不可用。",
                failure_code=str(metadata.get("failure_reason") or "navigation_backend_unavailable"),
                metadata={**runtime_metadata, "navigation_state": self._clone_contract(state)},
            )
            return
        try:
            navigation_state = navigation_backend.get_navigation_state()
        except Exception:
            return
        if navigation_state.current_goal_id != current_goal.goal_id:
            if navigation_state.current_goal_id and navigation_state.status not in {
                NavigationStatus.IDLE,
                NavigationStatus.SUCCEEDED,
                NavigationStatus.FAILED,
                NavigationStatus.CANCELLED,
            }:
                try:
                    navigation_backend.cancel_goal()
                except Exception:
                    return
            target_pose = current_goal.target_pose
            if target_pose is not None:
                LOGGER.info(
                    "平台探索下发 frontier 导航目标 backend=%s previous_goal_id=%s previous_status=%s goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s metadata=%s",
                    navigation_backend_name,
                    navigation_state.current_goal_id,
                    getattr(navigation_state.status, "value", navigation_state.status),
                    current_goal.goal_id,
                    float(target_pose.position.x),
                    float(target_pose.position.y),
                    float(target_pose.position.z),
                    target_pose.frame_id,
                    dict(current_goal.metadata),
                )
            accepted = navigation_backend.set_goal(current_goal)
            if not accepted:
                try:
                    state = navigation_backend.get_navigation_state()
                except Exception:
                    return
                metadata = dict(getattr(state, "metadata", {}) or {})
                readiness = metadata.get("nav2_readiness") if isinstance(metadata.get("nav2_readiness"), dict) else {}
                waiting = self._normalize_exploration_waiting_for(
                    metadata.get("waiting_for") or readiness.get("waiting_for") or "navigation_backend"
                )
                self._platform_exploration_runtime.block_on_prerequisite(
                    waiting_for=waiting,
                    message=state.message or "%s 后端拒绝探索前沿目标。" % navigation_backend.provider_name,
                    failure_code=str(metadata.get("failure_reason") or "navigation_goal_rejected"),
                    metadata={**runtime_metadata, "navigation_state": self._clone_contract(state)},
                )
            return
        if navigation_state.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
            self._platform_exploration_runtime.reject_current_target(
                navigation_state.message or "探索前沿导航失败。"
            )
            return
        if bool(navigation_backend.is_goal_reached() or navigation_state.goal_reached):
            self._platform_exploration_runtime.mark_navigation_arrived()

    def get_platform_exploration_state(self) -> ExplorationState:
        if not self._is_geometry_profile_enabled():
            return self._platform_exploration_runtime.get_exploration_state()
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            terminal_state = self._external_exploration_terminal_state
        if request is not None or terminal_state is not None:
            return self._get_external_script_exploration_state()
        with self._exploration_loop_lock:
            if self._platform_exploration_runtime.is_running:
                self._ensure_platform_exploration_loop_locked()
        state = self._platform_exploration_runtime.get_exploration_state()
        state = self._augment_exploration_state_with_mapping_diagnostics(state)
        frontier_metadata: Dict[str, object] = {}
        latest_frontiers: List[Dict[str, object]] = []
        platform_frontiers = self._platform_exploration_runtime.get_latest_frontiers() if hasattr(self._platform_exploration_runtime, "get_latest_frontiers") else []
        if platform_frontiers:
            latest_frontiers = list(platform_frontiers)
            current_frontier = latest_frontiers[0]
            frontier_source = (
                current_frontier.get("metadata", {}).get("source")
                if isinstance(current_frontier.get("metadata"), dict)
                else None
            )
            frontier_metadata = {
                "latest_frontiers": latest_frontiers,
                "current_frontier": current_frontier,
                "frontier_source": frontier_source or "platform_frontier_explorer",
            }
        external_frontier_metadata, external_frontier_count = self._external_debug_frontier_metadata()
        if external_frontier_metadata:
            frontier_metadata = external_frontier_metadata
            latest_frontiers = list(external_frontier_metadata.get("latest_frontiers") or [])
        metadata = {
            **dict(getattr(state, "metadata", {}) or {}),
            **frontier_metadata,
            **self._exploration_loop_metadata(tick_source="get_exploration_state"),
        }
        update: Dict[str, object] = {"metadata": metadata}
        if external_frontier_count is not None:
            update["frontier_count"] = external_frontier_count
        elif latest_frontiers:
            update["frontier_count"] = len(latest_frontiers)
        return state.model_copy(update=update, deep=True)

    def _external_debug_frontier_metadata(self) -> tuple[Dict[str, object], Optional[int]]:
        with self._exploration_loop_lock:
            if self._external_geometry_session_mode != "explore":
                return {}, None
            payload = self._clone_metadata(self._external_debug_frontier_payload or {})
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

    @staticmethod
    def _metadata_indicates_exploration_done(metadata: Dict[str, object]) -> bool:
        if bool(metadata.get("exploration_terminal")) or bool(metadata.get("terminal")):
            return True
        status_text = str(metadata.get("status_text") or "").upper()
        if "DONE (EXPLORATION FINISHED)" in status_text:
            return True
        observer_meta = metadata.get("external_frontier_debug_observer")
        if isinstance(observer_meta, dict):
            observer_payload = observer_meta.get("metadata") if isinstance(observer_meta.get("metadata"), dict) else {}
            if bool(observer_payload.get("exploration_terminal")):
                return True
            observer_status = str(observer_payload.get("status_text") or "").upper()
            if "DONE (EXPLORATION FINISHED)" in observer_status:
                return True
        return False

    def _external_exploration_terminal_from_frontier_observer(self) -> Optional[ExplorationState]:
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            payload = self._clone_metadata(self._external_debug_frontier_payload or {})
        if request is None or not payload:
            return None
        payload_metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        frontier_metadata, _ = self._external_debug_frontier_metadata()
        if not self._metadata_indicates_exploration_done({**frontier_metadata, **payload_metadata}):
            return None
        terminal_status = str(payload_metadata.get("terminal_status") or "succeeded").lower()
        status = ExplorationStatus.SUCCEEDED if terminal_status == "succeeded" else ExplorationStatus.FAILED
        status_text = str(payload_metadata.get("status_text") or "").strip()
        message = "GO2_RTABNAV2 原生探索已完成。"
        if status_text:
            message = "GO2_RTABNAV2 原生探索已完成：%s" % status_text
        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        return self._finish_external_script_exploration(
            status=status,
            reason="external_explorer_frontier_done",
            message=message,
            notify_compat=False,
            process_status=process_status,
            kill_geometry_process=False,
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
        message = explorer_message or "GO2_RTABNAV2 原生 frontier 探索运行中。"
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
                message="GO2_RTABNAV2 原生探索已达到最大时长，任务已结束。",
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
                    message="GO2_RTABNAV2 原生探索脚本已正常退出。",
                    notify_compat=False,
                    process_status=process_status,
                    kill_geometry_process=False,
                )
                return self._augment_exploration_state_with_mapping_diagnostics(state)
            state = self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_process_exited",
                message="GO2_RTABNAV2 原生探索脚本已退出，exit_code=%s。" % exit_code,
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
        rtabmap_readiness = diagnostics.get("rtabmap_readiness") if isinstance(diagnostics.get("rtabmap_readiness"), dict) else {}
        nav2_status = runtime_status.get("nav2") if isinstance(runtime_status.get("nav2"), dict) else self._build_nav2_status()
        waiting_for = None
        message = "GO2_RTABNAV2 原生 frontier 探索运行中。"
        exploration_signals_active = bool(
            explorer_message is not None
            or (frontier_count is not None and frontier_count > 0)
            or bool((frontier_metadata or {}).get("plan_fresh"))
        )
        if exploration_signals_active:
            message = explorer_message or message
        elif diagnostics.get("waiting_for"):
            waiting_for = str(diagnostics.get("waiting_for"))
            message = str(diagnostics.get("message") or "等待 GO2_RTABNAV2 探索输入链路 ready。")
        elif not bool(rtabmap_readiness.get("localization_ready")):
            waiting_for = str(rtabmap_readiness.get("waiting_for") or "external_slam_nav2")
            message = str(rtabmap_readiness.get("message") or "等待 GO2_RTABNAV2 SLAM/TF ready。")
        elif not bool(nav2_status.get("ready")):
            waiting_for = self._normalize_exploration_waiting_for(nav2_status.get("waiting_for") or "nav2_action")
            message = str(nav2_status.get("message") or "等待 GO2_RTABNAV2 Nav2 ready。")
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
        metadata["rtabmap_readiness"] = self._clone_metadata(rtabmap_readiness)
        metadata["nav2_readiness"] = self._clone_metadata(nav2_status)
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
        seen: set[str] = set()
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
                "message": "GO2_RTABNAV2 原生探索已完成。",
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

    def _build_exploration_runtime_metadata(self, *, snapshot, current_pose, occupancy_grid, cost_map) -> Dict[str, object]:
        live_diagnostics = self.get_live_mapping_diagnostics()
        rtabmap_readiness = live_diagnostics.get("rtabmap_readiness") if isinstance(live_diagnostics.get("rtabmap_readiness"), dict) else {}
        nav2_status = self._build_nav2_status()
        navigation_backend_policy = self._navigation_backend_policy()
        navigation_backend = self._resolve_navigation_backend()
        navigation_backend_name = getattr(navigation_backend, "provider_name", None) if navigation_backend is not None else None
        navigation_backend_ready = False
        navigation_backend_message = str(
            navigation_backend_policy.get("message")
            or "探索 frontier 需要导航后端执行，当前未绑定可用导航后端。"
        )
        navigation_backend_waiting_for = "navigation_backend"
        if navigation_backend is not None:
            availability_checker = getattr(navigation_backend, "is_navigation_available", None)
            try:
                navigation_backend_ready = bool(availability_checker()) if callable(availability_checker) else bool(navigation_backend.is_available())
            except Exception as exc:
                navigation_backend_ready = False
                navigation_backend_message = "导航后端可用性检查失败：%s" % exc
            else:
                if navigation_backend_ready:
                    navigation_backend_message = "%s 导航后端可用于探索 frontier 执行。" % navigation_backend_name
                    navigation_backend_waiting_for = None
                elif navigation_backend_name == "nav2_action_navigation":
                    navigation_backend_waiting_for = self._normalize_exploration_waiting_for(nav2_status.get("waiting_for") or "nav2_lifecycle")
                    navigation_backend_message = str(nav2_status.get("message") or "等待 Nav2 lifecycle/action/costmap ready。")
        map_stream = rtabmap_readiness.get("map_stream") if isinstance(rtabmap_readiness.get("map_stream"), dict) else {}
        map_revision = self._exploration_map_revision(snapshot, occupancy_grid, map_stream)
        costmap_revision = self._exploration_costmap_revision(snapshot, cost_map, map_stream, nav2_status)
        pose_revision = self._exploration_pose_revision(current_pose)
        metadata: Dict[str, object] = {
            "map_revision": map_revision,
            "costmap_revision": costmap_revision,
            "pose_revision": pose_revision,
            "map_stream": self._clone_metadata(map_stream),
            "rtabmap_readiness": self._clone_metadata(rtabmap_readiness),
            "nav2_readiness": self._clone_metadata(nav2_status),
            "navigation_backend": {
                "provider_name": navigation_backend_name,
                "ready": navigation_backend_ready,
                "waiting_for": navigation_backend_waiting_for,
                "message": navigation_backend_message,
                "policy": self._clone_metadata(navigation_backend_policy),
            },
            "navigation_backend_policy": self._clone_metadata(navigation_backend_policy),
            "live_map_streaming_ready": bool(rtabmap_readiness.get("streaming_ready")),
            "nav2_ready": bool(nav2_status.get("ready")),
            "navigation_backend_ready": navigation_backend_ready,
            "frontier_freshness": {
                "fresh": True,
                "map_revision": map_revision,
                "costmap_revision": costmap_revision,
                "pose_revision": pose_revision,
                "checked_at": _utc_now_iso(),
            },
        }
        waiting_for = None
        message = None
        failure_code = None
        if occupancy_grid is None:
            waiting_for = "map"
            message = "等待外部 SLAM/Nav2 /map_obstacles 首帧。"
            failure_code = "map_unavailable"
        elif not bool(rtabmap_readiness.get("streaming_ready")):
            waiting_for = "live_map_streaming"
            message = str(rtabmap_readiness.get("message") or live_diagnostics.get("message") or "等待 live map streaming。")
            failure_code = "live_map_streaming_unavailable"
        elif not bool(rtabmap_readiness.get("tf_chain", {}).get("ready")):
            waiting_for = "tf"
            message = "等待外部 SLAM/Nav2 所需 TF 链 ready。"
            failure_code = "tf_unavailable"
        elif current_pose is None:
            waiting_for = "tf"
            message = "等待定位位姿和 TF 可用。"
            failure_code = "pose_unavailable"
        elif cost_map is None:
            waiting_for = "costmap"
            message = "等待最新 costmap。"
            failure_code = "costmap_unavailable"
        elif not navigation_backend_ready:
            waiting_for = self._normalize_exploration_waiting_for(navigation_backend_waiting_for or "navigation_backend")
            message = navigation_backend_message
            failure_code = "navigation_backend_not_ready"
        if waiting_for:
            metadata.update({"waiting_for": waiting_for, "waiting_detail": message, "failure_code": failure_code})
            metadata["frontier_freshness"] = {**dict(metadata["frontier_freshness"]), "fresh": False, "stale_reason": waiting_for}
        return metadata

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

    def _frontier_goal_is_stale(self, goal: NavigationGoal, metadata: Dict[str, object]) -> bool:
        goal_metadata = dict(getattr(goal, "metadata", {}) or {})
        for key in ("map_revision", "costmap_revision"):
            goal_revision = str(goal_metadata.get(key) or "").strip()
            current_revision = str(metadata.get(key) or "").strip()
            if goal_revision and current_revision and goal_revision != current_revision:
                return True
        return False

    def _exploration_map_revision(self, snapshot, occupancy_grid, map_stream: Dict[str, object]) -> str:
        revision = map_stream.get("content_revision")
        if revision is not None:
            return "map_stream:%s" % revision
        if snapshot is not None:
            return "%s:%s" % (getattr(snapshot, "version_id", "map"), getattr(snapshot, "revision", ""))
        timestamp = getattr(occupancy_grid, "timestamp", None)
        return "map:%s" % (timestamp.isoformat() if hasattr(timestamp, "isoformat") else "unknown")

    def _exploration_costmap_revision(self, snapshot, cost_map, map_stream: Dict[str, object], nav2_status: Dict[str, object]) -> str:
        revision = map_stream.get("costmap_revision")
        if revision is not None:
            return "costmap_stream:%s" % revision
        costmaps = nav2_status.get("costmaps") if isinstance(nav2_status.get("costmaps"), dict) else {}
        global_costmap = costmaps.get("global") if isinstance(costmaps.get("global"), dict) else {}
        count = global_costmap.get("message_count")
        if count is not None:
            return "nav2_global_costmap:%s" % count
        timestamp = getattr(cost_map, "timestamp", None)
        if hasattr(timestamp, "isoformat"):
            return "costmap:%s" % timestamp.isoformat()
        if snapshot is not None:
            return "%s:costmap:%s" % (getattr(snapshot, "version_id", "map"), getattr(snapshot, "revision", ""))
        return "costmap:unknown"

    def _exploration_pose_revision(self, pose) -> str:
        if pose is None:
            return "pose:missing"
        timestamp = getattr(pose, "timestamp", None)
        if hasattr(timestamp, "isoformat"):
            return "pose:%s" % timestamp.isoformat()
        position = getattr(pose, "position", None)
        if position is None:
            return "pose:unknown"
        return "pose:%.3f:%.3f:%.3f" % (float(position.x), float(position.y), float(position.z))


    def _notify_compat_exploration_stopped(self) -> None:
        backend = self._resolve_backend(ExplorationDataPlane, ExplorationProvider)
        if backend is None:
            return
        if backend is self._platform_exploration_runtime:
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
            canceller()
        except Exception:
            pass

    def _cancel_platform_exploration_navigation(self, *, current_goal=None) -> None:
        goal = current_goal or self._platform_exploration_runtime.get_current_navigation_goal()
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
            navigation_backend.cancel_goal()

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
    （外部 SLAM/Nav2 侧车和平台数据面），但定位状态由平台状态机管理，
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
                        "raw_odom_input": self.runtime._clone_metadata(self.runtime._ros_bridge_status_stub("odometry_publisher")),
                        "tf_localization_provider": self.runtime._clone_metadata(self.runtime._ros_bridge_status_stub("tf_localization_provider")),
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
            "tf_localization_provider": self.runtime._clone_metadata(self.runtime._ros_bridge_status_stub("tf_localization_provider")),
            "live_map_provider": self.runtime._clone_metadata(self.runtime._ros_bridge_status_stub("live_map_provider")),
        }

    def get_occupancy_grid(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.occupancy_grid
        # RosBridgeProvider 仅暴露 gRPC 协议方法（meta/data）作为推送侧契约；
        # 当前推送路径（subscribe_map_updates → _attach_active_map_version_to_live_map_payload）
        # 仅 seed 来自 active map version 的 semantic_map，不携带 occupancy_grid/cost_map；
        # snapshot 缺失时返回 None（push 路径在 Rust 桥当前实现下不会补齐 occupancy/cost）。
        return None

    def get_cost_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.cost_map
        # RosBridgeProvider 仅暴露 gRPC 协议方法（meta/data）作为推送侧契约；
        # 当前推送路径（subscribe_map_updates → _attach_active_map_version_to_live_map_payload）
        # 仅 seed 来自 active map version 的 semantic_map，不携带 occupancy_grid/cost_map；
        # snapshot 缺失时返回 None（push 路径在 Rust 桥当前实现下不会补齐 occupancy/cost）。
        return None

    def get_semantic_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.semantic_map
        # RosBridgeProvider 仅暴露 gRPC 协议方法（meta/data）作为推送侧契约；
        # 当前推送路径（subscribe_map_updates → _attach_active_map_version_to_live_map_payload）
        # 仅 seed 来自 active map version 的 semantic_map，不携带 occupancy_grid/cost_map；
        # snapshot 缺失时返回 None（push 路径在 Rust 桥当前实现下不会补齐 occupancy/cost）。
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
                getattr(backend, "provider_name", None),
                getattr(goal, "goal_id", None),
                float(target_pose.position.x),
                float(target_pose.position.y),
                float(target_pose.position.z),
                target_pose.frame_id,
            )
        return backend.set_goal(goal)

    def cancel_goal(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.cancel_goal()

    def subscribe_navigation_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        backend = self.runtime._resolve_navigation_backend()
        subscriber = getattr(backend, "subscribe_navigation_updates", None)
        if not callable(subscriber):
            return None
        return subscriber(callback)

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        unsubscribe = getattr(backend, "unsubscribe_navigation_updates", None)
        if not callable(unsubscribe):
            return False
        return bool(unsubscribe(subscription_id))

    def get_navigation_state(self):
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            from contracts.navigation import NavigationState, NavigationStatus

            policy = self.runtime._navigation_backend_policy()
            message = "平台几何运行时未绑定 Nav2 action provider。"
            return NavigationState(
                status=NavigationStatus.IDLE,
                message=message,
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "nav2_action_active": bool(policy.get("nav2_action_active")),
                },
            )
        return backend.get_navigation_state()

    def is_goal_reached(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.is_goal_reached()

    def start_exploration(self, request) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.start_platform_exploration(request)

    def stop_exploration(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.stop_platform_exploration()

    def sync_exploration_after_task(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.sync_platform_exploration_after_task()

    def subscribe_exploration_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_exploration_updates(callback)

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.unsubscribe_exploration_updates(subscription_id)

    def get_exploration_state(self):
        return self.runtime.get_platform_exploration_state()
