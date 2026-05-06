from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Protocol, runtime_checkable
from urllib.parse import quote

from contracts.geometry import Pose, Vector3
from contracts.map_workspace import MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType, MapVersion
from contracts.navigation import ExplorationState, ExplorationStatus, NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import MapSnapshot
from drivers.common import ExplorationDataPlane, LocalizationDataPlane, MappingDataPlane, NavigationDataPlane, RobotAssemblyBase
from providers.exploration import ExplorationProvider
from providers.localization import LocalizationProvider
from providers.maps import MapProvider
from providers.motion import MotionControl
from providers.navigation import NavigationProvider
from providers.pointcloud import PointCloudProvider
from services.geometry.config import PlatformGeometryConfig
from services.geometry.live_map_provider import Ros2LiveMapProvider
from services.geometry.nav2_action_provider import Nav2ActionNavigationProvider
from services.geometry.odometry_publisher import Ros2OdometryPublisher
from services.geometry.pointcloud_publisher import Ros2PointCloudPublisher
from services.geometry.tf_localization_provider import Ros2TfLocalizationProvider
from services.navigation.config import ExplorationConfig, NavigationBackendConfig
from services.navigation.exploration_runtime import PlatformExplorationRuntime
from services.navigation.frontier import FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner


_GEOMETRY_SIDE_CAR_RESTART_LIMIT = 2
_GEOMETRY_MONITOR_INTERVAL_SEC = 1.0
_GEOMETRY_STREAMING_WINDOW_SEC = 3.0
_NAV2_SIDECAR_MODES = {"sidecar", "managed", "launch", "internal"}
_NAV2_EXTERNAL_MODES = {"external", "external_action", "action", "client", "pending"}
_NAV2_DISABLED_MODES = {"", "disabled", "off", "none"}
_REMOVED_PLATFORM_GRID_MODES = {"platform", "platform_grid", "grid", "fallback", "platform_fallback"}
_EXTERNAL_GEOMETRY_STACKS = {"external", "go2slam_external", "external_go2slam"}


@dataclass
class _ManagedProcess:
    name: str
    command: List[str]
    log_path: Path
    cwd: Path
    process: Optional[subprocess.Popen] = None
    started_at: Optional[float] = None
    started_at_iso: Optional[str] = None
    exit_code: Optional[int] = None
    last_error: Optional[str] = None
    restart_count: int = 0

    def start(self, *, restart: bool = False) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        if restart:
            self.restart_count += 1
        with self.log_path.open("ab") as log_file:
            log_file.write(("\n[%s] start %s\n" % (_utc_now_iso(), self.command_display())).encode("utf-8"))
            self.process = subprocess.Popen(
                self.command,
                cwd=str(self.cwd),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        self.started_at = time.monotonic()
        self.started_at_iso = _utc_now_iso()
        self.exit_code = None
        self.last_error = None

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

    def stop(self, *, timeout_sec: float = 5.0) -> None:
        process = self.process
        if process is None or process.poll() is not None:
            self.poll()
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except Exception:
            try:
                process.terminate()
            except Exception:
                pass
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
            try:
                process.wait(timeout=1.0)
            except Exception:
                pass
        self.poll()

    def status(self) -> Dict[str, object]:
        self.poll()
        return {
            "name": self.name,
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
    1. 约束平台地图版本对应的 RTAB-Map / Nav2 资产目录；
    2. 管理当前激活的平台地图版本几何上下文；
    3. 在历史地图激活时，把平台版本快照重新组织为平台几何输出。
    """

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        catalog_root: str,
        robot: RobotAssemblyBase,
    ) -> None:
        self.config = config
        self.robot = robot
        self._catalog_root = Path(catalog_root).expanduser()
        self._catalog_root.mkdir(parents=True, exist_ok=True)
        self._active_map_version: Optional[MapVersion] = None
        self._last_activation_result: Dict[str, object] = {}
        self._live_map_provider = Ros2LiveMapProvider(config)
        self._tf_localization_provider = Ros2TfLocalizationProvider(config)
        self._odometry_publisher = Ros2OdometryPublisher(config, self._resolve_raw_localization_backend)
        self._point_cloud_publisher = Ros2PointCloudPublisher(config, self._resolve_point_cloud_provider)
        self._live_geometry_start_lock = threading.RLock()
        self._live_geometry_start_attempted = False
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
        self._nav2_launch_attempted = False
        self._nav2_action_provider = Nav2ActionNavigationProvider(self)
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
        if not self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时或 RTAB-Map profile 已禁用。")
            return
        try:
            self._set_runtime_state("preflight", "正在检查 ROS2、脚本、话题与 TF 配置。")
            preflight_errors = self._run_preflight_checks()
            if preflight_errors:
                message = "平台几何运行时预检失败：" + "；".join(preflight_errors)
                self._set_runtime_state("failed", message, blocked=True)
                if self.config.require_ros2:
                    raise RuntimeError(message)
                return
            if self._uses_external_geometry_stack():
                self._set_runtime_state("starting_external", "正在启动外部 Go2 SLAM/Nav2 几何脚本。")
                self._ensure_external_geometry_observers_started()
            else:
                self._set_runtime_state("starting_inputs", "正在启动点云桥、里程计 TF 与静态 TF 输入链路。")
                self._ensure_live_geometry_started()
            self._start_required_processes()
            if self._uses_external_geometry_stack():
                self._set_runtime_state("starting_external", "外部 Go2 SLAM/Nav2 几何脚本已启动，等待 /map 与 TF。")
            else:
                self._set_runtime_state("starting_rtabmap", "RTAB-Map 侧车已启动，等待 /map 首帧。")
            self._start_monitor_thread()
        except Exception as exc:
            self._record_runtime_diagnostic("geometry_start", exc)
            self._set_runtime_state("failed", "平台几何运行时启动失败：%s" % exc, blocked=True)
            self._stop_managed_processes()
            self._live_map_provider.stop()
            self._tf_localization_provider.stop()
            self._odometry_publisher.stop()
            self._point_cloud_publisher.stop()
            self._live_geometry_start_attempted = False
            with self._runtime_state_lock:
                self._runtime_started = False
            if self.config.require_ros2:
                raise

    def stop(self) -> None:
        """停止平台几何运行时。"""

        self._set_runtime_state("stopping", "正在停止平台几何侧车进程。")
        self._runtime_stop_event.set()
        self._stop_platform_exploration_loop(cancel_navigation=True, notify=True)
        thread = self._runtime_monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._stop_managed_processes()
        self._live_map_provider.stop()
        self._tf_localization_provider.stop()
        self._odometry_publisher.stop()
        self._point_cloud_publisher.stop()
        self._live_geometry_start_attempted = False
        with self._runtime_state_lock:
            self._runtime_started = False
            self._runtime_monitor_thread = None
        if self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时已停止。")
        else:
            self._set_runtime_state("disabled", "平台几何运行时或 RTAB-Map profile 已禁用。")

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

    def _rtabmap_database_path(self) -> Path:
        root = Path(str(self.config.rtabmap.database_root or "")).expanduser()
        if root.suffix == ".db":
            return root
        return root / "live_rtabmap.db"

    def _geometry_stack(self) -> str:
        return str(getattr(self.config, "stack", "") or "").strip().lower()

    def _uses_external_geometry_stack(self) -> bool:
        return self._geometry_stack() in _EXTERNAL_GEOMETRY_STACKS

    def _external_process_name(self) -> str:
        name = str(getattr(self.config.external, "process_name", "") or "").strip()
        return name or "go2slam_external"

    def _external_log_name(self) -> str:
        log_name = str(getattr(self.config.external, "log_name", "") or "").strip()
        return log_name or "go2slam_external.log"

    def _external_script_path(self) -> Path:
        return Path(str(getattr(self.config.external, "script_path", "") or "")).expanduser()

    def _external_args(self) -> List[str]:
        raw_args = getattr(self.config.external, "args", ())
        if isinstance(raw_args, str):
            return shlex.split(raw_args)
        return [str(item) for item in raw_args if str(item or "").strip()]

    def _build_external_geometry_command(self) -> List[str]:
        return [str(self._external_script_path()), *self._external_args()]

    def _external_input_bridge_enabled(self) -> bool:
        return bool(getattr(self.config.external, "input_bridge_enabled", False))

    def _external_point_cloud_bridge_enabled(self) -> bool:
        return self._external_input_bridge_enabled() or bool(
            getattr(self.config.external, "point_cloud_bridge_enabled", False)
        )

    def _run_preflight_checks(self) -> List[str]:
        errors: List[str] = []
        if self._uses_external_geometry_stack():
            return self._run_external_geometry_preflight_checks()
        launch_script = self._geometry_script("launch_ros2_stack.sh")
        if not launch_script.exists():
            errors.append("缺少 ROS2 环境启动脚本 %s" % launch_script)
        sidecar_script = self._geometry_script("start_rtabmap_sidecar.sh")
        if not sidecar_script.exists():
            errors.append("缺少 RTAB-Map sidecar 脚本 %s" % sidecar_script)
        odom_bridge_script = self._geometry_script("odom_tf_bridge.py")
        if not odom_bridge_script.exists():
            errors.append("缺少 odom TF bridge 脚本 %s" % odom_bridge_script)
        setup_script = Path(str(self.config.setup_script or "")).expanduser()
        ros_setup_available = setup_script.exists() or any(path.exists() for path in self._ros_setup_candidates())
        if self.config.require_ros2 and not setup_script.exists():
            errors.append("缺少 geometry setup script %s" % setup_script)
        if shutil.which("ros2") is None and not ros_setup_available:
            errors.append("未找到 ros2 命令、geometry setup script 或系统 ROS setup，无法启动 ROS2 sidecar")
        required_topics = {
            "odom_topic": self.config.topics.odom_topic,
            "point_cloud_topic": self.config.topics.point_cloud_topic,
            "occupancy_topic": self.config.topics.occupancy_topic,
            "tf_topic": self.config.topics.tf_topic,
            "tf_static_topic": self.config.topics.tf_static_topic,
        }
        for name, value in required_topics.items():
            if not str(value or "").strip():
                errors.append("缺少 geometry topic 配置 %s" % name)
        required_frames = {
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "imu_frame": getattr(self.config.frames, "imu_frame", ""),
        }
        for name, value in required_frames.items():
            if not str(value or "").strip():
                errors.append("缺少 geometry TF frame 配置 %s" % name)
        if not str(self.config.rtabmap.launch_package or "").strip():
            errors.append("缺少 RTAB-Map launch package 配置")
        if not str(self.config.rtabmap.launch_file or "").strip():
            errors.append("缺少 RTAB-Map launch file 配置")
        nav2_mode = self._nav2_mode()
        if nav2_mode in _NAV2_SIDECAR_MODES:
            if not str(self.config.nav2.launch_package or "").strip():
                errors.append("Nav2 sidecar 模式缺少 launch package 配置")
            if not str(self.config.nav2.launch_file or "").strip():
                errors.append("Nav2 sidecar 模式缺少 launch file 配置")
            map_yaml = str(getattr(self.config.nav2, "map_yaml", "") or "").strip()
            if map_yaml and not Path(map_yaml).expanduser().exists():
                errors.append("Nav2 map yaml 不存在 %s" % map_yaml)
            params_file = str(getattr(self.config.nav2, "params_file", "") or "").strip()
            if params_file and not Path(params_file).expanduser().exists():
                errors.append("Nav2 params file 不存在 %s" % params_file)
        return errors

    def _run_external_geometry_preflight_checks(self) -> List[str]:
        errors: List[str] = []
        if not bool(getattr(self.config.external, "enabled", True)):
            errors.append("外部 geometry 栈已禁用，但 NUWAX_GEOMETRY_STACK=%s 需要外部脚本。" % self.config.stack)
        raw_script_path = str(getattr(self.config.external, "script_path", "") or "").strip()
        script_path = self._external_script_path()
        if not raw_script_path:
            errors.append("缺少外部 geometry script 配置 NUWAX_GEOMETRY_EXTERNAL_SCRIPT")
        elif not script_path.exists():
            errors.append("外部 geometry script 不存在 %s" % script_path)
        elif not script_path.is_file():
            errors.append("外部 geometry script 不是文件 %s" % script_path)
        elif not os.access(script_path, os.X_OK):
            errors.append("外部 geometry script 不可执行 %s" % script_path)
        required_topics = {
            "occupancy_topic": self.config.topics.occupancy_topic,
            "cost_map_topic": self.config.topics.cost_map_topic,
            "tf_topic": self.config.topics.tf_topic,
            "tf_static_topic": self.config.topics.tf_static_topic,
        }
        for name, value in required_topics.items():
            if not str(value or "").strip():
                errors.append("缺少 external geometry topic 配置 %s" % name)
        required_frames = {
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
        }
        for name, value in required_frames.items():
            if not str(value or "").strip():
                errors.append("缺少 external geometry TF frame 配置 %s" % name)
        if self._nav2_mode() in _NAV2_SIDECAR_MODES:
            errors.append("外部 Go2 SLAM/Nav2 栈已提供 Nav2；请把 NUWAX_NAV2_MODE 配置为 external。")
        return errors

    def _ros_setup_candidates(self) -> List[Path]:
        return [
            Path("/opt/ros/humble/setup.bash"),
            Path("/opt/ros/jazzy/setup.bash"),
            Path("/opt/ros/iron/setup.bash"),
            Path("/opt/ros/foxy/setup.bash"),
        ]

    def _start_required_processes(self) -> None:
        if self._uses_external_geometry_stack():
            if self._external_input_bridge_enabled():
                self._start_managed_process(
                    "external_odom_tf_bridge",
                    self._build_odom_tf_bridge_command(),
                    "external_odom_tf_bridge.log",
                )
            self._start_managed_process(
                self._external_process_name(),
                self._build_external_geometry_command(),
                self._external_log_name(),
            )
            if self._external_input_bridge_enabled():
                bridge_message = "外部 Go2 SLAM/Nav2 脚本托管启动；平台启动 ROS2 odom/点云输入桥与 odom TF bridge，不启动内置 RTAB-Map/Nav2。"
            elif self._external_point_cloud_bridge_enabled():
                bridge_message = "外部 Go2 SLAM/Nav2 脚本托管启动；平台只启动 ROS2 点云输入桥，跳过 odom/odom_tf_bridge。"
            else:
                bridge_message = "外部 Go2 SLAM/Nav2 脚本托管启动；平台跳过内置 RTAB-Map/Nav2/odom_tf_bridge。"
            self._record_runtime_diagnostic(
                "go2slam_external",
                bridge_message,
            )
            return
        self._start_managed_process("odom_tf_bridge", self._build_odom_tf_bridge_command(), "odom_tf_bridge.log")
        self._start_static_tf_process("static_tf_base_to_body", self.config.frames.base_frame, "body")
        self._start_static_tf_process("static_tf_base_to_lidar", self.config.frames.base_frame, self.config.frames.lidar_frame)
        self._start_static_tf_process("static_tf_base_to_imu", self.config.frames.base_frame, getattr(self.config.frames, "imu_frame", "utlidar_imu"))
        self._start_managed_process("rtabmap", self._build_rtabmap_command(), "rtabmap_live.log")
        self._start_nav2_process_if_configured()

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
        return self._build_launch_prefix() + [
            "python3",
            str(self._geometry_script("odom_tf_bridge.py")),
            "--odom-topic",
            str(self.config.topics.odom_topic),
            "--odom-frame",
            str(self.config.frames.odom_frame),
            "--default-child-frame",
            str(self.config.frames.base_frame),
            "--child-frame",
            str(self.config.frames.base_frame),
        ]

    def _build_rtabmap_command(self) -> List[str]:
        db_path = self._rtabmap_database_path()
        db_path.parent.mkdir(parents=True, exist_ok=True)
        return self._build_launch_prefix() + [
            "ros2",
            "launch",
            str(self.config.rtabmap.launch_package),
            str(self.config.rtabmap.launch_file),
            "namespace:=/",
            "rtabmapviz:=false",
            "rviz:=false",
            "depth:=false",
            "subscribe_rgb:=false",
            "subscribe_rgbd:=false",
            "frame_id:=%s" % self.config.frames.base_frame,
            "map_frame_id:=%s" % self.config.frames.map_frame,
            "odom_frame_id:=%s" % self.config.frames.odom_frame,
            "publish_tf_map:=true",
            "visual_odometry:=false",
            "icp_odometry:=false",
            "subscribe_scan_cloud:=true",
            "scan_cloud_topic:=%s" % self.config.topics.point_cloud_topic,
            "odom_topic:=%s" % self.config.topics.odom_topic,
            "imu_topic:=%s" % self.config.topics.imu_topic,
            "qos:=2",
            "qos_scan:=2",
            "qos_odom:=1",
            "qos_imu:=2",
            "approx_sync:=false",
            "queue_size:=30",
            "wait_for_transform:=0.5",
            "database_path:=%s" % db_path,
            "args:=--delete_db_on_start --RGBD/CreateOccupancyGrid true --Grid/FromDepth false --Grid/RangeMax 8.0 --Grid/CellSize 0.05 --Grid/3D true --Grid/RayTracing true --Grid/FlatObstacleDetected true --Grid/MaxGroundHeight 0.35 --Grid/MaxObstacleHeight 1.8 --Grid/NormalsSegmentation false --Grid/MinClusterSize 5",
        ]

    def _build_static_tf_command(self, parent_frame: str, child_frame: str) -> List[str]:
        return self._build_launch_prefix() + [
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            str(parent_frame),
            str(child_frame),
        ]

    def _build_nav2_command(self) -> List[str]:
        command = self._build_launch_prefix() + [
            "ros2",
            "launch",
            str(self.config.nav2.launch_package),
            str(self.config.nav2.launch_file),
            "use_sim_time:=%s" % str(bool(getattr(self.config.nav2, "use_sim_time", False))).lower(),
            "autostart:=%s" % str(bool(getattr(self.config.nav2, "autostart", True))).lower(),
        ]
        map_yaml = str(getattr(self.config.nav2, "map_yaml", "") or "").strip()
        if map_yaml:
            command.append("map:=%s" % map_yaml)
        params_file = str(getattr(self.config.nav2, "params_file", "") or "").strip()
        if params_file:
            command.append("params_file:=%s" % params_file)
        return command

    def _start_static_tf_process(self, name: str, parent_frame: str, child_frame: str) -> None:
        if not str(parent_frame or "").strip() or not str(child_frame or "").strip():
            self._record_runtime_diagnostic(name, "静态 TF 缺少 parent 或 child frame。")
            return
        if parent_frame == child_frame:
            self._record_runtime_diagnostic(name, "静态 TF parent 与 child 相同，跳过启动。")
            return
        self._start_managed_process(name, self._build_static_tf_command(parent_frame, child_frame), "%s.log" % name)

    def _start_nav2_process_if_configured(self) -> None:
        nav2_mode = self._nav2_mode()
        if nav2_mode in _NAV2_DISABLED_MODES:
            return
        self._nav2_launch_attempted = True
        if nav2_mode in _NAV2_EXTERNAL_MODES:
            self._record_runtime_diagnostic("nav2_external", "Nav2 配置为外部模式，平台只暴露状态，不把 GridNavigationPlanner 伪装成 Nav2。")
            return
        if nav2_mode in _NAV2_SIDECAR_MODES:
            self._set_runtime_state("starting_nav2", "正在启动 Nav2 sidecar。")
            self._start_managed_process("nav2", self._build_nav2_command(), "nav2.log")
            return
        self._record_runtime_diagnostic("nav2_mode", "未知 Nav2 模式 %s，按待接入状态暴露。" % nav2_mode)

    def _start_managed_process(self, name: str, command: List[str], log_name: str, *, restart: bool = False) -> None:
        current = self._managed_processes.get(name)
        if current is not None and current.is_running():
            return
        process = current or _ManagedProcess(
            name=name,
            command=list(command),
            log_path=self._geometry_log_dir() / log_name,
            cwd=self._repo_root(),
        )
        process.command = list(command)
        process.start(restart=restart)
        self._managed_processes[name] = process

    def _stop_managed_processes(self) -> None:
        for process in list(self._managed_processes.values()):
            process.stop()

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

    def _monitor_processes_once(self) -> None:
        for name, process in list(self._managed_processes.items()):
            exit_code = process.poll()
            if exit_code is None or process.is_running():
                continue
            if name == "nav2" and self._nav2_mode() not in _NAV2_SIDECAR_MODES:
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
        process_failure = self._blocking_process_failure()
        if process_failure:
            self._set_runtime_state("failed", process_failure, blocked=True)
            return
        live_map_status = self._clone_metadata(self._live_map_provider.get_status())
        tf_localization_status = self._clone_metadata(self._tf_localization_provider.get_status())
        odometry_status = self._clone_metadata(self._odometry_publisher.get_status())
        point_cloud_status = self._clone_metadata(self._point_cloud_publisher.get_status())
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
            self._set_runtime_state("waiting_odom", str(readiness.get("message") or "等待 Go2 odom 输入。"))
            return
        if not bool(readiness.get("first_frame_ready")):
            self._set_runtime_state("waiting_map", str(readiness.get("message") or "等待 RTAB-Map /map 首帧。"))
            return
        if not bool(readiness.get("streaming_ready")):
            self._set_runtime_state("first_frame", str(readiness.get("message") or "RTAB-Map 已发布 /map 首帧，正在确认连续流式更新。"))
            return
        if not bool(readiness.get("localization_ready")):
            self._set_runtime_state("streaming", str(readiness.get("message") or "地图 streaming 已就绪，等待 RTAB-Map localization/relocalization。"))
            return
        nav2_status = self._build_nav2_status()
        nav2_phase = str(nav2_status.get("phase") or "")
        if nav2_phase == "starting":
            self._set_runtime_state("starting_nav2", "Nav2 sidecar 正在启动。")
            return
        if nav2_phase == "failed":
            self._set_runtime_state("failed", str(nav2_status.get("message") or "Nav2 sidecar 失败。"), blocked=True)
            return
        exploration_state = self._platform_exploration_runtime.get_exploration_state()
        if getattr(exploration_state.status, "value", exploration_state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }:
            message = (
                "外部 go2slam 地图 streaming 已就绪，平台探索任务运行中。"
                if self._uses_external_geometry_stack()
                else "RTAB-Map localization 与地图 streaming 已就绪，平台探索任务运行中。"
            )
            self._set_runtime_state("exploring", message)
            return
        if nav2_phase != "ready":
            message = (
                "外部 go2slam 地图 streaming 已就绪；Nav2 状态：%s。"
                if self._uses_external_geometry_stack()
                else "RTAB-Map localization 与地图 streaming 已就绪；Nav2 状态：%s。"
            )
            self._set_runtime_state("streaming", message % nav2_status.get("phase"))
            return
        if self._uses_external_geometry_stack():
            self._set_runtime_state("ready", "外部 Go2 SLAM/Nav2 地图 streaming、TF 与 Nav2 均已就绪。")
        else:
            self._set_runtime_state("ready", "RTAB-Map localization、地图 streaming 与 Nav2 均已就绪。")

    def _blocking_process_failure(self) -> Optional[str]:
        for name, process in self._managed_processes.items():
            if process.is_running():
                continue
            exit_code = process.poll()
            if exit_code is None:
                continue
            if process.restart_count >= _GEOMETRY_SIDE_CAR_RESTART_LIMIT and name != "nav2":
                return "%s 进程已退出，exit_code=%s。" % (name, exit_code)
            if name == "nav2" and self._nav2_mode() in _NAV2_SIDECAR_MODES and process.restart_count >= _GEOMETRY_SIDE_CAR_RESTART_LIMIT:
                return "Nav2 sidecar 已退出，exit_code=%s。" % exit_code
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
        live_map_status = live_map_status or self._clone_metadata(self._live_map_provider.get_status())
        tf_localization_status = tf_localization_status or self._clone_metadata(self._tf_localization_provider.get_status())
        odometry_status = odometry_status or self._clone_metadata(self._odometry_publisher.get_status())
        point_cloud_status = point_cloud_status or self._clone_metadata(self._point_cloud_publisher.get_status())
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
        waiting_for, wait_message = self._classify_live_mapping_wait(
            live_map_status=live_map_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )
        odom_bridge = self._managed_processes.get("odom_tf_bridge")
        rtabmap_process = self._managed_processes.get("rtabmap")
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        rtabmap_info_publisher_count = live_map_status.get("rtabmap_info_publisher_count")
        if rtabmap_info_publisher_count is None:
            rtabmap_info_publisher_count = live_map_status.get("rtabmap_info_has_publisher")
        go2_odom_ready = bool(
            odom_bridge is not None
            and odom_bridge.is_running()
            and int(odometry_status.get("published_count") or 0) > 0
            and int(point_cloud_status.get("published_count") or 0) > 0
            and (point_cloud_source.get("timestamp") or data_plane_status.get("started") is not False)
        )
        first_frame_ready = bool(
            live_map_status.get("occupancy_grid_ready")
            and int(live_map_status.get("occupancy_message_count") or 0) > 0
            and live_map_status.get("occupancy_has_publisher") is not False
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        localization_ready = bool(
            streaming_ready
            and rtabmap_process is not None
            and rtabmap_process.is_running()
            and tf_chain.get("ready")
            and map_stream.get("source_timestamp_fresh")
            and (rtabmap_info_publisher_count is True or int(rtabmap_info_publisher_count or 0) > 0)
        )
        relocalization_ready = bool(localization_ready and int(map_stream.get("content_revision") or 0) > 0)
        if not go2_odom_ready:
            message = "Go2 odom/TF 输入未就绪；需要平台 odom 发布桥、odom_tf_bridge 运行，且点云已发布。"
            waiting_for = waiting_for or "go2_odom"
        elif not first_frame_ready:
            message = wait_message
        elif not streaming_ready:
            message = "RTAB-Map 已发布 /map 首帧，但 /map message_count、source timestamp 或内容指标尚未证明连续 streaming。"
            waiting_for = "rtabmap_streaming"
        elif not localization_ready:
            message = "RTAB-Map /map 已 streaming，但 /rtabmap/info、TF 链或进程状态尚未证明 localization/relocalization ready。"
            waiting_for = "rtabmap_localization"
        else:
            message = "Go2 odom、RTAB-Map /map streaming、/rtabmap/info 与 TF 链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": go2_odom_ready,
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "localization_ready": localization_ready,
            "relocalization_ready": relocalization_ready,
            "waiting_for": waiting_for,
            "message": message,
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "rtabmap_info": {
                "topic": "/rtabmap/info",
                "publisher_count": rtabmap_info_publisher_count,
                "has_publisher": rtabmap_info_publisher_count is True or int(rtabmap_info_publisher_count or 0) > 0,
            },
            "rtabmap_process": rtabmap_process.status() if rtabmap_process is not None else None,
            "odometry_publisher": odometry_status,
            "odom_tf_bridge_process": odom_bridge.status() if odom_bridge is not None else None,
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
        odometry_status = odometry_status or self._clone_metadata(self._odometry_publisher.get_status())
        point_cloud_status = point_cloud_status or self._clone_metadata(self._point_cloud_publisher.get_status())
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
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        localization_ready = bool(process_running and input_bridge_ready and streaming_ready and tf_chain.get("ready"))
        if not process_running:
            message = "外部 Go2 SLAM/Nav2 脚本尚未运行。"
            waiting_for = "external_process"
        elif not input_bridge_ready:
            message = (
                "外部 go2slam 点云输入桥未就绪；需要平台发布点云输入。"
                if point_cloud_bridge_enabled and not input_bridge_enabled
                else "外部 go2slam 输入桥未就绪；需要平台发布 /odom、点云输入，并运行 odom -> base TF bridge。"
            )
            waiting_for = "external_input_bridge"
        elif not first_frame_ready:
            message = "等待外部 go2slam 发布 /map 首帧。"
            waiting_for = "external_map"
        elif not streaming_ready:
            message = "外部 go2slam 已发布 /map 首帧，正在确认连续 streaming。"
            waiting_for = "external_map_streaming"
        elif not tf_chain.get("ready"):
            message = "外部 go2slam /map 已 streaming，等待 map -> odom -> base TF 链就绪。"
            waiting_for = "external_tf"
        else:
            message = "外部 Go2 SLAM/Nav2 /map streaming 与 TF 链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": bool(process_running and input_bridge_ready),
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "localization_ready": localization_ready,
            "relocalization_ready": localization_ready,
            "waiting_for": waiting_for,
            "message": message,
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "rtabmap_info": {
                "topic": "/rtabmap/info",
                "publisher_count": None,
                "has_publisher": None,
                "not_required": True,
            },
            "rtabmap_process": None,
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
            },
            "external_process": external_process.status() if external_process is not None else None,
        }

    def _build_tf_chain_status(self, *, tf_localization_status: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        tf_localization_status = tf_localization_status or self._clone_metadata(self._tf_localization_provider.get_status())
        if self._uses_external_geometry_stack():
            required_processes = [self._external_process_name()]
            if self._external_input_bridge_enabled():
                required_processes.append("external_odom_tf_bridge")
        else:
            required_processes = ["odom_tf_bridge"]
            if str(self.config.frames.base_frame or "").strip() != "body":
                required_processes.append("static_tf_base_to_body")
            if str(self.config.frames.base_frame or "").strip() != str(self.config.frames.lidar_frame or "").strip():
                required_processes.append("static_tf_base_to_lidar")
            imu_frame = str(getattr(self.config.frames, "imu_frame", "") or "").strip()
            if imu_frame and str(self.config.frames.base_frame or "").strip() != imu_frame:
                required_processes.append("static_tf_base_to_imu")
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
        chain_ready = not missing and not stopped and pose_ready and map_to_base_ready
        return {
            "ready": chain_ready,
            "process_ready": not missing and not stopped,
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
            "validation": (
                "external_go2slam_tf_provider_must_resolve_map_to_base_pose"
                if self._uses_external_geometry_stack()
                else "ros2_tf_provider_must_resolve_map_to_base_pose"
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
        source_fresh = latest_age_sec is not None and latest_age_sec <= max(1.0, float(self.config.rtabmap.timestamp_quality.max_tf_age_sec) * 4.0)
        count_recent = last_growth_at is not None and (now - last_growth_at).total_seconds() <= _GEOMETRY_STREAMING_WINDOW_SEC
        content_recent = last_content_change_at is not None and (now - last_content_change_at).total_seconds() <= _GEOMETRY_STREAMING_WINDOW_SEC * 2.0
        growth_window = (now - first_growth_at).total_seconds() if first_growth_at is not None else 0.0
        streaming = bool(
            count >= max(first_count + 1, 2)
            and source_fresh
            and count_recent
            and (content_recent or known_count > 0)
            and growth_window <= _GEOMETRY_STREAMING_WINDOW_SEC * max(1, count - first_count + 1)
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

    def _nav2_mode(self) -> str:
        return str(getattr(self.config.nav2, "mode", "external") or "external").strip().lower()

    def _navigation_backend_policy(self) -> Dict[str, object]:
        mode = self._nav2_mode()
        nav2_provider = getattr(self, "_nav2_action_provider", None)
        nav2_provider_name = getattr(nav2_provider, "provider_name", "nav2_action_navigation")
        policy = {
            "mode": mode,
            "execution_chain": "nav2_action_only",
            "single_execution_chain": True,
            "platform_grid_removed": True,
            "removed_backend_kind": "platform_grid",
            "removed_backend_provider": None,
            "nav2_action_provider_name": nav2_provider_name,
            "nav2_action_active": False,
            "platform_grid_active": False,
            "selected_provider_name": None,
            "active_backend_provider": None,
            "active_backend_kind": None,
            "message": None,
        }
        if mode in _NAV2_DISABLED_MODES:
            policy.update({
                "selection": "disabled",
                "message": "Nav2 已禁用，平台未选择导航执行后端。",
            })
        elif mode in _REMOVED_PLATFORM_GRID_MODES:
            policy.update({
                "selection": "removed_platform_grid",
                "message": "platform_grid 自定义导航执行链已删除；请使用 Nav2 action 模式。",
            })
        elif mode in _NAV2_EXTERNAL_MODES or mode in _NAV2_SIDECAR_MODES:
            policy.update({
                "selection": "nav2_action",
                "selected_provider_name": nav2_provider_name,
                "active_backend_provider": nav2_provider_name,
                "active_backend_kind": "nav2_action",
                "nav2_action_active": True,
                "message": "当前正式导航执行链为 Nav2 action。",
            })
        else:
            policy.update({
                "selection": "unsupported",
                "message": "Nav2 模式 %s 尚未接入导航后端选择。" % mode,
            })
        policy["active_navigation_backend"] = policy["active_backend_provider"]
        return policy

    def _with_navigation_backend_policy(self, status: Dict[str, object]) -> Dict[str, object]:
        policy = self._navigation_backend_policy()
        return {
            **dict(status or {}),
            "navigation_backend_policy": policy,
            "active_navigation_backend": policy.get("active_backend_provider"),
            "nav2_action_active": bool(policy.get("nav2_action_active")),
            "platform_grid_active": bool(policy.get("platform_grid_active")),
            "platform_grid_removed": bool(policy.get("platform_grid_removed")),
        }

    def _build_nav2_status(self) -> Dict[str, object]:
        mode = self._nav2_mode()
        process = self._managed_processes.get("nav2")
        if mode in _NAV2_DISABLED_MODES:
            phase = "disabled"
            message = "Nav2 已禁用；平台不再提供 platform_grid 自定义导航执行链。"
        elif mode in _REMOVED_PLATFORM_GRID_MODES:
            phase = "removed_platform_grid"
            message = "platform_grid 自定义导航执行链已删除；请把 NUWAX_NAV2_MODE 配置为 sidecar 或 external。"
        elif mode in _NAV2_EXTERNAL_MODES:
            provider = getattr(self, "_nav2_action_provider", None)
            if provider is not None:
                return self._with_navigation_backend_policy(provider.get_nav2_status())
            phase = "external"
            message = "Nav2 配置为外部模式，但 action provider 尚未初始化。"
        elif mode in _NAV2_SIDECAR_MODES:
            if process is None:
                phase = "not_started"
                message = "Nav2 sidecar 尚未启动。"
            elif process.is_running():
                provider = getattr(self, "_nav2_action_provider", None)
                if provider is not None:
                    status = provider.get_nav2_status()
                    status["process"] = process.status()
                    status["launch_attempted"] = self._nav2_launch_attempted
                    return self._with_navigation_backend_policy(status)
                phase = "blocked"
                message = "Nav2 sidecar 进程已启动，但 action provider 尚未初始化。"
            else:
                phase = "failed"
                message = "Nav2 sidecar 进程已退出，exit_code=%s。" % process.exit_code
        else:
            phase = "pending_integration"
            message = "Nav2 模式 %s 尚未接入。" % mode
        status = {
            "mode": mode,
            "phase": phase,
            "message": message,
            "action_name": self.config.nav2.action_name,
            "launch_package": self.config.nav2.launch_package,
            "launch_file": self.config.nav2.launch_file,
            "global_costmap_topic": self.config.nav2.global_costmap_topic,
            "local_costmap_topic": self.config.nav2.local_costmap_topic,
            "costmap_source": self.config.nav2.costmap_source,
            "platform_grid_removed": True,
            "grid_navigation_backend": None,
            "grid_navigation_is_nav2": False,
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
            "tf_localization_provider": self._tf_localization_provider.get_status(),
            "diagnostics": diagnostics,
            "launch_ros2_stack": {
                "script": str(self._geometry_script("launch_ros2_stack.sh")),
                "starts_rtabmap_or_nav2_by_default": False,
                "message": "launch_ros2_stack.sh 只准备 ROS2 环境；RTAB-Map/Nav2 由 PlatformGeometryRuntime 显式启动。",
            },
        }

    def _ensure_live_geometry_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        with self._live_geometry_start_lock:
            if (
                self._live_map_provider.is_available()
                and self._tf_localization_provider.is_available()
                and self._odometry_publisher.is_available()
                and self._point_cloud_publisher.is_available()
            ):
                return
            self._live_geometry_start_attempted = True
            try:
                self._odometry_publisher.start()
                self._point_cloud_publisher.start()
                self._live_map_provider.start()
                self._tf_localization_provider.start()
            except Exception:
                self._odometry_publisher.stop()
                self._point_cloud_publisher.stop()
                self._live_map_provider.stop()
                self._live_geometry_start_attempted = False
                self._tf_localization_provider.stop()
                raise

    def _ensure_external_geometry_observers_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        with self._live_geometry_start_lock:
            input_bridge_enabled = self._external_input_bridge_enabled()
            point_cloud_bridge_enabled = self._external_point_cloud_bridge_enabled()
            input_bridge_ready = True
            if input_bridge_enabled:
                input_bridge_ready = self._odometry_publisher.is_available() and self._point_cloud_publisher.is_available()
            elif point_cloud_bridge_enabled:
                input_bridge_ready = self._point_cloud_publisher.is_available()
            if self._live_map_provider.is_available() and self._tf_localization_provider.is_available() and input_bridge_ready:
                return
            self._live_geometry_start_attempted = True
            try:
                if input_bridge_enabled:
                    self._odometry_publisher.start()
                if point_cloud_bridge_enabled:
                    self._point_cloud_publisher.start()
                self._live_map_provider.start()
                self._tf_localization_provider.start()
            except Exception:
                if input_bridge_enabled:
                    self._odometry_publisher.stop()
                if point_cloud_bridge_enabled:
                    self._point_cloud_publisher.stop()
                self._live_map_provider.stop()
                self._tf_localization_provider.stop()
                self._live_geometry_start_attempted = False
                raise

    def is_available(self) -> bool:
        """返回是否存在可供平台消费的几何后端。"""

        if not self._is_geometry_profile_enabled():
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
            elif backend is self._live_map_provider:
                source_kind = "platform_live_map_provider"
            elif backend is self._tf_localization_provider:
                source_kind = "platform_tf_localization_provider"
            else:
                source_kind = "platform_backend"
            backend_name = getattr(backend, "provider_name", None)
        return {
            "source_kind": source_kind,
            "backend_name": backend_name,
            "runtime_status": self.get_runtime_status(),
            "live_map_provider": self._live_map_provider.get_status(),
            "tf_localization_provider": self._tf_localization_provider.get_status(),
            "odometry_publisher": self._odometry_publisher.get_status(),
            "point_cloud_publisher": self._point_cloud_publisher.get_status(),
            "navigation_backend_policy": self._navigation_backend_policy(),
            "active_map_name": self._active_map_version.map_name if self._active_map_version is not None else None,
            "active_map_version_id": self._active_map_version.version_id if self._active_map_version is not None else None,
            **self.config.build_metadata(),
        }

    def get_live_mapping_diagnostics(self) -> Dict[str, object]:
        """返回探索建图输入链路的可行动诊断。"""

        live_map_status = self._clone_metadata(self._live_map_provider.get_status())
        tf_localization_status = self._clone_metadata(self._tf_localization_provider.get_status())
        odometry_status = self._clone_metadata(self._odometry_publisher.get_status())
        point_cloud_status = self._clone_metadata(self._point_cloud_publisher.get_status())
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
            "rtabmap_profile_name": self.config.rtabmap.profile_name,
            "rtabmap_launch_package": self.config.rtabmap.launch_package,
            "rtabmap_launch_file": self.config.rtabmap.launch_file,
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
        point_cloud_error = str(point_cloud_status.get("last_error") or "").strip()
        if point_cloud_error:
            return "point_cloud_bridge_error", "ROS2 点云桥错误：%s" % point_cloud_error
        live_map_error = str(live_map_status.get("last_error") or "").strip()
        if live_map_error:
            return "live_map_provider_error", "ROS2 实时地图订阅错误：%s" % live_map_error
        source_error = str(point_cloud_source.get("last_error") or "").strip()
        occupancy_grid_ready = bool(live_map_status.get("occupancy_grid_ready"))
        if source_error and not occupancy_grid_ready:
            return "point_cloud_input", source_error
        published_count = int(point_cloud_status.get("published_count") or 0)
        point_cloud_topic = str(point_cloud_status.get("topic") or point_cloud_status.get("point_cloud_topic") or self.config.topics.point_cloud_topic)
        source_topic = str(
            point_cloud_source.get("source_topic")
            or data_plane_status.get("point_cloud_source_topic")
            or data_plane_status.get("direct_dds_point_cloud_topic")
            or "rt/utlidar/cloud"
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
                "rtabmap_occupancy_grid",
                "点云桥已向 %s 发布 %d 帧，等待 RTAB-Map 在 %s 发布首帧占据栅格。"
                % (point_cloud_topic, published_count, occupancy_topic),
            )
        occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
        return None, "建图链路已收到 %s 首帧占据栅格。" % occupancy_topic

    def _attach_live_mapping_diagnostics_to_request(self, request):
        metadata = dict(getattr(request, "metadata", {}) or {})
        metadata["initial_mapping_chain_diagnostics"] = self.get_live_mapping_diagnostics()
        copier = getattr(request, "model_copy", None)
        if callable(copier):
            return request.model_copy(update={"metadata": metadata}, deep=True)
        request.metadata = metadata
        return request

    def _augment_exploration_state_with_mapping_diagnostics(self, state):
        diagnostics = self.get_live_mapping_diagnostics()
        metadata = dict(getattr(state, "metadata", {}) or {})
        current_waiting_for = str(metadata.get("waiting_for") or "").strip() or None
        diagnostic_waiting_for = diagnostics.get("waiting_for")
        active = getattr(state.status, "value", state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }
        override_waiting = bool(
            active
            and diagnostic_waiting_for
            and (
                current_waiting_for in {None, "occupancy_grid", "mapping_ready", "point_cloud_input", "rtabmap_occupancy_grid"}
                or str(diagnostic_waiting_for).endswith("_error")
            )
        )
        if override_waiting:
            metadata["waiting_for"] = diagnostic_waiting_for
            metadata["waiting_detail"] = diagnostics.get("message")
        metadata["mapping_chain_diagnostics"] = diagnostics
        updates = {"metadata": metadata}
        if override_waiting and diagnostics.get("message"):
            updates["message"] = diagnostics.get("message")
        return state.model_copy(update=updates, deep=True)

    def clear_active_map_version(self, *, reason: str = "") -> None:
        """清空当前激活的平台地图版本上下文。"""

        self._active_map_version = None
        self._last_activation_result = {"cleared": True, "reason": str(reason or "").strip() or None}

    def prepare_map_version_assets(self, version: MapVersion) -> MapVersion:
        """为平台地图版本补齐并落盘几何资产描述。"""

        previous_asset = version.geometry_asset
        parameter_summary = self._build_parameter_summary()
        profile_summary = self._clone_metadata(parameter_summary.get("profile") or {})
        rtabmap_db_path = self._version_rtabmap_database_path(version.map_name, version.version_id)
        pcd_path = self._version_pcd_asset_path(version.map_name, version.version_id)
        built_at = previous_asset.built_at if previous_asset is not None else version.timestamp
        rtabmap_run = self._build_rtabmap_run_metadata()
        database_metadata = self._materialize_rtabmap_database(rtabmap_db_path)
        pcd_metadata = self._materialize_pcd_asset(pcd_path)
        quality_metrics = self._build_geometry_quality_metrics(
            database_metadata=database_metadata,
            pcd_metadata=pcd_metadata,
            rtabmap_run=rtabmap_run,
        )
        asset_refs = (
            MapGeometryAssetRef(
                asset_type=MapGeometryAssetType.RTABMAP_DATABASE,
                path=str(rtabmap_db_path),
                frame_id=version.frame_id,
                profile_summary=profile_summary,
                built_at=built_at,
                quality_metrics=dict(quality_metrics.get("database") or {}),
                metadata={"required": True},
            ),
            MapGeometryAssetRef(
                asset_type=MapGeometryAssetType.PCD,
                path=str(pcd_path),
                frame_id=version.frame_id,
                profile_summary=profile_summary,
                built_at=built_at,
                quality_metrics=dict(quality_metrics.get("pcd") or {}),
                metadata={"required": False},
            ),
        )
        geometry_asset = MapGeometryAssetBinding(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_stack=self.config.stack,
            frame_id=version.frame_id,
            rtabmap_db_path=str(rtabmap_db_path),
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
                "source_mode": "live_streaming_checkpoint" if quality_metrics.get("live_streaming_ready") else "offline_checkpoint",
                "streaming_readiness": self._clone_metadata(quality_metrics.get("streaming_readiness")),
                "rtabmap_run": rtabmap_run,
                "session_id": rtabmap_run.get("session_id"),
                "run_id": rtabmap_run.get("run_id"),
                "database": database_metadata,
                "pcd": pcd_metadata,
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
                    "rtabmap_session_id": rtabmap_run.get("session_id"),
                    "rtabmap_run_id": rtabmap_run.get("run_id"),
                    "rtabmap_database_path": database_metadata.get("database_source_path") or database_metadata.get("database_target_path"),
                    "rtabmap_live_streaming_ready": quality_metrics.get("live_streaming_ready"),
                    "map_version_source_mode": quality_metrics.get("source_mode"),
                    "checkpoint_ready": quality_metrics.get("checkpoint_ready"),
                },
            },
            deep=True,
        )

    def activate_map_version(self, version: MapVersion) -> MapSnapshot:
        """激活平台地图版本对应的几何资产。"""

        prepared_version = self.prepare_map_version_assets(version)
        geometry_asset = prepared_version.geometry_asset
        assert geometry_asset is not None
        activation_result = self._activate_backend_map_version(prepared_version)
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
        return self._live_map_provider.subscribe_map_updates(callback)

    def unsubscribe_map_updates(self, subscription_id: int) -> bool:
        return self._live_map_provider.unsubscribe_map_updates(subscription_id)

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
        live_status = self._clone_metadata(self._live_map_provider.get_status())
        update_sequence = live_status.get("update_sequence")
        occupancy_sequence = int(live_status.get("occupancy_sequence") or live_status.get("occupancy_message_count") or 0)
        cost_map_sequence = int(live_status.get("cost_map_sequence") or live_status.get("cost_map_message_count") or 0)
        message_count = max(int(live_status.get("message_count") or 0), occupancy_sequence + cost_map_sequence)
        content_revision = max(
            int(live_status.get("occupancy_revision") or 0),
            int(live_status.get("cost_map_revision") or 0),
        )
        revision_value = update_sequence or message_count or content_revision
        try:
            live_revision = max(1, int(revision_value or 1))
        except (TypeError, ValueError):
            live_revision = 1
        seed = "|".join(str(part or "") for part in (
            live_status.get("provider_name"),
            live_status.get("occupancy_topic"),
            live_status.get("cost_map_topic"),
            live_status.get("started"),
        ))
        digest = hashlib.sha256(seed.encode("utf-8")).hexdigest()[:16] if seed.strip("|") else "current"
        metadata = {
            "source_kind": "platform_live_map_provider",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "rtabmap_live_streaming_ready": True,
            "source_name": live_status.get("provider_name") or self._live_map_provider.provider_name,
            "update_sequence": update_sequence,
            "occupancy_sequence": occupancy_sequence,
            "cost_map_sequence": cost_map_sequence,
            "message_count": message_count,
            "occupancy_content_revision": live_status.get("occupancy_revision"),
            "cost_map_content_revision": live_status.get("cost_map_revision"),
            "content_revision": content_revision,
            "occupancy_metrics": self._clone_metadata(live_status.get("occupancy_metrics") or {}),
            "cost_map_metrics": self._clone_metadata(live_status.get("cost_map_metrics") or {}),
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
            source_name=str(live_status.get("provider_name") or self._live_map_provider.provider_name),
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
                    and geometry_asset.quality_metrics.get("database", {}).get("present", False)
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
        if self._tf_localization_provider.is_available():
            return self._tf_localization_provider
        return self._resolve_raw_localization_backend()

    def _resolve_mapping_backend(self):
        if self._live_map_provider.is_available():
            return self._live_map_provider
        return self._resolve_backend(MappingDataPlane, MapProvider)

    def _resolve_point_cloud_provider(self):
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, PointCloudProvider):
            return providers
        return None

    def _resolve_navigation_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        mode = self._nav2_mode()
        if mode in _NAV2_DISABLED_MODES:
            return None
        if mode in _REMOVED_PLATFORM_GRID_MODES:
            return None
        nav2_provider = getattr(self, "_nav2_action_provider", None)
        if mode in _NAV2_EXTERNAL_MODES or mode in _NAV2_SIDECAR_MODES:
            return nav2_provider
        return None

    def _resolve_exploration_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = getattr(self, "_platform_exploration_runtime", None)
        if backend is not None:
            return backend
        return self._resolve_backend(ExplorationDataPlane, ExplorationProvider)

    def _safe_map_dir(self, map_name: str) -> Path:
        return self._catalog_root / quote(str(map_name or "").strip(), safe="")

    def _version_dir(self, map_name: str, version_id: str) -> Path:
        return self._safe_map_dir(map_name) / "versions" / quote(str(version_id or "").strip(), safe="")

    def _version_geometry_root(self, map_name: str, version_id: str) -> Path:
        return self._version_dir(map_name, version_id) / "geometry" / self.config.stack

    def _geometry_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "geometry_asset.json"

    def _version_rtabmap_database_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "rtabmap" / self.config.rtabmap.outputs.database_asset_name

    def _version_pcd_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "pcd" / self.config.rtabmap.outputs.pcd_asset_name

    def _build_parameter_summary(self) -> Dict[str, object]:
        return self._clone_metadata(self.config.build_metadata())

    def _materialize_rtabmap_database(self, target_path: Path) -> Dict[str, object]:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path = self._locate_live_rtabmap_database()
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
        source_before = self._file_metadata(resolved_source)
        if resolved_source != resolved_target:
            shutil.copy2(resolved_source, resolved_target)
        source_after = self._file_metadata(resolved_source)
        return {
            "database_present": target_path.exists(),
            "database_source_kind": "live_runtime",
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

    def _materialize_pcd_asset(self, target_path: Path) -> Dict[str, object]:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path = self._locate_live_pcd_asset()
        if source_path is None:
            if target_path.exists() and target_path.is_file():
                return {
                    "pcd_present": True,
                    "pcd_source_kind": "persisted_asset",
                    "pcd_source_path": str(target_path.resolve()),
                    "pcd_target_path": str(target_path.resolve()),
                    "pcd_target": self._file_metadata(target_path),
                }
            return {
                "pcd_present": False,
                "pcd_source_kind": "unavailable",
                "pcd_source_path": None,
                "pcd_target_path": str(target_path.resolve()),
            }
        resolved_source = source_path.resolve()
        resolved_target = target_path.resolve()
        if resolved_source != resolved_target:
            shutil.copy2(resolved_source, resolved_target)
        return {
            "pcd_present": target_path.exists(),
            "pcd_source_kind": "live_runtime",
            "pcd_source_path": str(resolved_source),
            "pcd_target_path": str(resolved_target),
            "pcd_source": self._file_metadata(resolved_source),
            "pcd_target": self._file_metadata(resolved_target),
        }

    def _build_geometry_quality_metrics(
        self,
        *,
        database_metadata: Dict[str, object],
        pcd_metadata: Dict[str, object],
        rtabmap_run: Dict[str, object],
    ) -> Dict[str, object]:
        diagnostics = []
        database_present = bool(database_metadata.get("database_present", False))
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
        if not database_present:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "rtabmap_database_missing",
                    "message": "地图版本缺少 RTAB-Map database 资产。",
                }
            )
        if database_present and not live_streaming_ready:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "rtabmap_database_checkpoint_only",
                    "message": "RTAB-Map database 已落盘为离线 checkpoint，但未证明 live rtabmap.db 正在随 /map streaming 更新。",
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
                    "message": "未能从 RTAB-Map 进程状态解析真实 run_id。",
                }
            )
        return {
            "status": "ready" if database_present else "failed",
            "live_streaming_ready": live_streaming_ready,
            "checkpoint_ready": database_present,
            "source_mode": "live_streaming_checkpoint" if live_streaming_ready else "offline_checkpoint",
            "streaming_readiness": readiness,
            "rtabmap_run": rtabmap_run,
            "database": {
                "present": database_present,
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
        process = self._managed_processes.get("rtabmap")
        db_path = self._locate_live_rtabmap_database()
        started_at = process.started_at_iso if process is not None else None
        pid = process.process.pid if process is not None and process.process is not None and process.is_running() else None
        source = str(db_path.resolve()) if db_path is not None else None
        seed = "|".join(str(part or "") for part in (source, started_at, pid))
        digest = hashlib.sha256(seed.encode("utf-8")).hexdigest()[:16] if seed.strip("|") else ""
        run_id = "rtabmap_%s" % digest if digest else None
        return {
            "session_id": run_id,
            "run_id": run_id,
            "database_path": source,
            "started_at": started_at,
            "pid": pid,
            "node_name": "rtabmap",
            "log_path": str(process.log_path) if process is not None else None,
            "process_running": process.is_running() if process is not None else False,
        }

    def get_live_pointcloud_map_asset_summary(self) -> Dict[str, object]:
        """返回当前累计点云地图 PCD 资产候选，不读取扫描帧。"""

        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            geometry_asset = self._active_map_version.geometry_asset
            current_path = geometry_asset.pcd_path
            if current_path:
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
            if candidate.exists() and candidate.is_file():
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
                if current_pcd.exists() and current_pcd.is_file():
                    return current_pcd

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_pcd_map_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if candidate_path.exists() and candidate_path.is_file():
                        return candidate_path

        configured_root = Path(str(self.config.rtabmap.pcd_root or "").strip()).expanduser()
        candidates = []
        if configured_root.suffix == ".pcd":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / self.config.rtabmap.outputs.pcd_asset_name,
                    configured_root / "cloud_map.pcd",
                    configured_root / "map.pcd",
                    configured_root / "current.pcd",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _locate_live_rtabmap_database(self) -> Optional[Path]:
        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            current_db = Path(self._active_map_version.geometry_asset.rtabmap_db_path).expanduser()
            if current_db.exists() and current_db.is_file():
                return current_db

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_rtabmap_database_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if candidate_path.exists() and candidate_path.is_file():
                        return candidate_path

        configured_root = Path(str(self.config.rtabmap.database_root or "").strip()).expanduser()
        candidates = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "rtabmap.db",
                    configured_root / "current.db",
                    configured_root / "active.db",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _activate_backend_map_version(self, version: MapVersion) -> Dict[str, object]:
        geometry_asset = version.geometry_asset
        activation_payload = {
            "map_name": version.map_name,
            "version_id": version.version_id,
            "geometry_stack": self.config.stack,
            "rtabmap_db_path": geometry_asset.rtabmap_db_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "frame_id": version.frame_id,
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
        }
        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "mapping_backend_unavailable",
            }
        activator = getattr(mapping_backend, "activate_platform_map_version", None)
        if not callable(activator):
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "backend_missing_activate_platform_map_version",
            }
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
            return {
                **activation_payload,
                "backend_switch_applied": True,
                **self._clone_metadata(result),
            }
        return {
            **activation_payload,
            "backend_switch_applied": True,
            "backend_result": result,
        }

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
        return bool(self._live_map_provider.is_map_available() or self._active_map_version is not None)

    def subscribe_localization_updates(self, callback) -> Optional[int]:
        """订阅平台 TF 定位输出。"""

        if not self._is_geometry_profile_enabled():
            return None
        return self._tf_localization_provider.subscribe_localization(callback)

    def unsubscribe_localization_updates(self, subscription_id: Optional[int]) -> None:
        self._tf_localization_provider.unsubscribe_localization(subscription_id)

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
        self._ensure_live_geometry_started()
        request = self._attach_live_mapping_diagnostics_to_request(request)
        old_thread = None
        with self._exploration_loop_lock:
            self._exploration_loop_stop_event.set()
            self._exploration_loop_generation += 1
            old_thread = self._exploration_loop_thread
        if old_thread is not None and old_thread.is_alive() and old_thread is not threading.current_thread():
            old_thread.join(timeout=max(0.2, self._exploration_loop_interval_sec() * 2.0))
        self._cancel_platform_exploration_navigation()
        accepted = self._platform_exploration_runtime.start_exploration(request)
        if not accepted:
            return False
        with self._exploration_loop_lock:
            self._exploration_loop_stop_event.clear()
            self._exploration_loop_generation += 1
            self._exploration_tick_count = 0
            self._exploration_last_tick_at_iso = None
            self._exploration_last_tick_monotonic = None
            self._exploration_last_tick_source = "start_exploration"
            self._ensure_platform_exploration_loop_locked()
        self._notify_exploration_update(self.get_platform_exploration_state())
        return True

    def stop_platform_exploration(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self._stop_platform_exploration_loop(cancel_navigation=True, notify=True)

    def report_platform_exploration_hit_stop(self, reason: str) -> None:
        if not self._is_geometry_profile_enabled():
            return
        thread = None
        with self._exploration_loop_lock:
            self._exploration_loop_stop_event.set()
            self._exploration_loop_generation += 1
            thread = self._exploration_loop_thread
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        self._platform_exploration_runtime.report_hit_stop(reason)
        self._cancel_platform_exploration_navigation(current_goal=current_goal)
        self._notify_compat_exploration_stopped()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.2, self._exploration_loop_interval_sec() * 2.0))
        self._notify_exploration_update(self.get_platform_exploration_state())

    def _stop_platform_exploration_loop(self, *, cancel_navigation: bool, notify: bool) -> bool:
        thread = None
        with self._exploration_loop_lock:
            self._exploration_loop_stop_event.set()
            self._exploration_loop_generation += 1
            thread = self._exploration_loop_thread
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        stopped = self._platform_exploration_runtime.stop_exploration()
        if cancel_navigation:
            self._cancel_platform_exploration_navigation(current_goal=current_goal)
        if stopped:
            self._notify_compat_exploration_stopped()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.2, self._exploration_loop_interval_sec() * 2.0))
        if notify:
            self._notify_exploration_update(self.get_platform_exploration_state())
        return stopped

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
        if waiting_for:
            self._cancel_platform_exploration_navigation()
            self._platform_exploration_runtime.block_on_prerequisite(
                waiting_for=str(waiting_for),
                message=str(runtime_metadata.get("waiting_detail") or "探索前置条件未就绪。"),
                failure_code=str(runtime_metadata.get("failure_code") or "exploration_prerequisite_unavailable"),
                metadata=runtime_metadata,
            )
            return
        current_goal_before_tick = self._platform_exploration_runtime.get_current_navigation_goal()
        if current_goal_before_tick is not None and self._frontier_goal_is_stale(current_goal_before_tick, runtime_metadata):
            self._cancel_platform_exploration_navigation(current_goal=current_goal_before_tick)
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
            state = navigation_backend.get_navigation_state()
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
        navigation_state = navigation_backend.get_navigation_state()
        if navigation_state.current_goal_id != current_goal.goal_id:
            if navigation_state.current_goal_id and navigation_state.status not in {
                NavigationStatus.IDLE,
                NavigationStatus.SUCCEEDED,
                NavigationStatus.FAILED,
                NavigationStatus.CANCELLED,
            }:
                navigation_backend.cancel_goal()
            accepted = navigation_backend.set_goal(current_goal)
            if not accepted:
                state = navigation_backend.get_navigation_state()
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
            if self._platform_exploration_runtime.is_running:
                self._ensure_platform_exploration_loop_locked()
        state = self._platform_exploration_runtime.get_exploration_state()
        state = self._augment_exploration_state_with_mapping_diagnostics(state)
        metadata = {
            **dict(getattr(state, "metadata", {}) or {}),
            **self._exploration_loop_metadata(tick_source="get_exploration_state"),
        }
        return state.model_copy(update={"metadata": metadata}, deep=True)

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
            message = "等待 RTAB-Map /map 首帧。"
            failure_code = "map_unavailable"
        elif not bool(rtabmap_readiness.get("streaming_ready")):
            waiting_for = "live_map_streaming"
            message = str(rtabmap_readiness.get("message") or live_diagnostics.get("message") or "等待 live map streaming。")
            failure_code = "live_map_streaming_unavailable"
        elif not bool(rtabmap_readiness.get("tf_chain", {}).get("ready")):
            waiting_for = "tf"
            message = "等待 RTAB-Map/Nav2 所需 TF 链 ready。"
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
            old_value = str(goal_metadata.get(key) or "")
            new_value = str(metadata.get(key) or "")
            if old_value and new_value and old_value != new_value:
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
    （RTAB-Map/Nav2 侧车或 Go2 SDK），但定位状态由平台状态机管理，
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
                "last_error": "平台几何运行时或 RTAB-Map profile 已禁用。",
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
                        "raw_odom_input": self.runtime._clone_metadata(self.runtime._odometry_publisher.get_status()),
                        "tf_localization_provider": self.runtime._clone_metadata(self.runtime._tf_localization_provider.get_status()),
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
            "tf_localization_provider": self.runtime._clone_metadata(self.runtime._tf_localization_provider.get_status()),
            "live_map_provider": self.runtime._clone_metadata(self.runtime._live_map_provider.get_status()),
        }

    def get_occupancy_grid(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.occupancy_grid
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_occupancy_grid()

    def get_cost_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.cost_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_cost_map()

    def get_semantic_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.semantic_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_semantic_map()

    def set_goal(self, goal):
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.set_goal(goal)

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
            message = "平台几何运行时未绑定导航后端。"
            if policy.get("selection") == "removed_platform_grid":
                message = "platform_grid 自定义导航执行链已删除；请配置并使用 Nav2 action 导航。"
            return NavigationState(
                status=NavigationStatus.IDLE,
                message=message,
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "nav2_action_active": bool(policy.get("nav2_action_active")),
                    "platform_grid_active": bool(policy.get("platform_grid_active")),
                    "platform_grid_removed": bool(policy.get("platform_grid_removed")),
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
