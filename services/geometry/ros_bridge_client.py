"""nuwax_ros_bridge 子进程生命周期管理。

负责 spawn / monitor / stop Rust gRPC 子进程，
提供健康检查和崩溃自动重启。
"""
from __future__ import annotations

import logging
import os
import signal
import shlex
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.ros_bridge_client")

_DEFAULT_BINARY_NAME = "nuwax-ros-bridge"
_DEFAULT_PORT = 50051
_STARTUP_TIMEOUT_SEC = 10.0
_HEALTH_CHECK_INTERVAL_SEC = 2.0
_STOP_TIMEOUT_SEC = 5.0
_MAX_RESTART_ATTEMPTS = 3
_RESTART_BACKOFF_BASE_SEC = 1.0


class RosBridgeSubprocess:
    """管理 nuwax-ros-bridge Rust 子进程的生命周期。"""

    def __init__(
        self,
        binary_path: Optional[str] = None,
        port: int = _DEFAULT_PORT,
        ros_distro: str = "foxy",
    ) -> None:
        self._binary_path = self._resolve_binary(binary_path)
        self._port = port
        self._ros_distro = ros_distro
        self._process: Optional[subprocess.Popen] = None
        self._ready_file: Optional[str] = None
        self._lock = threading.RLock()
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._restart_count = 0
        self._started = False
        self._attached = False
        self._last_error: Optional[str] = None

    @property
    def grpc_target(self) -> str:
        return "127.0.0.1:%d" % self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            if self._attached:
                return self._is_port_open()
            return self._process is not None and self._process.poll() is None

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def _warn_if_video_relay_missing(self) -> None:
        """挂接已有 bridge 时探测视频中转产物，缺失则告警（不阻断 geometry）。

        若挂接到的旧 bridge 未启用 video_relay，则 /tmp/nuwax_video.sock 与
        .shm 不存在，感知视频源将永远 available=False 且静默失败，故显式告警。
        """
        shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
        uds_path = os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock")
        shm_ok = os.path.exists(shm_path)
        uds_ok = os.path.exists(uds_path)
        if not (shm_ok and uds_ok):
            LOGGER.warning(
                "挂接现有 nuwax-ros-bridge，但视频中转产物缺失 (shm=%s exists=%s, uds=%s exists=%s)；"
                "该 bridge 可能未启用 video_relay，持续视频感知将不可用。"
                "请重启 bridge 或确认 video_streamer 已连接。",
                shm_path,
                shm_ok,
                uds_path,
                uds_ok,
            )

    def start(self) -> bool:
        """启动 Rust 子进程并等待 gRPC server 就绪。"""
        with self._lock:
            if self._started:
                return True
            if self._is_port_open():
                self._attached = True
                self._started = True
                LOGGER.info("nuwax-ros-bridge 已在端口 %d 运行，挂接现有进程", self._port)
                self._warn_if_video_relay_missing()
                return True
            self._stop_event.clear()
            self._restart_count = 0
            success = self._spawn_process()
            if success:
                self._started = True
                self._start_monitor_thread()
            return success

    def stop(self) -> None:
        """优雅停止子进程。"""
        with self._lock:
            self._started = False
            self._attached = False
            self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
        self._kill_process()

    def _spawn_process(self) -> bool:
        """fork 子进程并等待 ready 文件出现。"""
        pid_tag = os.getpid()
        self._ready_file = os.path.join("/tmp", "nuwax_ros_bridge_%d.ready" % pid_tag)
        if os.path.exists(self._ready_file):
            os.unlink(self._ready_file)

        env = os.environ.copy()
        env["NUWAX_ROS_BRIDGE_PORT"] = str(self._port)
        env["NUWAX_ROS_BRIDGE_READY_FILE"] = self._ready_file
        self._apply_bridge_env_aliases(env)
        # 使用 CycloneDDS RMW 以直接订阅 Go2 原生 DDS 话题
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        cyclonedds_xml = os.path.expanduser("~/cyclonedds_ws/cyclonedds.xml")
        if os.path.exists(cyclonedds_xml):
            env["CYCLONEDDS_URI"] = cyclonedds_xml
        ros_setup = "/opt/ros/%s/setup.bash" % self._ros_distro
        cyclonedds_setup = os.path.expanduser("~/cyclonedds_ws/install/setup.bash")
        source_scripts = [ros_setup]
        if os.path.exists(cyclonedds_setup):
            source_scripts.append(cyclonedds_setup)
        external_env_script = self._resolve_external_env_script()
        if external_env_script is not None:
            source_scripts.append(str(external_env_script))
        source_cmd = " && ".join("source %s" % shlex.quote(script) for script in source_scripts)
        cmd = "%s && exec %s" % (source_cmd, shlex.quote(self._binary_path))

        try:
            self._process = subprocess.Popen(
                ["bash", "-c", cmd],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as e:
            self._last_error = "启动子进程失败: %s" % e
            LOGGER.error(self._last_error)
            return False

        deadline = time.monotonic() + _STARTUP_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                stderr = self._process.stderr.read().decode(errors="replace") if self._process.stderr else ""
                self._last_error = "子进程启动后立即退出 (code=%s): %s" % (self._process.returncode, stderr[:200])
                LOGGER.error(self._last_error)
                return False
            if os.path.exists(self._ready_file):
                LOGGER.info("nuwax-ros-bridge 就绪, pid=%d, port=%d", self._process.pid, self._port)
                self._attached = False
                return True
            time.sleep(0.1)

        self._last_error = "子进程启动超时 (%ss)，ready 文件未出现" % _STARTUP_TIMEOUT_SEC
        LOGGER.error(self._last_error)
        self._kill_process()
        return False

    def _kill_process(self) -> None:
        """终止子进程：SIGTERM → 等待 → SIGKILL。"""
        with self._lock:
            proc = self._process
            if self._attached:
                self._attached = False
                self._process = None
                return
            if proc is None:
                return
            if proc.poll() is not None:
                self._process = None
                return

        try:
            proc.terminate()
            try:
                proc.wait(timeout=_STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                LOGGER.warning("子进程 SIGTERM 超时，发送 SIGKILL")
                proc.kill()
                proc.wait(timeout=2.0)
        except OSError:
            pass

        with self._lock:
            self._process = None

        if self._ready_file and os.path.exists(self._ready_file):
            try:
                os.unlink(self._ready_file)
            except OSError:
                pass

    def _start_monitor_thread(self) -> None:
        """启动后台监控线程。"""
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="ros_bridge_monitor",
            daemon=True,
        )
        self._monitor_thread.start()

    def _monitor_loop(self) -> None:
        """监控子进程存活状态，崩溃时自动重启。"""
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=_HEALTH_CHECK_INTERVAL_SEC)
            if self._stop_event.is_set():
                break
            with self._lock:
                if not self._started:
                    break
                if self._process is None:
                    break
                ret = self._process.poll()
            if ret is not None:
                LOGGER.error("nuwax-ros-bridge 子进程异常退出 (code=%s)", ret)
                if self._restart_count >= _MAX_RESTART_ATTEMPTS:
                    self._last_error = f"子进程连续崩溃 {_MAX_RESTART_ATTEMPTS} 次，放弃重启"
                    LOGGER.error(self._last_error)
                    with self._lock:
                        self._started = False
                    break
                backoff = _RESTART_BACKOFF_BASE_SEC * (2 ** self._restart_count)
                self._restart_count += 1
                LOGGER.info("第 %d 次重启，等待 %.1fs", self._restart_count, backoff)
                self._stop_event.wait(timeout=backoff)
                if self._stop_event.is_set():
                    break
                success = self._spawn_process()
                if not success:
                    LOGGER.error("重启失败: %s", self._last_error)

    def _is_port_open(self) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", self._port), timeout=0.25):
                return True
        except OSError:
            return False

    @staticmethod
    def _apply_bridge_env_aliases(env: dict) -> None:
        aliases = {
            "NUWAX_MAP_TOPIC": ("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map"),
            "NUWAX_TF_TOPIC": ("NUWAX_GEOMETRY_TF_TOPIC", "/tf"),
            "NUWAX_NAV_ACTION": ("NUWAX_NAV2_ACTION_NAME", "/navigate_to_pose"),
            "NUWAX_GO2_ODOM_SOURCE": ("NUWAX_GEOMETRY_ODOM_TOPIC", "/utlidar/robot_odom"),
            "NUWAX_GO2_POINTCLOUD_SOURCE": ("NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "/utlidar/cloud"),
            "NUWAX_CLOUD_MAP_TOPIC": ("NUWAX_GEOMETRY_CLOUD_MAP_TOPIC", ""),
        }
        for target, (source, default_value) in aliases.items():
            if str(env.get(target) or "").strip():
                continue
            value = str(env.get(source) or "").strip() or default_value
            env[target] = value

    @staticmethod
    def _resolve_external_env_script() -> Optional[Path]:
        configured = os.environ.get("NUWAX_ROS_BRIDGE_ENV_SCRIPT")
        if configured:
            path = Path(configured).expanduser()
            return path if path.exists() else None
        external_script = os.environ.get("NUWAX_GEOMETRY_EXTERNAL_SCRIPT")
        if not external_script:
            return None
        candidate = Path(external_script).expanduser().resolve().parent / "scripts" / "env.sh"
        return candidate if candidate.exists() else None

    @staticmethod
    def _resolve_binary(binary_path: Optional[str]) -> str:
        """解析二进制路径。"""
        if binary_path:
            return binary_path
        candidates = [
            Path(__file__).parent / "nuwax_ros_bridge" / "target" / "release" / _DEFAULT_BINARY_NAME,
            Path.home() / "nuwax_ros_bridge" / "target" / "release" / _DEFAULT_BINARY_NAME,
            Path("/usr/local/bin") / _DEFAULT_BINARY_NAME,
        ]
        for p in candidates:
            if p.exists():
                return str(p)
        return _DEFAULT_BINARY_NAME
