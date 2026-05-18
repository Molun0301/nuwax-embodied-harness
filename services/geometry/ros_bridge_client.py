"""nuwax_ros_bridge 子进程生命周期管理。

负责 spawn / monitor / stop Rust gRPC 子进程，
提供健康检查和崩溃自动重启。
"""
from __future__ import annotations

import logging
import os
import signal
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
        self._last_error: Optional[str] = None

    @property
    def grpc_target(self) -> str:
        return "127.0.0.1:%d" % self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def start(self) -> bool:
        """启动 Rust 子进程并等待 gRPC server 就绪。"""
        with self._lock:
            if self._started:
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
        ros_setup = "/opt/ros/%s/setup.bash" % self._ros_distro
        cmd = "source %s && exec %s" % (ros_setup, self._binary_path)

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
