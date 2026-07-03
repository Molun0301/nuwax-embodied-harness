"""nuwax_geometry_runtime 子进程生命周期管理。

负责 spawn / monitor / stop Rust gRPC 子进程，提供健康检查和崩溃自动重启。

本文件替代旧的子进程生命周期管理模块，
本仓库 Phase -0.5 选型后 nuwax-ros-bridge Rust crate 已改名为
`nuwax-geometry-runtime`（同步仓库名、二进制名、ready file 命名空间）。
调用方统一 import 本文件。

设计要点（与原模块一致）：

* spawn 子进程并等待 ready 文件
* attach 模式：端口探测 + gRPC 验活双门（参见 grpc-attach-zombie-bridge-2026-06-27）
* 监控线程 2s 一次轮询，崩溃时 backoff 重启，最多 3 次
* ROS2 残留相关 env（RMW_IMPLEMENTATION / CYCLONEDDS_URI / NUWAX_ROS_BRIDGE_*）已删除：
  本服务不再 spawn 任何 ROS2 节点 / DDS 话题订阅；所有 ROS2 接触面均已迁出
  （见 docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/01_接触面详细盘点.md）。
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

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.geometry_runtime_client")

_DEFAULT_BINARY_NAME = "nuwax-geometry-runtime"
_DEFAULT_PORT = 50051
_STARTUP_TIMEOUT_SEC = 10.0
_HEALTH_CHECK_INTERVAL_SEC = 2.0
_STOP_TIMEOUT_SEC = 5.0
_MAX_RESTART_ATTEMPTS = 3
_RESTART_BACKOFF_BASE_SEC = 1.0
# attach 状态下端口探测到 listen 后，做一次 gRPC 健康检查（GetStatus）的缓存 TTL（秒）。
# 监控线程 2s 一次轮询 is_running，TTL 1s 既保证轮询期内不重复 RPC，又能在下一次轮询及时反映故障。
_ATTACHED_HEALTH_CHECK_TTL_SEC = 1.0
# attach 验活/GetStatus 单次 RPC 超时。复用 _read_deadline_sec 默认值 2.0s；
# 单独定义便于测试时压低到 0.5s。
_ATTACHED_HEALTH_CHECK_TIMEOUT_SEC = float(
    os.environ.get("NUWAX_GEOMETRY_RUNTIME_ATTACH_HEALTH_TIMEOUT_SEC", "2.0")
)
# 紧急跳过 attach gRPC 验活的 hotfix 开关；置 1 后退回旧的「仅端口探测」行为。
# 默认关闭：attach 路径必须 RPC 验活，避免挂接已僵死的进程导致后续 UNAVAILABLE 死锁。
_SKIP_ATTACH_HEALTH_CHECK = os.environ.get(
    "NUWAX_GEOMETRY_RUNTIME_SKIP_HEALTH_CHECK", "0"
).strip().lower() in {"1", "true", "yes", "on"}


class GeometryRuntimeSubprocess:
    """管理 nuwax-geometry-runtime Rust 子进程的生命周期。"""

    def __init__(
        self,
        binary_path: Optional[str] = None,
        port: int = _DEFAULT_PORT,
        ros_distro: str = "humble",
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
        # attach 状态下 gRPC 健康检查缓存（monitor_loop / is_running 复用）
        self._attached_health_check_at = 0.0
        self._attached_health_check_ok = False

    @property
    def grpc_target(self) -> str:
        return "127.0.0.1:%d" % self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            if self._attached:
                # attach 状态下不能仅凭端口探测判定可用：进程僵死/被冲掉后端口可能短暂
                # 仍在 LISTEN 或 TIME_WAIT，但任何 RPC 都会立即 UNAVAILABLE，会让上层
                # _start_geometry_runtime_subprocess 误以为"已就绪"而走 fast-return，
                # 留下永久 _started=True 假活锁（参见 grpc-attach-zombie-bridge-2026-06-27）。
                # 这里走带 TTL 缓存的 gRPC 健康检查；失败时清状态允许 respawn。
                if not self._is_port_open():
                    self._reset_attached_state_locked()
                    return False
                return self._check_attached_health_locked()
            return self._process is not None and self._process.poll() is None

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def _warn_if_video_relay_missing(self) -> None:
        """挂接已有 runtime 时探测视频中转产物，缺失则告警（不阻断 geometry）。

        若挂接到的旧 runtime 未启用 video_relay，则 /tmp/nuwax_video.sock 与
        .shm 不存在，感知视频源将永远 available=False 且静默失败，故显式告警。
        """
        shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
        uds_path = os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock")
        shm_ok = os.path.exists(shm_path)
        uds_ok = os.path.exists(uds_path)
        if not (shm_ok and uds_ok):
            LOGGER.warning(
                "挂接现有 nuwax-geometry-runtime，但视频中转产物缺失 (shm=%s exists=%s, uds=%s exists=%s)；"
                "该 runtime 可能未启用 video_relay，持续视频感知将不可用。"
                "请重启 runtime 或确认 video_streamer 已连接。",
                shm_path,
                shm_ok,
                uds_ok,
            )

    def start(self) -> bool:
        """启动 Rust 子进程并等待 gRPC server 就绪。

        attach 路径必须 RPC 验活：仅端口探测不能区分「进程还活着能响应 RPC」
        与「进程僵死/被冲掉但 listen socket 仍在」两种状态。后者若直接挂接，
        后续 GetFrameTree/GetCurrentPose 都会立即 UNAVAILABLE，且 self._started
        永远为 True 让下一次 start() 走 short-circuit 不再 spawn，
        形成永久假活锁（参见 grpc-attach-zombie-bridge-2026-06-27）。
        验活失败则**跳过 attach 继续走 spawn 路径**，让 spawn 取代失败的端口占用。
        """
        with self._lock:
            if self._started:
                # 已启动状态也允许 zombie 自愈：attach 但 RPC 不可用 → 清状态后走 spawn。
                if self._attached and not self._check_attached_health_locked(
                    force=True
                ):
                    LOGGER.warning(
                        "nuwax-geometry-runtime attach 状态健康检查失败，强制清理并重新 spawn"
                    )
                    self._reset_attached_state_locked()
                    self._started = False
                else:
                    return True
            if self._is_port_open():
                if self._verify_attached_runtime_locked():
                    self._attached = True
                    self._started = True
                    self._attached_health_check_at = time.monotonic()
                    self._attached_health_check_ok = True
                    LOGGER.info("nuwax-geometry-runtime 已在端口 %d 运行，gRPC 验活通过，挂接现有进程", self._port)
                    self._warn_if_video_relay_missing()
                    return True
                LOGGER.warning(
                    "端口 %d 已被占用但 gRPC 验活失败（疑似僵死/被冲掉的 nuwax-geometry-runtime 残留），"
                    "跳过 attach 继续走 spawn 路径",
                    self._port,
                )
                self._reset_attached_state_locked()
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
            self._attached_health_check_at = 0.0
            self._attached_health_check_ok = False
            self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
        self._kill_process()

    def disconnect(self) -> None:
        """仅清空 attach/started 状态与健康检查缓存，不杀进程也不动监听 socket。

        供 GeometryRuntimeProvider.connect() 失败后调用：
        - 让下一次 GeometryRuntimeSubprocess.start() 不再 short-circuit，能走到 spawn 路径取代僵死 attach。
        - 不主动 kill 端口：调用方已确认 attach 不可用，spawn 时新进程会重新 bind 同端口
          （依赖 start_gateway.sh cleanup_stale_geometry_runtime_processes 已杀过旧进程），
          或者 spawn 失败时由调用方根据 last_error 决策。
        """
        with self._lock:
            self._reset_attached_state_locked()
            self._started = False

    def _spawn_process(self) -> bool:
        """fork 子进程并等待 ready 文件出现。"""
        pid_tag = os.getpid()
        self._ready_file = os.path.join("/tmp", "nuwax_geometry_runtime_%d.ready" % pid_tag)
        if os.path.exists(self._ready_file):
            os.unlink(self._ready_file)

        env = os.environ.copy()
        env["NUWAX_GEOMETRY_RUNTIME_PORT"] = str(self._port)
        env["NUWAX_GEOMETRY_RUNTIME_READY_FILE"] = self._ready_file
        self._apply_runtime_env_aliases(env)

        try:
            self._process = subprocess.Popen(
                [str(self._binary_path)],
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
                LOGGER.info("nuwax-geometry-runtime 就绪, pid=%d, port=%d", self._process.pid, self._port)
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
            name="geometry_runtime_monitor",
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
                LOGGER.error("nuwax-geometry-runtime 子进程异常退出 (code=%s)", ret)
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

    def _verify_attached_runtime_locked(self) -> bool:
        """attach 候选进程是否真的能响应 gRPC GetStatus。

        用一次性 insecure_channel + GetStatus RPC 直接打 127.0.0.1:port。
        成功 → 返回 True（可挂接）。失败/超时/拒绝 → 返回 False（僵死/被冲掉，spawn 取代）。
        """
        if _SKIP_ATTACH_HEALTH_CHECK:
            return True
        try:
            import grpc  # 局部 import 避免拉起整个 gRPC 栈（无 grpc 也能跑非 gRPC 路径）
            from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
            from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
        except Exception as exc:  # pragma: no cover - 导入失败时降级放行
            LOGGER.warning("attach 验活: gRPC proto 导入失败，降级为端口探测: %s", exc)
            return True
        try:
            channel = grpc.insecure_channel(self.grpc_target)
            try:
                stub = pb2_grpc.GeometryRuntimeStub(channel)
                stub.GetStatus(pb2.Empty(), timeout=_ATTACHED_HEALTH_CHECK_TIMEOUT_SEC)
                return True
            finally:
                channel.close()
        except Exception as exc:
            LOGGER.info("attach 验活: gRPC GetStatus 失败: %s", exc)
            return False

    def _check_attached_health_locked(self, *, force: bool = False) -> bool:
        """带 TTL 缓存的 attach 健康检查。

        TTL 内复用上次结果，避免监控线程 2s 轮询时每轮都打 RPC。
        force=True 跳过缓存（仅用于 start() 顶部 zombie 自愈路径）。
        失败时**清空 attach/started 状态**，让下次 start() 走完整 spawn。
        """
        if _SKIP_ATTACH_HEALTH_CHECK:
            return True
        now = time.monotonic()
        if not force and self._attached_health_check_ok and (
            now - self._attached_health_check_at
        ) < _ATTACHED_HEALTH_CHECK_TTL_SEC:
            return True
        ok = self._verify_attached_runtime_locked()
        self._attached_health_check_at = now
        self._attached_health_check_ok = ok
        if not ok:
            LOGGER.warning(
                "attach 状态下 gRPC 健康检查失败，清状态允许 respawn"
            )
            self._reset_attached_state_locked()
        return ok

    def _reset_attached_state_locked(self) -> None:
        """调用方必须已持有 self._lock。清空 attach 状态与健康检查缓存。"""
        self._attached = False
        self._attached_health_check_at = 0.0
        self._attached_health_check_ok = False

    @staticmethod
    def _apply_runtime_env_aliases(env: dict) -> None:
        """把 Nuwax 顶层语义 env 变量下沉为 nuwax-geometry-runtime 内部变量。

        nuwax-geometry-runtime 子进程不再依赖 ROS2 节点 / DDS 话题（已全部迁出），
        因此这里的 alias 列表不再包含 /utlidar/robot_odom 等 ROS2 话题源：
        设备位姿由 LCM/SHM 直推（参见 docs/.../06_LCM接入与wire-format验证.md）。
        """
        aliases = {
            "NUWAX_MAP_FRAME": ("NUWAX_GEOMETRY_MAP_FRAME", "map"),
            "NUWAX_BASE_FRAME": ("NUWAX_GEOMETRY_BASE_FRAME", "base_link"),
        }
        for target, (source, default_value) in aliases.items():
            if str(env.get(target) or "").strip():
                continue
            value = str(env.get(source) or "").strip() or default_value
            env[target] = value

    @staticmethod
    def _resolve_external_env_script() -> Optional[Path]:
        configured = os.environ.get("NUWAX_GEOMETRY_RUNTIME_ENV_SCRIPT")
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
            Path(__file__).parent / "nuwax_geometry_runtime" / "target" / "release" / _DEFAULT_BINARY_NAME,
            Path.home() / "nuwax_geometry_runtime" / "target" / "release" / _DEFAULT_BINARY_NAME,
            Path("/usr/local/bin") / _DEFAULT_BINARY_NAME,
        ]
        for p in candidates:
            if p.exists():
                return str(p)
        return _DEFAULT_BINARY_NAME
