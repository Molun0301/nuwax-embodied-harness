"""nuwax_geometry_runtime attach-only 客户端生命周期管理。

进程唯一 owner 是 ``scripts/systemd/start_geometry_runtime.sh``（由 systemd 或
``scripts/start_gateway.sh`` 调用）。Python 只挂接并健康检查现有 gRPC runtime，
不 spawn、不重启、也不终止外部进程。

本文件替代旧的子进程生命周期管理模块，
本仓库 Phase -0.5 选型后 nuwax-ros-bridge Rust crate 已改名为
`nuwax-geometry-runtime`（同步仓库名、二进制名、ready file 命名空间）。
调用方统一 import 本文件。

设计要点（与原模块一致）：

* attach 模式：端口探测 + gRPC 验活双门（参见 grpc-attach-zombie-bridge-2026-06-27）
* ROS2 残留相关 env（RMW_IMPLEMENTATION / CYCLONEDDS_URI / NUWAX_ROS_BRIDGE_*）已删除：
  本服务不再 spawn 任何 ROS2 节点 / DDS 话题订阅；所有 ROS2 接触面均已迁出
  （见 docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/01_接触面详细盘点.md）。
"""
from __future__ import annotations

import logging
import os
import socket
import threading
import time
from typing import Any, Optional

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.geometry_runtime_client")

_DEFAULT_PORT = 50051
_CANONICAL_LAUNCHER = "scripts/systemd/start_geometry_runtime.sh"
# attach 状态下端口探测到 listen 后，做一次 gRPC 健康检查（GetStatus）的缓存 TTL（秒）。
# Runtime monitor 轮询 is_running 时复用 TTL，避免每次状态读取都发 RPC。
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
    """兼容旧类名的 attach-only nuwax-geometry-runtime 客户端。"""

    def __init__(
        self,
        binary_path: Optional[str] = None,
        port: int = _DEFAULT_PORT,
        **_legacy_kwargs: Any,
    ) -> None:
        # 历史兼容：旧调用方可能传 ros_distro / 等 ROS2 概念参数；
        # 现 nuwax-geometry-runtime 不再依赖 ROS2 distro，按 **_legacy_kwargs 吞掉即可。
        if _legacy_kwargs:
            LOGGER.debug(
                "GeometryRuntimeSubprocess 已忽略历史参数: %s",
                sorted(_legacy_kwargs.keys()),
            )
        if binary_path:
            LOGGER.debug(
                "attach-only client 忽略 binary_path=%s；进程由 %s 管理",
                binary_path,
                _CANONICAL_LAUNCHER,
            )
        self._port = port
        self._lock = threading.RLock()
        self._started = False
        self._attached = False
        self._last_error: Optional[str] = None
        # attach 状态下 gRPC 健康检查缓存（runtime monitor / is_running 复用）
        self._attached_health_check_at = 0.0
        self._attached_health_check_ok = False

    @property
    def grpc_target(self) -> str:
        return "127.0.0.1:%d" % self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            if not self._attached:
                return False
            if not self._is_port_open():
                self._reset_attached_state_locked()
                return False
            return self._check_attached_health_locked()

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
        """挂接由 canonical launcher 启动且能响应 GetStatus 的 runtime。"""

        with self._lock:
            if self._started:
                if self._attached and self._check_attached_health_locked(force=True):
                    return True
                self._reset_attached_state_locked()
            if not self._is_port_open():
                self._last_error = (
                    "nuwax-geometry-runtime 未监听 127.0.0.1:%d；"
                    "请由 %s 启动 canonical runtime" % (self._port, _CANONICAL_LAUNCHER)
                )
                LOGGER.error(self._last_error)
                return False
            if not self._verify_attached_runtime_locked():
                self._last_error = (
                    "127.0.0.1:%d 已监听但 geometry GetStatus 验活失败；"
                    "请检查 %s 的 owner 日志" % (self._port, _CANONICAL_LAUNCHER)
                )
                LOGGER.error(self._last_error)
                self._reset_attached_state_locked()
                return False
            self._attached = True
            self._started = True
            self._attached_health_check_at = time.monotonic()
            self._attached_health_check_ok = True
            self._last_error = None
            LOGGER.info(
                "nuwax-geometry-runtime 已在端口 %d 运行，gRPC 验活通过，Python 只挂接",
                self._port,
            )
            self._warn_if_video_relay_missing()
            return True

    def stop(self) -> None:
        """释放 Python 本地挂接；外部 runtime 生命周期归 canonical owner。"""

        with self._lock:
            self._started = False
            self._reset_attached_state_locked()

    def disconnect(self) -> None:
        """仅清空 attach/started 状态与健康检查缓存，不杀进程也不动监听 socket。

        供 GeometryRuntimeProvider.connect() 失败后调用，让下一次 ``start()``
        重新执行端口 + gRPC 双门验活；不向外部 owner 发送信号。
        """
        with self._lock:
            self._reset_attached_state_locked()
            self._started = False

    def _is_port_open(self) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", self._port), timeout=0.25):
                return True
        except OSError:
            return False

    def _verify_attached_runtime_locked(self) -> bool:
        """attach 候选进程是否真的能响应 gRPC GetStatus。

        用一次性 insecure_channel + GetStatus RPC 直接打 127.0.0.1:port。
        成功 → 返回 True（可挂接）。失败/超时/拒绝 → 返回 False。
        """
        if _SKIP_ATTACH_HEALTH_CHECK:
            return True
        try:
            import grpc  # 局部 import 避免拉起整个 gRPC 栈（无 grpc 也能跑非 gRPC 路径）
            from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
            from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
        except Exception as exc:  # pragma: no cover - 部署损坏时 fail-closed
            LOGGER.error("attach 验活: gRPC proto 导入失败: %s", exc)
            return False
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

        TTL 内复用上次结果，避免 runtime monitor 轮询时每轮都打 RPC。
        force=True 跳过缓存（仅用于 start() 顶部 zombie 自愈路径）。
        失败时清空 attach/started 状态，让下次 start() 重新执行双门验活。
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
            LOGGER.warning("attach 状态下 gRPC 健康检查失败，清状态等待 canonical owner 恢复")
            self._reset_attached_state_locked()
        return ok

    def _reset_attached_state_locked(self) -> None:
        """调用方必须已持有 self._lock。清空 attach 状态与健康检查缓存。"""
        self._started = False
        self._attached = False
        self._attached_health_check_at = 0.0
        self._attached_health_check_ok = False
