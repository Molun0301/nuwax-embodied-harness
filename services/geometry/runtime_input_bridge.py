"""Phase 5: Go2 sensor producer → GeometryRuntime 输入桥.

2026-07-07 Phase 5 新增，禁止 silent fallback；provider 不可用时不要假装 ok。

设计目标
========

- 把 ``StreamingLocalizationProvider`` / ``StreamingPointCloudProvider`` 的回调
  真正转译成 ``GeometryRuntimeProvider.publish_odom`` / ``publish_point_cloud``，
  让 mapping / nav / explore 真实 readiness 有数据来源。
- 维护运行时 counters（``published_odom_count`` / ``published_tf_count`` /
  ``published_pointcloud_count`` / ``last_error`` / ``last_publish_latency_ms``），
  便于 Phase 7 mapping_ready_poll 收敛时核对。
- 严格禁止隐性 fallback：

    * ``start()`` 后若 provider 未连，``is_ready()`` 必须返回 ``False``，
      ``RuntimeInputBridge.last_error`` 必须记录原因，不得用缓存占位。
    * 任何单帧发布失败都要计入 ``dropped_count`` 并刷新 ``last_error``，
      不允许把 publisher 未就绪当作已发。

Lifecycle
=========

1. ``ensure_runtime_connected()`` — 等待 ``GeometryRuntimeProvider.is_connected()``。
2. ``start()`` — 注册 provider callback（仅一次）。
3. ``stop()`` — 注销 callback，flush 一帧标记（不发送假数据）。
4. 重复 ``start()`` / ``stop()`` 幂等。

背压策略
========

- odom: 直发，不节流；calling thread 同步走 gRPC。
- pointcloud:

    * 默认阈值 ``MAX_POINTS_BEFORE_DOWNSAMPLE = 100_000``；
    * 大于阈值时使用 ``points_xyz_array`` 降采样到 ``MAX_POINTS_AFTER_DOWNSAMPLE``；
    * 若 ndarray 不可用，使用 ``points`` 列表并发出 ``downsampled=True`` metadata。
    * ``PublishPointCloudRequest`` 携带规范化 XYZ bytes、字段布局、frame 与时间戳；
      若 ``point_count * point_step`` 超过
      ``SHM_PAYLOAD_THRESHOLD_BYTES`` (默认 8 MiB)，调用方应在外层把 bytes
      落到 SHM 并以 ref 形式 publish（本桥仅记录意图，不替换上游实现）。
"""

from __future__ import annotations

import inspect
import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import numpy as np

from contracts.geometry import FrameTree, Pose
from contracts.pointcloud import PointCloudFrame
from providers.localization import StreamingLocalizationProvider
from providers.pointcloud import StreamingPointCloudProvider

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime_input_bridge")

# Phase 5 硬约束：背压与采样阈值（与 §5 / §13.4 对齐）。
MAX_POINTS_BEFORE_DOWNSAMPLE = 100_000
MAX_POINTS_AFTER_DOWNSAMPLE = 50_000
SHM_PAYLOAD_THRESHOLD_BYTES = 8 * 1024 * 1024  # 8 MiB

# 默认 frame_id 兜底：odom 维度。真实 frame_id 应来自 provider。
_DEFAULT_ODOM_FRAME = "odom"

# sensor_msgs/PointField constants for the normalized XYZ float32 payload emitted below.
_CANONICAL_XYZ_FIELDS = (
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1},
)


def _datetime_to_unix_ns(value: datetime) -> int:
    """Convert a contract timestamp exactly at Python's microsecond precision."""
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    utc_value = value.astimezone(timezone.utc)
    epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
    delta = utc_value - epoch
    return (
        (delta.days * 86_400 + delta.seconds) * 1_000_000_000
        + delta.microseconds * 1_000
    )


@dataclass
class BridgeCounters:
    """运行时计数器（counters），用于 readiness / 诊断。"""

    published_odom_count: int = 0
    published_tf_count: int = 0
    published_pointcloud_count: int = 0
    dropped_odom_count: int = 0
    dropped_pointcloud_count: int = 0
    downsampled_pointcloud_count: int = 0
    last_publish_latency_ms: float = 0.0
    last_publish_at_monotonic: Optional[float] = None
    last_error: Optional[str] = None
    last_error_at_monotonic: Optional[float] = None
    last_published_point_count: int = 0
    last_published_point_step: int = 0

    def as_dict(self) -> Dict[str, Any]:
        return {
            "published_odom_count": self.published_odom_count,
            "published_tf_count": self.published_tf_count,
            "published_pointcloud_count": self.published_pointcloud_count,
            "dropped_odom_count": self.dropped_odom_count,
            "dropped_pointcloud_count": self.dropped_pointcloud_count,
            "downsampled_pointcloud_count": self.downsampled_pointcloud_count,
            "last_publish_latency_ms": self.last_publish_latency_ms,
            "last_publish_at_monotonic": self.last_publish_at_monotonic,
            "last_error": self.last_error,
            "last_error_at_monotonic": self.last_error_at_monotonic,
            "last_published_point_count": self.last_published_point_count,
            "last_published_point_step": self.last_published_point_step,
        }


@dataclass
class RuntimeInputBridge:
    """Go2 sensor producer → GeometryRuntime 输入桥。

    参数
    ----
    runtime_provider
        必须实现 ``is_connected()`` 与 ``publish_odom`` / ``publish_point_cloud``；
        推荐传入 ``GeometryRuntimeProvider`` 实例。
    localization_provider
        必须实现 ``subscribe_localization`` / ``unsubscribe_localization``。
    point_cloud_provider
        必须实现 ``subscribe_point_cloud`` / ``unsubscribe_point_cloud``。
    max_points_before_downsample
        点数超过该阈值时触发降采样（默认 ``MAX_POINTS_BEFORE_DOWNSAMPLE``）。
    max_points_after_downsample
        降采样后目标点数上限（默认 ``MAX_POINTS_AFTER_DOWNSAMPLE``）。
    """

    runtime_provider: Any
    localization_provider: Optional[StreamingLocalizationProvider] = None
    point_cloud_provider: Optional[StreamingPointCloudProvider] = None
    max_points_before_downsample: int = MAX_POINTS_BEFORE_DOWNSAMPLE
    max_points_after_downsample: int = MAX_POINTS_AFTER_DOWNSAMPLE

    counters: BridgeCounters = field(default_factory=BridgeCounters)

    # === lifecycle state ===
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False)
    _started: bool = False
    _localization_subscription_id: Optional[int] = None
    _point_cloud_subscription_id: Optional[int] = None
    # Phase 5: 显式记录 provider 不可用 vs runtime 未连，不允许混用同一句 "no provider"。
    _missing_localization_provider: bool = False
    _missing_point_cloud_provider: bool = False
    _runtime_unavailable: bool = False

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def ensure_runtime_connected(self, *, timeout_sec: float = 5.0, poll_interval_sec: float = 0.1) -> bool:
        """等待 runtime gRPC facade 就绪。

        与 start() 的硬顺序：必须先连 runtime，再订 provider；否则 callback 内
        publish_odom 会因为 stub 未建立而 silently drop，破坏 readiness 收敛。
        """
        connected_attr = getattr(self.runtime_provider, "is_connected", None)
        if connected_attr is None:
            self._record_error("runtime_provider.is_connected 不存在")
            with self._lock:
                self._runtime_unavailable = True
            return False

        def is_connected() -> bool:
            current = getattr(self.runtime_provider, "is_connected", False)
            return bool(current() if callable(current) else current)

        # 第一次立即检查：connected=True 时不需要任何 sleep。
        if is_connected():
            with self._lock:
                self._runtime_unavailable = False
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            time.sleep(max(0.0, float(poll_interval_sec)))
            if is_connected():
                with self._lock:
                    self._runtime_unavailable = False
                return True
        with self._lock:
            self._runtime_unavailable = True
        self._record_error(f"runtime gRPC facade 未就绪 timeout={timeout_sec}s")
        return False

    def start(self) -> bool:
        """订 provider callback。

        Returns
        -------
        bool
            ``True`` 表示至少 ``LocalizationProvider`` 订阅成功。pointcloud 不可用
            时不会阻止 ``start()``，但 ``is_ready()`` 在订阅两者都失败时返回 ``False``。
        """
        with self._lock:
            if self._started:
                return True
            if not self.ensure_runtime_connected(timeout_sec=0.0):
                LOGGER.warning("RuntimeInputBridge.start: runtime 未连，不订 provider。")
                return False
            self._started = True
            self._localization_subscription_id = None
            self._point_cloud_subscription_id = None

        localization_ok = self._subscribe_localization()
        point_cloud_ok = self._subscribe_point_cloud()
        with self._lock:
            self._missing_localization_provider = not localization_ok
            self._missing_point_cloud_provider = not point_cloud_ok

        if not localization_ok and not point_cloud_ok:
            self._record_error("LocalizationProvider 与 PointCloudProvider 均不可用")
            LOGGER.warning("RuntimeInputBridge.start: 两个 provider 都不可用。")
            # 不回滚 _started：让调用方通过 is_ready() 看到 False，并显式 stop() 释放。
            return False
        if not localization_ok:
            self._record_error("LocalizationProvider 不可用，odom 不会推送")
        if not point_cloud_ok:
            self._record_error("PointCloudProvider 不可用，pointcloud 不会推送")
        return True

    def stop(self) -> None:
        """注销 callback，刷新 counters 标记。"""
        with self._lock:
            if not self._started:
                return
            self._started = False
            loc_id = self._localization_subscription_id
            pc_id = self._point_cloud_subscription_id
            self._localization_subscription_id = None
            self._point_cloud_subscription_id = None

        if loc_id is not None and self.localization_provider is not None:
            try:
                self.localization_provider.unsubscribe_localization(loc_id)
            except Exception as exc:
                LOGGER.debug("注销 LocalizationProvider 订阅失败：%s", exc)
        if pc_id is not None and self.point_cloud_provider is not None:
            try:
                self.point_cloud_provider.unsubscribe_point_cloud(pc_id)
            except Exception as exc:
                LOGGER.debug("注销 PointCloudProvider 订阅失败：%s", exc)

    def is_started(self) -> bool:
        with self._lock:
            return self._started

    def is_ready(self) -> bool:
        """综合就绪状态：runtime 已连 + 至少 LocalizationProvider 订阅成功。"""
        with self._lock:
            if not self._started:
                return False
            if self._runtime_unavailable:
                return False
            if self._missing_localization_provider:
                return False
            return True

    # ------------------------------------------------------------------
    # subscription helpers
    # ------------------------------------------------------------------

    def _subscribe_localization(self) -> bool:
        if self.localization_provider is None:
            return False
        subscriber = getattr(self.localization_provider, "subscribe_localization", None)
        if not callable(subscriber):
            return False
        try:
            subscription_id = subscriber(self._on_localization)
        except Exception as exc:
            self._record_error(f"LocalizationProvider.subscribe_localization 抛异常：{exc}")
            return False
        if subscription_id is None:
            self._record_error("LocalizationProvider.subscribe_localization 返回 None")
            return False
        with self._lock:
            self._localization_subscription_id = int(subscription_id)
        return True

    def _subscribe_point_cloud(self) -> bool:
        if self.point_cloud_provider is None:
            return False
        subscriber = getattr(self.point_cloud_provider, "subscribe_point_cloud", None)
        if not callable(subscriber):
            return False
        try:
            subscription_id = subscriber(self._on_point_cloud)
        except Exception as exc:
            self._record_error(f"PointCloudProvider.subscribe_point_cloud 抛异常：{exc}")
            return False
        if subscription_id is None:
            self._record_error("PointCloudProvider.subscribe_point_cloud 返回 None")
            return False
        with self._lock:
            self._point_cloud_subscription_id = int(subscription_id)
        return True

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------

    def _on_localization(
        self,
        pose: Pose,
        frame_tree: Optional[FrameTree],
        metadata: Optional[Dict[str, Any]],
    ) -> None:
        if not self.is_started():
            return
        if pose is None:
            self.counters.dropped_odom_count += 1
            self._record_error("localization callback 收到 None pose")
            return

        publisher = getattr(self.runtime_provider, "publish_odom", None)
        if not callable(publisher):
            self.counters.dropped_odom_count += 1
            self._record_error("runtime_provider.publish_odom 不存在")
            return

        frame_id = str(getattr(pose, "frame_id", "") or _DEFAULT_ODOM_FRAME)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        pos_x = float(getattr(position, "x", 0.0))
        pos_y = float(getattr(position, "y", 0.0))
        pos_z = float(getattr(position, "z", 0.0))
        qx = float(getattr(orientation, "x", 0.0))
        qy = float(getattr(orientation, "y", 0.0))
        qz = float(getattr(orientation, "z", 0.0))
        qw = float(getattr(orientation, "w", 1.0))
        timestamp = getattr(pose, "timestamp", None)
        timestamp_ns = (
            _datetime_to_unix_ns(timestamp)
            if isinstance(timestamp, datetime)
            else 0
        )

        start = time.monotonic()
        try:
            args = (pos_x, pos_y, pos_z, qx, qy, qz, qw)
            metadata = {"frame_id": frame_id, "timestamp_ns": timestamp_ns}
            try:
                inspect.signature(publisher).bind(*args, **metadata)
            except (TypeError, ValueError):
                success = bool(publisher(*args, frame_id=frame_id))
            else:
                success = bool(publisher(*args, **metadata))
        except Exception as exc:
            self.counters.dropped_odom_count += 1
            self._record_error(f"publish_odom 抛异常：{exc}")
            return
        latency_ms = (time.monotonic() - start) * 1000.0
        if not success:
            self.counters.dropped_odom_count += 1
            self._record_error("publish_odom 返回 False（runtime 未连或 RPC 失败）")
            return

        self.counters.published_odom_count += 1
        self.counters.last_publish_latency_ms = latency_ms
        self.counters.last_publish_at_monotonic = time.monotonic()

        # FrameTree 当前 gRPC facade 没有 PublishTf 接口；这里只记账，
        # 真实 TF publish 走 Phase 4 启用的 lcm_runtime 内部订阅 + /tf channel。
        if frame_tree is not None and getattr(frame_tree, "transforms", None):
            self.counters.published_tf_count += 1

    def _on_point_cloud(self, point_cloud: PointCloudFrame) -> None:
        if not self.is_started():
            return
        if point_cloud is None:
            self.counters.dropped_pointcloud_count += 1
            self._record_error("point_cloud callback 收到 None")
            return

        publisher = getattr(self.runtime_provider, "publish_point_cloud", None)
        if not callable(publisher):
            self.counters.dropped_pointcloud_count += 1
            self._record_error("runtime_provider.publish_point_cloud 不存在")
            return

        # 背压：点 > 阈值时降采样（保留 ndarray 路径）。
        downsampled = False
        original_count = int(getattr(point_cloud, "point_count", 0) or 0)
        payload = point_cloud
        if original_count > self.max_points_before_downsample:
            payload = self._downsample_point_cloud(point_cloud)
            downsampled = True
            self.counters.downsampled_pointcloud_count += 1

        point_step = self._resolve_point_step(payload)
        bytes_payload = self._encode_point_cloud_bytes(payload)
        if bytes_payload is None:
            self.counters.dropped_pointcloud_count += 1
            self._record_error(
                f"PointCloud 编码失败：frame={payload.frame_id} count={payload.point_count}"
            )
            return

        start = time.monotonic()
        try:
            timestamp = getattr(payload, "timestamp", None)
            timestamp_ns = (
                _datetime_to_unix_ns(timestamp)
                if isinstance(timestamp, datetime)
                else 0
            )
            metadata = {
                "frame_id": str(getattr(payload, "frame_id", "") or "map"),
                "timestamp_ns": timestamp_ns,
                "fields": _CANONICAL_XYZ_FIELDS,
                "point_count": len(bytes_payload) // point_step,
                "is_dense": bool(getattr(payload, "is_dense", True)),
            }
            try:
                inspect.signature(publisher).bind(bytes_payload, point_step, **metadata)
            except (TypeError, ValueError):
                # Compatibility for pre-extension provider implementations. The real
                # GeometryRuntimeProvider accepts metadata; legacy test/extension
                # providers still receive the original two positional arguments.
                result = publisher(bytes_payload, point_step)
            else:
                result = publisher(bytes_payload, point_step, **metadata)
        except Exception as exc:
            self.counters.dropped_pointcloud_count += 1
            self._record_error(f"publish_point_cloud 抛异常：{exc}")
            return
        latency_ms = (time.monotonic() - start) * 1000.0

        success = bool(result)
        if not success:
            self.counters.dropped_pointcloud_count += 1
            self._record_error("publish_point_cloud 返回 False（runtime 未连或 RPC 失败）")
            return

        self.counters.published_pointcloud_count += 1
        self.counters.last_publish_latency_ms = latency_ms
        self.counters.last_publish_at_monotonic = time.monotonic()
        self.counters.last_published_point_count = int(payload.point_count)
        self.counters.last_published_point_step = int(point_step)

        if downsampled:
            LOGGER.debug(
                "PointCloud 降采样推送：original=%d after=%d step=%d",
                original_count,
                payload.point_count,
                point_step,
            )

    # ------------------------------------------------------------------
    # payload helpers
    # ------------------------------------------------------------------

    def _downsample_point_cloud(self, point_cloud: PointCloudFrame) -> PointCloudFrame:
        target = self.max_points_after_downsample
        arr = getattr(point_cloud, "points_xyz_array", None)
        if isinstance(arr, np.ndarray) and arr.ndim == 2 and arr.shape[0] > target:
            stride = max(1, int(np.ceil(arr.shape[0] / float(target))))
            downsampled = arr[::stride][:target]
            try:
                return point_cloud.model_copy(
                    update={
                        "point_count": int(downsampled.shape[0]),
                        "points_xyz_array": downsampled,
                        "metadata": {
                            **dict(getattr(point_cloud, "metadata", {}) or {}),
                            "downsampled": True,
                            "downsample_target": int(target),
                        },
                    },
                    deep=False,
                )
            except Exception:
                LOGGER.debug("PointCloud ndarray 降采样后 model_copy 失败，回退 list 路径。", exc_info=True)
        # list fallback
        points = list(getattr(point_cloud, "points", []) or [])
        if not points:
            return point_cloud
        stride = max(1, int(np.ceil(len(points) / float(target))))
        sampled = points[::stride][:target]
        try:
            return point_cloud.model_copy(
                update={
                    "point_count": len(sampled),
                    "points": sampled,
                    "metadata": {
                        **dict(getattr(point_cloud, "metadata", {}) or {}),
                        "downsampled": True,
                        "downsample_target": int(target),
                    },
                },
                deep=False,
            )
        except Exception:
            return point_cloud

    @staticmethod
    def _resolve_point_step(point_cloud: PointCloudFrame) -> int:
        # sensor_msgs/PointCloud2 标准字段：x/y/z float32 = 12 bytes；
        # 本桥不引入 fields 推断逻辑，避免对 PointField metadata 假设过强。
        return 12

    @staticmethod
    def _encode_point_cloud_bytes(point_cloud: PointCloudFrame) -> Optional[bytes]:
        """把 PointCloudFrame 编码为 ``sensor_msgs/PointCloud2`` 兼容的 little-endian 字节。

        当前 Rust 端 ``encode_pointcloud_payload`` 期望 ``point_step(u32 LE) + raw bytes``，
        因此本桥只产出 ``raw bytes``（不含 4-byte point_step），由调用方在
        ``publish_point_cloud(bytes, point_step)`` 内合并；这里避免重复打包。
        """
        arr = getattr(point_cloud, "points_xyz_array", None)
        if isinstance(arr, np.ndarray) and arr.ndim == 2 and arr.shape[1] >= 3:
            # 强制 float32 little-endian 与 sensor_msgs/PointCloud2 对齐
            if arr.dtype != np.float32:
                arr = arr.astype(np.float32, copy=False)
            return np.ascontiguousarray(arr[:, :3]).tobytes()
        points = getattr(point_cloud, "points", None) or []
        if not points:
            return None
        flat = np.empty((len(points), 3), dtype=np.float32)
        for idx, point in enumerate(points):
            flat[idx, 0] = float(getattr(point, "x", 0.0))
            flat[idx, 1] = float(getattr(point, "y", 0.0))
            flat[idx, 2] = float(getattr(point, "z", 0.0))
        return np.ascontiguousarray(flat).tobytes()

    # ------------------------------------------------------------------
    # status / diagnostics
    # ------------------------------------------------------------------

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "started": self._started,
                "ready": self.is_ready(),
                "runtime_unavailable": self._runtime_unavailable,
                "missing_localization_provider": self._missing_localization_provider,
                "missing_point_cloud_provider": self._missing_point_cloud_provider,
                "localization_subscription_id": self._localization_subscription_id,
                "point_cloud_subscription_id": self._point_cloud_subscription_id,
                "counters": self.counters.as_dict(),
            }

    def _record_error(self, message: str) -> None:
        with self._lock:
            self.counters.last_error = message
            self.counters.last_error_at_monotonic = time.monotonic()
        LOGGER.debug("RuntimeInputBridge: %s", message)


def build_default_runtime_input_bridge(
    *,
    runtime_provider: Any,
    localization_provider: Optional[StreamingLocalizationProvider] = None,
    point_cloud_provider: Optional[StreamingPointCloudProvider] = None,
) -> RuntimeInputBridge:
    """工厂方法，集中默认值，便于测试与生产链路复用。"""
    return RuntimeInputBridge(
        runtime_provider=runtime_provider,
        localization_provider=localization_provider,
        point_cloud_provider=point_cloud_provider,
    )


__all__ = [
    "BridgeCounters",
    "MAX_POINTS_BEFORE_DOWNSAMPLE",
    "MAX_POINTS_AFTER_DOWNSAMPLE",
    "SHM_PAYLOAD_THRESHOLD_BYTES",
    "RuntimeInputBridge",
    "build_default_runtime_input_bridge",
]
