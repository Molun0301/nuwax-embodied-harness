"""机器人标准传感器与设备定位流到 GeometryRuntime 的输入桥。

设计目标
========

- 把 ``StreamingLocalizationProvider`` / ``StreamingPointCloudProvider`` 的回调
  真正转译成 ``GeometryRuntimeProvider.publish_odom`` / ``publish_point_cloud``，
  让 mapping / nav / explore 真实 readiness 有数据来源。
- 原生算法优先消费 ``StreamingSensorSampleProvider`` 的规范 LiDAR/IMU 数据，
  不要求机器人自行实现定位，也不重复订阅显示数据流。
- 维护运行时 counters（``published_odom_count`` / ``observed_frame_tree_count`` /
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
2. ``start()`` — 原生模式先建立独立发布线程，再注册 provider callback（仅一次）。
3. ``stop()`` — 停止接收、注销 callback，等待已接收回调及在途发布退出并回收线程。
4. 重复 ``start()`` / ``stop()`` 幂等。

背压策略
========

- odom: 直发，不节流；calling thread 同步走 gRPC。
- 原生 LiDAR/IMU：回调组装后进入既有四帧队列，由独立单拥有方线程顺序发布；
  队列满时承接旧帧惯性，停止取消未领取帧，不占用传感器读取线程等待发布。
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
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Deque, Dict, Optional, Tuple

import numpy as np

from adapters.sensors.lio_measurement import (
    DEFAULT_MAX_IMU_SAMPLES,
    LioMeasurementAssembler,
    LioMeasurementAssemblerError,
)
from contracts.geometry import FrameTree, Pose
from contracts.pointcloud import PointCloudFrame
from contracts.sensors import (
    CanonicalImuSample,
    CanonicalLidarFrame,
    CanonicalLioMeasurement,
    RegistrationState,
)
from core.logging_utils import LogRateLimiter
from providers.localization import StreamingLocalizationProvider
from providers.pointcloud import StreamingPointCloudProvider
from providers.sensors import StreamingSensorSampleProvider

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime_input_bridge")

# Phase 5 硬约束：背压与采样阈值（与 §5 / §13.4 对齐）。
MAX_POINTS_BEFORE_DOWNSAMPLE = 100_000
MAX_POINTS_AFTER_DOWNSAMPLE = 50_000
SHM_PAYLOAD_THRESHOLD_BYTES = 8 * 1024 * 1024  # 8 MiB

# 默认 frame_id 兜底：odom 维度。真实 frame_id 应来自 provider。
_DEFAULT_ODOM_FRAME = "odom"
_LIO_BACKEND_ENV = "NUWAX_GEOMETRY_LIO_BACKEND"
_DEVICE_LIO_BACKEND = "device"
_LIO_SENSOR_FRAME_ENV = "NUWAX_GEOMETRY_LIDAR_FRAME"
_DEFAULT_LIO_SENSOR_FRAME = "utlidar_lidar"
_MAX_LIO_PUBLISH_QUEUE_SIZE = 4
_APPLIED_TRANSFORM_PROVENANCE_PREFIX = "calibration_applied:"
_INPUT_ERROR_LOG_INTERVAL_SEC = 5.0
# 仅汇总已知逐样本重复诊断，固定键集合不会随任意异常正文增长。
_REPEATED_INPUT_ERROR_KEYS = {
    "lio_not_ready: IMU 尚未覆盖 LiDAR 扫描开始与末端，保留有界等待窗口": "imu_coverage_pending",
    "time_regression: IMU source_sequence 和 timestamp_ns 必须严格递增": "imu_time_regression",
    "time_regression: LiDAR source_sequence 和 timestamp_ns 必须严格递增": "lidar_time_regression",
}

# 下游发布使用的标准 PointField 布局；可选字段只在真实存在时进入字段表。
_CANONICAL_XYZ_FIELDS = (
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1},
)
_CANONICAL_XYZI_FIELDS = _CANONICAL_XYZ_FIELDS + (
    {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
)
_CANONICAL_XYZ_OFFSET_FIELDS = _CANONICAL_XYZ_FIELDS + (
    {"name": "offset_ns", "offset": 12, "datatype": 6, "count": 1},
)
_GO2_L2_FIELDS = _CANONICAL_XYZI_FIELDS + (
    {"name": "offset_ns", "offset": 16, "datatype": 6, "count": 1},
)

# 点云布局固定；将 dtype（数据类型描述）对象提升到模块级，避免每帧重复构造。
_XYZ_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
)
_XYZI_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")]
)
_XYZ_OFFSET_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")]
)
_GO2_L2_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("offset_ns", "<u4"),
    ]
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


@dataclass(frozen=True)
class LioImuSample:
    timestamp_ns: int
    frame_id: str
    accel_mps2: Tuple[float, float, float]
    gyro_rps: Tuple[float, float, float]


@dataclass
class BridgeCounters:
    """运行时计数器（counters），用于 readiness / 诊断。"""

    published_odom_count: int = 0
    observed_frame_tree_count: int = 0
    # 兼容旧诊断键。Python bridge 不拥有 TF publisher，因此该值恒为 0。
    published_tf_count: int = 0
    published_pointcloud_count: int = 0
    emitted_lio_measurement_count: int = 0
    lio_not_ready_count: int = 0
    dropped_odom_count: int = 0
    dropped_pointcloud_count: int = 0
    dropped_lio_imu_count: int = 0
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
            "observed_frame_tree_count": self.observed_frame_tree_count,
            "published_tf_count": self.published_tf_count,
            "tf_owner": "rust_canonical",
            "tf_observation_semantics": "observed_not_published",
            "published_pointcloud_count": self.published_pointcloud_count,
            "emitted_lio_measurement_count": self.emitted_lio_measurement_count,
            "lio_not_ready_count": self.lio_not_ready_count,
            "dropped_odom_count": self.dropped_odom_count,
            "dropped_pointcloud_count": self.dropped_pointcloud_count,
            "dropped_lio_imu_count": self.dropped_lio_imu_count,
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
    """机器人标准传感器与定位流到 GeometryRuntime 的输入桥。

    参数
    ----
    runtime_provider
        必须实现 ``is_connected()`` 与 ``publish_odom`` / ``publish_point_cloud``；
        推荐传入 ``GeometryRuntimeProvider`` 实例。
    localization_provider
        必须实现 ``subscribe_localization`` / ``unsubscribe_localization``。
    point_cloud_provider
        必须实现 ``subscribe_point_cloud`` / ``unsubscribe_point_cloud``。
    sensor_sample_provider
        原生模式的标准 LiDAR/IMU 输入；提供时不要求设备定位能力。
    max_points_before_downsample
        点数超过该阈值时触发降采样（默认 ``MAX_POINTS_BEFORE_DOWNSAMPLE``）。
    max_points_after_downsample
        降采样后目标点数上限（默认 ``MAX_POINTS_AFTER_DOWNSAMPLE``）。
    """

    runtime_provider: Any
    localization_provider: Optional[StreamingLocalizationProvider] = None
    point_cloud_provider: Optional[StreamingPointCloudProvider] = None
    sensor_sample_provider: Optional[StreamingSensorSampleProvider] = None
    max_points_before_downsample: int = MAX_POINTS_BEFORE_DOWNSAMPLE
    max_points_after_downsample: int = MAX_POINTS_AFTER_DOWNSAMPLE

    counters: BridgeCounters = field(default_factory=BridgeCounters)
    _error_log_limiter: LogRateLimiter = field(default_factory=LogRateLimiter, init=False, repr=False)
    _lio_measurement_assembler: LioMeasurementAssembler = field(
        default_factory=LioMeasurementAssembler,
        init=False,
        repr=False,
    )
    _lio_condition: threading.Condition = field(
        default_factory=threading.Condition,
        init=False,
        repr=False,
    )
    _lio_publish_queue: Deque[Tuple[int, CanonicalLioMeasurement]] = field(
        default_factory=deque,
        init=False,
        repr=False,
    )
    _lio_generation: int = field(default=0, init=False)
    _lio_accepting: bool = field(default=False, init=False)
    _lio_sensor_frame: str = field(
        default=_DEFAULT_LIO_SENSOR_FRAME,
        init=False,
    )
    _lio_publish_owner_active: bool = field(default=False, init=False)
    _lio_publish_owner_thread_id: Optional[int] = field(default=None, init=False)
    _lio_publisher_thread: Optional[threading.Thread] = field(default=None, init=False, repr=False)
    _native_lio_enabled: bool = field(default=False, init=False)
    _lio_in_flight_count: int = field(default=0, init=False)
    _callback_in_flight_count: int = field(default=0, init=False)
    _callback_in_flight_by_thread: Dict[int, int] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _lio_reset_pending_generation: Optional[int] = field(default=None, init=False)
    _point_cloud_publisher_signature_key: Optional[Tuple[int, int]] = field(
        default=None,
        init=False,
        repr=False,
    )
    _point_cloud_publisher_accepts_metadata: Optional[bool] = field(
        default=None,
        init=False,
        repr=False,
    )

    # === lifecycle state ===
    _lifecycle_operation_lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
    )
    _counter_lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
    )
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False)
    _started: bool = False
    _localization_subscription_id: Optional[int] = None
    _point_cloud_subscription_id: Optional[int] = None
    _sensor_sample_subscription_id: Optional[int] = None
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
            ``True`` 表示至少一条输入订阅成功；算法能力就绪由 Rust 中的实际数据决定。
        """
        current_thread_id = threading.get_ident()
        while True:
            with self._lio_condition:
                if self._lio_reset_pending_generation is not None:
                    if (
                        self._callback_in_flight_by_thread.get(
                            current_thread_id,
                            0,
                        )
                        > 0
                        or self._lio_publish_owner_thread_id
                        == current_thread_id
                    ):
                        self._record_error(
                            "拒绝在尚未退出的 provider callback 内重启 "
                            "RuntimeInputBridge"
                        )
                        return False
                    while self._lio_reset_pending_generation is not None:
                        if self._reset_lio_assembler_if_quiescent_locked():
                            break
                        self._lio_condition.wait()
            with self._lifecycle_operation_lock:
                with self._lio_condition:
                    if self._lio_reset_pending_generation is not None:
                        continue
                    previous_generation = self._lio_generation
                started = self._start_new_generation_locked()
                rollback = None
                if not started and self._native_lio_enabled and self._lio_generation != previous_generation:
                    # 锁内撤销本次启动，锁外等待；发布线程也可能重入 stop 取得此锁。
                    self.stop(_wait_for_drain=False)
                    with self._lio_condition:
                        rollback = (self._lio_generation, self._lio_publisher_thread)
            if rollback is not None:
                self._wait_for_stop_drain(*rollback)
            return started

    def _start_new_generation_locked(self) -> bool:
        """在 lifecycle operation lock 内启动一个新的 bridge generation。"""

        selected_lio_backend = os.environ.get(
            _LIO_BACKEND_ENV, _DEVICE_LIO_BACKEND
        )
        if selected_lio_backend not in {
            _DEVICE_LIO_BACKEND,
            "fastlio2",
            "pointlio",
        }:
            self._record_error(
                f"拒绝未知 {_LIO_BACKEND_ENV}={selected_lio_backend!r} "
                "的 provider forwarding"
            )
            return False
        with self._lock:
            if self._started:
                return True
        # 重入停止的最后释放者只标记排空；新代际开始前仍须回收实际线程句柄。
        with self._lio_condition:
            previous_publisher = self._lio_publisher_thread
        self._join_native_lio_publisher(previous_publisher)
        with self._lock:
            if not self.ensure_runtime_connected(timeout_sec=0.0):
                LOGGER.warning(
                    "RuntimeInputBridge.start: runtime 未连，不订 provider。"
                )
                return False
            self._started = True
            self._localization_subscription_id = None
            self._point_cloud_subscription_id = None
        with self._lio_condition:
            self._lio_generation += 1
            lio_generation = self._lio_generation
            self._lio_accepting = True
            self._lio_reset_pending_generation = None
            self._lio_publish_owner_thread_id = None
            self._native_lio_enabled = selected_lio_backend != _DEVICE_LIO_BACKEND
            self._lio_sensor_frame = os.environ.get(
                _LIO_SENSOR_FRAME_ENV,
                _DEFAULT_LIO_SENSOR_FRAME,
            ).strip()
            self._lio_publish_queue.clear()
            self._lio_measurement_assembler.reset()

        if self._native_lio_enabled:
            try:
                self._start_native_lio_publisher(lio_generation)
            except Exception as exc:
                self._record_error("原生输入发布线程启动失败：%s" % exc)
                return False

        if self.sensor_sample_provider is None:
            for candidate in (self.point_cloud_provider, self.localization_provider):
                if isinstance(candidate, StreamingSensorSampleProvider):
                    self.sensor_sample_provider = candidate
                    break
        native_samples = (
            selected_lio_backend != _DEVICE_LIO_BACKEND
            and self.sensor_sample_provider is not None
        )
        if native_samples:
            # 标准 LiDAR/IMU 流是原生算法的唯一源，不要求机器人自产位姿。
            sensor_ok = self._subscribe_sensor_samples(lio_generation)
            localization_ok = False
            point_cloud_ok = sensor_ok
        else:
            sensor_ok = False
            localization_ok = self._subscribe_localization(lio_generation)
            with self._lio_condition:
                generation_current = self._lio_callback_is_current(lio_generation)
            # 只在重入 stop 后跳过后续订阅，定位能力缺失不应禁用点云。
            point_cloud_ok = (
                self._subscribe_point_cloud(lio_generation) if generation_current else False
            )
        with self._lock:
            started_after_subscribe = self._started
        with self._lio_condition:
            generation_after_subscribe = (
                self._lio_generation == lio_generation
                and self._lio_accepting
            )
        if not started_after_subscribe or not generation_after_subscribe:
            localization_ok = False
            point_cloud_ok = False
        with self._lock:
            self._missing_localization_provider = not localization_ok
            self._missing_point_cloud_provider = not point_cloud_ok

        if not localization_ok and not point_cloud_ok:
            self._record_error(
                "LocalizationProvider 与 PointCloudProvider 均不可用"
            )
            LOGGER.warning("RuntimeInputBridge.start: 两个 provider 都不可用。")
            # 网关不会接管启动失败的桥，原生模式必须自行回收新增的发布线程。
            # start 在锁外完成原生回滚排空；设备模式仍由显式 stop 释放。
            return False
        if not localization_ok and not sensor_ok:
            self._record_error("LocalizationProvider 不可用，odom 不会推送")
        if not point_cloud_ok:
            self._record_error("PointCloudProvider 不可用，pointcloud 不会推送")
        return True

    def stop(self, *, _wait_for_drain: bool = True) -> None:
        """注销 callback；外部调用等待 drain，callback 重入由最后 release 延迟 reset。"""
        stopping_thread_id = threading.get_ident()
        with self._lock:
            started_snapshot = self._started
        with self._lio_condition:
            pending_stop = self._lio_reset_pending_generation is not None
            current_callback_count = (
                self._callback_in_flight_by_thread.get(
                    stopping_thread_id,
                    0,
                )
            )
            current_owns_publish = (
                self._lio_publish_owner_active
                and self._lio_publish_owner_thread_id
                == stopping_thread_id
            )
        if (
            not started_snapshot
            and pending_stop
            and (current_callback_count > 0 or current_owns_publish)
        ):
            return

        with self._lifecycle_operation_lock:
            with self._lock:
                transitioned = self._started
                if transitioned:
                    self._started = False
                    loc_id = self._localization_subscription_id
                    pc_id = self._point_cloud_subscription_id
                    sensor_id = self._sensor_sample_subscription_id
                    self._localization_subscription_id = None
                    self._point_cloud_subscription_id = None
                    self._sensor_sample_subscription_id = None
                else:
                    loc_id = None
                    pc_id = None
                    sensor_id = None

            if transitioned:
                with self._lio_condition:
                    self._lio_accepting = False
                    self._lio_generation += 1
                    self._lio_reset_pending_generation = (
                        self._lio_generation
                    )
                    self._lio_publish_queue.clear()
                    self._lio_condition.notify_all()

                if sensor_id is not None and self.sensor_sample_provider is not None:
                    try:
                        self.sensor_sample_provider.unsubscribe_sensor_sample(sensor_id)
                    except Exception as exc:
                        LOGGER.debug("注销标准传感器订阅失败：%s", exc)
                if (
                    loc_id is not None
                    and self.localization_provider is not None
                ):
                    try:
                        self.localization_provider.unsubscribe_localization(
                            loc_id
                        )
                    except Exception as exc:
                        LOGGER.debug(
                            "注销 LocalizationProvider 订阅失败：%s",
                            exc,
                        )
                if (
                    pc_id is not None
                    and self.point_cloud_provider is not None
                ):
                    try:
                        self.point_cloud_provider.unsubscribe_point_cloud(
                            pc_id
                        )
                    except Exception as exc:
                        LOGGER.debug(
                            "注销 PointCloudProvider 订阅失败：%s",
                            exc,
                        )

            with self._lio_condition:
                stopped_generation = self._lio_generation
                stopped_publisher = self._lio_publisher_thread

        with self._lio_condition:
            current_callback_count = (
                self._callback_in_flight_by_thread.get(
                    stopping_thread_id,
                    0,
                )
            )
            current_owns_publish = (
                self._lio_publish_owner_active
                and self._lio_publish_owner_thread_id
                == stopping_thread_id
            )
            should_drain = (
                self._lio_reset_pending_generation is not None
                or self._lio_publish_owner_active
                or self._lio_in_flight_count > 0
                or self._callback_in_flight_count > 0
                or self._lio_publisher_thread is not None
            )
        if not _wait_for_drain or current_callback_count > 0 or current_owns_publish:
            return
        if should_drain:
            self._wait_for_stop_drain(stopped_generation, stopped_publisher)

    def _wait_for_stop_drain(self, generation: int, publisher: Optional[threading.Thread]) -> None:
        """只等待本次停止的代际与句柄，不能误等并发重启后的空闲线程。"""

        with self._lio_condition:
            while self._lio_generation == generation and (
                self._lio_publish_owner_active
                or self._lio_in_flight_count > 0
                or self._callback_in_flight_count > 0
            ):
                self._lio_condition.wait()
            if self._lio_generation == generation:
                self._reset_lio_assembler_if_quiescent_locked()
        self._join_native_lio_publisher(publisher)

    def _start_native_lio_publisher(self, generation: int) -> None:
        """订阅前建立当前代际唯一发布拥有方，启动失败不留下活跃哨兵。"""
        worker = threading.Thread(
            target=self._drain_native_lio_measurements,
            args=(generation,),
            name="nuwax-native-lio-publisher-%d" % generation,
            daemon=True,
        )
        with self._lio_condition:
            self._lio_publisher_thread = worker
            self._lio_publish_owner_active = True
        try:
            worker.start()
        except Exception:
            with self._lio_condition:
                self._lio_publisher_thread = None
                self._lio_publish_owner_active = False
                self._lio_publish_owner_thread_id = None
                self._lio_condition.notify_all()
            raise

    def _join_native_lio_publisher(self, worker: Optional[threading.Thread]) -> None:
        """在状态/条件锁外等待旧线程真正退出；重入拥有方不得等待自身。"""
        if worker is None or worker is threading.current_thread():
            return
        worker.join()
        with self._lio_condition:
            if self._lio_publisher_thread is worker:
                self._lio_publisher_thread = None
            self._lio_condition.notify_all()

    def is_started(self) -> bool:
        with self._lock:
            return self._started

    def is_ready(self) -> bool:
        """输入桥就绪：运行时已连接且至少有一条有效输入订阅。"""
        with self._lock:
            if not self._started:
                return False
            if self._runtime_unavailable:
                return False
            return any(subscription is not None for subscription in (
                self._localization_subscription_id,
                self._point_cloud_subscription_id,
                self._sensor_sample_subscription_id,
            ))

    # ------------------------------------------------------------------
    # subscription helpers
    # ------------------------------------------------------------------

    def _subscribe_sensor_samples(self, generation: int) -> bool:
        provider = self.sensor_sample_provider
        if provider is None:
            return False
        try:
            subscription_id = provider.subscribe_sensor_sample(
                lambda sample: self._on_sensor_sample(sample, generation)
            )
        except Exception as exc:
            self._record_error("标准传感器订阅失败：%s" % exc)
            return False
        if subscription_id is None:
            return False
        with self._lio_condition:
            current = self._lio_callback_is_current(generation)
        if not current:
            provider.unsubscribe_sensor_sample(subscription_id)
            return False
        with self._lock:
            self._sensor_sample_subscription_id = subscription_id
        return True

    def _on_sensor_sample(self, sample: Any, generation: int) -> None:
        if not isinstance(sample, (CanonicalImuSample, CanonicalLidarFrame)):
            return
        if not self._admit_bridge_callback(generation):
            return
        try:
            with self._lio_condition:
                if not self._lio_callback_is_current(generation):
                    return
                try:
                    measurement = (
                        self._lio_measurement_assembler.add_imu(sample)
                        if isinstance(sample, CanonicalImuSample)
                        else self._lio_measurement_assembler.add_lidar(sample)
                    )
                except LioMeasurementAssemblerError as exc:
                    self._record_lio_assembler_error(exc)
                    return
                self._enqueue_ready_lio_measurements(measurement, generation)
        finally:
            self._release_bridge_callback()

    def _subscribe_localization(self, lio_generation: int) -> bool:
        if self.localization_provider is None:
            return False
        subscriber = getattr(self.localization_provider, "subscribe_localization", None)
        if not callable(subscriber):
            return False

        def callback(
            pose: Pose,
            frame_tree: Optional[FrameTree],
            metadata: Optional[Dict[str, Any]],
        ) -> None:
            self._on_localization(
                pose,
                frame_tree,
                metadata,
                _lio_generation=lio_generation,
            )

        try:
            subscription_id = subscriber(callback)
        except Exception as exc:
            self._record_error(f"LocalizationProvider.subscribe_localization 抛异常：{exc}")
            return False
        if subscription_id is None:
            self._record_error("LocalizationProvider.subscribe_localization 返回 None")
            return False
        with self._lock:
            generation_current = self._started
            generation_current = generation_current and (
                self._localization_subscription_id is None
                or self._localization_subscription_id == int(subscription_id)
            )
        with self._lio_condition:
            generation_current = generation_current and (
                self._lio_generation == lio_generation
                and self._lio_accepting
            )
        if not generation_current:
            try:
                self.localization_provider.unsubscribe_localization(
                    int(subscription_id)
                )
            except Exception as exc:
                LOGGER.debug(
                    "清理失效 LocalizationProvider 订阅失败：%s",
                    exc,
                )
            return False
        with self._lock:
            self._localization_subscription_id = int(subscription_id)
        return True

    def _subscribe_point_cloud(self, lio_generation: int) -> bool:
        if self.point_cloud_provider is None:
            return False
        subscriber = getattr(self.point_cloud_provider, "subscribe_point_cloud", None)
        if not callable(subscriber):
            return False

        def callback(point_cloud: PointCloudFrame) -> None:
            self._on_point_cloud(
                point_cloud,
                _lio_generation=lio_generation,
            )

        try:
            subscription_id = subscriber(callback)
        except Exception as exc:
            self._record_error(f"PointCloudProvider.subscribe_point_cloud 抛异常：{exc}")
            return False
        if subscription_id is None:
            self._record_error("PointCloudProvider.subscribe_point_cloud 返回 None")
            return False
        with self._lock:
            generation_current = self._started
            generation_current = generation_current and (
                self._point_cloud_subscription_id is None
                or self._point_cloud_subscription_id == int(subscription_id)
            )
        with self._lio_condition:
            generation_current = generation_current and (
                self._lio_generation == lio_generation
                and self._lio_accepting
            )
        if not generation_current:
            try:
                self.point_cloud_provider.unsubscribe_point_cloud(
                    int(subscription_id)
                )
            except Exception as exc:
                LOGGER.debug(
                    "清理失效 PointCloudProvider 订阅失败：%s",
                    exc,
                )
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
        *,
        _lio_generation: Optional[int] = None,
        _callback_admitted: bool = False,
    ) -> None:
        if not _callback_admitted:
            if not self._admit_bridge_callback(_lio_generation):
                return
            try:
                self._on_localization(
                    pose,
                    frame_tree,
                    metadata,
                    _lio_generation=_lio_generation,
                    _callback_admitted=True,
                )
            finally:
                self._release_bridge_callback()
            return
        callback_metadata = dict(metadata or {})
        native_lio_backend = self._native_lio_enabled
        if native_lio_backend:
            canonical_imu = callback_metadata.get("canonical_imu_sample")
            if isinstance(canonical_imu, CanonicalImuSample):
                with self._lio_condition:
                    if not self._lio_callback_is_current(_lio_generation):
                        return
                    try:
                        measurement = self._lio_measurement_assembler.add_imu(
                            canonical_imu
                        )
                    except LioMeasurementAssemblerError as exc:
                        self._record_lio_assembler_error(exc)
                    else:
                        self._enqueue_ready_lio_measurements(
                            measurement,
                            self._lio_generation,
                        )
            else:
                self._increment_counter("lio_not_ready_count")
                self._record_error(
                    "lio_not_ready: 原生定位回调缺少真实 canonical_imu_sample，"
                    "不能回退到设备里程计"
                )
            # 原生后端拥有里程计输出；旧定位订阅仅作为真实惯性输入载体。
            return

        if pose is None:
            self._increment_counter("dropped_odom_count")
            self._record_error("localization callback 收到 None pose")
            return

        if frame_tree is not None and getattr(frame_tree, "transforms", None):
            self._increment_counter("observed_frame_tree_count")

        publisher = getattr(self.runtime_provider, "publish_odom", None)
        if not callable(publisher):
            self._increment_counter("dropped_odom_count")
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
            self._increment_counter("dropped_odom_count")
            self._record_error(f"publish_odom 抛异常：{exc}")
            return
        latency_ms = (time.monotonic() - start) * 1000.0
        if not success:
            self._increment_counter("dropped_odom_count")
            self._record_error("publish_odom 返回 False（runtime 未连或 RPC 失败）")
            return

        self._increment_counter("published_odom_count")
        self._update_counters(
            last_publish_latency_ms=latency_ms,
            last_publish_at_monotonic=time.monotonic(),
        )

        # TF 发布权属于 Rust canonical owner；Python 仅记录输入侧观测。

    def _on_point_cloud(
        self,
        point_cloud: PointCloudFrame,
        *,
        _lio_generation: Optional[int] = None,
        _callback_admitted: bool = False,
    ) -> None:
        if not _callback_admitted:
            if not self._admit_bridge_callback(_lio_generation):
                return
            try:
                self._on_point_cloud(
                    point_cloud,
                    _lio_generation=_lio_generation,
                    _callback_admitted=True,
                )
            finally:
                self._release_bridge_callback()
            return
        if point_cloud is None:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("point_cloud callback 收到 None")
            return

        point_cloud_metadata = dict(getattr(point_cloud, "metadata", {}) or {})
        raw_frame_id = str(getattr(point_cloud, "frame_id", "") or "")
        frame_id = raw_frame_id.strip()
        if not frame_id or frame_id != raw_frame_id:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error(
                "拒绝缺失或带首尾空白的 PointCloud frame_id："
                f"frame_id={raw_frame_id or '<empty>'!r}"
            )
            return
        canonical_lidar = point_cloud_metadata.get("canonical_lidar_frame")
        has_raw_sensor_payload = isinstance(canonical_lidar, CanonicalLidarFrame)
        legacy_registered_metadata = point_cloud_metadata.get("source") in {
            "direct_pointcloud",
            "go2_direct_dds_pointcloud",
        }
        native_lio_backend = self._native_lio_enabled
        if has_raw_sensor_payload or legacy_registered_metadata:
            source_frame = (
                canonical_lidar.frame_id
                if has_raw_sensor_payload
                else str(point_cloud_metadata.get("source_frame") or "").strip()
            )
            registered_frame = str(
                point_cloud_metadata.get("registered_frame") or ""
            ).strip()
            registered_to_odom = (
                point_cloud_metadata.get("point_cloud_xyz_registered_to_odom") is True
            )
            if native_lio_backend:
                frame_id = source_frame
            else:
                if not registered_to_odom:
                    self._increment_counter("dropped_pointcloud_count")
                    self._record_error(
                        "拒绝 Go2 未注册传感器帧点云："
                        f"source_frame={source_frame or frame_id or '<empty>'}; "
                        "需要可靠外参在上游完成 XYZ 注册并显式声明"
                    )
                    return
                if not registered_frame or frame_id != registered_frame:
                    self._increment_counter("dropped_pointcloud_count")
                    self._record_error(
                        "拒绝 Go2 点云注册断言不一致："
                        f"source_frame={source_frame or '<empty>'} "
                        f"frame_id={frame_id or '<empty>'} "
                        f"registered_frame={registered_frame or '<empty>'}"
                    )
                    return

        if native_lio_backend:
            with self._lio_condition:
                if not self._lio_callback_is_current(_lio_generation):
                    return
                if not isinstance(canonical_lidar, CanonicalLidarFrame):
                    self._increment_counter("dropped_pointcloud_count")
                    self._increment_counter("lio_not_ready_count")
                    self._record_error(
                        "lio_not_ready: native LIO 点云缺少 canonical LiDAR 帧"
                    )
                    return
                try:
                    measurement = self._lio_measurement_assembler.add_lidar(
                        canonical_lidar
                    )
                except LioMeasurementAssemblerError as exc:
                    self._increment_counter("dropped_pointcloud_count")
                    self._record_lio_assembler_error(exc)
                    return
                if measurement is None:
                    self._increment_counter("lio_not_ready_count")
                    self._record_error(
                        "lio_not_ready: IMU 尚未覆盖 LiDAR 扫描开始与末端，"
                        "保留有界等待窗口"
                    )
                    return
                self._enqueue_ready_lio_measurements(
                    measurement,
                    self._lio_generation,
                )
            return

        imu_samples = []

        publisher = getattr(self.runtime_provider, "publish_point_cloud", None)
        if not callable(publisher):
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("runtime_provider.publish_point_cloud 不存在")
            return

        # canonical LiDAR 是 BODY registration，仅供 native LIO；非 native/device
        # 仍沿上游已注册的 odom 点云发布，不能让 BODY bytes 冒充 odom。
        downsampled = False
        original_count = int(getattr(point_cloud, "point_count", 0) or 0)
        if isinstance(canonical_lidar, CanonicalLidarFrame):
            payload_metadata = dict(getattr(point_cloud, "metadata", {}) or {})
            payload_metadata.pop("canonical_lidar_frame", None)
            payload = point_cloud.model_copy(
                update={"metadata": payload_metadata},
                deep=False,
            )
        else:
            payload = point_cloud
        if original_count > self.max_points_before_downsample:
            payload = self._downsample_point_cloud(payload)
            downsampled = True
            self._increment_counter("downsampled_pointcloud_count")

        point_step, fields = self._resolve_point_layout(payload)
        bytes_payload = self._encode_point_cloud_bytes(payload)
        if bytes_payload is None:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error(
                f"PointCloud 编码失败：frame={payload.frame_id} count={payload.point_count}"
            )
            return

        start = time.monotonic()
        try:
            timestamp = getattr(payload, "timestamp", None)
            source_timestamp_ns = point_cloud_metadata.get("source_timestamp_ns")
            if source_timestamp_ns is not None:
                timestamp_ns = int(source_timestamp_ns)
                if timestamp_ns < 0:
                    raise ValueError("source_timestamp_ns 不能为负数")
            else:
                timestamp_ns = (
                    _datetime_to_unix_ns(timestamp)
                    if isinstance(timestamp, datetime)
                    else 0
                )
            metadata = {
                "frame_id": frame_id,
                "timestamp_ns": timestamp_ns,
                "fields": fields,
                "point_count": len(bytes_payload) // point_step,
                "is_dense": bool(getattr(payload, "is_dense", True)),
                "imu_samples": imu_samples,
            }
            if not self._publisher_accepts_metadata(publisher):
                raise TypeError(
                    "pointcloud publisher 必须接受 "
                    "frame_id/timestamp_ns/fields/point_count/is_dense metadata"
                )
            result = publisher(bytes_payload, point_step, **metadata)
        except Exception as exc:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error(f"publish_point_cloud 抛异常：{exc}")
            return
        latency_ms = (time.monotonic() - start) * 1000.0

        success = bool(result)
        if not success:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("publish_point_cloud 返回 False（runtime 未连或 RPC 失败）")
            return

        self._increment_counter("published_pointcloud_count")
        self._update_counters(
            last_publish_latency_ms=latency_ms,
            last_publish_at_monotonic=time.monotonic(),
            last_published_point_count=int(payload.point_count),
            last_published_point_step=int(point_step),
        )

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

    def _admit_bridge_callback(self, generation: Optional[int]) -> bool:
        """原子登记当前代 callback，供 stop 按 delivery thread 排空。"""

        callback_thread_id = threading.get_ident()
        with self._lio_condition:
            if not self._lio_callback_is_current(generation):
                return False
            self._callback_in_flight_count += 1
            self._callback_in_flight_by_thread[callback_thread_id] = (
                self._callback_in_flight_by_thread.get(callback_thread_id, 0)
                + 1
            )
            return True

    def _release_bridge_callback(self) -> None:
        callback_thread_id = threading.get_ident()
        with self._lio_condition:
            thread_count = self._callback_in_flight_by_thread.get(
                callback_thread_id,
                0,
            )
            if thread_count <= 0 or self._callback_in_flight_count <= 0:
                self._record_error(
                    "RuntimeInputBridge callback in-flight 释放不匹配"
                )
                self._lio_condition.notify_all()
                return
            if thread_count == 1:
                self._callback_in_flight_by_thread.pop(
                    callback_thread_id,
                    None,
                )
            else:
                self._callback_in_flight_by_thread[callback_thread_id] = (
                    thread_count - 1
                )
            self._callback_in_flight_count -= 1
            self._reset_lio_assembler_if_quiescent_locked()
            self._lio_condition.notify_all()

    def _reset_lio_assembler_if_quiescent_locked(self) -> bool:
        """在 condition 内延迟完成 stop reset，避免重入 callback 等待自身。"""

        if (
            self._lio_reset_pending_generation is None
            or self._lio_reset_pending_generation != self._lio_generation
            or self._lio_publish_owner_active
            or self._lio_in_flight_count > 0
            or self._callback_in_flight_count > 0
        ):
            return False
        self._lio_measurement_assembler.reset()
        self._lio_reset_pending_generation = None
        return True

    def _increment_counter(self, name: str, amount: int = 1) -> None:
        with self._counter_lock:
            setattr(self.counters, name, getattr(self.counters, name) + amount)

    def _update_counters(self, **values: Any) -> None:
        with self._counter_lock:
            for name, value in values.items():
                setattr(self.counters, name, value)

    def _prepare_native_lio_measurement(
        self,
        measurement: CanonicalLioMeasurement,
    ) -> Optional[CanonicalLioMeasurement]:
        """验证 raw RPC 的 frame/provenance，并执行 canonical 有界采样。"""

        calibration_ref = measurement.lidar.calibration_ref
        expected_provenance = (
            _APPLIED_TRANSFORM_PROVENANCE_PREFIX + calibration_ref
            if calibration_ref
            and calibration_ref == calibration_ref.strip()
            else None
        )
        if measurement.lidar.registration_state != RegistrationState.SENSOR and (
            expected_provenance is None
            or measurement.lidar.transform_provenance
            != expected_provenance
        ):
            self._increment_counter("dropped_pointcloud_count")
            self._increment_counter("lio_not_ready_count")
            self._record_error(
                "lio_not_ready: canonical LiDAR 缺少与 calibration_ref "
                "精确绑定的已应用坐标变换 provenance；拒绝 raw/unapplied "
                "payload 进入 native raw RPC"
            )
            return None

        registration_frame = measurement.lidar.registration_frame_id
        expected_frame = self._lio_sensor_frame
        if (
            not expected_frame
            or registration_frame != expected_frame
            or measurement.lidar.frame_id != registration_frame
            or any(
                sample.frame_id != registration_frame
                for sample in measurement.imu_samples
            )
        ):
            self._increment_counter("dropped_pointcloud_count")
            self._increment_counter("lio_not_ready_count")
            self._record_error(
                "lio_not_ready: canonical payload 尚未变换到共享且已配置的 "
                "registration frame；现有 native raw RPC 不承载 "
                "calibration/registration 变换"
            )
            return None
        if (
            measurement.lidar.point_count <= self.max_points_before_downsample
            or measurement.lidar.point_count <= self.max_points_after_downsample
        ):
            return measurement
        downsampled = self._downsample_canonical_lidar(measurement.lidar)
        if downsampled is None:
            self._increment_counter("dropped_pointcloud_count")
            self._increment_counter("lio_not_ready_count")
            self._record_error(
                "lio_not_ready: canonical LiDAR 无法在保留扫描末端语义时有界降采样"
            )
            return None
        self._increment_counter("downsampled_pointcloud_count")
        return CanonicalLioMeasurement(
            lidar=downsampled,
            imu_samples=measurement.imu_samples,
            scan_begin_timestamp_ns=measurement.scan_begin_timestamp_ns,
            scan_end_timestamp_ns=measurement.scan_end_timestamp_ns,
        )

    def _downsample_canonical_lidar(
        self,
        lidar: CanonicalLidarFrame,
    ) -> Optional[CanonicalLidarFrame]:
        """同步采样 XYZ/intensity/time，并强制保留首尾点与最大时间偏移。"""

        if (
            lidar.xyz_f32_le is None
            or lidar.intensity_f32_le is None
            or lidar.time_offset_ns_le is None
            or lidar.point_count <= 0
        ):
            return None
        target = min(self.max_points_after_downsample, lidar.point_count)
        if target <= 0:
            return None
        try:
            xyz = np.frombuffer(lidar.xyz_f32_le, dtype="<f4").reshape((-1, 3))
            intensity = np.frombuffer(lidar.intensity_f32_le, dtype="<f4")
            offsets = np.frombuffer(lidar.time_offset_ns_le, dtype="<u8")
            required = {
                0,
                lidar.point_count - 1,
                int(np.argmax(offsets)),
            }
            if len(required) > target:
                return None
            selected = set(
                int(index)
                for index in np.linspace(
                    0,
                    lidar.point_count - 1,
                    num=target,
                    dtype=np.int64,
                )
            )
            selected.update(required)
            for index in sorted(selected - required, reverse=True):
                if len(selected) <= target:
                    break
                selected.remove(index)
            if len(selected) < target:
                for index in range(lidar.point_count):
                    selected.add(index)
                    if len(selected) == target:
                        break
            indices = np.asarray(sorted(selected), dtype=np.int64)
            values = lidar.model_dump()
            values.update(
                point_count=int(indices.size),
                xyz_f32_le=np.ascontiguousarray(xyz[indices], dtype="<f4").tobytes(),
                intensity_f32_le=np.ascontiguousarray(
                    intensity[indices], dtype="<f4"
                ).tobytes(),
                time_offset_ns_le=np.ascontiguousarray(
                    offsets[indices], dtype="<u8"
                ).tobytes(),
            )
            if lidar.ring_u16_le is not None:
                ring = np.frombuffer(lidar.ring_u16_le, dtype="<u2")
                values["ring_u16_le"] = np.ascontiguousarray(
                    ring[indices], dtype="<u2"
                ).tobytes()
            return CanonicalLidarFrame(**values)
        except (TypeError, ValueError):
            return None

    def _enqueue_ready_lio_measurements(
        self,
        measurement: Optional[CanonicalLioMeasurement],
        generation: int,
    ) -> None:
        """在条件锁内组装到既有有界队列，唤醒独立发布拥有方。"""

        while measurement is not None:
            prepared = self._prepare_native_lio_measurement(measurement)
            if prepared is not None:
                self._lio_publish_queue.append((generation, prepared))
                if len(self._lio_publish_queue) > _MAX_LIO_PUBLISH_QUEUE_SIZE:
                    dropped_generation, dropped = self._lio_publish_queue.popleft()
                    self._increment_counter("dropped_pointcloud_count")
                    self._record_error(
                        "native LIO publish FIFO 已满，丢弃最旧点云并承接其 IMU"
                    )
                    self._carry_dropped_lio_imu(dropped.imu_samples, dropped_generation)
            measurement = self._lio_measurement_assembler.pop_ready()
        if self._lio_publish_queue:
            self._lio_condition.notify_all()

    def _carry_dropped_lio_imu(
        self, samples: Tuple[CanonicalImuSample, ...], generation: int
    ) -> None:
        """在既有发布锁内把被丢点云的惯性历史交给下一保留帧。"""

        while self._lio_publish_queue:
            next_generation, measurement = self._lio_publish_queue[0]
            if next_generation != generation:
                self._increment_counter("dropped_lio_imu_count", len(samples))
                self._increment_counter("lio_not_ready_count")
                self._record_error("拒绝跨代际承接 %d 条 LIO IMU" % len(samples))
                return
            by_timestamp = {sample.timestamp_ns: sample for sample in samples}
            conflicting_samples = 0
            for sample in measurement.imu_samples:
                previous = by_timestamp.setdefault(sample.timestamp_ns, sample)
                if previous != sample:
                    conflicting_samples += 1
            if conflicting_samples:
                # 同时刻不同内容说明源身份已失真，不能任意挑选一个值继续积分。
                self._lio_publish_queue.popleft()
                self._increment_counter("dropped_pointcloud_count")
                dropped_samples = len(by_timestamp) + conflicting_samples
                self._increment_counter("dropped_lio_imu_count", dropped_samples)
                self._increment_counter("lio_not_ready_count")
                self._record_error(
                    "LIO IMU 同时间戳内容冲突，拒绝承接并丢弃 %d 条样本"
                    % dropped_samples
                )
                return
            samples = tuple(by_timestamp[stamp] for stamp in sorted(by_timestamp))
            excess = len(samples) - DEFAULT_MAX_IMU_SAMPLES
            if excess > 0:
                samples = samples[excess:]
                self._increment_counter("dropped_lio_imu_count", excess)
                self._increment_counter("lio_not_ready_count")
                self._record_error(
                    "LIO IMU 承接超过 %d 条上限，丢弃最旧 %d 条待发样本，连续区间无法保证"
                    % (DEFAULT_MAX_IMU_SAMPLES, excess)
                )
            if samples[0].timestamp_ns > measurement.scan_begin_timestamp_ns:
                self._lio_publish_queue.popleft()
                self._increment_counter("dropped_pointcloud_count")
                self._increment_counter("lio_not_ready_count")
                self._record_error("LIO IMU 承接溢出后不覆盖扫描开始，拒绝该点云")
                continue
            self._lio_publish_queue[0] = (
                generation,
                CanonicalLioMeasurement(
                    lidar=measurement.lidar,
                    imu_samples=samples,
                    scan_begin_timestamp_ns=measurement.scan_begin_timestamp_ns,
                    scan_end_timestamp_ns=measurement.scan_end_timestamp_ns,
                ),
            )
            return
        self._increment_counter("dropped_lio_imu_count", len(samples))
        self._record_error("LIO IMU 承接后没有可覆盖点云，丢弃 %d 条样本" % len(samples))

    def _drain_native_lio_measurements(self, generation: int) -> None:
        """当前代际独立线程在条件锁外顺序发布，空闲等待，停止后退出。"""
        with self._lio_condition:
            self._lio_publish_owner_thread_id = threading.get_ident()
        try:
            while True:
                with self._lio_condition:
                    while self._lio_callback_is_current(generation) and not self._lio_publish_queue:
                        self._lio_condition.wait()
                    if not self._lio_callback_is_current(generation):
                        return
                    queued_generation, measurement = self._lio_publish_queue.popleft()
                    if queued_generation != generation:
                        continue
                    self._lio_in_flight_count += 1
                try:
                    self._publish_native_lio_measurement(measurement)
                except Exception as exc:
                    self._increment_counter("dropped_pointcloud_count")
                    self._record_error("publish canonical LIO measurement 未处理异常：%s" % exc)
                finally:
                    with self._lio_condition:
                        self._lio_in_flight_count -= 1
                        self._lio_condition.notify_all()
        except Exception as exc:
            self._record_error("原生输入发布线程异常退出：%s" % exc)
            self.stop()
        finally:
            with self._lio_condition:
                self._lio_publish_owner_active = False
                self._lio_publish_owner_thread_id = None
                self._reset_lio_assembler_if_quiescent_locked()
                self._lio_condition.notify_all()

    def _lio_callback_is_current(self, generation: Optional[int]) -> bool:
        """在 LIO condition 内拒绝 stop/restart 前创建的 callback。"""

        return self._lio_accepting and (
            generation is None or generation == self._lio_generation
        )

    def _record_lio_assembler_error(self, error: LioMeasurementAssemblerError) -> None:
        """记录 LIO 准入拒绝；不把 UNKNOWN 标定伪装成 ready。"""

        self._increment_counter("lio_not_ready_count")
        self._record_error("%s: %s" % (error.code.value, str(error)))

    def _publish_native_lio_measurement(
        self, measurement: CanonicalLioMeasurement
    ) -> None:
        """仅发布扫描末端已被 IMU 覆盖的 canonical measurement。"""

        publisher = getattr(self.runtime_provider, "publish_point_cloud", None)
        if not callable(publisher):
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("runtime_provider.publish_point_cloud 不存在")
            return
        bytes_payload = self._encode_canonical_lidar_bytes(measurement.lidar)
        if bytes_payload is None:
            self._increment_counter("dropped_pointcloud_count")
            self._increment_counter("lio_not_ready_count")
            self._record_error("lio_not_ready: canonical LiDAR payload 编码失败")
            return
        registration_frame = measurement.lidar.registration_frame_id
        imu_samples = tuple(
            LioImuSample(
                timestamp_ns=sample.timestamp_ns,
                frame_id=registration_frame,
                accel_mps2=sample.accel_mps2,
                gyro_rps=sample.gyro_rps,
            )
            for sample in measurement.imu_samples
        )
        metadata = {
            "frame_id": registration_frame,
            "timestamp_ns": measurement.scan_begin_timestamp_ns,
            "fields": (
                _GO2_L2_FIELDS if measurement.lidar.intensity_f32_le is not None
                else _CANONICAL_XYZ_OFFSET_FIELDS
            ),
            "point_count": measurement.lidar.point_count,
            "is_dense": measurement.lidar.is_dense,
            "imu_samples": imu_samples,
        }
        start = time.monotonic()
        point_step = 20 if measurement.lidar.intensity_f32_le is not None else 16
        try:
            result = publisher(bytes_payload, point_step, **metadata)
        except Exception as exc:
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("publish canonical LIO measurement 抛异常：%s" % exc)
            return
        if not bool(result):
            self._increment_counter("dropped_pointcloud_count")
            self._record_error("publish canonical LIO measurement 返回 False")
            return
        self._increment_counter("published_pointcloud_count")
        self._increment_counter("emitted_lio_measurement_count")
        self._update_counters(
            last_publish_latency_ms=(time.monotonic() - start) * 1000.0,
            last_published_point_count=measurement.lidar.point_count,
            last_published_point_step=point_step,
            last_publish_at_monotonic=time.monotonic(),
        )

    @staticmethod
    def _encode_canonical_lidar_bytes(
        lidar: CanonicalLidarFrame,
    ) -> Optional[bytes]:
        """将已通过 canonical 合同校验的 LiDAR bytes 编为 native 点布局。"""

        if (
            lidar.xyz_f32_le is None
            or lidar.time_offset_ns_le is None
        ):
            return None
        try:
            xyz = np.frombuffer(lidar.xyz_f32_le, dtype="<f4").reshape((-1, 3))
            offsets_u64 = np.frombuffer(lidar.time_offset_ns_le, dtype="<u8")
            if np.any(offsets_u64 > np.iinfo(np.uint32).max):
                return None
            packed = np.empty(
                lidar.point_count,
                dtype=_GO2_L2_DTYPE if lidar.intensity_f32_le is not None else _XYZ_OFFSET_DTYPE,
            )
            packed["x"], packed["y"], packed["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            if lidar.intensity_f32_le is not None:
                packed["intensity"] = np.frombuffer(lidar.intensity_f32_le, dtype="<f4")
            packed["offset_ns"] = offsets_u64
            return packed.tobytes()
        except (TypeError, ValueError):
            return None

    def _downsample_point_cloud(self, point_cloud: PointCloudFrame) -> PointCloudFrame:
        target = self.max_points_after_downsample
        arr = getattr(point_cloud, "points_xyz_array", None)
        if isinstance(arr, np.ndarray) and arr.ndim == 2 and arr.shape[0] > target:
            stride = max(1, int(np.ceil(arr.shape[0] / float(target))))
            downsampled = arr[::stride][:target]
            try:
                metadata = dict(getattr(point_cloud, "metadata", {}) or {})
                for key in (
                    "source_xyz_array",
                    "intensity_array",
                    "offset_ns_array",
                ):
                    values = metadata.get(key)
                    if isinstance(values, np.ndarray) and values.shape[0] == arr.shape[0]:
                        metadata[key] = np.ascontiguousarray(values[::stride][:target])
                canonical_lidar = metadata.get("canonical_lidar_frame")
                if isinstance(canonical_lidar, CanonicalLidarFrame):
                    canonical_xyz = np.frombuffer(
                        canonical_lidar.xyz_f32_le or b"", dtype="<f4"
                    )
                    canonical_xyz = canonical_xyz.reshape((-1, 3))
                    canonical_intensity = np.frombuffer(
                        canonical_lidar.intensity_f32_le or b"", dtype="<f4"
                    )
                    canonical_offsets = np.frombuffer(
                        canonical_lidar.time_offset_ns_le or b"", dtype="<u8"
                    )
                    if (
                        canonical_xyz.shape[0] == arr.shape[0]
                        and canonical_intensity.shape == (arr.shape[0],)
                        and canonical_offsets.shape == (arr.shape[0],)
                    ):
                        sampled_xyz = canonical_xyz[::stride][:target]
                        sampled_intensity = canonical_intensity[::stride][:target]
                        sampled_offsets = canonical_offsets[::stride][:target]
                        metadata["canonical_lidar_frame"] = canonical_lidar.model_copy(
                            update={
                                "point_count": int(sampled_xyz.shape[0]),
                                "xyz_f32_le": np.ascontiguousarray(sampled_xyz, dtype="<f4").tobytes(),
                                "intensity_f32_le": np.ascontiguousarray(sampled_intensity, dtype="<f4").tobytes(),
                                "time_offset_ns_le": np.ascontiguousarray(sampled_offsets, dtype="<u8").tobytes(),
                            }
                        )
                metadata.update(
                    {
                        "downsampled": True,
                        "downsample_target": int(target),
                    }
                )
                return point_cloud.model_copy(
                    update={
                        "point_count": int(downsampled.shape[0]),
                        "points_xyz_array": downsampled,
                        "metadata": metadata,
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
        return RuntimeInputBridge._resolve_point_layout(point_cloud)[0]

    @staticmethod
    def _resolve_point_fields(point_cloud: PointCloudFrame) -> Tuple[Dict[str, Any], ...]:
        return RuntimeInputBridge._resolve_point_layout(point_cloud)[1]

    @staticmethod
    def _resolve_point_layout(
        point_cloud: PointCloudFrame,
    ) -> Tuple[int, Tuple[Dict[str, Any], ...]]:
        """一次解析点云字段布局，返回固定 point step 与字段表。"""

        metadata = dict(getattr(point_cloud, "metadata", {}) or {})
        canonical_lidar = metadata.get("canonical_lidar_frame")
        if isinstance(canonical_lidar, CanonicalLidarFrame):
            return (
                (20, _GO2_L2_FIELDS) if canonical_lidar.intensity_f32_le is not None
                else (16, _CANONICAL_XYZ_OFFSET_FIELDS)
            )
        has_intensity = metadata.get("intensity_array") is not None
        has_offsets = metadata.get("offset_ns_array") is not None
        if has_intensity and has_offsets:
            return 20, _GO2_L2_FIELDS
        if has_intensity:
            return 16, _CANONICAL_XYZI_FIELDS
        if has_offsets:
            return 16, _CANONICAL_XYZ_OFFSET_FIELDS
        return 12, _CANONICAL_XYZ_FIELDS

    @staticmethod
    def _publisher_signature_cache_key(publisher: Any) -> Tuple[int, int]:
        """为函数、绑定方法和 callable 对象生成稳定的签名缓存键。"""

        owner = getattr(publisher, "__self__", None)
        function = getattr(publisher, "__func__", None)
        if owner is not None and function is not None:
            return id(owner), id(function)
        return id(publisher), id(type(publisher))

    def _publisher_accepts_metadata(self, publisher: Any) -> bool:
        """缓存 publisher 参数合同，避免每帧重复执行 inspect.signature。"""

        key = self._publisher_signature_cache_key(publisher)
        with self._lock:
            if self._point_cloud_publisher_signature_key == key:
                return bool(self._point_cloud_publisher_accepts_metadata)

        try:
            inspect.signature(publisher).bind(
                b"",
                0,
                frame_id="",
                timestamp_ns=0,
                fields=(),
                point_count=0,
                is_dense=True,
                imu_samples=(),
            )
        except (TypeError, ValueError):
            accepts_metadata = False
        else:
            accepts_metadata = True

        with self._lock:
            self._point_cloud_publisher_signature_key = key
            self._point_cloud_publisher_accepts_metadata = accepts_metadata
        return accepts_metadata

    @staticmethod
    def _encode_point_cloud_bytes(point_cloud: PointCloudFrame) -> Optional[bytes]:
        """把 PointCloudFrame 编码为 ``sensor_msgs/PointCloud2`` 兼容的 little-endian 字节。

        当前 Rust 端 ``encode_pointcloud_payload`` 期望 ``point_step(u32 LE) + raw bytes``，
        因此本桥只产出 ``raw bytes``（不含 4-byte point_step），由调用方在
        ``publish_point_cloud(bytes, point_step)`` 内合并；这里避免重复打包。
        """
        metadata = dict(getattr(point_cloud, "metadata", {}) or {})
        canonical_lidar = metadata.get("canonical_lidar_frame")
        arr = getattr(point_cloud, "points_xyz_array", None)
        if isinstance(canonical_lidar, CanonicalLidarFrame):
            return RuntimeInputBridge._encode_canonical_lidar_bytes(canonical_lidar)
        if isinstance(arr, np.ndarray) and arr.ndim == 2 and arr.shape[1] >= 3:
            xyz = np.ascontiguousarray(arr[:, :3], dtype="<f4")
            intensity = metadata.get("intensity_array")
            offset_ns = metadata.get("offset_ns_array")
            if intensity is not None and not (
                isinstance(intensity, np.ndarray)
                and intensity.shape == (xyz.shape[0],)
            ):
                return None
            if offset_ns is not None and not (
                isinstance(offset_ns, np.ndarray)
                and offset_ns.shape == (xyz.shape[0],)
            ):
                return None
            if intensity is None and offset_ns is None:
                # XYZ 数组已经连续且 little-endian；仅建立结构化视图，不再复制。
                return xyz.view(_XYZ_DTYPE).reshape(-1).tobytes()
            if intensity is not None and offset_ns is not None:
                packed = np.empty(xyz.shape[0], dtype=_GO2_L2_DTYPE)
                packed["intensity"] = intensity
                packed["offset_ns"] = offset_ns
            elif intensity is not None:
                packed = np.empty(xyz.shape[0], dtype=_XYZI_DTYPE)
                packed["intensity"] = intensity
            else:
                packed = np.empty(xyz.shape[0], dtype=_XYZ_OFFSET_DTYPE)
                packed["offset_ns"] = offset_ns
            packed["x"] = xyz[:, 0]
            packed["y"] = xyz[:, 1]
            packed["z"] = xyz[:, 2]
            return packed.tobytes()
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
            started = self._started
            ready = (
                self._started
                and not self._runtime_unavailable
                and any(subscription is not None for subscription in (
                    self._localization_subscription_id,
                    self._point_cloud_subscription_id,
                    self._sensor_sample_subscription_id,
                ))
            )
            runtime_unavailable = self._runtime_unavailable
            missing_localization_provider = self._missing_localization_provider
            missing_point_cloud_provider = self._missing_point_cloud_provider
            localization_subscription_id = self._localization_subscription_id
            point_cloud_subscription_id = self._point_cloud_subscription_id
            sensor_sample_subscription_id = self._sensor_sample_subscription_id
        with self._counter_lock:
            counters = self.counters.as_dict()
        return {
            "started": started,
            "ready": ready,
            "runtime_unavailable": runtime_unavailable,
            "missing_localization_provider": missing_localization_provider,
            "missing_point_cloud_provider": missing_point_cloud_provider,
            "localization_subscription_id": localization_subscription_id,
            "point_cloud_subscription_id": point_cloud_subscription_id,
            "sensor_sample_subscription_id": sensor_sample_subscription_id,
            "counters": counters,
        }

    def _record_error(self, message: str) -> None:
        self._update_counters(
            last_error=message,
            last_error_at_monotonic=time.monotonic(),
        )
        error_key = _REPEATED_INPUT_ERROR_KEYS.get(message)
        if error_key is None:
            LOGGER.debug("RuntimeInputBridge: %s", message)
        elif LOGGER.isEnabledFor(logging.DEBUG):
            # 放行前不创建日志记录；最新错误与完整计数始终已在上方更新。
            self._error_log_limiter.log(
                LOGGER,
                logging.DEBUG,
                "RuntimeInputBridge: %s",
                message,
                key=error_key,
                interval_sec=_INPUT_ERROR_LOG_INTERVAL_SEC,
                extra={"event": "geometry.input.diagnostic", "diagnostic_kind": error_key},
            )


def build_default_runtime_input_bridge(
    *,
    runtime_provider: Any,
    localization_provider: Optional[StreamingLocalizationProvider] = None,
    point_cloud_provider: Optional[StreamingPointCloudProvider] = None,
    sensor_sample_provider: Optional[StreamingSensorSampleProvider] = None,
) -> RuntimeInputBridge:
    """工厂方法，集中默认值，便于测试与生产链路复用。"""
    return RuntimeInputBridge(
        runtime_provider=runtime_provider,
        localization_provider=localization_provider,
        point_cloud_provider=point_cloud_provider,
        sensor_sample_provider=sensor_sample_provider,
    )


__all__ = [
    "BridgeCounters",
    "MAX_POINTS_BEFORE_DOWNSAMPLE",
    "MAX_POINTS_AFTER_DOWNSAMPLE",
    "SHM_PAYLOAD_THRESHOLD_BYTES",
    "RuntimeInputBridge",
    "build_default_runtime_input_bridge",
]
