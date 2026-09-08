"""Go2 设备侧数据面。

平台地图、定位、导航和探索主链由 services.geometry / services.mapping /
services.localization / services.navigation 承载；本模块只保留 Go2 官方 DDS 输入、
IMU/点云观测与只读 RobotState 服务状态。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import logging
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, TYPE_CHECKING, Tuple

import numpy as np

from adapters.sensors.go2_calibration import (
    GO2_CALIBRATION_PROFILE,
    Go2CalibrationProfile,
    go2_calibration_profile_from_config,
)
from adapters.sensors.go2_l2 import (
    Go2L2DecodeError,
    adapt_go2_l2_imu,
    adapt_go2_l2_lidar,
    decode_go2_l2_pointcloud,
)
from contracts.geometry import FrameTree, Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame, PointField
from contracts.robot_state import IMUState
from contracts.sensors import RegistrationState, SensorSample
from core.logging_utils import SamplingLogFilter as _SamplingLogFilter

if TYPE_CHECKING:
    from drivers.go2.settings import Go2DataPlaneConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane")
LOGGER.addFilter(_SamplingLogFilter(sample_rate=0.01))


_STATIONARY_YAW_RATE_FILTER_TAU_SEC = 0.5
_MAX_STATIONARY_YAW_SAMPLE_GAP_SEC = 1.0
_GO2_UTLIDAR_FRAME = GO2_CALIBRATION_PROFILE.lidar_source_frame
_POINT_CLOUD_POSE_HISTORY_SIZE = 256
_MAX_POINT_CLOUD_POSE_SKEW_SEC = 0.5
_NATIVE_IMU_READER_HISTORY_DEPTH = 256
_NATIVE_CLOUD_READER_HISTORY_DEPTH = 16
_NATIVE_STATE_SAMPLE_INTERVAL_SEC = 0.05


class _HistoryChannelSubscriber:
    """通过公开接收接口复用有界读取器和等待资源，关闭时先取消在读操作。"""

    def __init__(
        self, modules, topic, message_type, qos, *,
        read_batch_size: int = 1, latest_sample_key: Optional[Callable[[object], int]] = None,
    ) -> None:
        self._modules = modules
        self._topic_name = topic
        self._message_type = message_type
        self._qos = qos
        self._read_batch_size = int(read_batch_size) if latest_sample_key is not None else 1
        if self._read_batch_size <= 0:
            raise ValueError("Go2 DDS 读取批次大小必须为正")
        self._latest_sample_key = latest_sample_key
        self._dropped_sample_count = 0
        self._lock = threading.RLock()
        self._initialized = False
        self._initializing = False
        self._closed = False
        self._reading_thread_id: Optional[int] = None
        self._participant = None
        self._topic = None
        self._reader = None
        self._read_condition = None
        self._waitset = None
        self._guard_condition = None

    def Init(self) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("Go2 DDS 读取器已关闭，重启须创建新代际实例")
            if self._initialized:
                return
            if self._initializing:
                raise RuntimeError("Go2 DDS 读取器初始化不允许重入")
            self._initializing = True
            try:
                # SDK 已按设备接口初始化域0；新增参与者复用该域，不重设域级配置。
                self._participant = self._modules["DomainParticipant"](0)
                self._topic = self._modules["Topic"](
                    self._participant, self._topic_name, self._message_type,
                )
                self._reader = self._modules["DataReader"](
                    self._participant, self._topic, self._qos,
                )
                self._read_condition = self._modules["ReadCondition"](
                    self._reader, self._modules["ReadConditionMask"],
                )
                self._waitset = self._modules["WaitSet"](self._participant)
                self._guard_condition = self._modules["GuardCondition"](self._participant)
                self._waitset.attach(self._read_condition)
                self._waitset.attach(self._guard_condition)
                if self._closed:
                    raise RuntimeError("Go2 DDS 读取器初始化期间已关闭")
                self._initialized = True
            except Exception:
                self._closed = True
                self._release_entities_locked()
                raise
            finally:
                self._initializing = False

    def Read(self, timeout=None):
        with self._lock:
            if self._closed or not self._initialized:
                return None
            if self._reading_thread_id is not None:
                raise RuntimeError("Go2 DDS 读取器只允许所属线程逐次读取")
            self._reading_thread_id = threading.get_ident()
            reader, condition, waitset = self._reader, self._read_condition, self._waitset
        try:
            deadline = None if timeout is None else time.monotonic() + max(0.0, float(timeout))
            while True:
                with self._lock:
                    if self._closed:
                        return None
                samples = reader.take(self._read_batch_size, condition=condition)
                if samples:
                    # 点云是可替代扫描：在昂贵解析/分发之前丢弃当前批次的旧云。
                    # 惯性读取器仍每次只取一条，源时间及完整惯性历史不变。
                    selected = (
                        max(samples, key=self._latest_sample_key)
                        if self._latest_sample_key is not None else samples[0]
                    )
                    with self._lock:
                        if self._closed:
                            return None
                        self._dropped_sample_count += len(samples) - 1
                        return selected
                if deadline is None:
                    remaining_ns = (1 << 63) - 1
                else:
                    remaining_ns = int(max(0.0, deadline - time.monotonic()) * 1_000_000_000)
                    if remaining_ns == 0:
                        return None
                waitset.wait(remaining_ns)
        finally:
            with self._lock:
                self._reading_thread_id = None
                if self._closed:
                    self._release_entities_locked()

    @property
    def dropped_sample_count(self) -> int:
        with self._lock:
            return self._dropped_sample_count

    def Close(self) -> None:
        with self._lock:
            self._closed = True
            if self._guard_condition is not None:
                self._guard_condition.set(True)
            # 由外层已有有界线程等待负责排空；不得在这里无期限等待反序列化。
            # 在读/初始化的最后退出者释放实体，迟到样本仍被关闭事实拒绝。
            if self._initializing or self._reading_thread_id is not None:
                return
            self._release_entities_locked()

    def _release_entities_locked(self) -> None:
        waitset = self._waitset
        if waitset is not None:
            for condition in (self._read_condition, self._guard_condition):
                if condition is not None:
                    try:
                        waitset.detach(condition)
                    except Exception:
                        LOGGER.debug("分离 Go2 DDS 读取条件失败。", exc_info=True)
        # Python 绑定由拥有引用析构实体，不调用SDK私有成员或删除执行中的读取器。
        self._waitset = None
        self._read_condition = None
        self._guard_condition = None
        self._reader = None
        self._topic = None
        self._participant = None
        self._initialized = False


@dataclass(frozen=True)
class _TimedPoseSnapshot:
    generation: int
    received_at_monotonic: float
    pose: Pose
    yaw_stabilization_offset_rad: float
    stationary_yaw_locked: bool


def _normalize_angle_radians(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _quaternion_yaw_radians(quaternion: Quaternion) -> float:
    return math.atan2(
        2.0
        * (
            (float(quaternion.w) * float(quaternion.z))
            + (float(quaternion.x) * float(quaternion.y))
        ),
        1.0
        - 2.0
        * (
            (float(quaternion.y) * float(quaternion.y))
            + (float(quaternion.z) * float(quaternion.z))
        ),
    )


def _rotate_quaternion_world_yaw(
    quaternion: Quaternion,
    yaw_offset_rad: float,
) -> Quaternion:
    half_offset = 0.5 * float(yaw_offset_rad)
    cosine = math.cos(half_offset)
    sine = math.sin(half_offset)
    return Quaternion(
        x=(cosine * float(quaternion.x)) - (sine * float(quaternion.y)),
        y=(sine * float(quaternion.x)) + (cosine * float(quaternion.y)),
        z=(cosine * float(quaternion.z)) + (sine * float(quaternion.w)),
        w=(cosine * float(quaternion.w)) - (sine * float(quaternion.z)),
    )


def _rotate_registered_xyz_about_pose(
    points_xyz: np.ndarray,
    yaw_offset_rad: float,
    origin_x: float,
    origin_y: float,
) -> np.ndarray:
    if abs(float(yaw_offset_rad)) <= 1e-12:
        return points_xyz
    cosine = math.cos(float(yaw_offset_rad))
    sine = math.sin(float(yaw_offset_rad))
    relative_x = points_xyz[:, 0].astype(np.float64) - float(origin_x)
    relative_y = points_xyz[:, 1].astype(np.float64) - float(origin_y)
    rotated = np.array(points_xyz, dtype=np.float32, copy=True, order="C")
    rotated[:, 0] = float(origin_x) + (cosine * relative_x) - (sine * relative_y)
    rotated[:, 1] = float(origin_y) + (sine * relative_x) + (cosine * relative_y)
    return rotated


def _quaternion_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm_squared = (x * x) + (y * y) + (z * z) + (w * w)
    if norm_squared <= 1e-12:
        raise ValueError("位姿四元数模长为零")
    scale = 2.0 / norm_squared
    return np.asarray(
        [
            [1.0 - (scale * ((y * y) + (z * z))), scale * ((x * y) - (z * w)), scale * ((x * z) + (y * w))],
            [scale * ((x * y) + (z * w)), 1.0 - (scale * ((x * x) + (z * z))), scale * ((y * z) - (x * w))],
            [scale * ((x * z) - (y * w)), scale * ((y * z) + (x * w)), 1.0 - (scale * ((x * x) + (y * y)))],
        ],
        dtype=np.float64,
    )


def _transform_utlidar_xyz_to_odom(
    points_xyz: np.ndarray,
    pose: Pose,
    calibration_profile: Go2CalibrationProfile = GO2_CALIBRATION_PROFILE,
) -> np.ndarray:
    odom_from_body_rotation = _quaternion_rotation_matrix(pose.orientation)
    odom_from_body_translation = np.asarray(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=np.float64,
    )
    points_in_body = calibration_profile.transform_lidar_xyz(points_xyz)
    points_in_odom = (
        points_in_body @ odom_from_body_rotation.T
    ) + odom_from_body_translation
    return np.ascontiguousarray(points_in_odom, dtype=np.float32)


def _unitree_wxyz_quaternion_to_contract(values: Sequence[float]) -> Quaternion:
    raw_values = [float(item) for item in list(values or [])[:4]]
    if len(raw_values) != 4:
        raise ValueError("Unitree quaternion 必须提供完整四元数")
    if not all(math.isfinite(item) for item in raw_values):
        raise ValueError("Unitree quaternion 不能包含 NaN/Inf")

    w, x, y, z = raw_values
    norm = float(np.sqrt((w * w) + (x * x) + (y * y) + (z * z)))
    if norm <= 1e-12:
        raise ValueError("Unitree quaternion 模长不能为零")
    inv_norm = 1.0 / norm
    return Quaternion(x=x * inv_norm, y=y * inv_norm, z=z * inv_norm, w=w * inv_norm)


def _unitree_stamp_to_datetime_ns(stamp: object, label: str) -> Tuple[datetime, int]:
    try:
        sec = int(getattr(stamp, "sec"))
        nanosec = int(getattr(stamp, "nanosec"))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} stamp.sec/nanosec 非法：{exc}") from exc
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f"{label} stamp 越界：sec={sec} nanosec={nanosec}")
    try:
        timestamp = datetime.fromtimestamp(sec, tz=timezone.utc).replace(
            microsecond=nanosec // 1_000
        )
    except (OverflowError, OSError, ValueError) as exc:
        raise ValueError(f"{label} stamp 无法转换：{exc}") from exc
    return timestamp, (sec * 1_000_000_000) + nanosec


def _point_cloud_source_time_key(message: object) -> int:
    """只比较合法源时间头；选中帧仍须通过原完整载荷/坐标/水位校验。"""
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    try:
        sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return -1
    return sec * 1_000_000_000 + nanosec if sec >= 0 and 0 <= nanosec < 1_000_000_000 else -1


def _prepare_go2_official_sdk_environment(config: "Go2DataPlaneConfig") -> None:
    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    importlib.invalidate_caches()


def _load_go2_official_sdk_modules(
    config: "Go2DataPlaneConfig",
    *,
    include_dds_topics: bool = False,
) -> Optional[Dict[str, object]]:
    _prepare_go2_official_sdk_environment(config)
    try:
        modules: Dict[str, object] = {
            "ChannelFactoryInitialize": getattr(
                importlib.import_module("unitree_sdk2py.core.channel"),
                "ChannelFactoryInitialize",
            ),
            "RobotStateClient": getattr(
                importlib.import_module("unitree_sdk2py.go2.robot_state.robot_state_client"),
                "RobotStateClient",
            ),
        }
        if include_dds_topics:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            unitree_go_dds_module = importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
            modules["ChannelSubscriber"] = getattr(channel_module, "ChannelSubscriber")
            modules["SportModeState"] = getattr(unitree_go_dds_module, "SportModeState_")
            if config.lio_backend in {"fastlio2", "pointlio"}:
                qos_module = importlib.import_module("cyclonedds.qos")
                core_module = importlib.import_module("cyclonedds.core")
                modules["DomainParticipant"] = getattr(
                    importlib.import_module("cyclonedds.domain"), "DomainParticipant",
                )
                modules["Topic"] = getattr(importlib.import_module("cyclonedds.topic"), "Topic")
                modules["DataReader"] = getattr(importlib.import_module("cyclonedds.sub"), "DataReader")
                for name in ("ReadCondition", "WaitSet", "GuardCondition"):
                    modules[name] = getattr(core_module, name)
                modules["ReadConditionMask"] = (
                    core_module.ViewState.Any | core_module.InstanceState.Alive | core_module.SampleState.NotRead
                )
                modules["Qos"] = getattr(qos_module, "Qos")
                modules["HistoryKeepLast"] = getattr(getattr(qos_module, "Policy").History, "KeepLast")
                modules["ResourceLimits"] = getattr(getattr(qos_module, "Policy"), "ResourceLimits")
            # sensor_msgs/PointCloud2 用于 DirectDDS 点云订阅；缺失则 point_cloud_enabled
            # 启动时记录到 point_cloud_bridge_last_error，避免 silent fallback。
            try:
                sensor_pc_module = importlib.import_module("unitree_sdk2py.idl.sensor_msgs.msg.dds_")
                modules["PointCloud2"] = getattr(sensor_pc_module, "PointCloud2_")
            except Exception as exc:
                LOGGER.debug("导入 unitree_sdk2py PointCloud2 失败：%s", exc)
        return modules
    except Exception as exc:
        LOGGER.warning("导入 Go2 官方 SDK 模块失败：%s", exc)
        return None


class Go2DirectDdsBridge:
    """Go2 官方 DDS 原始输入桥（直连 unitree_sdk2py，绕开 ROS2 中间层）。"""

    def __init__(self, config: "Go2DataPlaneConfig") -> None:
        self.config = config
        # 原生定位两类样本共用创建时标定；设备自带模式不消费或验证定位覆盖。
        self._calibration_profile = (
            go2_calibration_profile_from_config(config.lio_body_from_sensor, config.lio_body_from_imu)
            if config.lio_backend in {"fastlio2", "pointlio"}
            else GO2_CALIBRATION_PROFILE
        )
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._lifecycle_generation = 0
        self._active_generation: Optional[int] = None
        self._started = False
        self._direct_dds_started = False
        self._direct_dds_stop_event: Optional[threading.Event] = None
        self._direct_dds_threads: List[threading.Thread] = []
        self._direct_pose_subscriber = None
        self._direct_point_cloud_subscriber = None
        self._direct_point_cloud_msg_type = None
        self._latest_pose_from_dds: Optional[Pose] = None
        self._latest_imu_state_from_dds: Optional[IMUState] = None
        self._latest_local_point_cloud: Optional[PointCloudFrame] = None
        self._latest_pose_generation: Optional[int] = None
        self._latest_imu_generation: Optional[int] = None
        self._latest_imu_source_timestamp_ns: Optional[int] = None
        self._latest_point_cloud_generation: Optional[int] = None
        self._last_accepted_point_cloud_source_timestamp_ns: Optional[int] = None
        self._latest_pose_source = ""
        self._latest_imu_source_topic = ""
        self._latest_point_cloud_source_topic = ""
        self._localization_callbacks: Dict[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]] = {}
        self._point_cloud_callbacks: Dict[int, Callable[[PointCloudFrame], None]] = {}
        self._sensor_sample_callbacks: Dict[int, Callable[[SensorSample], None]] = {}
        self._dds_input_timing: Dict[str, Deque[Dict[str, Any]]] = {}
        self._next_callback_id = 1
        self._direct_pose_ready = False
        self._direct_imu_ready = False
        self._direct_point_cloud_ready = False
        # Phase 5：点云 bridge 错误显式记录，禁止 silent fallback。
        self._point_cloud_bridge_last_error: Optional[str] = None
        self._point_cloud_bridge_last_error_at: Optional[float] = None
        self._point_cloud_bridge_topic: str = ""

    def start(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                if self._started and self._active_generation is not None:
                    return
                self._lifecycle_generation += 1
                self._active_generation = self._lifecycle_generation
                self._direct_dds_stop_event = threading.Event()
                self._started = True
                self._invalidate_observations_locked("Go2 DirectDDS 正在启动")
            try:
                self._start_direct_dds_backend()
            except Exception as exc:
                self._stop_direct_dds_backend()
                with self._lock:
                    self._started = False
                    self._active_generation = None
                    self._invalidate_observations_locked(f"Go2 DirectDDS 启动失败：{exc}")
                raise

    def stop(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                self._started = False
                self._active_generation = None
                self._invalidate_observations_locked("Go2 DirectDDS 已停止")
            self._stop_direct_dds_backend()

    def is_running(self) -> bool:
        with self._lock:
            return self._started and self._active_generation is not None

    def get_current_pose(self) -> Optional[Pose]:
        with self._lock:
            active = (
                self._started
                and self._direct_dds_started
                and self._active_generation is not None
                and self._latest_pose_generation == self._active_generation
            )
            return (
                self._latest_pose_from_dds.model_copy(deep=False)
                if active and self._latest_pose_from_dds is not None
                else None
            )

    def get_imu_state(self) -> Optional[IMUState]:
        with self._lock:
            active = (
                self._started
                and self._direct_dds_started
                and self._active_generation is not None
                and self._latest_imu_generation == self._active_generation
            )
            return (
                self._latest_imu_state_from_dds.model_copy(deep=False)
                if active and self._latest_imu_state_from_dds is not None
                else None
            )

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        with self._lock:
            active = (
                self._started
                and self._direct_dds_started
                and self._active_generation is not None
                and self._direct_point_cloud_ready
                and self._latest_point_cloud_generation == self._active_generation
            )
            point_cloud = (
                self._latest_local_point_cloud.model_copy(deep=False)
                if active and self._latest_local_point_cloud is not None
                else None
            )
        if point_cloud is None or max_points is None or point_cloud.point_count <= max_points:
            return point_cloud
        # 高性能路径：优先使用 ndarray 降采样
        if point_cloud.points_xyz_array is not None:
            arr = point_cloud.points_xyz_array
            stride = max(1, int(np.ceil(arr.shape[0] / float(max_points))))
            downsampled = arr[::stride][:max_points]
            return point_cloud.model_copy(
                update={
                    "point_count": int(downsampled.shape[0]),
                    "points_xyz_array": downsampled,
                    "metadata": {**dict(point_cloud.metadata), "downsampled": True},
                },
                deep=False,
            )
        stride = max(1, int(np.ceil(point_cloud.point_count / float(max_points))))
        points = point_cloud.points[::stride][:max_points]
        return point_cloud.model_copy(
            update={
                "point_count": len(points),
                "points": points,
                "metadata": {**dict(point_cloud.metadata), "downsampled": True},
            },
            deep=False,
        )

    def _next_subscription_id(self) -> int:
        callback_id = self._next_callback_id
        self._next_callback_id += 1
        return callback_id

    def subscribe_localization(self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]) -> int:
        with self._lock:
            callback_id = self._next_subscription_id()
            self._localization_callbacks[callback_id] = callback
            return callback_id

    def subscribe_sensor_sample(self, callback: Callable[[SensorSample], None]) -> int:
        with self._lock:
            callback_id = self._next_subscription_id()
            self._sensor_sample_callbacks[callback_id] = callback
            return callback_id

    def unsubscribe_sensor_sample(self, callback_id: int) -> None:
        with self._lock:
            self._sensor_sample_callbacks.pop(int(callback_id), None)

    def _publish_sensor_sample(self, sample: SensorSample, generation: int) -> None:
        """标准输入先于显示回调；沿原同步回调代际与外层交付围栏收尾。"""
        with self._lock:
            callbacks = tuple(self._sensor_sample_callbacks.values())
            delivery_token = self._direct_dds_stop_event
        for callback in callbacks:
            with self._lock:
                if not self._is_generation_active_locked(generation, delivery_token):
                    return
            try:
                callback(sample)
            except Exception:
                LOGGER.debug("分发 Go2 规范传感器样本失败。", exc_info=True)

    def unsubscribe_localization(self, callback_id: int) -> None:
        with self._lock:
            self._localization_callbacks.pop(int(callback_id), None)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> int:
        with self._lock:
            callback_id = self._next_subscription_id()
            self._point_cloud_callbacks[callback_id] = callback
            return callback_id

    def unsubscribe_local_point_cloud(self, callback_id: int) -> None:
        with self._lock:
            self._point_cloud_callbacks.pop(int(callback_id), None)

    def _record_dds_input_timing(
        self, kind: str, stamp: object, generation: int,
        read_started: float, read_returned: float, received_wall_ns: int,
    ) -> None:
        """每个源只留十个秒桶，区分返回时已旧与回调处理变慢，不保存载荷。"""
        completed = time.monotonic()
        try:
            sec, nanosec = int(stamp.sec), int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return
        source_ns = sec * 1_000_000_000 + nanosec
        values = {
            "source_age_ms": (received_wall_ns - source_ns) / 1_000_000,
            "read_ms": (read_returned - read_started) * 1000.0,
            "callback_ms": (completed - read_returned) * 1000.0,
        }
        with self._lock:
            if not self._is_generation_active_locked(generation):
                return
            buckets = self._dds_input_timing.setdefault(kind, deque(maxlen=10))
            second = int(completed)
            if not buckets or buckets[-1]["second"] != second:
                buckets.append({"second": second, "count": 0})
            bucket = buckets[-1]
            bucket["count"] += 1
            bucket["source_ns"] = source_ns
            for name, value in values.items():
                total, maximum, _last = bucket.get(name, (0.0, value, value))
                bucket[name] = (total + value, max(maximum, value), value)

    def _input_timing_snapshot_locked(self) -> Dict[str, Any]:
        """诊断读取只汇总近期常量规模计数；无新输入不延长窗口。"""
        if not self._dds_input_timing:
            return {}
        oldest_second = int(time.monotonic()) - 9
        result = {}
        for kind, history in self._dds_input_timing.items():
            buckets = [bucket for bucket in history if bucket["second"] >= oldest_second]
            count = sum(bucket["count"] for bucket in buckets)
            item = {"window_sec": 10, "sample_count": count}
            if count:
                item["last_source_timestamp_ns"] = buckets[-1]["source_ns"]
                for name in ("source_age_ms", "read_ms", "callback_ms"):
                    item[name + "_latest"] = buckets[-1][name][2]
                    item[name + "_mean"] = sum(bucket[name][0] for bucket in buckets) / count
                    item[name + "_max"] = max(bucket[name][1] for bucket in buckets)
            result[kind] = item
        return result

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            active_generation = self._active_generation
            pose_active = (
                self._started
                and self._direct_dds_started
                and active_generation is not None
                and self._direct_pose_ready
                and self._latest_pose_generation == active_generation
            )
            imu_active = (
                self._started
                and self._direct_dds_started
                and active_generation is not None
                and self._direct_imu_ready
                and self._latest_imu_generation == active_generation
            )
            point_cloud_active = (
                self._started
                and self._direct_dds_started
                and active_generation is not None
                and self._direct_point_cloud_ready
                and self._latest_local_point_cloud is not None
                and self._latest_point_cloud_generation == active_generation
            )
            return {
                "started": self._started and active_generation is not None,
                "direct_dds_started": self._direct_dds_started and active_generation is not None,
                "lifecycle_generation": self._lifecycle_generation,
                "active_generation": active_generation,
                "input_timing": self._input_timing_snapshot_locked(),
                "dds_pose_ready": pose_active,
                "dds_imu_ready": imu_active,
                "dds_cloud_ready": point_cloud_active,
                "pose_source": self._latest_pose_source if pose_active else "",
                "imu_source_topic": self._latest_imu_source_topic if imu_active else "",
                "point_cloud_source_topic": (self._latest_point_cloud_source_topic if point_cloud_active else ""),
                "latest_point_cloud_points": (self._latest_local_point_cloud.point_count if point_cloud_active else 0),
                "dds_point_cloud_reader_drop_count": getattr(
                    self._direct_point_cloud_subscriber, "dropped_sample_count", 0
                ),
                "last_accepted_point_cloud_source_timestamp_ns": (self._last_accepted_point_cloud_source_timestamp_ns),
                # Phase 5：§13.4 要求 status 暴露这些字段。
                "point_cloud_bridge_topic": self._point_cloud_bridge_topic,
                "point_cloud_bridge_last_error": self._point_cloud_bridge_last_error,
                "point_cloud_bridge_last_error_at_monotonic": self._point_cloud_bridge_last_error_at,
                "stationary_yaw_locked": bool(
                    getattr(self, "_stationary_yaw_locked", False)
                ),
                "orientation_yaw_rate_rps": float(
                    getattr(self, "_stationary_yaw_rate_rps", 0.0)
                ),
                "yaw_stabilization_offset_rad": float(
                    getattr(self, "_stationary_yaw_offset_rad", 0.0)
                ),
                "point_cloud_yaw_stabilization_applied": bool(
                    point_cloud_active
                    and self._latest_local_point_cloud.metadata.get(
                        "yaw_stabilization_applied",
                        False,
                    )
                ),
            }

    def _record_point_cloud_bridge_error(
        self,
        message: str,
        *,
        generation: Optional[int] = None,
    ) -> None:
        error_message = str(message)
        with self._lifecycle_lock:
            with self._lock:
                if generation is not None and generation != self._active_generation:
                    return
                self._invalidate_point_cloud_locked(error_message)
        # LOGGER 上的 SamplingLogFilter 对 DEBUG 做 1% 采样；状态始终记录完整错误。
        LOGGER.debug("Go2 DirectDDS 点云帧拒绝：%s", error_message)

    def _invalidate_point_cloud_locked(self, message: str) -> None:
        self._latest_local_point_cloud = None
        self._latest_point_cloud_generation = None
        self._latest_point_cloud_source_topic = ""
        self._direct_point_cloud_ready = False
        self._point_cloud_bridge_last_error = str(message)
        self._point_cloud_bridge_last_error_at = time.monotonic()

    def _invalidate_observations_locked(self, message: str) -> None:
        self._dds_input_timing.clear()
        self._latest_pose_from_dds = None
        self._latest_imu_state_from_dds = None
        self._latest_pose_generation = None
        self._latest_imu_generation = None
        self._latest_imu_source_timestamp_ns = None
        self._latest_pose_source = ""
        self._latest_imu_source_topic = ""
        self._direct_pose_ready = False
        self._direct_imu_ready = False
        self._native_state_sample_at = None
        self._native_state_sample_key = None
        self._reset_stationary_yaw_filter_locked()
        self._invalidate_point_cloud_locked(message)

    def _reset_stationary_yaw_filter_locked(self) -> None:
        self._stationary_yaw_last_raw_rad: Optional[float] = None
        self._stationary_yaw_last_sample_at: Optional[float] = None
        self._stationary_yaw_rate_rps = 0.0
        self._stationary_yaw_output_rad: Optional[float] = None
        self._stationary_yaw_offset_rad = 0.0
        self._stationary_yaw_locked = False
        pose_history = getattr(self, "_pose_history", None)
        if pose_history is None:
            self._pose_history: Deque[_TimedPoseSnapshot] = deque(
                maxlen=_POINT_CLOUD_POSE_HISTORY_SIZE
            )
        else:
            pose_history.clear()

    def _nearest_pose_snapshot_locked(
        self,
        *,
        generation: int,
        received_at_monotonic: float,
    ) -> Tuple[Optional[_TimedPoseSnapshot], Optional[float]]:
        pose_history = getattr(self, "_pose_history", ())
        candidates = (
            snapshot
            for snapshot in pose_history
            if snapshot.generation == generation
        )
        nearest = min(
            candidates,
            key=lambda snapshot: abs(
                snapshot.received_at_monotonic - received_at_monotonic
            ),
            default=None,
        )
        if nearest is None:
            return None, None
        return nearest, abs(nearest.received_at_monotonic - received_at_monotonic)

    def _stabilize_stationary_yaw_locked(
        self,
        message: object,
        orientation: Quaternion,
    ) -> Quaternion:
        raw_yaw = _quaternion_yaw_radians(orientation)
        sample_at = time.monotonic()
        last_raw_yaw = getattr(self, "_stationary_yaw_last_raw_rad", None)
        last_sample_at = getattr(self, "_stationary_yaw_last_sample_at", None)
        filtered_rate = float(getattr(self, "_stationary_yaw_rate_rps", 0.0))
        contiguous_sample = last_raw_yaw is not None and last_sample_at is not None
        if contiguous_sample:
            elapsed = sample_at - last_sample_at
            contiguous_sample = 0.0 < elapsed <= _MAX_STATIONARY_YAW_SAMPLE_GAP_SEC
            if contiguous_sample:
                raw_rate = _normalize_angle_radians(raw_yaw - last_raw_yaw) / elapsed
                alpha = 1.0 - math.exp(
                    -elapsed / _STATIONARY_YAW_RATE_FILTER_TAU_SEC
                )
                filtered_rate += alpha * (raw_rate - filtered_rate)
            else:
                filtered_rate = 0.0

        self._stationary_yaw_last_raw_rad = raw_yaw
        self._stationary_yaw_last_sample_at = sample_at
        self._stationary_yaw_rate_rps = filtered_rate

        deadband_rps = float(self.config.direct_dds.gyro_deadband_rps)
        try:
            robot_idle = int(getattr(message, "mode")) == 0 and int(
                getattr(message, "gait_type")
            ) == 0
        except (AttributeError, TypeError, ValueError):
            robot_idle = False

        if deadband_rps <= 0.0 or not contiguous_sample:
            self._stationary_yaw_output_rad = raw_yaw
            self._stationary_yaw_offset_rad = 0.0
            self._stationary_yaw_locked = False
            return orientation

        output_yaw = getattr(self, "_stationary_yaw_output_rad", None)
        if robot_idle and abs(filtered_rate) <= deadband_rps:
            if output_yaw is None:
                output_yaw = raw_yaw
            self._stationary_yaw_offset_rad = _normalize_angle_radians(
                output_yaw - raw_yaw
            )
            self._stationary_yaw_locked = True
        else:
            output_yaw = _normalize_angle_radians(
                raw_yaw + float(getattr(self, "_stationary_yaw_offset_rad", 0.0))
            )
            self._stationary_yaw_locked = False

        self._stationary_yaw_output_rad = output_yaw
        return _rotate_quaternion_world_yaw(
            orientation,
            _normalize_angle_radians(output_yaw - raw_yaw),
        )

    def _is_generation_active_locked(
        self,
        generation: Optional[int],
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        if (
            generation is None
            or not self._started
            or not self._direct_dds_started
            or generation != self._active_generation
        ):
            return False
        if stop_event is not None:
            return stop_event is self._direct_dds_stop_event and not stop_event.is_set()
        return self._direct_dds_stop_event is not None and not self._direct_dds_stop_event.is_set()

    def _active_generation_snapshot(self) -> Optional[int]:
        with self._lock:
            generation = self._active_generation
            return generation if self._is_generation_active_locked(generation) else None

    @staticmethod
    def _close_direct_dds_subscriber(subscriber: object, label: str) -> None:
        close = getattr(subscriber, "Close", None)
        if not callable(close):
            return
        try:
            close()
        except Exception:
            LOGGER.debug("关闭 Go2 官方 DDS %s 失败。", label, exc_info=True)

    def _start_direct_dds_backend(self) -> None:
        with self._lock:
            self._direct_dds_started = False
            generation = self._active_generation
            stop_event = self._direct_dds_stop_event
            self._invalidate_observations_locked("Go2 DirectDDS 等待后端启动")
        if generation is None or stop_event is None or stop_event.is_set():
            raise RuntimeError("Go2 DirectDDS 启动代际无效")
        if not getattr(self.config, "direct_dds", None) or not self.config.direct_dds.enabled:
            self._record_point_cloud_bridge_error("Go2 DirectDDS 后端未启用")
            return
        modules = _load_go2_official_sdk_modules(self.config, include_dds_topics=True)
        if modules is None:
            message = "Go2 DirectDDS SDK 模块加载失败"
            self._record_point_cloud_bridge_error(message)
            raise RuntimeError(message)

        channel_factory_initialize = modules["ChannelFactoryInitialize"]
        iface = str(self.config.dds_iface or "").strip()
        try:
            if iface:
                channel_factory_initialize(0, iface)
            else:
                channel_factory_initialize(0)
        except TypeError:
            try:
                channel_factory_initialize(0)
            except Exception as exc:
                LOGGER.warning("初始化 Go2 官方 DDS 直连失败：%s", exc)
                self._record_point_cloud_bridge_error(f"DirectDDS channel 初始化失败：{exc}")
                raise RuntimeError(f"DirectDDS channel 初始化失败：{exc}") from exc
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 直连失败：%s", exc)
            self._record_point_cloud_bridge_error(f"DirectDDS channel 初始化失败：{exc}")
            raise RuntimeError(f"DirectDDS channel 初始化失败：{exc}") from exc

        point_cloud_enabled = bool(self.config.direct_dds.point_cloud_enabled)
        pose_subscriber = None
        point_cloud_subscriber = None
        point_cloud_type = None
        try:
            subscriber_class = modules["ChannelSubscriber"]
            native_input = self.config.lio_backend in {"fastlio2", "pointlio"}

            def make_subscriber(topic, message_type, history_depth, *, latest_cloud=False):
                if not native_input:
                    return subscriber_class(topic, message_type)
                required = (
                    "DomainParticipant", "Topic", "DataReader", "ReadCondition", "WaitSet", "GuardCondition",
                    "Qos", "HistoryKeepLast", "ResourceLimits",
                )
                if (
                    any(not callable(modules.get(name)) for name in required)
                    or not isinstance(modules.get("ReadConditionMask"), int)
                ):
                    raise RuntimeError("Go2原生输入缺少公开读取器有界历史接口")
                qos = modules["Qos"](
                    modules["HistoryKeepLast"](history_depth),
                    modules["ResourceLimits"](
                        max_samples=history_depth,
                        max_instances=1,
                        max_samples_per_instance=history_depth,
                    ),
                )
                return _HistoryChannelSubscriber(
                    modules, topic, message_type, qos,
                    read_batch_size=history_depth,
                    latest_sample_key=_point_cloud_source_time_key if latest_cloud else None,
                )

            pose_subscriber = make_subscriber(
                self.config.direct_dds.pose_topic,
                modules["SportModeState"],
                _NATIVE_IMU_READER_HISTORY_DEPTH,
            )
            pose_subscriber.Init()

            if point_cloud_enabled:
                point_cloud_type = modules.get("PointCloud2")
                if point_cloud_type is None:
                    message = (
                        "Go2 DirectDDS 点云已启用，但 unitree_sdk2py PointCloud2 类型不可用；"
                        " 请检查 unitree_sdk2py.idl.sensor_msgs.msg.dds_ 是否被安装。"
                    )
                    LOGGER.warning(message)
                    self._record_point_cloud_bridge_error(message)
                    raise RuntimeError(message)
                point_cloud_subscriber = make_subscriber(
                    self.config.direct_dds.point_cloud_topic,
                    point_cloud_type,
                    _NATIVE_CLOUD_READER_HISTORY_DEPTH,
                    latest_cloud=True,
                )
                point_cloud_subscriber.Init()
                LOGGER.info(
                    "Go2 DirectDDS 点云订阅已启动：topic=%s",
                    self.config.direct_dds.point_cloud_topic,
                )
            else:
                self._record_point_cloud_bridge_error("Go2 DirectDDS 点云订阅未启用")
                LOGGER.info(
                    "Go2 DirectDDS 点云订阅未启用：config.direct_dds.point_cloud_enabled=false，"
                    "需要启用该开关并保证 unitree SDK 提供 sensor_msgs/PointCloud2 IDL。"
                )
            if native_input:
                LOGGER.info(
                    "Go2原生输入读取器已配置有界历史及单实例资源限额：惯性=%s 点云=%s",
                    _NATIVE_IMU_READER_HISTORY_DEPTH,
                    _NATIVE_CLOUD_READER_HISTORY_DEPTH if point_cloud_enabled else 0,
                )
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 订阅器失败：%s", str(exc))
            self._close_direct_dds_subscriber(point_cloud_subscriber, "点云订阅器")
            self._close_direct_dds_subscriber(pose_subscriber, "位姿订阅器")
            with self._lock:
                self._direct_pose_subscriber = None
                self._direct_point_cloud_subscriber = None
                self._direct_point_cloud_msg_type = None
            self._record_point_cloud_bridge_error(f"DirectDDS 订阅器初始化失败：{exc}")
            raise RuntimeError(f"DirectDDS 订阅器初始化失败：{exc}") from exc

        with self._lock:
            self._direct_pose_subscriber = pose_subscriber
            self._direct_point_cloud_subscriber = point_cloud_subscriber
            self._direct_point_cloud_msg_type = point_cloud_type
        self._direct_dds_threads = [
            threading.Thread(
                target=self._direct_pose_reader_loop,
                args=(generation, stop_event, pose_subscriber),
                name=f"go2_direct_pose_reader_g{generation}",
                daemon=True,
            ),
        ]
        if self._direct_point_cloud_subscriber is not None:
            self._direct_dds_threads.append(
                threading.Thread(
                    target=self._direct_point_cloud_reader_loop,
                    args=(generation, stop_event, point_cloud_subscriber),
                    name=f"go2_direct_point_cloud_reader_g{generation}",
                    daemon=True,
                )
            )
        with self._lock:
            if generation != self._active_generation or stop_event is not self._direct_dds_stop_event:
                raise RuntimeError("Go2 DirectDDS 启动代际在订阅初始化期间失效")
            self._direct_dds_started = True
        try:
            for thread in self._direct_dds_threads:
                thread.start()
        except Exception as exc:
            LOGGER.warning("启动 Go2 官方 DDS 读取线程失败：%s", exc)
            self._stop_direct_dds_backend()
            self._record_point_cloud_bridge_error(f"DirectDDS 读取线程启动失败：{exc}")
            raise RuntimeError(f"DirectDDS 读取线程启动失败：{exc}") from exc

    def _stop_direct_dds_backend(self) -> None:
        with self._lock:
            self._direct_dds_started = False
            self._invalidate_observations_locked("Go2 DirectDDS 后端已停止")
            stop_event = self._direct_dds_stop_event
            threads = tuple(self._direct_dds_threads)
            pose_subscriber = self._direct_pose_subscriber
            point_cloud_subscriber = self._direct_point_cloud_subscriber
            self._direct_dds_threads = []
            self._direct_pose_subscriber = None
            self._direct_point_cloud_subscriber = None
            self._direct_point_cloud_msg_type = None
        if stop_event is not None:
            stop_event.set()
        if pose_subscriber is not None:
            try:
                pose_subscriber.Close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 DDS 订阅器失败。", exc_info=True)
        if point_cloud_subscriber is not None and point_cloud_subscriber is not pose_subscriber:
            try:
                point_cloud_subscriber.Close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 DDS 点云订阅器失败。", exc_info=True)
        current_thread = threading.current_thread()
        join_timeout = max(1.0, float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5)) + 0.5)
        for thread in threads:
            if thread is current_thread or not thread.is_alive():
                continue
            thread.join(timeout=join_timeout)
            if thread.is_alive():
                LOGGER.warning("Go2 DirectDDS reader 未在 stop 窗口退出，已由 lifecycle generation 隔离：%s", thread.name)
        with self._lock:
            self._direct_dds_started = False
            self._invalidate_observations_locked("Go2 DirectDDS 后端已停止")

    def _direct_pose_reader_loop(
        self,
        generation: Optional[int] = None,
        stop_event: Optional[threading.Event] = None,
        subscriber: object = None,
    ) -> None:
        timeout_sec = float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5))
        source_topic = f"dds:{self.config.direct_dds.pose_topic}"
        with self._lock:
            generation = self._active_generation if generation is None else generation
            stop_event = self._direct_dds_stop_event if stop_event is None else stop_event
            subscriber = self._direct_pose_subscriber if subscriber is None else subscriber
        if stop_event is None or subscriber is None:
            return
        while not stop_event.is_set():
            read_started = time.monotonic()
            try:
                sample = subscriber.Read(timeout_sec)
            except Exception:
                with self._lock:
                    if not self._is_generation_active_locked(generation, stop_event):
                        return
                LOGGER.debug("读取 Go2 官方 DDS 位姿失败。", exc_info=True)
                stop_event.wait(0.1)
                continue
            if sample is None:
                continue
            read_returned, received_wall_ns = time.monotonic(), time.time_ns()
            with self._lock:
                if not self._is_generation_active_locked(generation, stop_event):
                    return
            try:
                self._on_sport_mode_state(sample, source_topic=source_topic, generation=generation)
            finally:
                self._record_dds_input_timing(
                    "imu", getattr(sample, "stamp", None), generation,
                    read_started, read_returned, received_wall_ns,
                )

    def _on_sport_mode_state(
        self,
        message: object,
        *,
        source_topic: str = "",
        generation: Optional[int] = None,
    ) -> None:
        if generation is None:
            generation = self._active_generation_snapshot()
        if generation is None:
            return
        native_input = self.config.lio_backend in {"fastlio2", "pointlio"}
        canonical_imu_sample = None
        if native_input:
            # 原生估计器只需要每条原始惯性；完整设备状态不参与算法输入。
            stamp = message.stamp
            source_timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            raw_imu = message.imu_state
            try:
                canonical_imu_sample = adapt_go2_l2_imu(
                    calibration_profile=self._calibration_profile,
                    timestamp_ns=source_timestamp_ns,
                    frame_id=self.config.expected_imu_frame,
                    accel_mps2=raw_imu.accelerometer,
                    gyro_rps=raw_imu.gyroscope,
                    source_sequence=source_timestamp_ns,
                    registration_state=RegistrationState.SENSOR,
                )
            except ValueError:
                LOGGER.debug("Go2 IMU canonical 映射失败。", exc_info=True)
            if canonical_imu_sample is not None:
                self._publish_sensor_sample(canonical_imu_sample, generation)
            now = time.monotonic()
            state_key = (generation, getattr(message, "mode", None), getattr(message, "gait_type", None))
            with self._lock:
                if not self._is_generation_active_locked(generation):
                    return
                previous_at = getattr(self, "_native_state_sample_at", None)
                if (
                    previous_at is not None
                    and now - previous_at < _NATIVE_STATE_SAMPLE_INTERVAL_SEC
                    and state_key == getattr(self, "_native_state_sample_key", None)
                ):
                    return
                self._native_state_sample_at = now
                self._native_state_sample_key = state_key
        try:
            pose = self._build_pose_from_sport_mode_state(message)
            imu_state = self._build_imu_from_sport_mode_state(message, source_pose=pose)
        except ValueError:
            if not native_input:
                raise
            LOGGER.debug("设备展示位姿不可用，原生惯性输入继续。", exc_info=True)
            return
        _imu_timestamp, imu_source_timestamp_ns = _unitree_stamp_to_datetime_ns(
            getattr(message, "stamp", None), "Go2 SportModeState"
        )
        if not native_input and imu_state is not None:
            try:
                canonical_imu_sample = adapt_go2_l2_imu(
                    calibration_profile=self._calibration_profile,
                    timestamp_ns=int(imu_source_timestamp_ns),
                    frame_id=imu_state.frame_id,
                    accel_mps2=(
                        float(imu_state.linear_acceleration_m_s2.x),
                        float(imu_state.linear_acceleration_m_s2.y),
                        float(imu_state.linear_acceleration_m_s2.z),
                    ),
                    gyro_rps=(
                        tuple(float(value) for value in message.imu_state.gyroscope[:3])
                        if self.config.lio_backend in {"fastlio2", "pointlio"}
                        else (
                            float(imu_state.angular_velocity_rad_s.x),
                            float(imu_state.angular_velocity_rad_s.y),
                            float(imu_state.angular_velocity_rad_s.z),
                        )
                    ),
                    source_sequence=int(imu_source_timestamp_ns),
                    registration_state=(
                        RegistrationState.SENSOR
                        if self.config.lio_backend in {"fastlio2", "pointlio"}
                        else RegistrationState.BODY
                    ),
                )
            except ValueError:
                LOGGER.debug("Go2 IMU canonical 映射失败。", exc_info=True)
        frame_tree = FrameTree(root_frame_id=pose.frame_id)
        metadata = {
            "source_topic": str(source_topic or ""),
            "source_generation": generation,
            "device_mode": getattr(message, "mode", None),
            "device_gait_type": getattr(message, "gait_type", None),
            "imu_source_topic": f"{source_topic}#imu_state" if source_topic else "",
            "dds_pose_ready": True,
            "dds_imu_ready": imu_state is not None,
            "canonical_imu_sample": canonical_imu_sample,
        }
        with self._lifecycle_lock:
            with self._lock:
                if not self._is_generation_active_locked(generation):
                    return
                stabilized_orientation = self._stabilize_stationary_yaw_locked(
                    message,
                    pose.orientation,
                )
                pose = pose.model_copy(
                    update={"orientation": stabilized_orientation},
                    deep=False,
                )
                imu_state = imu_state.model_copy(
                    update={"orientation": stabilized_orientation},
                    deep=False,
                )
                pose_history = getattr(self, "_pose_history", None)
                if pose_history is None:
                    pose_history = deque(maxlen=_POINT_CLOUD_POSE_HISTORY_SIZE)
                    self._pose_history = pose_history
                pose_history.append(
                    _TimedPoseSnapshot(
                        generation=generation,
                        received_at_monotonic=float(
                            self._stationary_yaw_last_sample_at
                        ),
                        pose=pose.model_copy(deep=False),
                        yaw_stabilization_offset_rad=float(
                            self._stationary_yaw_offset_rad
                        ),
                        stationary_yaw_locked=bool(self._stationary_yaw_locked),
                    )
                )
                metadata["stationary_yaw_locked"] = self._stationary_yaw_locked
                metadata["orientation_yaw_rate_rps"] = self._stationary_yaw_rate_rps
                metadata["yaw_stabilization_offset_rad"] = (
                    self._stationary_yaw_offset_rad
                )
                self._latest_pose_from_dds = pose
                self._latest_imu_state_from_dds = imu_state
                self._latest_pose_generation = generation
                self._latest_imu_generation = generation if imu_state is not None else None
                self._latest_imu_source_timestamp_ns = (
                    imu_source_timestamp_ns if imu_state is not None else None
                )
                self._direct_pose_ready = True
                self._direct_imu_ready = imu_state is not None
                self._latest_pose_source = str(source_topic or "")
                self._latest_imu_source_topic = f"{source_topic}#imu_state" if source_topic else ""
                callbacks = tuple(self._localization_callbacks.values())
                delivery_token = self._direct_dds_stop_event
        if canonical_imu_sample is not None and not native_input:
            self._publish_sensor_sample(canonical_imu_sample, generation)
        for callback in callbacks:
            with self._lock:
                if not self._is_generation_active_locked(generation, delivery_token):
                    return
            try:
                callback(pose.model_copy(deep=False), frame_tree.model_copy(deep=False), dict(metadata))
            except Exception:
                LOGGER.debug("分发 Go2 DDS 定位帧失败。", exc_info=True)

    def _build_pose_from_sport_mode_state(self, message: object) -> Pose:
        imu_state = getattr(message, "imu_state", None)
        quaternion_values = getattr(imu_state, "quaternion", None)
        if imu_state is None or quaternion_values is None:
            raise ValueError("Go2 输入缺少实测 quaternion")
        quaternion = _unitree_wxyz_quaternion_to_contract(quaternion_values)
        position = list(getattr(message, "position", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0])
        while len(position) < 3:
            position.append(0.0)
        expected_odom_frame = str(getattr(self.config, "expected_odom_frame", "") or "").strip()
        if not expected_odom_frame:
            raise ValueError(
                "Go2DataPlaneConfig.expected_odom_frame 必须由 PlatformGeometryConfig.frames.odom_frame 注入；"
                "禁止使用硬编码兜底字符串。"
            )
        return Pose(
            timestamp=_unitree_stamp_to_datetime_ns(
                getattr(message, "stamp", None), "Go2 SportModeState"
            )[0],
            frame_id=expected_odom_frame,
            position=Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2])),
            orientation=quaternion,
        )

    def _build_imu_from_sport_mode_state(
        self, message: object, *, source_pose: Optional[Pose] = None,
    ) -> IMUState:
        imu_state = getattr(message, "imu_state", None)
        quaternion_values = getattr(imu_state, "quaternion", None)
        if imu_state is None or quaternion_values is None:
            raise ValueError("Go2 输入缺少实测 quaternion")
        # 同一实际消息的位姿已经校验四元数和采样时间，惯性快照可直接复用。
        quaternion = (
            source_pose.orientation if source_pose is not None
            else _unitree_wxyz_quaternion_to_contract(quaternion_values)
        )
        gyroscope_values = getattr(imu_state, "gyroscope", None)
        accelerometer_values = getattr(imu_state, "accelerometer", None)
        if gyroscope_values is None or accelerometer_values is None:
            raise ValueError("Go2 输入缺少实测 gyroscope/accelerometer")
        gyroscope = list(gyroscope_values)
        accelerometer = list(accelerometer_values)
        if len(gyroscope) != 3 or len(accelerometer) != 3:
            raise ValueError("Go2 gyroscope/accelerometer 必须各含 3 个轴")
        angular_velocity = [float(value) for value in gyroscope[:3]]
        gyro_deadband_rps = float(self.config.direct_dds.gyro_deadband_rps)
        angular_velocity = [
            0.0 if abs(value) <= gyro_deadband_rps else value
            for value in angular_velocity
        ]
        expected_imu_frame = str(getattr(self.config, "expected_imu_frame", "") or "").strip()
        if not expected_imu_frame:
            raise ValueError(
                "Go2DataPlaneConfig.expected_imu_frame 必须由 assembly.defaults.frame_ids['imu'] 注入；"
                "禁止使用硬编码兜底字符串。"
            )
        timestamp = (
            source_pose.timestamp if source_pose is not None
            else _unitree_stamp_to_datetime_ns(
                getattr(message, "stamp", None), "Go2 SportModeState"
            )[0]
        )
        return IMUState(
            timestamp=timestamp,
            frame_id=expected_imu_frame,
            orientation=quaternion,
            angular_velocity_rad_s=Vector3(
                x=angular_velocity[0],
                y=angular_velocity[1],
                z=angular_velocity[2],
            ),
            linear_acceleration_m_s2=Vector3(
                x=float(accelerometer[0]),
                y=float(accelerometer[1]),
                z=float(accelerometer[2]),
            ),
        )

    # ------------------------------------------------------------------
    # PointCloud2 订阅循环
    # ------------------------------------------------------------------

    def _direct_point_cloud_reader_loop(
        self,
        generation: Optional[int] = None,
        stop_event: Optional[threading.Event] = None,
        subscriber: object = None,
    ) -> None:
        """Go2 官方 sensor_msgs/PointCloud2 直订阅循环。

        Phase 5 硬要求：必须真正填充 ``_latest_local_point_cloud`` 与 ``_point_cloud_callbacks``。
        如果订阅器或消息类型不可用，``_start_direct_dds_backend`` 在启动时记录错误并跳过本线程。
        """
        timeout_sec = float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5))
        source_topic = f"dds:{self.config.direct_dds.point_cloud_topic}"
        with self._lock:
            generation = self._active_generation if generation is None else generation
            stop_event = self._direct_dds_stop_event if stop_event is None else stop_event
            subscriber = self._direct_point_cloud_subscriber if subscriber is None else subscriber
            self._point_cloud_bridge_topic = self.config.direct_dds.point_cloud_topic
        if stop_event is None or subscriber is None:
            return
        while not stop_event.is_set():
            read_started = time.monotonic()
            try:
                sample = subscriber.Read(timeout_sec)
            except Exception as exc:
                with self._lock:
                    if not self._is_generation_active_locked(generation, stop_event):
                        return
                LOGGER.debug("读取 Go2 官方 DDS 点云失败。", exc_info=True)
                self._record_point_cloud_bridge_error(
                    f"PointCloud2 Read 异常：{exc}",
                    generation=generation,
                )
                stop_event.wait(0.1)
                continue
            if sample is None:
                continue
            read_returned, received_wall_ns = time.monotonic(), time.time_ns()
            with self._lock:
                if not self._is_generation_active_locked(generation, stop_event):
                    return
            try:
                self._on_point_cloud_message(
                    sample,
                    source_topic=source_topic,
                    generation=generation,
                )
            except Exception as exc:
                LOGGER.debug("分发 Go2 DDS 点云失败。", exc_info=True)
                self._record_point_cloud_bridge_error(
                    f"PointCloud2 callback 异常：{exc}",
                    generation=generation,
                )
            finally:
                self._record_dds_input_timing(
                    "pointcloud", getattr(getattr(sample, "header", None), "stamp", None), generation,
                    read_started, read_returned, received_wall_ns,
                )

    def _on_point_cloud_message(
        self,
        message: object,
        *,
        source_topic: str,
        generation: Optional[int] = None,
    ) -> None:
        """把 ``sensor_msgs/PointCloud2`` IDL 解析为 ``PointCloudFrame`` 并分发。"""
        if generation is None:
            generation = self._active_generation_snapshot()
        if generation is None:
            return

        def reject(message_text: str) -> None:
            self._record_point_cloud_bridge_error(message_text, generation=generation)

        header = getattr(message, "header", None)
        if header is None:
            reject("PointCloud2 header 缺失")
            return
        source_frame = str(getattr(header, "frame_id", "") or "").strip()
        if not source_frame:
            reject("PointCloud2 header.frame_id 为空")
            return
        upstream_registered_to_odom = bool(
            getattr(
                self.config.direct_dds,
                "point_cloud_xyz_registered_to_odom",
                False,
            )
        )
        point_cloud_received_at = (
            time.monotonic()
            if source_frame == _GO2_UTLIDAR_FRAME
            else None
        )

        stamp = getattr(header, "stamp", None)
        if stamp is None:
            reject("PointCloud2 header.stamp 缺失")
            return
        try:
            source_timestamp, source_timestamp_ns = _unitree_stamp_to_datetime_ns(
                stamp,
                "PointCloud2 header.stamp",
            )
        except ValueError as exc:
            reject(str(exc))
            return

        try:
            decoded = decode_go2_l2_pointcloud(message)
        except Go2L2DecodeError as exc:
            reject(str(exc))
            return
        source_frame = decoded.source_frame
        width = decoded.width
        height = decoded.height
        point_step = decoded.point_step
        row_step = decoded.row_step
        is_dense = decoded.is_dense
        coordinate_fields = decoded.fields
        xyz_array = decoded.xyz_array
        source_xyz_array = xyz_array.copy()
        intensity_array = decoded.intensity_array
        offset_ns_array = decoded.offset_ns_array
        try:
            canonical_lidar_frame = (
                adapt_go2_l2_lidar(
                    decoded,
                    calibration_profile=self._calibration_profile,
                    timestamp_ns=source_timestamp_ns,
                    source_sequence=source_timestamp_ns,
                    registration_state=(
                        RegistrationState.SENSOR
                        if self.config.lio_backend in {"fastlio2", "pointlio"}
                        else RegistrationState.BODY
                    ),
                )
                if source_frame == _GO2_UTLIDAR_FRAME
                else None
            )
        except (Go2L2DecodeError, ValueError) as exc:
            reject(f"Go2 L2 canonical LiDAR 映射失败：{exc}")
            return

        registered_to_odom = False
        frame_id = source_frame
        registered_frame = ""
        registration_assertion = "absent"
        driver_sensor_to_odom_applied = False
        pose_sample_skew_sec: Optional[float] = None
        yaw_stabilization_applied = False
        yaw_stabilization_offset_rad = 0.0
        yaw_stabilization_origin_xy = [0.0, 0.0]
        stationary_yaw_locked = False
        if source_frame == _GO2_UTLIDAR_FRAME and self.config.lio_backend == "device":
            with self._lock:
                if not self._is_generation_active_locked(generation):
                    return
                pose_snapshot, pose_sample_skew_sec = (
                    self._nearest_pose_snapshot_locked(
                        generation=generation,
                        received_at_monotonic=float(point_cloud_received_at),
                    )
                )
            if (
                pose_snapshot is not None
                and pose_sample_skew_sec is not None
                and pose_sample_skew_sec <= _MAX_POINT_CLOUD_POSE_SKEW_SEC
            ):
                registered_frame = str(
                    getattr(self.config, "expected_odom_frame", "") or ""
                ).strip()
                if not registered_frame:
                    reject("点云 driver 注册需要 expected_odom_frame，但配置为空")
                    return
                try:
                    xyz_array = _transform_utlidar_xyz_to_odom(
                        xyz_array,
                        pose_snapshot.pose,
                        self._calibration_profile,
                    )
                except (TypeError, ValueError) as exc:
                    reject(f"utlidar_lidar 到 odom 坐标变换失败：{exc}")
                    return
                registered_to_odom = True
                registration_assertion = "driver_sensor_to_odom"
                driver_sensor_to_odom_applied = True
                frame_id = registered_frame
                yaw_stabilization_offset_rad = (
                    pose_snapshot.yaw_stabilization_offset_rad
                )
                stationary_yaw_locked = pose_snapshot.stationary_yaw_locked
                yaw_stabilization_applied = (
                    abs(yaw_stabilization_offset_rad) > 1e-12
                )
                yaw_stabilization_origin_xy = [
                    float(pose_snapshot.pose.position.x),
                    float(pose_snapshot.pose.position.y),
                ]
        elif source_frame != _GO2_UTLIDAR_FRAME and upstream_registered_to_odom:
            registered_frame = str(getattr(self.config, "expected_odom_frame", "") or "").strip()
            if not registered_frame:
                reject("点云声明已注册到 odom，但 expected_odom_frame 为空")
                return
            registered_to_odom = True
            registration_assertion = "explicit_config"
            frame_id = registered_frame
            with self._lock:
                if not self._is_generation_active_locked(generation):
                    return
                pose_history = getattr(self, "_pose_history", ())
                pose_snapshot = next(
                    (
                        snapshot
                        for snapshot in reversed(pose_history)
                        if snapshot.generation == generation
                    ),
                    None,
                )
                if pose_snapshot is not None:
                    yaw_stabilization_offset_rad = (
                        pose_snapshot.yaw_stabilization_offset_rad
                    )
                    stationary_yaw_locked = pose_snapshot.stationary_yaw_locked
                else:
                    yaw_stabilization_offset_rad = float(
                        getattr(self, "_stationary_yaw_offset_rad", 0.0)
                    )
                    stationary_yaw_locked = bool(
                        getattr(self, "_stationary_yaw_locked", False)
                    )
            if abs(yaw_stabilization_offset_rad) > 1e-12:
                if pose_snapshot is None:
                    reject(
                        "点云需要偏航稳定补偿，但当前 lifecycle 没有同代 odom 位姿"
                    )
                    return
                origin_x = float(pose_snapshot.pose.position.x)
                origin_y = float(pose_snapshot.pose.position.y)
                xyz_array = _rotate_registered_xyz_about_pose(
                    xyz_array,
                    yaw_stabilization_offset_rad,
                    origin_x,
                    origin_y,
                )
                yaw_stabilization_applied = True
                yaw_stabilization_origin_xy = [origin_x, origin_y]

        try:
            point_cloud = PointCloudFrame(
                timestamp=source_timestamp,
                frame_id=frame_id,
                point_count=int(xyz_array.shape[0]),
                fields=[
                    PointField(
                        timestamp=source_timestamp,
                        name=name,
                        offset=int(coordinate_fields[name]["offset"]),
                        datatype=coordinate_fields[name]["contract_datatype"],
                        count=int(coordinate_fields[name]["count"]),
                    )
                    for name in ("x", "y", "z", "intensity", "offset_ns")
                    if name in coordinate_fields
                ],
                points=[],
                points_xyz_array=xyz_array,
                is_dense=is_dense,
                metadata={
                    "source_topic": source_topic,
                    "source_frame": source_frame,
                    "source_timestamp_sec": source_timestamp_ns // 1_000_000_000,
                    "source_timestamp_nanosec": source_timestamp_ns % 1_000_000_000,
                    "source_timestamp_ns": source_timestamp_ns,
                    "width": width,
                    "height": height,
                    "point_step": point_step,
                    "row_step": row_step,
                    "source_fields": [
                        {
                            "name": name,
                            "offset": int(coordinate_fields[name]["offset"]),
                            "datatype": int(coordinate_fields[name]["datatype"]),
                            "count": int(coordinate_fields[name]["count"]),
                        }
                        for name in ("x", "y", "z", "intensity", "offset_ns")
                        if name in coordinate_fields
                    ],
                    "source_xyz_array": source_xyz_array,
                    "intensity_array": intensity_array,
                    "offset_ns_array": offset_ns_array,
                    "canonical_lidar_frame": canonical_lidar_frame,
                    "point_cloud_xyz_registered_to_odom": registered_to_odom,
                    "upstream_point_cloud_xyz_registered_to_odom": upstream_registered_to_odom,
                    "registration_assertion": registration_assertion,
                    "registered_frame": registered_frame,
                    "driver_sensor_to_odom_applied": driver_sensor_to_odom_applied,
                    "pose_sample_skew_sec": pose_sample_skew_sec,
                    "max_pose_sample_skew_sec": _MAX_POINT_CLOUD_POSE_SKEW_SEC,
                    "body_from_sensor_translation_m": (
                        list(self._calibration_profile.lidar_translation_m)
                        if source_frame == _GO2_UTLIDAR_FRAME
                        else []
                    ),
                    "body_from_sensor_rpy_rad": (
                        list(self._calibration_profile.lidar_rpy_rad)
                        if source_frame == _GO2_UTLIDAR_FRAME
                        else []
                    ),
                    "body_from_sensor_rotation_row_major": (
                        self._calibration_profile.lidar_rotation.ravel().tolist()
                        if source_frame == _GO2_UTLIDAR_FRAME
                        else []
                    ),
                    "calibration_ref": (
                        canonical_lidar_frame.calibration_ref
                        if canonical_lidar_frame is not None
                        else None
                    ),
                    "yaw_stabilization_applied": yaw_stabilization_applied,
                    "yaw_stabilization_offset_rad": yaw_stabilization_offset_rad,
                    "yaw_stabilization_origin_xy": yaw_stabilization_origin_xy,
                    "stationary_yaw_locked": stationary_yaw_locked,
                    "source": "direct_pointcloud",
                },
            )
        except Exception as exc:
            reject(f"PointCloudFrame 构造失败：{exc}")
            return

        rejection_error = ""
        with self._lifecycle_lock:
            with self._lock:
                last_accepted_timestamp_ns = self._last_accepted_point_cloud_source_timestamp_ns
                if not self._is_generation_active_locked(generation):
                    return
                if last_accepted_timestamp_ns is not None and source_timestamp_ns <= last_accepted_timestamp_ns:
                    rejection_error = (
                        "PointCloud2 source timestamp 必须严格递增："
                        f"incoming={source_timestamp_ns} "
                        f"last_accepted={last_accepted_timestamp_ns}"
                    )
                    self._invalidate_point_cloud_locked(rejection_error)
                    callbacks = ()
                else:
                    self._latest_local_point_cloud = point_cloud
                    self._latest_point_cloud_generation = generation
                    self._last_accepted_point_cloud_source_timestamp_ns = source_timestamp_ns
                    self._latest_point_cloud_source_topic = source_topic
                    self._direct_point_cloud_ready = True
                    self._point_cloud_bridge_last_error = None
                    self._point_cloud_bridge_last_error_at = None
                    callbacks = tuple(self._point_cloud_callbacks.values())
                delivery_token = self._direct_dds_stop_event

        if rejection_error:
            LOGGER.debug("Go2 DirectDDS 点云帧拒绝：%s", rejection_error)
            return

        if canonical_lidar_frame is not None:
            self._publish_sensor_sample(canonical_lidar_frame, generation)
        for callback in callbacks:
            with self._lock:
                if not self._is_generation_active_locked(generation, delivery_token):
                    return
            try:
                callback(point_cloud.model_copy(deep=False))
            except Exception:
                LOGGER.debug("分发 Go2 DDS 点云回调失败。", exc_info=True)


class _RuntimeDeliveryGate:
    def __init__(self, generation: int) -> None:
        self.generation = int(generation)
        self._condition = threading.Condition(threading.Lock())
        self._active = False
        self._in_flight = 0
        self._in_flight_by_thread: Dict[int, int] = {}

    def activate(self) -> None:
        with self._condition:
            self._active = True

    def try_enter(self) -> bool:
        thread_id = threading.get_ident()
        with self._condition:
            if not self._active:
                return False
            self._in_flight += 1
            self._in_flight_by_thread[thread_id] = self._in_flight_by_thread.get(thread_id, 0) + 1
            return True

    def leave(self) -> None:
        thread_id = threading.get_ident()
        with self._condition:
            count = self._in_flight_by_thread.get(thread_id, 0)
            if count <= 1:
                self._in_flight_by_thread.pop(thread_id, None)
            else:
                self._in_flight_by_thread[thread_id] = count - 1
            self._in_flight -= 1
            self._condition.notify_all()

    def revoke(self) -> None:
        with self._condition:
            self._active = False
            self._condition.notify_all()

    def current_thread_is_delivering(self) -> bool:
        with self._condition:
            return self._in_flight_by_thread.get(threading.get_ident(), 0) > 0

    def wait_until_drained_except_current(self) -> None:
        thread_id = threading.get_ident()
        with self._condition:
            while self._in_flight > self._in_flight_by_thread.get(thread_id, 0):
                self._condition.wait()


@dataclass
class Go2DataPlaneRuntime:
    """Go2 设备侧运行时。"""

    config: "Go2DataPlaneConfig"
    bridge: Optional[Go2DirectDdsBridge] = None

    def __post_init__(self) -> None:
        self.bridge = self.bridge or Go2DirectDdsBridge(self.config)
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._lifecycle_condition = threading.Condition(self._lifecycle_lock)
        self._lifecycle_generation = 0
        self._active_generation: Optional[int] = None
        self._started = False
        self._delivery_gate: Optional[_RuntimeDeliveryGate] = None
        self._stopping = False
        self._stopping_thread_id: Optional[int] = None
        self._stopping_gate: Optional[_RuntimeDeliveryGate] = None
        self._official_robot_state_client = None
        self._official_robot_state_ready = False
        self._official_generation: Optional[int] = None
        self._official_service_states: Dict[str, Dict[str, object]] = {}
        self._startup_diagnostics: List[str] = []

    def start(self) -> None:
        if not self.config.enabled:
            self.stop()
            return
        with self._lifecycle_condition:
            while self._stopping:
                self._lifecycle_condition.wait()
            with self._lock:
                if self._started and self._active_generation is not None:
                    return
                self._lifecycle_generation += 1
                generation = self._lifecycle_generation
                delivery_gate = _RuntimeDeliveryGate(generation)
                self._active_generation = generation
                self._started = False
                self._delivery_gate = delivery_gate
                stale_client = self._clear_official_backend_locked()
                self._startup_diagnostics = []
            self._close_official_client(stale_client)
            try:
                if self.bridge is not None:
                    self.bridge.start()
                self._start_official_backend()
                delivery_gate.activate()
                with self._lock:
                    if (
                        self._active_generation != generation
                        or self._delivery_gate is not delivery_gate
                    ):
                        raise RuntimeError("Go2 data-plane start generation was revoked")
                    self._started = True
            except Exception:
                with self._lock:
                    self._started = False
                    self._active_generation = None
                    if self._delivery_gate is delivery_gate:
                        self._delivery_gate = None
                    client = self._clear_official_backend_locked()
                delivery_gate.revoke()
                if self.bridge is not None:
                    self.bridge.stop()
                self._close_official_client(client)
                delivery_gate.wait_until_drained_except_current()
                raise

    def stop(self) -> None:
        current_thread_id = threading.get_ident()
        waiting_gate = None
        waited_for_other_stop = False
        with self._lifecycle_condition:
            if self._stopping:
                stopping_gate = self._stopping_gate
                if (
                    self._stopping_thread_id == current_thread_id
                    or (
                        stopping_gate is not None
                        and stopping_gate.current_thread_is_delivering()
                    )
                ):
                    return
                waiting_gate = stopping_gate
                waited_for_other_stop = True
                while self._stopping:
                    self._lifecycle_condition.wait()
            else:
                self._stopping = True
                self._stopping_thread_id = current_thread_id
                with self._lock:
                    self._started = False
                    self._active_generation = None
                    delivery_gate = self._delivery_gate
                    self._delivery_gate = None
                    client = self._clear_official_backend_locked()
                self._stopping_gate = delivery_gate
        if waited_for_other_stop:
            if waiting_gate is not None:
                waiting_gate.wait_until_drained_except_current()
            return
        if delivery_gate is not None:
            delivery_gate.revoke()
        try:
            try:
                if self.bridge is not None:
                    self.bridge.stop()
            finally:
                self._close_official_client(client)
            if delivery_gate is not None:
                delivery_gate.wait_until_drained_except_current()
        finally:
            with self._lifecycle_condition:
                self._stopping = False
                self._stopping_thread_id = None
                self._stopping_gate = None
                self._lifecycle_condition.notify_all()

    def is_started(self) -> bool:
        with self._lock:
            return self._started and self._active_generation is not None

    def is_localization_available(self) -> bool:
        return self.get_current_pose() is not None

    def get_current_pose(self) -> Optional[Pose]:
        with self._lock:
            active = self._started and self._active_generation is not None
        return None if not active or self.bridge is None else self.bridge.get_current_pose()

    def get_frame_tree(self) -> Optional[FrameTree]:
        pose = self.get_current_pose()
        if pose is None:
            return None
        return FrameTree(root_frame_id=pose.frame_id)

    def get_localization_state(self) -> Dict[str, Any]:
        pose = self.get_current_pose()
        bridge_status = self.bridge.get_status() if self.bridge is not None else {}
        return {
            "tracking_state": "odom_available" if pose is not None else "odom_lost",
            "map_loaded": False,
            "quality_score": 1.0 if pose is not None else 0.0,
            "last_error": None if pose is not None else "Go2 DDS 位姿尚未收到。",
            "pose_source": bridge_status.get("pose_source"),
            "dds_pose_ready": bridge_status.get("dds_pose_ready"),
        }

    def get_imu_state(self) -> Optional[IMUState]:
        with self._lock:
            active = self._started and self._active_generation is not None
        return None if not active or self.bridge is None else self.bridge.get_imu_state()

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        with self._lock:
            active = self._started and self._active_generation is not None
        if not active or self.bridge is None:
            return None
        return self.bridge.get_latest_local_point_cloud(max_points=max_points)

    def _deliver_if_committed(self, callback: Callable[..., None], *args: object) -> None:
        with self._lock:
            delivery_gate = self._delivery_gate
            generation = self._active_generation
            committed = (
                self._started
                and delivery_gate is not None
                and generation == delivery_gate.generation
            )
        if not committed or delivery_gate is None or not delivery_gate.try_enter():
            return
        try:
            with self._lock:
                if (
                    not self._started
                    or self._active_generation != generation
                    or self._delivery_gate is not delivery_gate
                ):
                    return
            callback(*args)
        finally:
            delivery_gate.leave()

    def subscribe_sensor_sample(self, callback: Callable[[SensorSample], None]) -> Optional[int]:
        if self.bridge is None:
            return None
        return self.bridge.subscribe_sensor_sample(
            lambda sample: self._deliver_if_committed(callback, sample)
        )

    def unsubscribe_sensor_sample(self, callback_id: Optional[int]) -> None:
        if self.bridge is not None and callback_id is not None:
            self.bridge.unsubscribe_sensor_sample(callback_id)

    def subscribe_localization(
        self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]
    ) -> Optional[int]:
        if self.bridge is None:
            return None

        def guarded_callback(
            pose: Pose,
            frame_tree: Optional[FrameTree],
            metadata: Dict[str, object],
        ) -> None:
            self._deliver_if_committed(callback, pose, frame_tree, metadata)

        return self.bridge.subscribe_localization(guarded_callback)

    def unsubscribe_localization(self, callback_id: Optional[int]) -> None:
        if self.bridge is None or callback_id is None:
            return
        self.bridge.unsubscribe_localization(callback_id)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        if self.bridge is None:
            return None

        def guarded_callback(point_cloud: PointCloudFrame) -> None:
            self._deliver_if_committed(callback, point_cloud)

        return self.bridge.subscribe_local_point_cloud(guarded_callback)

    def unsubscribe_local_point_cloud(self, callback_id: Optional[int]) -> None:
        if self.bridge is None or callback_id is None:
            return
        self.bridge.unsubscribe_local_point_cloud(callback_id)

    def get_status(self) -> Dict[str, Any]:
        bridge_status = self.bridge.get_status() if self.bridge is not None else {}
        with self._lock:
            active_generation = self._active_generation if self._started else None
            official_active = (
                active_generation is not None
                and self._official_robot_state_ready
                and self._official_generation == active_generation
            )
            official_service_states = dict(self._official_service_states) if official_active else {}
            startup_diagnostics = list(self._startup_diagnostics)
        return {
            "started": active_generation is not None,
            "enabled": bool(self.config.enabled),
            "lifecycle_generation": self._lifecycle_generation,
            "active_generation": active_generation,
            "official_backend_enabled": self._uses_official_backend(),
            "official_robot_state_ready": official_active,
            "official_service_states": official_service_states,
            "startup_diagnostics": startup_diagnostics,
            "bridge": bridge_status,
        }

    def _clear_official_backend_locked(self) -> object:
        client = self._official_robot_state_client
        self._official_robot_state_client = None
        self._official_robot_state_ready = False
        self._official_generation = None
        self._official_service_states = {}
        return client

    @staticmethod
    def _close_official_client(client: object) -> None:
        close = getattr(client, "Close", None)
        if callable(close):
            try:
                close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 robot_state 客户端失败。", exc_info=True)

    def _uses_official_backend(self) -> bool:
        return bool(getattr(self.config, "official", None) and self.config.official.enabled)

    def _start_official_backend(self) -> None:
        if not self.config.enabled or not self._uses_official_backend():
            return
        with self._lock:
            generation = self._active_generation
        if generation is None:
            raise RuntimeError("Go2 official backend start requires an active lifecycle generation")
        modules = self._load_official_sdk_modules()
        if modules is None:
            raise RuntimeError("导入 Go2 官方 SDK 模块失败")

        channel_factory_initialize = modules["ChannelFactoryInitialize"]
        iface = str(self.config.dds_iface or "").strip()
        try:
            if iface:
                channel_factory_initialize(0, iface)
            else:
                channel_factory_initialize(0)
        except TypeError:
            try:
                channel_factory_initialize(0)
            except Exception as exc:
                message = f"初始化 Go2 官方 DDS 失败：{exc}"
                self._startup_diagnostics.append(message)
                LOGGER.warning(message)
                raise RuntimeError(message) from exc
        except Exception as exc:
            message = f"初始化 Go2 官方 DDS 失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)
            raise RuntimeError(message) from exc

        client = None
        try:
            client = modules["RobotStateClient"]()
            client.SetTimeout(self.config.official.client_timeout_sec)
            client.Init()
            with self._lock:
                if generation != self._active_generation:
                    raise RuntimeError("Go2 official backend generation was revoked")
                self._official_robot_state_client = client
                self._official_robot_state_ready = True
                self._official_generation = generation
            self._refresh_official_service_states()
        except Exception as exc:
            message = f"初始化 Go2 官方 robot_state 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)
            with self._lock:
                published_client = self._clear_official_backend_locked()
            self._close_official_client(published_client if published_client is not None else client)
            raise RuntimeError(message) from exc

    def _load_official_sdk_modules(self) -> Optional[Dict[str, object]]:
        modules = _load_go2_official_sdk_modules(self.config, include_dds_topics=False)
        if modules is not None:
            return modules
        self._startup_diagnostics.append("导入 Go2 官方 SDK 模块失败。")
        return None

    def _refresh_official_service_states(self) -> None:
        with self._lock:
            client = self._official_robot_state_client
            generation = self._official_generation
        if client is None or generation is None:
            return
        try:
            code, items = client.ServiceList()
        except Exception:
            LOGGER.debug("读取 Go2 官方服务列表失败。", exc_info=True)
            return
        if code != 0 or not items:
            return
        service_states: Dict[str, Dict[str, object]] = {}
        for item in items:
            service_states[str(getattr(item, "name", ""))] = {
                "status": int(getattr(item, "status", -1)),
                "protect": bool(getattr(item, "protect", False)),
            }
        with self._lock:
            if (
                client is self._official_robot_state_client
                and generation == self._official_generation
                and generation == self._active_generation
            ):
                self._official_service_states = service_states


@dataclass
class Go2DeviceDataPlane:
    """装配层使用的 Go2 窄设备数据面。"""

    runtime: Go2DataPlaneRuntime

    def start(self) -> None:
        self.runtime.start()

    def stop(self) -> None:
        self.runtime.stop()

    def get_status(self) -> Dict[str, Any]:
        return self.runtime.get_status()

    def is_localization_available(self) -> bool:
        return self.runtime.is_localization_available()

    def get_current_pose(self) -> Optional[Pose]:
        return self.runtime.get_current_pose()

    def get_frame_tree(self) -> Optional[FrameTree]:
        return self.runtime.get_frame_tree()

    def get_localization_state(self) -> Dict[str, Any]:
        return self.runtime.get_localization_state()

    def get_imu_state(self) -> Optional[IMUState]:
        return self.runtime.get_imu_state()

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        return self.runtime.get_latest_local_point_cloud(max_points=max_points)

    def subscribe_localization(
        self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]
    ) -> Optional[int]:
        return self.runtime.subscribe_localization(callback)

    def subscribe_sensor_sample(self, callback: Callable[[SensorSample], None]) -> Optional[int]:
        return self.runtime.subscribe_sensor_sample(callback)

    def unsubscribe_sensor_sample(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_sensor_sample(callback_id)

    def unsubscribe_localization(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_localization(callback_id)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        return self.runtime.subscribe_local_point_cloud(callback)

    def unsubscribe_local_point_cloud(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_local_point_cloud(callback_id)
