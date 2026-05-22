"""Go2 设备侧数据面。

平台地图、定位、导航和探索主链由 services.geometry / services.mapping /
services.localization / services.navigation 承载；本模块只保留 Go2 官方 DDS 输入、
IMU/点云观测、运动命令和实时避障控制。
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import logging
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING

import numpy as np

from contracts.geometry import FrameTree, Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame, PointField, PointFieldDataType
from contracts.robot_state import IMUState
from logging_utils import LogRateLimiter

if TYPE_CHECKING:
    from drivers.go2.settings import Go2DataPlaneConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane")

from logging_utils import SamplingLogFilter as _SamplingLogFilter
LOGGER.addFilter(_SamplingLogFilter(sample_rate=0.01))
_GO2_INFO_LOG_RATE_LIMIT_SEC = 30.0
_GO2_MOTION_ERROR_LOG_RATE_LIMIT_SEC = 5.0
_GO2_OBSTACLE_STATUS_REFRESH_INTERVAL_SEC = 5.0
_GO2_OBSTACLE_BEST_EFFORT_ERROR_CODES = {3102, 3104}


_POINT_FIELD_DTYPE_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def _unitree_wxyz_quaternion_to_contract(values: Sequence[float]) -> Quaternion:
    raw_values = [float(item) for item in list(values or [])[:4]]
    while len(raw_values) < 4:
        raw_values.append(0.0)
    if all(abs(item) < 1e-6 for item in raw_values):
        return Quaternion(w=1.0)

    w, x, y, z = raw_values
    norm = float(np.sqrt((w * w) + (x * x) + (y * y) + (z * z)))
    if norm <= 1e-12:
        return Quaternion(w=1.0)
    inv_norm = 1.0 / norm
    return Quaternion(
        x=x * inv_norm,
        y=y * inv_norm,
        z=z * inv_norm,
        w=w * inv_norm,
    )


def _prepend_env_path(name: str, value: str) -> None:
    normalized = str(value or "").strip()
    if not normalized:
        return
    current = str(os.environ.get(name, "") or "").strip()
    entries = [item for item in current.split(":") if item.strip()]
    if normalized in entries:
        entries = [item for item in entries if item != normalized]
    os.environ[name] = ":".join([normalized, *entries]) if entries else normalized


def _prepare_go2_official_sdk_environment(config: "Go2DataPlaneConfig") -> None:
    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    cyclonedds_lib_dir = str(getattr(config, "cyclonedds_lib_dir", "") or "").strip()
    if cyclonedds_lib_dir:
        _prepend_env_path("LD_LIBRARY_PATH", cyclonedds_lib_dir)

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
            "SportClient": getattr(
                importlib.import_module("unitree_sdk2py.go2.sport.sport_client"),
                "SportClient",
            ),
            "RobotStateClient": getattr(
                importlib.import_module("unitree_sdk2py.go2.robot_state.robot_state_client"),
                "RobotStateClient",
            ),
            "ObstaclesAvoidClient": getattr(
                importlib.import_module("unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client"),
                "ObstaclesAvoidClient",
            ),
        }
        if include_dds_topics:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            unitree_go_dds_module = importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
            modules["ChannelSubscriber"] = getattr(channel_module, "ChannelSubscriber")
            modules["SportModeState"] = getattr(unitree_go_dds_module, "SportModeState_")
            modules["DirectPointCloud2"] = getattr(
                importlib.import_module("unitree_sdk2py.idl.sensor_msgs.msg.dds_"),
                "PointCloud2_",
            )
        return modules
    except Exception as exc:
        LOGGER.warning("导入 Go2 官方 SDK 模块失败：%s", exc)
        return None


class RclpyGo2RosBridge:
    """Go2 官方 DDS 原始输入桥。"""

    def __init__(self, config: "Go2DataPlaneConfig") -> None:
        self.config = config
        self._lock = threading.RLock()
        self._started = False
        self._direct_dds_started = False
        self._direct_dds_stop_event = threading.Event()
        self._direct_dds_threads: List[threading.Thread] = []
        self._direct_pose_subscriber = None
        self._direct_point_cloud_subscriber = None
        self._direct_point_cloud_worker_condition = threading.Condition()
        self._direct_point_cloud_pending_sample: Optional[object] = None
        self._direct_point_cloud_pending_source_topic = ""
        self._direct_point_cloud_pending_sequence = 0
        self._direct_point_cloud_processed_sequence = 0
        self._direct_point_cloud_dispatch_condition = threading.Condition()
        self._direct_point_cloud_pending_dispatch_frame: Optional[PointCloudFrame] = None
        self._direct_point_cloud_pending_dispatch_sequence = 0
        self._direct_point_cloud_dispatched_sequence = 0
        self._direct_point_cloud_reader_drop_count = 0
        self._direct_point_cloud_dispatch_drop_count = 0
        self._latest_pose_from_dds: Optional[Pose] = None
        self._latest_imu_state_from_dds: Optional[IMUState] = None
        self._latest_local_point_cloud: Optional[PointCloudFrame] = None
        self._latest_pose_source = ""
        self._latest_imu_source_topic = ""
        self._latest_point_cloud_source_topic = ""
        self._localization_callbacks: Dict[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]] = {}
        self._point_cloud_callbacks: Dict[int, Callable[[PointCloudFrame], None]] = {}
        self._next_callback_id = 1
        self._direct_pose_ready = False
        self._direct_imu_ready = False
        self._direct_point_cloud_ready = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._start_direct_dds_backend()

    def stop(self) -> None:
        if not self._started:
            return
        self._stop_direct_dds_backend()
        self._started = False

    def is_running(self) -> bool:
        return self._started

    def get_current_pose(self) -> Optional[Pose]:
        with self._lock:
            return self._latest_pose_from_dds.model_copy(deep=False) if self._latest_pose_from_dds is not None else None

    def get_imu_state(self) -> Optional[IMUState]:
        with self._lock:
            return self._latest_imu_state_from_dds.model_copy(deep=False) if self._latest_imu_state_from_dds is not None else None

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        with self._lock:
            point_cloud = self._latest_local_point_cloud.model_copy(deep=False) if self._latest_local_point_cloud is not None else None
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

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "started": self._started,
                "direct_dds_started": self._direct_dds_started,
                "dds_pose_ready": self._direct_pose_ready,
                "dds_imu_ready": self._direct_imu_ready,
                "dds_cloud_ready": self._direct_point_cloud_ready,
                "pose_source": self._latest_pose_source,
                "imu_source_topic": self._latest_imu_source_topic,
                "point_cloud_source_topic": self._latest_point_cloud_source_topic,
                "latest_point_cloud_points": self._latest_local_point_cloud.point_count if self._latest_local_point_cloud is not None else 0,
                "dds_point_cloud_reader_drop_count": self._direct_point_cloud_reader_drop_count,
                "dds_point_cloud_dispatch_drop_count": self._direct_point_cloud_dispatch_drop_count,
            }

    def _start_direct_dds_backend(self) -> None:
        if not getattr(self.config, "direct_dds", None) or not self.config.direct_dds.enabled:
            return
        modules = _load_go2_official_sdk_modules(self.config, include_dds_topics=True)
        if modules is None:
            return

        channel_factory_initialize = modules["ChannelFactoryInitialize"]
        iface = str(self.config.dds_iface or "").strip()
        try:
            if iface:
                channel_factory_initialize(0, iface)
            else:
                channel_factory_initialize(0)
        except TypeError:
            channel_factory_initialize(0)
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 直连失败：%s", exc)
            return

        try:
            subscriber_class = modules["ChannelSubscriber"]
            self._direct_pose_subscriber = subscriber_class(
                self.config.direct_dds.pose_topic,
                modules["SportModeState"],
            )
            self._direct_pose_subscriber.Init()
            if self.config.direct_dds.point_cloud_enabled:
                self._direct_point_cloud_subscriber = subscriber_class(
                    self.config.direct_dds.point_cloud_topic,
                    modules["DirectPointCloud2"],
                )
                self._direct_point_cloud_subscriber.Init()
            else:
                LOGGER.info("SDK 点云订阅已禁用，由 Rust bridge 直接订阅 Go2 原生 ROS2 话题")
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 订阅器失败：%s", exc)
            self._direct_pose_subscriber = None
            self._direct_point_cloud_subscriber = None
            return

        self._direct_dds_stop_event.clear()
        threads: List[threading.Thread] = [
            threading.Thread(target=self._direct_pose_reader_loop, name="go2_direct_pose_reader", daemon=True),
        ]
        if self.config.direct_dds.point_cloud_enabled:
            threads.extend([
                threading.Thread(target=self._direct_point_cloud_reader_loop, name="go2_direct_point_cloud_reader", daemon=True),
                threading.Thread(target=self._direct_point_cloud_worker_loop, name="go2_direct_point_cloud_worker", daemon=True),
                threading.Thread(target=self._direct_point_cloud_dispatch_loop, name="go2_direct_point_cloud_dispatch", daemon=True),
            ])
        self._direct_dds_threads = threads
        for thread in self._direct_dds_threads:
            thread.start()
        self._direct_dds_started = True

    def _stop_direct_dds_backend(self) -> None:
        self._direct_dds_stop_event.set()
        with self._direct_point_cloud_worker_condition:
            self._direct_point_cloud_worker_condition.notify_all()
        with self._direct_point_cloud_dispatch_condition:
            self._direct_point_cloud_dispatch_condition.notify_all()
        for thread in self._direct_dds_threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        self._direct_dds_threads = []
        for subscriber in (self._direct_pose_subscriber, self._direct_point_cloud_subscriber):
            if subscriber is None:
                continue
            try:
                subscriber.Close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 DDS 订阅器失败。", exc_info=True)
        self._direct_pose_subscriber = None
        self._direct_point_cloud_subscriber = None
        with self._direct_point_cloud_worker_condition:
            self._direct_point_cloud_pending_sample = None
            self._direct_point_cloud_pending_source_topic = ""
        with self._direct_point_cloud_dispatch_condition:
            self._direct_point_cloud_pending_dispatch_frame = None
        self._direct_dds_started = False

    def _direct_pose_reader_loop(self) -> None:
        timeout_sec = float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5))
        source_topic = f"dds:{self.config.direct_dds.pose_topic}"
        while not self._direct_dds_stop_event.is_set():
            subscriber = self._direct_pose_subscriber
            if subscriber is None:
                return
            try:
                sample = subscriber.Read(timeout_sec)
            except Exception:
                LOGGER.debug("读取 Go2 官方 DDS 位姿失败。", exc_info=True)
                time.sleep(0.1)
                continue
            if sample is None:
                continue
            self._on_sport_mode_state(sample, source_topic=source_topic)

    def _direct_point_cloud_reader_loop(self) -> None:
        timeout_sec = float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5))
        source_topic = f"dds:{self.config.direct_dds.point_cloud_topic}"
        while not self._direct_dds_stop_event.is_set():
            subscriber = self._direct_point_cloud_subscriber
            if subscriber is None:
                return
            try:
                sample = subscriber.Read(timeout_sec)
            except Exception:
                LOGGER.debug("读取 Go2 官方 DDS 点云失败。", exc_info=True)
                time.sleep(0.1)
                continue
            if sample is None:
                continue
            self._queue_direct_point_cloud_sample(sample, source_topic=source_topic)

    def _queue_direct_point_cloud_sample(self, sample: object, *, source_topic: str) -> None:
        with self._direct_point_cloud_worker_condition:
            if self._direct_point_cloud_pending_sequence != self._direct_point_cloud_processed_sequence:
                with self._lock:
                    self._direct_point_cloud_reader_drop_count += 1
            self._direct_point_cloud_pending_sequence += 1
            self._direct_point_cloud_pending_sample = sample
            self._direct_point_cloud_pending_source_topic = str(source_topic or "")
            self._direct_point_cloud_worker_condition.notify()

    def _direct_point_cloud_worker_loop(self) -> None:
        while not self._direct_dds_stop_event.is_set():
            with self._direct_point_cloud_worker_condition:
                while (
                    not self._direct_dds_stop_event.is_set()
                    and self._direct_point_cloud_pending_sequence == self._direct_point_cloud_processed_sequence
                ):
                    self._direct_point_cloud_worker_condition.wait(timeout=0.5)
                if self._direct_dds_stop_event.is_set():
                    return
                sequence = self._direct_point_cloud_pending_sequence
                sample = self._direct_point_cloud_pending_sample
                source_topic = self._direct_point_cloud_pending_source_topic
                self._direct_point_cloud_processed_sequence = sequence
            if sample is None:
                continue
            self._on_direct_point_cloud(sample, source_topic=source_topic)

    def _queue_direct_point_cloud_dispatch(self, point_cloud: PointCloudFrame) -> None:
        with self._direct_point_cloud_dispatch_condition:
            if self._direct_point_cloud_pending_dispatch_sequence != self._direct_point_cloud_dispatched_sequence:
                with self._lock:
                    self._direct_point_cloud_dispatch_drop_count += 1
            self._direct_point_cloud_pending_dispatch_sequence += 1
            self._direct_point_cloud_pending_dispatch_frame = point_cloud
            self._direct_point_cloud_dispatch_condition.notify()

    def _direct_point_cloud_dispatch_loop(self) -> None:
        while not self._direct_dds_stop_event.is_set():
            with self._direct_point_cloud_dispatch_condition:
                while (
                    not self._direct_dds_stop_event.is_set()
                    and self._direct_point_cloud_pending_dispatch_sequence == self._direct_point_cloud_dispatched_sequence
                ):
                    self._direct_point_cloud_dispatch_condition.wait(timeout=0.5)
                if self._direct_dds_stop_event.is_set():
                    return
                sequence = self._direct_point_cloud_pending_dispatch_sequence
                point_cloud = self._direct_point_cloud_pending_dispatch_frame
                self._direct_point_cloud_dispatched_sequence = sequence
            if point_cloud is None:
                continue
            with self._lock:
                callbacks = tuple(self._point_cloud_callbacks.values())
            for callback in callbacks:
                try:
                    callback(point_cloud.model_copy(deep=False))
                except Exception:
                    LOGGER.debug("分发 Go2 DDS 点云帧失败。", exc_info=True)

    def _on_sport_mode_state(self, message: object, *, source_topic: str = "") -> None:
        pose = self._build_pose_from_sport_mode_state(message)
        imu_state = self._build_imu_from_sport_mode_state(message)
        frame_tree = FrameTree(root_frame_id=pose.frame_id)
        metadata = {
            "source_topic": str(source_topic or ""),
            "imu_source_topic": f"{source_topic}#imu_state" if source_topic else "",
            "dds_pose_ready": True,
            "dds_imu_ready": imu_state is not None,
        }
        with self._lock:
            self._latest_pose_from_dds = pose
            self._latest_imu_state_from_dds = imu_state
            self._direct_pose_ready = True
            self._direct_imu_ready = imu_state is not None
            self._latest_pose_source = str(source_topic or "")
            self._latest_imu_source_topic = f"{source_topic}#imu_state" if source_topic else ""
            callbacks = tuple(self._localization_callbacks.values())
        for callback in callbacks:
            try:
                callback(pose.model_copy(deep=False), frame_tree.model_copy(deep=False), dict(metadata))
            except Exception:
                LOGGER.debug("分发 Go2 DDS 定位帧失败。", exc_info=True)

    def _on_direct_point_cloud(self, message: object, *, source_topic: str = "") -> None:
        points_xyz = self._extract_xyz_from_point_cloud(message)
        if points_xyz.size == 0:
            with self._lock:
                self._direct_point_cloud_ready = True
                self._latest_point_cloud_source_topic = str(source_topic or "")
            return
        max_points = max(50, int(getattr(self.config.direct_dds, "max_points_per_scan", 1200)))
        if points_xyz.shape[0] > max_points:
            stride = max(1, int(np.ceil(points_xyz.shape[0] / float(max_points))))
            points_xyz = points_xyz[::stride][:max_points]

        # 高性能路径：使用 ndarray 直接构造，跳过逐点 Vector3 实例化
        point_cloud = PointCloudFrame(
            frame_id=str(getattr(message, "header", None) and getattr(message.header, "frame_id", "") or "utlidar_lidar"),
            point_count=int(points_xyz.shape[0]),
            fields=[
                PointField(name="x", offset=0, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointFieldDataType.FLOAT32, count=1),
            ],
            points_xyz_array=points_xyz.astype(np.float32, copy=False),
            timestamp=datetime.now(timezone.utc),
            metadata={"source_topic": str(source_topic or "")},
        )
        with self._lock:
            self._latest_local_point_cloud = point_cloud
            self._direct_point_cloud_ready = True
            self._latest_point_cloud_source_topic = str(source_topic or "")
        self._queue_direct_point_cloud_dispatch(point_cloud)

    def _build_pose_from_sport_mode_state(self, message: object) -> Pose:
        imu_state = getattr(message, "imu_state", None)
        quaternion = _unitree_wxyz_quaternion_to_contract(
            getattr(imu_state, "quaternion", [1.0, 0.0, 0.0, 0.0]) or [1.0, 0.0, 0.0, 0.0]
        )
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
            frame_id=expected_odom_frame,
            position=Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2])),
            orientation=quaternion,
        )

    def _build_imu_from_sport_mode_state(self, message: object) -> IMUState:
        imu_state = getattr(message, "imu_state", None)
        quaternion = _unitree_wxyz_quaternion_to_contract(
            getattr(imu_state, "quaternion", [1.0, 0.0, 0.0, 0.0]) or [1.0, 0.0, 0.0, 0.0]
        )
        gyroscope = list(getattr(imu_state, "gyroscope", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0])
        accelerometer = list(getattr(imu_state, "accelerometer", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0])
        while len(gyroscope) < 3:
            gyroscope.append(0.0)
        while len(accelerometer) < 3:
            accelerometer.append(0.0)
        return IMUState(
            frame_id="body",
            orientation=quaternion,
            angular_velocity_rad_s=Vector3(x=float(gyroscope[0]), y=float(gyroscope[1]), z=float(gyroscope[2])),
            linear_acceleration_m_s2=Vector3(
                x=float(accelerometer[0]),
                y=float(accelerometer[1]),
                z=float(accelerometer[2]),
            ),
        )

    def _extract_xyz_from_point_cloud(self, message: object) -> np.ndarray:
        fields = list(getattr(message, "fields", []) or [])
        endian_prefix = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
        field_specs: Dict[str, Tuple[int, np.dtype]] = {}
        for field in fields:
            name = str(getattr(field, "name", "") or "").strip()
            if name not in {"x", "y", "z"}:
                continue
            datatype = int(getattr(field, "datatype", 0) or 0)
            count = int(getattr(field, "count", 1) or 1)
            base_dtype = _POINT_FIELD_DTYPE_MAP.get(datatype)
            if base_dtype is None or count != 1:
                continue
            field_specs[name] = (int(getattr(field, "offset", 0)), np.dtype(base_dtype).newbyteorder(endian_prefix))
        if not {"x", "y", "z"}.issubset(field_specs):
            return np.empty((0, 3), dtype=np.float32)

        raw_data = bytes(getattr(message, "data", b"") or b"")
        point_step = int(getattr(message, "point_step", 0) or 0)
        width = int(getattr(message, "width", 0) or 0)
        height = int(getattr(message, "height", 1) or 1)
        declared_count = max(0, width * height)
        if point_step <= 0 or declared_count <= 0 or len(raw_data) < point_step:
            return np.empty((0, 3), dtype=np.float32)
        point_count = min(declared_count, len(raw_data) // point_step)
        max_field_end = max(offset + dtype.itemsize for offset, dtype in field_specs.values())
        if max_field_end > point_step:
            return np.empty((0, 3), dtype=np.float32)

        try:
            dtype = np.dtype(
                {
                    "names": ["x", "y", "z"],
                    "formats": [field_specs[axis][1] for axis in ("x", "y", "z")],
                    "offsets": [field_specs[axis][0] for axis in ("x", "y", "z")],
                    "itemsize": point_step,
                }
            )
            structured = np.frombuffer(raw_data, dtype=dtype, count=point_count)
            points = np.column_stack((structured["x"], structured["y"], structured["z"])).astype(np.float32, copy=False)
        except Exception:
            LOGGER.debug("向量化解析 Go2 DDS 点云失败。", exc_info=True)
            return np.empty((0, 3), dtype=np.float32)
        finite_mask = np.isfinite(points).all(axis=1)
        if not bool(finite_mask.any()):
            return np.empty((0, 3), dtype=np.float32)
        return np.ascontiguousarray(points[finite_mask], dtype=np.float32)


@dataclass
class Go2DataPlaneRuntime:
    """Go2 设备侧运行时。"""

    config: "Go2DataPlaneConfig"
    bridge: Optional[RclpyGo2RosBridge] = None

    def __post_init__(self) -> None:
        self.bridge = self.bridge or RclpyGo2RosBridge(self.config)
        self._started = False
        self._official_sport_client = None
        self._official_robot_state_client = None
        self._official_obstacles_avoid_client = None
        self._official_sport_ready = False
        self._official_robot_state_ready = False
        self._official_obstacles_avoid_ready = False
        self._official_obstacles_avoid_switch_state: Optional[bool] = None
        self._official_obstacles_avoid_switch_state_last_attempt_at: Optional[float] = None
        self._official_obstacles_avoid_switch_state_observed_at: Optional[float] = None
        self._official_obstacles_avoid_switch_state_error: Optional[str] = None
        self._official_obstacles_avoid_switch_state_error_code: Optional[int] = None
        self._official_obstacles_avoid_remote_control = False
        self._obstacle_avoidance_enabled = bool(self.config.official.obstacle_avoidance_default_enabled)
        self._official_service_states: Dict[str, Dict[str, object]] = {}
        self._startup_diagnostics: List[str] = []
        self._last_motion_command_route: Optional[str] = None
        self._info_log_limiter = LogRateLimiter()

    def start(self) -> None:
        if not self.config.enabled:
            return
        if self._started:
            return
        if self.bridge is not None:
            self.bridge.start()
        self._start_official_backend()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self.stop_motion_command()
        if self.bridge is not None:
            self.bridge.stop()
        self._started = False

    def is_started(self) -> bool:
        return self._started

    def is_localization_available(self) -> bool:
        return self.get_current_pose() is not None

    def get_current_pose(self) -> Optional[Pose]:
        return None if self.bridge is None else self.bridge.get_current_pose()

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
        return None if self.bridge is None else self.bridge.get_imu_state()

    def get_latest_local_point_cloud(self, *, max_points: Optional[int] = None) -> Optional[PointCloudFrame]:
        if self.bridge is None:
            return None
        return self.bridge.get_latest_local_point_cloud(max_points=max_points)

    def subscribe_localization(self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]) -> Optional[int]:
        if self.bridge is None:
            return None
        return self.bridge.subscribe_localization(callback)

    def unsubscribe_localization(self, callback_id: Optional[int]) -> None:
        if self.bridge is None or callback_id is None:
            return
        self.bridge.unsubscribe_localization(callback_id)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        if self.bridge is None:
            return None
        return self.bridge.subscribe_local_point_cloud(callback)

    def unsubscribe_local_point_cloud(self, callback_id: Optional[int]) -> None:
        if self.bridge is None or callback_id is None:
            return
        self.bridge.unsubscribe_local_point_cloud(callback_id)

    def can_accept_motion_command(self) -> bool:
        return self._uses_official_backend() and self._official_sport_ready

    def send_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        if not self.can_accept_motion_command():
            raise RuntimeError("当前 Go2 数据面未就绪，不能直接下发运动命令。")
        return self._send_official_motion_command(vx, vy, vyaw)

    def stop_motion_command(self) -> None:
        if not self._uses_official_backend():
            return
        self._stop_official_motion()

    def is_obstacle_avoidance_control_available(self) -> bool:
        return self._official_obstacles_avoid_client is not None and self._official_obstacles_avoid_ready

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, Any]:
        previous_enabled = self._obstacle_avoidance_enabled
        self._obstacle_avoidance_enabled = bool(enabled)
        try:
            self._ensure_official_services_enabled(service_names=("obstacles_avoid",), reason="set_obstacle_avoidance")
            switch_enabled = self._set_obstacle_avoidance_switch(bool(enabled))
            if not enabled and self._official_obstacles_avoid_remote_control:
                self._require_obstacle_avoidance_client().Move(0.0, 0.0, 0.0)
                self._set_obstacle_avoidance_remote_command(False)
        except Exception:
            self._obstacle_avoidance_enabled = previous_enabled
            raise
        LOGGER.debug("官方避障已%s。", "启用" if switch_enabled else "禁用")
        return self.get_obstacle_avoidance_status()

    def is_obstacle_avoidance_enabled(self) -> bool:
        return self._obstacle_avoidance_enabled

    def get_obstacle_avoidance_status(self) -> Dict[str, Any]:
        backend_ready = self.is_obstacle_avoidance_control_available()
        if backend_ready:
            self._refresh_obstacle_avoidance_switch_state_for_status()
        switch_enabled = (
            bool(self._official_obstacles_avoid_switch_state)
            if self._official_obstacles_avoid_switch_state is not None
            else False
        )
        return {
            "obstacle_avoidance_enabled": self._obstacle_avoidance_enabled,
            "default_enabled": bool(getattr(getattr(self.config, "official", None), "obstacle_avoidance_default_enabled", True)),
            "backend_ready": backend_ready,
            "switch_enabled": switch_enabled,
            "switch_state_cached": self._official_obstacles_avoid_switch_state is not None,
            "switch_state_observed": self._official_obstacles_avoid_switch_state_observed_at is not None,
            "switch_state_error": self._official_obstacles_avoid_switch_state_error,
            "switch_state_error_code": self._official_obstacles_avoid_switch_state_error_code,
            "remote_command_enabled": self._official_obstacles_avoid_remote_control,
            "control_path": "obstacles_avoid" if backend_ready and self._obstacle_avoidance_enabled else "sport",
        }

    def get_status(self) -> Dict[str, Any]:
        bridge_status = self.bridge.get_status() if self.bridge is not None else {}
        return {
            "started": self._started,
            "enabled": bool(self.config.enabled),
            "official_backend_enabled": self._uses_official_backend(),
            "official_sport_ready": self._official_sport_ready,
            "official_robot_state_ready": self._official_robot_state_ready,
            "official_obstacles_avoid_ready": self._official_obstacles_avoid_ready,
            "official_service_states": dict(self._official_service_states),
            "obstacle_avoidance": self.get_obstacle_avoidance_status(),
            "startup_diagnostics": list(self._startup_diagnostics),
            "bridge": bridge_status,
        }

    def _uses_official_backend(self) -> bool:
        return bool(getattr(self.config, "official", None) and self.config.official.enabled)

    def _start_official_backend(self) -> None:
        if not self.config.enabled or not self._uses_official_backend():
            return
        modules = self._load_official_sdk_modules()
        if modules is None:
            return

        channel_factory_initialize = modules["ChannelFactoryInitialize"]
        iface = str(self.config.dds_iface or "").strip()
        try:
            if iface:
                channel_factory_initialize(0, iface)
            else:
                channel_factory_initialize(0)
        except TypeError:
            channel_factory_initialize(0)
        except Exception as exc:
            message = f"初始化 Go2 官方 DDS 失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)
            return

        try:
            self._official_sport_client = modules["SportClient"]()
            self._official_sport_client.SetTimeout(self.config.official.client_timeout_sec)
            self._official_sport_client.Init()
            self._official_sport_ready = True
        except Exception as exc:
            message = f"初始化 Go2 官方 sport 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        try:
            self._official_robot_state_client = modules["RobotStateClient"]()
            self._official_robot_state_client.SetTimeout(self.config.official.client_timeout_sec)
            self._official_robot_state_client.Init()
            self._official_robot_state_ready = True
            self._refresh_official_service_states()
        except Exception as exc:
            message = f"初始化 Go2 官方 robot_state 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        try:
            self._official_obstacles_avoid_client = modules["ObstaclesAvoidClient"]()
            self._official_obstacles_avoid_client.SetTimeout(self.config.official.client_timeout_sec)
            self._official_obstacles_avoid_client.Init()
            if hasattr(self._official_obstacles_avoid_client, "GetServerApiVersion"):
                self._official_obstacles_avoid_client.GetServerApiVersion()
            self._official_obstacles_avoid_ready = True
            self._read_obstacle_avoidance_switch_state()
        except Exception as exc:
            message = f"初始化 Go2 官方 obstacles_avoid 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        if self._official_robot_state_ready and self.config.official.auto_start_services:
            self._ensure_official_services_enabled(service_names=self.config.official.service_names, reason="startup_auto_start")
        if self._official_obstacles_avoid_ready:
            try:
                self.set_obstacle_avoidance_enabled(self._obstacle_avoidance_enabled)
            except Exception as exc:
                message = f"同步 Go2 官方避障默认状态失败：{exc}"
                self._startup_diagnostics.append(message)
                LOGGER.warning(message)

    def _load_official_sdk_modules(self) -> Optional[Dict[str, object]]:
        modules = _load_go2_official_sdk_modules(self.config, include_dds_topics=False)
        if modules is not None:
            return modules
        self._startup_diagnostics.append("导入 Go2 官方 SDK 模块失败。")
        return None

    def _refresh_official_service_states(self) -> None:
        client = self._official_robot_state_client
        if client is None:
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
        self._official_service_states = service_states

    def _ensure_official_service_enabled(self, service_name: str) -> None:
        normalized = str(service_name or "").strip()
        if not normalized or self._official_robot_state_client is None:
            return
        try:
            code = self._official_robot_state_client.ServiceSwitch(normalized, True)
        except Exception:
            LOGGER.debug("切换 Go2 官方服务失败 service=%s", normalized, exc_info=True)
            return
        if code != 0:
            message = f"启用 Go2 官方服务失败 service={normalized} code={code}"
            if code in {3102, 3104}:
                self._info_log_limiter.log(
                    LOGGER,
                    logging.INFO,
                    "Go2 官方服务当前未就绪，按最佳努力模式继续",
                    key=("go2_service_best_effort", normalized, code),
                    interval_sec=_GO2_INFO_LOG_RATE_LIMIT_SEC,
                    extra={"event": "go2.service.best_effort", "service_name": normalized, "code": code},
                )
                return
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

    def _ensure_official_services_enabled(self, *, service_names: Sequence[str], reason: str) -> None:
        if not self.config.enabled or not self._uses_official_backend():
            return
        if not self._official_robot_state_ready or self._official_robot_state_client is None:
            return
        normalized_names = tuple(str(item or "").strip() for item in service_names if str(item or "").strip())
        if not normalized_names:
            return
        self._info_log_limiter.log(
            LOGGER,
            logging.INFO,
            "按需检查 Go2 官方服务",
            key=("go2_service_ensure_batch", reason, normalized_names),
            interval_sec=_GO2_INFO_LOG_RATE_LIMIT_SEC,
            extra={"event": "go2.service.ensure", "reason": reason, "services": list(normalized_names)},
        )
        self._refresh_official_service_states()
        for service_name in normalized_names:
            self._ensure_official_service_enabled(service_name)
        self._refresh_official_service_states()

    def _require_obstacle_avoidance_client(self):
        client = self._official_obstacles_avoid_client
        if client is None or not self._official_obstacles_avoid_ready:
            raise RuntimeError("Go2 官方 obstacles_avoid 服务未就绪。")
        return client

    def _read_obstacle_avoidance_switch_state(self) -> bool:
        client = self._require_obstacle_avoidance_client()
        now = time.monotonic()
        self._official_obstacles_avoid_switch_state_last_attempt_at = now
        code, enabled = client.SwitchGet()
        if code != 0:
            self._remember_obstacle_avoidance_switch_state_error(
                f"读取 Go2 官方 obstacles_avoid 状态失败 code={code}",
                code=code,
            )
            raise RuntimeError(f"读取 Go2 官方 obstacles_avoid 状态失败 code={code}")
        self._official_obstacles_avoid_switch_state = bool(enabled)
        self._official_obstacles_avoid_switch_state_observed_at = now
        self._official_obstacles_avoid_switch_state_error = None
        self._official_obstacles_avoid_switch_state_error_code = None
        return bool(enabled)

    def _refresh_obstacle_avoidance_switch_state_for_status(self) -> None:
        now = time.monotonic()
        last_attempt_at = self._official_obstacles_avoid_switch_state_last_attempt_at
        if (
            last_attempt_at is not None
            and now - last_attempt_at < _GO2_OBSTACLE_STATUS_REFRESH_INTERVAL_SEC
        ):
            return
        try:
            self._read_obstacle_avoidance_switch_state()
        except Exception as exc:
            if self._official_obstacles_avoid_switch_state_error != str(exc):
                self._remember_obstacle_avoidance_switch_state_error(
                    str(exc),
                    code=self._extract_obstacle_avoidance_error_code(exc),
                )
            if self._is_obstacle_avoidance_best_effort_error(exc):
                self._info_log_limiter.log(
                    LOGGER,
                    logging.INFO,
                    "读取 Go2 官方避障开关状态暂不可用，沿用缓存状态",
                    key=("go2_obstacles_avoid_status_read_best_effort", str(exc)),
                    interval_sec=_GO2_INFO_LOG_RATE_LIMIT_SEC,
                    extra={
                        "event": "go2.obstacles_avoid.status_read_best_effort",
                        "error": str(exc),
                        "error_code": self._official_obstacles_avoid_switch_state_error_code,
                    },
                )
                return
            self._info_log_limiter.log(
                LOGGER,
                logging.WARNING,
                "读取官方避障开关状态失败: %s",
                exc,
                key=("go2_obstacles_avoid_status_read_error", str(exc)),
                interval_sec=_GO2_INFO_LOG_RATE_LIMIT_SEC,
                extra={
                    "event": "go2.obstacles_avoid.status_read_error",
                    "error": str(exc),
                    "error_code": self._official_obstacles_avoid_switch_state_error_code,
                },
            )

    def _set_obstacle_avoidance_switch(self, enabled: bool) -> bool:
        client = self._require_obstacle_avoidance_client()
        code = client.SwitchSet(bool(enabled))
        if code != 0:
            self._remember_obstacle_avoidance_switch_state_error(
                f"切换 Go2 官方 obstacles_avoid 开关失败 code={code}",
                code=code,
            )
            raise RuntimeError(f"切换 Go2 官方 obstacles_avoid 开关失败 code={code}")
        try:
            return self._read_obstacle_avoidance_switch_state()
        except Exception:
            self._official_obstacles_avoid_switch_state = bool(enabled)
            return bool(enabled)

    def _set_obstacle_avoidance_remote_command(self, enabled: bool) -> None:
        if not enabled and not self._official_obstacles_avoid_remote_control:
            return
        client = self._require_obstacle_avoidance_client()
        code = client.UseRemoteCommandFromApi(bool(enabled))
        if code != 0:
            raise RuntimeError(f"切换 Go2 官方 obstacles_avoid 远程命令模式失败 code={code}")
        self._official_obstacles_avoid_remote_control = bool(enabled)
        LOGGER.debug("Go2 官方避障远程命令模式已%s。", "启用" if enabled else "关闭")

    def _ensure_obstacle_avoidance_active(self) -> bool:
        if not self._obstacle_avoidance_enabled:
            return False
        if not self.is_obstacle_avoidance_control_available():
            return False
        try:
            switch_enabled = self._set_obstacle_avoidance_switch(True)
            if not switch_enabled:
                raise RuntimeError("Go2 官方 obstacles_avoid 已请求开启，但返回状态仍为关闭。")
            if not self._official_obstacles_avoid_remote_control:
                self._set_obstacle_avoidance_remote_command(True)
            return True
        except Exception as exc:
            if not self._is_obstacle_avoidance_best_effort_error(exc):
                raise
            self._official_obstacles_avoid_switch_state = False
            self._info_log_limiter.log(
                LOGGER,
                logging.INFO,
                "Go2 官方避障当前未就绪，当前帧改走 sport 控制链",
                key=("go2_obstacles_avoid_best_effort", str(exc)),
                interval_sec=_GO2_INFO_LOG_RATE_LIMIT_SEC,
                extra={"event": "go2.obstacles_avoid.best_effort", "error": str(exc)},
            )
            try:
                self._release_obstacle_avoidance_remote_command()
            except Exception:
                LOGGER.debug("回收 Go2 官方避障远程命令模式失败。", exc_info=True)
            return False

    def _is_obstacle_avoidance_best_effort_error(self, error: Exception) -> bool:
        code = self._extract_obstacle_avoidance_error_code(error)
        return code in _GO2_OBSTACLE_BEST_EFFORT_ERROR_CODES

    def _extract_obstacle_avoidance_error_code(self, error: Exception) -> Optional[int]:
        message = str(error)
        marker = "code="
        if marker not in message:
            return None
        tail = message.split(marker, 1)[1].strip()
        digits = []
        for char in tail:
            if char.isdigit() or (char == "-" and not digits):
                digits.append(char)
                continue
            break
        if not digits:
            return None
        try:
            return int("".join(digits))
        except ValueError:
            return None

    def _remember_obstacle_avoidance_switch_state_error(self, message: str, *, code: object = None) -> None:
        self._official_obstacles_avoid_switch_state_error = str(message)
        try:
            self._official_obstacles_avoid_switch_state_error_code = int(code) if code is not None else None
        except (TypeError, ValueError):
            self._official_obstacles_avoid_switch_state_error_code = None

    def _release_obstacle_avoidance_remote_command(self) -> None:
        if not self._official_obstacles_avoid_remote_control:
            return
        if not self.is_obstacle_avoidance_control_available():
            self._official_obstacles_avoid_remote_control = False
            return
        self._set_obstacle_avoidance_remote_command(False)

    def _send_official_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        if self._ensure_obstacle_avoidance_active():
            route = "obstacles_avoid"
            code = self._require_obstacle_avoidance_client().Move(float(vx), float(vy), float(vyaw))
            self._log_motion_command_route(route, vx, vy, vyaw, code)
            return code
        if self._official_sport_client is None:
            return -1
        self._release_obstacle_avoidance_remote_command()
        route = "sport"
        code = self._official_sport_client.Move(float(vx), float(vy), float(vyaw))
        self._log_motion_command_route(route, vx, vy, vyaw, code)
        return code

    def _log_motion_command_route(self, route: str, vx: float, vy: float, vyaw: float, code: int) -> None:
        should_log = code != 0 or route != self._last_motion_command_route
        self._last_motion_command_route = route if code == 0 else None
        if not should_log:
            return
        extra = {
            "event": "go2.motion.command_route",
            "route": route,
            "vx": float(vx),
            "vy": float(vy),
            "vyaw": float(vyaw),
            "code": code,
            "obstacle_avoidance_enabled": self._obstacle_avoidance_enabled,
            "remote_command_enabled": self._official_obstacles_avoid_remote_control,
        }
        if code != 0:
            self._info_log_limiter.log(
                LOGGER,
                logging.WARNING,
                "Go2 运动命令下发异常",
                key=("go2_motion_command_error", route, code),
                interval_sec=_GO2_MOTION_ERROR_LOG_RATE_LIMIT_SEC,
                extra=extra,
            )
            return
        LOGGER.debug("Go2 运动命令链路已切换", extra=extra)

    def _stop_official_motion(self) -> None:
        self._last_motion_command_route = None
        if self.is_obstacle_avoidance_control_available() and self._official_obstacles_avoid_remote_control:
            try:
                self._require_obstacle_avoidance_client().Move(0.0, 0.0, 0.0)
            except Exception:
                LOGGER.debug("停止 Go2 官方避障运动失败。", exc_info=True)
            try:
                self._release_obstacle_avoidance_remote_command()
            except Exception:
                LOGGER.debug("释放 Go2 官方避障远程命令模式失败。", exc_info=True)
        if self._official_sport_client is None:
            return
        try:
            self._official_sport_client.StopMove()
        except Exception:
            LOGGER.debug("停止 Go2 官方运动失败。", exc_info=True)


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

    def subscribe_localization(self, callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]) -> Optional[int]:
        return self.runtime.subscribe_localization(callback)

    def unsubscribe_localization(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_localization(callback_id)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        return self.runtime.subscribe_local_point_cloud(callback)

    def unsubscribe_local_point_cloud(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_local_point_cloud(callback_id)

    def can_accept_motion_command(self) -> bool:
        return self.runtime.can_accept_motion_command()

    def send_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        return self.runtime.send_motion_command(vx, vy, vyaw)

    def stop_motion_command(self) -> None:
        self.runtime.stop_motion_command()

    def is_obstacle_avoidance_control_available(self) -> bool:
        return self.runtime.is_obstacle_avoidance_control_available()

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, Any]:
        return self.runtime.set_obstacle_avoidance_enabled(enabled)
