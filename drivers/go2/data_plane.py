"""Go2 设备侧数据面。

平台地图、定位、导航和探索主链由 services.geometry / services.mapping /
services.localization / services.navigation 承载；本模块只保留 Go2 官方 DDS 输入、
IMU/点云观测与只读 RobotState 服务状态。
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import logging
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, TYPE_CHECKING

import numpy as np

from contracts.geometry import FrameTree, Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame, PointField, PointFieldDataType
from contracts.robot_state import IMUState
from core.logging_utils import SamplingLogFilter as _SamplingLogFilter

if TYPE_CHECKING:
    from drivers.go2.settings import Go2DataPlaneConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane")
LOGGER.addFilter(_SamplingLogFilter(sample_rate=0.01))


_POINT_FIELD_DTYPES = {
    2: (np.dtype("u1"), PointFieldDataType.UINT8),
    3: (np.dtype("<i2"), PointFieldDataType.INT16),
    4: (np.dtype("<u2"), PointFieldDataType.UINT16),
    5: (np.dtype("<i4"), PointFieldDataType.INT32),
    6: (np.dtype("<u4"), PointFieldDataType.UINT32),
    7: (np.dtype("<f4"), PointFieldDataType.FLOAT32),
    8: (np.dtype("<f8"), PointFieldDataType.FLOAT64),
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
        self._latest_point_cloud_generation: Optional[int] = None
        self._last_accepted_point_cloud_source_timestamp_ns: Optional[int] = None
        self._latest_pose_source = ""
        self._latest_imu_source_topic = ""
        self._latest_point_cloud_source_topic = ""
        self._localization_callbacks: Dict[int, Callable[[Pose, Optional[FrameTree], Dict[str, object]], None]] = {}
        self._point_cloud_callbacks: Dict[int, Callable[[PointCloudFrame], None]] = {}
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
                "dds_pose_ready": pose_active,
                "dds_imu_ready": imu_active,
                "dds_cloud_ready": point_cloud_active,
                "pose_source": self._latest_pose_source if pose_active else "",
                "imu_source_topic": self._latest_imu_source_topic if imu_active else "",
                "point_cloud_source_topic": (self._latest_point_cloud_source_topic if point_cloud_active else ""),
                "latest_point_cloud_points": (self._latest_local_point_cloud.point_count if point_cloud_active else 0),
                "last_accepted_point_cloud_source_timestamp_ns": (self._last_accepted_point_cloud_source_timestamp_ns),
                # Phase 5：§13.4 要求 status 暴露这些字段。
                "point_cloud_bridge_topic": self._point_cloud_bridge_topic,
                "point_cloud_bridge_last_error": self._point_cloud_bridge_last_error,
                "point_cloud_bridge_last_error_at_monotonic": self._point_cloud_bridge_last_error_at,
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
        self._latest_pose_from_dds = None
        self._latest_imu_state_from_dds = None
        self._latest_pose_generation = None
        self._latest_imu_generation = None
        self._latest_pose_source = ""
        self._latest_imu_source_topic = ""
        self._direct_pose_ready = False
        self._direct_imu_ready = False
        self._invalidate_point_cloud_locked(message)

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
            pose_subscriber = subscriber_class(
                self.config.direct_dds.pose_topic,
                modules["SportModeState"],
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
                point_cloud_subscriber = subscriber_class(
                    self.config.direct_dds.point_cloud_topic,
                    point_cloud_type,
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
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 订阅器失败：%s", exc)
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
            with self._lock:
                if not self._is_generation_active_locked(generation, stop_event):
                    return
            self._on_sport_mode_state(sample, source_topic=source_topic, generation=generation)

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
        pose = self._build_pose_from_sport_mode_state(message)
        imu_state = self._build_imu_from_sport_mode_state(message)
        frame_tree = FrameTree(root_frame_id=pose.frame_id)
        metadata = {
            "source_topic": str(source_topic or ""),
            "imu_source_topic": f"{source_topic}#imu_state" if source_topic else "",
            "dds_pose_ready": True,
            "dds_imu_ready": imu_state is not None,
        }
        with self._lifecycle_lock:
            with self._lock:
                if not self._is_generation_active_locked(generation):
                    return
                self._latest_pose_from_dds = pose
                self._latest_imu_state_from_dds = imu_state
                self._latest_pose_generation = generation
                self._latest_imu_generation = generation if imu_state is not None else None
                self._direct_pose_ready = True
                self._direct_imu_ready = imu_state is not None
                self._latest_pose_source = str(source_topic or "")
                self._latest_imu_source_topic = f"{source_topic}#imu_state" if source_topic else ""
                callbacks = tuple(self._localization_callbacks.values())
                delivery_token = self._direct_dds_stop_event
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
        expected_imu_frame = str(getattr(self.config, "expected_imu_frame", "") or "").strip()
        if not expected_imu_frame:
            raise ValueError(
                "Go2DataPlaneConfig.expected_imu_frame 必须由 assembly.defaults.frame_ids['imu'] 注入；"
                "禁止使用硬编码兜底字符串。"
            )
        return IMUState(
            frame_id=expected_imu_frame,
            orientation=quaternion,
            angular_velocity_rad_s=Vector3(x=float(gyroscope[0]), y=float(gyroscope[1]), z=float(gyroscope[2])),
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

        stamp = getattr(header, "stamp", None)
        if stamp is None:
            reject("PointCloud2 header.stamp 缺失")
            return
        try:
            source_sec = int(getattr(stamp, "sec"))
            source_nanosec = int(getattr(stamp, "nanosec"))
        except (AttributeError, TypeError, ValueError) as exc:
            reject(f"PointCloud2 header.stamp.sec/nanosec 非法：{exc}")
            return
        if source_sec < 0 or not 0 <= source_nanosec < 1_000_000_000:
            reject(
                f"PointCloud2 header.stamp 越界：sec={source_sec} nanosec={source_nanosec}"
            )
            return
        try:
            source_timestamp = datetime.fromtimestamp(source_sec, tz=timezone.utc).replace(
                microsecond=source_nanosec // 1_000
            )
        except (OverflowError, OSError, ValueError) as exc:
            reject(f"PointCloud2 header.stamp 无法转换：{exc}")
            return
        source_timestamp_ns = (source_sec * 1_000_000_000) + source_nanosec

        try:
            data = bytes(getattr(message, "data", b"") or b"")
            point_step = int(getattr(message, "point_step", 0) or 0)
            row_step = int(getattr(message, "row_step", 0) or 0)
            width = int(getattr(message, "width", 0) or 0)
            height = int(getattr(message, "height", 0) or 0)
            fields = list(getattr(message, "fields", None) or [])
        except (TypeError, ValueError) as exc:
            reject(f"PointCloud2 基础布局字段非法：{exc}")
            return
        is_bigendian = bool(getattr(message, "is_bigendian", False))
        is_dense = bool(getattr(message, "is_dense", True))

        if is_bigendian:
            reject(
                f"PointCloud2 is_bigendian=True 不被支持，跳过 frame topic={source_topic}"
            )
            return
        if not data:
            reject("PointCloud2 message.data 为空")
            return
        if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0:
            reject(
                "PointCloud2 布局维度必须为正："
                f"width={width} height={height} point_step={point_step} row_step={row_step}"
            )
            return
        minimum_row_step = width * point_step
        if row_step < minimum_row_step:
            reject(
                f"PointCloud2 row_step 小于 width*point_step：row_step={row_step} minimum={minimum_row_step}"
            )
            return
        expected_data_size = height * row_step
        if len(data) != expected_data_size:
            reject(
                f"PointCloud2 data 长度与 height*row_step 不一致：data={len(data)} expected={expected_data_size}"
            )
            return

        coordinate_fields = {}
        for raw_field in fields:
            name = str(getattr(raw_field, "name", "") or "").strip().lower()
            if name not in ("x", "y", "z"):
                continue
            if name in coordinate_fields:
                reject(f"PointCloud2 {name} 字段重复")
                return
            try:
                offset = int(getattr(raw_field, "offset"))
                datatype = int(getattr(raw_field, "datatype"))
                count = int(getattr(raw_field, "count"))
            except (AttributeError, TypeError, ValueError) as exc:
                reject(f"PointCloud2 {name} 字段布局非法：{exc}")
                return
            dtype_spec = _POINT_FIELD_DTYPES.get(datatype)
            if dtype_spec is None:
                reject(f"PointCloud2 {name} 字段 datatype={datatype} 不被支持")
                return
            if count != 1:
                reject(f"PointCloud2 {name} 字段 count 必须为 1，实际为 {count}")
                return
            numpy_dtype, contract_datatype = dtype_spec
            field_end = offset + (numpy_dtype.itemsize * count)
            if offset < 0 or field_end > point_step:
                reject(
                    f"PointCloud2 {name} 字段越过 point_step："
                    f"offset={offset} size={numpy_dtype.itemsize} point_step={point_step}"
                )
                return
            coordinate_fields[name] = {
                "offset": offset,
                "datatype": datatype,
                "count": count,
                "numpy_dtype": numpy_dtype,
                "contract_datatype": contract_datatype,
                "end": field_end,
            }

        missing_fields = [name for name in ("x", "y", "z") if name not in coordinate_fields]
        if missing_fields:
            reject(f"PointCloud2 缺少坐标字段：{','.join(missing_fields)}")
            return
        ranges = [(name, int(spec["offset"]), int(spec["end"])) for name, spec in coordinate_fields.items()]
        for index, (name, start, end) in enumerate(ranges):
            for other_name, other_start, other_end in ranges[index + 1 :]:
                if start < other_end and other_start < end:
                    reject(
                        f"PointCloud2 坐标字段重叠：{name}[{start}:{end}] 与 {other_name}[{other_start}:{other_end}]"
                    )
                    return

        try:
            xyz_array = np.empty((height * width, 3), dtype=np.float32)
            for column, name in enumerate(("x", "y", "z")):
                spec = coordinate_fields[name]
                values = np.ndarray(
                    shape=(height, width),
                    dtype=spec["numpy_dtype"],
                    buffer=data,
                    offset=int(spec["offset"]),
                    strides=(row_step, point_step),
                )
                xyz_array[:, column] = np.ravel(values, order="C")
            xyz_array = np.ascontiguousarray(xyz_array)
        except (TypeError, ValueError) as exc:
            reject(f"PointCloud2 ndarray 解析失败：{exc}")
            return

        registered_to_odom = bool(
            getattr(
                self.config.direct_dds,
                "point_cloud_xyz_registered_to_odom",
                False,
            )
        )
        frame_id = source_frame
        registered_frame = ""
        if registered_to_odom:
            registered_frame = str(getattr(self.config, "expected_odom_frame", "") or "").strip()
            if not registered_frame:
                reject("点云声明已注册到 odom，但 expected_odom_frame 为空")
                return
            frame_id = registered_frame

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
                    for name in ("x", "y", "z")
                ],
                points=[],
                points_xyz_array=xyz_array,
                is_dense=is_dense,
                metadata={
                    "source_topic": source_topic,
                    "source_frame": source_frame,
                    "source_timestamp_sec": source_sec,
                    "source_timestamp_nanosec": source_nanosec,
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
                        for name in ("x", "y", "z")
                    ],
                    "point_cloud_xyz_registered_to_odom": registered_to_odom,
                    "registration_assertion": ("explicit_config" if registered_to_odom else "absent"),
                    "registered_frame": registered_frame,
                    "source": "go2_direct_dds_pointcloud",
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

    def unsubscribe_localization(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_localization(callback_id)

    def subscribe_local_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        return self.runtime.subscribe_local_point_cloud(callback)

    def unsubscribe_local_point_cloud(self, callback_id: Optional[int]) -> None:
        self.runtime.unsubscribe_local_point_cloud(callback_id)
