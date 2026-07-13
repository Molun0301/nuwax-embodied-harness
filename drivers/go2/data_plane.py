"""Go2 设备侧数据面。

平台地图、定位、导航和探索主链由 services.geometry / services.mapping /
services.localization / services.navigation 承载；本模块只保留 Go2 官方 DDS 输入、
IMU/点云观测与只读 RobotState 服务状态。
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import logging
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, TYPE_CHECKING

import numpy as np

from contracts.geometry import FrameTree, Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import IMUState
from core.logging_utils import SamplingLogFilter as _SamplingLogFilter

if TYPE_CHECKING:
    from drivers.go2.settings import Go2DataPlaneConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane")
LOGGER.addFilter(_SamplingLogFilter(sample_rate=0.01))



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
                sensor_pc_module = importlib.import_module(
                    "unitree_sdk2py.idl.sensor_msgs.msg.dds_"
                )
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
        self._started = False
        self._direct_dds_started = False
        self._direct_dds_stop_event = threading.Event()
        self._direct_dds_threads: List[threading.Thread] = []
        self._direct_pose_subscriber = None
        self._direct_point_cloud_subscriber = None
        self._direct_point_cloud_msg_type = None
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
        # Phase 5：点云 bridge 错误显式记录，禁止 silent fallback。
        self._point_cloud_bridge_last_error: Optional[str] = None
        self._point_cloud_bridge_last_error_at: Optional[float] = None
        self._point_cloud_bridge_topic: str = ""

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
                # Phase 5：§13.4 要求 status 暴露这些字段。
                "point_cloud_bridge_topic": self._point_cloud_bridge_topic,
                "point_cloud_bridge_last_error": self._point_cloud_bridge_last_error,
                "point_cloud_bridge_last_error_at_monotonic": self._point_cloud_bridge_last_error_at,
            }

    def _record_point_cloud_bridge_error(self, message: str) -> None:
        with self._lock:
            self._point_cloud_bridge_last_error = str(message)
            self._point_cloud_bridge_last_error_at = time.monotonic()

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

        point_cloud_enabled = bool(self.config.direct_dds.point_cloud_enabled)
        try:
            subscriber_class = modules["ChannelSubscriber"]
            self._direct_pose_subscriber = subscriber_class(
                self.config.direct_dds.pose_topic,
                modules["SportModeState"],
            )
            self._direct_pose_subscriber.Init()

            self._direct_point_cloud_subscriber = None
            self._direct_point_cloud_msg_type = None
            if point_cloud_enabled:
                point_cloud_type = modules.get("PointCloud2")
                if point_cloud_type is None:
                    message = (
                        "Go2 DirectDDS 点云已启用，但 unitree_sdk2py PointCloud2 类型不可用；"
                        " 请检查 unitree_sdk2py.idl.sensor_msgs.msg.dds_ 是否被安装。"
                    )
                    LOGGER.warning(message)
                    self._record_point_cloud_bridge_error(message)
                else:
                    self._direct_point_cloud_msg_type = point_cloud_type
                    self._direct_point_cloud_subscriber = subscriber_class(
                        self.config.direct_dds.point_cloud_topic,
                        point_cloud_type,
                    )
                    self._direct_point_cloud_subscriber.Init()
                    LOGGER.info(
                        "Go2 DirectDDS 点云订阅已启动：topic=%s",
                        self.config.direct_dds.point_cloud_topic,
                    )
            else:
                LOGGER.info(
                    "Go2 DirectDDS 点云订阅未启用：config.direct_dds.point_cloud_enabled=false，"
                    "需要启用该开关并保证 unitree SDK 提供 sensor_msgs/PointCloud2 IDL。"
                )
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 订阅器失败：%s", exc)
            self._record_point_cloud_bridge_error(f"DirectDDS 订阅器初始化失败：{exc}")
            self._direct_pose_subscriber = None
            return

        self._direct_dds_stop_event.clear()
        self._direct_dds_threads = [
            threading.Thread(target=self._direct_pose_reader_loop, name="go2_direct_pose_reader", daemon=True),
        ]
        if self._direct_point_cloud_subscriber is not None:
            self._direct_dds_threads.append(
                threading.Thread(
                    target=self._direct_point_cloud_reader_loop,
                    name="go2_direct_point_cloud_reader",
                    daemon=True,
                )
            )
        for thread in self._direct_dds_threads:
            thread.start()
        self._direct_dds_started = True

    def _stop_direct_dds_backend(self) -> None:
        self._direct_dds_stop_event.set()
        for thread in self._direct_dds_threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        self._direct_dds_threads = []
        if self._direct_pose_subscriber is not None:
            try:
                self._direct_pose_subscriber.Close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 DDS 订阅器失败。", exc_info=True)
        self._direct_pose_subscriber = None
        if self._direct_point_cloud_subscriber is not None:
            try:
                self._direct_point_cloud_subscriber.Close()
            except Exception:
                LOGGER.debug("关闭 Go2 官方 DDS 点云订阅器失败。", exc_info=True)
        self._direct_point_cloud_subscriber = None
        self._direct_point_cloud_msg_type = None
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

    def _direct_point_cloud_reader_loop(self) -> None:
        """Go2 官方 sensor_msgs/PointCloud2 直订阅循环。

        Phase 5 硬要求：必须真正填充 ``_latest_local_point_cloud`` 与 ``_point_cloud_callbacks``。
        如果订阅器或消息类型不可用，``_start_direct_dds_backend`` 在启动时记录错误并跳过本线程。
        """
        timeout_sec = float(getattr(self.config.direct_dds, "read_timeout_sec", 0.5))
        source_topic = f"dds:{self.config.direct_dds.point_cloud_topic}"
        with self._lock:
            self._point_cloud_bridge_topic = self.config.direct_dds.point_cloud_topic
        while not self._direct_dds_stop_event.is_set():
            subscriber = self._direct_point_cloud_subscriber
            if subscriber is None:
                return
            try:
                sample = subscriber.Read(timeout_sec)
            except Exception as exc:
                LOGGER.debug("读取 Go2 官方 DDS 点云失败。", exc_info=True)
                self._record_point_cloud_bridge_error(f"PointCloud2 Read 异常：{exc}")
                time.sleep(0.1)
                continue
            if sample is None:
                continue
            try:
                self._on_point_cloud_message(sample, source_topic=source_topic)
            except Exception as exc:
                LOGGER.debug("分发 Go2 DDS 点云失败。", exc_info=True)
                self._record_point_cloud_bridge_error(f"PointCloud2 callback 异常：{exc}")

    def _on_point_cloud_message(self, message: object, *, source_topic: str) -> None:
        """把 ``sensor_msgs/PointCloud2`` IDL 解析为 ``PointCloudFrame`` 并分发。"""
        data = bytes(getattr(message, "data", b"") or b"")
        point_step = int(getattr(message, "point_step", 0) or 0)
        width = int(getattr(message, "width", 0) or 0)
        height = int(getattr(message, "height", 0) or 0)
        is_bigendian = bool(getattr(message, "is_bigendian", False))
        is_dense = bool(getattr(message, "is_dense", True))

        if not data:
            self._record_point_cloud_bridge_error("PointCloud2 message.data 为空")
            return
        if point_step <= 0:
            # 兜底：假定 xyz float32 little-endian
            point_step = 12
        if is_bigendian:
            # 当前 LCM/Rust 端只接受 little-endian；这里记录一次并跳过本帧，避免 silent fallback。
            self._record_point_cloud_bridge_error(
                f"PointCloud2 is_bigendian=True 不被支持，跳过 frame topic={source_topic}"
            )
            return

        xyz_array: Optional[np.ndarray] = None
        try:
            count = len(data) // point_step
            if count <= 0:
                self._record_point_cloud_bridge_error(
                    f"PointCloud2 data/point_step 比例异常 data={len(data)} step={point_step}"
                )
                return
            flat = np.frombuffer(data, dtype=np.float32, count=count * 3).reshape(count, 3)
            xyz_array = np.ascontiguousarray(flat)
        except Exception as exc:
            self._record_point_cloud_bridge_error(f"PointCloud2 ndarray 解析失败：{exc}")
            return

        frame_id = str(getattr(self.config, "expected_odom_frame", "") or "odom")
        try:
            point_cloud = PointCloudFrame(
                frame_id=frame_id,
                point_count=int(xyz_array.shape[0]),
                fields=[],
                points=[],
                points_xyz_array=xyz_array,
                is_dense=is_dense,
                metadata={
                    "source_topic": source_topic,
                    "width": width,
                    "height": height,
                    "point_step": point_step,
                    "source": "go2_direct_dds_pointcloud",
                },
            )
        except Exception as exc:
            self._record_point_cloud_bridge_error(f"PointCloudFrame 构造失败：{exc}")
            return

        with self._lock:
            self._latest_local_point_cloud = point_cloud
            self._latest_point_cloud_source_topic = source_topic
            self._direct_point_cloud_ready = True
            callbacks = tuple(self._point_cloud_callbacks.values())

        for callback in callbacks:
            try:
                callback(point_cloud.model_copy(deep=False))
            except Exception:
                LOGGER.debug("分发 Go2 DDS 点云回调失败。", exc_info=True)


@dataclass
class Go2DataPlaneRuntime:
    """Go2 设备侧运行时。"""

    config: "Go2DataPlaneConfig"
    bridge: Optional[Go2DirectDdsBridge] = None

    def __post_init__(self) -> None:
        self.bridge = self.bridge or Go2DirectDdsBridge(self.config)
        self._started = False
        self._official_robot_state_client = None
        self._official_robot_state_ready = False
        self._official_service_states: Dict[str, Dict[str, object]] = {}
        self._startup_diagnostics: List[str] = []

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

    def get_status(self) -> Dict[str, Any]:
        bridge_status = self.bridge.get_status() if self.bridge is not None else {}
        return {
            "started": self._started,
            "enabled": bool(self.config.enabled),
            "official_backend_enabled": self._uses_official_backend(),
            "official_robot_state_ready": self._official_robot_state_ready,
            "official_service_states": dict(self._official_service_states),
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
            self._official_robot_state_client = modules["RobotStateClient"]()
            self._official_robot_state_client.SetTimeout(self.config.official.client_timeout_sec)
            self._official_robot_state_client.Init()
            self._official_robot_state_ready = True
            self._refresh_official_service_states()
        except Exception as exc:
            message = f"初始化 Go2 官方 robot_state 客户端失败：{exc}"
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
