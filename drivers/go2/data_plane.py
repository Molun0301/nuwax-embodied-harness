from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
import importlib
import importlib.util
import json
import logging
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.navigation import ExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.pointcloud import PointCloudFrame, PointField, PointFieldDataType
from contracts.robot_state import IMUState
from drivers.go2.collision_detector import Go2CollisionDetector, Go2CollisionDetectorConfig, CollisionStatus
from drivers.go2.cost_mapper import Go2CostMapper, Go2CostMapperConfig, Go2CostMapperResult
from drivers.go2.voxel_mapper import Go2VoxelMapper, Go2VoxelMapperConfig, Go2VoxelMapperResult
from drivers.go2.local_planner import Go2DWALocalPlanner, Go2LocalPlannerConfig, Go2VelocityCommand, LocalPlannerState
from services.mapping.occupancy_mapping import (
    OccupancyMapBuildResult as Go2GlobalMapBuildResult,
    PointCloudScanObservation as LocalMapScan,
    SparseOccupancyMapBuilder as Go2SparseGlobalMapBuilder,
    build_sliding_window_map,
)
from services.navigation.config import ExplorationConfig
from services.navigation.frontier import FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner, NavigationSession
from logging_utils import LogRateLimiter

if TYPE_CHECKING:
    from drivers.go2.settings import Go2DataPlaneConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane")
_GO2_INFO_LOG_RATE_LIMIT_SEC = 30.0
_GO2_MOTION_ERROR_LOG_RATE_LIMIT_SEC = 5.0
_GO2_SELF_RING_ANGLE_BIN_COUNT = 24
_GO2_SELF_RING_MIN_COVERAGE_RATIO = 0.75
_GO2_SELF_RING_MAX_RADIAL_SPREAD_M = 0.18
_GO2_SELF_RING_MAX_RADIUS_M = 1.10

_KNOWN_GRID_MAP_LAYERS = {
    "elevation",
    "variance",
    "traversability",
    "normal_x",
    "normal_y",
    "normal_z",
    "upper_bound",
    "time",
    "min_filter",
    "smooth",
    "inpaint",
    "erosion",
    "rgb",
}

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
    """把宇树官方 `wxyz` 顺序四元数转成平台统一的 `xyzw` 合同。"""

    raw_values = [float(item) for item in list(values or [])[:4]]
    while len(raw_values) < 4:
        raw_values.append(0.0)
    if all(abs(item) < 1e-6 for item in raw_values):
        return Quaternion(w=1.0)

    w, x, y, z = raw_values
    norm = math.sqrt((w * w) + (x * x) + (y * y) + (z * z))
    if norm <= 1e-12:
        return Quaternion(w=1.0)
    inv_norm = 1.0 / norm
    return Quaternion(
        x=x * inv_norm,
        y=y * inv_norm,
        z=z * inv_norm,
        w=w * inv_norm,
    )


def _quaternion_rotation_matrix(rotation: Quaternion) -> np.ndarray:
    """返回合同四元数对应的 3x3 旋转矩阵。"""

    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-12:
        return np.identity(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.asarray(
        [
            [1.0 - (2.0 * ((y * y) + (z * z))), 2.0 * ((x * y) - (z * w)), 2.0 * ((x * z) + (y * w))],
            [2.0 * ((x * y) + (z * w)), 1.0 - (2.0 * ((x * x) + (z * z))), 2.0 * ((y * z) - (x * w))],
            [2.0 * ((x * z) - (y * w)), 2.0 * ((y * z) + (x * w)), 1.0 - (2.0 * ((x * x) + (y * y)))],
        ],
        dtype=np.float64,
    )

def _prepend_env_path(name: str, value: str) -> None:
    """把路径前置到指定环境变量。"""

    normalized = str(value or "").strip()
    if not normalized:
        return
    current = str(os.environ.get(name, "") or "").strip()
    entries = [item for item in current.split(":") if item.strip()]
    if normalized in entries:
        entries = [item for item in entries if item != normalized]
    os.environ[name] = ":".join([normalized, *entries]) if entries else normalized


def _prepare_go2_official_sdk_environment(config: "Go2DataPlaneConfig") -> None:
    """准备 unitree_sdk2py 导入所需的 Go2 官方运行环境。"""

    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    cyclonedds_lib_dir = str(getattr(config, "cyclonedds_lib_dir", "") or "").strip()
    if cyclonedds_lib_dir:
        _prepend_env_path("LD_LIBRARY_PATH", cyclonedds_lib_dir)

    importlib.invalidate_caches()


def _is_ros_environment_active() -> bool:
    """判断当前进程是否已经进入 ROS2 环境。"""

    return any(
        str(os.environ.get(name, "") or "").strip()
        for name in ("ROS_VERSION", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_DISTRO")
    )


def _uses_sdk2_in_process(config: "Go2DataPlaneConfig") -> bool:
    """判断当前进程是否计划启用 Unitree SDK2。"""

    direct_dds_enabled = bool(getattr(getattr(config, "direct_dds", None), "enabled", False))
    official_backend_enabled = bool(getattr(getattr(config, "official", None), "enabled", False))
    return direct_dds_enabled or official_backend_enabled


def _sdk2_ros_conflict_message() -> str:
    """返回 SDK2 与 ROS2 进程冲突说明。"""

    return (
        "根据宇树官方约束，Unitree SDK2 与 ROS2 环境不能在同一进程中同时初始化；"
        "请在未进入 ROS2 环境的终端中启动宿主机网关，ROS2 能力需要独立进程侧车。"
    )


def _load_go2_official_sdk_modules(
    config: "Go2DataPlaneConfig",
    *,
    include_dds_topics: bool = False,
) -> Optional[Dict[str, object]]:
    """加载 Go2 官方 SDK 模块。"""

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
            modules["ChannelSubscriber"] = getattr(channel_module, "ChannelSubscriber")
            modules["SportModeState"] = getattr(
                importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_"),
                "SportModeState_",
            )
            modules["DirectPointCloud2"] = getattr(
                importlib.import_module("unitree_sdk2py.idl.sensor_msgs.msg.dds_"),
                "PointCloud2_",
            )
        return modules
    except Exception as exc:
        LOGGER.warning("导入 Go2 官方 SDK 模块失败：%s", exc)
        return None


@dataclass
class ManagedRos2Process:
    """受控的 ROS2 子进程。"""

    name: str
    command: str
    setup_script: str = ""
    cyclonedds_lib_dir: str = ""
    process: Optional[subprocess.Popen] = field(default=None, init=False, repr=False)

    def start(self) -> None:
        """启动子进程。"""

        if self.process is not None and self.process.poll() is None:
            return
        shell_parts: List[str] = []
        if self.setup_script.strip():
            shell_parts.append(f"source {self.setup_script}")
        if self.cyclonedds_lib_dir.strip():
            quoted_lib_dir = shlex.quote(self.cyclonedds_lib_dir)
            shell_parts.append(f"export LD_LIBRARY_PATH={quoted_lib_dir}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}")
        shell_parts.append(self.command)
        shell_command = " && ".join(shell_parts)
        LOGGER.info("启动 ROS2 子进程 name=%s command=%s", self.name, shell_command)
        self.process = subprocess.Popen(
            ["bash", "-lc", shell_command],
            start_new_session=True,
            stdout=None,
            stderr=None,
        )

    def stop(self, timeout_sec: float = 5.0) -> None:
        """停止子进程。"""

        if self.process is None or self.process.poll() is not None:
            return
        LOGGER.info("停止 ROS2 子进程 name=%s pid=%s", self.name, self.process.pid)
        self.process.terminate()
        try:
            self.process.wait(timeout=max(0.1, timeout_sec))
        except subprocess.TimeoutExpired:
            LOGGER.warning("ROS2 子进程超时未退出，强制杀死 name=%s pid=%s", self.name, self.process.pid)
            self.process.kill()
            self.process.wait(timeout=2.0)


@dataclass(frozen=True)
class _SelectedMapLayers:
    """桥层当前对外暴露的统一地图图层集合。"""

    source_name: str = ""
    occupancy_grid: Optional[OccupancyGrid] = None
    cost_map: Optional[CostMap] = None
    semantic_map: Optional[SemanticMap] = None

    def has_any_layer(self) -> bool:
        return self.occupancy_grid is not None or self.cost_map is not None or self.semantic_map is not None

    def is_running(self) -> bool:
        """返回子进程是否仍在运行。"""

        return self.process is not None and self.process.poll() is None


class RclpyGo2RosBridge:
    """Go2 端侧数据桥。

    优先直接消费宇树官方 DDS 位姿与点云，再用 ROS2 话题补齐 TF、地图层和兼容输入。
    """

    def __init__(self, config: "Go2DataPlaneConfig") -> None:
        self.config = config
        self._lock = threading.RLock()
        self._started = False
        self._owns_rclpy_context = False
        self._rclpy = None
        self._node = None
        self._executor = None
        self._executor_thread: Optional[threading.Thread] = None
        self._ros_started = False
        self._ros_in_process_disabled_reason = ""
        self._direct_dds_started = False
        self._direct_dds_stop_event = threading.Event()
        self._direct_dds_threads: List[threading.Thread] = []
        self._direct_pose_subscriber = None
        self._direct_point_cloud_subscriber = None
        self._action_client = None
        self._goal_handle = None
        self._current_goal: Optional[NavigationGoal] = None
        self._latest_pose: Optional[Pose] = None
        self._latest_pose_from_ros: Optional[Pose] = None
        self._latest_pose_from_dds: Optional[Pose] = None
        self._latest_pose_source = ""
        self._latest_imu_state: Optional[IMUState] = None
        self._latest_imu_state_from_ros: Optional[IMUState] = None
        self._latest_imu_state_from_dds: Optional[IMUState] = None
        self._tf_transforms: Dict[Tuple[str, str], Transform] = {}
        self._latest_occupancy_direct: Optional[OccupancyGrid] = None
        self._latest_occupancy_from_grid: Optional[OccupancyGrid] = None
        self._latest_occupancy_from_global_cloud: Optional[OccupancyGrid] = None
        self._latest_occupancy_from_local_cloud: Optional[OccupancyGrid] = None
        self._latest_cost_direct: Optional[CostMap] = None
        self._latest_cost_from_grid: Optional[CostMap] = None
        self._latest_cost_from_global_cloud: Optional[CostMap] = None
        self._latest_cost_from_local_cloud: Optional[CostMap] = None
        self._latest_semantic_map: Optional[SemanticMap] = None
        self._latest_semantic_map_from_global_cloud: Optional[SemanticMap] = None
        self._latest_semantic_map_from_local_cloud: Optional[SemanticMap] = None
        self._latest_navigation_state = NavigationState(status=NavigationStatus.IDLE)
        self._goal_reached = False
        self._last_error: Optional[str] = None
        self._ros_setup_loaded = False
        self._latest_odom_source_topic = ""
        self._latest_imu_source_topic = ""
        self._latest_point_cloud_source_topic = ""
        self._latest_global_map_source = ""
        self._latest_local_map_source = ""
        self._point_cloud_available = False
        self._direct_pose_ready = False
        self._ros_pose_ready = False
        self._direct_imu_ready = False
        self._ros_imu_ready = False
        self._direct_point_cloud_ready = False
        self._ros_point_cloud_ready = False
        self._ros_occupancy_ready = False
        self._ros_grid_map_ready = False
        self._ros_cost_map_ready = False
        self._global_map_ready = False
        self._local_map_ready = False
        self._global_map_builder = Go2SparseGlobalMapBuilder(self.config.map_synthesis)
        self._local_map_scans: Deque[LocalMapScan] = deque()
        self._last_global_map_update_monotonic = 0.0
        self._last_local_map_update_monotonic = 0.0
        self._last_voxel_cache_update_monotonic = 0.0
        self._latest_detected_self_ring_radius_m = 0.0
        self._point_cloud_mapping_runtime_enabled = not self._should_defer_point_cloud_mapping_until_runtime_enabled()
        self._global_map_cache_restored = False
        self._voxel_map_cache_restored = False

        self._collision_detector = Go2CollisionDetector(self.config.collision_detector)
        self._cost_mapper = Go2CostMapper(self.config.cost_mapper)
        self._voxel_mapper = Go2VoxelMapper(self.config.voxel_mapper)
        self._local_planner = Go2DWALocalPlanner(self.config.local_planning)
        self._frontier_explorer: Optional[FrontierExplorer] = None
        self._last_collision_status = CollisionStatus.SAFE
        self._last_local_velocity_cmd: Optional[Go2VelocityCommand] = None
        if self._point_cloud_mapping_runtime_enabled:
            with self._lock:
                self._restore_mapping_runtime_cache_unlocked(reason="bridge_init")

    def start(self) -> None:
        """启动 Go2 数据桥。"""

        with self._lock:
            if self._started:
                return
            if _uses_sdk2_in_process(self.config) and _is_ros_environment_active():
                message = _sdk2_ros_conflict_message()
                self._last_error = message
                raise RuntimeError(message)
            self._start_direct_dds_backend()

            modules = None
            if _uses_sdk2_in_process(self.config):
                self._ros_in_process_disabled_reason = (
                    "当前进程已启用 Unitree SDK2，已按官方约束禁用进程内 ROS2 订阅。"
                )
                if self.config.require_ros2:
                    self._last_error = self._ros_in_process_disabled_reason
                    raise RuntimeError(self._ros_in_process_disabled_reason)
                LOGGER.info(self._ros_in_process_disabled_reason)
            else:
                self._ros_in_process_disabled_reason = ""
                modules = self._load_ros_modules()

            if modules is not None:
                self._rclpy = modules["rclpy"]
                if not self._rclpy.ok():
                    self._rclpy.init(args=None)
                    self._owns_rclpy_context = True
                node_class = modules["Node"]
                executor_class = modules["MultiThreadedExecutor"]
                self._node = node_class("nuwax_go2_data_plane")
                self._executor = executor_class(num_threads=2)
                self._executor.add_node(self._node)

                self._create_subscriptions(modules)
                self._executor_thread = threading.Thread(
                    target=self._executor.spin,
                    name="nuwax_go2_ros2_executor",
                    daemon=True,
                )
                self._executor_thread.start()
                self._ros_started = True
            elif self.config.require_ros2 and not self._direct_dds_started:
                message = "当前环境不可导入 rclpy 或 ROS2 消息包，且 Go2 官方 DDS 直连未就绪。"
                self._last_error = message
                raise RuntimeError(message)
            elif not self._direct_dds_started:
                message = "当前环境不可导入 rclpy 或 ROS2 消息包，Go2 数据桥保持未启用。"
                self._last_error = message
                LOGGER.warning(message)
                return

            self._started = True
            self._last_error = None
            LOGGER.info(
                "Go2 数据桥已启动 direct_dds=%s ros2=%s",
                self._direct_dds_started,
                self._ros_started,
            )

    def stop(self) -> None:
        """停止 Go2 数据桥。"""

        with self._lock:
            if not self._started:
                return
            self._stop_direct_dds_backend()
            try:
                if self._executor is not None:
                    self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                LOGGER.exception("关闭 ROS2 executor 失败。")
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                LOGGER.exception("销毁 ROS2 节点失败。")
            if self._executor_thread is not None:
                self._executor_thread.join(timeout=2.0)
            if self._owns_rclpy_context and self._rclpy is not None:
                try:
                    self._rclpy.shutdown()
                except Exception:
                    LOGGER.exception("关闭 rclpy 上下文失败。")
            self._executor = None
            self._node = None
            self._executor_thread = None
            self._action_client = None
            self._goal_handle = None
            self._ros_started = False
            self._started = False
            LOGGER.info("Go2 数据桥已停止。")

    def is_running(self) -> bool:
        """返回桥是否已启动。"""

        return self._started

    def is_localization_available(self) -> bool:
        """返回定位数据是否可用。"""

        return self._latest_pose is not None or bool(self._tf_transforms)

    def is_map_available(self) -> bool:
        """返回地图数据是否可用。"""

        with self._lock:
            return self._select_effective_map_layers_unlocked().has_any_layer()

    def is_navigation_available(self) -> bool:
        """桥本身不再提供导航控制能力。"""

        return False

    def get_current_pose(self) -> Optional[Pose]:
        """读取当前位姿。"""

        with self._lock:
            return self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None

    def get_frame_tree(self) -> Optional[FrameTree]:
        """读取当前 TF 树快照。"""

        with self._lock:
            transforms = [item.model_copy(deep=True) for item in self._tf_transforms.values()]
            if self._latest_pose is not None:
                base_frame = self.config.map_synthesis.base_frame
                parent_frame = self._latest_pose.frame_id
                synthetic_key = (parent_frame, base_frame)
                if synthetic_key not in self._tf_transforms:
                    transforms.append(
                        Transform(
                            parent_frame_id=parent_frame,
                            child_frame_id=base_frame,
                            translation=self._latest_pose.position,
                            rotation=self._latest_pose.orientation,
                            authority="odom_pose",
                        )
                    )
            if not transforms:
                return None
            return FrameTree(
                root_frame_id=self.config.map_synthesis.map_frame,
                transforms=transforms,
            )

    def get_imu_state(self) -> Optional[IMUState]:
        """读取当前 IMU 状态。"""

        with self._lock:
            return self._latest_imu_state.model_copy(deep=True) if self._latest_imu_state is not None else None

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """读取占据栅格地图。"""

        with self._lock:
            grid = self._select_effective_map_layers_unlocked().occupancy_grid
            return grid.model_copy(deep=True) if grid is not None else None

    def get_cost_map(self) -> Optional[CostMap]:
        """读取代价地图。"""

        with self._lock:
            cost_map = self._select_effective_map_layers_unlocked().cost_map
            return cost_map.model_copy(deep=True) if cost_map is not None else None

    def get_collision_status(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> CollisionStatus:
        """实时碰撞检测。"""
        return self._collision_detector.check_collision(current_pose, scan_ranges)

    def process_pointcloud_for_costmap(
        self,
        points_xyz: np.ndarray,
        sensor_x: float,
        sensor_y: float,
    ) -> Optional[Go2CostMapperResult]:
        """将3D点云转换为2D代价地图。"""
        return self._cost_mapper.process_pointcloud(points_xyz, sensor_x, sensor_y)

    def add_pointcloud_to_voxel_map(
        self,
        points_xyz: np.ndarray,
        timestamp: float = 0.0,
    ) -> Optional[Go2VoxelMapperResult]:
        """添加一帧点云到体素地图。"""
        return self._voxel_mapper.add_frame(points_xyz, timestamp)

    def compute_local_velocity(
        self,
        current_pose: Pose,
        global_plan: List[Tuple[float, float]],
        cost_map: Optional[np.ndarray] = None,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> Go2VelocityCommand:
        """用DWA计算局部动态避障速度。"""
        return self._local_planner.compute_velocity(
            current_pose, global_plan, cost_map, scan_ranges
        )

    def check_exploration_safety(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray] = None,
    ):
        """检测探索期间的安全性。"""
        if self._frontier_explorer is None:
            self._frontier_explorer = FrontierExplorer(ExplorationConfig())
        return self._frontier_explorer.check_exploration_safety(current_pose, scan_ranges)

    def get_voxel_map_pointcloud(self) -> Optional[np.ndarray]:
        """获取体素地图生成的全局点云。"""
        return self._voxel_mapper.get_global_pointcloud()

    def get_voxel_map_count(self) -> int:
        """获取当前体素数量。"""
        return self._voxel_mapper.get_voxel_count()

    def get_latest_local_point_cloud(self, *, max_points: int = 512) -> Optional[PointCloudFrame]:
        """返回最近局部窗口聚合后的点云帧。"""

        with self._lock:
            scans = list(self._local_map_scans)
            current_pose = self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None
            source_topic = str(self._latest_point_cloud_source_topic or "").strip()

        if not scans:
            return None

        raw_batches: List[np.ndarray] = []
        latest_stamp_sec = 0.0
        source_labels: List[str] = []
        for scan in scans:
            points = np.asarray(scan.points_xyz_world, dtype=np.float32)
            if points.ndim != 2 or points.shape[0] <= 0 or points.shape[1] < 3:
                continue
            raw_batches.append(points[:, :3])
            latest_stamp_sec = max(latest_stamp_sec, float(scan.stamp_sec))
            if scan.source_label and scan.source_label not in source_labels:
                source_labels.append(scan.source_label)

        if not raw_batches:
            return None

        point_cloud = np.concatenate(raw_batches, axis=0)
        raw_point_count = int(point_cloud.shape[0])
        target_count = max(1, int(max_points))
        if raw_point_count > target_count:
            sample_indices = np.linspace(0, raw_point_count - 1, num=target_count, dtype=np.int64)
            point_cloud = point_cloud[sample_indices]

        frame_id = (
            str(current_pose.frame_id or "").strip()
            if current_pose is not None
            else str(self.config.map_synthesis.map_frame or "").strip()
        ) or str(self.config.map_synthesis.map_frame)
        timestamp = (
            datetime.fromtimestamp(latest_stamp_sec, tz=timezone.utc)
            if latest_stamp_sec > 0.0
            else datetime.now(timezone.utc)
        )
        points = [
            Vector3(
                x=float(row[0]),
                y=float(row[1]),
                z=float(row[2]),
            )
            for row in point_cloud
        ]
        return PointCloudFrame(
            timestamp=timestamp,
            frame_id=frame_id,
            point_count=len(points),
            fields=[
                PointField(name="x", offset=0, datatype=PointFieldDataType.FLOAT32),
                PointField(name="y", offset=4, datatype=PointFieldDataType.FLOAT32),
                PointField(name="z", offset=8, datatype=PointFieldDataType.FLOAT32),
            ],
            points=points,
            is_dense=True,
            metadata={
                "kind": "recent_local_scan_window",
                "source_topic": source_topic or None,
                "source_labels": tuple(source_labels),
                "scan_count": len(scans),
                "raw_point_count": raw_point_count,
                "sampled_point_count": len(points),
                "latest_scan_stamp_sec": round(float(latest_stamp_sec), 6) if latest_stamp_sec > 0.0 else None,
            },
        )

    def get_local_planner_state(self) -> LocalPlannerState:
        """获取局部规划器状态。"""
        return self._local_planner.state

    def reset_local_planner(self) -> None:
        """重置局部规划器。"""
        self._local_planner.reset()

    def get_semantic_map(self) -> Optional[SemanticMap]:
        """读取语义地图。"""

        with self._lock:
            semantic_map = self._select_effective_map_layers_unlocked().semantic_map
            return semantic_map.model_copy(deep=True) if semantic_map is not None else None

    def set_goal(self, goal: NavigationGoal) -> bool:
        """桥本身不再处理导航目标。"""

        del goal
        return False

    def cancel_goal(self) -> bool:
        """桥本身不再处理导航取消。"""

        return False

    def get_navigation_state(self) -> NavigationState:
        """读取导航状态。"""

        with self._lock:
            return self._latest_navigation_state.model_copy(deep=True)

    def is_goal_reached(self) -> bool:
        """判断导航目标是否已到达。"""

        with self._lock:
            return self._goal_reached

    def get_status(self) -> Dict[str, object]:
        """返回当前桥状态。"""

        with self._lock:
            effective_map_layers = self._select_effective_map_layers_unlocked()
            return {
                "started": self._started,
                "ros_started": self._ros_started,
                "ros_in_process_disabled_reason": self._ros_in_process_disabled_reason,
                "direct_dds_started": self._direct_dds_started,
                "localization_available": self.is_localization_available(),
                "map_available": effective_map_layers.has_any_layer(),
                "navigation_available": self.is_navigation_available(),
                "pose_source": self._latest_pose_source,
                "dds_pose_ready": self._direct_pose_ready,
                "ros_pose_ready": self._ros_pose_ready,
                "odom_source_topic": self._latest_odom_source_topic,
                "imu_available": self._latest_imu_state is not None,
                "dds_imu_ready": self._direct_imu_ready,
                "ros_imu_ready": self._ros_imu_ready,
                "imu_source_topic": self._latest_imu_source_topic,
                "dds_cloud_ready": self._direct_point_cloud_ready,
                "ros_point_cloud_ready": self._ros_point_cloud_ready,
                "point_cloud_source_topic": self._latest_point_cloud_source_topic,
                "point_cloud_available": self._point_cloud_available,
                "point_cloud_mapping_runtime_enabled": self._point_cloud_mapping_runtime_enabled,
                "point_cloud_mapping_deferred_until_runtime_enable": (
                    self._should_defer_point_cloud_mapping_until_runtime_enabled()
                ),
                "effective_map_source": effective_map_layers.source_name,
                "effective_map_layers": [
                    layer_name
                    for layer_name, layer_value in (
                        ("occupancy_grid", effective_map_layers.occupancy_grid),
                        ("cost_map", effective_map_layers.cost_map),
                        ("semantic_map", effective_map_layers.semantic_map),
                    )
                    if layer_value is not None
                ],
                "ros_map_ready": self._ros_occupancy_ready or self._ros_grid_map_ready or self._ros_cost_map_ready,
                "global_map_ready": self._global_map_ready,
                "global_map_source": self._latest_global_map_source,
                "global_map_cache_restored": self._global_map_cache_restored,
                "local_map_ready": self._local_map_ready,
                "local_map_source": self._latest_local_map_source,
                "voxel_map_ready": self._voxel_mapper.has_data(),
                "voxel_map_count": self._voxel_mapper.get_voxel_count(),
                "voxel_map_cache_restored": self._voxel_map_cache_restored,
                "last_error": self._last_error,
            }

    def _same_grid_layout(self, primary, candidate) -> bool:
        if primary is None or candidate is None:
            return False
        primary_frame = str(getattr(primary, "frame_id", "") or "")
        candidate_frame = str(getattr(candidate, "frame_id", "") or "")
        primary_origin = getattr(getattr(primary, "origin", None), "position", None)
        candidate_origin = getattr(getattr(candidate, "origin", None), "position", None)
        if (
            primary_frame
            and candidate_frame
            and not frame_ids_semantically_equal(primary_frame, candidate_frame)
        ):
            return False
        return (
            int(getattr(primary, "width", 0)) == int(getattr(candidate, "width", 0))
            and int(getattr(primary, "height", 0)) == int(getattr(candidate, "height", 0))
            and abs(float(getattr(primary, "resolution_m", 0.0)) - float(getattr(candidate, "resolution_m", 0.0))) <= 1e-6
            and abs(float(getattr(primary_origin, "x", 0.0)) - float(getattr(candidate_origin, "x", 0.0))) <= 1e-6
            and abs(float(getattr(primary_origin, "y", 0.0)) - float(getattr(candidate_origin, "y", 0.0))) <= 1e-6
        )

    def _normalize_selected_map_layers_unlocked(
        self,
        *,
        source_name: str,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
        semantic_map: Optional[SemanticMap],
    ) -> _SelectedMapLayers:
        normalized_cost_map = cost_map
        normalized_semantic_map = semantic_map
        if occupancy_grid is not None and normalized_cost_map is not None and not self._same_grid_layout(occupancy_grid, normalized_cost_map):
            normalized_cost_map = None
        reference_frame_id = ""
        if occupancy_grid is not None:
            reference_frame_id = str(occupancy_grid.frame_id or "")
        elif normalized_cost_map is not None:
            reference_frame_id = str(normalized_cost_map.frame_id or "")
        if (
            normalized_semantic_map is not None
            and reference_frame_id
            and normalized_semantic_map.frame_id
            and not frame_ids_semantically_equal(normalized_semantic_map.frame_id, reference_frame_id)
        ):
            normalized_semantic_map = None
        return _SelectedMapLayers(
            source_name=source_name,
            occupancy_grid=occupancy_grid,
            cost_map=normalized_cost_map,
            semantic_map=normalized_semantic_map,
        )

    def _select_effective_map_layers_unlocked(self) -> _SelectedMapLayers:
        prefer_platform_point_cloud = bool(
            self._point_cloud_mapping_runtime_enabled
            and (
                self._latest_occupancy_from_global_cloud is not None
                or self._latest_cost_from_global_cloud is not None
                or self._latest_semantic_map_from_global_cloud is not None
                or self._latest_occupancy_from_local_cloud is not None
                or self._latest_cost_from_local_cloud is not None
                or self._latest_semantic_map_from_local_cloud is not None
            )
        )
        ordered_sources = (
            (
                "platform_global_cloud",
                self._latest_occupancy_from_global_cloud,
                self._latest_cost_from_global_cloud,
                self._latest_semantic_map_from_global_cloud,
            ),
            (
                "platform_local_cloud",
                self._latest_occupancy_from_local_cloud,
                self._latest_cost_from_local_cloud,
                self._latest_semantic_map_from_local_cloud,
            ),
            (
                "ros_grid_map",
                self._latest_occupancy_from_grid,
                self._latest_cost_from_grid,
                self._latest_semantic_map,
            ),
            (
                "ros_direct_map",
                self._latest_occupancy_direct,
                self._latest_cost_direct,
                None,
            ),
        )
        if not prefer_platform_point_cloud:
            ordered_sources = (
                ordered_sources[2],
                ordered_sources[3],
                ordered_sources[0],
                ordered_sources[1],
            )

        normalized_sources = [
            self._normalize_selected_map_layers_unlocked(
                source_name=source_name,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                semantic_map=semantic_map,
            )
            for source_name, occupancy_grid, cost_map, semantic_map in ordered_sources
        ]
        for layers in normalized_sources:
            if layers.occupancy_grid is not None:
                return layers
        for layers in normalized_sources:
            if layers.cost_map is not None:
                return layers
        for layers in normalized_sources:
            if layers.semantic_map is not None:
                return layers
        return _SelectedMapLayers()

    def _load_ros_modules(self) -> Optional[Dict[str, object]]:
        self._ensure_ros_python_environment()
        try:
            modules = {
                "rclpy": importlib.import_module("rclpy"),
                "Node": getattr(importlib.import_module("rclpy.node"), "Node"),
                "MultiThreadedExecutor": getattr(importlib.import_module("rclpy.executors"), "MultiThreadedExecutor"),
                "Odometry": getattr(importlib.import_module("nav_msgs.msg"), "Odometry"),
                "RosOccupancyGrid": getattr(importlib.import_module("nav_msgs.msg"), "OccupancyGrid"),
                "TFMessage": getattr(importlib.import_module("tf2_msgs.msg"), "TFMessage"),
                "Imu": getattr(importlib.import_module("sensor_msgs.msg"), "Imu"),
                "PointCloud2": getattr(importlib.import_module("sensor_msgs.msg"), "PointCloud2"),
            }
            try:
                modules["GridMap"] = getattr(importlib.import_module("grid_map_msgs.msg"), "GridMap")
            except Exception:
                modules["GridMap"] = None
            try:
                modules["Nav2Costmap"] = getattr(importlib.import_module("nav2_msgs.msg"), "Costmap")
            except Exception:
                modules["Nav2Costmap"] = None
            return modules
        except Exception as exc:
            if self.config.require_ros2 or not self._direct_dds_started:
                LOGGER.exception("导入 ROS2 Python 模块失败。")
            else:
                LOGGER.warning("Go2 端侧 ROS2 Python 环境当前不可用，继续使用官方 DDS 直连：%s", exc)
                LOGGER.debug("导入 ROS2 Python 模块失败详情。", exc_info=True)
            return None

    def _ensure_ros_python_environment(self) -> None:
        """把 setup.bash 导出的环境同步到当前 Python 进程。"""

        if self._ros_setup_loaded:
            return
        if importlib.util.find_spec("rclpy") is not None:
            self._ros_setup_loaded = True
            return

        setup_script = str(self.config.setup_script or "").strip()
        if not setup_script:
            LOGGER.warning("Go2 ROS2 setup 脚本未配置，无法自动导入 ROS2 Python 环境。")
            return

        setup_path = Path(setup_script).expanduser()
        if not setup_path.exists():
            LOGGER.warning("Go2 ROS2 setup 脚本不存在，无法自动导入 ROS2 Python 环境：%s", setup_path)
            return

        try:
            completed = subprocess.run(
                [
                    "bash",
                    "-lc",
                    f"source {shlex.quote(str(setup_path))} >/dev/null 2>&1 && env -0",
                ],
                check=True,
                capture_output=True,
                env=dict(os.environ),
            )
        except Exception:
            LOGGER.exception("执行 Go2 ROS2 setup 脚本失败：%s", setup_path)
            return

        loaded_env = self._parse_null_delimited_environment(completed.stdout)
        if not loaded_env:
            LOGGER.warning("Go2 ROS2 setup 脚本未导出可用环境：%s", setup_path)
            return

        self._merge_ros_environment(loaded_env)
        importlib.invalidate_caches()
        self._ros_setup_loaded = True
        LOGGER.info("已自动加载 Go2 ROS2 Python 环境 setup=%s", setup_path)

    def _parse_null_delimited_environment(self, payload: bytes) -> Dict[str, str]:
        """把 env -0 输出解析成环境变量字典。"""

        result: Dict[str, str] = {}
        for raw_item in payload.split(b"\x00"):
            if not raw_item or b"=" not in raw_item:
                continue
            key_bytes, value_bytes = raw_item.split(b"=", 1)
            key = key_bytes.decode("utf-8", errors="ignore").strip()
            if not key:
                continue
            result[key] = value_bytes.decode("utf-8", errors="ignore")
        return result

    def _merge_ros_environment(self, loaded_env: Dict[str, str]) -> None:
        """把 ROS2 相关环境注入当前进程，并同步 PYTHONPATH 到 sys.path。"""

        for key, value in loaded_env.items():
            os.environ[key] = value

        python_path = str(loaded_env.get("PYTHONPATH", "") or "").strip()
        if not python_path:
            return

        inserted_paths: List[str] = []
        for raw_item in python_path.split(":"):
            item = raw_item.strip()
            if not item or item in inserted_paths:
                continue
            inserted_paths.append(item)

        for item in reversed(inserted_paths):
            if item not in sys.path:
                sys.path.insert(0, item)

    def _start_direct_dds_backend(self) -> None:
        """启动 Go2 官方 DDS 原始数据订阅。"""

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
            self._direct_point_cloud_subscriber = subscriber_class(
                self.config.direct_dds.point_cloud_topic,
                modules["DirectPointCloud2"],
            )
            self._direct_point_cloud_subscriber.Init()
        except Exception as exc:
            LOGGER.warning("初始化 Go2 官方 DDS 订阅器失败：%s", exc)
            self._direct_pose_subscriber = None
            self._direct_point_cloud_subscriber = None
            return

        self._direct_dds_stop_event.clear()
        self._direct_dds_threads = [
            threading.Thread(
                target=self._direct_pose_reader_loop,
                name="go2_direct_pose_reader",
                daemon=True,
            ),
            threading.Thread(
                target=self._direct_point_cloud_reader_loop,
                name="go2_direct_point_cloud_reader",
                daemon=True,
            ),
        ]
        for thread in self._direct_dds_threads:
            thread.start()
        self._direct_dds_started = True

    def _stop_direct_dds_backend(self) -> None:
        """停止 Go2 官方 DDS 原始数据订阅。"""

        self._direct_dds_stop_event.set()
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
            self._on_direct_point_cloud(sample, source_topic=source_topic)

    def _update_preferred_pose_unlocked(self) -> None:
        """按照优先级更新当前位姿。"""

        if self._latest_pose_from_dds is not None:
            self._latest_pose = self._latest_pose_from_dds
            self._latest_pose_source = f"dds:{self.config.direct_dds.pose_topic}"
        else:
            self._latest_pose = self._latest_pose_from_ros
            self._latest_pose_source = self._latest_odom_source_topic
        self._maybe_refresh_global_map_outputs_unlocked()

    def _update_preferred_imu_unlocked(self) -> None:
        """按照优先级更新当前 IMU 来源。"""

        if self._latest_imu_state_from_dds is not None:
            self._latest_imu_state = self._latest_imu_state_from_dds
            self._latest_imu_source_topic = f"dds:{self.config.direct_dds.pose_topic}#imu_state"
            return
        if self._latest_imu_state_from_ros is not None:
            self._latest_imu_state = self._latest_imu_state_from_ros
            self._latest_imu_source_topic = str(self.config.topics.imu_topic or "")
            return
        self._latest_imu_state = None
        self._latest_imu_source_topic = ""

    def _update_preferred_point_cloud_status_unlocked(self) -> None:
        """按照优先级更新当前点云来源状态。"""

        if self._direct_point_cloud_ready:
            self._latest_point_cloud_source_topic = f"dds:{self.config.direct_dds.point_cloud_topic}"
            self._point_cloud_available = True
            return
        if self._ros_point_cloud_ready:
            self._latest_point_cloud_source_topic = str(self.config.topics.point_cloud_topic or "")
            self._point_cloud_available = True
            return
        self._latest_point_cloud_source_topic = ""
        self._point_cloud_available = False

    def _create_subscriptions(self, modules: Dict[str, object]) -> None:
        assert self._node is not None
        topics = self.config.topics
        subscribed_topics = set()
        odom_topics = [topics.odom_topic, *list(getattr(topics, "secondary_odom_topics", ()) or ())]
        for topic_name in odom_topics:
            normalized = str(topic_name or "").strip()
            if not normalized or normalized in subscribed_topics:
                continue
            subscribed_topics.add(normalized)
            self._node.create_subscription(
                modules["Odometry"],
                normalized,
                lambda message, source_topic=normalized: self._on_odom(message, source_topic=source_topic),
                20,
            )
        self._node.create_subscription(modules["TFMessage"], topics.tf_topic, self._on_tf_message, 50)
        self._node.create_subscription(modules["TFMessage"], topics.tf_static_topic, self._on_tf_message, 10)
        if str(getattr(topics, "imu_topic", "") or "").strip():
            self._node.create_subscription(
                modules["Imu"],
                topics.imu_topic,
                lambda message, source_topic=topics.imu_topic: self._on_imu(message, source_topic=source_topic),
                20,
            )
        if str(getattr(topics, "point_cloud_topic", "") or "").strip():
            self._node.create_subscription(
                modules["PointCloud2"],
                topics.point_cloud_topic,
                lambda message, source_topic=topics.point_cloud_topic: self._on_point_cloud(
                    message, source_topic=source_topic
                ),
                5,
            )
        self._node.create_subscription(modules["RosOccupancyGrid"], topics.occupancy_topic, self._on_occupancy_grid, 10)
        if modules.get("GridMap") is not None and topics.grid_map_topic.strip():
            self._node.create_subscription(modules["GridMap"], topics.grid_map_topic, self._on_grid_map, 10)
        if modules.get("Nav2Costmap") is not None and topics.cost_map_topic.strip():
            self._node.create_subscription(modules["Nav2Costmap"], topics.cost_map_topic, self._on_cost_map, 10)

    def _on_odom(self, message: object, *, source_topic: str = "") -> None:
        pose = self._build_pose_from_odom(message)
        with self._lock:
            self._latest_pose_from_ros = pose
            self._ros_pose_ready = True
            self._latest_odom_source_topic = str(source_topic or "")
            self._update_preferred_pose_unlocked()

    def _on_imu(self, message: object, *, source_topic: str = "") -> None:
        imu_state = self._build_imu_from_ros(message)
        with self._lock:
            self._latest_imu_state_from_ros = imu_state
            self._ros_imu_ready = imu_state is not None
            if not str(self.config.topics.imu_topic or "").strip():
                self._latest_imu_source_topic = str(source_topic or "")
            self._update_preferred_imu_unlocked()

    def _is_point_cloud_mapping_enabled(self) -> bool:
        """判断当前是否启用了任意点云地图合成链。"""

        return bool(
            self._point_cloud_mapping_runtime_enabled
            and (
                self.config.map_synthesis.local_map_enabled
                or self.config.map_synthesis.global_map_enabled
            )
        )

    def _should_defer_point_cloud_mapping_until_runtime_enabled(self) -> bool:
        """判断是否应在显式触发前延后点云地图合成。"""

        if not (
            self.config.map_synthesis.local_map_enabled
            or self.config.map_synthesis.global_map_enabled
        ):
            return False
        official_cfg = getattr(self.config, "official", None)
        return bool(getattr(official_cfg, "enabled", False) and not getattr(official_cfg, "auto_start_services", True))

    def _clear_point_cloud_mapping_outputs_unlocked(self) -> None:
        """清理 bridge 内部点云建图产物，恢复到冷启动状态。"""

        self._global_map_builder = Go2SparseGlobalMapBuilder(self.config.map_synthesis)
        self._voxel_mapper.reset()
        self._local_map_scans.clear()
        self._last_global_map_update_monotonic = 0.0
        self._last_local_map_update_monotonic = 0.0
        self._last_voxel_cache_update_monotonic = 0.0
        self._latest_occupancy_from_global_cloud = None
        self._latest_cost_from_global_cloud = None
        self._latest_semantic_map_from_global_cloud = None
        self._latest_occupancy_from_local_cloud = None
        self._latest_cost_from_local_cloud = None
        self._latest_semantic_map_from_local_cloud = None
        self._global_map_ready = False
        self._local_map_ready = False
        self._latest_global_map_source = ""
        self._latest_local_map_source = ""
        self._global_map_cache_restored = False
        self._voxel_map_cache_restored = False

    def _mapping_runtime_cache_root(self) -> Path:
        root = str(getattr(self.config, "named_map_archive_root", "") or "").strip()
        if not root:
            root = str(Path.cwd() / "runtime_data" / "go2_named_maps")
        return Path(root).expanduser() / "_runtime_cache"

    def _global_map_cache_path(self) -> Path:
        return self._mapping_runtime_cache_root() / "global_map_state.json"

    def _voxel_map_cache_path(self) -> Path:
        return self._mapping_runtime_cache_root() / "voxel_map_points.npz"

    def _persist_global_map_cache_unlocked(self) -> None:
        """把当前实时全局图状态持久化到本地缓存。"""

        if not bool(self.config.map_synthesis.global_map_enabled):
            return

        has_builder_state = self._global_map_builder.has_data()
        has_snapshot = any(
            item is not None
            for item in (
                self._latest_occupancy_from_global_cloud,
                self._latest_cost_from_global_cloud,
                self._latest_semantic_map_from_global_cloud,
            )
        )
        state_path = self._global_map_cache_path()
        if not has_builder_state and not has_snapshot:
            if state_path.exists():
                state_path.unlink()
            return

        state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "saved_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "source_label": self._latest_global_map_source,
            "builder_state": self._global_map_builder.export_state(),
            "occupancy_grid": (
                self._latest_occupancy_from_global_cloud.model_dump(mode="json")
                if self._latest_occupancy_from_global_cloud is not None
                else None
            ),
            "cost_map": (
                self._latest_cost_from_global_cloud.model_dump(mode="json")
                if self._latest_cost_from_global_cloud is not None
                else None
            ),
            "semantic_map": (
                self._latest_semantic_map_from_global_cloud.model_dump(mode="json")
                if self._latest_semantic_map_from_global_cloud is not None
                else None
            ),
        }
        state_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _restore_global_map_cache_unlocked(self) -> bool:
        """从本地缓存恢复实时全局图状态。"""

        if not bool(self.config.map_synthesis.global_map_enabled):
            self._global_map_cache_restored = False
            return False
        state_path = self._global_map_cache_path()
        if not state_path.exists():
            self._global_map_cache_restored = False
            return False
        try:
            payload = json.loads(state_path.read_text(encoding="utf-8"))
        except Exception as exc:
            LOGGER.warning("读取 Go2 全局图缓存失败 path=%s error=%s", state_path, exc)
            self._global_map_cache_restored = False
            return False

        restored_builder = self._global_map_builder.restore_state(payload.get("builder_state"))
        try:
            self._latest_occupancy_from_global_cloud = (
                OccupancyGrid.model_validate(payload["occupancy_grid"])
                if payload.get("occupancy_grid") is not None
                else None
            )
            self._latest_cost_from_global_cloud = (
                CostMap.model_validate(payload["cost_map"])
                if payload.get("cost_map") is not None
                else None
            )
            self._latest_semantic_map_from_global_cloud = (
                SemanticMap.model_validate(payload["semantic_map"])
                if payload.get("semantic_map") is not None
                else None
            )
        except Exception as exc:
            LOGGER.warning("解析 Go2 全局图缓存失败 path=%s error=%s", state_path, exc)
            self._latest_occupancy_from_global_cloud = None
            self._latest_cost_from_global_cloud = None
            self._latest_semantic_map_from_global_cloud = None
            self._global_map_ready = False
            self._global_map_cache_restored = restored_builder
            return restored_builder

        self._global_map_ready = any(
            item is not None
            for item in (
                self._latest_occupancy_from_global_cloud,
                self._latest_cost_from_global_cloud,
                self._latest_semantic_map_from_global_cloud,
            )
        )
        self._latest_global_map_source = str(payload.get("source_label") or self._latest_global_map_source or "")
        self._global_map_cache_restored = restored_builder or self._global_map_ready
        return self._global_map_cache_restored

    def _persist_voxel_map_cache_unlocked(self) -> None:
        """把当前体素地图点云持久化到本地缓存。"""

        if not (bool(self.config.voxel_mapper.enabled) and bool(self.config.map_synthesis.global_map_enabled)):
            return
        state_path = self._voxel_map_cache_path()
        points_xyz = self._voxel_mapper.get_global_pointcloud()
        if points_xyz is None or points_xyz.size == 0:
            if state_path.exists():
                state_path.unlink()
            return
        state_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(state_path, points_xyz=np.asarray(points_xyz, dtype=np.float32))

    def _restore_voxel_map_cache_unlocked(self) -> bool:
        """从本地缓存恢复体素地图。"""

        if not (bool(self.config.voxel_mapper.enabled) and bool(self.config.map_synthesis.global_map_enabled)):
            self._voxel_map_cache_restored = False
            return False
        state_path = self._voxel_map_cache_path()
        if not state_path.exists():
            self._voxel_map_cache_restored = False
            return False
        try:
            with np.load(state_path) as payload:
                points_xyz = np.asarray(payload["points_xyz"], dtype=np.float32)
        except Exception as exc:
            LOGGER.warning("读取 Go2 体素图缓存失败 path=%s error=%s", state_path, exc)
            self._voxel_map_cache_restored = False
            return False
        if points_xyz.ndim != 2 or points_xyz.shape[1] < 3 or points_xyz.size == 0:
            self._voxel_map_cache_restored = False
            return False
        self._voxel_mapper.restore_pointcloud(points_xyz[:, :3], timestamp=time.time())
        self._voxel_map_cache_restored = self._voxel_mapper.has_data()
        return self._voxel_map_cache_restored

    def _restore_mapping_runtime_cache_unlocked(self, *, reason: str = "") -> bool:
        """恢复点云建图运行态缓存。"""

        restored_global = self._restore_global_map_cache_unlocked()
        restored_voxel = self._restore_voxel_map_cache_unlocked()
        restored_any = restored_global or restored_voxel
        if restored_any:
            LOGGER.info(
                "Go2 点云建图运行态已从缓存恢复 reason=%s global=%s voxel=%s",
                reason or "unknown",
                restored_global,
                restored_voxel,
            )
        return restored_any

    def _maybe_refresh_global_map_outputs_unlocked(self) -> None:
        """当仅恢复了稀疏状态但尚未恢复快照时，按当前位姿补生成一次地图快照。"""

        if not (
            self._point_cloud_mapping_runtime_enabled
            and bool(self.config.map_synthesis.global_map_enabled)
            and self._latest_pose is not None
            and self._global_map_builder.has_data()
        ):
            return
        if self._global_map_ready:
            return
        build_result = self._global_map_builder.build()
        occupancy, cost_map, semantic_map = self._build_contract_maps_from_global_result(
            build_result,
            current_pose=self._latest_pose,
        )
        self._latest_occupancy_from_global_cloud = occupancy
        self._latest_cost_from_global_cloud = cost_map
        self._latest_semantic_map_from_global_cloud = semantic_map
        self._global_map_ready = bool(occupancy is not None or cost_map is not None or semantic_map is not None)
        if build_result is not None:
            self._latest_global_map_source = build_result.source_label
        self._persist_global_map_cache_unlocked()

    def enable_mapping_runtime(self, *, reason: str = "") -> Dict[str, object]:
        """显式启用点云地图合成链。"""

        restored_runtime_cache = False
        with self._lock:
            already_enabled = self._point_cloud_mapping_runtime_enabled
            self._point_cloud_mapping_runtime_enabled = True
            if not already_enabled:
                restored_runtime_cache = self._restore_mapping_runtime_cache_unlocked(
                    reason=reason or "enable_mapping_runtime"
                )
                self._maybe_refresh_global_map_outputs_unlocked()
            status = {
                "point_cloud_mapping_runtime_enabled": self._point_cloud_mapping_runtime_enabled,
                "point_cloud_mapping_deferred_until_runtime_enable": (
                    self._should_defer_point_cloud_mapping_until_runtime_enabled()
                ),
                "point_cloud_available": self._point_cloud_available,
                "global_map_ready": self._global_map_ready,
                "local_map_ready": self._local_map_ready,
                "voxel_map_ready": self._voxel_mapper.has_data(),
                "restored_runtime_cache": restored_runtime_cache,
            }
        if not already_enabled:
            LOGGER.info("Go2 点云地图合成链已启用 reason=%s", reason or "mapping_runtime")
        return status

    def disable_mapping_runtime(self, *, reason: str = "") -> Dict[str, object]:
        """在按需模式下回收点云地图合成链并清理已有建图产物。"""

        with self._lock:
            supports_runtime_toggle = self._should_defer_point_cloud_mapping_until_runtime_enabled()
            was_enabled = self._point_cloud_mapping_runtime_enabled
            if supports_runtime_toggle:
                self._point_cloud_mapping_runtime_enabled = False
                self._clear_point_cloud_mapping_outputs_unlocked()
            status = {
                "point_cloud_mapping_runtime_enabled": self._point_cloud_mapping_runtime_enabled,
                "point_cloud_mapping_deferred_until_runtime_enable": supports_runtime_toggle,
                "point_cloud_available": self._point_cloud_available,
                "global_map_ready": self._global_map_ready,
                "local_map_ready": self._local_map_ready,
                "voxel_map_ready": self._voxel_mapper.has_data(),
                "map_available": self.is_map_available(),
            }
        if supports_runtime_toggle and was_enabled:
            LOGGER.info("Go2 点云地图合成链已回收 reason=%s", reason or "mapping_runtime")
        return status

    def _on_point_cloud(self, message: object, *, source_topic: str = "") -> None:
        with self._lock:
            current_pose = self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None
            self._ros_point_cloud_ready = True
            self._update_preferred_point_cloud_status_unlocked()
            should_ingest = not self._direct_dds_started or not self._direct_point_cloud_ready
        if current_pose is None or not should_ingest or not self._is_point_cloud_mapping_enabled():
            return
        self._ingest_point_cloud_sample(
            message,
            current_pose=current_pose,
            source_label="ros_point_cloud",
        )

    def _on_sport_mode_state(self, message: object, *, source_topic: str = "") -> None:
        pose = self._build_pose_from_sport_mode_state(message)
        imu_state = self._build_imu_from_sport_mode_state(message)
        with self._lock:
            self._latest_pose_from_dds = pose
            self._direct_pose_ready = True
            self._latest_odom_source_topic = str(source_topic or "")
            self._latest_imu_state_from_dds = imu_state
            self._direct_imu_ready = imu_state is not None
            self._update_preferred_pose_unlocked()
            self._update_preferred_imu_unlocked()

    def _on_direct_point_cloud(self, message: object, *, source_topic: str = "") -> None:
        with self._lock:
            current_pose = self._latest_pose.model_copy(deep=True) if self._latest_pose is not None else None
            self._direct_point_cloud_ready = True
            self._update_preferred_point_cloud_status_unlocked()
        if current_pose is None or not self._is_point_cloud_mapping_enabled():
            return
        self._ingest_point_cloud_sample(
            message,
            current_pose=current_pose,
            source_label="direct_dds_point_cloud",
        )

    def _on_tf_message(self, message: object) -> None:
        transforms = getattr(message, "transforms", [])
        with self._lock:
            for transform in transforms:
                parent_frame = str(getattr(getattr(transform, "header", None), "frame_id", "") or "").strip()
                child_frame = str(getattr(transform, "child_frame_id", "") or "").strip()
                if not parent_frame or not child_frame:
                    continue
                self._tf_transforms[(parent_frame, child_frame)] = self._build_transform_from_ros(transform)

    def _on_occupancy_grid(self, message: object) -> None:
        with self._lock:
            self._latest_occupancy_direct = self._build_contract_occupancy_grid(message)
            self._ros_occupancy_ready = True

    def _on_cost_map(self, message: object) -> None:
        with self._lock:
            self._latest_cost_direct = self._build_contract_cost_map(message)
            self._ros_cost_map_ready = True

    def _on_grid_map(self, message: object) -> None:
        occupancy = self._build_occupancy_from_grid_map(message)
        cost_map = self._build_cost_map_from_grid_map(message)
        semantic_map = self._build_semantic_map_from_grid_map(message)
        with self._lock:
            self._latest_occupancy_from_grid = occupancy
            self._latest_cost_from_grid = cost_map
            self._latest_semantic_map = semantic_map
            self._ros_grid_map_ready = bool(occupancy is not None or cost_map is not None or semantic_map is not None)

    def _build_pose_from_odom(self, message: object) -> Pose:
        header = getattr(message, "header", None)
        pose_with_cov = getattr(message, "pose", None)
        pose_obj = getattr(pose_with_cov, "pose", None)
        frame_id = str(getattr(header, "frame_id", "") or self.config.map_synthesis.map_frame)
        return Pose(
            frame_id=frame_id,
            position=Vector3(
                x=float(getattr(getattr(pose_obj, "position", None), "x", 0.0)),
                y=float(getattr(getattr(pose_obj, "position", None), "y", 0.0)),
                z=float(getattr(getattr(pose_obj, "position", None), "z", 0.0)),
            ),
            orientation=Quaternion(
                x=float(getattr(getattr(pose_obj, "orientation", None), "x", 0.0)),
                y=float(getattr(getattr(pose_obj, "orientation", None), "y", 0.0)),
                z=float(getattr(getattr(pose_obj, "orientation", None), "z", 0.0)),
                w=float(getattr(getattr(pose_obj, "orientation", None), "w", 1.0)),
            ),
        )

    def _build_imu_from_ros(self, message: object) -> IMUState:
        header = getattr(message, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or "imu")
        orientation = getattr(message, "orientation", None)
        angular_velocity = getattr(message, "angular_velocity", None)
        linear_acceleration = getattr(message, "linear_acceleration", None)
        return IMUState(
            frame_id=frame_id,
            orientation=Quaternion(
                x=float(getattr(orientation, "x", 0.0)),
                y=float(getattr(orientation, "y", 0.0)),
                z=float(getattr(orientation, "z", 0.0)),
                w=float(getattr(orientation, "w", 1.0)),
            ),
            angular_velocity_rad_s=Vector3(
                x=float(getattr(angular_velocity, "x", 0.0)),
                y=float(getattr(angular_velocity, "y", 0.0)),
                z=float(getattr(angular_velocity, "z", 0.0)),
            ),
            linear_acceleration_m_s2=Vector3(
                x=float(getattr(linear_acceleration, "x", 0.0)),
                y=float(getattr(linear_acceleration, "y", 0.0)),
                z=float(getattr(linear_acceleration, "z", 0.0)),
            ),
        )

    def _build_pose_from_sport_mode_state(self, message: object) -> Pose:
        imu_state = getattr(message, "imu_state", None)
        quaternion = _unitree_wxyz_quaternion_to_contract(
            getattr(imu_state, "quaternion", [1.0, 0.0, 0.0, 0.0]) or [1.0, 0.0, 0.0, 0.0]
        )

        position = list(getattr(message, "position", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0])
        while len(position) < 3:
            position.append(0.0)

        return Pose(
            frame_id=self.config.map_synthesis.map_frame,
            position=Vector3(
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            ),
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
            frame_id=str(self.config.map_synthesis.base_frame or "body"),
            orientation=quaternion,
            angular_velocity_rad_s=Vector3(
                x=float(gyroscope[0]),
                y=float(gyroscope[1]),
                z=float(gyroscope[2]),
            ),
            linear_acceleration_m_s2=Vector3(
                x=float(accelerometer[0]),
                y=float(accelerometer[1]),
                z=float(accelerometer[2]),
            ),
        )

    def _ingest_point_cloud_sample(
        self,
        message: object,
        *,
        current_pose: Pose,
        source_label: str,
    ) -> None:
        points_xyz = self._extract_xyz_from_point_cloud(message)
        if points_xyz.size == 0:
            return

        world_points, sensor_x, sensor_y = self._transform_point_cloud_to_map(points_xyz, current_pose=current_pose)
        if world_points.size == 0:
            return

        now_sec = time.time()
        now_mono = time.monotonic()
        with self._lock:
            voxel_updated = False
            if bool(self.config.voxel_mapper.enabled):
                try:
                    voxel_result = self._voxel_mapper.add_frame(world_points, timestamp=now_sec)
                    voxel_updated = voxel_result is not None and self._voxel_mapper.has_data()
                except Exception as exc:
                    LOGGER.warning("Go2 体素地图更新失败 source=%s error=%s", source_label, exc)

            if self.config.map_synthesis.global_map_enabled:
                ingested = self._global_map_builder.ingest_scan(
                    world_points=world_points,
                    sensor_x=sensor_x,
                    sensor_y=sensor_y,
                    source_label=source_label,
                )
                if ingested and (
                    now_mono - self._last_global_map_update_monotonic
                    >= self.config.map_synthesis.global_map_update_interval_sec
                ):
                    build_result = self._global_map_builder.build()
                    occupancy, cost_map, semantic_map = self._build_contract_maps_from_global_result(
                        build_result,
                        current_pose=current_pose,
                    )
                    self._latest_occupancy_from_global_cloud = occupancy
                    self._latest_cost_from_global_cloud = cost_map
                    self._latest_semantic_map_from_global_cloud = semantic_map
                    self._global_map_ready = bool(occupancy is not None or cost_map is not None or semantic_map is not None)
                    if build_result is not None:
                        self._latest_global_map_source = build_result.source_label
                    self._persist_global_map_cache_unlocked()
                    self._last_global_map_update_monotonic = now_mono

            if self.config.map_synthesis.local_map_enabled:
                self._local_map_scans.append(
                    LocalMapScan(
                        stamp_sec=now_sec,
                        sensor_x=sensor_x,
                        sensor_y=sensor_y,
                        source_label=source_label,
                        points_xyz_world=world_points,
                    )
                )
                self._prune_local_map_scans_unlocked(now_sec)
                if now_mono - self._last_local_map_update_monotonic < self.config.map_synthesis.local_map_update_interval_sec:
                    return
                occupancy, cost_map, semantic_map = self._build_local_maps_from_scans_unlocked(current_pose)
                self._latest_occupancy_from_local_cloud = occupancy
                self._latest_cost_from_local_cloud = cost_map
                self._latest_semantic_map_from_local_cloud = semantic_map
                self._local_map_ready = bool(occupancy is not None or cost_map is not None or semantic_map is not None)
                self._last_local_map_update_monotonic = now_mono

            voxel_cache_interval_sec = max(0.05, float(getattr(self.config.voxel_mapper, "publish_interval_sec", 0.5)))
            if voxel_updated and (
                now_mono - self._last_voxel_cache_update_monotonic >= voxel_cache_interval_sec
            ):
                self._persist_voxel_map_cache_unlocked()
                self._last_voxel_cache_update_monotonic = now_mono

    def _extract_xyz_from_point_cloud(self, message: object) -> np.ndarray:
        fields = list(getattr(message, "fields", []) or [])
        endian_prefix = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
        field_specs: Dict[str, Tuple[int, np.dtype]] = {}
        for field in fields:
            name = str(getattr(field, "name", "") or "").strip()
            if name in {"x", "y", "z"}:
                datatype = int(getattr(field, "datatype", 0) or 0)
                count = int(getattr(field, "count", 1) or 1)
                base_dtype = _POINT_FIELD_DTYPE_MAP.get(datatype)
                if base_dtype is None or count != 1:
                    continue
                field_specs[name] = (
                    int(getattr(field, "offset", 0)),
                    np.dtype(base_dtype).newbyteorder(endian_prefix),
                )
        if {"x", "y", "z"} - set(field_specs):
            return np.empty((0, 3), dtype=np.float32)

        point_step = int(getattr(message, "point_step", 0) or 0)
        payload = bytes(getattr(message, "data", b"") or b"")
        if point_step <= 0 or not payload or len(payload) < point_step:
            return np.empty((0, 3), dtype=np.float32)

        point_dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [field_specs["x"][1], field_specs["y"][1], field_specs["z"][1]],
                "offsets": [field_specs["x"][0], field_specs["y"][0], field_specs["z"][0]],
                "itemsize": point_step,
            }
        )
        width = int(getattr(message, "width", 0) or 0)
        height = max(1, int(getattr(message, "height", 1) or 1))
        row_step = int(getattr(message, "row_step", 0) or 0)
        expected_row_bytes = point_step * width if width > 0 else 0
        if width > 0 and expected_row_bytes > 0 and row_step >= expected_row_bytes:
            row_chunks: List[np.ndarray] = []
            for row_index in range(height):
                row_start = row_index * row_step
                row_end = row_start + expected_row_bytes
                if row_end > len(payload):
                    break
                row_chunks.append(np.frombuffer(payload[row_start:row_end], dtype=point_dtype, count=width))
            points = np.concatenate(row_chunks) if row_chunks else np.empty((0,), dtype=point_dtype)
        else:
            count = len(payload) // point_step
            points = np.frombuffer(payload, dtype=point_dtype, count=count)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        xyz = np.stack((points["x"], points["y"], points["z"]), axis=1).astype(np.float32, copy=False)
        finite_mask = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite_mask]
        if xyz.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        max_points = int(getattr(self.config.direct_dds, "max_points_per_scan", 1200))
        if xyz.shape[0] > max_points:
            stride = max(1, int(math.ceil(float(xyz.shape[0]) / float(max_points))))
            xyz = xyz[::stride]
        return xyz

    def _transform_point_cloud_to_map(self, points_xyz: np.ndarray, *, current_pose: Pose) -> Tuple[np.ndarray, float, float]:
        if points_xyz.size == 0:
            return np.empty((0, 3), dtype=np.float32), float(current_pose.position.x), float(current_pose.position.y)

        map_cfg = self.config.map_synthesis

        range_mask = np.hypot(points_xyz[:, 0], points_xyz[:, 1]) <= float(map_cfg.local_map_max_range_m)
        filtered = points_xyz[range_mask]
        point_cloud_self_filter_radius_m = max(0.0, float(getattr(map_cfg, "point_cloud_self_filter_radius_m", 0.0)))
        if point_cloud_self_filter_radius_m > 1e-6 and filtered.size != 0:
            self_mask = np.hypot(filtered[:, 0], filtered[:, 1]) >= point_cloud_self_filter_radius_m
            filtered = filtered[self_mask]
        if filtered.size == 0:
            return np.empty((0, 3), dtype=np.float32), float(current_pose.position.x), float(current_pose.position.y)

        body_rotation = _quaternion_rotation_matrix(current_pose.orientation)
        lidar_yaw_offset_rad = float(map_cfg.lidar_yaw_offset_rad)
        if abs(lidar_yaw_offset_rad) > 1e-9:
            yaw_cos = math.cos(lidar_yaw_offset_rad)
            yaw_sin = math.sin(lidar_yaw_offset_rad)
            lidar_rotation = np.asarray(
                [
                    [yaw_cos, -yaw_sin, 0.0],
                    [yaw_sin, yaw_cos, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            sensor_rotation = body_rotation @ lidar_rotation
        else:
            sensor_rotation = body_rotation

        sensor_offset = np.asarray(
            [
                float(map_cfg.lidar_offset_x_m),
                float(map_cfg.lidar_offset_y_m),
                float(map_cfg.lidar_offset_z_m),
            ],
            dtype=np.float64,
        )
        base_position = np.asarray(
            [
                float(current_pose.position.x),
                float(current_pose.position.y),
                float(current_pose.position.z),
            ],
            dtype=np.float64,
        )
        sensor_position = base_position + (body_rotation @ sensor_offset)
        transformed_points = filtered.astype(np.float64, copy=False) @ sensor_rotation.T
        transformed_points += sensor_position

        relative_z = transformed_points[:, 2] - float(current_pose.position.z)
        z_mask = (relative_z >= float(map_cfg.local_map_min_obstacle_height_m)) & (
            relative_z <= float(map_cfg.local_map_max_obstacle_height_m)
        )
        filtered_sensor_points = filtered[z_mask]
        self._latest_detected_self_ring_radius_m = self._detect_self_obstacle_ring_radius(filtered_sensor_points)
        world_points = transformed_points[z_mask].astype(np.float32, copy=False)
        return world_points, float(sensor_position[0]), float(sensor_position[1])

    def _detect_self_obstacle_ring_radius(self, points_xyz: np.ndarray) -> float:
        """检测传感器周围是否存在近乎环状的自体伪障碍回波。"""

        if points_xyz.size == 0:
            return 0.0
        min_radius_m = max(0.0, float(getattr(self.config.map_synthesis, "point_cloud_self_filter_radius_m", 0.0)))
        if min_radius_m <= 1e-6:
            return 0.0
        max_radius_m = min(
            max(min_radius_m + 0.1, float(getattr(self.config.map_synthesis, "local_map_max_range_m", 0.0))),
            _GO2_SELF_RING_MAX_RADIUS_M,
        )
        if max_radius_m <= min_radius_m + 1e-6:
            return 0.0

        distances = np.hypot(points_xyz[:, 0], points_xyz[:, 1])
        ring_mask = (distances >= min_radius_m) & (distances <= max_radius_m)
        if int(np.count_nonzero(ring_mask)) < _GO2_SELF_RING_ANGLE_BIN_COUNT:
            return 0.0

        ring_points = points_xyz[ring_mask]
        ring_distances = distances[ring_mask]
        angles_deg = (np.degrees(np.arctan2(ring_points[:, 1], ring_points[:, 0])) + 360.0) % 360.0
        bin_width_deg = 360.0 / float(_GO2_SELF_RING_ANGLE_BIN_COUNT)
        nearest_by_bin = np.full((_GO2_SELF_RING_ANGLE_BIN_COUNT,), np.inf, dtype=np.float32)
        for angle_deg, distance_m in zip(angles_deg, ring_distances):
            bin_index = min(
                _GO2_SELF_RING_ANGLE_BIN_COUNT - 1,
                int(math.floor(float(angle_deg) / bin_width_deg)),
            )
            if float(distance_m) < float(nearest_by_bin[bin_index]):
                nearest_by_bin[bin_index] = float(distance_m)

        covered_mask = np.isfinite(nearest_by_bin)
        coverage_ratio = float(np.count_nonzero(covered_mask)) / float(_GO2_SELF_RING_ANGLE_BIN_COUNT)
        if coverage_ratio < _GO2_SELF_RING_MIN_COVERAGE_RATIO:
            return 0.0

        covered_distances = nearest_by_bin[covered_mask].astype(np.float32, copy=False)
        radial_spread_m = float(np.percentile(covered_distances, 90.0) - np.percentile(covered_distances, 10.0))
        if radial_spread_m > _GO2_SELF_RING_MAX_RADIAL_SPREAD_M:
            return 0.0
        return float(
            min(
                max_radius_m,
                float(np.percentile(covered_distances, 95.0)),
            )
        )

    def _prune_local_map_scans_unlocked(self, now_sec: float) -> None:
        max_age_sec = float(self.config.map_synthesis.local_map_max_scan_age_sec)
        max_scans = int(self.config.map_synthesis.local_map_max_scans)
        while self._local_map_scans and (now_sec - self._local_map_scans[0].stamp_sec) > max_age_sec:
            self._local_map_scans.popleft()
        while len(self._local_map_scans) > max_scans:
            self._local_map_scans.popleft()

    def _build_local_maps_from_scans_unlocked(
        self,
        current_pose: Pose,
    ) -> Tuple[Optional[OccupancyGrid], Optional[CostMap], Optional[SemanticMap]]:
        build_result = build_sliding_window_map(
            scans=list(self._local_map_scans),
            current_pose=current_pose,
            config=self.config.map_synthesis,
        )
        if build_result is None:
            return None, None, None

        build_result = self._clear_navigation_origin_footprint(
            build_result=build_result,
            current_pose=current_pose,
        )
        frame_id = current_pose.frame_id or self.config.map_synthesis.map_frame
        self._latest_local_map_source = build_result.source_label

        origin_pose = Pose(
            frame_id=frame_id,
            position=Vector3(
                x=float(build_result.origin_x),
                y=float(build_result.origin_y),
                z=float(current_pose.position.z),
            ),
            orientation=Quaternion(w=1.0),
        )
        occupancy_grid = OccupancyGrid(
            map_id="go2_local_cloud_occupancy",
            frame_id=frame_id,
            width=int(build_result.width),
            height=int(build_result.height),
            resolution_m=float(build_result.resolution_m),
            origin=origin_pose,
            data=[int(value) for value in build_result.occupancy_data.flatten().tolist()],
        )
        cost_map = CostMap(
            map_id="go2_local_cloud_cost",
            frame_id=frame_id,
            width=int(build_result.width),
            height=int(build_result.height),
            resolution_m=float(build_result.resolution_m),
            origin=origin_pose,
            data=[float(value) for value in build_result.cost_data.flatten().tolist()],
        )
        semantic_map = self._build_semantic_map_from_local_cloud(
            occupancy=build_result.occupancy_data,
            occupied_mask=build_result.occupied_mask,
            frame_id=frame_id,
            origin=origin_pose,
            resolution=float(build_result.resolution_m),
            width=int(build_result.width),
            height=int(build_result.height),
            region_source=build_result.source_label,
        )
        return occupancy_grid, cost_map, semantic_map

    def _clear_navigation_origin_footprint(
        self,
        *,
        build_result: Go2GlobalMapBuildResult,
        current_pose: Pose,
    ) -> Go2GlobalMapBuildResult:
        """在导航图中为当前位置保留可通行足迹，避免机器人被自身周边膨胀代价封死。"""

        resolution_m = max(1e-6, float(build_result.resolution_m))
        map_inflation_radius_m = max(
            float(getattr(self.config.map_synthesis, "global_map_inflation_radius_m", 0.0)),
            float(getattr(self.config.map_synthesis, "local_map_inflation_radius_m", 0.0)),
        )
        planner_inflation_radius_m = float(getattr(self.config.navigation, "planner_inflation_radius_m", 0.0))
        point_cloud_self_filter_radius_m = float(
            getattr(self.config.map_synthesis, "point_cloud_self_filter_radius_m", 0.0)
        )
        detected_self_ring_radius_m = max(0.0, float(getattr(self, "_latest_detected_self_ring_radius_m", 0.0)))
        radius_m = max(
            float(getattr(self.config.official, "goal_tolerance_m", 0.0)),
            float(getattr(self.config.local_planning, "goal_tolerance_m", 0.0)),
            planner_inflation_radius_m + map_inflation_radius_m + resolution_m * 0.5,
            point_cloud_self_filter_radius_m
            + planner_inflation_radius_m
            + map_inflation_radius_m
            + resolution_m * 0.5,
            detected_self_ring_radius_m
            + planner_inflation_radius_m
            + map_inflation_radius_m
            + resolution_m * 0.5,
        )
        if radius_m <= 1e-6:
            return build_result

        center_col = int((float(current_pose.position.x) - float(build_result.origin_x)) / resolution_m)
        center_row = int((float(current_pose.position.y) - float(build_result.origin_y)) / resolution_m)
        if (
            center_col < 0
            or center_row < 0
            or center_col >= int(build_result.width)
            or center_row >= int(build_result.height)
        ):
            return build_result

        occupancy_data = np.asarray(build_result.occupancy_data, dtype=np.int32).copy()
        cost_data = np.asarray(build_result.cost_data, dtype=np.float32).copy()
        occupied_mask = np.asarray(build_result.occupied_mask, dtype=bool).copy()

        radius_cells = max(1, int(math.ceil(radius_m / resolution_m)))
        updated = False
        for row in range(max(0, center_row - radius_cells), min(int(build_result.height), center_row + radius_cells + 1)):
            for col in range(max(0, center_col - radius_cells), min(int(build_result.width), center_col + radius_cells + 1)):
                if math.hypot((float(row - center_row)) * resolution_m, (float(col - center_col)) * resolution_m) > radius_m + 1e-6:
                    continue
                occupancy_data[row, col] = 0
                cost_data[row, col] = 0.0
                occupied_mask[row, col] = False
                updated = True

        if not updated:
            return build_result

        return Go2GlobalMapBuildResult(
            width=int(build_result.width),
            height=int(build_result.height),
            resolution_m=resolution_m,
            origin_x=float(build_result.origin_x),
            origin_y=float(build_result.origin_y),
            occupancy_data=occupancy_data,
            cost_data=cost_data,
            occupied_mask=occupied_mask,
            source_label=build_result.source_label,
            known_cell_count=int(build_result.known_cell_count),
        )

    def _build_contract_maps_from_global_result(
        self,
        build_result: Optional[Go2GlobalMapBuildResult],
        *,
        current_pose: Pose,
    ) -> Tuple[Optional[OccupancyGrid], Optional[CostMap], Optional[SemanticMap]]:
        if build_result is None:
            return None, None, None
        build_result = self._clear_navigation_origin_footprint(
            build_result=build_result,
            current_pose=current_pose,
        )
        frame_id = current_pose.frame_id or self.config.map_synthesis.map_frame
        origin_pose = Pose(
            frame_id=frame_id,
            position=Vector3(
                x=float(build_result.origin_x),
                y=float(build_result.origin_y),
                z=float(current_pose.position.z),
            ),
            orientation=Quaternion(w=1.0),
        )
        occupancy_grid = OccupancyGrid(
            map_id="go2_global_cloud_occupancy",
            frame_id=frame_id,
            width=int(build_result.width),
            height=int(build_result.height),
            resolution_m=float(build_result.resolution_m),
            origin=origin_pose,
            data=[int(value) for value in build_result.occupancy_data.flatten().tolist()],
        )
        cost_map = CostMap(
            map_id="go2_global_cloud_cost",
            frame_id=frame_id,
            width=int(build_result.width),
            height=int(build_result.height),
            resolution_m=float(build_result.resolution_m),
            origin=origin_pose,
            data=[float(value) for value in build_result.cost_data.flatten().tolist()],
        )
        semantic_map = self._build_semantic_map_from_global_cloud(
            occupancy=build_result.occupancy_data,
            occupied_mask=build_result.occupied_mask,
            frame_id=frame_id,
            origin=origin_pose,
            resolution=float(build_result.resolution_m),
            width=int(build_result.width),
            height=int(build_result.height),
            region_source=build_result.source_label,
        )
        return occupancy_grid, cost_map, semantic_map

    def _build_semantic_map_from_local_cloud(
        self,
        *,
        occupancy: np.ndarray,
        occupied_mask: np.ndarray,
        frame_id: str,
        origin: Pose,
        resolution: float,
        width: int,
        height: int,
        region_source: str,
    ) -> Optional[SemanticMap]:
        return self._build_semantic_map_from_grid_data(
            occupancy=occupancy,
            occupied_mask=occupied_mask,
            frame_id=frame_id,
            origin=origin,
            resolution=resolution,
            width=width,
            height=height,
            region_source=region_source,
            map_id="go2_local_cloud_semantic_map",
        )

    def _build_semantic_map_from_global_cloud(
        self,
        *,
        occupancy: np.ndarray,
        occupied_mask: np.ndarray,
        frame_id: str,
        origin: Pose,
        resolution: float,
        width: int,
        height: int,
        region_source: str,
    ) -> Optional[SemanticMap]:
        return self._build_semantic_map_from_grid_data(
            occupancy=occupancy,
            occupied_mask=occupied_mask,
            frame_id=frame_id,
            origin=origin,
            resolution=resolution,
            width=width,
            height=height,
            region_source=region_source,
            map_id="go2_global_cloud_semantic_map",
        )

    def _build_semantic_map_from_grid_data(
        self,
        *,
        occupancy: np.ndarray,
        occupied_mask: np.ndarray,
        frame_id: str,
        origin: Pose,
        resolution: float,
        width: int,
        height: int,
        region_source: str,
        map_id: str,
    ) -> Optional[SemanticMap]:
        free_mask = np.asarray(occupancy == 0, dtype=np.uint8)
        hazard_mask = np.asarray(occupied_mask, dtype=np.uint8)
        regions: List[SemanticRegion] = []
        if np.any(free_mask):
            regions.extend(
                self._regions_from_mask(
                    mask=free_mask,
                    score_source=np.asarray(free_mask, dtype=np.float32),
                    label="traversable",
                    frame_id=frame_id,
                    width=width,
                    height=height,
                    origin=origin,
                    resolution=resolution,
                    region_source=region_source,
                )
            )
        if np.any(hazard_mask):
            regions.extend(
                self._regions_from_mask(
                    mask=hazard_mask,
                    score_source=np.asarray(hazard_mask, dtype=np.float32),
                    label="hazard",
                    frame_id=frame_id,
                    width=width,
                    height=height,
                    origin=origin,
                    resolution=resolution,
                    region_source=region_source,
                )
            )
        if not regions:
            return None
        return SemanticMap(
            map_id=map_id,
            frame_id=frame_id,
            regions=regions,
            metadata={"source": region_source},
        )

    def _yaw_from_quaternion(self, rotation: Quaternion) -> float:
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_transform_from_ros(self, message: object) -> Transform:
        transform = getattr(message, "transform", None)
        header = getattr(message, "header", None)
        return Transform(
            parent_frame_id=str(getattr(header, "frame_id", "") or self.config.map_synthesis.map_frame),
            child_frame_id=str(getattr(message, "child_frame_id", "") or self.config.map_synthesis.base_frame),
            translation=Vector3(
                x=float(getattr(getattr(transform, "translation", None), "x", 0.0)),
                y=float(getattr(getattr(transform, "translation", None), "y", 0.0)),
                z=float(getattr(getattr(transform, "translation", None), "z", 0.0)),
            ),
            rotation=Quaternion(
                x=float(getattr(getattr(transform, "rotation", None), "x", 0.0)),
                y=float(getattr(getattr(transform, "rotation", None), "y", 0.0)),
                z=float(getattr(getattr(transform, "rotation", None), "z", 0.0)),
                w=float(getattr(getattr(transform, "rotation", None), "w", 1.0)),
            ),
            authority="ros2_tf",
        )

    def _build_contract_occupancy_grid(self, message: object) -> OccupancyGrid:
        info = getattr(message, "info", None)
        header = getattr(message, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or self.config.map_synthesis.map_frame)
        return OccupancyGrid(
            map_id="go2_occupancy_map",
            frame_id=frame_id,
            width=int(getattr(info, "width", 1)),
            height=int(getattr(info, "height", 1)),
            resolution_m=float(getattr(info, "resolution", 0.1) or 0.1),
            origin=self._build_pose_from_ros_pose(getattr(info, "origin", None), frame_id=frame_id),
            data=[int(value) for value in getattr(message, "data", [])],
        )

    def _build_contract_cost_map(self, message: object) -> CostMap:
        metadata = getattr(message, "metadata", None)
        frame_id = str(getattr(metadata, "layer", "") or self.config.map_synthesis.map_frame)
        if metadata is not None and getattr(metadata, "map_load_time", None) is not None:
            frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or self.config.map_synthesis.map_frame)
        width = int(getattr(metadata, "size_x", getattr(message, "width", 1)))
        height = int(getattr(metadata, "size_y", getattr(message, "height", 1)))
        resolution = float(getattr(metadata, "resolution", getattr(message, "resolution", 0.1)) or 0.1)
        origin_pose = getattr(metadata, "origin", getattr(message, "origin", None))
        raw_data = getattr(message, "data", [])
        return CostMap(
            map_id="go2_official_cost_map",
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=self._build_pose_from_ros_pose(origin_pose, frame_id=frame_id),
            data=[float(value) for value in raw_data],
        )

    def _build_pose_from_ros_pose(self, pose_obj: object, *, frame_id: str) -> Pose:
        return Pose(
            frame_id=frame_id,
            position=Vector3(
                x=float(getattr(getattr(pose_obj, "position", None), "x", 0.0)),
                y=float(getattr(getattr(pose_obj, "position", None), "y", 0.0)),
                z=float(getattr(getattr(pose_obj, "position", None), "z", 0.0)),
            ),
            orientation=Quaternion(
                x=float(getattr(getattr(pose_obj, "orientation", None), "x", 0.0)),
                y=float(getattr(getattr(pose_obj, "orientation", None), "y", 0.0)),
                z=float(getattr(getattr(pose_obj, "orientation", None), "z", 0.0)),
                w=float(getattr(getattr(pose_obj, "orientation", None), "w", 1.0)),
            ),
        )

    def _grid_map_dimensions(self, message: object) -> Tuple[int, int]:
        info = getattr(message, "info", None)
        resolution = float(getattr(info, "resolution", 0.1) or 0.1)
        length_x = float(getattr(info, "length_x", resolution))
        length_y = float(getattr(info, "length_y", resolution))
        width = max(1, int(round(length_x / resolution)))
        height = max(1, int(round(length_y / resolution)))
        return width, height

    def _grid_map_origin_pose(self, message: object, *, frame_id: str) -> Pose:
        info = getattr(message, "info", None)
        pose_obj = getattr(info, "pose", None)
        length_x = float(getattr(info, "length_x", 0.0))
        length_y = float(getattr(info, "length_y", 0.0))
        center_x = float(getattr(getattr(pose_obj, "position", None), "x", 0.0))
        center_y = float(getattr(getattr(pose_obj, "position", None), "y", 0.0))
        center_z = float(getattr(getattr(pose_obj, "position", None), "z", 0.0))
        return Pose(
            frame_id=frame_id,
            position=Vector3(
                x=center_x - length_x / 2.0,
                y=center_y - length_y / 2.0,
                z=center_z,
            ),
            orientation=Quaternion(
                x=float(getattr(getattr(pose_obj, "orientation", None), "x", 0.0)),
                y=float(getattr(getattr(pose_obj, "orientation", None), "y", 0.0)),
                z=float(getattr(getattr(pose_obj, "orientation", None), "z", 0.0)),
                w=float(getattr(getattr(pose_obj, "orientation", None), "w", 1.0)),
            ),
        )

    def _extract_grid_layer(self, message: object, layer_name: str) -> Optional[np.ndarray]:
        layers = list(getattr(message, "layers", []))
        if layer_name not in layers:
            return None
        layer_index = layers.index(layer_name)
        data_entries = list(getattr(message, "data", []))
        if layer_index >= len(data_entries):
            return None
        raw_data = np.asarray(list(getattr(data_entries[layer_index], "data", [])), dtype=np.float32)
        width, height = self._grid_map_dimensions(message)
        if raw_data.size != width * height:
            return None
        matrix = raw_data.reshape(height, width)
        outer_start_index = int(getattr(message, "outer_start_index", 0))
        inner_start_index = int(getattr(message, "inner_start_index", 0))
        if outer_start_index:
            matrix = np.roll(matrix, -outer_start_index, axis=0)
        if inner_start_index:
            matrix = np.roll(matrix, -inner_start_index, axis=1)
        return matrix

    def _build_occupancy_from_grid_map(self, message: object) -> Optional[OccupancyGrid]:
        traversability = self._extract_grid_layer(message, self.config.map_synthesis.traversability_layer)
        if traversability is None:
            return None
        validity = None
        if self.config.map_synthesis.validity_layer.strip():
            validity = self._extract_grid_layer(message, self.config.map_synthesis.validity_layer)
        occupancy = np.clip(np.rint((1.0 - traversability) * 100.0), 0, 100).astype(np.int32)
        occupancy = np.where(traversability >= self.config.map_synthesis.free_threshold, 0, occupancy)
        occupancy = np.where(traversability <= self.config.map_synthesis.lethal_threshold, 100, occupancy)
        if validity is not None:
            occupancy = np.where(validity <= 0.0, -1, occupancy)
        width, height = self._grid_map_dimensions(message)
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or self.config.map_synthesis.map_frame)
        return OccupancyGrid(
            map_id="go2_gridmap_occupancy",
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=float(getattr(getattr(message, "info", None), "resolution", 0.1) or 0.1),
            origin=self._grid_map_origin_pose(message, frame_id=frame_id),
            data=[int(value) for value in occupancy.flatten().tolist()],
        )

    def _build_cost_map_from_grid_map(self, message: object) -> Optional[CostMap]:
        traversability = self._extract_grid_layer(message, self.config.map_synthesis.traversability_layer)
        if traversability is None:
            return None
        validity = None
        if self.config.map_synthesis.validity_layer.strip():
            validity = self._extract_grid_layer(message, self.config.map_synthesis.validity_layer)
        cost = np.clip((1.0 - traversability) * 100.0, 0.0, 100.0)
        if validity is not None:
            cost = np.where(validity <= 0.0, 100.0, cost)
        width, height = self._grid_map_dimensions(message)
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or self.config.map_synthesis.map_frame)
        return CostMap(
            map_id="go2_gridmap_cost",
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=float(getattr(getattr(message, "info", None), "resolution", 0.1) or 0.1),
            origin=self._grid_map_origin_pose(message, frame_id=frame_id),
            data=[float(value) for value in cost.flatten().tolist()],
        )

    def _build_semantic_map_from_grid_map(self, message: object) -> Optional[SemanticMap]:
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", "") or self.config.map_synthesis.map_frame)
        width, height = self._grid_map_dimensions(message)
        origin = self._grid_map_origin_pose(message, frame_id=frame_id)
        resolution = float(getattr(getattr(message, "info", None), "resolution", 0.1) or 0.1)
        semantic_layers = list(self.config.map_synthesis.semantic_layers)
        if not semantic_layers:
            semantic_layers = [
                layer
                for layer in list(getattr(message, "layers", []))
                if layer not in _KNOWN_GRID_MAP_LAYERS
            ]
        regions: List[SemanticRegion] = []
        for layer_name in semantic_layers:
            mask_source = self._extract_grid_layer(message, layer_name)
            if mask_source is None:
                continue
            mask = np.asarray(mask_source >= self.config.map_synthesis.semantic_threshold, dtype=np.uint8)
            regions.extend(
                self._regions_from_mask(
                    mask=mask,
                    score_source=mask_source,
                    label=layer_name,
                    frame_id=frame_id,
                    width=width,
                    height=height,
                    origin=origin,
                    resolution=resolution,
                )
            )
        if not regions:
            traversability = self._extract_grid_layer(message, self.config.map_synthesis.traversability_layer)
            if traversability is not None:
                regions.extend(
                    self._regions_from_mask(
                        mask=np.asarray(traversability >= self.config.map_synthesis.free_threshold, dtype=np.uint8),
                        score_source=traversability,
                        label="traversable",
                        frame_id=frame_id,
                        width=width,
                        height=height,
                        origin=origin,
                        resolution=resolution,
                    )
                )
                regions.extend(
                    self._regions_from_mask(
                        mask=np.asarray(traversability <= self.config.map_synthesis.lethal_threshold, dtype=np.uint8),
                        score_source=1.0 - traversability,
                        label="hazard",
                        frame_id=frame_id,
                        width=width,
                        height=height,
                        origin=origin,
                        resolution=resolution,
                    )
                )
        if not regions:
            return None
        return SemanticMap(
            map_id="go2_semantic_map",
            frame_id=frame_id,
            regions=regions,
            metadata={"source": "grid_map"},
        )

    def _regions_from_mask(
        self,
        *,
        mask: np.ndarray,
        score_source: np.ndarray,
        label: str,
        frame_id: str,
        width: int,
        height: int,
        origin: Pose,
        resolution: float,
        region_source: str = "grid_map",
    ) -> List[SemanticRegion]:
        if mask.size == 0:
            return []
        visited = np.zeros_like(mask, dtype=np.uint8)
        regions: List[SemanticRegion] = []
        region_index = 0
        for row in range(height):
            for col in range(width):
                if mask[row, col] == 0 or visited[row, col] == 1:
                    continue
                component = self._flood_fill(mask, visited, start_row=row, start_col=col)
                if len(component) < self.config.map_synthesis.semantic_min_cells:
                    continue
                xs: List[float] = []
                ys: List[float] = []
                scores: List[float] = []
                rows = [cell[0] for cell in component]
                cols = [cell[1] for cell in component]
                for item_row, item_col in component:
                    xs.append(origin.position.x + (item_col + 0.5) * resolution)
                    ys.append(origin.position.y + (item_row + 0.5) * resolution)
                    scores.append(float(score_source[item_row, item_col]))
                min_row = min(rows)
                max_row = max(rows)
                min_col = min(cols)
                max_col = max(cols)
                polygon = [
                    Vector3(x=origin.position.x + min_col * resolution, y=origin.position.y + min_row * resolution, z=0.0),
                    Vector3(x=origin.position.x + (max_col + 1) * resolution, y=origin.position.y + min_row * resolution, z=0.0),
                    Vector3(x=origin.position.x + (max_col + 1) * resolution, y=origin.position.y + (max_row + 1) * resolution, z=0.0),
                    Vector3(x=origin.position.x + min_col * resolution, y=origin.position.y + (max_row + 1) * resolution, z=0.0),
                ]
                region_index += 1
                regions.append(
                    SemanticRegion(
                        region_id=f"{label}_{region_index}",
                        label=label,
                        centroid=Pose(
                            frame_id=frame_id,
                            position=Vector3(
                                x=float(sum(xs) / len(xs)),
                                y=float(sum(ys) / len(ys)),
                                z=0.0,
                            ),
                            orientation=Quaternion(w=1.0),
                        ),
                        polygon_points=polygon,
                        attributes={
                            "source": region_source,
                            "cell_count": len(component),
                            "score_mean": float(sum(scores) / len(scores)),
                        },
                    )
                )
        return regions

    def _flood_fill(self, mask: np.ndarray, visited: np.ndarray, *, start_row: int, start_col: int) -> List[Tuple[int, int]]:
        queue = [(start_row, start_col)]
        visited[start_row, start_col] = 1
        component: List[Tuple[int, int]] = []
        height, width = mask.shape
        while queue:
            row, col = queue.pop()
            component.append((row, col))
            neighbors = (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            )
            for next_row, next_col in neighbors:
                if next_row < 0 or next_row >= height or next_col < 0 or next_col >= width:
                    continue
                if visited[next_row, next_col] == 1 or mask[next_row, next_col] == 0:
                    continue
                visited[next_row, next_col] = 1
                queue.append((next_row, next_col))
        return component


class Go2DataPlaneRuntime:
    """Go2 定位、地图、导航、探索数据面运行时。"""

    def __init__(
        self,
        config: "Go2DataPlaneConfig",
        *,
        bridge: Optional[Any] = None,
    ) -> None:
        self.config = config
        self.bridge = bridge or RclpyGo2RosBridge(config)
        self._processes: List[ManagedRos2Process] = []
        self._startup_diagnostics: List[str] = []
        self._navigation_lock = threading.RLock()
        self._exploration_lock = threading.RLock()
        self._exploration_stop_event = threading.Event()
        self._exploration_thread: Optional[threading.Thread] = None
        self._exploration_state = ExplorationState(status=ExplorationStatus.IDLE)
        self._official_sport_client = None
        self._official_robot_state_client = None
        self._official_obstacles_avoid_client = None
        self._official_sport_ready = False
        self._official_robot_state_ready = False
        self._official_obstacles_avoid_ready = False
        self._official_obstacles_avoid_remote_control = False
        self._official_obstacles_avoid_switch_state: Optional[bool] = None
        self._last_motion_command_route: Optional[str] = None
        self._official_service_states: Dict[str, Dict[str, object]] = {}
        self._official_goal_thread: Optional[threading.Thread] = None
        self._official_goal_stop_event = threading.Event()
        self._official_navigation_goal: Optional[NavigationGoal] = None
        self._official_navigation_state = NavigationState(status=NavigationStatus.IDLE, message="官方导航后端未启用。")
        self._official_goal_reached = False
        self._official_last_error: Optional[str] = None
        self._loaded_named_map_name: Optional[str] = None
        self._loaded_named_map_mode: str = "none"
        self._loaded_named_map_saved_at: Optional[str] = None
        self._loaded_named_map_requires_localization: bool = False
        self._archived_occupancy_grid: Optional[OccupancyGrid] = None
        self._archived_cost_map: Optional[CostMap] = None
        self._archived_semantic_map: Optional[SemanticMap] = None
        self._grid_navigation_planner = GridNavigationPlanner(self.config.navigation)
        self._frontier_explorer = FrontierExplorer(self.config.exploration)
        self._obstacle_avoidance_enabled = bool(
            getattr(getattr(self.config, "official", None), "obstacle_avoidance_default_enabled", True)
        )
        self._started = False
        self._info_log_limiter = LogRateLimiter()

    def _named_map_archive_root(self) -> Path:
        root = str(getattr(self.config, "named_map_archive_root", "") or "").strip()
        if not root:
            root = str(Path.cwd() / "runtime_data" / "go2_named_maps")
        return Path(root).expanduser()

    def _normalize_named_map_name(self, map_name: str) -> str:
        normalized = []
        for char in str(map_name or "").strip().lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        return "".join(normalized).strip("_") or "map"

    def _named_map_archive_path(self, map_name: str) -> Path:
        return self._named_map_archive_root() / self._normalize_named_map_name(map_name) / "snapshot.json"

    def _clear_archived_named_map_snapshot(self) -> None:
        self._archived_occupancy_grid = None
        self._archived_cost_map = None
        self._archived_semantic_map = None

    def _has_archived_named_map_snapshot(self) -> bool:
        return any(
            item is not None
            for item in (
                self._archived_occupancy_grid,
                self._archived_cost_map,
                self._archived_semantic_map,
            )
        )

    def _uses_archived_named_map_snapshot(self) -> bool:
        return self._loaded_named_map_mode == "archive_snapshot" and self._has_archived_named_map_snapshot()

    def get_loaded_map_name(self) -> Optional[str]:
        """返回当前命名地图运行态绑定的地图名称。"""

        return self._loaded_named_map_name

    def load_named_map(
        self,
        map_name: str,
        *,
        reason: str = "",
        allow_missing: bool = False,
    ) -> Dict[str, Any]:
        """按地图名加载命名地图运行态。

        兼容路径语义：
        1. 若本地存在归档快照，则进入 `archive_snapshot` 模式。
        2. 若归档缺失但允许缺失，则只绑定当前 live runtime（实时运行态）到该名称。
        3. `archive_snapshot` 模式不会伪造定位可用状态。

        说明：
        该能力仅保留给兼容/调试路径使用，
        平台主线不再依赖它作为地图真值来源。
        """

        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            raise ValueError("地图名称不能为空。")

        archive_path = self._named_map_archive_path(resolved_name)
        if archive_path.exists():
            payload = json.loads(archive_path.read_text(encoding="utf-8"))
            self._archived_occupancy_grid = (
                OccupancyGrid.model_validate(payload["occupancy_grid"])
                if payload.get("occupancy_grid") is not None
                else None
            )
            self._archived_cost_map = (
                CostMap.model_validate(payload["cost_map"])
                if payload.get("cost_map") is not None
                else None
            )
            self._archived_semantic_map = (
                SemanticMap.model_validate(payload["semantic_map"])
                if payload.get("semantic_map") is not None
                else None
            )
            self._loaded_named_map_name = resolved_name
            self._loaded_named_map_mode = "archive_snapshot"
            self._loaded_named_map_saved_at = str(payload.get("saved_at", "") or "") or None
            self._loaded_named_map_requires_localization = True
            LOGGER.info("Go2 命名地图归档已加载 map_name=%s reason=%s", resolved_name, reason or "load_named_map")
            return {
                "requested_map_name": resolved_name,
                "loaded_map_name": resolved_name,
                "loaded": True,
                "archive_found": True,
                "runtime_mode": self._loaded_named_map_mode,
                "requires_localization": True,
                "archive_path": str(archive_path),
                "saved_at": self._loaded_named_map_saved_at,
            }

        if not allow_missing:
            return {
                "requested_map_name": resolved_name,
                "loaded_map_name": self._loaded_named_map_name,
                "loaded": False,
                "archive_found": False,
                "runtime_mode": self._loaded_named_map_mode,
                "requires_localization": self._loaded_named_map_requires_localization,
                "archive_path": str(archive_path),
            }

        self._clear_archived_named_map_snapshot()
        self._loaded_named_map_name = resolved_name
        self._loaded_named_map_mode = "live_runtime"
        self._loaded_named_map_saved_at = None
        self._loaded_named_map_requires_localization = False
        LOGGER.info("Go2 命名地图运行态已绑定到 live runtime map_name=%s reason=%s", resolved_name, reason or "load_named_map")
        return {
            "requested_map_name": resolved_name,
            "loaded_map_name": resolved_name,
            "loaded": True,
            "archive_found": False,
            "runtime_mode": self._loaded_named_map_mode,
            "requires_localization": False,
            "archive_path": str(archive_path),
        }

    def save_named_map(self, map_name: str, *, reason: str = "") -> Dict[str, Any]:
        """把当前地图快照保存为命名地图归档。

        该能力仅保留给兼容/调试路径使用，
        平台主线不再依赖它作为地图版本保存路径。
        """

        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            raise ValueError("地图名称不能为空。")
        occupancy_grid = self.bridge.get_occupancy_grid()
        cost_map = self.bridge.get_cost_map()
        semantic_map = self.bridge.get_semantic_map()
        if occupancy_grid is None and cost_map is None and semantic_map is None:
            return {
                "map_name": resolved_name,
                "saved": False,
                "runtime_mode": self._loaded_named_map_mode,
                "reason": "当前没有可归档的地图快照。",
            }
        archive_path = self._named_map_archive_path(resolved_name)
        archive_path.parent.mkdir(parents=True, exist_ok=True)
        saved_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        archive_path.write_text(
            json.dumps(
                {
                    "map_name": resolved_name,
                    "saved_at": saved_at,
                    "reason": str(reason or ""),
                    "occupancy_grid": occupancy_grid.model_dump(mode="json") if occupancy_grid is not None else None,
                    "cost_map": cost_map.model_dump(mode="json") if cost_map is not None else None,
                    "semantic_map": semantic_map.model_dump(mode="json") if semantic_map is not None else None,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        self._clear_archived_named_map_snapshot()
        self._loaded_named_map_name = resolved_name
        self._loaded_named_map_mode = "live_runtime"
        self._loaded_named_map_saved_at = saved_at
        self._loaded_named_map_requires_localization = False
        LOGGER.info("Go2 命名地图归档已保存 map_name=%s reason=%s", resolved_name, reason or "save_named_map")
        return {
            "map_name": resolved_name,
            "saved": True,
            "runtime_mode": self._loaded_named_map_mode,
            "archive_path": str(archive_path),
            "saved_at": saved_at,
        }

    def get_named_map_runtime_status(self) -> Dict[str, Any]:
        """返回当前命名地图运行态状态。

        该状态仅用于兼容/调试观测，
        不代表平台当前正式地图真值源。
        """

        return {
            "loaded_map_name": self._loaded_named_map_name,
            "runtime_mode": self._loaded_named_map_mode,
            "saved_at": self._loaded_named_map_saved_at,
            "requires_localization": self._loaded_named_map_requires_localization,
            "archive_root": str(self._named_map_archive_root()),
            "archive_snapshot_active": self._uses_archived_named_map_snapshot(),
            "runtime_scope": "compatibility_only",
            "used_by_platform": False,
        }

    def is_obstacle_avoidance_control_available(self) -> bool:
        """返回官方避障控制链路是否真正可用。"""

        return self._official_obstacles_avoid_client is not None and self._official_obstacles_avoid_ready

    def _require_obstacle_avoidance_client(self):
        client = self._official_obstacles_avoid_client
        if client is None or not self._official_obstacles_avoid_ready:
            raise RuntimeError("Go2 官方 obstacles_avoid 服务未就绪。")
        return client

    def _read_obstacle_avoidance_switch_state(self) -> bool:
        client = self._require_obstacle_avoidance_client()
        code, enabled = client.SwitchGet()
        if code != 0:
            raise RuntimeError(f"读取 Go2 官方 obstacles_avoid 状态失败 code={code}")
        self._official_obstacles_avoid_switch_state = bool(enabled)
        return bool(enabled)

    def _set_obstacle_avoidance_switch(self, enabled: bool) -> bool:
        client = self._require_obstacle_avoidance_client()
        code = client.SwitchSet(bool(enabled))
        if code != 0:
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
        LOGGER.info("Go2 官方避障远程命令模式已%s。", "启用" if enabled else "关闭")

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, Any]:
        """开关实时避障功能，并返回真实后端状态。"""

        previous_enabled = self._obstacle_avoidance_enabled
        self._obstacle_avoidance_enabled = bool(enabled)
        try:
            self._ensure_official_services_enabled(
                service_names=("obstacles_avoid",),
                reason="set_obstacle_avoidance",
            )
            switch_enabled = self._set_obstacle_avoidance_switch(bool(enabled))
            if not enabled and self._official_obstacles_avoid_remote_control:
                self._require_obstacle_avoidance_client().Move(0.0, 0.0, 0.0)
                self._set_obstacle_avoidance_remote_command(False)
        except Exception:
            self._obstacle_avoidance_enabled = previous_enabled
            raise
        LOGGER.info("官方避障已%s。", "启用" if switch_enabled else "禁用")
        return self.get_obstacle_avoidance_status()

    def is_obstacle_avoidance_enabled(self) -> bool:
        """返回当前避障是否开启。"""
        return self._obstacle_avoidance_enabled

    def get_obstacle_avoidance_status(self) -> Dict[str, Any]:
        """返回实时避障运行状态。"""

        backend_ready = self.is_obstacle_avoidance_control_available()
        switch_enabled = (
            bool(self._official_obstacles_avoid_switch_state)
            if self._official_obstacles_avoid_switch_state is not None
            else False
        )
        if backend_ready:
            try:
                switch_enabled = self._read_obstacle_avoidance_switch_state()
            except Exception as exc:
                LOGGER.warning("读取官方避障开关状态失败: %s", exc)
        return {
            "obstacle_avoidance_enabled": self._obstacle_avoidance_enabled,
            "default_enabled": bool(
                getattr(getattr(self.config, "official", None), "obstacle_avoidance_default_enabled", True)
            ),
            "backend_ready": backend_ready,
            "switch_enabled": switch_enabled,
            "remote_command_enabled": self._official_obstacles_avoid_remote_control,
            "control_path": "obstacles_avoid" if backend_ready and self._obstacle_avoidance_enabled else "sport",
        }

    def ensure_mapping_runtime_enabled(self, *, reason: str = "") -> Dict[str, Any]:
        """按需启用 Go2 官方建图相关服务。"""

        service_names = self._mapping_related_official_service_names()
        self._ensure_official_services_enabled(service_names=service_names, reason=reason or "mapping_runtime")
        bridge_mapping_runtime = None
        enable_bridge_mapping = getattr(self.bridge, "enable_mapping_runtime", None)
        if callable(enable_bridge_mapping):
            try:
                bridge_mapping_runtime = enable_bridge_mapping(reason=reason or "mapping_runtime")
            except TypeError:
                bridge_mapping_runtime = enable_bridge_mapping()
            except Exception:
                LOGGER.exception("按需启用 Go2 点云地图合成链失败 reason=%s", reason or "mapping_runtime")
        return {
            "service_names": list(service_names),
            "service_states": dict(self._official_service_states),
            "localization_available": self.is_localization_available(),
            "map_available": self.is_map_available(),
            "navigation_available": self.is_navigation_available(),
            "bridge_mapping_runtime": bridge_mapping_runtime,
        }

    def disable_mapping_runtime(self, *, reason: str = "") -> Dict[str, Any]:
        """在按需模式下回收 bridge 侧点云建图运行态。"""

        bridge_mapping_runtime = None
        disable_bridge_mapping = getattr(self.bridge, "disable_mapping_runtime", None)
        if callable(disable_bridge_mapping):
            try:
                bridge_mapping_runtime = disable_bridge_mapping(reason=reason or "mapping_runtime")
            except TypeError:
                bridge_mapping_runtime = disable_bridge_mapping()
            except Exception:
                LOGGER.exception("回收 Go2 点云地图合成链失败 reason=%s", reason or "mapping_runtime")
        return {
            "service_names": list(self._mapping_related_official_service_names()),
            "service_states": dict(self._official_service_states),
            "localization_available": self.is_localization_available(),
            "map_available": self.is_map_available(),
            "navigation_available": self.is_navigation_available(),
            "bridge_mapping_runtime": bridge_mapping_runtime,
        }

    def _mapping_related_official_service_names(self) -> Tuple[str, ...]:
        """返回建图相关官方服务名。"""

        service_names = []
        for raw_name in self.config.official.service_names:
            normalized = str(raw_name or "").strip()
            if not normalized or normalized == "obstacles_avoid":
                continue
            service_names.append(normalized)
        return tuple(service_names)

    def _ensure_official_services_enabled(self, *, service_names: Sequence[str], reason: str) -> None:
        """按需确保指定官方服务已经拉起。"""

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
            extra={
                "event": "go2.service.ensure",
                "reason": reason,
                "services": list(normalized_names),
            },
        )
        self._refresh_official_service_states()
        for service_name in normalized_names:
            self._ensure_official_service_enabled(service_name)
        self._refresh_official_service_states()
        self._refresh_official_navigation_idle_state()

    def _ensure_obstacle_avoidance_active(self) -> bool:
        """确保官方避障处于激活状态，并切到官方避障远程命令模式。"""

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
                extra={
                    "event": "go2.obstacles_avoid.best_effort",
                    "error": str(exc),
                },
            )
            try:
                self._release_obstacle_avoidance_remote_command()
            except Exception:
                LOGGER.debug("回收 Go2 官方避障远程命令模式失败。", exc_info=True)
            return False

    def _is_obstacle_avoidance_best_effort_error(self, error: Exception) -> bool:
        message = str(error)
        return "code=3102" in message or "code=3104" in message

    def _release_obstacle_avoidance_remote_command(self) -> None:
        """在切回普通运动控制前释放官方避障远程命令模式。"""

        if not self._official_obstacles_avoid_remote_control:
            return
        if not self.is_obstacle_avoidance_control_available():
            self._official_obstacles_avoid_remote_control = False
            return
        self._set_obstacle_avoidance_remote_command(False)

    def _send_official_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        """根据当前避障配置发送官方运动命令。"""

        if self._ensure_obstacle_avoidance_active():
            route = "obstacles_avoid"
            code = self._require_obstacle_avoidance_client().Move(
                float(vx),
                float(vy),
                float(vyaw),
            )
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
        """记录当前实际运动命令链路，便于远端排查避障是否接管。"""

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
        LOGGER.info("Go2 运动命令链路已切换", extra=extra)

    def can_accept_motion_command(self) -> bool:
        """返回当前数据面是否可以直接下发运动命令。"""

        return self._uses_official_backend() and self._official_sport_ready

    def send_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        """对外暴露的统一运动命令入口。"""

        if not self.can_accept_motion_command():
            raise RuntimeError("当前 Go2 数据面未就绪，不能直接下发运动命令。")
        return self._send_official_motion_command(vx, vy, vyaw)

    def stop_motion_command(self) -> None:
        """对外暴露的统一停止命令入口。"""

        if not self.can_accept_motion_command():
            return
        self._stop_official_motion()

    def start(self) -> None:
        """启动数据面运行时。"""

        if not self.config.enabled:
            LOGGER.info("Go2 数据面未启用，跳过启动。")
            return
        if self._started:
            return
        if _uses_sdk2_in_process(self.config) and _is_ros_environment_active():
            message = _sdk2_ros_conflict_message()
            self._startup_diagnostics = [message]
            raise RuntimeError(message)
        self._startup_diagnostics = []
        self._start_official_backend()
        self._processes = []
        self.bridge.start()
        self._wait_for_initial_readiness()
        self._refresh_official_navigation_idle_state()
        self._started = True

    def stop(self) -> None:
        """停止数据面运行时。"""

        self.stop_exploration()
        self._stop_official_navigation(update_state=False)
        self.bridge.stop()
        for process in reversed(self._processes):
            process.stop()
        self._started = False

    def is_started(self) -> bool:
        """返回数据面是否已启动。"""

        return self._started

    def is_localization_available(self) -> bool:
        """返回定位能力是否可用。"""

        if self._loaded_named_map_requires_localization:
            return False
        return self.config.enabled and bool(self.bridge.is_localization_available())

    def is_map_available(self) -> bool:
        """返回地图能力是否可用。"""

        if self._uses_archived_named_map_snapshot():
            return True
        return self.config.enabled and bool(self.bridge.is_map_available())

    def is_navigation_available(self) -> bool:
        """返回导航能力是否可用。"""

        return self.config.enabled and self._official_sport_ready and self.is_localization_available()

    def is_exploration_available(self) -> bool:
        """返回探索能力是否可用。"""

        if not self.config.exploration.enabled or not self.is_navigation_available():
            return False
        return self.is_localization_available() or self.is_map_available()

    def get_current_pose(self) -> Optional[Pose]:
        """读取当前位姿。"""

        if self._loaded_named_map_requires_localization:
            return None
        return self.bridge.get_current_pose()

    def get_frame_tree(self) -> Optional[FrameTree]:
        """读取当前 TF 树。"""

        if self._loaded_named_map_requires_localization:
            return None
        return self.bridge.get_frame_tree()

    def get_imu_state(self) -> Optional[IMUState]:
        """读取当前 IMU 状态。"""

        return self.bridge.get_imu_state()

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """读取占据栅格地图。"""

        if self._uses_archived_named_map_snapshot():
            return self._archived_occupancy_grid.model_copy(deep=True) if self._archived_occupancy_grid is not None else None
        return self.bridge.get_occupancy_grid()

    def get_cost_map(self) -> Optional[CostMap]:
        """读取代价地图。"""

        if self._uses_archived_named_map_snapshot():
            return self._archived_cost_map.model_copy(deep=True) if self._archived_cost_map is not None else None
        return self.bridge.get_cost_map()

    def get_semantic_map(self) -> Optional[SemanticMap]:
        """读取语义地图。"""

        if self._uses_archived_named_map_snapshot():
            return self._archived_semantic_map.model_copy(deep=True) if self._archived_semantic_map is not None else None
        return self.bridge.get_semantic_map()

    def set_goal(self, goal: NavigationGoal) -> bool:
        """提交导航目标。"""

        if self._uses_official_backend():
            return self._set_official_goal(goal)
        return self.bridge.set_goal(goal)

    def cancel_goal(self) -> bool:
        """取消导航目标。"""

        if self._uses_official_backend():
            return self._stop_official_navigation(update_state=True)
        return self.bridge.cancel_goal()

    def get_navigation_state(self) -> NavigationState:
        """读取导航状态。"""

        if self._uses_official_backend():
            with self._navigation_lock:
                return self._official_navigation_state.model_copy(deep=True)
        return self.bridge.get_navigation_state()

    def is_goal_reached(self) -> bool:
        """判断当前导航目标是否已完成。"""

        if self._uses_official_backend():
            with self._navigation_lock:
                return self._official_goal_reached
        return self.bridge.is_goal_reached()

    def start_exploration(self, request: ExploreAreaRequest) -> bool:
        """启动探索任务。"""

        self.ensure_mapping_runtime_enabled(reason="exploration")
        if not self.is_exploration_available():
            with self._exploration_lock:
                self._exploration_state = ExplorationState(
                    current_request_id=request.request_id,
                    status=ExplorationStatus.FAILED,
                    strategy=request.strategy,
                    message=self._build_exploration_unavailable_message(),
                )
            return False
        with self._exploration_lock:
            if self._exploration_thread is not None and self._exploration_thread.is_alive():
                return False
            self._exploration_stop_event.clear()
            self._exploration_state = ExplorationState(
                current_request_id=request.request_id,
                status=ExplorationStatus.ACCEPTED,
                strategy=request.strategy,
                message="探索任务已接受。",
            )
            self._exploration_thread = threading.Thread(
                target=self._run_exploration,
                args=(request,),
                name=f"go2_exploration_{request.request_id}",
                daemon=True,
            )
            self._exploration_thread.start()
        return True

    def stop_exploration(self) -> bool:
        """停止探索任务。"""

        with self._exploration_lock:
            self._exploration_stop_event.set()
            thread = self._exploration_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        with self._exploration_lock:
            current_request_id = self._exploration_state.current_request_id
            self._exploration_state = ExplorationState(
                current_request_id=current_request_id,
                status=ExplorationStatus.CANCELLED if current_request_id else ExplorationStatus.IDLE,
                strategy=self._exploration_state.strategy,
                message="探索任务已取消。" if current_request_id else None,
            )
            self._exploration_thread = None
        self.cancel_goal()
        return True

    def get_exploration_state(self) -> ExplorationState:
        """读取探索状态。"""

        with self._exploration_lock:
            return self._exploration_state.model_copy(deep=True)

    def get_status(self) -> Dict[str, object]:
        """返回当前数据面状态。"""

        bridge_status = self.bridge.get_status() if hasattr(self.bridge, "get_status") else {}
        if bridge_status:
            bridge_status = dict(bridge_status)
            raw_bridge_navigation_available = bool(bridge_status.get("navigation_available", False))
            bridge_status["bridge_navigation_available"] = raw_bridge_navigation_available
            bridge_status["navigation_available"] = self.is_navigation_available()
            bridge_status["navigation_availability_source"] = (
                "official_backend" if self._uses_official_backend() else "bridge"
            )

        return {
            "enabled": self.config.enabled,
            "started": self._started,
            "bridge": bridge_status,
            "localization_available": self.is_localization_available(),
            "map_available": self.is_map_available(),
            "navigation_available": self.is_navigation_available(),
            "exploration_available": self.is_exploration_available(),
            "named_map_runtime": self.get_named_map_runtime_status(),
            "startup_diagnostics": list(self._startup_diagnostics),
            "official_backend": {
                "enabled": self._uses_official_backend(),
                "sport_ready": self._official_sport_ready,
                "robot_state_ready": self._official_robot_state_ready,
                "obstacles_avoid_ready": self._official_obstacles_avoid_ready,
                "obstacle_avoidance": self.get_obstacle_avoidance_status(),
                "last_error": self._official_last_error,
                "service_states": dict(self._official_service_states),
            },
            "managed_processes": [
                {
                    "name": process.name,
                    "command": process.command,
                    "running": process.is_running(),
                }
                for process in self._processes
            ],
        }

    def _uses_official_backend(self) -> bool:
        return bool(getattr(self.config, "official", None) and self.config.official.enabled)

    def _wait_for_initial_readiness(self) -> None:
        """在启动阶段等待关键数据就绪，避免平台首轮刷新拿到空结果。"""

        if not self.config.startup_wait_for_ready:
            return
        if not self._wait_for_condition(
            condition=self.bridge.is_localization_available,
            timeout_sec=self.config.startup_localization_timeout_sec,
        ):
            message = (
                "Go2 数据面启动后在 "
                f"{self.config.startup_localization_timeout_sec:.1f} 秒内未收到定位数据。"
            )
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        if not self._should_wait_for_map_ready():
            return
        if self._wait_for_condition(
            condition=self.bridge.is_map_available,
            timeout_sec=self.config.startup_map_timeout_sec,
        ):
            return
        message = f"Go2 数据面启动后在 {self.config.startup_map_timeout_sec:.1f} 秒内未生成地图数据。"
        self._startup_diagnostics.append(message)
        LOGGER.warning(message)

    def _should_wait_for_map_ready(self) -> bool:
        """判断启动阶段是否应该等待地图就绪。"""

        if self._uses_official_backend() and not self.config.official.auto_start_services:
            return False
        if self.config.map_synthesis.local_map_enabled:
            return True
        if str(self.config.topics.occupancy_topic or "").strip():
            return True
        if str(self.config.topics.grid_map_topic or "").strip():
            return True
        if str(self.config.topics.cost_map_topic or "").strip():
            return True
        return False

    def _wait_for_condition(self, *, condition: Any, timeout_sec: float) -> bool:
        """轮询等待指定条件成立。"""

        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        poll_interval_sec = max(0.02, float(self.config.startup_poll_interval_sec))
        while time.monotonic() <= deadline:
            try:
                if bool(condition()):
                    return True
            except Exception:
                LOGGER.debug("等待 Go2 数据面条件时发生异常。", exc_info=True)
            time.sleep(poll_interval_sec)
        try:
            return bool(condition())
        except Exception:
            LOGGER.debug("读取 Go2 数据面最终条件状态失败。", exc_info=True)
            return False

    def _refresh_official_navigation_idle_state(self) -> None:
        """把官方导航空闲态更新为当前真实后端状态。"""

        if not self._uses_official_backend():
            return
        with self._navigation_lock:
            if self._official_navigation_goal is not None:
                return
            if self._official_sport_ready:
                self._official_navigation_state = NavigationState(
                    status=NavigationStatus.IDLE,
                    current_pose=self.get_current_pose(),
                    message="Go2 工程导航后端已就绪，等待目标。",
                    metadata={"backend": "official_grid_navigation"},
                )
            else:
                self._official_navigation_state = NavigationState(
                    status=NavigationStatus.IDLE,
                    current_pose=self.get_current_pose(),
                    message="Go2 工程导航后端未就绪。",
                    metadata={"backend": "official_grid_navigation"},
                )

    def _start_official_backend(self) -> None:
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
        except Exception:
            LOGGER.debug("初始化 Go2 官方 DDS 通道失败。", exc_info=True)

        timeout = float(self.config.official.client_timeout_sec)

        try:
            sport_client = modules["SportClient"]()
            sport_client.SetTimeout(timeout)
            sport_client.Init()
            self._official_sport_client = sport_client
            self._official_sport_ready = True
        except Exception as exc:
            self._official_last_error = f"初始化 Go2 官方运动客户端失败：{exc}"
            self._startup_diagnostics.append(self._official_last_error)
            LOGGER.warning(self._official_last_error)

        try:
            robot_state_client = modules["RobotStateClient"]()
            robot_state_client.SetTimeout(timeout)
            robot_state_client.Init()
            self._official_robot_state_client = robot_state_client
            self._official_robot_state_ready = True
        except Exception as exc:
            message = f"初始化 Go2 官方 robot_state 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        try:
            obstacles_avoid_client = modules["ObstaclesAvoidClient"]()
            obstacles_avoid_client.SetTimeout(timeout)
            obstacles_avoid_client.Init()
            self._official_obstacles_avoid_client = obstacles_avoid_client
            version_code, version = obstacles_avoid_client.GetServerApiVersion()
            if version_code == 0:
                self._official_obstacles_avoid_ready = True
                LOGGER.info("Go2 官方 obstacles_avoid 客户端已连接 server_api_version=%s", version)
                try:
                    self._official_obstacles_avoid_switch_state = self._set_obstacle_avoidance_switch(
                        self._obstacle_avoidance_enabled
                    )
                except Exception as exc:
                    LOGGER.warning("同步 Go2 官方 obstacles_avoid 初始开关状态失败: %s", exc)
            else:
                message = f"Go2 官方 obstacles_avoid 当前未就绪 code={version_code}"
                if version_code in {3102, 3104}:
                    LOGGER.info("%s，先继续启动其余 Go2 端侧能力。", message)
                else:
                    self._startup_diagnostics.append(message)
                    LOGGER.warning(message)
        except Exception as exc:
            message = f"初始化 Go2 官方 obstacles_avoid 客户端失败：{exc}"
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

        if self._official_robot_state_ready and self.config.official.auto_start_services:
            self._ensure_official_services_enabled(
                service_names=self.config.official.service_names,
                reason="startup_auto_start",
            )

    def _load_official_sdk_modules(self) -> Optional[Dict[str, object]]:
        modules = _load_go2_official_sdk_modules(self.config, include_dds_topics=False)
        if modules is not None:
            return modules
        message = "导入 Go2 官方 SDK 模块失败。"
        self._startup_diagnostics.append(message)
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
                    extra={
                        "event": "go2.service.best_effort",
                        "service_name": normalized,
                        "code": code,
                    },
                )
                return
            self._startup_diagnostics.append(message)
            LOGGER.warning(message)

    def _set_official_goal(self, goal: NavigationGoal) -> bool:
        if goal.target_pose is None or not self.is_navigation_available():
            return False

        self._stop_official_navigation(update_state=False)
        stop_event = threading.Event()
        with self._navigation_lock:
            self._official_goal_stop_event = stop_event
            self._official_navigation_goal = goal.model_copy(deep=True)
            self._official_goal_reached = False
            self._official_navigation_state = NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.ACCEPTED,
                current_pose=self.get_current_pose(),
                message="已提交 Go2 工程导航目标。",
                metadata={"backend": "official_grid_navigation"},
            )
            self._official_goal_thread = threading.Thread(
                target=self._run_official_navigation_goal,
                args=(goal.model_copy(deep=True), stop_event),
                name=f"go2_official_nav_{goal.goal_id}",
                daemon=True,
            )
            thread = self._official_goal_thread
        thread.start()
        return True

    def _stop_official_navigation(self, *, update_state: bool) -> bool:
        with self._navigation_lock:
            stop_event = self._official_goal_stop_event
            thread = self._official_goal_thread
            current_goal = self._official_navigation_goal
            stop_event.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._stop_official_motion()
        with self._navigation_lock:
            self._official_goal_thread = None
            self._official_navigation_goal = None
            if update_state:
                self._official_goal_reached = False
                self._official_navigation_state = NavigationState(
                    current_goal_id=current_goal.goal_id if current_goal is not None else None,
                    status=NavigationStatus.CANCELLED if current_goal is not None else NavigationStatus.IDLE,
                    current_pose=self.get_current_pose(),
                    message="Go2 工程导航目标已取消。" if current_goal is not None else None,
                    metadata={"backend": "official_grid_navigation"},
                )
        return current_goal is not None

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

    def _run_official_navigation_goal(self, goal: NavigationGoal, stop_event: threading.Event) -> None:
        try:
            current_pose = self.get_current_pose()
            if current_pose is None:
                self._set_official_navigation_failed(goal, "当前官方里程计尚未就绪。")
                return
            if goal.target_pose is None:
                self._set_official_navigation_failed(goal, "导航目标缺少目标位姿。")
                return
            if (
                goal.target_pose.frame_id
                and current_pose.frame_id
                and not frame_ids_semantically_equal(goal.target_pose.frame_id, current_pose.frame_id)
            ):
                self._set_official_navigation_failed(
                    goal,
                    f"目标坐标系 {goal.target_pose.frame_id} 与当前里程计坐标系 {current_pose.frame_id} 不一致。",
                )
                return

            session = NavigationSession(
                goal=goal,
                planner=self._grid_navigation_planner,
                planner_config=self.config.navigation,
                control_config=self.config.official,
            )
            with self._navigation_lock:
                self._official_navigation_state = NavigationState(
                    current_goal_id=goal.goal_id,
                    status=NavigationStatus.PLANNING,
                    current_pose=current_pose,
                    remaining_distance_m=self._compute_direct_goal_distance(
                        current_pose=current_pose,
                        target_pose=goal.target_pose,
                    ),
                    remaining_yaw_rad=self._sanitize_navigation_state_remaining_yaw(
                        self._compute_direct_goal_yaw_error(
                            current_pose=current_pose,
                            target_pose=goal.target_pose,
                        )
                    ),
                    goal_reached=False,
                    message="Go2 工程导航正在生成初始路径。",
                    metadata={"backend": "official_grid_navigation"},
                )

            while not stop_event.is_set():
                current_pose = self.get_current_pose()
                if current_pose is None:
                    time.sleep(self.config.official.control_loop_interval_sec)
                    continue
                tick_result = session.tick(
                    current_pose=current_pose,
                    occupancy_grid=self.get_occupancy_grid(),
                    cost_map=self.get_cost_map(),
                    now_monotonic=time.monotonic(),
                )
                if tick_result.status == NavigationStatus.SUCCEEDED:
                    self._stop_official_motion()
                    with self._navigation_lock:
                        self._official_goal_reached = True
                        self._official_navigation_state = NavigationState(
                            current_goal_id=goal.goal_id,
                            status=NavigationStatus.SUCCEEDED,
                            current_pose=current_pose,
                            remaining_distance_m=0.0,
                            remaining_yaw_rad=0.0,
                            goal_reached=True,
                            message=tick_result.message,
                            metadata=tick_result.metadata,
                        )
                        self._official_goal_thread = None
                        self._official_navigation_goal = None
                    return
                if tick_result.status == NavigationStatus.FAILED:
                    self._set_official_navigation_failed(goal, tick_result.message, metadata=tick_result.metadata)
                    return
                if tick_result.status == NavigationStatus.PLANNING:
                    self._stop_official_motion()
                    with self._navigation_lock:
                        self._official_navigation_state = NavigationState(
                            current_goal_id=goal.goal_id,
                            status=NavigationStatus.PLANNING,
                            current_pose=current_pose,
                            remaining_distance_m=tick_result.remaining_distance_m,
                            remaining_yaw_rad=self._sanitize_navigation_state_remaining_yaw(
                                tick_result.remaining_yaw_rad
                            ),
                            goal_reached=False,
                            message=tick_result.message,
                            metadata=tick_result.metadata,
                        )
                    time.sleep(self.config.official.control_loop_interval_sec)
                    continue

                code = self._send_official_motion_command(
                    float(tick_result.linear_x_mps),
                    0.0,
                    float(tick_result.angular_z_rps),
                )
                if code != 0:
                    self._set_official_navigation_failed(goal, f"Go2 官方运动命令执行失败 code={code}")
                    return
                with self._navigation_lock:
                    self._official_navigation_state = NavigationState(
                        current_goal_id=goal.goal_id,
                        status=NavigationStatus.RUNNING,
                        current_pose=current_pose,
                        remaining_distance_m=tick_result.remaining_distance_m,
                        remaining_yaw_rad=self._sanitize_navigation_state_remaining_yaw(
                            tick_result.remaining_yaw_rad
                        ),
                        goal_reached=False,
                        message=tick_result.message,
                        metadata=tick_result.metadata,
                    )
                time.sleep(self.config.official.control_loop_interval_sec)

            self._stop_official_motion()
        except Exception:
            LOGGER.exception("执行 Go2 官方运动导航失败。")
            self._set_official_navigation_failed(goal, "执行 Go2 官方运动导航时发生异常。")
        finally:
            with self._navigation_lock:
                if self._official_goal_thread is threading.current_thread():
                    self._official_goal_thread = None

    def _set_official_navigation_failed(
        self,
        goal: NavigationGoal,
        message: str,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> None:
        self._stop_official_motion()
        failure_metadata = {"backend": "official_grid_navigation"}
        if metadata:
            failure_metadata.update(dict(metadata))
        with self._navigation_lock:
            self._official_goal_reached = False
            self._official_navigation_goal = None
            self._official_navigation_state = NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.FAILED,
                current_pose=self.get_current_pose(),
                message=message,
                metadata=failure_metadata,
            )
            self._official_last_error = message

    def _compute_direct_goal_distance(self, *, current_pose: Pose, target_pose: Pose) -> float:
        dx = float(target_pose.position.x - current_pose.position.x)
        dy = float(target_pose.position.y - current_pose.position.y)
        return math.hypot(dx, dy)

    def _compute_direct_goal_yaw_error(self, *, current_pose: Pose, target_pose: Pose) -> float:
        current_yaw = self._yaw_from_quaternion(current_pose.orientation)
        target_yaw = self._yaw_from_quaternion(target_pose.orientation)
        return self._normalize_angle(target_yaw - current_yaw)

    def _sanitize_navigation_state_remaining_yaw(self, yaw_error_rad: Optional[float]) -> Optional[float]:
        if yaw_error_rad is None:
            return None
        return abs(float(yaw_error_rad))

    def _compute_official_goal_errors(self, *, current_pose: Pose, target_pose: Pose) -> Tuple[float, float, float]:
        dx = float(target_pose.position.x - current_pose.position.x)
        dy = float(target_pose.position.y - current_pose.position.y)
        distance = math.hypot(dx, dy)
        current_yaw = self._yaw_from_quaternion(current_pose.orientation)
        desired_heading = math.atan2(dy, dx) if distance > 1e-6 else current_yaw
        target_yaw = self._yaw_from_quaternion(target_pose.orientation)
        heading_error = self._normalize_angle(desired_heading - current_yaw)
        goal_yaw_error = self._normalize_angle(target_yaw - current_yaw)
        return distance, heading_error, goal_yaw_error

    def _yaw_from_quaternion(self, rotation: Quaternion) -> float:
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _build_exploration_unavailable_message(self) -> str:
        with self._exploration_lock:
            waiting_for = str((self._exploration_state.metadata or {}).get("waiting_for") or "").strip()
            current_message = str(self._exploration_state.message or "").strip()
        if waiting_for:
            return waiting_for
        if current_message and "等待" in current_message:
            return current_message
        if self._startup_diagnostics:
            return self._startup_diagnostics[-1]
        if not self.is_localization_available():
            return "当前定位尚未就绪，无法生成探索候选点。"
        if not self.is_map_available():
            return "当前地图尚未就绪，无法生成探索候选点。"
        return "当前地图中未找到可用探索候选点。"

    def _run_exploration(self, request: ExploreAreaRequest) -> None:
        try:
            with self._exploration_lock:
                self._exploration_state = ExplorationState(
                    current_request_id=request.request_id,
                    status=ExplorationStatus.RUNNING,
                    strategy=request.strategy,
                    message="探索任务运行中。",
                )
            completed = 0
            attempted_poses: List[Pose] = []
            frontier_count = 0
            iteration = 0
            known_cells = self._count_known_map_cells()
            no_gain_rounds = 0
            exploration_started_at = time.monotonic()
            final_status = ExplorationStatus.SUCCEEDED
            final_message = "探索任务完成。"

            while not self._exploration_stop_event.is_set():
                if request.max_duration_sec is not None and (
                    time.monotonic() - exploration_started_at
                ) >= float(request.max_duration_sec):
                    final_status = ExplorationStatus.FAILED
                    final_message = f"探索超时，已运行 {float(request.max_duration_sec):.1f} 秒。"
                    break

                if not self._wait_for_exploration_prerequisites(
                    request=request,
                    completed=completed,
                    iteration=iteration,
                    attempted_poses=attempted_poses,
                    exploration_started_at=exploration_started_at,
                ):
                    final_status = ExplorationStatus.FAILED
                    final_message = self._build_exploration_unavailable_message()
                    break

                candidate_poses = self._build_exploration_candidates(
                    request,
                    attempted_poses=attempted_poses,
                )
                frontier_count = len(candidate_poses)
                if not candidate_poses:
                    if completed > 0:
                        final_status = ExplorationStatus.SUCCEEDED
                        final_message = "探索完成，当前无更多可达前沿。"
                    else:
                        final_status = ExplorationStatus.FAILED
                        final_message = self._build_exploration_unavailable_message()
                    break

                iteration += 1
                pose = candidate_poses[0]
                if self._exploration_stop_event.is_set():
                    break
                goal = NavigationGoal(
                    goal_id=f"{request.request_id}_candidate_{iteration}",
                    target_pose=pose,
                    target_name=request.target_name,
                    metadata={"exploration_request_id": request.request_id, "candidate_index": iteration},
                )
                if not self.set_goal(goal):
                    attempted_poses.append(pose)
                    with self._exploration_lock:
                        self._exploration_state = ExplorationState(
                            current_request_id=request.request_id,
                            status=ExplorationStatus.RUNNING,
                            strategy=request.strategy,
                            covered_ratio=float(completed) / float(max(1, iteration)),
                            frontier_count=frontier_count,
                            message="探索候选已生成，但导航后端暂未接受目标，准备重试其他前沿。",
                            metadata={
                                "known_cells": known_cells,
                                "attempted_frontiers": len(attempted_poses),
                            },
                        )
                    time.sleep(max(0.05, float(self.config.exploration.status_poll_interval_sec)))
                    continue
                success = self._wait_navigation_goal(goal_timeout_sec=self.config.exploration.goal_timeout_sec)
                attempted_poses.append(pose)
                if success:
                    completed += 1

                next_known_cells = self._count_known_map_cells()
                info_gain_cells = max(0, next_known_cells - known_cells)
                known_cells = max(known_cells, next_known_cells)
                if self.get_occupancy_grid() is not None:
                    if info_gain_cells <= int(self.config.exploration.frontier_min_information_gain_cells):
                        no_gain_rounds += 1
                    else:
                        no_gain_rounds = 0
                    if no_gain_rounds >= int(self.config.exploration.frontier_max_no_gain_rounds):
                        final_status = ExplorationStatus.SUCCEEDED
                        final_message = "探索停止：连续多轮未获得足够的新地图信息。"
                        with self._exploration_lock:
                            self._exploration_state = ExplorationState(
                                current_request_id=request.request_id,
                                status=ExplorationStatus.RUNNING,
                                strategy=request.strategy,
                                covered_ratio=float(completed) / float(max(1, iteration)),
                                frontier_count=frontier_count,
                                message=(
                                    f"探索进行中，已完成 {completed} 次到达，"
                                    f"本轮新增已知栅格 {info_gain_cells}。"
                                ),
                                metadata={
                                    "known_cells": known_cells,
                                    "info_gain_cells": info_gain_cells,
                                    "attempted_frontiers": len(attempted_poses),
                                },
                            )
                        break

                with self._exploration_lock:
                    self._exploration_state = ExplorationState(
                        current_request_id=request.request_id,
                        status=ExplorationStatus.RUNNING,
                        strategy=request.strategy,
                        covered_ratio=float(completed) / float(max(1, iteration)),
                        frontier_count=frontier_count,
                        message=(
                            f"探索进行中，已完成 {completed} 次到达，"
                            f"剩余候选 {frontier_count}，本轮新增已知栅格 {info_gain_cells}。"
                        ),
                        metadata={
                            "known_cells": known_cells,
                            "info_gain_cells": info_gain_cells,
                            "attempted_frontiers": len(attempted_poses),
                        },
                    )
            with self._exploration_lock:
                if self._exploration_stop_event.is_set():
                    self._exploration_state = ExplorationState(
                        current_request_id=request.request_id,
                        status=ExplorationStatus.CANCELLED,
                        strategy=request.strategy,
                        covered_ratio=float(completed) / float(max(1, iteration)),
                        frontier_count=frontier_count,
                        message="探索任务已取消。",
                    )
                else:
                    final_metadata = dict(self._exploration_state.metadata or {})
                    final_metadata.update(
                        {
                            "known_cells": known_cells,
                            "attempted_frontiers": len(attempted_poses),
                            "completed_frontiers": completed,
                        }
                    )
                    self._exploration_state = ExplorationState(
                        current_request_id=request.request_id,
                        status=final_status,
                        strategy=request.strategy,
                        covered_ratio=float(completed) / float(max(1, iteration)),
                        frontier_count=0,
                        message=final_message,
                        metadata=final_metadata,
                    )
        except Exception:
            LOGGER.exception("执行 Go2 探索任务失败。")
            with self._exploration_lock:
                self._exploration_state = ExplorationState(
                    current_request_id=request.request_id,
                    status=ExplorationStatus.FAILED,
                    strategy=request.strategy,
                    message="探索任务执行异常。",
                )
        finally:
            with self._exploration_lock:
                self._exploration_thread = None

    def _build_exploration_candidates(
        self,
        request: ExploreAreaRequest,
        *,
        attempted_poses: Optional[List[Pose]] = None,
    ) -> List[Pose]:
        current_pose = self.get_current_pose()
        center_pose = self._resolve_exploration_center_pose(request, current_pose=current_pose)
        occupancy_grid = self.get_occupancy_grid()
        if center_pose is None or current_pose is None or occupancy_grid is None:
            return []

        frontier_search_radius = self._resolve_frontier_search_radius(request)
        cost_map = self.get_cost_map()
        attempted_items = tuple(attempted_poses or [])

        if (
            self._should_prefer_frontier_candidates(strategy=request.strategy)
            and self.config.exploration.frontier_enabled
        ):
            frontier_candidates = self._frontier_explorer.select_candidates(
                current_pose=current_pose,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                planner=self._grid_navigation_planner,
                center_pose=center_pose,
                radius_m=frontier_search_radius,
                attempted_poses=attempted_items,
            )
            if frontier_candidates:
                return [candidate.pose for candidate in frontier_candidates]
        return []

    def _wait_for_exploration_prerequisites(
        self,
        *,
        request: ExploreAreaRequest,
        completed: int,
        iteration: int,
        attempted_poses: Sequence[Pose],
        exploration_started_at: float,
    ) -> bool:
        """等待探索前置条件就绪，避免建图冷启动时过早选点。"""

        wait_timeout_sec = max(0.1, float(self.config.exploration.prerequisite_wait_timeout_sec))
        poll_interval_sec = max(0.05, float(self.config.exploration.prerequisite_poll_interval_sec))
        deadline_monotonic = time.monotonic() + wait_timeout_sec
        if request.max_duration_sec is not None:
            remaining_duration_sec = float(request.max_duration_sec) - (time.monotonic() - exploration_started_at)
            if remaining_duration_sec <= 0.0:
                return False
            deadline_monotonic = min(deadline_monotonic, time.monotonic() + remaining_duration_sec)

        while not self._exploration_stop_event.is_set():
            current_pose = self.get_current_pose()
            occupancy_grid = self.get_occupancy_grid()
            waiting_metadata: Dict[str, object] = {
                "attempted_frontiers": len(attempted_poses),
            }
            gap_message = self._describe_exploration_prerequisite_gap(
                current_pose=current_pose,
                occupancy_grid=occupancy_grid,
            )
            if gap_message is None and completed <= 0 and current_pose is not None and occupancy_grid is not None:
                gap_message, frontier_metadata = self._describe_exploration_frontier_gap(
                    request=request,
                    current_pose=current_pose,
                    occupancy_grid=occupancy_grid,
                )
                waiting_metadata.update(frontier_metadata)
            if gap_message is None:
                return True

            waiting_metadata["waiting_for"] = gap_message
            with self._exploration_lock:
                self._exploration_state = ExplorationState(
                    current_request_id=request.request_id,
                    status=ExplorationStatus.RUNNING,
                    strategy=request.strategy,
                    covered_ratio=float(completed) / float(max(1, iteration)),
                    frontier_count=None,
                    message=gap_message,
                    metadata=waiting_metadata,
                )
            if time.monotonic() >= deadline_monotonic:
                return False
            time.sleep(poll_interval_sec)
        return False

    def _describe_exploration_prerequisite_gap(
        self,
        *,
        current_pose: Optional[Pose],
        occupancy_grid: Optional[OccupancyGrid],
    ) -> Optional[str]:
        """描述当前探索起步仍缺失的前置条件。"""

        if current_pose is None:
            return "当前定位尚未就绪，等待探索起步条件。"
        if occupancy_grid is None:
            return "当前探索地图尚未就绪，等待首帧地图。"
        if (
            occupancy_grid.frame_id
            and current_pose.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, current_pose.frame_id)
        ):
            return "当前地图与定位坐标系尚未对齐，等待探索起步条件。"
        return None

    def _describe_exploration_frontier_gap(
        self,
        *,
        request: ExploreAreaRequest,
        current_pose: Pose,
        occupancy_grid: OccupancyGrid,
    ) -> Tuple[Optional[str], Dict[str, object]]:
        """判断当前地图是否已形成与机器人可达区域连通的前沿。"""

        if not self._should_prefer_frontier_candidates(strategy=request.strategy):
            return None, {}
        if not bool(self.config.exploration.frontier_enabled):
            return None, {}

        occupancy = np.asarray(occupancy_grid.data, dtype=np.int32).reshape(occupancy_grid.height, occupancy_grid.width)
        free_mask = occupancy == 0
        unknown_mask = occupancy < 0
        if not np.any(free_mask) or not np.any(unknown_mask):
            return "当前探索地图仍未形成有效前沿，等待地图继续收敛。", {
                "frontier_cells": 0,
                "frontier_clusters_total": 0,
                "frontier_clusters_ge_min": 0,
                "reachable_frontier_clusters": 0,
            }

        cost_map = self.get_cost_map()
        cost_map_layout_mismatch = (
            cost_map is not None
            and not self._grid_navigation_planner._same_grid_layout(occupancy_grid, cost_map)
        )
        cost_map_frame_mismatch = (
            cost_map is not None
            and occupancy_grid.frame_id
            and cost_map.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, cost_map.frame_id)
        )
        if cost_map_layout_mismatch or cost_map_frame_mismatch:
            cost_map = None
        frontier_mask = self._frontier_explorer._build_frontier_mask(
            free_mask=free_mask,
            unknown_mask=unknown_mask,
            cost_map=cost_map,
        )
        frontier_cell_count = int(np.count_nonzero(frontier_mask))
        if frontier_cell_count == 0:
            return "当前探索地图仍未形成有效前沿，等待地图继续收敛。", {
                "frontier_cells": 0,
                "frontier_clusters_total": 0,
                "frontier_clusters_ge_min": 0,
                "reachable_frontier_clusters": 0,
            }

        center_pose = self._resolve_exploration_center_pose(request, current_pose=current_pose)
        if center_pose is None:
            return None, {}

        max_radius_m = self._resolve_frontier_search_radius(request)
        normalized_radius_m = None if max_radius_m is None else max(0.1, float(max_radius_m))
        filtered_clusters: List[List[Tuple[int, int]]] = []
        all_clusters = self._frontier_explorer._cluster_frontiers(frontier_mask)
        for cluster in all_clusters:
            if len(cluster) < int(self.config.exploration.frontier_min_cluster_cells):
                continue
            if normalized_radius_m is not None:
                cluster_points_world = [
                    (
                        float(occupancy_grid.origin.position.x) + (float(col) + 0.5) * float(occupancy_grid.resolution_m),
                        float(occupancy_grid.origin.position.y) + (float(row) + 0.5) * float(occupancy_grid.resolution_m),
                    )
                    for row, col in cluster
                ]
                centroid_x = sum(point[0] for point in cluster_points_world) / float(len(cluster_points_world))
                centroid_y = sum(point[1] for point in cluster_points_world) / float(len(cluster_points_world))
                center_distance_m = math.hypot(
                    centroid_x - center_pose.position.x,
                    centroid_y - center_pose.position.y,
                )
                if center_distance_m > normalized_radius_m:
                    continue
            filtered_clusters.append(cluster)

        metadata: Dict[str, object] = {
            "frontier_cells": frontier_cell_count,
            "frontier_clusters_total": len(all_clusters),
            "frontier_clusters_ge_min": len(filtered_clusters),
            "reachable_frontier_clusters": 0,
        }
        if not filtered_clusters:
            return "当前探索地图仍未形成满足阈值的前沿，等待地图继续收敛。", metadata

        planning_map = self._grid_navigation_planner._build_planning_map(
            current_frame_id=current_pose.frame_id,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        if planning_map is None:
            return None, metadata

        start_cell = planning_map.world_to_cell(current_pose.position.x, current_pose.position.y)
        if start_cell is None:
            return "当前定位尚未落入可规划区域，等待地图继续收敛。", metadata
        start_cell = planning_map.nearest_traversable(
            start_cell,
            max_radius_cells=self._grid_navigation_planner._search_radius_cells(planning_map),
        )
        if start_cell is None:
            return "当前机器人附近暂无可规划区域，等待地图继续收敛。", metadata

        reachable = np.zeros((planning_map.height, planning_map.width), dtype=bool)
        pending_cells: Deque[Tuple[int, int]] = deque([start_cell])
        reachable[start_cell[0], start_cell[1]] = True
        while pending_cells:
            row, col = pending_cells.popleft()
            for row_offset, col_offset, _step_cost in self._grid_navigation_planner._NEIGHBORS:
                next_row = row + row_offset
                next_col = col + col_offset
                if (
                    next_row < 0
                    or next_row >= planning_map.height
                    or next_col < 0
                    or next_col >= planning_map.width
                ):
                    continue
                if reachable[next_row, next_col] or bool(planning_map.blocked[next_row, next_col]):
                    continue
                reachable[next_row, next_col] = True
                pending_cells.append((next_row, next_col))

        reachable_cluster_count = 0
        search_radius_cells = self._grid_navigation_planner._search_radius_cells(planning_map)
        for cluster in filtered_clusters:
            cluster_reachable = False
            for row, col in cluster:
                adjusted_cell = None if bool(planning_map.blocked[row, col]) else (row, col)
                if adjusted_cell is None:
                    adjusted_cell = planning_map.nearest_traversable(
                        (row, col),
                        max_radius_cells=search_radius_cells,
                    )
                if adjusted_cell is None:
                    continue
                if bool(reachable[adjusted_cell[0], adjusted_cell[1]]):
                    cluster_reachable = True
                    break
            if cluster_reachable:
                reachable_cluster_count += 1

        metadata["reachable_cells"] = int(np.count_nonzero(reachable))
        metadata["reachable_frontier_clusters"] = reachable_cluster_count
        if reachable_cluster_count == 0:
            return "当前地图前沿已生成，但与机器人可达区域仍断开，等待地图继续收敛。", metadata
        return None, metadata

    def _resolve_frontier_search_radius(self, request: ExploreAreaRequest) -> Optional[float]:
        """解析 frontier 搜索半径。

        无显式半径、也无显式探索中心时，默认做全局 frontier 搜索，
        避免把探索错误地限制在机器人附近的固定圆域。
        """

        if request.radius_m is not None:
            return max(0.1, float(request.radius_m))
        if request.center_pose is not None or bool(str(request.target_name or "").strip()):
            return max(0.1, float(self.config.exploration.sample_radius_m))
        return None

    def _should_prefer_frontier_candidates(self, *, strategy: str) -> bool:
        """判断当前探索策略是否应优先尝试前沿候选。"""

        normalized = str(strategy or "").strip().lower()
        return normalized in {"", "frontier", "coverage", "auto"}

    def _resolve_exploration_center_pose(
        self,
        request: ExploreAreaRequest,
        *,
        current_pose: Optional[Pose],
    ) -> Optional[Pose]:
        center_pose = request.center_pose
        if center_pose is None and request.target_name:
            semantic_map = self.get_semantic_map()
            if semantic_map is not None:
                normalized = request.target_name.strip().lower()
                for region in semantic_map.regions:
                    aliases = [str(region.attributes.get("alias", ""))]
                    aliases.extend(str(item) for item in region.attributes.get("aliases", []) if item)
                    names = {region.label.strip().lower(), region.region_id.strip().lower()}
                    names.update(item.strip().lower() for item in aliases if item.strip())
                    if normalized in names and region.centroid is not None:
                        center_pose = region.centroid
                        break
        if center_pose is None:
            center_pose = current_pose
        return center_pose

    def _count_known_map_cells(self) -> int:
        return self._frontier_explorer.count_known_cells(self.get_occupancy_grid())

    def _wait_navigation_goal(self, *, goal_timeout_sec: float) -> bool:
        deadline = time.time() + max(0.1, goal_timeout_sec)
        while time.time() < deadline:
            if self._exploration_stop_event.is_set():
                self.cancel_goal()
                return False
            state = self.get_navigation_state()
            if self.is_goal_reached() or state.status == NavigationStatus.SUCCEEDED:
                return True
            if state.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
                return False
            time.sleep(max(0.05, self.config.exploration.status_poll_interval_sec))
        self.cancel_goal()
        return False
