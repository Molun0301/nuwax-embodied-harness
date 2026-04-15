from __future__ import annotations

from typing import Any, Dict, Optional, Protocol, runtime_checkable

from contracts.geometry import FrameTree, Pose
from contracts.maps import CostMap, OccupancyGrid, SemanticMap
from contracts.navigation import ExplorationState, ExploreAreaRequest, NavigationGoal, NavigationState
from contracts.robot_state import IMUState


@runtime_checkable
class DataPlaneLifecycle(Protocol):
    """数据面生命周期协议。"""

    def start(self) -> None:
        """启动数据面。"""

    def stop(self) -> None:
        """停止数据面。"""


@runtime_checkable
class DataPlaneStatusReporter(Protocol):
    """数据面状态报告协议。"""

    def get_status(self) -> Dict[str, Any]:
        """返回当前数据面状态。"""


@runtime_checkable
class RobotStateSensorDataPlane(Protocol):
    """机器人状态传感协议。"""

    def get_imu_state(self) -> Optional[IMUState]:
        """返回当前 IMU（惯性测量单元）状态。"""


@runtime_checkable
class LocalizationDataPlane(Protocol):
    """定位数据面协议。"""

    def is_localization_available(self) -> bool:
        """返回定位能力是否可用。"""

    def get_current_pose(self) -> Optional[Pose]:
        """返回当前位姿。"""

    def get_frame_tree(self) -> Optional[FrameTree]:
        """返回当前坐标系树。"""


@runtime_checkable
class MappingDataPlane(Protocol):
    """地图数据面协议。"""

    def is_map_available(self) -> bool:
        """返回地图能力是否可用。"""

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """返回占据栅格地图。"""

    def get_cost_map(self) -> Optional[CostMap]:
        """返回代价地图。"""

    def get_semantic_map(self) -> Optional[SemanticMap]:
        """返回语义地图。"""


@runtime_checkable
class NavigationDataPlane(Protocol):
    """导航数据面协议。"""

    def is_navigation_available(self) -> bool:
        """返回导航能力是否可用。"""

    def set_goal(self, goal: NavigationGoal) -> bool:
        """提交导航目标。"""

    def cancel_goal(self) -> bool:
        """取消导航目标。"""

    def get_navigation_state(self) -> NavigationState:
        """返回导航状态。"""

    def is_goal_reached(self) -> bool:
        """返回当前目标是否到达。"""


@runtime_checkable
class ExplorationDataPlane(Protocol):
    """探索数据面协议。"""

    def is_exploration_available(self) -> bool:
        """返回探索能力是否可用。"""

    def start_exploration(self, request: ExploreAreaRequest) -> bool:
        """启动探索任务。"""

    def stop_exploration(self) -> bool:
        """停止探索任务。"""

    def get_exploration_state(self) -> ExplorationState:
        """返回探索状态。"""


@runtime_checkable
class RobotDataPlane(
    DataPlaneLifecycle,
    LocalizationDataPlane,
    MappingDataPlane,
    NavigationDataPlane,
    ExplorationDataPlane,
    Protocol,
):
    """平台默认依赖的通用机器人数据面协议。"""


@runtime_checkable
class ManagedRobotDataPlane(
    RobotDataPlane,
    RobotStateSensorDataPlane,
    DataPlaneStatusReporter,
    Protocol,
):
    """当前机器人入口默认托管的数据面协议。"""


@runtime_checkable
class MotionCommandDataPlane(Protocol):
    """运动命令数据面协议。"""

    def can_accept_motion_command(self) -> bool:
        """返回当前是否可接受直接运动命令。"""

    def send_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        """发送底层运动命令。"""

    def stop_motion_command(self) -> None:
        """停止底层运动命令。"""


@runtime_checkable
class MappingRuntimeControlDataPlane(Protocol):
    """建图运行态开关协议。"""

    def ensure_mapping_runtime_enabled(self, *, reason: str = "") -> Dict[str, Any]:
        """按需启用建图运行态。"""

    def disable_mapping_runtime(self, *, reason: str = "") -> Dict[str, Any]:
        """回收建图运行态。"""


@runtime_checkable
class NamedMapRuntimeStatusDataPlane(Protocol):
    """命名地图运行态状态协议。"""

    def get_named_map_runtime_status(self) -> Dict[str, Any]:
        """返回命名地图兼容运行态状态。"""


@runtime_checkable
class LoadedMapNameDataPlane(Protocol):
    """当前加载地图名称协议。"""

    def get_loaded_map_name(self) -> Optional[str]:
        """返回当前加载的地图名称。"""


@runtime_checkable
class NamedMapCompatibilityDataPlane(
    NamedMapRuntimeStatusDataPlane,
    LoadedMapNameDataPlane,
    Protocol,
):
    """命名地图兼容入口协议。"""

    def load_named_map(
        self,
        map_name: str,
        *,
        reason: str = "",
        allow_missing: bool = False,
    ) -> Dict[str, Any]:
        """加载命名地图兼容运行态。"""

    def save_named_map(self, map_name: str, *, reason: str = "") -> Dict[str, Any]:
        """保存命名地图兼容归档。"""


@runtime_checkable
class ObstacleAvoidanceControlDataPlane(Protocol):
    """实时避障开关协议。"""

    def is_obstacle_avoidance_control_available(self) -> bool:
        """返回避障开关是否可用。"""

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, Any]:
        """切换实时避障状态。"""
