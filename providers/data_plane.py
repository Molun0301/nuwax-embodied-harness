"""平台数据面与机器人装配协议。

历史上位于 drivers/common（drivers/common/data_plane.py 与 assembly.py），实为
「面向服务层的最小能力接口」契约（全部为 typing.Protocol，无任何硬件代码）。下沉至
providers 以解开 services ⇄ drivers 包级循环（services 改依赖此处 Protocol，drivers
反向 re-export 并由 drivers 提供具体实现）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Protocol, Tuple, runtime_checkable

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
class MotionCommandDataPlane(Protocol):
    """运动命令数据面协议。"""

    def can_accept_motion_command(self) -> bool:
        """返回当前是否可接受直接运动命令。"""

    def send_motion_command(self, vx: float, vy: float, vyaw: float) -> int:
        """发送底层运动命令。"""

    def stop_motion_command(self) -> None:
        """停止底层运动命令。"""


@runtime_checkable
class ObstacleAvoidanceControlDataPlane(Protocol):
    """实时避障开关协议。"""

    def is_obstacle_avoidance_control_available(self) -> bool:
        """返回避障开关是否可用。"""

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, Any]:
        """切换实时避障状态。"""


@runtime_checkable
class ManagedRobotDeviceDataPlane(
    DataPlaneLifecycle,
    RobotStateSensorDataPlane,
    DataPlaneStatusReporter,
    Protocol,
):
    """机器人入口默认保留的窄数据面协议。

    仅包含生命周期、输入流辅助状态与 IMU 观测；运动命令与安全
    开关不属于默认设备数据面，也不承载平台 map/nav/exploration 主链。
    """


@runtime_checkable
class RobotAssemblyProtocol(Protocol):
    """机器人装配入口的服务层最小契约。

    仅声明 services 层（robot_state_service、geometry runtime）实际消费的成员，
    使服务层依赖此 Protocol 而非具体的 drivers.common.assembly.RobotAssemblyBase，
    解开 services → drivers 反向依赖。drivers 侧 RobotAssemblyBase 自然满足此结构。
    """

    @property
    def manifest(self) -> Any:
        """机器人清单（robot_name/robot_model 等元数据）。"""

    @property
    def defaults(self) -> Any:
        """机器人默认值（frame_ids 等）。"""

    @property
    def providers(self) -> Any:
        """机器人提供器门面（state/localization/calibration 等）。"""

    def get_status(self) -> Any:
        """返回当前装配状态。"""

    def get_adapter_health_statuses(self) -> Tuple[Any, ...]:
        """汇总全部适配器健康状态。"""


__all__ = [
    "DataPlaneLifecycle",
    "DataPlaneStatusReporter",
    "RobotStateSensorDataPlane",
    "LocalizationDataPlane",
    "MappingDataPlane",
    "NavigationDataPlane",
    "ExplorationDataPlane",
    "MotionCommandDataPlane",
    "ObstacleAvoidanceControlDataPlane",
    "ManagedRobotDeviceDataPlane",
    "RobotAssemblyProtocol",
]
