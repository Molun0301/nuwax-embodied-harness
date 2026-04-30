from __future__ import annotations

from typing import Callable, List, Optional, Protocol, runtime_checkable

from contracts.robot_state import IMUState, JointState, RobotState
from providers.base import BaseProvider


@runtime_checkable
class StateProvider(BaseProvider, Protocol):
    """机器人状态读取接口。"""

    def get_robot_state(self) -> RobotState:
        """返回当前机器人状态快照。"""

    def get_joint_states(self) -> List[JointState]:
        """返回全部关节状态。"""

    def get_imu_state(self) -> Optional[IMUState]:
        """返回 IMU 状态。"""



@runtime_checkable
class StreamingStateProvider(BaseProvider, Protocol):
    """机器人状态流式订阅接口。"""

    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> Optional[int]:
        """订阅机器人状态源头帧。"""

    def unsubscribe_robot_state(self, subscription_id: Optional[int]) -> None:
        """取消机器人状态订阅。"""
