from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from contracts.geometry import Twist
from contracts.motion_feedback import MotionExecutionFeedback
from providers.base import BaseProvider


@runtime_checkable
class MotionControl(BaseProvider, Protocol):
    """运动控制接口。"""

    def send_twist(self, twist: Twist) -> None:
        """发送速度控制指令。"""

    def stop_motion(self) -> None:
        """停止当前运动。"""



@runtime_checkable
class MotionExecutionFeedbackProvider(Protocol):
    """只返回执行器实际采样；缺失或断流保留未知。"""

    def get_motion_execution_feedback(self) -> Optional[MotionExecutionFeedback]:
        """立即返回已采样的MotionExecutionFeedback（运动执行反馈）或None，不等待新设备帧。"""


@runtime_checkable
class LocomotionModeControl(Protocol):
    """由原运动守卫在停车后调用的显式步态接口。"""

    def set_locomotion_mode(self, mode_id: str) -> None:
        """有界提交命名模式请求；不等待模式完成，完成由守卫核对实际反馈。"""
