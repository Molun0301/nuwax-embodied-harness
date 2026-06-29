"""G1 端侧控制子包。

封装宇树 G1 官方 SDK 的高层控制客户端：
- LocoController: 封装 unitree_sdk2py.g1.loco.LocoClient（damp/squat/stand/wave/shake/Move 等）
- ArmController: 封装 unitree_sdk2py.g1.arm.G1ArmActionClient（手臂 Action 透传）
"""

from __future__ import annotations

from drivers.g1.control.arm_controller import ArmController
from drivers.g1.control.loco_controller import LocoController

__all__ = [
    "ArmController",
    "LocoController",
]
