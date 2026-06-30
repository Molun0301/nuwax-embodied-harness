"""G1 端侧控制子包。

封装宇树 G1 官方 SDK 的高层控制客户端：
- LocoController: 封装 unitree_sdk2py.g1.loco.LocoClient（damp/squat/stand/Move 等）
- ArmController: 封装 unitree_sdk2py.g1.arm.G1ArmActionClient（挥手/握手等 ArmAction）
"""

from __future__ import annotations

from drivers.g1.control.arm_controller import ArmController
from drivers.g1.control.loco_controller import LocoController

__all__ = [
    "ArmController",
    "LocoController",
]
