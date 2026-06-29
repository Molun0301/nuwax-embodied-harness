from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import RobotDefaults

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_g1_defaults(config: "NuwaxRobotBridgeConfig", iface: Optional[str] = None) -> RobotDefaults:
    """生成 G1 入口骨架默认配置。"""

    return RobotDefaults(
        frame_ids={
            "map": "world/g1/map",
            "base": "world/g1/base",
        },
        topics={
            "robot_state": "robot.g1.state",
        },
        parameters={
            "robot_id": "g1",
            "robot_model": "unitree_g1",
            "dds_iface": iface or config.dds.iface,
        },
    )


# G1 帧 ID 字典（被 drivers/g1/settings.py 引用，7838badb 提交遗漏补全）
# 与 ralplan Round 3 收敛的 G1 frame 命名一致 + 现有 env 默认值对齐
G1_DEFAULT_FRAME_IDS = {
    "map": "world/g1/map",
    "base": "world/g1/base",
    "imu": "world/g1/imu",
    "camera_front": "world/g1/camera_front",
}
