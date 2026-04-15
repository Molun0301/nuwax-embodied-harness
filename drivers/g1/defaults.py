from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import RobotDefaults

if TYPE_CHECKING:
    from settings import NuwaxRobotBridgeConfig


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
