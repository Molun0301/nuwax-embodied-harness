from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotDefaults, RobotManifest
from drivers.puppypi.capabilities import PUPPYPI_CAPABILITY_MATRIX

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_puppypi_defaults(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotDefaults:
    del config
    return RobotDefaults(
        frame_ids={
            "base": "world/puppypi/base",
            "camera_front": "world/puppypi/camera_front",
        },
        topics={},
        parameters={"robot_id": "puppypi", "robot_model": "puppypi"},
    )


def build_puppypi_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    del config
    return RobotManifest(
        robot_name="puppypi",
        robot_model="puppypi",
        entrypoint="drivers/puppypi/assembly.py:create_puppypi_assembly",
        description="幻尔 PuppyPi 薄驱动：运动经容器 motion_proxy，视频经容器 video_proxy 推流。",
        capability_matrix=PUPPYPI_CAPABILITY_MATRIX,
        required_components=(
            ComponentBinding(
                name="puppypi_motion_proxy",
                path="deploy/puppypi/platform_bridge",
                description="容器侧 motion_proxy / video_proxy。",
            ),
        ),
    )
