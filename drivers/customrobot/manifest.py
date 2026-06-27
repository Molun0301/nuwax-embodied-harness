from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotDefaults, RobotManifest, RobotProfile
from drivers.customrobot.capabilities import CUSTOMROBOT_CAPABILITY_MATRIX

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_customrobot_defaults(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotDefaults:
    """构造自定义机器人默认配置骨架。"""

    del config
    return RobotDefaults(frame_ids={}, topics={}, parameters={"robot_id": "customrobot", "robot_model": "customrobot"})


def build_customrobot_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    """构造自定义机器人入口骨架清单。"""

    del config
    return RobotManifest(
        robot_name="customrobot",
        robot_model="customrobot",
        entrypoint="drivers/customrobot/assembly.py:create_customrobot_assembly",
        description="用户自定义机器人入口骨架，供后续按 sensor 或厂商 SDK 装配。",
        capability_matrix=CUSTOMROBOT_CAPABILITY_MATRIX,
        required_components=(
            ComponentBinding(
                name="custom_robot_driver",
                path="drivers/customrobot",
                description="用户自定义机器人实现根目录。",
            ),
        ),
        sensor_topology_ref="",
        robot_profile=RobotProfile(
            robot_name="customrobot",
            tier_recommendation="lite",
            required_phases=(
                "preflight", "system", "python_env", "build_ws",
                "config_env", "health",
            ),
            forbidden_phases=("data_plane", "models"),
            required_apt_packages=(),
            forbidden_apt_packages=("ros-${ROS_DISTRO}-rtabmap-ros",),
            required_python_packages=(),
            external_slam_script="",
            calibration_required=False,
            slam_dimension="3d",
            memory_mode_recommendation="disabled",
            perception_mode_recommendation="cloud-scene",
            telemetry_interval_sec=5.0,
            telemetry_process_keywords=("gateway", "robot_bridge", "ros2"),
        ),
    )


CUSTOMROBOT_MANIFEST = build_customrobot_manifest()
