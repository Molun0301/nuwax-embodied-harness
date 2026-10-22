from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotDefaults, RobotManifest, RobotProfile
from drivers.customrobot.capabilities import CUSTOMROBOT_CAPABILITY_MATRIX

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_customrobot_defaults(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotDefaults:
    """构造标准输入默认命名；物理标定仍须由接入方提供。"""

    frames = config.geometry.frames if config is not None else None
    return RobotDefaults(
        frame_ids={
            "base": frames.base_frame if frames is not None else "base_link",
            "camera_front": "world/customrobot/camera_front",
            "imu": "world/customrobot/imu",
            "lidar": "world/customrobot/lidar",
        },
        topics={"state": "robot.customrobot.state", "point_cloud": "robot.customrobot.point_cloud"},
        parameters={"robot_id": "customrobot", "robot_model": "customrobot"},
    )


def build_customrobot_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    """构造通用标准输入装配清单。"""

    del config
    return RobotManifest(
        robot_name="customrobot",
        robot_model="customrobot",
        entrypoint="drivers/customrobot/assembly.py:create_customrobot_assembly",
        description="通用标准数据接入；发布传感器帧并按需注入运动执行与标定，支持离线回放。",
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
                "preflight", "system", "python_env", "geometry_runtime_deps",
                "config_env", "health",
            ),
            forbidden_phases=("data_plane", "models"),
            required_apt_packages=(),
            forbidden_apt_packages=(),
            required_python_packages=(),
            external_slam_script="",
            calibration_required=False,
            slam_dimension="3d",
            memory_mode_recommendation="disabled",
            perception_mode_recommendation="cloud-scene",
            telemetry_interval_sec=5.0,
            telemetry_process_keywords=("gateway", "nuwax_geometry_runtime", "robot_bridge"),
        ),
    )


CUSTOMROBOT_MANIFEST = build_customrobot_manifest()
