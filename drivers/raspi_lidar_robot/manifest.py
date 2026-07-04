from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from drivers.common.manifest import ComponentBinding, RobotDefaults, RobotManifest, RobotProfile
from drivers.raspi_lidar_robot.capabilities import RASPI_LIDAR_CAPABILITY_MATRIX

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_raspi_lidar_robot_defaults(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotDefaults:
    """树莓派 + 2D 激光雷达小机器狗的默认配置骨架。"""
    del config
    return RobotDefaults(
        frame_ids={
            "base": "world/raspi_lidar_robot/base",
            "laser": "world/raspi_lidar_robot/laser",
            "camera": "world/raspi_lidar_robot/camera",
        },
        topics={
            "robot_state": "robot.raspi_lidar_robot.state",
            "image": "robot.raspi_lidar_robot.image",
            "scan": "robot.raspi_lidar_robot.scan",
        },
        parameters={
            "robot_id": "raspi_lidar_robot",
            "robot_model": "raspi_lidar_robot",
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 115200,
            "lidar_frame": "world/raspi_lidar_robot/laser",
        },
    )


def build_raspi_lidar_robot_manifest(config: Optional["NuwaxRobotBridgeConfig"] = None) -> RobotManifest:
    """树莓派 + 2D 激光雷达 + USB 摄像头小机器狗入口清单。"""
    del config
    return RobotManifest(
        robot_name="raspi_lidar_robot",
        robot_model="raspi_lidar_robot",
        entrypoint="drivers/raspi_lidar_robot/assembly.py:create_raspi_lidar_robot_assembly",
        description="树莓派 + 2D 激光雷达 + USB 摄像头小机器狗；tier=lite；2D SLAM 走 native runtime（nuwax-geometry-runtime），不再引入 ROS2 包。",
        capability_matrix=RASPI_LIDAR_CAPABILITY_MATRIX,
        required_components=(
            ComponentBinding(
                name="serial_chassis_driver",
                path="drivers.raspi_lidar_robot",
                description="底盘串口控制（pyserial）。",
            ),
        ),
        optional_components=(
            ComponentBinding(
                name="raspi_rtabmap_script",
                path="services.geometry.external_script_runner.RaspiRtabMapScript",
                description="2D 激光雷达 native SLAM launcher（nuwax-geometry-runtime 2D 后端）。",
                required=False,
            ),
            ComponentBinding(
                name="raspi_camera_v4l2",
                path="drivers.common.streaming.V4L2VideoStreamer",
                description="USB 摄像头 V4L2 抓帧。",
                required=False,
            ),
        ),
        default_sensors=("laser_2d", "usb_camera"),
        default_audio_backends=(),
        default_adapters=(),
        sensor_topology_ref="",
        robot_profile=RobotProfile(
            robot_name="raspi_lidar_robot",
            tier_recommendation="lite",
            required_phases=(
                "preflight", "system", "python_env", "geometry_runtime_deps",
                "config_env", "health",
            ),
            forbidden_phases=("data_plane", "models", "systemd", "nuwaclaw"),
            required_apt_packages=(
                "python3-pip", "python3-serial", "python3-opencv",
            ),
            forbidden_apt_packages=(),
            required_python_packages=("pyserial>=3.5",),
            external_slam_script="RaspiNativeSlamScript",
            calibration_required=False,
            slam_dimension="2d",
            memory_mode_recommendation="disabled",
            perception_mode_recommendation="cloud-scene",
            telemetry_interval_sec=5.0,
            telemetry_process_keywords=("gateway", "nuwax_geometry_runtime", "robot_bridge"),
        ),
    )


RASPI_LIDAR_ROBOT_MANIFEST = build_raspi_lidar_robot_manifest()
