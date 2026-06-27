from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RaspiLidarRobotProviderBundle:
    """树莓派 + 2D 激光雷达小机器狗 provider 占位。"""

    provider_name: str = "raspi_lidar_robot_placeholder"
    provider_version: str = "0.1.0"
