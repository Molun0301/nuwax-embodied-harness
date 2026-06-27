"""raspi_lidar_robot 机器人入口导出。"""

from drivers.raspi_lidar_robot.assembly import create_raspi_lidar_robot_assembly
from drivers.raspi_lidar_robot.manifest import RASPI_LIDAR_ROBOT_MANIFEST

__all__ = [
    "RASPI_LIDAR_ROBOT_MANIFEST",
    "create_raspi_lidar_robot_assembly",
]
