from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class RobotDriverRegistration:
    """机器人入口注册条目。"""

    robot_name: str
    package: str
    factory_name: str
    private_capabilities_module: str
    description: str


ROBOT_DRIVER_REGISTRY: Dict[str, RobotDriverRegistration] = {
    "go2": RobotDriverRegistration(
        robot_name="go2",
        package="drivers.go2",
        factory_name="create_go2_assembly",
        private_capabilities_module="drivers.go2.private_capabilities",
        description="Go2 正式参考实现。",
    ),
    "g1": RobotDriverRegistration(
        robot_name="g1",
        package="drivers.g1",
        factory_name="create_g1_assembly",
        private_capabilities_module="drivers.g1.private_capabilities",
        description="G1 机器人入口骨架。",
    ),
    "customrobot": RobotDriverRegistration(
        robot_name="customrobot",
        package="drivers.customrobot",
        factory_name="create_customrobot_assembly",
        private_capabilities_module="drivers.customrobot.private_capabilities",
        description="用户自定义机器人入口骨架。",
    ),
    "raspi_lidar_robot": RobotDriverRegistration(
        robot_name="raspi_lidar_robot",
        package="drivers.raspi_lidar_robot",
        factory_name="create_raspi_lidar_robot_assembly",
        private_capabilities_module="drivers.raspi_lidar_robot.private_capabilities",
        description="树莓派 + 2D 激光雷达 + USB 摄像头小机器狗；tier=lite；原生 2D 建图模式。",
    ),
}


def get_robot_driver_registration(robot_name: str) -> RobotDriverRegistration:
    """返回指定机器人注册条目。"""

    normalized = str(robot_name or "").strip().lower()
    if not normalized:
        raise ValueError("机器人名称不能为空。")
    registration = ROBOT_DRIVER_REGISTRY.get(normalized)
    if registration is None:
        supported = ", ".join(sorted(ROBOT_DRIVER_REGISTRY))
        raise ValueError(f"未注册的机器人入口: {normalized}。当前支持: {supported}")
    return registration
