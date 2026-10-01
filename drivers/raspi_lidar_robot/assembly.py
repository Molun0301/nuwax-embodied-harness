from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.raspi_lidar_robot.manifest import build_raspi_lidar_robot_defaults, build_raspi_lidar_robot_manifest
from drivers.raspi_lidar_robot.providers import RaspiLidarRobotProviderBundle

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


@dataclass
class RaspiLidarRobotAssembly(RobotAssemblyBase):
    """树莓派 + 2D 激光雷达小机器狗入口骨架。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None

    def __post_init__(self) -> None:
        self.defaults = build_raspi_lidar_robot_defaults(self.config)
        self.manifest = build_raspi_lidar_robot_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        self.providers = RaspiLidarRobotProviderBundle()

    def has_motion_actuator(self) -> Optional[bool]:
        """Only the unchanged empty scaffold can prove it owns no actuator."""
        if (type(self.providers) is RaspiLidarRobotProviderBundle
                and set(vars(self.providers)) <= {"provider_name", "provider_version"}
                and not self.list_adapters()
                and self.manifest.capability_matrix.metadata.get("implemented") is False):
            return False
        return None

    def start(self) -> None:
        """安全空实现：骨架入口未实现，启动为 no-op。"""

    def stop(self) -> None:
        """安全空实现：骨架入口未实现，停止为 no-op。"""

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        implemented = bool(self.manifest.capability_matrix.metadata.get("implemented", False))
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=implemented,
            control_mode="unknown",
            low_level_ready=False,
            low_level_running=False,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )


def create_raspi_lidar_robot_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> RaspiLidarRobotAssembly:
    """创建树莓派 + 2D 激光雷达小机器狗入口骨架。"""

    del iface
    return RaspiLidarRobotAssembly(config=config, adapter_configs=adapter_configs)
