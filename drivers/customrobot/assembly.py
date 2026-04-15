from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.customrobot.manifest import build_customrobot_defaults, build_customrobot_manifest
from drivers.customrobot.providers import CustomRobotProviderBundle

if TYPE_CHECKING:
    from settings import NuwaxRobotBridgeConfig


@dataclass
class CustomRobotAssembly(RobotAssemblyBase):
    """自定义机器人入口骨架。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None

    def __post_init__(self) -> None:
        self.defaults = build_customrobot_defaults(self.config)
        self.manifest = build_customrobot_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        self.providers = CustomRobotProviderBundle()

    def start(self) -> None:
        raise NotImplementedError("customrobot 入口骨架尚未实现。")

    def stop(self) -> None:
        raise NotImplementedError("customrobot 入口骨架尚未实现。")

    def get_status(self) -> RobotAssemblyStatus:
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=False,
            control_mode="unknown",
            low_level_ready=False,
            low_level_running=False,
            adapter_count=len(self.get_adapter_health_statuses()),
            healthy_adapter_count=sum(1 for status in self.get_adapter_health_statuses() if status.is_healthy),
        )


def create_customrobot_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> CustomRobotAssembly:
    """创建自定义机器人入口骨架。"""

    del iface
    return CustomRobotAssembly(config=config, adapter_configs=adapter_configs)
