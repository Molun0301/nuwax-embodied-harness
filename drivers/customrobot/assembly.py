from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.customrobot.manifest import build_customrobot_defaults, build_customrobot_manifest
from drivers.customrobot.providers import CustomRobotProviderBundle

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


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
        """安全空实现：骨架入口未实现，启动为 no-op，使网关启动后如实呈现未实现态。"""

    def stop(self) -> None:
        """安全空实现：骨架入口未实现，停止为 no-op。"""

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        # 消费 manifest implemented 元数据（capabilities.py metadata["implemented"]）声明未实现态，
        # 装配生命周期据此如实呈现，而非硬编码。
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


def create_customrobot_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> CustomRobotAssembly:
    """创建自定义机器人入口骨架。"""

    del iface
    return CustomRobotAssembly(config=config, adapter_configs=adapter_configs)
