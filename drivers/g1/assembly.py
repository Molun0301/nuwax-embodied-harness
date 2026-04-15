from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.g1.defaults import build_g1_defaults
from drivers.g1.manifest import build_g1_manifest
from drivers.g1.providers import G1ProviderBundle

if TYPE_CHECKING:
    from settings import NuwaxRobotBridgeConfig


@dataclass
class G1RobotAssembly(RobotAssemblyBase):
    """G1 入口骨架。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None

    def __post_init__(self) -> None:
        self.defaults = build_g1_defaults(self.config, self.iface)
        self.manifest = build_g1_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        self.providers = G1ProviderBundle()

    def start(self) -> None:
        raise NotImplementedError("G1 入口骨架尚未实现。")

    def stop(self) -> None:
        raise NotImplementedError("G1 入口骨架尚未实现。")

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


def create_g1_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> G1RobotAssembly:
    """创建 G1 入口骨架。"""

    return G1RobotAssembly(config=config, iface=iface, adapter_configs=adapter_configs)
