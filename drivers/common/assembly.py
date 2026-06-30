from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, Any

from adapters.base import AdapterBase, AdapterConfig, AdapterHealthStatus
from drivers.common.manifest import RobotDefaults, RobotManifest


@dataclass(frozen=True)
class RobotAssemblyStatus:
    """机器人装配状态。"""

    robot_name: str
    initialized: bool
    control_mode: str
    low_level_ready: bool
    low_level_running: bool
    adapter_count: int = 0
    healthy_adapter_count: int = 0
    # 修复 openspec/changes/fix-g1-loco-init-no-motion-switcher：
    # 优雅降级后暴露失败原因（默认空字符串）；仅 G1 装配启用，其他机器人保留默认。
    last_error: str = ""


class RobotAssemblyBase(ABC):
    """机器人装配入口基类。"""

    manifest: RobotManifest
    defaults: RobotDefaults
    _adapter_registry: Dict[str, AdapterBase[Any, Any]]
    _adapter_config_registry: Dict[str, AdapterConfig]

    def _initialize_adapter_runtime(self, adapter_configs: Optional[Dict[str, AdapterConfig]] = None) -> None:
        """初始化适配器运行时注册表。"""

        self._adapter_registry = {}
        self._adapter_config_registry = dict(adapter_configs or {})

    def get_adapter_config(self, adapter_name: str) -> Optional[AdapterConfig]:
        """获取指定适配器的配置覆盖。"""

        return self._adapter_config_registry.get(adapter_name)

    def resolve_adapter_config(self, default_config: AdapterConfig) -> AdapterConfig:
        """解析默认配置与外部覆盖。"""

        override = self.get_adapter_config(default_config.name)
        if override is None:
            return default_config
        return override.model_copy(deep=True)

    def bind_adapter(self, adapter: AdapterBase[Any, Any]) -> AdapterBase[Any, Any]:
        """绑定适配器到机器人入口。"""

        if adapter.adapter_name in self._adapter_registry:
            raise ValueError(f"适配器 {adapter.adapter_name} 已存在。")

        override = self.get_adapter_config(adapter.adapter_name)
        if override is not None:
            adapter.config = override.model_copy(deep=True)

        self._adapter_registry[adapter.adapter_name] = adapter
        return adapter

    def get_adapter(self, adapter_name: str) -> Optional[AdapterBase[Any, Any]]:
        """获取已绑定的适配器。"""

        return self._adapter_registry.get(adapter_name)

    def list_adapters(self) -> Tuple[AdapterBase[Any, Any], ...]:
        """列出当前所有已绑定适配器。"""

        return tuple(self._adapter_registry.values())

    def initialize_registered_adapters(self) -> None:
        """初始化所有已绑定适配器。"""

        for adapter in self._adapter_registry.values():
            adapter.initialize()

    def stop_registered_adapters(self) -> None:
        """停止所有已绑定适配器。"""

        for adapter in self._adapter_registry.values():
            adapter.stop()

    def get_adapter_health_statuses(self) -> Tuple[AdapterHealthStatus, ...]:
        """汇总全部适配器健康状态。"""

        return tuple(adapter.health_check() for adapter in self._adapter_registry.values())

    @abstractmethod
    def start(self) -> None:
        """启动机器人入口。"""

    @abstractmethod
    def stop(self) -> None:
        """停止机器人入口。"""

    @abstractmethod
    def get_status(self) -> RobotAssemblyStatus:
        """返回当前装配状态。"""
