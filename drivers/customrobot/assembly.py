from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, Optional

from adapters.base import AdapterConfig
from contracts.calibration import SensorTopology
from contracts.motion_limits import MotionCommandLimits
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults
from drivers.customrobot.manifest import build_customrobot_defaults, build_customrobot_manifest
from drivers.customrobot.providers import CustomRobotProviderBundle
from providers.motion import MotionControl

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


@dataclass
class CustomRobotAssembly(RobotAssemblyBase):
    """标准数据输入与可注入运动执行器的通用机器人入口。"""

    config: NuwaxRobotBridgeConfig
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None
    motion_control: Optional[MotionControl] = None
    sensor_topology: Optional[SensorTopology] = None
    defaults: Optional[RobotDefaults] = None

    def __post_init__(self) -> None:
        self.defaults = self.defaults or build_customrobot_defaults(self.config)
        self.manifest = build_customrobot_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        self.providers = CustomRobotProviderBundle(
            motion_control=self.motion_control, sensor_topology=self.sensor_topology,
        )

    def has_motion_actuator(self) -> Optional[bool]:
        """Injected motion is present; external adapters may own unknown actuators."""
        if self.motion_control is not None:
            return True
        if self.list_adapters():
            return None
        return False

    def get_motion_command_limits(self) -> Optional[MotionCommandLimits]:
        """接入方只返回已发现的本地策略快照；目录读取不能发起设备请求。"""
        getter = getattr(self.motion_control, "get_motion_command_limits", None)
        return getter() if callable(getter) else None

    def start(self) -> None:
        """启用标准输入并初始化接入方绑定的适配器。"""

        if self.providers.is_available():
            return
        self.providers.start()
        try:
            self.initialize_registered_adapters()
        except Exception:
            try:
                self.stop_registered_adapters()
            finally:
                self.providers.stop()
            raise

    def stop(self) -> None:
        """停止输入与执行器，清除旧样本，避免重启后读取旧帧。"""

        try:
            self.stop_registered_adapters()
        finally:
            self.providers.stop()

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.providers.is_available(),
            control_mode="unknown",
            low_level_ready=False,
            low_level_running=False,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )


def create_customrobot_assembly(
    config: NuwaxRobotBridgeConfig,
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
    *,
    motion_control: Optional[MotionControl] = None,
    sensor_topology: Optional[SensorTopology] = None,
    defaults: Optional[RobotDefaults] = None,
) -> CustomRobotAssembly:
    """创建通用装配；不连接硬件，实际输入和执行器由接入方提供。"""

    del iface
    return CustomRobotAssembly(
        config=config, adapter_configs=adapter_configs, motion_control=motion_control,
        sensor_topology=sensor_topology, defaults=defaults,
    )
