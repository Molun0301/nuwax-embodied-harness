"""设备事件在网关进程订阅，沿通用提供方与既有输入桥交付。"""
from __future__ import annotations
from dataclasses import dataclass
from copy import deepcopy
import logging
from typing import Optional

from contracts.calibration import SensorTopology
from contracts.capabilities import CapabilityMatrix
from contracts.sensors import SensorStreamFailure
from core.config import install_home, value
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults, RobotManifest, RobotProfile
from drivers.customrobot.providers import CustomRobotProviderBundle
from .clock import CaptureTimes, source_time_path
from .device import Device
from .events import EventDecoder


class Providers(CustomRobotProviderBundle):
    provider_name = 'unitree_sim_inputs'

    def set_camera_pair(self, pair):
        with self._lock:
            pairs = self._sensor_topology.pairs if self._sensor_topology else []
            self._sensor_topology = SensorTopology(robot_id='unitree_sim', pairs=[
                item for item in pairs if item.child_frame_id != pair.child_frame_id] + [pair])

    def fail(self, reason):
        self._notify('sensor_sample', SensorStreamFailure(reason=reason))
        try:
            self.stop()
        except Exception as error:
            logging.getLogger(__name__).error('断流后停止请求失败：%s', error)


@dataclass(frozen=True)
class Status(RobotAssemblyStatus):
    error: Optional[str] = None
    device_capabilities: Optional[dict] = None


class UnitreeSimAssembly(RobotAssemblyBase):
    def __init__(self, config, adapter_configs=None):
        self._initialize_adapter_runtime(adapter_configs)
        self.config = config
        model = value('unitree_sim.model')
        self.manifest = RobotManifest(
            robot_name='unitree_sim', robot_model=model,
            entrypoint='drivers/unitree_sim:create_unitree_sim_assembly',
            description='unitree-sim/1 设备协议接入，沿用标准输入与运动守卫。',
            capability_matrix=CapabilityMatrix(robot_model=model, capabilities=[]),
            robot_profile=RobotProfile(robot_name='unitree_sim', tier_recommendation='full', calibration_required=True,
                required_phases=('preflight', 'system', 'python_env', 'geometry_runtime_deps', 'config_env', 'health'),
                required_python_packages=('msgpack==1.0.8',)))
        base_frame = value('robot.base_frame')
        self.defaults = RobotDefaults(frame_ids={'base': base_frame},
                                      topics={'state': 'robot.unitree_sim.state'}, parameters={'robot_model': model})
        self.device = Device(value('unitree_sim.base_url'), model, base_frame)
        self.providers = Providers(motion_control=self.device)
        self.clock = None

    def has_motion_actuator(self):
        return True

    def get_motion_command_limits(self):
        return self.device.get_motion_command_limits()

    def start(self):
        if self.providers.is_available():
            return
        self.device.bind()
        self.clock = CaptureTimes(source_time_path(install_home()))
        decoder = EventDecoder(self.providers, self.device, self.clock)
        self.providers.start()
        try:
            self.device.subscribe(decoder.accept, self.providers.fail)
            self.initialize_registered_adapters()
        except Exception:
            try:
                self.providers.stop()
            finally:
                self.device.close()
                self.clock.close()
                self.clock = None
            raise

    def stop(self):
        try:
            self.providers.stop()
        finally:
            self.device.close()
            if self.clock is not None:
                self.clock.close()
                self.clock = None
            self.stop_registered_adapters()

    def get_status(self):
        return Status(robot_name='unitree_sim', initialized=self.providers.is_available() and self.device.running,
                      control_mode='motion_control', low_level_ready=False, low_level_running=False,
                      error=self.device.error, device_capabilities=deepcopy(self.device.backend))


def create_unitree_sim_assembly(config, iface=None, adapter_configs=None):
    return UnitreeSimAssembly(config, adapter_configs)
