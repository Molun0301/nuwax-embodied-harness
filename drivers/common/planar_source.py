"""可选二维雷达的装配与资源拥有方，向平台只交付规范样本提供器。"""
from __future__ import annotations

import json
from pathlib import Path

from contracts.sensors import SensorCapabilities, SensorDescriptor
from providers.laser_scan import StreamingLaserScanProvider


class ManagedPlanarSource:
    def __init__(self, provider, adapter):
        self.provider = provider
        self.adapter = adapter

    def start(self) -> None:
        self.provider.start()
        self.adapter.initialize()

    def stop(self) -> None:
        self.adapter.stop()
        self.provider.stop()

    def is_ready(self) -> bool:
        return self.provider.is_available() and self.adapter.is_receiving()


def create_planar_sensor_source(config_text: str, session: dict) -> ManagedPlanarSource:
    """按已捕获的二维会话配置装配首个思岚驱动，不创建机器人或运动连接。"""
    from drivers.sensors.slamtec import SlamtecConfig, SlamtecLidarAdapter
    config = json.loads(config_text)
    if not isinstance(config, dict) or config.pop("driver", "slamtec") != "slamtec":
        raise ValueError("二维设备入口目前支持slamtec驱动")
    session = dict(session)
    if session.get("clock_domain") == "monotonic_ns" and session.get("clock_id") == "local_boot":
        session["clock_id"] = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if (session.get("clock_domain") != "monotonic_ns"
            or session.get("clock_id") != Path("/proc/sys/kernel/random/boot_id").read_text().strip()):
        raise ValueError("思岚输入必须绑定本机单调时钟和启动身份")
    for key in ("frame_id", "sensor_id", "calibration_ref"):
        config[key] = session[key]
    settings = SlamtecConfig(**config)
    descriptor = SensorDescriptor(
        sensor_id=settings.sensor_id, sensor_kind="lidar_2d", frame_id=settings.frame_id,
        clock_domain=session["clock_domain"], clock_id=session["clock_id"], calibration_ref=settings.calibration_ref,
        capabilities=SensorCapabilities(dimensionality="2d"), adapter_name="slamtec", adapter_version="1",
    )
    provider = StreamingLaserScanProvider(descriptor)
    adapter = SlamtecLidarAdapter(settings, provider.publish_sensor_sample)
    return ManagedPlanarSource(provider, adapter)
