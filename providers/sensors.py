"""设备无关的传感器提供器窄接口。"""

from __future__ import annotations

from typing import Callable, Optional, Protocol, runtime_checkable

from contracts.sensors import SensorDescriptor, SensorSample
from providers.base import BaseProvider


@runtime_checkable
class SensorDescriptorProvider(BaseProvider, Protocol):
    """暴露稳定 descriptor，不承担设备格式转换。"""

    def get_sensor_descriptor(self) -> SensorDescriptor:
        """返回当前 provider 已协商的传感器描述。"""


@runtime_checkable
class StreamingSensorSampleProvider(BaseProvider, Protocol):
    """规范化传感器样本的源头流式接口。

    一个设备装配可同时提供 LiDAR 和 IMU，不要求它伪装成单个传感器描述。
    单传感器的描述查询仍由独立的 SensorDescriptorProvider 提供。
    """

    def subscribe_sensor_sample(self, callback: Callable[[SensorSample], None]) -> Optional[int]:
        """订阅规范化传感器样本。"""

    def unsubscribe_sensor_sample(self, subscription_id: Optional[int]) -> None:
        """取消规范化传感器样本订阅。"""
