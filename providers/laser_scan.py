"""二维 LaserScan 的 latest/subscribe 窄接口。"""

from __future__ import annotations

from threading import RLock
from typing import Callable, Dict, Optional, Protocol

from contracts.pointcloud import LaserScanFrame
from contracts.sensors import SensorDescriptor


class LaserScanConverter(Protocol):
    """可注入的扫描转换器；提供器不依赖具体适配器实现。"""

    def adapt(self, message: object) -> LaserScanFrame:
        """将源消息转换为标准扫描。"""


class StreamingLaserScanProvider:
    """线程安全的 latest-only LaserScan provider。

    provider 只负责样本缓存和只读订阅，不拥有 ROS/串口连接，也不把二维扫描
    转换成三维点云或 LIO measurement。
    """

    provider_name = "laser_scan"
    provider_version = "laser-scan-provider-v1"

    def __init__(
        self,
        descriptor: SensorDescriptor,
        *,
        adapter: Optional[LaserScanConverter] = None,
    ) -> None:
        self._descriptor = descriptor
        self._adapter = adapter
        self._lock = RLock()
        self._latest: Optional[LaserScanFrame] = None
        self._subscriptions: Dict[int, Callable[[LaserScanFrame], None]] = {}
        self._next_subscription_id = 1
        self._started = False
        self._published_count = 0
        self._dropped_latest_count = 0
        self._callback_error_count = 0
        self._last_error: Optional[str] = None

        if descriptor.sensor_kind.value != "lidar_2d":
            raise ValueError("LaserScan provider 需要 lidar_2d descriptor")
        if descriptor.capabilities.supports_lio:
            raise ValueError("LaserScan provider 不允许 supports_lio=True")

    def get_sensor_descriptor(self) -> SensorDescriptor:
        """返回不可变的传感器能力描述。"""

        return self._descriptor

    def is_available(self) -> bool:
        with self._lock:
            return self._started

    def start(self) -> bool:
        with self._lock:
            if self._started:
                return False
            self._started = True
            return True

    def stop(self) -> bool:
        with self._lock:
            if not self._started:
                return False
            self._started = False
            return True

    def publish(self, scan: object) -> bool:
        """发布一帧已校验的 LaserScanFrame，慢订阅者只保留最新帧。"""

        if not isinstance(scan, LaserScanFrame):
            if self._adapter is None:
                raise TypeError("provider 需要 LaserScanFrame 或配置 adapter")
            scan = self._adapter.adapt(scan)
        callbacks = []
        with self._lock:
            if not self._started:
                return False
            if self._latest is not None:
                self._dropped_latest_count += 1
            self._latest = scan
            self._published_count += 1
            callbacks = list(self._subscriptions.values())
        for callback in callbacks:
            try:
                callback(scan)
            except Exception as exc:  # 单个订阅者失败不阻断其他订阅者。
                with self._lock:
                    self._callback_error_count += 1
                    self._last_error = str(exc)
        return True

    def get_latest_laser_scan(self) -> Optional[LaserScanFrame]:
        with self._lock:
            return self._latest

    def get_latest(self) -> Optional[LaserScanFrame]:
        return self.get_latest_laser_scan()

    def subscribe_laser_scan(
        self, callback: Callable[[LaserScanFrame], None]
    ) -> int:
        if not callable(callback):
            raise TypeError("LaserScan callback 必须可调用")
        with self._lock:
            subscription_id = self._next_subscription_id
            self._next_subscription_id += 1
            self._subscriptions[subscription_id] = callback
            return subscription_id

    def subscribe(self, callback: Callable[[LaserScanFrame], None]) -> int:
        return self.subscribe_laser_scan(callback)

    def unsubscribe_laser_scan(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        with self._lock:
            self._subscriptions.pop(subscription_id, None)

    def unsubscribe(self, subscription_id: Optional[int]) -> None:
        self.unsubscribe_laser_scan(subscription_id)

    def get_status(self) -> dict:
        with self._lock:
            return {
                "started": self._started,
                "available": self._started,
                "has_latest": self._latest is not None,
                "subscriber_count": len(self._subscriptions),
                "published_count": self._published_count,
                "dropped_latest_count": self._dropped_latest_count,
                "callback_error_count": self._callback_error_count,
                "last_error": self._last_error,
                "supports_lio": False,
            }


__all__ = ["StreamingLaserScanProvider"]
