from __future__ import annotations

from typing import Callable, ContextManager, Optional, Protocol, runtime_checkable

from contracts.pointcloud import PointCloudFrame
from providers.base import BaseProvider


@runtime_checkable
class PointCloudProvider(BaseProvider, Protocol):
    """点云读取接口。"""

    def get_latest_point_cloud(self) -> Optional[PointCloudFrame]:
        """返回最近可用的一帧点云。"""


@runtime_checkable
class DemandPointCloudProvider(Protocol):
    """提供按真实业务使用者启停点云采集的能力。"""

    def point_cloud_consumer(self) -> ContextManager:
        """在使用期间持有点云需求，退出时释放。"""


@runtime_checkable
class StreamingPointCloudProvider(BaseProvider, Protocol):
    """点云源头流式订阅接口。"""

    def subscribe_point_cloud(self, callback: Callable[[PointCloudFrame], None]) -> Optional[int]:
        """订阅点云源头帧。"""

    def unsubscribe_point_cloud(self, subscription_id: Optional[int]) -> None:
        """取消点云订阅。"""
