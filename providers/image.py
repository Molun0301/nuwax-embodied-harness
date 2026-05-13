from __future__ import annotations

from typing import Callable, Optional, Protocol, runtime_checkable

from contracts.image import CameraInfo, ImageFrame
from providers.base import BaseProvider


@runtime_checkable
class ImageProvider(BaseProvider, Protocol):
    """图像获取接口。"""

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        """抓取一帧图像。"""

    def get_camera_info(self, camera_id: Optional[str] = None) -> Optional[CameraInfo]:
        """返回相机基础参数。"""


@runtime_checkable
class StreamingImageProvider(BaseProvider, Protocol):
    """源头推送图像流接口。"""

    def subscribe_image(self, callback: Callable[[ImageFrame], None]) -> Optional[int]:
        """订阅源头推送图像帧。"""

    def unsubscribe_image(self, subscription_id: Optional[int]) -> None:
        """取消源头推送图像帧订阅。"""

