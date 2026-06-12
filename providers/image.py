from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from contracts.image import CameraInfo, ImageFrame
from providers.base import BaseProvider


@runtime_checkable
class ImageProvider(BaseProvider, Protocol):
    """图像获取接口。"""

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        """抓取一帧图像。"""

    def get_camera_info(self, camera_id: Optional[str] = None) -> Optional[CameraInfo]:
        """返回相机基础参数。"""

