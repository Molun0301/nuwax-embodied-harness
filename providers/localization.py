from __future__ import annotations

from typing import Callable, Dict, Optional, Protocol, runtime_checkable

from contracts.geometry import FrameTree, Pose
from providers.base import BaseProvider


@runtime_checkable
class LocalizationProvider(BaseProvider, Protocol):
    """定位读取接口。"""

    def get_current_pose(self) -> Optional[Pose]:
        """返回当前机器人位姿。"""

    def get_frame_tree(self) -> Optional[FrameTree]:
        """返回当前 TF 坐标树快照。"""


@runtime_checkable
class StreamingLocalizationProvider(BaseProvider, Protocol):
    """定位源头流式订阅接口。"""

    def subscribe_localization(
        self,
        callback: Callable[[Pose, Optional[FrameTree], Dict[str, object]], None],
    ) -> Optional[int]:
        """订阅定位位姿与坐标树源头帧。"""

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        """取消定位订阅。"""
