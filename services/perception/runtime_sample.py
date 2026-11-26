"""统一感知循环发布的同帧数据，消费者不再自行启动检测循环。"""
from dataclasses import dataclass
from typing import Optional

from contracts.geometry import FrameTree, Pose
from contracts.runtime_views import PerceptionContext


@dataclass(frozen=True)
class PerceptionSample:
    context: PerceptionContext
    map_name: str
    pose: Optional[Pose] = None
    frame_tree: Optional[FrameTree] = None
    image_bytes: Optional[bytes] = None
    inputs: Optional[tuple] = None
    complete: bool = False
