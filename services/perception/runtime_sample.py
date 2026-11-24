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
    # 发布时取出的拍照时刻点云帧，后台补几何时与当时缓存合并使用。
    point_cloud_frames: tuple = ()
