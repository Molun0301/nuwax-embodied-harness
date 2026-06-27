"""drivers/common/streaming —— 视频流抽象（cross-robot-generalization-foundations G6）。

`VideoStreamer` 抽象类只暴露 `start() / stop() / get_frame() -> ImageFrame`。
具体协议（DDS / WebRTC / V4L2）由子类实现；协议名仅在 `get_video_streamer()` 工厂
内部 select，不在 Provider 接口暴露。

平台侧消费入口：`get_video_streamer(robot_name, camera_id)`，由 driver/assembly.py 在
`__post_init__` 阶段调 `bind_adapter()` 注入。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class StreamerDescriptor:
    """视频流实现描述，供工厂返回。"""

    backend: str  # "dds" | "webrtc" | "v4l2" | "mock"
    implementation: str  # 类名，如 "Go2DdsVideoStreamer"


class VideoStreamer(ABC):
    """视频流抽象类。"""

    backend: str = ""

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_frame(self, timeout_sec: float = 1.0) -> Optional[bytes]:
        """抓一帧；返回 JPEG/PNG bytes 或 None（无帧）。"""


class Go2DdsVideoStreamer(VideoStreamer):
    """Go2 走 Unitree DDS 推流；薄适配，逻辑来自原 drivers/go2/video_streamer.py。

    本期保留为占位（实际实现仍走 drivers/go2/video_streamer.py），下一 PR 把原文件
    全部下沉到本类。
    """

    backend = "dds"

    def __init__(self, *, camera_id: str = "front_camera", iface: Optional[str] = None) -> None:
        self._camera_id = camera_id
        self._iface = iface
        self._running: bool = False

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def get_frame(self, timeout_sec: float = 1.0) -> Optional[bytes]:
        if not self._running:
            return None
        # 实际取帧由 drivers/go2/video_streamer.py 业务承担；本类为薄适配。
        return None


class G1WebRtcVideoStreamer(VideoStreamer):
    """G1 走 Unitree WebRTC 接收。"""

    backend = "webrtc"

    def __init__(self, *, camera_id: str = "front_camera", webrtc_client=None) -> None:
        self._camera_id = camera_id
        self._client = webrtc_client
        self._running: bool = False

    def start(self) -> None:
        self._running = self._client is not None

    def stop(self) -> None:
        self._running = False

    def get_frame(self, timeout_sec: float = 1.0) -> Optional[bytes]:
        if not self._running or self._client is None:
            return None
        # 实际取帧由 aiortc 客户端承担；本期占位。
        return None


class V4L2VideoStreamer(VideoStreamer):
    """树莓派 / 通用 V4L2 USB 摄像头抓帧。"""

    backend = "v4l2"

    def __init__(self, *, camera_id: str = "front_camera", device: str = "/dev/video0") -> None:
        self._camera_id = camera_id
        self._device = device
        self._running: bool = False

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def get_frame(self, timeout_sec: float = 1.0) -> Optional[bytes]:
        if not self._running:
            return None
        # 实际取帧由 cv2.VideoCapture(device).read() 承担；本期占位。
        return None


__all__ = [
    "StreamerDescriptor",
    "VideoStreamer",
    "Go2DdsVideoStreamer",
    "G1WebRtcVideoStreamer",
    "V4L2VideoStreamer",
]