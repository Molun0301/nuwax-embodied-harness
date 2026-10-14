"""drivers/common/streaming 工厂入口。"""

from drivers.common.streaming.base import (
    G1WebRtcVideoStreamer,
    Go2DdsVideoStreamer,
    StreamerDescriptor,
    V4L2VideoStreamer,
    VideoStreamer,
)

_ROBOT_BACKEND = {
    "go2": Go2DdsVideoStreamer,
    "g1": G1WebRtcVideoStreamer,
    "raspi_lidar_robot": V4L2VideoStreamer,
}


def get_video_streamer(robot_name: str, camera_id: str = "front_camera", **kwargs) -> VideoStreamer:
    """按 robot_name 选 backend，构造 VideoStreamer 实例。

    跨机器人变更记录：cross-robot-generalization-foundations G6。
    """

    cls = _ROBOT_BACKEND.get(robot_name)
    if cls is None:
        raise ValueError(
            f"未知 robot_name={robot_name!r}；当前支持: {sorted(_ROBOT_BACKEND)}"
        )
    return cls(camera_id=camera_id, **kwargs)


__all__ = [
    "G1WebRtcVideoStreamer",
    "Go2DdsVideoStreamer",
    "StreamerDescriptor",
    "V4L2VideoStreamer",
    "VideoStreamer",
    "get_video_streamer",
]