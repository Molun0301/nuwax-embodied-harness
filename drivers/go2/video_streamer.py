from __future__ import annotations

import asyncio
import importlib
import logging
import os
from pathlib import Path
import sys
import time
from typing import Callable, Optional, TypeVar

from drivers.go2.settings import Go2DataPlaneConfig, load_go2_data_plane_config
from drivers.common.video_uds_sender import VideoUdsSender


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.video_streamer")
T = TypeVar("T")


class Go2VideoStreamerConfigError(RuntimeError):
    """Go2 端侧推流配置错误。"""


def _prepare_go2_sdk_environment(config: Go2DataPlaneConfig) -> None:
    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    importlib.invalidate_caches()


def _initialize_channel(config: Go2DataPlaneConfig) -> None:
    channel_module = importlib.import_module("unitree_sdk2py.core.channel")
    channel_factory_initialize = getattr(channel_module, "ChannelFactoryInitialize")
    iface = str(config.dds_iface or "").strip()
    try:
        if iface:
            channel_factory_initialize(0, iface)
        else:
            channel_factory_initialize(0)
    except TypeError:
        channel_factory_initialize(0)


def _create_video_client(config: Go2DataPlaneConfig):
    _prepare_go2_sdk_environment(config)
    if not any(Path(path).expanduser().joinpath("unitree_sdk2py").exists() for path in sys.path):
        raise Go2VideoStreamerConfigError(
            "未找到 unitree_sdk2py。请在 drivers/go2/data_plane.env 设置 GO2_SDK_PATH，"
            "例如 GO2_SDK_PATH=/home/unitree/unitree_sdk2_python。"
        )
    _initialize_channel(config)
    video_client_factory = getattr(
        importlib.import_module("unitree_sdk2py.go2.video.video_client"),
        "VideoClient",
    )
    client = video_client_factory()
    client.SetTimeout(float(getattr(config.official, "client_timeout_sec", 3.0)))
    client.Init()
    return client


def _extract_jpeg_payload(data: object) -> Optional[bytes]:
    raw = bytes(data or b"")
    soi = raw.find(b"\xff\xd8")
    eoi = raw.rfind(b"\xff\xd9")
    if soi < 0 or eoi <= soi:
        return None
    return raw[soi : eoi + 2]


async def _run_blocking(callable_obj: Callable[..., T], *args: object, **kwargs: object) -> T:
    """在兼容 Python 3.8 的线程池中执行阻塞 SDK 调用。"""

    to_thread = getattr(asyncio, "to_thread", None)
    if callable(to_thread):
        return await to_thread(callable_obj, *args, **kwargs)
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


async def _stream_once(config: Go2DataPlaneConfig) -> None:
    stream_config = config.video_stream
    client = await _run_blocking(_create_video_client, config)
    frame_interval_sec = 1.0 / max(1.0, float(stream_config.fps))

    uds_path = os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock")
    uds_sender = VideoUdsSender(uds_path, reconnect_interval=1.0)
    if not uds_sender.connect():
        raise RuntimeError(f"无法连接 Rust 视频中转 UDS: {uds_path}")
    LOGGER.info("视频推流已连接 Rust UDS: %s", uds_path)

    while True:
        started_at = time.monotonic()
        code, data = await _run_blocking(client.GetImageSample)
        if code != 0 or not data:
            raise RuntimeError(f"Go2 VideoClient.GetImageSample 失败: code={code}")
        payload = _extract_jpeg_payload(data)
        if payload is None:
            raise RuntimeError("Go2 VideoClient 返回的数据不是完整 JPEG。")
        if not uds_sender.send_frame(payload):
            raise RuntimeError("UDS 发送失败，连接已断开。")
        elapsed = time.monotonic() - started_at
        remaining = frame_interval_sec - elapsed
        if remaining > 0:
            await asyncio.sleep(remaining)


async def run_forever(config: Go2DataPlaneConfig) -> None:
    if not config.video_stream.enabled:
        LOGGER.info("Go2 端侧视频推流未启用。")
        return
    reconnect_sec = max(0.1, float(config.video_stream.reconnect_sec))
    while True:
        try:
            await _stream_once(config)
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception("Go2 端侧视频推流中断，%.1fs 后重连。", reconnect_sec)
            await asyncio.sleep(reconnect_sec)


def main() -> None:
    logging.basicConfig(level=os.environ.get("GO2_VIDEO_STREAM_LOG_LEVEL", "INFO"))
    config = load_go2_data_plane_config()
    asyncio.run(run_forever(config))


if __name__ == "__main__":
    main()
