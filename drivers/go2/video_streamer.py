from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import importlib
import json
import logging
import os
from pathlib import Path
import sys
import time
from typing import Callable, Optional, TypeVar
from urllib.parse import urlencode, urlsplit, urlunsplit, parse_qsl

import websockets

from drivers.go2.defaults import GO2_DEFAULT_CAMERA_INFO
from drivers.go2.settings import Go2DataPlaneConfig, load_go2_data_plane_config


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.video_streamer")
T = TypeVar("T")


class Go2VideoStreamerConfigError(RuntimeError):
    """Go2 端侧推流配置错误。"""


def _prepend_env_path(name: str, value: str) -> None:
    normalized = str(value or "").strip()
    if not normalized:
        return
    current = str(os.environ.get(name, "") or "").strip()
    entries = [item for item in current.split(":") if item.strip()]
    if normalized in entries:
        entries = [item for item in entries if item != normalized]
    os.environ[name] = ":".join([normalized, *entries]) if entries else normalized


def _prepare_go2_sdk_environment(config: Go2DataPlaneConfig) -> None:
    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    cyclonedds_lib_dir = str(getattr(config, "cyclonedds_lib_dir", "") or "").strip()
    if cyclonedds_lib_dir:
        _prepend_env_path("LD_LIBRARY_PATH", cyclonedds_lib_dir)

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


def _gateway_url_with_token(url: str, token: str) -> str:
    normalized = str(url or "").strip()
    if not normalized:
        normalized = "ws://127.0.0.1:8766/streams/video.ingest"
    if not token:
        return normalized
    parts = urlsplit(normalized)
    query = dict(parse_qsl(parts.query, keep_blank_values=True))
    query.setdefault("access_token", token)
    return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), parts.fragment))


async def _run_blocking(callable_obj: Callable[..., T], *args: object, **kwargs: object) -> T:
    """在兼容 Python 3.8 的线程池中执行阻塞 SDK 调用。"""

    to_thread = getattr(asyncio, "to_thread", None)
    if callable(to_thread):
        return await to_thread(callable_obj, *args, **kwargs)
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


async def _stream_once(config: Go2DataPlaneConfig) -> None:
    stream_config = config.video_stream
    gateway_url = _gateway_url_with_token(stream_config.gateway_url, stream_config.access_token)
    client = await _run_blocking(_create_video_client, config)
    frame_interval_sec = 1.0 / max(1.0, float(stream_config.fps))
    camera_id = str(stream_config.camera_id or GO2_DEFAULT_CAMERA_INFO.camera_id)
    frame_id = str(stream_config.frame_id or GO2_DEFAULT_CAMERA_INFO.frame_id)
    width_px = int(stream_config.width_px or GO2_DEFAULT_CAMERA_INFO.width_px)
    height_px = int(stream_config.height_px or GO2_DEFAULT_CAMERA_INFO.height_px)
    camera_info = GO2_DEFAULT_CAMERA_INFO.model_copy(
        update={
            "camera_id": camera_id,
            "frame_id": frame_id,
            "width_px": width_px,
            "height_px": height_px,
        },
        deep=True,
    ).model_dump(mode="json")

    async with websockets.connect(gateway_url, max_size=None, ping_interval=15, ping_timeout=15) as websocket:
        await websocket.send(
            json.dumps(
                {
                    "type": "video.start",
                    "camera_id": camera_id,
                    "frame_id": frame_id,
                    "width_px": width_px,
                    "height_px": height_px,
                    "encoding": "jpeg",
                    "source_kind": "go2_side_stream_push",
                    "robot": "unitree_go2",
                    "streamer_version": "go2_video_streamer.v1",
                    "camera_info": camera_info,
                    "started_at": datetime.now(timezone.utc).isoformat(),
                },
                ensure_ascii=False,
            )
        )
        try:
            await asyncio.wait_for(websocket.recv(), timeout=3.0)
        except asyncio.TimeoutError:
            LOGGER.warning("平台视频 ingest 未在 3 秒内确认，继续推流。")

        while True:
            started_at = time.monotonic()
            code, data = await _run_blocking(client.GetImageSample)
            if code != 0 or not data:
                raise RuntimeError(f"Go2 VideoClient.GetImageSample 失败: code={code}")
            payload = _extract_jpeg_payload(data)
            if payload is None:
                raise RuntimeError("Go2 VideoClient 返回的数据不是完整 JPEG。")
            await websocket.send(payload)
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
