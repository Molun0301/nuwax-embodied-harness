from __future__ import annotations

import asyncio
import importlib
import logging
import os
from pathlib import Path
import sys
import time
from typing import Callable, Optional, TypeVar

from drivers.common.video_uds_sender import VideoUdsSender
from drivers.g1.settings import G1DataPlaneConfig, load_g1_data_plane_config

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.video_streamer")
T = TypeVar("T")

_DEFAULT_VIDEHUB_CYCLONEDDS_URI = "/unitree/module/video_hub_pc4/cyclonedds.xml"
_VIDEHUB_READY_TIMEOUT_SEC = float(
    os.environ.get("NUWAX_VIDEO_BACKEND_READY_TIMEOUT_SEC", os.environ.get("G1_VIDEHUB_READY_TIMEOUT_SEC", "60"))
)


class G1VideoStreamerConfigError(RuntimeError):
    """G1 端侧推流配置错误。"""


def _prepend_env_path(name: str, value: str) -> None:
    normalized = str(value or "").strip()
    if not normalized:
        return
    current = str(os.environ.get(name, "") or "").strip()
    entries = [item for item in current.split(":") if item.strip()]
    if normalized in entries:
        entries = [item for item in entries if item != normalized]
    os.environ[name] = ":".join([normalized, *entries]) if entries else normalized


def _resolve_cyclonedds_uri(config: G1DataPlaneConfig) -> str:
    configured = str(getattr(config, "cyclonedds_uri", "") or "").strip()
    if configured:
        return configured
    if Path(_DEFAULT_VIDEHUB_CYCLONEDDS_URI).is_file():
        return _DEFAULT_VIDEHUB_CYCLONEDDS_URI
    return ""


def _prepare_g1_sdk_environment(config: G1DataPlaneConfig) -> None:
    sdk_path = str(getattr(config, "sdk_path", "") or "").strip()
    if sdk_path:
        candidate = Path(sdk_path).expanduser()
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

    cyclonedds_lib_dir = str(getattr(config, "cyclonedds_lib_dir", "") or "").strip()
    if cyclonedds_lib_dir:
        _prepend_env_path("LD_LIBRARY_PATH", cyclonedds_lib_dir)

    cyclonedds_uri = _resolve_cyclonedds_uri(config)
    if cyclonedds_uri:
        os.environ["CYCLONEDDS_URI"] = cyclonedds_uri

    importlib.invalidate_caches()


def _initialize_channel(config: G1DataPlaneConfig) -> None:
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


def _create_video_client(config: G1DataPlaneConfig):
    _prepare_g1_sdk_environment(config)
    if not any(Path(path).expanduser().joinpath("unitree_sdk2py").exists() for path in sys.path):
        raise G1VideoStreamerConfigError(
            "未找到 unitree_sdk2py。请在 drivers/g1/data_plane.env 设置 G1_SDK_PATH，"
            "例如 G1_SDK_PATH=/home/unitree/unitree_sdk2_python。"
        )
    _initialize_channel(config)
    video_client_factory = getattr(
        importlib.import_module("unitree_sdk2py.go2.video.video_client"),
        "VideoClient",
    )
    client = video_client_factory()
    client.SetTimeout(float(getattr(config.control, "video_client_timeout_sec", 3.0)))
    client.Init()
    stabilize_sec = float(getattr(config.control, "post_init_stabilize_sec", 0.0))
    if stabilize_sec > 0:
        time.sleep(stabilize_sec)
    return client


def _extract_jpeg_payload(data: object) -> Optional[bytes]:
    raw = bytes(data or b"")
    soi = raw.find(b"\xff\xd8")
    eoi = raw.rfind(b"\xff\xd9")
    if soi < 0 or eoi <= soi:
        return None
    return raw[soi : eoi + 2]


def _wait_for_first_frame(client: object, *, timeout_sec: float) -> bytes:
    """等待 unitree_sdk2py VideoClient 首次成功取帧。"""
    deadline = time.monotonic() + max(1.0, timeout_sec)
    last_code = None
    while time.monotonic() < deadline:
        code, data = client.GetImageSample()
        last_code = code
        if code == 0 and data:
            payload = _extract_jpeg_payload(data)
            if payload is not None:
                LOGGER.info("G1 SDK VideoClient 首次取帧成功，JPEG %d bytes。", len(payload))
                return payload
        time.sleep(0.5)
    raise RuntimeError(
        "G1 SDK VideoClient.GetImageSample 在 %.0fs 内未就绪（最后 code=%s）。"
        "请确认 videohub_pc4 已启动且前置摄像头可用。"
        % (timeout_sec, last_code)
    )


async def _run_blocking(callable_obj: Callable[..., T], *args: object, **kwargs: object) -> T:
    to_thread = getattr(asyncio, "to_thread", None)
    if callable(to_thread):
        return await to_thread(callable_obj, *args, **kwargs)
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


async def _stream_once(config: G1DataPlaneConfig) -> None:
    stream_config = config.video_stream
    client = await _run_blocking(_create_video_client, config)
    ready_timeout = _VIDEHUB_READY_TIMEOUT_SEC
    await _run_blocking(_wait_for_first_frame, client, timeout_sec=ready_timeout)

    frame_interval_sec = 1.0 / max(1.0, float(stream_config.fps))

    uds_path = os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock")
    uds_timeout = float(os.environ.get("NUWAX_VIDEO_UDS_READY_TIMEOUT_SEC", "30"))
    uds_sender = VideoUdsSender(uds_path, reconnect_interval=1.0)
    if not await _run_blocking(uds_sender.wait_until_connected, uds_timeout):
        raise RuntimeError(
            f"无法连接 Rust 视频中转 UDS: {uds_path}（等待 {uds_timeout:.0f}s）"
        )
    LOGGER.info("G1 视频推流已连接 Rust UDS: %s", uds_path)

    consecutive_failures = 0
    while True:
        started_at = time.monotonic()
        code, data = await _run_blocking(client.GetImageSample)
        if code != 0 or not data:
            consecutive_failures += 1
            if consecutive_failures >= 30:
                raise RuntimeError(f"G1 VideoClient.GetImageSample 连续失败: code={code}")
            await asyncio.sleep(0.1)
            continue
        payload = _extract_jpeg_payload(data)
        if payload is None:
            consecutive_failures += 1
            if consecutive_failures >= 30:
                raise RuntimeError("G1 VideoClient 连续返回非完整 JPEG。")
            await asyncio.sleep(0.1)
            continue
        if not uds_sender.send_frame(payload):
            consecutive_failures += 1
            if consecutive_failures >= 10:
                raise RuntimeError("UDS 发送连续失败，连接已断开。")
            await asyncio.sleep(0.2)
            continue
        consecutive_failures = 0
        elapsed = time.monotonic() - started_at
        remaining = frame_interval_sec - elapsed
        if remaining > 0:
            await asyncio.sleep(remaining)


async def run_forever(config: G1DataPlaneConfig) -> None:
    if not config.video_stream.enabled:
        LOGGER.info("G1 端侧视频推流未启用。")
        return
    reconnect_sec = max(0.1, float(config.video_stream.reconnect_sec))
    while True:
        try:
            await _stream_once(config)
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception("G1 端侧视频推流中断，%.1fs 后重连。", reconnect_sec)
            await asyncio.sleep(reconnect_sec)


def main() -> None:
    logging.basicConfig(level=os.environ.get("G1_VIDEO_STREAM_LOG_LEVEL", "INFO"))
    config = load_g1_data_plane_config()
    asyncio.run(run_forever(config))


if __name__ == "__main__":
    main()