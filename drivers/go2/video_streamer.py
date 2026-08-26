from __future__ import annotations

import asyncio
import importlib
import importlib.metadata
import logging
import math
import os
import re
from typing import Any, Callable, Optional, Tuple, TypeVar

from drivers.common.video_uds_sender import VideoUdsSender
from drivers.go2.settings import Go2DataPlaneConfig, load_go2_data_plane_config


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.video_streamer")
T = TypeVar("T")
_FIRST_FRAME_TIMEOUT_SEC = 15.0
_CONSUMER_SHUTDOWN_TIMEOUT_SEC = 2.0
_MAX_JPEG_PAYLOAD_BYTES = 512 * 1024
_JPEG_QUALITY_STEPS = (90, 80, 70, 60, 50, 40, 30, 20)
_AES_128_KEY_PATTERN = re.compile(r"^[0-9a-fA-F]{32}$")
_AES_KEY_REQUIRED_TEXT = "Go2 WebRTC AES-128 密钥缺失。"
_AES_KEY_REJECTED_TEXT = "Go2 WebRTC AES-128 密钥被拒绝。"


class Go2VideoStreamerConfigError(RuntimeError):
    """Go2 端侧推流配置错误。"""


def _load_webrtc_api() -> Tuple[Callable[..., Any], Any, Tuple[type, type]]:
    try:
        installed_version = importlib.metadata.version("unitree-webrtc-connect")
        if installed_version != "2.2.0":
            raise RuntimeError(
                "unitree-webrtc-connect 版本不匹配："
                f"required=2.2.0 actual={installed_version}"
            )
        module = importlib.import_module("unitree_webrtc_connect")
        connection_factory = getattr(module, "UnitreeWebRTCConnection")
        connection_method = getattr(module, "WebRTCConnectionMethod")
        local_sta = getattr(connection_method, "LocalSTA")
        fatal_auth_errors = (
            getattr(module, "AesKeyRequiredError"),
            getattr(module, "AesKeyRejectedError"),
        )
    except Exception as exc:
        raise Go2VideoStreamerConfigError(
            "无法加载 unitree-webrtc-connect==2.2.0 的官方 WebRTC API。"
            "请通过 Go2 目标部署 profile 安装 Python 依赖，并确认系统包 "
            "python3-pyaudio 与 portaudio19-dev 已安装。"
            f"原始错误: {exc}"
        ) from exc
    return connection_factory, local_sta, fatal_auth_errors


def _create_webrtc_connection(
    config: Go2DataPlaneConfig,
) -> Tuple[Any, Tuple[type, type]]:
    stream_config = config.video_stream
    robot_ip = str(stream_config.robot_ip or "").strip()
    if not robot_ip:
        raise Go2VideoStreamerConfigError(
            "未配置 Go2 WebRTC 机器人地址。请在 drivers/go2/data_plane.env "
            "设置 GO2_ROBOT_IP，或设置通用 ROBOT_IP。"
        )

    aes_128_key = str(stream_config.aes_128_key or "").strip()
    if aes_128_key and _AES_128_KEY_PATTERN.fullmatch(aes_128_key) is None:
        raise Go2VideoStreamerConfigError(
            "GO2_WEBRTC_AES_128_KEY 必须留空或填写 32 位十六进制 AES-128 key。"
        )

    connection_factory, local_sta, fatal_auth_errors = _load_webrtc_api()
    connection = connection_factory(
        local_sta,
        ip=robot_ip,
        aes_128_key=aes_128_key or None,
        device_type="Go2",
    )
    return connection, fatal_auth_errors


def _encode_jpeg_frame(frame: Any) -> bytes:
    try:
        cv2 = importlib.import_module("cv2")
        image = frame.to_ndarray(format="bgr24")
        if getattr(image, "ndim", 0) < 2:
            raise ValueError("视频帧没有有效的二维尺寸")

        while True:
            smallest_payload: Optional[bytes] = None
            for quality in _JPEG_QUALITY_STEPS:
                encoded_ok, encoded = cv2.imencode(
                    ".jpg",
                    image,
                    (int(cv2.IMWRITE_JPEG_QUALITY), quality),
                )
                if not encoded_ok:
                    continue
                payload = encoded.tobytes()
                if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
                    raise RuntimeError("WebRTC 视频帧编码结果不是完整 JPEG。")
                if len(payload) <= _MAX_JPEG_PAYLOAD_BYTES:
                    return payload
                if smallest_payload is None or len(payload) < len(smallest_payload):
                    smallest_payload = payload

            if smallest_payload is None:
                raise RuntimeError("WebRTC 视频帧 JPEG 编码失败。")

            height, width = int(image.shape[0]), int(image.shape[1])
            if height <= 1 and width <= 1:
                raise RuntimeError(
                    "WebRTC 视频帧无法编码到 512 KiB UDS 载荷上限以内。"
                )
            ratio = math.sqrt(_MAX_JPEG_PAYLOAD_BYTES / float(len(smallest_payload)))
            scale = min(0.85, max(0.10, ratio * 0.90))
            resized_width = max(1, int(width * scale))
            resized_height = max(1, int(height * scale))
            if resized_width == width and resized_height == height:
                if width >= height and width > 1:
                    resized_width -= 1
                elif height > 1:
                    resized_height -= 1
            image = cv2.resize(
                image,
                (resized_width, resized_height),
                interpolation=cv2.INTER_AREA,
            )
    except Exception as exc:
        if isinstance(exc, RuntimeError):
            raise
        raise RuntimeError(f"WebRTC 视频帧转换或 JPEG 编码失败: {exc}") from exc


async def _run_blocking(callable_obj: Callable[..., T], *args: object, **kwargs: object) -> T:
    """在兼容 Python 3.8 的线程池中执行阻塞调用。"""

    to_thread = getattr(asyncio, "to_thread", None)
    if callable(to_thread):
        return await to_thread(callable_obj, *args, **kwargs)
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


async def _consume_video_track(
    track: Any,
    uds_sender: VideoUdsSender,
    first_frame: asyncio.Event,
) -> None:
    try:
        while True:
            frame = await track.recv()
            payload = await _run_blocking(_encode_jpeg_frame, frame)
            if len(payload) > _MAX_JPEG_PAYLOAD_BYTES:
                raise RuntimeError("JPEG 帧超过 512 KiB UDS 载荷上限。")
            if not await _run_blocking(uds_sender.send_frame, payload):
                raise RuntimeError("UDS 发送失败，Rust 视频中转连接已断开。")
            first_frame.set()
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Go2 WebRTC 视频轨道中断: {exc}") from exc


async def _cancel_and_wait_consumer(task: asyncio.Task) -> bool:
    if not task.done():
        task.cancel()
    done, _ = await asyncio.wait(
        (task,),
        timeout=_CONSUMER_SHUTDOWN_TIMEOUT_SEC,
    )
    if task not in done:
        LOGGER.error(
            "Go2 WebRTC consumer Task 在 %.1f 秒内未停止。",
            _CONSUMER_SHUTDOWN_TIMEOUT_SEC,
        )
        return False
    await asyncio.gather(task, return_exceptions=True)
    return True


async def _stream_once(config: Go2DataPlaneConfig) -> None:
    stream_config = config.video_stream
    uds_path = os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock")
    uds_sender = VideoUdsSender(
        uds_path,
        reconnect_interval=max(0.1, float(stream_config.reconnect_sec)),
    )
    connection = None
    video_channel = None
    channel_enabled = False
    accepting_track = True
    consumer_task: Optional[asyncio.Task] = None
    first_frame_waiter: Optional[asyncio.Task] = None

    try:
        connection, fatal_auth_errors = _create_webrtc_connection(config)
        if not await _run_blocking(uds_sender.connect):
            raise RuntimeError(f"无法连接 Rust 视频中转 UDS: {uds_path}")
        LOGGER.info("视频推流已连接 Rust UDS: %s", uds_path)

        try:
            await connection.connect()
        except fatal_auth_errors as exc:
            required_error, rejected_error = fatal_auth_errors
            if isinstance(exc, required_error):
                raise Go2VideoStreamerConfigError(_AES_KEY_REQUIRED_TEXT) from None
            if isinstance(exc, rejected_error):
                raise Go2VideoStreamerConfigError(_AES_KEY_REJECTED_TEXT) from None
            raise Go2VideoStreamerConfigError(_AES_KEY_REJECTED_TEXT) from None

        video_channel = getattr(connection, "video", None)
        add_track_callback = getattr(video_channel, "add_track_callback", None)
        switch_video_channel = getattr(video_channel, "switchVideoChannel", None)
        if not callable(add_track_callback) or not callable(switch_video_channel):
            raise Go2VideoStreamerConfigError(
                "unitree-webrtc-connect 视频 API 不兼容：缺少 "
                "video.add_track_callback 或 video.switchVideoChannel。"
            )

        loop = asyncio.get_running_loop()
        first_frame = asyncio.Event()
        track_registered = asyncio.Event()
        first_frame_deadline = loop.time() + _FIRST_FRAME_TIMEOUT_SEC

        async def receive_track(track: Any) -> None:
            nonlocal consumer_task
            if not accepting_track or consumer_task is not None:
                return
            consumer_task = asyncio.create_task(
                _consume_video_track(track, uds_sender, first_frame),
                name="go2-video-track-consumer",
            )
            track_registered.set()

        add_track_callback(receive_track)
        switch_video_channel(True)
        channel_enabled = True

        try:
            await asyncio.wait_for(
                track_registered.wait(),
                timeout=max(0.0, first_frame_deadline - loop.time()),
            )
        except asyncio.TimeoutError:
            raise RuntimeError(
                f"Go2 WebRTC 已连接，但 {_FIRST_FRAME_TIMEOUT_SEC:.0f} 秒内未收到视频轨道。"
            ) from None
        if consumer_task is None:
            raise RuntimeError("Go2 WebRTC 视频轨道未登记 consumer Task。")

        first_frame_waiter = asyncio.create_task(first_frame.wait())
        done, _ = await asyncio.wait(
            (first_frame_waiter, consumer_task),
            timeout=max(0.0, first_frame_deadline - loop.time()),
            return_when=asyncio.FIRST_COMPLETED,
        )
        if consumer_task in done:
            await consumer_task
        if first_frame_waiter not in done:
            raise RuntimeError(
                f"Go2 WebRTC 已连接，但 {_FIRST_FRAME_TIMEOUT_SEC:.0f} 秒内未收到视频帧。"
            )
        await first_frame_waiter
        LOGGER.info("Go2 WebRTC 视频轨道已开始持续推流: robot_ip=%s", stream_config.robot_ip)
        await consumer_task
    finally:
        accepting_track = False
        if first_frame_waiter is not None and not first_frame_waiter.done():
            first_frame_waiter.cancel()
            await asyncio.gather(first_frame_waiter, return_exceptions=True)
        consumer_stopped = True
        if consumer_task is not None:
            consumer_stopped = await _cancel_and_wait_consumer(consumer_task)
        if channel_enabled and video_channel is not None:
            try:
                video_channel.switchVideoChannel(False)
            except Exception:
                LOGGER.warning("关闭 Go2 WebRTC 视频通道失败。", exc_info=True)
        if connection is not None:
            disconnect = getattr(connection, "disconnect", None)
            if callable(disconnect):
                try:
                    await disconnect()
                except Exception:
                    LOGGER.warning("断开 Go2 WebRTC 连接失败。", exc_info=True)
        uds_sender.close()
        if not consumer_stopped and consumer_task is not None:
            done, _ = await asyncio.wait(
                (consumer_task,),
                timeout=_CONSUMER_SHUTDOWN_TIMEOUT_SEC,
            )
            if consumer_task in done:
                await asyncio.gather(consumer_task, return_exceptions=True)
            else:
                LOGGER.error("关闭 UDS 后 Go2 WebRTC consumer Task 仍未停止。")


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
        except Go2VideoStreamerConfigError:
            LOGGER.exception("Go2 端侧视频推流配置或依赖错误，停止进程。")
            raise
        except Exception:
            LOGGER.exception("Go2 端侧视频推流中断，%.1fs 后重连。", reconnect_sec)
            await asyncio.sleep(reconnect_sec)


def main() -> None:
    logging.basicConfig(level=os.environ.get("GO2_VIDEO_STREAM_LOG_LEVEL", "INFO"))
    config = load_go2_data_plane_config()
    try:
        asyncio.run(run_forever(config))
    except Go2VideoStreamerConfigError:
        return


if __name__ == "__main__":
    main()
