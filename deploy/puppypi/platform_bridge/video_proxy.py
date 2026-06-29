#!/usr/bin/env python3
"""PuppyPi 容器侧视频代理：ROS2 Image -> JPEG -> 共享内存。"""

from __future__ import annotations

import logging
import mmap
import os
import struct
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # type: ignore[assignment]


LOGGER = logging.getLogger("puppypi_video_proxy")

IMAGE_TOPIC = os.environ.get("PUPPYPI_IMAGE_TOPIC", "").strip()
SHM_PATH = os.environ.get("PUPPYPI_SHM_PATH", os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")).strip()
FPS = float(os.environ.get("PUPPYPI_VIDEO_FPS", "10"))
JPEG_QUALITY = int(os.environ.get("PUPPYPI_VIDEO_JPEG_QUALITY", os.environ.get("PUPPYPI_JPEG_QUALITY", "70")))
WIDTH = int(os.environ.get("PUPPYPI_VIDEO_WIDTH", "640"))
HEIGHT = int(os.environ.get("PUPPYPI_VIDEO_HEIGHT", "480"))
SHM_MAGIC = 0x4E55_5741_5856_4944
SHM_VERSION = 1
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024
BUFFER_SIZE = HEADER_SIZE + MAX_FRAME_SIZE * 2


class VideoShmWriter:
    """写入与 services.geometry.video_shm_reader 兼容的双缓冲 JPEG SHM。"""

    def __init__(self, shm_path: str) -> None:
        self._shm_path = shm_path
        self._file = None
        self._mm: Optional[mmap.mmap] = None
        self._sequence = 0

    def open(self) -> None:
        if self._mm is not None:
            return
        self._file = open(self._shm_path, "r+b")
        self._file.truncate(BUFFER_SIZE)
        self._mm = mmap.mmap(self._file.fileno(), BUFFER_SIZE, access=mmap.ACCESS_WRITE)
        self._mm[:HEADER_SIZE] = b"\x00" * HEADER_SIZE
        struct.pack_into("<QI", self._mm, 0, SHM_MAGIC, SHM_VERSION)
        self._mm.flush()

    def write_frame(self, jpeg_data: bytes, *, width: int, height: int) -> bool:
        self.open()
        if len(jpeg_data) == 0 or len(jpeg_data) > MAX_FRAME_SIZE:
            LOGGER.warning("JPEG 帧大小无效: %s bytes，最大 %s bytes", len(jpeg_data), MAX_FRAME_SIZE)
            return False

        mm = self._mm
        assert mm is not None
        seq = self._sequence + 1
        active_buffer = seq % 2
        buf_offset = HEADER_SIZE + active_buffer * MAX_FRAME_SIZE

        mm[buf_offset:buf_offset + len(jpeg_data)] = jpeg_data
        now_ns = time.time_ns()
        struct.pack_into("<IHHQI", mm, 24, len(jpeg_data), int(width), int(height), now_ns, active_buffer)
        # sequence 最后发布，读侧用它做 seqlock 双读校验。
        struct.pack_into("<Q", mm, 16, seq)
        self._sequence = seq
        mm.flush()
        return True

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None
        if self._file is not None:
            self._file.close()
            self._file = None


class VideoProxy:
    def __init__(self) -> None:
        self._bridge = CvBridge() if CvBridge is not None else None
        self._writer = VideoShmWriter(SHM_PATH)
        self._latest_jpeg: Optional[bytes] = None
        self._jpeg_lock = threading.Lock()
        self._jpeg_event = threading.Event()

    def run(self) -> None:
        if not IMAGE_TOPIC:
            LOGGER.error(
                "PUPPYPI_IMAGE_TOPIC 未设置，例如 PUPPYPI_IMAGE_TOPIC=/usb_cam/image_raw"
            )
            sys.exit(1)
        if not SHM_PATH:
            LOGGER.error("PUPPYPI_SHM_PATH/NUWAX_VIDEO_SHM_PATH 未设置")
            sys.exit(1)
        try:
            self._writer.open()
        except Exception:
            LOGGER.exception("初始化 SHM 失败: %s", SHM_PATH)
            sys.exit(1)

        rclpy.init()
        node = rclpy.create_node("puppypi_video_proxy")
        node.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        LOGGER.info(
            "Subscribing image topic=%s, writing JPEG to SHM %s @ %.1f fps",
            IMAGE_TOPIC,
            SHM_PATH,
            FPS,
        )

        interval = 1.0 / max(1.0, FPS)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not self._jpeg_event.wait(timeout=0.01):
                continue
            with self._jpeg_lock:
                frame = self._latest_jpeg
                self._jpeg_event.clear()
            if not frame:
                continue
            try:
                ok = self._writer.write_frame(frame, width=WIDTH, height=HEIGHT)
            except Exception:
                LOGGER.warning("SHM 写入失败，等待共享内存文件就绪: %s", SHM_PATH, exc_info=True)
                time.sleep(1.0)
                continue
            if not ok:
                continue
            time.sleep(interval)

    def _on_image(self, message: Image) -> None:
        try:
            image = self._image_message_to_bgr(message)
            resized = cv2.resize(image, (WIDTH, HEIGHT))
            ok, buffer = cv2.imencode(".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                return
            with self._jpeg_lock:
                self._latest_jpeg = buffer.tobytes()
            self._jpeg_event.set()
        except Exception:
            LOGGER.debug("图像转换失败", exc_info=True)

    def _image_message_to_bgr(self, message: Image):
        if self._bridge is not None:
            return self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

        if message.encoding not in ("bgr8", "rgb8"):
            raise ValueError(f"cv_bridge 不可用，暂不支持图像编码: {message.encoding}")
        channels = 3
        row_bytes = int(message.width) * channels
        data = np.frombuffer(message.data, dtype=np.uint8)
        if int(message.step) > row_bytes:
            data = data.reshape((int(message.height), int(message.step)))[:, :row_bytes]
        image = data.reshape((int(message.height), int(message.width), channels))
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image


def main() -> None:
    logging.basicConfig(level=os.environ.get("PUPPYPI_PROXY_LOG_LEVEL", "INFO"))
    VideoProxy().run()


if __name__ == "__main__":
    main()
