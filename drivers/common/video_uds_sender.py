"""视频帧 UDS 发送器 — 将 JPEG 帧推送到 Rust 视频中转进程。

协议：每帧 = [4字节 LE 长度][JPEG 数据]（与 Rust 视频中转进程约定，不含魔数/头部）。

历史上位于 services/geometry/video_uds_sender.py，实为端侧推流的纯传输工具，与平台几何
服务无业务关系，被错放在 services 下并由驱动 try/except 兜底引用，构成 drivers ⇄ services
循环。现迁至 drivers/common（发送端归端侧）。接收端为 Rust 中转进程，平台侧
services/geometry/video_shm_reader.py 走的是 SHM（64字节魔数头）另一套协议，二者无共享帧
格式常量，迁移无需提取公共契约。
"""
from __future__ import annotations

import socket
import struct
import time
import logging
from pathlib import Path
from typing import Optional

LOGGER = logging.getLogger(__name__)


class VideoUdsSender:
    """通过 Unix Domain Socket 发送视频帧到 Rust 中转进程。"""

    def __init__(self, uds_path: str | Path, reconnect_interval: float = 2.0):
        self._path = str(uds_path)
        self._sock: Optional[socket.socket] = None
        self._reconnect_interval = reconnect_interval
        self._last_connect_attempt = 0.0
        self._frames_sent = 0

    def connect(self) -> bool:
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_interval:
            return False
        self._last_connect_attempt = now
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect(self._path)
            sock.settimeout(None)
            self._sock = sock
            LOGGER.info("已连接到 Rust 视频中转: %s", self._path)
            return True
        except (OSError, ConnectionRefusedError):
            return False

    def send_frame(self, jpeg_data: bytes) -> bool:
        """发送一帧 JPEG 数据。返回是否成功。"""
        if self._sock is None:
            if not self.connect():
                return False
        try:
            header = struct.pack("<I", len(jpeg_data))
            self._sock.sendall(header + jpeg_data)
            self._frames_sent += 1
            return True
        except (OSError, BrokenPipeError):
            self.close()
            return False

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def frames_sent(self) -> int:
        return self._frames_sent
