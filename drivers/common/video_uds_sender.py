"""视频帧 UDS 发送器 — 将 JPEG 帧推送到 Rust 视频中转进程。

协议：每帧 = [4字节 LE 长度][JPEG 数据]（与 Rust 视频中转进程约定，不含魔数/头部）。

历史上位于 services/geometry/video_uds_sender.py，实为端侧推流的纯传输工具，与平台几何
服务无业务关系，被错放在 services 下并由驱动 try/except 兜底引用，构成 drivers ⇄ services
循环。现迁至 drivers/common（发送端归端侧）。接收端为 Rust 中转进程，平台侧
services/geometry/video_shm_reader.py 走的是 SHM（64字节魔数头）另一套协议，二者无共享帧
格式常量，迁移无需提取公共契约。
"""
from __future__ import annotations

import logging
import socket
import struct
import time
from pathlib import Path
from typing import Optional, Union

LOGGER = logging.getLogger(__name__)


class VideoUdsSender:
    """通过 Unix Domain Socket 发送视频帧到 Rust 中转进程。

    发送由推流协程的单个线程池任务串行驱动，close() 由事件循环触发。二者之间
    只需要一个终态标志：close() 后不再重连、不再发送；在途写入由 shutdown()
    打断，失败按普通 I/O 错误回收。不需要额外加锁。
    """

    def __init__(self, uds_path: Union[str, Path], reconnect_interval: float = 2.0):
        self._path = str(uds_path)
        self._sock: Optional[socket.socket] = None
        self._reconnect_interval = reconnect_interval
        self._last_connect_attempt = 0.0
        self._closed = False
        # 复用长度头，sendmsg 两段直发，省掉每帧一份完整 JPEG 拼接副本。
        self._length_header = bytearray(4)
        self._frames_sent = 0

    def connect(self) -> bool:
        if self._closed:
            return False
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_interval:
            return False
        self._last_connect_attempt = now
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect(self._path)
            sock.settimeout(None)
        except (OSError, ConnectionRefusedError):
            return False
        if self._closed:
            # close() 发生在连接握手期间，直接丢弃这条连接。
            self._discard(sock)
            return False
        self._sock = sock
        LOGGER.info("已连接到 Rust 视频中转: %s", self._path)
        return True

    def send_frame(self, jpeg_data: bytes) -> bool:
        """发送一帧 JPEG 数据。返回是否成功。"""
        if self._closed:
            return False
        sock = self._sock
        if sock is None:
            if not self.connect():
                return False
            sock = self._sock
            if sock is None:
                return False
        try:
            self._write_frame(sock, jpeg_data)
        except (OSError, BrokenPipeError):
            # 发送失败只回收当前连接，后续仍可按间隔重连；close() 才是终态。
            if self._sock is sock:
                self._sock = None
            self._discard(sock)
            return False
        self._frames_sent += 1
        return True

    def _write_frame(self, sock: socket.socket, jpeg_data: bytes) -> None:
        """写出 [4字节 LE 长度][JPEG]，不产生中间拼接副本。"""
        struct.pack_into("<I", self._length_header, 0, len(jpeg_data))
        sendmsg = getattr(sock, "sendmsg", None)
        if not callable(sendmsg):
            # 非 Unix 平台或测试替身的兼容回退。
            sock.sendall(bytes(self._length_header) + jpeg_data)
            return
        pending = [memoryview(self._length_header), memoryview(jpeg_data)]
        try:
            while pending:
                sent = sendmsg(pending)
                if sent <= 0:
                    raise OSError("视频 UDS 连接未发送数据")
                while pending and sent >= len(pending[0]):
                    sent -= len(pending[0])
                    pending.pop(0).release()
                if pending and sent:
                    view = pending[0]
                    pending[0] = view[sent:]
                    view.release()
        finally:
            for view in pending:
                view.release()

    @staticmethod
    def _discard(sock: socket.socket) -> None:
        """shutdown 打断在途写入，再关闭套接字。"""
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except (OSError, AttributeError):
            pass
        try:
            sock.close()
        except OSError:
            pass

    def close(self) -> None:
        self._closed = True
        sock = self._sock
        self._sock = None
        if sock is not None:
            self._discard(sock)

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def frames_sent(self) -> int:
        return self._frames_sent
