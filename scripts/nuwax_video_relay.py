#!/usr/bin/env python3
"""轻量视频中继：UDS JPEG 帧 → SHM（与 Rust video_relay.rs 协议一致）。

在 NUWAX_GEOMETRY_ENABLED=false 或 ROS2 桥接禁用时，由 start_gateway.sh 启动，
避免拉起完整 nuwax-ros-bridge（ROS2 在 G1 上易 OOM/崩溃，但视频仍需稳定）。
"""
from __future__ import annotations

import logging
import mmap
import os
import socket
import struct
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

LOGGER = logging.getLogger("nuwax_robot_bridge.video_relay")

SHM_MAGIC = 0x4E55_5741_5856_4944
SHM_VERSION = 1
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024
BUFFER_SIZE = HEADER_SIZE + MAX_FRAME_SIZE * 2


def _init_shm(path: Path) -> mmap.mmap:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb+") as handle:
        handle.truncate(BUFFER_SIZE)
    handle = open(path, "r+b")
    mm = mmap.mmap(handle.fileno(), BUFFER_SIZE, access=mmap.ACCESS_WRITE)
    handle.close()
    mm[0:8] = struct.pack("<Q", SHM_MAGIC)
    mm[8:12] = struct.pack("<I", SHM_VERSION)
    mm.flush()
    return mm


def _handle_connection(conn: socket.socket, mm: mmap.mmap, sequence: int) -> int:
    while True:
        header = conn.recv(4)
        if len(header) < 4:
            raise ConnectionError("推流端关闭连接")
        frame_len = struct.unpack("<I", header)[0]
        if frame_len <= 0 or frame_len > MAX_FRAME_SIZE:
            raise ValueError(f"帧大小无效: {frame_len}")

        sequence += 1
        active_buf = sequence % 2
        buf_offset = HEADER_SIZE + active_buf * MAX_FRAME_SIZE

        view = memoryview(mm)[buf_offset : buf_offset + frame_len]
        remaining = frame_len
        while remaining > 0:
            chunk = conn.recv_into(view, remaining)
            if chunk <= 0:
                raise ConnectionError("读取帧数据中断")
            remaining -= chunk

        now_ns = time.time_ns()
        mm[24:28] = struct.pack("<I", frame_len)
        mm[32:40] = struct.pack("<Q", now_ns)
        mm[40:44] = struct.pack("<I", active_buf)
        mm[16:24] = struct.pack("<Q", sequence)
        mm.flush()

    return sequence


def run_forever() -> None:
    shm_path = Path(os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm"))
    uds_path = Path(os.environ.get("NUWAX_VIDEO_UDS_PATH", "/tmp/nuwax_video.sock"))

    mm = _init_shm(shm_path)
    LOGGER.info("视频共享内存已创建: %s (%d bytes)", shm_path, BUFFER_SIZE)

    if uds_path.exists():
        uds_path.unlink()
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(uds_path))
    listener.listen(4)
    LOGGER.info("视频 UDS 监听: %s", uds_path)

    sequence = 0
    while True:
        conn, _ = listener.accept()
        conn.settimeout(None)
        LOGGER.info("视频推流器已连接")
        try:
            sequence = _handle_connection(conn, mm, sequence)
        except Exception as exc:
            LOGGER.info("视频连接断开: %s", exc)
        finally:
            conn.close()


def main() -> int:
    logging.basicConfig(
        level=os.environ.get("NUWAX_VIDEO_RELAY_LOG_LEVEL", "INFO"),
        format="%(levelname)s:%(name)s:%(message)s",
    )
    try:
        run_forever()
    except KeyboardInterrupt:
        return 0
    except Exception:
        LOGGER.exception("视频中继异常退出")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())