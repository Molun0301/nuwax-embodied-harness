"""Go2 视频推流进程直接发布 JPEG 共享内存，格式见 video_shm_reader.py。"""
from __future__ import annotations

import mmap
import os
from pathlib import Path
import struct
import threading
import time
from typing import Union
import uuid


SHM_MAGIC = 0x4E55_5741_5856_4944
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024
BUFFER_SIZE = HEADER_SIZE + 2 * MAX_FRAME_SIZE


class VideoShmWriter:
    """单写者双缓冲；先写数据和元信息，最后发布递增序号。"""

    def __init__(self, shm_path: Union[str, Path]) -> None:
        path = Path(shm_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        generation = uuid.uuid4()
        candidate = path.with_name(f".{path.name}.{generation.hex}")
        # 新写者用完整的新文件替换路径，避免截断读取方仍映射的文件。
        with candidate.open("w+b") as backing:
            backing.truncate(BUFFER_SIZE)
            self._mm = mmap.mmap(backing.fileno(), BUFFER_SIZE)
            struct.pack_into("<QI", self._mm, 0, SHM_MAGIC, 1)
            self._mm[44:60] = generation.bytes
            os.replace(candidate, path)
        self._sequence = 0
        self._lock = threading.Lock()

    def write_frame(self, jpeg_data: bytes, width: int, height: int) -> None:
        with self._lock:
            sequence = self._sequence + 1
            active_buffer = sequence % 2
            offset = HEADER_SIZE + active_buffer * MAX_FRAME_SIZE
            # 序号为零时读取方不采用；写完后一次发布新的非零序号。
            struct.pack_into("<Q", self._mm, 16, 0)
            self._mm[offset : offset + len(jpeg_data)] = jpeg_data
            struct.pack_into(
                "<IHHQI", self._mm, 24,
                len(jpeg_data), width, height, time.time_ns(), active_buffer,
            )
            struct.pack_into("<Q", self._mm, 16, sequence)
            self._sequence = sequence

    def close(self) -> None:
        with self._lock:
            self._mm.close()
