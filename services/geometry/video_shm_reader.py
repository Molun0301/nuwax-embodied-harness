"""从 Rust 视频中转进程的共享内存读取 JPEG 帧。

共享内存布局（与 Rust video_relay.rs 对应）：
  Header (64 bytes):
    0-7:   magic (u64 LE) = 0x4E55_5741_5856_4944
    8-11:  version (u32 LE) = 1
    12-15: pad
    16-23: sequence (u64 LE) — 每帧递增
    24-27: frame_size (u32 LE)
    28-29: width (u16 LE)
    30-31: height (u16 LE)
    32-39: timestamp_ns (u64 LE)
    40-43: active_buffer (u32 LE) — 0 or 1
    44-63: reserved
  Buffer 0: [HEADER_SIZE .. HEADER_SIZE + MAX_FRAME_SIZE)
  Buffer 1: [HEADER_SIZE + MAX_FRAME_SIZE .. HEADER_SIZE + 2*MAX_FRAME_SIZE)
"""
from __future__ import annotations

import mmap
import struct
import time
from pathlib import Path
from typing import Optional, Tuple, Union

SHM_MAGIC = 0x4E55_5741_5856_4944
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024  # 512KB

_HEADER_FMT = "<QIIQIHHQI"  # magic, version, pad, seq, size, w, h, ts, buf_idx
_HEADER_STRUCT = struct.Struct(_HEADER_FMT)


class VideoShmReader:
    """零拷贝共享内存视频帧读取器。"""

    def __init__(self, shm_path: Union[str, Path]):
        self._path = Path(shm_path)
        self._mm: Optional[mmap.mmap] = None
        self._last_seq: int = 0

    def open(self) -> bool:
        if self._mm is not None:
            return True
        if not self._path.exists():
            return False
        try:
            f = open(self._path, "r+b")
            self._mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            f.close()
            magic = struct.unpack_from("<Q", self._mm, 0)[0]
            if magic != SHM_MAGIC:
                self.close()
                return False
            return True
        except (OSError, ValueError):
            return False

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def read_latest(self) -> Optional[Tuple[bytes, int, int]]:
        """读取最新帧。返回 (jpeg_data, sequence, timestamp_ns) 或 None。

        seqlock 读侧校验：读 seq(前)→读数据→读 seq(后)，若读期间 seq 变化（写者已发布新帧）
        则重试，上限 3 次。与 Rust 写侧 fence(Release)（video_relay.rs）配套，消除撕裂读。
        """
        if self._mm is None and not self.open():
            return None
        mm = self._mm
        assert mm is not None

        for _ in range(3):
            seq = struct.unpack_from("<Q", mm, 16)[0]
            if seq == 0 or seq == self._last_seq:
                return None

            frame_size = struct.unpack_from("<I", mm, 24)[0]
            timestamp_ns = struct.unpack_from("<Q", mm, 32)[0]
            active_buf = struct.unpack_from("<I", mm, 40)[0]

            if frame_size == 0 or frame_size > MAX_FRAME_SIZE:
                return None

            buf_offset = HEADER_SIZE + int(active_buf) * MAX_FRAME_SIZE
            data = bytes(mm[buf_offset:buf_offset + frame_size])

            # Verify sequence didn't change during read (torn read check)
            seq_after = struct.unpack_from("<Q", mm, 16)[0]
            if seq_after == seq:
                self._last_seq = seq
                return (data, seq, timestamp_ns)
            # seq 在读取期间被写者更新，重试读取最新帧。
        # 连续 3 次都被写者抢占，放弃本轮（上层轮询下次重试）。
        return None

    def has_new_frame(self) -> bool:
        """检查是否有新帧可用。"""
        if self._mm is None and not self.open():
            return False
        seq = struct.unpack_from("<Q", self._mm, 16)[0]
        return seq > 0 and seq != self._last_seq

    @property
    def last_sequence(self) -> int:
        return self._last_seq
