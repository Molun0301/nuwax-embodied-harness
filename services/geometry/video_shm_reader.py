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
    44-59: writer_generation (128-bit) — 全零表示旧写者
    60-63: flags (u32 LE)
  Buffer 0: [HEADER_SIZE .. HEADER_SIZE + MAX_FRAME_SIZE)
  Buffer 1: [HEADER_SIZE + MAX_FRAME_SIZE .. HEADER_SIZE + 2*MAX_FRAME_SIZE)
"""
from __future__ import annotations

import mmap
import os
import struct
import threading
import time
from pathlib import Path
from typing import Optional, Tuple, Union

SHM_MAGIC = 0x4E55_5741_5856_4944
HEADER_SIZE = 64
MAX_FRAME_SIZE = 512 * 1024  # 512KB
GENERATION_OFFSET = 44
GENERATION_SIZE = 16

_HEADER_FMT = "<QIIQIHHQI"  # magic, version, pad, seq, size, w, h, ts, buf_idx
_HEADER_STRUCT = struct.Struct(_HEADER_FMT)
_LEGACY_GENERATION = b"\x00" * GENERATION_SIZE


class VideoShmReader:
    """共享内存视频帧读取器。"""

    def __init__(self, shm_path: Union[str, Path]):
        self._path = Path(shm_path)
        self._lock = threading.RLock()
        self._mm: Optional[mmap.mmap] = None
        self._fd: Optional[int] = None
        self._backing_identity: Optional[Tuple[int, int, int]] = None
        self._last_identity: Optional[Tuple[bytes, int]] = None
        self._last_seq: int = 0
        self._next_mapping_check_at = 0.0

    def open(self) -> bool:
        with self._lock:
            self._next_mapping_check_at = 0.0
            return self._ensure_mapping()

    def close(self) -> None:
        with self._lock:
            self._drop_mapping(reset_identity=True)

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            # 解释器关闭期间模块状态可能已经部分销毁。
            pass

    @staticmethod
    def _close_resources(mm: Optional[mmap.mmap], fd: Optional[int]) -> None:
        if mm is not None:
            try:
                mm.close()
            except (BufferError, OSError, ValueError):
                pass
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _drop_mapping(self, *, reset_identity: bool) -> None:
        mm, fd = self._mm, self._fd
        self._mm = None
        self._fd = None
        self._backing_identity = None
        self._next_mapping_check_at = 0.0
        if reset_identity:
            self._last_identity = None
            self._last_seq = 0
        self._close_resources(mm, fd)

    def _ensure_mapping(self) -> bool:
        """把路径原子切换到当前 inode，并保留其 fd 与映射。"""
        now = time.monotonic()
        # 发布头逐帧读取，文件替换每250毫秒检查一次；正常取帧不反复进入文件系统。
        if self._fd is not None and self._backing_identity is not None:
            if now < self._next_mapping_check_at:
                return True
            try:
                stat_result = os.stat(self._path)
                current_identity = (
                    int(stat_result.st_dev),
                    int(stat_result.st_ino),
                    int(stat_result.st_size),
                )
            except OSError:
                self._drop_mapping(reset_identity=True)
                return False
            if current_identity == self._backing_identity:
                self._next_mapping_check_at = now + 0.25
                return True

        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        try:
            candidate_fd = os.open(self._path, flags)
        except OSError:
            self._drop_mapping(reset_identity=True)
            return False

        candidate_mm: Optional[mmap.mmap] = None
        try:
            stat_result = os.fstat(candidate_fd)
            candidate_identity = (
                int(stat_result.st_dev),
                int(stat_result.st_ino),
                int(stat_result.st_size),
            )
            if candidate_identity == self._backing_identity and self._fd is not None:
                self._close_resources(None, candidate_fd)
                return True
            if stat_result.st_size < HEADER_SIZE:
                raise ValueError("视频共享内存头部不完整")

            header = os.pread(candidate_fd, HEADER_SIZE, 0)
            if len(header) != HEADER_SIZE or struct.unpack_from("<Q", header, 0)[0] != SHM_MAGIC:
                raise ValueError("视频共享内存 magic 无效")
            candidate_mm = mmap.mmap(
                candidate_fd,
                int(stat_result.st_size),
                access=mmap.ACCESS_READ,
            )
        except (OSError, ValueError, struct.error):
            self._close_resources(candidate_mm, candidate_fd)
            self._drop_mapping(reset_identity=True)
            return False

        old_mm, old_fd = self._mm, self._fd
        self._mm = candidate_mm
        self._fd = candidate_fd
        self._backing_identity = candidate_identity
        self._next_mapping_check_at = now + 0.25
        self._last_identity = None
        self._last_seq = 0
        self._close_resources(old_mm, old_fd)
        return True

    @staticmethod
    def _parse_header(header: bytes) -> Optional[Tuple[bytes, int, int, int, int]]:
        if len(header) != HEADER_SIZE:
            return None
        try:
            magic, _, _, seq, frame_size, _, _, timestamp_ns, active_buf = (
                _HEADER_STRUCT.unpack_from(header)
            )
        except struct.error:
            return None
        if magic != SHM_MAGIC or seq == 0:
            return None
        if frame_size == 0 or frame_size > MAX_FRAME_SIZE or active_buf not in (0, 1):
            return None
        generation = header[GENERATION_OFFSET : GENERATION_OFFSET + GENERATION_SIZE]
        if generation == _LEGACY_GENERATION:
            generation = _LEGACY_GENERATION
        return generation, int(seq), int(frame_size), int(timestamp_ns), int(active_buf)

    def _bounds_are_valid(self, frame_size: int, active_buf: int) -> bool:
        identity = self._backing_identity
        if identity is None:
            return False
        buf_offset = HEADER_SIZE + active_buf * MAX_FRAME_SIZE
        return buf_offset <= identity[2] and frame_size <= identity[2] - buf_offset

    def read_latest(self) -> Optional[Tuple[bytes, int, int]]:
        """读取最新帧。返回 (jpeg_data, sequence, timestamp_ns) 或 None。

        seqlock 读侧校验：读 generation/seq(前)→读数据→读 generation/seq(后)。
        若读期间身份变化则重试，上限 3 次；旧写者的全零 generation 仍按合法代处理。
        """
        with self._lock:
            if not self._ensure_mapping():
                return None
            # 写者先创建定长文件再原子发布；重启替换 inode，不原地截断已发布文件。
            # 直接复制映射，避免三次 pread 释放解释器锁后被整图处理拖过一个视频周期。
            mm = self._mm
            for _ in range(3):
                parsed = self._parse_header(mm[:HEADER_SIZE])
                if parsed is None:
                    return None
                generation, seq, frame_size, timestamp_ns, active_buf = parsed
                frame_identity = (generation, seq)
                if frame_identity == self._last_identity:
                    return None
                if not self._bounds_are_valid(frame_size, active_buf):
                    return None

                buf_offset = HEADER_SIZE + active_buf * MAX_FRAME_SIZE
                data = mm[buf_offset : buf_offset + frame_size]
                parsed_after = self._parse_header(mm[:HEADER_SIZE])
                if parsed_after is None:
                    continue
                generation_after, seq_after, _, _, _ = parsed_after
                if generation_after != generation or seq_after != seq:
                    continue

                self._last_identity = frame_identity
                self._last_seq = seq
                return data, seq, timestamp_ns
            return None

    def has_new_frame(self) -> bool:
        """检查是否有新帧可用。"""
        with self._lock:
            if not self._ensure_mapping():
                return False
            # 就绪探测只读发布头；不因探测期间到达下一帧而误报视频不可用。
            parsed = self._parse_header(self._mm[:HEADER_SIZE])
            if parsed is None:
                return False
            generation, seq, frame_size, _, active_buf = parsed
            if not self._bounds_are_valid(frame_size, active_buf):
                return False
            return (generation, seq) != self._last_identity

    @property
    def last_sequence(self) -> int:
        with self._lock:
            return self._last_seq
