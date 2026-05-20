"""从 Rust 点云桥的共享内存读取处理后的点云数据（零拷贝）。

共享内存布局（与 Rust pointcloud.rs PointCloudShmWriter 对应）：
  Header (32 bytes):
    0-7:   magic (u64 LE) = 0x4E55_5741_5850_434C ("NUWAXPCL")
    8-11:  version (u32 LE) = 1
    12-15: point_count (u32 LE)
    16-23: timestamp_ns (u64 LE)
    24-31: sequence (u64 LE) — 每帧递增
  Data:
    [32 .. 32 + point_count * 12) — float32 XYZ 连续布局
"""
from __future__ import annotations

import mmap
import struct
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

PC_SHM_MAGIC = 0x4E55_5741_5850_434C
PC_HEADER_SIZE = 32
PC_MAX_POINTS = 4096


class PointCloudShmReader:
    """零拷贝共享内存点云读取器。"""

    def __init__(self, shm_path: str | Path = "/tmp/nuwax_pointcloud.shm"):
        self._path = Path(shm_path)
        self._mm: Optional[mmap.mmap] = None
        self._last_seq: int = 0

    def open(self) -> bool:
        """打开共享内存文件。成功返回 True。"""
        if self._mm is not None:
            return True
        if not self._path.exists():
            return False
        try:
            f = open(self._path, "r+b")
            self._mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            f.close()
            magic = struct.unpack_from("<Q", self._mm, 0)[0]
            if magic != PC_SHM_MAGIC:
                self.close()
                return False
            return True
        except (OSError, ValueError):
            return False

    def close(self) -> None:
        """关闭共享内存映射。"""
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def read_latest(self) -> Optional[Tuple[np.ndarray, int, int]]:
        """读取最新点云帧。

        Returns:
            (points_xyz: ndarray(N,3) float32, timestamp_ns: int, sequence: int)
            如果无新帧或不可用则返回 None。
        """
        if self._mm is None and not self.open():
            return None
        mm = self._mm
        assert mm is not None

        # 读取 sequence（头部最后写入，作为数据完整性标志）
        seq = struct.unpack_from("<Q", mm, 24)[0]
        if seq == 0 or seq == self._last_seq:
            return None

        point_count = struct.unpack_from("<I", mm, 12)[0]
        timestamp_ns = struct.unpack_from("<Q", mm, 16)[0]

        if point_count == 0 or point_count > PC_MAX_POINTS:
            return None

        data_size = point_count * 12  # 3 × float32
        data_offset = PC_HEADER_SIZE
        # 零拷贝：直接从 mmap buffer 构造 ndarray
        buf = mm[data_offset:data_offset + data_size]
        points = np.frombuffer(buf, dtype=np.float32).reshape(-1, 3).copy()

        # Torn read 检查：确认读取期间 sequence 未变
        seq_after = struct.unpack_from("<Q", mm, 24)[0]
        if seq_after != seq:
            return None

        self._last_seq = seq
        return (points, timestamp_ns, seq)

    def has_new_frame(self) -> bool:
        """检查是否有新帧可用。"""
        if self._mm is None and not self.open():
            return False
        seq = struct.unpack_from("<Q", self._mm, 24)[0]
        return seq > 0 and seq != self._last_seq

    @property
    def last_sequence(self) -> int:
        return self._last_seq
