"""行为时间与样本来源时间。"""
from __future__ import annotations

import asyncio
from ctypes import c_int64
import mmap
import os
import struct
import threading
import time
from functools import lru_cache

CLOCK_OFFSET_ENV = "NUWAX_DEVICE_CLOCK_OFFSET"
SOURCE_FUTURE_TOLERANCE_NS = 500_000_000
_CLOCK_SIZE = 40
_POLL_SEC = 0.010
_FIELDS = struct.Struct("<QdQ")
_SEQUENCE = struct.Struct("<Q")
# 单个自然对齐的主机字节序 i64，供同一 64 位宿主机上的进程共享。
_OFFSET = struct.Struct("@q")
_mapping = None
_mapping_lock = threading.Lock()


def uses_device_clock():
    return bool(os.environ.get("NUWAX_DEVICE_CLOCK"))


def _device_sample():
    global _mapping
    if _mapping is None:
        with _mapping_lock:
            if _mapping is None:
                path = os.environ["NUWAX_DEVICE_CLOCK"]
                while True:
                    try:
                        file = open(path, "rb")
                        break
                    except FileNotFoundError:
                        time.sleep(_POLL_SEC)
                with file:
                    _mapping = mmap.mmap(file.fileno(), _CLOCK_SIZE, access=mmap.ACCESS_READ)
    while True:
        before, = _SEQUENCE.unpack_from(_mapping, 8)
        if before & 1 == 0:
            sample = _FIELDS.unpack_from(_mapping, 16)
            after, = _SEQUENCE.unpack_from(_mapping, 8)
            if before == after:
                return sample


def initialize_source_clock_offset(path):
    """监督者在业务锁内创建本次业务使用的共享钟差。"""
    from core.config import atomic_write
    atomic_write(path, _OFFSET.pack(0))


@lru_cache(maxsize=2)
def _offset_mapping(path, access):
    with open(path, "r+b" if access == mmap.ACCESS_WRITE else "rb") as file:
        return mmap.mmap(file.fileno(), _OFFSET.size, access=access)


def publish_source_clock_offset_ns(path, offset_ns):
    """采集拥有方更新最近一次估计的有符号钟差。"""
    c_int64.from_buffer(_offset_mapping(path, mmap.ACCESS_WRITE)).value = offset_ns


def source_clock_offset_ns():
    """读取真机钟差，未启用钟差时为零。"""
    path = os.environ.get(CLOCK_OFFSET_ENV)
    return _OFFSET.unpack_from(_offset_mapping(path, mmap.ACCESS_READ))[0] if path else 0


def source_now_ns():
    """仿真使用平台时刻，真机使用以钟差对齐的 Unix 纳秒。"""
    return _device_sample()[0] if uses_device_clock() else time.time_ns() + source_clock_offset_ns()


def monotonic_ns():
    return _device_sample()[0] if uses_device_clock() else time.monotonic_ns()


def monotonic():
    return _device_sample()[0] / 1_000_000_000 if uses_device_clock() else time.monotonic()


def next_wait(deadline):
    """返回下一次真实等待秒数，到达行为期限时返回零。"""
    if not uses_device_clock():
        return max(0.0, deadline - time.monotonic())
    source_ns, rate, paused = _device_sample()
    remaining = deadline - source_ns / 1_000_000_000
    if remaining <= 0:
        return 0.0
    if paused:
        return _POLL_SEC
    if rate == 0:
        return 0.001
    return min(remaining / rate, _POLL_SEC)


def wait_until(deadline, wait):
    """等到行为期限，或由事件、条件变量提前唤醒。"""
    while True:
        seconds = next_wait(deadline)
        if wait(seconds):
            return True
        if seconds == 0:
            return False


def sleep(seconds):
    wait_until(monotonic() + seconds, time.sleep)


async def async_sleep(seconds):
    deadline = monotonic() + seconds
    while True:
        remaining = next_wait(deadline)
        if remaining == 0:
            return
        await asyncio.sleep(remaining)
