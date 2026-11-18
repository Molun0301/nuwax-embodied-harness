"""行为时间与样本来源时间。"""
from __future__ import annotations

import asyncio
import mmap
import os
import struct
import threading
import time

_CLOCK_SIZE = 40
_POLL_SEC = 0.010
_FIELDS = struct.Struct("<QdQ")
_SEQUENCE = struct.Struct("<Q")
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


def source_now_ns():
    """当前样本来源时刻，未配置设备时为系统 Unix 纳秒。"""
    return _device_sample()[0] if uses_device_clock() else time.time_ns()


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
