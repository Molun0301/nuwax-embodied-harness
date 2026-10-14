"""同宿主真实采集时间映射；只转换比较时间，不改来源样本时间。"""
from __future__ import annotations

import os
import struct
import time

_MAGIC = b'NEHTIME1'
_CAPACITY = 16384
_RECORD = struct.Struct('<QQQQ')


class SourceTimeError(RuntimeError):
    """已启用采集映射但对应样本不可用。"""


def acquired_times(source_stamp: int) -> tuple[int, int | None]:
    """返回真实采集(Unix, monotonic)；普通设备默认保留原Unix时间。"""
    path = os.environ.get('NUWAX_SOURCE_TIME_FILE', '')
    if not path:
        return source_stamp, None
    if not os.path.isabs(path):
        raise SourceTimeError('NUWAX_SOURCE_TIME_FILE必须为绝对路径')
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            header = os.pread(fd, 16, 0)
            if len(header) != 16 or header[:8] != _MAGIC:
                raise SourceTimeError('采集时间表格式无效')
            latest = struct.unpack_from('<Q', header, 8)[0]
            if latest == 0:
                raise SourceTimeError('采集时间表尚无记录')

            def read(sequence):
                offset = 16 + ((sequence - 1) % _CAPACITY) * _RECORD.size
                raw = os.pread(fd, _RECORD.size, offset)
                if len(raw) != _RECORD.size:
                    raise SourceTimeError('采集时间记录不完整')
                before, stamp, unix_ns, mono_ns = _RECORD.unpack(raw)
                after = os.pread(fd, 8, offset)
                if before != sequence or after != struct.pack('<Q', sequence):
                    raise SourceTimeError('采集时间记录正在更新或已被覆盖')
                return stamp, unix_ns, mono_ns

            oldest = max(1, latest - _CAPACITY + 1)
            last = read(latest)
            if source_stamp > last[0] + 1000:
                raise SourceTimeError('来源时间超出采集映射范围')
            # 仅容忍最后一个源时间的纳秒浮点舍入；其余取前一真实采集步。
            if source_stamp >= last[0]:
                return last[1], last[2]
            # 不预读最老槽；生产者可能正在复用它，近期查询无需接触。
            low, high = oldest, latest - 1
            match = None
            while low <= high:
                middle = (low + high) // 2
                current = read(middle)
                if current[0] <= source_stamp:
                    match = current
                    low = middle + 1
                else:
                    high = middle - 1
            if match is None:
                raise SourceTimeError('来源时间超出采集映射范围')
            return match[1], match[2]
        finally:
            os.close(fd)
    except OSError as exc:
        raise SourceTimeError('采集时间表不可读：' + str(exc)) from exc


def acquired_unix_ns(source_stamp: int) -> int:
    return acquired_times(source_stamp)[0]


def acquired_monotonic_ns(source_stamp: int) -> int | None:
    return acquired_times(source_stamp)[1]


def source_age_ns(source_stamp: int, *, now_unix_ns=None, now_monotonic_ns=None) -> int:
    unix_ns, mono_ns = acquired_times(source_stamp)
    if mono_ns is not None:
        return (time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns) - mono_ns
    return (time.time_ns() if now_unix_ns is None else now_unix_ns) - unix_ns


def acquired_after(source_stamp: int, *, issued_unix_ns: int, issued_monotonic_ns: int) -> bool:
    unix_ns, mono_ns = acquired_times(source_stamp)
    return mono_ns >= issued_monotonic_ns if mono_ns is not None else unix_ns >= issued_unix_ns
