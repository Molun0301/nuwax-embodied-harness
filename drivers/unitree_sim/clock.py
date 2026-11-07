"""NEH 驱动拥有的真实采集时间配对表；来源采样时间保持不变。"""
from __future__ import annotations
import os
from pathlib import Path
import struct


def source_time_path(home):
    return Path(home) / "run/unitree_sim_source_time"


def source_timestamp(payload, sim_stamp):
    mapping = payload['source_clock_mapping']
    if mapping['mode'] != 'realtime':
        raise ValueError('离线时钟不能作为实时设备输入。')
    return mapping['unix_anchor_ns'] + sim_stamp - mapping['sim_anchor_ns']


class CaptureTimes:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fd = os.open(str(path), os.O_CREAT | os.O_TRUNC | os.O_WRONLY, 0o600)
        os.ftruncate(self.fd, 16 + 16384 * 32)
        os.pwrite(self.fd, b"NEHTIME1" + struct.pack('<Q', 0), 0)
        self.sequence = 0
        self.latest = -1

    def record(self, payload, stamp):
        if stamp <= self.latest:
            return
        sequence = self.sequence + 1
        offset = 16 + ((sequence - 1) % 16384) * 32
        os.pwrite(self.fd, struct.pack('<Q', 0), offset)
        os.pwrite(self.fd, struct.pack('<QQQ', stamp, payload['acquisition_unix_ns'],
                                     payload['acquisition_monotonic_ns']), offset + 8)
        os.pwrite(self.fd, struct.pack('<Q', sequence), offset)
        os.pwrite(self.fd, struct.pack('<Q', sequence), 8)
        self.sequence, self.latest = sequence, stamp

    def close(self):
        os.close(self.fd)
