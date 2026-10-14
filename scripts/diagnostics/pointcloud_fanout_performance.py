#!/usr/bin/env python3
"""实际点云 WS 发送/编码方法的离线配对测量；socket/source 为合成边界。

不测网络、机器人或 Rust 采集。每轮使用新 cursor，因此复用路径也必须完成
一次完整验证/编码；不以预热缓存命中伪装单客户端改善。每轮只保留一个路径的
输出，路径次序交替；缓存驱逐成本包含在测量内。运行前应协调独占短基准窗口。
"""
from __future__ import annotations

import argparse
import asyncio
import json
import platform
import statistics
import sys
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np

from contracts.streams import StreamFrame, StreamPayloadKind
from gateways.ws import data_streams as ws
from gateways.ws.pointcloud_encoding import CanonicalPointCloudEncoder


class Socket:
    def __init__(self):
        self.payload = None

    async def send_bytes(self, value):
        self.payload = value

    async def send_text(self, value):
        self.payload = value


def frame(raw, cursor):
    return StreamFrame(topic='pointcloud_map.snapshot', cursor=cursor, source='offline-probe',
                       payload_kind=StreamPayloadKind.BINARY, payload=raw, frame_id='map', revision='cloud:7',
                       metadata={'source': 'live', 'source_epoch': 'offline-epoch', 'stream_generation': 2,
                                     'revision': 'cloud:7', 'revision_ordinal': 7, 'frame_id': 'map',
                                     'source_timestamp_ns': 123456789, 'point_count': len(raw)//12,
                                     'format': 'float32-xyz-le'})


async def sample(value, clients, owner, expected=None):
    sockets = [Socket() for _ in range(clients)]
    cpu = time.process_time_ns()
    wall = time.perf_counter_ns()
    await asyncio.gather(*(ws._send_ws_frame(socket, value, pointcloud_encoder=owner) for socket in sockets))
    elapsed_wall = (time.perf_counter_ns()-wall)/1e6
    elapsed_cpu = (time.process_time_ns()-cpu)/1e6
    # Checks are deliberately outside the timed region.
    assert all(socket.payload == sockets[0].payload for socket in sockets)
    if expected is not None:
        assert sockets[0].payload == expected
    result = {'wall_ms': elapsed_wall, 'process_cpu_ms': elapsed_cpu,
                  'unique_output_buffers': len({id(socket.payload) for socket in sockets}),
                  'unique_output_bytes': sum(len(value) for value in {id(s.payload): s.payload for s in sockets}.values()),
                  'packet_bytes': len(sockets[0].payload)}
    return result


def summarize(values):
    result = {}
    for name in ('wall_ms', 'process_cpu_ms'):
        series = [value[name] for value in values]
        result[name] = {'median': statistics.median(series), 'minimum': min(series), 'maximum': max(series)}
    result['unique_output_buffers'] = values[-1]['unique_output_buffers']
    result['unique_output_bytes'] = values[-1]['unique_output_bytes']
    result['packet_bytes'] = values[-1]['packet_bytes']
    return result


async def main(args):
    sources = [ROOT/'gateways/ws/data_streams.py', ROOT/'gateways/ws/pointcloud_encoding.py', Path(__file__)]
    report = {'scope': 'synthetic source/socket, production WS encode/send; no network or Go2 CPU attribution',
                  'python': sys.version, 'numpy': np.__version__, 'platform': platform.platform(), 'samples': args.samples,
                  'source_files': {str(path.relative_to(ROOT)): {'bytes': path.stat().st_size,
                                      'mtime_ns': path.stat().st_mtime_ns} for path in sources}, 'cases': []}
    for points in (1000, 60000, 180000, 1000000):
        raw = np.arange(points*3, dtype='<f4').tobytes()
        for clients in (1, 4, 8):
            owner = CanonicalPointCloudEncoder(ws._encode_ws_frame, ws._run_blocking)
            samples = {'baseline': [], 'reuse': []}
            try:
                # Warm both execution routes; timed cursors differ from these and each other.
                for index in range(2):
                    await sample(frame(raw, index+1), clients, None)
                    await sample(frame(raw, index+1), clients, owner)
                for index in range(args.samples):
                    current = frame(raw, index+100)
                    expected = await ws._run_blocking(ws._encode_ws_frame, current)
                    order = ('baseline', 'reuse') if index % 2 == 0 else ('reuse', 'baseline')
                    for name in order:
                        samples[name].append(await sample(current, clients, owner if name == 'reuse' else None, expected))
                scan_counts = {}
                for name in ('baseline', 'reuse'):
                    finite = np.isfinite
                    with patch.object(ws.np, 'isfinite', wraps=finite) as scan:
                        await sample(frame(raw, 10000), clients, owner if name == 'reuse' else None)
                        scan_counts[name] = scan.call_count
                assert scan_counts == {'baseline': clients, 'reuse': 1}
                case = {'points': points, 'clients': clients, 'summaries': {name: summarize(values)
                            for name, values in samples.items()}, 'finite_scan_counts': scan_counts,
                            'reserved_cache_bytes': owner._charged_bytes, 'retained_entries': len(owner._entries), 'samples': samples}
                report['cases'].append(case)
            finally:
                await owner.close()
            assert owner._charged_bytes == 0 and not owner._entries
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps({key: value for key, value in report.items() if key != 'cases'}, indent=2))
    for case in report['cases']:
        baseline, reuse = case['summaries']['baseline'], case['summaries']['reuse']
        print(f"{case['points']:>8} points {case['clients']} clients: wall "
              f"{baseline['wall_ms']['median']:.3f}->{reuse['wall_ms']['median']:.3f} ms, CPU "
              f"{baseline['process_cpu_ms']['median']:.3f}->{reuse['process_cpu_ms']['median']:.3f} ms, "
              f"scans {case['finite_scan_counts']}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=11)
    parser.add_argument('--output', default='/tmp/neh-pointcloud-reuse-performance.json')
    arguments = parser.parse_args()
    if arguments.samples < 3:
        parser.error('--samples must be at least 3')
    asyncio.run(main(arguments))
