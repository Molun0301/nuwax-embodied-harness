"""只读保存生产近期点云与原始地面成员索引，用于来源对账。"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'services/nuwax-workspace/lcmtypes/generated/python'))
from nuwax_msgs.RecentCloud3D import RecentCloud3D
from nuwax_msgs.LioCloudEnvelope import LioCloudEnvelope
import lcm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--target-cell', default='[-7,-2,-7]')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('采集时间必须为正有限秒数')
    args.output.mkdir(parents=True, exist_ok=True)
    target = json.loads(args.target_cell)
    if not isinstance(target, list) or len(target) != 3 or any(type(value) is not int for value in target):
        parser.error('目标体素需要三个整数坐标')
    if (args.output / '原生近期局部帧.jsonl').exists():
        parser.error('输出目录已有原始近期证据，请选择新目录')
    target_patch = [value // 4 for value in target[:2]]
    quantum = struct.unpack('<f', struct.pack('<f', .05))[0]
    bus = lcm.LCM(args.url)
    records = []
    frames = (args.output / '原生近期局部帧.jsonl').open('w', encoding='utf-8')

    def accept(channel, payload):
        try:
            recent = RecentCloud3D.decode(payload)
            if recent.cleared:
                return
            cloud = LioCloudEnvelope.decode(bytes(recent.cloud_bytes))
            packet = cloud.cloud
            offsets = {field.name: field.offset for field in packet.fields}
            indices = []
            points = []
            target_indices = []
            ground = set(recent.ground_point_indices)
            for index in range(packet.width * packet.height):
                start = (index // packet.width) * packet.row_step + (index % packet.width) * packet.point_step
                point = [struct.unpack_from('<f', packet.data, start + offsets[axis])[0] for axis in ('x', 'y', 'z')]
                if not all(math.isfinite(value) for value in point):
                    continue
                key = [math.floor(value / quantum) for value in point]
                if [value // 4 for value in key[:2]] == target_patch:
                    indices.append(index)
                    points.append(point)
                if key == target:
                    target_indices.append(index)
            frame = {'sequence': cloud.source_sequence, 'timestamp_ns': cloud.source_timestamp_ns,
                     'source': {'kind': 'lidar', 'backend': cloud.backend,
                                'runtime_generation': cloud.runtime_generation, 'sensor_frame_id': cloud.sensor_frame_id},
                     'session_id': recent.session_id, 'map_frame_generation': recent.map_frame_generation,
                     'points': points, 'indices': indices, 'published_ground_indices': list(recent.ground_point_indices),
                     'target_indices': target_indices,
                     'unclassified_target_indices': [index for index in target_indices if index not in ground]}
            frames.write(json.dumps(frame, ensure_ascii=False) + '\n')
            frames.flush()
            (args.output / f'{len(records):04d}_{cloud.source_sequence}.lcm').write_bytes(payload)
            summary = {key: frame[key] for key in ('sequence', 'timestamp_ns', 'target_indices', 'unclassified_target_indices')}
            summary.update(channel=channel, emitted_points=cloud.emitted_point_count,
                           ground_points=len(ground), patch_points=len(points))
            records.append(summary)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
        except Exception as exc:
            print(json.dumps({'decode_error': str(exc), 'channel': channel, 'prefix': payload[:16].hex()}), flush=True)

    subscription = bus.subscribe('/nav/observations3d.*', accept)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            bus.handle_timeout(200)
    finally:
        bus.unsubscribe(subscription)
        frames.close()
        (args.output / '近期源汇总.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
    return 0 if records else 1


if __name__ == '__main__':
    raise SystemExit(main())
