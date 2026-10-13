"""Bounded no-hash delivery measurements, direct actual encoder/projection calls.
No runtime construction, network, devices, persistence, or application hashes.

Run from the checkout root with the project interpreter:
    python scripts/diagnostics/stream_delivery_probe.py > delivery-results.json
Options: --direct-batches (default 60), --threadpool-batches (default 30).
Single-client results are primary; 4/8 clients are scaling context.
This is not an end-to-end socket benchmark or an event-loop-delay measurement.
"""
import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from contracts.geometry import Pose, Vector3
from contracts.streams import StreamFrame
from contracts.events import RuntimeEvent, RuntimeEventCategory
from gateways.ws.data_streams import _encode_ws_frame
from gateways.feedback import build_public_runtime_event
from fastapi.concurrency import run_in_threadpool


def summary(values):
    values = sorted(values)
    return {'median_ms': statistics.median(values) * 1000, 'p95_ms': values[int((len(values)-1)*.95)] * 1000,
            'max_ms': max(values) * 1000}

async def measure(direct_batches=60, threadpool_batches=30):
    NOW = datetime(2026, 10, 13, tzinfo=timezone.utc)
    poses = [Pose(frame_id='map', position=Vector3(x=i * .1, y=i * .03), timestamp=NOW) for i in range(100)]
    trajectory = StreamFrame(topic='localization.trajectory', source='localization_service', cursor=1,
                             payload={'trajectory': poses}, timestamp=NOW)
    events = {
        'task_progress': RuntimeEvent(event_id='delivery-probe-task', event_type='task.progress',
            category=RuntimeEventCategory.TASK, source='task_manager', cursor=1,
            payload={'state': 'running', 'progress': .5, 'message': 'Working', 'stage': 'navigation'}, timestamp=NOW),
        'robot_32_items': RuntimeEvent(event_id='delivery-probe-robot', event_type='robot.telemetry',
            category=RuntimeEventCategory.ROBOT, source='robot', cursor=2,
            payload={'readings': [{'name': 'sensor%d' % i, 'value': i * .25, 'unit': 'm'} for i in range(32)]}, timestamp=NOW),
    }
    output = {'python': sys.version.split()[0], 'fixture_trajectory_poses': len(poses), 'direct_batches': direct_batches,
              'threadpool_batches': threadpool_batches, 'results': []}
    for name, fn, fixture in [('trajectory', _encode_ws_frame, trajectory)] + [
        (name, build_public_runtime_event, event) for name, event in events.items()]:
        encoded = fn(fixture)
        size = len(encoded.encode('utf8')) if isinstance(encoded, str) else len(json.dumps(encoded, ensure_ascii=False).encode('utf8'))
        for clients in (1, 4, 8):
            values = []
            for _ in range(direct_batches):
                start = time.perf_counter()
                for _client in range(clients):
                    fn(fixture)
                values.append(time.perf_counter()-start)
            output['results'].append({'case': name, 'mode': 'direct_cpu', 'clients': clients,
                 'calls_per_publication': clients, 'wire_bytes': size, **summary(values)})
    for name, event in events.items():
        for clients in (1, 4, 8):
            values = []
            for _ in range(threadpool_batches):
                start = time.perf_counter()
                await asyncio.gather(*(run_in_threadpool(build_public_runtime_event, event) for _ in range(clients)))
                values.append(time.perf_counter()-start)
            output['results'].append({'case': name, 'mode': 'gateway_threadpool_fanout', 'clients': clients, **summary(values)})
    # Prove mutable inputs make identity-only generic memoization semantically invalid.
    before = json.loads(_encode_ws_frame(trajectory))
    poses[0].position.x = 99.0
    after = json.loads(_encode_ws_frame(trajectory))
    assert before['frame']['payload']['trajectory'][0]['position']['x'] != after['frame']['payload']['trajectory'][0]['position']['x']
    event = events['robot_32_items']
    before = build_public_runtime_event(event)
    event.payload['readings'][0]['value'] = 99.0
    assert build_public_runtime_event(event) != before
    output['mutable_input_regressions'] = 'trajectory nested pose and RuntimeEvent payload changes are visible on next encoding'
    print(json.dumps(output, indent=2))

def positive_count(value):
    count = int(value)
    if count < 1:
        raise argparse.ArgumentTypeError("batch count must be positive")
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--direct-batches', type=positive_count, default=60,
                        help='direct CPU batches per case (default: 60)')
    parser.add_argument('--threadpool-batches', type=positive_count, default=30,
                        help='actual gateway threadpool batches per case (default: 30)')
    args = parser.parse_args()
    asyncio.run(measure(args.direct_batches, args.threadpool_batches))


if __name__ == '__main__':
    main()
