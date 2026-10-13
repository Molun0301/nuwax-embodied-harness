#!/usr/bin/env python3
"""Bounded paired public-builder benchmark for exact envelope-size reuse."""
import json
import statistics
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.check_tool_result_size_reuse import baseline, candidate, wire_size

def payload(kind, count):
    if kind == 'text':
        return {'result': {'observation_context': {'camera_id': 'front', 'metadata': {'notes': '安全图片\\\"😀\n' * count}}}}
    return {'result': {'observations': [{'camera_id': 'front_%d' % i, 'metadata': {'confidence': 0.75, 'labels': ['画面', 'object'], 'position': [1, 2, 3]}} for i in range(count)]}}

def main():
    rows = []
    for kind, counts in [('text', (32, 512, 8192)), ('list', (4, 64, 1024))]:
        for count in counts:
            value = payload(kind, count)
            old, new = baseline(value), candidate(value)
            assert old == new
            encoded_bytes = wire_size(old)
            assert encoded_bytes <= 512 * 1024
            times = {'baseline': [], 'candidate': []}; cpu = {'baseline': [], 'candidate': []}
            iterations = 10 if count >= 1024 else 30
            for round_number in range(5):
                implementations = [('baseline', baseline), ('candidate', candidate)]
                if round_number % 2:
                    implementations.reverse()
                for label, fn in implementations:
                    start_cpu, start = time.process_time(), time.perf_counter()
                    for _ in range(iterations):
                        result = fn(value)
                    times[label].append((time.perf_counter() - start) / iterations)
                    cpu[label].append((time.process_time() - start_cpu) / iterations)
            row = {'kind': kind, 'count': count, 'encoded_bytes': encoded_bytes,
                   'iterations_per_round': iterations, 'rounds': 5, 'output_equal': True,
                   'seconds_per_build': times, 'cpu_seconds_per_build': cpu,
                   'median_reduction_pct': 100 * (1 - statistics.median(times['candidate']) / statistics.median(times['baseline']))}
            rows.append(row); print(json.dumps(row), flush=True)
    Path(sys.argv[1]).write_text(json.dumps({'python': sys.version, 'production_modified': False, 'rows': rows}, indent=2))

if __name__ == '__main__':
    main()
