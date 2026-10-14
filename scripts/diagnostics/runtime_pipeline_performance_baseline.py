#!/usr/bin/env python3
"""runtime_pipeline_performance_baseline 最小开发机性能基线 runner。

该工具只测本地纯 Python 模拟路径，用于在修改生产代码前留下可复现的
延迟、吞吐、资源和队列基线。真实 Go2 DDS、LCM、WebRTC、Motion Guard
UDS、远端 WS/SSE 和 Jetson 链路始终标记为 ``NOT_EVALUATED``。

示例::

    python3 scripts/diagnostics/runtime_pipeline_performance_baseline.py \
        --scenario pc-m --round 2 --samples 5
    python3 scripts/diagnostics/runtime_pipeline_performance_baseline.py \
        --scenario vid-15 --consumers 4 --output /tmp/runtime_pipeline_performance_baseline-vid.json
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import resource
import sys
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple


SCENARIO_NAMES = (
    "pc-s",
    "pc-m",
    "pc-l",
    "pc-x",
    "img-640",
    "vid-15",
    "stream-burst",
)
PC_SIZES = {"pc-s": 1_200, "pc-m": 10_000, "pc-l": 50_000, "pc-x": 100_000}
MAX_POINT_COUNT = 100_000
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
STREAM_QUEUE_CAPACITY = 64


def _rss_bytes() -> Optional[int]:
    """Read this process RSS without requiring psutil."""

    try:
        with open("/proc/self/status", "r", encoding="ascii") as status_file:
            for line in status_file:
                if line.startswith("VmRSS:"):
                    fields = line.split()
                    return int(fields[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    try:
        # Linux reports ru_maxrss in KiB; other Unix systems may report bytes.
        value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        return value * 1024 if sys.platform.startswith("linux") else value
    except (OSError, ValueError):
        return None


def _environment() -> Dict[str, Any]:
    try:
        cpu_count = os.cpu_count()
    except (AttributeError, OSError):
        cpu_count = None
    return {
        "category": "DEVELOPMENT_HOST",
        "python": platform.python_version(),
        "platform": platform.platform(),
        "cpu_count": cpu_count,
        "real_go2_dds": "NOT_EVALUATED",
        "real_lcm": "NOT_EVALUATED",
        "real_webrtc": "NOT_EVALUATED",
        "motion_guard_uds": "NOT_EVALUATED",
        "remote_ws_sse": "NOT_EVALUATED",
        "jetson": "NOT_EVALUATED",
    }


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _pointcloud_work(point_count: int, variant: str) -> Dict[str, Any]:
    """Decode/transform/sample one deterministic XYZ or XYZI-like cloud."""

    if point_count > MAX_POINT_COUNT:
        return {
            "input_count": point_count,
            "accepted": 0,
            "published": 0,
            "dropped": point_count,
            "queue_max_depth": 0,
            "functional": {
                "point_count": point_count,
                "accepted_points": 0,
                "variant": "XYZ-only" if variant != "pc-m" else "XYZI+offset",
                "frame": "map",
                "source_timestamp": "present",
                "over_limit_rejected": True,
                "failure_closed": True,
                "coordinate_sum": [0.0, 0.0, 0.0],
            },
        }

    accepted = 0
    dropped = 0
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    # Generate and consume in one loop so this remains a bounded local fixture.
    for index in range(point_count):
        x = (index % 251) * 0.01 - 1.25
        y = ((index * 17) % 251) * 0.01 - 1.25
        z = ((index * 7) % 101) * 0.01
        # The transform is deliberately simple but deterministic; it does not
        # claim to model a real DDS or sensor decoder.
        tx = x + 0.12
        ty = y - 0.08
        tz = z + 0.03
        if (index & 3) == 0:
            accepted += 1
            sum_x += tx
            sum_y += ty
            sum_z += tz
        else:
            dropped += 1
    return {
        "input_count": point_count,
        "accepted": accepted,
        "published": accepted,
        "dropped": dropped,
        "queue_max_depth": 0,
        "functional": {
            "point_count": point_count,
            "accepted_points": accepted,
            "variant": "XYZ-only" if variant != "pc-m" else "XYZI+offset",
            "frame": "map",
            "source_timestamp": "present",
            "over_limit_rejected": False,
            "failure_closed": False,
            "coordinate_sum": [round(sum_x, 6), round(sum_y, 6), round(sum_z, 6)],
        },
    }


def _image_work() -> Dict[str, Any]:
    """Run a fixed-size image decode/normalize simulation."""

    pixels = IMAGE_WIDTH * IMAGE_HEIGHT
    total = 0
    # A sparse deterministic scan keeps the fixture fast while exercising the
    # same fixed image dimensions on every sample.
    for index in range(0, pixels, 3):
        total += (index * 13 + 17) & 0xFF
    return {
        "input_count": pixels,
        "accepted": 1,
        "published": 1,
        "dropped": 0,
        "queue_max_depth": 1,
        "functional": {
            "width": IMAGE_WIDTH,
            "height": IMAGE_HEIGHT,
            "channels": 3,
            "backend": "pure_simulation",
            "pixel_accumulator": total,
            "inference": "NOT_EVALUATED",
        },
    }


def _video_work(consumers: int) -> Dict[str, Any]:
    """Simulate one fixed JPEG frame fan-out to local consumers."""

    payload_size = 64 * 1024
    # Bytes are shared by all local consumers; no JPEG codec or WebRTC source
    # is available in this diagnostic path.
    payload = bytes((index * 29 + 11) & 0xFF for index in range(payload_size))
    published = 0
    for consumer in range(consumers):
        published += len(payload) > consumer
    return {
        "input_count": 1,
        "accepted": 1,
        "published": published,
        "dropped": 0,
        "queue_max_depth": consumers,
        "functional": {
            "width": IMAGE_WIDTH,
            "height": IMAGE_HEIGHT,
            "fps": 15,
            "jpeg_bytes": len(payload),
            "consumers": consumers,
            "shared_payload": True,
            "webrtc": "NOT_EVALUATED",
        },
    }


def _stream_work() -> Dict[str, Any]:
    """Simulate latest-only slots plus a bounded reliable FIFO."""

    latest: Dict[str, int] = {}
    reliable: Deque[Tuple[str, int]] = deque()
    latest_replacements = 0
    reliable_fifo_drops = 0
    accepted = 0
    max_depth = 0
    sequence = 0
    for burst_index in range(128):
        topic = "pose" if burst_index % 4 == 0 else "costmap"
        if burst_index % 3 == 0:
            if topic in latest:
                latest_replacements += 1
            latest[topic] = sequence
        else:
            if len(reliable) >= STREAM_QUEUE_CAPACITY:
                reliable_fifo_drops += 1
            else:
                reliable.append(("event", sequence))
        accepted += 1
        sequence += 1
        max_depth = max(max_depth, len(latest) + len(reliable))
    dropped = latest_replacements + reliable_fifo_drops
    published = len(latest) + len(reliable)
    reliable.clear()
    latest.clear()
    return {
        "input_count": 128,
        "accepted": accepted,
        "published": published,
        "dropped": dropped,
        "queue_max_depth": max_depth,
        "functional": {
            "latest_topics": 2,
            "reliable_fifo_capacity": STREAM_QUEUE_CAPACITY,
            "latest_slot_replacements": latest_replacements,
            "reliable_fifo_drops": reliable_fifo_drops,
            "remote_ws_sse": "NOT_EVALUATED",
        },
    }


def _runner_for(
    scenario: str, input_size: Optional[int], consumers: int
) -> Tuple[Callable[[], Dict[str, Any]], Dict[str, Any]]:
    normalized = scenario.lower()
    if normalized in PC_SIZES:
        count = input_size if input_size is not None else PC_SIZES[normalized]
        if count < 1:
            raise ValueError("input size must be positive")
        return (
            lambda: _pointcloud_work(count, normalized),
            {
                "kind": "pointcloud",
                "name": normalized.upper(),
                "points": count,
                "max_points": MAX_POINT_COUNT,
                "configuration": "XYZ-only" if normalized != "pc-m" else "XYZI+offset",
            },
        )
    if normalized == "img-640":
        return (
            _image_work,
            {
                "kind": "image",
                "name": "IMG-640",
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "backend": "pure_simulation",
            },
        )
    if normalized == "vid-15":
        if consumers < 1:
            raise ValueError("consumers must be positive")
        return (
            lambda: _video_work(consumers),
            {
                "kind": "video",
                "name": "VID-15",
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "fps": 15,
                "consumers": consumers,
                "codec": "fixed-payload-simulation",
            },
        )
    if normalized == "stream-burst":
        return (
            _stream_work,
            {
                "kind": "stream",
                "name": "STREAM-BURST",
                "messages_per_sample": 128,
                "queue_capacity": STREAM_QUEUE_CAPACITY,
                "consumers": consumers,
            },
        )
    raise ValueError("unknown scenario: {}".format(scenario))


def _measure(
    scenario: str,
    input_size: Optional[int],
    consumers: int,
    rounds: int,
    samples_per_round: int,
    warmup: int,
) -> Dict[str, Any]:
    operation, input_config = _runner_for(scenario, input_size, consumers)
    for _ in range(warmup):
        operation()

    samples: List[Dict[str, Any]] = []
    functional: Dict[str, Any] = {}
    rss_peak = 0
    wall_total_ns = 0
    cpu_total_ns = 0
    for round_index in range(rounds):
        for sample_index in range(samples_per_round):
            rss_before = _rss_bytes()
            wall_before = time.perf_counter_ns()
            cpu_before = time.process_time_ns()
            result = operation()
            cpu_after = time.process_time_ns()
            wall_after = time.perf_counter_ns()
            rss_after = _rss_bytes()
            elapsed_ns = max(1, wall_after - wall_before)
            cpu_ns = max(0, cpu_after - cpu_before)
            wall_total_ns += elapsed_ns
            cpu_total_ns += cpu_ns
            if rss_after is not None:
                rss_peak = max(rss_peak, rss_after)
            functional = result.get("functional", {})
            samples.append(
                {
                    "round": round_index + 1,
                    "sample": sample_index + 1,
                    "elapsed_ms": elapsed_ns / 1_000_000.0,
                    "cpu_ms": cpu_ns / 1_000_000.0,
                    "rss_bytes": rss_after,
                    "input_count": result.get("input_count", 0),
                    "accepted": result.get("accepted", 0),
                    "published": result.get("published", 0),
                    "dropped": result.get("dropped", 0),
                    "queue_max_depth": result.get("queue_max_depth", 0),
                }
            )

    elapsed_ms = [sample["elapsed_ms"] for sample in samples]
    total_input = sum(int(sample["input_count"]) for sample in samples)
    total_accepted = sum(int(sample["accepted"]) for sample in samples)
    total_published = sum(int(sample["published"]) for sample in samples)
    total_dropped = sum(int(sample["dropped"]) for sample in samples)
    total_seconds = wall_total_ns / 1_000_000_000.0
    rss_values = [sample["rss_bytes"] for sample in samples if sample["rss_bytes"]]
    return {
        "scenario": input_config["name"],
        "status": "PASS",
        "input": input_config,
        "config": {
            "warmup": warmup,
            "rounds": rounds,
            "samples_per_round": samples_per_round,
            "samples_total": len(samples),
            "consumers": consumers,
        },
        "environment": _environment(),
        "samples": samples,
        "latency_ms": {
            "p50": _percentile(elapsed_ms, 50.0),
            "p95": _percentile(elapsed_ms, 95.0),
            "p99": _percentile(elapsed_ms, 99.0),
            "min": min(elapsed_ms) if elapsed_ms else 0.0,
            "max": max(elapsed_ms) if elapsed_ms else 0.0,
        },
        "throughput": {
            "input_per_s": total_input / total_seconds if total_seconds else 0.0,
            "accepted_per_s": total_accepted / total_seconds if total_seconds else 0.0,
            "published_per_s": total_published / total_seconds if total_seconds else 0.0,
            "dropped_per_s": total_dropped / total_seconds if total_seconds else 0.0,
        },
        "accepted": total_accepted,
        "published": total_published,
        "dropped": total_dropped,
        "cpu": {
            "available": True,
            "process_ms_total": cpu_total_ns / 1_000_000.0,
            "cpu_percent_estimate": (
                cpu_total_ns * 100.0 / wall_total_ns if wall_total_ns else 0.0
            ),
        },
        "rss": {
            "available": bool(rss_values),
            "current_bytes": rss_values[-1] if rss_values else None,
            "peak_bytes": rss_peak if rss_values else None,
        },
        "queue": {
            "max_depth": max(
                int(sample["queue_max_depth"]) for sample in samples
            )
            if samples
            else 0,
            "applicable": input_config["kind"] in ("image", "video", "stream"),
        },
        "functional": functional,
        "evaluation": {
            "local_simulation": "EVALUATED",
            "real_go2_dds": "NOT_EVALUATED",
            "real_lcm": "NOT_EVALUATED",
            "real_webrtc": "NOT_EVALUATED",
            "motion_guard_uds": "NOT_EVALUATED",
            "remote_ws_sse": "NOT_EVALUATED",
            "jetson": "NOT_EVALUATED",
        },
    }


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        default="all",
        help="scenario name (pc-s, pc-m, pc-l, pc-x, img-640, vid-15, stream-burst, all)",
    )
    parser.add_argument(
        "--input-size",
        "--size",
        dest="input_size",
        type=int,
        default=None,
        help="override point count for PC scenarios",
    )
    parser.add_argument(
        "--input-scale",
        dest="input_scale",
        choices=("S", "M", "L", "X", "s", "m", "l", "x"),
        default=None,
        help="select PC-S/M/L/X input scale",
    )
    parser.add_argument(
        "--consumers", type=int, default=1, help="local video/stream consumer count"
    )
    parser.add_argument(
        "--round", "--rounds", dest="rounds", type=int, default=1, help="repeat rounds"
    )
    parser.add_argument(
        "--samples", type=int, default=3, help="samples collected per round"
    )
    parser.add_argument(
        "--warmup", type=int, default=1, help="warmup calls excluded from samples"
    )
    parser.add_argument("--output", default=None, help="optional JSON output path")
    args = parser.parse_args(argv)
    if args.input_scale and (
        args.scenario.lower() == "all" or args.scenario.lower() in PC_SIZES
    ):
        args.scenario = "pc-{}".format(args.input_scale.lower())
    if args.rounds < 1 or args.samples < 1 or args.warmup < 0:
        parser.error("rounds and samples must be positive; warmup cannot be negative")
    if args.consumers < 1:
        parser.error("consumers must be positive")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    scenarios = SCENARIO_NAMES if args.scenario.lower() == "all" else (args.scenario.lower(),)
    results = [
        _measure(
            scenario,
            args.input_size,
            args.consumers,
            args.rounds,
            args.samples,
            args.warmup,
        )
        for scenario in scenarios
    ]
    if len(results) == 1:
        payload: Any = results[0]
    else:
        payload = {
            "runner": "runtime_pipeline_performance_baseline",
            "status": "PASS",
            "scenario": "ALL",
            "results": results,
            "environment": _environment(),
            "evaluation": {
                "local_simulation": "EVALUATED",
                "real_go2_dds": "NOT_EVALUATED",
                "real_lcm": "NOT_EVALUATED",
                "real_webrtc": "NOT_EVALUATED",
                "motion_guard_uds": "NOT_EVALUATED",
                "remote_ws_sse": "NOT_EVALUATED",
                "jetson": "NOT_EVALUATED",
            },
        }
    encoded = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as output_file:
            output_file.write(encoded)
            output_file.write("\n")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
