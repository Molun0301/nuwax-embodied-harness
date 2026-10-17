"""只读记录原生里程计、公共里程计、规范坐标变换和地图位姿的实际接收时刻。"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import signal
import threading
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "services/nuwax-workspace/lcmtypes/generated/python"))
from geometry_msgs.PoseStamped import PoseStamped
from nav_msgs.Odometry import Odometry
from nuwax_msgs.LioOdometryEnvelope import LioOdometryEnvelope
from tf2_msgs.TFMessage import TFMessage
import lcm


def header_time(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nsec)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=45)
    parser.add_argument("--private-prefix", default="/lio")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("记录时间必须为正有限数")
    import re
    pattern = rf"^({re.escape(args.private_prefix)}/(f2|pl)/[0-9]+/o|/geometry/lio/odometry|/tf|/go2/pose_map)(#.*)?$"
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output / "位姿交付阶段.jsonl"
    if output.exists():
        parser.error("输出目录已有记录，请保留原证据并选择新目录")
    stopped = threading.Event()
    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(stop_signal, lambda *_: stopped.set())
    counts: Counter = Counter()
    failures = []
    bus = lcm.LCM(args.url)
    types = [LioOdometryEnvelope, Odometry, PoseStamped, TFMessage]
    fingerprints = {item._get_packed_fingerprint(): item for item in types}
    with output.open("w", encoding="utf-8", buffering=1) as stream:
        def accept(channel, payload):
            wall_ns, mono_ns = time.time_ns(), time.monotonic_ns()
            cls = fingerprints.get(payload[:8])
            if cls is None:
                counts["unknown_fingerprint"] += 1
                return
            try:
                message = cls.decode(payload)
                common = {"channel": channel, "received_unix_ns": wall_ns,
                          "received_monotonic_ns": mono_ns, "bytes": len(payload)}
                if cls is LioOdometryEnvelope:
                    rows = [{"kind": "public_odometry", "source_timestamp_ns": message.source_timestamp_ns,
                             "source_sequence": message.source_sequence, "runtime_generation": message.runtime_generation}]
                elif cls is Odometry:
                    rows = [{"kind": "native_odometry", "source_timestamp_ns": header_time(message.header),
                             "source_sequence": message.header.seq}]
                elif cls is PoseStamped:
                    rows = [{"kind": "map_pose", "source_timestamp_ns": header_time(message.header),
                             "source_sequence": message.header.seq}]
                else:
                    rows = [{"kind": "canonical_transform", "source_timestamp_ns": header_time(edge.header),
                             "parent": edge.header.frame_id, "child": edge.child_frame_id}
                            for edge in message.transforms]
                for row in rows:
                    counts[row["kind"]] += 1
                    stream.write(json.dumps({**common, **row}, ensure_ascii=False) + "\n")
            except Exception as exc:
                failures.append({"channel": channel, "error": str(exc), "received_unix_ns": wall_ns})
        subscription = bus.subscribe(pattern, accept)
        (args.output / "就绪.json").write_text(json.dumps({"pattern": pattern,
            "pid": __import__("os").getpid(), "ready_unix_ns": time.time_ns()}, ensure_ascii=False), encoding="utf-8")
        deadline = time.monotonic() + args.seconds
        try:
            while not stopped.is_set() and time.monotonic() < deadline:
                try:
                    bus.handle_timeout(100)
                except OSError:
                    if not stopped.is_set():
                        raise
        finally:
            bus.unsubscribe(subscription)
    (args.output / "交付采集汇总.json").write_text(json.dumps({"counts": counts, "errors": failures,
        "scope": "本独立订阅器接收时刻；不冒充生产线程内部采样、入队或处理时刻"},
        ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if any(counts[k] for k in ("native_odometry", "public_odometry", "map_pose")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
