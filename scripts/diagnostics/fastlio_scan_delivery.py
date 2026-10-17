"""用真实原生进程检查扫描末端覆盖后能否在下一帧之前交付当前位姿。"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import lcm
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "services/nuwax-workspace/lcmtypes/generated/python"))
from nav_msgs import Odometry
from sensor_msgs import Imu, PointCloud2, PointField


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-ns", type=int, required=True)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--wait-ms", type=float, default=100.0)
    parser.add_argument("--scan-duration-ns", type=int, default=95_000_000)
    parser.add_argument("--imu-shortfall-ns", type=int, default=0,
                        help="末端惯性真实缺测纳秒数，用于验证整数覆盖拒绝")
    args = parser.parse_args()
    if not 90_000_000 < args.scan_duration_ns < 100_000_000:
        parser.error("本夹具扫描时长须在90至100毫秒之间")
    if not 0 <= args.imu_shortfall_ns < 5_000_000:
        parser.error("末端缺测须在0至5000000纳秒之间")
    args.output.mkdir(parents=True, exist_ok=False)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    url = f"udpm://239.255.76.67:{port}?ttl=0"
    prefix = f"/scan_timing/{os.getpid()}"
    cloud_topic, imu_topic, odom_topic = (prefix + suffix for suffix in (
        "/input#sensor_msgs.PointCloud2", "/imu#sensor_msgs.Imu", "/odom#nav_msgs.Odometry"))
    bus = lcm.LCM(url)
    outputs = []

    def receive(_channel, payload):
        value = Odometry.decode(payload)
        outputs.append({"source_ns": value.header.stamp.sec * 10**9 + value.header.stamp.nsec,
                        "received_monotonic_ns": time.monotonic_ns()})

    subscription = bus.subscribe(odom_topic, receive)
    command = [str(args.binary.resolve()), "--odometry", odom_topic,
        "--input_lidar", cloud_topic, "--input_imu", imu_topic,
        "--sensor_frame_id", "scan_sensor", "--child_frame_id", "scan_body",
        "--frame_id", "odom", "--extrinsic_r", "1,0,0,0,1,0,0,0,1",
        "--extrinsic_t", "0,0,0", "--initial_map_frames", "1",
        "--filter_size_surf", ".1", "--filter_size_map", ".2", "--scan_publish_en", "false"]
    coords = np.linspace(-3, 3, 20)
    points = np.asarray([(x, y, -1.) for x in coords for y in coords]
        + [(3., y, z) for y in coords for z in np.linspace(-1, 2, 10)]
        + [(x, 3., z) for x in coords for z in np.linspace(-1, 2, 10)], dtype="<f4")
    packed = np.empty(len(points), dtype=[("xyz", "<f4", 3), ("offset", "<u4")])
    packed["xyz"] = points
    packed["offset"] = np.linspace(0, args.scan_duration_ns, len(points), dtype="<u4")
    # 末端保留多个真实同时间回波，使原预处理抽点后仍有真实末端点。
    packed["offset"][-4:] = args.scan_duration_ns
    cloud = PointCloud2()
    cloud.header.frame_id = "scan_sensor"
    cloud.height, cloud.width, cloud.point_step = 1, len(points), 16
    cloud.row_step = cloud.data_length = len(points) * 16
    cloud.is_dense = True
    cloud.data = packed.tobytes()
    for name, offset, datatype in [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("offset_ns", 12, 6)]:
        field = PointField()
        field.name, field.offset, field.datatype, field.count = name, offset, datatype, 1
        cloud.fields.append(field)
    cloud.fields_length = len(cloud.fields)
    records = []
    with (args.output / "原生进程.log").open("w") as log:
        process = subprocess.Popen(command, env={**os.environ, "LCM_DEFAULT_URL": url,
            "OMP_NUM_THREADS": "2"}, stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(.5)
            assert process.poll() is None, "原生进程启动失败"
            for frame in range(args.frames):
                stamp = args.base_ns + frame * 100_000_000
                for index, offset in enumerate(range(0, 100_000_000, 5_000_000)):
                    sample = Imu()
                    sample.header.frame_id = "scan_sensor"
                    sample.header.seq = frame * 20 + index
                    sample_stamp = stamp + (args.scan_duration_ns - args.imu_shortfall_ns if index == 19 else offset)
                    sample.header.stamp.sec, sample.header.stamp.nsec = divmod(sample_stamp, 10**9)
                    sample.orientation.w, sample.linear_acceleration.z = 1., 9.81
                    bus.publish(imu_topic, sample.encode())
                cloud.header.seq = frame
                cloud.header.stamp.sec, cloud.header.stamp.nsec = divmod(stamp, 10**9)
                expected = stamp + args.scan_duration_ns
                started = time.monotonic_ns()
                bus.publish(cloud_topic, cloud.encode())
                deadline = time.monotonic() + args.wait_ms / 1000
                while time.monotonic() < deadline:
                    bus.handle_timeout(5)
                current = next((r for r in outputs if abs(r["source_ns"] - expected) <= 1000), None)
                records.append({"frame": frame, "scan_end_ns": expected,
                    "before_next_frame": current is not None,
                    "delivery_ms": (current["received_monotonic_ns"] - started) / 1e6 if current else None,
                    "latest_source_ns": outputs[-1]["source_ns"] if outputs else None})
                assert process.poll() is None, "原生处理过程退出"
        finally:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            bus.unsubscribe(subscription)
    first = next((r["frame"] for r in records if r["latest_source_ns"] is not None), None)
    checked = [r for r in records if first is not None and r["frame"] > first]
    coverage_rejections = (args.output / "原生进程.log").read_text().count("reject typed cloud: IMU does not cover scan window")
    report = {"command": command, "imu_shortfall_ns": args.imu_shortfall_ns,
        "coverage_rejections": coverage_rejections, "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        "source_base_ns": args.base_ns, "returncode": process.returncode, "first_output_frame": first,
        "checked_frames": len(checked), "late_frames": sum(not r["before_next_frame"] for r in checked),
        "rows": records, "outputs": outputs,
        "scope": "真实FastLIO2原生进程与隔离LCM通道；静态合成场景，整数末端惯性精确覆盖；不验证定位精度"}
    (args.output / "逐帧交付结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ("command", "rows", "outputs")}, ensure_ascii=False))
    passed = (not outputs and coverage_rejections == args.frames) if args.imu_shortfall_ns else bool(checked and report["late_frames"] == 0)
    return 0 if passed and process.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
