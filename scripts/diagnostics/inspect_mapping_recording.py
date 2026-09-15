"""只读检查端侧几何记录并导出全程轨迹，不连接机器人。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sqlite3
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from services.geometry.recording import iter_recorded_sensors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="端侧recording.sqlite3文件")
    parser.add_argument("--trajectory-output", type=Path, help="导出轨迹JSONL文件，包含全部原始源时间和记录标识")
    parser.add_argument("--validate-sensors", action="store_true", help="逐条读回并验证完整传感器载荷；大记录需要更长时间")
    args = parser.parse_args()
    connection = sqlite3.connect(args.input.resolve().as_uri() + "?mode=ro", uri=True)
    try:
        info = {key: json.loads(value) for key, value in connection.execute("SELECT key,value FROM info")}
        if info.get("schema_version") != 1:
            raise ValueError("不支持的几何记录格式")
        result = {
            "input": str(args.input.resolve()), "info": info,
            "integrity": connection.execute("PRAGMA quick_check").fetchone()[0],
            "counts": dict(connection.execute("SELECT kind,COUNT(*) FROM records GROUP BY kind")),
            "source_time_note": "source_ns保留各传感器原时间域，received_ns为端侧Unix接收时间；不以显示时间覆盖源时间。",
        }
        if args.trajectory_output:
            args.trajectory_output.parent.mkdir(parents=True, exist_ok=True)
            count = 0
            with args.trajectory_output.open("w", encoding="utf-8") as output:
                for record_id, stamp, received, payload in connection.execute(
                    "SELECT id,source_ns,received_ns,payload FROM records WHERE kind='localization.pose' ORDER BY id"
                ):
                    output.write(json.dumps({"record_id": record_id, "source_timestamp_ns": stamp,
                        "received_timestamp_ns": received, **json.loads(payload)}, ensure_ascii=False) + "\n")
                    count += 1
            result["trajectory_records"] = count
        if args.validate_sensors:
            result["validated_sensor_records"] = sum(1 for _ in iter_recorded_sensors(args.input))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["integrity"] == "ok" and not info.get("status", {}).get("incomplete", True) else 1
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
