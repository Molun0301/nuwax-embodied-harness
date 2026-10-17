"""开发工具共用的数据读取；保留原始坐标和时间，不自动配准。"""
from __future__ import annotations

import json
import math
from pathlib import Path
import struct


def load_poses(path: Path, allow_empty=False) -> list[dict]:
    text = path.read_text(encoding="utf-8").strip()
    rows = json.loads(text) if text.startswith("[") else [json.loads(line) for line in text.splitlines() if line.strip()]
    if not isinstance(rows, list) or (not rows and not allow_empty):
        raise ValueError("轨迹不能为空")
    previous = -1
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"轨迹第{index + 1}项应为对象")
        stamp, position, quaternion = row.get("timestamp_ns"), row.get("p"), row.get("q")
        if type(stamp) is not int or stamp < previous or stamp < 0:
            raise ValueError(f"轨迹第{index + 1}项的timestamp_ns必须为非负、非递减整数")
        for name, value, length in (("p", position, 3), ("q", quaternion, 4)):
            if not isinstance(value, list) or len(value) != length or not all(
                type(v) in (float, int) and math.isfinite(v) for v in value
            ):
                raise ValueError(f"轨迹第{index + 1}项的{name}应有{length}个有限数值")
        if abs(sum(v * v for v in quaternion) - 1) > 0.01:
            raise ValueError(f"轨迹第{index + 1}项的q必须为单位四元数，顺序为x,y,z,w")
        previous = stamp
    return rows


def cloud_statistics(path: Path) -> dict:
    """分块核对小端四个float32组成的XYZT（坐标及相对秒数）记录。"""
    size = path.stat().st_size
    if not size or size % 16:
        raise ValueError("点云必须是非空XYZT文件，每点16字节")
    low, high = [math.inf] * 4, [-math.inf] * 4
    with path.open("rb") as stream:
        while block := stream.read(16 * 65536):
            for row in struct.iter_unpack("<4f", block):
                if not all(math.isfinite(v) for v in row) or row[3] < 0:
                    raise ValueError("点云含非有限坐标或负相对时间")
                for index, value in enumerate(row):
                    low[index], high[index] = min(low[index], value), max(high[index], value)
    return {"point_count": size // 16, "bounds": low[:3] + high[:3], "time_range_seconds": [low[3], high[3]]}


def write_json(path: Path, value):
    """先写同目录临时文件，再原子发布，避免半份清单。"""
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)
