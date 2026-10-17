"""将历史原生重放的同源位姿/雷达输出转换为可复用对比数据。"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile

from tool_data import load_poses, write_json


def export_replay(args, add_variant):
    try:
        import lcm
        import numpy as np
    except ImportError as exc:
        raise ValueError("原生重放导出需要numpy和lcm，使用已安装项目核心依赖的Python环境") from exc
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "services/nuwax-workspace/lcmtypes/generated/python"))
    from nav_msgs import Odometry
    from sensor_msgs import PointCloud2

    folder = args.folder.resolve()
    info = json.loads((folder / "重放结果.json").read_text(encoding="utf-8"))
    command = info["command"]

    def option(name):
        if name not in command or command.index(name) + 1 == len(command):
            raise ValueError(f"原生重放缺少{name}参数，不能推断外参或通道")
        return command[command.index(name) + 1]

    rotation = np.asarray([float(v) for v in option("--extrinsic_r").split(",")]).reshape(3, 3)
    translation = np.asarray([float(v) for v in option("--extrinsic_t").split(",")])
    if translation.shape != (3,) or not np.isfinite(translation).all() or not np.isfinite(rotation).all():
        raise ValueError("外参数量或数值无效")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5) or abs(np.linalg.det(rotation) - 1) > 1e-5:
        raise ValueError("外参旋转矩阵不是有效正交旋转")
    if not all(math.isfinite(v) and v > 0 for v in (args.voxel_size, args.time_bin_seconds)):
        raise ValueError("显示采样间距和时间分箱必须为有限正数")
    cloud_channel, pose_channel = option("--lidar"), option("--odometry")
    sensor_frame = option("--sensor_frame_id")
    expected = load_poses(folder / "原生轨迹.jsonl")
    base = info["source_base_ns"]
    pending, rows, seen = {}, [], set()
    last_bin = None
    raw_points = 0
    with tempfile.TemporaryDirectory(prefix="nuwax-lio-export-") as temporary:
        output = Path(temporary)
        cloud_file = output / "cloud.xyzt"
        log = lcm.EventLog(str(folder / "原生位姿与点云.lcm"), "r")
        try:
            with cloud_file.open("wb") as stream:
                for event in log:
                    if event.channel == pose_channel:
                        message = Odometry.decode(event.data)
                        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nsec
                        if stamp in pending:
                            raise ValueError(f"位姿时间戳重复：{stamp}")
                        pose = message.pose.pose
                        pending[stamp] = {"timestamp_ns": stamp,
                                          "p": [pose.position.x, pose.position.y, pose.position.z],
                                          "q": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}
                    elif event.channel == cloud_channel:
                        message = PointCloud2.decode(event.data)
                        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nsec
                        row = pending.pop(stamp, None)
                        if row is None or len(rows) >= len(expected) or row != expected[len(rows)]:
                            raise ValueError(f"点云与原生轨迹没有完整同时间配对：{stamp}")
                        if message.height != 1 or message.is_bigendian or message.header.frame_id != sensor_frame:
                            raise ValueError("仅支持与重放声明雷达坐标系一致的小端单行点云")
                        fields = {field.name: field for field in message.fields}
                        if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
                               or not 0 <= fields[name].offset <= message.point_step - 4 for name in ("x", "y", "z")):
                            raise ValueError("点云缺少有效的float32坐标字段")
                        if len(message.data) < message.width * message.point_step:
                            raise ValueError("点云数据被截断")
                        points = np.column_stack([np.ndarray((message.width,), dtype="<f4", buffer=message.data,
                                                             offset=fields[name].offset, strides=(message.point_step,))
                                                  for name in ("x", "y", "z")])
                        # 与原生成器的旋转库一致，计算时归一化四元数，导出的原轨迹不改写。
                        quaternion = np.asarray(row["q"], dtype=np.float64)
                        x, y, z, w = quaternion / np.linalg.norm(quaternion)
                        pose_rotation = np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                        world = (points.astype(np.float64) @ rotation.T + translation) @ pose_rotation.T + row["p"]
                        elapsed = (stamp - base) / 1e9
                        if elapsed < 0 or not np.isfinite(world).all():
                            raise ValueError("点云存在非有限坐标或时间早于source_base_ns")
                        time_bin = math.floor(elapsed / args.time_bin_seconds)
                        if time_bin != last_bin:
                            seen.clear()
                            last_bin = time_bin
                        # 只保存每个时间分箱内首次观测的体素，不消除往返重影。
                        keys = np.floor(world / args.voxel_size).astype(np.int64)
                        keep = []
                        for index, key in enumerate(map(tuple, keys)):
                            if key not in seen:
                                seen.add(key)
                                keep.append(index)
                        selected = world[keep]
                        np.column_stack([selected, np.full(len(keep), elapsed)]).astype("<f4").tofile(stream)
                        raw_points += len(points)
                        rows.append(row)
        finally:
            log.close()
        if pending or len(rows) != len(expected) or len(rows) != info["clouds"] or len(rows) != info["poses"]:
            raise ValueError("重放的位姿/点云数量或剩余配对与声明不一致")
        poses_file = output / "轨迹.json"
        write_json(poses_file, rows)
        return add_variant(args.workspace.resolve(), args.id, args.title, cloud_file, poses_file,
                           args.source_label, args.reference, base, args.change,
                           {"source": {"replay_directory": str(folder), "source_recording": info.get("source"),
                                       "command": command, "label": args.source_label},
                            "location": str(folder), "raw_point_count": raw_points,
                            "display_sampling": {"voxel_size_m": args.voxel_size, "time_bin_seconds": args.time_bin_seconds},
                            "replay_observations": {"returncode": info.get("returncode"), "errors_count": len(info.get("errors", [])),
                                                    "output_timeouts": info.get("output_timeouts", [])}})
