"""Geometry只读视图到算法输入类型的转换，不建立第二份地图快照。"""
from datetime import datetime, timezone

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells


def source_time(timestamp_ns):
    return datetime.fromtimestamp(timestamp_ns // 1_000_000_000, timezone.utc).replace(
        microsecond=timestamp_ns % 1_000_000_000 // 1000)


def algorithm_pose(value):
    return Pose(frame_id=value.frame_id, timestamp=source_time(value.timestamp_ns),
                position=Vector3(x=value.x, y=value.y, z=value.z),
                orientation=Quaternion(x=value.qx, y=value.qy, z=value.qz, w=value.qw))


def algorithm_grid(value):
    timestamp = source_time(value.timestamp_ns)
    return OccupancyGrid(map_id=value.map_name, frame_id=value.frame_id, width=value.width, height=value.height,
        resolution_m=value.resolution, timestamp=timestamp,
        origin=Pose(frame_id=value.frame_id, timestamp=timestamp,
                    position=Vector3(x=value.origin_x, y=value.origin_y)), data=PackedOccupancyCells(value.data))


def map_binding(value):
    return {"map_name": value.map_name, "frame_id": value.frame_id, "map_revision": str(value.timestamp_ns)}


def same_scope(left, right):
    return all(left.get(name) == right.get(name) for name in ("map_name", "frame_id"))
