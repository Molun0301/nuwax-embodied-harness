"""从已有流历史取同一采集时刻的几何输入，不用处理时刻的机体位置。"""
from dataclasses import dataclass

import numpy as np

from contracts.geometry import FrameTree, Pose, Transform
from contracts.pointcloud import PointCloudFrame
from services.localization.frame_tree_ops import resolve_frame_transform


@dataclass(frozen=True)
class ObservationGeometry:
    pose: object = None
    frame_tree: object = None
    point_cloud: object = None
    reason: str = "capture_pose_unavailable"
    sources: tuple = ()


def _stamp(payload, metadata):
    value = metadata.get("source_timestamp_ns")
    return int(value) if value is not None else int(payload.timestamp.timestamp() * 1e9)


def capture_geometry(stream_bus, image_frame):
    """复用有界历史；纳秒容差仅吸收现有浮点时间序列化误差。"""
    if stream_bus is None:
        return ObservationGeometry(reason="geometry_stream_unavailable")
    stamp = _stamp(image_frame, image_frame.metadata)
    clock = image_frame.metadata.get("source_clock_id") or image_frame.metadata.get("clock_id")
    boundary = stream_bus.replay_snapshot(limit=0).map_identity_start_cursor or 0
    frames = stream_bus.replay(after_cursor=boundary, topics=("localization.pose", "localization.frame_tree", "pointcloud.frame"))
    pose_frames = [f for f in frames if f.topic == "localization.pose" and isinstance(f.payload, Pose)
                   and abs(_stamp(f.payload, f.metadata) - stamp) <= 1000]
    if not pose_frames:
        return ObservationGeometry()
    pose = pose_frames[-1].payload
    # 动态 TF 的包装时间可能是发布时间，不能用于采集配对。
    trees = [f.payload for f in frames if f.topic == "localization.frame_tree" and isinstance(f.payload, FrameTree)]
    static = [t for t in trees[-1].transforms if t.authority == "calibration_static"] if trees else []
    camera_edge = next((t for t in static if t.child_frame_id == image_frame.frame_id), None)
    if camera_edge is None:
        return ObservationGeometry(pose=pose, reason="capture_camera_calibration_unavailable")
    base = camera_edge.parent_frame_id
    dynamic_bodies = {t.child_frame_id for t in trees[-1].transforms if t.authority != "calibration_static"}
    if base not in dynamic_bodies:
        return ObservationGeometry(pose=pose, reason="capture_reference_frame_unavailable")
    tree = FrameTree(root_frame_id=pose.frame_id, timestamp=pose.timestamp, transforms=[
        Transform(parent_frame_id=pose.frame_id, child_frame_id=base, translation=pose.position,
                  rotation=pose.orientation, timestamp=pose.timestamp, authority="capture_localization_pose"), *static])
    arrays, sources, seen = [], [], set()
    used_map_cloud = False
    for frame in reversed(frames):
        cloud = frame.payload
        if frame.topic != "pointcloud.frame" or not isinstance(cloud, PointCloudFrame):
            continue
        metadata = cloud.metadata
        map_cloud = cloud.frame_id == pose.frame_id and not metadata.get("canonical_lidar_frame")
        if clock and metadata.get("clock_id") != clock and not map_cloud:
            continue
        sample = metadata.get("canonical_lidar_frame")
        cloud_stamp = _stamp(cloud, metadata)
        if sample is not None:
            if not (sample.timestamp_ns - 1000 <= stamp <= sample.timestamp_ns + (sample.scan_duration_ns or 0) + 1000):
                continue
        elif map_cloud:
            # 地图系点已完成坐标注册；复用本地图最近的既有点云，不重复搬动它。
            if used_map_cloud or cloud_stamp > stamp + 1000:
                continue
        elif abs(cloud_stamp - stamp) > 1000:
            continue
        key = (metadata.get("source_id") or getattr(sample, "sensor_id", None), cloud_stamp, cloud.frame_id)
        if key in seen:
            continue
        points = cloud.points_xyz_array
        if points is None:
            points = np.asarray([[p.x, p.y, p.z] for p in cloud.points], dtype=np.float64)
        if not len(points):
            continue
        if sample is not None:
            if sample.time_offset_ns_le is None:
                continue
            offsets = np.frombuffer(sample.time_offset_ns_le, dtype="<u8").astype(np.int64)
            # 原始扫描各批属于不同机体时刻，只取与图像同时采集的一批。
            points = points[np.abs(offsets + int(sample.timestamp_ns) - stamp) <= 1000]
        if not len(points):
            continue
        transform = resolve_frame_transform(tree, source_frame_id=cloud.frame_id, target_frame_id=pose.frame_id)
        if transform is None:
            continue
        arrays.append(np.asarray(points) @ transform.matrix[:3, :3].T + transform.matrix[:3, 3])
        seen.add(key)
        used_map_cloud = used_map_cloud or map_cloud
        sources.append({"source_id": key[0], "frame_id": cloud.frame_id, "source_timestamp_ns": cloud_stamp,
                        "selected_point_count": len(points), "canonical_lidar": sample is not None})
    if not arrays:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")
    points = np.concatenate(arrays, axis=0)
    cloud = PointCloudFrame(frame_id=pose.frame_id, timestamp=image_frame.timestamp, point_count=len(points),
                            points_xyz_array=points, metadata={"source_timestamp_ns": stamp, "clock_id": clock})
    return ObservationGeometry(pose, tree, cloud, "", tuple(sources))
