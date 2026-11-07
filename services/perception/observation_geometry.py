"""从已有流历史取同一采集时刻的几何输入，不用处理时刻的机体位置。"""
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from contracts.geometry import FrameTree, Pose, Transform, Vector3, Quaternion
from contracts.frame_semantics import frame_ids_semantically_equal
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
    value = metadata.get("source_timestamp_ns", getattr(payload, "timestamp_ns", None))
    return int(value) if value is not None else int(payload.timestamp.timestamp() * 1e9)


def contract_pose(value):
    """把 Geometry 的位姿值转成感知算法使用的坐标契约，保留源时间。"""
    if value is None or isinstance(value, Pose):
        return value
    return Pose(frame_id=value.frame_id,
        timestamp=datetime.fromtimestamp(value.timestamp_ns / 1e9, timezone.utc),
        position=Vector3(x=value.x, y=value.y, z=value.z),
        orientation=Quaternion(x=value.qx, y=value.qy, z=value.qz, w=value.qw))


def contract_frame_tree(transforms, pose):
    if pose is None:
        return None
    return FrameTree(root_frame_id=pose.frame_id, timestamp=pose.timestamp, transforms=[
        Transform(parent_frame_id=value.parent_frame, child_frame_id=value.child_frame,
            translation=Vector3(x=value.x, y=value.y, z=value.z),
            rotation=Quaternion(x=value.qx, y=value.qy, z=value.qz, w=value.qw),
            timestamp=datetime.fromtimestamp(value.timestamp_ns / 1e9, timezone.utc),
            authority="calibration_static" if value.timestamp_ns == 0 else "geometry")
        for value in transforms])


def capture_geometry(stream_bus, image_frame, *, transforms=(), map_name=None, point_cloud_frames=None):
    """复用有界历史；纳秒容差仅吸收现有浮点时间序列化误差。"""
    if stream_bus is None:
        return ObservationGeometry(reason="geometry_stream_unavailable")
    stamp = _stamp(image_frame, image_frame.metadata)
    clock = image_frame.metadata.get("source_clock_id") or image_frame.metadata.get("clock_id")
    frames = stream_bus.replay(topics=("localization.pose", "pointcloud.frame"))
    if point_cloud_frames is not None:
        frames = tuple(frame for frame in frames if frame.topic == "localization.pose") + tuple(point_cloud_frames)
    pose_frames = [f for f in frames if f.topic == "localization.pose"
                   and (map_name is None or f.metadata.get("map_name") == map_name)
                   and abs(_stamp(f.payload, f.metadata) - stamp) <= 1000]
    if not pose_frames:
        return ObservationGeometry()
    pose = contract_pose(pose_frames[-1].payload)
    original_tree = contract_frame_tree(transforms, pose)
    static = [t for t in original_tree.transforms if t.authority == "calibration_static"]
    camera_edge = next((t for t in static if t.child_frame_id == image_frame.frame_id), None)
    if camera_edge is None:
        return ObservationGeometry(pose=pose, reason="capture_camera_calibration_unavailable")
    base = camera_edge.parent_frame_id
    dynamic_bodies = {t.child_frame_id for t in original_tree.transforms if t.authority != "calibration_static"}
    if not any(frame_ids_semantically_equal(base, body) for body in dynamic_bodies):
        return ObservationGeometry(pose=pose, reason="capture_reference_frame_unavailable")
    tree = FrameTree(root_frame_id=pose.frame_id, timestamp=pose.timestamp, transforms=[
        Transform(parent_frame_id=pose.frame_id, child_frame_id=base, translation=pose.position,
                  rotation=pose.orientation, timestamp=pose.timestamp, authority="capture_localization_pose"), *static])
    selected = None
    for frame in frames:
        cloud = frame.payload
        if frame.topic != "pointcloud.frame" or not isinstance(cloud, PointCloudFrame):
            continue
        metadata = cloud.metadata
        if map_name is not None and frame.metadata.get("map_name") != map_name:
            continue
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
            if cloud_stamp > stamp + 1000:
                continue
        elif abs(cloud_stamp - stamp) > 1000:
            continue
        key = (metadata.get("source_id") or getattr(sample, "sensor_id", None), cloud_stamp, cloud.frame_id)
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
            distances = np.abs(offsets + int(sample.timestamp_ns) - stamp)
            paired = distances <= 1000
            points = points[paired]
            distance = int(distances[paired].min()) if len(points) else None
        else:
            distance = abs(cloud_stamp - stamp)
        if not len(points):
            continue
        rank = (distance, -cloud_stamp)
        if selected is None or rank < selected[0]:
            selected = (rank, cloud, points, key, sample is not None)
    if selected is None:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")
    _, source_cloud, points, key, canonical = selected
    transform = resolve_frame_transform(tree, source_frame_id=source_cloud.frame_id, target_frame_id=pose.frame_id)
    if transform is None:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")
    points = np.asarray(points) @ transform.matrix[:3, :3].T + transform.matrix[:3, 3]
    sources = ({"source_id": key[0], "frame_id": source_cloud.frame_id, "source_timestamp_ns": key[1],
                "selected_point_count": len(points), "canonical_lidar": canonical},)
    cloud = PointCloudFrame(frame_id=pose.frame_id, timestamp=image_frame.timestamp, point_count=len(points),
                            points_xyz_array=points, metadata={"source_timestamp_ns": stamp, "clock_id": clock})
    return ObservationGeometry(pose, tree, cloud, "", tuple(sources))
