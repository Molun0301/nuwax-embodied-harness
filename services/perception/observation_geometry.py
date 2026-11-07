"""从已有流历史取同一采集时刻的几何输入，不用处理时刻的机体位置。"""
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from contracts.geometry import FrameTree, Pose, Transform, Vector3, Quaternion
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.pointcloud import PointCloudFrame
from services.localization.frame_tree_ops import resolve_frame_transform

_POSE_PAIR_WINDOW_NS = 200_000_000
_POINT_CLOUD_PAIR_WINDOW_NS = 100_000_000
_LATEST_POSE_WINDOW_NS = 100_000_000


@dataclass(frozen=True)
class ObservationGeometry:
    pose: object = None
    frame_tree: object = None
    point_cloud: object = None
    reason: str = "capture_pose_unavailable"
    sources: tuple = ()
    message: str = ""


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


def _interpolate_pose(samples, stamp):
    """只在前后来源位姿均落入窗口时插值，不外推。"""
    before = next((item for item in reversed(samples) if item[0] <= stamp), None)
    after = next((item for item in samples if item[0] >= stamp), None)
    if before is None or after is None or stamp-before[0] > _POSE_PAIR_WINDOW_NS or after[0]-stamp > _POSE_PAIR_WINDOW_NS:
        return None
    left, right = before[1], after[1]
    weight = 0.0 if before[0] == after[0] else (stamp-before[0])/(after[0]-before[0])
    position = tuple((1-weight)*getattr(left.position, axis)+weight*getattr(right.position, axis) for axis in "xyz")
    first = np.array([getattr(left.orientation, axis) for axis in "xyzw"], dtype=np.float64)
    second = np.array([getattr(right.orientation, axis) for axis in "xyzw"], dtype=np.float64)
    first, second = first/np.linalg.norm(first), second/np.linalg.norm(second)
    dot = float(np.dot(first, second))
    if dot < 0:
        second, dot = -second, -dot
    # 球面插值取最短弧；零角极限使用线性式，避免除以零。
    if dot > 1-1e-12:
        rotation = (1-weight)*first+weight*second
    else:
        angle = np.arccos(np.clip(dot, -1.0, 1.0))
        rotation = (np.sin((1-weight)*angle)*first+np.sin(weight*angle)*second)/np.sin(angle)
    rotation /= np.linalg.norm(rotation)
    return Pose(frame_id=left.frame_id, timestamp=datetime.fromtimestamp(stamp/1e9, timezone.utc),
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(**dict(zip("xyzw", rotation))))


def _pose_tree(pose, base, static):
    return FrameTree(root_frame_id=pose.frame_id, timestamp=pose.timestamp, transforms=[
        Transform(parent_frame_id=pose.frame_id, child_frame_id=base, translation=pose.position,
                  rotation=pose.orientation, timestamp=pose.timestamp, authority="capture_localization_pose"), *static])


def _pose_at_source_time(samples, stamp):
    """先用前后样本插值；晚于最新位姿不超过0.1秒时使用最新位姿。"""
    pose = _interpolate_pose(samples, stamp)
    if pose is not None:
        return pose
    if samples and 0 < stamp-samples[-1][0] <= _LATEST_POSE_WINDOW_NS:
        return samples[-1][1].model_copy(update={"timestamp": datetime.fromtimestamp(stamp/1e9, timezone.utc)})
    return None


def capture_geometry(stream_bus, image_frame, *, transforms=(), map_name=None, point_cloud_frames=None):
    """用来源时间窗口插值图像位姿，并将最近整帧扫描按批次配准到地图系。"""
    if stream_bus is None:
        return ObservationGeometry(reason="geometry_stream_unavailable")
    stamp = _stamp(image_frame, image_frame.metadata)
    clock = image_frame.metadata.get("source_clock_id") or image_frame.metadata.get("clock_id")
    frames = stream_bus.replay(topics=("localization.pose", "pointcloud.frame"))
    if point_cloud_frames is not None:
        frames = tuple(frame for frame in frames if frame.topic == "localization.pose") + tuple(point_cloud_frames)
    pose_samples = sorted(((_stamp(f.payload, f.metadata), contract_pose(f.payload)) for f in frames
                           if f.topic == "localization.pose" and (map_name is None or f.metadata.get("map_name") == map_name)),
                          key=lambda item: item[0])
    pose = _pose_at_source_time(pose_samples, stamp)
    if pose is None:
        return ObservationGeometry(message="图像时刻的位姿不可用")
    pose = pose.model_copy(update={"timestamp": image_frame.timestamp})
    original_tree = contract_frame_tree(transforms, pose)
    static = [t for t in original_tree.transforms if t.authority == "calibration_static"]
    camera_edge = next((t for t in static if t.child_frame_id == image_frame.frame_id), None)
    if camera_edge is None:
        return ObservationGeometry(pose=pose, reason="capture_camera_calibration_unavailable")
    base = camera_edge.parent_frame_id
    dynamic_bodies = {t.child_frame_id for t in original_tree.transforms if t.authority != "calibration_static"}
    if not any(frame_ids_semantically_equal(base, body) for body in dynamic_bodies):
        return ObservationGeometry(pose=pose, reason="capture_reference_frame_unavailable")
    tree = _pose_tree(pose, base, static)
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
        if map_cloud:
            # 地图系点已完成坐标注册；复用本地图最近的既有点云，不重复搬动它。
            if cloud_stamp > stamp:
                continue
        elif abs(cloud_stamp-stamp) > _POINT_CLOUD_PAIR_WINDOW_NS:
            continue
        key = (metadata.get("source_id") or getattr(sample, "sensor_id", None), cloud_stamp, cloud.frame_id)
        points = cloud.points_xyz_array
        if points is None:
            points = np.asarray([[p.x, p.y, p.z] for p in cloud.points], dtype=np.float64)
        if not len(points):
            continue
        rank = (abs(cloud_stamp-stamp), -cloud_stamp)
        if selected is None or rank < selected[0]:
            selected = (rank, cloud, points, key, sample, map_cloud)
    if selected is None:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")
    _, source_cloud, points, key, sample, map_cloud = selected
    batch_count = 0
    if not map_cloud:
        offsets = (np.frombuffer(sample.time_offset_ns_le, dtype="<u8").astype(np.int64)
                   if sample is not None and sample.time_offset_ns_le is not None else np.zeros(len(points), dtype=np.int64))
        scan_stamp = int(sample.timestamp_ns) if sample is not None else key[1]
        mapped = np.empty((len(points), 3), dtype=np.float64)
        for offset in np.unique(offsets):
            batch_pose = _pose_at_source_time(pose_samples, scan_stamp+int(offset))
            if batch_pose is None:
                return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable", message="点云批次时刻的位姿不可用")
            transform = resolve_frame_transform(_pose_tree(batch_pose, base, static),
                source_frame_id=source_cloud.frame_id, target_frame_id=pose.frame_id)
            if transform is None:
                return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")
            batch = offsets == offset
            mapped[batch] = np.asarray(points)[batch] @ transform.matrix[:3, :3].T+transform.matrix[:3, 3]
            batch_count += 1
        points = mapped
    sources = ({"source_id": key[0], "frame_id": source_cloud.frame_id, "source_timestamp_ns": key[1],
                "selected_point_count": len(points), "canonical_lidar": sample is not None,
                "point_batch_count": batch_count, "image_delta_ns": key[1]-stamp},)
    cloud = PointCloudFrame(frame_id=pose.frame_id, timestamp=image_frame.timestamp, point_count=len(points),
                            points_xyz_array=points, metadata={"source_timestamp_ns": stamp, "clock_id": clock})
    return ObservationGeometry(pose, tree, cloud, "", tuple(sources))
