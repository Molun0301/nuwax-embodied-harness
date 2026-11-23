"""用各帧的来源时间配准拍照前后的几何输入。"""
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from typing import Optional

import numpy as np

from contracts.geometry import FrameTree, Pose, Transform, Vector3, Quaternion
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.pointcloud import PointCloudFrame
from services.localization.frame_tree_ops import resolve_frame_transform

_POSE_PAIR_WINDOW_NS = 200_000_000
# Go2 雷达约 15.4 帧/秒，单帧只扫一个扇区，约 10 帧铺满相机视野。
_POINT_CLOUD_MERGE_WINDOW_NS = 350_000_000
_LATEST_POSE_WINDOW_NS = 100_000_000
# 按 1 米/秒和 1 弧度/秒估算，取 10 毫秒桶中心时几米内的误差为毫米至厘米级。
_POINT_TIME_BUCKET_NS = 10_000_000


@dataclass(frozen=True)
class ObservationGeometry:
    pose: object = None
    frame_tree: object = None
    point_cloud: object = None
    reason: str = "capture_pose_unavailable"
    sources: tuple = ()
    message: str = ""

    @property
    def point_batch_count(self) -> int:
        return sum(source["point_batch_count"] for source in self.sources)


def _stamp(payload, metadata):
    value = metadata.get("source_timestamp_ns", getattr(payload, "timestamp_ns", None))
    return int(value) if value is not None else int(payload.timestamp.timestamp() * 1e9)


def contract_pose(value):
    """把 Geometry 的位姿值转成感知算法使用的坐标契约，保留源时间。"""
    if value is None or isinstance(value, Pose):
        return value
    return Pose(frame_id=value.frame_id,
        timestamp=datetime.fromtimestamp(value.timestamp_ns / 1e9, timezone.utc),
        source_timestamp_ns=value.timestamp_ns, session_id=value.session_id, correction_seq=value.correction_seq,
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
                source_timestamp_ns=stamp, session_id=left.session_id, correction_seq=left.correction_seq,
                mixed_correction=(left.session_id, left.correction_seq) != (right.session_id, right.correction_seq),
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


def _pose_samples(frames, map_name):
    return tuple(sorted(((_stamp(f.payload, f.metadata), contract_pose(f.payload)) for f in frames
                         if f.topic == "localization.pose" and (map_name is None or f.metadata.get("map_name") == map_name)),
                        key=lambda item: item[0]))


def _capture_reference(image_frame, transforms, pose_samples):
    pose = _pose_at_source_time(pose_samples, _stamp(image_frame, image_frame.metadata))
    if pose is None:
        return ObservationGeometry(message="图像时刻的位姿不可用")
    pose = pose.model_copy(update={"timestamp": image_frame.timestamp})
    original_tree = contract_frame_tree(transforms, pose)
    static = [t for t in original_tree.transforms if t.authority == "calibration_static"]
    camera_edge = next((t for t in static if t.child_frame_id == image_frame.frame_id), None)
    if camera_edge is None:
        return ObservationGeometry(pose=pose, reason="capture_camera_calibration_unavailable")
    base = camera_edge.parent_frame_id
    dynamic_bodies = [t.child_frame_id for t in original_tree.transforms if t.authority != "calibration_static"]
    body = base if base in dynamic_bodies else next(
        (body for body in dynamic_bodies if frame_ids_semantically_equal(base, body)), None)
    if body is None:
        return ObservationGeometry(pose=pose, reason="capture_reference_frame_unavailable")
    if body != base:
        # 相机和雷达的静态外参可能挂在同一机身的不同名称下。
        static.append(Transform(parent_frame_id=base, child_frame_id=body,
                                timestamp=camera_edge.timestamp, authority="calibration_static"))
    tree = _pose_tree(pose, base, static)
    return ObservationGeometry(pose, tree, reason="")


def _matching_clouds(image_frame, frames, map_frame_id, map_name):
    stamp = _stamp(image_frame, image_frame.metadata)
    clock = image_frame.metadata.get("source_clock_id") or image_frame.metadata.get("clock_id")
    for frame in frames:
        cloud = frame.payload
        if frame.topic != "pointcloud.frame" or not isinstance(cloud, PointCloudFrame):
            continue
        metadata = cloud.metadata
        if map_name is not None and frame.metadata.get("map_name") != map_name:
            continue
        map_cloud = cloud.frame_id == map_frame_id and not metadata.get("canonical_lidar_frame")
        if clock and metadata.get("clock_id") != clock and not map_cloud:
            continue
        cloud_stamp = _stamp(cloud, metadata)
        if map_cloud:
            # 地图系点已完成坐标注册；复用本地图最近的既有点云，不重复搬动它。
            if cloud_stamp > stamp:
                continue
        elif abs(cloud_stamp-stamp) > _POINT_CLOUD_MERGE_WINDOW_NS:
            continue
        yield cloud_stamp, cloud, map_cloud


@dataclass(frozen=True)
class GeometryCapture:
    """同一图像的原始测量快照，投影线程只消费封存的输入。"""

    reference: ObservationGeometry
    pose_samples: tuple
    clouds: tuple
    map_name: Optional[str]

    @classmethod
    def before_detection(cls, stream_bus, image_frame, transforms, map_name, point_cloud_frames):
        if stream_bus is None:
            return cls(ObservationGeometry(reason="geometry_stream_unavailable"), (), (), map_name)
        poses = _pose_samples(stream_bus.replay(topics=("localization.pose",)), map_name)
        reference = _capture_reference(image_frame, transforms, poses)
        if reference.frame_tree is None:
            return cls(reference, poses, (), map_name)
        stamp = _stamp(image_frame, image_frame.metadata)
        clouds = tuple(row for row in _matching_clouds(image_frame, point_cloud_frames, reference.pose.frame_id, map_name)
                       if row[0] <= stamp)
        return cls(reference, poses, clouds, map_name)

    def after_detection(self, stream_bus, image_frame, point_cloud_frames):
        if self.reference.frame_tree is None:
            return self
        stamp = _stamp(image_frame, image_frame.metadata)
        clouds = tuple(row for row in _matching_clouds(image_frame, point_cloud_frames,
                                                        self.reference.pose.frame_id, self.map_name)
                       if row[0] > stamp)
        poses = dict(self.pose_samples)
        for time, pose in _pose_samples(stream_bus.replay(topics=("localization.pose",)), self.map_name):
            if time > stamp:
                poses[time] = pose
        return replace(self, pose_samples=tuple(sorted(poses.items())), clouds=self.clouds + clouds)

    def project(self, image_frame):
        return _project_geometry(image_frame, self.reference, self.pose_samples, self.clouds)


def capture_geometry(stream_bus, image_frame, *, transforms=(), map_name=None, point_cloud_frames=None):
    """用同一份可用帧重建几何，供独立配对调用使用。"""
    if stream_bus is None:
        return ObservationGeometry(reason="geometry_stream_unavailable")
    frames = stream_bus.replay(topics=("localization.pose", "pointcloud.frame"))
    poses = _pose_samples(frames, map_name)
    reference = _capture_reference(image_frame, transforms, poses)
    if reference.frame_tree is None:
        return reference
    clouds = _matching_clouds(image_frame, frames if point_cloud_frames is None else point_cloud_frames,
                              reference.pose.frame_id, map_name)
    return _project_geometry(image_frame, reference, poses, clouds)


def _project_geometry(image_frame, reference, pose_samples, clouds):
    if reference.frame_tree is None:
        return reference
    pose, tree = reference.pose, reference.frame_tree
    base = tree.transforms[0].child_frame_id
    static = tree.transforms[1:]
    stamp = _stamp(image_frame, image_frame.metadata)
    clock = image_frame.metadata.get("source_clock_id") or image_frame.metadata.get("clock_id")
    scans = []
    for cloud_stamp, cloud, map_cloud in clouds:
        sample = cloud.metadata.get("canonical_lidar_frame")
        points = cloud.points_xyz_array
        if points is None:
            points = np.asarray([[p.x, p.y, p.z] for p in cloud.points], dtype=np.float64)
        if not len(points):
            continue
        rank = (abs(cloud_stamp-stamp), -cloud_stamp)
        scans.append((rank, cloud, points, cloud_stamp, sample, map_cloud))
    if not scans:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable")

    # 仍由最接近图像时刻的帧决定使用地图系点云还是雷达扫描。
    nearest = min(scans, key=lambda scan: scan[0])
    use_map_cloud = nearest[-1]
    selected = (nearest,) if use_map_cloud else (scan for scan in scans if not scan[-1])
    registered, sources = [], []
    mixed = pose.mixed_correction
    message = ""
    for _, source_cloud, points, cloud_stamp, sample, map_cloud in selected:
        batch_count = 0
        if not map_cloud:
            if sample is not None and sample.time_offset_ns_le is not None:
                offsets = np.frombuffer(sample.time_offset_ns_le, dtype="<u8").astype(np.int64)
            elif source_cloud.metadata.get("offset_ns_array") is not None:
                # 显示/感知原始数组保留逐点时间，不为几何配对创建标准字节载荷。
                offsets = np.asarray(source_cloud.metadata["offset_ns_array"], dtype=np.int64)
            else:
                offsets = np.zeros(len(points), dtype=np.int64)
            scan_stamp = int(sample.timestamp_ns) if sample is not None else cloud_stamp
            point_buckets = offsets // _POINT_TIME_BUCKET_NS
            mapped = np.empty((len(points), 3), dtype=np.float64)
            scan_mixed = False
            for bucket in np.unique(point_buckets):
                center_offset = int(bucket) * _POINT_TIME_BUCKET_NS + _POINT_TIME_BUCKET_NS // 2
                batch_pose = _pose_at_source_time(pose_samples, scan_stamp + center_offset)
                if batch_pose is None:
                    message = "点云批次时刻的位姿不可用"
                    mapped = None
                    break
                transform = resolve_frame_transform(_pose_tree(batch_pose, base, static),
                    source_frame_id=source_cloud.frame_id, target_frame_id=pose.frame_id)
                if transform is None:
                    mapped = None
                    break
                batch = point_buckets == bucket
                mapped[batch] = np.asarray(points)[batch] @ transform.matrix[:3, :3].T+transform.matrix[:3, 3]
                scan_mixed |= batch_pose.mixed_correction or (
                    batch_pose.session_id, batch_pose.correction_seq) != (pose.session_id, pose.correction_seq)
                batch_count += 1
            if mapped is None:
                continue
            points = mapped
            mixed |= scan_mixed
        registered.append(points)
        sources.append({"source_id": source_cloud.metadata.get("source_id") or getattr(sample, "sensor_id", None),
                        "frame_id": source_cloud.frame_id, "source_timestamp_ns": cloud_stamp,
                        "selected_point_count": len(points), "canonical_lidar": sample is not None,
                        "point_batch_count": batch_count, "image_delta_ns": cloud_stamp-stamp})
    if not registered:
        return ObservationGeometry(pose, tree, reason="capture_pointcloud_unavailable", message=message)
    if not use_map_cloud:
        pose = pose.model_copy(update={"mixed_correction": mixed})
    points = registered[0] if len(registered) == 1 else np.concatenate(registered)
    cloud = PointCloudFrame(frame_id=pose.frame_id, timestamp=image_frame.timestamp, point_count=len(points),
                            points_xyz_array=points, metadata={"source_timestamp_ns": stamp, "clock_id": clock})
    return ObservationGeometry(pose, tree, cloud, "", tuple(sources))
