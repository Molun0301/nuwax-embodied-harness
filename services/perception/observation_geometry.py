"""将照片来源位姿与同一版本的累积地图点云配对。"""
from dataclasses import dataclass, replace
from datetime import datetime, timezone

import cv2
import numpy as np

from contracts.geometry import FrameTree, Pose, Transform, Vector3, Quaternion
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.pointcloud import PointCloudFrame
from services.geometry.client import GeometryError
from services.localization.frame_tree_ops import resolve_frame_transform

_POSE_PAIR_WINDOW_NS = 200_000_000
_LATEST_POSE_WINDOW_NS = 100_000_000
ACCUMULATED_CLOUD_MAX_DEPTH_M = 8.0
ACCUMULATED_CLOUD_UNAVAILABLE = "accumulated_cloud_unavailable"


@dataclass(frozen=True)
class ObservationGeometry:
    pose: object = None
    frame_tree: object = None
    point_cloud: object = None
    reason: str = "capture_pose_unavailable"
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


def _pose_samples(frames, map_name, map_frame_id):
    return tuple(sorted(((_stamp(f.payload, f.metadata), contract_pose(f.payload)) for f in frames
                         if f.topic == "localization.pose" and f.metadata.get("map_name") == map_name
                         and frame_ids_semantically_equal(f.payload.frame_id, map_frame_id)),
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
    tree = _pose_tree(pose, base, static)
    return ObservationGeometry(pose, tree, reason="")


def project_accumulated_cloud(reference, image_frame, camera_info, geometry_client):
    """用照片的相机坐标系确定地图取点区域，并核对这批点的校正版本。"""
    pose = reference.pose
    camera_to_map = resolve_frame_transform(
        reference.frame_tree, source_frame_id=camera_info.frame_id, target_frame_id=pose.frame_id,
    ).matrix
    width, height = image_frame.width_px, image_frame.height_px
    corners = np.array([(0, 0), (width, 0), (0, height), (width, height)], dtype=np.float64)
    center = np.array([camera_info.cx, camera_info.cy])
    focal = np.array([camera_info.fx, camera_info.fy])
    model = (camera_info.distortion_model or "").strip().lower()
    coefficients = camera_info.distortion_coefficients
    rays = (corners - center) / focal
    if model in {"fisheye", "equidistant", "kannala_brandt", "kb"}:
        distortion = np.asarray(coefficients or [0.0] * 4, dtype=np.float64)
        rays = cv2.fisheye.undistortPoints(rays[:, None, :], np.eye(3), distortion).reshape(-1, 2)
    elif any(coefficients):
        rays = cv2.undistortPoints(rays[:, None, :], np.eye(3), np.asarray(coefficients)).reshape(-1, 2)
    far_corners = np.column_stack((rays, np.ones(len(rays)))) * ACCUMULATED_CLOUD_MAX_DEPTH_M
    vertices = np.vstack((np.zeros(3), far_corners))
    map_vertices = vertices @ camera_to_map[:3, :3].T + camera_to_map[:3, 3]
    try:
        points, session_id, correction_seq = geometry_client.get_dense_cloud_region(
            map_vertices.min(axis=0), map_vertices.max(axis=0),
        )
    except GeometryError as error:
        return replace(reference, reason=ACCUMULATED_CLOUD_UNAVAILABLE, message=error.message)
    if pose.mixed_correction or (pose.session_id, pose.correction_seq) != (session_id, correction_seq):
        return replace(reference, reason=ACCUMULATED_CLOUD_UNAVAILABLE, message="照片位姿与完整点云版本不一致")
    cloud = PointCloudFrame(
        frame_id=pose.frame_id, timestamp=image_frame.timestamp, point_count=len(points),
        points_xyz_array=points, metadata={"source_topic": "geometry.dense_cloud_region"},
    )
    return replace(reference, point_cloud=cloud)
