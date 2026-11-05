"""场景几何的实际输入，只在区域写入或分割时比较。"""
from __future__ import annotations


def geometry_value(space):
    boundary = space.boundary
    if boundary is None:
        return (space.frame_id, None)
    def point(value):
        return None if value is None else (value.x, value.y, value.z)
    centroid = boundary.centroid
    pose = None if centroid is None else (centroid.frame_id, point(centroid.position),
        (centroid.orientation.x, centroid.orientation.y, centroid.orientation.z, centroid.orientation.w))
    metadata = boundary.metadata or {}
    return (space.frame_id, boundary.frame_id, tuple(point(p) for p in boundary.polygon_points),
        point(boundary.min_corner), point(boundary.max_corner), pose, boundary.area_m2,
        metadata.get('holes', []), metadata.get('room_geometry'))



def semantic_geometry_value(space):
    """标签沿用遵循原轮廓、坐标系和洞的规则。"""
    boundary = space.boundary
    return (space.frame_id,
        tuple((point.x, point.y, point.z) for point in boundary.polygon_points) if boundary else (),
        boundary.metadata.get('holes', []) if boundary else [])

def binding_value(binding):
    binding = binding or {}
    return tuple(binding.get(key) for key in ('map_name', 'frame_id')) + (
        str(binding.get('map_revision', '')),)


def graph_geometry_value(space):
    if not space.metadata.get('scene_mapping_managed') or space.review_status.value in ('stale', 'rejected'):
        return None
    return (geometry_value(space), binding_value(space.metadata.get('map_binding')))
