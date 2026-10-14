"""Explicit scene geometry inputs, compared only at owner write/segmentation time."""
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
    """Label continuity follows the original contour/frame/hole contract."""
    boundary = space.boundary
    return (space.frame_id,
        tuple((point.x, point.y, point.z) for point in boundary.polygon_points) if boundary else (),
        boundary.metadata.get('holes', []) if boundary else [])

def binding_value(binding):
    binding = binding or {}
    return tuple(binding.get(key) for key in ('map_name', 'source_name', 'source_epoch', 'frame_id')) + (
        str(binding.get('update_sequence', '')),)


def graph_geometry_value(space):
    if not space.metadata.get('scene_mapping_managed') or space.review_status.value in ('stale', 'rejected'):
        return None
    return (geometry_value(space), binding_value(space.metadata.get('map_binding')))


# SQLite's date functions round fractional seconds. This collation keeps the
# supported contract parser and exact microseconds, and is only used on misses.
from datetime import datetime, timedelta
from pydantic import TypeAdapter, ValidationError

_TIMESTAMP_ADAPTER = TypeAdapter(datetime)


def _timestamp_order(value):
    try:
        timestamp = _TIMESTAMP_ADAPTER.validate_python(value)
    except ValidationError:
        # Malformed legacy fields sort after valid values in descending reads.
        # If selected, normal model decoding reports the invalid source; no row
        # is silently accepted as healthy or removed from storage.
        return (0, value)
    offset = timestamp.utcoffset() or timedelta(0)
    micros = ((timestamp.toordinal() * 86400 + timestamp.hour * 3600 + timestamp.minute * 60 + timestamp.second)
              * 1000000 + timestamp.microsecond)
    micros -= (offset.days * 86400 + offset.seconds) * 1000000 + offset.microseconds
    return (1, micros)


def compare_scene_timestamps(left, right):
    before, after = _timestamp_order(left), _timestamp_order(right)
    return (before > after) - (before < after)
