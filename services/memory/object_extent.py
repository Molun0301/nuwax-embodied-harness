"""Bounded metric display geometry, independent of navigation target semantics.

A detector box describes its declared local axes. Point bounds describe only the
observed support in map axes, not a completed solid or its hidden dimensions.
Offsets are map-frame vectors from the instance's (possibly smoothed) pose;
fresh bounds are rebased to that pose so their measured map center stays exact.
"""
from __future__ import annotations

import math
from typing import Dict, Optional

from contracts.frame_semantics import frame_ids_semantically_equal

_AXES = ('x', 'y', 'z')
_IDENTITY = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def _vector(value, keys=_AXES):
    if not isinstance(value, dict):
        return None
    if not all(isinstance(value.get(key), (int, float)) and not isinstance(value[key], bool)
               and math.isfinite(value[key]) for key in keys):
        return None
    return {key: float(value[key]) for key in keys}


def _quaternion(value):
    result = _vector(value, (*_AXES, 'w'))
    if result is None:
        return None
    norm = math.sqrt(sum(item * item for item in result.values()))
    if not math.isfinite(norm) or norm < 1e-12:
        return None
    return {key: item / norm for key, item in result.items()}


def _same_frame(left, right):
    if not isinstance(left, str) or not left:
        return False
    try:
        return frame_ids_semantically_equal(left, right)
    except ValueError:
        return False


def _canonical(value, frame_id):
    if not isinstance(value, dict) or not _same_frame(value.get('frame_id'), frame_id):
        return None
    size = _vector(value.get('size_m'))
    offset = _vector(value.get('center_offset_m'))
    orientation = _quaternion(value.get('orientation'))
    source = value.get('source')
    if size is None or offset is None or orientation is None or source not in ('detection3d', 'observed_points'):
        return None
    if min(size.values()) < 0 or max(size.values()) <= 0:
        return None
    if source == 'detection3d' and min(size.values()) <= 0:
        return None
    return {'frame_id': frame_id, 'size_m': size, 'center_offset_m': offset,
            'orientation': orientation if source == 'detection3d' else dict(_IDENTITY),
            'source': source, 'source_observation_id': str(value.get('source_observation_id') or '')}


def resolve_object_extent(record) -> Optional[Dict[str, object]]:
    """Normalize current/legacy instance or projected-observation metric evidence."""
    pose = record.pose
    if pose is None:
        return None
    frame_id = pose.frame_id
    attributes = record.attributes or {}
    metadata = getattr(record, 'metadata', None) or {}
    for values in (metadata, attributes):
        extent = _canonical(values.get('object_extent'), frame_id)
        if extent is not None:
            return extent
    # A robot-position fallback does not establish object geometry, even if it
    # carries stale detector attributes. Canonical retained geometry above is OK.
    if attributes.get('projection_method') == 'current_pose_fallback' or metadata.get('observation_only'):
        return None
    if any(values.get('extent_projection_status') == 'frame_transform_unverified'
           or values.get('projection_degradation_reason') == 'camera_points_relative_pose_fallback'
           for values in (metadata, attributes)):
        return None
    source_id = str(getattr(record, 'source_observation_id', None) or metadata.get('last_source_observation_id') or '')
    for values in (metadata, attributes):
        for key in ('bbox_3d_map', 'object_3d_bbox'):
            box = values.get(key)
            if not isinstance(box, dict) or not _same_frame(box.get('frame_id'), frame_id):
                continue
            low, high = _vector(box.get('min')), _vector(box.get('max'))
            if low is not None and high is not None:
                size = {axis: high[axis] - low[axis] for axis in _AXES}
                center = {axis: (low[axis] + high[axis]) / 2 for axis in _AXES}
            else:
                size, center = _vector(box.get('extent_m')), _vector(box.get('center'))
            if size is None or center is None:
                continue
            extent = _canonical({'frame_id': frame_id, 'size_m': size,
                'center_offset_m': {axis: center[axis] - getattr(pose.position, axis) for axis in _AXES},
                'orientation': _IDENTITY, 'source': 'observed_points', 'source_observation_id': source_id}, frame_id)
            if extent is not None:
                return extent
    # Legacy detector dimensions already live in the projected pose's local axes.
    # Require the explicit detection3d provenance rather than image bbox size keys.
    if str(metadata.get('projection_method') or attributes.get('projection_method') or '') == 'detection3d_pose':
        return _canonical({'frame_id': frame_id,
            'size_m': {axis: attributes.get('size_' + axis + '_m') for axis in _AXES},
            'center_offset_m': dict.fromkeys(_AXES, 0.0), 'orientation': pose.orientation.model_dump(),
            'source': 'detection3d', 'source_observation_id': source_id}, frame_id)
    return None


def merge_object_extent(current, observation, *, pose=None) -> Optional[Dict[str, object]]:
    """Retain dimensions through weaker observations; never grow boxes by union."""
    previous = resolve_object_extent(current) if current is not None else None
    incoming = None if observation.attributes.get('projection_method') == 'current_pose_fallback' else resolve_object_extent(observation)
    if previous is not None and not frame_ids_semantically_equal(previous['frame_id'], observation.pose.frame_id):
        previous = None
    if incoming is None:
        return previous
    if previous is not None and previous['source'] == 'detection3d' and incoming['source'] == 'observed_points':
        return previous
    if pose is not None:
        # Keep the observed map-space midpoint exact even when the navigation
        # centroid is filtered across uneven surface samples. Retained dimensions
        # without fresh bounds instead follow the latest accepted object position.
        incoming['center_offset_m'] = {
            axis: incoming['center_offset_m'][axis] + getattr(observation.pose.position, axis) - getattr(pose.position, axis)
            for axis in _AXES
        }
    return incoming


def object_extent_payload(record) -> Optional[Dict[str, object]]:
    """The lightweight HTTP and stream contract; at most eight footprint corners."""
    extent = resolve_object_extent(record)
    if extent is None:
        return None
    center = {axis: getattr(record.pose.position, axis) + extent['center_offset_m'][axis] for axis in _AXES}
    if not all(math.isfinite(value) for value in center.values()):
        return None
    size, q = extent['size_m'], extent['orientation']
    x, y, z, w = (q[key] for key in ('x', 'y', 'z', 'w'))
    rotation = ((1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)))
    corners = []
    for sx in (-.5, .5):
        for sy in (-.5, .5):
            for sz in (-.5, .5):
                vector = (sx*size['x'], sy*size['y'], sz*size['z'])
                corners.append(tuple(center[axis] + sum(rotation[index][j]*vector[j] for j in range(3))
                                     for index, axis in enumerate(_AXES)))
    if not all(math.isfinite(value) for point in corners for value in point):
        return None
    # Monotone chain of the XY projection also handles tilted boxes and planes.
    points = sorted(set((point[0], point[1]) for point in corners))
    def half(sequence):
        result = []
        for point in sequence:
            while len(result) >= 2:
                a, b = result[-2:]
                if (b[0]-a[0])*(point[1]-a[1]) - (b[1]-a[1])*(point[0]-a[0]) > 0:
                    break
                result.pop()
            result.append(point)
        return result
    footprint = points if len(points) < 3 else half(points)[:-1] + half(reversed(points))[:-1]
    return {'frame_id': extent['frame_id'], 'center': center, 'size_m': size, 'orientation': q,
            'source': extent['source'], 'source_observation_id': extent['source_observation_id'],
            'footprint_xy': [{'x': point[0], 'y': point[1]} for point in footprint],
            'top_z_m': max(point[2] for point in corners)}
