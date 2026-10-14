"""Bounded structure-only planar archive reader; IDs are not content digests."""
import json
import math
import re
import struct

from services.geometry.checkpoint_payload import FrozenPayload

MAX_HEADER = 64 * 1024
MAX_MEMBER = 512 * 1024 * 1024
MAX_GRID = 16 * 1024 * 1024
MAX_ARCHIVE = 12 + MAX_HEADER + 2 * MAX_MEMBER + MAX_GRID


def _fields(value, names):
    if not isinstance(value, dict) or set(value) != set(names.split()):
        raise ValueError("planar archive fields mismatch")


def _integer(value, maximum, minimum=0):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError("planar archive integer out of range")


def _finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("planar archive number is not finite")


def _text(value, nonempty=True):
    if not isinstance(value, str) or (nonempty and not value.strip()) or len(value.encode('utf-8')) > 255 or '\0' in value:
        raise ValueError("planar archive text is invalid")


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate planar archive field")
        result[key] = value
    return result


def read_planar_structure(payload, *, checkpoint_id=None, session_id=None, generation=None,
                          timestamp_ns=None, occupancy=None, cells=None):
    return _read_planar_structure(payload, checkpoint_id=checkpoint_id, session_id=session_id,
        generation=generation, timestamp_ns=timestamp_ns, occupancy=occupancy, cells=cells)


def _read_planar_structure(payload, *, checkpoint_id=None, session_id=None, generation=None,
                           timestamp_ns=None, occupancy=None, cells=None, validate_grid=True):
    """Parse once, inspect bounded grid, and optionally bind canonical provenance.

    Legacy member aliases are parsed only. No graph body is scanned or copied.
    The caller retains the immutable bytes/FrozenPayload owner.
    """
    if not 12 <= len(payload) <= MAX_ARCHIVE or payload[:8] not in (b'NWS2D001', b'NWS2D002'):
        raise ValueError("unsupported planar archive")
    version = 2 if payload[:8] == b'NWS2D002' else 1
    header_size, = struct.unpack('<I', payload[8:12])
    if not 1 <= header_size <= MAX_HEADER or 12 + header_size > len(payload):
        raise ValueError("planar archive header budget exceeded")
    header = json.loads(payload[12:12 + header_size].decode('utf-8'), object_pairs_hook=_pairs,
                        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    assigned_id = None
    if version == 2:
        _fields(header, 'checkpoint_id source')
        assigned_id = header['checkpoint_id']
        if not isinstance(assigned_id, str) or re.fullmatch('[0-9a-f]{32}', assigned_id) is None or assigned_id == '0' * 32:
            raise ValueError("invalid assigned planar ID")
        source = header['source']
    else:
        source = header
    if checkpoint_id is not None and assigned_id != checkpoint_id:
        raise ValueError("planar checkpoint identity mismatch")
    _fields(source, 'version session estimate source_unix_timestamp_ns width height resolution origin members')
    if type(source['version']) is not int or source['version'] != version:
        raise ValueError("planar archive source version mismatch")
    session = source['session']
    _fields(session, 'session_id map_name session_generation sensor_id frame_id map_frame_id odom_frame_id body_frame_id clock_domain clock_id calibration_ref odometry_source core_config')
    for key in ('session_id', 'map_name', 'sensor_id', 'frame_id', 'map_frame_id', 'odom_frame_id', 'body_frame_id', 'calibration_ref'):
        _text(session[key])
    _text(session['clock_id'], False)
    if session['clock_domain'] not in ('unix_ns', 'monotonic_ns', 'device_ns') or (session['clock_domain'] != 'unix_ns' and not session['clock_id']):
        raise ValueError("planar clock identity invalid")
    if session['odometry_source'] not in ('device', 'laser') or not isinstance(session['core_config'], dict):
        raise ValueError("planar session configuration invalid")
    _integer(session['session_generation'], (1 << 64) - 1, 1)
    frames = [session[key].strip().lstrip('/') for key in ('map_frame_id', 'odom_frame_id', 'body_frame_id', 'frame_id')]
    extrinsic = session['core_config'].get('base_from_laser')
    if not isinstance(extrinsic, list) or len(extrinsic) != 3 or any(not frame for frame in frames) or len(set(frames[:3])) != 3 or frames[3] in frames[:2]:
        raise ValueError("planar frame configuration invalid")
    for value in extrinsic:
        _finite(value)
    if frames[3] == frames[2] and any(abs(value) > 1e-9 for value in extrinsic):
        raise ValueError("planar extrinsic identity mismatch")
    estimate = source['estimate']
    _fields(estimate, 'source_timestamp_ns clock_domain clock_id source_sequence graph_revision optimizations anchor_odom_from_base odom_from_base map_from_base map_from_odom covariance matching_response local_response disposition node_count edge_count')
    for key in ('source_timestamp_ns', 'source_sequence', 'graph_revision', 'optimizations'):
        _integer(estimate[key], (1 << 64) - 1, 1 if key == 'graph_revision' else 0)
    for key in ('node_count', 'edge_count'):
        _integer(estimate[key], (1 << 32) - 1)
    for key in ('clock_domain', 'clock_id', 'disposition'):
        if not isinstance(estimate[key], str):
            raise ValueError("invalid estimate text")
    for key in ('anchor_odom_from_base', 'odom_from_base', 'map_from_base', 'map_from_odom', 'covariance'):
        if not isinstance(estimate[key], list) or len(estimate[key]) != (9 if key == 'covariance' else 3):
            raise ValueError("invalid estimate vector")
        for value in estimate[key]:
            _finite(value)
    for key in ('matching_response', 'local_response'):
        _finite(estimate[key])
    _integer(source['source_unix_timestamp_ns'], (1 << 64) - 1, 1)
    for key in ('width', 'height'):
        _integer(source[key], (1 << 32) - 1, 1)
    _finite(source['resolution'])
    if source['resolution'] <= 0 or not isinstance(source['origin'], list) or len(source['origin']) != 2:
        raise ValueError("invalid planar geometry")
    for value in source['origin']:
        _finite(value)
    members = source['members']
    if not isinstance(members, list) or len(members) != 3:
        raise ValueError("invalid planar members")
    offset = 12 + header_size
    grid_offset = None
    for index, name in enumerate(('graph.posegraph', 'graph.data', 'grid.i8')):
        member = members[index]
        _fields(member, 'name size' if version == 2 else 'name size sha256')
        if member['name'] != name:
            raise ValueError("invalid planar member role")
        _integer(member['size'], MAX_GRID if index == 2 else MAX_MEMBER, 1)
        if version == 1:
            alias = member['sha256']
            if not isinstance(alias, list) or len(alias) != 32 or not any(alias):
                raise ValueError("invalid legacy member alias")
            for value in alias:
                _integer(value, 255)
        if index == 2:
            grid_offset = offset
        offset += member['size']
    if offset != len(payload) or members[2]['size'] != source['width'] * source['height']:
        raise ValueError("planar archive length mismatch")
    if ((session_id is not None and session['session_id'] != session_id)
            or (generation is not None and session['session_generation'] != generation)
            or (timestamp_ns is not None and source['source_unix_timestamp_ns'] != timestamp_ns)):
        raise ValueError("planar archive provenance mismatch")
    if occupancy is not None:
        expected = (occupancy['width'], occupancy['height'], occupancy['resolution'], [occupancy['origin_x'], occupancy['origin_y']], occupancy['frame_id'])
        actual = (source['width'], source['height'], struct.unpack('<f', struct.pack('<f', source['resolution']))[0], source['origin'], session['map_frame_id'])
        if actual != expected:
            raise ValueError("planar canonical geometry mismatch")
    if cells is not None and len(cells) != members[2]['size']:
        raise ValueError("planar canonical grid size mismatch")
    if not validate_grid:
        return header, 12 + header_size, grid_offset
    for position in range(grid_offset, offset, 1024 * 1024):
        block = payload[position:min(position + 1024 * 1024, offset)]
        if any(value not in (0, 100, 255) for value in block):
            raise ValueError("planar grid has invalid cell class")
        if cells is not None and block != cells[position - grid_offset:position - grid_offset + len(block)]:
            raise ValueError("planar canonical grid differs from archive")
    return header


def assign_legacy_planar_archive(payload, checkpoint_id):
    """Reseal an old immutable archive using its caller-owned assigned candidate ID.

    Preserve the old owner. Parse historical aliases without executing them;
    copy each member once and validate grid classes during that same copy.
    This helper does not allocate IDs, persist assets, or publish selectors.
    """
    if not isinstance(payload, (bytes, FrozenPayload)) or payload[:8] != b"NWS2D001":
        raise ValueError("legacy planar resealing requires NWS2D001")
    if not isinstance(checkpoint_id, str) or re.fullmatch('[0-9a-f]{32}', checkpoint_id) is None or checkpoint_id == '0' * 32:
        raise ValueError("invalid assigned planar ID")
    source, start, grid_offset = _read_planar_structure(payload, validate_grid=False)
    source = dict(source, version=2, members=[{'name': member['name'], 'size': member['size']} for member in source['members']])
    header = json.dumps({'checkpoint_id': checkpoint_id, 'source': source},
        separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')
    if not 1 <= len(header) <= MAX_HEADER:
        raise ValueError("assigned planar header exceeds budget")
    def chunks():
        yield b'NWS2D002' + struct.pack('<I', len(header)) + header
        for position in range(start, len(payload), 1024 * 1024):
            block = payload[position:min(position + 1024 * 1024, len(payload))]
            if position + len(block) > grid_offset:
                grid = block[max(0, grid_offset - position):]
                if any(value not in (0, 100, 255) for value in grid):
                    raise ValueError("planar grid has invalid cell class")
            yield block
    return FrozenPayload(chunks(), maximum_bytes=MAX_ARCHIVE)
