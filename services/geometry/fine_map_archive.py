"""精细地图资产的结构与来源验证；按块读取，不计算内容摘要。"""
import json
import math
import struct

import numpy as np

from services.geometry.checkpoint_payload import FrozenPayload

MAX_FINE_ARCHIVE_BYTES = 1024 * 1024 * 1024
_MAX_LAYER_BLOCK_BYTES = 4 * 1024 * 1024
_FINE_RECORD_DTYPE = np.dtype([("local", "<u2"), ("stamp", "<u8"), ("hits", "<u2")])


# Both NumPy 1.24.4/2.4.4 cross near 256 records; 512 has a clear margin.
# This selects computation only; the archive admission bound stays 32768.
_FINE_VECTOR_MIN_POINTS = 512


def _validate_vector_block_evidence(block, points, source_time):
    # Views only retain this already bounded block for the duration of the call.
    records = np.frombuffer(block, dtype=_FINE_RECORD_DTYPE, count=points, offset=16)
    local, stamp, hits = records["local"], records["stamp"], records["hits"]
    if (np.any(local >= 32768) or np.any(local[1:] <= local[:-1])
            or np.any(stamp > np.uint64(source_time))
            or np.any((stamp == 0) != (hits == 0))):
        raise ValueError("精细地图块观察证据错误")


def _unique_json_fields(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("精细地图根包含重复字段")
        result[key] = value
    return result


def _reject_nonfinite_json(_):
    raise ValueError("精细地图根包含非有限数")


def read_fine_archive_root(payload):
    """Parse only bounded root JSON, returning magic, root and block offset.

    This is not archive admission: root semantics, blocks and source bindings
    still require validate_fine_archive before the payload can be accepted.
    """
    if not isinstance(payload, (bytes, FrozenPayload)) or not 24 <= len(payload) <= MAX_FINE_ARCHIVE_BYTES:
        raise ValueError("精细地图资产类型或大小错误")
    magic = payload[:8]
    if magic not in (b"NUWAXAR1", b"NUWAXAR2", b"NUWAXAR3", b"NUWAXAR4", b"NUWAXAR5"):
        raise ValueError("精细地图资产格式未知")
    legacy = magic == b"NUWAXAR1"
    root_offset = 48 if legacy else 16
    root_length = struct.unpack("<Q", payload[8:16])[0]
    if not 1 <= root_length <= 32 * 1024 * 1024 or root_offset + root_length + 8 > len(payload):
        raise ValueError("精细地图根长度错误")
    if legacy and not any(payload[16:48]):
        raise ValueError("精细地图旧根别名为空")
    root_bytes = payload[root_offset:root_offset + root_length]

    root = json.loads(root_bytes, object_pairs_hook=_unique_json_fields, parse_constant=_reject_nonfinite_json)
    return magic, root, root_offset + root_length


def validate_fine_archive(payload, *, session=None, generation=None, cloud_revision=None,
                          source_timestamp_ns=None, frame_generation=None, producer_generation=None):
    """读取 AR1–AR5 完整根并核对块结构和原提交；旧摘要字段仅保留为别名。"""
    magic, root, offset = read_fine_archive_root(payload)
    legacy = magic == b"NUWAXAR1"
    root_fields = {"version", "domain", "revision", "source_timestamp_ns", "voxel_size_m", "blocks"}
    if not legacy:
        root_fields.add("geometry_revision")
    version = int(chr(magic[-1]))
    if version >= 4 and isinstance(root, dict) and "source_watermarks" in root:
        root_fields.add("source_watermarks")
    if not isinstance(root, dict) or set(root) != root_fields:
        raise ValueError("精细地图根字段错误")

    def integer(value, low, high):
        if type(value) is not int or not low <= value <= high:
            raise ValueError("精细地图整数范围错误")
        return value

    def identity_bytes(value, size):
        if not isinstance(value, list) or len(value) != size:
            raise ValueError("精细地图对象身份尺寸错误")
        result = bytes(integer(v, 0, 255) for v in value)
        if not any(result):
            raise ValueError("精细地图对象身份为空")
        return result

    integer(root["version"], version, version)
    if version >= 4:
        _validate_source_watermarks(root.get("source_watermarks", []), integer)
    revision = integer(root["revision"], 1, 2 ** 64 - 1)
    source_time = integer(root["source_timestamp_ns"], 1, 2 ** 64 - 1)
    if not legacy:
        integer(root["geometry_revision"], 1, revision)
    size = root["voxel_size_m"]
    if type(size) not in (int, float) or not 0 < size <= 1048576 or not math.isfinite(size):
        raise ValueError("精细地图分辨率错误")
    domain = root["domain"]
    if not isinstance(domain, dict) or set(domain) != {"session", "generation", "frame_generation", "producer_generation"}:
        raise ValueError("精细地图域字段错误")
    name = domain["session"]
    if not isinstance(name, str) or not name or name.strip() != name or len(name.encode("utf-8")) > 1024 or any(ord(c) < 32 or 127 <= ord(c) < 160 for c in name):
        raise ValueError("精细地图会话错误")
    for field in ("generation", "producer_generation"):
        integer(domain[field], 1, 2 ** 64 - 1)
    integer(domain["frame_generation"], 0, 2 ** 64 - 1)
    for actual, expected in ((name, session), (domain["generation"], generation),
                             (domain["frame_generation"], frame_generation),
                             (domain["producer_generation"], producer_generation),
                             (revision, cloud_revision), (source_time, source_timestamp_ns)):
        if expected is not None and (type(expected) is not type(actual) or actual != expected):
            raise ValueError("精细地图资产与原占据/点云提交不一致")
    blocks = root["blocks"]
    if not isinstance(blocks, list) or len(blocks) > 131072:
        raise ValueError("精细地图块目录超预算")
    if struct.unpack("<Q", payload[offset:offset + 8])[0] != len(blocks):
        raise ValueError("精细地图资产块数错误")
    offset += 8
    heightfields = {}
    previous = None
    object_points = {}
    object_offsets = {}
    ground_bindings = {}
    reference_fields = ({"key", "sha256", "geometry_sha256", "points"} if legacy else
                        {"key", "object", "block_revision", "geometry_revision", "points"})
    for reference in blocks:
        if not isinstance(reference, dict) or set(reference) != reference_fields:
            raise ValueError("精细地图块引用字段错误")
        key = reference["key"]
        if not isinstance(key, list) or len(key) != 3:
            raise ValueError("精细地图块坐标错误")
        key = tuple(integer(v, -(2 ** 26), 2 ** 26 - 1) for v in key)
        if previous is not None and key <= previous:
            raise ValueError("精细地图块重复或无序")
        previous = key
        points = integer(reference["points"], 1, 32768)
        if legacy:
            object_id = ("legacy", identity_bytes(reference["sha256"], 32))
            identity_bytes(reference["geometry_sha256"], 32)
        else:
            object_ref = reference["object"]
            if not isinstance(object_ref, dict) or set(object_ref) != {"kind", "value"}:
                raise ValueError("精细地图对象身份字段错误")
            kind = object_ref["kind"]
            if kind not in ("assigned", "legacy"):
                raise ValueError("精细地图对象身份种类错误")
            object_id = (kind, identity_bytes(object_ref["value"], 16 if kind == "assigned" else 32))
            block_revision = integer(reference["block_revision"], 1, revision)
            integer(reference["geometry_revision"], 1, min(block_revision, root["geometry_revision"]))
        if object_points.setdefault(object_id, points) != points:
            raise ValueError("精细地图同一对象记录数不一致")
        if offset + 4 > len(payload):
            raise ValueError("精细地图块被截断")
        length = struct.unpack("<I", payload[offset:offset + 4])[0]
        offset += 4
        records_end = 16 + points * 12
        if (length < records_end or length > (_MAX_LAYER_BLOCK_BYTES if version >= 3 else records_end)
                or offset + length > len(payload)):
            raise ValueError("精细地图块长度错误")
        block = payload[offset:offset + length]
        first_offset, first_length = object_offsets.setdefault(object_id, (offset, length))
        if first_offset != offset and (first_length != length or payload[first_offset:first_offset + length] != block):
            raise ValueError("精细地图同一对象身份的实际字节不一致")
        offset += length
        sourced = version >= 4 and block[:8] == b"NUWAXBL3"
        versioned = version >= 5 and block[:8] == b"NUWAXBL4"
        layered = (version >= 3 and block[:8] == b"NUWAXBL2") or sourced or versioned
        layer_length = struct.unpack("<I", block[12:16])[0]
        if (block[:8] != b"NUWAXBL1" and not layered
                or struct.unpack("<I", block[8:12])[0] != points
                or length != records_end + layer_length or (not layered and layer_length != 0)):
            raise ValueError("精细地图块格式错误")
        if points < _FINE_VECTOR_MIN_POINTS:
            last = -1
            for local, stamp, hits in struct.iter_unpack("<HQH", memoryview(block)[16:records_end]):
                if not last < local < 32768 or stamp > source_time or (stamp == 0) != (hits == 0):
                    raise ValueError("精细地图块观察证据错误")
                last = local
        else:
            _validate_vector_block_evidence(block, points, source_time)
        if layered:
            layers = json.loads(block[records_end:], object_pairs_hook=_unique_json_fields, parse_constant=_reject_nonfinite_json)
            if versioned:
                if ground_bindings.setdefault(object_id, (key, size)) != (key, size):
                    raise ValueError("精细地图地表对象空间绑定改变")
                evidence = {local: stamp for local, stamp, _ in struct.iter_unpack("<HQH", memoryview(block)[16:records_end])}
                _validate_ground_records(layers, integer, key, evidence, heightfields)
            else:
                _validate_layer_structure(layers, integer, sourced=sourced)
    if offset != len(payload):
        raise ValueError("精细地图资产存在尾随数据")
    if version >= 5:
        _validate_heightfield_columns(heightfields, size)
    return root


def _validate_observation_source(source, integer):
    """Return a compact identity key; Legacy samples have no source group."""
    if not isinstance(source, dict):
        raise ValueError("精细地图观测来源格式错误")
    kind = source.get("kind")
    if kind == "lidar" and set(source) == {"kind", "backend", "runtime_generation", "sensor_frame_id"}:
        backend = integer(source["backend"], 1, 3)
        generation = integer(source["runtime_generation"], 1, 2 ** 63 - 1)
        frame = source["sensor_frame_id"]
        if isinstance(frame, str) and frame:
            return kind, backend, generation, frame
    elif kind == "auxiliary" and set(source) == {"kind", "sensor_id", "clock_id", "runtime_generation", "session_generation"}:
        runtime = integer(source["runtime_generation"], 1, 2 ** 64 - 1)
        session = integer(source["session_generation"], 1, 2 ** 64 - 1)
        sensor, clock = source["sensor_id"], source["clock_id"]
        if isinstance(sensor, str) and sensor.strip() and isinstance(clock, str) and clock.strip():
            return kind, sensor, clock, runtime, session
    raise ValueError("精细地图观测来源格式错误")


def _validate_source_watermarks(watermarks, integer):
    if not isinstance(watermarks, list):
        raise ValueError("精细地图来源水位格式错误")
    sources = set()
    for watermark in watermarks:
        if not isinstance(watermark, dict) or set(watermark) != {"source", "source_timestamp_ns", "source_sequence"}:
            raise ValueError("精细地图来源水位格式错误")
        source = _validate_observation_source(watermark["source"], integer)
        if source in sources:
            raise ValueError("精细地图来源水位重复")
        sources.add(source)
        integer(watermark["source_timestamp_ns"], 1, 2 ** 64 - 1)
        integer(watermark["source_sequence"], 1 if source[0] == "lidar" else 0, 2 ** 64 - 1)


def _validate_layer_structure(layers, integer, *, sourced=False):
    """Read BL2/BL3 wire shapes; physical layer semantics belong to Rust."""
    def fail():
        raise ValueError("精细地图层格式错误")

    def vector(value):
        if (not isinstance(value, list) or len(value) != 3
                or any(type(v) not in (int, float) or not -1.7976931348623157e308 <= v <= 1.7976931348623157e308 for v in value)):
            fail()

    if not isinstance(layers, list) or not 1 <= len(layers) <= 32768:
        fail()
    for entry in layers:
        if not isinstance(entry, list) or len(entry) != 2:
            fail()
        anchor, layer = entry
        integer(anchor, 0, 65535)
        fields = {"members", "plane_origin", "normal", "samples"}
        if sourced and isinstance(layer, dict) and "sample_sources" in layer:
            fields.add("sample_sources")
        if not isinstance(layer, dict) or set(layer) != fields:
            fail()
        members, samples = layer["members"], layer["samples"]
        if (not isinstance(members, list) or not 1 <= len(members) <= 32
                or not isinstance(samples, list) or not 1 <= len(samples) <= 32):
            fail()
        for member in members:
            integer(member, 0, 65535)
        vector(layer["normal"])
        vector(layer["plane_origin"])
        for sample in samples:
            if not isinstance(sample, dict) or set(sample) != {"point", "source_sequence", "source_timestamp_ns", "point_index"}:
                fail()
            vector(sample["point"])
            integer(sample["source_sequence"], 0, 2 ** 64 - 1)
            integer(sample["source_timestamp_ns"], 0, 2 ** 64 - 1)
            integer(sample["point_index"], 0, 2 ** 32 - 1)
        if sourced:
            groups = layer.get("sample_sources", [])
            if not isinstance(groups, list) or len(groups) > len(samples):
                fail()
            sources, positions = set(), set()
            for group in groups:
                if not isinstance(group, dict) or set(group) != {"source", "sample_positions"}:
                    fail()
                source = _validate_observation_source(group["source"], integer)
                if source in sources:
                    fail()
                sources.add(source)
                indexes = group["sample_positions"]
                if not isinstance(indexes, list) or not 1 <= len(indexes) <= len(samples):
                    fail()
                for position in indexes:
                    integer(position, 0, len(samples) - 1)
                    if position in positions:
                        fail()
                    positions.add(position)
                    sample = samples[position]
                    integer(sample["source_timestamp_ns"], 1, 2 ** 64 - 1)
                    integer(sample["source_sequence"], 1 if source[0] == "lidar" else 0, 2 ** 64 - 1)


def _validate_ground_records(records, integer, block_key, evidence, columns):
    """BL4 data boundary; current root members define the declared surface."""
    if not isinstance(records, list) or not 1 <= len(records) <= 32768:
        raise ValueError("精细地图地表记录数量错误")
    last, claimed = -1, set()
    for entry in records:
        if not isinstance(entry, list) or len(entry) != 2:
            raise ValueError("精细地图地表记录错误")
        local, tagged = entry
        integer(local, 0, 32767)
        if local <= last or not isinstance(tagged, dict) or set(tagged) != {"representation", "data"}:
            raise ValueError("精细地图地表记录顺序或标签错误")
        last = local
        value = tagged["data"]
        if tagged["representation"] == "legacy_plane":
            _validate_layer_structure([[local, value]], integer, sourced=True)
            members = value["members"]
        elif tagged["representation"] == "heightfield_v1":
            if not isinstance(value, dict) or set(value) != {"anchor", "height_m", "occupied_bounds", "members", "samples"}:
                raise ValueError("精细地图高程片字段错误")
            anchor, bounds, pairs, samples = (value[k] for k in ("anchor", "occupied_bounds", "members", "samples"))
            if not isinstance(anchor, list) or len(anchor) != 3:
                raise ValueError("精细地图高程锚错误")
            anchor = tuple(integer(v, -(2 ** 31), 2 ** 31 - 1) for v in anchor)
            h = value["height_m"]
            if (type(h) not in (int, float) or not math.isfinite(h)
                    or not isinstance(bounds, list) or len(bounds) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)
                    or not bounds[0] < bounds[1]
                    or not isinstance(pairs, list) or not 1 <= len(pairs) <= 32
                    or not isinstance(samples, list) or len(samples) > 32):
                raise ValueError("精细地图高程片数值错误")
            group = columns.setdefault(anchor, (h, bounds, {}))
            if group[:2] != (h, bounds):
                raise ValueError("精细地图高程片几何不一致")
            members = []
            for pair in pairs:
                if not isinstance(pair, list) or len(pair) != 2:
                    raise ValueError("精细地图高程成员错误")
                member, bins = pair
                integer(member, 0, 32767)
                if (not isinstance(bins, list) or len(bins) != 8
                        or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in bins)):
                    raise ValueError("精细地图高程质量错误")
                xyz = (block_key[0] * 32 + member % 32,
                       block_key[1] * 32 + member // 32 % 32,
                       block_key[2] * 32 + member // 1024)
                if xyz[:2] != anchor[:2] or xyz[2] in group[2]:
                    raise ValueError("精细地图高程成员重叠或异列")
                group[2][xyz[2]] = bins
                members.append(member)
            identities = set()
            for sample in samples:
                if not isinstance(sample, dict) or set(sample) != {"source", "point", "source_sequence", "source_timestamp_ns", "point_index"}:
                    raise ValueError("精细地图高程诊断样本错误")
                source = sample["source"]
                source_id = (("legacy",) if source == {"kind": "legacy"}
                             else _validate_observation_source(source, integer))
                sequence = integer(sample["source_sequence"], 0 if source_id[0] == "auxiliary" else 1, 2 ** 64 - 1)
                stamp = integer(sample["source_timestamp_ns"], 1, 2 ** 64 - 1)
                index = integer(sample["point_index"], 0, 2 ** 32 - 1)
                point = sample["point"]
                if (not isinstance(point, list) or len(point) != 3
                        or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)
                        or stamp > max((evidence.get(m, 0) for m in members), default=0)
                        or (source_id, sequence, stamp, index) in identities):
                    raise ValueError("精细地图高程诊断样本证据错误")
                identities.add((source_id, sequence, stamp, index))
        else:
            raise ValueError("精细地图地表表示未知")
        if (members[0] != local or members != sorted(set(members))
                or any(m % 1024 != local % 1024 or m not in evidence or m in claimed for m in members)):
            raise ValueError("精细地图地表成员归属错误")
        claimed.update(members)


def _validate_heightfield_columns(columns, resolution):
    # The original Rust root stores f32 resolution. Match that value before f64
    # moments; eight sub-bins encode statistics, not extra measurement precision.
    q = struct.unpack("<f", struct.pack("<f", resolution))[0]
    if q <= 0:
        raise ValueError("精细地图分辨率错误")
    for anchor, (declared_height, bounds, members) in columns.items():
        ordered = sorted(members.items())
        total = sum(mass for _, bins in ordered for mass in bins)
        rounding = 2.220446049250313e-16 * (len(ordered) * 8 + 1)
        if (not math.isfinite(total) or total <= 0 or abs(total - 1) > rounding
                or anchor[2] != ordered[0][0]
                or bounds != [ordered[0][0] * q, (ordered[-1][0] + 1) * q]):
            raise ValueError("精细地图完整高程列质量或范围错误")
        low, high = total * .1, total * .9
        cumulative = moment = retained = 0.0
        width = q / 8
        for z, bins in ordered:
            for i, mass in enumerate(bins):
                end = cumulative + mass
                keep = max(0.0, min(end, high) - max(cumulative, low))
                centre = (z * 8 + i + .5) * width
                moment += keep * centre
                retained += keep
                cumulative = end
        height = (math.floor((moment / retained) / width) + .5) * width
        if height != declared_height:
            raise ValueError("精细地图完整高程列代表高度错误")
