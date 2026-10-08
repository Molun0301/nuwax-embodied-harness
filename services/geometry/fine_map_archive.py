"""精细地图资产的结构与来源验证；按块读取，不计算内容摘要。"""
import json
import math
import struct

import numpy as np

from services.geometry.checkpoint_payload import FrozenPayload

MAX_FINE_ARCHIVE_BYTES = 1024 * 1024 * 1024
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


def validate_fine_archive(payload, *, session=None, generation=None, cloud_revision=None,
                          source_timestamp_ns=None, frame_generation=None, producer_generation=None):
    """读取 AR1/AR2 完整根并核对块结构和原提交；旧摘要字段仅保留为别名。"""
    if not isinstance(payload, (bytes, FrozenPayload)) or not 24 <= len(payload) <= MAX_FINE_ARCHIVE_BYTES:
        raise ValueError("精细地图资产类型或大小错误")
    magic = payload[:8]
    if magic not in (b"NUWAXAR1", b"NUWAXAR2"):
        raise ValueError("精细地图资产格式未知")
    legacy = magic == b"NUWAXAR1"
    root_offset = 48 if legacy else 16
    root_length = struct.unpack("<Q", payload[8:16])[0]
    if not 1 <= root_length <= 32 * 1024 * 1024 or root_offset + root_length + 8 > len(payload):
        raise ValueError("精细地图根长度错误")
    if legacy and not any(payload[16:48]):
        raise ValueError("精细地图旧根别名为空")
    root_bytes = payload[root_offset:root_offset + root_length]

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("精细地图根包含重复字段")
            result[key] = value
        return result

    def nonfinite(_):
        raise ValueError("精细地图根包含非有限数")

    root = json.loads(root_bytes, object_pairs_hook=unique, parse_constant=nonfinite)
    root_fields = {"version", "domain", "revision", "source_timestamp_ns", "voxel_size_m", "blocks"}
    if not legacy:
        root_fields.add("geometry_revision")
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

    version = 1 if legacy else 2
    integer(root["version"], version, version)
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
    offset = root_offset + root_length
    if struct.unpack("<Q", payload[offset:offset + 8])[0] != len(blocks):
        raise ValueError("精细地图资产块数错误")
    offset += 8
    previous = None
    object_points = {}
    object_offsets = {}
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
        if length != 16 + points * 12 or offset + length > len(payload):
            raise ValueError("精细地图块长度错误")
        block = payload[offset:offset + length]
        first_offset = object_offsets.setdefault(object_id, offset)
        if first_offset != offset and payload[first_offset:first_offset + length] != block:
            raise ValueError("精细地图同一对象身份的实际字节不一致")
        offset += length
        if block[:8] != b"NUWAXBL1" or struct.unpack("<I", block[8:12])[0] != points or block[12:16] != bytes(4):
            raise ValueError("精细地图块格式错误")
        if points < _FINE_VECTOR_MIN_POINTS:
            last = -1
            for local, stamp, hits in struct.iter_unpack("<HQH", block[16:]):
                if not last < local < 32768 or stamp > source_time or (stamp == 0) != (hits == 0):
                    raise ValueError("精细地图块观察证据错误")
                last = local
        else:
            _validate_vector_block_evidence(block, points, source_time)
    if offset != len(payload):
        raise ValueError("精细地图资产存在尾随数据")
    return root
