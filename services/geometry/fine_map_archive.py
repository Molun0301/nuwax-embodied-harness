"""精细地图资产的独立字节验证；按块读取，不展开整张点云。"""
import hashlib
import json
import math
import struct

from services.geometry.checkpoint_payload import FrozenPayload

MAX_FINE_ARCHIVE_BYTES = 1024 * 1024 * 1024


def validate_fine_archive(payload, *, session=None, generation=None, cloud_revision=None, source_timestamp_ns=None):
    """核对完整根、每个块和原提交；返回仍保持原始数值的根清单。"""
    if not isinstance(payload, (bytes, FrozenPayload)) or not 56 <= len(payload) <= MAX_FINE_ARCHIVE_BYTES:
        raise ValueError("精细地图资产类型或大小错误")
    if payload[:8] != b"NUWAXAR1":
        raise ValueError("精细地图资产格式未知")
    root_length = struct.unpack("<Q", payload[8:16])[0]
    if not 1 <= root_length <= 32 * 1024 * 1024 or root_length + 56 > len(payload):
        raise ValueError("精细地图根长度错误")
    root_bytes = payload[48:48 + root_length]
    if hashlib.sha256(root_bytes).digest() != payload[16:48]:
        raise ValueError("精细地图根摘要错误")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("精细地图根包含重复字段")
            result[key] = value
        return result

    root = json.loads(root_bytes, object_pairs_hook=unique, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("精细地图根包含非有限数")))
    if not isinstance(root, dict) or set(root) != {"version", "domain", "revision", "source_timestamp_ns", "voxel_size_m", "blocks"}:
        raise ValueError("精细地图根字段错误")

    def integer(value, low, high):
        if type(value) is not int or not low <= value <= high:
            raise ValueError("精细地图整数范围错误")
        return value

    def digest_bytes(value):
        if not isinstance(value, list) or len(value) != 32:
            raise ValueError("精细地图摘要尺寸错误")
        result = bytes(integer(v, 0, 255) for v in value)
        if not any(result):
            raise ValueError("精细地图摘要为空")
        return result

    if integer(root["version"], 1, 1) != 1:
        raise ValueError("精细地图根版本错误")
    integer(root["revision"], 1, 2 ** 64 - 1)
    integer(root["source_timestamp_ns"], 1, 2 ** 64 - 1)
    size = root["voxel_size_m"]
    if type(size) not in (int, float) or not math.isfinite(size) or not 0 < size <= 1048576:
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
    for actual, expected in ((name, session), (domain["generation"], generation), (root["revision"], cloud_revision), (root["source_timestamp_ns"], source_timestamp_ns)):
        if expected is not None and actual != expected:
            raise ValueError("精细地图资产与原占据/点云提交不一致")
    blocks = root["blocks"]
    if not isinstance(blocks, list) or len(blocks) > 131072:
        raise ValueError("精细地图块目录超预算")
    offset = 48 + root_length
    if struct.unpack("<Q", payload[offset:offset + 8])[0] != len(blocks):
        raise ValueError("精细地图资产块数错误")
    offset += 8
    previous = None
    for reference in blocks:
        if not isinstance(reference, dict) or set(reference) != {"key", "sha256", "geometry_sha256", "points"}:
            raise ValueError("精细地图块引用字段错误")
        key = reference["key"]
        if not isinstance(key, list) or len(key) != 3:
            raise ValueError("精细地图块坐标错误")
        key = tuple(integer(v, -(2 ** 26), 2 ** 26 - 1) for v in key)
        if previous is not None and key <= previous:
            raise ValueError("精细地图块重复或无序")
        previous = key
        points = integer(reference["points"], 1, 32768)
        expected_hash = digest_bytes(reference["sha256"])
        geometry_hash = digest_bytes(reference["geometry_sha256"])
        if offset + 4 > len(payload):
            raise ValueError("精细地图块被截断")
        length = struct.unpack("<I", payload[offset:offset + 4])[0]
        offset += 4
        if length != 16 + points * 12 or offset + length > len(payload):
            raise ValueError("精细地图块长度错误")
        block = payload[offset:offset + length]
        offset += length
        if hashlib.sha256(block).digest() != expected_hash or block[:8] != b"NUWAXBL1" or struct.unpack("<I", block[8:12])[0] != points or block[12:16] != bytes(4):
            raise ValueError("精细地图块摘要或格式错误")
        geometry = hashlib.sha256()
        last = -1
        for local, stamp, hits in struct.iter_unpack("<HQH", block[16:]):
            if not last < local < 32768 or stamp > root["source_timestamp_ns"] or (stamp == 0) != (hits == 0):
                raise ValueError("精细地图块观察证据错误")
            last = local
            geometry.update(struct.pack("<H", local))
        if geometry.digest() != geometry_hash:
            raise ValueError("精细地图块几何摘要错误")
    if offset != len(payload):
        raise ValueError("精细地图资产存在尾随数据")
    return root
