"""完整扫描点云资产与分块传输；不执行空间抽样。"""
import json
import math
import struct
import threading

import numpy as np

from services.geometry.checkpoint_payload import FrozenPayload, payload_digest

MAX_DENSE_ARCHIVE_BYTES = 8 * 1024 * 1024 * 1024
DENSE_CHUNK_POINTS = 65_536


def read_dense_metadata(payload):
    if not isinstance(payload, (bytes, FrozenPayload)) or not 24 <= len(payload) <= MAX_DENSE_ARCHIVE_BYTES:
        raise ValueError("完整点云资产类型或大小非法")
    if payload[:8] != b"NUWAXDC1":
        raise ValueError("完整点云资产格式未知")
    length = struct.unpack("<I", payload[8:12])[0]
    if not 1 <= length <= 65_536 or length + 12 >= len(payload):
        raise ValueError("完整点云资产头长度非法")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("完整点云资产头字段重复")
            result[key] = value
        return result

    metadata = json.loads(payload[12:12 + length], object_pairs_hook=unique)
    fields = {"version", "session_id", "session_generation", "cloud_revision", "source_timestamp_ns",
              "frame_id", "point_count", "bounds_min", "bounds_max"}
    if not isinstance(metadata, dict) or set(metadata) != fields:
        raise ValueError("完整点云资产头字段非法")
    for key in ("version", "session_generation", "cloud_revision", "source_timestamp_ns", "point_count"):
        if type(metadata[key]) is not int or not 1 <= metadata[key] < 2 ** 64:
            raise ValueError("完整点云资产整数非法")
    if metadata["version"] != 1 or len(payload) != 12 + length + metadata["point_count"] * 12:
        raise ValueError("完整点云资产版本或点数不符")
    for key in ("session_id", "frame_id"):
        if not isinstance(metadata[key], str) or not metadata[key].strip() or len(metadata[key].encode("utf-8")) > 1024:
            raise ValueError("完整点云资产身份非法")
    for key in ("bounds_min", "bounds_max"):
        bounds = metadata[key]
        if not isinstance(bounds, list) or len(bounds) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds):
            raise ValueError("完整点云资产边界非法")
    if any(low > high for low, high in zip(metadata["bounds_min"], metadata["bounds_max"])):
        raise ValueError("完整点云资产边界顺序错误")
    return metadata, 12 + length


def validate_dense_archive(payload, *, session=None, generation=None, cloud_revision=None,
                           source_timestamp_ns=None, frame_id=None):
    metadata, offset = read_dense_metadata(payload)
    for key, expected in (("session_id", session), ("session_generation", generation),
                          ("cloud_revision", cloud_revision), ("source_timestamp_ns", source_timestamp_ns),
                          ("frame_id", frame_id)):
        if expected is not None and metadata[key] != expected:
            raise ValueError("完整点云资产与原地图提交不一致")
    lower, upper = np.full(3, np.inf), np.full(3, -np.inf)
    while offset < len(payload):
        data = payload[offset:offset + DENSE_CHUNK_POINTS * 12]
        points = np.frombuffer(data, dtype="<f4").reshape(-1, 3)
        if not np.isfinite(points).all():
            raise ValueError("完整点云资产包含无效坐标")
        lower = np.minimum(lower, points.min(axis=0))
        upper = np.maximum(upper, points.max(axis=0))
        offset += len(data)
    if not np.array_equal(lower, np.asarray(metadata["bounds_min"], dtype=np.float32)) or not np.array_equal(upper, np.asarray(metadata["bounds_max"], dtype=np.float32)):
        raise ValueError("完整点云资产边界与数据不符")
    return metadata


def dense_chunk_payload(response, *, expected_epoch="", offset_points=0):
    """检查远端有界分块，整数身份以字符串交给浏览器。"""
    data = bytes(response.xyz_f32_le)
    if not response.epoch or len(response.epoch) > 128 or (expected_epoch and response.epoch != expected_epoch):
        raise ValueError("完整点云分块来源不一致")
    if response.offset_points != offset_points or len(data) % 12 or len(data) > DENSE_CHUNK_POINTS * 12:
        raise ValueError("完整点云分块位置或长度不符")
    if response.total_points < offset_points + len(data) // 12 or (not data and offset_points != response.total_points):
        raise ValueError("完整点云分块点数不符")
    if not response.frame_id or not response.session_id or response.revision == 0 or response.session_generation == 0:
        raise ValueError("完整点云分块缺少地图身份")
    if len(response.bounds_min) != 3 or len(response.bounds_max) != 3 or any(
        not math.isfinite(low) or not math.isfinite(high) or low > high
        for low, high in zip(response.bounds_min, response.bounds_max)
    ) or not np.isfinite(np.frombuffer(data, dtype="<f4")).all():
        raise ValueError("完整点云分块坐标或边界非法")
    return {
        "epoch": response.epoch, "frame_id": response.frame_id, "session_id": response.session_id,
        "session_generation": str(response.session_generation), "revision": str(response.revision),
        "source_timestamp_ns": str(response.source_timestamp_ns), "total_points": str(response.total_points),
        "offset_points": str(response.offset_points), "point_count": len(data) // 12,
        "bounds_min": list(response.bounds_min), "bounds_max": list(response.bounds_max),
    }, data


class SavedDenseCloudCache:
    """仅保留当前已验证资产的私有只读载荷，分块读取不重复加载整图。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._key = None
        self._payload = None
        self._metadata = None
        self._offset = 0
        self._epoch = None
        self._closed = False

    def close(self):
        with self._lock:
            self._closed = True
            self._key = self._payload = self._metadata = self._epoch = None

    def read(self, source, *, expected_epoch="", offset_points=0):
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore

        key = (str(source["bundle_root"]), source["backend_kind"], source["manifest_sha256"], source["frame_id"])
        with self._lock:
            if self._closed:
                raise RuntimeError("完整点云读取已关闭")
            if key != self._key:
                self._key = self._payload = self._metadata = self._epoch = None
                stored = MappingBackendCheckpointStore(source["bundle_root"]).load(source["backend_kind"])
                if stored.manifest_sha256 != source["manifest_sha256"]:
                    raise ValueError("完整点云检查点与活动资产不一致")
                payload = stored.checkpoint.dense_archive
                if payload is None:
                    raise RuntimeError("当前地图没有完整扫描数据；旧体素地图无法还原全部扫描点")
                metadata, offset = read_dense_metadata(payload)
                if metadata["frame_id"] != source["frame_id"]:
                    raise ValueError("完整点云坐标系与活动资产不一致")
                self._key, self._payload, self._metadata, self._offset = key, payload, metadata, offset
                self._epoch = "saved:" + payload_digest(payload).hex()
            if expected_epoch and expected_epoch != self._epoch:
                raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
            metadata = self._metadata
            if type(offset_points) is not int or not 0 <= offset_points <= metadata["point_count"]:
                raise ValueError("完整点云分块位置非法")
            count = min(DENSE_CHUNK_POINTS, metadata["point_count"] - offset_points)
            start = self._offset + offset_points * 12
            return {
                "epoch": self._epoch, "frame_id": metadata["frame_id"], "session_id": metadata["session_id"],
                "session_generation": str(metadata["session_generation"]), "revision": str(metadata["cloud_revision"]),
                "source_timestamp_ns": str(metadata["source_timestamp_ns"]), "total_points": str(metadata["point_count"]),
                "offset_points": str(offset_points), "point_count": count,
                "bounds_min": metadata["bounds_min"], "bounds_max": metadata["bounds_max"],
            }, self._payload[start:start + count * 12]
