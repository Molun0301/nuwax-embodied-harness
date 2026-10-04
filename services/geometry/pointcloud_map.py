from __future__ import annotations

from abc import ABC, abstractmethod
from collections import OrderedDict
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import ctypes
from functools import lru_cache
import math
import mmap
import os
from pathlib import Path
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from contracts.geometry_runtime import DEFAULT_MAP_FRAME


_PCD_NUMPY_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
}


_LOD_LIMITS = {
    "preview": 60000,
    "medium": 180000,
    "full": None,
}


class PointCloudMapError(RuntimeError):
    """点云地图解析或缓存错误。"""


class UnsupportedPointCloudMapError(PointCloudMapError):
    """当前解析器不支持的点云地图格式。"""


class PointCloudMapSourceError(PointCloudMapError):
    """统一点云来源打开、读取或关闭错误。"""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        source: str,
        retryable: bool = False,
        terminal: bool = False,
    ) -> None:
        super().__init__(message)
        self.code = str(code)
        self.source = str(source)
        self.retryable = bool(retryable)
        self.terminal = bool(terminal)


@lru_cache(maxsize=1)
def _cloud_map_atomic_reader():
    """Resolve the platform atomic ABI once; never install software at runtime."""
    try:
        library = ctypes.PyDLL("libatomic.so.1")
        load = getattr(library, "__atomic_load_8")
        load.argtypes = (ctypes.c_void_p, ctypes.c_int)
        load.restype = ctypes.c_uint64
        fence = library.atomic_thread_fence
        fence.argtypes = (ctypes.c_int,)
        fence.restype = None
        lock_free = getattr(library, "__atomic_is_lock_free")
        lock_free.argtypes = (ctypes.c_size_t, ctypes.c_void_p)
        lock_free.restype = ctypes.c_bool
        if not lock_free(8, None):
            return None, "cloud-map v3 requires lock-free platform u64 atomics"
        return (library, load, fence, lock_free), None
    except (OSError, AttributeError) as exc:
        return None, f"cloud-map v3 requires platform libatomic.so.1: {exc}"


def _cloud_map_acquire_sequence(mm: mmap.mmap) -> int:
    functions, error = _cloud_map_atomic_reader()
    if functions is None:
        raise PointCloudMapSourceError("live_atomic_unavailable", error, source="live", terminal=True)
    # Only a temporary zero-copy view obtains the address of a read-only mmap.
    # It is released before returning: close/reopen never retains exported views.
    view = np.frombuffer(mm, dtype="<u8", count=1, offset=16)
    try:
        address = ctypes.c_void_p(view.ctypes.data)
        if address.value % 8:
            raise PointCloudMapSourceError("live_atomic_unsupported", "cloud-map v3 needs lock-free aligned u64 loads", source="live", terminal=True)
        return int(functions[1](address, 2))  # __ATOMIC_ACQUIRE
    finally:
        del view


def _cloud_map_read_fence() -> None:
    functions, error = _cloud_map_atomic_reader()
    if functions is None:
        raise PointCloudMapSourceError("live_atomic_unavailable", error, source="live", terminal=True)
    functions[2](5)  # __ATOMIC_SEQ_CST: finish payload loads before final sequence


@dataclass(frozen=True)
class PointCloudMapChunk:
    """Owned immutable XYZ for one producer-addressed page; unchanged pages are shared."""

    chunk_index: int
    revision_ordinal: int
    point_count: int
    xyz_bytes: bytes

    def __post_init__(self) -> None:
        if self.chunk_index < 0 or self.revision_ordinal <= 0 or self.point_count <= 0:
            raise ValueError("invalid pointcloud chunk identity")
        if len(self.xyz_bytes) != self.point_count * 12:
            raise ValueError("pointcloud chunk XYZ size mismatch")


@dataclass(frozen=True)
class PointCloudMapSnapshot:
    """Batch03 规范点云在只读观察边界上的统一快照。"""

    source: str
    source_epoch: str
    revision: str
    revision_ordinal: int
    frame_id: str
    source_timestamp_ns: int
    point_count: int
    format: str
    xyz_bytes: bytes
    stream_generation: int
    provenance: Dict[str, object]

    chunks: Tuple[PointCloudMapChunk, ...] = ()
    layout_generation: int = 0
    chunk_size_points: int = 0
    base_revision_ordinal: int = 0
    reset: bool = False

    @property
    def chunked(self) -> bool:
        return self.chunk_size_points > 0

    def __post_init__(self) -> None:
        if self.source not in {"live", "saved"}:
            raise ValueError("pointcloud source 必须是 live 或 saved。")
        if not self.source_epoch or not self.revision or not self.frame_id:
            raise ValueError("pointcloud snapshot 身份字段不能为空。")
        if self.revision_ordinal < 0 or self.source_timestamp_ns < 0 or self.stream_generation < 0:
            raise ValueError("pointcloud snapshot 序号与时间必须非负。")
        if self.point_count < 0:
            raise ValueError("pointcloud snapshot point_count must be nonnegative")
        if self.chunked:
            if self.xyz_bytes or self.layout_generation <= 0:
                raise ValueError("chunked pointcloud must carry pages and layout identity")
            expected = (self.point_count + self.chunk_size_points - 1) // self.chunk_size_points
            if len(self.chunks) != expected:
                raise ValueError("pointcloud snapshot chunk count mismatch")
            for index, chunk in enumerate(self.chunks):
                count = min(self.chunk_size_points, self.point_count - index * self.chunk_size_points)
                if chunk.chunk_index != index or chunk.point_count != count or chunk.revision_ordinal > self.revision_ordinal:
                    raise ValueError("pointcloud snapshot page metadata mismatch")
        elif len(self.xyz_bytes) != self.point_count * 12:
            raise ValueError("pointcloud snapshot 点数与 XYZ 字节长度不一致。")
        if self.format != "float32-xyz-le":
            raise ValueError("pointcloud snapshot 仅支持 float32-xyz-le。")


class PointCloudMapSource(ABC):
    """实时与保存点云来源共享的同步只读生命周期合同。"""

    source: str

    @abstractmethod
    def open(self) -> None:
        """幂等打开来源。"""

    @abstractmethod
    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        """读取当前稳定快照；没有可用快照时返回 None。"""

    @abstractmethod
    def cancel(self) -> None:
        """幂等取消当前生命周期。"""

    @abstractmethod
    def close(self) -> None:
        """幂等关闭；关闭后 read 必须保持静默。"""


@dataclass(frozen=True)
class CloudMapMmapSnapshot:
    """Rust cloud-map mmap v2 的稳定只读快照。"""

    source_epoch: str
    version: int
    active_buffer: int
    revision: int
    point_count: int
    timestamp_ns: int
    voxel_size: float
    xyz_bytes: bytes
    mapping_session_id: str = ""
    session_generation: int = 0
    occupancy_revision: int = 0
    chunks: Tuple[PointCloudMapChunk, ...] = ()
    layout_generation: int = 0
    chunk_size_points: int = 0
    base_revision_ordinal: int = 0
    reset: bool = False

    @property
    def chunked(self) -> bool:
        return self.chunk_size_points > 0



@dataclass(frozen=True)
class _FileIdentity:
    device: int
    inode: int
    ctime_ns: int
    size: int

    @property
    def epoch(self) -> str:
        return "dev:%d:ino:%d:ctime:%d" % (
            self.device,
            self.inode,
            self.ctime_ns,
        )


@dataclass(frozen=True)
class PcdHeader:
    """PCD 点云文件头。"""

    fields: Tuple[str, ...]
    size: Tuple[int, ...]
    type: Tuple[str, ...]
    count: Tuple[int, ...]
    width: int
    height: int
    points: int
    data: str
    header_bytes: int
    raw: Dict[str, str]


@dataclass(frozen=True)
class PointCloudMap:
    """可供 Web 调试面板消费的累计点云地图缓存对象。"""

    path: str
    revision: str
    frame_id: str
    source_kind: str
    point_count: int
    points_xyz: np.ndarray
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]
    header: PcdHeader
    loaded_at: datetime
    source_timestamp: datetime
    load_ms: float
    source_point_count: Optional[int] = None

    def lod_points(self, lod: str) -> np.ndarray:
        normalized = str(lod or "preview").strip().lower() or "preview"
        max_points = _LOD_LIMITS.get(normalized)
        if max_points is None or self.point_count <= max_points:
            return self.points_xyz
        stride = max(1, int(math.ceil(float(self.point_count) / float(max_points))))
        return self.points_xyz[::stride][:max_points]

    def lods(self) -> List[Dict[str, object]]:
        result: List[Dict[str, object]] = []
        for level in ("preview", "medium", "full"):
            points = self.lod_points(level)
            result.append(
                {
                    "level": level,
                    "point_count": int(points.shape[0]),
                    "stride": self._lod_stride(level),
                }
            )
        return result

    def binary_xyz(self, lod: str) -> bytes:
        points = self.lod_points(lod)
        little_endian = np.asarray(points, dtype="<f4", order="C")
        return little_endian.tobytes(order="C")

    def _lod_stride(self, lod: str) -> int:
        normalized = str(lod or "preview").strip().lower() or "preview"
        max_points = _LOD_LIMITS.get(normalized)
        if max_points is None or self.point_count <= max_points:
            return 1
        return max(1, int(math.ceil(float(self.point_count) / float(max_points))))


class PointCloudMapCache:
    """按点云资产路径、mtime 和大小缓存只读地图。"""

    def __init__(self, max_cache_bytes: int = 64 * 1024 * 1024) -> None:
        self._lock = threading.RLock()
        self._items = OrderedDict()
        self._max_cache_bytes = max(0, int(max_cache_bytes))
        self._cache_bytes = 0

    def _cached(self, key):
        with self._lock:
            cached = self._items.get(key)
            if cached is not None:
                self._items.move_to_end(key)
            return cached

    def _remember(self, key, loaded):
        with self._lock:
            # 同一路径的旧文件身份退出；不同观察预算可分别缓存。
            for old_key in list(self._items):
                if old_key[0] == key[0] and old_key[1] != key[1]:
                    self._cache_bytes -= self._items.pop(old_key).points_xyz.nbytes
            previous = self._items.pop(key, None)
            if previous is not None:
                self._cache_bytes -= previous.points_xyz.nbytes
            size = loaded.points_xyz.nbytes
            if size > self._max_cache_bytes:
                return
            while self._items and (self._cache_bytes + size > self._max_cache_bytes
                                   or len(self._items) >= 8):
                _, evicted = self._items.popitem(last=False)
                self._cache_bytes -= evicted.points_xyz.nbytes
            self._items[key] = loaded
            self._cache_bytes += size

    def load_preview(
        self, path: Path, *, frame_id: str = DEFAULT_MAP_FRAME,
        source_kind: str = "map_asset", max_points: Optional[int] = None,
        should_cancel=None,
    ) -> PointCloudMap:
        """分块读取资产；默认保留全部点，仅显式预览请求才抽样。"""
        from services.geometry.pointcloud_asset_stream import check_cancelled, read_preview_xyz

        resolved = path.expanduser().resolve(strict=False)
        check_cancelled(should_cancel)
        identity = _file_identity(resolved)
        if identity is None:
            raise PointCloudMapError("点云地图不存在：%s" % resolved)
        if max_points is not None:
            max_points = int(max_points)
            if max_points <= 0:
                raise ValueError("显式预览点数必须为正数。")
        key = (str(resolved), identity, frame_id, source_kind, max_points)
        cached = self._cached(key)
        if cached is not None:
            return cached
        started_at = time.perf_counter()
        stat_result = resolved.stat()
        header, points, total, lower, upper = read_preview_xyz(
            resolved, max_points, should_cancel=should_cancel,
        )
        revision = "sha256:%s" % _sha256_file(resolved, should_cancel=should_cancel)
        check_cancelled(should_cancel)
        if _file_identity(resolved) != identity:
            raise PointCloudMapError("点云资产在读取期间被替换或修改。")
        points.flags.writeable = False
        loaded = PointCloudMap(
            path=str(resolved), revision=revision, frame_id=frame_id, source_kind=source_kind,
            point_count=len(points), points_xyz=points, bounds_min=lower, bounds_max=upper,
            header=header, loaded_at=datetime.now(timezone.utc),
            source_timestamp=datetime.fromtimestamp(stat_result.st_mtime, tz=timezone.utc),
            load_ms=(time.perf_counter() - started_at) * 1000.0, source_point_count=total,
        )
        self._remember(key, loaded)
        return loaded

    def load(
        self,
        path: Path,
        *,
        frame_id: str = DEFAULT_MAP_FRAME,
        source_kind: str = "map_asset",
    ) -> PointCloudMap:
        suffix = path.suffix.strip().lower()
        if suffix == ".pcd":
            return self.load_pcd(path, frame_id=frame_id, source_kind=source_kind)
        if suffix == ".ply":
            return self.load_ply(path, frame_id=frame_id, source_kind=source_kind)
        raise UnsupportedPointCloudMapError("不支持的点云地图扩展名：%s" % (suffix or "<empty>"))

    def load_pcd(
        self,
        path: Path,
        *,
        frame_id: str = DEFAULT_MAP_FRAME,
        source_kind: str = "map_asset_pcd",
    ) -> PointCloudMap:
        return self._load_asset(
            path,
            frame_id=frame_id,
            source_kind=source_kind,
            expected_suffix=".pcd",
            reader=read_pcd_xyz,
        )

    def load_ply(
        self,
        path: Path,
        *,
        frame_id: str = DEFAULT_MAP_FRAME,
        source_kind: str = "map_asset_ply",
    ) -> PointCloudMap:
        return self._load_asset(
            path,
            frame_id=frame_id,
            source_kind=source_kind,
            expected_suffix=".ply",
            reader=read_ply_xyz,
        )

    def _load_asset(
        self,
        path: Path,
        *,
        frame_id: str,
        source_kind: str,
        expected_suffix: str,
        reader,
    ) -> PointCloudMap:
        from services.geometry.pointcloud_asset_stream import finite_xyz_mask, xyz_bounds

        resolved = path.expanduser().resolve(strict=False)
        if not resolved.exists() or not resolved.is_file():
            raise PointCloudMapError("点云地图不存在：%s" % resolved)
        if resolved.suffix.lower() != expected_suffix:
            raise UnsupportedPointCloudMapError(
                "点云地图扩展名不匹配：expected=%s actual=%s"
                % (expected_suffix, resolved.suffix.lower())
            )
        stat_result = resolved.stat()
        identity = _file_identity(resolved)
        key = (str(resolved), identity, frame_id, source_kind, None)
        cached = self._cached(key)
        if cached is not None:
            return cached
        started_at = time.perf_counter()
        header, points_xyz = reader(resolved)
        if points_xyz.size == 0:
            raise PointCloudMapError("点云地图没有可用 x/y/z 点。")
        finite_mask = finite_xyz_mask(points_xyz)
        if not bool(finite_mask.all()):
            points_xyz = points_xyz[finite_mask]
        if points_xyz.size == 0:
            raise PointCloudMapError("点云地图全部点均为非有限数。")
        points_xyz = np.asarray(points_xyz, dtype=np.float32, order="C")
        bounds_min_arr, bounds_max_arr = xyz_bounds(points_xyz)
        source_timestamp = datetime.fromtimestamp(stat_result.st_mtime, tz=timezone.utc)
        content_revision = "sha256:%s" % _sha256_file(resolved)
        loaded = PointCloudMap(
            path=str(resolved),
            revision=content_revision,
            frame_id=str(frame_id or DEFAULT_MAP_FRAME),
            source_kind=str(source_kind or "map_asset"),
            point_count=int(points_xyz.shape[0]),
            points_xyz=points_xyz,
            bounds_min=tuple(float(value) for value in bounds_min_arr),
            bounds_max=tuple(float(value) for value in bounds_max_arr),
            header=header,
            loaded_at=datetime.now(timezone.utc),
            source_timestamp=source_timestamp,
            load_ms=(time.perf_counter() - started_at) * 1000.0,
        )
        if _file_identity(resolved) != identity:
            raise PointCloudMapError("点云资产在读取期间被替换或修改。")
        points_xyz.flags.writeable = False
        self._remember(key, loaded)
        return loaded


def write_pcd_binary_xyz(path: Path, points_xyz: np.ndarray) -> None:
    """将 Nx3 float32 点云写入 binary PCD（仅 x/y/z 字段）。"""

    points = np.asarray(points_xyz, dtype=np.float32, order="C")
    if points.ndim != 2 or points.shape[1] != 3:
        raise PointCloudMapError("点云数组必须是 Nx3。")
    finite_mask = np.isfinite(points).all(axis=1)
    if not bool(finite_mask.all()):
        points = points[finite_mask]
    if points.size == 0:
        raise PointCloudMapError("点云数组没有可用点。")
    point_count = int(points.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n"
    ) % (point_count, point_count)
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    with resolved.open("wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(memoryview(points).cast("B"))


def read_ply_xyz(path: Path) -> Tuple[PcdHeader, np.ndarray]:
    """分块解析PLY（多边形文件格式）顶点，保留完整XYZ数组接口。"""
    from services.geometry.pointcloud_asset_stream import read_all_xyz

    return read_all_xyz(path.expanduser().resolve(), format_hint=".ply")


def read_pcd_point_count(path: Path) -> int:
    """仅解析 PCD 头中的 POINTS 数量，用于快速校验资产是否可用。"""

    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise PointCloudMapError("PCD 点云文件不存在：%s" % resolved)
    with resolved.open("rb") as stream:
        header = _read_pcd_header(stream)
    return int(header.points)


def read_pcd_xyz(path: Path) -> Tuple[PcdHeader, np.ndarray]:
    """分块读取PCD（点云数据文件），不保留整文件字节副本。"""
    from services.geometry.pointcloud_asset_stream import read_all_xyz

    return read_all_xyz(path, format_hint=".pcd")


def _read_pcd_header(stream) -> PcdHeader:
    raw: Dict[str, str] = {}
    header_bytes = 0
    while True:
        line_bytes = stream.readline(65537)
        if not line_bytes:
            raise PointCloudMapError("PCD 文件缺少 DATA 行。")
        header_bytes += len(line_bytes)
        if len(line_bytes) > 65536 or header_bytes > 4 * 1024 * 1024:
            raise PointCloudMapError("PCD 文件头过长。")
        line = line_bytes.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        key, value = parts[0], parts[1] if len(parts) == 2 else ""
        key_upper = key.upper()
        raw[key_upper] = value.strip()
        if key_upper == "DATA":
            break

    fields = tuple(_split_words(raw.get("FIELDS", "")))
    if not fields:
        raise PointCloudMapError("PCD 文件缺少 FIELDS。")
    size = tuple(_parse_ints(raw.get("SIZE", ""), default=4, expected=len(fields)))
    field_types = tuple(item.upper() for item in _split_words(raw.get("TYPE", "")))
    if not field_types:
        field_types = tuple("F" for _ in fields)
    field_types = tuple(_pad_sequence(field_types, len(fields), "F"))
    count = tuple(_parse_ints(raw.get("COUNT", ""), default=1, expected=len(fields)))
    width = _parse_positive_int(raw.get("WIDTH"), default=0)
    height = _parse_positive_int(raw.get("HEIGHT"), default=1)
    points = _parse_positive_int(raw.get("POINTS"), default=width * height)
    if points <= 0:
        points = width * height
    data = raw.get("DATA", "").strip().lower()
    return PcdHeader(
        fields=fields,
        size=size,
        type=field_types,
        count=count,
        width=width,
        height=height,
        points=points,
        data=data,
        header_bytes=header_bytes,
        raw=raw,
    )


def _pcd_binary_dtype(header: PcdHeader) -> np.dtype:
    dtype_fields = []
    for name, size, field_type, count in zip(header.fields, header.size, header.type, header.count):
        numpy_type = _PCD_NUMPY_TYPES.get((field_type.upper(), int(size)))
        if numpy_type is None:
            raise UnsupportedPointCloudMapError("不支持的 PCD 字段类型：%s size=%s" % (field_type, size))
        if count == 1:
            dtype_fields.append((name, numpy_type))
        else:
            dtype_fields.append((name, numpy_type, (int(count),)))
    return np.dtype(dtype_fields)


def _field_values(records: np.ndarray, name: str) -> np.ndarray:
    if name not in records.dtype.names:
        raise ValueError("PCD 缺少 %s 字段。" % name)
    values = records[name]
    if values.ndim > 1:
        values = values[:, 0]
    return values.astype(np.float32, copy=False)


def _xyz_indexes(columns: Sequence[str]) -> List[int]:
    indexes: List[int] = []
    for name in ("x", "y", "z"):
        try:
            indexes.append(columns.index(name))
        except ValueError as exc:
            raise PointCloudMapError("PCD 缺少 %s 字段。" % name) from exc
    return indexes


def _expanded_columns(fields: Sequence[str], counts: Sequence[int]) -> List[str]:
    columns: List[str] = []
    for field, count in zip(fields, counts):
        if int(count) == 1:
            columns.append(field)
        else:
            for index in range(int(count)):
                columns.append("%s_%d" % (field, index))
    return columns


def _split_words(value: str) -> List[str]:
    return [item for item in str(value or "").strip().split() if item]


def _parse_ints(value: str, *, default: int, expected: int) -> List[int]:
    values = []
    for item in _split_words(value):
        try:
            values.append(int(item))
        except ValueError as exc:
            raise PointCloudMapError("PCD header 整数字段非法：%s" % item) from exc
    return _pad_sequence(values, expected, default)


def _pad_sequence(values: Sequence, expected: int, default):
    result = list(values)
    if len(result) > expected:
        return result[:expected]
    while len(result) < expected:
        result.append(default)
    return result


def _parse_positive_int(value: Optional[str], *, default: int) -> int:
    try:
        parsed = int(str(value).strip()) if value is not None else default
    except ValueError:
        parsed = default
    return max(0, parsed)


def _sha256_file(path: Path, *, should_cancel=None) -> str:
    from services.geometry.pointcloud_asset_stream import check_cancelled

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            check_cancelled(should_cancel)
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _file_identity(path: Path) -> Optional[_FileIdentity]:
    try:
        stat_result = path.stat()
    except (FileNotFoundError, OSError):
        return None
    if not path.is_file():
        return None
    return _FileIdentity(
        device=int(stat_result.st_dev),
        inode=int(stat_result.st_ino),
        ctime_ns=int(stat_result.st_ctime_ns),
        size=int(stat_result.st_size),
    )


class CloudMapMmapReader:
    """读取 Rust bridge 写入的累积地图双缓冲 mmap。"""

    MAGIC = 0x4E55_5741_584D_4150  # "NUWAXMAP"
    HEADER_SIZE = 64
    SINGLE_BUFFER_SIZE = 16 * 1024 * 1024  # 16MB
    HEADER_FMT = "<QIIIQQf"
    HEADER_STRUCT_SIZE = struct.calcsize(HEADER_FMT)  # 40 bytes
    SLOT_METADATA_FMT = "<QI"
    SLOT_METADATA_SIZE = struct.calcsize(SLOT_METADATA_FMT)  # 12 bytes
    SLOT_METADATA_OFFSET = HEADER_STRUCT_SIZE
    SLOT_METADATA_VERSION = 2
    PAGED_VERSION = 3
    PAGED_HEADER_FMT = "<QIIQQQQ16sIIIIfIQQQQQ"
    PAGED_CORE_HEADER_SIZE = struct.calcsize(PAGED_HEADER_FMT)
    PAGED_HEADER_SIZE = 512
    CHUNK_SIZE_POINTS = 1024

    def __init__(self, shm_path: str = "/tmp/nuwax_cloudmap.mmap"):
        self._path = Path(shm_path)
        self._mm: Optional[mmap.mmap] = None
        self._fd = None
        self._opened_identity: Optional[_FileIdentity] = None
        self._observed_file_identity: Optional[_FileIdentity] = None
        self._open_generation = 0
        self._source_epoch: Optional[str] = None
        self._snapshot_cache: Optional[Tuple[Tuple[object, ...], CloudMapMmapSnapshot]] = None
        self._paged_snapshot: Optional[CloudMapMmapSnapshot] = None
        self._last_read_busy = False
        self._sampled_snapshot_cache = None

    @property
    def source_epoch(self) -> Optional[str]:
        if self._paged_snapshot is not None:
            return self._paged_snapshot.source_epoch
        return self._source_epoch

    def is_available(self) -> bool:
        current_identity = _file_identity(self._path)
        if current_identity is None:
            self.close()
            return False
        if self._mm is not None:
            if self._fd is None:
                self.close()
            else:
                try:
                    descriptor_stat = os.fstat(self._fd.fileno())
                except (OSError, ValueError):
                    self.close()
                else:
                    descriptor_identity = _FileIdentity(
                        device=int(descriptor_stat.st_dev),
                        inode=int(descriptor_stat.st_ino),
                        ctime_ns=int(descriptor_stat.st_ctime_ns),
                        size=int(descriptor_stat.st_size),
                    )
                    if (
                        descriptor_identity.device != current_identity.device
                        or descriptor_identity.inode != current_identity.inode
                        or descriptor_identity.size != current_identity.size
                        or self._opened_identity is None
                        or self._opened_identity.size != current_identity.size
                    ):
                        self.close()
                    else:
                        # 正常写入新页也可能改变 ctime；只撤销字节复用，不改来源 epoch。
                        if self._observed_file_identity != descriptor_identity:
                            self._snapshot_cache = None
                        self._observed_file_identity = descriptor_identity
        if self._mm is None:
            try:
                self._open()
            except Exception:
                self.close()
                return False
        return self._mm is not None

    def _open(self) -> None:
        self.close()
        fd = open(self._path, "rb")
        try:
            descriptor_stat = os.fstat(fd.fileno())
            descriptor_identity = _FileIdentity(
                device=int(descriptor_stat.st_dev),
                inode=int(descriptor_stat.st_ino),
                ctime_ns=int(descriptor_stat.st_ctime_ns),
                size=int(descriptor_stat.st_size),
            )
            named_identity = _file_identity(self._path)
            if (
                named_identity is None
                or named_identity.device != descriptor_identity.device
                or named_identity.inode != descriptor_identity.inode
                or named_identity.size != descriptor_identity.size
            ):
                raise PointCloudMapError("cloud-map mmap 在打开期间被替换。")
            mm = mmap.mmap(fd.fileno(), 0, access=mmap.ACCESS_READ)
        except Exception:
            fd.close()
            raise
        self._open_generation += 1
        self._fd = fd
        self._mm = mm
        self._opened_identity = descriptor_identity
        self._observed_file_identity = descriptor_identity
        self._source_epoch = "dev:%d:ino:%d:opened:%d:generation:%d" % (
            descriptor_identity.device,
            descriptor_identity.inode,
            descriptor_identity.ctime_ns,
            self._open_generation,
        )

    def close(self) -> None:
        self._snapshot_cache = None
        self._paged_snapshot = None
        self._sampled_snapshot_cache = None
        self._last_read_busy = False
        if self._mm is not None:
            self._mm.close()
            self._mm = None
        if self._fd is not None:
            self._fd.close()
            self._fd = None
        self._opened_identity = None
        self._observed_file_identity = None
        self._source_epoch = None

    def read_header(self) -> Optional[Tuple]:
        if not self.is_available():
            return None
        assert self._mm is not None
        if len(self._mm) < self.HEADER_STRUCT_SIZE:
            return None
        if struct.unpack_from("<I", self._mm, 8)[0] == self.PAGED_VERSION:
            raw = self._mm[:self.PAGED_HEADER_SIZE]
            if len(raw) != self.PAGED_HEADER_SIZE:
                return None
            vals = struct.unpack(self.PAGED_HEADER_FMT, raw[:self.PAGED_CORE_HEADER_SIZE])
            if vals[0] != self.MAGIC or vals[3] & 1:
                return None
            return (vals[0], vals[1], 0, vals[8], vals[5], vals[4], vals[12])
        raw = self._mm[0:self.HEADER_STRUCT_SIZE]
        vals = struct.unpack(self.HEADER_FMT, raw)
        if vals[0] != self.MAGIC:
            return None
        return vals

    def _read_slot_metadata(self, slot: int) -> Optional[Tuple[int, int]]:
        if slot not in (0, 1) or self._mm is None:
            return None
        offset = self.SLOT_METADATA_OFFSET + slot * self.SLOT_METADATA_SIZE
        raw = self._mm[offset : offset + self.SLOT_METADATA_SIZE]
        if len(raw) != self.SLOT_METADATA_SIZE:
            return None
        revision, point_count = struct.unpack(self.SLOT_METADATA_FMT, raw)
        return int(revision), int(point_count)

    def read_stable_snapshot(
        self,
        max_points: Optional[int] = None,
    ) -> Optional[CloudMapMmapSnapshot]:
        """读取包含完整 v2 身份与时间字段的稳定点云快照。"""

        self._last_read_busy = False
        if (self.is_available() and self._mm is not None and len(self._mm) >= 12
                and struct.unpack_from("<I", self._mm, 8)[0] == self.PAGED_VERSION):
            snapshot = self._read_paged_snapshot()
            return self._limit_paged_snapshot(snapshot, max_points)
        cached = self._snapshot_cache
        self._snapshot_cache = None
        for _ in range(3):
            # 不稳定、不可用或异常读取撤销复用；重试重新取得自有字节。
            previous_cache, cached = cached, None
            if not self.is_available():
                return None
            source_epoch = self.source_epoch
            file_identity = self._observed_file_identity
            mm = self._mm
            if source_epoch is None or file_identity is None or mm is None:
                return None
            header_raw = bytes(mm[0:self.HEADER_STRUCT_SIZE])
            if len(header_raw) != self.HEADER_STRUCT_SIZE:
                return None
            header = struct.unpack(self.HEADER_FMT, header_raw)
            magic, version, active_buffer, legacy_point_count, timestamp_ns, revision, voxel_size = header
            if magic != self.MAGIC:
                return None
            revision = int(revision)
            active_buffer = int(active_buffer)
            if revision == 0 or active_buffer not in (0, 1):
                return None
            read_buf = active_buffer
            slot_metadata = None
            if version >= self.SLOT_METADATA_VERSION:
                if read_buf != (revision - 1) % 2:
                    continue
                slot_metadata = self._read_slot_metadata(read_buf)
                if slot_metadata is None or slot_metadata[0] != revision:
                    continue
                point_count = slot_metadata[1]
                if int(legacy_point_count) != point_count:
                    continue
            else:
                point_count = int(legacy_point_count)
                if point_count == 0:
                    return None

            max_buffer_points = self.SINGLE_BUFFER_SIZE // (3 * 4)
            if point_count > max_buffer_points:
                return None
            selected_max_points = max_buffer_points if max_points is None else min(max(1, int(max_points)), max_buffer_points)
            identity = (source_epoch, file_identity, header_raw, slot_metadata, selected_max_points)
            cached_snapshot = (
                previous_cache[1]
                if version >= self.SLOT_METADATA_VERSION
                and previous_cache is not None
                and previous_cache[0] == identity
                else None
            )
            buf_offset = self.HEADER_SIZE + read_buf * self.SINGLE_BUFFER_SIZE
            if cached_snapshot is not None:
                actual_count = cached_snapshot.point_count
                xyz_bytes = cached_snapshot.xyz_bytes
            elif point_count == 0:
                actual_count = 0
                xyz_bytes = b""
            elif point_count > selected_max_points:
                points = np.frombuffer(
                    mm,
                    dtype="<f4",
                    count=point_count * 3,
                    offset=buf_offset,
                ).reshape(point_count, 3)
                sample_indices = np.linspace(
                    0,
                    point_count - 1,
                    num=selected_max_points,
                    dtype=np.intp,
                )
                sampled = np.ascontiguousarray(points[sample_indices], dtype="<f4")
                del points
                actual_count = selected_max_points
                xyz_bytes = sampled.tobytes(order="C")
            else:
                data_size = point_count * 12
                xyz_bytes = bytes(mm[buf_offset:buf_offset + data_size])
                if len(xyz_bytes) != data_size:
                    return None
                actual_count = point_count

            if not self.is_available():
                return None
            if self.source_epoch != source_epoch or self._mm is not mm:
                continue
            header_raw_after = bytes(mm[0:self.HEADER_STRUCT_SIZE])
            if header_raw_after != header_raw:
                continue
            if version >= self.SLOT_METADATA_VERSION:
                slot_metadata_after = self._read_slot_metadata(read_buf)
                if slot_metadata_after != slot_metadata:
                    continue
            file_identity_stable = self._observed_file_identity == file_identity
            if cached_snapshot is not None and not file_identity_stable:
                # 同 inode 原地重建不能借用旧字节；下一轮按原有槽校验重新复制。
                continue
            snapshot = cached_snapshot or CloudMapMmapSnapshot(
                source_epoch=source_epoch,
                version=int(version),
                active_buffer=active_buffer,
                revision=revision,
                point_count=actual_count,
                timestamp_ns=int(timestamp_ns),
                voxel_size=float(voxel_size),
                xyz_bytes=xyz_bytes,
            )
            if version >= self.SLOT_METADATA_VERSION and file_identity_stable:
                # 仅保留一次已验证的 owned bytes；不保留 mmap 视图或跨读者缓存。
                self._snapshot_cache = (identity, snapshot)
            return snapshot
        return None

    def _limit_paged_snapshot(self, snapshot, max_points):
        # Explicit legacy sampling request. The normal live path has no limit
        # and never materializes contiguous XYZ. Gather only selected points.
        if snapshot is None or max_points is None or snapshot.point_count <= max(1, int(max_points)):
            return snapshot
        limit = max(1, int(max_points))
        key = (snapshot.source_epoch, snapshot.layout_generation, snapshot.revision, limit)
        if self._sampled_snapshot_cache is not None and self._sampled_snapshot_cache[0] == key:
            return self._sampled_snapshot_cache[1]
        indices = np.linspace(0, snapshot.point_count - 1, num=limit, dtype=np.intp)
        selected = np.empty((limit, 3), dtype="<f4")
        for chunk in snapshot.chunks:
            offset = chunk.chunk_index * snapshot.chunk_size_points
            begin, end = np.searchsorted(indices, [offset, offset + chunk.point_count])
            if begin != end:
                points = np.frombuffer(chunk.xyz_bytes, dtype="<f4").reshape(-1, 3)
                selected[begin:end] = points[indices[begin:end] - offset]
        sampled = replace(snapshot, point_count=limit, xyz_bytes=selected.tobytes(),
                          chunks=(), chunk_size_points=0, base_revision_ordinal=0, reset=True)
        self._sampled_snapshot_cache = (key, sampled)
        return sampled

    def _read_paged_snapshot(self) -> Optional[CloudMapMmapSnapshot]:
        """Read only producer-dirty page bytes, retaining immutable unchanged pages.

        The bounded version table is metadata, never a coordinate comparison. A
        complete bootstrap is needed only on first observation or layout reset.
        """
        self._last_read_busy = False
        for _ in range(3):
            if not self.is_available() or self._mm is None:
                return None
            mm = self._mm
            if len(mm) < self.PAGED_HEADER_SIZE:
                return None
            sequence_before = _cloud_map_acquire_sequence(mm)
            if sequence_before & 1:
                self._last_read_busy = True
                continue
            raw = mm[:self.PAGED_HEADER_SIZE]
            values = struct.unpack(self.PAGED_HEADER_FMT, raw[:self.PAGED_CORE_HEADER_SIZE])
            (magic, version, header_size, sequence, revision, timestamp_ns,
             layout_generation, producer_epoch, point_count, capacity, chunk_size,
             capacity_chunks, voxel_size, _reserved, _session, table_offset,
             payload_offset, total_size, _reserved2) = values
            if magic != self.MAGIC or version != self.PAGED_VERSION:
                return None
            if sequence != sequence_before:
                self._last_read_busy = True
                continue
            if revision == 0 or layout_generation == 0 or not any(producer_epoch):
                return None
            if (header_size != self.PAGED_HEADER_SIZE or chunk_size != self.CHUNK_SIZE_POINTS
                    or capacity <= 0 or point_count > capacity
                    or capacity_chunks != (capacity + chunk_size - 1) // chunk_size
                    or table_offset < header_size
                    or payload_offset < table_offset + capacity_chunks * 8
                    or total_size != payload_offset + capacity * 12 or total_size > len(mm)):
                return None
            source_epoch = "producer:" + producer_epoch.hex()
            session_id_length = struct.unpack_from("<I", raw, 160)[0]
            if session_id_length > 256:
                return None
            try:
                mapping_session_id = raw[164:164 + session_id_length].decode("utf-8")
            except UnicodeDecodeError:
                return None
            occupancy_revision = struct.unpack_from("<Q", raw, 152)[0]
            previous = self._paged_snapshot
            reset = (previous is None or previous.source_epoch != source_epoch
                     or previous.layout_generation != layout_generation
                     or revision < previous.revision)
            if previous is not None and not reset and previous.revision == revision:
                _cloud_map_read_fence()
                if sequence_before == _cloud_map_acquire_sequence(mm) and self.is_available() and self._mm is mm:
                    return previous
                self._last_read_busy = True
                continue
            count = (point_count + chunk_size - 1) // chunk_size
            page_raw = mm[table_offset:table_offset + count * 8]
            versions = struct.unpack(f"<{count}Q", page_raw) if count else ()
            chunks = []
            valid = True
            for index, page_revision in enumerate(versions):
                if page_revision == 0 or page_revision > revision:
                    valid = False
                    break
                count_in_page = min(chunk_size, point_count - index * chunk_size)
                old = (previous.chunks[index] if not reset and previous is not None
                       and index < len(previous.chunks) else None)
                if old is not None and old.revision_ordinal == page_revision and old.point_count == count_in_page:
                    chunks.append(old)
                else:
                    start = payload_offset + index * chunk_size * 12
                    payload = mm[start:start + count_in_page * 12]
                    if len(payload) != count_in_page * 12:
                        valid = False
                        break
                    chunks.append(PointCloudMapChunk(index, int(page_revision), count_in_page, payload))
            _cloud_map_read_fence()
            if sequence_before != _cloud_map_acquire_sequence(mm):
                self._last_read_busy = True
                continue
            if not valid:
                return None
            if not self.is_available() or self._mm is not mm:
                continue
            # File metadata is not a producer epoch; the UUID is independent of
            # inode, timestamp, observer reopening and map revision.
            snapshot = CloudMapMmapSnapshot(
                source_epoch=source_epoch, version=version, active_buffer=0,
                revision=int(revision), point_count=int(point_count),
                timestamp_ns=int(timestamp_ns), voxel_size=float(voxel_size),
                xyz_bytes=b"", chunks=tuple(chunks),
                mapping_session_id=mapping_session_id, session_generation=int(_session),
                occupancy_revision=int(occupancy_revision),
                layout_generation=int(layout_generation), chunk_size_points=chunk_size,
                base_revision_ordinal=0 if reset else previous.revision, reset=reset,
            )
            self._paged_snapshot = snapshot
            return snapshot
        return None

    def read_stable(
        self,
        max_points: Optional[int] = None,
    ) -> Optional[Tuple[int, int, bytes]]:
        """兼容旧调用方，返回完整 u64 revision、点数与 XYZ 字节。"""

        snapshot = self.read_stable_snapshot(max_points=max_points)
        if snapshot is None:
            return None
        if snapshot.chunked:
            # Explicit compatibility/export API. Live streaming never calls it.
            if max_points is not None and snapshot.point_count > max(1, int(max_points)):
                limit = max(1, int(max_points))
                indices = np.linspace(0, snapshot.point_count - 1, num=limit, dtype=np.intp)
                points = np.empty((limit, 3), dtype="<f4")
                for index, slot in enumerate(indices):
                    page = snapshot.chunks[int(slot) // snapshot.chunk_size_points]
                    offset = (int(slot) % snapshot.chunk_size_points) * 12
                    points[index] = np.frombuffer(page.xyz_bytes, dtype="<f4", count=3, offset=offset)
                return snapshot.revision, limit, points.tobytes()
            return snapshot.revision, snapshot.point_count, b"".join(chunk.xyz_bytes for chunk in snapshot.chunks)
        return snapshot.revision, snapshot.point_count, snapshot.xyz_bytes

    def build_binary_frame(
        self,
        max_points: Optional[int] = None,
    ) -> Optional[bytes]:
        """构建 Wire Format binary 帧: revision(u32) + point_count(u32) + XYZ data。"""
        result = self.read_stable(max_points)
        if result is None:
            return None
        revision, point_count, xyz_bytes = result
        import struct as _struct
        header_bytes = _struct.pack("<II", revision & 0xFFFFFFFF, point_count)
        return header_bytes + xyz_bytes


class LivePointCloudMapSource(PointCloudMapSource):
    """把 Batch03 cloud-map mmap 适配为统一只读来源。"""

    source = "live"

    def __init__(
        self,
        reader: CloudMapMmapReader,
        *,
        frame_id: str = DEFAULT_MAP_FRAME,
        max_points: Optional[int] = None,
    ) -> None:
        self._reader = reader
        self._frame_id = str(frame_id or DEFAULT_MAP_FRAME)
        self._max_points = None if max_points is None else max(1, int(max_points))
        self._opened = False
        self._cancelled = False
        self._closed = False
        self._observed_open_epoch: Optional[str] = None
        self._last_revision: Optional[int] = None
        self._restart_generation = 0

    def open(self) -> None:
        if self._closed or self._opened:
            return
        self._cancelled = False
        self._opened = True

    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        if self._closed or self._cancelled or (cancel_event is not None and cancel_event.is_set()):
            return None
        self.open()
        if not self._reader.is_available():
            self._last_revision = None
            raise PointCloudMapSourceError(
                "live_unavailable",
                "实时 cloud-map mmap 不可用。",
                source=self.source,
                retryable=True,
            )
        try:
            snapshot = self._reader.read_stable_snapshot(max_points=self._max_points)
        except PointCloudMapSourceError as exc:
            if exc.terminal:
                # A missing native atomic ABI is a lifecycle capability error,
                # not a per-frame retry loop. Recreating the source retries setup.
                self._cancelled = True
            raise
        except Exception as exc:
            self._last_revision = None
            raise PointCloudMapSourceError(
                "live_read_failed",
                str(exc),
                source=self.source,
                retryable=True,
            ) from exc
        if snapshot is None:
            if getattr(self._reader, "_last_read_busy", False):
                return None
            try:
                header = self._reader.read_header()
            except Exception as exc:
                self._last_revision = None
                raise PointCloudMapSourceError(
                    "live_header_failed",
                    str(exc),
                    source=self.source,
                    retryable=True,
                ) from exc
            if header is None:
                self._last_revision = None
                raise PointCloudMapSourceError(
                    "live_invalid_header",
                    "实时 cloud-map mmap 头无效。",
                    source=self.source,
                    retryable=True,
                )
            return None
        reader_epoch = snapshot.source_epoch
        if self._observed_open_epoch != reader_epoch:
            self._observed_open_epoch = reader_epoch
            self._last_revision = None
            self._restart_generation = 0
        elif self._last_revision is not None:
            if snapshot.revision == self._last_revision:
                return None
            if snapshot.revision < self._last_revision:
                self._restart_generation += 1
                self._last_revision = None
        source_epoch = reader_epoch
        if self._restart_generation > 0:
            source_epoch = "%s:observed-restart:%d" % (
                reader_epoch,
                self._restart_generation,
            )
        result = PointCloudMapSnapshot(
            source=self.source,
            source_epoch=source_epoch,
            revision="cloud:%d" % snapshot.revision,
            revision_ordinal=snapshot.revision,
            frame_id=self._frame_id,
            source_timestamp_ns=snapshot.timestamp_ns,
            point_count=snapshot.point_count,
            format="float32-xyz-le",
            xyz_bytes=snapshot.xyz_bytes,
            chunks=snapshot.chunks,
            layout_generation=snapshot.layout_generation,
            chunk_size_points=snapshot.chunk_size_points,
            base_revision_ordinal=snapshot.base_revision_ordinal,
            reset=snapshot.reset,
            stream_generation=0,
            provenance={
                "canonical_owner": "nuwax_geometry_runtime",
                "canonical_revision": True,
                "mmap_version": snapshot.version,
                "active_buffer": snapshot.active_buffer,
                "voxel_size": snapshot.voxel_size,
                "shm_path": str(self._reader._path),
                "observer_restart_generation": self._restart_generation,
                "writer_session_id_available": bool(snapshot.mapping_session_id),
                "mapping_session_id": snapshot.mapping_session_id,
                "session_generation": snapshot.session_generation,
                "occupancy_revision": snapshot.occupancy_revision,
                "mapping_source_epoch": (f"{snapshot.mapping_session_id}:{snapshot.session_generation}"
                                         if snapshot.mapping_session_id else None),
            },
        )
        self._last_revision = snapshot.revision
        return result

    def cancel(self) -> None:
        if self._closed:
            return
        self._cancelled = True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._cancelled = True
        self._opened = False
        self._reader.close()


class SavedPointCloudMapSource(PointCloudMapSource):
    """把 Geometry 保存的 PCD/PLY 资产适配为统一只读来源。"""

    source = "saved"

    def __init__(
        self,
        path: Path,
        *,
        cache: Optional[PointCloudMapCache] = None,
        frame_id: str = DEFAULT_MAP_FRAME,
        source_kind: str = "map_asset",
        map_name: Optional[str] = None,
        asset_manifest_sha256: Optional[str] = None,
        asset_sha256: Optional[str] = None,
        provenance: Optional[Dict[str, object]] = None,
    ) -> None:
        self._path = path.expanduser().resolve(strict=False)
        self._cache = cache or PointCloudMapCache()
        self._frame_id = str(frame_id or DEFAULT_MAP_FRAME)
        self._source_kind = str(source_kind or "map_asset")
        self._map_name = str(map_name) if map_name is not None else None
        self._asset_manifest_sha256 = str(asset_manifest_sha256 or "").strip().lower() or None
        self._asset_sha256 = str(asset_sha256 or "").strip().lower() or None
        self._provenance = dict(provenance or {})
        self._opened = False
        self._cancelled = False
        self._closed = False
        self._last_identity = None
        self._last_snapshot = None

    def open(self) -> None:
        if self._closed or self._opened:
            return
        self._cancelled = False
        self._opened = True

    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        if self._closed or self._cancelled or (cancel_event is not None and cancel_event.is_set()):
            return None
        self.open()
        identity = _file_identity(self._path)
        if identity is None:
            raise PointCloudMapSourceError(
                "saved_unavailable",
                "保存点云资产不可用：%s" % self._path,
                source=self.source,
                retryable=True,
            )
        if identity == self._last_identity and self._last_snapshot is not None:
            return self._last_snapshot
        from services.geometry.pointcloud_asset_stream import PointCloudReadCancelled

        def should_cancel():
            return self._closed or self._cancelled or (
                cancel_event is not None and cancel_event.is_set()
            )

        try:
            pointcloud_map = self._cache.load_preview(
                self._path,
                frame_id=self._frame_id,
                source_kind=self._source_kind,
                should_cancel=should_cancel,
            )
        except PointCloudReadCancelled:
            return None
        except UnsupportedPointCloudMapError as exc:
            raise PointCloudMapSourceError(
                "saved_format_unsupported",
                str(exc),
                source=self.source,
                terminal=True,
            ) from exc
        except (PointCloudMapError, OSError) as exc:
            raise PointCloudMapSourceError(
                "saved_read_failed",
                str(exc),
                source=self.source,
                retryable=True,
            ) from exc
        computed_sha256 = pointcloud_map.revision.partition(":")[2]
        identity_after = _file_identity(self._path)
        if identity_after is None or identity_after != identity:
            raise PointCloudMapSourceError(
                "saved_replaced_during_read",
                "保存点云资产在读取期间被替换。",
                source=self.source,
                retryable=True,
            )
        if self._asset_sha256 and self._asset_sha256 != computed_sha256:
            raise PointCloudMapSourceError(
                "saved_asset_hash_mismatch",
                "保存点云资产摘要与 Geometry 元数据不一致。",
                source=self.source,
                terminal=True,
            )
        if self._asset_manifest_sha256:
            revision = "manifest:%s" % self._asset_manifest_sha256
            revision_kind = "asset_manifest_sha256"
            canonical_identity = True
        elif self._asset_sha256:
            revision = "asset:%s" % self._asset_sha256
            revision_kind = "asset_sha256"
            canonical_identity = True
        else:
            revision = "asset:%s" % computed_sha256
            revision_kind = "computed_asset_sha256"
            canonical_identity = False
        if should_cancel():
            return None
        snapshot = PointCloudMapSnapshot(
            source=self.source,
            source_epoch="path:%s:%s" % (str(self._path), identity.epoch),
            revision=revision,
            revision_ordinal=max(0, int(identity.ctime_ns)),
            frame_id=pointcloud_map.frame_id,
            source_timestamp_ns=int(pointcloud_map.source_timestamp.timestamp() * 1_000_000_000),
            point_count=pointcloud_map.point_count,
            format="float32-xyz-le",
            xyz_bytes=pointcloud_map.binary_xyz("full"),
            stream_generation=0,
            provenance={
                **self._provenance,
                "canonical_owner": "nuwax_geometry_runtime",
                "canonical_revision": canonical_identity,
                "revision_kind": revision_kind,
                "map_name": self._map_name,
                "source_kind": pointcloud_map.source_kind,
                "asset_path": pointcloud_map.path,
                "asset_format": self._path.suffix.lower().lstrip("."),
                "asset_sha256": computed_sha256,
                "cache_key_mtime_ns": int(pointcloud_map.source_timestamp.timestamp() * 1_000_000_000),
                "cache_key_size": identity_after.size,
                "load_ms": pointcloud_map.load_ms,
                "asset_point_count": pointcloud_map.header.points,
                "finite_point_count": pointcloud_map.source_point_count,
                "display_point_count": pointcloud_map.point_count,
                "display_sampled": pointcloud_map.point_count < pointcloud_map.header.points,
                "display_sampling": "none",
                "bounds_min": pointcloud_map.bounds_min,
                "bounds_max": pointcloud_map.bounds_max,
            },
        )

        self._last_identity = identity
        self._last_snapshot = snapshot
        return snapshot

    def cancel(self) -> None:
        if self._closed:
            return
        self._cancelled = True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._last_snapshot = None
        self._last_identity = None
        self._cancelled = True
        self._opened = False


class PointCloudMapSourceSwitch:
    """串行选择 live/saved，并独占观察侧来源与修订水位。"""

    def __init__(
        self,
        live_source: PointCloudMapSource,
        saved_source: PointCloudMapSource,
    ) -> None:
        self._live_source = live_source
        self._saved_source = saved_source
        self._active_source: Optional[str] = None
        self._active_epoch: Optional[str] = None
        self._stream_generation = 0
        self._watermarks: Dict[Tuple[str, str], int] = {}
        self._opened = False
        self._cancelled = False
        self._closed = False
        self._lock = threading.RLock()

    @property
    def active_source(self) -> Optional[str]:
        with self._lock:
            return self._active_source

    @property
    def stream_generation(self) -> int:
        with self._lock:
            return self._stream_generation

    def open(self) -> None:
        with self._lock:
            if self._closed or self._opened:
                return
            self._cancelled = False
            self._live_source.open()
            self._saved_source.open()
            self._opened = True

    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        with self._lock:
            if self._closed or self._cancelled or (cancel_event is not None and cancel_event.is_set()):
                return None
            self.open()
            live_snapshot, live_usable = self._try_read(self._live_source, cancel_event)
            if live_snapshot is not None:
                return self._accept(live_snapshot)
            if live_usable and self._active_source == self._live_source.source:
                return None
            saved_snapshot, _ = self._try_read(self._saved_source, cancel_event)
            if saved_snapshot is None:
                return None
            return self._accept(saved_snapshot)

    def _try_read(
        self,
        source: PointCloudMapSource,
        cancel_event: Optional[threading.Event],
    ) -> Tuple[Optional[PointCloudMapSnapshot], bool]:
        try:
            return source.read(cancel_event), True
        except PointCloudMapSourceError as exc:
            # 读取失败与“没有新修订”不同；恢复后即使修订相同也必须重新交付。
            if self._active_source == source.source:
                self._active_source = None
                self._active_epoch = None
            if not exc.retryable:
                raise
            return None, False

    def _accept(self, snapshot: PointCloudMapSnapshot) -> Optional[PointCloudMapSnapshot]:
        key = (snapshot.source, snapshot.source_epoch)
        previous = self._watermarks.get(key)
        source_changed = (
            self._active_source != snapshot.source
            or self._active_epoch != snapshot.source_epoch
        )
        if previous is not None:
            if snapshot.revision_ordinal < previous:
                return None
            if snapshot.revision_ordinal == previous and not source_changed:
                return None
        if source_changed:
            self._stream_generation += 1
            self._active_source = snapshot.source
            self._active_epoch = snapshot.source_epoch
        self._watermarks[key] = snapshot.revision_ordinal
        return replace(snapshot, stream_generation=self._stream_generation)

    def cancel(self) -> None:
        with self._lock:
            if self._closed or self._cancelled:
                return
            self._cancelled = True
            self._live_source.cancel()
            self._saved_source.cancel()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._cancelled = True
            self._opened = False
            self._live_source.close()
            self._saved_source.close()
            self._active_source = None
            self._active_epoch = None
