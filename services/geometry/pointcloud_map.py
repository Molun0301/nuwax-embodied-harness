from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


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
    """按 PCD 路径、mtime 和大小缓存累计点云地图。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._items: Dict[Tuple[str, int, int], PointCloudMap] = {}

    def load_pcd(
        self,
        path: Path,
        *,
        frame_id: str = "map",
        source_kind: str = "map_version_pcd",
    ) -> PointCloudMap:
        resolved = path.expanduser().resolve(strict=False)
        if not resolved.exists() or not resolved.is_file():
            raise PointCloudMapError("PCD 点云地图不存在：%s" % resolved)
        stat = resolved.stat()
        key = (str(resolved), int(stat.st_mtime_ns), int(stat.st_size))
        with self._lock:
            cached = self._items.get(key)
            if cached is not None:
                return cached
        started_at = time.perf_counter()
        header, points_xyz = read_pcd_xyz(resolved)
        if points_xyz.size == 0:
            raise PointCloudMapError("PCD 点云地图没有可用 x/y/z 点。")
        finite_mask = np.isfinite(points_xyz).all(axis=1)
        if not bool(finite_mask.all()):
            points_xyz = points_xyz[finite_mask]
        if points_xyz.size == 0:
            raise PointCloudMapError("PCD 点云地图全部点均为非有限数。")
        points_xyz = np.asarray(points_xyz, dtype=np.float32, order="C")
        bounds_min_arr = points_xyz.min(axis=0)
        bounds_max_arr = points_xyz.max(axis=0)
        source_timestamp = datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc)
        loaded = PointCloudMap(
            path=str(resolved),
            revision="pcd:%d:%d" % (int(stat.st_mtime_ns), int(stat.st_size)),
            frame_id=str(frame_id or "map"),
            source_kind=str(source_kind or "map_version_pcd"),
            point_count=int(points_xyz.shape[0]),
            points_xyz=points_xyz,
            bounds_min=tuple(float(value) for value in bounds_min_arr),
            bounds_max=tuple(float(value) for value in bounds_max_arr),
            header=header,
            loaded_at=datetime.now(timezone.utc),
            source_timestamp=source_timestamp,
            load_ms=(time.perf_counter() - started_at) * 1000.0,
        )
        with self._lock:
            self._items[key] = loaded
            if len(self._items) > 8:
                newest_keys = sorted(self._items.keys(), key=lambda item: (item[1], item[2]))[-8:]
                self._items = {item: self._items[item] for item in newest_keys}
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
        handle.write(points.tobytes(order="C"))


def read_ply_xyz(path: Path) -> Tuple[PcdHeader, np.ndarray]:
    """读取 PLY 顶点坐标，返回与 read_pcd_xyz 相同形状的 float32 Nx3 数组。"""

    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise PointCloudMapError("PLY 点云文件不存在：%s" % resolved)
    header_lines: List[str] = []
    with resolved.open("rb") as handle:
        while True:
            line_bytes = handle.readline()
            if not line_bytes:
                raise PointCloudMapError("PLY 文件缺少 end_header。")
            line = line_bytes.decode("utf-8", errors="replace").strip()
            header_lines.append(line)
            if line == "end_header":
                break
        format_line = next((item for item in header_lines if item.startswith("format ")), "")
        vertex_count = 0
        properties: List[Tuple[str, str]] = []
        in_vertex = False
        for line in header_lines:
            if line.startswith("element "):
                parts = line.split()
                in_vertex = len(parts) >= 3 and parts[1] == "vertex"
                if in_vertex:
                    vertex_count = int(parts[2])
                    properties = []
                continue
            if in_vertex and line.startswith("property "):
                parts = line.split()
                if len(parts) >= 3:
                    properties.append((parts[1], parts[2]))
        if vertex_count <= 0:
            raise PointCloudMapError("PLY 文件缺少 vertex 数量。")
        if "binary_little_endian" in format_line:
            dtype_fields = []
            for kind, name in properties:
                if kind != "float":
                    raise UnsupportedPointCloudMapError("PLY 仅支持 float 顶点属性。")
                size = 8 if name == "double" else 4
                dtype_fields.append((name, "<f%d" % size))
            if not dtype_fields:
                raise PointCloudMapError("PLY 顶点没有属性定义。")
            vertex_dtype = np.dtype(dtype_fields)
            payload = handle.read(vertex_count * vertex_dtype.itemsize)
            if len(payload) < vertex_count * vertex_dtype.itemsize:
                raise PointCloudMapError("PLY 二进制数据不完整。")
            vertices = np.frombuffer(payload, dtype=vertex_dtype, count=vertex_count)
            if "x" not in vertices.dtype.names or "y" not in vertices.dtype.names or "z" not in vertices.dtype.names:
                raise PointCloudMapError("PLY 顶点缺少 x/y/z 字段。")
            xyz = np.stack(
                [vertices["x"], vertices["y"], vertices["z"]],
                axis=1,
            ).astype(np.float32, copy=False)
            header = PcdHeader(
                fields=("x", "y", "z"),
                size=(4, 4, 4),
                type=("F", "F", "F"),
                count=(1, 1, 1),
                width=vertex_count,
                height=1,
                points=vertex_count,
                data="binary",
                header_bytes=0,
                raw={},
            )
            return header, xyz
        if "ascii" in format_line:
            values = np.fromfile(handle, sep=" ", dtype=np.float64)
            width = len(properties) or 3
            if values.size < vertex_count * width:
                raise PointCloudMapError("PLY ASCII 数据不完整。")
            matrix = values[: vertex_count * width].reshape(vertex_count, width)
            name_to_index = {name: index for index, (_, name) in enumerate(properties)}
            try:
                xyz = matrix[:, [name_to_index["x"], name_to_index["y"], name_to_index["z"]]].astype(np.float32)
            except KeyError as exc:
                raise PointCloudMapError("PLY 顶点缺少 x/y/z 字段。") from exc
            header = PcdHeader(
                fields=("x", "y", "z"),
                size=(4, 4, 4),
                type=("F", "F", "F"),
                count=(1, 1, 1),
                width=vertex_count,
                height=1,
                points=vertex_count,
                data="ascii",
                header_bytes=0,
                raw={},
            )
            return header, xyz
        raise UnsupportedPointCloudMapError("不支持的 PLY 格式：%s" % format_line)


def read_pcd_point_count(path: Path) -> int:
    """仅解析 PCD 头中的 POINTS 数量，用于快速校验资产是否可用。"""

    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise PointCloudMapError("PCD 点云文件不存在：%s" % resolved)
    with resolved.open("rb") as stream:
        header = _read_pcd_header(stream)
    return int(header.points)


def read_pcd_xyz(path: Path) -> Tuple[PcdHeader, np.ndarray]:
    """读取 PCD 文件中的 x/y/z 字段，返回 float32 Nx3 数组。"""

    with path.open("rb") as stream:
        header = _read_pcd_header(stream)
        if header.data == "ascii":
            payload = stream.read().decode("utf-8", errors="replace")
            return header, _read_ascii_xyz(header, payload)
        if header.data == "binary":
            payload = stream.read()
            return header, _read_binary_xyz(header, payload)
        if header.data == "binary_compressed":
            raise UnsupportedPointCloudMapError("当前不支持 binary_compressed PCD 点云地图。")
        raise UnsupportedPointCloudMapError("不支持的 PCD DATA 类型：%s" % header.data)


def _read_pcd_header(stream) -> PcdHeader:
    raw: Dict[str, str] = {}
    header_bytes = 0
    while True:
        line_bytes = stream.readline()
        if not line_bytes:
            raise PointCloudMapError("PCD 文件缺少 DATA 行。")
        header_bytes += len(line_bytes)
        line = line_bytes.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
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


def _read_ascii_xyz(header: PcdHeader, payload: str) -> np.ndarray:
    columns = _expanded_columns(header.fields, header.count)
    indexes = _xyz_indexes(columns)
    row_width = len(columns)
    values = np.fromstring(payload, sep=" ", dtype=np.float64)
    if values.size < row_width:
        return np.empty((0, 3), dtype=np.float32)
    row_count = values.size // row_width
    values = values[: row_count * row_width].reshape((row_count, row_width))
    xyz = values[:, indexes]
    return np.asarray(xyz, dtype=np.float32)


def _read_binary_xyz(header: PcdHeader, payload: bytes) -> np.ndarray:
    dtype = _pcd_binary_dtype(header)
    if dtype.itemsize <= 0:
        raise PointCloudMapError("PCD binary dtype 无效。")
    available_points = len(payload) // dtype.itemsize
    count = header.points if header.points > 0 else available_points
    count = min(count, available_points)
    records = np.frombuffer(payload, dtype=dtype, count=count)
    try:
        x = _field_values(records, "x")
        y = _field_values(records, "y")
        z = _field_values(records, "z")
    except ValueError as exc:
        raise PointCloudMapError(str(exc)) from exc
    xyz = np.column_stack((x, y, z))
    return np.asarray(xyz, dtype=np.float32)


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
