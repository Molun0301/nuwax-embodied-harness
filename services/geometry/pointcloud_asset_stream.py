"""点云资产的有界顺序读取；完整读取与网页抽样共用格式校验。"""

from contextlib import contextmanager
from itertools import chain
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from services.geometry.pointcloud_map import (
    PcdHeader,
    PointCloudMapError,
    UnsupportedPointCloudMapError,
    _expanded_columns,
    _field_values,
    _pcd_binary_dtype,
    _read_pcd_header,
    _xyz_indexes,
)


CHUNK_BYTES = 4 * 1024 * 1024
ASCII_CHUNK_POINTS = 8192
_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
}


class PointCloudReadCancelled(PointCloudMapError):
    """读取被拥有方取消，不作为资产损坏。"""


def check_cancelled(should_cancel: Optional[Callable[[], bool]]) -> None:
    if should_cancel is not None and should_cancel():
        raise PointCloudReadCancelled("点云资产读取已取消。")


def finite_xyz_mask(points):
    """按坐标列计算掩码，避免对每个三元素短行重复归约。"""
    return np.isfinite(points[:, 0]) & np.isfinite(points[:, 1]) & np.isfinite(points[:, 2])


def xyz_bounds(points):
    """连续遍历各坐标列，保持与整数组按行归约相同的边界。"""
    return ([points[:, axis].min() for axis in range(3)],
            [points[:, axis].max() for axis in range(3)])


def _read_ply_header(stream):
    if stream.readline().strip() != b"ply":
        raise PointCloudMapError("PLY 文件缺少格式标识。")
    encoding = ""
    vertex_count = 0
    properties = []
    in_vertex = False
    seen_vertex = False
    for _ in range(4096):
        raw = stream.readline(65537)
        if not raw or len(raw) > 65536:
            raise PointCloudMapError("PLY 文件头不完整或过长。")
        parts = raw.decode("ascii", errors="strict").split()
        if not parts or parts[0] in {"comment", "obj_info"}:
            continue
        if parts[0] == "end_header":
            break
        if parts[0] == "format" and len(parts) == 3:
            encoding = parts[1]
        elif parts[0] == "element" and len(parts) == 3:
            count = int(parts[2])
            if count < 0:
                raise PointCloudMapError("PLY 元素数量不能为负。")
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                if seen_vertex:
                    raise PointCloudMapError("PLY 包含重复 vertex 定义。")
                seen_vertex = True
                vertex_count = count
            elif not seen_vertex and count:
                raise UnsupportedPointCloudMapError("PLY 顶点前的非空元素暂不支持。")
        elif in_vertex and parts[0] == "property":
            if len(parts) != 3 or parts[1] not in _PLY_TYPES:
                raise UnsupportedPointCloudMapError("PLY 不支持列表或未知顶点属性。")
            properties.append((parts[2], _PLY_TYPES[parts[1]]))
    else:
        raise PointCloudMapError("PLY 文件缺少 end_header。")
    if vertex_count <= 0:
        raise PointCloudMapError("PLY 文件缺少有效 vertex 数量。")
    if encoding not in {"ascii", "binary_little_endian"}:
        raise UnsupportedPointCloudMapError("不支持的 PLY 格式：%s" % encoding)
    dtype = np.dtype(properties)
    _xyz_indexes(list(dtype.names or ()))
    header = PcdHeader(
        fields=tuple(dtype.names), size=tuple(dtype[name].itemsize for name in dtype.names),
        type=tuple({"f": "F", "i": "I", "u": "U"}[dtype[name].kind] for name in dtype.names),
        count=(1,) * len(properties), width=vertex_count, height=1, points=vertex_count,
        data="ascii" if encoding == "ascii" else "binary", header_bytes=stream.tell(), raw={},
    )
    return header, dtype


def _iter_binary(stream, header, dtype, should_cancel):
    if dtype.itemsize <= 0 or dtype.itemsize > CHUNK_BYTES:
        raise PointCloudMapError("点云记录长度无效或超过读取块预算。")
    # 在分配或遍历前检查声明长度，截断资产不能作为成功的小地图返回。
    import os

    available = os.fstat(stream.fileno()).st_size - stream.tell()
    if header.points * dtype.itemsize > available:
        raise PointCloudMapError("点云二进制数据不完整。")
    for start in range(0, header.points, max(1, CHUNK_BYTES // dtype.itemsize)):
        check_cancelled(should_cancel)
        count = min(header.points - start, max(1, CHUNK_BYTES // dtype.itemsize))
        records = np.fromfile(stream, dtype=dtype, count=count)
        if len(records) != count:
            raise PointCloudMapError("点云二进制数据不完整。")
        yield np.column_stack([_field_values(records, name) for name in ("x", "y", "z")])


def _iter_ascii(stream, header, should_cancel):
    columns = _expanded_columns(header.fields, header.count)
    indexes = _xyz_indexes(columns)
    remaining = header.points
    while remaining:
        check_cancelled(should_cancel)
        wanted = min(remaining, ASCII_CHUNK_POINTS)
        rows = []
        input_bytes = 0
        while len(rows) < wanted and input_bytes < CHUNK_BYTES:
            raw = stream.readline(65537)
            if not raw:
                raise PointCloudMapError("点云文本数据不完整。")
            if len(raw) > 65536:
                raise PointCloudMapError("点云文本记录过长。")
            input_bytes += len(raw)
            row = raw.strip()
            if row and not row.startswith(b"#"):
                rows.append(row)
        if not rows:
            continue
        count = len(rows)
        try:
            values = np.loadtxt(rows, dtype=np.float32, ndmin=2)
        except ValueError as exc:
            raise PointCloudMapError("点云文本记录格式错误。") from exc
        if values.shape != (count, len(columns)):
            raise PointCloudMapError("点云文本字段数量与文件头不一致。")
        yield np.ascontiguousarray(values[:, indexes])
        remaining -= count


def _checked_chunks(chunks, stream, header, suffix, should_cancel):
    yield from chunks
    # PLY顶点后可以跟面等元素；PCD文本多出的点不能绕过声明点数校验。
    if suffix == ".pcd" and header.data == "ascii":
        while True:
            check_cancelled(should_cancel)
            line = stream.readline(65537)
            if not line:
                break
            if len(line) > 65536 or (line.strip() and not line.lstrip().startswith(b"#")):
                raise PointCloudMapError("PCD 文本点数超过头部声明。")


@contextmanager
def open_pointcloud_chunks(path: Path, *, should_cancel=None, format_hint=None):
    """以至多4MiB二进制输入块读取XYZ；上下文退出即关闭文件。"""

    check_cancelled(should_cancel)
    suffix = format_hint or path.suffix.lower()
    try:
        with path.open("rb") as stream:
            if suffix == ".pcd":
                header = _read_pcd_header(stream)
                dtype = _pcd_binary_dtype(header)
            elif suffix == ".ply":
                header, dtype = _read_ply_header(stream)
            else:
                raise UnsupportedPointCloudMapError("不支持的点云地图扩展名：%s" % suffix)
            _xyz_indexes(list(header.fields))
            if header.points <= 0:
                raise PointCloudMapError("点云文件没有有效点数声明。")
            if header.data == "binary":
                chunks = _iter_binary(stream, header, dtype, should_cancel)
            elif header.data == "ascii":
                chunks = _iter_ascii(stream, header, should_cancel)
            else:
                raise UnsupportedPointCloudMapError("不支持的点云 DATA 类型：%s" % header.data)
            yield header, _checked_chunks(chunks, stream, header, suffix, should_cancel)
    except (ValueError, OverflowError, UnicodeError) as exc:
        raise PointCloudMapError("点云文件头或字段无效：%s" % exc) from exc


def read_all_xyz(path: Path, *, format_hint: str):
    """保留完整数组接口，仅分配一份最终XYZ和有界临时块。"""

    with open_pointcloud_chunks(path, format_hint=format_hint) as (header, chunks):
        # 先读取首块，使损坏的巨大点数声明在全量分配前失败。
        first = next(chunks)
        points = np.empty((header.points, 3), dtype=np.float32)
        points[:len(first)] = first
        offset = len(first)
        for chunk in chunks:
            points[offset:offset + len(chunk)] = chunk
            offset += len(chunk)
        return header, points


def read_preview_xyz(path: Path, max_points: Optional[int], *, should_cancel=None):
    """全量校验与统计，均匀选择文件位置；保留点数与资产总点数分开返回。"""

    with open_pointcloud_chunks(path, should_cancel=should_cancel) as (header, chunks):
        first = next(chunks)
        selected_count = header.points if max_points is None else min(header.points, max_points)
        # 整数等间距索引包含两端，不受块边界影响，也不分配与原始点数等长的索引。
        if selected_count == header.points:
            indices = None
        elif selected_count == 1:
            indices = np.zeros(1, dtype=np.int64)
        else:
            indices = (np.arange(selected_count, dtype=np.int64)
                       * (header.points - 1) // (selected_count - 1))
        selected = np.empty((selected_count, 3), dtype=np.float32)
        lower = np.full(3, np.inf)
        upper = np.full(3, -np.inf)
        offset = kept = total = 0
        first_finite = None
        for chunk in chain((first,), chunks):
            mask = finite_xyz_mask(chunk)
            valid = chunk if bool(mask.all()) else chunk[mask]
            total += len(valid)
            if len(valid):
                if first_finite is None:
                    first_finite = valid[0].copy()
                chunk_lower, chunk_upper = xyz_bounds(valid)
                lower = np.minimum(lower, chunk_lower)
                upper = np.maximum(upper, chunk_upper)
            if indices is None:
                selected[kept:kept + len(valid)] = valid
                kept += len(valid)
            else:
                begin, end = np.searchsorted(indices, [offset, offset + len(chunk)])
                local = indices[begin:end] - offset
                local = local[mask[local]]
                selected[kept:kept + len(local)] = chunk[local]
                kept += len(local)
            offset += len(chunk)
        if not total:
            raise PointCloudMapError("点云地图没有可用有限点。")
        if not kept:
            selected[0] = first_finite
            kept = 1
        # 释放未用的预分配容量，避免大量无效点让缓存低估实际占用。
        points = selected if kept == selected_count else selected[:kept].copy()
        return header, points, total, tuple(lower), tuple(upper)
