from __future__ import annotations

from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any
import json

from fastapi.encoders import jsonable_encoder
from pydantic import BaseModel

from contracts.map_workspace import MapAssetBundle, MapWorkspace
from contracts.maps import CostMap, OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot


def to_jsonable(value: Any) -> Any:
    """把任意运行时对象转换为可序列化结构。"""

    # 快速路径：Pydantic 模型直接使用 model_dump，比 jsonable_encoder 快 2-5x
    if isinstance(value, BaseModel):
        return value.model_dump(mode="json")
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        value = asdict(value)
    return jsonable_encoder(
        value,
        custom_encoder={
            Path: str,
            bytes: lambda data: f"<bytes:{len(data)}>",
        },
    )


def to_json_text(value: Any) -> str:
    """导出适合 MCP 文本通道的 JSON 字符串。"""

    return json.dumps(to_jsonable(value), ensure_ascii=False, sort_keys=True)


def to_map_json_bytes(value: Any) -> bytes:
    """在工作线程编码完整地图响应，限制单次栅格展开与编码的工作量。"""

    # 其它字段仍由正式模型转换时间、枚举等类型；仅延后地图嵌套字段和栅格数据。
    deferred_fields = {
        MapWorkspace: {"latest_map_snapshot"},
        MapAssetBundle: {"snapshot"},
        MapSnapshot: {"occupancy_grid", "cost_map"},
        OccupancyGrid: {"data"},
        CostMap: {"data"},
    }

    def compact(item):
        return json.dumps(
            item, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")

    def cells(data):
        if isinstance(data, PackedOccupancyCells):
            yield data.canonical_json
            return
        yield b"["
        for start in range(0, len(data), 16384):
            if start:
                yield b","
            yield compact(data[start:start + 16384])[1:-1]
        yield b"]"

    def encode(item):
        deferred = deferred_fields.get(type(item))
        if deferred is not None:
            payload = item.model_dump(mode="json", exclude=deferred)
            yield b"{"
            # 仅输出序列化器保留的键与延后字段，仍按正式模型的字段顺序编码。
            fields = type(item).model_fields
            names = [name for name in fields if name in payload or name in deferred]
            names.extend(name for name in payload if name not in fields)
            for index, name in enumerate(names):
                if index:
                    yield b","
                yield compact(name) + b":"
                if name in deferred:
                    child = getattr(item, name)
                    if name == "data":
                        yield from cells(child)
                    else:
                        yield from encode(child)
                else:
                    yield from encode(payload[name])
            yield b"}"
        elif isinstance(item, dict):
            yield b"{"
            for index, (key, child) in enumerate(item.items()):
                if index:
                    yield b","
                # 字典键保留标准JSON的数字、布尔及空值转换规则。
                yield compact({key: None})[1:-5]
                yield from encode(child)
            yield b"}"
        elif isinstance(item, (list, tuple)):
            yield b"["
            for index, child in enumerate(item):
                if index:
                    yield b","
                yield from encode(child)
            yield b"]"
        else:
            yield compact(to_jsonable(item))

    # 发送前完成编码，错误仍由既有HTTP错误处理器返回，不能发出半份地图。
    return b"".join(encode(value))
