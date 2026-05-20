from __future__ import annotations

from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any
import json

from fastapi.encoders import jsonable_encoder
from pydantic import BaseModel


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
