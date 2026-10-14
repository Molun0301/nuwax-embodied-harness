"""库内PGO参数保存校验，与Rust启动读取共享字段定义。"""
from __future__ import annotations

import json
import math
from pathlib import Path

PGO_OPTIONS_ENV = "NUWAX_GEOMETRY_PGO_OPTIONS"
_SCHEMA = json.loads(Path(__file__).with_suffix(".json").read_text(encoding="utf-8"))


def normalize_pgo_options(raw: str) -> str:
    """仅覆盖显式提供的参数；空值恢复所选前端默认，非法输入不降级。"""
    if not raw.strip():
        return ""

    def unique_object(pairs):
        result = {}
        for name, value in pairs:
            if name in result:
                raise ValueError(f"PGO参数重复：{name}")
            result[name] = value
        return result

    data = json.loads(raw, object_pairs_hook=unique_object)
    if not isinstance(data, dict):
        raise ValueError("PGO参数必须是JSON对象")
    for name, value in data.items():
        spec = _SCHEMA["parameters"].get(name)
        if spec is None:
            raise ValueError(f"未知PGO参数：{name}")
        valid = type(value) in ((int,) if spec["type"] == "int" else (int, float))
        valid = valid and math.isfinite(value)
        if valid and "minimum" in spec:
            valid = value >= spec["minimum"]
        if valid and "exclusive_minimum" in spec:
            valid = value > spec["exclusive_minimum"]
        if valid and "maximum" in spec:
            valid = value <= spec["maximum"]
        if not valid:
            raise ValueError(f"PGO参数{name}的类型或范围无效：{spec['description']}")
    return json.dumps(data, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
