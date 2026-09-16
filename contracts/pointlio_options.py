"""Point原生调参的保存校验；与Rust启动预检共享参数定义。"""
from __future__ import annotations

import json
import math
from pathlib import Path

POINTLIO_OPTIONS_ENV = "NUWAX_GEOMETRY_POINTLIO_OPTIONS"
_SCHEMA = json.loads(Path(__file__).with_suffix(".json").read_text(encoding="utf-8"))


def _finite_number(value) -> bool:
    return type(value) in (int, float) and abs(value) <= 3.4028234663852886e38 and math.isfinite(value)


def normalize_pointlio_options(raw: str) -> str:
    """校验明确支持的参数；留空撤销覆盖，禁止悄悄忽略键或错误类型。"""
    if not raw.strip():
        return ""
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Point参数重复：{key}")
            result[key] = value
        return result
    data = json.loads(raw, object_pairs_hook=unique_object)
    if not isinstance(data, dict):
        raise ValueError("Point参数必须是JSON对象")
    for name, value in data.items():
        spec = _SCHEMA["parameters"].get(name)
        if spec is None:
            reason = _SCHEMA["unsupported"].get(name, "未知参数")
            raise ValueError(f"Point参数{name}不受支持：{reason}")
        kind = spec["type"]
        if kind == "bool":
            valid = isinstance(value, bool)
        elif kind == "enum":
            valid = isinstance(value, str) and value in spec["choices"]
        elif kind == "vector3":
            valid = isinstance(value, list) and len(value) == 3 and all(
                _finite_number(v) for v in value
            ) and sum(v*v for v in value) > 1e-12
        else:
            valid = type(value) in ((int,) if kind == "int" else (int, float)) and _finite_number(value)
            if valid and "minimum" in spec:
                valid = value >= spec["minimum"]
            if valid and "exclusive_minimum" in spec:
                valid = value > spec["exclusive_minimum"]
            if valid and "maximum" in spec:
                valid = value <= spec["maximum"]
            # C++参数仍使用float解析，校验其可表示范围。
            if valid and kind == "number":
                valid = abs(value) <= 3.4028234663852886e38 and (value == 0 or abs(value) >= 1.1754943508222875e-38)
        if not valid:
            raise ValueError(f"Point参数{name}的类型或范围无效")
    if data.get("use_imu_as_input", False) and not data.get("imu_en", True):
        raise ValueError("惯性输入模式必须启用惯性")
    if data.get("blind", .5) >= data.get("det_range", 100.):
        raise ValueError("Point盲区必须小于最大距离")
    return json.dumps(data, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
