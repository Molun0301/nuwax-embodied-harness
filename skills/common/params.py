"""参数校验公共助手（P2-4 批0 脚手架）。

从 gateways/runtime.py 下放的纯参数校验语义函数，供 skills 层 handler 与网关共用。
仅依赖 core.errors.GatewayError（core 为 skills 合法下游，不构成 skills→gateways 反向依赖）。

行为与抛错条件与迁移前的 GatewayRuntime._require_str/_require_float/_float/
_optional_str/_normalize_query 完全一致。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from core.errors import GatewayError

__all__ = [
    "require_str",
    "require_float",
    "optional_float",
    "optional_int",
    "optional_str",
    "normalize_query",
    "coerce_optional_str_list",
]


def require_str(payload: Dict[str, Any], key: str) -> str:
    """取必填字符串参数，缺失或空白抛 GatewayError。"""

    value = str(payload.get(key, "")).strip()
    if not value:
        raise GatewayError(
            f"参数 {key} 不能为空。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    return value


def require_float(payload: Dict[str, Any], key: str) -> float:
    """取必填浮点参数，缺失键抛 GatewayError，非数字抛 GatewayError。"""

    if key not in payload:
        raise GatewayError(
            f"缺少参数 {key}。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    return optional_float(payload, key, 0.0)


def optional_float(payload: Dict[str, Any], key: str, default: float) -> float:
    """取可选浮点参数带默认值，存在但非数字抛 GatewayError。

    对应迁移前的 GatewayRuntime._float。
    """

    if key not in payload:
        return default
    try:
        return float(payload[key])
    except (TypeError, ValueError) as exc:
        raise GatewayError(
            f"参数 {key} 必须为数字。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        ) from exc


def optional_int(payload: Dict[str, Any], key: str, default: int) -> int:
    """取可选整数参数带默认值，存在但非整数抛 GatewayError。

    对应迁移前的 GatewayRuntime._int。
    """

    if key not in payload:
        return default
    try:
        return int(payload[key])
    except (TypeError, ValueError) as exc:
        raise GatewayError(
            f"参数 {key} 必须为整数。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        ) from exc


def optional_str(payload: Dict[str, Any], key: str) -> Optional[str]:
    """取可选字符串参数，None/bool/空白归一为 None。"""

    value = payload.get(key)
    if value is None or isinstance(value, bool):
        return None
    normalized = str(value).strip()
    return normalized or None


def normalize_query(value: Optional[str]) -> str:
    """归一化查询串：保留字母数字与中文字符（小写化），其余转空白并压缩。"""

    normalized = "".join(
        char.lower() if char.isalnum() or "一" <= char <= "鿿" else " "
        for char in str(value or "")
    )
    return " ".join(normalized.split())


def coerce_optional_str_list(value: Any) -> List[str]:
    """将任意输入归一化为去空白后的非空字符串列表。

    对应迁移前的 GatewayRuntime._coerce_optional_str_list，行为完全一致：
    None/bool 视为空；字符串取 strip 后单元素列表；list/tuple/set/frozenset
    逐项 strip 过滤空白；其余标量 str 化。
    """

    if value is None or isinstance(value, bool):
        return []
    if isinstance(value, str):
        normalized = value.strip()
        return [normalized] if normalized else []
    if isinstance(value, (list, tuple, set, frozenset)):
        return [str(item).strip() for item in value if str(item or "").strip()]
    return [str(value).strip()] if str(value).strip() else []
