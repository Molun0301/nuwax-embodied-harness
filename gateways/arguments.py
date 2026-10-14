from __future__ import annotations

from typing import Any, Dict

from gateways.errors import GatewayError


def validate_closed_schema(value: Any, schema: Dict[str, Any], *, path: str) -> None:
    """按公共能力输入结构拒绝未声明字段。"""

    if not isinstance(schema, dict):
        return
    schema_type = schema.get("type")
    if schema_type == "object" or "properties" in schema:
        if not isinstance(value, dict):
            return
        properties = schema.get("properties")
        properties = properties if isinstance(properties, dict) else {}
        if schema.get("additionalProperties") is False:
            unexpected = sorted(str(key) for key in value if key not in properties)
            if unexpected:
                raise GatewayError(
                    f"{path} 包含 schema 未声明的字段: {', '.join(unexpected)}。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                    details={
                        "path": path,
                        "unexpected_properties": unexpected,
                    },
                )
        for key, child_schema in properties.items():
            if key in value:
                validate_closed_schema(
                    value[key],
                    child_schema,
                    path=f"{path}.{key}",
                )
        return
    if schema_type == "array" and isinstance(value, list):
        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, item in enumerate(value):
                validate_closed_schema(
                    item,
                    item_schema,
                    path=f"{path}[{index}]",
                )
