from __future__ import annotations

import json
from functools import lru_cache
from typing import Any, Dict

from jsonschema.exceptions import SchemaError, ValidationError
from jsonschema.validators import validator_for

from gateways.errors import GatewayError


@lru_cache(maxsize=128)
def _prepared_argument_validator(schema_json: str):
    """按完整结构内容复用验证器，注册替换和原地更新自然产生新版本。"""
    schema = json.loads(schema_json)
    stack = [schema]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            for key in ("$ref", "$dynamicRef", "$recursiveRef"):
                ref = item.get(key)
                if isinstance(ref, str) and not ref.startswith("#"):
                    raise GatewayError("工具参数结构含不允许的远端引用。",
                                       error_code="agent_schema_unsupported", http_status=500)
            stack.extend(item.values())
        elif isinstance(item, list):
            stack.extend(item)
    validator = validator_for(schema)
    validator.check_schema(schema)
    return validator(schema)


def validate_arguments(arguments: Dict[str, Any], schema: Dict[str, Any]) -> None:
    """每次只验证实际参数；结构版本改变才检查定义与引用。"""
    try:
        schema_json = json.dumps(schema, sort_keys=True, separators=(",", ":"), allow_nan=False)
        # 限制缓存保留的单项大小；超大结构仍按原完整校验执行。
        prepare = _prepared_argument_validator if len(schema_json) <= 64 * 1024 else _prepared_argument_validator.__wrapped__
        prepare(schema_json).validate(arguments)
    except ValidationError as exc:
        raise GatewayError("参数不符合工具声明的输入结构。", error_code="invalid_params", http_status=422, jsonrpc_code=-32602,
                           details={"path": list(exc.absolute_path), "validator": exc.validator}) from exc
    except (SchemaError, TypeError, ValueError) as exc:
        raise GatewayError("工具参数结构无效。", error_code="agent_schema_invalid", http_status=500) from exc
