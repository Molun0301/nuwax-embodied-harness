"""协议共用的完整结果与错误编码。"""
from __future__ import annotations

from typing import Any

from core.errors import as_gateway_error
from gateways.serialization import to_jsonable, to_json_text


def build_public_error_payload(error, **_context) -> dict:
    """错误语义由业务层拥有，协议层保留原始详情。"""
    error = as_gateway_error(error)
    return {"code": error.error_code, "message": error.message,
            "details": to_jsonable(error.details)}


def build_task_summary(status, **_unused) -> dict:
    """返回任务执行器拥有的完整快照。"""
    return to_jsonable(status)


def build_public_runtime_event(event: Any) -> dict:
    return to_jsonable(event)


def build_mcp_tool_result(result: Any) -> dict:
    """MCP（模型上下文协议）的文本与结构化通道承载同一完整结果。"""
    content = to_jsonable(result)
    return {"isError": isinstance(content, dict) and bool(content.get("error")),
            "content": [{"type": "text", "text": to_json_text(content)}],
            "structuredContent": content}
