from __future__ import annotations

import asyncio
import json
import logging
import time
from collections import OrderedDict
from uuid import uuid4
from typing import Dict, Optional, Any, List, Mapping, Tuple

from contracts.skills import SkillDescriptor
from fastapi import APIRouter
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse
from fastapi.responses import Response
from fastapi.responses import StreamingResponse

from gateways.arguments import validate_closed_schema as _validate_closed_schema
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError, as_gateway_error
from gateways.feedback import (
    MCP_RESULT_MAX_BYTES,
    build_mcp_tool_result,
    build_public_error_payload,
)
from gateways.request_context import categorize_gateway_error
from gateways.runtime import GatewayRuntime
from gateways.updates import run_mcp_operation
from gateways.feedback_stream import feedback_cursor, feedback_events, format_feedback_sse


MCP_LOGGER = logging.getLogger("nuwax_robot_bridge.gateway.mcp")
MCP_JSONRPC_ID_MAX_BYTES = 512
MCP_REQUEST_MAX_BYTES = 256 * 1024


def _jsonrpc_result(request_id: Any, result: Dict[str, Any]) -> Dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


_SUPPORTED_PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18")
_DEFAULT_PROTOCOL_VERSION = _SUPPORTED_PROTOCOL_VERSIONS[0]


def _jsonrpc_error(request_id: Any, *, code: int, message: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    error: Dict[str, Any] = {"code": code, "message": message}
    if data:
        error["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": error}


def _strict_json_constant(value: str) -> None:
    raise ValueError(f"Non-standard JSON constant: {value}")


def _validated_jsonrpc_id(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError("JSON-RPC id must be a string, integer, or null")
    if isinstance(value, str):
        encoded_length = len(value.encode("utf-8"))
    else:
        encoded_length = len(str(value).encode("utf-8"))
    if encoded_length > MCP_JSONRPC_ID_MAX_BYTES:
        raise ValueError("JSON-RPC id exceeds the byte limit")
    return value


def _json_bytes(value: Any) -> int:
    return len(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    )


def _public_gateway_error_response(
    request: Request,
    error: GatewayError,
    *,
    rpc_request_id: Any = None,
) -> JSONResponse:
    error_category = categorize_gateway_error(error.error_code, error.http_status)
    request_id = str(getattr(request.state, "request_id", None) or "").strip()
    public_error = build_public_error_payload(
        error,
        request_id=request_id or None,
        error_category=error_category,
    )
    return JSONResponse(
        _jsonrpc_error(
            rpc_request_id,
            code=error.jsonrpc_code,
            message=public_error["message"],
            data=public_error,
        ),
        status_code=error.http_status,
    )


def _build_mcp_request_context(
    request: Request,
    *,
    rpc_request_id: Any,
    method: str,
    tool_name: Optional[str] = None,
) -> Dict[str, Any]:
    state_context = getattr(request.state, "request_context", None)
    context = dict(state_context) if isinstance(state_context, Mapping) else {}
    request_id = getattr(request.state, "request_id", None)
    if request_id:
        context["request_id"] = request_id
    context.setdefault("method", request.method)
    context.setdefault("path", request.url.path)
    client = request.client
    if client is not None:
        context.setdefault("client_ip", client.host)
    context["mcp_method"] = method
    if rpc_request_id is not None:
        context["rpc_request_id"] = rpc_request_id
    if tool_name:
        context["tool_name"] = tool_name
    return {key: value for key, value in context.items() if value is not None and value != ""}


_MCP_COLLECTIONS = {
    "platform": {"display_name": "平台侧功能集合", "layers": {"platform"}},
    "device": {"display_name": "端侧设备功能集合", "layers": {"robot_private", "ops_debug"}},
}

# 这些能力除收录在 platform.function 枚举外，还以独立 MCP tool 暴露，
# 避免 NuwaClaw 代理缓存的 platform 枚举漏掉新功能时 Agent 无法发现。
#
# V2 扩展（navigation-and-exploration-boundary §6.3）：从 3 扩到 7 个 tool：
#   start_mapping / stop_mapping / stop_exploration / navigate_to_pose /
#   cancel_navigation / get_navigation_state / start_exploration（v2 带 boundary + typed stop conditions）。
_MCP_STANDALONE_PLATFORM_TOOLS: Tuple[str, ...] = (
    "start_mapping",
    "stop_mapping",
    "stop_exploration",
    "navigate_to_pose",
    "cancel_navigation",
    "get_navigation_state",
    "start_exploration",
)

_MCP_STANDALONE_TOOL_TARGETS: Dict[str, str] = {
    tool_name: tool_name for tool_name in _MCP_STANDALONE_PLATFORM_TOOLS
}
_MCP_STANDALONE_TOOL_TARGETS["start_exploration"] = "explore_area_v2"


# V2 新增 tool 的 input_schema（spec mcp-end-to-end §MCP 工具四件套 + §错误码契约统一）。
_MCP_STANDALONE_TOOL_SCHEMAS: Dict[str, Dict[str, Any]] = {
    "navigate_to_pose": {
        "type": "object",
        "properties": {
            "pose": {
                "type": "object",
                "description": "目标位姿 Pose2D。",
                "properties": {
                    "x": {"type": "number", "description": "X 坐标（米）。"},
                    "y": {"type": "number", "description": "Y 坐标（米）。"},
                    "yaw": {"type": "number", "description": "偏航角（弧度）。"},
                    "frame_id": {"type": "string", "description": "坐标系 ID，默认 map。"},
                },
                "required": ["x", "y"],
            },
            "timeout_sec": {
                "type": "integer",
                "minimum": 1,
                "default": 60,
                "description": "超时秒数（默认 60s）。",
            },
        },
        "required": ["pose"],
        "additionalProperties": False,
    },
    "cancel_navigation": {
        "type": "object",
        "properties": {
            "request_id": {
                "type": "string",
                "description": "可选；指定 request_id 时仅取消该目标，缺省取消当前 ACTIVE。",
            },
        },
        "additionalProperties": False,
    },
    "get_navigation_state": {
        "type": "object",
        "properties": {},
        "additionalProperties": False,
    },
    "start_exploration": {
        "type": "object",
        "properties": {
            "boundary_line_segments": {
                "type": "array",
                "description": "手划多段线段；每段 [p1, p2]，p = {x, y, yaw?}。",
                "items": {
                    "type": "array",
                    "minItems": 2,
                    "maxItems": 2,
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "yaw": {"type": "number"},
                            "frame_id": {"type": "string"},
                        },
                        "required": ["x", "y"],
                        "additionalProperties": False,
                    },
                },
                "minItems": 1,
            },
            "map_name": {
                "type": "string",
                "description": "目标地图名称（可选；缺省走当前激活地图）。",
            },
            "frame_id": {"type": "string", "default": "map"},
            "strategy": {"type": "string", "default": "wavefront"},
            "arbitration": {"type": "string", "default": "prefer_wavefront"},
            "stop_conditions": {
                "type": "object",
                "properties": {
                    "max_coverage_pct": {
                        "type": "number",
                        "exclusiveMinimum": 0.0,
                        "maximum": 100.0,
                    },
                    "no_frontier_timeout_ms": {"type": "integer", "minimum": 1},
                    "max_duration_ms": {"type": "integer", "minimum": 1},
                    "max_goals": {"type": "integer", "minimum": 1},
                },
                "additionalProperties": False,
            },
            "max_navigation_retries": {
                "type": "integer",
                "minimum": 0,
                "default": 1,
            },
            "blacklist_radius_m": {
                "type": "number",
                "minimum": 0.0,
                "default": 0.75,
            },
            "max_duration_sec": {
                "type": "number",
                "exclusiveMinimum": 0.0,
            },
            "poll_interval_sec": {
                "type": "number",
                "exclusiveMinimum": 0.0,
                "default": 0.2,
            },
            "timeout_sec": {
                "type": "number",
                "exclusiveMinimum": 0.0,
            },
        },
        "additionalProperties": False,
    },
}


def _tool_layer(descriptor: SkillDescriptor) -> str:
    return str(getattr(descriptor, "metadata", {}).get("capability_layer") or "platform")


def _tool_catalog_item(runtime: GatewayRuntime, descriptor: SkillDescriptor, layer: str) -> Dict[str, Any]:
    capability_descriptor = runtime.capability_registry.get_descriptor(descriptor.capability_name)
    return {
        "name": descriptor.name,
        "display_name": descriptor.display_name,
        "description": descriptor.description,
        "input_schema": capability_descriptor.input_schema or {"type": "object", "properties": {}, "additionalProperties": False},
        "output_schema": capability_descriptor.output_schema or {"type": "object", "properties": {}, "additionalProperties": False},
        "execution_mode": capability_descriptor.execution_mode.value,
        "risk_level": capability_descriptor.risk_level.value,
        "capability_layer": layer,
    }


def _build_collection_catalogs(
    runtime: GatewayRuntime,
    *,
    tool_names: Optional[Tuple[str, ...]] = None,
    refresh_availability: bool = True,
) -> Dict[str, Dict[str, Dict[str, Any]]]:
    catalogs: Dict[str, Dict[str, Dict[str, Any]]] = {name: {} for name in _MCP_COLLECTIONS}
    descriptors = runtime.skill_registry.list_descriptors()
    if refresh_availability:
        runtime._refresh_dynamic_capability_matrix()
    for descriptor in descriptors:
        # 调用只读取候选能力，避免为无关工具重复查询支持矩阵和组装参数模式。
        if tool_names is not None and descriptor.name not in tool_names:
            continue
        if not descriptor.exposed_to_agent:
            continue
        layer = _tool_layer(descriptor)
        collection_name = next(
            (name for name, config in _MCP_COLLECTIONS.items() if layer in config["layers"]),
            None,
        )
        if collection_name is None:
            continue
        if refresh_availability:
            try:
                registration = runtime.capability_registry.get_registration(descriptor.capability_name)
                availability = runtime.capability_registry.get_availability(
                    descriptor.capability_name,
                    robot_model=runtime.robot.manifest.robot_model,
                )
            except Exception:
                continue
            if not registration.runnable or not availability.supported:
                continue
        catalogs[collection_name][descriptor.name] = _tool_catalog_item(runtime, descriptor, layer)
    return catalogs


def _build_collection_description(collection_name: str, catalog: Dict[str, Dict[str, Any]]) -> str:
    display_name = _MCP_COLLECTIONS[collection_name]["display_name"]
    if not catalog:
        return f"{display_name}。当前没有可调用功能。"
    function_names = "、".join(sorted(catalog))
    description = (
        f"{display_name}。通过 function 选择具体功能，通过 parameters 传入该功能参数。"
        f"当前可用：{function_names}。参数不符合对应能力 schema 时会返回 invalid_params。"
    )
    relative_schema = catalog.get("relative_move", {}).get("input_schema", {})
    if relative_schema.get("description"):
        description += "\nrelative_move（相对移动）：" + str(relative_schema["description"])
    return description


def _build_collection_input_schema(catalog: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    function_names = sorted(catalog)
    return {
        "type": "object",
        "properties": {
            "function": {
                "type": "string",
                "enum": function_names,
                "description": "集合内具体功能名。",
            },
            "parameters": {
                "type": "object",
                "description": "传给具体功能的参数；服务端按所选能力 schema 严格校验。",
            },
        },
        "required": ["function"],
        "additionalProperties": False,
    }


def _build_standalone_platform_tools(platform_catalog: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    tools: List[Dict[str, Any]] = []
    for tool_name in _MCP_STANDALONE_PLATFORM_TOOLS:
        target_name = _MCP_STANDALONE_TOOL_TARGETS[tool_name]
        item = platform_catalog.get(target_name)
        fallback_schema = _MCP_STANDALONE_TOOL_SCHEMAS.get(tool_name)
        # 目录只发布当前可调用目标；参数模式只能补充已存在目标的描述。
        if item is None:
            continue
        tools.append(
            {
                "name": tool_name,
                "description": (
                    f"{item['display_name']}。{item['description']} "
                    "这是独立 MCP 工具，直接传参调用，不要走 platform.function。"
                ),
                "inputSchema": item.get("input_schema") or fallback_schema or {"type": "object", "properties": {}, "additionalProperties": False},
            }
        )
    return tools


def _build_collection_tools(runtime: GatewayRuntime) -> List[Dict[str, Any]]:
    catalogs = _build_collection_catalogs(runtime)
    platform_catalog = catalogs["platform"]
    standalone_tools = _build_standalone_platform_tools(platform_catalog)
    collection_tools = [
        {
            "name": collection_name,
            "description": _build_collection_description(collection_name, catalogs[collection_name]),
            "inputSchema": _build_collection_input_schema(catalogs[collection_name]),
        }
        for collection_name in ("platform", "device")
    ]
    if standalone_tools:
        geometry_names = ", ".join(tool["name"] for tool in standalone_tools)
        collection_tools[0]["description"] = (
            f"平台侧功能集合。几何建图相关请优先使用独立 MCP 工具：{geometry_names}。"
            "其余能力通过 function 选择，parameters 传参。\n"
            + collection_tools[0]["description"]
        )
    return standalone_tools + collection_tools


def _resolve_tool_invocation(
    runtime: GatewayRuntime,
    tool_name: str,
    arguments: Dict[str, Any],
) -> Tuple[str, Dict[str, Any]]:
    if not isinstance(arguments, dict):
        raise GatewayError(
            "MCP tool arguments 必须是对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
            details={"tool_name": tool_name},
        )
    if tool_name in _MCP_COLLECTIONS:
        return _resolve_collection_call(runtime, tool_name, arguments)

    target_name = _MCP_STANDALONE_TOOL_TARGETS.get(tool_name)
    candidates = (tool_name,) if target_name is None else (tool_name, target_name)
    catalogs = _build_collection_catalogs(runtime, tool_names=candidates, refresh_availability=False)
    if target_name is not None and target_name in catalogs["platform"]:
        item = catalogs["platform"][target_name]
        schema = (
            item.get("input_schema")
            or _MCP_STANDALONE_TOOL_SCHEMAS.get(tool_name)
            or {"type": "object", "properties": {}, "additionalProperties": False}
        )
        _validate_closed_schema(arguments, schema, path="arguments")
        parameters = arguments
        return target_name, parameters
    for catalog in catalogs.values():
        if tool_name in catalog:
            _validate_closed_schema(
                arguments,
                catalog[tool_name].get("input_schema") or {},
                path="arguments",
            )
            parameters = arguments
            return tool_name, parameters

    raise GatewayError(
        f"未知 MCP 工具: {tool_name}。",
        error_code="capability_not_registered",
        http_status=404,
        jsonrpc_code=-32601,
        details={"tool_name": tool_name},
    )



def _negotiate_protocol_version(params: Dict[str, Any]) -> str:
    requested_version = str(params.get("protocolVersion") or "").strip()
    if not requested_version:
        return _DEFAULT_PROTOCOL_VERSION
    if requested_version in _SUPPORTED_PROTOCOL_VERSIONS:
        return requested_version
    raise GatewayError(
        f"不支持的 MCP 协议版本: {requested_version}。",
        error_code="unsupported_mcp_protocol_version",
        http_status=400,
        jsonrpc_code=-32602,
        details={
            "requested_protocol_version": requested_version,
            "supported_protocol_versions": list(_SUPPORTED_PROTOCOL_VERSIONS),
        },
    )


def _build_initialize_result(runtime: GatewayRuntime, protocol_version: str) -> Dict[str, Any]:
    base_url = runtime.config.gateway.public_base_url.rstrip("/")
    return {
        "protocolVersion": protocol_version,
        "capabilities": {"tools": {"listChanged": False}, "logging": {}},
        "serverInfo": {"name": "nuwax_robot_bridge", "version": "0.9.0"},
        "metadata": {
            "transport": {
                "type": "streamable_http",
                "endpoint": f"{base_url}/mcp",
                "sse_get_supported": True,
            },
            "events": {
                "websocket_url": f"{base_url}/events/ws",
                "sse_url": f"{base_url}/events/stream",
            },
            "streams": {
                "websocket_url": f"{base_url}/streams/ws",
                "sse_url": f"{base_url}/streams/sse",
            },
            "artifacts": {"base_url": f"{base_url}/artifacts"},
            "feedback": {"sse_url": f"{base_url}/api/feedback/stream",
                         "mcp_notification": "notifications/message",
                         "agent_action": "将通知data作为观测加入会话；attention=wake时由宿主唤醒。"},
        },
    }


def _format_sse(event_name: str, payload: Dict[str, Any]) -> str:
    return (
        f"event: {event_name}\n"
        f"data: {json.dumps(payload, ensure_ascii=False, allow_nan=False)}\n\n"
    )


def _resolve_collection_call(runtime: GatewayRuntime, collection_name: str, arguments: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    if collection_name not in _MCP_COLLECTIONS:
        raise GatewayError(
            f"未知 MCP 集合工具: {collection_name}。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    if not isinstance(arguments, dict):
        raise GatewayError(
            "集合工具 arguments 必须是对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    _validate_closed_schema(
        arguments,
        _build_collection_input_schema({}),
        path="arguments",
    )
    function_name = str(arguments.get("function", "")).strip()
    if not function_name:
        raise GatewayError(
            "集合工具调用缺少 function。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    parameters = arguments.get("parameters", {})
    if parameters is None:
        parameters = {}
    if not isinstance(parameters, dict):
        raise GatewayError(
            "集合工具 parameters 必须是对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    catalog = _build_collection_catalogs(runtime, tool_names=(function_name,), refresh_availability=False)[collection_name]
    if function_name not in catalog:
        # 错误诊断列出已公开注册的功能名，不触发执行准入或可用性刷新。
        available_catalog = _build_collection_catalogs(runtime, refresh_availability=False)[collection_name]
        raise GatewayError(
            f"功能 {function_name} 不属于 {collection_name} 集合。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
            details={
                "collection": collection_name,
                "function": function_name,
                "available_functions": sorted(available_catalog),
            },
        )
    _validate_closed_schema(
        parameters,
        catalog[function_name].get("input_schema") or {},
        path="arguments.parameters",
    )
    return function_name, parameters


def build_mcp_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造 MCP JSON-RPC 路由。"""

    router = APIRouter()
    # 只保存连接的日志级别，不拥有任务或模型会话；有界淘汰后客户端重新初始化。
    logging_sessions = OrderedDict()
    log_levels = ("debug", "info", "notice", "warning", "error", "critical", "alert", "emergency")

    def session_for(request):
        identity = request.headers.get("mcp-session-id")
        if identity is None:
            return None
        session = logging_sessions.get(identity)
        if session is None:
            raise GatewayError("MCP连接已过期，请重新初始化。", error_code="mcp_session_expired", http_status=404)
        logging_sessions.move_to_end(identity)
        return session

    @router.get("/mcp")
    async def mcp_stream(request: Request) -> Response:
        session = session_for(request)
        cursor, reason = feedback_cursor(runtime, request.headers.get("last-event-id"))

        async def event_generator():
            async for kind, payload, identity in feedback_events(
                    runtime, request, after_cursor=cursor, reset_reason=reason):
                if kind in {"keepalive", "feedback.ready"}:
                    yield ": feedback connection active\n\n"
                    continue
                if kind == "feedback.gap":
                    level, logger_name = "warning", "harness.feedback"
                    payload = {"event_type": "feedback.gap", "attention": "wake", **payload}
                else:
                    level = payload["severity"]
                    logger_name = payload["source"].get("capability_name") or payload["source"]["component"]
                if kind != "feedback.gap" and log_levels.index(level) < log_levels.index(
                        session["level"] if session else "info"):
                    continue
                yield format_feedback_sse("message", {
                    "jsonrpc": "2.0", "method": "notifications/message",
                    "params": {"level": level, "logger": logger_name, "data": payload},
                }, identity)

        return StreamingResponse(event_generator(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @router.post("/mcp")
    async def mcp_endpoint(request: Request) -> Response:
        content_length = request.headers.get("content-length")
        if content_length:
            try:
                if int(content_length) > MCP_REQUEST_MAX_BYTES:
                    return JSONResponse(
                        _jsonrpc_error(None, code=-32600, message="Request body too large"),
                        status_code=413,
                    )
            except ValueError:
                return JSONResponse(
                    _jsonrpc_error(None, code=-32600, message="Invalid Content-Length"),
                    status_code=400,
                )

        raw = await request.body()
        if len(raw) > MCP_REQUEST_MAX_BYTES:
            return JSONResponse(
                _jsonrpc_error(None, code=-32600, message="Request body too large"),
                status_code=413,
            )
        try:
            body = json.loads(raw, parse_constant=_strict_json_constant)
        except (TypeError, ValueError, json.JSONDecodeError):
            return JSONResponse(_jsonrpc_error(None, code=-32700, message="Parse error"), status_code=400)

        if not isinstance(body, dict):
            return JSONResponse(_jsonrpc_error(None, code=-32600, message="Invalid Request"), status_code=400)

        has_request_id = "id" in body
        request_id = body.get("id")
        response_request_id = request_id
        if has_request_id:
            try:
                request_id = _validated_jsonrpc_id(request_id)
                response_request_id = request_id
            except ValueError:
                return JSONResponse(
                    _jsonrpc_error(None, code=-32600, message="Invalid Request"),
                    status_code=400,
                )
        if body.get("jsonrpc") != "2.0":
            return JSONResponse(
                _jsonrpc_error(response_request_id, code=-32600, message="Invalid Request"),
                status_code=400,
            )
        raw_method = body.get("method")
        if not isinstance(raw_method, str) or not raw_method:
            return JSONResponse(
                _jsonrpc_error(response_request_id, code=-32600, message="Invalid Request"),
                status_code=400,
            )
        method = raw_method
        params = body.get("params", {})
        if params is None:
            params = {}
        http_request_id = str(getattr(request.state, "request_id", None) or "").strip()
        tool_name = ""
        function_name = ""
        tool_call_failure_logged = False

        try:
            if not has_request_id or method not in {"tools/call", "tools/list"}:
                # 通知/会话控制不是停车捷径，保留维护期间的拒绝口径。
                # 它们不调用设备；准入检查在原解析之后、任何会话写入之前。
                await run_in_threadpool(run_mcp_operation, request, lambda: None)
            if not has_request_id:
                return Response(status_code=202)

            if not isinstance(params, dict):
                raise GatewayError(
                    "JSON-RPC params 必须是对象。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )

            if method == "initialize":
                protocol_version = _negotiate_protocol_version(params)
                session_id = uuid4().hex
                logging_sessions[session_id] = {"level": "info"}
                if len(logging_sessions) > 256:
                    logging_sessions.popitem(last=False)
                return JSONResponse(_jsonrpc_result(request_id, _build_initialize_result(runtime, protocol_version)),
                                    headers={"Mcp-Session-Id": session_id})

            if method == "logging/setLevel":
                session = session_for(request)
                if session is None or params.get("level") not in log_levels:
                    raise GatewayError("设置通知级别需要初始化返回的Mcp-Session-Id及有效level。",
                                       error_code="invalid_params", http_status=422, jsonrpc_code=-32602)
                session["level"] = params["level"]
                return JSONResponse(_jsonrpc_result(request_id, {}))

            if method == "tools/list":
                tools = await run_in_threadpool(lambda: _build_collection_tools(runtime))
                response_payload = _jsonrpc_result(request_id, {"tools": tools})
                response = JSONResponse(response_payload)
                if len(response.body) > MCP_RESULT_MAX_BYTES:
                    raise GatewayError(
                        "MCP 工具目录响应超过安全大小限制。",
                        error_code="mcp_response_too_large",
                        http_status=500,
                        jsonrpc_code=-32603,
                        details={"tool_count": len(tools)},
                    )
                return response

            if method == "tools/call":
                name = str(params.get("name", "")).strip()
                tool_name = name
                arguments = params.get("arguments", {})
                if not name:
                    raise GatewayError(
                        "tools/call 缺少 name。",
                        error_code="invalid_params",
                        http_status=422,
                        jsonrpc_code=-32602,
                    )
                if not isinstance(arguments, dict):
                    raise GatewayError(
                        "tools/call arguments 必须是对象。",
                        error_code="invalid_params",
                        http_status=422,
                        jsonrpc_code=-32602,
                        details={"tool_name": name},
                    )
                # 参数模式解析仍在工作线程执行，避免阻塞其它连接的事件循环。
                function_name, function_arguments = await run_in_threadpool(
                    _resolve_tool_invocation,
                    runtime,
                    name,
                    arguments,
                )
                request_context = _build_mcp_request_context(
                    request,
                    rpc_request_id=request_id,
                    method=method,
                    tool_name=function_name,
                )
                started_at = time.perf_counter()
                log_extra = {
                    "event": "mcp.call.started",
                    "request_id": http_request_id,
                    "rpc_request_id": request_id,
                    "mcp_method": method,
                    "tool_name": name,
                    "capability_name": function_name,
                    "requested_by": "local",
                }
                MCP_LOGGER.debug("MCP 工具调用开始", extra=log_extra)
                try:
                    result = await run_in_threadpool(
                        lambda: run_mcp_operation(
                            request,
                            lambda: runtime.invoke_tool(
                                function_name,
                                function_arguments,
                                requested_by="local",
                                request_context=request_context,
                                project_result=False,
                            ),
                            tool_name=function_name,
                        )
                    )
                    structured = result
                except Exception as call_exc:
                    duration_ms = (time.perf_counter() - started_at) * 1000.0
                    call_error = as_gateway_error(call_exc)
                    call_error_category = categorize_gateway_error(
                        call_error.error_code,
                        call_error.http_status,
                    )
                    tool_call_failure_logged = True
                    MCP_LOGGER.exception(
                        "MCP 工具调用失败",
                        extra={
                            **log_extra,
                            "event": "mcp.call.failed",
                            "duration_ms": duration_ms,
                            "error_code": call_error.error_code,
                            "error_category": call_error_category,
                        },
                    )
                    raise
                duration_ms = (time.perf_counter() - started_at) * 1000.0
                completed_extra = {
                    **log_extra,
                    "event": "mcp.call.completed",
                    "duration_ms": duration_ms,
                }
                MCP_LOGGER.log(
                    logging.INFO if duration_ms >= 1_000.0 else logging.DEBUG,
                    "MCP 工具调用完成",
                    extra=completed_extra,
                )
                empty_result_envelope = _jsonrpc_result(request_id, None)
                result_budget = max(
                    256,
                    MCP_RESULT_MAX_BYTES
                    - (_json_bytes(empty_result_envelope) - _json_bytes(None)),
                )
                response_payload = _jsonrpc_result(
                    request_id,
                    await run_in_threadpool(
                        lambda: build_mcp_tool_result(structured, max_total_bytes=result_budget)
                    ),
                )
                response = JSONResponse(response_payload)
                if len(response.body) > MCP_RESULT_MAX_BYTES:
                    raise GatewayError(
                        "MCP 工具结果响应超过安全大小限制。",
                        error_code="mcp_response_too_large",
                        http_status=500,
                        jsonrpc_code=-32603,
                    )
                return response
            raise GatewayError(
                "不支持的 MCP 方法。",
                error_code="unsupported_mcp_method",
                http_status=400,
                jsonrpc_code=-32601,
                details={
                    "supported_methods": ["initialize", "tools/list", "tools/call", "logging/setLevel"],
                },
            )
        except Exception as exc:
            gateway_error = as_gateway_error(exc)
            if gateway_error.error_code == "update_maintenance":
                return JSONResponse(_jsonrpc_error(
                    request_id, code=gateway_error.jsonrpc_code, message=gateway_error.message,
                    data={"code": gateway_error.error_code, "message": gateway_error.message},
                ), status_code=503)
            error_category = categorize_gateway_error(gateway_error.error_code, gateway_error.http_status)
            request.state.error_category = error_category
            if not tool_call_failure_logged:
                rejected_extra = {
                    "event": "mcp.request.rejected",
                    "request_id": http_request_id,
                    "rpc_request_id": request_id,
                    "mcp_method": method,
                    "tool_name": tool_name or None,
                    "capability_name": function_name or None,
                    "error_code": gateway_error.error_code,
                    "error_category": error_category,
                }
                if isinstance(exc, GatewayError):
                    MCP_LOGGER.warning("MCP 请求被拒绝", extra=rejected_extra)
                else:
                    MCP_LOGGER.exception("MCP 请求处理失败", extra=rejected_extra)
            public_error = build_public_error_payload(
                gateway_error,
                request_id=http_request_id or None,
                capability_name=function_name or tool_name or None,
                task_id=str(gateway_error.details.get("task_id") or "").strip() or None,
                error_category=error_category,
            )
            return JSONResponse(
                _jsonrpc_error(
                    request_id,
                    code=gateway_error.jsonrpc_code,
                    message=public_error["message"],
                    data=public_error,
                )
            )

    return router
