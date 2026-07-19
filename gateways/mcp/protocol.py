from __future__ import annotations

import asyncio
from typing import Dict, Optional, Any, List, Tuple
import json

from contracts.skills import SkillDescriptor
from fastapi import APIRouter
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse
from fastapi.responses import Response
from fastapi.responses import StreamingResponse

from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError, as_gateway_error
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_json_text, to_jsonable


def _jsonrpc_result(request_id: Any, result: Dict[str, Any]) -> Dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


_SUPPORTED_PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18")
_DEFAULT_PROTOCOL_VERSION = _SUPPORTED_PROTOCOL_VERSIONS[0]


def _jsonrpc_error(request_id: Any, *, code: int, message: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    error: Dict[str, Any] = {"code": code, "message": message}
    if data:
        error["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": error}


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
            "map_version_id": {"type": "string"},
            "map_revision": {"type": "integer", "minimum": 0},
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
                    "stop_on_map_revision_change": {"type": "boolean"},
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


def _build_collection_catalogs(runtime: GatewayRuntime) -> Dict[str, Dict[str, Dict[str, Any]]]:
    catalogs: Dict[str, Dict[str, Dict[str, Any]]] = {name: {} for name in _MCP_COLLECTIONS}
    refresh_availability = getattr(runtime, "_refresh_dynamic_capability_matrix", None)
    if callable(refresh_availability):
        refresh_availability()
    descriptors = runtime.skill_registry.list_descriptors()
    for descriptor in descriptors:
        try:
            registration = runtime.capability_registry.get_registration(descriptor.capability_name)
        except Exception:
            continue
        if not registration.runnable:
            continue
        get_availability = getattr(runtime.capability_registry, "get_availability", None)
        if callable(get_availability):
            robot_model = getattr(
                getattr(getattr(runtime, "robot", None), "manifest", None),
                "robot_model",
                None,
            )
            try:
                availability = get_availability(
                    descriptor.capability_name,
                    robot_model=robot_model,
                )
            except Exception:
                continue
            if not availability.supported:
                continue
        layer = _tool_layer(descriptor)
        for collection_name, config in _MCP_COLLECTIONS.items():
            if layer in config["layers"]:
                catalogs[collection_name][descriptor.name] = _tool_catalog_item(runtime, descriptor, layer)
                break
    return catalogs


def _build_collection_description(collection_name: str, catalog: Dict[str, Dict[str, Any]]) -> str:
    display_name = _MCP_COLLECTIONS[collection_name]["display_name"]
    if not catalog:
        return f"{display_name}。当前没有可调用功能。"
    lines = [f"{display_name}。通过 function 选择具体功能，通过 parameters 传入该功能参数。", "可用功能："]
    for item in catalog.values():
        lines.append(
            f"- {item['name']}：{item['display_name']}。{item['description']} "
            f"执行模式: {item['execution_mode']}；风险级别: {item['risk_level']}。"
        )
    return "\n".join(lines)


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
                "description": "传给具体功能的参数，结构见 x-nuwax-function-catalog。",
            },
        },
        "required": ["function"],
        "additionalProperties": False,
        "x-nuwax-function-catalog": catalog,
    }


def _build_standalone_platform_tools(platform_catalog: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    tools: List[Dict[str, Any]] = []
    for tool_name in _MCP_STANDALONE_PLATFORM_TOOLS:
        target_name = _MCP_STANDALONE_TOOL_TARGETS[tool_name]
        item = platform_catalog.get(target_name)
        fallback_schema = _MCP_STANDALONE_TOOL_SCHEMAS.get(tool_name)
        # tools/list 与 tools/call 共享同一个 runnable catalog。Schema fallback
        # 只能补充已存在目标的参数描述，不能凭空广告不可调用工具。
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

    catalogs = _build_collection_catalogs(runtime)
    target_name = _MCP_STANDALONE_TOOL_TARGETS.get(tool_name)
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


def _validate_closed_schema(value: Any, schema: Dict[str, Any], *, path: str) -> None:
    """Enforce the closed-object boundary advertised by MCP input schemas."""

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
                _validate_closed_schema(
                    value[key],
                    child_schema,
                    path=f"{path}.{key}",
                )
        return
    if schema_type == "array" and isinstance(value, list):
        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, item in enumerate(value):
                _validate_closed_schema(
                    item,
                    item_schema,
                    path=f"{path}[{index}]",
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
        "capabilities": {"tools": {"listChanged": False}},
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
        },
    }


def _format_sse(event_name: str, payload: Dict[str, Any]) -> str:
    return f"event: {event_name}\ndata: {json.dumps(payload, ensure_ascii=False)}\n\n"


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
    catalog = _build_collection_catalogs(runtime)[collection_name]
    _validate_closed_schema(
        arguments,
        _build_collection_input_schema(catalog),
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
    if function_name not in catalog:
        raise GatewayError(
            f"功能 {function_name} 不属于 {collection_name} 集合。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
            details={
                "collection": collection_name,
                "function": function_name,
                "available_functions": sorted(catalog),
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

    @router.get("/mcp")
    async def mcp_stream(request: Request) -> Response:
        try:
            access_manager.authenticate_http(request)
        except GatewayError as exc:
            return JSONResponse(_jsonrpc_error(None, code=exc.jsonrpc_code, message=exc.message, data=exc.to_payload()), status_code=exc.http_status)

        async def event_generator():
            yield _format_sse("endpoint", {"endpoint": "/mcp", "protocol_versions": list(_SUPPORTED_PROTOCOL_VERSIONS)})
            while not await request.is_disconnected():
                await asyncio.sleep(runtime.config.gateway.sse_keepalive_sec)
                yield _format_sse("keepalive", {"endpoint": "/mcp"})

        return StreamingResponse(event_generator(), media_type="text/event-stream")

    @router.post("/mcp")
    async def mcp_endpoint(request: Request) -> Response:
        try:
            access = access_manager.authenticate_http(request)
        except GatewayError as exc:
            return JSONResponse(_jsonrpc_error(None, code=exc.jsonrpc_code, message=exc.message, data=exc.to_payload()), status_code=exc.http_status)

        raw = await request.body()
        try:
            body = json.loads(raw)
        except Exception:
            return JSONResponse(_jsonrpc_error(None, code=-32700, message="Parse error"), status_code=400)

        if not isinstance(body, dict):
            return JSONResponse(_jsonrpc_error(None, code=-32600, message="Invalid Request"), status_code=400)

        has_request_id = "id" in body
        request_id = body.get("id")
        method = str(body.get("method", ""))
        params = body.get("params", {})
        if params is None:
            params = {}

        try:
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
                return JSONResponse(_jsonrpc_result(request_id, _build_initialize_result(runtime, protocol_version)))

            if method == "tools/list":
                tools = await run_in_threadpool(lambda: _build_collection_tools(runtime))
                return JSONResponse(_jsonrpc_result(request_id, {"tools": tools}))

            if method == "tools/call":
                name = str(params.get("name", "")).strip()
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
                function_name, function_arguments = _resolve_tool_invocation(runtime, name, arguments)
                try:
                    result = await run_in_threadpool(
                        lambda: runtime.invoke_tool(
                            function_name,
                            function_arguments,
                            requested_by=access.principal,
                        )
                    )
                    structured = to_jsonable(result)
                except Exception as exc:
                    raise as_gateway_error(exc) from exc
                return JSONResponse(
                    _jsonrpc_result(
                        request_id,
                        {
                            "content": [{"type": "text", "text": to_json_text(structured)}],
                            "structuredContent": structured,
                        },
                    )
                )
            return JSONResponse(_jsonrpc_error(request_id, code=-32601, message=f"Unknown method: {method}"))
        except Exception as exc:
            gateway_error = as_gateway_error(exc)
            return JSONResponse(
                _jsonrpc_error(
                    request_id,
                    code=gateway_error.jsonrpc_code,
                    message=gateway_error.message,
                    data=gateway_error.to_payload(),
                )
            )

    return router
