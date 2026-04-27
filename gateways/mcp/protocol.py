from __future__ import annotations

from typing import Dict, Optional, Any, List, Tuple
import json

from fastapi import APIRouter
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse
from fastapi.responses import Response

from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError, as_gateway_error
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_json_text, to_jsonable


def _jsonrpc_result(request_id: Any, result: Dict[str, Any]) -> Dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def _jsonrpc_error(request_id: Any, *, code: int, message: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    error: Dict[str, Any] = {"code": code, "message": message}
    if data:
        error["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": error}


_MCP_COLLECTIONS = {
    "platform": {"display_name": "平台侧功能集合", "layers": {"platform"}},
    "device": {"display_name": "端侧设备功能集合", "layers": {"robot_private", "ops_debug"}},
}


def _tool_layer(view: Any) -> str:
    return str(getattr(view.descriptor, "metadata", {}).get("capability_layer") or "platform")


def _tool_catalog_item(runtime: GatewayRuntime, view: Any, layer: str) -> Dict[str, Any]:
    capability_descriptor = runtime.capability_registry.get_descriptor(view.capability_name)
    return {
        "name": view.descriptor.name,
        "display_name": view.descriptor.display_name,
        "description": view.descriptor.description,
        "input_schema": capability_descriptor.input_schema or {"type": "object", "properties": {}, "additionalProperties": False},
        "output_schema": capability_descriptor.output_schema or {"type": "object", "properties": {}, "additionalProperties": False},
        "execution_mode": capability_descriptor.execution_mode.value,
        "risk_level": capability_descriptor.risk_level.value,
        "capability_layer": layer,
    }


def _build_collection_catalogs(runtime: GatewayRuntime) -> Dict[str, Dict[str, Dict[str, Any]]]:
    catalogs: Dict[str, Dict[str, Dict[str, Any]]] = {name: {} for name in _MCP_COLLECTIONS}
    views = runtime.list_tools(exposed_only=None, include_unsupported=False)
    for view in views:
        layer = _tool_layer(view)
        for collection_name, config in _MCP_COLLECTIONS.items():
            if layer in config["layers"]:
                catalogs[collection_name][view.descriptor.name] = _tool_catalog_item(runtime, view, layer)
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


def _build_collection_tools(runtime: GatewayRuntime) -> List[Dict[str, Any]]:
    catalogs = _build_collection_catalogs(runtime)
    return [
        {
            "name": collection_name,
            "description": _build_collection_description(collection_name, catalogs[collection_name]),
            "inputSchema": _build_collection_input_schema(catalogs[collection_name]),
        }
        for collection_name in ("platform", "device")
    ]


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
    catalog = _build_collection_catalogs(runtime)[collection_name]
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
    return function_name, parameters


def build_mcp_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造 MCP JSON-RPC 路由。"""

    router = APIRouter()

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

        request_id = body.get("id")
        method = body.get("method", "")
        params = body.get("params", {})
        if params is None:
            params = {}

        try:
            if not isinstance(params, dict):
                raise GatewayError(
                    "JSON-RPC params 必须是对象。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )

            if method == "initialize":
                return JSONResponse(
                    _jsonrpc_result(
                        request_id,
                        {
                            "protocolVersion": "2025-11-25",
                            "capabilities": {"tools": {"listChanged": False}},
                            "serverInfo": {"name": "nuwax_robot_bridge", "version": "0.9.0"},
                            "metadata": {
                                "events": {
                                    "websocket_url": f"{runtime.config.gateway.public_base_url}/events/ws",
                                    "sse_url": f"{runtime.config.gateway.public_base_url}/events/stream",
                                },
                                "artifacts": {"base_url": f"{runtime.config.gateway.public_base_url}/artifacts"},
                            },
                        },
                    )
                )

            if method == "tools/list":
                return JSONResponse(_jsonrpc_result(request_id, {"tools": _build_collection_tools(runtime)}))

            if method == "tools/call":
                name = str(params.get("name", "")).strip()
                arguments = params.get("arguments", {})
                if arguments is None:
                    arguments = {}
                if not name:
                    raise GatewayError(
                        "tools/call 缺少 name。",
                        error_code="invalid_params",
                        http_status=422,
                        jsonrpc_code=-32602,
                    )
                function_name, function_arguments = _resolve_collection_call(runtime, name, arguments)
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
