"""MCP（模型上下文协议）传输适配，业务调用统一进入 invoke。"""
from __future__ import annotations

import json
import logging
from collections import OrderedDict
from uuid import uuid4

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse, Response, StreamingResponse

from core.errors import GatewayError, as_gateway_error
from gateways.arguments import strict_json_constant
from gateways.feedback import build_mcp_tool_result, build_public_error_payload
from gateways.feedback_stream import feedback_cursor, feedback_events, format_feedback_sse
from gateways.invoke import invoke

LOGGER = logging.getLogger("nuwax_robot_bridge.gateway.mcp")
_SUPPORTED_PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18")


def _jsonrpc_result(request_id, result):
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def _jsonrpc_error(request_id, *, code, message, data=None):
    error = {"code": code, "message": message}
    if data is not None:
        error["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": error}


def _error_code(error):
    if error.error_code in {"unknown_capability", "unsupported_mcp_method"}:
        return -32601
    if error.error_code == "invalid_params":
        return -32602
    return -32603


def _build_tools(platform):
    return [{"name": cap.name, "description": cap.description, "inputSchema": cap.input_schema}
            for cap in platform.capabilities.agent_tools(platform.robot.manifest.robot_name)]


def _build_initialize_result(platform, version):
    base = platform.config.gateway.public_base_url.rstrip("/")
    return {"protocolVersion": version,
            "capabilities": {"tools": {"listChanged": False}, "logging": {}},
            "serverInfo": {"name": "nuwax_robot_bridge", "version": "0.9.0"},
            "metadata": {"transport": {"type": "streamable_http", "endpoint": base + "/mcp",
                                        "sse_get_supported": True},
                         "events": {"websocket_url": base + "/events/ws", "sse_url": base + "/events/stream"},
                         "streams": {"websocket_url": base + "/streams/ws", "sse_url": base + "/streams/sse"},
                         "artifacts": {"base_url": base + "/artifacts"},
                         "feedback": {"sse_url": base + "/api/feedback/stream",
                                      "mcp_notification": "notifications/message"}}}


def build_mcp_router(platform, access_manager=None) -> APIRouter:
    router = APIRouter()
    sessions = OrderedDict()
    levels = ("debug", "info", "notice", "warning", "error", "critical", "alert", "emergency")

    def session_for(request):
        identity = request.headers.get("mcp-session-id")
        if identity is None:
            return None
        session = sessions.get(identity)
        if session is None:
            raise GatewayError("MCP 连接已过期，请重新初始化。", error_code="mcp_session_expired", http_status=404)
        sessions.move_to_end(identity)
        return session

    @router.get("/mcp")
    async def mcp_stream(request: Request):
        session = session_for(request)
        cursor, reason = feedback_cursor(platform, request.headers.get("last-event-id"))

        async def generate():
            async for kind, payload, identity in feedback_events(platform, request, after_cursor=cursor, reset_reason=reason):
                if kind in {"keepalive", "feedback.ready"}:
                    yield ": feedback connection active\n\n"
                    continue
                if kind == "feedback.gap":
                    level, logger = "warning", "harness.feedback"
                    payload = {"event_type": kind, "attention": "wake", **payload}
                else:
                    level = payload["severity"]
                    logger = payload["source"].get("capability_name") or payload["source"]["component"]
                if levels.index(level) < levels.index(session["level"] if session else "info"):
                    continue
                yield format_feedback_sse("message", {"jsonrpc": "2.0", "method": "notifications/message",
                    "params": {"level": level, "logger": logger, "data": payload}}, identity)

        return StreamingResponse(generate(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @router.post("/mcp")
    async def endpoint(request: Request):
        try:
            body = json.loads(await request.body(), parse_constant=strict_json_constant)
        except (ValueError, UnicodeError):
            return JSONResponse(_jsonrpc_error(None, code=-32700, message="Parse error"), status_code=400)
        if (not isinstance(body, dict) or body.get("jsonrpc") != "2.0"
                or not isinstance(body.get("method"), str)):
            return JSONResponse(_jsonrpc_error(None, code=-32600, message="Invalid Request"), status_code=400)
        request_id = body.get("id")
        if isinstance(request_id, bool) or not isinstance(request_id, (str, int, type(None))):
            return JSONResponse(_jsonrpc_error(None, code=-32600, message="Invalid Request"), status_code=400)
        if "id" not in body:
            return Response(status_code=202)
        method, params = body["method"], body.get("params", {})
        calling = False
        try:
            if not isinstance(params, dict):
                raise GatewayError("params 必须是对象。", error_code="invalid_params", http_status=400)
            if method == "ping":
                result = {}
            elif method == "initialize":
                version = params.get("protocolVersion")
                if version not in _SUPPORTED_PROTOCOL_VERSIONS:
                    version = _SUPPORTED_PROTOCOL_VERSIONS[0]
                identity = uuid4().hex
                sessions[identity] = {"level": "info"}
                if len(sessions) > 256:
                    sessions.popitem(last=False)
                return JSONResponse(_jsonrpc_result(request_id, _build_initialize_result(platform, version)),
                                    headers={"Mcp-Session-Id": identity})
            elif method == "logging/setLevel":
                session = session_for(request)
                if session is None or params.get("level") not in levels:
                    raise GatewayError("请提供有效通知级别与初始化返回的会话。", error_code="invalid_params", http_status=400)
                session["level"] = params["level"]
                result = {}
            elif method == "tools/list":
                result = {"tools": _build_tools(platform)}
            elif method == "tools/call":
                name = params.get("name")
                if not isinstance(name, str):
                    raise GatewayError("tools/call 缺少工具名。", error_code="invalid_params", http_status=400)
                if name not in {cap.name for cap in platform.capabilities.agent_tools(platform.robot.manifest.robot_name)}:
                    raise GatewayError("未知 MCP 工具。", error_code="unknown_capability", http_status=404)
                calling = True
                result = build_mcp_tool_result(await run_in_threadpool(
                    invoke, platform, name, params.get("arguments", {}),
                    "mcp:" + (request.headers.get("mcp-session-id") or "client")))
            else:
                raise GatewayError("不支持的 MCP 方法。", error_code="unsupported_mcp_method", http_status=400)
            return JSONResponse(_jsonrpc_result(request_id, result))
        except Exception as exc:
            error = as_gateway_error(exc)
            if not isinstance(exc, GatewayError):
                LOGGER.exception("MCP 请求处理失败")
            public = build_public_error_payload(error)
            if calling and error.error_code != "unknown_capability":
                result = build_mcp_tool_result({"error": public})
                return JSONResponse(_jsonrpc_result(request_id, result))
            return JSONResponse(_jsonrpc_error(request_id, code=_error_code(error), message=error.message, data=public))

    return router
