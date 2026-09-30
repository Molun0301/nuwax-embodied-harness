from __future__ import annotations

from contextlib import asynccontextmanager
import logging
import sys
import threading
import time
from typing import Dict, Any

from fastapi import FastAPI
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse
from pathlib import Path
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware
from starlette.requests import Request

from drivers.entry import create_active_robot_assembly
from gateways.auth import GatewayAccessManager
from gateways.agent.router import build_agent_router
from gateways.agent.service import AgentToolService
from gateways.errors import GatewayError
from gateways.feedback import build_public_error_payload
from gateways.request_context import (
    REQUEST_ID_HEADER,
    categorize_gateway_error,
    error_category_from_status,
    normalize_request_id,
)
from gateways.http.api import build_http_router
from gateways.http.console import build_console_router
from gateways.http.logs import build_logs_router
from gateways.mcp.protocol import build_mcp_router
from gateways.relay.server import create_relay_app
from gateways.runtime import GatewayRuntime, create_default_gateway_runtime
from gateways.ws.data_streams import build_data_stream_router
from gateways.ws.events import build_event_router
from core.logging_utils import LogRateLimiter, correlation_context
from core.process_title import set_process_title
from core.settings import NuwaxRobotBridgeConfig, configure_logging, load_config


APP_LOGGER = logging.getLogger("nuwax_robot_bridge.gateway")
HTTP_LOGGER = logging.getLogger("nuwax_robot_bridge.http")
_LOW_VALUE_HTTP_PATHS = frozenset(
    (
        "/api/health",
        "/api/tasks",
        "/api/memory/instances",
        "/mcp",
        "/api/logs",
        "/console",
        "/console/",
        "/events/stream",
        "/streams/sse",
        "/streams/video.mjpeg",
        "/ops",
        "/debug",
        "/debug/",
    )
)
_LOW_VALUE_HTTP_PREFIXES = ("/static/",)
_REQUEST_LOG_RATE_LIMIT_SEC = 30.0
_HTTP_SLOW_REQUEST_THRESHOLD_MS = 1000.0


def _resolve_client_host(request: Request) -> str:
    forwarded_for = request.headers.get("x-forwarded-for", "").strip()
    if forwarded_for:
        return forwarded_for.split(",", 1)[0].strip()
    if request.client is not None and request.client.host:
        return request.client.host
    return "-"


def _should_rate_limit_http_request(path: str, status_code: int, duration_ms: float) -> bool:
    if status_code >= 400:
        return False
    if duration_ms >= _HTTP_SLOW_REQUEST_THRESHOLD_MS:
        return False
    return path in _LOW_VALUE_HTTP_PATHS or any(path.startswith(prefix) for prefix in _LOW_VALUE_HTTP_PREFIXES)


class RequestLoggingMiddleware:
    """直接包装异步网关接口，流式响应体不经过额外任务和内存队列。"""

    def __init__(self, app, *, request_log_limiter):
        self.app = app
        self._request_log_limiter = request_log_limiter

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return
        request = Request(scope, receive)
        request_id_context = normalize_request_id(request.headers.get(REQUEST_ID_HEADER))
        request.state.request_id = request_id_context.request_id
        request.state.request_id_generated = request_id_context.request_id_generated
        request.state.request_id_replaced = request_id_context.request_id_replaced
        request.state.request_id_invalid_reason = request_id_context.request_id_invalid_reason

        with correlation_context(request_id=request_id_context.request_id):
            start_at = time.perf_counter()
            method = request.method
            path = request.url.path
            client_host = _resolve_client_host(request)
            base_extra = {
                "method": method,
                "path": path,
                "client_ip": client_host,
                "request_id": request_id_context.request_id,
                "request_id_generated": request_id_context.request_id_generated,
                "request_id_replaced": request_id_context.request_id_replaced,
            }
            if request_id_context.request_id_invalid_reason:
                base_extra["request_id_invalid_reason"] = request_id_context.request_id_invalid_reason

            response_started = False

            async def send_response(message):
                nonlocal response_started
                if message["type"] == "http.response.start":
                    response_started = True
                    status_code = message["status"]
                    request_header = REQUEST_ID_HEADER.lower().encode("ascii")
                    message["headers"] = [
                        (key, value) for key, value in message.get("headers", [])
                        if key.lower() != request_header
                    ] + [(request_header, request_id_context.request_id.encode("ascii"))]
                    duration_ms = (time.perf_counter() - start_at) * 1000.0
                    waiting_for_video = (
                        path == "/streams/video.mjpeg"
                        and status_code == 503
                        and getattr(request.state, "error_category", None) == "not_ready"
                    )
                    level = logging.INFO
                    if waiting_for_video:
                        level = logging.DEBUG
                    elif status_code >= 500:
                        level = logging.ERROR
                    elif status_code >= 400:
                        level = logging.WARNING
                    extra = {
                        **base_extra,
                        "event": "http.request.completed",
                        "status_code": status_code,
                        "duration_ms": duration_ms,
                        "error_category": getattr(request.state, "error_category", None)
                        or error_category_from_status(status_code),
                    }
                    if duration_ms >= _HTTP_SLOW_REQUEST_THRESHOLD_MS:
                        if level < logging.WARNING:
                            level = logging.WARNING
                        extra["slow_request"] = True
                        extra["slow_request_threshold_ms"] = _HTTP_SLOW_REQUEST_THRESHOLD_MS
                    if (
                        waiting_for_video and duration_ms < _HTTP_SLOW_REQUEST_THRESHOLD_MS
                    ) or _should_rate_limit_http_request(path, status_code, duration_ms):
                        extra["low_value_request"] = True
                        extra["slow_request_threshold_ms"] = _HTTP_SLOW_REQUEST_THRESHOLD_MS
                        self._request_log_limiter.log(
                            HTTP_LOGGER,
                            logging.DEBUG,
                            "HTTP 请求完成",
                            key=("http_request", method, path, status_code),
                            interval_sec=_REQUEST_LOG_RATE_LIMIT_SEC,
                            extra=extra,
                        )
                    else:
                        HTTP_LOGGER.log(level, "HTTP 请求完成", extra=extra)
                await send(message)

            try:
                await self.app(scope, receive, send_response)
            except Exception:
                if response_started:
                    raise
                duration_ms = (time.perf_counter() - start_at) * 1000.0
                error_category = getattr(request.state, "error_category", None) or "internal"
                failed_extra = {
                    **base_extra,
                    "event": "http.request.failed",
                    "status_code": 500,
                    "duration_ms": duration_ms,
                    "error_category": error_category,
                }
                HTTP_LOGGER.exception("HTTP 请求异常", extra=failed_extra)
                response = JSONResponse(
                    status_code=500,
                    content={
                        "error": {
                            "code": "internal_error",
                            "message": "网关内部错误。",
                            "request_id": request_id_context.request_id,
                            "error_category": error_category,
                            "retryable": False,
                            "next_action": "使用 request_id 在详细日志中定位根因，修正条件后再调用。",
                        }
                    },
                    headers={REQUEST_ID_HEADER: request_id_context.request_id},
                )
                await response(scope, receive, send)


def create_gateway_app(
    runtime: GatewayRuntime,
    config: NuwaxRobotBridgeConfig,
    *,
    start_runtime_on_lifespan: bool = True,
) -> FastAPI:
    """创建宿主机网关应用。"""

    access_manager = GatewayAccessManager(config.gateway)
    agent_tools = None
    agent_tools_lock = threading.Lock()
    request_log_limiter = LogRateLimiter()

    def get_agent_tools():
        nonlocal agent_tools
        with agent_tools_lock:
            if agent_tools is None:
                agent_tools = AgentToolService(runtime, access_manager)
                app.state.agent_tools = agent_tools
            return agent_tools

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        try:
            if start_runtime_on_lifespan:
                await run_in_threadpool(get_agent_tools)
                APP_LOGGER.info("宿主机网关运行时准备启动。")
                await run_in_threadpool(runtime.start)
            yield
        finally:
            try:
                if start_runtime_on_lifespan:
                    APP_LOGGER.info("宿主机网关运行时准备停止。")
                    await run_in_threadpool(runtime.stop)
            finally:
                if agent_tools is not None:
                    agent_tools.close()

    app = FastAPI(title="Nuwax Embodied Harness（NEH）网关", lifespan=lifespan)
    app.state.runtime = runtime
    app.state.config = config
    app.state.access_manager = access_manager
    app.state.agent_tools = agent_tools

    app.add_middleware(RequestLoggingMiddleware, request_log_limiter=request_log_limiter)

    @app.exception_handler(GatewayError)
    async def _gateway_error_handler(request: Request, exc: GatewayError) -> JSONResponse:
        request_id = getattr(request.state, "request_id", None) or normalize_request_id(None).request_id
        error_category = categorize_gateway_error(exc.error_code, exc.http_status)
        request.state.error_category = error_category
        payload = build_public_error_payload(
            exc,
            request_id=request_id,
            error_category=error_category,
        )
        log_extra = {
            "event": "gateway.error",
            "request_id": request_id,
            "error_code": exc.error_code,
            "error_category": error_category,
            "status_code": exc.http_status,
        }
        if exc.http_status >= 500:
            APP_LOGGER.error(
                "网关返回服务端错误 code=%s status=%s message=%s",
                exc.error_code,
                exc.http_status,
                exc.message,
                exc_info=(type(exc), exc, exc.__traceback__),
                extra=log_extra,
            )
        else:
            APP_LOGGER.warning(
                "网关返回业务错误 code=%s status=%s message=%s",
                exc.error_code,
                exc.http_status,
                exc.message,
                extra=log_extra,
            )
        return JSONResponse(
            status_code=exc.http_status,
            content={"error": payload},
            headers={REQUEST_ID_HEADER: request_id},
        )

    @app.get("/")
    async def root() -> Dict[str, Any]:
        return {
            "name": "nuwax_robot_bridge_gateway",
            "robot_name": runtime.robot.manifest.robot_name,
            "robot_model": runtime.robot.manifest.robot_model,
            "mcp_path": "/mcp",
            "http_api_base": "/api",
            "ops_path": "/ops",
            "console_path": "/console",
            "event_paths": {"ws": "/events/ws", "sse": "/events/stream"},
            "stream_paths": {"ws": "/streams/ws", "sse": "/streams/sse"},
            "artifacts_base": "/artifacts",
        }

    app.include_router(build_http_router(runtime, access_manager))
    app.include_router(build_agent_router(get_agent_tools))
    from gateways.feedback_stream import build_feedback_router
    app.include_router(build_feedback_router(runtime))
    app.include_router(build_console_router(runtime))
    app.include_router(build_logs_router(runtime, access_manager))
    app.include_router(build_mcp_router(runtime, access_manager))
    app.include_router(build_event_router(runtime, access_manager))
    app.include_router(build_data_stream_router(runtime, access_manager))
    static_dir = Path(__file__).resolve().parent / "http" / "static"
    if static_dir.is_dir():
        # 压缩仅作用于页面静态资源，避免缓冲视频和实时数据流。
        app.mount(
            "/static",
            GZipMiddleware(StaticFiles(directory=str(static_dir)), minimum_size=1024, compresslevel=1),
            name="static",
        )
    return app


def create_gateway_app_from_env() -> FastAPI:
    """按环境配置创建宿主机网关应用。"""

    # 网关同时承担高频采集和网络交付；短时间片避免计算线程反复抢占刚唤醒的网页线程。
    sys.setswitchinterval(0.001)
    set_process_title("nuwax-gateway")
    config = load_config()
    configure_logging(config.logging)
    APP_LOGGER.info(
        "日志系统已初始化 log_file=%s level=%s",
        config.logging.log_file,
        config.logging.level,
    )
    robot = create_active_robot_assembly(config)
    runtime = create_default_gateway_runtime(config, robot)
    APP_LOGGER.info(
        "宿主机网关应用已创建 robot=%s model=%s artifact_dir=%s memory_db=%s",
        robot.manifest.robot_name,
        robot.manifest.robot_model,
        config.gateway.artifact_dir,
        config.runtime_data.memory_db_path,
    )
    return create_gateway_app(runtime, config)


def create_relay_app_from_env() -> FastAPI:
    """按环境配置创建容器内边车转发器。"""

    config = load_config()
    configure_logging(config.logging)
    APP_LOGGER.info(
        "容器 relay 应用已创建 upstream=%s listen=%s:%s",
        config.relay.upstream_base_url,
        config.relay.host,
        config.relay.port,
    )
    return create_relay_app(config.relay)
