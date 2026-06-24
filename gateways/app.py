from __future__ import annotations

from contextlib import asynccontextmanager
import logging
import time
from typing import Dict, Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pathlib import Path
from fastapi.staticfiles import StaticFiles
from starlette.requests import Request

from drivers.entry import create_active_robot_assembly
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.request_context import (
    REQUEST_ID_HEADER,
    categorize_gateway_error,
    error_category_from_status,
    normalize_request_id,
)
from gateways.http.api import build_http_router
from gateways.http.debug_panel import build_debug_panel_router
from gateways.mcp.protocol import build_mcp_router
from gateways.relay.server import create_relay_app
from gateways.runtime import GatewayRuntime, create_default_gateway_runtime
from gateways.ws.data_streams import build_data_stream_router
from gateways.ws.events import build_event_router
from core.logging_utils import LogRateLimiter
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
        "/events/stream",
        "/streams/sse",
        "/streams/video.mjpeg",
        "/ops",
        "/debug",
        "/debug/",
    )
)
_LOW_VALUE_HTTP_PREFIXES = ("/api/debug/", "/static/")
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


def create_gateway_app(
    runtime: GatewayRuntime,
    config: NuwaxRobotBridgeConfig,
    *,
    start_runtime_on_lifespan: bool = True,
) -> FastAPI:
    """创建宿主机网关应用。"""

    access_manager = GatewayAccessManager(config.gateway)
    request_log_limiter = LogRateLimiter()

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        if start_runtime_on_lifespan:
            APP_LOGGER.info("宿主机网关运行时准备启动。")
            runtime.start()
        try:
            yield
        finally:
            if start_runtime_on_lifespan:
                APP_LOGGER.info("宿主机网关运行时准备停止。")
                runtime.stop()

    app = FastAPI(title="nuwax_robot_bridge_gateway", lifespan=lifespan)
    app.state.runtime = runtime
    app.state.config = config
    app.state.access_manager = access_manager

    @app.middleware("http")
    async def _request_logging_middleware(request: Request, call_next):
        request_id_context = normalize_request_id(request.headers.get(REQUEST_ID_HEADER))
        request.state.request_id = request_id_context.request_id
        request.state.request_id_generated = request_id_context.request_id_generated
        request.state.request_id_replaced = request_id_context.request_id_replaced
        request.state.request_id_invalid_reason = request_id_context.request_id_invalid_reason

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

        try:
            response = await call_next(request)
        except Exception:
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
            return JSONResponse(
                status_code=500,
                content={
                    "error": {
                        "code": "internal_error",
                        "message": "网关内部错误。",
                        "request_id": request_id_context.request_id,
                        "error_category": error_category,
                    }
                },
                headers={REQUEST_ID_HEADER: request_id_context.request_id},
            )

        response.headers[REQUEST_ID_HEADER] = request_id_context.request_id
        duration_ms = (time.perf_counter() - start_at) * 1000.0
        level = logging.INFO
        if response.status_code >= 500:
            level = logging.ERROR
        elif response.status_code >= 400:
            level = logging.WARNING
        extra = {
            **base_extra,
            "event": "http.request.completed",
            "status_code": response.status_code,
            "duration_ms": duration_ms,
            "error_category": getattr(request.state, "error_category", None)
            or error_category_from_status(response.status_code),
        }
        if duration_ms >= _HTTP_SLOW_REQUEST_THRESHOLD_MS:
            if level < logging.WARNING:
                level = logging.WARNING
            extra["slow_request"] = True
            extra["slow_request_threshold_ms"] = _HTTP_SLOW_REQUEST_THRESHOLD_MS
        if _should_rate_limit_http_request(path, response.status_code, duration_ms):
            extra["low_value_request"] = True
            extra["slow_request_threshold_ms"] = _HTTP_SLOW_REQUEST_THRESHOLD_MS
            request_log_limiter.log(
                HTTP_LOGGER,
                logging.DEBUG,
                "HTTP 请求完成",
                key=("http_request", method, path, response.status_code),
                interval_sec=_REQUEST_LOG_RATE_LIMIT_SEC,
                extra=extra,
            )
        else:
            HTTP_LOGGER.log(level, "HTTP 请求完成", extra=extra)
        return response

    @app.exception_handler(GatewayError)
    async def _gateway_error_handler(request: Request, exc: GatewayError) -> JSONResponse:
        request_id = getattr(request.state, "request_id", None) or normalize_request_id(None).request_id
        error_category = categorize_gateway_error(exc.error_code, exc.http_status)
        request.state.error_category = error_category
        payload = exc.to_payload()
        payload["request_id"] = request_id
        payload["error_category"] = error_category
        APP_LOGGER.warning(
            "网关返回业务错误 code=%s status=%s message=%s",
            exc.error_code,
            exc.http_status,
            exc.message,
            extra={
                "event": "gateway.error",
                "request_id": request_id,
                "error_code": exc.error_code,
                "error_category": error_category,
                "status_code": exc.http_status,
            },
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
            "debug_panel_path": "/debug",
            "event_paths": {"ws": "/events/ws", "sse": "/events/stream"},
            "stream_paths": {"ws": "/streams/ws", "sse": "/streams/sse"},
            "artifacts_base": "/artifacts",
        }

    app.include_router(build_http_router(runtime, access_manager))
    app.include_router(build_debug_panel_router(runtime, access_manager))
    app.include_router(build_mcp_router(runtime, access_manager))
    app.include_router(build_event_router(runtime, access_manager))
    app.include_router(build_data_stream_router(runtime, access_manager))
    static_dir = Path(__file__).resolve().parent / "http" / "static"
    if static_dir.is_dir():
        app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    return app


def create_gateway_app_from_env() -> FastAPI:
    """按环境配置创建宿主机网关应用。"""

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
