from __future__ import annotations

from contextlib import asynccontextmanager
import logging
import time
from typing import Dict, Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from starlette.requests import Request

from drivers.entry import create_active_robot_assembly
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.http.api import build_http_router
from gateways.mcp.protocol import build_mcp_router
from gateways.relay.server import create_relay_app
from gateways.runtime import GatewayRuntime, create_default_gateway_runtime
from gateways.ws.events import build_event_router
from logging_utils import LogRateLimiter
from settings import NuwaxRobotBridgeConfig, configure_logging, load_config


APP_LOGGER = logging.getLogger("nuwax_robot_bridge.gateway")
HTTP_LOGGER = logging.getLogger("nuwax_robot_bridge.http")
_LOW_VALUE_HTTP_PATHS = frozenset(("/api/health", "/events/stream", "/ops"))
_REQUEST_LOG_RATE_LIMIT_SEC = 30.0


def _resolve_client_host(request: Request) -> str:
    forwarded_for = request.headers.get("x-forwarded-for", "").strip()
    if forwarded_for:
        return forwarded_for.split(",", 1)[0].strip()
    if request.client is not None and request.client.host:
        return request.client.host
    return "-"


def _should_rate_limit_http_request(path: str, status_code: int) -> bool:
    if status_code >= 400:
        return False
    return path in _LOW_VALUE_HTTP_PATHS


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
        start_at = time.perf_counter()
        method = request.method
        path = request.url.path
        client_host = _resolve_client_host(request)
        try:
            response = await call_next(request)
        except Exception:
            duration_ms = (time.perf_counter() - start_at) * 1000.0
            HTTP_LOGGER.exception(
                "HTTP 请求异常",
                extra={
                    "event": "http.request.failed",
                    "method": method,
                    "path": path,
                    "client_ip": client_host,
                    "duration_ms": duration_ms,
                },
            )
            raise

        duration_ms = (time.perf_counter() - start_at) * 1000.0
        level = logging.INFO
        if response.status_code >= 500:
            level = logging.ERROR
        elif response.status_code >= 400:
            level = logging.WARNING
        extra = {
            "event": "http.request.completed",
            "method": method,
            "path": path,
            "status_code": response.status_code,
            "client_ip": client_host,
            "duration_ms": duration_ms,
        }
        if _should_rate_limit_http_request(path, response.status_code):
            request_log_limiter.log(
                HTTP_LOGGER,
                level,
                "HTTP 请求完成",
                key=("http_request", method, path, response.status_code),
                interval_sec=_REQUEST_LOG_RATE_LIMIT_SEC,
                extra=extra,
            )
        else:
            HTTP_LOGGER.log(level, "HTTP 请求完成", extra=extra)
        return response

    @app.exception_handler(GatewayError)
    async def _gateway_error_handler(_: Request, exc: GatewayError) -> JSONResponse:
        APP_LOGGER.warning(
            "网关返回业务错误 code=%s status=%s message=%s",
            exc.error_code,
            exc.http_status,
            exc.message,
        )
        return JSONResponse(status_code=exc.http_status, content={"error": exc.to_payload()})

    @app.get("/")
    async def root() -> Dict[str, Any]:
        return {
            "name": "nuwax_robot_bridge_gateway",
            "robot_name": runtime.robot.manifest.robot_name,
            "robot_model": runtime.robot.manifest.robot_model,
            "mcp_path": "/mcp",
            "http_api_base": "/api",
            "ops_path": "/ops",
            "event_paths": {"ws": "/events/ws", "sse": "/events/stream"},
            "artifacts_base": "/artifacts",
        }

    app.include_router(build_http_router(runtime, access_manager))
    app.include_router(build_mcp_router(runtime, access_manager))
    app.include_router(build_event_router(runtime, access_manager))
    return app


def create_gateway_app_from_env() -> FastAPI:
    """按环境配置创建宿主机网关应用。"""

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
