from __future__ import annotations

import logging
import time
from typing import Dict, Optional, Any

from fastapi import FastAPI
from fastapi import HTTPException
from fastapi import Request
from fastapi.responses import JSONResponse
from fastapi.responses import Response
from fastapi.responses import StreamingResponse
import httpx

from logging_utils import LogRateLimiter
from gateways.request_context import REQUEST_ID_HEADER, error_category_from_status, normalize_request_id
from settings import RelayServerConfig


RELAY_LOGGER = logging.getLogger("nuwax_robot_bridge.relay")
_LOW_VALUE_RELAY_PATHS = frozenset(("/relay/health", "/events/stream"))
_RELAY_REQUEST_LOG_RATE_LIMIT_SEC = 30.0


def _resolve_client_host(request: Request) -> str:
    forwarded_for = request.headers.get("x-forwarded-for", "").strip()
    if forwarded_for:
        return forwarded_for.split(",", 1)[0].strip()
    if request.client is not None and request.client.host:
        return request.client.host
    return "-"


def _should_rate_limit_relay_request(path: str, status_code: int) -> bool:
    if status_code >= 400:
        return False
    return path in _LOW_VALUE_RELAY_PATHS


def create_relay_app(
    relay_config: RelayServerConfig,
    *,
    transport: Optional[httpx.AsyncBaseTransport] = None,
) -> FastAPI:
    """创建容器内边车转发器。"""

    app = FastAPI(title="nuwax_robot_bridge_relay")
    app.state.relay_config = relay_config
    app.state.http_transport = transport
    request_log_limiter = LogRateLimiter()

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
                "event": "relay.request.failed",
                "status_code": 500,
                "duration_ms": duration_ms,
                "error_category": error_category,
            }
            upstream_status_code = getattr(request.state, "upstream_status_code", None)
            upstream_duration_ms = getattr(request.state, "upstream_duration_ms", None)
            if upstream_status_code is not None:
                failed_extra["upstream_status_code"] = upstream_status_code
            if upstream_duration_ms is not None:
                failed_extra["upstream_duration_ms"] = upstream_duration_ms
            RELAY_LOGGER.exception("Relay 请求异常", extra=failed_extra)
            return JSONResponse(
                status_code=500,
                content={
                    "error": {
                        "code": "relay_internal_error",
                        "message": "Relay 内部错误。",
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
            "event": "relay.request.completed",
            "status_code": response.status_code,
            "duration_ms": duration_ms,
            "error_category": getattr(request.state, "error_category", None)
            or error_category_from_status(response.status_code),
        }
        upstream_status_code = getattr(request.state, "upstream_status_code", None)
        upstream_duration_ms = getattr(request.state, "upstream_duration_ms", None)
        if upstream_status_code is not None:
            extra["upstream_status_code"] = upstream_status_code
        if upstream_duration_ms is not None:
            extra["upstream_duration_ms"] = upstream_duration_ms
        if _should_rate_limit_relay_request(path, response.status_code):
            request_log_limiter.log(
                RELAY_LOGGER,
                level,
                "Relay 请求完成",
                key=("relay_request", method, path, response.status_code),
                interval_sec=_RELAY_REQUEST_LOG_RATE_LIMIT_SEC,
                extra=extra,
            )
        else:
            RELAY_LOGGER.log(level, "Relay 请求完成", extra=extra)
        return response

    @app.get("/relay/health")
    async def relay_health() -> Dict[str, Any]:
        return {
            "relay_enabled": relay_config.enabled,
            "upstream_base_url": relay_config.upstream_base_url,
            "supports": ["mcp", "http", "sse", "artifacts"],
        }

    @app.api_route("/mcp", methods=["POST"])
    async def relay_mcp(request: Request) -> Response:
        return await _forward_request(app, request, "/mcp")

    @app.api_route("/api/{path:path}", methods=["GET", "POST"])
    async def relay_api(path: str, request: Request) -> Response:
        return await _forward_request(app, request, f"/api/{path}")

    @app.get("/artifacts/{path:path}")
    async def relay_artifacts(path: str, request: Request) -> Response:
        return await _forward_request(app, request, f"/artifacts/{path}")

    @app.get("/events/stream")
    async def relay_sse(request: Request) -> Response:
        return await _forward_request(app, request, "/events/stream", stream=True)

    return app


async def _forward_request(
    app: FastAPI,
    request: Request,
    path: str,
    *,
    stream: bool = False,
) -> Response:
    relay_config: RelayServerConfig = app.state.relay_config
    _assert_incoming_token(request, relay_config)
    upstream_url = f"{relay_config.upstream_base_url.rstrip('/')}{path}"
    headers = _build_upstream_headers(request, relay_config.upstream_token)
    content = await request.body()
    upstream_start_at = time.perf_counter()

    try:
        async with httpx.AsyncClient(transport=app.state.http_transport, timeout=None) as client:
            if stream:
                upstream_request = client.build_request(
                    request.method,
                    upstream_url,
                    headers=headers,
                    params=request.query_params,
                    content=content,
                )
                upstream_response = await client.send(upstream_request, stream=True)
                _record_upstream_observation(request, upstream_response.status_code, upstream_start_at)

                async def _iter_bytes():
                    try:
                        async with upstream_response:
                            async for chunk in upstream_response.aiter_raw():
                                yield chunk
                    except Exception:
                        request.state.error_category = "upstream"
                        RELAY_LOGGER.exception(
                            "Relay 流式转发迭代失败",
                            extra={
                                "event": "relay.upstream.stream_failed",
                                "request_id": getattr(request.state, "request_id", "-"),
                                "error_category": "upstream",
                                "upstream_url": upstream_url,
                                "upstream_status_code": getattr(request.state, "upstream_status_code", upstream_response.status_code),
                                "upstream_duration_ms": getattr(request.state, "upstream_duration_ms", None),
                                "stream": True,
                            },
                        )
                        raise

                return StreamingResponse(
                    _iter_bytes(),
                    status_code=upstream_response.status_code,
                    headers=_filter_response_headers(upstream_response.headers),
                    media_type=upstream_response.headers.get("content-type"),
                )

            upstream_response = await client.request(
                request.method,
                upstream_url,
                headers=headers,
                params=request.query_params,
                content=content,
            )
            _record_upstream_observation(request, upstream_response.status_code, upstream_start_at)
            return Response(
                content=upstream_response.content,
                status_code=upstream_response.status_code,
                headers=_filter_response_headers(upstream_response.headers),
                media_type=upstream_response.headers.get("content-type"),
            )
    except httpx.HTTPError as exc:
        _record_upstream_observation(request, None, upstream_start_at)
        request.state.error_category = "upstream"
        RELAY_LOGGER.exception(
            "Relay 转发失败",
            extra={
                "event": "relay.upstream.failed",
                "request_id": getattr(request.state, "request_id", "-"),
                "error_category": "upstream",
                "upstream_url": upstream_url,
                "upstream_status_code": getattr(request.state, "upstream_status_code", None),
                "upstream_duration_ms": getattr(request.state, "upstream_duration_ms", None),
                "stream": stream,
            },
        )
        raise HTTPException(status_code=502, detail=f"relay upstream error: {exc}") from exc


def _assert_incoming_token(request: Request, relay_config: RelayServerConfig) -> None:
    auth_header = request.headers.get("authorization", "").strip()
    token = ""
    if auth_header.lower().startswith("bearer "):
        token = auth_header[7:].strip()
    if not token:
        token = request.query_params.get("access_token", "").strip()
    if token and token in relay_config.incoming_tokens:
        return
    request.state.error_category = "auth"
    raise HTTPException(status_code=401, detail="relay unauthorized")


def _build_upstream_headers(request: Request, upstream_token: str) -> Dict[str, str]:
    headers: Dict[str, str] = {}
    for key, value in request.headers.items():
        normalized = key.lower()
        if normalized in {"host", "content-length", "authorization", "x-request-id"}:
            continue
        headers[key] = value
    request_id = getattr(request.state, "request_id", None)
    if request_id:
        headers[REQUEST_ID_HEADER] = request_id
    if upstream_token:
        headers["Authorization"] = f"Bearer {upstream_token}"
    return headers


def _record_upstream_observation(request: Request, status_code: Optional[int], start_at: float) -> None:
    request.state.upstream_duration_ms = (time.perf_counter() - start_at) * 1000.0
    if status_code is not None:
        request.state.upstream_status_code = status_code
    if status_code is None or status_code >= 500:
        request.state.error_category = "upstream"


def _filter_response_headers(headers: httpx.Headers) -> Dict[str, str]:
    blocked = {"content-length", "connection", "transfer-encoding", "keep-alive", "x-request-id"}
    return {key: value for key, value in headers.items() if key.lower() not in blocked}
