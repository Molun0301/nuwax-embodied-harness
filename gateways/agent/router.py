"""公开工具接口；所有业务执行仍进入既有能力与任务主链。"""

from __future__ import annotations

import json

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse

from core.errors import GatewayError, as_gateway_error
from gateways.agent.feedback import INPUT_MAX_BYTES
from gateways.feedback import build_public_error_payload


async def _reply(operation, request: Request):
    try:
        return await run_in_threadpool(operation)
    except Exception as exc:
        error = as_gateway_error(exc)
        return JSONResponse(status_code=error.http_status, content={"error": build_public_error_payload(
            error, request_id=getattr(request.state, "request_id", None))})


def _invalid_constant(value):
    raise ValueError("不允许非有限数")


def build_agent_router(service) -> APIRouter:
    router = APIRouter(prefix="/api/agent")
    # 应用可以延后构造服务；不访问本接口的只读应用无需创建账本或订阅。
    resolve = service if callable(service) else lambda: service

    @router.get("")
    async def discover(request: Request):
        return await _reply(lambda: resolve().discover(), request)

    @router.get("/tools")
    async def tools(request: Request):
        return await _reply(lambda: resolve().list_tools(), request)

    @router.post("/invoke")
    async def invoke(request: Request):
        try:
            chunks = bytearray()
            async for chunk in request.stream():
                chunks.extend(chunk)
                if len(chunks) > INPUT_MAX_BYTES:
                    raise GatewayError("请求体超过大小限制。", error_code="agent_payload_too_large", http_status=413)
            try:
                body = json.loads(chunks, parse_constant=_invalid_constant)
            except (ValueError, UnicodeError, RecursionError) as exc:
                raise GatewayError("请求体必须是有限数值组成的有效 JSON。", error_code="invalid_params", http_status=422) from exc
            if not isinstance(body, dict) or set(body) != {"request_id", "tool_name", "arguments"}:
                raise GatewayError("请求体仅包含 request_id、tool_name、arguments 三个必填字段。",
                                   error_code="invalid_params", http_status=422)
        except Exception as exc:
            error = as_gateway_error(exc)
            return JSONResponse(status_code=error.http_status, content={"error": build_public_error_payload(error)})
        return await _reply(lambda: resolve().invoke(**body), request)

    @router.get("/calls/{request_id}")
    async def call(request_id: str, request: Request):
        return await _reply(lambda: resolve().get_call(request_id), request)

    @router.get("/calls/{request_id}/result")
    async def call_result(request_id: str, request: Request):
        return await _reply(lambda: resolve().get_call_result(request_id), request)

    @router.get("/tasks/{task_id}")
    async def task(task_id: str, request: Request):
        return await _reply(lambda: resolve().get_task(task_id), request)

    @router.get("/tasks/{task_id}/result")
    async def task_result(task_id: str, request: Request):
        return await _reply(lambda: resolve().get_task_result(task_id), request)

    @router.get("/tasks/{task_id}/record")
    async def task_record(task_id: str, request: Request):
        return await _reply(lambda: resolve().get_task_record(task_id), request)

    @router.get("/tasks/{task_id}/wait")
    async def task_wait(task_id: str, request: Request):
        def wait():
            try:
                timeout = float(request.query_params.get("timeout_sec", "30"))
            except ValueError as exc:
                raise GatewayError("timeout_sec 必须是有限数。", error_code="invalid_params", http_status=422) from exc
            return resolve().wait_task(task_id, timeout)
        return await _reply(wait, request)

    @router.post("/tasks/{task_id}/cancel")
    async def task_cancel(task_id: str, request: Request):
        return await _reply(lambda: resolve().cancel_task(task_id), request)

    return router
