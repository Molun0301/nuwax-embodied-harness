"""公开工具接口；所有业务执行仍进入既有能力与任务主链。"""

from __future__ import annotations

import asyncio
import json

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse, Response

from core.errors import GatewayError, as_gateway_error
from gateways.agent.feedback import INPUT_MAX_BYTES
from gateways.feedback import build_public_error_payload


async def _reply(operation, request: Request):
    try:
        return await run_in_threadpool(operation)
    except Exception as exc:
        return _error_reply(exc, request)


def _error_reply(exc, request: Request):
    error = as_gateway_error(exc)
    return JSONResponse(status_code=error.http_status, content={"error": build_public_error_payload(
        error, request_id=getattr(request.state, "request_id", None))})


async def _wait_for_disconnect(request: Request) -> None:
    # 此 GET 路由没有请求体参数，也没有其他 receive/stream 消费者。单一接收者
    # 丢弃可选请求体块，不缓冲、不使用会消费请求消息的 is_disconnected 轮询。
    while True:
        if (await request.receive())["type"] == "http.disconnect":
            return


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
        try:
            try:
                timeout = float(request.query_params.get("timeout_sec", "30"))
            except ValueError as exc:
                raise GatewayError("timeout_sec 必须是有限数。", error_code="invalid_params", http_status=422) from exc

            async def wait():
                # 延迟构造可能打开 SQLite，仍需在线程中完成。
                active_service = await run_in_threadpool(resolve)
                return await active_service.wait_task_async(task_id, timeout)

            waiting = asyncio.create_task(wait())
            disconnected = asyncio.create_task(_wait_for_disconnect(request))
            try:
                done, _ = await asyncio.wait((waiting, disconnected), return_when=asyncio.FIRST_COMPLETED)
                if disconnected in done:
                    disconnected.result()
                    # 客户端已离开；记录为中止请求，不将其误报为机器人任务取消或 500。
                    return Response(status_code=499)
                return await waiting
            finally:
                waiting.cancel()
                disconnected.cancel()
                await asyncio.gather(waiting, disconnected, return_exceptions=True)
        except Exception as exc:  # noqa: BLE001 - async public error boundary, matching _reply
            return _error_reply(exc, request)

    @router.post("/tasks/{task_id}/cancel")
    async def task_cancel(task_id: str, request: Request):
        return await _reply(lambda: resolve().cancel_task(task_id), request)

    return router
