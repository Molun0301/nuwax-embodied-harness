"""智能体 HTTP（超文本传输协议）入口，只编解码工具调用和任务状态。"""
from __future__ import annotations

import asyncio

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool

from core.errors import GatewayError


def build_agent_router(service) -> APIRouter:
    router = APIRouter(prefix="/api/agent")
    resolve = service if callable(service) else lambda: service

    @router.get("")
    async def discover():
        return await run_in_threadpool(lambda: resolve().discover())

    @router.get("/tools")
    async def tools():
        return await run_in_threadpool(lambda: resolve().list_tools())

    @router.post("/invoke")
    async def invoke(request: Request):
        body = await request.json()
        if not isinstance(body, dict) or not isinstance(body.get("tool_name"), str):
            raise GatewayError("调用请求必须包含 tool_name 与 arguments。", error_code="invalid_params", http_status=400)
        return await run_in_threadpool(lambda: resolve().invoke(body["tool_name"], body.get("arguments", {})))

    @router.get("/tasks")
    async def tasks(limit: int = 50):
        return await run_in_threadpool(lambda: resolve().list_tasks(limit))

    @router.get("/tasks/{task_id}")
    @router.get("/tasks/{task_id}/result")
    @router.get("/tasks/{task_id}/record")
    async def task(task_id: str):
        return await run_in_threadpool(lambda: resolve().get_task(task_id))

    @router.get("/tasks/{task_id}/wait")
    async def task_wait(task_id: str, request: Request, timeout_sec: float = 30):
        active_service = resolve()
        waiting = asyncio.create_task(active_service.wait_task_async(task_id, timeout_sec))

        async def disconnected():
            while (await request.receive())["type"] != "http.disconnect":
                pass

        disconnect = asyncio.create_task(disconnected())
        try:
            done, _ = await asyncio.wait((waiting, disconnect), return_when=asyncio.FIRST_COMPLETED)
            if waiting in done:
                return waiting.result()
        finally:
            waiting.cancel()
            disconnect.cancel()
            await asyncio.gather(waiting, disconnect, return_exceptions=True)

    @router.post("/tasks/{task_id}/cancel")
    async def cancel(task_id: str):
        return await run_in_threadpool(lambda: resolve().cancel_task(task_id))

    return router
