"""共用事件订阅：推送、回放、缺口和连接清理。"""

import asyncio
import json
import re

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import StreamingResponse
from typing import Optional

from core.errors import GatewayError
from gateways.feedback_messages import TERMINAL_EVENTS, build_feedback_message, is_feedback_event
from gateways.ws.client_queue import ClientQueue


def feedback_cursor(runtime, value):
    """续传标识含总线实例，防止重启后游标数值碰巧相同。"""
    if not value:
        return 0, None
    if not re.fullmatch(r"[a-f0-9]{32}:[0-9]{1,20}", value):
        raise GatewayError("反馈续传标识无效。", error_code="invalid_params", http_status=422)
    instance, cursor = value.split(":")
    if instance != runtime.event_bus.instance_id:
        return 0, "stream_restarted"
    return int(cursor), None


def feedback_id(runtime, cursor):
    return "%s:%s" % (runtime.event_bus.instance_id, cursor)


def format_feedback_sse(kind, payload, event_id=None):
    prefix = "id: %s\n" % event_id if event_id else ""
    return prefix + "event: %s\ndata: %s\n\n" % (
        kind, json.dumps(payload, ensure_ascii=False, allow_nan=False))


async def feedback_events(runtime, request, *, after_cursor=0, reset_reason=None,
                          request_id=None, task_id=None, capability_name=None, queue_size=200):
    """仅事件到达时取消息；保活不读取任务状态，也不推进已交付游标。"""
    queue = ClientQueue(asyncio.get_running_loop(), maxsize=queue_size,
                        is_terminal=lambda event: event.event_type in TERMINAL_EVENTS)

    def receive(event):
        if is_feedback_event(event):
            queue.put(event)

    subscription, replay = runtime.event_bus.subscribe_with_replay(receive, after_cursor=after_cursor)
    cursor = after_cursor

    def project(event):
        message = build_feedback_message(runtime, event)
        if message is None:
            return None
        if request_id and message.get("request_id") != request_id:
            return None
        if task_id and message.get("task_id") != task_id:
            return None
        if capability_name and message["source"].get("capability_name") != capability_name:
            return None
        return message

    try:
        reason = reset_reason or replay.gap_reason
        if not reason and not after_cursor and replay.oldest_cursor > 1:
            reason = "history_window"
        if reason:
            cursor = max(0, replay.oldest_cursor - 1)
            yield "feedback.gap", {
                "reason": reason, "resync_required": True,
                "message": "反馈历史不完整，请核对相关任务与日志；不要重新执行动作。",
                "oldest_cursor": replay.oldest_cursor, "latest_cursor": replay.latest_cursor,
            }, feedback_id(runtime, cursor)
        yield "feedback.ready", {"stream_id": runtime.event_bus.instance_id,
                                 "latest_cursor": replay.latest_cursor}, None
        for event in replay.events:
            message = await run_in_threadpool(project, event)
            cursor = event.cursor
            if message is not None:
                yield "feedback.message", message, feedback_id(runtime, cursor)
        while not await request.is_disconnected():
            try:
                event = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
            except asyncio.TimeoutError:
                yield "keepalive", {"stream_id": runtime.event_bus.instance_id}, None
                continue
            drops = queue.take_drops()
            if drops:
                # 本连接立即结束，由客户端从最后实际接收的位置重连；不越过丢失区间。
                yield "feedback.gap", {"reason": "client_queue_overflow", "resync_required": True,
                                       "message": "接收过慢，请从已接收位置恢复订阅。", **drops}, None
                return
            # 原子回放捕获的事件可能尚在派发队列中，忽略交接边界的重复通知。
            if event.cursor <= cursor:
                continue
            cursor = event.cursor
            message = await run_in_threadpool(project, event)
            if message is not None:
                yield "feedback.message", message, feedback_id(runtime, cursor)
    finally:
        queue.close()
        runtime.event_bus.unsubscribe(subscription)


def build_feedback_router(runtime):
    router = APIRouter()

    @router.get("/api/feedback/stream")
    async def stream(request: Request, after: Optional[str] = None,
                     request_id: Optional[str] = None, task_id: Optional[str] = None,
                     capability_name: Optional[str] = None):
        cursor, reason = feedback_cursor(runtime, request.headers.get("last-event-id") or after)

        async def generate():
            async for kind, payload, identity in feedback_events(
                    runtime, request, after_cursor=cursor, reset_reason=reason,
                    request_id=request_id, task_id=task_id, capability_name=capability_name):
                yield format_feedback_sse(kind, payload, identity)

        return StreamingResponse(generate(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    return router
