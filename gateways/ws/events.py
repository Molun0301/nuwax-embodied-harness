from __future__ import annotations

import asyncio
import json
from typing import Dict, List, Optional, Tuple

from fastapi import APIRouter
from fastapi import Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import StreamingResponse
from starlette.websockets import WebSocket
from starlette.websockets import WebSocketDisconnect

from contracts.events import RuntimeEventCategory
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.feedback import build_public_runtime_event
from gateways.runtime import GatewayRuntime
from gateways.ws.client_queue import ClientQueue


_TERMINAL_EVENT_TYPES = frozenset(
    {
        "task.succeeded",
        "task.failed",
        "task.cancelled",
        "task.timeout",
        "task.resource_quarantined",
    }
)


def _parse_categories(values: Optional[str]) -> Optional[Tuple[RuntimeEventCategory, ...]]:
    if not values:
        return None
    categories: List[RuntimeEventCategory] = []
    for raw_item in values.split(","):
        item = raw_item.strip()
        if not item:
            continue
        categories.append(RuntimeEventCategory(item))
    return tuple(categories) if categories else None


def _parse_event_types(values: Optional[str]) -> Optional[Tuple[str, ...]]:
    if not values:
        return None
    event_types = tuple(item.strip() for item in values.split(",") if item.strip())
    return event_types if event_types else None


def _is_terminal_event(event) -> bool:
    return str(event.event_type or "") in _TERMINAL_EVENT_TYPES


def _take_queue_gap_notice(queue: ClientQueue, runtime: GatewayRuntime) -> Optional[Dict[str, object]]:
    state = queue.take_drops()
    if not isinstance(state, dict) or not state.get("dropped_count"):
        return None
    first_dropped_cursor = state.get("first_dropped_cursor")
    effective_after_cursor = (
        max(0, int(first_dropped_cursor) - 1)
        if isinstance(first_dropped_cursor, int) and not isinstance(first_dropped_cursor, bool)
        else 0
    )
    latest_cursor = runtime.event_bus.latest_cursor()
    notice = {
        "cursor": effective_after_cursor,
        "latest_cursor": latest_cursor,
        "reason": "client_queue_overflow",
        "dropped_count": int(state["dropped_count"]),
        "first_dropped_cursor": first_dropped_cursor,
        "last_dropped_cursor": state.get("last_dropped_cursor"),
        "terminal_dropped": bool(state.get("terminal_dropped")),
        "effective_after_cursor": effective_after_cursor,
        "resync_required": True,
        "next_action": "使用 effective_after_cursor 重新连接事件流。",
    }
    return notice


def _replay_gap_notice(snapshot) -> Optional[Dict[str, object]]:
    if not snapshot.replay_gap:
        return None
    return {
        "cursor": snapshot.effective_after_cursor,
        "reason": snapshot.gap_reason,
        "requested_after_cursor": snapshot.requested_after_cursor,
        "effective_after_cursor": snapshot.effective_after_cursor,
        "oldest_cursor": snapshot.oldest_cursor,
        "latest_cursor": snapshot.latest_cursor,
        "resync_required": snapshot.resync_required,
        "next_action": "按本次回放重建状态，并从 latest_cursor 继续。",
    }


def build_event_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造事件流路由。"""

    router = APIRouter()

    @router.get("/events/stream")
    async def sse_events(
        request: Request,
        after_cursor: int = 0,
        categories: Optional[str] = None,
        event_types: Optional[str] = None,
        max_events: Optional[int] = None,
    ) -> StreamingResponse:
        category_filter = _parse_categories(categories)
        event_type_filter = _parse_event_types(event_types)

        async def event_generator():
            loop = asyncio.get_running_loop()
            queue = ClientQueue(loop, maxsize=200, is_terminal=_is_terminal_event)
            emitted = 0

            subscription_id, replay_snapshot = runtime.event_bus.subscribe_with_replay(
                queue.put,
                after_cursor=after_cursor,
                categories=category_filter,
                event_types=event_type_filter,
            )
            try:
                replay_gap = _replay_gap_notice(replay_snapshot)
                if replay_gap is not None:
                    yield _format_sse_payload(
                        "keepalive",
                        replay_gap,
                        cursor=replay_gap["cursor"],
                    )
                for event in replay_snapshot.events:
                    payload = await run_in_threadpool(build_public_runtime_event, event)
                    yield _format_sse_payload("runtime.event", payload, cursor=event.cursor)
                    emitted += 1
                    if max_events is not None and emitted >= max_events:
                        return

                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        event = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
                        payload = await run_in_threadpool(build_public_runtime_event, event)
                        queue_gap = _take_queue_gap_notice(queue, runtime)
                        if queue_gap is not None:
                            yield _format_sse_payload(
                                "keepalive",
                                queue_gap,
                                cursor=queue_gap["cursor"],
                            )
                        yield _format_sse_payload("runtime.event", payload, cursor=payload.get("cursor"))
                        emitted += 1
                        if max_events is not None and emitted >= max_events:
                            return
                    except asyncio.TimeoutError:
                        yield _format_sse_payload(
                            "keepalive",
                            {"cursor": runtime.event_bus.latest_cursor()},
                            cursor=runtime.event_bus.latest_cursor(),
                        )
            finally:
                queue.close()
                runtime.event_bus.unsubscribe(subscription_id)

        return StreamingResponse(event_generator(), media_type="text/event-stream")

    @router.websocket("/events/ws")
    async def websocket_events(websocket: WebSocket) -> None:
        try:
            category_filter = _parse_categories(websocket.query_params.get("categories"))
            event_type_filter = _parse_event_types(websocket.query_params.get("event_types"))
            after_cursor = int(websocket.query_params.get("after_cursor", "0"))
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"事件订阅初始化失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue = ClientQueue(loop, maxsize=200, is_terminal=_is_terminal_event)

        subscription_id, replay_snapshot = runtime.event_bus.subscribe_with_replay(
            queue.put,
            after_cursor=after_cursor,
            categories=category_filter,
            event_types=event_type_filter,
        )
        try:
            replay_gap = _replay_gap_notice(replay_snapshot)
            if replay_gap is not None:
                await websocket.send_json({"type": "keepalive", **replay_gap})
            for event in replay_snapshot.events:
                payload = await run_in_threadpool(build_public_runtime_event, event)
                await websocket.send_json({"type": "event", "event": payload})

            while True:
                try:
                    event = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.ws_ping_interval_sec)
                    payload = await run_in_threadpool(build_public_runtime_event, event)
                    queue_gap = _take_queue_gap_notice(queue, runtime)
                    if queue_gap is not None:
                        await websocket.send_json({"type": "keepalive", **queue_gap})
                    await websocket.send_json({"type": "event", "event": payload})
                except asyncio.TimeoutError:
                    await websocket.send_json(
                        {"type": "keepalive", "cursor": runtime.event_bus.latest_cursor()}
                    )
        except WebSocketDisconnect:
            return
        finally:
            queue.close()
            runtime.event_bus.unsubscribe(subscription_id)

    return router


def _format_sse_payload(event_name: str, payload: Dict[str, object], *, cursor: Optional[object] = None) -> str:
    lines = []
    if cursor is not None:
        lines.append(f"id: {cursor}")
    lines.append(f"event: {event_name}")
    lines.append(f"data: {json.dumps(payload, ensure_ascii=False, allow_nan=False)}")
    lines.append("")
    return "\n".join(lines) + "\n"
