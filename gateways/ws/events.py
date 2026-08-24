from __future__ import annotations

import asyncio
import json
from typing import Dict, List, Optional, Tuple

from fastapi import APIRouter
from fastapi import Request
from fastapi.responses import StreamingResponse
from starlette.websockets import WebSocket
from starlette.websockets import WebSocketDisconnect

from contracts.events import RuntimeEventCategory
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.feedback import build_public_runtime_event
from gateways.runtime import GatewayRuntime


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


def _safe_enqueue(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue, payload: Dict[str, object]) -> None:
    def _push() -> None:
        if queue.full():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        queue.put_nowait(payload)

    loop.call_soon_threadsafe(_push)


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
            queue: asyncio.Queue[Dict[str, object]] = asyncio.Queue(maxsize=200)
            emitted = 0

            for event in runtime.event_bus.replay(
                after_cursor=after_cursor,
                categories=category_filter,
                event_types=event_type_filter,
            ):
                payload = build_public_runtime_event(event)
                yield _format_sse_payload("runtime.event", payload, cursor=event.cursor)
                emitted += 1
                if max_events is not None and emitted >= max_events:
                    return

            subscription_id = runtime.event_bus.subscribe(
                lambda event: _safe_enqueue(loop, queue, build_public_runtime_event(event)),
                categories=category_filter,
                event_types=event_type_filter,
            )
            try:
                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        payload = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
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
        queue: asyncio.Queue[Dict[str, object]] = asyncio.Queue(maxsize=200)

        for event in runtime.event_bus.replay(
            after_cursor=after_cursor,
            categories=category_filter,
            event_types=event_type_filter,
        ):
            await websocket.send_json({"type": "event", "event": build_public_runtime_event(event)})

        subscription_id = runtime.event_bus.subscribe(
            lambda event: _safe_enqueue(loop, queue, build_public_runtime_event(event)),
            categories=category_filter,
            event_types=event_type_filter,
        )
        try:
            while True:
                try:
                    payload = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.ws_ping_interval_sec)
                    await websocket.send_json({"type": "event", "event": payload})
                except asyncio.TimeoutError:
                    await websocket.send_json(
                        {"type": "keepalive", "cursor": runtime.event_bus.latest_cursor()}
                    )
        except WebSocketDisconnect:
            return
        finally:
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
