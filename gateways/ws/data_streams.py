from __future__ import annotations

import asyncio
import json
from typing import Dict, Iterable, Optional, Tuple

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse
from starlette.websockets import WebSocket, WebSocketDisconnect

from contracts.streams import StreamFrame
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable


def _parse_topics(values: Optional[str]) -> Optional[Tuple[str, ...]]:
    if not values:
        return None
    topics = tuple(item.strip() for item in values.split(",") if item.strip())
    return topics if topics else None


def _normalize_after_cursor(after_cursor: int, latest_cursor: int) -> int:
    if after_cursor <= 0:
        return 0
    if latest_cursor > 0 and after_cursor > latest_cursor:
        return 0
    return after_cursor


def _replay_stream_frames(
    runtime: GatewayRuntime,
    *,
    after_cursor: int,
    topics: Optional[Iterable[str]],
) -> Tuple[StreamFrame, ...]:
    oldest_cursor = runtime.stream_bus.oldest_cursor()
    if after_cursor <= 0 or (oldest_cursor > 0 and after_cursor < oldest_cursor):
        return runtime.stream_bus.latest_frames(topics=topics)
    return runtime.stream_bus.replay(after_cursor=after_cursor, topics=topics)


def _safe_enqueue(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue, frame: StreamFrame, runtime: GatewayRuntime) -> None:
    def _push() -> None:
        if queue.full():
            try:
                dropped = queue.get_nowait()
                topic = dropped.topic if isinstance(dropped, StreamFrame) else "unknown"
                runtime.stream_bus.publish_dropped(topic, source="streams_router")
            except asyncio.QueueEmpty:
                pass
        queue.put_nowait(frame)

    loop.call_soon_threadsafe(_push)


def build_data_stream_router(runtime: GatewayRuntime, access_manager: GatewayAccessManager) -> APIRouter:
    """构造实时数据流路由。"""

    router = APIRouter()

    @router.get("/streams/sse")
    async def sse_streams(
        request: Request,
        topics: Optional[str] = None,
        after_cursor: int = 0,
        max_frames: Optional[int] = None,
    ) -> StreamingResponse:
        access_manager.authenticate_http(request)
        topic_filter = _parse_topics(topics)
        after_cursor = _normalize_after_cursor(after_cursor, runtime.stream_bus.latest_cursor())

        async def frame_generator():
            loop = asyncio.get_running_loop()
            queue: asyncio.Queue[StreamFrame] = asyncio.Queue(maxsize=200)
            emitted = 0

            for frame in _replay_stream_frames(runtime, after_cursor=after_cursor, topics=topic_filter):
                yield _format_sse_payload(frame)
                emitted += 1
                if max_frames is not None and emitted >= max_frames:
                    return

            subscription_id = runtime.stream_bus.subscribe(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                topics=topic_filter,
            )
            try:
                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
                        yield _format_sse_payload(frame)
                        emitted += 1
                        if max_frames is not None and emitted >= max_frames:
                            return
                    except asyncio.TimeoutError:
                        yield _format_sse_keepalive(runtime.stream_bus.latest_cursor())
            finally:
                runtime.stream_bus.unsubscribe(subscription_id)

        return StreamingResponse(frame_generator(), media_type="text/event-stream")

    @router.websocket("/streams/ws")
    async def websocket_streams(websocket: WebSocket) -> None:
        try:
            access_manager.authenticate_websocket(websocket)
            topic_filter = _parse_topics(websocket.query_params.get("topics"))
            after_cursor = int(websocket.query_params.get("after_cursor", "0"))
            after_cursor = _normalize_after_cursor(after_cursor, runtime.stream_bus.latest_cursor())
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"数据流订阅初始化失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[StreamFrame] = asyncio.Queue(maxsize=200)

        try:
            for frame in _replay_stream_frames(runtime, after_cursor=after_cursor, topics=topic_filter):
                await websocket.send_json({"type": "stream.frame", "frame": to_jsonable(frame)})
        except WebSocketDisconnect:
            return

        subscription_id = runtime.stream_bus.subscribe(
            lambda frame: _safe_enqueue(loop, queue, frame, runtime),
            topics=topic_filter,
        )
        try:
            while True:
                try:
                    frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.ws_ping_interval_sec)
                    await websocket.send_json({"type": "stream.frame", "frame": to_jsonable(frame)})
                except asyncio.TimeoutError:
                    await websocket.send_json({"type": "keepalive", "cursor": runtime.stream_bus.latest_cursor()})
        except WebSocketDisconnect:
            return
        finally:
            runtime.stream_bus.unsubscribe(subscription_id)

    return router


def _format_sse_payload(frame: StreamFrame) -> str:
    return _format_sse("stream.frame", to_jsonable(frame), cursor=frame.cursor)


def _format_sse_keepalive(cursor: int) -> str:
    return _format_sse("keepalive", {"cursor": cursor}, cursor=cursor)


def _format_sse(event_name: str, payload: Dict[str, object], *, cursor: Optional[object] = None) -> str:
    lines = []
    if cursor is not None:
        lines.append(f"id: {cursor}")
    lines.append(f"event: {event_name}")
    lines.append(f"data: {json.dumps(payload, ensure_ascii=False)}")
    lines.append("")
    return "\n".join(lines) + "\n"
