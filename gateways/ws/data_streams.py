from __future__ import annotations

import asyncio
import base64
from collections.abc import Mapping
from dataclasses import fields as dataclass_fields, is_dataclass
from datetime import date, datetime
from decimal import Decimal
from enum import Enum
import json
import math
from pathlib import Path
import time
from typing import Any, Dict, Iterable, Optional, Tuple
from uuid import UUID

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, StreamingResponse
from starlette.websockets import WebSocket, WebSocketDisconnect

from contracts.streams import StreamFrame, StreamPayloadKind
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.feedback import build_public_runtime_event
from gateways.runtime import GatewayRuntime

try:
    from services.geometry.video_shm_reader import VideoShmReader
except ImportError:
    VideoShmReader = None


_STREAM_QUEUE_MAXSIZE = 200
_STREAM_REPLAY_MAX_FRAMES = 200
_STREAM_JSON_MAX_DEPTH = 12
_STREAM_JSON_MAX_NODES = 2048
_STREAM_JSON_MAX_ENTRIES = 4096
_STREAM_JSON_MAX_CONTAINER_ITEMS = 256
_STREAM_JSON_MAX_STRING_CHARS = 8192
_STREAM_JSON_MAX_TOTAL_STRING_CHARS = 64 * 1024
_STREAM_JSON_MAX_BYTES = 16 * 1024
_STREAM_JSON_MAX_INTEGER_BITS = 256
_TERMINAL_TASK_EVENT_TYPES = frozenset(
    {
        "task.succeeded",
        "task.failed",
        "task.cancelled",
        "task.timeout",
        "task.resource_quarantined",
    }
)


class _JsonProjectionBudget:
    def __init__(self) -> None:
        self.nodes_remaining = _STREAM_JSON_MAX_NODES
        self.entries_remaining = _STREAM_JSON_MAX_ENTRIES
        self.text_chars_remaining = _STREAM_JSON_MAX_TOTAL_STRING_CHARS
        self.bytes_remaining = _STREAM_JSON_MAX_BYTES

    def take_node(self) -> bool:
        if self.nodes_remaining <= 0:
            return False
        self.nodes_remaining -= 1
        return True

    def take_entry(self) -> bool:
        if self.entries_remaining <= 0:
            return False
        self.entries_remaining -= 1
        return True


def _bounded_json_text(
    value: str,
    budget: _JsonProjectionBudget,
    *,
    max_chars: int = _STREAM_JSON_MAX_STRING_CHARS,
) -> str:
    available = min(max(0, int(max_chars)), budget.text_chars_remaining)
    if available <= 0:
        return ""
    truncated = len(value) > available
    content_limit = max(0, available - 1) if truncated else available
    fragment = value[:content_limit].encode("utf-8", errors="replace").decode("utf-8")
    result = f"{fragment}…" if truncated else fragment
    budget.text_chars_remaining -= len(result)
    return result


def _safe_type_name(value: object, budget: _JsonProjectionBudget) -> str:
    value_type = type(value)
    module = getattr(value_type, "__module__", "")
    name = getattr(value_type, "__qualname__", getattr(value_type, "__name__", "object"))
    type_name = f"{module}.{name}" if module and module != "builtins" else str(name)
    return _bounded_json_text(type_name, budget, max_chars=256)


def _project_json_key(key: object, budget: _JsonProjectionBudget, index: int) -> str:
    if isinstance(key, str):
        raw_key = key
    elif key is None:
        raw_key = "null"
    elif isinstance(key, bool):
        raw_key = "true" if key else "false"
    elif isinstance(key, int):
        raw_key = str(key) if key.bit_length() <= _STREAM_JSON_MAX_INTEGER_BITS else f"<integer:{key.bit_length()}bits>"
    elif isinstance(key, float):
        if math.isnan(key):
            raw_key = "NaN"
        elif math.isinf(key):
            raw_key = "Infinity" if key > 0 else "-Infinity"
        else:
            raw_key = str(key)
    elif isinstance(key, Enum) and isinstance(key.value, (str, int, float, bool)):
        raw_key = str(key.value)
    else:
        raw_key = f"<{type(key).__name__}:{index}>"
    return _bounded_json_text(raw_key, budget, max_chars=256) or f"item_{index}"


def _project_json_bytes(value: object, budget: _JsonProjectionBudget) -> Dict[str, object]:
    size = len(value)  # type: ignore[arg-type]
    included = min(size, budget.bytes_remaining)
    budget.bytes_remaining -= included
    chunk = bytes(value[:included])  # type: ignore[index]
    encoded = base64.b64encode(chunk).decode("ascii")
    return {
        "$type": "bytes",
        "encoding": "base64",
        "data": _bounded_json_text(encoded, budget),
        "size": size,
        "truncated": included < size,
    }


def _stream_frame_values(frame: StreamFrame) -> Dict[str, object]:
    model_fields = getattr(type(frame), "model_fields", {})
    values = {
        str(field_name): getattr(frame, field_name)
        for field_name in model_fields
    }
    if frame.topic == "tasks.state":
        values["payload"] = build_public_runtime_event(frame.payload)
    return values


def _project_json_value(
    value: object,
    budget: _JsonProjectionBudget,
    active_ids: set[int],
    *,
    depth: int,
) -> object:
    if not budget.take_node():
        return "<truncated:node_budget>"
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        if value.bit_length() <= _STREAM_JSON_MAX_INTEGER_BITS:
            return value
        return f"<integer:{value.bit_length()}bits>"
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0 else "-Infinity"
        return value
    if isinstance(value, str):
        return _bounded_json_text(value, budget)
    if isinstance(value, (bytes, bytearray, memoryview)):
        return _project_json_bytes(value, budget)
    if isinstance(value, Enum):
        enum_value = value.value
        if isinstance(enum_value, (str, int, float, bool)) or enum_value is None:
            return _project_json_value(enum_value, budget, active_ids, depth=depth + 1)
        return _safe_type_name(value, budget)
    if isinstance(value, Decimal):
        if not value.is_finite():
            return str(value)
        return _bounded_json_text(str(value), budget)
    if isinstance(value, (datetime, date)):
        try:
            return _bounded_json_text(value.isoformat(), budget, max_chars=160)
        except Exception:
            return _safe_type_name(value, budget)
    if isinstance(value, (Path, UUID)):
        return _bounded_json_text(str(value), budget)
    if depth >= _STREAM_JSON_MAX_DEPTH:
        return "<truncated:max_depth>"

    value_id = id(value)
    if value_id in active_ids:
        return "<truncated:cycle>"
    active_ids.add(value_id)
    try:
        if isinstance(value, StreamFrame):
            return _project_json_value(
                _stream_frame_values(value),
                budget,
                active_ids,
                depth=depth + 1,
            )

        if isinstance(value, Mapping):
            projected: Dict[str, object] = {}
            try:
                for index, (key, item) in enumerate(value.items()):
                    if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS:
                        projected["$truncated"] = "item_limit"
                        break
                    if not budget.take_entry():
                        projected["$truncated"] = "entry_budget"
                        break
                    projected_key = _project_json_key(key, budget, index)
                    while projected_key in projected:
                        projected_key = f"{projected_key}_{index}"
                    projected[projected_key] = _project_json_value(
                        item,
                        budget,
                        active_ids,
                        depth=depth + 1,
                    )
            except Exception as exc:
                projected["$projection_error"] = type(exc).__name__
            return projected

        if isinstance(value, (list, tuple, set, frozenset)):
            projected_items = []
            try:
                for index, item in enumerate(value):
                    if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS:
                        projected_items.append("<truncated:item_limit>")
                        break
                    if not budget.take_entry():
                        projected_items.append("<truncated:entry_budget>")
                        break
                    projected_items.append(
                        _project_json_value(
                            item,
                            budget,
                            active_ids,
                            depth=depth + 1,
                        )
                    )
            except Exception as exc:
                projected_items.append(f"<projection_error:{type(exc).__name__}>")
            return projected_items

        model_fields = getattr(type(value), "model_fields", None)
        if isinstance(model_fields, Mapping):
            field_values: Dict[str, object] = {}
            for index, field_name in enumerate(model_fields):
                if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS or not budget.take_entry():
                    field_values["$truncated"] = "field_budget"
                    break
                try:
                    field_values[str(field_name)] = getattr(value, field_name)
                except Exception as exc:
                    field_values[str(field_name)] = f"<projection_error:{type(exc).__name__}>"
            return _project_json_value(field_values, budget, active_ids, depth=depth + 1)

        if is_dataclass(value) and not isinstance(value, type):
            field_values = {}
            for index, field in enumerate(dataclass_fields(value)):
                if index >= _STREAM_JSON_MAX_CONTAINER_ITEMS or not budget.take_entry():
                    field_values["$truncated"] = "field_budget"
                    break
                try:
                    field_values[field.name] = getattr(value, field.name)
                except Exception as exc:
                    field_values[field.name] = f"<projection_error:{type(exc).__name__}>"
            return _project_json_value(field_values, budget, active_ids, depth=depth + 1)

        try:
            attributes = vars(value)
        except (TypeError, ValueError):
            attributes = None
        projected_object: Dict[str, object] = {"$type": _safe_type_name(value, budget)}
        if isinstance(attributes, Mapping) and attributes:
            projected_object["attributes"] = _project_json_value(
                attributes,
                budget,
                active_ids,
                depth=depth + 1,
            )
        return projected_object
    finally:
        active_ids.discard(value_id)


def _strict_json_dumps(value: object, *, compact: bool = False) -> str:
    projected = _project_json_value(
        value,
        _JsonProjectionBudget(),
        set(),
        depth=0,
    )
    return json.dumps(
        projected,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":") if compact else None,
    )


async def _send_ws_json(websocket: WebSocket, payload: object) -> None:
    await websocket.send_text(_strict_json_dumps(payload, compact=True))


def _parse_topics(values: Optional[str]) -> Optional[Tuple[str, ...]]:
    if not values:
        return None
    topics = tuple(item.strip() for item in values.split(",") if item.strip())
    return topics if topics else None


def _normalize_after_cursor(after_cursor: int, latest_cursor: int) -> int:
    if after_cursor <= 0:
        return 0
    if after_cursor > latest_cursor:
        return 0
    return after_cursor


def _replay_stream_frames(
    runtime: GatewayRuntime,
    *,
    after_cursor: int,
    topics: Optional[Iterable[str]],
) -> Tuple[StreamFrame, ...]:
    _, frames = _build_replay_batch(
        runtime,
        after_cursor=after_cursor,
        topics=topics,
    )
    return frames


def _build_replay_batch(
    runtime: GatewayRuntime,
    *,
    after_cursor: int,
    topics: Optional[Iterable[str]],
    limit: int = _STREAM_REPLAY_MAX_FRAMES,
    snapshot=None,
) -> Tuple[Dict[str, Any], Tuple[StreamFrame, ...]]:
    topic_filter = tuple(str(topic) for topic in topics) if topics is not None else None
    if snapshot is None:
        snapshot = runtime.stream_bus.replay_snapshot(
            after_cursor=after_cursor,
            topics=topic_filter,
        )
    latest_cursor = int(snapshot.latest_cursor)
    oldest_cursor = int(snapshot.oldest_cursor)
    retained_frames = tuple(snapshot.frames)
    topic_oldest_cursors = dict(snapshot.topic_oldest_cursors)
    topic_evicted_through_cursors = dict(snapshot.topic_evicted_through_cursors)
    requested_after_cursor = max(0, int(after_cursor or 0))
    normalized_after_cursor = _normalize_after_cursor(requested_after_cursor, latest_cursor)
    gap_reason: Optional[str] = None
    if requested_after_cursor > 0 and requested_after_cursor > latest_cursor:
        gap_reason = "cursor_ahead_of_stream"
    elif requested_after_cursor > 0 and any(
        requested_after_cursor < evicted_through_cursor
        for evicted_through_cursor in topic_evicted_through_cursors.values()
    ):
        gap_reason = "cursor_expired"

    frames = retained_frames
    if gap_reason and not frames:
        frames = runtime.stream_bus.latest_frames(topics=topic_filter)

    frame_limit = max(1, int(limit))
    truncated = len(frames) > frame_limit
    if truncated:
        frames = frames[-frame_limit:]

    frames = tuple(
        frame
        for frame in frames
        if frame.cursor is None or frame.cursor <= latest_cursor
    )
    status: Dict[str, Any] = {
        "requested_after_cursor": requested_after_cursor,
        "effective_after_cursor": normalized_after_cursor,
        "oldest_cursor": oldest_cursor,
        "topic_oldest_cursors": topic_oldest_cursors,
        "topic_evicted_through_cursors": topic_evicted_through_cursors,
        "latest_cursor": latest_cursor,
        "replay_gap": gap_reason is not None,
        "gap_reason": gap_reason,
        "resync_required": gap_reason is not None,
        "truncated": truncated,
        "frame_count": len(frames),
    }
    return status, frames


async def _run_blocking(callable_obj, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


def _frame_event_type(frame: StreamFrame) -> str:
    payload = getattr(frame, "payload", None)
    if isinstance(payload, dict):
        return str(payload.get("event_type") or payload.get("runtime_event_type") or "")
    return str(getattr(payload, "event_type", "") or "")


def _frame_priority(frame: StreamFrame) -> int:
    event_type = _frame_event_type(frame)
    if frame.topic == "tasks.state" and event_type in _TERMINAL_TASK_EVENT_TYPES:
        return 100
    if frame.topic == "logs.entry":
        payload = frame.payload if isinstance(frame.payload, dict) else {}
        severity = str(payload.get("severity") or payload.get("level") or "").lower()
        return 80 if severity in {"error", "critical"} else 20
    if frame.topic == "tasks.state":
        return 60
    if frame.payload_kind == StreamPayloadKind.DIAGNOSTIC:
        return 70
    if frame.payload_kind == StreamPayloadKind.BINARY:
        return 5
    return 40


def _record_queue_drop(queue: asyncio.Queue, frame: StreamFrame) -> None:
    state = getattr(queue, "_nuwax_drop_state", None)
    if not isinstance(state, dict):
        state = {
            "dropped_count": 0,
            "first_dropped_cursor": None,
            "last_dropped_cursor": None,
            "topics": set(),
        }
        setattr(queue, "_nuwax_drop_state", state)
    state["dropped_count"] += 1
    cursor = getattr(frame, "cursor", None)
    if cursor is not None:
        state["first_dropped_cursor"] = state["first_dropped_cursor"] or cursor
        state["last_dropped_cursor"] = cursor
    state["topics"].add(str(getattr(frame, "topic", "unknown")))


def _take_queue_gap_notice(queue: asyncio.Queue, runtime: GatewayRuntime) -> Optional[Dict[str, Any]]:
    state = getattr(queue, "_nuwax_drop_state", None)
    if not isinstance(state, dict) or not state.get("dropped_count"):
        return None
    notice = {
        "reason": "client_queue_overflow",
        "dropped_count": int(state["dropped_count"]),
        "first_dropped_cursor": state.get("first_dropped_cursor"),
        "last_dropped_cursor": state.get("last_dropped_cursor"),
        "topics": sorted(state.get("topics") or ()),
        "latest_cursor": runtime.stream_bus.latest_cursor(),
        "resync_required": True,
        "next_action": "重新获取任务快照；日志等增量数据从最新游标继续。",
    }
    setattr(queue, "_nuwax_drop_state", None)
    return notice


def _safe_enqueue(loop: asyncio.AbstractEventLoop, queue: asyncio.Queue, frame: StreamFrame, runtime: GatewayRuntime) -> None:
    def _push() -> None:
        if frame.payload_kind == StreamPayloadKind.BINARY:
            retained = []
            while not queue.empty():
                queued = queue.get_nowait()
                if queued.payload_kind == StreamPayloadKind.BINARY and queued.topic == frame.topic:
                    continue
                retained.append(queued)
            for queued in retained:
                queue.put_nowait(queued)
        if queue.full():
            queued_frames = []
            while not queue.empty():
                queued_frames.append(queue.get_nowait())
            incoming_priority = _frame_priority(frame)
            drop_index = min(
                range(len(queued_frames)),
                key=lambda index: (_frame_priority(queued_frames[index]), index),
            )
            if _frame_priority(queued_frames[drop_index]) > incoming_priority:
                for queued in queued_frames:
                    queue.put_nowait(queued)
                _record_queue_drop(queue, frame)
                return
            dropped = queued_frames.pop(drop_index)
            _record_queue_drop(queue, dropped)
            for queued in queued_frames:
                queue.put_nowait(queued)
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
        resume_after_cursor = int(after_cursor or 0)
        if resume_after_cursor <= 0:
            resume_after_cursor = _positive_int(request.headers.get("last-event-id"), 0)

        async def frame_generator():
            loop = asyncio.get_running_loop()
            queue: asyncio.Queue[StreamFrame] = asyncio.Queue(maxsize=_STREAM_QUEUE_MAXSIZE)
            emitted = 0
            subscription_id, replay_snapshot = runtime.stream_bus.subscribe_with_replay(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                after_cursor=resume_after_cursor,
                topics=topic_filter,
            )
            try:
                replay_status, replay_frames = _build_replay_batch(
                    runtime,
                    after_cursor=resume_after_cursor,
                    topics=topic_filter,
                    snapshot=replay_snapshot,
                )
                replay_latest_cursor = replay_status["latest_cursor"]
                for frame in replay_frames:
                    yield _format_sse_payload(frame)
                    emitted += 1
                    if max_frames is not None and emitted >= max_frames:
                        return
                yield _format_sse("stream.replay", replay_status)
                while True:
                    if await request.is_disconnected():
                        break
                    try:
                        frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.sse_keepalive_sec)
                        if frame.cursor is not None and frame.cursor <= replay_latest_cursor:
                            continue
                        gap_notice = _take_queue_gap_notice(queue, runtime)
                        if gap_notice is not None:
                            yield _format_sse(
                                "stream.gap",
                                gap_notice,
                            )
                        yield _format_sse_payload(frame)
                        emitted += 1
                        if max_frames is not None and emitted >= max_frames:
                            return
                    except asyncio.TimeoutError:
                        yield _format_sse_keepalive(runtime.stream_bus.latest_cursor())
            finally:
                runtime.stream_bus.unsubscribe(subscription_id)

        return StreamingResponse(frame_generator(), media_type="text/event-stream")

    @router.get("/streams/video.mjpeg")
    async def video_mjpeg_stream(
        request: Request,
        camera_id: Optional[str] = None,
        fps: float = 15.0,
    ):
        access_manager.authenticate_http(request)
        video_available = False
        if VideoShmReader is not None:
            import os as _os
            _shm_path = _os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
            _probe = VideoShmReader(_shm_path)
            video_available = _probe.has_new_frame()
            _probe.close()
        if not video_available:
            return JSONResponse(
                status_code=503,
                content={
                    "error": "video_stream_not_available",
                    "reason": "rust_bridge_no_frames",
                    "message": "Rust 视频中转尚未收到帧数据。",
                },
            )
        requested_fps = float(fps or 15.0)
        if not math.isfinite(requested_fps):
            requested_fps = 15.0
        frame_interval_sec = 1.0 / min(max(requested_fps, 1.0), 24.0)

        async def frame_generator():
            shm_reader = None
            if VideoShmReader is not None:
                import os
                shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
                shm_reader = VideoShmReader(shm_path)

            while not await request.is_disconnected():
                started_at = time.monotonic()
                frame_data = None
                if shm_reader is not None:
                    result = shm_reader.read_latest()
                    if result is not None:
                        frame_data, _, _ = result
                if frame_data is not None:
                    yield _format_mjpeg_frame(frame_data)
                elapsed = time.monotonic() - started_at
                if elapsed < frame_interval_sec:
                    await asyncio.sleep(frame_interval_sec - elapsed)

        return StreamingResponse(
            frame_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @router.websocket("/streams/ws")
    async def websocket_streams(websocket: WebSocket) -> None:
        try:
            access_manager.authenticate_websocket(websocket)
            topic_filter = _parse_topics(websocket.query_params.get("topics"))
            after_cursor = int(websocket.query_params.get("after_cursor", "0"))
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"数据流订阅初始化失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[StreamFrame] = asyncio.Queue(maxsize=_STREAM_QUEUE_MAXSIZE)
        subscription_id: Optional[str] = None

        try:
            subscription_id, replay_snapshot = runtime.stream_bus.subscribe_with_replay(
                lambda frame: _safe_enqueue(loop, queue, frame, runtime),
                after_cursor=after_cursor,
                topics=topic_filter,
            )
            replay_status, replay_frames = _build_replay_batch(
                runtime,
                after_cursor=after_cursor,
                topics=topic_filter,
                snapshot=replay_snapshot,
            )
            replay_latest_cursor = replay_status["latest_cursor"]
            for frame in replay_frames:
                if getattr(frame, 'payload_kind', None) == StreamPayloadKind.BINARY:
                    continue  # binary 帧不回放
                await _send_ws_json(websocket, {"type": "stream.frame", "frame": frame})
            await _send_ws_json(websocket, {"type": "stream.replay", **replay_status})
            while True:
                try:
                    frame = await asyncio.wait_for(queue.get(), timeout=runtime.config.gateway.ws_ping_interval_sec)
                    if frame.cursor is not None and frame.cursor <= replay_latest_cursor:
                        continue
                    gap_notice = _take_queue_gap_notice(queue, runtime)
                    if gap_notice is not None:
                        await _send_ws_json(websocket, {"type": "stream.gap", **gap_notice})
                    if getattr(frame, 'payload_kind', None) == StreamPayloadKind.BINARY and isinstance(frame.payload, (bytes, bytearray)):
                        await websocket.send_bytes(frame.payload)
                    else:
                        await _send_ws_json(websocket, {"type": "stream.frame", "frame": frame})
                except asyncio.TimeoutError:
                    await _send_ws_json(
                        websocket,
                        {"type": "keepalive", "cursor": runtime.stream_bus.latest_cursor()},
                    )
        except WebSocketDisconnect:
            return
        finally:
            if subscription_id is not None:
                runtime.stream_bus.unsubscribe(subscription_id)

    return router


def _positive_int(value: object, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return int(default)
    return parsed if parsed > 0 else int(default)


def _format_mjpeg_frame(data: bytes) -> bytes:
    headers = (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        + f"Content-Length: {len(data)}".encode("ascii")
        + b"\r\n\r\n"
    )
    return headers + data + b"\r\n"


def _format_sse_payload(frame: StreamFrame) -> str:
    return _format_sse("stream.frame", frame, cursor=frame.cursor)


def _format_sse_keepalive(cursor: int) -> str:
    return _format_sse("keepalive", {"cursor": cursor})


def _format_sse(event_name: str, payload: object, *, cursor: Optional[object] = None) -> str:
    lines = []
    if cursor is not None:
        lines.append(f"id: {cursor}")
    lines.append(f"event: {event_name}")
    lines.append(f"data: {_strict_json_dumps(payload)}")
    lines.append("")
    return "\n".join(lines) + "\n"
