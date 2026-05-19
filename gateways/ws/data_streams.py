from __future__ import annotations

import asyncio
import json
import math
from uuid import uuid4
import time
from typing import Any, Dict, Iterable, Optional, Tuple

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, StreamingResponse
from starlette.websockets import WebSocket, WebSocketDisconnect

from contracts.streams import StreamFrame
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.runtime import GatewayRuntime
from gateways.serialization import to_jsonable

try:
    from services.geometry.video_shm_reader import VideoShmReader
except ImportError:
    VideoShmReader = None


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


async def _run_blocking(callable_obj, *args, **kwargs):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callable_obj(*args, **kwargs))


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

    @router.get("/streams/video.mjpeg")
    async def video_mjpeg_stream(
        request: Request,
        camera_id: Optional[str] = None,
        fps: float = 15.0,
    ):
        access_manager.authenticate_http(request)
        # 检查视频源可用性：共享内存或 WebSocket 接收器
        video_available = False
        if VideoShmReader is not None:
            import os as _os
            _shm_path = _os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
            _probe = VideoShmReader(_shm_path)
            if _probe.has_new_frame():
                video_available = True
            _probe.close()
        if not video_available:
            status = await _run_blocking(runtime.video_stream_receiver.status_snapshot)
            video_available = bool(status.get("available"))
        if not video_available:
            return JSONResponse(
                status_code=503,
                content={
                    "error": "video_stream_not_available",
                    "reason": "video_ingest_waiting_first_frame",
                    "message": "尚未收到视频首帧（Rust 桥和 WebSocket 均无数据）。",
                },
            )
        requested_fps = float(fps or 15.0)
        if not math.isfinite(requested_fps):
            requested_fps = 15.0
        frame_interval_sec = 1.0 / min(max(requested_fps, 1.0), 24.0)

        async def frame_generator():
            last_sequence = 0
            shm_reader = None
            if VideoShmReader is not None:
                import os
                shm_path = os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm")
                shm_reader = VideoShmReader(shm_path)

            while not await request.is_disconnected():
                started_at = time.monotonic()

                # 快速路径：从共享内存读取（Rust 视频中继进程写入）
                frame_data = None
                if shm_reader is not None:
                    result = shm_reader.read_latest()
                    if result is not None:
                        frame_data, seq, _ = result
                        last_sequence = seq

                # 回退路径：从 video_stream_receiver 读取（Python WebSocket 路径）
                if frame_data is None:
                    latest_frame = await _run_blocking(
                        runtime.video_stream_receiver.wait_for_frame,
                        last_sequence,
                        timeout_sec=frame_interval_sec,
                        camera_id=camera_id,
                    )
                    if latest_frame is not None:
                        last_sequence = latest_frame.sequence
                        frame_data = latest_frame.data

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

    @router.websocket("/streams/video.ingest")
    async def video_ingest_stream(websocket: WebSocket) -> None:
        client_id = f"go2-video-{uuid4().hex[:12]}"
        try:
            access_manager.authenticate_websocket(websocket)
        except Exception as exc:
            error = exc if isinstance(exc, GatewayError) else GatewayError(f"视频推流接入鉴权失败: {exc}")
            await websocket.close(code=4403, reason=error.message)
            return

        await websocket.accept()
        camera_id = "front_camera"
        frame_id = "go2/front_camera"
        width_px = 1280
        height_px = 720
        source_metadata: Dict[str, Any] = {"source_kind": "go2_side_stream_push"}

        try:
            start_message = await websocket.receive()
            if start_message.get("text") is not None:
                try:
                    payload = json.loads(str(start_message.get("text") or "{}"))
                except json.JSONDecodeError as exc:
                    raise GatewayError(f"视频推流启动消息不是合法 JSON: {exc}", error_code="video_ingest_bad_start", http_status=400) from exc
                if str(payload.get("type") or "") not in {"video.start", "video_start", ""}:
                    raise GatewayError("视频推流首条消息必须是 video.start。", error_code="video_ingest_bad_start", http_status=400)
                encoding = str(payload.get("encoding") or "jpeg").strip().lower()
                if encoding not in {"jpeg", "jpg", "image/jpeg"}:
                    raise GatewayError("视频推流 ingest 仅接受 JPEG 编码帧。", error_code="video_ingest_unsupported_encoding", http_status=415)
                camera_id = str(payload.get("camera_id") or camera_id)
                frame_id = str(payload.get("frame_id") or frame_id)
                width_px = _positive_int(payload.get("width_px"), width_px)
                height_px = _positive_int(payload.get("height_px"), height_px)
                source_metadata = {
                    "source_kind": str(payload.get("source_kind") or "go2_side_stream_push"),
                    "streamer_version": payload.get("streamer_version"),
                    "robot": payload.get("robot"),
                    "camera_info": payload.get("camera_info"),
                }
                await _run_blocking(
                    runtime.video_stream_receiver.start_stream,
                    client_id=client_id,
                    camera_id=camera_id,
                    frame_id=frame_id,
                    width_px=width_px,
                    height_px=height_px,
                    encoding="jpeg",
                    metadata=source_metadata,
                )
                runtime._publish_video_stream_frame(reason="video_ingest_connected")
                await websocket.send_json({"type": "video.accepted", "client_id": client_id})
            elif start_message.get("bytes") is not None:
                await _run_blocking(
                    runtime.video_stream_receiver.start_stream,
                    client_id=client_id,
                    camera_id=camera_id,
                    frame_id=frame_id,
                    width_px=width_px,
                    height_px=height_px,
                    encoding="jpeg",
                    metadata=source_metadata,
                )
                await _run_blocking(
                    runtime.video_stream_receiver.ingest_jpeg,
                    start_message["bytes"],
                    client_id=client_id,
                    camera_id=camera_id,
                    frame_id=frame_id,
                    width_px=width_px,
                    height_px=height_px,
                    metadata=source_metadata,
                )
                runtime._publish_video_stream_frame(reason="video_ingest_frame")
            else:
                raise GatewayError("视频推流首条消息为空。", error_code="video_ingest_bad_start", http_status=400)

            while True:
                message = await websocket.receive()
                if message.get("type") == "websocket.disconnect":
                    break
                if message.get("bytes") is not None:
                    latest_frame = await _run_blocking(
                        runtime.video_stream_receiver.ingest_jpeg,
                        message["bytes"],
                        client_id=client_id,
                        camera_id=camera_id,
                        frame_id=frame_id,
                        width_px=width_px,
                        height_px=height_px,
                        metadata=source_metadata,
                    )
                    if latest_frame.sequence == 1 or latest_frame.sequence % 5 == 0:
                        runtime._publish_video_stream_frame(reason="video_ingest_frame")
                    continue
                if message.get("text") is not None:
                    control = json.loads(str(message.get("text") or "{}"))
                    if str(control.get("type") or "") == "video.stop":
                        break
                    if str(control.get("type") or "") == "video.ping":
                        await websocket.send_json({"type": "video.pong", "client_id": client_id})
                        continue
                    continue
        except WebSocketDisconnect:
            pass
        except json.JSONDecodeError as exc:
            error_message = f"视频推流控制消息不是合法 JSON: {exc}"
            try:
                await websocket.close(code=1003, reason=error_message[:120])
            except Exception:
                pass
            await _run_blocking(
                runtime.video_stream_receiver.mark_disconnected,
                client_id=client_id,
                error=error_message,
            )
            runtime._publish_video_stream_frame(reason="video_ingest_control_error")
            return
        except GatewayError as exc:
            try:
                await websocket.close(code=1003, reason=exc.message[:120])
            except Exception:
                pass
            await _run_blocking(
                runtime.video_stream_receiver.mark_disconnected,
                client_id=client_id,
                error=exc.message,
            )
            runtime._publish_video_stream_frame(reason="video_ingest_rejected")
            return
        except Exception as exc:
            error_message = str(getattr(exc, "message", exc))
            try:
                await websocket.close(code=1011, reason=error_message[:120])
            except Exception:
                pass
            await _run_blocking(
                runtime.video_stream_receiver.mark_disconnected,
                client_id=client_id,
                error=error_message,
            )
            runtime._publish_video_stream_frame(reason="video_ingest_error")
            return
        finally:
            await _run_blocking(runtime.video_stream_receiver.mark_disconnected, client_id=client_id)
            runtime._publish_video_stream_frame(reason="video_ingest_disconnected")

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
