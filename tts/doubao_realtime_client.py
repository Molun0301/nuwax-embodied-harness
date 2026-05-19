from __future__ import annotations

import asyncio
from concurrent.futures import TimeoutError as FutureTimeoutError
import functools
import inspect
import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

from settings import APP_CONFIG, DoubaoConfig, TTSLogBridgeConfig
from .doubao_ws import (
    TTS_BIDIRECTIONAL_URL,
    DoubaoProtocolError,
    Event,
    build_cancel_session_frame,
    build_finish_connection_frame,
    build_finish_session_frame,
    build_header_candidates,
    build_start_connection_frame,
    build_start_payload,
    build_start_session_frame,
    build_task_request_frame,
    build_text_payload,
    parse_frame,
)


LOGGER = logging.getLogger("doubao_tts")


try:
    import websockets
except ImportError:
    websockets = None


@dataclass
class SpeechSettings:
    """单次 TTS 会话的语音参数。"""

    speaker: str
    resource_id: str
    uid: str
    audio_format: str = "pcm"
    sample_rate: int = 24000
    speech_rate: int = 0
    loudness_rate: int = 0
    emotion: str = ""
    emotion_scale: int = 4
    enable_subtitle: bool = False
    enable_timestamp: bool = False
    disable_markdown_filter: bool = True
    silence_duration_ms: int = 0
    explicit_language: str = ""
    model: str = ""
    interrupt: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionCommand:
    kind: str
    text: str = ""
    reason: str = ""


@dataclass
class SessionState:
    session_id: str
    settings: SpeechSettings
    queue: asyncio.Queue
    created_at: float = field(default_factory=time.time)
    last_frame_at: float = field(default_factory=time.time)
    last_audio_at: float = 0.0
    playback_started: bool = False
    connection_started: asyncio.Event = field(default_factory=asyncio.Event)
    session_started: asyncio.Event = field(default_factory=asyncio.Event)
    session_finished: asyncio.Event = field(default_factory=asyncio.Event)
    task: Optional[asyncio.Task] = None


def speech_settings_from_log_config(
    config: TTSLogBridgeConfig,
    credentials: Optional[DoubaoConfig] = None,
) -> SpeechSettings:
    """将日志桥接配置映射为豆包会话参数。"""

    doubao = credentials or APP_CONFIG.doubao
    speaker = (config.speaker or doubao.default_speaker).strip()
    if not speaker:
        raise ValueError("缺少 speaker，请设置 TTS_LOG_BRIDGE_SPEAKER 或 DOUBAO_DEFAULT_SPEAKER")

    return SpeechSettings(
        speaker=speaker,
        resource_id=(config.resource_id or doubao.resource_id).strip() or doubao.resource_id,
        uid=(config.uid or doubao.default_uid).strip() or doubao.default_uid,
        audio_format=(config.audio_format or doubao.default_audio_format).strip() or doubao.default_audio_format,
        sample_rate=int(config.sample_rate or doubao.default_sample_rate),
        speech_rate=int(config.speech_rate),
        loudness_rate=int(config.loudness_rate),
        emotion=config.emotion.strip(),
        emotion_scale=int(config.emotion_scale),
        enable_subtitle=bool(config.enable_subtitle),
        enable_timestamp=bool(config.enable_timestamp),
        disable_markdown_filter=bool(config.disable_markdown_filter),
        silence_duration_ms=int(config.silence_duration_ms),
        explicit_language=config.explicit_language.strip(),
        model=(config.model or doubao.default_model).strip(),
        interrupt=bool(config.interrupt),
        metadata={},
    )


class DoubaoRealtimeClient:
    """本地豆包双向流式客户端。

    这个类直接在宿主机与豆包上游建立 WebSocket，会话生命周期全部在本进程内完成，
    不再经过本地 HTTP 和 playback websocket 中转。
    """

    def __init__(self, credentials: DoubaoConfig, playback_manager: Any) -> None:
        self.credentials = credentials
        self.playback_manager = playback_manager
        self.logger = LOGGER
        self.sessions = {}
        self.active_session_id = ""
        self._loop = None
        self._thread = None
        self._ready = threading.Event()
        self._stop_event = threading.Event()
        self._last_error = ""

    def start(self) -> bool:
        if websockets is None:
            raise RuntimeError("缺少依赖 websockets，请安装后再使用本地 Doubao 流式客户端")
        if not self.credentials.app_id or not self.credentials.access_key:
            raise ValueError("缺少豆包鉴权配置，请检查 DOUBAO_APP_ID 和 DOUBAO_ACCESS_KEY")
        if self._thread is not None and self._thread.is_alive():
            return False

        self._ready.clear()
        self._stop_event.clear()
        self._last_error = ""

        def _runner() -> None:
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)
            self._ready.set()
            try:
                loop.run_forever()
            finally:
                pending = asyncio.all_tasks(loop)
                for task in pending:
                    task.cancel()
                if pending:
                    loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                loop.close()

        self._thread = threading.Thread(
            target=_runner,
            name="DoubaoRealtimeClient",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(timeout=3.0):
            raise RuntimeError("启动本地 Doubao 流式客户端超时")
        self.logger.info("本地 Doubao 流式客户端已启动")
        return True

    def stop(self, timeout: float = 5.0) -> bool:
        loop = self._loop
        thread = self._thread
        if loop is None:
            return False

        self._stop_event.set()
        try:
            future = asyncio.run_coroutine_threadsafe(self._shutdown_async(), loop)
            future.result(timeout=timeout)
        except Exception:
            pass

        try:
            loop.call_soon_threadsafe(loop.stop)
        except Exception:
            pass

        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)

        self._thread = None
        self._loop = None
        self.logger.info("本地 Doubao 流式客户端已停止")
        return True

    def status(self) -> Dict[str, Any]:
        return {
            "running": self._thread is not None and self._thread.is_alive(),
            "active_session_id": self.active_session_id,
            "session_count": len(self.sessions),
            "last_error": self._last_error,
            "has_credentials": bool(self.credentials.app_id and self.credentials.access_key),
            "default_resource_id": self.credentials.resource_id,
            "default_speaker": self.credentials.default_speaker,
        }

    def start_stream(self, settings: SpeechSettings, session_id: Optional[str] = None) -> str:
        self._ensure_started()
        future = asyncio.run_coroutine_threadsafe(
            self._start_stream_async(settings, session_id=session_id),
            self._loop,
        )
        try:
            return future.result(timeout=10.0)
        except FutureTimeoutError as exc:
            future.cancel()
            self._last_error = "start_stream timed out"
            raise TimeoutError("启动本地 Doubao 会话超时") from exc

    def append_stream(self, session_id: str, text: str) -> bool:
        self._ensure_started()
        future = asyncio.run_coroutine_threadsafe(
            self._append_stream_async(session_id, text),
            self._loop,
        )
        try:
            return bool(future.result(timeout=10.0))
        except FutureTimeoutError as exc:
            future.cancel()
            self._last_error = "append_stream timed out"
            raise TimeoutError("追加本地 Doubao 文本超时") from exc

    def finish_stream(self, session_id: str) -> bool:
        self._ensure_started()
        future = asyncio.run_coroutine_threadsafe(
            self._finish_stream_async(session_id),
            self._loop,
        )
        try:
            return bool(future.result(timeout=10.0))
        except FutureTimeoutError as exc:
            future.cancel()
            self._last_error = "finish_stream timed out"
            raise TimeoutError("结束本地 Doubao 会话超时") from exc

    def cancel_stream(self, session_id: str, reason: str = "user-cancelled") -> bool:
        self._ensure_started()
        future = asyncio.run_coroutine_threadsafe(
            self._cancel_stream_async(session_id, reason),
            self._loop,
        )
        try:
            return bool(future.result(timeout=10.0))
        except FutureTimeoutError as exc:
            future.cancel()
            self._last_error = "cancel_stream timed out"
            raise TimeoutError("取消本地 Doubao 会话超时") from exc

    def _ensure_started(self) -> None:
        if self._loop is None or self._thread is None or not self._thread.is_alive():
            self.start()

    async def _run_in_thread(self, func: Any, *args: Any, **kwargs: Any) -> Any:
        loop = asyncio.get_running_loop()
        call = functools.partial(func, *args, **kwargs)
        return await loop.run_in_executor(None, call)

    async def _shutdown_async(self) -> None:
        for session_id in list(self.sessions.keys()):
            try:
                await self._cancel_stream_async(session_id, "service-stopping")
            except Exception:
                pass
        for state in list(self.sessions.values()):
            if state.task is not None:
                state.task.cancel()
        await asyncio.sleep(0)

    async def _start_stream_async(
        self,
        settings: SpeechSettings,
        session_id: Optional[str] = None,
    ) -> str:
        new_session_id = (session_id or "").strip() or uuid.uuid4().hex
        existing = self.sessions.get(new_session_id)
        if existing is not None:
            if existing.task is None or not existing.task.done():
                self.active_session_id = new_session_id
                self.logger.debug("复用已存在的本地 Doubao 会话: session=%s", new_session_id)
                return new_session_id
            self.sessions.pop(new_session_id, None)

        if self.active_session_id and self.active_session_id != new_session_id:
            if settings.interrupt:
                await self._cancel_stream_async(self.active_session_id, "replaced-by-new-session")
            else:
                raise ValueError("当前已有活跃TTS会话: %s" % self.active_session_id)

        state = SessionState(
            session_id=new_session_id,
            settings=settings,
            queue=asyncio.Queue(),
        )
        self.sessions[new_session_id] = state
        self.active_session_id = new_session_id
        state.task = asyncio.create_task(
            self._run_session(state),
            name="doubao-session-%s" % new_session_id,
        )
        self.logger.debug(
            "已创建本地 Doubao 会话: session=%s speaker=%s resource_id=%s",
            new_session_id,
            settings.speaker,
            settings.resource_id,
        )
        return new_session_id

    async def _append_stream_async(self, session_id: str, text: str) -> bool:
        state = self.sessions.get(session_id)
        if state is None:
            return False
        if not text.strip():
            return False
        await state.queue.put(SessionCommand(kind="text", text=text))
        return True

    async def _finish_stream_async(self, session_id: str) -> bool:
        state = self.sessions.get(session_id)
        if state is None:
            return False
        await state.queue.put(SessionCommand(kind="finish"))
        return True

    async def _cancel_stream_async(self, session_id: str, reason: str) -> bool:
        state = self.sessions.get(session_id)
        if state is None:
            return False
        self._drop_pending_commands(state)
        await state.queue.put(SessionCommand(kind="cancel", reason=reason))
        asyncio.create_task(
            self._interrupt_playback_async(session_id, reason),
            name="doubao-interrupt-%s" % session_id,
        )
        if self.active_session_id == session_id:
            self.active_session_id = ""
        return True

    async def _interrupt_playback_async(self, session_id: str, reason: str) -> None:
        try:
            await self._run_in_thread(self.playback_manager.interrupt_session, session_id, reason)
        except Exception as exc:
            self.logger.warning(
                "异步中断本地播放失败: session=%s reason=%s error=%s",
                session_id,
                reason,
                exc,
            )

    async def _run_session(self, state: SessionState) -> None:
        try:
            last_exc = None
            for resource_id in self._resource_id_candidates(state.settings.resource_id):
                request_id = uuid.uuid4().hex
                header_candidates = build_header_candidates(
                    self.credentials,
                    request_id,
                    resource_id=resource_id,
                )

                for header_mode, headers in header_candidates:
                    try:
                        self.logger.debug(
                            "连接豆包上游: session=%s auth_mode=%s resource_id=%s",
                            state.session_id,
                            header_mode,
                            resource_id,
                        )
                        await self._run_session_once(state, headers)
                        return
                    except Exception as exc:
                        last_exc = exc
                        status_code, response_headers = self._extract_ws_error_details(exc)
                        tt_logid = response_headers.get("x-tt-logid", "")
                        if status_code in {401, 403}:
                            self.logger.warning(
                                "豆包鉴权失败: session=%s auth_mode=%s resource_id=%s status=%s x-tt-logid=%s",
                                state.session_id,
                                header_mode,
                                resource_id,
                                status_code,
                                tt_logid or "-",
                            )
                            continue
                        raise

            if last_exc is not None:
                raise last_exc
        except Exception as exc:
            self._last_error = str(exc)
            self.logger.exception("本地 Doubao 会话失败: session=%s error=%s", state.session_id, exc)
            await self._run_in_thread(self.playback_manager.handle_error, state.session_id, str(exc))
        finally:
            await self._cleanup_session(state)

    async def _run_session_once(self, state: SessionState, headers: Dict[str, str]) -> None:
        async with websockets.connect(
            TTS_BIDIRECTIONAL_URL,
            **self._doubao_connect_kwargs(headers),
        ) as websocket:
            self.logger.debug("豆包上游连接成功: session=%s", state.session_id)
            receiver_task = asyncio.create_task(
                self._recv_loop(state, websocket),
                name="doubao-recv-%s" % state.session_id,
            )

            await websocket.send(build_start_connection_frame())
            await self._wait_for_event_or_receiver(
                state.connection_started,
                receiver_task,
                timeout=15,
                label="connection_started",
            )

            await websocket.send(
                build_start_session_frame(state.session_id, build_start_payload(state.settings))
            )
            await self._wait_for_event_or_receiver(
                state.session_started,
                receiver_task,
                timeout=15,
                label="session_started",
            )

            while True:
                command = await state.queue.get()
                if command.kind == "text":
                    await self._ensure_playback_started(state)
                    self.logger.debug(
                        "发送 TaskRequest: session=%s chars=%s",
                        state.session_id,
                        len(command.text),
                    )
                    await self._send_frame(
                        websocket,
                        state.session_id,
                        "TaskRequest",
                        build_task_request_frame(
                            state.session_id,
                            build_text_payload(command.text, state.settings),
                        ),
                    )
                elif command.kind == "finish":
                    self.logger.debug("发送 FinishSession: session=%s", state.session_id)
                    await self._send_frame(
                        websocket,
                        state.session_id,
                        "FinishSession",
                        build_finish_session_frame(state.session_id),
                    )
                    break
                elif command.kind == "cancel":
                    self.logger.debug("发送 CancelSession: session=%s", state.session_id)
                    await self._send_frame(
                        websocket,
                        state.session_id,
                        "CancelSession",
                        build_cancel_session_frame(state.session_id, command.reason)
                    )
                    break

            await self._wait_for_session_finish(state, receiver_task)

            try:
                await websocket.send(build_finish_connection_frame())
            except Exception as exc:
                self.logger.warning(
                    "发送 FinishConnection 失败，忽略并继续清理: session=%s error=%s",
                    state.session_id,
                    exc,
                )

            await asyncio.gather(receiver_task, return_exceptions=True)

    async def _wait_for_event_or_receiver(
        self,
        event: asyncio.Event,
        receiver_task: asyncio.Task,
        timeout: float,
        label: str,
    ) -> None:
        event_task = asyncio.create_task(event.wait(), name="wait-%s" % label)
        try:
            done, _ = await asyncio.wait(
                {event_task, receiver_task},
                timeout=timeout,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if receiver_task in done:
                await receiver_task
            if event_task in done and event.is_set():
                return
            raise TimeoutError("Timed out waiting for %s" % label)
        finally:
            if not event_task.done():
                event_task.cancel()

    def _doubao_connect_kwargs(self, headers: Dict[str, str]) -> Dict[str, Any]:
        params = inspect.signature(websockets.connect).parameters
        kwargs = {
            "max_size": None,
            # 豆包上游在长时间流式会话中偶发对 websockets 客户端 ping 不稳定，
            # 这里关闭库内 keepalive，避免本地 1011 ping timeout 抢先把可用会话杀掉。
            "ping_interval": None,
            "ping_timeout": None,
        }
        if "extra_headers" in params:
            kwargs["extra_headers"] = headers
        elif "additional_headers" in params:
            kwargs["additional_headers"] = headers
        else:
            raise RuntimeError("当前 websockets.connect 不支持额外请求头参数")
        return kwargs

    def _extract_ws_error_details(self, exc: Exception) -> Tuple[Optional[int], Dict[str, str]]:
        status_code = getattr(exc, "status_code", None)
        headers_obj = getattr(exc, "headers", None)

        response = getattr(exc, "response", None)
        if response is not None:
            status_code = getattr(response, "status_code", status_code)
            if status_code is None:
                status_code = getattr(response, "status", status_code)
            headers_obj = getattr(response, "headers", headers_obj)

        headers = {}
        if headers_obj is None:
            return status_code, headers

        raw_items = getattr(headers_obj, "raw_items", None)
        if callable(raw_items):
            try:
                return status_code, {str(k).lower(): str(v) for k, v in raw_items()}
            except Exception:
                pass

        items = getattr(headers_obj, "items", None)
        if callable(items):
            try:
                return status_code, {str(k).lower(): str(v) for k, v in items()}
            except Exception:
                pass

        return status_code, headers

    def _resource_id_candidates(self, resource_id: str) -> List[str]:
        normalized = resource_id.strip()
        if not normalized:
            return ["seed-tts-1.0", "volc.service_type.10029"]
        if normalized == "seed-tts-2.0":
            normalized = "seed-tts-1.0"
        candidates = [normalized]
        aliases = {
            "seed-tts-1.0": "volc.service_type.10029",
            "volc.service_type.10029": "seed-tts-1.0",
        }
        alias = aliases.get(normalized)
        if alias:
            candidates.append(alias)
        return candidates

    async def _recv_loop(self, state: SessionState, websocket: Any) -> None:
        async for frame_bytes in websocket:
            if not isinstance(frame_bytes, (bytes, bytearray)):
                continue

            try:
                frame = parse_frame(bytes(frame_bytes))
            except Exception:
                preview = bytes(frame_bytes[:64]).hex()
                self.logger.exception(
                    "解析豆包上游帧失败: session=%s len=%s first64=%s",
                    state.session_id,
                    len(frame_bytes),
                    preview,
                )
                raise

            if frame.is_error:
                state.session_finished.set()
                message = frame.message or "Doubao websocket returned an error frame"
                if frame.code is not None:
                    message = "%s (code=%s)" % (message, frame.code)
                raise DoubaoProtocolError(message)

            event = frame.event
            state.last_frame_at = time.time()

            if event == Event.CONNECTION_STARTED:
                state.connection_started.set()
                self.logger.debug("豆包连接已启动: session=%s", state.session_id)
                continue

            if event == Event.SESSION_STARTED:
                state.session_started.set()
                self.logger.debug("豆包会话已启动: session=%s", state.session_id)
                continue

            if event == Event.TTS_RESPONSE:
                audio = frame.audio_bytes()
                if audio:
                    state.last_audio_at = time.time()
                    self.playback_manager.write_audio(state.session_id, audio)
                continue

            if event == Event.TTS_SENTENCE_END:
                self.playback_manager.handle_sentence_end(state.session_id, frame.json_payload or {})
                continue

            if event == Event.TTS_SUBTITLE:
                self.playback_manager.handle_subtitle(state.session_id, frame.json_payload or {})
                continue

            if event in (Event.SESSION_FINISHED, Event.SESSION_CANCELLED):
                state.session_finished.set()
                await self._run_in_thread(
                    self.playback_manager.finish_session,
                    state.session_id,
                    frame.json_payload or {},
                )
                continue

            if event in (Event.SESSION_FAILED, Event.CONNECTION_FAILED):
                state.session_finished.set()
                message = frame.message or "Doubao websocket returned a failure event"
                raise DoubaoProtocolError(message)

    async def _cleanup_session(self, state: SessionState) -> None:
        self.sessions.pop(state.session_id, None)
        if self.active_session_id == state.session_id:
            self.active_session_id = ""

    async def _ensure_playback_started(self, state: SessionState) -> None:
        if state.playback_started:
            return
        await self._run_in_thread(
            self.playback_manager.begin_session,
            session_id=state.session_id,
            audio_format=state.settings.audio_format,
            sample_rate=state.settings.sample_rate,
            channels=1,
            metadata=state.settings.metadata,
        )
        state.playback_started = True

    def _drop_pending_commands(self, state: SessionState) -> None:
        while True:
            try:
                state.queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    async def _send_frame(
        self,
        websocket: Any,
        session_id: str,
        action: str,
        frame: bytes,
        timeout: float = 10.0,
    ) -> None:
        try:
            await asyncio.wait_for(websocket.send(frame), timeout=timeout)
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                "%s 发送超时: session=%s timeout=%ss" % (action, session_id, timeout)
            ) from exc

    async def _wait_for_session_finish(
        self,
        state: SessionState,
        receiver_task: asyncio.Task,
        timeout: float = 20.0,
        idle_timeout: float = 3.0,
    ) -> None:
        deadline = time.monotonic() + timeout
        while True:
            if state.session_finished.is_set():
                return
            if receiver_task.done():
                await receiver_task
            now = time.monotonic()
            if state.last_audio_at and (time.time() - state.last_audio_at) >= idle_timeout:
                await self._force_finish_playback(
                    state,
                    "audio_idle_after_finish",
                    extra={"idle_timeout_sec": idle_timeout},
                )
                return
            if now >= deadline:
                await self._force_finish_playback(state, "session_finished_timeout")
                return
            await asyncio.sleep(0.2)

    async def _force_finish_playback(
        self,
        state: SessionState,
        reason: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload = {"forced_finish": True, "reason": reason}
        if extra:
            payload.update(extra)
        self.logger.warning(
            "未等到豆包 SessionFinished，按本地收口继续结束连接: session=%s reason=%s",
            state.session_id,
            reason,
        )
        try:
            await self._run_in_thread(
                self.playback_manager.finish_session,
                state.session_id,
                payload,
            )
        except Exception as exc:
            self.logger.warning(
                "本地强制收口播放失败: session=%s error=%s",
                state.session_id,
                exc,
            )
        state.session_finished.set()


def build_doubao_realtime_client(
    playback_manager: Any,
    config: Optional[DoubaoConfig] = None,
) -> DoubaoRealtimeClient:
    """从统一配置创建本地 Doubao 流式客户端。"""

    return DoubaoRealtimeClient(config or APP_CONFIG.doubao, playback_manager)
