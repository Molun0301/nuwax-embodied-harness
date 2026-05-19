#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
本地日志转 TTS 守护程序。

职责：
1. 监听 nuwaxbot 主日志中的模型输出分片。
2. 在宿主机内直接调用本地 Doubao 双向流式客户端。
3. 不再经过本地 HTTP API 和 playback websocket 中转。
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
import re
import threading
import time
from typing import Dict, List, Optional, Any

from settings import APP_CONFIG, DoubaoConfig, TTSLogBridgeConfig
from .doubao_realtime_client import DoubaoRealtimeClient, speech_settings_from_log_config


LOGGER = logging.getLogger("log_tts_bridge")


@dataclass
class BridgeConfig:
    """日志转 TTS 的运行配置。"""

    enabled: bool = True
    log_path: str = "/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot/logs/main.log"
    require_root: bool = True
    speaker: str = ""
    resource_id: str = "seed-tts-1.0"
    uid: str = "nuwax_robot_bridge-tts"
    model: str = ""
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
    interrupt: bool = True
    explicit_language: str = "zh"
    poll_interval_sec: float = 0.2
    quiet_period_sec: float = 1.5
    min_text_length: int = 2
    flush_interval_sec: float = 0.35
    stream_min_chars: int = 8
    start_from_end: bool = True
    request_timeout_sec: float = 15.0
    start_retry_backoff_sec: float = 2.0
    max_start_failures_per_turn: int = 3


@dataclass
class SessionBuffer:
    """按 ACP 会话暂存模型输出文本和 TTS 状态。"""

    raw_text: str = ""
    spoken_text: str = ""
    last_chunk_at: float = 0.0
    last_flush_at: float = 0.0
    last_spoken_hash: str = ""
    tts_session_id: str = ""
    requested_tts_session_id: str = ""
    finish_requested: bool = False
    turn_index: int = 0
    start_failures: int = 0
    last_start_attempt_at: float = 0.0
    last_start_error: str = ""


def bridge_config_from_settings(config: TTSLogBridgeConfig) -> BridgeConfig:
    """把统一配置映射为桥接运行配置。"""

    return BridgeConfig(
        enabled=config.enabled,
        log_path=config.log_path,
        require_root=config.require_root,
        speaker=config.speaker,
        resource_id=config.resource_id,
        uid=config.uid,
        model=config.model,
        audio_format=config.audio_format,
        sample_rate=config.sample_rate,
        speech_rate=config.speech_rate,
        loudness_rate=config.loudness_rate,
        emotion=config.emotion,
        emotion_scale=config.emotion_scale,
        enable_subtitle=config.enable_subtitle,
        enable_timestamp=config.enable_timestamp,
        disable_markdown_filter=config.disable_markdown_filter,
        silence_duration_ms=config.silence_duration_ms,
        interrupt=config.interrupt,
        explicit_language=config.explicit_language,
        poll_interval_sec=config.poll_interval_sec,
        quiet_period_sec=config.quiet_period_sec,
        min_text_length=config.min_text_length,
        flush_interval_sec=config.flush_interval_sec,
        stream_min_chars=config.stream_min_chars,
        start_from_end=config.start_from_end,
        request_timeout_sec=config.request_timeout_sec,
        start_retry_backoff_sec=config.start_retry_backoff_sec,
        max_start_failures_per_turn=config.max_start_failures_per_turn,
    )


class LogTTSBridge:
    """监听日志并把回复持续送入本地 Doubao 流式客户端。"""

    def __init__(
        self,
        config: BridgeConfig,
        tts_client: DoubaoRealtimeClient,
        doubao_config: DoubaoConfig,
        stop_event: Optional[threading.Event] = None,
    ) -> None:
        self.config = config
        self.tts_client = tts_client
        self.doubao_config = doubao_config
        self._stop_event = stop_event or threading.Event()
        self._buffers = {}
        self._prompt_block_lines = None
        self._last_result_stop_reason = ""
        self._file = None
        self._inode = None
        self._position = 0

    def run(self) -> None:
        LOGGER.info(
            "日志转TTS守护程序已启动: log=%s speaker=%s mode=direct_local_stream",
            self.config.log_path,
            self.config.speaker,
        )
        while not self._stop_event.is_set():
            self._ensure_file_open()
            if self._file is None:
                time.sleep(self.config.poll_interval_sec)
                continue

            line = self._file.readline()
            if line:
                self._position = self._file.tell()
                self._process_line(line.rstrip("\n"))
                self._flush_sessions()
                continue

            self._request_finish_for_idle_sessions()
            self._flush_sessions()
            self._check_rotation()
            time.sleep(self.config.poll_interval_sec)
        self._close_file()

    def _ensure_file_open(self) -> None:
        path = Path(self.config.log_path).expanduser()
        if not path.exists():
            if self._file is not None:
                self._close_file()
            return
        if self._file is not None:
            return

        try:
            self._file = path.open("r", encoding="utf-8", errors="ignore")
        except PermissionError as exc:
            LOGGER.error("打开日志文件失败，权限不足: %s", exc)
            return

        stat = path.stat()
        self._inode = stat.st_ino
        if self.config.start_from_end:
            self._file.seek(0, os.SEEK_END)
            self._position = self._file.tell()
        else:
            self._position = 0
        LOGGER.info("开始监听日志文件: %s start_from_end=%s", path, self.config.start_from_end)

    def _close_file(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._inode = None
        self._position = 0

    def _check_rotation(self) -> None:
        path = Path(self.config.log_path).expanduser()
        if self._file is None or not path.exists():
            return
        stat = path.stat()
        if self._inode != stat.st_ino or stat.st_size < self._position:
            LOGGER.info("检测到日志轮转或截断，重新打开: %s", path)
            self._close_file()
            self._ensure_file_open()

    def _process_line(self, line: str) -> None:
        if self._prompt_block_lines is not None:
            self._prompt_block_lines.append(line)
            if line.strip() == "}":
                self._handle_prompt_completed_block(self._prompt_block_lines)
                self._prompt_block_lines = None
            return

        if "Prompt completed {" in line:
            self._prompt_block_lines = [line]
            return

        payload = self._extract_json_payload(line)
        if payload is None:
            return

        if payload.get("method") == "session/update":
            params = payload.get("params") or {}
            session_id = str(params.get("sessionId") or "").strip()
            update = params.get("update") or {}
            update_type = update.get("sessionUpdate")
            if update_type in {"agent_message_chunk", "agent_message"}:
                text = self._extract_update_text(update)
                if session_id and text:
                    self._append_chunk(session_id, text)
            return

        stop_reason = str((payload.get("result") or {}).get("stopReason") or "").strip()
        if stop_reason:
            self._last_result_stop_reason = stop_reason
            if stop_reason == "end_turn":
                self._request_finish_for_single_recent_session("acp_result_end_turn")

    def _extract_json_payload(self, line: str) -> Optional[Dict[str, Any]]:
        marker = "📥 "
        if marker not in line:
            return None
        start = line.find("{", line.find(marker))
        if start < 0:
            return None
        raw_json = line[start:].strip()
        try:
            value = json.loads(raw_json)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None

    def _extract_update_text(self, update: Dict[str, Any]) -> str:
        content = update.get("content")
        if isinstance(content, dict):
            if content.get("type") == "text":
                return str(content.get("text") or "")
        if isinstance(content, list):
            parts = []
            for item in content:
                if not isinstance(item, dict):
                    continue
                nested = item.get("content")
                if isinstance(nested, dict) and nested.get("type") == "text":
                    parts.append(str(nested.get("text") or ""))
            return "".join(parts)
        return ""

    def _append_chunk(self, session_id: str, text: str) -> None:
        if not text:
            return
        buffer = self._buffers.setdefault(session_id, SessionBuffer())
        if not buffer.requested_tts_session_id:
            buffer.turn_index += 1
            buffer.requested_tts_session_id = self._build_tts_session_id(session_id, buffer.turn_index)
        if not buffer.raw_text:
            buffer.start_failures = 0
            buffer.last_start_attempt_at = 0.0
            buffer.last_start_error = ""
        buffer.raw_text += text
        buffer.last_chunk_at = time.time()
        buffer.finish_requested = False

    def _handle_prompt_completed_block(self, lines: List[str]) -> None:
        block = "\n".join(lines)
        session_match = re.search(r"sessionId:\s*'([^']+)'", block)
        reason_match = re.search(r"stopReason:\s*'([^']+)'", block)
        if not session_match or not reason_match:
            return
        session_id = session_match.group(1).strip()
        stop_reason = reason_match.group(1).strip()
        if stop_reason == "end_turn":
            self._request_finish_for_session(session_id, "prompt_completed_end_turn")

    def _request_finish_for_single_recent_session(self, source: str) -> None:
        recent = [
            session_id
            for session_id, buffer in self._buffers.items()
            if buffer.raw_text and (time.time() - buffer.last_chunk_at) <= 5.0
        ]
        if len(recent) == 1:
            self._request_finish_for_session(recent[0], source)

    def _request_finish_for_idle_sessions(self) -> None:
        now = time.time()
        for session_id, buffer in list(self._buffers.items()):
            if not buffer.raw_text:
                continue
            if (now - buffer.last_chunk_at) >= self.config.quiet_period_sec:
                self._request_finish_for_session(session_id, "quiet_timeout")

    def _request_finish_for_session(self, session_id: str, source: str) -> None:
        buffer = self._buffers.setdefault(session_id, SessionBuffer())
        if not buffer.raw_text:
            return
        if not buffer.finish_requested:
            LOGGER.debug("检测到一轮模型回复结束，准备收口TTS流: session=%s source=%s", session_id, source)
        buffer.finish_requested = True

    def _flush_sessions(self) -> None:
        for session_id, buffer in list(self._buffers.items()):
            self._flush_session_delta(session_id, buffer, force=buffer.finish_requested)
            if buffer.finish_requested:
                self._try_finish_session(session_id, buffer)

    def _flush_session_delta(self, session_id: str, buffer: SessionBuffer, force: bool) -> None:
        normalized_text = self._normalize_for_speech(buffer.raw_text)
        if len(normalized_text) < self.config.min_text_length and not force:
            return

        delta = self._compute_incremental_delta(buffer.spoken_text, normalized_text)
        if not delta:
            return

        now = time.time()
        should_send = force
        if not should_send and len(delta) >= self.config.stream_min_chars:
            should_send = True
        if not should_send and self._contains_flush_punctuation(delta):
            should_send = True
        if not should_send and (now - buffer.last_flush_at) >= self.config.flush_interval_sec:
            should_send = True
        if not should_send:
            return

        if not self._ensure_tts_stream(session_id, buffer):
            return
        if not self._append_stream_text(session_id, buffer, delta):
            return

        buffer.spoken_text = normalized_text
        buffer.last_flush_at = now

    def _try_finish_session(self, session_id: str, buffer: SessionBuffer) -> None:
        normalized_text = self._normalize_for_speech(buffer.raw_text)
        if normalized_text and buffer.spoken_text != normalized_text:
            return

        if buffer.tts_session_id:
            if not self._finish_stream(session_id, buffer.tts_session_id):
                buffer.tts_session_id = ""
                return

        text_hash = hashlib.sha1(buffer.spoken_text.encode("utf-8")).hexdigest() if buffer.spoken_text else ""
        if text_hash and text_hash != buffer.last_spoken_hash:
            buffer.last_spoken_hash = text_hash

        LOGGER.debug(
            "本轮日志TTS已完成: session=%s tts_session=%s chars=%s",
            session_id,
            buffer.tts_session_id or "-",
            len(buffer.spoken_text),
        )
        buffer.raw_text = ""
        buffer.spoken_text = ""
        buffer.last_chunk_at = 0.0
        buffer.last_flush_at = 0.0
        buffer.tts_session_id = ""
        buffer.requested_tts_session_id = ""
        buffer.finish_requested = False
        buffer.start_failures = 0
        buffer.last_start_attempt_at = 0.0
        buffer.last_start_error = ""

    def _normalize_for_speech(self, text: str) -> str:
        text = text.replace("\r\n", "\n")
        text = re.sub(r"```.*?```", " ", text, flags=re.DOTALL)
        text = re.sub(r"`([^`]+)`", r"\1", text)
        text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)
        text = re.sub(r"https?://\S+", " ", text)
        text = re.sub(r"^#+\s*", "", text, flags=re.MULTILINE)
        text = re.sub(r"^\s*[-*+]\s+", "", text, flags=re.MULTILINE)
        text = re.sub(r"^\s*\d+\.\s+", "", text, flags=re.MULTILINE)
        text = re.sub(r"(\*\*|__)(.*?)\1", r"\2", text)
        text = re.sub(r"(?<!\*)\*(?!\*)([^*]+)(?<!\*)\*(?!\*)", r"\1", text)
        text = re.sub(r"(?<!_)_(?!_)([^_]+)(?<!_)_(?!_)", r"\1", text)

        clean_lines = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            lower_line = line.lower()
            if any(
                token in lower_line
                for token in (
                    "session_id",
                    "playback_ws_url",
                    '"session_id"',
                    '"playback_ws_url"',
                )
            ):
                continue
            clean_lines.append(line)

        text = "\n".join(clean_lines)
        text = re.sub(r"[🐕🔊✅❌⚠️]", " ", text)
        text = re.sub(r"\s+", " ", text).strip()
        return text

    def _build_speech_settings(self):
        return speech_settings_from_log_config(
            TTSLogBridgeConfig(
                enabled=self.config.enabled,
                log_path=self.config.log_path,
                require_root=self.config.require_root,
                speaker=self.config.speaker,
                resource_id=self.config.resource_id,
                uid=self.config.uid,
                model=self.config.model,
                audio_format=self.config.audio_format,
                sample_rate=self.config.sample_rate,
                speech_rate=self.config.speech_rate,
                loudness_rate=self.config.loudness_rate,
                emotion=self.config.emotion,
                emotion_scale=self.config.emotion_scale,
                enable_subtitle=self.config.enable_subtitle,
                enable_timestamp=self.config.enable_timestamp,
                disable_markdown_filter=self.config.disable_markdown_filter,
                silence_duration_ms=self.config.silence_duration_ms,
                interrupt=self.config.interrupt,
                explicit_language=self.config.explicit_language,
                start_from_end=self.config.start_from_end,
                poll_interval_sec=self.config.poll_interval_sec,
                quiet_period_sec=self.config.quiet_period_sec,
                flush_interval_sec=self.config.flush_interval_sec,
                stream_min_chars=self.config.stream_min_chars,
                min_text_length=self.config.min_text_length,
                request_timeout_sec=self.config.request_timeout_sec,
            ),
            self.doubao_config,
        )

    def _ensure_tts_stream(self, session_id: str, buffer: SessionBuffer) -> bool:
        if buffer.tts_session_id:
            return True
        now = time.time()
        if buffer.last_start_attempt_at and (now - buffer.last_start_attempt_at) < self.config.start_retry_backoff_sec:
            return False
        try:
            settings = self._build_speech_settings()
            buffer.last_start_attempt_at = now
            buffer.tts_session_id = self.tts_client.start_stream(
                settings,
                session_id=buffer.requested_tts_session_id
                or self._build_tts_session_id(session_id, buffer.turn_index or 1),
            )
        except Exception as exc:
            buffer.start_failures += 1
            buffer.last_start_error = self._format_exception(exc)
            LOGGER.error(
                "创建本地 Doubao 会话失败: session=%s requested_tts_session=%s failures=%s error=%s",
                session_id,
                buffer.requested_tts_session_id or "-",
                buffer.start_failures,
                buffer.last_start_error,
            )
            if buffer.finish_requested and buffer.start_failures >= self.config.max_start_failures_per_turn:
                LOGGER.warning(
                    "当前轮次TTS启动连续失败，丢弃过期文本避免堆积: session=%s requested_tts_session=%s chars=%s last_error=%s",
                    session_id,
                    buffer.requested_tts_session_id or "-",
                    len(self._normalize_for_speech(buffer.raw_text)),
                    buffer.last_start_error or "-",
                )
                self._reset_buffer_after_drop(buffer)
            return False

        buffer.start_failures = 0
        buffer.last_start_error = ""
        LOGGER.debug("TTS流已创建: session=%s tts_session=%s", session_id, buffer.tts_session_id)
        return True

    def _append_stream_text(self, session_id: str, buffer: SessionBuffer, text: str) -> bool:
        if not buffer.tts_session_id:
            return False
        try:
            ok = self.tts_client.append_stream(buffer.tts_session_id, text)
        except Exception as exc:
            LOGGER.error(
                "追加本地 Doubao 文本失败: session=%s tts_session=%s error=%s",
                session_id,
                buffer.tts_session_id,
                exc,
            )
            buffer.tts_session_id = ""
            return False
        if not ok:
            LOGGER.error("追加本地 Doubao 文本失败: session=%s tts_session=%s", session_id, buffer.tts_session_id)
            buffer.tts_session_id = ""
            return False
        return True

    def _finish_stream(self, session_id: str, tts_session_id: str) -> bool:
        try:
            ok = self.tts_client.finish_stream(tts_session_id)
        except Exception as exc:
            LOGGER.error("结束本地 Doubao 会话失败: session=%s tts_session=%s error=%s", session_id, tts_session_id, exc)
            return False
        if not ok:
            LOGGER.error("结束本地 Doubao 会话失败: session=%s tts_session=%s", session_id, tts_session_id)
            return False
        LOGGER.debug("TTS流已结束: session=%s tts_session=%s", session_id, tts_session_id)
        return True

    def _compute_incremental_delta(self, previous: str, current: str) -> str:
        prefix_len = 0
        max_len = min(len(previous), len(current))
        while prefix_len < max_len and previous[prefix_len] == current[prefix_len]:
            prefix_len += 1
        return current[prefix_len:]

    def _contains_flush_punctuation(self, text: str) -> bool:
        return any(char in text for char in "。！？!?；;\n")

    def _build_tts_session_id(self, session_id: str, turn_index: int) -> str:
        digest = hashlib.sha1(session_id.encode("utf-8")).hexdigest()[:12]
        return "logtts-%s-%s" % (digest, max(1, int(turn_index)))

    def _reset_buffer_after_drop(self, buffer: SessionBuffer) -> None:
        buffer.raw_text = ""
        buffer.spoken_text = ""
        buffer.last_chunk_at = 0.0
        buffer.last_flush_at = 0.0
        buffer.tts_session_id = ""
        buffer.requested_tts_session_id = ""
        buffer.finish_requested = False
        buffer.start_failures = 0
        buffer.last_start_attempt_at = 0.0
        buffer.last_start_error = ""

    def _format_exception(self, exc: Exception) -> str:
        text = str(exc).strip()
        if text:
            return "%s: %s" % (exc.__class__.__name__, text)
        return exc.__class__.__name__


class LogTTSBridgeService:
    """线程化的日志转 TTS 服务，供 go2_proxy_server 托管。"""

    def __init__(
        self,
        config: BridgeConfig,
        tts_client: DoubaoRealtimeClient,
        doubao_config: DoubaoConfig,
    ) -> None:
        self.config = config
        self.tts_client = tts_client
        self.doubao_config = doubao_config
        self._stop_event = threading.Event()
        self._thread = None
        self._worker = LogTTSBridge(config, tts_client, doubao_config, stop_event=self._stop_event)
        self._last_error = ""

    def start(self) -> bool:
        if not self.config.enabled:
            LOGGER.info("日志转TTS桥接已禁用")
            return False
        if self._thread is not None and self._thread.is_alive():
            LOGGER.info("日志转TTS桥接已在运行")
            return False
        if self.config.require_root and hasattr(os, "geteuid") and os.geteuid() != 0:
            raise PermissionError("日志转TTS桥接需要 root 运行，当前无法读取 %s" % self.config.log_path)
        if not self.config.speaker:
            raise ValueError("日志转TTS桥接缺少 speaker，请配置 TTS_LOG_BRIDGE_SPEAKER")

        self.tts_client.start()
        self._stop_event.clear()
        self._last_error = ""

        def _runner() -> None:
            try:
                self._worker.run()
            except Exception as exc:
                self._last_error = str(exc)
                LOGGER.exception("日志转TTS桥接线程异常退出: %s", exc)

        self._thread = threading.Thread(
            target=_runner,
            name="LogTTSBridge",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("日志转TTS桥接已启动")
        return True

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        LOGGER.info("日志转TTS桥接已停止")

    def status(self) -> Dict[str, Any]:
        return {
            "enabled": self.config.enabled,
            "running": self._thread is not None and self._thread.is_alive(),
            "log_path": self.config.log_path,
            "require_root": self.config.require_root,
            "root_ok": (not self.config.require_root) or (not hasattr(os, "geteuid")) or os.geteuid() == 0,
            "speaker": self.config.speaker,
            "resource_id": self.config.resource_id,
            "last_error": self._last_error,
            "mode": "direct_local_stream",
        }


def build_log_tts_bridge_from_config(
    config: Optional[TTSLogBridgeConfig],
    tts_client: DoubaoRealtimeClient,
    doubao_config: Optional[DoubaoConfig] = None,
) -> LogTTSBridgeService:
    """从统一配置创建日志桥接服务。"""

    actual_config = config or APP_CONFIG.log_tts
    actual_doubao = doubao_config or APP_CONFIG.doubao
    return LogTTSBridgeService(
        bridge_config_from_settings(actual_config),
        tts_client,
        actual_doubao,
    )
