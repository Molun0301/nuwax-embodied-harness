#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Go2 机身扬声器 WebRTC 音频客户端。

说明：
1. 这条链路面向 Go2 机身扬声器，而不是 Jetson 本地声卡。
2. 依赖 unitree_webrtc_connect，可选安装；缺失时会给出清晰错误。
"""

from __future__ import annotations

import asyncio
import base64
from concurrent.futures import TimeoutError as FutureTimeoutError
import hashlib
import json
import logging
import os
from pathlib import Path
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Any


logger = logging.getLogger(__name__)


AUDIO_API = {
    "GET_AUDIO_LIST": 1001,
    "SELECT_START_PLAY": 1002,
    "PAUSE": 1003,
    "UNSUSPEND": 1004,
    "SET_PLAY_MODE": 1007,
    "UPLOAD_AUDIO_FILE": 2001,
    "ENTER_MEGAPHONE": 4001,
    "EXIT_MEGAPHONE": 4002,
    "UPLOAD_MEGAPHONE": 4003,
}

PLAY_MODES = {
    "NO_CYCLE": "no_cycle",
}


class RobotWebRTCError(RuntimeError):
    """机器人扬声器 WebRTC 链路错误。"""


class RobotWebRTCAudioHubClient:
    """使用 Unitree WebRTC Audio Hub 向机器人扬声器上传并播放音频。"""

    def __init__(
        self,
        robot_ip: str,
        connection_mode: str = "ai",
        python_path: str = "",
        connect_timeout: float = 15.0,
        request_timeout: float = 20.0,
        upload_chunk_size: int = 61440,
        megaphone_chunk_size: int = 16384,
    ) -> None:
        self.robot_ip = robot_ip.strip()
        self.connection_mode = connection_mode
        self.python_path = python_path.strip()
        self.connect_timeout = float(connect_timeout)
        self.request_timeout = float(request_timeout)
        self.upload_chunk_size = int(upload_chunk_size)
        self.megaphone_chunk_size = int(megaphone_chunk_size)

        self._loop = None
        self._thread = None
        self._ready = threading.Event()
        self._stop_event = threading.Event()
        self._conn = None
        self._rtc_topic = None
        self._connected = False
        self._last_error = ""
        self._last_uploaded_name = ""
        self._megaphone_active = False
        self._megaphone_block_index = 0
        self._megaphone_started_at = 0.0
        self._megaphone_sample_rate = 0
        self._megaphone_channels = 1
        self._megaphone_audio_format = "pcm"
        self._megaphone_streamed_bytes = 0

    def status(self) -> Dict[str, Any]:
        return {
            "connected": self._connected,
            "robot_ip": self.robot_ip,
            "connection_mode": self.connection_mode,
            "python_path": self.python_path,
            "last_uploaded_name": self._last_uploaded_name,
            "last_error": self._last_error,
            "megaphone_active": self._megaphone_active,
            "megaphone_streamed_bytes": self._megaphone_streamed_bytes,
        }

    def connect(self) -> None:
        """建立 WebRTC 连接。

        这里使用后台事件循环线程，避免阻塞主控制线程。
        """

        if self._connected:
            return
        if not self.robot_ip:
            raise RobotWebRTCError("未配置 GO2 机身扬声器地址，请设置 TTS_PLAYER_ROBOT_IP 或 GO2_ROBOT_IP")

        rtc_topic, driver_cls, connection_method = self._import_driver()
        self._rtc_topic = rtc_topic
        self._ready.clear()
        self._stop_event.clear()
        self._last_error = ""

        def start_background_loop() -> None:
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)
            loop.set_exception_handler(self._handle_asyncio_exception)

            async def async_connect() -> None:
                try:
                    self._conn = driver_cls(connection_method.LocalSTA, ip=self.robot_ip)
                    await self._conn.connect()
                    if hasattr(self._conn, "datachannel") and hasattr(
                        self._conn.datachannel, "disableTrafficSaving"
                    ):
                        await self._conn.datachannel.disableTrafficSaving(True)
                    self._connected = True
                except Exception as exc:
                    self._last_error = str(exc)
                finally:
                    self._ready.set()

                while not self._stop_event.is_set():
                    await asyncio.sleep(0.2)

            task = loop.create_task(async_connect())
            try:
                loop.run_forever()
            finally:
                task.cancel()
                try:
                    loop.run_until_complete(asyncio.gather(task, return_exceptions=True))
                except Exception:
                    pass
                loop.close()

        self._thread = threading.Thread(
            target=start_background_loop,
            name="RobotWebRTCAudioHub",
            daemon=True,
        )
        self._thread.start()

        if not self._ready.wait(timeout=self.connect_timeout):
            raise RobotWebRTCError("等待机器人 WebRTC 连接超时")
        if not self._connected:
            raise RobotWebRTCError(self._last_error or "机器人 WebRTC 连接失败")

        logger.info("机器人扬声器 WebRTC 已连接: ip=%s", self.robot_ip)

    def disconnect(self) -> None:
        if self._loop is None:
            return
        self._stop_event.set()

        async def async_disconnect() -> None:
            try:
                if self._conn is not None:
                    await self._conn.disconnect()
            except Exception:
                pass

        try:
            future = asyncio.run_coroutine_threadsafe(async_disconnect(), self._loop)
            future.result(timeout=2.0)
        except Exception:
            pass

        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        self._thread = None
        self._loop = None
        self._conn = None
        self._connected = False

    def pause(self) -> None:
        self._webrtc_request(AUDIO_API["PAUSE"], {})

    def upload_and_play_wav(self, wav_bytes: bytes, file_name: str) -> Dict[str, Any]:
        self.connect()
        unique_id = self._upload_audio_to_robot(wav_bytes, file_name)
        self._play_audio_on_robot(unique_id)
        self._last_uploaded_name = file_name
        return {
            "route": "robot_webrtc",
            "robot_ip": self.robot_ip,
            "file_name": file_name,
            "unique_id": unique_id,
            "bytes_uploaded": len(wav_bytes),
        }

    def upload_megaphone_wav(self, wav_bytes: bytes, duration_sec: float) -> Dict[str, Any]:
        self.connect()
        self._upload_and_play_megaphone(wav_bytes, duration_sec)
        self._last_uploaded_name = "megaphone"
        return {
            "route": "robot_webrtc_megaphone",
            "robot_ip": self.robot_ip,
            "bytes_uploaded": len(wav_bytes),
            "duration_sec": duration_sec,
        }

    def start_streaming_megaphone(
        self,
        sample_rate: int,
        channels: int,
        audio_format: str = "pcm",
    ) -> Dict[str, Any]:
        """开始实时 megaphone 会话。

        这里会先进入 megaphone 模式，并发送一个带超大占位长度的 WAV 头，
        便于机器人端按连续音频流解码后续 PCM 数据。
        """

        self.connect()
        if self._megaphone_active:
            return {
                "route": "robot_webrtc_megaphone_stream",
                "robot_ip": self.robot_ip,
                "sample_rate": self._megaphone_sample_rate,
                "channels": self._megaphone_channels,
            }

        self._webrtc_request(AUDIO_API["ENTER_MEGAPHONE"], {})
        time.sleep(0.2)

        self._megaphone_active = True
        self._megaphone_block_index = 0
        self._megaphone_started_at = time.time()
        self._megaphone_sample_rate = int(sample_rate)
        self._megaphone_channels = int(channels)
        self._megaphone_audio_format = audio_format
        self._megaphone_streamed_bytes = 0

        if audio_format == "pcm":
            header = self._build_streaming_wav_header(sample_rate, channels)
            self._send_megaphone_payload(header)
        else:
            logger.warning("实时 megaphone 非 PCM 格式未验证，将直接发送原始字节流: format=%s", audio_format)

        return {
            "route": "robot_webrtc_megaphone_stream",
            "robot_ip": self.robot_ip,
            "sample_rate": self._megaphone_sample_rate,
            "channels": self._megaphone_channels,
        }

    def stream_megaphone_chunk(self, payload: bytes) -> None:
        """向实时 megaphone 会话连续追加音频块。"""

        if not payload:
            return
        if not self._megaphone_active:
            raise RobotWebRTCError("megaphone 流式会话尚未开始")
        self._send_megaphone_payload(payload)
        self._megaphone_streamed_bytes += len(payload)

    def finish_streaming_megaphone(self, duration_sec: float = 0.0) -> Dict[str, Any]:
        """结束实时 megaphone 会话。

        会在退出前留一个很短的尾巴时间，让机器人端把已经收到的音频播完。
        """

        if not self._megaphone_active:
            return {
                "route": "robot_webrtc_megaphone_stream",
                "robot_ip": self.robot_ip,
                "bytes_uploaded": 0,
                "duration_sec": 0.0,
            }

        played_for = max(0.0, time.time() - self._megaphone_started_at)
        expected_total = max(0.0, float(duration_sec))
        remain = max(0.0, expected_total - played_for)
        time.sleep(min(1.0, remain + 0.3))
        self._exit_megaphone()
        self._last_uploaded_name = "megaphone-stream"
        return {
            "route": "robot_webrtc_megaphone_stream",
            "robot_ip": self.robot_ip,
            "bytes_uploaded": self._megaphone_streamed_bytes,
            "duration_sec": duration_sec,
        }

    def abort_streaming_megaphone(self) -> None:
        if not self._megaphone_active:
            return
        self._exit_megaphone()

    def _import_driver(self):
        self._prepare_extra_pythonpath()
        try:
            from unitree_webrtc_connect.constants import RTC_TOPIC  # type: ignore[import-untyped]
            from unitree_webrtc_connect.webrtc_driver import (  # type: ignore[import-untyped]
                UnitreeWebRTCConnection,
                WebRTCConnectionMethod,
            )
        except ModuleNotFoundError as exc:
            install_command = "%s -m pip install unitree_webrtc_connect_leshy==2.0.7" % sys.executable
            extra_path_hint = ""
            if self.python_path:
                extra_path_hint = " 已尝试追加 TTS_PLAYER_ROBOT_PYTHONPATH=%s。" % self.python_path
            raise RobotWebRTCError(
                "缺少 unitree_webrtc_connect，无法使用 robot_webrtc 播放链路。"
                "请在当前解释器下安装依赖：%s。%s 当前解释器=%s"
                % (install_command, extra_path_hint, sys.executable)
            ) from exc
        except Exception as exc:
            raise RobotWebRTCError("导入 unitree_webrtc_connect 失败: %s" % exc) from exc
        return RTC_TOPIC, UnitreeWebRTCConnection, WebRTCConnectionMethod

    def _prepare_extra_pythonpath(self) -> None:
        """按配置把额外 site-packages 路径加入搜索路径。"""

        if not self.python_path:
            return
        for raw_path in reversed(self.python_path.split(os.pathsep)):
            path_text = raw_path.strip()
            if not path_text:
                continue
            path = Path(path_text).expanduser()
            if not path.exists():
                logger.warning("忽略不存在的 robot_webrtc 依赖路径: %s", path)
                continue
            resolved = str(path.resolve())
            if resolved not in sys.path:
                sys.path.insert(0, resolved)
                logger.debug("已追加 robot_webrtc 依赖路径: %s", resolved)

    def _handle_asyncio_exception(self, loop, context) -> None:
        """忽略连接关闭阶段常见的 MediaStreamError 噪声。"""

        del loop
        exception = context.get("exception")
        message = str(context.get("message") or "")
        if exception is not None:
            exc_name = exception.__class__.__name__
            if exc_name == "MediaStreamError":
                logger.debug("忽略 robot_webrtc 关闭阶段的 MediaStreamError")
                return
        if "AsyncIOEventEmitter" in message and "callback" in message:
            logger.debug("忽略 robot_webrtc 关闭阶段的异步回调噪声: %s", message)
            return
        if exception is not None:
            logger.error(
                "robot_webrtc 异步事件异常: %s",
                message or context,
                exc_info=(type(exception), exception, exception.__traceback__),
            )
            return
        logger.error("robot_webrtc 异步事件异常: %s", message or context)

    def _publish_request(self, topic: str, data: Dict[str, Any]) -> Any:
        if not self._connected or self._loop is None or self._conn is None:
            raise RobotWebRTCError("机器人 WebRTC 尚未连接")
        future = asyncio.run_coroutine_threadsafe(
            self._conn.datachannel.pub_sub.publish_request_new(topic, data),
            self._loop,
        )
        try:
            return future.result(timeout=self.request_timeout)
        except FutureTimeoutError as exc:
            self._last_error = "robot_webrtc 请求超时: %.1fs" % self.request_timeout
            raise RobotWebRTCError(self._last_error) from exc
        except Exception as exc:
            self._last_error = self._format_error(exc)
            raise RobotWebRTCError(self._last_error) from exc

    def _webrtc_request(self, api_id: int, parameter: Optional[Dict[str, Any]] = None) -> Any:
        if self._rtc_topic is None:
            raise RobotWebRTCError("机器人 WebRTC 主题未初始化")
        request_data = {
            "api_id": api_id,
            "parameter": json.dumps(parameter or {}, ensure_ascii=False),
        }
        return self._publish_request(self._rtc_topic["AUDIO_HUB_REQ"], request_data)

    def _upload_audio_to_robot(self, audio_data: bytes, filename: str) -> str:
        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode("utf-8")
        chunks = [
            b64_data[i : i + self.upload_chunk_size]
            for i in range(0, len(b64_data), self.upload_chunk_size)
        ]

        logger.debug("开始上传机器人扬声音频: file=%s chunks=%s", filename, len(chunks))
        for index, chunk in enumerate(chunks, 1):
            parameter = {
                "file_name": filename,
                "file_type": "wav",
                "file_size": len(audio_data),
                "current_block_index": index,
                "total_block_number": len(chunks),
                "block_content": chunk,
                "current_block_size": len(chunk),
                "file_md5": file_md5,
                "create_time": int(time.time() * 1000),
            }
            self._webrtc_request(AUDIO_API["UPLOAD_AUDIO_FILE"], parameter)

        response = self._webrtc_request(AUDIO_API["GET_AUDIO_LIST"], {})
        data = self._extract_audio_list(response)
        for audio in data:
            if audio.get("CUSTOM_NAME") == filename:
                return str(audio.get("UNIQUE_ID"))

        logger.warning("未在机器人音频列表中找到刚上传的文件，回退使用文件名作为标识: %s", filename)
        return filename

    def _play_audio_on_robot(self, unique_id: str) -> None:
        self._webrtc_request(AUDIO_API["SET_PLAY_MODE"], {"play_mode": PLAY_MODES["NO_CYCLE"]})
        time.sleep(0.1)
        self._webrtc_request(AUDIO_API["SELECT_START_PLAY"], {"unique_id": unique_id})

    def _upload_and_play_megaphone(self, audio_data: bytes, duration_sec: float) -> None:
        self._webrtc_request(AUDIO_API["ENTER_MEGAPHONE"], {})
        time.sleep(0.2)
        try:
            b64_data = base64.b64encode(audio_data).decode("utf-8")
            chunks = [
                b64_data[i : i + self.megaphone_chunk_size]
                for i in range(0, len(b64_data), self.megaphone_chunk_size)
            ]
            for index, chunk in enumerate(chunks, 1):
                parameter = {
                    "current_block_size": len(chunk),
                    "block_content": chunk,
                    "current_block_index": index,
                    "total_block_number": len(chunks),
                }
                self._webrtc_request(AUDIO_API["UPLOAD_MEGAPHONE"], parameter)
                if index < len(chunks):
                    pause = self._megaphone_chunk_pause()
                    if pause > 0:
                        time.sleep(pause)
            time.sleep(max(0.15, float(duration_sec)) + 0.25)
        finally:
            try:
                self._webrtc_request(AUDIO_API["EXIT_MEGAPHONE"], {})
            except Exception:
                pass

    def _send_megaphone_payload(self, payload: bytes) -> None:
        b64_data = base64.b64encode(payload).decode("utf-8")
        chunks = [
            b64_data[i : i + self.megaphone_chunk_size]
            for i in range(0, len(b64_data), self.megaphone_chunk_size)
        ]
        for chunk in chunks:
            self._megaphone_block_index += 1
            parameter = {
                "current_block_size": len(chunk),
                "block_content": chunk,
                "current_block_index": self._megaphone_block_index,
                "total_block_number": self._megaphone_block_index,
            }
            self._webrtc_request(AUDIO_API["UPLOAD_MEGAPHONE"], parameter)

    def _megaphone_chunk_pause(self) -> float:
        base_chunk = 4096.0
        chunk_size = max(base_chunk, float(self.megaphone_chunk_size))
        return 0.01 * (base_chunk / chunk_size)

    @staticmethod
    def _format_error(exc: Exception) -> str:
        message = str(exc).strip()
        if message:
            return message
        return exc.__class__.__name__

    def _exit_megaphone(self) -> None:
        try:
            self._webrtc_request(AUDIO_API["EXIT_MEGAPHONE"], {})
        finally:
            self._megaphone_active = False
            self._megaphone_block_index = 0
            self._megaphone_started_at = 0.0
            self._megaphone_sample_rate = 0
            self._megaphone_channels = 1
            self._megaphone_audio_format = "pcm"

    def _build_streaming_wav_header(self, sample_rate: int, channels: int) -> bytes:
        """构造一个可持续追加 PCM 的 WAV 头。

        这里使用超大占位长度，让接收端把后续 megaphone 数据继续当作同一条 WAV 流。
        """

        sample_rate = max(1, int(sample_rate))
        channels = max(1, int(channels))
        bits_per_sample = 16
        block_align = channels * bits_per_sample // 8
        byte_rate = sample_rate * block_align
        data_size = 0x7FFFF000
        riff_size = min(0xFFFFFFFF, data_size + 36)
        return struct.pack(
            "<4sI4s4sIHHIIHH4sI",
            b"RIFF",
            riff_size,
            b"WAVE",
            b"fmt ",
            16,
            1,
            channels,
            sample_rate,
            byte_rate,
            block_align,
            bits_per_sample,
            b"data",
            data_size,
        )

    @staticmethod
    def _extract_audio_list(response: Any) -> List[Dict[str, Any]]:
        if not isinstance(response, dict):
            return []
        data = response.get("data")
        if not isinstance(data, dict):
            return []
        inner = data.get("data", "{}")
        if isinstance(inner, dict):
            payload = inner
        else:
            try:
                payload = json.loads(inner)
            except Exception:
                return []
        audio_list = payload.get("audio_list", [])
        if isinstance(audio_list, list):
            return [item for item in audio_list if isinstance(item, dict)]
        return []
