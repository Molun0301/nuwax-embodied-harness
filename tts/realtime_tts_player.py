#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Go2 端本地 TTS 播放管理器

职责：
1. 在本进程内接收 Doubao 流式客户端推送的音频块。
2. 按配置选择播放后端。
   - robot_webrtc: Go2 机身扬声器新链路
   - alsa_local: Jetson 本地 ALSA/Pulse 兼容链路
"""

from __future__ import annotations

import asyncio
import audioop
import fcntl
import logging
import os
import queue
import shutil
import socket
import struct
import subprocess
import threading
import time
import wave
from dataclasses import replace
from io import BytesIO
from typing import Dict, List, Optional, Set, Any

from logging_utils import compact_log_value
from settings import APP_CONFIG, LocalPlaybackConfig, RobotWebRTCPlaybackConfig, TTSPlayerConfig
from .robot_webrtc_audio import RobotWebRTCAudioHubClient


logger = logging.getLogger(__name__)


try:
    import websockets
except ImportError:
    websockets = None


def _format_exception_message(exc: Exception) -> str:
    message = str(exc).strip()
    if message:
        return message
    return exc.__class__.__name__


def _resolve_interface_ipv4(interface_name: str) -> str:
    interface_name = (interface_name or "").strip()
    if not interface_name:
        return ""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = struct.pack("256s", interface_name[:15].encode("utf-8"))
        response = fcntl.ioctl(sock.fileno(), 0x8915, packed)
        return socket.inet_ntoa(response[20:24])
    finally:
        sock.close()


def _clamp_volume(volume: float) -> float:
    try:
        volume = float(volume)
    except (TypeError, ValueError):
        return 1.0
    return max(0.0, min(1.0, volume))


def _pcm_to_wav_bytes(payload: bytes, sample_rate: int, channels: int) -> bytes:
    """将 PCM 字节包装成 WAV，供机器人 Audio Hub 上传。"""

    buffer = BytesIO()
    with wave.open(buffer, "wb") as wav_file:
        wav_file.setnchannels(max(1, int(channels)))
        wav_file.setsampwidth(2)
        wav_file.setframerate(max(1, int(sample_rate)))
        wav_file.writeframes(payload)
    return buffer.getvalue()


class LocalAudioBackend:
    """Jetson 本地音频播放兼容链路。"""

    def __init__(self, config: LocalPlaybackConfig, volume: float = 1.0) -> None:
        self.config = config
        self.process = None
        self.audio_format = ""
        self.sample_rate = 0
        self.channels = 1
        self.playback_sample_rate = 0
        self.playback_channels = 1
        self.backend_name = ""
        self.last_error = ""
        self.last_stderr = ""
        self.bytes_written = 0
        self.last_audio_at = 0.0
        self.route_prepared = False
        self.route_backend = ""
        self.route_device = config.audio_device
        self.speaker_enabled = False
        self._volume = _clamp_volume(volume)
        self._stderr_thread = None
        self._stderr_stop = threading.Event()
        self._ratecv_state = None

    def start(self, audio_format: str, sample_rate: int, channels: int = 1) -> None:
        self.stop(force=True)
        errors = []
        playback_sample_rate = sample_rate
        playback_channels = channels
        for backend_name, cmd, playback_sample_rate, playback_channels in self._build_commands(
            audio_format,
            sample_rate,
            channels,
        ):
            logger.debug("启动TTS播放后端(%s): %s", backend_name, " ".join(cmd))
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self._attach_process(process, backend_name)
            time.sleep(0.15)
            if process.poll() is None:
                self.audio_format = audio_format
                self.sample_rate = sample_rate
                self.channels = channels
                self.playback_sample_rate = playback_sample_rate
                self.playback_channels = playback_channels
                self.bytes_written = 0
                self.last_audio_at = 0.0
                self.last_error = ""
                self._ratecv_state = None
                logger.info("TTS播放后端已启动: backend=%s pid=%s", backend_name, process.pid)
                return

            stderr_output = self._drain_stderr(process)
            self.last_error = stderr_output or "播放进程启动后立即退出"
            errors.append("%s(exit=%s): %s" % (backend_name, process.returncode, self.last_error))
            logger.warning(
                "TTS播放后端启动失败: backend=%s exit=%s error=%s",
                backend_name,
                process.returncode,
                self.last_error,
            )
            self._disable_speaker_if_needed()
            self._detach_process()

        raise RuntimeError("所有本地TTS播放后端都启动失败: %s" % "; ".join(errors))

    def write(self, payload: bytes) -> None:
        if not payload or self.process is None or self.process.stdin is None:
            return
        if self.process.poll() is not None:
            self.last_error = self._drain_stderr(self.process) or "播放进程已退出"
            logger.warning(
                "TTS播放后端已退出: backend=%s exit=%s error=%s",
                self.backend_name or "-",
                self.process.returncode,
                self.last_error,
            )
            self.stop(force=True)
            return
        try:
            payload = self._apply_volume(payload)
            self.process.stdin.write(payload)
            self.process.stdin.flush()
            self.bytes_written += len(payload)
            self.last_audio_at = time.time()
        except BrokenPipeError:
            logger.warning("TTS播放后端管道已关闭")
            self.stop(force=True)
        except OSError as exc:
            self.last_error = str(exc)
            logger.warning("写入TTS播放后端失败: %s", exc)
            self.stop(force=True)

    def finish(self) -> None:
        if self.process is None:
            return
        if self.process.stdin:
            try:
                self.process.stdin.close()
            except OSError:
                pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self._finalize_process()

    def stop(self, force: bool = True) -> None:
        if self.process is None:
            return
        if self.process.stdin:
            try:
                self.process.stdin.close()
            except OSError:
                pass
        if force:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        else:
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self._finalize_process()

    def set_volume(self, volume: float) -> float:
        self._volume = _clamp_volume(volume)
        return self._volume

    def get_volume(self) -> float:
        return self._volume

    def status(self) -> Dict[str, Any]:
        return {
            "backend_name": self.backend_name,
            "audio_format": self.audio_format,
            "sample_rate": self.sample_rate,
            "channels": self.channels,
            "playback_sample_rate": self.playback_sample_rate,
            "playback_channels": self.playback_channels,
            "audio_device": self.config.audio_device,
            "enable_spk_ctl": self.config.enable_spk_ctl,
            "route_prepared": self.route_prepared,
            "route_backend": self.route_backend,
            "route_device": self.route_device,
            "speaker_enabled": self.speaker_enabled,
            "volume": self._volume,
            "bytes_written": self.bytes_written,
            "last_audio_at": self.last_audio_at,
            "last_error": self.last_error,
            "last_stderr": self.last_stderr,
            "backend_running": self.process is not None and self.process.poll() is None,
            "backend_pid": self.process.pid if self.process is not None else None,
        }

    def _build_commands(self, audio_format: str, sample_rate: int, channels: int):
        commands = []
        prefer_aplay = self.config.preferred_backend != "ffplay"
        if audio_format == "pcm":
            aplay_device = self._prepare_pcm_route(sample_rate, channels)
            playback_sample_rate, playback_channels = self._resolve_pcm_playback_params(
                sample_rate,
                channels,
            )
            if prefer_aplay and shutil.which("aplay"):
                commands.append(
                    (
                        "aplay",
                        self._aplay_command(
                            playback_sample_rate,
                            playback_channels,
                            audio_device=aplay_device or None,
                        ),
                        playback_sample_rate,
                        playback_channels,
                    )
                )
            if shutil.which("ffplay"):
                commands.append(
                    (
                        "ffplay",
                        self._ffplay_command(
                            playback_sample_rate,
                            playback_channels,
                            raw_pcm=True,
                        ),
                        playback_sample_rate,
                        playback_channels,
                    )
                )
            if not prefer_aplay and shutil.which("aplay"):
                commands.append(
                    (
                        "aplay",
                        self._aplay_command(
                            playback_sample_rate,
                            playback_channels,
                            audio_device=aplay_device or None,
                        ),
                        playback_sample_rate,
                        playback_channels,
                    )
                )
            if commands:
                return commands
            raise RuntimeError("未找到可用的PCM播放后端: aplay / ffplay")

        if shutil.which("ffplay"):
            return [
                (
                    "ffplay",
                    self._ffplay_command(sample_rate, channels, raw_pcm=False),
                    sample_rate,
                    channels,
                )
            ]
        raise RuntimeError("非PCM播放需要ffplay")

    def _aplay_command(self, sample_rate: int, channels: int, audio_device: Optional[str] = None):
        command = ["aplay", "-q"]
        device = audio_device if audio_device is not None else self.config.audio_device
        if device:
            command.extend(["-D", device])
        command.extend(
            [
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-r",
                str(sample_rate),
                "-c",
                str(channels),
            ]
        )
        return command

    def _prepare_pcm_route(self, sample_rate: int, channels: int) -> str:
        self.route_prepared = False
        self.route_backend = ""
        self.route_device = self.config.audio_device
        self.speaker_enabled = False

        if self.config.audio_device:
            return self.config.audio_device

        spk_device = self._try_enable_speaker()
        if spk_device:
            self.route_prepared = True
            self.route_backend = "spk-ctl"
            self.route_device = spk_device
            self.speaker_enabled = True
            logger.debug("已启用喇叭功放控制: device=%s", spk_device)
            return spk_device

        default_device = self._preferred_alsa_device()
        if default_device:
            self._prepare_pulse_sink(default_device)
            self.route_prepared = True
            self.route_backend = "alsa-default"
            self.route_device = default_device
            logger.debug("已选择系统默认播放设备: device=%s", default_device)
            return default_device

        if not self.config.enable_ape_route:
            return ""

        ape_device = self._try_prepare_ape_route(sample_rate, channels)
        if ape_device:
            self.route_prepared = True
            self.route_backend = "ape"
            self.route_device = ape_device
            logger.debug(
                "已配置APE播放路由: card=%s speaker=%s admaif=%s device=%s",
                self.config.ape_card,
                self.config.ape_speaker,
                self.config.ape_admaif,
                ape_device,
            )
            return ape_device

        return ""

    def _resolve_pcm_playback_params(self, sample_rate: int, channels: int):
        playback_sample_rate = sample_rate
        playback_channels = channels
        if self.route_backend in {"alsa-default", "ape", "spk-ctl"}:
            if playback_sample_rate < 48000:
                playback_sample_rate = 48000
            if playback_channels == 1:
                playback_channels = 2
        if playback_sample_rate != sample_rate or playback_channels != channels:
            logger.debug(
                "已启用PCM播放格式转换: input=%sHz/%sch output=%sHz/%sch route=%s",
                sample_rate,
                channels,
                playback_sample_rate,
                playback_channels,
                self.route_backend or "-",
            )
        return playback_sample_rate, playback_channels

    def _try_enable_speaker(self) -> str:
        if not self.config.enable_spk_ctl or not shutil.which("spk-ctl"):
            return ""
        try:
            subprocess.run(
                ["spk-ctl", "enable"],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception as exc:
            logger.warning("启用喇叭功放控制失败: %s", exc)
            return ""
        return "default"

    def _preferred_alsa_device(self) -> str:
        pcms = self._alsa_pcm_names()
        for device in ("default", "pulse", "sysdefault:CARD=APE", "plughw:CARD=APE,DEV=0"):
            if device in pcms:
                return device
        return ""

    def _alsa_pcm_names(self) -> Set[str]:
        if not shutil.which("aplay"):
            return set()
        try:
            result = subprocess.run(
                ["aplay", "-L"],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception:
            return set()

        names = set()
        for raw_line in result.stdout.splitlines():
            line = raw_line.rstrip()
            if not line or line[:1].isspace():
                continue
            names.add(line.strip())
        return names

    def _prepare_pulse_sink(self, device: str) -> None:
        if device not in {"default", "pulse"} or not shutil.which("pactl"):
            return
        sinks = self._pulse_sink_names()
        if not sinks:
            return
        sink = sinks[0]
        try:
            subprocess.run(
                ["pactl", "set-default-sink", sink],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception:
            pass
        try:
            subprocess.run(
                ["pactl", "set-sink-mute", sink, "0"],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception as exc:
            logger.warning("取消Pulse sink静音失败: sink=%s error=%s", sink, exc)
        volume_text = self._pactl_read_text(["pactl", "get-sink-volume", sink])
        mute_text = self._pactl_read_text(["pactl", "get-sink-mute", sink])
        logger.debug(
            "Pulse sink已准备: sink=%s volume=%s mute=%s",
            sink,
            volume_text or "-",
            mute_text or "-",
        )

    def _pulse_sink_names(self) -> List[str]:
        text = self._pactl_read_text(["pactl", "list", "sinks", "short"])
        if not text:
            return []
        names = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                names.append(parts[1])
        return names

    def _pactl_read_text(self, command: List[str]) -> str:
        try:
            result = subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception:
            return ""
        return "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())

    def _try_prepare_ape_route(self, sample_rate: int, channels: int) -> str:
        if not shutil.which("amixer"):
            return ""
        if self.config.ape_admaif < 1:
            return ""

        controls = self._ape_controls()
        if not controls:
            return ""

        admaif_name = "ADMAIF%d" % self.config.ape_admaif
        device = "hw:%s,%d" % (self.config.ape_card, self.config.ape_admaif - 1)
        speaker = self.config.ape_speaker

        commands = []
        if "%s Mux" % speaker in controls:
            commands.append(self._amixer_cset_command("%s Mux" % speaker, admaif_name))
        else:
            return ""

        if "%s Audio Channels" % speaker in controls:
            commands.append(self._amixer_cset_command("%s Audio Channels" % speaker, str(max(1, channels))))
        if "%s Sample Rate" % speaker in controls:
            commands.append(self._amixer_cset_command("%s Sample Rate" % speaker, str(sample_rate)))
        if "%s Audio Bit Format" % speaker in controls:
            commands.append(self._amixer_cset_command("%s Audio Bit Format" % speaker, "16"))
        if "%s Channel Select" % speaker in controls:
            channel_select = "Stereo" if channels >= 2 else "Left"
            commands.append(self._amixer_cset_command("%s Channel Select" % speaker, channel_select))
        if channels == 1 and "%s Mono To Stereo" % speaker in controls:
            commands.append(self._amixer_cset_command("%s Mono To Stereo" % speaker, "Copy"))

        try:
            for command in commands:
                subprocess.run(
                    command,
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=1.0,
                )
        except Exception as exc:
            logger.warning("配置APE播放路由失败: %s", exc)
            return ""

        return device

    def _ape_controls(self) -> Set[str]:
        try:
            result = subprocess.run(
                ["amixer", "-c", self.config.ape_card, "scontrols"],
                check=True,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except Exception:
            return set()

        controls = set()
        for line in result.stdout.splitlines():
            line = line.strip()
            marker = "Simple mixer control '"
            if marker not in line:
                continue
            start = line.find(marker) + len(marker)
            end = line.find("'", start)
            if end > start:
                controls.add(line[start:end])
        return controls

    def _amixer_cset_command(self, control_name: str, value: str) -> List[str]:
        return ["amixer", "-c", self.config.ape_card, "cset", "name=%s" % control_name, str(value)]

    def _ffplay_command(self, sample_rate: int, channels: int, raw_pcm: bool):
        command = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error"]
        if raw_pcm:
            command.extend(
                [
                    "-f",
                    "s16le",
                    "-ar",
                    str(sample_rate),
                    "-ac",
                    str(channels),
                ]
            )
        command.extend(["-i", "pipe:0"])
        return command

    def _attach_process(self, process, backend_name: str) -> None:
        self.process = process
        self.backend_name = backend_name
        self.last_stderr = ""
        self._stderr_stop = threading.Event()
        self._stderr_thread = threading.Thread(
            target=self._stderr_reader,
            args=(process, backend_name, self._stderr_stop),
            daemon=True,
            name="TTSAudioSinkStderr",
        )
        self._stderr_thread.start()

    def _detach_process(self) -> None:
        self.process = None
        self.backend_name = ""
        self._stderr_stop.set()
        self._stderr_thread = None

    def _finalize_process(self) -> None:
        if self.process is not None:
            self.last_stderr = self._drain_stderr(self.process) or self.last_stderr
        self._disable_speaker_if_needed()
        self._detach_process()

    def _disable_speaker_if_needed(self) -> None:
        if not self.speaker_enabled or not shutil.which("spk-ctl"):
            self.speaker_enabled = False
            return
        commands = (
            ["spk-ctl", "disables"],
            ["spk-ctl", "disable"],
        )
        for command in commands:
            try:
                subprocess.run(
                    command,
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=1.0,
                )
                logger.debug("已关闭喇叭功放控制: %s", " ".join(command[1:]))
                self.speaker_enabled = False
                return
            except Exception:
                continue
        logger.warning("关闭喇叭功放控制失败")
        self.speaker_enabled = False

    def _stderr_reader(self, process, backend_name: str, stop_event: threading.Event) -> None:
        if process.stderr is None:
            return
        while not stop_event.is_set():
            chunk = process.stderr.readline()
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            self.last_stderr = text
            logger.debug(
                "TTS 播放后端输出",
                extra={
                    "event": "tts.backend.stderr",
                    "backend": backend_name,
                    "stderr_preview": compact_log_value(text, max_length=240),
                },
            )

    def _drain_stderr(self, process) -> str:
        if process.stderr is None:
            return ""
        try:
            chunk = process.stderr.read()
        except Exception:
            return self.last_stderr
        if not chunk:
            return self.last_stderr
        text = chunk.decode("utf-8", errors="replace").strip()
        if text:
            self.last_stderr = text
        return self.last_stderr

    def _apply_volume(self, payload: bytes) -> bytes:
        payload = self._convert_pcm_output(payload)
        if self.audio_format != "pcm":
            return payload
        if self._volume >= 0.999:
            return payload
        try:
            return audioop.mul(payload, 2, self._volume)
        except Exception as exc:
            self.last_error = "软件音量调整失败: %s" % exc
            logger.warning(self.last_error)
            return payload

    def _convert_pcm_output(self, payload: bytes) -> bytes:
        if self.audio_format != "pcm" or not payload:
            return payload

        converted = payload
        try:
            if self.sample_rate != self.playback_sample_rate:
                converted, self._ratecv_state = audioop.ratecv(
                    converted,
                    2,
                    self.channels,
                    self.sample_rate,
                    self.playback_sample_rate,
                    self._ratecv_state,
                )
            if self.channels == 1 and self.playback_channels == 2:
                converted = audioop.tostereo(converted, 2, 1, 1)
            elif self.channels == 2 and self.playback_channels == 1:
                converted = audioop.tomono(converted, 2, 0.5, 0.5)
        except Exception as exc:
            self.last_error = "PCM格式转换失败: %s" % exc
            logger.warning(self.last_error)
            self._ratecv_state = None
            return payload
        return converted


class RobotSpeakerBackend:
    """Go2 机身扬声器播放后端。

    该后端不会直接写本地声卡，而是把会话音频发给机器人扬声器。
    实现策略：
    1. 默认走更稳定的整段 megaphone 上传。
    2. 若显式开启实时 megaphone 流式上传，则在失败时自动回退整段上传。
    """

    def __init__(self, config: RobotWebRTCPlaybackConfig, volume: float = 1.0) -> None:
        self.config = config
        self.client = RobotWebRTCAudioHubClient(
            robot_ip=config.robot_ip,
            connection_mode=config.connection_mode,
            python_path=config.python_path,
            connect_timeout=config.connect_timeout,
            request_timeout=config.request_timeout,
            upload_chunk_size=config.upload_chunk_size,
            megaphone_chunk_size=config.megaphone_chunk_size,
        )
        self.audio_format = ""
        self.sample_rate = 0
        self.channels = 1
        self.bytes_written = 0
        self.last_audio_at = 0.0
        self.last_error = ""
        self.last_stderr = ""
        self.route_backend = "robot_webrtc"
        self.route_device = config.robot_ip or "-"
        self.backend_name = "robot_webrtc"
        self._volume = _clamp_volume(volume)
        self._pcm_buffer = bytearray()
        self._uploaded_result = {}
        self._streaming_enabled = False
        self._streaming_active = False
        self._streaming_failed = ""
        self._sentence_segment_enabled = False
        self._segment_queue = None
        self._segment_thread = None
        self._segment_stop = threading.Event()
        self._segment_lock = threading.Lock()
        self._segment_index = 0
        self._segment_min_duration_sec = max(0.0, float(config.sentence_segment_min_duration_sec))

    def start(self, audio_format: str, sample_rate: int, channels: int = 1) -> None:
        self.audio_format = audio_format
        self.sample_rate = int(sample_rate)
        self.channels = int(channels)
        self.bytes_written = 0
        self.last_audio_at = 0.0
        self.last_error = ""
        self.last_stderr = ""
        self._pcm_buffer = bytearray()
        self._uploaded_result = {}
        self._streaming_enabled = False
        self._streaming_active = False
        self._streaming_failed = ""
        self._sentence_segment_enabled = False
        self._segment_queue = queue.Queue()
        self._segment_thread = None
        self._segment_stop.clear()
        self._segment_index = 0
        self.client.connect()
        logger.debug("机器人扬声器后端已就绪: ip=%s mode=%s", self.config.robot_ip, self.config.connection_mode)

        if self.config.use_megaphone and self.config.streaming_enabled and self.audio_format == "pcm":
            logger.warning("robot_webrtc 实时 megaphone 仍属实验路径，当前会优先尝试实时流式，失败后自动回退整段上传")
            try:
                self._uploaded_result = self.client.start_streaming_megaphone(
                    sample_rate=self.sample_rate,
                    channels=self.channels,
                    audio_format=self.audio_format,
                )
                self._streaming_enabled = True
                self._streaming_active = True
                logger.debug(
                    "机器人扬声器已进入实时 megaphone 模式: ip=%s sample_rate=%s channels=%s",
                    self.config.robot_ip,
                    self.sample_rate,
                    self.channels,
                )
            except Exception as exc:
                self._streaming_failed = str(exc)
                self._streaming_enabled = True
                self._streaming_active = False
                logger.warning("机器人扬声器实时 megaphone 启动失败，将回退整段上传: %s", exc)

        if self.config.use_megaphone and not self._streaming_active and self.audio_format in {"pcm", "wav"}:
            self._sentence_segment_enabled = True
            self._segment_thread = threading.Thread(
                target=self._segment_worker,
                name="RobotSpeakerSegmentWorker",
                daemon=True,
            )
            self._segment_thread.start()
            logger.debug("机器人扬声器已启用按句分段上传模式")

    def write(self, payload: bytes) -> None:
        if not payload:
            return
        if self.audio_format not in {"pcm", "wav"}:
            self.last_error = "机器人扬声器后端仅支持 pcm/wav，当前=%s" % self.audio_format
            raise RuntimeError(self.last_error)

        if self.audio_format == "pcm" and self._volume < 0.999:
            try:
                payload = audioop.mul(payload, 2, self._volume)
            except Exception as exc:
                self.last_error = "机器人扬声器软件音量调整失败: %s" % exc
                logger.warning(self.last_error)

        with self._segment_lock:
            self._pcm_buffer.extend(payload)
        self.bytes_written += len(payload)
        self.last_audio_at = time.time()

        if self._streaming_active:
            try:
                self.client.stream_megaphone_chunk(payload)
            except Exception as exc:
                self._streaming_failed = str(exc)
                self.last_error = "机器人扬声器实时 megaphone 传输失败: %s" % exc
                logger.warning("%s，回退到整段上传", self.last_error)
                try:
                    self.client.abort_streaming_megaphone()
                except Exception:
                    pass
                self._streaming_active = False

    def finish(self) -> None:
        try:
            if self._sentence_segment_enabled and not self._streaming_active:
                self.mark_sentence_end(force=True)
                if self._segment_queue is not None:
                    self._segment_queue.join()
                return

            if not self._pcm_buffer and not self._streaming_active:
                return

            duration_sec = self._estimate_duration_sec()

            if self._streaming_active and not self._streaming_failed:
                try:
                    self._uploaded_result = self.client.finish_streaming_megaphone(duration_sec)
                    self._streaming_active = False
                    self._pcm_buffer = bytearray()
                    return
                except Exception as exc:
                    self._streaming_failed = str(exc)
                    logger.warning("机器人扬声器实时 megaphone 收尾失败，将回退整段上传: %s", exc)
                    try:
                        self.client.abort_streaming_megaphone()
                    except Exception:
                        pass
                    self._streaming_active = False

            if not self._pcm_buffer:
                return

            if self.audio_format == "pcm":
                wav_bytes = _pcm_to_wav_bytes(bytes(self._pcm_buffer), self.sample_rate, self.channels)
            elif self.audio_format == "wav":
                wav_bytes = bytes(self._pcm_buffer)
            else:
                raise RuntimeError("不支持的机器人扬声器音频格式: %s" % self.audio_format)

            if self.config.use_megaphone:
                self._uploaded_result = self.client.upload_megaphone_wav(wav_bytes, duration_sec)
            else:
                file_name = "tts_%d.wav" % int(time.time() * 1000)
                self._uploaded_result = self.client.upload_and_play_wav(wav_bytes, file_name)
        except Exception as exc:
            self.last_error = str(exc)
            raise
        finally:
            with self._segment_lock:
                self._pcm_buffer = bytearray()

    def stop(self, force: bool = True) -> None:
        self._stop_segment_worker()
        if self._streaming_active:
            try:
                self.client.abort_streaming_megaphone()
            except Exception:
                pass
            self._streaming_active = False
        try:
            self.client.pause()
        except Exception:
            pass
        if force:
            try:
                self.client.disconnect()
            except Exception:
                pass
        with self._segment_lock:
            self._pcm_buffer = bytearray()

    def mark_sentence_end(self, force: bool = False) -> bool:
        if not self._sentence_segment_enabled or self._streaming_active:
            return False
        segment = self._take_segment(force=force)
        if segment is None:
            return False
        if self._segment_queue is None:
            return False
        self._segment_queue.put(segment)
        logger.debug(
            "机器人扬声器分段已入队: index=%s bytes=%s duration=%.2fs",
            segment["index"],
            segment["bytes_uploaded"],
            segment["duration_sec"],
        )
        return True

    def set_volume(self, volume: float) -> float:
        self._volume = _clamp_volume(volume)
        return self._volume

    def get_volume(self) -> float:
        return self._volume

    def status(self) -> Dict[str, Any]:
        client_status = self.client.status()
        return {
            "backend_name": self.backend_name,
            "audio_format": self.audio_format,
            "sample_rate": self.sample_rate,
            "channels": self.channels,
            "playback_sample_rate": self.sample_rate,
            "playback_channels": self.channels,
            "audio_device": "",
            "enable_spk_ctl": False,
            "route_prepared": client_status.get("connected", False),
            "route_backend": self.route_backend,
            "route_device": self.route_device,
            "speaker_enabled": True,
            "volume": self._volume,
            "bytes_written": self.bytes_written,
            "last_audio_at": self.last_audio_at,
            "last_error": self.last_error or client_status.get("last_error", ""),
            "last_stderr": self.last_stderr,
            "backend_running": client_status.get("connected", False),
            "backend_pid": None,
            "robot_webrtc": client_status,
            "upload_result": dict(self._uploaded_result),
            "streaming_enabled": self._streaming_enabled,
            "streaming_active": self._streaming_active,
            "streaming_failed": self._streaming_failed,
            "sentence_segment_enabled": self._sentence_segment_enabled,
            "sentence_segment_min_duration_sec": self._segment_min_duration_sec,
        }

    def _estimate_duration_sec(self) -> float:
        with self._segment_lock:
            pcm_len = len(self._pcm_buffer)
        if self.audio_format != "pcm" or self.sample_rate <= 0 or self.channels <= 0:
            return 0.0
        frame_size = 2 * self.channels
        if frame_size <= 0:
            return 0.0
        total_frames = pcm_len / float(frame_size)
        return total_frames / float(self.sample_rate)

    def _segment_duration_sec(self, payload_len: int) -> float:
        if self.audio_format != "pcm" or self.sample_rate <= 0 or self.channels <= 0:
            return 0.0
        frame_size = 2 * self.channels
        if frame_size <= 0:
            return 0.0
        total_frames = payload_len / float(frame_size)
        return total_frames / float(self.sample_rate)

    def _take_segment(self, force: bool) -> Optional[Dict[str, Any]]:
        with self._segment_lock:
            if not self._pcm_buffer:
                return None
            duration_sec = self._segment_duration_sec(len(self._pcm_buffer))
            if not force and duration_sec < self._segment_min_duration_sec:
                return None
            if self.audio_format == "pcm":
                wav_bytes = _pcm_to_wav_bytes(bytes(self._pcm_buffer), self.sample_rate, self.channels)
            elif self.audio_format == "wav":
                wav_bytes = bytes(self._pcm_buffer)
            else:
                raise RuntimeError("不支持的机器人扬声器音频格式: %s" % self.audio_format)
            self._segment_index += 1
            self._pcm_buffer = bytearray()
        return {
            "index": self._segment_index,
            "wav_bytes": wav_bytes,
            "bytes_uploaded": len(wav_bytes),
            "duration_sec": duration_sec,
        }

    def _segment_worker(self) -> None:
        while not self._segment_stop.is_set():
            if self._segment_queue is None:
                return
            try:
                segment = self._segment_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if segment is None:
                self._segment_queue.task_done()
                return
            try:
                self._uploaded_result = self._upload_segment_with_retry(segment)
                self.last_error = ""
                logger.debug(
                    "机器人扬声器分段播放完成: index=%s duration=%.2fs",
                    segment["index"],
                    segment["duration_sec"],
                )
            except Exception as exc:
                self.last_error = _format_exception_message(exc)
                logger.warning(
                    "机器人扬声器分段播放失败: index=%s error=%s",
                    segment["index"],
                    self.last_error,
                )
            finally:
                self._segment_queue.task_done()

    def _stop_segment_worker(self) -> None:
        if self._segment_queue is not None:
            try:
                self._segment_queue.put_nowait(None)
            except Exception:
                pass
        self._segment_stop.set()
        if self._segment_thread is not None and self._segment_thread.is_alive():
            self._segment_thread.join(timeout=2.0)
        self._segment_thread = None
        self._segment_queue = None
        self._sentence_segment_enabled = False

    def _upload_segment_with_retry(self, segment: Dict[str, Any]) -> Dict[str, Any]:
        last_exc = None
        for attempt in range(2):
            try:
                return self.client.upload_megaphone_wav(
                    segment["wav_bytes"],
                    segment["duration_sec"],
                )
            except Exception as exc:
                last_exc = exc
                if attempt >= 1 or not self._should_retry_segment(exc):
                    break
                logger.warning(
                    "机器人扬声器分段上传异常，准备重连后重试: index=%s attempt=%s error=%s",
                    segment["index"],
                    attempt + 1,
                    _format_exception_message(exc),
                )
                try:
                    self.client.disconnect()
                except Exception:
                    pass
                time.sleep(0.2)
        if last_exc is None:
            raise RuntimeError("unknown segment upload failure")
        raise last_exc

    @staticmethod
    def _should_retry_segment(exc: Exception) -> bool:
        message = _format_exception_message(exc).lower()
        return (
            "data channel is not open" in message
            or "timed out" in message
            or "timeout" in message
            or "closed" in message
        )


class RealtimeTTSPlayer:
    """本地 TTS 播放管理器。

    这个类只负责选择播放后端、接收音频块并驱动实际播放。
    它不再订阅本地 websocket，而是由本进程内的 Doubao 客户端直接调用。
    """

    def __init__(self, config: TTSPlayerConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._running = False
        self._current_session_id = None
        self._last_error = ""
        self._active_backend_mode = ""
        self._current_backend = None

    def start(self) -> bool:
        with self._lock:
            if self._running:
                logger.debug("本地TTS播放管理器已在运行")
                return False
            self._running = True
        logger.info("本地TTS播放管理器已启动")
        return True

    def stop(self, timeout: float = 5.0) -> bool:
        del timeout
        self._stop_backend(force=True)
        with self._lock:
            self._running = False
            self._current_session_id = None
        logger.info("本地TTS播放管理器已停止")
        return True

    def is_running(self) -> bool:
        return self._running

    def set_volume(self, volume: float) -> float:
        volume = _clamp_volume(volume)
        self.config.initial_volume = volume
        backend = self._current_backend
        if backend is not None:
            volume = backend.set_volume(volume)
        logger.debug("本地TTS播放软件音量设置为: %.2f", volume)
        return volume

    def begin_session(
        self,
        session_id: str,
        audio_format: str,
        sample_rate: int,
        channels: int = 1,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        del metadata
        if not self._running:
            self.start()
        with self._lock:
            previous_session_id = self._current_session_id
            self._current_session_id = session_id
        if previous_session_id and previous_session_id != session_id:
            logger.debug("检测到新的TTS会话，停止旧会话: old=%s new=%s", previous_session_id, session_id)
            self._stop_backend(force=True)
        self._start_backend_for_session(audio_format, sample_rate, channels)
        logger.debug(
            "TTS会话开始: %s backend=%s",
            session_id,
            self._active_backend_mode or self.config.backend_mode,
        )

    def write_audio(self, session_id: str, payload: bytes) -> None:
        if not payload:
            return
        if self._current_session_id != session_id:
            return
        if self._current_backend is None:
            return
        self._current_backend.write(payload)

    def finish_session(self, session_id: str, payload: Optional[Dict[str, Any]] = None) -> None:
        del payload
        if self._current_session_id != session_id:
            return
        self._finish_backend_for_session()
        backend_status = self._current_backend.status() if self._current_backend is not None else {}
        logger.debug(
            "TTS 会话结束",
            extra={
                "event": "tts.session.finished",
                "session_id": session_id,
                "bytes_written": backend_status.get("bytes_written"),
                "backend": backend_status.get("backend_name") or "-",
                "route_backend": backend_status.get("route_backend") or "-",
                "route_device": backend_status.get("route_device") or "-",
                "last_error_preview": compact_log_value(backend_status.get("last_error") or "-", max_length=240),
                "last_stderr_preview": compact_log_value(backend_status.get("last_stderr") or "-", max_length=240),
            },
        )
        with self._lock:
            self._current_session_id = None

    def interrupt_session(self, session_id: str, reason: str = "") -> None:
        if self._current_session_id != session_id:
            return
        logger.info("TTS会话被打断: session=%s reason=%s", session_id, reason or "-")
        self._stop_backend(force=True)
        with self._lock:
            self._current_session_id = None

    def handle_subtitle(self, session_id: str, payload: Dict[str, Any]) -> None:
        if self._current_session_id != session_id:
            return
        if self.config.print_subtitle:
            logger.debug(
                "TTS 字幕",
                extra={
                    "event": "tts.subtitle",
                    "session_id": session_id,
                    "subtitle_preview": compact_log_value(payload, max_length=240),
                },
            )

    def handle_sentence_end(self, session_id: str, payload: Dict[str, Any]) -> None:
        del payload
        if self._current_session_id != session_id:
            return
        if self._current_backend is None:
            return
        marker = getattr(self._current_backend, "mark_sentence_end", None)
        if callable(marker):
            try:
                marker()
            except Exception as exc:
                logger.warning("处理句子结束分段失败: session=%s error=%s", session_id, exc)

    def handle_error(self, session_id: str, message: str) -> None:
        self._last_error = message
        logger.error("TTS错误: session=%s error=%s", session_id, message)
        if self._current_session_id == session_id:
            self._stop_backend(force=True)
            with self._lock:
                self._current_session_id = None

    def status(self) -> Dict[str, Any]:
        backend_status = {}
        if self._current_backend is not None:
            backend_status = self._current_backend.status()
        with self._lock:
            return {
                "running": self.is_running(),
                "print_subtitle": self.config.print_subtitle,
                "configured_backend_mode": self.config.backend_mode,
                "fallback_backend_mode": self.config.fallback_backend_mode,
                "active_backend_mode": self._active_backend_mode or self.config.backend_mode,
                "local_playback": {
                    "preferred_backend": self.config.local.preferred_backend,
                    "audio_device": self.config.local.audio_device,
                    "enable_spk_ctl": self.config.local.enable_spk_ctl,
                    "enable_ape_route": self.config.local.enable_ape_route,
                    "ape_card": self.config.local.ape_card,
                    "ape_admaif": self.config.local.ape_admaif,
                    "ape_speaker": self.config.local.ape_speaker,
                },
                "robot_playback": {
                    "robot_ip": self.config.robot.robot_ip,
                    "connection_mode": self.config.robot.connection_mode,
                    "use_megaphone": self.config.robot.use_megaphone,
                    "streaming_enabled": self.config.robot.streaming_enabled,
                    "streaming_chunk_ms": self.config.robot.streaming_chunk_ms,
                    "auto_fallback_to_local": self.config.robot.auto_fallback_to_local,
                },
                "audio_sink": backend_status,
                "current_session_id": self._current_session_id,
                "last_error": self._last_error,
            }

    def _start_backend_for_session(self, audio_format: str, sample_rate: int, channels: int) -> None:
        primary_mode = self.config.backend_mode
        fallback_mode = self.config.fallback_backend_mode

        primary_error = None
        try:
            self._activate_backend(primary_mode)
            self._current_backend.start(audio_format=audio_format, sample_rate=sample_rate, channels=channels)
            return
        except Exception as exc:
            primary_error = exc
            logger.warning("主播放后端启动失败: mode=%s error=%s", primary_mode, exc)

        if not fallback_mode or fallback_mode == primary_mode:
            raise RuntimeError(str(primary_error))

        if primary_mode == "robot_webrtc" and not self.config.robot.auto_fallback_to_local:
            raise RuntimeError(str(primary_error))

        logger.info("尝试回退到兼容播放后端: %s", fallback_mode)
        self._activate_backend(fallback_mode)
        self._current_backend.start(audio_format=audio_format, sample_rate=sample_rate, channels=channels)

    def _finish_backend_for_session(self) -> None:
        if self._current_backend is None:
            return
        self._current_backend.finish()

    def _stop_backend(self, force: bool) -> None:
        if self._current_backend is None:
            return
        try:
            self._current_backend.stop(force=force)
        except Exception as exc:
            logger.warning("停止播放后端失败: %s", exc)

    def _activate_backend(self, mode: str) -> None:
        if self._current_backend is not None:
            try:
                self._current_backend.stop(force=True)
            except Exception:
                pass

        volume = self.config.initial_volume
        if self._current_backend is not None:
            try:
                volume = self._current_backend.get_volume()
            except Exception:
                pass

        if mode == "alsa_local":
            self._current_backend = LocalAudioBackend(self.config.local, volume=volume)
        elif mode == "robot_webrtc":
            self._current_backend = RobotSpeakerBackend(self.config.robot, volume=volume)
        else:
            raise RuntimeError("未知播放后端模式: %s" % mode)

        self._active_backend_mode = mode


def build_player_from_config(config: Optional[TTSPlayerConfig] = None) -> RealtimeTTSPlayer:
    player_config = replace(config or APP_CONFIG.tts)
    return RealtimeTTSPlayer(player_config)


def build_player_from_env() -> RealtimeTTSPlayer:
    """兼容旧入口，内部已改为统一配置。"""

    return build_player_from_config(APP_CONFIG.tts)
