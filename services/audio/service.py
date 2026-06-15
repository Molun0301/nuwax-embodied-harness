from __future__ import annotations

from collections import deque
import threading
import time

from contracts.events import RuntimeEventCategory
from core import EventBus
from typing import Deque, Dict, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    # settings 是横跨全平台的 god 配置模块；服务层不得在运行时硬依赖它（治理规则 R6）。
    # 这里仅用于构造函数 config 形参的类型注解，文件头 `from __future__ import annotations`
    # 已保证注解惰性求值，运行时不会触发 settings 导入。
    from core.settings import NuwaxRobotBridgeConfig


class AudioService:
    """统一音量与文本播报服务。

    实时语音（豆包 WebRTC TTS）已随 tts 包一并下线，文本播报固定走记录模式。
    """

    def __init__(
        self,
        *,
        config: NuwaxRobotBridgeConfig,
        robot,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 50,
    ) -> None:
        self._config = config
        self._robot = robot
        self._event_bus = event_bus
        self._lock = threading.RLock()
        self._software_volume = max(0.0, min(1.0, float(config.tts.initial_volume)))
        self._speech_history: Deque[Dict[str, object]] = deque(maxlen=max(1, history_limit))
        self._last_error = ""

    def stop(self) -> None:
        """停止音频运行时。"""

        # 实时语音后端已下线，无运行时资源需要释放。
        return

    def get_volume_state(self) -> Dict[str, object]:
        """读取当前音量状态。"""

        result: Dict[str, object] = {
            "volume": self._software_volume,
            "backend": "software",
            "realtime_speech_available": False,
            "last_error": self._last_error,
        }
        if hasattr(self._robot, "get_vui_volume_info"):
            try:
                robot_volume = self._robot.get_vui_volume_info()
                result["robot_volume"] = robot_volume
                result["volume"] = float(robot_volume.get("volume", result["volume"]))
                result["backend"] = str(robot_volume.get("backend") or "vui")
            except Exception as exc:
                result["robot_volume_error"] = str(exc)
        return result

    def set_volume(self, volume: float) -> Dict[str, object]:
        """设置当前音量。"""

        normalized = max(0.0, min(1.0, float(volume)))
        self._software_volume = normalized
        result: Dict[str, object] = {"volume": normalized, "backend": "software"}

        if hasattr(self._robot, "set_vui_volume_ratio"):
            try:
                result["robot_volume"] = self._robot.set_vui_volume_ratio(
                    normalized,
                    self._config.volume.auto_enable_vui_switch,
                )
                result["backend"] = "vui"
            except Exception as exc:
                self._last_error = str(exc)
                result["robot_volume_error"] = str(exc)
        self._publish_audio_event(
            "audio.volume_changed",
            "音量已更新。",
            payload={"volume": normalized, "backend": result.get("backend")},
        )
        return result

    def speak_text(
        self,
        text: str,
        *,
        session_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """提交一次文本播报。"""

        content = str(text or "").strip()
        if not content:
            raise ValueError("播报文本不能为空。")

        requested_session_id = session_id or f"speech_{time.time_ns()}"
        result: Dict[str, object] = {
            "accepted": True,
            "session_id": requested_session_id,
            "text": content,
            "mode": "record_only",
            "realtime_started": False,
        }

        record = {
            **result,
            "metadata": dict(metadata or {}),
            "timestamp_ms": int(time.time() * 1000),
        }
        self._speech_history.append(record)
        self._publish_audio_event(
            "audio.speech_requested",
            "已接收文本播报请求。",
            payload={"session_id": result["session_id"], "mode": result["mode"]},
        )
        return record

    def list_speech_history(self, *, limit: Optional[int] = None) -> Tuple[Dict[str, object], ...]:
        """列出近期播报记录。"""

        items = list(self._speech_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def is_volume_available(self) -> bool:
        """判断音量控制是否可用。"""

        return True

    def is_speech_available(self) -> bool:
        """判断文本播报是否可用。

        实时语音后端已下线，播报固定走记录模式，因此始终返回 True。
        """

        return True

    def _publish_audio_event(self, event_type: str, message: str, *, payload: Dict[str, object]) -> None:
        if self._event_bus is None:
            return
        self._event_bus.publish(
            self._event_bus.build_event(
                event_type,
                category=RuntimeEventCategory.SYSTEM,
                source="audio_service",
                message=message,
                payload=payload,
            )
        )
