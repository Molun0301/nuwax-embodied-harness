from __future__ import annotations

from contracts.events import RuntimeEventCategory
from core import EventBus
from typing import Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    # settings 是横跨全平台的 god 配置模块；服务层不得在运行时硬依赖它（治理规则 R6）。
    # 这里仅用于构造函数 config 形参的类型注解，文件头 `from __future__ import annotations`
    # 已保证注解惰性求值，运行时不会触发 settings 导入。
    from core.settings import NuwaxRobotBridgeConfig


class AudioService:
    """统一机器人音量服务。"""

    def __init__(
        self,
        *,
        config: NuwaxRobotBridgeConfig,
        robot,
        event_bus: Optional[EventBus] = None,
    ) -> None:
        self._config = config
        self._robot = robot
        self._event_bus = event_bus
        self._software_volume: Optional[float] = None
        self._last_error = ""

    def stop(self) -> None:
        """停止音频运行时。"""

        # 仅保留音量控制，无音频播放运行时资源需要释放。
        return

    def get_volume_state(self) -> Dict[str, object]:
        """读取当前音量状态。"""

        result: Dict[str, object] = {
            "volume": self._software_volume,
            "backend": "software",
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

    def is_volume_available(self) -> bool:
        """判断音量控制是否可用。"""

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
