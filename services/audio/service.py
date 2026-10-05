from __future__ import annotations

import math
import threading
from copy import deepcopy
from typing import TYPE_CHECKING, Dict, Optional

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from core import EventBus

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


class AudioService:
    """统一音量服务；显式设备操作串行，健康读取只消费已完成的快照。"""

    def __init__(self, *, config: NuwaxRobotBridgeConfig, robot, event_bus: Optional[EventBus] = None) -> None:
        self._config = config
        self._robot = robot
        self._event_bus = event_bus
        # 设备调用不能持有快照锁，否则健康读取仍会等待设备。
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._cached_state: Dict[str, object] = {
            "volume": None,
            "backend": "software",
            "last_error": "",
            "observed_at": None,
            "freshness": "unknown",
            "refresh_in_progress": False,
        }

    def stop(self) -> None:
        """仅有显式音量操作；不拥有音频采集或播放后台资源。"""
        return

    def get_cached_volume_state(self) -> Dict[str, object]:
        """不访问设备；时间戳仅代表最近一次真实结果，不声明其仍是当前音量。"""
        with self._state_lock:
            return deepcopy(self._cached_state)

    def _begin_operation(self) -> None:
        with self._state_lock:
            self._cached_state["refresh_in_progress"] = True

    def _finish_operation(self, *, robot_volume=None, software_volume=None, error=None) -> Dict[str, object]:
        # 复制及验证设备结果在快照锁外完成，不保留设备拥有的可变字典。
        observed = None
        if error is None and robot_volume is not None:
            try:
                observed = deepcopy(dict(robot_volume))
                volume = observed.get("volume")
                if volume is None or isinstance(volume, bool):
                    raise ValueError("设备音量结果缺少有效读回值。")
                volume = float(volume)
                if not math.isfinite(volume) or not 0.0 <= volume <= 1.0:
                    raise ValueError("设备音量结果必须是 0 到 1 之间的有限值。")
            except Exception as exc:  # noqa: BLE001 - Device adapters may raise backend-specific errors.
                error = str(exc)
                observed = None
        else:
            volume = software_volume
        with self._state_lock:
            state = self._cached_state
            state["refresh_in_progress"] = False
            if error is not None:
                state["last_error"] = str(error)
                state["robot_volume_error"] = str(error)
                state["freshness"] = "stale_after_error" if state["observed_at"] is not None else "unknown"
            elif observed is not None or software_volume is not None:
                state.pop("robot_volume_error", None)
                state["last_error"] = ""
                if observed is not None:
                    state["robot_volume"] = observed
                    state["backend"] = str(observed.get("backend") or "vui")
                else:
                    state.pop("robot_volume", None)
                    state["backend"] = "software"
                state["volume"] = volume
                state["observed_at"] = utc_now().isoformat()
                state["freshness"] = "unverified_since_observation"
            return deepcopy(state)

    def get_volume_state(self) -> Dict[str, object]:
        """显式读取设备音量；无设备能力时返回软件状态。"""
        with self._operation_lock:
            if not hasattr(self._robot, "get_vui_volume_info"):
                return self.get_cached_volume_state()
            self._begin_operation()
            try:
                result = self._robot.get_vui_volume_info()
            except Exception as exc:  # noqa: BLE001 - Device adapters may raise backend-specific errors.
                return self._finish_operation(error=str(exc))
            return self._finish_operation(robot_volume=result if result is not None else {})

    def set_volume(self, volume: float) -> Dict[str, object]:
        """显式设置音量；仅真实读回结果或软件后端成功更新进入快照。"""
        normalized = max(0.0, min(1.0, float(volume)))
        with self._operation_lock:
            self._begin_operation()
            if hasattr(self._robot, "set_vui_volume_ratio"):
                try:
                    robot_volume = self._robot.set_vui_volume_ratio(
                        normalized,
                        self._config.volume.auto_enable_vui_switch,
                    )
                except Exception as exc:  # noqa: BLE001 - Device adapters may raise backend-specific errors.
                    result = self._finish_operation(error=str(exc))
                else:
                    result = self._finish_operation(robot_volume=robot_volume if robot_volume is not None else {})
            else:
                result = self._finish_operation(software_volume=normalized)
        self._publish_audio_event(
            "audio.volume_changed",
            "音量已更新。",
            payload={"volume": normalized, "backend": result.get("backend")},
        )
        return result

    def is_volume_available(self) -> bool:
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
