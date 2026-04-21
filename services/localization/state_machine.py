"""平台定位状态机。

定义平台定位会话的状态转换逻辑，支持 IDLE -> LOCALIZING -> READY -> STALE/FAILED 的完整生命周期。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timedelta
from enum import Enum
from typing import Dict, Optional, Set, Any

from contracts.base import utc_now


class LocalizationStateTransition(Enum):
    """定位状态转换类型。"""

    START = "start"
    POSE_RECEIVED = "pose_received"
    TRACKING_LOST = "tracking_lost"
    RECOVERED = "recovered"
    RELOCALIZE = "relocalize"
    FAILED = "failed"
    RESET = "reset"


@dataclass
class LocalizationDiagnostics:
    """平台定位诊断信息。"""

    backend_name: str
    pose_available: bool
    tracking_quality: float = 0.0
    time_since_last_pose_sec: float = 0.0
    backend_state: Optional[str] = None
    tracking_lost_count: int = 0
    relocalization_attempts: int = 0
    last_error: Optional[str] = None
    metadata: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典（兼容 Pydantic model_dump）。"""

        return {
            "backend_name": self.backend_name,
            "pose_available": self.pose_available,
            "tracking_quality": self.tracking_quality,
            "time_since_last_pose_sec": self.time_since_last_pose_sec,
            "backend_state": self.backend_state,
            "tracking_lost_count": self.tracking_lost_count,
            "relocalization_attempts": self.relocalization_attempts,
            "last_error": self.last_error,
            "metadata": dict(self.metadata),
        }

    def model_dump(self, mode: str = "json") -> Dict[str, Any]:
        """Pydantic 兼容的序列化方法。"""

        return self.to_dict()


@dataclass
class LocalizationStateMachine:
    """平台定位状态机。

    管理 LocalizationSession 生命周期，支持以下状态转换：
    - IDLE -> LOCALIZING: 开始定位
    - LOCALIZING -> READY: 收到有效位姿
    - READY -> STALE: 跟踪丢失（可恢复）
    - STALE -> READY: 重新恢复跟踪
    - STALE -> FAILED: 恢复失败
    - LOCALIZING -> FAILED: 定位失败
    - 任意状态 -> LOCALIZING: 重新定位
    """

    # 状态转移超时配置
    STALE_TIMEOUT_SEC: float = 5.0  # 超过此时间无位姿更新则进入 STALE
    FAILED_TIMEOUT_SEC: float = 30.0  # STALE 状态超过此时间则进入 FAILED

    # 跟踪质量阈值
    MIN_TRACKING_QUALITY_FOR_READY: float = 0.3
    MIN_TRACKING_QUALITY_FOR_STALE: float = 0.1

    def __init__(self) -> None:
        self._current_state: str = "idle"  # LocalizationSessionStatus
        self._last_pose_timestamp: Optional[datetime] = None
        self._last_pose_available: bool = False
        self._tracking_quality: float = 0.0
        self._tracking_lost_count: int = 0
        self._relocalization_attempts: int = 0
        self._stale_since: Optional[datetime] = None
        self._failed_since: Optional[datetime] = None
        self._backend_state: Optional[str] = None

    @property
    def current_state(self) -> str:
        """返回当前状态。"""

        return self._current_state

    def start(self) -> str:
        """开始定位流程，转移到 LOCALIZING 状态。"""

        self._relocalization_attempts += 1
        self._stale_since = None
        self._failed_since = None
        self._current_state = "localizing"
        return self._current_state

    def pose_received(
        self,
        pose_available: bool,
        tracking_quality: float = 1.0,
        backend_state: Optional[str] = None,
    ) -> str:
        """收到位姿数据，更新状态。"""

        self._last_pose_available = pose_available
        self._tracking_quality = tracking_quality
        self._backend_state = backend_state

        if pose_available:
            self._last_pose_timestamp = utc_now()
            self._tracking_lost_count = 0
            self._stale_since = None
            self._failed_since = None

            # 根据跟踪质量决定最终状态
            if tracking_quality >= self.MIN_TRACKING_QUALITY_FOR_READY:
                self._current_state = "ready"
            elif tracking_quality >= self.MIN_TRACKING_QUALITY_FOR_STALE:
                self._current_state = "stale"
            else:
                self._current_state = "localizing"
        else:
            # 无位姿，根据之前状态决定
            if self._current_state == "ready":
                self._tracking_lost_count += 1
                # 进入 STALE 状态开始计时
                if self._stale_since is None:
                    self._stale_since = utc_now()
                self._current_state = "stale"

        return self._current_state

    def check_timeouts(self) -> str:
        """检查超时并更新状态。"""

        now = utc_now()

        # 检查 STALE -> FAILED 超时
        if self._current_state == "stale" and self._stale_since is not None:
            elapsed = (now - self._stale_since).total_seconds()
            if elapsed >= self.FAILED_TIMEOUT_SEC:
                self._failed_since = now
                self._current_state = "failed"

        # 检查是否应该从 STALE 恢复
        if self._current_state == "stale" and self._last_pose_available:
            elapsed_since_stale = (now - self._stale_since).total_seconds() if self._stale_since else 0
            if elapsed_since_stale < self.STALE_TIMEOUT_SEC and self._tracking_quality >= self.MIN_TRACKING_QUALITY_FOR_READY:
                self._current_state = "ready"
                self._stale_since = None

        return self._current_state

    def relocalize(self) -> str:
        """执行重定位，转移到 LOCALIZING 状态。"""

        return self.start()

    def reset(self) -> str:
        """重置状态机到 IDLE。"""

        self._current_state = "idle"
        self._last_pose_timestamp = None
        self._last_pose_available = False
        self._tracking_quality = 0.0
        self._tracking_lost_count = 0
        self._relocalization_attempts = 0
        self._stale_since = None
        self._failed_since = None
        self._backend_state = None
        return self._current_state

    def get_diagnostics(
        self,
        backend_name: str,
        last_error: Optional[str] = None,
    ) -> LocalizationDiagnostics:
        """返回当前诊断信息。"""

        now = utc_now()
        time_since_last_pose = 0.0
        if self._last_pose_timestamp is not None:
            time_since_last_pose = (now - self._last_pose_timestamp).total_seconds()

        return LocalizationDiagnostics(
            backend_name=backend_name,
            pose_available=self._last_pose_available,
            tracking_quality=self._tracking_quality,
            time_since_last_pose_sec=time_since_last_pose,
            backend_state=self._backend_state,
            tracking_lost_count=self._tracking_lost_count,
            relocalization_attempts=self._relocalization_attempts,
            last_error=last_error,
            metadata={
                "current_state": self._current_state,
                "stale_since": self._stale_since.isoformat() if self._stale_since else None,
                "failed_since": self._failed_since.isoformat() if self._failed_since else None,
            },
        )

    def is_ready_for_navigation(self) -> bool:
        """判断当前是否可用于导航。"""

        return self._current_state == "ready"


def create_localization_state_machine() -> LocalizationStateMachine:
    """创建默认的定位状态机实例。"""

    return LocalizationStateMachine()
