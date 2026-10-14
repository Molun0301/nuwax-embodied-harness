"""平台导航状态机。

定义平台导航会话的状态转换逻辑，将后端状态映射到 NavigationStatus。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Optional, Any

from contracts.base import utc_now
from contracts.navigation import NavigationStatus


class NavigationStateMachine:
    """平台导航状态机。

    管理平台导航目标生命周期，支持以下状态转换：
    - IDLE -> ACCEPTED: 目标被接受
    - ACCEPTED -> PLANNING: 开始规划路径
    - PLANNING -> RUNNING: 开始执行
    - RUNNING -> SUCCEEDED: 导航完成
    - RUNNING -> FAILED: 导航失败
    - RUNNING -> CANCELLED: 导航取消
    - 任意 -> IDLE: 重置

    后端状态统一映射到平台 IDLE / ACCEPTED / RUNNING / SUCCEEDED /
    CANCELLED / FAILED。
    """

    def __init__(self) -> None:
        self._current_status: NavigationStatus = NavigationStatus.IDLE
        self._status_message: Optional[str] = None
        self._start_time: Optional[datetime] = None
        self._end_time: Optional[datetime] = None
        self._current_goal_id: Optional[str] = None
        self._recovery_attempts: int = 0
        self._last_error: Optional[str] = None
        self._nav2_state: Optional[str] = None

    @property
    def current_status(self) -> NavigationStatus:
        """返回当前导航状态。"""

        return self._current_status

    @property
    def current_goal_id(self) -> Optional[str]:
        """返回当前目标ID。"""

        return self._current_goal_id

    @property
    def is_running(self) -> bool:
        """判断导航是否正在运行。"""

        return self._current_status in {
            NavigationStatus.ACCEPTED,
            NavigationStatus.PLANNING,
            NavigationStatus.RUNNING,
        }

    @property
    def is_terminal(self) -> bool:
        """判断导航是否到达终态。"""

        return self._current_status in {
            NavigationStatus.SUCCEEDED,
            NavigationStatus.FAILED,
            NavigationStatus.CANCELLED,
        }

    @property
    def elapsed_time_sec(self) -> float:
        """返回导航已运行时间（秒）。"""

        if self._start_time is None:
            return 0.0
        end = self._end_time if self._end_time else utc_now()
        return (end - self._start_time).total_seconds()

    def set_goal(self, goal_id: str, message: str = "导航目标已接受。") -> NavigationStatus:
        """设置导航目标，转移到 ACCEPTED 状态。"""

        self._current_status = NavigationStatus.ACCEPTED
        self._status_message = message
        self._current_goal_id = goal_id
        self._start_time = utc_now()
        self._end_time = None
        self._recovery_attempts = 0
        self._last_error = None
        self._nav2_state = "accepted"
        return self._current_status

    def start_planning(self, message: str = "开始路径规划。") -> NavigationStatus:
        """开始路径规划，转移到 PLANNING 状态。"""

        if self._current_status == NavigationStatus.ACCEPTED:
            self._current_status = NavigationStatus.PLANNING
            self._status_message = message
        return self._current_status

    def start_running(self, message: str = "开始执行导航。") -> NavigationStatus:
        """开始执行导航，转移到 RUNNING 状态。"""

        if self._current_status in {NavigationStatus.ACCEPTED, NavigationStatus.PLANNING}:
            self._current_status = NavigationStatus.RUNNING
            self._status_message = message
        return self._current_status

    def succeed(self, message: str = "导航目标已到达。") -> NavigationStatus:
        """导航成功，转移到 SUCCEEDED 状态。"""

        if self._current_status in {
            NavigationStatus.ACCEPTED,
            NavigationStatus.PLANNING,
            NavigationStatus.RUNNING,
        }:
            self._current_status = NavigationStatus.SUCCEEDED
            self._status_message = message
            self._end_time = utc_now()
        return self._current_status

    def fail(self, error_message: str = "导航失败。") -> NavigationStatus:
        """导航失败，转移到 FAILED 状态。"""

        self._current_status = NavigationStatus.FAILED
        self._status_message = error_message
        self._last_error = error_message
        self._end_time = utc_now()
        return self._current_status

    def cancel(self, message: str = "导航已取消。") -> NavigationStatus:
        """取消导航，转移到 CANCELLED 状态。"""

        if self._current_status in {
            NavigationStatus.ACCEPTED,
            NavigationStatus.PLANNING,
            NavigationStatus.RUNNING,
        }:
            self._current_status = NavigationStatus.CANCELLED
            self._status_message = message
            self._end_time = utc_now()
        return self._current_status

    def enter_recovery(self, message: str = "进入恢复行为。") -> NavigationStatus:
        """进入恢复状态。"""

        self._recovery_attempts += 1
        self._status_message = message
        return self._current_status

    def reset(self) -> NavigationStatus:
        """重置状态机到 IDLE。"""

        self._current_status = NavigationStatus.IDLE
        self._status_message = None
        self._start_time = None
        self._end_time = None
        self._current_goal_id = None
        self._recovery_attempts = 0
        self._last_error = None
        self._nav2_state = None
        return self._current_status

    def update_from_nav2_state(
        self,
        nav2_status: str,
        message: Optional[str] = None,
        goal_reached: bool = False,
    ) -> NavigationStatus:
        """根据后端状态更新平台状态。

        状态映射：
        - idle -> IDLE
        - accepted -> ACCEPTED
        - executing -> RUNNING
        - succeeded -> SUCCEEDED
        - canceled/cancelled -> CANCELLED
        - failed -> FAILED
        - recovery -> RUNNING (with recovery flag)
        """

        status_map = {
            "idle": NavigationStatus.IDLE,
            "accepted": NavigationStatus.ACCEPTED,
            "executing": NavigationStatus.RUNNING,
            "succeeded": NavigationStatus.SUCCEEDED,
            "canceled": NavigationStatus.CANCELLED,
            "cancelled": NavigationStatus.CANCELLED,
            "failed": NavigationStatus.FAILED,
            "planning": NavigationStatus.PLANNING,
            "running": NavigationStatus.RUNNING,
            "recovery": NavigationStatus.RUNNING,
        }

        normalized = nav2_status.strip().lower()
        self._nav2_state = normalized

        if normalized == "recovery":
            self.enter_recovery(message or "导航后端执行恢复行为。")
            return self._current_status

        new_status = status_map.get(normalized)
        if new_status is None:
            # 未知状态，保持当前状态
            return self._current_status

        # 根据当前状态和后端状态决定状态转移
        if new_status == NavigationStatus.SUCCEEDED and goal_reached:
            return self.succeed(message or "导航目标已到达。")

        if new_status == NavigationStatus.FAILED:
            return self.fail(message or "导航失败。")

        if new_status == NavigationStatus.CANCELLED:
            return self.cancel(message or "导航已取消。")

        if new_status == NavigationStatus.RUNNING:
            return self.start_running(message or "开始执行导航。")

        if new_status == NavigationStatus.PLANNING:
            return self.start_planning(message or "开始路径规划。")

        if new_status == NavigationStatus.ACCEPTED and self._current_status == NavigationStatus.IDLE:
            return self.set_goal(self._current_goal_id or "unknown", message or "目标已接受。")

        self._current_status = new_status
        if message:
            self._status_message = message
        return self._current_status

    def update_from_backend_state(
        self,
        backend_name: str,
        backend_status: str,
        message: Optional[str] = None,
        goal_reached: bool = False,
    ) -> NavigationStatus:
        """根据通用后端状态更新平台状态。"""

        # 所有后端使用同一状态表。
        status_map = {
            "idle": NavigationStatus.IDLE,
            "accepted": NavigationStatus.ACCEPTED,
            "goal_set": NavigationStatus.ACCEPTED,
            "planning": NavigationStatus.PLANNING,
            "executing": NavigationStatus.RUNNING,
            "running": NavigationStatus.RUNNING,
            "reached": NavigationStatus.SUCCEEDED,
            "succeeded": NavigationStatus.SUCCEEDED,
            "canceled": NavigationStatus.CANCELLED,
            "cancelled": NavigationStatus.CANCELLED,
            "failed": NavigationStatus.FAILED,
            "completed": NavigationStatus.SUCCEEDED,
            "recovery": NavigationStatus.RUNNING,
        }

        normalized = backend_status.strip().lower()
        new_status = status_map.get(normalized)
        if new_status is None:
            return self._current_status

        if new_status == NavigationStatus.SUCCEEDED and goal_reached:
            return self.succeed(message or "导航目标已到达。")
        if new_status == NavigationStatus.FAILED:
            return self.fail(message or "导航失败。")
        if new_status == NavigationStatus.CANCELLED:
            return self.cancel(message or "导航已取消。")
        if new_status == NavigationStatus.RUNNING:
            if normalized == "recovery":
                self.enter_recovery(message or "导航后端进入恢复行为。")
            return self.start_running(message or "开始执行导航。")
        if new_status == NavigationStatus.PLANNING:
            return self.start_planning(message or "开始路径规划。")
        if new_status == NavigationStatus.ACCEPTED and self._current_status == NavigationStatus.IDLE:
            return self.set_goal(self._current_goal_id or "unknown", message or "目标已接受。")

        self._current_status = new_status
        if message:
            self._status_message = message
        return self._current_status

    def get_state_summary(self) -> Dict[str, Any]:
        """返回状态机摘要。"""

        return {
            "status": self._current_status.value,
            "message": self._status_message,
            "goal_id": self._current_goal_id,
            "is_running": self.is_running,
            "is_terminal": self.is_terminal,
            "elapsed_time_sec": self.elapsed_time_sec,
            "recovery_attempts": self._recovery_attempts,
            "last_error": self._last_error,
            "backend_state": self._nav2_state,
            "start_time": self._start_time.isoformat() if self._start_time else None,
            "end_time": self._end_time.isoformat() if self._end_time else None,
        }


@dataclass
class NavigationDiagnostics:
    """平台导航诊断信息。"""

    backend_name: str
    backend_status: str
    platform_status: NavigationStatus
    goal_id: Optional[str] = None
    remaining_distance_m: Optional[float] = None
    remaining_yaw_rad: Optional[float] = None
    recovery_in_progress: bool = False
    last_error: Optional[str] = None
    metadata: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典。"""

        return {
            "backend_name": self.backend_name,
            "backend_status": self.backend_status,
            "platform_status": self.platform_status.value,
            "goal_id": self.goal_id,
            "remaining_distance_m": self.remaining_distance_m,
            "remaining_yaw_rad": self.remaining_yaw_rad,
            "recovery_in_progress": self.recovery_in_progress,
            "last_error": self.last_error,
            "metadata": dict(self.metadata),
        }

    def model_dump(self, mode: str = "json") -> Dict[str, Any]:
        """Pydantic 兼容的序列化方法。"""

        return self.to_dict()


def create_navigation_state_machine() -> NavigationStateMachine:
    """创建默认的导航状态机实例。"""

    return NavigationStateMachine()


def map_nav2_status_to_platform(nav2_status: str) -> NavigationStatus:
    """将后端状态映射到平台 NavigationStatus。"""

    status_map = {
        "idle": NavigationStatus.IDLE,
        "accepted": NavigationStatus.ACCEPTED,
        "executing": NavigationStatus.RUNNING,
        "succeeded": NavigationStatus.SUCCEEDED,
        "canceled": NavigationStatus.CANCELLED,
        "cancelled": NavigationStatus.CANCELLED,
        "failed": NavigationStatus.FAILED,
        "planning": NavigationStatus.PLANNING,
        "running": NavigationStatus.RUNNING,
    }
    return status_map.get(nav2_status.strip().lower(), NavigationStatus.IDLE)
