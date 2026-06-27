from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from contracts.navigation import NavigationGoal, NavigationState
from providers.base import BaseProvider


@runtime_checkable
class NavigationProvider(BaseProvider, Protocol):
    """导航后端统一接口。

    V1 仅暴露 set_goal/cancel_goal/get_navigation_state/is_goal_reached；
    nav2_action_proxy 接入后扩展为 navigate/cancel/get_state 三方法，
    并暴露 nav_action_namespace 字段以便多机器人 namespace 复用。
    """

    # —— V1 既有接口 ——
    def set_goal(self, goal: NavigationGoal) -> bool:
        """提交一个非阻塞导航目标。"""

    def cancel_goal(self) -> bool:
        """取消当前导航目标。"""

    def get_navigation_state(self) -> NavigationState:
        """读取当前导航状态。"""

    def is_goal_reached(self) -> bool:
        """判断当前目标是否已到达。"""

    # —— V2 新增（nav2_action_proxy 接入） ——
    nav_action_namespace: str = "/navigate_to_pose"

    def navigate(self, goal: NavigationGoal, timeout_sec: Optional[int] = None) -> str:
        """提交导航目标（Nav2 action proxy gRPC），返回 request_id。"""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        """取消指定 request_id 的导航；request_id 为空时取消当前 ACTIVE。"""

    def get_state(self, request_id: Optional[str] = None) -> NavigationState:
        """读取指定 request_id 的导航状态快照。"""
