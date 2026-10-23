from __future__ import annotations

from typing import Callable, Optional, Protocol, runtime_checkable

from contracts.navigation import NavigationGoal, NavigationState
from providers.base import BaseProvider


@runtime_checkable
class NavigationProvider(BaseProvider, Protocol):
    """导航后端统一接口。

    V1 暴露 set_goal/cancel_goal/get_navigation_state/is_goal_reached；
    V2 的 navigate/cancel/get_state 直接委托当前 geometry runtime gRPC backend。
    """

    # —— V1 既有接口 ——
    def set_goal(self, goal: NavigationGoal) -> bool:
        """提交一个非阻塞导航目标。"""

    def cancel_goal(self, request_id: Optional[str] = None) -> bool:
        """取消指定目标；省略标识时由后端取消当前目标。"""

    def get_navigation_state(self) -> NavigationState:
        """读取当前导航状态。"""

    def get_navigation_control_state(self, *, deadline_monotonic: Optional[float] = None) -> NavigationState:
        """读取后端控制状态，不附加展示与规划器就绪查询。"""

    def is_goal_reached(self) -> bool:
        """判断当前目标是否已到达。"""

    # —— V2 request-id 接口 ——
    nav_action_namespace: str = "/navigate_to_pose"

    def navigate(self, goal: NavigationGoal, timeout_sec: Optional[int] = None) -> str:
        """提交导航目标并返回 request_id。"""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        """取消指定 request_id 的导航；request_id 为空时取消当前 ACTIVE。"""

    def get_state(self, request_id: Optional[str] = None) -> NavigationState:
        """读取指定 request_id 的导航状态快照。"""


@runtime_checkable
class NavigationUpdateProvider(Protocol):
    """导航状态 push 是可选能力，独立声明以免旧 provider 被假实现。"""

    def subscribe_navigation_updates(
        self,
        callback: Callable[[NavigationState], None],
    ) -> Optional[int]:
        """订阅权威导航状态变化。"""

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        """取消导航状态订阅。"""
