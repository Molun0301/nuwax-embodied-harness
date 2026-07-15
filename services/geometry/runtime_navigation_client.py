"""runtime_navigation_client — platform 侧 navigation 串行锁 + 状态机。

统一使用 ``runtime_navigation_client``：
- 通过 ``GeometryRuntimeProvider`` 走 ``SetNavigationGoal`` / ``CancelNavigation`` /
  ``SubscribeNavigationState`` 等 gRPC facade。
- 串行锁 / 状态机 / 错误码契约与 design.md 决策 D1 + D3 保持一致。

Phase 3 目标态 import 路径：``from services.geometry.runtime_navigation_client import ...``
"""
from __future__ import annotations

import asyncio
import logging
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from contracts.geometry import Pose
from contracts.navigation import NavigationGoal
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime_navigation_client")

_DEFAULT_PENDING_QUEUE_MAX_SIZE = 4
_DEFAULT_TIMEOUT_SEC = 60
# 状态机数字值与 design.md 附录 K proto NavigationStatus 一一对应
_STATE_PENDING = 1
_STATE_ACTIVE = 2
_STATE_SUCCEEDED = 3
_STATE_FAILED = 4
_STATE_CANCELED = 5
_STATE_TERMINAL = frozenset((_STATE_SUCCEEDED, _STATE_FAILED, _STATE_CANCELED))


class NavigationFailureCode(str, Enum):
    """导航失败原因枚举；与 design.md NavigationFailureCode 一一对应。"""

    UNKNOWN = "UNKNOWN"
    TIMEOUT = "TIMEOUT"
    OBSTACLE = "OBSTACLE"
    NO_PATH = "NO_PATH"
    CANCELED = "CANCELED"
    QUEUE_FULL = "QUEUE_FULL"
    PROXY_UNAVAILABLE = "PROXY_UNAVAILABLE"


@dataclass
class NavigationError:
    """平台侧统一的导航错误结构。"""

    code: NavigationFailureCode
    message: str
    recoverable: bool = True

    def to_dict(self) -> Dict[str, object]:
        return {"code": self.code.value, "message": self.message, "recoverable": self.recoverable}


@dataclass
class RuntimeNavigationRequest:
    """本地 dataclass 形式的 navigation goal 请求。"""

    request_id: str
    pose: Pose
    timeout_sec: int = _DEFAULT_TIMEOUT_SEC

    @classmethod
    def create(
        cls,
        pose: Pose,
        *,
        timeout_sec: int = _DEFAULT_TIMEOUT_SEC,
        request_id: Optional[str] = None,
    ) -> "RuntimeNavigationRequest":
        return cls(
            request_id=request_id or str(uuid.uuid4()),
            pose=pose,
            timeout_sec=timeout_sec,
        )


@dataclass
class NavigationStateUpdate:
    """本地 dataclass 形式的 navigation state update。"""

    state: int
    request_id: str
    timestamp_ms: int
    message: str = ""
    failure: Optional[NavigationError] = None
    pose: Optional[Pose] = None

    def is_terminal(self) -> bool:
        return self.state in _STATE_TERMINAL

    def to_dict(self) -> Dict[str, object]:
        return {
            "state": self.state,
            "request_id": self.request_id,
            "timestamp_ms": self.timestamp_ms,
            "message": self.message,
            "failure": self.failure.to_dict() if self.failure is not None else None,
        }


class _RuntimeUnavailable(Exception):
    """GeometryRuntimeProvider 不可用 / gRPC 失败。"""


class RuntimeNavigationClient:
    """platform 侧 navigation 串行锁客户端。

    通过正式 GeometryRuntimeProvider 接口完成导航事务：
    - ``navigate()``：通过正式 ``GeometryRuntimeProvider.set_goal`` 发 exact goal；
    - ``cancel()``：通过兼容入口转入正式 ``cancel_goal`` active-goal 栅栏；
    - ``subscribe_state()``：通过 ``GeometryRuntimeProvider.subscribe_navigation_state`` 拉流。
    """

    def __init__(
        self,
        provider: Optional[GeometryRuntimeProvider] = None,
        *,
        pending_queue_max_size: int = _DEFAULT_PENDING_QUEUE_MAX_SIZE,
    ) -> None:
        self._provider = provider
        self._pending_queue_max_size = pending_queue_max_size
        self._lock = asyncio.Lock()
        self._pending: "asyncio.Queue[Tuple[RuntimeNavigationRequest, asyncio.Future[NavigationStateUpdate], asyncio.Event]]" = asyncio.Queue()
        self._active: Optional[asyncio.Task] = None
        self._state_subscribers: List[Callable[[NavigationStateUpdate], None]] = []
        self._closed = False

    def attach_provider(self, provider: GeometryRuntimeProvider) -> None:
        """绑定/重新绑定 GeometryRuntimeProvider。"""
        self._provider = provider

    def subscribe_state(self, callback: Callable[[NavigationStateUpdate], None]) -> None:
        """注册状态回调；只增不减，调用方负责生命周期。"""
        self._state_subscribers.append(callback)

    async def navigate(self, request: RuntimeNavigationRequest) -> NavigationStateUpdate:
        if self._closed:
            raise RuntimeError("RuntimeNavigationClient 已关闭。")
        future: asyncio.Future[NavigationStateUpdate] = asyncio.get_event_loop().create_future()
        event = asyncio.Event()
        await self._pending.put((request, future, event))
        if self._pending.qsize() > self._pending_queue_max_size:
            try:
                self._pending.get_nowait()
            except asyncio.QueueEmpty:
                pass
            future.set_exception(
                NavigationError(
                    code=NavigationFailureCode.QUEUE_FULL,
                    message="PENDING 队列已满",
                    recoverable=True,
                )
            )
            raise _queue_full_error()
        return await future

    async def cancel(self, request_id: str) -> None:
        if self._closed:
            return
        provider = self._provider
        if provider is None:
            return
        exact_goal_id = str(request_id or "").strip()
        if not exact_goal_id:
            return
        provider.cancel_navigation(exact_goal_id)

    async def shutdown(self) -> None:
        self._closed = True
        if self._active is not None and not self._active.done():
            self._active.cancel()
            try:
                await self._active
            except (asyncio.CancelledError, Exception):
                pass
            self._active = None

    async def drive(self) -> None:
        """单 worker 串行消费 PENDING → ACTIVE → 终态。"""
        while not self._closed:
            request, future, event = await self._pending.get()
            if self._closed:
                future.set_exception(RuntimeError("RuntimeNavigationClient 已关闭。"))
                continue
            try:
                update = await self._submit_active(request)
            except Exception as exc:  # noqa: BLE001
                update = NavigationStateUpdate(
                    state=_STATE_FAILED,
                    request_id=request.request_id,
                    timestamp_ms=int(time.time() * 1000),
                    message=str(exc),
                    failure=NavigationError(
                        code=NavigationFailureCode.PROXY_UNAVAILABLE,
                        message=str(exc),
                    ),
                )
            event.set()
            if not future.done():
                future.set_result(update)
            self._dispatch_state(update)

    async def _submit_active(self, request: RuntimeNavigationRequest) -> NavigationStateUpdate:
        provider = self._provider
        if provider is None:
            raise _RuntimeUnavailable("GeometryRuntimeProvider 未绑定。")
        accepted = provider.set_goal(
            NavigationGoal(
                goal_id=str(request.request_id or "").strip(),
                target_pose=request.pose,
            )
        )
        if not accepted:
            raise _RuntimeUnavailable("SetNavigationGoal 被 geometry provider 拒绝。")
        return NavigationStateUpdate(
            state=_STATE_ACTIVE,
            request_id=request.request_id,
            timestamp_ms=int(time.time() * 1000),
            message="目标已下发，等待 runtime 推送状态。",
        )

    def _dispatch_state(self, update: NavigationStateUpdate) -> None:
        for callback in list(self._state_subscribers):
            try:
                callback(update)
            except Exception as exc:  # noqa: BLE001
                LOGGER.debug("runtime_navigation_client state callback 异常：%s", exc, exc_info=True)


def _queue_full_error() -> RuntimeError:
    return RuntimeError(
        "%s"
        % NavigationError(
            code=NavigationFailureCode.QUEUE_FULL,
            message="PENDING 队列已满",
            recoverable=True,
        ).to_dict()
    )


_DEFAULT_CLIENT: Optional[RuntimeNavigationClient] = None


def get_default_client() -> RuntimeNavigationClient:
    global _DEFAULT_CLIENT
    if _DEFAULT_CLIENT is None:
        _DEFAULT_CLIENT = RuntimeNavigationClient()
    return _DEFAULT_CLIENT
