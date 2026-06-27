"""navigation_proxy_client：platform 侧 gRPC client + 串行锁 + 状态机。

设计要点（openspec/changes/navigation-and-exploration-boundary/design.md 决策 D1 + D3）：

* 进程位置：与网关同进程（fastrtps），与独立 Nav2 进程（cyclonedds）通过 gRPC over TCP
  loopback 通信（跨 RMW 桥接；不违反 services 禁 rclpy 直订 Nav2 的硬约束）。
* 串行锁：``asyncio.Lock`` 串行化所有进入的 ``navigate()`` 请求；并发 → 队列入 PENDING
  （上限 4），空闲 → 直接 ACTIVE。
* 状态机：PENDING → ACTIVE → 终态（SUCCEEDED / FAILED / CANCELED）；单向流转不回退。
* 错误码契约：``NavigationFailureCode`` 7 值（UNKNOWN / TIMEOUT / OBSTACLE / NO_PATH /
  CANCELED / QUEUE_FULL / PROXY_UNAVAILABLE）。

限制（CI 守门硬约束）：

* 本文件禁 ``import rclpy`` / ``import rclpy.action`` / ``from rclpy...`` —— services/geometry/
  守门规则。
* 本文件禁 drivers 包 import —— services 层禁 drivers。
* Py3.8.10 兼容：禁 PEP 604 union（``X | Y`` 在类型注解）；用 ``Union[X, Y]`` 或 ``Optional[X]``。
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from contracts.geometry import Pose

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.navigation_proxy_client")

_DEFAULT_GRPC_PORT = 50061
_DEFAULT_GRPC_HOST = "127.0.0.1"
_DEFAULT_PENDING_QUEUE_MAX_SIZE = 4
_DEFAULT_TIMEOUT_SEC = 60
_STATE_TERMINAL = frozenset((3, 4, 5))  # SUCCEEDED / FAILED / CANCELED
# enum 数字值与 design.md 附录 K proto NavigationStatus 一一对应
_STATE_PENDING = 1
_STATE_ACTIVE = 2
_STATE_SUCCEEDED = 3
_STATE_FAILED = 4
_STATE_CANCELED = 5
_FAILURE_QUEUE_FULL = 5
_FAILURE_PROXY_UNAVAILABLE = 6


class NavigationFailureCode(str, Enum):
    """导航失败原因枚举；与 .proto NavigationFailureCode 一一对应。"""

    UNKNOWN = "UNKNOWN"
    TIMEOUT = "TIMEOUT"
    OBSTACLE = "OBSTACLE"
    NO_PATH = "NO_PATH"
    CANCELED = "CANCELED"
    QUEUE_FULL = "QUEUE_FULL"
    PROXY_UNAVAILABLE = "PROXY_UNAVAILABLE"


@dataclass
class NavigationError:
    """平台侧统一的导航错误结构（与 mcp-end-to-end spec 一致）。"""

    code: NavigationFailureCode
    message: str
    recoverable: bool = True

    def to_dict(self) -> Dict[str, object]:
        return {"code": self.code.value, "message": self.message, "recoverable": self.recoverable}


@dataclass
class NavigateToPoseRequest:
    """本地 dataclass 形式的 NavigateToPoseRequest（与 .proto 字段一一对应）。"""

    request_id: str
    pose: Pose
    timeout_sec: int = _DEFAULT_TIMEOUT_SEC

    @classmethod
    def create(cls, pose: Pose, *, timeout_sec: int = _DEFAULT_TIMEOUT_SEC, request_id: Optional[str] = None) -> "NavigateToPoseRequest":
        return cls(request_id=request_id or str(uuid.uuid4()), pose=pose, timeout_sec=timeout_sec)


@dataclass
class NavigationStateUpdate:
    """本地 dataclass 形式的 NavigationStateUpdate。"""

    request_id: str
    status: int
    current_pose: Optional[Pose] = None
    failure: Optional[NavigationError] = None
    timestamp_ms: int = 0

    @property
    def is_terminal(self) -> bool:
        return self.status in _STATE_TERMINAL

    @property
    def status_label(self) -> str:
        mapping = {
            1: "PENDING",
            2: "ACTIVE",
            3: "SUCCEEDED",
            4: "FAILED",
            5: "CANCELED",
        }
        return mapping.get(self.status, "UNKNOWN")

    def to_dict(self) -> Dict[str, object]:
        payload: Dict[str, object] = {
            "request_id": self.request_id,
            "status": self.status_label,
            "timestamp_ms": self.timestamp_ms,
        }
        if self.current_pose is not None:
            payload["current_pose"] = self.current_pose.model_dump(mode="json")
        if self.failure is not None:
            payload["failure"] = self.failure.to_dict()
        return payload


class _ProxyUnavailable(Exception):
    """gRPC 通道建立失败或 server 不可达 —— 内部异常，对外转换为 PROXY_UNAVAILABLE 错误码。"""


class NavigationProxyClient:
    """platform 侧 gRPC client + 串行锁 + PENDING 队列 + 状态机。

    用法（services/geometry/runtime._dispatch_navigate 调用）::

        client = NavigationProxyClient()
        request = NavigateToPoseRequest.create(pose)
        state = await client.navigate(request)  # 阻塞到终态
    """

    def __init__(
        self,
        *,
        grpc_host: Optional[str] = None,
        grpc_port: Optional[int] = None,
        pending_queue_max_size: Optional[int] = None,
        stub_factory: Optional[Callable[[str], Any]] = None,
    ) -> None:
        self._grpc_host = grpc_host or os.environ.get("NAV_PROXY_GRPC_HOST", _DEFAULT_GRPC_HOST)
        self._grpc_port = grpc_port if grpc_port is not None else int(
            os.environ.get("NAV_PROXY_GRPC_PORT", str(_DEFAULT_GRPC_PORT))
        )
        self._pending_queue_max_size = pending_queue_max_size if pending_queue_max_size is not None else int(
            os.environ.get("NAV_PROXY_PENDING_QUEUE_MAX_SIZE", str(_DEFAULT_PENDING_QUEUE_MAX_SIZE))
        )
        self._stub_factory = stub_factory  # 测试注入点（mock stub）
        self._lock = asyncio.Lock()
        self._pending: "asyncio.Queue[Tuple[NavigateToPoseRequest, asyncio.Future[NavigationStateUpdate], asyncio.Event]]" = asyncio.Queue()
        self._active: Optional[NavigationStateUpdate] = None
        self._closed = False
        self._stub: Any = None
        self._channel: Any = None

    # ------------------------------------------------------------------
    # 通道管理（懒加载；失败转 _ProxyUnavailable）
    # ------------------------------------------------------------------

    def _ensure_channel(self) -> Any:
        if self._stub is not None:
            return self._stub
        if self._stub_factory is not None:
            self._stub = self._stub_factory("%s:%d" % (self._grpc_host, self._grpc_port))
            return self._stub
        try:
            import grpc  # type: ignore
        except Exception as exc:  # pragma: no cover - 静态验证环境无 grpc
            raise _ProxyUnavailable("grpcio 未安装：%s" % exc)
        target = "%s:%d" % (self._grpc_host, self._grpc_port)
        try:
            self._channel = grpc.insecure_channel(target)
            self._stub = _build_stub_from_channel(self._channel)
        except Exception as exc:
            raise _ProxyUnavailable("gRPC 通道建立失败：%s" % exc)
        return self._stub

    async def close(self) -> None:
        self._closed = True
        if self._channel is not None:
            try:
                close_fn = getattr(self._channel, "close", None)
                if callable(close_fn):
                    result = close_fn()
                    if asyncio.iscoroutine(result):
                        await result
            except Exception:
                LOGGER.debug("关闭 gRPC 通道失败", exc_info=True)
            self._channel = None
        self._stub = None

    # ------------------------------------------------------------------
    # 公开 API：navigate / cancel / get_state
    # ------------------------------------------------------------------

    async def navigate(self, request: NavigateToPoseRequest) -> NavigationStateUpdate:
        """提交一个导航请求；阻塞到 SUCCEEDED / FAILED / CANCELED 任一终态。"""
        if self._closed:
            raise RuntimeError("NavigationProxyClient 已关闭。")
        loop = asyncio.get_running_loop()
        future: "asyncio.Future[NavigationStateUpdate]" = loop.create_future()
        event = asyncio.Event()
        # PENDING 队列上限保护（不入锁，避免与串行锁冲突；queue.put 不阻塞因为 unbounded）
        if self._pending.qsize() >= self._pending_queue_max_size:
            raise _queue_full_error()
        await self._pending.put((request, future, event))
        # 启动串行 worker（仅首次启动）
        if not hasattr(self, "_worker_task") or self._worker_task is None or self._worker_task.done():
            self._worker_task = asyncio.create_task(self._serial_worker(), name="nav_proxy_serial_worker")
        # 阻塞等待终态
        try:
            update = await future
        except asyncio.CancelledError:  # pragma: no cover
            raise
        return update

    async def cancel(self, request_id: str) -> Tuple[bool, str]:
        """取消指定 request_id 的导航。"""
        if not request_id:
            return False, "request_id 为空"
        try:
            stub = self._ensure_channel()
            request_pb = _build_cancel_request_pb(request_id)
            response = await _invoke_cancel_async(stub, request_pb)
            return bool(getattr(response, "canceled", False)), str(getattr(response, "message", ""))
        except _ProxyUnavailable as exc:
            return False, str(exc)
        except Exception as exc:
            LOGGER.warning("cancel_navigation 异常 request_id=%s error=%s", request_id, exc)
            return False, str(exc)

    async def get_state(self) -> Dict[str, object]:
        """查询当前 gRPC 通道可达性 + 队列状态（轻量级）。"""
        state: Dict[str, object] = {
            "proxy_unavailable": False,
            "pending_queue_size": self._pending.qsize(),
            "pending_queue_max_size": self._pending_queue_max_size,
            "grpc_host": self._grpc_host,
            "grpc_port": self._grpc_port,
        }
        try:
            stub = self._ensure_channel()
            # 通道健康（grpcio 健康检查在静态环境不易复现；退化为主动小 RPC）
            state["proxy_unavailable"] = False
            state["stub_ready"] = stub is not None
        except _ProxyUnavailable as exc:
            state["proxy_unavailable"] = True
            state["error"] = str(exc)
        return state

    # ------------------------------------------------------------------
    # 串行 worker（PENDING FIFO → ACTIVE → 终态 → 接力下一）
    # ------------------------------------------------------------------

    async def _serial_worker(self) -> None:
        while not self._closed:
            try:
                request, future, event = await asyncio.wait_for(self._pending.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            except Exception as exc:  # pragma: no cover - 防御性
                LOGGER.debug("serial_worker 拉取异常：%s", exc)
                continue
            if future.done():
                continue
            async with self._lock:
                try:
                    update = await self._submit_active(request)
                except _ProxyUnavailable as exc:
                    update = NavigationStateUpdate(
                        request_id=request.request_id,
                        status=_STATE_FAILED,
                        failure=NavigationError(
                            code=NavigationFailureCode.PROXY_UNAVAILABLE,
                            message="导航代理离线，请联系运维",
                            recoverable=False,
                        ),
                        timestamp_ms=int(time.time() * 1000),
                    )
                except Exception as exc:
                    LOGGER.exception("submit_active 异常 request_id=%s", request.request_id)
                    update = NavigationStateUpdate(
                        request_id=request.request_id,
                        status=_STATE_FAILED,
                        failure=NavigationError(
                            code=NavigationFailureCode.UNKNOWN,
                            message="导航代理内部异常：%s" % exc,
                            recoverable=True,
                        ),
                        timestamp_ms=int(time.time() * 1000),
                    )
                finally:
                    if not future.done():
                        future.set_result(update)

    async def _submit_active(self, request: NavigateToPoseRequest) -> NavigationStateUpdate:
        """调 gRPC 双向流，阻塞到终态。"""
        stub = self._ensure_channel()
        request_iter = _build_request_iterator(request)
        update = NavigationStateUpdate(request_id=request.request_id, status=_STATE_ACTIVE, timestamp_ms=int(time.time() * 1000))
        try:
            async for raw in _invoke_navigate_async(stub, request_iter):
                status = int(getattr(raw, "status", 0))
                update = NavigationStateUpdate(
                    request_id=str(getattr(raw, "request_id", request.request_id)),
                    status=status,
                    current_pose=_extract_pose(getattr(raw, "current_pose", None)),
                    failure=_extract_failure(getattr(raw, "failure", None)),
                    timestamp_ms=int(getattr(raw, "timestamp_ms", 0)),
                )
                if update.is_terminal:
                    return update
        except _ProxyUnavailable:
            raise
        except Exception as exc:
            return NavigationStateUpdate(
                request_id=request.request_id,
                status=_STATE_FAILED,
                failure=NavigationError(
                    code=NavigationFailureCode.UNKNOWN,
                    message=str(exc),
                    recoverable=True,
                ),
                timestamp_ms=int(time.time() * 1000),
            )
        # 流未产出终态 —— 视为 timeout 兜底
        return NavigationStateUpdate(
            request_id=request.request_id,
            status=_STATE_FAILED,
            failure=NavigationError(
                code=NavigationFailureCode.TIMEOUT,
                message="导航代理未返回终态",
                recoverable=True,
            ),
            timestamp_ms=int(time.time() * 1000),
        )


# ----------------------------------------------------------------------
# proto 适配层（grpcio / 异步语义封装；proto 未生成时降级为本地实现）
# ----------------------------------------------------------------------


def _build_stub_from_channel(channel: Any) -> Any:
    """构造 NavigationProxyService stub；proto 未生成时返回本地 stub 标记。"""
    try:
        from protos import nav2_action_proxy_pb2_grpc as pb2_grpc  # type: ignore

        return pb2_grpc.NavigationProxyServiceStub(channel)
    except Exception:
        # proto 未编译 —— 返回 None；调用方须通过 stub_factory 注入 mock
        return None


def _build_request_iterator(request: NavigateToPoseRequest) -> Any:
    """构造 gRPC NavigateToPoseRequest 迭代器（异步可迭代）。"""

    async def _aiter():
        try:
            from protos import nav2_action_proxy_pb2 as pb2  # type: ignore

            req = pb2.NavigateToPoseRequest()
            req.request_id = request.request_id
            req.pose.x = float(request.pose.position.x)
            req.pose.y = float(request.pose.position.y)
            req.pose.yaw = float(request.pose.orientation.w if request.pose.orientation.w else 0.0)
            req.timeout_sec = int(request.timeout_sec)
            yield req
        except Exception:
            # proto 未生成 —— 产出本地 dataclass 形式（仅 stub_factory 注入的 mock stub 可用）
            yield request

    return _aiter()


def _build_cancel_request_pb(request_id: str) -> Any:
    try:
        from protos import nav2_action_proxy_pb2 as pb2  # type: ignore

        req = pb2.CancelRequest()
        req.request_id = request_id
        return req
    except Exception:
        return request_id


async def _invoke_navigate_async(stub: Any, request_iterator: Any) -> Any:
    """调用 stub.NavigateToPose；兼容 sync / async 返回。"""
    if stub is None:
        raise _ProxyUnavailable("stub 为空（proto 未编译且未注入 stub_factory）")
    call = getattr(stub, "NavigateToPose", None)
    if call is None:
        raise _ProxyUnavailable("stub 缺 NavigateToPose 方法")
    response = call(request_iterator)
    # sync stub -> 同步迭代；async stub -> 异步迭代（grpcio 1.70 提供 async def）
    if hasattr(response, "__aiter__"):
        async for item in response:
            yield item
    else:
        for item in response:
            yield item


async def _invoke_cancel_async(stub: Any, request_pb: Any) -> Any:
    if stub is None:
        raise _ProxyUnavailable("stub 为空（proto 未编译且未注入 stub_factory）")
    call = getattr(stub, "CancelNavigateToPose", None)
    if call is None:
        raise _ProxyUnavailable("stub 缺 CancelNavigateToPose 方法")
    response = call(request_pb)
    if asyncio.iscoroutine(response):
        response = await response
    return response


def _extract_pose(pb_pose: Any) -> Optional[Pose]:
    if pb_pose is None:
        return None
    try:
        from contracts.geometry import Pose as PoseContract

        return PoseContract(
            position={"x": float(getattr(pb_pose, "x", 0.0)), "y": float(getattr(pb_pose, "y", 0.0)), "z": 0.0},
            orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            frame_id="map",
        )
    except Exception:
        return None


def _extract_failure(pb_failure: Any) -> Optional[NavigationError]:
    if pb_failure is None:
        return None
    try:
        code_value = int(getattr(pb_failure, "code", 0))
        mapping = {
            0: NavigationFailureCode.UNKNOWN,
            1: NavigationFailureCode.TIMEOUT,
            2: NavigationFailureCode.OBSTACLE,
            3: NavigationFailureCode.NO_PATH,
            4: NavigationFailureCode.CANCELED,
            5: NavigationFailureCode.QUEUE_FULL,
            6: NavigationFailureCode.PROXY_UNAVAILABLE,
        }
        code = mapping.get(code_value, NavigationFailureCode.UNKNOWN)
        return NavigationError(
            code=code,
            message=str(getattr(pb_failure, "message", "")),
            recoverable=bool(getattr(pb_failure, "recoverable", True)),
        )
    except Exception:
        return None


def _queue_full_error() -> RuntimeError:
    return RuntimeError(
        "%s" % NavigationError(
            code=NavigationFailureCode.QUEUE_FULL,
            message="PENDING 队列已满",
            recoverable=True,
        ).to_dict()
    )


# 单例（worker-C 复用；不强制）
_DEFAULT_CLIENT: Optional[NavigationProxyClient] = None


def get_default_client() -> NavigationProxyClient:
    global _DEFAULT_CLIENT
    if _DEFAULT_CLIENT is None:
        _DEFAULT_CLIENT = NavigationProxyClient()
    return _DEFAULT_CLIENT