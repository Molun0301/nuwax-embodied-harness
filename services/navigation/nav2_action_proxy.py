"""nav2_action_proxy：独立 ROS 进程 + gRPC server + Nav2 action client。

设计要点（openspec/changes/navigation-and-exploration-boundary/design.md 决策 D1）：

* 进程拓扑：独立 Python 进程，由 ``services/navigation/nav2_action_proxy_launch.sh``
  拉起；cyclonedds RMW（``export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``）。
* 跨 RMW 隔离：平台 services 层（rclpy=fastrtps）禁止直订 Nav2 action server；
  本进程通过 gRPC 与 ``services/geometry/navigation_proxy_client.py`` 通信。
* 进程内状态机：每个 ``request_id`` 走 PENDING -> ACTIVE -> 终态 单向流转；
  并发请求 FIFO 入队，串行 lock 保护 Nav2 action server 单目标调度。
* gRPC 接口：实现 ``NavigationProxyService`` 的 2 个 method
  （``NavigateToPose`` 双向流 + ``CancelNavigateToPose`` 单向）。

环境变量：

* ``NAV_PROXY_GRPC_PORT``   —— gRPC 监听端口，默认 ``core/settings.py:NAV_PROXY_GRPC_PORT``。
* ``NAV_PROXY_RMW``         —— RMW 实现名，默认 ``rmw_cyclonedds_cpp``。
* ``NAV_PROXY_PENDING_QUEUE_MAX_SIZE`` —— PENDING 队列上限，默认 4。
* ``NAV_ACTION_NAMESPACE``  —— Nav2 action namespace，默认 ``/navigate_to_pose``。

限制：

* services 层禁 rclpy —— 本文件位于 services/navigation/，但本进程是独立 ROS 进程，
  *不* 与平台 services 层 rclpy 共享 context（独立 RMW 实现）。
* CI 守门仍允许本文件 ``import rclpy`` / ``rclpy.action``（该规则仅约束
  ``services/geometry/``，本进程属独立进程的边界例外）。
"""
from __future__ import annotations

import argparse
import logging
import os
import queue
import signal
import sys
import threading
import time
import uuid
from concurrent import futures
from typing import Any, Dict, List, Optional

# proto 生成产物 —— 由 protos/navigation_proxy.proto 编译得到（worker-A tasks 1.1-1.3）。
# 若 worker-A 尚未生成，本进程以 ``_PROTO_AVAILABLE = False`` 退化为 stub server，
# 仍能启动 gRPC server + 接受 Cancel，避免阻塞其他 worker。
try:
    from protos import nav2_action_proxy_pb2 as pb2  # type: ignore
    from protos import nav2_action_proxy_pb2_grpc as pb2_grpc  # type: ignore
    _PROTO_AVAILABLE = True
except Exception as _PROTO_IMPORT_ERROR:  # pragma: no cover - 编译未完成时降级
    pb2 = None  # type: ignore
    pb2_grpc = None  # type: ignore
    _PROTO_AVAILABLE = False

LOGGER = logging.getLogger("nuwax_robot_bridge.nav2_action_proxy")

_DEFAULT_GRPC_PORT = 50061
_DEFAULT_RMW = "rmw_cyclonedds_cpp"
_DEFAULT_PENDING_QUEUE_MAX_SIZE = 4
_DEFAULT_ACTION_NAMESPACE = "/navigate_to_pose"
_STATUS_PENDING = 1
_STATUS_ACTIVE = 2
_STATUS_SUCCEEDED = 3
_STATUS_FAILED = 4
_STATUS_CANCELED = 5
_FAILURE_QUEUE_FULL = 5
_FAILURE_PROXY_UNAVAILABLE = 6


def _read_env_int(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        LOGGER.warning("环境变量 %s=%s 无法解析为 int，使用默认值 %s", name, raw, default)
        return default


def _read_env_str(name: str, default: str) -> str:
    raw = os.environ.get(name, "").strip()
    return raw or default


class _NavGoalState:
    """单 request_id 的内部状态机（不含 rclpy 类型，便于单元测试）。"""

    __slots__ = ("request_id", "status", "failure_code", "failure_message", "pose", "timeout_sec", "active_handle", "stream_queue", "created_at")

    def __init__(self, request_id: str, pose: Dict[str, float], timeout_sec: int) -> None:
        self.request_id = request_id
        self.pose = pose
        self.timeout_sec = timeout_sec
        self.status: int = _STATUS_PENDING
        self.failure_code: int = 0
        self.failure_message: str = ""
        self.active_handle: Any = None
        self.stream_queue: "queue.Queue[Any]" = queue.Queue(maxsize=64)
        self.created_at: float = time.monotonic()


class Nav2ActionProxyServicer:
    """gRPC servicer 实现（无 ROS 依赖，便于本地 mock 测试）。"""

    def __init__(self, pending_queue_max_size: int = _DEFAULT_PENDING_QUEUE_MAX_SIZE) -> None:
        self._lock = threading.Lock()
        self._pending_queue: List[str] = []
        self._states: Dict[str, _NavGoalState] = {}
        self._active_id: Optional[str] = None
        self._pending_queue_max_size = pending_queue_max_size
        self._shutdown = threading.Event()
        # 实际 ROS action client 注入点（独立进程拉起 rclpy 节点后回填）
        self._ros_action_client: Any = None
        self._ros_node: Any = None

    # ------------------------------------------------------------------
    # 状态机对外接口
    # ------------------------------------------------------------------

    def set_ros_clients(self, *, action_client: Any, node: Any) -> None:
        """外部 ROS 上下文拉起后回填；无 ROS 时 action_client 保持 None。"""
        self._ros_action_client = action_client
        self._ros_node = node

    def is_proxy_unavailable(self) -> bool:
        return self._shutdown.is_set() or (self._ros_action_client is None and not _PROTO_AVAILABLE)

    def shutdown(self) -> None:
        self._shutdown.set()

    # ------------------------------------------------------------------
    # 内部辅助
    # ------------------------------------------------------------------

    def _state_to_update(self, state: _NavGoalState) -> Any:
        if not _PROTO_AVAILABLE:
            return None
        update = pb2.NavigationStateUpdate()
        update.request_id = state.request_id
        update.status = state.status
        update.current_pose.x = float(state.pose.get("x", 0.0))
        update.current_pose.y = float(state.pose.get("y", 0.0))
        update.current_pose.yaw = float(state.pose.get("yaw", 0.0))
        if state.status in (_STATUS_FAILED, _STATUS_CANCELED) and state.failure_code:
            update.failure.code = state.failure_code
            update.failure.message = state.failure_message
            update.failure.recoverable = state.failure_code != _FAILURE_QUEUE_FULL
        update.timestamp_ms = int(time.time() * 1000)
        return update

    def _enqueue_pending(self, state: _NavGoalState) -> bool:
        """加入 PENDING 队列；返回 True 成功 / False 队列已满。"""
        with self._lock:
            if self._active_id is None and not self._pending_queue:
                # 空闲 -> 直接 ACTIVE
                self._active_id = state.request_id
                state.status = _STATUS_ACTIVE
                return True
            if len(self._pending_queue) >= self._pending_queue_max_size:
                return False
            self._pending_queue.append(state.request_id)
            return True

    def _advance_after_terminal(self) -> Optional[str]:
        """当前 ACTIVE 进入终态后，从 PENDING FIFO 接力下一个；返回新 ACTIVE id。"""
        with self._lock:
            if self._active_id is not None:
                return None
            if not self._pending_queue:
                return None
            next_id = self._pending_queue.pop(0)
            self._active_id = next_id
            state = self._states.get(next_id)
            if state is not None:
                state.status = _STATUS_ACTIVE
            return next_id

    # ------------------------------------------------------------------
    # gRPC method：NavigateToPose 双向流
    # ------------------------------------------------------------------

    def NavigateToPose(self, request_iterator, context):  # noqa: N802 — gRPC method 命名
        if not _PROTO_AVAILABLE:
            context.set_code(13)  # INTERNAL
            context.set_details("navigation_proxy.proto 编译产物未生成，本进程降级为 stub。")
            return
        # 接收首个 NavigateToPoseRequest 入队；后续若收到 CancelRequest 则取消。
        first_request = None
        try:
            for incoming in request_iterator:
                incoming_type = incoming.DESCRIPTOR.name if hasattr(incoming, "DESCRIPTOR") else ""
                if incoming_type == "NavigateToPoseRequest":
                    first_request = incoming
                    request_id = str(incoming.request_id) or str(uuid.uuid4())
                    pose = {
                        "x": float(incoming.pose.x),
                        "y": float(incoming.pose.y),
                        "yaw": float(incoming.pose.yaw),
                    }
                    timeout_sec = int(getattr(incoming, "timeout_sec", 60) or 60)
                    state = _NavGoalState(request_id=request_id, pose=pose, timeout_sec=timeout_sec)
                    with self._lock:
                        self._states[request_id] = state
                    enqueued = self._enqueue_pending(state)
                    if not enqueued:
                        state.status = _STATUS_FAILED
                        state.failure_code = _FAILURE_QUEUE_FULL
                        state.failure_message = "PENDING 队列已满"
                        yield self._state_to_update(state)
                        # 立即接力下一 PENDING（若队列后续被消费）
                        continue
                    # 推送 PENDING/ACTIVE 首条状态
                    yield self._state_to_update(state)
                elif incoming_type == "CancelRequest":
                    self._do_cancel(str(getattr(incoming, "request_id", "")))
        except Exception as exc:
            LOGGER.exception("NavigateToPose 双向流异常：%s", exc)
            return

    def _do_cancel(self, request_id: str) -> None:
        if not request_id:
            return
        with self._lock:
            state = self._states.get(request_id)
            if state is None:
                return
            if state.status in (_STATUS_SUCCEEDED, _STATUS_FAILED, _STATUS_CANCELED):
                return
            if request_id in self._pending_queue:
                self._pending_queue.remove(request_id)
                state.status = _STATUS_CANCELED
                state.failure_code = 4  # CANCELED
                state.failure_message = "PENDING 阶段被取消"
                state.stream_queue.put(self._state_to_update(state))
                return
            if request_id == self._active_id:
                # 调用 Nav2 action cancel（无 ROS 时仅更新状态机）
                handle = state.active_handle
                if handle is not None:
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        LOGGER.debug("Nav2 cancel_goal_async 失败", exc_info=True)
                state.status = _STATUS_CANCELED
                state.failure_code = 4
                state.failure_message = "ACTIVE 阶段被取消"
                state.stream_queue.put(self._state_to_update(state))
                self._active_id = None
                self._advance_after_terminal()

    # ------------------------------------------------------------------
    # gRPC method：CancelNavigateToPose 单向
    # ------------------------------------------------------------------

    def CancelNavigateToPose(self, request, context):  # noqa: N802
        response = pb2.CancelResponse()
        if not _PROTO_AVAILABLE:
            response.canceled = False
            response.message = "proto 未生成，stub server 不处理取消"
            return response
        request_id = str(getattr(request, "request_id", ""))
        with self._lock:
            state = self._states.get(request_id)
            if state is None:
                response.canceled = False
                response.message = "目标已完成或不存在"
                return response
            if state.status in (_STATUS_SUCCEEDED, _STATUS_FAILED, _STATUS_CANCELED):
                response.canceled = False
                response.message = "目标已完成或不存在"
                return response
        self._do_cancel(request_id)
        response.canceled = True
        response.message = "ok"
        return response


def _build_action_client(node: Any, action_namespace: str) -> Any:
    """构造 Nav2 NavigateToPose action client；失败抛 RuntimeError。"""
    try:
        from rclpy.action import ActionClient  # type: ignore
        from nav2_msgs.action import NavigateToPose  # type: ignore
    except Exception as exc:  # pragma: no cover - 实机 ROS 环境兜底
        raise RuntimeError("rclpy / nav2_msgs 未安装：%s" % exc)
    return ActionClient(node, NavigateToPose, action_namespace)


def _start_ros_node(action_namespace: str) -> Any:
    """初始化 rclpy 节点 + NavigateToPose action client；无 ROS 时抛 RuntimeError。"""
    try:
        import rclpy  # type: ignore
    except Exception as exc:
        raise RuntimeError("rclpy 未安装：%s" % exc)
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("nuwax_nav2_action_proxy")
    action_client = _build_action_client(node, action_namespace)
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.destroy_node()
        raise RuntimeError("Nav2 action server %s 不可用" % action_namespace)
    return node, action_client


def _serve_forever(grpc_port: int, pending_queue_max_size: int, action_namespace: str) -> int:
    """启动 gRPC server；返回退出码（0 正常 / 非 0 异常）。"""
    if not _PROTO_AVAILABLE:
        LOGGER.warning(
            "protos/navigation_proxy_pb2*.py 不可用（import error=%s），"
            "本进程以 stub 模式启动，无法处理真实 NavigateToPose 请求。",
            _PROTO_IMPORT_ERROR,
        )
    server = None
    servicer = Nav2ActionProxyServicer(pending_queue_max_size=pending_queue_max_size)
    if _PROTO_AVAILABLE and pb2_grpc is not None:
        server = grpc_server_module().server(futures.ThreadPoolExecutor(max_workers=8))
        pb2_grpc.add_NavigationProxyServiceServicer_to_server(servicer, server)
        server.add_insecure_port("127.0.0.1:%d" % grpc_port)
        server.start()
        LOGGER.info("Nav2 action proxy gRPC server 已监听 127.0.0.1:%d", grpc_port)

    ros_node = None
    ros_action_client = None
    if _PROTO_AVAILABLE:
        try:
            ros_node, ros_action_client = _start_ros_node(action_namespace)
            servicer.set_ros_action_client(action_client=ros_action_client, node=ros_node)
            LOGGER.info("Nav2 action client 已连接 action namespace=%s", action_namespace)
        except Exception as exc:
            LOGGER.warning("Nav2 action client 未启动（继续以 stub 模式运行）：%s", exc)

    stop_event = threading.Event()

    def _on_signal(signum, frame):  # noqa: ARG001
        LOGGER.info("收到信号 signum=%s，准备退出", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        if server is not None:
            server.stop(grace=2.0)
        if ros_node is not None:
            try:
                ros_node.destroy_node()
            except Exception:
                LOGGER.debug("销毁 rclpy 节点失败", exc_info=True)
        try:
            import rclpy  # type: ignore
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        LOGGER.info("Nav2 action proxy 已退出")
    return 0


def grpc_server_module():  # pragma: no cover - 仅在 proto 可用时被调用
    import grpc  # type: ignore

    return grpc


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nav2 action proxy（独立 ROS 进程 + gRPC）")
    parser.add_argument("--grpc-port", type=int, default=_read_env_int("NAV_PROXY_GRPC_PORT", _DEFAULT_GRPC_PORT))
    parser.add_argument("--pending-queue-max-size", type=int, default=_read_env_int("NAV_PROXY_PENDING_QUEUE_MAX_SIZE", _DEFAULT_PENDING_QUEUE_MAX_SIZE))
    parser.add_argument("--action-namespace", type=str, default=_read_env_str("NAV_ACTION_NAMESPACE", _DEFAULT_ACTION_NAMESPACE))
    parser.add_argument("--rmw", type=str, default=_read_env_str("NAV_PROXY_RMW", _DEFAULT_RMW))
    parser.add_argument("--log-level", type=str, default=_read_env_str("NAV_PROXY_LOG_LEVEL", "INFO"))
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    # RMW 强制覆盖（即使 launcher 已 export，这里兜底以防直接 python3 调用）
    os.environ["RMW_IMPLEMENTATION"] = args.rmw
    LOGGER.info(
        "nav2_action_proxy 启动 grpc_port=%s pending_queue_max_size=%s action_namespace=%s rmw=%s proto_available=%s",
        args.grpc_port,
        args.pending_queue_max_size,
        args.action_namespace,
        args.rmw,
        _PROTO_AVAILABLE,
    )
    return _serve_forever(
        grpc_port=args.grpc_port,
        pending_queue_max_size=args.pending_queue_max_size,
        action_namespace=args.action_namespace,
    )


if __name__ == "__main__":
    sys.exit(main())