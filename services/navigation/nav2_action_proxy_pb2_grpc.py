# -*- coding: utf-8 -*-
# 手写 stub：protos/navigation_proxy.proto 对应 gRPC service 描述
#
# 设计背景：
# - grpcio-tools 不可用环境（精简 CI 容器、部分开发机）的 fallback stub。
# - 提供 servicer / stubr / stub 类骨架与 method 名常量，保证 import 成功。
# - 实际 RPC 调用需要真实的 grpcio 库（实机已有 grpcio 1.70.0）。
"""nav2_action_proxy_pb2_grpc module."""

from __future__ import annotations

try:
    import grpc
except ImportError:
    # 容错：精简 CI / 离线环境下 grpc 不可用时仍允许 import 不崩。
    # 实际 RPC 路径会在 server / client 启动时报 AttributeError。
    grpc = None  # type: ignore

from services.navigation import nav2_action_proxy_pb2 as nav2_action_proxy__pb2


class NavigationProxyServiceServicer(object):
    """独立 ROS 进程（nav2_action_proxy.py）实现的 gRPC servicer 基类。

    平台侧（services/geometry/navigation_proxy_client.py）调用以下两个 method。
    """

    def NavigateToPose(self, request_iterator, context):
        """双向流：client → server 持续提交 NavigateToPoseRequest；server → client 推送 NavigationStateUpdate。"""
        raise NotImplementedError("NavigateToPose must be implemented")

    def CancelNavigateToPose(self, request, context):
        """单向：取消指定 request_id 的导航。"""
        raise NotImplementedError("CancelNavigateToPose must be implemented")


def add_NavigationProxyServiceServicer_to_server(servicer, server):
    """把 servicer 注册到 grpc server。"""

    if grpc is None:
        raise RuntimeError("grpcio 不可用，无法注册 NavigationProxyService servicer")

    rpc_method_handlers = {
        "NavigateToPose": grpc.stream_stream_rpc_method_handler(
            servicer.NavigateToPose,
            request_deserializer=nav2_action_proxy__pb2.NavigateToPoseRequest.FromString,
            response_serializer=nav2_action_proxy__pb2.NavigationStateUpdate.SerializeToString,
        ),
        "CancelNavigateToPose": grpc.unary_unary_rpc_method_handler(
            servicer.CancelNavigateToPose,
            request_deserializer=nav2_action_proxy__pb2.CancelRequest.FromString,
            response_serializer=nav2_action_proxy__pb2.CancelResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_service_handler(
        "nuwax.navigation_proxy.NavigationProxyService", rpc_method_handlers
    )
    server.add_generic_rpc_handlers((generic_handler,))


class NavigationProxyServiceStub(object):
    """platform 侧 gRPC client stub（services/geometry/navigation_proxy_client.py 使用）。"""

    def __init__(self, channel):
        if grpc is None:
            raise RuntimeError("grpcio 不可用，无法创建 NavigationProxyService stub")

        self.NavigateToPose = channel.stream_stream(
            "/nuwax.navigation_proxy.NavigationProxyService/NavigateToPose",
            request_serializer=nav2_action_proxy__pb2.NavigateToPoseRequest.SerializeToString,
            response_deserializer=nav2_action_proxy__pb2.NavigationStateUpdate.FromString,
        )
        self.CancelNavigateToPose = channel.unary_unary(
            "/nuwax.navigation_proxy.NavigationProxyService/CancelNavigateToPose",
            request_serializer=nav2_action_proxy__pb2.CancelRequest.SerializeToString,
            response_deserializer=nav2_action_proxy__pb2.CancelResponse.FromString,
        )


__all__ = [
    "NavigationProxyServiceServicer",
    "NavigationProxyServiceStub",
    "add_NavigationProxyServiceServicer_to_server",
]  # noqa: WPS410