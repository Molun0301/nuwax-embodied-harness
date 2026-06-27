# -*- coding: utf-8 -*-
# 手写 stub：protos/navigation_proxy.proto 的 Python 等价物
#
# 设计背景：
# - grpcio-tools 在某些精简环境（如部分 CI 容器）不可用，代码不能因 import 而崩溃。
# - 本文件作为回退 stub：导出同名 message 类与枚举常量，对外提供 dataclass-like 字段
#   读写；不直接构造 protobuf wire format（依赖 grpcio/protobuf 在 server / client 侧
#   的真实实现）。
#
# 字段名、字段类型、嵌套 message 结构与 protos/navigation_proxy.proto 完全对应；
# 这与 design.md 附录 K 的契约一致。
"""nav2_action_proxy_pb2 module."""

from __future__ import annotations

from typing import Optional

# ============================================================
# 枚举（用 int 常量暴露；按 design.md 附录 K 顺序）
# ============================================================

# NavigationStatus
NAVIGATION_STATUS_UNKNOWN = 0
NAVIGATION_STATUS_PENDING = 1
NAVIGATION_STATUS_ACTIVE = 2
NAVIGATION_STATUS_SUCCEEDED = 3
NAVIGATION_STATUS_FAILED = 4
NAVIGATION_STATUS_CANCELED = 5

# NavigationFailureCode
NAVIGATION_FAILURE_UNKNOWN = 0
NAVIGATION_FAILURE_TIMEOUT = 1
NAVIGATION_FAILURE_OBSTACLE = 2
NAVIGATION_FAILURE_NO_PATH = 3
NAVIGATION_FAILURE_CANCELED = 4
NAVIGATION_FAILURE_QUEUE_FULL = 5
NAVIGATION_FAILURE_PROXY_UNAVAILABLE = 6


# ============================================================
# Message：纯 Python 类，字段访问 / repr 与 protobuf 一致
# ============================================================


class _ProtoBase:
    """极简 message 基类：提供 .SerializeToString / FromString 接口占位。

    真实序列化由 grpcio server / client 端的 protoc-gen 生成的代码负责；
    本回退 stub 只保证字段读写与字段名稳定。
    """

    def __init__(self, **kwargs):
        # 收集所有声明字段，未传值时使用 None / 0 / "" 兜底
        for field in self._FIELDS:
            setattr(self, field, kwargs.get(field, self._DEFAULTS.get(field)))

    def __repr__(self):
        parts = [
            "{}={!r}".format(f, getattr(self, f))
            for f in self._FIELDS
            if getattr(self, f) not in (None, 0, "", False)
        ]
        return "{}({})".format(self.__class__.__name__, ", ".join(parts))

    def SerializeToString(self):  # noqa: N802 — protobuf 风格
        return b""

    def ParseFromString(self, _data):  # noqa: N802
        return 0

    def CopyFrom(self, other):  # noqa: N802
        for f in self._FIELDS:
            setattr(self, f, getattr(other, f, None))
        return self


class Pose2D(_ProtoBase):
    """2D 位姿。"""

    _FIELDS = ("x", "y", "yaw")
    _DEFAULTS = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        super(Pose2D, self).__init__(x=float(x), y=float(y), yaw=float(yaw))


class NavigationFailure(_ProtoBase):
    """失败信息。"""

    _FIELDS = ("code", "message", "recoverable")
    _DEFAULTS = {"code": NAVIGATION_FAILURE_UNKNOWN, "message": "", "recoverable": False}

    def __init__(
        self,
        code: int = NAVIGATION_FAILURE_UNKNOWN,
        message: str = "",
        recoverable: bool = False,
    ):
        super(NavigationFailure, self).__init__(
            code=int(code),
            message=str(message),
            recoverable=bool(recoverable),
        )


class NavigateToPoseRequest(_ProtoBase):
    """请求：提交一个导航目标。"""

    _FIELDS = ("request_id", "pose", "timeout_sec")
    _DEFAULTS = {"request_id": "", "pose": None, "timeout_sec": 60}

    def __init__(
        self,
        request_id: str = "",
        pose: Optional[Pose2D] = None,
        timeout_sec: int = 60,
    ):
        super(NavigateToPoseRequest, self).__init__(
            request_id=str(request_id),
            pose=pose if pose is not None else Pose2D(),
            timeout_sec=int(timeout_sec),
        )


class NavigationStateUpdate(_ProtoBase):
    """状态更新：双向流的载荷。"""

    _FIELDS = ("request_id", "status", "current_pose", "failure", "timestamp_ms")
    _DEFAULTS = {
        "request_id": "",
        "status": NAVIGATION_STATUS_UNKNOWN,
        "current_pose": None,
        "failure": None,
        "timestamp_ms": 0,
    }

    def __init__(
        self,
        request_id: str = "",
        status: int = NAVIGATION_STATUS_UNKNOWN,
        current_pose: Optional[Pose2D] = None,
        failure: Optional[NavigationFailure] = None,
        timestamp_ms: int = 0,
    ):
        super(NavigationStateUpdate, self).__init__(
            request_id=str(request_id),
            status=int(status),
            current_pose=current_pose if current_pose is not None else Pose2D(),
            failure=failure if failure is not None else NavigationFailure(),
            timestamp_ms=int(timestamp_ms),
        )


class CancelRequest(_ProtoBase):
    """取消请求。"""

    _FIELDS = ("request_id",)
    _DEFAULTS = {"request_id": ""}

    def __init__(self, request_id: str = ""):
        super(CancelRequest, self).__init__(request_id=str(request_id))


class CancelResponse(_ProtoBase):
    """取消响应。"""

    _FIELDS = ("canceled", "message")
    _DEFAULTS = {"canceled": False, "message": ""}

    def __init__(self, canceled: bool = False, message: str = ""):
        super(CancelResponse, self).__init__(canceled=bool(canceled), message=str(message))


# 模块级符号导出（与 protoc-gen 兼容）
DESCRIPTOR = None  # stub 不暴露 FileDescriptor
__all__ = [
    "Pose2D",
    "NavigationFailure",
    "NavigateToPoseRequest",
    "NavigationStateUpdate",
    "CancelRequest",
    "CancelResponse",
    "NAVIGATION_STATUS_UNKNOWN",
    "NAVIGATION_STATUS_PENDING",
    "NAVIGATION_STATUS_ACTIVE",
    "NAVIGATION_STATUS_SUCCEEDED",
    "NAVIGATION_STATUS_FAILED",
    "NAVIGATION_STATUS_CANCELED",
    "NAVIGATION_FAILURE_UNKNOWN",
    "NAVIGATION_FAILURE_TIMEOUT",
    "NAVIGATION_FAILURE_OBSTACLE",
    "NAVIGATION_FAILURE_NO_PATH",
    "NAVIGATION_FAILURE_CANCELED",
    "NAVIGATION_FAILURE_QUEUE_FULL",
    "NAVIGATION_FAILURE_PROXY_UNAVAILABLE",
]