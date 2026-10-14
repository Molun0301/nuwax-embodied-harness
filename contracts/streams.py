from __future__ import annotations

from typing import Any, Literal, Optional

from pydantic import ConfigDict, Field

from contracts.base import MetadataDict, TimestampedContract
from contracts.compat import StrEnum


class StreamPayloadKind(StrEnum):
    """实时流载荷类型。"""

    JSON = "json"
    BINARY = "binary"
    UNSUPPORTED = "unsupported"
    DIAGNOSTIC = "diagnostic"
    KEEPALIVE = "keepalive"


class StreamFrame(TimestampedContract):
    """平台实时数据帧。"""

    schema_version: int = Field(default=1, ge=1, description="契约模式版本号，跨进程演进协商用，新增字段必须 Optional+default 保持向后兼容。")
    topic: str = Field(description="流主题。")
    payload_kind: StreamPayloadKind = Field(default=StreamPayloadKind.JSON, description="载荷类型。")
    cursor: Optional[int] = Field(default=None, ge=1, description="流游标。")
    source: str = Field(description="数据来源。")
    payload: Any = Field(default=None, description="数据载荷。")
    frame_id: Optional[str] = Field(default=None, description="坐标系标识。")
    sequence: Optional[int] = Field(default=None, description="源头序列号。")
    revision: Optional[str] = Field(default=None, description="内容修订号。")
    status: str = Field(default="available", description="数据状态。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class InstanceSnapshotFrame(StreamFrame):
    """生产者移交的公开实例快照；信封冻结，嵌套载荷按发布后只读共享。

    仅供 SemanticInstancePublisher 发布新建的公开行字典。构造后生产者及
    订阅者均不得修改或复用 payload/metadata；不复制、不扫描嵌套载荷。
    总线只用 model_copy 分配游标，保留此所有权契约。任意诊断输入继续使用
    普通 StreamFrame，不能仅声明相同 topic/source 就获得编码复用。
    """

    model_config = ConfigDict(frozen=True)
    topic: Literal["memory.instances"] = "memory.instances"
    payload_kind: Literal[StreamPayloadKind.JSON] = StreamPayloadKind.JSON
