from __future__ import annotations

from typing import Any, Optional

from pydantic import Field

from contracts.base import MetadataDict, TimestampedContract
from compat import StrEnum


class StreamPayloadKind(StrEnum):
    """实时流载荷类型。"""

    JSON = "json"
    BINARY = "binary"
    UNSUPPORTED = "unsupported"
    DIAGNOSTIC = "diagnostic"
    KEEPALIVE = "keepalive"


class StreamFrame(TimestampedContract):
    """平台实时数据帧。"""

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
