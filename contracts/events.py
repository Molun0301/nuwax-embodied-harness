from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import MetadataDict, TimestampedContract
from contracts.naming import validate_event_type, validate_task_id
from typing import Optional


class RuntimeEventCategory(StrEnum):
    """运行时事件分类。"""

    TASK = "task"
    ROBOT = "robot"
    SAFETY = "safety"
    PERCEPTION = "perception"
    SYSTEM = "system"


class RuntimeEventSeverity(StrEnum):
    """运行时事件等级。"""

    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class RuntimeEvent(TimestampedContract):
    """运行时统一事件。"""

    event_id: str = Field(description="事件标识。")
    event_type: str = Field(description="事件类型。")
    category: RuntimeEventCategory = Field(description="事件分类。")
    cursor: Optional[int] = Field(default=None, ge=1, description="事件游标。")
    source: str = Field(description="事件来源。")
    subject_id: Optional[str] = Field(default=None, description="事件主体标识。")
    task_id: Optional[str] = Field(default=None, description="关联任务标识。")
    severity: RuntimeEventSeverity = Field(default=RuntimeEventSeverity.INFO, description="事件等级。")
    message: Optional[str] = Field(default=None, description="事件说明。")
    payload: MetadataDict = Field(default_factory=dict, description="事件载荷。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("event_type")
    @classmethod
    def _validate_event_type(cls, value: str) -> str:
        return validate_event_type(value)

    @field_validator("task_id")
    @classmethod
    def _validate_task_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return value
        return validate_task_id(value)
