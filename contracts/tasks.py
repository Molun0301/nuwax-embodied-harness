from __future__ import annotations

from datetime import datetime
from contracts.compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import MetadataDict, TimestampedContract, utc_now
from contracts.naming import validate_capability_name, validate_event_type, validate_task_id
from typing import List, Optional


class TaskState(StrEnum):
    """任务状态。"""

    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"


class TaskSpec(TimestampedContract):
    """任务定义。"""

    task_id: str = Field(description="任务标识。")
    capability_name: str = Field(description="对应能力名称。")
    requested_by: Optional[str] = Field(default=None, description="调用方标识。")
    input_payload: MetadataDict = Field(default_factory=dict, description="任务输入。")
    required_resources: List[str] = Field(default_factory=list, description="任务依赖资源。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("task_id")
    @classmethod
    def _validate_task_id(cls, value: str) -> str:
        return validate_task_id(value)

    @field_validator("capability_name")
    @classmethod
    def _validate_capability_name(cls, value: str) -> str:
        return validate_capability_name(value)


class TaskStatus(TimestampedContract):
    """任务状态快照。"""

    task_id: str = Field(description="任务标识。")
    state: TaskState = Field(description="当前任务状态。")
    progress: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="任务进度。")
    stage: Optional[str] = Field(default=None, description="当前阶段。")
    message: Optional[str] = Field(default=None, description="当前状态说明。")
    started_at: Optional[datetime] = Field(default=None, description="开始时间。")
    updated_at: datetime = Field(default_factory=utc_now, description="状态更新时间。")
    completed_at: Optional[datetime] = Field(default=None, description="结束时间。")

    @field_validator("task_id")
    @classmethod
    def _validate_task_id(cls, value: str) -> str:
        return validate_task_id(value)


class TaskEvent(TimestampedContract):
    """任务事件。"""

    event_id: str = Field(description="事件标识。")
    event_type: str = Field(description="事件类型。")
    task_id: Optional[str] = Field(default=None, description="关联任务标识。")
    state: Optional[TaskState] = Field(default=None, description="事件触发后的任务状态。")
    message: Optional[str] = Field(default=None, description="事件说明。")
    payload: MetadataDict = Field(default_factory=dict, description="事件载荷。")

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
