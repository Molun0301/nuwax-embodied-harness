from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import ContractModel, MetadataDict
from contracts.naming import validate_capability_name
from typing import List, Optional


class CapabilityExecutionMode(StrEnum):
    """能力执行模式。"""

    SYNC = "sync"
    ASYNC_TASK = "async_task"
    STREAM = "stream"


class CapabilityRiskLevel(StrEnum):
    """能力风险级别。"""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    ADMIN = "admin"


class CapabilityDescriptor(ContractModel):
    """能力元数据定义。"""

    name: str = Field(description="能力名称。")
    display_name: str = Field(description="中文展示名称。")
    description: str = Field(description="能力说明。")
    execution_mode: CapabilityExecutionMode = Field(description="执行模式。")
    risk_level: CapabilityRiskLevel = Field(description="风险级别。")
    input_contract: Optional[str] = Field(default=None, description="输入契约名称。")
    output_contract: Optional[str] = Field(default=None, description="输出契约名称。")
    input_schema: MetadataDict = Field(default_factory=dict, description="输入参数结构模式。")
    output_schema: MetadataDict = Field(default_factory=dict, description="输出结构模式。")
    required_resources: List[str] = Field(default_factory=list, description="依赖资源。")
    timeout_sec: Optional[int] = Field(default=None, ge=1, description="默认超时时间。")
    cancel_supported: bool = Field(default=False, description="是否支持取消。")
    exposed_to_agent: bool = Field(default=False, description="是否默认对上层智能体可见。")
    tags: List[str] = Field(default_factory=list, description="标签。")

    @field_validator("name")
    @classmethod
    def _validate_name(cls, value: str) -> str:
        return validate_capability_name(value)


class CapabilityAvailability(ContractModel):
    """能力可用性条目。"""

    name: str = Field(description="能力名称。")
    supported: bool = Field(description="是否支持。")
    reason: Optional[str] = Field(default=None, description="不支持或部分支持原因。")
    metadata: MetadataDict = Field(default_factory=dict, description="补充信息。")

    @field_validator("name")
    @classmethod
    def _validate_name(cls, value: str) -> str:
        return validate_capability_name(value)


class CapabilityMatrix(ContractModel):
    """机器人或平台能力矩阵。"""

    robot_model: str = Field(description="机器人型号。")
    capabilities: List[CapabilityAvailability] = Field(default_factory=list, description="能力可用性列表。")
    metadata: MetadataDict = Field(default_factory=dict, description="补充说明。")
