from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import ContractModel, MetadataDict
from contracts.naming import validate_capability_name
from typing import List, Optional


class SkillCategory(StrEnum):
    """技能分类。"""

    SYSTEM = "system"
    OBSERVATION = "observation"
    MOTION = "motion"
    AUDIO = "audio"
    MEMORY = "memory"
    TASK = "task"
    ADMIN = "admin"


class SkillDescriptor(ContractModel):
    """技能元数据定义。"""

    name: str = Field(description="技能名称，也是对外工具名称。")
    display_name: str = Field(description="中文展示名称。")
    description: str = Field(description="技能说明。")
    category: SkillCategory = Field(description="技能分类。")
    capability_name: str = Field(description="底层绑定的能力名称。")
    exposed_to_agent: bool = Field(default=True, description="是否默认对普通智能体暴露。")
    tags: List[str] = Field(default_factory=list, description="技能标签。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("name", "capability_name")
    @classmethod
    def _validate_names(cls, value: str) -> str:
        return validate_capability_name(value)


class SkillRuntimeView(ContractModel):
    """技能运行时视图。"""

    descriptor: SkillDescriptor = Field(description="技能描述。")
    supported: bool = Field(description="当前是否支持。")
    runnable: bool = Field(description="底层能力是否可执行。")
    reason: Optional[str] = Field(default=None, description="不支持原因。")
    capability_name: str = Field(description="底层能力名称。")
    input_schema: MetadataDict = Field(default_factory=dict, description="输入结构模式。")
    output_schema: MetadataDict = Field(default_factory=dict, description="输出结构模式。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")
