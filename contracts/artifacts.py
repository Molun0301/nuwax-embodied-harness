from __future__ import annotations

from compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from contracts.naming import validate_artifact_id
from typing import Dict, List, Optional


class ArtifactKind(StrEnum):
    """制品类型。"""

    IMAGE = "image"
    MAP_SNAPSHOT = "map_snapshot"
    POINTCLOUD = "pointcloud"
    AUDIO = "audio"
    LOG = "log"
    DEBUG_BUNDLE = "debug_bundle"
    OTHER = "other"


class ArtifactRef(TimestampedContract):
    """制品引用。"""

    artifact_id: str = Field(description="制品标识。")
    kind: ArtifactKind = Field(description="制品类型。")
    mime_type: str = Field(description="MIME 类型。")
    size_bytes: Optional[int] = Field(default=None, ge=0, description="制品大小。")
    uri: Optional[str] = Field(default=None, description="制品访问地址。")
    sha256: Optional[str] = Field(default=None, description="文件摘要。")
    metadata: MetadataDict = Field(default_factory=dict, description="制品元数据。")

    @field_validator("artifact_id")
    @classmethod
    def _validate_artifact_id(cls, value: str) -> str:
        return validate_artifact_id(value)


class ArtifactRetentionPolicy(ContractModel):
    """制品清理策略。"""

    retention_days: Optional[int] = Field(default=None, ge=1, description="最大保留天数。")
    max_count: Optional[int] = Field(default=None, ge=1, description="最大保留制品数量。")
    max_total_bytes: Optional[int] = Field(default=None, ge=1, description="最大保留总字节数。")
    cleanup_batch_size: int = Field(default=200, ge=1, description="单次清理最多处理的制品数。")


class ArtifactStorageSummary(TimestampedContract):
    """制品存储摘要。"""

    artifact_count: int = Field(default=0, ge=0, description="当前制品数量。")
    total_size_bytes: int = Field(default=0, ge=0, description="当前制品总大小。")
    by_kind: Dict[str, int] = Field(default_factory=dict, description="按类型统计。")
    oldest_artifact_id: Optional[str] = Field(default=None, description="最旧制品标识。")
    newest_artifact_id: Optional[str] = Field(default=None, description="最新制品标识。")


class ArtifactCleanupResult(TimestampedContract):
    """制品清理结果。"""

    removed_count: int = Field(default=0, ge=0, description="清理掉的制品数量。")
    freed_bytes: int = Field(default=0, ge=0, description="释放的字节数。")
    removed_artifact_ids: List[str] = Field(default_factory=list, description="被清理掉的制品标识。")
    protected_count: int = Field(default=0, ge=0, description="因仍被引用而保留的制品数量。")
    protected_artifact_ids: List[str] = Field(default_factory=list, description="因引用保护保留的制品标识。")
    summary: ArtifactStorageSummary = Field(description="清理后的存储摘要。")
