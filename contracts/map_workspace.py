from __future__ import annotations

from datetime import datetime
from pathlib import PurePosixPath

from contracts.compat import StrEnum

from pydantic import Field, field_validator, model_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract, utc_now
from contracts.memory import MemoryStoreSummary
from contracts.naming import validate_frame_id
from contracts.runtime_views import ExplorationContext, LocalizationSession, LocalizationSnapshot, MapSnapshot, NavigationContext
from typing import List, Optional


class MapAssetStatus(StrEnum):
    """地图资产状态。"""

    CREATED = "created"
    ACTIVATING = "activating"
    MAPPING = "mapping"
    LOCALIZING = "localizing"
    READY = "ready"
    STALE = "stale"
    DELETED = "deleted"


class MapAsset(TimestampedContract):
    """命名地图资产元数据。"""

    map_name: str = Field(description="地图名称，作为地图资产主键。")
    display_name: str = Field(description="地图展示名称。")
    bound_memory_library_name: str = Field(description="绑定的同名记忆库名称。")
    status: MapAssetStatus = Field(default=MapAssetStatus.CREATED, description="地图资产状态。")
    active: bool = Field(default=False, description="当前是否为激活地图。")
    finalized: bool = Field(default=False, description="地图是否已有不可变的 finalized 资产包。")
    asset_manifest_sha256: Optional[str] = Field(default=None, description="finalized 资产包清单 SHA-256。")
    finalized_at: Optional[datetime] = Field(default=None, description="地图资产包完成原子提交的时间。")
    frame_id: Optional[str] = Field(default=None, description="当前地图使用的坐标系。")
    localization_ready: bool = Field(default=False, description="当前是否已完成定位。")
    created_at: datetime = Field(default_factory=utc_now, description="创建时间。")
    updated_at: datetime = Field(default_factory=utc_now, description="更新时间。")
    last_activated_at: Optional[datetime] = Field(default=None, description="最近一次激活时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "display_name", "bound_memory_library_name")
    @classmethod
    def _validate_names(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图名称不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)

    @field_validator("asset_manifest_sha256")
    @classmethod
    def _validate_asset_manifest_sha256(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip().lower()
        if len(resolved) != 64 or any(char not in "0123456789abcdef" for char in resolved):
            raise ValueError("地图资产清单 SHA-256 非法。")
        return resolved


class MapWorkspace(TimestampedContract):
    """当前激活地图工作区的聚合视图。"""

    active_map_name: Optional[str] = Field(default=None, description="当前激活地图名称。")
    active_memory_library_name: Optional[str] = Field(default=None, description="当前激活记忆库名称。")
    map_asset: Optional[MapAsset] = Field(default=None, description="当前激活地图资产。")
    map_loaded: bool = Field(default=False, description="当前地图快照是否已加载。")
    localization_ready: bool = Field(default=False, description="当前是否已完成定位。")
    active_localization_session: Optional[LocalizationSession] = Field(
        default=None,
        description="当前激活的平台定位会话。",
    )
    mapping_runtime_enabled: bool = Field(default=False, description="当前建图运行态是否启用。")
    latest_map_snapshot: Optional[MapSnapshot] = Field(default=None, description="当前地图快照。")
    latest_localization_snapshot: Optional[LocalizationSnapshot] = Field(default=None, description="当前定位快照。")
    latest_memory_summary: Optional[MemoryStoreSummary] = Field(default=None, description="当前记忆摘要。")
    navigation_context: Optional[NavigationContext] = Field(default=None, description="当前导航上下文。")
    exploration_context: Optional[ExplorationContext] = Field(default=None, description="当前探索上下文。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加工作区元数据。")


class MapGeometryAssetType(StrEnum):
    """平台地图资产包几何资产类型。"""

    NAVIGATION_MAP = "navigation_map"
    PCD = "pcd"
    MAPPING_DATABASE = "mapping_database"
    NUWAX_MAPPING_OCCUPANCY_GRID = "nuwax_mapping_occupancy_grid"
    NUWAX_MAPPING_COSTMAP = "nuwax_mapping_costmap"
    NUWAX_MAPPING_CLOUD_MAP = "nuwax_mapping_cloud_map"


class MapGeometryAssetRef(ContractModel):
    """平台地图资产包中的单个几何资产引用。"""

    asset_type: MapGeometryAssetType = Field(description="几何资产类型。")
    path: str = Field(description="几何资产文件路径。")
    frame_id: Optional[str] = Field(default=None, description="资产对应坐标系。")
    profile_summary: MetadataDict = Field(default_factory=dict, description="生成该资产时的 profile 摘要。")
    built_at: datetime = Field(default_factory=utc_now, description="资产构建时间。")
    quality_metrics: MetadataDict = Field(default_factory=dict, description="资产质量指标。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("path")
    @classmethod
    def _validate_asset_path(cls, value: str) -> str:
        return _validate_bundle_relative_path(value, "几何资产路径")

    @field_validator("frame_id")
    @classmethod
    def _validate_asset_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)


class MapGeometryAssetBinding(ContractModel):
    """命名地图的 finalized 几何资产描述。"""

    map_name: str = Field(description="所属地图名称。")
    geometry_stack: str = Field(default="nuwax_geometry_runtime", description="当前绑定的几何栈标识。")
    frame_id: Optional[str] = Field(default=None, description="该几何资产的主图层坐标系。")
    mapping_database_path: str = Field(description="建图数据库文件路径。")
    pcd_path: Optional[str] = Field(default=None, description="PCD 点云地图资产路径。")
    occupancy_projection: str = Field(description="占据栅格投影来源说明。")
    costmap_source: str = Field(description="代价地图来源说明。")
    parameter_summary: MetadataDict = Field(default_factory=dict, description="几何栈参数摘要。")
    profile_summary: MetadataDict = Field(default_factory=dict, description="几何 profile 摘要。")
    quality_metrics: MetadataDict = Field(default_factory=dict, description="几何资产质量指标。")
    assets: List[MapGeometryAssetRef] = Field(default_factory=list, description="该地图资产包绑定的几何资产列表。")
    built_at: datetime = Field(default_factory=utc_now, description="该几何资产构建时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "geometry_stack", "occupancy_projection", "costmap_source")
    @classmethod
    def _validate_required_geometry_text(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图几何资产字段不能为空。")
        return resolved

    @field_validator("mapping_database_path")
    @classmethod
    def _validate_mapping_database_path(cls, value: str) -> str:
        return _validate_bundle_relative_path(value, "建图数据库路径")

    @field_validator("pcd_path")
    @classmethod
    def _validate_optional_geometry_path(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        return _validate_bundle_relative_path(resolved, "PCD 路径") if resolved else None

    @field_validator("frame_id")
    @classmethod
    def _validate_geometry_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)

    @model_validator(mode="after")
    def _derive_geometry_asset_refs(self) -> "MapGeometryAssetBinding":
        if self.assets:
            return self
        profile_summary = dict(self.profile_summary or self.parameter_summary.get("profile") or {})
        state_asset_type = MapGeometryAssetType.MAPPING_DATABASE
        database_quality = dict(self.quality_metrics.get("database") or {})
        database_required = bool(database_quality.get("required", True))
        database_present = bool(database_quality.get("present", False))
        refs = []
        if database_required or database_present:
            refs.append(
                MapGeometryAssetRef(
                    asset_type=state_asset_type,
                    path=self.mapping_database_path,
                    frame_id=self.frame_id,
                    profile_summary=profile_summary,
                    built_at=self.built_at,
                    quality_metrics=database_quality,
                    metadata={"required": database_required},
                )
            )
        if self.pcd_path:
            refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.PCD,
                    path=self.pcd_path,
                    frame_id=self.frame_id,
                    profile_summary=profile_summary,
                    built_at=self.built_at,
                    quality_metrics=dict(self.quality_metrics.get("pcd") or {}),
                )
            )
        self.assets = refs
        return self


class MapAssetBundle(TimestampedContract):
    """一个 map_name 唯一且不可变的 finalized 地图资产包。"""

    map_name: str = Field(description="所属地图名称。")
    source_name: str = Field(description="生成资产包时使用的地图来源名称。")
    source_snapshot_id: str = Field(description="生成资产包时的运行态快照标识。")
    frame_id: Optional[str] = Field(default=None, description="资产包对应的地图坐标系。")
    snapshot: MapSnapshot = Field(description="持久化的地图快照。")
    geometry_asset: Optional[MapGeometryAssetBinding] = Field(
        default=None,
        description="资产包绑定的几何资产信息。",
    )
    manifest_sha256: Optional[str] = Field(default=None, description="资产包清单 SHA-256；加载时计算。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "source_name", "source_snapshot_id")
    @classmethod
    def _validate_required_text(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图资产包字段不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_bundle_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)

    @field_validator("manifest_sha256")
    @classmethod
    def _validate_manifest_sha256(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip().lower()
        if len(resolved) != 64 or any(char not in "0123456789abcdef" for char in resolved):
            raise ValueError("地图资产包清单 SHA-256 非法。")
        return resolved

    @model_validator(mode="after")
    def _validate_bundle_identity(self) -> "MapAssetBundle":
        if self.source_name != self.snapshot.source_name:
            raise ValueError("地图资产包 source_name 与运行态快照不一致。")
        if self.source_snapshot_id != self.snapshot.version_id:
            raise ValueError("地图资产包 source_snapshot_id 与运行态快照不一致。")
        if self.geometry_asset is not None and self.geometry_asset.map_name != self.map_name:
            raise ValueError("地图资产包与 geometry_asset 的 map_name 不一致。")
        if (
            self.geometry_asset is not None
            and self.frame_id
            and self.geometry_asset.frame_id
            and self.frame_id != self.geometry_asset.frame_id
        ):
            raise ValueError("地图资产包与 geometry_asset 的 frame_id 不一致。")
        snapshot_map_name = str(self.snapshot.metadata.get("map_name") or "").strip()
        if snapshot_map_name and snapshot_map_name != self.map_name:
            raise ValueError("地图资产包与运行态快照的 map_name 不一致。")
        if self.frame_id and self.snapshot.frame_id and self.frame_id != self.snapshot.frame_id:
            raise ValueError("地图资产包与运行态快照的 frame_id 不一致。")
        return self


def _validate_bundle_relative_path(value: object, description: str) -> str:
    resolved = str(value or "").strip()
    if not resolved:
        raise ValueError("%s不能为空。" % description)
    if "\\" in resolved:
        raise ValueError("%s必须使用 POSIX 相对路径。" % description)
    path = PurePosixPath(resolved)
    if path.is_absolute() or resolved != path.as_posix() or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError("%s必须是资产包内的规范相对路径。" % description)
    return path.as_posix()
