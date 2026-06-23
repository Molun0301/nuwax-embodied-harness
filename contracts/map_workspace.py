from __future__ import annotations

from datetime import datetime

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
    latest_version_id: Optional[str] = Field(default=None, description="最近一版地图版本标识。")
    latest_revision: int = Field(default=0, ge=0, description="最近地图修订号。")
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
    """平台地图版本几何资产类型。"""

    EXTERNAL_SLAM_NAV2_STATE = "external_slam_nav2_state"
    PCD = "pcd"
    # RTAB-Map SQLite .db 文件——当前主线生产资产类型。
    RTABMAP_DATABASE = "rtabmap_database"


class MapGeometryAssetRef(ContractModel):
    """平台地图版本中的单个几何资产引用。"""

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
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("几何资产路径不能为空。")
        return resolved

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
    """平台地图版本绑定的几何资产描述。"""

    map_name: str = Field(description="所属地图名称。")
    version_id: str = Field(description="关联的平台地图版本标识。")
    geometry_stack: str = Field(default="external_slam_nav2", description="当前绑定的几何栈标识。")
    frame_id: Optional[str] = Field(default=None, description="该版本几何主图层坐标系。")
    rtabmap_database_path: str = Field(description="RTAB-Map 数据库文件路径。")
    # 老命名兼容：旧 version.json 仍然写 rtabmap_db_path；构造时若只给该字段会被迁移到 rtabmap_database_path。
    rtabmap_db_path: Optional[str] = Field(default=None, description="[DEPRECATED] 老版本 rtabmap_db_path 兼容字段，新代码请使用 rtabmap_database_path。")
    pcd_path: Optional[str] = Field(default=None, description="PCD 点云地图资产路径。")
    occupancy_projection: str = Field(description="占据栅格投影来源说明。")
    costmap_source: str = Field(description="代价地图来源说明。")
    parameter_summary: MetadataDict = Field(default_factory=dict, description="几何栈参数摘要。")
    profile_summary: MetadataDict = Field(default_factory=dict, description="几何 profile 摘要。")
    quality_metrics: MetadataDict = Field(default_factory=dict, description="几何资产质量指标。")
    assets: List[MapGeometryAssetRef] = Field(default_factory=list, description="该地图版本绑定的几何资产列表。")
    built_at: datetime = Field(default_factory=utc_now, description="该几何资产构建时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "version_id", "geometry_stack", "rtabmap_database_path", "occupancy_projection", "costmap_source")
    @classmethod
    def _validate_required_geometry_text(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图几何资产字段不能为空。")
        return resolved

    @field_validator("rtabmap_db_path")
    @classmethod
    def _validate_legacy_rtabmap_db_path(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        return resolved or None

    @field_validator("pcd_path")
    @classmethod
    def _validate_optional_geometry_path(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        return resolved or None

    @field_validator("frame_id")
    @classmethod
    def _validate_geometry_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)

    @model_validator(mode="before")
    @classmethod
    def _migrate_legacy_rtabmap_database_path(cls, data):
        """老 version.json 只有 rtabmap_db_path/旧后端状态路径时一次性迁移。

        新代码统一写 rtabmap_database_path；rtabmap_db_path 保留 read-side 兼容。
        """

        if not isinstance(data, dict):
            return data
        legacy_backend_state_path = data.pop("backend_state_path", None)
        if not data.get("rtabmap_database_path"):
            legacy_path = data.get("rtabmap_db_path") or legacy_backend_state_path
            if legacy_path:
                data["rtabmap_database_path"] = legacy_path
        # 双向回写：保证旧消费方读到的 rtabmap_db_path 仍然是合法路径。
        if data.get("rtabmap_database_path") and not data.get("rtabmap_db_path"):
            data["rtabmap_db_path"] = data["rtabmap_database_path"]
        return data

    @model_validator(mode="after")
    def _derive_geometry_asset_refs(self) -> "MapGeometryAssetBinding":
        if self.assets:
            return self
        profile_summary = dict(self.profile_summary or self.parameter_summary.get("profile") or {})
        stack_lower = str(self.geometry_stack or "").strip().lower()
        is_external_stack = stack_lower == "external_slam_nav2"
        # external_slam_nav2 当前默认后端是 RTAB-Map。
        state_asset_type = MapGeometryAssetType.RTABMAP_DATABASE
        database_quality = dict(self.quality_metrics.get("database") or {})
        database_required = bool(database_quality.get("required", not is_external_stack))
        database_present = bool(database_quality.get("present", False))
        refs = []
        if database_required or database_present:
            refs.append(
                MapGeometryAssetRef(
                    asset_type=state_asset_type,
                    path=self.rtabmap_database_path,
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


class MapVersion(TimestampedContract):
    """平台侧持久化地图版本。"""

    map_name: str = Field(description="所属地图名称。")
    version_id: str = Field(description="平台地图版本标识。")
    revision: int = Field(default=1, ge=1, description="平台侧递增地图修订号。")
    source_name: str = Field(description="生成该版本时使用的地图来源名称。")
    source_version_id: str = Field(description="生成该版本时的来源快照版本号。")
    frame_id: Optional[str] = Field(default=None, description="该版本对应的地图坐标系。")
    snapshot: MapSnapshot = Field(description="持久化的地图快照。")
    geometry_asset: Optional[MapGeometryAssetBinding] = Field(
        default=None,
        description="该版本绑定的几何资产信息。",
    )
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "version_id", "source_name", "source_version_id")
    @classmethod
    def _validate_required_text(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图版本字段不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_version_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip()
        if not resolved:
            return None
        return validate_frame_id(resolved)
