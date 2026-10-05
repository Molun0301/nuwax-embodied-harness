from __future__ import annotations

import math
import unicodedata
from copy import deepcopy
from datetime import datetime
from pathlib import PurePosixPath
from typing import Dict, List, Literal, Mapping, Optional

from pydantic import ConfigDict, Field, PrivateAttr, field_serializer, field_validator, model_serializer, model_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract, utc_now
from contracts.compat import StrEnum
from contracts.map_identity import AssetIdentityV2
from contracts.memory import MemoryStoreSummary
from contracts.naming import validate_frame_id
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import (
    ExplorationContext,
    LocalizationSession,
    LocalizationSnapshot,
    MapSnapshot,
    NavigationContext,
)

MAP_GEOREFERENCE_SCHEMA_VERSION = 1
MAP_GEOREFERENCE_RELATIVE_PATH = "georeference/georeference-v1.json"
MAP_GEOREFERENCE_SOURCE_CRS = "EPSG:4326"
MAP_GEOREFERENCE_PROJECTED_CRS = "EPSG:3857"
MAP_GEOREFERENCE_AXIS_CONVENTION = "projected_x_east_y_north_z_up"
MAP_GEOREFERENCE_ALTITUDE_POLICY = "map_plane_constant"
MAP_GEOREFERENCE_MAX_LATITUDE_DEG = 85.051_128_779_806_6
MAP_GEOREFERENCE_MAX_COORDINATE_M = 1_000_000_000.0


class MapGeoreferenceV1(ContractModel):
    """最终地图资产使用的不可变地理参考第一版合同。"""

    model_config = ConfigDict(frozen=True)

    schema_version: int = Field(description="地理参考合同版本，固定为 1。")
    map_name: str = Field(description="最终地图名称。")
    map_frame_id: str = Field(description="右手系且 Z 轴向上的地图坐标系。")
    source_crs: str = Field(description="地理坐标参考系，固定为 EPSG:4326。")
    projected_crs: str = Field(description="投影坐标参考系，固定为 EPSG:3857。")
    axis_convention: str = Field(description="投影坐标轴约定。")
    anchor_lat_deg: float = Field(description="锚点的 WGS84 纬度。")
    anchor_lng_deg: float = Field(description="锚点的 WGS84 经度。")
    anchor_easting_m: float = Field(description="锚点的 EPSG:3857 东向坐标。")
    anchor_northing_m: float = Field(description="锚点的 EPSG:3857 北向坐标。")
    map_origin_x_m: float = Field(description="锚点对应的地图坐标系 X 坐标。")
    map_origin_y_m: float = Field(description="锚点对应的地图坐标系 Y 坐标。")
    map_from_projected_yaw_rad: float = Field(description="投影坐标系到地图坐标系的逆时针偏航角。")
    altitude_policy: str = Field(description="高度策略，固定为地图平面常量。")
    map_plane_z_m: float = Field(description="所有目标使用的地图平面 Z 坐标。")

    @field_validator("schema_version", mode="before")
    @classmethod
    def _validate_georeference_schema_version(cls, value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value != MAP_GEOREFERENCE_SCHEMA_VERSION:
            raise ValueError("地理参考 schema_version 必须为 1。")
        return value

    @field_validator("map_name")
    @classmethod
    def _validate_georeference_map_name(cls, value: str) -> str:
        if (
            not value
            or value.strip() != value
            or len(value.encode("utf-8")) > 255
            or any(unicodedata.category(character) == "Cc" for character in value)
        ):
            raise ValueError("地理参考 map_name 必须规范、无控制字符且不超过 255 字节。")
        return value

    @field_validator("map_frame_id")
    @classmethod
    def _validate_georeference_frame_id(cls, value: str) -> str:
        if value.strip() != value or len(value.encode("utf-8")) > 255:
            raise ValueError("地理参考 map_frame_id 非法。")
        return validate_frame_id(value)

    @field_validator(
        "anchor_lat_deg",
        "anchor_lng_deg",
        "anchor_easting_m",
        "anchor_northing_m",
        "map_origin_x_m",
        "map_origin_y_m",
        "map_from_projected_yaw_rad",
        "map_plane_z_m",
        mode="before",
    )
    @classmethod
    def _validate_georeference_number_input(cls, value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("地理参考数值字段必须为数字。")
        return float(value)

    @model_validator(mode="after")
    def _validate_georeference_identity(self) -> MapGeoreferenceV1:
        required_labels = {
            "source_crs": MAP_GEOREFERENCE_SOURCE_CRS,
            "projected_crs": MAP_GEOREFERENCE_PROJECTED_CRS,
            "axis_convention": MAP_GEOREFERENCE_AXIS_CONVENTION,
            "altitude_policy": MAP_GEOREFERENCE_ALTITUDE_POLICY,
        }
        for field_name, expected in required_labels.items():
            if getattr(self, field_name) != expected:
                raise ValueError(f"地理参考 {field_name} 必须为 {expected}。")

        numeric_fields = (
            "anchor_lat_deg",
            "anchor_lng_deg",
            "anchor_easting_m",
            "anchor_northing_m",
            "map_origin_x_m",
            "map_origin_y_m",
            "map_from_projected_yaw_rad",
            "map_plane_z_m",
        )
        for field_name in numeric_fields:
            value = getattr(self, field_name)
            if not math.isfinite(value):
                raise ValueError(f"地理参考 {field_name} 必须为有限数。")
            if value == 0.0 and math.copysign(1.0, value) < 0.0:
                raise ValueError(f"地理参考 {field_name} 的零值必须使用正零编码。")

        if not (-MAP_GEOREFERENCE_MAX_LATITUDE_DEG <= self.anchor_lat_deg <= MAP_GEOREFERENCE_MAX_LATITUDE_DEG):
            raise ValueError("地理参考 anchor_lat_deg 超出 EPSG:3857 范围。")
        if not -180.0 <= self.anchor_lng_deg <= 180.0:
            raise ValueError("地理参考 anchor_lng_deg 超出 WGS84 范围。")
        if not -math.pi <= self.map_from_projected_yaw_rad < math.pi:
            raise ValueError("地理参考 map_from_projected_yaw_rad 必须位于 [-pi, pi)。")
        for field_name in ("map_origin_x_m", "map_origin_y_m", "map_plane_z_m"):
            if abs(getattr(self, field_name)) > MAP_GEOREFERENCE_MAX_COORDINATE_M:
                raise ValueError(f"地理参考 {field_name} 超出地图坐标范围。")

        latitude_rad = math.radians(self.anchor_lat_deg)
        expected_easting = 6_378_137.0 * math.radians(self.anchor_lng_deg)
        expected_northing = 6_378_137.0 * math.log(math.tan(math.pi / 4.0 + latitude_rad / 2.0))
        if (
            abs(expected_easting - self.anchor_easting_m) > 1.0e-6
            or abs(expected_northing - self.anchor_northing_m) > 1.0e-6
        ):
            raise ValueError("地理参考投影锚点与 WGS84 锚点不一致。")
        return self

    def model_copy(
        self,
        *,
        update: Optional[Mapping[str, object]] = None,
        deep: bool = False,
    ) -> "MapGeoreferenceV1":
        return _validated_model_copy(self, update=update, deep=deep)


class MapGeoreferenceBindingV1(ContractModel):
    """地理参考合同与资产包内精确原文字节的绑定。"""

    model_config = ConfigDict(frozen=True)

    schema_version: int = Field(
        default=MAP_GEOREFERENCE_SCHEMA_VERSION,
        description="绑定版本，固定为 1。",
    )
    relative_path: str = Field(
        default=MAP_GEOREFERENCE_RELATIVE_PATH,
        description="地理参考文件的规范相对路径。",
    )
    raw_sha256: Optional[str] = Field(
        default=None,
        description="地理参考原文字节 SHA-256；候选发布时计算，最终资产中必须存在。",
    )
    contract: MapGeoreferenceV1 = Field(description="经过类型校验的地理参考合同。")

    @field_validator("schema_version", mode="before")
    @classmethod
    def _validate_binding_schema_version(cls, value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value != MAP_GEOREFERENCE_SCHEMA_VERSION:
            raise ValueError("地理参考绑定 schema_version 必须为 1。")
        return value

    @field_validator("relative_path")
    @classmethod
    def _validate_binding_relative_path(cls, value: str) -> str:
        if value != MAP_GEOREFERENCE_RELATIVE_PATH:
            raise ValueError("地理参考文件必须使用固定规范路径。")
        return value

    @field_validator("raw_sha256")
    @classmethod
    def _validate_binding_raw_sha256(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        if (
            value.strip() != value
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
            or value == "0" * 64
        ):
            raise ValueError("地理参考原文 SHA-256 非法。")
        return value

    def model_copy(
        self,
        *,
        update: Optional[Mapping[str, object]] = None,
        deep: bool = False,
    ) -> "MapGeoreferenceBindingV1":
        changes = dict(update or {})
        if self.raw_sha256 is not None:
            for field_name in (
                "schema_version",
                "relative_path",
                "raw_sha256",
                "contract",
            ):
                if field_name in changes and changes[field_name] != getattr(self, field_name):
                    raise TypeError("已绑定原文摘要的地理参考不可改写。")
        return _validated_model_copy(self, update=changes, deep=deep)


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
    identity_schema_version: Literal[1, 2] = 1
    asset_identity: Optional[AssetIdentityV2] = None
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


    @model_serializer(mode="wrap")
    def _serialize_asset_generation(self, handler):
        document = handler(self)
        if self.identity_schema_version == 1:
            document.pop("identity_schema_version", None)
            document.pop("asset_identity", None)
        return document

    @model_validator(mode="before")
    @classmethod
    def _validate_asset_identity_generation(cls, value):
        if isinstance(value, dict):
            _validate_identity_generation(value, "asset_manifest_sha256")
        return value

    @model_validator(mode="after")
    def _validate_asset_identity_pair(self):
        _validate_identity_generation({"identity_schema_version": self.identity_schema_version,
            "asset_identity": self.asset_identity, "asset_manifest_sha256": self.asset_manifest_sha256},
            "asset_manifest_sha256")
        return self


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
    PLANAR_GRAPH = "planar_graph"
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
    mapping_database_path: str = Field(description="完整建图数据库或二维图档案文件路径。")
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

    model_config = ConfigDict(frozen=True)

    map_name: str = Field(description="所属地图名称。")
    source_name: str = Field(description="生成资产包时使用的地图来源名称。")
    frame_id: Optional[str] = Field(default=None, description="资产包对应的地图坐标系。")
    snapshot: MapSnapshot = Field(description="持久化的地图快照。")
    geometry_asset: Optional[MapGeometryAssetBinding] = Field(
        default=None,
        description="资产包绑定的几何资产信息。",
    )
    georeference: Optional[MapGeoreferenceBindingV1] = Field(
        default=None,
        description="资产包绑定的不可变地理参考；缺失表示只有本地地图坐标。",
    )
    identity_schema_version: Literal[1, 2] = 1
    asset_identity: Optional[AssetIdentityV2] = None
    asset_state: Literal["legacy", "candidate", "finalized"] = "legacy"
    _assigned_owned: bool = PrivateAttr(default=False)
    manifest_sha256: Optional[str] = Field(default=None, description="资产包清单 SHA-256；加载时计算。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "source_name")
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

    @model_validator(mode="before")
    @classmethod
    def _validate_bundle_identity_generation(cls, value):
        if isinstance(value, dict):
            _validate_identity_generation(value, "manifest_sha256")
            assigned = value.get("identity_schema_version", 1) == 2
            if (assigned and value.get("asset_state") not in {"candidate", "finalized"}
                    or not assigned and value.get("asset_state", "legacy") != "legacy"):
                raise ValueError("asset_state must explicitly match its identity generation")
        return value

    @model_validator(mode="after")
    def _validate_bundle_identity(self) -> "MapAssetBundle":
        if self.source_name != self.snapshot.source_name:
            raise ValueError("地图资产包 source_name 与运行态快照不一致。")
        if self.map_name != self.snapshot.map_name:
            raise ValueError("地图资产包 map_name 与运行态快照不一致。")
        if self.geometry_asset is not None and self.geometry_asset.map_name != self.map_name:
            raise ValueError("地图资产包与 geometry_asset 的 map_name 不一致。")
        if not self.frame_id:
            raise ValueError("地图资产包必须声明 frame_id。")
        if self.georeference is not None:
            if self.georeference.contract.map_name != self.map_name:
                raise ValueError("地图资产包与 georeference 的 map_name 不一致。")
            if self.georeference.contract.map_frame_id != self.frame_id:
                raise ValueError("地图资产包与 georeference 的 frame_id 不一致。")
        if (
            self.geometry_asset is not None
            and self.frame_id
            and self.geometry_asset.frame_id
            and self.frame_id != self.geometry_asset.frame_id
        ):
            raise ValueError("地图资产包与 geometry_asset 的 frame_id 不一致。")
        if self.frame_id and self.snapshot.frame_id and self.frame_id != self.snapshot.frame_id:
            raise ValueError("地图资产包与运行态快照的 frame_id 不一致。")
        _validate_identity_generation({"identity_schema_version": self.identity_schema_version,
            "asset_identity": self.asset_identity, "manifest_sha256": self.manifest_sha256}, "manifest_sha256")
        if self.identity_schema_version == 2:
            if self.asset_state not in {"candidate", "finalized"}:
                raise ValueError("assigned bundles require an explicit candidate/finalized state")
            if self.georeference is not None and self.georeference.raw_sha256 is not None:
                raise ValueError("assigned georeference may not carry a legacy digest")
            if not self._assigned_owned:
                # Capture once. Subsequent reads and state promotion share immutable
                # cell owners and copy only mutable, small descriptors.
                object.__setattr__(self, "snapshot", _owned_snapshot_view(
                    super().__getattribute__("snapshot"), capture=True))
                object.__setattr__(self, "geometry_asset", deepcopy(super().__getattribute__("geometry_asset")))
                object.__setattr__(self, "metadata", deepcopy(super().__getattribute__("metadata")))
                self._assigned_owned = True
        return self

    @model_serializer(mode="wrap")
    def _serialize_bundle_generation(self, handler):
        document = handler(self)
        if self.identity_schema_version == 1:
            for key in ("identity_schema_version", "asset_identity", "asset_state"):
                document.pop(key, None)
        return document

    @field_serializer("snapshot", mode="wrap", when_used="json")
    def _serialize_snapshot(self, snapshot, handler, info):
        if self.identity_schema_version != 2:
            return handler(snapshot)
        # Tuple-backed cost cells are immutable internally. Expansion is confined
        # to this explicit serialization boundary, never ordinary owner reads.
        document = snapshot.model_dump(mode="json", exclude={"cost_map": {"data"}}, exclude_none=info.exclude_none)
        if snapshot.cost_map is not None:
            document["cost_map"]["data"] = list(snapshot.cost_map.data)
        return document

    @property
    def snapshot_identity(self) -> Dict[str, object]:
        """只复制来源身份标量，不为读取修订而复制完整已封存图层。"""
        snapshot = super().__getattribute__("snapshot")
        return {"schema_version": snapshot.schema_version, "map_name": snapshot.map_name,
                "source_name": snapshot.source_name, "source_epoch": snapshot.source_epoch,
                "frame_id": snapshot.frame_id, "update_sequence": str(snapshot.update_sequence)}

    def __getattribute__(self, name: str):
        value = super().__getattribute__(name)
        if name not in {"snapshot", "geometry_asset", "metadata"}:
            return value
        try:
            assigned = super().__getattribute__("identity_schema_version") == 2
            owned = super().__getattribute__("__pydantic_private__").get("_assigned_owned", False)
            if assigned and owned:
                return _owned_snapshot_view(value) if name == "snapshot" else deepcopy(value)
            manifest_sha256 = super().__getattribute__("manifest_sha256")
        except AttributeError:
            return value
        if manifest_sha256 is None:
            return value
        return deepcopy(value)

    def __deepcopy__(self, memo=None):
        if self.identity_schema_version == 2:
            copied = self.model_copy()
            if memo is not None:
                memo[id(self)] = copied
            return copied
        return super().__deepcopy__(memo=memo)

    def model_copy(
        self,
        *,
        update: Optional[Mapping[str, object]] = None,
        deep: bool = False,
    ) -> "MapAssetBundle":
        changes = dict(update or {})
        if self.identity_schema_version == 2:
            effective = {key: value for key, value in changes.items()
                         if value != super(MapAssetBundle, self).__getattribute__(key)}
            if effective and not (effective == {"asset_state": "finalized"} and self.asset_state == "candidate"):
                raise TypeError("assigned asset owners are immutable; allocate a new candidate for changes")
            # Pydantic's generic deep copy expands packed cells. The owner is
            # already isolated and its public descriptors are detached views.
            return super().model_copy(update=effective, deep=False)
        if self.manifest_sha256 is not None:
            changed_fields = {field_name for field_name, value in changes.items() if value != getattr(self, field_name)}
            if changed_fields:
                raise TypeError("已绑定清单摘要的地图资产包不可改写。")
        return _validated_model_copy(self, update=changes, deep=deep)


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


def _validated_model_copy(
    model: ContractModel,
    *,
    update: Optional[Mapping[str, object]],
    deep: bool,
):
    payload: Dict[str, object] = model.model_dump(mode="python")
    if deep:
        payload = deepcopy(payload)
    payload.update(deepcopy(dict(update or {})) if deep else dict(update or {}))
    return type(model).model_validate(payload)


def _validate_identity_generation(value, digest_field):
    schema = value.get("identity_schema_version", 1)
    if type(schema) is not int or schema not in {1, 2}:
        raise ValueError("identity_schema_version must be explicitly 1 or 2")
    identity = value.get("asset_identity")
    if schema == 2:
        if identity is None or value.get(digest_field) is not None:
            raise ValueError("assigned assets require their assigned ID and forbid legacy digests")
    elif identity is not None:
        raise ValueError("legacy assets cannot carry assigned identities")


def _owned_snapshot_view(snapshot, *, capture=False):
    """Copy descriptor state without recursively cloning snapshot cell bodies."""
    changes = {}
    for field_name in type(snapshot).model_fields:
        value = object.__getattribute__(snapshot, field_name)
        if field_name in {"occupancy_grid", "cost_map"} and value is not None:
            layer_changes = {key: deepcopy(object.__getattribute__(value, key))
                             for key in type(value).model_fields if key != "data"}
            data = object.__getattribute__(value, "data")
            if capture:
                if field_name == "occupancy_grid" and type(data) is not PackedOccupancyCells:
                    data = PackedOccupancyCells(bytes(cell & 255 for cell in data))
                elif field_name == "cost_map":
                    data = tuple(data)
            layer_changes["data"] = data
            changes[field_name] = value.model_copy(update=layer_changes, deep=False)
        else:
            changes[field_name] = deepcopy(value)
    return snapshot.model_copy(update=changes, deep=False)
