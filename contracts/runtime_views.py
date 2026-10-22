from __future__ import annotations

from contracts.compat import StrEnum
from datetime import datetime
from copy import deepcopy
import math

from pydantic import ConfigDict, Field, PlainSerializer, PlainValidator, PrivateAttr, field_validator, model_serializer, model_validator

from contracts.artifacts import ArtifactRef
from contracts.base import ContractModel, MetadataDict, TimestampedContract, utc_now
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.image import CameraInfo
from contracts.maps import CostMap, OccupancyGrid, SemanticMap
from contracts.map_identity import AssetIdentityV2
from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import AssignedRelocalizationTerminal
from contracts.navigation import ExplorationState, ExploreAreaRequest, NavigationGoal, NavigationState
from contracts.perception import Observation
from contracts.robot_state import RobotState, SafetyState
from contracts.spatial_memory import (
    SceneSpace as SpatialSceneSpace,
    SemanticRegion as SpatialSemanticRegion,
    SpatialAnchor,
)
from typing import List, Literal, Mapping, Optional
from typing_extensions import Annotated


class DiagnosticLevel(StrEnum):
    """诊断等级。"""

    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class DiagnosticItem(TimestampedContract):
    """单条诊断项。"""

    component: str = Field(description="诊断组件。")
    level: DiagnosticLevel = Field(description="诊断等级。")
    message: str = Field(description="诊断说明。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class DiagnosticSnapshot(TimestampedContract):
    """一次状态采样对应的诊断快照。"""

    robot_name: str = Field(description="机器人名称。")
    items: List[DiagnosticItem] = Field(default_factory=list, description="诊断项列表。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class RobotStatusSnapshot(TimestampedContract):
    """机器人状态服务返回的聚合快照。"""

    robot_name: str = Field(description="机器人名称。")
    robot_model: str = Field(description="机器人型号。")
    assembly_status: MetadataDict = Field(default_factory=dict, description="装配状态。")
    robot_state: RobotState = Field(description="机器人状态。")
    safety_state: SafetyState = Field(description="安全状态。")
    diagnostics: List[DiagnosticItem] = Field(default_factory=list, description="诊断项列表。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加信息。")


class ObservationContext(TimestampedContract):
    """观察上下文。"""

    camera_id: str = Field(description="观察所属相机。")
    observation: Observation = Field(description="观察结果。")
    image_artifact: Optional[ArtifactRef] = Field(default=None, description="关联图像制品。")
    camera_info: Optional[CameraInfo] = Field(default=None, description="相机参数。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加上下文。")


class SceneObjectSummary(TimestampedContract):
    """单类目标的场景摘要。"""

    label: str = Field(description="目标标签。")
    count: int = Field(default=0, ge=0, description="当前检测到的数量。")
    tracked_count: int = Field(default=0, ge=0, description="当前处于可跟踪状态的数量。")
    max_score: float = Field(default=0.0, ge=0.0, le=1.0, description="当前最高置信度。")
    camera_ids: List[str] = Field(default_factory=list, description="出现过的相机标识。")
    track_ids: List[str] = Field(default_factory=list, description="关联轨迹标识。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")


class SceneSummary(TimestampedContract):
    """面向上层技能的场景语义摘要。"""

    headline: str = Field(description="一句话中文摘要。")
    details: List[str] = Field(default_factory=list, description="结构化中文要点。")
    objects: List[SceneObjectSummary] = Field(default_factory=list, description="按标签聚合的目标摘要。")
    detection_count: int = Field(default=0, ge=0, description="当前二维检测数量。")
    active_track_count: int = Field(default=0, ge=0, description="当前活跃轨迹数量。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加摘要元数据。")


class PerceptionContext(TimestampedContract):
    """感知处理上下文。"""

    camera_id: str = Field(description="感知所属相机。")
    observation: Observation = Field(description="标准观察结果。")
    scene_summary: SceneSummary = Field(description="结构化场景摘要。")
    image_artifact: Optional[ArtifactRef] = Field(default=None, description="关联图像制品。")
    camera_info: Optional[CameraInfo] = Field(default=None, description="相机参数。")
    pipeline_name: str = Field(description="处理管线名称。")
    detector_backend: str = Field(description="检测后端名称。")
    tracker_backend: str = Field(description="跟踪后端名称。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加上下文。")


class PerceptionRuntimeStatus(TimestampedContract):
    """持续视频感知运行时状态。"""

    enabled: bool = Field(default=False, description="当前是否启用持续感知功能。")
    auto_start: bool = Field(default=False, description="运行时启动时是否自动拉起。")
    running: bool = Field(default=False, description="后台感知线程是否正在运行。")
    camera_id: str = Field(default="front_camera", description="当前持续处理的相机标识。")
    interval_sec: float = Field(default=0.5, ge=0.05, description="连续抓帧周期。")
    detector_backend: str = Field(default="", description="当前持续感知使用的检测后端。")
    source_name: str = Field(default="", description="当前图像来源名称。")
    processed_frames: int = Field(default=0, ge=0, description="累计处理帧数。")
    failure_count: int = Field(default=0, ge=0, description="累计失败次数。")
    last_success_message: Optional[str] = Field(default=None, description="最近一次成功处理说明。")
    last_error: Optional[str] = Field(default=None, description="最近一次错误信息。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加状态元数据。")


class LocalizationSessionStatus(StrEnum):
    """平台定位会话状态。"""

    IDLE = "idle"
    LOCALIZING = "localizing"
    READY = "ready"
    STALE = "stale"
    FAILED = "failed"


class RelocalizationCandidate(TimestampedContract):
    """地图匹配重定位候选结果。"""

    backend: str = Field(default="unknown", description="产生候选的重定位后端。")
    pose: Optional[Pose] = Field(default=None, description="候选位姿。")
    fitness: Optional[float] = Field(default=None, description="匹配 fitness，未知时为空。")
    inlier_ratio: Optional[float] = Field(default=None, description="ICP 内点比例，未知时为空。")
    covariance: Optional[MetadataDict] = Field(default=None, description="协方差摘要，未知时为空。")
    asset_refs: List[MetadataDict] = Field(default_factory=list, description="候选所引用的地图几何资产。")
    failure_reason: Optional[str] = Field(default=None, description="候选失败原因。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加诊断元数据。")


# Assigned session descriptors are deliberately plain metadata. Reject objects
# before calling serializers/deepcopy: an accidental map/cloud owner must never
# be traversed by this boundary. No new identity hash, clock or size cap is used.
_ASSIGNED_SESSION_FORBIDDEN_KEYS = frozenset({
    "asset_manifest_sha256", "manifest_sha256", "content_sha256",
    "activation_identity_sha256", "payload_sha256", "snapshot_sha256",
    "asset_bundle_sha256", "activation_request_identity_sha256", "target_receipt_sha256",
    "activation_target_manifest_sha256", "algorithm_options_sha256", "bundle_manifest_sha256",
    "catalog_manifest_sha256", "cloud_source_sha256", "committed_occupancy_sha256",
    "committed_payload_sha256", "companion_sha256", "deadline_policy_sha256",
    "expected_asset_manifest_sha256", "expected_previous_manifest_sha256",
    "request_identity_sha256", "resolved_asset_manifest_sha256", "runtime_asset_manifest_sha256",
    "runtime_manifest_sha256", "source_policy_sha256", "target_manifest_sha256", "terminal_proof_sha256",
    "map_version_id", "platform_map_version_id", "latest_map_version_id",
    "localization_session_map_version_id", "seed_map_version_id",
    "mapping_session_id", "session_generation", "activation_receipt_v2",
    "relocalization_terminal_v2", "occupancy_grid", "cost_map", "cells",
    "cloud", "source_points", "xyz_f32", "checkpoint_payload",
    "candidate_payload_v3",
})


def _copy_assigned_session_descriptor(value):
    """Detach JSON-like descriptors without invoking arbitrary object hooks."""
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    if type(value) is dict:
        if any(type(key) is not str or key in _ASSIGNED_SESSION_FORBIDDEN_KEYS for key in value):
            raise ValueError("assigned session metadata contains legacy authority or payload fields")
        return {key: _copy_assigned_session_descriptor(item) for key, item in value.items()}
    if type(value) in (list, tuple):
        return [_copy_assigned_session_descriptor(item) for item in value]
    raise ValueError("assigned session metadata must contain only plain descriptors")


def _copy_assigned_localization_fields(value, model_type=RelocalizationCandidate):
    """Project known small fields before any serializer/deepcopy can see Any.

    Candidate metadata/covariance/asset refs and forged provider pose/tree fields
    may contain arbitrary models. Project the known geometry fields recursively
    and reject bodies directly before any generic serializer or deepcopy sees
    them. Final shape/value validation remains with the existing contracts.
    """
    if type(value) is model_type:
        raw = dict(object.__getattribute__(value, "__dict__"))
    elif type(value) is dict:
        raw = dict(value)
    else:
        raise ValueError("assigned localization fields require plain typed descriptors")
    if any(type(key) is not str or key not in model_type.model_fields for key in raw):
        raise ValueError("assigned localization descriptor contains unsupported fields")
    detached = {}
    for name, item in raw.items():
        if name == "timestamp" and type(item) is datetime:
            detached[name] = item  # immutable scalar, with no serialization hook
        elif model_type is RelocalizationCandidate and name == "pose" and item is not None:
            detached[name] = _copy_assigned_localization_fields(item, Pose)
        elif ((model_type is Pose and name == "position")
              or (model_type is Transform and name == "translation")):
            detached[name] = _copy_assigned_localization_fields(item, Vector3)
        elif ((model_type is Pose and name == "orientation")
              or (model_type is Transform and name == "rotation")):
            detached[name] = _copy_assigned_localization_fields(item, Quaternion)
        elif model_type is FrameTree and name == "transforms":
            if type(item) not in (list, tuple):
                raise ValueError("assigned frame tree transforms must be a plain sequence")
            detached[name] = [_copy_assigned_localization_fields(transform, Transform) for transform in item]
        else:
            detached[name] = _copy_assigned_session_descriptor(item)
    return detached


def _session_activation_owner(value):
    if type(value) is AssignedActivationReceipt:
        return value
    if type(value) is dict:
        return AssignedActivationReceipt.admit(value)
    raise ValueError("activation_receipt_v2 requires an admitted activation receipt")


def _session_terminal_owner(value):
    if type(value) is AssignedRelocalizationTerminal:
        return value
    if type(value) is dict:
        return AssignedRelocalizationTerminal.from_dict(value)
    raise ValueError("relocalization_terminal_v2 requires an admitted terminal")


_SessionActivationOwner = Annotated[
    AssignedActivationReceipt, PlainValidator(_session_activation_owner),
    PlainSerializer(lambda owner: owner.to_dict(), return_type=dict),
]
_SessionTerminalOwner = Annotated[
    AssignedRelocalizationTerminal, PlainValidator(_session_terminal_owner),
    PlainSerializer(lambda owner: owner.to_dict(), return_type=dict),
]


class LocalizationSession(TimestampedContract):
    """平台侧定位会话。"""

    identity_schema_version: Literal[1, 2] = 1
    asset_identity: Optional[AssetIdentityV2] = None
    activation_receipt_v2: Optional[_SessionActivationOwner] = None
    relocalization_terminal_v2: Optional[_SessionTerminalOwner] = None
    _assigned_owned: bool = PrivateAttr(default=False)

    session_id: str = Field(description="平台定位会话标识。")
    map_name: str = Field(description="当前绑定的地图名称。")
    asset_manifest_sha256: Optional[str] = Field(
        default=None,
        description="当前绑定地图资产清单的不可变 SHA-256 摘要。",
    )
    frame_id: Optional[str] = Field(default=None, description="当前定位会话目标坐标系。")
    status: LocalizationSessionStatus = Field(
        default=LocalizationSessionStatus.LOCALIZING,
        description="平台定位会话状态。",
    )
    source_name: str = Field(default="platform_localization_runtime", description="定位会话来源名称。")
    pose_available: bool = Field(default=False, description="当前是否已有可用于平台定位的位姿。")
    geometry_asset: Optional[MetadataDict] = Field(default=None, description="当前定位会话绑定的几何资产摘要。")
    relocalization_candidates: List[RelocalizationCandidate] = Field(default_factory=list, description="最近地图匹配重定位候选。")
    last_relocalization: Optional[RelocalizationCandidate] = Field(default=None, description="最近一次重定位结果。")
    last_localized_at: Optional[datetime] = Field(default=None, description="最近一次定位就绪时间。")
    last_error: Optional[str] = Field(default=None, description="最近一次定位失败或未就绪说明。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name")
    @classmethod
    def _validate_map_name(cls, value: str) -> str:
        return _validate_map_asset_name(value)

    @field_validator("asset_manifest_sha256")
    @classmethod
    def _validate_asset_manifest_sha256(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        resolved = str(value or "").strip().lower()
        if len(resolved) != 64 or any(
            character not in "0123456789abcdef" for character in resolved
        ):
            raise ValueError("asset_manifest_sha256 必须为 64 位十六进制字符串。")
        return resolved


    @model_validator(mode="before")
    @classmethod
    def _admit_session_generation(cls, value):
        if not isinstance(value, dict):
            return value
        schema = value.get("identity_schema_version", 1)
        if type(schema) is not int or schema not in (1, 2):
            raise ValueError("identity_schema_version must be exactly 1 or 2")
        if schema == 1:
            if any(value.get(key) is not None for key in (
                "asset_identity", "activation_receipt_v2", "relocalization_terminal_v2",
            )):
                raise ValueError("legacy sessions cannot carry assigned owners")
            return value
        value = dict(value)
        for key in ("metadata", "geometry_asset"):
            if key in value:
                value[key] = _copy_assigned_session_descriptor(value[key])
        # Candidates are UI summaries only. Their small public values are copied;
        # canonical terminal bytes stay in the admitted terminal, by reference.
        for key in ("relocalization_candidates", "last_relocalization"):
            if key in value:
                items = value[key] if key == "relocalization_candidates" else [value[key]]
                detached = [None if item is None else _copy_assigned_localization_fields(item) for item in items]
                value[key] = detached if key == "relocalization_candidates" else detached[0]
        identity = value.get("asset_identity")
        if type(identity) is AssetIdentityV2:
            # AssetIdentityV2 semantics remain unchanged; reject unvalidated
            # model_copy/model_construct values at this new admission boundary.
            AssetIdentityV2.model_validate({"asset_id": identity.asset_id})
        return value

    @model_validator(mode="after")
    def _seal_assigned_session(self):
        if self.identity_schema_version == 1:
            return self
        owner = self.activation_receipt_v2
        terminal = self.relocalization_terminal_v2
        if (type(self.asset_identity) is not AssetIdentityV2 or owner is None
                or self.asset_manifest_sha256 is not None
                or self.map_name != owner.map_name or self.asset_identity != owner.asset_identity):
            raise ValueError("assigned session requires the exact map, asset and activation owner without digest")
        if terminal is not None and (
            terminal.disposition not in (1, 2)
            or not terminal.request.activation_receipt.exact_replay_of(owner)
            or terminal.request.asset_identity != self.asset_identity
        ):
            raise ValueError("assigned session terminal must be applied for its exact activation owner")
        if self.status == LocalizationSessionStatus.READY and not self.pose_available:
            raise ValueError("assigned READY requires a usable pose")
        for descriptor in (self.metadata, self.geometry_asset):
            if descriptor and descriptor.get("map_name", self.map_name) != self.map_name:
                raise ValueError("assigned session descriptor map_name differs")
            if descriptor and "asset_identity" in descriptor and descriptor["asset_identity"] != {"asset_id": self.asset_identity.asset_id}:
                raise ValueError("assigned session descriptor asset identity differs")
        self.__pydantic_private__["_assigned_owned"] = True
        return self

    @model_serializer(mode="wrap")
    def _serialize_session_generation(self, handler):
        document = handler(self)
        if self.identity_schema_version == 1:
            for key in ("identity_schema_version", "asset_identity", "activation_receipt_v2", "relocalization_terminal_v2"):
                document.pop(key, None)
        return document

    def __setattr__(self, name, value):
        if getattr(self, "__pydantic_private__", None) and self.__pydantic_private__.get("_assigned_owned", False):
            raise TypeError("assigned sessions are sealed; construct a validated successor")
        if (self.__dict__.get("identity_schema_version") == 1
                and name in {"identity_schema_version", "asset_identity", "activation_receipt_v2", "relocalization_terminal_v2"}):
            # Pydantic after-validator failure can leave an assigned value in
            # place. Reject before its assignment machinery touches a legacy
            # object already retained by the active session/history/event queue.
            raise TypeError("legacy session owner fields require explicit assigned admission")
        return super().__setattr__(name, value)

    def __getattribute__(self, name):
        value = super().__getattribute__(name)
        if name in {"metadata", "geometry_asset", "relocalization_candidates", "last_relocalization"}:
            private = super().__getattribute__("__pydantic_private__")
            if private and private.get("_assigned_owned", False):
                return deepcopy(value)
        return value

    def model_copy(self, *, update: Optional[Mapping[str, object]] = None, deep: bool = False):
        changes = dict(update or {})
        if self.identity_schema_version == 1:
            if any(key in changes for key in ("identity_schema_version", "asset_identity", "activation_receipt_v2", "relocalization_terminal_v2")):
                schema = changes.get("identity_schema_version", 1)
                if type(schema) is not int or schema != 1 or any(changes.get(key) is not None for key in ("asset_identity", "activation_receipt_v2", "relocalization_terminal_v2")):
                    raise TypeError("legacy sessions require explicit assigned admission")
            return super().model_copy(update=update, deep=deep)
        owner_fields = {"identity_schema_version", "asset_identity", "activation_receipt_v2",
                        "relocalization_terminal_v2", "session_id", "map_name", "asset_manifest_sha256", "frame_id"}
        for name in owner_fields & changes.keys():
            if changes[name] != getattr(self, name):
                raise TypeError("assigned session owners cannot be rewritten by model_copy")
        # Preserve receipt references even for deep=True, and revalidate every
        # changed descriptor; generic Pydantic model_copy skips validation.
        values = {name: getattr(self, name) for name in type(self).model_fields}
        values.update(changes)
        return type(self).model_validate(values)

    def __deepcopy__(self, memo=None):
        if self.identity_schema_version != 2:
            return super().__deepcopy__(memo=memo)
        copied = self.model_copy()
        if memo is not None:
            memo[id(self)] = copied
        return copied


class TopologyNodeSummary(TimestampedContract):
    """平台导航语义层中的拓扑节点摘要。"""

    node_id: str = Field(description="拓扑节点标识。")
    label: str = Field(description="拓扑节点名称。")
    frame_id: str = Field(description="拓扑节点所在坐标系。")
    map_name: Optional[str] = Field(default=None, description="所属地图名称。")
    anchor_id: Optional[str] = Field(default=None, description="关联锚点标识。")
    pose: Optional[Pose] = Field(default=None, description="节点中心位姿。")
    aliases: List[str] = Field(default_factory=list, description="节点别名。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name")
    @classmethod
    def _validate_map_name(cls, value: Optional[str]) -> Optional[str]:
        return _validate_optional_map_asset_name(value)


class NavigationSemanticContext(TimestampedContract):
    """平台当前激活地图的语义导航上下文。"""

    map_name: Optional[str] = Field(default=None, description="当前地图名称。")
    localization_session_id: Optional[str] = Field(default=None, description="当前绑定的定位会话。")
    localization_ready: bool = Field(default=False, description="当前平台定位是否已就绪。")
    semantic_regions: List[SpatialSemanticRegion] = Field(default_factory=list, description="当前语义区域集合。")
    scene_spaces: List[SpatialSceneSpace] = Field(default_factory=list, description="当前场景空间集合。")
    topology_nodes: List[TopologyNodeSummary] = Field(default_factory=list, description="当前拓扑节点集合。")
    anchors: List[SpatialAnchor] = Field(default_factory=list, description="当前空间锚点集合。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name")
    @classmethod
    def _validate_map_name(cls, value: Optional[str]) -> Optional[str]:
        return _validate_optional_map_asset_name(value)


def _validate_map_asset_name(value: object) -> str:
    resolved = str(value or "").strip()
    if not resolved:
        raise ValueError("map_name 不能为空。")
    return resolved


def _validate_optional_map_asset_name(value: object) -> Optional[str]:
    if value is None or not str(value).strip():
        return None
    return _validate_map_asset_name(value)


class LocalizationSnapshot(TimestampedContract):
    """定位服务共享只读快照；修改内容时构造下一份快照。"""

    model_config = ConfigDict(frozen=True)

    source_name: str = Field(description="定位来源名称。")
    current_pose: Optional[Pose] = Field(default=None, description="当前位姿。")
    frame_tree: Optional[FrameTree] = Field(default=None, description="当前坐标树快照。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class MapSnapshot(TimestampedContract):
    """地图服务快照；发布后共享只读图层，修改内容须构造下一份快照。"""

    model_config = ConfigDict(frozen=True)

    map_name: str = Field(description="用户选择的地图名称。")
    source_name: str = Field(description="地图来源名称。")
    source_epoch: str = Field(description="来源生命周期内稳定、跨生命周期变化的作用域标识。")
    update_sequence: int = Field(ge=1, description="source_epoch 作用域内递增的更新序号。")
    schema_version: Literal[2] = Field(default=2, description="显式来源代际与修订身份，不包含内容摘要。")
    _source_owner_id: Optional[int] = PrivateAttr(default=None)
    frame_id: Optional[str] = Field(default=None, description="地图快照统一消费坐标系。")
    occupancy_grid: Optional[OccupancyGrid] = Field(default=None, description="占据栅格地图。")
    cost_map: Optional[CostMap] = Field(default=None, description="代价地图。")
    semantic_map: Optional[SemanticMap] = Field(default=None, description="语义地图。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_name", "source_name", "source_epoch")
    @classmethod
    def _validate_snapshot_identity_text(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("地图快照身份字段不能为空。")
        return resolved

    @model_validator(mode="before")
    @classmethod
    def _read_legacy_snapshot(cls, value):
        if isinstance(value, dict) and type(value.get("schema_version", 1)) is not int:
            raise ValueError("地图快照 schema_version 必须为整数。")
        if isinstance(value, dict) and value.get("schema_version", 1) == 1:
            # 旧摘要只是旧格式字段，不能重新计算或当作新修订凭据。
            value = dict(value)
            value.pop("content_sha256", None)
            value["schema_version"] = 2
            if isinstance(value.get("metadata"), dict):
                value["metadata"] = dict(value["metadata"])
                value["metadata"].pop("content_sha256", None)
        return value

    @model_validator(mode="after")
    def _validate_snapshot_identity(self) -> "MapSnapshot":
        """仅在构造边界核对图层；普通读取和元数据复制不遍历整张地图。"""

        occupancy_grid = self.occupancy_grid
        cost_map = self.cost_map
        semantic_map = self.semantic_map
        map_name = self.map_name
        frame_id = self.frame_id
        layers = tuple(
            layer
            for layer in (occupancy_grid, cost_map, semantic_map)
            if layer is not None
        )
        if not frame_id:
            for layer in layers:
                candidate = str(getattr(layer, "frame_id", "") or "").strip()
                if candidate:
                    frame_id = candidate
                    object.__setattr__(self, "frame_id", frame_id)
                    break
        for layer in layers:
            if str(layer.map_id or "").strip() != map_name:
                raise ValueError("地图快照图层 map_id 必须等于 map_name。")
            if frame_id and layer.frame_id != frame_id:
                raise ValueError("地图快照图层 frame_id 不一致。")
        return self


class NavigationContext(TimestampedContract):
    """导航服务上下文。"""

    current_goal: Optional[NavigationGoal] = Field(default=None, description="当前导航目标。")
    navigation_state: NavigationState = Field(description="导航状态。")
    backend_name: Optional[str] = Field(default=None, description="导航后端名称。")
    goal_reached: bool = Field(default=False, description="当前目标是否到达。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class ServiceHealthStatus(StrEnum):
    """Web 监控服务健康状态。"""

    AVAILABLE = "available"
    READY = "ready"
    IDLE = "idle"
    DEGRADED = "degraded"
    STALE = "stale"
    MISSING = "missing"
    ERROR = "error"
    UNSUPPORTED = "unsupported"
    DISABLED = "disabled"


class ServiceHealthEntry(ContractModel):
    """单个系统服务的健康展示条目。"""

    service_id: str = Field(description="稳定服务标识。")
    display_name: str = Field(description="前端展示用中文服务名。")
    group: str = Field(default="系统服务", description="服务分组。")
    status: ServiceHealthStatus = Field(default=ServiceHealthStatus.MISSING, description="服务健康状态。")
    summary: str = Field(default="", description="一句话状态摘要。")
    evidence: str = Field(default="", description="可展示证据。")
    source: str = Field(default="", description="状态来源。")
    updated_at: Optional[datetime] = Field(default=None, description="状态更新时间。")
    reason: Optional[str] = Field(default=None, description="机器可读原因。")
    advice: str = Field(default="", description="排障建议。")
    related_streams: List[str] = Field(default_factory=list, description="相关实时流 topic。")
    expected_process_roles: List[str] = Field(default_factory=list, description="期望关联的进程角色。")
    process_evidence: List[MetadataDict] = Field(default_factory=list, description="关联进程证据。")
    order: int = Field(default=0, description="前端排序权重。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class ServiceHealthSummary(TimestampedContract):
    """Web 监控服务健康聚合快照。"""

    status: ServiceHealthStatus = Field(default=ServiceHealthStatus.MISSING, description="整体服务健康状态。")
    authoritative: bool = Field(default=True, description="是否由后端权威健康聚合器生成。")
    generated_at: datetime = Field(default_factory=utc_now, description="生成时间。")
    entries: List[ServiceHealthEntry] = Field(default_factory=list, description="服务健康条目。")
    source: str = Field(default="gateway_runtime", description="聚合来源。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class RuntimeGovernanceSummary(TimestampedContract):
    """运行时观察与治理摘要。"""

    mapping_profile: MetadataDict = Field(default_factory=dict, description="建图 profile 摘要。")
    geometry_asset: MetadataDict = Field(default_factory=dict, description="当前地图几何资产摘要。")
    localization_session: MetadataDict = Field(default_factory=dict, description="当前定位会话摘要。")
    last_relocalization: MetadataDict = Field(default_factory=dict, description="最近一次地图匹配重定位摘要。")
    navigation_control_policy: MetadataDict = Field(default_factory=dict, description="导航控制策略摘要。")
    service_health: ServiceHealthSummary = Field(default_factory=ServiceHealthSummary, description="Web 监控服务健康聚合快照。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加治理元数据。")


class ExplorationContext(TimestampedContract):
    """探索服务上下文。"""

    current_request: Optional[ExploreAreaRequest] = Field(default=None, description="当前探索请求。")
    exploration_state: ExplorationState = Field(description="探索状态。")
    backend_name: Optional[str] = Field(default=None, description="探索后端名称。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")
