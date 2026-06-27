from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import Field, field_validator, model_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from contracts.geometry import Pose
from contracts.naming import validate_location_id, validate_memory_id, validate_scene_space_id
from contracts.spatial_memory import (
    NavigationCandidate,
    ObservationEvent,
    SceneSpace,
    SceneSpaceReviewStatus,
    SemanticInstance,
)
from datetime import datetime
from typing import List, Optional


class MemoryRecordKind(StrEnum):
    """记忆记录类型。"""

    TAGGED_LOCATION = "tagged_location"
    SCENE = "scene"
    NOTE = "note"
    OBJECT_INSTANCE = "object_instance"
    OBSERVATION_EVENT = "observation_event"
    SCENE_SPACE = "scene_space"
    NAVIGATION_TRAJECTORY = "navigation_trajectory"
    SEMANTIC_GRAPH_RELATION = "semantic_graph_relation"
    SEMANTIC_SUMMARY = "semantic_summary"


class TaggedLocation(TimestampedContract):
    """命名地点记录。"""

    location_id: str = Field(description="地点记录标识。")
    name: str = Field(description="地点名称。")
    normalized_name: str = Field(description="归一化名称。")
    aliases: List[str] = Field(default_factory=list, description="地点别名列表。")
    pose: Pose = Field(description="记录时的机器人位姿。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    localization_source_name: Optional[str] = Field(default=None, description="定位来源名称。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点标识。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域标识。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间标识。")
    semantic_labels: List[str] = Field(default_factory=list, description="关联语义标签。")
    observation_id: Optional[str] = Field(default=None, description="关联观察标识。")
    perception_headline: Optional[str] = Field(default=None, description="关联感知摘要标题。")
    memory_id: Optional[str] = Field(default=None, description="关联语义记忆标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("location_id")
    @classmethod
    def _validate_location_id(cls, value: str) -> str:
        return validate_location_id(value)

    @field_validator("memory_id")
    @classmethod
    def _validate_memory_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_memory_id(value)

    @field_validator("scene_space_id")
    @classmethod
    def _validate_scene_space_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_scene_space_id(value)


class SemanticMemoryEntry(TimestampedContract):
    """面向检索与上层消费的摘要记忆条目。"""

    memory_id: str = Field(description="语义记忆标识。")
    kind: MemoryRecordKind = Field(default=MemoryRecordKind.SCENE, description="记录类型。")
    title: str = Field(description="中文标题。")
    summary: str = Field(description="中文摘要。")
    tags: List[str] = Field(default_factory=list, description="检索标签。")
    linked_location_id: Optional[str] = Field(default=None, description="关联地点标识。")
    pose: Optional[Pose] = Field(default=None, description="关联位姿。")
    last_observed_at: Optional[datetime] = Field(
        default=None,
        description=(
            "最近一次观测时间（UTC）；机器人通过 navigate_to_pose 到达物品坐标 1m 内时刷新。"
            "spec 记忆库与地图绑定 §到达物品坐标 场景依赖此字段。"
        ),
    )
    observation_count: int = Field(default=0, ge=0, description="累计观测次数。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点标识。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域标识。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间标识。")
    semantic_labels: List[str] = Field(default_factory=list, description="关联语义标签。")
    observation_id: Optional[str] = Field(default=None, description="关联观察标识。")
    perception_headline: Optional[str] = Field(default=None, description="关联感知摘要标题。")
    artifact_ids: List[str] = Field(default_factory=list, description="关联制品标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("memory_id")
    @classmethod
    def _validate_memory_id(cls, value: str) -> str:
        return validate_memory_id(value)

    @field_validator("linked_location_id")
    @classmethod
    def _validate_linked_location_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_location_id(value)

    @field_validator("scene_space_id")
    @classmethod
    def _validate_entry_scene_space_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_scene_space_id(value)


class MemoryQueryMatch(TimestampedContract):
    """记忆查询匹配结果。"""

    record_kind: MemoryRecordKind = Field(description="命中的记录类型。")
    record_id: str = Field(description="命中的记录标识。")
    score: float = Field(ge=0.0, le=1.0, description="相似度分数。")
    reason: Optional[str] = Field(default=None, description="命中原因说明。")
    tagged_location: Optional[TaggedLocation] = Field(default=None, description="地点记录。")
    semantic_memory: Optional[SemanticMemoryEntry] = Field(default=None, description="语义记忆记录。")
    semantic_instance: Optional[SemanticInstance] = Field(default=None, description="语义实例记录。")
    observation_event: Optional[ObservationEvent] = Field(default=None, description="观察事件记录。")
    scene_space: Optional[SceneSpace] = Field(default=None, description="场景空间记录。")
    navigation_candidate: Optional[NavigationCandidate] = Field(
        default=None,
        description="可直接交给导航层的候选目标。",
    )
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="after")
    def _validate_payload(self) -> "MemoryQueryMatch":
        has_location = self.tagged_location is not None
        has_memory = self.semantic_memory is not None
        has_instance = self.semantic_instance is not None
        has_event = self.observation_event is not None
        has_scene_space = self.scene_space is not None
        payload_count = sum(
            1 for item in (has_location, has_memory, has_instance, has_event, has_scene_space) if item
        )
        if payload_count != 1:
            raise ValueError("MemoryQueryMatch 必须且只能包含一种记录载荷。")
        if has_location and self.record_kind != MemoryRecordKind.TAGGED_LOCATION:
            raise ValueError("TaggedLocation 结果的 record_kind 必须为 tagged_location。")
        if has_memory and self.record_kind in {
            MemoryRecordKind.TAGGED_LOCATION,
            MemoryRecordKind.OBJECT_INSTANCE,
            MemoryRecordKind.OBSERVATION_EVENT,
            MemoryRecordKind.SCENE_SPACE,
        }:
            raise ValueError("语义记忆结果不能使用当前 record_kind。")
        if has_instance and self.record_kind != MemoryRecordKind.OBJECT_INSTANCE:
            raise ValueError("语义实例结果的 record_kind 必须为 object_instance。")
        if has_event and self.record_kind != MemoryRecordKind.OBSERVATION_EVENT:
            raise ValueError("观察事件结果的 record_kind 必须为 observation_event。")
        if has_scene_space and self.record_kind != MemoryRecordKind.SCENE_SPACE:
            raise ValueError("场景空间结果的 record_kind 必须为 scene_space。")
        return self


class MemoryQueryResult(TimestampedContract):
    """记忆查询结果集合。"""

    query: str = Field(description="原始查询文本。")
    similarity_threshold: float = Field(ge=0.0, le=1.0, description="使用的相似度阈值。")
    matches: List[MemoryQueryMatch] = Field(default_factory=list, description="按分数排序的命中项。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class GoalSpatialRelation(ContractModel):
    """目标解析出的空间关系。"""

    relation: str = Field(description="空间关系，例如 near、left_of、inside。")
    text: str = Field(default="", description="原始关系文本。")
    anchor_text: Optional[str] = Field(default=None, description="关系锚点文本。")
    anchor_kind: Optional[str] = Field(default=None, description="关系锚点类别。")
    direction: Optional[str] = Field(default=None, description="方向提示。")
    distance_m: Optional[float] = Field(default=None, ge=0.0, description="显式距离约束。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="关系解析置信度。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class GoalConstraint(ContractModel):
    """目标解析出的约束。"""

    name: str = Field(description="约束名称。")
    kind: str = Field(default="attribute", description="约束类别，例如 attribute、temporal、negative。")
    value: Optional[str] = Field(default=None, description="约束值。")
    operator: str = Field(default="match", description="约束操作符。")
    hard_filter: bool = Field(default=False, description="是否作为硬过滤条件。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="约束解析置信度。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class GoalSpec(ContractModel):
    """自然语言目标的结构化平台表示。"""

    raw_query: str = Field(description="原始用户文本。")
    normalized_query: str = Field(description="归一化查询文本。")
    expected_action: str = Field(default="search", description="期望动作，例如 search、navigate、verify。")
    target_category: Optional[str] = Field(default=None, description="目标类别。")
    target_kind: Optional[str] = Field(default=None, description="目标类型，例如 object、place、mixed。")
    attributes: List[GoalConstraint] = Field(default_factory=list, description="属性约束。")
    spatial_relations: List[GoalSpatialRelation] = Field(default_factory=list, description="空间关系约束。")
    constraints: List[GoalConstraint] = Field(default_factory=list, description="通用约束。")
    forbidden_zones: List[GoalConstraint] = Field(default_factory=list, description="禁止区域或否定约束。")
    ambiguity_hints: List[str] = Field(default_factory=list, description="需要澄清的歧义提示。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class GoalCandidateScoreBreakdown(ContractModel):
    """导航候选可解释评分分解。"""

    semantic_score: float = Field(default=0.0, ge=0.0, le=1.0, description="语义匹配分。")
    spatial_score: float = Field(default=0.0, ge=0.0, le=1.0, description="空间关系分。")
    reachability_score: float = Field(default=0.0, ge=0.0, le=1.0, description="可达性分。")
    freshness_score: float = Field(default=0.0, ge=0.0, le=1.0, description="新鲜度分。")
    verification_score: float = Field(default=0.0, ge=0.0, le=1.0, description="复核证据分。")
    failure_penalty: float = Field(default=0.0, ge=0.0, le=1.0, description="失败历史惩罚。")
    safety_score: float = Field(default=0.0, ge=0.0, le=1.0, description="安全分。")
    total_score: float = Field(default=0.0, ge=0.0, le=1.0, description="综合排序分。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class NavigationCandidateResolution(ContractModel):
    """候选解析批次摘要。"""

    goal_spec: GoalSpec = Field(description="结构化目标。")
    returned_candidate_count: int = Field(default=0, ge=0, description="返回候选数量。")
    direct_candidate_count: int = Field(default=0, ge=0, description="可直接导航候选数量。")
    top_decision: str = Field(default="reject", description="建议动作：navigate、clarify、explore、reject。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="解析批次置信度。")
    reasons: List[str] = Field(default_factory=list, description="解析原因。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class MemoryPayloadFilter(ContractModel):
    """记忆检索过滤条件。"""

    record_kinds: List[MemoryRecordKind] = Field(default_factory=list, description="限制检索的记录类型。")
    map_version_id: Optional[str] = Field(default=None, description="限定地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="限定定位会话。")
    linked_location_id: Optional[str] = Field(default=None, description="限定关联地点标识。")
    camera_ids: List[str] = Field(default_factory=list, description="限定相机标识列表。")
    semantic_labels_any: List[str] = Field(default_factory=list, description="要求命中任一语义标签。")
    visual_labels_any: List[str] = Field(default_factory=list, description="要求命中任一视觉目标标签。")
    vision_tags_any: List[str] = Field(default_factory=list, description="要求命中任一视觉语义标签。")
    topo_node_ids: List[str] = Field(default_factory=list, description="限定拓扑节点列表。")
    anchor_ids: List[str] = Field(default_factory=list, description="限定空间锚点列表。")
    scene_space_ids: List[str] = Field(default_factory=list, description="限定场景空间列表。")
    review_statuses: List[SceneSpaceReviewStatus] = Field(default_factory=list, description="限定场景空间审核状态。")
    instance_types: List[str] = Field(default_factory=list, description="限定实例类型列表。")
    movabilities: List[str] = Field(default_factory=list, description="限定实例可移动性列表。")
    created_after: Optional[datetime] = Field(default=None, description="仅保留指定时间之后的记录。")
    created_before: Optional[datetime] = Field(default=None, description="仅保留指定时间之前的记录。")
    max_age_sec: Optional[float] = Field(default=None, ge=0.0, description="仅保留最近若干秒内的记录。")
    last_seen_after: Optional[datetime] = Field(default=None, description="仅保留指定时间之后仍见到的记录。")
    last_seen_before: Optional[datetime] = Field(default=None, description="仅保留指定时间之前最后见到的记录。")
    near_pose: Optional[Pose] = Field(default=None, description="限定靠近指定位姿。")
    max_distance_m: Optional[float] = Field(default=None, ge=0.0, description="与 near_pose 的最大距离。")

    @field_validator("scene_space_ids")
    @classmethod
    def _validate_filter_scene_space_ids(cls, value: List[str]) -> List[str]:
        return [validate_scene_space_id(item) for item in value]


class MemoryNavigationCandidate(TimestampedContract):
    """记忆检索生成的导航候选。"""

    record_kind: MemoryRecordKind = Field(description="来源记录类型。")
    record_id: str = Field(description="来源记录标识。")
    target_pose: Pose = Field(description="推荐导航目标位姿。")
    target_name: str = Field(description="候选目标名称。")
    linked_location_id: Optional[str] = Field(default=None, description="关联地点标识。")
    map_version_id: Optional[str] = Field(default=None, description="候选所在地图版本。")
    topo_node_id: Optional[str] = Field(default=None, description="候选拓扑节点标识。")
    semantic_region_id: Optional[str] = Field(default=None, description="候选语义区域标识。")
    scene_space_id: Optional[str] = Field(default=None, description="候选场景空间标识。")
    distance_m: Optional[float] = Field(default=None, ge=0.0, description="相对当前位姿的距离。")
    verification_query: Optional[str] = Field(default=None, description="到点后建议复核的文本查询。")
    verification_artifact_id: Optional[str] = Field(default=None, description="到点复核参考制品。")
    verification_memory_id: Optional[str] = Field(default=None, description="到点复核参考记忆。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加导航候选信息。")

    @field_validator("record_id")
    @classmethod
    def _validate_navigation_record_id(cls, value: str) -> str:
        if value.startswith("loc_"):
            return validate_location_id(value)
        if value.startswith("mem_"):
            return validate_memory_id(value)
        if value.startswith("ssp_"):
            return validate_scene_space_id(value)
        return value


    @field_validator("scene_space_id")
    @classmethod
    def _validate_navigation_scene_space_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_scene_space_id(value)


class MemoryArrivalVerification(TimestampedContract):
    """导航到点后的复核结果。"""

    query: str = Field(description="用于复核的目标查询。")
    verified: bool = Field(description="是否通过复核。")
    score: float = Field(ge=0.0, le=1.0, description="综合复核分数。")
    matched_labels: List[str] = Field(default_factory=list, description="当前场景命中的标签。")
    matched_memory_id: Optional[str] = Field(default=None, description="所复核的目标记忆标识。")
    reason: Optional[str] = Field(default=None, description="复核原因说明。")
    goal_id: Optional[str] = Field(default=None, description="关联导航目标标识。")
    candidate_id: Optional[str] = Field(default=None, description="关联导航候选标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加复核信息。")


class MemoryStoreSummary(TimestampedContract):
    """记忆服务摘要。"""

    tagged_location_count: int = Field(default=0, ge=0, description="地点记录数量。")
    semantic_memory_count: int = Field(default=0, ge=0, description="语义记忆数量。")
    semantic_instance_count: int = Field(default=0, ge=0, description="语义实例数量。")
    observation_event_count: int = Field(default=0, ge=0, description="观察事件数量。")
    scene_space_count: int = Field(default=0, ge=0, description="场景空间数量。")
    navigation_trajectory_count: int = Field(default=0, ge=0, description="导航轨迹事件数量。")
    semantic_graph_relation_count: int = Field(default=0, ge=0, description="语义图关系数量。")
    semantic_summary_count: int = Field(default=0, ge=0, description="语义摘要节点数量。")
    last_location_id: Optional[str] = Field(default=None, description="最近地点标识。")
    last_memory_id: Optional[str] = Field(default=None, description="最近语义记忆标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


MemoryQueryMatch.model_rebuild()
