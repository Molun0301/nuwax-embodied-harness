from __future__ import annotations

from datetime import datetime

from compat import StrEnum

from pydantic import Field, field_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from contracts.geometry import Pose, Vector3
from contracts.naming import (
    validate_anchor_id,
    validate_frame_id,
    validate_instance_id,
    validate_navigation_candidate_id,
    validate_navigation_trajectory_event_id,
    validate_observation_event_id,
    validate_observation_id,
    validate_scene_space_id,
)
from typing import List, Optional


class SpatialAnchorKind(StrEnum):
    """空间锚点类型。"""

    PLACE_NODE = "place_node"
    TOPOLOGY_NODE = "topology_node"
    SEMANTIC_REGION = "semantic_region"
    OBJECT_INSTANCE = "object_instance"


class InstanceMovability(StrEnum):
    """实例可移动性分类。"""

    STATIC = "static"
    MOVABLE = "movable"
    TRANSIENT = "transient"


class InstanceLifecycleState(StrEnum):
    """实例生命周期状态。"""

    ACTIVE = "active"
    UNCERTAIN = "uncertain"
    STALE = "stale"
    REMOVED = "removed"


class InstanceDynamicState(StrEnum):
    """实例短时动态状态。"""

    STATIC = "static"
    DYNAMIC = "dynamic"
    QUASI_STATIC = "quasi_static"
    UNKNOWN = "unknown"


class InstanceObservationWindowSample(TimestampedContract):
    """实例短时观测窗口样本。"""

    pose: Pose = Field(description="观察到的对象位姿，统一使用 map frame。")
    observer_pose: Optional[Pose] = Field(
        default=None,
        description="观测时相机/机器人位姿(map frame)，区别于对象位姿，供射线三角化与延迟点云重投影使用。",
    )
    bbox_xyxy: List[float] = Field(default_factory=list, description="观察时二维包围盒。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="观察置信度。")
    track_id: Optional[str] = Field(default=None, description="观察关联的跟踪标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class DeferredProjectionInput(ContractModel):
    """延迟投影(射线三角化 / 点云重投影)所需的单次观测输入缓存。

    独立于 observation_window 滚动窗口，不被 8 样本上限驱逐，
    供 pending 实例在累积点云稠密后重投影时使用。
    """

    observer_pose: Pose = Field(description="观测时相机/机器人位姿(map frame)。")
    bbox_xyxy: List[float] = Field(default_factory=list, description="观测时二维包围盒。")
    track_id: Optional[str] = Field(default=None, description="观测关联的跟踪标识(三角化主键)。")
    camera_frame_id: Optional[str] = Field(default=None, description="相机坐标系标识。")
    fx: Optional[float] = Field(default=None, description="相机内参 fx。")
    fy: Optional[float] = Field(default=None, description="相机内参 fy。")
    cx: Optional[float] = Field(default=None, description="相机内参 cx。")
    cy: Optional[float] = Field(default=None, description="相机内参 cy。")
    image_width_px: Optional[int] = Field(default=None, description="图像宽度像素。")
    image_height_px: Optional[int] = Field(default=None, description="图像高度像素。")
    ray_origin_map: Optional[List[float]] = Field(default=None, description="射线光心(map frame, [x,y,z])。")
    ray_dir_map: Optional[List[float]] = Field(default=None, description="射线方向单位向量(map frame, [x,y,z])。")
    cone_half_angle_rad: Optional[float] = Field(
        default=None,
        description="bbox 角张成的视锥半角(弧度)，供延迟点云锥内取点。",
    )
    source_observation_id: Optional[str] = Field(default=None, description="来源观测事件标识。")
    timestamp: Optional[datetime] = Field(default=None, description="观测时间。")


class SpatialAnchor(TimestampedContract):
    """可长期引用的空间锚点。"""

    anchor_id: str = Field(description="锚点标识。")
    anchor_kind: SpatialAnchorKind = Field(description="锚点类型。")
    name: str = Field(description="锚点名称。")
    pose: Pose = Field(description="锚点位姿，统一使用 map frame。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域。")
    semantic_labels: List[str] = Field(default_factory=list, description="关联语义标签。")
    inspection_poses: List[Pose] = Field(default_factory=list, description="建议巡检位姿列表。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("anchor_id")
    @classmethod
    def _validate_anchor_id(cls, value: str) -> str:
        return validate_anchor_id(value)


class SemanticRegion(TimestampedContract):
    """面向记忆和导航消费的语义区域。"""

    region_id: str = Field(description="区域标识。")
    anchor_id: str = Field(description="区域锚点标识。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    frame_id: str = Field(description="区域坐标系，统一应为 map。")
    label: str = Field(description="区域主标签。")
    aliases: List[str] = Field(default_factory=list, description="区域别名。")
    centroid: Optional[Pose] = Field(default=None, description="区域中心位姿。")
    polygon_points: List[Vector3] = Field(default_factory=list, description="区域轮廓点集合。")
    topo_node_ids: List[str] = Field(default_factory=list, description="关联拓扑节点集合。")
    inspection_poses: List[Pose] = Field(default_factory=list, description="建议巡检位姿列表。")
    graph_relation_ids: List[str] = Field(default_factory=list, description="关联语义图关系集合。")
    summary_node_ids: List[str] = Field(default_factory=list, description="关联层级摘要节点集合。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")

    @field_validator("anchor_id")
    @classmethod
    def _validate_region_anchor_id(cls, value: str) -> str:
        return validate_anchor_id(value)


class SceneSpaceReviewStatus(StrEnum):
    """场景空间审核状态。"""

    CANDIDATE = "candidate"
    AUTO_LABELED = "auto_labeled"
    REVIEWED = "reviewed"
    CORRECTED = "corrected"
    REJECTED = "rejected"
    STALE = "stale"


class SemanticGraphRelationKind(StrEnum):
    """长期语义图关系类型。"""

    CONTAINS = "contains"
    NEARBY = "nearby"
    OBSERVED_FROM = "observed_from"
    REACHABLE_FROM = "reachable_from"
    SAME_INSTANCE = "same_instance"
    BLOCKED_BY = "blocked_by"
    FAILED_AT = "failed_at"


class NavigationTrajectoryEventKind(StrEnum):
    """导航历史轨迹事件类型。"""

    VISITED_POSE = "visited_pose"
    REACHABLE_POSE = "reachable_pose"
    FAILED_POSE = "failed_pose"


class SemanticSummaryKind(StrEnum):
    """长期语义图层级摘要类型。"""

    REGION = "region"
    OBJECT_CLUSTER = "object_cluster"
    TARGET_CATEGORY_CLUSTER = "target_category_cluster"


class SceneSpaceBoundary(TimestampedContract):
    """由感知样本聚合出的场景空间边界。"""

    frame_id: str = Field(description="边界所属坐标系。")
    centroid: Optional[Pose] = Field(default=None, description="边界中心位姿。")
    polygon_points: List[Vector3] = Field(default_factory=list, description="二维或三维边界轮廓点。")
    min_corner: Optional[Vector3] = Field(default=None, description="边界包围盒最小角点。")
    max_corner: Optional[Vector3] = Field(default=None, description="边界包围盒最大角点。")
    area_m2: Optional[float] = Field(default=None, ge=0.0, description="估计平面面积，单位平方米。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="边界聚合置信度。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("frame_id")
    @classmethod
    def _validate_boundary_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)


class SceneObservationSample(TimestampedContract):
    """支撑场景空间识别的一次观察样本摘要。"""

    sample_id: str = Field(description="场景空间内部样本标识。")
    source_observation_id: Optional[str] = Field(default=None, description="来源观察标识。")
    observation_event_id: Optional[str] = Field(default=None, description="关联观察事件标识。")
    camera_id: Optional[str] = Field(default=None, description="来源相机。")
    pose: Optional[Pose] = Field(default=None, description="采样时机器人或相机位姿。")
    artifact_ids: List[str] = Field(default_factory=list, description="关联图像或模型制品。")
    visual_labels: List[str] = Field(default_factory=list, description="样本中的视觉目标标签。")
    vision_tags: List[str] = Field(default_factory=list, description="样本中的多模态语义标签。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="样本对场景空间的支持置信度。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("source_observation_id")
    @classmethod
    def _validate_source_observation_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_observation_id(value)

    @field_validator("observation_event_id")
    @classmethod
    def _validate_sample_event_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_observation_event_id(value)


class SceneCandidateLabel(TimestampedContract):
    """模型自动产出的场景空间候选标签。"""

    label: str = Field(description="候选标签。")
    normalized_label: Optional[str] = Field(default=None, description="归一化候选标签。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="候选标签置信度。")
    source_model: Optional[str] = Field(default=None, description="产出该标签的模型名称。")
    evidence_observation_sample_ids: List[str] = Field(default_factory=list, description="支持该标签的样本标识。")
    evidence_observation_event_ids: List[str] = Field(default_factory=list, description="支持该标签的观察事件。")
    attributes: MetadataDict = Field(default_factory=dict, description="标签属性。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("evidence_observation_event_ids")
    @classmethod
    def _validate_label_event_ids(cls, value: List[str]) -> List[str]:
        return [validate_observation_event_id(item) for item in value]


class SceneModelSummary(TimestampedContract):
    """图文或多模态模型对场景空间的摘要。"""

    summary: str = Field(description="模型自动生成的场景空间摘要。")
    provider_name: Optional[str] = Field(default=None, description="模型服务提供方。")
    model_name: Optional[str] = Field(default=None, description="模型名称。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="摘要置信度。")
    evidence_observation_sample_ids: List[str] = Field(default_factory=list, description="支撑摘要的样本标识。")
    generated_at: Optional[datetime] = Field(default=None, description="模型摘要生成时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class SceneSpace(TimestampedContract):
    """面向语义导航和记忆检索的场景空间。"""

    scene_space_id: str = Field(description="场景空间标识。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    frame_id: str = Field(description="场景空间统一坐标系。")
    boundary: Optional[SceneSpaceBoundary] = Field(default=None, description="场景空间边界。")
    entry_poses: List[Pose] = Field(default_factory=list, description="建议进入该空间的位姿列表。")
    inspection_poses: List[Pose] = Field(default_factory=list, description="建议巡检该空间的位姿列表。")
    observation_sample_ids: List[str] = Field(default_factory=list, description="支持该空间的观察样本标识。")
    observation_samples: List[SceneObservationSample] = Field(default_factory=list, description="支持该空间的观察样本摘要。")
    model_summary: Optional[SceneModelSummary] = Field(default=None, description="模型自动生成的空间摘要。")
    candidate_labels: List[SceneCandidateLabel] = Field(default_factory=list, description="模型自动生成的候选标签。")
    primary_label: Optional[str] = Field(default=None, description="当前采用的空间主标签。")
    human_aliases: List[str] = Field(default_factory=list, description="人工维护的别名集合。")
    review_status: SceneSpaceReviewStatus = Field(
        default=SceneSpaceReviewStatus.CANDIDATE,
        description="场景空间人工审核状态。",
    )
    semantic_updated_at: Optional[datetime] = Field(default=None, description="语义信息最近更新时间。")
    linked_anchor_ids: List[str] = Field(default_factory=list, description="关联空间锚点集合。")
    linked_object_instance_ids: List[str] = Field(default_factory=list, description="关联语义对象实例集合。")
    linked_topology_node_ids: List[str] = Field(default_factory=list, description="关联拓扑节点集合。")
    linked_semantic_region_ids: List[str] = Field(default_factory=list, description="关联语义区域集合。")
    linked_observation_event_ids: List[str] = Field(default_factory=list, description="关联观察事件集合。")
    graph_relation_ids: List[str] = Field(default_factory=list, description="关联语义图关系集合。")
    summary_node_ids: List[str] = Field(default_factory=list, description="关联层级摘要节点集合。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("scene_space_id")
    @classmethod
    def _validate_scene_space_id(cls, value: str) -> str:
        return validate_scene_space_id(value)

    @field_validator("frame_id")
    @classmethod
    def _validate_scene_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @field_validator("linked_anchor_ids")
    @classmethod
    def _validate_linked_anchor_ids(cls, value: List[str]) -> List[str]:
        return [validate_anchor_id(item) for item in value]

    @field_validator("linked_object_instance_ids")
    @classmethod
    def _validate_linked_object_instance_ids(cls, value: List[str]) -> List[str]:
        return [validate_instance_id(item) for item in value]

    @field_validator("linked_observation_event_ids")
    @classmethod
    def _validate_linked_observation_event_ids(cls, value: List[str]) -> List[str]:
        return [validate_observation_event_id(item) for item in value]


class SemanticInstance(TimestampedContract):
    """投影并关联后的语义实例，代表世界中的持续对象实体。"""

    instance_id: str = Field(description="实例标识。")
    anchor_id: str = Field(description="实例空间锚点标识。")
    label: str = Field(description="实例主标签。")
    display_name: Optional[str] = Field(default=None, description="展示名称。")
    pose: Pose = Field(description="实例位姿，统一使用 map frame。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间。")
    instance_type: str = Field(default="object", description="实例类型。")
    movability: InstanceMovability = Field(default=InstanceMovability.MOVABLE, description="可移动性分类。")
    dynamic_state: InstanceDynamicState = Field(
        default=InstanceDynamicState.UNKNOWN,
        description="短时观测窗口推断出的动态状态。",
    )
    observation_window: List[InstanceObservationWindowSample] = Field(
        default_factory=list,
        description="用于动态实体隔离的短时观测窗口。",
    )
    deferred_projection_cache: List[DeferredProjectionInput] = Field(
        default_factory=list,
        description="延迟投影(射线三角化/点云重投影)输入缓存，独立于 observation_window，不被 8 样本上限驱逐。",
    )
    lifecycle_state: InstanceLifecycleState = Field(
        default=InstanceLifecycleState.ACTIVE,
        description="生命周期状态。",
    )
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="当前综合置信度。")
    observation_count: int = Field(default=0, ge=0, description="累计观察次数。")
    first_seen_ts: datetime = Field(description="首次观察时间。")
    last_seen_ts: datetime = Field(description="最近观察时间。")
    last_observation_event_id: Optional[str] = Field(default=None, description="最近关联的观察事件。")
    supporting_observation_event_ids: List[str] = Field(default_factory=list, description="支持该实例的观察事件。")
    artifact_ids: List[str] = Field(default_factory=list, description="关联制品。")
    semantic_labels: List[str] = Field(default_factory=list, description="语义标签。")
    visual_labels: List[str] = Field(default_factory=list, description="视觉目标标签。")
    vision_tags: List[str] = Field(default_factory=list, description="视觉语义标签。")
    inspection_poses: List[Pose] = Field(default_factory=list, description="建议巡检位姿列表。")
    graph_relation_ids: List[str] = Field(default_factory=list, description="关联语义图关系集合。")
    summary_node_ids: List[str] = Field(default_factory=list, description="关联层级摘要节点集合。")
    attributes: MetadataDict = Field(default_factory=dict, description="附加属性。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("instance_id")
    @classmethod
    def _validate_instance_id(cls, value: str) -> str:
        return validate_instance_id(value)

    @field_validator("anchor_id")
    @classmethod
    def _validate_instance_anchor_id(cls, value: str) -> str:
        return validate_anchor_id(value)

    @field_validator("last_observation_event_id")
    @classmethod
    def _validate_last_event_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_observation_event_id(value)


class ObservationEvent(TimestampedContract):
    """一次带空间锚定的观察事件，代表不可回写的事实层记录。"""

    event_id: str = Field(description="观察事件标识。")
    title: str = Field(description="事件标题。")
    summary: str = Field(description="事件摘要。")
    camera_id: Optional[str] = Field(default=None, description="来源相机。")
    pose: Optional[Pose] = Field(default=None, description="事件发生位姿。")
    frame_id: Optional[str] = Field(default=None, description="事件证据坐标系。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间。")
    anchor_id: Optional[str] = Field(default=None, description="关联锚点。")
    source_observation_id: Optional[str] = Field(default=None, description="来源观察标识。")
    source_memory_id: Optional[str] = Field(default=None, description="来源记忆标识。")
    linked_instance_ids: List[str] = Field(default_factory=list, description="关联实例集合。")
    artifact_ids: List[str] = Field(default_factory=list, description="关联制品。")
    graph_relation_ids: List[str] = Field(default_factory=list, description="关联语义图关系集合。")
    semantic_labels: List[str] = Field(default_factory=list, description="语义标签。")
    visual_labels: List[str] = Field(default_factory=list, description="视觉目标标签。")
    vision_tags: List[str] = Field(default_factory=list, description="视觉语义标签。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("event_id")
    @classmethod
    def _validate_event_id(cls, value: str) -> str:
        return validate_observation_event_id(value)

    @field_validator("anchor_id")
    @classmethod
    def _validate_event_anchor_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_anchor_id(value)

    @field_validator("frame_id")
    @classmethod
    def _validate_event_frame_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_frame_id(value)


class NavigationCandidate(TimestampedContract):
    """面向导航层输出的空间候选。"""

    candidate_id: str = Field(description="导航候选标识。")
    anchor_id: str = Field(description="关联锚点标识。")
    source_collection: str = Field(description="来源集合，例如 place_nodes、object_instances。")
    record_id: str = Field(description="来源记录标识。")
    target_name: str = Field(description="导航目标名称。")
    inspection_pose: Pose = Field(description="建议用于导航的巡检位姿。")
    anchor_pose: Optional[Pose] = Field(default=None, description="锚点中心位姿。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    topo_node_id: Optional[str] = Field(default=None, description="关联拓扑节点。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间。")
    distance_m: Optional[float] = Field(default=None, ge=0.0, description="相对当前位姿的距离。")
    score: float = Field(default=0.0, ge=0.0, le=1.0, description="综合候选分数。")
    verification_query: Optional[str] = Field(default=None, description="建议到点后复核的文本。")
    verification_artifact_id: Optional[str] = Field(default=None, description="复核参考制品。")
    verification_event_id: Optional[str] = Field(default=None, description="复核参考观察事件。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("candidate_id")
    @classmethod
    def _validate_candidate_id(cls, value: str) -> str:
        return validate_navigation_candidate_id(value)

    @field_validator("anchor_id")
    @classmethod
    def _validate_candidate_anchor_id(cls, value: str) -> str:
        return validate_anchor_id(value)

    @field_validator("verification_event_id")
    @classmethod
    def _validate_candidate_event_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_observation_event_id(value)


class NavigationTrajectoryEvent(TimestampedContract):
    """平台侧 ego trajectory（自我轨迹）节点。"""

    trajectory_event_id: str = Field(description="轨迹事件标识。")
    event_kind: NavigationTrajectoryEventKind = Field(description="轨迹事件类型。")
    pose: Optional[Pose] = Field(default=None, description="机器人历史位姿。")
    target_pose: Optional[Pose] = Field(default=None, description="关联候选目标位姿。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    navigation_candidate_id: Optional[str] = Field(default=None, description="关联导航候选标识。")
    source_record_kind: Optional[str] = Field(default=None, description="关联候选来源记录类型。")
    source_record_id: Optional[str] = Field(default=None, description="关联候选来源记录标识。")
    target_name: Optional[str] = Field(default=None, description="候选目标名称。")
    success: Optional[bool] = Field(default=None, description="执行或复核是否成功。")
    reason_code: Optional[str] = Field(default=None, description="成功或失败原因码。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("trajectory_event_id")
    @classmethod
    def _validate_trajectory_event_id(cls, value: str) -> str:
        return validate_navigation_trajectory_event_id(value)

    @field_validator("navigation_candidate_id")
    @classmethod
    def _validate_trajectory_candidate_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_navigation_candidate_id(value)


class SemanticGraphRelation(TimestampedContract):
    """长期语义图关系边。"""

    relation_id: str = Field(description="关系标识。")
    relation_kind: SemanticGraphRelationKind = Field(description="关系类型。")
    source_node_kind: str = Field(description="源节点类型。")
    source_node_id: str = Field(description="源节点标识。")
    target_node_kind: str = Field(description="目标节点类型。")
    target_node_id: str = Field(description="目标节点标识。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    confidence: float = Field(default=1.0, ge=0.0, le=1.0, description="关系置信度。")
    evidence_observation_event_ids: List[str] = Field(default_factory=list, description="支撑观察事件集合。")
    evidence_trajectory_event_ids: List[str] = Field(default_factory=list, description="支撑轨迹事件集合。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("evidence_observation_event_ids")
    @classmethod
    def _validate_relation_event_ids(cls, value: List[str]) -> List[str]:
        return [validate_observation_event_id(item) for item in value]


class SemanticSummaryNode(TimestampedContract):
    """长期语义图层级摘要节点。"""

    summary_id: str = Field(description="摘要节点标识。")
    summary_kind: SemanticSummaryKind = Field(description="摘要类型。")
    title: str = Field(description="摘要标题。")
    summary: str = Field(description="结构化摘要文本。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="关联定位会话。")
    scene_space_id: Optional[str] = Field(default=None, description="关联场景空间。")
    semantic_region_id: Optional[str] = Field(default=None, description="关联语义区域。")
    label: Optional[str] = Field(default=None, description="聚合目标类别或区域标签。")
    member_node_ids: List[str] = Field(default_factory=list, description="被摘要成员节点集合。")
    relation_ids: List[str] = Field(default_factory=list, description="关联关系集合。")
    evidence_observation_event_ids: List[str] = Field(default_factory=list, description="支撑观察事件集合。")
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="摘要置信度。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("scene_space_id")
    @classmethod
    def _validate_summary_scene_space_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_scene_space_id(value)

    @field_validator("evidence_observation_event_ids")
    @classmethod
    def _validate_summary_event_ids(cls, value: List[str]) -> List[str]:
        return [validate_observation_event_id(item) for item in value]


class VerificationResult(TimestampedContract):
    """导航到点后的复核结果。"""

    anchor_id: Optional[str] = Field(default=None, description="复核目标锚点。")
    verified: bool = Field(description="是否通过复核。")
    score: float = Field(ge=0.0, le=1.0, description="综合复核分数。")
    reason: Optional[str] = Field(default=None, description="复核原因说明。")
    matched_labels: List[str] = Field(default_factory=list, description="当前场景命中的标签。")
    matched_instance_ids: List[str] = Field(default_factory=list, description="命中的实例标识。")
    matched_observation_event_ids: List[str] = Field(default_factory=list, description="命中的观察事件。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("anchor_id")
    @classmethod
    def _validate_verification_anchor_id(cls, value: Optional[str]) -> Optional[str]:
        if not value:
            return value
        return validate_anchor_id(value)


class GroundingQueryPlan(ContractModel):
    """自然语言 grounding 规划结果。"""

    raw_query: str = Field(description="原始查询文本。")
    normalized_query: str = Field(description="归一化查询文本。")
    intent: str = Field(description="解析出的意图。")
    target_class: Optional[str] = Field(default=None, description="目标类别。")
    attributes: List[str] = Field(default_factory=list, description="属性词集合。")
    spatial_hint: Optional[str] = Field(default=None, description="空间提示。")
    temporal_hint: Optional[str] = Field(default=None, description="时间提示。")
    preferred_collections: List[str] = Field(default_factory=list, description="优先检索的集合。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")
