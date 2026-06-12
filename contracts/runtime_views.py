from __future__ import annotations

from contracts.compat import StrEnum
from datetime import datetime

from pydantic import Field, model_validator

from contracts.artifacts import ArtifactRef
from contracts.base import MetadataDict, TimestampedContract
from contracts.geometry import FrameTree, Pose
from contracts.image import CameraInfo
from contracts.maps import CostMap, OccupancyGrid, SemanticMap
from contracts.navigation import ExplorationState, ExploreAreaRequest, NavigationGoal, NavigationState
from contracts.perception import Observation
from contracts.robot_state import RobotState, SafetyState
from contracts.spatial_memory import (
    SceneSpace as SpatialSceneSpace,
    SemanticRegion as SpatialSemanticRegion,
    SpatialAnchor,
)
from typing import List, Optional


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
    """RTAB-Map/ICP 重定位候选结果。"""

    backend: str = Field(default="unknown", description="产生候选的重定位后端。")
    pose: Optional[Pose] = Field(default=None, description="候选位姿。")
    fitness: Optional[float] = Field(default=None, description="匹配 fitness，未知时为空。")
    inlier_ratio: Optional[float] = Field(default=None, description="ICP 内点比例，未知时为空。")
    covariance: Optional[MetadataDict] = Field(default=None, description="协方差摘要，未知时为空。")
    asset_refs: List[MetadataDict] = Field(default_factory=list, description="候选所引用的地图几何资产。")
    failure_reason: Optional[str] = Field(default=None, description="候选失败原因。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加诊断元数据。")


class LocalizationSession(TimestampedContract):
    """平台侧定位会话。"""

    session_id: str = Field(description="平台定位会话标识。")
    map_name: str = Field(description="当前绑定的地图名称。")
    map_version_id: str = Field(description="当前绑定的地图版本标识。")
    frame_id: Optional[str] = Field(default=None, description="当前定位会话目标坐标系。")
    status: LocalizationSessionStatus = Field(
        default=LocalizationSessionStatus.LOCALIZING,
        description="平台定位会话状态。",
    )
    source_name: str = Field(default="platform_localization_runtime", description="定位会话来源名称。")
    pose_available: bool = Field(default=False, description="当前是否已有可用于平台定位的位姿。")
    geometry_asset: Optional[MetadataDict] = Field(default=None, description="当前定位会话绑定的几何资产摘要。")
    relocalization_candidates: List[RelocalizationCandidate] = Field(default_factory=list, description="最近 RTAB-Map/ICP 重定位候选。")
    last_relocalization: Optional[RelocalizationCandidate] = Field(default=None, description="最近一次重定位结果。")
    last_localized_at: Optional[datetime] = Field(default=None, description="最近一次定位就绪时间。")
    last_error: Optional[str] = Field(default=None, description="最近一次定位失败或未就绪说明。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class TopologyNodeSummary(TimestampedContract):
    """平台导航语义层中的拓扑节点摘要。"""

    node_id: str = Field(description="拓扑节点标识。")
    label: str = Field(description="拓扑节点名称。")
    frame_id: str = Field(description="拓扑节点所在坐标系。")
    map_version_id: Optional[str] = Field(default=None, description="关联地图版本。")
    anchor_id: Optional[str] = Field(default=None, description="关联锚点标识。")
    pose: Optional[Pose] = Field(default=None, description="节点中心位姿。")
    aliases: List[str] = Field(default_factory=list, description="节点别名。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class NavigationSemanticContext(TimestampedContract):
    """平台当前激活地图的语义导航上下文。"""

    map_name: Optional[str] = Field(default=None, description="当前地图名称。")
    map_version_id: Optional[str] = Field(default=None, description="当前平台地图版本。")
    localization_session_id: Optional[str] = Field(default=None, description="当前绑定的定位会话。")
    localization_ready: bool = Field(default=False, description="当前平台定位是否已就绪。")
    semantic_regions: List[SpatialSemanticRegion] = Field(default_factory=list, description="当前语义区域集合。")
    scene_spaces: List[SpatialSceneSpace] = Field(default_factory=list, description="当前场景空间集合。")
    topology_nodes: List[TopologyNodeSummary] = Field(default_factory=list, description="当前拓扑节点集合。")
    anchors: List[SpatialAnchor] = Field(default_factory=list, description="当前空间锚点集合。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class LocalizationSnapshot(TimestampedContract):
    """定位服务快照。"""

    source_name: str = Field(description="定位来源名称。")
    current_pose: Optional[Pose] = Field(default=None, description="当前位姿。")
    frame_tree: Optional[FrameTree] = Field(default=None, description="当前坐标树快照。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class MapSnapshot(TimestampedContract):
    """地图服务快照。"""

    source_name: str = Field(description="地图来源名称。")
    version_id: str = Field(description="地图版本标识。")
    revision: int = Field(default=1, ge=1, description="来源内递增版本号。")
    frame_id: Optional[str] = Field(default=None, description="地图快照统一消费坐标系。")
    occupancy_grid: Optional[OccupancyGrid] = Field(default=None, description="占据栅格地图。")
    cost_map: Optional[CostMap] = Field(default=None, description="代价地图。")
    semantic_map: Optional[SemanticMap] = Field(default=None, description="语义地图。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="after")
    def _derive_frame_id(self) -> "MapSnapshot":
        """在未显式提供时，从地图层中推导统一坐标系。"""

        if self.frame_id:
            return self
        for layer in (self.occupancy_grid, self.cost_map, self.semantic_map):
            candidate = str(getattr(layer, "frame_id", "") or "").strip() if layer is not None else ""
            if candidate:
                self.frame_id = candidate
                break
        return self


class NavigationContext(TimestampedContract):
    """导航服务上下文。"""

    current_goal: Optional[NavigationGoal] = Field(default=None, description="当前导航目标。")
    navigation_state: NavigationState = Field(description="导航状态。")
    backend_name: Optional[str] = Field(default=None, description="导航后端名称。")
    goal_reached: bool = Field(default=False, description="当前目标是否到达。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class RuntimeGovernanceSummary(TimestampedContract):
    """运行时观察与治理摘要。"""

    rtabmap_profile: MetadataDict = Field(default_factory=dict, description="RTAB-Map profile 摘要。")
    geometry_asset: MetadataDict = Field(default_factory=dict, description="当前地图几何资产摘要。")
    localization_session: MetadataDict = Field(default_factory=dict, description="当前定位会话摘要。")
    last_relocalization: MetadataDict = Field(default_factory=dict, description="最近一次 RTAB-Map/ICP 重定位摘要。")
    navigation_control_policy: MetadataDict = Field(default_factory=dict, description="导航控制策略摘要。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加治理元数据。")


class ExplorationContext(TimestampedContract):
    """探索服务上下文。"""

    current_request: Optional[ExploreAreaRequest] = Field(default=None, description="当前探索请求。")
    exploration_state: ExplorationState = Field(description="探索状态。")
    backend_name: Optional[str] = Field(default=None, description="探索后端名称。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")
