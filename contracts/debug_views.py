from __future__ import annotations

from datetime import datetime
from typing import List, Optional

from compat import StrEnum
from pydantic import Field

from contracts.base import ContractModel, MetadataDict, utc_now


class DebugDataStatus(StrEnum):
    """调试数据可用性状态。"""

    AVAILABLE = "available"
    MISSING = "missing"
    STALE = "stale"
    ERROR = "error"
    UNSUPPORTED = "unsupported"


class DebugError(ContractModel):
    """调试接口错误说明。"""

    code: str = Field(description="错误码。")
    message: str = Field(description="可展示错误说明。")
    details: MetadataDict = Field(default_factory=dict, description="附加错误细节。")


class DebugEnvelope(ContractModel):
    """只读调试接口统一观测信封。"""

    ok: bool = Field(description="本次读取是否成功取得可用数据。")
    type: str = Field(description="负载类型。")
    source: str = Field(description="只读数据来源。")
    frame_id: Optional[str] = Field(default=None, description="坐标系标识。")
    source_timestamp: datetime = Field(description="来源数据时间戳。")
    gateway_timestamp: datetime = Field(default_factory=utc_now, description="网关聚合时间戳。")
    sequence: Optional[int] = Field(default=None, description="连续数据序号。")
    revision: Optional[str] = Field(default=None, description="快照版本。")
    frequency_hz: Optional[float] = Field(default=None, description="观测到的来源频率。")
    latency_ms: float = Field(default=0.0, ge=0.0, description="网关相对来源延迟。")
    stale: bool = Field(default=False, description="数据是否过期。")
    status: DebugDataStatus = Field(description="数据可用性状态。")
    data: Optional[MetadataDict] = Field(default=None, description="业务负载。")
    error: Optional[DebugError] = Field(default=None, description="错误说明。")


class DebugSourceSummary(ContractModel):
    """调试数据源摘要。"""

    name: str = Field(description="数据源名称。")
    status: DebugDataStatus = Field(description="数据源状态。")
    frame_id: Optional[str] = Field(default=None, description="坐标系。")
    source_timestamp: Optional[datetime] = Field(default=None, description="最新来源时间戳。")
    gateway_timestamp: datetime = Field(default_factory=utc_now, description="网关采样时间戳。")
    latency_ms: Optional[float] = Field(default=None, ge=0.0, description="来源到网关延迟。")
    stale: bool = Field(default=False, description="是否过期。")
    frequency_hz: Optional[float] = Field(default=None, description="观测频率。")
    revision: Optional[str] = Field(default=None, description="来源版本。")
    sequence: Optional[int] = Field(default=None, description="来源序号。")
    message: Optional[str] = Field(default=None, description="状态说明。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class DebugOverviewView(ContractModel):
    """调试面板首屏概览。"""

    health: DebugSourceSummary
    state: DebugSourceSummary
    localization: DebugSourceSummary
    map: DebugSourceSummary
    navigation: DebugSourceSummary
    exploration: DebugSourceSummary
    runtime_governance: DebugSourceSummary
    pointcloud_map: DebugSourceSummary
    pointcloud: DebugSourceSummary
    video: DebugSourceSummary
    logs: DebugSourceSummary


class DebugMapView(ContractModel):
    """二维地图调试视图。"""

    map_snapshot: Optional[MetadataDict] = None
    occupancy_grid: Optional[MetadataDict] = None
    cost_map: Optional[MetadataDict] = None
    semantic_map: Optional[MetadataDict] = None
    legend: MetadataDict = Field(default_factory=lambda: {"unknown": -1, "free": 0, "occupied": 100})


class DebugLocalizationView(ContractModel):
    """定位与轨迹调试视图。"""

    localization_snapshot: Optional[MetadataDict] = None
    pose: Optional[MetadataDict] = None
    frame_tree: Optional[MetadataDict] = None
    trajectory: List[MetadataDict] = Field(default_factory=list)


class DebugNavigationView(ContractModel):
    """导航与探索调试视图。"""

    navigation_context: Optional[MetadataDict] = None
    exploration_context: Optional[MetadataDict] = None
    frontier_count: Optional[int] = None
    waiting_for: Optional[str] = None
    mapping_chain_diagnostics: MetadataDict = Field(default_factory=dict)
    frontiers: Optional[List[MetadataDict]] = None


class DebugPointCloudView(ContractModel):
    """点云调试视图。"""

    frame_id: Optional[str] = None
    source: Optional[str] = None
    status: Optional[DebugDataStatus] = None
    metadata: MetadataDict = Field(default_factory=dict)
    point_count: int = 0
    fields: List[MetadataDict] = Field(default_factory=list)
    points: List[MetadataDict] = Field(default_factory=list)
    format: str = "xyz"
    payload_bytes: Optional[int] = None
    artifact_ref: Optional[str] = None
    data_uri: Optional[str] = None
    protection: MetadataDict = Field(default_factory=dict)
    metrics: MetadataDict = Field(default_factory=dict)


class DebugPointCloudMapView(ContractModel):
    """累计点云地图调试视图。"""

    map_name: Optional[str] = None
    map_version_id: Optional[str] = None
    revision: Optional[str] = None
    frame_id: Optional[str] = None
    source: Optional[str] = None
    source_kind: Optional[str] = None
    geometry_stack: Optional[str] = None
    rtabmap_db_path: Optional[str] = None
    pcd_path: Optional[str] = None
    point_count: int = 0
    fields: List[MetadataDict] = Field(default_factory=list)
    bounds: Optional[MetadataDict] = None
    lods: List[MetadataDict] = Field(default_factory=list)
    format: str = "float32-xyz"
    payload_bytes: Optional[int] = None
    metadata: MetadataDict = Field(default_factory=dict)
    metrics: MetadataDict = Field(default_factory=dict)


class DebugVideoView(ContractModel):
    """视频调试视图。"""

    camera_id: Optional[str] = None
    frame_id: Optional[str] = None
    width_px: Optional[int] = None
    height_px: Optional[int] = None
    encoding: Optional[str] = None
    data_uri: Optional[str] = None
    uri: Optional[str] = None
    artifact_ref: Optional[str] = None
    payload_bytes: Optional[int] = None
    metadata: MetadataDict = Field(default_factory=dict)
    metrics: MetadataDict = Field(default_factory=dict)


class DebugLogEntry(ContractModel):
    """日志或运行时事件调试条目。"""

    sequence: Optional[int] = None
    timestamp: datetime
    level: str = "info"
    source: str
    message: str
    event_type: Optional[str] = None
    payload: MetadataDict = Field(default_factory=dict)


class DebugLogsTailView(ContractModel):
    """日志 tail 调试视图。"""

    next_cursor: int = 0
    entries: List[DebugLogEntry] = Field(default_factory=list)
    source_status: DebugDataStatus = DebugDataStatus.MISSING
    source_message: Optional[str] = None
    metrics: MetadataDict = Field(default_factory=dict, description="日志读取分段耗时与载荷观测。")
