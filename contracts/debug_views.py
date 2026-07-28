from __future__ import annotations

from datetime import datetime
from typing import Any, List, Optional

from contracts.compat import StrEnum
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
    diagnostics: MetadataDict = Field(default_factory=dict)
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
    failure_code: Optional[str] = None
    frontier_freshness: MetadataDict = Field(default_factory=dict)
    map_revision: Optional[str] = None
    costmap_revision: Optional[str] = None
    navigation_readiness: MetadataDict = Field(default_factory=dict)
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
    revision: Optional[str] = None
    frame_id: Optional[str] = None
    source: Optional[str] = None
    source_kind: Optional[str] = None
    geometry_stack: Optional[str] = None
    mapping_database_path: Optional[str] = None
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
    runtime_event_type: Optional[str] = None
    runtime_message: Optional[str] = None
    display_message: Optional[str] = None
    request_id: Optional[str] = None
    task_id: Optional[str] = None
    capability_name: Optional[str] = None
    duration_ms: Optional[float] = None
    error_category: Optional[str] = None
    payload_preview: Optional[Any] = None
    metadata_preview: Optional[Any] = None
    details_compact: MetadataDict = Field(default_factory=dict)
    suppressed_count: Optional[int] = None
    payload: MetadataDict = Field(default_factory=dict)


class DebugLogsTailView(ContractModel):
    """日志 tail 调试视图。"""

    next_cursor: int = 0
    entries: List[DebugLogEntry] = Field(default_factory=list)
    source_status: DebugDataStatus = DebugDataStatus.MISSING
    source_message: Optional[str] = None
    metrics: MetadataDict = Field(default_factory=dict, description="日志读取分段耗时与载荷观测。")


class CpuMetric(ContractModel):
    """系统 CPU（中央处理器）占用指标。"""

    percent: float = Field(default=0.0, ge=0.0, description="整机 CPU 占用率，已按核平均，范围 0~100。")
    per_core: Optional[List[float]] = Field(default=None, description="每核占用率列表，可选。")
    loadavg: Optional[List[float]] = Field(default=None, description="1/5/15 分钟平均负载，可选（部分平台不支持）。")


class MemoryMetric(ContractModel):
    """系统运存（内存）使用指标。字节为原始值，单位换算交由前端。"""

    total_bytes: int = Field(default=0, ge=0, description="物理内存总量（字节）。")
    used_bytes: int = Field(default=0, ge=0, description="已用物理内存（字节）。")
    available_bytes: int = Field(default=0, ge=0, description="可用物理内存（字节）。")
    percent: float = Field(default=0.0, ge=0.0, description="内存占用率，范围 0~100。")
    swap_used_bytes: Optional[int] = Field(default=None, ge=0, description="已用交换分区（字节），可选。")
    swap_total_bytes: Optional[int] = Field(default=None, ge=0, description="交换分区总量（字节），可选。")


class ProcessMetric(ContractModel):
    """单个被监控进程的资源指标。"""

    pid: int = Field(description="进程号。")
    name: str = Field(description="进程名称。")
    matched_keyword: Optional[str] = Field(default=None, description="命中的白名单关键字或匹配来源（如 process_tree）。")
    display_name: Optional[str] = Field(default=None, description="前端展示用中文角色名。")
    role: Optional[str] = Field(default=None, description="项目内稳定进程角色标识。")
    cmdline_preview: Optional[str] = Field(default=None, description="截断并脱敏后的命令行摘要。")
    rss_bytes: int = Field(default=0, ge=0, description="常驻内存集 RSS（字节）。")
    cpu_percent: float = Field(default=0.0, ge=0.0, description="进程 CPU 占用率；多核可超 100%（按核累加，不平均）。")


class SystemMetricsSnapshot(ContractModel):
    """性能监控一次完整采样快照，经 system.metrics 流推送前端。"""

    status: DebugDataStatus = Field(default=DebugDataStatus.MISSING, description="采集整体状态。")
    psutil_available: bool = Field(default=False, description="psutil 是否可用（缺失则全链路降级）。")
    sample_interval_sec: float = Field(default=2.0, gt=0.0, description="采样周期（秒），即两次采样的时间间隔。")
    monitor_scope: str = Field(default="", description="当前监控范围的可读说明（进程树 + 关键字集）。")
    cpu: CpuMetric = Field(default_factory=CpuMetric, description="系统 CPU 指标。")
    memory: MemoryMetric = Field(default_factory=MemoryMetric, description="系统内存指标。")
    processes: List[ProcessMetric] = Field(default_factory=list, description="被监控进程指标列表。")
    sampled_at: float = Field(default=0.0, ge=0.0, description="采样时刻的 Unix 时间戳（秒）。")
    source: str = Field(default="psutil", description="采集来源。")
