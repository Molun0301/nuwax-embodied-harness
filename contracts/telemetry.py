"""平台遥测契约，与网页视图独立。"""

from __future__ import annotations
from typing import List, Optional
from pydantic import Field
from contracts.base import ContractModel
from contracts.compat import StrEnum


class TelemetryStatus(StrEnum):
    """遥测数据可用性状态。"""

    AVAILABLE = "available"
    MISSING = "missing"
    STALE = "stale"
    ERROR = "error"
    UNSUPPORTED = "unsupported"


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

    status: TelemetryStatus = Field(default=TelemetryStatus.MISSING, description="采集整体状态。")
    psutil_available: bool = Field(default=False, description="psutil 是否可用（缺失则全链路降级）。")
    sample_interval_sec: float = Field(default=2.0, gt=0.0, description="采样周期（秒），即两次采样的时间间隔。")
    monitor_scope: str = Field(default="", description="当前监控范围的可读说明（进程树 + 关键字集）。")
    cpu: CpuMetric = Field(default_factory=CpuMetric, description="系统 CPU 指标。")
    memory: MemoryMetric = Field(default_factory=MemoryMetric, description="系统内存指标。")
    processes: List[ProcessMetric] = Field(default_factory=list, description="被监控进程指标列表。")
    sampled_at: float = Field(default=0.0, ge=0.0, description="采样时刻的 Unix 时间戳（秒）。")
    source: str = Field(default="psutil", description="采集来源。")
