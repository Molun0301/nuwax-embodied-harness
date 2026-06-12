from __future__ import annotations

from datetime import datetime

from contracts.compat import StrEnum
from pydantic import Field, model_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from typing import List, Optional


class EvaluationTaskKind(StrEnum):
    """语义导航评估任务集类型。"""

    KNOWN_TARGET_REVISIT = "known_target_revisit"
    NEW_TARGET_DISCOVERY = "new_target_discovery"
    WRONG_CANDIDATE_RECOVERY = "wrong_candidate_recovery"
    DYNAMIC_OBSTACLE_INTERFERENCE = "dynamic_obstacle_interference"
    LONG_HORIZON_MULTI_TARGET = "long_horizon_multi_target"
    SEMANTIC_EXPLORATION_COVERAGE = "semantic_exploration_coverage"
    UNKNOWN = "unknown"


class EvaluationTraceStatus(StrEnum):
    """任务级 trace 状态。"""

    STARTED = "started"
    CANDIDATES_RESOLVED = "candidates_resolved"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    UNKNOWN = "unknown"


class EvaluationSliceKey(ContractModel):
    """评估聚合切分维度。"""

    map_version_id: str = Field(default="unknown", description="地图版本。")
    perception_provider: str = Field(default="unknown", description="感知 provider（提供方）。")
    navigation_profile: str = Field(default="unknown", description="导航 profile（配置档）。")
    target_category: str = Field(default="unknown", description="目标类别。")
    distance_bucket: str = Field(default="unknown", description="距离等级。")

    def as_key(self) -> str:
        return "|".join(
            [
                self.map_version_id,
                self.perception_provider,
                self.navigation_profile,
                self.target_category,
                self.distance_bucket,
            ]
        )


class SemanticRetrievalMetrics(ContractModel):
    """语义检索指标。"""

    query: str = Field(description="检索文本。")
    candidate_count: int = Field(default=0, ge=0, description="候选总数。")
    returned_candidate_count: int = Field(default=0, ge=0, description="返回候选数。")
    top1_hit: Optional[bool] = Field(default=None, description="top-1 是否命中真值。")
    topk_hit: Optional[bool] = Field(default=None, description="top-k 是否命中真值。")
    wrong_candidate_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="错误候选率。")
    confidence_calibration_error: Optional[float] = Field(default=None, ge=0.0, description="候选置信度校准误差。")
    llm_latency_ms: Optional[float] = Field(default=None, ge=0.0, description="LLM（大语言模型）延迟。")
    candidate_confidences: List[float] = Field(default_factory=list, description="候选置信度列表。")
    expected_candidate_ids: List[str] = Field(default_factory=list, description="真值候选标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class NavigationEvaluationMetrics(ContractModel):
    """导航执行指标。"""

    success_rate_sample: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="单任务 SR（成功率）样本。")
    spl_sample: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="单任务 SPL（路径长度加权成功率）样本。")
    trajectory_length_m: Optional[float] = Field(default=None, ge=0.0, description="TL（轨迹长度），单位米。")
    shortest_path_length_m: Optional[float] = Field(default=None, ge=0.0, description="最短路径长度，单位米。")
    arrival_error_m: Optional[float] = Field(default=None, ge=0.0, description="到达误差，单位米。")
    duration_ms: Optional[float] = Field(default=None, ge=0.0, description="导航耗时。")
    replan_count: int = Field(default=0, ge=0, description="重规划次数。")
    status: str = Field(default="unknown", description="导航终态。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class SafetyEvaluationMetrics(ContractModel):
    """安全指标。"""

    collision_count: int = Field(default=0, ge=0, description="碰撞次数。")
    emergency_stop_count: int = Field(default=0, ge=0, description="急停次数。")
    official_obstacle_avoidance_trigger_count: int = Field(default=0, ge=0, description="官方避障触发次数。")
    near_miss_count: int = Field(default=0, ge=0, description="近距离风险次数。")
    cancel_count: int = Field(default=0, ge=0, description="取消次数。")
    timeout_count: int = Field(default=0, ge=0, description="超时次数。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class MemoryEvaluationMetrics(ContractModel):
    """记忆闭环指标。"""

    new_observation_write_success: Optional[bool] = Field(default=None, description="新观察是否写入成功。")
    duplicate_instance_merge: Optional[bool] = Field(default=None, description="重复实例是否合并。")
    counter_evidence_generated: Optional[bool] = Field(default=None, description="反证是否生成。")
    stale_memory_pollution: Optional[bool] = Field(default=None, description="是否发生旧记忆污染。")
    new_observation_write_success_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="新观察写入成功率。")
    duplicate_instance_merge_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="重复实例合并率。")
    counter_evidence_generation_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="反证生成率。")
    stale_memory_pollution_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="旧记忆污染率。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class SemanticNavigationEvaluationTrace(TimestampedContract):
    """语义导航任务标准 trace。"""

    trace_id: str = Field(description="评估 trace 标识。")
    task_kind: EvaluationTaskKind = Field(default=EvaluationTaskKind.UNKNOWN, description="任务集类型。")
    status: EvaluationTraceStatus = Field(default=EvaluationTraceStatus.STARTED, description="trace 状态。")
    slice_key: EvaluationSliceKey = Field(default_factory=EvaluationSliceKey, description="聚合切分维度。")
    goal: MetadataDict = Field(default_factory=dict, description="目标信息。")
    candidate_list: List[MetadataDict] = Field(default_factory=list, description="候选列表。")
    selected_candidate: Optional[MetadataDict] = Field(default=None, description="最终选择候选。")
    route: MetadataDict = Field(default_factory=dict, description="路由信息。")
    execution_status: str = Field(default="unknown", description="执行状态。")
    verification_result: Optional[MetadataDict] = Field(default=None, description="复核结果。")
    memory_update: Optional[MetadataDict] = Field(default=None, description="记忆更新结果。")
    latency_ms: Optional[float] = Field(default=None, ge=0.0, description="端到端任务耗时。")
    retrieval_metrics: Optional[SemanticRetrievalMetrics] = Field(default=None, description="语义检索指标。")
    navigation_metrics: Optional[NavigationEvaluationMetrics] = Field(default=None, description="导航指标。")
    safety_metrics: Optional[SafetyEvaluationMetrics] = Field(default=None, description="安全指标。")
    memory_metrics: Optional[MemoryEvaluationMetrics] = Field(default=None, description="记忆指标。")
    started_at: Optional[datetime] = Field(default=None, description="任务开始时间。")
    completed_at: Optional[datetime] = Field(default=None, description="任务完成时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class EvaluationSliceSummary(ContractModel):
    """单个切片的聚合摘要。"""

    slice_key: EvaluationSliceKey = Field(description="切分维度。")
    task_count: int = Field(default=0, ge=0, description="任务数量。")
    success_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="SR 聚合。")
    spl: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="SPL 聚合。")
    top1_hit_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="top-1 命中率。")
    topk_hit_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="top-k 命中率。")
    wrong_candidate_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="错误候选率。")
    collision_count: int = Field(default=0, ge=0, description="碰撞次数。")
    emergency_stop_count: int = Field(default=0, ge=0, description="急停次数。")
    cancel_count: int = Field(default=0, ge=0, description="取消次数。")
    timeout_count: int = Field(default=0, ge=0, description="超时次数。")
    memory_write_success_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="记忆写入成功率。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class EvaluationGateResult(TimestampedContract):
    """评估质量门禁结果。"""

    passed: bool = Field(description="门禁是否通过。")
    checked_task_count: int = Field(default=0, ge=0, description="纳入检查任务数。")
    failed_reasons: List[str] = Field(default_factory=list, description="失败原因。")
    slice_summaries: List[EvaluationSliceSummary] = Field(default_factory=list, description="切片摘要。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class EvaluationRunSummary(TimestampedContract):
    """评估运行聚合摘要。"""

    task_count: int = Field(default=0, ge=0, description="任务数量。")
    gate_result: EvaluationGateResult = Field(description="质量门禁结果。")
    supported_task_kinds: List[EvaluationTaskKind] = Field(default_factory=lambda: list(EvaluationTaskKind), description="已建模任务集。")
    slice_summaries: List[EvaluationSliceSummary] = Field(default_factory=list, description="切片摘要。")
    latest_trace_id: Optional[str] = Field(default=None, description="最近 trace 标识。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="after")
    def _sync_gate_summaries(self) -> "EvaluationRunSummary":
        if not self.slice_summaries and self.gate_result.slice_summaries:
            self.slice_summaries = list(self.gate_result.slice_summaries)
        return self
