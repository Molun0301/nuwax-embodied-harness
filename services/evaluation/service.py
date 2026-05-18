from __future__ import annotations

from collections import deque
from datetime import datetime
import math
import time
from typing import Deque, Dict, Iterable, List, Optional, Sequence

from contracts.base import MetadataDict, utc_now
from contracts.evaluation import (
    EvaluationGateResult,
    EvaluationRunSummary,
    EvaluationSliceKey,
    EvaluationSliceSummary,
    EvaluationTaskKind,
    EvaluationTraceStatus,
    MemoryEvaluationMetrics,
    NavigationEvaluationMetrics,
    SafetyEvaluationMetrics,
    SemanticNavigationEvaluationTrace,
    SemanticRetrievalMetrics,
)
from contracts.events import RuntimeEventCategory
from contracts.navigation import NavigationGoal, NavigationStatus
from contracts.runtime_views import NavigationContext
from core import EventBus, StateNamespace, StateStore


class EvaluationService:
    """平台侧语义导航任务级评估与质量门禁。"""

    def __init__(
        self,
        *,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 200,
    ) -> None:
        self._state_store = state_store
        self._event_bus = event_bus
        self._traces: Deque[SemanticNavigationEvaluationTrace] = deque(maxlen=max(1, history_limit))
        self._traces_by_id: Dict[str, SemanticNavigationEvaluationTrace] = {}
        self._started_at_monotonic: Dict[str, float] = {}

    def start_trace(
        self,
        *,
        goal: MetadataDict,
        task_kind: EvaluationTaskKind = EvaluationTaskKind.UNKNOWN,
        slice_key: Optional[EvaluationSliceKey] = None,
        trace_id: Optional[str] = None,
        metadata: Optional[MetadataDict] = None,
    ) -> SemanticNavigationEvaluationTrace:
        resolved_trace_id = str(trace_id or self._build_trace_id()).strip()
        trace = SemanticNavigationEvaluationTrace(
            trace_id=resolved_trace_id,
            task_kind=task_kind,
            status=EvaluationTraceStatus.STARTED,
            slice_key=slice_key or self._slice_key_from_metadata(goal),
            goal=dict(goal or {}),
            started_at=utc_now(),
            metadata=dict(metadata or {}),
        )
        self._started_at_monotonic[resolved_trace_id] = time.perf_counter()
        return self._store_trace(trace, event_type="evaluation.trace_started")

    def record_retrieval(
        self,
        *,
        query: str,
        candidates: Sequence[object],
        resolution_metadata: Optional[MetadataDict] = None,
        goal_spec: Optional[object] = None,
        trace_id: Optional[str] = None,
        started_at_monotonic: Optional[float] = None,
    ) -> SemanticNavigationEvaluationTrace:
        metadata = dict(resolution_metadata or {})
        goal_spec_payload = self._model_dump(goal_spec)
        if isinstance(goal_spec_payload, dict):
            metadata.setdefault("goal_spec", goal_spec_payload)
        resolved_trace_id = str(trace_id or metadata.get("evaluation_trace_id") or self._build_trace_id("retrieval")).strip()
        candidate_payloads = [self._candidate_payload(item) for item in candidates]
        expected_ids = self._expected_candidate_ids(metadata)
        confidence_values = [self._coerce_float(item.get("score") or item.get("ranking_total_score")) for item in candidate_payloads]
        confidences = [max(0.0, min(1.0, item)) for item in confidence_values if item is not None]
        top1_hit = self._top_hit(candidate_payloads[:1], expected_ids)
        topk_hit = self._top_hit(candidate_payloads, expected_ids)
        wrong_rate = self._wrong_candidate_rate(candidate_payloads, expected_ids)
        calibration_error = self._confidence_calibration_error(candidate_payloads, expected_ids, confidences)
        llm_latency_ms = self._coerce_float(metadata.get("llm_latency_ms") or metadata.get("grounding_latency_ms"))
        if llm_latency_ms is None and started_at_monotonic is not None:
            llm_latency_ms = max(0.0, (time.perf_counter() - started_at_monotonic) * 1000.0)
        metrics = SemanticRetrievalMetrics(
            query=query,
            candidate_count=int(metadata.get("candidate_count") or len(candidate_payloads) or 0),
            returned_candidate_count=int(metadata.get("returned_candidate_count") or len(candidate_payloads) or 0),
            top1_hit=top1_hit,
            topk_hit=topk_hit,
            wrong_candidate_rate=wrong_rate,
            confidence_calibration_error=calibration_error,
            llm_latency_ms=llm_latency_ms,
            candidate_confidences=confidences,
            expected_candidate_ids=expected_ids,
            metadata={**metadata, "goal_spec": goal_spec_payload},
        )
        trace = self._traces_by_id.get(resolved_trace_id)
        if trace is None:
            trace = SemanticNavigationEvaluationTrace(
                trace_id=resolved_trace_id,
                task_kind=self._task_kind_from_metadata(metadata),
                status=EvaluationTraceStatus.CANDIDATES_RESOLVED,
                slice_key=self._slice_key_from_metadata(metadata, candidates=candidate_payloads),
                goal={"query": query, "goal_spec": goal_spec_payload},
                candidate_list=candidate_payloads,
                selected_candidate=candidate_payloads[0] if candidate_payloads else None,
                retrieval_metrics=metrics,
                started_at=utc_now(),
                metadata=dict(metadata),
            )
        else:
            trace = trace.model_copy(
                update={
                    "status": EvaluationTraceStatus.CANDIDATES_RESOLVED,
                    "candidate_list": candidate_payloads,
                    "selected_candidate": candidate_payloads[0] if candidate_payloads else trace.selected_candidate,
                    "retrieval_metrics": metrics,
                    "slice_key": self._merge_slice(trace.slice_key, metadata, candidate_payloads),
                    "metadata": {**dict(trace.metadata), **metadata},
                },
                deep=True,
            )
        return self._store_trace(trace, event_type="evaluation.retrieval_recorded")

    def record_navigation_context(
        self,
        *,
        context: NavigationContext,
        safety_snapshot: Optional[MetadataDict] = None,
        trace_id: Optional[str] = None,
    ) -> SemanticNavigationEvaluationTrace:
        goal = context.current_goal
        state = context.navigation_state
        metadata = {**dict(getattr(state, "metadata", {}) or {}), **dict(getattr(context, "metadata", {}) or {})}
        resolved_trace_id = str(trace_id or metadata.get("evaluation_trace_id") or self._trace_id_from_goal(goal) or self._build_trace_id("nav")).strip()
        status_text = str(getattr(state.status, "value", state.status))
        safety_metrics = self._safety_metrics(status_text, safety_snapshot or metadata.get("safety_snapshot") or {})
        nav_metrics = self._navigation_metrics(context=context, started_at_monotonic=self._started_at_monotonic.get(resolved_trace_id))
        candidate_payload = self._metadata_candidate(metadata)
        trace = self._traces_by_id.get(resolved_trace_id)
        update = {
            "status": self._trace_status_from_navigation(status_text),
            "goal": self._goal_payload(goal),
            "selected_candidate": candidate_payload or (trace.selected_candidate if trace is not None else None),
            "route": metadata.get("navigation_submission_route") or {},
            "execution_status": status_text,
            "navigation_metrics": nav_metrics,
            "safety_metrics": safety_metrics,
            "slice_key": self._merge_slice(trace.slice_key if trace is not None else None, metadata, [candidate_payload] if candidate_payload else []),
            "metadata": {**(dict(trace.metadata) if trace is not None else {}), **metadata},
        }
        if self._is_terminal_status(status_text):
            update["completed_at"] = utc_now()
            update["latency_ms"] = self._latency_ms(resolved_trace_id)
        if trace is None:
            trace = SemanticNavigationEvaluationTrace(
                trace_id=resolved_trace_id,
                task_kind=self._task_kind_from_metadata(metadata),
                started_at=utc_now(),
                **update,
            )
        else:
            trace = trace.model_copy(update=update, deep=True)
        return self._store_trace(trace, event_type="evaluation.navigation_recorded")

    def record_task_outcome(
        self,
        *,
        task_trace: Optional[MetadataDict],
        execution_status: str,
        verification_result: Optional[object] = None,
        memory_update: Optional[MetadataDict] = None,
        navigation_context: Optional[NavigationContext] = None,
        navigation_goal: Optional[NavigationGoal] = None,
        navigation_candidate: Optional[object] = None,
        failure_reason_code: Optional[str] = None,
        trace_id: Optional[str] = None,
    ) -> SemanticNavigationEvaluationTrace:
        raw_trace = dict(task_trace or {})
        goal_payload = dict(raw_trace.get("goal") or {}) or self._goal_payload(navigation_goal)
        metadata = {**dict(raw_trace.get("metadata") or {}), **dict(getattr(navigation_goal, "metadata", {}) or {})}
        if isinstance(raw_trace.get("next_action"), dict):
            metadata["next_action"] = dict(raw_trace.get("next_action") or {})
        resolved_trace_id = str(trace_id or metadata.get("evaluation_trace_id") or self._trace_id_from_goal(navigation_goal) or self._build_trace_id("outcome")).strip()
        verification_payload = self._verification_payload(verification_result) or raw_trace.get("verification")
        memory_payload = dict(memory_update or raw_trace.get("memory_update") or {})
        raw_candidate = raw_trace.get("candidate")
        candidate_payload = self._candidate_payload(navigation_candidate) if navigation_candidate is not None else (self._candidate_payload(raw_candidate) if isinstance(raw_candidate, dict) else raw_candidate)
        nav_metrics = None
        safety_metrics = None
        if navigation_context is not None:
            nav_metrics = self._navigation_metrics(context=navigation_context, started_at_monotonic=self._started_at_monotonic.get(resolved_trace_id))
            safety_metrics = self._safety_metrics(execution_status, dict(getattr(navigation_context.navigation_state, "metadata", {}) or {}).get("safety_snapshot") or {})
        memory_metrics = self._memory_metrics(memory_payload, verification_payload, failure_reason_code)
        status = self._trace_status_from_execution(execution_status, verification_payload)
        trace = self._traces_by_id.get(resolved_trace_id)
        update = {
            "status": status,
            "goal": goal_payload,
            "selected_candidate": candidate_payload or (trace.selected_candidate if trace is not None else None),
            "route": raw_trace.get("route") or (dict(getattr(getattr(navigation_context, "navigation_state", None), "metadata", {}) or {}).get("navigation_submission_route") if navigation_context is not None else {}) or {},
            "execution_status": execution_status,
            "verification_result": verification_payload,
            "memory_update": memory_payload,
            "navigation_metrics": nav_metrics or (trace.navigation_metrics if trace is not None else None),
            "safety_metrics": safety_metrics or (trace.safety_metrics if trace is not None else None),
            "memory_metrics": memory_metrics,
            "slice_key": self._merge_slice(trace.slice_key if trace is not None else None, metadata, [candidate_payload] if isinstance(candidate_payload, dict) else []),
            "completed_at": utc_now(),
            "latency_ms": self._latency_ms(resolved_trace_id),
            "metadata": {**(dict(trace.metadata) if trace is not None else {}), **metadata, "failure_reason_code": failure_reason_code or ""},
        }
        if trace is None:
            trace = SemanticNavigationEvaluationTrace(
                trace_id=resolved_trace_id,
                task_kind=self._task_kind_from_metadata(metadata),
                candidate_list=[candidate_payload] if isinstance(candidate_payload, dict) else [],
                started_at=utc_now(),
                **update,
            )
        else:
            trace = trace.model_copy(update=update, deep=True)
        return self._store_trace(trace, event_type="evaluation.task_outcome_recorded")

    def list_traces(self, *, limit: Optional[int] = None) -> Sequence[SemanticNavigationEvaluationTrace]:
        items = list(self._traces)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def get_summary(self) -> EvaluationRunSummary:
        traces = list(self._traces)
        slice_summaries = self._build_slice_summaries(traces)
        stage9_summary = self._stage9_completion_summary(traces)
        gate_result = self._gate_result(traces, slice_summaries, stage9_summary)
        return EvaluationRunSummary(
            task_count=len(traces),
            gate_result=gate_result,
            slice_summaries=slice_summaries,
            latest_trace_id=traces[-1].trace_id if traces else None,
            metadata={
                "task_sets": [item.value for item in EvaluationTaskKind],
                "slice_dimensions": ["map_version_id", "perception_provider", "navigation_profile", "target_category", "distance_bucket"],
                "stage9_completion_criteria": stage9_summary,
            },
        )

    def _store_trace(self, trace: SemanticNavigationEvaluationTrace, *, event_type: str) -> SemanticNavigationEvaluationTrace:
        if trace.trace_id not in self._traces_by_id:
            self._traces.append(trace)
        else:
            for index, item in enumerate(self._traces):
                if item.trace_id == trace.trace_id:
                    self._traces[index] = trace
                    break
        self._traces_by_id[trace.trace_id] = trace
        self._state_store.write(
            StateNamespace.SYSTEM,
            "latest_semantic_navigation_evaluation_trace",
            trace,
            source="evaluation_service",
            metadata={"kind": "semantic_navigation_evaluation_trace", "trace_id": trace.trace_id},
        )
        summary = self.get_summary()
        self._state_store.write(
            StateNamespace.SYSTEM,
            "semantic_navigation_evaluation_summary",
            summary,
            source="evaluation_service",
            metadata={"kind": "semantic_navigation_evaluation_summary"},
        )
        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    event_type,
                    category=RuntimeEventCategory.SYSTEM,
                    source="evaluation_service",
                    subject_id=trace.trace_id,
                    message="语义导航评估 trace 已更新。",
                    payload={
                        "trace_id": trace.trace_id,
                        "status": trace.status.value,
                        "execution_status": trace.execution_status,
                        "slice_key": trace.slice_key.model_dump(mode="json"),
                        "gate_passed": summary.gate_result.passed,
                    },
                    metadata={"trace_id": trace.trace_id, "status": trace.status.value},
                )
            )
        return trace

    def _build_trace_id(self, prefix: str = "eval") -> str:
        return "%s_%d" % (prefix, time.time_ns())

    def _trace_id_from_goal(self, goal: Optional[NavigationGoal]) -> Optional[str]:
        metadata = dict(getattr(goal, "metadata", {}) or {})
        return str(metadata.get("evaluation_trace_id") or "").strip() or None

    def _goal_payload(self, goal: Optional[NavigationGoal]) -> MetadataDict:
        if goal is None:
            return {}
        return {
            "goal_id": goal.goal_id,
            "target_name": goal.target_name or "",
            "map_name": goal.map_name or "",
            "target_pose": self._pose_payload(goal.target_pose),
            "metadata": dict(goal.metadata or {}),
        }

    def _candidate_payload(self, candidate: object) -> MetadataDict:
        if isinstance(candidate, dict):
            payload = dict(candidate)
            metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else payload
            return {**payload, **self._candidate_audit_fields(payload, metadata)}
        metadata = dict(getattr(candidate, "metadata", {}) or {})
        record_kind = str(getattr(getattr(candidate, "record_kind", None), "value", getattr(candidate, "record_kind", "")) or metadata.get("record_kind") or "")
        payload = {
            "candidate_id": str(metadata.get("candidate_id") or getattr(candidate, "candidate_id", "") or getattr(candidate, "record_id", "")),
            "record_id": str(getattr(candidate, "record_id", "") or metadata.get("record_id") or ""),
            "record_kind": record_kind,
            "target_name": str(getattr(candidate, "target_name", "") or metadata.get("target_name") or ""),
            "score": self._coerce_float(metadata.get("ranking_total_score") or getattr(candidate, "score", None)),
            "ranking_total_score": self._coerce_float(metadata.get("ranking_total_score") or getattr(candidate, "score", None)),
            "distance_m": self._coerce_float(getattr(candidate, "distance_m", None) or metadata.get("distance_m")),
            "map_version_id": str(getattr(candidate, "map_version_id", "") or metadata.get("map_version_id") or ""),
            "localization_session_id": str(getattr(candidate, "localization_session_id", "") or metadata.get("localization_session_id") or ""),
            "scene_space_id": str(getattr(candidate, "scene_space_id", "") or metadata.get("scene_space_id") or ""),
            "semantic_region_id": str(getattr(candidate, "semantic_region_id", "") or metadata.get("semantic_region_id") or ""),
            "metadata": metadata,
        }
        return {**payload, **self._candidate_audit_fields(candidate, metadata)}

    def _candidate_audit_fields(self, candidate: object, metadata: MetadataDict) -> MetadataDict:
        score_breakdown = metadata.get("score_breakdown") if isinstance(metadata.get("score_breakdown"), dict) else {}
        navigation_quality = metadata.get("navigation_quality") if isinstance(metadata.get("navigation_quality"), dict) else {}
        anchor_id = str(getattr(candidate, "anchor_id", "") or metadata.get("anchor_id") or "")
        verification_event_id = str(getattr(candidate, "verification_event_id", "") or metadata.get("verification_event_id") or "")
        verification_memory_id = str(getattr(candidate, "verification_memory_id", "") or metadata.get("verification_memory_id") or "")
        observation_event_ids = metadata.get("observation_event_ids") if isinstance(metadata.get("observation_event_ids"), list) else []
        if verification_event_id and verification_event_id not in observation_event_ids:
            observation_event_ids = [verification_event_id, *observation_event_ids]
        evidence = {
            "verification_query": str(getattr(candidate, "verification_query", "") or metadata.get("verification_query") or ""),
            "verification_artifact_id": str(getattr(candidate, "verification_artifact_id", "") or metadata.get("verification_artifact_id") or ""),
            "verification_memory_id": verification_memory_id,
            "verification_event_id": verification_event_id,
            "observation_event_ids": observation_event_ids,
            "linked_object_instance_ids": list(metadata.get("linked_object_instance_ids") or []),
            "failure_reason_codes": list(metadata.get("failure_reason_codes") or []),
            "failure_trajectory_event_ids": list(metadata.get("failure_trajectory_event_ids") or []),
        }
        return {
            "anchor": {
                "anchor_id": anchor_id,
                "anchor_pose": self._pose_payload(getattr(candidate, "anchor_pose", None)),
                "inspection_pose": self._pose_payload(getattr(candidate, "inspection_pose", None) or getattr(candidate, "target_pose", None)),
                "target_pose": self._pose_payload(getattr(candidate, "target_pose", None) or getattr(candidate, "inspection_pose", None)),
                "pose_source": str(metadata.get("coordinate_source") or metadata.get("projection_method") or "platform_graph_candidate"),
            },
            "evidence": evidence,
            "score_breakdown": score_breakdown,
            "navigation_quality": navigation_quality,
            "resolution_decision": str(metadata.get("resolution_decision") or ""),
            "allowed_for_navigation": self._coerce_bool(metadata.get("allowed_for_navigation")),
            "excluded_reason": str(metadata.get("excluded_reason") or metadata.get("rejection_reason") or ""),
            "exclusion_reasons": list(metadata.get("exclusion_reasons") or []),
            "goal_spec": metadata.get("goal_spec") if isinstance(metadata.get("goal_spec"), dict) else {},
            "resolution_summary": metadata.get("resolution_summary") if isinstance(metadata.get("resolution_summary"), dict) else {},
            "long_term_static_target_eligible": self._coerce_bool(metadata.get("long_term_static_target_eligible")),
            "dynamic_short_term_obstacle_context": metadata.get("dynamic_short_term_obstacle_context") if isinstance(metadata.get("dynamic_short_term_obstacle_context"), dict) else {},
            "projection_confidence": self._coerce_float(metadata.get("projection_confidence")),
            "support_point_count": metadata.get("support_point_count"),
        }

    def _metadata_candidate(self, metadata: MetadataDict) -> Optional[MetadataDict]:
        candidate = metadata.get("selected_candidate") or metadata.get("navigation_candidate")
        return dict(candidate) if isinstance(candidate, dict) else None

    def _verification_payload(self, verification: Optional[object]) -> Optional[MetadataDict]:
        if verification is None:
            return None
        payload = self._model_dump(verification)
        if isinstance(payload, dict):
            return payload
        return None

    def _model_dump(self, value: Optional[object]) -> object:
        if value is None:
            return None
        dumper = getattr(value, "model_dump", None)
        if callable(dumper):
            return dumper(mode="json")
        if isinstance(value, (dict, list, tuple, str, int, float, bool)):
            return value
        return str(value)

    def _pose_payload(self, pose: Optional[object]) -> Optional[MetadataDict]:
        if pose is None:
            return None
        position = getattr(pose, "position", None)
        return {
            "frame_id": getattr(pose, "frame_id", None),
            "x": self._coerce_float(getattr(position, "x", None)),
            "y": self._coerce_float(getattr(position, "y", None)),
            "z": self._coerce_float(getattr(position, "z", None)),
        }

    def _expected_candidate_ids(self, metadata: MetadataDict) -> List[str]:
        raw = metadata.get("expected_candidate_ids") or metadata.get("expected_candidates") or metadata.get("ground_truth_candidate_ids") or []
        if isinstance(raw, str):
            return [raw] if raw else []
        if isinstance(raw, Iterable):
            return [str(item) for item in raw if str(item or "").strip()]
        return []

    def _top_hit(self, candidates: Sequence[MetadataDict], expected_ids: Sequence[str]) -> Optional[bool]:
        if not expected_ids:
            return None
        expected = set(expected_ids)
        return any(self._candidate_matches(item, expected) for item in candidates)

    def _wrong_candidate_rate(self, candidates: Sequence[MetadataDict], expected_ids: Sequence[str]) -> Optional[float]:
        if not candidates or not expected_ids:
            return None
        expected = set(expected_ids)
        wrong = sum(0 if self._candidate_matches(item, expected) else 1 for item in candidates)
        return wrong / float(len(candidates))

    def _candidate_matches(self, candidate: MetadataDict, expected: set[str]) -> bool:
        values = {
            str(candidate.get("candidate_id") or ""),
            str(candidate.get("record_id") or ""),
            str((candidate.get("metadata") or {}).get("candidate_id") or "") if isinstance(candidate.get("metadata"), dict) else "",
        }
        return bool(values & expected)

    def _confidence_calibration_error(
        self,
        candidates: Sequence[MetadataDict],
        expected_ids: Sequence[str],
        confidences: Sequence[float],
    ) -> Optional[float]:
        if not candidates or not expected_ids or not confidences:
            return None
        expected = set(expected_ids)
        errors = []
        for candidate, confidence in zip(candidates, confidences):
            observed = 1.0 if self._candidate_matches(candidate, expected) else 0.0
            errors.append(abs(confidence - observed))
        return sum(errors) / float(len(errors)) if errors else None

    def _navigation_metrics(self, *, context: NavigationContext, started_at_monotonic: Optional[float]) -> NavigationEvaluationMetrics:
        state = context.navigation_state
        metadata = dict(state.metadata or {})
        status_text = str(getattr(state.status, "value", state.status))
        success = bool(context.goal_reached or state.status == NavigationStatus.SUCCEEDED)
        shortest = self._coerce_float(metadata.get("shortest_path_length_m") or metadata.get("planned_path_length_m"))
        trajectory = self._coerce_float(metadata.get("trajectory_length_m") or metadata.get("distance_travelled_m") or metadata.get("path_length_m"))
        arrival_error = self._coerce_float(metadata.get("arrival_error_m") or state.remaining_distance_m)
        duration_ms = self._coerce_float(metadata.get("duration_ms"))
        if duration_ms is None and started_at_monotonic is not None and self._is_terminal_status(status_text):
            duration_ms = max(0.0, (time.perf_counter() - started_at_monotonic) * 1000.0)
        spl = None
        if success:
            if shortest is not None and trajectory is not None and max(shortest, trajectory) > 0:
                spl = shortest / max(shortest, trajectory)
            else:
                spl = 1.0
        return NavigationEvaluationMetrics(
            success_rate_sample=1.0 if success else (0.0 if self._is_terminal_status(status_text) else None),
            spl_sample=spl,
            trajectory_length_m=trajectory,
            shortest_path_length_m=shortest,
            arrival_error_m=arrival_error,
            duration_ms=duration_ms,
            replan_count=int(metadata.get("replan_count") or (metadata.get("feedback") or {}).get("number_of_recoveries") or 0) if isinstance(metadata.get("feedback") or {}, dict) else 0,
            status=status_text,
            metadata=metadata,
        )

    def _safety_metrics(self, status_text: str, safety_snapshot: MetadataDict) -> SafetyEvaluationMetrics:
        safe_stop = safety_snapshot.get("safe_stop") if isinstance(safety_snapshot.get("safe_stop"), dict) else {}
        obstacle = safety_snapshot.get("official_obstacle_avoidance") if isinstance(safety_snapshot.get("official_obstacle_avoidance"), dict) else {}
        terminal = safety_snapshot.get("terminal_feedback") if isinstance(safety_snapshot.get("terminal_feedback"), dict) else {}
        recoveries = int(terminal.get("number_of_recoveries") or 0) if isinstance(terminal, dict) else 0
        return SafetyEvaluationMetrics(
            collision_count=int(safety_snapshot.get("collision_count") or terminal.get("collision_count") or 0),
            emergency_stop_count=1 if bool(safe_stop.get("safe_stop_active") or safe_stop.get("is_estopped")) else 0,
            official_obstacle_avoidance_trigger_count=1 if bool(obstacle.get("obstacle_avoidance_enabled") and recoveries > 0) else 0,
            near_miss_count=int(safety_snapshot.get("near_miss_count") or safety_snapshot.get("near_distance_risk_count") or 0),
            cancel_count=1 if status_text == "cancelled" else 0,
            timeout_count=1 if status_text == "timeout" or bool(safety_snapshot.get("timeout")) else 0,
            metadata=dict(safety_snapshot or {}),
        )

    def _memory_metrics(
        self,
        memory_payload: MetadataDict,
        verification_payload: Optional[MetadataDict],
        failure_reason_code: Optional[str],
    ) -> MemoryEvaluationMetrics:
        write_success = self._coerce_bool(memory_payload.get("memory_written"))
        if write_success is None:
            outcome = memory_payload.get("outcome") if isinstance(memory_payload.get("outcome"), dict) else {}
            write_success = self._coerce_bool(outcome.get("memory_written"))
        counter_evidence = bool(failure_reason_code) or (
            isinstance(verification_payload, dict) and self._coerce_bool(verification_payload.get("verified")) is False
        )
        duplicate_merge = self._coerce_bool(memory_payload.get("duplicate_instance_merged") or memory_payload.get("duplicate_instance_merge"))
        stale_pollution = self._coerce_bool(memory_payload.get("stale_memory_pollution"))
        return MemoryEvaluationMetrics(
            new_observation_write_success=write_success,
            duplicate_instance_merge=duplicate_merge,
            counter_evidence_generated=counter_evidence,
            stale_memory_pollution=stale_pollution,
            new_observation_write_success_rate=1.0 if write_success is True else (0.0 if write_success is False else None),
            duplicate_instance_merge_rate=1.0 if duplicate_merge is True else (0.0 if duplicate_merge is False else None),
            counter_evidence_generation_rate=1.0 if counter_evidence else 0.0,
            stale_memory_pollution_rate=1.0 if stale_pollution is True else (0.0 if stale_pollution is False else None),
            metadata=dict(memory_payload or {}),
        )

    def _slice_key_from_metadata(self, metadata: MetadataDict, *, candidates: Sequence[MetadataDict] = ()) -> EvaluationSliceKey:
        candidate = candidates[0] if candidates else {}
        candidate_metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        distance = self._coerce_float(metadata.get("distance_m") or candidate.get("distance_m"))
        return EvaluationSliceKey(
            map_version_id=str(metadata.get("map_version_id") or candidate.get("map_version_id") or candidate_metadata.get("map_version_id") or "unknown") or "unknown",
            perception_provider=str(metadata.get("perception_provider") or metadata.get("detector_backend") or candidate_metadata.get("perception_provider") or "unknown") or "unknown",
            navigation_profile=str(metadata.get("navigation_profile") or metadata.get("geometry_profile") or candidate_metadata.get("navigation_profile") or "unknown") or "unknown",
            target_category=str(metadata.get("target_category") or candidate_metadata.get("target_category") or metadata.get("query_scope") or "unknown") or "unknown",
            distance_bucket=str(metadata.get("distance_bucket") or self._distance_bucket(distance) or "unknown") or "unknown",
        )

    def _merge_slice(
        self,
        current: Optional[EvaluationSliceKey],
        metadata: MetadataDict,
        candidates: Sequence[MetadataDict],
    ) -> EvaluationSliceKey:
        incoming = self._slice_key_from_metadata(metadata, candidates=candidates)
        if current is None:
            return incoming
        return EvaluationSliceKey(
            map_version_id=self._prefer_known(current.map_version_id, incoming.map_version_id),
            perception_provider=self._prefer_known(current.perception_provider, incoming.perception_provider),
            navigation_profile=self._prefer_known(current.navigation_profile, incoming.navigation_profile),
            target_category=self._prefer_known(current.target_category, incoming.target_category),
            distance_bucket=self._prefer_known(current.distance_bucket, incoming.distance_bucket),
        )

    def _prefer_known(self, current: str, incoming: str) -> str:
        return incoming if current in {"", "unknown"} and incoming not in {"", "unknown"} else (current or incoming or "unknown")

    def _distance_bucket(self, distance: Optional[float]) -> Optional[str]:
        if distance is None:
            return None
        if distance < 3.0:
            return "near"
        if distance < 10.0:
            return "middle"
        return "far"

    def _task_kind_from_metadata(self, metadata: MetadataDict) -> EvaluationTaskKind:
        raw = str(metadata.get("evaluation_task_kind") or metadata.get("task_kind") or "unknown").strip()
        try:
            return EvaluationTaskKind(raw)
        except ValueError:
            return EvaluationTaskKind.UNKNOWN

    def _trace_status_from_navigation(self, status_text: str) -> EvaluationTraceStatus:
        if status_text == "succeeded":
            return EvaluationTraceStatus.SUCCEEDED
        if status_text == "failed":
            return EvaluationTraceStatus.FAILED
        if status_text == "cancelled":
            return EvaluationTraceStatus.CANCELLED
        return EvaluationTraceStatus.RUNNING

    def _trace_status_from_execution(self, execution_status: str, verification_payload: Optional[MetadataDict]) -> EvaluationTraceStatus:
        if execution_status in {"arrival_verified", "succeeded"}:
            return EvaluationTraceStatus.SUCCEEDED
        if execution_status in {"cancelled"}:
            return EvaluationTraceStatus.CANCELLED
        if execution_status in {"timeout"}:
            return EvaluationTraceStatus.TIMEOUT
        if isinstance(verification_payload, dict) and verification_payload.get("verified") is True:
            return EvaluationTraceStatus.SUCCEEDED
        return EvaluationTraceStatus.FAILED

    def _is_terminal_status(self, status_text: str) -> bool:
        return status_text in {"succeeded", "failed", "cancelled", "timeout"}

    def _latency_ms(self, trace_id: str) -> Optional[float]:
        started = self._started_at_monotonic.get(trace_id)
        if started is None:
            return None
        return max(0.0, (time.perf_counter() - started) * 1000.0)

    def _build_slice_summaries(self, traces: Sequence[SemanticNavigationEvaluationTrace]) -> List[EvaluationSliceSummary]:
        grouped: Dict[str, List[SemanticNavigationEvaluationTrace]] = {}
        keys: Dict[str, EvaluationSliceKey] = {}
        for trace in traces:
            key = trace.slice_key.as_key()
            grouped.setdefault(key, []).append(trace)
            keys[key] = trace.slice_key
        return [self._slice_summary(keys[key], items) for key, items in grouped.items()]

    def _slice_summary(self, slice_key: EvaluationSliceKey, traces: Sequence[SemanticNavigationEvaluationTrace]) -> EvaluationSliceSummary:
        nav_metrics = [item.navigation_metrics for item in traces if item.navigation_metrics is not None]
        retrieval_metrics = [item.retrieval_metrics for item in traces if item.retrieval_metrics is not None]
        safety_metrics = [item.safety_metrics for item in traces if item.safety_metrics is not None]
        memory_metrics = [item.memory_metrics for item in traces if item.memory_metrics is not None]
        return EvaluationSliceSummary(
            slice_key=slice_key,
            task_count=len(traces),
            success_rate=self._mean([item.success_rate_sample for item in nav_metrics]),
            spl=self._mean([item.spl_sample for item in nav_metrics]),
            top1_hit_rate=self._bool_mean([item.top1_hit for item in retrieval_metrics]),
            topk_hit_rate=self._bool_mean([item.topk_hit for item in retrieval_metrics]),
            wrong_candidate_rate=self._mean([item.wrong_candidate_rate for item in retrieval_metrics]),
            collision_count=sum(item.collision_count for item in safety_metrics),
            emergency_stop_count=sum(item.emergency_stop_count for item in safety_metrics),
            cancel_count=sum(item.cancel_count for item in safety_metrics),
            timeout_count=sum(item.timeout_count for item in safety_metrics),
            memory_write_success_rate=self._mean([item.new_observation_write_success_rate for item in memory_metrics]),
        )

    def _gate_result(
        self,
        traces: Sequence[SemanticNavigationEvaluationTrace],
        slice_summaries: Sequence[EvaluationSliceSummary],
        stage9_summary: MetadataDict,
    ) -> EvaluationGateResult:
        failed_reasons: List[str] = []
        if not traces:
            failed_reasons.append("no_evaluation_trace")
        for trace in traces:
            if not trace.goal:
                failed_reasons.append("trace_missing_goal:%s" % trace.trace_id)
            if trace.retrieval_metrics is None and not trace.candidate_list:
                failed_reasons.append("trace_missing_retrieval:%s" % trace.trace_id)
            if trace.navigation_metrics is None and trace.execution_status not in {"current_scene_found", "unknown"}:
                failed_reasons.append("trace_missing_navigation_metrics:%s" % trace.trace_id)
            if trace.safety_metrics is None and trace.execution_status not in {"current_scene_found", "unknown"}:
                failed_reasons.append("trace_missing_safety_metrics:%s" % trace.trace_id)
            if trace.memory_metrics is None and trace.memory_update is not None:
                failed_reasons.append("trace_missing_memory_metrics:%s" % trace.trace_id)
            for reason in self._stage9_trace_failed_reasons(trace):
                failed_reasons.append("%s:%s" % (reason, trace.trace_id))
        return EvaluationGateResult(
            passed=not failed_reasons,
            checked_task_count=len(traces),
            failed_reasons=failed_reasons[:50],
            slice_summaries=list(slice_summaries),
            metadata={
                "gate_version": "stage9-v1",
                "stage9_completion_criteria": stage9_summary,
                "p95_latency_ms": stage9_summary.get("metrics", {}).get("p95_latency_ms") if isinstance(stage9_summary.get("metrics"), dict) else None,
                "candidate_evidence_complete": stage9_summary.get("candidate_evidence", {}).get("complete") if isinstance(stage9_summary.get("candidate_evidence"), dict) else False,
                "failure_recovery_action_coverage": stage9_summary.get("failure_recovery_actions", {}).get("coverage") if isinstance(stage9_summary.get("failure_recovery_actions"), dict) else {},
                "low_confidence_anchor_intercepted": stage9_summary.get("safety_intercepts", {}).get("low_confidence_anchor_intercepted") if isinstance(stage9_summary.get("safety_intercepts"), dict) else False,
                "dynamic_entity_long_term_blocked": stage9_summary.get("safety_intercepts", {}).get("dynamic_entity_long_term_blocked") if isinstance(stage9_summary.get("safety_intercepts"), dict) else False,
            },
        )

    def _stage9_trace_failed_reasons(self, trace: SemanticNavigationEvaluationTrace) -> List[str]:
        reasons: List[str] = []
        metadata = dict(trace.metadata or {})
        candidates = list(trace.candidate_list or [])
        if trace.selected_candidate:
            candidates = [trace.selected_candidate, *candidates]
        if trace.retrieval_metrics is not None or candidates:
            if not any(self._candidate_has_goal_spec(item) for item in candidates) and not self._metadata_has_goal_spec(metadata):
                reasons.append("stage9_missing_goal_spec")
            if candidates and not all(self._candidate_has_navigation_audit(item) for item in candidates):
                reasons.append("stage9_candidate_audit_incomplete")
        if trace.navigation_metrics is not None and not self._route_is_configured_navigation_stack(trace.route):
            reasons.append("stage9_navigation_route_not_configured_stack")
        if trace.execution_status in {"arrival_verified", "arrival_verification_failed"} and trace.verification_result is None:
            reasons.append("stage9_missing_arrival_verification")
        if trace.execution_status not in {"unknown", "current_scene_found"} and trace.memory_update is not None and trace.memory_metrics is None:
            reasons.append("stage9_missing_memory_feedback_metrics")
        if trace.status in {EvaluationTraceStatus.FAILED, EvaluationTraceStatus.CANCELLED, EvaluationTraceStatus.TIMEOUT}:
            next_action = self._trace_next_action(trace)
            if str(next_action.get("action") or "") not in self._stage9_recovery_actions():
                reasons.append("stage9_missing_failure_recovery_action")
        return reasons

    def _stage9_completion_summary(self, traces: Sequence[SemanticNavigationEvaluationTrace]) -> MetadataDict:
        candidates = [candidate for trace in traces for candidate in ([trace.selected_candidate] if trace.selected_candidate else []) + list(trace.candidate_list or []) if isinstance(candidate, dict)]
        terminal_traces = [trace for trace in traces if trace.status in {EvaluationTraceStatus.SUCCEEDED, EvaluationTraceStatus.FAILED, EvaluationTraceStatus.CANCELLED, EvaluationTraceStatus.TIMEOUT}]
        failed_traces = [trace for trace in traces if trace.status in {EvaluationTraceStatus.FAILED, EvaluationTraceStatus.CANCELLED, EvaluationTraceStatus.TIMEOUT}]
        latencies = [item.latency_ms for item in traces if item.latency_ms is not None]
        nav_metrics = [item.navigation_metrics for item in traces if item.navigation_metrics is not None]
        safety_metrics = [item.safety_metrics for item in traces if item.safety_metrics is not None]
        memory_metrics = [item.memory_metrics for item in traces if item.memory_metrics is not None]
        candidate_evidence_complete = bool(candidates) and all(self._candidate_has_navigation_audit(item) for item in candidates)
        recovery_actions = self._stage9_recovery_actions()
        observed_actions = [str(self._trace_next_action(trace).get("action") or "") for trace in failed_traces]
        low_confidence_candidates = [item for item in candidates if self._candidate_low_confidence_or_anchor_risk(item)]
        dynamic_candidates = [item for item in candidates if self._candidate_dynamic_or_not_static(item)]
        criteria = [
            self._criterion_status("natural_language_goal_to_goal_spec_topk", any(self._candidate_has_goal_spec(item) for item in candidates) or any(self._metadata_has_goal_spec(dict(trace.metadata or {})) for trace in traces), "GoalSpec 与 top-k 候选进入检索 trace。", "等待语义检索样本。"),
            self._criterion_status("long_term_memory_candidates_with_explanations", candidate_evidence_complete, "候选含 anchor/evidence/confidence/reachability 审计字段。", "等待完整候选审计样本。"),
            self._criterion_status("semantic_exploration_when_no_reliable_candidate", any(self._trace_has_decision(trace, {"explore"}) for trace in traces), "无可靠候选可进入 explore 决策。", "未观测到 explore 样本。"),
            self._criterion_status("configured_navigation_stack_execution", any(trace.navigation_metrics is not None and self._route_is_configured_navigation_stack(trace.route) for trace in traces), "导航 trace 走当前配置的 geometry 路由。", "等待导航执行样本。"),
            self._criterion_status("arrival_verification", any(trace.verification_result is not None for trace in traces), "到点复核结果进入 trace。", "等待到点复核样本。"),
            self._criterion_status("memory_feedback_affects_ranking", bool(memory_metrics), "成功/失败写回与记忆指标进入评估。", "等待记忆写回样本。"),
            self._criterion_status("dynamic_entity_not_polluting_static_memory", bool(dynamic_candidates) and all(not self._candidate_allowed_for_navigation(item) for item in dynamic_candidates), "动态实体候选被长期静态记忆/直导航拦截。", "未观测动态实体候选样本。"),
            self._criterion_status("low_confidence_3d_anchor_not_direct_navigation", bool(low_confidence_candidates) and all(not self._candidate_allowed_for_navigation(item) for item in low_confidence_candidates), "低置信或低质量三维锚点未直接导航。", "未观测低置信锚点样本。"),
            self._criterion_status("task_trace_and_sr_spl_cc_tl_p95_metrics", bool(traces) and bool(nav_metrics) and self._p95(latencies) is not None, "trace 聚合 SR/SPL/CC/TL/P95。", "等待带 latency 的导航样本。"),
            self._criterion_status("failure_recovery_actions", bool(failed_traces) and all(action in recovery_actions for action in observed_actions), "失败 trace 含候选切换/重规划/探索/澄清/安全停止动作。", "未观测失败恢复样本。"),
        ]
        return {
            "schema_version": "stage9-v1",
            "criteria": criteria,
            "metrics": {
                "sr": self._mean([item.success_rate_sample for item in nav_metrics]),
                "spl": self._mean([item.spl_sample for item in nav_metrics]),
                "cc": sum(item.collision_count for item in safety_metrics),
                "tl_m": self._mean([item.trajectory_length_m for item in nav_metrics]),
                "p95_latency_ms": self._p95(latencies),
                "latency_sample_count": len(latencies),
                "terminal_task_count": len(terminal_traces),
            },
            "candidate_evidence": {
                "complete": candidate_evidence_complete,
                "candidate_count": len(candidates),
                "complete_count": sum(1 for item in candidates if self._candidate_has_navigation_audit(item)),
            },
            "failure_recovery_actions": {
                "required_actions": sorted(recovery_actions),
                "observed_actions": sorted({item for item in observed_actions if item}),
                "coverage": {action: action in observed_actions for action in sorted(recovery_actions)},
            },
            "safety_intercepts": {
                "low_confidence_anchor_candidate_count": len(low_confidence_candidates),
                "low_confidence_anchor_intercepted": bool(low_confidence_candidates) and all(not self._candidate_allowed_for_navigation(item) for item in low_confidence_candidates),
                "dynamic_entity_candidate_count": len(dynamic_candidates),
                "dynamic_entity_long_term_blocked": bool(dynamic_candidates) and all(not self._candidate_allowed_for_navigation(item) for item in dynamic_candidates),
            },
            "sample_notes": [item["risk"] for item in criteria if item["status"] == "not_observed"],
        }

    def _criterion_status(self, criterion_id: str, observed: bool, evidence: str, risk: str) -> MetadataDict:
        return {"id": criterion_id, "status": "observed" if observed else "not_observed", "evidence": evidence if observed else "", "risk": "" if observed else risk}

    def _candidate_has_goal_spec(self, candidate: MetadataDict) -> bool:
        if isinstance(candidate.get("goal_spec"), dict) and candidate.get("goal_spec"):
            return True
        metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        return isinstance(metadata.get("goal_spec"), dict) and bool(metadata.get("goal_spec"))

    def _metadata_has_goal_spec(self, metadata: MetadataDict) -> bool:
        goal_spec = metadata.get("goal_spec")
        return isinstance(goal_spec, dict) and bool(goal_spec)

    def _candidate_has_navigation_audit(self, candidate: MetadataDict) -> bool:
        metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        anchor = candidate.get("anchor") if isinstance(candidate.get("anchor"), dict) else {}
        evidence = candidate.get("evidence") if isinstance(candidate.get("evidence"), dict) else {}
        navigation_quality = candidate.get("navigation_quality") if isinstance(candidate.get("navigation_quality"), dict) else metadata.get("navigation_quality")
        score_breakdown = candidate.get("score_breakdown") if isinstance(candidate.get("score_breakdown"), dict) else metadata.get("score_breakdown")
        has_anchor = bool(anchor.get("anchor_id") or metadata.get("anchor_id") or anchor.get("target_pose") or anchor.get("inspection_pose"))
        has_evidence = bool(evidence.get("verification_query") or evidence.get("verification_memory_id") or evidence.get("verification_event_id") or evidence.get("observation_event_ids") or metadata.get("verification_memory_id"))
        has_confidence = self._coerce_float(candidate.get("ranking_total_score") or candidate.get("score") or metadata.get("ranking_total_score") or metadata.get("candidate_score")) is not None
        has_reachability = isinstance(score_breakdown, dict) and "reachability_score" in score_breakdown
        return has_anchor and has_evidence and has_confidence and has_reachability and isinstance(navigation_quality, dict)

    def _trace_has_decision(self, trace: SemanticNavigationEvaluationTrace, decisions: set[str]) -> bool:
        candidates = ([trace.selected_candidate] if trace.selected_candidate else []) + list(trace.candidate_list or [])
        return any(str((item or {}).get("resolution_decision") or ((item or {}).get("metadata") or {}).get("resolution_decision") or "") in decisions for item in candidates if isinstance(item, dict))

    def _route_is_configured_navigation_stack(self, route: MetadataDict) -> bool:
        if not route:
            return False
        text = " ".join(str(value).lower() for value in route.values())
        return "geometry" in text or "go2slam" in text or "navigation" in text

    def _trace_next_action(self, trace: SemanticNavigationEvaluationTrace) -> MetadataDict:
        metadata = dict(trace.metadata or {})
        next_action = metadata.get("next_action")
        if isinstance(next_action, dict):
            return next_action
        memory_next_action = trace.memory_update.get("next_action") if isinstance(trace.memory_update, dict) else None
        return dict(memory_next_action) if isinstance(memory_next_action, dict) else {}

    def _stage9_recovery_actions(self) -> set[str]:
        return {"try_next_candidate", "replan", "semantic_exploration", "request_clarification", "stop_and_report"}

    def _candidate_allowed_for_navigation(self, candidate: MetadataDict) -> bool:
        metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        allowed = self._coerce_bool(candidate.get("allowed_for_navigation"))
        if allowed is None:
            allowed = self._coerce_bool(metadata.get("allowed_for_navigation"))
        return allowed is True

    def _candidate_low_confidence_or_anchor_risk(self, candidate: MetadataDict) -> bool:
        metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        confidence = self._coerce_float(candidate.get("projection_confidence") or metadata.get("projection_confidence"))
        quality = candidate.get("navigation_quality") if isinstance(candidate.get("navigation_quality"), dict) else metadata.get("navigation_quality")
        quality_level = str((quality or {}).get("level") or metadata.get("navigation_quality_level") or "") if isinstance(quality, dict) else str(metadata.get("navigation_quality_level") or "")
        reasons = [str(item) for item in (candidate.get("exclusion_reasons") or metadata.get("exclusion_reasons") or [])]
        return (confidence is not None and confidence < 0.35) or quality_level in {"needs_review", "continue_exploration", "not_navigable"} or any("projection" in reason or "confidence" in reason for reason in reasons)

    def _candidate_dynamic_or_not_static(self, candidate: MetadataDict) -> bool:
        metadata = candidate.get("metadata") if isinstance(candidate.get("metadata"), dict) else {}
        dynamic_context = candidate.get("dynamic_short_term_obstacle_context") if isinstance(candidate.get("dynamic_short_term_obstacle_context"), dict) else metadata.get("dynamic_short_term_obstacle_context")
        static_eligible = self._coerce_bool(candidate.get("long_term_static_target_eligible"))
        if static_eligible is None:
            static_eligible = self._coerce_bool(metadata.get("long_term_static_target_eligible"))
        dynamic_state = str(metadata.get("dynamic_state") or "")
        return bool(dynamic_context) or static_eligible is False or dynamic_state in {"dynamic", "quasi_static"}

    def _p95(self, values: Iterable[Optional[float]]) -> Optional[float]:
        items = sorted(float(item) for item in values if item is not None and not math.isnan(float(item)))
        if not items:
            return None
        index = max(0, math.ceil(0.95 * len(items)) - 1)
        return items[index]

    def _mean(self, values: Iterable[Optional[float]]) -> Optional[float]:
        items = [float(item) for item in values if item is not None and not math.isnan(float(item))]
        return sum(items) / float(len(items)) if items else None

    def _bool_mean(self, values: Iterable[Optional[bool]]) -> Optional[float]:
        items = [1.0 if item else 0.0 for item in values if item is not None]
        return sum(items) / float(len(items)) if items else None

    def _coerce_float(self, value: object) -> Optional[float]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _coerce_bool(self, value: object) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        text = str(value or "").strip().lower()
        if text == "true":
            return True
        if text == "false":
            return False
        return None
