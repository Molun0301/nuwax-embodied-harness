"""在一个任务线程内识别、查询、导航并核对目标。"""
from __future__ import annotations

from contextlib import contextmanager
import math
import logging
from typing import Any, Dict, List, Optional, Tuple

from contracts.frame_semantics import frame_ids_semantically_equal
from core.errors import GatewayError
from gateways.capabilities.navigation import navigate_to_pose
from gateways.capabilities.exploration import explore_area
from gateways.capabilities.maps import load_map, save_map, start_mapping
from skills.common import params as skill_params
from skills.memory import remember_sample, save_sample_photo

LOGGER = logging.getLogger(__name__)


class _SearchHit(Exception):
    """只退出当前探索步骤，保留父任务继续核对目标。"""


class _TargetSearch:
    _normalize_query = staticmethod(skill_params.normalize_query)

    def __init__(self, platform, ctx, args, capability):
        self.platform, self.ctx, self.args = platform, ctx, args
        self.runtime = platform.perception.video_runtime
        self.query = args["query"]
        self.camera_id = args.get("camera_id")
        self.backend = args.get("detector_backend")
        self.reason = f"{capability}:{ctx.task_id}"
        self.map_name = args.get("map_name") or platform.geometry_state.snapshot().map_name
        self.threshold = args.get("verify_similarity_threshold", .55)
        self.rejected = set()

    @contextmanager
    def acquired(self):
        self.runtime.acquire(self.reason, interval_sec=self.args.get("search_refresh_interval_sec", .8),
            camera_id=self.camera_id, detector_backend_name=self.backend, open_vocabulary_classes=(self.query,))
        try:
            yield
        finally:
            self.runtime.release(self.reason)

    def _frame_ids_match(self, left, right):
        if not left or not right:
            return False
        try:
            return frame_ids_semantically_equal(left, right)
        except ValueError:
            return False

    def _target_navigation_map_frame_ids(self):
        frame_id = self.platform.geometry_state.snapshot().info.map_frame_id
        normalized = frame_id.strip().lower().replace("/", "_")
        disallowed = (not normalized or any(token in normalized for token in ("camera", "cam", "lidar", "imu", "base", "footprint"))
                      or any(self._frame_ids_match(frame_id, value) for value in ("odom", "odom_combined", "base", "base_link", "base_footprint")))
        return {"map"} if disallowed else {"map", frame_id}

    def _target_pose_is_map_frame(self, frame_id):
        return any(self._frame_ids_match(frame_id, value) for value in self._target_navigation_map_frame_ids())

    def prepare_map(self, *, create_if_missing=False):
        state = self.platform.geometry_state.snapshot()
        if state.map_name == self.map_name and (not create_if_missing or state.can_navigate):
            return
        saved = self.platform.maps.get(self.map_name) if create_if_missing else None
        self.ctx.check()
        if create_if_missing and not (saved is not None and saved.saved):
            start_mapping(self.platform, {"map_name": self.map_name})
        else:
            load_map(self.platform, self.ctx, {"map_name": self.map_name})
        self.ctx.check()
        self.platform.geometry_state.wait_for(
            lambda state: state.map_name == self.map_name and state.can_navigate, self.ctx)
        self.platform.memory.wait_for_library(self.map_name, self.ctx)

    def refresh(self, *, frame_count=1, interval_sec=.05, remember=True, source="target_search"):
        samples = self.runtime.run_burst(self.ctx, reason=self.reason, frame_count=frame_count,
                                         interval_sec=interval_sec)
        sample = samples[-1]
        return sample, self.observe_sample(sample, frame_count=len(samples), remember=remember, source=source)

    def observe_sample(self, sample, *, frame_count=1, remember=True, source="target_search"):
        evidence = self._build_target_detection_evidence(self.query, sample.context)
        written = None
        if remember and evidence["has_detection_evidence"]:
            try:
                written = remember_sample(self.platform, sample, title="目标识别：" + self.query[:48],
                    summary=sample.context.scene_summary.headline, tags=[self.query, *evidence["matched_labels"]],
                    metadata={"source": source, "capture_mode": "target_search_burst",
                        "target_search_query": self.query, "target_detection_evidence": True,
                        "detection_metadata": {"query": self.query, "open_vocabulary_classes": [self.query]}, **evidence})
            except Exception:
                LOGGER.exception("目标检测观察写入失败 query=%s", self.query)
        return {"frame_count": frame_count, "scene_headline": sample.context.scene_summary.headline,
            "camera_id": sample.context.camera_id, "detection_evidence": evidence, "memory_written": written is not None}

    def verify(self, sample, *, candidate=None, goal_id=None):
        save_sample_photo(self.platform, sample)
        result = self.platform.memory.verify_arrival_observation(self.query, perception_context=sample.context,
            current_pose=sample.pose, navigation_candidate=candidate, similarity_threshold=self.threshold,
            expected_map_name=self.map_name, goal_id=goal_id)
        evidence = self._build_target_detection_evidence(self.query, sample.context)
        if result.verified and not evidence["has_detection_evidence"]:
            result = result.model_copy(update={"verified": False,
                "reason": "当前场景缺少目标检测证据，已拒绝开放词汇找物命中。",
                "metadata": {**result.metadata, "target_detection_required": True, "target_detection_evidence": evidence}})
        return result

    def search_state(self, sample):
        verification = self.verify(sample)
        candidates = tuple(candidate for candidate in self.platform.memory.resolve_navigation_candidates(
            self.query, camera_id=self.camera_id, map_name=self.map_name, limit=5)
            if candidate.record_id not in self.rejected)
        diagnostics = [self._assess_target_navigation_candidate(candidate, query=self.query) for candidate in candidates]
        allowed = [candidate for candidate, diagnostic in zip(candidates, diagnostics) if diagnostic["allowed_for_navigation"]]
        selected, clarification = self._select_low_ambiguity_target_candidates(
            query=self.query, candidates=allowed, candidate_diagnostics=diagnostics)
        return {"verification": verification, "candidates": selected, "candidate_diagnostics": diagnostics,
                "clarification": clarification}

    def _assess_target_navigation_candidate(self, candidate, *, query=""):
        diagnostic = self._build_target_candidate_diagnostic(candidate, query=query)
        if diagnostic["record_kind"] not in {"object_instance", "observation_event"}:
            reason = "target_navigation_source_not_object_evidence"
        elif not diagnostic["has_target_evidence"]:
            reason = "target_evidence_missing"
        elif not diagnostic["has_goal_pose"] or not diagnostic["target_frame_id"]:
            reason = "target_pose_missing"
        elif not diagnostic["target_pose_in_map_frame"]:
            reason = "target_pose_not_map_frame"
        elif "current_pose_fallback" in {diagnostic["projection_method"], diagnostic["geometry_projection_method"]}:
            reason = "projection_method_current_pose_fallback"
        elif diagnostic["support_point_count"] is None or diagnostic["support_point_count"] <= 0:
            reason = "support_point_count_below_threshold"
        elif not diagnostic["has_projection_evidence"]:
            reason = "projection_evidence_missing"
        elif not diagnostic["projection_confidence_valid"]:
            reason = "projection_confidence_invalid"
        elif diagnostic["projection_confidence"] is not None and diagnostic["projection_confidence"] < .2:
            reason = "projection_confidence_below_threshold"
        elif diagnostic["segmentation_review_required"]:
            reason = diagnostic["segmentation_quality_reason"] or "segmentation_review_required"
        else:
            reason = self._candidate_rejection(candidate)
        diagnostic.update(allowed_for_navigation=reason is None, rejection_reason=reason)
        return diagnostic

    @staticmethod
    def _candidate_rejection(candidate):
        """候选评分与衰减只由记忆核心拥有，这里消费其最终判定。"""
        metadata = candidate.metadata
        if metadata["allowed_for_navigation"] is True and metadata["resolution_decision"] == "navigate":
            return None
        return metadata["excluded_reason"] or metadata["resolution_decision"]

    def writeback(self, sample, verification, candidate, goal_id):
        candidate_id = candidate.candidate_id if candidate is not None else ""
        record_id = candidate.record_id if candidate is not None else ""
        record_kind = candidate.metadata["record_kind"] if candidate is not None else ""
        target_name = candidate.target_name if candidate is not None else self.query
        outcome = {"memory_written": False, "skipped_reason": "", "candidate_confidence_decay": 0.0}
        try:
            entry = remember_sample(self.platform, sample,
                title=("到点观察：" if verification.verified else "到点反证：") + self.query[:48],
                tags=list(dict.fromkeys([self.query, target_name, *verification.matched_labels])),
                metadata={"source": "target_arrival_verify", "capture_mode": "arrival_writeback",
                    "arrival_writeback": True, "arrival_verified": verification.verified,
                    "verification_score": round(verification.score, 6), "verification_reason": verification.reason,
                    "verification_reason_code": verification.metadata.get("arrival_reason_code", ""),
                    "candidate_id": candidate_id, "goal_id": goal_id, "record_id": record_id,
                    "record_kind": record_kind, "target_name": target_name,
                    "map_name": sample.map_name, "matched_memory_id": verification.matched_memory_id or "",
                    "verification_query": verification.metadata.get("verification_query", self.query),
                    "evidence_kind": "success_evidence" if verification.verified else "failure_counter_evidence"})
        except Exception:
            LOGGER.exception("到点观察写回失败 query=%s candidate_id=%s", self.query, candidate_id)
            outcome["skipped_reason"] = "ingest_exception"
            return outcome
        outcome.update(memory_written=entry is not None, memory_id=entry.memory_id if entry is not None else None)
        if not verification.verified and (record_id or candidate_id):
            amount = max(.05, min(.25, .1 + max(0.0, .55 - verification.score)))
            try:
                applied = self.platform.memory.apply_candidate_confidence_decay(map_name=sample.map_name,
                    record_id=record_id, candidate_id=candidate_id, record_kind=record_kind,
                    decay_amount=amount, reason_code=verification.metadata.get("arrival_reason_code", "insufficient_evidence"))
                outcome["candidate_confidence_decay"] = float(applied or 0.0)
            except Exception:
                LOGGER.exception("到点反证衰减失败 record_id=%s", record_id)
        return outcome

    def current_result(self, state, refresh):
        return {"query": self.query, "found_mode": "current_scene", "perception_refresh": refresh,
            "arrival_verification": state["verification"], "navigation_candidate": None,
            "navigation_context": None, "attempted_candidate_ids": [],
            "candidate_diagnostics": state["candidate_diagnostics"], "rejected_candidate_ids": sorted(self.rejected)}

    def navigate_candidates(self, state, *, allow_not_found):
        diagnostics = state["candidate_diagnostics"]
        if state["clarification"] is not None and not allow_not_found:
            self.ctx.progress(1.0, "clarification_required", "找到多个可能目标，需要澄清后再导航。")
            return {"query": self.query, "found_mode": "clarification_required", "action": "clarify",
                "clarification": state["clarification"], "navigation_candidate": None, "navigation_context": None,
                "attempted_candidate_ids": [], "candidate_diagnostics": diagnostics,
                "rejected_candidate_ids": sorted(self.rejected)}
        self.rejected.update(item["record_id"] for item in diagnostics if not item["allowed_for_navigation"] and item["record_id"])
        attempted, last_error, last_verification = [], None, None
        for candidate in state["candidates"]:
            attempted.append(candidate.record_id)
            diagnostic = next(item for item in diagnostics if item["record_id"] == candidate.record_id)
            diagnostic["navigation_attempted"] = True
            pose = candidate.inspection_pose
            orientation = pose.orientation
            yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))
            self.ctx.progress(.55, "target_candidate_navigation", "正在前往记忆候选目标。", candidate_record_id=candidate.record_id)
            try:
                navigation = navigate_to_pose(self.platform, self.ctx, {"x": pose.position.x, "y": pose.position.y,
                    "z": pose.position.z, "yaw_rad": yaw, "frame_id": pose.frame_id,
                    "motion_generation": self.args["motion_generation"]})
            except GatewayError as error:
                self.ctx.check()
                self.rejected.add(candidate.record_id)
                diagnostic.update(allowed_for_navigation=False, rejection_reason=error.error_code)
                last_error = error
                continue
            sample, refresh = self.refresh(frame_count=3, remember=False, source="target_arrival_verify")
            verification = self.verify(sample, candidate=candidate, goal_id=navigation["goal_id"])
            writeback = self.writeback(sample, verification, candidate, navigation["goal_id"])
            if verification.verified:
                return {"query": self.query, "found_mode": "memory_candidate_navigation",
                    "navigation_candidate": candidate, "navigation_context": navigation,
                    "arrival_verification": verification, "observation_writeback": writeback,
                    "perception_refresh": refresh, "attempted_candidate_ids": attempted,
                    "candidate_diagnostics": diagnostics, "rejected_candidate_ids": sorted(self.rejected)}
            self.rejected.add(candidate.record_id)
            diagnostic.update(allowed_for_navigation=False, rejection_reason=verification.reason or "arrival_verification_failed")
            last_verification = verification
        if allow_not_found:
            return None
        details = {"map_name": self.map_name, "query": self.query, "attempted_candidate_ids": attempted,
                   "candidate_diagnostics": diagnostics, "rejected_candidate_ids": sorted(self.rejected)}
        if last_verification is not None:
            raise GatewayError("已到达候选位置，但目标复核未通过：" + last_verification.reason,
                error_code="arrival_verification_failed", http_status=409,
                details={**details, "arrival_verification": last_verification.model_dump(mode="json")})
        if last_error is not None:
            raise GatewayError("查找目标时导航候选全部失败：" + last_error.message,
                error_code=last_error.error_code, http_status=last_error.http_status, details=details)
        raise GatewayError("未找到具备目标检测和地图投影证据的目标：" + self.query,
            error_code="target_not_found_with_evidence", http_status=404, details=details)

    def _target_label_matches_query(self, query: str, label: str) -> bool:
        normalized_query = self._normalize_query(query)
        normalized_label = self._normalize_query(label)
        if not normalized_query or not normalized_label:
            return False
        return (
            normalized_query == normalized_label
            or normalized_query in normalized_label
            or normalized_label in normalized_query
        )

    def _target_query_matches_any(self, query: str, values: Any) -> bool:
        normalized_query = self._normalize_query(query)
        if not normalized_query:
            return False
        if isinstance(values, dict):
            iterable = values.values()
        elif isinstance(values, (list, tuple, set, frozenset)):
            iterable = values
        else:
            iterable = (values,)
        for value in iterable:
            if isinstance(value, dict):
                if self._target_query_matches_any(normalized_query, value):
                    return True
                continue
            if isinstance(value, (list, tuple, set, frozenset)):
                if self._target_query_matches_any(normalized_query, value):
                    return True
                continue
            if self._target_label_matches_query(normalized_query, str(value or "")):
                return True
        return False

    def _build_target_detection_evidence(self, query: str, perception_context) -> Dict[str, Any]:
        evidence: Dict[str, Any] = {
            "has_detection_evidence": False,
            "matched_labels": [],
            "matched_detection_count": 0,
            "best_detection_score": None,
            "best_detection_label": "",
            "projection_method": "",
            "geometry_projection_method": "",
            "projection_confidence": None,
            "support_point_count": None,
            "projection_degradation_reason": "",
        }
        if perception_context is None:
            return evidence
        matched_detections = []
        for detection in perception_context.observation.detections_2d:
            labels = [detection.label]
            attributes = dict(detection.attributes or {})
            labels.extend(self._coerce_optional_str_list(attributes.get("aliases")))
            if any(self._target_label_matches_query(query, label) for label in labels):
                matched_detections.append(detection)
        if not matched_detections:
            return evidence
        matched_detections.sort(key=lambda item: item.score, reverse=True)
        best_detection = matched_detections[0]
        attributes = dict(best_detection.attributes or {})
        evidence.update(
            {
                "has_detection_evidence": True,
                "matched_labels": sorted(
                    {
                        str(detection.label).strip()
                        for detection in matched_detections
                        if str(detection.label or "").strip()
                    }
                ),
                "matched_detection_count": len(matched_detections),
                "best_detection_score": float(best_detection.score),
                "best_detection_label": str(best_detection.label),
                "projection_method": str(attributes.get("projection_method") or ""),
                "geometry_projection_method": str(attributes.get("geometry_projection_method") or ""),
                "projection_confidence": self._coerce_optional_float(attributes.get("projection_confidence")),
                "support_point_count": self._coerce_optional_int(attributes.get("support_point_count")),
                "projection_degradation_reason": str(attributes.get("projection_degradation_reason") or ""),
            }
        )
        return evidence

    def _coerce_optional_float(self, value: Any) -> Optional[float]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_int(self, value: Any) -> Optional[int]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_bool(self, value: Any) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        normalized = str(value or "").strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
        return None

    def _coerce_optional_str_list(self, value: Any) -> List[str]:
        return skill_params.coerce_optional_str_list(value)

    def _build_target_candidate_diagnostic(
        self,
        candidate,
        *,
        query: str = "",
        allowed_for_navigation: Optional[bool] = None,
        rejection_reason: Optional[str] = None,
    ) -> Dict[str, Any]:
        metadata = dict(getattr(candidate, "metadata", {}) or {})
        normalized_query = self._normalize_query(query)
        candidate_score = self._coerce_optional_float(metadata.get("candidate_score"))
        projection_confidence_raw = metadata.get("projection_confidence")
        projection_confidence_present = "projection_confidence" in metadata
        projection_confidence = self._coerce_optional_float(projection_confidence_raw)
        projection_confidence_valid = (
            not projection_confidence_present
            or (projection_confidence is not None and math.isfinite(projection_confidence))
        )
        support_point_count = self._coerce_optional_int(metadata.get("support_point_count"))
        segmentation_score = self._coerce_optional_float(metadata.get("segmentation_score"))
        predicted_iou = self._coerce_optional_float(metadata.get("predicted_iou"))
        segmentation_quality_score = self._coerce_optional_float(metadata.get("segmentation_quality_score"))
        stability_score = self._coerce_optional_float(metadata.get("stability_score"))
        coverage = self._coerce_optional_float(metadata.get("coverage"))
        map_projection_coverage = self._coerce_optional_float(metadata.get("map_projection_coverage"))
        depth_coverage = self._coerce_optional_float(metadata.get("depth_coverage"))
        reprojection_error = self._coerce_optional_float(metadata.get("reprojection_error"))
        segmentation_support_point_count = self._coerce_optional_int(metadata.get("segmentation_support_point_count"))
        has_structured_segmentation = bool(self._coerce_optional_bool(metadata.get("has_structured_segmentation")))
        segmentation_review_required = bool(self._coerce_optional_bool(metadata.get("segmentation_review_required")))
        segmentation_risk_factors = self._coerce_optional_str_list(metadata.get("segmentation_risk_factors"))
        segmentation_occlusion_flags = self._coerce_optional_str_list(metadata.get("segmentation_occlusion_flags"))
        segmentation_truncation_flags = self._coerce_optional_str_list(metadata.get("segmentation_truncation_flags"))
        navigable = self._coerce_optional_bool(metadata.get("navigable"))
        navigation_quality = metadata.get("navigation_quality")
        if not isinstance(navigation_quality, dict):
            navigation_quality = {}
        else:
            navigation_quality = dict(navigation_quality)
        target_pose = candidate.inspection_pose
        record_kind_value = candidate.metadata["record_kind"]
        linked_object_instance_ids = self._coerce_optional_str_list(metadata.get("linked_object_instance_ids"))
        evidence_event_id = str(
            metadata.get("observation_event_id")
            or metadata.get("last_observation_event_id")
            or metadata.get("linked_observation_event_id")
            or metadata.get("verification_event_id")
            or ""
        ).strip()
        target_detection_evidence = self._coerce_optional_bool(metadata.get("target_detection_evidence")) is True
        has_detection_evidence_flag = self._coerce_optional_bool(metadata.get("has_detection_evidence")) is True
        matched_detection_count = self._coerce_optional_int(metadata.get("matched_detection_count"))
        detection_metadata = metadata.get("detection_metadata")
        if not isinstance(detection_metadata, dict):
            detection_metadata = {}
        detection_query_values = [
            metadata.get("target_search_query"),
            metadata.get("search_query"),
            metadata.get("query"),
            detection_metadata.get("query"),
            detection_metadata.get("target_query"),
            detection_metadata.get("target_search_query"),
            detection_metadata.get("open_vocabulary_query"),
        ]
        detection_query = next((str(value).strip() for value in detection_query_values if str(value or "").strip()), "")
        matched_labels = self._coerce_optional_str_list(metadata.get("matched_labels"))
        best_detection_label = str(metadata.get("best_detection_label") or "").strip()
        detection_label_values = [
            matched_labels,
            best_detection_label,
            metadata.get("detection_label"),
            metadata.get("target_label"),
            detection_metadata.get("matched_labels"),
            detection_metadata.get("best_detection_label"),
            detection_metadata.get("open_vocabulary_classes"),
        ]
        query_matched_fields = []
        if self._target_query_matches_any(normalized_query, detection_query_values):
            query_matched_fields.append("detection_query")
        if self._target_query_matches_any(normalized_query, detection_label_values):
            query_matched_fields.append("detection_label")
        explicit_query_match_keys = (
            "query_matched_detection_evidence",
            "detection_query_matched",
            "target_query_matched",
            "target_search_query_matched",
            "open_vocabulary_query_matched",
            "query_matched",
        )
        explicit_query_match_evidence = any(
            self._coerce_optional_bool(source.get(key)) is True
            for source in (metadata, detection_metadata)
            for key in explicit_query_match_keys
        )
        has_query_match_evidence = bool(normalized_query and (query_matched_fields or explicit_query_match_evidence))
        has_detection_signal = bool(
            target_detection_evidence
            or has_detection_evidence_flag
            or (matched_detection_count is not None and matched_detection_count > 0)
            or matched_labels
            or best_detection_label
            or detection_query
            or explicit_query_match_evidence
        )
        query_matched_detection_evidence = bool(has_query_match_evidence and has_detection_signal)
        query_matched_event_evidence = bool(evidence_event_id and has_query_match_evidence)
        projection_method = str(metadata.get("projection_method") or "")
        geometry_projection_method = str(metadata.get("geometry_projection_method") or "")
        geometry_projection_status = str(metadata.get("geometry_projection_status") or "")
        segmentation_projection_status = str(metadata.get("segmentation_projection_status") or "")
        has_projection_geometry_state = bool(
            projection_method
            or geometry_projection_method
            or geometry_projection_status
            or segmentation_projection_status
        )
        has_projection_evidence = bool(has_projection_geometry_state)
        target_frame_id = str(getattr(target_pose, "frame_id", "") or "") if target_pose is not None else ""
        accepted_map_frame_ids = sorted(self._target_navigation_map_frame_ids())
        target_pose_in_map_frame = self._target_pose_is_map_frame(target_frame_id)
        target_evidence_source = ""
        if query_matched_detection_evidence:
            target_evidence_source = "detection"
        elif query_matched_event_evidence:
            target_evidence_source = "observation_event"
        has_detection_evidence = bool(target_evidence_source)
        diagnostic = {
            "record_id": str(getattr(candidate, "record_id", "") or ""),
            "record_kind": record_kind_value,
            "target_name": str(getattr(candidate, "target_name", "") or ""),
            "map_name": getattr(candidate, "map_name", None),
            "topo_node_id": getattr(candidate, "topo_node_id", None),
            "semantic_region_id": getattr(candidate, "semantic_region_id", None),
            "distance_m": getattr(candidate, "distance_m", None),
            "verification_query": getattr(candidate, "verification_query", None),
            "has_goal_pose": target_pose is not None,
            "target_frame_id": target_frame_id,
            "target_pose_in_map_frame": target_pose_in_map_frame,
            "accepted_map_frame_ids": accepted_map_frame_ids,
            "camera_id": str(metadata.get("camera_id") or ""),
            "backend_type": str(metadata.get("backend_type") or metadata.get("backend") or "memory_service"),
            "backend": str(metadata.get("backend") or metadata.get("backend_type") or "memory_service"),
            "source_collection": candidate.source_collection,
            "candidate_id": candidate.candidate_id,
            "anchor_id": candidate.anchor_id,
            "candidate_score": candidate_score,
            "has_target_evidence": has_detection_evidence,
            "target_evidence_source": target_evidence_source,
            "evidence_record_kind": record_kind_value if record_kind_value in {"object_instance", "observation_event"} else "",
            "linked_object_instance_ids": linked_object_instance_ids,
            "evidence_event_id": evidence_event_id,
            "target_detection_evidence": target_detection_evidence,
            "has_detection_evidence_flag": has_detection_evidence_flag,
            "matched_detection_count": matched_detection_count,
            "detection_query": detection_query,
            "query_match_evidence_fields": query_matched_fields,
            "explicit_query_match_evidence": explicit_query_match_evidence,
            "query_matched_detection_evidence": query_matched_detection_evidence,
            "query_matched_event_evidence": query_matched_event_evidence,
            "projection_method": projection_method,
            "geometry_projection_method": geometry_projection_method,
            "projection_confidence": projection_confidence,
            "projection_confidence_present": projection_confidence_present,
            "projection_confidence_valid": projection_confidence_valid,
            "support_point_count": support_point_count,
            "has_projection_evidence": has_projection_evidence,
            "has_projection_geometry_state": has_projection_geometry_state,
            "projection_degradation_reason": str(metadata.get("projection_degradation_reason") or ""),
            "geometry_projection_status": geometry_projection_status,
            "segmentation_projection_status": segmentation_projection_status,
            "has_structured_segmentation": has_structured_segmentation,
            "segmentation_score": segmentation_score,
            "predicted_iou": predicted_iou,
            "segmentation_quality_score": segmentation_quality_score,
            "stability_score": stability_score,
            "coverage": coverage,
            "map_projection_coverage": map_projection_coverage,
            "depth_coverage": depth_coverage,
            "reprojection_error": reprojection_error,
            "segmentation_support_point_count": segmentation_support_point_count,
            "segmentation_quality": str(metadata.get("segmentation_quality") or ""),
            "segmentation_review_required": segmentation_review_required,
            "segmentation_risk_factors": segmentation_risk_factors,
            "segmentation_quality_reason": str(metadata.get("segmentation_quality_reason") or "").strip() or None,
            "segmentation_occlusion_flags": segmentation_occlusion_flags,
            "segmentation_truncation_flags": segmentation_truncation_flags,
            "navigable": navigable,
            "non_navigable_reason": str(metadata.get("non_navigable_reason") or "").strip() or None,
            "navigation_quality": navigation_quality,
            "navigation_quality_level": str(navigation_quality.get("level") or "").strip() or None,
            "recommended_action": str(navigation_quality.get("recommended_action") or "").strip() or None,
            "allowed_for_navigation": allowed_for_navigation,
            "rejection_reason": rejection_reason,
            "navigation_attempted": False,
        }
        for key in ("goal_spec", "resolution_summary", "score_breakdown", "resolution_decision",
                    "ranking_total_score", "failure_penalty", "failure_reason_codes", "llm_boundary", "coordinate_source"):
            if key in metadata:
                diagnostic[key] = metadata[key]
        return diagnostic

    def _target_candidate_score_from_diagnostic(self, diagnostic: Dict[str, Any]) -> Optional[float]:
        score = self._coerce_optional_float(diagnostic.get("candidate_score"))
        if score is not None:
            return score
        navigation_quality = diagnostic.get("navigation_quality")
        if isinstance(navigation_quality, dict):
            return self._coerce_optional_float(navigation_quality.get("confidence"))
        return None

    def _build_target_clarification(
        self,
        *,
        query: str,
        reason: str,
        candidate_diagnostics: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        candidates = [
            dict(item)
            for item in candidate_diagnostics
            if item.get("allowed_for_navigation") is True
        ]
        return {
            "action": "clarify",
            "reason": reason,
            "query": query,
            "message": "找到多个可导航候选，请补充更明确的目标描述或选择候选 record_id。",
            "candidates": candidates[:5],
        }

    def _select_low_ambiguity_target_candidates(
        self,
        *,
        query: str,
        candidates: List[Any],
        candidate_diagnostics: List[Dict[str, Any]],
    ) -> Tuple[List[Any], Optional[Dict[str, Any]]]:
        if not candidates:
            return candidates, None
        diagnostics_by_record_id = {str(item.get("record_id") or ""): item for item in candidate_diagnostics}
        eligible_candidates = []
        for candidate in candidates:
            diagnostic = diagnostics_by_record_id.get(str(getattr(candidate, "record_id", "") or ""), {})
            if diagnostic.get("allowed_for_navigation") is not True:
                continue
            if not diagnostic.get("has_goal_pose"):
                continue
            if diagnostic.get("segmentation_review_required"):
                continue
            eligible_candidates.append(candidate)
        if not eligible_candidates:
            return [], None
        if len(eligible_candidates) == 1:
            return [eligible_candidates[0]], None

        top_candidate = eligible_candidates[0]
        second_candidate = eligible_candidates[1]
        top_diagnostic = diagnostics_by_record_id.get(str(getattr(top_candidate, "record_id", "") or ""), {})
        second_diagnostic = diagnostics_by_record_id.get(str(getattr(second_candidate, "record_id", "") or ""), {})
        top_score = self._target_candidate_score_from_diagnostic(top_diagnostic)
        second_score = self._target_candidate_score_from_diagnostic(second_diagnostic)
        if top_score is None or second_score is None or (top_score - second_score) < 0.08:
            clarification = self._build_target_clarification(
                query=query,
                reason="ambiguous_navigation_candidates",
                candidate_diagnostics=candidate_diagnostics,
            )
            for item in candidate_diagnostics:
                if item.get("allowed_for_navigation") is True:
                    item["ambiguity_state"] = "needs_clarification"
                    item["clarification_reason"] = "ambiguous_navigation_candidates"
            return [], clarification
        return [top_candidate], None

    def _find_scene_object(self, query: str, perception_context):
        normalized_query = self._normalize_query(query)
        for item in perception_context.scene_summary.objects:
            if normalized_query == self._normalize_query(item.label):
                return item
        for item in perception_context.scene_summary.objects:
            candidates = {self._normalize_query(item.label)}
            candidates.update(self._normalize_query(camera_id) for camera_id in item.camera_ids)
            if normalized_query in candidates:
                return item
        if normalized_query and normalized_query in self._normalize_query(perception_context.scene_summary.headline):
            return perception_context.scene_summary.objects[0] if perception_context.scene_summary.objects else None
        return None


class _ExplorationSearchContext:
    """状态等待内只看新识别快照，正式记忆查询在退出探索后进行。"""

    def __init__(self, search, previous):
        self.search = search
        self.previous = previous
        self.sample = None

    def __getattr__(self, name):
        return getattr(self.search.ctx, name)

    def check(self):
        self.search.ctx.check()
        sample = self.search.runtime.latest(camera_id=self.search.camera_id,
                                            detector_backend_name=self.search.backend)
        if sample is not None and sample is not self.previous:
            self.previous = sample
            if self.search._build_target_detection_evidence(self.search.query, sample.context)["has_detection_evidence"]:
                self.sample = sample
                raise _SearchHit()


def find_target(platform, ctx, args):
    search = _TargetSearch(platform, ctx, args, "find_target")
    search.prepare_map()
    with search.acquired():
        ctx.progress(.05, "search_prepare", "准备查找目标。")
        sample, refresh = search.refresh(source="find_target_precheck")
        state = search.search_state(sample)
        if state["verification"].verified:
            ctx.progress(1.0, "completed", "目标已在当前场景中找到。")
            return search.current_result(state, refresh)
        result = search.navigate_candidates(state, allow_not_found=False)
        if result.get("action") != "clarify":
            ctx.progress(1.0, "completed", "已找到并复核目标。")
        result.setdefault("perception_refresh", refresh)
        return result


def inspect_target(platform, ctx, args):
    search = _TargetSearch(platform, ctx, args, "inspect_target")
    with search.acquired():
        ctx.progress(.05, "inspect_observe", "开始检查目标。")
        sample, _ = search.refresh(remember=False)
        matched = search._find_scene_object(search.query, sample.context)
        if matched is None and args.get("auto_navigate", False):
            ctx.progress(.3, "inspect_navigate", "当前画面未找到目标，尝试导航到命名位置。")
            target = platform.navigation_memory.resolve(search.query, camera_id=search.camera_id)
            pose = target.pose
            navigate_to_pose(platform, ctx, {"x": pose.x, "y": pose.y, "z": pose.z,
                "yaw_rad": pose.yaw, "frame_id": pose.frame_id, "motion_generation": args["motion_generation"]},
                region=target.region)
            sample, _ = search.refresh(frame_count=5, interval_sec=.12, remember=False)
            matched = search._find_scene_object(search.query, sample.context)
        if matched is None:
            raise GatewayError("未在当前场景中找到目标：" + search.query,
                               error_code="target_not_found", http_status=404)
        entry = remember_sample(platform, sample, title="检查结果：" + search.query,
            summary=sample.context.scene_summary.headline, tags=[search.query, matched.label],
            metadata={"source": "inspect_target"})
        ctx.progress(1.0, "completed", "目标检查完成。")
        return {"query": search.query, "matched_object": matched, "perception_context": sample.context,
                "semantic_memory": entry}


def explore_and_find_target(platform, ctx, args):
    search = _TargetSearch(platform, ctx, args, "explore_and_find_target")
    search.prepare_map(create_if_missing=True)
    with search.acquired():
        ctx.progress(.03, "search_prepare", "准备探索并查找目标。")
        sample, refresh = search.refresh(source="explore_find_precheck")
        state = search.search_state(sample)
        if state["verification"].verified:
            ctx.progress(1.0, "completed", "目标已在当前场景中找到。")
            return search.current_result(state, refresh)
        result = search.navigate_candidates(state, allow_not_found=True)
        if result is not None:
            ctx.progress(1.0, "completed", "已根据记忆候选找到目标。")
            return result
        while True:
            proxy = _ExplorationSearchContext(search, sample)
            exploration = None
            hit = False
            try:
                exploration = explore_area(platform, proxy, {key: value for key, value in args.items()
                    if key in {"map_name", "boundary_line_segments", "stop_conditions", "return_policy",
                        "max_navigation_retries", "blacklist_radius_m", "max_duration_sec", "save_map", "motion_generation"}})
            except _SearchHit:
                hit = True
                sample = search.runtime.complete_sample(proxy.sample)
                refresh = search.observe_sample(sample, source="explore_find_cycle")
            if not hit:
                sample, refresh = search.refresh(source="explore_find_final")
            state = search.search_state(sample)
            result = (search.current_result(state, refresh) if state["verification"].verified
                      else search.navigate_candidates(state, allow_not_found=True))
            if result is not None:
                if hit and args.get("save_map", True):
                    result["saved_map"] = save_map(platform, ctx, {"map_name": search.map_name})
                result["exploration_context"] = exploration
                ctx.progress(1.0, "completed", "探索中已找到并复核目标。")
                return result
            if not hit:
                raise GatewayError("探索已完成，但仍未找到目标：" + search.query,
                    error_code="target_not_found_after_exploration", http_status=404,
                    details={"map_name": search.map_name, "query": search.query,
                        "candidate_diagnostics": state["candidate_diagnostics"],
                        "rejected_candidate_ids": sorted(search.rejected)})
