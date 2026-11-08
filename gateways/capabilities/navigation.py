"""位姿导航、命名地点与找物任务共用导航执行入口。"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict
import logging
import math
import time
from itertools import chain
from typing import Any, Dict, List, Optional, Tuple

from contracts.spatial_memory import NavigationCandidate
from core import components
from core.capability import Capability
from core.errors import GatewayError
from core.target_query import coerce_optional_str_list, normalize_query, target_label_matches_query
from core.tasks import Cancelled
from gateways.capabilities.common import rpc_errors, schema
from gateways.capabilities.exploration import explore_area
from gateways.capabilities.maps import load_map, save_map, start_mapping
from gateways.capabilities.memory import remember_sample, save_sample_photo
from services.geometry.client import GeometryError, NavigationGoal
from services.geometry.state import GoalStatus, Pose
from services.navigation.places import map_coordinates

LOGGER = logging.getLogger(__name__)


@rpc_errors
def navigate_to_pose(platform, ctx, args, *, region=None):
    ctx.check()
    snapshot = platform.geometry_state.snapshot()
    if "z" in args:
        z = args["z"]
    elif snapshot.info.pipeline == "slam_2d":
        z = 0.0
    else:
        z = platform.geometry_state.wait_for_pose(ctx).z
    yaw = args.get("yaw_rad", 0.0)
    pose = Pose(args["x"], args["y"], z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2),
                args.get("frame_id", snapshot.info.map_frame_id), time.time_ns())
    try:
        goal_id = platform.geometry.set_goal(NavigationGoal(pose, args["motion_generation"],
            allow_reverse=args.get("allow_reverse"), position_tolerance_m=args.get("tolerance_position_m", 0.15),
            yaw_tolerance_rad=args.get("tolerance_yaw_rad", 0.3) if "yaw_rad" in args else math.pi),
            timeout_sec=ctx.remaining())
    except GeometryError as error:
        if error.code == "FAILED_PRECONDITION":
            raise GatewayError(error.message, error_code="not_localized", http_status=409) from error
        raise
    finished = False
    try:
        # 控制应答可先于订阅到达；先接到自己的目标，避免消费尚在途的加载/定位状态。
        first = platform.geometry_state.wait_for(lambda state: (
            state.navigation is not None and state.navigation.goal_id == goal_id
            or state.previous_navigation is not None and state.previous_navigation.goal_id == goal_id), ctx)
        initial = first.navigation if first.navigation is not None and first.navigation.goal_id == goal_id else first.previous_navigation
        last = None
        for navigation in chain((initial,), platform.geometry_state.follow_goal(goal_id, ctx)):
            if navigation is last:
                continue
            last = navigation
            ctx.progress(0.0, navigation.status.value, navigation.message,
                         goal_id=goal_id, remaining_distance_m=navigation.remaining_distance_m)
            if navigation.status == GoalStatus.SUCCEEDED:
                finished = True
                return {"goal_id": goal_id, "arrival": "target"}
            if navigation.status in {GoalStatus.CANCELLED, GoalStatus.REPLACED, GoalStatus.STOPPED}:
                finished = True
                raise Cancelled(navigation.message or "导航已被取消或更新的命令停止")
            if (region is not None and navigation.no_feasible_path
                    and platform.navigation_memory.contains(region, platform.geometry_state.pose())):
                finished = True
                platform.geometry.cancel_goal(goal_id, timeout_sec=ctx.remaining())
                return {"goal_id": goal_id, "arrival": "region"}
            if navigation.status == GoalStatus.FAILED:
                finished = True
                raise GatewayError(navigation.message, error_code="navigation_failed", http_status=409)
    finally:
        if not finished:
            try:
                platform.geometry.cancel_goal(goal_id)
            except GeometryError:
                LOGGER.exception("取消导航目标失败：%s", goal_id)


@rpc_errors
def navigate_to_place(platform, ctx, args):
    state = platform.geometry_state.snapshot()
    reference = platform.maps.read_meta(state.map_name).get("georeference")
    if reference is None:
        raise GatewayError("当前地图没有地理参考。", error_code="map_georeference_missing", http_status=409)
    place = platform.places.search(args["query_text"], timeout_sec=ctx.remaining())
    ctx.check()
    x, y, z = map_coordinates(place["lat"], place["lon"], reference)
    result = navigate_to_pose(platform, ctx, {"x": x, "y": y, "z": z,
        "tolerance_position_m": 0.35,
        "frame_id": reference["map_frame_id"], "motion_generation": args["motion_generation"]})
    return {**result, "place": place}


@rpc_errors
def cancel_navigation(platform, args):
    if components.status().get("geometry", {}).get("state") != "running":
        return {"running": False, "navigation": None, "message": "没有活动目标"}
    platform.geometry.cancel_goal(args.get("goal_id", ""))
    return navigation_state(platform, {})


def navigation_state(platform, args):
    state = platform.geometry_state.snapshot()
    return {"running": platform.geometry_state.connected,
        "navigation": None if state is None or state.navigation is None else asdict(state.navigation),
        "exploration": None if state is None or state.exploration is None else asdict(state.exploration)}


class _SearchHit(Exception):
    """只退出当前探索步骤，保留父任务继续核对目标。"""


class _TargetSearch:
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
        self.capability = capability
        self.attempted = []
        self.candidate_diagnostics = {}

    @contextmanager
    def acquired(self):
        self.runtime.acquire(self.reason, interval_sec=self.args.get("search_refresh_interval_sec", .8),
            camera_id=self.camera_id, detector_backend_name=self.backend, open_vocabulary_classes=(self.query,))
        try:
            yield
        except GatewayError as error:
            if self.capability in {"find_target", "explore_and_find_target"}:
                error.details.update(map_name=self.map_name, query=self.query,
                    attempted_candidate_ids=list(self.attempted),
                    candidate_diagnostics=list(self.candidate_diagnostics.values()),
                    rejected_candidate_ids=sorted(self.rejected))
            raise
        finally:
            self.runtime.release(self.reason)

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
                        "projection_method": evidence["projection_method"],
                        "geometry_projection_method": evidence["geometry_projection_method"],
                        "projection_confidence": evidence["projection_confidence"],
                        "support_point_count": evidence["support_point_count"],
                        "projection_degradation_reason": evidence["projection_degradation_reason"]})
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
        if goal_id is not None:
            result = self.platform.memory.record_arrival_verification(result, map_name=self.map_name)
        return result

    def search_state(self, sample):
        verification = self.verify(sample)
        candidates = tuple(candidate for candidate in self.platform.memory.resolve_navigation_candidates(
            self.query, camera_id=self.camera_id, map_name=self.map_name, limit=5, object_target=True)
            if candidate.record_id not in self.rejected)
        diagnostics = [self._build_target_candidate_diagnostic(candidate) for candidate in candidates]
        self.candidate_diagnostics.update((item["record_id"], item) for item in diagnostics)
        allowed = [candidate for candidate, diagnostic in zip(candidates, diagnostics) if diagnostic["allowed_for_navigation"]]
        selected, clarification = self._select_low_ambiguity_target_candidates(
            query=self.query, candidates=allowed, candidate_diagnostics=diagnostics)
        return {"verification": verification, "candidates": selected, "candidate_diagnostics": diagnostics,
                "clarification": clarification}

    def writeback(self, sample, verification, candidate, goal_id):
        candidate_id = candidate.candidate_id if candidate is not None else verification.candidate_id or ""
        record_id = candidate.record_id if candidate is not None else verification.metadata.get("record_id", "")
        record_kind = candidate.metadata["record_kind"] if candidate is not None else verification.metadata.get("record_kind", "")
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
            self.attempted.append(candidate.record_id)
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
                    "tolerance_position_m": 0.35,
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
            labels.extend(coerce_optional_str_list(attributes.get("aliases")))
            if any(target_label_matches_query(query, label) for label in labels):
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

    @staticmethod
    def _build_target_candidate_diagnostic(candidate: NavigationCandidate) -> Dict[str, Any]:
        metadata = candidate.metadata
        allowed = metadata["allowed_for_navigation"] is True and metadata["resolution_decision"] == "navigate"
        return {
            "record_id": candidate.record_id,
            "record_kind": metadata["record_kind"],
            **metadata["target_navigation_evidence"],
            "projection_method": metadata["projection_method"],
            "projection_confidence": metadata["projection_confidence"],
            "support_point_count": metadata["support_point_count"],
            "navigation_quality": metadata["navigation_quality"],
            "candidate_score": metadata["candidate_score"],
            "resolution_decision": metadata["resolution_decision"],
            "allowed_for_navigation": allowed,
            "rejection_reason": None if allowed else metadata["excluded_reason"] or metadata["resolution_decision"],
            "navigation_attempted": False,
        }

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
        candidates: List[NavigationCandidate],
        candidate_diagnostics: List[Dict[str, Any]],
    ) -> Tuple[List[NavigationCandidate], Optional[Dict[str, Any]]]:
        if len(candidates) <= 1:
            return candidates, None
        top_candidate, second_candidate = candidates[:2]
        top_score = top_candidate.metadata["candidate_score"]
        second_score = second_candidate.metadata["candidate_score"]
        if (top_score - second_score) < 0.08:
            clarification = self._build_target_clarification(
                query=query,
                reason="ambiguous_navigation_candidates",
                candidate_diagnostics=candidate_diagnostics,
            )
            return [], clarification
        return [top_candidate], None

    def _find_scene_object(self, query: str, perception_context):
        normalized_query = normalize_query(query)
        for item in perception_context.scene_summary.objects:
            if normalized_query == normalize_query(item.label):
                return item
        for item in perception_context.scene_summary.objects:
            candidates = {normalize_query(item.label)}
            candidates.update(normalize_query(camera_id) for camera_id in item.camera_ids)
            if normalized_query in candidates:
                return item
        if normalized_query and normalized_query in normalize_query(perception_context.scene_summary.headline):
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
                "tolerance_position_m": 0.35,
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


@rpc_errors
def navigate_to_named_location(platform, ctx, args):
    """命名地点沿用普通导航，并共享连拍复核、记录、写回和失败降权。"""
    target = platform.navigation_memory.resolve(args["target_name"], camera_id=args.get("camera_id"))
    pose = target.pose
    navigation = navigate_to_pose(platform, ctx, {"x": pose.x, "y": pose.y, "z": pose.z,
        "tolerance_position_m": 0.35,
        "yaw_rad": pose.yaw, "frame_id": pose.frame_id, "motion_generation": args["motion_generation"]},
        region=target.region)
    if not args.get("verify_on_arrival", True):
        return {**navigation, "target_name": target.target_name}
    search = _TargetSearch(platform, ctx, {**args, "query": target.target_name}, "navigate_to_named_location")
    with search.acquired():
        try:
            sample, refresh = search.refresh(frame_count=3, remember=False, source="named_arrival_verify")
            save_sample_photo(platform, sample)
            candidate = target.candidate if isinstance(target.candidate, NavigationCandidate) else None
            verification = platform.memory.verify_arrival_observation(target.target_name,
                perception_context=sample.context, current_pose=sample.pose,
                target_record=target.candidate, navigation_candidate=candidate,
                similarity_threshold=args.get("verify_similarity_threshold", .55),
                expected_map_name=search.map_name, goal_id=navigation["goal_id"])
            verification = platform.memory.record_arrival_verification(verification, map_name=search.map_name)
            writeback = search.writeback(sample, verification, candidate, navigation["goal_id"])
        except GatewayError as error:
            raise GatewayError(error.message, error_code="arrival_verification_failed", http_status=409,
                               details={"cause": error.to_payload()}) from error
        if not verification.verified:
            raise GatewayError(verification.reason or "到达核对失败", error_code="arrival_verification_failed",
                http_status=409, details=verification.model_dump(mode="json"))
        return {**navigation, "target_name": target.target_name,
                "verification": verification.model_dump(mode="json"),
                "observation_writeback": writeback, "perception_refresh": refresh}


TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
CAPABILITIES = [
    Capability("navigate_to_pose", "导航到当前地图坐标中的位姿。",
        schema({"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                "yaw_rad": {"type": "number"}, "frame_id": {"type": "string"},
                "tolerance_position_m": {"type": "number", "exclusiveMinimum": 0, "default": 0.15},
                "tolerance_yaw_rad": {"type": "number", "exclusiveMinimum": 0, "default": 0.3},
                "allow_reverse": {"type": "boolean"}, "timeout_sec": TIMEOUT}, ("x", "y")),
        "task", navigate_to_pose, needs=("nav_stack", "geometry"), motion=True),
    Capability("navigate_to_place", "通过开放街道地图地名查找并导航。",
        schema({"query_text": {"type": "string", "minLength": 1}, "timeout_sec": TIMEOUT}, ("query_text",)),
        "task", navigate_to_place, needs=("nav_stack", "geometry"), motion=True),
    Capability("navigate_to_named_location", "导航到记忆中的命名地点或区域，可在到达后核对。",
        schema({"target_name": {"type": "string"}, "camera_id": {"type": "string"},
                "verify_on_arrival": {"type": "boolean"},
                "verify_similarity_threshold": {"type": "number", "minimum": 0, "maximum": 1},
                "timeout_sec": TIMEOUT}, ("target_name",)),
        "task", navigate_to_named_location, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900),
    Capability("cancel_navigation", "取消指定目标或当前目标，不等待停稳。",
        schema({"goal_id": {"type": "string"}}), "action", cancel_navigation),
    Capability("get_navigation_state", "读取 Geometry 导航与探索状态。", schema(), "query", navigation_state),
]
