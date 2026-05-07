from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import CostMap, OccupancyGrid

if TYPE_CHECKING:
    from services.navigation.config import ExplorationConfig
    from services.navigation.runtime import GridNavigationPlanner


Cell2D = Tuple[int, int]


class ExplorationSafetyStatus(IntEnum):
    SAFE = 0
    WARNING = 1
    DANGER = 2


@dataclass
class ExplorationSafetyResult:
    """探索安全性检查结果。"""

    status: ExplorationSafetyStatus
    min_obstacle_distance_m: float
    recommended_speed_mps: float
    stop_required: bool


@dataclass
class FrontierCandidate:
    """前沿探索候选。"""

    pose: Pose
    score: float
    cluster_size: int
    info_gain_cells: int
    path_distance_m: float
    average_cost: float
    semantic_gain: float = 0.0
    semantic_gain_breakdown: Dict[str, float] = field(default_factory=dict)


class FrontierExplorer:
    """平台显式前沿检测与收益排序器。"""

    _NEIGHBORS: Tuple[Tuple[int, int], ...] = (
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    )

    def __init__(self, config: "ExplorationConfig") -> None:
        self.config = config
        self._last_scan_ranges: Optional[np.ndarray] = None
        self._exploration_active: bool = False
        self._last_filter_diagnostics: Dict[str, object] = {}

    def check_exploration_safety(
        self,
        current_pose: Pose,
        scan_ranges: Optional[np.ndarray] = None,
    ) -> ExplorationSafetyResult:
        """检测探索期间的安全性，返回是否需要停止或减速。"""
        if scan_ranges is None:
            if self._last_scan_ranges is None:
                return ExplorationSafetyResult(
                    status=ExplorationSafetyStatus.SAFE,
                    min_obstacle_distance_m=999.0,
                    recommended_speed_mps=0.35,
                    stop_required=False,
                )
            scan_ranges = self._last_scan_ranges

        self._last_scan_ranges = scan_ranges

        valid_mask = (scan_ranges >= 0.05) & (scan_ranges <= 25.0)
        if not np.any(valid_mask):
            return ExplorationSafetyResult(
                status=ExplorationSafetyStatus.SAFE,
                min_obstacle_distance_m=999.0,
                recommended_speed_mps=0.35,
                stop_required=False,
            )

        valid_ranges = scan_ranges[valid_mask]
        min_distance = float(np.min(valid_ranges))

        front_arc = scan_ranges[len(scan_ranges)//3:2*len(scan_ranges)//3]
        front_min = float(np.min(front_arc)) if len(front_arc) > 0 else min_distance

        if front_min <= 0.3 or min_distance <= 0.25:
            return ExplorationSafetyResult(
                status=ExplorationSafetyStatus.DANGER,
                min_obstacle_distance_m=min(front_min, min_distance),
                recommended_speed_mps=0.0,
                stop_required=True,
            )
        elif front_min <= 0.6 or min_distance <= 0.4:
            return ExplorationSafetyResult(
                status=ExplorationSafetyStatus.WARNING,
                min_obstacle_distance_m=min(front_min, min_distance),
                recommended_speed_mps=0.15,
                stop_required=False,
            )

        return ExplorationSafetyResult(
            status=ExplorationSafetyStatus.SAFE,
            min_obstacle_distance_m=min(front_min, min_distance),
            recommended_speed_mps=0.25,
            stop_required=False,
        )

    def start_exploration(self) -> None:
        """开始探索，必须先调用此方法。"""
        self._exploration_active = True
        self._last_scan_ranges = None

    def stop_exploration(self) -> None:
        """停止探索。"""
        self._exploration_active = False

    @property
    def is_exploration_active(self) -> bool:
        return self._exploration_active

    def get_last_filter_diagnostics(self) -> Dict[str, object]:
        return dict(self._last_filter_diagnostics)

    def select_candidates(
        self,
        *,
        current_pose: Pose,
        occupancy_grid: OccupancyGrid,
        cost_map: Optional[CostMap],
        planner: "GridNavigationPlanner",
        center_pose: Pose,
        radius_m: Optional[float],
        attempted_poses: Sequence[Pose],
        semantic_context: Optional[Dict[str, Any]] = None,
    ) -> List[FrontierCandidate]:
        """按收益排序返回可达前沿候选。"""

        diagnostics: Dict[str, object] = {
            "frontier_enabled": bool(self.config.frontier_enabled),
            "frame_mismatch": 0,
            "free_cells": 0,
            "unknown_cells": 0,
            "frontier_cells_before_cost_filter": 0,
            "frontier_cells": 0,
            "high_cost": 0,
            "cluster_total": 0,
            "cluster_too_small": 0,
            "cluster_out_of_radius": 0,
            "cluster_revisit": 0,
            "cluster_forbidden_zone": 0,
            "cluster_out_of_semantic_distance": 0,
            "plan_preview_failed": 0,
            "plan_preview_failed_by_reason": {},
            "selected_candidates": 0,
            "cost_map_used": False,
            "cost_map_layout_mismatch": False,
            "semantic_context_used": bool(isinstance(semantic_context, dict) and semantic_context),
        }

        def finish(candidates: List[FrontierCandidate]) -> List[FrontierCandidate]:
            diagnostics["selected_candidates"] = len(candidates)
            self._last_filter_diagnostics = dict(diagnostics)
            return candidates

        if not bool(self.config.frontier_enabled):
            return finish([])
        if (
            occupancy_grid.frame_id
            and current_pose.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, current_pose.frame_id)
        ):
            diagnostics["frame_mismatch"] = 1
            diagnostics["occupancy_frame_id"] = occupancy_grid.frame_id
            diagnostics["pose_frame_id"] = current_pose.frame_id
            return finish([])

        occupancy = np.asarray(occupancy_grid.data, dtype=np.int32).reshape(occupancy_grid.height, occupancy_grid.width)
        free_mask = occupancy == 0
        unknown_mask = occupancy < 0
        diagnostics["free_cells"] = int(np.count_nonzero(free_mask))
        diagnostics["unknown_cells"] = int(np.count_nonzero(unknown_mask))
        if not np.any(free_mask) or not np.any(unknown_mask):
            return finish([])
        effective_cost_map = cost_map if self._same_grid_layout(occupancy_grid, cost_map) else None
        diagnostics["cost_map_used"] = effective_cost_map is not None
        diagnostics["cost_map_layout_mismatch"] = cost_map is not None and effective_cost_map is None

        frontier_mask = self._build_frontier_mask(
            free_mask=free_mask,
            unknown_mask=unknown_mask,
            cost_map=None,
        )
        diagnostics["frontier_cells_before_cost_filter"] = int(np.count_nonzero(frontier_mask))
        if effective_cost_map is not None and int(effective_cost_map.width) == occupancy_grid.width and int(effective_cost_map.height) == occupancy_grid.height:
            costs = np.asarray(effective_cost_map.data, dtype=np.float32).reshape(occupancy_grid.height, occupancy_grid.width)
            high_cost_mask = frontier_mask & (costs > float(self.config.max_goal_cost))
            diagnostics["high_cost"] = int(np.count_nonzero(high_cost_mask))
            frontier_mask &= costs <= float(self.config.max_goal_cost)
        diagnostics["frontier_cells"] = int(np.count_nonzero(frontier_mask))
        if not np.any(frontier_mask):
            return finish([])

        clusters = self._cluster_frontiers(frontier_mask)
        diagnostics["cluster_total"] = len(clusters)
        if not clusters:
            return finish([])

        candidates: List[FrontierCandidate] = []
        revisit_separation_m = float(self.config.frontier_revisit_separation_m)
        current_yaw = _yaw_from_pose(current_pose)
        semantic_payload = semantic_context if isinstance(semantic_context, dict) else {}
        max_radius_m = None if radius_m is None else max(0.1, float(radius_m))
        semantic_max_distance_m = self._semantic_context_float(semantic_payload, "max_distance_m")
        if semantic_max_distance_m is None:
            semantic_max_distance_m = self._semantic_context_float(semantic_payload, "max_exploration_distance_m")
        plan_failures: Dict[str, int] = {}
        for cluster in clusters:
            if len(cluster) < int(self.config.frontier_min_cluster_cells):
                diagnostics["cluster_too_small"] = int(diagnostics["cluster_too_small"]) + 1
                continue
            cluster_points_world = [
                _cell_to_world(
                    row=row,
                    col=col,
                    origin_x=float(occupancy_grid.origin.position.x),
                    origin_y=float(occupancy_grid.origin.position.y),
                    resolution=float(occupancy_grid.resolution_m),
                )
                for row, col in cluster
            ]
            centroid_x = sum(point[0] for point in cluster_points_world) / float(len(cluster_points_world))
            centroid_y = sum(point[1] for point in cluster_points_world) / float(len(cluster_points_world))
            center_distance_m = math.hypot(
                centroid_x - center_pose.position.x,
                centroid_y - center_pose.position.y,
            )
            if max_radius_m is not None and center_distance_m > max_radius_m:
                diagnostics["cluster_out_of_radius"] = int(diagnostics["cluster_out_of_radius"]) + 1
                continue
            current_distance_m = math.hypot(centroid_x - current_pose.position.x, centroid_y - current_pose.position.y)
            if semantic_max_distance_m is not None and current_distance_m > max(0.1, semantic_max_distance_m):
                diagnostics["cluster_out_of_semantic_distance"] = int(diagnostics["cluster_out_of_semantic_distance"]) + 1
                continue

            best_cell = min(
                cluster,
                key=lambda cell: self._candidate_cell_score(
                    cell=cell,
                    centroid_x=centroid_x,
                    centroid_y=centroid_y,
                    occupancy_grid=occupancy_grid,
                    cost_map=effective_cost_map,
                ),
            )
            goal_x, goal_y = _cell_to_world(
                row=best_cell[0],
                col=best_cell[1],
                origin_x=float(occupancy_grid.origin.position.x),
                origin_y=float(occupancy_grid.origin.position.y),
                resolution=float(occupancy_grid.resolution_m),
            )
            if any(
                math.hypot(goal_x - attempted.position.x, goal_y - attempted.position.y) < revisit_separation_m
                for attempted in attempted_poses
            ):
                diagnostics["cluster_revisit"] = int(diagnostics["cluster_revisit"]) + 1
                continue

            yaw_rad = math.atan2(goal_y - current_pose.position.y, goal_x - current_pose.position.x)
            goal_pose = Pose(
                frame_id=current_pose.frame_id,
                position=Vector3(x=goal_x, y=goal_y, z=center_pose.position.z),
                orientation=_quaternion_from_yaw(yaw_rad),
            )
            if self._pose_in_forbidden_zone(goal_pose, semantic_payload):
                diagnostics["cluster_forbidden_zone"] = int(diagnostics["cluster_forbidden_zone"]) + 1
                continue
            planner_fn = getattr(planner, "plan", None)
            if callable(planner_fn):
                outcome = planner_fn(
                    current_pose=current_pose,
                    target_pose=goal_pose,
                    occupancy_grid=occupancy_grid,
                    cost_map=effective_cost_map,
                )
                plan = getattr(outcome, "plan", None)
                failure_reason = str(getattr(outcome, "reason", "plan_unavailable") or "plan_unavailable")
            else:
                plan = planner.plan_preview(
                    current_pose=current_pose,
                    target_pose=goal_pose,
                    occupancy_grid=occupancy_grid,
                    cost_map=effective_cost_map,
                )
                failure_reason = "plan_preview_unavailable"
            if plan is None or plan.planning_mode != "goal":
                if plan is not None and plan.planning_mode != "goal":
                    failure_reason = "planning_mode_%s" % plan.planning_mode
                diagnostics["plan_preview_failed"] = int(diagnostics["plan_preview_failed"]) + 1
                plan_failures[failure_reason] = plan_failures.get(failure_reason, 0) + 1
                continue

            average_cost = self._cluster_average_cost(cluster=cluster, cost_map=effective_cost_map)
            heading_error = abs(_normalize_angle(yaw_rad - current_yaw))
            heading_bonus = max(0.0, math.cos(heading_error))
            info_gain_cells = len(cluster)
            semantic_breakdown = self._semantic_gain_breakdown(
                goal_pose=goal_pose,
                current_pose=current_pose,
                center_pose=center_pose,
                info_gain_cells=info_gain_cells,
                path_distance_m=float(plan.total_distance_m),
                average_cost=average_cost,
                semantic_context=semantic_payload,
            )
            semantic_gain = float(semantic_breakdown.get("total", 0.0))
            score = (
                float(self.config.frontier_info_gain_weight) * float(info_gain_cells)
                - float(self.config.frontier_distance_weight) * float(plan.total_distance_m)
                - float(self.config.frontier_cost_weight) * (average_cost / 100.0)
                + float(self.config.frontier_heading_weight) * heading_bonus
                + float(getattr(self.config, "frontier_semantic_gain_weight", 0.0)) * semantic_gain
            )
            candidates.append(
                FrontierCandidate(
                    pose=goal_pose,
                    score=score,
                    cluster_size=len(cluster),
                    info_gain_cells=info_gain_cells,
                    path_distance_m=float(plan.total_distance_m),
                    average_cost=average_cost,
                    semantic_gain=semantic_gain,
                    semantic_gain_breakdown=semantic_breakdown,
                )
            )

        candidates.sort(
            key=lambda item: (
                -float(item.score),
                -int(item.info_gain_cells),
                float(item.path_distance_m),
            )
        )
        diagnostics["plan_preview_failed_by_reason"] = dict(plan_failures)
        return finish(candidates)


    def _semantic_gain_breakdown(
        self,
        *,
        goal_pose: Pose,
        current_pose: Pose,
        center_pose: Pose,
        info_gain_cells: int,
        path_distance_m: float,
        average_cost: float,
        semantic_context: Dict[str, Any],
    ) -> Dict[str, float]:
        min_gain_cells = max(1.0, float(getattr(self.config, "frontier_min_information_gain_cells", 1)))
        historical_unobserved = max(0.0, min(1.0, float(info_gain_cells) / (min_gain_cells * 4.0)))
        distance_score = max(0.0, min(1.0, 1.0 / (1.0 + max(0.0, float(path_distance_m)))))
        safety_score = max(0.0, min(1.0, 1.0 - (max(0.0, float(average_cost)) / 100.0)))
        target_terms = self._semantic_target_terms(semantic_context)
        nearby_labels = self._nearby_semantic_labels(goal_pose, semantic_context)
        target_category_prior = self._label_overlap_score(target_terms, nearby_labels)
        if target_terms and not nearby_labels:
            target_category_prior = 0.05
        area_type = self._area_type_score(target_terms, nearby_labels, semantic_context)
        historical_failure = self._historical_failure_penalty(goal_pose, semantic_context)
        forbidden_penalty = 1.0 if self._pose_in_forbidden_zone(goal_pose, semantic_context) else 0.0
        center_distance = math.hypot(
            goal_pose.position.x - center_pose.position.x,
            goal_pose.position.y - center_pose.position.y,
        )
        center_bias = max(0.0, min(1.0, 1.0 / (1.0 + center_distance)))
        total = (
            (0.24 * target_category_prior)
            + (0.16 * area_type)
            + (0.24 * historical_unobserved)
            + (0.14 * distance_score)
            + (0.16 * safety_score)
            + (0.06 * center_bias)
            - (0.34 * historical_failure)
            - (0.5 * forbidden_penalty)
        )
        return {
            "target_category_prior": round(target_category_prior, 6),
            "area_type": round(area_type, 6),
            "historical_unobserved": round(historical_unobserved, 6),
            "historical_failure_penalty": round(historical_failure, 6),
            "distance_score": round(distance_score, 6),
            "safety_score": round(safety_score, 6),
            "center_bias": round(center_bias, 6),
            "forbidden_zone_penalty": round(forbidden_penalty, 6),
            "total": round(max(-1.0, min(1.0, total)), 6),
        }

    def _semantic_context_float(self, semantic_context: Dict[str, Any], key: str) -> Optional[float]:
        value = semantic_context.get(key)
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) and number > 0.0 else None

    def _semantic_target_terms(self, semantic_context: Dict[str, Any]) -> List[str]:
        raw_terms: List[str] = []
        for key in ("target_category", "target_name", "query", "search_query", "target_kind"):
            value = semantic_context.get(key)
            if isinstance(value, str) and value.strip():
                raw_terms.append(value)
        for key in ("target_aliases", "target_terms", "semantic_labels"):
            values = semantic_context.get(key)
            if isinstance(values, (list, tuple, set, frozenset)):
                raw_terms.extend(str(item) for item in values if str(item or "").strip())
        return self._normalize_terms(raw_terms)

    def _nearby_semantic_labels(self, goal_pose: Pose, semantic_context: Dict[str, Any]) -> List[str]:
        labels: List[str] = []
        for key in ("scene_spaces", "semantic_regions", "area_priors", "nearby_areas"):
            values = semantic_context.get(key)
            if not isinstance(values, (list, tuple)):
                continue
            for item in values:
                if not isinstance(item, dict):
                    continue
                pose_xy = self._pose_xy_from_mapping(item)
                radius_m = self._semantic_context_float(item, "radius_m") or self._semantic_context_float(item, "max_distance_m") or 6.0
                if pose_xy is not None:
                    distance = math.hypot(goal_pose.position.x - pose_xy[0], goal_pose.position.y - pose_xy[1])
                    if distance > radius_m:
                        continue
                for label_key in ("label", "primary_label", "name", "region_type", "area_type"):
                    value = item.get(label_key)
                    if isinstance(value, str) and value.strip():
                        labels.append(value)
                for list_key in ("aliases", "semantic_labels", "candidate_labels", "tags"):
                    value = item.get(list_key)
                    if isinstance(value, (list, tuple, set, frozenset)):
                        for label in value:
                            if isinstance(label, dict):
                                label = label.get("label") or label.get("name")
                            if str(label or "").strip():
                                labels.append(str(label))
        return self._normalize_terms(labels)

    def _area_type_score(self, target_terms: Sequence[str], nearby_labels: Sequence[str], semantic_context: Dict[str, Any]) -> float:
        if not nearby_labels:
            return 0.0
        area_terms = self._normalize_terms(
            [
                str(semantic_context.get("expected_area_type") or ""),
                str(semantic_context.get("target_kind") or ""),
                *target_terms,
            ]
        )
        return max(self._label_overlap_score(area_terms, nearby_labels), 0.15 if nearby_labels else 0.0)

    def _label_overlap_score(self, left: Sequence[str], right: Sequence[str]) -> float:
        if not left or not right:
            return 0.0
        best = 0.0
        for left_item in left:
            for right_item in right:
                if not left_item or not right_item:
                    continue
                if left_item == right_item:
                    best = max(best, 1.0)
                elif left_item in right_item or right_item in left_item:
                    best = max(best, 0.72)
                else:
                    left_tokens = set(left_item.replace("_", " ").split())
                    right_tokens = set(right_item.replace("_", " ").split())
                    if left_tokens and right_tokens:
                        best = max(best, len(left_tokens & right_tokens) / float(len(left_tokens | right_tokens)))
        return max(0.0, min(1.0, best))

    def _historical_failure_penalty(self, goal_pose: Pose, semantic_context: Dict[str, Any]) -> float:
        failure_items: List[Any] = []
        for key in ("failed_poses", "failed_frontier_poses", "negative_observations", "rejected_frontiers"):
            value = semantic_context.get(key)
            if isinstance(value, (list, tuple)):
                failure_items.extend(value)
        if not failure_items:
            return 0.0
        radius_m = self._semantic_context_float(semantic_context, "failure_radius_m") or 2.0
        penalty = 0.0
        for item in failure_items:
            xy = self._pose_xy_from_mapping(item)
            if xy is None:
                continue
            distance = math.hypot(goal_pose.position.x - xy[0], goal_pose.position.y - xy[1])
            if distance <= radius_m:
                penalty = max(penalty, 1.0 - (distance / max(0.01, radius_m)))
        return max(0.0, min(1.0, penalty))

    def _pose_in_forbidden_zone(self, pose: Pose, semantic_context: Dict[str, Any]) -> bool:
        zones = semantic_context.get("forbidden_zones")
        if not isinstance(zones, (list, tuple)):
            return False
        for zone in zones:
            if not isinstance(zone, dict):
                continue
            xy = self._pose_xy_from_mapping(zone)
            radius_m = self._semantic_context_float(zone, "radius_m") or self._semantic_context_float(zone, "distance_m")
            if xy is not None and radius_m is not None:
                if math.hypot(pose.position.x - xy[0], pose.position.y - xy[1]) <= radius_m:
                    return True
            min_x = self._semantic_context_float(zone, "min_x")
            max_x = self._semantic_context_float(zone, "max_x")
            min_y = self._semantic_context_float(zone, "min_y")
            max_y = self._semantic_context_float(zone, "max_y")
            if None not in (min_x, max_x, min_y, max_y):
                if min(min_x, max_x) <= pose.position.x <= max(min_x, max_x) and min(min_y, max_y) <= pose.position.y <= max(min_y, max_y):
                    return True
        return False

    def _pose_xy_from_mapping(self, value: Any) -> Optional[Tuple[float, float]]:
        if isinstance(value, Pose):
            return (float(value.position.x), float(value.position.y))
        if not isinstance(value, dict):
            return None
        for key in ("pose", "target_pose", "centroid", "center_pose", "boundary"):
            nested = value.get(key)
            if isinstance(nested, dict):
                if key == "boundary" and isinstance(nested.get("centroid"), dict):
                    nested = nested.get("centroid")
                xy = self._pose_xy_from_mapping(nested)
                if xy is not None:
                    return xy
        position = value.get("position")
        if isinstance(position, dict):
            try:
                return (float(position.get("x")), float(position.get("y")))
            except (TypeError, ValueError):
                return None
        coordinates = value.get("coordinates")
        if isinstance(coordinates, (list, tuple)) and len(coordinates) >= 2:
            try:
                return (float(coordinates[0]), float(coordinates[1]))
            except (TypeError, ValueError):
                return None
        try:
            return (float(value.get("x")), float(value.get("y")))
        except (TypeError, ValueError):
            return None

    def _normalize_terms(self, values: Sequence[str]) -> List[str]:
        normalized: List[str] = []
        for value in values:
            item = str(value or "").strip().lower().replace("-", "_")
            if item and item not in normalized:
                normalized.append(item)
        return normalized

    def _same_grid_layout(self, occupancy_grid: OccupancyGrid, cost_map: Optional[CostMap]) -> bool:
        if cost_map is None:
            return False
        if (
            occupancy_grid.frame_id
            and cost_map.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, cost_map.frame_id)
        ):
            return False
        return (
            int(occupancy_grid.width) == int(cost_map.width)
            and int(occupancy_grid.height) == int(cost_map.height)
            and abs(float(occupancy_grid.resolution_m) - float(cost_map.resolution_m)) <= 1e-6
            and abs(float(occupancy_grid.origin.position.x) - float(cost_map.origin.position.x)) <= 1e-6
            and abs(float(occupancy_grid.origin.position.y) - float(cost_map.origin.position.y)) <= 1e-6
        )

    def count_known_cells(self, occupancy_grid: Optional[OccupancyGrid]) -> int:
        """统计当前地图已知栅格数量。"""

        if occupancy_grid is None:
            return 0
        return int(sum(1 for value in occupancy_grid.data if int(value) >= 0))

    def _build_frontier_mask(
        self,
        *,
        free_mask: np.ndarray,
        unknown_mask: np.ndarray,
        cost_map: Optional[CostMap],
    ) -> np.ndarray:
        height, width = free_mask.shape
        padded_unknown = np.pad(unknown_mask.astype(np.uint8), 1, constant_values=0)
        unknown_neighbors = np.zeros((height, width), dtype=np.uint8)
        for row_offset, col_offset in self._NEIGHBORS:
            unknown_neighbors += padded_unknown[
                1 + row_offset : 1 + row_offset + height,
                1 + col_offset : 1 + col_offset + width,
            ]

        frontier_mask = free_mask & (unknown_neighbors >= int(self.config.frontier_min_unknown_neighbors))
        if cost_map is not None and int(cost_map.width) == width and int(cost_map.height) == height:
            costs = np.asarray(cost_map.data, dtype=np.float32).reshape(height, width)
            frontier_mask &= costs <= float(self.config.max_goal_cost)
        return frontier_mask

    def _cluster_frontiers(self, frontier_mask: np.ndarray) -> List[List[Cell2D]]:
        height, width = frontier_mask.shape
        visited = np.zeros((height, width), dtype=np.uint8)
        clusters: List[List[Cell2D]] = []
        for row in range(height):
            for col in range(width):
                if not frontier_mask[row, col] or visited[row, col] == 1:
                    continue
                cluster: List[Cell2D] = []
                queue = [(row, col)]
                visited[row, col] = 1
                while queue:
                    current_row, current_col = queue.pop()
                    cluster.append((current_row, current_col))
                    for row_offset, col_offset in self._NEIGHBORS:
                        next_row = current_row + row_offset
                        next_col = current_col + col_offset
                        if next_row < 0 or next_row >= height or next_col < 0 or next_col >= width:
                            continue
                        if visited[next_row, next_col] == 1 or not frontier_mask[next_row, next_col]:
                            continue
                        visited[next_row, next_col] = 1
                        queue.append((next_row, next_col))
                clusters.append(cluster)
        return clusters

    def _candidate_cell_score(
        self,
        *,
        cell: Cell2D,
        centroid_x: float,
        centroid_y: float,
        occupancy_grid: OccupancyGrid,
        cost_map: Optional[CostMap],
    ) -> float:
        world_x, world_y = _cell_to_world(
            row=cell[0],
            col=cell[1],
            origin_x=float(occupancy_grid.origin.position.x),
            origin_y=float(occupancy_grid.origin.position.y),
            resolution=float(occupancy_grid.resolution_m),
        )
        centroid_distance = math.hypot(world_x - centroid_x, world_y - centroid_y)
        cell_cost = 0.0
        if cost_map is not None and int(cost_map.width) == int(occupancy_grid.width) and int(cost_map.height) == int(occupancy_grid.height):
            index = cell[0] * int(cost_map.width) + cell[1]
            if 0 <= index < len(cost_map.data):
                cell_cost = float(cost_map.data[index])
        return centroid_distance + (cell_cost / 100.0)

    def _cluster_average_cost(self, *, cluster: Sequence[Cell2D], cost_map: Optional[CostMap]) -> float:
        if cost_map is None:
            return 0.0
        costs: List[float] = []
        width = int(cost_map.width)
        for row, col in cluster:
            index = row * width + col
            if 0 <= index < len(cost_map.data):
                costs.append(float(cost_map.data[index]))
        if not costs:
            return 0.0
        return float(sum(costs) / float(len(costs)))


def _cell_to_world(*, row: int, col: int, origin_x: float, origin_y: float, resolution: float) -> Tuple[float, float]:
    return (
        origin_x + (float(col) + 0.5) * resolution,
        origin_y + (float(row) + 0.5) * resolution,
    )


def _yaw_from_pose(pose: Pose) -> float:
    rotation = pose.orientation
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    half_yaw = yaw_rad * 0.5
    return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))



__all__ = [
    "ExplorationSafetyResult",
    "ExplorationSafetyStatus",
    "FrontierCandidate",
    "FrontierExplorer",
]
