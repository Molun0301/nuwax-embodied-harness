from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import logging
import math
from typing import List, Optional, Sequence, Tuple, TYPE_CHECKING

import cv2
import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import CostMap, OccupancyGrid

if TYPE_CHECKING:
    from services.navigation.config import ExplorationConfig
    from services.navigation.runtime import GridNavigationPlanner


LOGGER = logging.getLogger("nuwax_robot_bridge.navigation.frontier")

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
    ) -> List[FrontierCandidate]:
        """按收益排序返回可达前沿候选。"""

        if not bool(self.config.frontier_enabled):
            return []
        if (
            occupancy_grid.frame_id
            and current_pose.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, current_pose.frame_id)
        ):
            return []

        occupancy = np.asarray(occupancy_grid.data, dtype=np.int32).reshape(occupancy_grid.height, occupancy_grid.width)
        free_mask = occupancy == 0
        unknown_mask = occupancy < 0
        if not np.any(free_mask) or not np.any(unknown_mask):
            return []
        effective_cost_map = cost_map if self._same_grid_layout(occupancy_grid, cost_map) else None

        frontier_mask = self._build_frontier_mask(
            free_mask=free_mask,
            unknown_mask=unknown_mask,
            cost_map=effective_cost_map,
        )
        if not np.any(frontier_mask):
            return []

        clusters = self._cluster_frontiers(frontier_mask)
        if not clusters:
            return []

        candidates: List[FrontierCandidate] = []
        revisit_separation_m = float(self.config.frontier_revisit_separation_m)
        current_yaw = _yaw_from_pose(current_pose)
        max_radius_m = None if radius_m is None else max(0.1, float(radius_m))
        LOGGER.debug(
            "frontier 候选生成开始 current=(%.3f, %.3f, %.3f) frame=%s map_frame=%s grid=%sx%s resolution=%.3f origin=(%.3f, %.3f) costmap_frame=%s clusters=%d radius_m=%s attempted=%d",
            float(current_pose.position.x),
            float(current_pose.position.y),
            float(current_pose.position.z),
            current_pose.frame_id,
            occupancy_grid.frame_id,
            int(occupancy_grid.width),
            int(occupancy_grid.height),
            float(occupancy_grid.resolution_m),
            float(occupancy_grid.origin.position.x),
            float(occupancy_grid.origin.position.y),
            getattr(cost_map, "frame_id", None) if cost_map is not None else None,
            len(clusters),
            None if radius_m is None else float(radius_m),
            len(attempted_poses),
        )
        for cluster in clusters:
            if len(cluster) < int(self.config.frontier_min_cluster_cells):
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
            LOGGER.debug(
                "frontier 候选坐标生成 cell=(%d,%d) world=(%.3f, %.3f, %.3f) centroid=(%.3f, %.3f) frame=%s cluster_size=%d",
                int(best_cell[0]),
                int(best_cell[1]),
                float(goal_x),
                float(goal_y),
                float(center_pose.position.z),
                float(centroid_x),
                float(centroid_y),
                current_pose.frame_id,
                len(cluster),
            )
            if any(
                math.hypot(goal_x - attempted.position.x, goal_y - attempted.position.y) < revisit_separation_m
                for attempted in attempted_poses
            ):
                continue
            goal_distance_m = math.hypot(goal_x - current_pose.position.x, goal_y - current_pose.position.y)
            min_goal_distance_m = max(float(self.config.reach_distance_m), float(occupancy_grid.resolution_m) * 2.0)
            if goal_distance_m <= min_goal_distance_m:
                continue

            yaw_rad = math.atan2(goal_y - current_pose.position.y, goal_x - current_pose.position.x)
            goal_pose = Pose(
                frame_id=current_pose.frame_id,
                position=Vector3(x=goal_x, y=goal_y, z=center_pose.position.z),
                orientation=_quaternion_from_yaw(yaw_rad),
            )
            plan = planner.plan_preview(
                current_pose=current_pose,
                target_pose=goal_pose,
                occupancy_grid=occupancy_grid,
                cost_map=effective_cost_map,
            )
            if plan is None or plan.planning_mode != "goal":
                continue

            average_cost = self._cluster_average_cost(cluster=cluster, cost_map=effective_cost_map)
            heading_error = abs(_normalize_angle(yaw_rad - current_yaw))
            heading_bonus = max(0.0, math.cos(heading_error))
            info_gain_cells = len(cluster)
            score = (
                float(self.config.frontier_info_gain_weight) * float(info_gain_cells)
                - float(self.config.frontier_distance_weight) * float(plan.total_distance_m)
                - float(self.config.frontier_cost_weight) * (average_cost / 100.0)
                + float(self.config.frontier_heading_weight) * heading_bonus
            )
            candidates.append(
                FrontierCandidate(
                    pose=goal_pose,
                    score=score,
                    cluster_size=len(cluster),
                    info_gain_cells=info_gain_cells,
                    path_distance_m=float(plan.total_distance_m),
                    average_cost=average_cost,
                )
            )
            LOGGER.debug(
                "frontier 候选坐标可达 goal=(%.3f, %.3f, %.3f) yaw=%.3f frame=%s score=%.3f distance_m=%.3f average_cost=%.3f info_gain=%d",
                float(goal_pose.position.x),
                float(goal_pose.position.y),
                float(goal_pose.position.z),
                float(yaw_rad),
                goal_pose.frame_id,
                float(score),
                float(plan.total_distance_m),
                float(average_cost),
                int(info_gain_cells),
            )

        candidates.sort(
            key=lambda item: (
                -float(item.score),
                -int(item.info_gain_cells),
                float(item.path_distance_m),
            )
        )
        if candidates:
            best = candidates[0]
            LOGGER.debug(
                "frontier 候选生成完成 count=%d best=(%.3f, %.3f, %.3f) frame=%s score=%.3f distance_m=%.3f info_gain=%d",
                len(candidates),
                float(best.pose.position.x),
                float(best.pose.position.y),
                float(best.pose.position.z),
                best.pose.frame_id,
                float(best.score),
                float(best.path_distance_m),
                int(best.info_gain_cells),
            )
        else:
            LOGGER.debug("frontier 候选生成完成 count=0")
        return candidates

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
        if not frontier_mask.any():
            return []
        num_labels, labels, _stats, _ = cv2.connectedComponentsWithStats(
            frontier_mask.astype(np.uint8), connectivity=8
        )
        clusters: List[List[Cell2D]] = []
        for label_id in range(1, num_labels):  # 0 是背景
            ys, xs = np.where(labels == label_id)
            cluster: List[Cell2D] = [(int(y), int(x)) for y, x in zip(ys, xs)]
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
