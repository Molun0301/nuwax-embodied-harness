from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import List, Optional, Sequence, Tuple, TYPE_CHECKING

import cv2
import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.maps import CostMap, OccupancyGrid
from services.navigation.path_tools import resample_waypoints

if TYPE_CHECKING:
    from services.navigation.config import NavigationBackendConfig


Point2D = Tuple[float, float]
Cell2D = Tuple[int, int]


@dataclass
class GridPlan:
    """二维网格路径规划结果。"""

    frame_id: str
    waypoints: List[Point2D]
    planning_target: Point2D
    total_distance_m: float
    source_map: str
    planning_mode: str
    goal_in_map: bool
    target_adjusted: bool


@dataclass
class _PlanningOutcome:
    """路径规划内部结果。"""

    plan: Optional[GridPlan]
    message: str
    reason: str
    can_retry: bool = False


@dataclass
class _PlanningMap:
    """统一后的规划栅格。"""

    frame_id: str
    width: int
    height: int
    resolution_m: float
    origin_x: float
    origin_y: float
    blocked: np.ndarray
    traversal_cost: np.ndarray
    source_map: str

    def contains_world(self, x: float, y: float, *, padding_m: float = 0.0) -> bool:
        min_x = self.origin_x + max(0.0, float(padding_m))
        min_y = self.origin_y + max(0.0, float(padding_m))
        max_x = self.origin_x + self.width * self.resolution_m - max(0.0, float(padding_m))
        max_y = self.origin_y + self.height * self.resolution_m - max(0.0, float(padding_m))
        return min_x <= x < max_x and min_y <= y < max_y

    def world_to_cell(self, x: float, y: float) -> Optional[Cell2D]:
        col = int((x - self.origin_x) / self.resolution_m)
        row = int((y - self.origin_y) / self.resolution_m)
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None
        return row, col

    def cell_to_world(self, row: int, col: int) -> Point2D:
        return (
            self.origin_x + (float(col) + 0.5) * self.resolution_m,
            self.origin_y + (float(row) + 0.5) * self.resolution_m,
        )

    def nearest_traversable(self, cell: Cell2D, *, max_radius_cells: int) -> Optional[Cell2D]:
        row, col = cell
        if 0 <= row < self.height and 0 <= col < self.width and not bool(self.blocked[row, col]):
            return row, col

        best_cell: Optional[Cell2D] = None
        best_score = float("inf")
        for radius in range(1, max(1, max_radius_cells) + 1):
            for next_row in range(max(0, row - radius), min(self.height, row + radius + 1)):
                for next_col in range(max(0, col - radius), min(self.width, col + radius + 1)):
                    if self.blocked[next_row, next_col]:
                        continue
                    score = abs(next_row - row) + abs(next_col - col)
                    if score < best_score:
                        best_score = float(score)
                        best_cell = (next_row, next_col)
            if best_cell is not None:
                return best_cell
        return None

    def line_is_clear(self, start_cell: Cell2D, end_cell: Cell2D) -> bool:
        for row, col in _bresenham_cells(start_cell, end_cell):
            if row < 0 or row >= self.height or col < 0 or col >= self.width:
                return False
            if bool(self.blocked[row, col]):
                return False
        return True

    def line_is_clear_world(self, start_point: Point2D, end_point: Point2D) -> bool:
        start_cell = self.world_to_cell(*start_point)
        end_cell = self.world_to_cell(*end_point)
        if start_cell is None or end_cell is None:
            return False
        return self.line_is_clear(start_cell, end_cell)


class GridNavigationPlanner:
    """二维网格路径规划器。

    仅用于 frontier 可达性评估和路径预览，不再作为正式导航执行器。
    """

    _NEIGHBORS: Tuple[Tuple[int, int, float], ...] = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    def __init__(self, config: "NavigationBackendConfig") -> None:
        self.config = config

    def plan(
        self,
        *,
        current_pose: Pose,
        target_pose: Pose,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
    ) -> _PlanningOutcome:
        if (
            current_pose.frame_id
            and target_pose.frame_id
            and not frame_ids_semantically_equal(current_pose.frame_id, target_pose.frame_id)
        ):
            return _PlanningOutcome(
                plan=None,
                message=(
                    f"目标坐标系 {target_pose.frame_id} 与当前坐标系 {current_pose.frame_id} 不一致，"
                    "当前导航后端不做运行时坐标转换。"
                ),
                reason="frame_mismatch",
            )

        planning_map = self._build_planning_map(
            current_frame_id=current_pose.frame_id,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        if planning_map is None:
            return _PlanningOutcome(
                plan=None,
                message="当前代价地图未就绪，导航退回直驱跟踪模式。",
                reason="map_unavailable",
                can_retry=True,
            )

        start_cell = planning_map.world_to_cell(current_pose.position.x, current_pose.position.y)
        if start_cell is None:
            return _PlanningOutcome(
                plan=None,
                message="当前位姿落在规划地图外，等待局部地图刷新。",
                reason="start_out_of_bounds",
                can_retry=True,
            )
        start_cell = planning_map.nearest_traversable(
            start_cell,
            max_radius_cells=self._search_radius_cells(planning_map),
        )
        if start_cell is None:
            return _PlanningOutcome(
                plan=None,
                message="当前机器人周围没有可通行栅格，无法规划路径。",
                reason="start_blocked",
                can_retry=True,
            )

        target_point, planning_mode, goal_in_map = self._select_planning_target(
            planning_map=planning_map,
            start_point=(current_pose.position.x, current_pose.position.y),
            goal_point=(target_pose.position.x, target_pose.position.y),
        )
        if target_point is None:
            return _PlanningOutcome(
                plan=None,
                message="目标点既不在当前地图内，也无法生成局部地平线子目标。",
                reason="no_horizon_target",
                can_retry=True,
            )

        target_cell = planning_map.world_to_cell(*target_point)
        if target_cell is None:
            return _PlanningOutcome(
                plan=None,
                message="规划子目标落在地图外，等待地图刷新。",
                reason="target_out_of_bounds",
                can_retry=True,
            )
        adjusted_target_cell = planning_map.nearest_traversable(
            target_cell,
            max_radius_cells=self._search_radius_cells(planning_map),
        )
        if adjusted_target_cell is None:
            return _PlanningOutcome(
                plan=None,
                message="目标附近没有可通行栅格，当前无法到达。",
                reason="target_blocked",
                can_retry=True,
            )

        cell_path = self._a_star(
            planning_map=planning_map,
            start_cell=start_cell,
            goal_cell=adjusted_target_cell,
        )
        if not cell_path:
            return _PlanningOutcome(
                plan=None,
                message="当前局部地图中无法生成可执行路径。",
                reason="path_not_found",
                can_retry=True,
            )

        simplified_cells = self._simplify_path_cells(planning_map=planning_map, cell_path=cell_path)
        waypoints = [planning_map.cell_to_world(row, col) for row, col in simplified_cells]
        final_target_point = planning_map.cell_to_world(*adjusted_target_cell)
        if math.hypot(final_target_point[0] - target_point[0], final_target_point[1] - target_point[1]) <= planning_map.resolution_m:
            final_target_point = target_point
        if not waypoints or _point_distance(waypoints[-1], final_target_point) > max(0.05, planning_map.resolution_m * 0.5):
            waypoints.append(final_target_point)
        waypoints = self._deduplicate_waypoints(waypoints)
        resample_spacing_m = float(getattr(self.config, "path_resample_spacing_m", 0.0))
        if resample_spacing_m > 1e-6 and len(waypoints) >= 2:
            waypoints = resample_waypoints(
                waypoints,
                spacing_m=resample_spacing_m,
                smoothing_window=int(getattr(self.config, "path_smoothing_window", 0)),
            )
            if not waypoints or _point_distance(waypoints[-1], final_target_point) > max(0.05, planning_map.resolution_m * 0.5):
                waypoints.append(final_target_point)
            waypoints = self._deduplicate_waypoints(waypoints)
        total_distance_m = _polyline_length(waypoints)
        return _PlanningOutcome(
            plan=GridPlan(
                frame_id=planning_map.frame_id,
                waypoints=waypoints,
                planning_target=final_target_point,
                total_distance_m=total_distance_m,
                source_map=planning_map.source_map,
                planning_mode=planning_mode,
                goal_in_map=goal_in_map,
                target_adjusted=adjusted_target_cell != target_cell,
            ),
            message="规划完成。",
            reason="planned",
        )

    def plan_preview(
        self,
        *,
        current_pose: Pose,
        target_pose: Pose,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
    ) -> Optional[GridPlan]:
        outcome = self.plan(
            current_pose=current_pose,
            target_pose=target_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        return outcome.plan

    def _build_planning_map(
        self,
        *,
        current_frame_id: str,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
    ) -> Optional[_PlanningMap]:
        compatible_cost = cost_map if self._is_map_compatible(frame_id=current_frame_id, map_like=cost_map) else None
        compatible_occupancy = occupancy_grid if self._is_map_compatible(frame_id=current_frame_id, map_like=occupancy_grid) else None
        if (
            current_frame_id
            and compatible_cost is None
            and compatible_occupancy is None
            and (cost_map is not None or occupancy_grid is not None)
        ):
            return None

        primary = compatible_cost or compatible_occupancy or cost_map or occupancy_grid
        if primary is None:
            return None

        width = int(primary.width)
        height = int(primary.height)
        resolution_m = float(primary.resolution_m)
        frame_id = str(primary.frame_id or current_frame_id)
        origin_x = float(primary.origin.position.x)
        origin_y = float(primary.origin.position.y)
        traversal_cost = np.zeros((height, width), dtype=np.float32)
        source_labels: List[str] = []

        if compatible_cost is not None and self._same_grid_layout(primary, compatible_cost):
            traversal_cost = np.asarray(compatible_cost.data, dtype=np.float32).reshape(height, width).copy()
            source_labels.append("cost_map")

        blocked = np.zeros((height, width), dtype=bool)
        if compatible_occupancy is not None and self._same_grid_layout(primary, compatible_occupancy):
            occupancy = np.asarray(compatible_occupancy.data, dtype=np.int32).reshape(height, width)
            blocked |= occupancy >= int(self.config.occupancy_lethal_threshold)
            unknown_mask = occupancy < 0
            traversal_cost = np.maximum(traversal_cost, unknown_mask.astype(np.float32) * float(self.config.unknown_cell_cost))
            source_labels.append("occupancy_grid")

        blocked |= traversal_cost >= float(self.config.lethal_cost_threshold)
        inflated_blocked = self._inflate_blocked_mask(
            blocked=blocked,
            resolution_m=resolution_m,
        )
        traversal_cost[inflated_blocked] = 100.0

        if compatible_cost is None and compatible_occupancy is None:
            if cost_map is None and occupancy_grid is None:
                return None
            source_labels.append("best_effort_map")
        source_map = "+".join(source_labels) if source_labels else "unknown_map"
        return _PlanningMap(
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution_m,
            origin_x=origin_x,
            origin_y=origin_y,
            blocked=inflated_blocked,
            traversal_cost=traversal_cost,
            source_map=source_map,
        )

    def _is_map_compatible(self, *, frame_id: str, map_like) -> bool:
        if map_like is None:
            return False
        map_frame = str(getattr(map_like, "frame_id", "") or "")
        if map_frame and frame_id and not frame_ids_semantically_equal(map_frame, frame_id):
            return False
        return True

    def _same_grid_layout(self, primary, candidate) -> bool:
        return (
            int(primary.width) == int(candidate.width)
            and int(primary.height) == int(candidate.height)
            and abs(float(primary.resolution_m) - float(candidate.resolution_m)) <= 1e-6
            and abs(float(primary.origin.position.x) - float(candidate.origin.position.x)) <= 1e-6
            and abs(float(primary.origin.position.y) - float(candidate.origin.position.y)) <= 1e-6
        )

    def _inflate_blocked_mask(self, *, blocked: np.ndarray, resolution_m: float) -> np.ndarray:
        radius_m = max(0.0, float(self.config.planner_inflation_radius_m))
        if radius_m <= 1e-6 or not np.any(blocked):
            return blocked.copy()
        radius_cells = max(1, int(math.ceil(radius_m / max(1e-6, resolution_m))))
        kernel_size = 2 * radius_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        return cv2.dilate(blocked.astype(np.uint8), kernel).astype(bool)

    def _search_radius_cells(self, planning_map: _PlanningMap) -> int:
        search_radius_m = max(
            float(self.config.planner_inflation_radius_m) * 2.0,
            float(self.config.planning_horizon_margin_m),
            planning_map.resolution_m * 2.0,
        )
        return max(1, int(math.ceil(search_radius_m / max(planning_map.resolution_m, 1e-6))))

    def _select_planning_target(
        self,
        *,
        planning_map: _PlanningMap,
        start_point: Point2D,
        goal_point: Point2D,
    ) -> Tuple[Optional[Point2D], str, bool]:
        goal_padding_m = float(self.config.planning_horizon_margin_m)
        if planning_map.contains_world(*goal_point, padding_m=goal_padding_m):
            return goal_point, "goal", True
        if planning_map.contains_world(*goal_point, padding_m=0.0):
            return goal_point, "goal_edge", True

        goal_distance_m = _point_distance(start_point, goal_point)
        if goal_distance_m <= 1e-6:
            return goal_point, "goal", True

        max_distance_m = min(
            goal_distance_m,
            math.hypot(planning_map.width * planning_map.resolution_m, planning_map.height * planning_map.resolution_m),
        )
        step_m = max(planning_map.resolution_m * 2.0, 0.25)
        dx = goal_point[0] - start_point[0]
        dy = goal_point[1] - start_point[1]
        direction_x = dx / goal_distance_m
        direction_y = dy / goal_distance_m
        fallback_candidate: Optional[Point2D] = None

        sampled_distance = max_distance_m
        while sampled_distance > step_m:
            candidate = (
                start_point[0] + direction_x * sampled_distance,
                start_point[1] + direction_y * sampled_distance,
            )
            if planning_map.contains_world(*candidate, padding_m=goal_padding_m):
                return candidate, "local_horizon", False
            if fallback_candidate is None and planning_map.contains_world(*candidate, padding_m=0.0):
                fallback_candidate = candidate
            sampled_distance -= step_m
        if fallback_candidate is not None:
            return fallback_candidate, "local_horizon_edge", False
        return None, "local_horizon", False

    def _a_star(
        self,
        *,
        planning_map: _PlanningMap,
        start_cell: Cell2D,
        goal_cell: Cell2D,
    ) -> List[Cell2D]:
        if start_cell == goal_cell:
            return [start_cell]

        width = planning_map.width
        height = planning_map.height
        goal_row, goal_col = goal_cell
        total_cells = width * height
        g_score = np.full(total_cells, np.inf, dtype=np.float64)
        came_from = np.full(total_cells, -1, dtype=np.int64)
        closed = np.zeros(total_cells, dtype=bool)

        start_index = start_cell[0] * width + start_cell[1]
        goal_index = goal_row * width + goal_col
        g_score[start_index] = 0.0
        open_heap: List[Tuple[float, int]] = [
            (self._heuristic(start_cell, goal_cell, planning_map.resolution_m), start_index)
        ]

        while open_heap:
            _priority, current_index = heapq.heappop(open_heap)
            if closed[current_index]:
                continue
            if current_index == goal_index:
                return self._reconstruct_path(came_from=came_from, width=width, current_index=current_index)
            closed[current_index] = True
            row = current_index // width
            col = current_index % width

            for row_offset, col_offset, step_cost in self._NEIGHBORS:
                next_row = row + row_offset
                next_col = col + col_offset
                if next_row < 0 or next_row >= height or next_col < 0 or next_col >= width:
                    continue
                if bool(planning_map.blocked[next_row, next_col]):
                    continue
                next_index = next_row * width + next_col
                if closed[next_index]:
                    continue
                traversal_penalty = float(planning_map.traversal_cost[next_row, next_col]) / 100.0
                tentative_score = (
                    float(g_score[current_index])
                    + step_cost * planning_map.resolution_m
                    + traversal_penalty * planning_map.resolution_m * 2.5
                )
                if tentative_score + 1e-9 >= float(g_score[next_index]):
                    continue
                g_score[next_index] = tentative_score
                came_from[next_index] = current_index
                heuristic = self._heuristic((next_row, next_col), goal_cell, planning_map.resolution_m)
                heapq.heappush(open_heap, (tentative_score + heuristic, next_index))

        return []

    def _heuristic(self, current_cell: Cell2D, goal_cell: Cell2D, resolution_m: float) -> float:
        row_delta = abs(goal_cell[0] - current_cell[0])
        col_delta = abs(goal_cell[1] - current_cell[1])
        diagonal = min(row_delta, col_delta)
        straight = max(row_delta, col_delta) - diagonal
        return (diagonal * math.sqrt(2.0) + straight) * resolution_m

    def _reconstruct_path(self, *, came_from: np.ndarray, width: int, current_index: int) -> List[Cell2D]:
        path: List[Cell2D] = []
        while current_index >= 0:
            path.append((current_index // width, current_index % width))
            current_index = int(came_from[current_index])
        path.reverse()
        return path

    def _simplify_path_cells(self, *, planning_map: _PlanningMap, cell_path: Sequence[Cell2D]) -> List[Cell2D]:
        if len(cell_path) <= 2:
            return list(cell_path)
        simplified = [cell_path[0]]
        anchor_index = 0
        while anchor_index < len(cell_path) - 1:
            next_index = len(cell_path) - 1
            while next_index > anchor_index + 1:
                if planning_map.line_is_clear(cell_path[anchor_index], cell_path[next_index]):
                    break
                next_index -= 1
            simplified.append(cell_path[next_index])
            anchor_index = next_index
        return simplified

    def _deduplicate_waypoints(self, waypoints: Sequence[Point2D]) -> List[Point2D]:
        deduplicated: List[Point2D] = []
        for waypoint in waypoints:
            if deduplicated and _point_distance(deduplicated[-1], waypoint) <= 1e-6:
                continue
            deduplicated.append(waypoint)
        return deduplicated


def _bresenham_cells(start_cell: Cell2D, end_cell: Cell2D) -> List[Cell2D]:
    start_row, start_col = start_cell
    end_row, end_col = end_cell
    row_delta = abs(end_row - start_row)
    col_delta = abs(end_col - start_col)
    step_row = 1 if start_row < end_row else -1
    step_col = 1 if start_col < end_col else -1
    error = row_delta - col_delta
    row = start_row
    col = start_col
    result: List[Cell2D] = []
    while True:
        result.append((row, col))
        if row == end_row and col == end_col:
            break
        double_error = 2 * error
        if double_error > -col_delta:
            error -= col_delta
            row += step_row
        if double_error < row_delta:
            error += row_delta
            col += step_col
    return result


def _point_distance(start: Point2D, end: Point2D) -> float:
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _polyline_length(polyline: Sequence[Point2D]) -> float:
    if len(polyline) <= 1:
        return 0.0
    total = 0.0
    for index in range(1, len(polyline)):
        total += _point_distance(polyline[index - 1], polyline[index])
    return total



__all__ = [
    "GridNavigationPlanner",
    "GridPlan",
]
