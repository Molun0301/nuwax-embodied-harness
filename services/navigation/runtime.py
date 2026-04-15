from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from compat import StrEnum
from typing import Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.maps import CostMap, OccupancyGrid
from contracts.navigation import NavigationGoal, NavigationStatus
from services.navigation.path_tools import evaluate_path_safety, resample_waypoints

if TYPE_CHECKING:
    from services.navigation.config import NavigationBackendConfig, NavigationControlConfig


Point2D = Tuple[float, float]
Cell2D = Tuple[int, int]
_PLATFORM_NAVIGATION_BACKEND_NAME = "platform_grid_navigation"


class NavigationExecutionState(StrEnum):
    """导航执行状态机。"""

    IDLE = "idle"
    PLANNING = "planning"
    FOLLOWING_PATH = "following_path"
    FINAL_YAW_ALIGNMENT = "final_yaw_alignment"
    DIRECT_FALLBACK = "direct_fallback"
    RECOVERY = "recovery"


@dataclass
class GridPlan:
    """平台网格导航一次路径规划结果。"""

    frame_id: str
    waypoints: List[Point2D]
    planning_target: Point2D
    total_distance_m: float
    source_map: str
    planning_mode: str
    goal_in_map: bool
    target_adjusted: bool


@dataclass
class NavigationTickResult:
    """导航执行单次控制更新结果。"""

    status: NavigationStatus
    message: str
    remaining_distance_m: Optional[float] = None
    remaining_yaw_rad: Optional[float] = None
    goal_reached: bool = False
    linear_x_mps: float = 0.0
    angular_z_rps: float = 0.0
    metadata: Dict[str, object] = field(default_factory=dict)


@dataclass
class _PolylineProjection:
    """当前位姿在折线上的投影。"""

    projected_point: Point2D
    along_distance_m: float
    lateral_distance_m: float
    segment_index: int


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
    """平台二维网格路径规划器。"""

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
        offsets: List[Tuple[int, int]] = []
        for row_offset in range(-radius_cells, radius_cells + 1):
            for col_offset in range(-radius_cells, radius_cells + 1):
                if math.hypot(float(row_offset), float(col_offset)) <= float(radius_cells) + 1e-6:
                    offsets.append((row_offset, col_offset))

        height, width = blocked.shape
        inflated = blocked.copy()
        occupied_rows, occupied_cols = np.where(blocked)
        for row, col in zip(occupied_rows.tolist(), occupied_cols.tolist()):
            for row_offset, col_offset in offsets:
                next_row = row + row_offset
                next_col = col + col_offset
                if 0 <= next_row < height and 0 <= next_col < width:
                    inflated[next_row, next_col] = True
        return inflated

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
        if planning_map.contains_world(*goal_point, padding_m=float(self.config.planning_horizon_margin_m)):
            return goal_point, "goal", True

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
        horizon_padding_m = float(self.config.planning_horizon_margin_m)

        sampled_distance = max_distance_m
        while sampled_distance > step_m:
            candidate = (
                start_point[0] + direction_x * sampled_distance,
                start_point[1] + direction_y * sampled_distance,
            )
            if planning_map.contains_world(*candidate, padding_m=horizon_padding_m):
                return candidate, "local_horizon", False
            sampled_distance -= step_m
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


class NavigationSession:
    """单个导航目标的规划与控制会话。"""

    def __init__(
        self,
        *,
        goal: NavigationGoal,
        planner: GridNavigationPlanner,
        planner_config: "NavigationBackendConfig",
        control_config: "NavigationControlConfig",
    ) -> None:
        self.goal = goal
        self._planner = planner
        self._planner_config = planner_config
        self._control_config = control_config
        self._current_plan: Optional[GridPlan] = None
        self._plan_version = 0
        self._last_plan_time_monotonic = 0.0
        self._last_progress_time_monotonic = 0.0
        self._best_goal_distance_m = float("inf")
        self._plan_failure_count = 0
        self._last_replan_reason = "initial_plan"
        self._execution_state = NavigationExecutionState.IDLE

    def tick(
        self,
        *,
        current_pose: Pose,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
        now_monotonic: float,
    ) -> NavigationTickResult:
        target_pose = self.goal.target_pose
        if target_pose is None:
            return NavigationTickResult(
                status=NavigationStatus.FAILED,
                message="导航目标缺少目标位姿。",
            )
        if (
            current_pose.frame_id
            and target_pose.frame_id
            and not frame_ids_semantically_equal(current_pose.frame_id, target_pose.frame_id)
        ):
            return NavigationTickResult(
                status=NavigationStatus.FAILED,
                message=f"目标坐标系 {target_pose.frame_id} 与当前位姿坐标系 {current_pose.frame_id} 不一致。",
            )

        goal_distance_m = _point_distance(
            (current_pose.position.x, current_pose.position.y),
            (target_pose.position.x, target_pose.position.y),
        )
        goal_yaw_error_rad = _normalize_angle(
            _yaw_from_pose(target_pose) - _yaw_from_pose(current_pose)
        )
        self._update_progress(goal_distance_m=goal_distance_m, now_monotonic=now_monotonic)

        if goal_distance_m <= float(self._control_config.goal_tolerance_m):
            if abs(goal_yaw_error_rad) <= float(self._control_config.goal_yaw_tolerance_rad):
                self._execution_state = NavigationExecutionState.IDLE
                return NavigationTickResult(
                    status=NavigationStatus.SUCCEEDED,
                    message="平台网格导航目标已到达。",
                    remaining_distance_m=0.0,
                    remaining_yaw_rad=0.0,
                    goal_reached=True,
                    metadata={
                        "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                        "controller_mode": "goal_reached",
                        "execution_state": NavigationExecutionState.IDLE,
                        "plan_version": self._plan_version,
                    },
                )
            self._execution_state = NavigationExecutionState.FINAL_YAW_ALIGNMENT
            return NavigationTickResult(
                status=NavigationStatus.RUNNING,
                message="已到达目标位置，正在进行最终朝向对齐。",
                remaining_distance_m=goal_distance_m,
                remaining_yaw_rad=goal_yaw_error_rad,
                linear_x_mps=0.0,
                angular_z_rps=self._clamp(
                    goal_yaw_error_rad * float(self._planner_config.yaw_gain),
                    -float(self._control_config.max_yaw_rate_rps),
                    float(self._control_config.max_yaw_rate_rps),
                ),
                metadata={
                    "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                    "controller_mode": "final_yaw_alignment",
                    "execution_state": NavigationExecutionState.FINAL_YAW_ALIGNMENT,
                    "plan_version": self._plan_version,
                },
            )

        replan_reason = self._determine_replan_reason(
            current_pose=current_pose,
            goal_distance_m=goal_distance_m,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            now_monotonic=now_monotonic,
        )
        if replan_reason is not None:
            replan_result = self._replan(
                current_pose=current_pose,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                reason=replan_reason,
                now_monotonic=now_monotonic,
            )
            if replan_result is not None:
                return replan_result

        if self._current_plan is None:
            return self._direct_fallback_tick(
                current_pose=current_pose,
                goal_distance_m=goal_distance_m,
                goal_yaw_error_rad=goal_yaw_error_rad,
            )

        projection = _project_to_polyline(
            point=(current_pose.position.x, current_pose.position.y),
            polyline=self._current_plan.waypoints,
        )
        path_remaining_m = max(0.0, self._current_plan.total_distance_m - projection.along_distance_m)
        estimated_remaining_m = path_remaining_m
        if self._current_plan.planning_mode != "goal":
            estimated_remaining_m += _point_distance(
                self._current_plan.planning_target,
                (target_pose.position.x, target_pose.position.y),
            )

        lookahead_distance_m = min(
            self._current_plan.total_distance_m,
            projection.along_distance_m + max(0.2, float(self._planner_config.lookahead_distance_m)),
        )
        lookahead_point = _point_on_polyline(
            polyline=self._current_plan.waypoints,
            target_distance_m=lookahead_distance_m,
        )
        current_yaw = _yaw_from_pose(current_pose)
        heading_error_rad = _normalize_angle(
            math.atan2(lookahead_point[1] - current_pose.position.y, lookahead_point[0] - current_pose.position.x) - current_yaw
        )

        if abs(heading_error_rad) >= float(self._planner_config.rotate_in_place_heading_rad):
            linear_x_mps = 0.0
        else:
            heading_scale = max(
                0.15,
                1.0 - abs(heading_error_rad) / max(float(self._planner_config.heading_slowdown_rad), 1e-3),
            )
            remaining_scale = min(
                1.0,
                estimated_remaining_m / max(float(self._planner_config.lookahead_distance_m), float(self._control_config.goal_tolerance_m)),
            )
            linear_x_mps = self._clamp(
                float(self._control_config.max_linear_velocity_mps) * heading_scale * remaining_scale,
                0.08,
                float(self._control_config.max_linear_velocity_mps),
            )

        angular_z_rps = self._clamp(
            heading_error_rad * float(self._planner_config.yaw_gain),
            -float(self._control_config.max_yaw_rate_rps),
            float(self._control_config.max_yaw_rate_rps),
        )
        self._execution_state = NavigationExecutionState.FOLLOWING_PATH
        return NavigationTickResult(
            status=NavigationStatus.RUNNING,
            message="平台网格导航执行中。",
            remaining_distance_m=estimated_remaining_m,
            remaining_yaw_rad=goal_yaw_error_rad,
            linear_x_mps=linear_x_mps,
            angular_z_rps=angular_z_rps,
            metadata={
                "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                "controller_mode": "planned_path_follow",
                "execution_state": NavigationExecutionState.FOLLOWING_PATH,
                "heading_error_rad": heading_error_rad,
                "goal_yaw_error_rad": goal_yaw_error_rad,
                "path_deviation_m": projection.lateral_distance_m,
                "plan_version": self._plan_version,
                "plan_source_map": self._current_plan.source_map,
                "planning_mode": self._current_plan.planning_mode,
                "replan_reason": self._last_replan_reason,
                "lookahead_point": {
                    "x": float(lookahead_point[0]),
                    "y": float(lookahead_point[1]),
                },
            },
        )

    def preview_reachability(
        self,
        *,
        current_pose: Pose,
        target_pose: Pose,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
    ) -> Optional[GridPlan]:
        return self._planner.plan_preview(
            current_pose=current_pose,
            target_pose=target_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )

    def _determine_replan_reason(
        self,
        *,
        current_pose: Pose,
        goal_distance_m: float,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
        now_monotonic: float,
    ) -> Optional[str]:
        if self._current_plan is None:
            return "initial_plan"
        if occupancy_grid is None and cost_map is None:
            return None
        if now_monotonic - self._last_plan_time_monotonic >= float(self._planner_config.replan_interval_sec):
            return "periodic_replan"

        projection = _project_to_polyline(
            point=(current_pose.position.x, current_pose.position.y),
            polyline=self._current_plan.waypoints,
        )
        if projection.lateral_distance_m >= float(self._planner_config.max_path_deviation_m):
            return "path_deviation"

        if now_monotonic - self._last_progress_time_monotonic >= float(self._planner_config.stuck_timeout_sec):
            return "stuck_replan"

        if self._current_plan.planning_mode == "local_horizon":
            to_planning_target_m = _point_distance(
                (current_pose.position.x, current_pose.position.y),
                self._current_plan.planning_target,
            )
            if to_planning_target_m <= max(
                float(self._planner_config.lookahead_distance_m),
                float(self._control_config.goal_tolerance_m) * 2.0,
            ):
                return "advance_horizon"

        planning_map = self._planner._build_planning_map(
            current_frame_id=current_pose.frame_id,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        if planning_map is not None and self._path_blocked_ahead(
            planning_map=planning_map,
            current_pose=current_pose,
            projection=projection,
        ):
            return "obstacle_ahead"
        del goal_distance_m
        return None

    def _path_blocked_ahead(
        self,
        *,
        planning_map: _PlanningMap,
        current_pose: Pose,
        projection: _PolylineProjection,
    ) -> bool:
        if self._current_plan is None:
            return False
        lookahead_distance_m = min(
            self._current_plan.total_distance_m,
            projection.along_distance_m + max(
                float(self._planner_config.lookahead_distance_m),
                float(self._planner_config.obstacle_check_distance_m),
            ),
        )
        lookahead_point = _point_on_polyline(
            polyline=self._current_plan.waypoints,
            target_distance_m=lookahead_distance_m,
        )
        return not planning_map.line_is_clear_world(
            (current_pose.position.x, current_pose.position.y),
            lookahead_point,
        )

    def _replan(
        self,
        *,
        current_pose: Pose,
        occupancy_grid: Optional[OccupancyGrid],
        cost_map: Optional[CostMap],
        reason: str,
        now_monotonic: float,
    ) -> Optional[NavigationTickResult]:
        cooldown_sec = max(0.0, float(self._planner_config.replan_cooldown_sec))
        if self._current_plan is not None and (now_monotonic - self._last_plan_time_monotonic) < cooldown_sec:
            return None

        planning_outcome = self._planner.plan(
            current_pose=current_pose,
            target_pose=self.goal.target_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        self._last_replan_reason = reason
        if planning_outcome.reason == "map_unavailable":
            self._current_plan = None
            return None
        if planning_outcome.plan is None:
            self._plan_failure_count += 1
            execution_state = (
                NavigationExecutionState.PLANNING
                if reason == "initial_plan"
                else NavigationExecutionState.RECOVERY
            )
            self._execution_state = execution_state
            if self._plan_failure_count >= int(self._planner_config.max_replan_failures):
                return NavigationTickResult(
                    status=NavigationStatus.FAILED,
                    message=planning_outcome.message,
                    metadata={
                        "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                        "execution_state": execution_state,
                        "replan_reason": reason,
                        "plan_failures": self._plan_failure_count,
                    },
                )
            return NavigationTickResult(
                status=NavigationStatus.PLANNING,
                message=planning_outcome.message,
                metadata={
                    "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                    "execution_state": execution_state,
                    "replan_reason": reason,
                    "plan_failures": self._plan_failure_count,
                },
            )

        path_safety = self._evaluate_path_safety(
            plan=planning_outcome.plan,
            occupancy_grid=occupancy_grid,
        )
        if path_safety is not None and not path_safety.is_safe:
            self._plan_failure_count += 1
            self._execution_state = NavigationExecutionState.RECOVERY
            message = (
                "规划路径与障碍物掩码重叠比例过高，已拒绝执行并等待重规划。"
            )
            if self._plan_failure_count >= int(self._planner_config.max_replan_failures):
                return NavigationTickResult(
                    status=NavigationStatus.FAILED,
                    message=message,
                    metadata={
                        "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                        "execution_state": NavigationExecutionState.RECOVERY,
                        "replan_reason": "path_mask_blocked",
                        "path_occupied_ratio": path_safety.occupied_ratio,
                        "plan_failures": self._plan_failure_count,
                    },
                )
            return NavigationTickResult(
                status=NavigationStatus.PLANNING,
                message=message,
                metadata={
                    "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                    "execution_state": NavigationExecutionState.RECOVERY,
                    "replan_reason": "path_mask_blocked",
                    "path_occupied_ratio": path_safety.occupied_ratio,
                    "occupied_path_cells": path_safety.occupied_cell_count,
                    "total_path_cells": path_safety.total_cell_count,
                    "plan_failures": self._plan_failure_count,
                },
            )

        self._current_plan = planning_outcome.plan
        self._plan_version += 1
        self._plan_failure_count = 0
        self._last_plan_time_monotonic = now_monotonic
        if self._last_progress_time_monotonic <= 0.0:
            self._last_progress_time_monotonic = now_monotonic
        return None

    def _direct_fallback_tick(
        self,
        *,
        current_pose: Pose,
        goal_distance_m: float,
        goal_yaw_error_rad: float,
    ) -> NavigationTickResult:
        target_pose = self.goal.target_pose
        current_yaw = _yaw_from_pose(current_pose)
        desired_heading_rad = math.atan2(
            float(target_pose.position.y - current_pose.position.y),
            float(target_pose.position.x - current_pose.position.x),
        )
        heading_error_rad = _normalize_angle(desired_heading_rad - current_yaw)

        if abs(heading_error_rad) >= float(self._planner_config.rotate_in_place_heading_rad):
            linear_x_mps = 0.0
        else:
            linear_x_mps = self._clamp(
                goal_distance_m * 0.45,
                0.08,
                float(self._control_config.max_linear_velocity_mps),
            )
        angular_z_rps = self._clamp(
            heading_error_rad * float(self._planner_config.yaw_gain),
            -float(self._control_config.max_yaw_rate_rps),
            float(self._control_config.max_yaw_rate_rps),
        )
        self._execution_state = NavigationExecutionState.DIRECT_FALLBACK
        return NavigationTickResult(
            status=NavigationStatus.RUNNING,
            message="地图未就绪，当前按降级直驱模式执行导航。",
            remaining_distance_m=goal_distance_m,
            remaining_yaw_rad=goal_yaw_error_rad,
            linear_x_mps=linear_x_mps,
            angular_z_rps=angular_z_rps,
            metadata={
                "backend": _PLATFORM_NAVIGATION_BACKEND_NAME,
                "controller_mode": "direct_fallback",
                "execution_state": NavigationExecutionState.DIRECT_FALLBACK,
                "heading_error_rad": heading_error_rad,
                "goal_yaw_error_rad": goal_yaw_error_rad,
                "plan_version": self._plan_version,
            },
        )

    def _evaluate_path_safety(
        self,
        *,
        plan: GridPlan,
        occupancy_grid: Optional[OccupancyGrid],
    ):
        if occupancy_grid is None:
            return None
        if (
            occupancy_grid.frame_id
            and plan.frame_id
            and not frame_ids_semantically_equal(occupancy_grid.frame_id, plan.frame_id)
        ):
            return None
        robot_width_m = float(getattr(self._planner_config, "path_mask_robot_width_m", 0.0))
        if robot_width_m <= 1e-6:
            return None
        return evaluate_path_safety(
            occupancy_grid,
            waypoints=plan.waypoints,
            robot_width_m=robot_width_m,
            obstacle_threshold=int(getattr(self._planner_config, "occupancy_lethal_threshold", 65)),
            max_occupied_ratio=float(getattr(self._planner_config, "path_mask_max_occupied_ratio", 0.05)),
        )

    def _update_progress(self, *, goal_distance_m: float, now_monotonic: float) -> None:
        if goal_distance_m + float(self._planner_config.progress_epsilon_m) < self._best_goal_distance_m:
            self._best_goal_distance_m = goal_distance_m
            self._last_progress_time_monotonic = now_monotonic
        elif self._last_progress_time_monotonic <= 0.0:
            self._last_progress_time_monotonic = now_monotonic

    def _clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


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


def _yaw_from_pose(pose: Pose) -> float:
    rotation = pose.orientation
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _point_distance(start: Point2D, end: Point2D) -> float:
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _polyline_length(polyline: Sequence[Point2D]) -> float:
    if len(polyline) <= 1:
        return 0.0
    total = 0.0
    for index in range(1, len(polyline)):
        total += _point_distance(polyline[index - 1], polyline[index])
    return total


def _project_to_polyline(*, point: Point2D, polyline: Sequence[Point2D]) -> _PolylineProjection:
    if len(polyline) == 1:
        return _PolylineProjection(
            projected_point=polyline[0],
            along_distance_m=0.0,
            lateral_distance_m=_point_distance(point, polyline[0]),
            segment_index=0,
        )

    best_projection = _PolylineProjection(
        projected_point=polyline[0],
        along_distance_m=0.0,
        lateral_distance_m=float("inf"),
        segment_index=0,
    )
    traversed_distance_m = 0.0
    for segment_index in range(1, len(polyline)):
        segment_start = polyline[segment_index - 1]
        segment_end = polyline[segment_index]
        segment_dx = segment_end[0] - segment_start[0]
        segment_dy = segment_end[1] - segment_start[1]
        segment_length_sq = segment_dx * segment_dx + segment_dy * segment_dy
        if segment_length_sq <= 1e-12:
            continue
        projection_ratio = (
            ((point[0] - segment_start[0]) * segment_dx) + ((point[1] - segment_start[1]) * segment_dy)
        ) / segment_length_sq
        projection_ratio = max(0.0, min(1.0, projection_ratio))
        projected_point = (
            segment_start[0] + segment_dx * projection_ratio,
            segment_start[1] + segment_dy * projection_ratio,
        )
        lateral_distance_m = _point_distance(point, projected_point)
        segment_length_m = math.sqrt(segment_length_sq)
        along_distance_m = traversed_distance_m + segment_length_m * projection_ratio
        if lateral_distance_m < best_projection.lateral_distance_m:
            best_projection = _PolylineProjection(
                projected_point=projected_point,
                along_distance_m=along_distance_m,
                lateral_distance_m=lateral_distance_m,
                segment_index=segment_index - 1,
            )
        traversed_distance_m += segment_length_m
    return best_projection


def _point_on_polyline(*, polyline: Sequence[Point2D], target_distance_m: float) -> Point2D:
    if not polyline:
        return (0.0, 0.0)
    if len(polyline) == 1 or target_distance_m <= 0.0:
        return polyline[0]
    traversed_distance_m = 0.0
    for segment_index in range(1, len(polyline)):
        segment_start = polyline[segment_index - 1]
        segment_end = polyline[segment_index]
        segment_length_m = _point_distance(segment_start, segment_end)
        if traversed_distance_m + segment_length_m >= target_distance_m:
            remaining_distance_m = target_distance_m - traversed_distance_m
            ratio = remaining_distance_m / max(segment_length_m, 1e-9)
            return (
                segment_start[0] + (segment_end[0] - segment_start[0]) * ratio,
                segment_start[1] + (segment_end[1] - segment_start[1]) * ratio,
            )
        traversed_distance_m += segment_length_m
    return polyline[-1]


Go2GridPlan = GridPlan
Go2GridNavigationPlanner = GridNavigationPlanner
Go2NavigationTickResult = NavigationTickResult
Go2NavigationSession = NavigationSession


__all__ = [
    "GridNavigationPlanner",
    "GridPlan",
    "NavigationExecutionState",
    "NavigationSession",
    "NavigationTickResult",
    "Go2GridNavigationPlanner",
    "Go2GridPlan",
    "Go2NavigationSession",
    "Go2NavigationTickResult",
]
