from __future__ import annotations

import math

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import CostMap, OccupancyGrid
from contracts.navigation import NavigationGoal, NavigationStatus
from services.navigation.config import ExplorationConfig, NavigationBackendConfig, NavigationControlConfig
from services.navigation.frontier import FrontierExplorer
from typing import Optional, Set, Tuple

from services.navigation.runtime import GridNavigationPlanner, NavigationSession


class _StaticBridge:
    """测试用静态平台数据桥。"""

    def __init__(self) -> None:
        self.localization_available = True
        self.map_available = True
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def is_localization_available(self) -> bool:
        return self.localization_available

    def is_map_available(self) -> bool:
        return self.map_available

    def is_navigation_available(self) -> bool:
        return False

    def get_status(self):
        return {"started": self.started}

    def get_current_pose(self):
        return None

    def get_frame_tree(self):
        return None

    def get_imu_state(self):
        return None

    def get_occupancy_grid(self):
        return None

    def get_cost_map(self):
        return None

    def get_semantic_map(self):
        return None

    def set_goal(self, goal):
        del goal
        return False

    def cancel_goal(self):
        return False

    def get_navigation_state(self):
        return None

    def is_goal_reached(self):
        return False


def _build_pose(*, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = "odom") -> Pose:
    half_yaw = yaw_rad * 0.5
    return Pose(
        frame_id=frame_id,
        position=Vector3(x=x, y=y, z=0.0),
        orientation=Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw)),
    )


def _build_maps(
    *,
    width: int,
    height: int,
    resolution_m: float,
    origin_x: float,
    origin_y: float,
    blocked_cells: Set[Tuple[int, int]],
) -> Tuple[OccupancyGrid, CostMap]:
    occupancy = [-1] * (width * height)
    cost = [100.0] * (width * height)
    for row in range(height):
        for col in range(width):
            index = row * width + col
            occupancy[index] = 0
            cost[index] = 0.0
    for row, col in blocked_cells:
        index = row * width + col
        occupancy[index] = 100
        cost[index] = 100.0
    origin = _build_pose(x=origin_x, y=origin_y)
    return (
        OccupancyGrid(
            map_id="test_occupancy",
            frame_id="odom",
            width=width,
            height=height,
            resolution_m=resolution_m,
            origin=origin,
            data=occupancy,
        ),
        CostMap(
            map_id="test_cost_map",
            frame_id="odom",
            width=width,
            height=height,
            resolution_m=resolution_m,
            origin=origin,
            data=cost,
        ),
    )


def _build_partial_maps(
    *,
    width: int,
    height: int,
    resolution_m: float,
    origin_x: float,
    origin_y: float,
    free_cells: Set[Tuple[int, int]],
    blocked_cells: Set[Tuple[int, int]],
) -> Tuple[OccupancyGrid, CostMap]:
    occupancy = [-1] * (width * height)
    cost = [100.0] * (width * height)
    for row, col in free_cells:
        index = row * width + col
        occupancy[index] = 0
        cost[index] = 0.0
    for row, col in blocked_cells:
        index = row * width + col
        occupancy[index] = 100
        cost[index] = 100.0
    origin = _build_pose(x=origin_x, y=origin_y)
    return (
        OccupancyGrid(
            map_id="test_frontier_occupancy",
            frame_id="odom",
            width=width,
            height=height,
            resolution_m=resolution_m,
            origin=origin,
            data=occupancy,
        ),
        CostMap(
            map_id="test_frontier_cost_map",
            frame_id="odom",
            width=width,
            height=height,
            resolution_m=resolution_m,
            origin=origin,
            data=cost,
        ),
    )


def _build_session(
    *,
    goal_pose: Pose,
    planner_config: Optional[NavigationBackendConfig] = None,
    control_config: Optional[NavigationControlConfig] = None,
) -> NavigationSession:
    planner_cfg = planner_config or NavigationBackendConfig(
        planner_inflation_radius_m=0.0,
        replan_interval_sec=60.0,
        replan_cooldown_sec=0.0,
        max_replan_failures=4,
    )
    control_cfg = control_config or NavigationControlConfig(
        goal_tolerance_m=0.25,
        goal_yaw_tolerance_rad=0.25,
        max_linear_velocity_mps=0.35,
        max_yaw_rate_rps=0.8,
    )
    planner = GridNavigationPlanner(planner_cfg)
    return NavigationSession(
        goal=NavigationGoal(goal_id="test_goal", target_pose=goal_pose),
        planner=planner,
        planner_config=planner_cfg,
        control_config=control_cfg,
    )


def test_grid_planner_can_route_around_blocking_wall() -> None:
    blocked_cells = {(row, 8) for row in range(20) if row != 10}
    occupancy, cost_map = _build_maps(
        width=20,
        height=20,
        resolution_m=0.5,
        origin_x=0.0,
        origin_y=0.0,
        blocked_cells=blocked_cells,
    )
    planner = GridNavigationPlanner(
        NavigationBackendConfig(
            planner_inflation_radius_m=0.0,
            planning_horizon_margin_m=0.2,
        )
    )
    current_pose = _build_pose(x=1.0, y=2.0)
    goal_pose = _build_pose(x=8.5, y=2.0)

    plan = planner.plan_preview(
        current_pose=current_pose,
        target_pose=goal_pose,
        occupancy_grid=occupancy,
        cost_map=cost_map,
    )

    assert plan is not None
    assert plan.planning_mode == "goal"
    assert plan.total_distance_m > 7.5
    assert any(waypoint_y > 4.5 for _waypoint_x, waypoint_y in plan.waypoints)


def test_grid_planner_accepts_scoped_odom_alias() -> None:
    occupancy, cost_map = _build_maps(
        width=10,
        height=10,
        resolution_m=0.5,
        origin_x=0.0,
        origin_y=0.0,
        blocked_cells=set(),
    )
    planner = GridNavigationPlanner(
        NavigationBackendConfig(
            planner_inflation_radius_m=0.0,
            planning_horizon_margin_m=0.2,
        )
    )
    current_pose = _build_pose(x=1.0, y=1.0, frame_id="world/test/odom")
    goal_pose = _build_pose(x=3.0, y=1.0, frame_id="odom")

    plan = planner.plan_preview(
        current_pose=current_pose,
        target_pose=goal_pose,
        occupancy_grid=occupancy,
        cost_map=cost_map,
    )

    assert plan is not None
    assert plan.frame_id == "odom"


def test_navigation_session_replans_when_path_is_blocked_ahead() -> None:
    session = _build_session(goal_pose=_build_pose(x=6.0, y=1.0))
    free_occupancy, free_cost_map = _build_maps(
        width=20,
        height=20,
        resolution_m=0.5,
        origin_x=0.0,
        origin_y=0.0,
        blocked_cells=set(),
    )

    first_tick = session.tick(
        current_pose=_build_pose(x=1.0, y=1.0),
        occupancy_grid=free_occupancy,
        cost_map=free_cost_map,
        now_monotonic=0.0,
    )

    assert first_tick.status == NavigationStatus.RUNNING
    assert first_tick.metadata["controller_mode"] == "planned_path_follow"
    assert first_tick.metadata["plan_version"] == 1

    blocked_occupancy, blocked_cost_map = _build_maps(
        width=20,
        height=20,
        resolution_m=0.5,
        origin_x=0.0,
        origin_y=0.0,
        blocked_cells={(2, 5), (2, 6), (2, 7)},
    )
    second_tick = session.tick(
        current_pose=_build_pose(x=1.5, y=1.0),
        occupancy_grid=blocked_occupancy,
        cost_map=blocked_cost_map,
        now_monotonic=2.0,
    )

    assert second_tick.status == NavigationStatus.RUNNING
    assert second_tick.metadata["controller_mode"] == "planned_path_follow"
    assert second_tick.metadata["plan_version"] == 2
    assert second_tick.metadata["replan_reason"] == "obstacle_ahead"


def test_navigation_session_falls_back_to_direct_control_without_map() -> None:
    session = _build_session(goal_pose=_build_pose(x=2.0, y=0.0))

    tick = session.tick(
        current_pose=_build_pose(x=0.0, y=0.0),
        occupancy_grid=None,
        cost_map=None,
        now_monotonic=0.0,
    )

    assert tick.status == NavigationStatus.RUNNING
    assert tick.metadata["controller_mode"] == "direct_fallback"
    assert tick.linear_x_mps > 0.0


def test_navigation_session_uses_local_horizon_when_goal_is_outside_map() -> None:
    session = _build_session(goal_pose=_build_pose(x=10.0, y=2.5))
    occupancy, cost_map = _build_maps(
        width=10,
        height=10,
        resolution_m=0.5,
        origin_x=0.0,
        origin_y=0.0,
        blocked_cells=set(),
    )

    tick = session.tick(
        current_pose=_build_pose(x=2.5, y=2.5),
        occupancy_grid=occupancy,
        cost_map=cost_map,
        now_monotonic=0.0,
    )

    assert tick.status == NavigationStatus.RUNNING
    assert tick.metadata["controller_mode"] == "planned_path_follow"
    assert tick.metadata["planning_mode"] == "local_horizon"



def test_frontier_explorer_prefers_reachable_frontier_cluster() -> None:
    planner = GridNavigationPlanner(
        NavigationBackendConfig(
            planner_inflation_radius_m=0.0,
            planning_horizon_margin_m=0.2,
        )
    )
    explorer = FrontierExplorer(
        ExplorationConfig(
            enabled=True,
            frontier_enabled=True,
            frontier_min_cluster_cells=3,
            frontier_revisit_separation_m=0.4,
            max_goal_cost=75.0,
        )
    )
    free_cells = {(row, col) for row in range(7, 13) for col in range(7, 13)}
    free_cells.update({(row, col) for row in range(7, 13) for col in range(15, 18)})
    blocked_cells = {(row, 14) for row in range(6, 14)}
    occupancy, cost_map = _build_partial_maps(
        width=24,
        height=24,
        resolution_m=0.5,
        origin_x=-6.0,
        origin_y=-6.0,
        free_cells=free_cells,
        blocked_cells=blocked_cells,
    )

    candidates = explorer.select_candidates(
        current_pose=_build_pose(x=-0.5, y=-0.5),
        occupancy_grid=occupancy,
        cost_map=cost_map,
        planner=planner,
        center_pose=_build_pose(x=0.0, y=0.0),
        radius_m=6.0,
        attempted_poses=(),
    )

    assert candidates
    assert all(candidate.pose.position.x <= 1.0 for candidate in candidates)

def test_frontier_explorer_uses_global_search_when_radius_is_not_explicit() -> None:
    planner = GridNavigationPlanner(
        NavigationBackendConfig(
            planner_inflation_radius_m=0.0,
            planning_horizon_margin_m=0.2,
        )
    )
    explorer = FrontierExplorer(
        ExplorationConfig(
            enabled=True,
            frontier_enabled=True,
            frontier_min_cluster_cells=3,
            frontier_revisit_separation_m=0.4,
            max_goal_cost=75.0,
        )
    )

    free_cells = {(row, col) for row in range(8, 15) for col in range(8, 15)}
    free_cells.update({(row, col) for row in range(10, 13) for col in range(15, 23)})
    blocked_cells = {(7, col) for col in range(7, 16)}
    blocked_cells.update({(15, col) for col in range(7, 16)})
    blocked_cells.update({(row, 7) for row in range(7, 16)})
    blocked_cells.update({(row, 15) for row in range(7, 16) if row not in {10, 11, 12}})
    blocked_cells.update({(9, col) for col in range(15, 23)})
    blocked_cells.update({(13, col) for col in range(15, 23)})

    occupancy, cost_map = _build_partial_maps(
        width=32,
        height=24,
        resolution_m=0.5,
        origin_x=-8.0,
        origin_y=-6.0,
        free_cells=free_cells,
        blocked_cells=blocked_cells,
    )
    current_pose = _build_pose(x=-2.25, y=-0.25)

    local_candidates = explorer.select_candidates(
        current_pose=current_pose,
        occupancy_grid=occupancy,
        cost_map=cost_map,
        planner=planner,
        center_pose=current_pose,
        radius_m=1.5,
        attempted_poses=(),
    )
    global_candidates = explorer.select_candidates(
        current_pose=current_pose,
        occupancy_grid=occupancy,
        cost_map=cost_map,
        planner=planner,
        center_pose=current_pose,
        radius_m=None,
        attempted_poses=(),
    )

    assert local_candidates == []
    assert global_candidates
    assert math.hypot(
        global_candidates[0].pose.position.x - current_pose.position.x,
        global_candidates[0].pose.position.y - current_pose.position.y,
    ) > 1.5
