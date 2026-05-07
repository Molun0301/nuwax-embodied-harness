from __future__ import annotations

from dataclasses import dataclass


@dataclass
class NavigationBackendConfig:
    """二维网格路径评估配置。"""

    enabled: bool = True
    occupancy_lethal_threshold: int = 65
    lethal_cost_threshold: float = 90.0
    unknown_cell_cost: float = 55.0
    planner_inflation_radius_m: float = 0.05
    planning_horizon_margin_m: float = 0.60
    path_resample_spacing_m: float = 0.0
    path_smoothing_window: int = 0


@dataclass
class ExplorationConfig:
    """平台统一前沿探索配置。"""

    enabled: bool = False
    frontier_enabled: bool = True
    frontier_min_cluster_cells: int = 6
    frontier_min_unknown_neighbors: int = 1
    frontier_info_gain_weight: float = 1.0
    frontier_distance_weight: float = 1.0
    frontier_cost_weight: float = 0.35
    frontier_heading_weight: float = 0.20
    frontier_semantic_gain_weight: float = 8.0
    frontier_revisit_separation_m: float = 0.75
    frontier_max_no_gain_rounds: int = 2
    frontier_min_information_gain_cells: int = 12
    prerequisite_wait_timeout_sec: float = 15.0
    prerequisite_poll_interval_sec: float = 0.2
    sample_radius_m: float = 1.5
    sample_count: int = 8
    max_goal_cost: float = 75.0
    goal_timeout_sec: float = 20.0
    reach_distance_m: float = 0.45
    status_poll_interval_sec: float = 0.2


__all__ = [
    "ExplorationConfig",
    "NavigationBackendConfig",
]
