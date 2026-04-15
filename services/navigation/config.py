from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class NavigationControlConfig:
    """平台统一导航执行控制配置。"""

    enabled: bool = True
    auto_start_services: bool = False
    service_names: Tuple[str, ...] = (
        "unitree_lidar",
        "unitree_lidar_slam",
        "voxel_height_mapping",
        "obstacles_avoid",
    )
    client_timeout_sec: float = 3.0
    goal_tolerance_m: float = 0.45
    goal_yaw_tolerance_rad: float = 0.45
    control_loop_interval_sec: float = 0.1
    max_linear_velocity_mps: float = 0.35
    max_yaw_rate_rps: float = 0.8
    obstacle_avoidance_default_enabled: bool = True


@dataclass
class NavigationBackendConfig:
    """平台统一网格导航后端配置。"""

    enabled: bool = True
    occupancy_lethal_threshold: int = 65
    lethal_cost_threshold: float = 90.0
    unknown_cell_cost: float = 55.0
    planner_inflation_radius_m: float = 0.05
    planning_horizon_margin_m: float = 0.60
    lookahead_distance_m: float = 0.80
    obstacle_check_distance_m: float = 1.00
    max_path_deviation_m: float = 0.75
    replan_cooldown_sec: float = 0.40
    replan_interval_sec: float = 1.20
    stuck_timeout_sec: float = 2.50
    progress_epsilon_m: float = 0.12
    max_replan_failures: int = 8
    rotate_in_place_heading_rad: float = 0.95
    heading_slowdown_rad: float = 1.20
    yaw_gain: float = 1.35
    path_resample_spacing_m: float = 0.0
    path_smoothing_window: int = 0
    path_mask_robot_width_m: float = 0.0
    path_mask_max_occupied_ratio: float = 0.05


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
    "NavigationControlConfig",
]
