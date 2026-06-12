"""Read platform geometry topic/frame settings for go2_nav launch files."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict


def _env(name: str, default: str) -> str:
    value = os.environ.get(name, "").strip()
    return value or default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return int(float(raw))
    except ValueError:
        return default


@dataclass(frozen=True)
class GeometryLaunchConfig:
    odom_topic: str
    point_cloud_topic: str
    scan_topic: str
    map_topic: str
    map_frame: str
    odom_frame: str
    base_frame: str
    lidar_frame: str
    imu_topic: str
    robot_radius: float
    inflation_radius: float
    inflation_cost_scaling: float
    exploration_min_frontier_size: int
    exploration_failures_before_relaxed: int
    exploration_failures_before_corridor: int
    exploration_failures_before_retry: int
    exploration_bootstrap_forward_m: float
    exploration_min_nav_commit_sec: float
    exploration_max_goal_retries: int
    exploration_post_goal_pause_sec: float
    exploration_min_travel_m: float
    exploration_goal_setback_m: float
    cost_map_topic: str
    nav2_action_name: str

    @classmethod
    def from_env(cls) -> "GeometryLaunchConfig":
        return cls(
            odom_topic=_env("NUWAX_GEOMETRY_ODOM_TOPIC", "/utlidar/robot_odom"),
            point_cloud_topic=_env(
                "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "/utlidar/cloud_deskewed"
            ),
            scan_topic=_env("NUWAX_GEOMETRY_SCAN_TOPIC", "/scan"),
            map_topic=_env("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map"),
            map_frame=_env("NUWAX_GEOMETRY_MAP_FRAME", "map"),
            odom_frame=_env("NUWAX_GEOMETRY_ODOM_FRAME", "odom"),
            base_frame=_env("NUWAX_GEOMETRY_BASE_FRAME", "base_link"),
            lidar_frame=_env("NUWAX_GEOMETRY_LIDAR_FRAME", "utlidar_lidar"),
            imu_topic=_env("NUWAX_GEOMETRY_IMU_TOPIC", "/utlidar/imu"),
            robot_radius=_env_float("NUWAX_GEOMETRY_ROBOT_RADIUS", 0.30),
            inflation_radius=_env_float("NUWAX_GEOMETRY_INFLATION_RADIUS", 0.60),
            inflation_cost_scaling=_env_float(
                "NUWAX_GEOMETRY_INFLATION_COST_SCALING", 2.5
            ),
            exploration_min_frontier_size=_env_int(
                "NUWAX_GEOMETRY_EXPLORATION_MIN_FRONTIER_SIZE", 25
            ),
            exploration_failures_before_relaxed=_env_int(
                "NUWAX_GEOMETRY_EXPLORATION_FAILURES_BEFORE_RELAXED", 4
            ),
            exploration_failures_before_corridor=_env_int(
                "NUWAX_GEOMETRY_EXPLORATION_FAILURES_BEFORE_CORRIDOR", 7
            ),
            exploration_failures_before_retry=_env_int(
                "NUWAX_GEOMETRY_EXPLORATION_FAILURES_BEFORE_RETRY", 10
            ),
            exploration_bootstrap_forward_m=_env_float(
                "NUWAX_GEOMETRY_EXPLORATION_BOOTSTRAP_FORWARD_M", 0.2
            ),
            exploration_min_nav_commit_sec=_env_float(
                "NUWAX_GEOMETRY_EXPLORATION_MIN_NAV_COMMIT_SEC", 15.0
            ),
            exploration_max_goal_retries=_env_int(
                "NUWAX_GEOMETRY_EXPLORATION_MAX_GOAL_RETRIES", 4
            ),
            exploration_post_goal_pause_sec=_env_float(
                "NUWAX_GEOMETRY_EXPLORATION_POST_GOAL_PAUSE_SEC", 0.0
            ),
            exploration_min_travel_m=_env_float(
                "NUWAX_GEOMETRY_EXPLORATION_MIN_TRAVEL_M", 1.2
            ),
            exploration_goal_setback_m=_env_float(
                "NUWAX_GEOMETRY_EXPLORATION_GOAL_SETBACK_M", 0.6
            ),
            cost_map_topic=_env(
                "NUWAX_GEOMETRY_COST_MAP_TOPIC", "/global_costmap/costmap"
            ),
            nav2_action_name=_env("NUWAX_NAV2_ACTION_NAME", "/navigate_to_pose"),
        )

    def frontier_explorer_parameters(self) -> Dict[str, Any]:
        """ROS params for frontier_explorer (office + corridor tuned defaults)."""
        return {
            'map_topic': self.map_topic,
            'global_frame': self.map_frame,
            'robot_frame': self.base_frame,
            'replan_period': 5.0,
            'min_frontier_size': self.exploration_min_frontier_size,
            'info_gain_radius': 1.5,
            'info_gain_weight': 1.0,
            'size_weight': 0.6,
            'travel_weight': 0.8,
            # Heavier turn penalty so the explorer prefers frontiers roughly
            # ahead of the robot instead of ones behind it (turn=120deg+),
            # which forced slow in-place spins ("turn-a-step-then-stop").
            'turn_weight': 0.6,
            'clearance_weight': 0.8,
            'max_goal_candidates_per_frontier': 80,
            'blacklist_radius': 1.2,
            'blacklist_ttl': 60.0,
            'max_consecutive_failures': 10,
            'failures_before_relaxed': self.exploration_failures_before_relaxed,
            'failures_before_corridor': self.exploration_failures_before_corridor,
            'failures_before_retry_reset': self.exploration_failures_before_retry,
            'relaxed_min_goal_clearance_m': 0.15,
            # relaxed kicks in AFTER failures, so it must be <= normal min_travel
            # (0.7) — requiring a farther goal when we're already struggling makes
            # no sense and was an inversion bug.
            'relaxed_min_travel_m': 0.5,
            'corridor_travel_weight_scale': 1.35,
            'corridor_turn_weight_scale': 1.5,
            'retry_cooldown_sec': 12.0,
            'bootstrap_forward_enabled': True,
            'bootstrap_forward_m': self.exploration_bootstrap_forward_m,
            'startup_grace_period': 15.0,
            'min_travel_m': self.exploration_min_travel_m,
            'min_goal_clearance_m': 0.25,
            'goal_setback_m': self.exploration_goal_setback_m,
            # Validate goals against Nav2's global costmap (live obstacles +
            # inflation + unknown) so we never dispatch an un-plannable goal.
            'costmap_topic': self.cost_map_topic,
            'costmap_lethal_cost': 90,
            'costmap_allow_unknown': True,
            'require_costmap_traversable': True,
            'goal_success_tolerance': 0.35,
            'post_goal_pause_sec': self.exploration_post_goal_pause_sec,
            'min_goal_progress_m': 0.40,
            'success_min_elapsed_sec': 4.0,
            'min_nav_commit_sec': self.exploration_min_nav_commit_sec,
            'max_goal_retries': self.exploration_max_goal_retries,
            'goal_retry_delay_sec': 3.0,
            'goal_accept_timeout': 12.0,
            'stuck_escape_enabled': True,
            'stuck_escape_forward_m': 0.35,
            'stuck_escape_after_failures': 5,
            'stuck_escape_max_attempts': 3,
        }

    @property
    def nav2_action_basename(self) -> str:
        name = self.nav2_action_name.strip()
        if name.startswith("/"):
            return name[1:]
        return name


def materialize_nav2_params(go2_nav_dir: str, cfg: GeometryLaunchConfig) -> str:
    """Patch nav2_params.yaml with platform geometry settings and return path."""
    import yaml

    source = os.path.join(go2_nav_dir, "config", "nav2_params.yaml")
    with open(source, "r", encoding="utf-8") as handle:
        data: Dict[str, Any] = yaml.safe_load(handle)

    bt = data["bt_navigator"]["ros__parameters"]
    bt["odom_topic"] = cfg.odom_topic
    bt["global_frame"] = cfg.map_frame
    bt["robot_base_frame"] = cfg.base_frame
    bt["default_bt_xml_filename"] = os.path.join(
        go2_nav_dir, "config", "navigate_simple_replanning.xml"
    )

    controller = data["controller_server"]["ros__parameters"]
    controller["odom_topic"] = cfg.odom_topic

    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    local["global_frame"] = cfg.odom_frame
    local["robot_base_frame"] = cfg.base_frame
    local["robot_radius"] = cfg.robot_radius
    local["obstacle_layer"]["scan"]["topic"] = cfg.scan_topic
    local["inflation_layer"]["inflation_radius"] = cfg.inflation_radius
    local["inflation_layer"]["cost_scaling_factor"] = cfg.inflation_cost_scaling

    global_cm = data["global_costmap"]["global_costmap"]["ros__parameters"]
    global_cm["global_frame"] = cfg.map_frame
    global_cm["robot_base_frame"] = cfg.base_frame
    global_cm["robot_radius"] = cfg.robot_radius
    global_cm["obstacle_layer"]["scan"]["topic"] = cfg.scan_topic
    global_cm["inflation_layer"]["inflation_radius"] = cfg.inflation_radius
    global_cm["inflation_layer"]["cost_scaling_factor"] = cfg.inflation_cost_scaling

    runtime_dir = os.environ.get(
        "GO2_NAV2_PARAMS_RUNTIME_DIR",
        os.path.join(os.path.expanduser("~"), ".cache", "go2_nav"),
    )
    os.makedirs(runtime_dir, exist_ok=True)
    target = os.path.join(runtime_dir, "nav2_params.effective.yaml")
    with open(target, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    return target
