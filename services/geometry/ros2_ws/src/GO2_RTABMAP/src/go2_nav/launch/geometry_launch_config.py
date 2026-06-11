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
            nav2_action_name=_env("NUWAX_NAV2_ACTION_NAME", "/navigate_to_pose"),
        )

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
