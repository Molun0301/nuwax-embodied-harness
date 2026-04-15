from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple


def build_default_geometry_setup_script() -> str:
    """返回当前仓库内默认几何侧车 setup 脚本路径。"""

    return str(Path(__file__).resolve().parents[1] / "drivers" / "go2" / "ros2_ws" / "install" / "setup.bash")


def build_default_rtabmap_database_root() -> str:
    """返回默认 RTAB-Map 数据库存储根目录。"""

    return str(Path(__file__).resolve().parents[1] / "runtime_data" / "geometry" / "rtabmap")


@dataclass
class GeometryTopicsConfig:
    """平台几何侧车统一话题配置。"""

    odom_topic: str = "/utlidar/robot_odom"
    secondary_odom_topics: Tuple[str, ...] = (
        "/uslam/localization/odom",
        "/lio_sam_ros2/mapping/odometry",
        "/odom_lio",
    )
    imu_topic: str = "/utlidar/imu"
    point_cloud_topic: str = "/utlidar/cloud"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    occupancy_topic: str = "/map"
    cost_map_topic: str = "/global_costmap/costmap"


@dataclass
class GeometryFrameConfig:
    """平台几何侧车统一坐标系配置。"""

    map_frame: str = "odom"
    base_frame: str = "body"
    lidar_frame: str = "utlidar_lidar"


@dataclass
class RtabmapConfig:
    """RTAB-Map 侧车配置。"""

    database_root: str = field(default_factory=build_default_rtabmap_database_root)
    launch_package: str = "rtabmap_ros"
    launch_file: str = "rtabmap.launch.py"
    occupancy_projection: str = "projected_occupancy"


@dataclass
class Nav2Config:
    """Nav2 侧车配置。"""

    action_name: str = "/navigate_to_pose"
    launch_package: str = "nav2_bringup"
    launch_file: str = "navigation_launch.py"
    global_costmap_topic: str = "/global_costmap/costmap"
    local_costmap_topic: str = "/local_costmap/costmap"
    costmap_source: str = "nav2_global"


@dataclass
class PlatformGeometryConfig:
    """平台几何引擎统一配置。"""

    enabled: bool = False
    stack: str = "rtabmap_nav2"
    require_ros2: bool = False
    setup_script: str = field(default_factory=build_default_geometry_setup_script)
    cyclonedds_lib_dir: str = ""
    dds_iface: str = ""
    topics: GeometryTopicsConfig = field(default_factory=GeometryTopicsConfig)
    frames: GeometryFrameConfig = field(default_factory=GeometryFrameConfig)
    rtabmap: RtabmapConfig = field(default_factory=RtabmapConfig)
    nav2: Nav2Config = field(default_factory=Nav2Config)

    def build_metadata(self) -> Dict[str, object]:
        """返回便于状态面记录的几何配置摘要。"""

        return {
            "geometry_stack": self.stack,
            "geometry_enabled": self.enabled,
            "require_ros2": self.require_ros2,
            "setup_script": self.setup_script,
            "cyclonedds_lib_dir": self.cyclonedds_lib_dir or None,
            "dds_iface": self.dds_iface or None,
            "topics": {
                "odom_topic": self.topics.odom_topic,
                "imu_topic": self.topics.imu_topic,
                "point_cloud_topic": self.topics.point_cloud_topic,
                "tf_topic": self.topics.tf_topic,
                "tf_static_topic": self.topics.tf_static_topic,
                "occupancy_topic": self.topics.occupancy_topic,
                "cost_map_topic": self.topics.cost_map_topic,
            },
            "frames": {
                "map_frame": self.frames.map_frame,
                "base_frame": self.frames.base_frame,
                "lidar_frame": self.frames.lidar_frame,
            },
            "rtabmap": {
                "database_root": self.rtabmap.database_root,
                "launch_package": self.rtabmap.launch_package,
                "launch_file": self.rtabmap.launch_file,
                "occupancy_projection": self.rtabmap.occupancy_projection,
            },
            "nav2": {
                "action_name": self.nav2.action_name,
                "launch_package": self.nav2.launch_package,
                "launch_file": self.nav2.launch_file,
                "global_costmap_topic": self.nav2.global_costmap_topic,
                "local_costmap_topic": self.nav2.local_costmap_topic,
                "costmap_source": self.nav2.costmap_source,
            },
        }

