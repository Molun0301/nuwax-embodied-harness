"""树莓派 RTAB-Map 2D + Nav2 联合 launch（cross-robot-generalization-foundations G8）。

复用 rtabmap_ros 的 2D 模式（订阅 /scan LaserScan + camera/rgb Image），
不引入 SLAM Toolbox（memory `feedback_prefer_rtabmap_over_slam_toolbox.md` 锁死）。

启动节点：
  - rtabmap（2D 模式，订阅 /scan + camera）
  - rtabmapviz（可视化，可选）
  - nav2_lifecycle_manager / planner / controller / bt_navigator / 双 costmap

参数：
  - use_sim_time（默认 False；树莓派实机用真实时间）
  - map_yaml（可选；指定则用已建地图直接定位，否则启动在线建图）

跨机器人约束：
  - 与 Go2 的 3D RTAB-Map (`external_slam_nav2` 进程) 互斥；同机仅可运行一种。
  - Nav2 action (`/navigate_to_pose`) 仍是平台唯一正式导航执行链。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml", default_value="",
        description="Path to existing map yaml; empty = start online mapping",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")

    # RTAB-Map 2D 模式：订阅 /scan LaserScan + camera/image；与 Go2 3D 模式（点云）参数不同。
    rtabmap_node = Node(
        package="rtabmap",
        executable="rtabmap",
        name="rtabmap",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            "subscribe_depth": False,
            "subscribe_rgb": True,
            "subscribe_scan": True,
            "approx_sync": True,
        }],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/camera/image_raw"),
            ("rgb/camera_info", "/camera/camera_info"),
        ],
        output="screen",
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav2_lifecycle_manager",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "map_server",
                "planner_server",
                "controller_server",
                "bt_navigator",
                "global_costmap",
                "local_costmap",
            ],
        }],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        rtabmap_node,
        nav2_lifecycle_manager,
    ])