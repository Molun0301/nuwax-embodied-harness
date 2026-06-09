"""
Combined launch: RTAB-Map mapping + Nav2 + cmd_vel bridge + frontier explorer.

Use this for fully autonomous mapping: the robot drives itself toward
unknown space, builds the map, stops when the area is fully explored. The
resulting database (default ~/rtabmap.db) can be reused by
go2_navigation.launch.py for localization-only operation.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from geometry_launch_config import GeometryLaunchConfig  # noqa: E402


def generate_launch_description():
    cfg = GeometryLaunchConfig.from_env()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    database_path = LaunchConfiguration('database_path', default='~/rtabmap.db')

    go2_nav_dir = get_package_share_directory('go2_nav')
    non_nav_output = 'log' if os.environ.get('GO2_NAV2_LOG_ONLY') == '1' else 'screen'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/rtabmap.db'),

        # RTAB-Map mapping (SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_nav_dir, 'launch', 'rtabmap_mapping.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'database_path': database_path,
            }.items(),
        ),

        # Nav2 stack
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(go2_nav_dir, 'launch', 'nav2.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # cmd_vel -> GO2 sport API bridge
        Node(
            package='go2_nav',
            executable='cmd_vel_bridge.py',
            name='cmd_vel_bridge',
            output=non_nav_output,
            parameters=[{
                'max_vx': 0.4,
                'max_vy': 0.2,
                'max_vyaw': 0.8,
                'cmd_timeout': 0.5,
            }],
        ),

        # Frontier-based autonomous exploration.
        # Starts ~5s late so /map and Nav2 lifecycle have time to come up.
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='go2_nav',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output=non_nav_output,
                    parameters=[{
                        'map_topic': cfg.map_topic,
                        'global_frame': cfg.map_frame,
                        'robot_frame': cfg.base_frame,
                        'replan_period': 3.0,           # re-evaluate every 3s
                        'min_frontier_size': 10,        # cells, after clustering
                        # Balanced frontier utility:
                        # reward nearby unknown area + useful frontier length,
                        # penalize long travel and sharp heading changes,
                        # prefer goal points with local obstacle clearance.
                        'info_gain_radius': 1.5,        # m around candidate goal
                        'info_gain_weight': 1.0,
                        'size_weight': 0.6,
                        'travel_weight': 0.8,
                        'turn_weight': 0.25,
                        'clearance_weight': 0.8,
                        'max_goal_candidates_per_frontier': 80,
                        'blacklist_radius': 0.5,        # m
                        'blacklist_ttl': 60.0,          # s
                        'max_consecutive_failures': 10,
                        # Extra warmup to avoid Nav2 "send_goal failed" race.
                        'startup_grace_period': 15.0,
                        # Skip frontiers the robot is already beside; require a
                        # real drive before accepting Nav2 success.
                        'min_travel_m': 0.5,
                        # Reject goals squeezed into tight corners (planner/collision).
                        'min_goal_clearance_m': 0.25,
                        'goal_success_tolerance': 0.45,
                    }],
                ),
            ],
        ),
    ])
