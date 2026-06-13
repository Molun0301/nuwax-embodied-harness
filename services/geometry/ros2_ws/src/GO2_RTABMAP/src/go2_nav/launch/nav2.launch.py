"""
Nav2 launch for Unitree GO2.

nav2_params.yaml is patched at launch time from NUWAX_GEOMETRY_* environment.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from geometry_launch_config import GeometryLaunchConfig, materialize_nav2_params  # noqa: E402


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cfg = GeometryLaunchConfig.from_env()
    go2_nav_dir = get_package_share_directory('go2_nav')
    nav2_params_file = materialize_nav2_params(go2_nav_dir, cfg)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recovery_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recovery_server',
                    'bt_navigator',
                ],
            }],
        ),

        # Nav2 Foxy only publishes /plan when a compatible subscriber exists; relay
        # for RViz (see plan_relay.py).
        Node(
            package='go2_nav',
            executable='plan_relay.py',
            name='plan_relay',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
