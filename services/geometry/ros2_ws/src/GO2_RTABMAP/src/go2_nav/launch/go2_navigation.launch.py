"""
Combined launch: RTAB-Map localization + Nav2 + cmd_vel bridge.
Use this to navigate within an existing map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    database_path = LaunchConfiguration('database_path', default='~/rtabmap.db')

    go2_nav_dir = get_package_share_directory('go2_nav')
    non_nav_output = 'log' if os.environ.get('GO2_NAV2_LOG_ONLY') == '1' else 'screen'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/rtabmap.db'),

        # RTAB-Map localization (existing map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_nav_dir, 'launch', 'rtabmap_localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'database_path': database_path,
            }.items(),
        ),

        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_nav_dir, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
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
    ])
