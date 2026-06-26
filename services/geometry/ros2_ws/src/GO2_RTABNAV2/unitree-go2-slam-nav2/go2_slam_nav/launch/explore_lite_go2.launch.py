# GO2 autonomous exploration via m-explore (explore_lite).
# Requires nav_go2 (or explore_go2 without this node) already running.
#
#   ros2 launch go2_slam_nav explore_lite_go2.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_slam_nav')
    params_file = os.path.join(go2_share, 'config', 'explore_go2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='explore_lite parameter file',
        ),
        DeclareLaunchArgument(
            'return_to_init',
            default_value='false',
            choices=['true', 'false'],
            description='Return to start pose after exploration completes',
        ),

        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'return_to_init': LaunchConfiguration('return_to_init'),
                },
            ],
        ),
    ])