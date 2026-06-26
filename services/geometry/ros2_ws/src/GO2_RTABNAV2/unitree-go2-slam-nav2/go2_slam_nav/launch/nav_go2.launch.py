# GO2: SLAM + Nav2 + sport_ctrl (default: map while navigating).
#
# Source (use huang_grok/ws, NOT go2_slam_nav_ws):
#   source /opt/ros/jazzy/setup.bash
#   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
#   source ~/huang_grok/ws/install/setup.bash
#
# Default — SLAM + navigate (continue existing map):
#   ros2 launch go2_slam_nav nav_go2.launch.py
#
# Fresh map from scratch (deletes ~/.ros/rtabmap.db):
#   ros2 launch go2_slam_nav nav_go2.launch.py restart_map:=true
#
# Mapping only (no Nav2, no costmap):
#   ros2 launch go2_slam_nav mapping_go2.launch.py restart_map:=true
#
# Navigate on fixed map only (no map growth):
#   ros2 launch go2_slam_nav nav_go2.launch.py localize_only:=true
#
# Kill stale nodes first:
#   pgrep -f 'rtabmap|nav2_|sport_ctrl|sensor_stamp_relay|go2_tf_relay|obstacle_grid' | xargs -r kill -9

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_slam_nav._ros_distro import nav2_params_basename


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('go2_slam_nav'),
        'config', nav2_params_basename(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'localize_only', default_value='false', choices=['true', 'false'],
            description='false=SLAM+nav (default); true=locate only, no map growth',
        ),
        DeclareLaunchArgument(
            'restart_map', default_value='false', choices=['true', 'false'],
            description='true=delete rtabmap DB and restart',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'nav2_startup_delay', default_value='15.0',
            description='Wait before starting Nav2 (seconds; allow /map_obstacles + lifecycle)',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            choices=['debug', 'info', 'warn', 'error', 'fatal'],
        ),
        DeclareLaunchArgument(
            'use_obstacle_grid', default_value='true',
            choices=['true', 'false'],
            description='Fuse /cloud_obstacles into /map_obstacles',
        ),
        DeclareLaunchArgument(
            'cloud_in', default_value='/utlidar/cloud_deskewed',
            description='GO2 lidar input topic for sensor_stamp_relay',
        ),
        DeclareLaunchArgument(
            'cloud_frame_mode', default_value='deskewed',
            choices=['deskewed', 'base', 'raw'],
            description='deskewed=cloud_deskewed+odom (default); base=cloud_base passthrough',
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            choices=['mapping', 'explore', 'localization'],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'), 'launch', 'mapping_go2.launch.py',
                ])
            ),
            launch_arguments=[
                ('use_rtabmapviz', 'false'),
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
                ('use_obstacle_grid', LaunchConfiguration('use_obstacle_grid')),
                ('cloud_in', LaunchConfiguration('cloud_in')),
                ('cloud_frame_mode', LaunchConfiguration('cloud_frame_mode')),
                ('slam_mode', LaunchConfiguration('slam_mode')),
            ],
        ),

        Node(
            package='go2_slam_nav',
            executable='go2_tf_relay',
            name='go2_tf_relay',
            output='screen',
        ),

        Node(
            package='go2_cmd_processor',
            executable='sport_ctrl',
            name='sport_ctrl',
            output='screen',
            parameters=[{
                'log_level': LaunchConfiguration('log_level'),
                'min_linear_speed': 0.08,
                'min_lateral_speed': 0.0,
                'allow_lateral': False,
                'min_angular_speed': 0.08,
            }],
        ),

        TimerAction(
            period=LaunchConfiguration('nav2_startup_delay'),
            actions=[
                Node(
                    package='go2_slam_nav',
                    executable='nav_progress_monitor',
                    name='nav_progress_monitor',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'info'],
                    parameters=[{
                        'slam_mode': LaunchConfiguration('slam_mode'),
                        'heartbeat_sec': 8.0,
                        'frontier_stats_log_sec': 30.0,
                    }],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('go2_slam_nav'),
                            'launch',
                            'navigation_go2.launch.py',
                        ])
                    ),
                    launch_arguments=[
                        ('use_sim_time', LaunchConfiguration('use_sim_time')),
                        ('params_file', nav2_params),
                        ('autostart', 'true'),
                        ('log_level', LaunchConfiguration('log_level')),
                    ],
                ),
            ],
        ),
    ])