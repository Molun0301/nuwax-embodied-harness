# GO2: SLAM + Nav2 + m-explore (explore_lite) autonomous exploration.
#
# Source:
#   source /opt/ros/jazzy/setup.bash
#   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
#   source ~/huang_grok/ws/install/setup.bash
#
# Full stack (mapping + nav + exploration):
#   ros2 launch go2_slam_nav explore_go2.launch.py restart_map:=true
#
# Exploration only (nav_go2 already running in another terminal):
#   ros2 launch go2_slam_nav explore_lite_go2.launch.py
#
# Kill stale nodes:
#   pgrep -f 'rtabmap|nav2_|sport_ctrl|sensor_stamp_relay|go2_tf_relay|explore_node|obstacle_grid|explore_preroll' | xargs -r kill -9

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    go2_share = get_package_share_directory('go2_slam_nav')
    explore_params = os.path.join(go2_share, 'config', 'explore_go2.yaml')

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go2_slam_nav'),
                'launch',
                'explore_lite_go2.launch.py',
            ])
        ),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('return_to_init', LaunchConfiguration('return_to_init')),
            ('params_file', explore_params),
        ],
    )

    preroll_node = Node(
        package='go2_slam_nav',
        executable='explore_preroll',
        name='explore_preroll',
        output='screen',
        parameters=[{
            'distance_m': LaunchConfiguration('explore_preroll_distance'),
            'linear_speed': LaunchConfiguration('explore_preroll_speed'),
            'scan_rotation_rad': LaunchConfiguration('explore_preroll_scan_rad'),
            'odom_topic': LaunchConfiguration('explore_preroll_odom_topic'),
            'cmd_vel_topic': '/cmd_vel',
            'nav2_wait_timeout': 90.0,
            'nav2_stabilize_sec': 8.0,
            'nav2_check_period_sec': 1.0,
        }],
        condition=IfCondition(LaunchConfiguration('explore_preroll')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'localize_only', default_value='false', choices=['true', 'false'],
            description='false=SLAM+explore (default); true=fixed map only',
        ),
        DeclareLaunchArgument(
            'restart_map', default_value='false', choices=['true', 'false'],
            description='true=delete rtabmap DB and restart',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'nav2_startup_delay', default_value='20.0',
            description='Wait before starting Nav2 (seconds; Jetson needs headroom after SLAM)',
        ),
        DeclareLaunchArgument(
            'explore_startup_delay', default_value='30.0',
            description='Wait before starting explore_lite when preroll is disabled',
        ),
        DeclareLaunchArgument(
            'explore_preroll', default_value='true', choices=['true', 'false'],
            description='Drive forward before explore_lite to prime SLAM map',
        ),
        DeclareLaunchArgument(
            'explore_preroll_delay', default_value='30.0',
            description='Wait before starting preroll (seconds from launch; after Nav2 has ~10s to activate)',
        ),
        DeclareLaunchArgument(
            'explore_preroll_distance', default_value='1.0',
            description='Forward distance in meters before in-place rotation scan',
        ),
        DeclareLaunchArgument(
            'explore_preroll_speed', default_value='0.3',
            description='Forward speed during preroll (m/s, matches sport_ctrl floor)',
        ),
        DeclareLaunchArgument(
            'explore_preroll_scan_rad', default_value='6.28',
            description='In-place rotation after forward preroll (rad; 6.28≈360°)',
        ),
        DeclareLaunchArgument(
            'explore_preroll_odom_topic', default_value='/go2/odom',
            description='Odometry topic for preroll distance integration',
        ),
        DeclareLaunchArgument(
            'return_to_init', default_value='false', choices=['true', 'false'],
            description='Return to start pose after exploration completes',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            choices=['debug', 'info', 'warn', 'error', 'fatal'],
        ),
        DeclareLaunchArgument(
            'cloud_in', default_value='/utlidar/cloud_deskewed',
        ),
        DeclareLaunchArgument(
            'cloud_frame_mode', default_value='deskewed',
            choices=['deskewed', 'base', 'raw'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'), 'launch', 'nav_go2.launch.py',
                ])
            ),
            launch_arguments=[
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('nav2_startup_delay', LaunchConfiguration('nav2_startup_delay')),
                ('log_level', LaunchConfiguration('log_level')),
                ('use_obstacle_grid', 'true'),
                ('cloud_in', LaunchConfiguration('cloud_in')),
                ('cloud_frame_mode', LaunchConfiguration('cloud_frame_mode')),
                ('slam_mode', 'explore'),
            ],
        ),

        TimerAction(
            period=LaunchConfiguration('explore_preroll_delay'),
            actions=[preroll_node],
            condition=IfCondition(LaunchConfiguration('explore_preroll')),
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=preroll_node,
                on_exit=[explore_lite_launch],
            ),
            condition=IfCondition(LaunchConfiguration('explore_preroll')),
        ),

        TimerAction(
            period=LaunchConfiguration('explore_startup_delay'),
            actions=[explore_lite_launch],
            condition=UnlessCondition(LaunchConfiguration('explore_preroll')),
        ),
    ])