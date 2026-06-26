# GO2 real robot: LiDAR-only RTAB-Map mapping entry point.
#
#   source /opt/ros/jazzy/setup.bash
#   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
#   source ~/huang_grok/ws/install/setup.bash
#   ros2 pkg prefix go2_slam_nav   # must be ~/huang_grok/ws/install/go2_slam_nav
#   ros2 launch go2_slam_nav mapping_go2.launch.py
# Opens RViz2 by default (2D /map + 3D MapCloud). rtabmap_viz is off by default.
#
# Fresh map:
#   ros2 launch go2_slam_nav mapping_go2.launch.py restart_map:=true
#
# Stop stale nodes before relaunch (run as separate commands; do NOT chain pkill
# with ros2 launch in one line — pkill may match and kill the launcher shell):
#   pgrep -f 'lib/go2_slam_nav/sensor_stamp_relay' | xargs -r kill -9
#   pgrep -f 'rtabmap' | xargs -r kill -9
#
# Pure RTAB-Map ICP odometry (instead of GO2 built-in odom):
#   ros2 launch go2_slam_nav mapping_go2.launch.py odom_source:=icp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rtabmapviz', default_value='false',
            choices=['true', 'false'],
            description='Start rtabmap_viz (default off; use RViz instead)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            choices=['true', 'false'],
            description='Open RViz2 for 2D /map visualization',
        ),
        DeclareLaunchArgument(
            'localize_only', default_value='false',
            choices=['true', 'false'],
            description='Localize only, do not grow the map',
        ),
        DeclareLaunchArgument(
            'restart_map', default_value='true',
            choices=['true', 'false'],
            description='Delete previous RTAB-Map database and restart',
        ),
        DeclareLaunchArgument(
            'odom_source', default_value='robot',
            choices=['robot', 'icp'],
            description='Use GO2 built-in odom or RTAB-Map ICP odometry',
        ),
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            choices=['true', 'false'],
            description='Enable LiDAR deskewing when odom_source:=icp',
        ),
        DeclareLaunchArgument(
            'use_stamp_relay', default_value='true',
            choices=['true', 'false'],
            description='Re-stamp GO2 sensor messages to local ROS time',
        ),
        DeclareLaunchArgument(
            'cloud_in', default_value='/utlidar/cloud_deskewed',
            description='GO2 cloud input (cloud_deskewed + robot_odom)',
        ),
        DeclareLaunchArgument(
            'cloud_frame_mode', default_value='deskewed',
            choices=['deskewed', 'base', 'raw'],
            description='deskewed: odom->body transform (default); base: cloud_base passthrough',
        ),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/go2/cloud',
            description='LiDAR cloud topic consumed by RTAB-Map',
        ),
        DeclareLaunchArgument(
            'use_obstacle_grid', default_value='true',
            choices=['true', 'false'],
            description='Route B: fuse /cloud_obstacles into /map_obstacles (2D/3D aligned)',
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            choices=['mapping', 'explore', 'localization'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'),
                    'launch',
                    'rtab_lidar_go2.launch.py',
                ])
            ),
            launch_arguments=[
                ('use_rtabmapviz', LaunchConfiguration('use_rtabmapviz')),
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
                ('odom_source', LaunchConfiguration('odom_source')),
                ('deskewing', LaunchConfiguration('deskewing')),
                ('use_stamp_relay', LaunchConfiguration('use_stamp_relay')),
                ('cloud_in', LaunchConfiguration('cloud_in')),
                ('cloud_frame_mode', LaunchConfiguration('cloud_frame_mode')),
                ('lidar_topic', LaunchConfiguration('lidar_topic')),
                ('use_obstacle_grid', LaunchConfiguration('use_obstacle_grid')),
                ('slam_mode', LaunchConfiguration('slam_mode')),
            ],
        ),
    ])