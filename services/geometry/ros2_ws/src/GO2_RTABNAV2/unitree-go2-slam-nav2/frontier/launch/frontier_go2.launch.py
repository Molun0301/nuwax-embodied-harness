# GO2 frontier exploration only — requires nav_go2 (or explore_go2 without this node) running.
#
#   ros2 launch frontier frontier_go2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('goal_topic', default_value='goal_pose'),
        DeclareLaunchArgument('base_frame', default_value='go2_base_link'),
        DeclareLaunchArgument(
            'navigate_status_topic',
            default_value='/navigate_to_pose/_action/status',
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('timer_period', default_value='5.0'),
        DeclareLaunchArgument('min_goal_distance', default_value='1.0'),
        DeclareLaunchArgument('sensor_fov_deg', default_value='100.0'),
        DeclareLaunchArgument('sensor_range', default_value='1.0'),
        DeclareLaunchArgument('downsample_factor', default_value='2'),

        Node(
            package='frontier',
            executable='explore_update2',
            name='frontier_exploration',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'navigate_status_topic': LaunchConfiguration('navigate_status_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'timer_period': LaunchConfiguration('timer_period'),
                'min_goal_distance': LaunchConfiguration('min_goal_distance'),
                'sensor_fov_deg': LaunchConfiguration('sensor_fov_deg'),
                'sensor_range': LaunchConfiguration('sensor_range'),
                'downsample_factor': LaunchConfiguration('downsample_factor'),
            }],
        ),
    ])