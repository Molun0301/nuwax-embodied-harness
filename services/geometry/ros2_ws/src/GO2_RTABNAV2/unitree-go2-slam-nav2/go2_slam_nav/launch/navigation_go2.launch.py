# Minimal Nav2 for GO2: no route_server, docking_server, collision_monitor.
# Foxy: recoveries_server (no smoother / velocity_smoother).
# Jazzy+: behavior_server + velocity_smoother (matches upstream Nav2 bringup).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from go2_slam_nav._ros_distro import is_foxy


def _configured_params(params_file, namespace, autostart, use_sim_time):
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    if is_foxy():
        param_substitutions['default_bt_xml_filename'] = os.path.join(
            get_package_share_directory('go2_slam_nav'),
            'config', 'behavior_trees', 'navigate_to_pose_go2_foxy.xml',
        )
    return RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )


def _foxy_nodes(configured_params, remappings, autostart, use_sim_time, log_level):
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ]


def _jazzy_nodes(configured_params, remappings, autostart, log_level, go2_bt_params):
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]
    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, go2_bt_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    go2_slam_nav_dir = get_package_share_directory('go2_slam_nav')
    bt_dir = os.path.join(go2_slam_nav_dir, 'config', 'behavior_trees')
    go2_bt_params = {
        'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose_go2.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            bt_dir, 'navigate_through_poses_go2.xml'),
    }

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    configured_params = _configured_params(params_file, namespace, autostart, use_sim_time)

    nav_nodes = (
        _foxy_nodes(configured_params, remappings, autostart, use_sim_time, log_level)
        if is_foxy()
        else _jazzy_nodes(configured_params, remappings, autostart, log_level, go2_bt_params)
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        GroupAction(nav_nodes),
    ])