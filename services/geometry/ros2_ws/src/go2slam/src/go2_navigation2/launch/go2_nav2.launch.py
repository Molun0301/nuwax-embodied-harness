import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def create_localization_nodes(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    map_yaml_file = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    autostart = LaunchConfiguration("autostart").perform(context).lower() == "true"
    use_respawn = LaunchConfiguration("use_respawn").perform(context).lower() == "true"
    log_level = LaunchConfiguration("log_level").perform(context)

    return [
        SetParameter(name="use_sim_time", value=use_sim_time),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": ["map_server", "amcl"],
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description():
    nav2_pkg = get_package_share_directory("go2_navigation2")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    default_map_yaml = os.path.abspath(
        os.path.join(nav2_pkg, "..", "..", "..", "..", "maps", "map_1775398678.yaml")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map")
    localization_mode = LaunchConfiguration("localization")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_pcd_initial_pose = LaunchConfiguration("use_pcd_initial_pose")
    pcd_map = LaunchConfiguration("pcd_map")
    pcd_initial_pose_cloud_topic = LaunchConfiguration("pcd_initial_pose_cloud_topic")
    auto_spin_after_initial_pose = LaunchConfiguration("auto_spin_after_initial_pose")
    use_waypoint_follower = LaunchConfiguration("use_waypoint_follower")
    use_front_align_cmd_vel_gate = LaunchConfiguration("use_front_align_cmd_vel_gate")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam'"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    localization_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        actions=[
            OpaqueFunction(function=create_localization_nodes),
        ],
    )

    try:
        get_package_share_directory("nav2_smoother")
        has_nav2_smoother = True
    except PackageNotFoundError:
        has_nav2_smoother = False

    try:
        get_package_share_directory("nav2_behaviors")
        recovery_package = "nav2_behaviors"
        recovery_executable = "behavior_server"
        recovery_name = "behavior_server"
    except PackageNotFoundError:
        recovery_package = "nav2_recoveries"
        recovery_executable = "recoveries_server"
        recovery_name = "recoveries_server"

    nav_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        recovery_name,
        "bt_navigator",
    ]
    if has_nav2_smoother:
        nav_lifecycle_nodes.insert(2, "smoother_server")

    smoother_nodes = []
    if has_nav2_smoother:
        smoother_nodes.append(
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            )
        )

    nav2_nodes = GroupAction(
        actions=[
            SetParameter(name="use_sim_time", value=use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="go2_navigation2",
                executable="front_align_cmd_vel_gate",
                name="front_align_cmd_vel_gate",
                output="screen",
                condition=IfCondition(use_front_align_cmd_vel_gate),
                parameters=[
                    params_file,
                    {
                        "robot_base_frame": "base_footprint",
                        "lookahead_distance": 0.6,
                        "align_yaw_tolerance": 0.45,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package=recovery_package,
                executable=recovery_executable,
                name=recovery_name,
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            *smoother_nodes,
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": nav_lifecycle_nodes,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                condition=IfCondition(use_waypoint_follower),
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="go2_navigation2",
                executable="pcd_initial_pose_publisher",
                name="pcd_initial_pose_publisher",
                output="screen",
                condition=IfCondition(use_pcd_initial_pose),
                parameters=[
                    {
                        "pcd_path": pcd_map,
                        "cloud_topic": pcd_initial_pose_cloud_topic,
                        "map_frame": "map",
                        "base_frame": "base_footprint",
                        "initial_pose_topic": "/initialpose",
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="go2_navigation2",
                executable="initial_pose_spin_relocalizer",
                name="initial_pose_spin_relocalizer",
                output="screen",
                condition=IfCondition(auto_spin_after_initial_pose),
                parameters=[
                    {
                        "initial_pose_topic": "/initialpose",
                        "cmd_vel_topic": "/cmd_vel",
                        "rotations": 3.0,
                        "angular_speed": 0.5,
                        "start_delay_sec": 1.0,
                        "publish_rate_hz": 20.0,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        ),
        launch_arguments={"driver_use_rviz": "false"}.items(),
    )

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
        )
    )

    pointcloud_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
        )
    )

    explore_node = None

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=os.path.join(nav2_pkg, "config", "nav2_params.yaml")),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(go2_slam_pkg, "config", "mapper_params_online_async.yaml"),
            ),
            DeclareLaunchArgument("map", default_value=default_map_yaml),
            DeclareLaunchArgument("localization", default_value="amcl"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=os.path.join(nav2_pkg, "rviz", "nav2_map_view.rviz")),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_pcd_initial_pose", default_value="false"),
            DeclareLaunchArgument("pcd_map", default_value=""),
            DeclareLaunchArgument("pcd_initial_pose_cloud_topic", default_value="/utlidar/cloud_deskewed"),
            DeclareLaunchArgument("auto_spin_after_initial_pose", default_value="false"),
            DeclareLaunchArgument("use_waypoint_follower", default_value="false"),
            DeclareLaunchArgument(
                "use_front_align_cmd_vel_gate",
                default_value="true",
                description="Go2 cmd_vel 整形节点开关；其他机器人接入时通过 use_front_align_cmd_vel_gate:=false 关闭",
            ),
            robot_bringup,
            robot_localization,
            pointcloud_pipeline,
            localization_nodes,
            slam_launch,
            nav2_nodes,
            rviz_node,
        ]
    )
