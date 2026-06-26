# GO2 real robot: LiDAR-only RTAB-Map mapping using built-in /utlidar topics.
#
# Prerequisites (after `source ~/.bashrc`):
#   - /utlidar/cloud_deskewed (sensor_msgs/PointCloud2, frame: odom, GO2 deskewed)
#   - /utlidar/cloud_base      (sensor_msgs/PointCloud2, frame: base_link, gravity-leveled)
#   - /utlidar/cloud           (raw lidar frame, only if manual extrinsic transform is needed)
#
# Default: cloud_deskewed (motion-compensated) + robot_odom -> body frame.
#   - /utlidar/robot_odom   (nav_msgs/Odometry, odom -> base_link)
#   - TF: odom -> base_link, base_link -> utlidar_lidar (published by GO2 onboard)
#
# Usage:
#   ros2 launch go2_slam_nav mapping_go2.launch.py
#   ros2 launch go2_slam_nav mapping_go2.launch.py restart_map:=true
#   ros2 launch go2_slam_nav mapping_go2.launch.py odom_source:=icp deskewing:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# GO2 onboard topics use RELIABLE QoS (depth=1). RTAB-Map defaults to sensor QoS.
GO2_QOS = {
    'qos': 1,
    'qos_scan': 1,
    'qos_odom': 1,
    'qos_imu': 1,
    'topic_queue_size': 30,
    'sync_queue_size': 30,
    'approx_sync_max_interval': 0.2,
}

# Isolated TF tree: RTAB-Map nodes only listen to /go2/tf, avoiding onboard
# odom->base_link broadcasts that use stale GO2 timestamps (TF_OLD_DATA spam).
GO2_TF_REMAPPINGS = [
    ('/tf', '/go2/tf'),
    ('/tf_static', '/go2/tf_static'),
    ('tf', '/go2/tf'),
    ('tf_static', '/go2/tf_static'),
]

RTABMAP_TUNING_BASE = [
    # LiDAR-only: disable visual feature pipeline (suppress computeTransform warnings)
    'Vis/MaxFeatures', '0',
    'Kp/MaxFeatures', '0',
    'Kp/DetectorStrategy', '0',
    'RGBD/AngularUpdate', '0.02',
    'RGBD/LinearUpdate', '0.02',
    'RGBD/CreateOccupancyGrid', 'true',
    'Grid/Sensor', '0',
    'Grid/RangeMin', '0.3',
    'Grid/RangeMax', '20.0',
    'Grid/MaxGroundHeight', '0.04',
    'Grid/MaxObstacleHeight', '1.5',
    'Grid/RayTracing', 'true',
    'Mem/NotLinkedNodesKept', 'false',
    'Mem/STMSize', '30',
    'Mem/LaserScanNormalK', '20',
    'Reg/Strategy', '1',
    'Reg/Force3DoF', 'true',
    'Icp/PointToPlaneGroundNormalsUp', '0.9',
    'Icp/VoxelSize', '0.05',
    'Icp/RangeMin', '0.3',
    'Icp/PointToPlaneK', '20',
    'Icp/PointToPlaneRadius', '0',
    'Icp/PointToPlane', 'true',
    'Icp/Iterations', '10',
    'Icp/Epsilon', '0.001',
    'Icp/MaxTranslation', '2',
    'Icp/MaxCorrespondenceDistance', '0.5',
    'Icp/Strategy', '1',
    'Icp/OutlierRatio', '0.7',
    'Icp/CorrespondenceRatio', '0.2',
    'GridGlobal/MaxNodes', '0',
    'Optimizer/Iterations', '1',
]


def _proximity_tuning_args():
    """LiDAR proximity: local ICP loop closure + global scan map relocalization (all modes)."""
    return [
        # Local proximity loop closure (drift correction on re-visit)
        'RGBD/ProximityBySpace', 'true',
        # 0 = 全图搜索。50 走几圈后搜不到起点，漂移重影无法校正。
        'RGBD/ProximityMaxGraphDepth', '0',
        'RGBD/ProximityPathMaxNeighbors', '10',
        'RGBD/ProximityPathFilteringRadius', '1.0',
        'Rtabmap/LoopThr', '0.11',
        # Global assembled scan map (mapping / explore / localization)
        'RGBD/ProximityGlobalScanMap', 'true',
    ]


def _rtabmap_tuning_args(localize_only):
    return RTABMAP_TUNING_BASE + _proximity_tuning_args()


RTABMAP_CORE_PARAMS = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_scan_cloud': True,
    'approx_sync': True,
    'wait_for_transform': 0.5,
    'publish_tf': True,
    'map_frame_id': 'map',
    **GO2_QOS,
}

GO2_MAP_REMAPPINGS = [
    ('map', '/map'),
    ('map_updates', '/map_updates'),
    ('mapData', '/mapData'),
    ('mapGraph', '/mapGraph'),
    ('cloud_map', '/cloud_map'),
    ('cloud_ground', '/cloud_ground'),
    ('cloud_obstacles', '/cloud_obstacles'),
    # ROS2 相对话题 info -> /info；显式重映射为 /rtabmap/info 与文档/监控一致
    ('info', '/rtabmap/info'),
]


def _rtabmap_params(lidar_frame_id, odom_frame_id, use_sim_time):
    return {
        'frame_id': lidar_frame_id,
        'odom_frame_id': odom_frame_id,
        'use_sim_time': use_sim_time,
        **RTABMAP_CORE_PARAMS,
    }


def _mapping_mode_args(localize_only):
    return [
        'Mem/IncrementalMemory',
        PythonExpression([
            '"false" if "', localize_only, '" == "true" else "true"'
        ]),
        'Mem/InitWMWithAllNodes',
        PythonExpression([
            '"true" if "', localize_only, '" == "true" else "false"'
        ]),
        'Mem/LocalizationDataSaved',
        PythonExpression([
            '"false" if "', localize_only, '" == "true" else "true"'
        ]),
    ]


def generate_launch_description():
    localize_only = LaunchConfiguration('localize_only')
    odom_source = LaunchConfiguration('odom_source')
    restart_map = LaunchConfiguration('restart_map')
    deskewing = LaunchConfiguration('deskewing')
    use_rtabmapviz = LaunchConfiguration('use_rtabmapviz')

    use_icp = PythonExpression(["'", odom_source, "' == 'icp'"])
    use_robot = PythonExpression(["'", odom_source, "' == 'robot'"])
    use_deskewing = PythonExpression([
        "'", odom_source, "' == 'icp' and '", deskewing, "' == 'true'"
    ])
    keep_db = PythonExpression(["'", restart_map, "' == 'false'"])
    reset_db = PythonExpression(["'", restart_map, "' == 'true'"])
    viz_robot = PythonExpression([
        "'", use_rtabmapviz, "' == 'true' and '", odom_source, "' == 'robot'"
    ])
    viz_icp = PythonExpression([
        "'", use_rtabmapviz, "' == 'true' and '", odom_source, "' == 'icp'"
    ])
    robot_keep_db = PythonExpression([
        "'", odom_source, "' == 'robot' and '", restart_map, "' == 'false'"
    ])
    robot_reset_db = PythonExpression([
        "'", odom_source, "' == 'robot' and '", restart_map, "' == 'true'"
    ])
    icp_keep_db = PythonExpression([
        "'", odom_source, "' == 'icp' and '", restart_map, "' == 'false'"
    ])
    icp_reset_db = PythonExpression([
        "'", odom_source, "' == 'icp' and '", restart_map, "' == 'true'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
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
            'use_rtabmapviz', default_value='false',
            choices=['true', 'false'],
            description='rtabmap_viz GUI (default off; visualization is in RViz)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            choices=['true', 'false'],
            description='Open RViz2 for 2D /map and live point cloud',
        ),
        DeclareLaunchArgument(
            'use_obstacle_grid', default_value='true',
            choices=['true', 'false'],
            description='Fuse /cloud_obstacles into /map_obstacles for Nav2 and explore',
        ),
        DeclareLaunchArgument(
            'rtabmap_log_level', default_value='WARN',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            choices=['mapping', 'explore', 'localization'],
            description='Progress log label: mapping / explore / localization',
        ),
        DeclareLaunchArgument(
            'odom_source', default_value='robot',
            choices=['robot', 'icp'],
            description='robot: /utlidar/robot_odom; icp: RTAB-Map icp_odometry',
        ),
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            choices=['true', 'false'],
            description='Enable LiDAR deskewing (icp mode only)',
        ),
        DeclareLaunchArgument(
            'use_stamp_relay', default_value='true',
            choices=['true', 'false'],
            description='Re-stamp GO2 messages to local ROS time (required for RTAB-Map)',
        ),
        DeclareLaunchArgument(
            'cloud_in', default_value='/utlidar/cloud_deskewed',
            description='GO2 cloud input (cloud_deskewed + robot_odom)',
        ),
        DeclareLaunchArgument(
            'cloud_frame_mode', default_value='deskewed',
            choices=['deskewed', 'base', 'raw'],
            description='deskewed: cloud_deskewed in odom -> body (default); base: cloud_base passthrough',
        ),
        DeclareLaunchArgument(
            'transform_cloud_to_base', default_value='false',
            choices=['true', 'false'],
            description='Legacy alias for cloud_frame_mode:=raw',
        ),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/go2/cloud',
        ),
        DeclareLaunchArgument(
            'imu_topic', default_value='/utlidar/imu',
        ),
        DeclareLaunchArgument(
            'odom_topic', default_value='/go2/odom',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='go2_base_link',
            description='Robot base frame used by RTAB-Map (not raw lidar frame)',
        ),
        DeclareLaunchArgument(
            'base_frame_id', default_value='go2_base_link',
            description='Base frame for the isolated GO2 SLAM TF tree',
        ),
        DeclareLaunchArgument(
            'use_odom_bridge', default_value='false',
            choices=['true', 'false'],
        ),

        Node(
            package='go2_slam_nav',
            executable='sensor_stamp_relay',
            name='sensor_stamp_relay',
            output='screen',
            parameters=[{
                'cloud_in': LaunchConfiguration('cloud_in'),
                'cloud_out': '/go2/cloud',
                'odom_in': '/utlidar/robot_odom',
                'odom_out': '/go2/odom',
                'publish_odom_tf': True,
                'cloud_frame_mode': LaunchConfiguration('cloud_frame_mode'),
                'transform_cloud_to_base': LaunchConfiguration('transform_cloud_to_base'),
                'flatten_odom_3dof': True,
                'odom_frame_id': 'go2_odom',
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'deskewed_max_odom_delta': 0.25,
                'use_source_stamp': False,
                'deskewed_sync_odom': True,
            }],
            remappings=GO2_TF_REMAPPINGS,
            condition=IfCondition(LaunchConfiguration('use_stamp_relay')),
        ),

        # Only needed when relaying raw /utlidar/cloud with manual extrinsic correction.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.28945', '0', '-0.046825',
                '0', '0.9913405653290647', '0', '0.1313159683094573',
                'go2_base_link', 'utlidar_lidar',
            ],
            remappings=GO2_TF_REMAPPINGS,
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('cloud_frame_mode'), "' == 'raw' or '",
                LaunchConfiguration('transform_cloud_to_base'), "' == 'true'"
            ])),
        ),

        # L1 SDK IMU->LiDAR static TF (not published by GO2 onboard stack).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.007698', '0.014655', '-0.00667',
                '0', '0', '0',
                'utlidar_imu', 'utlidar_lidar',
            ],
            remappings=GO2_TF_REMAPPINGS,
            condition=IfCondition(use_deskewing),
        ),

        Node(
            package='go2_slam_nav',
            executable='odom_tf_bridge',
            name='odom_tf_bridge',
            output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('odom_topic'),
                'output_odom_topic': '/odom',
                'odom_frame_id': 'odom',
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'publish_tf': False,
            }],
            condition=IfCondition(LaunchConfiguration('use_odom_bridge')),
        ),

        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': 'icp_odom',
                'guess_frame_id': 'go2_odom',
                'publish_tf': True,
                'wait_for_transform': 0.5,
                'expected_update_rate': 10.0,
                'deskewing': LaunchConfiguration('deskewing'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                **GO2_QOS,
            }],
            remappings=GO2_TF_REMAPPINGS + [
                ('scan_cloud', LaunchConfiguration('lidar_topic')),
                ('imu', LaunchConfiguration('imu_topic')),
            ],
            arguments=[
                'Icp/VoxelSize', '0.05',
                'Icp/RangeMin', '0.3',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/MaxCorrespondenceDistance', '0.5',
                'Icp/MaxTranslation', '2',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.05',
                'OdomF2M/ScanMaxSize', '10000',
                'OdomF2M/BundleAdjustment', 'false',
            ],
            condition=IfCondition(use_icp),
        ),

        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            output='screen',
            parameters=[{
                'max_clouds': 10,
                'fixed_frame_id': '',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=GO2_TF_REMAPPINGS + [('cloud', 'odom_filtered_input_scan')],
            condition=IfCondition(use_icp),
        ),

        # RTAB-Map: robot odom, keep existing DB
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[_rtabmap_params(
                LaunchConfiguration('lidar_frame_id'),
                'go2_odom',
                LaunchConfiguration('use_sim_time'),
            )],
            remappings=GO2_TF_REMAPPINGS + GO2_MAP_REMAPPINGS + [
                ('scan_cloud', LaunchConfiguration('lidar_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
            condition=IfCondition(robot_keep_db),
            arguments=_mapping_mode_args(localize_only) + _rtabmap_tuning_args(localize_only) + [
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ],
        ),

        # RTAB-Map: robot odom, reset DB
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[_rtabmap_params(
                LaunchConfiguration('lidar_frame_id'),
                'go2_odom',
                LaunchConfiguration('use_sim_time'),
            )],
            remappings=GO2_TF_REMAPPINGS + GO2_MAP_REMAPPINGS + [
                ('scan_cloud', LaunchConfiguration('lidar_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
            condition=IfCondition(robot_reset_db),
            arguments=_mapping_mode_args(localize_only) + _rtabmap_tuning_args(localize_only) + [
                '--delete_db_on_start',
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ],
        ),

        # RTAB-Map: ICP odom, keep existing DB
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[_rtabmap_params(
                LaunchConfiguration('lidar_frame_id'),
                'icp_odom',
                LaunchConfiguration('use_sim_time'),
            )],
            remappings=GO2_TF_REMAPPINGS + GO2_MAP_REMAPPINGS + [
                ('scan_cloud', 'assembled_cloud'),
                ('odom', 'icp_odom'),
            ],
            condition=IfCondition(icp_keep_db),
            arguments=_mapping_mode_args(localize_only) + _rtabmap_tuning_args(localize_only) + [
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ],
        ),

        # RTAB-Map: ICP odom, reset DB
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[_rtabmap_params(
                LaunchConfiguration('lidar_frame_id'),
                'icp_odom',
                LaunchConfiguration('use_sim_time'),
            )],
            remappings=GO2_TF_REMAPPINGS + GO2_MAP_REMAPPINGS + [
                ('scan_cloud', 'assembled_cloud'),
                ('odom', 'icp_odom'),
            ],
            condition=IfCondition(icp_reset_db),
            arguments=_mapping_mode_args(localize_only) + _rtabmap_tuning_args(localize_only) + [
                '--delete_db_on_start',
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ],
        ),

        # rtabmap_viz: robot odom
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'odom_frame_id': 'go2_odom',
                'subscribe_odom_info': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=GO2_TF_REMAPPINGS + [
                ('scan_cloud', LaunchConfiguration('lidar_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
            condition=IfCondition(viz_robot),
        ),

        # rtabmap_viz: ICP odom
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'odom_frame_id': 'icp_odom',
                'subscribe_odom_info': True,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=GO2_TF_REMAPPINGS + [
                ('scan_cloud', 'odom_filtered_input_scan'),
                ('odom', 'icp_odom'),
            ],
            condition=IfCondition(viz_icp),
        ),

        Node(
            package='go2_slam_nav',
            executable='map_robot_pose_publisher',
            name='map_robot_pose_publisher',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': LaunchConfiguration('base_frame_id'),
                'output_topic': '/go2/pose_map',
                'publish_rate': 10.0,
            }],
        ),

        Node(
            package='go2_slam_nav',
            executable='rtabmap_progress_monitor',
            name='rtabmap_progress_monitor',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'info_topic': '/rtabmap/info',
                'info_topic_fallback': '/info',
                'heartbeat_sec': 15.0,
                'startup_grace_sec': 30.0,
                'slam_mode': LaunchConfiguration('slam_mode'),
            }],
        ),

        Node(
            package='go2_slam_nav',
            executable='obstacle_grid_projector',
            name='obstacle_grid_projector',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('go2_slam_nav'),
                    'config',
                    'obstacle_grid_go2.yaml',
                ),
            ],
            condition=IfCondition(LaunchConfiguration('use_obstacle_grid')),
        ),

        # RViz2: 2D occupancy grid (/map) + live point cloud (/go2/cloud)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'),
                    'config',
                    'mapping_go2.rviz',
                ]),
            ],
            remappings=GO2_TF_REMAPPINGS,
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])