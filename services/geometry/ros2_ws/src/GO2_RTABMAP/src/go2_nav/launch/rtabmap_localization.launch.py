"""
RTAB-Map localization launch for Unitree GO2.
Loads an existing database (built by rtabmap_mapping.launch.py) and runs in
LOCALIZATION mode: no new nodes are added to the map, ICP refines the pose
against stored scans, and map->odom TF is published once relocalized.

Pipeline mirrors mapping launch (same /utlidar/cloud_deskewed input, same
cloud_to_scan for Nav2 /scan), only RTAB-Map behavior differs.

Initial pose: send geometry_msgs/PoseWithCovarianceStamped on /initialpose
(RViz "2D Pose Estimate" button works). Without an explicit pose, RTAB-Map
tries to relocalize from the last saved pose.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    database_path = LaunchConfiguration('database_path',
        default=os.path.expanduser('~/rtabmap.db'))

    rtabmap_parameters = {
        'use_sim_time': use_sim_time,

        # Inputs - same as mapping launch
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': False,

        # Frame IDs
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # QoS
        'qos_scan': 0,
        'qos_odom': 1,

        # Sync
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,

        # TF tolerance
        'tf_tolerance': 1.0,
        'wait_for_transform': 1.0,

        # ============ LOCALIZATION MODE ============
        # Don't add new nodes to the map; just relocalize.
        'Mem/IncrementalMemory': 'false',
        # Load all nodes from the database into working memory on startup
        # so ICP can match against any stored location.
        'Mem/InitWMWithAllNodes': 'true',
        # Allow some short-term memory for transient localization smoothing.
        'Mem/STMSize': '30',
        # Don't start a new map on relocalization loss
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        # Don't keep nodes not linked in the graph
        'Mem/NotLinkedNodesKept': 'false',

        # Detection rate - can be higher in localization (lighter than SLAM)
        'Rtabmap/DetectionRate': '3.0',
        'Rtabmap/CreateIntermediateNodes': 'true',

        # ============ ICP for relocalization ============
        # Looser ICP than mapping: we need to converge from a possibly large
        # initial offset when relocalizing.
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '1.0',  # wider than mapping (0.2)
        'Icp/Iterations': '30',
        'Icp/PointToPlaneK': '20',
        'Icp/Epsilon': '0.001',

        # Registration - ICP, 2D constrained (quadruped on flat ground)
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',

        # ============ Relocalization tuning ============
        # Make proximity-based loop closure (no vision available) more
        # aggressive so we can relocalize quickly.
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',  # search whole graph
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/ProximityPathFilteringRadius': '1.0',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        # Don't snap to graph origin on startup; relocalize against stored scans.
        'RGBD/StartAtOrigin': 'false',
        'RGBD/LocalImmunizationRatio': '0.25',
        # Slightly lower loop-closure threshold for ICP-only mode
        'Rtabmap/LoopThr': '0.11',

        # ============ Grid map (same as mapping for consistency) ============
        # Even in localization mode, RTAB-Map publishes /map from the loaded
        # database. Grid params here control how it's rebuilt for display.
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',
        'Grid/3D': 'true',
        'Grid/CellSize': '0.10',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '8.0',
        'Grid/MaxGroundHeight': '-0.15',
        'Grid/MinGroundHeight': '-0.50',
        'Grid/MaxObstacleHeight': '1.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/PreVoxelFiltering': 'true',
        'Grid/CellSize_PreVoxel': '0.10',
        'Grid/MinClusterSize': '3',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '0',
        'Grid/RayTracing': 'true',
        'Grid/Scan2dUnknownSpaceFilled': 'false',

        # Global grid (same as mapping)
        'GridGlobal/UpdateError': '0.01',
        'GridGlobal/ProbHit': '0.85',
        'GridGlobal/ProbMiss': '0.45',
        'GridGlobal/OccupancyThr': '0.5',
        'GridGlobal/ProbClampingMin': '0.10',
        'GridGlobal/ProbClampingMax': '0.95',
        'GridGlobal/FullUpdate': 'true',
        'GridGlobal/MinSize': '0.0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/FootprintRadius': '0.0',

        # Optimizer
        'Optimizer/Strategy': '1',
        'Optimizer/GravitySigma': '0.3',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path',
            default_value=os.path.expanduser('~/rtabmap.db')),

        # Odom -> base_link TF broadcaster (from /utlidar/robot_odom)
        Node(
            package='go2_nav',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster',
            output='screen',
        ),

        # Convert cloud_deskewed (in odom frame) -> /scan in base_link.
        # Nav2 obstacle layer needs /scan; the cloud is shared with RTAB-Map.
        Node(
            package='go2_nav',
            executable='cloud_to_scan.py',
            name='cloud_to_scan',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/cloud_deskewed',
                'output_topic': '/scan',
                'target_frame': 'base_link',
                'min_height': -0.20,
                'max_height': 0.30,
                'range_min': 0.30,
                'range_max': 8.0,
                'angle_increment': 0.01,
            }],
        ),

        # RTAB-Map node in LOCALIZATION mode.
        # NOTE: no '--delete_db_on_start' here, so the database is preserved
        # and loaded on startup.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters, {'database_path': database_path}],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('odom', '/utlidar/robot_odom'),
            ],
        ),
    ])
