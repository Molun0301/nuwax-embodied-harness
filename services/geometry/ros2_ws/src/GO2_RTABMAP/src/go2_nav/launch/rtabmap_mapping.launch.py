"""
RTAB-Map mapping (SLAM) launch for Unitree GO2.

Pipeline:
  /utlidar/cloud_deskewed (motion-compensated 3D, in odom frame)
    ├──> RTAB-Map (subscribe_scan_cloud) -> builds graph + /map + /cloud_map
    └──> cloud_to_scan.py (tf2: odom->base_link) -> /scan -> Nav2 costmap

Both consumers share the same source cloud so /scan and /map are spatially
consistent. cloud_to_scan applies a horizontal Z slice in base_link frame
to extract a clean 2D LaserScan; RTAB-Map uses the full 3D cloud.

For localization-only (load existing db), use rtabmap_localization.launch.py.
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

        # Inputs - use /utlidar/cloud_deskewed (motion-compensated 3D cloud,
        # in lidar frame, preserves real Z/height info).
        # cloud_base might have tilt baked in wrong, flattening Z.
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

        # RTAB-Map core - 3Hz keeps CPU below 100%, prevents Nav2 timing issues
        'Rtabmap/DetectionRate': '3.0',
        'Rtabmap/CreateIntermediateNodes': 'true',

        # Memory
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',

        # ICP parameters - 3D scan_cloud uses point-to-plane (works better with 3D)
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/Iterations': '20',

        # Registration - ICP, 2D constrained
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',

        # Grid map parameters (3D cloud input, Livox L1 4-ring sparse cloud)
        # Key insight: 4-ring data is too sparse for normal estimation,
        # so we rely on pure height-based ground/obstacle separation.
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',
        # 3D voxel grid: preserves Z in /cloud_map, /cloud_obstacles, /cloud_ground.
        # Nav2's /map (2D) is still derived as a projection - works for navigation.
        # CellSize bumped to 0.10 to keep CPU low.
        'Grid/3D': 'true',
        'Grid/CellSize': '0.10',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '8.0',

        'cloud_output_voxelized': True,
        'cloud_subtract_filtering': True,
        'cloud_subtract_filtering_min_neighbors': 2,
        
        # Ground/obstacle by HEIGHT in base_link frame.
        # GO2: base_link ≈ body center (~30cm above ground).
        # Floor ≈ z=-0.30, body ≈ z=-0.10..+0.10, walls/obstacles span larger.
        'Grid/MaxGroundHeight': '-0.15',  # ground = z below -0.15
        'Grid/MinGroundHeight': '-0.50',
        'Grid/MaxObstacleHeight': '1.0',  # ignore ceiling above 1m
        # Disable normals segmentation - too sparse for L1 4-ring lidar
        'Grid/NormalsSegmentation': 'false',
        # Pre-voxel filter to consolidate sparse points
        'Grid/PreVoxelFiltering': 'true',
        'Grid/CellSize_PreVoxel': '0.10',
        # Loosen cluster filter (4-ring data forms small clusters naturally)
        'Grid/MinClusterSize': '3',
        'Grid/FlatObstacleDetected': 'true',
        # Loosen noise filter (sparse legit points shouldn't be rejected)
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '0',
        'Grid/RayTracing': 'true',
        # Not applicable for scan_cloud input
        'Grid/Scan2dUnknownSpaceFilled': 'false',

        # Global grid parameters
        # Sticky obstacles: hits accumulate to high log-odds, misses are small,
        # so a brief in-place rotation can't clear walls. Real moving obstacles
        # still get cleared after sustained free-space observation (~5s).
        # log-odds: hit=+1.73, miss=-0.20, clampMax=+2.94
        #   → need ~15 consecutive misses to clear a saturated obstacle (~5s @ 3Hz)
        'GridGlobal/UpdateError': '0.01',
        'GridGlobal/ProbHit': '0.85',    # +1.73 log-odds per hit
        'GridGlobal/ProbMiss': '0.45',   # -0.20 log-odds per miss (slow clear)
        'GridGlobal/OccupancyThr': '0.5',
        'GridGlobal/ProbClampingMin': '0.10',
        'GridGlobal/ProbClampingMax': '0.95',  # +2.94 log-odds saturation
        'GridGlobal/FullUpdate': 'true',
        'GridGlobal/MinSize': '0.0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/FootprintRadius': '0.0',

        # Loop closure / proximity
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/StartAtOrigin': 'false',
        'RGBD/LocalImmunizationRatio': '0.25',

        # Optimizer
        'Optimizer/Strategy': '1',
        'Optimizer/GravitySigma': '0.3',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path',
            default_value=os.path.expanduser('~/rtabmap.db')),

        # Odom -> base_link TF broadcaster
        Node(
            package='go2_nav',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster',
            output='screen',
        ),

        # NOTE: We rely on Unitree's driver to publish base_link -> utlidar_lidar
        # TF (with the real mount translation + tilt). If RTAB-Map reports
        # "Could not get transform from base_link to utlidar_lidar" then we
        # need to add it back here with proper values. Quick check:
        #   ros2 run tf2_ros tf2_echo base_link utlidar_lidar

        # Convert cloud_deskewed (motion-compensated, in odom frame) to /scan
        # in base_link frame. cloud_to_scan.py uses tf2 to transform odom->base_link.
        # IMPORTANT: must use the SAME input as RTAB-Map (cloud_deskewed) so
        # /scan and /map are spatially consistent.
        Node(
            package='go2_nav',
            executable='cloud_to_scan.py',
            name='cloud_to_scan',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/cloud_deskewed',
                'output_topic': '/scan',
                'target_frame': 'base_link',
                # Narrow horizontal slice for clean wall detection.
                # In base_link frame: floor ≈ -0.30, ceiling > +0.50.
                'min_height': -0.20,
                'max_height': 0.30,
                'range_min': 0.30,
                'range_max': 8.0,
                'angle_increment': 0.01,  # ~0.57°
            }],
        ),

        # RTAB-Map SLAM node (subscribes to 3D scan cloud)
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
            arguments=['--delete_db_on_start'],
        ),
    ])
