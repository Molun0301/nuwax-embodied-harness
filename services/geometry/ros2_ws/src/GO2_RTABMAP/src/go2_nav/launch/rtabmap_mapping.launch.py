"""
RTAB-Map mapping (SLAM) launch for Unitree GO2.

Topic/frame defaults come from NUWAX_GEOMETRY_* (loaded by go2.sh from platform .env).
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from geometry_launch_config import GeometryLaunchConfig  # noqa: E402


def generate_launch_description():
    cfg = GeometryLaunchConfig.from_env()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    database_path = LaunchConfiguration(
        'database_path', default=os.path.expanduser('~/rtabmap.db'))
    non_nav_output = 'log' if os.environ.get('GO2_NAV2_LOG_ONLY') == '1' else 'screen'

    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': False,
        'frame_id': cfg.base_frame,
        'odom_frame_id': cfg.odom_frame,
        'map_frame_id': cfg.map_frame,
        'qos_scan': 0,
        'qos_odom': 1,
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'tf_tolerance': 1.0,
        'wait_for_transform': 1.0,
        'Rtabmap/DetectionRate': '3.0',
        'Rtabmap/CreateIntermediateNodes': 'true',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/Iterations': '20',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',
        'Grid/3D': 'true',
        'Grid/CellSize': '0.10',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '8.0',
        'cloud_output_voxelized': True,
        'cloud_subtract_filtering': True,
        'cloud_subtract_filtering_min_neighbors': 2,
        # Slightly lenient ground/obstacle split: sparse 4-line lidar often tags
        # low hits (carpet, slope, body roll) as obstacles when MaxGroundHeight
        # is too tight. FlatObstacleDetected off avoids ground-bump false walls.
        'Grid/MaxGroundHeight': '-0.05',
        'Grid/MinGroundHeight': '-0.50',
        'Grid/MaxObstacleHeight': '1.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/PreVoxelFiltering': 'true',
        'Grid/CellSize_PreVoxel': '0.10',
        'Grid/MinClusterSize': '5',
        'Grid/FlatObstacleDetected': 'false',
        'Grid/NoiseFilteringRadius': '0.15',
        'Grid/NoiseFilteringMinNeighbors': '2',
        'Grid/RayTracing': 'true',
        'Grid/Scan2dUnknownSpaceFilled': 'false',
        'GridGlobal/UpdateError': '0.01',
        # Occupancy log-odds: balance rear blind-spot stickiness vs false walls.
        # ProbHit 0.80 needs ~2 hits before occupied (was 0.9 / 1 frame). Higher
        # OccupancyThr and noise/cluster filters above drop sparse ghost cells.
        'GridGlobal/ProbHit': '0.80',
        'GridGlobal/ProbMiss': '0.45',
        'GridGlobal/OccupancyThr': '0.55',
        'GridGlobal/ProbClampingMin': '0.15',
        'GridGlobal/ProbClampingMax': '0.95',
        'GridGlobal/FullUpdate': 'true',
        'GridGlobal/MinSize': '0.0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/FootprintRadius': '0.0',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/StartAtOrigin': 'false',
        'RGBD/LocalImmunizationRatio': '0.25',
        'Optimizer/Strategy': '1',
        'Optimizer/GravitySigma': '0.3',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/rtabmap.db')),

        Node(
            package='go2_nav',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster',
            output=non_nav_output,
            parameters=[{
                'odom_topic': cfg.odom_topic,
                'odom_frame': cfg.odom_frame,
                'base_frame': cfg.base_frame,
            }],
        ),

        Node(
            package='go2_nav',
            executable='cloud_to_scan.py',
            name='cloud_to_scan',
            output=non_nav_output,
            parameters=[{
                'input_topic': cfg.point_cloud_topic,
                'output_topic': cfg.scan_topic,
                'target_frame': cfg.base_frame,
                'min_height': -0.20,
                'max_height': 0.30,
                'range_min': 0.30,
                'range_max': 8.0,
                'angle_increment': 0.01,
            }],
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output=non_nav_output,
            parameters=[rtabmap_parameters, {'database_path': database_path}],
            remappings=[
                ('scan_cloud', cfg.point_cloud_topic),
                ('odom', cfg.odom_topic),
            ],
            arguments=['--delete_db_on_start'],
        ),
    ])
