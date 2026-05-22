"""
=============================================================================
GO2 机器狗 LiDAR SLAM Launch 文件
=============================================================================
功能：使用 RTAB-Map 对 Unitree GO2 机器狗的 L1 4D LiDAR 进行 SLAM 建图
数据源：
  - /utlidar/cloud_base  : 已转换到 base_link 坐标系的 3D 点云 (PointCloud2)
  - /utlidar/robot_odom  : GO2 自带的融合里程计 (nav_msgs/Odometry)
输出：
  - /map                 : 2D 栅格地图 (nav_msgs/OccupancyGrid)
  - /cloud_map           : 3D 全局点云地图 (sensor_msgs/PointCloud2)
  - /cloud_ground        : 地面点云
  - /cloud_obstacles     : 障碍物点云
  - TF: map -> odom      : 全局位姿校正（回环检测后更新）
  - TF: odom -> base_link: 实时里程计位姿（由 odom_to_tf.py 发布）

使用方法：
  ros2 launch go2_slam go2_lidar_slam.launch.py

前提条件：
  1. sudo apt install ros-foxy-rtabmap-ros
  2. GO2 的 LiDAR 驱动在运行（/utlidar/cloud_base 有数据）
  3. GO2 的里程计在运行（/utlidar/robot_odom 有数据）
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    database_path = LaunchConfiguration('database_path')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    incremental_memory = LaunchConfiguration('incremental_memory')
    init_wm_with_all_nodes = LaunchConfiguration('init_wm_with_all_nodes')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    planar_odom = LaunchConfiguration('planar_odom')

    # =========================================================================
    # 节点1: odom_to_tf — 里程计转 TF 广播器
    # =========================================================================
    # 作用：GO2 的 /utlidar/robot_odom 只发布 Odometry 消息，不发布 TF。
    #       但 rtabmap 需要 odom -> base_link 的 TF 变换才能工作。
    #       这个 Python 脚本订阅 /utlidar/robot_odom，把位姿转成 TF 广播出去。
    odom_to_tf = ExecuteProcess(
        cmd=['python3', os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'odom_to_tf.py'),
            '--odom-topic', odom_topic,
            '--odom-frame', odom_frame,
            '--base-frame', base_frame,
            '--planar', planar_odom],
        output='screen',
    )

    # =========================================================================
    # 节点2: static_transform_publisher — 静态 TF 发布
    # =========================================================================
    # 作用：发布 base_link -> utlidar_lidar 的静态变换。
    # 为什么是单位变换（全0）？
    #   因为我们使用的是 /utlidar/cloud_base 这个 topic，它的点云已经被
    #   GO2 的驱动转换到了 base_link 坐标系，所以 LiDAR frame 和 base_link
    #   之间不需要额外旋转。
    # 参数格式（Foxy）：x y z yaw pitch roll parent_frame child_frame
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, lidar_frame],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # =========================================================================
    # 节点3: rtabmap — SLAM 核心节点
    # =========================================================================
    # RTAB-Map 在这里只做两件事：
    #   1. 接收点云 + 里程计，构建 2D/3D 地图
    #   2. 检测回环（机器人回到之前去过的地方），修正累积误差
    rtabmap_node = Node(
    package='rtabmap_slam',
    executable='rtabmap',
    name='rtabmap',
    output='screen',
    arguments=[rtabmap_args],
    parameters=[{
        'use_sim_time': use_sim_time,
        'database_path': database_path,
        'frame_id': base_frame,
        'odom_frame_id': odom_frame,
        'map_frame_id': map_frame,
        'subscribe_scan_cloud': True,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'tf_tolerance': 1.0,
        'wait_for_transform': 1.0,
        # Unitree L1 publishes faster than 3 Hz; accepting more scan clouds
        # makes the assembled RTAB-Map cloud less sparse while staying modest.
        'Rtabmap/DetectionRate': '8.0',
        'Rtabmap/CreateIntermediateNodes': 'true',
        'Reg/Strategy': '1',
        # GO2 is a ground robot. Keep the localization graph planar
        # (x/y/yaw) so z/roll/pitch noise from odom/ICP cannot accumulate as
        # apparent global drift. The local occupancy/grid data below remains 3D.
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityGlobalScanMap': 'true',
        'RGBD/ProximityOdomGuess': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/StartAtOrigin': 'false',
        'RGBD/LocalImmunizationRatio': '0.25',
        'Mem/IncrementalMemory': ParameterValue(incremental_memory, value_type=str),
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Mem/InitWMWithAllNodes': ParameterValue(init_wm_with_all_nodes, value_type=str),
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Rtabmap/MaxRetrieved': '2',

        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',
        # Build local occupancy data in 3D so cloud_map/cloud_obstacles keep
        # height structure instead of being collapsed into a 2D layer.
        'Grid/3D': 'true',
        'Grid/RangeMax': '15.0',
        'Grid/RangeMin': '0.2',
        'Grid/CellSize': '0.05',
        'Grid/MaxGroundHeight': '0.12',
        'Grid/MinGroundHeight': '-0.6',
        'Grid/MaxObstacleHeight': '2.5',
        'Grid/NormalsSegmentation': 'true',
        'Grid/PreVoxelFiltering': 'false',
        'Grid/MinClusterSize': '1',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '0',
        'Grid/RayTracing': 'true',
        'Grid/Scan2dUnknownSpaceFilled': 'false',

        'GridGlobal/UpdateError': '0.01',
        'GridGlobal/ProbHit': '0.82',
        'GridGlobal/ProbMiss': '0.47',
        'GridGlobal/OccupancyThr': '0.45',
        'GridGlobal/ProbClampingMin': '0.1',
        'GridGlobal/ProbClampingMax': '0.96',
        'GridGlobal/FullUpdate': 'true',
        'GridGlobal/MinSize': '0.0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/FootprintRadius': '0.0',

        'Icp/VoxelSize': '0.05',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/CorrespondenceRatio': '0.25',
        'Icp/MaxTranslation': '0.5',
        'Icp/MaxRotation': '0.7',
        'Icp/Iterations': '20',

        'Optimizer/Strategy': '1',
        'Optimizer/GravitySigma': '0.3',
    }],
    remappings=[
        ('scan_cloud', point_cloud_topic),
        ('odom', odom_topic),
    ],
)
















    # =========================================================================
    # 启动顺序
    # =========================================================================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/.ros/go2_slam_rtabmap.db'),
        ),
        DeclareLaunchArgument('point_cloud_topic', default_value='/utlidar/cloud_base'),
        DeclareLaunchArgument('odom_topic', default_value='/utlidar/robot_odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='utlidar_lidar'),
        DeclareLaunchArgument('incremental_memory', default_value='true'),
        DeclareLaunchArgument('init_wm_with_all_nodes', default_value='false'),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('planar_odom', default_value='true'),
        odom_to_tf,         # 1. 先启动里程计转TF，确保TF树完整
        static_tf_lidar,    # 2. 发布静态TF
        rtabmap_node,       # 3. 最后启动SLAM
    ])
