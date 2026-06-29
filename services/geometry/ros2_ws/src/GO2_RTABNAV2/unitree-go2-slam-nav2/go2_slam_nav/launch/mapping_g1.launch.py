# G1 mapping 模式 launch entry（Unitree G1 + Livox Mid-360 + 3D RTAB-Map）。
#
# 与 mapping_go2.launch.py 同结构（IncludeLaunchDescription rtab_lidar_go2.launch.py），
# 但物理量参数与 RTAB-Map 模式按 G1 实机话题清单（2026-06-29）覆盖：
#   - lidar topic / imu topic / odom topic / base frame / lidar frame 从 ROS env 注入
#   - RTAB-Map 走 3D 模式（subscribe_scan_cloud + qos=BEST_EFFORT + Reg/Force3DoF=false）
#   - subscribe_imu=true（Livox Mid-360 内置 IMU 作 gravity 约束）
#
# v4 D4 决策：详见 docs/开发文档/Ralplan/g1-rtabnav2-适配/方案.md 与
#             openspec/changes/g1-rtabnav2-adapter/{proposal,design,specs/...}.md。
#
# 物理量注入路径：scripts/start_gateway.sh → NUWAX_ACTIVE_ROBOT=g1 → source .env.G1_example
#                 → 覆盖 NUWAX_GEOMETRY_* 键 → 本 launch 通过 os.environ 读入 LaunchConfiguration。
#
# 用法：
#   cd ~/nuwax_robot_bridge/services/geometry/ros2_ws/src/GO2_RTABNAV2
#   ROBOT=g1 ./robot_slam.sh --mapping
# （scripts/start_gateway.sh 已在启动前 source .env.G1_example；本脚本读取 ROS env）

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _env_or_default(env_key, default):
    return os.environ.get(env_key, default)


def generate_launch_description():
    # 物理量从 ROS env 注入（不允许 default_value 写死机器人物理量——OpenSpec spec.g1-rtabnav2-adapter）
    cloud_in = _env_or_default('NUWAX_GEOMETRY_POINT_CLOUD_TOPIC', '/utlidar/cloud_livox_mid360')
    odom_topic = _env_or_default('NUWAX_GEOMETRY_ODOM_TOPIC', '/unitree/slam_mapping/odom')
    imu_topic = _env_or_default('NUWAX_GEOMETRY_IMU_TOPIC', '/utlidar/imu_livox_mid360')
    base_frame = _env_or_default('NUWAX_GEOMETRY_BASE_FRAME', 'pelvis')
    lidar_frame = _env_or_default('NUWAX_GEOMETRY_LIDAR_FRAME', 'livox_mid360_link')

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
        # v4 D4: G1 端默认走 robot odom（/unitree/slam_mapping/odom 已带 6Dof）
        DeclareLaunchArgument(
            'odom_source', default_value='robot',
            choices=['robot', 'icp'],
            description='Use Unitree SLAM odom or RTAB-Map ICP odometry',
        ),
        # v4 D4: Livox Mid-360 不提供 deskewed；走 robot odom 时不需 deskewing
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            choices=['true', 'false'],
            description='Enable LiDAR deskewing when odom_source:=icp',
        ),
        DeclareLaunchArgument(
            'use_stamp_relay', default_value='true',
            choices=['true', 'false'],
            description='Re-stamp G1 sensor messages to local ROS time',
        ),
        # G1 物理量：从 ROS env 注入（v4 决策）
        DeclareLaunchArgument('cloud_in', default_value=cloud_in),
        DeclareLaunchArgument(
            'cloud_frame_mode', default_value='base',
            choices=['deskewed', 'base', 'raw'],
            description='G1 用 base（无 deskewed 版）；deskewing 留空',
        ),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/g1/cloud',
            description='G1 cloud topic consumed by RTAB-Map (sensor_stamp_relay 输出)',
        ),
        DeclareLaunchArgument(
            'imu_topic', default_value=imu_topic,
            description='G1 IMU topic (Livox Mid-360 builtin IMU)',
        ),
        DeclareLaunchArgument(
            'odom_topic', default_value=odom_topic,
            description='G1 odom topic (Unitree SLAM mapping output)',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value=lidar_frame,
            description='G1 lidar frame (livox_ros_driver2 default)',
        ),
        DeclareLaunchArgument(
            'base_frame_id', default_value=base_frame,
            description='G1 robot base frame (pelvis, URDF)',
        ),
        DeclareLaunchArgument(
            'use_obstacle_grid', default_value='true',
            choices=['true', 'false'],
            description='Fuse /cloud_obstacles into /map_obstacles',
        ),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            choices=['mapping', 'explore', 'localization'],
        ),
        # v4 D4: G1 端 qos=BEST_EFFORT (livox_ros_driver2 默认)
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='G1 RTAB-Map QoS (2=BEST_EFFORT, livox_ros_driver2)',
        ),
        DeclareLaunchArgument(
            'subscribe_imu', default_value='true',
            choices=['true', 'false'],
            description='G1 RTAB-Map subscribe IMU (Mid-360 builtin IMU)',
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
                # 透传到 rtab_lidar_go2.launch.py（v4 D4 G1 专属参数）
                ('imu_topic', LaunchConfiguration('imu_topic')),
                ('odom_topic', LaunchConfiguration('odom_topic')),
                ('lidar_frame_id', LaunchConfiguration('lidar_frame_id')),
                ('base_frame_id', LaunchConfiguration('base_frame_id')),
                ('qos', LaunchConfiguration('qos')),
                ('subscribe_imu', LaunchConfiguration('subscribe_imu')),
            ],
        ),
    ])