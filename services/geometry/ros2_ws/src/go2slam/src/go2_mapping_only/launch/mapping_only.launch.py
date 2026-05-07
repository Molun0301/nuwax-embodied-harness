from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz.',
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/utlidar/cloud_deskewed',
        description='Deskewed point cloud topic from the robot.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/utlidar/robot_odom',
        description='Built-in robot odometry topic.',
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/go2_built_map',
        description='Accumulated map topic.',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/go2_robot_path',
        description='Robot path topic.',
    )
    output_frame_id_arg = DeclareLaunchArgument(
        'output_frame_id',
        default_value='map',
        description='Frame id used for the published accumulated point-cloud map and path.',
    )
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/home/huang/xxx/output/go2_built_map.pcd',
        description='PCD output file path.',
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.10',
        description='Voxel leaf size in meters. Use 0 or a negative value to disable voxel filtering.',
    )
    publish_every_n_scans_arg = DeclareLaunchArgument(
        'publish_every_n_scans',
        default_value='5',
        description='Publish the accumulated map every N scans. Use 0 to disable periodic publishing.',
    )
    downsample_every_n_scans_arg = DeclareLaunchArgument(
        'downsample_every_n_scans',
        default_value='3',
        description='Apply voxel downsampling every N scans. Use 0 to disable periodic downsampling.',
    )
    enable_icp_z_correction_arg = DeclareLaunchArgument(
        'enable_icp_z_correction',
        default_value='true',
        description='Whether to compensate stale odometry z with ICP between scans.',
    )

    map_builder_node = Node(
        package='go2_mapping_only',
        executable='go2_map_builder',
        name='go2_map_builder',
        output='screen',
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'output_frame_id': LaunchConfiguration('output_frame_id'),
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
            'publish_every_n_scans': LaunchConfiguration('publish_every_n_scans'),
            'downsample_every_n_scans': LaunchConfiguration('downsample_every_n_scans'),
            'enable_icp_z_correction': LaunchConfiguration('enable_icp_z_correction'),
            'save_path': LaunchConfiguration('save_path'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('go2_mapping_only'),
            'rviz_cfg',
            'mapping_only.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        cloud_topic_arg,
        odom_topic_arg,
        map_topic_arg,
        path_topic_arg,
        output_frame_id_arg,
        save_path_arg,
        voxel_leaf_size_arg,
        publish_every_n_scans_arg,
        downsample_every_n_scans_arg,
        enable_icp_z_correction_arg,
        map_builder_node,
        rviz_node,
    ])
