from setuptools import setup
import os
from glob import glob

package_name = 'go2_slam_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'behavior_trees'),
            glob('config/behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bedigital',
    maintainer_email='bedigital@todo.todo',
    description='GO2 SLAM and Nav2 launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_bridge = go2_slam_nav.odom_tf_bridge:main',
            'sensor_stamp_relay = go2_slam_nav.sensor_stamp_relay:main',
            'go2_tf_relay = go2_slam_nav.go2_tf_relay:main',
            'obstacle_grid_projector = go2_slam_nav.obstacle_grid_projector:main',
            'explore_preroll = go2_slam_nav.explore_preroll:main',
            'rtabmap_progress_monitor = go2_slam_nav.rtabmap_progress_monitor:main',
            'nav_progress_monitor = go2_slam_nav.nav_progress_monitor:main',
            'nav_loop_guard = go2_slam_nav.nav_loop_guard:main',
            'rviz_goal_bridge = go2_slam_nav.rviz_goal_bridge:main',
            'go2_stack_health = go2_slam_nav.go2_stack_health:main',
            'map_robot_pose_publisher = go2_slam_nav.map_robot_pose_publisher:main',
        ],
    },
)