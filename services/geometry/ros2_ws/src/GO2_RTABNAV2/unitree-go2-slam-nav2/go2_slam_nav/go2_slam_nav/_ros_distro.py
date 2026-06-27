"""ROS distro helpers for dual Jazzy/Foxy launch files."""

import os


def ros_distro() -> str:
    return os.environ.get('ROS_DISTRO', 'foxy').strip().lower()


def is_foxy() -> bool:
    return ros_distro() == 'foxy'


def nav2_params_basename() -> str:
    return 'nav2_params_go2_foxy.yaml' if is_foxy() else 'nav2_params_go2.yaml'