#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Go2 关节配置。
定义关节索引和安全限制。
"""

from settings import APP_CONFIG

# 关节ID映射 (来自 unitree_legged_const.py)
LegID = {
    "FR_0": 0,
    "FR_1": 1,
    "FR_2": 2,
    "FL_0": 3,
    "FL_1": 4,
    "FL_2": 5,
    "BR_0": 6,
    "BR_1": 7,
    "BR_2": 8,
    "BL_0": 9,
    "BL_1": 10,
    "BL_2": 11,
    "FR": 0,
    "FL": 3,
    "BR": 6,
    "BL": 9,
}

LegID_reverse = {value: key for key, value in LegID.items()}

JOINT_NAMES = [
    "FR_0",
    "FR_1",
    "FR_2",
    "FL_0",
    "FL_1",
    "FL_2",
    "BR_0",
    "BR_1",
    "BR_2",
    "BL_0",
    "BL_1",
    "BL_2",
]

SAFETY_LIMITS = {
    "FR_0": [-1.5, 1.5],
    "FR_1": [-0.2, 2.5],
    "FR_2": [-2.8, -0.5],
    "FL_0": [-1.5, 1.5],
    "FL_1": [-0.2, 2.5],
    "FL_2": [-2.8, -0.5],
    "BR_0": [-1.5, 1.5],
    "BR_1": [-0.2, 2.5],
    "BR_2": [-2.8, -0.5],
    "BL_0": [-1.5, 1.5],
    "BL_1": [-0.2, 2.5],
    "BL_2": [-2.8, -0.5],
}

DEFAULT_KP = APP_CONFIG.low_level.default_kp
DEFAULT_KD = APP_CONFIG.low_level.default_kd
DEFAULT_MAX_VELOCITY = APP_CONFIG.low_level.max_velocity
DEFAULT_MAX_TORQUE = APP_CONFIG.low_level.max_torque


def get_joint_id(joint_name: str) -> int:
    """获取关节索引。"""

    if joint_name in LegID:
        return LegID[joint_name]
    raise ValueError("未知关节: %s" % joint_name)


def get_joint_name(joint_id: int) -> str:
    """获取关节名称。"""

    if joint_id in LegID_reverse:
        return LegID_reverse[joint_id]
    raise ValueError("未知关节ID: %s" % joint_id)


def check_safety_limit(joint_name: str, position: float) -> bool:
    """检查位置是否在安全范围内。"""

    if joint_name not in SAFETY_LIMITS:
        return True
    limits = SAFETY_LIMITS[joint_name]
    return limits[0] <= position <= limits[1]


def clamp_position(joint_name: str, position: float) -> float:
    """限制位置在安全范围内。"""

    if joint_name not in SAFETY_LIMITS:
        return position
    limits = SAFETY_LIMITS[joint_name]
    return max(limits[0], min(limits[1], position))
