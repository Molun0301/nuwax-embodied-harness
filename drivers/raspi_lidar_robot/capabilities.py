from __future__ import annotations

from contracts.capabilities import CapabilityMatrix

# 树莓派 + 2D 激光雷达小机器狗的能力矩阵（本期未完整实现；只声明元数据）
RASPI_LIDAR_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="raspi_lidar_robot",
    capabilities=[],
    metadata={
        "entrypoint": "drivers/raspi_lidar_robot",
        "implemented": False,
        "slam_dimension": "2d",
        "tier_recommendation": "lite",
    },
)
