"""Go2 端侧兼容与调试实现。

这里保留的是历史 Go2 局部规划、补图和碰撞检测实现，
仅服务于设备侧兼容路径、命名地图和调试观测，不属于平台正式主链。
"""

from drivers.go2.compat.collision_detector import CollisionStatus, Go2CollisionDetector, Go2CollisionDetectorConfig
from drivers.go2.compat.cost_mapper import Go2CostMapper, Go2CostMapperConfig, Go2CostMapperResult
from drivers.go2.compat.local_planner import (
    Go2DWALocalPlanner,
    Go2LocalPlannerConfig,
    Go2VelocityCommand,
    LocalPlannerState,
)
from drivers.go2.compat.voxel_mapper import Go2VoxelMapper, Go2VoxelMapperConfig, Go2VoxelMapperResult

__all__ = [
    "CollisionStatus",
    "Go2CollisionDetector",
    "Go2CollisionDetectorConfig",
    "Go2CostMapper",
    "Go2CostMapperConfig",
    "Go2CostMapperResult",
    "Go2DWALocalPlanner",
    "Go2LocalPlannerConfig",
    "Go2VelocityCommand",
    "LocalPlannerState",
    "Go2VoxelMapper",
    "Go2VoxelMapperConfig",
    "Go2VoxelMapperResult",
]
