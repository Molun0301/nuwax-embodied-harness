"""平台适配器导出。"""

from adapters.base import (
    AdapterBase,
    AdapterCategory,
    AdapterConfig,
    AdapterHealthStatus,
    AdapterLifecycleState,
)
from adapters.localization.odom_pose_adapter import OdomPoseAdapter
from adapters.mapping.slam_occupancy_adapter import SlamOccupancyAdapter
from adapters.perception.external_perception_service_adapter import ExternalPerceptionServiceAdapter
from adapters.streams.ros_image_adapter import RosImageAdapter
from adapters.streams.webrtc_image_adapter import WebRTCImageAdapter

__all__ = [
    "AdapterBase",
    "AdapterCategory",
    "AdapterConfig",
    "AdapterHealthStatus",
    "AdapterLifecycleState",
    "ExternalPerceptionServiceAdapter",
    "OdomPoseAdapter",
    "RosImageAdapter",
    "SlamOccupancyAdapter",
    "WebRTCImageAdapter",
]
