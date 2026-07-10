"""定位类适配器导出。"""

from adapters.localization.geometry_state_adapter import (
    BackendLocalizationState,
    GeometryLocalizationStateMapping,
    map_backend_state_to_platform_status,
    create_mapping_state_mapping,
    create_rtabmap_state_mapping,  # v4 R-N14 Phase 4 B 类 alias
    create_go2_sdk_state_mapping,
    create_default_state_mapping,
    compute_tracking_quality,
)

__all__ = [
    "BackendLocalizationState",
    "GeometryLocalizationStateMapping",
    "map_backend_state_to_platform_status",
    "create_mapping_state_mapping",
    "create_rtabmap_state_mapping",  # v4 R-N14 Phase 4 B 类 alias
    "create_go2_sdk_state_mapping",
    "create_default_state_mapping",
    "compute_tracking_quality",
]
