from contracts.geometry_runtime import (
    ExternalGeometryStackConfig,
    GeometryFrameConfig,
    GeometryOdometryQualityConfig,
    GeometryPointCloudPreprocessConfig,
    GeometryTimestampQualityConfig,
    GeometryTopicsConfig,
    GeometryTraversabilityFilterConfig,
    LcmChannelConfig,
    MappingBackendConfig,
    MappingOutputConfig,
    NavigationBackendConfig,
    PlatformGeometryConfig,
    build_default_geometry_runtime_binary,
    build_default_geometry_runtime_port,
    build_default_geometry_runtime_ready_file,
    build_default_geometry_runtime_start_script,
    build_default_lcm_url,
    build_default_nuwax_workspace_dir,
    build_default_rtabmap_database_root,
    build_default_rtabmap_pcd_root,
)

__all__ = [
    "ExternalGeometryStackConfig",
    "GeometryFrameConfig",
    "GeometryOdometryQualityConfig",
    "GeometryPointCloudPreprocessConfig",
    "GeometryTimestampQualityConfig",
    "GeometryTopicsConfig",
    "GeometryTraversabilityFilterConfig",
    "LcmChannelConfig",
    "MappingBackendConfig",
    "MappingOutputConfig",
    "NavigationBackendConfig",
    "PlatformGeometryConfig",
    "build_default_geometry_runtime_binary",
    "build_default_geometry_runtime_port",
    "build_default_geometry_runtime_ready_file",
    "build_default_geometry_runtime_start_script",
    "build_default_lcm_url",
    "build_default_nuwax_workspace_dir",
    "build_default_rtabmap_database_root",
    "build_default_rtabmap_pcd_root",
]

# v4 rename (R-N14 Phase 1):B 类保留——旧 `RtabmapConfig` / `RtabmapOutputConfig`
# 类名作为 alias re-export 保留 1 个版本周期,通过 telemetry 计数驱动清理。
# 下游 import 仍可写 `from services.geometry.config import RtabmapConfig`,
# 但应尽快迁移到 `MappingBackendConfig`。
RtabmapConfig = MappingBackendConfig  # type: ignore[misc,assignment]
RtabmapOutputConfig = MappingOutputConfig  # type: ignore[misc,assignment]