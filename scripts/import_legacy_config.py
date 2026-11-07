"""升级时一次性把旧环境方案导入 TOML；后续版本删除本工具。"""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import sys
from typing import Dict

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from core.config import ConfigStore, atomic_write, specifications

_KEY = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
LEGACY_KEYS = {'ARK_API_KEY': 'perception.openai_api_key',
 'DASHSCOPE_API_KEY': 'memory.embedding_api_key',
 'GO2_DATA_PLANE_DDS_IFACE': 'go2.data_plane_dds_iface',
 'GO2_DATA_PLANE_ENABLED': 'go2.data_plane_enabled',
 'GO2_DDS_IFACE': 'go2.dds_iface',
 'GO2_DIRECT_DDS_ENABLED': 'go2.direct_dds_enabled',
 'GO2_DIRECT_DDS_LOW_STATE_TOPIC': 'go2.direct_dds_low_state_topic',
 'GO2_DIRECT_DDS_POINT_CLOUD_ENABLED': 'go2.direct_dds_point_cloud_enabled',
 'GO2_DIRECT_DDS_POINT_CLOUD_TOPIC': 'go2.direct_dds_point_cloud_topic',
 'GO2_DIRECT_DDS_POINT_CLOUD_XYZ_REGISTERED_TO_ODOM': 'go2.direct_dds_point_cloud_xyz_registered_to_odom',
 'GO2_DIRECT_DDS_POSE_TOPIC': 'go2.direct_dds_pose_topic',
 'GO2_DIRECT_DDS_READ_TIMEOUT_SEC': 'go2.direct_dds_read_timeout_sec',
 'GO2_IMU_GYRO_DEADBAND_RPS': 'go2.imu_gyro_deadband_rps',
 'GO2_LOW_LEVEL_INTERVAL_SEC': 'go2.low_level_interval_sec',
 'GO2_LOW_LEVEL_KD': 'go2.low_level_kd',
 'GO2_LOW_LEVEL_KP': 'go2.low_level_kp',
 'GO2_LOW_LEVEL_MAX_TORQUE': 'go2.low_level_max_torque',
 'GO2_LOW_LEVEL_MAX_VELOCITY': 'go2.low_level_max_velocity',
 'GO2_OFFICIAL_BACKEND_ENABLED': 'go2.official_backend_enabled',
 'GO2_OFFICIAL_CLIENT_TIMEOUT_SEC': 'go2.official_client_timeout_sec',
 'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS': 'go2.official_max_linear_velocity_mps',
 'GO2_OFFICIAL_MAX_YAW_RATE_RPS': 'go2.official_max_yaw_rate_rps',
 'GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS': 'go2.official_yaw_rate_deadband_rps',
 'GO2_ROBOT_IP': 'go2.robot_ip',
 'GO2_VIDEO_STREAM_ENABLED': 'go2.video_stream_enabled',
 'GO2_VIDEO_STREAM_LOG_LEVEL': 'logging.go2_video_level',
 'GO2_VIDEO_STREAM_RECONNECT_SEC': 'go2.video_stream_reconnect_sec',
 'GO2_VOLUME_AUTO_ENABLE_VUI_SWITCH': 'go2.volume_auto_enable_vui_switch',
 'GO2_VUI_MAX_LEVEL': 'go2.vui_max_level',
 'GO2_VUI_MIN_LEVEL': 'go2.vui_min_level',
 'GO2_WEBRTC_AES_128_KEY': 'go2.webrtc_aes_128_key',
 'NUWAX_ACTIVE_ROBOT': 'robot.driver',
 'NUWAX_ARTIFACT_CLEANUP_BATCH_SIZE': 'gateway.artifact_cleanup_batch_size',
 'NUWAX_ARTIFACT_MAX_COUNT': 'gateway.artifact_max_count',
 'NUWAX_ARTIFACT_MAX_TOTAL_BYTES': 'gateway.artifact_max_total_bytes',
 'NUWAX_ARTIFACT_RETENTION_DAYS': 'gateway.artifact_retention_days',
 'NUWAX_BASE_FRAME': 'robot.base_frame',
 'NUWAX_EXPLORE_POSE_TTL_MS': 'navigation.explore_pose_ttl_ms',
 'NUWAX_FACILITY_PROVIDERS_JSON': 'navigation.facility_providers_json',
 'NUWAX_GATEWAY_ARTIFACT_DIR': 'gateway.artifact_dir',
 'NUWAX_GATEWAY_HOST': 'gateway.host',
 'NUWAX_GATEWAY_POINTCLOUD3D_COLOR': 'gateway.pointcloud3d_color',
 'NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR': 'gateway.pointcloud3d_ground_color',
 'NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS': 'gateway.pointcloud3d_height_colors',
 'NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR': 'gateway.pointcloud3d_object_color',
 'NUWAX_GATEWAY_POINTCLOUD3D_SIZE': 'gateway.pointcloud3d_size',
 'NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M': 'gateway.pointcloud3d_spacing_m',
 'NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR': 'gateway.pointcloud3d_structure_color',
 'NUWAX_GATEWAY_PORT': 'gateway.port',
 'NUWAX_GATEWAY_PUBLIC_BASE_URL': 'gateway.public_base_url',
 'NUWAX_GATEWAY_SSE_KEEPALIVE_SEC': 'gateway.sse_keepalive_sec',
 'NUWAX_GATEWAY_TELEMETRY_ENABLED': 'gateway.telemetry_enabled',
 'NUWAX_GATEWAY_TELEMETRY_INCLUDE_PROCESS_TREE': 'gateway.telemetry_include_process_tree',
 'NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC': 'gateway.telemetry_interval_sec',
 'NUWAX_GATEWAY_TELEMETRY_PROCESS_KEYWORDS': 'gateway.telemetry_process_keywords',
 'NUWAX_GATEWAY_WS_PING_INTERVAL_SEC': 'gateway.ws_ping_interval_sec',
 'NUWAX_GEOMETRY_CORRECTION_BACKEND': 'geometry.correction_backend',
 'NUWAX_GEOMETRY_ENABLED': 'geometry.enabled',
 'NUWAX_GEOMETRY_EXTERNAL_ENABLED': 'geometry.external_enabled',
 'NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB': 'geometry.fine_cloud_cache_auto_max_mib',
 'NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB': 'geometry.fine_cloud_cache_mib',
 'NUWAX_GEOMETRY_GRPC_MAX_MESSAGE_BYTES': 'gateway.geometry_grpc_max_message_bytes',
 'NUWAX_GEOMETRY_LIDAR_FRAME': 'robot.lidar_frame',
 'NUWAX_GEOMETRY_LIO_BACKEND': 'geometry.lio_backend',
 'NUWAX_GEOMETRY_LIO_BODY_FROM_IMU': 'geometry.lio_body_from_imu',
 'NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR': 'geometry.lio_body_from_sensor',
 'NUWAX_GEOMETRY_LIO_INITIAL_MAP_FRAMES': 'geometry.lio_initial_map_frames',
 'NUWAX_GEOMETRY_LIO_MAP_VOXEL_SIZE_M': 'geometry.lio_map_voxel_size_m',
 'NUWAX_GEOMETRY_LIO_SCAN_VOXEL_SIZE_M': 'geometry.lio_scan_voxel_size_m',
 'NUWAX_GEOMETRY_MAPPING_BACKEND': 'geometry.mapping_backend',
 'NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS': 'geometry.native_lio_pair_max_skew_ms',
 'NUWAX_GEOMETRY_ODOM_FRAME': 'robot.odom_frame',
 'NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC': 'geometry.odom_max_age_sec',
 'NUWAX_GEOMETRY_PGO_OPTIONS': 'geometry.pgo_options',
 'NUWAX_GEOMETRY_PIPELINE': 'geometry.pipeline',
 'NUWAX_GEOMETRY_POINTLIO_OPTIONS': 'geometry.pointlio_options',
 'NUWAX_GEOMETRY_RUNTIME_PORT': 'geometry.runtime_port',
 'NUWAX_GEOMETRY_SLAM2D_CONFIG': 'geometry.slam2d_config',
 'NUWAX_GEOMETRY_SLAM2D_DEVICE': 'geometry.slam2d_config',
 'NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG': 'navigation.traversability_max_slope_deg',
 'NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M': 'navigation.traversability_max_step_height_m',
 'NUWAX_LCM_URL': 'transport.lcm_url',
 'NUWAX_LOG_BACKUP_COUNT': 'logging.backup_count',
 'NUWAX_LOG_DIR': 'logging.dir',
 'NUWAX_LOG_FILE': 'logging.file',
 'NUWAX_LOG_FORMAT': 'logging.format',
 'NUWAX_LOG_LEVEL': 'logging.level',
 'NUWAX_LOG_LEVEL_CONFIG': 'logging.level_config',
 'NUWAX_LOG_LEVEL_DRIVER': 'logging.level_driver',
 'NUWAX_LOG_LEVEL_GATEWAY': 'logging.level_gateway',
 'NUWAX_LOG_LEVEL_GEOMETRY': 'logging.level_geometry',
 'NUWAX_LOG_LEVEL_NAVIGATION': 'logging.level_navigation',
 'NUWAX_LOG_LEVEL_PERCEPTION': 'logging.level_perception',
 'NUWAX_LOG_LEVEL_RUNTIME': 'logging.level_runtime',
 'NUWAX_LOG_LEVEL_SETTINGS': 'logging.level_settings',
 'NUWAX_LOG_LEVEL_SKILLS': 'logging.level_skills',
 'NUWAX_LOG_MAX_BYTES': 'logging.max_bytes',
 'NUWAX_MANIPULATION_BASE_URL': 'manipulation.base_url',
 'NUWAX_MANIPULATION_ENABLED': 'manipulation.enabled',
 'NUWAX_MANIPULATION_HTTP_TIMEOUT_SEC': 'manipulation.http_timeout_sec',
 'NUWAX_MANIPULATION_POLL_INTERVAL_SEC': 'manipulation.poll_interval_sec',
 'NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS': 'geometry.mapping_initial_safe_radius_meters',
 'NUWAX_MAPPING_RAYTRACING_GRACE_DEPTH': 'geometry.mapping_raytracing_grace_depth',
 'NUWAX_MAPPING_RAYTRACING_GRAZE_COS': 'geometry.mapping_raytracing_graze_cos',
 'NUWAX_MAPPING_RAYTRACING_MAX_HEALTH': 'geometry.mapping_raytracing_max_health',
 'NUWAX_MAPPING_RAYTRACING_MAX_RANGE': 'geometry.mapping_raytracing_max_range',
 'NUWAX_MAPPING_RAYTRACING_MIN_HEALTH': 'geometry.mapping_raytracing_min_health',
 'NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE': 'geometry.mapping_raytracing_ray_subsample',
 'NUWAX_MAPPING_RAYTRACING_RECENCY_WINDOW': 'geometry.mapping_raytracing_recency_window',
 'NUWAX_MAPPING_RAYTRACING_SHADOW_DEPTH': 'geometry.mapping_raytracing_shadow_depth',
 'NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE': 'geometry.mapping_raytracing_voxel_size',
 'NUWAX_MAPPING_RECORDING_DIR': 'geometry.mapping_recording_dir',
 'NUWAX_MAPPING_RECORDING_ENABLED': 'geometry.mapping_recording_enabled',
 'NUWAX_MAPPING_RECORDING_MAX_BYTES': 'geometry.mapping_recording_max_bytes',
 'NUWAX_MAPPING_RESOLUTION': 'geometry.mapping_resolution',
 'NUWAX_MAP_CATALOG_ROOT': 'geometry.map_catalog_root',
 'NUWAX_MAP_FRAME': 'robot.map_frame',
 'NUWAX_MEMORY_ASSOCIATION_CONFIRM_MIN_OBSERVATIONS': 'memory.association_confirm_min_observations',
 'NUWAX_MEMORY_ASSOCIATION_CONFIRM_WINDOW_SEC': 'memory.association_confirm_window_sec',
 'NUWAX_MEMORY_ASSOCIATION_CROSS_LABEL_OVERLAP': 'memory.association_cross_label_overlap',
 'NUWAX_MEMORY_ASSOCIATION_CROSS_LABEL_SIZE_RATIO': 'memory.association_cross_label_size_ratio',
 'NUWAX_MEMORY_ASSOCIATION_GATE_SIGMA': 'memory.association_gate_sigma',
 'NUWAX_MEMORY_ASSOCIATION_MAX_VOXELS': 'memory.association_max_voxels',
 'NUWAX_MEMORY_ASSOCIATION_VOXEL_SIZE_M': 'memory.association_voxel_size_m',
 'NUWAX_MEMORY_EMBEDDING_BASE_URL': 'memory.embedding_base_url',
 'NUWAX_MEMORY_EMBEDDING_TIMEOUT_SEC': 'memory.embedding_timeout_sec',
 'NUWAX_MEMORY_IMAGE_EMBEDDING_DIM': 'memory.image_embedding_dim',
 'NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL': 'memory.image_embedding_model',
 'NUWAX_MEMORY_TEXT_EMBEDDING_DIM': 'memory.text_embedding_dim',
 'NUWAX_MEMORY_TEXT_EMBEDDING_MODEL': 'memory.text_embedding_model',
 'NUWAX_MOTION_CONTROL_FACTORY': 'motion_guard.control_factory',
 'NUWAX_MOTION_GUARD_BACKEND': 'motion_guard.backend',
 'NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS': 'robot.actuator_max_linear_mps',
 'NUWAX_NAV_ACTUATOR_MAX_YAW_RPS': 'robot.actuator_max_yaw_rps',
 'NUWAX_NAV_ACTUATOR_MOTION_MODEL': 'robot.actuator_motion_model',
 'NUWAX_NAV_AUTOSTART': 'navigation.autostart',
 'NUWAX_NAV_CAPABILITIES_JSON': 'navigation.capabilities_json',
 'NUWAX_NAV_LOCAL_HORIZON_M': 'navigation.local_horizon_m',
 'NUWAX_NAV_MAX_STEP_DOWN_M': 'navigation.max_step_down_m',
 'NUWAX_NAV_MAX_STEP_UP_M': 'navigation.max_step_up_m',
 'NUWAX_NAV_MIN_STAIR_TREAD_M': 'navigation.min_stair_tread_m',
 'NUWAX_NAV_PCT_TERRAIN_SOURCE': 'navigation.pct_terrain_source',
 'NUWAX_NAV_SURFACE_VOXEL_SIZE_M': 'navigation.surface_voxel_size_m',
 'NUWAX_NOMINATIM_URL': 'map_cloud.nominatim_url',
 'NUWAX_PERCEPTION_BURST_FRAME_COUNT': 'perception.burst_frame_count',
 'NUWAX_PERCEPTION_BURST_INTERVAL_SEC': 'perception.burst_interval_sec',
 'NUWAX_PERCEPTION_BYTETRACK_HIGH_THRESHOLD': 'perception.bytetrack_high_threshold',
 'NUWAX_PERCEPTION_BYTETRACK_IOU': 'perception.bytetrack_iou',
 'NUWAX_PERCEPTION_BYTETRACK_LOST_TTL_FRAMES': 'perception.bytetrack_lost_ttl_frames',
 'NUWAX_PERCEPTION_BYTETRACK_LOW_THRESHOLD': 'perception.bytetrack_low_threshold',
 'NUWAX_PERCEPTION_BYTETRACK_NAME': 'perception.bytetrack_name',
 'NUWAX_PERCEPTION_BYTETRACK_TRAJECTORY_LIMIT': 'perception.bytetrack_trajectory_limit',
 'NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND': 'perception.default_detector_backend',
 'NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND': 'perception.default_scene_backend',
 'NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND': 'perception.default_segmentation_backend',
 'NUWAX_PERCEPTION_DEFERRED_STRATEGY': 'perception.deferred_strategy',
 'NUWAX_PERCEPTION_GROUND_TOLERANCE_M': 'perception.ground_tolerance_m',
 'NUWAX_PERCEPTION_KEYFRAME_MIN_INTERVAL_SEC': 'perception.keyframe_min_interval_sec',
 'NUWAX_PERCEPTION_KEYFRAME_REMEMBER_ENABLED': 'perception.keyframe_remember_enabled',
 'NUWAX_PERCEPTION_KEYFRAME_TRANSLATION_M': 'perception.keyframe_translation_m',
 'NUWAX_PERCEPTION_KEYFRAME_YAW_DEG': 'perception.keyframe_yaw_deg',
 'NUWAX_PERCEPTION_OPENAI_API_STYLE': 'perception.openai_api_style',
 'NUWAX_PERCEPTION_OPENAI_BACKEND_NAME': 'perception.openai_backend_name',
 'NUWAX_PERCEPTION_OPENAI_BASE_URL': 'perception.openai_base_url',
 'NUWAX_PERCEPTION_OPENAI_ENABLED': 'perception.openai_enabled',
 'NUWAX_PERCEPTION_OPENAI_MAX_TOKENS': 'perception.openai_max_tokens',
 'NUWAX_PERCEPTION_OPENAI_MODEL': 'perception.openai_model',
 'NUWAX_PERCEPTION_OPENAI_TEMPERATURE': 'perception.openai_temperature',
 'NUWAX_PERCEPTION_OPENAI_TIMEOUT_SEC': 'perception.openai_timeout_sec',
 'NUWAX_PERCEPTION_POSTPROCESS_NMS_ENABLED': 'perception.postprocess_nms_enabled',
 'NUWAX_PERCEPTION_POSTPROCESS_NMS_IOU': 'perception.postprocess_nms_iou',
 'NUWAX_PERCEPTION_SEGMENTATION_ALLOW_BBOX_ONLY_FALLBACK': 'perception.segmentation_allow_bbox_only_fallback',
 'NUWAX_PERCEPTION_SEGMENTATION_BACKEND_KIND': 'perception.segmentation_backend_kind',
 'NUWAX_PERCEPTION_SEGMENTATION_BACKEND_NAME': 'perception.segmentation_backend_name',
 'NUWAX_PERCEPTION_SEGMENTATION_DEPTH_COVERAGE_THRESHOLD': 'perception.segmentation_depth_coverage_threshold',
 'NUWAX_PERCEPTION_SEGMENTATION_EMIT_POLYGON': 'perception.segmentation_emit_polygon',
 'NUWAX_PERCEPTION_SEGMENTATION_EMIT_QUALITY_DETAILS': 'perception.segmentation_emit_quality_details',
 'NUWAX_PERCEPTION_SEGMENTATION_EMIT_RAW_MASK': 'perception.segmentation_emit_raw_mask',
 'NUWAX_PERCEPTION_SEGMENTATION_EMIT_RUNTIME_SUMMARY': 'perception.segmentation_emit_runtime_summary',
 'NUWAX_PERCEPTION_SEGMENTATION_ENABLED': 'perception.segmentation_enabled',
 'NUWAX_PERCEPTION_SEGMENTATION_LOW_LATENCY_BACKEND_NAME': 'perception.segmentation_low_latency_backend_name',
 'NUWAX_PERCEPTION_SEGMENTATION_MAP_PROJECTION_COVERAGE_THRESHOLD': 'perception.segmentation_map_projection_coverage_threshold',
 'NUWAX_PERCEPTION_SEGMENTATION_POLYGON_SIMPLIFICATION_EPSILON_PX': 'perception.segmentation_polygon_simplification_epsilon_px',
 'NUWAX_PERCEPTION_SEGMENTATION_PREDICTED_IOU_THRESHOLD': 'perception.segmentation_predicted_iou_threshold',
 'NUWAX_PERCEPTION_SEGMENTATION_PRIMARY_BACKEND_NAME': 'perception.segmentation_primary_backend_name',
 'NUWAX_PERCEPTION_SEGMENTATION_REPROJECTION_ERROR_THRESHOLD_PX': 'perception.segmentation_reprojection_error_threshold_px',
 'NUWAX_PERCEPTION_SEGMENTATION_ROUTE': 'perception.segmentation_route',
 'NUWAX_PERCEPTION_SEGMENTATION_SCORE_THRESHOLD': 'perception.segmentation_score_threshold',
 'NUWAX_PERCEPTION_SEGMENTATION_STABILITY_SCORE_THRESHOLD': 'perception.segmentation_stability_score_threshold',
 'NUWAX_PERCEPTION_SELF_FILTER_BBOX_IOU': 'perception.self_filter_bbox_iou',
 'NUWAX_PERCEPTION_SELF_FILTER_COVERAGE': 'perception.self_filter_coverage',
 'NUWAX_PERCEPTION_SELF_FILTER_DILATION_PX': 'perception.self_filter_dilation_px',
 'NUWAX_PERCEPTION_SELF_FILTER_ENABLED': 'perception.self_filter_enabled',
 'NUWAX_PERCEPTION_SELF_FILTER_MAX_JOINT_AGE_SEC': 'perception.self_filter_max_joint_age_sec',
 'NUWAX_PERCEPTION_SELF_FILTER_STATIC_REGIONS_JSON': 'perception.self_filter_static_regions_json',
 'NUWAX_PERCEPTION_STREAM_CAMERA_ID': 'perception.stream_camera_id',
 'NUWAX_PERCEPTION_STREAM_DETECTOR_BACKEND': 'perception.stream_detector_backend',
 'NUWAX_PERCEPTION_STREAM_FAILURE_BACKOFF_SEC': 'perception.stream_failure_backoff_sec',
 'NUWAX_PERCEPTION_STREAM_INTERVAL_SEC': 'perception.stream_interval_sec',
 'NUWAX_PERCEPTION_SUPPORT_MIN_POINTS': 'memory.association_min_support_points',
 'NUWAX_PERCEPTION_TRACKER_BACKEND': 'perception.tracker_backend',
 'NUWAX_PERCEPTION_TRIANGULATION_MAX_RESIDUAL_M': 'perception.triangulation_max_residual_m',
 'NUWAX_PERCEPTION_TRIANGULATION_MIN_PARALLAX_DEG': 'perception.triangulation_min_parallax_deg',
 'NUWAX_PERCEPTION_TRIANGULATION_MIN_RAYS': 'perception.triangulation_min_rays',
 'NUWAX_PERCEPTION_YOLO_WORLD_ALIASES': 'perception.yolo_world_aliases',
 'NUWAX_PERCEPTION_YOLO_WORLD_BACKEND_NAME': 'perception.yolo_world_backend_name',
 'NUWAX_PERCEPTION_YOLO_WORLD_CLASSES': 'perception.yolo_world_classes',
 'NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE': 'perception.yolo_world_confidence',
 'NUWAX_PERCEPTION_YOLO_WORLD_DEVICE': 'perception.yolo_world_device',
 'NUWAX_PERCEPTION_YOLO_WORLD_DYNAMIC_CLASS_KEYS': 'perception.yolo_world_dynamic_class_keys',
 'NUWAX_PERCEPTION_YOLO_WORLD_ENABLED': 'perception.yolo_world_enabled',
 'NUWAX_PERCEPTION_YOLO_WORLD_IMAGE_SIZE': 'perception.yolo_world_image_size',
 'NUWAX_PERCEPTION_YOLO_WORLD_IOU': 'perception.yolo_world_iou',
 'NUWAX_PERCEPTION_YOLO_WORLD_MAX_DETECTIONS': 'perception.yolo_world_max_detections',
 'NUWAX_PERCEPTION_YOLO_WORLD_MODEL': 'perception.yolo_world_model',
 'NUWAX_PERCEPTION_YOLO_WORLD_USE_WORKER': 'perception.yolo_world_use_worker',
 'NUWAX_PERSON_FOLLOW_AUTOSTART': 'person_follow.autostart',
 'NUWAX_RAW_SCAN_DIRECTORY': 'geometry.raw_scan_directory',
 'NUWAX_RAW_SCAN_INTERVAL_S': 'geometry.raw_scan_interval_s',
 'NUWAX_RAW_SCAN_MAX_MIB': 'geometry.raw_scan_max_mib',
 'NUWAX_RAW_SCAN_QUEUE_FRAMES': 'geometry.raw_scan_queue_frames',
 'NUWAX_RAW_SCAN_ROTATION_DEG': 'geometry.raw_scan_rotation_deg',
 'NUWAX_RAW_SCAN_TRANSLATION_M': 'geometry.raw_scan_translation_m',
 'NUWAX_REAL_LCM_REQUIRED': 'transport.real_lcm_required',
 'NUWAX_RELAY_ENABLED': 'relay.enabled',
 'NUWAX_RELAY_INCOMING_TOKENS': 'relay.incoming_tokens',
 'NUWAX_RELAY_UPSTREAM_BASE_URL': 'relay.upstream_base_url',
 'NUWAX_RELAY_UPSTREAM_TOKEN': 'relay.upstream_token',
 'NUWAX_ROBOT_CENTER_X_M': 'robot.center_x_m',
 'NUWAX_ROBOT_CENTER_Y_M': 'robot.center_y_m',
 'NUWAX_ROBOT_CENTER_Z_M': 'robot.center_z_m',
 'NUWAX_ROBOT_HEIGHT_M': 'robot.height_m',
 'NUWAX_ROBOT_LENGTH_M': 'robot.length_m',
 'NUWAX_ROBOT_MODEL_BASE_LINK': 'robot.model_base_link',
 'NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON': 'robot.model_joint_positions_json',
 'NUWAX_ROBOT_MODEL_PATH': 'robot.model_path',
 'NUWAX_ROBOT_WIDTH_M': 'robot.width_m',
 'NUWAX_RUNTIME_DATA_DIR': 'geometry.runtime_data_dir',
 'NUWAX_SCENE_MAPPING_API_KEY': 'semantic_map.NUWAX_SCENE_MAPPING_API_KEY',
 'NUWAX_SCENE_MAPPING_AUTO_APPLY': 'semantic_map.NUWAX_SCENE_MAPPING_AUTO_APPLY',
 'NUWAX_SCENE_MAPPING_AUTO_START': 'semantic_map.NUWAX_SCENE_MAPPING_AUTO_START',
 'NUWAX_SCENE_MAPPING_BACKEND': 'semantic_map.NUWAX_SCENE_MAPPING_BACKEND',
 'NUWAX_SCENE_MAPPING_BASE_URL': 'semantic_map.NUWAX_SCENE_MAPPING_BASE_URL',
 'NUWAX_SCENE_MAPPING_DOORWAY_WIDTH_M': 'semantic_map.NUWAX_SCENE_MAPPING_DOORWAY_WIDTH_M',
 'NUWAX_SCENE_MAPPING_ENABLED': 'semantic_map.NUWAX_SCENE_MAPPING_ENABLED',
 'NUWAX_SCENE_MAPPING_INTERVAL_SEC': 'semantic_map.NUWAX_SCENE_MAPPING_INTERVAL_SEC',
 'NUWAX_SCENE_MAPPING_MAX_CELLS': 'semantic_map.NUWAX_SCENE_MAPPING_MAX_CELLS',
 'NUWAX_SCENE_MAPPING_MAX_EVIDENCE_AGE_SEC': 'semantic_map.NUWAX_SCENE_MAPPING_MAX_EVIDENCE_AGE_SEC',
 'NUWAX_SCENE_MAPPING_MAX_IMAGES': 'semantic_map.NUWAX_SCENE_MAPPING_MAX_IMAGES',
 'NUWAX_SCENE_MAPPING_MAX_REGIONS_PER_CYCLE': 'semantic_map.NUWAX_SCENE_MAPPING_MAX_REGIONS_PER_CYCLE',
 'NUWAX_SCENE_MAPPING_MIN_AREA_M2': 'semantic_map.NUWAX_SCENE_MAPPING_MIN_AREA_M2',
 'NUWAX_SCENE_MAPPING_MIN_EVIDENCE_VIEWS': 'semantic_map.NUWAX_SCENE_MAPPING_MIN_EVIDENCE_VIEWS',
 'NUWAX_SCENE_MAPPING_MIN_LABEL_MARGIN': 'semantic_map.NUWAX_SCENE_MAPPING_MIN_LABEL_MARGIN',
 'NUWAX_SCENE_MAPPING_MIN_LABEL_SCORE': 'semantic_map.NUWAX_SCENE_MAPPING_MIN_LABEL_SCORE',
 'NUWAX_SCENE_MAPPING_MODEL': 'semantic_map.NUWAX_SCENE_MAPPING_MODEL',
 'NUWAX_SCENE_MAPPING_TIMEOUT_SEC': 'semantic_map.NUWAX_SCENE_MAPPING_TIMEOUT_SEC',
 'NUWAX_WAVEFRONT_GOAL_TIMEOUT_MS': 'navigation.wavefront_goal_timeout_ms',
 'OPENAI_API_KEY': 'perception.openai_api_key',
 'ROBOT_IP': 'go2.robot_ip',
 '_NUWAX_INSTALLER_SAVED_NUWAX_MEMORY_IMAGE_EMBEDDING_DIM': 'memory.image_embedding_dim',
 '_NUWAX_INSTALLER_SAVED_NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL': 'memory.image_embedding_model',
 '_NUWAX_INSTALLER_SAVED_NUWAX_MEMORY_TEXT_EMBEDDING_DIM': 'memory.text_embedding_dim',
 '_NUWAX_INSTALLER_SAVED_NUWAX_MEMORY_TEXT_EMBEDDING_MODEL': 'memory.text_embedding_model'}

def parse_env_line(line: str):
    """解析一行，不执行命令、变量替换或转义换行；保留引号内部的井号。"""
    line = line.strip()
    if line.startswith("export "):
        line = line[7:].lstrip()
    if not line or line.startswith("#") or "=" not in line:
        return None
    key, value = line.split("=", 1)
    key, value = key.strip(), value.strip()
    if not _KEY.fullmatch(key):
        return None
    if value[:1] in {"'", '"'}:
        quote = value[0]
        chars = []
        index = 1
        while index < len(value):
            ch = value[index]
            if ch == quote:
                return key, "".join(chars)
            if quote == '"' and ch == "\\" and index + 1 < len(value) and value[index + 1] in {"\\", quote, "$", "`"}:
                index += 1
                ch = value[index]
            chars.append(ch)
            index += 1
        # 旧文件可能含未闭合引号，按原文保留，不让一个坏条目隐藏其余配置。
        return key, value
    return key, re.split(r"\s+#", value, maxsplit=1)[0].rstrip()

def read_env_values(path: Path) -> Dict[str, str]:
    if not path.is_file():
        return {}
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        parsed = parse_env_line(line)
        if parsed is not None:
            values[parsed[0]] = parsed[1]
    return values

def convert_values(raw, robot, code_root=ROOT, *, missing_authorizations=None):
    specs = specifications(code_root)
    values = {}
    legacy_credentials = raw
    # 旧读取方对专用地址和主密钥有明确优先级。
    raw = dict(raw)
    raw["GO2_ROBOT_IP"] = raw.get("GO2_ROBOT_IP", "").strip() or raw.get("ROBOT_IP", "")
    raw["OPENAI_API_KEY"] = raw.get("OPENAI_API_KEY", "").strip() or raw.get("ARK_API_KEY", "")
    for kind in ("TEXT", "IMAGE"):
        model = "NUWAX_MEMORY_" + kind + "_EMBEDDING_MODEL"
        dim = "NUWAX_MEMORY_" + kind + "_EMBEDDING_DIM"
        current = raw.get(model, "").strip()
        if not current or current in ("lexical-only", "lexical_only", "disabled") or current.startswith("hashing"):
            if raw.get("_NUWAX_INSTALLER_SAVED_" + model, "").strip():
                raw[model] = raw["_NUWAX_INSTALLER_SAVED_" + model]
                raw[dim] = raw.get("_NUWAX_INSTALLER_SAVED_" + dim, raw.get(dim, ""))
    for old, text in raw.items():
        key = LEGACY_KEYS.get(old)
        if not key or old in ("ROBOT_IP", "ARK_API_KEY", "NUWAX_GEOMETRY_SLAM2D_DEVICE") or old.startswith("_NUWAX_INSTALLER_SAVED_"):
            continue
        spec = specs[key]
        text = text.strip()
        if text in spec.get("special_values", ()):
            value = text
        elif not text:
            value = spec["default"]
        elif spec["type"] == "boolean":
            if text.lower() not in ("1", "true", "yes", "on", "0", "false", "no", "off"):
                raise ValueError(old + " 不是布尔值")
            value = text.lower() in ("1", "true", "yes", "on")
        elif spec["type"] == "integer":
            number = float(text)
            if not number.is_integer():
                raise ValueError(old + " 必须为整数")
            value = int(number)
        elif spec["type"] == "float":
            value = float(text)
        elif spec["type"] == "array":
            value = [item.strip() for item in text.split(",") if item.strip()]
        else:
            value = text
        values[key] = value
    values["robot.driver"] = robot
    facility_key = "navigation.facility_providers_json"
    if facility_key in values:
        facilities = json.loads(values[facility_key])
        for name, config in facilities.items():
            variable = config.pop("authorization_env", None)
            if variable is not None:
                if variable in legacy_credentials:
                    config["authorization"] = legacy_credentials[variable]
                else:
                    config.pop("authorization", None)
                    if missing_authorizations is not None:
                        missing_authorizations.append(name)
        values[facility_key] = json.dumps(facilities, ensure_ascii=False, separators=(",", ":"))
    device = raw.get("NUWAX_GEOMETRY_SLAM2D_DEVICE", "").strip()
    if device:
        planar = json.loads(values.get("geometry.slam2d_config", "") or "{}")
        planar["device"] = json.loads(device)
        values["geometry.slam2d_config"] = json.dumps(planar, ensure_ascii=False, separators=(",", ":"))
    return values


def import_legacy(home, code_root=ROOT, *, manager_home=None, asset_root=None):
    home = Path(home)
    profiles = home / "config/profiles"
    files = sorted(profiles.glob("*/*.env"))
    if not files:
        return {"imported": [], "renamed": [], "missing_facility_authorizations": []}
    counts = Counter(path.stem for path in files)
    names = {path: path.parent.name + "_" + path.stem if counts[path.stem] > 1 else path.stem for path in files}
    active = home / "data/settings/active.env"
    selected = active.resolve() if active.is_symlink() else None
    if selected not in names:
        # 旧受管安装的 installed.json 记录当前机器人，.active 记录其方案。
        record = home / "current/installed.json"
        robot = json.loads(record.read_text())["robot"] if record.is_file() else None
        selector = profiles / robot / ".active" if robot else None
        if selector is not None and selector.exists():
            selected = profiles / robot / (selector.read_text().strip() + ".env")
    if selected not in names:
        raise ValueError("旧安装没有选中有效的活动方案")
    if len(set(names.values())) != len(names):
        raise ValueError("机器人前缀后的方案名仍有冲突，请先在旧方案中调整名称")
    converted = {}
    missing_facility_authorizations = []
    for path in files:
        missing = []
        converted[path] = convert_values(read_env_values(path), path.parent.name, code_root,
                                         missing_authorizations=missing)
        missing_facility_authorizations.extend({"profile": names[path], "facility": name} for name in missing)
    store = ConfigStore(home, asset_root or code_root)
    renamed = []
    for path, values in converted.items():
        store.write_profile(names[path], values)
        if names[path] != path.stem:
            renamed.append({"old": str(path.relative_to(home)), "new": "config/profiles/" + names[path] + ".toml"})
    atomic_write(store.selector, (names[selected] + "\n").encode("utf-8"))
    store._composed_values(store.effective(startup=True))
    for path in files:
        path.unlink()
    for directory in {path.parent for path in files}:
        (directory / ".active").unlink(missing_ok=True)
        if not any(directory.iterdir()):
            directory.rmdir()
    if active.is_symlink():
        active.unlink()
    return {"imported": [names[path] for path in files], "renamed": renamed,
            "missing_facility_authorizations": missing_facility_authorizations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(import_legacy(args.home), ensure_ascii=False))


if __name__ == "__main__":
    main()
