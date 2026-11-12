//! 与模式同名的运行段；所有字段均必须在合成配置中出现。
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Robot {
    #[serde(deserialize_with = "crate::optional_number")]
    pub actuator_max_linear_mps: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub actuator_max_yaw_rps: Option<f64>,
    pub actuator_motion_model: String,
    pub base_frame: String,
    #[serde(deserialize_with = "crate::optional_number")]
    pub center_x_m: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub center_y_m: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub center_z_m: Option<f64>,
    pub driver: String,
    #[serde(deserialize_with = "crate::optional_number")]
    pub height_m: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub length_m: Option<f64>,
    pub lidar_frame: String,
    pub map_frame: String,
    pub model_base_link: String,
    pub model_joint_positions_json: String,
    pub model_path: String,
    pub odom_frame: String,
    #[serde(deserialize_with = "crate::optional_number")]
    pub width_m: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Transport {
    pub lcm_url: String,
    pub real_lcm_required: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Geometry {
    pub correction_backend: String,
    pub enabled: bool,
    pub external_enabled: bool,
    pub fine_cloud_cache_auto_max_mib: i64,
    pub fine_cloud_cache_mib: CacheBudget,
    pub lio_backend: String,
    pub lio_body_from_imu: String,
    pub lio_body_from_sensor: String,
    #[serde(deserialize_with = "crate::optional_number")]
    pub lio_initial_map_frames: Option<i64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub lio_map_voxel_size_m: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub lio_scan_voxel_size_m: Option<f64>,
    pub map_catalog_root: String,
    pub mapping_backend: String,
    pub mapping_initial_safe_radius_meters: f64,
    pub mapping_raytracing_grace_depth: f64,
    pub mapping_raytracing_graze_cos: f64,
    pub mapping_raytracing_max_health: i64,
    pub mapping_raytracing_max_range: f64,
    pub mapping_raytracing_min_health: i64,
    pub mapping_raytracing_ray_subsample: i64,
    pub mapping_raytracing_recency_window: i64,
    pub mapping_raytracing_shadow_depth: f64,
    pub mapping_raytracing_voxel_size: f64,
    pub mapping_recording_dir: String,
    pub mapping_recording_enabled: bool,
    pub mapping_recording_max_bytes: i64,
    pub mapping_resolution: f64,
    pub native_lio_pair_max_skew_ms: i64,
    pub odom_max_age_sec: f64,
    pub pgo_options: String,
    pub pipeline: String,
    pub pointlio_options: String,
    pub raw_scan_directory: String,
    pub raw_scan_interval_s: f64,
    pub raw_scan_max_mib: i64,
    pub raw_scan_rotation_deg: f64,
    pub raw_scan_translation_m: f64,
    pub runtime_data_dir: String,
    pub runtime_port: i64,
    pub slam2d_config: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Navigation {
    pub autostart: bool,
    pub capabilities_json: String,
    pub explore_pose_ttl_ms: i64,
    pub facility_providers_json: String,
    pub local_horizon_m: f64,
    pub obstacle_clearance_m: f64,
    #[serde(deserialize_with = "crate::optional_number")]
    pub max_step_down_m: Option<f64>,
    #[serde(deserialize_with = "crate::optional_number")]
    pub max_step_up_m: Option<f64>,
    pub min_stair_tread_m: f64,
    pub pct_terrain_source: String,
    pub surface_voxel_size_m: f64,
    pub traversability_max_slope_deg: f64,
    pub traversability_max_step_height_m: f64,
    pub wavefront_goal_timeout_ms: i64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PersonFollow {
    pub autostart: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Logging {
    pub backup_count: i64,
    pub dir: String,
    pub file: String,
    pub format: String,
    pub go2_video_level: String,
    pub level: String,
    pub level_config: String,
    pub level_driver: String,
    pub level_gateway: String,
    pub level_geometry: String,
    pub level_navigation: String,
    pub level_perception: String,
    pub level_runtime: String,
    pub level_settings: String,
    pub level_skills: String,
    pub max_bytes: i64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MapCloud {
    pub nominatim_url: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Derived {
    pub nav_profile: String,
    pub actuator_max_linear_mps: f64,
    pub actuator_max_yaw_rps: f64,
    pub actuator_motion_model: String,
    pub robot_geometry_json: String,
}

/// 模式允许整数 MiB 或 auto，具体范围沿既有缓存解析边界。
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(untagged)]
pub enum CacheBudget {
    Mib(i64),
    Mode(String),
}
impl std::fmt::Display for CacheBudget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Mib(n) => write!(f, "{n}"),
            Self::Mode(s) => write!(f, "{s}"),
        }
    }
}
