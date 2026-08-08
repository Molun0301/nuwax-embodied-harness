//! 离线 Geometry 重放的流式数据合同与准入校验。
//!
//! 本模块只完成与后端无关的结构和数值准入。阶段 2 在把配置转换为
//! crate-private kernel 类型时，仍须调用现有 kernel 校验，后者是后端合法性的真源。

use serde::{Deserialize, Serialize};
use thiserror::Error;

const MAX_IDENTITY_BYTES: usize = 255;
// SSOT：`pointcloud.rs::POINTCLOUD_MAX_FRAME_POINTS`。
// 阶段 1 的独立 lib 不加载在线 pointcloud 模块，因此在此保持精确镜像。
const POINTCLOUD_MAX_FRAME_POINTS: usize = 100_000;
const MAP_GEOREFERENCE_SCHEMA_VERSION: u32 = 1;
const MAP_GEOREFERENCE_SOURCE_CRS: &str = "EPSG:4326";
const MAP_GEOREFERENCE_PROJECTED_CRS: &str = "EPSG:3857";
const MAP_GEOREFERENCE_AXIS_CONVENTION: &str = "projected_x_east_y_north_z_up";
const MAP_GEOREFERENCE_ALTITUDE_POLICY: &str = "map_plane_constant";
const MAP_GEOREFERENCE_MAX_LATITUDE_DEG: f64 = 85.051_128_779_806_6;
const MAP_GEOREFERENCE_MAX_COORDINATE_M: f64 = 1_000_000_000.0;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OfflineBackendKind {
    Voxel,
    Raytracing,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineVoxelConfigV1 {
    pub resolution_m: f64,
    pub voxel_size_m: f64,
    pub initial_safe_radius_m: f64,
    pub window_capacity: usize,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineRaytracingConfigV1 {
    pub resolution_m: f64,
    pub voxel_size_m: f64,
    pub max_range_m: f64,
    pub ray_subsample: u32,
    pub shadow_depth_m: f64,
    pub grace_depth_m: f64,
    pub min_health: i32,
    pub max_health: i32,
    pub graze_cos: f64,
    pub recency_window: u32,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "kind", content = "config", rename_all = "snake_case")]
pub enum OfflineBackendConfigV1 {
    Voxel(OfflineVoxelConfigV1),
    Raytracing(OfflineRaytracingConfigV1),
}

impl OfflineBackendConfigV1 {
    const fn kind(&self) -> OfflineBackendKind {
        match self {
            Self::Voxel(_) => OfflineBackendKind::Voxel,
            Self::Raytracing(_) => OfflineBackendKind::Raytracing,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflinePointXyzV1 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflinePoseV1 {
    pub translation_xyz_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineReplayFrameV1 {
    pub source_sequence: u64,
    pub source_timestamp_ns: i64,
    pub sensor_frame_id: String,
    pub map_frame_id: String,
    pub pose_t_map_sensor: OfflinePoseV1,
    pub points_xyz: Vec<OfflinePointXyzV1>,
}

/// `contracts/map_workspace.py::MapGeoreferenceV1` 的 Rust 输入镜像。
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineGeoreferenceV1 {
    pub schema_version: u32,
    pub map_name: String,
    pub map_frame_id: String,
    pub source_crs: String,
    pub projected_crs: String,
    pub axis_convention: String,
    pub anchor_lat_deg: f64,
    pub anchor_lng_deg: f64,
    pub anchor_easting_m: f64,
    pub anchor_northing_m: f64,
    pub map_origin_x_m: f64,
    pub map_origin_y_m: f64,
    pub map_from_projected_yaw_rad: f64,
    pub altitude_policy: String,
    pub map_plane_z_m: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineReplayConfigV1 {
    pub map_frame_id: String,
    pub backend_kind: OfflineBackendKind,
    pub backend: OfflineBackendConfigV1,
    pub georeference: OfflineGeoreferenceV1,
}

/// 一次重放的不可变元数据；帧通过 `validate_frame` 逐个准入。
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OfflineReplayRequestV1 {
    pub map_name: String,
    pub source_id: String,
    pub config: OfflineReplayConfigV1,
}

/// V2 快照绑定的、已经通过 V1 validator 校验的重放身份。
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflineReplayIdentityV2 {
    pub map_name: String,
    pub source_id: String,
    pub map_frame_id: String,
}

/// 后续 `MapSnapshot` 构造所需的 canonical seed；最终来源身份由 Stage 5 决定。
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflineMapSnapshotSeedV2 {
    pub map_name: String,
    pub map_frame_id: String,
    pub update_sequence: u64,
}

pub struct OfflineReplayValidator {
    request: ValidatedOfflineReplayRequestV1,
    last: Option<(u64, u64)>,
}

impl OfflineReplayValidator {
    pub fn request(&self) -> &ValidatedOfflineReplayRequestV1 {
        &self.request
    }

    pub const fn last_source_sequence(&self) -> Option<u64> {
        match self.last {
            Some((sequence, _)) => Some(sequence),
            None => None,
        }
    }

    pub const fn last_source_timestamp_ns(&self) -> Option<u64> {
        match self.last {
            Some((_, timestamp_ns)) => Some(timestamp_ns),
            None => None,
        }
    }

    pub fn validate_frame(
        mut self,
        frame: OfflineReplayFrameV1,
    ) -> Result<(ValidatedOfflineReplayFrameV1, Self), OfflineReplayValidationError> {
        let validated = validate_frame(&self.request, self.last, frame)?;
        self.last = Some((validated.source_sequence, validated.source_timestamp_ns));
        Ok((validated, self))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidatedOfflineReplayRequestV1 {
    map_name: String,
    source_id: String,
    config: ValidatedOfflineReplayConfigV1,
}

impl ValidatedOfflineReplayRequestV1 {
    pub fn map_name(&self) -> &str {
        &self.map_name
    }

    pub fn source_id(&self) -> &str {
        &self.source_id
    }

    pub fn map_frame_id(&self) -> &str {
        &self.config.map_frame_id
    }

    pub const fn backend_kind(&self) -> OfflineBackendKind {
        self.config.backend_kind
    }

    pub fn georeference(&self) -> &OfflineGeoreferenceV1 {
        &self.config.georeference
    }

    pub fn backend_config(&self) -> &OfflineBackendConfigV1 {
        &self.config.backend
    }

    pub fn identity_v2(&self) -> OfflineReplayIdentityV2 {
        OfflineReplayIdentityV2 {
            map_name: self.map_name.clone(),
            source_id: self.source_id.clone(),
            map_frame_id: self.config.map_frame_id.clone(),
        }
    }

    pub fn map_snapshot_seed_v2(&self, update_sequence: u64) -> OfflineMapSnapshotSeedV2 {
        OfflineMapSnapshotSeedV2 {
            map_name: self.map_name.clone(),
            map_frame_id: self.config.map_frame_id.clone(),
            update_sequence,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidatedOfflineReplayFrameV1 {
    source_sequence: u64,
    source_timestamp_ns: u64,
    sensor_frame_id: String,
    map_frame_id: String,
    pose_t_map_sensor: ValidatedOfflinePoseV1,
    points_xyz: Vec<OfflinePointXyzV1>,
}

impl ValidatedOfflineReplayFrameV1 {
    pub const fn source_sequence(&self) -> u64 {
        self.source_sequence
    }

    pub const fn source_timestamp_ns(&self) -> u64 {
        self.source_timestamp_ns
    }

    pub fn sensor_frame_id(&self) -> &str {
        &self.sensor_frame_id
    }

    pub fn map_frame_id(&self) -> &str {
        &self.map_frame_id
    }

    pub fn translation_xyz_m(&self) -> [f64; 3] {
        self.pose_t_map_sensor.translation_xyz_m
    }

    pub fn rotation_xyzw(&self) -> [f64; 4] {
        self.pose_t_map_sensor.rotation_xyzw
    }

    pub fn points_xyz(&self) -> &[OfflinePointXyzV1] {
        &self.points_xyz
    }
}

#[derive(Clone, Debug, PartialEq)]
struct ValidatedOfflinePoseV1 {
    translation_xyz_m: [f64; 3],
    rotation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, PartialEq)]
struct ValidatedOfflineReplayConfigV1 {
    map_frame_id: String,
    backend_kind: OfflineBackendKind,
    backend: OfflineBackendConfigV1,
    georeference: OfflineGeoreferenceV1,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum OfflineReplayValidationError {
    #[error("{field} is required and must be canonical text")]
    InvalidIdentity { field: &'static str },
    #[error("point cloud must contain 1..={POINTCLOUD_MAX_FRAME_POINTS} points")]
    InvalidPointCount,
    #[error("{field} must be finite")]
    InvalidFiniteNumber { field: &'static str },
    #[error("{field} must be finite and positive")]
    InvalidPositiveNumber { field: &'static str },
    #[error("{field} must be finite and non-negative")]
    InvalidNonNegativeNumber { field: &'static str },
    #[error("point {point_index} contains a non-finite XYZ component")]
    InvalidPoint { point_index: usize },
    #[error("quaternion must contain finite values and have non-zero norm")]
    InvalidQuaternion,
    #[error("source timestamp must be non-negative")]
    NegativeTimestamp,
    #[error("source sequence and timestamp must be strictly increasing")]
    NonMonotonicFrame,
    #[error("frame map identity does not match the replay request")]
    FrameMismatch,
    #[error("backend config kind does not match backend_kind")]
    BackendMismatch,
    #[error("georeference map identity does not match the replay request")]
    GeoreferenceMismatch,
    #[error("georeference schema_version must be 1")]
    InvalidGeoreferenceSchema,
    #[error("georeference contains a negative zero value")]
    NegativeZeroGeoreference,
    #[error("georeference uses an unsupported fixed label")]
    InvalidGeoreferenceLabel,
    #[error("georeference numeric field is outside its admitted range")]
    InvalidGeoreferenceRange,
    #[error("georeference projected anchor does not match its WGS84 anchor")]
    InvalidProjectedAnchor,
    #[error("raytracing min_health must be less than max_health")]
    InvalidRayHealthRange,
}

pub struct OfflineGeometryFacade;

impl OfflineGeometryFacade {
    pub fn start(
        request: OfflineReplayRequestV1,
    ) -> Result<OfflineReplayValidator, OfflineReplayValidationError> {
        validate_identity("map_name", &request.map_name)?;
        validate_identity("source_id", &request.source_id)?;
        validate_frame_id("config.map_frame_id", &request.config.map_frame_id)?;
        if request.config.backend.kind() != request.config.backend_kind {
            return Err(OfflineReplayValidationError::BackendMismatch);
        }
        validate_backend(&request.config.backend)?;
        validate_georeference(
            &request.config.georeference,
            &request.map_name,
            &request.config.map_frame_id,
        )?;

        Ok(OfflineReplayValidator {
            request: ValidatedOfflineReplayRequestV1 {
                map_name: request.map_name,
                source_id: request.source_id,
                config: ValidatedOfflineReplayConfigV1 {
                    map_frame_id: request.config.map_frame_id,
                    backend_kind: request.config.backend_kind,
                    backend: request.config.backend,
                    georeference: request.config.georeference,
                },
            },
            last: None,
        })
    }
}

fn validate_frame(
    request: &ValidatedOfflineReplayRequestV1,
    previous: Option<(u64, u64)>,
    frame: OfflineReplayFrameV1,
) -> Result<ValidatedOfflineReplayFrameV1, OfflineReplayValidationError> {
    validate_frame_id("frame.sensor_frame_id", &frame.sensor_frame_id)?;
    validate_frame_id("frame.map_frame_id", &frame.map_frame_id)?;
    if frame.map_frame_id != request.config.map_frame_id {
        return Err(OfflineReplayValidationError::FrameMismatch);
    }
    let timestamp_ns = u64::try_from(frame.source_timestamp_ns)
        .map_err(|_| OfflineReplayValidationError::NegativeTimestamp)?;
    if let Some((sequence, timestamp)) = previous {
        if frame.source_sequence <= sequence || timestamp_ns <= timestamp {
            return Err(OfflineReplayValidationError::NonMonotonicFrame);
        }
    }
    if frame.points_xyz.is_empty() || frame.points_xyz.len() > POINTCLOUD_MAX_FRAME_POINTS {
        return Err(OfflineReplayValidationError::InvalidPointCount);
    }
    if frame
        .pose_t_map_sensor
        .translation_xyz_m
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(OfflineReplayValidationError::InvalidFiniteNumber {
            field: "frame.pose_t_map_sensor.translation_xyz_m",
        });
    }
    let rotation = normalize_quaternion(frame.pose_t_map_sensor.rotation_xyzw)?;
    for (point_index, point) in frame.points_xyz.iter().enumerate() {
        if [point.x, point.y, point.z]
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(OfflineReplayValidationError::InvalidPoint { point_index });
        }
    }

    Ok(ValidatedOfflineReplayFrameV1 {
        source_sequence: frame.source_sequence,
        source_timestamp_ns: timestamp_ns,
        sensor_frame_id: frame.sensor_frame_id,
        map_frame_id: frame.map_frame_id,
        pose_t_map_sensor: ValidatedOfflinePoseV1 {
            translation_xyz_m: frame.pose_t_map_sensor.translation_xyz_m,
            rotation_xyzw: rotation,
        },
        points_xyz: frame.points_xyz,
    })
}

fn validate_backend(backend: &OfflineBackendConfigV1) -> Result<(), OfflineReplayValidationError> {
    match backend {
        OfflineBackendConfigV1::Voxel(config) => {
            positive("backend.resolution_m", config.resolution_m)?;
            positive("backend.voxel_size_m", config.voxel_size_m)?;
            non_negative(
                "backend.initial_safe_radius_m",
                config.initial_safe_radius_m,
            )?;
            if config.window_capacity == 0 {
                return Err(OfflineReplayValidationError::InvalidPositiveNumber {
                    field: "backend.window_capacity",
                });
            }
        }
        OfflineBackendConfigV1::Raytracing(config) => {
            positive("backend.resolution_m", config.resolution_m)?;
            positive("backend.voxel_size_m", config.voxel_size_m)?;
            non_negative("backend.max_range_m", config.max_range_m)?;
            non_negative("backend.shadow_depth_m", config.shadow_depth_m)?;
            non_negative("backend.grace_depth_m", config.grace_depth_m)?;
            if config.ray_subsample == 0 {
                return Err(OfflineReplayValidationError::InvalidPositiveNumber {
                    field: "backend.ray_subsample",
                });
            }
            if !config.graze_cos.is_finite() {
                return Err(OfflineReplayValidationError::InvalidFiniteNumber {
                    field: "backend.graze_cos",
                });
            }
            if config.min_health >= config.max_health {
                return Err(OfflineReplayValidationError::InvalidRayHealthRange);
            }
        }
    }
    Ok(())
}

fn validate_georeference(
    georeference: &OfflineGeoreferenceV1,
    map_name: &str,
    map_frame_id: &str,
) -> Result<(), OfflineReplayValidationError> {
    if georeference.schema_version != MAP_GEOREFERENCE_SCHEMA_VERSION {
        return Err(OfflineReplayValidationError::InvalidGeoreferenceSchema);
    }
    validate_identity("georeference.map_name", &georeference.map_name)?;
    validate_frame_id("georeference.map_frame_id", &georeference.map_frame_id)?;
    if georeference.map_name != map_name || georeference.map_frame_id != map_frame_id {
        return Err(OfflineReplayValidationError::GeoreferenceMismatch);
    }
    if georeference.source_crs != MAP_GEOREFERENCE_SOURCE_CRS
        || georeference.projected_crs != MAP_GEOREFERENCE_PROJECTED_CRS
        || georeference.axis_convention != MAP_GEOREFERENCE_AXIS_CONVENTION
        || georeference.altitude_policy != MAP_GEOREFERENCE_ALTITUDE_POLICY
    {
        return Err(OfflineReplayValidationError::InvalidGeoreferenceLabel);
    }
    let numeric = [
        georeference.anchor_lat_deg,
        georeference.anchor_lng_deg,
        georeference.anchor_easting_m,
        georeference.anchor_northing_m,
        georeference.map_origin_x_m,
        georeference.map_origin_y_m,
        georeference.map_from_projected_yaw_rad,
        georeference.map_plane_z_m,
    ];
    if numeric.iter().any(|value| !value.is_finite()) {
        return Err(OfflineReplayValidationError::InvalidFiniteNumber {
            field: "georeference",
        });
    }
    if numeric
        .iter()
        .any(|value| *value == 0.0 && value.is_sign_negative())
    {
        return Err(OfflineReplayValidationError::NegativeZeroGeoreference);
    }
    if !(-MAP_GEOREFERENCE_MAX_LATITUDE_DEG..=MAP_GEOREFERENCE_MAX_LATITUDE_DEG)
        .contains(&georeference.anchor_lat_deg)
        || !(-180.0..=180.0).contains(&georeference.anchor_lng_deg)
        || !(-std::f64::consts::PI..std::f64::consts::PI)
            .contains(&georeference.map_from_projected_yaw_rad)
        || [
            georeference.map_origin_x_m,
            georeference.map_origin_y_m,
            georeference.map_plane_z_m,
        ]
        .iter()
        .any(|value| value.abs() > MAP_GEOREFERENCE_MAX_COORDINATE_M)
    {
        return Err(OfflineReplayValidationError::InvalidGeoreferenceRange);
    }
    let latitude_rad = georeference.anchor_lat_deg.to_radians();
    let expected_easting = 6_378_137.0 * georeference.anchor_lng_deg.to_radians();
    let expected_northing = 6_378_137.0
        * (std::f64::consts::FRAC_PI_4 + latitude_rad / 2.0)
            .tan()
            .ln();
    if (expected_easting - georeference.anchor_easting_m).abs() > 1.0e-6
        || (expected_northing - georeference.anchor_northing_m).abs() > 1.0e-6
    {
        return Err(OfflineReplayValidationError::InvalidProjectedAnchor);
    }
    Ok(())
}

fn normalize_quaternion(quaternion: [f64; 4]) -> Result<[f64; 4], OfflineReplayValidationError> {
    if quaternion.iter().any(|value| !value.is_finite()) {
        return Err(OfflineReplayValidationError::InvalidQuaternion);
    }
    let norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if !norm.is_finite() || norm == 0.0 {
        return Err(OfflineReplayValidationError::InvalidQuaternion);
    }
    Ok(quaternion.map(|value| value / norm))
}

fn validate_identity(field: &'static str, value: &str) -> Result<(), OfflineReplayValidationError> {
    if value.is_empty()
        || value.trim() != value
        || value.len() > MAX_IDENTITY_BYTES
        || value.chars().any(char::is_control)
    {
        return Err(OfflineReplayValidationError::InvalidIdentity { field });
    }
    Ok(())
}

fn validate_frame_id(field: &'static str, value: &str) -> Result<(), OfflineReplayValidationError> {
    validate_identity(field, value)?;
    if !value.split('/').all(|segment| {
        let mut characters = segment.chars();
        matches!(characters.next(), Some(first) if first.is_ascii_alphabetic())
            && characters.all(|character| character.is_ascii_alphanumeric() || character == '_')
    }) {
        return Err(OfflineReplayValidationError::InvalidIdentity { field });
    }
    Ok(())
}

fn positive(field: &'static str, value: f64) -> Result<(), OfflineReplayValidationError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(OfflineReplayValidationError::InvalidPositiveNumber { field });
    }
    Ok(())
}

fn non_negative(field: &'static str, value: f64) -> Result<(), OfflineReplayValidationError> {
    if !value.is_finite() || value < 0.0 {
        return Err(OfflineReplayValidationError::InvalidNonNegativeNumber { field });
    }
    Ok(())
}
