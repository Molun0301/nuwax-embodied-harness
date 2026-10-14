//! Shared LIO backend slot with two independently executable estimator implementations.

#![deny(unsafe_code)]

pub mod mid360_pcap;
pub mod native;
pub mod pointlio_options;

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use std::path::Path;

const MAX_CLOUD_POINTS: usize = 250_000;
const MAX_GO2_L2_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;
const MAX_IMU_SAMPLES: usize = 10_000;
const MAX_FRAME_ID_BYTES: usize = 255;
const MAX_SCAN_DURATION_NS: u64 = 1_000_000_000;
const MAX_FRAME_GAP_NS: i64 = 60_000_000_000;
const MAX_ABS_POINT_COORDINATE_M: f64 = 1_000_000.0;
const MAX_INTENSITY: f64 = 1_000_000.0;
const MAX_ABS_ACCEL_MPS2: f64 = 1_000.0;
const MAX_ABS_GYRO_RPS: f64 = 1_000.0;
const MAX_REGISTRATION_POINTS: usize = 1_024;
const MAX_REGISTRATION_ITERATIONS: usize = 12;
const MAX_REGISTRATION_DISTANCE_EVALUATIONS: usize =
    MAX_REGISTRATION_POINTS * MAX_REGISTRATION_POINTS * MAX_REGISTRATION_ITERATIONS;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackendKind {
    FastLio2,
    PointLio,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimedPoint {
    pub offset_ns: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimestampedCloud {
    pub timestamp_ns: i64,
    pub frame_id: String,
    pub points: Vec<TimedPoint>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    pub timestamp_ns: i64,
    pub accel_mps2: [f64; 3],
    pub gyro_rps: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LioInputFrame {
    pub cloud: TimestampedCloud,
    pub imu: Vec<ImuSample>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Go2PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Go2PointCloud2 {
    pub timestamp_ns: i64,
    pub frame_id: String,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<Go2PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Go2ImuSample {
    pub timestamp_ns: i64,
    pub frame_id: String,
    pub accel_mps2: [f64; 3],
    pub gyro_rps: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Go2L2AdapterConfig {
    pub lidar_frame_id: String,
    pub imu_frame_id: String,
    pub body_frame_id: String,
    pub lidar_to_body_translation_m: [f64; 3],
    pub lidar_to_body_rotation_row_major: [f64; 9],
}

impl Go2L2AdapterConfig {
    pub fn adapt(
        &self,
        cloud: &Go2PointCloud2,
        imu: &[Go2ImuSample],
    ) -> Result<LioInputFrame, String> {
        validate_go2_adapter_config(self)?;
        if imu.is_empty() {
            return Err("Go2 L2 frame has no IMU samples".into());
        }
        if imu.len() > MAX_IMU_SAMPLES {
            return Err(format!(
                "Go2 L2 frame has {} IMU samples; limit is {MAX_IMU_SAMPLES}",
                imu.len()
            ));
        }
        if cloud.timestamp_ns < 0 {
            return Err("Go2 L2 cloud timestamp must be non-negative".into());
        }
        if cloud.frame_id != self.lidar_frame_id {
            return Err(format!(
                "Go2 L2 cloud frame_id {:?} does not match configured lidar frame {:?}",
                cloud.frame_id, self.lidar_frame_id
            ));
        }
        if cloud.is_bigendian {
            return Err("Go2 L2 PointCloud2 must use little-endian scalars".into());
        }
        if cloud.height == 0 || cloud.width == 0 {
            return Err("Go2 L2 PointCloud2 dimensions must be non-zero".into());
        }
        let point_count = usize::try_from(cloud.height)
            .ok()
            .and_then(|height| {
                usize::try_from(cloud.width)
                    .ok()
                    .and_then(|width| height.checked_mul(width))
            })
            .ok_or_else(|| "Go2 L2 PointCloud2 point count overflowed".to_string())?;
        if point_count > MAX_CLOUD_POINTS {
            return Err(format!(
                "Go2 L2 PointCloud2 has {point_count} points; limit is {MAX_CLOUD_POINTS}"
            ));
        }
        let point_step = usize::try_from(cloud.point_step)
            .map_err(|_| "Go2 L2 PointCloud2 point_step is out of range".to_string())?;
        if point_step == 0 {
            return Err("Go2 L2 PointCloud2 point_step must be non-zero".into());
        }
        let width = usize::try_from(cloud.width)
            .map_err(|_| "Go2 L2 PointCloud2 width is out of range".to_string())?;
        let height = usize::try_from(cloud.height)
            .map_err(|_| "Go2 L2 PointCloud2 height is out of range".to_string())?;
        let row_step = usize::try_from(cloud.row_step)
            .map_err(|_| "Go2 L2 PointCloud2 row_step is out of range".to_string())?;
        let packed_row = width
            .checked_mul(point_step)
            .ok_or_else(|| "Go2 L2 PointCloud2 packed row size overflowed".to_string())?;
        if row_step < packed_row {
            return Err("Go2 L2 PointCloud2 row_step is smaller than width * point_step".into());
        }
        let expected_bytes = height
            .checked_mul(row_step)
            .ok_or_else(|| "Go2 L2 PointCloud2 payload size overflowed".to_string())?;
        if expected_bytes > MAX_GO2_L2_PAYLOAD_BYTES {
            return Err(format!(
                "Go2 L2 PointCloud2 payload has {expected_bytes} bytes; limit is {MAX_GO2_L2_PAYLOAD_BYTES}"
            ));
        }
        if cloud.data.len() != expected_bytes {
            return Err(format!(
                "Go2 L2 PointCloud2 payload has {} bytes; expected {expected_bytes}",
                cloud.data.len()
            ));
        }

        let x = required_field(&cloud.fields, "x", point_step, true)?;
        let y = required_field(&cloud.fields, "y", point_step, true)?;
        let z = required_field(&cloud.fields, "z", point_step, true)?;
        let intensity = optional_field(&cloud.fields, "intensity", point_step, false)?;
        let offset_ns = optional_field(&cloud.fields, "offset_ns", point_step, false)?;
        if let Some(field) = offset_ns {
            if !matches!(field.datatype, 1..=6) {
                return Err("Go2 L2 offset_ns must use an integer PointField datatype".into());
            }
        }

        let mut points = Vec::with_capacity(point_count);
        for row in 0..height {
            for column in 0..width {
                let start = row * row_step + column * point_step;
                let point = &cloud.data[start..start + point_step];
                let coordinates = [
                    read_numeric(point, x)?,
                    read_numeric(point, y)?,
                    read_numeric(point, z)?,
                ];
                let point_intensity = intensity
                    .map(|field| read_numeric(point, field))
                    .transpose()?
                    .unwrap_or(0.0);
                if coordinates
                    .iter()
                    .chain(std::iter::once(&point_intensity))
                    .any(|value| !value.is_finite())
                {
                    continue;
                }
                let point_offset_ns = offset_ns
                    .map(|field| read_integer(point, field))
                    .transpose()?
                    .unwrap_or(0);
                let transformed = apply_lidar_to_body(self, coordinates);
                if transformed.iter().any(|value| !value.is_finite()) {
                    continue;
                }
                points.push(TimedPoint {
                    offset_ns: point_offset_ns,
                    x: transformed[0],
                    y: transformed[1],
                    z: transformed[2],
                    intensity: point_intensity,
                });
            }
        }

        let imu = imu
            .iter()
            .map(|sample| {
                if sample.frame_id != self.imu_frame_id {
                    return Err(format!(
                        "Go2 L2 IMU frame_id {:?} does not match configured IMU frame {:?}",
                        sample.frame_id, self.imu_frame_id
                    ));
                }
                Ok(ImuSample {
                    timestamp_ns: sample.timestamp_ns,
                    accel_mps2: sample.accel_mps2,
                    gyro_rps: sample.gyro_rps,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let frame = LioInputFrame {
            cloud: TimestampedCloud {
                timestamp_ns: cloud.timestamp_ns,
                frame_id: self.body_frame_id.clone(),
                points,
            },
            imu,
        };
        validate(&frame, None)?;
        Ok(frame)
    }
}

fn validate_go2_adapter_config(config: &Go2L2AdapterConfig) -> Result<(), String> {
    for (name, frame) in [
        ("lidar_frame_id", &config.lidar_frame_id),
        ("imu_frame_id", &config.imu_frame_id),
        ("body_frame_id", &config.body_frame_id),
    ] {
        if frame.trim().is_empty() || frame.len() > MAX_FRAME_ID_BYTES {
            return Err(format!(
                "Go2 L2 {name} must be a non-empty bounded frame ID"
            ));
        }
    }
    if config.lidar_frame_id == config.body_frame_id {
        return Err("Go2 L2 lidar and body frame IDs must be distinct".into());
    }
    if config.imu_frame_id != config.body_frame_id {
        return Err("Go2 L2 IMU frame must already be the configured body frame".into());
    }
    if config
        .lidar_to_body_translation_m
        .iter()
        .chain(&config.lidar_to_body_rotation_row_major)
        .any(|value| !value.is_finite())
    {
        return Err("Go2 L2 lidar-to-body extrinsic must contain finite values".into());
    }
    let rotation = &config.lidar_to_body_rotation_row_major;
    let rows = [
        [rotation[0], rotation[1], rotation[2]],
        [rotation[3], rotation[4], rotation[5]],
        [rotation[6], rotation[7], rotation[8]],
    ];
    const TOLERANCE: f64 = 1.0e-6;
    for row in &rows {
        let norm = row.iter().map(|value| value * value).sum::<f64>();
        if (norm - 1.0).abs() > TOLERANCE {
            return Err("Go2 L2 lidar-to-body rotation rows must have unit length".into());
        }
    }
    for (left, right) in [(0, 1), (0, 2), (1, 2)] {
        let dot = rows[left]
            .iter()
            .zip(rows[right])
            .map(|(a, b)| a * b)
            .sum::<f64>();
        if dot.abs() > TOLERANCE {
            return Err("Go2 L2 lidar-to-body rotation rows must be orthogonal".into());
        }
    }
    let determinant = rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]);
    if (determinant - 1.0).abs() > TOLERANCE {
        return Err("Go2 L2 lidar-to-body rotation must be right-handed".into());
    }
    Ok(())
}

fn required_field<'a>(
    fields: &'a [Go2PointField],
    name: &str,
    point_step: usize,
    coordinates_only: bool,
) -> Result<&'a Go2PointField, String> {
    optional_field(fields, name, point_step, coordinates_only)?
        .ok_or_else(|| format!("Go2 L2 PointCloud2 is missing required field {name:?}"))
}

fn optional_field<'a>(
    fields: &'a [Go2PointField],
    name: &str,
    point_step: usize,
    coordinates_only: bool,
) -> Result<Option<&'a Go2PointField>, String> {
    let mut matches = fields.iter().filter(|field| field.name == name);
    let Some(field) = matches.next() else {
        return Ok(None);
    };
    if matches.next().is_some() {
        return Err(format!("Go2 L2 PointCloud2 field {name:?} is duplicated"));
    }
    if field.count != 1 {
        return Err(format!(
            "Go2 L2 PointCloud2 field {name:?} must have count 1"
        ));
    }
    if coordinates_only && !matches!(field.datatype, 7 | 8) {
        return Err(format!(
            "Go2 L2 PointCloud2 coordinate field {name:?} must be FLOAT32 or FLOAT64"
        ));
    }
    let scalar_size = pointfield_scalar_size(field.datatype)
        .ok_or_else(|| format!("Go2 L2 PointCloud2 field {name:?} has unsupported datatype"))?;
    let offset = usize::try_from(field.offset)
        .map_err(|_| format!("Go2 L2 PointCloud2 field {name:?} offset is out of range"))?;
    let end = offset
        .checked_add(scalar_size)
        .ok_or_else(|| format!("Go2 L2 PointCloud2 field {name:?} end offset overflowed"))?;
    if end > point_step {
        return Err(format!(
            "Go2 L2 PointCloud2 field {name:?} exceeds point_step"
        ));
    }
    Ok(Some(field))
}

fn pointfield_scalar_size(datatype: u8) -> Option<usize> {
    match datatype {
        1 | 2 => Some(1),
        3 | 4 => Some(2),
        5..=7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

fn read_numeric(point: &[u8], field: &Go2PointField) -> Result<f64, String> {
    let start = usize::try_from(field.offset)
        .map_err(|_| format!("field {:?} offset is out of range", field.name))?;
    let value = match field.datatype {
        1 => i8::from_le_bytes([point[start]]) as f64,
        2 => u8::from_le_bytes([point[start]]) as f64,
        3 => i16::from_le_bytes(point[start..start + 2].try_into().unwrap()) as f64,
        4 => u16::from_le_bytes(point[start..start + 2].try_into().unwrap()) as f64,
        5 => i32::from_le_bytes(point[start..start + 4].try_into().unwrap()) as f64,
        6 => u32::from_le_bytes(point[start..start + 4].try_into().unwrap()) as f64,
        7 => f32::from_le_bytes(point[start..start + 4].try_into().unwrap()) as f64,
        8 => f64::from_le_bytes(point[start..start + 8].try_into().unwrap()),
        _ => return Err(format!("field {:?} has unsupported datatype", field.name)),
    };
    Ok(value)
}

fn read_integer(point: &[u8], field: &Go2PointField) -> Result<u64, String> {
    let value = read_numeric(point, field)?;
    if !value.is_finite() || value < 0.0 || value.fract() != 0.0 {
        return Err(format!(
            "Go2 L2 PointCloud2 field {:?} must contain a non-negative integer",
            field.name
        ));
    }
    Ok(value as u64)
}

fn apply_lidar_to_body(config: &Go2L2AdapterConfig, point: [f64; 3]) -> [f64; 3] {
    let rotation = &config.lidar_to_body_rotation_row_major;
    let translation = &config.lidar_to_body_translation_m;
    [
        rotation[0] * point[0] + rotation[1] * point[1] + rotation[2] * point[2] + translation[0],
        rotation[3] * point[0] + rotation[4] * point[1] + rotation[5] * point[2] + translation[1],
        rotation[6] * point[0] + rotation[7] * point[1] + rotation[8] * point[2] + translation[2],
    ]
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OdometryEstimate {
    pub timestamp_ns: i64,
    pub frame_id: String,
    pub child_frame_id: String,
    pub position_m: [f64; 3],
    pub yaw_rad: f64,
    pub linear_velocity_mps: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LioHealth {
    pub healthy: bool,
    pub backend: BackendKind,
    pub processed_frames: u64,
    pub imu_samples: u64,
    pub registered_points: usize,
    pub status: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LioOutputFrame {
    pub odometry: OdometryEstimate,
    pub registered_cloud: TimestampedCloud,
    pub health: LioHealth,
}

pub trait LioEstimator {
    fn kind(&self) -> BackendKind;
    fn process(&mut self, frame: &LioInputFrame) -> Result<LioOutputFrame, String>;
}

#[derive(Debug, Clone, Default)]
struct EstimatorState {
    timestamp_ns: Option<i64>,
    position: [f64; 3],
    velocity: [f64; 3],
    yaw: f64,
    previous_scan: Option<Vec<TimedPoint>>,
    frames: u64,
    imu_samples: u64,
}

#[derive(Debug, Default)]
pub struct FastLio2Estimator {
    state: EstimatorState,
}

#[derive(Debug, Default)]
pub struct PointLioEstimator {
    state: EstimatorState,
}

fn validate(frame: &LioInputFrame, previous_timestamp_ns: Option<i64>) -> Result<(), String> {
    if frame.cloud.timestamp_ns < 0 {
        return Err("cloud timestamp must be non-negative".into());
    }
    if frame.cloud.frame_id.trim().is_empty() {
        return Err("cloud frame_id must be non-empty".into());
    }
    if frame.cloud.frame_id.len() > MAX_FRAME_ID_BYTES {
        return Err(format!("cloud frame_id exceeds {MAX_FRAME_ID_BYTES} bytes"));
    }
    if frame.cloud.points.is_empty() {
        return Err("cloud has no points".into());
    }
    if frame.cloud.points.len() > MAX_CLOUD_POINTS {
        return Err(format!(
            "cloud has {} points; limit is {MAX_CLOUD_POINTS}",
            frame.cloud.points.len()
        ));
    }
    if frame.imu.is_empty() {
        return Err("frame has no IMU samples".into());
    }
    if frame.imu.len() > MAX_IMU_SAMPLES {
        return Err(format!(
            "frame has {} IMU samples; limit is {MAX_IMU_SAMPLES}",
            frame.imu.len()
        ));
    }
    if let Some(previous) = previous_timestamp_ns {
        let gap = frame
            .cloud
            .timestamp_ns
            .checked_sub(previous)
            .ok_or_else(|| "cloud timestamp difference overflowed".to_string())?;
        if gap <= 0 {
            return Err("cloud timestamps must be strictly increasing".into());
        }
        if gap > MAX_FRAME_GAP_NS {
            return Err(format!("cloud timestamp gap exceeds {MAX_FRAME_GAP_NS} ns"));
        }
        if frame
            .imu
            .iter()
            .any(|sample| sample.timestamp_ns < previous)
        {
            return Err("IMU timestamp must not be older than the previous cloud".into());
        }
    }
    if frame
        .cloud
        .points
        .iter()
        .any(|point| point.offset_ns > MAX_SCAN_DURATION_NS)
    {
        return Err(format!("point offset exceeds {MAX_SCAN_DURATION_NS} ns"));
    }
    if frame.cloud.points.iter().any(|point| {
        [point.x, point.y, point.z]
            .into_iter()
            .any(|value| !value.is_finite() || value.abs() > MAX_ABS_POINT_COORDINATE_M)
            || !point.intensity.is_finite()
            || !(0.0..=MAX_INTENSITY).contains(&point.intensity)
    }) {
        return Err("cloud points must contain finite, bounded coordinates and intensity".into());
    }
    if frame.imu.iter().any(|sample| {
        sample.timestamp_ns < 0
            || sample
                .accel_mps2
                .iter()
                .any(|value| !value.is_finite() || value.abs() > MAX_ABS_ACCEL_MPS2)
            || sample
                .gyro_rps
                .iter()
                .any(|value| !value.is_finite() || value.abs() > MAX_ABS_GYRO_RPS)
    }) {
        return Err(
            "IMU samples must contain non-negative timestamps and finite, bounded values".into(),
        );
    }
    if frame
        .imu
        .windows(2)
        .any(|pair| pair[0].timestamp_ns >= pair[1].timestamp_ns)
    {
        return Err("IMU timestamps must be strictly increasing".into());
    }
    if frame
        .imu
        .iter()
        .any(|sample| sample.timestamp_ns > frame.cloud.timestamp_ns)
    {
        return Err("IMU timestamp is newer than cloud timestamp".into());
    }
    let imu_span = frame
        .cloud
        .timestamp_ns
        .checked_sub(frame.imu[0].timestamp_ns)
        .ok_or_else(|| "cloud and IMU timestamp difference overflowed".to_string())?;
    if imu_span > MAX_FRAME_GAP_NS {
        return Err(format!(
            "cloud and IMU timestamp span exceeds {MAX_FRAME_GAP_NS} ns"
        ));
    }
    Ok(())
}

fn rotate_translate(point: &TimedPoint, yaw: f64, position: [f64; 3]) -> TimedPoint {
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    TimedPoint {
        offset_ns: point.offset_ns,
        x: cos_yaw * point.x - sin_yaw * point.y + position[0],
        y: sin_yaw * point.x + cos_yaw * point.y + position[1],
        z: point.z + position[2],
        intensity: point.intensity,
    }
}

fn output(
    kind: BackendKind,
    state: &EstimatorState,
    cloud: TimestampedCloud,
    healthy: bool,
    status: &str,
) -> LioOutputFrame {
    LioOutputFrame {
        odometry: OdometryEstimate {
            timestamp_ns: cloud.timestamp_ns,
            frame_id: "odom".into(),
            child_frame_id: "base_link".into(),
            position_m: state.position,
            yaw_rad: state.yaw,
            linear_velocity_mps: state.velocity,
        },
        health: LioHealth {
            healthy,
            backend: kind,
            processed_frames: state.frames,
            imu_samples: state.imu_samples,
            registered_points: cloud.points.len(),
            status: status.into(),
        },
        registered_cloud: cloud,
    }
}

#[derive(Debug, Clone, Copy)]
struct Registration {
    translation: [f64; 2],
    yaw: f64,
    correspondences: usize,
}

fn bounded_registration_sample(points: &[TimedPoint]) -> Vec<&TimedPoint> {
    if points.len() <= MAX_REGISTRATION_POINTS {
        return points.iter().collect();
    }

    (0..MAX_REGISTRATION_POINTS)
        .map(|sample| {
            let index = sample * (points.len() - 1) / (MAX_REGISTRATION_POINTS - 1);
            &points[index]
        })
        .collect()
}

fn register_scan(reference: &[TimedPoint], scan: &[TimedPoint]) -> Result<Registration, String> {
    if reference.len() < 3 || scan.len() < 3 {
        return Err("bad geometry: fewer than three scan points".into());
    }
    let reference = bounded_registration_sample(reference);
    let scan = bounded_registration_sample(scan);
    let mut remaining_distance_evaluations = MAX_REGISTRATION_DISTANCE_EVALUATIONS;
    let mut translation = [0.0, 0.0];
    let mut yaw: f64 = 0.0;
    let gate_sq = 2.5_f64.powi(2);
    let mut final_count = 0;
    for _ in 0..MAX_REGISTRATION_ITERATIONS {
        let (sin_yaw, cos_yaw) = yaw.sin_cos();
        let mut pairs = Vec::new();
        for point in &scan {
            remaining_distance_evaluations = remaining_distance_evaluations
                .checked_sub(reference.len())
                .ok_or_else(|| "registration distance-evaluation budget exhausted".to_string())?;
            let transformed = [
                cos_yaw * point.x - sin_yaw * point.y + translation[0],
                sin_yaw * point.x + cos_yaw * point.y + translation[1],
            ];
            if let Some((nearest, distance_sq)) = reference
                .iter()
                .copied()
                .map(|candidate| {
                    let distance_sq = (candidate.x - transformed[0]).powi(2)
                        + (candidate.y - transformed[1]).powi(2);
                    (candidate, distance_sq)
                })
                .min_by(|left, right| left.1.total_cmp(&right.1))
                .filter(|(_, distance_sq)| *distance_sq <= gate_sq)
            {
                pairs.push((transformed, [nearest.x, nearest.y], distance_sq));
            }
        }
        if pairs.len() < 3 {
            return Err("bad geometry: insufficient gated correspondences".into());
        }
        pairs.sort_by(|left, right| left.2.total_cmp(&right.2));
        pairs.truncate(pairs.len().saturating_sub(pairs.len() / 4).max(3));
        final_count = pairs.len();
        let inv = 1.0 / pairs.len() as f64;
        let source_center = pairs.iter().fold([0.0; 2], |mut sum, pair| {
            sum[0] += pair.0[0] * inv;
            sum[1] += pair.0[1] * inv;
            sum
        });
        let target_center = pairs.iter().fold([0.0; 2], |mut sum, pair| {
            sum[0] += pair.1[0] * inv;
            sum[1] += pair.1[1] * inv;
            sum
        });
        let spread = pairs
            .iter()
            .map(|pair| {
                (pair.0[0] - source_center[0]).powi(2) + (pair.0[1] - source_center[1]).powi(2)
            })
            .sum::<f64>();
        if spread < 1e-6 {
            return Err("bad geometry: degenerate scan spread".into());
        }
        let (cross, dot) = pairs.iter().fold((0.0, 0.0), |(cross, dot), pair| {
            let sx = pair.0[0] - source_center[0];
            let sy = pair.0[1] - source_center[1];
            let tx = pair.1[0] - target_center[0];
            let ty = pair.1[1] - target_center[1];
            (cross + sx * ty - sy * tx, dot + sx * tx + sy * ty)
        });
        let delta_yaw = cross.atan2(dot);
        let (delta_sin, delta_cos) = delta_yaw.sin_cos();
        let delta_translation = [
            target_center[0] - (delta_cos * source_center[0] - delta_sin * source_center[1]),
            target_center[1] - (delta_sin * source_center[0] + delta_cos * source_center[1]),
        ];
        translation = [
            delta_cos * translation[0] - delta_sin * translation[1] + delta_translation[0],
            delta_sin * translation[0] + delta_cos * translation[1] + delta_translation[1],
        ];
        yaw = wrap(yaw + delta_yaw)?;
        if delta_yaw.abs() < 1e-7 && delta_translation[0].hypot(delta_translation[1]) < 1e-7 {
            break;
        }
    }
    Ok(Registration {
        translation,
        yaw,
        correspondences: final_count,
    })
}

impl LioEstimator for FastLio2Estimator {
    fn kind(&self) -> BackendKind {
        BackendKind::FastLio2
    }

    fn process(&mut self, frame: &LioInputFrame) -> Result<LioOutputFrame, String> {
        validate(frame, self.state.timestamp_ns)?;
        let dt = self.state.timestamp_ns.map_or(0.0, |previous| {
            (frame.cloud.timestamp_ns - previous) as f64 / 1e9
        });
        let samples = frame.imu.len() as f64;
        let mean_accel = frame.imu.iter().fold([0.0; 3], |mut sum, sample| {
            for (out, value) in sum.iter_mut().zip(sample.accel_mps2) {
                *out += value / samples;
            }
            sum
        });
        let mean_gyro_z = frame
            .imu
            .iter()
            .map(|sample| sample.gyro_rps[2])
            .sum::<f64>()
            / samples;
        for (axis, accel) in mean_accel.iter().copied().enumerate() {
            self.state.position[axis] += self.state.velocity[axis] * dt + 0.5 * accel * dt * dt;
            self.state.velocity[axis] += accel * dt;
        }
        self.state.yaw = wrap(self.state.yaw + mean_gyro_z * dt)?;
        let registration = self
            .state
            .previous_scan
            .as_deref()
            .map(|previous| register_scan(previous, &frame.cloud.points))
            .transpose();
        let (healthy, status) = match registration {
            Ok(Some(registration)) => {
                self.state.position[0] += 0.7 * registration.translation[0];
                self.state.position[1] += 0.7 * registration.translation[1];
                self.state.yaw = wrap(self.state.yaw + 0.7 * registration.yaw)?;
                (
                    true,
                    format!("tracking_scan_iterated:{}", registration.correspondences),
                )
            }
            Ok(None) => (true, "initializing".into()),
            Err(error) => (false, error),
        };
        self.state.previous_scan = Some(frame.cloud.points.clone());
        self.state.timestamp_ns = Some(frame.cloud.timestamp_ns);
        self.state.frames += 1;
        self.state.imu_samples += frame.imu.len() as u64;
        let registered = TimestampedCloud {
            timestamp_ns: frame.cloud.timestamp_ns,
            frame_id: "odom".into(),
            points: frame
                .cloud
                .points
                .iter()
                .map(|point| rotate_translate(point, self.state.yaw, self.state.position))
                .collect(),
        };
        Ok(output(
            self.kind(),
            &self.state,
            registered,
            healthy,
            &status,
        ))
    }
}

impl LioEstimator for PointLioEstimator {
    fn kind(&self) -> BackendKind {
        BackendKind::PointLio
    }

    fn process(&mut self, frame: &LioInputFrame) -> Result<LioOutputFrame, String> {
        validate(frame, self.state.timestamp_ns)?;
        let mut previous = self.state.timestamp_ns.unwrap_or(frame.imu[0].timestamp_ns);
        for sample in &frame.imu {
            let dt = (sample.timestamp_ns - previous).max(0) as f64 / 1e9;
            for axis in 0..3 {
                self.state.velocity[axis] += sample.accel_mps2[axis] * dt;
                self.state.position[axis] += self.state.velocity[axis] * dt;
            }
            self.state.yaw = wrap(self.state.yaw + sample.gyro_rps[2] * dt)?;
            previous = sample.timestamp_ns;
        }
        let tail_dt = (frame.cloud.timestamp_ns - previous).max(0) as f64 / 1e9;
        for axis in 0..3 {
            self.state.position[axis] += self.state.velocity[axis] * tail_dt;
        }
        let mean_gyro_z = frame
            .imu
            .iter()
            .map(|sample| sample.gyro_rps[2])
            .sum::<f64>()
            / frame.imu.len() as f64;
        let deskewed = frame
            .cloud
            .points
            .iter()
            .map(|point| {
                let deskew = mean_gyro_z * point.offset_ns as f64 / 1e9;
                rotate_translate(point, -deskew, [0.0; 3])
            })
            .collect::<Vec<_>>();
        let registration = self
            .state
            .previous_scan
            .as_deref()
            .map(|previous| register_scan(previous, &deskewed))
            .transpose();
        let (healthy, status) = match registration {
            Ok(Some(registration)) => {
                self.state.position[0] += 0.45 * registration.translation[0];
                self.state.position[1] += 0.45 * registration.translation[1];
                self.state.yaw = wrap(self.state.yaw + 0.45 * registration.yaw)?;
                (
                    true,
                    format!(
                        "tracking_point_incremental:{}",
                        registration.correspondences
                    ),
                )
            }
            Ok(None) => (true, "initializing".into()),
            Err(error) => (false, error),
        };
        self.state.previous_scan = Some(deskewed.clone());
        self.state.timestamp_ns = Some(frame.cloud.timestamp_ns);
        self.state.frames += 1;
        self.state.imu_samples += frame.imu.len() as u64;
        let registered = TimestampedCloud {
            timestamp_ns: frame.cloud.timestamp_ns,
            frame_id: "odom".into(),
            points: deskewed
                .iter()
                .map(|point| rotate_translate(point, self.state.yaw, self.state.position))
                .collect(),
        };
        Ok(output(
            self.kind(),
            &self.state,
            registered,
            healthy,
            &status,
        ))
    }
}

fn wrap(angle: f64) -> Result<f64, String> {
    if !angle.is_finite() {
        return Err("yaw must remain finite".into());
    }
    let wrapped = angle % TAU;
    Ok(if wrapped > PI {
        wrapped - TAU
    } else if wrapped < -PI {
        wrapped + TAU
    } else {
        wrapped
    })
}

pub struct LioBackendSlot {
    active: Option<Box<dyn LioEstimator + Send>>,
}

impl LioBackendSlot {
    #[must_use]
    pub fn new() -> Self {
        Self { active: None }
    }

    pub fn start(&mut self, kind: BackendKind) -> Result<(), String> {
        if self.active.is_some() {
            return Err("LIO session already has an active owner".into());
        }
        self.active = Some(match kind {
            BackendKind::FastLio2 => Box::<FastLio2Estimator>::default(),
            BackendKind::PointLio => Box::<PointLioEstimator>::default(),
        });
        Ok(())
    }

    pub fn process(&mut self, frame: &LioInputFrame) -> Result<LioOutputFrame, String> {
        self.active
            .as_mut()
            .ok_or_else(|| "LIO session has no active owner".to_string())?
            .process(frame)
    }

    pub fn stop(&mut self) {
        self.active = None;
    }

    #[must_use]
    pub fn active_kind(&self) -> Option<BackendKind> {
        self.active.as_ref().map(|backend| backend.kind())
    }
}

impl Default for LioBackendSlot {
    fn default() -> Self {
        Self::new()
    }
}

pub fn run_recorded(kind: BackendKind, input: &Path, output: &Path) -> Result<(), String> {
    let bytes = std::fs::read(input).map_err(|error| error.to_string())?;
    let frames: Vec<LioInputFrame> =
        serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
    let mut slot = LioBackendSlot::new();
    slot.start(kind)?;
    let results = frames
        .iter()
        .map(|frame| slot.process(frame))
        .collect::<Result<Vec<_>, _>>()?;
    let encoded = serde_json::to_vec_pretty(&results).map_err(|error| error.to_string())?;
    std::fs::write(output, encoded).map_err(|error| error.to_string())
}

pub fn cli(kind: BackendKind) -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let input = args
        .next()
        .ok_or_else(|| "usage: BACKEND INPUT OUTPUT".to_string())?;
    let output = args
        .next()
        .ok_or_else(|| "usage: BACKEND INPUT OUTPUT".to_string())?;
    if args.next().is_some() {
        return Err("usage: BACKEND INPUT OUTPUT".into());
    }
    run_recorded(kind, Path::new(&input), Path::new(&output))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn go2_adapter_config() -> Go2L2AdapterConfig {
        Go2L2AdapterConfig {
            lidar_frame_id: "go2_lidar".into(),
            imu_frame_id: "body".into(),
            body_frame_id: "body".into(),
            lidar_to_body_translation_m: [1.0, 2.0, 3.0],
            lidar_to_body_rotation_row_major: [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    fn push_l2_point(data: &mut Vec<u8>, intensity: u16, x: f32, y: f32, z: f32, offset_ns: u32) {
        data.extend_from_slice(&intensity.to_le_bytes());
        data.extend_from_slice(&[0, 0]);
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&offset_ns.to_le_bytes());
    }

    fn go2_cloud_fixture() -> Go2PointCloud2 {
        let mut data = Vec::new();
        push_l2_point(&mut data, 42, 1.0, 2.0, 3.0, 123);
        push_l2_point(&mut data, 7, f32::NAN, 0.0, 0.0, 999);
        push_l2_point(&mut data, 65_535, -1.0, 4.0, 0.5, 456);
        Go2PointCloud2 {
            timestamp_ns: 2_000_000_000,
            frame_id: "go2_lidar".into(),
            height: 1,
            width: 3,
            fields: vec![
                Go2PointField {
                    name: "offset_ns".into(),
                    offset: 16,
                    datatype: 6,
                    count: 1,
                },
                Go2PointField {
                    name: "intensity".into(),
                    offset: 0,
                    datatype: 4,
                    count: 1,
                },
                Go2PointField {
                    name: "z".into(),
                    offset: 12,
                    datatype: 7,
                    count: 1,
                },
                Go2PointField {
                    name: "x".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                Go2PointField {
                    name: "y".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
                Go2PointField {
                    name: "ring".into(),
                    offset: 2,
                    datatype: 4,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 20,
            row_step: 60,
            data,
        }
    }

    fn go2_imu_fixture() -> Vec<Go2ImuSample> {
        vec![
            Go2ImuSample {
                timestamp_ns: 1_800_000_000,
                frame_id: "body".into(),
                accel_mps2: [1.25, -2.5, 9.81],
                gyro_rps: [0.1, -0.2, 0.3],
            },
            Go2ImuSample {
                timestamp_ns: 1_900_000_000,
                frame_id: "body".into(),
                accel_mps2: [1.5, -2.0, 9.7],
                gyro_rps: [0.15, -0.25, 0.35],
            },
        ]
    }

    #[test]
    fn go2_l2_adapter_parses_layout_drops_nan_and_applies_extrinsic_once() {
        let frame = go2_adapter_config()
            .adapt(&go2_cloud_fixture(), &go2_imu_fixture())
            .unwrap();

        assert_eq!(frame.cloud.timestamp_ns, 2_000_000_000);
        assert_eq!(frame.cloud.frame_id, "body");
        assert_eq!(frame.cloud.points.len(), 2);
        assert_eq!(
            frame.cloud.points[0],
            TimedPoint {
                offset_ns: 123,
                x: -1.0,
                y: 3.0,
                z: 6.0,
                intensity: 42.0,
            }
        );
        assert_eq!(
            frame.cloud.points[1],
            TimedPoint {
                offset_ns: 456,
                x: -3.0,
                y: 1.0,
                z: 3.5,
                intensity: 65_535.0,
            }
        );
    }

    #[test]
    fn go2_l2_adapter_preserves_si_imu_values_and_axes() {
        let source = go2_imu_fixture();
        let frame = go2_adapter_config()
            .adapt(&go2_cloud_fixture(), &source)
            .unwrap();

        assert_eq!(frame.imu.len(), source.len());
        for (adapted, original) in frame.imu.iter().zip(source) {
            assert_eq!(adapted.timestamp_ns, original.timestamp_ns);
            assert_eq!(adapted.accel_mps2, original.accel_mps2);
            assert_eq!(adapted.gyro_rps, original.gyro_rps);
        }
    }

    #[test]
    fn go2_l2_adapter_uses_zero_for_absent_optional_fields() {
        let mut cloud = go2_cloud_fixture();
        cloud.width = 1;
        cloud.point_step = 12;
        cloud.row_step = 12;
        cloud.fields = vec![
            Go2PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            Go2PointField {
                name: "y".into(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            Go2PointField {
                name: "z".into(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        cloud.data.clear();
        cloud.data.extend_from_slice(&1.0_f32.to_le_bytes());
        cloud.data.extend_from_slice(&2.0_f32.to_le_bytes());
        cloud.data.extend_from_slice(&3.0_f32.to_le_bytes());

        let frame = go2_adapter_config()
            .adapt(&cloud, &go2_imu_fixture())
            .unwrap();
        assert_eq!(frame.cloud.points[0].intensity, 0.0);
        assert_eq!(frame.cloud.points[0].offset_ns, 0);
    }

    #[test]
    fn go2_l2_adapter_rejects_big_endian_and_invalid_layout() {
        let mut cloud = go2_cloud_fixture();
        cloud.is_bigendian = true;
        assert!(go2_adapter_config()
            .adapt(&cloud, &go2_imu_fixture())
            .unwrap_err()
            .contains("little-endian"));

        cloud.is_bigendian = false;
        cloud
            .fields
            .iter_mut()
            .find(|field| field.name == "x")
            .unwrap()
            .offset = 18;
        assert!(go2_adapter_config()
            .adapt(&cloud, &go2_imu_fixture())
            .unwrap_err()
            .contains("exceeds point_step"));
    }

    #[test]
    fn go2_l2_adapter_rejects_oversized_payload_and_imu_batch() {
        let mut cloud = go2_cloud_fixture();
        cloud.width = 1;
        cloud.height = 1;
        cloud.point_step = (MAX_GO2_L2_PAYLOAD_BYTES as u32) + 1;
        cloud.row_step = cloud.point_step;
        cloud.data.clear();
        let error = go2_adapter_config()
            .adapt(&cloud, &go2_imu_fixture())
            .unwrap_err();
        assert!(error.contains("payload"));

        let cloud = go2_cloud_fixture();
        let imu = vec![go2_imu_fixture()[0].clone(); MAX_IMU_SAMPLES + 1];
        let error = go2_adapter_config().adapt(&cloud, &imu).unwrap_err();
        assert!(error.contains("IMU samples"));
    }

    #[test]
    fn both_backends_consume_the_same_canonical_frame() {
        let mut imu = go2_imu_fixture();
        for sample in &mut imu {
            sample.accel_mps2 = [0.0; 3];
            sample.gyro_rps = [0.0; 3];
        }
        let frame = go2_adapter_config()
            .adapt(&go2_cloud_fixture(), &imu)
            .unwrap();
        let original = frame.clone();
        let mut fastlio2 = FastLio2Estimator::default();
        let mut pointlio = PointLioEstimator::default();

        let fastlio2_output = fastlio2.process(&frame).unwrap();
        let pointlio_output = pointlio.process(&frame).unwrap();

        assert_eq!(frame, original);
        assert_eq!(
            fastlio2_output.registered_cloud,
            pointlio_output.registered_cloud
        );
        assert_eq!(fastlio2_output.health.registered_points, 2);
        assert_eq!(pointlio_output.health.registered_points, 2);
    }

    fn maximum_size_frame(timestamp_ns: i64) -> LioInputFrame {
        let points = (0..MAX_CLOUD_POINTS)
            .map(|index| TimedPoint {
                offset_ns: (index % 1_000) as u64 * 1_000,
                x: (index % 500) as f64 * 0.01,
                y: (index / 500) as f64 * 0.01,
                z: (index % 7) as f64 * 0.001,
                intensity: (index % 256) as f64,
            })
            .collect();
        LioInputFrame {
            cloud: TimestampedCloud {
                timestamp_ns,
                frame_id: "mid360_link".into(),
                points,
            },
            imu: vec![
                ImuSample {
                    timestamp_ns: timestamp_ns - 100_000_000,
                    accel_mps2: [0.1, 0.0, 0.0],
                    gyro_rps: [0.0, 0.0, 0.01],
                },
                ImuSample {
                    timestamp_ns: timestamp_ns - 50_000_000,
                    accel_mps2: [0.1, 0.0, 0.0],
                    gyro_rps: [0.0, 0.0, 0.01],
                },
            ],
        }
    }

    #[test]
    fn maximum_legal_cloud_has_bounded_registration_work_and_finite_output() {
        let first = maximum_size_frame(1_000_000_000);
        let second = maximum_size_frame(2_000_000_000);
        assert_eq!(
            bounded_registration_sample(&first.cloud.points).len(),
            MAX_REGISTRATION_POINTS
        );
        assert_eq!(MAX_REGISTRATION_DISTANCE_EVALUATIONS, 12 * 1_024 * 1_024);

        for kind in [BackendKind::FastLio2, BackendKind::PointLio] {
            let mut slot = LioBackendSlot::new();
            slot.start(kind).unwrap();
            slot.process(&first).unwrap();
            let output = slot.process(&second).unwrap();
            assert!(output.health.healthy);
            assert_eq!(output.registered_cloud.points.len(), MAX_CLOUD_POINTS);
            assert!(output.odometry.yaw_rad.is_finite());
            assert!(output
                .odometry
                .position_m
                .iter()
                .chain(&output.odometry.linear_velocity_mps)
                .all(|value| value.is_finite()));
            assert!(output.registered_cloud.points.iter().all(|point| {
                [point.x, point.y, point.z, point.intensity]
                    .into_iter()
                    .all(f64::is_finite)
            }));
        }
    }
}
