//! Shared LIO backend slot with two independently executable estimator implementations.

#![deny(unsafe_code)]

pub mod native;

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use std::path::Path;

const MAX_CLOUD_POINTS: usize = 250_000;
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
