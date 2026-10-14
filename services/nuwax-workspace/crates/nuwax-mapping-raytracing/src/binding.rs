// Copyright 2026 Nuwax Team
// SPDX-License-Identifier: Apache-2.0

//! Language-neutral JSON boundary for the ray-tracing runtimes.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::debug::{RayTraceComparison, RayTraceDebugEvent};
use crate::transformer::{Pose, RayTraceError, RayTraceFrame, RayTraceMap, RayTraceOutput};
use crate::voxel_ray_tracer::{
    conservative_ray_step_bound, Config, MAX_REQUEST_RAY_STEPS, MAX_STEPS_PER_RAY,
    MIN_VOXEL_SIZE_METERS,
};

const MAX_RESPONSE_BYTES: u64 = 64 * 1024 * 1024;
const MAX_REQUEST_FRAMES: usize = 65_536;
// Conservative JSON-plus-retained-memory estimates for point-heavy fields.
const POINT_RESPONSE_BYTES: u64 = 64;
const NORMAL_RESPONSE_BYTES: u64 = 160;
const RETAINED_VOXEL_BYTES: u64 = 256;
const MAP_OUTPUT_FIXED_BYTES: u64 = 256;
const COMPARISON_OUTPUT_FIXED_BYTES: u64 = 1_024;
const COMPARISON_STATE_FIXED_BYTES: u64 = 4_096;
const MAX_DEBUG_TRAJECTORY_POINTS: u64 = 4_096;

/// One complete, one-shot boundary request.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RayTraceRequest {
    pub mode: RayTraceMode,
    #[serde(default)]
    pub config: ConfigDto,
    /// Overrides `config.emit_every` for this request when present.
    #[serde(default, alias = "emit_every")]
    pub update_every: Option<u32>,
    pub frames: Vec<FrameDto>,
    /// Flush a final partial cadence batch. Defaults to true for batch callers.
    #[serde(default = "default_finish")]
    pub finish: bool,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RayTraceMode {
    Map,
    Comparison,
}

/// JSON representation of one timestamped sensor-frame cloud.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameDto {
    pub points: Vec<[f32; 3]>,
    #[serde(default)]
    pub pose: Option<PoseDto>,
    pub timestamp: f64,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseDto {
    pub translation: [f32; 3],
    pub rotation_xyzw: [f32; 4],
}

/// Config DTO with defaults matching the migrated Dimos Python boundary.
#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ConfigDto {
    pub voxel_size: f32,
    pub max_range: f32,
    pub ray_subsample: u32,
    pub shadow_depth: f32,
    pub grace_depth: f32,
    pub min_health: i32,
    pub max_health: i32,
    pub graze_cos: f32,
    pub recency_window: u32,
    pub emit_every: u32,
    pub global_emit_every: u32,
    pub region_percentile: f32,
}

impl Default for ConfigDto {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: -2,
            max_health: 1,
            graze_cos: 0.7,
            recency_window: 15,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        }
    }
}

impl From<ConfigDto> for Config {
    fn from(value: ConfigDto) -> Self {
        Self {
            voxel_size: value.voxel_size,
            max_range: value.max_range,
            ray_subsample: value.ray_subsample,
            shadow_depth: value.shadow_depth,
            grace_depth: value.grace_depth,
            min_health: value.min_health,
            max_health: value.max_health,
            graze_cos: value.graze_cos,
            recency_window: value.recency_window,
            emit_every: value.emit_every,
            global_emit_every: value.global_emit_every,
            region_percentile: value.region_percentile,
        }
    }
}

#[derive(Debug, Serialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum RayTraceResponse {
    Map {
        frame_id: &'static str,
        outputs: Vec<MapOutputDto>,
    },
    Comparison {
        frame_id: &'static str,
        outputs: Vec<RayTraceDebugEvent>,
    },
}

#[derive(Debug, Serialize)]
pub struct MapOutputDto {
    pub world_points: Vec<(f32, f32, f32)>,
    pub frame_count: u64,
    pub timestamp: f64,
    pub bounds: BoundsDto,
}

#[derive(Debug, Serialize)]
pub struct BoundsDto {
    pub center_x: f32,
    pub center_y: f32,
    pub radius: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl From<RayTraceOutput> for MapOutputDto {
    fn from(value: RayTraceOutput) -> Self {
        Self {
            world_points: value.world_points,
            frame_count: value.frame_count,
            timestamp: value.timestamp,
            bounds: BoundsDto {
                center_x: value.bounds.center_x,
                center_y: value.bounds.center_y,
                radius: value.bounds.radius,
                z_min: value.bounds.z_min,
                z_max: value.bounds.z_max,
            },
        }
    }
}

#[derive(Debug, Error)]
pub enum BindingError {
    #[error(transparent)]
    RayTrace(#[from] RayTraceError),
    #[error("invalid ray-tracing request work: {0}")]
    RequestWork(String),
}

/// Execute a parsed request against the production ray-tracing runtime.
pub fn execute(request: RayTraceRequest) -> Result<RayTraceResponse, BindingError> {
    validate_request_work(&request)?;
    let RayTraceRequest {
        mode,
        config,
        update_every,
        frames,
        finish,
    } = request;
    let mut config = Config::from(config);
    if let Some(update_every) = update_every {
        config.emit_every = update_every;
    }

    match mode {
        RayTraceMode::Map => execute_map(config, frames, finish),
        RayTraceMode::Comparison => execute_comparison(config, frames, finish),
    }
}

/// Reject requests whose aggregate DDA work or retained response estimate
/// exceeds the public library contract, including callers that bypass the JSON
/// process executable.
pub fn validate_request_work(request: &RayTraceRequest) -> Result<(), BindingError> {
    let config = &request.config;
    if request.frames.len() > MAX_REQUEST_FRAMES {
        return Err(BindingError::RequestWork(format!(
            "request contains more than the {MAX_REQUEST_FRAMES}-frame limit"
        )));
    }
    if !config.voxel_size.is_finite() || config.voxel_size < MIN_VOXEL_SIZE_METERS {
        return Err(BindingError::RequestWork(format!(
            "voxel_size must be finite and at least {MIN_VOXEL_SIZE_METERS} m"
        )));
    }
    if !config.max_range.is_finite() || config.max_range < 0.0 {
        return Err(BindingError::RequestWork(
            "max_range must be finite and non-negative".to_owned(),
        ));
    }
    if config.ray_subsample == 0 {
        return Err(BindingError::RequestWork(
            "ray_subsample must be greater than zero".to_owned(),
        ));
    }
    if config.max_range > 0.0 {
        let configured_steps = conservative_ray_step_bound(
            f64::from(config.max_range),
            f64::from(config.shadow_depth),
            f64::from(config.voxel_size),
        )
        .ok_or_else(|| {
            BindingError::RequestWork(
                "ray work configuration must contain finite non-negative values".to_owned(),
            )
        })?;
        if configured_steps > MAX_STEPS_PER_RAY {
            return Err(BindingError::RequestWork(format!(
                "configured ray requires up to {configured_steps} steps, exceeding the {MAX_STEPS_PER_RAY}-step per-ray limit"
            )));
        }
    }

    let multiplier = match request.mode {
        RayTraceMode::Map => 1_u64,
        RayTraceMode::Comparison => 4_u64,
    };
    let total_input_points = request.frames.iter().try_fold(0_u64, |total, frame| {
        let points = u64::try_from(frame.points.len()).map_err(|_| work_overflow())?;
        total.checked_add(points).ok_or_else(work_overflow)
    })?;
    let mut total_steps = 0_u64;
    let mut weighted_voxel_bound = 0_u64;
    let mut response_bytes = 0_u64;
    let mut posed_frames = 0_u64;
    let mut last_frame_points = 0_u64;
    let mut batch_points = 0_u64;
    let mut batch_origins = 0_u64;
    let mut pending = false;
    let emit_every = request.update_every.unwrap_or(config.emit_every);
    for (frame_index, frame) in request.frames.iter().enumerate() {
        if frame.pose.is_none() {
            continue;
        }
        posed_frames = posed_frames.checked_add(1).ok_or_else(work_overflow)?;
        last_frame_points = u64::try_from(frame.points.len()).map_err(|_| work_overflow())?;
        if last_frame_points > 0 {
            if emit_every == 0 {
                batch_points = 0;
                batch_origins = 0;
            }
            batch_points = batch_points
                .checked_add(last_frame_points)
                .ok_or_else(work_overflow)?;
            batch_origins = batch_origins.checked_add(1).ok_or_else(work_overflow)?;
        }
        pending = true;
        for (point_index, point) in frame.points.iter().enumerate() {
            weighted_voxel_bound = weighted_voxel_bound
                .checked_add(multiplier)
                .ok_or_else(work_overflow)?;
            if point_index % config.ray_subsample as usize != 0 {
                continue;
            }
            let range = f64::from(point[0])
                .hypot(f64::from(point[1]))
                .hypot(f64::from(point[2]));
            if !range.is_finite() {
                return Err(BindingError::RequestWork(format!(
                    "point at frame {frame_index}, index {point_index} has an unbounded range"
                )));
            }
            if config.max_range > 0.0 && range > f64::from(config.max_range) {
                continue;
            }
            let steps = conservative_ray_step_bound(
                range,
                f64::from(config.shadow_depth),
                f64::from(config.voxel_size),
            )
            .ok_or_else(|| {
                BindingError::RequestWork(format!(
                    "point at frame {frame_index}, index {point_index} has an invalid ray work estimate"
                ))
            })?;
            if steps > MAX_STEPS_PER_RAY {
                return Err(BindingError::RequestWork(format!(
                    "point at frame {frame_index}, index {point_index} requires {steps} steps, exceeding the {MAX_STEPS_PER_RAY}-step per-ray limit"
                )));
            }
            let request_steps = steps.checked_mul(multiplier).ok_or_else(work_overflow)?;
            total_steps = total_steps
                .checked_add(request_steps)
                .ok_or_else(work_overflow)?;
            weighted_voxel_bound = weighted_voxel_bound
                .checked_add(request_steps)
                .ok_or_else(work_overflow)?;
            if total_steps > MAX_REQUEST_RAY_STEPS {
                return Err(BindingError::RequestWork(format!(
                    "request requires more than the {MAX_REQUEST_RAY_STEPS}-step total ray-work limit"
                )));
            }
        }
        validate_retained_state(
            request.mode,
            RetainedStateEstimate {
                total_input_points,
                weighted_voxel_bound,
                batch_points,
                batch_origins,
                sensor_points: last_frame_points,
                posed_frames,
                response_bytes,
            },
        )?;
        if emit_every > 0 && posed_frames % u64::from(emit_every) == 0 {
            add_response_event(
                &mut response_bytes,
                request.mode,
                weighted_voxel_bound,
                last_frame_points,
                posed_frames,
            )?;
            validate_retained_state(
                request.mode,
                RetainedStateEstimate {
                    total_input_points,
                    weighted_voxel_bound,
                    batch_points,
                    batch_origins,
                    sensor_points: last_frame_points,
                    posed_frames,
                    response_bytes,
                },
            )?;
            batch_points = 0;
            batch_origins = 0;
            pending = false;
        }
    }
    if request.finish && pending {
        add_response_event(
            &mut response_bytes,
            request.mode,
            weighted_voxel_bound,
            last_frame_points,
            posed_frames,
        )?;
        validate_retained_state(
            request.mode,
            RetainedStateEstimate {
                total_input_points,
                weighted_voxel_bound,
                batch_points,
                batch_origins,
                sensor_points: last_frame_points,
                posed_frames,
                response_bytes,
            },
        )?;
    }
    Ok(())
}

struct RetainedStateEstimate {
    total_input_points: u64,
    weighted_voxel_bound: u64,
    batch_points: u64,
    batch_origins: u64,
    sensor_points: u64,
    posed_frames: u64,
    response_bytes: u64,
}

fn validate_retained_state(
    mode: RayTraceMode,
    estimate: RetainedStateEstimate,
) -> Result<(), BindingError> {
    let RetainedStateEstimate {
        total_input_points,
        weighted_voxel_bound,
        batch_points,
        batch_origins,
        sensor_points,
        posed_frames,
        response_bytes,
    } = estimate;
    let map_count = match mode {
        RayTraceMode::Map => 1_u64,
        RayTraceMode::Comparison => 4_u64,
    };
    let input_bytes = total_input_points
        .checked_mul(POINT_RESPONSE_BYTES)
        .ok_or_else(work_overflow)?;
    let map_bytes = weighted_voxel_bound
        .checked_mul(RETAINED_VOXEL_BYTES)
        .ok_or_else(work_overflow)?;
    let batch_bytes = batch_points
        .checked_add(batch_origins)
        .and_then(|points| points.checked_mul(map_count))
        .and_then(|points| points.checked_mul(POINT_RESPONSE_BYTES))
        .ok_or_else(work_overflow)?;
    let mut state_bytes = input_bytes
        .checked_add(map_bytes)
        .and_then(|bytes| bytes.checked_add(batch_bytes))
        .ok_or_else(work_overflow)?;
    if matches!(mode, RayTraceMode::Comparison) {
        let last_frame = sensor_points
            .checked_mul(map_count)
            .and_then(|points| points.checked_mul(POINT_RESPONSE_BYTES))
            .ok_or_else(work_overflow)?;
        let trajectory = posed_frames
            .min(MAX_DEBUG_TRAJECTORY_POINTS)
            .checked_mul(POINT_RESPONSE_BYTES)
            .ok_or_else(work_overflow)?;
        state_bytes = state_bytes
            .checked_add(last_frame)
            .and_then(|bytes| bytes.checked_add(trajectory))
            .and_then(|bytes| bytes.checked_add(COMPARISON_STATE_FIXED_BYTES))
            .ok_or_else(work_overflow)?;
    }
    let total_bytes = state_bytes
        .checked_add(response_bytes)
        .ok_or_else(work_overflow)?;
    if total_bytes > MAX_RESPONSE_BYTES {
        return Err(BindingError::RequestWork(format!(
            "estimated retained state and response exceed the {MAX_RESPONSE_BYTES}-byte limit"
        )));
    }
    Ok(())
}

fn add_response_event(
    response_bytes: &mut u64,
    mode: RayTraceMode,
    weighted_voxel_bound: u64,
    sensor_points: u64,
    posed_frames: u64,
) -> Result<(), BindingError> {
    let event_bytes = match mode {
        RayTraceMode::Map => weighted_voxel_bound
            .checked_mul(POINT_RESPONSE_BYTES)
            .and_then(|bytes| bytes.checked_add(MAP_OUTPUT_FIXED_BYTES))
            .ok_or_else(work_overflow)?,
        RayTraceMode::Comparison => {
            let world_points = weighted_voxel_bound
                .checked_mul(POINT_RESPONSE_BYTES)
                .ok_or_else(work_overflow)?;
            let normals = weighted_voxel_bound
                .checked_mul(NORMAL_RESPONSE_BYTES)
                .ok_or_else(work_overflow)?;
            let sensor_points = sensor_points
                .checked_mul(POINT_RESPONSE_BYTES)
                .ok_or_else(work_overflow)?;
            let trajectory = posed_frames
                .min(MAX_DEBUG_TRAJECTORY_POINTS)
                .checked_mul(POINT_RESPONSE_BYTES)
                .ok_or_else(work_overflow)?;
            world_points
                .checked_add(normals)
                .and_then(|bytes| bytes.checked_add(sensor_points))
                .and_then(|bytes| bytes.checked_add(trajectory))
                .and_then(|bytes| bytes.checked_add(COMPARISON_OUTPUT_FIXED_BYTES))
                .ok_or_else(work_overflow)?
        }
    };
    *response_bytes = response_bytes
        .checked_add(event_bytes)
        .ok_or_else(work_overflow)?;
    if *response_bytes > MAX_RESPONSE_BYTES {
        return Err(BindingError::RequestWork(format!(
            "estimated response exceeds the {MAX_RESPONSE_BYTES}-byte limit"
        )));
    }
    Ok(())
}

fn work_overflow() -> BindingError {
    BindingError::RequestWork("request work estimate overflowed".to_owned())
}

fn execute_map(
    config: Config,
    frames: Vec<FrameDto>,
    finish: bool,
) -> Result<RayTraceResponse, BindingError> {
    let mut runtime = RayTraceMap::new(config)?;
    let mut outputs = Vec::new();
    for frame in frames {
        if let Some(output) = runtime.process(frame.into())? {
            outputs.push(output.into());
        }
    }
    if finish {
        if let Some(output) = runtime.finish() {
            outputs.push(output.into());
        }
    }
    Ok(RayTraceResponse::Map {
        frame_id: "world",
        outputs,
    })
}

fn execute_comparison(
    config: Config,
    frames: Vec<FrameDto>,
    finish: bool,
) -> Result<RayTraceResponse, BindingError> {
    let mut runtime = RayTraceComparison::new(&config)?;
    let mut outputs = Vec::new();
    for frame in frames {
        if let Some(output) = runtime.process(frame.into())? {
            outputs.push(output);
        }
    }
    if finish {
        if let Some(output) = runtime.finish() {
            outputs.push(output);
        }
    }
    Ok(RayTraceResponse::Comparison {
        frame_id: "world",
        outputs,
    })
}

impl From<FrameDto> for RayTraceFrame {
    fn from(value: FrameDto) -> Self {
        Self {
            points: value
                .points
                .into_iter()
                .map(|point| (point[0], point[1], point[2]))
                .collect(),
            pose: value.pose.map(Into::into),
            timestamp: value.timestamp,
        }
    }
}

impl From<PoseDto> for Pose {
    fn from(value: PoseDto) -> Self {
        Self {
            translation: (
                value.translation[0],
                value.translation[1],
                value.translation[2],
            ),
            rotation_xyzw: (
                value.rotation_xyzw[0],
                value.rotation_xyzw[1],
                value.rotation_xyzw[2],
                value.rotation_xyzw[3],
            ),
        }
    }
}

const fn default_finish() -> bool {
    true
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn direct_library_execute_enforces_total_work_budget() {
        let points = vec![[49.0, 0.0, 0.0]; 600];
        let request: RayTraceRequest = serde_json::from_value(json!({
            "mode": "map",
            "config": {
                "voxel_size": 0.001,
                "max_range": 50.0,
                "ray_subsample": 1,
                "shadow_depth": 0.2
            },
            "frames": [{
                "points": points,
                "pose": {
                    "translation": [0.0, 0.0, 0.0],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                },
                "timestamp": 1.0
            }],
            "finish": false
        }))
        .unwrap();

        let error = execute(request).unwrap_err();
        assert!(error.to_string().contains("total ray-work limit"));
    }

    #[test]
    fn response_budget_accepts_limit_and_rejects_next_byte() {
        let mut accepted = MAX_RESPONSE_BYTES - MAP_OUTPUT_FIXED_BYTES;
        add_response_event(&mut accepted, RayTraceMode::Map, 0, 0, 0).unwrap();
        assert_eq!(accepted, MAX_RESPONSE_BYTES);

        let mut rejected = MAX_RESPONSE_BYTES - MAP_OUTPUT_FIXED_BYTES + 1;
        let error = add_response_event(&mut rejected, RayTraceMode::Map, 0, 0, 0).unwrap_err();
        assert!(error.to_string().contains("estimated response exceeds"));
    }

    #[test]
    fn direct_library_rejects_quadratic_comparison_response_before_construction() {
        let frames = (0..500)
            .map(|index| {
                json!({
                    "points": [[1.0, 0.0, 0.0]],
                    "pose": {
                        "translation": [index as f32 * 2.0, 0.0, 0.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                    },
                    "timestamp": index as f64
                })
            })
            .collect::<Vec<_>>();
        let request: RayTraceRequest = serde_json::from_value(json!({
            "mode": "comparison",
            "config": {
                "voxel_size": 0.5,
                "max_range": 2.0,
                "shadow_depth": 0.0,
                "min_health": 0,
                "emit_every": 1
            },
            "frames": frames,
            "finish": true
        }))
        .unwrap();

        let error = execute(request).unwrap_err();
        assert!(error
            .to_string()
            .contains("estimated retained state and response exceed"));
    }

    #[test]
    fn direct_library_limits_retained_state_without_an_emission() {
        let points = vec![[1.0, 0.0, 0.0]; 70_000];
        let request: RayTraceRequest = serde_json::from_value(json!({
            "mode": "comparison",
            "config": {
                "voxel_size": 0.5,
                "max_range": 2.0,
                "ray_subsample": 100_000,
                "shadow_depth": 0.0,
                "emit_every": 0
            },
            "frames": [{
                "points": points,
                "pose": {
                    "translation": [0.0, 0.0, 0.0],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                },
                "timestamp": 1.0
            }],
            "finish": false
        }))
        .unwrap();

        let error = execute(request).unwrap_err();
        assert!(error
            .to_string()
            .contains("estimated retained state and response exceed"));
    }

    #[test]
    fn direct_library_rejects_single_long_ray_without_an_emission() {
        let request: RayTraceRequest = serde_json::from_value(json!({
            "mode": "comparison",
            "config": {
                "voxel_size": 0.001,
                "max_range": 115.0,
                "shadow_depth": 0.0,
                "emit_every": 0
            },
            "frames": [{
                "points": [[115.0, 0.0, 0.0]],
                "pose": {
                    "translation": [0.0, 0.0, 0.0],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                },
                "timestamp": 1.0
            }],
            "finish": false
        }))
        .unwrap();

        let error = execute(request).unwrap_err();
        assert!(error
            .to_string()
            .contains("estimated retained state and response exceed"));
    }

    #[test]
    fn map_emission_peak_includes_batch_buffers_and_new_event() {
        let points = (0..160_000)
            .map(|index| [index as f32 + 2.0, 0.0, 0.0])
            .collect::<Vec<_>>();
        let request: RayTraceRequest = serde_json::from_value(json!({
            "mode": "map",
            "config": {
                "voxel_size": 0.5,
                "max_range": 1.0,
                "shadow_depth": 0.0,
                "emit_every": 1
            },
            "frames": [{
                "points": points,
                "pose": {
                    "translation": [0.0, 0.0, 0.0],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                },
                "timestamp": 1.0
            }],
            "finish": false
        }))
        .unwrap();

        let error = execute(request).unwrap_err();
        assert!(error
            .to_string()
            .contains("estimated retained state and response exceed"));
    }
}
