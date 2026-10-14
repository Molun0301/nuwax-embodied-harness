// Copyright 2026 Nuwax Team
// SPDX-License-Identifier: Apache-2.0

//! Stateful sensor-frame point-cloud transformer for the ray-traced voxel map.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use thiserror::Error;
use validator::Validate;

use crate::voxel_ray_tracer::{
    batch_local_bounds, iter_global_normals, iter_global_points, update_map_prevalidated,
    validate_update_inputs, Config, LocalBounds, UpdateMapError, VoxelMap,
};

pub type Point3 = (f32, f32, f32);

/// A world pose represented by translation followed by an `(x, y, z, w)` quaternion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: Point3,
    pub rotation_xyzw: (f32, f32, f32, f32),
}

impl Pose {
    pub const fn identity_at(x: f32, y: f32, z: f32) -> Self {
        Self {
            translation: (x, y, z),
            rotation_xyzw: (0.0, 0.0, 0.0, 1.0),
        }
    }
}

/// One timestamped sensor-frame cloud. Frames without a pose are skipped.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTraceFrame {
    pub points: Vec<Point3>,
    pub pose: Option<Pose>,
    pub timestamp: f64,
}

/// Cylinder containing the points accumulated since the previous emission.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegionBounds {
    pub center_x: f32,
    pub center_y: f32,
    pub radius: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl RegionBounds {
    pub const fn as_tuple(self) -> (f32, f32, f32, f32, f32) {
        (
            self.center_x,
            self.center_y,
            self.radius,
            self.z_min,
            self.z_max,
        )
    }
}

/// A local world-frame view of the accumulated map.
#[derive(Clone, Debug, PartialEq)]
pub struct RayTraceOutput {
    pub world_points: Vec<Point3>,
    pub frame_count: u64,
    pub timestamp: f64,
    pub bounds: RegionBounds,
}

#[derive(Debug, Error, PartialEq)]
pub enum RayTraceError {
    #[error("invalid ray-tracing configuration: {0}")]
    InvalidConfig(String),
    #[error("frame timestamp must be finite")]
    NonFiniteTimestamp,
    #[error("pose contains a non-finite value")]
    NonFinitePose,
    #[error("pose quaternion must have a finite, non-zero norm")]
    InvalidQuaternion,
    #[error("point at index {index} contains a non-finite value")]
    NonFinitePoint { index: usize },
    #[error(transparent)]
    UpdateMap(#[from] UpdateMapError),
}

/// Active, transport-independent equivalent of Dimos `RayTraceMap`.
///
/// The runtime accepts sensor-frame clouds, registers posed frames into the
/// world, updates the existing voxel-ray algorithm, and emits on `emit_every`.
/// Call [`Self::finish`] once the input stream ends to flush a remainder.
pub struct RayTraceMap {
    config: Config,
    map: VoxelMap,
    frame_count: u64,
    last_timestamp: Option<f64>,
    last_origin: Option<Point3>,
    batch_points: Vec<Point3>,
    batch_origins: Vec<Point3>,
    pending: bool,
}

pub(crate) struct PreparedRayTraceFrame {
    world_points: Vec<Point3>,
    origin: Point3,
    timestamp: f64,
}

impl RayTraceMap {
    pub fn new(config: Config) -> Result<Self, RayTraceError> {
        validate_config(&config)?;
        Ok(Self {
            config,
            map: VoxelMap::default(),
            frame_count: 0,
            last_timestamp: None,
            last_origin: None,
            batch_points: Vec::new(),
            batch_origins: Vec::new(),
            pending: false,
        })
    }

    /// Process one frame, returning an output only when the cadence is reached.
    /// Invalid frames are rejected before any mapper state is changed.
    pub fn process(
        &mut self,
        frame: RayTraceFrame,
    ) -> Result<Option<RayTraceOutput>, RayTraceError> {
        let Some(prepared) = self.prepare_frame(frame)? else {
            return Ok(None);
        };
        Ok(self.apply_prepared(prepared))
    }

    pub(crate) fn prepare_frame(
        &self,
        frame: RayTraceFrame,
    ) -> Result<Option<PreparedRayTraceFrame>, RayTraceError> {
        let Some(pose) = frame.pose else {
            return Ok(None);
        };
        if !frame.timestamp.is_finite() {
            return Err(RayTraceError::NonFiniteTimestamp);
        }

        let (origin, rotation) = validate_pose(pose)?;
        let translation = Vector3::new(origin.0, origin.1, origin.2);
        let mut world_points = Vec::with_capacity(frame.points.len());
        for (index, point) in frame.points.into_iter().enumerate() {
            if !point.0.is_finite() || !point.1.is_finite() || !point.2.is_finite() {
                return Err(RayTraceError::NonFinitePoint { index });
            }
            let world =
                rotation.transform_vector(&Vector3::new(point.0, point.1, point.2)) + translation;
            if !world.iter().all(|value| value.is_finite()) {
                return Err(RayTraceError::NonFinitePoint { index });
            }
            world_points.push((world.x, world.y, world.z));
        }
        validate_update_inputs(origin, &world_points, &self.config)?;

        Ok(Some(PreparedRayTraceFrame {
            world_points,
            origin,
            timestamp: frame.timestamp,
        }))
    }

    pub(crate) fn apply_prepared(
        &mut self,
        prepared: PreparedRayTraceFrame,
    ) -> Option<RayTraceOutput> {
        let PreparedRayTraceFrame {
            world_points,
            origin,
            timestamp,
        } = prepared;
        update_map_prevalidated(&mut self.map, origin, &world_points, &self.config);
        if !world_points.is_empty() {
            if self.config.emit_every == 0 {
                self.batch_points.clear();
                self.batch_origins.clear();
            }
            self.batch_points.extend_from_slice(&world_points);
            self.batch_origins.push(origin);
        }
        self.frame_count += 1;
        self.last_timestamp = Some(timestamp);
        self.last_origin = Some(origin);
        self.pending = true;

        if self.config.emit_every > 0 && self.frame_count % u64::from(self.config.emit_every) == 0 {
            return Some(self.emit());
        }
        None
    }

    /// Flush a final partial batch. Repeated calls without new input return `None`.
    pub fn finish(&mut self) -> Option<RayTraceOutput> {
        self.pending.then(|| self.emit())
    }

    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Snapshot every healthy global voxel and its fitted normal.
    #[must_use]
    pub fn global_snapshot(&self) -> Vec<(Point3, [f32; 3])> {
        iter_global_normals(&self.map, self.config.voxel_size).collect()
    }

    fn emit(&mut self) -> RayTraceOutput {
        let origin = self
            .last_origin
            .expect("pending output always has a posed frame");
        let bounds = if self.batch_origins.is_empty() {
            RegionBounds {
                center_x: origin.0,
                center_y: origin.1,
                radius: 0.0,
                z_min: origin.2,
                z_max: origin.2,
            }
        } else {
            let margin = self.config.shadow_depth + self.config.voxel_size;
            let (center_x, center_y, radius, z_min, z_max) = batch_local_bounds(
                &self.batch_points,
                &self.batch_origins,
                self.config.region_percentile,
                margin,
            );
            RegionBounds {
                center_x,
                center_y,
                radius,
                z_min,
                z_max,
            }
        };
        let local_bounds = LocalBounds {
            origin_x: bounds.center_x,
            origin_y: bounds.center_y,
            r_xy_max_sq: bounds.radius * bounds.radius,
            z_min: bounds.z_min,
            z_max: bounds.z_max,
        };
        let world_points = iter_global_points(&self.map, self.config.voxel_size)
            .filter(|&(x, y, z)| local_bounds.contains(x, y, z))
            .collect();

        self.batch_points.clear();
        self.batch_origins.clear();
        self.pending = false;

        RayTraceOutput {
            world_points,
            frame_count: self.frame_count,
            timestamp: self
                .last_timestamp
                .expect("pending output always has a timestamp"),
            bounds,
        }
    }
}

fn validate_config(config: &Config) -> Result<(), RayTraceError> {
    config
        .validate()
        .map_err(|error| RayTraceError::InvalidConfig(error.to_string()))?;
    let finite = [
        config.voxel_size,
        config.max_range,
        config.shadow_depth,
        config.grace_depth,
        config.graze_cos,
        config.region_percentile,
    ]
    .into_iter()
    .all(f32::is_finite);
    if !finite || !(config.shadow_depth + config.voxel_size).is_finite() {
        return Err(RayTraceError::InvalidConfig(
            "all floating-point fields and the bounds margin must be finite".to_owned(),
        ));
    }
    Ok(())
}

fn validate_pose(pose: Pose) -> Result<(Point3, UnitQuaternion<f32>), RayTraceError> {
    let values = [
        pose.translation.0,
        pose.translation.1,
        pose.translation.2,
        pose.rotation_xyzw.0,
        pose.rotation_xyzw.1,
        pose.rotation_xyzw.2,
        pose.rotation_xyzw.3,
    ];
    if !values.into_iter().all(f32::is_finite) {
        return Err(RayTraceError::NonFinitePose);
    }
    let (x, y, z, w) = pose.rotation_xyzw;
    let quaternion = Quaternion::new(w, x, y, z);
    let norm_squared = quaternion.norm_squared();
    if !norm_squared.is_finite() || norm_squared <= f32::EPSILON {
        return Err(RayTraceError::InvalidQuaternion);
    }
    Ok((pose.translation, UnitQuaternion::new_normalize(quaternion)))
}
