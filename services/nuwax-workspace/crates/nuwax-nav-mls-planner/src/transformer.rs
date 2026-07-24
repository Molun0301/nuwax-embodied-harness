// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Stateful point-cloud to MLS path transformation.

use std::time::Instant;
use validator::Validate;

use crate::edges::edges_to_segments;
use crate::mls_planner::{Config, Planner, RegionBounds};
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Pose and map observation consumed by [`MlsPlanTransformer`].
pub struct MapObservation {
    /// Observation identifier retained in the output.
    pub id: u64,
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Sensor pose `(x, y, z)`; absent poses are skipped.
    pub pose: Option<(f32, f32, f32)>,
    /// Local voxel-map points in world coordinates.
    pub points: Vec<(f32, f32, f32)>,
    /// Cylinder replaced by this local observation.
    pub region_bounds: RegionBounds,
}

/// Timing information for one transformation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TransformTimings {
    /// Region update duration in milliseconds.
    pub update_ms: f64,
    /// Planning duration in milliseconds.
    pub plan_ms: f64,
    /// Combined duration in milliseconds.
    pub total_ms: f64,
}

/// A graph edge expanded into a world-space line segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DebugEdge {
    /// Segment start.
    pub from: (f32, f32, f32),
    /// Segment end.
    pub to: (f32, f32, f32),
    /// Cost of the node edge that owns this segment.
    pub cost: f32,
}

/// Planner artifacts emitted with every transformed observation.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlannerArtifacts {
    /// Occupied voxel centers.
    pub voxel_map: Vec<(f32, f32, f32)>,
    /// Standable surface cells and wall clearance.
    pub surface_clearance: Vec<((f32, f32, f32), f32)>,
    /// Sparse graph nodes.
    pub nodes: Vec<(f32, f32, f32)>,
    /// Expanded node-edge geometry.
    pub node_edges: Vec<DebugEdge>,
}

/// Output corresponding to one posed input observation.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanObservation {
    /// Input observation identifier.
    pub id: u64,
    /// Input timestamp in seconds.
    pub timestamp: f64,
    /// Start point after dropping the sensor pose by robot height.
    pub start: (f32, f32, f32),
    /// World-frame path; empty when no route was found.
    pub path: Vec<(f32, f32, f32)>,
    /// Whether a complete route was found.
    pub planned: bool,
    /// Number of occupied voxels after the update.
    pub voxels: usize,
    /// Per-stage timings.
    pub timings: TransformTimings,
    /// Debug and evaluator artifacts.
    pub artifacts: PlannerArtifacts,
}

/// Stateful equivalent of Dimos `MLSPlan` over the Rust planner core.
pub struct MlsPlanTransformer {
    planner: Planner,
    config: Config,
    goal: (f32, f32, f32),
}

/// Input validation failure at the stateful transformer boundary.
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum TransformError {
    /// Planner configuration failed structural or finite-value validation.
    #[error("invalid planner configuration: {0}")]
    InvalidConfig(String),
    /// The fixed planner goal contains a non-finite coordinate.
    #[error("goal must contain only finite values")]
    InvalidGoal,
    /// The observation timestamp is NaN or infinite.
    #[error("observation timestamp must be finite")]
    InvalidTimestamp,
    /// The observation pose contains a non-finite coordinate.
    #[error("observation pose must contain only finite values")]
    InvalidPose,
    /// An observation map point contains a non-finite coordinate.
    #[error("observation point at index {index} must contain only finite values")]
    InvalidPoint {
        /// Zero-based point index.
        index: usize,
    },
    /// Region values are non-finite, negative-radius, or vertically reversed.
    #[error("observation region bounds are invalid")]
    InvalidRegion,
}

impl MlsPlanTransformer {
    /// Construct a transformer with a fixed goal.
    #[must_use]
    pub fn new(config: Config, goal: (f32, f32, f32)) -> Self {
        Self::try_new(config, goal).expect("invalid input passed to MlsPlanTransformer::new")
    }

    /// Construct a transformer after validating configuration and goal.
    pub fn try_new(config: Config, goal: (f32, f32, f32)) -> Result<Self, TransformError> {
        validate_config(&config)?;
        validate_point(goal).ok_or(TransformError::InvalidGoal)?;
        Ok(Self {
            planner: Planner::default(),
            config,
            goal,
        })
    }

    /// Replace the active fixed goal.
    pub fn set_goal(&mut self, goal: (f32, f32, f32)) {
        self.try_set_goal(goal)
            .expect("invalid input passed to MlsPlanTransformer::set_goal");
    }

    /// Replace the active goal after finite-value validation.
    pub fn try_set_goal(&mut self, goal: (f32, f32, f32)) -> Result<(), TransformError> {
        validate_point(goal).ok_or(TransformError::InvalidGoal)?;
        self.goal = goal;
        Ok(())
    }

    /// Transform one observation. Poseless observations are skipped.
    #[must_use]
    pub fn transform(&mut self, observation: MapObservation) -> Option<PlanObservation> {
        self.try_transform(observation)
            .expect("invalid input passed to MlsPlanTransformer::transform")
    }

    /// Validate and transform one observation without mutating state on error.
    pub fn try_transform(
        &mut self,
        observation: MapObservation,
    ) -> Result<Option<PlanObservation>, TransformError> {
        validate_observation(&observation)?;
        let Some((x, y, z)) = observation.pose else {
            return Ok(None);
        };
        let start = (x, y, z - self.config.robot_height);

        let update_start = Instant::now();
        self.planner.update_region(
            &observation.points,
            &observation.region_bounds,
            &self.config,
        );
        let update_done = Instant::now();
        let path = self
            .planner
            .plan(start, self.goal, &self.config)
            .unwrap_or_default();
        let plan_done = Instant::now();
        let planned = !path.is_empty();
        let artifacts = planner_artifacts(&self.planner, self.config.voxel_size);

        Ok(Some(PlanObservation {
            id: observation.id,
            timestamp: observation.timestamp,
            start,
            path,
            planned,
            voxels: self.planner.voxel_count(),
            timings: TransformTimings {
                update_ms: (update_done - update_start).as_secs_f64() * 1_000.0,
                plan_ms: (plan_done - update_done).as_secs_f64() * 1_000.0,
                total_ms: (plan_done - update_start).as_secs_f64() * 1_000.0,
            },
            artifacts,
        }))
    }

    /// Inspect the accumulated planner state.
    #[must_use]
    pub fn planner(&self) -> &Planner {
        &self.planner
    }
}

fn validate_config(config: &Config) -> Result<(), TransformError> {
    config
        .validate()
        .map_err(|error| TransformError::InvalidConfig(error.to_string()))?;
    let values = [
        config.voxel_size,
        config.robot_height,
        config.surface_closing_radius,
        config.node_spacing_m,
        config.wall_clearance_m,
        config.wall_buffer_m,
        config.wall_buffer_weight,
        config.step_threshold_m,
        config.step_penalty_weight,
        config.goal_tolerance,
        config.viz_publish_hz,
    ];
    if config.world_frame.trim().is_empty() || !values.into_iter().all(f32::is_finite) {
        return Err(TransformError::InvalidConfig(
            "world_frame must be non-empty and numeric fields must be finite".into(),
        ));
    }
    Ok(())
}

fn validate_observation(observation: &MapObservation) -> Result<(), TransformError> {
    if !observation.timestamp.is_finite() {
        return Err(TransformError::InvalidTimestamp);
    }
    if observation
        .pose
        .is_some_and(|pose| validate_point(pose).is_none())
    {
        return Err(TransformError::InvalidPose);
    }
    for (index, &point) in observation.points.iter().enumerate() {
        if validate_point(point).is_none() {
            return Err(TransformError::InvalidPoint { index });
        }
    }
    let bounds = &observation.region_bounds;
    let finite = [
        bounds.origin_x,
        bounds.origin_y,
        bounds.radius,
        bounds.z_min,
        bounds.z_max,
    ]
    .into_iter()
    .all(f32::is_finite);
    if !finite || bounds.radius < 0.0 || bounds.z_min > bounds.z_max {
        return Err(TransformError::InvalidRegion);
    }
    Ok(())
}

fn validate_point(point: (f32, f32, f32)) -> Option<()> {
    [point.0, point.1, point.2]
        .into_iter()
        .all(f32::is_finite)
        .then_some(())
}

fn voxel_center((ix, iy, iz): VoxelKey, voxel_size: f32) -> (f32, f32, f32) {
    (
        (ix as f32 + 0.5) * voxel_size,
        (iy as f32 + 0.5) * voxel_size,
        (iz as f32 + 0.5) * voxel_size,
    )
}

fn planner_artifacts(planner: &Planner, voxel_size: f32) -> PlannerArtifacts {
    let graph = planner.graph();
    let voxel_map = planner
        .voxel_keys()
        .map(|key| voxel_center(key, voxel_size))
        .collect();
    let surface_clearance = planner
        .surface_clearance()
        .into_iter()
        .map(|(key, clearance)| {
            let (ix, iy, iz) = key;
            (surface_point_xyz(ix, iy, iz, voxel_size), clearance)
        })
        .collect();
    let nodes = graph.nodes.iter().map(|node| node.pos).collect();
    let node_edges = edges_to_segments(&graph.cells, &graph.cell_state, &graph.node_edges)
        .into_iter()
        .map(|(from, to, cost)| DebugEdge {
            from: surface_point_xyz(from.0, from.1, from.2, voxel_size),
            to: surface_point_xyz(to.0, to.1, to.2, voxel_size),
            cost,
        })
        .collect();
    PlannerArtifacts {
        voxel_map,
        surface_clearance,
        nodes,
        node_edges,
    }
}
