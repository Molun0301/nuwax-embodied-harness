//! Runtime-facing relocalization acceptance, correction, and map-merge boundary.

use nalgebra::{Isometry3, Point3};
use thiserror::Error;

use super::{relocation_pipeline_with_config, RelocalizationConfig, RelocalizationResult};

/// Configuration for one relocalization runtime owner.
#[derive(Debug, Clone, Copy)]
pub struct RelocalizationRuntimeConfig {
    /// Minimum local-map point count before running global registration.
    pub min_local_points: usize,
    /// Runtime acceptance threshold applied after algorithm-level gates.
    pub fitness_threshold: f64,
    /// Multi-scale registration configuration.
    pub algorithm: RelocalizationConfig,
}

impl Default for RelocalizationRuntimeConfig {
    fn default() -> Self {
        Self {
            min_local_points: 50_000,
            fitness_threshold: 0.45,
            algorithm: RelocalizationConfig::default(),
        }
    }
}

/// Runtime disposition of one local-map observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelocalizationStatus {
    /// The local map did not meet the configured point gate.
    InsufficientPoints,
    /// Registration ran but failed convergence or fitness acceptance.
    Rejected,
    /// Registration was accepted and the correction owner was updated.
    Accepted,
}

/// Diagnostics and transforms emitted for one runtime attempt.
#[derive(Debug, Clone, Copy)]
pub struct RelocalizationUpdate {
    /// Runtime disposition.
    pub status: RelocalizationStatus,
    /// Transform placing local/world points into the loaded map.
    pub local_to_map: Option<Isometry3<f64>>,
    /// Inverse correction used to place the loaded map in local/world coordinates.
    pub world_to_map: Option<Isometry3<f64>>,
    /// Fine-scale accepted inlier ratio.
    pub fitness: f64,
    /// Largest feature correspondence set.
    pub feature_matches: usize,
    /// Largest RANSAC consensus.
    pub ransac_inliers: usize,
}

/// Relocalization runtime construction errors.
#[derive(Debug, Error, PartialEq)]
pub enum RelocalizationRuntimeError {
    /// The loaded global map is too small for feature registration.
    #[error("global map requires at least three points")]
    GlobalMapTooSmall,
    /// The loaded global map contains a non-finite coordinate.
    #[error("global map must contain only finite points")]
    NonFiniteGlobalMap,
    /// Runtime configuration is invalid.
    #[error("invalid relocalization runtime configuration: {0}")]
    InvalidConfig(&'static str),
}

/// Owns a loaded global map and the latest accepted world/map correction.
pub struct RelocalizationRuntime {
    config: RelocalizationRuntimeConfig,
    global_map: Vec<Point3<f64>>,
    world_to_map: Option<Isometry3<f64>>,
    attempts: u64,
    accepted: u64,
}

impl RelocalizationRuntime {
    /// Create a runtime owner for one loaded map.
    pub fn new(
        global_map: Vec<Point3<f64>>,
        config: RelocalizationRuntimeConfig,
    ) -> Result<Self, RelocalizationRuntimeError> {
        if global_map.len() < 3 {
            return Err(RelocalizationRuntimeError::GlobalMapTooSmall);
        }
        if global_map.iter().any(|point| !point_is_finite(point)) {
            return Err(RelocalizationRuntimeError::NonFiniteGlobalMap);
        }
        if config.min_local_points < 3 {
            return Err(RelocalizationRuntimeError::InvalidConfig(
                "min_local_points must be at least three",
            ));
        }
        if !config.fitness_threshold.is_finite() || !(0.0..=1.0).contains(&config.fitness_threshold)
        {
            return Err(RelocalizationRuntimeError::InvalidConfig(
                "fitness_threshold must be finite and within zero to one",
            ));
        }
        if !algorithm_config_is_valid(config.algorithm) {
            return Err(RelocalizationRuntimeError::InvalidConfig(
                "algorithm configuration contains an invalid count, distance, or tolerance",
            ));
        }
        Ok(Self {
            config,
            global_map,
            world_to_map: None,
            attempts: 0,
            accepted: 0,
        })
    }

    /// Run multi-scale feature registration and update the correction on acceptance.
    #[must_use]
    pub fn process_local_map(&mut self, local_map: &[Point3<f64>]) -> RelocalizationUpdate {
        if local_map.iter().any(|point| !point_is_finite(point)) {
            return RelocalizationUpdate {
                status: RelocalizationStatus::Rejected,
                local_to_map: None,
                world_to_map: self.world_to_map,
                fitness: 0.0,
                feature_matches: 0,
                ransac_inliers: 0,
            };
        }
        if local_map.len() < self.config.min_local_points {
            return RelocalizationUpdate {
                status: RelocalizationStatus::InsufficientPoints,
                local_to_map: None,
                world_to_map: self.world_to_map,
                fitness: 0.0,
                feature_matches: 0,
                ransac_inliers: 0,
            };
        }
        self.attempts = self.attempts.saturating_add(1);
        let result =
            relocation_pipeline_with_config(local_map, &self.global_map, self.config.algorithm);
        self.accept_result(result)
    }

    /// Latest accepted loaded-map-to-local/world correction.
    #[must_use]
    pub fn world_to_map(&self) -> Option<Isometry3<f64>> {
        self.world_to_map
    }

    /// Number of registration attempts that passed the point-count gate.
    #[must_use]
    pub fn attempts(&self) -> u64 {
        self.attempts
    }

    /// Number of accepted corrections.
    #[must_use]
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// Transform the loaded global map into local/world coordinates.
    #[must_use]
    pub fn corrected_global_map(&self) -> Option<Vec<Point3<f64>>> {
        let correction = self.world_to_map?;
        Some(
            self.global_map
                .iter()
                .map(|point| correction * point)
                .collect(),
        )
    }

    /// Merge the local map with the corrected loaded map.
    #[must_use]
    pub fn merged_map(&self, local_map: &[Point3<f64>]) -> Option<Vec<Point3<f64>>> {
        let corrected = self.corrected_global_map()?;
        let mut merged = Vec::with_capacity(local_map.len() + corrected.len());
        merged.extend_from_slice(local_map);
        merged.extend(corrected);
        Some(merged)
    }

    /// Clear the accepted correction and counters without unloading the map.
    pub fn reset(&mut self) {
        self.world_to_map = None;
        self.attempts = 0;
        self.accepted = 0;
    }

    fn accept_result(&mut self, result: RelocalizationResult) -> RelocalizationUpdate {
        let accepted = result.registration.converged
            && result.fitness.is_finite()
            && result.fitness >= self.config.fitness_threshold
            && result.registration.mse.is_finite()
            && pose_is_finite(&result.registration.transformation);
        if !accepted {
            return RelocalizationUpdate {
                status: RelocalizationStatus::Rejected,
                local_to_map: None,
                world_to_map: self.world_to_map,
                fitness: result.fitness,
                feature_matches: result.feature_matches,
                ransac_inliers: result.ransac_inliers,
            };
        }
        let local_to_map = result.registration.transformation;
        let world_to_map = local_to_map.inverse();
        if !pose_is_finite(&world_to_map) {
            return RelocalizationUpdate {
                status: RelocalizationStatus::Rejected,
                local_to_map: None,
                world_to_map: self.world_to_map,
                fitness: result.fitness,
                feature_matches: result.feature_matches,
                ransac_inliers: result.ransac_inliers,
            };
        }
        self.world_to_map = Some(world_to_map);
        self.accepted = self.accepted.saturating_add(1);
        RelocalizationUpdate {
            status: RelocalizationStatus::Accepted,
            local_to_map: Some(local_to_map),
            world_to_map: Some(world_to_map),
            fitness: result.fitness,
            feature_matches: result.feature_matches,
            ransac_inliers: result.ransac_inliers,
        }
    }
}

fn point_is_finite(point: &Point3<f64>) -> bool {
    point.coords.iter().all(|value| value.is_finite())
}

fn pose_is_finite(pose: &Isometry3<f64>) -> bool {
    pose.translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        && pose.rotation.coords.iter().all(|value| value.is_finite())
}

fn algorithm_config_is_valid(config: RelocalizationConfig) -> bool {
    config
        .voxel_sizes
        .iter()
        .all(|size| size.is_finite() && *size > 0.0)
        && config.max_feature_neighbors >= 3
        && config.ransac_iterations > 0
        && config.min_feature_matches >= 3
        && config.fine_voxel_size.is_finite()
        && config.fine_voxel_size > 0.0
        && config.fine_correspondence_distance.is_finite()
        && config.fine_correspondence_distance > 0.0
        && config.min_final_inliers >= 3
        && config.min_final_fitness.is_finite()
        && (0.0..=1.0).contains(&config.min_final_fitness)
        && config.icp.max_iterations > 0
        && config.icp.mse_tolerance.is_finite()
        && config.icp.mse_tolerance >= 0.0
        && config.icp.max_correspondence_distance.is_finite()
        && config.icp.max_correspondence_distance > 0.0
}
