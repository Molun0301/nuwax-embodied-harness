//! Synchronous module boundary for the extended terrain-map processor.
//!
//! The runtime owns scheduling and transport. This adapter owns typed input
//! validation, lifecycle state, and exactly-once processing of each scan.

use crate::{
    NavError, Odometry, PointCloud, TerrainMapExtConfig, TerrainMapExtProcessor, TimedPointCloud,
    Timestamp,
};

/// Recommended lifecycle period (10 Hz).
pub const TICK_PERIOD_MS: u64 = 100;

/// A single lifecycle output.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleOutput {
    /// A processed persistent terrain-map snapshot.
    TerrainMap(PointCloud),
    /// No scan is waiting to be processed, or the processed scan was empty.
    Idle,
}

/// Typed module input.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleInput {
    /// Registered scan in the world frame.
    RegisteredScan(PointCloud),
    /// Robot odometry used to centre and roll the terrain grid.
    Odometry(Odometry),
    /// Fast local terrain used to fill the near-field output.
    LocalTerrain(PointCloud),
    /// Clear accumulated points inside the configured clearing distance.
    Clear,
}

/// Lifecycle and typed input contract.
pub trait Module: Send + Sync + 'static {
    /// Start processing. Repeated calls while running are idempotent.
    fn start(&mut self) -> Result<(), NavError>;
    /// Stop processing and discard pending work.
    fn stop(&mut self) -> Result<(), NavError>;
    /// Consume a registered scan.
    fn on_registered_scan(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Update robot odometry.
    fn on_odometry(&mut self, odometry: Odometry) -> Result<(), NavError>;
    /// Update the local near-field terrain snapshot.
    fn on_local_terrain(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Request a clearing pass on the next processed scan.
    fn request_clearing(&mut self) -> Result<(), NavError>;
    /// Process at most one pending scan.
    fn tick(&mut self) -> Result<ModuleOutput, NavError>;
}

/// Lifecycle wrapper around [`TerrainMapExtProcessor`].
pub struct TerrainMapExtModule {
    config: TerrainMapExtConfig,
    processor: TerrainMapExtProcessor,
    running: bool,
    scan_pending: bool,
    pending_stamp: Timestamp,
}

impl TerrainMapExtModule {
    /// Construct with the default terrain-map configuration.
    #[must_use]
    pub fn new() -> Self {
        Self::with_config(TerrainMapExtConfig::default())
    }

    /// Construct with an explicit terrain-map configuration.
    #[must_use]
    pub fn with_config(config: TerrainMapExtConfig) -> Self {
        Self {
            processor: TerrainMapExtProcessor::new(config.clone()),
            config,
            running: false,
            scan_pending: false,
            pending_stamp: Timestamp::default(),
        }
    }

    /// Whether the module is accepting input and ticks.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Dispatch a typed input through the same validated handlers used by a runtime.
    pub fn handle_input(&mut self, input: ModuleInput) -> Result<(), NavError> {
        match input {
            ModuleInput::RegisteredScan(cloud) => self.on_registered_scan(cloud),
            ModuleInput::Odometry(odometry) => self.on_odometry(odometry),
            ModuleInput::LocalTerrain(cloud) => self.on_local_terrain(cloud),
            ModuleInput::Clear => self.request_clearing(),
        }
    }

    fn ensure_running(&self) -> Result<(), NavError> {
        if self.running {
            Ok(())
        } else {
            Err(NavError::InvalidInput {
                field: "module",
                reason: "terrain-map module is not running".into(),
            })
        }
    }
}

impl Default for TerrainMapExtModule {
    fn default() -> Self {
        Self::new()
    }
}

impl Module for TerrainMapExtModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        self.processor = TerrainMapExtProcessor::new(self.config.clone());
        self.scan_pending = false;
        self.pending_stamp = Timestamp::default();
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        self.running = false;
        self.scan_pending = false;
        Ok(())
    }

    fn on_registered_scan(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "registered_scan")?;
        if cloud.is_empty() {
            return Ok(());
        }

        let seconds = cloud.stamp.nanos as f64 / 1_000_000_000.0;
        let times = vec![seconds; cloud.points.len()];
        self.processor.ingest_registered_scan(TimedPointCloud {
            points: &cloud.points,
            times: &times,
        })?;
        self.pending_stamp = cloud.stamp;
        self.scan_pending = true;
        Ok(())
    }

    fn on_odometry(&mut self, odometry: Odometry) -> Result<(), NavError> {
        self.ensure_running()?;
        if !odometry.is_finite() {
            return Err(NavError::InvalidInput {
                field: "odometry",
                reason: "pose or twist contains a non-finite value".into(),
            });
        }
        self.processor.update_odometry(odometry.pose);
        Ok(())
    }

    fn on_local_terrain(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "terrain_map")?;
        self.processor.update_local_terrain(&cloud.points);
        Ok(())
    }

    fn request_clearing(&mut self) -> Result<(), NavError> {
        self.ensure_running()?;
        self.processor.request_clearing();
        Ok(())
    }

    fn tick(&mut self) -> Result<ModuleOutput, NavError> {
        self.ensure_running()?;
        if !self.scan_pending {
            return Ok(ModuleOutput::Idle);
        }

        let output = self.processor.process_tick(self.pending_stamp)?;
        self.scan_pending = false;
        Ok(output.map_or(ModuleOutput::Idle, ModuleOutput::TerrainMap))
    }
}

fn validate_cloud(cloud: &PointCloud, field: &'static str) -> Result<(), NavError> {
    cloud.validate().map_err(|reason| NavError::InvalidInput {
        field,
        reason: reason.into(),
    })?;
    if cloud
        .points
        .iter()
        .any(|point| !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite())
    {
        return Err(NavError::InvalidInput {
            field,
            reason: "point cloud contains a non-finite coordinate".into(),
        });
    }
    Ok(())
}
