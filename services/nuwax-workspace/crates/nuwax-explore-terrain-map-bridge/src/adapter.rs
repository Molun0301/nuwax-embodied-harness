//! Executable exploration adapter over the unique terrain-map algorithm owner.

use nuwax_nav_terrain_map_ext::module::{Module, ModuleInput, ModuleOutput, TerrainMapExtModule};
use nuwax_nav_terrain_map_ext::{NavError, Odometry, PointCloud};

use crate::ExplorationTerrainMapExtProfile;

/// Exploration lifecycle wrapper that delegates all processing to
/// `nuwax-nav-terrain-map-ext`.
pub struct ExplorationTerrainMapAdapter {
    profile: ExplorationTerrainMapExtProfile,
    owner: TerrainMapExtModule,
}

impl ExplorationTerrainMapAdapter {
    /// Construct a stopped adapter from an exploration profile.
    #[must_use]
    pub fn new(profile: ExplorationTerrainMapExtProfile) -> Self {
        let owner = TerrainMapExtModule::with_config(profile.config.clone());
        Self { profile, owner }
    }

    /// Construct with the default exploration profile.
    #[must_use]
    pub fn default_exploration() -> Self {
        Self::new(ExplorationTerrainMapExtProfile::default_exploration())
    }

    /// Selected profile.
    #[must_use]
    pub fn profile(&self) -> &ExplorationTerrainMapExtProfile {
        &self.profile
    }

    /// Start processing. Repeated starts are idempotent.
    pub fn start(&mut self) -> Result<(), NavError> {
        self.owner.start()
    }

    /// Stop processing and discard pending work.
    pub fn stop(&mut self) -> Result<(), NavError> {
        self.owner.stop()
    }

    /// Whether the delegated owner is running.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.owner.is_running()
    }

    /// Update corrected odometry.
    pub fn on_odometry(&mut self, odometry: Odometry) -> Result<(), NavError> {
        self.owner.handle_input(ModuleInput::Odometry(odometry))
    }

    /// Update the fast near-field terrain snapshot.
    pub fn on_local_terrain(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.owner.handle_input(ModuleInput::LocalTerrain(cloud))
    }

    /// Queue one registered world-frame scan.
    pub fn on_registered_scan(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.owner.handle_input(ModuleInput::RegisteredScan(cloud))
    }

    /// Request near-field clearing on the next processed scan.
    pub fn request_clearing(&mut self) -> Result<(), NavError> {
        self.owner.handle_input(ModuleInput::Clear)
    }

    /// Process at most one queued scan.
    pub fn tick(&mut self) -> Result<ModuleOutput, NavError> {
        self.owner.tick()
    }
}

impl Default for ExplorationTerrainMapAdapter {
    fn default() -> Self {
        Self::default_exploration()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_terrain_map_ext::{Point3, Timestamp};

    fn cloud(second: i64, points: Vec<Point3>) -> PointCloud {
        PointCloud {
            stamp: Timestamp::from_unix(second, 0),
            points,
            ..PointCloud::default()
        }
    }

    #[test]
    fn adapter_runs_the_unique_owner_lifecycle_and_processing_pipeline() {
        let mut profile = ExplorationTerrainMapExtProfile::default_exploration();
        profile.config.check_terrain_connectivity = false;
        profile.config.voxel_point_update_threshold = 1;
        profile.config.voxel_time_update_threshold = 0.0;
        let mut adapter = ExplorationTerrainMapAdapter::new(profile);
        assert!(adapter.on_odometry(Odometry::default()).is_err());

        adapter.start().expect("start adapter");
        adapter.start().expect("idempotent start");
        adapter
            .on_odometry(Odometry::default())
            .expect("odometry input");
        adapter
            .on_local_terrain(cloud(1, vec![Point3::new(0.2, 0.0, -0.2)]))
            .expect("local terrain input");
        adapter
            .on_registered_scan(cloud(1, vec![Point3::new(1.0, 0.0, -0.2)]))
            .expect("registered scan input");

        match adapter.tick().expect("process tick") {
            ModuleOutput::TerrainMap(output) => {
                assert!(output.points.iter().any(|point| point.x == 1.0));
                assert!(output.points.iter().any(|point| point.x == 0.2));
            }
            ModuleOutput::Idle => panic!("queued scan unexpectedly produced no output"),
        }
        assert_eq!(adapter.tick().expect("idle tick"), ModuleOutput::Idle);
        adapter.stop().expect("stop adapter");
        assert!(!adapter.is_running());
    }
}
