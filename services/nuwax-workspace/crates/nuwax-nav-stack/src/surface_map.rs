//! 导航与探索共用的三维物理表面更新和禁区派生缓存。

use nuwax_codec::lcm::schema;
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBuildKind, SurfaceBuildStats, SurfaceMapBuilder, SurfaceSnapshot,
};
use std::sync::Arc;

/// Publication identities are assigned by the owner, never derived from payload bytes.
#[derive(Clone, PartialEq, Eq)]
enum GeometryRevision {
    Fine {
        domain: nuwax_map_chunks::Domain,
        resolution: u32,
        geometry_revision: u64,
    },
    FullCloud {
        domain: nuwax_map_chunks::Domain,
        resolution: u32,
        publication_revision: i64,
    },
}

/// No independent virtual-mask revision exists yet. Occupancy commits invalidate it
/// conservatively, including commits whose actual mask did not change.
#[derive(Clone, PartialEq, Eq)]
struct MaskRevision {
    domain: nuwax_map_chunks::Domain,
    occupancy_revision: i64,
}

struct DerivedSurface {
    mask_revision: Option<MaskRevision>,
    physical: Arc<SurfaceSnapshot>,
    snapshot: Arc<SurfaceSnapshot>,
}

/// 仅缓存几何计算；调用者仍须核对消息来源、修订、接收时刻与更新票据。
pub struct SurfaceMapCache {
    config: Config,
    builder: SurfaceMapBuilder,
    physical: Option<(GeometryRevision, Arc<SurfaceSnapshot>)>,
    latest: Option<DerivedSurface>,
    last_stats: Option<SurfaceBuildStats>,
    fine: Option<crate::surface_chunks::FineMapReader>,
    lcm_url: String,
    fine_built_root: Option<Arc<nuwax_map_chunks::Manifest>>,
    observation_enabled: bool,
    fine_observed_root: Option<Arc<nuwax_map_chunks::Manifest>>,
    observations: Option<Arc<nuwax_nav_mls_planner::SurfaceObservations>>,
}

impl SurfaceMapCache {
    /// 捕获同一拥有方的固定几何参数。
    pub fn new(config: Config) -> Self {
        Self {
            builder: SurfaceMapBuilder::new(config.clone()),
            config,
            physical: None,
            latest: None,
            last_stats: None,
            fine: None,
            fine_built_root: None,
            observation_enabled: false,
            fine_observed_root: None,
            observations: None,
            lcm_url: std::env::var("NUWAX_LCM_URL")
                .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=0".into()),
        }
    }

    /// 使用启动时捕获的同一完整模型构建消费者缓存。
    pub fn configured(config: &crate::surface_config::SurfaceNavigationConfig) -> Self {
        let mut cache = Self::new(config.geometry.clone());
        cache.observation_enabled = config.body_envelope.is_some()
            || config.follower.max_observation_age_sec.is_some();
        cache.builder =
            SurfaceMapBuilder::configured(config.geometry.clone(), config.body_envelope.clone());
        cache
    }

    /// 撤销计算复用状态；不会改动已交付快照或发布任何业务状态。
    pub fn reset(&mut self) {
        self.builder.reset();
        self.physical = None;
        self.latest = None;
        self.last_stats = None;
        self.fine = None;
        self.fine_built_root = None;
        self.fine_observed_root = None;
        self.observations = None;
    }

    /// 最近一次物理表面工作量；禁区派生工作单独于此统计。
    pub fn last_build_stats(&self) -> Option<SurfaceBuildStats> {
        self.last_stats
    }

    #[cfg(test)]
    pub(crate) fn physical_snapshot(&self) -> Option<Arc<SurfaceSnapshot>> {
        self.physical
            .as_ref()
            .map(|(_, snapshot)| Arc::clone(snapshot))
    }

    fn record_reuse(&mut self) {
        if let Some(stats) = self.last_stats.as_mut() {
            stats.kind = SurfaceBuildKind::Reused;
            stats.input_points = 0;
            stats.changed_columns = 0;
            stats.changed_surface_cells = 0;
            stats.checked_cells = 0;
            stats.global_cost_refresh = false;
        }
    }

    /// 适配已准入的完整消息；使用拥有方修订复用，否则按体素列变化构建。
    pub fn build(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<Arc<SurfaceSnapshot>, String> {
        schema::validate_navigation_map_3d(message)?;
        if message.cleared {
            self.reset();
            return Err("surface map cleared by mapping owner".into());
        }
        if message.committed_map.frame_id != self.config.world_frame {
            return Err("surface map frame differs from configured map frame".into());
        }
        let fine_root = if message.fine.is_some() {
            if self.fine.is_none() {
                self.fine = Some(crate::surface_chunks::FineMapReader::new(
                    self.lcm_url.clone(),
                )?);
            }
            Some(self.fine.as_mut().unwrap().receive(message)?)
        } else {
            None
        };
        let observations = if self.observation_enabled {
            match fine_root.as_ref() {
                Some(root) => Some(self.fine.as_mut().unwrap().observations(root,
                    self.fine_observed_root.as_deref().zip(self.observations.as_ref()))?),
                None => None,
            }
        } else { None };
        let domain = schema::navigation_map_3d_domain(message);
        let physical_revision = match fine_root.as_ref() {
            Some(root) => GeometryRevision::Fine {
                domain: root.domain.clone(),
                resolution: root.voxel_size_m.to_bits(),
                geometry_revision: root.geometry_revision,
            },
            None => GeometryRevision::FullCloud {
                domain: domain.clone(),
                resolution: message.source_voxel_size.to_bits(),
                publication_revision: message.cloud_revision,
            },
        };
        let mask_revision = message.virtual_obstacles.as_ref().map(|_| MaskRevision {
            domain,
            occupancy_revision: message.committed_map.revision,
        });
        let mut frontier_changes = None;
        let physical = if let Some((_, snapshot)) = self
            .physical
            .as_ref()
            .filter(|(previous, _)| *previous == physical_revision)
        {
            let snapshot = Arc::clone(snapshot);
            self.record_reuse();
            snapshot
        } else {
            let result = if let Some(root) = &fine_root {
                let previous = self.fine_built_root.as_deref().filter(|old| {
                    old.domain == root.domain
                        && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits()
                });
                let (updates, read_points) =
                    self.fine
                        .as_mut()
                        .unwrap()
                        .columns(root, previous, self.config.voxel_size)?;
                self.builder
                    .build_column_updates(
                        &updates,
                        root.voxel_size_m,
                        previous.is_none(),
                        read_points,
                    )
                    .map_err(|error| error.to_string())?
            } else {
                let points =
                    schema::navigation_map_3d_points(message).map_err(|error| error.to_string())?;
                self.builder
                    .build(&points, message.source_voxel_size)
                    .map_err(|error| error.to_string())?
            };
            self.fine_built_root = fine_root.clone();
            if result.stats.kind != SurfaceBuildKind::Full {
                frontier_changes = Some(result.changed_columns.clone());
            }
            self.last_stats = Some(result.stats);
            self.physical = Some((physical_revision, Arc::clone(&result.snapshot)));
            result.snapshot
        };
        // An evidence-only fine publication advances the retained root without
        // rebuilding geometry. Full-cloud compatibility goes through the builder's
        // existing exact column comparison on each new publication revision.
        self.fine_built_root = fine_root.clone();
        if let Some(previous) = self.latest.as_ref().filter(|previous|
            Arc::ptr_eq(&previous.physical, &physical) && previous.mask_revision == mask_revision)
        {
            let snapshot = &previous.snapshot;
            let same_observations = match (snapshot.observations(), observations.as_ref()) {
                (None, None) => true,
                (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                _ => false,
            };
            let snapshot = if same_observations { Arc::clone(snapshot) }
                else { Arc::new(snapshot.with_observations(observations.clone())) };
            self.observations = observations;
            self.fine_observed_root = fine_root;
            self.latest = Some(DerivedSurface { mask_revision, physical, snapshot: Arc::clone(&snapshot) });
            return Ok(snapshot);
        }
        let snapshot = if let Some(mask) = message.virtual_obstacles.as_ref() {
            let blocked: Vec<_> = mask.data.iter().map(|value| *value == 100).collect();
            Arc::new(
                physical
                    .with_virtual_obstacles(
                        [mask.info.origin.position[0], mask.info.origin.position[1]],
                        f64::from(mask.info.resolution),
                        mask.info.width as u32,
                        mask.info.height as u32,
                        &blocked,
                    )
                    .map_err(|error| error.to_string())?,
            )
        } else {
            Arc::clone(&physical)
        };
        if let (Some(changes), Some(previous)) = (frontier_changes, self.latest.as_ref()) {
            snapshot.inherit_frontier_cache(&previous.snapshot, &changes);
        }
        let snapshot = if self.observation_enabled {
            Arc::new(snapshot.with_observations(observations.clone()))
        } else { snapshot };
        self.observations = observations;
        self.fine_observed_root = fine_root;
        self.latest = Some(DerivedSurface { mask_revision, physical, snapshot: Arc::clone(&snapshot) });
        Ok(snapshot)
    }
}
