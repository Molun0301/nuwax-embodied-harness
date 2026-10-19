//! 导航与探索共用的三维物理表面更新和禁区派生缓存。

use nuwax_codec::lcm::schema;
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBuildKind, SurfaceBuildStats, SurfaceMapBuilder, SurfaceSnapshot,
};
use std::sync::Arc;

pub(crate) use nuwax_nav_runtime::surface_changes::SurfaceGeometryChanges;

/// An unavailable candidate is not a revocation of the last complete map.
/// Only missing immutable-object transport/deadline failures are retryable;
/// malformed roots, objects, replies and geometry remain authoritative failures.
#[derive(Debug, thiserror::Error)]
pub enum SurfaceMapBuildError {
    #[error("{0}")]
    IncompleteFineSource(MissingFineObject),
    #[error("{0}")]
    Invalid(String),
}

/// The original fetch closure knows the exact absent immutable object. Keep its
/// reason for diagnostics; never recover identity by parsing that string.
#[derive(Debug, thiserror::Error)]
#[error("{reason}")]
pub struct MissingFineObject {
    pub object: nuwax_map_chunks::ObjectIdentity,
    pub key: nuwax_map_chunks::Key,
    pub reason: String,
}

impl From<String> for SurfaceMapBuildError {
    fn from(reason: String) -> Self { Self::Invalid(reason) }
}
impl From<&str> for SurfaceMapBuildError {
    fn from(reason: &str) -> Self { Self::Invalid(reason.into()) }
}

pub(crate) struct SurfaceMapUpdate {
    pub snapshot: Arc<SurfaceSnapshot>,
    pub change_basis: Option<Arc<SurfaceSnapshot>>,
    pub changes: Arc<SurfaceGeometryChanges>,
}

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

/// 修订先走常量时间快路；同域较新提交可再逐格证明虚拟几何完全未变。
#[derive(Clone, PartialEq, Eq)]
struct MaskRevision {
    domain: nuwax_map_chunks::Domain,
    commit_boot_nonce: [u8; 16],
    occupancy_revision: i64,
}

#[derive(Clone, PartialEq, Eq)]
struct MaskLayout {
    resolution: u32,
    width: i32,
    height: i32,
    position: [u64; 3],
    orientation: [u64; 4],
}
impl MaskLayout {
    fn new(mask: &schema::OccupancyGrid) -> Self {
        Self {
            resolution: mask.info.resolution.to_bits(),
            width: mask.info.width,
            height: mask.info.height,
            position: mask.info.origin.position.map(f64::to_bits),
            orientation: mask.info.origin.orientation_xyzw.map(f64::to_bits),
        }
    }
}

/// One current failed dependency, owned beside the original retry deadline.
/// This is not a negative-cache table. The source Arc is already installed;
/// Only fixed-size mask layout and the domain (session <= 1024 bytes) are
/// copied once per real failure; no mask/cloud/block payloads are copied.
pub struct SurfaceMapRetryDependency {
    object: nuwax_map_chunks::ObjectIdentity,
    key: nuwax_map_chunks::Key,
    domain: nuwax_map_chunks::Domain,
    commit_boot_nonce: [u8; 16],
    source: Option<Arc<SurfaceSnapshot>>,
    mask: Option<MaskLayout>,
}

impl SurfaceMapRetryDependency {
    /// `source`, when present, must have passed the owner's original installed
    /// Arc/ticket and incomplete-mask retention checks for this failed message.
    /// None is for Nav's already-revoked/unavailable input, never permission to
    /// retain an executable map. F1 always supplies its proven installed Arc.
    pub fn new(
        object: nuwax_map_chunks::ObjectIdentity,
        key: nuwax_map_chunks::Key,
        root: &nuwax_map_chunks::Manifest,
        message: &schema::NavigationMap3D,
        source: Option<Arc<SurfaceSnapshot>>,
    ) -> Self {
        Self { object, key, domain: root.domain.clone(), commit_boot_nonce: message.commit_boot_nonce,
            source, mask: message.virtual_obstacles.as_ref().map(MaskLayout::new) }
    }

    pub fn retained_source(&self) -> Option<&Arc<SurfaceSnapshot>> { self.source.as_ref() }

    /// Caller first proves the retained Arc is still actually installed under
    /// its original owner fence. Supplied bytes only permit an early attempt;
    /// the unchanged build/Receiver path still validates every byte and binding.
    pub fn still_blocked(
        &self,
        message: &schema::NavigationMap3D,
        root: Option<&nuwax_map_chunks::Manifest>,
    ) -> bool {
        let (Some(root), Some(fine)) = (root, message.fine.as_ref()) else { return false; };
        if message.cleared || self.domain != root.domain
            || self.commit_boot_nonce != message.commit_boot_nonce
            || fine.included.contains_key(&self.object) {
            return false;
        }
        // Admission already proved strict key order. Watermark/same-key roots
        // need only binary search. A miss/replacement is not proof of absence:
        // the same immutable object can still be referenced at another key.
        let same_key = root.blocks.binary_search_by_key(&self.key, |block| block.key)
            .ok().is_some_and(|index| root.blocks[index].object == self.object);
        if !same_key && !root.blocks.iter().any(|block| block.object == self.object) {
            return false;
        }
        let Some(source) = self.source.as_ref() else { return true; };
        match (&self.mask, &message.virtual_obstacles) {
            (None, None) => true,
            (Some(layout), Some(mask)) => *layout == MaskLayout::new(mask)
                && source.matches_virtual_obstacles(
                    [mask.info.origin.position[0], mask.info.origin.position[1]],
                    f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32,
                    mask.data.iter().map(|value| *value == 100)),
            _ => false,
        }
    }
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct MaskWork {
    pub compared_cells: usize,
    pub derivations: usize,
}

struct DerivedSurface {
    mask_revision: Option<MaskRevision>,
    mask_layout: Option<MaskLayout>,
    physical: Arc<SurfaceSnapshot>,
    snapshot: Arc<SurfaceSnapshot>,
}

impl DerivedSurface {
    // Both paths borrow the original mask and retained obstacle storage. No
    // second mask vector or full publication is retained to prove equality.
    fn matches_mask_contents(
        &self,
        mask: &schema::OccupancyGrid,
        mut observed_cell: impl FnMut(),
    ) -> bool {
        self.snapshot.matches_virtual_obstacles(
            [mask.info.origin.position[0], mask.info.origin.position[1]],
            f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32,
            mask.data.iter().map(|value| { observed_cell(); *value == 100 }),
        )
    }
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
    #[cfg(test)]
    mask_work: MaskWork,
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
            #[cfg(test)]
            mask_work: MaskWork::default(),
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

    /// Opt-in integration seam for a receiver populated through its original
    /// bounded transfer / missing-object fetch path. Build still revalidates the
    /// publication and runs the original Reader, observations and surface builder.
    /// This does not exercise outer RPC transport or replace any geometry source.
    #[cfg(feature = "surface-source-qualification")]
    pub fn configured_with_admitted_fine_source(
        config: &crate::surface_config::SurfaceNavigationConfig,
        receiver: nuwax_map_chunks::Receiver,
    ) -> Self {
        let mut cache = Self::configured(config);
        cache.fine = Some(crate::surface_chunks::FineMapReader::from_admitted_receiver(receiver));
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

    #[cfg(test)]
    pub(crate) fn mask_work(&self) -> MaskWork { self.mask_work }

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
        self.build_typed(message).map_err(|error| error.to_string())
    }

    /// Preserve the existing incomplete-versus-invalid classification for owners
    /// that can keep an already installed complete source while fetching.
    pub fn build_typed(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<Arc<SurfaceSnapshot>, SurfaceMapBuildError> {
        self.build_with_changes(message).map(|update| update.snapshot)
    }

    /// Call only for IncompleteFineSource from build_typed(message). The message
    /// has passed the original schema admission; no partial root was committed.
    /// This proves only cache/source correspondence and mask geometry. The
    /// caller must still prove this Arc is its actually installed source and
    /// apply its original ticket/domain/clear fences without renewing clocks.
    pub fn retained_source_for_incomplete(
        &self,
        message: &schema::NavigationMap3D,
    ) -> Option<Arc<SurfaceSnapshot>> {
        if message.cleared || message.fine.is_none() { return None; }
        let previous = self.latest.as_ref()?;
        let (revision, physical) = self.physical.as_ref()?;
        if !Arc::ptr_eq(&previous.physical, physical) { return None; }
        let GeometryRevision::Fine { domain, .. } = revision else { return None; };
        if *domain != schema::navigation_map_3d_domain(message) { return None; }
        let same_mask = match (&previous.mask_revision, &message.virtual_obstacles) {
            (None, None) => true,
            (Some(old), Some(mask)) => old.domain == *domain
                && old.commit_boot_nonce == message.commit_boot_nonce
                // Incomplete retention uses actual borrowed contents, including
                // same-receipt input; no new trust in an identity-only shortcut.
                && previous.mask_layout.as_ref() == Some(&MaskLayout::new(mask))
                && previous.matches_mask_contents(mask, || {}),
            _ => false,
        };
        same_mask.then(|| Arc::clone(&previous.snapshot))
    }

    pub(crate) fn build_with_changes(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<SurfaceMapUpdate, SurfaceMapBuildError> {
        // A failed mask derivation may have advanced physical/builder state. Its
        // next local delta cannot be attached to the previous successful source.
        let change_basis = self.latest.as_ref().map(|old| Arc::clone(&old.snapshot));
        let intact_basis = self.latest.as_ref().zip(self.physical.as_ref())
            .is_some_and(|(old, (_, physical))| Arc::ptr_eq(&old.physical, physical));
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
        let (observations, fine_changes) = if self.observation_enabled {
            match fine_root.as_ref() {
                Some(root) => {
                    let (observations, changes) = self.fine.as_mut().unwrap().observations_with_changes(root,
                        self.fine_observed_root.as_deref().zip(self.observations.as_ref()))?;
                    (Some(observations), changes)
                }
                None => (None, self.observations.is_none().then(Vec::new)),
            }
        } else { (None, Some(Vec::new())) };
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
            commit_boot_nonce: message.commit_boot_nonce,
            occupancy_revision: message.committed_map.revision,
        });
        let mask_layout = message.virtual_obstacles.as_ref().map(MaskLayout::new);
        let mut frontier_changes = None;
        let mut coarse_changes = Some(Vec::new());
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
                coarse_changes = Some(result.changed_columns);
            } else { coarse_changes = None; }
            self.last_stats = Some(result.stats);
            self.physical = Some((physical_revision, Arc::clone(&result.snapshot)));
            result.snapshot
        };
        // An evidence-only fine publication advances the retained root without
        // rebuilding geometry. Full-cloud compatibility goes through the builder's
        // existing exact column comparison on each new publication revision.
        self.fine_built_root = fine_root.clone();
        #[cfg(test)]
        let mut compared_cells = 0;
        let same_mask = self.latest.as_ref().is_some_and(|previous| {
            if previous.mask_revision == mask_revision { return true; }
            let (Some(old), Some(next), Some(mask)) =
                (&previous.mask_revision, &mask_revision, &message.virtual_obstacles)
                else { return false; };
            if old.domain != next.domain || old.commit_boot_nonce != next.commit_boot_nonce
                || old.occupancy_revision >= next.occupancy_revision
                || previous.mask_layout != mask_layout
            { return false; }
            // 比较借用完整输入与已有积分图，不保留或复制另一份遮罩。
            previous.matches_mask_contents(mask, || {
                #[cfg(test)] { compared_cells += 1; }
            })
        });
        #[cfg(test)] { self.mask_work.compared_cells += compared_cells; }
        let reusable = same_mask && self.latest.as_ref()
            .is_some_and(|old| Arc::ptr_eq(&old.physical, &physical));
        let changes = |snapshot: &Arc<SurfaceSnapshot>| {
            if !intact_basis { return SurfaceGeometryChanges::Unknown; }
            if change_basis.as_ref().is_some_and(|old| nuwax_nav_pct::same_source_geometry(old, snapshot)) {
                return SurfaceGeometryChanges::Empty;
            }
            if !same_mask || change_basis.as_ref().is_none_or(|old|
                old.source_voxel_size_m().to_bits() != snapshot.source_voxel_size_m().to_bits()
                    || old.ground_evidence() != snapshot.ground_evidence()) {
                return SurfaceGeometryChanges::Unknown;
            }
            match (fine_changes, coarse_changes) {
                (Some(fine), Some(coarse)) if fine.is_empty() && coarse.is_empty() => SurfaceGeometryChanges::Empty,
                (Some(fine), Some(coarse)) => SurfaceGeometryChanges::Columns {
                    fine_size: snapshot.source_voxel_size_m(), fine,
                    planner_size: snapshot.config().voxel_size, coarse,
                },
                _ => SurfaceGeometryChanges::Unknown,
            }
        };
        if reusable {
            let previous = self.latest.as_ref().expect("已核对保留派生图");
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
            self.latest = Some(DerivedSurface { mask_revision, mask_layout, physical, snapshot: Arc::clone(&snapshot) });
            let changes = Arc::new(changes(&snapshot));
            return Ok(SurfaceMapUpdate { snapshot, change_basis, changes });
        }
        let snapshot = if let Some(mask) = message.virtual_obstacles.as_ref() {
            #[cfg(test)] { self.mask_work.derivations += 1; }
            // 仅较新修订的逐格证明允许复用遮罩存储；同修订快路不证明输入内容。
            let retained = self.latest.as_ref()
                .filter(|previous| same_mask && previous.mask_revision != mask_revision)
                .and_then(|previous| physical.with_retained_virtual_obstacles_from(&previous.snapshot));
            if let Some(result) = retained {
                Arc::new(result.map_err(|error| error.to_string())?)
            } else {
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
            }
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
        self.latest = Some(DerivedSurface { mask_revision, mask_layout, physical, snapshot: Arc::clone(&snapshot) });
        let changes = Arc::new(changes(&snapshot));
        Ok(SurfaceMapUpdate { snapshot, change_basis, changes })
    }
}
