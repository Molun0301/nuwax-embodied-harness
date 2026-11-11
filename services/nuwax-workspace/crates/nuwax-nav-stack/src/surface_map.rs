//! 导航与探索共用的三维物理表面更新和禁区派生缓存。

use nuwax_codec::lcm::schema;
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBuildKind, SurfaceBuildStats, SurfaceMapBuilder, SurfaceSnapshot,
};
use std::sync::Arc;

pub(crate) use nuwax_nav_runtime::surface_changes::SurfaceGeometryChanges;

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
    revision: i64,
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
    ) -> bool {
        self.snapshot.matches_virtual_obstacles(
            [mask.info.origin.position[0], mask.info.origin.position[1]],
            f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32,
            mask.data.iter().map(|value| *value == 100),
        )
    }
}

/// 缓存已在解码与接收边界准入的地图几何。
pub struct SurfaceMapCache {
    config: Config,
    builder: SurfaceMapBuilder,
    physical: Option<(GeometryRevision, Arc<SurfaceSnapshot>)>,
    latest: Option<DerivedSurface>,
    last_stats: Option<SurfaceBuildStats>,
    fine: Option<crate::surface_chunks::FineMapReader>,
    fine_built_root: Option<Arc<nuwax_map_chunks::Manifest>>,
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
            fine_observed_root: None,
            observations: None,
        }
    }

    /// 使用启动时捕获的同一完整模型构建消费者缓存。
    pub fn configured(config: &crate::surface_config::SurfaceNavigationConfig) -> Self {
        let mut cache = Self::new(config.geometry.clone());
        cache.builder =
            SurfaceMapBuilder::configured(config.geometry.clone(), Some(config.body_envelope.clone()));
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

    /// 适配已完成协议校验的完整消息；按拥有方修订与体素列变化构建。
    pub fn build(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<Arc<SurfaceSnapshot>, String> {
        self.build_with_changes(message).map(|update| update.snapshot)
    }

    pub(crate) fn build_with_changes(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<SurfaceMapUpdate, String> {
        let started = std::time::Instant::now();
        // A failed mask derivation may have advanced physical/builder state. Its
        // next local delta cannot be attached to the previous successful source.
        let change_basis = self.latest.as_ref().map(|old| Arc::clone(&old.snapshot));
        let intact_basis = self.latest.as_ref().zip(self.physical.as_ref())
            .is_some_and(|(old, (_, physical))| Arc::ptr_eq(&old.physical, physical));
        if message.cleared {
            self.reset();
            return Err("surface map cleared by mapping owner".into());
        }
        let fine_root = if message.fine.is_some() {
            if self.fine.is_none() {
                self.fine = Some(crate::surface_chunks::FineMapReader::new(&message.fine.as_ref().unwrap().repository_path));
            }
            Some(self.fine.as_mut().unwrap().receive(message)?)
        } else {
            None
        };
        let receive_ms = started.elapsed().as_secs_f64() * 1000.0;
        // 原生规划始终消费完整精细几何和地面记录，与旧机体复核开关无关。
        let (observations, fine_changes) = match fine_root.as_ref() {
            Some(root) => {
                let (observations, changes) = self.fine.as_mut().unwrap().observations_with_changes(root,
                    self.fine_observed_root.as_deref().zip(self.observations.as_ref()))?;
                (Some(observations), changes)
            }
            None => (None, self.observations.is_none().then(Vec::new)),
        };
        let observations_ms = started.elapsed().as_secs_f64() * 1000.0 - receive_ms;
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
                publication_revision: message.revision,
            },
        };
        let mask_revision = message.virtual_obstacles.as_ref().map(|_| MaskRevision {
            domain,
            revision: message.revision,
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
                let (updates, read_points) = nuwax_nav_mls_planner::surface_chunks::column_updates(
                    root, previous, self.config.voxel_size,
                    observations.as_ref().unwrap(), fine_changes.as_deref(),
                );
                self.builder
                    .build_column_updates(
                        &updates,
                        root.voxel_size_m,
                        previous.is_none(),
                        read_points,
                    )
                    .map_err(|error| error.to_string())?
            } else {
                let points = admitted_cloud_points(&message.cloud);
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
        tracing::debug!(target: "navigation.surface.build", event="surface.build.stages",
            cloud_revision=message.revision, receive_ms, observations_ms,
            geometry_ms=started.elapsed().as_secs_f64()*1000.0-receive_ms-observations_ms,
            stats=?self.last_stats, "表面构建阶段耗时");
        let same_mask = self.latest.as_ref().is_some_and(|previous| {
            if previous.mask_revision == mask_revision { return true; }
            let (Some(old), Some(next), Some(mask)) =
                (&previous.mask_revision, &mask_revision, &message.virtual_obstacles)
                else { return false; };
            if old.domain != next.domain || previous.mask_layout != mask_layout
            { return false; }
            // 比较借用完整输入与已有积分图，不保留或复制另一份遮罩。
            previous.matches_mask_contents(mask)
        });
        let reusable = same_mask && self.latest.as_ref()
            .is_some_and(|old| Arc::ptr_eq(&old.physical, &physical));
        let changes = |snapshot: &Arc<SurfaceSnapshot>| {
            if !intact_basis { return SurfaceGeometryChanges::Unknown; }
            if change_basis.as_ref().is_some_and(|old| nuwax_nav_runtime::native_map::same_source_geometry(old, snapshot)) {
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
        let snapshot = Arc::new(snapshot.with_observations(observations.clone()));
        self.observations = observations;
        self.fine_observed_root = fine_root;
        self.latest = Some(DerivedSurface { mask_revision, mask_layout, physical, snapshot: Arc::clone(&snapshot) });
        let changes = Arc::new(changes(&snapshot));
        Ok(SurfaceMapUpdate { snapshot, change_basis, changes })
    }
}

/// 解码边界已确认字段、布局与有限坐标；此处只展开坐标，不重复校验协议。
fn admitted_cloud_points(cloud: &schema::PointCloud2) -> Vec<(f32, f32, f32)> {
    let axes = ["x", "y", "z"].map(|name| {
        let field = cloud.fields.iter().find(|field| field.name == name).unwrap();
        (field.offset as usize, field.datatype)
    });
    let mut points = Vec::with_capacity(cloud.width as usize * cloud.height as usize);
    for row in 0..cloud.height as usize {
        for column in 0..cloud.width as usize {
            let base = row * cloud.row_step as usize + column * cloud.point_step as usize;
            let xyz = axes.map(|(offset, datatype)| {
                let start = base + offset;
                if datatype == 7 {
                    let bytes = cloud.data[start..start + 4].try_into().unwrap();
                    if cloud.is_bigendian { f32::from_be_bytes(bytes) } else { f32::from_le_bytes(bytes) }
                } else {
                    let bytes = cloud.data[start..start + 8].try_into().unwrap();
                    (if cloud.is_bigendian { f64::from_be_bytes(bytes) } else { f64::from_le_bytes(bytes) }) as f32
                }
            });
            points.push((xyz[0], xyz[1], xyz[2]));
        }
    }
    points
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn physical_snapshot(cache: &SurfaceMapCache) -> Option<Arc<SurfaceSnapshot>> {
        cache.physical.as_ref().map(|(_, snapshot)| Arc::clone(snapshot))
    }

    #[test]
    fn admitted_coordinates_preserve_endianness_offsets_and_padded_rows() {
        for big_endian in [false, true] {
            for datatype in [7, 8] {
                let bytes = if datatype == 7 { 4 } else { 8 };
                let mut cloud = schema::PointCloud2 {
                    width: 2, height: 2, point_step: bytes * 4, row_step: bytes * 9,
                    is_bigendian: big_endian,
                    fields: ["z", "x", "y"].iter().enumerate().map(|(index, name)| schema::PointField {
                        name: (*name).into(), offset: index as i32 * bytes, datatype, count: 1,
                    }).collect(),
                    ..Default::default()
                };
                cloud.data = vec![0; (cloud.row_step * cloud.height) as usize];
                for index in 0..4 {
                    let base = index / 2 * cloud.row_step as usize + index % 2 * cloud.point_step as usize;
                    for (axis, offset) in [(0, bytes), (1, bytes * 2), (2, 0)] {
                        let value = (index * 3 + axis) as f64;
                        let encoded = if datatype == 7 {
                            if big_endian { (value as f32).to_be_bytes() } else { (value as f32).to_le_bytes() }.to_vec()
                        } else {
                            if big_endian { value.to_be_bytes() } else { value.to_le_bytes() }.to_vec()
                        };
                        let start = base + offset as usize;
                        cloud.data[start..start + bytes as usize].copy_from_slice(&encoded);
                    }
                }
                assert_eq!(admitted_cloud_points(&cloud), vec![(0.,1.,2.),(3.,4.,5.),(6.,7.,8.),(9.,10.,11.)]);
            }
        }
    }
}
