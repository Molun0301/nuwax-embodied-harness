//! 原点云拥有方维护的固定精度源；完整块提交和展示容量互不影响。
use super::{dynamic_cloud::DynamicCloudFilter, voxel_index::VoxelIndex, voxel_key, VOXEL_SIZE};
use ahash::AHashSet;
use nalgebra::Point3;
use nuwax_map_chunks::{Repository, SealedMap, Writer};
use std::collections::VecDeque;
use std::sync::{Arc, Weak};

pub(super) struct FineCloud {
    writer: Writer,
    dynamic: DynamicCloudFilter,
    failure: Option<String>,
    readers: FineCloudReaders,
    commits: u64,
    baseline: Option<nuwax_map_chunks::RootIdentity>,
}
impl FineCloud {
    pub(super) fn from_archive(file: std::fs::File) -> Result<(Self, nuwax_map_chunks::ArchivedRoot), String> {
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file)?;
        let mut cloud = Self::new(archive.root().voxel_size_m())?;
        let root = archive.import(cloud.writer.repository())?;
        cloud.writer.restore_archived(&root)?;
        cloud.baseline = Some(root.identity());
        Ok((cloud, root))
    }
    pub(super) fn new(size: f32) -> Result<Self, String> {
        Ok(Self { writer: Writer::new(Repository::temporary()?, size, 32 * 1024 * 1024)?,
            dynamic: DynamicCloudFilter::default(), failure: None, readers: FineCloudReaders::default(), commits: 0, baseline: None })
    }
    pub(super) fn fresh() -> Result<Self, String> { Self::new(VOXEL_SIZE) }
    #[cfg(test)]
    pub(super) fn use_legacy_filter(&mut self) { self.dynamic.legacy_only = true; }
    pub(super) fn removed_keys(&self) -> &[(i32, i32, i32)] { self.dynamic.removed_keys() }
    pub(super) fn voxel_size(&self) -> f32 { self.writer.voxel_size_m() }
    pub(super) fn is_dynamic(&self, point: &Point3<f64>) -> bool {
        self.dynamic.is_dynamic([point.x, point.y, point.z])
    }
    pub(super) fn accumulate(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64) -> Result<(), String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let result = self.accumulate_inner(points, origin, timestamp_ns);
        // 存储故障后不能把部分更新公开成完整提交；由原拥有方的重建/新会话恢复。
        if let Err(error) = &result { self.failure = Some(error.clone()); }
        result
    }
    fn accumulate_inner(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64) -> Result<(), String> {
        self.dynamic.clear_frame();
        let size = self.writer.voxel_size_m();
        let hits = points.iter().map(|p| voxel_key([p.x, p.y, p.z], size).ok_or("精细输入坐标无效"))
            .collect::<Result<AHashSet<_>, _>>()?;
        if let Some(origin) = origin {
            // 原动态过滤器的射线最多30米；只读这部分真实块，远场静态结构保留。
            let low = voxel_key(origin.map(|v| v - 30.0 - f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let high = voxel_key(origin.map(|v| v + 30.0 + f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let local = self.writer.region([low.0, low.1, low.2], [high.0, high.1, high.2], 2_000_000)?;
            let mut voxels: VoxelIndex = local.into_iter().map(|(key, _)| (key[0], key[1], key[2])).collect();
            self.dynamic.clear_observed_free_space_with_hits(&mut voxels, size, origin, timestamp_ns, points, &hits);
            self.writer.remove(self.dynamic.removed_keys().iter().map(|key| [key.0, key.1, key.2]))?;
        }
        if timestamp_ns == 0 {
            self.writer.seed_unknown(hits.into_iter().map(|key| [key.0, key.1, key.2]))
        } else {
            self.writer.observe(hits.into_iter().map(|key| [key.0, key.1, key.2]), timestamp_ns)
        }
    }
    pub(super) fn publication_floor(&self, domain: &nuwax_map_chunks::Domain) -> u64 {
        self.baseline.as_ref().filter(|root| &root.domain == domain).map_or(0, |root| root.revision)
    }
    pub(super) fn readers(&self) -> FineCloudReaders { self.readers.clone() }
    pub(super) fn adopt_readers(&mut self, readers: FineCloudReaders) { self.readers = readers; }
    pub(super) fn seal(&mut self, domain: nuwax_map_chunks::Domain, revision: u64, timestamp_ns: u64) -> Result<Arc<SealedMap>, String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let sealed = Arc::new(self.writer.seal(domain, revision, timestamp_ns)?);
        self.baseline = Some(sealed.manifest().identity());
        self.commits = self.commits.wrapping_add(1);
        if self.commits % 16 == 0 {
            let live = {
                let mut readers = self.readers.0.lock();
                let live = readers.readers.iter().filter_map(Weak::upgrade).collect::<Vec<_>>();
                readers.readers.retain(|r| r.strong_count() > 0);
                live
            };
            // GC stays in private preparation, outside the short installation
            // lock. The writer also pins this not-yet-published baseline.
            if let Err(error) = self.writer.collect_sealed(&live) {
                tracing::warn!(%error, "fine-map object collection deferred");
            }
        }
        Ok(sealed)
    }
    pub(super) fn rehome_committed(&mut self, expected: &nuwax_map_chunks::RootIdentity,
        new_frame: u64) -> Result<Arc<SealedMap>, String> {
        if self.baseline.as_ref() != Some(expected) {
            return Err("fine frame rehome does not name the committed baseline".into());
        }
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let root = Arc::new(self.writer.rehome_published_frame(expected, new_frame)?);
        self.baseline = Some(root.manifest().identity());
        Ok(root)
    }
    pub(super) fn retain_committed(&mut self, sealed: &Arc<SealedMap>) { self.readers.retain_committed(sealed); }

}

#[derive(Default)]
struct RetainedRoots {
    recent: VecDeque<Arc<SealedMap>>,
    readers: Vec<Weak<SealedMap>>,
}
/// Shared retention survives transfer of the mutable accumulator to a worker.
#[derive(Clone, Default)]
pub(super) struct FineCloudReaders(Arc<parking_lot::Mutex<RetainedRoots>>);
impl FineCloudReaders {
    pub(super) fn retain_committed(&self, sealed: &Arc<SealedMap>) {
        let mut readers = self.0.lock();
        readers.recent.push_back(Arc::clone(sealed));
        readers.readers.push(Arc::downgrade(sealed));
        while readers.recent.len() > 16 || (readers.recent.len() > 1 && readers.recent.iter().map(|r| r.manifest().blocks.len()).sum::<usize>() > 262144) {
            readers.recent.pop_front();
        }
    }
    /// Resolve the exact retained publication before starting asynchronous IO.
    pub(super) fn pin_root(&self, identity: &nuwax_map_chunks::RootIdentity,
        reference: &nuwax_map_chunks::BlockRef) -> Result<Arc<SealedMap>, String> {
        let retained = self.0.lock();
        let root = retained.recent.iter().find(|root| root.manifest().identity() == *identity)
            .cloned().or_else(|| retained.readers.iter().filter_map(Weak::upgrade)
                .find(|root| root.manifest().identity() == *identity))
            .ok_or("requested fine-map root expired; request the latest complete root")?;
        let manifest = root.manifest();
        let index = manifest.blocks.binary_search_by_key(&reference.key, |block| block.key)
            .map_err(|_| "requested block is not a member of the retained root")?;
        if &manifest.blocks[index] != reference {
            return Err("requested block identity/revisions differ from the retained root".into());
        }
        Ok(root)
    }
}

#[cfg(test)]
mod revision_tests {
    use super::*;
    fn domain() -> nuwax_map_chunks::Domain {
        nuwax_map_chunks::Domain { session: "owner".into(), generation: 3, frame_generation: 4, producer_generation: 5 }
    }
    #[test]
    fn retained_lookup_requires_exact_root_and_block_and_pins_old_reader() {
        let mut source = FineCloud::fresh().unwrap();
        source.accumulate(&[Point3::new(-0.025, 0.025, 0.175)], None, 1).unwrap();
        let first = source.seal(domain(), 1, 1).unwrap();
        let readers = source.readers();
        let identity = first.manifest().identity();
        let reference = first.manifest().blocks[0].clone();
        assert!(readers.pin_root(&identity, &reference).is_err());
        source.retain_committed(&first);
        let pinned = readers.pin_root(&identity, &reference).unwrap();
        let mut forged = reference.clone();
        forged.block_revision += 1;
        assert!(readers.pin_root(&identity, &forged).is_err());
        let mut wrong_domain = identity.clone();
        wrong_domain.domain.frame_generation += 1;
        assert!(readers.pin_root(&wrong_domain, &reference).is_err());
        for revision in 2..=20 {
            source.accumulate(&[Point3::new(-0.025, 0.025, 0.175)], None, revision).unwrap();
            let root = source.seal(domain(), revision, revision).unwrap();
            assert_eq!(root.manifest().geometry_revision, 1);
            source.retain_committed(&root);
        }
        assert!(pinned.repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).is_ok());
        assert!(readers.pin_root(&identity, &reference).is_ok());
        drop(first);
        drop(pinned);
        assert!(readers.pin_root(&identity, &reference).is_err());
    }
}
