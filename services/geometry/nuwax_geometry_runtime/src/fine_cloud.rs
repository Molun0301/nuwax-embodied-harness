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
    recent: VecDeque<Arc<SealedMap>>,
    readers: Vec<Weak<SealedMap>>,
    commits: u64,
}
impl FineCloud {
    pub(super) fn from_archive(file: std::fs::File) -> Result<(Self, nuwax_map_chunks::Manifest), String> {
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file)?;
        let mut cloud = Self::new(archive.manifest().voxel_size_m)?;
        let root = archive.import(cloud.writer.repository())?;
        cloud.writer.restore(&root)?;
        Ok((cloud, root))
    }
    pub(super) fn new(size: f32) -> Result<Self, String> {
        Ok(Self { writer: Writer::new(Repository::temporary()?, size, 32 * 1024 * 1024)?,
            dynamic: DynamicCloudFilter::default(), failure: None, recent: VecDeque::new(), readers: Vec::new(), commits: 0 })
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
    pub(super) fn seal(&mut self, timestamp_ns: u64) -> Result<Arc<SealedMap>, String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let sealed = Arc::new(self.writer.seal(timestamp_ns)?);
        self.recent.push_back(Arc::clone(&sealed));
        self.readers.push(Arc::downgrade(&sealed));
        // 慢读者可补取近期根；更旧的接收者重新获取最新完整根。
        while self.recent.len() > 16 || (self.recent.len() > 1 && self.recent.iter().map(|r| r.blocks.len()).sum::<usize>() > 262144) {
            self.recent.pop_front();
        }
        self.commits += 1;
        if self.commits % 16 == 0 {
            let live = self.readers.iter().filter_map(Weak::upgrade).collect::<Vec<_>>();
            self.readers.retain(|r| r.strong_count() > 0);
            self.writer.repository().collect_sealed(&live)?;
        }
        Ok(sealed)
    }
}
