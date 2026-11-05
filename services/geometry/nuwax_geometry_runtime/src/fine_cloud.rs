//! 原点云拥有方维护的固定精度源；完整块提交和展示容量互不影响。
use super::{dynamic_cloud::DynamicCloudFilter, voxel_index::VoxelIndex, voxel_key, VOXEL_SIZE};
use ahash::AHashSet;
use nalgebra::Point3;
use nuwax_map_chunks::{CachePolicy, Repository, SealedMap, Writer};
use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, OnceLock, Weak};
#[path = "heightfield_capture.rs"]
mod heightfield_capture;

pub(crate) const FINE_CLOUD_CACHE_MIB_VAR: &str = "NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB";
pub(crate) const FINE_CLOUD_CACHE_AUTO_MAX_VAR: &str =
    "NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB";
const MIB: usize = 1024 * 1024;
/// 自动模式的起步预算。
const AUTO_START_MIB: usize = 32;
const DEFAULT_CACHE: FineCloudCache = FineCloudCache::Auto { max_mib: 256 };
static CONFIGURED_CACHE: OnceLock<FineCloudCache> = OnceLock::new();

/// 精细点云写入器缓存配置：固定预算，或从 32 MiB 起按需增长到上限的自动模式。
/// 在线建图与 PGO 私有重放的写入器各自独立按此创建；两种模式写入的地图相同。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FineCloudCache {
    Fixed { mib: usize },
    Auto { max_mib: usize },
}

/// 解析两个配置值（`None` 为未设置），与配置保存时的规范化一致：去掉首尾空白，空白或
/// 未设置按缺省（自动模式、上限 256 MiB）；数值只接受十进制数字。上限只在自动模式下
/// 生效，固定模式忽略它。其它非法值返回带变量名和合法范围的错误。
pub(crate) fn parse_fine_cloud_cache(
    mode: Option<&str>,
    auto_max: Option<&str>,
) -> Result<FineCloudCache, String> {
    let integer = |value: &str, low: usize, high: usize| {
        value
            .bytes()
            .all(|b| b.is_ascii_digit())
            .then(|| value.parse::<usize>().ok())
            .flatten()
            .filter(|v| (low..=high).contains(v))
    };
    match mode.map(str::trim).filter(|value| !value.is_empty()) {
        None | Some("auto") => match auto_max.map(str::trim).filter(|value| !value.is_empty()) {
            None => Ok(DEFAULT_CACHE),
            Some(value) => integer(value, 32, 4096).map(|max_mib| FineCloudCache::Auto { max_mib }).ok_or_else(||
                format!("{FINE_CLOUD_CACHE_AUTO_MAX_VAR} 取值非法：{value:?}，应为 32～4096 的整数（MiB）")),
        },
        Some(value) => integer(value, 16, 4096).map(|mib| FineCloudCache::Fixed { mib }).ok_or_else(||
            format!("{FINE_CLOUD_CACHE_MIB_VAR} 取值非法：{value:?}，应为 auto 或 16～4096 的整数（MiB）")),
    }
}

/// 启动时按配置设定一次；未设定时（例如单元测试）按缺省自动模式。
pub(crate) fn configure_fine_cloud_cache(cache: FineCloudCache) -> Result<(), String> {
    CONFIGURED_CACHE
        .set(cache)
        .map_err(|_| "fine-cloud writer cache is already configured".into())
}

pub(crate) fn configured_writer_cache() -> (usize, CachePolicy) {
    match CONFIGURED_CACHE.get().copied().unwrap_or(DEFAULT_CACHE) {
        FineCloudCache::Fixed { mib } => (mib * MIB, CachePolicy::Fixed),
        FineCloudCache::Auto { max_mib } => (
            AUTO_START_MIB * MIB,
            CachePolicy::Auto {
                max_bytes: max_mib * MIB,
            },
        ),
    }
}
pub(crate) struct FineCloud {
    writer: Writer,
    dynamic: DynamicCloudFilter,
    failure: Option<String>,
    readers: FineCloudReaders,
    commits: u64,
    #[cfg(test)]
    last_revoke_read_blocks: usize,
    #[cfg(test)]
    last_revoke_block_lookups: usize,
    #[cfg(test)]
    last_revoke_ms: f64,
}
impl FineCloud {
    #[cfg(test)]
    pub(crate) fn fresh() -> Result<Self, String> {
        Self::new(VOXEL_SIZE)
    }
    #[cfg(test)]
    fn voxel_size(&self) -> f32 {
        self.writer.voxel_size_m()
    }
    #[cfg(test)]
    fn last_revoke_read_blocks(&self) -> usize {
        self.last_revoke_read_blocks
    }
    #[cfg(test)]
    fn last_revoke_block_lookups(&self) -> usize {
        self.last_revoke_block_lookups
    }
    #[cfg(test)]
    fn revoke_index_memory_bytes(&self) -> usize {
        0
    }
    #[cfg(test)]
    fn shared_frame(&self) -> Option<&crate::dynamic_backend::MappedFrame> {
        self.dynamic.shared_frame()
    }

    pub(crate) fn new(size: f32) -> Result<Self, String> {
        let (budget, policy) = configured_writer_cache();
        let mut writer = Writer::new(Repository::temporary()?, size, budget)?;
        writer.set_cache_policy(policy)?;
        Ok(Self {
            writer,
            dynamic: Default::default(),
            failure: None,
            readers: Default::default(),
            commits: 0,
            #[cfg(test)]
            last_revoke_read_blocks: 0,
            #[cfg(test)]
            last_revoke_block_lookups: 0,
            #[cfg(test)]
            last_revoke_ms: 0.,
        })
    }
    pub(crate) fn from_archive(file: std::fs::File) -> Result<Self, String> {
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file)?;
        let mut value = Self::new(archive.root().voxel_size_m())?;
        let root = archive.import(value.writer.repository())?;
        value.writer.restore_archived(&root)?;
        Ok(value)
    }
    pub(crate) fn removed_keys(&self) -> &[(i32, i32, i32)] {
        self.dynamic.removed_keys()
    }
    pub(crate) fn is_dynamic(&self, p: &Point3<f64>) -> bool {
        self.dynamic.is_dynamic([p.x, p.y, p.z])
    }
    pub(crate) fn accumulate(
        &mut self,
        session_id: u64,
        sensor_id: &str,
        sequence: u64,
        points: &[Point3<f64>],
        origin: [f64; 3],
        stamp: u64,
        shared: Option<crate::dynamic_backend::MappedFrame>,
    ) -> Result<Vec<u32>, String> {
        let result = (|| {
            let keys = self.accumulate_inner(points, Some(origin), stamp, shared)?;
            let returns: Vec<_> = points
                .iter()
                .zip(keys)
                .enumerate()
                .map(|(index, (p, key))| {
                    heightfield_capture::Return::from_key(
                        [p.x, p.y, p.z],
                        key,
                        index as u32,
                        self.dynamic.is_dynamic([p.x, p.y, p.z]),
                        self.writer.voxel_size_m(),
                    )
                })
                .collect();
            let removed: Vec<_> = self
                .dynamic
                .removed_keys()
                .iter()
                .map(|k| [k.0, k.1, k.2])
                .collect();
            let source = Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
                sensor_id: sensor_id.into(),
                clock_id: "unix".into(),
                runtime_generation: 1,
                session_generation: session_id,
            });
            heightfield_capture::capture(
                &mut self.writer,
                &returns,
                &removed,
                sequence,
                stamp,
                source,
            )
        })();
        if let Err(error) = &result {
            self.failure = Some(error.clone())
        }
        result
    }
    fn accumulate_inner(
        &mut self,
        points: &[Point3<f64>],
        origin: Option<[f64; 3]>,
        timestamp_ns: u64,
        mut shared: Option<crate::dynamic_backend::MappedFrame>,
    ) -> Result<Vec<nuwax_map_chunks::Key>, String> {
        self.dynamic.clear_frame();
        #[cfg(test)]
        {
            self.last_revoke_read_blocks = 0;
            self.last_revoke_block_lookups = 0;
            self.last_revoke_ms = 0.0;
        }
        let size = self.writer.voxel_size_m();
        let keys = points
            .iter()
            .map(|p| {
                voxel_key([p.x, p.y, p.z], size)
                    .map(|k| [k.0, k.1, k.2])
                    .ok_or("精细输入坐标无效")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let hits: AHashSet<_> = keys.iter().map(|k| (k[0], k[1], k[2])).collect();
        // 零时间种子也先取得DUFO分类；observe内部不插入，其空增量应绕过旧窗口查询。
        if shared.is_none() {
            if let Some(origin) = origin {
                shared = Some(self.dynamic.observe_dufo(points, origin, timestamp_ns)?);
            }
        }
        #[cfg(test)]
        let revoke_started = std::time::Instant::now();
        if shared.as_ref().is_some_and(|f| !f.has_clears()) {
            self.dynamic
                .adopt(shared.take().unwrap(), &mut VoxelIndex::default(), size);
        } else if let Some(origin) = origin {
            #[cfg(test)]
            let reference = crate::dynamic_backend::reference_revoke();
            #[cfg(not(test))]
            let reference = false;
            if let Some(frame) = shared.as_ref().filter(|_| !reference) {
                let mut blocks = BTreeSet::new();
                for (low, high) in frame
                    .clear_regions()
                    .expect("DUFO帧必须提供新增空单元迭代器")
                {
                    let low = voxel_key(low, size).ok_or("精细动态窗口越界")?;
                    let high = voxel_key(high, size).ok_or("精细动态窗口越界")?;
                    let lookups = self.writer.collect_intersecting_blocks(
                        [low.0, low.1, low.2],
                        [high.0, high.1, high.2],
                        &mut blocks,
                    )?;
                    #[cfg(test)]
                    {
                        self.last_revoke_block_lookups += lookups;
                    }
                    #[cfg(not(test))]
                    let _ = lookups;
                }
                let (removed, read_blocks) =
                    self.writer.matching_keys_in_blocks(&blocks, |key| {
                        frame.clears(super::voxel_center_f64((key[0], key[1], key[2]), size))
                    })?;
                #[cfg(test)]
                {
                    self.last_revoke_read_blocks = read_blocks;
                }
                #[cfg(not(test))]
                let _ = read_blocks;
                let mut voxels = removed
                    .into_iter()
                    .map(|key| (key[0], key[1], key[2]))
                    .collect();
                // 仍由原过滤器复核并生成撤销增量，后续地面关联与提交语义不变。
                self.dynamic
                    .adopt(shared.take().unwrap(), &mut voxels, size);
                self.writer.remove(
                    self.dynamic
                        .removed_keys()
                        .iter()
                        .map(|key| [key.0, key.1, key.2]),
                )?;
            } else {
                let (low, high) = shared.as_ref().and_then(|f| f.clear_bounds()).unwrap_or((
                    origin.map(|v| v - 30.0 - f64::from(size)),
                    origin.map(|v| v + 30.0 + f64::from(size)),
                ));
                let low = voxel_key(low, size).ok_or("精细动态窗口越界")?;
                let high = voxel_key(high, size).ok_or("精细动态窗口越界")?;
                let local = self.writer.region(
                    [low.0, low.1, low.2],
                    [high.0, high.1, high.2],
                    2_000_000,
                )?;
                let mut voxels: VoxelIndex = local
                    .into_iter()
                    .map(|(key, _)| (key[0], key[1], key[2]))
                    .collect();
                if let Some(shared) = shared {
                    self.dynamic.adopt(shared, &mut voxels, size);
                } else {
                    self.dynamic.observe_with_ground(
                        &mut voxels,
                        size,
                        origin,
                        timestamp_ns,
                        points,
                    );
                }
                self.writer.remove(
                    self.dynamic
                        .removed_keys()
                        .iter()
                        .map(|key| [key.0, key.1, key.2]),
                )?;
            }
        }
        #[cfg(test)]
        {
            self.last_revoke_ms = revoke_started.elapsed().as_secs_f64() * 1000.0;
        }
        if timestamp_ns == 0 {
            self.writer
                .seed_unknown(hits.into_iter().map(|key| [key.0, key.1, key.2]))?;
        } else {
            let retained: AHashSet<_> = points
                .iter()
                .zip(&keys)
                .filter_map(|(p, &k)| (!self.dynamic.is_dynamic([p.x, p.y, p.z])).then_some(k))
                .collect();
            self.writer.observe(retained, timestamp_ns)?;
        }
        Ok(keys)
    }
    pub(crate) fn seal(
        &mut self,
        domain: nuwax_map_chunks::Domain,
        revision: u64,
        timestamp_ns: u64,
    ) -> Result<Arc<SealedMap>, String> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        let sealed = Arc::new(self.writer.seal(domain, revision, timestamp_ns)?);
        self.commits = self.commits.wrapping_add(1);
        if self.commits % 16 == 0 {
            let live = {
                let mut readers = self.readers.0.lock();
                let live = readers
                    .readers
                    .iter()
                    .filter_map(Weak::upgrade)
                    .collect::<Vec<_>>();
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
    pub(crate) fn retain_committed(&mut self, sealed: &Arc<SealedMap>) {
        self.readers.retain_committed(sealed);
    }
}

#[derive(Default)]
struct RetainedRoots {
    recent: VecDeque<Arc<SealedMap>>,
    readers: Vec<Weak<SealedMap>>,
}
#[derive(Clone, Default)]
pub(crate) struct FineCloudReaders(Arc<parking_lot::Mutex<RetainedRoots>>);
impl FineCloudReaders {
    pub(crate) fn retain_committed(&self, sealed: &Arc<SealedMap>) {
        let mut retained = self.0.lock();
        retained.readers.push(Arc::downgrade(sealed));
        retained.recent.push_back(sealed.clone());
        while retained.recent.len() > 2 {
            retained.recent.pop_front();
        }
    }
}

#[cfg(test)]
#[path = "fine_cloud_revoke_locality_tests.rs"]
mod revoke_locality_tests;

#[cfg(test)]
#[path = "fine_cloud_cache_config_tests.rs"]
mod cache_config_tests;
