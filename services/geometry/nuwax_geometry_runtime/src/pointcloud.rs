use ahash::AHashSet as HashSet;
use std::io::{Error, ErrorKind};
use std::os::unix::fs::MetadataExt;
use std::path::Path;
use std::sync::atomic::{fence, Ordering};
use std::sync::Arc;

use memmap2::MmapMut;
use nalgebra::Point3;
use tracing::{info, warn};

use crate::mapping_commit::{CloudCommitReceipt, OccupancyCommitReceipt};

#[path = "voxel_index.rs"]
mod voxel_index;
use voxel_index::VoxelIndex;

#[path = "dynamic_cloud.rs"]
mod dynamic_cloud;
use dynamic_cloud::DynamicCloudFilter;

#[path = "fine_cloud.rs"]
mod fine_cloud;
use fine_cloud::FineCloud;

#[path = "ordered_voxel_snapshot.rs"]
mod ordered_voxel_snapshot;
use ordered_voxel_snapshot::OrderedVoxelSnapshot;

pub const VOXEL_SIZE: f32 = 0.05;

/// 单次输入点云允许的 raw point 上限(防御性 sanity cap)。
///
/// 单帧 LIDAR(PointCloud2)原始点云合理上限 ~30k(Unitree Go2 雷达规格);
/// 100k 已远超物理合理范围。MappingSession 的 8M 滚动预算是跨帧累计上限，
/// 不能用作单帧上限。
///
/// 超过此上限说明上游 LCM 帧 `point_step` 或 `data` 长度异常,直接拒绝
/// 该帧，避免按异常 raw_count 分配超大 Vec(2026-07-08 实机 abort 模式)。
pub const POINTCLOUD_MAX_FRAME_POINTS: usize = 100_000;
pub(crate) const IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES: usize = 120 * 1024 * 1024;

/// 单次 cloud_map 帧允许的 point_step 字节数上限(防御性 sanity cap)。
///
/// PointCloud2 协议字段 `point_step` 描述每个点的字节宽度;`sensor_msgs`
/// 实际值在 12~48 bytes 之间(xyz=f32×3=12,xyz+intensity=16,xyz+rgb=32 等)。
/// 设 256 字节作为宽松上界,覆盖任何扩展字段。
///
/// 超过此上限或低于 12 字节 → 拒绝该帧,防止 `bytes.len()/point_step`
/// 推算 raw_count 时产生巨大值。
pub const POINTCLOUD_MAX_POINT_STEP_BYTES: u32 = 256;

// ─── 累积地图双缓冲共享内存写入器 ───

const CM_SHM_MAGIC: u64 = 0x4E55_5741_584D_4150; // "NUWAXMAP"
pub(crate) const CM_SHM_VERSION: u32 = 2;
/// Tightly packed little-endian XYZ, three f32 values per point.
pub(crate) const CM_PAYLOAD_SCHEMA_VERSION: u32 = 1;
const CM_HEADER_SIZE: usize = 64;
pub(crate) const CM_SINGLE_BUFFER_SIZE: usize = 16 * 1024 * 1024; // 16MB per buffer
pub(crate) const CM_TOTAL_SIZE: usize = CM_HEADER_SIZE + CM_SINGLE_BUFFER_SIZE * 2;
pub(crate) const CM_POINT_SIZE_BYTES: usize = 12;
const CM_MAX_POINTS: usize = CM_SINGLE_BUFFER_SIZE / CM_POINT_SIZE_BYTES;
const MAX_VOXEL_SIZE: f32 = 1_048_576.0;

type VoxelKey = (i32, i32, i32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ProductOwnedMmapIdentity {
    device: u64,
    inode: u64,
}

impl ProductOwnedMmapIdentity {
    fn from_metadata(metadata: &std::fs::Metadata) -> Self {
        Self {
            device: metadata.dev(),
            inode: metadata.ino(),
        }
    }
}

pub(crate) fn capture_product_owned_mmap(
    path: &Path,
    file: &std::fs::File,
) -> std::io::Result<ProductOwnedMmapIdentity> {
    let descriptor_metadata = file.metadata()?;
    let named_metadata = std::fs::symlink_metadata(path)?;
    let descriptor_identity = ProductOwnedMmapIdentity::from_metadata(&descriptor_metadata);
    let named_identity = ProductOwnedMmapIdentity::from_metadata(&named_metadata);
    if !named_metadata.file_type().is_file() || named_identity != descriptor_identity {
        return Err(Error::other(format!(
            "product-owned mmap path changed during creation: path={} descriptor={descriptor_identity:?} named={named_identity:?}",
            path.display()
        )));
    }
    Ok(descriptor_identity)
}

pub(crate) fn unlink_product_owned_mmap(
    path: &Path,
    expected_identity: ProductOwnedMmapIdentity,
) -> std::io::Result<()> {
    let named_metadata = match std::fs::symlink_metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == ErrorKind::NotFound => return Ok(()),
        Err(error) => return Err(error),
    };
    let actual_identity = ProductOwnedMmapIdentity::from_metadata(&named_metadata);
    if !named_metadata.file_type().is_file() || actual_identity != expected_identity {
        return Err(Error::other(format!(
            "refusing to unlink replaced mmap path: path={} expected={expected_identity:?} actual={actual_identity:?}",
            path.display()
        )));
    }

    match std::fs::remove_file(path) {
        Ok(()) => {}
        Err(error) if error.kind() == ErrorKind::NotFound => return Ok(()),
        Err(error) => return Err(error),
    }
    match std::fs::symlink_metadata(path) {
        Err(error) if error.kind() == ErrorKind::NotFound => Ok(()),
        Ok(metadata) => Err(Error::other(format!(
            "mmap path was recreated during cleanup: path={} actual={:?}",
            path.display(),
            ProductOwnedMmapIdentity::from_metadata(&metadata)
        ))),
        Err(error) => Err(error),
    }
}

const CM_ACTIVE_BUFFER_OFFSET: usize = 12;
const CM_GLOBAL_POINT_COUNT_OFFSET: usize = 16;
const CM_TIMESTAMP_NS_OFFSET: usize = 20;
const CM_GLOBAL_REVISION_OFFSET: usize = 28;
const CM_VOXEL_SIZE_OFFSET: usize = 36;
const CM_SLOT0_REVISION_OFFSET: usize = 40;
const CM_SLOT0_POINT_COUNT_OFFSET: usize = 48;
const CM_SLOT1_REVISION_OFFSET: usize = 52;
const CM_SLOT1_POINT_COUNT_OFFSET: usize = 60;

pub(crate) fn cloud_map_shm_path() -> std::path::PathBuf {
    std::path::PathBuf::from(
        std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into()),
    )
}

pub(crate) fn cloud_map_active_slot(revision: u64) -> Option<u32> {
    revision.checked_sub(1).map(|value| (value % 2) as u32)
}

pub struct CloudMapShmWriter {
    mmap: MmapMut,
    shm_path: std::path::PathBuf,
    shm_identity: Option<ProductOwnedMmapIdentity>,
    revision: u64,
    voxel_size: f32,
    session_generation: u64,
    voxels: VoxelIndex,
    ordered_voxels: OrderedVoxelSnapshot,
    dynamic_filter: DynamicCloudFilter,
    fine_cloud: Option<FineCloud>,
    committed_fine_cloud: Option<Arc<nuwax_map_chunks::SealedMap>>,
    max_points: usize,
    last_occupancy_receipt: Option<OccupancyCommitReceipt>,
    last_cloud_receipt: Option<CloudCommitReceipt>,
    typed_publish_state: Arc<TypedCloudPublishState>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PendingTypedCloudClear {
    pub(crate) revision: u64,
    pub(crate) point_count: u32,
    pub(crate) voxel_size: f32,
    pub(crate) session_generation: u64,
}

#[derive(Default)]
struct TypedCloudPublishProgress {
    published_identity: Option<(u64, u64)>,
    pending_clear: Option<PendingTypedCloudClear>,
}

pub(crate) struct TypedCloudPublishState {
    sequencer: tokio::sync::Mutex<()>,
    progress: parking_lot::Mutex<TypedCloudPublishProgress>,
    session_generation_tx: tokio::sync::watch::Sender<u64>,
}

impl TypedCloudPublishState {
    fn new() -> Self {
        let (session_generation_tx, _) = tokio::sync::watch::channel(0);
        Self {
            sequencer: tokio::sync::Mutex::new(()),
            progress: parking_lot::Mutex::new(TypedCloudPublishProgress::default()),
            session_generation_tx,
        }
    }

    pub(crate) async fn lock_sequencer(&self) -> tokio::sync::MutexGuard<'_, ()> {
        self.sequencer.lock().await
    }

    pub(crate) fn session_generation_rx(&self) -> tokio::sync::watch::Receiver<u64> {
        self.session_generation_tx.subscribe()
    }

    pub(crate) fn observe_session_generation(&self, session_generation: u64) {
        if *self.session_generation_tx.borrow() != session_generation {
            self.session_generation_tx.send_replace(session_generation);
        }
    }

    pub(crate) fn pending_clear_for_session(
        &self,
        session_generation: u64,
    ) -> Option<PendingTypedCloudClear> {
        let mut progress = self.progress.lock();
        if progress
            .pending_clear
            .as_ref()
            .is_some_and(|clear| clear.session_generation != session_generation)
        {
            progress.pending_clear = None;
        }
        progress.pending_clear.clone()
    }

    pub(crate) fn published_identity(&self) -> Option<(u64, u64)> {
        self.progress.lock().published_identity
    }

    pub(crate) fn mark_published(&self, identity: (u64, u64)) {
        let mut progress = self.progress.lock();
        progress.published_identity = Some(identity);
        if progress
            .pending_clear
            .as_ref()
            .map(|clear| (clear.session_generation, clear.revision))
            == Some(identity)
        {
            progress.pending_clear = None;
        }
    }

    fn reserve_session_clear(&self, clear: PendingTypedCloudClear) {
        let session_generation = clear.session_generation;
        self.progress.lock().pending_clear = Some(clear);
        self.session_generation_tx.send_replace(session_generation);
    }
}

/// One successfully committed cumulative cloud-map revision and the exact
/// canonical map-frame voxel snapshot available to typed transport sinks.
pub(crate) struct CommittedCloudMapSnapshot {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub session_generation: u64,
    pub xyz_f32_le: Vec<u8>,
    /// Exact SHA-256 identity of the packed XYZ payload.
    pub xyz_sha256: [u8; 32],
}

/// 私有重建得到的完整累积状态，包含尚未达到删除门限的动态反证。
pub(crate) struct PreparedCloudAccumulator {
    voxels: VoxelIndex,
    voxel_size: f32,
    dynamic_filter: DynamicCloudFilter,
    ordered_voxels: OrderedVoxelSnapshot,
    fine_cloud: Option<FineCloud>,
}

/// 先于激活持锁提交完成读取、逐块校验和私有导入。
pub(crate) struct PreparedFineCloud { cloud: FineCloud }

pub(crate) fn prepare_fine_cloud_archive(file: std::fs::File, session: &str, generation: u64,
    cloud_revision: u64, source_timestamp_ns: u64) -> Result<PreparedFineCloud, String> {
    let (cloud, root) = FineCloud::from_archive(file)?;
    if root.domain.session != session || root.domain.generation != generation
        || root.revision != cloud_revision || root.source_timestamp_ns != source_timestamp_ns {
        return Err("精细地图资产与原占据/点云提交不一致".into());
    }
    Ok(PreparedFineCloud { cloud })
}

pub(crate) struct CorrelatedCloudMapCommit {
    pub(crate) snapshot: CommittedCloudMapSnapshot,
    pub(crate) receipt: CloudCommitReceipt,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ImmutableActivatedTarget {
    pub(crate) frame_id: String,
    pub(crate) source_revision: u64,
    pub(crate) point_count: u32,
    pub(crate) voxel_size: f32,
    pub(crate) xyz_f32_le: Vec<u8>,
    pub(crate) xyz_sha256: [u8; 32],
    pub(crate) payload_schema_version: u32,
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_immutable_activated_target(
    frame_id: &str,
    expected_map_frame_id: &str,
    source_revision: u64,
    point_count: u32,
    voxel_size: f32,
    xyz_f32_le: &[u8],
    xyz_sha256: &str,
    payload_schema_version: u32,
) -> Result<ImmutableActivatedTarget, String> {
    if payload_schema_version != CM_PAYLOAD_SCHEMA_VERSION {
        return Err(format!(
            "unsupported immutable target schema: expected={} actual={payload_schema_version}",
            CM_PAYLOAD_SCHEMA_VERSION
        ));
    }
    if frame_id.is_empty()
        || frame_id.len() > 256
        || frame_id != frame_id.trim()
        || frame_id != expected_map_frame_id
    {
        return Err(format!(
            "immutable target frame must be canonical {expected_map_frame_id}, received {frame_id}"
        ));
    }
    if source_revision == 0 {
        return Err("immutable target source_revision must be positive".to_owned());
    }
    if point_count == 0 {
        return Err("immutable target must contain at least one point".to_owned());
    }
    if !voxel_size.is_finite() || voxel_size <= 0.0 || voxel_size > MAX_VOXEL_SIZE {
        return Err("immutable target voxel_size must be finite and positive".to_owned());
    }

    let expected_size = usize::try_from(point_count)
        .map_err(|_| "immutable target point_count exceeds usize".to_owned())?
        .checked_mul(CM_POINT_SIZE_BYTES)
        .ok_or_else(|| "immutable target payload size overflow".to_owned())?;
    if expected_size > IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES {
        return Err(format!(
            "immutable target exceeds {} bytes: {expected_size}",
            IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES
        ));
    }
    if xyz_f32_le.len() != expected_size {
        return Err(format!(
            "immutable target payload size mismatch: expected={expected_size} actual={}",
            xyz_f32_le.len()
        ));
    }

    let expected_digest = decode_checkpoint_sha256(xyz_sha256)?;
    let actual_digest = crate::map_checksum::sha256_bytes(xyz_f32_le);
    if actual_digest != expected_digest {
        return Err("immutable target xyz_sha256 mismatch".to_owned());
    }
    for (index, point) in xyz_f32_le.chunks_exact(CM_POINT_SIZE_BYTES).enumerate() {
        let coordinates = [
            f32::from_le_bytes(point[0..4].try_into().unwrap()),
            f32::from_le_bytes(point[4..8].try_into().unwrap()),
            f32::from_le_bytes(point[8..12].try_into().unwrap()),
        ];
        if coordinates.iter().any(|value| !value.is_finite()) {
            return Err(format!("immutable target point {index} is non-finite"));
        }
    }

    Ok(ImmutableActivatedTarget {
        frame_id: frame_id.to_owned(),
        source_revision,
        point_count,
        voxel_size,
        xyz_f32_le: xyz_f32_le.to_vec(),
        xyz_sha256: actual_digest,
        payload_schema_version,
    })
}

pub(crate) fn decode_checkpoint_sha256(value: &str) -> Result<[u8; 32], String> {
    let value = value.trim_matches(|character: char| character.is_ascii_whitespace());
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err("immutable target xyz_sha256 must be exactly 64 hexadecimal digits".to_owned());
    }
    let mut digest = [0_u8; 32];
    for (index, byte) in digest.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&value[index * 2..index * 2 + 2], 16)
            .map_err(|_| "immutable target xyz_sha256 is invalid".to_owned())?;
    }
    Ok(digest)
}

impl CloudMapShmWriter {
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(shm_path)?;
        let shm_identity = capture_product_owned_mmap(shm_path, &file)?;
        file.set_len(CM_TOTAL_SIZE as u64)?;
        let mmap = unsafe { MmapMut::map_mut(&file)? };
        info!(
            "累积地图双缓冲 mmap 已创建: {:?} ({} bytes)",
            shm_path, CM_TOTAL_SIZE
        );
        Ok(Self::from_mapping(
            mmap,
            shm_path.to_path_buf(),
            Some(shm_identity),
        ))
    }

    /// 私有重建不对外发布，使用匿名内存，避免每次纠偏创建和删除临时映射文件。
    pub(crate) fn new_private() -> std::io::Result<Self> {
        Ok(Self::from_mapping(
            MmapMut::map_anon(CM_TOTAL_SIZE)?,
            Default::default(),
            None,
        ))
    }

    fn from_mapping(
        mut mmap: MmapMut,
        shm_path: std::path::PathBuf,
        shm_identity: Option<ProductOwnedMmapIdentity>,
    ) -> Self {
        mmap[0..8].copy_from_slice(&CM_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&CM_SHM_VERSION.to_le_bytes());
        mmap[12..CM_HEADER_SIZE].fill(0);
        Self {
            mmap,
            shm_path,
            shm_identity,
            revision: 0,
            voxel_size: VOXEL_SIZE,
            session_generation: 0,
            voxels: VoxelIndex::new(),
            ordered_voxels: OrderedVoxelSnapshot::default(),
            dynamic_filter: DynamicCloudFilter::default(),
            fine_cloud: None,
            committed_fine_cloud: None,
            max_points: CM_MAX_POINTS,
            last_occupancy_receipt: None,
            last_cloud_receipt: None,
            typed_publish_state: Arc::new(TypedCloudPublishState::new()),
        }
    }

    pub(crate) fn cleanup_product_owned_file(&mut self) -> std::io::Result<()> {
        match self.shm_identity {
            Some(identity) => unlink_product_owned_mmap(&self.shm_path, identity),
            None => Ok(()),
        }
    }

    pub(crate) fn typed_publish_state(&self) -> Arc<TypedCloudPublishState> {
        self.typed_publish_state.clone()
    }

    /// Increment the independent cloud-map voxel accumulator with one accepted
    /// canonical map-frame lidar message. This is O(new frame points), not
    /// O(MappingSession's rolling 8M point window).
    pub fn accumulate_mapping_frame(
        &mut self,
        session_generation: u64,
        points: &[Point3<f64>],
    ) -> bool {
        if session_generation == 0
            || points.is_empty()
            || points.len() > POINTCLOUD_MAX_FRAME_POINTS
        {
            return false;
        }
        if self.session_generation != session_generation {
            self.reset_accumulator(session_generation);
        }

        let mut accepted = 0_usize;
        for point in points {
            let point = [point.x, point.y, point.z];
            let Some(key) = voxel_key(point, self.voxel_size) else {
                continue;
            };
            if self.voxels.insert(key) {
                self.ordered_voxels.inserted(key, self.max_points);
            }
            accepted += 1;
        }
        if accepted == 0 {
            return false;
        }
        self.coarsen_to_fit()
    }

    /// 原生两种里程计共用：先撤销实测空闲中的历史点，再加入当前全部障碍。
    pub(crate) fn accumulate_mapping_scan(
        &mut self,
        session_generation: u64,
        points: &[Point3<f64>],
        sensor_origin_map: Option<[f64; 3]>,
        source_timestamp_ns: u64,
    ) -> bool {
        if points.is_empty()
            || points.len() > POINTCLOUD_MAX_FRAME_POINTS
            || session_generation == 0
            || sensor_origin_map.is_some_and(|origin| voxel_key(origin, self.voxel_size).is_none())
        {
            return false;
        }
        // 一次计算全部体素键，同时完成有限值/范围校验；失败前不改变任何累加状态。
        let key_voxel_size = self.voxel_size;
        let Some(mut hits) = points
            .iter()
            .map(|point| voxel_key([point.x, point.y, point.z], self.voxel_size))
            .collect::<Option<HashSet<_>>>()
        else {
            return false;
        };
        if self.session_generation != session_generation {
            self.reset_accumulator(session_generation);
            if self.voxel_size != key_voxel_size {
                hits = points
                    .iter()
                    .filter_map(|point| voxel_key([point.x, point.y, point.z], self.voxel_size))
                    .collect();
            }
        }
        if self.fine_cloud.is_none() {
            match FineCloud::fresh() {
                Ok(fine) => self.fine_cloud = Some(fine),
                Err(error) => { warn!(%error, "精细导航源创建失败"); return false; }
            }
        }
        if let Err(error) = self.fine_cloud.as_mut().unwrap().accumulate(points, sensor_origin_map, source_timestamp_ns) {
            warn!(%error, "精细导航源拒绝不完整输入");
            return false;
        }
        if let Some(origin) = sensor_origin_map {
            let removed = self.dynamic_filter.clear_observed_free_space_with_hits(
                &mut self.voxels,
                self.voxel_size,
                origin,
                source_timestamp_ns,
                points,
                &hits,
            );
            if removed > 0 {
                for &key in self.dynamic_filter.removed_keys() {
                    self.ordered_voxels.removed(key, self.max_points);
                }
                tracing::debug!(removed, source_timestamp_ns, "三维地图已清除动态残留体素");
            }
        }
        if self.voxel_size != key_voxel_size {
            // 切换自适应粗化会话时保留原接纳/过滤语义，常规扫描复用已经验证的键。
            return self.accumulate_mapping_frame(session_generation, points);
        }
        for key in hits {
            if self.voxels.insert(key) {
                self.ordered_voxels.inserted(key, self.max_points);
            }
        }
        self.coarsen_to_fit()
    }

    /// Serialize the current voxel centroids into the double-buffered SHM.
    /// The caller invokes this only when the corresponding occupancy snapshot
    /// committed, so high-rate lidar ingestion only performs incremental hash
    /// updates between map revisions.
    pub fn commit_accumulated(&mut self) -> Option<(u64, u32, f32)> {
        self.publish_snapshot(false)
    }

    /// Commit the cumulative voxel map and expose a deterministic packed XYZ
    /// snapshot for the corresponding typed `/global_map` publication.
    pub(crate) fn commit_accumulated_snapshot(
        &mut self,
        occupancy_receipt: &OccupancyCommitReceipt,
    ) -> Result<Option<CorrelatedCloudMapCommit>, String> {
        self.validate_occupancy_receipt(occupancy_receipt)?;
        let fine = if occupancy_receipt.source_timestamp_ns == 0 { None }
            else { self.fine_cloud.as_mut().map(|fine| fine.seal(occupancy_receipt.source_timestamp_ns)).transpose()? };
        if self.commit_accumulated().is_none() {
            return Ok(None);
        }
        let snapshot = self
            .committed_snapshot()
            .ok_or_else(|| "committed cloud-map snapshot is inconsistent".to_owned())?;
        let receipt = CloudCommitReceipt {
            commit_id: occupancy_receipt.commit_id,
            session_generation: occupancy_receipt.session_generation,
            occupancy_revision: occupancy_receipt.occupancy_revision,
            cloud_revision: snapshot.revision,
            source_timestamp_ns: occupancy_receipt.source_timestamp_ns,
            point_count: snapshot.point_count,
        };
        self.last_occupancy_receipt = Some(occupancy_receipt.clone());
        self.last_cloud_receipt = Some(receipt.clone());
        self.committed_fine_cloud = fine;
        Ok(Some(CorrelatedCloudMapCommit { snapshot, receipt }))
    }

    /// Read the exact active SHM payload without advancing its revision.
    pub(crate) fn committed_snapshot(&self) -> Option<CommittedCloudMapSnapshot> {
        if self.revision == 0 || self.session_generation == 0 {
            return None;
        }
        let active_buf = u32::from_le_bytes(
            self.mmap[CM_ACTIVE_BUFFER_OFFSET..CM_ACTIVE_BUFFER_OFFSET + 4]
                .try_into()
                .ok()?,
        );
        if Some(active_buf) != cloud_map_active_slot(self.revision) {
            return None;
        }
        let active_buf = usize::try_from(active_buf).ok()?;
        let (slot_revision_offset, slot_point_count_offset) = slot_metadata_offsets(active_buf)?;
        let global_revision = u64::from_le_bytes(
            self.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8]
                .try_into()
                .ok()?,
        );
        let slot_revision = u64::from_le_bytes(
            self.mmap[slot_revision_offset..slot_revision_offset + 8]
                .try_into()
                .ok()?,
        );
        if global_revision != self.revision || slot_revision != self.revision {
            return None;
        }
        let point_count = u32::from_le_bytes(
            self.mmap[slot_point_count_offset..slot_point_count_offset + 4]
                .try_into()
                .ok()?,
        );
        let global_point_count = u32::from_le_bytes(
            self.mmap[CM_GLOBAL_POINT_COUNT_OFFSET..CM_GLOBAL_POINT_COUNT_OFFSET + 4]
                .try_into()
                .ok()?,
        );
        if global_point_count != point_count {
            return None;
        }
        let voxel_size = f32::from_le_bytes(
            self.mmap[CM_VOXEL_SIZE_OFFSET..CM_VOXEL_SIZE_OFFSET + 4]
                .try_into()
                .ok()?,
        );
        if voxel_size.to_bits() != self.voxel_size.to_bits() {
            return None;
        }
        let data_size = usize::try_from(point_count)
            .ok()?
            .checked_mul(CM_POINT_SIZE_BYTES)?;
        if data_size > CM_SINGLE_BUFFER_SIZE {
            return None;
        }
        let start = CM_HEADER_SIZE + active_buf * CM_SINGLE_BUFFER_SIZE;
        let end = start.checked_add(data_size)?;
        let xyz_f32_le = self.mmap.get(start..end)?.to_vec();
        let xyz_sha256 = crate::map_checksum::sha256_bytes(&xyz_f32_le);
        Some(CommittedCloudMapSnapshot {
            revision: self.revision,
            point_count,
            voxel_size,
            session_generation: self.session_generation,
            xyz_f32_le,
            xyz_sha256,
        })
    }

    pub(crate) fn last_cloud_receipt(&self) -> Option<CloudCommitReceipt> {
        self.last_cloud_receipt.clone()
    }

    pub(crate) fn committed_fine_cloud(&self) -> Option<Arc<nuwax_map_chunks::SealedMap>> {
        self.committed_fine_cloud.clone()
    }

    /// Fence a newly started mapping session from the prior session's 3D map.
    /// A successful call publishes a new revision with a slot-local point count
    /// of zero. Replaying the same generation is idempotent.
    pub(crate) fn begin_session(&mut self, session_generation: u64) -> Option<(u64, u32, f32)> {
        if session_generation == 0 || self.session_generation == session_generation {
            return None;
        }
        self.reset_accumulator(session_generation);
        let (revision, point_count, voxel_size) = self.publish_snapshot(true)?;
        self.typed_publish_state
            .reserve_session_clear(PendingTypedCloudClear {
                revision,
                point_count,
                voxel_size,
                session_generation,
            });
        Some((revision, point_count, voxel_size))
    }

    pub(crate) fn validate_checkpoint(
        &self,
        voxel_size: f32,
        point_count: u32,
        xyz_f32_le: &[u8],
    ) -> Result<(), String> {
        decode_checkpoint_voxels(voxel_size, point_count, xyz_f32_le, self.max_points).map(|_| ())
    }

    pub(crate) fn take_prepared_accumulator(&mut self) -> PreparedCloudAccumulator {
        PreparedCloudAccumulator {
            voxels: std::mem::take(&mut self.voxels),
            voxel_size: self.voxel_size,
            dynamic_filter: std::mem::take(&mut self.dynamic_filter),
            ordered_voxels: std::mem::take(&mut self.ordered_voxels),
            fine_cloud: self.fine_cloud.take(),
        }
    }

    pub(crate) fn install_prepared_accumulator(
        &mut self,
        receipt: &OccupancyCommitReceipt,
        prepared: PreparedCloudAccumulator,
    ) -> Result<CorrelatedCloudMapCommit, String> {
        self.revision
            .checked_add(1)
            .ok_or("cloud-map revision exhausted")?;
        self.validate_occupancy_receipt_for_generation(receipt, receipt.session_generation)?;
        if prepared.voxels.is_empty() || prepared.voxels.len() > self.max_points {
            return Err("PGO点云状态超过容量".into());
        }
        self.session_generation = receipt.session_generation;
        self.voxel_size = prepared.voxel_size;
        self.voxels = prepared.voxels;
        self.dynamic_filter = prepared.dynamic_filter;
        self.ordered_voxels = prepared.ordered_voxels;
        self.fine_cloud = prepared.fine_cloud;
        self.commit_accumulated_snapshot(receipt)?
            .ok_or_else(|| "PGO点云发布失败".into())
    }

    /// Restore one persisted map-frame cloud checkpoint into the live voxel
    /// accumulator and publish it as the first non-empty revision of the new
    /// mapping generation. Subsequent lidar frames continue from these voxels.
    pub(crate) fn install_checkpoint(
        &mut self,
        occupancy_receipt: &OccupancyCommitReceipt,
        voxel_size: f32,
        point_count: u32,
        xyz_f32_le: &[u8],
    ) -> Result<CorrelatedCloudMapCommit, String> {
        self.install_checkpoint_with_fine(occupancy_receipt, voxel_size, point_count, xyz_f32_le, None)
    }

    pub(crate) fn install_checkpoint_with_fine(
        &mut self, occupancy_receipt: &OccupancyCommitReceipt, voxel_size: f32, point_count: u32,
        xyz_f32_le: &[u8], fine: Option<PreparedFineCloud>,
    ) -> Result<CorrelatedCloudMapCommit, String> {
        let session_generation = occupancy_receipt.session_generation;
        if session_generation == 0 {
            return Err("cloud-map checkpoint generation must be positive".to_string());
        }
        self.revision
            .checked_add(1)
            .ok_or_else(|| "cloud-map revision exhausted".to_string())?;
        let voxels =
            decode_checkpoint_voxels(voxel_size, point_count, xyz_f32_le, self.max_points)?;
        self.validate_occupancy_receipt_for_generation(occupancy_receipt, session_generation)?;
        let fine = match fine {
            Some(prepared) => prepared.cloud,
            None => {
                let mut fine = FineCloud::new(voxel_size)?;
                let points = xyz_f32_le.chunks_exact(12).map(|bytes| Point3::new(
                    f32::from_le_bytes(bytes[0..4].try_into().unwrap()) as f64,
                    f32::from_le_bytes(bytes[4..8].try_into().unwrap()) as f64,
                    f32::from_le_bytes(bytes[8..12].try_into().unwrap()) as f64,
                )).collect::<Vec<_>>();
                // 旧检查点只有历史几何，恢复提交时间不能冒充逐点观察时间。
                fine.accumulate(&points, None, 0)?;
                fine
            }
        };
        self.session_generation = session_generation;
        self.voxel_size = voxel_size;
        self.voxels = voxels.into();
        self.ordered_voxels.invalidate();
        self.dynamic_filter = DynamicCloudFilter::default();
        self.fine_cloud = Some(fine);
        self.commit_accumulated_snapshot(occupancy_receipt)?
            .ok_or_else(|| "cloud-map checkpoint publication failed".to_string())
    }

    fn publish_snapshot(&mut self, allow_empty: bool) -> Option<(u64, u32, f32)> {
        if self.session_generation == 0 || (!allow_empty && self.voxels.is_empty()) {
            return None;
        }
        let point_count = self.voxels.len();
        let data_size = point_count.checked_mul(CM_POINT_SIZE_BYTES)?;
        if point_count > self.max_points || data_size > CM_SINGLE_BUFFER_SIZE {
            warn!(
                "cloud_map voxel snapshot exceeds SHM capacity: points={} bytes={} max_points={} max_bytes={}",
                point_count,
                data_size,
                self.max_points,
                CM_SINGLE_BUFFER_SIZE
            );
            return None;
        }
        let point_count = u32::try_from(point_count).ok()?;
        let revision = self.revision.checked_add(1)?;
        let active_buf = usize::try_from(cloud_map_active_slot(revision)?).ok()?;
        let buf_offset = CM_HEADER_SIZE + active_buf * CM_SINGLE_BUFFER_SIZE;
        let (slot_revision_offset, slot_point_count_offset) = slot_metadata_offsets(active_buf)?;

        // Invalidate the selected slot before overwriting either its data or count.
        // Readers only accept a slot whose revision matches the final global revision.
        self.mmap[slot_revision_offset..slot_revision_offset + 8]
            .copy_from_slice(&0_u64.to_le_bytes());
        self.mmap[slot_point_count_offset..slot_point_count_offset + 4]
            .copy_from_slice(&0_u32.to_le_bytes());
        // The invalid marker must become globally ordered before any payload
        // bytes in this slot are overwritten by the new snapshot.
        fence(Ordering::SeqCst);
        let keys = self.ordered_voxels.ordered(&self.voxels);
        for (index, &key) in keys.iter().enumerate() {
            let point = voxel_center_f32(key, self.voxel_size);
            let offset = buf_offset + index * CM_POINT_SIZE_BYTES;
            self.mmap[offset..offset + 4].copy_from_slice(&point[0].to_le_bytes());
            self.mmap[offset + 4..offset + 8].copy_from_slice(&point[1].to_le_bytes());
            self.mmap[offset + 8..offset + 12].copy_from_slice(&point[2].to_le_bytes());
        }
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        self.mmap[slot_point_count_offset..slot_point_count_offset + 4]
            .copy_from_slice(&point_count.to_le_bytes());
        // Publish the slot-local tuple only after its XYZ payload and count are visible.
        fence(Ordering::Release);
        self.mmap[slot_revision_offset..slot_revision_offset + 8]
            .copy_from_slice(&revision.to_le_bytes());

        // Global metadata is diagnostic/compatibility state. Global revision is
        // the final publication field and therefore cannot pair an old revision
        // with a newly overwritten slot count.
        fence(Ordering::Release);
        self.mmap[CM_ACTIVE_BUFFER_OFFSET..CM_ACTIVE_BUFFER_OFFSET + 4]
            .copy_from_slice(&(active_buf as u32).to_le_bytes());
        self.mmap[CM_GLOBAL_POINT_COUNT_OFFSET..CM_GLOBAL_POINT_COUNT_OFFSET + 4]
            .copy_from_slice(&point_count.to_le_bytes());
        self.mmap[CM_TIMESTAMP_NS_OFFSET..CM_TIMESTAMP_NS_OFFSET + 8]
            .copy_from_slice(&now_ns.to_le_bytes());
        self.mmap[CM_VOXEL_SIZE_OFFSET..CM_VOXEL_SIZE_OFFSET + 4]
            .copy_from_slice(&self.voxel_size.to_le_bytes());
        fence(Ordering::Release);
        self.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8]
            .copy_from_slice(&revision.to_le_bytes());
        self.revision = revision;
        // 这是启动时重新初始化的进程间通信映射；上方发布屏障保证可见性。
        // 地图资产有独立持久化流程，实时提交不等待整段文件同步写盘。
        Some((revision, point_count, self.voxel_size))
    }

    fn reset_accumulator(&mut self, session_generation: u64) {
        self.fine_cloud = None;
        self.committed_fine_cloud = None;
        self.voxels.clear();
        self.ordered_voxels.invalidate();
        self.dynamic_filter = DynamicCloudFilter::default();
        self.voxel_size = VOXEL_SIZE;
        self.session_generation = session_generation;
        self.last_occupancy_receipt = None;
        self.last_cloud_receipt = None;
    }

    fn validate_occupancy_receipt(&self, receipt: &OccupancyCommitReceipt) -> Result<(), String> {
        self.validate_occupancy_receipt_for_generation(receipt, self.session_generation)
    }

    fn validate_occupancy_receipt_for_generation(
        &self,
        receipt: &OccupancyCommitReceipt,
        session_generation: u64,
    ) -> Result<(), String> {
        if receipt.commit_id.boot_nonce == 0
            || receipt.commit_id.sequence == 0
            || receipt.session_id.trim().is_empty()
            || receipt.session_generation == 0
            || receipt.occupancy_revision == 0
            || receipt.backend_generation == 0
        {
            return Err("cloud-map occupancy receipt identity is incomplete".to_owned());
        }
        if receipt.session_generation != session_generation {
            return Err(format!(
                "cloud-map occupancy receipt generation mismatch: receipt={} writer={}",
                receipt.session_generation, session_generation
            ));
        }
        if let Some(previous) = self.last_occupancy_receipt.as_ref() {
            if receipt.commit_id.boot_nonce != previous.commit_id.boot_nonce
                || receipt.commit_id.sequence <= previous.commit_id.sequence
                || receipt.occupancy_revision <= previous.occupancy_revision
            {
                return Err("cloud-map occupancy receipt is duplicate or out of order".to_owned());
            }
        }
        Ok(())
    }

    fn coarsen_to_fit(&mut self) -> bool {
        if self.max_points == 0 {
            self.voxels.clear();
            self.ordered_voxels.invalidate();
            return false;
        }
        while self.voxels.len() > self.max_points {
            let next_voxel_size = self.voxel_size * 2.0;
            if !next_voxel_size.is_finite() || next_voxel_size > MAX_VOXEL_SIZE {
                warn!(
                    "cloud_map voxel accumulator cannot fit SHM safely: voxels={} max_points={} voxel_size={}",
                    self.voxels.len(),
                    self.max_points,
                    self.voxel_size
                );
                self.voxels.clear();
                self.ordered_voxels.invalidate();
                self.session_generation = 0;
                return false;
            }
            let previous_voxel_size = self.voxel_size;
            let previous = std::mem::take(&mut self.voxels);
            let mut coarsened = VoxelIndex::with_capacity(previous.len().min(self.max_points));
            for previous_key in previous {
                let point = voxel_center_f64(previous_key, previous_voxel_size);
                let Some(key) = voxel_key(point, next_voxel_size) else {
                    continue;
                };
                coarsened.insert(key);
            }
            self.voxels = coarsened;
            self.ordered_voxels.invalidate();
            self.voxel_size = next_voxel_size;
            // 体素身份改变后，不把细网格的反证套到新的粗网格。
            self.dynamic_filter = DynamicCloudFilter::default();
        }
        !self.voxels.is_empty()
    }
}

fn decode_checkpoint_voxels(
    voxel_size: f32,
    point_count: u32,
    xyz_f32_le: &[u8],
    max_points: usize,
) -> Result<HashSet<VoxelKey>, String> {
    if !voxel_size.is_finite() || voxel_size <= 0.0 || voxel_size > MAX_VOXEL_SIZE {
        return Err("cloud-map checkpoint voxel_size must be finite and positive".to_string());
    }
    let point_count = usize::try_from(point_count)
        .map_err(|_| "cloud-map checkpoint point_count exceeds usize".to_string())?;
    if point_count == 0 {
        return Err("cloud-map checkpoint must contain at least one point".to_string());
    }
    if point_count > max_points || point_count > CM_MAX_POINTS {
        return Err(format!(
            "cloud-map checkpoint exceeds SHM capacity: points={point_count} max_points={}",
            max_points.min(CM_MAX_POINTS)
        ));
    }
    let expected_size = point_count
        .checked_mul(CM_POINT_SIZE_BYTES)
        .ok_or_else(|| "cloud-map checkpoint payload size overflow".to_string())?;
    if xyz_f32_le.len() != expected_size {
        return Err(format!(
            "cloud-map checkpoint payload size mismatch: expected={expected_size} actual={}",
            xyz_f32_le.len()
        ));
    }

    let mut voxels = HashSet::with_capacity(point_count);
    for (index, bytes) in xyz_f32_le.chunks_exact(CM_POINT_SIZE_BYTES).enumerate() {
        let point = [
            f32::from_le_bytes(bytes[0..4].try_into().unwrap()) as f64,
            f32::from_le_bytes(bytes[4..8].try_into().unwrap()) as f64,
            f32::from_le_bytes(bytes[8..12].try_into().unwrap()) as f64,
        ];
        let key = voxel_key(point, voxel_size).ok_or_else(|| {
            format!("cloud-map checkpoint point {index} is non-finite or out of range")
        })?;
        voxels.insert(key);
    }
    if voxels.len() != point_count {
        return Err(format!(
            "cloud-map checkpoint contains duplicate voxels: declared={point_count} unique={}",
            voxels.len()
        ));
    }
    if voxels.is_empty() {
        return Err("cloud-map checkpoint contains no usable voxels".to_string());
    }
    Ok(voxels)
}

fn slot_metadata_offsets(slot: usize) -> Option<(usize, usize)> {
    match slot {
        0 => Some((CM_SLOT0_REVISION_OFFSET, CM_SLOT0_POINT_COUNT_OFFSET)),
        1 => Some((CM_SLOT1_REVISION_OFFSET, CM_SLOT1_POINT_COUNT_OFFSET)),
        _ => None,
    }
}

fn voxel_center_f64(key: VoxelKey, voxel_size: f32) -> [f64; 3] {
    let voxel_size = f64::from(voxel_size);
    [
        (f64::from(key.0) + 0.5) * voxel_size,
        (f64::from(key.1) + 0.5) * voxel_size,
        (f64::from(key.2) + 0.5) * voxel_size,
    ]
}

fn voxel_center_f32(key: VoxelKey, voxel_size: f32) -> [f32; 3] {
    let point = voxel_center_f64(key, voxel_size);
    [point[0] as f32, point[1] as f32, point[2] as f32]
}

fn voxel_key(point: [f64; 3], voxel_size: f32) -> Option<VoxelKey> {
    if !voxel_size.is_finite() || voxel_size <= 0.0 || point.iter().any(|value| !value.is_finite())
    {
        return None;
    }
    let inv = 1.0 / f64::from(voxel_size);
    let mut key = [0_i32; 3];
    for (index, value) in point.into_iter().enumerate() {
        let scaled = (value * inv).floor();
        if scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
            return None;
        }
        key[index] = scaled as i32;
    }
    Some((key[0], key[1], key[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fine_cloud_survives_display_coarsening_and_seals_the_exact_commit() {
        let path = test_path("fine-cloud-coarsening");
        let mut writer = CloudMapShmWriter::new(&path).unwrap();
        writer.max_points = 4;
        let points = (0..30).map(|x| Point3::new(f64::from(x) * 0.051, 0.025, 0.175)).collect::<Vec<_>>();
        assert!(writer.accumulate_mapping_scan(1, &points, None, 100));
        let first_receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 17, sequence: 1 },
            session_id: "精细源验证".into(), session_generation: 1, occupancy_revision: 1,
            source_timestamp_ns: 100, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        let display = writer.commit_accumulated_snapshot(&first_receipt).unwrap().unwrap();
        assert!(display.snapshot.voxel_size > VOXEL_SIZE);
        assert!(display.snapshot.point_count <= 4);
        let fine = writer.committed_fine_cloud().unwrap();
        let domain = nuwax_map_chunks::Domain { session: "精细源验证".into(), generation: 1, frame_generation: 0, producer_generation: 1 };
        let root = fine.manifest(domain.clone(), 1).unwrap();
        assert_eq!(root.voxel_size_m, VOXEL_SIZE);
        assert_eq!(root.points(), 30);
        let mut reader = nuwax_map_chunks::RegionReader::new(fine.repository.clone(), 4 * 1024 * 1024);
        let saved = reader.region(&root, [-100; 3], [100; 3], 100).unwrap();
        assert!(saved.iter().all(|(key, evidence)| key[2] == 3 && evidence.observed_ns == 100));
        assert!(writer.accumulate_mapping_scan(1, &[Point3::new(-0.1, 0.025, 0.325)], None, 200));
        assert!(Arc::ptr_eq(&fine, &writer.committed_fine_cloud().unwrap()));
        assert_eq!(reader.region(&root, [-100; 3], [100; 3], 100).unwrap(), saved);
        let receipt = OccupancyCommitReceipt { commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 17, sequence: 2 },
            occupancy_revision: 2, source_timestamp_ns: 200, ..first_receipt };
        writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
        let next = writer.committed_fine_cloud().unwrap();
        assert_eq!(next.manifest(domain, 2).unwrap().points(), 31);
        writer.cleanup_product_owned_file().unwrap();
    }

    #[test]
    fn pgo_accumulator_install_preserves_pending_dynamic_evidence() {
        let mut source = CloudMapShmWriter::new(&test_path("pgo-private-dynamic")).unwrap();
        let mut target = CloudMapShmWriter::new(&test_path("pgo-installed-dynamic")).unwrap();
        let origin = Some([0.025; 3]);
        let person = [Point3::new(2.025, 0.025, 0.025)];
        let background = [Point3::new(4.025, 0.025, 0.025)];
        source.accumulate_mapping_scan(1, &person, origin, 1);
        source.accumulate_mapping_scan(1, &background, origin, 2);
        source.accumulate_mapping_scan(1, &background, origin, 3);
        assert!(source.voxels.contains(&(40, 0, 0)));
        let receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 1,
                sequence: 1,
            },
            session_id: "pgo-dynamic".into(),
            session_generation: 2,
            occupancy_revision: 1,
            source_timestamp_ns: 3,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        target
            .install_prepared_accumulator(&receipt, source.take_prepared_accumulator())
            .unwrap();
        assert!(target.voxels.contains(&(40, 0, 0)));
        // 私有重建已有两次有效反证；安装后的第三次必须直接删除，不能重新计票。
        assert!(target.accumulate_mapping_scan(2, &background, origin, 4));
        assert!(!target.voxels.contains(&(40, 0, 0)));
        source.cleanup_product_owned_file().unwrap();
        target.cleanup_product_owned_file().unwrap();
    }

    #[test]
    fn dynamic_cloud_checkpoint_and_session_reset_keep_only_committed_geometry() {
        let path = test_path("dynamic-checkpoint");
        let mut writer = CloudMapShmWriter::new(&path).unwrap();
        let origin = Some([0.025; 3]);
        let person = [Point3::new(2.025, 0.025, 0.025)];
        let background = [Point3::new(4.025, 0.025, 0.025)];
        assert!(writer.accumulate_mapping_scan(1, &person, origin, 1));
        for stamp in 2..=3 {
            assert!(writer.accumulate_mapping_scan(1, &background, origin, stamp));
        }
        let before = writer.voxels.clone();
        assert!(!writer.accumulate_mapping_scan(1, &background, Some([f64::NAN; 3]), 4));
        assert!(!writer.accumulate_mapping_scan(1, &[Point3::new(f64::NAN, 0.0, 0.0)], origin, 4));
        assert!(writer.accumulate_mapping_scan(1, &background, None, 4));
        assert_eq!(writer.voxels, before);
        writer.commit_accumulated().unwrap();
        let checkpoint = writer.committed_snapshot().unwrap();
        let receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 1,
                sequence: 1,
            },
            session_id: "dynamic-restored".to_owned(),
            session_generation: 2,
            occupancy_revision: 1,
            source_timestamp_ns: 5,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        writer
            .install_checkpoint(
                &receipt,
                checkpoint.voxel_size,
                checkpoint.point_count,
                &checkpoint.xyz_f32_le,
            )
            .unwrap();
        // 检查点不携带未完成的删除投票，恢复后重新获得三次反证。
        for stamp in 6..=8 {
            assert!(writer.accumulate_mapping_scan(2, &background, origin, stamp));
            assert_eq!(writer.voxels.contains(&(40, 0, 0)), stamp < 8);
        }
        writer.commit_accumulated().unwrap();
        let cleaned = writer.committed_snapshot().unwrap();
        assert_eq!(cleaned.point_count, 1);
        assert_eq!(
            decode_checkpoint_voxels(
                cleaned.voxel_size,
                cleaned.point_count,
                &cleaned.xyz_f32_le,
                CM_MAX_POINTS
            )
            .unwrap(),
            *writer.voxels
        );
        writer.begin_session(3).unwrap();
        assert!(writer.accumulate_mapping_scan(3, &person, origin, 1));
        assert!(writer.accumulate_mapping_scan(3, &background, origin, 2));
        assert!(writer.voxels.contains(&(40, 0, 0)));
        drop(writer);
        std::fs::remove_file(path).unwrap();
    }

    fn test_path(label: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "nuwax-cloudmap-{label}-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ))
    }

    fn read_u32(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    fn read_u64(bytes: &[u8], offset: usize) -> u64 {
        u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
    }

    #[test]
    fn product_cleanup_unlinks_owned_cloud_mmap() {
        let path = test_path("owned-cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");

        writer
            .cleanup_product_owned_file()
            .expect("owned cloud mmap cleanup");

        assert!(!path.exists());
    }

    #[test]
    fn product_cleanup_rejects_replaced_cloud_mmap() {
        let path = test_path("replaced-cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        std::fs::remove_file(&path).expect("remove owned name");
        std::fs::write(&path, b"foreign replacement").expect("install replacement");

        let error = writer
            .cleanup_product_owned_file()
            .expect_err("replacement must be preserved");

        assert!(error
            .to_string()
            .contains("refusing to unlink replaced mmap path"));
        assert_eq!(
            std::fs::read(&path).expect("replacement remains"),
            b"foreign replacement"
        );
        std::fs::remove_file(path).expect("remove replacement");
    }

    #[test]
    fn product_cleanup_is_idempotent_when_cloud_mmap_is_missing() {
        let path = test_path("missing-cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        std::fs::remove_file(&path).expect("remove owned name");

        writer
            .cleanup_product_owned_file()
            .expect("first missing cleanup");
        writer
            .cleanup_product_owned_file()
            .expect("repeated missing cleanup");
    }

    #[test]
    fn immutable_target_decoder_retains_exact_bytes_and_accepts_legacy_hex_formatting() {
        let xyz_f32_le = [1.0_f32, -2.0, 3.5]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let digest = format!(
            " \t{}\r\n",
            crate::map_checksum::sha256_hex(&xyz_f32_le).to_ascii_uppercase()
        );
        let target = decode_immutable_activated_target(
            "map",
            "map",
            9,
            1,
            0.1,
            &xyz_f32_le,
            &digest,
            CM_PAYLOAD_SCHEMA_VERSION,
        )
        .expect("strict immutable target");
        assert_eq!(target.xyz_f32_le, xyz_f32_le);
        assert_eq!(
            target.xyz_sha256,
            crate::map_checksum::sha256_bytes(&target.xyz_f32_le)
        );
    }

    #[test]
    fn immutable_target_decoder_rejects_malformed_and_unbounded_payloads() {
        let finite = [1.0_f32, 2.0, 3.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let digest = crate::map_checksum::sha256_hex(&finite);
        let decode = |frame: &str,
                      revision: u64,
                      count: u32,
                      voxel: f32,
                      bytes: &[u8],
                      checksum: &str,
                      schema: u32| {
            decode_immutable_activated_target(
                frame, "map", revision, count, voxel, bytes, checksum, schema,
            )
        };
        assert!(decode("/map", 9, 1, 0.1, &finite, &digest, 1).is_err());
        assert!(decode("map", 0, 1, 0.1, &finite, &digest, 1).is_err());
        assert!(decode("map", 9, 0, 0.1, &[], &digest, 1).is_err());
        assert!(decode("map", 9, 1, -0.0, &finite, &digest, 1).is_err());
        assert!(decode("map", 9, 1, 0.1, &finite, &digest, 2).is_err());
        assert!(decode("map", 9, 1, 0.1, &finite, &"00".repeat(32), 1).is_err());

        let mut trailing = finite.clone();
        trailing.push(0);
        let trailing_digest = crate::map_checksum::sha256_hex(&trailing);
        assert!(decode("map", 9, 1, 0.1, &trailing, &trailing_digest, 1).is_err());

        let nonfinite = [f32::INFINITY, 2.0, 3.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let nonfinite_digest = crate::map_checksum::sha256_hex(&nonfinite);
        assert!(decode("map", 9, 1, 0.1, &nonfinite, &nonfinite_digest, 1).is_err());

        let over_limit_count =
            u32::try_from(IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES / CM_POINT_SIZE_BYTES + 1).unwrap();
        assert!(decode("map", 9, over_limit_count, 0.1, &[], &"00".repeat(32), 1).is_err());
    }

    #[test]
    fn incremental_frames_advance_the_shared_memory_revision() {
        let path = test_path("mapping-session");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        let first_frame = [Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.01, 0.01)];
        let second_frame = [Point3::new(1.0, 2.0, 3.0)];

        assert!(writer.accumulate_mapping_frame(1, &first_frame));
        let first = writer
            .commit_accumulated()
            .expect("first accumulated snapshot");
        assert_eq!(first.0, 1);
        assert_eq!(first.1, 1, "nearby points share the 5 cm voxel");
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, 8), CM_SHM_VERSION);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT0_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT0_POINT_COUNT_OFFSET), 1);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT1_REVISION_OFFSET), 0);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT1_POINT_COUNT_OFFSET), 0);

        assert!(writer.accumulate_mapping_frame(1, &second_frame));
        let second = writer
            .commit_accumulated()
            .expect("second accumulated snapshot");
        assert_eq!(second.0, 2);
        assert_eq!(second.1, 2);
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_ACTIVE_BUFFER_OFFSET), 1);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT1_REVISION_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT1_POINT_COUNT_OFFSET), 2);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn begin_session_publishes_one_zero_frame_per_generation() {
        let path = test_path("session-clear");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");

        let first_clear = writer.begin_session(1).expect("first session clear");
        assert_eq!(first_clear, (1, 0, VOXEL_SIZE));
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 0);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT0_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT0_POINT_COUNT_OFFSET), 0);
        assert!(writer.begin_session(1).is_none());
        assert_eq!(writer.revision, 1, "same generation must be idempotent");

        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(0.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap().0, 2);
        let second_clear = writer.begin_session(2).expect("second session clear");
        assert_eq!(second_clear, (3, 0, VOXEL_SIZE));
        assert!(writer.voxels.is_empty());
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 0);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT0_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT0_POINT_COUNT_OFFSET), 0);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn restored_checkpoint_accepts_later_live_frames() {
        let path = test_path("checkpoint-restore");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        let occupancy_receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 1,
                sequence: 1,
            },
            session_id: "checkpoint-session".to_owned(),
            session_generation: 7,
            occupancy_revision: 11,
            source_timestamp_ns: 13,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        let checkpoint_points = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]];
        let xyz_f32_le = checkpoint_points
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();

        let restored = writer
            .install_checkpoint(&occupancy_receipt, VOXEL_SIZE, 2, &xyz_f32_le)
            .expect("restore checkpoint");
        assert_eq!(restored.snapshot.revision, 1);
        assert_eq!(restored.snapshot.point_count, 2);
        assert_eq!(restored.snapshot.voxel_size, VOXEL_SIZE);
        assert_eq!(
            restored.snapshot.xyz_sha256,
            crate::map_checksum::sha256_bytes(&restored.snapshot.xyz_f32_le)
        );
        assert_eq!(restored.receipt.commit_id, occupancy_receipt.commit_id);
        assert_eq!(restored.receipt.occupancy_revision, 11);
        assert_eq!(restored.receipt.cloud_revision, 1);
        assert_eq!(writer.session_generation, 7);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 2);

        // 激活回执有时间，但没有精细资产的旧点必须保留未知观察时间。
        let fine = writer.committed_fine_cloud().unwrap();
        let root = fine.manifest(nuwax_map_chunks::Domain {
            session: occupancy_receipt.session_id.clone(), generation: 7,
            frame_generation: 0, producer_generation: 1,
        }, 1).unwrap();
        let mut reader = nuwax_map_chunks::RegionReader::new(fine.repository.clone(), 1024 * 1024);
        let evidence = reader.region(&root, [-100; 3], [100; 3], 10).unwrap();
        assert_eq!(evidence.len(), 2);
        assert!(evidence.iter().all(|(_, evidence)| evidence.observed_ns == 0 && evidence.hits == 0));

        assert!(writer.accumulate_mapping_frame(7, &[Point3::new(2.025, 0.025, 0.025)]));
        let continued = writer.commit_accumulated().expect("continued snapshot");
        assert_eq!(continued, (2, 3, VOXEL_SIZE));
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 3);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn rejected_checkpoint_receipt_does_not_mutate_writer_state() {
        let path = test_path("checkpoint-invalid-receipt");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        assert!(writer.accumulate_mapping_frame(3, &[Point3::new(0.0, 0.0, 0.0)]));
        let before_revision = writer.revision;
        let before_generation = writer.session_generation;
        let before_voxel_size = writer.voxel_size;
        let before_voxels = writer.voxels.clone();
        let before_global_revision = read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET);
        let invalid_receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 0,
                sequence: 1,
            },
            session_id: "checkpoint-session".to_owned(),
            session_generation: 7,
            occupancy_revision: 11,
            source_timestamp_ns: 13,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        let xyz_f32_le = [0.025_f32, 0.025, 0.025]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();

        let Err(error) =
            writer.install_checkpoint(&invalid_receipt, VOXEL_SIZE * 2.0, 1, &xyz_f32_le)
        else {
            panic!("invalid receipt must reject checkpoint");
        };

        assert!(error.contains("occupancy receipt identity is incomplete"));
        assert_eq!(writer.revision, before_revision);
        assert_eq!(writer.session_generation, before_generation);
        assert_eq!(writer.voxel_size, before_voxel_size);
        assert_eq!(writer.voxels, before_voxels);
        assert_eq!(
            read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET),
            before_global_revision
        );
        assert!(writer.last_occupancy_receipt.is_none());
        assert!(writer.last_cloud_receipt.is_none());

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn correlated_cloud_commit_rejects_duplicate_receipts() {
        let path = test_path("receipt-order");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        assert!(writer.accumulate_mapping_frame(7, &[Point3::new(0.0, 0.0, 0.0)]));
        let first_receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 19,
                sequence: 1,
            },
            session_id: "receipt-session".to_owned(),
            session_generation: 7,
            occupancy_revision: 41,
            source_timestamp_ns: 101,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 3,
        };

        let first = writer
            .commit_accumulated_snapshot(&first_receipt)
            .expect("valid receipt")
            .expect("first cloud commit");
        assert_eq!(first.receipt.commit_id, first_receipt.commit_id);
        assert_eq!(first.receipt.occupancy_revision, 41);
        assert_eq!(first.receipt.cloud_revision, 1);
        assert_ne!(
            first.receipt.occupancy_revision, first.receipt.cloud_revision,
            "occupancy and cloud revisions are independent domains"
        );

        assert!(writer.accumulate_mapping_frame(7, &[Point3::new(1.0, 0.0, 0.0)]));
        let revision_before_duplicate = writer.revision;
        assert!(writer.commit_accumulated_snapshot(&first_receipt).is_err());
        assert_eq!(writer.revision, revision_before_duplicate);

        let second_receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 19,
                sequence: 2,
            },
            occupancy_revision: 42,
            source_timestamp_ns: 102,
            ..first_receipt
        };
        let second = writer
            .commit_accumulated_snapshot(&second_receipt)
            .expect("next receipt")
            .expect("second cloud commit");
        assert_eq!(second.receipt.commit_id, second_receipt.commit_id);
        assert_eq!(second.receipt.occupancy_revision, 42);
        assert_eq!(second.receipt.cloud_revision, 2);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn session_switch_clears_voxels_without_resetting_revision() {
        let path = test_path("session-switch");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(0.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap().0, 1);

        assert!(writer.accumulate_mapping_frame(2, &[Point3::new(10.0, 0.0, 0.0)]));
        assert_eq!(writer.voxels.len(), 1);
        let second = writer.commit_accumulated().expect("new session snapshot");
        assert_eq!(second.0, 2);
        assert_eq!(second.1, 1);
        assert_eq!(writer.session_generation, 2);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn snapshots_coarsen_past_two_meters_instead_of_overrunning_the_buffer() {
        let path = test_path("capacity");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        writer.max_points = 1;
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
        ];

        assert!(writer.accumulate_mapping_frame(1, &points));
        assert!(writer.voxel_size > 2.0);
        assert!(writer.voxels.len() <= writer.max_points);
        let update = writer.commit_accumulated().expect("capacity-safe snapshot");
        assert_eq!(update.1, 1);
        assert!(usize::try_from(update.1).unwrap() * 12 <= CM_SINGLE_BUFFER_SIZE);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn v2_slot_count_shrinks_with_the_revision_that_publishes_it() {
        let path = test_path("v2-slot-metadata");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        let first_session = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        assert!(writer.accumulate_mapping_frame(1, &first_session));
        assert_eq!(writer.commit_accumulated().unwrap(), (1, 3, VOXEL_SIZE));

        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(3.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap(), (2, 4, VOXEL_SIZE));
        assert_eq!(read_u64(&writer.mmap, CM_SLOT0_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT0_POINT_COUNT_OFFSET), 3);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT1_REVISION_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT1_POINT_COUNT_OFFSET), 4);

        assert!(writer.accumulate_mapping_frame(2, &[Point3::new(10.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap(), (3, 1, VOXEL_SIZE));
        assert_eq!(read_u32(&writer.mmap, CM_ACTIVE_BUFFER_OFFSET), 0);
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 1);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT0_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT0_POINT_COUNT_OFFSET), 1);
        assert_eq!(read_u64(&writer.mmap, CM_SLOT1_REVISION_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_SLOT1_POINT_COUNT_OFFSET), 4);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn oversized_single_frame_does_not_change_writer_state() {
        let path = test_path("single-frame-cap");
        let mut writer = CloudMapShmWriter::new(&path).expect("cloud map writer");
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(0.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap().0, 1);
        let voxels_before = writer.voxels.clone();
        let oversized = vec![Point3::new(5.0, 5.0, 5.0); POINTCLOUD_MAX_FRAME_POINTS + 1];

        assert!(!writer.accumulate_mapping_frame(1, &oversized));
        assert_eq!(writer.voxels, voxels_before);
        assert_eq!(writer.revision, 1);
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 1);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }
}
