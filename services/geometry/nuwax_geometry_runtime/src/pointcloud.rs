use std::collections::HashSet;
use std::sync::atomic::{fence, Ordering};
use std::sync::Arc;

use memmap2::MmapMut;
use nalgebra::Point3;
use tracing::{error, info, warn};

use crate::mapping_commit::{CloudCommitReceipt, OccupancyCommitReceipt};

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
    revision: u64,
    voxel_size: f32,
    session_generation: u64,
    voxels: HashSet<VoxelKey>,
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
}

pub(crate) struct CorrelatedCloudMapCommit {
    pub(crate) snapshot: CommittedCloudMapSnapshot,
    pub(crate) receipt: CloudCommitReceipt,
}

impl CloudMapShmWriter {
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(shm_path)?;
        file.set_len(CM_TOTAL_SIZE as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        mmap[0..8].copy_from_slice(&CM_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&CM_SHM_VERSION.to_le_bytes());
        mmap[12..CM_HEADER_SIZE].fill(0);
        info!(
            "累积地图双缓冲 mmap 已创建: {:?} ({} bytes)",
            shm_path, CM_TOTAL_SIZE
        );
        Ok(Self {
            mmap,
            revision: 0,
            voxel_size: VOXEL_SIZE,
            session_generation: 0,
            voxels: HashSet::new(),
            max_points: CM_MAX_POINTS,
            last_occupancy_receipt: None,
            last_cloud_receipt: None,
            typed_publish_state: Arc::new(TypedCloudPublishState::new()),
        })
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
            self.voxels.insert(key);
            accepted += 1;
        }
        if accepted == 0 {
            return false;
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
        Some(CommittedCloudMapSnapshot {
            revision: self.revision,
            point_count,
            voxel_size,
            session_generation: self.session_generation,
            xyz_f32_le: self.mmap.get(start..end)?.to_vec(),
        })
    }

    pub(crate) fn last_cloud_receipt(&self) -> Option<CloudCommitReceipt> {
        self.last_cloud_receipt.clone()
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
        self.session_generation = session_generation;
        self.voxel_size = voxel_size;
        self.voxels = voxels;
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
        let mut keys = self.voxels.iter().copied().collect::<Vec<_>>();
        keys.sort_unstable();
        for (index, key) in keys.into_iter().enumerate() {
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
        if let Err(e) = self.mmap.flush_range(0, buf_offset + data_size) {
            error!("累积地图 mmap flush 失败: {}", e);
        }
        Some((revision, point_count, self.voxel_size))
    }

    fn reset_accumulator(&mut self, session_generation: u64) {
        self.voxels.clear();
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
                self.session_generation = 0;
                return false;
            }
            let previous_voxel_size = self.voxel_size;
            let previous = std::mem::take(&mut self.voxels);
            let mut coarsened = HashSet::with_capacity(previous.len().min(self.max_points));
            for previous_key in previous {
                let point = voxel_center_f64(previous_key, previous_voxel_size);
                let Some(key) = voxel_key(point, next_voxel_size) else {
                    continue;
                };
                coarsened.insert(key);
            }
            self.voxels = coarsened;
            self.voxel_size = next_voxel_size;
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
        assert_eq!(restored.receipt.commit_id, occupancy_receipt.commit_id);
        assert_eq!(restored.receipt.occupancy_revision, 11);
        assert_eq!(restored.receipt.cloud_revision, 1);
        assert_eq!(writer.session_generation, 7);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 2);

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
