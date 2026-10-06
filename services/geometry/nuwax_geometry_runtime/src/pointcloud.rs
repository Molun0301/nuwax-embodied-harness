use ahash::AHashSet as HashSet;
use std::io::{Error, ErrorKind};
use std::os::unix::fs::MetadataExt;
use std::path::Path;
use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};

use memmap2::MmapMut;
use nalgebra::Point3;
use tracing::warn;

use crate::mapping_commit::{CloudCommitReceipt, OccupancyCommitReceipt};
use crate::dense_cloud::{DenseCloud, DenseCloudReaders, DenseSnapshot};

#[path = "voxel_index.rs"]
mod voxel_index;
use voxel_index::VoxelIndex;

#[path = "dynamic_cloud.rs"]
mod dynamic_cloud;
use dynamic_cloud::DynamicCloudFilter;

#[path = "fine_cloud.rs"]
mod fine_cloud;
use fine_cloud::FineCloud;

#[path = "paged_voxel_snapshot.rs"]
mod paged_voxel_snapshot;
use paged_voxel_snapshot::{PagedVoxelSnapshot, CHUNK_POINTS};

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
pub(crate) const CM_SHM_VERSION: u32 = 3;
/// Tightly packed little-endian XYZ, three f32 values per point.
pub(crate) const CM_PAYLOAD_SCHEMA_VERSION: u32 = 1;
const CM_HEADER_SIZE: usize = 512;
pub(crate) const CM_SINGLE_BUFFER_SIZE: usize = 16 * 1024 * 1024; // 16MB per buffer
const CM_CAPACITY_CHUNKS: usize = CM_MAX_POINTS.div_ceil(CHUNK_POINTS);
const CM_PAGE_TABLE_OFFSET: usize = CM_HEADER_SIZE;
const CM_PAYLOAD_OFFSET: usize = (CM_HEADER_SIZE + CM_CAPACITY_CHUNKS * 8).div_ceil(4096) * 4096;
pub(crate) const CM_TOTAL_SIZE: usize = CM_PAYLOAD_OFFSET + CM_MAX_POINTS * CM_POINT_SIZE_BYTES;
pub(crate) const CM_POINT_SIZE_BYTES: usize = 12;
const CM_MAX_POINTS: usize = CM_SINGLE_BUFFER_SIZE / CM_POINT_SIZE_BYTES;
pub(crate) const MAX_VOXEL_SIZE: f32 = 1_048_576.0;

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

const CM_SEQUENCE_OFFSET: usize = 16;
const CM_GLOBAL_REVISION_OFFSET: usize = 24;
const CM_TIMESTAMP_NS_OFFSET: usize = 32;
const CM_LAYOUT_GENERATION_OFFSET: usize = 40;
const CM_PRODUCER_EPOCH_OFFSET: usize = 48;
const CM_GLOBAL_POINT_COUNT_OFFSET: usize = 64;
const CM_VOXEL_SIZE_OFFSET: usize = 80;

fn sequence_atomic(mapping: &[u8]) -> &AtomicU64 {
    // mmap bases are page-aligned and this offset is a multiple of eight.
    // The only access to this word after initialization is atomic.
    unsafe { &*(mapping.as_ptr().add(CM_SEQUENCE_OFFSET).cast::<AtomicU64>()) }
}

pub(crate) fn cloud_map_shm_path() -> std::path::PathBuf {
    std::path::PathBuf::from(
        std::env::var("NUWAX_CLOUDMAP_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_cloudmap.mmap".into()),
    )
}

pub(crate) fn cloud_map_active_slot(revision: u64) -> Option<u32> {
    revision.checked_sub(1).map(|_| 0)
}

#[derive(Clone)]
struct PendingFineFrameRehome {
    ticket: uuid::Uuid,
    proof: crate::mapping_adapter::InitialFrameEstablishment,
    baseline: Option<OccupancyCommitReceipt>,
}

pub struct CloudMapShmWriter {
    #[cfg(test)]
    last_stage_ms: [f64; 3],
    mmap: MmapMut,
    shm_path: std::path::PathBuf,
    shm_identity: Option<ProductOwnedMmapIdentity>,
    revision: u64,
    voxel_size: f32,
    session_generation: u64,
    producer_epoch: [u8; 16],
    layout_generation: u64,
    publication_sequence: u64,
    published_pages: Vec<Arc<[u8]>>,
    published_dirty_pages: Vec<usize>,
    publication_receipt: Option<OccupancyCommitReceipt>,
    voxels: VoxelIndex,
    ordered_voxels: PagedVoxelSnapshot,
    dynamic_filter: DynamicCloudFilter,
    fine_cloud: Option<FineCloud>,
    committed_fine_cloud: Option<Arc<nuwax_map_chunks::SealedMap>>,
    fine_owner: Option<(u64, u64)>,
    defer_fine_retention: bool,
    pending_fine_frame_rehome: Option<PendingFineFrameRehome>,
    dense_cloud: Option<DenseCloud>,
    committed_dense_cloud: Option<DenseSnapshot>,
    pending_dense_readers: Option<DenseCloudReaders>,
    pending_fine_readers: Option<fine_cloud::FineCloudReaders>,
    scan_preparation: Option<uuid::Uuid>,
    dense_error: Option<String>,
    dense_history_missing: bool,
    max_points: usize,
    last_occupancy_receipt: Option<OccupancyCommitReceipt>,
    last_cloud_receipt: Option<CloudCommitReceipt>,
    typed_publish_state: Arc<TypedCloudPublishState>,
    // 仅由本写入器提交的修订可复用；新提交和会话重置撤销本地引用。
    committed_snapshot_cache: OnceLock<CommittedCloudMapSnapshot<Arc<[u8]>>>,
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

/// Content-free producer-owned identity of one committed map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CommittedCloudMapMetadata {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub session_generation: u64,
    pub producer_epoch: [u8; 16],
    pub layout_generation: u64,
}

/// One successfully committed cumulative cloud-map revision and the exact
/// canonical map-frame voxel snapshot available to typed transport sinks.
#[derive(Clone)]
pub(crate) struct CommittedCloudMapSnapshot<Payload = Vec<u8>> {
    pub producer_epoch: [u8; 16],
    pub layout_generation: u64,
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub session_generation: u64,
    pub xyz_f32_le: Payload,
}

/// 私有重建得到的完整累积状态，包含尚未达到删除门限的动态反证。
pub(crate) struct PreparedCloudAccumulator {
    voxels: VoxelIndex,
    voxel_size: f32,
    dynamic_filter: DynamicCloudFilter,
    ordered_voxels: PagedVoxelSnapshot,
    fine_cloud: Option<FineCloud>,
    dense_cloud: Option<DenseCloud>,
    dense_error: Option<String>,
    dense_history_missing: bool,
}

/// 先于激活持锁提交完成读取、逐块校验和私有导入。
pub(crate) struct PreparedFineCloud { cloud: FineCloud }

impl PreparedFineCloud {
    pub(crate) fn voxel_size(&self) -> f32 { self.cloud.voxel_size() }
}

pub(crate) fn prepare_fine_cloud_archive(file: std::fs::File, session: &str, generation: u64,
    cloud_revision: u64, source_timestamp_ns: u64) -> Result<PreparedFineCloud, String> {
    let (cloud, root) = FineCloud::from_archive(file)?;
    if root.domain().session != session || root.domain().generation != generation
        || root.revision() != cloud_revision || root.source_timestamp_ns() != source_timestamp_ns {
        return Err("精细地图资产与原占据/点云提交不一致".into());
    }
    Ok(PreparedFineCloud { cloud })
}

pub(crate) struct CorrelatedCloudMapCommit {
    pub(crate) snapshot: CommittedCloudMapMetadata,
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
        // Publish a new inode atomically: never truncate a file still mapped by
        // a reader. The independent producer UUID also fences same-inode ABA.
        let temporary = shm_path.with_file_name(format!(".{}.{}",
            shm_path.file_name().unwrap_or_default().to_string_lossy(), uuid::Uuid::new_v4()));
        let file = std::fs::OpenOptions::new().read(true).write(true)
            .create_new(true).open(&temporary)?;
        let prepared = (|| {
            file.set_len(CM_TOTAL_SIZE as u64)?;
            let mmap = unsafe { MmapMut::map_mut(&file)? };
            let mut writer = Self::from_mapping(mmap, shm_path.to_path_buf(), None);
            std::fs::rename(&temporary, shm_path)?;
            writer.shm_identity = Some(capture_product_owned_mmap(shm_path, &file)?);
            Ok(writer)
        })();
        if prepared.is_err() { let _ = std::fs::remove_file(&temporary); }
        prepared
    }

    /// 私有重建不对外发布，使用匿名内存，避免每次纠偏创建和删除临时映射文件。
    pub(crate) fn new_private() -> std::io::Result<Self> {
        Ok(Self::from_mapping(
            MmapMut::map_anon(CM_TOTAL_SIZE)?,
            Default::default(),
            None,
        ))
    }

    #[cfg(test)]
    pub(crate) fn new_for_test(path: &std::path::Path) -> std::io::Result<Self> {
        let mut writer = Self::new(path)?;
        writer.bind_fine_map_owner(0, 1).unwrap();
        Ok(writer)
    }
    #[cfg(test)]
    pub(crate) fn new_private_for_test() -> std::io::Result<Self> {
        let mut writer = Self::new_private()?;
        writer.bind_fine_map_owner(0, 1).unwrap();
        Ok(writer)
    }

    fn from_mapping(
        mut mmap: MmapMut,
        shm_path: std::path::PathBuf,
        shm_identity: Option<ProductOwnedMmapIdentity>,
    ) -> Self {
        let producer_epoch = *uuid::Uuid::new_v4().as_bytes();
        mmap[..CM_HEADER_SIZE].fill(0);
        mmap[0..8].copy_from_slice(&CM_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&CM_SHM_VERSION.to_le_bytes());
        mmap[12..16].copy_from_slice(&(CM_HEADER_SIZE as u32).to_le_bytes());
        mmap[CM_PRODUCER_EPOCH_OFFSET..CM_PRODUCER_EPOCH_OFFSET + 16].copy_from_slice(&producer_epoch);
        mmap[68..72].copy_from_slice(&(CM_MAX_POINTS as u32).to_le_bytes());
        mmap[72..76].copy_from_slice(&(CHUNK_POINTS as u32).to_le_bytes());
        mmap[76..80].copy_from_slice(&(CM_CAPACITY_CHUNKS as u32).to_le_bytes());
        mmap[96..104].copy_from_slice(&(CM_PAGE_TABLE_OFFSET as u64).to_le_bytes());
        mmap[104..112].copy_from_slice(&(CM_PAYLOAD_OFFSET as u64).to_le_bytes());
        mmap[112..120].copy_from_slice(&(CM_TOTAL_SIZE as u64).to_le_bytes());
        Self {
            mmap,
            shm_path,
            shm_identity,
            revision: 0,
            voxel_size: VOXEL_SIZE,
            session_generation: 0,
            producer_epoch,
            layout_generation: 1,
            publication_sequence: 0,
            published_pages: Vec::new(),
            published_dirty_pages: Vec::new(),
            publication_receipt: None,
            voxels: VoxelIndex::new(),
            ordered_voxels: PagedVoxelSnapshot::default(),
            dynamic_filter: DynamicCloudFilter::default(),
            #[cfg(test)]
            last_stage_ms: [0.0; 3],
            fine_cloud: None,
            committed_fine_cloud: None,
            fine_owner: None,
            defer_fine_retention: false,
            pending_fine_frame_rehome: None,
            dense_cloud: None,
            committed_dense_cloud: None,
            pending_dense_readers: None,
            pending_fine_readers: None,
            scan_preparation: None,
            dense_error: None,
            dense_history_missing: false,
            max_points: CM_MAX_POINTS,
            last_occupancy_receipt: None,
            last_cloud_receipt: None,
            typed_publish_state: Arc::new(TypedCloudPublishState::new()),
            committed_snapshot_cache: OnceLock::new(),
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
        #[cfg(test)]
        let phase_started = std::time::Instant::now();
        if let Err(error) = self.fine_cloud.as_mut().unwrap().accumulate(points, sensor_origin_map, source_timestamp_ns) {
            warn!(%error, "精细导航源拒绝不完整输入");
            return false;
        }
        #[cfg(test)]
        { self.last_stage_ms[0] = phase_started.elapsed().as_secs_f64() * 1000.0; }
        #[cfg(test)]
        let phase_started = std::time::Instant::now();
        // 完整显示保留静态扫描原坐标和重复观测，同步撤销动态历史。
        // 故障锁存并撤销完整源，不能继续发布缺帧历史或影响原导航地图。
        if !self.dense_history_missing && self.dense_error.is_none() {
            let result = (|| {
                let fine = self.fine_cloud.as_ref().unwrap();
                if self.dense_cloud.is_none() { self.dense_cloud = Some(DenseCloud::with_voxel_size(fine.voxel_size())?); }
                let dense = self.dense_cloud.as_mut().unwrap();
                #[cfg(test)]
                if self.dynamic_filter.legacy_only { return dense.append(points); }
                dense.remove_voxels(fine.removed_keys(), fine.voxel_size())?;
                let static_points = points.iter().filter(|point| !fine.is_dynamic(point)).copied().collect::<Vec<_>>();
                dense.append(&static_points)
            })();
            if let Err(error) = result {
                warn!(%error, "完整扫描归档失败，完整点云不可用");
                self.dense_error = Some(error);
                self.committed_dense_cloud = None;
            }
        }
        #[cfg(test)]
        { self.last_stage_ms[1] = phase_started.elapsed().as_secs_f64() * 1000.0; }
        #[cfg(test)]
        let phase_started = std::time::Instant::now();
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
        let result = self.coarsen_to_fit();
        #[cfg(test)]
        { self.last_stage_ms[2] = phase_started.elapsed().as_secs_f64() * 1000.0; }
        result
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
        if self.voxels.is_empty() { return Ok(None); }
        let domain = if occupancy_receipt.source_timestamp_ns == 0 || self.fine_cloud.is_none() { None }
            else {
                let (frame_generation, producer_generation) = self.fine_owner
                    .ok_or("fine-map publication has no bound runtime/frame owner")?;
                Some(nuwax_map_chunks::Domain {
                    session: occupancy_receipt.session_id.clone(),
                    generation: occupancy_receipt.session_generation,
                    frame_generation, producer_generation,
                })
            };
        // Reserve the existing publication clock before sealing. A same-domain
        // restart continues its persisted revision; a new domain restamps it.
        let floor = domain.as_ref().map_or(0, |domain| self.fine_cloud.as_ref().unwrap().publication_floor(domain));
        let revision = self.prepare_publication(false, floor).ok_or("cloud publication cannot be reserved")?;
        let fine = domain.map(|domain| self.fine_cloud.as_mut().unwrap()
            .seal(domain, revision, occupancy_receipt.source_timestamp_ns)).transpose()?;
        self.publication_receipt = Some(occupancy_receipt.clone());
        self.publish_prepared_snapshot(revision)
            .ok_or("reserved cloud publication failed")?;
        let snapshot = self
            .committed_metadata()
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
        if !self.defer_fine_retention {
            if let Some(root) = &fine { self.fine_cloud.as_mut().unwrap().retain_committed(root); }
        }
        self.committed_fine_cloud = fine;
        self.committed_dense_cloud = if self.dense_error.is_none() && receipt.source_timestamp_ns > 0 {
            self.dense_cloud.as_ref().and_then(|cloud| cloud.snapshot(&receipt, &occupancy_receipt.session_id))
        } else { None };
        Ok(Some(CorrelatedCloudMapCommit { snapshot, receipt }))
    }

    pub(crate) fn committed_dense_cloud(&self) -> Result<Option<DenseSnapshot>, String> {
        if let Some(error) = &self.dense_error { return Err(format!("完整扫描归档失败：{error}")); }
        Ok(self.committed_dense_cloud.clone())
    }

    fn dense_readers(&self) -> Option<DenseCloudReaders> {
        self.dense_cloud.as_ref().map(DenseCloud::readers).or_else(|| self.pending_dense_readers.clone())
    }

    pub(crate) fn dense_cloud_for_read(&mut self, epoch: &str, offset: u64) -> Result<Option<DenseSnapshot>, String> {
        let Some(latest) = self.committed_dense_cloud()? else { return Ok(None); };
        self.dense_readers().ok_or("完整点云拥有方已退役")?.pin_read(latest, epoch, offset).map(Some)
    }

    pub(crate) fn dense_cloud_update_for_read(&mut self, epoch: &str, offset: u64)
        -> Result<Option<(DenseSnapshot, Option<DenseSnapshot>)>, String> {
        let Some(latest) = self.committed_dense_cloud()? else { return Ok(None); };
        self.dense_readers().ok_or("完整点云拥有方已退役")?.pin_update(latest, epoch, offset).map(Some)
    }

    pub(crate) fn dense_read_epoch_available(&self, epoch: &str) -> bool {
        self.dense_error.is_none() && self.committed_dense_cloud.as_ref().is_some_and(|latest|
            self.dense_readers().is_some_and(|readers| readers.read_epoch_available(&latest.epoch, epoch)))
    }

    /// 移交可变累积状态，公开读者继续使用原提交；计算结果只凭本次票据安装。
    pub(crate) fn prepare_scan_work(&mut self) -> Result<(uuid::Uuid, Self), String> {
        if self.scan_preparation.is_some() { return Err("点云已有在途准备".into()); }
        let mut work = Self::new_private().map_err(|e| e.to_string())?;
        let ticket = uuid::Uuid::new_v4();
        self.pending_dense_readers = self.dense_cloud.as_ref().map(DenseCloud::readers);
        self.pending_fine_readers = self.fine_cloud.as_ref().map(FineCloud::readers);
        work.replace_accumulator(self.take_prepared_accumulator());
        work.revision = self.revision;
        work.session_generation = self.session_generation;
        work.producer_epoch = self.producer_epoch;
        work.layout_generation = self.layout_generation;
        work.publication_sequence = self.publication_sequence;
        work.published_pages = self.published_pages.clone();
        work.publication_receipt = self.publication_receipt.clone();
        work.max_points = self.max_points;
        work.last_occupancy_receipt = self.last_occupancy_receipt.clone();
        work.last_cloud_receipt = self.last_cloud_receipt.clone();
        work.committed_fine_cloud = self.committed_fine_cloud.clone();
        work.fine_owner = self.fine_owner;
        work.defer_fine_retention = true;
        work.committed_dense_cloud = self.committed_dense_cloud.clone();
        self.scan_preparation = Some(ticket);
        Ok((ticket, work))
    }

    pub(crate) fn install_scan_work(&mut self, ticket: uuid::Uuid, mut work: Self) -> Result<(), String> {
        if self.scan_preparation != Some(ticket) {
            return Err("point-cloud preparation ticket is stale".into());
        }
        if self.session_generation != work.session_generation {
            return Err(format!("point-cloud preparation generation mismatch: writer={} prepared={}",
                self.session_generation, work.session_generation));
        }
        if let Some(handoff) = &self.pending_fine_frame_rehome {
            if handoff.ticket != ticket { return Err("fine frame handoff belongs to another scan ticket".into()); }
            if let Some(receipt) = &work.last_occupancy_receipt {
                handoff.proof.validate_writer_lineage(receipt)?;
                if let Some(base) = &handoff.baseline {
                    if receipt.commit_id.sequence < base.commit_id.sequence || receipt.occupancy_revision < base.occupancy_revision
                        || ((receipt.commit_id.sequence == base.commit_id.sequence || receipt.occupancy_revision == base.occupancy_revision)
                            && receipt != base) {
                        return Err("prepared frame handoff changed or rolled back its captured source receipt".into());
                    }
                }
            } else if handoff.baseline.is_some() { return Err("prepared frame handoff lost its source receipt".into()); }
            work.apply_initial_fine_frame_rehome(&handoff.proof)?;
        }
        if self.fine_owner != work.fine_owner {
            return Err("prepared cloud owner changed outside the admitted frame handoff".into());
        }
        if work.revision != self.revision {
            if self.revision.checked_add(1) != Some(work.revision) {
                return Err(format!("point-cloud preparation revision mismatch: writer={} prepared={}",
                    self.revision, work.revision));
            }
            // Carry immutable current pages across private preparation. Only
            // pages written by this accepted work item reach public SHM.
            self.producer_epoch = work.producer_epoch;
            self.layout_generation = work.layout_generation;
            self.voxel_size = work.voxel_size;
            self.published_pages = std::mem::take(&mut work.published_pages);
            self.published_dirty_pages = std::mem::take(&mut work.published_dirty_pages);
            self.publication_receipt = work.publication_receipt.clone();
            let count = work.voxels.len() as u32;
            let timestamp = u64::from_le_bytes(work.mmap[CM_TIMESTAMP_NS_OFFSET..CM_TIMESTAMP_NS_OFFSET + 8].try_into().unwrap());
            self.write_paged_publication(work.revision, count, timestamp)
                .ok_or("cloud-map publication sequence exhausted")?;
            self.revision = work.revision;
            self.committed_snapshot_cache = std::mem::take(&mut work.committed_snapshot_cache);
        }
        self.replace_accumulator(work.take_prepared_accumulator());
        self.last_occupancy_receipt = work.last_occupancy_receipt.take();
        self.last_cloud_receipt = work.last_cloud_receipt.take();
        self.committed_fine_cloud = work.committed_fine_cloud.take();
        self.fine_owner = work.fine_owner;
        if let (Some(owner), Some(root)) = (self.fine_cloud.as_mut(), self.committed_fine_cloud.as_ref()) {
            owner.retain_committed(root);
        }
        self.committed_dense_cloud = work.committed_dense_cloud.take();
        self.pending_dense_readers = None;
        self.pending_fine_readers = None;
        self.pending_fine_frame_rehome = None;
        self.scan_preparation = None;
        Ok(())
    }

    fn replace_accumulator(&mut self, prepared: PreparedCloudAccumulator) {
        self.voxels = prepared.voxels;
        self.voxel_size = prepared.voxel_size;
        self.dynamic_filter = prepared.dynamic_filter;
        self.ordered_voxels = prepared.ordered_voxels;
        self.replace_fine_cloud(prepared.fine_cloud);
        self.dense_cloud = prepared.dense_cloud;
        self.dense_error = prepared.dense_error;
        self.dense_history_missing = prepared.dense_history_missing;
    }

    pub(crate) fn committed_metadata(&self) -> Option<CommittedCloudMapMetadata> {
        if self.revision == 0 || self.session_generation == 0 { return None; }
        let sequence = sequence_atomic(&self.mmap).load(Ordering::Acquire);
        if sequence & 1 != 0 { return None; }
        let revision = u64::from_le_bytes(self.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8].try_into().ok()?);
        let point_count = u32::from_le_bytes(self.mmap[CM_GLOBAL_POINT_COUNT_OFFSET..CM_GLOBAL_POINT_COUNT_OFFSET + 4].try_into().ok()?);
        let layout = u64::from_le_bytes(self.mmap[CM_LAYOUT_GENERATION_OFFSET..CM_LAYOUT_GENERATION_OFFSET + 8].try_into().ok()?);
        if revision != self.revision || layout != self.layout_generation
            || self.mmap[CM_PRODUCER_EPOCH_OFFSET..CM_PRODUCER_EPOCH_OFFSET + 16] != self.producer_epoch
            || sequence_atomic(&self.mmap).load(Ordering::Acquire) != sequence { return None; }
        let voxel_size = f32::from_le_bytes(self.mmap[CM_VOXEL_SIZE_OFFSET..CM_VOXEL_SIZE_OFFSET + 4].try_into().ok()?);
        let session_generation = u64::from_le_bytes(self.mmap[88..96].try_into().ok()?);
        Some(CommittedCloudMapMetadata { revision, point_count, voxel_size,
            session_generation, producer_epoch: self.producer_epoch,
            layout_generation: self.layout_generation })
    }

    #[cfg(test)]
    pub(crate) fn full_snapshot_materialized(&self) -> bool {
        self.committed_snapshot_cache.get().is_some()
    }

    /// Explicit full export only. Ordinary commits and publication guards use
    /// committed_metadata; unchanged page bytes remain shared between commits.
    pub(crate) fn committed_snapshot(&self) -> Option<CommittedCloudMapSnapshot> {
        let snapshot = self.shared_committed_snapshot()?;
        Some(CommittedCloudMapSnapshot {
            producer_epoch: snapshot.producer_epoch, layout_generation: snapshot.layout_generation,
            revision: snapshot.revision, point_count: snapshot.point_count,
            voxel_size: snapshot.voxel_size, session_generation: snapshot.session_generation,
            xyz_f32_le: snapshot.xyz_f32_le.to_vec(),
        })
    }

    pub(crate) fn shared_committed_snapshot(&self) -> Option<CommittedCloudMapSnapshot<Arc<[u8]>>> {
        let metadata = self.committed_metadata()?;
        let snapshot = self.committed_snapshot_cache.get_or_init(|| {
            let mut bytes = Vec::with_capacity(metadata.point_count as usize * CM_POINT_SIZE_BYTES);
            for page in &self.published_pages { bytes.extend_from_slice(page); }
            let xyz_f32_le: Arc<[u8]> = bytes.into();
            CommittedCloudMapSnapshot {
                producer_epoch: metadata.producer_epoch, layout_generation: metadata.layout_generation,
                revision: metadata.revision, point_count: metadata.point_count,
                voxel_size: metadata.voxel_size, session_generation: metadata.session_generation,
                xyz_f32_le,
            }
        });
        Some(snapshot.clone())
    }

    /// Retain an already materialized payload without exporting or copying it.
    /// Descriptor validation remains the caller's responsibility.
    pub(crate) fn materialized_committed_snapshot(&self) -> Option<CommittedCloudMapSnapshot<Arc<[u8]>>> {
        self.committed_snapshot_cache.get().cloned()
    }

    pub(crate) fn last_cloud_receipt(&self) -> Option<CloudCommitReceipt> {
        self.last_cloud_receipt.clone()
    }

    /// The actual frozen occupancy baseline that owns the retained cloud commit.
    pub(crate) fn last_occupancy_receipt(&self) -> Option<OccupancyCommitReceipt> {
        self.last_occupancy_receipt.clone()
    }

    fn replace_fine_cloud(&mut self, mut fine: Option<FineCloud>) {
        // A complete rebuild/checkpoint replaces mutable storage, not the
        // retained-root lookup authority used by already-published readers.
        let retained = self.fine_cloud.as_ref().map(FineCloud::readers)
            .or_else(|| self.pending_fine_readers.clone());
        if let (Some(owner), Some(readers)) = (fine.as_mut(), retained) { owner.adopt_readers(readers); }
        self.fine_cloud = fine;
    }

    pub(crate) fn bind_fine_map_owner(&mut self, frame_generation: u64, producer_generation: u64) -> Result<(), String> {
        if producer_generation == 0 { return Err("fine-map producer generation must be positive".into()); }
        self.fine_owner = Some((frame_generation, producer_generation));
        Ok(())
    }

    pub(crate) fn bind_fine_map_frame(&mut self, frame_generation: u64) -> Result<(), String> {
        let (_, producer) = self.fine_owner.ok_or("fine-map runtime owner is not bound")?;
        self.bind_fine_map_owner(frame_generation, producer)
    }

    /// Rehome unchanged committed map-frame coordinates when the first real
    /// odometry anchor establishes the session's frame domain. This is not a
    /// scan or occupancy commit and never advances either publication clock.
    pub(crate) fn rehome_committed_fine_frame(&mut self,
        proof: &crate::mapping_adapter::InitialFrameEstablishment) -> Result<bool, String> {
        if let Some(baseline) = &self.last_occupancy_receipt { proof.validate_writer_baseline(baseline)?; }
        let baseline = self.last_occupancy_receipt.clone();
        let changed = self.apply_initial_fine_frame_rehome(proof)?;
        if let Some(ticket) = self.scan_preparation {
            self.pending_fine_frame_rehome = Some(PendingFineFrameRehome { ticket, proof: proof.clone(), baseline });
        }
        Ok(changed)
    }

    fn apply_initial_fine_frame_rehome(&mut self,
        proof: &crate::mapping_adapter::InitialFrameEstablishment) -> Result<bool, String> {
        let (old_frame, new_frame) = (proof.old_frame(), proof.new_frame());
        let (session, generation) = (proof.session_id(), proof.session_generation());
        if old_frame.checked_add(1) != Some(new_frame) {
            return Err("fine frame transition is stale or exhausted".into());
        }
        let (_, producer) = self.fine_owner.ok_or("fine runtime owner is not bound")?;
        if self.fine_owner != Some((old_frame, producer)) {
            return Err("fine frame transition does not name its current owner".into());
        }
        let Some(previous) = self.committed_fine_cloud.clone() else {
            if self.revision != 0 && self.session_generation != generation {
                return Err("empty fine frame transition crossed the writer session".into());
            }
            self.bind_fine_map_owner(new_frame, producer)?;
            return Ok(false);
        };
        let root = previous.manifest();
        let metadata = self.committed_metadata().ok_or("fine frame transition lacks committed metadata")?;
        let cloud = self.last_cloud_receipt.as_ref().ok_or("fine frame transition lacks cloud receipt")?;
        let occupancy = self.last_occupancy_receipt.as_ref().ok_or("fine frame transition lacks occupancy receipt")?;
        if root.domain.session != session || root.domain.generation != generation
            || root.domain.frame_generation != old_frame || root.domain.producer_generation != producer
            || occupancy.session_id != session || occupancy.session_generation != generation
            || cloud.session_generation != generation || cloud.cloud_revision != root.revision
            || metadata.revision != root.revision || metadata.session_generation != generation
            || cloud.commit_id != occupancy.commit_id || cloud.occupancy_revision != occupancy.occupancy_revision
            || cloud.source_timestamp_ns != root.source_timestamp_ns {
            return Err("fine frame transition differs from immutable committed ownership".into());
        }
        let retained = if self.defer_fine_retention { None } else {
            Some(self.fine_cloud.as_ref().map(FineCloud::readers).or_else(|| self.pending_fine_readers.clone())
                .ok_or("fine frame transition lost retained readers")?)
        };
        let next = if let Some(fine) = self.fine_cloud.as_mut() {
            fine.rehome_committed(&root.identity(), new_frame)?
        } else if self.scan_preparation.is_some() {
            Arc::new(previous.rehome_frame(&root.identity(), new_frame)?)
        } else { return Err("fine frame transition has no source owner".into()); };
        if let Some(readers) = retained { readers.retain_committed(&next); }
        self.committed_fine_cloud = Some(next);
        self.fine_owner = Some((new_frame, producer));
        Ok(true)
    }

    pub(crate) fn accepts_prepared_fine_frame_transition(&self, old_frame: u64, new_frame: u64) -> bool {
        self.pending_fine_frame_rehome.as_ref().is_some_and(|handoff|
            self.scan_preparation == Some(handoff.ticket) && handoff.proof.old_frame() == old_frame
                && handoff.proof.new_frame() == new_frame)
    }

    pub(crate) fn pin_fine_map_root(&self, identity: &nuwax_map_chunks::RootIdentity,
        reference: &nuwax_map_chunks::BlockRef) -> Result<Arc<nuwax_map_chunks::SealedMap>, String> {
        self.fine_cloud.as_ref().map(FineCloud::readers).or_else(|| self.pending_fine_readers.clone())
            .ok_or("fine-map owner is unavailable")?.pin_root(identity, reference)
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
            dense_cloud: self.dense_cloud.take(),
            dense_error: self.dense_error.take(),
            dense_history_missing: self.dense_history_missing,
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
        self.scan_preparation = None;
        self.pending_dense_readers = None;
        self.pending_fine_readers = None;
        self.pending_fine_frame_rehome = None;
        self.session_generation = receipt.session_generation;
        self.voxel_size = prepared.voxel_size;
        self.voxels = prepared.voxels;
        self.dynamic_filter = prepared.dynamic_filter;
        self.ordered_voxels = prepared.ordered_voxels;
        self.ordered_voxels.invalidate();
        self.replace_fine_cloud(prepared.fine_cloud);
        self.dense_cloud = prepared.dense_cloud;
        self.dense_error = prepared.dense_error;
        self.dense_history_missing = prepared.dense_history_missing;
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
        self.install_checkpoint_with_sources(occupancy_receipt, voxel_size, point_count, xyz_f32_le, fine, None)
    }

    pub(crate) fn install_checkpoint_with_sources(
        &mut self, occupancy_receipt: &OccupancyCommitReceipt, voxel_size: f32, point_count: u32,
        xyz_f32_le: &[u8], fine: Option<PreparedFineCloud>, dense: Option<DenseCloud>,
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
        self.scan_preparation = None;
        self.pending_dense_readers = None;
        self.pending_fine_readers = None;
        self.pending_fine_frame_rehome = None;
        self.session_generation = session_generation;
        self.voxel_size = voxel_size;
        self.voxels = voxels.into();
        self.ordered_voxels.invalidate();
        self.dynamic_filter = DynamicCloudFilter::default();
        self.replace_fine_cloud(Some(fine));
        self.dense_history_missing = dense.is_none();
        self.dense_cloud = dense;
        self.dense_error = None;
        self.committed_dense_cloud = None;
        self.commit_accumulated_snapshot(occupancy_receipt)?
            .ok_or_else(|| "cloud-map checkpoint publication failed".to_string())
    }

    fn prepare_publication(&self, allow_empty: bool, revision_floor: u64) -> Option<u64> {
        if self.session_generation == 0 || (!allow_empty && self.voxels.is_empty()) { return None; }
        if self.voxels.len() > self.max_points || self.voxels.len() > CM_MAX_POINTS { return None; }
        let revision = self.revision.max(revision_floor).checked_add(1)?;
        self.publication_sequence.checked_add(2)?;
        if self.ordered_voxels.needs_rebuild() { self.layout_generation.checked_add(1)?; }
        Some(revision)
    }

    fn publish_snapshot(&mut self, allow_empty: bool) -> Option<(u64, u32, f32)> {
        let revision = self.prepare_publication(allow_empty, 0)?;
        self.publish_prepared_snapshot(revision)
    }

    fn publish_prepared_snapshot(&mut self, revision: u64) -> Option<(u64, u32, f32)> {
        if revision <= self.revision { return None; }
        self.publication_sequence.checked_add(2)?;
        let point_count = self.voxels.len();
        if self.ordered_voxels.needs_rebuild() {
            let next_layout = self.layout_generation.checked_add(1)?;
            self.ordered_voxels.prepare(&self.voxels);
            self.layout_generation = next_layout;
            self.published_pages.clear();
        }
        let count = point_count.div_ceil(CHUNK_POINTS);
        self.published_pages.resize_with(count, || Arc::from([]));
        let dirty = self.ordered_voxels.dirty_pages().filter(|page| *page < count).collect::<Vec<_>>();
        for &page in &dirty {
            let start = page * CHUNK_POINTS;
            let end = (start + CHUNK_POINTS).min(point_count);
            let mut bytes = Vec::with_capacity((end - start) * CM_POINT_SIZE_BYTES);
            for &key in &self.ordered_voxels.keys()[start..end] {
                for coordinate in voxel_center_f32(key, self.voxel_size) {
                    bytes.extend_from_slice(&coordinate.to_le_bytes());
                }
            }
            self.published_pages[page] = bytes.into();
        }
        self.published_dirty_pages = dirty;
        let timestamp = self.publication_receipt.as_ref().map(|r| r.source_timestamp_ns)
            .unwrap_or_else(|| std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default().as_nanos() as u64);
        self.write_paged_publication(revision, point_count as u32, timestamp)?;
        self.revision = revision;
        self.ordered_voxels.committed();
        self.committed_snapshot_cache.take();
        Some((revision, point_count as u32, self.voxel_size))
    }

    fn write_paged_publication(&mut self, revision: u64, point_count: u32, timestamp: u64) -> Option<()> {
        let sequence = self.publication_sequence.checked_add(2)?;
        // Begin must be globally ordered BEFORE any ordinary payload stores.
        sequence_atomic(&self.mmap).store(sequence - 1, Ordering::SeqCst);
        fence(Ordering::SeqCst);
        for &page in &self.published_dirty_pages {
            let start = CM_PAYLOAD_OFFSET + page * CHUNK_POINTS * CM_POINT_SIZE_BYTES;
            let bytes = &self.published_pages[page];
            self.mmap[start..start + bytes.len()].copy_from_slice(bytes);
            let version_offset = CM_PAGE_TABLE_OFFSET + page * 8;
            self.mmap[version_offset..version_offset + 8].copy_from_slice(&revision.to_le_bytes());
        }
        self.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8].copy_from_slice(&revision.to_le_bytes());
        self.mmap[CM_TIMESTAMP_NS_OFFSET..CM_TIMESTAMP_NS_OFFSET + 8].copy_from_slice(&timestamp.to_le_bytes());
        self.mmap[CM_LAYOUT_GENERATION_OFFSET..CM_LAYOUT_GENERATION_OFFSET + 8].copy_from_slice(&self.layout_generation.to_le_bytes());
        self.mmap[CM_PRODUCER_EPOCH_OFFSET..CM_PRODUCER_EPOCH_OFFSET + 16].copy_from_slice(&self.producer_epoch);
        self.mmap[CM_GLOBAL_POINT_COUNT_OFFSET..CM_GLOBAL_POINT_COUNT_OFFSET + 4].copy_from_slice(&point_count.to_le_bytes());
        self.mmap[CM_VOXEL_SIZE_OFFSET..CM_VOXEL_SIZE_OFFSET + 4].copy_from_slice(&self.voxel_size.to_le_bytes());
        self.mmap[88..96].copy_from_slice(&self.session_generation.to_le_bytes());
        self.mmap[128..420].fill(0);
        if let Some(receipt) = &self.publication_receipt {
            self.mmap[128..144].copy_from_slice(&receipt.commit_id.boot_nonce.to_le_bytes());
            self.mmap[144..152].copy_from_slice(&receipt.commit_id.sequence.to_le_bytes());
            self.mmap[152..160].copy_from_slice(&receipt.occupancy_revision.to_le_bytes());
            let id = receipt.session_id.as_bytes();
            // The receipt boundary validates the complete UTF-8 identity before
            // any publication mutation; never truncate or split an identifier.
            let length = id.len();
            self.mmap[160..164].copy_from_slice(&(length as u32).to_le_bytes());
            self.mmap[164..164 + length].copy_from_slice(&id[..length]);
        }
        sequence_atomic(&self.mmap).store(sequence, Ordering::Release);
        self.publication_sequence = sequence;
        Some(())
    }

    fn reset_accumulator(&mut self, session_generation: u64) {
        self.committed_snapshot_cache.take();
        self.publication_receipt = None;
        self.scan_preparation = None;
        self.pending_dense_readers = None;
        self.pending_fine_readers = None;
        self.pending_fine_frame_rehome = None;
        self.fine_cloud = None;
        self.committed_fine_cloud = None;
        self.dense_cloud = None;
        self.committed_dense_cloud = None;
        self.dense_error = None;
        self.dense_history_missing = false;
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
            || receipt.session_id.len() > 256
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

pub(crate) fn voxel_key(point: [f64; 3], voxel_size: f32) -> Option<VoxelKey> {
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

    /// The Python peer acknowledges each committed phase after reading real SHM.
    /// No transport, legacy full export, content digest or robot is involved.
    #[test]
    fn same_domain_archive_restore_reserves_existing_cloud_clock_then_private_work_advances_once() {
        let receipt = |sequence, timestamp| OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 81, sequence },
            session_id: "restored-owner".into(), session_generation: 1, occupancy_revision: sequence,
            source_timestamp_ns: timestamp, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        let mut source = CloudMapShmWriter::new_private_for_test().unwrap();
        source.revision = 69;
        assert!(source.accumulate_mapping_scan(1, &[Point3::new(0.125, 0.125, 0.125)], None, 100));
        source.commit_accumulated_snapshot(&receipt(1, 100)).unwrap().unwrap();
        let sealed = source.committed_fine_cloud().unwrap();
        let snapshot = source.committed_snapshot().unwrap();
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let fine = prepare_fine_cloud_archive(file, "restored-owner", 1, 70, 100).unwrap();
        let mut restored = CloudMapShmWriter::new_private_for_test().unwrap();
        let committed = restored.install_checkpoint_with_fine(&receipt(2, 100), snapshot.voxel_size,
            snapshot.point_count, &snapshot.xyz_f32_le, Some(fine)).unwrap();
        assert_eq!(committed.receipt.cloud_revision, 71);
        assert_eq!(restored.committed_fine_cloud().unwrap().manifest().geometry_revision, 70);
        let (ticket, mut work) = restored.prepare_scan_work().unwrap();
        assert!(work.accumulate_mapping_scan(1, &[Point3::new(0.125, 0.125, 0.125)], None, 101));
        work.commit_accumulated_snapshot(&receipt(3, 101)).unwrap().unwrap();
        restored.install_scan_work(ticket, work).unwrap();
        assert_eq!(restored.last_cloud_receipt().unwrap().cloud_revision, 72);
    }

    #[test]
    fn coordinate_rebuild_does_not_admit_an_old_private_scan_as_frame_rehome() {
        let receipt = |sequence| OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 91, sequence },
            session_id: "coordinate-owner".into(), session_generation: 1, occupancy_revision: sequence,
            source_timestamp_ns: sequence, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        writer.accumulate_mapping_scan(1, &[Point3::new(0.025, 0.025, 0.025)], None, 1);
        writer.commit_accumulated_snapshot(&receipt(1)).unwrap().unwrap();
        let (ticket, mut old_work) = writer.prepare_scan_work().unwrap();
        old_work.accumulate_mapping_scan(1, &[Point3::new(1.025, 0.025, 0.025)], None, 2);
        old_work.commit_accumulated_snapshot(&receipt(2)).unwrap().unwrap();
        let mut rebuilt = CloudMapShmWriter::new_private_for_test().unwrap();
        rebuilt.accumulate_mapping_scan(1, &[Point3::new(10.025, 0.025, 0.025)], None, 3);
        writer.bind_fine_map_frame(1).unwrap();
        assert!(!writer.accepts_prepared_fine_frame_transition(0, 1));
        writer.install_prepared_accumulator(&receipt(3), rebuilt.take_prepared_accumulator()).unwrap();
        let current = writer.committed_fine_cloud().unwrap();
        assert!(writer.install_scan_work(ticket, old_work).is_err());
        assert!(Arc::ptr_eq(&current, &writer.committed_fine_cloud().unwrap()));
        assert_eq!(current.manifest().domain.frame_generation, 1);
    }

    #[test]
    fn private_fine_candidate_is_not_fetchable_until_installed() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let receipt = |sequence, timestamp| OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 55, sequence },
            session_id: "retained".into(), session_generation: 1, occupancy_revision: sequence,
            source_timestamp_ns: timestamp, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        assert!(writer.accumulate_mapping_scan(1, &[Point3::new(0.1, 0.1, 0.1)], None, 1));
        writer.commit_accumulated_snapshot(&receipt(1, 1)).unwrap().unwrap();
        let old = writer.committed_fine_cloud().unwrap();
        let old_id = old.manifest().identity();
        let old_ref = old.manifest().blocks[0].clone();
        let (ticket, mut work) = writer.prepare_scan_work().unwrap();
        assert!(writer.pin_fine_map_root(&old_id, &old_ref).is_ok());
        assert!(work.accumulate_mapping_scan(1, &[Point3::new(0.1, 0.1, 0.1)], None, 2));
        work.commit_accumulated_snapshot(&receipt(2, 2)).unwrap().unwrap();
        let next = work.committed_fine_cloud().unwrap();
        let next_id = next.manifest().identity();
        let next_ref = next.manifest().blocks[0].clone();
        assert!(writer.pin_fine_map_root(&next_id, &next_ref).is_err());
        assert_eq!(writer.last_cloud_receipt().unwrap().cloud_revision, old_id.revision);
        writer.install_scan_work(ticket, work).unwrap();
        assert!(writer.pin_fine_map_root(&next_id, &next_ref).is_ok());
        assert_eq!(next.manifest().geometry_revision, old.manifest().geometry_revision);
        assert!(next_ref.block_revision > old_ref.block_revision);
        let mut rebuild = CloudMapShmWriter::new_private_for_test().unwrap();
        assert!(rebuild.accumulate_mapping_scan(1, &[Point3::new(2.1, 0.1, 0.1)], None, 3));
        writer.install_prepared_accumulator(&receipt(3, 3), rebuild.take_prepared_accumulator()).unwrap();
        assert!(writer.pin_fine_map_root(&old_id, &old_ref).is_ok());
        assert!(writer.pin_fine_map_root(&next_id, &next_ref).is_ok());
    }

    #[test]
    #[ignore = "run through scripts/check_pointcloud_native_interop.py with a Python mmap peer"]
    fn native_paged_writer_python_reader_interop() {
        use std::io::{self, Write};
        fn phase(number: u32) {
            println!("NUWAX_CLOUD_PAGE_PHASE:{number}");
            io::stdout().flush().unwrap();
            let mut line = String::new();
            assert!(io::stdin().read_line(&mut line).unwrap() > 0, "Python peer closed");
            assert_eq!(line.trim(), number.to_string());
        }
        let path = std::path::PathBuf::from(std::env::var_os("NUWAX_CLOUD_PAGE_INTEROP_PATH")
            .expect("Python peer supplies isolated fixture path"));
        let mut writer = CloudMapShmWriter::new_for_test(&path).unwrap();
        writer.reset_accumulator(1);
        // Binary-exact centers make cross-language byte assertions unambiguous.
        writer.voxel_size = 0.5;
        let points = (0..4096).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect::<Vec<_>>();
        assert!(writer.accumulate_mapping_frame(1, &points));
        writer.commit_accumulated().unwrap();
        phase(1);
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(5000.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        phase(2);
        // Actual edit semantics: remove an old voxel, then add its replacement.
        let key = voxel_key([0.0, 0.0, 0.0], writer.voxel_size).unwrap();
        assert!(writer.voxels.remove(&key));
        writer.ordered_voxels.removed(key, writer.max_points);
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(6000.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        phase(3);
        let key = voxel_key([2048.0, 0.0, 0.0], writer.voxel_size).unwrap();
        assert!(writer.voxels.remove(&key));
        writer.ordered_voxels.removed(key, writer.max_points);
        writer.commit_accumulated().unwrap();
        phase(4);
        writer.begin_session(2).unwrap();
        phase(5);
        assert!(writer.accumulate_mapping_frame(2, &[Point3::new(0.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        phase(6);
        assert!(!writer.full_snapshot_materialized());
    }

    #[test]
    fn paged_commit_and_private_install_reuse_unchanged_bytes() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let points = (0..10000).map(|i| Point3::new(i as f64 * 0.1, 0.0, 0.0)).collect::<Vec<_>>();
        assert!(writer.accumulate_mapping_frame(1, &points));
        writer.commit_accumulated().unwrap();
        assert!(!writer.full_snapshot_materialized());
        let first_page = writer.published_pages[0].clone();
        let first_metadata = writer.committed_metadata().unwrap();
        let first_version = read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET);
        let (ticket, mut work) = writer.prepare_scan_work().unwrap();
        assert!(work.accumulate_mapping_frame(1, &[Point3::new(2000.0, 0.0, 0.0)]));
        work.commit_accumulated().unwrap();
        assert_eq!(work.published_dirty_pages, vec![9]);
        assert!(Arc::ptr_eq(&first_page, &work.published_pages[0]));
        assert_eq!(writer.committed_metadata().unwrap(), first_metadata);
        writer.install_scan_work(ticket, work).unwrap();
        assert!(!writer.full_snapshot_materialized());
        assert_eq!(writer.committed_metadata().unwrap().producer_epoch, first_metadata.producer_epoch);
        assert_eq!(writer.committed_metadata().unwrap().point_count, 10001);
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET), first_version);
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET + 9 * 8), first_metadata.revision + 1);
        assert!(Arc::ptr_eq(&first_page, &writer.published_pages[0]));
        assert_eq!(writer.published_pages.iter().map(|page| page.len()).sum::<usize>(), 10001 * 12);
        assert!(!writer.full_snapshot_materialized());
    }

    #[test]
    fn receipt_session_identity_is_never_truncated_or_split() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(1.0, 0.0, 0.0)]));
        let mut receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 7, sequence: 1 },
            session_id: format!("{}é", "a".repeat(255)), session_generation: 1,
            occupancy_revision: 1, source_timestamp_ns: 10,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel, backend_generation: 1,
        };
        assert!(writer.commit_accumulated_snapshot(&receipt).is_err());
        assert_eq!(writer.revision, 0);
        receipt.session_id = format!("{}é", "a".repeat(254));
        writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
        assert_eq!(read_u32(&writer.mmap, 160), 256);
        assert_eq!(&writer.mmap[164..420], receipt.session_id.as_bytes());
        assert!(!writer.full_snapshot_materialized());
    }

    #[test]
    fn materialized_snapshot_retains_existing_payload_without_exporting_and_commit_invalidates_cache() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        assert!(writer.materialized_committed_snapshot().is_none());
        assert!(!writer.full_snapshot_materialized());
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(1.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        assert!(writer.committed_metadata().is_some());
        assert!(writer.materialized_committed_snapshot().is_none());
        assert!(!writer.full_snapshot_materialized());

        let exported = writer.shared_committed_snapshot().unwrap();
        let retained = writer.materialized_committed_snapshot().unwrap();
        assert!(writer.full_snapshot_materialized());
        assert!(Arc::ptr_eq(&exported.xyz_f32_le, &retained.xyz_f32_le));
        assert_eq!(retained.revision, exported.revision);
        let original = retained.xyz_f32_le.to_vec();

        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(2.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        assert!(writer.committed_metadata().unwrap().revision > retained.revision);
        assert!(writer.materialized_committed_snapshot().is_none());
        assert!(!writer.full_snapshot_materialized());
        assert!(Arc::ptr_eq(&exported.xyz_f32_le, &retained.xyz_f32_le));
        assert_eq!(retained.xyz_f32_le.as_ref(), original.as_slice());

        let next = writer.shared_committed_snapshot().unwrap();
        assert_eq!(next.point_count, 2);
        assert!(!Arc::ptr_eq(&retained.xyz_f32_le, &next.xyz_f32_le));
        assert!(Arc::ptr_eq(&next.xyz_f32_le,
            &writer.materialized_committed_snapshot().unwrap().xyz_f32_le));
        assert_eq!(retained.xyz_f32_le.as_ref(), original.as_slice());
    }

    #[test]
    fn shared_snapshot_tracks_commits_and_keeps_old_payload_immutable() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        writer.begin_session(1).unwrap();
        let cleared = writer.shared_committed_snapshot().unwrap();
        assert!(cleared.xyz_f32_le.is_empty());
        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(1.0, 0.0, 0.0)]));
        writer.commit_accumulated().unwrap();
        let first = writer.shared_committed_snapshot().unwrap();
        let again = writer.shared_committed_snapshot().unwrap();
        assert!(Arc::ptr_eq(&first.xyz_f32_le, &again.xyz_f32_le));
        let metadata = writer.committed_metadata().unwrap();
        assert_eq!(first.producer_epoch, metadata.producer_epoch);
        assert_eq!(first.layout_generation, metadata.layout_generation);
        assert_eq!(first.session_generation, metadata.session_generation);
        assert_eq!(first.revision, metadata.revision);
        assert_eq!(first.point_count, metadata.point_count);
        assert_eq!(first.voxel_size, metadata.voxel_size);
        let expected_xyz: Vec<u8> = voxel_center_f32(voxel_key([1.0, 0.0, 0.0], first.voxel_size).unwrap(), first.voxel_size)
            .into_iter().flat_map(f32::to_le_bytes).collect();
        assert_eq!(first.xyz_f32_le.as_ref(), expected_xyz.as_slice());
        let owned = writer.committed_snapshot().unwrap();
        assert_eq!(owned.xyz_f32_le.as_slice(), first.xyz_f32_le.as_ref());
        assert_eq!(owned.producer_epoch, first.producer_epoch);
        assert_eq!(owned.layout_generation, first.layout_generation);
        assert_eq!(owned.session_generation, first.session_generation);
        assert_eq!(owned.revision, first.revision);
        assert_eq!(owned.point_count, first.point_count);
        assert_eq!(owned.voxel_size, first.voxel_size);
        let original = first.xyz_f32_le.to_vec();

        // 私有准备尚未安装时，读者继续观察原提交；安装后直接接续准备端的快照。
        let (ticket, mut work) = writer.prepare_scan_work().unwrap();
        assert!(work.accumulate_mapping_frame(1, &[Point3::new(2.0, 0.0, 0.0)]));
        work.commit_accumulated().unwrap();
        let prepared = work.shared_committed_snapshot().unwrap();
        assert!(Arc::ptr_eq(&first.xyz_f32_le, &writer.shared_committed_snapshot().unwrap().xyz_f32_le));
        assert!(writer.install_scan_work(uuid::Uuid::new_v4(), CloudMapShmWriter::new_private_for_test().unwrap()).is_err());
        writer.install_scan_work(ticket, work).unwrap();
        let installed = writer.shared_committed_snapshot().unwrap();
        assert!(Arc::ptr_eq(&prepared.xyz_f32_le, &installed.xyz_f32_le));
        assert_eq!(installed.point_count, 2);
        assert_eq!(first.xyz_f32_le.as_ref(), original.as_slice());

        // 共享内存元数据损坏仍拒绝，不因已有快照绕过槽位校验。
        let revision = writer.revision;
        writer.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8].fill(0);
        assert!(writer.shared_committed_snapshot().is_none());
        writer.mmap[CM_GLOBAL_REVISION_OFFSET..CM_GLOBAL_REVISION_OFFSET + 8].copy_from_slice(&revision.to_le_bytes());
        writer.begin_session(2).unwrap();
        let reset = writer.shared_committed_snapshot().unwrap();
        assert_eq!(reset.session_generation, 2);
        assert!(reset.xyz_f32_le.is_empty());
        assert_eq!(installed.point_count, 2);
        assert_eq!(first.xyz_f32_le.as_ref(), original.as_slice());
    }

    #[test]
    #[ignore = "大点云重复读取对照，显式运行并记录耗时"]
    fn committed_snapshot_read_budget() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        writer.begin_session(1).unwrap();
        // 使用真实提交入口生成接近共享槽容量的规范载荷。
        let points: Vec<_> = (0..1_300_000).map(|i| Point3::new(
            (i % 1000) as f64 * 0.1, (i / 1000) as f64 * 0.1, 0.0)).collect();
        for chunk in points.chunks(POINTCLOUD_MAX_FRAME_POINTS) {
            assert!(writer.accumulate_mapping_frame(1, chunk));
        }
        writer.commit_accumulated().unwrap();
        let started = std::time::Instant::now();
        let first = writer.shared_committed_snapshot().unwrap();
        let cold_ms = started.elapsed().as_secs_f64() * 1000.0;
        let started = std::time::Instant::now();
        for _ in 0..7 {
            std::hint::black_box(writer.shared_committed_snapshot().unwrap());
        }
        let repeated_7_ms = started.elapsed().as_secs_f64() * 1000.0;
        let started = std::time::Instant::now();
        for _ in 0..700 {
            let snapshot = writer.shared_committed_snapshot().unwrap();
            assert!(Arc::ptr_eq(&first.xyz_f32_le, &snapshot.xyz_f32_le));
            std::hint::black_box(snapshot);
        }
        println!("snapshot_bytes={} cold_ms={cold_ms:.3} repeated_7_ms={repeated_7_ms:.6} repeated_700_ms={:.3}",
            first.xyz_f32_le.len(), started.elapsed().as_secs_f64() * 1000.0);
    }

    #[test]
    #[ignore = "需要显式提供已冻结位姿的记录，仅用于同源性能测量"]
    fn dynamic_recorded_scan_benchmark() {
        use std::io::Read;
        let path = std::env::var("NUWAX_DYNAMIC_RECORDED_SCANS").expect("指定配对扫描文件");
        let limit = std::env::var("NUWAX_DYNAMIC_BENCHMARK_FRAMES").ok().and_then(|v| v.parse::<usize>().ok()).unwrap_or(300);
        let origin: [f64; 3] = serde_json::from_str(&std::env::var("NUWAX_DYNAMIC_SENSOR_ORIGIN_BODY").expect("指定真实雷达机体原点")).unwrap();
        let mut file = std::io::BufReader::new(std::fs::File::open(path).unwrap());
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let legacy = std::env::var_os("NUWAX_DYNAMIC_BENCHMARK_LEGACY").is_some();
        if legacy {
            writer.reset_accumulator(1);
            writer.dynamic_filter.legacy_only = true;
            let mut fine = FineCloud::fresh().unwrap();
            fine.use_legacy_filter();
            writer.fine_cloud = Some(fine);
        }
        let mut durations = Vec::new();
        let mut stages = [0.0; 3];
        let mut count = 0;
        for _ in 0..limit {
            let mut header = [0; 68];
            match file.read_exact(&mut header) {
                Ok(()) => (),
                Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => break,
                Err(error) => panic!("读取扫描失败：{error}"),
            }
            let stamp = u64::from_le_bytes(header[..8].try_into().unwrap());
            let pose: [f64; 7] = std::array::from_fn(|i| f64::from_le_bytes(header[8 + i * 8..16 + i * 8].try_into().unwrap()));
            let transform = nalgebra::Isometry3::from_parts(nalgebra::Translation3::new(pose[0], pose[1], pose[2]),
                nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(pose[6], pose[3], pose[4], pose[5])));
            let size = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
            assert!(size <= POINTCLOUD_MAX_FRAME_POINTS);
            let mut bytes = vec![0; size * 12];
            file.read_exact(&mut bytes).unwrap();
            let points = bytes.chunks_exact(12).map(|b| transform * Point3::new(
                f32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
                f32::from_le_bytes(b[4..8].try_into().unwrap()) as f64,
                f32::from_le_bytes(b[8..12].try_into().unwrap()) as f64)).collect::<Vec<_>>();
            let origin = (transform * Point3::from(origin)).coords.into();
            let start = std::time::Instant::now();
            assert!(writer.accumulate_mapping_scan(1, &points, Some(origin), stamp));
            durations.push(start.elapsed().as_secs_f64() * 1000.0);
            for i in 0..3 { stages[i] += writer.last_stage_ms[i]; }
            count += size;
        }
        durations.sort_by(f64::total_cmp);
        assert!(!durations.is_empty());
        eprintln!("{}", serde_json::json!({"frames": durations.len(), "source_points": count,
            "map_voxels": writer.voxels.len(), "stage_mean_ms": stages.map(|v| v / durations.len() as f64), "mean_ms": durations.iter().sum::<f64>() / durations.len() as f64,
            "p50_ms": durations[durations.len()/2], "p95_ms": durations[durations.len()*95/100],
            "max_ms": durations.last().unwrap(), "labeled_humans": false}));
    }

    #[test]
    fn passing_person_is_live_obstacle_but_not_dense_history_and_survives_save_restore() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let mut static_total = 0_u64;
        let mut people_total = 0_u64;
        let mut last_receipt = None;
        for frame in 1..=22 {
            let person = (7..=16).contains(&frame).then(|| -0.5 + (frame - 7) as f64 * 0.12);
            let (points, labels) = corridor(person, (frame % 3) as f64);
            people_total += labels.iter().filter(|label| **label).count() as u64;
            static_total += labels.iter().filter(|label| !**label).count() as u64;
            assert!(writer.accumulate_mapping_scan(1, &points, Some(ORIGIN), frame * 100_000_000));
            // 过滤后的导航源仍包含本帧所有人体回波，不能让静态显示需求抹除避障依据。
            for point in points.iter().zip(&labels).filter(|(_, label)| **label).map(|(point, _)| point) {
                assert!(writer.voxels.contains(&voxel_key([point.x, point.y, point.z], writer.voxel_size).unwrap()));
            }
            let receipt = OccupancyCommitReceipt {
                commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 80, sequence: frame },
                session_id: "人员经过".into(), session_generation: 1, occupancy_revision: frame,
                source_timestamp_ns: frame * 100_000_000, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                backend_generation: 1,
            };
            writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
            last_receipt = Some(receipt);
        }
        let snapshot = writer.committed_dense_cloud().unwrap().unwrap();
        let mut offset = 0;
        let mut remaining_person = 0;
        let mut remaining_static = 0;
        while offset < snapshot.point_count {
            let bytes = snapshot.read(offset, crate::dense_cloud::CHUNK_POINTS).unwrap();
            offset += bytes.len() as u64 / 12;
            for bytes in bytes.chunks_exact(12) {
                let xyz: [f32; 3] = std::array::from_fn(|axis| f32::from_le_bytes(bytes[axis * 4..axis * 4 + 4].try_into().unwrap()));
                if xyz[0] > 2.7 && xyz[0] < 3.3 && xyz[2] > 0.1 && xyz[2] < 1.9 { remaining_person += 1; }
                else { remaining_static += 1; }
            }
        }
        eprintln!("完整扫描人体原始={people_total} 剩余={remaining_person} 静态原始={static_total} 保留={remaining_static}");
        assert!(remaining_person * 5 < people_total);
        assert_eq!(remaining_static, static_total);
        let (mut archive, _) = snapshot.archive("map").unwrap();
        crate::dense_cloud::validate_archive(&mut archive).unwrap();
        let restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
        let restored = restored.snapshot(&snapshot.receipt, "人员经过").unwrap();
        assert_eq!(restored.point_count, snapshot.point_count);
        assert_eq!(restored.read(0, crate::dense_cloud::CHUNK_POINTS).unwrap(), snapshot.read(0, crate::dense_cloud::CHUNK_POINTS).unwrap());
        let receipt = last_receipt.unwrap();
        let mut next = CloudMapShmWriter::new_private_for_test().unwrap();
        next.install_prepared_accumulator(&receipt, writer.take_prepared_accumulator()).unwrap();
        assert_eq!(next.committed_dense_cloud().unwrap().unwrap().point_count, snapshot.point_count);
        assert!(next.accumulate_mapping_scan(2, &corridor(Some(0.0), 0.0).0, Some(ORIGIN), 1));
        assert!(next.fine_cloud.as_ref().unwrap().removed_keys().is_empty(), "换图不复用旧动态证据");
    }

    #[test]
    fn prepared_scan_keeps_previous_dense_reader_until_install() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let first = vec![Point3::new(1.0, 0.0, 0.5), Point3::new(1.2, 0.0, 0.5)];
        assert!(writer.accumulate_mapping_scan(1, &first, None, 100));
        let receipt = |sequence, timestamp| OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 91, sequence },
            session_id: "准备读者".into(), session_generation: 1, occupancy_revision: sequence,
            source_timestamp_ns: timestamp, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        writer.commit_accumulated_snapshot(&receipt(1, 100)).unwrap().unwrap();
        let old = writer.committed_dense_cloud().unwrap().unwrap();
        let (ticket, mut work) = writer.prepare_scan_work().unwrap();
        let second = vec![Point3::new(2.0, 0.0, 0.5)];
        assert!(work.accumulate_mapping_scan(1, &second, None, 200));
        work.commit_accumulated_snapshot(&receipt(2, 200)).unwrap().unwrap();
        let reader = writer.dense_readers().unwrap();
        let pinned = reader.pin_read(old.clone(), &old.epoch, 0).unwrap();
        assert_eq!(pinned.point_count, old.point_count);
        writer.install_scan_work(ticket, work).unwrap();
        assert!(writer.committed_dense_cloud().unwrap().unwrap().point_count > old.point_count);
    }

    #[test]
    fn dense_cloud_follows_commits_rebuilds_and_checkpoint_availability() {
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let points = vec![Point3::new(0.011, 0.012, 0.013), Point3::new(0.014, 0.012, 0.013)];
        let receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 17, sequence: 1 },
            session_id: "完整扫描".into(), session_generation: 1, occupancy_revision: 1,
            source_timestamp_ns: 100, backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        assert!(writer.accumulate_mapping_scan(1, &points, None, 100));
        assert!(writer.committed_dense_cloud().unwrap().is_none());
        let commit = writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
        assert!(!writer.full_snapshot_materialized());
        let explicit_checkpoint = writer.committed_snapshot().unwrap();
        assert_eq!(commit.snapshot.point_count, 1);
        let dense = writer.committed_dense_cloud().unwrap().unwrap();
        assert_eq!(dense.point_count, 2);
        assert!(writer.accumulate_mapping_scan(1, &points, None, 200));
        assert_eq!(writer.committed_dense_cloud().unwrap().unwrap().point_count, 2);

        let mut rebuilt = CloudMapShmWriter::new_private_for_test().unwrap();
        let corrected = points.iter().map(|p| p + nalgebra::Vector3::new(1., 0., 0.)).collect::<Vec<_>>();
        assert!(rebuilt.accumulate_mapping_scan(1, &corrected, None, 200));
        let receipt2 = OccupancyCommitReceipt { commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 17, sequence: 2 },
            occupancy_revision: 2, source_timestamp_ns: 200, ..receipt.clone() };
        writer.install_prepared_accumulator(&receipt2, rebuilt.take_prepared_accumulator()).unwrap();
        let current = writer.committed_dense_cloud().unwrap().unwrap();
        assert_ne!(current.epoch, dense.epoch);
        assert_eq!(current.point_count, 2);
        assert_eq!(f32::from_le_bytes(current.read(0, 2).unwrap()[..4].try_into().unwrap()), 1.011_f32);
        assert_eq!(f32::from_le_bytes(dense.read(0, 2).unwrap()[..4].try_into().unwrap()), 0.011_f32);

        let mut legacy = CloudMapShmWriter::new_private_for_test().unwrap();
        legacy.install_checkpoint(&receipt, commit.snapshot.voxel_size, commit.snapshot.point_count, &explicit_checkpoint.xyz_f32_le).unwrap();
        assert!(legacy.accumulate_mapping_scan(1, &points, None, 200));
        legacy.commit_accumulated_snapshot(&receipt2).unwrap().unwrap();
        assert!(legacy.committed_dense_cloud().unwrap().is_none(), "旧资产不能以新扫描后缀冒充完整历史");

        assert!(writer.accumulate_mapping_scan(2, &points, None, 300));
        assert!(writer.committed_dense_cloud().unwrap().is_none(), "换会话不能泄漏旧地图");
    }

    #[test]
    fn fine_cloud_survives_display_coarsening_and_seals_the_exact_commit() {
        let path = test_path("fine-cloud-coarsening");
        let mut writer = CloudMapShmWriter::new_for_test(&path).unwrap();
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
        let root = fine.manifest();
        assert_eq!(root.domain, domain);
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
        assert_eq!(next.manifest().points(), 31);
        writer.cleanup_product_owned_file().unwrap();
    }

    #[test]
    fn pgo_accumulator_install_preserves_pending_dynamic_evidence() {
        let mut source = CloudMapShmWriter::new_for_test(&test_path("pgo-private-dynamic")).unwrap();
        let mut target = CloudMapShmWriter::new_for_test(&test_path("pgo-installed-dynamic")).unwrap();
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).unwrap();
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");

        writer
            .cleanup_product_owned_file()
            .expect("owned cloud mmap cleanup");

        assert!(!path.exists());
    }

    #[test]
    fn product_cleanup_rejects_replaced_cloud_mmap() {
        let path = test_path("replaced-cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 1);

        assert!(writer.accumulate_mapping_frame(1, &second_frame));
        let second = writer
            .commit_accumulated()
            .expect("second accumulated snapshot");
        assert_eq!(second.0, 2);
        assert_eq!(second.1, 2);
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 2);
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 2);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn begin_session_publishes_one_zero_frame_per_generation() {
        let path = test_path("session-clear");
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");

        let first_clear = writer.begin_session(1).expect("first session clear");
        assert_eq!(first_clear, (1, 0, VOXEL_SIZE));
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 1);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 0);
        assert_eq!(writer.published_pages.len(), 0);
        assert_eq!(sequence_atomic(&writer.mmap).load(Ordering::Acquire) & 1, 0);
        assert!(writer.begin_session(1).is_none());
        assert_eq!(writer.revision, 1, "same generation must be idempotent");

        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(0.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap().0, 2);
        let second_clear = writer.begin_session(2).expect("second session clear");
        assert_eq!(second_clear, (3, 0, VOXEL_SIZE));
        assert!(writer.voxels.is_empty());
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 0);
        assert_eq!(writer.published_pages.len(), 0);
        assert_eq!(sequence_atomic(&writer.mmap).load(Ordering::Acquire) & 1, 0);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn restored_checkpoint_accepts_later_live_frames() {
        let path = test_path("checkpoint-restore");
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        assert!(!writer.full_snapshot_materialized());
        let explicit = writer.committed_snapshot().unwrap();
        assert_eq!(explicit.xyz_f32_le, xyz_f32_le);
        assert_eq!(restored.receipt.commit_id, occupancy_receipt.commit_id);
        assert_eq!(restored.receipt.occupancy_revision, 11);
        assert_eq!(restored.receipt.cloud_revision, 1);
        assert_eq!(writer.session_generation, 7);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 2);

        // 激活回执有时间，但没有精细资产的旧点必须保留未知观察时间。
        let fine = writer.committed_fine_cloud().unwrap();
        let root = fine.manifest();
        assert_eq!(root.domain.session, occupancy_receipt.session_id);
        assert_eq!(root.domain.generation, 7);
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
    fn v3_point_count_and_layout_shrink_with_the_committed_revision() {
        let path = test_path("v2-slot-metadata");
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
        let first_session = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        assert!(writer.accumulate_mapping_frame(1, &first_session));
        assert_eq!(writer.commit_accumulated().unwrap(), (1, 3, VOXEL_SIZE));

        assert!(writer.accumulate_mapping_frame(1, &[Point3::new(3.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap(), (2, 4, VOXEL_SIZE));
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET), 2);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 4);
        let layout = writer.layout_generation;

        assert!(writer.accumulate_mapping_frame(2, &[Point3::new(10.0, 0.0, 0.0)]));
        assert_eq!(writer.commit_accumulated().unwrap(), (3, 1, VOXEL_SIZE));
        assert!(writer.layout_generation > layout);
        assert_eq!(read_u64(&writer.mmap, CM_GLOBAL_REVISION_OFFSET), 3);
        assert_eq!(read_u32(&writer.mmap, CM_GLOBAL_POINT_COUNT_OFFSET), 1);
        assert_eq!(read_u64(&writer.mmap, CM_PAGE_TABLE_OFFSET), 3);
        assert_eq!(writer.published_pages[0].len(), 12);

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn oversized_single_frame_does_not_change_writer_state() {
        let path = test_path("single-frame-cap");
        let mut writer = CloudMapShmWriter::new_for_test(&path).expect("cloud map writer");
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
