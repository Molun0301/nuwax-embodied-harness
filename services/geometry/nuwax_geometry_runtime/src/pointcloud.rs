use std::collections::HashSet;
use std::sync::atomic::{fence, Ordering};

use memmap2::MmapMut;
use nalgebra::Point3;
use tracing::{error, info, warn};

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
}

/// One successfully committed cumulative cloud-map revision and the exact
/// canonical map-frame voxel snapshot available to typed transport sinks.
pub(crate) struct CommittedCloudMapSnapshot {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub xyz_f32_le: Vec<u8>,
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
        })
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
    pub(crate) fn commit_accumulated_snapshot(&mut self) -> Option<CommittedCloudMapSnapshot> {
        let (revision, point_count, voxel_size) = self.commit_accumulated()?;
        let mut keys = self.voxels.iter().copied().collect::<Vec<_>>();
        keys.sort_unstable();
        let mut xyz_f32_le = Vec::with_capacity(keys.len() * CM_POINT_SIZE_BYTES);
        for key in keys {
            for coordinate in voxel_center_f32(key, voxel_size) {
                xyz_f32_le.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        debug_assert_eq!(xyz_f32_le.len() / CM_POINT_SIZE_BYTES, point_count as usize);
        Some(CommittedCloudMapSnapshot {
            revision,
            point_count,
            voxel_size,
            xyz_f32_le,
        })
    }

    /// Fence a newly started mapping session from the prior session's 3D map.
    /// A successful call publishes a new revision with a slot-local point count
    /// of zero. Replaying the same generation is idempotent.
    pub(crate) fn begin_session(&mut self, session_generation: u64) -> Option<(u64, u32, f32)> {
        if session_generation == 0 || self.session_generation == session_generation {
            return None;
        }
        self.reset_accumulator(session_generation);
        self.publish_snapshot(true)
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
        session_generation: u64,
        voxel_size: f32,
        point_count: u32,
        xyz_f32_le: &[u8],
    ) -> Result<(u64, u32, f32), String> {
        if session_generation == 0 {
            return Err("cloud-map checkpoint generation must be positive".to_string());
        }
        self.revision
            .checked_add(1)
            .ok_or_else(|| "cloud-map revision exhausted".to_string())?;
        let voxels =
            decode_checkpoint_voxels(voxel_size, point_count, xyz_f32_le, self.max_points)?;
        self.session_generation = session_generation;
        self.voxel_size = voxel_size;
        self.voxels = voxels;
        self.publish_snapshot(false)
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
        for (index, point) in self
            .voxels
            .iter()
            .map(|key| voxel_center_f32(*key, self.voxel_size))
            .enumerate()
        {
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
        let checkpoint_points = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]];
        let xyz_f32_le = checkpoint_points
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();

        let restored = writer
            .install_checkpoint(7, VOXEL_SIZE, 2, &xyz_f32_le)
            .expect("restore checkpoint");
        assert_eq!(restored, (1, 2, VOXEL_SIZE));
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
