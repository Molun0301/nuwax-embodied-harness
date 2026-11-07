//! 同一地图拥有的精细层、稠密原扫描和网页共享内存；不持有会话状态副本。
use crate::{
    dense_cloud::{DenseCloud, DenseSnapshot},
    dynamic_backend::{MappedFrame, OnlineDufo},
    mapper::{isometry_matrix, matrix_isometry, pose_from_isometry, pose_isometry, MappingFrame},
    proto,
};
use memmap2::MmapMut;
use nalgebra::{Isometry3, Point3};
use std::{
    fs::File,
    path::{Path, PathBuf},
    sync::{
        atomic::{fence, AtomicU64, Ordering},
        Arc,
    },
};
#[path = "dynamic_cloud.rs"]
mod dynamic_cloud;
#[path = "fine_cloud.rs"]
mod fine_cloud;
#[path = "paged_voxel_snapshot.rs"]
mod paged_voxel_snapshot;
use fine_cloud::FineCloud;
pub(crate) use fine_cloud::{
    configure_fine_cloud_cache, configured_writer_cache, parse_fine_cloud_cache,
};
use paged_voxel_snapshot::{PagedVoxelSnapshot, CHUNK_POINTS};

pub const VOXEL_SIZE: f32 = 0.05;
// 录制文件格式保留已有会话字段；导航地图无需依赖它。
pub(crate) const SCAN_LOG_GENERATION: u64 = 1;
const DEFAULT_CLOUDMAP_SHM_PATH: &str = include_str!("../proto/cloudmap_shm_path.txt");
pub const POINTCLOUD_MAX_FRAME_POINTS: usize = 100_000;
pub const POINTCLOUD_MAX_POINT_STEP_BYTES: u32 = 256;
pub(crate) const MAX_VOXEL_SIZE: f32 = 1_048_576.;
const CM_MAGIC: u64 = 0x4E55_5741_584D_4150;
const CM_HEADER: usize = 512;
const CM_MAX_POINTS: usize = 16 * 1024 * 1024 / 12;
const CM_CHUNKS: usize = CM_MAX_POINTS.div_ceil(CHUNK_POINTS);
const CM_PAYLOAD: usize = (CM_HEADER + CM_CHUNKS * 8).div_ceil(4096) * 4096;
const CM_TOTAL: usize = CM_PAYLOAD + CM_MAX_POINTS * 12;
type VoxelKey = (i32, i32, i32);
type VoxelIndex = ahash::AHashSet<VoxelKey>;
#[derive(Clone, Default)]
pub(crate) struct CloudSnapshot {
    pub fine: Option<Arc<nuwax_map_chunks::SealedMap>>,
    pub fine_archive: Option<Arc<File>>,
    pub dense: Option<DenseSnapshot>,
    pub display: Arc<[u8]>,
    pub voxel_size: f32,
    pub revision: u64,
    pub source_timestamp_ns: u64,
}
pub(crate) struct CloudMap {
    session_id: u64,
    fine: FineCloud,
    dense: DenseCloud,
    dufo: OnlineDufo,
    voxels: VoxelIndex,
    pages: PagedVoxelSnapshot,
    voxel_size: f32,
    revision: u64,
    stamp: u64,
    surface: Option<Arc<nuwax_map_chunks::SealedMap>>,
    dense_snapshot: Option<DenseSnapshot>,
    shm: Option<CloudMapShmWriter>,
    logs: std::collections::BTreeMap<String, nuwax_cloud_cleanup::scan_log::ScanLog>,
    last_recorded: std::collections::BTreeMap<String, nuwax_cloud_cleanup::scan_log::ScanMetadata>,
    save_policy: nuwax_cloud_cleanup::scan_log::SavePolicy,
    poses: Vec<(u64, [f64; 16])>,
    recording: bool,
    raw_directory: Option<PathBuf>,
    readonly_classifier: Option<FineCloud>,
    history_scans: Vec<(PathBuf, u64)>,
    dirty: bool,
}
impl CloudMap {
    pub fn new(
        session_id: u64,
        shm_path: Option<&Path>,
        snapshot: Option<CloudSnapshot>,
    ) -> Result<Self, String> {
        let mut fine = FineCloud::new(VOXEL_SIZE)?;
        let mut dense = DenseCloud::new()?;
        let mut voxels = VoxelIndex::new();
        let mut saved_order = Vec::new();
        let mut size = VOXEL_SIZE;
        let (mut revision, mut stamp) = (0, 0);
        let mut surface = None;
        if let Some(snapshot) = snapshot {
            size = snapshot.voxel_size;
            revision = snapshot.revision;
            stamp = snapshot.source_timestamp_ns;
            let archive = if let Some(file) = snapshot.fine_archive {
                Some(file.try_clone().map_err(|e| e.to_string())?)
            } else if let Some(root) = snapshot.fine {
                let (file, _) = nuwax_map_chunks::archive_file(root.manifest(), &root.repository)?;
                Some(file)
            } else {
                None
            };
            if let Some(file) = archive {
                let (restored, publication) = FineCloud::from_archive(file)?;
                fine = restored;
                if publication.domain == domain(session_id) {
                    revision = revision.max(publication.revision) + 1;
                }
            }
            if let Some(source) = snapshot.dense {
                let (file, _) = source.archive("map")?;
                let metadata = source.metadata("map");
                dense = DenseCloud::from_archive(file, &metadata)?;
            }
            for bytes in snapshot.display.chunks_exact(12) {
                let point = [
                    f32::from_le_bytes(bytes[0..4].try_into().unwrap()) as f64,
                    f32::from_le_bytes(bytes[4..8].try_into().unwrap()) as f64,
                    f32::from_le_bytes(bytes[8..12].try_into().unwrap()) as f64,
                ];
                let key = voxel_key(point, size).ok_or("地图点云坐标无效")?;
                voxels.insert(key);
                saved_order.push(key);
            }
            if stamp > 0 {
                surface = Some(fine.seal(domain(session_id), revision.max(1), stamp)?);
            }
        }
        let saved_size = size;
        while voxels.len() > CM_MAX_POINTS {
            let next = size * 2.;
            voxels = voxels
                .iter()
                .filter_map(|key| voxel_key(voxel_center_f64(*key, size), next))
                .collect();
            size = next;
        }
        let mut pages = PagedVoxelSnapshot::default();
        if size == saved_size {
            for key in saved_order {
                pages.inserted(key, CM_MAX_POINTS);
            }
        } else {
            pages.invalidate();
            pages.prepare(&voxels);
        }
        let shm = shm_path
            .map(CloudMapShmWriter::new)
            .transpose()
            .map_err(|e| e.to_string())?;
        let dense_snapshot = dense.snapshot(stamp, revision, &session_id.to_string());
        let mut value = Self {
            session_id,
            fine,
            dense,
            dufo: Default::default(),
            voxels,
            pages,
            voxel_size: size,
            revision,
            stamp,
            surface,
            dense_snapshot,
            shm,
            logs: Default::default(),
            last_recorded: Default::default(),
            save_policy: nuwax_cloud_cleanup::scan_log::SavePolicy::from_runtime()?,
            poses: Vec::new(),
            recording: true,
            raw_directory: None,
            readonly_classifier: None,
            history_scans: Vec::new(),
            dirty: false,
        };
        value.publish()?;
        Ok(value)
    }
    pub fn attach_shared(&mut self, path: &Path) -> Result<(), String> {
        self.shm = Some(CloudMapShmWriter::new(path).map_err(|e| e.to_string())?);
        self.pages.invalidate();
        self.publish()
    }
    pub fn accumulate(
        &mut self,
        sequence: u64,
        frame: &MappingFrame,
        points: &[Point3<f64>],
        origin: [f64; 3],
        map_from_odom: Isometry3<f64>,
    ) -> Result<(MappedFrame, Vec<u32>, Vec<u32>), String> {
        let odom = pose_isometry(&frame.odom_pose);
        let odom_points: Vec<_> = frame
            .points_body
            .iter()
            .map(|p| odom * Point3::from(*p))
            .collect();
        let origin_odom = odom * Point3::from(frame.ray_origin_body);
        let (mut dynamic, indices) = self.dufo.observe(
            &odom_points,
            [origin_odom.x, origin_odom.y, origin_odom.z],
            frame.odom_pose.timestamp_ns,
            map_from_odom,
        )?;
        dynamic.bind_points(points, &indices);
        let ground = self.fine.accumulate(
            self.session_id,
            &frame.sensor_id,
            sequence,
            points,
            origin,
            frame.odom_pose.timestamp_ns,
            Some(dynamic.clone()),
        )?;
        let removed = self.fine.removed_keys();
        self.dense.remove_voxels(removed, VOXEL_SIZE)?;
        let retained: Vec<_> = points
            .iter()
            .filter(|p| !self.fine.is_dynamic(p))
            .copied()
            .collect();
        self.dense.append(&retained)?;
        let hits: ahash::AHashSet<_> = points
            .iter()
            .filter_map(|p| voxel_key([p.x, p.y, p.z], self.voxel_size))
            .collect();
        let removed: Vec<_> = self
            .voxels
            .iter()
            .copied()
            .filter(|key| {
                !hits.contains(key) && dynamic.clears(voxel_center_f64(*key, self.voxel_size))
            })
            .collect();
        for key in removed {
            self.voxels.remove(&key);
            self.pages.removed(key, CM_MAX_POINTS);
        }
        for p in &retained {
            let key = voxel_key([p.x, p.y, p.z], self.voxel_size).ok_or("点云坐标无效")?;
            if self.voxels.insert(key) {
                self.pages.inserted(key, CM_MAX_POINTS);
            }
        }
        while self.voxels.len() > CM_MAX_POINTS {
            let next = self.voxel_size * 2.;
            if !next.is_finite() || next > MAX_VOXEL_SIZE {
                return Err("展示点云体素超过范围".into());
            }
            let keys = self
                .voxels
                .iter()
                .filter_map(|key| voxel_key(voxel_center_f64(*key, self.voxel_size), next))
                .collect();
            self.voxels = keys;
            self.voxel_size = next;
            self.pages.invalidate();
        }
        self.dirty = true;
        self.stamp = frame.odom_pose.timestamp_ns;
        self.poses
            .push((self.stamp, isometry_matrix(&(map_from_odom * odom))));
        if self.recording {
            self.record(sequence, frame, map_from_odom)?;
        }
        Ok((dynamic, indices, ground))
    }
    fn record(
        &mut self,
        sequence: u64,
        frame: &MappingFrame,
        transform: Isometry3<f64>,
    ) -> Result<(), String> {
        use nuwax_cloud_cleanup::scan_log::{ScanLog, ScanMetadata, Session};
        let metadata = ScanMetadata {
            source_timestamp_ns: frame.odom_pose.timestamp_ns,
            source_sequence: sequence,
            sensor: frame.sensor_id.clone(),
            source_generation: 1,
            odom_from_body: isometry_matrix(&pose_isometry(&frame.odom_pose)),
            map_from_odom: isometry_matrix(&transform),
            sensor_origin_body: Some(frame.ray_origin_body.map(|v| v as f32)),
            observation_minimum_range_m: Some(
                crate::dynamic_backend::online_params().min_range as f64,
            ),
            point_count: frame.points_body.len(),
        };
        let selected = self.save_policy.selects(self.last_recorded.get(&frame.sensor_id), &metadata);
        if self.raw_directory.is_none() {
            self.raw_directory = Some(
                nuwax_cloud_cleanup::scan_log::scan_directory()
                    .join(format!(".geometry-{}", uuid::Uuid::new_v4())),
            );
        }
        if !self.logs.contains_key(&frame.sensor_id) {
            let root = self
                .raw_directory
                .as_ref()
                .unwrap()
                .join(format!("source-{}", self.logs.len()));
            self.logs.insert(
                frame.sensor_id.clone(),
                ScanLog::open(
                    &root,
                    Session {
                        session_id: self.session_id.to_string(),
                        generation: SCAN_LOG_GENERATION,
                    },
                )?,
            );
        }
        let log = self.logs.get_mut(&frame.sensor_id).unwrap();
        if selected {
            log.append(
                &metadata,
                frame.points_body.iter().map(|p| p.map(|v| v as f32)),
                None,
            )?;
        }
        log.progress(frame.odom_pose.timestamp_ns, 0)?;
        if selected {
            self.last_recorded.insert(frame.sensor_id.clone(), metadata);
        }
        Ok(())
    }
    pub fn commit(&mut self, stamp: u64) -> Result<(), String> {
        if !self.dirty || self.stamp == 0 || stamp == 0 {
            return Ok(());
        }
        self.revision += 1;
        self.dirty = false;
        self.surface = Some(
            self.fine
                .seal(domain(self.session_id), self.revision, self.stamp)?,
        );
        self.fine.retain_committed(self.surface.as_ref().unwrap());
        self.dense_snapshot =
            self.dense
                .snapshot(self.stamp, self.revision, &self.session_id.to_string());
        self.publish()
    }
    fn publish(&mut self) -> Result<(), String> {
        let rebuild = self.pages.needs_rebuild();
        if rebuild {
            self.pages.prepare(&self.voxels);
        }
        if let Some(shm) = self.shm.as_mut() {
            if rebuild {
                shm.layout += 1;
            }
            shm.publish(
                self.session_id,
                self.revision,
                self.stamp,
                self.voxel_size,
                &self.pages,
            )?;
        }
        self.pages.committed();
        Ok(())
    }
    pub fn classify_readonly(
        &mut self,
        sequence: u64,
        frame: &MappingFrame,
        points: &[Point3<f64>],
        origin: [f64; 3],
        map_from_odom: Isometry3<f64>,
    ) -> Result<Vec<u32>, String> {
        if self.readonly_classifier.is_none() {
            let classifier = if let Some(root) = &self.surface {
                let (file, _) = nuwax_map_chunks::archive_file(root.manifest(), &root.repository)?;
                FineCloud::from_archive(file)?.0
            } else {
                FineCloud::new(VOXEL_SIZE)?
            };
            self.readonly_classifier = Some(classifier);
        }
        let odom = pose_isometry(&frame.odom_pose);
        let odom_points: Vec<_> = frame
            .points_body
            .iter()
            .map(|p| odom * Point3::from(*p))
            .collect();
        let origin_odom = odom * Point3::from(frame.ray_origin_body);
        let (mut dynamic, indices) = self.dufo.observe(
            &odom_points,
            [origin_odom.x, origin_odom.y, origin_odom.z],
            frame.odom_pose.timestamp_ns,
            map_from_odom,
        )?;
        dynamic.bind_points(points, &indices);
        self.readonly_classifier.as_mut().unwrap().accumulate(
            self.session_id,
            &frame.sensor_id,
            sequence,
            points,
            origin,
            frame.odom_pose.timestamp_ns,
            Some(dynamic),
        )
    }
    pub fn disable_recording(&mut self) {
        self.recording = false;
    }
    pub fn surface(&self) -> Option<Arc<nuwax_map_chunks::SealedMap>> {
        self.surface.clone()
    }
    pub fn snapshot(&self) -> CloudSnapshot {
        let display: Vec<u8> = self
            .pages
            .keys()
            .iter()
            .flat_map(|key| {
                voxel_center_f64(*key, self.voxel_size)
                    .into_iter()
                    .flat_map(|v| (v as f32).to_le_bytes())
            })
            .collect();
        CloudSnapshot {
            fine: self.surface.clone(),
            fine_archive: None,
            dense: self.dense_snapshot.clone(),
            display: display.into(),
            voxel_size: self.voxel_size,
            revision: self.revision,
            source_timestamp_ns: self.stamp,
        }
    }
    pub fn scan_prefix(&self) -> Result<Vec<(PathBuf, u64)>, String> {
        let mut values = self.history_scans.clone();
        for log in self.logs.values() {
            log.sync()?;
            values.push((
                log.path.clone(),
                std::fs::metadata(&log.path)
                    .map_err(|e| e.to_string())?
                    .len(),
            ));
        }
        Ok(values)
    }
    pub fn load_history(&mut self, scans: Vec<(PathBuf, u64)>, poses: Vec<(u64, [f64; 16])>) {
        self.history_scans = scans;
        self.poses = poses;
    }
    pub fn final_poses(&self) -> Vec<(u64, [f64; 16])> {
        self.poses.clone()
    }
    pub fn replay_frames(&self) -> Result<Vec<MappingFrame>, String> {
        let mut result = Vec::new();
        for log in self.logs.values() {
            log.sync()?;
            let mut reader = nuwax_cloud_cleanup::scan_log::ScanReader::open(&log.path)?;
            while let Some(scan) = reader.next_scan()? {
                result.push(MappingFrame {
                    session_id: self.session_id,
                    odom_pose: pose_from_isometry(
                        matrix_isometry(&scan.metadata.odom_from_body),
                        "odom",
                        scan.metadata.source_timestamp_ns,
                    ),
                    points_body: scan.points.into_iter().map(|p| p.map(f64::from)).collect(),
                    ray_origin_body: scan
                        .metadata
                        .sensor_origin_body
                        .ok_or("扫描缺少雷达原点")?
                        .map(f64::from),
                    mapping_only: false,
                    sensor_id: scan.metadata.sensor.clone(),
                    registered_cloud: None,
                });
            }
        }
        result.sort_by_key(|frame| frame.odom_pose.timestamp_ns);
        Ok(result)
    }
    pub fn adopt_publication(&mut self, previous: &mut Self) -> Result<(), String> {
        self.revision = previous.revision + 1;
        self.surface = Some(
            self.fine
                .seal(domain(self.session_id), self.revision, self.stamp)?,
        );
        self.dense_snapshot =
            self.dense
                .snapshot(self.stamp, self.revision, &self.session_id.to_string());
        self.shm = previous.shm.take();
        self.logs = std::mem::take(&mut previous.logs);
        self.last_recorded = std::mem::take(&mut previous.last_recorded);
        self.raw_directory = previous.raw_directory.take();
        self.recording = previous.recording;
        self.history_scans = std::mem::take(&mut previous.history_scans);
        self.pages.invalidate();
        self.publish()
    }
    pub fn dense_chunk(
        &mut self,
        request: proto::DenseCloudChunkRequest,
        frame_id: &str,
    ) -> Result<proto::DenseCloudChunk, String> {
        let latest = self.dense_snapshot.clone().ok_or("当前地图没有点云")?;
        let (snapshot, base) = if request.allow_delta {
            self.dense
                .pin_update(latest, &request.expected_epoch, request.offset_points)?
        } else {
            (
                self.dense
                    .pin_read(latest, &request.expected_epoch, request.offset_points)?,
                None,
            )
        };
        let removed_ranges = base
            .as_ref()
            .map(|old| snapshot.removed_prefix_ranges(old, request.offset_points))
            .transpose()?
            .unwrap_or_default();
        let removed_count: u64 = removed_ranges.chunks_exact(2).map(|range| range[1]).sum();
        let offset = request.offset_points - removed_count;
        let data = snapshot.read(offset, request.max_points)?;
        let mut result = proto::DenseCloudChunk {
            epoch: snapshot.epoch.clone(),
            source_timestamp_ns: snapshot.source_timestamp_ns,
            frame_id: frame_id.into(),
            total_points: snapshot.point_count,
            offset_points: offset,
            xyz_f32_le: data,
            bounds_min: snapshot.bounds_min.to_vec(),
            bounds_max: snapshot.bounds_max.to_vec(),
            base_epoch: String::new(),
            base_offset_points: 0,
            removed_ranges,
        };
        if let Some(base) = base {
            result.base_epoch = base.epoch.clone();
            result.base_offset_points = request.offset_points;
        }
        Ok(result)
    }
}
fn domain(session_id: u64) -> nuwax_map_chunks::Domain {
    nuwax_map_chunks::Domain {
        session: session_id.to_string(),
        }
}
pub(crate) fn cloud_map_shm_path() -> std::path::PathBuf {
    std::path::PathBuf::from(DEFAULT_CLOUDMAP_SHM_PATH.trim())
}
struct CloudMapShmWriter {
    mmap: MmapMut,
    sequence: u64,
    layout: u64,
    epoch: [u8; 16],
    path: PathBuf,
    device: u64,
    inode: u64,
}
impl CloudMapShmWriter {
    fn new(path: &Path) -> std::io::Result<Self> {
        let temp = path.with_extension(format!("{}.tmp", uuid::Uuid::new_v4()));
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&temp)?;
        let result = (|| {
            file.set_len(CM_TOTAL as u64)?;
            let mut mmap = unsafe { MmapMut::map_mut(&file)? };
            mmap[..CM_HEADER].fill(0);
            mmap[0..8].copy_from_slice(&CM_MAGIC.to_le_bytes());
            mmap[8..12].copy_from_slice(&3u32.to_le_bytes());
            mmap[12..16].copy_from_slice(&(CM_HEADER as u32).to_le_bytes());
            mmap[68..72].copy_from_slice(&(CM_MAX_POINTS as u32).to_le_bytes());
            mmap[72..76].copy_from_slice(&(CHUNK_POINTS as u32).to_le_bytes());
            mmap[76..80].copy_from_slice(&(CM_CHUNKS as u32).to_le_bytes());
            mmap[96..104].copy_from_slice(&(CM_HEADER as u64).to_le_bytes());
            mmap[104..112].copy_from_slice(&(CM_PAYLOAD as u64).to_le_bytes());
            mmap[112..120].copy_from_slice(&(CM_TOTAL as u64).to_le_bytes());
            std::fs::rename(&temp, path)?;
            use std::os::unix::fs::MetadataExt;
            let metadata = file.metadata()?;
            Ok(Self {
                mmap,
                sequence: 0,
                layout: 1,
                epoch: *uuid::Uuid::new_v4().as_bytes(),
                path: path.to_path_buf(),
                device: metadata.dev(),
                inode: metadata.ino(),
            })
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(temp);
        }
        result
    }
    fn publish(
        &mut self,
        session: u64,
        revision: u64,
        stamp: u64,
        size: f32,
        pages: &PagedVoxelSnapshot,
    ) -> Result<(), String> {
        self.sequence += 2;
        let sequence = unsafe { &*(self.mmap.as_ptr().add(16).cast::<AtomicU64>()) };
        sequence.store(self.sequence - 1, Ordering::SeqCst);
        fence(Ordering::SeqCst);
        for page in pages.dirty_pages() {
            let start = page * CHUNK_POINTS;
            if start >= pages.keys().len() {
                continue;
            }
            for (index, key) in pages.keys()[start..pages.keys().len().min(start + CHUNK_POINTS)]
                .iter()
                .enumerate()
            {
                for (axis, coordinate) in voxel_center_f64(*key, size).into_iter().enumerate() {
                    let offset = CM_PAYLOAD + (start + index) * 12 + axis * 4;
                    self.mmap[offset..offset + 4]
                        .copy_from_slice(&(coordinate as f32).to_le_bytes());
                }
            }
            self.mmap[CM_HEADER + page * 8..CM_HEADER + page * 8 + 8]
                .copy_from_slice(&revision.to_le_bytes());
        }
        self.mmap[24..32].copy_from_slice(&revision.to_le_bytes());
        self.mmap[32..40].copy_from_slice(&stamp.to_le_bytes());
        self.mmap[40..48].copy_from_slice(&self.layout.to_le_bytes());
        self.mmap[48..64].copy_from_slice(&self.epoch);
        self.mmap[64..68].copy_from_slice(&(pages.keys().len() as u32).to_le_bytes());
        self.mmap[80..84].copy_from_slice(&size.to_le_bytes());
        self.mmap[88..96].copy_from_slice(&session.to_le_bytes());
        unsafe { &*(self.mmap.as_ptr().add(16).cast::<AtomicU64>()) }
            .store(self.sequence, Ordering::Release);
        Ok(())
    }
}
fn voxel_center_f64(key: VoxelKey, size: f32) -> [f64; 3] {
    [key.0, key.1, key.2].map(|v| (f64::from(v) + 0.5) * f64::from(size))
}
pub(crate) fn voxel_key(point: [f64; 3], size: f32) -> Option<VoxelKey> {
    if !size.is_finite() || size <= 0. || point.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let mut key = [0; 3];
    for (i, value) in point.into_iter().enumerate() {
        let value = (value / f64::from(size)).floor();
        if value < f64::from(i32::MIN) || value > f64::from(i32::MAX) {
            return None;
        }
        key[i] = value as i32;
    }
    Some((key[0], key[1], key[2]))
}

impl Drop for CloudMap {
    fn drop(&mut self) {
        self.logs.clear();
        if let Some(path) = self.raw_directory.take() {
            let _ = std::fs::remove_dir_all(path);
        }
    }
}

impl Drop for CloudMapShmWriter {
    fn drop(&mut self) {
        // 清空仍映射旧文件的网页读者；只删除本对象实际创建的文件。
        let mut empty = PagedVoxelSnapshot::default();
        empty.invalidate();
        empty.prepare(&VoxelIndex::new());
        let _ = self.publish(1, 0, 0, VOXEL_SIZE, &empty);
        use std::os::unix::fs::MetadataExt;
        if std::fs::metadata(&self.path)
            .is_ok_and(|m| m.dev() == self.device && m.ino() == self.inode)
        {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

#[cfg(test)]
#[path = "pointcloud_tests.rs"]
mod tests;
