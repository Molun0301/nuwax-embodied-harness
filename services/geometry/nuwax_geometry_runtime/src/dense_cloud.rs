//! 已接纳扫描的原坐标归档；不可变可见性快照同步撤销已证实动态的历史点。
use crate::mapping_commit::CloudCommitReceipt;
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    os::unix::fs::{FileExt, OpenOptionsExt},
    sync::Arc,
};

pub(crate) const MAX_ARCHIVE_BYTES: u64 = 8 * 1024 * 1024 * 1024;
pub(crate) const CHUNK_POINTS: u32 = 65_536;
const MAGIC: &[u8; 8] = b"NUWAXDC1";
const MAX_HEADER_BYTES: usize = 65_536;
const PAGE_POINTS: usize = 4096;
const INDEX_BLOCK_M: f64 = 1.6;
const MAX_INDEX_ENTRIES: usize = 1_000_000;
type Key = (i32, i32, i32);

#[derive(Clone)]
struct Page {
    length: usize,
    visible: [u64; PAGE_POINTS / 64],
    count: u64,
    low: [f32; 3],
    high: [f32; 3],
}

impl Default for Page {
    fn default() -> Self {
        Self {
            length: 0,
            visible: [0; PAGE_POINTS / 64],
            count: 0,
            low: [f32::INFINITY; 3],
            high: [f32::NEG_INFINITY; 3],
        }
    }
}

fn index_block(point: [f64; 3]) -> Key {
    let p = point.map(|v| (v / INDEX_BLOCK_M).floor() as i32);
    (p[0], p[1], p[2])
}

fn decode_point(bytes: &[u8]) -> [f32; 3] {
    std::array::from_fn(|i| f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap()))
}

pub(crate) struct DenseCloud {
    file: Arc<File>,
    keys_file: File,
    voxel_size: f32,
    epoch: String,
    point_count: u64,
    stored_count: u64,
    pages: Arc<Vec<Arc<Page>>>,
    block_pages: Option<HashMap<Key, HashSet<usize>>>,
    block_entries: usize,
    live_keys: Option<HashSet<Key>>,
    readers: DenseCloudReaders,
    bounds_min: [f32; 3],
    bounds_max: [f32; 3],
}

/// 读者只拥有已提交的不可变快照，独立于正在计算的归档写入器。
#[derive(Clone, Default)]
pub(crate) struct DenseCloudReaders(Arc<parking_lot::Mutex<std::collections::VecDeque<(std::time::Instant, DenseSnapshot)>>>);

#[derive(Clone)]
pub(crate) struct DenseSnapshot {
    file: Arc<File>,
    pages: Arc<Vec<Arc<Page>>>,
    page_ends: Arc<Vec<u64>>,
    pub epoch: String,
    pub point_count: u64,
    pub bounds_min: [f32; 3],
    pub bounds_max: [f32; 3],
    pub receipt: CloudCommitReceipt,
    pub session_id: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Metadata {
    pub version: u32,
    pub session_id: String,
    pub session_generation: u64,
    pub cloud_revision: u64,
    pub source_timestamp_ns: u64,
    pub frame_id: String,
    pub point_count: u64,
    pub bounds_min: [f32; 3],
    pub bounds_max: [f32; 3],
}

fn temporary_file() -> Result<File, String> {
    let path = std::env::temp_dir().join(format!("nuwax-dense-cloud-{}", uuid::Uuid::new_v4()));
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(&path)
        .map_err(|e| e.to_string())?;
    std::fs::remove_file(path).map_err(|e| e.to_string())?;
    Ok(file)
}

impl DenseCloud {
    #[cfg(test)]
    pub fn new() -> Result<Self, String> {
        Self::with_voxel_size(crate::pointcloud::VOXEL_SIZE)
    }

    pub fn with_voxel_size(voxel_size: f32) -> Result<Self, String> {
        if !voxel_size.is_finite() || voxel_size <= 0.0 {
            return Err("完整点云索引精度非法".into());
        }
        Ok(Self {
            file: Arc::new(temporary_file()?),
            keys_file: temporary_file()?,
            voxel_size,
            epoch: uuid::Uuid::new_v4().simple().to_string(),
            point_count: 0,
            stored_count: 0,
            pages: Arc::new(Vec::new()),
            block_pages: Some(HashMap::new()),
            block_entries: 0,
            live_keys: Some(HashSet::new()),
            readers: Default::default(),
            bounds_min: [f32::INFINITY; 3],
            bounds_max: [f32::NEG_INFINITY; 3],
        })
    }

    pub fn append(&mut self, points: &[Point3<f64>]) -> Result<(), String> {
        let count = self
            .stored_count
            .checked_add(points.len() as u64)
            .ok_or("完整点云计数溢出")?;
        if count > (MAX_ARCHIVE_BYTES - MAX_HEADER_BYTES as u64 - 12) / 12 {
            return Err(
                "完整点云存储达到8GiB容量，无法继续保留全部扫描；请保存后开始新地图".into(),
            );
        }
        let mut bytes = Vec::with_capacity(points.len() * 12);
        let mut keys = Vec::with_capacity(points.len() * 12);
        let (mut low, mut high) = (self.bounds_min, self.bounds_max);
        for point in points {
            // 在降为单精度之前保存与精细地图完全相同的体素键，避免边界舍入导致漏删/误删。
            let key = crate::pointcloud::voxel_key([point.x, point.y, point.z], self.voxel_size)
                .ok_or("完整点云索引坐标越界")?;
            for value in [key.0, key.1, key.2] {
                keys.extend_from_slice(&value.to_le_bytes());
            }
            for (axis, coordinate) in [point.x, point.y, point.z].into_iter().enumerate() {
                let value = coordinate as f32;
                if !value.is_finite() {
                    return Err("完整点云包含无效坐标".into());
                }
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }
        // 使用显式偏移；读者持有旧前缀，追加不能改变其内容或共享文件游标。
        self.file
            .write_all_at(&bytes, self.stored_count * 12)
            .map_err(|e| e.to_string())?;
        self.keys_file
            .write_all_at(&keys, self.stored_count * 12)
            .map_err(|e| e.to_string())?;
        let pages = Arc::make_mut(&mut self.pages);
        // 每页只执行一次写时复制；旧快照仍持有追加前的可见性。
        let mut index = 0;
        while index < points.len() {
            let position = self.stored_count as usize + index;
            let page_index = position / PAGE_POINTS;
            if page_index == pages.len() {
                pages.push(Arc::new(Page::default()));
            }
            let page = Arc::make_mut(&mut pages[page_index]);
            let end = points.len().min(index + PAGE_POINTS - position % PAGE_POINTS);
            for index in index..end {
                let local = (self.stored_count as usize + index) % PAGE_POINTS;
                let bytes = &bytes[index * 12..index * 12 + 12];
                page.visible[local / 64] |= 1 << (local % 64);
                page.length = local + 1;
                page.count += 1;
                let point = decode_point(bytes);
                for axis in 0..3 {
                    page.low[axis] = page.low[axis].min(point[axis]);
                    page.high[axis] = page.high[axis].max(point[axis]);
                }
                let source = points[index];
                if let Some(blocks) = &mut self.block_pages {
                    if blocks.entry(index_block([source.x, source.y, source.z])).or_default().insert(page_index) {
                        self.block_entries += 1;
                    }
                    // 达到预算后回退逐页检查，不丢点，也不无限扩大派生索引。
                    if self.block_entries > MAX_INDEX_ENTRIES { self.block_pages = None; }
                }
                if let Some(live) = &mut self.live_keys {
                    // 复用降精度前已验证并写入的键，不重复计算或从单精度坐标推导。
                    let offset = index * 12;
                    let key: [i32; 3] = std::array::from_fn(|axis| {
                        let offset = offset + axis * 4;
                        i32::from_le_bytes(keys[offset..offset + 4].try_into().unwrap())
                    });
                    live.insert((key[0], key[1], key[2]));
                    if live.len() > MAX_INDEX_ENTRIES { self.live_keys = None; }
                }
            }
            index = end;
        }
        self.point_count += points.len() as u64;
        self.stored_count = count;
        self.bounds_min = low;
        self.bounds_max = high;
        Ok(())
    }

    /// 仅检查删除区域关联的文件页。旧快照仍持有原可见性，读取/保存不会跨修订串帧。
    pub fn remove_voxels(&mut self, keys: &[Key], voxel_size: f32) -> Result<usize, String> {
        if keys.is_empty() {
            return Ok(0);
        }
        if !voxel_size.is_finite() || voxel_size <= 0.0 {
            return Err("完整点云删除精度非法".into());
        }
        if voxel_size != self.voxel_size {
            return Err("完整点云与精细地图索引精度不一致".into());
        }
        // 精细导航保留当前动态障碍，密集源可能从未加入它；无需为这些键重读历史。
        let removed: HashSet<_> = keys.iter().copied().filter(|key|
            self.live_keys.as_ref().is_none_or(|live| live.contains(key))).collect();
        if removed.is_empty() { return Ok(0); }
        let mut candidates = HashSet::new();
        for &(x, y, z) in &removed {
            let low = index_block([x, y, z].map(|v| v as f64 * f64::from(voxel_size)));
            let high = index_block([x, y, z].map(|v| (v as f64 + 1.0) * f64::from(voxel_size)));
            if self.block_pages.is_none() || voxel_size as f64 > INDEX_BLOCK_M {
                candidates.extend(0..self.pages.len());
                break;
            }
            for bx in low.0..=high.0 {
                for by in low.1..=high.1 {
                    for bz in low.2..=high.2 {
                        if let Some(pages) = self.block_pages.as_ref().unwrap().get(&(bx, by, bz)) {
                            candidates.extend(pages.iter().copied());
                        }
                    }
                }
            }
        }
        let mut changes = Vec::new();
        let mut count = 0;
        for index in candidates {
            let old = &self.pages[index];
            if old.count == 0 {
                continue;
            }
            let mut keys = vec![0; old.length * 12];
            self.keys_file
                .read_exact_at(&mut keys, (index * PAGE_POINTS * 12) as u64)
                .map_err(|e| e.to_string())?;
            let mut page = (**old).clone();
            for local in 0..old.length {
                if page.visible[local / 64] & (1 << (local % 64)) == 0 { continue; }
                let key: [i32; 3] = std::array::from_fn(|axis| {
                    let offset = local * 12 + axis * 4;
                    i32::from_le_bytes(keys[offset..offset + 4].try_into().unwrap())
                });
                if removed.contains(&(key[0], key[1], key[2])) {
                    page.visible[local / 64] &= !(1 << (local % 64));
                    page.count -= 1;
                }
            }
            if page.count == old.count { continue; }
            // 仅实际修改的页需要读取坐标并重算边界。
            page.low = [f32::INFINITY; 3];
            page.high = [f32::NEG_INFINITY; 3];
            if page.count > 0 {
                let mut bytes = vec![0; old.length * 12];
                self.file.read_exact_at(&mut bytes, (index * PAGE_POINTS * 12) as u64)
                    .map_err(|e| e.to_string())?;
                for (local, bytes) in bytes.chunks_exact(12).enumerate() {
                    if page.visible[local / 64] & (1 << (local % 64)) == 0 { continue; }
                    let point = decode_point(bytes);
                    for axis in 0..3 {
                        page.low[axis] = page.low[axis].min(point[axis]);
                        page.high[axis] = page.high[axis].max(point[axis]);
                    }
                }
            }
            count += (old.count - page.count) as usize;
            changes.push((index, Arc::new(page)));
        }
        if let Some(live) = &mut self.live_keys {
            for key in &removed { live.remove(key); }
        }
        if count > 0 {
            let pages = Arc::make_mut(&mut self.pages);
            for (index, page) in changes {
                pages[index] = page;
            }
            self.point_count -= count as u64;
            self.bounds_min = [f32::INFINITY; 3];
            self.bounds_max = [f32::NEG_INFINITY; 3];
            for page in pages.iter() {
                for axis in 0..3 {
                    self.bounds_min[axis] = self.bounds_min[axis].min(page.low[axis]);
                    self.bounds_max[axis] = self.bounds_max[axis].max(page.high[axis]);
                }
            }
            // 删除打破了追加前缀，客户端必须按新内容代际重取，不能只比较点数。
            self.epoch = uuid::Uuid::new_v4().simple().to_string();
        }
        Ok(count)
    }

    pub fn snapshot(
        &self,
        receipt: &CloudCommitReceipt,
        session_id: &str,
    ) -> Option<DenseSnapshot> {
        (self.point_count > 0).then(|| DenseSnapshot {
            file: Arc::clone(&self.file),
            pages: Arc::clone(&self.pages),
            page_ends: Arc::new(
                self.pages
                    .iter()
                    .scan(0_u64, |sum, page| {
                        *sum += page.count;
                        Some(*sum)
                    })
                    .collect(),
            ),
            epoch: self.epoch.clone(),
            point_count: self.point_count,
            bounds_min: self.bounds_min,
            bounds_max: self.bounds_max,
            receipt: receipt.clone(),
            session_id: session_id.to_owned(),
        })
    }

    pub(crate) fn readers(&self) -> DenseCloudReaders { self.readers.clone() }

    #[cfg(test)]
    pub fn pin_read(&mut self, latest: DenseSnapshot, epoch: &str, offset: u64) -> Result<DenseSnapshot, String> {
        self.readers.pin_read(latest, epoch, offset)
    }

    #[cfg(test)]
    pub fn pin_update(&mut self, latest: DenseSnapshot, epoch: &str, offset: u64) -> Result<(DenseSnapshot, Option<DenseSnapshot>), String> {
        self.readers.pin_update(latest, epoch, offset)
    }

    #[cfg(test)]
    pub fn read_epoch_available(&self, epoch: &str) -> bool {
        self.readers.read_epoch_available(&self.epoch, epoch)
    }

    #[cfg(test)]
    pub fn from_archive(file: File, expected: &Metadata) -> Result<Self, String> {
        Self::from_archive_with_voxel_size(file, expected, crate::pointcloud::VOXEL_SIZE)
    }

    pub fn from_archive_with_voxel_size(mut file: File, expected: &Metadata, voxel_size: f32) -> Result<Self, String> {
        let (metadata, offset) = read_metadata(&mut file)?;
        if metadata.session_id != expected.session_id
            || metadata.session_generation != expected.session_generation
            || metadata.cloud_revision != expected.cloud_revision
            || metadata.source_timestamp_ns != expected.source_timestamp_ns
            || metadata.frame_id != expected.frame_id
        {
            return Err("完整点云资产与原地图提交不一致".into());
        }
        let mut cloud = Self::with_voxel_size(voxel_size)?;
        file.seek(SeekFrom::Start(offset))
            .map_err(|e| e.to_string())?;
        let mut remaining = metadata.point_count;
        while remaining > 0 {
            let count = remaining.min(CHUNK_POINTS as u64) as usize;
            let mut data = vec![0; count * 12];
            file.read_exact(&mut data).map_err(|e| e.to_string())?;
            let points = data
                .chunks_exact(12)
                .map(|p| {
                    Point3::new(
                        f32::from_le_bytes(p[0..4].try_into().unwrap()) as f64,
                        f32::from_le_bytes(p[4..8].try_into().unwrap()) as f64,
                        f32::from_le_bytes(p[8..12].try_into().unwrap()) as f64,
                    )
                })
                .collect::<Vec<_>>();
            cloud.append(&points)?;
            remaining -= count as u64;
        }
        if cloud.bounds_min != metadata.bounds_min || cloud.bounds_max != metadata.bounds_max {
            return Err("完整点云资产边界与数据不符".into());
        }
        Ok(cloud)
    }
}

impl DenseSnapshot {
    /// 只返回已安装旧前缀中消失的区间，保留全部幸存点的顺序和坐标。
    pub fn removed_prefix_ranges(&self, old: &Self, offset: u64) -> Result<Vec<u64>, String> {
        if !Arc::ptr_eq(&self.file, &old.file) || self.session_id != old.session_id
            || self.receipt.session_generation != old.receipt.session_generation || offset > old.point_count {
            return Err("DENSE_CLOUD_SOURCE_CHANGED".into());
        }
        let mut ranges: Vec<u64> = Vec::new();
        let mut logical = 0_u64;
        for (index, page) in old.pages.iter().enumerate() {
            if logical >= offset { break; }
            let current = self.pages.get(index).ok_or("DENSE_CLOUD_SOURCE_CHANGED")?;
            if Arc::ptr_eq(page, current) {
                logical += page.count.min(offset - logical);
                continue;
            }
            for (word_index, &bits) in page.visible.iter().enumerate() {
                let mut bits = bits;
                while bits != 0 && logical < offset {
                    let bit = bits.trailing_zeros();
                    if current.visible[word_index] & (1_u64 << bit) == 0 {
                        let n = ranges.len();
                        if n >= 2 && ranges[n - 2] + ranges[n - 1] == logical {
                            ranges[n - 1] += 1;
                        } else {
                            // 差异过大时显式重新领取，避免无界响应。
                            if n >= 8192 { return Err("DENSE_CLOUD_SOURCE_CHANGED".into()); }
                            ranges.extend_from_slice(&[logical, 1]);
                        }
                    }
                    logical += 1;
                    bits &= bits - 1;
                }
            }
        }
        if logical != offset { return Err("DENSE_CLOUD_SOURCE_CHANGED".into()); }
        Ok(ranges)
    }

    pub fn read(&self, offset: u64, count: u32) -> Result<Vec<u8>, String> {
        if count == 0 || count > CHUNK_POINTS || offset > self.point_count {
            return Err("完整点云分块位置或大小非法".into());
        }
        let count = (self.point_count - offset).min(count as u64) as usize;
        let mut output = Vec::with_capacity(count * 12);
        if count == 0 {
            return Ok(output);
        }
        let start = self.page_ends.partition_point(|end| *end <= offset);
        let mut skip = offset
            - if start == 0 {
                0
            } else {
                self.page_ends[start - 1]
            };
        for index in start..self.pages.len() {
            let page = &self.pages[index];
            if page.count == 0 {
                continue;
            }
            let mut bytes = vec![0; page.length * 12];
            self.file
                .read_exact_at(&mut bytes, (index * PAGE_POINTS * 12) as u64)
                .map_err(|e| e.to_string())?;
            for (local, point) in bytes.chunks_exact(12).enumerate() {
                if page.visible[local / 64] & (1 << (local % 64)) == 0 {
                    continue;
                }
                if skip > 0 {
                    skip -= 1;
                    continue;
                }
                output.extend_from_slice(point);
                if output.len() == count * 12 {
                    return Ok(output);
                }
            }
        }
        Err("完整点云可见性索引与点数不符".into())
    }

    pub fn metadata(&self, frame_id: &str) -> Metadata {
        Metadata {
            version: 1,
            session_id: self.session_id.clone(),
            session_generation: self.receipt.session_generation,
            cloud_revision: self.receipt.cloud_revision,
            source_timestamp_ns: self.receipt.source_timestamp_ns,
            frame_id: frame_id.to_owned(),
            point_count: self.point_count,
            bounds_min: self.bounds_min,
            bounds_max: self.bounds_max,
        }
    }

    pub fn archive(&self, frame_id: &str) -> Result<(File, u64), String> {
        let header = serde_json::to_vec(&self.metadata(frame_id)).map_err(|e| e.to_string())?;
        if header.len() > MAX_HEADER_BYTES {
            return Err("完整点云资产元数据过大".into());
        }
        let mut file = temporary_file()?;
        file.write_all(MAGIC)
            .and_then(|_| file.write_all(&(header.len() as u32).to_le_bytes()))
            .and_then(|_| file.write_all(&header))
            .map_err(|e| e.to_string())?;
        let mut offset = 0;
        while offset < self.point_count {
            let data = self.read(offset, CHUNK_POINTS)?;
            offset += data.len() as u64 / 12;
            file.write_all(&data).map_err(|e| e.to_string())?;
        }
        let size = file.stream_position().map_err(|e| e.to_string())?;
        let expected = (12 + header.len() as u64).checked_add(self.point_count.checked_mul(12)
            .ok_or("完整点云归档大小溢出")?).ok_or("完整点云归档大小溢出")?;
        if size != expected || size > MAX_ARCHIVE_BYTES || file.metadata().map_err(|e| e.to_string())?.len() != size {
            return Err("完整点云归档大小不符或超过预算".into());
        }
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        Ok((file, size))
    }
}

#[cfg(test)]
std::thread_local! { pub(crate) static FORBID_ARCHIVE_HASH: std::cell::Cell<bool> = const { std::cell::Cell::new(false) }; }

/// Explicit legacy-only adapter; new archive creation returns owned bytes and size.
pub(crate) fn hash_reader(reader: &mut impl Read) -> Result<(u64, [u8; 32]), String> {
    #[cfg(test)] FORBID_ARCHIVE_HASH.with(|v| assert!(!v.get(), "legacy archive hash reached v2 path"));
    let mut hash = Sha256::new();
    let mut size = 0_u64;
    let mut buffer = vec![0; 1024 * 1024];
    loop {
        let count = reader.read(&mut buffer).map_err(|e| e.to_string())?;
        if count == 0 {
            break;
        }
        size += count as u64;
        if size > MAX_ARCHIVE_BYTES {
            return Err("点云制品超过存储容量".into());
        }
        hash.update(&buffer[..count]);
    }
    Ok((size, hash.finalize().into()))
}

pub(crate) fn read_metadata(file: &mut File) -> Result<(Metadata, u64), String> {
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    let size = file.metadata().map_err(|e| e.to_string())?.len();
    if !(24..=MAX_ARCHIVE_BYTES).contains(&size) {
        return Err("完整点云资产大小非法".into());
    }
    let mut prefix = [0; 12];
    file.read_exact(&mut prefix).map_err(|e| e.to_string())?;
    if &prefix[..8] != MAGIC {
        return Err("完整点云资产格式未知".into());
    }
    let length = u32::from_le_bytes(prefix[8..12].try_into().unwrap()) as usize;
    if length == 0 || length > MAX_HEADER_BYTES {
        return Err("完整点云资产头长度非法".into());
    }
    let mut header = vec![0; length];
    file.read_exact(&mut header).map_err(|e| e.to_string())?;
    let metadata: Metadata = serde_json::from_slice(&header).map_err(|e| e.to_string())?;
    let offset = 12 + length as u64;
    if offset > size { return Err("完整点云资产头截断".into()); }
    if metadata.version != 1
        || metadata.point_count == 0
        || metadata.point_count > (size - offset) / 12
        || size - offset != metadata.point_count * 12
        || metadata.session_generation == 0
        || metadata.cloud_revision == 0
        || metadata.source_timestamp_ns == 0
        || metadata.session_id.trim().is_empty()
        || metadata.session_id.len() > 1024
        || metadata.frame_id.trim().is_empty()
        || metadata.frame_id.len() > 1024
        || (0..3).any(|i| {
            !metadata.bounds_min[i].is_finite()
                || !metadata.bounds_max[i].is_finite()
                || metadata.bounds_min[i] > metadata.bounds_max[i]
        })
    {
        return Err("完整点云资产身份、点数或边界非法".into());
    }
    Ok((metadata, offset))
}

pub(crate) fn validate_archive(file: &mut File) -> Result<(), String> {
    let (metadata, offset) = read_metadata(file)?;
    let mut remaining = metadata.point_count;
    let (mut low, mut high) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    file.seek(SeekFrom::Start(offset))
        .map_err(|e| e.to_string())?;
    while remaining > 0 {
        let count = remaining.min(CHUNK_POINTS as u64) as usize;
        let mut bytes = vec![0; count * 12];
        file.read_exact(&mut bytes).map_err(|e| e.to_string())?;
        for (index, bytes) in bytes.chunks_exact(4).enumerate() {
            let value = f32::from_le_bytes(bytes.try_into().unwrap());
            if !value.is_finite() {
                return Err("完整点云资产包含无效坐标".into());
            }
            low[index % 3] = low[index % 3].min(value);
            high[index % 3] = high[index % 3].max(value);
        }
        remaining -= count as u64;
    }
    if low != metadata.bounds_min || high != metadata.bounds_max {
        return Err("完整点云资产边界与数据不符".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping_commit::MappingCommitId;

    fn receipt(revision: u64) -> CloudCommitReceipt {
        CloudCommitReceipt {
            commit_id: MappingCommitId {
                boot_nonce: 7,
                sequence: revision,
            },
            session_generation: 2,
            occupancy_revision: revision,
            cloud_revision: revision,
            source_timestamp_ns: revision * 100,
            point_count: 1,
        }
    }

    #[test]
    #[ignore = "release-mode dense append timing; run alone with --nocapture"]
    fn benchmark_dense_append_static_mix_and_retained_snapshots() {
        for n in [1_000, 30_000, 100_000] {
            let points = (0..n).map(|i| Point3::new(
                (i % 317) as f64 * 0.051 + 0.025,
                ((i / 317) % 317) as f64 * 0.051 + 0.025,
                (i % 7) as f64 * 0.1 + 0.025,
            )).collect::<Vec<_>>();
            for dynamic_mix in [false, true] {
                for live_index in [false, true] {
                    for retained in [false, true] {
                        let mut timings = Vec::new();
                        for repetition in 0..17 {
                            let mut cloud = DenseCloud::new().unwrap();
                            if !live_index { cloud.live_keys = None; }
                            let mut snapshots = Vec::new();
                            if retained {
                                for revision in 0..4 {
                                    cloud.append(&points[..n.min(997)]).unwrap();
                                    snapshots.push(cloud.snapshot(&receipt(revision), "追加基准").unwrap());
                                }
                            }
                            let started = std::time::Instant::now();
                            let static_points = points.iter().enumerate()
                                .filter(|(i, _)| !dynamic_mix || i % 4 != 0)
                                .map(|(_, point)| *point).collect::<Vec<_>>();
                            cloud.append(std::hint::black_box(&static_points)).unwrap();
                            let elapsed = started.elapsed().as_nanos();
                            if repetition >= 2 { timings.push(elapsed); }
                            std::hint::black_box((&cloud, &snapshots));
                        }
                        timings.sort_unstable();
                        println!("dense_append n={n} dynamic_mix={dynamic_mix} live_index={live_index} retained={retained} median_ns={}", timings[timings.len() / 2]);
                    }
                }
            }
        }
    }

    #[test]
    fn append_spans_partial_pages_preserving_keys_bounds_and_snapshots() {
        for live_index in [false, true] {
            let mut cloud = DenseCloud::new().unwrap();
            if !live_index { cloud.live_keys = None; }
            let boundary = f64::from(0.05_f32) * 40.0;
            let points = (0..PAGE_POINTS * 2 + 17).map(|i| {
                let x = if i % 2 == 0 { boundary - 1e-10 } else { boundary + 1e-10 };
                Point3::new(x, if i % 3 == 0 { -0.025 } else { 0.025 }, (i % 11) as f64 * 0.05)
            }).collect::<Vec<_>>();
            let mut expected = Vec::new();
            let mut frozen = Vec::new();
            let mut start = 0;
            for end in [1, PAGE_POINTS - 1, PAGE_POINTS, PAGE_POINTS + 1, points.len()] {
                cloud.append(&points[start..end]).unwrap();
                expected.extend(points[start..end].iter().flat_map(|p| [p.x, p.y, p.z].into_iter().flat_map(|v| (v as f32).to_le_bytes())));
                let snapshot = cloud.snapshot(&receipt(end as u64), "分页追加").unwrap();
                assert_eq!(snapshot.read(0, CHUNK_POINTS).unwrap(), expected);
                for (old, old_bytes) in &frozen {
                    let old: &DenseSnapshot = old;
                    assert_eq!(old.read(0, CHUNK_POINTS).unwrap(), *old_bytes);
                }
                let (mut archive, _) = snapshot.archive("map").unwrap();
                validate_archive(&mut archive).unwrap();
                let restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
                assert_eq!(restored.snapshot(&receipt(99), "恢复").unwrap().read(0, CHUNK_POINTS).unwrap(), expected);
                frozen.push((snapshot, expected.clone()));
                start = end;
            }
            let mut stored_keys = vec![0; points.len() * 12];
            cloud.keys_file.read_exact_at(&mut stored_keys, 0).unwrap();
            let expected_keys: Vec<_> = points.iter().flat_map(|p| {
                let (x, y, z) = crate::pointcloud::voxel_key([p.x, p.y, p.z], 0.05).unwrap();
                [x, y, z].into_iter().flat_map(i32::to_le_bytes)
            }).collect();
            assert_eq!(stored_keys, expected_keys);
            let remove: Vec<_> = points.iter().step_by(2).map(|p|
                crate::pointcloud::voxel_key([p.x, p.y, p.z], 0.05).unwrap()).collect();
            assert_eq!(cloud.remove_voxels(&remove, 0.05).unwrap(), (points.len() + 1) / 2);
            let survivors: Vec<_> = points.iter().skip(1).step_by(2).flat_map(|p|
                [p.x, p.y, p.z].into_iter().flat_map(|v| (v as f32).to_le_bytes())).collect();
            assert_eq!(cloud.snapshot(&receipt(100), "删除").unwrap().read(0, CHUNK_POINTS).unwrap(), survivors);
            assert_eq!(frozen.last().unwrap().0.read(0, CHUNK_POINTS).unwrap(), expected);
        }
    }

    #[test]
    fn append_failure_keeps_published_state_and_retry_overwrites_unpublished_tail() {
        let mut cloud = DenseCloud::new().unwrap();
        cloud.append(&[Point3::new(0.025, 0.025, 0.025)]).unwrap();
        let before = cloud.snapshot(&receipt(1), "失败重试").unwrap();
        let before_bytes = before.read(0, CHUNK_POINTS).unwrap();
        let before_pages = Arc::clone(&cloud.pages);
        let before_live = cloud.live_keys.clone();
        let before_entries = cloud.block_entries;
        let size = cloud.file.metadata().unwrap().len();
        assert!(cloud.append(&[Point3::new(1.0, 2.0, 3.0), Point3::new(f64::NAN, 0.0, 0.0)]).is_err());
        assert_eq!(cloud.file.metadata().unwrap().len(), size);
        // 坐标文件写入成功而键文件失败，未发布的尾部不进入任何快照。
        let writable_keys = std::mem::replace(&mut cloud.keys_file, File::open("/dev/null").unwrap());
        assert!(cloud.append(&[Point3::new(9.0, 8.0, 7.0)]).is_err());
        assert_eq!(cloud.file.metadata().unwrap().len(), size + 12);
        assert_eq!(cloud.point_count, 1);
        assert_eq!(cloud.stored_count, 1);
        assert!(Arc::ptr_eq(&cloud.pages, &before_pages));
        assert_eq!(cloud.live_keys, before_live);
        assert_eq!(cloud.block_entries, before_entries);
        assert_eq!(cloud.bounds_min, before.bounds_min);
        assert_eq!(cloud.bounds_max, before.bounds_max);
        assert_eq!(cloud.snapshot(&receipt(2), "失败重试").unwrap().read(0, CHUNK_POINTS).unwrap(), before_bytes);
        cloud.keys_file = writable_keys;
        let next = Point3::new(1.025, -2.025, 3.025);
        cloud.append(&[next]).unwrap();
        let expected: Vec<_> = before_bytes.iter().copied().chain([next.x, next.y, next.z].into_iter().flat_map(|v| (v as f32).to_le_bytes())).collect();
        let after = cloud.snapshot(&receipt(3), "失败重试").unwrap();
        assert_eq!(after.read(0, CHUNK_POINTS).unwrap(), expected);
        assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), before_bytes);
        let (mut archive, _) = after.archive("map").unwrap();
        validate_archive(&mut archive).unwrap();
    }

    #[test]
    fn dynamic_removal_uses_source_voxel_keys_before_float_rounding() {
        let boundary = f64::from(0.05_f32) * 40.0;
        let points = [
            Point3::new(boundary - 1e-10, 0.025, 0.025),
            Point3::new(boundary + 1e-10, 0.025, 0.025),
        ];
        assert_eq!(points[0].x as f32, points[1].x as f32);
        let mut cloud = DenseCloud::new().unwrap();
        cloud.append(&points).unwrap();
        assert_eq!(cloud.remove_voxels(&[(39, 0, 0)], 0.05).unwrap(), 1);
        assert_eq!(cloud.snapshot(&receipt(1), "边界").unwrap().point_count, 1);
        assert_eq!(cloud.remove_voxels(&[(40, 0, 0)], 0.05).unwrap(), 1);
        assert!(cloud.snapshot(&receipt(2), "空图").is_none());
    }

    #[test]
    fn dynamic_removal_preserves_frozen_reads_order_coordinates_and_archive() {
        let mut cloud = DenseCloud::new().unwrap();
        let points = (0..12_000)
            .map(|i| {
                if i % 3 == 0 {
                    Point3::new(2.025, 0.025, 1.025)
                } else {
                    Point3::new(-0.123, (i % 100) as f64 * 0.03, 0.007)
                }
            })
            .collect::<Vec<_>>();
        cloud.append(&points).unwrap();
        let before = cloud.snapshot(&receipt(1), "动态归档").unwrap();
        cloud.pin_read(before.clone(), "", 0).unwrap();
        assert_eq!(cloud.remove_voxels(&[(40, 0, 20)], 0.05).unwrap(), 4000);
        let after = cloud.snapshot(&receipt(2), "动态归档").unwrap();
        assert_eq!(after.point_count, 8000);
        assert_ne!(before.epoch, after.epoch);
        assert_eq!(before.read(0, 12_000).unwrap().len(), 12_000 * 12);
        let expected = points
            .iter()
            .enumerate()
            .filter(|(i, _)| i % 3 != 0)
            .flat_map(|(_, point)| {
                [point.x, point.y, point.z]
                    .into_iter()
                    .flat_map(|v| (v as f32).to_le_bytes())
            })
            .collect::<Vec<_>>();
        assert_eq!(after.read(0, 12_000).unwrap(), expected);
        assert_eq!(
            after.read(4093, 100).unwrap(),
            expected[4093 * 12..4193 * 12]
        );
        let pinned = cloud.pin_read(after.clone(), &before.epoch, 6553).unwrap();
        assert_eq!(pinned.epoch, before.epoch);
        assert!(cloud
            .pin_read(after.clone(), &before.epoch, before.point_count)
            .is_err());
        let (mut archive, _) = after.archive("map").unwrap();
        validate_archive(&mut archive).unwrap();
        let restored = DenseCloud::from_archive(archive, &after.metadata("map")).unwrap();
        assert_eq!(
            restored
                .snapshot(&receipt(3), "恢复")
                .unwrap()
                .read(0, 12_000)
                .unwrap(),
            expected
        );
        cloud.block_pages = None;
        cloud.live_keys = None;
        cloud.append(&[Point3::new(2.025, 0.025, 1.025)]).unwrap();
        let newest = cloud.snapshot(&receipt(4), "动态归档").unwrap();
        assert_eq!(newest.point_count, 8001);
        assert_eq!(newest.read(8000, 1).unwrap().len(), 12);
        assert_eq!(after.read(0, 12_000).unwrap(), expected);
        assert_eq!(cloud.remove_voxels(&[(40, 0, 20)], 0.05).unwrap(), 1);
        assert_eq!(cloud.snapshot(&receipt(5), "预算回退").unwrap().read(0, 12_000).unwrap(), expected);
    }

    #[test]
    fn delta_keeps_surviving_prefix_and_unread_points_through_multiple_removals() {
        let mut cloud = DenseCloud::new().unwrap();
        let points = (0..10_000).map(|i| Point3::new(i as f64 * 0.1 + 0.025, 0.025, 1.025)).collect::<Vec<_>>();
        cloud.append(&points).unwrap();
        let old = cloud.snapshot(&receipt(1), "增量").unwrap();
        cloud.pin_update(old.clone(), "", 0).unwrap();
        cloud.remove_voxels(&[(4, 0, 20), (8000, 0, 20), (18000, 0, 20)], 0.05).unwrap();
        cloud.append(&[Point3::new(9999.025, 0.025, 1.025)]).unwrap();
        let current = cloud.snapshot(&receipt(2), "增量").unwrap();
        let ranges = current.removed_prefix_ranges(&old, 5000).unwrap();
        assert_eq!(ranges, vec![2, 1, 4000, 1]);
        let old_bytes = old.read(0, 5000).unwrap();
        let mut installed = Vec::new();
        for (i, point) in old_bytes.chunks_exact(12).enumerate() {
            if i != 2 && i != 4000 { installed.extend_from_slice(point); }
        }
        installed.extend_from_slice(&current.read(4998, CHUNK_POINTS).unwrap());
        assert_eq!(installed, current.read(0, CHUNK_POINTS).unwrap());
        let (_, base) = cloud.pin_update(current.clone(), &old.epoch, 5000).unwrap();
        assert!(base.is_some());
        let replacement = DenseCloud::new().unwrap();
        assert!(!replacement.read_epoch_available(&old.epoch));
    }

    #[test]
    fn dense_cloud_preserves_over_a_million_sub_voxel_points_and_frozen_prefixes() {
        let mut cloud = DenseCloud::new().unwrap();
        let scan = vec![Point3::new(0.011, -0.023, 1.037); 100_001];
        for _ in 0..11 {
            cloud.append(&scan).unwrap();
        }
        let frozen = cloud.snapshot(&receipt(1), "密集验证").unwrap();
        assert_eq!(frozen.point_count, 1_100_011);
        cloud.append(&[Point3::new(0.012, -0.024, 1.038)]).unwrap();
        assert!(frozen.read(frozen.point_count, 1).unwrap().is_empty());
        let latest = cloud.snapshot(&receipt(2), "密集验证").unwrap();
        assert_eq!(latest.point_count, frozen.point_count + 1);
        let first = frozen.read(0, CHUNK_POINTS).unwrap();
        assert_eq!(
            f32::from_le_bytes(first[..4].try_into().unwrap()),
            0.011_f32
        );
        let (mut file, size) = frozen.archive("map").unwrap();
        assert!(size > 12_000_000);
        validate_archive(&mut file).unwrap();
        file.seek(SeekFrom::Start(0)).unwrap();
        assert_eq!(file.metadata().unwrap().len(), size);
        let restored = DenseCloud::from_archive(file, &frozen.metadata("map")).unwrap();
        let restored = restored.snapshot(&receipt(3), "恢复验证").unwrap();
        assert_ne!(restored.epoch, frozen.epoch);
        assert_eq!(restored.point_count, frozen.point_count);
        let mut offset = 0;
        while offset < frozen.point_count {
            let bytes = frozen.read(offset, CHUNK_POINTS).unwrap();
            assert_eq!(bytes, restored.read(offset, CHUNK_POINTS).unwrap());
            offset += bytes.len() as u64 / 12;
        }
        drop(cloud);
        assert_eq!(frozen.read(0, CHUNK_POINTS).unwrap(), first);
    }

    #[test]
    fn dense_cloud_rejects_wrong_identity_truncation_and_invalid_coordinates() {
        let mut cloud = DenseCloud::new().unwrap();
        cloud.append(&[Point3::new(0.011, 0.012, 0.013)]).unwrap();
        assert!(cloud.append(&[Point3::new(f64::NAN, 0., 0.)]).is_err());
        let snapshot = cloud.snapshot(&receipt(1), "源").unwrap();
        assert_eq!(snapshot.point_count, 1);
        assert!(snapshot.read(2, 1).is_err());
        assert!(snapshot.read(0, CHUNK_POINTS + 1).is_err());
        let (file, _) = snapshot.archive("map").unwrap();
        let mut wrong = snapshot.metadata("map");
        wrong.session_generation += 1;
        assert!(DenseCloud::from_archive(file, &wrong).is_err());
        let (mut file, size) = snapshot.archive("map").unwrap();
        file.set_len(size - 1).unwrap();
        assert!(validate_archive(&mut file).is_err());
        let (mut file, _) = snapshot.archive("map").unwrap();
        let (_, offset) = read_metadata(&mut file).unwrap();
        file.write_all_at(&f32::NAN.to_le_bytes(), offset).unwrap();
        assert!(validate_archive(&mut file).is_err());
    }
}

impl DenseCloudReaders {
    /// 删除期间允许读者完成已经领取的不可变版本；读到尾部后再切换最新版本。
    pub fn pin_read(
        &self,
        latest: DenseSnapshot,
        expected_epoch: &str,
        offset: u64,
    ) -> Result<DenseSnapshot, String> {
        let mut readers = self.0.lock();
        readers
            .retain(|(at, _)| at.elapsed() < std::time::Duration::from_secs(30));
        let snapshot = if expected_epoch.is_empty() || expected_epoch == latest.epoch {
            latest
        } else {
            readers
                .iter()
                .find(|(_, snapshot)| {
                    snapshot.epoch == expected_epoch && offset < snapshot.point_count
                })
                .map(|(_, snapshot)| snapshot.clone())
                .ok_or("DENSE_CLOUD_SOURCE_CHANGED")?
        };
        readers.retain(|(_, old)| old.epoch != snapshot.epoch);
        readers
            .push_back((std::time::Instant::now(), snapshot.clone()));
        while readers.len() > 8 {
            readers.pop_front();
        }
        Ok(snapshot)
    }

    /// 捕获删除前缀的基准和最新快照；差异计算在拥有方锁外执行。
    pub fn pin_update(
        &self, latest: DenseSnapshot, epoch: &str, offset: u64,
    ) -> Result<(DenseSnapshot, Option<DenseSnapshot>), String> {
        let mut readers = self.0.lock();
        readers.retain(|(at, _)| at.elapsed() < std::time::Duration::from_secs(30));
        let base = if !epoch.is_empty() && epoch != latest.epoch {
            Some(readers.iter().find(|(_, old)| old.epoch == epoch && offset <= old.point_count)
                .map(|(_, old)| old.clone()).ok_or("DENSE_CLOUD_SOURCE_CHANGED")?)
        } else { None };
        readers.retain(|(_, old)| old.epoch != latest.epoch);
        readers.push_back((std::time::Instant::now(), latest.clone()));
        while readers.len() > 8 { readers.pop_front(); }
        Ok((latest, base))
    }

    pub fn read_epoch_available(&self, latest_epoch: &str, epoch: &str) -> bool {
        latest_epoch == epoch
            || self.0.lock().iter().any(|(at, snapshot)| {
                at.elapsed() < std::time::Duration::from_secs(30) && snapshot.epoch == epoch
            })
    }

}
