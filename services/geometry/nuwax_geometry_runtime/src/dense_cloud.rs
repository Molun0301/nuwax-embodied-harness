//! 已接纳扫描的原坐标归档；追加文件只公开规范提交时冻结的前缀。
use crate::mapping_commit::CloudCommitReceipt;
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

pub(crate) struct DenseCloud {
    file: Arc<File>,
    epoch: String,
    point_count: u64,
    bounds_min: [f32; 3],
    bounds_max: [f32; 3],
}

#[derive(Clone)]
pub(crate) struct DenseSnapshot {
    file: Arc<File>,
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
    pub fn new() -> Result<Self, String> {
        Ok(Self {
            file: Arc::new(temporary_file()?),
            epoch: uuid::Uuid::new_v4().simple().to_string(),
            point_count: 0,
            bounds_min: [f32::INFINITY; 3],
            bounds_max: [f32::NEG_INFINITY; 3],
        })
    }

    pub fn append(&mut self, points: &[Point3<f64>]) -> Result<(), String> {
        let count = self
            .point_count
            .checked_add(points.len() as u64)
            .ok_or("完整点云计数溢出")?;
        if count > (MAX_ARCHIVE_BYTES - MAX_HEADER_BYTES as u64 - 12) / 12 {
            return Err(
                "完整点云存储达到8GiB容量，无法继续保留全部扫描；请保存后开始新地图".into(),
            );
        }
        let mut bytes = Vec::with_capacity(points.len() * 12);
        let (mut low, mut high) = (self.bounds_min, self.bounds_max);
        for point in points {
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
            .write_all_at(&bytes, self.point_count * 12)
            .map_err(|e| e.to_string())?;
        self.point_count = count;
        self.bounds_min = low;
        self.bounds_max = high;
        Ok(())
    }

    pub fn snapshot(
        &self,
        receipt: &CloudCommitReceipt,
        session_id: &str,
    ) -> Option<DenseSnapshot> {
        (self.point_count > 0).then(|| DenseSnapshot {
            file: Arc::clone(&self.file),
            epoch: self.epoch.clone(),
            point_count: self.point_count,
            bounds_min: self.bounds_min,
            bounds_max: self.bounds_max,
            receipt: receipt.clone(),
            session_id: session_id.to_owned(),
        })
    }

    pub fn from_archive(mut file: File, expected: &Metadata) -> Result<Self, String> {
        let (metadata, offset) = read_metadata(&mut file)?;
        if metadata.session_id != expected.session_id
            || metadata.session_generation != expected.session_generation
            || metadata.cloud_revision != expected.cloud_revision
            || metadata.source_timestamp_ns != expected.source_timestamp_ns
            || metadata.frame_id != expected.frame_id
        {
            return Err("完整点云资产与原地图提交不一致".into());
        }
        let mut cloud = Self::new()?;
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
    pub fn read(&self, offset: u64, count: u32) -> Result<Vec<u8>, String> {
        if count == 0 || count > CHUNK_POINTS || offset > self.point_count {
            return Err("完整点云分块位置或大小非法".into());
        }
        let count = (self.point_count - offset).min(count as u64) as usize;
        let mut bytes = vec![0; count * 12];
        self.file
            .read_exact_at(&mut bytes, offset * 12)
            .map_err(|e| e.to_string())?;
        Ok(bytes)
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

    pub fn archive(&self, frame_id: &str) -> Result<(File, u64, [u8; 32]), String> {
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
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        let (size, sha) = hash_reader(&mut file)?;
        Ok((file, size, sha))
    }
}

pub(crate) fn hash_reader(reader: &mut impl Read) -> Result<(u64, [u8; 32]), String> {
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
        let (mut file, size, sha) = frozen.archive("map").unwrap();
        assert!(size > 12_000_000);
        validate_archive(&mut file).unwrap();
        file.seek(SeekFrom::Start(0)).unwrap();
        assert_eq!(hash_reader(&mut file).unwrap(), (size, sha));
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
        let (file, _, _) = snapshot.archive("map").unwrap();
        let mut wrong = snapshot.metadata("map");
        wrong.session_generation += 1;
        assert!(DenseCloud::from_archive(file, &wrong).is_err());
        let (mut file, size, _) = snapshot.archive("map").unwrap();
        file.set_len(size - 1).unwrap();
        assert!(validate_archive(&mut file).is_err());
        let (mut file, _, _) = snapshot.archive("map").unwrap();
        let (_, offset) = read_metadata(&mut file).unwrap();
        file.write_all_at(&f32::NAN.to_le_bytes(), offset).unwrap();
        assert!(validate_archive(&mut file).is_err());
    }
}
