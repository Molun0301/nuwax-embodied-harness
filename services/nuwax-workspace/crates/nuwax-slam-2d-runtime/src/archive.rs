//! 有界流式二维图档案；图结构、扫描和对应物理栅格共同组成一个不可分割资产。
use crate::protocol::{Estimate, FileArtifact, Session};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::os::unix::fs::OpenOptionsExt;
use std::{
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    path::Path,
};

pub const MAGIC: &[u8; 8] = b"NWS2D001";
pub const MAX_HEADER: usize = 64 * 1024;
pub const MAX_MEMBER: u64 = 512 * 1024 * 1024;
pub const MAX_GRID: u64 = 16 * 1024 * 1024;
pub const MAX_ARCHIVE: u64 = 2 * MAX_MEMBER + MAX_GRID + MAX_HEADER as u64 + 12;
pub const CHUNK_BYTES: usize = 1024 * 1024;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Member {
    pub name: String,
    pub size: u64,
    pub sha256: [u8; 32],
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Metadata {
    pub version: u32,
    pub session: Session,
    pub estimate: Estimate,
    pub source_unix_timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin: [f64; 2],
    pub members: [Member; 3],
}
impl Metadata {
    pub fn validate(&self) -> Result<(), String> {
        self.session.validate()?;
        if self.version != 1
            || self.estimate.graph_revision == 0
            || self.source_unix_timestamp_ns == 0
            || self.width == 0
            || self.height == 0
            || !self.resolution.is_finite()
            || self.resolution <= 0.0
            || self.origin.iter().any(|v| !v.is_finite())
        {
            return Err("二维图档案版本、来源或栅格几何非法".into());
        }
        for (index, name) in ["graph.posegraph", "graph.data", "grid.i8"]
            .iter()
            .enumerate()
        {
            let member = &self.members[index];
            let limit = if index == 2 { MAX_GRID } else { MAX_MEMBER };
            if member.name != *name
                || member.size == 0
                || member.size > limit
                || member.sha256 == [0; 32]
            {
                return Err("二维图档案成员身份或预算非法".into());
            }
        }
        if self.members[2].size != u64::from(self.width) * u64::from(self.height) {
            return Err("二维物理栅格尺寸与档案成员不一致".into());
        }
        Ok(())
    }
    pub fn compatible_with(&self, session: &Session) -> Result<(), String> {
        session.validate()?;
        let previous = &self.session;
        if previous.sensor_id != session.sensor_id
            || previous.calibration_ref != session.calibration_ref
            || previous.frame_id != session.frame_id
            || previous.body_frame_id != session.body_frame_id
            || previous.map_frame_id != session.map_frame_id
            || previous.odom_frame_id != session.odom_frame_id
        {
            return Err("二维图传感器、坐标或标定身份不兼容".into());
        }
        // 完整算法参数由实际核心解析与加载核对；时钟启动身份允许重新建立。
        Ok(())
    }
}
pub fn open_regular(path: &Path) -> Result<File, String> {
    let file = OpenOptions::new()
        .read(true)
        .custom_flags(nix::libc::O_NOFOLLOW)
        .open(path)
        .map_err(|e| e.to_string())?;
    if !file.metadata().map_err(|e| e.to_string())?.is_file() {
        return Err("二维档案不是普通文件".into());
    }
    Ok(file)
}
pub fn hash_reader(reader: &mut impl Read) -> Result<(u64, [u8; 32]), String> {
    let mut hash = Sha256::new();
    let mut size = 0u64;
    let mut buffer = vec![0u8; CHUNK_BYTES];
    loop {
        let n = reader.read(&mut buffer).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        size = size.checked_add(n as u64).ok_or("二维档案大小溢出")?;
        if size > MAX_ARCHIVE {
            return Err("二维档案超过总预算".into());
        }
        hash.update(&buffer[..n]);
    }
    Ok((size, hash.finalize().into()))
}
pub fn describe(path: &Path) -> Result<FileArtifact, String> {
    let mut file = open_regular(path)?;
    let (payload_size, sha256) = hash_reader(&mut file)?;
    Ok(FileArtifact {
        file_name: path
            .file_name()
            .and_then(|n| n.to_str())
            .ok_or("档案文件名非法")?
            .into(),
        payload_size,
        sha256,
    })
}
pub fn read_header(file: &mut File) -> Result<(Metadata, u64), String> {
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    let mut prefix = [0; 12];
    file.read_exact(&mut prefix).map_err(|e| e.to_string())?;
    if &prefix[..8] != MAGIC {
        return Err("不是受支持的二维完整图档案".into());
    }
    let size = u32::from_le_bytes(prefix[8..].try_into().unwrap()) as usize;
    if size == 0 || size > MAX_HEADER {
        return Err("二维档案元数据超过预算".into());
    }
    let mut bytes = vec![0; size];
    file.read_exact(&mut bytes).map_err(|e| e.to_string())?;
    let metadata: Metadata = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    metadata.validate()?;
    let expected = 12 + size as u64 + metadata.members.iter().map(|m| m.size).sum::<u64>();
    if expected > MAX_ARCHIVE || file.metadata().map_err(|e| e.to_string())?.len() != expected {
        return Err("二维档案长度与成员清单不一致".into());
    }
    Ok((metadata, 12 + size as u64))
}
pub fn verify(file: &mut File) -> Result<Metadata, String> {
    let (metadata, _) = read_header(file)?;
    for member in &metadata.members {
        let (size, sha) = hash_reader(&mut (&mut *file).take(member.size))?;
        if size != member.size || sha != member.sha256 {
            return Err(format!("二维档案成员损坏：{}", member.name));
        }
    }
    Ok(metadata)
}
pub fn grid(file: &mut File) -> Result<(Metadata, Vec<i8>), String> {
    let (metadata, start) = read_header(file)?;
    file.seek(SeekFrom::Start(
        start + metadata.members[0].size + metadata.members[1].size,
    ))
    .map_err(|e| e.to_string())?;
    let mut data = vec![0; metadata.members[2].size as usize];
    file.read_exact(&mut data).map_err(|e| e.to_string())?;
    if nuwax_codec::hash::sha256_bytes(&data) != metadata.members[2].sha256
        || data.iter().any(|v| !matches!(*v, 0 | 100 | 255))
    {
        return Err("二维档案物理栅格损坏或类别非法".into());
    }
    Ok((metadata, data.into_iter().map(|v| v as i8).collect()))
}
pub fn write(
    path: &Path,
    metadata: &Metadata,
    member_paths: [&Path; 3],
) -> Result<FileArtifact, String> {
    metadata.validate()?;
    let header = serde_json::to_vec(metadata).map_err(|e| e.to_string())?;
    if header.len() > MAX_HEADER {
        return Err("二维图元数据超过预算".into());
    }
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)
        .map_err(|e| e.to_string())?;
    let result = (|| {
        output
            .write_all(MAGIC)
            .and_then(|_| output.write_all(&(header.len() as u32).to_le_bytes()))
            .and_then(|_| output.write_all(&header))
            .map_err(|e| e.to_string())?;
        let mut buffer = vec![0; CHUNK_BYTES];
        for (member, path) in metadata.members.iter().zip(member_paths) {
            let mut input = open_regular(path)?;
            let mut hash = Sha256::new();
            let mut size = 0;
            loop {
                let n = input.read(&mut buffer).map_err(|e| e.to_string())?;
                if n == 0 {
                    break;
                }
                size += n as u64;
                if size > member.size {
                    return Err("二维档案写入期间成员增长".into());
                }
                hash.update(&buffer[..n]);
                output.write_all(&buffer[..n]).map_err(|e| e.to_string())?;
            }
            if size != member.size || <[u8; 32]>::from(hash.finalize()) != member.sha256 {
                return Err("二维档案写入期间成员变化".into());
            }
        }
        output.sync_all().map_err(|e| e.to_string())?;
        describe(path)
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(path);
    }
    result
}
pub fn unpack(file: &mut File, directory: &Path) -> Result<Metadata, String> {
    let (metadata, _) = read_header(file)?;
    let mut created = Vec::new();
    let result = (|| {
        let mut buffer = vec![0; CHUNK_BYTES];
        for member in &metadata.members {
            let path = directory.join(&member.name);
            let mut output = OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(0o600)
                .open(&path)
                .map_err(|e| e.to_string())?;
            created.push(path);
            let mut left = member.size;
            let mut hash = Sha256::new();
            while left > 0 {
                let n = buffer.len().min(left as usize);
                file.read_exact(&mut buffer[..n])
                    .map_err(|e| e.to_string())?;
                hash.update(&buffer[..n]);
                output.write_all(&buffer[..n]).map_err(|e| e.to_string())?;
                left -= n as u64;
            }
            if <[u8; 32]>::from(hash.finalize()) != member.sha256 {
                return Err("二维图档案成员摘要不一致".into());
            }
            output.sync_all().map_err(|e| e.to_string())?;
        }
        Ok(metadata)
    })();
    if result.is_err() {
        for path in created {
            let _ = std::fs::remove_file(path);
        }
    }
    result
}
