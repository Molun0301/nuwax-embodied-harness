//! 有界流式二维图档案；图结构、扫描和对应物理栅格共同组成一个不可分割资产。
use crate::protocol::{ArtifactIdentity, Estimate, Session};
use serde::{Deserialize, Serialize};
use std::os::unix::fs::OpenOptionsExt;
use std::{
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    path::Path,
};

pub const MAGIC: &[u8; 8] = b"NWS2D001";
pub const ASSIGNED_MAGIC: &[u8; 8] = b"NWS2D002";
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
pub struct Metadata<M = Member> {
    pub version: u32,
    pub session: Session,
    pub estimate: Estimate,
    pub source_unix_timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin: [f64; 2],
    pub members: [M; 3],
}
impl<M: MemberLayout> Metadata<M> {
    pub fn validate(&self) -> Result<(), String> {
        self.session.validate()?;
        if self.version != M::VERSION
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
            if member.name() != *name
                || member.size() == 0
                || member.size() > limit
                || !member.identity_valid()
            {
                return Err("二维图档案成员身份或预算非法".into());
            }
        }
        if self.members[2].size() != u64::from(self.width) * u64::from(self.height) {
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
/// Fixed member names are roles; assigned members belong to their archive checkpoint.
pub trait MemberLayout {
    const VERSION: u32;
    fn name(&self) -> &str;
    fn size(&self) -> u64;
    fn identity_valid(&self) -> bool;
}
impl MemberLayout for Member {
    const VERSION: u32 = 1;
    fn name(&self) -> &str { &self.name }
    fn size(&self) -> u64 { self.size }
    fn identity_valid(&self) -> bool { self.sha256 != [0; 32] }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AssignedMember {
    pub name: String,
    pub size: u64,
}
impl MemberLayout for AssignedMember {
    const VERSION: u32 = 2;
    fn name(&self) -> &str { &self.name }
    fn size(&self) -> u64 { self.size }
    fn identity_valid(&self) -> bool { true }
}
/// Frozen by the exporting owner, before canonical occupancy receipt binding.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AssignedMetadata {
    pub checkpoint_id: String,
    pub source: Metadata<AssignedMember>,
}
impl AssignedMetadata {
    pub fn validate(&self) -> Result<(), String> {
        validate_checkpoint_id(&self.checkpoint_id)?;
        self.source.validate()
    }
}
/// Shared by archive metadata and the runtime descriptor. Matches the upper-layer ID contract.
pub fn validate_checkpoint_id(value: &str) -> Result<(), String> {
    if value.len() != 32
        || !value.bytes().all(|v| v.is_ascii_digit() || (b'a'..=b'f').contains(&v))
        || !value.bytes().any(|v| v != b'0') {
        return Err("二维图检查点编号非法".into());
    }
    Ok(())
}
#[derive(Clone, Debug, PartialEq)]
pub struct AssignedArtifact {
    pub payload_size: u64,
    pub checkpoint_id: String,
}
#[derive(Clone, Debug, PartialEq)]
pub enum ArchiveMetadata {
    Legacy(Metadata),
    Assigned(AssignedMetadata),
}
impl ArchiveMetadata {
    pub fn identity(&self) -> ArtifactIdentity {
        match self {
            Self::Legacy(_) => ArtifactIdentity::Legacy,
            Self::Assigned(m) => ArtifactIdentity::Assigned { checkpoint_id: m.checkpoint_id.clone() },
        }
    }
    pub fn compatible_with(&self, session: &Session) -> Result<(), String> {
        match self { Self::Legacy(m) => m.compatible_with(session), Self::Assigned(m) => m.source.compatible_with(session) }
    }
    pub fn estimate(&self) -> &Estimate {
        match self { Self::Legacy(m) => &m.estimate, Self::Assigned(m) => &m.source.estimate }
    }
    pub fn source_unix_timestamp_ns(&self) -> u64 {
        match self { Self::Legacy(m) => m.source_unix_timestamp_ns, Self::Assigned(m) => m.source.source_unix_timestamp_ns }
    }
    pub fn dimensions(&self) -> (u32, u32) {
        match self { Self::Legacy(m) => (m.width, m.height), Self::Assigned(m) => (m.source.width, m.source.height) }
    }
    pub fn resolution(&self) -> f64 {
        match self { Self::Legacy(m) => m.resolution, Self::Assigned(m) => m.source.resolution }
    }
    pub fn origin(&self) -> [f64; 2] {
        match self { Self::Legacy(m) => m.origin, Self::Assigned(m) => m.source.origin }
    }
    fn sizes(&self) -> [u64; 3] {
        match self {
            Self::Legacy(m) => m.members.each_ref().map(|m| m.size),
            Self::Assigned(m) => m.source.members.each_ref().map(|m| m.size),
        }
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
/// Structure-only reader for both versions. Historical digests are parsed, not executed.
/// The caller supplies a frozen owner-held stream; identifiers are not content checksums.
pub fn read_structure(file: &mut (impl Read + Seek)) -> Result<(ArchiveMetadata, u64), String> {
    let length = file.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    let mut prefix = [0; 12];
    file.read_exact(&mut prefix).map_err(|e| e.to_string())?;
    let magic = &prefix[..8];
    if magic != MAGIC && magic != ASSIGNED_MAGIC {
        return Err("不是受支持的二维完整图档案".into());
    }
    let size = u32::from_le_bytes(prefix[8..].try_into().unwrap()) as usize;
    if size == 0 || size > MAX_HEADER {
        return Err("二维档案元数据超过预算".into());
    }
    let mut bytes = vec![0; size];
    file.read_exact(&mut bytes).map_err(|e| e.to_string())?;
    let metadata = if magic == MAGIC {
        let m: Metadata = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        m.validate()?;
        ArchiveMetadata::Legacy(m)
    } else {
        let m: AssignedMetadata = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        m.validate()?;
        ArchiveMetadata::Assigned(m)
    };
    let start = 12 + size as u64;
    let expected = start + metadata.sizes().iter().sum::<u64>();
    if expected > MAX_ARCHIVE || length != expected {
        return Err("二维档案长度与成员清单不一致".into());
    }
    Ok((metadata, start))
}
/// Bind an already opened owner-held stream to its descriptor, without scanning member bodies.
pub fn read_bound_structure(file: &mut (impl Read + Seek), payload_size: u64,
    identity: &ArtifactIdentity) -> Result<(ArchiveMetadata, u64), String> {
    let (metadata, start) = read_structure(file)?;
    if payload_size != start + metadata.sizes().iter().sum::<u64>() || metadata.identity() != *identity {
        return Err("二维档案描述与结构身份不一致".into());
    }
    Ok((metadata, start))
}
pub fn read_header(file: &mut File) -> Result<(Metadata, u64), String> {
    match read_structure(file)? {
        (ArchiveMetadata::Legacy(m), start) => Ok((m, start)),
        _ => Err("旧二维图读取入口不支持编号档案".into()),
    }
}
fn read_grid(file: &mut (impl Read + Seek), start: u64, sizes: [u64; 3]) -> Result<Vec<u8>, String> {
    file.seek(SeekFrom::Start(start + sizes[0] + sizes[1])).map_err(|e| e.to_string())?;
    let mut data = vec![0; sizes[2] as usize];
    file.read_exact(&mut data).map_err(|e| e.to_string())?;
    Ok(data)
}
/// Reads only header and grid, never the two graph bodies and never a digest.
pub fn grid_structure(file: &mut (impl Read + Seek)) -> Result<(ArchiveMetadata, Vec<i8>), String> {
    let (metadata, start) = read_structure(file)?;
    let data = read_grid(file, start, metadata.sizes())?;
    grid_chunk(&data)?;
    Ok((metadata, data.into_iter().map(|v| v as i8).collect()))
}
/// Shared bounded member transfer; the observer handles grid categories during the copy.
fn copy_member(input: &mut impl Read, output: &mut impl Write, size: u64,
    buffer: &mut [u8], mut observe: impl FnMut(&[u8]) -> Result<(), String>) -> Result<(), String> {
    let mut left = size;
    while left > 0 {
        let n = buffer.len().min(left as usize);
        input.read_exact(&mut buffer[..n]).map_err(|e| e.to_string())?;
        observe(&buffer[..n])?;
        output.write_all(&buffer[..n]).map_err(|e| e.to_string())?;
        left -= n as u64;
    }
    Ok(())
}
fn grid_chunk(bytes: &[u8]) -> Result<(), String> {
    if bytes.iter().any(|b| !matches!(*b, 0 | 100 | 255)) {
        return Err("二维档案物理栅格类别非法".into());
    }
    Ok(())
}
/// Writes once from frozen member streams. Publication remains the existing owner's job.
/// The caller assigns checkpoint_id once and retains it across copies/reopens.
pub fn write_assigned(path: &Path, metadata: &AssignedMetadata,
    member_paths: [&Path; 3]) -> Result<AssignedArtifact, String> {
    metadata.validate()?;
    let mut inputs = member_paths.map(open_regular);
    let [a, b, c] = &mut inputs;
    let mut readers: [&mut dyn Read; 3] = [a.as_mut().map_err(|e| e.clone())?,
        b.as_mut().map_err(|e| e.clone())?, c.as_mut().map_err(|e| e.clone())?];
    let mut output = OpenOptions::new().write(true).create_new(true).mode(0o600)
        .open(path).map_err(|e| e.to_string())?;
    let result = (|| {
        let payload_size = write_assigned_members(&mut output, metadata, &mut readers)?;
        output.sync_all().map_err(|e| e.to_string())?;
        Ok(AssignedArtifact { payload_size, checkpoint_id: metadata.checkpoint_id.clone() })
    })();
    if result.is_err() { let _ = std::fs::remove_file(path); }
    result
}
fn write_assigned_members(output: &mut impl Write, metadata: &AssignedMetadata,
    inputs: &mut [&mut dyn Read; 3]) -> Result<u64, String> {
    let header = serde_json::to_vec(metadata).map_err(|e| e.to_string())?;
    if header.len() > MAX_HEADER { return Err("二维图元数据超过预算".into()); }
    output.write_all(ASSIGNED_MAGIC)
        .and_then(|_| output.write_all(&(header.len() as u32).to_le_bytes()))
        .and_then(|_| output.write_all(&header)).map_err(|e| e.to_string())?;
    let mut buffer = vec![0; CHUNK_BYTES];
    for (index, (member, input)) in metadata.source.members.iter().zip(inputs).enumerate() {
        copy_member(input, output, member.size, &mut buffer, |bytes| {
            if index == 2 { grid_chunk(bytes) } else { Ok(()) }
        })?;
        let mut extra = [0];
        if input.read(&mut extra).map_err(|e| e.to_string())? != 0 {
            return Err("二维档案写入期间成员增长".into());
        }
    }
    Ok(12 + header.len() as u64 + metadata.source.members.iter().map(|m| m.size).sum::<u64>())
}
/// Structure-only extraction also supports legacy import without executing old digests.
pub fn unpack_structure(file: &mut (impl Read + Seek), directory: &Path) -> Result<ArchiveMetadata, String> {
    let (metadata, _) = read_structure(file)?;
    unpack_parsed(file, directory, &metadata)?;
    Ok(metadata)
}
/// The stream must remain at the body offset returned by read_structure/read_bound_structure.
/// Reuses that parsed header and retains grid cells from the single extraction pass for native load.
pub(crate) fn unpack_parsed(file: &mut impl Read, directory: &Path,
    metadata: &ArchiveMetadata) -> Result<Vec<i8>, String> {
    let mut created = Vec::new();
    let result = (|| {
        let mut buffer = vec![0; CHUNK_BYTES];
        let mut grid = Vec::with_capacity(metadata.sizes()[2] as usize);
        for (index, (name, size)) in ["graph.posegraph", "graph.data", "grid.i8"].into_iter()
            .zip(metadata.sizes()).enumerate() {
            let path = directory.join(name);
            let mut output = OpenOptions::new().write(true).create_new(true).mode(0o600)
                .open(&path).map_err(|e| e.to_string())?;
            created.push(path);
            copy_member(file, &mut output, size, &mut buffer, |bytes| {
                if index == 2 {
                    for value in bytes {
                        if !matches!(*value, 0 | 100 | 255) { return Err("二维档案物理栅格类别非法".into()); }
                        grid.push(*value as i8);
                    }
                }
                Ok(())
            })?;
            output.sync_all().map_err(|e| e.to_string())?;
        }
        Ok(grid)
    })();
    if result.is_err() {
        for path in created { let _ = std::fs::remove_file(path); }
    }
    result
}

#[cfg(test)]
#[path = "../tests/assigned_archive/mod.rs"]
mod assigned_tests;
