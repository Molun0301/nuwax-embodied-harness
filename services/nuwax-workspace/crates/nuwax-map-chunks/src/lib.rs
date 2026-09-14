//! 固定精度三维块、完整根清单和可靠缺片恢复；不拥有传输或运动任务。
#![deny(unsafe_code)]

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

mod repository;
pub use repository::{Repository, RegionReader};
mod writer;
pub use writer::Writer;
mod archive;
pub use archive::{archive_file, ArchiveReader, MAX_ARCHIVE_BYTES};

/// 每个方向32个细体素；负坐标使用欧几里得除法。
pub const BLOCK_SIDE: i32 = 32;
pub const MAX_BLOCKS: usize = 131_072;
pub const MAX_BLOCK_BYTES: usize = 16 + 32_768 * 12;
pub const MAX_MANIFEST_BYTES: usize = 32 * 1024 * 1024;
pub type Key = [i32; 3];
pub type Hash = [u8; 32];

pub fn digest(bytes: &[u8]) -> Hash { Sha256::digest(bytes).into() }
pub fn block_key(key: Key) -> Key { key.map(|v| v.div_euclid(BLOCK_SIDE)) }
pub fn local_key(key: Key) -> u16 {
    (key[0].rem_euclid(32) + 32 * key[1].rem_euclid(32) + 1024 * key[2].rem_euclid(32)) as u16
}
pub fn global_key(block: Key, local: u16) -> Result<Key, String> {
    if local >= 32768 { return Err("块内体素编号越界".into()); }
    let local = i32::from(local);
    let offsets = [local % 32, local / 32 % 32, local / 1024];
    let mut result = [0; 3];
    for i in 0..3 {
        result[i] = block[i].checked_mul(32).and_then(|v| v.checked_add(offsets[i]))
            .ok_or("全局体素坐标溢出")?;
    }
    Ok(result)
}

/// 单体素的最后实际观测时间和独立命中次数；二者均为零表示旧资产没有时间证据。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Evidence { pub observed_ns: u64, pub hits: u16 }
pub type Block = BTreeMap<u16, Evidence>;
/// 几何身份只取占据成员，观测续期不触发全图表面重建。
pub fn block_geometry_digest(block: &Block) -> Hash {
    let mut hash = Sha256::new();
    for key in block.keys() { hash.update(key.to_le_bytes()); }
    hash.finalize().into()
}

pub fn encode_block(block: &Block) -> Result<Vec<u8>, String> {
    if block.is_empty() || block.len() > 32768 { return Err("空块必须从根清单删除".into()); }
    let mut out = b"NUWAXBL1".to_vec();
    out.extend_from_slice(&(block.len() as u32).to_le_bytes());
    out.extend_from_slice(&0_u32.to_le_bytes());
    for (&key, value) in block {
        if key >= 32768 || (value.observed_ns == 0) != (value.hits == 0) { return Err("体素观察证据无效".into()); }
        out.extend_from_slice(&key.to_le_bytes());
        out.extend_from_slice(&value.observed_ns.to_le_bytes());
        out.extend_from_slice(&value.hits.to_le_bytes());
    }
    Ok(out)
}
pub fn decode_block(bytes: &[u8]) -> Result<Block, String> {
    if bytes.len() < 16 || bytes.len() > MAX_BLOCK_BYTES || &bytes[..8] != b"NUWAXBL1" || bytes[12..16] != [0; 4] {
        return Err("精细块格式或长度无效".into());
    }
    let count = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
    if count == 0 || count > 32768 || bytes.len() != 16 + count * 12 { return Err("精细块记录数不匹配".into()); }
    let mut out = BTreeMap::new();
    let mut previous = None;
    for record in bytes[16..].chunks_exact(12) {
        let key = u16::from_le_bytes(record[..2].try_into().unwrap());
        let observed_ns = u64::from_le_bytes(record[2..10].try_into().unwrap());
        let hits = u16::from_le_bytes(record[10..12].try_into().unwrap());
        if key >= 32768 || (observed_ns == 0) != (hits == 0) || previous.is_some_and(|p| p >= key) { return Err("精细块重复、无序或无效证据".into()); }
        previous = Some(key);
        out.insert(key, Evidence { observed_ns, hits });
    }
    Ok(out)
}

/// 直接承接原地图拥有方身份，不另建独立执行代次。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Domain {
    pub session: String,
    pub generation: u64,
    pub frame_generation: u64,
    pub producer_generation: u64,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BlockRef { pub key: Key, pub sha256: Hash, pub geometry_sha256: Hash, pub points: u32 }
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub version: u32,
    pub domain: Domain,
    pub revision: u64,
    pub source_timestamp_ns: u64,
    pub voxel_size_m: f32,
    pub blocks: Vec<BlockRef>,
}

/// 与占据提交共同封存的精细块集合；身份由实际发布拥有方附加。
#[derive(Clone)]
pub struct SealedMap {
    pub voxel_size_m: f32,
    pub source_timestamp_ns: u64,
    pub blocks: Arc<Vec<BlockRef>>,
    pub repository: Repository,
}
impl SealedMap {
    pub fn manifest(&self, domain: Domain, revision: u64) -> Result<Manifest, String> {
        let manifest = Manifest { version: 1, domain, revision, source_timestamp_ns: self.source_timestamp_ns,
            voxel_size_m: self.voxel_size_m, blocks: (*self.blocks).clone() };
        manifest.validate()?;
        Ok(manifest)
    }
}
impl Manifest {
    pub fn validate(&self) -> Result<(), String> {
        if self.version != 1 || self.revision == 0 || self.source_timestamp_ns == 0
            || self.domain.session.is_empty() || self.domain.session.len() > 1024
            || self.domain.session.trim() != self.domain.session || self.domain.session.chars().any(char::is_control)
            || self.domain.generation == 0 || self.domain.producer_generation == 0
            || !self.voxel_size_m.is_finite() || self.voxel_size_m <= 0.0 || self.voxel_size_m > 1_048_576.0
            || self.blocks.len() > MAX_BLOCKS { return Err("精细地图根身份、精度或容量无效".into()); }
        let mut previous = None;
        for block in &self.blocks {
            global_key(block.key, 32767)?;
            if block.sha256 == [0; 32] || block.geometry_sha256 == [0; 32] || block.points == 0 || block.points > 32768
                || previous.is_some_and(|p| p >= block.key) { return Err("根清单块引用重复、无序或无效".into()); }
            previous = Some(block.key);
        }
        Ok(())
    }
    pub fn encode(&self) -> Result<Vec<u8>, String> {
        self.validate()?;
        let bytes = serde_json::to_vec(self).map_err(|e| e.to_string())?;
        if bytes.len() > MAX_MANIFEST_BYTES { return Err("根清单超过编码预算".into()); }
        Ok(bytes)
    }
    pub fn decode(bytes: &[u8], expected: Hash) -> Result<Self, String> {
        if bytes.len() > MAX_MANIFEST_BYTES || digest(bytes) != expected { return Err("根清单摘要或大小无效".into()); }
        let value: Self = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
        value.validate()?;
        // 确定编码同时拒绝重复字段、非规范数字及不同键顺序引入的多重内容身份。
        if value.encode()? != bytes { return Err("根清单不是规范编码".into()); }
        Ok(value)
    }
    pub fn points(&self) -> u64 { self.blocks.iter().map(|b| u64::from(b.points)).sum() }
    pub fn changed_from(&self, previous: &Self) -> Vec<Key> {
        let old: BTreeMap<_, _> = previous.blocks.iter().map(|b| (b.key, b.sha256)).collect();
        let new: BTreeMap<_, _> = self.blocks.iter().map(|b| (b.key, b.sha256)).collect();
        old.keys().chain(new.keys()).copied().collect::<BTreeSet<_>>().into_iter()
            .filter(|key| old.get(key) != new.get(key)).collect()
    }
}

/// 完整根决定成员关系；携带块只用于省去可靠补取，缺片绝不部分安装。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transfer { pub root: Vec<u8>, pub root_sha256: Hash, pub included: BTreeMap<Hash, Vec<u8>> }
pub struct Receiver {
    current: Option<Arc<Manifest>>,
    repository: Repository,
    verified: BTreeMap<Hash, (u32, u64, Hash)>,
    readers: Vec<std::sync::Weak<Manifest>>,
}
impl Receiver {
    pub fn new(repository: Repository) -> Self { Self { current: None, repository, verified: BTreeMap::new(), readers: Vec::new() } }
    pub fn current(&self) -> Option<Arc<Manifest>> { self.current.clone() }
    pub fn clear(&mut self) { self.current = None; self.verified.clear(); }
    /// 调用者先用原票据准入域及修订，并在返回后再次检查票据；本层只做完整性事务。
    pub fn receive(&mut self, transfer: &Transfer, expected: &Domain,
        fetch: impl FnMut(Hash) -> Result<Vec<u8>, String>) -> Result<Arc<Manifest>, String> {
        let candidate = Arc::new(Manifest::decode(&transfer.root, transfer.root_sha256)?);
        if &candidate.domain != expected { return Err("精细根跨地图域".into()); }
        let result = self.receive_inner(transfer, expected, fetch);
        let mut live = self.readers.iter().filter_map(std::sync::Weak::upgrade).collect::<Vec<_>>();
        self.readers.retain(|r| r.strong_count() > 0);
        live.push(candidate);
        if let Some(current) = &self.current { live.push(Arc::clone(current)); }
        let required = live.iter().flat_map(|r| r.blocks.iter().map(|b| b.sha256)).collect::<BTreeSet<_>>();
        self.verified.retain(|hash, _| required.contains(hash));
        // 保留一次未完成补取的有效块及所有实际读取中的根，限制故障重试的磁盘增长。
        let cleanup = self.repository.collect_unreferenced(&live);
        result.and_then(|root| cleanup.map(|_| root))
    }
    fn receive_inner(&mut self, transfer: &Transfer, expected: &Domain,
        mut fetch: impl FnMut(Hash) -> Result<Vec<u8>, String>) -> Result<Arc<Manifest>, String> {
        let manifest = Manifest::decode(&transfer.root, transfer.root_sha256)?;
        if &manifest.domain != expected { return Err("精细根跨地图域".into()); }
        if self.current.as_ref().is_some_and(|old| old.domain == manifest.domain && old.revision >= manifest.revision) {
            return Err("精细根修订未前进".into());
        }
        let required: BTreeSet<_> = manifest.blocks.iter().map(|b| b.sha256).collect();
        if transfer.included.keys().any(|hash| !required.contains(hash)) { return Err("携带了根清单之外的块".into()); }
        for block in &manifest.blocks {
            if let Some(&(points, latest, geometry)) = self.verified.get(&block.sha256) {
                if points != block.points || latest > manifest.source_timestamp_ns || geometry != block.geometry_sha256 { return Err("已验证块与新根引用不符".into()); }
                if !transfer.included.contains_key(&block.sha256) { continue; }
            }
            let bytes = if let Some(bytes) = transfer.included.get(&block.sha256) { bytes.clone() }
                else if let Ok(bytes) = self.repository.read_object(block.sha256, MAX_BLOCK_BYTES) { bytes }
                else { fetch(block.sha256)? };
            if digest(&bytes) != block.sha256 { return Err("补取块摘要不匹配".into()); }
            let decoded = decode_block(&bytes)?;
            if decoded.len() != block.points as usize || block_geometry_digest(&decoded) != block.geometry_sha256 || decoded.values().any(|e| e.observed_ns > manifest.source_timestamp_ns) {
                return Err("块记录数或观察时间超出根清单".into());
            }
            self.repository.put_object(block.sha256, &bytes)?;
            self.verified.insert(block.sha256, (block.points, decoded.values().map(|e| e.observed_ns).max().unwrap_or(0), block.geometry_sha256));
        }
        self.repository.put_object(transfer.root_sha256, &transfer.root)?;
        let manifest = Arc::new(manifest);
        self.current = Some(Arc::clone(&manifest));
        self.readers.push(Arc::downgrade(&manifest));
        self.verified.retain(|hash, _| required.contains(hash));
        Ok(manifest)
    }
}
