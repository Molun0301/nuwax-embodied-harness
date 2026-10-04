//! 固定精度三维块、完整根清单和可靠缺片恢复；不拥有传输或运动任务。
#![deny(unsafe_code)]

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

mod repository;
pub use repository::{Repository, RegionReader};
mod writer;
pub use writer::Writer;
mod archive;
pub use archive::{archive_file, ArchiveReader, ArchivedRoot, LegacyRoot, MAX_ARCHIVE_BYTES};

/// 每个方向32个细体素；负坐标使用欧几里得除法。
pub const BLOCK_SIDE: i32 = 32;
pub const MAX_BLOCKS: usize = 131_072;
pub const MAX_BLOCK_BYTES: usize = 16 + 32_768 * 12;
pub const MAX_MANIFEST_BYTES: usize = 32 * 1024 * 1024;
pub type Key = [i32; 3];

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

/// Assigned identity of immutable bytes, never derived from their content.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ObjectId(pub [u8; 16]);
impl ObjectId {
    pub fn random() -> Result<Self, String> {
        loop { let mut bytes = [0; 16]; getrandom::fill(&mut bytes).map_err(|e| e.to_string())?;
            if bytes != [0; 16] { return Ok(Self(bytes)); } }
    }
}
/// Historical alias retained verbatim without recomputing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LegacyAlias(pub [u8; 32]);
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case", deny_unknown_fields)]
pub enum ObjectIdentity { Assigned(ObjectId), Legacy(LegacyAlias) }
impl From<ObjectId> for ObjectIdentity { fn from(id: ObjectId) -> Self { Self::Assigned(id) } }
impl From<LegacyAlias> for ObjectIdentity { fn from(id: LegacyAlias) -> Self { Self::Legacy(id) } }
impl ObjectIdentity {
    pub fn validate(self) -> Result<(), String> {
        let valid = match self { Self::Assigned(v) => v.0 != [0; 16], Self::Legacy(v) => v.0 != [0; 32] };
        if valid { Ok(()) } else { Err("zero object identity".into()) }
    }
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Domain { pub session: String, pub generation: u64, pub frame_generation: u64, pub producer_generation: u64 }
impl Domain {
    pub fn validate(&self) -> Result<(), String> {
        if self.session.is_empty() || self.session.len() > 1024 || self.session.trim() != self.session
            || self.session.chars().any(char::is_control) || self.generation == 0 || self.producer_generation == 0 {
            Err("invalid root domain".into())
        } else { Ok(()) }
    }
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RootIdentity { pub domain: Domain, pub revision: u64 }
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BlockRef { pub key: Key, pub object: ObjectIdentity, pub block_revision: u64, pub geometry_revision: u64, pub points: u32 }
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub version: u32, pub domain: Domain, pub revision: u64, pub geometry_revision: u64,
    pub source_timestamp_ns: u64, pub voxel_size_m: f32, pub blocks: Vec<BlockRef>,
}
/// The v1 spelling is a wire format only. Both aliases are opaque.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegacyBlockRef { pub key: Key, pub sha256: LegacyAlias, pub geometry_sha256: LegacyAlias, pub points: u32 }
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegacyManifest {
    pub version: u32, pub domain: Domain, pub revision: u64, pub source_timestamp_ns: u64,
    pub voxel_size_m: f32, pub blocks: Vec<LegacyBlockRef>,
}
fn validate_header(domain: &Domain, revision: u64, timestamp: u64, voxel: f32, count: usize) -> Result<(), String> {
    domain.validate()?;
    if revision == 0 || timestamp == 0 || !voxel.is_finite() || voxel <= 0.0 || voxel > 1_048_576.0 || count > MAX_BLOCKS {
        return Err("invalid root revision, evidence bound, resolution or capacity".into());
    }
    Ok(())
}
fn validate_members<'a>(members: impl Iterator<Item = (Key, ObjectIdentity, u32)> + 'a) -> Result<(), String> {
    let mut previous = None; let mut intrinsic = BTreeMap::new();
    for (key, object, points) in members {
        global_key(key, 32767)?; object.validate()?;
        if points == 0 || points > 32768 || previous.is_some_and(|old| old >= key) { return Err("invalid, duplicate or unsorted block reference".into()); }
        if intrinsic.insert(object, points).is_some_and(|old| old != points) { return Err("same object with inconsistent point count".into()); }
        previous = Some(key);
    }
    Ok(())
}
impl Manifest {
    pub fn validate(&self) -> Result<(), String> {
        validate_header(&self.domain, self.revision, self.source_timestamp_ns, self.voxel_size_m, self.blocks.len())?;
        if self.version != 2 || self.geometry_revision == 0 || self.geometry_revision > self.revision { return Err("invalid current root revision".into()); }
        validate_members(self.blocks.iter().map(|b| (b.key, b.object, b.points)))?;
        for b in &self.blocks {
            if b.geometry_revision == 0 || b.geometry_revision > b.block_revision || b.block_revision > self.revision || b.geometry_revision > self.geometry_revision {
                return Err("invalid block revision ordering".into());
            }
        }
        Ok(())
    }
    pub fn identity(&self) -> RootIdentity { RootIdentity { domain: self.domain.clone(), revision: self.revision } }
    pub fn encode(&self) -> Result<Vec<u8>, String> {
        self.validate()?; let bytes = serde_json::to_vec(self).map_err(|e| e.to_string())?;
        if bytes.len() > MAX_MANIFEST_BYTES { return Err("root exceeds encoding budget".into()); } Ok(bytes)
    }
    pub fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() > MAX_MANIFEST_BYTES { return Err("root exceeds encoding budget".into()); }
        let value: Self = serde_json::from_slice(bytes).map_err(|e| e.to_string())?; value.validate()?; Ok(value)
    }
    pub fn points(&self) -> u64 { self.blocks.iter().map(|b| u64::from(b.points)).sum() }
    pub fn changed_from(&self, old: &Self) -> Vec<Key> {
        let a: BTreeMap<_, _> = old.blocks.iter().map(|b| (b.key, b.object)).collect();
        let b: BTreeMap<_, _> = self.blocks.iter().map(|b| (b.key, b.object)).collect();
        let all = self.domain != old.domain || self.voxel_size_m != old.voxel_size_m;
        a.keys().chain(b.keys()).copied().collect::<BTreeSet<_>>().into_iter().filter(|k| all || a.get(k) != b.get(k)).collect()
    }
}
impl LegacyManifest {
    pub fn validate(&self) -> Result<(), String> {
        validate_header(&self.domain, self.revision, self.source_timestamp_ns, self.voxel_size_m, self.blocks.len())?;
        if self.version != 1 || self.blocks.iter().any(|b| b.geometry_sha256.0 == [0;32]) { return Err("invalid legacy root".into()); }
        validate_members(self.blocks.iter().map(|b| (b.key, ObjectIdentity::Legacy(b.sha256), b.points)))
    }
    pub fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() > MAX_MANIFEST_BYTES { return Err("legacy root exceeds budget".into()); }
        let root: Self = serde_json::from_slice(bytes).map_err(|e| e.to_string())?; root.validate()?; Ok(root)
    }
    pub fn points(&self) -> u64 { self.blocks.iter().map(|b| u64::from(b.points)).sum() }
}
#[derive(Clone)]
pub struct SealedMap { root: Arc<Manifest>, pub repository: Repository }
impl SealedMap { pub fn manifest(&self) -> &Arc<Manifest> { &self.root } }
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transfer { pub root: Vec<u8>, pub included: BTreeMap<ObjectIdentity, Vec<u8>> }
#[derive(Debug, Clone, Copy)]
struct Intrinsic { points: u32, latest: u64 }
fn intrinsic(block: &Block) -> Intrinsic {
    Intrinsic { points: block.len() as u32, latest: block.values().map(|e| e.observed_ns).max().unwrap_or(0) }
}
fn admit_intrinsic(value: Intrinsic, points: u32, bound: u64) -> Result<(), String> {
    if value.points != points || value.latest > bound { Err("block count or evidence exceeds root".into()) } else { Ok(()) }
}
pub struct Receiver {
    current: Option<Arc<Manifest>>, repository: Repository, reader: RegionReader,
    verified: BTreeMap<ObjectIdentity, Intrinsic>, readers: Vec<std::sync::Weak<Manifest>>,
}
impl Receiver {
    pub fn new(repository: Repository) -> Self { Self { current: None, reader: RegionReader::new(repository.clone(), 4 * 1024 * 1024), repository, verified: BTreeMap::new(), readers: Vec::new() } }
    pub fn current(&self) -> Option<Arc<Manifest>> { self.current.clone() }
    pub fn clear(&mut self) { self.current = None; self.verified.clear(); }
    pub fn receive(&mut self, transfer: &Transfer, expected: &Domain,
        mut fetch: impl FnMut(&Manifest, &BlockRef) -> Result<Vec<u8>, String>) -> Result<Arc<Manifest>, String> {
        let candidate = Arc::new(Manifest::decode(&transfer.root)?);
        if &candidate.domain != expected { return Err("root belongs to another domain".into()); }
        let same = self.current.as_ref().is_some_and(|old| **old == *candidate);
        if !same && self.current.as_ref().is_some_and(|old| old.domain == candidate.domain && old.revision >= candidate.revision) {
            return Err("root revision did not advance".into());
        }
        // A true duplicate with no payload does no object IO, membership read or GC scan.
        if same && transfer.included.is_empty() { return Ok(self.current.as_ref().unwrap().clone()); }
        let result = (|| {
            let required: BTreeMap<_, _> = candidate.blocks.iter().map(|b| (b.object, b.points)).collect();
            for (&object, bytes) in &transfer.included {
                let points = *required.get(&object).ok_or("included object is outside root")?;
                let decoded = Arc::new(decode_block(bytes)?); let info = intrinsic(&decoded);
                admit_intrinsic(info, points, candidate.source_timestamp_ns)?;
                self.repository.put_object(object, bytes)?;
                self.reader.cache_decoded(object, decoded, info);
                self.verified.insert(object, info);
            }
            if same { return Ok(self.current.as_ref().unwrap().clone()); }
            for block in &candidate.blocks {
                if let Some(&info) = self.verified.get(&block.object) {
                    admit_intrinsic(info, block.points, candidate.source_timestamp_ns)?; continue;
                }
                let bytes = match self.repository.read_object(block.object, MAX_BLOCK_BYTES) {
                    Ok(bytes) => bytes,
                    Err(error) => {
                        // Existing unreadable/corrupt paths cannot be replaced by a fetch.
                        if self.repository.object_exists(block.object)? { return Err(error); }
                        let bytes = fetch(&candidate, block)?;
                        let decoded = Arc::new(decode_block(&bytes)?); let info = intrinsic(&decoded);
                        admit_intrinsic(info, block.points, candidate.source_timestamp_ns)?;
                        self.repository.put_object(block.object, &bytes)?;
                        self.reader.cache_decoded(block.object, decoded, info);
                        self.verified.insert(block.object, info); continue;
                    }
                };
                let decoded = Arc::new(decode_block(&bytes)?); let info = intrinsic(&decoded);
                admit_intrinsic(info, block.points, candidate.source_timestamp_ns)?;
                self.reader.cache_decoded(block.object, decoded, info);
                        self.verified.insert(block.object, info);
            }
            if let Some(old) = self.current.clone() {
                if old.domain == candidate.domain { self.validate_progression(&old, &candidate)?; }
            }
            self.current = Some(candidate.clone()); self.readers.push(Arc::downgrade(&candidate));
            Ok(candidate.clone())
        })();
        // Valid duplicate inclusions cannot create garbage outside the pinned current root.
        if same { return result; }
        let mut live: Vec<_> = self.readers.iter().filter_map(std::sync::Weak::upgrade).collect();
        self.readers.retain(|r| r.strong_count() > 0);
        live.push(candidate); if let Some(root) = &self.current { live.push(root.clone()); }
        let objects: BTreeSet<_> = live.iter().flat_map(|r| r.blocks.iter().map(|b| b.object)).collect();
        self.verified.retain(|id, _| objects.contains(id));
        self.reader.retain_objects(&objects);
        let cleanup = self.repository.collect_unreferenced(&live);
        result.and_then(|root| cleanup.map(|_| root))
    }
    fn validate_progression(&mut self, old: &Manifest, new: &Manifest) -> Result<(), String> {
        if new.geometry_revision < old.geometry_revision { return Err("root geometry revision rolled back".into()); }
        let previous: BTreeMap<_, _> = old.blocks.iter().map(|b| (b.key, b)).collect();
        let next: BTreeSet<_> = new.blocks.iter().map(|b| b.key).collect();
        let mut membership_changed = previous.keys().any(|key| !next.contains(key));
        for b in &new.blocks {
            let Some(a) = previous.get(&b.key) else { membership_changed = true; continue; };
            if b.block_revision < a.block_revision || b.geometry_revision < a.geometry_revision { return Err("block revision rolled back".into()); }
            if b.object != a.object {
                if b.block_revision <= a.block_revision { return Err("changed object needs advanced evidence revision".into()); }
                let after = self.reader.block(b)?; let before = self.reader.block(a)?;
                if before.keys().ne(after.keys()) {
                    membership_changed = true;
                    if b.geometry_revision <= a.geometry_revision { return Err("changed members need advanced geometry revision".into()); }
                }
            }
        }
        if membership_changed && new.geometry_revision <= old.geometry_revision { return Err("changed membership needs advanced root geometry revision".into()); }
        Ok(())
    }
}
#[cfg(test)]
mod tests;
