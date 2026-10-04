use super::*;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};

pub const MAX_ARCHIVE_BYTES: u64 = 1024 * 1024 * 1024;
const CURRENT_MAGIC: &[u8; 8] = b"NUWAXAR2";
const LEGACY_MAGIC: &[u8; 8] = b"NUWAXAR1";

#[derive(Debug, Clone, PartialEq)]
pub struct LegacyRoot { pub manifest: LegacyManifest, pub alias: LegacyAlias, pub raw_json: Vec<u8> }
impl LegacyRoot {
    pub fn decode(raw_json: Vec<u8>, alias: LegacyAlias) -> Result<Self, String> {
        ObjectIdentity::Legacy(alias).validate()?;
        Ok(Self { manifest: LegacyManifest::decode(&raw_json)?, alias, raw_json })
    }
}
#[derive(Debug, Clone, PartialEq)]
pub enum ArchivedRoot { Legacy(LegacyRoot), Current(Manifest) }
impl ArchivedRoot {
    pub fn domain(&self) -> &Domain { match self { Self::Legacy(r) => &r.manifest.domain, Self::Current(r) => &r.domain } }
    pub fn revision(&self) -> u64 { match self { Self::Legacy(r) => r.manifest.revision, Self::Current(r) => r.revision } }
    pub fn identity(&self) -> RootIdentity { RootIdentity { domain: self.domain().clone(), revision: self.revision() } }
    pub fn source_timestamp_ns(&self) -> u64 { match self { Self::Legacy(r) => r.manifest.source_timestamp_ns, Self::Current(r) => r.source_timestamp_ns } }
    pub fn voxel_size_m(&self) -> f32 { match self { Self::Legacy(r) => r.manifest.voxel_size_m, Self::Current(r) => r.voxel_size_m } }
    pub fn points(&self) -> u64 { match self { Self::Legacy(r) => r.manifest.points(), Self::Current(r) => r.points() } }
    pub fn objects(&self) -> Vec<(Key, ObjectIdentity, u32)> {
        match self {
            Self::Current(r) => r.blocks.iter().map(|b| (b.key, b.object, b.points)).collect(),
            Self::Legacy(r) => r.manifest.blocks.iter().map(|b| (b.key, ObjectIdentity::Legacy(b.sha256), b.points)).collect(),
        }
    }
}

/// 只写 AR2；逐块流式输出，根已有拥有方身份，无根或归档摘要槽。
pub fn archive_file(root: &Manifest, repository: &Repository) -> Result<(File, u64), String> {
    let root_bytes = root.encode()?;
    let expected_size = 24 + root_bytes.len() as u64
        + root.blocks.iter().map(|b| 20 + u64::from(b.points) * 12).sum::<u64>();
    if expected_size > MAX_ARCHIVE_BYTES { return Err("精细地图资产超出1吉比字节预算".into()); }
    let mut file = tempfile::tempfile().map_err(|e| e.to_string())?;
    let mut write = |bytes: &[u8]| file.write_all(bytes).map_err(|e| e.to_string());
    write(CURRENT_MAGIC)?;
    write(&(root_bytes.len() as u64).to_le_bytes())?;
    write(&root_bytes)?;
    write(&(root.blocks.len() as u64).to_le_bytes())?;
    for reference in &root.blocks {
        let bytes = repository.read_object(reference.object, MAX_BLOCK_BYTES)?;
        let block = decode_block(&bytes)?;
        admit_intrinsic(intrinsic(&block), reference.points, root.source_timestamp_ns)?;
        write(&(bytes.len() as u32).to_le_bytes())?;
        write(&bytes)?;
    }
    let size = file.stream_position().map_err(|e| e.to_string())?;
    if size != expected_size { return Err("精细资产编码大小不符".into()); }
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    Ok((file, size))
}

struct ArchiveObject { offset: u64, size: usize, object: ObjectIdentity, points: u32 }
/// 旧 AR1 保留原根字节和已有别名；局部读取不解包整张地图。
pub struct ArchiveReader<R> {
    reader: R,
    root: ArchivedRoot,
    offsets: BTreeMap<Key, ArchiveObject>,
    first: BTreeMap<ObjectIdentity, Key>,
    checked: BTreeSet<Key>,
}
impl<R: Read + Seek> ArchiveReader<R> {
    pub fn open(mut reader: R) -> Result<Self, String> {
        let length = reader.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
        if !(24..=MAX_ARCHIVE_BYTES).contains(&length) { return Err("精细资产文件长度无效".into()); }
        reader.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        let mut header = [0; 16]; reader.read_exact(&mut header).map_err(|e| e.to_string())?;
        let legacy = match &header[..8] {
            magic if magic == CURRENT_MAGIC => false,
            magic if magic == LEGACY_MAGIC => true,
            _ => return Err("精细资产格式未知".into()),
        };
        let size = u64::from_le_bytes(header[8..16].try_into().unwrap());
        let overhead = if legacy { 56 } else { 24 };
        if size > MAX_MANIFEST_BYTES as u64 || size + overhead > length { return Err("精细资产根超预算或截断".into()); }
        let alias = if legacy {
            let mut alias = [0; 32]; reader.read_exact(&mut alias).map_err(|e| e.to_string())?;
            Some(LegacyAlias(alias))
        } else { None };
        let mut bytes = vec![0; size as usize]; reader.read_exact(&mut bytes).map_err(|e| e.to_string())?;
        let root = if let Some(alias) = alias { ArchivedRoot::Legacy(LegacyRoot::decode(bytes, alias)?) }
            else { ArchivedRoot::Current(Manifest::decode(&bytes)?) };
        let mut count = [0; 8]; reader.read_exact(&mut count).map_err(|e| e.to_string())?;
        let objects = root.objects();
        if u64::from_le_bytes(count) != objects.len() as u64 { return Err("精细资产块数与根不符".into()); }
        let mut offsets = BTreeMap::new();
        let mut first = BTreeMap::new();
        for (key, object, points) in objects {
            let mut size = [0; 4]; reader.read_exact(&mut size).map_err(|e| e.to_string())?;
            let size = u32::from_le_bytes(size) as usize;
            if size != 16 + points as usize * 12 || size > MAX_BLOCK_BYTES { return Err("精细资产块长度不符".into()); }
            let offset = reader.stream_position().map_err(|e| e.to_string())?;
            if offset.checked_add(size as u64).is_none_or(|end| end > length) { return Err("精细资产块截断".into()); }
            first.entry(object).or_insert(key);
            offsets.insert(key, ArchiveObject { offset, size, object, points });
            reader.seek(SeekFrom::Current(size as i64)).map_err(|e| e.to_string())?;
        }
        if reader.stream_position().map_err(|e| e.to_string())? != length { return Err("精细资产带尾随数据".into()); }
        Ok(Self { reader, root, offsets, first, checked: BTreeSet::new() })
    }
    pub fn root(&self) -> &ArchivedRoot { &self.root }
    pub fn block(&mut self, reference: &BlockRef) -> Result<Vec<u8>, String> {
        let ArchivedRoot::Current(root) = &self.root else { return Err("旧资产需要旧引用或空间键读取".into()); };
        let index = root.blocks.binary_search_by_key(&reference.key, |b| b.key).map_err(|_| "精细资产不含请求块")?;
        if &root.blocks[index] != reference { return Err("请求描述与精细资产根不符".into()); }
        self.block_by_key(reference.key)
    }
    pub fn legacy_block(&mut self, reference: &LegacyBlockRef) -> Result<Vec<u8>, String> {
        let ArchivedRoot::Legacy(root) = &self.root else { return Err("新资产不能使用旧引用".into()); };
        let index = root.manifest.blocks.binary_search_by_key(&reference.key, |b| b.key).map_err(|_| "旧精细资产不含请求块")?;
        if &root.manifest.blocks[index] != reference { return Err("请求描述与旧精细资产根不符".into()); }
        self.block_by_key(reference.key)
    }
    fn read_block(&mut self, key: Key) -> Result<(Vec<u8>, Block), String> {
        let item = self.offsets.get(&key).ok_or("精细资产不含请求块")?;
        let mut bytes = vec![0; item.size];
        self.reader.seek(SeekFrom::Start(item.offset)).map_err(|e| e.to_string())?;
        self.reader.read_exact(&mut bytes).map_err(|e| e.to_string())?;
        // Validate duplicate aliases on first consumption, without an eager archive scan.
        if !self.checked.contains(&key) {
            let first_key = self.first[&item.object];
            if first_key != key {
                let first = &self.offsets[&first_key];
                let mut first_bytes = vec![0; first.size];
                self.reader.seek(SeekFrom::Start(first.offset)).map_err(|e| e.to_string())?;
                self.reader.read_exact(&mut first_bytes).map_err(|e| e.to_string())?;
                if first_bytes != bytes { return Err("same archive object identity has conflicting bytes".into()); }
            }
        }
        let block = decode_block(&bytes)?;
        admit_intrinsic(intrinsic(&block), item.points, self.root.source_timestamp_ns())?;
        self.checked.insert(key);
        Ok((bytes, block))
    }
    pub fn block_by_key(&mut self, key: Key) -> Result<Vec<u8>, String> { self.read_block(key).map(|(bytes, _)| bytes) }
    /// 全部验证完成才交付可用根；无引用的部分导入交给临时目录或拥有方 GC。
    pub fn import(&mut self, repository: &Repository) -> Result<ArchivedRoot, String> {
        let objects = self.offsets.iter().map(|(&key, item)| (key, item.object)).collect::<Vec<_>>();
        let mut installed = BTreeSet::new();
        for (key, id) in objects {
            let bytes = self.block_by_key(key)?;
            if installed.insert(id) { repository.put_object(id, &bytes)?; }
        }
        if let ArchivedRoot::Legacy(root) = &self.root { repository.install_legacy_root(root)?; }
        Ok(self.root.clone())
    }
    pub fn region(&mut self, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        if (0..3).any(|i| min[i] > max[i]) { return Err("精细资产局部边界倒置".into()); }
        let low = block_key(min); let high = block_key(max);
        let selected = self.offsets.range([low[0], i32::MIN, i32::MIN]..=[high[0], i32::MAX, i32::MAX])
            .map(|(&key, _)| key).filter(|key| (1..3).all(|i| key[i] >= low[i] && key[i] <= high[i])).collect::<Vec<_>>();
        let mut result = Vec::new();
        for block_key in selected {
            for (local, evidence) in self.read_block(block_key)?.1 {
                let key = global_key(block_key, local)?;
                if (0..3).all(|i| key[i] >= min[i] && key[i] <= max[i]) {
                    if result.len() >= point_budget { return Err("精细资产局部点预算耗尽".into()); }
                    result.push((key, evidence));
                }
            }
        }
        Ok(result)
    }
}
