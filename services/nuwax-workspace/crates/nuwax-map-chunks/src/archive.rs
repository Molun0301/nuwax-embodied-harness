use super::*;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};

pub const MAX_ARCHIVE_BYTES: u64 = 1024 * 1024 * 1024;
const MAGIC: &[u8; 8] = b"NUWAXAR1";

/// 流式生成匿名精细资产；块边界保留，文件大小不决定常驻内存。
pub fn archive_file(root: &Manifest, repository: &Repository) -> Result<(File, u64, Hash), String> {
    let root_bytes = root.encode()?;
    let expected_size = 56 + root_bytes.len() as u64 + root.blocks.iter().map(|b| 20 + u64::from(b.points) * 12).sum::<u64>();
    if expected_size > MAX_ARCHIVE_BYTES { return Err("精细地图资产超出1吉比字节预算".into()); }
    let mut file = tempfile::tempfile().map_err(|e| e.to_string())?;
    let mut sha = Sha256::new();
    let mut write = |bytes: &[u8]| -> Result<(), String> { file.write_all(bytes).map_err(|e| e.to_string())?; sha.update(bytes); Ok(()) };
    write(MAGIC)?;
    write(&(root_bytes.len() as u64).to_le_bytes())?;
    write(&digest(&root_bytes))?;
    write(&root_bytes)?;
    write(&(root.blocks.len() as u64).to_le_bytes())?;
    for block in &root.blocks {
        let bytes = repository.read_object(block.sha256, MAX_BLOCK_BYTES)?;
        let decoded = decode_block(&bytes)?;
        if decoded.len() != block.points as usize || block_geometry_digest(&decoded) != block.geometry_sha256
            || decoded.values().any(|e| e.observed_ns > root.source_timestamp_ns) { return Err("精细资产块与根不符".into()); }
        write(&(bytes.len() as u32).to_le_bytes())?;
        write(&bytes)?;
    }
    let size = file.stream_position().map_err(|e| e.to_string())?;
    if size != expected_size { return Err("精细资产编码大小不符".into()); }
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    Ok((file, size, sha.finalize().into()))
}

/// 从完整根定位块文件偏移，局部读取不解包整张地图。
pub struct ArchiveReader<R> {
    reader: R,
    root: Manifest,
    offsets: BTreeMap<Key, (u64, usize)>,
}
impl<R: Read + Seek> ArchiveReader<R> {
    pub fn open(mut reader: R) -> Result<Self, String> {
        let length = reader.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
        if length > MAX_ARCHIVE_BYTES || length < 56 { return Err("精细资产文件长度无效".into()); }
        reader.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        let mut header = [0; 48]; reader.read_exact(&mut header).map_err(|e| e.to_string())?;
        if &header[..8] != MAGIC { return Err("精细资产格式未知".into()); }
        let size = u64::from_le_bytes(header[8..16].try_into().unwrap());
        if size > MAX_MANIFEST_BYTES as u64 { return Err("精细资产根超预算".into()); }
        let mut bytes = vec![0; size as usize]; reader.read_exact(&mut bytes).map_err(|e| e.to_string())?;
        let root = Manifest::decode(&bytes, header[16..48].try_into().unwrap())?;
        let mut count = [0; 8]; reader.read_exact(&mut count).map_err(|e| e.to_string())?;
        if u64::from_le_bytes(count) != root.blocks.len() as u64 { return Err("精细资产块数与根不符".into()); }
        let mut offsets = BTreeMap::new();
        for block in &root.blocks {
            let mut size = [0; 4]; reader.read_exact(&mut size).map_err(|e| e.to_string())?;
            let size = u32::from_le_bytes(size) as usize;
            if size != 16 + block.points as usize * 12 { return Err("精细资产块长度不符".into()); }
            let offset = reader.stream_position().map_err(|e| e.to_string())?;
            offsets.insert(block.key, (offset, size));
            reader.seek(SeekFrom::Current(size as i64)).map_err(|e| e.to_string())?;
        }
        if reader.stream_position().map_err(|e| e.to_string())? != length { return Err("精细资产截断或带尾随数据".into()); }
        Ok(Self { reader, root, offsets })
    }
    pub fn manifest(&self) -> &Manifest { &self.root }
    pub fn block(&mut self, reference: &BlockRef) -> Result<Vec<u8>, String> {
        let &(offset, size) = self.offsets.get(&reference.key).ok_or("精细资产不含请求块")?;
        let mut bytes = vec![0; size];
        self.reader.seek(SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
        self.reader.read_exact(&mut bytes).map_err(|e| e.to_string())?;
        if digest(&bytes) != reference.sha256 { return Err("精细资产块摘要错误".into()); }
        let block = decode_block(&bytes)?;
        if block.len() != reference.points as usize || block_geometry_digest(&block) != reference.geometry_sha256
            || block.values().any(|e| e.observed_ns > self.root.source_timestamp_ns) { return Err("精细资产几何或观察证据错误".into()); }
        Ok(bytes)
    }
    /// 激活前逐块验证并导入；失败不交付可用根，未引用对象可由临时目录释放。
    pub fn import(&mut self, repository: &Repository) -> Result<Manifest, String> {
        for reference in self.root.blocks.clone() {
            let bytes = self.block(&reference)?;
            repository.put_object(reference.sha256, &bytes)?;
        }
        let bytes = self.root.encode()?;
        repository.put_object(digest(&bytes), &bytes)?;
        Ok(self.root.clone())
    }
    pub fn region(&mut self, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        if (0..3).any(|i| min[i] > max[i]) { return Err("精细资产局部边界倒置".into()); }
        let low = block_key(min); let high = block_key(max);
        let selected = self.root.blocks.iter().filter(|b| (0..3).all(|i| b.key[i] >= low[i] && b.key[i] <= high[i])).cloned().collect::<Vec<_>>();
        let mut result = Vec::new();
        for reference in selected {
            for (local, evidence) in decode_block(&self.block(&reference)?)? {
                let key = global_key(reference.key, local)?;
                if (0..3).all(|i| key[i] >= min[i] && key[i] <= max[i]) {
                    if result.len() >= point_budget { return Err("精细资产局部点预算耗尽".into()); }
                    result.push((key, evidence));
                }
            }
        }
        Ok(result)
    }
}
