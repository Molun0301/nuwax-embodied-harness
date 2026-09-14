use super::*;
use std::fs::{self, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

static SERIAL: AtomicU64 = AtomicU64::new(0);

/// 内容寻址对象目录；远程调用者通过传输字节，不传本地路径。
#[derive(Clone)]
pub struct Repository { root: Arc<PathBuf>, _temporary: Option<Arc<tempfile::TempDir>> }
impl Repository {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, String> {
        fs::create_dir_all(path.as_ref()).map_err(|e| e.to_string())?;
        Ok(Self { root: Arc::new(path.as_ref().to_path_buf()), _temporary: None })
    }
    /// 运行期对象由最后一个拥有方释放；资产保存另用持久目录。
    pub fn temporary() -> Result<Self, String> {
        let directory = tempfile::Builder::new().prefix("nuwax-map-chunks-").tempdir().map_err(|e| e.to_string())?;
        Ok(Self { root: Arc::new(directory.path().to_path_buf()), _temporary: Some(Arc::new(directory)) })
    }
    pub fn path(&self) -> &Path { &self.root }
    fn object_path(&self, hash: Hash) -> PathBuf {
        self.root.join(hash.iter().map(|v| format!("{v:02x}")).collect::<String>())
    }
    /// 有界分配并重新核对摘要；截断、写坏和错对象不会返回可用载荷。
    pub fn read_object(&self, hash: Hash, max_bytes: usize) -> Result<Vec<u8>, String> {
        let path = self.object_path(hash);
        let meta = fs::symlink_metadata(&path).map_err(|e| e.to_string())?;
        if !meta.file_type().is_file() || meta.len() > max_bytes as u64 { return Err("对象类型或大小无效".into()); }
        let mut bytes = Vec::with_capacity(meta.len() as usize);
        fs::File::open(&path).map_err(|e| e.to_string())?.take(max_bytes as u64 + 1)
            .read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        if bytes.len() > max_bytes || digest(&bytes) != hash { return Err("对象长度或摘要无效".into()); }
        Ok(bytes)
    }
    /// 写入临时文件后原子替换；已有正确对象直接复用。
    pub fn put_object(&self, hash: Hash, bytes: &[u8]) -> Result<(), String> {
        if bytes.len() > MAX_MANIFEST_BYTES || digest(bytes) != hash { return Err("待写对象摘要或大小无效".into()); }
        if self.read_object(hash, bytes.len()).is_ok() { return Ok(()); }
        let temporary = self.root.join(format!(".pending-{}-{}", std::process::id(), SERIAL.fetch_add(1, Ordering::Relaxed)));
        let result = (|| {
            let mut file = OpenOptions::new().write(true).create_new(true).open(&temporary).map_err(|e| e.to_string())?;
            file.write_all(bytes).map_err(|e| e.to_string())?;
            // 运行期目录只承载同机完整对象，关闭即释放；持久资产才等待存储设备。
            // write_all完成后原子改名，读者仍核对完整长度和内容摘要。
            if self._temporary.is_none() { file.sync_all().map_err(|e| e.to_string())?; }
            fs::rename(&temporary, self.object_path(hash)).map_err(|e| e.to_string())?;
            Ok(())
        })();
        if result.is_err() { let _ = fs::remove_file(temporary); }
        result
    }
    pub fn read_manifest(&self, hash: Hash) -> Result<Manifest, String> {
        Manifest::decode(&self.read_object(hash, MAX_MANIFEST_BYTES)?, hash)
    }
    /// 资产导出必须完整复制根所引用的对象，再交付根摘要。
    pub fn copy_snapshot(&self, manifest: &Manifest, destination: &Self) -> Result<Hash, String> {
        manifest.validate()?;
        for block in &manifest.blocks {
            let bytes = self.read_object(block.sha256, MAX_BLOCK_BYTES)?;
            let decoded = decode_block(&bytes)?;
            if decoded.len() != block.points as usize || block_geometry_digest(&decoded) != block.geometry_sha256 { return Err("导出块几何或记录数不符".into()); }
            destination.put_object(block.sha256, &bytes)?;
        }
        let bytes = manifest.encode()?;
        let hash = digest(&bytes);
        destination.put_object(hash, &bytes)?;
        fs::File::open(destination.path()).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?;
        Ok(hash)
    }
    /// 仅由拥有方列出全部仍有读取权的根；删除无引用历史对象。
    pub fn collect_unreferenced(&self, roots: &[Arc<Manifest>]) -> Result<usize, String> {
        let mut live = BTreeSet::new();
        for root in roots {
            live.insert(self.object_path(digest(&root.encode()?)));
            for block in &root.blocks { live.insert(self.object_path(block.sha256)); }
        }
        self.collect_paths(live)
    }
    /// 运行中的封存块不另写根文件；保持所有仍在读取的块引用。
    pub fn collect_sealed(&self, roots: &[Arc<SealedMap>]) -> Result<usize, String> {
        let live = roots.iter().flat_map(|root| root.blocks.iter().map(|b| self.object_path(b.sha256))).collect();
        self.collect_paths(live)
    }
    fn collect_paths(&self, live: BTreeSet<PathBuf>) -> Result<usize, String> {
        let mut removed = 0;
        for entry in fs::read_dir(self.path()).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            let name = entry.file_name();
            let Some(name) = name.to_str() else { continue; };
            if name.len() == 64 && name.bytes().all(|b| b.is_ascii_hexdigit()) && !live.contains(&entry.path()) {
                fs::remove_file(entry.path()).map_err(|e| e.to_string())?;
                removed += 1;
            }
        }
        Ok(removed)
    }
}

/// 块缓存按实际点数估算内存并限定容量，查询的结果另受点预算约束。
pub struct RegionReader {
    repository: Repository,
    cache: BTreeMap<Hash, (Arc<Block>, u64, usize)>,
    budget_bytes: usize,
    used_bytes: usize,
    clock: u64,
}
impl RegionReader {
    pub fn new(repository: Repository, budget_bytes: usize) -> Self {
        Self { repository, cache: BTreeMap::new(), budget_bytes, used_bytes: 0, clock: 0 }
    }
    pub fn cached_bytes(&self) -> usize { self.used_bytes }
    pub fn block(&mut self, reference: &BlockRef) -> Result<Arc<Block>, String> {
        self.clock = self.clock.wrapping_add(1);
        if let Some((block, age, _)) = self.cache.get_mut(&reference.sha256) {
            if block.len() != reference.points as usize || block_geometry_digest(block) != reference.geometry_sha256 { return Err("缓存块几何或记录数与根不符".into()); }
            *age = self.clock;
            return Ok(Arc::clone(block));
        }
        let block = Arc::new(decode_block(&self.repository.read_object(reference.sha256, MAX_BLOCK_BYTES)?)?);
        if block.len() != reference.points as usize || block_geometry_digest(&block) != reference.geometry_sha256 { return Err("局部块几何或记录数与根不符".into()); }
        let bytes = block.len() * 64 + 128;
        if bytes <= self.budget_bytes {
            while self.used_bytes + bytes > self.budget_bytes {
                let Some(key) = self.cache.iter().min_by_key(|(_, (_, age, _))| *age).map(|(key, _)| *key) else { break; };
                self.used_bytes -= self.cache.remove(&key).unwrap().2;
            }
            self.cache.insert(reference.sha256, (Arc::clone(&block), self.clock, bytes));
            self.used_bytes += bytes;
        }
        Ok(block)
    }
    /// 包含边界的体素查询；空间范围在读取前过滤，超预算整体报错。
    pub fn region(&mut self, root: &Manifest, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        root.validate()?;
        if (0..3).any(|i| min[i] > max[i]) { return Err("局部范围倒置".into()); }
        let low = block_key(min); let high = block_key(max);
        let mut out = Vec::new();
        // 清单有序，按第一坐标跳过整个远场；不读取远场内容对象。
        let start = root.blocks.partition_point(|b| b.key[0] < low[0]);
        for reference in root.blocks[start..].iter().take_while(|b| b.key[0] <= high[0]) {
            if (1..3).any(|i| reference.key[i] < low[i] || reference.key[i] > high[i]) { continue; }
            for (&local, &evidence) in self.block(reference)?.iter() {
                let key = global_key(reference.key, local)?;
                if (0..3).any(|i| key[i] < min[i] || key[i] > max[i]) { continue; }
                if evidence.observed_ns > root.source_timestamp_ns { return Err("局部证据晚于提交根".into()); }
                if out.len() >= point_budget { return Err("局部精细点查询超出预算".into()); }
                out.push((key, evidence));
            }
        }
        Ok(out)
    }
}
