use super::*;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;

/// Typed immutable object directory; legacy filenames retain their exact namespace.
#[derive(Clone)]
pub struct Repository {
    root: Arc<PathBuf>, _temporary: Option<Arc<tempfile::TempDir>>,
    #[cfg(test)]
    stats: Arc<Stats>,
}
/// 只读对象目录；打开入口不创建目录，也不持有写入方的临时目录所有权。
#[derive(Clone)]
pub struct ReadOnlyRepository {
    root: Arc<PathBuf>,
    #[cfg(test)]
    stats: Arc<Stats>,
}
impl ReadOnlyRepository {
    pub fn open(path: impl AsRef<Path>) -> Self {
        Self { root: Arc::new(path.as_ref().to_path_buf()), #[cfg(test)] stats: Arc::default() }
    }
}
impl From<Repository> for ReadOnlyRepository {
    fn from(repository: Repository) -> Self { repository.read_only() }
}
#[cfg(test)]
struct Stats { reads: std::sync::atomic::AtomicUsize, puts: std::sync::atomic::AtomicUsize, scans: std::sync::atomic::AtomicUsize, fail_after: std::sync::atomic::AtomicIsize }
#[cfg(test)]
impl Default for Stats {
    fn default() -> Self { Self { reads: 0.into(), puts: 0.into(), scans: 0.into(), fail_after: (-1).into() } }
}
impl Repository {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, String> {
        fs::create_dir_all(path.as_ref()).map_err(|e| e.to_string())?;
        Ok(Self { root: Arc::new(path.as_ref().to_path_buf()), _temporary: None, #[cfg(test)] stats: Arc::default() })
    }
    pub fn temporary() -> Result<Self, String> {
        let directory = tempfile::Builder::new().prefix("nuwax-map-chunks-").tempdir().map_err(|e| e.to_string())?;
        Ok(Self { root: Arc::new(directory.path().to_path_buf()), _temporary: Some(Arc::new(directory)), #[cfg(test)] stats: Arc::default() })
    }
    pub fn path(&self) -> &Path { &self.root }
    pub fn read_only(&self) -> ReadOnlyRepository {
        ReadOnlyRepository { root: self.root.clone(), #[cfg(test)] stats: self.stats.clone() }
    }
    fn object_path(&self, object: ObjectIdentity) -> PathBuf { self.read_only().object_path(object) }
    pub(crate) fn object_exists(&self, object: ObjectIdentity) -> Result<bool, String> {
        match fs::symlink_metadata(self.read_only().object_path(object)) {
            Ok(_) => Ok(true), Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(false), Err(e) => Err(e.to_string()),
        }
    }
    pub fn read_object(&self, object: ObjectIdentity, max_bytes: usize) -> Result<Vec<u8>, String> {
        self.read_only().read_object(object, max_bytes)
    }
    /// Publish once using a no-replace hard link. Existing IDs must have exact equal bytes.
    pub fn put_object(&self, object: ObjectIdentity, bytes: &[u8]) -> Result<(), String> {
        object.validate()?;
        if bytes.len() > MAX_BLOCK_BYTES.max(MAX_MANIFEST_BYTES) { return Err("object exceeds repository budget".into()); }
        #[cfg(test)] {
            use std::sync::atomic::Ordering::Relaxed;
            self.stats.puts.fetch_add(1, Relaxed);
            let left = self.stats.fail_after.load(Relaxed);
            if left == 0 { return Err("injected object write failure".into()); }
            if left > 0 { self.stats.fail_after.fetch_sub(1, Relaxed); }
        }
        if self.object_exists(object)? {
            return if self.read_object(object, bytes.len())? == bytes { Ok(()) }
                else { Err("immutable object identity conflicts with existing bytes".into()) };
        }
        let mut temporary = tempfile::Builder::new().prefix(".pending-").tempfile_in(self.path()).map_err(|e| e.to_string())?;
        temporary.write_all(bytes).map_err(|e| e.to_string())?;
        if self._temporary.is_none() { temporary.as_file().sync_all().map_err(|e| e.to_string())?; }
        match fs::hard_link(temporary.path(), self.object_path(object)) {
            Ok(()) => {
                if self._temporary.is_none() { File::open(self.path()).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?; }
                Ok(())
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                if self.read_object(object, bytes.len())? == bytes { Ok(()) } else { Err("immutable object identity conflicts with existing bytes".into()) }
            }
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn copy_snapshot(&self, root: &Manifest, destination: &Self) -> Result<(), String> {
        root.validate()?; let mut copied = BTreeSet::new();
        for block in &root.blocks {
            if !copied.insert(block.object) { continue; }
            let bytes = self.read_object(block.object, MAX_BLOCK_BYTES)?;
            let (decoded, records) = decode_block_parts(&bytes)?;
            admit_intrinsic(intrinsic(&decoded), block.points, root.source_timestamp_ns)?;
            admit_format(intrinsic_bytes(&bytes, &decoded), root)?;
            ground_shared::admit(intrinsic_bytes(&bytes, &decoded), &records, root)?;
            destination.put_object(block.object, &bytes)?;
        }
        Ok(())
    }
    pub fn collect_unreferenced(&self, roots: &[Arc<Manifest>]) -> Result<usize, String> {
        self.collect_objects(&roots.iter().flat_map(|r| r.blocks.iter().map(|b| b.object)).collect())
    }
    pub fn collect_sealed(&self, roots: &[Arc<SealedMap>]) -> Result<usize, String> {
        self.collect_objects(&roots.iter().flat_map(|r| r.root.blocks.iter().map(|b| b.object)).collect())
    }
    pub fn collect_objects(&self, live: &BTreeSet<ObjectIdentity>) -> Result<usize, String> {
        #[cfg(test)] self.stats.scans.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let live: BTreeSet<_> = live.iter().map(|id| self.object_path(*id)).collect();
        let mut removed = 0;
        for entry in fs::read_dir(self.path()).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?; let name = entry.file_name();
            let Some(name) = name.to_str() else { continue; };
            let typed = name.strip_prefix("id-").is_some_and(|s| s.len() == 32 && lowercase_hex(s));
            if typed && !live.contains(&entry.path()) {
                fs::remove_file(entry.path()).map_err(|e| e.to_string())?; removed += 1;
            }
        }
        Ok(removed)
    }
    #[cfg(test)]
    pub(crate) fn fail_put_after(&self, count: Option<usize>) {
        self.stats.fail_after.store(count.map_or(-1, |n| n as isize), std::sync::atomic::Ordering::Relaxed);
    }
    #[cfg(test)]
    pub(crate) fn counts(&self) -> (usize, usize, usize) {
        use std::sync::atomic::Ordering::Relaxed;
        (self.stats.reads.load(Relaxed), self.stats.puts.load(Relaxed), self.stats.scans.load(Relaxed))
    }
}
fn lowercase_hex(value: &str) -> bool { value.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c)) }
fn hex(bytes: &[u8]) -> String { bytes.iter().map(|b| format!("{b:02x}")).collect() }

impl ReadOnlyRepository {
    fn object_path(&self, object: ObjectIdentity) -> PathBuf {
        let name = match object {
            ObjectIdentity::Assigned(id) => format!("id-{}", hex(&id.0)),
        };
        self.root.join(name)
    }
    fn open_object(&self, object: ObjectIdentity, max_bytes: usize) -> Result<(File, usize), String> {
        object.validate()?;
        #[cfg(test)] self.stats.reads.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let mut options = OpenOptions::new(); options.read(true);
        // Inspect the actual opened descriptor. Neither symlinks nor FIFOs may redirect/block IO.
        #[cfg(unix)] options.custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
        let file = options.open(self.object_path(object)).map_err(|e| e.to_string())?;
        let meta = file.metadata().map_err(|e| e.to_string())?;
        if !meta.is_file() || meta.len() > max_bytes as u64 { return Err("invalid object type or size".into()); }
        Ok((file, meta.len() as usize))
    }
    pub fn read_object(&self, object: ObjectIdentity, max_bytes: usize) -> Result<Vec<u8>, String> {
        let (file, total) = self.open_object(object, max_bytes)?;
        let mut bytes = Vec::with_capacity(total);
        file.take(max_bytes as u64 + 1).read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        if bytes.len() != total || bytes.len() > max_bytes { return Err("object length changed or exceeds budget".into()); }
        Ok(bytes)
    }
}

/// 写入器与同进程读者共用一份有界解码缓存；独立仓库读者保留自己的预算。
#[derive(Clone)]
pub struct RegionReader {
    repository: ReadOnlyRepository,
    cache: Arc<std::sync::Mutex<DecodedCache>>,
}
struct DecodedCache {
    cache: BTreeMap<ObjectIdentity, (Arc<Block>, Intrinsic, u64, usize, GroundRecords)>,
    budget_bytes: usize, used_bytes: usize, clock: u64,
}
impl RegionReader {
    pub fn new(repository: impl Into<ReadOnlyRepository>, budget_bytes: usize) -> Self {
        Self { repository: repository.into(), cache: Arc::new(std::sync::Mutex::new(DecodedCache {
            cache: BTreeMap::new(), budget_bytes, used_bytes: 0, clock: 0,
        })) }
    }
    pub(crate) fn set_repository(&mut self, repository: ReadOnlyRepository) {
        self.repository = repository;
    }
    pub fn ground_records(&mut self, reference: &BlockRef) -> Result<GroundRecords, String> {
        self.parts(reference.object, reference.points).map(|(_, _, records)| records)
    }
    pub fn ground_parts(&mut self, reference: &BlockRef) -> Result<(Arc<Block>, GroundRecords), String> {
        self.parts(reference.object, reference.points).map(|(block, _, records)| (block, records))
    }
    #[cfg(test)]
    pub(crate) fn read_count(&self) -> usize {
        self.repository.stats.reads.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn cached_bytes(&self) -> usize { self.cache.lock().unwrap().used_bytes }
    #[cfg(test)]
    pub(crate) fn cache_is_empty(&self) -> bool { self.cache.lock().unwrap().cache.is_empty() }
    pub(crate) fn retain_objects(&mut self, live: &BTreeSet<ObjectIdentity>) {
        let mut cache = self.cache.lock().unwrap();
        cache.cache.retain(|id, _| live.contains(id));
        cache.used_bytes = cache.cache.values().map(|entry| entry.3).sum();
    }
    pub fn block(&mut self, reference: &BlockRef) -> Result<Arc<Block>, String> {
        self.object(reference.object, reference.points)
    }
    pub(crate) fn object(&mut self, object: ObjectIdentity, points: u32) -> Result<Arc<Block>, String> {
        self.parts(object, points).map(|(block, _, _)| block)
    }
    pub(crate) fn parts(&mut self, object: ObjectIdentity, points: u32) -> Result<(Arc<Block>, Intrinsic, GroundRecords), String> {
        self.cache.lock().unwrap().parts(&self.repository, object, points)
    }
    pub(crate) fn cache_decoded(&mut self, object: ObjectIdentity, block: Arc<Block>, info: Intrinsic, layers: GroundRecords) {
        let layer_bytes = layers.bytes();
        self.cache_written(object, block, info, layers, layer_bytes);
    }
    pub(crate) fn cache_written(&mut self, object: ObjectIdentity, block: Arc<Block>, info: Intrinsic, layers: GroundRecords, layer_bytes: usize) {
        self.cache.lock().unwrap().cache_decoded(object, block, info, layers, layer_bytes);
    }
    pub fn region(&mut self, root: &Manifest, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        root.validate()?;
        if (0..3).any(|i| min[i] > max[i]) { return Err("reversed region".into()); }
        let low = block_key(min); let high = block_key(max); let mut out = Vec::new();
        let start = root.blocks.partition_point(|b| b.key[0] < low[0]);
        for reference in root.blocks[start..].iter().take_while(|b| b.key[0] <= high[0]) {
            if (1..3).any(|i| reference.key[i] < low[i] || reference.key[i] > high[i]) { continue; }
            for (&local, &evidence) in self.block(reference)?.iter() {
                let key = global_key(reference.key, local)?;
                if (0..3).any(|i| key[i] < min[i] || key[i] > max[i]) { continue; }
                if evidence.observed_ns > root.source_timestamp_ns { return Err("evidence exceeds root bound".into()); }
                if out.len() >= point_budget { return Err("region exceeds point budget".into()); }
                out.push((key, evidence));
            }
        }
        Ok(out)
    }
}

impl DecodedCache {
    /// One decode even when a large block cannot stay in the bounded cache.
    fn parts(&mut self, repository: &ReadOnlyRepository, object: ObjectIdentity, points: u32) -> Result<(Arc<Block>, Intrinsic, GroundRecords), String> {
        self.clock = self.clock.wrapping_add(1);
        if let Some((block, info, age, _, layers)) = self.cache.get_mut(&object) {
            if info.points != points { return Err("cached object point count conflicts with root".into()); }
            *age = self.clock; return Ok((block.clone(), *info, layers.clone()));
        }
        let bytes = repository.read_object(object, MAX_BLOCK_BYTES)?;
        let (block, layers) = decode_block_parts(&bytes)?;
        let block = Arc::new(block);
        let info = intrinsic_bytes(&bytes, &block);
        if info.points != points { return Err("object point count conflicts with root".into()); }
        self.cache_decoded(object, block.clone(), info, layers.clone(), layers.bytes());
        Ok((block, info, layers))
    }
    fn cache_decoded(&mut self, object: ObjectIdentity, block: Arc<Block>, info: Intrinsic, layers: GroundRecords, layer_bytes: usize) {
        if self.cache.contains_key(&object) { return; }
        let bytes = block.len() * 64 + 128 + layer_bytes;
        if bytes <= self.budget_bytes {
            while self.used_bytes + bytes > self.budget_bytes {
                let Some(key) = self.cache.iter().min_by_key(|(_, (_, _, age, _, _))| *age).map(|(key, _)| *key) else { break; };
                self.used_bytes -= self.cache.remove(&key).unwrap().3;
            }
            self.cache.insert(object, (block, info, self.clock, bytes, layers)); self.used_bytes += bytes;
        }
    }
}
