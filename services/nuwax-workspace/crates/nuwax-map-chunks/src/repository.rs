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
    fn object_path(&self, object: ObjectIdentity) -> PathBuf {
        let name = match object {
            ObjectIdentity::Assigned(id) => format!("id-{}", hex(&id.0)),
            ObjectIdentity::Legacy(alias) => hex(&alias.0),
        };
        self.root.join(name)
    }
    pub(crate) fn object_exists(&self, object: ObjectIdentity) -> Result<bool, String> {
        match fs::symlink_metadata(self.object_path(object)) {
            Ok(_) => Ok(true), Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(false), Err(e) => Err(e.to_string()),
        }
    }
    pub fn read_object(&self, object: ObjectIdentity, max_bytes: usize) -> Result<Vec<u8>, String> {
        object.validate()?;
        #[cfg(test)] self.stats.reads.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let mut options = OpenOptions::new(); options.read(true);
        // Inspect the actual opened descriptor. Neither symlinks nor FIFOs may redirect/block IO.
        #[cfg(unix)] options.custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
        let file = options.open(self.object_path(object)).map_err(|e| e.to_string())?;
        let meta = file.metadata().map_err(|e| e.to_string())?;
        if !meta.is_file() || meta.len() > max_bytes as u64 { return Err("invalid object type or size".into()); }
        let mut bytes = Vec::with_capacity(meta.len() as usize);
        file.take(max_bytes as u64 + 1).read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        if bytes.len() != meta.len() as usize || bytes.len() > max_bytes { return Err("object length changed or exceeds budget".into()); }
        Ok(bytes)
    }
    /// Publish once using a no-replace hard link. Existing IDs must have exact equal bytes.
    pub fn put_object(&self, object: ObjectIdentity, bytes: &[u8]) -> Result<(), String> {
        object.validate()?;
        if bytes.len() > MAX_MANIFEST_BYTES { return Err("object exceeds repository budget".into()); }
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
    pub fn install_legacy_root(&self, root: &LegacyRoot) -> Result<(), String> {
        let admitted = LegacyRoot::decode(root.raw_json.clone(), root.alias)?;
        if admitted.manifest != root.manifest { return Err("legacy root bytes differ from supplied metadata".into()); }
        self.put_object(ObjectIdentity::Legacy(root.alias), &root.raw_json)
    }
    pub fn copy_snapshot(&self, root: &Manifest, destination: &Self) -> Result<(), String> {
        root.validate()?; let mut copied = BTreeSet::new();
        for block in &root.blocks {
            if !copied.insert(block.object) { continue; }
            let bytes = self.read_object(block.object, MAX_BLOCK_BYTES)?;
            admit_intrinsic(intrinsic(&decode_block(&bytes)?), block.points, root.source_timestamp_ns)?;
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
            let legacy = name.len() == 64 && lowercase_hex(name);
            if (typed || legacy) && !live.contains(&entry.path()) {
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

/// The cache stores only intrinsic metadata, so aliases can be shared across keys/owners.
pub struct RegionReader {
    repository: Repository,
    cache: BTreeMap<ObjectIdentity, (Arc<Block>, Intrinsic, u64, usize)>,
    budget_bytes: usize, used_bytes: usize, clock: u64,
}
impl RegionReader {
    pub fn new(repository: Repository, budget_bytes: usize) -> Self {
        Self { repository, cache: BTreeMap::new(), budget_bytes, used_bytes: 0, clock: 0 }
    }
    pub fn cached_bytes(&self) -> usize { self.used_bytes }
    #[cfg(test)]
    pub(crate) fn cache_is_empty(&self) -> bool { self.cache.is_empty() }
    pub(crate) fn retain_objects(&mut self, live: &BTreeSet<ObjectIdentity>) {
        self.cache.retain(|id, _| live.contains(id));
        self.used_bytes = self.cache.values().map(|entry| entry.3).sum();
    }
    pub fn block(&mut self, reference: &BlockRef) -> Result<Arc<Block>, String> {
        self.object(reference.object, reference.points)
    }
    pub(crate) fn object(&mut self, object: ObjectIdentity, points: u32) -> Result<Arc<Block>, String> {
        self.clock = self.clock.wrapping_add(1);
        if let Some((block, info, age, _)) = self.cache.get_mut(&object) {
            if info.points != points { return Err("cached object point count conflicts with root".into()); }
            *age = self.clock; return Ok(block.clone());
        }
        let block = Arc::new(decode_block(&self.repository.read_object(object, MAX_BLOCK_BYTES)?)?);
        let info = intrinsic(&block);
        if info.points != points { return Err("object point count conflicts with root".into()); }
        self.cache_decoded(object, block.clone(), info);
        Ok(block)
    }
    pub(crate) fn cache_decoded(&mut self, object: ObjectIdentity, block: Arc<Block>, info: Intrinsic) {
        if self.cache.contains_key(&object) { return; }
        let bytes = block.len() * 64 + 128;
        if bytes <= self.budget_bytes {
            while self.used_bytes + bytes > self.budget_bytes {
                let Some(key) = self.cache.iter().min_by_key(|(_, (_, _, age, _))| *age).map(|(key, _)| *key) else { break; };
                self.used_bytes -= self.cache.remove(&key).unwrap().3;
            }
            self.cache.insert(object, (block, info, self.clock, bytes)); self.used_bytes += bytes;
        }
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
