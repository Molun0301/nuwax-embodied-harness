use super::*;

/// 只有当前脏块驻留；提交后写入内容对象并释放块体，远场只保留清单。
pub struct Writer {
    repository: Repository,
    blocks: BTreeMap<Key, BlockRef>,
    dirty: BTreeMap<Key, Block>,
    reader: RegionReader,
    voxel_size_m: f32,
    dirty_point_budget: usize,
    dirty_points: usize,
    max_observed_ns: u64,
}
impl Writer {
    pub fn new(repository: Repository, voxel_size_m: f32, cache_bytes: usize) -> Result<Self, String> {
        if !voxel_size_m.is_finite() || voxel_size_m <= 0.0 || voxel_size_m > 1_048_576.0 || cache_bytes < 4 * 1024 * 1024 {
            return Err("分块写入精度或缓存预算无效".into());
        }
        Ok(Self {
            reader: RegionReader::new(repository.clone(), cache_bytes / 2), repository,
            blocks: BTreeMap::new(), dirty: BTreeMap::new(), voxel_size_m,
            dirty_point_budget: cache_bytes / 128, dirty_points: 0, max_observed_ns: 0,
        })
    }
    pub fn repository(&self) -> &Repository { &self.repository }
    pub fn voxel_size_m(&self) -> f32 { self.voxel_size_m }
    pub fn resident_bytes(&self) -> usize { self.dirty_points * 64 + self.reader.cached_bytes() }
    pub fn block_count(&self) -> usize { self.blocks.len() + self.dirty.keys().filter(|key| !self.blocks.contains_key(*key)).count() }
    pub fn restore(&mut self, root: &Manifest) -> Result<(), String> {
        root.validate()?;
        if root.voxel_size_m != self.voxel_size_m { return Err("恢复精细根的实际精度不符".into()); }
        // 逐块核对，过程不改变已安装写入状态。
        for reference in &root.blocks {
            let block = self.reader.block(reference)?;
            if block.values().any(|e| e.observed_ns > root.source_timestamp_ns) { return Err("恢复根包含未来观察".into()); }
        }
        self.blocks = root.blocks.iter().map(|b| (b.key, b.clone())).collect();
        self.dirty.clear(); self.dirty_points = 0;
        self.max_observed_ns = root.source_timestamp_ns;
        Ok(())
    }
    fn load_dirty(&mut self, key: Key) -> Result<&mut Block, String> {
        if !self.dirty.contains_key(&key) {
            if self.dirty_points + 32768 > self.dirty_point_budget { self.flush()?; }
            let block = match self.blocks.get(&key) {
                Some(reference) => (*self.reader.block(reference)?).clone(),
                None => BTreeMap::new(),
            };
            self.dirty_points += block.len();
            self.dirty.insert(key, block);
        }
        Ok(self.dirty.get_mut(&key).unwrap())
    }
    /// 同一次扫描先分组去重；同一时间的重复传送不增加命中质量。
    pub fn observe(&mut self, keys: impl IntoIterator<Item = Key>, timestamp_ns: u64) -> Result<(), String> {
        if timestamp_ns == 0 { return Err("精细观察必须带实际来源时间".into()); }
        self.update(keys, timestamp_ns)
    }
    /// 旧资产只补入几何；保留没有实际观察时间的事实。
    pub fn seed_unknown(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> {
        self.update(keys, 0)
    }
    fn update(&mut self, keys: impl IntoIterator<Item = Key>, timestamp_ns: u64) -> Result<(), String> {
        let mut grouped: BTreeMap<Key, BTreeSet<u16>> = BTreeMap::new();
        for key in keys {
            grouped.entry(block_key(key)).or_default().insert(local_key(key));
        }
        let added_blocks = grouped.keys().filter(|key| !self.blocks.contains_key(*key) && !self.dirty.contains_key(*key)).count();
        if self.block_count() + added_blocks > MAX_BLOCKS { return Err("精细地图块目录超预算".into()); }
        for (key, hits) in grouped {
            let block = self.load_dirty(key)?;
            let before = block.len();
            for local in hits {
                let old = block.entry(local).or_insert(Evidence { observed_ns: timestamp_ns, hits: u16::from(timestamp_ns > 0) });
                if timestamp_ns > old.observed_ns {
                    old.hits = old.hits.saturating_add(1);
                    old.observed_ns = timestamp_ns;
                }
            }
            self.dirty_points += self.dirty[&key].len() - before;
        }
        self.max_observed_ns = self.max_observed_ns.max(timestamp_ns);
        Ok(())
    }
    /// 撤销必须来自原建图拥有方的真实空闲证据，不能按年龄删除静态结构。
    pub fn remove(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> {
        let mut grouped: BTreeMap<Key, Vec<u16>> = BTreeMap::new();
        for key in keys { grouped.entry(block_key(key)).or_default().push(local_key(key)); }
        for (key, removed) in grouped {
            if !self.blocks.contains_key(&key) && !self.dirty.contains_key(&key) { continue; }
            let block = self.load_dirty(key)?;
            let before = block.len();
            for local in removed { block.remove(&local); }
            self.dirty_points -= before - self.dirty[&key].len();
        }
        Ok(())
    }
    pub fn flush(&mut self) -> Result<(), String> {
        // 对象写入失败时脏块仍在内存，已写对象尚未被任何根公开。
        let mut refs = Vec::with_capacity(self.dirty.len());
        for (&key, block) in &self.dirty {
            if block.is_empty() { refs.push((key, None)); continue; }
            let bytes = encode_block(block)?;
            let sha256 = digest(&bytes);
            self.repository.put_object(sha256, &bytes)?;
            refs.push((key, Some(BlockRef { key, sha256, geometry_sha256: block_geometry_digest(block), points: block.len() as u32 })));
        }
        for (key, reference) in refs {
            match reference {
                Some(reference) => { self.blocks.insert(key, reference); }
                None => { self.blocks.remove(&key); }
            }
        }
        self.dirty.clear(); self.dirty_points = 0;
        Ok(())
    }
    pub fn snapshot(&mut self, domain: Domain, revision: u64, timestamp_ns: u64) -> Result<Arc<Manifest>, String> {
        let sealed = self.seal(timestamp_ns)?;
        let root = sealed.manifest(domain, revision)?;
        let bytes = root.encode()?;
        self.repository.put_object(digest(&bytes), &bytes)?;
        Ok(Arc::new(root))
    }
    pub fn seal(&mut self, timestamp_ns: u64) -> Result<SealedMap, String> {
        if timestamp_ns == 0 { return Err("封存精细块必须带真实提交时间".into()); }
        if self.max_observed_ns > timestamp_ns { return Err("精细根不能封存尚未提交的后续扫描".into()); }
        self.flush()?;
        Ok(SealedMap { voxel_size_m: self.voxel_size_m, source_timestamp_ns: timestamp_ns,
            blocks: Arc::new(self.blocks.values().cloned().collect()), repository: self.repository.clone() })
    }
    /// 对当前累积状态做局部动态反证查询；不公开未提交根。
    pub fn region(&mut self, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        if (0..3).any(|i| min[i] > max[i]) { return Err("局部范围倒置".into()); }
        let low = block_key(min); let high = block_key(max);
        let keys = self.blocks.keys().chain(self.dirty.keys()).copied().filter(|key|
            (0..3).all(|i| key[i] >= low[i] && key[i] <= high[i])).collect::<BTreeSet<_>>();
        let mut out = Vec::new();
        for key in keys {
            let block = if let Some(block) = self.dirty.get(&key) { Arc::new(block.clone()) }
                else { self.reader.block(&self.blocks[&key])? };
            for (&local, &evidence) in block.iter() {
                let key = global_key(key, local)?;
                if (0..3).all(|i| key[i] >= min[i] && key[i] <= max[i]) {
                    if out.len() >= point_budget { return Err("精细累积局部查询超预算".into()); }
                    out.push((key, evidence));
                }
            }
        }
        Ok(out)
    }
}
