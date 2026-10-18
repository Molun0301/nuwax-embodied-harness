use super::*;

struct Dirty { block: Block, layers: LayerObservations, changed: bool }
use terrain_observations::layer_bytes;
fn watermark_bytes(source: &LayerSource) -> usize { source.resident_bytes() + 80 }
struct Pending { object: ObjectIdentity, bytes: Vec<u8>, installed: bool }
/// Dirty payload is bounded; membership baselines retain references, never copied member lists.
pub struct Writer {
    repository: Repository, blocks: BTreeMap<Key, BlockRef>, dirty: BTreeMap<Key, Dirty>,
    pending: BTreeMap<Key, Pending>, touched: BTreeSet<Key>, membership_touched: BTreeSet<Key>,
    source_watermarks: BTreeMap<Arc<LayerSource>, (u64, u64)>,
    source_watermark_bytes: usize, source_watermarks_changed: bool,
    published: Option<Arc<Manifest>>, legacy_baseline: Option<(Domain, u64)>, reader: RegionReader,
    id_source: Box<dyn FnMut() -> Result<ObjectId, String> + Send>,
    #[cfg(test)] pub(crate) encoded_blocks: usize,
    voxel_size_m: f32, dirty_point_budget: usize, dirty_points: usize, dirty_layer_bytes: usize, max_observed_ns: u64,
}
impl Writer {
    pub fn new(repository: Repository, voxel_size_m: f32, cache_bytes: usize) -> Result<Self, String> {
        Self::with_id_source(repository, voxel_size_m, cache_bytes, ObjectId::random)
    }
    pub fn with_id_source(repository: Repository, voxel_size_m: f32, cache_bytes: usize,
        id_source: impl FnMut() -> Result<ObjectId, String> + Send + 'static) -> Result<Self, String> {
        if !voxel_size_m.is_finite() || voxel_size_m <= 0.0 || voxel_size_m > 1_048_576.0 || cache_bytes < 4 * 1024 * 1024 {
            return Err("invalid writer resolution or cache budget".into());
        }
        Ok(Self { reader: RegionReader::new(repository.clone(), cache_bytes / 2), repository,
            blocks: BTreeMap::new(), dirty: BTreeMap::new(), pending: BTreeMap::new(), touched: BTreeSet::new(), membership_touched: BTreeSet::new(),
            source_watermarks: BTreeMap::new(), source_watermark_bytes: 0, source_watermarks_changed: false, published: None, legacy_baseline: None, id_source: Box::new(id_source), #[cfg(test)] encoded_blocks: 0, voxel_size_m, dirty_point_budget: cache_bytes / 128, dirty_points: 0, dirty_layer_bytes: 0, max_observed_ns: 0 })
    }
    pub fn repository(&self) -> &Repository { &self.repository }
    /// Last captured original (timestamp, sequence) in this exact namespace.
    pub fn source_watermark(&self, source: &LayerSource) -> Option<(u64, u64)> {
        self.source_watermarks.get(source).copied()
    }
    /// Advance only this source's lexicographic frame marker. Timestamp advance
    /// permits an original lower sequence; different clocks are never compared.
    pub fn record_source_watermark(&mut self, source: Arc<LayerSource>, timestamp: u64, sequence: u64) -> Result<(), String> {
        SourceWatermark { source: source.clone(), source_timestamp_ns: timestamp, source_sequence: sequence }.validate()?;
        let next = (timestamp, sequence);
        if let Some(previous) = self.source_watermarks.get(source.as_ref()) {
            if next < *previous { return Err("source watermark rolled back".into()); }
            if next == *previous { return Ok(()); }
        } else {
            let bytes = watermark_bytes(&source);
            if bytes > MAX_MANIFEST_BYTES.saturating_sub(self.source_watermark_bytes) {
                return Err("source watermarks exceed resident budget".into());
            }
            self.source_watermark_bytes += bytes;
        }
        self.source_watermarks.insert(source, next);
        self.source_watermarks_changed = true;
        Ok(())
    }
    pub fn has_unpublished_changes(&self) -> bool {
        self.source_watermarks_changed || !self.touched.is_empty() || !self.membership_touched.is_empty() || !self.pending.is_empty()
    }
    /// Move only the published baseline into the next frame. Dirty evidence,
    /// pressure-flushed references and partially installed frozen IDs stay pending.
    pub fn rehome_published_frame(&mut self, expected: &RootIdentity, new_frame: u64) -> Result<SealedMap, String> {
        let previous = self.published.clone().ok_or("frame rehome requires a published root")?;
        let published = SealedMap { root: previous.clone(), repository: self.repository.clone() };
        // All fallible validation precedes changes to either baseline or working state.
        let rehomed = published.rehome_frame(expected, new_frame)?;
        for reference in &previous.blocks {
            if let Some(working) = self.blocks.get_mut(&reference.key) {
                if working.object == reference.object {
                    working.block_revision = rehomed.root.revision;
                    working.geometry_revision = rehomed.root.revision;
                }
            }
        }
        // The reader cache is keyed by immutable object identity, not the frame.
        // Keep it and all unpublished state, including the future evidence bound.
        self.published = Some(rehomed.root.clone());
        Ok(rehomed)
    }
    pub fn voxel_size_m(&self) -> f32 { self.voxel_size_m }
    pub fn resident_bytes(&self) -> usize { self.source_watermark_bytes + self.dirty_layer_bytes() + self.dirty_points * 64 + self.reader.cached_bytes() + self.pending.values().map(|p| p.bytes.len()).sum::<usize>() }
    pub fn block_count(&self) -> usize {
        self.blocks.len()
            + self.dirty.iter().filter(|(key, dirty)| !dirty.block.is_empty() && !self.blocks.contains_key(*key)).count()
            - self.dirty.iter().filter(|(key, dirty)| dirty.block.is_empty() && self.blocks.contains_key(*key)).count()
    }
    pub fn restore(&mut self, root: &Manifest) -> Result<(), String> {
        root.validate()?;
        if root.voxel_size_m != self.voxel_size_m { return Err("restored resolution differs from writer".into()); }
        let mut latest = 0;
        for reference in &root.blocks {
            let (_, info, _) = self.reader.parts(reference.object, reference.points)?;
            admit_format(info, root)?;
            admit_intrinsic(info, reference.points, root.source_timestamp_ns)?; latest = latest.max(info.latest);
        }
        let source_watermarks: BTreeMap<_, _> = root.source_watermarks.iter().map(|mark|
            (mark.source.clone(), (mark.source_timestamp_ns, mark.source_sequence))).collect();
        let source_watermark_bytes = source_watermarks.keys().map(|source| watermark_bytes(source)).sum::<usize>();
        if source_watermark_bytes > MAX_MANIFEST_BYTES { return Err("source watermarks exceed resident budget".into()); }
        self.source_watermarks = source_watermarks; self.source_watermark_bytes = source_watermark_bytes;
        self.blocks = root.blocks.iter().map(|b| (b.key, b.clone())).collect();
        self.reset_pending(); self.max_observed_ns = latest;
        self.published = Some(Arc::new(root.clone())); self.legacy_baseline = None; Ok(())
    }
    pub fn restore_legacy(&mut self, root: &LegacyManifest) -> Result<(), String> {
        root.validate()?;
        if root.voxel_size_m != self.voxel_size_m { return Err("restored resolution differs from writer".into()); }
        let mut latest = 0;
        for reference in &root.blocks {
            let object = ObjectIdentity::Legacy(reference.sha256);
            let (_, info, layers) = self.reader.parts(object, reference.points)?;
            if !layers.is_empty() { return Err("legacy root cannot contain observed layers".into()); }
            admit_intrinsic(info, reference.points, root.source_timestamp_ns)?; latest = latest.max(info.latest);
        }
        self.source_watermarks.clear(); self.source_watermark_bytes = 0;
        self.blocks = root.blocks.iter().map(|b| (b.key, BlockRef { key: b.key, object: ObjectIdentity::Legacy(b.sha256), block_revision: 0, geometry_revision: 0, points: b.points })).collect();
        self.reset_pending(); self.max_observed_ns = latest; self.published = None;
        self.legacy_baseline = Some((root.domain.clone(), root.revision)); Ok(())
    }
    pub fn restore_archived(&mut self, root: &ArchivedRoot) -> Result<(), String> {
        match root { ArchivedRoot::Current(root) => self.restore(root), ArchivedRoot::Legacy(root) => self.restore_legacy(&root.manifest) }
    }
    fn reset_pending(&mut self) {
        self.dirty.clear(); self.pending.clear(); self.touched.clear(); self.membership_touched.clear(); self.dirty_points = 0; self.dirty_layer_bytes = 0; self.source_watermarks_changed = false;
    }
    fn load_dirty(&mut self, key: Key) -> Result<&mut Dirty, String> {
        if !self.dirty.contains_key(&key) {
            if self.dirty_points + 32768 + self.dirty_layer_bytes() / 64 > self.dirty_point_budget { self.flush()?; }
            let (block, layers) = match self.blocks.get(&key) {
                Some(reference) => {
                    let (block, _, layers) = self.reader.parts(reference.object, reference.points)?;
                    ((*block).clone(), (*layers).clone())
                },
                None => (Block::new(), LayerObservations::new()),
            };
            self.dirty_layer_bytes += layers.values().map(layer_bytes).sum::<usize>();
            self.dirty_points += block.len(); self.dirty.insert(key, Dirty { block, layers, changed: false });
        }
        Ok(self.dirty.get_mut(&key).unwrap())
    }
    fn dirty_layer_bytes(&self) -> usize {
        self.dirty_layer_bytes
    }
    /// Read only one touched block; capture keeps no second whole-map statistics cache.
    pub fn layer_observations(&mut self, key: Key) -> Result<LayerObservations, String> {
        if let Some(dirty) = self.dirty.get(&key) { return Ok(dirty.layers.clone()); }
        match self.blocks.get(&key) { Some(reference) => Ok((*self.reader.layers(reference)?).clone()), None => Ok(LayerObservations::new()) }
    }
    /// Replace a touched block's associated layers after original point admission.
    /// Occupancy must already contain the real member hits. On error no layers change.
    pub fn replace_layer_observations(&mut self, key: Key, layers: LayerObservations) -> Result<(), String> {
        let dirty = self.load_dirty(key)?;
        terrain_observations::validate_layers(&layers, &dirty.block)?;
        if dirty.layers == layers { return Ok(()); }
        let before = dirty.layers.values().map(layer_bytes).sum::<usize>();
        let after = layers.values().map(layer_bytes).sum::<usize>();
        dirty.layers = layers; dirty.changed = true;
        self.dirty_layer_bytes = self.dirty_layer_bytes - before + after;
        self.pending.remove(&key); self.touched.insert(key); self.membership_touched.insert(key);
        Ok(())
    }
    pub fn observe(&mut self, keys: impl IntoIterator<Item = Key>, timestamp_ns: u64) -> Result<(), String> {
        if timestamp_ns == 0 { return Err("observation requires a source timestamp".into()); } self.update(keys, timestamp_ns)
    }
    pub fn seed_unknown(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> { self.update(keys, 0) }
    fn update(&mut self, keys: impl IntoIterator<Item = Key>, timestamp_ns: u64) -> Result<(), String> {
        let mut grouped: BTreeMap<Key, BTreeSet<u16>> = BTreeMap::new();
        for key in keys { grouped.entry(block_key(key)).or_default().insert(local_key(key)); }
        let added = grouped.keys().filter(|key| !self.blocks.contains_key(*key) && !self.dirty.contains_key(*key)).count();
        if self.block_count() + added > MAX_BLOCKS { return Err("block directory exceeds budget".into()); }
        for (key, hits) in grouped {
            let dirty = self.load_dirty(key)?; let before = dirty.block.len(); let mut changed = false;
            for local in hits {
                match dirty.block.entry(local) {
                    std::collections::btree_map::Entry::Vacant(entry) => {
                        entry.insert(Evidence { observed_ns: timestamp_ns, hits: u16::from(timestamp_ns > 0) }); changed = true;
                    }
                    std::collections::btree_map::Entry::Occupied(mut entry) => {
                        let old = entry.get_mut();
                        if timestamp_ns > old.observed_ns { old.observed_ns = timestamp_ns; old.hits = old.hits.saturating_add(1); changed = true; }
                    }
                }
            }
            let added = dirty.block.len() - before; dirty.changed |= changed;
            self.dirty_points += added;
            if changed {
                // Once frozen bytes change, even after a partial flush, never reuse their ID.
                self.pending.remove(&key); self.touched.insert(key); self.max_observed_ns = self.max_observed_ns.max(timestamp_ns);
            }
            if added > 0 { self.membership_touched.insert(key); }
        }
        Ok(())
    }
    pub fn remove(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> {
        let quantum=f64::from(self.voxel_size_m);
        let mut grouped: BTreeMap<Key, BTreeSet<u16>> = BTreeMap::new();
        for key in keys { grouped.entry(block_key(key)).or_default().insert(local_key(key)); }
        for (key, removed) in grouped {
            if !self.blocks.contains_key(&key) && !self.dirty.contains_key(&key) { continue; }
            let dirty = self.load_dirty(key)?; let before = dirty.block.len();
            for local in &removed { dirty.block.remove(local); }
            let before_layer_bytes=dirty.layers.values().map(layer_bytes).sum::<usize>();
            dirty.layers=std::mem::take(&mut dirty.layers).into_values().filter_map(|mut layer| {
                let previous_members=layer.members.len();
                layer.members.retain(|member|dirty.block.contains_key(member));
                if layer.members.is_empty() {return None;}
                if layer.members.len()!=previous_members {
                    // 删除只撤销被射线证实消失的成员及其原样本；剩余真实成员
                    // 继续使用捕获时已拟合的平面，不能退回无地面身份的占据。
                    layer.samples.retain(|sample| {
                        let sample_key=sample.point.map(|value|(value/quantum).floor() as i32);
                        block_key(sample_key)!=key || !removed.contains(&local_key(sample_key))
                    });
                }
                (!layer.samples.is_empty()).then(||(layer.members[0],layer))
            }).collect();
            let removed_layer_bytes=before_layer_bytes-dirty.layers.values().map(layer_bytes).sum::<usize>();
            let removed = before - dirty.block.len(); dirty.changed |= removed > 0; self.dirty_points -= removed; self.dirty_layer_bytes -= removed_layer_bytes;
            if removed > 0 { self.pending.remove(&key); self.touched.insert(key); self.membership_touched.insert(key); }
        }
        Ok(())
    }
    pub fn flush(&mut self) -> Result<(), String> {
        let keys: Vec<_> = self.dirty.iter().filter(|(_, d)| d.changed && !d.block.is_empty()).map(|(k, _)| *k).collect();
        for key in keys {
            if !self.pending.contains_key(&key) {
                let object = ObjectIdentity::Assigned((self.id_source)()?); object.validate()?;
                let bytes = encode_layer_block(&self.dirty[&key].block, &self.dirty[&key].layers)?;
                #[cfg(test)] { self.encoded_blocks += 1; }
                self.pending.insert(key, Pending { object, bytes, installed: false });
            }
            let pending = self.pending.get_mut(&key).unwrap();
            if !pending.installed {
                self.repository.put_object(pending.object, &pending.bytes)?;
                pending.installed = true;
            }
        }
        // Do not change current references until every frozen payload is installed.
        for (key, dirty) in &self.dirty {
            if !dirty.changed { continue; }
            if dirty.block.is_empty() { self.blocks.remove(key); continue; }
            let previous = self.blocks.get(key);
            let reference = BlockRef { key: *key, object: self.pending[key].object,
                block_revision: previous.map_or(0, |b| b.block_revision), geometry_revision: previous.map_or(0, |b| b.geometry_revision), points: dirty.block.len() as u32 };
            self.blocks.insert(*key, reference);
        }
        self.dirty.clear(); self.pending.clear(); self.dirty_points = 0; self.dirty_layer_bytes = 0; Ok(())
    }
    pub fn snapshot(&mut self, domain: Domain, revision: u64, timestamp_ns: u64) -> Result<Arc<Manifest>, String> {
        Ok(self.seal(domain, revision, timestamp_ns)?.root)
    }
    pub fn seal(&mut self, domain: Domain, revision: u64, timestamp_ns: u64) -> Result<SealedMap, String> {
        validate_header(&domain, revision, timestamp_ns, self.voxel_size_m, self.block_count())?;
        if self.max_observed_ns > timestamp_ns { return Err("root bound precedes observed evidence".into()); }
        let same_domain = self.published.as_ref().is_some_and(|r| r.domain == domain);
        if self.published.as_ref().is_some_and(|r| r.domain == domain && revision <= r.revision)
            || self.legacy_baseline.as_ref().is_some_and(|(d, r)| *d == domain && revision <= *r) {
            return Err("publication revision did not advance".into());
        }
        self.flush()?;
        let mut changed_members = BTreeSet::new();
        if same_domain {
            let old = self.published.as_ref().unwrap();
            for key in &self.membership_touched {
                let previous = old.blocks.binary_search_by_key(key, |b| b.key).ok().map(|i| &old.blocks[i]);
                match (previous, self.blocks.get(key)) {
                    (None, None) => {},
                    (Some(a), Some(b)) if a.object == b.object => {},
                    (Some(a), Some(b)) => {
                        let a = self.reader.block(a)?; let b = self.reader.block(b)?;
                        if a.keys().ne(b.keys()) || !layer_geometry_eq(&*self.reader.layers(previous.unwrap())?, &*self.reader.layers(&self.blocks[key])?) { changed_members.insert(*key); }
                    }
                    _ => { changed_members.insert(*key); }
                }
            }
        }
        let geometry_revision = if !same_domain || !changed_members.is_empty() { revision } else { self.published.as_ref().unwrap().geometry_revision };
        let mut blocks = self.blocks.clone();
        let reuse_ordered_baseline = same_domain && changed_members.is_empty();
        let mut previous_index = 0_usize;
        for (key, block) in &mut blocks {
            if !same_domain { block.block_revision = revision; block.geometry_revision = revision; }
            else {
                if self.touched.contains(key) { block.block_revision = revision; }
                if changed_members.contains(key) { block.geometry_revision = revision; }
                else {
                    let previous = self.published.as_ref().unwrap();
                    if reuse_ordered_baseline {
                        block.geometry_revision = previous.blocks[previous_index].geometry_revision;
                        previous_index += 1;
                    } else if let Ok(index) = previous.blocks.binary_search_by_key(key, |b| b.key) {
                        block.geometry_revision = previous.blocks[index].geometry_revision;
                    }
                }
            }
        }
        let root = Arc::new(Manifest { version: 4, domain, revision, geometry_revision, source_timestamp_ns: timestamp_ns,
            voxel_size_m: self.voxel_size_m, blocks: blocks.values().cloned().collect(),
            source_watermarks: self.source_watermarks.iter().map(|(source, &(source_timestamp_ns, source_sequence))|
                SourceWatermark { source: source.clone(), source_timestamp_ns, source_sequence }).collect() });
        root.validate()?;
        self.blocks = blocks; self.published = Some(root.clone()); self.legacy_baseline = None;
        self.touched.clear(); self.membership_touched.clear(); self.source_watermarks_changed = false;
        Ok(SealedMap { root, repository: self.repository.clone() })
    }
    pub fn retained_objects(&self) -> BTreeSet<ObjectIdentity> {
        self.blocks.values().map(|b| b.object).chain(self.pending.values().map(|p| p.object))
            .chain(self.published.iter().flat_map(|r| r.blocks.iter().map(|b| b.object))).collect()
    }
    pub fn collect_sealed(&self, roots: &[Arc<SealedMap>]) -> Result<usize, String> {
        let mut retained = self.retained_objects();
        retained.extend(roots.iter().flat_map(|r| r.root.blocks.iter().map(|b| b.object)));
        self.repository.collect_objects(&retained)
    }
    pub fn region(&mut self, min: Key, max: Key, point_budget: usize) -> Result<Vec<(Key, Evidence)>, String> {
        if (0..3).any(|i| min[i] > max[i]) { return Err("reversed region".into()); }
        let low = block_key(min); let high = block_key(max);
        let keys = self.blocks
            .range([low[0], i32::MIN, i32::MIN]..=[high[0], i32::MAX, i32::MAX])
            .map(|(&key, _)| key)
            .chain(self.dirty
                .range([low[0], i32::MIN, i32::MIN]..=[high[0], i32::MAX, i32::MAX])
                .map(|(&key, _)| key))
            .filter(|key| (1..3).all(|i| key[i] >= low[i] && key[i] <= high[i]))
            .collect::<BTreeSet<_>>();
        let mut out = Vec::new();
        for key in keys {
            let persisted_block: Arc<Block>;
            let block: &Block = if let Some(dirty) = self.dirty.get(&key) {
                &dirty.block
            } else {
                persisted_block = self.reader.block(&self.blocks[&key])?;
                persisted_block.as_ref()
            };
            for (&local, &evidence) in block.iter() {
                let key = global_key(key, local)?;
                if (0..3).all(|i| key[i] >= min[i] && key[i] <= max[i]) {
                    if out.len() >= point_budget { return Err("region exceeds point budget".into()); } out.push((key, evidence));
                }
            }
        }
        Ok(out)
    }
    #[cfg(test)]
    pub(crate) fn pending_ids(&self) -> Vec<ObjectIdentity> { self.pending.values().map(|p| p.object).collect() }
}

#[cfg(test)]
#[path = "writer_seal_cursor_tests.rs"]
mod seal_cursor_tests;

#[cfg(test)]
#[path = "writer_accounting_tests.rs"]
mod accounting_tests;

#[cfg(test)]
#[path = "writer_region_tests.rs"]
mod region_tests;
