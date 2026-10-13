use super::*;

struct Dirty { block: Block, layers: LayerObservations, changed: bool }
struct Pending { object: ObjectIdentity, bytes: Vec<u8>, installed: bool }
/// Dirty payload is bounded; membership baselines retain references, never copied member lists.
pub struct Writer {
    repository: Repository, blocks: BTreeMap<Key, BlockRef>, dirty: BTreeMap<Key, Dirty>,
    pending: BTreeMap<Key, Pending>, touched: BTreeSet<Key>, membership_touched: BTreeSet<Key>,
    layer_format: bool,
    published: Option<Arc<Manifest>>, legacy_baseline: Option<(Domain, u64)>, reader: RegionReader,
    id_source: Box<dyn FnMut() -> Result<ObjectId, String> + Send>,
    #[cfg(test)] pub(crate) encoded_blocks: usize,
    voxel_size_m: f32, dirty_point_budget: usize, dirty_points: usize, max_observed_ns: u64,
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
            layer_format: false, published: None, legacy_baseline: None, id_source: Box::new(id_source), #[cfg(test)] encoded_blocks: 0, voxel_size_m, dirty_point_budget: cache_bytes / 128, dirty_points: 0, max_observed_ns: 0 })
    }
    pub fn repository(&self) -> &Repository { &self.repository }
    pub fn has_unpublished_changes(&self) -> bool {
        !self.touched.is_empty() || !self.membership_touched.is_empty() || !self.pending.is_empty()
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
    pub fn resident_bytes(&self) -> usize { self.dirty_layer_bytes() + self.dirty_points * 64 + self.reader.cached_bytes() + self.pending.values().map(|p| p.bytes.len()).sum::<usize>() }
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
            let layers = self.reader.layers(reference)?;
            if !layers.is_empty() && root.version < 3 { return Err("observed layers require root version 3".into()); }
            let info = intrinsic(&*self.reader.block(reference)?); admit_intrinsic(info, reference.points, root.source_timestamp_ns)?; latest = latest.max(info.latest);
        }
        self.layer_format = root.version >= 3;
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
            let view = BlockRef { key: reference.key, object, points: reference.points, block_revision: 0, geometry_revision: 0 };
            if !self.reader.layers(&view)?.is_empty() { return Err("legacy root cannot contain observed layers".into()); }
            let info = intrinsic(&*self.reader.object(object, reference.points)?);
            admit_intrinsic(info, reference.points, root.source_timestamp_ns)?; latest = latest.max(info.latest);
        }
        self.layer_format = false;
        self.blocks = root.blocks.iter().map(|b| (b.key, BlockRef { key: b.key, object: ObjectIdentity::Legacy(b.sha256), block_revision: 0, geometry_revision: 0, points: b.points })).collect();
        self.reset_pending(); self.max_observed_ns = latest; self.published = None;
        self.legacy_baseline = Some((root.domain.clone(), root.revision)); Ok(())
    }
    pub fn restore_archived(&mut self, root: &ArchivedRoot) -> Result<(), String> {
        match root { ArchivedRoot::Current(root) => self.restore(root), ArchivedRoot::Legacy(root) => self.restore_legacy(&root.manifest) }
    }
    fn reset_pending(&mut self) {
        self.dirty.clear(); self.pending.clear(); self.touched.clear(); self.membership_touched.clear(); self.dirty_points = 0;
    }
    fn load_dirty(&mut self, key: Key) -> Result<&mut Dirty, String> {
        if !self.dirty.contains_key(&key) {
            if self.dirty_points + 32768 + self.dirty_layer_bytes() / 64 > self.dirty_point_budget { self.flush()?; }
            let block = match self.blocks.get(&key) { Some(reference) => (*self.reader.block(reference)?).clone(), None => Block::new() };
            let layers = match self.blocks.get(&key) { Some(reference) => (*self.reader.layers(reference)?).clone(), None => LayerObservations::new() };
            self.dirty_points += block.len(); self.dirty.insert(key, Dirty { block, layers, changed: false });
        }
        Ok(self.dirty.get_mut(&key).unwrap())
    }
    fn dirty_layer_bytes(&self) -> usize {
        self.dirty.values().flat_map(|d| d.layers.values()).map(|l| 128 + l.members.len() * 2 + l.samples.len() * 64).sum()
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
        dirty.layers = layers; dirty.changed = true;
        self.layer_format = true;
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
        let mut grouped: BTreeMap<Key, BTreeSet<u16>> = BTreeMap::new();
        for key in keys { grouped.entry(block_key(key)).or_default().insert(local_key(key)); }
        for (key, removed) in grouped {
            if !self.blocks.contains_key(&key) && !self.dirty.contains_key(&key) { continue; }
            let dirty = self.load_dirty(key)?; let before = dirty.block.len();
            for local in removed { dirty.block.remove(&local); }
            dirty.layers.retain(|_, layer| layer.members.iter().all(|member| dirty.block.contains_key(member)));
            let removed = before - dirty.block.len(); dirty.changed |= removed > 0; self.dirty_points -= removed;
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
        self.dirty.clear(); self.pending.clear(); self.dirty_points = 0; Ok(())
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
        for (key, block) in &mut blocks {
            if !same_domain { block.block_revision = revision; block.geometry_revision = revision; }
            else {
                if self.touched.contains(key) { block.block_revision = revision; }
                if changed_members.contains(key) { block.geometry_revision = revision; }
                else {
                    let previous = self.published.as_ref().unwrap();
                    if let Ok(index) = previous.blocks.binary_search_by_key(key, |b| b.key) {
                        block.geometry_revision = previous.blocks[index].geometry_revision;
                    }
                }
            }
        }
        let root = Arc::new(Manifest { version: if self.layer_format { 3 } else { 2 }, domain, revision, geometry_revision, source_timestamp_ns: timestamp_ns,
            voxel_size_m: self.voxel_size_m, blocks: blocks.values().cloned().collect() });
        root.validate()?;
        self.blocks = blocks; self.published = Some(root.clone()); self.legacy_baseline = None;
        self.touched.clear(); self.membership_touched.clear();
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
        let keys = self.blocks.keys().chain(self.dirty.keys()).copied().filter(|key| (0..3).all(|i| key[i] >= low[i] && key[i] <= high[i])).collect::<BTreeSet<_>>();
        let mut out = Vec::new();
        for key in keys {
            let block = if let Some(dirty) = self.dirty.get(&key) { Arc::new(dirty.block.clone()) } else { self.reader.block(&self.blocks[&key])? };
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
