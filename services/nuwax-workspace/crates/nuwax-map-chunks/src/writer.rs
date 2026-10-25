use super::*;
use rayon::prelude::*;

/// `layer_bytes` 始终等于 `layers.bytes()`，在每次替换记录时同步更新，免去整块重算。
struct Dirty { block: Arc<Block>, layers: GroundRecords, layer_bytes: usize, changed: bool }

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
    write_format: u32,
    voxel_size_m: f32, dirty_point_budget: usize, dirty_points: usize, dirty_layer_bytes: usize, max_observed_ns: u64,
    // 自动模式的预算上限（字节）；`None` 为固定预算。首次超过上限时记下所需与上限。
    cache_limit_bytes: Option<usize>, cache_warning: Option<(usize, usize)>,
    #[cfg(test)] pub(crate) cache_warnings: usize,
}
/// 写入器缓存策略。固定：始终使用构造时的预算；自动：即将压力刷写时，若把预算扩到
/// 能放下当前脏块后仍不超过上限就扩大预算、不刷写，否则照原方式刷写。预算只增不减，
/// 两种策略下写入的地图内容相同，只影响刷写时机。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CachePolicy { Fixed, Auto { max_bytes: usize } }
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
            source_watermarks: BTreeMap::new(), source_watermark_bytes: 0, source_watermarks_changed: false, published: None, legacy_baseline: None, id_source: Box::new(id_source), #[cfg(test)] encoded_blocks: 0, write_format: 4, voxel_size_m, dirty_point_budget: cache_bytes / 128, dirty_points: 0, dirty_layer_bytes: 0, max_observed_ns: 0,
            cache_limit_bytes: None, cache_warning: None, #[cfg(test)] cache_warnings: 0 })
    }
    pub fn repository(&self) -> &Repository { &self.repository }
    /// 切换缓存策略；自动模式的上限不能低于当前预算（预算只增不减）。
    pub fn set_cache_policy(&mut self, policy: CachePolicy) -> Result<(), String> {
        match policy {
            CachePolicy::Fixed => self.cache_limit_bytes = None,
            CachePolicy::Auto { max_bytes } => {
                if max_bytes < self.cache_budget_bytes() { return Err("automatic cache limit is below the current budget".into()); }
                self.cache_limit_bytes = Some(max_bytes);
            }
        }
        Ok(())
    }
    /// 当前缓存预算（字节）。
    pub fn cache_budget_bytes(&self) -> usize { self.dirty_point_budget * 128 }
    /// 自动模式首次超过上限时的（所需字节，上限字节）；之后不再更新。
    pub fn cache_limit_warning(&self) -> Option<(usize, usize)> { self.cache_warning }
    /// 自动模式下把预算扩到能放下当前脏块（至少翻倍，不超过上限）。超过上限时只在
    /// 第一次记警告并返回假，由调用方照原方式压力刷写。
    fn grow_cache(&mut self, needed_points: usize) -> bool {
        let Some(limit) = self.cache_limit_bytes else { return false; };
        if needed_points <= limit / 128 {
            self.dirty_point_budget = needed_points.max(self.dirty_point_budget.saturating_mul(2)).min(limit / 128);
            return true;
        }
        if self.cache_warning.is_none() {
            self.cache_warning = Some((needed_points * 128, limit));
            #[cfg(test)] { self.cache_warnings += 1; }
            tracing::warn!(event = "map_chunks.writer_cache_limit", needed_bytes = needed_points * 128, limit_bytes = limit,
                "精细点云写入器缓存已到自动上限，按原方式压力刷写");
        }
        false
    }
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
        let mut bindings = BTreeMap::new();
        for reference in &root.blocks {
            let (block, info, records) = self.reader.parts(reference.object, reference.points)?;
            admit_format(info, root)?;
            if root.version >= 5 && info.versioned {
                ground_records::validate_legacy_sample_occupancy(&records, &block, reference.key, root.voxel_size_m)?;
                admit_layer_spatial_binding(&mut bindings, reference.object, &records, reference.key, root.voxel_size_m)?;
            }
            admit_intrinsic(info, reference.points, root.source_timestamp_ns)?; latest = latest.max(info.latest);
        }
        let source_watermarks: BTreeMap<_, _> = root.source_watermarks.iter().map(|mark|
            (mark.source.clone(), (mark.source_timestamp_ns, mark.source_sequence))).collect();
        let source_watermark_bytes = source_watermarks.keys().map(|source| watermark_bytes(source)).sum::<usize>();
        if source_watermark_bytes > MAX_MANIFEST_BYTES { return Err("source watermarks exceed resident budget".into()); }
        if root.version >= 5 {
            let mut columns = BTreeSet::new();
            for reference in &root.blocks {
                ground_records::heightfield_columns(reference.key, &self.reader.ground_records(reference)?, &mut columns)?;
            }
            ground_records::validate_root_columns(root, &mut self.reader, &columns)?;
        }
        self.source_watermarks = source_watermarks; self.source_watermark_bytes = source_watermark_bytes;
        self.write_format = root.version.max(4);
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
        self.source_watermarks.clear(); self.source_watermark_bytes = 0; self.write_format = 4;
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
            let needed = self.dirty_points + 32768 + self.dirty_layer_bytes() / 64;
            if needed > self.dirty_point_budget && !self.grow_cache(needed) { self.flush()?; }
            let (block, layers) = match self.blocks.get(&key) {
                Some(reference) => {
                    let (block, _, layers) = self.reader.parts(reference.object, reference.points)?;
                    (block, layers)
                },
                None => (Arc::new(Block::new()), GroundRecords::empty()),
            };
            let layer_bytes = layers.bytes();
            self.dirty_layer_bytes += layer_bytes;
            self.dirty_points += block.len(); self.dirty.insert(key, Dirty { block, layers, layer_bytes, changed: false });
        }
        Ok(self.dirty.get_mut(&key).unwrap())
    }
    fn dirty_layer_bytes(&self) -> usize {
        self.dirty_layer_bytes
    }
    /// Read only one touched block; capture keeps no second whole-map statistics cache.
    pub fn layer_observations(&mut self, key: Key) -> Result<LayerObservations, String> {
        if let Some(dirty) = self.dirty.get(&key) { return dirty.layers.legacy().map(|v| v.as_ref().clone()); }
        match self.blocks.get(&key) { Some(reference) => Ok((*self.reader.layers(reference)?).clone()), None => Ok(LayerObservations::new()) }
    }
    /// Replace a touched block's associated layers after original point admission.
    /// Occupancy must already contain the real member hits. On error no layers change.
    pub fn replace_layer_observations(&mut self, key: Key, layers: LayerObservations) -> Result<(), String> {
        let dirty = self.load_dirty(key)?;
        terrain_observations::validate_layers(&layers, &dirty.block)?;
        if *dirty.layers.legacy()? == layers { return Ok(()); }
        let before = dirty.layer_bytes;
        let after = layers.values().map(terrain_observations::layer_bytes).sum::<usize>();
        dirty.layers = GroundRecords::Legacy(Arc::new(layers)); dirty.layer_bytes = after; dirty.changed = true;
        self.dirty_layer_bytes = self.dirty_layer_bytes - before + after;
        self.pending.remove(&key); self.touched.insert(key); self.membership_touched.insert(key);
        Ok(())
    }
    fn column_parts(&mut self, xy: [i32; 2]) -> Result<Vec<(Key, Arc<Block>, GroundRecords)>, String> {
        let low = [xy[0].div_euclid(32), xy[1].div_euclid(32), i32::MIN];
        let high = [low[0], low[1], i32::MAX];
        let keys: BTreeSet<_> = self.blocks.range(low..=high).map(|(k, _)| *k)
            .chain(self.dirty.range(low..=high).map(|(k, _)| *k)).collect();
        let mut parts = Vec::new();
        for key in keys {
            if let Some(dirty) = self.dirty.get(&key) {
                parts.push((key, dirty.block.clone(), dirty.layers.clone()));
            } else {
                let reference = &self.blocks[&key];
                let (block, _, records) = self.reader.parts(reference.object, reference.points)?;
                parts.push((key, block, records));
            }
        }
        Ok(parts)
    }
    pub fn ground_column(&mut self, xy: [i32; 2]) -> Result<ColumnGroundRecords, String> {
        let parts = self.column_parts(xy)?;
        ground_records::assemble_column(xy, self.voxel_size_m,
            &parts.into_iter().map(|(k, _, r)| (k, r)).collect::<Vec<_>>())
    }
    /// One capture snapshot: read each affected native block-XY once, then
    /// assemble each requested XY from the original shared records. No second
    /// persistent column cache or per-point block read is introduced.
    pub fn ground_columns(&mut self, columns: impl IntoIterator<Item = [i32; 2]>)
        -> Result<BTreeMap<[i32; 2], ColumnGroundRecords>, String> {
        let columns: BTreeSet<_> = columns.into_iter().collect();
        let parts = self.prepare_ground_blocks(columns.iter().copied())?;
        let mut groups: BTreeMap<[i32; 2], Vec<(Key, GroundRecords)>> = BTreeMap::new();
        for (key, (_, records)) in parts {
            groups.entry([key[0], key[1]]).or_default().push((key, records));
        }
        let mut result = BTreeMap::new();
        for xy in columns {
            let Some(parts) = groups.get(&[xy[0].div_euclid(32), xy[1].div_euclid(32)]) else { continue; };
            let column = ground_records::assemble_column(xy, self.voxel_size_m, parts)?;
            if column.classified_empty || !column.legacy.is_empty() || !column.heightfields.is_empty() { result.insert(xy, column); }
        }
        Ok(result)
    }
    /// 与对同一列集合调用 `ground_columns` 的结果和首个错误相同，只是 `samples` 判为假
    /// 的列不带高度场诊断样本。列集合以 `(x, y最小, y最大)` 行区间给出：先按原顺序读取
    /// 同一组块，再只组装块内实际带地面记录的列；无记录的列组装结果必为空且不会出错，
    /// 因此不再逐列查找。
    pub fn ground_columns_in_rows(&mut self, rows: &[(i32, i32, i32)], samples: impl Fn([i32; 2]) -> bool + Sync)
        -> Result<BTreeMap<[i32; 2], ColumnGroundRecords>, String> {
        let mut merged: Vec<(i32, i32, i32)> = rows.iter().copied().filter(|row| row.1 <= row.2).collect();
        merged.sort_unstable();
        merged.dedup_by(|next, last| {
            let joined = last.0 == next.0 && i64::from(next.1) <= i64::from(last.2) + 1;
            if joined { last.2 = last.2.max(next.2); }
            joined
        });
        let contains = |xy: [i32; 2]| {
            let index = merged.partition_point(|row| (row.0, row.1) <= (xy[0], xy[1]));
            index > 0 && merged[index - 1].0 == xy[0] && xy[1] <= merged[index - 1].2
        };
        // 每个块列取一个代表列，块集合与原入口逐列换算出的集合相同。
        let block_xys: BTreeSet<_> = merged.iter().flat_map(|&(x, low, high)|
            (low.div_euclid(32)..=high.div_euclid(32)).map(move |by| [x.div_euclid(32), by])).collect();
        let parts = self.prepare_ground_blocks(block_xys.iter().map(|b| [b[0] * 32, b[1] * 32]))?;
        let mut groups: BTreeMap<[i32; 2], Vec<(Key, GroundRecords)>> = BTreeMap::new();
        for (key, (_, records)) in parts {
            groups.entry([key[0], key[1]]).or_default().push((key, records));
        }
        // 每块只遍历一次记录，按列归集；块按原顺序、块内键升序（同列即高度升序），
        // 与逐块逐层查找的组装顺序相同。键只取原查找范围内的 32 层。
        let mut columns: BTreeMap<[i32; 2], Vec<(Key, GroundRecordRef<'_>)>> = BTreeMap::new();
        for (bxy, parts) in &groups {
            for (key, records) in parts {
                for (local, record) in records.iter().filter(|(local, _)| local / 1024 < 32) {
                    let local_xy = i32::from(local % 1024);
                    let xy = [bxy[0] * 32 + local_xy % 32, bxy[1] * 32 + local_xy / 32];
                    if contains(xy) { columns.entry(xy).or_default().push((*key, record)); }
                }
            }
        }
        // 各列组装互不依赖：并行组装后按列升序取结果，返回列序上的第一个错误。
        let columns: Vec<_> = columns.into_iter().collect();
        let q = self.voxel_size_m;
        let assembled: Vec<_> = columns.into_par_iter()
            .map(|(xy, records)| (xy, ground_records::assemble_column_records(xy, q, records, samples(xy)))).collect();
        let mut result = BTreeMap::new();
        for (xy, column) in assembled {
            let column = column?;
            if column.classified_empty || !column.legacy.is_empty() || !column.heightfields.is_empty() { result.insert(xy, column); }
        }
        Ok(result)
    }
    /// Replace the complete declared column, not arbitrary independent fragments.
    /// Every fallible read/validation precedes the infallible dirty installation.
    pub fn replace_ground_column(&mut self, xy: [i32; 2], column: &ColumnGroundRecords) -> Result<(), String> {
        self.replace_ground_columns([(xy, column)])
    }
    /// One admitted frame may replace many XY columns in the same blocks.
    /// Gather each block once; edits are staged as per-block deltas and validated
    /// on the overlaid view, then applied in place only after every check passed.
    pub fn replace_ground_columns<'a>(&mut self,
        columns: impl IntoIterator<Item = ([i32; 2], &'a ColumnGroundRecords)>) -> Result<(), String> {
        let columns: BTreeMap<_, _> = columns.into_iter().collect();
        let prepared = self.prepare_ground_blocks(columns.keys().copied())?;
        // 各列暂存只读���记录、互不依赖：并行计算后按列序合并，返回列序上的第一个错误。
        let columns: Vec<_> = columns.into_iter().collect();
        let q = self.voxel_size_m;
        let staged: Vec<_> = columns.par_iter().map(|(xy, column)| Self::stage_column(*xy, q, column, &prepared)).collect();
        let mut deltas = BTreeMap::new();
        for pieces in staged { Self::merge_staged(&mut deltas, pieces?); }
        let changed: BTreeSet<_> = deltas.keys().copied().collect();
        self.validate_staged(&prepared, &deltas, &changed)?;
        self.install_ground_parts(prepared, deltas, &changed);
        self.write_format = 5;
        Ok(())
    }
    /// 每个改动块在“原记录 + 增量”的视图上做整块校验，顺序与在改好的整表上校验相同。
    /// 各块校验互不依赖：并行校验后按块键升序取结果，返回第一个错误。
    fn validate_staged(&self, prepared: &BTreeMap<Key, (Arc<Block>, GroundRecords)>,
        deltas: &BTreeMap<Key, BTreeMap<u16, Option<GroundRecord>>>, changed: &BTreeSet<Key>) -> Result<(), String> {
        let q = self.voxel_size_m;
        let keys: Vec<_> = changed.iter().copied().collect();
        let results: Vec<_> = keys.par_iter().map(|key| {
            let (block, records) = &prepared[key];
            match deltas.get(key) {
                // 有增量的块安装后必为版本化记录，按版本化分支校验。
                Some(delta) => {
                    let view = ground_records::overlay(records, delta);
                    ground_records::validate_versioned_records(view.len(), view.iter().copied(), block)?;
                    ground_records::validate_versioned_legacy_samples(view.iter().map(|(_, r)| *r), block, *key, q)
                }
                None => {
                    ground_records::validate_records(records, block)?;
                    ground_records::validate_legacy_sample_occupancy(records, block, *key, q)
                }
            }
        }).collect();
        results.into_iter().collect()
    }
    /// 把一列的分块增量并入总增量；同一批各列的键互不重叠。
    fn merge_staged(deltas: &mut BTreeMap<Key, BTreeMap<u16, Option<GroundRecord>>>,
        pieces: Vec<(Key, BTreeMap<u16, Option<GroundRecord>>)>) {
        for (key, piece) in pieces { deltas.entry(key).or_insert_with(BTreeMap::new).extend(piece); }
    }
    fn prepare_ground_blocks(&mut self, columns: impl IntoIterator<Item = [i32; 2]>)
        -> Result<BTreeMap<Key, (Arc<Block>, GroundRecords)>, String> {
        let block_xys: BTreeSet<_> = columns.into_iter().map(|xy| [xy[0].div_euclid(32),xy[1].div_euclid(32)]).collect();
        let mut prepared = BTreeMap::new();
        for bxy in block_xys {
            for (key, block, records) in self.column_parts([bxy[0]*32,bxy[1]*32])? {
                prepared.insert(key,(block,records));
            }
        }
        Ok(prepared)
    }
    /// 只读原记录，把本列的替换记成各块的增量；同一批各列的键互不重叠。
    fn stage_column(xy: [i32; 2], q: f32, column: &ColumnGroundRecords,
        prepared: &BTreeMap<Key, (Arc<Block>, GroundRecords)>)
        -> Result<Vec<(Key, BTreeMap<u16, Option<GroundRecord>>)>, String> {
        let mut additions = ground_records::split_column(xy, q, column)?;
        if additions.keys().any(|key| !prepared.contains_key(key)) { return Err("heightfield members lack original occupied block".into()); }
        let mut pieces = Vec::new();
        let local_xy = local_key([xy[0], xy[1], 0]) % 1024;
        let low = [xy[0].div_euclid(32),xy[1].div_euclid(32),i32::MIN];
        let high = [low[0],low[1],i32::MAX];
        if column.classified_empty {
            if let Some((&key, _)) = prepared.range(low..=high).find(|(_, (block, _))|
                (0..32_u16).any(|z| block.contains_key(&(local_xy+1024*z)))) {
                additions.entry(key).or_default().insert(local_xy, GroundRecord::ClassifiedEmptyColumn {});
            }
        }
        for (key, (_, records)) in prepared.range(low..=high) {
            let add = additions.remove(key).unwrap_or_default();
            let same = (0..32_u16).all(|z| {
                let k = local_xy + 1024*z;
                records.get(k) == add.get(&k).map(GroundRecord::as_ref)
            });
            if same { continue; }
            // 与原先“删除本列全部 32 层再写入新记录”的结果相同。
            let mut delta = BTreeMap::new();
            for z in 0..32_u16 {
                let k = local_xy + 1024*z;
                if records.get(k).is_some() { delta.insert(k, None); }
            }
            delta.extend(add.into_iter().map(|(k, value)| (k, Some(value))));
            pieces.push((*key, delta));
        }
        Ok(pieces)
    }
    fn install_ground_parts(&mut self, prepared: BTreeMap<Key, (Arc<Block>, GroundRecords)>,
        mut deltas: BTreeMap<Key, BTreeMap<u16, Option<GroundRecord>>>, changed: &BTreeSet<Key>) {
        // No IO, encoding, pressure-flush or full-block equality comparison after
        // the first fragment is installed. The dirty set came from actual edits.
        for (key, (block, mut records)) in prepared {
            if !changed.contains(&key) { continue; }
            // 先取出旧脏块，释放它对记录表的共享引用，增量随后原地应用。
            let mut layer_bytes = match self.dirty.remove(&key) {
                Some(old) => {
                    self.dirty_points -= old.block.len(); self.dirty_layer_bytes -= old.layer_bytes;
                    old.layer_bytes
                }
                None => records.bytes(),
            };
            if let Some(delta) = deltas.remove(&key) {
                let mutable = records.versioned_mut();
                for (k, change) in delta {
                    if let Some(old) = mutable.remove(&k) { layer_bytes -= ground_records::record_bytes(old.as_ref()); }
                    if let Some(value) = change {
                        layer_bytes += ground_records::record_bytes(value.as_ref());
                        mutable.insert(k, value);
                    }
                }
            }
            let records = records.compact(&block);
            self.dirty_points += block.len(); self.dirty_layer_bytes += layer_bytes;
            self.dirty.insert(key, Dirty { block, layers: records, layer_bytes, changed: true });
            self.pending.remove(&key); self.touched.insert(key); self.membership_touched.insert(key);
        }
    }
    pub fn remove(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> {
        if self.write_format < 5 { return self.remove_legacy(keys); }
        let keys: BTreeSet<_> = keys.into_iter().collect();
        let block_keys: BTreeSet<_> = keys.iter().map(|k| block_key(*k)).collect();
        let mut versioned = false;
        for key in block_keys {
            if let Some(dirty) = self.dirty.get(&key) { versioned |= matches!(dirty.layers, GroundRecords::Versioned(_)); }
            else if let Some(reference) = self.blocks.get(&key) { versioned |= matches!(self.reader.ground_records(reference)?, GroundRecords::Versioned(_)); }
        }
        if !versioned { return self.remove_legacy(keys); }
        let columns: BTreeSet<_> = keys.iter().map(|k| [k[0],k[1]]).collect();
        let mut prepared = self.prepare_ground_blocks(columns.iter().copied())?;
        let records: Vec<_> = prepared.iter().map(|(k,(_,r))| (*k,r.clone())).collect();
        let mut originals = Vec::new();
        for xy in columns { originals.push((xy, ground_records::assemble_column(xy, self.voxel_size_m, &records)?)); }
        drop(records);
        let mut changed = BTreeSet::new();
        let mut deltas = BTreeMap::new();
        for key in &keys {
            if let Some((block, _)) = prepared.get_mut(&block_key(*key)) {
                if block.contains_key(&local_key(*key)) {
                    Arc::make_mut(block).remove(&local_key(*key)); changed.insert(block_key(*key));
                }
            }
        }
        for (xy, mut column) in originals {
            let was_native = column.classified_empty || !column.heightfields.is_empty();
            column.legacy = column.legacy.into_iter().filter_map(|(block, mut layer)| {
                layer.members.retain(|m| prepared[&block].0.contains_key(m));
                if layer.members.is_empty() { return None; }
                layer.samples.retain(|sample| {
                    let key = sample.point.map(|v| (v * (1.0 / f64::from(self.voxel_size_m))).floor() as i32);
                    !keys.contains(&key)
                });
                (!layer.samples.is_empty()).then_some((block,layer))
            }).collect();
            column.heightfields = column.heightfields.into_iter().filter_map(|mut surface| {
                let live = surface.column.members().map(|(z,_)| z).filter(|z| !keys.contains(&[xy[0],xy[1],*z])).collect();
                surface.column = surface.column.retain_members(&live)?;
                surface.samples.retain(|sample| {
                    let key = sample.point.map(|v| (v * (1.0 / f64::from(self.voxel_size_m))).floor() as i32);
                    !keys.contains(&key)
                });
                Some(surface)
            }).collect();
            column.classified_empty = was_native && column.legacy.is_empty() && column.heightfields.is_empty();
            Self::merge_staged(&mut deltas, Self::stage_column(xy, self.voxel_size_m, &column, &prepared)?);
        }
        changed.extend(deltas.keys().copied());
        self.validate_staged(&prepared, &deltas, &changed)?;
        self.install_ground_parts(prepared, deltas, &changed);
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
                match Arc::make_mut(&mut dirty.block).entry(local) {
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
    fn remove_legacy(&mut self, keys: impl IntoIterator<Item = Key>) -> Result<(), String> {
        let quantum=f64::from(self.voxel_size_m);
        let mut grouped: BTreeMap<Key, BTreeSet<u16>> = BTreeMap::new();
        for key in keys { grouped.entry(block_key(key)).or_default().insert(local_key(key)); }
        for (key, removed) in grouped {
            if !self.blocks.contains_key(&key) && !self.dirty.contains_key(&key) { continue; }
            let dirty = self.load_dirty(key)?; let before = dirty.block.len();
            for local in &removed { Arc::make_mut(&mut dirty.block).remove(local); }
            let before_layer_bytes=dirty.layer_bytes;
            let layers = dirty.layers.legacy_mut()?;
            *layers=std::mem::take(layers).into_values().filter_map(|mut layer| {
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
            dirty.layer_bytes=dirty.layers.bytes();
            let removed_layer_bytes=before_layer_bytes-dirty.layer_bytes;
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
                let bytes = encode_ground_block(&self.dirty[&key].block, &self.dirty[&key].layers)?;
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
                        let (a, a_records) = self.reader.ground_parts(a)?; let (b, b_records) = self.reader.ground_parts(b)?;
                        if a.keys().ne(b.keys()) || !a_records.geometry_eq(&b_records) { changed_members.insert(*key); }
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
        let root = Arc::new(Manifest { version: self.write_format, domain, revision, geometry_revision, source_timestamp_ns: timestamp_ns,
            voxel_size_m: self.voxel_size_m, blocks: blocks.values().cloned().collect(),
            source_watermarks: self.source_watermarks.iter().map(|(source, &(source_timestamp_ns, source_sequence))|
                SourceWatermark { source: source.clone(), source_timestamp_ns, source_sequence }).collect() });
        root.validate()?;
        if self.write_format >= 5 {
            let mut columns = BTreeSet::new();
            for key in &self.membership_touched {
                if let Some(reference) = blocks.get(key) {
                    ground_records::heightfield_columns(*key, &self.reader.ground_records(reference)?, &mut columns)?;
                }
                if let Some(old) = &self.published {
                    if let Ok(i) = old.blocks.binary_search_by_key(key, |r| r.key) {
                        ground_records::heightfield_columns(*key, &self.reader.ground_records(&old.blocks[i])?, &mut columns)?;
                    }
                }
            }
            ground_records::validate_root_columns(&root, &mut self.reader, &columns)?;
        }
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

#[cfg(test)]
mod capture_column_tests {
    use super::*;
    #[test]
    fn capture_batch_reads_each_native_block_once_and_preserves_columns() {
        let repo = Repository::temporary().unwrap();
        let mut writer = Writer::new(repo.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        let mut expected = BTreeMap::new();
        for xy in [[0,0],[1,0],[32,0]] {
            let column = crate::heightfield_observations::HeightfieldColumn::from_counts(xy, 0.05,
                [-1,0,33].into_iter().map(|z|(z,[1,0,0,0,0,0,0,0])).collect()).unwrap().unwrap();
            writer.observe(column.members().map(|(z,_)|[xy[0],xy[1],z]),20).unwrap();
            expected.insert(xy, ColumnGroundRecords { classified_empty: false, legacy:vec![],
                heightfields:vec![HeightfieldSurface {column,samples:vec![]}] });
        }
        writer.replace_ground_columns(expected.iter().map(|(xy,column)|(*xy,column))).unwrap();
        let root = writer.snapshot(Domain {session:"capture-read".into(),generation:1,
            frame_generation:1,producer_generation:1},1,20).unwrap();
        assert_eq!(root.blocks.len(),6);
        writer.reader = RegionReader::new(repo.clone(),0);
        let before = repo.counts().0;
        let actual = writer.ground_columns([[0,0],[1,0],[32,0],[0,0],[200,200]]).unwrap();
        assert_eq!(repo.counts().0-before,6,"one original decode request per distinct native block");
        assert_eq!(actual,expected);
        assert!(!writer.has_unpublished_changes());
    }
}
