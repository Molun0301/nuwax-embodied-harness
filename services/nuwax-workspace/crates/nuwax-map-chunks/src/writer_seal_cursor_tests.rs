use super::*;

const BUDGET: usize = 4 * 1024 * 1024;
fn domain() -> Domain {
    Domain { session: "seal-cursor-oracle".into(), generation: 1, frame_generation: 0, producer_generation: 1 }
}
fn deterministic_writer(budget: usize) -> Writer {
    let mut sequence = 0_u64;
    Writer::with_id_source(Repository::temporary().unwrap(), 0.125, budget, move || {
        sequence += 1;
        let mut bytes = [0_u8; 16];
        bytes[..8].copy_from_slice(&sequence.to_le_bytes());
        Ok(ObjectId(bytes))
    }).unwrap()
}

struct Pair { actual: Writer, oracle: Writer }
impl Pair {
    fn new(budget: usize) -> Self {
        Self { actual: deterministic_writer(budget), oracle: deterministic_writer(budget) }
    }
    fn edit(&mut self, action: impl Fn(&mut Writer) -> Result<(), String>) -> Result<(), String> {
        let actual = action(&mut self.actual);
        let expected = action(&mut self.oracle);
        assert_eq!(actual, expected);
        self.assert_state();
        actual
    }
    fn seal(&mut self, domain: Domain, revision: u64, timestamp: u64) -> Result<Arc<Manifest>, String> {
        let actual = self.actual.seal(domain.clone(), revision, timestamp).map(|sealed| sealed.root);
        let expected = self.oracle.frozen_binary_seal(domain, revision, timestamp).map(|sealed| sealed.root);
        assert_eq!(actual, expected); // Complete Manifest, not only revision counters.
        self.assert_state();
        actual
    }
    fn rehome(&mut self, frame: u64) -> Arc<Manifest> {
        let expected = self.actual.published.as_ref().unwrap().identity();
        let actual = self.actual.rehome_published_frame(&expected, frame).map(|sealed| sealed.root);
        let oracle = self.oracle.rehome_published_frame(&expected, frame).map(|sealed| sealed.root);
        assert_eq!(actual, oracle);
        self.assert_state();
        actual.unwrap()
    }
    fn assert_state(&self) {
        let a = &self.actual;
        let b = &self.oracle;
        assert_eq!(a.blocks, b.blocks);
        assert_eq!(a.published, b.published);
        assert_eq!(a.legacy_baseline, b.legacy_baseline);
        assert_eq!(a.touched, b.touched);
        assert_eq!(a.membership_touched, b.membership_touched);
        let dirty = |w: &Writer| w.dirty.iter().map(|(key, d)|
            (*key, d.block.clone(), d.layers.clone(), d.changed)).collect::<Vec<_>>();
        let pending = |w: &Writer| w.pending.iter().map(|(key, p)|
            (*key, p.object, p.bytes.clone(), p.installed)).collect::<Vec<_>>();
        assert_eq!(dirty(a), dirty(b));
        assert_eq!(pending(a), pending(b));
        assert_eq!(a.source_watermarks, b.source_watermarks);
        assert_eq!(a.source_watermark_bytes, b.source_watermark_bytes);
        assert_eq!(a.source_watermarks_changed, b.source_watermarks_changed);
        assert_eq!(a.max_observed_ns, b.max_observed_ns);
        assert_eq!((a.dirty_points, a.dirty_layer_bytes, a.dirty_point_budget),
            (b.dirty_points, b.dirty_layer_bytes, b.dirty_point_budget));
        assert_eq!(a.encoded_blocks, b.encoded_blocks);
        assert_eq!(a.resident_bytes(), b.resident_bytes());
        assert_eq!(a.retained_objects(), b.retained_objects());
        assert_eq!(a.has_unpublished_changes(), b.has_unpublished_changes());
        assert_eq!(a.repository.counts(), b.repository.counts());
        for w in [a, b] {
            if let Some(old) = &w.published {
                let previous = old.blocks.iter().map(|r| r.key).collect::<BTreeSet<_>>();
                let mut current = w.blocks.keys().copied().collect::<BTreeSet<_>>();
                for (key, dirty) in &w.dirty {
                    if dirty.block.is_empty() { current.remove(key); } else { current.insert(*key); }
                }
                // Test-only invariant check. No production statistics or key-set check is added.
                assert!(current.symmetric_difference(&previous).all(|key| w.membership_touched.contains(key)));
            }
        }
    }
}

struct OldReaders {
    actual: Arc<SealedMap>, oracle: Arc<SealedMap>, before: Manifest,
    actual_reader: RegionReader, oracle_reader: RegionReader,
    cached: Vec<(Arc<Block>, Arc<Block>, Vec<u8>)>,
}
impl OldReaders {
    fn capture(pair: &Pair) -> Self {
        let actual = Arc::new(SealedMap { root: pair.actual.published.clone().unwrap(), repository: pair.actual.repository.clone() });
        let oracle = Arc::new(SealedMap { root: pair.oracle.published.clone().unwrap(), repository: pair.oracle.repository.clone() });
        let before = (*actual.root).clone();
        let mut actual_reader = RegionReader::new(actual.repository.clone(), BUDGET);
        let mut oracle_reader = RegionReader::new(oracle.repository.clone(), BUDGET);
        let cached = before.blocks.iter().map(|reference| {
            let a = actual_reader.block(reference).unwrap();
            let b = oracle_reader.block(reference).unwrap();
            assert_eq!(a, b);
            let bytes = actual.repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap();
            assert_eq!(bytes, oracle.repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap());
            (a, b, bytes)
        }).collect();
        pair.assert_state();
        Self { actual, oracle, before, actual_reader, oracle_reader, cached }
    }
    fn verify(&mut self, pair: &Pair) {
        assert_eq!(*self.actual.root, self.before);
        assert_eq!(*self.oracle.root, self.before);
        for (reference, (cached_a, cached_b, bytes)) in self.before.blocks.iter().zip(&self.cached) {
            let a = self.actual_reader.block(reference).unwrap();
            let b = self.oracle_reader.block(reference).unwrap();
            assert!(Arc::ptr_eq(&a, cached_a));
            assert!(Arc::ptr_eq(&b, cached_b));
            assert_eq!(a, b);
            assert_eq!(self.actual.repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap(), *bytes);
            assert_eq!(self.oracle.repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap(), *bytes);
        }
        pair.assert_state();
    }
}

// Source-frozen original Writer::seal from HEAD 627fd961. Do not delegate to the candidate.
impl Writer {
    fn frozen_binary_seal(&mut self, domain: Domain, revision: u64, timestamp_ns: u64) -> Result<SealedMap, String> {
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
        let root = Arc::new(Manifest { version: 4, domain, revision, geometry_revision, source_timestamp_ns: timestamp_ns,
            voxel_size_m: self.voxel_size_m, blocks: blocks.values().cloned().collect(),
            source_watermarks: self.source_watermarks.iter().map(|(source, &(source_timestamp_ns, source_sequence))|
                SourceWatermark { source: source.clone(), source_timestamp_ns, source_sequence }).collect() });
        root.validate()?;
        self.blocks = blocks; self.published = Some(root.clone()); self.legacy_baseline = None;
        self.touched.clear(); self.membership_touched.clear(); self.source_watermarks_changed = false;
        Ok(SealedMap { root, repository: self.repository.clone() })
    }
}


fn geometry(root: &Manifest) -> Vec<u64> { root.blocks.iter().map(|b| b.geometry_revision).collect() }
fn typed_source() -> Arc<LayerSource> {
    Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1, sensor_frame_id: "lidar".into() })
}
fn layers(timestamp: u64, height: f64) -> LayerObservations {
    [(0, LayerObservation {
        members: vec![0], plane_origin: [0.0, 0.0, height], normal: [0.0, 0.0, 1.0],
        samples: vec![LayerSample { source: Arc::new(LayerSource::Legacy), point: [0.0, 0.0, 0.0],
            source_sequence: timestamp, source_timestamp_ns: timestamp, point_index: 0 }],
    })].into()
}

#[test]
fn seal_cursor_noop_evidence_and_watermarks_match_complete_roots() {
    let mut pair = Pair::new(BUDGET * 2);
    let empty = pair.seal(domain(), 1, 1).unwrap();
    assert!(empty.blocks.is_empty());
    assert_eq!(pair.seal(domain(), 2, 1).unwrap().geometry_revision, 1);
    let points = [[-64, 0, 0], [0, -32, 0], [0, 0, -32], [64, 0, 0]];
    pair.edit(|w| w.observe(points, 1)).unwrap();
    pair.seal(domain(), 3, 1).unwrap();
    pair.edit(|w| w.observe([[-63, 0, 0]], 2)).unwrap();
    pair.seal(domain(), 5, 2).unwrap();
    pair.edit(|w| w.observe([[0, 0, -31]], 3)).unwrap();
    let baseline = pair.seal(domain(), 7, 3).unwrap();
    assert_eq!(geometry(&baseline), vec![5, 3, 7, 3]);
    let mut readers = OldReaders::capture(&pair);
    let counts = pair.actual.repository.counts();
    let noop = pair.seal(domain(), 8, 3).unwrap();
    assert_eq!(noop.blocks, baseline.blocks);
    assert_eq!(pair.actual.repository.counts(), counts);
    for (index, point) in points.into_iter().enumerate() {
        let stamp = 4 + index as u64;
        pair.edit(|w| w.observe([point], stamp)).unwrap();
        let next = pair.seal(domain(), 9 + index as u64, stamp).unwrap();
        assert_eq!(geometry(&next), geometry(&baseline));
        assert_eq!(next.geometry_revision, baseline.geometry_revision);
    }
    pair.edit(|w| w.observe(points, 8)).unwrap();
    let all_evidence = pair.seal(domain(), 13, 8).unwrap();
    assert_eq!(geometry(&all_evidence), geometry(&baseline));
    assert!(all_evidence.blocks.iter().all(|block| block.block_revision == 13));
    pair.edit(|w| w.record_source_watermark(typed_source(), 9, 1)).unwrap();
    let mark = pair.seal(domain(), 14, 9).unwrap();
    assert_eq!(geometry(&mark), geometry(&baseline));
    assert_eq!(mark.source_watermarks.len(), 1);
    readers.verify(&pair);
}

#[test]
fn seal_cursor_net_remove_readd_survives_real_pressure_flush() {
    let mut pair = Pair::new(BUDGET); // One resident point + the next block triggers pressure flush.
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    let baseline = pair.seal(domain(), 3, 1).unwrap();
    let mut readers = OldReaders::capture(&pair);
    pair.edit(|w| w.remove([[0, 0, 0]])).unwrap();
    pair.edit(Writer::flush).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0]], 2)).unwrap();
    let encoded = pair.actual.encoded_blocks;
    pair.edit(|w| w.observe([[32, 0, 0]], 2)).unwrap();
    assert_eq!(pair.actual.encoded_blocks, encoded + 1); // Real production load_dirty pressure path.
    assert_eq!(pair.actual.blocks[&[0, 0, 0]].geometry_revision, 0);
    let next = pair.seal(domain(), 4, 2).unwrap();
    assert_eq!(geometry(&next), geometry(&baseline));
    assert_eq!(next.geometry_revision, 3);
    assert_ne!(next.blocks[0].object, baseline.blocks[0].object);
    pair.edit(|w| w.observe([[64, 0, 0]], 3)).unwrap();
    pair.edit(|w| w.remove([[64, 0, 0]])).unwrap();
    let cancelled = pair.seal(domain(), 5, 3).unwrap();
    assert_eq!(cancelled.blocks, next.blocks);
    assert_eq!(cancelled.geometry_revision, 3);
    readers.verify(&pair);
}

#[test]
fn seal_cursor_rehome_preserves_pressure_flushed_and_dirty_net_geometry() {
    let mut pair = Pair::new(BUDGET);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0], [64, 0, 0]], 1)).unwrap();
    pair.seal(domain(), 3, 1).unwrap();
    pair.seal(domain(), 7, 1).unwrap();
    let mut readers = OldReaders::capture(&pair);
    pair.edit(|w| w.remove([[0, 0, 0]])).unwrap();
    pair.edit(Writer::flush).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0]], 2)).unwrap();
    let encoded = pair.actual.encoded_blocks;
    pair.edit(|w| w.observe([[32, 0, 0]], 2)).unwrap();
    assert_eq!(pair.actual.encoded_blocks, encoded + 1);
    let counts = pair.actual.repository.counts();
    let moved = pair.rehome(1);
    assert_eq!(pair.actual.repository.counts(), counts);
    assert_eq!(geometry(&moved), vec![7; 3]);
    let next = pair.seal(moved.domain.clone(), 8, 2).unwrap();
    assert_eq!(geometry(&next), vec![7; 3]);
    assert_eq!(next.geometry_revision, 7);
    assert_eq!(next.blocks[0].block_revision, 8);
    assert_eq!(next.blocks[1].block_revision, 8);
    assert_eq!(next.blocks[2], moved.blocks[2]);
    readers.verify(&pair);
}

#[test]
fn seal_cursor_same_count_key_swap_and_sparse_inheritance_keep_fallback_results() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0], [64, 0, 0], [96, 0, 0]], 1)).unwrap();
    pair.seal(domain(), 1, 1).unwrap();
    pair.edit(|w| w.observe([[33, 0, 0]], 2)).unwrap();
    let baseline = pair.seal(domain(), 3, 2).unwrap();
    assert_eq!(geometry(&baseline), vec![1, 3, 1, 1]);
    pair.edit(|w| w.remove([[0, 0, 0]])).unwrap();
    pair.edit(|w| w.observe([[128, 0, 0]], 3)).unwrap();
    let swapped = pair.seal(domain(), 4, 3).unwrap();
    assert_eq!(swapped.blocks.len(), baseline.blocks.len());
    assert_eq!(swapped.blocks.iter().map(|b| b.key).collect::<Vec<_>>(), vec![[1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0]]);
    assert_eq!(geometry(&swapped), vec![3, 1, 1, 4]);
    assert_eq!(swapped.geometry_revision, 4); // Same N=M, but changed_members cannot be empty.
    pair.edit(|w| w.observe([[34, 0, 0], [65, 0, 0], [97, 0, 0]], 4)).unwrap();
    let sparse = pair.seal(domain(), 5, 4).unwrap();
    assert_eq!(geometry(&sparse), vec![5, 5, 5, 4]); // L=1, no synchronous cursor path.
    pair.edit(|w| w.remove([[32, 0, 0], [33, 0, 0], [34, 0, 0], [64, 0, 0], [65, 0, 0], [96, 0, 0], [97, 0, 0]])).unwrap();
    let tail = pair.seal(domain(), 6, 4).unwrap();
    assert_eq!(tail.blocks, vec![sparse.blocks[3].clone()]);
    assert_eq!(tail.geometry_revision, 6);
    pair.edit(|w| w.remove([[128, 0, 0]])).unwrap();
    assert!(pair.seal(domain(), 7, 4).unwrap().blocks.is_empty());
    assert_eq!(pair.seal(domain(), 8, 4).unwrap().geometry_revision, 7);
    let mut all_changed = Pair::new(BUDGET * 2);
    all_changed.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    all_changed.seal(domain(), 1, 1).unwrap();
    all_changed.edit(|w| w.observe([[1, 0, 0], [33, 0, 0]], 2)).unwrap();
    let no_inheritance = all_changed.seal(domain(), 2, 2).unwrap();
    assert_eq!(geometry(&no_inheritance), vec![2; 2]); // Same-domain, nonempty directory, L=0.
}

#[test]
fn seal_cursor_layer_evidence_and_geometry_have_distinct_revision_results() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    pair.edit(|w| w.replace_layer_observations([0, 0, 0], layers(1, 0.0))).unwrap();
    let baseline = pair.seal(domain(), 1, 1).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0]], 2)).unwrap();
    pair.edit(|w| w.replace_layer_observations([0, 0, 0], layers(2, 0.0))).unwrap();
    let evidence = pair.seal(domain(), 2, 2).unwrap();
    assert_eq!(geometry(&evidence), geometry(&baseline));
    assert_eq!(evidence.geometry_revision, 1);
    pair.edit(|w| w.replace_layer_observations([0, 0, 0], layers(2, 0.01))).unwrap();
    let changed = pair.seal(domain(), 3, 2).unwrap();
    assert_eq!(geometry(&changed), vec![3, 1]);
    assert_eq!(changed.geometry_revision, 3);
}

#[test]
fn seal_cursor_restore_domain_and_legacy_keep_original_revision_contract() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    let root = pair.seal(domain(), 3, 1).unwrap();
    pair.edit(|w| w.restore(&root)).unwrap(); // Reuse the same Writer, preserving the ID source sequence.
    assert_eq!(pair.seal(domain(), 4, 1).unwrap().blocks, root.blocks);
    let mut next_domain = domain(); next_domain.generation += 1;
    let moved = pair.seal(next_domain, 1, 1).unwrap();
    assert_eq!(geometry(&moved), vec![1; 2]);
    assert_eq!(moved.blocks.iter().map(|b| b.object).collect::<Vec<_>>(), root.blocks.iter().map(|b| b.object).collect::<Vec<_>>());
    let mut legacy = Pair::new(BUDGET);
    let alias = LegacyAlias([17; 32]);
    let bytes = encode_block(&[(0, Evidence { observed_ns: 1, hits: 1 })].into()).unwrap();
    legacy.edit(|w| w.repository.put_object(ObjectIdentity::Legacy(alias), &bytes)).unwrap();
    let root = LegacyManifest { version: 1, domain: domain(), revision: 9, source_timestamp_ns: 1,
        voxel_size_m: 0.125, blocks: vec![LegacyBlockRef { key: [0, 0, 0], sha256: alias, geometry_sha256: LegacyAlias([18; 32]), points: 1 }] };
    legacy.edit(|w| w.restore_legacy(&root)).unwrap();
    assert!(legacy.actual.published.is_none());
    let current = legacy.seal(domain(), 10, 1).unwrap();
    assert_eq!(geometry(&current), vec![10]);
    assert_eq!(legacy.seal(domain(), 11, 1).unwrap().blocks, current.blocks);
}

#[test]
fn seal_cursor_failed_install_rehome_and_retry_preserve_frozen_ids() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0], [64, 0, 0]], 1)).unwrap();
    pair.seal(domain(), 2, 1).unwrap();
    pair.seal(domain(), 7, 1).unwrap();
    let mut readers = OldReaders::capture(&pair);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 2)).unwrap();
    pair.edit(|w| { w.repository.fail_put_after(Some(1)); Ok(()) }).unwrap();
    assert_eq!(pair.seal(domain(), 8, 2).unwrap_err(), "injected object write failure");
    let ids = pair.actual.pending_ids();
    assert_eq!(ids.len(), 2);
    let encoded = pair.actual.encoded_blocks;
    assert!(Arc::ptr_eq(pair.actual.published.as_ref().unwrap(), &readers.actual.root));
    let moved = pair.rehome(1);
    assert_eq!(pair.actual.pending_ids(), ids);
    assert_eq!(pair.seal(moved.domain.clone(), 8, 2).unwrap_err(), "injected object write failure");
    assert_eq!(pair.actual.encoded_blocks, encoded);
    pair.edit(|w| { w.repository.fail_put_after(None); Ok(()) }).unwrap();
    let next = pair.seal(moved.domain.clone(), 8, 2).unwrap();
    assert_eq!(next.blocks[..2].iter().map(|b| b.object).collect::<Vec<_>>(), ids);
    assert_eq!(pair.actual.encoded_blocks, encoded);
    assert_eq!(geometry(&next), vec![7; 3]);
    readers.verify(&pair);
}

#[test]
fn seal_cursor_error_precedence_and_final_validation_keep_original_state() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    let old = pair.seal(domain(), 1, 1).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0]], 10)).unwrap();
    pair.edit(|w| { w.repository.fail_put_after(Some(0)); Ok(()) }).unwrap();
    let mut invalid = domain(); invalid.session.clear();
    assert_eq!(pair.seal(invalid, 1, 1).unwrap_err(), "invalid root domain");
    assert_eq!(pair.seal(domain(), 1, 9).unwrap_err(), "root bound precedes observed evidence");
    assert_eq!(pair.seal(domain(), 1, 10).unwrap_err(), "publication revision did not advance");
    assert_eq!(pair.seal(domain(), 2, 10).unwrap_err(), "injected object write failure");
    assert!(Arc::ptr_eq(pair.actual.published.as_ref().unwrap(), &old));
    pair.edit(|w| { w.repository.fail_put_after(None); Ok(()) }).unwrap();
    let published = pair.seal(domain(), 2, 10).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0]], 11)).unwrap();
    // Fault injection changes a validation field, not keys or membership tracking.
    pair.edit(|w| { w.blocks.get_mut(&[1, 0, 0]).unwrap().points = 0; Ok(()) }).unwrap();
    let puts = pair.actual.repository.counts().1;
    assert_eq!(pair.seal(domain(), 3, 11).unwrap_err(), "invalid, duplicate or unsorted block reference");
    assert_eq!(pair.actual.repository.counts().1, puts + 1); // Original flush side effect remains.
    assert!(Arc::ptr_eq(pair.actual.published.as_ref().unwrap(), &published));
    assert_ne!(pair.actual.blocks[&[0, 0, 0]].object, published.blocks[0].object);
    assert!(pair.actual.touched.contains(&[0, 0, 0]));
    assert!(pair.actual.pending.is_empty());
    pair.edit(|w| { w.blocks.get_mut(&[1, 0, 0]).unwrap().points = 1; Ok(()) }).unwrap();
    let encoded = pair.actual.encoded_blocks;
    pair.seal(domain(), 3, 11).unwrap();
    assert_eq!(pair.actual.encoded_blocks, encoded);
}

fn object_file(repository: &Repository, object: ObjectIdentity) -> std::path::PathBuf {
    let name = match object {
        ObjectIdentity::Assigned(id) => format!("id-{}", id.0.iter().map(|b| format!("{b:02x}")).collect::<String>()),
        ObjectIdentity::Legacy(alias) => alias.0.iter().map(|b| format!("{b:02x}")).collect::<String>(),
    };
    repository.path().join(name)
}
fn hide_object(w: &mut Writer, object: ObjectIdentity, hide: bool) -> Result<(), String> {
    let file = object_file(&w.repository, object);
    let held = file.with_extension("oracle-held");
    if hide { std::fs::rename(&file, &held) } else { std::fs::rename(&held, &file) }.map_err(|e| e.to_string())?;
    w.reader = RegionReader::new(w.repository.clone(), BUDGET / 2); // Avoid a cache hit masking the intended read failure.
    Ok(())
}

#[test]
fn seal_cursor_partial_edit_and_membership_read_failures_preserve_tracking() {
    let mut pair = Pair::new(BUDGET * 2);
    pair.edit(|w| w.observe([[0, 0, 0], [32, 0, 0]], 1)).unwrap();
    let old = pair.seal(domain(), 1, 1).unwrap();
    let unavailable = old.blocks[1].object;
    pair.edit(|w| hide_object(w, unavailable, true)).unwrap();
    assert!(pair.edit(|w| w.observe([[1, 0, 0], [32, 0, 0]], 2)).is_err());
    assert!(pair.actual.membership_touched.contains(&[0, 0, 0]));
    assert_eq!(pair.actual.dirty[&[0, 0, 0]].block.len(), 2);
    pair.edit(|w| hide_object(w, unavailable, false)).unwrap();
    let baseline = pair.seal(domain(), 2, 2).unwrap();
    pair.edit(|w| w.remove([[0, 0, 0], [1, 0, 0]])).unwrap();
    pair.edit(Writer::flush).unwrap();
    pair.edit(|w| w.observe([[0, 0, 0], [1, 0, 0]], 3)).unwrap();
    pair.edit(Writer::flush).unwrap();
    let old_object = baseline.blocks[0].object;
    pair.edit(|w| hide_object(w, old_object, true)).unwrap();
    assert!(pair.seal(domain(), 3, 3).is_err());
    assert!(Arc::ptr_eq(pair.actual.published.as_ref().unwrap(), &baseline));
    assert!(pair.actual.membership_touched.contains(&[0, 0, 0]));
    pair.edit(|w| hide_object(w, old_object, false)).unwrap();
    let next = pair.seal(domain(), 3, 3).unwrap();
    assert_eq!(geometry(&next), geometry(&baseline));
    assert_eq!(next.geometry_revision, 2);
}

#[test]
fn seal_cursor_selector_model_excludes_sparse_queries() {
    // Independent structural model only; these are not production counters or timing measurements.
    let cases = [
        (8_usize, vec![7_usize], vec![0, 1, 2, 3, 4, 5, 6], false, 1),
        (8, (0..8).collect(), (0..7).collect(), false, 1),
        (4, vec![1, 2, 3, 4], vec![0, 4], false, 3),
        (4, vec![0, 1, 2, 3], vec![], true, 4),
        (0, vec![], vec![], true, 0),
    ];
    for (old_count, current, changed, expected_cursor, expected_queries) in cases {
        let changed = changed.into_iter().collect::<BTreeSet<_>>();
        let use_cursor = changed.is_empty(); // Same-domain fixtures; changing domains never uses this lookup.
        let queries = current.iter().filter(|key| !changed.contains(*key)).count();
        assert_eq!(use_cursor, expected_cursor);
        assert_eq!(queries, expected_queries);
        if use_cursor {
            assert_eq!(current, (0..old_count).collect::<Vec<_>>());
            assert_eq!(queries, old_count);
        }
    }
}
