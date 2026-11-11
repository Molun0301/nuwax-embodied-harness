use super::*;

fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.05, 64 * 1024 * 1024).unwrap()
}

// Freeze the pre-change selector and output traversal. Do not replace this
// oracle with a call to the proposed selector or Writer::region.
fn full_scan_region(
    writer: &mut Writer,
    min: Key,
    max: Key,
    point_budget: usize,
) -> Result<Vec<(Key, Evidence)>, String> {
    if (0..3).any(|i| min[i] > max[i]) {
        return Err("reversed region".into());
    }
    let low = block_key(min);
    let high = block_key(max);
    let keys = writer.blocks.keys().chain(writer.dirty.keys()).copied()
        .filter(|key| (0..3).all(|i| key[i] >= low[i] && key[i] <= high[i]))
        .collect::<BTreeSet<_>>();
    let mut out = Vec::new();
    for key in keys {
        let block = if let Some(dirty) = writer.dirty.get(&key) {
            dirty.block.clone()
        } else {
            writer.reader.block(&writer.blocks[&key])?
        };
        for (&local, &evidence) in block.iter() {
            let key = global_key(key, local)?;
            if (0..3).all(|i| key[i] >= min[i] && key[i] <= max[i]) {
                if out.len() >= point_budget {
                    return Err("region exceeds point budget".into());
                }
                out.push((key, evidence));
            }
        }
    }
    Ok(out)
}

fn same_as_full_scan(
    writer: &mut Writer,
    min: Key,
    max: Key,
    budget: usize,
) -> Result<Vec<(Key, Evidence)>, String> {
    let expected = full_scan_region(writer, min, max, budget);
    let actual = writer.region(min, max, budget);
    assert_eq!(actual, expected, "min={min:?} max={max:?} budget={budget}");
    actual
}

#[test]
fn region_negative_block_boundaries_and_inclusive_endpoints() {
    let mut writer = writer();
    let xs = [-65, -64, -33, -32, -1, 0, 31, 32, 63, 64];
    writer.observe(xs.into_iter().map(|x| [x, 0, 0]), 1).unwrap();
    writer.flush().unwrap();
    for (min, max, expected) in [
        (-64, -33, vec![-64, -33]),
        (-32, -1, vec![-32, -1]),
        (-1, 0, vec![-1, 0]),
        (0, 31, vec![0, 31]),
        (31, 32, vec![31, 32]),
        (-65, 64, xs.to_vec()),
    ] {
        let actual = same_as_full_scan(&mut writer, [min, 0, 0], [max, 0, 0], 32).unwrap();
        assert_eq!(actual.into_iter().map(|(key, _)| key[0]).collect::<Vec<_>>(), expected);
    }
    assert_eq!(block_key([-33, -32, -1]), [-2, -1, -1]);
    assert_eq!(block_key([0, 31, 32]), [0, 0, 1]);
}

#[test]
fn region_x_slab_still_filters_both_other_axes_and_exact_voxel_bounds() {
    let mut writer = writer();
    // Interior-X blocks outside Y/Z are in the slab, but not the query.
    // Same-block voxels beyond exact endpoints must also remain excluded.
    let points = [
        [-32, -32, -32], [-1, -1, -1], [0, 0, 0], [31, 31, 31],
        [-33, 0, 0], [32, 0, 0], [0, -33, 0], [0, 32, 0],
        [0, 0, -33], [0, 0, 32], [0, 1, 0], [1, 0, 0],
    ];
    writer.observe(points, 1).unwrap();
    writer.flush().unwrap();
    let wide = same_as_full_scan(&mut writer, [-32; 3], [31; 3], 32).unwrap();
    let expected: BTreeSet<_> = points.into_iter().filter(|p|
        p.iter().all(|v| *v >= -32 && *v <= 31)).collect();
    assert_eq!(wide.iter().map(|(key, _)| *key).collect::<BTreeSet<_>>(), expected);
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [0; 3], 1).unwrap(),
        vec![([0; 3], Evidence { observed_ns: 1, hits: 1 })]);
    // Block order followed by u16 local order is the compatibility contract;
    // it is not global XYZ lexicographic order within each block.
    let small = same_as_full_scan(&mut writer, [0; 3], [1, 1, 0], 3).unwrap();
    assert_eq!(small.iter().map(|(key, _)| *key).collect::<Vec<_>>(),
        vec![[0, 0, 0], [1, 0, 0], [0, 1, 0]]);
}

#[test]
fn region_dirty_overrides_persisted_and_empty_dirty_hides_deleted_block() {
    let mut writer = writer();
    writer.observe([[0, 0, 0], [1, 0, 0], [32, 0, 0]], 1).unwrap();
    writer.flush().unwrap();
    writer.observe([[0, 0, 0], [2, 0, 0], [64, 0, 0]], 2).unwrap();
    writer.remove([[1, 0, 0], [32, 0, 0]]).unwrap();
    assert!(writer.blocks.contains_key(&[1, 0, 0]));
    assert!(writer.dirty[&[1, 0, 0]].block.is_empty());
    let expected = vec![
        ([0, 0, 0], Evidence { observed_ns: 2, hits: 2 }),
        ([2, 0, 0], Evidence { observed_ns: 2, hits: 1 }),
        ([64, 0, 0], Evidence { observed_ns: 2, hits: 1 }),
    ];
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [64, 0, 0], 3).unwrap(), expected);
    writer.flush().unwrap();
    assert!(!writer.blocks.contains_key(&[1, 0, 0]));
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [64, 0, 0], 3).unwrap(), expected);
}

#[test]
fn region_empty_dirty_only_block_and_zero_budget_are_preserved() {
    let mut writer = writer();
    writer.observe([[-1; 3]], 1).unwrap();
    writer.remove([[-1; 3]]).unwrap();
    assert!(!writer.blocks.contains_key(&[-1; 3]));
    assert!(writer.dirty[&[-1; 3]].block.is_empty());
    assert_eq!(same_as_full_scan(&mut writer, [-32; 3], [31; 3], 0), Ok(Vec::new()));
    writer.flush().unwrap();
    assert_eq!(same_as_full_scan(&mut writer, [-32; 3], [31; 3], 0), Ok(Vec::new()));
    writer.observe([[0; 3]], 2).unwrap();
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [0; 3], 0),
        Err("region exceeds point budget".into()));
}

#[test]
fn region_budget_errors_and_reversed_axes_match_the_original() {
    let mut writer = writer();
    writer.observe([[-32, 0, 0], [0, 0, 0], [32, 0, 0]], 1).unwrap();
    writer.flush().unwrap();
    for budget in 0..=4 {
        let result = same_as_full_scan(&mut writer, [-32, 0, 0], [32, 0, 0], budget);
        if budget < 3 { assert_eq!(result, Err("region exceeds point budget".into())); }
        else { assert_eq!(result.unwrap().len(), 3); }
    }
    for axis in 0..3 {
        let mut min = [0; 3];
        min[axis] = 1;
        assert_eq!(same_as_full_scan(&mut writer, min, [0; 3], 0), Err("reversed region".into()));
    }
}

#[test]
fn region_extreme_coordinates_need_no_overflowing_bound_arithmetic() {
    let mut writer = writer();
    writer.observe([[i32::MIN; 3], [i32::MAX; 3]], 1).unwrap();
    writer.flush().unwrap();
    assert_eq!(same_as_full_scan(&mut writer, [i32::MIN; 3], [i32::MAX; 3], 2).unwrap().len(), 2);
    for point in [[i32::MIN; 3], [i32::MAX; 3]] {
        assert_eq!(same_as_full_scan(&mut writer, point, point, 1).unwrap(),
            vec![(point, Evidence { observed_ns: 1, hits: 1 })]);
    }
}

#[test]
fn region_expanding_and_moving_window_sees_new_dirty_and_persisted_edges() {
    let mut writer = writer();
    writer.observe([[-64, 0, 0], [-32, 0, 0], [0, 0, 0], [32, 0, 0], [64, 0, 0]], 1).unwrap();
    writer.flush().unwrap();
    writer.observe([[96, 0, 0], [0, 0, 0]], 2).unwrap();
    for (min, max, count) in [
        (0, 0, 1), (-1, 31, 1), (-32, 32, 3), (-64, 96, 6),
        (32, 96, 3), (97, 128, 0), (-64, -33, 1),
    ] {
        assert_eq!(same_as_full_scan(&mut writer, [min, 0, 0], [max, 0, 0], 6).unwrap().len(), count);
    }
}

#[test]
fn region_partial_flush_keeps_dirty_visibility_and_original_evidence() {
    let mut writer = writer();
    writer.observe([[-32, 0, 0], [32, 0, 0]], 1).unwrap();
    writer.repository.fail_put_after(Some(1));
    assert!(writer.flush().is_err());
    assert_eq!(writer.pending.len(), 2);
    let result = same_as_full_scan(&mut writer, [-32, 0, 0], [32, 0, 0], 2).unwrap();
    assert_eq!(result.iter().map(|(key, _)| *key).collect::<Vec<_>>(), vec![[-32, 0, 0], [32, 0, 0]]);
    writer.repository.fail_put_after(None);
    writer.flush().unwrap();
    assert_eq!(same_as_full_scan(&mut writer, [-32, 0, 0], [32, 0, 0], 2).unwrap(), result);
}

#[test]
fn region_selector_visits_x_slab_not_the_full_directory() {
    // Selector-only cardinalities, not elapsed-time measurements. No block
    // files are needed to prove this property of the existing sorted keys.
    let directory: BTreeMap<Key, ()> = (-2048..2048)
        .flat_map(|x| (0..32).map(move |y| ([x, y, 0], ())))
        .collect();
    let low = [0, 0, 0];
    let high = [38, 31, 0];
    let full: BTreeSet<_> = directory.keys().copied()
        .filter(|key| (0..3).all(|i| key[i] >= low[i] && key[i] <= high[i])).collect();
    let slab: Vec<_> = directory
        .range([low[0], i32::MIN, i32::MIN]..=[high[0], i32::MAX, i32::MAX])
        .map(|(&key, _)| key).collect();
    let selected: BTreeSet<_> = slab.iter().copied()
        .filter(|key| (1..3).all(|i| key[i] >= low[i] && key[i] <= high[i])).collect();
    assert_eq!(directory.len(), 131_072);
    assert_eq!(slab.len(), 39 * 32);
    assert_eq!(selected, full);

    // Explicit adverse orientation: X pruning is not a 3D spatial index.
    let y_corridor: BTreeMap<Key, ()> = (0..1024).map(|y| ([0, y, 0], ())).collect();
    assert_eq!(y_corridor.range([0, i32::MIN, i32::MIN]..=[0, i32::MAX, i32::MAX]).count(), 1024);
    assert_eq!(y_corridor.keys().filter(|key| key[1] <= 31).count(), 32);
}

#[test]
fn region_borrowed_dirty_output_stays_owned_after_mutation_and_flush() {
    let mut writer = writer();
    writer.observe([[0, 0, 0], [1, 0, 0]], 1).unwrap();
    let before = same_as_full_scan(&mut writer, [0; 3], [1, 0, 0], 2).unwrap();
    writer.observe([[0, 0, 0]], 2).unwrap();
    writer.remove([[1, 0, 0]]).unwrap();
    writer.flush().unwrap();
    assert_eq!(before, vec![
        ([0, 0, 0], Evidence { observed_ns: 1, hits: 1 }),
        ([1, 0, 0], Evidence { observed_ns: 1, hits: 1 }),
    ]);
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [1, 0, 0], 2).unwrap(),
        vec![([0, 0, 0], Evidence { observed_ns: 2, hits: 2 })]);
}

#[test]
fn region_borrowed_dirty_keeps_budget_and_persisted_error_order() {
    let mut writer = writer();
    writer.observe([[0, 0, 0], [1, 0, 0]], 1).unwrap();
    // Unit-only fault injection: a later persisted reference is unreadable.
    // This never changes production identity validation or admission rules.
    writer.blocks.insert([1, 0, 0], BlockRef {
        key: [1, 0, 0], object: ObjectIdentity::Assigned(ObjectId([0; 16])),
        block_revision: 1, geometry_revision: 1, points: 1,
    });
    for budget in [0, 1] {
        assert_eq!(same_as_full_scan(&mut writer, [0; 3], [32, 0, 0], budget),
            Err("region exceeds point budget".into()));
    }
    // Existing behavior loads a selected persisted block before checking the
    // next point's output budget. Do not add a pre-read budget shortcut.
    for budget in [2, 3] {
        assert_eq!(same_as_full_scan(&mut writer, [0; 3], [32, 0, 0], budget),
            Err("zero object identity".into()));
    }
    // Dirty still shadows that broken persisted reference, even when empty.
    writer.dirty.insert([1, 0, 0], Dirty {
        block: Arc::new(Block::new()), layers: GroundRecords::empty(), layer_bytes: 0, changed: true,
        original: Original { block: Arc::new(Block::new()), layers: GroundRecords::empty(), bytes: 0 },
    });
    assert_eq!(same_as_full_scan(&mut writer, [0; 3], [32, 0, 0], 2).unwrap().len(), 2);
}

#[test]
fn local_blocks_read_only_scattered_intersections_not_the_x_slab() {
    let mut writer = writer();
    // 所有远端块具有同一个X，旧region的目录选择会访问整条Y走廊。
    writer.observe((-2048..2048).map(|y| [0, y * 32, 0]), 1).unwrap();
    writer.flush().unwrap();
    writer.reader = RegionReader::new(writer.repository.clone(), 0);
    let before = writer.repository.counts().0;
    let mut selected = BTreeSet::new();
    let mut lookups = 0;
    for y in [-2048 * 32, 2047 * 32, -2048 * 32] {
        lookups += writer.collect_intersecting_blocks([0, y, 0], [0, y, 0], &mut selected).unwrap();
    }
    assert_eq!(lookups, 3, "按实际坐标查询，重复区域也只各查一个位置");
    assert_eq!(selected.len(), 2);
    let (actual, reads) = writer.matching_keys_in_blocks(&selected, |_| true).unwrap();
    assert_eq!(reads, 2, "分散空单元之间的4094个块不能读取");
    assert_eq!(writer.repository.counts().0 - before, 2, "实际磁盘读取与所选块一致");
    assert_eq!(actual, vec![[0, -2048 * 32, 0], [0, 2047 * 32, 0]]);
}

#[test]
fn local_blocks_preserve_negative_boundaries_dirty_overrides_and_exact_predicate() {
    let mut writer = writer();
    writer.observe([[-33, -1, -1], [-32, -1, -1], [-1; 3], [0; 3], [31; 3], [32; 3]], 1).unwrap();
    writer.flush().unwrap();
    writer.remove([[-32, -1, -1], [32; 3]]).unwrap();
    writer.observe([[1, 0, 0]], 2).unwrap();
    let mut selected = BTreeSet::new();
    assert_eq!(writer.collect_intersecting_blocks([-1; 3], [32; 3], &mut selected).unwrap(), 27);
    assert_eq!(selected, BTreeSet::from([[-1; 3], [0; 3]]), "空脏块必须遮盖原持久引用");
    let exact = |key: Key| key == [-1; 3] || key == [1, 0, 0];
    let (actual, reads) = writer.matching_keys_in_blocks(&selected, exact).unwrap();
    assert_eq!(reads, 2);
    let expected = writer.region([-1; 3], [32; 3], 10).unwrap().into_iter()
        .filter_map(|(key, _)| exact(key).then_some(key)).collect::<Vec<_>>();
    assert_eq!(actual, expected);
    assert_eq!(writer.collect_intersecting_blocks([1, 0, 0], [0; 3], &mut selected), Err("reversed region".into()));
    for key in [[i32::MIN; 3], [i32::MAX; 3]] {
        writer.observe([key], 3).unwrap();
        let mut edge = BTreeSet::new();
        assert_eq!(writer.collect_intersecting_blocks(key, key, &mut edge).unwrap(), 1);
        assert_eq!(writer.matching_keys_in_blocks(&edge, |_| true).unwrap(), (vec![key], 1));
    }
}

#[test]
fn local_revoke_ground_maintenance_reads_only_the_affected_block_column() {
    let mut writer = writer();
    writer.observe((0..4096).map(|y| [0, y * 32, 0]).chain([[0, 0, 33]]), 20).unwrap();
    let column = heightfield_observations::HeightfieldColumn::from_counts([0, 0], 0.05,
        [0, 33].into_iter().map(|z| (z, [1, 0, 0, 0, 0, 0, 0, 0])).collect()).unwrap().unwrap();
    let ground = ColumnGroundRecords { classified_empty: false,
        heightfields: vec![HeightfieldSurface { column, samples: vec![] }] };
    writer.replace_ground_columns([([0, 0], &ground)]).unwrap();
    writer.flush().unwrap();
    assert_eq!(writer.blocks.len(), 4097);
    writer.reader = RegionReader::new(writer.repository.clone(), 0);
    let before = writer.repository.counts().0;
    writer.remove([[0, 0, 0]]).unwrap();
    let reads = writer.repository.counts().0 - before;
    assert_eq!(reads, 3, "版本检查读目标块一次，地面柱仅再读本XY的两个高度块");
    let retained = writer.ground_column([0, 0]).unwrap();
    let members: Vec<_> = retained.heightfields.iter().flat_map(|surface| surface.column.members())
        .map(|(z, _)| z).collect();
    assert_eq!(members, [33]);
    println!("FINE_REVOKE_GROUND blocks=4097 cold_object_reads={reads} unrelated_xy_reads=0");
}
