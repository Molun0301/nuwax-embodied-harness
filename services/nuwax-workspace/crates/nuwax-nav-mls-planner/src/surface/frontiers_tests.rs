use super::*;
use std::collections::BTreeSet;

fn config() -> Config {
    Config {
        voxel_size: 0.1,
        robot_height: 0.45,
        surface_closing_radius: 0.0,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.1,
        wall_buffer_weight: 0.0,
        step_threshold_m: 0.2,
        min_step_tread_depth_m: 0.2,
        world_frame: "map".into(),
        node_spacing_m: 0.3,
        max_slope_deg: 25.0,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    }
}
fn cells() -> BTreeSet<VoxelKey> {
    [0, 30]
        .into_iter()
        .flat_map(|z| (0..100).flat_map(move |x| (0..50).map(move |y| (x, y, z))))
        .collect()
}
fn points(cells: &BTreeSet<VoxelKey>) -> Vec<(f32, f32, f32)> {
    cells
        .iter()
        .map(|&(x, y, z)| {
            (
                (x as f32 + 0.5) * 0.1,
                (y as f32 + 0.5) * 0.1,
                (z as f32 + 0.5) * 0.1,
            )
        })
        .collect()
}
fn signature(map: &SurfaceSnapshot, boundary: &[[f64; 2]]) -> Vec<(VoxelKey, f64)> {
    map.reachable_frontiers([1.05, 1.05, 0.1], boundary)
        .unwrap()
        .iter()
        .map(|f| (f.cell, f.gain))
        .collect()
}
#[test]
fn same_snapshot_reuses_geometry_for_other_starts_but_rechecks_layer_reachability() {
    let map = SurfaceSnapshot::from_points(&points(&cells()), config(), 0.1).unwrap();
    let lower = map.reachable_frontiers([1.05, 1.05, 0.1], &[]).unwrap();
    let index = map.frontier_index(&[]);
    let upper = map.reachable_frontiers([1.05, 1.05, 3.1], &[]).unwrap();
    assert!(Arc::ptr_eq(&index, &map.frontier_index(&[])));
    assert!(lower.iter().all(|f| f.cell.2 == 0));
    assert!(upper.iter().all(|f| f.cell.2 == 30));
    assert!(!lower.is_empty() && !upper.is_empty());
}
#[test]
fn local_frontier_updates_match_full_rebuild_and_preserve_old_snapshot() {
    let mut input = cells();
    let mut builder = SurfaceMapBuilder::new(config());
    let old = builder.build(&points(&input), 0.1).unwrap().snapshot;
    let original = signature(&old, &[]);
    let old_index = old.frontier_index(&[]);
    let count = old.frontier_cache_stats().unwrap().evaluated_cells;
    for (cell, insert) in [
        ((50, 25, 0), false),
        ((50, 25, 0), true),
        ((70, 25, 30), false),
        ((70, 25, 30), true),
    ] {
        if insert {
            input.insert(cell);
        } else {
            input.remove(&cell);
        }
        let next = builder.build(&points(&input), 0.1).unwrap().snapshot;
        assert!(
            next.frontier_cache_stats().is_none(),
            "更新时只合并失效范围，不提前计算前沿"
        );
        let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
        assert_eq!(signature(&next, &[]), signature(&full, &[]));
        let stats = next.frontier_cache_stats().unwrap();
        assert!(
            stats.incremental && stats.evaluated_cells < count / 2,
            "{stats:?}"
        );
        assert!(stats.retained_frontiers > 0);
    }
    assert_eq!(signature(&old, &[]), original);
    assert!(Arc::ptr_eq(&old_index, &old.frontier_index(&[])));
}
#[test]
fn deferred_updates_accumulate_all_changes_without_retaining_snapshot_chains() {
    let mut input = cells();
    let mut builder = SurfaceMapBuilder::new(config());
    let first = builder.build(&points(&input), 0.1).unwrap().snapshot;
    signature(&first, &[]);
    let weak = Arc::downgrade(&first);
    drop(first);
    let mut last = None;
    for x in [30, 50, 70] {
        input.remove(&(x, 25, 0));
        let next = builder.build(&points(&input), 0.1).unwrap().snapshot;
        assert!(next.frontier_cache_stats().is_none());
        last = Some(next);
    }
    assert!(
        weak.upgrade().is_none(),
        "前沿只保留索引，不能把旧快照串成链"
    );
    let next = last.unwrap();
    let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
    assert_eq!(signature(&next, &[]), signature(&full, &[]));
}
#[test]
fn occupied_evidence_changes_frontier_gain_even_when_support_graph_is_identical() {
    let mut input = cells();
    input.insert((100, 25, 4));
    input.insert((100, 25, 8));
    let mut builder = SurfaceMapBuilder::new(config());
    let previous = builder.build(&points(&input), 0.1).unwrap().snapshot;
    let before = signature(&previous, &[]);
    input.remove(&(100, 25, 4));
    let result = builder.build(&points(&input), 0.1).unwrap();
    assert_eq!(result.stats.changed_surface_cells, 0);
    let after = signature(&result.snapshot, &[]);
    let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
    assert_eq!(after, signature(&full, &[]));
    assert_ne!(before, after, "遮挡证据变化须改变收益，不能只看支撑单元");
    assert!(result.snapshot.frontier_cache_stats().unwrap().incremental);
}
#[test]
fn boundary_and_virtual_obstacle_changes_discard_incompatible_frontier_indexes() {
    let input = cells();
    let raw = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
    signature(&raw, &[]);
    let boundary = [[0.2, 0.2], [9.5, 0.2], [9.5, 4.5], [0.2, 4.5]];
    assert!(signature(&raw, &boundary).is_empty());
    assert!(!raw.frontier_cache_stats().unwrap().incremental);
    assert!(!signature(&raw, &[]).is_empty());
    let empty = vec![false; 100 * 50];
    let old = raw
        .with_virtual_obstacles([0.0, 0.0], 0.1, 100, 50, &empty)
        .unwrap();
    signature(&old, &[]);
    let mut blocked = empty.clone();
    for y in 20..30 {
        for x in 40..60 {
            blocked[y * 100 + x] = true;
        }
    }
    let changed = raw
        .with_virtual_obstacles([0.0, 0.0], 0.1, 100, 50, &blocked)
        .unwrap();
    changed.inherit_frontier_cache(&old, &[]);
    assert!(changed.frontier_cache_stats().is_none());
    let fresh = raw
        .with_virtual_obstacles([0.0, 0.0], 0.1, 100, 50, &blocked)
        .unwrap();
    assert_eq!(signature(&changed, &[]), signature(&fresh, &[]));
    assert!(!changed.frontier_cache_stats().unwrap().incremental);
}

#[test]
fn local_bounds_visit_negative_columns_all_layers_and_reused_slots() {
    let mut input: BTreeSet<_> = [-30, 0, 30]
        .into_iter()
        .flat_map(|z| (-40..40).flat_map(move |x| (-30..30).map(move |y| (x, y, z))))
        .collect();
    let mut builder = SurfaceMapBuilder::new(config());
    let first = builder.build(&points(&input), 0.1).unwrap().snapshot;
    let original = first.frontier_index(&[]);
    for (cell, insert) in [
        ((-20, -10, 0), false),
        ((-20, -10, -30), false),
        ((-20, -10, 30), false),
        ((-20, -10, 0), true),
        ((-20, -10, -30), true),
        ((-20, -10, 30), true),
        ((-21, -10, 30), false),
    ] {
        if insert {
            input.insert(cell);
        } else {
            input.remove(&cell);
        }
        let next = builder.build(&points(&input), 0.1).unwrap().snapshot;
        let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
        let actual = next.frontier_index(&[]);
        assert!(next.frontier_cache_stats().unwrap().incremental);
        assert_eq!(actual.gains, full.frontier_index(&[]).gains);
        assert!(Arc::ptr_eq(&original, &first.frontier_index(&[])));
    }
}

#[test]
fn sparse_widely_separated_dirty_columns_match_full_frontiers() {
    let mut input: BTreeSet<_> = [-300, 300]
        .into_iter()
        .flat_map(|offset| {
            (0..8).flat_map(move |x| (0..8).map(move |y| (offset + x, offset + y, 0)))
        })
        .collect();
    let mut builder = SurfaceMapBuilder::new(config());
    let first = builder.build(&points(&input), 0.1).unwrap().snapshot;
    first.prepare_frontier_geometry(&[]).unwrap();
    input.remove(&(-296, -296, 0));
    let deferred = builder.build(&points(&input), 0.1).unwrap().snapshot;
    assert!(deferred.frontier_cache_stats().is_none());
    input.remove(&(304, 304, 0));
    let next = builder.build(&points(&input), 0.1).unwrap().snapshot;
    let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
    assert_eq!(
        next.frontier_index(&[]).gains,
        full.frontier_index(&[]).gains
    );
    assert_eq!(
        next.frontier_cache_stats().unwrap().evaluated_cells,
        input.len()
    );
}
