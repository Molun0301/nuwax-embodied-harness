use super::*;
use std::collections::{BTreeMap, BTreeSet};

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
/// 全部前沿簇（含未达标的）：成员格与开口标记 → 跨度、开口宽度、是否达标；同时核对索引内部一致。
#[allow(clippy::type_complexity)]
fn partition(map: &SurfaceSnapshot) -> BTreeMap<Vec<(VoxelKey, bool)>, (u64, u64, bool)> {
    let index = map.frontier_index(&[]);
    assert_eq!(index.cells.len(), index.clusters.values().map(|c| c.cells.len()).sum::<usize>());
    index.clusters.iter().map(|(id, cluster)| {
        let members: Vec<_> = cluster.cells.iter().map(|cell| {
            assert_eq!(index.cells[cell].cluster, *id);
            assert_eq!(index.gains.contains_key(cell), cluster.candidate, "收益只记在达标簇的格上");
            (*cell, index.cells[cell].opening)
        }).collect();
        (members, (cluster.span_m.to_bits(), cluster.opening_m.to_bits(), cluster.candidate))
    }).collect()
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
        assert_eq!(partition(&next), partition(&full));
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
    assert_eq!(partition(&next), partition(&full));
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
    assert_eq!(partition(&result.snapshot), partition(&full));
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
        assert_eq!(partition(&next), partition(&full));
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
    assert_eq!(partition(&next), partition(&full));
    assert_eq!(
        next.frontier_cache_stats().unwrap().evaluated_cells,
        input.len()
    );
}

#[test]
fn separated_dirty_blocks_match_full_rebuild_without_recomputing_between_them() {
    let mut input: BTreeSet<_> = [0, 30].into_iter()
        .flat_map(|z| (-64..384).flat_map(move |x| (-16..16).map(move |y| (x, y, z))))
        .collect();
    let mut builder = SurfaceMapBuilder::new(config());
    let first = builder.build(&points(&input), 0.1).unwrap().snapshot;
    first.frontier_index(&[]);
    let full_work = first.frontier_cache_stats().unwrap().evaluated_cells;
    for insert in [false, true] {
        let mut last = None;
        for cell in [(-50, 0, 0), (360, 0, 30)] {
            if insert { input.insert(cell); } else { input.remove(&cell); }
            last = Some(builder.build(&points(&input), 0.1).unwrap().snapshot);
        }
        let updated = last.unwrap();
        let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
        assert_eq!(updated.frontier_index(&[]).gains, full.frontier_index(&[]).gains);
        assert_eq!(partition(&updated), partition(&full));
        let stats = updated.frontier_cache_stats().unwrap();
        assert!(stats.incremental && stats.evaluated_cells < full_work / 3, "{stats:?}");
        assert!(stats.retained_frontiers > 0);
    }
}

/// `width`×`depth`格地面，y=`depth`一排1米高的墙，`gaps`内的墙位没有任何点（墙外未知）；
/// `partition`给出隔墙所在列与门洞（门洞有地面）。探索边界挡住左、右、下三边，向上越过墙。
fn walled(width: i32, depth: i32, gaps: &[std::ops::Range<i32>], partition: Option<(i32, std::ops::Range<i32>)>)
    -> (SurfaceSnapshot, Vec<[f64; 2]>) {
    let mut input = BTreeSet::new();
    let wall = |input: &mut BTreeSet<VoxelKey>, x: i32, y: i32| input.extend((0..10).map(|z| (x, y, z)));
    for x in 0..width {
        for y in 0..depth {
            if partition.as_ref().is_some_and(|(column, door)| x == *column && !door.contains(&y)) {
                wall(&mut input, x, y);
            } else {
                input.insert((x, y, 0));
            }
        }
        if !gaps.iter().any(|gap| gap.contains(&x)) {
            wall(&mut input, x, depth);
        }
    }
    let (right, top) = (f64::from(width) * 0.1 - 0.3, f64::from(depth + 10) * 0.1);
    let boundary = vec![[0.3, 0.3], [right, 0.3], [right, top], [0.3, top]];
    (SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap(), boundary)
}

/// 地面层（z=0）的达标簇；墙顶另成簇，不在其中。
fn floor_clusters(map: &SurfaceSnapshot, boundary: &[[f64; 2]]) -> Vec<SurfaceFrontierCluster> {
    map.frontier_clusters(boundary).unwrap().into_iter()
        .filter(|cluster| cluster.frontiers.iter().all(|f| f.cell.2 == 0)).collect()
}

#[test]
fn opening_is_one_cluster_with_gain_proportional_to_width() {
    let mut gains = Vec::new();
    for gap in [10, 20] {
        let (map, boundary) = walled(60, 30, &[20..20 + gap], None);
        let floor = floor_clusters(&map, &boundary);
        assert_eq!(floor.len(), 1, "{gap}格开口应是一个簇");
        let cluster = &floor[0];
        let depth = map.frontier_lookahead();
        assert_eq!(cluster.frontiers.len(), (gap * depth) as usize, "开口前一条lookahead格深的带");
        assert!((cluster.opening_m - f64::from(gap) * 0.1).abs() < 1e-6, "{}", cluster.opening_m);
        assert!((cluster.span_m - f64::from(gap - 1).hypot(f64::from(depth - 1)) * 0.1).abs() < 1e-6);
        assert!((cluster.gain - cluster.frontiers.len() as f64 * 0.01).abs() < 1e-6, "收益 = 格数 × 格面积");
        assert!(cluster.frontiers.iter().all(|f| f.gain == cluster.gain));
        gains.push(cluster.gain);
    }
    assert!((gains[1] / gains[0] - 2.0).abs() < 1e-9, "收益与开口宽度成正比：{gains:?}");
}

#[test]
fn gap_narrower_than_body_is_not_a_candidate_and_exact_body_width_is() {
    // 测试配置没有外形：机体宽度 = 2 × wall_clearance_m = 0.2米。
    for (gap, candidate) in [(1, false), (2, true)] {
        let (map, boundary) = walled(60, 30, &[30..30 + gap], None);
        let index = map.frontier_index(&boundary);
        let floor: Vec<_> = index.clusters.values().filter(|c| c.cells.iter().all(|cell| cell.2 == 0)).collect();
        assert_eq!(floor.len(), 1);
        assert!((floor[0].opening_m - f64::from(gap) * 0.1).abs() < 1e-6, "{}", floor[0].opening_m);
        // 一格缝前的带按全部格算跨度已不小于机体宽度，按开口宽度判断才滤得掉。
        assert!(floor[0].span_m + 1e-6 >= 0.2);
        assert_eq!(floor[0].candidate, candidate, "{gap}格缝");
        assert_eq!(floor_clusters(&map, &boundary).len(), usize::from(candidate));
        assert!(floor[0].cells.iter().all(|cell| index.gains.contains_key(cell) == candidate));
    }
}

/// Go2式外形（0.7×0.31米）的6米见方地面，去掉`missing`格；边界挡住地图四边。
fn go2_floor(missing: &[(i32, i32)]) -> SurfaceSnapshot {
    let input: BTreeSet<_> = (0..60).flat_map(|x| (0..60).map(move |y| (x, y, 0)))
        .filter(|&(x, y, _)| !missing.contains(&(x, y))).collect();
    let mut config = config();
    config.wall_clearance_m = 0.0;
    let body = SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.35, -0.155], [0.35, -0.155], [0.35, 0.155], [-0.35, 0.155]],
        z_bounds_m: [-0.15, 0.15],
        body_to_support_z_m: 0.3,
        max_step_up_m: 0.15,
        max_step_down_m: 0.15,
        max_cross_slope_deg: 18.0,
        support_uncertainty_m: 0.0,
    };
    SurfaceSnapshot::from_points(&points(&input), config, 0.1).unwrap().with_body_envelope(body).unwrap()
}

#[test]
fn single_missing_cell_cross_is_not_a_candidate_for_a_go2_body_and_its_frontier_counts_as_vanished() {
    let boundary = [[0.3, 0.3], [5.7, 0.3], [5.7, 5.7], [0.3, 5.7]];
    let single = go2_floor(&[(30, 30)]);
    let block = go2_floor(&[(30, 30), (31, 30), (30, 31), (31, 31)]);
    assert!((single.frontier_body_width() - 0.31).abs() < 1e-9);
    // 单格缺测：前沿是十字形簇，开口线是上下左右4格，开口宽度 0.2+0.1=0.3米 < 0.31米。
    let index = single.frontier_index(&boundary);
    assert_eq!(index.clusters.len(), 1);
    let cross = index.clusters.values().next().unwrap();
    assert_eq!(cross.cells.len(), 4 * single.frontier_lookahead() as usize);
    assert!(cross.span_m > 1.0 && (cross.opening_m - 0.3).abs() < 1e-6, "{} {}", cross.span_m, cross.opening_m);
    assert!(!cross.candidate && single.frontier_clusters(&boundary).unwrap().is_empty());
    // 2×2缺测：开口宽度约0.42米，是候选。
    let clusters = block.frontier_clusters(&boundary).unwrap();
    assert_eq!(clusters.len(), 1);
    assert!(clusters[0].opening_m > 0.4);
    // 途中换目标：所选格附近只剩未达标的簇时视为所选簇已缩到最小尺寸以下。
    let cell = [28, 30, 0];
    assert!(block.frontier_near(cell, 0.75, 0.3, &boundary).unwrap());
    assert!(!single.frontier_near(cell, 0.75, 0.3, &boundary).unwrap());
}

#[test]
fn two_rooms_each_with_an_opening_are_two_clusters() {
    let (map, boundary) = walled(80, 30, &[10..20, 55..65], Some((40, 5..15)));
    let floor = floor_clusters(&map, &boundary);
    assert_eq!(floor.len(), 2);
    for (cluster, room) in floor.iter().zip([0..40, 41..80]) {
        assert!(cluster.frontiers.iter().all(|f| room.contains(&f.cell.0)), "每个房间的开口一个簇");
    }
}

#[test]
fn incremental_clusters_merge_and_split_like_full_rebuild() {
    let mut input = cells();
    let mut builder = SurfaceMapBuilder::new(config());
    builder.build(&points(&input), 0.1).unwrap().snapshot.frontier_index(&[]);
    // 两个单格缺测相隔8格，各自一个十字形簇；中间再缺一格时连成一个簇，补回后又分开。
    let holes = |map: &SurfaceSnapshot| map.frontier_index(&[]).clusters.values()
        .filter(|c| c.cells.iter().all(|cell| cell.2 == 0 && (10..40).contains(&cell.1))).count();
    for (cell, insert, expected) in [
        ((40, 25, 0), false, 1),
        ((48, 25, 0), false, 2),
        ((44, 25, 0), false, 1),
        ((44, 25, 0), true, 2),
        ((40, 25, 0), true, 1),
    ] {
        if insert { input.insert(cell); } else { input.remove(&cell); }
        let next = builder.build(&points(&input), 0.1).unwrap().snapshot;
        let full = SurfaceSnapshot::from_points(&points(&input), config(), 0.1).unwrap();
        assert_eq!(partition(&next), partition(&full), "{cell:?} {insert}");
        assert_eq!(holes(&next), expected, "{cell:?} {insert}");
        let stats = next.frontier_cache_stats().unwrap();
        assert!(stats.incremental && stats.reclustered_cells * 10 < stats.frontier_count, "{stats:?}");
    }
}
