//! 对相同输入逐点比较缓存与原全量重建，计时也包含输入转换与原生地图装配。
use super::*;
use nuwax_nav_pct_native::{Planner, Route};
use std::sync::Arc;
use std::time::Instant;
#[path = "native_map_baseline_native.rs"]
mod baseline_native;

fn full(
    map: &SurfaceSnapshot,
    source: PctTerrainSource,
    boundary: &[[f64; 2]],
    offset: f64,
) -> Planner {
    let (config, cells, bands) = match source {
        PctTerrainSource::Tomography => {
            full_reference::full_tomography_input(map, offset, boundary)
        }
        PctTerrainSource::Mls => pct_input(map, offset, boundary),
    }
    .unwrap();
    Planner::from_cells(config, &cells, &bands).unwrap()
}

fn same_route(actual: Result<Route, String>, expected: Result<Route, String>) {
    match (actual, expected) {
        (Ok(a), Ok(b)) => {
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.path, b.path);
        }
        (Err(a), Err(b)) => assert_eq!(a, b),
        (a, b) => panic!("缓存与全量结果不同：{a:?} / {b:?}"),
    }
}

fn scene(side: i32, q: f32, changed: usize) -> SurfaceSnapshot {
    let (_, base) = crate::native_test_helpers::fixture();
    let mut config = base.config().clone();
    config.voxel_size = q;
    config.wall_clearance_m = 0.;
    config.surface_closing_radius = 0.;
    config.min_step_tread_depth_m = q * 2.;
    let points: Vec<_> = (0..side)
        .flat_map(|x| {
            (0..side).map(move |y| {
                let index = (x * side + y) as usize;
                (
                    (x as f32 + 0.5) * q,
                    (y as f32 + 0.5) * q,
                    if index < changed { 0.02 } else { 0.01 },
                )
            })
        })
        .collect();
    SurfaceSnapshot::from_points(&points, config, q)
        .unwrap()
        .with_body_envelope(base.body_envelope().unwrap().clone())
        .unwrap()
}

#[test]
fn cached_routes_equal_full_rebuild_for_map_recent_ground_masks_and_progressive_search() {
    check_sequence(None);
}

#[test]
#[ignore = "显式加载ba90c30d原生桥进行跨版本逐点比较"]
fn n1_original_native_equivalence() {
    check_sequence(Some(&baseline_native::Baseline::open()));
}

fn check_sequence(baseline: Option<&baseline_native::Baseline>) {
    for source in [PctTerrainSource::Tomography, PctTerrainSource::Mls] {
        let mut cache = CachedPctPlanner::new(source);
        let mut complete = 0;
        let mut approach = 0;
        let mut boundary_rejected = 0;
        for step in 0..10 {
            let map = scene(70, 0.1, step * 3);
            let recent_height = if step == 4 { -0.71 } else { 0.015 };
            let map = map
                .with_recent_obstacle_points(
                    step as u64 + 1,
                    step as u64 + 1,
                    0,
                    &[0],
                    [
                        (0, [1.1, 3.1, recent_height]),
                        (1, [2.1 + step as f64 * 0.2, 2.1, 0.45]),
                    ],
                )
                .unwrap();
            let map = if (3..7).contains(&step) {
                let mut mask = vec![false; 70 * 70];
                mask[24 * 70 + 24] = true;
                map.with_virtual_obstacles([0., 0.], 0.1, 70, 70, &mask)
                    .unwrap()
            } else {
                map
            };
            let boundary = if step >= 8 {
                vec![[0., 0.], [7., 0.], [7., 7.], [0., 7.]]
            } else {
                vec![]
            };
            let offset = if step == 9 { 0.45 } else { 0.4 };
            cache.update(&map, offset, &boundary).unwrap();
            let mut reference = full(&map, source, &boundary, offset);
            let start = [1.1, 1.1, offset + 0.01];
            let goal = [5.1, 1.1, offset + 0.01];
            let expected = if let Some(baseline) = baseline {
                let (config, cells, bands) = match source {
                    PctTerrainSource::Tomography => {
                        full_reference::full_tomography_input(&map, offset, &boundary)
                    }
                    PctTerrainSource::Mls => pct_input(&map, offset, &boundary),
                }
                .unwrap();
                baseline.plan(&config, &cells, &bands, start, goal, false)
            } else {
                reference.plan(start, None, goal)
            };
            complete += usize::from(expected.is_ok());
            same_route(cache.planner().plan(start, None, goal), expected);
            let expected = if let Some(baseline) = baseline {
                let (config, cells, bands) = match source {
                    PctTerrainSource::Tomography => {
                        full_reference::full_tomography_input(&map, offset, &boundary)
                    }
                    PctTerrainSource::Mls => pct_input(&map, offset, &boundary),
                }
                .unwrap();
                baseline.plan(
                    &config,
                    &cells,
                    &bands,
                    start,
                    [9.1, 1.1, offset + 0.01],
                    true,
                )
            } else {
                reference.plan_progressive(start, None, [9.1, 1.1, offset + 0.01], None)
            };
            boundary_rejected += usize::from(step >= 8 && expected.is_err());
            approach += usize::from(
                expected
                    .as_ref()
                    .is_ok_and(|r| r.kind == nuwax_nav_pct_native::RouteKind::Approach),
            );
            same_route(
                cache
                    .planner()
                    .plan_progressive(start, None, [9.1, 1.1, offset + 0.01], None),
                expected,
            );
        }
        assert_eq!(complete, 10, "完整路线必须实际成功");
        assert_eq!(approach, 8, "边界内缺测目标须实际产生逼近路线");
        assert_eq!(boundary_rejected, 2, "目标越出显式边界时保留原拒绝结果");
    }
}

#[test]
fn cached_fine_ground_records_refresh_only_changed_geometry_and_preserve_routes() {
    for source in [PctTerrainSource::Tomography, PctTerrainSource::Mls] {
        let mut cache = CachedPctPlanner::new(source);
        for step in 0..3 {
            let map = super::tomography_tests::heightfield_floor(
                |x| {
                    if step == 1 && x > 1. && x < 1.5 {
                        0.025
                    } else {
                        0.01
                    }
                },
                &[],
                0.05,
            );
            cache.update(&map, 0.4, &[]).unwrap();
            let mut reference = full(&map, source, &[], 0.4);
            let start = [0.125, 0.025, 0.41];
            let goal = [3.875, 0.025, 0.41];
            let expected = reference.plan(start, None, goal);
            assert!(expected.is_ok());
            same_route(cache.planner().plan(start, None, goal), expected);
        }
    }
}

#[test]
fn column_cache_matches_full_tomogram_when_minimum_moves_and_columns_disappear() {
    use nuwax_pct_tomography::{ColumnCache, TomographyConfig};
    let mut cache = ColumnCache::default();
    let mut geometry = BTreeMap::new();
    for step in 0..6 {
        let points = vec![
            [0.1, 0.1, 0.01],
            [0.1, 0.1, 0.75],
            [0.3, 0.1, if step == 2 { -0.11 } else { 0.01 }],
        ];
        let points = if step == 4 { &points[..1] } else { &points[..] };
        let recent = if step == 3 {
            vec![[0.1, 0.1, -0.3]]
        } else {
            Vec::new()
        };
        cache.replace_static(points, 0.2).unwrap();
        let cfg = TomographyConfig {
            resolution: 0.2,
            point_resolution: 0.05,
            slice_h0: 0.,
            slice_dh: 0.6,
        };
        for update in cache.update(&recent, &cfg).unwrap() {
            geometry.insert(update.column, (update.cells, update.bands));
        }
        let combined: Vec<_> = points.iter().chain(&recent).copied().collect();
        let min = combined
            .iter()
            .map(|p| f64::from(p[2]))
            .fold(f64::INFINITY, f64::min);
        let expected = nuwax_pct_tomography::build(
            &combined,
            &TomographyConfig {
                slice_h0: min + cfg.slice_dh,
                ..cfg
            },
        )
        .unwrap();
        assert_eq!(
            geometry
                .values()
                .flat_map(|v| v.0.iter().copied())
                .collect::<Vec<_>>(),
            expected.cells
        );
        assert_eq!(
            geometry
                .values()
                .flat_map(|v| v.1.iter().copied())
                .collect::<Vec<_>>(),
            expected.bands
        );
    }
}

#[test]
#[ignore = "100米合成图耗时对照，显式运行"]
fn n1_global_cache_benchmark() {
    // 400×400列、0.25米分辨率，覆盖100米×100米。
    let baseline = baseline_native::Baseline::open();
    let initial = scene(400, 0.25, 0);
    let changed = scene(400, 0.25, 32);
    let start = [45.125, 45.125, 0.41];
    let goal = [50.125, 45.125, 0.41];
    let mut cache = CachedPctPlanner::new(PctTerrainSource::Tomography);
    cache.update(&initial, 0.4, &[]).unwrap();
    cache.planner().plan(start, None, goal).unwrap();
    for (label, base) in [("近期障碍变化", &initial), ("地图变化32列", &changed)] {
        let mut old_ms = Vec::new();
        let mut new_ms = Vec::new();
        for run in 0..7 {
            cache.update(&initial, 0.4, &[]).unwrap();
            cache.planner().plan(start, None, goal).unwrap();
            let map = base
                .with_recent_obstacle_points(
                    run + 1,
                    run + 1,
                    0,
                    &[],
                    [(0, [47.125 + run as f64 * 0.25, 47.125, 0.5])],
                )
                .unwrap();
            let began = Instant::now();
            let (cfg, cells, bands) =
                full_reference::full_tomography_input(&map, 0.4, &[]).unwrap();
            let expected = baseline.plan(&cfg, &cells, &bands, start, goal, false);
            let old = began.elapsed().as_secs_f64() * 1000.;
            let began = Instant::now();
            cache.update(&map, 0.4, &[]).unwrap();
            let actual = cache.planner().plan(start, None, goal);
            let new = began.elapsed().as_secs_f64() * 1000.;
            assert!(expected.is_ok());
            same_route(actual, expected);
            if run > 0 {
                old_ms.push(old);
                new_ms.push(new);
            }
        }
        old_ms.sort_by(f64::total_cmp);
        new_ms.sort_by(f64::total_cmp);
        eprintln!("N1性能 {label}：全量中位数={:.3}ms，缓存中位数={:.3}ms，全量={old_ms:?}，缓存={new_ms:?}", old_ms[3], new_ms[3]);
    }
}

/// 两份快照间逐列层析采样不同的列（测试用对照：按列重新提取并比较，不依赖表面图登记）。
fn changed_tomography_columns(old: &SurfaceSnapshot, new: &SurfaceSnapshot, offset: f64) -> Vec<[i32; 2]> {
    let (resolution, _) = new.navigation_source_columns(None);
    let config = pct_config(new, offset, &[], resolution);
    let points = |map: &SurfaceSnapshot, column: [i32; 2]| {
        let mut points = Vec::new();
        if let Some(bands) = map.navigation_source_column(column) {
            column_tomography_points(tomography_ground(map), column, &bands, resolution, &config, &mut points);
        }
        points
    };
    let columns: BTreeSet<_> = old.navigation_source_columns(None).1.map(|(column, _)| column)
        .chain(new.navigation_source_columns(None).1.map(|(column, _)| column)).collect();
    columns.into_iter().filter(|&column| points(old, column) != points(new, column)).collect()
}

#[test]
fn column_cache_partial_static_replacement_matches_full_tomogram() {
    use nuwax_pct_tomography::{ColumnCache, TomographyConfig};
    let cfg = TomographyConfig { resolution: 0.2, point_resolution: 0.05, slice_h0: 0., slice_dh: 0.6 };
    let columns_of = |points: &[[f32; 3]]| -> BTreeMap<[i32; 2], Vec<f32>> {
        let mut columns = BTreeMap::<_, Vec<f32>>::new();
        for p in points {
            columns.entry([(f64::from(p[0]) / 0.2).floor() as i32, (f64::from(p[1]) / 0.2).floor() as i32])
                .or_default().push(p[2]);
        }
        columns.values_mut().for_each(|zs| zs.sort_by(f32::total_cmp));
        columns
    };
    // (静态采样, 近期地面, 是否按列替换)
    let steps: Vec<(Vec<[f32; 3]>, Vec<[f32; 3]>, bool)> = vec![
        (vec![[0.1, 0.1, 0.01], [0.1, 0.1, 0.75], [0.3, 0.1, 0.01], [0.5, 0.1, 0.02]], vec![], false),
        // 最低点移到第二列。
        (vec![[0.1, 0.1, 0.01], [0.1, 0.1, 0.75], [0.3, 0.1, -0.11], [0.5, 0.1, 0.02]], vec![], true),
        // 最低点所在列删除，整图最低点回升；近期地面低于静态。
        (vec![[0.1, 0.1, 0.01], [0.1, 0.1, 0.75], [0.5, 0.1, 0.02]], vec![[0.1, 0.1, -0.3]], true),
        // 按列替换之后的整图替换：删除一列、新增一列。
        (vec![[0.1, 0.1, 0.01], [0.1, 0.1, 0.75], [0.7, 0.1, 0.3]], vec![], false),
        // 列内删点、相同最低点出现在两列。
        (vec![[0.1, 0.1, 0.01], [0.7, 0.1, 0.3], [0.9, 0.1, 0.01]], vec![], true),
        (vec![[0.1, 0.1, 0.4], [0.9, 0.1, 0.01]], vec![], true),
    ];
    let mut cache = ColumnCache::default();
    let mut geometry = BTreeMap::new();
    let mut previous: Vec<[f32; 3]> = Vec::new();
    for (points, recent, partial) in &steps {
        if *partial {
            let (old, new) = (columns_of(&previous), columns_of(points));
            let changed: BTreeSet<_> = old.keys().chain(new.keys()).copied()
                .filter(|key| old.get(key) != new.get(key)).collect();
            let selected: Vec<_> = points.iter().copied().filter(|p| changed.contains(&[
                (f64::from(p[0]) / 0.2).floor() as i32, (f64::from(p[1]) / 0.2).floor() as i32])).collect();
            cache.replace_static_columns(&changed, &selected, 0.2).unwrap();
        } else {
            cache.replace_static(points, 0.2).unwrap();
        }
        for update in cache.update(recent, &cfg).unwrap() {
            geometry.insert(update.column, (update.cells, update.bands));
        }
        let combined: Vec<_> = points.iter().chain(recent).copied().collect();
        let min = combined.iter().map(|p| f64::from(p[2])).fold(f64::INFINITY, f64::min);
        let expected = nuwax_pct_tomography::build(&combined, &TomographyConfig { slice_h0: min + cfg.slice_dh, ..cfg.clone() }).unwrap();
        assert_eq!(geometry.values().flat_map(|v| v.0.iter().copied()).collect::<Vec<_>>(), expected.cells);
        assert_eq!(geometry.values().flat_map(|v| v.1.iter().copied()).collect::<Vec<_>>(), expected.bands);
        previous = points.clone();
    }
}

#[test]
fn registered_static_columns_update_only_those_columns_and_match_full_rebuild() {
    let height = |step: usize| move |x: f64| if (1..4).contains(&step) && x > 1. && x < 1.5 { 0.025 } else { 0.01 };
    // 每步：地面高度变化区间与额外占据体素；含新增、删除和最低点之外的变化。
    let extras: [&[[i32; 3]]; 5] = [&[], &[[30, 2, 6]], &[[30, 2, 6], [31, -3, 9]], &[[31, -3, 9]], &[]];
    let mut cache = CachedPctPlanner::new(PctTerrainSource::Tomography);
    let mut previous: Option<SurfaceSnapshot> = None;
    let mut partial_steps = 0;
    for (step, extra) in extras.iter().enumerate() {
        let map = super::tomography_tests::heightfield_floor(height(step), extra, 0.05);
        let map = match &previous {
            None => map.with_static_columns(None, None),
            Some(old) => {
                let changed = changed_tomography_columns(old, &map, 0.4);
                partial_steps += usize::from(!changed.is_empty());
                map.with_static_columns(Some(old), Some((changed, Vec::new())))
            }
        };
        // 带独立地面记录的完整精细源按列提取；登记覆盖上一份身份时缓存只处理登记列。
        assert!(tomography_ground(&map).is_some());
        if let Some(old) = &previous {
            assert!(map.static_columns().unwrap().columns_since(old.static_columns().unwrap().id()).is_some());
        }
        cache.update(&map, 0.4, &[]).unwrap();
        let mut full = CachedPctPlanner::new(PctTerrainSource::Tomography);
        full.update(&map, 0.4, &[]).unwrap();
        assert_eq!(cache.planner().geometry(), full.planner().geometry(), "第{step}步原生几何与全量重建不同");
        let start = [0.125, 0.025, 0.41];
        let goal = [3.875, 0.025, 0.41];
        same_route(cache.planner().plan(start, None, goal), full.planner().plan(start, None, goal));
        previous = Some(map);
    }
    assert_eq!(partial_steps, 4, "每一步都应有实际变化列");
}

/// 与 N1 耗时场景同规格（400×400列、0.25米、前32列地面变化）的完整精细源：
/// 每列一条独立地面记录，变化列的地面高 5 厘米。返回两份带观测的快照与精细源登记的变化列。
fn fine_benchmark_scene(side: i32, q: f32, changed: usize) -> (SurfaceSnapshot, SurfaceSnapshot, Vec<[i32; 2]>) {
    use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
    use nuwax_map_chunks::{ColumnGroundRecords, Domain, HeightfieldSurface, Receiver, Repository, Transfer, Writer};
    let bin = f64::from(q) / 8.0;
    let mut writer = Writer::new(Repository::temporary().unwrap(), q, 64 * 1024 * 1024).unwrap();
    let domain = Domain { session: "n3-benchmark".into() };
    let mut receiver = Receiver::new(Repository::temporary().unwrap());
    let (_, base) = crate::native_test_helpers::fixture();
    let mut config = base.config().clone();
    config.voxel_size = q;
    config.wall_clearance_m = 0.;
    config.surface_closing_radius = 0.;
    config.min_step_tread_depth_m = q * 2.;
    let mut previous: Option<(Arc<nuwax_map_chunks::Manifest>, Arc<nuwax_nav_mls_planner::SurfaceObservations>)> = None;
    let mut snapshots = Vec::new();
    let mut changes = Vec::new();
    for revision in 1..=2u64 {
        let mut points = Vec::new();
        for x in 0..side {
            for y in 0..side {
                let index = (x * side + y) as usize;
                if revision == 2 && index >= changed { continue; }
                let height = if revision == 2 { 0.06 } else { 0.01 };
                let center = (height / bin).floor() as i32;
                let mut counts = BTreeMap::<i32, [u32; 8]>::new();
                for offset in -3..=3 {
                    let sub = center + offset;
                    counts.entry(sub.div_euclid(8)).or_default()[sub.rem_euclid(8) as usize] += 4 - offset.abs() as u32;
                }
                let column = HeightfieldColumn::from_counts([x, y], q, counts).unwrap().unwrap();
                for (z, _) in column.members() {
                    writer.observe([[x, y, z]], revision * 20).unwrap();
                }
                writer.replace_ground_column([x, y], &ColumnGroundRecords {
                    classified_empty: false,
                    heightfields: vec![HeightfieldSurface { column, samples: vec![] }],
                }).unwrap();
            }
        }
        for x in 0..side {
            for y in 0..side {
                let index = (x * side + y) as usize;
                let height = if index < changed && revision == 2 { 0.06 } else { 0.01 };
                points.push(((x as f32 + 0.5) * q, (y as f32 + 0.5) * q, height as f32));
            }
        }
        let root = writer.snapshot(domain.clone(), revision, revision * 20).unwrap();
        let root = receiver.receive(&Transfer { root: root.encode().unwrap(),
            repository_path: writer.repository().path().to_string_lossy().into_owned() }, &domain).unwrap();
        let (observations, changed_columns) = nuwax_nav_mls_planner::surface_chunks::observations_with_changes(
            &mut receiver, &root, previous.as_ref().map(|(root, grid)| (root.as_ref(), grid))).unwrap();
        if let Some(columns) = changed_columns { changes = columns; }
        snapshots.push(SurfaceSnapshot::from_points(&points, config.clone(), q).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap()
            .with_observations(Some(Arc::clone(&observations))));
        previous = Some((root, observations));
    }
    let changed_map = snapshots.pop().unwrap();
    (snapshots.pop().unwrap(), changed_map, changes)
}

#[test]
#[ignore = "100米合成图耗时对照（完整精细源），显式运行"]
fn n3_registered_columns_benchmark() {
    let (initial, changed, columns) = fine_benchmark_scene(400, 0.25, 32);
    assert_eq!(columns.len(), 32, "精细源登记的变化列：{columns:?}");
    let start = [45.125, 45.125, 0.41];
    let goal = [50.125, 45.125, 0.41];
    for (label, base, is_changed) in [("近期障碍变化", &initial, false), ("地图变化32列", &changed, true)] {
        // 两条路径各自完整跑一遍 N1 的循环：每轮先回到初始图并规划，再换到本场景图加一帧近期障碍后计时。
        // 带登记时登记记录按时间顺序延续：在初始图与变化图之间切换时登记那32列，否则静态未变。
        let pass = |registered: bool| {
            let mut cache = CachedPctPlanner::new(PctTerrainSource::Tomography);
            let mut last = (initial.with_static_columns(None, None), false);
            let mut follow = |map: &SurfaceSnapshot, map_changed: bool| -> SurfaceSnapshot {
                if !registered { return map.clone(); }
                let changed_columns = if last.1 != map_changed { columns.clone() } else { Vec::new() };
                last = (map.with_static_columns(Some(&last.0), Some((changed_columns, Vec::new()))), map_changed);
                last.0.clone()
            };
            let first = follow(&initial, false);
            cache.update(&first, 0.4, &[]).unwrap();
            cache.planner().plan(start, None, goal).unwrap();
            let mut samples = Vec::new();
            for run in 0..7u64 {
                let back = follow(&initial, false);
                cache.update(&back, 0.4, &[]).unwrap();
                cache.planner().plan(start, None, goal).unwrap();
                let map = follow(base, is_changed).with_recent_obstacle_points(run + 1, run + 1, 0, &[],
                    [(0, [47.125 + run as f64 * 0.25, 47.125, 0.5])]).unwrap();
                let began = Instant::now();
                cache.update(&map, 0.4, &[]).unwrap();
                let route = cache.planner().plan(start, None, goal);
                let elapsed = began.elapsed().as_secs_f64() * 1000.;
                let mut full = CachedPctPlanner::new(PctTerrainSource::Tomography);
                full.update(&map, 0.4, &[]).unwrap();
                assert_eq!(cache.planner().geometry(), full.planner().geometry());
                assert!(route.is_ok());
                same_route(route, full.planner().plan(start, None, goal));
                if run > 0 { samples.push(elapsed); }
            }
            samples.sort_by(f64::total_cmp);
            samples
        };
        let full_ms = pass(false);
        let registered_ms = pass(true);
        eprintln!("N3耗时 {label}：整图比对中位数={:.3}ms，登记变化列中位数={:.3}ms，整图比对={full_ms:?}，登记变化列={registered_ms:?}",
            full_ms[3], registered_ms[3]);
    }
}

/// 拐点扫掠的跨版本对照：窄十字原版给出经中心转向的路线、本版无路线；宽通道两版逐点相同。
#[test]
#[ignore = "显式加载ba90c30d原生桥进行跨版本对照"]
fn n3_turn_sweep_against_original_native() {
    let baseline = baseline_native::Baseline::open();
    let q = 0.05;
    let config = Config {
        resolution: q, point_resolution: 0.03, robot_height: 0.45, body_to_ground: 0.4,
        max_step: 0.16, max_step_down: 0.16, body_z_min: -0.2, body_z_max: 0.15, search_margin: 1.0,
        footprint: vec![[-0.5, -0.15], [0.5, -0.15], [0.5, 0.15], [-0.5, 0.15]],
        ..Default::default()
    };
    let scene = |free: &dyn Fn(f64, f64) -> bool| {
        let cells: Vec<_> = (-80..80).flat_map(|x| (-80..80).map(move |y| Cell { x, y, ground: 0., ceiling: f64::INFINITY })).collect();
        let bands: Vec<_> = cells.iter().filter(|c| !free((f64::from(c.x) + 0.5) * q, (f64::from(c.y) + 0.5) * q))
            .map(|c| Band { x: c.x, y: c.y, low: 0., high: 1.5 }).collect();
        (cells, bands)
    };
    let (left, up) = ([-2.5, 0.025, 0.4], [0.025, 2.5, 0.4]);
    let (cells, bands) = scene(&|x, y| y.abs() <= 0.2 || x.abs() <= 0.2);
    let original = baseline.plan(&config, &cells, &bands, left, up, false).unwrap();
    assert!(original.path.iter().any(|p| p[0].abs() <= 0.2 && p[1].abs() <= 0.2), "原版经十字中心转向");
    let mut current = Planner::from_cells(config.clone(), &cells, &bands).unwrap();
    assert!(current.plan(left, None, up).is_err());
    eprintln!("N3拐点 窄十字：原版路线点数={}，本版无路线", original.path.len());
    // 2 米宽的直角通道：转向空间充足，两版路线逐点相同（起点朝向给与不给都相同）。
    let (cells, bands) = scene(&|x, y| ((-4.0..=1.0).contains(&x) && (-1.0..=1.0).contains(&y))
        || ((-1.0..=1.0).contains(&x) && (-1.0..=4.0).contains(&y)));
    let (start, goal) = ([-3.5, 0.025, 0.4], [0.025, 3.5, 0.4]);
    let original = baseline.plan(&config, &cells, &bands, start, goal, false).unwrap();
    let mut current = Planner::from_cells(config.clone(), &cells, &bands).unwrap();
    assert_eq!(current.plan(start, None, goal).unwrap().path, original.path);
    assert_eq!(current.plan(start, Some(0.), goal).unwrap().path, original.path);
    let original = baseline.plan(&config, &cells, &bands, start, goal, true).unwrap();
    assert_eq!(current.plan_progressive(start, Some(0.), goal, None).unwrap().path, original.path);
    eprintln!("N3拐点 宽直角通道：两版路线逐点相同，点数={}", original.path.len());
}
