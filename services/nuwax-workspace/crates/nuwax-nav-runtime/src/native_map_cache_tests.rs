//! 对相同输入逐点比较缓存与原全量重建，计时也包含输入转换与原生地图装配。
use super::*;
use nuwax_nav_pct_native::{Planner, Route};
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
                reference.plan(start, goal)
            };
            complete += usize::from(expected.is_ok());
            same_route(cache.planner().plan(start, goal), expected);
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
                reference.plan_progressive(start, [9.1, 1.1, offset + 0.01], None)
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
                    .plan_progressive(start, [9.1, 1.1, offset + 0.01], None),
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
            let expected = reference.plan(start, goal);
            assert!(expected.is_ok());
            same_route(cache.planner().plan(start, goal), expected);
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
    cache.planner().plan(start, goal).unwrap();
    for (label, base) in [("近期障碍变化", &initial), ("地图变化32列", &changed)] {
        let mut old_ms = Vec::new();
        let mut new_ms = Vec::new();
        for run in 0..7 {
            cache.update(&initial, 0.4, &[]).unwrap();
            cache.planner().plan(start, goal).unwrap();
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
            let actual = cache.planner().plan(start, goal);
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
