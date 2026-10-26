use crate::{build, build_region, Tomogram, TomographyConfig};

fn config() -> TomographyConfig {
    TomographyConfig {
        resolution: 0.25,
        slice_h0: 0.5,
        slice_dh: 0.5,
        point_resolution: 0.05,
    }
}

fn cells(map: &Tomogram) -> Vec<(i32, i32, f64, f64)> {
    map.cells
        .iter()
        .map(|cell| (cell.x, cell.y, cell.ground, cell.ceiling))
        .collect()
}

#[test]
fn exact_slice_is_ground_and_multiple_decks_keep_their_ceiling() {
    let map = build(
        &[
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.5],
            [0.1, 0.1, 1.2],
            [0.1, 0.1, 2.0],
            [0.1, 0.1, 2.0],
        ],
        &config(),
    )
    .unwrap();
    assert_eq!(
        cells(&map),
        vec![
            (0, 0, 0.5, f64::from(1.2_f32)),
            (0, 0, f64::from(1.2_f32), 2.0),
            (0, 0, 2.0, f64::INFINITY)
        ]
    );
    assert_eq!(map.bands.len(), 4);
}

#[test]
fn low_clearance_is_preserved_for_native_robot_checks() {
    let map = build(
        &[[0.1, 0.1, 0.0], [0.1, 0.1, 0.55], [0.1, 0.1, 0.65]],
        &config(),
    )
    .unwrap();
    assert_eq!(cells(&map)[0], (0, 0, 0.0, f64::from(0.55_f32)));
    assert_eq!(cells(&map).len(), 2);
    // 0.55至0.65之间没有切片，不凭空增加一个采样地面。
    assert_eq!(cells(&map)[1].2, f64::from(0.65_f32));
}

#[test]
fn slice_comparison_uses_the_upstream_float32_height() {
    let mut cfg = config();
    cfg.slice_h0 = 0.499999999;
    let map = build(&[[0.1, 0.1, 0.5], [0.1, 0.1, 1.0]], &cfg).unwrap();
    assert_eq!(
        cells(&map),
        vec![(0, 0, 0.5, 1.0), (0, 0, 1.0, f64::INFINITY)]
    );
}

#[test]
fn negative_cells_boundaries_and_disconnected_islands_stay_in_world_grid() {
    let map = build(
        &[
            [-0.25, -0.25, -1.0],
            [-0.001, 0.0, -1.0],
            [0.0, 0.25, -1.0],
            [10000.0, -10000.0, -1.0],
        ],
        &config(),
    )
    .unwrap();
    assert_eq!(
        cells(&map)
            .iter()
            .map(|cell| (cell.0, cell.1))
            .collect::<Vec<_>>(),
        vec![(-1, -1), (-1, 0), (0, 1), (40000, -40000)]
    );
    assert_eq!(map.bands.len(), 4);
}

#[test]
fn region_filters_points_and_keeps_the_full_map_grid() {
    let points = [[-1.0, 0.0, 0.0], [0.25, 0.0, 0.0], [1.0, 0.0, 2.0]];
    let map = build_region(&points, &config(), [0.0, 0.0], 1.0).unwrap();
    assert_eq!(map.cells.len(), 3);
    let region = build_region(&points, &config(), [0.25, 0.0], 0.0).unwrap();
    assert_eq!(cells(&region), vec![(1, 0, 0.0, f64::INFINITY)]);
    assert!(build_region(&points, &config(), [0.0, 0.0], -1.0).is_err());
    assert!(build_region(&points, &config(), [f64::NAN, 0.0], 1.0).is_err());
}

#[test]
fn occupancy_bands_keep_continuous_walls_and_separate_gaps() {
    let map = build(
        &[
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.05],
            [0.1, 0.1, 0.10],
            [0.1, 0.1, 0.50],
            [0.1, 0.1, 0.55],
        ],
        &config(),
    )
    .unwrap();
    let intervals: Vec<_> = map.bands.iter().map(|b| (b.low, b.high)).collect();
    assert_eq!(
        intervals,
        vec![(0.0, f64::from(0.10_f32)), (0.50, f64::from(0.55_f32))]
    );
}

#[test]
fn empty_nonfinite_and_invalid_configuration_have_explicit_results() {
    assert!(build(&[], &config()).unwrap().cells.is_empty());
    let map = build(
        &[[f32::NAN, 0.0, 1.0], [0.0, f32::INFINITY, 0.0]],
        &config(),
    )
    .unwrap();
    assert!(map.cells.is_empty());
    assert!(map.bands.is_empty());
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut cfg = config();
        cfg.resolution = value;
        assert!(build(&[], &cfg).is_err());
        let mut cfg = config();
        cfg.slice_dh = value;
        assert!(build(&[], &cfg).is_err());
        let mut cfg = config();
        cfg.point_resolution = value;
        assert!(build(&[], &cfg).is_err());
    }
    assert!(build(&[[f32::MAX, 0.0, 0.0]], &config()).is_err());
}

#[test]
fn sparse_output_matches_independent_upstream_slice_max_min() {
    let mut points = Vec::new();
    for x in -4..=4 {
        for y in -3..=3 {
            for index in 0..13 {
                let z = ((x * 17 + y * 13 + index * 7) % 31) as f32 * 0.125;
                points.push([x as f32 * 0.25 + 0.1, y as f32 * 0.25 + 0.1, z]);
            }
        }
    }
    let cfg = config();
    let actual = build(&points, &cfg).unwrap();
    let mut expected = Vec::new();
    let max_z = points
        .iter()
        .map(|p| p[2])
        .fold(f32::NEG_INFINITY, f32::max);
    let count = ((f64::from(max_z) - cfg.slice_h0) / cfg.slice_dh)
        .ceil()
        .max(0.0) as usize
        + 1;
    for x in -4..=4 {
        for y in -3..=3 {
            for layer in 0..count {
                let slice = (cfg.slice_h0 + layer as f64 * cfg.slice_dh) as f32;
                let mut ground = f32::NEG_INFINITY;
                let mut ceiling = f32::INFINITY;
                for point in &points {
                    if (f64::from(point[0]) / cfg.resolution).floor() != f64::from(x)
                        || (f64::from(point[1]) / cfg.resolution).floor() != f64::from(y)
                    {
                        continue;
                    }
                    if point[2] <= slice {
                        ground = ground.max(point[2]);
                    } else {
                        ceiling = ceiling.min(point[2]);
                    }
                }
                if ground.is_finite() {
                    let cell = (x, y, f64::from(ground), f64::from(ceiling));
                    if expected.last() != Some(&cell) {
                        expected.push(cell);
                    }
                }
            }
        }
    }
    assert_eq!(cells(&actual), expected);
}

#[test]
fn rayon_thread_count_does_not_change_geometry() {
    let points: Vec<_> = (0..20000)
        .map(|index| {
            [
                (index % 70) as f32 * 0.125 - 3.0,
                ((index / 70) % 80) as f32 * 0.125 - 4.0,
                (index % 19) as f32 * 0.1,
            ]
        })
        .collect();
    let run = |threads| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| build(&points, &config()).unwrap())
    };
    let one = run(1);
    let many = run(4);
    assert_eq!(cells(&one), cells(&many));
    assert_eq!(
        one.bands
            .iter()
            .map(|b| (b.x, b.y, b.low, b.high))
            .collect::<Vec<_>>(),
        many.bands
            .iter()
            .map(|b| (b.x, b.y, b.low, b.high))
            .collect::<Vec<_>>()
    );
}
