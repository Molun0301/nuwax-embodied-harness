use super::*;

fn scene(floors: &[f32]) -> SurfaceSnapshot {
    let (_, base) = crate::native_test_helpers::fixture();
    let points: Vec<_> = (0..40)
        .flat_map(|x| {
            (-6..7).flat_map(move |y| {
                floors
                    .iter()
                    .map(move |&height| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, height))
            })
        })
        .collect();
    SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1).unwrap()
}

#[test]
fn terrain_source_defaults_and_rejects_unknown_values() {
    assert_eq!(PctTerrainSource::default(), PctTerrainSource::Tomography);
    assert_eq!(
        PctTerrainSource::parse("  ").unwrap(),
        PctTerrainSource::Tomography
    );
    assert_eq!(
        PctTerrainSource::parse("tomography").unwrap(),
        PctTerrainSource::Tomography
    );
    assert_eq!(
        PctTerrainSource::parse(" mls ").unwrap(),
        PctTerrainSource::Mls
    );
    assert!(PctTerrainSource::parse("typo").is_err());
}

#[test]
fn tomography_keeps_two_floors_and_native_bridge_prevents_teleporting() {
    let map = scene(&[0.01, 2.01]);
    let (config, cells, bands) =
        pct_input_for_source(&map, 0.4, &[], PctTerrainSource::Tomography).unwrap();
    let levels: Vec<_> = cells
        .iter()
        .filter(|cell| cell.x == 10 && cell.y == 0)
        .collect();
    assert_eq!(levels.len(), 2);
    assert!((levels[0].ground - 0.01).abs() < 1e-6);
    assert!((levels[0].ceiling - 2.01).abs() < 1e-6);
    assert!((levels[1].ground - 2.01).abs() < 1e-6);
    assert!(levels[1].ceiling.is_infinite());
    let mut planner = nuwax_nav_pct_native::Planner::from_cells(config, &cells, &bands).unwrap();
    assert!(planner.plan([0.55, 0.05, 0.41], [3.45, 0.05, 0.41]).is_ok());
    assert!(planner.plan([0.55, 0.05, 2.41], [3.45, 0.05, 2.41]).is_ok());
    assert!(planner
        .plan([0.55, 0.05, 0.41], [3.45, 0.05, 2.41])
        .is_err());
}

#[test]
fn tomography_keeps_recent_obstacle_in_collision_bands_without_adding_a_floor() {
    let base = scene(&[0.01]);
    let recent: Vec<_> = (0..13)
        .flat_map(|y| {
            (0..9).map(move |z| {
                let index = (y * 9 + z) as u32;
                (index, [2.05, (y as f64 - 5.5) * 0.1, 0.15 + z as f64 * 0.1])
            })
        })
        .collect();
    let blocked = base
        .with_recent_obstacle_points(1, 1, 0, &[], recent)
        .unwrap();
    let (config, cells, bands) = pct_input_from_tomogram(&blocked, 0.4, &[]).unwrap();
    assert!(cells.iter().all(|cell| (cell.ground - 0.01).abs() < 1e-6));
    let mut clear = {
        let (cfg, c, b) = pct_input_from_tomogram(&base, 0.4, &[]).unwrap();
        nuwax_nav_pct_native::Planner::from_cells(cfg, &c, &b).unwrap()
    };
    assert!(clear.plan([0.55, 0.05, 0.41], [3.45, 0.05, 0.41]).is_ok());
    let mut blocked = nuwax_nav_pct_native::Planner::from_cells(config, &cells, &bands).unwrap();
    assert!(blocked
        .plan([0.55, 0.05, 0.41], [3.45, 0.05, 0.41])
        .is_err());
}

#[test]
fn old_source_keeps_original_input_and_both_sources_share_robot_configuration() {
    let map = scene(&[0.01]);
    let boundary = [[0., -0.5], [4., -0.5], [4., 0.6], [0., 0.6]];
    let (old, old_cells, old_bands) = pct_input(&map, 0.4, &boundary).unwrap();
    let (selected, selected_cells, selected_bands) =
        pct_input_for_source(&map, 0.4, &boundary, PctTerrainSource::Mls).unwrap();
    let (new, _, _) = pct_input_from_tomogram(&map, 0.4, &boundary).unwrap();
    let geometry = |cells: &[Cell]| {
        cells
            .iter()
            .map(|c| (c.x, c.y, c.ground, c.ceiling))
            .collect::<Vec<_>>()
    };
    let occupied = |bands: &[Band]| {
        bands
            .iter()
            .map(|b| (b.x, b.y, b.low, b.high))
            .collect::<Vec<_>>()
    };
    assert_eq!(geometry(&old_cells), geometry(&selected_cells));
    assert_eq!(occupied(&old_bands), occupied(&selected_bands));
    for cfg in [&selected, &new] {
        assert_eq!(cfg.resolution, old.resolution);
        assert_eq!(cfg.robot_height, old.robot_height);
        assert_eq!(cfg.max_step, old.max_step);
        assert_eq!(cfg.max_step_down, old.max_step_down);
        assert_eq!(cfg.max_slope, old.max_slope);
        assert_eq!(cfg.footprint, old.footprint);
        assert_eq!(cfg.body_z_min, old.body_z_min);
        assert_eq!(cfg.body_z_max, old.body_z_max);
        assert_eq!(cfg.boundary, boundary);
    }
}

fn profile_scene(height: impl Fn(f32) -> f32) -> SurfaceSnapshot {
    let (_, base) = crate::native_test_helpers::fixture();
    let mut points = Vec::new();
    for x in 0..40 {
        let x = (x as f32 + 0.5) * 0.1;
        for y in -6..7 {
            points.push((x, (y as f32 + 0.5) * 0.1, height(x)));
        }
    }
    SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1).unwrap()
}

fn route_on_tomography(map: &SurfaceSnapshot, start_z: f64, goal_z: f64) -> bool {
    let boundary = [[0., -0.6], [4., -0.6], [4., 0.7], [0., 0.7]];
    let (config, cells, bands) = pct_input_from_tomogram(map, 0.4, &boundary).unwrap();
    nuwax_nav_pct_native::Planner::from_cells(config, &cells, &bands)
        .unwrap()
        .plan([0.55, 0.05, start_z + 0.4], [3.45, 0.05, goal_z + 0.4])
        .is_ok()
}

#[test]
fn native_bridge_applies_step_limits_to_tomographic_heights() {
    let step = |height: f32| profile_scene(|x| if x < 2.0 { 0.01 } else { 0.01 + height });
    assert!(route_on_tomography(&step(0.15), 0.01, 0.16));
    assert!(!route_on_tomography(&step(0.35), 0.01, 0.36));
}

#[test]
fn native_bridge_distinguishes_continuous_ramps_from_independent_steps() {
    let ramp = |slope: f32| profile_scene(|x| 0.01 + slope * x);
    assert!(route_on_tomography(
        &ramp(0.2),
        0.01 + 0.2 * 0.55,
        0.01 + 0.2 * 3.45
    ));
    // 每格高差小于阶高上限的连续陡坡，仍须按原生坡度规则拒绝。
    assert!(!route_on_tomography(
        &ramp(0.9),
        0.01 + 0.9 * 0.55,
        0.01 + 0.9 * 3.45
    ));
}
