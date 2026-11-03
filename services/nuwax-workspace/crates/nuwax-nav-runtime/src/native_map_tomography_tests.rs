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

/// 精细源只有体素占据与独立地面记录，地面高度取自高度场统计。
/// `extra` 为非地面成员的占据体素，键为精细格坐标。
fn heightfield_floor(height: impl Fn(f64) -> f64, extra: &[[i32; 3]], max_step: f64) -> SurfaceSnapshot {
    use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
    use nuwax_map_chunks::{ColumnGroundRecords, Domain, HeightfieldSurface, Receiver, Repository, Transfer, Writer};
    use nuwax_nav_mls_planner::surface::{SurfaceObservation, SurfaceObservations};
    const Q: f32 = 0.05;
    let bin = f64::from(Q) / 8.0;
    let mut writer = Writer::new(Repository::temporary().unwrap(), Q, 4 * 1024 * 1024).unwrap();
    let mut points = Vec::new();
    for x in 0..80 {
        for y in -6..6 {
            // 回波在实际高度上下约2厘米内分布，贴近量化边界的地面占两层体素。
            let center = (height((f64::from(x) + 0.5) * f64::from(Q)) / bin).floor() as i32;
            let mut counts = BTreeMap::<i32, [u32; 8]>::new();
            for offset in -3..=3 {
                let sub = center + offset;
                counts.entry(sub.div_euclid(8)).or_default()[sub.rem_euclid(8) as usize] += 4 - offset.abs() as u32;
            }
            let column = HeightfieldColumn::from_counts([x, y], Q, counts).unwrap().unwrap();
            for (z, _) in column.members() {
                writer.observe([[x, y, z]], 20).unwrap();
                points.push(((x as f32 + 0.5) * Q, (y as f32 + 0.5) * Q, (z as f32 + 0.5) * Q));
            }
            writer.replace_ground_column([x, y], &ColumnGroundRecords {
                classified_empty: false,

                heightfields: vec![HeightfieldSurface { column, samples: vec![] }],
            }).unwrap();
        }
    }
    writer.observe(extra.iter().copied(), 20).unwrap();
    points.extend(extra.iter().map(|&[x, y, z]| ((x as f32 + 0.5) * Q, (y as f32 + 0.5) * Q, (z as f32 + 0.5) * Q)));
    let domain = Domain { session: "tomography-ground".into(), generation: 1, frame_generation: 0, producer_generation: 1 };
    let root = writer.snapshot(domain, 1, 20).unwrap();
    let mut receiver = Receiver::new(Repository::temporary().unwrap());
    receiver.receive(&Transfer { root: root.encode().unwrap(), included: BTreeMap::new() }, &root.domain,
        |_, r| writer.repository().read_object(r.object, nuwax_map_chunks::MAX_BLOCK_BYTES)).unwrap();
    let mut observations = SurfaceObservations::new(Q, 32).unwrap();
    let mut parts = Vec::new();
    for r in &root.blocks {
        let (block, records) = receiver.ground_parts(r).unwrap();
        observations.replace_block(r.key, block.iter().map(|(&local, e)| (
            nuwax_map_chunks::global_key(r.key, local).unwrap(),
            SurfaceObservation { observed_ns: e.observed_ns, hits: e.hits },
        )).collect()).unwrap();
        parts.push((r.key, records));
    }
    for xy in parts.iter().map(|(k, _)| [k[0], k[1]]).collect::<BTreeSet<_>>() {
        let group: Vec<_> = parts.iter().filter(|(k, _)| k[..2] == xy).cloned().collect();
        observations.replace_heightfield_columns(xy, &group);
    }
    observations.mark_complete_geometry_source();
    let (_, base) = crate::native_test_helpers::fixture();
    let body = nuwax_nav_mls_planner::SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.06, -0.06], [0.06, -0.06], [0.06, 0.06], [-0.06, 0.06]],
        // 机体底面落到支撑面，地面厚度若被当作占据会直接挡住机体。
        z_bounds_m: [-0.4, 0.1],
        body_to_support_z_m: 0.4,
        max_step_up_m: max_step,
        max_step_down_m: max_step,
        max_cross_slope_deg: 18.0,
        support_uncertainty_m: 0.0,
    };
    SurfaceSnapshot::from_points(&points, base.config().clone(), Q)
        .unwrap()
        .with_body_envelope(body)
        .unwrap()
        .with_observations(Some(std::sync::Arc::new(observations)))
}

fn heightfield_route(map: &SurfaceSnapshot, height: impl Fn(f64) -> f64) -> Result<Vec<[f64; 3]>, String> {
    let (config, cells, bands) = pct_input_from_tomogram(map, 0.4, &[]).unwrap();
    let mut planner = nuwax_nav_pct_native::Planner::from_cells(config, &cells, &bands)?;
    planner.plan([0.125, 0.025, height(0.125) + 0.4], [3.875, 0.025, height(3.875) + 0.4]).map(|route| route.path)
}

#[test]
fn tomography_uses_continuous_ground_records_across_voxel_boundaries() {
    // 1°缓坡在4米内升高约7厘米，跨过一个5厘米体素边界；阶高为0时只能按连续坡度通行。
    let height = |x: f64| 0.012 + 1.0_f64.to_radians().tan() * x;
    let map = heightfield_floor(height, &[], 0.0);
    let (config, cells, bands) = pct_input_from_tomogram(&map, 0.4, &[]).unwrap();
    assert_eq!(config.max_step, 0.0);
    // 每列一个记录高度；两层体素厚的地面只留零厚度的地面带，不成为机体障碍。
    assert_eq!(cells.len(), 80 * 12);
    assert!(bands.iter().all(|band| band.high == band.low));
    assert!(cells.iter().all(|cell| (cell.ground - height((f64::from(cell.x) + 0.5) * 0.05)).abs() < 0.02));
    let path = heightfield_route(&map, height).expect("缓坡跨量化边界应可通行");
    let limit = 18.0_f64.to_radians().tan() * 0.05 * 2.0_f64.sqrt();
    assert!(path.windows(2).all(|pair| (pair[1][2] - pair[0][2]).abs() <= limit + 1e-6));
    drop((config, cells, bands));

    // 非地面成员的占据体素仍是实体：地面以上一整排低矮实体挡住阶高0的机体。
    let wall: Vec<_> = (-6..6).map(|y| [40, y, 2]).collect();
    assert!(heightfield_route(&heightfield_floor(height, &wall, 0.0), height).is_err());
}

#[test]
fn tomography_ground_records_keep_real_steps_under_step_limit() {
    // 记录高度本身有5厘米跳变时是真实台阶：阶高0拒绝，阶高足够时通行。
    let height = |x: f64| if x < 2.0 { 0.012 } else { 0.062 };
    assert!(heightfield_route(&heightfield_floor(height, &[], 0.0), height).is_err());
    assert!(heightfield_route(&heightfield_floor(height, &[], 0.08), height).is_ok());
}
