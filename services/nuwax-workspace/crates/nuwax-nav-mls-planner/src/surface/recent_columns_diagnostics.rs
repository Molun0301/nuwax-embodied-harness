//! Frozen original oracle versus the actual production sorted-column builder.
use super::*;
use std::time::Instant;

// Frozen original per-point BTreeMap insertion, captured before production replacement.
fn original(
    map: &SurfaceSnapshot,
    source_sequence: u64,
    source_timestamp_ns: u64,
    retention_ns: u64,
    ground_point_indices: &[u32],
    points: impl IntoIterator<Item = (u32, [f64; 3])>,
) -> Result<SurfaceSnapshot, SurfaceError> {
    if source_sequence == 0 || source_timestamp_ns == 0 {
        return Err(SurfaceError::InvalidConfig(
            "recent cloud identity must be nonzero".into(),
        ));
    }
    let size = f64::from(map.source_voxel_size_m());
    if let Some(previous) = map.recent_obstacles.as_ref().and_then(|r| r.frames.last()) {
        if source_sequence <= previous.sequence || source_timestamp_ns < previous.timestamp {
            return Err(SurfaceError::InvalidConfig(
                "recent cloud is not source-monotonic".into(),
            ));
        }
    }
    let ground: AHashSet<u32> = ground_point_indices.iter().copied().collect();
    let mut columns: BTreeMap<[i32; 2], Vec<i32>> = BTreeMap::new();
    let mut surface_points: AHashMap<[i32; 3], Vec<[f64; 3]>> = AHashMap::new();
    let mut obstacles = AHashSet::new();
    for (index, point) in points {
        if !point.into_iter().all(f64::is_finite) {
            continue;
        }
        let scaled = point.map(|x| (x / size).floor());
        if scaled
            .into_iter()
            .any(|x| x < f64::from(i32::MIN) || x >= f64::from(i32::MAX))
        {
            return Err(SurfaceError::InvalidConfig(
                "recent point exceeds voxel range".into(),
            ));
        }
        let [x, y, z] = scaled.map(|x| x as i32);
        columns.entry([x, y]).or_default().push(z);
        if ground.contains(&index) {
            surface_points.entry([x, y, z]).or_default().push(point);
        } else {
            obstacles.insert([x, y, z]);
        }
    }
    surface_points.retain(|key, _| !obstacles.contains(key));
    for zs in columns.values_mut() {
        zs.sort_unstable();
        zs.dedup();
    }
    let mut frames = Vec::new();
    if retention_ns > 0 {
        if let Some(previous) = &map.recent_obstacles {
            if previous.size == size {
                frames.extend(
                    previous
                        .frames
                        .iter()
                        .filter(|frame| {
                            source_timestamp_ns.saturating_sub(frame.timestamp) <= retention_ns
                        })
                        .cloned(),
                );
            }
        }
    }
    if frames.len() >= 8 {
        frames.drain(..frames.len() - 7);
    }
    frames.push(Arc::new(RecentFrame {
        sequence: source_sequence,
        timestamp: source_timestamp_ns,
        columns,
        surface_points,
    }));
    let mut result = map.clone();
    result.recent_obstacles = Some(Arc::new(RecentObstacles { size, frames }));
    result.body_query_cache = Default::default();
    Ok(result)
}

fn candidate(
    map: &SurfaceSnapshot,
    source_sequence: u64,
    source_timestamp_ns: u64,
    retention_ns: u64,
    ground_point_indices: &[u32],
    points: impl IntoIterator<Item = (u32, [f64; 3])>,
) -> Result<(SurfaceSnapshot, ()), SurfaceError> {
    map.with_recent_obstacle_points(
        source_sequence,
        source_timestamp_ns,
        retention_ns,
        ground_point_indices,
        points,
    )
    .map(|snapshot| (snapshot, ()))
}

fn fixture() -> SurfaceSnapshot {
    let config = Config {
        voxel_size: 0.1,
        robot_height: 0.25,
        surface_closing_radius: 0.0,
        wall_clearance_m: 0.01,
        step_threshold_m: 0.21,
        min_step_tread_depth_m: 0.2,
        world_frame: "map".into(),
        node_spacing_m: 0.3,
        wall_buffer_m: 0.0,
        wall_buffer_weight: 0.0,
        max_slope_deg: 25.0,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    };
    let points: Vec<_> = (-10..20)
        .flat_map(|x| {
            (-10..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
        })
        .collect();
    SurfaceSnapshot::from_points(&points, config, 0.1)
        .unwrap()
        .with_body_envelope(body::SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.15, -0.15], [0.15, -0.15], [0.15, 0.15], [-0.15, 0.15]],
            z_bounds_m: [-0.2, 0.2],
            body_to_support_z_m: 0.4,
            max_step_up_m: 0.2,
            max_step_down_m: 0.2,
            max_cross_slope_deg: 35.0,
            support_uncertainty_m: 0.01,
        })
        .unwrap()
}

fn assert_frames(a: &SurfaceSnapshot, b: &SurfaceSnapshot) {
    let a = a.recent_obstacles.as_ref().unwrap();
    let b = b.recent_obstacles.as_ref().unwrap();
    assert_eq!(a.size.to_bits(), b.size.to_bits());
    assert_eq!(a.frames.len(), b.frames.len());
    for (a, b) in a.frames.iter().zip(&b.frames) {
        assert_eq!(a.sequence, b.sequence);
        assert_eq!(a.timestamp, b.timestamp);
        assert_eq!(a.columns, b.columns);
        assert_eq!(a.surface_points.len(), b.surface_points.len());
        for (key, points) in &a.surface_points {
            let actual = b.surface_points.get(key).unwrap();
            assert_eq!(
                points
                    .iter()
                    .map(|p| p.map(f64::to_bits))
                    .collect::<Vec<_>>(),
                actual
                    .iter()
                    .map(|p| p.map(f64::to_bits))
                    .collect::<Vec<_>>()
            );
        }
    }
}

fn downstream(map: &SurfaceSnapshot) -> (Vec<([u64; 3], [u64; 3])>, Vec<Result<(), String>>) {
    let q = [0.0, 0.0, 0.0, 1.0];
    let hints = map
        .recent_obstacle_hints([[-2.0, -2.0], [2.0, 2.0]], ([0.5, 0.5, 0.5], q))
        .unwrap()
        .into_iter()
        .map(|(a, b)| (a.map(f64::to_bits), b.map(f64::to_bits)))
        .collect();
    let queries = [[0.5, 0.5, 0.5], [-0.5, 0.5, 0.5], [1.5, 1.5, 0.5]]
        .into_iter()
        .map(|p| map.validate_body_pose(p, q).map_err(|e| e.to_string()))
        .collect();
    (hints, queries)
}

fn reordered(mut points: Vec<(u32, [f64; 3])>, order: usize) -> Vec<(u32, [f64; 3])> {
    points.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    if order == 1 {
        points.reverse();
    }
    if order == 2 {
        let mut seed = 19_u64;
        for i in (1..points.len()).rev() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            points.swap(i, (seed as usize) % (i + 1));
        }
    }
    points
}

#[test]
fn sorted_columns_exact_frames_and_query_corpus() {
    let map = fixture();
    let mut comparisons = 0;
    for shape in 0..5 {
        for membership in 0..3 {
            for order in 0..3 {
                for retain in [0, 1000] {
                    let points: Vec<_> = match shape {
                        0 => vec![],
                        1 => (0..48)
                            .map(|i| (i, [0.5 + (i % 3) as f64 * 0.001, 0.5, 0.55]))
                            .collect(),
                        2 => (0..48)
                            .map(|i| {
                                (
                                    i,
                                    [
                                        -0.95 + (i % 8) as f64 * 0.2,
                                        -0.75 + (i / 8) as f64 * 0.2,
                                        0.05 + (i % 3) as f64 * 0.25,
                                    ],
                                )
                            })
                            .collect(),
                        3 => (0..48)
                            .map(|i| (i, [if i % 2 == 0 { 0.0 } else { -0.0 }, 0.5, 0.55]))
                            .collect(),
                        _ => (0..48)
                            .map(|i| (i, [0.55, 0.55, 0.02 + (i % 5) as f64 * 0.01]))
                            .collect(),
                    };
                    let mut members: Vec<_> = points
                        .iter()
                        .filter(|(i, _)| membership == 0 || membership == 2 && i % 2 == 0)
                        .map(|(i, _)| *i)
                        .collect();
                    if order == 2 {
                        members.reverse();
                        if let Some(index) = members.first().copied() {
                            members.extend([index, index]);
                        }
                    }
                    let points = reordered(points, order);
                    let mut a = map.clone();
                    let mut b = map.clone();
                    for sequence in 1..=9 {
                        a = original(
                            &a,
                            sequence,
                            100 + sequence,
                            retain,
                            &members,
                            points.iter().copied(),
                        )
                        .unwrap();
                        b = candidate(
                            &b,
                            sequence,
                            100 + sequence,
                            retain,
                            &members,
                            points.iter().copied(),
                        )
                        .unwrap()
                        .0;
                        assert_frames(&a, &b);
                        comparisons += 1;
                        assert_eq!(
                            a.recent_obstacles.as_ref().unwrap().frames.len(),
                            if retain == 0 {
                                1
                            } else {
                                (sequence as usize).min(8)
                            }
                        );
                    }
                    assert_eq!(downstream(&a), downstream(&b));
                    // Retention expiry keeps only the genuinely current frame in both owners.
                    a = original(&a, 10, 5000, retain, &members, points.iter().copied()).unwrap();
                    b = candidate(&b, 10, 5000, retain, &members, points.iter().copied())
                        .unwrap()
                        .0;
                    assert_frames(&a, &b);
                    assert_eq!(a.recent_obstacles.as_ref().unwrap().frames.len(), 1);
                    comparisons += 1;
                }
            }
        }
    }
    assert_eq!(comparisons, 900);
}

#[test]
fn sorted_columns_error_order_and_input_consumption() {
    use std::cell::Cell;
    let base = fixture();
    let previous = base
        .with_recent_obstacle_points(5, 50, 1000, &[], [(0, [0.5, 0.5, 0.55])])
        .unwrap();
    for (sequence, time) in [(0, 60), (6, 0), (5, 60), (6, 49), (6, 50), (6, 60)] {
        for points in [
            vec![],
            vec![(0, [f64::NAN, 0.0, 0.0]), (1, [0.5, 0.5, 0.55])],
            vec![
                (0, [0.5, f64::INFINITY, 0.5]),
                (1, [-1e12, 0.0, 0.0]),
                (2, [0.5, 0.5, 0.55]),
            ],
            vec![(0, [1e12, 0.0, 0.0]), (1, [f64::NAN, 0.0, 0.0])],
        ] {
            let read_a = Cell::new(0);
            let read_b = Cell::new(0);
            let a = original(
                &previous,
                sequence,
                time,
                1000,
                &[0, 1],
                points
                    .iter()
                    .copied()
                    .inspect(|_| read_a.set(read_a.get() + 1)),
            );
            let b = candidate(
                &previous,
                sequence,
                time,
                1000,
                &[0, 1],
                points
                    .iter()
                    .copied()
                    .inspect(|_| read_b.set(read_b.get() + 1)),
            );
            assert_eq!(read_a.get(), read_b.get());
            match (a, b) {
                (Ok(a), Ok((b, _))) => {
                    assert_frames(&a, &b);
                    assert_eq!(downstream(&a), downstream(&b));
                }
                (Err(a), Err(b)) => assert_eq!(a.to_string(), b.to_string()),
                _ => panic!("candidate acceptance mismatch"),
            }
        }
    }
}

#[test]
#[ignore = "paired test-only recent-column candidate; exclusive CPU lane"]
fn sorted_columns_paired_builder_cost() {
    let terrain = fixture();
    let mut cases = Vec::new();
    for count in [1000, 10000, 100000] {
        for dispersed in [false, true] {
            for retain in [false, true] {
                let points: Vec<_> = (0..count)
                    .map(|i| {
                        let j = if dispersed { i } else { i % 64 };
                        (
                            i as u32,
                            [
                                0.05 + (j % 320) as f64 * 0.1,
                                0.05 + (j / 320) as f64 * 0.1,
                                if i % 4 == 0 { 0.05 } else { 0.55 },
                            ],
                        )
                    })
                    .collect();
                let members: Vec<_> = (0..count)
                    .filter(|i| i % 4 == 0)
                    .map(|i| i as u32)
                    .collect();
                let retention = if retain { 1000 } else { 0 };
                let mut previous_original = terrain.clone();
                let mut previous_candidate = terrain.clone();
                let mut original_us = Vec::new();
                let mut candidate_us = Vec::new();
                let mut capacities = None;
                for sequence in 1..=17 {
                    let original = || {
                        let start = Instant::now();
                        let value = original(
                            &previous_original,
                            sequence,
                            100 + sequence,
                            retention,
                            &members,
                            points.iter().copied(),
                        )
                        .unwrap();
                        (value, start.elapsed().as_secs_f64() * 1e6)
                    };
                    let trial = || {
                        let start = Instant::now();
                        let value = candidate(
                            &previous_candidate,
                            sequence,
                            100 + sequence,
                            retention,
                            &members,
                            points.iter().copied(),
                        )
                        .unwrap();
                        (value, start.elapsed().as_secs_f64() * 1e6)
                    };
                    let ((a, ta), ((b, ()), tb)) = if sequence % 2 == 0 {
                        let b = trial();
                        let a = original();
                        (a, b)
                    } else {
                        let a = original();
                        let b = trial();
                        (a, b)
                    };
                    assert_frames(&a, &b);
                    let fa = a.recent_obstacles.as_ref().unwrap().frames.last().unwrap();
                    let heights = |map: &SurfaceSnapshot| -> usize {
                        map.recent_obstacles
                            .as_ref()
                            .unwrap()
                            .frames
                            .iter()
                            .map(|frame| {
                                frame
                                    .columns
                                    .values()
                                    .map(|v| v.capacity() * 4)
                                    .sum::<usize>()
                            })
                            .sum()
                    };
                    let fb = b.recent_obstacles.as_ref().unwrap().frames.last().unwrap();
                    capacities = Some(serde_json::json!({
                        "original_current_frame_height_capacity_bytes":fa.columns.values().map(|v|v.capacity()*4).sum::<usize>(),
                        "candidate_current_frame_height_capacity_bytes":fb.columns.values().map(|v|v.capacity()*4).sum::<usize>(),
                        "original_retained_height_capacity_bytes":heights(&a),
                        "candidate_retained_height_capacity_bytes":heights(&b),
                        "columns":fa.columns.len()}));
                    if sequence > 8 {
                        original_us.push(ta);
                        candidate_us.push(tb);
                    }
                    // Both outputs coexist for exact comparison, outside the timed builder.
                    // Replaced previous/candidate-owner destruction is also outside timing.
                    previous_original = a;
                    previous_candidate = b;
                }
                cases.push(serde_json::json!({"points":count,"dispersed":dispersed,"frames":if retain {8}else{1},
            "original_us":original_us,"candidate_us":candidate_us,"capacities":capacities.unwrap()}));
            }
        }
    }
    println!(
        "RECENT_COLUMNS_TRIAL {}",
        serde_json::json!({"warmup_pairs":8,"measured_pairs":9,
        "capacity_scope":"actual final-height Vec capacities across retained frames; temporary capacities recorded separately in initial test-only trial; not allocator or RSS",
        "cases":cases})
    );
}
