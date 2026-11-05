use super::*;
#[test]
fn voxel_point_index_matches_full_filter_through_eviction_protection_and_restore() {
    let config = MappingBackendComputeConfig {
        resolution: 0.05,
        initial_safe_radius_meters: 0.0,
        window_capacity: 7,
    };
    let MappingKernel::Voxel(mut indexed) = MappingBackendSpec::Voxel.into_kernel(7) else {
        unreachable!()
    };
    let mut expected = VecDeque::new();
    for frame in 0..30 {
        let mut points: Vec<_> = (0..3)
            .map(|i| {
                Point3::new(
                    frame as f64 * 0.031 + i as f64 * 0.099,
                    (i - 1) as f64 * 0.1,
                    0.15,
                )
            })
            .collect();
        if frame == 7 {
            points.push(Point3::new(1e12, 0.0, 0.0));
        }
        indexed.append_frame(&points);
        expected.extend(points);
        while expected.len() > 7 {
            expected.pop_front();
        }
        let map_from_odom = if frame == 8 {
            nalgebra::Isometry3::translation(1e12, 0.0, 0.0)
        } else {
            nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(0.013, -0.2, 0.03),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, frame as f64 * 0.037),
            )
        };
        let odom_from_map = map_from_odom.inverse();
        let odom: Vec<_> = expected.iter().map(|p| odom_from_map * p).collect();
        let keys: Vec<_> = if frame == 8 {
            odom.iter()
                .filter_map(|p| crate::dynamic_scene::cell([p.x, p.y, p.z]))
                .collect()
        } else if frame % 4 == 0 {
            vec![(1000, 1000, 1000)]
        } else {
            odom.iter()
                .step_by(3)
                .filter_map(|p| crate::dynamic_scene::cell([p.x, p.y, p.z]))
                .collect()
        };
        let decision = crate::dynamic_backend::test_free_cells(
            crate::dynamic_scene::CELL_SIZE,
            keys.into_iter().map(|(x, y, z)| ([x, y, z], 0)),
            map_from_odom,
        );
        expected.retain(|p| !decision.clears([p.x, p.y, p.z]));
        indexed.retain_uncleared(Some(&decision));
        assert_eq!(
            indexed.current_window, expected,
            "第{frame}帧与原逐点结果不一致"
        );
        assert_eq!(
            indexed
                .window_point_indices
                .values()
                .map(VecDeque::len)
                .sum::<usize>(),
            expected.len()
        );
        assert!(
            indexed.current_window.slots.len() <= 7,
            "回收槽必须复用，不能累积超过窗口容量"
        );
        for (key, positions) in &indexed.window_point_indices {
            assert_eq!(indexed.window_cells[key] as usize, positions.len());
            for &position in positions {
                assert_eq!(&window_cell(indexed.current_window.point(position)), key);
            }
        }
        if matches!(frame, 5 | 17) {
            let payload =
                export_mapping_kernel_checkpoint(&MappingKernel::Voxel(indexed.clone())).unwrap();
            let MappingKernel::Voxel(restored) =
                restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &payload)
                    .unwrap()
            else {
                unreachable!()
            };
            indexed = restored;
            assert_eq!(indexed.current_window, expected);
        }
    }
}

#[test]
fn dufomap_private_kernels_revoke_history_with_the_final_native_verdict() {
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let config = MappingBackendComputeConfig {
        window_capacity: 1_000_000,
        ..compute_config()
    };
    let mut kernels = [
        MappingBackendSpec::Voxel.into_kernel(config.window_capacity),
        MappingBackendSpec::raytracing_defaults().into_kernel(config.window_capacity),
    ];
    let mut native =
        nuwax_dufomap_native::DufoMap::new(&crate::dynamic_backend::online_params()).unwrap();
    let mut original = Vec::new();
    for frame in 1..=22 {
        let person = (7..=16)
            .contains(&frame)
            .then(|| -0.5 + (frame - 7) as f64 * 0.12);
        let (points, _) = corridor(person, (frame % 3) as f64);
        let xyz: Vec<_> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        native.insert(&xyz, ORIGIN.map(|v| v as f32), true).unwrap();
        original.extend(points.iter().copied());
        let input = MappingBackendInput {
            source_timestamp_ns: frame * 100_000_000,
            received_at_ns: frame * 100_000_000,
            registered_frame_id: "map".into(),
            sensor_origin_map: ORIGIN,
            dynamic_frame: None,
            registered_points_map: points,
        };
        for kernel in &mut kernels {
            compute_candidate_in_place_with_interval(kernel, &input, config, 0).unwrap();
        }
    }
    let xyz: Vec<_> = original
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let flags = native.seen_free(&xyz).unwrap();
    assert!(flags.iter().any(|v| *v));
    let expected: Vec<_> = original
        .iter()
        .zip(&flags)
        .filter_map(|(p, free)| (!free).then_some(*p))
        .collect();
    let MappingKernel::Voxel(voxel) = &kernels[0] else {
        unreachable!()
    };
    assert_eq!(voxel.current_window.len(), expected.len());
    assert!(
        voxel.current_window.iter().eq(expected.iter()),
        "体素窗口历史必须逐点符合结束时判定，不能回用旧算法"
    );
    let ray = kernels[1].accumulated_points(&[]);
    let xyz: Vec<_> = ray
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    assert!(!xyz.is_empty());
    assert!(
        native.seen_free(&xyz).unwrap().iter().all(|v| !v),
        "射线健康历史不能残留已判为空的单元"
    );
}

fn compute_config() -> MappingBackendComputeConfig {
    MappingBackendComputeConfig {
        resolution: 0.1,
        initial_safe_radius_meters: 0.0,
        window_capacity: 1024,
    }
}

fn input(sequence: u64) -> MappingBackendInput {
    let timestamp_ns = sequence.saturating_mul(1_000_000_000);
    MappingBackendInput {
        source_timestamp_ns: timestamp_ns,
        received_at_ns: timestamp_ns,
        registered_frame_id: "map".to_owned(),
        sensor_origin_map: [0.0; 3],
        dynamic_frame: None,
        registered_points_map: vec![Point3::new(1.0, 0.0, 1.0)],
    }
}

#[test]
fn planar_scan_backends_use_current_frame_origin_and_survive_restore() {
    let config = MappingBackendComputeConfig { ..compute_config() };
    for spec in [
        MappingBackendSpec::Voxel,
        MappingBackendSpec::raytracing_defaults(),
    ] {
        let mut kernel = spec.into_kernel(config.window_capacity);
        let mut first = input(1);
        first.registered_points_map = vec![Point3::new(2.05, 0.05, 0.0)];
        compute_candidate(&mut kernel, &first, config)
            .unwrap()
            .unwrap();
        let mut next = input(2);
        next.sensor_origin_map = [10.05, 0.05, 0.0];
        next.registered_points_map = vec![Point3::new(12.05, 0.05, 0.0)];
        let candidate = compute_candidate(&mut kernel, &next, config)
            .unwrap()
            .unwrap();
        assert!(candidate.live_window_occupancy.origin[0] >= 9.0);
        assert_eq!(
            candidate.live_window_occupancy.data,
            candidate.latest_frame_occupancy.data
        );
        assert!(candidate.latest_frame_occupancy.data.contains(&100));
        let checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
        let mut restored = restore_mapping_kernel_checkpoint(spec, config, &checkpoint).unwrap();
        next.source_timestamp_ns += 1_000_000_000;
        next.received_at_ns += 1_000_000_000;
        let expected = compute_candidate(&mut kernel, &next, config)
            .unwrap()
            .unwrap();
        let actual = compute_candidate(&mut restored, &next, config)
            .unwrap()
            .unwrap();
        assert_eq!(
            actual.live_window_occupancy.data,
            expected.live_window_occupancy.data
        );
        next.source_timestamp_ns += 1_000_000_000;
        next.received_at_ns += 1_000_000_000;
        next.registered_points_map = vec![Point3::new(12.05, 0.05, -0.4)];
        let slanted = compute_candidate(&mut restored, &next, config)
            .unwrap()
            .unwrap();
        assert!(slanted.latest_frame_occupancy.data.contains(&0));
        assert!(!slanted.latest_frame_occupancy.data.contains(&100));
        // 斜射线离开高度带前不足最小观测距离时，仍没有可用二维证据。
        next.source_timestamp_ns += 1_000_000_000;
        next.received_at_ns += 1_000_000_000;
        next.registered_points_map = vec![Point3::new(10.15, 0.05, -0.4)];
        let missing = compute_candidate(&mut restored, &next, config)
            .unwrap()
            .unwrap();
        assert!(missing.latest_frame_occupancy.data.iter().all(|&v| v < 0));
    }
}

#[test]
fn voxel_selection_does_not_parse_raytracing_environment() {
    let spec =
        MappingBackendSpec::from_lookup(MappingBackendKind::Voxel, |_| Some("invalid".to_owned()))
            .expect("inactive raytracing config must not affect voxel startup");
    assert_eq!(spec, MappingBackendSpec::Voxel);
}

#[test]
fn raytracing_defaults_are_valid_and_deterministic() {
    let spec = MappingBackendSpec::from_lookup(MappingBackendKind::Raytracing, |_| None)
        .expect("defaults must pass standalone validation");
    assert_eq!(spec.stable_name(), "raytracing");
    let mut kernel = spec.into_kernel(1024);
    let points = [
        Point3::new(2.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(3.0, 1.0, 0.0),
    ];
    for _ in 0..3 {
        kernel
            .prepare_map_frame([0.0; 3], &points)
            .expect("valid map-frame points");
        kernel
            .commit_prepared_map_frame()
            .expect("prepared frame must commit");
    }
    let output = kernel.accumulated_points(&[]);
    assert!(!output.is_empty());
    assert!(output
        .windows(2)
        .all(|pair| { (pair[0].x, pair[0].y, pair[0].z) <= (pair[1].x, pair[1].y, pair[1].z) }));
}

#[test]
fn invalid_selected_config_fails_closed_with_key() {
    let error = MappingBackendSpec::from_lookup(MappingBackendKind::Raytracing, |key| {
        (key == RAY_SUBSAMPLE_ENV).then(|| "0".to_owned())
    })
    .expect_err("zero ray subsample must fail startup");
    assert!(error.contains("RAYTRACING_CONFIG_INVALID"));
    assert!(error.contains("ray_subsample"));
}

#[test]
fn rejected_frame_does_not_mutate_ray_snapshot() {
    let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
    let valid = [Point3::new(1.0, 0.0, 0.0)];
    for _ in 0..3 {
        kernel
            .prepare_map_frame([0.0; 3], &valid)
            .expect("valid frame");
        kernel
            .commit_prepared_map_frame()
            .expect("prepared frame must commit");
    }
    let before = kernel.accumulated_points(&[]).to_vec();
    let error = kernel
        .prepare_map_frame([0.0; 3], &[Point3::new(f64::MAX, 0.0, 0.0)])
        .expect_err("out-of-range point must be rejected");
    assert!(error.contains("RAYTRACING_UPDATE_REJECTED"));
    assert_eq!(kernel.accumulated_points(&[]), before);
}

#[test]
fn aggregate_frame_work_is_bounded_before_map_mutation() {
    let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
    let points = vec![Point3::new(49.0, 0.0, 0.0); 60_000];
    let error = kernel
        .prepare_map_frame([0.0; 3], &points)
        .expect_err("aggregate ray work must be bounded");
    assert!(error.contains("more than 50000000 ray steps"));
    assert!(!kernel.ready_to_build());
}

#[test]
fn voxel_checkpoint_point_budget_is_the_exact_64_mib_boundary() {
    assert_eq!(MAX_RETAINED_VOXEL_POINTS, 2_796_200);
    let maximum = MAPPING_BACKEND_VOXEL_HEADER_BYTES
        + MAX_RETAINED_VOXEL_POINTS * MAPPING_BACKEND_VOXEL_POINT_BYTES;
    let first_rejected = maximum + MAPPING_BACKEND_VOXEL_POINT_BYTES;
    assert!(maximum <= MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
    assert!(first_rejected > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
}

#[test]
fn binary_voxel_checkpoint_rejects_noncanonical_and_unbounded_inputs() {
    let config = compute_config();
    let empty = MappingBackendSpec::Voxel.into_kernel(config.window_capacity);
    let payload = export_mapping_kernel_checkpoint(&empty).expect("encode empty checkpoint");
    assert_eq!(payload.len(), MAPPING_BACKEND_VOXEL_HEADER_BYTES);

    let mut trailing = payload.clone();
    trailing.push(0);
    assert!(
        restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &trailing)
            .err()
            .expect("trailing byte must fail")
            .contains("length mismatch")
    );

    let mut unknown_flags = payload.clone();
    unknown_flags[13] = 1;
    assert!(
        restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &unknown_flags)
            .err()
            .expect("unknown flag must fail")
            .contains("flags/reserved")
    );

    let mut oversized_count = payload;
    oversized_count[MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + size_of::<u64>()
        ..MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + 2 * size_of::<u64>()]
        .copy_from_slice(&u64::MAX.to_le_bytes());
    assert!(
        restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &oversized_count,)
            .err()
            .expect("oversized point count must fail before allocation")
            .contains("voxel window contains")
    );

    let one_point = MappingKernel::Voxel(VoxelKernel {
        current_window: [Point3::new(1.0, 2.0, 3.0)].into_iter().collect(),
        window_cells: window_cells(&[Point3::new(1.0, 2.0, 3.0)].into_iter().collect()),
        window_point_indices: window_point_indices(
            &[Point3::new(1.0, 2.0, 3.0)].into_iter().collect(),
        ),
        capacity: config.window_capacity,
        last_candidate_point_count: 1,
        last_candidate_received_at_ns: 7,
        dirty_since_candidate: false,
        dufo: Default::default(),
    });
    let mut non_finite = export_mapping_kernel_checkpoint(&one_point).expect("encode one point");
    non_finite
        [MAPPING_BACKEND_VOXEL_HEADER_BYTES..MAPPING_BACKEND_VOXEL_HEADER_BYTES + size_of::<f64>()]
        .copy_from_slice(&f64::NAN.to_bits().to_le_bytes());
    assert!(
        restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &non_finite)
            .err()
            .expect("non-finite point must fail")
            .contains("non-finite")
    );
}

#[test]
fn failed_candidate_construction_does_not_mutate_backend_state() {
    let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
    for sequence in 1..=3 {
        compute_candidate(&mut kernel, &input(sequence), compute_config()).expect("seed ray state");
    }
    let MappingKernel::Raytracing(before) = &kernel else {
        panic!("ray kernel expected");
    };
    let voxel_count = before.map.voxels.len();
    let stable_points = kernel.accumulated_points(&[]);
    let last_candidate_point_count = before.last_candidate_point_count;
    let last_candidate_received_at_ns = before.last_candidate_received_at_ns;
    let mut invalid_config = compute_config();
    invalid_config.resolution = 0.0;
    let result = compute_candidate(&mut kernel, &input(4), invalid_config);
    assert!(result.is_err());
    let MappingKernel::Raytracing(ray) = &kernel else {
        panic!("ray kernel expected");
    };
    assert_eq!(ray.map.voxels.len(), voxel_count);
    assert_eq!(kernel.accumulated_points(&[]), stable_points);
    assert_eq!(ray.last_candidate_point_count, last_candidate_point_count);
    assert_eq!(
        ray.last_candidate_received_at_ns,
        last_candidate_received_at_ns
    );
}
