use super::*;

#[test]
fn native_pgo_request_preserves_committed_nanoseconds_and_rejects_other_frames() {
    use crate::{
        mapper::MapperConfig,
        mapping_backend::{self, MappingBackendInput, MappingBackendSpec},
        persistent_occupancy::PersistentOccupancyGrid,
    };
    use nuwax_codec::lcm::native_pgo as wire;

    let stamp = 1_794_200_973_126_363_516;
    let frame = crate::mapper::tests::room_frame(204, stamp);
    let points: Vec<_> = frame.points_body.iter().map(|p| Point3::from(*p)).collect();
    let input = MappingBackendInput {
        source_timestamp_ns: stamp,
        received_at_ns: stamp,
        registered_frame_id: "map".into(),
        sensor_origin_map: frame.ray_origin_body,
        dynamic_frame: None,
        registered_points_map: points.clone(),
    };
    let compute = MapperConfig::default().compute;
    for backend in [
        MappingBackendSpec::Voxel,
        MappingBackendSpec::raytracing_defaults(),
    ] {
        let mut kernel = backend.into_kernel(compute.window_capacity);
        let candidate = mapping_backend::compute_candidate_in_place(&mut kernel, &input, compute)
            .unwrap()
            .unwrap();
        let grid =
            PersistentOccupancyGrid::stage_observation(None, &candidate.latest_frame_occupancy)
                .unwrap();
        let committed_stamp = candidate.source_timestamp_ns;
        assert_eq!(committed_stamp, stamp);
        assert_eq!((grid.grid.ts * 1e9) as u64, stamp - 124);

        let mut next_frame = frame.clone();
        next_frame.odom_pose.timestamp_ns += 1_000_000;
        let next_input = MappingBackendInput {
            source_timestamp_ns: next_frame.odom_pose.timestamp_ns,
            received_at_ns: next_frame.odom_pose.timestamp_ns,
            ..input.clone()
        };
        assert!(
            mapping_backend::compute_candidate_in_place(&mut kernel, &next_input, compute)
                .unwrap()
                .is_none(),
            "未到构图节拍时，已提交地图仍属于前一帧"
        );

        for (lio_backend, backend_id) in [("fastlio2", 2), ("pointlio", 3)] {
            let mut geometry: nuwax_config::sections::Geometry =
                nuwax_config::test_support::section("geometry");
            geometry.mapping_backend = backend.stable_name().into();
            geometry.lio_backend = lio_backend.into();
            let corrector = Corrector::from_config(
                "native_pgo",
                &geometry,
                &nuwax_config::test_support::section("robot"),
                &nuwax_config::test_support::section("transport"),
            )
            .unwrap();
            let Mode::Native(owner) = &corrector.mode else {
                unreachable!()
            };
            let mut request = owner.request(&frame, &points, (&grid.grid, committed_stamp), 1);
            // 在线拥有方在封装请求前分配请求序号；这里只验证构造与编码，不启动通信。
            request.identity.request_sequence = 1;
            let encoded = wire::encode_native_pgo_request_wire(&request).unwrap();
            let decoded = wire::decode_native_pgo_request_wire(&encoded).unwrap();
            assert_eq!(decoded, request);
            assert_eq!(
                decoded.identity.committed_map.source_timestamp_ns,
                stamp as i64
            );
            assert_eq!(
                decoded.identity.registered_cloud.source_timestamp_ns,
                stamp as i64
            );
            assert_eq!(
                decoded.identity.correlated_odom.source_timestamp_ns,
                stamp as i64
            );
            assert_eq!(decoded.identity.source_timestamp_ns, stamp as i64);
            assert_eq!(decoded.identity.registered_cloud.backend, backend_id);

            let mut mismatched =
                owner.request(&next_frame, &points, (&grid.grid, committed_stamp), 2);
            mismatched.identity.request_sequence = 2;
            assert_eq!(
                mismatched.identity.committed_map.source_timestamp_ns,
                stamp as i64
            );
            let error = wire::encode_native_pgo_request_wire(&mismatched).unwrap_err();
            assert!(error.to_string().contains(
                "committed map, registered cloud, and request source timestamps must agree"
            ));
        }
    }
}

fn local_consensus_source() -> Vec<Point3<f64>> {
    let mut source = Vec::new();
    for index in 0..8 {
        source.push(Point3::new(
            (index & 1) as f64 * 0.8 + 0.03 * index as f64,
            ((index >> 1) & 1) as f64 * 0.9 + 0.011 * (index * index) as f64,
            ((index >> 2) & 1) as f64 * 0.7 + 0.009 * (index * index) as f64,
        ));
    }
    for index in 0..120 {
        source.push(Point3::new(
            5.0 + (index % 11) as f64 * 0.43,
            -3.0 + ((index * 7) % 19) as f64 * 0.31,
            0.2 + ((index * 13) % 23) as f64 * 0.19,
        ));
    }
    source
}
fn assert_transform(actual: Isometry3<f64>, expected: Isometry3<f64>) {
    let error = actual.inverse() * expected;
    assert!(
        error.translation.vector.norm() < 1e-6,
        "平移误差：{error:?}"
    );
    assert!(error.rotation.angle() < 1e-6, "朝向误差：{error:?}");
}

#[test]
fn relocalization_local_consensus_rejects_sparse_wrong_heading() {
    let source = local_consensus_source();
    let truth = Isometry3::from_parts(
        Translation3::new(30., 20., 0.),
        UnitQuaternion::from_euler_angles(0., 0., 37_f64.to_radians()),
    );
    let distractor = Isometry3::from_parts(
        Translation3::new(0.1, -0.05, 0.02),
        UnitQuaternion::from_euler_angles(0., 0., -17_f64.to_radians()),
    );
    let mut target: Vec<_> = source.iter().map(|point| truth * point).collect();
    target.extend(source[..8].iter().map(|point| distractor * point));
    let local = IcpSolver::new(IcpConfig {
        max_correspondence_distance: 0.8,
        ..Default::default()
    })
    .align(&source, &target);
    assert!(local.converged && local.mse < 1e-12);
    assert!((local.transformation.rotation.inverse() * truth.rotation).angle() > 0.9);
    let (actual, _, succeeded) = compute_relocalization(&source, &target, Isometry3::identity());
    assert!(succeeded, "完整结构应由全局配准找回");
    assert_transform(actual, truth);
}

#[test]
fn relocalization_local_consensus_accepts_supported_seed() {
    let source = local_consensus_source();
    let truth = Isometry3::from_parts(
        Translation3::new(0.03, -0.02, 0.01),
        UnitQuaternion::from_euler_angles(0., 0., 0.01),
    );
    let target: Vec<_> = source.iter().map(|point| truth * point).collect();
    let (actual, fitness, succeeded) =
        compute_relocalization(&source, &target, Isometry3::identity());
    assert!(succeeded);
    assert!(fitness < 1e-18);
    assert_transform(actual, truth);
}

#[test]
fn relocalization_local_consensus_preserves_nonzero_anchor_and_reuses_registration() {
    let source = local_consensus_source();
    let anchor = Isometry3::from_parts(
        Translation3::new(2., -3., 0.4),
        UnitQuaternion::from_euler_angles(0., 0., 0.7),
    );
    let truth = Isometry3::from_parts(
        Translation3::new(0.02, -0.01, 0.005),
        UnitQuaternion::from_euler_angles(0.01, -0.015, 0.008),
    ) * anchor;
    let target: Vec<_> = source.iter().map(|point| truth * point).collect();
    let mut registration = anchor;
    for _ in 0..2 {
        let (actual, _, succeeded) = compute_relocalization(&source, &target, registration);
        assert!(succeeded);
        assert_transform(actual, truth);
        registration = actual;
    }
}

#[test]
fn relocalization_local_consensus_rejects_tilted_estimate() {
    let source = local_consensus_source();
    let tilted = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_euler_angles(20_f64.to_radians(), 0., 0.),
    );
    let target: Vec<_> = source.iter().map(|point| tilted * point).collect();
    let (actual, _, succeeded) = compute_relocalization(&source, &target, tilted);
    assert!(!succeeded || is_upright(&actual, GRAVITY_TILT_MAX_DEG));
}

#[test]
fn relocalization_local_consensus_accepts_sparse_and_partial_view() {
    let full = local_consensus_source();
    let truth = Isometry3::from_parts(
        Translation3::new(31., 18., 0.2),
        UnitQuaternion::from_euler_angles(0., 0., 1.),
    );
    let target: Vec<_> = full.iter().map(|point| truth * point).collect();
    let initial = Isometry3::from_parts(
        Translation3::new(0.03, -0.02, 0.01),
        UnitQuaternion::from_euler_angles(0., 0., 0.01),
    ) * truth;
    for source in [
        full[..4].to_vec(),
        full.iter().step_by(4).copied().collect(),
    ] {
        let (actual, _, succeeded) = compute_relocalization(&source, &target, initial);
        assert!(succeeded, "地图其余区域不能稀释稀疏或部分视野的源点支持率");
        assert_transform(actual, truth);
    }
}

#[test]
fn relocalization_sparse_window_matches_and_rejects_degenerate_geometry() {
    let target = local_consensus_source();
    let source: Vec<_> = (0..8).flat_map(|_| target[..8].iter().copied()).collect();
    assert!(compute_relocalization(&source, &target, Isometry3::identity()).2);
    let degenerate = vec![Point3::new(10_000., 10_000., 10_000.); source.len()];
    assert!(!compute_relocalization(&degenerate, &target, Isometry3::identity()).2);
}

#[test]
fn library_retains_only_actually_accepted_keyframe_poses() {
    let mut corrector = Corrector {
        mode: Mode::Library(
            LoopClosureCoordinator::new(LoopClosurePipelineConfig::default()).unwrap(),
        ),
        first_stamp: None,
        sequence: 0,
    };
    for (stamp, x, expected) in [(1, 0., 1), (100_000_001, 0.01, 1), (1_000_000_001, 2., 2)] {
        let mut frame = crate::mapper::tests::room_frame(201, stamp);
        frame.odom_pose.position.as_mut().unwrap().x = x;
        let indices: Vec<_> = (0..frame.points_body.len() as u32).collect();
        assert!(corrector
            .process_frame(&frame, &indices, None, Isometry3::identity())
            .unwrap()
            .is_none());
        let Mode::Library(owner) = &corrector.mode else {
            unreachable!()
        };
        assert_eq!(owner.corrections().keyframes().len(), expected);
    }
}

#[tokio::test]
async fn accepted_pgo_loop_rebuilds_real_map_and_corrects_pose() {
    use crate::mapper::{MapperConfig, MappingFrame};
    let world: Vec<_> = (-20..=20)
        .flat_map(|a| {
            (-10..=20).flat_map(move |b| {
                let u = a as f64 * 0.1;
                let v = b as f64 * 0.1;
                [
                    Point3::new(4., u, v),
                    Point3::new(u + 1., 3., v),
                    Point3::new(u + 1., v, -1.),
                ]
            })
        })
        .collect();
    let config = MapperConfig {
        correction_backend: "library_pgo".into(),
        compute: crate::mapping_backend::MappingBackendComputeConfig {
            window_capacity: 32_768,
            ..MapperConfig::default().compute
        },
        ..Default::default()
    };
    let corrected = Mapper::start(202, config.clone()).unwrap();
    let raw = Mapper::start(
        203,
        MapperConfig {
            correction_backend: "none".into(),
            ..config
        },
    )
    .unwrap();
    let mut last = None;
    let mut before_loop = None;
    let mut after_loop = None;
    let mut previous_transform = Isometry3::identity();
    let mut reference = Corrector {
        mode: Mode::Library(
            LoopClosureCoordinator::new(LoopClosurePipelineConfig::default()).unwrap(),
        ),
        first_stamp: None,
        sequence: 0,
    };
    let mut reference_transform = Isometry3::identity();
    for step in 0..=24 {
        let x = if step <= 10 {
            step as f64 * 0.5
        } else {
            (20 - step.min(20)) as f64 * 0.5
        };
        let truth = Isometry3::translation(x, 0., 0.);
        let odom = Isometry3::translation(x + 0.2 * step as f64 / 24., 0., 0.);
        let stamp = 1_000_000_000 + step as u64 * 10_000_000_000;
        let frame = MappingFrame {
            session_id: 202,
            odom_pose: pose_from_isometry(odom, "odom", stamp),
            points_body: world
                .iter()
                .map(|p| (truth.inverse() * p).coords.into())
                .collect(),
            ray_origin_body: [0., 0., 0.],
            mapping_only: false,
            sensor_id: "lidar".into(),
            registered_cloud: None,
        };
        let indices: Vec<_> = (0..frame.points_body.len() as u32).collect();
        if let Some(update) = reference
            .process_frame(&frame, &indices, None, Isometry3::identity())
            .unwrap()
        {
            reference_transform = update.latest;
        }
        let mut raw_frame = frame.clone();
        raw_frame.session_id = 203;
        raw.feed(raw_frame);
        let raw_snapshot = raw.snapshot().await.unwrap();
        corrected.feed(frame.clone());
        let snapshot = corrected.snapshot().await.unwrap();
        let transform = corrected.map_from_odom();
        if (previous_transform.inverse() * transform)
            .translation
            .vector
            .norm()
            > 1e-4
        {
            before_loop = Some(raw_snapshot);
            after_loop = Some(snapshot.clone());
        }
        previous_transform = transform;
        last = Some(frame.odom_pose);
    }
    let before = before_loop.expect("真实回程扫描必须触发回环接纳");
    let after = after_loop.unwrap();
    assert_ne!(
        before.cloud.display, after.cloud.display,
        "回环接纳后须从真实扫描重建点云"
    );
    assert_ne!(
        before.occupancy, after.occupancy,
        "回环接纳后须更新占据栅格"
    );
    assert_eq!(after.scans.len(), 1, "重建继续保留真实扫描日志");
    let last = last.unwrap();
    let reference_pose = reference_transform * pose_isometry(&last);
    let pose = corrected.transform_pose(&last);
    let position = pose.position.unwrap();
    let expected = pose_isometry(&pose);
    let final_snapshot = corrected.snapshot().await.unwrap();
    let final_pose = pose_isometry(final_snapshot.last_pose.as_ref().unwrap());
    let pose_error = final_pose.inverse() * expected;
    let drift_error =
        (position.x * position.x + position.y * position.y + position.z * position.z).sqrt();
    corrected.shutdown().await.unwrap();
    raw.shutdown().await.unwrap();
    assert!(
        drift_error < 0.1
            && pose_error.translation.vector.norm() < 1e-6
            && pose_error.rotation.angle() < 1e-6,
        "0.2米累计漂移必须至少减半，且发布位姿必须采用最新校正：位置={position:?}，位姿发布误差={pose_error:?}，同一数值配置未剔除地面的对照位置={:?}",
        reference_pose.translation.vector
    );
}
