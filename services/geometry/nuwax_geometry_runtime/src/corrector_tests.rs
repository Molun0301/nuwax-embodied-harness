use super::*;
use nuwax_mapping::relocalization::{is_upright, GRAVITY_TILT_MAX_DEG};

#[test]
fn native_pgo_timeout_logs_request_stages_and_child_output_tail() {
    use std::{io::Write, os::unix::fs::PermissionsExt, sync::{Arc, Mutex}};
    use nuwax_nav_pgo_bridge as bridge;

    #[derive(Clone)]
    struct LogBuffer(Arc<Mutex<Vec<u8>>>);
    impl Write for LogBuffer {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }

    let fixture = std::path::PathBuf::from(env!("OUT_DIR"))
        .join(format!("pgo-timeout-{}", std::process::id()));
    std::fs::create_dir_all(&fixture).unwrap();
    let executable = fixture.join("pgo");
    let pid_file = fixture.join("pid");
    // 真实原生端先完成就绪，再暂停收发；不替换生产传输，不修改五秒期限。
    std::fs::write(&executable, format!(
        "#!/bin/sh\nprintf 'N6-20 stdout tail\\n'\nprintf 'N6-20 stderr tail\\n' >&2\nprintf '%s' \"$$\" > '{}'\nexec '{}' \"$@\"\n",
        pid_file.display(), bridge::discovery_native_executable().display()
    )).unwrap();
    std::fs::set_permissions(&executable, std::fs::Permissions::from_mode(0o755)).unwrap();
    std::fs::copy(bridge::discovery_group_guardian_executable(), fixture.join("nuwax-group-guardian")).unwrap();
    let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let port = socket.local_addr().unwrap().port();
    drop(socket);
    let mut config = bridge::OnlinePgoConfig::new(format!("udpm://239.255.76.64:{port}?ttl=0"), 204, 1);
    config.executable = executable;
    assert_eq!(config.result_timeout, std::time::Duration::from_secs(5));
    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(1).enable_all().build().unwrap();
    let (owner, ingress) = runtime.block_on(bridge::OnlinePgoOwner::start(config)).unwrap();
    let child: i32 = std::fs::read_to_string(&pid_file).unwrap().parse().unwrap();
    assert_eq!(unsafe { libc::kill(child, libc::SIGSTOP) }, 0);
    let mut corrector = Corrector::from_config(
        "native_pgo", &nuwax_config::test_support::section("geometry"),
        &nuwax_config::test_support::section("robot"), &nuwax_config::test_support::section("transport"),
    ).unwrap();
    let Mode::Native(native) = &mut corrector.mode else { unreachable!() };
    native.runtime = Some(runtime);
    native.owner = Some(owner);
    native.ingress = Some(ingress);
    let stamp = 1_794_200_973_126_363_516;
    let frame = crate::mapper::tests::room_frame(204, stamp);
    let points: Vec<_> = frame.points_body.iter().map(|p| Point3::from(*p)).collect();
    let mut grid = nuwax_mapping::OccupancyGrid::new(8, 8, 0.05);
    grid.frame_id = "map".into();
    let request = native.request(&frame, &points, (&grid, stamp), 1);
    let buffer = Arc::new(Mutex::new(Vec::new()));
    let writer = LogBuffer(Arc::clone(&buffer));
    let subscriber = tracing_subscriber::fmt().json().with_ansi(false)
        .with_writer(move || writer.clone()).finish();
    let error = tracing::subscriber::with_default(subscriber, || native.submit(request)).unwrap_err();
    assert!(error.contains("request 1 timed out"));
    assert!(native.owner.is_none(), "故障关闭证据已经被消费，不能留待Drop丢弃");
    let text = String::from_utf8(buffer.lock().unwrap().clone()).unwrap();
    let logs: Vec<serde_json::Value> = text.lines().filter_map(|line| serde_json::from_str(line).ok())
        .filter(|row: &serde_json::Value| row["fields"]["message"] == "原生PGO请求超时取证").collect();
    assert_eq!(logs.len(), 1, "故障路径应一次记录完整证据：{text}");
    let fields = &logs[0]["fields"];
    assert_eq!(fields["request_sequence"], 1);
    for (stage, count) in [("submitted", 1), ("publish_returned", 1), ("native_received", 0), ("result_received", 0)] {
        assert_eq!(fields[format!("{stage}_count")], count);
        let last = &fields[format!("{stage}_last_timestamp_ns")];
        assert!(!last.is_null(), "每个阶段必须有最后观察时刻字段");
        if count == 0 { assert_eq!(last, "None"); } else { assert!(last.as_str().unwrap().starts_with("Some(")); }
    }
    assert!(fields["stop_reason"].as_str().unwrap().contains("RequestTimedOut"));
    assert!(fields["process_stop"].as_str().unwrap().contains("SupervisedShutdown"));
    assert!(fields["stdout_tail"].as_str().unwrap().contains("N6-20 stdout tail"));
    assert!(fields["stderr_tail"].as_str().unwrap().contains("N6-20 stderr tail"));
    std::fs::remove_dir_all(fixture).unwrap();
}

impl PgoUpdate {
    pub(crate) fn constant_for_test(first_stamp: u64, correction: Isometry3<f64>) -> Self {
        Self {
            latest: correction,
            timeline: CorrectionTimeline::new(vec![CorrectionKeyframe {
                timestamp: 0.,
                local_pose: Isometry3::identity(),
                optimized_pose: correction,
            }])
            .unwrap(),
            first_stamp,
        }
    }
}

#[test]
fn native_pgo_request_preserves_committed_nanoseconds_and_skips_uncommitted_frames() {
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

            let corrector = crate::mapper::tests::frames_without_candidate_keep_session_healthy(
                backend, corrector, &frame,
            );
            assert_eq!(corrector.sequence, 0, "后继帧不能进入校正器或分配请求序号");
            assert!(corrector.first_stamp.is_none());
            let Mode::Native(owner) = &corrector.mode else { unreachable!() };
            assert!(owner.runtime.is_none() && owner.owner.is_none() && owner.ingress.is_none(),
                "未提交本帧地图时不能启动原生校正请求");
        }
    }
}

fn compute_for_test(
    source: &[Point3<f64>],
    target: &[Point3<f64>],
    initial: Isometry3<f64>,
) -> (Isometry3<f64>, f64, bool) {
    let source = SourceCloud::from_rays(source.iter().map(|p| (*p, Point3::origin())));
    let target = TargetCache::new(target, RelocalizationConfig::default());
    let (pose, fitness, succeeded, _) =
        compute_relocalization(&source, &target, &[initial], Isometry3::identity());
    (pose, fitness, succeeded)
}

#[test]
fn relocalization_initials_keep_order_saved_height_and_odometry_tilt() {
    let current = Isometry3::translation(100.0, 0.0, 0.0);
    let saved = Isometry3::translation(8.0, -4.0, 1.3);
    let odom = Isometry3::from_parts(
        Translation3::new(2.0, 3.0, 0.4),
        UnitQuaternion::from_euler_angles(0.04, -0.06, 0.8),
    );
    let initials = relocalization_initials(
        current,
        Some(saved),
        Some(proto::PlanarSeed {
            x: 5.0,
            y: 6.0,
            yaw: 1.1,
        }),
        odom,
    );
    assert_eq!(initials.len(), 3);
    assert_transform(initials[0], current);
    assert_transform(initials[1] * odom, saved);
    let body = initials[2] * odom;
    assert_transform(
        body,
        Isometry3::from_parts(
            Translation3::new(5.0, 6.0, 1.3),
            UnitQuaternion::from_euler_angles(0.04, -0.06, 1.1),
        ),
    );
    assert_eq!(
        relocalization_initials(current, None, None, odom),
        vec![current]
    );
    let without_saved = relocalization_initials(
        current,
        None,
        Some(proto::PlanarSeed {
            x: 5.0,
            y: 6.0,
            yaw: 1.1,
        }),
        odom,
    );
    assert!(((without_saved[1] * odom).translation.z).abs() < 1e-9);
}

fn local_consensus_source() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for a in 0..47 {
        for b in 0..32 {
            let (x, y) = (-3.0 + a as f64 * 0.13, -2.0 + b as f64 * 0.13);
            points.extend([Point3::new(x, y, -1.0), Point3::new(x, y, 1.73)]);
        }
    }
    for z in 0..22 {
        let z = -1.0 + z as f64 * 0.13;
        for a in 0..47 {
            let x = -3.0 + a as f64 * 0.13;
            points.extend([Point3::new(x, -2.0, z), Point3::new(x, 2.03, z)]);
        }
        for b in 0..32 {
            let y = -2.0 + b as f64 * 0.13;
            points.extend([Point3::new(-3.0, y, z), Point3::new(2.98, y, z)]);
        }
    }
    points
}
fn assert_transform(actual: Isometry3<f64>, expected: Isometry3<f64>) {
    let error = actual.inverse() * expected;
    assert!(
        error.translation.vector.norm() < 0.05,
        "平移误差：{error:?}"
    );
    assert!(
        error.rotation.angle().to_degrees() < 1.0,
        "朝向误差：{error:?}"
    );
}

#[test]
fn relocalization_local_consensus_accepts_supported_seed() {
    let source = local_consensus_source();
    let truth = Isometry3::from_parts(
        Translation3::new(31.0, 18.0, 0.2),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7),
    );
    let target: Vec<_> = source.iter().map(|p| truth * p).collect();
    let initial = Isometry3::from_parts(
        Translation3::new(0.3, -0.4, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 10_f64.to_radians()),
    );
    let initial = Isometry3::from_parts(
        truth.translation * initial.translation,
        truth.rotation * initial.rotation,
    );
    let (actual, fitness, succeeded) = compute_for_test(&source, &target, initial);
    assert!(succeeded);
    assert!(fitness >= 0.45 && fitness <= 1.0);
    assert_transform(actual, truth);
}

#[test]
fn relocalization_local_consensus_uses_second_initial_after_first_fails() {
    let points = local_consensus_source();
    let truth = Isometry3::translation(80.0, 60.0, 0.2);
    let target_points: Vec<_> = points.iter().map(|p| truth * p).collect();
    let target = TargetCache::new(&target_points, RelocalizationConfig::default());
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    let odom = Isometry3::identity();
    let first = local_registration(&source, &target, odom, odom).unwrap();
    assert!(!first.mse.is_finite());
    let initials = relocalization_initials(
        odom,
        Some(truth),
        Some(proto::PlanarSeed {
            x: 150.0,
            y: 160.0,
            yaw: 0.0,
        }),
        odom,
    );
    let (actual, _, succeeded, _) =
        compute_relocalization(&source, &target, &initials, odom);
    assert!(succeeded);
    assert_transform(actual, truth);
}

#[test]
fn relocalization_local_error_continues_to_second_initial_and_then_global() {
    let points = local_consensus_source();
    let mut target_points = points.clone();
    // 远处目标在双精度中有限，超过原生点云单精度范围，用于触发现有内核错误通道。
    target_points.extend(points.iter().map(|p| Point3::new(1e40, p.y, p.z)));
    let target = TargetCache::new(&target_points, RelocalizationConfig::default());
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    let bad = Isometry3::translation(1e40, 0.0, 0.0);
    let odom = Isometry3::identity();
    assert!(local_registration(&source, &target, bad, odom).is_err());
    let (pose, fitness, succeeded, ambiguous) =
        compute_relocalization(&source, &target, &[bad, odom], odom);
    assert!(succeeded && !ambiguous);
    assert!(fitness >= 0.45);
    assert_transform(pose, odom);
    let global = relocation_pipeline_cached(&source, &target, RelocalizationConfig::default());
    assert!(global.fitness >= 0.45);
    let actual = compute_relocalization(&source, &target, &[bad, bad], odom);
    assert_eq!(actual.0, global.registration.transformation);
    assert_eq!(actual.1, global.fitness);
    assert_eq!(actual.2, global.registration.converged);
    assert_eq!(actual.3, global.ambiguous);
}

#[test]
fn relocalization_local_consensus_preserves_nonzero_anchor_and_reuses_registration() {
    let points = local_consensus_source();
    let truth = Isometry3::from_parts(
        Translation3::new(2.0, -3.0, 0.4),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7),
    );
    let target_points: Vec<_> = points.iter().map(|p| truth * p).collect();
    let target = TargetCache::new(&target_points, RelocalizationConfig::default());
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    let mut registration =
        Isometry3::from_parts(Translation3::new(2.03, -3.02, 0.41), truth.rotation);
    for _ in 0..2 {
        let (actual, _, succeeded, _) =
            compute_relocalization(&source, &target, &[registration], Isometry3::identity());
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
        UnitQuaternion::from_euler_angles(20_f64.to_radians(), 0.0, 0.0),
    );
    let target: Vec<_> = source.iter().map(|p| tilted * p).collect();
    let (actual, _, succeeded) = compute_for_test(&source, &target, tilted);
    assert!(!succeeded || is_upright(&actual, GRAVITY_TILT_MAX_DEG));
}

#[tokio::test]
async fn relocalization_global_ambiguity_keeps_mapper_transform_and_exact_message() {
    use crate::mapper::MapperConfig;
    let source = local_consensus_source();
    let a = Isometry3::translation(6.0, 0.0, 0.0);
    let b = Isometry3::translation(18.0, 0.0, 0.0);
    let mut frame = crate::mapper::tests::room_frame(991, 1_000_000_000);
    frame.ray_origin_body = [6.0, 0.0, 0.0];
    frame.points_body = source
        .iter()
        .flat_map(|p| [a * p, b * p])
        .map(|p| [p.x, p.y, p.z])
        .collect();
    let mapper = Mapper::start(991, MapperConfig::default()).unwrap();
    mapper.feed(frame);
    let snapshot = mapper.snapshot().await.unwrap();
    mapper.shutdown().await.unwrap();
    let loaded = Mapper::load(992, MapperConfig::default(), snapshot, true).unwrap();
    let before = Isometry3::translation(100.0, 100.0, 0.0);
    loaded.set_map_from_odom(before).await.unwrap();
    let mut current = crate::mapper::tests::room_frame(992, 2_000_000_000);
    current.points_body = source.iter().map(|p| [p.x, p.y, p.z]).collect();
    current.ray_origin_body = [0.0; 3];
    let result = Corrector::relocalize(&loaded, vec![current], &mut None, None, None)
        .await
        .unwrap();
    assert!(!result.succeeded);
    assert_eq!(
        result.message,
        "重定位有歧义：两个位置同样吻合，请转动或移动一小段后重试"
    );
    assert!(result.pose.is_none());
    assert_eq!(loaded.map_from_odom(), before);
    loaded.shutdown().await.unwrap();
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
