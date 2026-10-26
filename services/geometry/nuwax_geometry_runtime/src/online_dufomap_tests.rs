//! 真实拥有方接线：原始扫描、历史撤销、冻结读者及会话切换。
use super::*;
use crate::admitted_cloud::AcceptedMappingCloud;
use crate::dynamic_backend::{test_backend, Backend};
use crate::lio_ingress::{AcceptedRegisteredCloud, CorrelatedOdometry, RegistrationBinding};
use crate::mapping_backend::{
    self, MappingBackendComputeConfig, MappingBackendInput, MappingBackendSpec,
};

fn admitted(
    points: Vec<Point3<f64>>,
    origin: [f64; 3],
    stamp: u64,
    generation: u64,
) -> AcceptedRegisteredCloud {
    AcceptedRegisteredCloud::from_map_points_for_test(
        points,
        stamp,
        stamp,
        "map",
        "base_link",
        RegistrationBinding {
            runtime_generation: 1,
            cloud_source_sequence: (stamp / 100_000_000 + 1) as i64,
            correlated_odometry: CorrelatedOdometry {
                source_sequence: (stamp / 100_000_000 + 1) as i64,
                source_timestamp_ns: stamp as i64,
            },
            mapping_session_id: "在线DUFOMap行为".into(),
            mapping_session_generation: generation,
            map_frame_generation: 0,
            frozen_odom_from_body: nalgebra::Isometry3::identity(),
            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
            registration_map_from_odom: nalgebra::Isometry3::identity(),
            registration_correction_revision: 0,
            sensor_origin_map: Some(origin),
            frame_capture_relocalization_epoch: Some(0),
        },
    )
}
fn receipt(frame: u64) -> OccupancyCommitReceipt {
    OccupancyCommitReceipt {
        commit_id: crate::mapping_commit::MappingCommitId {
            boot_nonce: 81,
            sequence: frame,
        },
        session_id: "在线DUFOMap行为".into(),
        session_generation: 1,
        occupancy_revision: frame,
        source_timestamp_ns: frame * 100_000_000,
        backend_kind: mapping_backend::MappingBackendKind::Voxel,
        backend_generation: 1,
    }
}
fn dense_points(snapshot: &DenseSnapshot) -> Vec<[f32; 3]> {
    let mut points = Vec::new();
    while points.len() < snapshot.point_count as usize {
        points.extend(
            snapshot
                .read(points.len() as u64, crate::dense_cloud::CHUNK_POINTS)
                .unwrap()
                .chunks_exact(12)
                .map(|b| {
                    std::array::from_fn(|i| {
                        f32::from_le_bytes(b[i * 4..i * 4 + 4].try_into().unwrap())
                    })
                }),
        );
    }
    points
}

#[test]
fn dufomap_history_matches_final_native_verdict_and_keeps_static_structure() {
    let _backend = test_backend(Backend::DufoMap);
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
    writer.begin_session(1).unwrap();
    let mut oracle =
        nuwax_dufomap_native::DufoMap::new(&crate::dynamic_backend::online_params()).unwrap();
    let mut originals = Vec::new();
    let mut truth = Vec::new();
    let mut source_keys = Vec::new();
    let mut initial_dense = None;
    let config = MappingBackendComputeConfig {
        resolution: 0.05,
        initial_safe_radius_meters: 0.0,
        window_capacity: 1_000_000,
    };
    let mut kernels = [
        MappingBackendSpec::Voxel.into_kernel(config.window_capacity),
        MappingBackendSpec::raytracing_defaults().into_kernel(config.window_capacity),
    ];
    for frame in 1..=22 {
        let person = (7..=16)
            .contains(&frame)
            .then(|| -0.5 + (frame - 7) as f64 * 0.12);
        let (points, labels) = corridor(person, (frame % 3) as f64);
        let xyz: Vec<_> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        oracle.insert(&xyz, ORIGIN.map(|v| v as f32), true).unwrap();
        source_keys.extend(points.iter().map(|p| voxel_key([p.x,p.y,p.z], VOXEL_SIZE).unwrap()));
        originals.extend(xyz);
        truth.extend(labels);
        let source = admitted(points.clone(), ORIGIN, frame * 100_000_000, 1);
        let shared = writer.classify_admitted_scan(&source).unwrap();
        assert_eq!(
            shared,
            writer.classify_admitted_scan(&source).unwrap(),
            "同一接纳扫描复用单次积分结果"
        );
        assert!(
            source.scan().dynamic_frame().is_none(),
            "新后端不得生成或归档旧帧决策"
        );
        assert!(source.scan().static_indices().is_some());
        let input = MappingBackendInput {
            session_id: "在线DUFOMap行为".into(),
            session_generation: 1,
            backend_generation: 1,
            frame_sequence: frame,
            source_timestamp_ns: frame * 100_000_000,
            received_at_ns: frame * 100_000_000,
            registered_frame_id: "map".into(),
            sensor_origin_map: ORIGIN,
            dynamic_frame: Some(shared.clone()),
            registered_points_map: points.clone(),
        };
        let wire = crate::dynamic_backend::MappedFrame::decode(&shared.encode().unwrap()).unwrap();
        assert_eq!(wire, shared, "隔离射线内核接收相同静态判定与撤销证据");
        for kernel in &mut kernels {
            let candidate = mapping_backend::compute_candidate_in_place_with_interval(
                kernel, &input, config, 0,
            )
            .unwrap()
            .unwrap();
            assert_eq!(
                candidate.accepted_points_map, points,
                "近期障碍交付完整原始扫描，包括本帧行人"
            );
        }
        let accepted = AcceptedMappingCloud::from(source);
        assert!(writer.accumulate_mapping_scan_with_source(
            1,
            &points,
            Some(ORIGIN),
            frame * 100_000_000,
            Some(&accepted)
        ));
        writer
            .commit_accumulated_snapshot(&receipt(frame))
            .unwrap()
            .unwrap();
        if frame == 10 {
            let snapshot = writer.committed_dense_cloud().unwrap().unwrap();
            let points = dense_points(&snapshot);
            initial_dense = Some((snapshot, points));
        }
    }
    let flags = oracle.seen_free(&originals).unwrap();
    let expected: Vec<_> = originals
        .iter()
        .zip(&flags)
        .filter_map(|(p, removed)| (!removed).then_some(*p))
        .collect();
    let snapshot = writer.committed_dense_cloud().unwrap().unwrap();
    assert_eq!(
        dense_points(&snapshot),
        expected,
        "历史撤销后完整静态点须逐点等于结束时原生判定，含重复点和顺序"
    );
    let expected_keys: HashSet<_> = source_keys.iter().zip(&flags).filter_map(|(k, free)| (!free).then_some(*k)).collect();
    assert_eq!(writer.voxels.len(), expected_keys.len(), "可见静态体素数须与结束时判定一致");
    assert!(writer.voxels.iter().all(|k| expected_keys.contains(k)), "可见图不能残留已判为空的历史体素");
    let fine = writer.committed_fine_cloud().unwrap();
    let mut reader = nuwax_map_chunks::RegionReader::new(fine.repository.clone(), 4*1024*1024);
    let mut fine_keys = HashSet::new();
    for block in &fine.manifest().blocks {
        fine_keys.extend(reader.block(block).unwrap().keys().map(|&local| { let k = nuwax_map_chunks::global_key(block.key, local).unwrap(); (k[0],k[1],k[2]) }));
    }
    assert_eq!(fine_keys.len(), expected_keys.len(), "精细静态体素数须与结束时判定一致");
    assert!(fine_keys.iter().all(|k| expected_keys.contains(k)), "精细图不能残留已判为空的历史体素");
    let deleted_people = flags
        .iter()
        .zip(&truth)
        .filter(|(gone, person)| **gone && **person)
        .count();
    let deleted_static = flags
        .iter()
        .zip(&truth)
        .filter(|(gone, person)| **gone && !**person)
        .count();
    assert!(deleted_people > 0, "横穿后被实测为空的行人历史必须真正删除");
    assert_eq!(deleted_static, 0, "墙面、地面和天花板必须保留");
    let (pinned, before_removal) = initial_dense.unwrap();
    let pinned_points = dense_points(&pinned);
    assert_eq!(pinned_points, before_removal, "后续历史撤销不能改写已领取快照的点序与内容");
    assert_eq!(
        pinned_points.len(),
        pinned.point_count as usize,
        "后续撤销不能改变已冻结读者"
    );
    assert!(snapshot.point_count < originals.len() as u64);
    let (archive, _) = snapshot.archive("map").unwrap();
    let restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
    assert_eq!(
        dense_points(
            &restored
                .snapshot(&snapshot.receipt, "在线DUFOMap行为")
                .unwrap()
        ),
        expected
    );
    let (points, _) = corridor(Some(0.0), 0.0);
    let mut installed = CloudMapShmWriter::new_private_for_test().unwrap();
    installed.install_prepared_accumulator(&receipt(22), writer.take_prepared_accumulator()).unwrap();
    let xyz: Vec<_> = points.iter().map(|p| [p.x as f32,p.y as f32,p.z as f32]).collect();
    oracle.insert(&xyz, ORIGIN.map(|v| v as f32), true).unwrap();
    let expected_dynamic = oracle.seen_free(&xyz).unwrap();
    assert!(expected_dynamic.iter().any(|v| *v));
    let source = admitted(points.clone(), ORIGIN, 2_300_000_000, 1);
    let actual = installed.classify_admitted_scan(&source).unwrap();
    assert_eq!(points.iter().map(|p| actual.is_dynamic([p.x,p.y,p.z])).collect::<Vec<_>>(), expected_dynamic,
        "PGO安装须保留冻结里程计系空闲证据与原日志拥有方");
    installed.begin_session(2).unwrap();
    let writer = &mut installed;
    let source = admitted(points.clone(), ORIGIN, 1, 2);
    let result = writer.classify_admitted_scan(&source).unwrap();
    assert!(
        points.iter().all(|p| !result.is_dynamic([p.x, p.y, p.z])),
        "新会话不能继承旧空闲证据"
    );
    assert!(writer.accumulate_mapping_scan_with_source(
        2,
        &points,
        Some(ORIGIN),
        1,
        Some(&source.into())
    ));
}

#[test]
fn both_backends_record_complete_v2_scan_before_static_selection() {
    use nuwax_cloud_cleanup::scan_log::{session_path, ScanReader, Session, StorageWorker};
    let root = std::env::temp_dir().join(format!("nuwax-g3-raw-{}", uuid::Uuid::new_v4()));
    struct Restore(Option<std::ffi::OsString>);
    impl Drop for Restore {
        fn drop(&mut self) {
            match &self.0 {
                Some(value) => std::env::set_var("NUWAX_RAW_SCAN_DIRECTORY", value),
                None => std::env::remove_var("NUWAX_RAW_SCAN_DIRECTORY"),
            }
        }
    }
    let _restore = Restore(std::env::var_os("NUWAX_RAW_SCAN_DIRECTORY"));
    std::env::set_var("NUWAX_RAW_SCAN_DIRECTORY", &root);
    let worker = StorageWorker::start().unwrap();
    let points = vec![
        Point3::new(1.1, 0.3, 0.7),
        Point3::new(1.1, 0.3, 0.7),
        Point3::new(-2.1, 0.4, 1.2),
    ];
    for (generation, backend) in [(1, Backend::DufoMap), (2, Backend::Legacy)] {
        let _backend = test_backend(backend);
        let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
        let source = admitted(
            points.clone(),
            [0.28945, 0.0, -0.046825],
            1_000_000_000,
            generation,
        );
        writer.classify_admitted_scan(&source).unwrap();
    }
    drop(worker);
    for generation in [1, 2] {
        let path = session_path(
            &root,
            &Session {
                session_id: "在线DUFOMap行为".into(),
                generation,
            },
        );
        let mut reader = ScanReader::open(&path).unwrap();
        let saved = reader.next_scan().unwrap().unwrap();
        assert_eq!(
            saved.points,
            points
                .iter()
                .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                .collect::<Vec<_>>(),
            "后端切换不能过滤日志原始点、重复点或顺序"
        );
        assert_eq!(
            saved.metadata.sensor_origin_body,
            Some([0.28945f32, 0.0, -0.046825])
        );
        assert!(reader.next_scan().unwrap().is_none());
    }
    std::fs::remove_dir_all(root).unwrap();
}
