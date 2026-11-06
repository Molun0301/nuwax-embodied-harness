use super::*;
#[test]
fn offline_cleanup_removes_person_from_unfiltered_saved_assets_and_keeps_static() {
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    use nuwax_cloud_cleanup::scan_log::{ScanLog, ScanMetadata, Session};
    let folder =
        std::env::temp_dir().join(format!("neh-dufo-saved-{}", uuid::Uuid::new_v4().simple()));
    std::fs::create_dir_all(&folder).unwrap();
    let session = Session {
        session_id: "unfiltered".into(),
        generation: 1,
    };
    let mut log = ScanLog::open(&folder.join("scans"), session.clone()).unwrap();
    let identity = Matrix4::identity().as_slice().try_into().unwrap();
    let mut dense = Vec::new();
    let mut human = HashSet::new();
    let mut static_keys = HashSet::new();
    for frame in 0..60 {
        let (points, truth) = corridor(
            (frame < 30).then_some(-1.0 + frame as f64 * 2.0 / 29.0),
            (frame % 3) as f64,
        );
        let stamp = (frame + 1) * 100_000_000;
        let metadata = ScanMetadata {
            source_timestamp_ns: stamp,
            source_sequence: frame + 1,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: identity,
            map_from_odom: identity,
            sensor_origin_body: Some(ORIGIN.map(|v| v as f32)),
            observation_minimum_range_m: None,
            point_count: points.len(),
        };
        log.append(
            &metadata,
            points.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            None,
        )
        .unwrap();
        log.progress(stamp, 0).unwrap();
        for (p, on_person) in points.iter().zip(truth) {
            let p = [p.x, p.y, p.z];
            let key = asset_key(p, 0.05);
            if on_person {
                if p[2] > 0.35 {
                    human.insert(key);
                }
            } else {
                static_keys.insert(key);
            }
            dense.push(p);
        }
    }
    log.sync().unwrap();
    let original: HashSet<_> = dense.iter().map(|&p| asset_key(p, 0.05)).collect();
    let mut writer = nuwax_map_chunks::Writer::new(
        nuwax_map_chunks::Repository::temporary().unwrap(),
        0.05,
        4 * 1024 * 1024,
    )
    .unwrap();
    writer.seed_unknown(original.iter().copied()).unwrap();
    let sealed = writer
        .seal(
            nuwax_map_chunks::Domain {
                session: session.session_id.clone(),
                },
            1,
            6_000_000_000,
        )
        .unwrap();
    let fine = folder.join("fine.bin");
    let (mut archive, _) =
        nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
    std::io::copy(&mut archive, &mut File::create(&fine).unwrap()).unwrap();
    let dense_path = folder.join("dense.bin");
    write_dense(&dense_path,serde_json::json!({"session_id":session.session_id,"session_generation":1,"cloud_revision":0,"source_timestamp_ns":6_000_000_000u64}),&dense).unwrap();
    let request = Request {
        fine,
        dense: dense_path,
        output: folder.join("cleaned"),
        strength_percent: 50,
        scan_logs: Vec::new(),
        scan_log: Some(log.path.clone()),
        final_poses: None,
        source_timestamp_ns: 6_000_000_000,
        backend: None,
        occupancy: None,
        occupancy_descriptor: None,
        cancel_file: None,
    };
    let report = prepare(&request).unwrap();
    assert!(report.offline_complete);
    assert_eq!(report.offline_frames, 60);
    let after: HashSet<_> = CompletedMap::read(&request.output.join("fine.bin"))
        .unwrap()
        .keys
        .into_iter()
        .collect();
    let human: HashSet<_> = human.difference(&static_keys).copied().collect();
    assert!(!human.is_empty());
    let removed = human.difference(&after).count();
    assert!(
        removed as f64 / human.len() as f64 > 0.95,
        "未过滤保存地图的行人身体应被后续射线移除：{removed}/{}",
        human.len()
    );
    assert!(static_keys.is_subset(&after), "墙与地面应完整保留");
    assert!(after.is_subset(&original));
    assert_eq!(
        report.preview.candidate_points,
        original.difference(&after).count()
    );
    assert!(report.dense_after < report.dense_before);
    drop(log);
    std::fs::remove_dir_all(folder).unwrap();
}
#[test]
fn final_pose_transforms_body_returns_and_sensor_origin_together() {
    use nuwax_cloud_cleanup::scan_log::ScanMetadata;
    let identity = Matrix4::identity().as_slice().try_into().unwrap();
    let metadata = ScanMetadata {
        source_timestamp_ns: 10,
        source_sequence: 1,
        sensor: "lidar".into(),
        source_generation: 1,
        odom_from_body: identity,
        map_from_odom: identity,
        sensor_origin_body: Some([0.28945, 0., -0.046825]),
        observation_minimum_range_m: None,
        point_count: 1,
    };
    let scan = LoggedScan {
        metadata,
        points: vec![[-2., 1., 3.]],
        intensities: None,
    };
    let corrected = nalgebra::Isometry3::new(
        nalgebra::Vector3::new(-5., 2., 1.),
        nalgebra::Vector3::new(0., 0., std::f64::consts::FRAC_PI_2),
    );
    let poses = HashMap::from([(
        10,
        corrected.to_homogeneous().as_slice().try_into().unwrap(),
    )]);
    let (points, origin) = scan_points(&scan, &poses).unwrap();
    for (&actual, expected) in points[0].iter().zip([-6., 0., 4.]) {
        assert!(
            (actual - expected).abs() < 1e-6,
            "单精度地图坐标应保持最终三维变换精度"
        );
    }
    for (actual, expected) in origin.into_iter().zip([-5., 2.28945, 0.953175]) {
        assert!((actual - expected).abs() < 1e-6);
    }
    let (uncorrected, _) = scan_points(&scan, &HashMap::new()).unwrap();
    assert_eq!(uncorrected, scan.points);
}

#[tokio::test]
async fn preview_and_apply_keep_review_files_and_install_only_new_geometry() {
    use crate::{
        map_store::MapStore,
        mapper::{Mapper, MapperConfig},
    };
    let folder = std::env::temp_dir().join(format!("neh-cleanup-command-{}", uuid::Uuid::new_v4()));
    let _guard = CleanupDirectory(folder.clone());
    let store = MapStore::new(folder.join("maps"));
    let mapper = Mapper::start(
        211,
        MapperConfig {
            compute: crate::mapping_backend::MappingBackendComputeConfig {
                window_capacity: 32_768,
                ..MapperConfig::default().compute
            },
            ..Default::default()
        },
    )
    .unwrap();
    mapper.feed(crate::mapper::tests::room_frame(211, 1_000_000_000));
    let snapshot = mapper.snapshot().await.unwrap();
    store.save("源图", None, snapshot).unwrap();
    mapper.shutdown().await.unwrap();
    let source = store.root().join("源图/geometry");
    let source_fine = std::fs::read(source.join("fine.map")).unwrap();
    let source_occupancy = std::fs::read(source.join("occupancy.pb")).unwrap();
    let work = folder.join("预览");
    let args: Vec<_> = [
        "preview".into(),
        "--map".into(),
        "源图".into(),
        "--strength".into(),
        "77".into(),
        "--out".into(),
        work.as_os_str().to_owned(),
    ]
    .into();
    run_command(&args, &store).unwrap();
    for member in [
        "preview.json",
        "source.xyzf32",
        "candidates.xyzf32",
        "result.xyzf32",
        "candidate/geometry/map.json",
    ] {
        assert!(
            work.join(member).is_file(),
            "必须保留供网关审阅的文件：{member}"
        );
    }
    let report: Report =
        serde_json::from_slice(&std::fs::read(work.join("preview.json")).unwrap()).unwrap();
    assert_eq!(report.preview.settings.strength_percent, 77);
    assert_eq!(std::fs::read(source.join("fine.map")).unwrap(), source_fine);
    assert_eq!(
        std::fs::read(source.join("occupancy.pb")).unwrap(),
        source_occupancy
    );
    assert!(!store.exists("目标图"));
    let apply: Vec<_> = [
        "apply".into(),
        "--work".into(),
        work.as_os_str().to_owned(),
        "--map".into(),
        "目标图".into(),
    ]
    .into();
    run_command(&apply, &store).unwrap();
    let installed = store.load("目标图").unwrap();
    let fine = nuwax_map_chunks::ArchiveReader::open(
        installed
            .cloud
            .fine_archive
            .as_ref()
            .unwrap()
            .try_clone()
            .unwrap(),
    )
    .unwrap();
    assert_eq!(installed.cloud.revision, fine.root().revision());
    assert_eq!(
        installed.cloud.display.as_ref(),
        std::fs::read(work.join("result.xyzf32")).unwrap()
    );
    assert!(
        run_command(&apply, &store).is_err(),
        "已有目标地图必须拒绝覆盖"
    );
    assert!(work.join("preview.json").exists());
    assert!(work.join("candidate/geometry/map.json").exists());
    assert!(!store.root().join("目标图/meta.json").exists());
    assert!(std::fs::read_dir(store.root().join("目标图"))
        .unwrap()
        .all(|entry| !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with('.')));
}
