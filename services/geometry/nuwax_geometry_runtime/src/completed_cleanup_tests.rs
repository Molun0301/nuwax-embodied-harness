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
                generation: 1,
                frame_generation: 0,
                producer_generation: 1,
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
