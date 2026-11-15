use super::*;

#[test]
fn recording_budget_stops_open_scan_and_progress_writes_while_accumulation_continues() {
    for stage in ["open", "scan", "progress"] {
        let root = std::env::temp_dir().join(format!("g1-budget-{}", uuid::Uuid::new_v4()));
        let mut map = CloudMap::new(940, None, None).unwrap();
        map.raw_directory = Some(root.clone());
        let mut frame = crate::mapper::tests::room_frame(940, 1_000_000_000);
        if stage != "open" {
            map.record(1, &frame, Isometry3::identity()).unwrap();
        }
        let source = root.join("source-0");
        std::fs::create_dir_all(&source).unwrap();
        let used: u64 = std::fs::read_dir(&source).unwrap().map(|entry| entry.unwrap().metadata().unwrap().len()).sum();
        let filler = File::create(source.join("budget-used")).unwrap();
        filler.set_len(nuwax_cloud_cleanup::scan_log::storage_budget_bytes().unwrap() - used).unwrap();
        let log_before = map.logs.values().next().map(|log| std::fs::metadata(&log.path).unwrap().len());
        frame.odom_pose.timestamp_ns += if stage == "progress" { 1 } else { 1_000_000_000 };
        let points: Vec<_> = frame.points_body.iter().map(|p| Point3::from(*p)).collect();
        map.accumulate(2, &frame, &points, frame.ray_origin_body, Isometry3::identity())
            .expect("预算耗尽不能使 accumulate 失败");
        assert!(!map.recording, "{stage}");
        assert!(map.take_recording_limit());
        assert!(!map.take_recording_limit(), "预算停止只通知一次");
        if let Some(before) = log_before {
            assert_eq!(map.logs.values().next().unwrap().path.metadata().unwrap().len(), before);
        }
        std::fs::remove_file(source.join("budget-used")).unwrap();
        frame.odom_pose.timestamp_ns += 2_000_000_000;
        let before = map.scan_prefix().unwrap();
        map.accumulate(3, &frame, &points, frame.ray_origin_body, Isometry3::identity()).unwrap();
        assert_eq!(map.poses.len(), 2);
        assert_eq!(map.scan_prefix().unwrap(), before, "释放空间后本会话也不恢复记录");
    }
}

#[test]
fn recording_keeps_other_io_errors_on_the_existing_failure_path() {
    let path = std::env::temp_dir().join(format!("g1-record-io-{}", uuid::Uuid::new_v4()));
    std::fs::write(&path, b"not a directory").unwrap();
    let mut map = CloudMap::new(941, None, None).unwrap();
    map.raw_directory = Some(path.clone());
    let frame = crate::mapper::tests::room_frame(941, 1_000_000_000);
    assert!(map.record(1, &frame, Isometry3::identity()).is_err());
    assert!(map.recording);
    assert!(!map.take_recording_limit());
    drop(map);
    std::fs::remove_file(path).unwrap();
}

#[test]
fn display_removals_match_full_scan_for_every_frame() {
    let mut total_removed = 0;
    for size in [VOXEL_SIZE, VOXEL_SIZE * 2., VOXEL_SIZE * 4.] {
        let mut map = CloudMap::new(931, None, None).unwrap();
        map.disable_recording();
        map.voxel_size = size;
        let transform = Isometry3::new(
            nalgebra::Vector3::new(0.37, -0.19, 0.08),
            nalgebra::Vector3::new(0.02, -0.03, 0.27),
        );
        for index in 0..8 {
            let (body, _) = crate::dynamic_scene::tests::corridor(
                (index < 2).then_some(0.), index as f64 * 0.3,
            );
            let frame = MappingFrame {
                session_id: 931,
                odom_pose: pose_from_isometry(Isometry3::identity(), "odom", index + 1),
                points_body: body.iter().map(|p| [p.x, p.y, p.z]).collect(),
                ray_origin_body: crate::dynamic_scene::tests::ORIGIN,
                mapping_only: false, sensor_id: "主雷达".into(), registered_cloud: None,
            };
            let points: Vec<_> = body.iter().map(|p| transform * p).collect();
            let origin = transform * Point3::from(frame.ray_origin_body);
            let before = map.voxels.clone();
            let (dynamic, _, _) = map.accumulate(index + 1, &frame, &points,
                [origin.x, origin.y, origin.z], transform).unwrap();
            let hits: VoxelIndex = points.iter()
                .filter_map(|p| voxel_key([p.x, p.y, p.z], size)).collect();
            // 保留旧实现的逐键判定作为独立对照，比较生产 accumulate 实际删除结果。
            let expected: VoxelIndex = before.iter().copied().filter(|key| {
                !hits.contains(key) && dynamic.clears(voxel_center_f64(*key, size))
            }).collect();
            let actual: VoxelIndex = before.difference(&map.voxels).copied().collect();
            assert_eq!(actual, expected, "体素边长={size}，帧={index}");
            total_removed += actual.len();
        }
    }
    assert!(total_removed > 0, "必须实际覆盖历史展示体素清除");
}
#[test]
fn dense_delta_returns_only_the_unread_surviving_tail() {
    let mut map = CloudMap::new(41, None, None).unwrap();
    let old: Vec<_> = (0..8)
        .map(|i| Point3::new(i as f64 * 0.1, 0., 0.5))
        .collect();
    map.dense.append(&old).unwrap();
    map.dense_snapshot = map.dense.snapshot(100, 1, "41");
    let first = map
        .dense_chunk(
            proto::DenseCloudChunkRequest {
                expected_epoch: String::new(),
                offset_points: 0,
                max_points: 100,
                allow_delta: false,
            },
            "map",
        )
        .unwrap();
    assert_eq!(first.total_points, 8);
    let keys = [
        voxel_key([old[1].x, old[1].y, old[1].z], VOXEL_SIZE).unwrap(),
        voxel_key([old[5].x, old[5].y, old[5].z], VOXEL_SIZE).unwrap(),
    ];
    map.dense.remove_voxels(&keys, VOXEL_SIZE).unwrap();
    let added = Point3::new(3., 4., 0.5);
    map.dense.append(&[added]).unwrap();
    map.dense_snapshot = map.dense.snapshot(200, 2, "41");
    let delta = map
        .dense_chunk(
            proto::DenseCloudChunkRequest {
                expected_epoch: first.epoch.clone(),
                offset_points: 8,
                max_points: 100,
                allow_delta: true,
            },
            "map",
        )
        .unwrap();
    assert_eq!(delta.base_epoch, first.epoch);
    assert_eq!(delta.base_offset_points, 8);
    assert_eq!(delta.offset_points, 6);
    assert_eq!(delta.total_points, 7);
    assert_eq!(delta.removed_ranges, vec![1, 1, 5, 1]);
    assert_eq!(
        delta.xyz_f32_le,
        [added.x, added.y, added.z]
            .into_iter()
            .flat_map(|v| (v as f32).to_le_bytes())
            .collect::<Vec<_>>()
    );
}
#[test]
fn cloud_shm_cleanup_only_unlinks_its_owned_inode() {
    let path =
        std::env::temp_dir().join(format!("geometry-cloud-cleanup-{}", uuid::Uuid::new_v4()));
    let writer = CloudMapShmWriter::new(&path).unwrap();
    assert!(path.exists());
    drop(writer);
    assert!(!path.exists());
    let writer = CloudMapShmWriter::new(&path).unwrap();
    let replacement = path.with_extension("replacement");
    std::fs::write(&replacement, b"replacement").unwrap();
    std::fs::rename(&replacement, &path).unwrap();
    drop(writer);
    assert_eq!(std::fs::read(&path).unwrap(), b"replacement");
    std::fs::remove_file(path).unwrap();
}

#[test]
fn synchronous_recording_selects_against_last_saved_frame_per_sensor_and_survives_pgo() {
    use nuwax_cloud_cleanup::scan_log::{SavePolicy, ScanReader};
    let root = std::env::temp_dir().join(format!("e7b-selected-scans-{}", uuid::Uuid::new_v4()));
    let policy = SavePolicy { translation_m: 0.1, rotation_deg: 3., interval_s: 1. };
    let mut map = CloudMap::new(920, None, None).unwrap();
    map.raw_directory = Some(root.clone());
    map.save_policy = policy;
    let frame = |sensor: &str, stamp, x, yaw| MappingFrame {
        session_id: 920,
        odom_pose: pose_from_isometry(Isometry3::new(nalgebra::Vector3::new(x, 0., 0.), nalgebra::Vector3::new(0., 0., yaw)), "odom", stamp),
        points_body: vec![[1., 0., 0.]], ray_origin_body: [0.; 3],
        mapping_only: false, sensor_id: sensor.into(), registered_cloud: None,
    };
    for (sequence, sensor, stamp, x, yaw) in [
        (1, "主雷达", 1_000_000_000, 0., 0.),
        (2, "主雷达", 1_100_000_000, 0.06, 0.),
        (3, "主雷达", 1_200_000_000, 0.1, 0.),
        (4, "主雷达", 1_300_000_000, 0.1, 0.06),
        (5, "副雷达", 1_400_000_000, 0.1, 0.06),
        (6, "主雷达", 1_600_000_000, 0.12, 0.06),
        (7, "主雷达", 2_300_000_000, 0.12, 0.06),
        (8, "主雷达", 2_400_000_000, 0.13, 0.06),
    ] {
        map.record(sequence, &frame(sensor, stamp, x, yaw), Isometry3::identity()).unwrap();
    }
    let mut replacement = CloudMap::new(920, None, None).unwrap();
    replacement.stamp = 2_400_000_000;
    replacement.save_policy = policy;
    replacement.adopt_publication(&mut map).unwrap();
    replacement.record(9, &frame("主雷达", 2_500_000_000, 0.15, 0.06), Isometry3::identity()).unwrap();
    for (sensor, expected, observed) in [("主雷达", vec![1, 3, 4, 7], 2_500_000_000), ("副雷达", vec![5], 1_400_000_000)] {
        let log = &replacement.logs[sensor];
        log.sync().unwrap();
        let mut reader = ScanReader::open(&log.path).unwrap();
        let mut sequences = Vec::new();
        while let Some(scan) = reader.next_scan().unwrap() { sequences.push(scan.metadata.source_sequence); }
        assert_eq!(sequences, expected, "{sensor}");
        assert_eq!(reader.observed_until_ns, observed, "跳过扫描仍记录源时间进度");
    }
    drop(replacement);
    drop(map);
}
