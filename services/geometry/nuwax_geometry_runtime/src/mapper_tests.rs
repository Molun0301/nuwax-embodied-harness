use super::*;
use crate::corrector::Corrector;

pub(crate) fn room_frame(session: u64, stamp: u64) -> MappingFrame {
    let (points, _) = crate::dynamic_scene::tests::corridor(None, 0.);
    MappingFrame {
        session_id: session,
        odom_pose: pose_from_isometry(Isometry3::identity(), "odom", stamp),
        points_body: points
            .into_iter()
            .step_by(4)
            .map(|p| [p.x, p.y, p.z])
            .collect(),
        ray_origin_body: crate::dynamic_scene::tests::ORIGIN,
        mapping_only: false,
        sensor_id: "lidar".into(),
        registered_cloud: None,
    }
}
fn config() -> MapperConfig {
    MapperConfig {
        map_name: "实测地图".into(),
        compute: MappingBackendComputeConfig {
            window_capacity: 32_768,
            ..MapperConfig::default().compute
        },
        ..Default::default()
    }
}

#[tokio::test]
async fn session_switch_discards_old_frames_and_save_flushes_the_latest_real_frame() {
    let mapper = Mapper::start(21, config()).unwrap();
    mapper.feed(room_frame(20, 1_000_000_000));
    assert!(mapper.snapshot().await.unwrap().occupancy.is_none());
    mapper.feed(room_frame(21, 1_000_000_000));
    mapper.feed(room_frame(21, 1_001_000_000));
    let saved = mapper.snapshot().await.unwrap();
    assert!(
        mapper.error_rx().borrow().is_none(),
        "{:?}",
        *mapper.error_rx().borrow()
    );
    assert_eq!(
        saved
            .occupancy
            .as_ref()
            .unwrap()
            .meta
            .as_ref()
            .unwrap()
            .timestamp_ns,
        1_001_000_000,
        "检查点必须包含节拍内最后一帧，不能保存旧候选"
    );
    assert!(saved.cloud.dense.as_ref().unwrap().point_count > 0);
    assert_eq!(saved.scans.len(), 1);
    assert!(mapper.error_rx().borrow().is_none());
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn first_odometry_pose_anchors_translation_and_yaw_once() {
    let mapper = Mapper::start(22, config()).unwrap();
    let mut first = room_frame(22, 1_000_000_000);
    first.odom_pose = pose_from_isometry(
        Isometry3::from_parts(
            Translation3::new(3., -2., 0.7),
            UnitQuaternion::from_euler_angles(0., 0., 0.4),
        ),
        "odom",
        1_000_000_000,
    );
    mapper.feed(first.clone());
    mapper.snapshot().await.unwrap();
    let anchored = mapper.transform_pose(&first.odom_pose);
    let position = anchored.position.unwrap();
    assert!(position.x.abs() < 1e-9 && position.y.abs() < 1e-9 && position.z.abs() < 1e-9);
    let transform = mapper.map_from_odom();
    let mut next = first;
    next.odom_pose.timestamp_ns += 100_000_000;
    next.odom_pose.position.as_mut().unwrap().x += 0.1;
    mapper.feed(next);
    mapper.snapshot().await.unwrap();
    assert_eq!(mapper.map_from_odom(), transform);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn loaded_map_stays_read_only_while_pose_and_recent_obstacles_continue() {
    let source = Mapper::start(23, config()).unwrap();
    source.feed(room_frame(23, 1_000_000_000));
    let baseline = source.snapshot().await.unwrap();
    source.shutdown().await.unwrap();
    let loaded = Mapper::load(24, config(), baseline.clone(), true).unwrap();
    let mut pose = loaded.pose_rx();
    let mut recent = loaded.recent_cloud_rx();
    let mut frame = room_frame(24, 2_000_000_000);
    frame.odom_pose.position.as_mut().unwrap().x = 1.2;
    loaded.feed(frame);
    tokio::time::timeout(Duration::from_secs(3), pose.changed())
        .await
        .unwrap()
        .unwrap();
    tokio::time::timeout(Duration::from_secs(3), recent.changed())
        .await
        .unwrap()
        .unwrap();
    let after = loaded.snapshot().await.unwrap();
    assert_eq!(after.cloud.display, baseline.cloud.display);
    assert_eq!(after.cloud.revision, baseline.cloud.revision);
    assert_eq!(after.occupancy, baseline.occupancy);
    assert_eq!(pose.borrow().as_ref().unwrap().timestamp_ns, 2_000_000_000);
    let recent =
        nuwax_codec::lcm::recent_cloud::decode_body(recent.borrow().as_ref().unwrap()).unwrap();
    assert_eq!(recent.cloud.unwrap().source_timestamp_ns, 2_000_000_000);
    assert!(recent.registration.is_some());
    loaded.shutdown().await.unwrap();
}

#[tokio::test]
async fn relocalization_reports_real_success_and_failure_and_rejects_pgo() {
    let source = Mapper::start(25, config()).unwrap();
    source.feed(room_frame(25, 1_000_000_000));
    let baseline = source.snapshot().await.unwrap();
    source.shutdown().await.unwrap();
    let loaded = Mapper::load(26, config(), baseline.clone(), true).unwrap();
    let missing = Corrector::relocalize(&loaded, None).await.unwrap();
    assert!(!missing.succeeded);
    assert_eq!(missing.frames_used, 0);
    loaded.feed(room_frame(26, 2_000_000_000));
    loaded.snapshot().await.unwrap();
    let matched = Corrector::relocalize(&loaded, None).await.unwrap();
    assert!(matched.succeeded, "真实同场景匹配失败：{}", matched.message);
    assert_eq!(matched.frames_used, 1);
    assert_eq!(matched.pose.unwrap().frame_id, "map");
    loaded.shutdown().await.unwrap();
    let mut pgo = config();
    pgo.correction_backend = "library_pgo".into();
    let loaded = Mapper::load(27, pgo, baseline, true).unwrap();
    assert_eq!(
        Corrector::relocalize(&loaded, None)
            .await
            .unwrap_err()
            .code(),
        tonic::Code::FailedPrecondition
    );
    loaded.shutdown().await.unwrap();
}

#[tokio::test]
async fn auxiliary_scans_do_not_enter_relocalization_and_keep_their_source_log() {
    let mapper = Mapper::start(28, config()).unwrap();
    mapper.feed(room_frame(28, 1_000_000_000));
    let mut auxiliary = room_frame(28, 1_000_000_000);
    auxiliary.mapping_only = true;
    auxiliary.sensor_id = "rear_lidar".into();
    mapper.feed(auxiliary);
    let snapshot = mapper.snapshot().await.unwrap();
    assert!(
        mapper.error_rx().borrow().is_none(),
        "{:?}",
        *mapper.error_rx().borrow()
    );
    assert_eq!(snapshot.scans.len(), 2);
    assert_eq!(mapper.recent_frames().await.unwrap().len(), 1);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn navigation_block_reads_pin_the_previous_committed_revision() {
    let mapper = Mapper::start(29, config()).unwrap();
    mapper.feed(room_frame(29, 1_000_000_000));
    mapper.snapshot().await.unwrap();
    let first = mapper.surface_map().await.unwrap().unwrap();
    let identity = first.manifest().identity();
    let reference = first.manifest().blocks[0].clone();
    mapper.feed(room_frame(29, 1_100_000_000));
    mapper.snapshot().await.unwrap();
    let result = mapper
        .navigation_map_block(identity, reference, Some((0, 64)))
        .await
        .unwrap();
    assert!(matches!(result,nuwax_map_chunks::ObjectRangeRead::Data{bytes,..}if !bytes.is_empty()));
    mapper.shutdown().await.unwrap();
}
