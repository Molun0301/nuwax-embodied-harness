use super::*;
use crate::corrector::Corrector;
use std::os::unix::ffi::OsStrExt;

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
    let header = recent.cloud.unwrap().header;
    assert_eq!(i64::from(header.stamp_sec) * 1_000_000_000 + i64::from(header.stamp_nsec), 2_000_000_000);
    assert!(recent.registration.is_some());
    loaded.shutdown().await.unwrap();
}

#[tokio::test]
async fn cold_load_republishes_the_same_domain_with_an_advanced_revision() {
    let source = Mapper::start(73, config()).unwrap();
    source.feed(room_frame(73, 1_000_000_000));
    let baseline = source.snapshot().await.unwrap();
    source.shutdown().await.unwrap();
    let loaded = Mapper::load(73, config(), baseline.clone(), true).unwrap();
    let restored = loaded.snapshot().await.unwrap();
    assert_eq!(restored.cloud.display, baseline.cloud.display);
    assert_eq!(restored.occupancy, baseline.occupancy);
    assert!(restored.cloud.revision > baseline.cloud.revision);
    let fine = loaded.surface_map().await.unwrap().unwrap();
    assert_eq!(fine.manifest().revision, restored.cloud.revision);
    loaded.shutdown().await.unwrap();
}

#[tokio::test]
async fn relocalization_reports_real_success_and_failure_and_rejects_pgo() {
    let source = Mapper::start(25, config()).unwrap();
    source.feed(room_frame(25, 1_000_000_000));
    let baseline = source.snapshot().await.unwrap();
    source.shutdown().await.unwrap();
    let loaded = Mapper::load(26, config(), baseline.clone(), true).unwrap();
    let (router, _, _) = crate::session::test_router(loaded.clone());
    let missing = Corrector::relocalize(&loaded, router.recent_frames())
        .await
        .unwrap();
    assert!(!missing.succeeded);
    assert_eq!(missing.frames_used, 0);
    router.publish_frame(room_frame(26, 2_000_000_000));
    loaded.snapshot().await.unwrap();
    let matched = Corrector::relocalize(&loaded, router.recent_frames())
        .await
        .unwrap();
    assert!(matched.succeeded, "真实同场景匹配失败：{}", matched.message);
    assert_eq!(matched.frames_used, 1);
    assert_eq!(matched.pose.unwrap().frame_id, "map");
    loaded.shutdown().await.unwrap();
    let mut pgo = config();
    pgo.correction_backend = "library_pgo".into();
    let loaded = Mapper::load(27, pgo, baseline, true).unwrap();
    assert_eq!(
        Corrector::relocalize(&loaded, router.recent_frames())
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
    let (router, _, _) = crate::session::test_router(mapper.clone());
    router.publish_frame(room_frame(28, 1_000_000_000));
    mapper.snapshot().await.unwrap();
    let mut auxiliary = room_frame(28, 1_000_000_000);
    auxiliary.mapping_only = true;
    auxiliary.sensor_id = "rear_lidar".into();
    router.publish_frame(auxiliary);
    let snapshot = mapper.snapshot().await.unwrap();
    assert!(
        mapper.error_rx().borrow().is_none(),
        "{:?}",
        *mapper.error_rx().borrow()
    );
    assert_eq!(snapshot.scans.len(), 2);
    assert_eq!(router.recent_frames().len(), 1);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn object_block_service_reads_unchanged_old_objects_and_reports_collected_objects() {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    let mapper = Mapper::start(29, config()).unwrap();
    mapper.feed(room_frame(29, 1_000_000_000));
    mapper.snapshot().await.unwrap();
    let first = mapper.fine_root().unwrap();
    let original: Vec<_> = first.manifest().blocks.iter().map(|b| b.object).collect();
    drop(first);
    for revision in 2..=5 {
        let mut frame = room_frame(29, revision * 1_000_000_000);
        frame.mapping_only = true;
        frame.points_body = vec![[12.,0.,0.]];
        mapper.feed(frame);
        mapper.snapshot().await.unwrap();
    }
    let current = mapper.fine_root().unwrap();
    assert!(current.manifest().revision >= 4);
    let unchanged = *original.iter().find(|object| current.manifest().blocks.iter().any(|b| b.object == **object))
        .expect("只改变远处输入时必须仍有未变化块");
    let expected = current.repository.read_object(unchanged, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
    drop(current);
    let reply = crate::rpc_runtime::test_block(mapper.clone(), unchanged).await.unwrap();
    let data = if let Some(encoded) = reply["data_lz4_base64"].as_str() {
        nuwax_codec::lcm::map_block_part::decompress(&STANDARD.decode(encoded).unwrap(), expected.len()).unwrap()
    } else { STANDARD.decode(reply["data_base64"].as_str().unwrap()).unwrap() };
    assert_eq!(data, expected);
    // 之后更新整幅房间，跨过原有16次提交回收周期，不持有旧根。
    for revision in 6..=33 {
        mapper.feed(room_frame(29, revision * 1_000_000_000));
        mapper.snapshot().await.unwrap();
    }
    let current = mapper.fine_root().unwrap();
    let collected = *original.iter().find(|&&object| current.repository.read_object(object, nuwax_map_chunks::MAX_BLOCK_BYTES).is_err())
        .expect("已替换对象应经原有周期回收");
    let error = crate::rpc_runtime::test_block(mapper.clone(), collected).await.unwrap_err();
    assert_eq!(error.code(), tonic::Code::NotFound);
    let cause = current.repository.read_object_range(
        collected, nuwax_map_chunks::MAX_BLOCK_BYTES, 0, nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES,
    ).unwrap_err();
    assert_eq!(error.message(), format!("地图块已被新修订替换或地图已切换：{cause}"));
    mapper.shutdown().await.unwrap();
}

fn test_grid(origin: [f64; 2], data: &[i8], stamp: f64) -> nuwax_mapping::OccupancyGrid {
    let mut grid = nuwax_mapping::OccupancyGrid::with_frame(data.len() as u32, 1, 1., "map");
    grid.origin = [origin[0], origin[1], 0.];
    grid.data = data.to_vec();
    grid.ts = stamp;
    grid
}
fn world_cell(grid: &nuwax_mapping::OccupancyGrid, x: f64, y: f64) -> i8 {
    let col = ((x - grid.origin[0]) / grid.resolution).floor() as u32;
    let row = ((y - grid.origin[1]) / grid.resolution).floor() as u32;
    grid.data[grid.index(row, col)]
}
#[test]
fn live_grid_merges_with_loaded_checkpoint_in_world_coordinates() {
    let old = test_grid([10., -2.], &[100, 0], 1.);
    let mut resumed = PersistentOccupancyGrid::from_grid(old).unwrap();
    resumed
        .merge_observation(&test_grid([11., -2.], &[0, 0, 0], 2.))
        .unwrap();
    assert_eq!(resumed.grid.origin, [10., -2., 0.]);
    assert_eq!(resumed.grid.width, 4);
    assert_eq!(resumed.grid.data, [100, 0, 0, 0]);
}
#[test]
fn live_grid_with_non_aligned_origin_projects_onto_checkpoint_lattice() {
    let old = test_grid([10., -2.], &[100, 0], 1.);
    let mut resumed = PersistentOccupancyGrid::from_grid(old).unwrap();
    resumed
        .merge_observation(&test_grid([11.2, -2.], &[0, 0], 2.))
        .unwrap();
    assert_eq!(resumed.grid.origin, [10., -2., 0.]);
    assert_eq!(resumed.grid.width, 4);
    assert_eq!(world_cell(&resumed.grid, 10.5, -1.5), 100);
    assert_eq!(world_cell(&resumed.grid, 12.7, -1.5), 0);
    assert_eq!(world_cell(&resumed.grid, 13.5, -1.5), -1);
}
#[test]
fn persistent_grid_keeps_old_world_cells_and_expands_for_new_cells() {
    let mut persistent =
        PersistentOccupancyGrid::from_grid(test_grid([0., 0.], &[100, 0], 1.)).unwrap();
    for (stamp, origin) in [(2., -2.), (3., 3.), (4., -4.)] {
        persistent
            .merge_observation(&test_grid([origin, 0.], &[0, 0], stamp))
            .unwrap();
    }
    assert_eq!(world_cell(&persistent.grid, 0.5, 0.5), 100);
    assert_eq!(world_cell(&persistent.grid, -3.5, 0.5), 0);
    assert_eq!(world_cell(&persistent.grid, 4.5, 0.5), 0);
    assert_eq!(persistent.grid.origin[0], -4.);
}
#[tokio::test]
async fn restored_checkpoint_accepts_later_live_frames() {
    let mapper = Mapper::start(61, config()).unwrap();
    mapper.feed(room_frame(61, 1_000_000_000));
    let checkpoint = mapper.snapshot().await.unwrap();
    mapper.shutdown().await.unwrap();
    let resumed = Mapper::load(62, config(), checkpoint.clone(), false).unwrap();
    resumed.feed(room_frame(62, 2_000_000_000));
    let later = resumed.snapshot().await.unwrap();
    assert_eq!(
        later.occupancy.unwrap().meta.unwrap().timestamp_ns,
        2_000_000_000
    );
    assert!(later.cloud.revision > checkpoint.cloud.revision);
    assert!(!later.backend_checkpoint.is_empty());
    resumed.shutdown().await.unwrap();
}
#[tokio::test]
async fn nonzero_map_odom_anchor_transforms_points_and_pose_before_publish() {
    let mapper = Mapper::start(63, config()).unwrap();
    mapper.feed(room_frame(63, 1_000_000_000));
    let checkpoint = mapper.snapshot().await.unwrap();
    mapper.shutdown().await.unwrap();
    let loaded = Mapper::load(64, config(), checkpoint, true).unwrap();
    let anchor = Isometry3::from_parts(
        Translation3::new(3., -4., 0.7),
        UnitQuaternion::from_euler_angles(0., 0., 0.4),
    );
    loaded.set_map_from_odom(anchor).await.unwrap();
    let frame = room_frame(64, 2_000_000_000);
    let expected_point = anchor * Point3::from(frame.points_body[0]);
    loaded.feed(frame.clone());
    let after = loaded.snapshot().await.unwrap();
    let expected_pose = loaded.transform_pose(&frame.odom_pose);
    assert_eq!(after.last_pose.unwrap(), expected_pose);
    let recent = nuwax_codec::lcm::recent_cloud::decode_body(
        loaded.recent_cloud_rx().borrow().as_ref().unwrap(),
    )
    .unwrap();
    let raw = recent.cloud.unwrap().data;
    let xyz = [0, 4, 8]
        .map(|offset| f32::from_le_bytes(raw[offset..offset + 4].try_into().unwrap()) as f64);
    assert!((Point3::from(xyz) - expected_point).norm() < 1e-5);
    loaded.shutdown().await.unwrap();
}
struct TestDirectory(PathBuf);
impl TestDirectory {
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!("b2-mapper-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
#[tokio::test]
async fn forbidden_zone_refresh_changes_effective_map_but_preserves_raw_checkpoint() {
    let directory = TestDirectory::new();
    let path = directory.0.join("forbidden_zones.json");
    let mapper = Mapper::start(
        65,
        MapperConfig {
            backend: MappingBackendSpec::SlamToolbox,
            ..config()
        },
    )
    .unwrap();
    mapper.feed_grid(
        65,
        kernel_grid_to_proto(&test_grid([0., 0.], &[0, 0, 0, 0], 1.), "实测地图"),
    );
    let raw = mapper.snapshot().await.unwrap().occupancy.unwrap();
    mapper.set_forbidden_zones(path.clone()).await.unwrap();
    let mut maps = mapper.map_rx();
    maps.borrow_and_update();
    std::fs::write(&path, r#"[{"id":"区域","shape":"rectangle","mode":"inside","vertices":[{"x":1.2,"y":0.2},{"x":1.8,"y":0.8}]}]"#).unwrap();
    tokio::time::timeout(Duration::from_secs(2), maps.changed())
        .await
        .unwrap()
        .unwrap();
    let update = maps.borrow().clone().unwrap();
    assert_ne!(update.data, update.raw_data);
    assert_eq!(update.raw_data, raw.data);
    assert_eq!(mapper.snapshot().await.unwrap().occupancy.unwrap(), raw);
    assert_eq!(mapper.occupancy(false).unwrap(), raw);
    assert_eq!(mapper.forbidden_status(), Ok(()));
    // 网关的原子替换必须同样触发刷新。
    let replacement = directory.0.join("replacement");
    std::fs::write(&replacement, r#"[]"#).unwrap();
    maps.borrow_and_update();
    std::fs::rename(replacement, &path).unwrap();
    tokio::time::timeout(Duration::from_secs(2), maps.changed())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(maps.borrow().as_ref().unwrap().data, raw.data);
    mapper.shutdown().await.unwrap();
}
#[tokio::test]
async fn malformed_document_fails_closed_and_recovers_on_file_change() {
    let directory = TestDirectory::new();
    let path = directory.0.join("forbidden_zones.json");
    let mapper = Mapper::start(
        66,
        MapperConfig {
            backend: MappingBackendSpec::SlamToolbox,
            ..config()
        },
    )
    .unwrap();
    mapper.feed_grid(
        66,
        kernel_grid_to_proto(&test_grid([0., 0.], &[0, 0], 1.), "实测地图"),
    );
    let raw = mapper.snapshot().await.unwrap().occupancy.unwrap();
    mapper.set_forbidden_zones(path.clone()).await.unwrap();
    let mut maps = mapper.map_rx();
    maps.borrow_and_update();
    std::fs::write(&path, "{bad").unwrap();
    tokio::time::timeout(Duration::from_secs(2), maps.changed())
        .await
        .unwrap()
        .unwrap();
    assert!(mapper
        .forbidden_status()
        .unwrap_err()
        .contains("禁行区格式无效"));
    assert!(maps.borrow().is_none());
    assert_eq!(mapper.snapshot().await.unwrap().occupancy.unwrap(), raw);
    maps.borrow_and_update();
    std::fs::write(&path, r#"[]"#).unwrap();
    tokio::time::timeout(Duration::from_secs(2), maps.changed())
        .await
        .unwrap()
        .unwrap();
    assert!(mapper.forbidden_status().is_ok());
    assert_eq!(maps.borrow().as_ref().unwrap().raw_data, raw.data);
    assert!(mapper.error_rx().borrow().is_none());
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn failed_forbidden_read_is_checked_again_with_unchanged_metadata() {
    let directory = TestDirectory::new();
    let source = directory.0.join("原图禁区.json");
    let target = directory.0.join("当前图禁区.json");
    let valid = br#"[]"#;
    let invalid = b"!\"";
    assert_eq!(valid.len(), invalid.len());
    std::fs::write(&source, valid).unwrap();
    let modified = std::fs::metadata(&source).unwrap().modified().unwrap();
    std::fs::write(&target, invalid).unwrap();
    std::fs::File::open(&target)
        .unwrap()
        .set_modified(modified)
        .unwrap();
    let mapper = Mapper::start(
        74,
        MapperConfig {
            backend: MappingBackendSpec::SlamToolbox,
            ..config()
        },
    )
    .unwrap();
    mapper.feed_grid(
        74,
        kernel_grid_to_proto(&test_grid([0., 0.], &[0, 0], 1.), "实测地图"),
    );
    let raw = mapper.snapshot().await.unwrap().occupancy.unwrap();
    mapper.set_forbidden_zones(source).await.unwrap();
    assert!(mapper
        .set_forbidden_zones(target.clone())
        .await
        .unwrap_err()
        .contains("禁行区格式无效"));
    assert!(mapper.occupancy(true).is_none());
    assert_eq!(mapper.snapshot().await.unwrap().occupancy.unwrap(), raw);
    let mut maps = mapper.map_rx();
    maps.borrow_and_update();
    std::fs::write(&target, valid).unwrap();
    std::fs::File::open(&target)
        .unwrap()
        .set_modified(modified)
        .unwrap();
    let metadata = std::fs::metadata(&target).unwrap();
    assert_eq!(metadata.modified().unwrap(), modified);
    assert_eq!(metadata.len(), valid.len() as u64);
    tokio::time::timeout(Duration::from_secs(2), maps.changed())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(mapper.forbidden_status(), Ok(()));
    assert_eq!(mapper.occupancy(true).unwrap().data, raw.data);
    assert_eq!(mapper.snapshot().await.unwrap().occupancy.unwrap(), raw);
    mapper.shutdown().await.unwrap();
}

#[test]
fn parse_occupancy_payload_rejects_huge_dimension() {
    let grid = proto::OccupancyGrid {
        meta: Some(proto::OccupancyGridMeta {
            width: u32::MAX,
            height: 1,
            resolution: 1.,
            frame_id: "map".into(),
            ..Default::default()
        }),
        ..Default::default()
    };
    assert!(proto_grid_to_kernel(&grid).unwrap_err().contains("8192"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn slow_worker_coalesces_frames_before_save_without_a_frame_backlog() {
    use std::io::Write;
    let directory = TestDirectory::new();
    let fifo = directory.0.join("forbidden_zones.json");
    let path = std::ffi::CString::new(fifo.as_os_str().as_bytes()).unwrap();
    assert_eq!(unsafe { libc::mkfifo(path.as_ptr(), 0o600) }, 0);
    let mapper = Mapper::start(67, config()).unwrap();
    mapper.feed(room_frame(67, 1_000_000_000));
    mapper.snapshot().await.unwrap();
    let object = mapper.fine_root().unwrap().manifest().blocks[0].object;
    let blocked_mapper = mapper.clone();
    let mut refresh = tokio::spawn(async move { blocked_mapper.set_forbidden_zones(fifo).await });
    let fifo_path = directory.0.join("forbidden_zones.json");
    let mut opening_writer = tokio::task::spawn_blocking(move || {
        std::fs::OpenOptions::new()
            .write(true)
            .open(fifo_path)
            .unwrap()
    });
    let writer = tokio::select! {
        opened = &mut opening_writer => opened.unwrap(),
        refreshed = &mut refresh => {
            use std::os::unix::fs::OpenOptionsExt;
            // 刷新前置失败时收回阻塞的写端，随后仍按原始错误使测试失败。
            let reader = std::fs::OpenOptions::new()
                .read(true)
                .custom_flags(libc::O_NONBLOCK)
                .open(directory.0.join("forbidden_zones.json"))
                .unwrap();
            let writer = opening_writer.await.unwrap();
            drop(writer);
            drop(reader);
            refreshed.unwrap().unwrap();
            panic!("禁行区刷新未经过阻塞夹具即结束");
        }
    };
    // 写端打开证明建图线程已被真实文件读取占住，块服务仍须返回。
    let reply = tokio::time::timeout(Duration::from_millis(350),
        crate::rpc_runtime::test_block(mapper.clone(), object)).await.unwrap().unwrap();
    assert!(reply["total_bytes"].as_u64().unwrap() > 0);
    // 保持写端打开，让工作线程继续停在读取，验证原帧合并行为。
    for stamp in 1..=1000 {
        mapper.feed(room_frame(67, 1_000_000_000 + stamp));
    }
    assert!(
        matches!(mapper.latest_input.lock().as_ref(), Some(Input::Frame(frame)) if frame.odom_pose.timestamp_ns == 1_000_001_000)
    );
    let saved_mapper = mapper.clone();
    let saved = tokio::spawn(async move { saved_mapper.snapshot().await });
    let regular_path = directory.0.join("replacement");
    let observed_path = directory.0.join("forbidden_zones.json");
    tokio::task::spawn_blocking(move || {
        let mut writer = writer;
        writer.write_all(br#"[]"#).unwrap();
        std::fs::write(&regular_path, br#"[]"#).unwrap();
        std::fs::rename(regular_path, observed_path).unwrap();
    })
    .await
    .unwrap();
    refresh.await.unwrap().unwrap();
    let snapshot = saved.await.unwrap().unwrap();
    assert_eq!(snapshot.last_pose.unwrap().timestamp_ns, 1_000_001_000);
    assert_eq!(snapshot.scans.len(), 1);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn backend_failure_preserves_the_completed_checkpoint_for_save() {
    let mapper = Mapper::start(
        68,
        MapperConfig {
            backend: MappingBackendSpec::SlamToolbox,
            ..config()
        },
    )
    .unwrap();
    mapper.feed_grid(
        68,
        kernel_grid_to_proto(&test_grid([0., 0.], &[0, 100], 1.), "实测地图"),
    );
    let good = mapper.snapshot().await.unwrap();
    let mut errors = mapper.error_rx();
    mapper.feed_grid(
        68,
        proto::OccupancyGrid {
            meta: Some(proto::OccupancyGridMeta {
                width: 2,
                height: 1,
                resolution: 1.,
                frame_id: "map".into(),
                ..Default::default()
            }),
            data: vec![],
            ..Default::default()
        },
    );
    tokio::time::timeout(Duration::from_secs(2), errors.changed())
        .await
        .unwrap()
        .unwrap();
    assert!(errors.borrow().is_some());
    assert_eq!(mapper.snapshot().await.unwrap().occupancy, good.occupancy);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn prepared_map_does_not_replace_live_shared_cloud_until_activation() {
    use std::os::unix::fs::MetadataExt;
    let directory = TestDirectory::new();
    let path = directory.0.join("cloud.shm");
    let mut shared = config();
    shared.cloudmap_shm_path = Some(path.clone());
    let live = Mapper::start(69, shared.clone()).unwrap();
    live.feed(room_frame(69, 1_000_000_000));
    let snapshot = live.snapshot().await.unwrap();
    let before = std::fs::read(&path).unwrap();
    let inode = std::fs::metadata(&path).unwrap().ino();
    let prepared = Mapper::prepare_load(70, shared.clone(), snapshot.clone(), true).unwrap();
    assert_eq!(std::fs::metadata(&path).unwrap().ino(), inode);
    assert_eq!(std::fs::read(&path).unwrap(), before);
    assert_eq!(
        prepared.snapshot().await.unwrap().occupancy,
        snapshot.occupancy
    );
    prepared.shutdown().await.unwrap();
    assert_eq!(std::fs::read(&path).unwrap(), before);
    let mut invalid = shared.clone();
    invalid.compute.window_capacity /= 2;
    assert!(Mapper::prepare_load(71, invalid, snapshot.clone(), true).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    let ready = Mapper::prepare_load(72, shared, snapshot.clone(), true).unwrap();
    ready.activate_shared_cloud().await.unwrap();
    let installed = std::fs::read(&path).unwrap();
    assert_ne!(std::fs::metadata(&path).unwrap().ino(), inode);
    assert_eq!(
        u64::from_le_bytes(installed[88..96].try_into().unwrap()),
        72
    );
    assert_eq!(
        u64::from_le_bytes(installed[24..32].try_into().unwrap()),
        snapshot.cloud.revision
    );
    let count = u32::from_le_bytes(installed[64..68].try_into().unwrap()) as usize;
    let payload = u64::from_le_bytes(installed[104..112].try_into().unwrap()) as usize;
    assert!(count > 0);
    assert!(installed[payload..payload + count * 12]
        .iter()
        .any(|byte| *byte != 0));
    live.shutdown().await.unwrap();
    assert!(path.exists());
    assert_eq!(
        u64::from_le_bytes(std::fs::read(&path).unwrap()[88..96].try_into().unwrap()),
        72
    );
    ready.shutdown().await.unwrap();
}

#[tokio::test]
async fn forbidden_zone_poll_detects_two_writes_with_the_same_mtime() {
    let directory = TestDirectory::new();
    let path = directory.0.join("forbidden_zones.json");
    let mapper = Mapper::start(
        75,
        MapperConfig {
            backend: MappingBackendSpec::SlamToolbox,
            ..config()
        },
    )
    .unwrap();
    mapper.feed_grid(
        75,
        kernel_grid_to_proto(&test_grid([0., 0.], &[0, 0], 1.), "实测地图"),
    );
    let raw = mapper.snapshot().await.unwrap().occupancy.unwrap();
    mapper.set_forbidden_zones(path.clone()).await.unwrap();
    let mut maps = mapper.map_rx();
    maps.borrow_and_update();

    let modified = std::time::UNIX_EPOCH + Duration::from_secs(1_700_000_000);
    let zone = r#"[{"id":"左格","shape":"rectangle","mode":"inside","vertices":[{"x":0.2,"y":0.2},{"x":0.4,"y":0.4}]}]"#;
    let clear = r#"[]"#;
    assert_ne!(zone.len(), clear.len());
    for (document, expected) in [(zone, vec![100, 0]), (clear, raw.data.clone())] {
        // 固定相同修改时间，强制第二次检查只能用长度变化触发重读。
        let replacement = directory.0.join("写入中");
        std::fs::write(&replacement, document).unwrap();
        std::fs::File::open(&replacement)
            .unwrap()
            .set_modified(modified)
            .unwrap();
        std::fs::rename(replacement, &path).unwrap();
        tokio::time::timeout(Duration::from_secs(3), maps.changed())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(std::fs::metadata(&path).unwrap().modified().unwrap(), modified);
        assert_eq!(maps.borrow_and_update().as_ref().unwrap().data, expected);
    }
    mapper.shutdown().await.unwrap();
}
