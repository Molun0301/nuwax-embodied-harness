// 由 pgo_map::tests::disk_tests 包含；行为回归与显式实录基准共用真实磁盘重放入口。
use super::*;
use crate::admitted_cloud::AcceptedMappingCloud;
use crate::dynamic_backend::{MappedFrame, OnlineDufo};
use crate::lio_ingress::AcceptedRegisteredCloud;
use nuwax_cloud_cleanup::scan_log::{
    Recorder, ReplaySource, SavePolicy, ScanMetadata, ScanReader, Session, StorageWorker,
};
use serde_json::json;
use std::{
    collections::HashSet,
    fs::File,
    io::{BufReader, Read},
    path::{Path, PathBuf},
    sync::{Mutex, MutexGuard},
};

static DISK_TEST_OWNER: Mutex<()> = Mutex::new(());

struct DiskTestEnvironment(Vec<(&'static str, Option<std::ffi::OsString>)>);
impl DiskTestEnvironment {
    fn defaults(root: &Path) -> Self {
        let names = [
            "NUWAX_RAW_SCAN_DIRECTORY",
            "NUWAX_RAW_SCAN_TRANSLATION_M",
            "NUWAX_RAW_SCAN_ROTATION_DEG",
            "NUWAX_RAW_SCAN_INTERVAL_S",
            "NUWAX_RAW_SCAN_QUEUE_FRAMES",
            "NUWAX_RAW_SCAN_MAX_MIB",
        ];
        let previous = names
            .into_iter()
            .map(|name| (name, std::env::var_os(name)))
            .collect();
        for name in names {
            std::env::remove_var(name);
        }
        std::env::set_var("NUWAX_RAW_SCAN_DIRECTORY", root);
        Self(previous)
    }
}
impl Drop for DiskTestEnvironment {
    fn drop(&mut self) {
        for (name, value) in &self.0 {
            match value {
                Some(value) => std::env::set_var(name, value),
                None => std::env::remove_var(name),
            }
        }
    }
}

struct DiskTestOwner {
    root: PathBuf,
    worker: Option<StorageWorker>,
    keep_evidence: bool,
    _environment: DiskTestEnvironment,
    _serial: MutexGuard<'static, ()>,
}
impl DiskTestOwner {
    fn new(evidence: Option<PathBuf>) -> Self {
        let serial = DISK_TEST_OWNER.lock().unwrap();
        let keep_evidence = evidence.is_some();
        let root = evidence.unwrap_or_else(|| {
            std::env::temp_dir().join(format!("nuwax-g2-disk-{}", uuid::Uuid::new_v4()))
        });
        std::fs::create_dir_all(&root).unwrap();
        let environment = DiskTestEnvironment::defaults(&root);
        let policy = SavePolicy::from_env().unwrap();
        assert_eq!(
            (policy.translation_m, policy.rotation_deg, policy.interval_s),
            (0.1, 3., 1.)
        );
        Self {
            root,
            worker: Some(StorageWorker::start().unwrap()),
            keep_evidence,
            _environment: environment,
            _serial: serial,
        }
    }
    fn recorder(&self, generation: u64) -> Recorder {
        Recorder::new(
            self.root.clone(),
            Session {
                session_id: "bounded-history".into(),
                generation,
            },
        )
        .unwrap()
    }
    fn finish_storage(&mut self) {
        drop(self.worker.take());
    }
}
impl Drop for DiskTestOwner {
    fn drop(&mut self) {
        self.finish_storage();
        if !self.keep_evidence {
            std::fs::remove_dir_all(&self.root).unwrap();
        }
    }
}

fn disk_config() -> MappingBackendComputeConfig {
    MappingBackendComputeConfig {
        resolution: 0.05,
        initial_safe_radius_meters: 0.,
        window_capacity: nuwax_mapping::mapping_session::DEFAULT_BOUNDED_CAPACITY,
    }
}

fn disk_source(
    index: usize,
    stamp: u64,
    pose: Isometry3<f64>,
    points: Vec<Point3<f64>>,
    origin_body: [f64; 3],
    generation: u64,
) -> AcceptedRegisteredCloud {
    let mut binding = test_binding();
    binding.cloud_source_sequence = index as i64 + 1;
    binding.correlated_odometry.source_sequence = index as i64 + 1;
    binding.correlated_odometry.source_timestamp_ns = stamp as i64;
    binding.mapping_session_generation = generation;
    binding.frozen_odom_from_body = pose;
    binding.sensor_origin_map = Some((pose * Point3::from(origin_body)).coords.into());
    AcceptedRegisteredCloud::from_map_points_for_test(points, stamp, stamp, "map", "body", binding)
}

fn disk_metadata(source: &AdmittedRegisteredScan) -> ScanMetadata {
    let b = source.binding();
    let origin = b.sensor_origin_map.map(|p| {
        (b.registration_map_from_odom * b.frozen_odom_from_body).inverse() * Point3::from(p)
    });
    ScanMetadata {
        source_timestamp_ns: source.source_timestamp_ns(),
        source_sequence: source.source_identity().source_sequence as u64,
        sensor: source.sensor_frame_id().into(),
        source_generation: b.runtime_generation as u64,
        odom_from_body: b
            .frozen_odom_from_body
            .to_homogeneous()
            .as_slice()
            .try_into()
            .unwrap(),
        map_from_odom: b
            .registration_map_from_odom
            .to_homogeneous()
            .as_slice()
            .try_into()
            .unwrap(),
        sensor_origin_body: origin.map(|p| [p.x as f32, p.y as f32, p.z as f32]),
        observation_minimum_range_m: Some(0.2),
        point_count: source.body_points().len(),
    }
}

fn disk_wait_written(source: &ReplaySource, stamp: u64) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if source
            .snapshot()
            .unwrap()
            .is_some_and(|p| p.observed_until_ns() >= stamp)
        {
            break;
        }
        assert!(Instant::now() < deadline, "后台日志没有发布完整扫描进度");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn disk_node(stamp: u64, pose: Isometry3<f64>) -> OptimizedKeyframe {
    let q = pose.rotation.coords;
    OptimizedKeyframe {
        source_timestamp_ns: stamp as i64,
        translation_m: pose.translation.vector.into(),
        rotation_xyzw: [q.x, q.y, q.z, q.w],
    }
}

fn disk_run(
    input: DiskReplayInput,
    trajectory: Vec<OptimizedKeyframe>,
    terminal: OnlineDufo,
) -> RebuiltMap {
    let latest = input.latest.clone();
    let (builder, mut cursor) = input
        .start(
            trajectory,
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            terminal,
            &|| false,
        )
        .unwrap();
    let mut builder = builder.into_background();
    loop {
        let (next_cursor, next_builder, ready) =
            cursor.append_slice(builder, &latest, &|| false).unwrap();
        cursor = next_cursor;
        builder = next_builder;
        assert!(
            builder.cloud_worker.is_none(),
            "每片返回时点云工作线程已经回收"
        );
        if ready {
            break;
        }
        std::thread::sleep(REPLAY_YIELD);
    }
    let prepared = builder.prepare(&|| false).unwrap();
    builder.finish(prepared, &|| false).unwrap()
}

fn disk_receipt(stamp: u64) -> crate::mapping_commit::OccupancyCommitReceipt {
    crate::mapping_commit::OccupancyCommitReceipt {
        commit_id: crate::mapping_commit::MappingCommitId {
            boot_nonce: 902,
            sequence: 1,
        },
        session_id: "bounded-history".into(),
        session_generation: 1,
        occupancy_revision: 1,
        source_timestamp_ns: stamp,
        backend_kind: mapping_backend::MappingBackendKind::Voxel,
        backend_generation: 1,
    }
}

fn disk_archive_spool(owner: &mut DiskTestOwner, rebuilt: &mut RebuiltMap) -> PathBuf {
    assert!(
        rebuilt.final_poses.is_empty(),
        "流式重建不能在内存累积最终位姿"
    );
    let spool = rebuilt
        .final_pose_spool
        .take()
        .expect("流式重建必须交付最终位姿暂存");
    if owner.worker.is_none() {
        owner.worker = Some(StorageWorker::start().unwrap());
    }
    nuwax_cloud_cleanup::scan_log::enqueue_spooled_final_poses(
        owner.root.clone(),
        nuwax_cloud_cleanup::scan_log::FinalPoses {
            session: Session {
                session_id: "bounded-history".into(),
                generation: 1,
            },
            cloud_revision: 1,
            frame_generation: Some(1),
            poses: Vec::new(),
        },
        spool,
    )
    .unwrap();
    owner.finish_storage();
    assert!(
        !std::fs::read_dir(&owner.root)
            .unwrap()
            .filter_map(Result::ok)
            .any(|entry| entry
                .file_name()
                .to_string_lossy()
                .starts_with(".pose-spool-")),
        "最终位姿交接后必须删除私有暂存文件"
    );
    std::fs::read_dir(&owner.root)
        .unwrap()
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .find(|path| {
            path.file_name()
                .unwrap()
                .to_string_lossy()
                .ends_with("-poses-1.json")
        })
        .expect("真实后台最终位姿存档必须落盘")
}

fn disk_saved_poses(owner: &mut DiskTestOwner, rebuilt: &mut RebuiltMap) -> Vec<(u64, [f64; 16])> {
    let path = disk_archive_spool(owner, rebuilt);
    let stored: nuwax_cloud_cleanup::scan_log::FinalPoses =
        serde_json::from_reader(File::open(path).unwrap()).unwrap();
    stored.poses
}

#[test]
fn streamed_history_crosses_eight_million_real_points_and_rebuilds_corrected_tail() {

    let mut owner = DiskTestOwner::new(None);
    let mut recorder = owner.recorder(1);
    let replay = recorder.replay_source();
    let mut history = ScanHistory::default();
    history.bind_replay_source(Some(replay.clone()));
    let frame = Arc::new(MappedFrame::unfiltered());
    let mut total = 0usize;
    let mut nodes = Vec::new();
    let mut last_source = None;
    for index in 0..129 {
        // 真实交接八百多万原点；重复空间位置让本用例只测历史规模而不扩大地图。
        let stamp = 1_000_000_000 + index as u64 * 50_000_000;
        let source = disk_source(
            index,
            stamp,
            Isometry3::identity(),
            vec![Point3::new(2.025, 0.025, 0.625); 62_501],
            [0.025, 0.025, 0.025],
            1,
        );
        source
            .scan()
            .record_static_indices((0..62_501).collect(), &frame)
            .unwrap();
        recorder
            .observe(
                disk_metadata(source.scan()),
                source
                    .scan()
                    .body_points()
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            )
            .unwrap();
        history.record(source.scan(), stamp).unwrap();
        if [0, 64, 128].contains(&index) {
            source.scan().retain_pgo_keyframe().unwrap();
            let t = index as f64 / 128.;
            nodes.push(disk_node(
                stamp,
                Isometry3::from_parts(
                    Translation3::new(0.2 * t, 0.1 * t, 0.02 * t),
                    UnitQuaternion::from_euler_angles(0., 0., 0.12 * t),
                ),
            ));
        }
        total += source.scan().body_points().len();
        assert_eq!(
            history
                .stream
                .latest
                .as_ref()
                .unwrap()
                .body_points
                .len(),
            62_501
        );
        last_source = Some(source);
        disk_wait_written(&replay, stamp);
    }
    assert!(total > 8_000_000, "必须实际接纳超过旧上限的源点");
    owner.finish_storage();
    let input = history.disk_input(&nodes).unwrap();
    let mut reader = ScanReader::open_prefix(&replay.snapshot().unwrap().unwrap()).unwrap();
    let mut logged = Vec::new();
    while let Some(scan) = reader.next_scan().unwrap() {
        logged.push(scan.metadata.source_timestamp_ns);
    }
    let tail = last_source.as_ref().unwrap().source_timestamp_ns();
    assert!(!logged.contains(&tail), "本夹具末帧应未达到默认抽帧条件");
    let expected = optimized_pose(nodes.last().unwrap())
        * Point3::new(2.025f32 as f64, 0.025f32 as f64, 0.625f32 as f64);
    let mut rebuilt = disk_run(input, nodes, OnlineDufo::default());
    assert_eq!(rebuilt.source_timestamp_ns, tail);
    assert_eq!(
        rebuilt.final_pose_spool.as_ref().unwrap().len(),
        (logged.len() + 1) as u64,
        "日志帧加真实未选末帧恰好各重放一次"
    );
    let saved = disk_saved_poses(&mut owner, &mut rebuilt);
    let last_pose = nalgebra::Matrix4::from_column_slice(&saved.last().unwrap().1);
    assert!((last_pose[(0, 3)] - 0.2).abs() < 1e-12);
    let size = f64::from(rebuilt.cloud.voxel_size);
    let key = |p: Point3<f64>| [p.x, p.y, p.z].map(|v| (v / size).floor() as i32);
    let keys: HashSet<_> = rebuilt
        .cloud
        .xyz_f32_le
        .chunks_exact(12)
        .map(|p| {
            key(Point3::new(
                f32::from_le_bytes(p[0..4].try_into().unwrap()) as f64,
                f32::from_le_bytes(p[4..8].try_into().unwrap()) as f64,
                f32::from_le_bytes(p[8..12].try_into().unwrap()) as f64,
            ))
        })
        .collect();
    assert!(
        keys.contains(&key(expected)),
        "超过旧上限后校正必须真正更新点云位置"
    );
}

#[test]
fn streamed_keyframes_use_exact_unsampled_pose_and_reset_isolates_old_store() {

    let mut owner = DiskTestOwner::new(None);
    let mut recorder = owner.recorder(1);
    let replay = recorder.replay_source();
    let mut history = ScanHistory::default();
    history.bind_replay_source(Some(replay.clone()));
    let frame = Arc::new(MappedFrame::unfiltered());
    let mut held = Vec::new();
    let mut nodes = Vec::new();
    for index in 0..6 {
        let stamp = 1_000_000_000 + index as u64 * 50_000_000;
        let local = Isometry3::translation(index as f64 * 0.017, 0., 0.);
        let source = disk_source(
            index,
            stamp,
            local,
            vec![Point3::new(1.025, 0.025, 0.625)],
            [0.; 3],
            1,
        );
        source
            .scan()
            .record_static_indices(vec![0], &frame)
            .unwrap();
        recorder
            .observe(
                disk_metadata(source.scan()),
                source
                    .scan()
                    .body_points()
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            )
            .unwrap();
        history.record(source.scan(), stamp).unwrap();
        if [0, 2, 5].contains(&index) {
            source.scan().retain_pgo_keyframe().unwrap();
            let corrected = if index == 0 {
                local
            } else {
                Isometry3::translation(0., index as f64 * 0.07, 0.) * local
            };
            nodes.push(disk_node(stamp, corrected));
        }
        held.push(source);
    }
    owner.finish_storage();
    let prefix = replay.snapshot().unwrap().unwrap();
    let mut reader = ScanReader::open_prefix(&prefix).unwrap();
    assert_eq!(
        reader
            .next_scan()
            .unwrap()
            .unwrap()
            .metadata
            .source_timestamp_ns,
        nodes[0].source_timestamp_ns as u64
    );
    assert!(
        reader.next_scan().unwrap().is_none(),
        "默认v2只选择第一帧，关键帧不能取邻居近似"
    );
    let input = history.disk_input(&nodes).unwrap();
    assert_eq!(input.locals[1], held[2].binding().frozen_odom_from_body);
    assert_eq!(input.locals[2], held[5].binding().frozen_odom_from_body);
    let old_store = history.stream.keyframes.clone();
    let mut rebuilt = disk_run(input, nodes.clone(), OnlineDufo::default());
    assert_eq!(rebuilt.final_pose_spool.as_ref().unwrap().len(), 2);
    let saved = disk_saved_poses(&mut owner, &mut rebuilt);
    assert_eq!(saved.last().unwrap().0, held[5].source_timestamp_ns());
    let tail = nalgebra::Matrix4::from_column_slice(&saved.last().unwrap().1);
    assert!((tail[(1, 3)] - 0.35).abs() < 1e-12);
    history.clear();
    assert!(history.stream.source.is_none());
    assert!(history.stream.latest.is_none());
    assert!(!history
        .stream
        .keyframes
        .same_store(&old_store));
    held[2].scan().retain_pgo_keyframe().unwrap();
    assert_eq!(old_store.len(), 3);
    assert_eq!(
        history.stream.keyframes.len(),
        0,
        "旧扫描不能污染新会话位姿表"
    );
    assert!(matches!(
        history.disk_input(&nodes),
        Err(RebuildError::HistoryUnavailable)
    ));
    history.bind_replay_source(Some(replay));
    let fresh = disk_source(
        8,
        2_000_000_000,
        Isometry3::identity(),
        vec![Point3::new(1., 0., 0.6)],
        [0.; 3],
        2,
    );
    fresh.scan().record_static_indices(vec![0], &frame).unwrap();
    history.record(fresh.scan(), 2_000_000_000).unwrap();
    fresh.scan().retain_pgo_keyframe().unwrap();
    assert!(
        matches!(
            history.disk_input(&[disk_node(2_000_000_000, Isometry3::identity())]),
            Err(RebuildError::HistoryUnavailable)
        ),
        "其它会话日志不可重放"
    );
}

#[test]
fn streamed_unavailable_or_cancelled_replay_keeps_current_map_and_releases_sources() {

    let mut owner = DiskTestOwner::new(None);
    let mut recorder = owner.recorder(1);
    let replay = recorder.replay_source();
    let mut current = CloudMapShmWriter::new_private_for_test().unwrap();
    current.accumulate_mapping_frame(1, &[Point3::new(7.025, 0.025, 0.625)]);
    current.commit_accumulated().unwrap();
    let before = current.committed_snapshot().unwrap().xyz_f32_le;
    let frame = Arc::new(MappedFrame::unfiltered());
    let source = disk_source(
        0,
        1_000_000_000,
        Isometry3::identity(),
        vec![Point3::new(1.025, 0.025, 0.625)],
        [0.; 3],
        1,
    );
    let retained = source.scan().retention_probe_for_test();
    source
        .scan()
        .record_static_indices(vec![0], &frame)
        .unwrap();
    let mut history = ScanHistory::default();
    history.record(source.scan(), 1_000_000_000).unwrap();
    source.scan().retain_pgo_keyframe().unwrap();
    let nodes = vec![disk_node(1_000_000_000, Isometry3::identity())];
    assert!(
        matches!(
            history.disk_input(&nodes),
            Err(RebuildError::HistoryUnavailable)
        ),
        "日志禁用应明确拒绝整图重建"
    );
    recorder
        .observe(
            disk_metadata(source.scan()),
            [[1.025, 0.025, 0.625]].into_iter(),
        )
        .unwrap();
    disk_wait_written(&replay, 1_000_000_000);
    history.bind_replay_source(Some(replay.clone()));
    let input = history.disk_input(&nodes).unwrap();
    let latest = input.latest.clone();
    let (builder, cursor) = input
        .start(
            nodes.clone(),
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            OnlineDufo::default(),
            &|| false,
        )
        .unwrap();
    let mut builder = builder.into_background();
    builder.force_parallel_lanes = true;
    let polls = std::cell::Cell::new(0);
    let cancelled = || {
        polls.set(polls.get() + 1);
        polls.get() >= 5
    };
    assert!(matches!(
        cursor.append_slice(builder, &latest, &cancelled),
        Err(RebuildError::Cancelled)
    ));
    assert!(polls.get() >= 5, "取消必须发生在已经推进的真实后台切片中");
    assert_eq!(current.committed_snapshot().unwrap().xyz_f32_le, before);
    std::fs::write(replay.path().with_extension("incomplete"), "合成不完整日志").unwrap();
    let input = history.disk_input(&nodes).unwrap();
    let (builder, cursor) = input
        .start(
            nodes.clone(),
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            OnlineDufo::default(),
            &|| false,
        )
        .unwrap();
    assert!(matches!(
        cursor.append_slice(builder.into_background(), &latest, &|| false),
        Err(RebuildError::HistoryUnavailable)
    ));
    std::fs::remove_file(replay.path().with_extension("incomplete")).unwrap();
    let mut invalid = disk_metadata(source.scan());
    invalid.source_timestamp_ns += 1;
    invalid.source_sequence += 1;
    invalid.odom_from_body[12] = 1.;
    // 由真实后台写盘拒绝非法载荷，不能只给测试替身返回失败。
    recorder
        .observe(invalid, [[f32::NAN, 0., 0.]].into_iter())
        .unwrap();
    owner.finish_storage();
    assert!(replay.snapshot().is_err());
    let input = history.disk_input(&nodes).unwrap();
    let (builder, cursor) = input
        .start(
            nodes,
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            OnlineDufo::default(),
            &|| false,
        )
        .unwrap();
    assert!(matches!(
        cursor.append_slice(builder.into_background(), &latest, &|| false),
        Err(RebuildError::HistoryUnavailable)
    ));
    assert_eq!(
        current.committed_snapshot().unwrap().xyz_f32_le,
        before,
        "失败候选不能替换现有地图"
    );
    drop(latest);
    drop(source);
    history.clear();
    assert!(!retained(), "取消、失败和清空后原始扫描拥有方必须释放");
    assert!(
        !std::fs::read_dir(&owner.root)
            .unwrap()
            .filter_map(Result::ok)
            .any(|entry| entry
                .file_name()
                .to_string_lossy()
                .starts_with(".pose-spool-")),
        "取消或失败后不能残留私有最终位姿暂存"
    );
    assert!(
        !std::fs::read_dir("/proc/self/task")
            .unwrap()
            .filter_map(Result::ok)
            .any(|entry| std::fs::read_to_string(entry.path().join("comm"))
                .is_ok_and(|name| name.trim() == "pgo-cloud")),
        "取消后不能残留私有点云线程"
    );
}

fn disk_dense_points(snapshot: &crate::dense_cloud::DenseSnapshot) -> Vec<[f32; 3]> {
    let mut out = Vec::new();
    while out.len() < snapshot.point_count as usize {
        let bytes = snapshot
            .read(out.len() as u64, crate::dense_cloud::CHUNK_POINTS)
            .unwrap();
        assert!(!bytes.is_empty());
        out.extend(bytes.chunks_exact(12).map(|p| {
            std::array::from_fn(|axis| {
                f32::from_le_bytes(p[axis * 4..axis * 4 + 4].try_into().unwrap())
            })
        }));
    }
    out
}

#[test]
fn streamed_terminal_b_filters_actual_corrected_fine_dense_and_visible_same_population() {

    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let mut owner = DiskTestOwner::new(None);
    let mut cloud = CloudMapShmWriter::new_private_for_test().unwrap();
    cloud.begin_session(1).unwrap();
    let mut history = ScanHistory::default();
    let mut oracle =
        nuwax_dufomap_native::DufoMap::new(&crate::dynamic_backend::online_params()).unwrap();
    let mut frames = Vec::new();
    let mut nodes = Vec::new();
    for index in 0..22 {
        let frame = index + 1;
        let person = (7..=16)
            .contains(&frame)
            .then(|| -0.5 + (frame - 7) as f64 * 0.12);
        let (points, truth) = corridor(person, (frame % 3) as f64);
        let stamp = frame as u64 * 100_000_000;
        let xyz: Vec<_> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        oracle.insert(&xyz, ORIGIN.map(|v| v as f32), true).unwrap();
        let source = disk_source(
            index,
            stamp,
            Isometry3::identity(),
            points.clone(),
            ORIGIN,
            1,
        );
        cloud.classify_admitted_scan(&source).unwrap();
        history.record(source.scan(), stamp).unwrap();
        if [1, 11, 22].contains(&frame) {
            source.scan().retain_pgo_keyframe().unwrap();
            let t = index as f64 / 21.;
            nodes.push(disk_node(
                stamp,
                Isometry3::from_parts(
                    Translation3::new(0.3 * t, -0.2 * t, 0.05 * t),
                    UnitQuaternion::from_euler_angles(0.03 * t, -0.04 * t, 0.2 * t),
                ),
            ));
        }
        assert!(cloud.accumulate_mapping_scan_with_source(
            1,
            &points,
            Some(ORIGIN),
            stamp,
            Some(&AcceptedMappingCloud::from(source))
        ));
        frames.push((stamp, points, truth));
    }
    history.bind_replay_source(cloud.pgo_replay_source());
    owner.finish_storage();
    let input = history.disk_input(&nodes).unwrap();
    let mut selected = Vec::new();
    let mut reader = ScanReader::open_prefix(&input.source.snapshot().unwrap().unwrap()).unwrap();
    while let Some(scan) = reader.next_scan().unwrap() {
        let (_, _, truth) = frames
            .iter()
            .find(|(stamp, _, _)| *stamp == scan.metadata.source_timestamp_ns)
            .unwrap();
        selected.push((
            scan.metadata.source_timestamp_ns,
            scan.points
                .into_iter()
                .map(|p| Point3::from(p.map(f64::from)))
                .collect::<Vec<_>>(),
            truth.clone(),
        ));
    }
    let latest = &input.latest;
    if selected.last().unwrap().0 != latest.source_timestamp_ns {
        selected.push((
            latest.source_timestamp_ns,
            latest.body_points.iter().copied().collect(),
            frames.last().unwrap().2.clone(),
        ));
    }
    let first = nodes[0].source_timestamp_ns;
    let timeline = CorrectionTimeline::new(
        nodes
            .iter()
            .map(|node| CorrectionKeyframe {
                timestamp: (node.source_timestamp_ns - first) as f64 / 1e9,
                local_pose: Isometry3::identity(),
                optimized_pose: optimized_pose(node),
            })
            .collect(),
    )
    .unwrap();
    let mut expected_dense = Vec::new();
    let mut expected_keys = HashSet::new();
    let mut deleted_people = 0;
    let mut deleted_static = 0;
    let mut population = 0;
    for (stamp, points, truth) in &selected {
        let xyz: Vec<_> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        let flags = oracle.seen_free(&xyz).unwrap();
        let pose = timeline
            .correct_pose((*stamp as i64 - first) as f64 / 1e9, &Isometry3::identity())
            .unwrap();
        for ((point, &dynamic), &person) in points.iter().zip(&flags).zip(truth) {
            population += 1;
            if dynamic {
                deleted_people += usize::from(person);
                deleted_static += usize::from(!person);
            } else {
                let mapped = pose * point;
                expected_dense.push([mapped.x as f32, mapped.y as f32, mapped.z as f32]);
                expected_keys.insert(
                    crate::pointcloud::voxel_key(
                        [mapped.x, mapped.y, mapped.z],
                        crate::pointcloud::VOXEL_SIZE,
                    )
                    .unwrap(),
                );
            }
        }
    }
    assert!(
        deleted_people > 0,
        "同一实际重建人口必须含终态B真正删除的人体"
    );
    assert_eq!(deleted_static, 0, "同一实际重建人口中的静态真值必须保留");
    let terminal = cloud.pgo_terminal_dufo();
    let rebuilt = disk_run(input, nodes, terminal);
    let visible: HashSet<_> = rebuilt
        .cloud
        .xyz_f32_le
        .chunks_exact(12)
        .map(|b| {
            let p: [f64; 3] = std::array::from_fn(|a| {
                f32::from_le_bytes(b[a * 4..a * 4 + 4].try_into().unwrap()) as f64
            });
            crate::pointcloud::voxel_key(p, rebuilt.cloud.voxel_size).unwrap()
        })
        .collect();
    assert_eq!(visible, expected_keys, "实际规范可见输出逐键采用同一B终态");
    let stamp = rebuilt.source_timestamp_ns;
    cloud
        .install_prepared_accumulator(&disk_receipt(stamp), rebuilt.accumulator)
        .unwrap();
    let actual_dense = disk_dense_points(&cloud.committed_dense_cloud().unwrap().unwrap());
    assert_eq!(
        actual_dense, expected_dense,
        "完整输出须按同一原点B标志过滤，保留原序与重复"
    );
    assert_eq!(actual_dense.len() + deleted_people, population);
    let fine = cloud.committed_fine_cloud().unwrap();
    let mut region = nuwax_map_chunks::RegionReader::new(fine.repository.clone(), 4 * 1024 * 1024);
    let mut fine_keys = HashSet::new();
    for reference in &fine.manifest().blocks {
        fine_keys.extend(region.block(reference).unwrap().keys().map(|&local| {
            let k = nuwax_map_chunks::global_key(reference.key, local).unwrap();
            (k[0], k[1], k[2])
        }));
    }
    assert_eq!(fine_keys, expected_keys, "实际精细输出逐键采用同一B终态");
}

// 实录只保留当前解码帧，避免将全部原始机体点预加载计入进程峰值。
struct StreamedRecordedInput(BufReader<File>);
impl StreamedRecordedInput {
    fn open(path: &Path) -> Self {
        Self(BufReader::new(File::open(path).unwrap()))
    }
    fn next(&mut self) -> Option<(u64, Isometry3<f64>, Vec<Point3<f64>>)> {
        let mut h = [0u8; 68];
        if self.0.read(&mut h[..1]).unwrap() == 0 {
            return None;
        }
        self.0.read_exact(&mut h[1..]).expect("实录扫描头截断");
        let stamp = u64::from_le_bytes(h[..8].try_into().unwrap());
        let p: [f64; 7] = std::array::from_fn(|i| {
            f64::from_le_bytes(h[8 + i * 8..16 + i * 8].try_into().unwrap())
        });
        assert!(p.iter().all(|v| v.is_finite()));
        let count = u32::from_le_bytes(h[64..68].try_into().unwrap()) as usize;
        assert!((1..=2_000_000).contains(&count));
        let mut b = vec![0u8; count * 12];
        self.0.read_exact(&mut b).expect("实录扫描点截断");
        let pose = Isometry3::from_parts(
            Translation3::new(p[0], p[1], p[2]),
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(p[6], p[3], p[4], p[5])),
        );
        let body = b
            .chunks_exact(12)
            .map(|p| {
                Point3::from(std::array::from_fn::<_, 3, _>(|a| {
                    f32::from_le_bytes(p[a * 4..a * 4 + 4].try_into().unwrap()) as f64
                }))
            })
            .collect();
        Some((stamp, pose, body))
    }
}

fn disk_usage() -> (u64, u64) {
    let mut usage: libc::rusage = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut usage) }, 0);
    let status = std::fs::read_to_string("/proc/self/status").unwrap();
    let current = status
        .lines()
        .find_map(|line| {
            line.strip_prefix("VmRSS:")
                .map(|s| s.split_whitespace().next().unwrap().parse().unwrap())
        })
        .unwrap();
    (current, usage.ru_maxrss as u64)
}

#[test]
#[ignore = "显式提供实录，发布测试程序独立进程执行3000和10000帧，主代理调度重负载"]
fn pgo_streamed_recorded_memory_cost() {

    let path = PathBuf::from(std::env::var_os("NUWAX_PGO_RECORDED_SCANS").expect("指定原生实录"));
    let frames: usize = std::env::var("NUWAX_PGO_RECORDED_FRAMES")
        .expect("指定3000或10000帧")
        .parse()
        .unwrap();
    assert!(frames >= 3);
    let origin: [f64; 3] =
        serde_json::from_str(&std::env::var("NUWAX_PGO_SENSOR_ORIGIN_BODY").expect("指定雷达原点"))
            .unwrap();
    let evidence = PathBuf::from(std::env::var_os("NUWAX_G2_EVIDENCE_DIR").expect("指定证据目录"));
    std::fs::create_dir_all(&evidence).unwrap();
    let mut owner = DiskTestOwner::new(Some(evidence.join("原始扫描日志")));
    let initial_usage = disk_usage();
    let mut input = StreamedRecordedInput::open(&path);
    let mut cloud = CloudMapShmWriter::new_private_for_test().unwrap();
    cloud.begin_session(1).unwrap();
    let config = disk_config();
    let mut kernel = MappingBackendSpec::Voxel.into_kernel(config.window_capacity);
    let mut persistent = None;
    let mut history = ScanHistory::default();
    let mut raw_nodes = Vec::new();
    let mut source_points = 0u64;
    let mut max_frame_points = 0usize;
    let mut max_history_points = 0usize;
    let mut max_online_rss = 0u64;
    let mut classify_s = Duration::ZERO;
    let mut kernel_s = Duration::ZERO;
    let mut cloud_s = Duration::ZERO;
    let mut history_s = Duration::ZERO;
    let online_started = Instant::now();
    for index in 0..frames {
        let (stamp, pose, body) = input.next().expect("实录不足指定帧数");
        max_frame_points = max_frame_points.max(body.len());
        source_points += body.len() as u64;
        let points: Vec<_> = body.iter().map(|p| pose * p).collect();
        let source = disk_source(index, stamp, pose, points.clone(), origin, 1);
        let started = Instant::now();
        let frame = cloud.classify_admitted_scan(&source).unwrap();
        classify_s += started.elapsed();
        let started = Instant::now();
        history.record(source.scan(), stamp).unwrap();
        history.bind_replay_source(cloud.pgo_replay_source());
        if [0, frames / 2, frames - 1].contains(&index) {
            source.scan().retain_pgo_keyframe().unwrap();
            raw_nodes.push((stamp, source.scan().binding().frozen_odom_from_body));
        }
        history_s += started.elapsed();
        max_history_points = max_history_points.max(
            history
                .stream
                .latest
                .as_ref()
                .unwrap()
                .body_points
                .len(),
        );
        let sensor_origin = (pose * Point3::from(origin)).coords.into();
        let backend_input = MappingBackendInput {
            session_id: "bounded-history".into(),
            session_generation: 1,
            backend_generation: 1,
            frame_sequence: index as u64 + 1,
            source_timestamp_ns: stamp,
            received_at_ns: stamp,
            registered_frame_id: "map".into(),
            sensor_origin_map: sensor_origin,
            dynamic_frame: Some(frame),
            registered_points_map: points.clone(),
        };
        let started = Instant::now();
        mapping_backend::validate_backend_input(&kernel, &backend_input).unwrap();
        if let Some(candidate) =
            mapping_backend::compute_candidate_in_place(&mut kernel, &backend_input, config)
                .unwrap()
        {
            persistent = Some(
                PersistentOccupancyGrid::stage_observation(
                    persistent.as_ref(),
                    &candidate.latest_frame_occupancy,
                )
                .unwrap(),
            );
        }
        kernel_s += started.elapsed();
        let started = Instant::now();
        assert!(cloud.accumulate_mapping_scan_with_source(
            1,
            &points,
            Some(sensor_origin),
            stamp,
            Some(&AcceptedMappingCloud::from(source))
        ));
        cloud_s += started.elapsed();
        max_online_rss = max_online_rss.max(disk_usage().0);
        if (index + 1) % 100 == 0 {
            eprintln!(
                "G2_ONLINE frames={} source_points={} history_points={} rss_kib={} elapsed_s={:.3}",
                index + 1,
                source_points,
                max_history_points,
                max_online_rss,
                online_started.elapsed().as_secs_f64()
            );
        }
    }
    let online_wall_s = online_started.elapsed().as_secs_f64();
    let online_usage = disk_usage();
    // 真实关闭写盘拥有方排空队列并同步；不能用放大队列或丢帧重试获得完整输入。
    owner.finish_storage();
    let replay = cloud
        .pgo_replay_source()
        .expect("真实在线日志记录器必须存在");
    let prefix = replay.snapshot().unwrap().expect("后台必须交付完整前缀");
    assert!(prefix.observed_until_ns() >= history.latest_timestamp_ns().unwrap());
    let mut reader = ScanReader::open_prefix(&prefix).unwrap();
    let (mut logged_frames, mut logged_points) = (0usize, 0u64);
    let mut last_logged_origin = None;
    while let Some(scan) = reader.next_scan().unwrap() {
        logged_frames += 1;
        logged_points += scan.points.len() as u64;
        last_logged_origin = Some((
            scan.metadata.source_timestamp_ns,
            scan.metadata.sensor_origin_body.unwrap(),
        ));
    }
    let nodes: Vec<_> = raw_nodes
        .iter()
        .enumerate()
        .map(|(i, (stamp, pose))| {
            let t = i as f64 / 2.;
            let delta = Isometry3::from_parts(
                Translation3::new(-0.08 * t, 0.05 * t, 0.01 * t),
                UnitQuaternion::from_euler_angles(0.005 * t, -0.008 * t, 0.03 * t),
            );
            disk_node(*stamp, delta * pose)
        })
        .collect();
    let disk = history.disk_input(&nodes).unwrap();
    assert_eq!(
        disk.locals,
        raw_nodes.iter().map(|(_, pose)| *pose).collect::<Vec<_>>()
    );
    assert_eq!(history.stream.keyframes.len(), 3);
    let latest = disk.latest.clone();
    let snapshot_started = Instant::now();
    let terminal = cloud.pgo_terminal_dufo();
    let snapshot_s = snapshot_started.elapsed().as_secs_f64();
    let terminal_tree_bytes = terminal.memory_bytes();
    let (terminal_journal_payload_bytes, terminal_journal_revisions) =
        terminal.free_change_payload_stats();
    let snapshot_usage = disk_usage();
    let rebuild_started = Instant::now();
    let (builder, mut cursor) = disk
        .start(
            nodes.clone(),
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            config,
            "map".into(),
            REBUILD_BUDGET,
            terminal,
            &|| false,
        )
        .unwrap();
    let mut builder = builder.into_background();
    let mut slices = 0;
    let mut max_rebuild_rss = disk_usage().0;
    loop {
        let (next_cursor, next_builder, ready) =
            cursor.append_slice(builder, &latest, &|| false).unwrap();
        cursor = next_cursor;
        builder = next_builder;
        slices += 1;
        max_rebuild_rss = max_rebuild_rss.max(disk_usage().0);
        if slices % 100 == 0 {
            eprintln!(
                "G2_REBUILD slices={slices} frames={} elapsed_s={:.3}",
                builder.processed,
                rebuild_started.elapsed().as_secs_f64()
            );
        }
        if ready {
            break;
        }
        std::thread::sleep(REPLAY_YIELD);
    }
    let replayed_frames = builder.processed;
    let processed_evidence_allocated_bytes =
        builder.processed_evidence.as_ref().unwrap().memory_bytes();
    let rebuild_kernel_s = builder.kernel_time.as_secs_f64();
    let rebuild_cloud_s = builder.cloud_time.as_secs_f64();
    let replay_usage = disk_usage();
    let prepare_started = Instant::now();
    // 与普通后台生产入口一致，准备可直接移交的后端恢复状态。
    let (advanced, prepared) = builder.advance(&[], &|| false).unwrap();
    builder = advanced;
    let prepare_s = prepare_started.elapsed().as_secs_f64();
    let prepare_usage = disk_usage();
    let expected_tail = optimized_pose(nodes.last().unwrap());
    // 选中末帧的真实重放原点来自v2单精度字段；未选末帧才沿用实时双精度原点。
    let expected_body_origin = match last_logged_origin {
        Some((stamp, origin)) if stamp == latest.source_timestamp_ns => {
            Point3::from(origin.map(f64::from))
        }
        _ => latest.sensor_origin_body.unwrap(),
    };
    let expected_origin = expected_tail * expected_body_origin;
    assert!(
        (Point3::from(builder.last_input.as_ref().unwrap().sensor_origin_map) - expected_origin)
            .norm()
            < 1e-8,
        "真实重建末帧雷达原点必须使用精确尾节点纠偏"
    );
    let mut rebuilt = builder.finish(prepared, &|| false).unwrap();
    let rebuild_wall_s = rebuild_started.elapsed().as_secs_f64();
    assert_eq!(rebuilt.source_timestamp_ns, latest.source_timestamp_ns);
    assert!(rebuilt.final_poses.is_empty());
    let pose_spool_records = rebuilt.final_pose_spool.as_ref().unwrap().len();
    let pose_spool_bytes = rebuilt.final_pose_spool.as_ref().unwrap().bytes();
    assert_eq!(pose_spool_records, replayed_frames as u64);
    let pose_archive_started = Instant::now();
    let pose_archive = disk_archive_spool(&mut owner, &mut rebuilt);
    let pose_archive_s = pose_archive_started.elapsed().as_secs_f64();
    let static_visible_points = rebuilt.cloud.point_count;
    let checkpoint_bytes = rebuilt.backend_checkpoint.len();
    let typed_backend_restore_prepared = rebuilt.prepared_backend.is_some();
    assert!(
        typed_backend_restore_prepared,
        "基准必须走普通后台的类型化恢复准备"
    );
    let install_started = Instant::now();
    cloud
        .install_prepared_accumulator(
            &disk_receipt(rebuilt.source_timestamp_ns),
            rebuilt.accumulator,
        )
        .unwrap();
    let install_s = install_started.elapsed().as_secs_f64();
    persistent = Some(rebuilt.persistent);
    drop(kernel);
    let final_usage = disk_usage();
    assert!(max_history_points <= max_frame_points);
    let dense_points = cloud.committed_dense_cloud().unwrap().unwrap().point_count;
    let fine_points = cloud.committed_fine_cloud().unwrap().manifest().points();
    // 分段构造同一平面结果，避免大型 json! 递归展开改变 crate 级编译限制。
    let mut output = serde_json::Map::new();
    for section in [
        json!({"source_file":path,"requested_frames":frames,"source_frames":frames,"source_points":source_points,
            "streamed_input":true,"preloaded_scan_history":false,"ground_capture":true,
            "selected_log_frames":logged_frames,"selected_log_points":logged_points,"log_bytes":prefix.valid_bytes(),
            "log_observed_until_ns":prefix.observed_until_ns(),"replayed_frames_including_unselected_tail":replayed_frames}),
        json!({"final_pose_spool_records":pose_spool_records,"final_pose_spool_bytes":pose_spool_bytes,
            "final_pose_archive":pose_archive,"final_pose_archive_s":pose_archive_s,"resident_final_pose_vector":0,
            "max_source_frame_points":max_frame_points,"resident_history_max_points":max_history_points,
            "retained_keyframe_poses":3}),
        json!({"online_wall_s":online_wall_s,"online_classify_s":classify_s.as_secs_f64(),"online_kernel_s":kernel_s.as_secs_f64(),
            "online_cloud_with_ground_capture_s":cloud_s.as_secs_f64(),"online_history_s":history_s.as_secs_f64(),
            "terminal_snapshot_s":snapshot_s,"terminal_tree_bytes":terminal_tree_bytes,"terminal_window_bytes":null,
            "terminal_window_metric_note":"当前OnlineDufo仅暴露树字节；克隆仍包含30帧窗口，未将树值冒充总量"}),
        json!({"rebuild_wall_s":rebuild_wall_s,"rebuild_kernel_s":rebuild_kernel_s,"rebuild_cloud_s":rebuild_cloud_s,
            "rebuild_prepare_s":prepare_s,"install_s":install_s,"rebuild_slices":slices,
            "production_start_budget_ms":REBUILD_BUDGET.as_millis(),"production_slice_ms":REPLAY_SLICE.as_millis(),
            "production_yield_ms":REPLAY_YIELD.as_millis(),"background_mode":true,"maximum_private_lanes":2}),
        json!({"processed_evidence_config_payload_bytes":crate::dynamic_backend::free_evidence::PROCESSED_EVIDENCE_BYTES,
            "free_change_config_payload_bytes":crate::dynamic_backend::free_evidence::FREE_CHANGE_PAYLOAD_BYTES,
            "free_change_config_revision_limit":crate::dynamic_backend::free_evidence::FREE_CHANGE_MAX_REVISIONS,
            "processed_evidence_allocated_bytes":processed_evidence_allocated_bytes,
            "terminal_journal_measured_payload_bytes":terminal_journal_payload_bytes,
            "terminal_journal_measured_revisions":terminal_journal_revisions,
            "bounded_evidence_metric_note":"config为配置上界，allocated含固定过滤器元数据，journal为该次快照有效载荷和条数；不含目录/Arc元数据且共享载荷不重复求和"}),
        json!({"peak_rss_kib_cumulative":{"initial":initial_usage.1,"after_online":online_usage.1,"after_snapshot":snapshot_usage.1,
            "after_replay":replay_usage.1,"after_prepare":prepare_usage.1,"after_install":final_usage.1},
            "sampled_current_rss_kib":{"online_max_after_frame":max_online_rss,"rebuild_max_after_slice":max_rebuild_rss,
            "after_online":online_usage.0,"after_snapshot":snapshot_usage.0,"after_prepare":prepare_usage.0,"after_install":final_usage.0}}),
        json!({"final_visible_points":static_visible_points,"final_fine_points":fine_points,"final_dense_points":dense_points,
            "final_occupancy_cells":persistent.as_ref().unwrap().grid.data.len(),"backend_checkpoint_bytes":checkpoint_bytes,"typed_backend_restore_prepared":typed_backend_restore_prepared,
            "trajectory":"三个真实对应源帧的精确冻结位姿，加非恒等滚转/俯仰/偏航及平移纠偏；不测求解器",
            "optimized_nodes":nodes.iter().map(|node|json!({"stamp":node.source_timestamp_ns,"translation":node.translation_m,"rotation_xyzw":node.rotation_xyzw})).collect::<Vec<_>>()}),
    ] {
        let serde_json::Value::Object(section) = section else {
            unreachable!()
        };
        output.extend(section);
    }
    std::fs::write(
        evidence.join("内存实录结果.json"),
        serde_json::to_vec_pretty(&output).unwrap(),
    )
    .unwrap();
    eprintln!(
        "G2_MEMORY_RESULT {}",
        serde_json::to_string(&output).unwrap()
    );
}

#[test]
fn streamed_long_history_keeps_progress_during_continuous_unrelated_free_changes() {

    use crate::dynamic_backend::OnlineDufo;
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let mut owner = DiskTestOwner::new(None);
    let mut recorder = owner.recorder(1);
    let replay = recorder.replay_source();
    let mut history = ScanHistory::default();
    history.bind_replay_source(Some(replay.clone()));
    let frame = Arc::new(MappedFrame::unfiltered());
    let point = Point3::new(8., 0.03, 0.65);
    let mut nodes = Vec::new();
    for index in 0..1500 {
        let stamp = 1_000_000_000 + index as u64 * 10_000_000;
        let source = disk_source(index, stamp, Isometry3::identity(), vec![point], ORIGIN, 1);
        source
            .scan()
            .record_static_indices(vec![0], &frame)
            .unwrap();
        recorder
            .observe(
                disk_metadata(source.scan()),
                source
                    .scan()
                    .body_points()
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            )
            .unwrap();
        history.record(source.scan(), stamp).unwrap();
        if [0, 1499].contains(&index) {
            source.scan().retain_pgo_keyframe().unwrap();
            nodes.push(disk_node(stamp, Isometry3::identity()));
        }
        // 功能夹具等待真实写入进度，不放大生产队列，也不把队列丢帧当成功。
        disk_wait_written(&replay, stamp);
    }
    owner.finish_storage();
    let (near, _) = corridor(None, 0.);
    let mut online = OnlineDufo::default();
    online
        .observe(&near, ORIGIN, 1, Isometry3::identity())
        .unwrap();
    assert!(!online
        .terminal_frame(&[point], &[point])
        .unwrap()
        .is_dynamic([point.x, point.y, point.z]));
    let input = history.disk_input(&nodes).unwrap();
    let latest = input.latest.clone();
    let (terminal, changes) = online.snapshot_since(online.free_revision());
    assert_eq!(changes, Some(Vec::new()));
    let (builder, mut cursor) = input
        .start(
            nodes.clone(),
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            terminal,
            &|| false,
        )
        .unwrap();
    let mut builder = builder.into_background();
    let mut updates = 0u64;
    loop {
        let (next_cursor, next_builder, ready) =
            cursor.append_slice(builder, &latest, &|| false).unwrap();
        cursor = next_cursor;
        builder = next_builder;
        if ready {
            break;
        }
        let shift = 100. + updates as f64 * 20.;
        let far: Vec<_> = near
            .iter()
            .map(|p| p + nalgebra::Vector3::new(shift, 0., 0.))
            .collect();
        online
            .observe(
                &far,
                [ORIGIN[0] + shift, ORIGIN[1], ORIGIN[2]],
                updates + 2,
                Isometry3::identity(),
            )
            .unwrap();
        let old_revision = builder.terminal_dufo.as_ref().unwrap().free_revision();
        let (tree, changes) = online.snapshot_since(old_revision);
        assert!(
            tree.free_revision() > old_revision,
            "每片之间都必须发生真实新增自由证据"
        );
        assert!(changes.as_ref().is_some_and(|cells| !cells.is_empty()));
        let processed = builder.processed;
        let stamp = builder.latest_timestamp_ns();
        assert!(processed > 0, "先重放保留点，再检验相关性");
        assert!(
            builder.update_terminal(tree, changes.as_deref()).is_ok(),
            "远处持续探索不应重启已经处理的近处历史"
        );
        assert_eq!(builder.processed, processed, "换树后处理进度必须保持");
        assert_eq!(
            builder.latest_timestamp_ns(),
            stamp,
            "换树后原游标尾部必须保持"
        );
        updates += 1;
        std::thread::sleep(REPLAY_YIELD);
    }
    assert!(updates >= 8, "长日志必须跨多个切片持续接收无关变化");
    assert_eq!(builder.latest_timestamp_ns(), latest.source_timestamp_ns);
    let retained_terminal = builder.terminal_dufo.as_ref().unwrap().clone();
    let processed = builder.processed;
    let prepared = builder.prepare(&|| false).unwrap();
    let rebuilt = builder.finish(prepared, &|| false).unwrap();
    assert!(rebuilt.final_poses.is_empty());
    assert_eq!(
        rebuilt.final_pose_spool.as_ref().unwrap().len(),
        processed as u64
    );
    assert_eq!(rebuilt.source_timestamp_ns, latest.source_timestamp_ns);
    assert!(
        rebuilt.cloud.point_count > 0,
        "持续探索下仍须实际完成地图重建"
    );

    // 另起同一日志的候选，已处理点后来被真实射线打空时必须交回新树以重启。
    let input = history.disk_input(&nodes).unwrap();
    let (builder, cursor) = input
        .start(
            nodes,
            Isometry3::identity(),
            MappingBackendSpec::Voxel,
            disk_config(),
            "map".into(),
            REBUILD_BUDGET,
            retained_terminal,
            &|| false,
        )
        .unwrap();
    let (cursor, mut builder, _) = cursor
        .append_slice(builder.into_background(), &latest, &|| false)
        .unwrap();
    assert!(builder.processed > 0);
    let old_revision = builder.terminal_dufo.as_ref().unwrap().free_revision();
    let old_progress = builder.processed;
    let through: Vec<_> = near
        .iter()
        .map(|p| p + nalgebra::Vector3::new(4., 0., 0.))
        .collect();
    for index in 0..32 {
        online
            .observe(
                &through,
                [ORIGIN[0] + 4., ORIGIN[1], ORIGIN[2]],
                updates + 100 + index,
                Isometry3::identity(),
            )
            .unwrap();
    }
    assert!(
        online
            .terminal_frame(&[point], &[point])
            .unwrap()
            .is_dynamic([point.x, point.y, point.z]),
        "新走廊射线必须真实穿过已经处理的旧墙面点"
    );
    let (tree, changes) = online.snapshot_since(old_revision);
    assert!(changes.is_some(), "相邻真实修订仍在有限日志中");
    let replacement = builder
        .update_terminal(tree, changes.as_deref())
        .err()
        .expect("命中已处理保留点时必须要求重启");
    assert!(replacement.free_revision() > old_revision);
    assert_eq!(
        builder.processed, old_progress,
        "拒绝更新时旧候选进度保持，等待拥有方统一重启"
    );
    assert_eq!(
        builder.terminal_dufo.as_ref().unwrap().free_revision(),
        old_revision
    );
    drop(cursor);
    drop(builder);
}
