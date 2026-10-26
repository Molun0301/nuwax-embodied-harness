//! Actual accumulated saved assets. Synthetic rays are not robot recordings.
use super::*;
use crate::completed_cleanup::{prepare, Request};
use crate::dynamic_scene::tests::{corridor, ORIGIN};
use crate::lio_ingress::{AcceptedRegisteredCloud, CorrelatedOdometry, RegistrationBinding};
use crate::mapping_backend::{
    compute_candidate_in_place_with_interval, export_mapping_kernel_checkpoint,
    MappingBackendComputeConfig, MappingBackendInput, MappingBackendSpec,
};
use nuwax_cloud_cleanup::offline_residual_cleanup::{asset_key, CompletedMap};
use std::{fs::File, path::PathBuf};
fn directory(name: &str) -> PathBuf {
    let root = std::env::var_os("NEH_CLEANUP_EVIDENCE_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    let folder = root.join(format!("{name}-{}", uuid::Uuid::new_v4().simple()));
    std::fs::create_dir_all(&folder).unwrap();
    folder
}
fn archive_file(mut file: File, path: &std::path::Path) {
    let mut out = File::create(path).unwrap();
    std::io::copy(&mut file, &mut out).unwrap();
    out.sync_all().unwrap();
}
fn keyset(map: &CompletedMap) -> ahash::AHashSet<[i32; 3]> {
    map.keys.iter().copied().collect()
}
#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "190帧×8601点的保存与全量离线重放，使用发布构建独立验收"
)]
fn completed_cleanup_saved_corridor_ten_seconds_after_departure() {
    run_saved_case(false, None);
}
#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "190帧×8601点的保存与全量离线重放，使用发布构建独立验收"
)]
fn completed_cleanup_saved_poles_legs_stairs_box() {
    run_saved_case(true, None);
}
#[test]
#[ignore = "需要原生配对实录文件，使用发布构建独立运行"]
fn completed_cleanup_recorded_original_scans() {
    let source = PathBuf::from(
        std::env::var_os("NEH_CLEANUP_RECORDED_SCANS").expect("指定原生配对扫描实录"),
    );
    run_saved_case(false, Some(&source));
}

fn recorded_frames(
    path: &std::path::Path,
) -> Vec<(u64, nalgebra::Isometry3<f64>, Vec<Point3<f64>>)> {
    use std::io::Read;
    let mut file = std::io::BufReader::new(File::open(path).unwrap());
    let mut scans = Vec::new();
    for _ in 0..190 {
        let mut header = [0; 68];
        file.read_exact(&mut header).expect("实录必须完整包含190帧");
        let stamp = u64::from_le_bytes(header[..8].try_into().unwrap());
        let pose: [f64; 7] = std::array::from_fn(|i| {
            f64::from_le_bytes(header[8 + i * 8..16 + i * 8].try_into().unwrap())
        });
        let size = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
        let mut bytes = vec![0; size * 12];
        file.read_exact(&mut bytes).unwrap();
        let odom_from_body = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(pose[0], pose[1], pose[2]),
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                pose[6], pose[3], pose[4], pose[5],
            )),
        );
        let points = bytes
            .chunks_exact(12)
            .map(|b| {
                odom_from_body
                    * Point3::new(
                        f32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
                        f32::from_le_bytes(b[4..8].try_into().unwrap()) as f64,
                        f32::from_le_bytes(b[8..12].try_into().unwrap()) as f64,
                    )
            })
            .collect();
        scans.push((stamp, odom_from_body, points));
    }
    scans
}

fn run_saved_case(rich: bool, recorded: Option<&std::path::Path>) {
    let folder = directory(if recorded.is_some() {
        "recorded-original-190-frames"
    } else if rich {
        "synthetic-static-structures"
    } else {
        "synthetic-corridor-initial-stand-slow-walk"
    });
    let session = format!("saved-human-{}", uuid::Uuid::new_v4().simple());
    let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
    assert!(writer.begin_session(1).is_some());
    let config = MappingBackendComputeConfig {
        resolution: 0.1,
        initial_safe_radius_meters: 0.0,
        window_capacity: 200_000,
    };
    let mut kernel = MappingBackendSpec::Voxel.into_kernel(config.window_capacity);
    let mut human = ahash::AHashSet::new();
    let mut static_keys = ahash::AHashSet::new();
    let mut structure_keys: std::collections::BTreeMap<&str, ahash::AHashSet<[i32; 3]>> =
        Default::default();
    let mut last_grid = None;
    let mut persistent: Option<crate::persistent_occupancy::PersistentOccupancyGrid> = None;
    let start = std::time::Instant::now();
    let mut stages = [0.0; 5];
    let recorded_frames = recorded.map(recorded_frames);
    let mut last_stamp = 0;
    let mut total_points = 0;
    for frame in 1..=190u64 {
        // 合成场景保留原射线密度与机体尺寸，最后100帧覆盖人离开十秒；实录使用原点与位姿。
        let person = if frame <= 50 {
            Some(0.)
        } else if frame <= 90 {
            Some((frame - 50) as f64 * 0.10)
        } else {
            None
        };
        let (points, truth, kinds, stamp, odom_from_body, origin) =
            if let Some(scans) = &recorded_frames {
                let (stamp, pose, points) = &scans[(frame - 1) as usize];
                let origin: [f64; 3] = (*pose * Point3::new(0.28945, 0., -0.046825)).coords.into();
                (
                    points.clone(),
                    vec![false; points.len()],
                    vec!["unlabeled_recorded"; points.len()],
                    *stamp,
                    *pose,
                    origin,
                )
            } else {
                let (points, truth, kinds) = scene(person, (frame % 3) as f64, rich);
                (
                    points,
                    truth,
                    kinds,
                    frame * 100_000_000,
                    nalgebra::Isometry3::identity(),
                    ORIGIN,
                )
            };
        last_stamp = stamp;
        total_points += points.len();
        for ((p, is_human), kind) in points.iter().zip(&truth).zip(kinds) {
            let k = asset_key([p.x, p.y, p.z], VOXEL_SIZE);
            if *is_human {
                human.insert(k);
            } else {
                static_keys.insert(k);
                structure_keys.entry(kind).or_default().insert(k);
            }
        }
        let binding = RegistrationBinding {
            runtime_generation: 1,
            cloud_source_sequence: frame as i64,
            correlated_odometry: CorrelatedOdometry {
                source_sequence: frame as i64,
                source_timestamp_ns: stamp as i64,
            },
            mapping_session_id: session.clone(),
            mapping_session_generation: 1,
            map_frame_generation: 0,
            frozen_odom_from_body: odom_from_body,
            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
            registration_map_from_odom: nalgebra::Isometry3::identity(),
            registration_correction_revision: 0,
            sensor_origin_map: Some(origin),
            frame_capture_relocalization_epoch: Some(0),
        };
        let accepted = AcceptedRegisteredCloud::from_map_points_for_test(
            points.clone(),
            stamp,
            stamp,
            "map",
            "base_link",
            binding,
        );
        let stage = std::time::Instant::now();
        let shared = writer.classify_admitted_scan(&accepted).unwrap();
        stages[0] += stage.elapsed().as_secs_f64();
        assert_eq!(shared.decision.labels.labels.len(), points.len());
        let input = MappingBackendInput {
            session_id: session.clone(),
            session_generation: 1,
            backend_generation: 1,
            frame_sequence: frame,
            source_timestamp_ns: stamp,
            received_at_ns: stamp,
            registered_frame_id: "map".into(),
            sensor_origin_map: origin,
            dynamic_frame: Some(shared),
            registered_points_map: points.clone(),
        };
        let stage = std::time::Instant::now();
        let candidate = compute_candidate_in_place_with_interval(&mut kernel, &input, config, 0)
            .unwrap()
            .unwrap();
        assert_eq!(
            candidate.accepted_points_map, points,
            "raw current returns stay available to realtime obstacles"
        );
        stages[1] += stage.elapsed().as_secs_f64();
        let stage = std::time::Instant::now();
        persistent = Some(
            crate::persistent_occupancy::PersistentOccupancyGrid::stage_observation(
                persistent.as_ref(),
                &candidate.latest_frame_occupancy,
            )
            .unwrap(),
        );
        last_grid = Some(persistent.as_ref().unwrap().grid.clone());
        stages[2] += stage.elapsed().as_secs_f64();
        let stage = std::time::Instant::now();
        let source = crate::admitted_cloud::AcceptedMappingCloud::from(accepted);
        assert!(
            writer.accumulate_mapping_scan_with_source(
                1,
                &points,
                Some(origin),
                stamp,
                Some(&source)
            ),
            "frame {frame}"
        );
        stages[3] += stage.elapsed().as_secs_f64();
        let stage = std::time::Instant::now();
        let receipt = OccupancyCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 133,
                sequence: frame,
            },
            session_id: session.clone(),
            session_generation: 1,
            occupancy_revision: frame,
            source_timestamp_ns: stamp,
            backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
            backend_generation: 1,
        };
        writer
            .commit_accumulated_snapshot(&receipt)
            .unwrap()
            .unwrap();
        stages[4] += stage.elapsed().as_secs_f64();
    }
    let online_seconds = start.elapsed().as_secs_f64();
    let source_cloud_revision = writer.last_cloud_receipt().unwrap().cloud_revision;
    let fine = writer.committed_fine_cloud().unwrap();
    archive_file(
        nuwax_map_chunks::archive_file(fine.manifest(), &fine.repository)
            .unwrap()
            .0,
        &folder.join("original-fine.bin"),
    );
    let dense = writer.committed_dense_cloud().unwrap().unwrap();
    archive_file(
        dense.archive("map").unwrap().0,
        &folder.join("original-dense.bin"),
    );
    std::fs::write(
        folder.join("original-backend.bin"),
        export_mapping_kernel_checkpoint(&kernel).unwrap(),
    )
    .unwrap();
    let original_cloud = writer.committed_snapshot().unwrap();
    std::fs::write(
        folder.join("original-cloud.xyzf32"),
        &original_cloud.xyz_f32_le,
    )
    .unwrap();
    let grid = last_grid.unwrap();
    std::fs::write(
        folder.join("original-occupancy.bin"),
        grid.data.iter().map(|&v| v as u8).collect::<Vec<_>>(),
    )
    .unwrap();
    let logs = std::env::var_os("NUWAX_RAW_SCAN_DIRECTORY")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("runtime_data/scan_logs"));
    let log = nuwax_cloud_cleanup::scan_log::session_path(
        &logs,
        &nuwax_cloud_cleanup::scan_log::Session {
            session_id: session.clone(),
            generation: 1,
        },
    );
    std::fs::copy(log, folder.join("raw-scans.scan")).unwrap();
    let descriptor = serde_json::json!({"width":grid.width,"height":grid.height,"resolution":grid.resolution,"origin_x":grid.origin[0],"origin_y":grid.origin[1],"frame_id":"map","timestamp_ns":last_stamp});
    std::fs::write(
        folder.join("occupancy-descriptor.json"),
        serde_json::to_vec_pretty(&descriptor).unwrap(),
    )
    .unwrap();
    let request = Request {
        fine: folder.join("original-fine.bin"),
        dense: folder.join("original-dense.bin"),
        output: folder.join("cleaned"),
        strength_percent: 50,
        scan_log: Some(folder.join("raw-scans.scan")),
        final_poses: None,
        source_timestamp_ns: last_stamp,
        cancel_file: None,
        backend: Some(folder.join("original-backend.bin")),
        occupancy: Some(folder.join("original-occupancy.bin")),
        occupancy_descriptor: Some(descriptor),
    };
    let original_fine = std::fs::read(&request.fine).unwrap();
    let original_dense = std::fs::read(&request.dense).unwrap();
    let before = CompletedMap::read(&request.fine).unwrap();
    let cleanup_start = std::time::Instant::now();
    let report = prepare(&request).unwrap();
    let cleanup_seconds = cleanup_start.elapsed().as_secs_f64();
    let after = CompletedMap::read(&request.output.join("fine.bin")).unwrap();
    let before_keys = keyset(&before);
    let after_keys = keyset(&after);
    let score = |keys: &ahash::AHashSet<[i32; 3]>| {
        let mut bins = [0usize; 3];
        let mut highest = 0f64;
        for k in keys.intersection(&human) {
            if static_keys.contains(k) {
                continue;
            }
            let height = (k[2] as f64 + 0.5) * f64::from(VOXEL_SIZE);
            highest = highest.max(height);
            bins[if height >= 0.15 {
                0
            } else if height > 0.05 {
                1
            } else {
                2
            }] += 1;
        }
        (bins, highest)
    };
    let (before_residual, before_highest) = score(&before_keys);
    let (after_residual, after_highest) = score(&after_keys);
    let static_removed = before_keys
        .difference(&after_keys)
        .filter(|k| static_keys.contains(*k))
        .count();
    let structures=structure_keys.iter().map(|(kind,keys)|(*kind,serde_json::json!({"observed_unique_voxels":keys.len(),"present_before":before_keys.intersection(keys).count(),"deleted":before_keys.difference(&after_keys).filter(|k|keys.contains(*k)).count()}))).collect::<std::collections::BTreeMap<_,_>>();
    let result = serde_json::json!({"fixture":if recorded.is_some(){"original paired robot recording, first 190 frames, all original returns; no human truth labels"}else if rich{"synthetic room with early/revealed poles, table legs, stairs and box; not a recording"}else{"existing synthetic ray-cast corridor, not a robot recording"},"structures":structures,"frames":190,"source_points_per_frame":if recorded.is_some(){total_points/190}else{61*141},"source_points_total":total_points,"source_points_per_frame_min":recorded_frames.as_ref().map(|v|v.iter().map(|f|f.2.len()).min().unwrap()),"source_points_per_frame_max":recorded_frames.as_ref().map(|v|v.iter().map(|f|f.2.len()).max().unwrap()),"recorded_source":recorded,"empty_tail_seconds":if recorded.is_none(){Some(10)}else{None},"truth_labels_available":recorded.is_none(),
        "before_human_ge15_5to15_le5_cm":if recorded.is_none(){Some(before_residual)}else{None},"after_human_ge15_5to15_le5_cm":if recorded.is_none(){Some(after_residual)}else{None},
        "before_human_highest_m":if recorded.is_none(){Some(before_highest)}else{None},"after_human_highest_m":if recorded.is_none(){Some(after_highest)}else{None},"new_static_voxels_deleted":if recorded.is_none(){Some(static_removed)}else{None},
        "preview_candidate_voxels":report.preview.candidate_points,"actual_removed_voxels":before_keys.difference(&after_keys).count(),"recovered_voxels":report.recovered_voxels,
        "seconds":start.elapsed().as_secs_f64(),"source_session":session,"source_cloud_revision":source_cloud_revision,"online_seconds":online_seconds,"cleanup_seconds":cleanup_seconds,"stage_seconds":{"classify":stages[0],"kernel":stages[1],"persistent_occupancy":stages[2],"accumulate":stages[3],"commit":stages[4]}});
    std::fs::write(
        folder.join("acceptance.json"),
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
    eprintln!("NEH_SAVED_RESULT {result}");
    assert_eq!(std::fs::read(&request.fine).unwrap(), original_fine);
    assert_eq!(std::fs::read(&request.dense).unwrap(), original_dense);
    assert_eq!(
        report.preview.candidate_points,
        before_keys.difference(&after_keys).count()
    );
    assert_eq!(report.offline_frames, 190);
    assert!(report.offline_complete);
    let mut dense_file = File::open(request.output.join("dense.bin")).unwrap();
    crate::dense_cloud::validate_archive(&mut dense_file).unwrap();
    assert_eq!(
        std::fs::metadata(request.output.join("cloud.xyzf32"))
            .unwrap()
            .len(),
        after.keys.len() as u64 * 12
    );
    assert!(
        std::fs::metadata(request.output.join("backend.bin"))
            .unwrap()
            .len()
            > 0
    );
    // 从磁盘新建恢复拥有方，核对清理产物确实可以独立恢复。
    let cleaned_payload = std::fs::read(request.output.join("backend.bin")).unwrap();
    let restored_worker =
        crate::mapping_backend::MappingBackendWorker::start(MappingBackendSpec::Voxel, 2, config)
            .unwrap();
    let restored_client = restored_worker.client();
    restored_client
        .restore_checkpoint(cleaned_payload.clone())
        .unwrap()
        .blocking_recv()
        .unwrap()
        .unwrap()
        .commit()
        .unwrap();
    assert_eq!(
        restored_client
            .export_checkpoint()
            .unwrap()
            .blocking_recv()
            .unwrap()
            .unwrap(),
        cleaned_payload
    );
    restored_worker.stop_and_join().unwrap();
    let imported = prepare_fine_cloud_archive(
        File::open(request.output.join("fine.bin")).unwrap(),
        &session,
        1,
        source_cloud_revision + 1,
        last_stamp,
    )
    .unwrap();
    assert_eq!(imported.voxel_size(), VOXEL_SIZE);
    let mut dense_archive = File::open(request.output.join("dense.bin")).unwrap();
    let (dense_metadata, _) = crate::dense_cloud::read_metadata(&mut dense_archive).unwrap();
    let restored_dense =
        crate::dense_cloud::DenseCloud::from_archive(dense_archive, &dense_metadata).unwrap();
    let mut cloud_receipt = writer.last_cloud_receipt().unwrap();
    cloud_receipt.cloud_revision += 1;
    assert_eq!(
        restored_dense
            .snapshot(&cloud_receipt, &session)
            .unwrap()
            .point_count,
        report.dense_after as u64
    );
    let cancel = folder.join("cancel");
    std::fs::write(&cancel, b"cancel").unwrap();
    let mut cancelled = request;
    cancelled.cancel_file = Some(cancel);
    cancelled.output = folder.join("cancelled");
    assert!(prepare(&cancelled).is_err());
    assert_eq!(std::fs::read(&cancelled.fine).unwrap(), original_fine);
    if recorded.is_none() {
        assert_eq!(
            after_residual[0], 0,
            ">=15cm human residuals must be solved; 5–15cm are reported separately"
        );
        assert_eq!(
            static_removed, 0,
            "cleanup must preserve retained static geometry"
        );
    }
}

#[test]
#[ignore = "requires an existing cumulative saved-map fixture; no source regeneration"]
fn completed_cleanup_saved_reprocess_existing() {
    let source =
        PathBuf::from(std::env::var_os("NEH_REPROCESS_CASE").expect("existing saved case"));
    let folder = directory("reprocess-original-saved-corridor");
    let old: serde_json::Value =
        serde_json::from_reader(File::open(source.join("acceptance.json")).unwrap()).unwrap();
    assert_eq!(old["frames"], 190);
    assert_eq!(old["source_points_per_frame"], 8601);
    let request = Request {
        fine: source.join("original-fine.bin"),
        dense: source.join("original-dense.bin"),
        output: folder.join("cleaned"),
        strength_percent: 50,
        scan_log: Some(source.join("raw-scans.scan")),
        final_poses: None,
        source_timestamp_ns: 19_000_000_000,
        cancel_file: None,
        backend: Some(source.join("original-backend.bin")),
        occupancy: None,
        occupancy_descriptor: None,
    };
    let mut human = ahash::AHashSet::new();
    let mut static_keys = ahash::AHashSet::new();
    for frame in 1..=190u64 {
        let person = if frame <= 50 {
            Some(0.)
        } else if frame <= 90 {
            Some((frame - 50) as f64 * 0.10)
        } else {
            None
        };
        let (points, truth) = corridor(person, (frame % 3) as f64);
        for (p, h) in points.iter().zip(truth) {
            let k = asset_key([p.x, p.y, p.z], VOXEL_SIZE);
            if h {
                human.insert(k);
            } else {
                static_keys.insert(k);
            }
        }
    }
    let start = std::time::Instant::now();
    let report = prepare(&request).unwrap();
    let before = keyset(&CompletedMap::read(&request.fine).unwrap());
    let after_map = CompletedMap::read(&request.output.join("fine.bin")).unwrap();
    let after = keyset(&after_map);
    let mut bins = [0usize; 3];
    let mut highest = 0f64;
    for k in after.intersection(&human) {
        if static_keys.contains(k) {
            continue;
        }
        let h = (k[2] as f64 + 0.5) * f64::from(VOXEL_SIZE);
        highest = highest.max(h);
        bins[if h >= 0.15 {
            0
        } else if h > 0.05 {
            1
        } else {
            2
        }] += 1;
    }
    let lost = before
        .difference(&after)
        .filter(|k| static_keys.contains(*k))
        .count();
    let result = serde_json::json!({"source_case":source,"fixture":"same immutable 190-frame saved synthetic corridor, not a recording","after_human_ge15_5to15_le5_cm":bins,"after_human_highest_m":highest,"new_static_voxels_deleted":lost,"preview_candidate_voxels":report.preview.candidate_points,"actual_removed_voxels":before.difference(&after).count(),"seconds":start.elapsed().as_secs_f64()});
    std::fs::write(
        folder.join("acceptance.json"),
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
    eprintln!("NEH_REPROCESS_RESULT {result}");
    assert_eq!(
        report.preview.candidate_points,
        before.difference(&after).count()
    );
    assert_eq!(bins[0], 0);
    assert_eq!(lost, 0);
}

fn scene(
    person: Option<f64>,
    phase: f64,
    rich: bool,
) -> (Vec<Point3<f64>>, Vec<bool>, Vec<&'static str>) {
    let (mut points, mut human) = corridor(person, phase);
    let mut kinds = vec!["room"; points.len()];
    if !rich {
        return (points, human, kinds);
    }
    for (index, p) in points.iter_mut().enumerate() {
        let delta = p.coords - nalgebra::Vector3::from(ORIGIN);
        let mut range = delta.norm();
        let direction = delta / range;
        for (x, y, r, h, kind) in [
            (2.5, 0.8, 0.015, 1.6, "early_pole"),
            (3.5, 0., 0.015, 1.6, "revealed_pole"),
            (4., -1.0, 0.025, 0.8, "table_legs"),
            (4., -1.8, 0.025, 0.8, "table_legs"),
            (4.7, -1.0, 0.025, 0.8, "table_legs"),
            (4.7, -1.8, 0.025, 0.8, "table_legs"),
        ] {
            let dx = ORIGIN[0] - x;
            let dy = ORIGIN[1] - y;
            let a = direction.x * direction.x + direction.y * direction.y;
            let b = 2. * (dx * direction.x + dy * direction.y);
            let c = dx * dx + dy * dy - r * r;
            let discriminant = b * b - 4. * a * c;
            if discriminant >= 0. {
                let t = (-b - discriminant.sqrt()) / (2. * a);
                let z = ORIGIN[2] + t * direction.z;
                if t > 0. && t < range && z >= 0. && z <= h {
                    range = t;
                    human[index] = false;
                    kinds[index] = kind;
                }
            }
        }
        for (lo, hi, kind) in [
            ([3.9, -1.9, 0.8], [4.8, -0.9, 0.85], "table_top"),
            ([5., 0.6, 0.], [5.6, 1.2, 0.7], "box"),
            ([4.5, 1.5, 0.], [4.8, 2.3, 0.15], "stairs"),
            ([4.8, 1.5, 0.], [5.1, 2.3, 0.30], "stairs"),
            ([5.1, 1.5, 0.], [5.4, 2.3, 0.45], "stairs"),
        ] {
            let mut enter = 0f64;
            let mut exit = f64::INFINITY;
            for a in 0..3 {
                if direction[a].abs() < 1e-12 {
                    if ORIGIN[a] < lo[a] || ORIGIN[a] > hi[a] {
                        exit = -1.;
                    }
                } else {
                    let x = (lo[a] - ORIGIN[a]) / direction[a];
                    let y = (hi[a] - ORIGIN[a]) / direction[a];
                    enter = enter.max(x.min(y));
                    exit = exit.min(x.max(y));
                }
            }
            if enter > 0. && enter <= exit && enter < range {
                range = enter;
                human[index] = false;
                kinds[index] = kind;
            }
        }
        *p = Point3::from(nalgebra::Vector3::from(ORIGIN) + direction * range);
    }
    (points, human, kinds)
}

#[test]
fn completed_cleanup_saved_floating_without_raw_log() {
    let folder = directory("synthetic-floating-support");
    let mut points = Vec::new();
    for x in -20..=20 {
        for y in -20..=20 {
            points.push(Point3::new(x as f64 * 0.05, y as f64 * 0.05, 0.));
        }
    }
    for z in 1..=30 {
        points.push(Point3::new(0., 0., z as f64 * 0.05));
    }
    let static_keys: ahash::AHashSet<_> = points
        .iter()
        .map(|p| asset_key([p.x, p.y, p.z], VOXEL_SIZE))
        .collect();
    for (x, z) in [(0.5, 0.20), (-0.5, 0.30)] {
        for a in 0..3 {
            for b in 0..3 {
                points.push(Point3::new(x + a as f64 * 0.03, 0.5 + b as f64 * 0.03, z));
            }
        }
    }
    let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
    assert!(writer.begin_session(1).is_some());
    assert!(writer.accumulate_mapping_scan(1, &points, Some([0., 0., 0.65]), 100));
    let receipt = OccupancyCommitReceipt {
        commit_id: crate::mapping_commit::MappingCommitId {
            boot_nonce: 134,
            sequence: 1,
        },
        session_id: "floating-support".into(),
        session_generation: 1,
        occupancy_revision: 1,
        source_timestamp_ns: 100,
        backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
        backend_generation: 1,
    };
    writer
        .commit_accumulated_snapshot(&receipt)
        .unwrap()
        .unwrap();
    let fine = writer.committed_fine_cloud().unwrap();
    archive_file(
        nuwax_map_chunks::archive_file(fine.manifest(), &fine.repository)
            .unwrap()
            .0,
        &folder.join("original-fine.bin"),
    );
    let dense = writer.committed_dense_cloud().unwrap().unwrap();
    archive_file(
        dense.archive("map").unwrap().0,
        &folder.join("original-dense.bin"),
    );
    std::fs::write(
        folder.join("original-cloud.xyzf32"),
        &writer.committed_snapshot().unwrap().xyz_f32_le,
    )
    .unwrap();
    let config = MappingBackendComputeConfig {
        resolution: 0.1,
        initial_safe_radius_meters: 0.0,
        window_capacity: 200_000,
    };
    let mut kernel = MappingBackendSpec::Voxel.into_kernel(config.window_capacity);
    let input = MappingBackendInput {
        session_id: "floating-support".into(),
        session_generation: 1,
        backend_generation: 1,
        frame_sequence: 1,
        source_timestamp_ns: 100,
        received_at_ns: 100,
        registered_frame_id: "map".into(),
        sensor_origin_map: [0., 0., 0.65],
        dynamic_frame: None,
        registered_points_map: points,
    };
    let grid = compute_candidate_in_place_with_interval(&mut kernel, &input, config, 0)
        .unwrap()
        .unwrap()
        .latest_frame_occupancy;
    std::fs::write(
        folder.join("original-backend.bin"),
        export_mapping_kernel_checkpoint(&kernel).unwrap(),
    )
    .unwrap();
    std::fs::write(
        folder.join("original-occupancy.bin"),
        grid.data.iter().map(|&v| v as u8).collect::<Vec<_>>(),
    )
    .unwrap();
    let descriptor = serde_json::json!({"width":grid.width,"height":grid.height,"resolution":grid.resolution,"origin_x":grid.origin[0],"origin_y":grid.origin[1],"frame_id":"map","timestamp_ns":100});
    std::fs::write(
        folder.join("occupancy-descriptor.json"),
        serde_json::to_vec_pretty(&descriptor).unwrap(),
    )
    .unwrap();
    let original = std::fs::read(folder.join("original-fine.bin")).unwrap();
    let before = keyset(&CompletedMap::read(&folder.join("original-fine.bin")).unwrap());
    let mut counts = Vec::new();
    for strength in [0, 37, 73, 100] {
        let request = Request {
            fine: folder.join("original-fine.bin"),
            dense: folder.join("original-dense.bin"),
            output: folder.join(format!("strength-{strength}")),
            strength_percent: strength,
            scan_log: None,
            final_poses: None,
            source_timestamp_ns: 100,
            cancel_file: None,
            backend: Some(folder.join("original-backend.bin")),
            occupancy: Some(folder.join("original-occupancy.bin")),
            occupancy_descriptor: Some(descriptor.clone()),
        };
        let report = prepare(&request).unwrap();
        assert!(!report.offline_complete);
        assert_eq!(report.offline_frames, 0);
        let after = keyset(&CompletedMap::read(&request.output.join("fine.bin")).unwrap());
        assert!(static_keys.is_subset(&after));
        assert_eq!(
            report.preview.candidate_points,
            before.difference(&after).count()
        );
        crate::dense_cloud::validate_archive(
            &mut File::open(request.output.join("dense.bin")).unwrap(),
        )
        .unwrap();
        assert_eq!(std::fs::read(&request.fine).unwrap(), original);
        counts.push(report.preview.candidate_points);
    }
    assert_eq!(counts[0], 0);
    assert!(counts[1] > counts[0] && counts[2] > counts[1]);
    assert!(counts[3] >= counts[2]);
    std::fs::write(folder.join("strength-counts.json"),serde_json::to_vec_pretty(&serde_json::json!({"strength_percent":[0,37,73,100],"actual_removed_voxels":counts,"static_loss":0,"raw_log":"absent; floating-only, no full human replay claim"})).unwrap()).unwrap();
}
