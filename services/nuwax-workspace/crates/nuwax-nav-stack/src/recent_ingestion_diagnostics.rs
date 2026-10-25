//! Baseline-only recent-source attribution. No transport or physical motion.
use super::*;
use nuwax_codec::lcm::{lio, recent_cloud};

fn fixture() -> (
    SurfaceNavigationConfig,
    GuardedMapAuthority,
    Arc<SurfaceSnapshot>,
) {
    let config =
        SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.2, 35.0, 0.2).unwrap();
    let terrain: Vec<_> = (0..20)
        .flat_map(|x| (0..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05)))
        .collect();
    let model = nuwax_nav_mls_planner::SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.15, -0.15], [0.15, -0.15], [0.15, 0.15], [-0.15, 0.15]],
        z_bounds_m: [-0.2, 0.2],
        body_to_support_z_m: 0.4,
        max_step_up_m: 0.2,
        max_step_down_m: 0.2,
        max_cross_slope_deg: 35.0,
        support_uncertainty_m: 0.01,
    };
    let map = Arc::new(
        SurfaceSnapshot::from_points(&terrain, config.geometry.clone(), 0.1)
            .unwrap()
            .with_body_envelope(model)
            .unwrap(),
    );
    let authority = GuardedMapAuthority {
        committed_map: schema::CommittedMapIdentity {
            session_id: "recent-ingestion-diagnostic".into(),
            session_generation: 1,
            revision: 1,
            commit_boot_nonce: [7; 16],
            commit_sequence: 1,
            source_timestamp_ns: 1,
            frame_id: "map".into(),
        },
        producer_runtime_generation: 17,
        map_frame_generation: 1,
        cloud_revision: 1,
        cloud_occupancy_revision: 1,
        cloud_commit_sequence: 1,
        cloud_source_timestamp_ns: 1,
        commit_boot_nonce: [7; 16],
    };
    (config, authority, map)
}

type SchedulerTask = tokio::task::JoinHandle<Result<(), nuwax_nav_runtime::RuntimeError>>;
fn scheduler(
    config: &SurfaceNavigationConfig,
) -> (
    nuwax_nav_runtime::NavSchedulerHandle,
    SchedulerTask,
    Arc<Mutex<Vec<String>>>,
) {
    let calls = Arc::new(Mutex::new(Vec::new()));
    let planner = nuwax_nav_runtime::SurfaceFollower::new(config.follower.clone()).unwrap();
    let scheduler = nuwax_nav_runtime::NavScheduler::new(
        crate::lcm_dataflow::guarded_tests::RecordingMotion(calls.clone()),
        planner,
        nuwax_nav_runtime::SchedulerConfig::default(),
    )
    .unwrap();
    let (handle, task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
    (handle, task, calls)
}

fn packet(
    points: &[[f64; 3]],
    sequence: i64,
    mixed: bool,
    big_endian: bool,
) -> recent_cloud::RecentCloud3D {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64;
    let fields = if mixed {
        // Reordered fields, mixed scalar widths and extra per-point bytes are legal.
        vec![("z", 16, 8), ("intensity", 12, 7), ("x", 0, 8), ("y", 8, 7)]
    } else {
        vec![("x", 0, 7), ("y", 4, 7), ("z", 8, 7)]
    };
    let step = if mixed { 32 } else { 12 };
    let mut data = vec![0_u8; points.len() * step];
    for (index, p) in points.iter().enumerate() {
        for (axis, offset, wide) in if mixed {
            [(0, 0, true), (1, 8, false), (2, 16, true)]
        } else {
            [(0, 0, false), (1, 4, false), (2, 8, false)]
        } {
            let start = index * step + offset;
            if wide {
                let bytes = if big_endian {
                    p[axis].to_be_bytes()
                } else {
                    p[axis].to_le_bytes()
                };
                data[start..start + 8].copy_from_slice(&bytes);
            } else {
                let bytes = if big_endian {
                    (p[axis] as f32).to_be_bytes()
                } else {
                    (p[axis] as f32).to_le_bytes()
                };
                data[start..start + 4].copy_from_slice(&bytes);
            }
        }
    }
    recent_cloud::RecentCloud3D {
        registration:None,
        producer_runtime_generation: 17,
        session_id: "recent-ingestion-diagnostic".into(),
        session_generation: 1,
        map_frame_generation: 1,
        commit_boot_nonce: [7; 16],
        ground_point_indices: (0..points.len())
            .filter(|i| i % 4 == 0)
            .map(|i| i as u32)
            .collect(),
        cloud: Some(lio::LioCloudEnvelope {
            schema_version: 1,
            backend: lio::LIO_BACKEND_DEVICE,
            runtime_generation: 2,
            source_sequence: sequence,
            source_timestamp_ns: now,
            correlated_odometry_source_sequence: sequence,
            correlated_odometry_source_timestamp_ns: now,
            sensor_frame_id: "sensor".into(),
            body_frame_id: "body".into(),
            odom_frame_id: "odom".into(),
            map_frame_id: "map".into(),
            registration_state: lio::LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count: points.len() as i64,
            emitted_point_count: points.len() as i64,
            cloud: schema::PointCloud2 {
                header: schema::Header {
                    seq: sequence as i32,
                    stamp_sec: (now / 1_000_000_000) as i32,
                    stamp_nsec: (now % 1_000_000_000) as i32,
                    frame_id: "map".into(),
                },
                height: 1,
                width: points.len() as i32,
                fields: fields
                    .into_iter()
                    .map(|(name, offset, datatype)| schema::PointField {
                        name: name.into(),
                        offset,
                        datatype,
                        count: 1,
                    })
                    .collect(),
                is_bigendian: big_endian,
                point_step: step as i32,
                row_step: (points.len() * step) as i32,
                data,
                is_dense: true,
            },
        }),
    }
}

fn hints(map: &SurfaceSnapshot) -> Vec<([f64; 3], [f64; 3])> {
    map.recent_obstacle_hints(
        [[-1.0, -1.0], [40.0, 40.0]],
        ([1.0, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0]),
    )
    .unwrap()
}

async fn reject_unchanged(
    state: &mut RecentSourceState,
    wire: &[u8],
    authority: &GuardedMapAuthority,
    handle: &nuwax_nav_runtime::NavSchedulerHandle,
    expected: &str,
) {
    let before = state.snapshot.as_ref().unwrap().clone();
    let watermark = state.watermark;
    let enabled = state.enabled;
    let error = state
        .consume(wire, Some(authority), handle)
        .await
        .unwrap_err();
    assert!(
        error.contains(expected),
        "expected {expected:?}, got {error:?}"
    );
    assert!(Arc::ptr_eq(&before, state.snapshot.as_ref().unwrap()));
    assert_eq!(state.watermark, watermark);
    assert_eq!(state.enabled, enabled);
}

#[tokio::test]
async fn baseline_layouts_and_original_member_indices() {
    let (config, authority, terrain) = fixture();
    let (handle, task, calls) = scheduler(&config);
    let points = [[0.51, 0.51, 0.55], [0.52, 0.52, 0.56], [0.75, 0.75, 0.55]];
    let mut reference = None;
    for (index, (mixed, big)) in [(false, false), (false, true), (true, false), (true, true)]
        .into_iter()
        .enumerate()
    {
        let mut state = RecentSourceState::new(None);
        state.attach_terrain(terrain.clone());
        let mut p = packet(&points, index as i64 + 1, mixed, big);
        p.ground_point_indices = vec![0, 2];
        if mixed {
            let cloud = &mut p.cloud.as_mut().unwrap().cloud;
            cloud.height = 3;
            cloud.width = 1;
            cloud.row_step = cloud.point_step;
        }
        // Ground point 0 shares its cell with obstacle 1, so the cell stays a volume.
        let wire = recent_cloud::encode_body(&p).unwrap();
        let decoded = recent_cloud::decode_body(&wire).unwrap();
        let cloud = point_cloud_from_schema(decoded.cloud.unwrap().cloud).unwrap();
        assert_eq!(cloud.points.len(), points.len());
        state
            .consume(&wire, Some(&authority), &handle)
            .await
            .unwrap();
        let seen = hints(state.snapshot.as_ref().unwrap());
        assert_eq!(seen.len(), 2);
        assert!(
            seen.iter().any(|(a, b)| a != b),
            "mixed ground/obstacle cell stays occupied"
        );
        assert!(
            seen.iter().any(|(a, b)| a == b),
            "separate ground return preserves measured point"
        );
        // f32 and f64 represent these coordinates differently; compare each result to
        // the same decoded scalars fed directly through the original builder.
        let expected = terrain
            .with_recent_obstacle_points(
                index as u64 + 1,
                p.cloud.as_ref().unwrap().source_timestamp_ns as u64,
                0,
                &[0, 2],
                cloud
                    .points
                    .iter()
                    .enumerate()
                    .map(|(i, p)| (i as u32, [p.x, p.y, p.z])),
            )
            .unwrap();
        assert_eq!(seen, hints(&expected));
        if !mixed {
            if let Some(old) = &reference {
                assert_eq!(&seen, old);
            } else {
                reference = Some(seen);
            }
        }
    }
    assert!(calls.lock().unwrap().is_empty());
    handle.shutdown().await.unwrap();
    task.await.unwrap().unwrap();
}

#[tokio::test]
async fn baseline_rejection_precedence_and_atomic_state() {
    let (config, authority, terrain) = fixture();
    let (handle, task, calls) = scheduler(&config);
    let mut state = RecentSourceState::new(None);
    state.attach_terrain(terrain);
    let base = packet(&[[0.5, 0.5, 0.55]], 1, false, false);
    let accepted = recent_cloud::encode_body(&base).unwrap();
    state
        .consume(&accepted, Some(&authority), &handle)
        .await
        .unwrap();
    reject_unchanged(&mut state, &accepted, &authority, &handle, "replay").await;

    // 当前发送边界也验证XYZ，因此先编码合法包，再模拟线上相同长度载荷损坏。
    fn corrupt_cloud(packet: &recent_cloud::RecentCloud3D, change: impl FnOnce(&mut schema::PointCloud2)) -> Vec<u8> {
        let mut wire = recent_cloud::encode_body(packet).unwrap();
        let mut cloud = packet.cloud.as_ref().unwrap().cloud.clone();
        let old = schema::encode_point_cloud2_body(&cloud).unwrap();
        assert!(wire.ends_with(&old));
        change(&mut cloud);
        let altered = schema::encode_point_cloud2_body(&cloud).unwrap();
        assert_eq!(altered.len(), old.len(), "仅替换载荷，保留完整包长度");
        let offset = wire.len() - old.len();
        wire[offset..].copy_from_slice(&altered);
        wire
    }
    let p = packet(&[[0.5, 0.5, 0.55]], 2, false, false);
    let invalid_type_and_axis = corrupt_cloud(&p, |cloud| {
        cloud.fields[0].datatype = 5;
        cloud.fields[2].name = "q".into();
    });
    reject_unchanged(&mut state, &invalid_type_and_axis, &authority, &handle,
        "XYZ fields must be unique scalar floats").await;
    let missing_axis = corrupt_cloud(&p, |cloud| cloud.fields[2].name = "q".into());
    reject_unchanged(&mut state, &missing_axis, &authority, &handle,
        "must declare XYZ coordinates").await;

    for points in [
        vec![[1e12, 0.5, 0.55], [0.5, f64::NAN, 0.55]],
        vec![[0.5, 0.5, f64::INFINITY]],
        vec![[f64::NAN, 0.5, 0.55]],
    ] {
        let damaged = packet(&points, 3, true, false);
        let finite: Vec<_> = points.iter().map(|p| p.map(|v| if v.is_finite() { v } else { 0. })).collect();
        let valid = packet(&finite, 3, true, false);
        let wire = corrupt_cloud(&valid, |cloud| cloud.data = damaged.cloud.as_ref().unwrap().cloud.data.clone());
        reject_unchanged(&mut state, &wire, &authority, &handle,
            "world coordinates must be finite").await;
    }
    let p = packet(&[[1e12, 0.5, 0.55]], 4, true, false);
    reject_unchanged(
        &mut state,
        &recent_cloud::encode_body(&p).unwrap(),
        &authority,
        &handle,
        "voxel range",
    )
    .await;

    let p = packet(&[[0.5, 0.5, 0.55]], 5, false, false);
    let good = recent_cloud::encode_body(&p).unwrap();
    reject_unchanged(&mut state, &good[..good.len() - 1], &authority, &handle, "").await;
    let cloud_body = schema::encode_point_cloud2_body(&p.cloud.as_ref().unwrap().cloud).unwrap();
    assert!(good.ends_with(&cloud_body));
    let cloud_start = good.len() - cloud_body.len();
    let mut wrong_count = good.clone();
    wrong_count[cloud_start - 16..cloud_start - 8].copy_from_slice(&2_i64.to_be_bytes());
    wrong_count[cloud_start - 8..cloud_start].copy_from_slice(&2_i64.to_be_bytes());
    reject_unchanged(
        &mut state,
        &wrong_count,
        &authority,
        &handle,
        "emitted_point_count",
    )
    .await;
    // This generated schema puts data length at the start. Its tail is row_step,
    // data bytes and is_dense; no guessed conventional ROS wire layout here.
    let row_offset = good.len() - 1 - p.cloud.as_ref().unwrap().cloud.data.len() - 4;
    assert_eq!(&good[row_offset..row_offset + 4], &12_i32.to_be_bytes());
    let mut padded_row = good.clone();
    padded_row[row_offset..row_offset + 4].copy_from_slice(&16_i32.to_be_bytes());
    reject_unchanged(&mut state, &padded_row, &authority, &handle, "row_step").await;
    let mut invalid_cloud = p.cloud.as_ref().unwrap().cloud.clone();
    invalid_cloud.fields[0].offset = 12;
    let invalid_body = schema::encode_point_cloud2_body(&invalid_cloud).unwrap();
    assert_eq!(invalid_body.len(), cloud_body.len());
    let mut bad_field = good.clone();
    bad_field[cloud_start..].copy_from_slice(&invalid_body);
    reject_unchanged(
        &mut state,
        &bad_field,
        &authority,
        &handle,
        "layout exceeds",
    )
    .await;
    for change in 0..3 {
        let mut p = packet(&[[0.5, 0.5, 0.55]], 6, false, false);
        match change {
            0 => p.commit_boot_nonce = [8; 16],
            1 => {
                let c = p.cloud.as_mut().unwrap();
                c.map_frame_id = "other".into();
                c.cloud.header.frame_id = "other".into();
            }
            _ => p.cloud.as_mut().unwrap().runtime_generation = 1,
        }
        let expected = match change {
            0 => "different navigation domain",
            1 => "frame or source time",
            _ => "replay",
        };
        reject_unchanged(
            &mut state,
            &recent_cloud::encode_body(&p).unwrap(),
            &authority,
            &handle,
            expected,
        )
        .await;
    }
    assert!(calls.lock().unwrap().is_empty());
    handle.shutdown().await.unwrap();
    task.await.unwrap().unwrap();
}

#[tokio::test]
async fn baseline_retention_and_generation_reset() {
    let (config, authority, terrain) = fixture();
    let (handle, task, calls) = scheduler(&config);
    for retain in [false, true] {
        let mut state = RecentSourceState::new(retain.then_some(Duration::from_secs(5)));
        state.attach_terrain(terrain.clone());
        for sequence in 1..=9 {
            let mut p = packet(&[[0.5, 0.5, 0.55]], sequence, false, false);
            p.ground_point_indices.clear();
            state
                .consume(
                    &recent_cloud::encode_body(&p).unwrap(),
                    Some(&authority),
                    &handle,
                )
                .await
                .unwrap();
            assert_eq!(
                hints(state.snapshot.as_ref().unwrap()).len(),
                if retain {
                    (sequence as usize).min(8)
                } else {
                    1
                }
            );
        }
        let mut p = packet(&[[0.5, 0.5, 0.55]], 1, false, false);
        p.cloud.as_mut().unwrap().runtime_generation = 3;
        p.ground_point_indices.clear();
        state
            .consume(
                &recent_cloud::encode_body(&p).unwrap(),
                Some(&authority),
                &handle,
            )
            .await
            .unwrap();
        assert_eq!(hints(state.snapshot.as_ref().unwrap()).len(), 1);
    }
    assert!(calls.lock().unwrap().is_empty());
    handle.shutdown().await.unwrap();
    task.await.unwrap().unwrap();
}

fn micros(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1e6
}

// Attribution is a separate execution, not instrumentation inside consume and
// not something to subtract from that call. Each stage below is nonoverlapping.
fn staged(
    wire: &[u8],
    previous: &SurfaceSnapshot,
    retain: bool,
) -> (Arc<SurfaceSnapshot>, [f64; 4]) {
    let start = Instant::now();
    let packet = recent_cloud::decode_body(wire).unwrap();
    let decode = micros(start);
    let cloud = packet.cloud.unwrap();
    let sequence = cloud.source_sequence as u64;
    let timestamp = cloud.source_timestamp_ns as u64;
    let count = cloud.emitted_point_count as usize;
    let start = Instant::now();
    let points = point_cloud_from_schema(cloud.cloud).unwrap();
    let xyz = micros(start);
    let start = Instant::now();
    assert_eq!(points.points.len(), count);
    assert!(points
        .points
        .iter()
        .all(|p| [p.x, p.y, p.z].into_iter().all(f64::is_finite)));
    let finite = micros(start);
    let start = Instant::now();
    let snapshot = Arc::new(
        previous
            .with_recent_obstacle_points(
                sequence,
                timestamp,
                if retain { 5_000_000_000 } else { 0 },
                &packet.ground_point_indices,
                points
                    .points
                    .into_iter()
                    .enumerate()
                    .map(|(i, p)| (i as u32, [p.x, p.y, p.z])),
            )
            .unwrap(),
    );
    let build = micros(start);
    (snapshot, [decode, xyz, finite, build])
}

#[tokio::test]
#[ignore = "bounded release baseline recent-ingestion attribution; coordinate exclusive CPU lane"]
async fn baseline_complete_recent_ingestion_cost() {
    let (config, authority, terrain) = fixture();
    let mut results = Vec::new();
    for count in [1_000, 10_000, 100_000] {
        for dispersed in [false, true] {
            for retain in [false, true] {
                let (handle, task, calls) = scheduler(&config);
                let mut state = RecentSourceState::new(retain.then_some(Duration::from_secs(5)));
                state.attach_terrain(terrain.clone());
                let mut stage_map = terrain.clone();
                let points: Vec<_> = (0..count)
                    .map(|i| {
                        let j = if dispersed { i } else { i % 64 };
                        [
                            0.05 + (j % 320) as f64 * 0.1,
                            0.05 + (j / 320) as f64 * 0.1,
                            if i % 4 == 0 { 0.05 } else { 0.55 },
                        ]
                    })
                    .collect();
                let mut full = Vec::new();
                let mut stages = Vec::new();
                let mut first_time = 0;
                for sequence in 1..=15 {
                    let p = packet(&points, sequence, false, false);
                    let source = p.cloud.as_ref().unwrap().source_timestamp_ns;
                    if sequence == 1 {
                        first_time = source;
                    }
                    // 8 warm publications establish the original bounded retention window.
                    assert!(
                        source - first_time < 5_000_000_000,
                        "cohort exceeded retained-source window"
                    );
                    let wire = recent_cloud::encode_body(&p).unwrap();
                    let (elapsed, stage) = if sequence % 2 == 0 {
                        let (next, stage) = staged(&wire, &stage_map, retain);
                        stage_map = next;
                        let start = Instant::now();
                        state
                            .consume(&wire, Some(&authority), &handle)
                            .await
                            .unwrap();
                        (micros(start), stage)
                    } else {
                        let start = Instant::now();
                        state
                            .consume(&wire, Some(&authority), &handle)
                            .await
                            .unwrap();
                        let elapsed = micros(start);
                        let (next, stage) = staged(&wire, &stage_map, retain);
                        stage_map = next;
                        (elapsed, stage)
                    };
                    assert_eq!(state.watermark, Some((2, sequence, source)));
                    assert!(nuwax_nav_runtime::native_map::same_source_geometry(
                        &terrain,
                        state.snapshot.as_ref().unwrap()
                    ));
                    if sequence > 8 {
                        full.push(elapsed);
                        stages.push(stage);
                    }
                }
                assert!(
                    calls.lock().unwrap().is_empty(),
                    "diagnostic must not request motion"
                );
                results.push(serde_json::json!({"points":count,"dispersed":dispersed,"retained_frame_target":if retain {8} else {1},
            "xyz_temporary_payload_bytes":count*std::mem::size_of::<Point3>(),
            "full_consume_us":full,"separate_stage_order":["codec_decode","xyz_materialize","finite_scan","recent_frame_build"],
            "separate_stage_us":stages}));
                handle.shutdown().await.unwrap();
                task.await.unwrap().unwrap();
            }
        }
    }
    println!(
        "RECENT_INGESTION_BASELINE {}",
        serde_json::json!({"warmups_per_cohort":8,"samples_per_cohort":7,
        "full_includes_scheduler_ack":true,"stages_are_separate_executions":true,"results":results})
    );
}
