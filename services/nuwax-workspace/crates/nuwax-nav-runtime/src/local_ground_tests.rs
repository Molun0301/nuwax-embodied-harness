//! 通过真实地图工作线程和原生检查验证旧图地面处理，不借助静态占据初始化器。
use super::*;
use nuwax_map_chunks::{
    heightfield_observations::HeightfieldColumn, ColumnGroundRecords, Domain, HeightfieldSurface,
    Repository, Writer,
};
use nuwax_nav_mls_planner::surface::{SurfaceObservation, SurfaceObservations};
use nuwax_nav_scan_native::{ScanTracker, TrackFailure, TrackResult};
use std::collections::{BTreeMap, BTreeSet};

fn source(ground: &[[f64; 3]], obstacles: &[[i32; 3]]) -> Arc<SurfaceSnapshot> {
    const Q: f32 = 0.05;
    let q = f64::from(Q);
    let mut writer = Writer::new(Repository::temporary().unwrap(), Q, 4 * 1024 * 1024).unwrap();
    let mut points = Vec::new();
    for &[x, y, height] in ground {
        let xy = [x, y].map(|v| (v / q).floor() as i32);
        let bin = (height / (q / 8.)).floor() as i32;
        let z = bin.div_euclid(8);
        let mut counts = [0; 8];
        counts[bin.rem_euclid(8) as usize] = 1;
        let column = HeightfieldColumn::from_counts(xy, Q, BTreeMap::from([(z, counts)]))
            .unwrap()
            .unwrap();
        writer.observe([[xy[0], xy[1], z]], 20).unwrap();
        writer
            .replace_ground_column(
                xy,
                &ColumnGroundRecords {
                    classified_empty: false,
                    heightfields: vec![HeightfieldSurface {
                        column,
                        samples: vec![],
                    }],
                },
            )
            .unwrap();
        points.push((x as f32, y as f32, height as f32));
    }
    writer.observe(obstacles.iter().copied(), 20).unwrap();
    points.extend(obstacles.iter().map(|p| {
        let p = p.map(|v| (v as f32 + 0.5) * Q);
        (p[0], p[1], p[2])
    }));
    let root = writer
        .snapshot(
            Domain {
                session: "n6-ground".into(),
            },
            1,
            20,
        )
        .unwrap();
    let mut observations = SurfaceObservations::new(Q, 32).unwrap();
    let mut parts = Vec::new();
    for block in &root.blocks {
        let bytes = writer
            .repository()
            .read_object(block.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
            .unwrap();
        let values = nuwax_map_chunks::decode_block(&bytes).unwrap();
        observations
            .replace_block(
                block.key,
                values
                    .iter()
                    .map(|(&local, e)| {
                        (
                            nuwax_map_chunks::global_key(block.key, local).unwrap(),
                            SurfaceObservation {
                                observed_ns: e.observed_ns,
                                hits: e.hits,
                            },
                        )
                    })
                    .collect(),
            )
            .unwrap();
        parts.push((
            block.key,
            nuwax_map_chunks::decode_ground_records(&bytes).unwrap(),
        ));
    }
    for xy in parts
        .iter()
        .map(|(k, _)| [k[0], k[1]])
        .collect::<BTreeSet<_>>()
    {
        let group: Vec<_> = parts
            .iter()
            .filter(|(k, _)| k[..2] == xy)
            .cloned()
            .collect();
        observations.replace_heightfield_columns(xy, &group);
    }
    observations.mark_complete_geometry_source();
    let (_, base) = crate::native_test_helpers::fixture();
    Arc::new(
        SurfaceSnapshot::from_points(&points, base.config().clone(), Q)
            .unwrap()
            .with_observations(Some(Arc::new(observations))),
    )
}

fn input(source: Arc<SurfaceSnapshot>) -> MapInput {
    let (config, _) = crate::native_test_helpers::fixture();
    let mut config = config.scan;
    config.resolution = f64::from(0.05_f32);
    config.body_length = 0.68;
    config.body_width = 0.328;
    config.body_center_x = 0.026;
    config.max_step_up = 0.1;
    let mut frame = (*crate::registered_frame::test_frame(1)).clone();
    // 单个远处真实注册回波只初始化概率地图，不贡献当前机身碰撞。
    frame.points_world = Arc::from([[-2., -2., -1.]]);
    frame.sensor_origin_world = [0., 0., 0.4];
    frame.body_position_world = frame.sensor_origin_world;
    MapInput {
        epoch: 1,
        source,
        config,
        boundary: Arc::new(Vec::new()),
        pose: [0., 0., 0.4],
        yaw: 0.,
        frame: Some(Arc::new(frame)),
        active: None,
        active_snapshot: None,
        phase: 0.,
    }
}

fn prepare(worker: &MapWorker, input: &MapInput) -> PreparedMap {
    worker.submit(input.clone());
    let until = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        if let Some(result) = worker.take() {
            return result.unwrap();
        }
        assert!(std::time::Instant::now() < until, "地图工作线程未交付");
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

fn check(map: &PreparedMap, position: [f64; 3]) -> Result<TrackResult, TrackFailure> {
    ScanTracker::default().track_and_check(
        &map.config,
        &map.snapshot,
        &MotionState {
            position,
            measured_velocity: 1,
            measured_yaw_rate: 1,
            ..Default::default()
        },
        [0.; 3],
        None,
    )
}

#[test]
fn n6_raised_persistent_ground_under_body_produces_trajectory() {
    let ground: Vec<_> = (-4..5)
        .flat_map(|x| {
            (-2..3).map(move |y| [(x as f64 + 0.5) * 0.05, (y as f64 + 0.5) * 0.05, 0.12])
        })
        .collect();
    let input = input(source(&ground, &[]));
    let native = crate::native_map::from_surface(&input.source).unwrap();
    assert_eq!(native.static_collision_ground.len(), ground.len());
    assert!(native.obstacles.is_empty(), "高度场成员必须走持久地面输入");
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    check(&map, input.pose).expect("旧图高12厘米的地面不能触发机身重叠");
    let route = [input.pose, [2., 0., 0.4]];
    let curve = ScanPlanner::default()
        .plan(
            &map.config,
            &map.snapshot,
            &route,
            &MotionState {
                position: input.pose,
                measured_velocity: 1,
                measured_yaw_rate: 1,
                ..Default::default()
            },
            None,
            0.,
        )
        .expect("过滤后必须能产出真实原生轨迹");
    assert!(!curve.samples(0., 0.05).is_empty());
}

#[test]
fn n6_persistent_solid_under_body_keeps_overlap_reason_32() {
    let input = input(source(&[[0.025, 0.025, 0.12]], &[[0, 0, 6]]));
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    let error = check(&map, input.pose).expect_err("持久实体不能随旧地面一起删除");
    assert_eq!(error.reason, 32);
    assert!(
        error.message.contains("BODY_OVERLAP_RECOVERY_FAILED"),
        "{error:?}"
    );
}

#[test]
fn n6_real_step_above_max_step_up_still_blocks() {
    let input = input(source(&[[0.625, 0.025, 0.18]], &[]));
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    check(&map, input.pose).expect("台阶在当前机身外");
    let error = check(&map, [0.625, 0., 0.4]).expect_err("前方真实超高台阶必须保留碰撞");
    assert_eq!(error.reason, 32);
}

#[test]
fn n6_recent_ground_overrides_persistent_column_in_both_height_directions() {
    for (old, recent, blocked) in [(0.18, 0., false), (0., 0.18, true)] {
        let base = source(&[[0.625, 0.025, old]], &[]);
        let observed = base
            .with_recent_obstacle_points(1, 1, 0, &[0], [(0, [0.625, 0.025, recent])])
            .unwrap();
        let input = input(Arc::new(observed));
        let map = prepare(&MapWorker::new(None).unwrap(), &input);
        let result = check(&map, [0.625, 0., 0.4]);
        assert_eq!(
            result.is_err(),
            blocked,
            "机身外同列也必须以近期地面判断：旧={old}，近期={recent}，结果={result:?}"
        );
    }
}

#[test]
fn n6_ground_filter_refreshes_for_yaw_height_and_subcell_translation() {
    let worker = MapWorker::new(None).unwrap();
    let mut input = input(source(&[[0.025, 0.225, 0.12]], &[]));
    let probe = [0., 0.225, 0.4];
    let first = prepare(&worker, &input);
    assert_eq!(
        check(&first, probe).unwrap_err().reason,
        32,
        "当前机身未覆盖的高地面保留"
    );
    input.yaw = std::f64::consts::FRAC_PI_2;
    let rotated = prepare(&worker, &input);
    check(&rotated, probe).expect("原地转向覆盖该格后，应刷新过时地面判定");
    input.pose[2] += 0.1;
    let raised = prepare(&worker, &input);
    assert_eq!(
        check(&raised, probe).unwrap_err().reason,
        32,
        "当前支撑高度改变后，该地面已不超容差，应重新保留"
    );
    assert_eq!(
        check(&first, probe).unwrap_err().reason,
        32,
        "刷新不能改变已经发布的不可变快照"
    );

    let worker = MapWorker::new(None).unwrap();
    let mut input = self::input(source(&[[0.025, 0.175, 0.12]], &[]));
    let covered = prepare(&worker, &input);
    check(&covered, [0., 0.175, 0.4]).unwrap();
    input.pose[1] = -0.04;
    let uncovered = prepare(&worker, &input);
    assert_eq!(check(&uncovered, [0., 0.175, 0.4]).unwrap_err().reason, 32);
    // 同一细格内移动到覆盖边缘，不能继续复用上一个覆盖范围。
    input.pose[1] = -0.001;
    let covered_again = prepare(&worker, &input);
    check(&covered_again, [0., 0.175, 0.4]).unwrap();
}

#[test]
fn n6_ground_filter_keeps_other_surface_above_current_floor() {
    let (_, base) = crate::native_test_helpers::fixture();
    // 旧全点云把地面与独立薄顶面都输出为表面；上层表面不能随当前地面被剔除。
    let raw = SurfaceSnapshot::from_points(
        &[(0.025, 0.025, 0.12), (0.025, 0.025, 0.4)],
        base.config().clone(),
        0.05,
    )
    .unwrap();
    let input = input(Arc::new(raw));
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    assert_eq!(check(&map, input.pose).unwrap_err().reason, 32);
}
