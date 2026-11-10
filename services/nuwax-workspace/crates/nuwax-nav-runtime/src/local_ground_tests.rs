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
    let mut columns = BTreeMap::<[i32; 2], Vec<HeightfieldSurface>>::new();
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
        columns.entry(xy).or_default().push(HeightfieldSurface {
            column,
            samples: vec![],
        });
        points.push((x as f32, y as f32, height as f32));
    }
    for (xy, heightfields) in columns {
        writer
            .replace_ground_column(
                xy,
                &ColumnGroundRecords {
                    classified_empty: false,
                    heightfields,
                },
            )
            .unwrap();
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
    let mut input = input(source(&ground, &[]));
    // 新规则只剔除可跨高度内的旧地面；12 厘米应配套大于它的可跨高度。
    input.config.max_step_up = 0.2;
    let native = crate::native_map::from_surface(&input.source).unwrap();
    assert_eq!(native.static_collision_ground.len(), ground.len());
    assert!(native.obstacles.is_empty(), "高度场成员必须走持久地面输入");
    assert!(crate::native_map::collision_ground(
        native.static_collision_ground,
        vec![],
        &input.config,
        input.pose,
        input.yaw,
    )
    .is_empty());
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
    let mut input = input(source(&[[0.025, 0.025, 0.12]], &[[0, 0, 6]]));
    input.config.max_step_up = 0.2;
    let native = crate::native_map::from_surface(&input.source).unwrap();
    assert!(!native.obstacles.is_empty(), "实体必须独立存在于非地面输入");
    assert!(
        crate::native_map::collision_ground(
            native.static_collision_ground,
            vec![],
            &input.config,
            input.pose,
            input.yaw,
        )
        .is_empty(),
        "须在同列旧地面已被剔除后验证实体阻挡"
    );
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
    for step in [0.1, f64::from(0.15_f32)] {
        for x in [0.025, 0.625] {
            let mut input = input(source(&[[x, 0.025, 0.18]], &[]));
            input.config.max_step_up = step;
            let map = prepare(&MapWorker::new(None).unwrap(), &input);
            if x > 0.5 {
                check(&map, input.pose).expect("台阶在当前机身外");
            }
            let error = check(&map, [x, 0., 0.4]).expect_err("机身下与前方超高台阶均须阻挡");
            assert_eq!(error.reason, 32);
        }
    }
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
    let mut input = input(source(&[[0.025, 0.325, 0.12]], &[]));
    let height = crate::native_map::from_surface(&input.source)
        .unwrap()
        .ground[0][2];
    input.config.max_step_up = height;
    let probe = [0., 0.325, 0.4];
    let first = prepare(&worker, &input);
    assert_eq!(
        check(&first, probe).unwrap_err().reason,
        32,
        "当前机身外扩后仍未覆盖的高地面保留"
    );
    input.yaw = std::f64::consts::FRAC_PI_2;
    let rotated = prepare(&worker, &input);
    check(&rotated, probe).expect("原地转向覆盖该格后，应刷新过时地面判定");
    input.pose[2] += 0.2;
    let raised = prepare(&worker, &input);
    assert_eq!(
        check(&raised, probe).unwrap_err().reason,
        32,
        "当前支撑高于该地面后，应重新保留"
    );
    assert_eq!(
        check(&first, probe).unwrap_err().reason,
        32,
        "刷新不能改变已经发布的不可变快照"
    );

    let worker = MapWorker::new(None).unwrap();
    let mut input = self::input(source(&[[0.025, 0.225, 0.12]], &[]));
    input.config.max_step_up = height;
    input.config.max_velocity = 0.;
    input.config.max_yaw_rate = 0.;
    let covered = prepare(&worker, &input);
    check(&covered, [0., 0.225, 0.4]).unwrap();
    input.pose[1] = -0.04;
    let uncovered = prepare(&worker, &input);
    assert_eq!(check(&uncovered, [0., 0.225, 0.4]).unwrap_err().reason, 32);
    // 同一细格内移动到覆盖边缘，不能继续复用上一个覆盖范围。
    input.pose[1] = -0.001;
    let covered_again = prepare(&worker, &input);
    check(&covered_again, [0., 0.225, 0.4]).unwrap();
}

#[test]
fn n6_multilayer_ground_eight_cm_apart_produces_trajectory() {
    let q = f64::from(0.05_f32);
    // 与 N6-10 的三峰间隔相同：两个独立高度场记录相差 8.125 厘米。
    let ground = [[0.025, 0.025, q * 11.5 / 8.], [0.025, 0.025, q * 24.5 / 8.]];
    let mut input = input(source(&ground, &[]));
    let native = crate::native_map::from_surface(&input.source).unwrap();
    assert_eq!(native.static_collision_ground.len(), 2);
    assert!(native.obstacles.is_empty());
    assert!((native.ground[1][2] - native.ground[0][2] - 0.08125).abs() < 1e-8);
    // 正好处于可跨高度上界，原生表面碰撞带也包含这个高度，能区分修前修后。
    input.config.max_step_up = native.ground[1][2];
    assert!(
        crate::native_map::collision_ground(
            native.static_collision_ground,
            vec![],
            &input.config,
            input.pose,
            input.yaw,
        )
        .is_empty(),
        "不能按最近层的五厘米关联范围保留上层旧地面"
    );
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    check(&map, input.pose).expect("同列两层可跨旧地面不能报原因32");
    let curve = ScanPlanner::default()
        .plan(
            &map.config,
            &map.snapshot,
            &[input.pose, [2., 0., 0.4]],
            &MotionState {
                position: input.pose,
                measured_velocity: 1,
                measured_yaw_rate: 1,
                ..Default::default()
            },
            None,
            0.,
        )
        .expect("剔除后必须产出原生轨迹");
    assert!(!curve.samples(0., 0.05).is_empty());
}

#[test]
fn n6_archived_column_ground_is_removed_and_produces_trajectory() {
    // N6-10 历史位姿持久层重算.json 与原生表面复算.cpp 的原值，不改高度或可跨上界。
    let position = [
        3.0432525477509524,
        -0.3996377514741919,
        0.006867227014326419,
    ];
    let xy = [2.8750000428408384, -0.7250000108033419];
    let ground = [
        -0.3281250048894435,
        -0.26562500395812094,
        -0.24687500367872417,
    ]
    .map(|z| [xy[0], xy[1], z]);
    // 同时覆盖归档完整三层及用户指定的最低层/高层两条真实记录。
    for ground in [ground.to_vec(), vec![ground[0], ground[2]]] {
        let mut input = input(source(&ground, &[]));
        input.pose = position;
        input.yaw = 0.6344043223712519;
        input.config.body_length = 0.6789949835614777;
        input.config.body_width = 0.32800000000271845;
        input.config.body_center_x = 0.026072508220619156;
        input.config.body_center_y = 1.2490009027033011e-15;
        input.config.body_to_ground = 0.40456216361899383;
        input.config.max_step_up = 0.15000000596046448;
        input.config.body_z_max = 0.01467999999999997;
        assert_eq!(
            position[2] - input.config.body_to_ground,
            -0.3976949366046674
        );
        let native = crate::native_map::from_surface(&input.source).unwrap();
        assert_eq!(
            native.static_collision_ground, ground,
            "高度场须保留真实归档数值"
        );
        assert!(native.obstacles.is_empty());
        assert!(native.ground.iter().all(|p| {
            [p[0], p[1]].map(|v| (v / input.config.resolution).floor() as i32) == [57, -15]
        }));
        let mut frame = (**input.frame.as_ref().unwrap()).clone();
        frame.sensor_origin_world = position;
        frame.body_position_world = position;
        frame.points_world = Arc::from([[position[0] - 2., position[1] - 2., position[2] - 1.4]]);
        input.frame = Some(Arc::new(frame));
        let map = prepare(&MapWorker::new(None).unwrap(), &input);
        let state = MotionState {
            position,
            yaw: input.yaw,
            measured_velocity: 1,
            measured_yaw_rate: 1,
            ..Default::default()
        };
        let tracked = ScanTracker::default()
            .track_and_check(&map.config, &map.snapshot, &state, [0.; 3], None)
            .expect("真实格[57,-15]不能再触发原因32");
        assert!(
            tracked.recovery_samples.is_empty(),
            "真实高层应被剔除，无需浅脱困"
        );
        assert!(crate::native_map::collision_ground(
            native.static_collision_ground,
            vec![],
            &input.config,
            input.pose,
            input.yaw,
        )
        .is_empty());
        let goal = [
            position[0] + 2. * input.yaw.cos(),
            position[1] + 2. * input.yaw.sin(),
            position[2],
        ];
        let curve = ScanPlanner::default()
            .plan(
                &map.config,
                &map.snapshot,
                &[position, goal],
                &state,
                None,
                0.,
            )
            .expect("真实归档地面过滤后必须产出原生轨迹");
        assert!(!curve.samples(0., input.config.sample_dt).is_empty());
    }
}

#[test]
fn n6_ground_filter_support_and_column_minimum_bounds() {
    let mut input = input(source(&[], &[]));
    input.config.max_step_up = f64::from(0.15_f32);
    // 最低层无更低地面且低于支撑，保留；等于支撑的非最低层由条件二剔除。
    let heights = [
        -0.01,
        0.,
        0.025,
        input.config.max_step_up,
        input.config.max_step_up + 0.001,
    ];
    let ground = heights.map(|z| [0.025, 0.025, z]);
    assert_eq!(
        crate::native_map::collision_ground(
            ground.to_vec(),
            vec![],
            &input.config,
            input.pose,
            input.yaw,
        ),
        vec![ground[0], ground[4]]
    );
}

#[test]
fn n6_column_minimum_filter_is_limited_to_expanded_body() {
    let q = f64::from(0.05_f32);
    let ground = [[0.625, 0.025, q * 11.5 / 8.], [0.625, 0.025, q * 24.5 / 8.]];
    let mut input = input(source(&ground, &[]));
    input.config.max_step_up = f64::from(0.15_f32);
    let native = crate::native_map::from_surface(&input.source).unwrap();
    assert_eq!(
        crate::native_map::collision_ground(
            native.static_collision_ground.clone(),
            vec![],
            &input.config,
            input.pose,
            input.yaw,
        ),
        native.static_collision_ground,
        "外扩机身之外，同列多层地面仍保留"
    );
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    assert_eq!(check(&map, [0.625, 0., 0.4]).unwrap_err().reason, 32);
}

#[test]
fn n6_ground_margin_covers_async_translation_and_rotation() {
    // 这些列在快照位姿的机身外，一个更新周期后会被平移或转动的机身覆盖。
    for (point, velocity, yaw_rate, period, position, yaw) in [
        ([0.025, 0.225, 0.12], 1., 0., 0.1, [0., 0.1, 0.4], 0.),
        ([0.325, 0.225, 0.12], 0., 2., 0.1, [0., 0., 0.4], 0.2),
    ] {
        let mut input = input(source(&[point], &[]));
        input.config.max_step_up = crate::native_map::from_surface(&input.source)
            .unwrap()
            .ground[0][2];
        input.config.max_velocity = velocity;
        input.config.max_lateral_velocity = velocity;
        input.config.max_yaw_rate = yaw_rate;
        input.config.sample_dt = period;
        let map = prepare(&MapWorker::new(None).unwrap(), &input);
        let result = ScanTracker::default().track_and_check(
            &map.config,
            &map.snapshot,
            &MotionState {
                position,
                yaw,
                measured_velocity: 1,
                measured_yaw_rate: 1,
                ..Default::default()
            },
            [0.; 3],
            None,
        );
        result.expect("异步快照外扩须覆盖一周期内的平移与转动");
    }
}

#[test]
fn n6_ground_margin_refreshes_when_motion_or_period_changes() {
    for change_period in [false, true] {
        let worker = MapWorker::new(None).unwrap();
        let mut input = input(source(&[[0.025, 0.325, 0.12]], &[]));
        input.config.max_step_up = crate::native_map::from_surface(&input.source)
            .unwrap()
            .ground[0][2];
        let first = prepare(&worker, &input);
        let probe = [0., 0.325, 0.4];
        assert_eq!(check(&first, probe).unwrap_err().reason, 32);
        if change_period {
            input.config.sample_dt = 0.3;
        } else {
            input.config.max_velocity = 3.;
        }
        let expanded = prepare(&worker, &input);
        check(&expanded, probe).expect("速度或更新周期改变须重建过滤范围");
        assert_eq!(check(&first, probe).unwrap_err().reason, 32, "旧快照不可变");
    }
}

#[test]
fn n6_ground_filter_keeps_other_surface_above_current_floor() {
    let (_, base) = crate::native_test_helpers::fixture();
    // 旧全点云把地面与独立薄顶面都输出为表面；超可跨高度的上层表面须保留。
    let raw = SurfaceSnapshot::from_points(
        &[(0.025, 0.025, 0.12), (0.025, 0.025, 0.4)],
        base.config().clone(),
        0.05,
    )
    .unwrap();
    let mut input = input(Arc::new(raw));
    input.config.max_step_up = 0.2;
    let native = crate::native_map::from_surface(&input.source).unwrap();
    let retained = crate::native_map::collision_ground(
        native.static_collision_ground,
        vec![],
        &input.config,
        input.pose,
        input.yaw,
    );
    assert_eq!(retained.len(), 1, "下层旧地面剔除，只保留超高薄表面");
    assert!(retained[0][2] > input.config.max_step_up);
    let map = prepare(&MapWorker::new(None).unwrap(), &input);
    // 仅朝向入口不启动脱困，直接证明原输入中的上层表面仍被原生碰撞查询拒绝。
    let occupied = ScanTracker::default()
        .track_and_check(
            &map.config,
            &map.snapshot,
            &MotionState {
                position: input.pose,
                measured_velocity: 1,
                measured_yaw_rate: 1,
                ..Default::default()
            },
            [0.; 3],
            Some(input.yaw),
        )
        .unwrap_err();
    assert_eq!(
        occupied.reason,
        nuwax_nav_scan_native::track_reason::NO_CANDIDATE
    );
    assert!(occupied.message.contains("Current body pose is occupied"));
    // 同一占据输入现在可证明原地转向脱困；有恢复样本才能与地面被过滤掉区分。
    let escaped = check(&map, input.pose).expect("保留的薄顶面允许通过证明后的转向脱困");
    assert!(!escaped.recovery_samples.is_empty());
    assert_eq!(&escaped.command[..2], &[0.; 2]);
    assert_ne!(escaped.command[2], 0.);
    let stopped = escaped.recovery_samples.last().unwrap();
    assert_eq!(stopped.velocity, [0.; 3]);
    assert_eq!(stopped.yaw_rate, 0.);
}
