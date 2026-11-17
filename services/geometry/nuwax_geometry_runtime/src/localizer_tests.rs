use super::*;

fn room() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for x in 0..47 {
        for y in 0..33 {
            let (x, y) = (-3.0 + x as f64 * 0.13, -2.0 + y as f64 * 0.13);
            points.extend([Point3::new(x, y, -1.0), Point3::new(x, y, 1.73)]);
        }
    }
    for z in 0..22 {
        let z = -1.0 + z as f64 * 0.13;
        for x in 0..47 {
            let x = -3.0 + x as f64 * 0.13;
            points.extend([Point3::new(x, -2.0, z), Point3::new(x, 2.16, z)]);
        }
        for y in 0..33 {
            let y = -2.0 + y as f64 * 0.13;
            points.extend([Point3::new(-3.0, y, z), Point3::new(2.98, y, z)]);
        }
        for u in 0..5 {
            points.push(Point3::new(1.9, 0.8 + u as f64 * 0.13, z));
        }
    }
    points
}
fn frame(
    map: &[Point3<f64>],
    truth: Isometry3<f64>,
    odom: Isometry3<f64>,
    stamp: u64,
) -> MappingFrame {
    MappingFrame {
        session_id: 1,
        odom_pose: pose_from_isometry(odom, "odom", stamp),
        points_body: map
            .iter()
            .map(|p| {
                let p = truth.inverse() * p;
                [p.x, p.y, p.z]
            })
            .collect(),
        ray_origin_body: [0.0; 3],
        mapping_only: false,
        sensor_id: "合成雷达".into(),
        registered_cloud: None,
    }
}

#[test]
fn sixty_rounds_follow_drift_and_report_room_and_large_map_timings() {
    let points = room();
    let target = TargetCache::new(&points, Default::default());
    let truth = Isometry3::from_parts(
        Translation3::new(0.4, -0.3, 0.2),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
    );
    let mut correction = Isometry3::identity();
    let mut maximum_translation: f64 = 0.0;
    let mut maximum_angle: f64 = 0.0;
    for second in 1..=60 {
        let odom = Isometry3::from_parts(
            Translation3::from(
                truth.translation.vector + Vector3::new(0.02 * second as f64, 0.0, 0.0),
            ),
            truth.rotation
                * UnitQuaternion::from_euler_angles(0.0, 0.0, (0.1 * second as f64).to_radians()),
        );
        let round = compute_round(
            &[frame(&points, truth, odom, second * 1_000_000_000)],
            correction,
            &target,
        )
        .unwrap();
        correction = round.correction.expect("房间扫描应通过评分");
        let error = truth.inverse() * correction * odom;
        maximum_translation = maximum_translation.max(error.translation.vector.norm());
        maximum_angle = maximum_angle.max(error.rotation.angle().to_degrees());
        assert!(
            maximum_translation <= 0.05,
            "第{second}轮平移={maximum_translation}"
        );
        assert!(maximum_angle <= 1.0, "第{second}轮转角={maximum_angle}");
        if second == 1 {
            eprintln!(
                "普通房间：源={} 30米目标={} 配准={:.3}ms 评分={:.3}ms",
                round.source_points, round.target_points, round.registration_ms, round.scoring_ms
            );
        }
    }
    let untracked = Vector3::new(0.02 * 60.0, 0.0, 0.0).norm();
    assert!(untracked > maximum_translation);
    eprintln!("60轮：跟踪最大平移={maximum_translation:.6}m 最大角度={maximum_angle:.6}deg；不开启60秒平移={untracked:.3}m 偏航=6deg");
    let mut large = points.clone();
    for x in -60..=60 {
        for y in -60..=60 {
            large.extend([
                Point3::new(x as f64 * 0.5, y as f64 * 0.5, -1.0),
                Point3::new(x as f64 * 0.5, y as f64 * 0.5, 1.73),
            ]);
        }
    }
    let target = TargetCache::new(&large, Default::default());
    let odom = Isometry3::from_parts(
        Translation3::from(truth.translation.vector + Vector3::new(0.02, 0.0, 0.0)),
        truth.rotation * UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1_f64.to_radians()),
    );
    let round = compute_round(
        &[frame(&points, truth, odom, 1)],
        Isometry3::identity(),
        &target,
    )
    .unwrap();
    assert!(round.correction.is_some());
    eprintln!(
        "60米方形地图：全图目标={} 源={} 30米目标={} 配准={:.3}ms 评分={:.3}ms",
        target.points.len(),
        round.source_points,
        round.target_points,
        round.registration_ms,
        round.scoring_ms
    );
}

#[test]
fn corridor_keeps_unconstrained_translation_and_corrects_across_walls() {
    let points: Vec<_> = (-150..=150)
        .flat_map(|x| {
            (-10..=20).flat_map(move |z| {
                [
                    Point3::new(x as f64 * 0.13, -2.0, z as f64 * 0.13),
                    Point3::new(x as f64 * 0.13, 2.0, z as f64 * 0.13),
                ]
            })
        })
        .collect();
    let target = TargetCache::new(&points, Default::default());
    let truth = Isometry3::identity();
    let odom = Isometry3::translation(0.08, 0.04, 0.0);
    // 长地图只观测机器人附近两面墙，避免远处贴墙掠射线的体素量化混入退化方向测试。
    let scan: Vec<_> = points
        .iter()
        .filter(|p| p.x.abs() <= 2.5)
        .copied()
        .collect();
    let round = compute_round(
        &[frame(&scan, truth, odom, 1)],
        Isometry3::identity(),
        &target,
    )
    .unwrap();
    let correction = round.correction.unwrap();
    assert!(
        correction.translation.x.abs() < 1e-9,
        "不约束方向被改动：{correction:?}"
    );
    assert!(
        correction.translation.y < -0.019,
        "垂直方向应修正：{correction:?}"
    );
}

async fn loaded_session() -> super::super::Session {
    use crate::mapper::MapperConfig;
    use crate::proto::SessionPhase;
    let points = room();
    let mapper = Mapper::start(1501, MapperConfig::default()).unwrap();
    let mut input = frame(&points, Isometry3::identity(), Isometry3::identity(), 1);
    input.session_id = 1501;
    mapper.feed(input);
    let snapshot = mapper.snapshot().await.unwrap();
    mapper.shutdown().await.unwrap();
    let loaded = Mapper::load(1502, MapperConfig::default(), snapshot, true).unwrap();
    let mut session = super::super::tests::actor(loaded.clone(), std::path::PathBuf::new()).await;
    session.install(loaded, SessionPhase::Loaded);
    session.relocalization_target = Some(Arc::new(TargetCache::new(&points, Default::default())));
    session
}

#[tokio::test]
async fn quality_warns_after_ten_bad_rounds_and_recovers_without_changing_phase_or_goal() {
    use crate::proto::{GoalStatus, NavigationGoal, NavigationStatus, SessionPhase};
    let mut session = loaded_session().await;
    let mapper = session.mapper().unwrap();
    session.install(mapper.clone(), SessionPhase::Localized);
    session.start_localizer(mapper.clone());
    let points = room();
    let wrong: Vec<_> = points
        .iter()
        .map(|p| Point3::new(p.x * 1.8, p.y * 1.6, p.z))
        .collect();
    session.state.navigation = Some(NavigationStatus {
        goal_id: "正在导航".into(),
        status: GoalStatus::Executing as i32,
        ..Default::default()
    });
    let before = mapper.map_from_odom();
    for round in 1..=10 {
        let mut input = frame(&wrong, Isometry3::identity(), Isometry3::identity(), round);
        input.session_id = mapper.session_id();
        session.router.publish_frame(input);
        let update = session.localizer.as_mut().unwrap().next().await.unwrap();
        assert_eq!(update.bad_rounds, round as u32);
        session.localization_update(update);
        assert_eq!(mapper.map_from_odom(), before);
        assert_eq!(session.phase(), SessionPhase::Localized);
        assert_eq!(
            session.state.navigation.as_ref().unwrap().status,
            GoalStatus::Executing as i32
        );
        if round < 10 {
            assert_ne!(session.state.reason, super::super::LOCALIZATION_DEGRADED);
        }
    }
    assert_eq!(session.state.reason, "定位质量下降，建议重新定位");
    assert_eq!(
        session
            .state
            .localization_quality
            .as_ref()
            .unwrap()
            .bad_rounds,
        10
    );
    let goal = session
        .goal(NavigationGoal {
            pose: Some(pose_from_isometry(Isometry3::identity(), "map", 1)),
            motion_generation: 1,
            ..Default::default()
        })
        .await;
    assert!(goal.is_ok(), "质量下降不得拒绝新目标：{goal:?}");
    session.router.clear_recent_frames();
    let mut input = frame(&points, Isometry3::identity(), Isometry3::identity(), 11);
    input.session_id = mapper.session_id();
    session.router.publish_frame(input);
    let update = session.localizer.as_mut().unwrap().next().await.unwrap();
    assert_eq!(update.bad_rounds, 0);
    session.localization_update(update);
    assert!(session.state.reason.is_empty());
    assert_eq!(session.phase(), SessionPhase::Localized);
    session.end_current().await.unwrap();
    assert!(session.localizer.is_none());
    assert!(session.state.localization_quality.is_none());
    session.nav.shutdown().await;
}

#[tokio::test]
async fn relocalization_stops_old_writer_restarts_on_success_and_failure_leaves_no_writer() {
    use crate::proto::{RelocalizeRequest, SessionPhase};
    let mut session = loaded_session().await;
    let mapper = session.mapper().unwrap();
    let points = room();
    let publish = |session: &super::super::Session| {
        let odom = Isometry3::translation(0.02, 0.0, 0.0);
        let mut input = frame(&points, Isometry3::identity(), odom, 999);
        input.session_id = mapper.session_id();
        session
            .router
            .publish_pose(mapper.session_id(), input.odom_pose.clone());
        session.router.publish_frame(input);
    };
    publish(&session);
    assert!(
        session
            .relocalize(RelocalizeRequest::default())
            .await
            .unwrap()
            .succeeded
    );
    assert!(session.localizer.is_some());
    let first_stop = session.localizer.as_ref().unwrap().stop.clone();
    let before = mapper.map_from_odom();
    let update = session.localizer.as_mut().unwrap().next().await.unwrap();
    assert_eq!(update.bad_rounds, 0);
    let pose = session.router.pose.borrow().clone().unwrap();
    let expected = mapper.map_from_odom() * Isometry3::translation(0.02, 0.0, 0.0);
    assert!((pose_isometry(&pose).translation.vector - expected.translation.vector).norm() < 1e-9);
    assert!(
        (mapper.map_from_odom().translation.vector - before.translation.vector).norm()
            <= 0.02 + 1e-9
    );
    publish(&session);
    assert!(
        session
            .relocalize(RelocalizeRequest::default())
            .await
            .unwrap()
            .succeeded
    );
    assert!(
        first_stop.send(()).is_err(),
        "旧跟踪线程必须已退出，接收端已回收"
    );
    assert!(session.localizer.is_some());
    let second_stop = session.localizer.as_ref().unwrap().stop.clone();
    session.router.clear_recent_frames();
    assert!(
        !session
            .relocalize(RelocalizeRequest::default())
            .await
            .unwrap()
            .succeeded
    );
    assert!(second_stop.send(()).is_err());
    assert!(session.localizer.is_none());
    assert_eq!(session.phase(), SessionPhase::Loaded);
    let stopped_transform = mapper.map_from_odom();
    publish(&session);
    tokio::time::sleep(Duration::from_millis(1150)).await;
    assert_eq!(
        mapper.map_from_odom(),
        stopped_transform,
        "退回已加载后不能继续跟踪写入"
    );
    session.install(mapper.clone(), SessionPhase::Localized);
    session.state.info.as_mut().unwrap().pipeline = "slam_2d".into();
    session.start_localizer(mapper.clone());
    assert!(session.localizer.is_none(), "二维链不启动跟踪");
    session.state.info.as_mut().unwrap().pipeline = "lio_3d".into();
    session.start_localizer(mapper);
    let fault_stop = session.localizer.as_ref().unwrap().stop.clone();
    session.fail("合成输入故障".into()).await;
    assert!(fault_stop.send(()).is_err());
    assert!(session.localizer.is_none());
    assert_eq!(session.phase(), SessionPhase::Fault);
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
}

#[test]
fn empty_input_skips_quality_and_step_limits_robot_pose_at_nonzero_odometry() {
    let points = room();
    let target = TargetCache::new(&points, Default::default());
    assert!(compute_round(&[], Isometry3::identity(), &target).is_none());
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    let odom = Isometry3::translation(20.0, -15.0, 0.4);
    let candidate = Isometry3::from_parts(
        Translation3::new(0.1, -0.1, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.5_f64.to_radians()),
    );
    let target_points: Vec<_> = points.iter().map(|p| candidate * p).collect();
    let target = TargetCache::new(&target_points, Default::default());
    let (result, limited) = constrained_step(&source, &target, Isometry3::identity(), candidate, odom);
    assert!(limited, "本轮修正量超过现有平移和转角限幅");
    let before = odom;
    let after = result * odom;
    assert!((after.translation.vector - before.translation.vector).norm() <= 0.02 + 1e-9);
    assert!(
        (before.rotation.inverse() * after.rotation)
            .angle()
            .to_degrees()
            <= 0.2 + 1e-9
    );
}

#[test]
fn pending_relocalization_command_has_no_tracking_writes() {
    use crate::proto::{RelocalizeRequest, SessionPhase};
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .max_blocking_threads(1)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        let mut session = loaded_session().await;
        let mapper = session.mapper().unwrap();
        session.install(mapper.clone(), SessionPhase::Localized);
        let mut input = frame(
            &room(),
            Isometry3::identity(),
            Isometry3::translation(0.04, 0.0, 0.0),
            123,
        );
        input.session_id = mapper.session_id();
        session.router.publish_frame(input.clone());
        session.start_localizer(mapper.clone());
        session.localizer.as_mut().unwrap().next().await.unwrap();
        let old_stop = session.localizer.as_ref().unwrap().stop.clone();
        let before = mapper.map_from_odom();
        session.router.clear_recent_frames();
        session.router.publish_frame(input);
        // 暂占唯一阻塞工作槽，使真实重定位命令持续未完成，期间仍可观察地图变换。
        let (release, released) = std::sync::mpsc::channel();
        let (entered, ready) = tokio::sync::oneshot::channel();
        let blocker = tokio::task::spawn_blocking(move || {
            let _ = entered.send(());
            let _ = released.recv();
        });
        ready.await.unwrap();
        {
            let pending = session.relocalize(RelocalizeRequest::default());
            tokio::pin!(pending);
            tokio::select! {
                result = &mut pending => panic!("阻塞槽未释放时命令不应完成：{result:?}"),
                _ = tokio::time::sleep(Duration::from_millis(1300)) => {}
            }
            assert!(old_stop.send(()).is_err(), "命令尚未完成时旧跟踪线程已退出");
            assert_eq!(
                mapper.map_from_odom(),
                before,
                "重定位命令待完成期间跟踪器不得写回"
            );
            release.send(()).unwrap();
            assert!(pending.await.unwrap().succeeded);
        }
        blocker.await.unwrap();
        assert!(session.localizer.is_some());
        session.end_current().await.unwrap();
        session.nav.shutdown().await;
    });
}

/// 沿用G6调查的两墙、地面与3,200个固定采样点。
fn g6_surfaces(board: bool) -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for i in 0..40 {
        for j in 0..20 {
            for y in [-2.025, 2.025] {
                points.push(Point3::new(-1.975 + i as f64 * 0.1, y, 0.125 + j as f64 * 0.1));
            }
        }
    }
    for i in 0..40 {
        for j in 0..40 {
            points.push(Point3::new(-1.975 + i as f64 * 0.1, -1.975 + j as f64 * 0.1, 0.025));
        }
    }
    if board {
        for i in 0..12 {
            for j in 0..16 {
                points.push(Point3::new(0.525, -0.575 + i as f64 * 0.1, 0.325 + j as f64 * 0.1));
            }
        }
    }
    points
}

fn g6_frame(session: u64, stamp: u64, x: f64, bias: f64, yaw: f64, board: bool) -> MappingFrame {
    MappingFrame {
        session_id: session,
        odom_pose: pose_from_isometry(
            Isometry3::from_parts(Translation3::new(x, bias, 0.), UnitQuaternion::from_euler_angles(0., 0., yaw)),
            "odom", stamp,
        ),
        points_body: g6_surfaces(board).iter().map(|p| [p.x - x, p.y, p.z]).collect(),
        ray_origin_body: [0.025, 0.025, 0.65],
        mapping_only: false,
        sensor_id: "G6合成雷达".into(),
        registered_cloud: None,
    }
}

fn g6_keys(root: &nuwax_map_chunks::SealedMap) -> std::collections::BTreeSet<[i32; 3]> {
    let mut reader = root.reader();
    root.manifest().blocks.iter().flat_map(|reference| {
        reader.block(reference).unwrap().keys().map(|&local| {
            nuwax_map_chunks::global_key(reference.key, local).unwrap()
        }).collect::<Vec<_>>()
    }).collect()
}

fn g6_wall_p95(root: &nuwax_map_chunks::SealedMap) -> f64 {
    let size = f64::from(root.manifest().voxel_size_m);
    let mut distances: Vec<_> = g6_keys(root).into_iter().map(|key| key.map(|v| (v as f64 + 0.5) * size))
        .filter(|p| p[2] > 0.1 && (p[1].abs() - 2.025).abs() < 0.3)
        .map(|p| (p[1].abs() - 2.025).abs()).collect();
    distances.sort_by(f64::total_cmp);
    distances[(distances.len() - 1) * 95 / 100]
}

async fn g6_loaded_session(id: u64, root: std::path::PathBuf) -> super::super::Session {
    let mapper = Mapper::start(id, crate::mapper::MapperConfig::default()).unwrap();
    mapper.feed(g6_frame(id, super::super::now_ns(), 0., 0., 0., false));
    let snapshot = mapper.snapshot().await.unwrap();
    // 先保存再加载，加载会话的历史扫描引用必须指向持久归档。
    let store = crate::map_store::MapStore::new(root.clone());
    store.save("G6原图", None, snapshot).unwrap();
    let snapshot = store.load("G6原图").unwrap();
    mapper.shutdown().await.unwrap();
    let loaded = Mapper::load(id + 1, crate::mapper::MapperConfig::default(), snapshot, true).unwrap();
    let mut session = super::super::tests::actor(loaded.clone(), root).await;
    session.install(loaded, crate::proto::SessionPhase::Localized);
    session.relocalization_target = Some(Arc::new(TargetCache::new(&g6_surfaces(false), Default::default())));
    session
}

/// 加速源时间，逐轮调用生产配准、变换发布、会话写入切换；不改跟踪器的真实秒钟。
async fn g6_tracking_round(session: &mut super::super::Session, input: &MappingFrame) -> bool {
    let mapper = session.mapper().unwrap();
    let target = session.relocalization_target.as_ref().unwrap();
    let round = compute_round(&[input.clone()], mapper.map_from_odom(), target).unwrap();
    let correction = round.correction.expect("固定场景的跟踪候选应吻合");
    session.router.publish_map_from_odom(mapper.session_id(), pose_from_isometry(correction, "map", input.odom_pose.timestamp_ns)).await;
    session.localization_update(TrackingUpdate {
        score: round.score, bad_rounds: 0, limited: round.limited, timestamp_ns: input.odom_pose.timestamp_ns,
    });
    round.limited
}

fn g6_metrics(name: &str, value: serde_json::Value) {
    eprintln!("G6实施 {name}: {value}");
    if let Some(path) = std::env::var_os("NUWAX_G6_IMPLEMENTATION_METRICS") {
        let path = std::path::PathBuf::from(path);
        std::fs::create_dir_all(&path).unwrap();
        std::fs::write(path.join(format!("{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
    }
}

#[test]
fn g6_limited_flag_uses_the_existing_translation_and_rotation_scales() {
    let points = room();
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    for (translation, angle, expected) in [
        (0., 0., false), (0.019, 0., false), (0.021, 0., true),
        (0., 0.19_f64, false), (0., 0.21, true), (0.021, 0.21, true),
    ] {
        let candidate = Isometry3::from_parts(Translation3::new(translation, 0., 0.),
            UnitQuaternion::from_euler_angles(0., 0., angle.to_radians()));
        let target_points: Vec<_> = points.iter().map(|p| candidate * p).collect();
        let target = TargetCache::new(&target_points, Default::default());
        let (_, limited) = constrained_step(&source, &target, Isometry3::identity(), candidate, Isometry3::identity());
        assert_eq!(limited, expected, "平移={translation}米，转角={angle}度");
    }
}

#[tokio::test]
async fn g6_ten_minute_revisit_rejects_clipped_writes_without_growing_or_thickening_walls() {
    let root = std::env::temp_dir().join(format!("nuwax-g6-revisit-{}", uuid::Uuid::new_v4()));
    let mut session = g6_loaded_session(1601, root.clone()).await;
    let mapper = session.mapper().unwrap();
    let target = session.relocalization_target.clone().unwrap();
    let initial = mapper.surface_map().await.unwrap().unwrap();
    let initial_count = initial.manifest().points();
    let initial_p95 = g6_wall_p95(&initial);
    let base = super::super::now_ns();
    let mut limited_rounds = 0;
    let mut writing_rounds = 0;
    for second in 0..600_u64 {
        let index = second * 5;
        let x = ((index % 400) as f64 / 200. - 1.).abs();
        // 5.5厘米初始偏差经2厘米限幅后留下调查里的3.5厘米残差；之后叠加原正弦扰动。
        let bias = 0.055 + 0.035 * (index as f64 * 0.071).sin();
        let yaw = 0.2_f64.to_radians() * (index as f64 * 0.053).sin();
        let input = g6_frame(mapper.session_id(), base + index * 200_000_000, x, bias, yaw, false);
        let limited = g6_tracking_round(&mut session, &input).await;
        let before = mapper.surface_map().await.unwrap().unwrap().manifest().revision;
        if limited { limited_rounds += 1; } else { writing_rounds += 1; }
        let mut recent = mapper.recent_cloud_rx();
        recent.borrow_and_update();
        for step in 0..5 {
            let i = index + step;
            let x = ((i % 400) as f64 / 200. - 1.).abs();
            session.router.publish_frame(g6_frame(mapper.session_id(), base + i * 200_000_000, x,
                0.055 + 0.035 * (i as f64 * 0.071).sin(),
                0.2_f64.to_radians() * (i as f64 * 0.053).sin(), false));
            mapper.surface_map().await.unwrap();
        }
        let after = mapper.surface_map().await.unwrap().unwrap().manifest().revision;
        if limited { assert_eq!(after, before, "第{second}轮限幅时必须只读"); }
        else { assert!(after > before, "第{second}轮追上后必须写入"); }
        assert!(recent.has_changed().unwrap(), "只读与写入期间近期点云都要继续发布");
    }
    let final_map = mapper.surface_map().await.unwrap().unwrap();
    let final_count = final_map.manifest().points();
    let final_p95 = g6_wall_p95(&final_map);
    assert!(limited_rounds > 0 && writing_rounds > 0, "必须实际覆盖限幅与恢复");
    assert!(final_count <= initial_count + initial_count / 100, "精细体素净增长超过1%：{initial_count}→{final_count}");
    assert!(final_p95 <= initial_p95 + 0.01, "墙面法向95分位增加超过1厘米：{initial_p95}→{final_p95}");
    assert!(Arc::ptr_eq(session.relocalization_target.as_ref().unwrap(), &target));
    g6_metrics("十分钟复走", serde_json::json!({"源时间秒":600,"帧数":3000,"限幅轮数":limited_rounds,
        "写入轮数":writing_rounds,"起始精细体素":initial_count,"最终精细体素":final_count,
        "初始墙面法向P95米":initial_p95,"最终墙面法向P95米":final_p95}));
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn g6_tracking_clips_then_resumes_writing_new_board_and_preserves_it_after_reload() {
    let root = std::env::temp_dir().join(format!("nuwax-g6-board-{}", uuid::Uuid::new_v4()));
    let mut session = g6_loaded_session(1603, root.clone()).await;
    let mapper = session.mapper().unwrap();
    let target = session.relocalization_target.clone().unwrap();
    let base = super::super::now_ns();
    let revision = mapper.surface_map().await.unwrap().unwrap().manifest().revision;
    let mut limited = Vec::new();
    session.localizer = Some(Localizer::start(session.router.clone(), mapper.clone(), target.clone()));
    session.router.publish_frame(g6_frame(mapper.session_id(), base, 0., 0.055, 0., true));
    mapper.surface_map().await.unwrap();
    for round in 0..3 {
        // 此用例使用真实逐秒工作线程，核对限幅标志经结果通道到会话后控制实际写入。
        let update = session.localizer.as_mut().unwrap().next().await.unwrap();
        assert_eq!(update.bad_rounds, 0, "限幅不改变本轮吻合质量");
        limited.push(update.limited);
        session.localization_update(update);
        let input = g6_frame(mapper.session_id(), base + (round + 1) * 1_000_000_000, 0., 0.055, 0., true);
        session.router.publish_frame(input);
        let written = mapper.surface_map().await.unwrap().unwrap().manifest().revision;
        if round < 2 { assert_eq!(written, revision, "吻合但被限幅的轮次只读"); }
        else { assert!(written > revision, "第三轮追上后恢复写入"); }
    }
    assert_eq!(limited, vec![true, true, false]);
    let board_count = |root: &nuwax_map_chunks::SealedMap| {
        let size = f64::from(root.manifest().voxel_size_m);
        g6_keys(root).iter().filter(|key| {
            let p = key.map(|v| (v as f64 + 0.5) * size);
            (p[0] - 0.525).abs() < 0.03 && p[1].abs() < 0.7 && p[2] > 0.3
        }).count()
    };
    let board = board_count(&mapper.surface_map().await.unwrap().unwrap());
    assert!(board > 0, "追上后真实新增板必须写入");
    assert!(Arc::ptr_eq(session.relocalization_target.as_ref().unwrap(), &target));
    session.save(crate::proto::SaveMapRequest { map_name: "G6新增板".into() }).await.unwrap();
    session.load(crate::proto::LoadMapRequest { map_name: "G6新增板".into() }).await.unwrap();
    let reloaded = session.mapper().unwrap().surface_map().await.unwrap().unwrap();
    assert_eq!(board_count(&reloaded), board, "保存重载后新增板必须完整可见");
    g6_metrics("新增板保存重载", serde_json::json!({"限幅标志":limited,"写入板体素":board,"重载板体素":board_count(&reloaded)}));
    session.end_current().await.unwrap();
    session.lio().unwrap().shutdown().await;
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}
