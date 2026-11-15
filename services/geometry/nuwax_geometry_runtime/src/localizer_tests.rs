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
            assert!(!session.state.reason.starts_with("定位质量下降："));
        }
    }
    assert!(session
        .state
        .reason
        .starts_with("定位质量下降：最近扫描与地图吻合度为 "));
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
    let result = constrained_step(&source, &target, Isometry3::identity(), candidate, odom);
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
