use super::*;
use crate::mapper::pose_from_isometry;

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
    let (result, limited) =
        constrained_step(&source, &target, Isometry3::identity(), candidate, odom);
    assert!(limited);
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
fn l6_write_permission_uses_both_existing_limits() {
    let points = room();
    let source = SourceCloud::from_rays(points.iter().map(|p| (*p, Point3::origin())));
    for (translation, angle, expected) in [
        (0., 0., false),
        (0.019, 0., false),
        (0.021, 0., true),
        (0., 0.19_f64, false),
        (0., 0.21, true),
        (0.021, 0.21, true),
    ] {
        let candidate = Isometry3::from_parts(
            Translation3::new(translation, 0., 0.),
            UnitQuaternion::from_euler_angles(0., 0., angle.to_radians()),
        );
        let target_points: Vec<_> = points.iter().map(|p| candidate * p).collect();
        let target = TargetCache::new(&target_points, Default::default());
        let (_, limited) = constrained_step(
            &source,
            &target,
            Isometry3::identity(),
            candidate,
            Isometry3::identity(),
        );
        assert_eq!(limited, expected, "平移={translation}米，转角={angle}度");
    }
}

// 沿用 G6 的两墙和地面，3,200 个采样点均落在精细体素中心。
fn l6_surfaces() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for i in 0..40 {
        for j in 0..20 {
            for y in [-2.025, 2.025] {
                points.push(Point3::new(
                    -1.975 + i as f64 * 0.1,
                    y,
                    0.125 + j as f64 * 0.1,
                ));
            }
        }
    }
    for i in 0..40 {
        for j in 0..40 {
            points.push(Point3::new(
                -1.975 + i as f64 * 0.1,
                -1.975 + j as f64 * 0.1,
                0.025,
            ));
        }
    }
    points
}

fn l6_frame(id: u64, stamp: u64, x: f64, bias: f64, yaw: f64, board: Option<f64>) -> MappingFrame {
    let mut scan = l6_surfaces();
    if let Some(distance) = board {
        for i in 0..12 {
            for j in 0..16 {
                let point = Point3::new(
                    -0.575 + i as f64 * 0.1,
                    -2.025 + distance,
                    0.325 + j as f64 * 0.1,
                );
                // 同一帧内即使有多次回波，每个体素也只有一票。
                scan.extend([point; 4]);
            }
        }
    }
    let truth = Isometry3::translation(x, 0., 0.);
    let odom = Isometry3::from_parts(
        Translation3::new(x, bias, 0.),
        UnitQuaternion::from_euler_angles(0., 0., yaw),
    );
    let mut input = frame(&scan, truth, odom, stamp);
    input.session_id = id;
    input.ray_origin_body = [0.025, 0.025, 0.65];
    input
}

fn l6_keys(root: &nuwax_map_chunks::SealedMap) -> std::collections::BTreeSet<[i32; 3]> {
    let mut reader = root.reader();
    let mut keys = std::collections::BTreeSet::new();
    for reference in &root.manifest().blocks {
        for &local in reader.block(reference).unwrap().keys() {
            keys.insert(nuwax_map_chunks::global_key(reference.key, local).unwrap());
        }
    }
    keys
}

async fn l6_loaded_session(id: u64, root: std::path::PathBuf) -> super::super::Session {
    use crate::mapper::MapperConfig;
    use crate::proto::{LoadMapRequest, SaveMapRequest};
    let mapper = Mapper::start(id, MapperConfig::default()).unwrap();
    let mut session = super::super::tests::actor(mapper.clone(), root).await;
    let input = l6_frame(id, super::super::now_ns(), 0., 0., 0., None);
    session.router.publish_frame(input);
    session
        .save(SaveMapRequest {
            map_name: "L6原图".into(),
        })
        .await
        .unwrap();
    session
        .load(LoadMapRequest {
            map_name: "L6原图".into(),
        })
        .await
        .unwrap();
    session
}

async fn l6_feed(session: &super::super::Session, mapper: &Mapper, frame: MappingFrame) {
    session.router.publish_frame(frame);
    mapper.surface_map().await.unwrap();
    assert!(
        mapper.error_rx().borrow().is_none(),
        "{:?}",
        *mapper.error_rx().borrow()
    );
}

#[tokio::test]
async fn l6_ten_minute_revisit_does_not_change_fine_or_display_map() {
    use crate::proto::{PlanarSeed, RelocalizeRequest};
    let root = std::env::temp_dir().join(format!("nuwax-l6-revisit-{}", uuid::Uuid::new_v4()));
    let mut session = l6_loaded_session(1601, root.clone()).await;
    let mapper = session.mapper().unwrap();
    let base = super::super::now_ns();
    l6_feed(
        &session,
        &mapper,
        l6_frame(mapper.session_id(), base, 0., 0., 0., None),
    )
    .await;
    let matched = session
        .relocalize(RelocalizeRequest {
            seed: Some(PlanarSeed::default()),
        })
        .await
        .unwrap();
    assert!(matched.succeeded, "{}", matched.message);
    // 源时间加速回放，逐轮调用同一个生产配准和结果采用入口。
    session.stop_localizer().await;
    let target = session.relocalization_target.clone().unwrap();
    let initial = mapper.snapshot().await.unwrap();
    let initial_fine = initial.cloud.fine.as_ref().unwrap();
    let mut limited_rounds = 0;
    let mut writing_rounds = 0;
    let mut recent = mapper.recent_cloud_rx();
    let first_stamp = base + 200_000_000;
    let input_at = |i: u64| {
        let x = ((i % 400) as f64 / 200. - 1.).abs();
        // 初始 5.5 厘米偏差经一次 2 厘米修正后仍余 3.5 厘米，再叠加 G6 原正弦扰动。
        let bias = 0.055 + 0.035 * (i as f64 * 0.071).sin();
        let yaw = 0.2_f64.to_radians() * (i as f64 * 0.053).sin();
        l6_frame(
            mapper.session_id(),
            first_stamp + i * 200_000_000,
            x,
            bias,
            yaw,
            None,
        )
    };
    for second in 0..600 {
        let start = second * 5;
        let input = input_at(start);
        let round = compute_round(&[input], mapper.map_from_odom(), &target).unwrap();
        assert!(
            round.correction.is_some(),
            "第{second}轮应吻合：{:?}",
            round.score
        );
        if round.limited {
            limited_rounds += 1;
        } else {
            writing_rounds += 1;
        }
        round.apply(&session.router, mapper.session_id()).await;
        recent.borrow_and_update();
        for index in start..start + 5 {
            l6_feed(&session, &mapper, input_at(index)).await;
        }
        assert!(recent.has_changed().unwrap(), "近期点云应继续发布");
    }
    assert!(limited_rounds > 0 && writing_rounds > 0);
    let after = mapper.snapshot().await.unwrap();
    let final_fine = after.cloud.fine.as_ref().unwrap();
    assert_eq!(l6_keys(initial_fine), l6_keys(final_fine));
    assert_eq!(
        initial_fine.manifest().revision,
        final_fine.manifest().revision
    );
    assert_eq!(initial.cloud.display, after.cloud.display);
    assert_eq!(initial.scans, after.scans);
    assert!(Arc::ptr_eq(
        session.relocalization_target.as_ref().unwrap(),
        &target
    ));
    eprintln!(
        "L6十分钟复走：3000帧，{limited_rounds}轮限幅，{writing_rounds}轮许可，精细体素 {}→{}，展示点 {}→{}",
        initial_fine.manifest().points(), final_fine.manifest().points(),
        initial.cloud.display.len() / 12, after.cloud.display.len() / 12
    );
    session.end_current().await.unwrap();
    session.lio().unwrap().shutdown().await;
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn l6_only_persistent_structure_far_from_the_base_is_saved_and_reloaded() {
    use crate::proto::{
        LoadMapRequest, PlanarSeed, RelocalizeRequest, SaveMapRequest, SessionPhase,
    };
    let ns = 1_000_000_000;
    for (name, distance, hits, accepted) in [
        ("离墙半米", 0.5, vec![0, 2, 4, 6, 8, 10, 12, 14], true),
        ("贴墙一分米", 0.1, vec![0, 2, 4, 6, 8, 10, 12, 14], false),
        ("只出现三帧", 0.5, vec![0, 6, 12], false),
        ("超过十秒的旧票", 0.5, vec![0, 2, 4, 6, 108, 110], false),
    ] {
        let root = std::env::temp_dir().join(format!("nuwax-l6-board-{}", uuid::Uuid::new_v4()));
        let mut session = l6_loaded_session(1603, root.clone()).await;
        let mapper = session.mapper().unwrap();
        let id = mapper.session_id();
        let start = super::super::now_ns();
        let frame_at = |stamp| l6_frame(id, stamp, 0., 0., 0., Some(distance));
        let initial = mapper.snapshot().await.unwrap();
        let initial_fine = initial.cloud.fine.as_ref().unwrap();
        let base = l6_keys(initial_fine);
        let revision = initial_fine.manifest().revision;
        l6_feed(&session, &mapper, frame_at(start)).await;
        let result = session
            .relocalize(RelocalizeRequest {
                seed: Some(PlanarSeed::default()),
            })
            .await
            .unwrap();
        assert!(result.succeeded, "{name}：{}", result.message);
        session.stop_localizer().await;
        // 重定位本身不允许计票。即使带板的帧已经超过一秒，第一次好轮前仍只读。
        for i in 1..=5 {
            l6_feed(&session, &mapper, frame_at(start + i * ns / 4)).await;
        }
        assert_eq!(
            mapper
                .surface_map()
                .await
                .unwrap()
                .unwrap()
                .manifest()
                .revision,
            revision
        );
        let target = session.relocalization_target.clone().unwrap();
        let round =
            compute_round(&[frame_at(start + 2 * ns)], mapper.map_from_odom(), &target).unwrap();
        assert!(
            round.correction.is_some() && !round.limited,
            "{name}：{:?}",
            round.score
        );
        round.apply(&session.router, id).await;
        for &tenth in &hits {
            l6_feed(
                &session,
                &mapper,
                frame_at(start + 2 * ns + tenth * (ns / 10)),
            )
            .await;
            if !accepted || tenth < 10 {
                assert_eq!(
                    mapper
                        .surface_map()
                        .await
                        .unwrap()
                        .unwrap()
                        .manifest()
                        .revision,
                    revision,
                    "{name}第{tenth}拍"
                );
            }
        }
        let after = mapper.snapshot().await.unwrap();
        let after_keys = l6_keys(after.cloud.fine.as_ref().unwrap());
        assert!(base.is_subset(&after_keys), "{name}：基础体素必须保留");
        let new_voxels = after_keys.len() - base.len();
        assert_eq!(new_voxels > 0, accepted, "{name}：新增 {new_voxels} 格");
        assert_eq!(
            after.cloud.display.len() > initial.cloud.display.len(),
            accepted,
            "{name}"
        );
        assert_eq!(initial.scans, after.scans, "{name}：历史扫描清单应相同");
        assert_eq!(
            initial.final_poses, after.final_poses,
            "{name}：历史扫描位姿应相同"
        );
        assert!(Arc::ptr_eq(
            session.relocalization_target.as_ref().unwrap(),
            &target
        ));
        if accepted {
            // 坏轮收回写入许可，近期点云仍然更新。
            let mut wrong = frame_at(start + 4 * ns);
            for p in &mut wrong.points_body {
                p[0] *= 1.8;
                p[1] *= 1.6;
            }
            let rejected = compute_round(&[wrong], mapper.map_from_odom(), &target).unwrap();
            assert!(rejected.correction.is_none());
            rejected.apply(&session.router, id).await;
            let written = after.cloud.fine.as_ref().unwrap().manifest().revision;
            let mut recent = mapper.recent_cloud_rx();
            recent.borrow_and_update();
            l6_feed(&session, &mapper, frame_at(start + 5 * ns)).await;
            assert_eq!(
                mapper
                    .surface_map()
                    .await
                    .unwrap()
                    .unwrap()
                    .manifest()
                    .revision,
                written
            );
            assert!(recent.has_changed().unwrap());
            let overwritten = session.save(SaveMapRequest::default()).await.unwrap();
            assert!(overwritten.replaced);
            assert_eq!(overwritten.map_name, "L6原图");
            assert_eq!(session.phase(), SessionPhase::Localized);
            let renamed = session
                .save(SaveMapRequest {
                    map_name: "L6另存图".into(),
                })
                .await
                .unwrap();
            assert!(!renamed.replaced);
            for name in ["L6原图", "L6另存图"] {
                session
                    .load(LoadMapRequest {
                        map_name: name.into(),
                    })
                    .await
                    .unwrap();
                let loaded = session.mapper().unwrap().snapshot().await.unwrap();
                assert_eq!(
                    l6_keys(loaded.cloud.fine.as_ref().unwrap()),
                    after_keys,
                    "{name}"
                );
                assert_eq!(loaded.cloud.display, after.cloud.display);
            }
        }
        eprintln!(
            "L6{name}：已确认精细体素 {new_voxels}，展示点净增 {}",
            (after.cloud.display.len() - initial.cloud.display.len()) / 12
        );
        session.end_current().await.unwrap();
        session.lio().unwrap().shutdown().await;
        session.nav.shutdown().await;
        std::fs::remove_dir_all(root).unwrap();
    }
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
