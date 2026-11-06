use super::*;

#[tokio::test]
async fn recent_input_survives_end_and_load_for_immediate_relocalization() {
    let mapper = Mapper::start(711, MapperConfig::default()).unwrap();
    let mut session = actor(mapper.clone(), PathBuf::new()).await;
    let points_body = (-10..=10).flat_map(|a| (-10..=10).flat_map(move |b| {
        let u = a as f64 * 0.1;
        let v = b as f64 * 0.1;
        [[3., u, v], [u, 2., v], [u, v, -0.5]]
    })).collect();
    let frame = MappingFrame {
        session_id: 711,
        odom_pose: pose_from_isometry(nalgebra::Isometry3::translation(4., 0., 0.), "odom", 1_000_000_000),
        points_body,
        ray_origin_body: [0.; 3],
        mapping_only: false,
        sensor_id: "lidar".into(),
        registered_cloud: None,
    };
    session.router.publish_frame(frame.clone());
    let snapshot = mapper.snapshot().await.unwrap();
    session.end_current().await.unwrap();
    let mut idle_frame = frame.clone();
    idle_frame.session_id = 0;
    idle_frame.odom_pose.timestamp_ns += 1;
    session.router.publish_frame(idle_frame);
    let loaded = Mapper::load(712, MapperConfig::default(), snapshot, true).unwrap();
    session.install(loaded, SessionPhase::Loaded);
    let frames = session.router.recent_frames();
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].odom_pose, frame.odom_pose, "输入缓冲保留原始里程计坐标");
    let result = session.relocalize(RelocalizeRequest::default()).await.unwrap();
    assert!(result.succeeded, "{}", result.message);
    assert_eq!(result.frames_used, 2);
    assert_eq!(session.phase(), SessionPhase::Localized);
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
}

async fn actor(mapper: Arc<Mapper>, root: PathBuf) -> Session {
    let (router, _, _) = test_router(mapper.clone());
    let (route, _) = watch::channel(router.route.borrow().clone());
    let router = FrameRouter {
        route: route.subscribe(),
        ..router
    };
    let state = GeometryState {
        phase: SessionPhase::Mapping as i32,
        map_name: "未保存图".into(),
        info: Some(RuntimeInfo {
            pipeline: "lio_3d".into(),
            ..Default::default()
        }),
        ..Default::default()
    };
    let (publisher, _) = watch::channel(state.clone());
    let (lio, _) = watch::channel(None);
    let (nav_status, _) = mpsc::unbounded_channel();
    Session {
        state,
        publisher,
        route,
        router,
        config: MapperConfig::default(),
        store: Arc::new(MapStore::new(root)),
        save_sources: HashSet::new(),
        next_session: mapper.session_id(),
        lio,
        nav: NavBridge::with_transport(
            nuwax_transport::LcmTransport::new_real("memq://").unwrap(),
            nav_status,
        )
        .await
        .unwrap(),
        explorer: None,
    }
}
#[tokio::test]
async fn component_fault_preserves_real_map_and_allows_save() {
    let mapper = Mapper::start(
        701,
        MapperConfig {
            map_name: "未保存图".into(),
            ..Default::default()
        },
    )
    .unwrap();
    mapper.feed_grid(
        701,
        OccupancyGrid {
            map_name: "未保存图".into(),
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: 1,
                ..Default::default()
            }),
            data: vec![0, 100, 255, 0],
        },
    );
    let before = mapper.snapshot().await.unwrap().occupancy.unwrap();
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2会话测试")
        .join(uuid::Uuid::new_v4().to_string());
    let mut session = actor(mapper.clone(), root.clone()).await;
    session.fail("原生LIO子进程退出".into()).await;
    assert_eq!(session.phase(), SessionPhase::Fault);
    assert_eq!(session.state.map_name, "未保存图");
    assert_eq!(
        session
            .mapper()
            .unwrap()
            .snapshot()
            .await
            .unwrap()
            .occupancy,
        Some(before.clone())
    );
    session.router.publish_planar(
        701,
        OccupancyGrid {
            data: vec![100; 4],
            ..before.clone()
        },
    );
    assert_eq!(
        mapper.snapshot().await.unwrap().occupancy,
        Some(before.clone()),
        "故障后停止接收新帧"
    );
    session.save(SaveMapRequest::default()).await.unwrap();
    assert_eq!(
        session.store.load("未保存图").unwrap().occupancy,
        Some(before)
    );
    assert_eq!(session.phase(), SessionPhase::Fault);
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}
#[tokio::test]
async fn navigation_terminal_and_cancel_do_not_duplicate_previous_navigation() {
    let mapper = Mapper::start(702, MapperConfig::default()).unwrap();
    let mut session = actor(mapper, PathBuf::new()).await;
    session.state.navigation = Some(NavigationStatus {
        goal_id: "当前".into(),
        status: GoalStatus::Executing as i32,
        ..Default::default()
    });
    session
        .navigation(NavigationStatus {
            goal_id: "当前".into(),
            status: GoalStatus::Succeeded as i32,
            ..Default::default()
        })
        .await;
    assert!(session.state.previous_navigation.is_none());
    session.replace_navigation().await.unwrap();
    assert_eq!(
        session.state.previous_navigation.as_ref().unwrap().goal_id,
        "当前"
    );
    session.state.navigation = Some(NavigationStatus {
        goal_id: "第二".into(),
        status: GoalStatus::WaitingInput as i32,
        ..Default::default()
    });
    session.cancel("", GoalStatus::Cancelled).await.unwrap();
    assert_eq!(
        session.state.previous_navigation.as_ref().unwrap().goal_id,
        "当前"
    );
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
}

#[tokio::test]
async fn correction_of_delayed_scan_preserves_newer_odometry_pose_and_timestamp() {
    let mapper = Mapper::start(703, MapperConfig::default()).unwrap();
    let (router, mut pose, _) = test_router(mapper.clone());
    let (routes_tx, routes) = watch::channel(router.route.borrow().clone());
    let router = FrameRouter {
        route: routes.clone(),
        ..router
    };
    let (maps, _) = watch::channel(None);
    let (faults, _) = mpsc::unbounded_channel();
    let forwarding = tokio::spawn(forward_maps(routes, maps, faults, router.clone()));
    router.publish_pose(
        703,
        pose_from_isometry(
            nalgebra::Isometry3::translation(5., 0., 0.),
            "odom",
            3_000_000_000,
        ),
    );
    let _ = pose.borrow_and_update();
    mapper.feed(crate::mapper::tests::room_frame(703, 2_000_000_000));
    mapper.snapshot().await.unwrap();
    router
        .publish_map_from_odom(
            703,
            pose_from_isometry(
                nalgebra::Isometry3::translation(10., 0., 0.),
                "map",
                2_000_000_000,
            ),
        )
        .await;
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let current = pose.borrow_and_update().clone().unwrap();
            assert_eq!(current.timestamp_ns, 3_000_000_000);
            if (current.position.unwrap().x - 15.).abs() < 1e-9 {
                break;
            }
            pose.changed().await.unwrap();
        }
    })
    .await
    .unwrap();
    drop(routes_tx);
    forwarding.abort();
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn stop_exploration_and_session_commands_do_not_wait_for_exploration_compute() {
    let mapper = Mapper::start(704, MapperConfig::default()).unwrap();
    let stamp = now_ns();
    mapper.feed_grid(
        704,
        OccupancyGrid {
            map_name: "未保存图".into(),
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: stamp,
                ..Default::default()
            }),
            data: vec![0; 4],
        },
    );
    mapper.snapshot().await.unwrap();
    let mut session = actor(mapper.clone(), PathBuf::new()).await;
    session.router.publish_pose(
        704,
        pose_from_isometry(nalgebra::Isometry3::identity(), "map", stamp),
    );
    let (entered, waiting) = oneshot::channel();
    let (release, block) = std::sync::mpsc::channel();
    let explorer = crate::explorer::tests::blocking_fixture(entered, block);
    session.explorer = Some(ExplorerWorker::start(
        explorer,
        mapper.clone(),
        session.router.pose.subscribe(),
    ));
    session.state.phase = SessionPhase::Exploring as i32;
    session.state.exploration = Some(ExplorationStatus {
        exploration_id: "阻塞计算".into(),
        phase: ExplorationPhase::Running as i32,
        ..Default::default()
    });
    session.state.navigation = Some(NavigationStatus {
        goal_id: "子目标".into(),
        status: GoalStatus::WaitingInput as i32,
        exploration_child: true,
        ..Default::default()
    });
    tokio::time::timeout(Duration::from_secs(2), waiting)
        .await
        .unwrap()
        .unwrap();
    // 计算源仍被通道阻塞时，真实会话命令循环必须完成取消、停止和新目标命令。
    let (commands, rx) = mpsc::unbounded_channel();
    let (_, faults) = mpsc::unbounded_channel();
    let (_, navigation) = mpsc::unbounded_channel();
    let state = session.publisher.subscribe();
    let running = tokio::spawn(session.run(rx, faults, navigation));
    let (reply, result) = oneshot::channel();
    commands
        .send(Command::StopExplore(
            StopExplorationRequest::default(),
            reply,
        ))
        .ok()
        .unwrap();
    tokio::time::timeout(Duration::from_secs(1), result)
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert_eq!(state.borrow().phase, SessionPhase::Mapping as i32);
    assert_eq!(
        state.borrow().navigation.as_ref().unwrap().status,
        GoalStatus::Cancelled as i32
    );
    let (reply, result) = oneshot::channel();
    commands
        .send(Command::Goal(
            NavigationGoal {
                pose: Some(pose_from_isometry(
                    nalgebra::Isometry3::translation(1., 0., 0.),
                    "map",
                    0,
                )),
                motion_generation: 7,
                ..Default::default()
            },
            reply,
        ))
        .ok()
        .unwrap();
    tokio::time::timeout(Duration::from_secs(1), result)
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    release.send(()).unwrap();
    let (reply, result) = oneshot::channel();
    commands.send(Command::Shutdown(reply)).ok().unwrap();
    result.await.unwrap();
    running.await.unwrap();
}

#[tokio::test]
async fn failed_navigation_submission_only_fails_goal_and_keeps_unsaved_map() {
    let mapper = Mapper::start(
        705,
        MapperConfig {
            map_name: "未保存图".into(),
            ..Default::default()
        },
    )
    .unwrap();
    mapper.feed_grid(
        705,
        OccupancyGrid {
            map_name: "未保存图".into(),
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: now_ns(),
                ..Default::default()
            }),
            data: vec![0, 100, 255, 0],
        },
    );
    let baseline = mapper.snapshot().await.unwrap().occupancy;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2会话测试")
        .join(uuid::Uuid::new_v4().to_string());
    std::fs::create_dir_all(root.join("未保存图")).unwrap();
    let mut session = actor(mapper.clone(), root.clone()).await;
    session.nav.stop_for_test().await;
    session.state.navigation = Some(NavigationStatus {
        goal_id: "旧目标".into(),
        status: GoalStatus::Executing as i32,
        ..Default::default()
    });
    assert_eq!(
        session.end_current().await.unwrap_err().code(),
        tonic::Code::Unavailable
    );
    assert_eq!(session.phase(), SessionPhase::Mapping);
    assert_eq!(mapper.snapshot().await.unwrap().occupancy, baseline);
    session.state.navigation.as_mut().unwrap().status = GoalStatus::Executing as i32;
    assert!(session.replace_navigation().await.is_err());
    assert_eq!(
        session.state.navigation.as_ref().unwrap().status,
        GoalStatus::Failed as i32
    );
    assert!(!session
        .state
        .navigation
        .as_ref()
        .unwrap()
        .message
        .is_empty());

    let result = session
        .goal(NavigationGoal {
            pose: Some(pose_from_isometry(
                nalgebra::Isometry3::translation(1., 0., 0.),
                "map",
                0,
            )),
            motion_generation: 7,
            ..Default::default()
        })
        .await
        .unwrap();
    assert_eq!(session.phase(), SessionPhase::Mapping);
    let goal = session.state.navigation.as_ref().unwrap();
    assert_eq!(goal.goal_id, result.goal_id);
    assert_eq!(goal.status, GoalStatus::Failed as i32);
    assert!(!goal.message.is_empty());
    assert_eq!(mapper.snapshot().await.unwrap().occupancy, baseline);
    session.save(SaveMapRequest::default()).await.unwrap();
    session.end_current().await.unwrap();
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn incompatible_saved_backend_keeps_current_unsaved_session() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2会话测试")
        .join(uuid::Uuid::new_v4().to_string());
    let source = Mapper::start(
        706,
        MapperConfig {
            map_name: "不同容量".into(),
            compute: crate::mapping_backend::MappingBackendComputeConfig {
                window_capacity: 32768,
                ..MapperConfig::default().compute
            },
            ..Default::default()
        },
    )
    .unwrap();
    source.feed(crate::mapper::tests::room_frame(706, 1_000_000_000));
    MapStore::new(root.clone())
        .save("不同容量", None, source.snapshot().await.unwrap())
        .unwrap();
    source.shutdown().await.unwrap();
    let mapper = Mapper::start(707, MapperConfig::default()).unwrap();
    mapper.feed_grid(
        707,
        OccupancyGrid {
            map_name: "未保存图".into(),
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: 2,
                ..Default::default()
            }),
            data: vec![0, 100, 255, 0],
        },
    );
    let baseline = mapper.snapshot().await.unwrap().occupancy;
    let mut session = actor(mapper.clone(), root.clone()).await;
    let before = session.state.clone();
    let result = session
        .load(LoadMapRequest {
            map_name: "不同容量".into(),
        })
        .await
        .unwrap_err();
    assert_eq!(result.code(), tonic::Code::FailedPrecondition);
    assert!(result.message().contains("capacity mismatch"));
    assert_eq!(session.state, before);
    assert_eq!(mapper.snapshot().await.unwrap().occupancy, baseline);
    session.save(SaveMapRequest::default()).await.unwrap();
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn one_mapping_error_does_not_repeat_route_changes_after_fault() {
    let mapper = Mapper::start(708, MapperConfig::default()).unwrap();
    mapper.feed_grid(
        708,
        OccupancyGrid {
            map_name: "未保存图".into(),
            meta: Some(OccupancyGridMeta {
                width: 1,
                height: 1,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: 1,
                ..Default::default()
            }),
            data: vec![100],
        },
    );
    mapper.snapshot().await.unwrap();
    let mut session = actor(mapper.clone(), PathBuf::new()).await;
    let (fault_tx, faults) = mpsc::unbounded_channel();
    session.router.faults = fault_tx.clone();
    let (map_tx, _) = watch::channel(None);
    let mut routes = session.route.subscribe();
    let forwarding = tokio::spawn(forward_maps(
        routes.clone(),
        map_tx,
        fault_tx,
        session.router.clone(),
    ));
    let mut state = session.publisher.subscribe();
    let (commands, rx) = mpsc::unbounded_channel();
    let (_, navigation) = mpsc::unbounded_channel();
    let running = tokio::spawn(session.run(rx, faults, navigation));
    mapper.feed_grid(708, OccupancyGrid::default());
    tokio::time::timeout(Duration::from_secs(2), async {
        while state.borrow_and_update().phase != SessionPhase::Fault as i32 {
            state.changed().await.unwrap();
        }
    })
    .await
    .unwrap();
    routes.borrow_and_update();
    assert!(
        tokio::time::timeout(Duration::from_millis(100), routes.changed())
            .await
            .is_err(),
        "同一建图故障不应反复发布路由变化"
    );
    assert_eq!(
        mapper.snapshot().await.unwrap().occupancy.unwrap().data,
        vec![100]
    );
    let (reply, result) = oneshot::channel();
    commands.send(Command::Shutdown(reply)).ok().unwrap();
    result.await.unwrap();
    running.await.unwrap();
    forwarding.abort();
}

#[tokio::test]
async fn faulted_planar_session_saves_owned_grid_and_cloud_and_rejects_resume_after_restart() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2会话测试")
        .join(uuid::Uuid::new_v4().to_string());
    let source = Mapper::start(709, MapperConfig::default()).unwrap();
    source.feed(crate::mapper::tests::room_frame(709, 1_000_000_000));
    let snapshot = source.snapshot().await.unwrap();
    assert!(!snapshot.cloud.display.is_empty());
    let config = MapperConfig {
        map_name: "未保存图".into(),
        backend: crate::mapping_backend::MappingBackendSpec::SlamToolbox,
        ..Default::default()
    };
    let mapper = Mapper::load(710, config.clone(), snapshot, false).unwrap();
    let held_grid = OccupancyGrid {
        map_name: "未保存图".into(),
        meta: Some(OccupancyGridMeta {
            width: 3,
            height: 2,
            resolution: 0.1,
            frame_id: "map".into(),
            timestamp_ns: 2_000_000_000,
            ..Default::default()
        }),
        data: vec![0, 100, 255, 0, 0, 100],
    };
    mapper.feed_grid(710, held_grid.clone());
    let held = mapper.snapshot().await.unwrap();
    let mut session = actor(mapper.clone(), root.clone()).await;
    session.config = config.clone();
    session.state.info.as_mut().unwrap().pipeline = "slam_2d".into();
    session.fail("二维工作进程退出".into()).await;
    session.save(SaveMapRequest::default()).await.unwrap();
    assert_eq!(session.phase(), SessionPhase::Fault);
    assert_eq!(session.state.reason, "二维工作进程退出");
    let saved = session.store.load("未保存图").unwrap();
    assert_eq!(saved.occupancy, Some(held_grid));
    assert_eq!(saved.cloud.display, held.cloud.display);
    assert_eq!(saved.cloud.revision, held.cloud.revision);
    assert!(saved.planar_checkpoint.is_none());
    assert_eq!(
        saved.planar_resume_unavailable_reason.as_deref(),
        Some(PLANAR_RESUME_UNAVAILABLE)
    );
    let descriptor: serde_json::Value =
        serde_json::from_slice(&std::fs::read(root.join("未保存图/geometry/map.json")).unwrap())
            .unwrap();
    assert_eq!(
        descriptor["planar_resume_unavailable_reason"],
        PLANAR_RESUME_UNAVAILABLE
    );
    assert_eq!(descriptor["planar"], false);
    assert!(!root.join("未保存图/geometry/planar.bin").exists());
    assert!(!root.join("未保存图/meta.json").exists());
    session.end_current().await.unwrap();
    session.nav.shutdown().await;

    // 新的会话拥有方直接从地图目录读取原因，不要求先加载或定位到这个残缺地图。
    let reloaded = Mapper::load(711, config.clone(), saved, true).unwrap();
    let mut restarted = actor(reloaded.clone(), root.clone()).await;
    restarted.config = config;
    restarted.state.info.as_mut().unwrap().pipeline = "slam_2d".into();
    for phase in [
        SessionPhase::Idle,
        SessionPhase::Loaded,
        SessionPhase::Localized,
        SessionPhase::Fault,
    ] {
        if phase == SessionPhase::Idle {
            restarted.route.send_replace(None);
            restarted.state.phase = phase as i32;
            restarted.state.map_name.clear();
        } else {
            restarted.install(reloaded.clone(), SessionPhase::Loaded);
            restarted.state.map_name = "未保存图".into();
            if phase == SessionPhase::Fault {
                restarted.fail("二维工作进程退出".into()).await;
            } else {
                restarted.state.phase = phase as i32;
            }
        }
        let before = restarted.state.clone();
        let error = restarted
            .start_mapping(StartMappingRequest {
                map_name: "未保存图".into(),
                continue_saved: true,
            })
            .await
            .unwrap_err();
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(error.message(), PLANAR_RESUME_UNAVAILABLE);
        assert_eq!(restarted.state, before);
    }
    assert_eq!(
        reloaded.snapshot().await.unwrap().cloud.display,
        held.cloud.display
    );
    restarted.end_current().await.unwrap();
    restarted.nav.shutdown().await;
    source.shutdown().await.unwrap();
    std::fs::remove_dir_all(root).unwrap();
}

async fn load_fault_saved_planar_map() -> (Session, PathBuf, OccupancyGrid) {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2会话测试")
        .join(uuid::Uuid::new_v4().to_string());
    let config = MapperConfig {
        map_name: "故障保存图".into(),
        backend: crate::mapping_backend::MappingBackendSpec::SlamToolbox,
        ..Default::default()
    };
    let mapper = Mapper::start(713, config.clone()).unwrap();
    mapper.feed_grid(
        713,
        OccupancyGrid {
            map_name: config.map_name.clone(),
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.1,
                frame_id: "map".into(),
                timestamp_ns: 1,
                ..Default::default()
            }),
            data: vec![0, 100, 255, 0],
        },
    );
    let grid = mapper.snapshot().await.unwrap().occupancy.unwrap();
    let mut session = actor(mapper, root.clone()).await;
    session.state.info.as_mut().unwrap().pipeline = "slam_2d".into();
    session.state.map_name = config.map_name.clone();
    session.config = config;
    session.fail("二维工作进程退出".into()).await;
    session.save(SaveMapRequest::default()).await.unwrap();
    session
        .load(LoadMapRequest {
            map_name: grid.map_name.clone(),
        })
        .await
        .unwrap();
    (session, root, grid)
}

#[tokio::test]
async fn fault_saved_planar_map_loads_its_grid_without_publishing_map_transform() {
    let (mut session, root, grid) = load_fault_saved_planar_map().await;
    assert_eq!(session.phase(), SessionPhase::Loaded);
    assert_eq!(session.state.reason, "缺少二维定位图，只能查看");
    assert!(session.snapshot().view_only);
    assert_eq!(session.mapper().unwrap().occupancy(false), Some(grid.clone()));
    assert_eq!(session.mapper().unwrap().occupancy(true), Some(grid));
    session.router.publish_pose(
        session.router.session_id(),
        pose_from_isometry(nalgebra::Isometry3::identity(), "odom", 2),
    );
    assert_eq!(session.snapshot().pose.unwrap().frame_id, "odom");
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn fault_saved_planar_map_rejects_relocalization_with_reason() {
    let (mut session, root, _) = load_fault_saved_planar_map().await;
    let error = session
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap_err();
    assert_eq!(error.code(), tonic::Code::FailedPrecondition);
    assert_eq!(error.message(), "缺少二维定位图，只能查看");
    assert_eq!(session.phase(), SessionPhase::Loaded);
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}

#[tokio::test]
async fn fault_saved_planar_map_rejects_navigation_while_unlocalized() {
    let (mut session, root, _) = load_fault_saved_planar_map().await;
    let error = session
        .goal(NavigationGoal {
            pose: Some(pose_from_isometry(nalgebra::Isometry3::identity(), "map", 1)),
            motion_generation: 7,
            ..Default::default()
        })
        .await
        .unwrap_err();
    assert_eq!(error.code(), tonic::Code::FailedPrecondition);
    assert_eq!(error.message(), "未定位");
    assert!(session.state.navigation.is_none());
    session.end_current().await.unwrap();
    session.nav.shutdown().await;
    std::fs::remove_dir_all(root).unwrap();
}
