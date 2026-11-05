//! 通过隔离的真实 Geometry 进程验收冻结协议与会话行为。
use nuwax_geometry_runtime::proto::{geometry_runtime_client::GeometryRuntimeClient, *};
use std::{
    fs,
    io::Write,
    path::PathBuf,
    process::{Child, Command, Stdio},
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tonic::{transport::Channel, Code};

struct Fixture {
    root: PathBuf,
    child: Child,
    client: GeometryRuntimeClient<Channel>,
}
impl Fixture {
    async fn start(native: bool, pgo: bool) -> Self {
        Self::start_with_missing_lio(native, pgo, false).await
    }
    async fn start_with_missing_lio(native: bool, pgo: bool, missing: bool) -> Self {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("target/B2行为测试")
            .join(uuid::Uuid::new_v4().to_string());
        fs::create_dir_all(&root).unwrap();
        let port = std::net::TcpListener::bind(("127.0.0.1", 0))
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let lcm_port = std::net::UdpSocket::bind(("127.0.0.1", 0))
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let log = fs::File::create(root.join("进程.log")).unwrap();
        let mut command = Command::new(env!("CARGO_BIN_EXE_nuwax-geometry-runtime"));
        command.env("NUWAX_GEOMETRY_RUNTIME_PORT",port.to_string()).env("NUWAX_GEOMETRY_RUNTIME_READY_FILE",root.join("ready"))
            .env("NUWAX_LCM_URL",format!("udpm://239.255.76.67:{lcm_port}?ttl=0"))
            .env("NUWAX_MAP_CATALOG_ROOT",root.join("maps")).env("NUWAX_RUNTIME_DATA_DIR",root.join("runtime"))
            .env("NUWAX_RAW_SCAN_DIRECTORY",root.join("scans")).env("NUWAX_CLOUDMAP_SHM_PATH",root.join("cloud.shm"))
            .env("NUWAX_GEOMETRY_PIPELINE","lio_3d").env("NUWAX_GEOMETRY_LIO_BACKEND",if native{"fastlio2"}else{"device"})
            .env("NUWAX_GEOMETRY_MAPPING_BACKEND","voxel").env("NUWAX_GEOMETRY_CORRECTION_BACKEND",if pgo{"library_pgo"}else{"relocalization"})
            .env("NUWAX_PERSON_FOLLOW_AUTOSTART","false").env("NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB","16")
            .env("NUWAX_MAP_FRAME","map").env("NUWAX_GEOMETRY_ODOM_FRAME","odom").env("NUWAX_BASE_FRAME","base_link")
            .env_remove("NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR").env("NUWAX_ROBOT_GEOMETRY_JSON",r#"{"footprint_xy_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"会话进程测试"}"#)
            .stdout(Stdio::from(log.try_clone().unwrap())).stderr(Stdio::from(log));
        if native {
            use std::os::unix::fs::PermissionsExt;
            let script = root.join("原生进程夹具");
            fs::write(
                &script,
                "#!/bin/sh\nprintf '%s\\n' \"$$\" > \"$B2_CHILD_PID_FILE\"\nexec /bin/sleep 60\n",
            )
            .unwrap();
            fs::set_permissions(&script, fs::Permissions::from_mode(0o755)).unwrap();
            command
                .env(
                    "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
                    if missing {
                        root.join("待恢复原生进程")
                    } else {
                        script
                    },
                )
                .env("B2_CHILD_PID_FILE", root.join("native.pid"))
                .env(
                    "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR",
                    r#"{"translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}"#,
                );
        }
        let mut child = command.spawn().unwrap();
        let ready = root.join("ready");
        let started = tokio::time::Instant::now();
        while !ready.exists() {
            if let Some(status) = child.try_wait().unwrap() {
                panic!(
                    "Geometry提前退出 {status}：{}",
                    fs::read_to_string(root.join("进程.log")).unwrap()
                );
            }
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "Geometry未就绪：{}",
                fs::read_to_string(root.join("进程.log")).unwrap()
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let client = GeometryRuntimeClient::connect(format!("http://127.0.0.1:{port}"))
            .await
            .unwrap()
            .max_decoding_message_size(128 * 1024 * 1024);
        Self {
            root,
            child,
            client,
        }
    }
    async fn state(&mut self) -> GeometryState {
        self.client.get_state(Empty {}).await.unwrap().into_inner()
    }
    async fn mapping(&mut self, name: &str) -> GeometryState {
        self.client
            .start_mapping(StartMappingRequest {
                map_name: name.into(),
                continue_saved: false,
            })
            .await
            .unwrap()
            .into_inner()
    }
    async fn odom(&mut self, stamp: u64, x: f64) {
        self.client
            .publish_odom(PublishOdomRequest {
                pose: Some(pose(stamp, x, "odom")),
                clock_domain: "unix_ns".into(),
                clock_id: "设备时钟".into(),
            })
            .await
            .unwrap();
    }
    async fn frame(&mut self, stamp: u64) {
        self.odom(stamp, 0.).await;
        self.client
            .publish_point_cloud(points(stamp))
            .await
            .unwrap();
    }
    async fn map_ready(&mut self) {
        let start = tokio::time::Instant::now();
        loop {
            match self
                .client
                .get_occupancy_grid(GetOccupancyGridRequest {
                    include_forbidden: false,
                })
                .await
            {
                Ok(grid) if !grid.get_ref().data.is_empty() => return,
                _ => {}
            }
            assert!(
                start.elapsed() < Duration::from_secs(5),
                "未生成地图，状态{:?}，日志{}",
                self.state().await,
                fs::read_to_string(self.root.join("进程.log")).unwrap()
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }
    async fn saved_map(&mut self, name: &str) {
        self.mapping(name).await;
        let stamp = now();
        self.frame(stamp).await;
        self.frame(stamp + 100_000_000).await;
        self.map_ready().await;
        let saved = self.client.save_map(SaveMapRequest::default()).await;
        assert!(
            saved.is_ok(),
            "保存失败{saved:?}，状态{:?}",
            self.state().await
        );
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = nix::sys::signal::kill(
            nix::unistd::Pid::from_raw(self.child.id() as i32),
            nix::sys::signal::Signal::SIGTERM,
        );
        let start = std::time::Instant::now();
        while self.child.try_wait().unwrap().is_none() && start.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(10));
        }
        if self.child.try_wait().unwrap().is_none() {
            let _ = self.child.kill();
        }
        let _ = self.child.wait();
    }
}
fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
fn pose(stamp: u64, x: f64, frame: &str) -> Pose {
    Pose {
        position: Some(Vec3 { x, y: 0., z: 0. }),
        orientation: Some(Quat {
            x: 0.,
            y: 0.,
            z: 0.,
            w: 1.,
        }),
        frame_id: frame.into(),
        timestamp_ns: stamp,
    }
}
fn goal(x: f64) -> NavigationGoal {
    NavigationGoal {
        pose: Some(pose(0, x, "map")),
        motion_generation: 7,
        ..Default::default()
    }
}
fn points(stamp: u64) -> PublishPointCloudRequest {
    let mut points = Vec::new();
    for x in 0..30 {
        for y in 0..30 {
            for p in [
                [1.0 + x as f32 * 0.05, -1.0 + y as f32 * 0.05, 0.0],
                [2.5, -1.0 + y as f32 * 0.05, x as f32 * 0.05],
                [1.0 + x as f32 * 0.05, 0.5, y as f32 * 0.05],
            ] {
                for value in p {
                    points.extend_from_slice(&value.to_le_bytes());
                }
            }
        }
    }
    PublishPointCloudRequest {
        point_count: (points.len() / 12) as u32,
        points,
        point_step: 12,
        frame_id: "odom".into(),
        timestamp_ns: stamp,
        is_dense: true,
        fields: ["x", "y", "z"]
            .into_iter()
            .enumerate()
            .map(|(i, name)| PointCloudField {
                name: name.into(),
                offset: i as u32 * 4,
                datatype: 7,
                count: 1,
            })
            .collect(),
        ..Default::default()
    }
}

#[tokio::test]
async fn session_transitions_save_collision_and_direct_load() {
    let mut f = Fixture::start(false, false).await;
    assert_eq!(f.state().await.phase, SessionPhase::Idle as i32);
    assert_eq!(
        f.client.set_goal(goal(1.)).await.unwrap_err().code(),
        Code::FailedPrecondition
    );
    assert_eq!(
        f.client
            .start_mapping(StartMappingRequest {
                map_name: "../图".into(),
                continue_saved: false
            })
            .await
            .unwrap_err()
            .code(),
        Code::InvalidArgument
    );
    f.saved_map("甲图").await;
    assert!(
        f.client
            .save_map(SaveMapRequest::default())
            .await
            .unwrap()
            .into_inner()
            .replaced
    );
    let renamed = f
        .client
        .save_map(SaveMapRequest {
            map_name: "另存图".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(!renamed.replaced);
    assert_eq!(f.state().await.map_name, "另存图");
    assert!(
        f.client
            .save_map(SaveMapRequest {
                map_name: "甲图".into()
            })
            .await
            .unwrap()
            .into_inner()
            .replaced
    );
    f.mapping("乙图").await;
    assert_eq!(
        f.client
            .save_map(SaveMapRequest {
                map_name: "甲图".into()
            })
            .await
            .unwrap_err()
            .code(),
        Code::AlreadyExists
    );
    let loaded = f
        .client
        .load_map(LoadMapRequest {
            map_name: "甲图".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(loaded.phase, SessionPhase::Loaded as i32);
    assert_eq!(loaded.map_name, "甲图");
    assert_eq!(
        f.client
            .start_mapping(StartMappingRequest {
                map_name: "甲图".into(),
                continue_saved: true
            })
            .await
            .unwrap_err()
            .code(),
        Code::FailedPrecondition
    );
    assert_eq!(
        f.client
            .stop_session(Empty {})
            .await
            .unwrap()
            .into_inner()
            .phase,
        SessionPhase::Idle as i32
    );
    assert_eq!(
        f.client
            .load_map(LoadMapRequest {
                map_name: "不存在".into()
            })
            .await
            .unwrap_err()
            .code(),
        Code::NotFound
    );
}

#[tokio::test]
async fn load_blocked_in_real_filesystem_keeps_pose_output() {
    use std::os::unix::ffi::OsStrExt;
    let mut f = Fixture::start(false, false).await;
    f.saved_map("定位图").await;
    let descriptor = f.root.join("maps/定位图/geometry/map.json");
    let original = fs::read(&descriptor).unwrap();
    fs::remove_file(&descriptor).unwrap();
    let path = std::ffi::CString::new(descriptor.as_os_str().as_bytes()).unwrap();
    assert_eq!(unsafe { libc::mkfifo(path.as_ptr(), 0o600) }, 0);
    let mut poses = f
        .client
        .subscribe_pose(Empty {})
        .await
        .unwrap()
        .into_inner();
    let _ = poses.message().await.unwrap();
    let mut loading = f.client.clone();
    let load = tokio::spawn(async move {
        loading
            .load_map(LoadMapRequest {
                map_name: "定位图".into(),
            })
            .await
    });
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(!load.is_finished());
    let stamp = now() + 500_000_000;
    f.odom(stamp, 4.).await;
    let observed = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let p = poses.message().await.unwrap().unwrap();
            if p.timestamp_ns == stamp {
                break p;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(observed.position.unwrap().x, 4.);
    assert!(!load.is_finished(), "文件读取仍被阻塞时位姿已经交付");
    tokio::task::spawn_blocking(move || {
        let mut writer = fs::File::create(&descriptor).unwrap();
        writer.write_all(&original).unwrap();
        drop(writer);
        fs::remove_file(&descriptor).unwrap();
        fs::write(&descriptor, &original).unwrap();
    })
    .await
    .unwrap();
    assert_eq!(
        load.await.unwrap().unwrap().into_inner().phase,
        SessionPhase::Loaded as i32
    );
}

#[tokio::test]
async fn goals_replace_wait_and_end_exploration() {
    let mut f = Fixture::start(false, false).await;
    f.mapping("目标图").await;
    let first = f
        .client
        .set_goal(goal(1.))
        .await
        .unwrap()
        .into_inner()
        .goal_id;
    assert_eq!(
        f.state().await.navigation.unwrap().status,
        GoalStatus::WaitingInput as i32
    );
    let second = f
        .client
        .set_goal(goal(2.))
        .await
        .unwrap()
        .into_inner()
        .goal_id;
    let state = f.state().await;
    assert_eq!(state.navigation.unwrap().goal_id, second);
    let old = state.previous_navigation.unwrap();
    assert_eq!(old.goal_id, first);
    assert_eq!(old.status, GoalStatus::Replaced as i32);
    let exploration = f
        .client
        .start_exploration(ExplorationRequest {
            motion_generation: 9,
            ..Default::default()
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(f.state().await.phase, SessionPhase::Exploring as i32);
    let progress = f
        .client
        .get_exploration_progress(ExplorationProgressRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert_eq!(progress.exploration_id, exploration.exploration_id);
    f.client.set_goal(goal(3.)).await.unwrap();
    let state = f.state().await;
    assert_eq!(state.phase, SessionPhase::Mapping as i32);
    assert_eq!(
        state.exploration.unwrap().phase,
        ExplorationPhase::Stopped as i32
    );
    f.client
        .start_exploration(ExplorationRequest {
            motion_generation: 10,
            resume_progress_json: progress.progress_json,
            ..Default::default()
        })
        .await
        .unwrap();
    f.client
        .stop_exploration(StopExplorationRequest::default())
        .await
        .unwrap();
    assert_eq!(f.state().await.phase, SessionPhase::Mapping as i32);
}

#[tokio::test]
async fn relocalization_failure_then_success_and_continue() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("重定位图").await;
    f.client
        .load_map(LoadMapRequest {
            map_name: "重定位图".into(),
        })
        .await
        .unwrap();
    let failed = f
        .client
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert!(!failed.succeeded);
    assert_eq!(f.state().await.phase, SessionPhase::Loaded as i32);
    assert!(!f.state().await.reason.is_empty());
    let stamp = now();
    f.frame(stamp).await;
    f.frame(stamp + 100_000_000).await;
    // 完成一次只读请求，确保刚才的真实输入已经到达当前地图对象。
    tokio::time::sleep(Duration::from_millis(100)).await;
    let matched = f
        .client
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert!(matched.succeeded, "{matched:?}");
    assert!(matched.frames_used > 0 && matched.frames_used <= 8);
    assert_eq!(matched.pose.unwrap().frame_id, "map");
    assert_eq!(f.state().await.phase, SessionPhase::Localized as i32);
    let continued = f
        .client
        .start_mapping(StartMappingRequest {
            map_name: "重定位图".into(),
            continue_saved: true,
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(continued.phase, SessionPhase::Mapping as i32);
    assert!(
        f.client
            .save_map(SaveMapRequest::default())
            .await
            .unwrap()
            .into_inner()
            .replaced
    );
}

#[tokio::test]
async fn native_child_kill_faults_session_without_exiting_process() {
    let mut f = Fixture::start(true, false).await;
    f.mapping("故障前").await;
    let parent = f.child.id();
    let pid = fs::read_to_string(f.root.join("native.pid"))
        .unwrap()
        .trim()
        .parse::<i32>()
        .unwrap();
    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(pid),
        nix::sys::signal::Signal::SIGKILL,
    )
    .unwrap();
    let start = tokio::time::Instant::now();
    loop {
        let state = f.state().await;
        if state.phase == SessionPhase::Fault as i32 {
            assert!(
                state.reason.contains("原生LIO子进程退出"),
                "{}",
                state.reason
            );
            break;
        }
        assert!(start.elapsed() < Duration::from_secs(3));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(f.child.try_wait().unwrap().is_none());
    assert_eq!(f.child.id(), parent);
    let fault = f.state().await;
    assert_eq!(fault.map_name, "故障前");
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
    assert_eq!(f.state().await.phase, SessionPhase::Fault as i32);
    assert!(f.root.join("maps/故障前/geometry/map.json").exists());
    f.client.stop_session(Empty {}).await.unwrap();
    assert_eq!(f.state().await.phase, SessionPhase::Idle as i32);
    assert_eq!(
        f.mapping("故障后").await.phase,
        SessionPhase::Mapping as i32
    );
    // 进程已创建后，夹具脚本才异步写出 PID；观察新进程仍沿用本例的三秒期限。
    let replacement = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let current = fs::read_to_string(f.root.join("native.pid"))
                .unwrap()
                .trim()
                .parse::<i32>()
                .unwrap();
            if current != pid {
                break current;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("恢复后应观察到新的原生子进程");
    assert_ne!(pid, replacement);
    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(replacement),
        nix::sys::signal::Signal::SIGKILL,
    )
    .unwrap();
    tokio::time::timeout(Duration::from_secs(3), async {
        while f.state().await.phase != SessionPhase::Fault as i32 {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    f.client.stop_session(Empty {}).await.unwrap();
    let loaded = f
        .client
        .load_map(LoadMapRequest {
            map_name: "故障前".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(loaded.phase, SessionPhase::Loaded as i32);
    assert_eq!(loaded.map_name, "故障前");
}

#[tokio::test]
async fn pgo_relocalization_is_explicitly_rejected() {
    let mut f = Fixture::start(false, true).await;
    f.mapping("回环图").await;
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
    f.client
        .load_map(LoadMapRequest {
            map_name: "回环图".into(),
        })
        .await
        .unwrap();
    let error = f
        .client
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap_err();
    assert_eq!(error.code(), Code::FailedPrecondition);
    assert!(error.message().contains("PGO"));
}

#[tokio::test]
async fn pose_updates_do_not_trigger_state_stream() {
    let mut f = Fixture::start(false, false).await;
    f.mapping("订阅图").await;
    let mut states = f
        .client
        .subscribe_state(Empty {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(
        states.message().await.unwrap().unwrap().phase,
        SessionPhase::Mapping as i32
    );
    f.odom(now(), 1.).await;
    assert!(
        tokio::time::timeout(Duration::from_millis(100), states.message())
            .await
            .is_err()
    );
    f.client.stop_session(Empty {}).await.unwrap();
    assert_eq!(
        states.message().await.unwrap().unwrap().phase,
        SessionPhase::Idle as i32
    );
}

#[tokio::test]
async fn shutdown_closes_live_subscriptions_and_removes_ready_file() {
    let mut f = Fixture::start(false, false).await;
    f.mapping("关闭图").await;
    let mut states = f
        .client
        .subscribe_state(Empty {})
        .await
        .unwrap()
        .into_inner();
    states.message().await.unwrap().unwrap();
    f.odom(now(), 0.).await;
    let mut poses = f
        .client
        .subscribe_pose(Empty {})
        .await
        .unwrap()
        .into_inner();
    poses.message().await.unwrap().unwrap();
    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(f.child.id() as i32),
        nix::sys::signal::Signal::SIGTERM,
    )
    .unwrap();
    let status = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            if let Some(status) = f.child.try_wait().unwrap() {
                break status;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("有状态与位姿订阅时仍应完整退出");
    assert!(status.success());
    assert!(!f.root.join("ready").exists());
    assert!(states.message().await.unwrap().is_none());
    assert!(poses.message().await.unwrap().is_none());
}

#[tokio::test]
async fn failed_start_and_load_preserve_current_unsaved_map() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("已存在").await;
    f.mapping("未保存").await;
    f.frame(now()).await;
    f.map_ready().await;
    let before = f
        .client
        .get_occupancy_grid(GetOccupancyGridRequest {
            include_forbidden: false,
        })
        .await
        .unwrap()
        .into_inner();
    let state = f.state().await;
    assert_eq!(
        f.client
            .start_mapping(StartMappingRequest {
                map_name: "已存在".into(),
                continue_saved: false
            })
            .await
            .unwrap_err()
            .code(),
        Code::AlreadyExists
    );
    assert_eq!(f.state().await, state);
    assert_eq!(
        f.client
            .load_map(LoadMapRequest {
                map_name: "不存在".into()
            })
            .await
            .unwrap_err()
            .code(),
        Code::NotFound
    );
    assert_eq!(f.state().await, state);
    let corrupt = f.root.join("maps/损坏/geometry");
    fs::create_dir_all(&corrupt).unwrap();
    fs::write(corrupt.join("map.json"), "{坏文件").unwrap();
    assert_eq!(
        f.client
            .load_map(LoadMapRequest {
                map_name: "损坏".into()
            })
            .await
            .unwrap_err()
            .code(),
        Code::DataLoss
    );
    assert_eq!(f.state().await, state);
    assert_eq!(
        f.client
            .get_occupancy_grid(GetOccupancyGridRequest {
                include_forbidden: false
            })
            .await
            .unwrap()
            .into_inner(),
        before
    );
    fs::write(
        f.root.join("maps/已存在/geometry/backend.bin"),
        "损坏检查点".as_bytes(),
    )
    .unwrap();
    assert_eq!(
        f.client
            .load_map(LoadMapRequest {
                map_name: "已存在".into()
            })
            .await
            .unwrap_err()
            .code(),
        Code::DataLoss
    );
    assert_eq!(f.state().await, state);
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
    assert!(f.root.join("maps/未保存/geometry/map.json").exists());
}

#[tokio::test]
async fn invalid_goal_intent_finishes_without_stranded_child() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("无效目标").await;
    let mut invalid = goal(2.);
    invalid.pose.as_mut().unwrap().position.as_mut().unwrap().x = f64::NAN;
    assert_eq!(
        f.client.set_goal(invalid).await.unwrap_err().code(),
        Code::InvalidArgument
    );
    let state = f.state().await;
    assert!(state.navigation.is_none());
    assert_eq!(state.phase, SessionPhase::Mapping as i32);
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
}

#[tokio::test]
async fn malformed_forbidden_file_fails_exploration_without_faulting_session() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("禁区错误").await;
    fs::write(f.root.join("maps/禁区错误/forbidden_zones.json"), "{错误").unwrap();
    f.client
        .start_exploration(ExplorationRequest {
            motion_generation: 7,
            ..Default::default()
        })
        .await
        .unwrap();
    let start = tokio::time::Instant::now();
    loop {
        let state = f.state().await;
        assert_ne!(state.phase, SessionPhase::Fault as i32, "{state:?}");
        if state
            .exploration
            .as_ref()
            .is_some_and(|s| s.phase == ExplorationPhase::Failed as i32)
        {
            assert_eq!(state.phase, SessionPhase::Mapping as i32);
            assert!(!state.exploration.unwrap().message.is_empty());
            break;
        }
        assert!(start.elapsed() < Duration::from_secs(3));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
    f.frame(now()).await;
}

#[tokio::test]
async fn initial_lio_start_failure_can_stop_then_start_after_executable_is_restored() {
    let mut f = Fixture::start_with_missing_lio(true, false, true).await;
    assert_eq!(f.state().await.phase, SessionPhase::Fault as i32);
    f.client.stop_session(Empty {}).await.unwrap();
    fs::rename(f.root.join("原生进程夹具"), f.root.join("待恢复原生进程")).unwrap();
    assert_eq!(
        f.mapping("恢复启动").await.phase,
        SessionPhase::Mapping as i32
    );
    tokio::time::timeout(Duration::from_secs(3), async {
        while !f.root.join("native.pid").exists() {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    assert!(f.child.try_wait().unwrap().is_none());
}
