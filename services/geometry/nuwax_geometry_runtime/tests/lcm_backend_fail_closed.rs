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
        Self::launch(native, pgo, missing, None).await
    }
    /// 使用本机编译的原生LIO；`bin/` 下的同名脚本先等 `delay` 秒再执行它，使输入先于子进程订阅到达。
    async fn start_real_native(program: &std::path::Path, delay: &str) -> Self {
        Self::launch(true, false, false, Some((program, delay))).await
    }
    async fn launch(
        native: bool,
        pgo: bool,
        missing: bool,
        real: Option<(&std::path::Path, &str)>,
    ) -> Self {
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
        let bin = root.join("bin");
        fs::create_dir_all(&bin).unwrap();
        let executable = bin.join("nuwax-geometry-runtime");
        fs::copy(env!("CARGO_BIN_EXE_nuwax-geometry-runtime"), &executable).unwrap();
        let mut config = nuwax_config::test_support::defaults();
        config.set("geometry", "runtime_port", i64::from(port));
        config.set(
            "transport",
            "lcm_url",
            format!("udpm://239.255.76.67:{lcm_port}?ttl=0"),
        );
        config.set(
            "geometry",
            "map_catalog_root",
            root.join("maps").to_str().unwrap(),
        );
        config.set(
            "geometry",
            "runtime_data_dir",
            root.join("runtime").to_str().unwrap(),
        );
        config.set(
            "geometry",
            "raw_scan_directory",
            root.join("scans").to_str().unwrap(),
        );
        config.set(
            "geometry",
            "lio_backend",
            if native { "fastlio2" } else { "device" },
        );
        config.set(
            "geometry",
            "correction_backend",
            if pgo { "library_pgo" } else { "relocalization" },
        );
        config.set("person_follow", "autostart", false);
        config.set("geometry", "fine_cloud_cache_mib", 16i64);
        config.set("derived","robot_geometry_json",r#"{"footprint_xy_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"会话进程测试"}"#);
        config.set("derived", "standing_ground_z_m", -0.2);
        config.set("derived", "nav_profile", "pct-scan-3d");
        config.set("derived", "min_executable_linear_mps", 0.0);
        config.set("derived", "min_executable_angular_rps", 0.0);
        config.set("derived", "actuator_max_linear_mps", 0.0);
        config.set("derived", "actuator_max_reverse_mps", 0.0);
        config.set("derived", "actuator_max_yaw_rps", 0.0);
        config.set("derived", "actuator_motion_model", "");
        let mut command = Command::new(executable);
        command
            .env("NUWAX_INSTALL_HOME", &root)
            .stdout(Stdio::from(log.try_clone().unwrap()))
            .stderr(Stdio::from(log));
        if native {
            use std::os::unix::fs::PermissionsExt;
            let identity = r#"{"translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}"#;
            if let Some((program, delay)) = real {
                let name = program.file_name().unwrap().to_str().unwrap();
                let wrapper = bin.join(name);
                fs::write(
                    &wrapper,
                    format!("#!/bin/sh\nsleep {delay}\nexec '{}' \"$@\"\n", program.display()),
                )
                .unwrap();
                fs::set_permissions(&wrapper, fs::Permissions::from_mode(0o755)).unwrap();
                config.set("geometry", "lio_backend", name.strip_suffix("_native").unwrap());
                config.set("geometry", "lio_body_from_imu", identity);
            } else {
                let script = root.join("原生进程夹具");
                fs::write(
                                &script,
                                "#!/bin/sh\nprintf '%s\\n' \"$$\" > \"$B2_CHILD_PID_FILE.tmp\"\nmv \"$B2_CHILD_PID_FILE.tmp\" \"$B2_CHILD_PID_FILE\"\nexec /bin/sleep 60\n",
                            )
                            .unwrap();
                fs::set_permissions(&script, fs::Permissions::from_mode(0o755)).unwrap();
                if !missing {
                    fs::copy(&script, bin.join("fastlio2_native")).unwrap();
                }
                command.env("B2_CHILD_PID_FILE", root.join("native.pid"));
            }
            config.set("geometry", "lio_body_from_sensor", identity);
        }
        config.write_install(&root);
        let mut child = command.spawn().unwrap();
        let ready = root.join(nuwax_config::GEOMETRY_READY);
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
    /// 按真实时间连续送 `count` 帧，帧间隔大于建图候选节拍。
    /// 与 `frame` 相同，另多一块加载后才出现的竖直障碍板。
    async fn obstacle_frame(&mut self, stamp: u64) {
        self.odom(stamp, 0.).await;
        let mut request = points(stamp);
        for y in 0..5 {
            for z in 0..21 {
                for value in [OBSTACLE_X, -0.4 + y as f32 * 0.05, z as f32 * 0.05] {
                    request.points.extend_from_slice(&value.to_le_bytes());
                }
            }
        }
        request.point_count = (request.points.len() / 12) as u32;
        self.client.publish_point_cloud(request).await.unwrap();
    }
    /// 二维物理栅格里障碍板所在处的障碍格数。
    async fn obstacle_cells(&mut self) -> usize {
        let grid = self
            .client
            .get_occupancy_grid(GetOccupancyGridRequest {
                include_forbidden: false,
            })
            .await
            .unwrap()
            .into_inner();
        let meta = grid.meta.unwrap();
        let resolution = f64::from(meta.resolution);
        grid.data
            .iter()
            .enumerate()
            .filter(|(index, value)| {
                let x = meta.origin_x + ((*index as u32 % meta.width) as f64 + 0.5) * resolution;
                let y = meta.origin_y + ((*index as u32 / meta.width) as f64 + 0.5) * resolution;
                **value == 100
                    && (x - f64::from(OBSTACLE_X)).abs() <= 0.1
                    && (-0.45..=-0.15).contains(&y)
            })
            .count()
    }
    /// 稠密点云里障碍板上、高出地面的点数。
    async fn obstacle_points(&mut self) -> usize {
        let (mut epoch, mut offset, mut count) = (String::new(), 0, 0);
        loop {
            let chunk = self
                .client
                .get_dense_cloud_chunk(DenseCloudChunkRequest {
                    expected_epoch: epoch,
                    offset_points: offset,
                    max_points: 65_536,
                    allow_delta: false,
                })
                .await
                .unwrap()
                .into_inner();
            count += chunk
                .xyz_f32_le
                .chunks_exact(12)
                .map(|bytes| {
                    [0, 4, 8].map(|at| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()))
                })
                .filter(|p| {
                    (p[0] - OBSTACLE_X).abs() <= 0.05 && (-0.45..=-0.15).contains(&p[1]) && p[2] >= 0.2
                })
                .count();
            offset += chunk.xyz_f32_le.len() as u64 / 12;
            if offset >= chunk.total_points {
                return count;
            }
            epoch = chunk.epoch;
        }
    }
    async fn frames(&mut self, count: usize) {
        for _ in 0..count {
            self.frame(now()).await;
            tokio::time::sleep(Duration::from_millis(40)).await;
        }
    }
    /// 给新建图送满默认持续性确认所需的五帧和一秒源时间跨度。
    async fn mapping_frames(&mut self) {
        self.frame(now()).await;
        for _ in 1..5 {
            tokio::time::sleep(Duration::from_millis(250)).await;
            self.frame(now()).await;
        }
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
        self.mapping_frames().await;
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
#[tokio::test]
async fn gateway_default_request_reads_dense_cloud_and_reports_source_change() {
    let mut fixture = Fixture::start(false, false).await;
    fixture.mapping("完整点云").await;
    fixture.mapping_frames().await;
    fixture.map_ready().await;

    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .unwrap();
    let output = Command::new("python3")
        .current_dir(repo)
        .env("NUWAX_INSTALL_HOME", &fixture.root)
        .args(["-c", r#"
import json
from types import SimpleNamespace

from fastapi.testclient import TestClient

from core.config import value
from core.settings import load_config
from gateways.app import create_gateway_app
from services.geometry.client import GeometryClient

config = load_config()
geometry = GeometryClient(f"127.0.0.1:{value('geometry.runtime_port')}")
try:
    runtime = SimpleNamespace(config=config, geometry=geometry)
    app = create_gateway_app(runtime, config, start_runtime_on_lifespan=False)
    with TestClient(app) as client:
        response = client.get('/api/pointcloud/dense')
        assert response.status_code == 200, response.text
        meta = json.loads(response.headers['X-Nuwax-Pointcloud'])
        assert meta['offset_points'] == 0
        assert 0 < len(response.content) == 12 * meta['total_points'] <= 12 * 65_536
        response = client.get('/api/pointcloud/dense', params={'epoch': 'expired', 'offset': 1})
        assert response.status_code == 409, response.text
        assert response.json()['error']['code'] == 'dense_cloud_source_changed', response.text
finally:
    geometry.close()
"#])
        .output()
        .unwrap();
    assert!(output.status.success(), "{}", String::from_utf8_lossy(&output.stderr));
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
const OBSTACLE_X: f32 = 0.6;

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

/// 合成静止房间的一帧原生输入：类Mid-360扫描落在 10×8×3 米房间内，惯性只有重力。
/// 惯性样本从 `imu_origin` 起每5毫秒一个；按生产组帧规则，每帧从不晚于上一帧扫描末端
/// （帧间隔100毫秒、扫描99毫秒，即本帧起点前1毫秒）的最后一个样本取到第一个不早于本帧末端的样本。
fn native_room_frame(stamp: u64, imu_origin: u64) -> PublishPointCloudRequest {
    const SENSOR: &str = "utlidar_lidar";
    let mut directions = Vec::new();
    for azimuth in 0..180 {
        for elevation in -7..=26 {
            let (a, e) = ((azimuth * 2) as f64, (elevation * 2) as f64);
            let (a, e) = (a.to_radians(), e.to_radians());
            directions.push([e.cos() * a.cos(), e.cos() * a.sin(), e.sin()]);
        }
    }
    let span = 99_000_000_u64;
    let count = directions.len() as u64;
    let mut points = Vec::new();
    for (index, d) in directions.iter().enumerate() {
        let reach = |value: f64, low: f64, high: f64| {
            if value > 0.0 {
                high / value
            } else if value < 0.0 {
                low / value
            } else {
                f64::INFINITY
            }
        };
        let t = reach(d[0], -5.0, 5.0)
            .min(reach(d[1], -4.0, 4.0))
            .min(reach(d[2], -0.5, 2.5));
        for value in [d[0] * t, d[1] * t, d[2] * t, 100.0] {
            points.extend_from_slice(&(value as f32).to_le_bytes());
        }
        points.extend_from_slice(&((index as u64 * span / count) as u32).to_le_bytes());
    }
    let step = 5_000_000;
    let first = (stamp - 1_000_000 - imu_origin) / step;
    let last = (stamp + span - imu_origin).div_ceil(step);
    let imu_samples = (first..=last)
        .map(|index| LioImuSample {
            timestamp_ns: imu_origin + index * step,
            frame_id: SENSOR.into(),
            accel_mps2_z: 9.81,
            ..Default::default()
        })
        .collect();
    PublishPointCloudRequest {
        point_count: count as u32,
        points,
        point_step: 20,
        frame_id: SENSOR.into(),
        timestamp_ns: stamp,
        is_dense: true,
        fields: [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("offset_ns", 16, 6)]
            .into_iter()
            .map(|(name, offset, datatype)| PointCloudField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            })
            .collect(),
        imu_samples,
        ..Default::default()
    }
}

/// 输入在原生子进程订阅完成之前就开始到达：会话不应故障，订阅后里程计持续输出。
#[tokio::test]
#[ignore = "需要本机编译的原生LIO：NUWAX_NATIVE_LIO 指向 fastlio2_native 或 pointlio_native"]
async fn native_input_arriving_before_child_subscribes_does_not_fault() {
    let program = PathBuf::from(std::env::var("NUWAX_NATIVE_LIO").unwrap());
    // 子进程启动后先等1秒才执行原生程序，期间送入的帧必然早于它的订阅。
    let mut f = Fixture::start_real_native(&program, "1").await;
    f.mapping("启动竞态").await;
    let start = tokio::time::Instant::now();
    let base = now();
    let imu_origin = base - 20_000_000;
    for k in 0..50_u64 {
        tokio::time::sleep_until(start + Duration::from_millis(100 * k)).await;
        let frame = native_room_frame(base + k * 100_000_000, imu_origin);
        let result = f.client.publish_point_cloud(frame).await.unwrap().into_inner();
        assert!(result.success, "{}", result.message);
    }
    tokio::time::sleep(Duration::from_millis(300)).await;
    let state = f.state().await;
    let log = fs::read_to_string(f.root.join("进程.log")).unwrap();
    assert_eq!(state.phase, SessionPhase::Mapping as i32, "{}；{log}", state.reason);
    assert!(state.reason.is_empty(), "{}", state.reason);
    let quality = state.odometry_quality.expect("订阅完成后原生里程计应持续输出");
    assert!(
        quality.timestamp_ns >= base + 40 * 100_000_000,
        "最近一帧里程计过旧：{quality:?}"
    );
}
#[tokio::test]
async fn session_transitions_save_collision_and_direct_load() {
    let mut f = Fixture::start(false, false).await;
    assert_eq!(f.state().await.phase, SessionPhase::Idle as i32);
    assert_eq!(
        f.client.set_goal(goal(1.)).await.unwrap_err().code(),
        Code::FailedPrecondition
    );
    // 空闲阶段重定位不会被受理：入口不等帧，直接拿到会话的拒绝。
    let started = tokio::time::Instant::now();
    assert_eq!(
        f.client
            .relocalize(RelocalizeRequest::default())
            .await
            .unwrap_err()
            .code(),
        Code::FailedPrecondition
    );
    assert!(started.elapsed() < Duration::from_secs(5), "{:?}", started.elapsed());
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
    let mut source = Fixture::start(false, false).await;
    source.saved_map("重定位图").await;
    let saved_map = source.root.join("maps/重定位图");
    drop(source);
    let mut f = Fixture::start(false, false).await;
    // 新输入拥有方确实没有近期雷达帧，不再依赖加载地图清空持续输入。
    fs::create_dir_all(f.root.join("maps")).unwrap();
    fs::rename(saved_map, f.root.join("maps/重定位图")).unwrap();
    f.client
        .load_map(LoadMapRequest {
            map_name: "重定位图".into(),
        })
        .await
        .unwrap();
    // 一直没有帧：入口等满 30 秒后交给会话，返回原来的原因。
    let started = tokio::time::Instant::now();
    let failed = f
        .client
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert!(started.elapsed() >= Duration::from_secs(30), "{:?}", started.elapsed());
    assert!(!failed.succeeded);
    assert_eq!(failed.message, "最近两秒没有可用雷达帧");
    assert_eq!(f.state().await.phase, SessionPhase::Loaded as i32);
    assert!(!f.state().await.reason.is_empty());
    f.frames(5).await;
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
async fn three_dimensional_relocalize_with_seed_passes_the_grpc_entry() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("初值图").await;
    // 从建图状态取真实地图位姿，初值不再单独写死。
    let truth = f.state().await.pose.unwrap();
    let position = truth.position.as_ref().unwrap();
    let rotation = truth.orientation.as_ref().unwrap();
    let seed = PlanarSeed {
        x: position.x,
        y: position.y,
        yaw: (2.0 * (rotation.w * rotation.z + rotation.x * rotation.y))
            .atan2(1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)),
    };
    eprintln!("夹具真实位姿：{truth:?}，请求初值：{seed:?}");
    f.client.stop_session(Empty {}).await.unwrap();
    f.client
        .load_map(LoadMapRequest {
            map_name: "初值图".into(),
        })
        .await
        .unwrap();
    f.frames(5).await;
    let matched = f
        .client
        .relocalize(RelocalizeRequest {
            seed: Some(seed),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(matched.succeeded, "{matched:?}");
    assert_eq!(f.state().await.phase, SessionPhase::Localized as i32);
}

#[tokio::test]
async fn relocalize_right_after_load_waits_for_frames_that_arrive_later() {
    let mut source = Fixture::start(false, false).await;
    source.saved_map("等帧图").await;
    let saved_map = source.root.join("maps/等帧图");
    drop(source);
    // 新进程没有任何雷达输入。
    let mut f = Fixture::start(false, false).await;
    fs::create_dir_all(f.root.join("maps")).unwrap();
    fs::rename(saved_map, f.root.join("maps/等帧图")).unwrap();
    f.client
        .load_map(LoadMapRequest {
            map_name: "等帧图".into(),
        })
        .await
        .unwrap();
    let mut client = f.client.clone();
    let started = tokio::time::Instant::now();
    let relocalize =
        tokio::spawn(async move { client.relocalize(RelocalizeRequest::default()).await });
    tokio::time::sleep(Duration::from_secs(1)).await;
    while !relocalize.is_finished() {
        assert!(started.elapsed() < Duration::from_secs(10), "帧到达后重定位仍未返回");
        f.frame(now()).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let matched = relocalize.await.unwrap().unwrap().into_inner();
    assert!(matched.succeeded, "{matched:?}");
    assert!(matched.frames_used >= 5, "{matched:?}");
    assert_eq!(f.state().await.phase, SessionPhase::Localized as i32);
}

#[tokio::test]
async fn l6_loaded_map_publishes_and_saves_new_structure_through_grpc() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("更新图").await;
    f.client.stop_session(Empty {}).await.unwrap();
    let load = LoadMapRequest {
        map_name: "更新图".into(),
    };
    f.client.load_map(load.clone()).await.unwrap();
    f.map_ready().await;
    assert_eq!(f.obstacle_cells().await, 0);
    assert_eq!(f.obstacle_points().await, 0);
    // 障碍在加载前已摆好：加载后的每一帧都带着它。
    let stamp = now();
    for index in 0..5 {
        f.obstacle_frame(stamp + index * 100_000_000).await;
    }
    tokio::time::sleep(Duration::from_millis(100)).await;
    let matched = f
        .client
        .relocalize(RelocalizeRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert!(matched.succeeded, "{matched:?}");
    assert_eq!(f.state().await.phase, SessionPhase::Localized as i32);
    assert_eq!(f.obstacle_cells().await, 0, "重定位成功时还没有已确认结构");
    let started = tokio::time::Instant::now();
    while f.obstacle_cells().await == 0 {
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "已定位后新障碍未写入地图，状态{:?}，日志{}",
            f.state().await,
            fs::read_to_string(f.root.join("进程.log")).unwrap()
        );
        f.obstacle_frame(now()).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let saved = f
        .client
        .save_map(SaveMapRequest::default())
        .await
        .unwrap()
        .into_inner();
    assert!(saved.replaced);
    assert_eq!(saved.map_name, "更新图");
    assert_eq!(f.state().await.phase, SessionPhase::Localized as i32);
    f.client.stop_session(Empty {}).await.unwrap();
    f.client.load_map(load).await.unwrap();
    f.map_ready().await;
    assert!(f.obstacle_cells().await > 0, "重新加载的二维栅格应有新障碍");
    assert_eq!(f.obstacle_points().await, 0, "加载后新增结构不进入稠密原扫描");
}

#[tokio::test]
async fn native_child_kill_faults_session_without_exiting_process() {
    let mut f = Fixture::start(true, false).await;
    f.mapping("故障前").await;
    let parent = f.child.id();
    tokio::time::timeout(Duration::from_secs(3), async {
        while !f.root.join("native.pid").exists() {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
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
async fn pgo_saved_map_relocalizes_after_load() {
    let mut f = Fixture::start(false, true).await;
    f.saved_map("回环图").await;
    f.client.stop_session(Empty {}).await.unwrap();
    let loaded = f.client
        .load_map(LoadMapRequest {
            map_name: "回环图".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(loaded.phase, SessionPhase::Loaded as i32);
    f.frames(5).await;
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
    assert!(!f.root.join(nuwax_config::GEOMETRY_READY).exists());
    assert!(states.message().await.unwrap().is_none());
    assert!(poses.message().await.unwrap().is_none());
}

#[tokio::test]
async fn failed_start_and_load_preserve_current_unsaved_map() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("已存在").await;
    f.mapping("未保存").await;
    f.mapping_frames().await;
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
async fn malformed_forbidden_file_waits_without_ending_exploration_and_recovers() {
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
            .is_some_and(|s| s.message.contains("禁行区格式无效"))
        {
            assert_eq!(state.phase, SessionPhase::Exploring as i32);
            let exploration = state.exploration.unwrap();
            assert_eq!(exploration.phase, ExplorationPhase::Running as i32);
            let reason = exploration.message;
            assert!(reason.contains("禁行区格式无效"), "{reason}");
            break;
        }
        assert!(start.elapsed() < Duration::from_secs(3));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    fs::write(f.root.join("maps/禁区错误/forbidden_zones.json"), b"[]").unwrap();
    let start = tokio::time::Instant::now();
    loop {
        let state = f.state().await;
        assert_eq!(state.phase, SessionPhase::Exploring as i32);
        let exploration = state.exploration.unwrap();
        assert_eq!(exploration.phase, ExplorationPhase::Running as i32);
        if !exploration.message.contains("禁行区") { break; }
        assert!(start.elapsed() < Duration::from_secs(3));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
    f.frame(now()).await;
}

#[tokio::test]
async fn forbidden_file_errors_reach_the_goal_and_recover_on_the_next_check() {
    let mut f = Fixture::start(false, false).await;
    f.saved_map("禁区目标").await;
    let raw = f
        .client
        .get_occupancy_grid(GetOccupancyGridRequest {
            include_forbidden: false,
        })
        .await
        .unwrap()
        .into_inner();
    let mut states = f
        .client
        .subscribe_state(Empty {})
        .await
        .unwrap()
        .into_inner();
    let goal_id = f
        .client
        .set_goal(goal(2.))
        .await
        .unwrap()
        .into_inner()
        .goal_id;
    let path = f.root.join("maps/禁区目标/forbidden_zones.json");
    fs::write(&path, "{错误").unwrap();
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let state = states.message().await.unwrap().unwrap();
            assert_eq!(state.phase, SessionPhase::Mapping as i32);
            if state.navigation.as_ref().is_some_and(|goal| {
                goal.goal_id == goal_id && goal.message.contains("禁行区格式无效")
            }) {
                break;
            }
        }
    })
    .await
    .unwrap();
    fs::remove_file(&path).unwrap();
    fs::create_dir(&path).unwrap();
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let state = states.message().await.unwrap().unwrap();
            assert_eq!(state.phase, SessionPhase::Mapping as i32);
            if state.navigation.as_ref().is_some_and(|goal| {
                goal.goal_id == goal_id && goal.message.contains("禁行区读取失败")
            }) {
                break;
            }
        }
    })
    .await
    .unwrap();
    let mut maps = f.client.subscribe_map(Empty {}).await.unwrap().into_inner();
    fs::remove_dir(&path).unwrap();
    let replacement = path.with_extension("tmp");
    fs::write(&replacement, br#"[]"#).unwrap();
    fs::rename(replacement, path).unwrap();
    let restored = tokio::time::timeout(Duration::from_secs(3), maps.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert_eq!(restored.raw_data, raw.data);
    assert_eq!(restored.data, raw.data);
    assert_eq!(
        f.client
            .get_occupancy_grid(GetOccupancyGridRequest {
                include_forbidden: false
            })
            .await
            .unwrap()
            .into_inner(),
        raw
    );
    f.client.save_map(SaveMapRequest::default()).await.unwrap();
}

#[tokio::test]
async fn initial_lio_start_failure_can_stop_then_start_after_executable_is_restored() {
    let mut f = Fixture::start_with_missing_lio(true, false, true).await;
    assert_eq!(f.state().await.phase, SessionPhase::Fault as i32);
    f.client.stop_session(Empty {}).await.unwrap();
    fs::rename(
        f.root.join("原生进程夹具"),
        f.root.join("bin/fastlio2_native"),
    )
    .unwrap();
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

#[test]
fn release_artifacts_discovery_exits_before_runtime_or_storage_setup() {
    let folder =
        std::env::temp_dir().join(format!("neh-artifact-discovery-{}", uuid::Uuid::new_v4()));
    std::fs::create_dir(&folder).unwrap();
    let ready = folder.join(nuwax_config::GEOMETRY_READY);
    std::fs::create_dir_all(ready.parent().unwrap()).unwrap();
    std::fs::write(&ready, b"existing-owner").unwrap();
    let mut config = nuwax_config::test_support::defaults();
    config.set("geometry", "fine_cloud_cache_mib", "invalid-cache-setting");
    config.set("geometry", "lio_backend", "invalid-input-backend");
    config.write_install(&folder);
    let output = Command::new(env!("CARGO_BIN_EXE_nuwax-geometry-runtime"))
        .arg("--print-release-artifacts")
        .env("NUWAX_INSTALL_HOME", &folder)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "只读发现应直接成功：{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(std::fs::read(&ready).unwrap(), b"existing-owner");
    let payload: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(
        payload["schema_version"],
        "nuwax.geometry.embedded-artifacts.v1"
    );
    let artifacts = payload["artifacts"].as_array().unwrap();
    assert_eq!(artifacts.len(), 2);
    assert_eq!(artifacts[0]["role"], "native_pgo_payload");
    assert_eq!(artifacts[1]["role"], "group_guardian");
    assert_eq!(
        artifacts[0]["path"].as_str().unwrap(),
        nuwax_nav_pgo_bridge::discovery_native_executable()
            .to_str()
            .unwrap()
    );
    assert_eq!(
        artifacts[1]["path"].as_str().unwrap(),
        nuwax_nav_pgo_bridge::discovery_group_guardian_executable()
            .to_str()
            .unwrap()
    );
    std::fs::remove_dir_all(folder).unwrap();
}
