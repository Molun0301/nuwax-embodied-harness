//! nuwax-geometry-runtime：LCM-first 几何 native runtime 子进程入口
//!
//! 替换旧 `nuwax-ros-bridge`：删除 ROS2 env/r2r/DDS 依赖；改为读
//! `NUWAX_GEOMETRY_RUNTIME_*` env + nuwax-workspace LCM channel。
//!
//! 启动顺序：
//! 1. tracing 初始化
//! 2. 解析 env：port、LCM channel 名、video shm/uds 路径、ready file 路径
//! 3. 启动 video_relay（HTTP/UDS→SHM，与 LCM 无关的视频中转）
//! 4. 启动 `lcm_runtime::spawn_geometry_runtime`（fake backend）
//! 5. 写 ready file（通知父进程 / systemd）
//! 6. tonic gRPC server 监听

mod lcm_runtime;
mod pointcloud;
mod server;
mod video_relay;

use std::net::SocketAddr;
use std::path::PathBuf;
use tonic::transport::Server;
use tracing::info;

pub mod proto {
    tonic::include_proto!("nuwax.geometry");
}

fn env_first(names: &[&str], default_value: &str) -> String {
    for name in names {
        if let Ok(value) = std::env::var(name) {
            if !value.trim().is_empty() {
                return value;
            }
        }
    }
    default_value.into()
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter("nuwax_geometry_runtime=info,nuwax_protocol=warn,nuwax_transport=warn,nuwax_codec=warn,nuwax_tf=warn")
        .with_target(false)
        .init();

    // ----------------------------------------------------------------------
    // env 解析（替换旧 NUWAX_ROS_BRIDGE_*）
    // ----------------------------------------------------------------------
    let port: u16 = std::env::var("NUWAX_GEOMETRY_RUNTIME_PORT")
        .unwrap_or_else(|_| "50051".into())
        .parse()
        .unwrap_or(50051);

    // LCM channel 名称（typed channel `/topic#msg_type`）
    let occupancy_channel = env_first(
        &["NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL"],
        "/map/occupancy#nav_msgs.OccupancyGrid",
    );
    let tf_channel = env_first(
        &["NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL"],
        "/tf#geometry_msgs.Transform",
    );
    let cloud_map_channel = std::env::var("NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL").ok()
        .filter(|s| !s.trim().is_empty())
        .or_else(|| Some("/go2/lidar/points#sensor_msgs.PointCloud2".into()));
    let nav_state_channel = std::env::var("NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL").ok()
        .filter(|s| !s.trim().is_empty())
        .or_else(|| Some("/nav/state#nuwax_msgs.NavigationState".into()));
    let frontiers_channel = std::env::var("NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL").ok()
        .filter(|s| !s.trim().is_empty())
        .or_else(|| Some("/explore/frontiers#nuwax_msgs.FrontierList".into()));

    let video_shm_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_video.shm".into())
    );
    let video_uds_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_UDS_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_video.sock".into())
    );

    let ready_path = std::env::var("NUWAX_GEOMETRY_RUNTIME_READY_FILE")
        .unwrap_or_else(|_| format!("/tmp/nuwax_geometry_runtime_{}.ready", std::process::id()));

    let addr: SocketAddr = format!("127.0.0.1:{}", port).parse()?;

    info!("启动 nuwax-geometry-runtime, addr: {}", addr);

    // ----------------------------------------------------------------------
    // video relay（保留——视频中转不属于 LCM/ROS2 范畴）
    // ----------------------------------------------------------------------
    let _video_handle = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path)?;
    info!("视频中转已启动: shm={:?}, uds={:?}", video_shm_path, video_uds_path);

    // ----------------------------------------------------------------------
    // LCM runtime state loop
    // ----------------------------------------------------------------------
    let cm_chan_for_runtime = cloud_map_channel.clone();
    let bridge = lcm_runtime::spawn_geometry_runtime(
        &occupancy_channel,
        &tf_channel,
        cm_chan_for_runtime.as_deref(),
        nav_state_channel.as_deref(),
        frontiers_channel.as_deref(),
    );

    let service = server::GeometryRuntimeService::new(bridge);

    // ----------------------------------------------------------------------
    // ready file
    // ----------------------------------------------------------------------
    std::fs::write(&ready_path, format!("{}", port))?;
    info!("ready file 写入: {:?}", ready_path);

    info!("gRPC server 就绪，等待连接...");

    Server::builder()
        .add_service(proto::geometry_runtime_server::GeometryRuntimeServer::new(service))
        .serve(addr)
        .await?;

    Ok(())
}