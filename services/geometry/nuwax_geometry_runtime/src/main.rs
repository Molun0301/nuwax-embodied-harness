//! nuwax-geometry-runtime：LCM-first 几何 native runtime 子进程入口
//!
//! 替换旧 `nuwax-ros-bridge`：删除 ROS2 env/r2r/DDS 依赖；改为读
//! `NUWAX_GEOMETRY_RUNTIME_*` env + nuwax-workspace LCM channel。
//!
//! 启动顺序：
//! 1. tracing 初始化
//! 2. 解析 env：port、LCM channel 名、video shm/uds 路径、ready file 路径
//! 3. 启动 video_relay（HTTP/UDS→SHM，与 LCM 无关的视频中转）
//! 4. 启动 `lcm_runtime::spawn_geometry_runtime`（fake backend 默认;
//!    real-lcm 启动期 fail-fast 返回 `LCM_BACKEND_UNAVAILABLE`）
//! 5. 写 ready file（通知父进程 / systemd）
//! 6. tonic gRPC server 监听

mod channel_registry;
mod lcm_runtime;
mod pointcloud;
mod runtime_diagnostics;
mod server;
mod typed_codec;
mod video_relay;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::process::ExitCode;
use tonic::transport::Server;
use tracing::{error, info};

pub mod proto {
    tonic::include_proto!("nuwax.geometry");
}

#[tokio::main]
async fn main() -> ExitCode {
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
    // Phase 2 收敛:env 探测统一走 channel_registry::resolve_channel,
    // 探测顺序 = NUWAX_GEOMETRY_RUNTIME_<X>_CHANNEL → NUWAX_LCM_CHANNEL_<X> → default
    let occupancy_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL",
        "NUWAX_LCM_CHANNEL_OCCUPANCY",
        "/map/occupancy#nav_msgs.OccupancyGrid",
    );
    let tf_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL",
        "NUWAX_LCM_CHANNEL_TF",
        "/tf#geometry_msgs.Transform",
    );
    let cloud_map_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
        "NUWAX_LCM_CHANNEL_CLOUD_MAP",
        "/go2/lidar/points#sensor_msgs.PointCloud2",
    );
    let nav_state_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
        "NUWAX_LCM_CHANNEL_GOAL_POSE",
        "/nav/state#nuwax_msgs.NavigationState",
    );
    let frontiers_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL",
        "NUWAX_LCM_CHANNEL_FRONTIER",
        "/explore/frontiers#nuwax_msgs.FrontierList",
    );
    let pose_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_POSE_CHANNEL",
        "NUWAX_LCM_CHANNEL_POSE_STAMPED",
        "/pose_stamped#geometry_msgs.PoseStamped",
    );
    let odometry_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
        "NUWAX_LCM_CHANNEL_ODOM",
        "/odom#nav_msgs.Odometry",
    );
    let cmd_vel_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_CMD_VEL",
        "/cmd_vel#geometry_msgs.Twist",
    );
    let nav_cancel_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel#nuwax_msgs.CancelNavigation",
    );

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

    let addr: SocketAddr = match format!("127.0.0.1:{}", port).parse() {
        Ok(a) => a,
        Err(e) => {
            error!("gRPC bind addr 解析失败: {}", e);
            return ExitCode::from(2);
        }
    };

    info!("启动 nuwax-geometry-runtime, addr: {}", addr);

    // ----------------------------------------------------------------------
    // video relay（保留——视频中转不属于 LCM/ROS2 范畴）
    // ----------------------------------------------------------------------
    if let Err(e) = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path) {
        error!("视频中转启动失败: {}", e);
        return ExitCode::from(3);
    }
    info!("视频中转已启动: shm={:?}, uds={:?}", video_shm_path, video_uds_path);

    // ----------------------------------------------------------------------
    // LCM runtime state loop
    // ----------------------------------------------------------------------
    let cm_chan_for_runtime = cloud_map_channel.clone();
    let bridge = match lcm_runtime::spawn_geometry_runtime(
        &occupancy_channel,
        &tf_channel,
        Some(&cm_chan_for_runtime),
        Some(&nav_state_channel),
        Some(&frontiers_channel),
        Some(&pose_channel),
        Some(&odometry_channel),
        Some(&cmd_vel_channel),
        Some(&nav_cancel_channel),
    ) {
        Ok(b) => b,
        Err(e) => {
            // Phase 1 fail-fast:real-lcm 请求启动期直接拒绝;
            // 不在 Real 失败路径 fallback fake-lcm。
            // 错误信息在 stderr 输出 "LCM_BACKEND_UNAVAILABLE" 关键字,
            // 供 start 脚本与 smoke test grep 识别。
            error!("{}", e);
            eprintln!("LCM_BACKEND_UNAVAILABLE: real-lcm backend requested but upstream dimos \
                       nuwax-transport/real-lcm feature is still a stub. Process will exit.");
            return ExitCode::from(4);
        }
    };

    let service = server::GeometryRuntimeService::new(bridge);

    // ----------------------------------------------------------------------
    // ready file
    // ----------------------------------------------------------------------
    if let Err(e) = std::fs::write(&ready_path, format!("{}", port)) {
        error!("ready file 写入失败: {:?}", e);
        return ExitCode::from(5);
    }
    info!("ready file 写入: {:?}", ready_path);

    info!("gRPC server 就绪，等待连接...");

    if let Err(e) = Server::builder()
        .add_service(proto::geometry_runtime_server::GeometryRuntimeServer::new(service))
        .serve(addr)
        .await
    {
        error!("gRPC server 退出: {}", e);
        return ExitCode::from(6);
    }
    ExitCode::from(0)
}