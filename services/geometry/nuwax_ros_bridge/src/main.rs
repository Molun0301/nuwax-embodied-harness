mod pointcloud;
mod ros2_bridge;
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
        .with_env_filter("nuwax_ros_bridge=info")
        .with_target(false)
        .init();

    let port: u16 = std::env::var("NUWAX_ROS_BRIDGE_PORT")
        .unwrap_or_else(|_| "50051".into())
        .parse()
        .unwrap_or(50051);
    let odom_topic = env_first(&["NUWAX_ODOM_TOPIC"], "/odom");
    let pointcloud_topic = env_first(&["NUWAX_POINTCLOUD_TOPIC"], "/utlidar/cloud_base");
    let map_topic = env_first(&["NUWAX_MAP_TOPIC", "NUWAX_GEOMETRY_OCCUPANCY_TOPIC"], "/map");
    let tf_topic = env_first(&["NUWAX_TF_TOPIC", "NUWAX_GEOMETRY_TF_TOPIC"], "/tf");
    let nav_action = env_first(&["NUWAX_NAV_ACTION", "NUWAX_NAV2_ACTION_NAME"], "/navigate_to_pose");
    let odom_source_topic = env_first(
        &["NUWAX_GO2_ODOM_SOURCE", "NUWAX_GEOMETRY_ODOM_TOPIC"],
        "/utlidar/robot_odom",
    );
    let pointcloud_source_topic = env_first(
        &["NUWAX_GO2_POINTCLOUD_SOURCE", "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC"],
        "/utlidar/cloud",
    );
    let direct_subscribe = std::env::var("NUWAX_GO2_DIRECT_SUBSCRIBE")
        .unwrap_or_else(|_| "true".into())
        .parse::<bool>()
        .unwrap_or(true);

    let video_shm_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_video.shm".into())
    );
    let video_uds_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_UDS_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_video.sock".into())
    );

    let addr: SocketAddr = format!("127.0.0.1:{}", port).parse()?;

    info!("启动 nuwax-ros-bridge, addr: {}", addr);

    // 启动视频帧 UDS 中转
    let _video_handle = video_relay::spawn_video_relay(
        &video_shm_path, &video_uds_path,
    )?;
    info!("视频中转已启动: shm={:?}, uds={:?}", video_shm_path, video_uds_path);

    let odom_src = if direct_subscribe { Some(odom_source_topic.as_str()) } else { None };
    let pc_src = if direct_subscribe { Some(pointcloud_source_topic.as_str()) } else { None };
    let bridge = ros2_bridge::spawn_ros2_bridge(
        &odom_topic, &pointcloud_topic, &map_topic, &tf_topic, &nav_action,
        odom_src, pc_src,
    )?;

    let service = server::GeometryRuntimeService::new(bridge);

    // 写 ready 文件通知父进程
    let ready_path = std::env::var("NUWAX_ROS_BRIDGE_READY_FILE")
        .unwrap_or_else(|_| format!("/tmp/nuwax_ros_bridge_{}.ready", std::process::id()));
    std::fs::write(&ready_path, format!("{}", port))?;

    info!("gRPC server 就绪，等待连接...");

    Server::builder()
        .add_service(proto::geometry_runtime_server::GeometryRuntimeServer::new(service))
        .serve(addr)
        .await?;

    Ok(())
}
