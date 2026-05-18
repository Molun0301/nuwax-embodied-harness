mod ros2_bridge;
mod server;

use std::net::SocketAddr;
use tonic::transport::Server;
use tracing::info;

pub mod proto {
    tonic::include_proto!("nuwax.geometry");
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
    let odom_topic = std::env::var("NUWAX_ODOM_TOPIC")
        .unwrap_or_else(|_| "/odom".into());
    let pointcloud_topic = std::env::var("NUWAX_POINTCLOUD_TOPIC")
        .unwrap_or_else(|_| "/utlidar/cloud".into());
    let map_topic = std::env::var("NUWAX_MAP_TOPIC")
        .unwrap_or_else(|_| "/map".into());
    let tf_topic = std::env::var("NUWAX_TF_TOPIC")
        .unwrap_or_else(|_| "/tf".into());
    let nav_action = std::env::var("NUWAX_NAV_ACTION")
        .unwrap_or_else(|_| "/navigate_to_pose".into());

    let addr: SocketAddr = format!("127.0.0.1:{}", port).parse()?;

    info!("启动 nuwax-ros-bridge, addr: {}", addr);

    let bridge = ros2_bridge::spawn_ros2_bridge(
        &odom_topic, &pointcloud_topic, &map_topic, &tf_topic, &nav_action
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
