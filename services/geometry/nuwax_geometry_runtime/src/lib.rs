//! Geometry（几何运行时）：一个会话拥有方、持续输入、直接地图目录。

pub mod proto {
    tonic::include_proto!("nuwax.geometry.v3");
}
pub mod api;
mod channel_registry;
mod completed_cleanup;
mod corrector;
mod dense_cloud;
mod dynamic_backend;
mod dynamic_scene;
mod explore_mission;
mod explorer;
mod forbidden_zones;
mod lio;
mod lio_ingress;
mod lio_owner;
mod map_store;
mod mapper;
mod mapping_backend;
mod native_input_delivery;
mod native_lio_owner;
mod nav_bridge;
mod outputs;
mod persistent_occupancy;
mod pointcloud;
mod rpc_runtime;
mod self_cloud;
pub mod session;
mod slam2d_owner;

use std::{path::PathBuf, process::ExitCode};
use tokio::net::TcpListener;
use tonic::transport::Server;

struct ReadyFile(PathBuf);
impl Drop for ReadyFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let port: u16 = std::env::var("NUWAX_GEOMETRY_RUNTIME_PORT")
        .unwrap_or_else(|_| "50051".into())
        .parse()?;
    let ready_path = PathBuf::from(
        std::env::var("NUWAX_GEOMETRY_RUNTIME_READY_FILE")
            .unwrap_or_else(|_| "runtime_data/geometry/nuwax_geometry_runtime.ready".into()),
    );
    if let Some(parent) = ready_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let _ = std::fs::remove_file(&ready_path);
    let ready = ReadyFile(ready_path.clone());
    let listener = TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, port)).await?;
    let session = session::SessionHandle::start().await?;
    let rpc = rpc_runtime::start(session.clone()).await?;
    let outputs = outputs::Outputs::start(session.clone())?;
    let api = api::GeometryApi::new(session.clone());
    let incoming = futures::stream::unfold(
        (listener, Some(ready_path)),
        move |(listener, ready_path)| async move {
            if let Some(path) = ready_path {
                if let Err(error) =
                    std::fs::write(path, format!("pid={}\nport={port}\n", std::process::id()))
                {
                    return Some((Err(error), (listener, None)));
                }
            }
            Some((
                listener.accept().await.map(|(stream, _)| stream),
                (listener, None),
            ))
        },
    );
    let shutdown = session.clone();
    let result = Server::builder()
        .add_service(
            proto::geometry_runtime_server::GeometryRuntimeServer::new(api)
                .max_decoding_message_size(128 * 1024 * 1024)
                .max_encoding_message_size(128 * 1024 * 1024),
        )
        .serve_with_incoming_shutdown(incoming, async move {
            shutdown_signal().await;
            drop(ready);
            shutdown.shutdown().await;
        })
        .await;
    outputs.shutdown().await;
    rpc.stop().await?;
    session.shutdown().await;
    result?;
    Ok(())
}

async fn shutdown_signal() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        let mut terminate = signal(SignalKind::terminate()).expect("注册终止信号");
        tokio::select! { _ = tokio::signal::ctrl_c() => {}, _ = terminate.recv() => {} }
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
    }
}

pub fn main() -> ExitCode {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .try_init();
    let cache = pointcloud::parse_fine_cloud_cache(
        std::env::var("NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB")
            .ok()
            .as_deref(),
        std::env::var("NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB")
            .ok()
            .as_deref(),
    );
    if let Err(reason) = cache.and_then(pointcloud::configure_fine_cloud_cache) {
        eprintln!("{reason}");
        return ExitCode::FAILURE;
    }
    if let Some(result) = completed_cleanup::run_requested() {
        return match result {
            Ok(()) => ExitCode::SUCCESS,
            Err(reason) => {
                eprintln!("{reason}");
                ExitCode::FAILURE
            }
        };
    }
    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("创建运行时失败：{error}");
            return ExitCode::FAILURE;
        }
    };
    match runtime.block_on(run()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("Geometry 启动或运行失败：{error}");
            ExitCode::FAILURE
        }
    }
}
