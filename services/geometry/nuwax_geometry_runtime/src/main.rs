// Phase 6 (codex-round3 R3-4)：main.rs line 113/165 同样被 `uninlined_format_args`
// 报告,风格性 lint,统一 file-level 标注。
#![allow(clippy::uninlined_format_args)]

//! nuwax-geometry-runtime：LCM-first 几何 native runtime 子进程入口
//!
//! 读取 `NUWAX_GEOMETRY_RUNTIME_*` env + nuwax-workspace LCM channel。
//!
//! 启动顺序：
//! 1. 校验 `NUWAX_LCM_BACKEND`，非法值在任何启动副作用前退出
//! 2. tracing 初始化
//! 3. 解析 env：port、LCM channel 名、video shm/uds 路径、ready file 路径
//! 4. 预绑定 gRPC listener（端口占用立即 fail-fast）
//! 5. 启动 video_relay（HTTP/UDS→SHM，与 LCM 无关的视频中转）
//! 6. 启动 `lcm_runtime::spawn_geometry_runtime`（real-lcm 启动失败立即退出）
//! 7. 在同一 LCM owner context 上注册 module-style RPC，再启动 tonic server
//! 8. 自检 HealthCheck 后写 ready file
//! 9. Stop RPC 关闭 listener，确认 LCM unsubscribe 后退出进程

// Phase 4: `pub` 而非私有 —— 兄弟模块(lcm_runtime/server)需要 `crate::channel_registry::*`。
pub mod channel_registry;
mod explore_runtime;
mod lcm_runtime;
// W41 (Bug #2 fix): `map_checksum` 模块是 W37 (RALPLAN §Phase M #3) 新增
// 的 SHA-256 helper,供 `mapping_adapter::build_and_publish` 算 checksum
// 并通过 `MapRevisionEvent.checksum` 广播;untracked 状态下 main.rs 漏
// 声明,补回 `mod map_checksum;`。
mod map_checksum;
mod mapping_adapter;
mod nav_runtime;
mod occupancy_shm;
mod pointcloud;
mod rpc_runtime;
mod runtime_diagnostics;
mod sequence_state;
mod server;
mod typed_codec;
mod video_relay;

use futures::stream;
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::{TcpListener, TcpStream};
use tonic::transport::Server;
use tracing::{error, info};

const GRPC_MESSAGE_LIMIT_BYTES: usize = 128 * 1024 * 1024;

pub mod proto {
    tonic::include_proto!("nuwax.geometry");
}

fn incoming_until_shutdown(
    listener: TcpListener,
    shutdown: server::ShutdownListener,
) -> (
    impl futures::Stream<Item = Result<TcpStream, io::Error>> + Send + 'static,
    tokio::sync::watch::Receiver<bool>,
) {
    let (request_rx, listener_closed_tx, graceful_rx) = shutdown.into_parts();
    let incoming = stream::unfold(
        Some((listener, request_rx, listener_closed_tx)),
        |state| async move {
            let (listener, mut request_rx, listener_closed_tx) = state?;
            if *request_rx.borrow() {
                drop(listener);
                listener_closed_tx.send_replace(true);
                return std::future::pending().await;
            }

            tokio::select! {
                biased;
                changed = request_rx.changed() => {
                    if changed.is_err() || *request_rx.borrow() {
                        drop(listener);
                        listener_closed_tx.send_replace(true);
                        std::future::pending().await
                    } else {
                        Some((
                            Err(io::Error::new(
                                io::ErrorKind::Interrupted,
                                "spurious shutdown channel update",
                            )),
                            Some((listener, request_rx, listener_closed_tx)),
                        ))
                    }
                }
                accepted = listener.accept() => {
                    let item = accepted.map(|(stream, _peer)| stream);
                    Some((item, Some((listener, request_rx, listener_closed_tx))))
                }
            }
        },
    );
    (incoming, graceful_rx)
}

async fn wait_for_grpc_health(addr: SocketAddr) -> Result<(), String> {
    let endpoint = format!("http://{addr}");
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if let Ok(mut client) =
                proto::geometry_runtime_client::GeometryRuntimeClient::connect(endpoint.clone())
                    .await
            {
                if let Ok(response) = client
                    .health_check(proto::HealthCheckRequest {})
                    .await
                    .map(tonic::Response::into_inner)
                {
                    if response.healthy && response.phase == "running" {
                        return Ok(());
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .map_err(|_| format!("gRPC HealthCheck 在 {addr} 上 5s 内未就绪"))?
}

fn remove_ready_file(path: &str) -> io::Result<()> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error),
    }
}

fn main() -> ExitCode {
    let lcm_backend = match lcm_runtime::LcmBackendKind::from_env() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(1);
        }
    };

    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("geometry runtime 初始化 Tokio 失败: {error}");
            return ExitCode::from(2);
        }
    };
    runtime.block_on(run(lcm_backend))
}

async fn run(lcm_backend: lcm_runtime::LcmBackendKind) -> ExitCode {
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
        "/tf#tf2_msgs.TFMessage",
    );
    let cloud_map_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
        "NUWAX_LCM_CHANNEL_CLOUD_MAP",
        "/cloud_map#sensor_msgs.PointCloud2",
    );
    let nav_state_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
        // Phase 4 修复:platform alias 从 `NUWAX_LCM_CHANNEL_GOAL_POSE` 改为
        // `NUWAX_LCM_CHANNEL_NAV_STATE`,与 channel_registry::TYPED_CHANNELS
        // 对齐。历史误用让运维改 nav_state channel 时改到 goal_pose env。
        "NUWAX_LCM_CHANNEL_NAV_STATE",
        "/nav/state#nuwax_msgs.NavigationState",
    );
    let frontiers_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL",
        "NUWAX_LCM_CHANNEL_FRONTIER",
        "/explore/frontiers#nuwax_msgs.FrontierList",
    );
    let odometry_channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
        "NUWAX_LCM_CHANNEL_ODOM",
        "/odom#nav_msgs.Odometry",
    );
    let video_shm_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_SHM_PATH").unwrap_or_else(|_| "/tmp/nuwax_video.shm".into()),
    );
    let video_uds_path = PathBuf::from(
        std::env::var("NUWAX_VIDEO_UDS_PATH").unwrap_or_else(|_| "/tmp/nuwax_video.sock".into()),
    );

    let ready_path = std::env::var("NUWAX_GEOMETRY_RUNTIME_READY_FILE").unwrap_or_else(|_| {
        let runtime_dir =
            std::env::var("NUWAX_RUNTIME_DATA_DIR").unwrap_or_else(|_| "runtime_data".into());
        format!("{runtime_dir}/geometry/nuwax_geometry_runtime.ready")
    });

    if let Err(e) = remove_ready_file(&ready_path) {
        error!("清理旧 ready file 失败: {:?}", e);
        return ExitCode::from(5);
    }

    let addr: SocketAddr = match format!("127.0.0.1:{}", port).parse() {
        Ok(a) => a,
        Err(e) => {
            error!("gRPC bind addr 解析失败: {}", e);
            return ExitCode::from(2);
        }
    };

    info!("启动 nuwax-geometry-runtime, addr: {}", addr);

    let listener = match TcpListener::bind(addr).await {
        Ok(listener) => listener,
        Err(e) => {
            error!("gRPC bind 失败 {}: {}", addr, e);
            return ExitCode::from(6);
        }
    };

    // ----------------------------------------------------------------------
    // video relay
    // ----------------------------------------------------------------------
    if let Err(e) = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path) {
        error!("视频中转启动失败: {}", e);
        return ExitCode::from(3);
    }
    info!(
        "视频中转已启动: shm={:?}, uds={:?}",
        video_shm_path, video_uds_path
    );

    // ----------------------------------------------------------------------
    // LCM runtime state loop
    // ----------------------------------------------------------------------
    let cm_chan_for_runtime = cloud_map_channel.clone();
    // ^ 保留这条 eprintln 不动 — main.rs:149 是上游异常分支,不属于崩溃链
    let bridge = match lcm_runtime::spawn_geometry_runtime(
        lcm_backend,
        &occupancy_channel,
        &tf_channel,
        Some(&cm_chan_for_runtime),
        Some(&nav_state_channel),
        Some(&frontiers_channel),
        Some(&odometry_channel),
    ) {
        Ok(b) => b,
        Err(e) => {
            // real-lcm 启动失败时直接 fail-fast(exit code = 4)。
            // 错误信息在 stderr 输出 "LCM_BACKEND_UNAVAILABLE" 关键字,
            // 供 start 脚本与 smoke test grep 识别。
            error!("{}", e);
            eprintln!(
                "LCM_BACKEND_UNAVAILABLE: real-lcm backend requested but liblcm 1.3~1.5 \
                       missing (apt-get install liblcm-dev). Process will exit."
            );
            return ExitCode::from(4);
        }
    };

    let (shutdown, shutdown_listener) = server::shutdown_channel();
    let rpc_context = match bridge.transport.shared_lcm_context() {
        Some(context) => context,
        None => {
            error!("Geometry LCM transport 未暴露真实 owner context，RPC 无法启动");
            return ExitCode::from(7);
        }
    };
    let service = server::GeometryRuntimeService::new_with_shutdown(bridge, shutdown.clone());
    let rpc_transport: Arc<dyn nuwax_rpc::RpcTransport> =
        Arc::new(nuwax_rpc::LcmRpcTransport::from_context(rpc_context));
    let rpc_servers = match rpc_runtime::serve_geometry_rpc(service.clone(), rpc_transport).await {
        Ok(servers) => servers,
        Err(e) => {
            error!("Geometry LCM RPC server 启动失败: {}", e);
            return ExitCode::from(7);
        }
    };
    info!(
        "Geometry LCM RPC 已启动: backend={}, methods={}",
        rpc_servers.backend(),
        rpc_servers.len()
    );
    let (incoming, mut graceful_rx) = incoming_until_shutdown(listener, shutdown_listener);
    let graceful_shutdown = async move {
        if !*graceful_rx.borrow() {
            let _ = graceful_rx.wait_for(|requested| *requested).await;
        }
    };
    let mut server_task = tokio::spawn(async move {
        Server::builder()
            .add_service(
                proto::geometry_runtime_server::GeometryRuntimeServer::new(service)
                    .max_decoding_message_size(GRPC_MESSAGE_LIMIT_BYTES)
                    .max_encoding_message_size(GRPC_MESSAGE_LIMIT_BYTES),
            )
            .serve_with_incoming_shutdown(incoming, graceful_shutdown)
            .await
    });

    let startup_check = wait_for_grpc_health(addr);
    tokio::pin!(startup_check);
    tokio::select! {
        check = &mut startup_check => {
            if let Err(e) = check {
                error!("{}", e);
                shutdown.request_shutdown();
                server_task.abort();
                let _ = (&mut server_task).await;
                if let Err(error) = rpc_servers.stop().await {
                    error!("Geometry LCM RPC shutdown 失败: {}", error);
                }
                return ExitCode::from(6);
            }
        }
        result = &mut server_task => {
            error!("gRPC server 在 startup health 前退出: {:?}", result);
            if let Err(error) = rpc_servers.stop().await {
                error!("Geometry LCM RPC shutdown 失败: {}", error);
            }
            return ExitCode::from(6);
        }
    }

    if let Some(parent) = std::path::Path::new(&ready_path).parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            error!("ready file 父目录创建失败: {:?}", e);
            shutdown.request_shutdown();
            server_task.abort();
            let _ = (&mut server_task).await;
            if let Err(error) = rpc_servers.stop().await {
                error!("Geometry LCM RPC shutdown 失败: {}", error);
            }
            return ExitCode::from(5);
        }
    }
    if let Err(e) = std::fs::write(&ready_path, format!("{}", port)) {
        error!("ready file 写入失败: {:?}", e);
        shutdown.request_shutdown();
        server_task.abort();
        let _ = (&mut server_task).await;
        if let Err(error) = rpc_servers.stop().await {
            error!("Geometry LCM RPC shutdown 失败: {}", error);
        }
        return ExitCode::from(5);
    }
    info!("gRPC HealthCheck 已通过, ready file 写入: {:?}", ready_path);

    let mut exit_code = match server_task.await {
        Ok(Ok(())) => ExitCode::from(0),
        Ok(Err(e)) => {
            error!("gRPC server 退出: {}", e);
            ExitCode::from(6)
        }
        Err(e) => {
            error!("gRPC server task 失败: {}", e);
            ExitCode::from(6)
        }
    };
    if let Err(error) = rpc_servers.stop().await {
        error!("Geometry LCM RPC shutdown 失败: {}", error);
        exit_code = ExitCode::from(7);
    }
    if let Err(e) = remove_ready_file(&ready_path) {
        error!("删除 ready file 失败: {:?}", e);
        return ExitCode::from(5);
    }
    exit_code
}
