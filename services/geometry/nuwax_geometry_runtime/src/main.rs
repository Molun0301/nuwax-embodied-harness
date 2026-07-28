// Phase 6 (codex-round3 R3-4)：main.rs line 113/165 同样被 `uninlined_format_args`
// 报告,风格性 lint,统一 file-level 标注。
#![allow(clippy::uninlined_format_args)]

//! nuwax-geometry-runtime：LCM-first 几何 native runtime 子进程入口
//!
//! 读取 `NUWAX_GEOMETRY_RUNTIME_*` env + nuwax-workspace LCM channel。
//!
//! 启动顺序：
//! 1. 纯函数校验 LCM backend、owner profile、全部 typed channel 与 frame
//! 2. 创建 Tokio runtime 并初始化 tracing
//! 3. 解析 port、video shm/uds 路径与 ready file 路径
//! 4. 预绑定 gRPC listener（端口占用立即 fail-fast）
//! 5. 启动 video_relay（HTTP/UDS→SHM，与 LCM 无关的视频中转）
//! 6. 用已验证绑定启动 `lcm_runtime::spawn_geometry_runtime`
//! 7. 在同一 LCM owner context 上注册 module-style RPC，再启动 tonic server
//! 8. 自检 HealthCheck 后写 ready file
//! 9. Stop RPC 关闭 listener，确认 LCM unsubscribe 后退出进程

// Phase 4: `pub` 而非私有 —— 兄弟模块(lcm_runtime/server)需要 `crate::channel_registry::*`。
pub mod channel_registry;
mod explore_runtime;
mod forbidden_zones;
mod lcm_runtime;
mod lio_ingress;
mod lio_owner;
mod native_lio_owner;
// W41 (Bug #2 fix): `map_checksum` 模块是 W37 (RALPLAN §Phase M #3) 新增
// 的 SHA-256 helper,供 `mapping_adapter::build_and_publish` 算 checksum
// 并通过 `MapRevisionEvent.checksum` 广播;untracked 状态下 main.rs 漏
// 声明,补回 `mod map_checksum;`。
mod map_checksum;
mod mapping_adapter;
mod mapping_backend;
mod mapping_commit;
mod nav_runtime;
mod occupancy_shm;
mod owner_profile;
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
#[cfg(not(test))]
const GRPC_AFTER_LIO_GRACE: Duration = Duration::from_secs(3);
#[cfg(test)]
const GRPC_AFTER_LIO_GRACE: Duration = Duration::from_millis(100);
#[cfg(not(test))]
const GRPC_FORCE_STOP_TIMEOUT: Duration = Duration::from_secs(1);
#[cfg(test)]
const GRPC_FORCE_STOP_TIMEOUT: Duration = Duration::from_millis(100);
#[cfg(not(test))]
const RPC_SERVER_STOP_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const RPC_SERVER_STOP_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const STARTUP_STOP_TIMEOUT: Duration = Duration::from_secs(32);
#[cfg(test)]
const STARTUP_STOP_TIMEOUT: Duration = Duration::from_secs(2);

type GrpcServerResult = Result<Result<(), tonic::transport::Error>, tokio::task::JoinError>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StartupHealth {
    Ready,
    ShutdownInProgress,
}

enum RuntimeTerminal {
    Grpc(GrpcServerResult),
    Lio(Result<(), String>),
}

struct StartupPreflight {
    owner_profile: owner_profile::OwnerProfile,
    mapping_backend: mapping_backend::MappingBackendSpec,
    channel_bindings: channel_registry::ValidatedChannelBindings,
    path_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
    frame_bindings: lcm_runtime::ValidatedFrameBindings,
    lio_bindings: lio_owner::DeviceLioOwnerSpec,
    lio_source: LioStartupPreflight,
}

enum LioStartupPreflight {
    Device,
    Native(native_lio_owner::NativeLioPreflight),
}

impl StartupPreflight {
    fn from_env() -> Result<Self, String> {
        let owner_profile = owner_profile::OwnerProfile::from_env()
            .map_err(|error| format!("invalid Geometry owner profile: {error}"))?;
        let mapping_backend =
            mapping_backend::MappingBackendSpec::from_env(owner_profile.mapping())?;
        owner_profile
            .admit_production()
            .map_err(|error| error.to_string())?;
        let channel_bindings = channel_registry::ValidatedChannelBindings::from_env()
            .map_err(|error| error.to_string())?;
        let path_binding = nav_runtime::path_envelope_binding()
            .map_err(|error| format!("LCM_PATH_PREFLIGHT_FAILED: {error}"))?;
        let frame_bindings =
            lcm_runtime::ValidatedFrameBindings::from_env().map_err(|error| error.to_string())?;
        let lio_bindings =
            lio_owner::DeviceLioOwnerSpec::from_env(&channel_bindings, &frame_bindings)?;
        let lio_source = match owner_profile.lio() {
            owner_profile::LioBackend::Device => LioStartupPreflight::Device,
            owner_profile::LioBackend::FastLio2 => {
                LioStartupPreflight::Native(native_lio_owner::NativeLioPreflight::from_env(
                    nuwax_lio_backends::native::NativeBackend::FastLio2,
                )?)
            }
            owner_profile::LioBackend::PointLio => {
                LioStartupPreflight::Native(native_lio_owner::NativeLioPreflight::from_env(
                    nuwax_lio_backends::native::NativeBackend::PointLio,
                )?)
            }
        };
        Ok(Self {
            owner_profile,
            mapping_backend,
            channel_bindings,
            path_binding,
            frame_bindings,
            lio_bindings,
            lio_source,
        })
    }
}

pub mod proto {
    tonic::include_proto!("nuwax.geometry");

    impl std::fmt::Debug for ExplorationRecoveryGuardProof {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("ExplorationRecoveryGuardProof([REDACTED])")
        }
    }

    impl std::fmt::Debug for RecoverExplorationAfterNavigationRolloverRequest {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("RecoverExplorationAfterNavigationRolloverRequest([REDACTED])")
        }
    }

    impl std::fmt::Debug for ReconcileExplorationAfterGlobalStopRequest {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("ReconcileExplorationAfterGlobalStopRequest([REDACTED])")
        }
    }

    #[cfg(test)]
    mod security_tests {
        use super::*;

        #[test]
        fn generated_recovery_wire_debug_is_redacted() {
            let token = vec![0x5a; 32];
            let proof = ExplorationRecoveryGuardProof {
                recovery_fence_token: token,
                ..ExplorationRecoveryGuardProof::default()
            };
            let rollover = RecoverExplorationAfterNavigationRolloverRequest {
                guard_proof: Some(proof.clone()),
                ..RecoverExplorationAfterNavigationRolloverRequest::default()
            };
            let reconciliation = ReconcileExplorationAfterGlobalStopRequest {
                held_guard_proof: Some(proof.clone()),
                ..ReconcileExplorationAfterGlobalStopRequest::default()
            };

            for rendered in [
                format!("{proof:?}"),
                format!("{rollover:?}"),
                format!("{reconciliation:?}"),
            ] {
                assert!(rendered.contains("[REDACTED]"));
                assert!(!rendered.contains("90, 90, 90"));
                assert!(!rendered.contains(&"5a".repeat(32)));
            }
        }

        #[test]
        fn generated_recovery_wire_types_disable_automatic_debug() {
            let generated = include_str!(concat!(env!("OUT_DIR"), "/nuwax.geometry.rs"));
            for type_name in [
                "ExplorationRecoveryGuardProof",
                "RecoverExplorationAfterNavigationRolloverRequest",
                "ReconcileExplorationAfterGlobalStopRequest",
            ] {
                assert!(
                    generated.contains(&format!("#[prost(skip_debug)]\npub struct {type_name}"))
                );
            }
        }
    }
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

async fn wait_for_grpc_health(addr: SocketAddr) -> Result<StartupHealth, String> {
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
                        return Ok(StartupHealth::Ready);
                    }
                    if matches!(response.phase.as_str(), "stopping" | "stopped") {
                        return Ok(StartupHealth::ShutdownInProgress);
                    }
                    if response.phase == "failed" {
                        return Err(format!("gRPC HealthCheck 在 {addr} 上报告 runtime failed"));
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .map_err(|_| format!("gRPC HealthCheck 在 {addr} 上 5s 内未就绪"))?
}

async fn shutdown_runtime_owner(
    owner: lio_owner::GeometryRuntimeOwner,
    intended_exit: ExitCode,
) -> ExitCode {
    match owner.shutdown().await {
        Ok(()) => intended_exit,
        Err(error) => {
            error!("Geometry LIO owner shutdown 失败: {}", error);
            ExitCode::from(7)
        }
    }
}

async fn stop_rpc_servers(servers: nuwax_rpc::RpcServerGroup) -> Result<(), String> {
    match tokio::time::timeout(RPC_SERVER_STOP_TIMEOUT, servers.stop()).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(error.to_string()),
        Err(_) => Err("Geometry LCM RPC shutdown timed out".to_string()),
    }
}

async fn cleanup_runtime(
    rpc_servers: nuwax_rpc::RpcServerGroup,
    runtime_owner: lio_owner::GeometryRuntimeOwner,
    intended_exit: ExitCode,
) -> ExitCode {
    let exit_code = match stop_rpc_servers(rpc_servers).await {
        Ok(()) => intended_exit,
        Err(error) => {
            error!("Geometry LCM RPC shutdown 失败: {}", error);
            ExitCode::from(7)
        }
    };
    shutdown_runtime_owner(runtime_owner, exit_code).await
}

async fn force_grpc_server_shutdown(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    shutdown: &server::ShutdownCoordinator,
) {
    shutdown.force_grpc_shutdown();
    if tokio::time::timeout(GRPC_FORCE_STOP_TIMEOUT, &mut *server_task)
        .await
        .is_err()
    {
        server_task.abort();
        let _ = (&mut *server_task).await;
    }
}

async fn finish_grpc_after_lio(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    shutdown: &server::ShutdownCoordinator,
) -> Option<GrpcServerResult> {
    match tokio::time::timeout(GRPC_AFTER_LIO_GRACE, &mut *server_task).await {
        Ok(result) => Some(result),
        Err(_) => {
            error!("LIO 已退出，但 gRPC server 未在宽限期内完成；强制关闭");
            force_grpc_server_shutdown(server_task, shutdown).await;
            None
        }
    }
}

async fn next_runtime_terminal(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    runtime_owner: &mut lio_owner::GeometryRuntimeOwner,
) -> RuntimeTerminal {
    tokio::select! {
        result = &mut *server_task => RuntimeTerminal::Grpc(result),
        result = runtime_owner.wait_lio_terminal() => RuntimeTerminal::Lio(result),
    }
}

async fn completed_runtime_terminal(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    runtime_owner: &mut lio_owner::GeometryRuntimeOwner,
) -> Option<RuntimeTerminal> {
    if server_task.is_finished() {
        return Some(RuntimeTerminal::Grpc((&mut *server_task).await));
    }
    if runtime_owner.is_lio_finished() {
        return Some(RuntimeTerminal::Lio(
            runtime_owner.wait_lio_terminal().await,
        ));
    }
    None
}

async fn runtime_terminal_exit_code(
    terminal: RuntimeTerminal,
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    shutdown: &server::ShutdownCoordinator,
) -> ExitCode {
    match terminal {
        RuntimeTerminal::Grpc(result) => {
            grpc_server_exit_code(result, shutdown.has_shutdown_failed())
        }
        RuntimeTerminal::Lio(Ok(())) => match finish_grpc_after_lio(server_task, shutdown).await {
            Some(result) => grpc_server_exit_code(result, shutdown.has_shutdown_failed()),
            None => ExitCode::from(7),
        },
        RuntimeTerminal::Lio(Err(error)) => {
            error!("Geometry LIO owner 异常退出: {}", error);
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
    }
}

async fn startup_stop_exit_code(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    runtime_owner: &mut lio_owner::GeometryRuntimeOwner,
    shutdown: &server::ShutdownCoordinator,
) -> ExitCode {
    match tokio::time::timeout(
        STARTUP_STOP_TIMEOUT,
        next_runtime_terminal(server_task, runtime_owner),
    )
    .await
    {
        Ok(terminal) => runtime_terminal_exit_code(terminal, server_task, shutdown).await,
        Err(_) => {
            error!("startup health 期间的 Stop 未在总截止时间内完成");
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
    }
}

fn grpc_server_exit_code(result: GrpcServerResult, shutdown_failed: bool) -> ExitCode {
    match result {
        Ok(Ok(())) if shutdown_failed => {
            error!("gRPC server 正常返回，但 shutdown acknowledgement 已失败");
            ExitCode::from(7)
        }
        Ok(Ok(())) => ExitCode::from(0),
        Ok(Err(error)) => {
            error!("gRPC server 退出: {}", error);
            ExitCode::from(6)
        }
        Err(error) => {
            error!("gRPC server task 失败: {}", error);
            ExitCode::from(6)
        }
    }
}

fn remove_ready_file(path: &str) -> io::Result<()> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error),
    }
}

struct ReadyFileLease {
    path: String,
    active: bool,
}

impl ReadyFileLease {
    fn acquire(path: String, port: u16) -> io::Result<Self> {
        if let Err(error) = std::fs::write(&path, port.to_string()) {
            let _ = remove_ready_file(&path);
            return Err(error);
        }
        Ok(Self { path, active: true })
    }

    fn revoke(&mut self) -> io::Result<()> {
        if !self.active {
            return Ok(());
        }
        remove_ready_file(&self.path)?;
        self.active = false;
        Ok(())
    }
}

impl Drop for ReadyFileLease {
    fn drop(&mut self) {
        if self.active {
            let _ = remove_ready_file(&self.path);
        }
    }
}

async fn revoke_ready_after_readiness_signal(
    shutdown: &server::ShutdownCoordinator,
    ready_lease: &mut ReadyFileLease,
) -> bool {
    shutdown.wait_for_readiness_revocation().await;
    match ready_lease.revoke() {
        Ok(()) => false,
        Err(error) => {
            error!("Stop 已开始，但删除 ready file 失败: {:?}", error);
            true
        }
    }
}

fn main() -> ExitCode {
    if let Some(result) = mapping_backend::run_requested_sidecar() {
        return match result {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("{error}");
                ExitCode::from(1)
            }
        };
    }

    let lcm_backend = match lcm_runtime::LcmBackendKind::from_env() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(1);
        }
    };

    let preflight = match StartupPreflight::from_env() {
        Ok(preflight) => preflight,
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
    runtime.block_on(run(lcm_backend, preflight))
}

async fn run(lcm_backend: lcm_runtime::LcmBackendKind, preflight: StartupPreflight) -> ExitCode {
    let StartupPreflight {
        owner_profile,
        mapping_backend,
        channel_bindings,
        path_binding,
        frame_bindings,
        lio_bindings,
        lio_source,
    } = preflight;
    let tf_binding = channel_bindings.binding(channel_registry::ChannelKey::Tf);
    tracing_subscriber::fmt()
        .with_env_filter("nuwax_geometry_runtime=info,nuwax_protocol=warn,nuwax_transport=warn,nuwax_codec=warn,nuwax_tf=warn")
        .with_target(false)
        .init();
    info!(
        "Geometry owner profile selected: lio={} mapping={} correction={}",
        owner_profile.lio().stable_name(),
        owner_profile.mapping().stable_name(),
        owner_profile.correction().stable_name()
    );

    // ----------------------------------------------------------------------
    // env 解析（替换旧 NUWAX_ROS_BRIDGE_*）
    // ----------------------------------------------------------------------
    let port: u16 = std::env::var("NUWAX_GEOMETRY_RUNTIME_PORT")
        .unwrap_or_else(|_| "50051".into())
        .parse()
        .unwrap_or(50051);

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
    let bridge = match lcm_runtime::spawn_geometry_runtime(
        lcm_backend,
        mapping_backend,
        channel_bindings,
        path_binding,
        frame_bindings,
    ) {
        Ok(b) => b,
        Err(e) => {
            // 启动失败时直接 fail-fast(exit code = 4)。错误保留稳定关键字，
            // 同时携带 transport、持久化或配置阶段的真实原因。
            error!("{}", e);
            eprintln!("{e}");
            return ExitCode::from(4);
        }
    };

    let runtime_generation = match i64::try_from(bridge.nav.producer_runtime_generation()) {
        Ok(generation) if generation > 0 => generation,
        Ok(generation) => {
            error!("LIO runtime generation 必须为正数: {}", generation);
            return ExitCode::from(4);
        }
        Err(_) => {
            error!(
                "LIO runtime generation 超出 i64 范围: {}",
                bridge.nav.producer_runtime_generation()
            );
            return ExitCode::from(4);
        }
    };
    let (lio_spec, ingress, rpc_point_cloud_binding): (
        lio_owner::LioOwnerSpec,
        Arc<dyn lio_owner::LioIngressSink>,
        Option<(nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType)>,
    ) = match lio_source {
        LioStartupPreflight::Device => {
            let ingress_spec = match lio_ingress::DeviceLioIngressSpec::new(
                runtime_generation,
                lio_bindings.sensor_frame.clone(),
                lio_bindings.body_frame.clone(),
                lio_bindings.odom_frame.clone(),
                lio_bindings.map_frame.clone(),
            ) {
                Ok(spec) => spec,
                Err(error) => {
                    error!("LIO ingress 配置失败: {}", error);
                    return ExitCode::from(4);
                }
            };
            let ingress = Arc::new(lio_ingress::CommonLioIngress::with_runtime(
                ingress_spec,
                bridge.clone(),
                lio_bindings.public_odometry.clone(),
                lio_bindings.public_cloud.clone(),
                tf_binding.clone(),
            ));
            let rpc_point_cloud_binding = Some(lio_bindings.legacy_cloud.clone());
            (lio_bindings.into(), ingress, rpc_point_cloud_binding)
        }
        LioStartupPreflight::Native(native_preflight) => {
            let native_runtime = match native_preflight.bind(
                runtime_generation,
                lio_bindings.sensor_frame.clone(),
                lio_bindings.body_frame.clone(),
                lio_bindings.odom_frame.clone(),
            ) {
                Ok(runtime) => runtime,
                Err(error) => {
                    error!("Native LIO generation 绑定失败: {}", error);
                    return ExitCode::from(4);
                }
            };
            let ingress_spec = match lio_ingress::DeviceLioIngressSpec::new_for_backend(
                native_runtime.backend_code,
                runtime_generation,
                lio_bindings.sensor_frame.clone(),
                lio_bindings.body_frame.clone(),
                lio_bindings.odom_frame.clone(),
                lio_bindings.map_frame.clone(),
            ) {
                Ok(spec) => spec,
                Err(error) => {
                    error!("Native LIO ingress 配置失败: {}", error);
                    return ExitCode::from(4);
                }
            };
            let ingress = match lio_ingress::NativeLioIngress::with_runtime(
                ingress_spec,
                bridge.clone(),
                lio_bindings.public_odometry.clone(),
                lio_bindings.public_cloud.clone(),
                tf_binding,
                native_runtime.body_from_sensor.clone(),
                native_runtime.pair_max_skew,
            ) {
                Ok(ingress) => Arc::new(ingress),
                Err(error) => {
                    error!("Native LIO ingress 配置失败: {}", error);
                    return ExitCode::from(4);
                }
            };
            (
                lio_owner::LioOwnerSpec::native(lio_bindings, native_runtime),
                ingress,
                None,
            )
        }
    };
    let lio = match lio_owner::LioOwnerHandle::start(
        bridge.transport.clone(),
        lio_spec,
        runtime_generation,
        ingress,
    )
    .await
    {
        Ok(lio) => lio,
        Err(error) => {
            error!("Geometry LIO owner 启动失败: {}", error);
            return ExitCode::from(4);
        }
    };
    let mut runtime_owner = lio_owner::GeometryRuntimeOwner::new(bridge, lio);
    let bridge = runtime_owner.handle();

    let (shutdown, shutdown_listener) =
        server::shutdown_channel_with_lio(runtime_owner.shutdown_control());
    let rpc_context = match bridge.transport.shared_lcm_context() {
        Some(context) => context,
        None => {
            error!("Geometry LCM transport 未暴露真实 owner context，RPC 无法启动");
            return shutdown_runtime_owner(runtime_owner, ExitCode::from(7)).await;
        }
    };
    let service = server::GeometryRuntimeService::new_with_shutdown_and_point_cloud_binding(
        bridge,
        shutdown.clone(),
        rpc_point_cloud_binding,
    );
    let readiness_service = service.clone();
    let rpc_transport: Arc<dyn nuwax_rpc::RpcTransport> =
        Arc::new(nuwax_rpc::LcmRpcTransport::from_context(rpc_context));
    let rpc_servers = match rpc_runtime::serve_geometry_rpc(service.clone(), rpc_transport).await {
        Ok(servers) => servers,
        Err(e) => {
            error!("Geometry LCM RPC server 启动失败: {}", e);
            return shutdown_runtime_owner(runtime_owner, ExitCode::from(7)).await;
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
            match check {
                Ok(StartupHealth::Ready) => {}
                Ok(StartupHealth::ShutdownInProgress) => {
                    info!("startup health 期间收到正常 Stop，等待有界退出");
                    let exit_code = startup_stop_exit_code(
                        &mut server_task,
                        &mut runtime_owner,
                        &shutdown,
                    )
                    .await;
                    return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
                }
                Err(error) => {
                    error!("{}", error);
                    force_grpc_server_shutdown(&mut server_task, &shutdown).await;
                    return cleanup_runtime(
                        rpc_servers,
                        runtime_owner,
                        ExitCode::from(6),
                    )
                    .await;
                }
            }
        }
        result = &mut server_task => {
            let exit_code = match result {
                Ok(Ok(())) if shutdown.has_shutdown_failed() => ExitCode::from(7),
                Ok(Ok(())) if shutdown.is_shutdown_requested() => ExitCode::from(0),
                Ok(Ok(())) => {
                    error!("gRPC server 在 startup health 前无 shutdown 请求即正常返回");
                    ExitCode::from(6)
                }
                result => grpc_server_exit_code(result, shutdown.has_shutdown_failed()),
            };
            return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
        }
        result = runtime_owner.wait_lio_terminal() => {
            let exit_code = runtime_terminal_exit_code(
                RuntimeTerminal::Lio(result),
                &mut server_task,
                &shutdown,
            )
            .await;
            return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
        }
    }

    if let Some(terminal) = completed_runtime_terminal(&mut server_task, &mut runtime_owner).await {
        let exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
        return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
    }

    if let Some(parent) = std::path::Path::new(&ready_path).parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            error!("ready file 父目录创建失败: {:?}", e);
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(rpc_servers, runtime_owner, ExitCode::from(5)).await;
        }
    }

    if let Some(terminal) = completed_runtime_terminal(&mut server_task, &mut runtime_owner).await {
        let exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
        return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
    }

    let mut ready_lease = match readiness_service
        .run_if_running(|| ReadyFileLease::acquire(ready_path.clone(), port))
        .await
    {
        Ok(Ok(lease)) => lease,
        Ok(Err(error)) => {
            error!("ready file 写入失败: {:?}", error);
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(rpc_servers, runtime_owner, ExitCode::from(5)).await;
        }
        Err(phase) if matches!(phase.as_str(), "stopping" | "stopped") => {
            info!(phase, "ready 发布前收到正常 Stop，等待有界退出");
            let exit_code =
                startup_stop_exit_code(&mut server_task, &mut runtime_owner, &shutdown).await;
            return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
        }
        Err(phase) => {
            error!(phase, "ready 发布前 runtime 已离开 running 状态");
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(rpc_servers, runtime_owner, ExitCode::from(7)).await;
        }
    };

    if let Some(terminal) = completed_runtime_terminal(&mut server_task, &mut runtime_owner).await {
        let ready_remove_failed = match ready_lease.revoke() {
            Ok(()) => false,
            Err(error) => {
                error!("删除 ready file 失败: {:?}", error);
                true
            }
        };
        drop(ready_lease);
        let mut exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
        if ready_remove_failed {
            exit_code = ExitCode::from(5);
        }
        return cleanup_runtime(rpc_servers, runtime_owner, exit_code).await;
    }

    info!("gRPC HealthCheck 已通过, ready file 写入: {:?}", ready_path);

    let (terminal, mut ready_remove_failed) = tokio::select! {
        terminal = next_runtime_terminal(&mut server_task, &mut runtime_owner) => {
            (terminal, false)
        }
        ready_remove_failed = revoke_ready_after_readiness_signal(&shutdown, &mut ready_lease) => {
            (
                next_runtime_terminal(&mut server_task, &mut runtime_owner).await,
                ready_remove_failed,
            )
        }
    };
    if let Err(error) = ready_lease.revoke() {
        error!("删除 ready file 失败: {:?}", error);
        ready_remove_failed = true;
    }
    drop(ready_lease);

    let mut exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
    if ready_remove_failed {
        exit_code = ExitCode::from(5);
    }
    cleanup_runtime(rpc_servers, runtime_owner, exit_code).await
}

#[cfg(test)]
mod lifecycle_tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[cfg(target_os = "linux")]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    struct ProcessIdentity {
        pid: u32,
        start_time_ticks: u64,
    }

    #[cfg(target_os = "linux")]
    struct LauncherGuard {
        launcher: Option<std::process::Child>,
        runtime_pid_path: PathBuf,
        test_dir: PathBuf,
    }

    #[cfg(target_os = "linux")]
    impl LauncherGuard {
        fn process_identity(pid: u32) -> Option<ProcessIdentity> {
            let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
            let (_, fields) = stat.rsplit_once(") ")?;
            let start_time_ticks = fields.split_whitespace().nth(19)?.parse().ok()?;
            Some(ProcessIdentity {
                pid,
                start_time_ticks,
            })
        }

        fn process_is_alive(identity: ProcessIdentity) -> bool {
            Self::process_identity(identity.pid) == Some(identity)
        }

        fn signal(pid: u32, name: &str) {
            let _ = std::process::Command::new("kill")
                .arg(format!("-{name}"))
                .arg(pid.to_string())
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status();
        }

        fn runtime_pid(&self) -> Option<u32> {
            std::fs::read_to_string(&self.runtime_pid_path)
                .ok()?
                .trim()
                .parse()
                .ok()
        }

        fn runtime_identity(&self) -> Option<ProcessIdentity> {
            Self::process_identity(self.runtime_pid()?)
        }

        fn launcher_is_alive(&mut self) -> bool {
            self.launcher
                .as_mut()
                .is_some_and(|child| child.try_wait().ok().flatten().is_none())
        }

        fn shutdown(&mut self) -> Option<std::process::ExitStatus> {
            let runtime_identity = self.runtime_identity();
            let mut launcher = self.launcher.take()?;
            let launcher_pid = launcher.id();
            Self::signal(launcher_pid, "TERM");
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            let status = loop {
                if let Ok(Some(status)) = launcher.try_wait() {
                    break Some(status);
                }
                if std::time::Instant::now() >= deadline {
                    let _ = launcher.kill();
                    break launcher.wait().ok();
                }
                std::thread::sleep(Duration::from_millis(20));
            };
            if let Some(runtime_identity) = runtime_identity {
                if Self::process_is_alive(runtime_identity) {
                    Self::signal(runtime_identity.pid, "TERM");
                }
                let deadline = std::time::Instant::now() + Duration::from_secs(1);
                while Self::process_is_alive(runtime_identity)
                    && std::time::Instant::now() < deadline
                {
                    std::thread::sleep(Duration::from_millis(20));
                }
                if Self::process_is_alive(runtime_identity) {
                    Self::signal(runtime_identity.pid, "KILL");
                }
            }
            status
        }
    }

    #[cfg(target_os = "linux")]
    impl Drop for LauncherGuard {
        fn drop(&mut self) {
            let _ = self.shutdown();
            let _ = std::fs::remove_dir_all(&self.test_dir);
        }
    }

    #[cfg(target_os = "linux")]
    fn wait_for_path_state(path: &std::path::Path, exists: bool, timeout: Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        while path.exists() != exists && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        path.exists() == exists
    }

    #[tokio::test]
    async fn grpc_completion_within_lio_grace_remains_normal() {
        let (shutdown, _listener) = server::shutdown_channel();
        let mut server_task = tokio::spawn(async { Ok::<(), tonic::transport::Error>(()) });

        let result = finish_grpc_after_lio(&mut server_task, &shutdown)
            .await
            .expect("immediate gRPC completion must remain normal");

        assert!(matches!(result, Ok(Ok(()))));
        assert!(!shutdown.is_shutdown_requested());
    }

    #[tokio::test]
    async fn grpc_after_lio_grace_forces_bounded_shutdown() {
        let (shutdown, mut listener) = server::shutdown_channel();
        let shutdown_seen = Arc::new(AtomicBool::new(false));
        let task_shutdown_seen = shutdown_seen.clone();
        let mut server_task = tokio::spawn(async move {
            listener.wait_for_request().await;
            task_shutdown_seen.store(true, Ordering::Release);
            Ok::<(), tonic::transport::Error>(())
        });

        let result = tokio::time::timeout(
            GRPC_AFTER_LIO_GRACE + GRPC_FORCE_STOP_TIMEOUT + Duration::from_secs(1),
            finish_grpc_after_lio(&mut server_task, &shutdown),
        )
        .await
        .expect("forced gRPC shutdown must remain bounded");

        assert!(result.is_none(), "forced completion is an abnormal exit");
        assert!(shutdown.is_shutdown_requested());
        assert!(shutdown_seen.load(Ordering::Acquire));
        assert!(server_task.is_finished());
    }

    #[tokio::test]
    async fn listener_ack_failure_makes_normal_grpc_completion_nonzero() {
        let (shutdown, listener) = server::shutdown_channel();
        drop(listener);

        let error = shutdown
            .request_and_wait_for_listener()
            .await
            .expect_err("closed listener ACK channel must fail");

        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(shutdown.has_shutdown_failed());
        assert_eq!(
            grpc_server_exit_code(Ok(Ok(())), shutdown.has_shutdown_failed()),
            ExitCode::from(7)
        );
    }

    #[tokio::test]
    async fn readiness_revocation_removes_ready_lease_without_a_runtime_terminal() {
        let ready_path = std::env::temp_dir().join(format!(
            "nuwax-geometry-ready-revocation-{}-{}",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let mut ready_lease =
            ReadyFileLease::acquire(ready_path.to_string_lossy().into_owned(), 50_051)
                .expect("test ready lease");
        assert!(ready_path.exists());
        let (shutdown, _listener) = server::shutdown_channel();
        let signal = shutdown.clone();
        tokio::spawn(async move {
            tokio::task::yield_now().await;
            signal.revoke_readiness();
        });

        let remove_failed = tokio::time::timeout(
            Duration::from_secs(1),
            revoke_ready_after_readiness_signal(&shutdown, &mut ready_lease),
        )
        .await
        .expect("readiness revocation deadline");

        assert!(!remove_failed);
        assert!(!ready_lease.active);
        assert!(!ready_path.exists());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn launcher_revokes_public_marker_while_failed_runtime_remains_alive() {
        use std::os::unix::fs::PermissionsExt;

        let test_dir = std::env::temp_dir().join(format!(
            "nuwax-geometry-launcher-ready-{}-{}",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let binary = test_dir.join("fake-geometry-runtime.sh");
        let ready = test_dir.join("geometry.ready");
        let internal_ready = PathBuf::from(format!("{}.runtime", ready.display()));
        let revoke = test_dir.join("revoke-ready");
        let runtime_pid_path = test_dir.join("runtime.pid");
        std::fs::create_dir_all(&test_dir).expect("launcher test directory");
        let mut guard = LauncherGuard {
            launcher: None,
            runtime_pid_path: runtime_pid_path.clone(),
            test_dir,
        };
        std::fs::write(
            &binary,
            "#!/usr/bin/env bash\n\
             set -euo pipefail\n\
             printf '%s' \"$$\" > \"${NUWAX_TEST_RUNTIME_PID_FILE}\"\n\
             printf '%s' \"${NUWAX_GEOMETRY_RUNTIME_PORT}\" > \"${NUWAX_GEOMETRY_RUNTIME_READY_FILE}.tmp\"\n\
             mv -f \"${NUWAX_GEOMETRY_RUNTIME_READY_FILE}.tmp\" \"${NUWAX_GEOMETRY_RUNTIME_READY_FILE}\"\n\
             trap 'exit 0' INT TERM\n\
             while [[ ! -f \"${NUWAX_TEST_REVOKE_FILE}\" ]]; do sleep 0.02; done\n\
             rm -f \"${NUWAX_GEOMETRY_RUNTIME_READY_FILE}\"\n\
             while true; do sleep 0.05; done\n",
        )
        .expect("fake geometry runtime");
        let mut permissions = std::fs::metadata(&binary)
            .expect("fake runtime metadata")
            .permissions();
        permissions.set_mode(0o755);
        std::fs::set_permissions(&binary, permissions).expect("fake runtime executable");
        let launcher_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(std::path::Path::parent)
            .and_then(std::path::Path::parent)
            .expect("repository root")
            .join("scripts/systemd/start_geometry_runtime.sh");
        guard.launcher = Some(
            std::process::Command::new("bash")
                .arg(launcher_path)
                .env("NUWAX_GEOMETRY_RUNTIME_BINARY", &binary)
                .env("NUWAX_GEOMETRY_RUNTIME_READY_FILE", &ready)
                .env("NUWAX_GEOMETRY_RUNTIME_PORT", "50051")
                .env("NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT", "2")
                .env("NUWAX_TEST_REVOKE_FILE", &revoke)
                .env("NUWAX_TEST_RUNTIME_PID_FILE", &runtime_pid_path)
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .spawn()
                .expect("start geometry launcher"),
        );

        assert!(wait_for_path_state(&ready, true, Duration::from_secs(5)));
        assert!(internal_ready.exists());
        let runtime_identity = guard.runtime_identity().expect("fake runtime identity");
        assert!(LauncherGuard::process_is_alive(runtime_identity));

        std::fs::write(&revoke, b"1").expect("trigger readiness revocation");
        assert!(wait_for_path_state(
            &internal_ready,
            false,
            Duration::from_secs(3)
        ));
        assert!(wait_for_path_state(&ready, false, Duration::from_secs(3)));
        assert!(guard.launcher_is_alive());
        assert!(LauncherGuard::process_is_alive(runtime_identity));

        let status = guard.shutdown().expect("launcher exit status");
        assert!(status.success(), "launcher exit status: {status}");
        assert!(!LauncherGuard::process_is_alive(runtime_identity));
    }
}
