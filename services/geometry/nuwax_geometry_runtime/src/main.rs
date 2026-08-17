// Phase 6 (codex-round3 R3-4)：main.rs line 113/165 同样被 `uninlined_format_args`
// 报告,风格性 lint,统一 file-level 标注。
#![allow(clippy::uninlined_format_args)]

//! nuwax-geometry-runtime：LCM-first 几何 native runtime 子进程入口
//!
//! 读取 `NUWAX_GEOMETRY_RUNTIME_*` env + nuwax-workspace LCM channel。
//!
//! 启动顺序：
//! 1. 预检 LCM backend、owner profile、Mapping、typed channel/frame、Correction 与 LIO 配置
//! 2. 创建 Tokio runtime，初始化 tracing，解析路径并预绑定 gRPC listener
//! 3. 创建 prepared Geometry runtime，再创建唯一共享的 relocalization fence（inactive `E0`）
//! 4. 构造 runtime-free native LIO probe，并在任何 raw frame 可达前绑定共享 fence
//! 5. 构造 routing gate 并启动 LIO owner，等待 subscription ACK 与首个有效 raw pair
//! 6. 启动 Geometry/Mapping runtime，注册 root task owner，并取得 TF/path/nav subscribe ACK
//! 7. 用同一个共享 fence 启动 Correction owner
//! 8. 转移 sealed LIO pair，绑定 runtime、Correction ingress 与 routing gate
//! 9. native 路径切到 deferred 模式并发布 startup odometry
//! 10. 保留 retained cloud，等待精确的 `StartMapping` 会话再 replay
//! 11. 组合持有 LIO、Mapping、Correction、Geometry task、gate 与 replay owner
//! 12. 启动 video relay、LCM RPC 与 tonic，HealthCheck 成功后写 ready file
//! 13. Stop 先撤销 ready/全部准入，再按 video、RPC、retained settlement、owned unsubscribe、
//!     Geometry task join、Correction、Mapping、LIO、产品 mmap 清理的顺序退出

// Phase 4: `pub` 而非私有 —— 兄弟模块(lcm_runtime/server)需要 `crate::channel_registry::*`。
pub mod channel_registry;
mod correction_coordinator;
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
mod persistent_occupancy;
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
#[cfg(unix)]
use tokio::signal::unix::{signal, SignalKind};
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
const VIDEO_RELAY_STOP_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const VIDEO_RELAY_STOP_TIMEOUT: Duration = Duration::from_secs(1);
#[cfg(not(test))]
const STARTUP_STOP_TIMEOUT: Duration = Duration::from_secs(32);
#[cfg(test)]
const STARTUP_STOP_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const PROCESS_STOP_DRAIN_TIMEOUT: Duration = Duration::from_secs(40);
#[cfg(test)]
const PROCESS_STOP_DRAIN_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const MAPPING_CLOUD_STOP_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const MAPPING_CLOUD_STOP_TIMEOUT: Duration = Duration::from_millis(250);
const FINAL_OWNER_ATTEMPT_LIMIT: usize = 3;
#[cfg(not(test))]
const FINAL_OWNER_ATTEMPT_BACKOFF: Duration = Duration::from_millis(50);
#[cfg(test)]
const FINAL_OWNER_ATTEMPT_BACKOFF: Duration = Duration::from_millis(1);

static GEOMETRY_RETAINED_EXIT_SELECTED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

type GrpcServerResult = Result<Result<(), tonic::transport::Error>, tokio::task::JoinError>;

#[cfg(unix)]
struct UnixShutdownSignals {
    interrupt: tokio::signal::unix::Signal,
    terminate: tokio::signal::unix::Signal,
}

#[cfg(unix)]
impl UnixShutdownSignals {
    fn register() -> io::Result<Self> {
        Ok(Self {
            interrupt: signal(SignalKind::interrupt())?,
            terminate: signal(SignalKind::terminate())?,
        })
    }

    async fn forward_to(mut self, service: server::GeometryRuntimeService) {
        loop {
            let signal_name = tokio::select! {
                signal = self.interrupt.recv() => signal.map(|_| "SIGINT"),
                signal = self.terminate.recv() => signal.map(|_| "SIGTERM"),
            };
            let Some(signal_name) = signal_name else {
                error!("Unix 关闭信号 stream 意外关闭");
                return;
            };
            info!(
                signal = signal_name,
                "收到 Unix 关闭信号，执行 Process Stop"
            );
            if let Err(error) = service.perform_process_stop().await {
                error!(signal = signal_name, error = %error, "Unix 信号 Process Stop 失败");
                return;
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StartupHealth {
    Ready,
    ShutdownInProgress,
}

#[derive(Default)]
struct FinalOwnerAttemptBudget {
    attempts: usize,
}

impl FinalOwnerAttemptBudget {
    fn next_attempt(&mut self) -> Option<usize> {
        if self.attempts == FINAL_OWNER_ATTEMPT_LIMIT {
            return None;
        }
        self.attempts += 1;
        Some(self.attempts)
    }

    fn exhausted(&self) -> bool {
        self.attempts == FINAL_OWNER_ATTEMPT_LIMIT
    }
}

#[allow(dead_code)]
enum GeometryProcessExitRetention {
    Production(Box<GeometryProductionOwner>),
    MappingAndLio {
        owner: Box<lio_owner::GeometryRuntimeOwner>,
        ingress_gate: Arc<lio_ingress::LioIngressRoutingGate>,
    },
    StartupRollbackAndLio {
        geometry: Box<lcm_runtime::GeometryRuntimeStartupRollbackOwner>,
        lio: lio_owner::LioOwnerHandle,
    },
}

#[cfg(not(test))]
fn retain_geometry_owner_until_process_exit(retention: GeometryProcessExitRetention) {
    GEOMETRY_RETAINED_EXIT_SELECTED.store(true, std::sync::atomic::Ordering::Release);
    let _: &'static mut GeometryProcessExitRetention = Box::leak(Box::new(retention));
}

#[cfg(test)]
fn geometry_process_exit_retention_for_tests(
) -> &'static std::sync::Mutex<Vec<GeometryProcessExitRetention>> {
    static RETAINED: std::sync::OnceLock<std::sync::Mutex<Vec<GeometryProcessExitRetention>>> =
        std::sync::OnceLock::new();
    RETAINED.get_or_init(|| std::sync::Mutex::new(Vec::new()))
}

#[cfg(test)]
fn retain_geometry_owner_until_process_exit(retention: GeometryProcessExitRetention) {
    geometry_process_exit_retention_for_tests()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .push(retention);
    GEOMETRY_RETAINED_EXIT_SELECTED.store(true, std::sync::atomic::Ordering::Release);
}

#[cfg(test)]
fn take_geometry_process_exit_retention_for_tests() -> GeometryProcessExitRetention {
    geometry_process_exit_retention_for_tests()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .pop()
        .expect("Geometry process-exit retention is populated")
}

#[cfg(test)]
fn reset_geometry_retained_exit_after_test_settlement() {
    assert!(
        geometry_process_exit_retention_for_tests()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_empty(),
        "retained-exit signal may reset only after all test retention is settled"
    );
    assert!(
        GEOMETRY_RETAINED_EXIT_SELECTED.swap(false, std::sync::atomic::Ordering::AcqRel,),
        "test settlement must clear a selected retained-exit signal"
    );
}

fn finalize_geometry_runtime(runtime: tokio::runtime::Runtime, exit_code: ExitCode) -> ExitCode {
    if GEOMETRY_RETAINED_EXIT_SELECTED.load(std::sync::atomic::Ordering::Acquire) {
        runtime.shutdown_background();
    } else {
        drop(runtime);
    }
    exit_code
}

enum RuntimeTerminal {
    Video(Box<video_relay::VideoRelayTerminal>),
    Grpc(GrpcServerResult),
    Lio(Result<(), String>),
    Correction(Result<(), String>),
    GeometryTasks(Result<(), String>),
    ProcessStopFailed,
    ReplayPoison(String),
}

fn prefer_sticky_runtime_terminal(
    candidate: RuntimeTerminal,
    process_stop_failed: bool,
    replay_poison: Option<String>,
    geometry_task_failure: Option<String>,
) -> RuntimeTerminal {
    if process_stop_failed {
        RuntimeTerminal::ProcessStopFailed
    } else if let Some(detail) = replay_poison {
        RuntimeTerminal::ReplayPoison(detail)
    } else if let Some(error) = geometry_task_failure {
        RuntimeTerminal::GeometryTasks(Err(error))
    } else {
        candidate
    }
}

struct StartupPreflight {
    owner_profile: owner_profile::OwnerProfile,
    mapping_backend: mapping_backend::MappingBackendSpec,
    channel_bindings: channel_registry::ValidatedChannelBindings,
    path_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
    frame_bindings: lcm_runtime::ValidatedFrameBindings,
    correction: correction_coordinator::CorrectionOwnerSpec,
    correction_provider: String,
    lio_bindings: lio_owner::DeviceLioOwnerSpec,
    lio_source: LioStartupPreflight,
}

enum LioStartupPreflight {
    Device,
    Native(native_lio_owner::NativeLioPreflight),
}

enum PendingLioIngress {
    Device {
        spec: lio_ingress::DeviceLioIngressSpec,
        probe: Arc<lio_ingress::CommonLioIngress>,
    },
    Native {
        spec: lio_ingress::DeviceLioIngressSpec,
        probe: Arc<lio_ingress::NativeLioIngress>,
        body_from_sensor: native_lio_owner::BodyFromSensorTransform,
        pair_max_skew: Duration,
    },
}

enum PreparedLioIngress {
    Device(Arc<lio_ingress::CommonLioIngress>),
    Native {
        ingress: Arc<lio_ingress::NativeLioIngress>,
        pair: Box<lio_ingress::ValidatedStartupPair>,
    },
}

enum BoundLioIngress {
    Device(Arc<dyn lio_owner::LioIngressSink>),
    Native {
        sink: Arc<dyn lio_owner::LioIngressSink>,
        ingress: Arc<lio_ingress::NativeLioIngress>,
        pair: Box<lio_ingress::ValidatedStartupPair>,
    },
}

impl PendingLioIngress {
    fn probe(&self) -> Arc<dyn lio_owner::LioIngressSink> {
        match self {
            Self::Device { probe, .. } => probe.clone(),
            Self::Native { probe, .. } => probe.clone(),
        }
    }

    fn bind_runtime(
        self,
        handle: lcm_runtime::GeometryRuntimeHandle,
        public_odometry: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
        public_cloud: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
        tf: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
    ) -> Result<PreparedLioIngress, String> {
        match self {
            Self::Device { spec, .. } => Ok(PreparedLioIngress::Device(Arc::new(
                lio_ingress::CommonLioIngress::with_runtime(
                    spec,
                    handle,
                    public_odometry,
                    public_cloud,
                    tf,
                ),
            ))),
            Self::Native {
                spec,
                probe,
                body_from_sensor,
                pair_max_skew,
            } => {
                let pair = probe.take_validated_startup_pair()?;
                lio_ingress::NativeLioIngress::with_runtime(
                    spec,
                    handle,
                    public_odometry,
                    public_cloud,
                    tf,
                    body_from_sensor,
                    pair_max_skew,
                )
                .map(|ingress| PreparedLioIngress::Native {
                    ingress: Arc::new(ingress),
                    pair: Box::new(pair),
                })
                .map_err(|error| error.to_string())
            }
        }
    }
}

impl PreparedLioIngress {
    fn bind_correction(
        self,
        correction_ingress: correction_coordinator::CorrectionFrameIngress,
        correction_applier: Arc<lcm_runtime::GeometryCanonicalCorrectionApplier>,
    ) -> BoundLioIngress {
        match self {
            Self::Device(inner) => {
                BoundLioIngress::Device(Arc::new(lio_ingress::CorrectionAwareLioIngress::device(
                    inner,
                    correction_ingress,
                    correction_applier,
                )))
            }
            Self::Native { ingress, pair } => BoundLioIngress::Native {
                sink: Arc::new(lio_ingress::CorrectionAwareLioIngress::native(
                    ingress.clone(),
                    correction_ingress,
                    correction_applier,
                )),
                ingress,
                pair,
            },
        }
    }
}

impl StartupPreflight {
    fn from_env() -> Result<Self, String> {
        let owner_profile = owner_profile::OwnerProfile::from_env()
            .map_err(|error| format!("invalid Geometry owner profile: {error}"))?;
        let mapping_backend =
            mapping_backend::MappingBackendSpec::from_env(match owner_profile.mapping() {
                owner_profile::MappingBackend::Voxel => mapping_backend::MappingBackendKind::Voxel,
                owner_profile::MappingBackend::Raytracing => {
                    mapping_backend::MappingBackendKind::Raytracing
                }
            })?;
        owner_profile
            .admit_production()
            .map_err(|error| error.to_string())?;
        let channel_bindings = channel_registry::ValidatedChannelBindings::from_env()
            .map_err(|error| error.to_string())?;
        let path_binding = nav_runtime::path_envelope_binding()
            .map_err(|error| format!("LCM_PATH_PREFLIGHT_FAILED: {error}"))?;
        let frame_bindings =
            lcm_runtime::ValidatedFrameBindings::from_env().map_err(|error| error.to_string())?;
        let correction = correction_coordinator::CorrectionOwnerSpec::preflight(
            owner_profile.correction(),
            &frame_bindings,
        )?;
        let correction_provider = match std::env::var("NUWAX_LCM_URL") {
            Ok(provider)
                if !provider.is_empty()
                    && provider.trim() == provider
                    && !provider.contains('\0') =>
            {
                provider
            }
            Ok(_) => return Err("CORRECTION_PREFLIGHT_FAILED invalid LCM provider".to_owned()),
            Err(std::env::VarError::NotPresent) => "udpm://239.255.76.67:7667?ttl=1".to_owned(),
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err("CORRECTION_PREFLIGHT_FAILED LCM provider is not Unicode".to_owned());
            }
        };
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
            correction,
            correction_provider,
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

struct GeometryProductionOwner {
    runtime: lio_owner::GeometryRuntimeOwner,
    correction: Option<correction_coordinator::CorrectionTransportOwner>,
    ingress_gate: Arc<lio_ingress::LioIngressRoutingGate>,
    correction_phase_a: correction_coordinator::CorrectionPhaseAControl,
    deferred_replay: Option<Arc<lio_ingress::NativeDeferredReplayControl>>,
    mapping_cloud_owner: Option<Arc<nuwax_mapping_cloud::MappingCloudOwner>>,
    shutdown_events: Option<lio_owner::GeometryShutdownEventEmitter>,
    geometry_task_settlement: Option<lcm_runtime::GeometryTaskSettlement>,
    geometry_task_failure: Option<String>,
    correction_settled: bool,
    correction_failure: Option<String>,
    correction_settled_emitted: bool,
    #[cfg(test)]
    replay_poison_for_terminal_tests: Option<String>,
    #[cfg(test)]
    shutdown_attempts_for_tests: usize,
}

struct GeometryProductionShutdownFailure {
    detail: String,
    pending_owner: Option<Box<GeometryProductionOwner>>,
}

impl std::fmt::Debug for GeometryProductionShutdownFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("GeometryProductionShutdownFailure")
            .field("detail", &self.detail)
            .field("has_pending_owner", &self.pending_owner.is_some())
            .finish()
    }
}

impl std::fmt::Display for GeometryProductionShutdownFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.detail)
    }
}

impl GeometryProductionShutdownFailure {
    fn terminal(detail: impl Into<String>) -> Self {
        Self {
            detail: detail.into(),
            pending_owner: None,
        }
    }

    fn unsettled(detail: impl Into<String>, owner: GeometryProductionOwner) -> Self {
        Self {
            detail: detail.into(),
            pending_owner: Some(Box::new(owner)),
        }
    }

    fn into_pending_owner(self) -> Option<Box<GeometryProductionOwner>> {
        self.pending_owner
    }
}

impl GeometryProductionOwner {
    fn new(
        runtime: lio_owner::GeometryRuntimeOwner,
        correction: correction_coordinator::CorrectionTransportOwner,
        ingress_gate: Arc<lio_ingress::LioIngressRoutingGate>,
        correction_phase_a: correction_coordinator::CorrectionPhaseAControl,
        deferred_replay: Option<Arc<lio_ingress::NativeDeferredReplayControl>>,
    ) -> Self {
        Self {
            runtime,
            correction: Some(correction),
            ingress_gate,
            correction_phase_a,
            deferred_replay,
            mapping_cloud_owner: None,
            shutdown_events: None,
            geometry_task_settlement: None,
            geometry_task_failure: None,
            correction_settled: false,
            correction_failure: None,
            correction_settled_emitted: false,
            #[cfg(test)]
            replay_poison_for_terminal_tests: None,
            #[cfg(test)]
            shutdown_attempts_for_tests: 0,
        }
    }

    fn with_mapping_cloud_owner(
        mut self,
        owner: Arc<nuwax_mapping_cloud::MappingCloudOwner>,
    ) -> Self {
        self.mapping_cloud_owner = Some(owner);
        self
    }

    fn with_shutdown_events(mut self, events: lio_owner::GeometryShutdownEventEmitter) -> Self {
        self.shutdown_events = Some(events);
        self
    }

    fn handle(&self) -> lcm_runtime::GeometryRuntimeHandle {
        self.runtime.handle()
    }

    fn close_ingress(&self) {
        self.runtime.close_task_admission();
        if let Some(deferred_replay) = &self.deferred_replay {
            deferred_replay.begin_phase_a();
        }
        lio_owner::LioIngressSink::close(self.ingress_gate.as_ref());
        self.correction_phase_a.begin();
    }

    async fn wait_lio_terminal(&mut self) -> Result<(), String> {
        self.runtime.wait_lio_terminal().await
    }

    fn is_lio_finished(&self) -> bool {
        self.runtime.is_lio_finished()
    }

    async fn wait_geometry_terminal(&mut self) -> Result<(), String> {
        self.runtime.wait_geometry_terminal().await
    }

    fn is_geometry_finished(&self) -> bool {
        self.runtime.is_geometry_finished()
    }

    async fn wait_correction_terminal(&mut self) -> Result<(), String> {
        match self.correction.as_mut() {
            Some(correction) => correction.wait_terminal().await,
            None => std::future::pending().await,
        }
    }

    fn is_correction_finished(&self) -> bool {
        self.correction
            .as_ref()
            .is_none_or(correction_coordinator::CorrectionTransportOwner::is_finished)
    }

    async fn wait_owned_terminal(&mut self) -> RuntimeTerminal {
        let correction = self
            .correction
            .as_mut()
            .expect("running Geometry production owner retains Correction owner");
        match &self.deferred_replay {
            Some(deferred_replay) => tokio::select! {
                biased;
                () = self.correction_phase_a.wait_process_stop_failed() => RuntimeTerminal::ProcessStopFailed,
                detail = deferred_replay.wait_poison() => RuntimeTerminal::ReplayPoison(detail),
                owned = self.runtime.wait_terminal() => match owned {
                    lio_owner::GeometryOwnedTerminal::Tasks(result) => RuntimeTerminal::GeometryTasks(result),
                    lio_owner::GeometryOwnedTerminal::Lio(result) => RuntimeTerminal::Lio(result),
                },
                result = correction.wait_terminal() => RuntimeTerminal::Correction(result),
            },
            None => tokio::select! {
                biased;
                () = self.correction_phase_a.wait_process_stop_failed() => RuntimeTerminal::ProcessStopFailed,
                owned = self.runtime.wait_terminal() => match owned {
                    lio_owner::GeometryOwnedTerminal::Tasks(result) => RuntimeTerminal::GeometryTasks(result),
                    lio_owner::GeometryOwnedTerminal::Lio(result) => RuntimeTerminal::Lio(result),
                },
                result = correction.wait_terminal() => RuntimeTerminal::Correction(result),
            },
        }
    }

    fn prefer_completed_failure(&self, candidate: RuntimeTerminal) -> RuntimeTerminal {
        prefer_sticky_runtime_terminal(
            candidate,
            self.correction_phase_a.process_stop_failed(),
            self.replay_poison_detail(),
            self.runtime.geometry_completed_failure(),
        )
    }

    fn completed_failure_terminal(&self) -> Option<RuntimeTerminal> {
        if self.correction_phase_a.process_stop_failed() {
            Some(RuntimeTerminal::ProcessStopFailed)
        } else if let Some(detail) = self.replay_poison_detail() {
            Some(RuntimeTerminal::ReplayPoison(detail))
        } else {
            self.runtime
                .geometry_completed_failure()
                .map(|error| RuntimeTerminal::GeometryTasks(Err(error)))
        }
    }

    fn replay_poison_detail(&self) -> Option<String> {
        let detail = self
            .deferred_replay
            .as_ref()
            .and_then(|replay| replay.poison_detail());
        #[cfg(test)]
        let detail = detail.or_else(|| self.replay_poison_for_terminal_tests.clone());
        detail
    }

    #[cfg(test)]
    fn inject_replay_poison_for_terminal_tests(&mut self, detail: impl Into<String>) {
        self.replay_poison_for_terminal_tests = Some(detail.into());
    }

    async fn shutdown(mut self) -> Result<(), GeometryProductionShutdownFailure> {
        #[cfg(test)]
        {
            self.shutdown_attempts_for_tests += 1;
        }
        self.close_ingress();
        if let Some(mapping_cloud_owner) = &self.mapping_cloud_owner {
            match mapping_cloud_owner
                .shutdown(MAPPING_CLOUD_STOP_TIMEOUT)
                .await
            {
                nuwax_mapping_cloud::MappingCloudShutdown::Settled => {
                    self.mapping_cloud_owner = None;
                }
                nuwax_mapping_cloud::MappingCloudShutdown::Pending {
                    request_id,
                    owner_epoch,
                } => {
                    return Err(GeometryProductionShutdownFailure::unsettled(
                        format!(
                            "MappingCloud owner shutdown pending: request_id={request_id} owner_epoch={owner_epoch}"
                        ),
                        self,
                    ));
                }
            }
        }
        if self.geometry_task_settlement.is_none() {
            let task_outcome = self
                .runtime
                .shutdown_geometry_tasks(self.shutdown_events.as_ref())
                .await;
            let (task_result, task_settlement) = task_outcome.into_parts();
            if !task_settlement.is_settled() {
                let detail = task_result.clone().err().unwrap_or_else(|| {
                    "Geometry task producers remained unsettled during production shutdown"
                        .to_owned()
                });
                return Err(GeometryProductionShutdownFailure::unsettled(detail, self));
            }
            self.geometry_task_settlement = Some(task_settlement);
            if let Err(error) = task_result {
                self.geometry_task_failure = Some(error);
            }
        }

        if !self.correction_settled {
            let correction = self
                .correction
                .take()
                .expect("pending Geometry production owner retains Correction owner");
            match correction.shutdown().await {
                Ok(settlement) => {
                    self.correction_failure = settlement.into_sticky_error();
                    self.correction_settled = true;
                    self.emit_correction_settled_once();
                }
                Err(pending) => {
                    let correction_detail = pending.to_string();
                    self.correction = Some(*pending.into_owner());
                    let detail = match &self.geometry_task_failure {
                        Some(task_error) => format!(
                            "Geometry task shutdown failed: {task_error}; {correction_detail}"
                        ),
                        None => correction_detail,
                    };
                    return Err(GeometryProductionShutdownFailure::unsettled(detail, self));
                }
            }
        }

        let task_settlement = *self
            .geometry_task_settlement
            .as_ref()
            .expect("Correction shutdown follows settled Geometry task producers");
        let runtime_terminal = match self
            .runtime
            .shutdown_after_geometry_tasks(task_settlement, self.shutdown_events.as_ref())
            .await
        {
            lio_owner::GeometryRuntimeAfterTasksAttempt::Pending { cause, sticky } => {
                let mut errors = Vec::new();
                if let Some(error) = &self.geometry_task_failure {
                    errors.push(format!("Geometry task shutdown failed: {error}"));
                }
                if let Some(error) = &self.correction_failure {
                    errors.push(error.clone());
                }
                if let Some(error) = sticky.detail() {
                    errors.push(error);
                }
                errors.push(format!("Geometry Mapping retirement pending: {cause}"));
                return Err(GeometryProductionShutdownFailure::unsettled(
                    errors.join("; "),
                    self,
                ));
            }
            lio_owner::GeometryRuntimeAfterTasksAttempt::Settled { terminal } => terminal,
        };
        self.geometry_task_settlement.take();
        let mut errors = Vec::new();
        if let Some(error) = self.geometry_task_failure.take() {
            errors.push(format!("Geometry task shutdown failed: {error}"));
        }
        if let Some(error) = self.correction_failure.take() {
            errors.push(error);
        }
        if let Err(error) = runtime_terminal {
            errors.push(format!("Geometry runtime shutdown failed: {error}"));
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(GeometryProductionShutdownFailure::terminal(
                errors.join("; "),
            ))
        }
    }

    fn emit_correction_settled_once(&mut self) {
        if self.correction_settled_emitted {
            return;
        }
        if let Some(events) = &self.shutdown_events {
            events.emit_completed(lio_owner::GeometryShutdownStage::CorrectionSettled);
        }
        self.correction_settled_emitted = true;
    }
}

async fn shutdown_runtime_owner(
    mut owner: GeometryProductionOwner,
    intended_exit: ExitCode,
) -> ExitCode {
    let mut observed_pending = false;
    let mut budget = FinalOwnerAttemptBudget::default();
    while let Some(attempt) = budget.next_attempt() {
        match owner.shutdown().await {
            Ok(()) if observed_pending => return ExitCode::from(7),
            Ok(()) => return intended_exit,
            Err(error) => {
                let detail = error.to_string();
                let Some(pending_owner) = error.into_pending_owner() else {
                    error!("Geometry production owner shutdown 失败: {}", detail);
                    return ExitCode::from(7);
                };
                observed_pending = true;
                error!(
                    attempt,
                    attempt_limit = FINAL_OWNER_ATTEMPT_LIMIT,
                    "Geometry production owner shutdown 未完成，保留同一 owner: {}",
                    detail,
                );
                if budget.exhausted() {
                    retain_geometry_owner_until_process_exit(
                        GeometryProcessExitRetention::Production(pending_owner),
                    );
                    return ExitCode::from(7);
                }
                owner = *pending_owner;
                tokio::time::sleep(FINAL_OWNER_ATTEMPT_BACKOFF).await;
            }
        }
    }
    unreachable!("final-owner attempt budget is checked after every pending result")
}

async fn rollback_mapping_and_lio(
    mut owner: lio_owner::GeometryRuntimeOwner,
    ingress_gate: Arc<lio_ingress::LioIngressRoutingGate>,
    intended_exit: ExitCode,
) -> ExitCode {
    lio_owner::LioIngressSink::close(ingress_gate.as_ref());
    let mut observed_pending = false;
    let mut budget = FinalOwnerAttemptBudget::default();
    while let Some(attempt) = budget.next_attempt() {
        match owner.shutdown().await {
            Ok(()) if observed_pending => return ExitCode::from(7),
            Ok(()) => return intended_exit,
            Err(error) => {
                let detail = error.to_string();
                let Some(pending_owner) = error.into_pending_owner() else {
                    error!("Geometry Mapping/LIO rollback 失败: {}", detail);
                    return ExitCode::from(7);
                };
                observed_pending = true;
                error!(
                    attempt,
                    attempt_limit = FINAL_OWNER_ATTEMPT_LIMIT,
                    "Geometry Mapping/LIO rollback 未 settled，保留同一 owner: {}",
                    detail,
                );
                if budget.exhausted() {
                    retain_geometry_owner_until_process_exit(
                        GeometryProcessExitRetention::MappingAndLio {
                            owner: pending_owner,
                            ingress_gate,
                        },
                    );
                    return ExitCode::from(7);
                }
                owner = *pending_owner;
                tokio::time::sleep(FINAL_OWNER_ATTEMPT_BACKOFF).await;
            }
        }
    }
    unreachable!("final-owner attempt budget is checked after every pending result")
}

async fn rollback_geometry_startup_and_lio(
    failure: lcm_runtime::GeometryRuntimeStartupFailure,
    lio: lio_owner::LioOwnerHandle,
) -> ExitCode {
    let (error, rollback_owner) = failure.into_parts();
    error!("{}", error);
    eprintln!("{error}");
    let mut rollback_failed = false;
    if let Some(rollback_owner) = rollback_owner {
        let mut rollback_owner = *rollback_owner;
        let mut budget = FinalOwnerAttemptBudget::default();
        while let Some(attempt) = budget.next_attempt() {
            match rollback_owner.shutdown().await {
                Ok(()) => break,
                Err(rollback_error) => {
                    rollback_failed = true;
                    let detail = rollback_error.to_string();
                    let Some(pending_owner) = rollback_error.into_pending_owner() else {
                        error!("Geometry startup task final join 失败: {}", detail);
                        break;
                    };
                    error!(
                        attempt,
                        attempt_limit = FINAL_OWNER_ATTEMPT_LIMIT,
                        "Geometry startup task final join 未完成，保留同一 owner: {}",
                        detail,
                    );
                    if budget.exhausted() {
                        retain_geometry_owner_until_process_exit(
                            GeometryProcessExitRetention::StartupRollbackAndLio {
                                geometry: pending_owner,
                                lio,
                            },
                        );
                        return ExitCode::from(7);
                    }
                    rollback_owner = *pending_owner;
                    tokio::time::sleep(FINAL_OWNER_ATTEMPT_BACKOFF).await;
                }
            }
        }
    }
    match lio.shutdown().await {
        Ok(()) if !rollback_failed => ExitCode::from(4),
        Ok(()) => ExitCode::from(7),
        Err(shutdown_error) => {
            error!("Geometry LIO rollback 失败: {}", shutdown_error);
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

fn shutdown_video_owner(
    owner: &mut video_relay::VideoRelayOwner,
    intended_exit: ExitCode,
) -> ExitCode {
    match owner.shutdown(VIDEO_RELAY_STOP_TIMEOUT) {
        Ok(terminal) if terminal.is_expected_shutdown() => intended_exit,
        Ok(terminal) => {
            error!("视频中转 shutdown 异常: {:?}", terminal);
            ExitCode::from(7)
        }
        Err(error) => {
            error!("视频中转 shutdown 失败: {}", error);
            ExitCode::from(7)
        }
    }
}

fn begin_runtime_cleanup_phase_a(
    shutdown: &server::ShutdownCoordinator,
    runtime_owner: &GeometryProductionOwner,
) {
    shutdown.begin_phase_a();
    runtime_owner.close_ingress();
}

async fn cleanup_runtime_before_rpc(
    shutdown: &server::ShutdownCoordinator,
    mut video_owner: video_relay::VideoRelayOwner,
    runtime_owner: GeometryProductionOwner,
    intended_exit: ExitCode,
) -> ExitCode {
    begin_runtime_cleanup_phase_a(shutdown, &runtime_owner);
    let exit_code = shutdown_video_owner(&mut video_owner, intended_exit);
    shutdown_runtime_owner(runtime_owner, exit_code).await
}

async fn cleanup_runtime(
    rpc_servers: nuwax_rpc::RpcServerGroup,
    mut video_owner: video_relay::VideoRelayOwner,
    runtime_owner: GeometryProductionOwner,
    shutdown: &server::ShutdownCoordinator,
    intended_exit: ExitCode,
) -> ExitCode {
    begin_runtime_cleanup_phase_a(shutdown, &runtime_owner);
    let intended_exit = shutdown_video_owner(&mut video_owner, intended_exit);
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

async fn finish_grpc_after_correction_stop(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    shutdown: &server::ShutdownCoordinator,
) -> Option<GrpcServerResult> {
    let drain = tokio::time::timeout(PROCESS_STOP_DRAIN_TIMEOUT, async {
        tokio::select! {
            result = &mut *server_task => Ok(result),
            () = shutdown.wait_for_process_stop_failure() => Err(()),
        }
    })
    .await;
    match drain {
        Ok(Ok(result)) => Some(result),
        Ok(Err(())) => {
            error!("Process Stop 在 tonic response drain 前失败；强制关闭 gRPC server");
            force_grpc_server_shutdown(server_task, shutdown).await;
            None
        }
        Err(_) => {
            error!("Process Stop 未在总截止时间内完成 tonic response drain；强制关闭");
            force_grpc_server_shutdown(server_task, shutdown).await;
            None
        }
    }
}

async fn next_runtime_terminal(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    video_owner: &video_relay::VideoRelayOwner,
    runtime_owner: &mut GeometryProductionOwner,
) -> RuntimeTerminal {
    let candidate = tokio::select! {
        biased;
        terminal = runtime_owner.wait_owned_terminal() => terminal,
        terminal = video_owner.wait_terminal() => RuntimeTerminal::Video(Box::new(terminal)),
        result = &mut *server_task => RuntimeTerminal::Grpc(result),
    };
    runtime_owner.prefer_completed_failure(candidate)
}

async fn completed_runtime_terminal(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    video_owner: &video_relay::VideoRelayOwner,
    runtime_owner: &mut GeometryProductionOwner,
) -> Option<RuntimeTerminal> {
    if let Some(terminal) = runtime_owner.completed_failure_terminal() {
        return Some(terminal);
    }
    if video_owner.is_finished() {
        let candidate = RuntimeTerminal::Video(Box::new(video_owner.wait_terminal().await));
        return Some(runtime_owner.prefer_completed_failure(candidate));
    }
    if server_task.is_finished() {
        let candidate = RuntimeTerminal::Grpc((&mut *server_task).await);
        return Some(runtime_owner.prefer_completed_failure(candidate));
    }
    if runtime_owner.is_geometry_finished() {
        let candidate =
            RuntimeTerminal::GeometryTasks(runtime_owner.wait_geometry_terminal().await);
        return Some(runtime_owner.prefer_completed_failure(candidate));
    }
    if runtime_owner.is_lio_finished() {
        let candidate = RuntimeTerminal::Lio(runtime_owner.wait_lio_terminal().await);
        return Some(runtime_owner.prefer_completed_failure(candidate));
    }
    if runtime_owner.is_correction_finished() {
        let candidate = RuntimeTerminal::Correction(runtime_owner.wait_correction_terminal().await);
        return Some(runtime_owner.prefer_completed_failure(candidate));
    }
    None
}

async fn runtime_terminal_exit_code(
    terminal: RuntimeTerminal,
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    shutdown: &server::ShutdownCoordinator,
) -> ExitCode {
    match terminal {
        RuntimeTerminal::Video(terminal) => {
            error!(?terminal, "视频中转在 runtime 运行期间异常退出");
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
        RuntimeTerminal::Grpc(result) if shutdown.has_shutdown_failed() => {
            grpc_server_exit_code(result, true)
        }
        RuntimeTerminal::Grpc(Ok(Ok(()))) if !shutdown.is_shutdown_requested() => {
            error!("gRPC server 在 runtime 运行期间无 shutdown 请求即正常返回");
            ExitCode::from(6)
        }
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
        RuntimeTerminal::Correction(Ok(())) => {
            match finish_grpc_after_correction_stop(server_task, shutdown).await {
                Some(Ok(Ok(())))
                    if shutdown.is_shutdown_requested() && !shutdown.has_shutdown_failed() =>
                {
                    ExitCode::from(0)
                }
                Some(result) => {
                    error!(
                        ?result,
                        shutdown_requested = shutdown.is_shutdown_requested(),
                        shutdown_failed = shutdown.has_shutdown_failed(),
                        "Process Stop correction terminal lacked a successful tonic response drain"
                    );
                    ExitCode::from(7)
                }
                None => ExitCode::from(7),
            }
        }
        RuntimeTerminal::Correction(Err(error)) => {
            error!("Geometry correction owner 异常退出: {}", error);
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
        RuntimeTerminal::GeometryTasks(result) => {
            match result {
                Ok(()) => error!("Geometry task supervisor exited without a root shutdown"),
                Err(error) => error!(%error, "Geometry task supervisor failed"),
            }
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
        RuntimeTerminal::ProcessStopFailed => {
            error!("Geometry Process Stop 在 tonic response drain 前失败或被取消");
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
        RuntimeTerminal::ReplayPoison(detail) => {
            error!(detail, "Geometry deferred LIO replay poisoned");
            force_grpc_server_shutdown(server_task, shutdown).await;
            ExitCode::from(7)
        }
    }
}

async fn startup_stop_exit_code(
    server_task: &mut tokio::task::JoinHandle<Result<(), tonic::transport::Error>>,
    video_owner: &video_relay::VideoRelayOwner,
    runtime_owner: &mut GeometryProductionOwner,
    shutdown: &server::ShutdownCoordinator,
) -> ExitCode {
    match tokio::time::timeout(
        STARTUP_STOP_TIMEOUT,
        next_runtime_terminal(server_task, video_owner, runtime_owner),
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

fn exit_code_after_ready_cleanup(exit_code: ExitCode, ready_remove_failed: bool) -> ExitCode {
    if ready_remove_failed && exit_code == ExitCode::SUCCESS {
        ExitCode::from(5)
    } else {
        exit_code
    }
}

fn print_requested_release_artifacts() -> Option<ExitCode> {
    let mut args = std::env::args_os();
    let _program = args.next();
    if args.next().as_deref() != Some(std::ffi::OsStr::new("--print-release-artifacts")) {
        return None;
    }
    if args.next().is_some() {
        eprintln!("--print-release-artifacts 不接受其它参数");
        return Some(ExitCode::from(2));
    }

    let payload = serde_json::json!({
        "schema_version": "nuwax.geometry.embedded-artifacts.v1",
        "artifacts": [
            {
                "role": "native_pgo_payload",
                "path": nuwax_nav_pgo_bridge::discovery_native_executable(),
            },
            {
                "role": "group_guardian",
                "path": nuwax_nav_pgo_bridge::discovery_group_guardian_executable(),
            },
        ],
    });
    match serde_json::to_string(&payload) {
        Ok(encoded) => {
            println!("{encoded}");
            Some(ExitCode::SUCCESS)
        }
        Err(error) => {
            eprintln!("序列化 release artifact discovery 失败: {error}");
            Some(ExitCode::from(2))
        }
    }
}

fn main() -> ExitCode {
    if let Some(exit_code) = print_requested_release_artifacts() {
        return exit_code;
    }

    match video_relay::run_requested_helper() {
        Ok(true) => return ExitCode::SUCCESS,
        Ok(false) => {}
        Err(error) => {
            eprintln!("视频中转 helper 失败: {error}");
            return ExitCode::from(1);
        }
    }

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
    let exit_code = runtime.block_on(run(lcm_backend, preflight));
    finalize_geometry_runtime(runtime, exit_code)
}

async fn run(lcm_backend: lcm_runtime::LcmBackendKind, preflight: StartupPreflight) -> ExitCode {
    let StartupPreflight {
        owner_profile,
        mapping_backend,
        channel_bindings,
        path_binding,
        frame_bindings,
        correction,
        correction_provider,
        lio_bindings,
        lio_source,
    } = preflight;
    let tf_binding = channel_bindings.binding(channel_registry::ChannelKey::Tf);
    tracing_subscriber::fmt()
        .with_env_filter("nuwax_geometry_runtime=info,nuwax_protocol=warn,nuwax_transport=warn,nuwax_codec=warn,nuwax_tf=warn")
        .with_target(false)
        .init();
    #[cfg(unix)]
    let unix_shutdown_signals = match UnixShutdownSignals::register() {
        Ok(signals) => signals,
        Err(error) => {
            error!("注册 Unix SIGINT/SIGTERM handler 失败: {}", error);
            return ExitCode::from(7);
        }
    };
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
    // LIO -> Mapping -> Correction owner activation
    // ----------------------------------------------------------------------
    let prepared_runtime = match lcm_runtime::prepare_geometry_runtime(lcm_backend) {
        Ok(prepared) => prepared,
        Err(e) => {
            error!("{}", e);
            eprintln!("{e}");
            return ExitCode::from(4);
        }
    };

    let runtime_generation = match i64::try_from(prepared_runtime.runtime_generation()) {
        Ok(generation) if generation > 0 => generation,
        Ok(generation) => {
            error!("LIO runtime generation 必须为正数: {}", generation);
            return ExitCode::from(4);
        }
        Err(_) => {
            error!(
                "LIO runtime generation 超出 i64 范围: {}",
                prepared_runtime.runtime_generation()
            );
            return ExitCode::from(4);
        }
    };
    let relocalization_mode_fence =
        Arc::new(correction_coordinator::RelocalizationModeFence::default());
    let (lio_spec, pending_ingress, rpc_point_cloud_binding): (
        lio_owner::LioOwnerSpec,
        PendingLioIngress,
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
            let probe = Arc::new(lio_ingress::CommonLioIngress::new(ingress_spec.clone()));
            let rpc_point_cloud_binding = Some(lio_bindings.legacy_cloud.clone());
            (
                lio_bindings.clone().into(),
                PendingLioIngress::Device {
                    spec: ingress_spec,
                    probe,
                },
                rpc_point_cloud_binding,
            )
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
            let probe = match lio_ingress::NativeLioIngress::without_runtime(
                ingress_spec.clone(),
                native_runtime.body_from_sensor.clone(),
                native_runtime.pair_max_skew,
            ) {
                Ok(ingress) => Arc::new(ingress),
                Err(error) => {
                    error!("Native LIO ingress 配置失败: {}", error);
                    return ExitCode::from(4);
                }
            };
            if let Err(error) =
                probe.bind_relocalization_mode_fence(Arc::clone(&relocalization_mode_fence))
            {
                error!("Native LIO correction mode fence 绑定失败: {}", error);
                return ExitCode::from(4);
            }
            (
                lio_owner::LioOwnerSpec::native(lio_bindings.clone(), native_runtime.clone()),
                PendingLioIngress::Native {
                    spec: ingress_spec,
                    probe,
                    body_from_sensor: native_runtime.body_from_sensor,
                    pair_max_skew: native_runtime.pair_max_skew,
                },
                Some(lio_bindings.legacy_cloud.clone()),
            )
        }
    };
    let ingress_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(
        pending_ingress.probe(),
    ));
    let lio = match lio_owner::LioOwnerHandle::start(
        prepared_runtime.transport(),
        lio_spec,
        runtime_generation,
        ingress_gate.clone(),
    )
    .await
    {
        Ok(lio) => lio,
        Err(error) => {
            error!("Geometry LIO owner 启动失败: {}", error);
            return ExitCode::from(4);
        }
    };
    let spawned_runtime = match lcm_runtime::spawn_geometry_runtime(
        prepared_runtime,
        mapping_backend,
        channel_bindings,
        path_binding,
        frame_bindings,
    )
    .await
    {
        Ok(spawned) => spawned,
        Err(error) => {
            return rollback_geometry_startup_and_lio(error, lio).await;
        }
    };
    let bridge = spawned_runtime.handle.clone();
    let runtime_owner = lio_owner::GeometryRuntimeOwner::new(spawned_runtime, lio);
    let correction_identity = correction_coordinator::CorrectionOwnerRuntimeIdentity {
        runtime_generation: runtime_generation as u64,
        backend_generation: runtime_generation as u64,
        correction_lease: correction_coordinator::CorrectionLeaseId {
            boot_nonce: uuid::Uuid::new_v4().as_u128(),
            generation: runtime_generation as u64,
        },
    };
    let correction_applier = match lcm_runtime::GeometryCanonicalCorrectionApplier::new(
        correction_identity.clone(),
        lio_bindings.map_frame.clone(),
        lio_bindings.odom_frame.clone(),
        lio_bindings.body_frame.clone(),
        bridge.state.clone(),
        bridge.transport.clone(),
        tf_binding.clone(),
    ) {
        Ok(applier) => Arc::new(applier),
        Err(error) => {
            error!("Geometry correction applier 配置失败: {}", error);
            return rollback_mapping_and_lio(runtime_owner, ingress_gate, ExitCode::from(4)).await;
        }
    };
    let correction_sink = match correction_coordinator::GeometryCorrectionSink::new_for_spec(
        correction_identity.clone(),
        &correction,
        lio_bindings.map_frame.clone(),
        lio_bindings.odom_frame.clone(),
        lio_bindings.body_frame.clone(),
        correction_applier.clone(),
    ) {
        Ok(sink) => Arc::new(sink),
        Err(error) => {
            error!("Geometry correction sink 配置失败: {}", error);
            return rollback_mapping_and_lio(runtime_owner, ingress_gate, ExitCode::from(4)).await;
        }
    };
    let correction_owner = match correction_coordinator::CorrectionTransportOwner::start_with_relocalization_mode_fence(
        correction,
        correction_identity,
        &correction_provider,
        correction_sink,
        relocalization_mode_fence,
    )
    .await
    {
        Ok(owner) => owner,
        Err(error) => {
            error!("Geometry correction owner 启动失败: {}", error);
            return rollback_mapping_and_lio(
                runtime_owner,
                ingress_gate,
                ExitCode::from(4),
            )
            .await;
        }
    };
    let correction_phase_a = correction_owner.phase_a_control();
    let relocalization_ingress = correction_owner.relocalization_ingress();
    let prepared_ingress = match pending_ingress.bind_runtime(
        bridge.clone(),
        lio_bindings.public_odometry.clone(),
        lio_bindings.public_cloud.clone(),
        tf_binding,
    ) {
        Ok(ingress) => ingress,
        Err(error) => {
            error!("Geometry LIO runtime handoff 绑定失败: {}", error);
            return shutdown_runtime_owner(
                GeometryProductionOwner::new(
                    runtime_owner,
                    correction_owner,
                    ingress_gate,
                    correction_phase_a,
                    None,
                ),
                ExitCode::from(4),
            )
            .await;
        }
    };
    let bound_ingress =
        prepared_ingress.bind_correction(correction_owner.ingress(), correction_applier.clone());
    let deferred_replay = match bound_ingress {
        BoundLioIngress::Device(ingress) => {
            if let Err(error) = ingress_gate.bind_and_open(ingress) {
                error!("Geometry LIO routing gate 激活失败: {}", error);
                return shutdown_runtime_owner(
                    GeometryProductionOwner::new(
                        runtime_owner,
                        correction_owner,
                        ingress_gate,
                        correction_phase_a,
                        None,
                    ),
                    ExitCode::from(4),
                )
                .await;
            }
            None
        }
        BoundLioIngress::Native {
            sink,
            ingress,
            pair,
        } => {
            let token = match ingress_gate.bind_deferred(sink) {
                Ok(token) => token,
                Err(error) => {
                    error!("Geometry native LIO deferred gate 绑定失败: {}", error);
                    return shutdown_runtime_owner(
                        GeometryProductionOwner::new(
                            runtime_owner,
                            correction_owner,
                            ingress_gate,
                            correction_phase_a,
                            None,
                        ),
                        ExitCode::from(4),
                    )
                    .await;
                }
            };
            let replay = lio_ingress::NativeDeferredReplayControl::new(
                ingress,
                ingress_gate.clone(),
                correction_owner.ingress(),
                correction_applier,
                correction_phase_a.clone(),
                token,
                *pair,
            );
            if let Err(error) = replay.publish_startup_odometry().await {
                error!(
                    "Geometry native LIO startup odometry handoff 失败: {}",
                    error
                );
                return shutdown_runtime_owner(
                    GeometryProductionOwner::new(
                        runtime_owner,
                        correction_owner,
                        ingress_gate,
                        correction_phase_a,
                        Some(replay),
                    ),
                    ExitCode::from(4),
                )
                .await;
            }
            Some(replay)
        }
    };
    let mut runtime_owner = GeometryProductionOwner::new(
        runtime_owner,
        correction_owner,
        ingress_gate.clone(),
        correction_phase_a.clone(),
        deferred_replay.clone(),
    )
    .with_shutdown_events(lio_owner::GeometryShutdownEventEmitter::new(
        runtime_generation as u64,
    ));
    let bridge = runtime_owner.handle();

    // ----------------------------------------------------------------------
    // External exposure begins only after all three owner slots are READY.
    // ----------------------------------------------------------------------
    let video_owner = match video_relay::spawn_video_relay(&video_shm_path, &video_uds_path) {
        Ok(owner) => owner,
        Err(e) => {
            error!("视频中转启动失败: {}", e);
            return shutdown_runtime_owner(runtime_owner, ExitCode::from(3)).await;
        }
    };
    info!(
        "视频中转已启动: shm={:?}, uds={:?}",
        video_shm_path, video_uds_path
    );

    let geometry_task_control = bridge
        .task_control()
        .expect("production Geometry runtime must expose task admission control");
    let (shutdown, shutdown_listener) = server::production_shutdown_channel_with_task_control(
        ingress_gate,
        correction_phase_a,
        deferred_replay.clone(),
        geometry_task_control,
    );
    let rpc_context = match bridge.transport.shared_lcm_context() {
        Some(context) => context,
        None => {
            error!("Geometry LCM transport 未暴露真实 owner context，RPC 无法启动");
            return cleanup_runtime_before_rpc(
                &shutdown,
                video_owner,
                runtime_owner,
                ExitCode::from(7),
            )
            .await;
        }
    };
    let service = server::GeometryRuntimeService::new_with_shutdown_and_point_cloud_binding(
        bridge,
        shutdown.clone(),
        rpc_point_cloud_binding,
        relocalization_ingress,
    );
    let mapping_cloud_owner = service.mapping_cloud_owner();
    runtime_owner = runtime_owner.with_mapping_cloud_owner(mapping_cloud_owner);
    let readiness_service = service.clone();
    let rpc_transport: Arc<dyn nuwax_rpc::RpcTransport> =
        Arc::new(nuwax_rpc::LcmRpcTransport::from_context(rpc_context));
    let rpc_servers = match rpc_runtime::serve_geometry_rpc(service.clone(), rpc_transport).await {
        Ok(servers) => servers,
        Err(e) => {
            error!("Geometry LCM RPC server 启动失败: {}", e);
            return cleanup_runtime_before_rpc(
                &shutdown,
                video_owner,
                runtime_owner,
                ExitCode::from(7),
            )
            .await;
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
    let server_service = service.clone();
    let mut server_task = tokio::spawn(async move {
        Server::builder()
            .add_service(
                proto::geometry_runtime_server::GeometryRuntimeServer::new(server_service)
                    .max_decoding_message_size(GRPC_MESSAGE_LIMIT_BYTES)
                    .max_encoding_message_size(GRPC_MESSAGE_LIMIT_BYTES),
            )
            .serve_with_incoming_shutdown(incoming, graceful_shutdown)
            .await
    });
    #[cfg(unix)]
    let _signal_forwarder = tokio::spawn(unix_shutdown_signals.forward_to(service));

    let startup_check = wait_for_grpc_health(addr);
    tokio::pin!(startup_check);
    tokio::select! {
        biased;
        terminal = next_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner) => {
            let exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
            return cleanup_runtime(
                rpc_servers,
                video_owner,
                runtime_owner,
                &shutdown,
                exit_code,
            )
            .await;
        }
        check = &mut startup_check => {
            match check {
                Ok(StartupHealth::Ready) => {}
                Ok(StartupHealth::ShutdownInProgress) => {
                    info!("startup health 期间收到正常 Stop，等待有界退出");
                    let exit_code = startup_stop_exit_code(
                        &mut server_task,
                        &video_owner,
                        &mut runtime_owner,
                        &shutdown,
                    )
                    .await;
                    return cleanup_runtime(
                        rpc_servers,
                        video_owner,
                        runtime_owner,
                        &shutdown,
                        exit_code,
                    )
                    .await;
                }
                Err(error) => {
                    error!("{}", error);
                    force_grpc_server_shutdown(&mut server_task, &shutdown).await;
                    return cleanup_runtime(
                        rpc_servers,
                        video_owner,
                        runtime_owner,
                        &shutdown,
                        ExitCode::from(6),
                    )
                    .await;
                }
            }
        }
    }

    if let Some(terminal) =
        completed_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner).await
    {
        let exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
        return cleanup_runtime(
            rpc_servers,
            video_owner,
            runtime_owner,
            &shutdown,
            exit_code,
        )
        .await;
    }

    if let Some(parent) = std::path::Path::new(&ready_path).parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            error!("ready file 父目录创建失败: {:?}", e);
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(
                rpc_servers,
                video_owner,
                runtime_owner,
                &shutdown,
                ExitCode::from(5),
            )
            .await;
        }
    }

    if let Some(terminal) =
        completed_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner).await
    {
        let exit_code = runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await;
        return cleanup_runtime(
            rpc_servers,
            video_owner,
            runtime_owner,
            &shutdown,
            exit_code,
        )
        .await;
    }

    let mut ready_lease = match readiness_service
        .run_if_running(|| ReadyFileLease::acquire(ready_path.clone(), port))
        .await
    {
        Ok(Ok(lease)) => lease,
        Ok(Err(error)) => {
            error!("ready file 写入失败: {:?}", error);
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(
                rpc_servers,
                video_owner,
                runtime_owner,
                &shutdown,
                ExitCode::from(5),
            )
            .await;
        }
        Err(phase) if matches!(phase.as_str(), "stopping" | "stopped") => {
            info!(phase, "ready 发布前收到正常 Stop，等待有界退出");
            let exit_code = startup_stop_exit_code(
                &mut server_task,
                &video_owner,
                &mut runtime_owner,
                &shutdown,
            )
            .await;
            return cleanup_runtime(
                rpc_servers,
                video_owner,
                runtime_owner,
                &shutdown,
                exit_code,
            )
            .await;
        }
        Err(phase) => {
            error!(phase, "ready 发布前 runtime 已离开 running 状态");
            force_grpc_server_shutdown(&mut server_task, &shutdown).await;
            return cleanup_runtime(
                rpc_servers,
                video_owner,
                runtime_owner,
                &shutdown,
                ExitCode::from(7),
            )
            .await;
        }
    };

    if let Some(terminal) =
        completed_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner).await
    {
        let ready_remove_failed = match ready_lease.revoke() {
            Ok(()) => false,
            Err(error) => {
                error!("删除 ready file 失败: {:?}", error);
                true
            }
        };
        drop(ready_lease);
        let exit_code = exit_code_after_ready_cleanup(
            runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await,
            ready_remove_failed,
        );
        return cleanup_runtime(
            rpc_servers,
            video_owner,
            runtime_owner,
            &shutdown,
            exit_code,
        )
        .await;
    }

    info!("gRPC HealthCheck 已通过, ready file 写入: {:?}", ready_path);

    let (terminal, mut ready_remove_failed) = tokio::select! {
        terminal = next_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner) => {
            (terminal, false)
        }
        ready_remove_failed = revoke_ready_after_readiness_signal(&shutdown, &mut ready_lease) => {
            (
                next_runtime_terminal(&mut server_task, &video_owner, &mut runtime_owner).await,
                ready_remove_failed,
            )
        }
    };
    if let Err(error) = ready_lease.revoke() {
        error!("删除 ready file 失败: {:?}", error);
        ready_remove_failed = true;
    }
    drop(ready_lease);

    let exit_code = exit_code_after_ready_cleanup(
        runtime_terminal_exit_code(terminal, &mut server_task, &shutdown).await,
        ready_remove_failed,
    );
    cleanup_runtime(
        rpc_servers,
        video_owner,
        runtime_owner,
        &shutdown,
        exit_code,
    )
    .await
}

#[cfg(test)]
pub(crate) mod lifecycle_tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    static PROCESS_EXIT_RETENTION_TEST_LOCK: tokio::sync::Mutex<()> =
        tokio::sync::Mutex::const_new(());

    struct LifecycleCorrectionSink {
        block_stop: bool,
        stop_called: AtomicBool,
        stop_calls: std::sync::atomic::AtomicU64,
        stop_started: tokio::sync::Notify,
        stop_release: tokio::sync::Notify,
        apply_count: std::sync::atomic::AtomicU64,
    }

    impl LifecycleCorrectionSink {
        fn new(block_stop: bool) -> Self {
            Self {
                block_stop,
                stop_called: AtomicBool::new(false),
                stop_calls: std::sync::atomic::AtomicU64::new(0),
                stop_started: tokio::sync::Notify::new(),
                stop_release: tokio::sync::Notify::new(),
                apply_count: std::sync::atomic::AtomicU64::new(0),
            }
        }
    }

    #[async_trait::async_trait]
    impl correction_coordinator::CorrectionCandidateSink for LifecycleCorrectionSink {
        async fn apply_candidate(
            &self,
            _candidate: correction_coordinator::CorrectionCandidate,
            _cancellation: correction_coordinator::CorrectionRequestCancellation,
        ) -> Result<correction_coordinator::CorrectionDisposition, String> {
            self.apply_count.fetch_add(1, Ordering::AcqRel);
            Ok(correction_coordinator::CorrectionDisposition::Applied)
        }

        async fn stop(&self) {
            self.stop_called.store(true, Ordering::Release);
            self.stop_calls.fetch_add(1, Ordering::AcqRel);
            self.stop_started.notify_waiters();
            if self.block_stop {
                self.stop_release.notified().await;
            }
        }
    }

    struct LifecycleLioSink {
        correction: correction_coordinator::CorrectionFrameIngress,
        fail_ready: bool,
        ready_seen: AtomicBool,
        closed: AtomicBool,
    }

    impl LifecycleLioSink {
        fn new(
            correction: correction_coordinator::CorrectionFrameIngress,
            fail_ready: bool,
        ) -> Self {
            Self {
                correction,
                fail_ready,
                ready_seen: AtomicBool::new(false),
                closed: AtomicBool::new(false),
            }
        }
    }

    #[async_trait::async_trait]
    impl lio_owner::LioIngressSink for LifecycleLioSink {
        fn close(&self) {
            self.closed.store(true, Ordering::Release);
        }

        async fn wait_startup_ready(
            &self,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            if !self.correction.is_open() {
                return Err("correction admission closed before LIO READY".to_owned());
            }
            self.ready_seen.store(true, Ordering::Release);
            if self.fail_ready {
                Err("injected later-slot LIO READY failure".to_owned())
            } else {
                Ok(())
            }
        }

        async fn accept_public_odometry(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_public_cloud(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_odometry(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }
    }

    struct PreCorrectionLioSink {
        fail_ready: bool,
        ready_seen: AtomicBool,
        closed: AtomicBool,
        readiness_probe: std::sync::Mutex<Option<server::ShutdownCoordinator>>,
        readiness_revoked_before_close: AtomicBool,
    }

    impl PreCorrectionLioSink {
        fn new(fail_ready: bool) -> Self {
            Self {
                fail_ready,
                ready_seen: AtomicBool::new(false),
                closed: AtomicBool::new(false),
                readiness_probe: std::sync::Mutex::new(None),
                readiness_revoked_before_close: AtomicBool::new(false),
            }
        }

        fn observe_readiness(&self, shutdown: server::ShutdownCoordinator) {
            *self.readiness_probe.lock().expect("readiness probe lock") = Some(shutdown);
        }
    }

    #[async_trait::async_trait]
    impl lio_owner::LioIngressSink for PreCorrectionLioSink {
        fn close(&self) {
            if self
                .readiness_probe
                .lock()
                .expect("readiness probe lock")
                .as_ref()
                .is_some_and(server::ShutdownCoordinator::is_readiness_revoked)
            {
                self.readiness_revoked_before_close
                    .store(true, Ordering::Release);
            }
            self.closed.store(true, Ordering::Release);
        }

        async fn wait_startup_ready(
            &self,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.ready_seen.store(true, Ordering::Release);
            if self.fail_ready {
                Err("injected LIO READY failure before Mapping/Correction startup".to_owned())
            } else {
                Ok(())
            }
        }

        async fn accept_public_odometry(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_public_cloud(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_odometry(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: nuwax_protocol::RawLcmFrame,
            _shutdown_rx: &mut tokio::sync::watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }
    }

    struct CleanupRpcSubscription;

    #[async_trait::async_trait]
    impl nuwax_rpc::RpcSubscription for CleanupRpcSubscription {
        async fn recv(&mut self) -> nuwax_rpc::RpcResult<Vec<u8>> {
            std::future::pending().await
        }
    }

    struct CleanupRpcTransport;

    #[async_trait::async_trait]
    impl nuwax_rpc::RpcTransport for CleanupRpcTransport {
        async fn publish(&self, _channel: &str, _payload: Vec<u8>) -> nuwax_rpc::RpcResult<()> {
            Ok(())
        }

        async fn subscribe(
            &self,
            _channel: &str,
        ) -> nuwax_rpc::RpcResult<Box<dyn nuwax_rpc::RpcSubscription>> {
            Ok(Box::new(CleanupRpcSubscription))
        }

        fn backend_name(&self) -> &'static str {
            "cleanup-rpc-test"
        }
    }

    fn lifecycle_binding(
        channel: &str,
        message_type: &str,
    ) -> (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType) {
        (
            nuwax_protocol::LcmTopic::new(channel).expect("lifecycle test topic"),
            nuwax_protocol::LcmMessageType::new(message_type).expect("lifecycle test message type"),
        )
    }

    fn lifecycle_lio_spec(tag: &str) -> lio_owner::DeviceLioOwnerSpec {
        static NEXT_SCOPE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
        let scope = NEXT_SCOPE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let tag_checksum = tag.bytes().fold(0_u16, |checksum, byte| {
            checksum.wrapping_mul(31).wrapping_add(u16::from(byte))
        });
        let prefix = format!("/g4/{:x}/{scope:x}/{tag_checksum:x}", std::process::id());
        lio_owner::DeviceLioOwnerSpec {
            public_odometry: lifecycle_binding(
                &format!("{prefix}/public-odom"),
                "nuwax_msgs.LioOdometryEnvelope",
            ),
            public_cloud: lifecycle_binding(
                &format!("{prefix}/public-cloud"),
                "nuwax_msgs.LioCloudEnvelope",
            ),
            status: lifecycle_binding(&format!("{prefix}/status"), "nuwax_msgs.LioStatus"),
            legacy_odometry: lifecycle_binding(
                &format!("{prefix}/source-odom"),
                "nav_msgs.Odometry",
            ),
            legacy_cloud: lifecycle_binding(
                &format!("{prefix}/source-cloud"),
                "sensor_msgs.PointCloud2",
            ),
            sensor_frame: "lidar".into(),
            body_frame: "base_link".into(),
            odom_frame: "odom".into(),
            map_frame: "map".into(),
        }
    }

    fn lifecycle_correction_spec() -> correction_coordinator::CorrectionOwnerSpec {
        correction_coordinator::CorrectionOwnerSpec::LibraryPgo {
            map_frame_id: "map".into(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            pipeline: Default::default(),
        }
    }

    fn lifecycle_correction_identity(
        generation: u64,
    ) -> correction_coordinator::CorrectionOwnerRuntimeIdentity {
        correction_coordinator::CorrectionOwnerRuntimeIdentity {
            runtime_generation: generation,
            backend_generation: generation,
            correction_lease: correction_coordinator::CorrectionLeaseId {
                boot_nonce: uuid::Uuid::new_v4().as_u128(),
                generation,
            },
        }
    }

    pub(crate) async fn start_lio_before_geometry_for_tests(
        transport: nuwax_transport::LcmTransport,
        generation: u64,
        tag: &str,
    ) -> (lio_owner::LioOwnerHandle, lio_owner::LioShutdownControl) {
        let ingress = Arc::new(PreCorrectionLioSink::new(false));
        let gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(ingress.clone()));
        let lio = lio_owner::LioOwnerHandle::start(
            transport,
            lifecycle_lio_spec(tag),
            i64::try_from(generation).expect("startup rollback generation fits i64"),
            gate,
        )
        .await
        .expect("LIO READY before Geometry production topology startup");
        assert!(ingress.ready_seen.load(Ordering::Acquire));
        let shutdown = lio.shutdown_control_for_tests();
        (lio, shutdown)
    }

    pub(crate) struct PendingGeometryStartupAndLioForTests {
        geometry: Box<lcm_runtime::GeometryRuntimeStartupRollbackOwner>,
        lio: lio_owner::LioOwnerHandle,
    }

    pub(crate) async fn first_geometry_startup_and_lio_rollback_attempt_for_tests(
        failure: lcm_runtime::GeometryRuntimeStartupFailure,
        lio: lio_owner::LioOwnerHandle,
    ) -> Option<PendingGeometryStartupAndLioForTests> {
        let (_, rollback_owner) = failure.into_parts();
        if let Some(rollback_owner) = rollback_owner {
            if let Err(error) = rollback_owner.shutdown().await {
                if let Some(geometry) = error.into_pending_owner() {
                    return Some(PendingGeometryStartupAndLioForTests { geometry, lio });
                }
                lio.shutdown()
                    .await
                    .expect("terminal Geometry rollback failure still shuts down LIO");
                panic!("Geometry startup rollback failed without a retained owner");
            }
        }
        lio.shutdown()
            .await
            .expect("settled Geometry startup rollback shuts down LIO");
        None
    }

    impl PendingGeometryStartupAndLioForTests {
        pub(crate) async fn final_join(self) {
            let Self { geometry, lio } = self;
            match geometry.shutdown().await {
                Ok(()) => {}
                Err(error) => {
                    assert!(
                        error.into_pending_owner().is_none(),
                        "Geometry startup rollback remained unsettled after release"
                    );
                }
            }
            lio.shutdown()
                .await
                .expect("LIO joins after Geometry startup rollback settles");
        }
    }

    pub(crate) struct LifecycleProductionFixture {
        _geometry_topology: lcm_runtime::ProductionTopologyTestFixture,
        owner: Option<GeometryProductionOwner>,
        handle: lcm_runtime::GeometryRuntimeHandle,
        mapping: Arc<parking_lot::Mutex<mapping_adapter::MappingAdapter>>,
        correction_sink: Arc<LifecycleCorrectionSink>,
        correction_ingress: correction_coordinator::CorrectionFrameIngress,
        correction_binding: (nuwax_protocol::LcmTopic, nuwax_protocol::LcmMessageType),
        lio_sink: Arc<PreCorrectionLioSink>,
        lio_shutdown: lio_owner::LioShutdownControl,
        ingress_gate: Arc<lio_ingress::LioIngressRoutingGate>,
        shutdown: server::ShutdownCoordinator,
        shutdown_listener: server::ShutdownListener,
        shutdown_events: lio_owner::GeometryShutdownEventRecorder,
        provider: String,
        cloud_path: PathBuf,
        occupancy_path: PathBuf,
    }

    pub(crate) async fn lifecycle_production_fixture(
        generation: u64,
        tag: &str,
        block_correction_stop: bool,
    ) -> LifecycleProductionFixture {
        let topology_setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|_| {}).await;
        lifecycle_production_fixture_with_setup(
            generation,
            tag,
            block_correction_stop,
            topology_setup,
        )
        .await
    }

    pub(crate) async fn lifecycle_production_fixture_with_setup(
        generation: u64,
        tag: &str,
        block_correction_stop: bool,
        topology_setup: lcm_runtime::ProductionTopologyTestSetup,
    ) -> LifecycleProductionFixture {
        let transport = topology_setup.transport();
        let lio_probe = Arc::new(PreCorrectionLioSink::new(false));
        let ingress_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(lio_probe.clone()));
        let lio = lio_owner::LioOwnerHandle::start(
            transport,
            lifecycle_lio_spec(tag),
            i64::try_from(generation).expect("lifecycle generation fits i64"),
            ingress_gate.clone(),
        )
        .await
        .expect("LIO subscription ACK and READY before later owners");
        assert!(lio_probe.ready_seen.load(Ordering::Acquire));
        let lio_shutdown = lio.shutdown_control_for_tests();

        let mut geometry_topology = match topology_setup.spawn().await {
            Ok(topology) => topology,
            Err(mut failure) => {
                let detail = failure.take_failure().to_string();
                lio.shutdown()
                    .await
                    .expect("unexpected Geometry startup failure must settle LIO");
                panic!("production topology spawn after LIO READY failed: {detail}");
            }
        };
        let (cloud_path, occupancy_path) = geometry_topology.product_paths();
        let spawned = geometry_topology.take_spawned();
        let handle = spawned.handle.clone();
        let provider = handle
            .transport
            .shared_lcm_context()
            .expect("Geometry real LCM context")
            .provider()
            .to_owned();

        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        mapping
            .lock()
            .start_session(
                &format!("lifecycle-session-{generation}"),
                &format!("lifecycle-map-{generation}"),
            )
            .expect("Mapping starts after LIO READY");
        assert!(mapping.lock().snapshot().active);

        let correction_sink = Arc::new(LifecycleCorrectionSink::new(block_correction_stop));
        let correction_owner =
            correction_coordinator::CorrectionTransportOwner::start_with_relocalization_mode_fence(
                lifecycle_correction_spec(),
                lifecycle_correction_identity(generation),
                &provider,
                correction_sink.clone(),
                Arc::new(correction_coordinator::RelocalizationModeFence::default()),
            )
            .await
            .expect("Correction starts only after LIO and Mapping");
        assert!(correction_owner.has_independent_real_contexts());
        let correction_binding = correction_owner
            .binding()
            .expect("generation-private correction binding");
        let correction_ingress = correction_owner.ingress();
        let correction_phase_a = correction_owner.phase_a_control();
        let lio_sink = Arc::new(PreCorrectionLioSink::new(false));
        ingress_gate
            .bind_and_open(lio_sink.clone())
            .expect("open runtime LIO route only after Correction starts");
        assert!(lio_probe.closed.load(Ordering::Acquire));
        assert!(ingress_gate.is_open());
        let (shutdown, shutdown_listener) = server::production_shutdown_channel_with_task_control(
            ingress_gate.clone(),
            correction_phase_a.clone(),
            None,
            handle
                .task_control()
                .expect("lifecycle fixture owns Geometry task admission"),
        );
        lio_sink.observe_readiness(shutdown.clone());
        let (shutdown_emitter, shutdown_events) =
            lio_owner::GeometryShutdownEventEmitter::recording(generation);
        let owner = GeometryProductionOwner::new(
            lio_owner::GeometryRuntimeOwner::new(spawned, lio),
            correction_owner,
            ingress_gate.clone(),
            correction_phase_a,
            None,
        )
        .with_shutdown_events(shutdown_emitter);

        LifecycleProductionFixture {
            _geometry_topology: geometry_topology,
            owner: Some(owner),
            handle,
            mapping,
            correction_sink,
            correction_ingress,
            correction_binding,
            lio_sink,
            lio_shutdown,
            ingress_gate,
            shutdown,
            shutdown_listener,
            shutdown_events,
            provider,
            cloud_path,
            occupancy_path,
        }
    }

    impl LifecycleProductionFixture {
        pub(crate) fn take_owner(&mut self) -> GeometryProductionOwner {
            self.owner
                .take()
                .expect("production lifecycle owner remains available")
        }

        pub(crate) fn handle(&self) -> lcm_runtime::GeometryRuntimeHandle {
            self.handle.clone()
        }

        pub(crate) fn topology_control(&self) -> Arc<lcm_runtime::GeometryTopologyTestControl> {
            self._geometry_topology.control.clone()
        }

        pub(crate) fn product_paths(&self) -> (PathBuf, PathBuf) {
            (self.cloud_path.clone(), self.occupancy_path.clone())
        }

        pub(crate) fn shutdown_coordinator(&self) -> server::ShutdownCoordinator {
            self.shutdown.clone()
        }

        pub(crate) async fn assert_geometry_terminal_reaches_main_arbitration(
            &mut self,
            expected: &str,
        ) {
            let video_shm_path = std::env::temp_dir().join(format!(
                "nuwax-terminal-arbitration-video-{}-{}.shm",
                std::process::id(),
                uuid::Uuid::new_v4()
            ));
            let video_uds_path = video_shm_path.with_extension("sock");
            let mut video_owner = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path)
                .expect("terminal arbitration video owner");
            let mut server_task = tokio::spawn(async {
                std::future::pending::<Result<(), tonic::transport::Error>>().await
            });
            let owner = self
                .owner
                .as_mut()
                .expect("production lifecycle owner remains available");
            let selected = tokio::time::timeout(
                Duration::from_secs(2),
                next_runtime_terminal(&mut server_task, &video_owner, owner),
            )
            .await
            .expect("real Geometry terminal reaches main select");
            match &selected {
                RuntimeTerminal::GeometryTasks(Err(error)) => {
                    assert!(error.contains(expected), "selected terminal={error:?}");
                }
                _ => panic!("main selected a non-Geometry terminal"),
            }

            let sticky = completed_runtime_terminal(&mut server_task, &video_owner, owner)
                .await
                .expect("completed probe observes sticky Geometry failure");
            match &sticky {
                RuntimeTerminal::GeometryTasks(Err(error)) => {
                    assert!(error.contains(expected), "sticky terminal={error:?}");
                }
                _ => panic!("completed probe lost the Geometry terminal"),
            }
            assert_eq!(
                runtime_terminal_exit_code(sticky, &mut server_task, &self.shutdown).await,
                ExitCode::from(7)
            );
            assert!(server_task.is_finished());
            assert_eq!(
                shutdown_video_owner(&mut video_owner, ExitCode::from(7)),
                ExitCode::from(7)
            );
            assert!(!video_shm_path.exists());
            assert!(!video_uds_path.exists());
        }

        pub(crate) fn shutdown_stages(&self) -> Vec<lio_owner::GeometryShutdownStage> {
            self.shutdown_events.stages()
        }

        pub(crate) fn mapping_active(&self) -> bool {
            self.mapping.lock().snapshot().active
        }

        pub(crate) fn correction_stop_called(&self) -> bool {
            self.correction_sink.stop_called.load(Ordering::Acquire)
        }

        pub(crate) fn assert_downstream_held_before_task_settlement(&self) {
            assert!(
                self.mapping_active(),
                "Mapping retired before Geometry tasks settled"
            );
            assert!(
                !self.correction_stop_called(),
                "Correction shutdown began before Geometry tasks settled"
            );
            assert!(
                !self.lio_shutdown.is_shutdown_requested(),
                "LIO shutdown began before Geometry tasks settled"
            );
            assert!(
                self.shutdown_stages().is_empty(),
                "shutdown completion event published before Geometry tasks settled"
            );
            assert!(
                self.cloud_path.exists(),
                "cloud-map mmap cleaned before settlement"
            );
            assert!(
                self.occupancy_path.exists(),
                "occupancy mmap cleaned before settlement"
            );
        }

        pub(crate) fn assert_shutdown_complete(&self) {
            assert!(!self.mapping_active());
            assert!(self.correction_stop_called());
            assert!(self.lio_shutdown.is_shutdown_requested());
            assert!(!self.cloud_path.exists());
            assert!(!self.occupancy_path.exists());
            assert_eq!(
                self.shutdown_stages(),
                vec![
                    lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                    lio_owner::GeometryShutdownStage::CorrectionSettled,
                    lio_owner::GeometryShutdownStage::MappingRetired,
                    lio_owner::GeometryShutdownStage::LioJoined,
                    lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
                ]
            );
        }
    }

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
    async fn unexpected_grpc_terminal_without_stop_is_nonzero() {
        let (shutdown, _listener) = server::shutdown_channel();
        let mut server_task =
            tokio::spawn(std::future::pending::<Result<(), tonic::transport::Error>>());

        let exit = runtime_terminal_exit_code(
            RuntimeTerminal::Grpc(Ok(Ok(()))),
            &mut server_task,
            &shutdown,
        )
        .await;

        assert_eq!(exit, ExitCode::from(6));
        assert!(!shutdown.is_shutdown_requested());
        server_task.abort();
        let _ = server_task.await;
    }

    #[test]
    fn sticky_runtime_failure_wins_over_clean_grpc_completion() {
        let geometry = prefer_sticky_runtime_terminal(
            RuntimeTerminal::Grpc(Ok(Ok(()))),
            false,
            None,
            Some("Geometry task failed".to_owned()),
        );
        assert!(matches!(geometry, RuntimeTerminal::GeometryTasks(Err(_))));

        let replay = prefer_sticky_runtime_terminal(
            RuntimeTerminal::Grpc(Ok(Ok(()))),
            false,
            Some("deferred replay poisoned".to_owned()),
            Some("Geometry task failed".to_owned()),
        );
        assert!(matches!(replay, RuntimeTerminal::ReplayPoison(_)));

        let process_stop = prefer_sticky_runtime_terminal(
            RuntimeTerminal::Grpc(Ok(Ok(()))),
            true,
            Some("deferred replay poisoned".to_owned()),
            Some("Geometry task failed".to_owned()),
        );
        assert!(matches!(process_stop, RuntimeTerminal::ProcessStopFailed));
    }

    fn terminal_test_video_owner(tag: &str) -> (video_relay::VideoRelayOwner, PathBuf, PathBuf) {
        let video_shm_path = std::env::temp_dir().join(format!(
            "nuwax-terminal-priority-{tag}-{}-{}.shm",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let video_uds_path = video_shm_path.with_extension("sock");
        let video_owner = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path)
            .expect("terminal-priority video owner");
        (video_owner, video_shm_path, video_uds_path)
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn production_terminal_priority_covers_startup_ready_runtime_and_drain_boundaries() {
        // A genuinely clean production fixture establishes the lowest-priority candidate.
        let mut clean = lifecycle_production_fixture(90, "terminal-clean", false).await;
        let clean_product_paths = clean.product_paths();
        assert!(clean_product_paths.0.exists());
        assert!(clean_product_paths.1.exists());
        let (mut clean_video, clean_video_shm, clean_video_uds) =
            terminal_test_video_owner("clean");
        let mut clean_server = tokio::spawn(async { Ok::<(), tonic::transport::Error>(()) });
        tokio::time::timeout(Duration::from_secs(1), async {
            while !clean_server.is_finished() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("clean tonic terminal deadline");
        let clean_terminal = completed_runtime_terminal(
            &mut clean_server,
            &clean_video,
            clean
                .owner
                .as_mut()
                .expect("clean production owner remains available"),
        )
        .await
        .expect("clean production terminal candidate");
        assert!(matches!(clean_terminal, RuntimeTerminal::Grpc(Ok(Ok(())))));
        assert!(!clean.shutdown_coordinator().has_shutdown_failed());
        assert_eq!(
            shutdown_video_owner(&mut clean_video, ExitCode::from(0)),
            ExitCode::from(0)
        );
        assert!(!clean_video_shm.exists());
        assert!(!clean_video_uds.exists());
        clean
            .take_owner()
            .shutdown()
            .await
            .expect("clean production owner shutdown");
        clean.assert_shutdown_complete();
        drop(clean);

        // The same production owner carries an actual owned-consumer failure through
        // startup selection, ready probes, and simultaneous runtime candidates.
        let topology_setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|control| {
            control.inject_terminal(
                "public-tf-observer",
                lcm_runtime::GeometryTopologyInjectedTerminal::Eof,
            );
        })
        .await;
        let mut priority =
            lifecycle_production_fixture_with_setup(91, "terminal-priority", false, topology_setup)
                .await;
        let topology_control = priority.topology_control();
        topology_control.release_terminal();
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let completed = priority
                    .owner
                    .as_ref()
                    .expect("priority production owner remains available")
                    .completed_failure_terminal();
                if matches!(completed, Some(RuntimeTerminal::GeometryTasks(Err(_)))) {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("owned GeometryTasks failure deadline");

        let (mut priority_video, priority_video_shm, priority_video_uds) =
            terminal_test_video_owner("hierarchy");
        let mut simultaneous_clean_server =
            tokio::spawn(async { Ok::<(), tonic::transport::Error>(()) });
        tokio::time::timeout(Duration::from_secs(1), async {
            while !simultaneous_clean_server.is_finished() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("simultaneous clean tonic terminal deadline");

        let startup_terminal = next_runtime_terminal(
            &mut simultaneous_clean_server,
            &priority_video,
            priority
                .owner
                .as_mut()
                .expect("startup priority owner remains available"),
        )
        .await;
        assert!(matches!(
            startup_terminal,
            RuntimeTerminal::GeometryTasks(Err(_))
        ));

        priority
            .owner
            .as_mut()
            .expect("ready priority owner remains available")
            .inject_replay_poison_for_terminal_tests("deferred replay poisoned");
        let ready_terminal = completed_runtime_terminal(
            &mut simultaneous_clean_server,
            &priority_video,
            priority
                .owner
                .as_mut()
                .expect("ready priority owner remains available"),
        )
        .await
        .expect("ready-before-publication sticky probe");
        assert!(matches!(ready_terminal, RuntimeTerminal::ReplayPoison(_)));

        let priority_shutdown = priority.shutdown_coordinator();
        drop(
            priority_shutdown
                .begin_process_stop_phase_a()
                .expect("production fixture exposes Correction Process Stop attempt"),
        );
        let runtime_terminal = next_runtime_terminal(
            &mut simultaneous_clean_server,
            &priority_video,
            priority
                .owner
                .as_mut()
                .expect("runtime priority owner remains available"),
        )
        .await;
        assert!(matches!(
            runtime_terminal,
            RuntimeTerminal::ProcessStopFailed
        ));
        let _ = (&mut simultaneous_clean_server).await;
        assert_eq!(
            shutdown_video_owner(&mut priority_video, ExitCode::from(7)),
            ExitCode::from(7)
        );
        assert!(!priority_video_shm.exists());
        assert!(!priority_video_uds.exists());
        let priority_failure = priority
            .take_owner()
            .shutdown()
            .await
            .expect_err("owned GeometryTasks failure remains sticky through shutdown");
        assert!(priority_failure.into_pending_owner().is_none());
        priority.assert_shutdown_complete();
        drop(priority);

        // A Process Stop failure that arrives during tonic response drain must
        // still be observed after the clean response becomes available.
        let mut drain = lifecycle_production_fixture(92, "terminal-drain", false).await;
        let drain_shutdown = drain.shutdown_coordinator();
        let drain_entered = Arc::new(tokio::sync::Notify::new());
        let task_entered = Arc::clone(&drain_entered);
        let task_shutdown = drain_shutdown.clone();
        let drain_server = tokio::spawn(async move {
            task_entered.notify_one();
            task_shutdown.wait_for_readiness_revocation().await;
            Ok::<(), tonic::transport::Error>(())
        });
        tokio::time::timeout(Duration::from_secs(1), drain_entered.notified())
            .await
            .expect("tonic response drain entered");
        let exit_shutdown = drain_shutdown.clone();
        let drain_exit = tokio::spawn(async move {
            let mut drain_server = drain_server;
            runtime_terminal_exit_code(
                RuntimeTerminal::Correction(Ok(())),
                &mut drain_server,
                &exit_shutdown,
            )
            .await
        });
        tokio::task::yield_now().await;
        assert!(!drain_exit.is_finished());
        drop(
            drain_shutdown
                .begin_process_stop_phase_a()
                .expect("drain fixture exposes Correction Process Stop attempt"),
        );
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(2), drain_exit)
                .await
                .expect("response drain sticky recheck deadline")
                .expect("response drain arbitration task"),
            ExitCode::from(7)
        );
        assert!(drain_shutdown.has_shutdown_failed());
        drain
            .take_owner()
            .shutdown()
            .await
            .expect("drain production owner shutdown");
        drain.assert_shutdown_complete();
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
    async fn none_process_stop_waits_for_tonic_response_drain_before_exit_zero() {
        let correction_sink = Arc::new(LifecycleCorrectionSink::new(false));
        let mut correction = correction_coordinator::CorrectionTransportOwner::start(
            correction_coordinator::CorrectionOwnerSpec::None,
            lifecycle_correction_identity(53),
            "unused-by-none",
            correction_sink,
        )
        .await
        .expect("none correction owner");
        let correction_phase_a = correction.phase_a_control();
        let lio_sink = Arc::new(LifecycleLioSink::new(correction.ingress(), false));
        let lio_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(lio_sink));
        let (shutdown, _listener) =
            server::production_shutdown_channel(lio_gate, correction_phase_a, None);
        let stop_attempt = shutdown
            .begin_process_stop_phase_a()
            .expect("production correction Stop attempt");
        shutdown.request_shutdown();
        correction
            .wait_terminal()
            .await
            .expect("none producer exits cleanly for Process Stop");
        stop_attempt.complete();

        let drain_entered = Arc::new(tokio::sync::Notify::new());
        let drain_release = Arc::new(tokio::sync::Notify::new());
        let task_entered = Arc::clone(&drain_entered);
        let task_release = Arc::clone(&drain_release);
        let server_task = tokio::spawn(async move {
            task_entered.notify_one();
            task_release.notified().await;
            Ok::<(), tonic::transport::Error>(())
        });
        tokio::time::timeout(Duration::from_secs(1), drain_entered.notified())
            .await
            .expect("synthetic tonic task entered response drain");
        let exit_shutdown = shutdown.clone();
        let exit_task = tokio::spawn(async move {
            let mut server_task = server_task;
            runtime_terminal_exit_code(
                RuntimeTerminal::Correction(Ok(())),
                &mut server_task,
                &exit_shutdown,
            )
            .await
        });
        tokio::task::yield_now().await;
        assert!(
            !exit_task.is_finished(),
            "clean correction terminal cannot bypass tonic response drain"
        );
        drain_release.notify_one();
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(1), exit_task)
                .await
                .expect("response-drained exit deadline")
                .expect("response-drained exit task"),
            ExitCode::from(0)
        );
        correction
            .shutdown()
            .await
            .expect("none owner shutdown remains clean");
    }

    #[tokio::test]
    async fn unexpected_correction_terminal_forces_exit_seven() {
        let (shutdown, mut listener) = server::shutdown_channel();
        let forced = Arc::new(AtomicBool::new(false));
        let task_forced = Arc::clone(&forced);
        let mut server_task = tokio::spawn(async move {
            listener.wait_for_request().await;
            task_forced.store(true, Ordering::Release);
            Ok::<(), tonic::transport::Error>(())
        });

        let exit = runtime_terminal_exit_code(
            RuntimeTerminal::Correction(Err("correction producer exited unexpectedly".to_owned())),
            &mut server_task,
            &shutdown,
        )
        .await;
        assert_eq!(exit, ExitCode::from(7));
        assert!(forced.load(Ordering::Acquire));
        assert!(server_task.is_finished());
    }

    #[tokio::test]
    async fn failed_process_stop_and_shutdown_ack_error_remain_exit_seven() {
        let (failed_shutdown, mut failed_listener) = server::shutdown_channel();
        let mut failed_server_task = tokio::spawn(async move {
            failed_listener.wait_for_request().await;
            Ok::<(), tonic::transport::Error>(())
        });
        assert_eq!(
            runtime_terminal_exit_code(
                RuntimeTerminal::ProcessStopFailed,
                &mut failed_server_task,
                &failed_shutdown,
            )
            .await,
            ExitCode::from(7)
        );

        let (ack_shutdown, ack_listener) = server::shutdown_channel();
        drop(ack_listener);
        ack_shutdown
            .request_and_wait_for_listener()
            .await
            .expect_err("closed listener ACK channel marks shutdown failed");
        let mut completed_server_task =
            tokio::spawn(async { Ok::<(), tonic::transport::Error>(()) });
        assert_eq!(
            runtime_terminal_exit_code(
                RuntimeTerminal::Correction(Ok(())),
                &mut completed_server_task,
                &ack_shutdown,
            )
            .await,
            ExitCode::from(7),
            "shutdown acknowledgement error must survive a clean correction terminal"
        );
    }

    #[tokio::test]
    async fn sticky_fatal_survives_terminal_drain_and_ready_cleanup_failure() {
        let ready_path = std::env::temp_dir().join(format!(
            "nuwax-geometry-ready-sticky-fatal-{}-{}",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let mut ready_lease =
            ReadyFileLease::acquire(ready_path.to_string_lossy().into_owned(), 50_051)
                .expect("sticky-fatal test ready lease");
        std::fs::remove_file(&ready_path).expect("replace ready file with removal-failure target");
        std::fs::create_dir(&ready_path).expect("ready removal-failure directory");

        let phase_a = correction_coordinator::CorrectionPhaseAControl::for_tests();
        let probe = Arc::new(PreCorrectionLioSink::new(false));
        let lio_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(probe));
        let (shutdown, listener) =
            server::production_shutdown_channel(lio_gate, phase_a.clone(), None);
        drop(listener);
        shutdown
            .request_and_wait_for_listener()
            .await
            .expect_err("listener ACK loss must set production coordinator fatal state");
        assert!(!phase_a.process_stop_failed());
        assert!(shutdown.has_shutdown_failed());
        assert!(shutdown.is_readiness_revoked());

        let selected = prefer_sticky_runtime_terminal(
            RuntimeTerminal::Correction(Ok(())),
            phase_a.process_stop_failed(),
            None,
            None,
        );
        assert!(matches!(selected, RuntimeTerminal::Correction(Ok(()))));

        let drain_entered = Arc::new(tokio::sync::Notify::new());
        let drain_release = Arc::new(tokio::sync::Notify::new());
        let task_entered = Arc::clone(&drain_entered);
        let task_release = Arc::clone(&drain_release);
        let server_task = tokio::spawn(async move {
            task_entered.notify_one();
            task_release.notified().await;
            Ok::<(), tonic::transport::Error>(())
        });
        tokio::time::timeout(Duration::from_secs(1), drain_entered.notified())
            .await
            .expect("clean tonic task entered terminal drain");
        let exit_shutdown = shutdown.clone();
        let exit_task = tokio::spawn(async move {
            let mut server_task = server_task;
            runtime_terminal_exit_code(selected, &mut server_task, &exit_shutdown).await
        });
        tokio::task::yield_now().await;
        assert!(
            !exit_task.is_finished(),
            "sticky fatal must still wait for the clean tonic response drain"
        );
        drain_release.notify_one();
        let terminal_exit = tokio::time::timeout(Duration::from_secs(1), exit_task)
            .await
            .expect("sticky terminal drain deadline")
            .expect("sticky terminal drain task");
        assert_eq!(terminal_exit, ExitCode::from(7));

        let ready_remove_failed = ready_lease.revoke().is_err();
        assert!(ready_remove_failed);
        assert_eq!(
            exit_code_after_ready_cleanup(terminal_exit, ready_remove_failed),
            ExitCode::from(7)
        );
        drop(ready_lease);
        std::fs::remove_dir(&ready_path).expect("sticky-fatal ready test cleanup");
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

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn production_owner_shuts_down_correction_before_mapping_lio_and_cleanup() {
        let topology_setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|control| {
            control.inject_terminal(
                "public-tf-observer",
                lcm_runtime::GeometryTopologyInjectedTerminal::Eof,
            );
        })
        .await;
        let fixture =
            lifecycle_production_fixture_with_setup(51, "reverse", true, topology_setup).await;
        fixture.topology_control().release_terminal();
        let geometry_task_failure = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if let Some(RuntimeTerminal::GeometryTasks(Err(error))) = fixture
                    .owner
                    .as_ref()
                    .and_then(GeometryProductionOwner::completed_failure_terminal)
                {
                    break error;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("sticky Geometry task failure reaches the root before shutdown");

        let LifecycleProductionFixture {
            _geometry_topology,
            owner,
            mapping,
            correction_sink,
            correction_ingress,
            correction_binding,
            lio_sink,
            lio_shutdown,
            ingress_gate,
            shutdown_events,
            provider,
            cloud_path,
            occupancy_path,
            ..
        } = fixture;
        assert!(correction_ingress.is_open());
        assert!(ingress_gate.is_open() || !lio_sink.closed.load(Ordering::Acquire));
        assert!(cloud_path.exists());
        assert!(occupancy_path.exists());

        let stop_started = correction_sink.stop_started.notified();
        let owner = owner.expect("production lifecycle owner");
        let shutdown_task = tokio::spawn(async move { owner.shutdown().await });
        tokio::time::timeout(Duration::from_secs(2), stop_started)
            .await
            .expect("correction stop entry");

        assert!(!correction_ingress.is_open());
        assert!(mapping.lock().snapshot().active);
        assert!(!lio_shutdown.is_shutdown_requested());
        assert!(lio_sink.closed.load(Ordering::Acquire));
        assert_eq!(
            shutdown_events.stages(),
            vec![lio_owner::GeometryShutdownStage::GeometryTasksSettled]
        );
        assert!(!shutdown_task.is_finished());

        let late_publisher =
            nuwax_transport::LcmTransport::new_real(&provider).expect("independent late publisher");
        late_publisher
            .publish(
                nuwax_protocol::RawLcmFrameBuilder::new(correction_binding.0, correction_binding.1)
                    .seqno(99)
                    .payload(vec![0x5a; 16])
                    .build(),
            )
            .await
            .expect("late output may publish after acknowledged unsubscribe");
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(correction_sink.apply_count.load(Ordering::Acquire), 0);

        let shutdown_failure = tokio::time::timeout(Duration::from_secs(5), shutdown_task)
            .await
            .expect("bounded production shutdown attempt")
            .expect("production shutdown task")
            .expect_err("blocked Correction returns the retained root owner");
        assert!(shutdown_failure
            .to_string()
            .contains(geometry_task_failure.as_str()));
        assert!(mapping.lock().snapshot().active);
        assert!(!lio_shutdown.is_shutdown_requested());
        assert!(cloud_path.exists());
        assert!(occupancy_path.exists());
        assert_eq!(
            shutdown_events.stages(),
            vec![lio_owner::GeometryShutdownStage::GeometryTasksSettled]
        );
        assert_eq!(correction_sink.stop_calls.load(Ordering::Acquire), 1);

        correction_sink.stop_release.notify_one();
        let pending_owner = *shutdown_failure
            .into_pending_owner()
            .expect("Correction timeout retains the complete production owner");
        let terminal_failure = pending_owner.shutdown().await.expect_err(
            "retry joins the same Correction owner but preserves sticky Geometry failure",
        );
        assert!(terminal_failure
            .to_string()
            .contains(geometry_task_failure.as_str()));
        assert!(
            terminal_failure.into_pending_owner().is_none(),
            "retry completed every owner despite the sticky terminal failure"
        );
        assert!(!mapping.lock().snapshot().active);
        assert!(lio_shutdown.is_shutdown_requested());
        assert!(!cloud_path.exists());
        assert!(!occupancy_path.exists());
        assert_eq!(
            correction_sink.stop_calls.load(Ordering::Acquire),
            1,
            "retry must join the retained stop task instead of starting a new owner"
        );
        assert_eq!(
            shutdown_events.stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::CorrectionSettled,
                lio_owner::GeometryShutdownStage::MappingRetired,
                lio_owner::GeometryShutdownStage::LioJoined,
                lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
            ]
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn settled_sticky_correction_error_tears_down_downstream_once_and_returns_error() {
        let mut fixture = lifecycle_production_fixture(56, "settled-correction-error", false).await;
        fixture
            .owner
            .as_mut()
            .expect("production lifecycle owner")
            .correction
            .as_mut()
            .expect("running owner retains Correction")
            .inject_terminal_failure_for_shutdown_tests("injected settled correction failure");

        let failure = fixture
            .take_owner()
            .shutdown()
            .await
            .expect_err("settled Correction failure remains a root terminal error");
        let detail = failure.to_string();
        assert!(detail.contains("injected settled correction failure"));
        assert!(
            failure.into_pending_owner().is_none(),
            "settled Correction failure cannot retain a pending root owner"
        );
        fixture.assert_shutdown_complete();
        assert_eq!(
            fixture
                .shutdown_stages()
                .iter()
                .filter(|stage| **stage == lio_owner::GeometryShutdownStage::CorrectionSettled)
                .count(),
            1
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn pending_mapping_join_preserves_prior_sticky_failures_and_event_state() {
        let topology_setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|control| {
            control.inject_terminal(
                "public-tf-observer",
                lcm_runtime::GeometryTopologyInjectedTerminal::Eof,
            );
        })
        .await;
        let mut fixture = lifecycle_production_fixture_with_setup(
            57,
            "mapping-pending-sticky",
            false,
            topology_setup,
        )
        .await;
        fixture
            .owner
            .as_mut()
            .expect("production lifecycle owner")
            .correction
            .as_mut()
            .expect("running owner retains Correction")
            .inject_terminal_failure_for_shutdown_tests("injected prior Correction failure");
        let retirement_gate = mapping_backend::MappingBackendRetirementTestGate::blocked();
        fixture
            .mapping
            .lock()
            .replace_worker_with_retirement_test_gate(retirement_gate.clone())
            .expect("install blocked Mapping retirement worker");
        fixture.topology_control().release_terminal();
        let geometry_task_failure = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if let Some(RuntimeTerminal::GeometryTasks(Err(error))) = fixture
                    .owner
                    .as_ref()
                    .and_then(GeometryProductionOwner::completed_failure_terminal)
                {
                    break error;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("sticky Geometry task failure reaches the root");

        let first_failure = fixture
            .take_owner()
            .shutdown()
            .await
            .expect_err("blocked Mapping join returns the complete pending root");
        let first_detail = first_failure.to_string();
        assert!(first_detail.contains(geometry_task_failure.as_str()));
        assert!(first_detail.contains("injected prior Correction failure"));
        assert!(first_detail.contains("Mapping retirement pending"));
        assert!(retirement_gate.entered());
        assert_eq!(
            fixture.mapping.lock().backend_owner_state_for_tests(),
            (false, true)
        );
        assert!(!fixture.lio_shutdown.is_shutdown_requested());
        assert!(fixture.cloud_path.exists());
        assert!(fixture.occupancy_path.exists());
        assert_eq!(
            fixture.shutdown_stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::CorrectionSettled,
            ]
        );

        retirement_gate.release();
        let final_failure = first_failure
            .into_pending_owner()
            .expect("Mapping timeout retains the complete production owner")
            .shutdown()
            .await
            .expect_err("prior settled failures remain the final exit status");
        let final_detail = final_failure.to_string();
        assert!(final_detail.contains(geometry_task_failure.as_str()));
        assert!(final_detail.contains("injected prior Correction failure"));
        assert!(final_failure.into_pending_owner().is_none());
        fixture.assert_shutdown_complete();
        assert_eq!(
            fixture
                .shutdown_stages()
                .iter()
                .filter(|stage| **stage == lio_owner::GeometryShutdownStage::CorrectionSettled)
                .count(),
            1
        );
        assert_eq!(
            fixture
                .shutdown_stages()
                .iter()
                .filter(|stage| **stage == lio_owner::GeometryShutdownStage::MappingRetired)
                .count(),
            1
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn pending_mapping_lifecycle_lock_withholds_lio_and_shm_until_retry() {
        let mut fixture = lifecycle_production_fixture(59, "mapping-lock-pending", false).await;
        let handle = fixture.handle();
        let lifecycle_guard = Arc::clone(&handle.mapping_lifecycle_lock)
            .lock_owned()
            .await;

        let first_failure = fixture
            .take_owner()
            .shutdown()
            .await
            .expect_err("held Mapping lifecycle lock returns the pending root");
        assert!(first_failure
            .to_string()
            .contains("timed out acquiring the Mapping lifecycle lock"));
        assert!(fixture.mapping_active());
        assert!(!fixture.lio_shutdown.is_shutdown_requested());
        assert!(fixture.cloud_path.exists());
        assert!(fixture.occupancy_path.exists());
        assert_eq!(
            fixture.shutdown_stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::CorrectionSettled,
            ]
        );

        drop(lifecycle_guard);
        first_failure
            .into_pending_owner()
            .expect("lifecycle-lock timeout retains the complete production owner")
            .shutdown()
            .await
            .expect("retry settles Mapping, LIO, and product resources");
        fixture.assert_shutdown_complete();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn shutdown_runtime_owner_exhausts_budget_and_retains_complete_root_until_mapping_settles(
    ) {
        let _retention_guard = PROCESS_EXIT_RETENTION_TEST_LOCK.lock().await;
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "production retention test must start without retained-owner residue"
        );
        let mut fixture =
            lifecycle_production_fixture(60, "shutdown-budget-retention", false).await;
        let retirement_gate = mapping_backend::MappingBackendRetirementTestGate::blocked();
        fixture
            .mapping
            .lock()
            .replace_worker_with_retirement_test_gate(retirement_gate.clone())
            .expect("install blocked Mapping retirement worker");

        let exit = shutdown_runtime_owner(fixture.take_owner(), ExitCode::SUCCESS).await;
        assert_eq!(exit, ExitCode::from(7));
        assert!(retirement_gate.entered());
        assert_eq!(
            fixture.mapping.lock().backend_owner_state_for_tests(),
            (false, true),
            "budget exhaustion must retain the unjoined Mapping worker"
        );
        assert!(
            !fixture.lio_shutdown.is_shutdown_requested(),
            "pending Mapping retirement must withhold LIO teardown"
        );
        assert!(fixture.cloud_path.exists());
        assert!(fixture.occupancy_path.exists());
        assert_eq!(
            fixture.shutdown_stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::CorrectionSettled,
            ]
        );
        assert_eq!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .len(),
            1,
            "the complete pending root must remain owned by process-exit retention"
        );

        let retained_exit_runtime =
            tokio::runtime::Runtime::new().expect("retained-exit finalizer test runtime");
        let (blocking_entered_tx, blocking_entered_rx) = tokio::sync::oneshot::channel();
        let (blocking_release_tx, blocking_release_rx) = std::sync::mpsc::channel();
        let blocking_completed = Arc::new(AtomicBool::new(false));
        let task_completed = blocking_completed.clone();
        let blocking_task = retained_exit_runtime.spawn_blocking(move || {
            let _ = blocking_entered_tx.send(());
            blocking_release_rx
                .recv()
                .expect("release retained-exit blocking task");
            task_completed.store(true, Ordering::Release);
        });
        tokio::time::timeout(Duration::from_secs(1), blocking_entered_rx)
            .await
            .expect("retained-exit blocking task entry deadline")
            .expect("retained-exit blocking task entry signal");

        let (finalizer_tx, finalizer_rx) = tokio::sync::oneshot::channel();
        let finalizer_thread = std::thread::spawn(move || {
            let exit = finalize_geometry_runtime(retained_exit_runtime, ExitCode::from(7));
            let _ = finalizer_tx.send(exit);
        });
        let finalizer_result = tokio::time::timeout(Duration::from_secs(1), finalizer_rx).await;
        assert!(
            !blocking_completed.load(Ordering::Acquire),
            "blocking task must remain unreleased when retained-exit finalizer returns"
        );
        blocking_release_tx
            .send(())
            .expect("release retained-exit blocking task after finalizer probe");
        tokio::time::timeout(Duration::from_secs(1), blocking_task)
            .await
            .expect("retained-exit blocking task completion deadline")
            .expect("retained-exit blocking task join");
        assert!(blocking_completed.load(Ordering::Acquire));
        finalizer_thread
            .join()
            .expect("retained-exit finalizer thread");
        assert_eq!(
            finalizer_result
                .expect("retained-exit finalizer must return before task release")
                .expect("retained-exit finalizer result signal"),
            ExitCode::from(7)
        );
        assert!(GEOMETRY_RETAINED_EXIT_SELECTED.load(Ordering::Acquire));

        let pending_owner = match take_geometry_process_exit_retention_for_tests() {
            GeometryProcessExitRetention::Production(owner) => owner,
            GeometryProcessExitRetention::MappingAndLio { .. } => {
                panic!("top-level production shutdown retained only a Mapping/LIO owner")
            }
            GeometryProcessExitRetention::StartupRollbackAndLio { .. } => {
                panic!("top-level production shutdown retained a startup rollback owner")
            }
        };
        assert_eq!(
            pending_owner.shutdown_attempts_for_tests, FINAL_OWNER_ATTEMPT_LIMIT,
            "top-level shutdown must exhaust the complete final-owner attempt budget"
        );
        retirement_gate.release();
        pending_owner
            .shutdown()
            .await
            .expect("released retained root settles Mapping, LIO, and product resources");

        assert_eq!(
            fixture.mapping.lock().backend_owner_state_for_tests(),
            (false, false),
            "released Mapping worker must be joined and precisely retired"
        );
        fixture.assert_shutdown_complete();
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "test retrieval must leave no retained-owner residue"
        );
        reset_geometry_retained_exit_after_test_settlement();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn rollback_mapping_and_lio_exhausts_budget_and_retains_bare_owner_until_mapping_settles()
    {
        let _retention_guard = PROCESS_EXIT_RETENTION_TEST_LOCK.lock().await;
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "bare rollback retention test must start without retained-owner residue"
        );

        let setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|_| {}).await;
        let product_paths = setup.product_paths();
        let lio_probe = Arc::new(PreCorrectionLioSink::new(false));
        let ingress_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(lio_probe.clone()));
        let lio = lio_owner::LioOwnerHandle::start(
            setup.transport(),
            lifecycle_lio_spec("mapping-lio-budget-retention"),
            62,
            ingress_gate.clone(),
        )
        .await
        .expect("LIO READY before bare Mapping/LIO rollback fixture");
        assert!(lio_probe.ready_seen.load(Ordering::Acquire));
        let lio_shutdown = lio.shutdown_control_for_tests();
        let mut topology = match setup.spawn().await {
            Ok(topology) => topology,
            Err(mut failure) => {
                let detail = failure.take_failure().to_string();
                lio.shutdown()
                    .await
                    .expect("unexpected Geometry startup failure still settles LIO");
                panic!("bare Mapping/LIO topology startup failed: {detail}")
            }
        };
        assert!(product_paths.0.exists());
        assert!(product_paths.1.exists());
        let spawned = topology.take_spawned();
        let mapping = spawned
            .handle
            .mapping_adapter
            .as_ref()
            .expect("production Mapping adapter")
            .clone();
        mapping
            .lock()
            .start_session("mapping-lio-budget-session", "mapping-lio-budget-map")
            .expect("start Mapping before bare rollback");
        let retirement_gate = mapping_backend::MappingBackendRetirementTestGate::blocked();
        mapping
            .lock()
            .replace_worker_with_retirement_test_gate(retirement_gate.clone())
            .expect("install blocked Mapping retirement worker");

        let exit = rollback_mapping_and_lio(
            lio_owner::GeometryRuntimeOwner::new(spawned, lio),
            ingress_gate.clone(),
            ExitCode::from(4),
        )
        .await;
        assert_eq!(exit, ExitCode::from(7));
        assert!(retirement_gate.entered());
        assert_eq!(
            mapping.lock().backend_owner_state_for_tests(),
            (false, true)
        );
        assert!(!lio_shutdown.is_shutdown_requested());
        assert!(ingress_gate.is_closed());
        assert!(lio_probe.closed.load(Ordering::Acquire));
        assert!(product_paths.0.exists());
        assert!(product_paths.1.exists());
        assert_eq!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .len(),
            1
        );

        let (pending_owner, retained_ingress_gate) =
            match take_geometry_process_exit_retention_for_tests() {
                GeometryProcessExitRetention::MappingAndLio {
                    owner,
                    ingress_gate,
                } => (owner, ingress_gate),
                GeometryProcessExitRetention::Production(_) => {
                    panic!("bare Mapping/LIO rollback retained a production owner")
                }
                GeometryProcessExitRetention::StartupRollbackAndLio { .. } => {
                    panic!("bare Mapping/LIO rollback retained a startup owner pair")
                }
            };
        assert!(Arc::ptr_eq(&retained_ingress_gate, &ingress_gate));
        assert!(retained_ingress_gate.is_closed());
        assert!(geometry_process_exit_retention_for_tests()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_empty());

        retirement_gate.release();
        pending_owner
            .shutdown()
            .await
            .expect("released bare owner settles Mapping, LIO, and product resources");
        assert_eq!(
            mapping.lock().backend_owner_state_for_tests(),
            (false, false)
        );
        assert!(lio_shutdown.is_shutdown_requested());
        assert!(!product_paths.0.exists());
        assert!(!product_paths.1.exists());
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "bare rollback test must leave no retained-owner residue"
        );
        reset_geometry_retained_exit_after_test_settlement();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn rollback_geometry_startup_and_lio_exhausts_budget_and_retains_owner_pair_until_scheduler_settles(
    ) {
        let _retention_guard = PROCESS_EXIT_RETENTION_TEST_LOCK.lock().await;
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "startup rollback retention test must start without retained-owner residue"
        );

        let setup = lcm_runtime::ProductionTopologyTestSetup::prepare(|control| {
            control.reject_registration("map-revision-coalescer");
            control.enable_scheduler_block();
        })
        .await;
        let product_paths = setup.product_paths();
        let control = setup.control.clone();
        let (lio, lio_shutdown) =
            start_lio_before_geometry_for_tests(setup.transport(), 63, "startup-budget-retention")
                .await;
        let mut failed = match setup.spawn().await {
            Err(failed) => failed,
            Ok(mut topology) => {
                let spawned = topology.take_spawned();
                let _ = lio_owner::GeometryRuntimeOwner::new(spawned, lio)
                    .shutdown()
                    .await;
                panic!("map-revision-coalescer registration unexpectedly succeeded")
            }
        };
        assert_eq!(failed.product_paths(), product_paths);
        let failure = failed.take_failure();
        assert!(failure.has_rollback_owner_for_tests());
        tokio::time::timeout(Duration::from_secs(2), async {
            while !control.scheduler_block_entered() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("pre-spawned scheduler entered its rollback block");
        assert!(control.startup_shutdown_stages().is_empty());
        assert!(!lio_shutdown.is_shutdown_requested());
        assert!(product_paths.0.exists());
        assert!(product_paths.1.exists());

        let exit = rollback_geometry_startup_and_lio(failure, lio).await;
        assert_eq!(exit, ExitCode::from(7));
        assert!(control.startup_shutdown_stages().is_empty());
        assert!(!lio_shutdown.is_shutdown_requested());
        assert!(product_paths.0.exists());
        assert!(product_paths.1.exists());
        assert_eq!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .len(),
            1
        );

        let (geometry, lio) = match take_geometry_process_exit_retention_for_tests() {
            GeometryProcessExitRetention::StartupRollbackAndLio { geometry, lio } => {
                (geometry, lio)
            }
            GeometryProcessExitRetention::Production(_) => {
                panic!("startup rollback retained a production owner")
            }
            GeometryProcessExitRetention::MappingAndLio { .. } => {
                panic!("startup rollback retained only a Mapping/LIO owner")
            }
        };
        assert!(geometry_process_exit_retention_for_tests()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_empty());

        control.release_scheduler_block();
        match geometry.shutdown().await {
            Ok(()) => {}
            Err(error) => {
                let detail = error.to_string();
                assert!(
                    error.into_pending_owner().is_none(),
                    "released startup rollback remained pending: {detail}"
                );
            }
        }
        lio.shutdown()
            .await
            .expect("LIO joins after retained startup rollback settles");
        assert!(lio_shutdown.is_shutdown_requested());
        assert!(!product_paths.0.exists());
        assert!(!product_paths.1.exists());
        assert_eq!(
            control.startup_shutdown_stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
            ]
        );
        assert!(
            geometry_process_exit_retention_for_tests()
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .is_empty(),
            "startup rollback test must leave no retained-owner residue"
        );
        reset_geometry_retained_exit_after_test_settlement();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn settled_mapping_join_error_emits_once_and_continues_lio_and_shm_cleanup() {
        let mut fixture = lifecycle_production_fixture(58, "mapping-settled-error", false).await;
        fixture
            .mapping
            .lock()
            .replace_worker_with_retirement_test_gate(
                mapping_backend::MappingBackendRetirementTestGate::failing(),
            )
            .expect("install failing Mapping retirement worker");

        let exit = shutdown_runtime_owner(fixture.take_owner(), ExitCode::SUCCESS).await;
        assert_eq!(exit, ExitCode::from(7));
        assert_eq!(
            fixture.mapping.lock().backend_owner_state_for_tests(),
            (false, false),
            "joined Err still acknowledges the exact backend generation"
        );
        fixture.assert_shutdown_complete();
        assert_eq!(
            fixture
                .shutdown_stages()
                .iter()
                .filter(|stage| **stage == lio_owner::GeometryShutdownStage::MappingRetired)
                .count(),
            1
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn settled_lio_error_is_sticky_and_still_retires_mapping_and_cleans_product_shm() {
        let mut fixture = lifecycle_production_fixture(61, "settled-lio-error", false).await;
        fixture
            .owner
            .as_mut()
            .expect("production lifecycle owner")
            .runtime
            .settle_lio_then_inject_terminal_failure_for_shutdown_tests(
                "injected error after the LIO supervisor was joined",
            )
            .await;
        assert!(fixture.lio_shutdown.is_shutdown_requested());
        assert!(fixture.shutdown_stages().is_empty());

        let exit = shutdown_runtime_owner(fixture.take_owner(), ExitCode::SUCCESS).await;
        assert_eq!(exit, ExitCode::from(7));
        assert_eq!(
            fixture.mapping.lock().backend_owner_state_for_tests(),
            (false, false),
            "settled LIO Err must leave Mapping precisely retired"
        );
        assert!(!fixture.cloud_path.exists());
        assert!(!fixture.occupancy_path.exists());
        let stages = fixture.shutdown_stages();
        assert_eq!(
            stages
                .iter()
                .filter(|stage| **stage == lio_owner::GeometryShutdownStage::LioJoined)
                .count(),
            1
        );
        assert_eq!(
            stages
                .iter()
                .filter(|stage| {
                    **stage == lio_owner::GeometryShutdownStage::ResourceCleanupComplete
                })
                .count(),
            1
        );
        fixture.assert_shutdown_complete();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn lio_ready_failure_does_not_start_mapping_or_correction() {
        let handle = lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-LCM Geometry handle");
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        let occupancy_path = handle.occupancy_shm_path.clone().map(PathBuf::from);
        let lio_sink = Arc::new(PreCorrectionLioSink::new(true));
        let ingress_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(lio_sink.clone()));

        let error = lio_owner::LioOwnerHandle::start(
            handle.transport.clone(),
            lifecycle_lio_spec("lio-fail"),
            52,
            ingress_gate,
        )
        .await
        .expect_err("injected LIO READY failure");

        assert!(error.contains("before Mapping/Correction startup"));
        assert!(lio_sink.ready_seen.load(Ordering::Acquire));
        assert!(lio_sink.closed.load(Ordering::Acquire));
        assert!(!mapping.lock().snapshot().active);
        assert_eq!(mapping.lock().snapshot().session_generation, 0);
        if let Some(writer) = &handle.occupancy_shm {
            writer
                .lock()
                .cleanup_product_owned_file()
                .expect("test occupancy cleanup");
        }
        if let Some(path) = occupancy_path {
            assert!(!path.exists());
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn correction_start_failure_rolls_back_mapping_and_lio() {
        let spawned = lcm_runtime::attach_root_task_owner_for_tests(
            lcm_runtime::build_handle_for_tests()
                .await
                .expect("real-LCM Geometry handle"),
        );
        let handle = spawned.handle.clone();
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        let occupancy_path = PathBuf::from(
            handle
                .occupancy_shm_path
                .clone()
                .expect("test occupancy mmap path"),
        );
        let lio_sink = Arc::new(PreCorrectionLioSink::new(false));
        let ingress_gate = Arc::new(lio_ingress::LioIngressRoutingGate::new(lio_sink.clone()));
        let lio = lio_owner::LioOwnerHandle::start(
            handle.transport.clone(),
            lifecycle_lio_spec("corr-fail"),
            54,
            ingress_gate.clone(),
        )
        .await
        .expect("LIO READY before Mapping");
        mapping
            .lock()
            .start_session("correction-failure-session", "correction-failure-map")
            .expect("Mapping starts before Correction");
        assert!(mapping.lock().snapshot().active);

        let correction_sink = Arc::new(LifecycleCorrectionSink::new(false));
        let correction_error = match correction_coordinator::CorrectionTransportOwner::start_with_relocalization_mode_fence(
            lifecycle_correction_spec(),
            lifecycle_correction_identity(54),
            " invalid-private-correction-provider ",
            correction_sink.clone(),
            Arc::new(correction_coordinator::RelocalizationModeFence::default()),
        )
        .await
        {
            Ok(owner) => {
                owner
                    .shutdown()
                    .await
                    .expect("unexpected Correction owner cleanup");
                panic!("invalid Correction provider unexpectedly started")
            }
            Err(error) => error,
        };
        assert!(correction_error.to_string().contains("invalid"));

        let exit = rollback_mapping_and_lio(
            lio_owner::GeometryRuntimeOwner::new(spawned, lio),
            ingress_gate,
            ExitCode::from(4),
        )
        .await;
        assert_eq!(exit, ExitCode::from(4));
        assert!(!mapping.lock().snapshot().active);
        assert!(lio_sink.closed.load(Ordering::Acquire));
        assert_eq!(correction_sink.apply_count.load(Ordering::Acquire), 0);
        assert!(!occupancy_path.exists());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn cleanup_enters_phase_a_before_video_and_rpc_teardown() {
        let LifecycleProductionFixture {
            _geometry_topology,
            owner,
            handle,
            mapping,
            correction_sink,
            correction_ingress,
            lio_sink,
            ingress_gate,
            shutdown,
            shutdown_listener: _shutdown_listener,
            shutdown_events,
            cloud_path,
            occupancy_path,
            ..
        } = lifecycle_production_fixture(55, "cleanup", true).await;
        let service =
            server::GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());
        let rpc_transport: Arc<dyn nuwax_rpc::RpcTransport> = Arc::new(CleanupRpcTransport);
        let rpc_servers = rpc_runtime::serve_geometry_rpc(service, rpc_transport)
            .await
            .expect("cleanup test LCM RPC servers");
        let video_shm_path = std::env::temp_dir().join(format!(
            "nuwax-cleanup-video-{}-{}.shm",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let video_uds_path = video_shm_path.with_extension("sock");
        let video_owner = video_relay::spawn_video_relay(&video_shm_path, &video_uds_path)
            .expect("cleanup test video owner");
        assert!(video_shm_path.exists());
        assert!(video_uds_path.exists());

        let stop_started = correction_sink.stop_started.notified();
        let owner = owner.expect("production lifecycle owner");
        let cleanup_shutdown = shutdown.clone();
        let cleanup_task = tokio::spawn(async move {
            cleanup_runtime(
                rpc_servers,
                video_owner,
                owner,
                &cleanup_shutdown,
                ExitCode::from(5),
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(3), stop_started)
            .await
            .expect("cleanup reached Correction only after video/RPC teardown");

        assert!(shutdown.is_readiness_revoked());
        assert!(!handle
            .task_control()
            .expect("production fixture task control")
            .admission_is_open());
        assert!(lio_sink
            .readiness_revoked_before_close
            .load(Ordering::Acquire));
        assert!(!ingress_gate.is_open());
        assert!(!correction_ingress.is_open());
        assert!(!video_shm_path.exists());
        assert!(!video_uds_path.exists());
        assert!(mapping.lock().snapshot().active);
        assert_eq!(
            shutdown_events.stages(),
            vec![lio_owner::GeometryShutdownStage::GeometryTasksSettled]
        );
        assert!(!cleanup_task.is_finished());

        correction_sink.stop_release.notify_one();
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(4), cleanup_task)
                .await
                .expect("cleanup task deadline")
                .expect("cleanup task join"),
            ExitCode::from(5),
            "successful teardown must preserve the earlier nonzero failure"
        );
        assert_eq!(
            shutdown_events.stages(),
            vec![
                lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                lio_owner::GeometryShutdownStage::CorrectionSettled,
                lio_owner::GeometryShutdownStage::MappingRetired,
                lio_owner::GeometryShutdownStage::LioJoined,
                lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
            ]
        );
        assert!(!cloud_path.exists());
        assert!(!occupancy_path.exists());
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
             if [[ \"$0\" == *start_geometry_runtime.sh ]]; then return 0; fi\n\
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
        let release_root = guard.test_dir.join("release-generation");
        let release_manifest = release_root.join("manifest.json");
        let prepare_release = r#"
import hashlib, json, pathlib, shutil, sys
source = pathlib.Path(sys.argv[1])
root = pathlib.Path(sys.argv[2])
entries = []
def canonical(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':')) + '\n').encode()
def add(role, name, executable=True, content=None):
    path = pathlib.Path('artifacts') / role.replace(':', '-') / name
    target = root / path
    target.parent.mkdir(parents=True, exist_ok=True)
    if content is None:
        shutil.copyfile(source, target)
    else:
        target.write_bytes(content)
    target.chmod(0o755 if executable else 0o644)
    payload = target.read_bytes()
    entries.append({'role': role, 'path': path.as_posix(), 'size': len(payload),
                    'sha256': hashlib.sha256(payload).hexdigest(), 'executable': executable})
add('geometry_runtime', source.name)
add('native_pgo_payload', 'native-pgo')
add('group_guardian', 'group-guardian')
add('offline_mapping_cli', 'mapping-cli')
add('native_lio_payload:fastlio2', 'nuwax-fastlio2-backend')
add('build_input_closure', 'build-inputs.json', False, b'{}\n')
entries.sort(key=lambda item: item['role'])
generation = hashlib.sha256(canonical(entries)).hexdigest()
manifest = {'schema_version': 'nuwax.geometry.release.v1', 'generation': generation,
            'native_lio_roles': ['fastlio2'], 'artifacts': entries}
manifest_path = root / 'manifest.json'
manifest_path.write_bytes(canonical(manifest))
manifest_path.chmod(0o644)
for directory in root.rglob('*'):
    if directory.is_dir():
        directory.chmod(0o755)
root.chmod(0o755)
"#;
        let release_status = std::process::Command::new("python3")
            .arg("-c")
            .arg(prepare_release)
            .arg("/bin/bash")
            .arg(&release_root)
            .status()
            .expect("prepare fake release manifest");
        assert!(release_status.success(), "prepare fake release failed");
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
                .env("BASH_ENV", &binary)
                .env("NUWAX_GEOMETRY_RELEASE_MANIFEST", &release_manifest)
                .env("NUWAX_GEOMETRY_RUNTIME_READY_FILE", &ready)
                .env("NUWAX_GEOMETRY_RUNTIME_PORT", "50051")
                .env("NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT", "2")
                .env("NUWAX_TEST_REVOKE_FILE", &revoke)
                .env("NUWAX_TEST_RUNTIME_PID_FILE", &runtime_pid_path)
                .env("NUWAX_GEOMETRY_LIO_BACKEND", "device")
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
