// Phase 6 (codex-round3 R3-4)：server.rs 多处 format! 调用被 clippy 1.88
// `uninlined_format_args` 报告(e.g. line 239/590/605/624/639 的 `format!("...{}", e)`),
// 风格性 lint。统一 file-level 标注不强行收敛——debug log 与 Status::internal
// 字符串拼接的可读性高于命名参数形式。
#![allow(clippy::uninlined_format_args)]

//! gRPC facade — `GeometryRuntime` service.
//!
//! 所有状态来自 `GeometryRuntimeHandle`（持有 LCM runtime SharedState）；
//! 与具体 backend 实现解耦。
//!
//! Phase 2 重构:
//! - 状态聚合统一走 `runtime_diagnostics`（health / readiness / blockers）
//! - typed payload encode 走 `typed_codec` 或 `nuwax_codec::lcm::schema`
//!   structured messages（禁止写 ad-hoc bytes）
//! - env 解析入口统一走 `channel_registry`

#[path = "server_planar.rs"]
mod planar;

use nuwax_codec::{lcm::schema, CodecError, CodecResult};
use nuwax_protocol::{LcmMessageType, LcmTopic};
#[cfg(test)]
use nuwax_protocol::RawLcmFrame;
use prost::Message;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Duration;
use tokio::sync::{watch, RwLock};
use tonic::{Request, Response, Status};
use tracing::info;

use crate::activation_contract::{ActivationRequestKey, InstalledActivationIdentityV2, InstalledTargetReceiptV2};
use crate::lcm_runtime::{
    accept_mapping_cloud_map_for_backend_candidate,
    publish_committed_global_map_in_frame, CloudMapNotification,
    CloudMapSessionAdmission, CloudMapSessionRegistration, CloudMapSessionReservation,
    CommittedMappingCloudMap, GeometryRuntimeHandle, MapRevisionEvent, NavigationUpdate, PoseState,
};
use crate::mapping_adapter::{
    finish_backend_retirement, MappingAdapter, MappingBackendRetirement,
    MappingCheckpointExportLease, MappingSessionSnapshot,
};
use crate::nav_runtime::{
    cancel as nav_cancel, compute_readiness, ensure_goal_id, readiness_to_reason, set_goal,
    CancelOutcome, MotionAdmissionProof, SetGoalInput,
};
use crate::occupancy_shm::{OccupancyGridShmWriter, OccupancySessionFenceTransaction};
use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::runtime_diagnostics::{self, RuntimeDiagnostics, ServiceState, SharedServiceState};
use crate::typed_codec;
use nuwax_exploration::{BoundarySegment, ExplorationBoundary};
use nuwax_mapping_cloud::{
    AssignedGeoreferenceV3, AssignedMapIdentityV3, AssignedMappingCloudGoalIntentV3,
    CloudQueryIdentityV1, MappingCloudOwner, MappingCloudProvider, MappingCloudRequest,
    MappingCloudTerminal, OsmClient,
};
use nuwax_nav_types_messages::geometry::Point2;

// The installed receipt already owns identity; mapping-cloud only projects it.
fn mapping_cloud_assigned_receipt(
    activation: &InstalledActivationIdentityV2,
    target: &InstalledTargetReceiptV2,
) -> ActivationReceiptV2 {
    ActivationReceiptV2 {
        request: Some(crate::activation_contract::request_to_wire(&activation.request)),
        activation: Some(activation.to_wire()),
        target: Some(target.to_wire()),
    }
}

// Mapping-cloud projects the assigned query result before shared goal admission.
fn mapping_cloud_assigned_goal_pose(
    intent: &AssignedMappingCloudGoalIntentV3,
) -> Result<schema::PoseStamped, Status> {
    let position = intent.map_position();
    let timestamp_ns = u64::try_from(intent.query().source_timestamp_ns)
        .map_err(|_| Status::internal("mapping-cloud timestamp is invalid"))?;
    Ok(schema::PoseStamped {
        header: schema::Header {
            seq: 0,
            stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000)
                .map_err(|_| Status::internal("mapping-cloud timestamp exceeds wire range"))?,
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: intent.active_map().map_frame_id.clone(),
        },
        pose: schema::Pose {
            position: [position.x_m, position.y_m, position.z_m],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
    })
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ActivationFaultPoint { BackendRestoreSubmission, SessionStart, OccupancyShmBegin,
    OccupancyInstall, CloudInstall, BackendCommit, ActivationRecord,
    SharedStateCommit, CacheClears, MapEvent, CloudEvent, TypedGlobalMap, Seal }

#[derive(Clone)]
pub struct GeometryRuntimeService {
    state: SharedServiceState,
    bridge: GeometryRuntimeHandle,
    base_frame_id: String,
    point_cloud_binding: Option<(LcmTopic, LcmMessageType)>,
    /// Native mode publishes raw cloud and IMU to generation-private channels.
    native_raw_input_binding: Option<((LcmTopic, LcmMessageType), (LcmTopic, LcmMessageType))>,
    native_raw_sensor_frame: Option<String>,
    native_input_failure: Arc<parking_lot::Mutex<Option<String>>>,
    native_input_delivery: Option<Arc<crate::native_input_delivery::NativeInputDelivery>>,
    native_self_filter: Arc<Result<Option<crate::self_cloud::NativeSelfFilter>, String>>,
    shutdown: Option<ShutdownCoordinator>,
    mapping_transition: Arc<tokio::sync::Mutex<()>>,
    graph_artifacts: Arc<parking_lot::Mutex<crate::planar_artifacts::Store>>,
    activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl,
    relocalization_ingress: Option<crate::correction_coordinator::RelocalizationCommandIngress>,
    canonical_correction: Option<Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>>,
    mapping_cloud_owner: Arc<MappingCloudOwner>,
    activation_checkpoint_owners: Arc<crate::activation_request::CheckpointOwnerInternerV2>,
    #[cfg(test)]
    activation_fault_after:
        Arc<parking_lot::Mutex<Option<ActivationFaultPoint>>>,
    #[cfg(test)]
    stop_phase_a_after_detach: Arc<AtomicBool>,
    #[cfg(test)]
    fail_start_mapping_stop_transition: Arc<AtomicBool>,
    #[cfg(test)]
    start_mapping_before_cloud_clear: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    stop_before_post_zero_lifecycle: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    stop_before_graceful: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
}

#[cfg(not(test))]
const STOP_LISTENER_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const STOP_LISTENER_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const STOP_LIFECYCLE_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const STOP_LIFECYCLE_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const STOP_EXPLORATION_DISPATCH_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const STOP_EXPLORATION_DISPATCH_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const STOP_NAVIGATION_DISPATCH_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const STOP_NAVIGATION_DISPATCH_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const STOP_EXPLORATION_SCHEDULER_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const STOP_EXPLORATION_SCHEDULER_TIMEOUT: Duration = Duration::from_millis(500);
#[cfg(not(test))]
const STOP_EXPLORATION_SAFETY_TIMEOUT: Duration = Duration::from_secs(6);
#[cfg(test)]
const STOP_EXPLORATION_SAFETY_TIMEOUT: Duration = Duration::from_millis(250);
const STOP_EXPLORATION_SAFETY_POLL: Duration = Duration::from_millis(20);
const MAPPING_BACKEND_CHECKPOINT_TIMEOUT: Duration = Duration::from_secs(10);
#[cfg(not(test))]
const MAPPING_TRANSITION_TIMEOUT: Duration = Duration::from_secs(30);
#[cfg(test)]
const MAPPING_TRANSITION_TIMEOUT: Duration = Duration::from_secs(1);
const MAPPING_CHECKPOINT_PROTO_ENVELOPE_MAX_BYTES: usize = 120 * 1024 * 1024;
#[cfg(not(test))]
const OCCUPANCY_SHM_COMMIT_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const OCCUPANCY_SHM_COMMIT_TIMEOUT: Duration = Duration::from_millis(250);

struct MappingCheckpointExportGuard {
    adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    lease: Option<MappingCheckpointExportLease>,
}

impl MappingCheckpointExportGuard {
    fn new(
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        lease: MappingCheckpointExportLease,
    ) -> Self {
        Self {
            adapter,
            lease: Some(lease),
        }
    }

    fn lease(&self) -> &MappingCheckpointExportLease {
        self.lease
            .as_ref()
            .expect("mapping checkpoint export guard is active")
    }

    fn finish(&mut self) {
        if let Some(lease) = self.lease.take() {
            self.adapter.lock().finish_backend_checkpoint_export(&lease);
        }
    }
}

impl Drop for MappingCheckpointExportGuard {
    fn drop(&mut self) {
        self.finish();
    }
}

struct StartMappingRpcWaiter {
    cancellation_tx: Option<watch::Sender<bool>>,
}

impl StartMappingRpcWaiter {
    fn new(cancellation_tx: watch::Sender<bool>) -> Self {
        Self {
            cancellation_tx: Some(cancellation_tx),
        }
    }

    fn complete(&mut self) {
        self.cancellation_tx.take();
    }
}

impl Drop for StartMappingRpcWaiter {
    fn drop(&mut self) {
        if let Some(cancellation_tx) = self.cancellation_tx.take() {
            cancellation_tx.send_replace(true);
        }
    }
}


fn configured_base_frame() -> String {
    resolve_base_frame_alias(
        std::env::var("NUWAX_BASE_FRAME").ok().as_deref(),
        std::env::var("NUWAX_GEOMETRY_BASE_FRAME").ok().as_deref(),
    )
}

fn resolve_base_frame_alias(preferred: Option<&str>, legacy: Option<&str>) -> String {
    preferred
        .and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        .or_else(|| {
            legacy.and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        })
        .unwrap_or_else(|| "base_link".to_string())
}

#[derive(Clone)]
pub(crate) struct ReadinessAuthority {
    inner: Arc<ReadinessAuthorityInner>,
}

struct ReadinessAuthorityInner {
    revoked: parking_lot::Mutex<bool>,
    revoked_tx: watch::Sender<bool>,
}

impl ReadinessAuthority {
    pub(crate) fn new() -> Self {
        let (revoked_tx, _) = watch::channel(false);
        Self {
            inner: Arc::new(ReadinessAuthorityInner {
                revoked: parking_lot::Mutex::new(false),
                revoked_tx,
            }),
        }
    }

    pub(crate) fn revoke(&self) {
        let mut revoked = self.inner.revoked.lock();
        if !*revoked {
            *revoked = true;
            self.inner.revoked_tx.send_replace(true);
        }
    }

    pub(crate) fn run_if_active<T>(&self, action: impl FnOnce() -> T) -> Result<T, String> {
        let revoked = self.inner.revoked.lock();
        if *revoked {
            return Err("readiness_revoked".to_owned());
        }
        Ok(action())
    }

    pub(crate) async fn wait_for_revocation(&self) {
        let mut revoked_rx = self.inner.revoked_tx.subscribe();
        if *revoked_rx.borrow() {
            return;
        }
        let _ = revoked_rx.wait_for(|revoked| *revoked).await;
    }

    #[cfg(test)]
    pub(crate) fn is_revoked(&self) -> bool {
        *self.inner.revoked.lock()
    }
}

impl Default for ReadinessAuthority {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct ShutdownCoordinator {
    request_tx: watch::Sender<bool>,
    listener_closed_rx: watch::Receiver<bool>,
    graceful_tx: watch::Sender<bool>,
    readiness: ReadinessAuthority,
    shutdown_failed: Arc<AtomicBool>,
    lio_gate: Option<Arc<crate::lio_ingress::LioIngressRoutingGate>>,
    correction_phase_a: Option<crate::correction_coordinator::CorrectionPhaseAControl>,
    geometry_tasks: Option<crate::lcm_runtime::GeometryRuntimeTaskControl>,
    stop_single_flight: Arc<tokio::sync::Mutex<()>>,
    mapping_transition: Arc<tokio::sync::Mutex<()>>,
    #[cfg(test)]
    correction_settlement_blocker: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
}

pub struct ShutdownListener {
    request_rx: watch::Receiver<bool>,
    listener_closed_tx: watch::Sender<bool>,
    graceful_rx: watch::Receiver<bool>,
}

type SubscriptionStream<T> =
    std::pin::Pin<Box<dyn futures::Stream<Item = Result<T, Status>> + Send + 'static>>;

async fn wait_for_subscription_shutdown(mut graceful_rx: Option<watch::Receiver<bool>>) {
    match graceful_rx.as_mut() {
        Some(receiver) => {
            let _ = receiver.wait_for(|requested| *requested).await;
        }
        None => std::future::pending::<()>().await,
    }
}

fn spawn_subscription_stream<T, F>(
    shutdown: Option<&ShutdownCoordinator>,
    tx: tokio::sync::mpsc::Sender<Result<T, Status>>,
    rx: tokio::sync::mpsc::Receiver<Result<T, Status>>,
    producer: F,
) -> SubscriptionStream<T>
where
    T: Send + 'static,
    F: std::future::Future<Output = ()> + Send + 'static,
{
    // 订阅同一优雅关闭信号，保持先释放生命周期锁、再结束远程流的既有顺序。
    let graceful_rx = shutdown.map(|coordinator| coordinator.graceful_tx.subscribe());
    let stream_graceful_rx = graceful_rx.clone();
    tokio::spawn(async move {
        tokio::select! {
            biased;
            _ = wait_for_subscription_shutdown(graceful_rx) => {}
            _ = tx.closed() => {}
            _ = producer => {}
        }
        // 分支结束即丢弃整个生产计算，连同订阅、锁等待和未完成发送一起回收。
    });
    // 关闭时放弃尚未交付的状态通知，避免慢客户端继续排空已缓存的大地图。
    Box::pin(futures::StreamExt::take_until(
        tokio_stream::wrappers::ReceiverStream::new(rx),
        wait_for_subscription_shutdown(stream_graceful_rx),
    ))
}

#[cfg(test)]
pub(crate) fn shutdown_channel() -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(None, None, None)
}

#[cfg(test)]
pub(crate) fn production_shutdown_channel(
    lio_gate: Arc<crate::lio_ingress::LioIngressRoutingGate>,
    correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
) -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(
        Some(lio_gate),
        Some(correction_phase_a),
        None,
    )
}

pub(crate) fn production_shutdown_channel_with_task_control(
    lio_gate: Arc<crate::lio_ingress::LioIngressRoutingGate>,
    correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
    geometry_tasks: crate::lcm_runtime::GeometryRuntimeTaskControl,
) -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(
        Some(lio_gate),
        Some(correction_phase_a),
        Some(geometry_tasks),
    )
}

fn shutdown_channel_inner(
    lio_gate: Option<Arc<crate::lio_ingress::LioIngressRoutingGate>>,
    correction_phase_a: Option<crate::correction_coordinator::CorrectionPhaseAControl>,
    geometry_tasks: Option<crate::lcm_runtime::GeometryRuntimeTaskControl>,
) -> (ShutdownCoordinator, ShutdownListener) {
    let (request_tx, request_rx) = watch::channel(false);
    let (listener_closed_tx, listener_closed_rx) = watch::channel(false);
    let (graceful_tx, graceful_rx) = watch::channel(false);
    let readiness = ReadinessAuthority::default();
    let mapping_transition = Arc::new(tokio::sync::Mutex::new(()));
    (
        ShutdownCoordinator {
            request_tx,
            listener_closed_rx,
            graceful_tx,
            readiness,
            shutdown_failed: Arc::new(AtomicBool::new(false)),
            lio_gate,
            correction_phase_a,
                geometry_tasks,
            stop_single_flight: Arc::new(tokio::sync::Mutex::new(())),
            mapping_transition,
            #[cfg(test)]
            correction_settlement_blocker: Arc::new(parking_lot::Mutex::new(None)),
        },
        ShutdownListener {
            request_rx,
            listener_closed_tx,
            graceful_rx,
        },
    )
}

impl ShutdownCoordinator {
    pub(crate) fn begin_phase_a(&self) {
        self.revoke_readiness();
        if let Some(geometry_tasks) = &self.geometry_tasks {
            geometry_tasks.close_admission();
        }
        if let Some(lio_gate) = &self.lio_gate {
            crate::lio_owner::LioIngressSink::close(lio_gate.as_ref());
        }
        if let Some(correction_phase_a) = &self.correction_phase_a {
            correction_phase_a.begin();
        }
    }

    pub(crate) fn begin_process_stop_phase_a(
        &self,
    ) -> Option<crate::correction_coordinator::CorrectionProcessStopAttempt> {
        self.revoke_readiness();
        if let Some(geometry_tasks) = &self.geometry_tasks {
            geometry_tasks.close_admission();
        }
        if let Some(lio_gate) = &self.lio_gate {
            crate::lio_owner::LioIngressSink::close(lio_gate.as_ref());
        }
        let attempt = self
            .correction_phase_a
            .as_ref()
            .map(crate::correction_coordinator::CorrectionPhaseAControl::begin_process_stop);
        attempt
    }

    pub fn request_shutdown(&self) {
        self.begin_phase_a();
        self.request_tx.send_replace(true);
    }

    pub(crate) fn is_shutdown_requested(&self) -> bool {
        *self.request_tx.borrow()
    }

    pub(crate) fn has_shutdown_failed(&self) -> bool {
        self.shutdown_failed.load(Ordering::Acquire)
            || self.correction_phase_a.as_ref().is_some_and(
                crate::correction_coordinator::CorrectionPhaseAControl::process_stop_failed,
            )

    }

    pub(crate) fn revoke_readiness(&self) {
        self.readiness.revoke();
    }

    pub(crate) async fn wait_for_readiness_revocation(&self) {
        self.readiness.wait_for_revocation().await;
    }

    #[cfg(test)]
    pub(crate) fn is_readiness_revoked(&self) -> bool {
        self.readiness.is_revoked()
    }

    fn mark_shutdown_failed(&self) {
        self.shutdown_failed.store(true, Ordering::Release);
        self.revoke_readiness();
    }

    pub(crate) fn force_grpc_shutdown(&self) {
        self.request_shutdown();
        self.graceful_tx.send_replace(true);
    }

    pub(crate) fn publish_graceful_shutdown(&self) {
        self.graceful_tx.send_replace(true);
    }


    async fn wait_for_correction_settlement(&self) {
        #[cfg(test)]
        {
            let blocker = self.correction_settlement_blocker.lock().take();
            if let Some(barrier) = blocker {
                barrier.wait().await;
                std::future::pending::<()>().await;
            }
        }
        if let Some(correction_phase_a) = &self.correction_phase_a {
            correction_phase_a.wait_settled().await;
        }
    }

    pub(crate) async fn wait_for_process_stop_failure(&self) {
        match &self.correction_phase_a {
            Some(correction_phase_a) => correction_phase_a.wait_process_stop_failed().await,
            None => std::future::pending::<()>().await,
        }
    }

    pub(crate) async fn request_and_wait_for_listener(&self) -> Result<(), Status> {
        let mut listener_closed_rx = self.listener_closed_rx.clone();
        if *listener_closed_rx.borrow() {
            return Ok(());
        }

        self.request_shutdown();
        let listener_result = tokio::time::timeout(
            STOP_LISTENER_TIMEOUT,
            listener_closed_rx.wait_for(|closed| *closed),
        )
        .await;

        match listener_result {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(_)) => {
                self.mark_shutdown_failed();
                Err(Status::unavailable("gRPC listener 关闭确认通道已断开"))
            }
            Err(_) => {
                self.mark_shutdown_failed();
                Err(Status::deadline_exceeded("关闭 gRPC listener 超时"))
            }
        }
    }
}

impl ShutdownListener {
    #[cfg(test)]
    pub async fn wait_for_request(&mut self) {
        if *self.request_rx.borrow() {
            return;
        }
        let _ = self.request_rx.wait_for(|requested| *requested).await;
    }

    #[cfg(test)]
    pub fn mark_listener_closed(&self) {
        self.listener_closed_tx.send_replace(true);
    }

    pub fn into_parts(
        self,
    ) -> (
        watch::Receiver<bool>,
        watch::Sender<bool>,
        watch::Receiver<bool>,
    ) {
        (self.request_rx, self.listener_closed_tx, self.graceful_rx)
    }
}

impl GeometryRuntimeService {
    /// 暂停新计算，让原位后端的既有批次正常提交，再将锁交给重定位准入。
    async fn drain_mapping_before_relocalization(
        &self,
    ) -> Result<(tokio::sync::OwnedMutexGuard<()>, Option<MappingCheckpointExportGuard>), Status> {
        tokio::time::timeout(MAPPING_TRANSITION_TIMEOUT, async {
            let mut lifecycle = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            self.ensure_running_admission("Relocalize").await?;
            let Some(adapter) = self.bridge.mapping_adapter.as_ref().cloned() else {
                return Ok((lifecycle, None));
            };
            let lease = {
                let mut mapping = adapter.lock();
                if !mapping.is_active() {
                    return Ok((lifecycle, None));
                }
                mapping.begin_backend_checkpoint_export().map_err(Status::failed_precondition)?
            };
            // 只复用计算排空屏障，不刷新或导出地图资产。
            let barrier = MappingCheckpointExportGuard::new(adapter.clone(), lease);
            loop {
                self.ensure_running_admission("Relocalize").await?;
                let drained = adapter.lock().backend_checkpoint_client_if_drained(barrier.lease())
                    .map_err(Status::failed_precondition)?.is_some();
                if drained {
                    return Ok((lifecycle, Some(barrier)));
                }
                drop(lifecycle);
                tokio::time::sleep(Duration::from_millis(5)).await;
                lifecycle = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            }
        }).await.map_err(|_| Status::deadline_exceeded("重定位准入前建图计算未能按时排空"))?
    }

    fn start_mapping_cloud_owner(bridge: &GeometryRuntimeHandle) -> Arc<MappingCloudOwner> {
        let owner_epoch = bridge.nav.producer_runtime_generation();
        Arc::new(
            MappingCloudOwner::start(
                owner_epoch,
                MappingCloudProvider::OsmNominatim(Arc::new(OsmClient::new())),
            )
            .expect("Geometry navigation runtime generation is a valid mapping-cloud owner epoch"),
        )
    }

    pub(crate) fn mapping_cloud_owner(&self) -> Arc<MappingCloudOwner> {
        Arc::clone(&self.mapping_cloud_owner)
    }

    pub(crate) async fn run_if_running<T>(&self, action: impl FnOnce() -> T) -> Result<T, String> {
        let state = self.state.read().await;
        if state.phase != "running" {
            return Err(state.phase.clone());
        }
        match &self.shutdown {
            Some(shutdown) => shutdown.readiness.run_if_active(action),
            None => Ok(action()),
        }
    }

    #[cfg(test)]
    pub fn new(bridge: GeometryRuntimeHandle) -> Self {
        let point_cloud_binding = (
            crate::channel_registry::channel_for(crate::channel_registry::ChannelKey::PointCloud),
            LcmMessageType::new("sensor_msgs/PointCloud2")
                .expect("registry PointCloud2 message type must remain valid"),
        );
        Self::new_with_point_cloud_binding(bridge, point_cloud_binding)
    }

    #[cfg(test)]
    pub(crate) fn new_with_point_cloud_binding(
        bridge: GeometryRuntimeHandle,
        point_cloud_binding: (LcmTopic, LcmMessageType),
    ) -> Self {
        let mapping_cloud_owner = Self::start_mapping_cloud_owner(&bridge);
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
            base_frame_id: configured_base_frame(),
            point_cloud_binding: Some(point_cloud_binding),
            native_raw_input_binding: None,
            native_raw_sensor_frame: None,
            native_input_failure: Arc::new(parking_lot::Mutex::new(None)),
            native_input_delivery: None,
            native_self_filter: Arc::new(Ok(None)),
            shutdown: None,
            mapping_transition: Arc::new(tokio::sync::Mutex::new(())),
            graph_artifacts: Arc::new(parking_lot::Mutex::new(crate::planar_artifacts::Store::default())),
            activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl::new(
                1,
            ),
            relocalization_ingress: None,
            canonical_correction: None,
            mapping_cloud_owner,
            activation_checkpoint_owners: Arc::new(crate::activation_request::CheckpointOwnerInternerV2::default()),
            activation_fault_after: Arc::new(parking_lot::Mutex::new(None)),
            stop_phase_a_after_detach: Arc::new(AtomicBool::new(false)),
            fail_start_mapping_stop_transition: Arc::new(AtomicBool::new(false)),
            start_mapping_before_cloud_clear: Arc::new(parking_lot::Mutex::new(None)),
            stop_before_post_zero_lifecycle: Arc::new(parking_lot::Mutex::new(None)),
            stop_before_graceful: Arc::new(parking_lot::Mutex::new(None)),
        }
    }

    #[cfg(test)]
    pub fn new_with_shutdown(bridge: GeometryRuntimeHandle, shutdown: ShutdownCoordinator) -> Self {
        let mut service = Self::new(bridge);
        service.mapping_transition = shutdown.mapping_transition.clone();
        if let Some(phase_a) = shutdown.correction_phase_a.as_ref() {
            service.activation_receipts = phase_a.activated_target_receipts();
        }
        service.shutdown = Some(shutdown);
        service
    }

    #[cfg(test)]
    fn new_without_point_cloud_binding(bridge: GeometryRuntimeHandle) -> Self {
        let mut service = Self::new(bridge);
        service.point_cloud_binding = None;
        service
    }

    pub(crate) fn with_canonical_correction(
        mut self, applier: Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>,
    ) -> Self {
        self.canonical_correction = Some(applier);
        self
    }

    pub(crate) fn new_with_shutdown_and_point_cloud_binding(
        bridge: GeometryRuntimeHandle,
        shutdown: ShutdownCoordinator,
        point_cloud_binding: Option<(LcmTopic, LcmMessageType)>,
        native_raw_input_binding: Option<((LcmTopic, LcmMessageType), (LcmTopic, LcmMessageType))>,
        native_ack_binding: Option<((LcmTopic, LcmMessageType), (LcmTopic, LcmMessageType))>,
        native_raw_sensor_frame: Option<String>,
        relocalization_ingress: Option<crate::correction_coordinator::RelocalizationCommandIngress>,
    ) -> Self {
        let activation_receipts = shutdown
            .correction_phase_a
            .as_ref()
            .map(crate::correction_coordinator::CorrectionPhaseAControl::activated_target_receipts)
            .unwrap_or_else(|| {
                crate::correction_coordinator::ActivatedTargetReceiptControl::new(1)
            });
        let mapping_cloud_owner = Self::start_mapping_cloud_owner(&bridge);
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
            base_frame_id: configured_base_frame(),
            point_cloud_binding,
            native_raw_input_binding,
            native_raw_sensor_frame,
            native_input_failure: Arc::new(parking_lot::Mutex::new(None)),
            native_input_delivery: native_ack_binding.map(|(ack, frame)| Arc::new(crate::native_input_delivery::NativeInputDelivery::new(ack, frame).with_shutdown(shutdown.request_tx.subscribe()))),
            native_self_filter: Arc::new(crate::self_cloud::NativeSelfFilter::from_env(&configured_base_frame())),
            mapping_transition: shutdown.mapping_transition.clone(),
            graph_artifacts: Arc::new(parking_lot::Mutex::new(crate::planar_artifacts::Store::default())),
            shutdown: Some(shutdown),
            activation_receipts,
            relocalization_ingress,
            canonical_correction: None,
            mapping_cloud_owner,
            activation_checkpoint_owners: Arc::new(crate::activation_request::CheckpointOwnerInternerV2::default()),
            #[cfg(test)]
            activation_fault_after: Arc::new(parking_lot::Mutex::new(None)),
            #[cfg(test)]
            stop_phase_a_after_detach: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            fail_start_mapping_stop_transition: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            start_mapping_before_cloud_clear: Arc::new(parking_lot::Mutex::new(None)),
            #[cfg(test)]
            stop_before_post_zero_lifecycle: Arc::new(parking_lot::Mutex::new(None)),
            #[cfg(test)]
            stop_before_graceful: Arc::new(parking_lot::Mutex::new(None)),
        }
    }

    /// 构造本进程的 RuntimeDiagnostics(读 ServiceState 算 uptime)
    async fn diagnostics(&self) -> RuntimeDiagnostics {
        let s = self.state.read().await;
        RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns)
    }

    async fn ensure_running_admission(&self, operation: &str) -> Result<(), Status> {
        let phase = self.state.read().await.phase.clone();
        if phase != "running" {
            return Err(Status::failed_precondition(format!(
                "{operation} requires runtime phase=running, current={phase}"
            )));
        }
        Ok(())
    }

    fn mapping_state_with_receipt(
        &self,
        snapshot: crate::mapping_adapter::MappingSessionSnapshot,
        message: impl Into<String>,
    ) -> MappingSessionState {
        let mut state = mapping_session_state(snapshot, message);
        self.activation_receipts.decorate_current_state(&mut state);
        if let Some(binding)=self.bridge.mapping_adapter.as_ref().and_then(|a|a.lock().planar.clone()) {
            state.runtime_generation=binding.client.identity().runtime_generation;
            state.planar_mode=if self.bridge.mapping_adapter.as_ref().unwrap().lock().planar_localization {"localization"}else{"mapping"}.into();
        }
        state
    }


    #[allow(clippy::too_many_arguments)]
    async fn finish_started_mapping_transition(
        &self,
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        snapshot: MappingSessionSnapshot,
        cloud_map_reservation: CloudMapSessionReservation,
        mut projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
        lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        _mapping_transition: tokio::sync::OwnedMutexGuard<()>,
    ) -> Result<Response<MappingSessionState>, Status> {
        // This retained transition still owns L. Clear predecessor localization
        // before replay/publication or a successful reply, without holding the
        // shared-state lock across backend creation or retirement.
        {
            let mut shared = self.bridge.state.write().await;
            crate::lcm_runtime::reset_canonical_localization_locked(&mut shared, None);
        }
        self.bridge
            .revoke_committed_map_projection_locked(&mut projection_guard)
            .map_err(Status::unavailable)?;
        drop(projection_guard);
        drop(lifecycle_guard);

        #[cfg(test)]
        if self
            .fail_start_mapping_stop_transition
            .swap(false, Ordering::AcqRel)
        {
            adapter.lock().exhaust_backend_generation_for_tests();
        }

        let mut occupancy_fence = None;
        if let Some(writer) = &self.bridge.occupancy_shm {
            let begin_result = writer
                .lock()
                .begin_session_transactional(snapshot.session_generation);
            match begin_result {
                Ok(transaction) => {
                    occupancy_fence = Some(OccupancySessionFenceRollback {
                        writer: Arc::clone(writer),
                        transaction,
                    });
                }
                Err(error) => {
                    let outcome = finish_failed_mapping_start_rollback(
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        self.bridge
                            .mapping_lifecycle_lock
                            .clone()
                            .lock_owned()
                            .await,
                        &snapshot,
                        None,
                        format!("occupancy SHM session fence failed: {error}"),
                    )
                    .await;
                    return Err(Status::internal(outcome.detail_with_diagnostics()));
                }
            }
        }

        let clear = cloud_map_reservation
            .activate(snapshot.session_generation)
            .await;
        let clear_failure = match &clear {
            CloudMapSessionAdmission::Registered(_) => {
                validate_deferred_cloud_map_clear(&clear, snapshot.session_generation).err()
            }
            CloudMapSessionAdmission::AdmissionClosed(detail)
            | CloudMapSessionAdmission::ClearRejected(detail) => Some(detail.clone()),
            CloudMapSessionAdmission::WriterUnavailable => {
                Some("cloud-map writer is unavailable".to_owned())
            }
        };
        if let Some(detail) = clear_failure {
            let detail = format!("non-deferred StartMapping cloud-map clear failed: {detail}");
            let outcome = finish_failed_mapping_start_rollback(
                &adapter,
                &self.bridge.mapping_lifecycle_lock,
                self.bridge
                    .mapping_lifecycle_lock
                    .clone()
                    .lock_owned()
                    .await,
                &snapshot,
                occupancy_fence.take(),
                detail,
            )
            .await;
            return Err(Status::unavailable(outcome.detail_with_diagnostics()));
        }
        drop(occupancy_fence.take());
        Ok(Response::new(self.mapping_state_with_receipt(
            snapshot,
            "mapping session active",
        )))
    }

    #[cfg(test)]
    fn inject_activation_fault_after(
        &self,
        step: ActivationFaultPoint,
    ) {
        *self.activation_fault_after.lock() = Some(step);
    }

    #[cfg(test)]
    fn inject_stop_phase_a_after_detach(&self) {
        self.stop_phase_a_after_detach
            .store(true, Ordering::Release);
    }


    #[cfg(test)]
    fn inject_start_mapping_before_cloud_clear_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.start_mapping_before_cloud_clear.lock() = Some(barrier);
    }


    #[cfg(test)]
    fn inject_stop_before_post_zero_lifecycle_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.stop_before_post_zero_lifecycle.lock() = Some(barrier);
    }


    #[cfg(test)]
    fn inject_stop_before_graceful_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.stop_before_graceful.lock() = Some(barrier);
    }
}

#[cfg(test)]
fn take_activation_fault_after(
    service: &GeometryRuntimeService,
    step: ActivationFaultPoint,
) -> bool {
    let mut fault = service.activation_fault_after.lock();
    if *fault == Some(step) {
        *fault = None;
        true
    } else {
        false
    }
}

fn relocalization_command_status(
    error: crate::correction_coordinator::RelocalizationCommandError,
) -> Status {
    use crate::correction_coordinator::RelocalizationCommandError;

    let detail = match error {
        RelocalizationCommandError::InvalidRequest(detail) => return Status::failed_precondition(detail),
        RelocalizationCommandError::Stopped => "RELOCALIZATION_OWNER_STOPPED",
        RelocalizationCommandError::OwnerUnavailable => "RELOCALIZATION_OWNER_UNAVAILABLE",
        RelocalizationCommandError::ResponseLost => "RELOCALIZATION_RESPONSE_LOST",
    };
    Status::unavailable(detail)
}


pub(crate) fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn mapping_session_state(
    snapshot: crate::mapping_adapter::MappingSessionSnapshot,
    message: impl Into<String>,
) -> MappingSessionState {
    MappingSessionState {
        target_kind:String::new(),planar_mode:String::new(),
        success: true,
        phase: snapshot.phase.as_str().to_string(),
        message: message.into(),
        session_id: snapshot.session_id,
        map_name: snapshot.map_name,
        session_generation: snapshot.session_generation,
        revision: snapshot.revision,
        active: snapshot.active,
        idempotent_replay: snapshot.idempotent_replay,

        mapping_backend_kind: snapshot.mapping_backend_kind,
        activation_receipt_schema_version: 0,
        activation_receipt_v2: None,
        relocalization_policy_v2: None,
        activation_receipt_disposition: ActivationReceiptDisposition::Unspecified as i32,
        runtime_generation: 0,
        backend_generation: 0,
        activation_request_id: String::new(),


        relocalization_target_available: false,


    }
}

type ActivationRequestIdentityClass = crate::activation_request::ActivationRequestMetadataV2;


pub(crate) fn validate_canonical_protocol_id(value: &str, label: &str) -> Result<(), String> {
    if value.is_empty()
        || value.len() > 128
        || value != value.trim()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:-".contains(&byte))
    {
        return Err(format!(
            "{label} must be 1..=128 canonical ASCII [A-Za-z0-9._:-]"
        ));
    }
    Ok(())
}

pub(crate) fn validate_canonical_text(value: &str, label: &str) -> Result<(), String> {
    if value.is_empty() || value.len() > 256 || value != value.trim() {
        return Err(format!("{label} must be 1..=256 bytes and already trimmed"));
    }
    Ok(())
}

fn classify_activation_request_identity(
    request: &ActivateMapRequest,
    runtime_generation: u64,
) -> Result<ActivationRequestIdentityClass, String> {
    crate::activation_request::ActivationRequestMetadataV2::from_wire(request, runtime_generation)
}


fn activation_failure_state(
    service: &GeometryRuntimeService,
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    token: &crate::correction_coordinator::ActivationToken,
    disposition: ActivationReceiptDisposition,
    detail: &str,
) -> MappingSessionState {
    let state = mapping_session_state(adapter.lock().snapshot(), detail);
    service
        .activation_receipts
        .invalidate(token, disposition, detail, state)
}

#[allow(clippy::too_many_arguments)]
async fn settle_failed_activation_retirement(
    service: &GeometryRuntimeService,
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    bridge: &GeometryRuntimeHandle,
    token: &crate::correction_coordinator::ActivationToken,
    _request_identity: ActivationRequestKey,
    retirement: Result<
        Option<crate::mapping_adapter::MappingBackendRetirement>,
        crate::mapping_adapter::MappingLifecycleError,
    >,
    requested_disposition: ActivationReceiptDisposition,
    detail: impl Into<String>,
    state: MappingSessionState,
) -> MappingSessionState {
    let mut disposition = requested_disposition;
    let mut detail = detail.into();
    let retirement = match retirement {
        Ok(retirement) => retirement,
        Err(error) => {
            disposition = ActivationReceiptDisposition::OutcomeUnknown;
            detail = format!("{detail}; exact backend detach failed: {error}");
            None
        }
    };
    if let Some(retirement) = retirement {
        if let Err(error) =
            finish_backend_retirement(adapter, &bridge.mapping_lifecycle_lock, Some(retirement))
                .await
        {
            disposition = ActivationReceiptDisposition::OutcomeUnknown;
            detail = format!("{detail}; backend retirement failed: {error}");
        }
        if !service.activation_receipts.validate_token(token) {
            disposition = ActivationReceiptDisposition::OutcomeUnknown;
            detail.push_str("; activation token retired during backend join");
        }
    }

    let _lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
    if adapter.lock().is_active() {
        disposition = ActivationReceiptDisposition::OutcomeUnknown;
        detail.push_str("; adapter remained active after retirement settlement");
    }
    service
        .activation_receipts
        .invalidate(token, disposition, &detail, state)
}

async fn prepare_assigned_activation_archives(
    service: &GeometryRuntimeService,
    owner: &crate::activation_request::CanonicalCheckpointOwnerV2,
    expected_map_frame_id: &str,
) -> Result<(Option<crate::pointcloud::PreparedFineCloud>, Option<crate::dense_cloud::DenseCloud>), Status> {
    // Reopen an independent read-only cursor from the admitted pinned object,
    // never a transfer-store lookup that could expire or name a recycled body.
    let open = |artifact: &crate::activation_request::SealedArtifactOwnerV2| {
        std::fs::File::open(format!("/proc/self/fd/{}", std::os::fd::AsRawFd::as_raw_fd(artifact.file())))
            .map_err(|error| Status::internal(error.to_string()))
    };
    let fine = if let Some(artifact) = owner.fine_map() {
        let file = open(artifact)?;
        let source_session = owner.occupancy_receipt().session_id.clone();
        let source_generation = owner.occupancy_receipt().session_generation;
        let receipt = owner.cloud_receipt().clone();
        let receiver = service.bridge.register_navigation_map_operation(async move {
            tokio::task::spawn_blocking(move || crate::pointcloud::prepare_fine_cloud_archive(file,
                &source_session, source_generation, receipt.cloud_revision, receipt.source_timestamp_ns))
                .await.map_err(|error| error.to_string()).and_then(|value| value)
        }).map_err(Status::unavailable)?;
        Some(receiver.await.map_err(|error| Status::unavailable(error.to_string()))?.map_err(Status::invalid_argument)?)
    } else { None };
    let dense = if let Some(artifact) = owner.dense_cloud() {
        let file = open(artifact)?;
        let voxel_size = fine.as_ref().map_or_else(|| owner.cloud().voxel_size(), |fine| fine.voxel_size());
        let expected = crate::dense_cloud::Metadata {
            version: 1, session_id: owner.occupancy_receipt().session_id.clone(),
            session_generation: owner.occupancy_receipt().session_generation,
            cloud_revision: owner.cloud_receipt().cloud_revision,
            source_timestamp_ns: owner.cloud_receipt().source_timestamp_ns,
            frame_id: expected_map_frame_id.to_owned(), point_count: 0, bounds_min: [0.; 3], bounds_max: [0.; 3],
        };
        let receiver = service.bridge.register_navigation_map_operation(async move {
            tokio::task::spawn_blocking(move || crate::dense_cloud::DenseCloud::from_archive_with_voxel_size(file, &expected, voxel_size))
                .await.map_err(|error| error.to_string()).and_then(|value| value)
        }).map_err(Status::unavailable)?;
        Some(receiver.await.map_err(|error| Status::unavailable(error.to_string()))?.map_err(Status::invalid_argument)?)
    } else { None };
    Ok((fine, dense))
}

#[allow(clippy::too_many_lines)]
async fn activate_map_r1(
    service: GeometryRuntimeService,
    request: ActivateMapRequest,
) -> Result<Response<MappingSessionState>, Status> {
    validate_mapping_checkpoint_envelope_size("ActivateMap request", request.encoded_len())
        .map_err(Status::resource_exhausted)?;
    let identity_class = classify_activation_request_identity(
        &request,
        service.activation_receipts.runtime_generation(),
    )
    .map_err(Status::invalid_argument)?;
    if request.session_id.trim().is_empty() || request.map_name.trim().is_empty() {
        return Err(Status::invalid_argument(
            "session_id and map_name must not be empty",
        ));
    }
    if !service.bridge.lcm_backend_available {
        return Err(Status::unavailable("mapping LCM backend is unavailable"));
    }
    let adapter = service
        .bridge
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
        .clone();
    let (expected_map_frame_id, selected_backend_kind) = {
        let adapter = adapter.lock();
        (
            adapter.configured_map_frame_id().to_owned(),
            adapter.backend_kind(),
        )
    };

    if selected_backend_kind==crate::mapping_backend::MappingBackendKind::SlamToolbox {
        return service.activate_planar_map(request,identity_class).await;
    }

    let (session_id, map_name, checkpoint, checkpoint_timestamp_ns, assigned_checkpoint,
         immutable_target, prepared_fine, prepared_dense, seed, activation_request_key) = {
        let metadata = &identity_class;
        let state = mapping_session_state(adapter.lock().snapshot(), "assigned activation admission");
        if let Some(terminal) = service.activation_receipts.assigned_terminal_before_admission(metadata, state) {
            return Ok(Response::new(terminal));
        }
        let admitted = crate::activation_request::ValidatedActivationRequestV2::admit(
            request, service.activation_receipts.runtime_generation(), selected_backend_kind,
            &expected_map_frame_id, |reference| service.graph_artifacts.lock().open(reference),
            &service.activation_checkpoint_owners,
        ).map_err(Status::invalid_argument)?;
        if service.activation_receipts.has_request(metadata.activation_request_id()) {
            let current = adapter.lock().snapshot();
            if let crate::correction_coordinator::ActivationLedgerAdmission::Terminal(terminal) =
                service.activation_receipts.stage_assigned_or_replay(admitted.clone(),
                    mapping_session_state(current, "assigned activation replay"), false) {
                return Ok(Response::new(*terminal));
            }
        }
        let (metadata, owner) = admitted.into_parts();
        let checkpoint = decode_checkpoint_grid_borrowed(owner.occupancy(), &expected_map_frame_id)
            .map_err(Status::invalid_argument)?;
        let timestamp = owner.occupancy().meta.as_ref().ok_or_else(|| Status::invalid_argument("assigned occupancy metadata missing"))?.timestamp_ns;
        {
            let writer = service.bridge.cloud_map_writer.lock();
            let writer = writer.as_ref().ok_or_else(|| Status::unavailable("cloud-map SHM writer is unavailable"))?;
            writer.validate_checkpoint(owner.cloud().voxel_size(), owner.cloud().point_count(), owner.cloud().xyz_f32_le())
                .map_err(Status::invalid_argument)?;
        }
        let currently_active = { adapter.lock().snapshot().active };
        let (fine, dense) = if currently_active { (None, None) }
            else { prepare_assigned_activation_archives(&service, &owner, &expected_map_frame_id).await? };
        let key = metadata.clone();
        let seed = crate::correction_coordinator::ActivationReceiptSeed {
            planar_target: None, activation_request_id: metadata.activation_request_id().to_owned(),
            request_key: key.clone(), checkpoint_owner: Some(owner.clone()),
        };
        (metadata.session_id().to_owned(), metadata.map_name().to_owned(), checkpoint, timestamp,
         owner.clone(), Some(owner), fine, dense, seed, key)
    };

    let _mapping_transition = tokio::time::timeout(
        MAPPING_TRANSITION_TIMEOUT,
        service.mapping_transition.lock(),
    )
    .await
    .map_err(|_| {
        Status::deadline_exceeded("timed out joining the in-flight Mapping lifecycle transition")
    })?;
    let bridge = service.bridge.clone();
    let mut projection_guard = Some(bridge.lock_explore_projection_owned().await);
    let mut lifecycle_guard = Some(bridge.mapping_lifecycle_lock.clone().lock_owned().await);
    service.ensure_running_admission("ActivateMap").await?;
    if bridge.explore.blocks_mapping_mutation().await {
        let exploration = bridge.explore.snapshot();

        return Err(Status::failed_precondition(format!(
            "exploration safety ownership must be released before ActivateMap: status={} request_id={} child_task_id={}",
            exploration.status, exploration.request_id, exploration.child_task_id
        )));
    }

    let current = adapter.lock().snapshot();
    let allow_new = !(current.active
        || current.phase == crate::mapping_adapter::MappingSessionPhase::Stopped
            && current.session_id == session_id);
    let admission = service.activation_receipts.stage_or_replay(
        seed,
        mapping_session_state(current.clone(), "activation admission"),
        allow_new,
    );
    let mut token = match admission {
        crate::correction_coordinator::ActivationLedgerAdmission::Terminal(state) => {

            return Ok(Response::new(*state));
        }
        crate::correction_coordinator::ActivationLedgerAdmission::Execute(token) => token,
    };

    if current.active {
        let mut projection_guard = projection_guard
            .take()
            .expect("active ActivateMap owns projection guard");
        let lifecycle_guard = lifecycle_guard
            .take()
            .expect("active ActivateMap owns lifecycle guard");
        macro_rules! settle_current_failure {
            ($disposition:expr, $detail:expr) => {{
                let mut detail = $detail;
                let failure_state =
                    mapping_session_state(adapter.lock().snapshot(), detail.clone());
                let retirement = adapter.lock().abort_started_session_detached(
                    &current.session_id,
                    current.session_generation,
                );
                if let Err(error) =
                    bridge.revoke_committed_map_projection_locked(&mut projection_guard)
                {
                    detail = format!("{detail}; projection revoke failed: {error}");
                }
                drop(lifecycle_guard);
                let terminal = settle_failed_activation_retirement(
                    &service,
                    &adapter,
                    &bridge,
                    &token,
                    activation_request_key.clone(),
                    retirement,
                    $disposition,
                    detail,
                    failure_state,
                )
                .await;
                return Ok(Response::new(terminal));
            }};
        }
        if current.session_id != session_id || current.map_name != map_name
            || !adapter.lock().checkpoint_activation_matches(&activation_request_key)
        {
            return Err(Status::failed_precondition("ACTIVATE_MAP_REQUEST_RETIRED"));
        }
        let proof = match service.activation_receipts.installed_target_receipt(&token) {
            Some(proof) => proof,
            None => {
                settle_current_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    "ACTIVATE_MAP_STAGED_PUBLICATION_PROOF_UNAVAILABLE".to_owned()
                );
            }
        };
        if !service.activation_receipts.typed_global_map_published(&token) {
            let committed = {
                let writer = bridge.cloud_map_writer.lock();
                let writer = writer.as_ref().ok_or_else(|| Status::unavailable("cloud-map writer unavailable"))?;
                let metadata = writer.committed_metadata().ok_or_else(|| Status::unavailable("cloud-map metadata unavailable"))?;
                let receipt = writer.last_cloud_receipt().ok_or_else(|| Status::unavailable("cloud-map receipt unavailable"))?;
                if metadata.session_generation != current.session_generation || receipt.session_generation != current.session_generation {
                    return Err(Status::failed_precondition("ACTIVATE_MAP_PUBLICATION_SESSION_RETIRED"));
                }
                CommittedMappingCloudMap {
                    notification: CloudMapNotification { revision: metadata.revision, point_count: metadata.point_count,
                        voxel_size: metadata.voxel_size, session_generation: metadata.session_generation },
                    source_timestamp_ns: receipt.source_timestamp_ns, metadata, receipt,
                }
            };
            publish_committed_global_map_in_frame(&bridge, &committed, &expected_map_frame_id).await
                .map_err(|error| Status::unavailable(format!("typed /global_map publication failed: {error}")))?;
            if !service.activation_receipts.mark_typed_global_map_published(&token) {
                return Err(Status::failed_precondition("ACTIVATE_MAP_PUBLICATION_TOKEN_RETIRED"));
            }
            #[cfg(test)]
            if take_activation_fault_after(&service, ActivationFaultPoint::TypedGlobalMap) {
                settle_current_failure!(ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_TYPED_GLOBAL_MAP_RETRY".to_owned());
            }
        }
        let mut replay = current.clone();
        replay.idempotent_replay = true;
        let state = mapping_session_state(replay, "map checkpoint publication resumed");
        // 当前写端发布成功后，用原安装收据封存；可用性由账本和已选拥有方共同投影。
        let target_receipt = proof.clone();
        let sealed = match service
            .activation_receipts
            .seal(&token, target_receipt, state)
        {
            Some(sealed) => sealed,
            None => {
                settle_current_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "ACTIVATE_MAP_SEAL_CAS_FAILED".to_owned()
                );
            }
        };
        #[cfg(test)]
        if take_activation_fault_after(
            &service,
            ActivationFaultPoint::Seal,
        ) {
            return Err(Status::unavailable("INJECTED_AFTER_SEAL_RESPONSE_LOSS"));
        }
        return Ok(Response::new(sealed));
    }

    macro_rules! return_precommit_failure_with_fence {
        ($disposition:expr, $detail:expr) => {{
            let detail = $detail;
            let state = activation_failure_state(&service, &adapter, &token, $disposition, &detail);
            return Ok(Response::new(state));
        }};
    }

    let staged_worker = match adapter.lock().stage_backend_worker_for_next_session() {
        Ok(worker) => worker,
        Err(error) => {
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                format!("ACTIVATE_MAP_BACKEND_STAGE_FAILED: {error}")
            );
        }
    };
    let reserved_backend_generation = staged_worker.backend_generation();
    token = match service
        .activation_receipts
        .reserve_backend_generation(&token, reserved_backend_generation)
    {
        Some(token) => token,
        None => {
            let _ = staged_worker.stop_and_join();
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_PROVISIONAL_TOKEN_CAS_FAILED"
            );
        }
    };

    let mut restore_resolution = {
        let restore = staged_worker.client().restore_owned_checkpoint(assigned_checkpoint.clone());
        let receiver = match restore {
            Ok(receiver) => receiver,
            Err(error) => {
                let _ = staged_worker.stop_and_join();
                return_precommit_failure_with_fence!(
                    ActivationReceiptDisposition::InvalidatedPrecommit,
                    format!("ACTIVATE_MAP_RESTORE_SUBMISSION_FAILED: {error:?}")
                );
            }
        };
        #[cfg(test)]
        if take_activation_fault_after(
            &service,
            ActivationFaultPoint::BackendRestoreSubmission,
        ) {
            drop(receiver);
            let stop_result = staged_worker.stop_and_join();
            let disposition = if stop_result.is_ok() {
                ActivationReceiptDisposition::InvalidatedPrecommit
            } else {
                ActivationReceiptDisposition::OutcomeUnknown
            };
            return_precommit_failure_with_fence!(
                disposition,
                "INJECTED_AFTER_BACKEND_RESTORE_SUBMISSION"
            );
        }
        drop(
            lifecycle_guard
                .take()
                .expect("ActivateMap restore waits without lifecycle guard"),
        );
        let restore = tokio::time::timeout(MAPPING_BACKEND_CHECKPOINT_TIMEOUT, receiver).await;
        if !service.activation_receipts.validate_token(&token) {
            let _ = staged_worker.stop_and_join();
            let _retired_lifecycle_guard = bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_RESTORE_TOKEN_RETIRED"
            );
        }
        lifecycle_guard = Some(bridge.mapping_lifecycle_lock.clone().lock_owned().await);
        if !service.activation_receipts.validate_token(&token) {
            let _ = staged_worker.stop_and_join();
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_RESTORE_TOKEN_RETIRED_AFTER_RELOCK"
            );
        }
        let resolution = match restore {
            Ok(Ok(Ok(resolution))) => resolution,
            Ok(Ok(Err(error))) => {
                let _ = staged_worker.stop_and_join();
                return_precommit_failure_with_fence!(
                    ActivationReceiptDisposition::InvalidatedPrecommit,
                    format!("ACTIVATE_MAP_RESTORE_REJECTED: {error}")
                );
            }
            Ok(Err(_)) | Err(_) => {
                let _ = staged_worker.stop_and_join();
                return_precommit_failure_with_fence!(
                    ActivationReceiptDisposition::InvalidatedPrecommit,
                    "ACTIVATE_MAP_RESTORE_TIMEOUT_OR_CLOSED"
                );
            }
        };

        Some(resolution)
    };

    let started = match adapter.lock().start_session_with_staged_backend(
        &session_id,
        &map_name,
        staged_worker,
    ) {
        Ok(started) => started,
        Err(error) => {
            if let Some(resolution) = restore_resolution.take() {
                let _ = resolution.abort();
            }
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                format!("ACTIVATE_MAP_SESSION_START_FAILED: {error}")
            );
        }
    };
    // activate_map_r1 runs in the existing retained task with L still held.
    // Reset only after a genuinely new session starts; replay returned above.
    {
        let mut shared = bridge.state.write().await;
        crate::lcm_runtime::reset_canonical_localization_locked(&mut shared, None);
    }
    macro_rules! settle_started_failure {
        ($disposition:expr, $detail:expr) => {{
            let mut detail = $detail;
            let failure_state = mapping_session_state(adapter.lock().snapshot(), detail.clone());
            let retirement = adapter
                .lock()
                .abort_started_session_detached(&started.session_id, started.session_generation);
            if let Err(error) = bridge.revoke_committed_map_projection_locked(
                projection_guard
                    .as_mut()
                    .expect("active ActivateMap settlement owns projection guard"),
            ) {
                detail = format!("{detail}; projection revoke failed: {error}");
            }
            drop(
                lifecycle_guard
                    .take()
                    .expect("active ActivateMap settlement owns lifecycle guard"),
            );
            let terminal = settle_failed_activation_retirement(
                &service,
                &adapter,
                &bridge,
                &token,
                activation_request_key.clone(),
                retirement,
                $disposition,
                detail,
                failure_state,
            )
            .await;
            return Ok(Response::new(terminal));
        }};
    }
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::SessionStart,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_SESSION_START".to_owned()
        );
    }
    let final_activation_identity = InstalledActivationIdentityV2 {
        request: identity_class.clone(), runtime_generation: service.activation_receipts.runtime_generation(),
        lifecycle_epoch: service.activation_receipts.lifecycle_epoch(), session_generation: started.session_generation,
        backend_kind: selected_backend_kind, backend_generation: reserved_backend_generation,
    };
    token = match service.activation_receipts.bind_activation_identity(
        &token,
        final_activation_identity.clone(),
        started.session_generation,
    ) {
        Some(token) => token,
        None => {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_FINAL_TOKEN_BIND_FAILED".to_owned()
            );
        }
    };


    if let Some(writer) = &bridge.occupancy_shm {
        let begin_result = { writer.lock().begin_session(started.session_generation) };
        if let Err(error) = begin_result {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                format!("ACTIVATE_MAP_OCCUPANCY_FENCE_FAILED: {error}")
            );
        }
        #[cfg(test)]
        if take_activation_fault_after(
            &service,
            ActivationFaultPoint::OccupancyShmBegin,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "INJECTED_AFTER_OCCUPANCY_SHM_BEGIN".to_owned()
            );
        }

    }

    let revision_floor = bridge.state.read().await.map.revision;
    let install_result = {
        let mut mapping = adapter.lock();
        mapping.checkpoint_localization_required = immutable_target.is_some();
        mapping.install_checkpoint(checkpoint, checkpoint_timestamp_ns, revision_floor)
    };
    let (map, event) = match install_result {
        Ok(result) => result,
        Err(error) => {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                format!("ACTIVATE_MAP_OCCUPANCY_INSTALL_FAILED: {error}")
            );
        }
    };
    let latest_occupancy_receipt = { adapter.lock().latest_occupancy_commit_receipt() };
    let occupancy_receipt = match latest_occupancy_receipt {
        Some(receipt)
            if receipt.occupancy_revision == event.revision
                && receipt.session_generation == started.session_generation
                && receipt.backend_generation == reserved_backend_generation =>
        {
            receipt
        }
        _ => {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_OCCUPANCY_RECEIPT_MISMATCH".to_owned()
            );
        }
    };
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::OccupancyInstall,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_OCCUPANCY_INSTALL".to_owned()
        );
    }


    let source_timestamp_ns = map.timestamp_ns;
    let cloud_commit = if let Some(target) = immutable_target.as_ref() {
        let fine_frame_generation = adapter.lock().global_frame_generation();
        let installed = {
            let mut writer = bridge.cloud_map_writer.lock();
            match writer.as_mut() {
                Some(writer) => writer.bind_fine_map_owner(
                    fine_frame_generation,
                    bridge.nav.producer_runtime_generation(),
                ).and_then(|()| writer.install_checkpoint_with_sources(
                    &occupancy_receipt,
                    target.voxel_size(),
                    target.point_count(),
                    target.xyz_f32_le(),
                    prepared_fine,
                    prepared_dense,
                )),
                None => Err("cloud-map SHM writer became unavailable".to_owned()),
            }
        };
        let correlated = match installed {
            Ok(correlated) => correlated,
            Err(error) => {
                settle_started_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    format!("ACTIVATE_MAP_CLOUD_INSTALL_OUTCOME_UNKNOWN: {error}")
                );
            }
        };
        if correlated.snapshot.session_generation != started.session_generation
            || correlated.snapshot.point_count != target.point_count()
            || correlated.snapshot.voxel_size.to_bits() != target.voxel_size().to_bits()
            || correlated.receipt.session_generation != started.session_generation
            || correlated.receipt.point_count != target.point_count()
        {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_CLOUD_EFFECT_UNVERIFIABLE".to_owned()
            );
        }

        #[cfg(test)]
        if take_activation_fault_after(
            &service,
            ActivationFaultPoint::CloudInstall,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "INJECTED_AFTER_CLOUD_INSTALL".to_owned()
            );
        }
        let target_receipt = InstalledTargetReceiptV2 { activation: final_activation_identity.clone(),
            source_member: assigned_checkpoint.cloud().member().clone(), occupancy_receipt: occupancy_receipt.clone(),
            cloud_receipt: Some(correlated.receipt.clone()) };
        let proof = target_receipt.clone();
        if !service
            .activation_receipts
            .store_installed_target_receipt(&token, proof)
        {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_TARGET_RECEIPT_STORE_CAS_FAILED".to_owned()
            );
        }
        let notification = CloudMapNotification {
            revision: correlated.snapshot.revision,
            point_count: target.point_count(),
            voxel_size: target.voxel_size(),
            session_generation: started.session_generation,
        };
        Some((
            notification.clone(),
            CommittedMappingCloudMap {
                notification,
                source_timestamp_ns,
                metadata: correlated.snapshot,
                receipt: correlated.receipt,
            },
            target_receipt,
        ))
    } else {
        None
    };

    if let Some(resolution) = restore_resolution.take() {
        if resolution.commit().is_err() {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_BACKEND_COMMIT_OUTCOME_UNKNOWN".to_owned()
            );
        }
        #[cfg(test)]
        if take_activation_fault_after(
            &service,
            ActivationFaultPoint::BackendCommit,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "INJECTED_AFTER_BACKEND_COMMIT".to_owned()
            );
        }

    }

    let activation_record_result = {
        let mut adapter = adapter.lock();
        adapter.record_checkpoint_activation(activation_request_key.clone())
    };
    if let Err(error) = activation_record_result {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            format!("ACTIVATE_MAP_RECORD_FAILED: {error}")
        );
    }
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::ActivationRecord,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_ACTIVATION_RECORD".to_owned()
        );
    }


    {
        let mut shared = bridge.state.write().await;
        shared.map = map;
        if let Some((notification, _, _)) = cloud_commit.as_ref() {
            shared.cloud_map_revision = notification.revision;
        }
    }
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::SharedStateCommit,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_SHARED_STATE_COMMIT".to_owned()
        );
    }

    bridge
        .revoke_committed_map_projection_locked(
            projection_guard
                .as_mut()
                .expect("ActivateMap owns projection guard through cutover"),
        )
        .map_err(Status::unavailable)?;
    *bridge.occupancy_shm_current.write().await = None;
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::CacheClears,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_CACHE_CLEARS".to_owned()
        );
    }

    let _ = bridge.map_revision_tx.send(event);
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::MapEvent,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_MAP_EVENT".to_owned()
        );
    }


    let target_receipt =
        if let Some((notification, committed, target_receipt)) = cloud_commit {
            let _ = bridge.cloud_map_tx.send(notification);
            #[cfg(test)]
        if take_activation_fault_after(
                &service,
                ActivationFaultPoint::CloudEvent,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_CLOUD_EVENT".to_owned()
                );
            }

            publish_committed_global_map_in_frame(&bridge, &committed, &expected_map_frame_id).await
                .map_err(|error| Status::unavailable(format!("typed /global_map publication failed: {error}")))?;
            if !service.activation_receipts.mark_typed_global_map_published(&token) {
                return Err(Status::failed_precondition("ACTIVATE_MAP_PUBLICATION_TOKEN_RETIRED"));
            }
            #[cfg(test)]
        if take_activation_fault_after(
                &service,
                ActivationFaultPoint::TypedGlobalMap,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_TYPED_GLOBAL_MAP".to_owned()
                );
            }

            target_receipt
        } else { unreachable!("编号三维检查点具有点云提交") };


    let snapshot = adapter.lock().snapshot();
    let activation_message = "map checkpoint activated";
    let state = mapping_session_state(snapshot, activation_message);
    let sealed = match service
        .activation_receipts
        .seal(&token, target_receipt, state)
    {
        Some(sealed) => sealed,
        None => {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPostcommit,
                "ACTIVATE_MAP_SEAL_CAS_FAILED".to_owned()
            );
        }
    };
    #[cfg(test)]
        if take_activation_fault_after(
        &service,
        ActivationFaultPoint::Seal,
    ) {
        drop(lifecycle_guard);
        return Err(Status::unavailable("INJECTED_AFTER_SEAL_RESPONSE_LOSS"));
    }
    drop(lifecycle_guard);
    Ok(Response::new(sealed))
}

fn mapping_lifecycle_status(error: crate::mapping_adapter::MappingLifecycleError) -> Status {
    use crate::mapping_adapter::MappingLifecycleError;
    match error {
        MappingLifecycleError::InvalidArgument(message) => Status::invalid_argument(message),
        MappingLifecycleError::Conflict(message) => Status::already_exists(message),
        MappingLifecycleError::StaleSession(message)
        | MappingLifecycleError::TerminalSession(message) => Status::failed_precondition(message),
        MappingLifecycleError::GenerationExhausted => {
            Status::resource_exhausted("mapping session generation exhausted")
        }
        MappingLifecycleError::Backend(message) => Status::unavailable(message),
    }
}

fn validate_deferred_cloud_map_clear(
    admission: &CloudMapSessionAdmission,
    expected_session_generation: u64,
) -> Result<(), String> {
    let notification = match admission {
        CloudMapSessionAdmission::Registered(notification) => notification,
        CloudMapSessionAdmission::WriterUnavailable => {
            return Err(format!(
                "cloud-map writer is unavailable for deferred Mapping generation {expected_session_generation}"
            ));
        }
        CloudMapSessionAdmission::ClearRejected(detail) => {
            return Err(format!("cloud-map clear was rejected: {detail}"));
        }
        CloudMapSessionAdmission::AdmissionClosed(detail) => {
            return Err(format!("cloud-map task admission is closed: {detail}"));
        }
    };
    if expected_session_generation == 0
        || notification.session_generation != expected_session_generation
        || notification.revision == 0
        || notification.point_count != 0
        || !notification.voxel_size.is_finite()
        || notification.voxel_size <= 0.0
        || notification.voxel_size.to_bits() != crate::pointcloud::VOXEL_SIZE.to_bits()
    {
        return Err(format!(
            "invalid deferred cloud-map clear identity: expected_generation={expected_session_generation} actual_generation={} revision={} point_count={} voxel_size={}",
            notification.session_generation,
            notification.revision,
            notification.point_count,
            notification.voxel_size,
        ));
    }
    Ok(())
}

struct OccupancySessionFenceRollback {
    writer: Arc<parking_lot::Mutex<OccupancyGridShmWriter>>,
    transaction: OccupancySessionFenceTransaction,
}

struct FailedMappingStartRollback {
    retirement: Option<MappingBackendRetirement>,
    errors: Vec<String>,
}

struct FailedMappingStartOutcome {
    canonical_detail: String,
    diagnostics: Vec<String>,
}

impl FailedMappingStartOutcome {
    fn detail_with_diagnostics(mut self) -> String {
        for diagnostic in self.diagnostics {
            self.canonical_detail.push_str("; ");
            self.canonical_detail.push_str(&diagnostic);
        }
        self.canonical_detail
    }
}

fn rollback_failed_mapping_start_locked(
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    started: &MappingSessionSnapshot,
    occupancy_fence: Option<OccupancySessionFenceRollback>,
) -> FailedMappingStartRollback {
    let mut errors = Vec::new();
    if let Some(occupancy_fence) = occupancy_fence {
        if let Err(error) = occupancy_fence
            .writer
            .lock()
            .rollback_session_fence(occupancy_fence.transaction)
        {
            errors.push(format!("occupancy session-fence rollback failed: {error}"));
        }
    }

    let stop_result = {
        let mut adapter = adapter.lock();
        adapter.stop_session_detached(&started.session_id)
    };
    let transition = match stop_result {
        Ok(transition) => Some(transition),
        Err(error) => {
            errors.push(format!("ordinary Mapping stop transition failed: {error}"));
            match adapter.lock().force_stop_started_session_detached(
                &started.session_id,
                started.session_generation,
            ) {
                Ok(transition) => Some(transition),
                Err(fallback_error) => {
                    errors.push(format!(
                        "terminal Mapping rollback fallback failed: {fallback_error}"
                    ));
                    None
                }
            }
        }
    };
    let retirement = transition.and_then(|transition| {
        if transition.snapshot.active {
            errors.push("Mapping rollback retained an active session".to_owned());
        }
        transition.retirement
    });
    FailedMappingStartRollback { retirement, errors }
}

async fn finish_failed_mapping_start_rollback(
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
    started: &MappingSessionSnapshot,
    occupancy_fence: Option<OccupancySessionFenceRollback>,
    canonical_detail: String,
) -> FailedMappingStartOutcome {
    let rollback = rollback_failed_mapping_start_locked(adapter, started, occupancy_fence);
    drop(lifecycle_guard);
    let mut diagnostics = rollback.errors;
    if let Err(error) =
        finish_backend_retirement(adapter, lifecycle_lock, rollback.retirement).await
    {
        diagnostics.push(format!("Mapping backend retirement failed: {error}"));
    }
    FailedMappingStartOutcome {
        canonical_detail,
        diagnostics,
    }
}


fn decode_checkpoint_grid_borrowed(
    checkpoint: &OccupancyGridData,
    expected_map_frame_id: &str,
) -> Result<nuwax_mapping::OccupancyGrid, String> {
    let meta = checkpoint.meta.as_ref().ok_or_else(|| "checkpoint.meta is required".to_string())?;
    decode_checkpoint_grid_parts(meta, checkpoint.data.len(), checkpoint.data.iter().copied(), expected_map_frame_id)
}

fn decode_checkpoint_grid_parts(
    meta: &OccupancyGridMeta,
    actual_size: usize,
    values: impl Iterator<Item = u8>,
    expected_map_frame_id: &str,
) -> Result<nuwax_mapping::OccupancyGrid, String> {
    if meta.width == 0 || meta.height == 0 {
        return Err("checkpoint width/height must be positive".to_string());
    }
    let expected_size = (meta.width as usize)
        .checked_mul(meta.height as usize)
        .ok_or_else(|| "checkpoint dimensions overflow".to_string())?;
    if actual_size != expected_size {
        return Err(format!(
            "checkpoint data size mismatch: expected={} actual={}",
            expected_size,
            actual_size
        ));
    }
    if !meta.resolution.is_finite() || meta.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_string());
    }
    let frame_id = meta.frame_id.trim();
    if frame_id != expected_map_frame_id {
        return Err(format!(
            "checkpoint frame must be {expected_map_frame_id}, received {}",
            meta.frame_id,
        ));
    }
    let data = values.map(|value| value as i8)
        .collect::<Vec<_>>();
    if data.iter().any(|value| !(-1..=100).contains(value)) {
        return Err("checkpoint occupancy values must be within [-1, 100]".to_string());
    }
    let mut grid = nuwax_mapping::OccupancyGrid::with_frame(
        meta.width,
        meta.height,
        f64::from(meta.resolution),
        frame_id,
    );
    grid.origin = [meta.origin_x, meta.origin_y, 0.0];
    grid.ts = meta.timestamp_ns as f64 / 1_000_000_000.0;
    grid.data = data;
    Ok(grid)
}

enum OccupancyGridValidationError {
    NotReady,
    InvalidMetadata(&'static str),
    DimensionsOverflow,
    PayloadSizeMismatch { expected: usize, actual: usize },
}

impl OccupancyGridValidationError {
    fn into_status(self) -> Status {
        match self {
            Self::NotReady => Status::failed_precondition("occupancy grid is not ready"),
            Self::InvalidMetadata(reason) => {
                Status::internal(format!("occupancy grid metadata is invalid: {reason}"))
            }
            Self::DimensionsOverflow => Status::internal("occupancy grid dimensions overflow"),
            Self::PayloadSizeMismatch { expected, actual } => Status::internal(format!(
                "occupancy grid payload size mismatch: expected={expected} actual={actual}"
            )),
        }
    }
}

fn validate_occupancy_grid(
    map: &crate::lcm_runtime::MapState,
) -> Result<(), OccupancyGridValidationError> {
    if map.revision == 0 {
        return Err(OccupancyGridValidationError::NotReady);
    }
    if map.width == 0 || map.height == 0 {
        return Err(OccupancyGridValidationError::InvalidMetadata(
            "dimensions must be nonzero",
        ));
    }
    if !map.resolution.is_finite() || map.resolution <= 0.0 {
        return Err(OccupancyGridValidationError::InvalidMetadata(
            "resolution must be finite and positive",
        ));
    }
    if map.frame_id.trim().is_empty() {
        return Err(OccupancyGridValidationError::InvalidMetadata(
            "frame_id must not be empty",
        ));
    }

    let expected_len = (map.width as usize)
        .checked_mul(map.height as usize)
        .ok_or(OccupancyGridValidationError::DimensionsOverflow)?;
    if map.data.len() != expected_len {
        return Err(OccupancyGridValidationError::PayloadSizeMismatch {
            expected: expected_len,
            actual: map.data.len(),
        });
    }
    Ok(())
}

fn occupancy_grid_meta(map: &crate::lcm_runtime::MapState) -> OccupancyGridMeta {
    OccupancyGridMeta {
        width: map.width,
        height: map.height,
        resolution: map.resolution,
        origin_x: map.origin_x,
        origin_y: map.origin_y,
        frame_id: map.frame_id.clone(),
        revision: map.revision,
        timestamp_ns: map.timestamp_ns,
    }
}

fn occupancy_checkpoint_proto(map: &crate::lcm_runtime::MapState) -> OccupancyGridData {
    OccupancyGridData {
        meta: Some(occupancy_grid_meta(map)),
        data: map.data.iter().map(|&value| value as u8).collect(),
    }
}

fn owned_occupancy_bytes(data: Vec<i8>) -> Vec<u8> {
    data.into_iter().map(|value| value as u8).collect()
}

fn cloud_checkpoint_proto(
    snapshot: crate::pointcloud::CommittedCloudMapSnapshot,
    frame_id: String,
) -> CloudMapCheckpoint {
    CloudMapCheckpoint {
        frame_id,
        source_revision: snapshot.revision,
        point_count: snapshot.point_count,
        voxel_size: snapshot.voxel_size,

        xyz_f32_le: snapshot.xyz_f32_le,
        payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
        member_identity: None,
    }
}

pub(crate) fn validate_mapping_checkpoint_envelope_size(
    label: &str,
    encoded_len: usize,
) -> Result<(), String> {
    if encoded_len > MAPPING_CHECKPOINT_PROTO_ENVELOPE_MAX_BYTES {
        return Err(format!(
            "{label} protobuf envelope exceeds {} bytes: {encoded_len}",
            MAPPING_CHECKPOINT_PROTO_ENVELOPE_MAX_BYTES
        ));
    }
    Ok(())
}


fn mapping_commit_identity_proto(
    identity: crate::mapping_commit::MappingCommitId,
) -> MappingCommitIdentity {
    MappingCommitIdentity {
        boot_nonce: identity.boot_nonce.to_be_bytes().to_vec(),
        sequence: identity.sequence,
    }
}

fn occupancy_commit_receipt_proto(
    receipt: &crate::mapping_commit::OccupancyCommitReceipt,
) -> CanonicalOccupancyCommitReceipt {
    CanonicalOccupancyCommitReceipt {
        commit_id: Some(mapping_commit_identity_proto(receipt.commit_id)),
        session_id: receipt.session_id.clone(),
        session_generation: receipt.session_generation,
        occupancy_revision: receipt.occupancy_revision,
        source_timestamp_ns: receipt.source_timestamp_ns,
        backend_kind: receipt.backend_kind.stable_name().to_owned(),
        backend_generation: receipt.backend_generation,
    }
}

fn cloud_commit_receipt_proto(
    receipt: &crate::mapping_commit::CloudCommitReceipt,
) -> CanonicalCloudCommitReceipt {
    CanonicalCloudCommitReceipt {
        commit_id: Some(mapping_commit_identity_proto(receipt.commit_id)),
        session_generation: receipt.session_generation,
        occupancy_revision: receipt.occupancy_revision,
        cloud_revision: receipt.cloud_revision,
        source_timestamp_ns: receipt.source_timestamp_ns,
        point_count: receipt.point_count,
    }
}

pub(crate) fn validate_mapping_commit_identity<'a>(
    identity: Option<&'a MappingCommitIdentity>,
    label: &str,
) -> Result<&'a MappingCommitIdentity, String> {
    let identity = identity.ok_or_else(|| format!("{label} commit_id is required"))?;
    if identity.boot_nonce.len() != 16 || identity.boot_nonce.iter().all(|value| *value == 0) {
        return Err(format!(
            "{label} commit boot_nonce must contain 16 bytes and must not be all zero"
        ));
    }
    if identity.sequence == 0 {
        return Err(format!("{label} commit sequence must be positive"));
    }
    Ok(identity)
}

pub(crate) fn validate_mapping_checkpoint_receipts(
    source_session_id: &str,
    source_session_generation: u64,
    backend_kind: &str,
    occupancy_receipt: Option<&CanonicalOccupancyCommitReceipt>,
    cloud_receipt: Option<&CanonicalCloudCommitReceipt>,
) -> Result<(), String> {
    let source_session_id = source_session_id.trim();
    if source_session_id.is_empty() || source_session_generation == 0 {
        return Err(
            "mapping backend checkpoint source session id/generation must be nonempty and positive"
                .to_owned(),
        );
    }
    let occupancy_receipt = occupancy_receipt
        .ok_or_else(|| "mapping backend checkpoint occupancy_receipt is required".to_owned())?;
    let cloud_receipt = cloud_receipt
        .ok_or_else(|| "mapping backend checkpoint cloud_receipt is required".to_owned())?;
    let occupancy_id = validate_mapping_commit_identity(
        occupancy_receipt.commit_id.as_ref(),
        "mapping occupancy receipt",
    )?;
    let cloud_id = validate_mapping_commit_identity(
        cloud_receipt.commit_id.as_ref(),
        "mapping cloud receipt",
    )?;

    if occupancy_receipt.session_id.trim().is_empty()
        || occupancy_receipt.session_id != source_session_id
        || occupancy_receipt.session_generation != source_session_generation
        || occupancy_receipt.occupancy_revision == 0
        || occupancy_receipt.backend_kind != backend_kind
        || occupancy_receipt.backend_generation == 0
    {
        return Err(
            "mapping backend checkpoint occupancy receipt identity is inconsistent".to_owned(),
        );
    }
    if cloud_receipt.session_generation != source_session_generation
        || cloud_receipt.occupancy_revision == 0
        || cloud_receipt.cloud_revision == 0
        || cloud_receipt.point_count == 0
    {
        return Err("mapping backend checkpoint cloud receipt identity is inconsistent".to_owned());
    }
    if cloud_id.boot_nonce != occupancy_id.boot_nonce {
        return Err("mapping backend checkpoint receipt boot nonces differ".to_owned());
    }
    if cloud_id.sequence > occupancy_id.sequence
        || cloud_receipt.occupancy_revision > occupancy_receipt.occupancy_revision
    {
        return Err("mapping backend checkpoint cloud receipt leads occupancy receipt".to_owned());
    }
    let same_sequence = cloud_id.sequence == occupancy_id.sequence;
    let same_occupancy_revision =
        cloud_receipt.occupancy_revision == occupancy_receipt.occupancy_revision;
    if same_sequence != same_occupancy_revision {
        return Err(
            "mapping backend checkpoint receipt sequence/revision ordering is inconsistent"
                .to_owned(),
        );
    }
    if same_sequence && cloud_receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns {
        return Err(
            "mapping backend checkpoint receipts with one commit id have different timestamps"
                .to_owned(),
        );
    }
    Ok(())
}

fn validate_planar_checkpoint(checkpoint: &MappingBackendCheckpoint) -> Result<(), String> {
    use crate::planar_artifacts::Store;
    let reference = checkpoint.graph_artifact.as_ref().ok_or("二维图必须通过分块制品引用加载")?;
    if checkpoint.backend_kind != "slam_toolbox" || checkpoint.target_kind != "planar_graph"
        || checkpoint.created_at_ns == 0 || checkpoint.source_session_id.is_empty() || checkpoint.source_session_generation == 0
        || !checkpoint.payload.is_empty() || checkpoint.cloud_receipt.is_some() || checkpoint.canonical_cloud_checkpoint.is_some()
        || checkpoint.backend_identity.is_some() || checkpoint.fine_map_artifact.is_some() || checkpoint.dense_cloud_artifact.is_some()
        || reference.size != checkpoint.payload_size || reference.size > nuwax_slam_2d_runtime::archive::MAX_ARCHIVE {
        return Err("二维完整图检查点类型、身份或预算非法".into());
    }
    Store::validate_reference(reference)?;
    let receipt = checkpoint.occupancy_receipt.as_ref().ok_or("二维图缺少规范占据收据")?;
    crate::activation_identity::occupancy_receipt_from_wire(receipt)?;
    let grid = checkpoint.canonical_occupancy_checkpoint.as_ref().ok_or("二维图缺少物理栅格")?;
    let meta = grid.meta.as_ref().ok_or("二维栅格缺少几何")?;
    if meta.width == 0 || meta.height == 0 || !meta.resolution.is_finite() || meta.resolution <= 0.0
        || !meta.origin_x.is_finite() || !meta.origin_y.is_finite() || meta.frame_id.is_empty()
        || grid.data.len() != (meta.width as usize).checked_mul(meta.height as usize).ok_or("occupancy dimensions overflow")?
        || grid.data.iter().any(|v| !(-1..=100).contains(&(*v as i8))) {
        return Err("二维物理栅格几何或正文非法".into());
    }
    if receipt.session_id != checkpoint.source_session_id || receipt.session_generation != checkpoint.source_session_generation
        || receipt.backend_kind != "slam_toolbox" || receipt.occupancy_revision != meta.revision
        || receipt.source_timestamp_ns != meta.timestamp_ns || meta.timestamp_ns == 0 || meta.revision == 0 {
        return Err("二维图物理栅格与规范收据身份冲突".into());
    }
    if checkpoint.schema_version != 2 || !crate::planar_artifacts::assigned_id(&checkpoint.checkpoint_id)
        || checkpoint.backend_payload_schema_version != 2 || reference.identity_schema_version != 2 {
        return Err("二维检查点身份代际非法".into());
    }
    validate_checkpoint_member_binding(checkpoint, reference.member_identity.as_ref().ok_or("二维图成员身份缺失")?, CheckpointMemberRole::PlanarGraph)?;
    validate_checkpoint_member_binding(checkpoint, checkpoint.occupancy_identity.as_ref().ok_or("二维占据成员身份缺失")?, CheckpointMemberRole::CanonicalOccupancy)?;
    Ok(())
}

fn validate_mapping_checkpoint_structure(checkpoint: &MappingBackendCheckpoint,
    expected_backend: crate::mapping_backend::MappingBackendKind) -> Result<(), String> {
    if checkpoint.backend_kind != expected_backend.stable_name() {
        return Err(format!(
            "mapping backend checkpoint kind mismatch: checkpoint={} selected={}",
            checkpoint.backend_kind,
            expected_backend.stable_name()
        ));
    }
    validate_mapping_checkpoint_receipts(
        &checkpoint.source_session_id,
        checkpoint.source_session_generation,
        &checkpoint.backend_kind,
        checkpoint.occupancy_receipt.as_ref(),
        checkpoint.cloud_receipt.as_ref(),
    )?;
    if checkpoint.created_at_ns == 0 {
        return Err("mapping backend checkpoint creation time must be positive".to_owned());
    }
    if checkpoint.payload.is_empty()
        || checkpoint.payload.len() > crate::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
    {
        return Err(format!(
            "mapping backend checkpoint payload exceeds 1..={} bytes",
            crate::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
        ));
    }
    let actual_size = u64::try_from(checkpoint.payload.len())
        .map_err(|_| "mapping backend checkpoint payload size exceeds u64".to_owned())?;
    if checkpoint.payload_size != actual_size {
        return Err(format!(
            "mapping backend checkpoint payload_size mismatch: declared={} actual={actual_size}",
            checkpoint.payload_size
        ));
    }
    Ok(())
}

// The outer identity generation is independent of the NWMBCKP binary payload schema.
const ASSIGNED_CHECKPOINT_ENVELOPE_VERSION: u32 = 2;

fn checkpoint_member_identity(checkpoint_id: &str, role: CheckpointMemberRole,
    occupancy: &CanonicalOccupancyCommitReceipt, cloud: &CanonicalCloudCommitReceipt,
    frame_id: &str, fine_domain: Option<&nuwax_map_chunks::Domain>) -> CheckpointMemberIdentity {
    CheckpointMemberIdentity { checkpoint_id: checkpoint_id.to_owned(), role: role as i32,
        occupancy_receipt: Some(occupancy.clone()), cloud_receipt: Some(cloud.clone()), frame_id: frame_id.to_owned(),
        frame_generation: fine_domain.map_or(0, |d| d.frame_generation), producer_generation: fine_domain.map_or(0, |d| d.producer_generation) }
}
fn validate_cloud_checkpoint_structure(checkpoint: &CloudMapCheckpoint, expected_frame: &str) -> Result<(), String> {
    if expected_frame.is_empty() || checkpoint.frame_id != expected_frame || checkpoint.frame_id.len() > 1024
        || checkpoint.frame_id.trim() != checkpoint.frame_id || checkpoint.frame_id.chars().any(char::is_control)
        || checkpoint.source_revision == 0 || checkpoint.point_count == 0
        || !checkpoint.voxel_size.is_finite() || checkpoint.voxel_size <= 0.0
        || checkpoint.xyz_f32_le.len() != (checkpoint.point_count as usize).checked_mul(12).ok_or("cloud checkpoint size overflow")?
        || checkpoint.xyz_f32_le.chunks_exact(4).any(|v| !f32::from_le_bytes(v.try_into().unwrap()).is_finite()) {
        return Err("cloud checkpoint geometry, size or frame is invalid".into());
    }
    Ok(())
}
fn validate_checkpoint_canonical_structure(checkpoint: &MappingBackendCheckpoint) -> Result<(), String> {
    let occupancy = checkpoint.canonical_occupancy_checkpoint.as_ref().ok_or("canonical occupancy required")?;
    let meta = occupancy.meta.as_ref().ok_or("canonical occupancy metadata required")?;
    if meta.width == 0 || meta.height == 0 || meta.revision == 0 || meta.timestamp_ns == 0
        || !meta.resolution.is_finite() || meta.resolution <= 0.0 || !meta.origin_x.is_finite() || !meta.origin_y.is_finite()
        || occupancy.data.len() != (meta.width as usize).checked_mul(meta.height as usize).ok_or("occupancy dimensions overflow")?
        || occupancy.data.iter().any(|v| !(-1..=100).contains(&(*v as i8))) { return Err("canonical occupancy geometry, size or cells are invalid".into()); }
    let cloud = checkpoint.canonical_cloud_checkpoint.as_ref().ok_or("canonical cloud required")?;
    validate_cloud_checkpoint_structure(cloud, &meta.frame_id)?;
    validate_mapping_checkpoint_bundle(checkpoint)
}
fn validate_checkpoint_member_binding(checkpoint: &MappingBackendCheckpoint, member: &CheckpointMemberIdentity, role: CheckpointMemberRole) -> Result<(), String> {
    crate::planar_artifacts::validate_member_identity(member)?;
    let frame = &checkpoint.canonical_occupancy_checkpoint.as_ref().and_then(|grid| grid.meta.as_ref()).ok_or("canonical occupancy metadata required")?.frame_id;
    if member.checkpoint_id != checkpoint.checkpoint_id || member.role != role as i32
        || member.occupancy_receipt != checkpoint.occupancy_receipt || member.cloud_receipt != checkpoint.cloud_receipt || member.frame_id != *frame {
        return Err("checkpoint member descriptor differs from its frozen parent".into());
    }
    Ok(())
}
pub(crate) fn validate_assigned_mapping_checkpoint(checkpoint: &MappingBackendCheckpoint, expected_backend: crate::mapping_backend::MappingBackendKind) -> Result<(), String> {
    if expected_backend == crate::mapping_backend::MappingBackendKind::SlamToolbox {
        if checkpoint.schema_version != 2 { return Err("assigned planar checkpoint requires schema 2".into()); }
        return validate_planar_checkpoint(checkpoint);
    }
    if checkpoint.schema_version != ASSIGNED_CHECKPOINT_ENVELOPE_VERSION
        || !crate::planar_artifacts::assigned_id(&checkpoint.checkpoint_id)
        || checkpoint.backend_payload_schema_version != crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION
        || checkpoint.graph_artifact.is_some() || !checkpoint.target_kind.is_empty()
        || expected_backend == crate::mapping_backend::MappingBackendKind::SlamToolbox { return Err("unsupported or mixed assigned checkpoint envelope".into()); }
    validate_mapping_checkpoint_structure(checkpoint, expected_backend)?;
    validate_checkpoint_canonical_structure(checkpoint)?;
    let payload = &checkpoint.payload;
    let kind = if expected_backend == crate::mapping_backend::MappingBackendKind::Voxel { 0 } else { 1 };
    if payload.len() < 16 || &payload[..8] != b"NWMBCKP\0" || payload[8..12] != 1_u32.to_le_bytes() || payload[12] != kind || payload[13..16] != [0; 3] {
        return Err("backend payload format/schema differs from its envelope".into());
    }
    validate_checkpoint_member_binding(checkpoint, checkpoint.backend_identity.as_ref().ok_or("backend member identity required")?, CheckpointMemberRole::BackendState)?;
    validate_checkpoint_member_binding(checkpoint, checkpoint.occupancy_identity.as_ref().ok_or("occupancy member identity required")?, CheckpointMemberRole::CanonicalOccupancy)?;
    let cloud = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
    if cloud.payload_schema_version != 2 { return Err("mixed canonical cloud identity generation".into()); }
    validate_checkpoint_member_binding(checkpoint, cloud.member_identity.as_ref().ok_or("cloud member identity required")?, CheckpointMemberRole::CanonicalCloud)?;
    if let (Some(fine), Some(dense)) = (&checkpoint.fine_map_artifact, &checkpoint.dense_cloud_artifact) {
        if fine.id == dense.id { return Err("distinct checkpoint members cannot share one artifact ID".into()); }
    }
    for (reference, role) in [(&checkpoint.fine_map_artifact, CheckpointMemberRole::FineMap), (&checkpoint.dense_cloud_artifact, CheckpointMemberRole::DenseCloud)] {
        if let Some(reference) = reference {
            crate::planar_artifacts::Store::validate_reference(reference)?;
            if reference.identity_schema_version != 2 { return Err("legacy artifact inside assigned checkpoint".into()); }
            validate_checkpoint_member_binding(checkpoint, reference.member_identity.as_ref().unwrap(), role)?;
        }
    }
    Ok(())
}
/// One assembly owner after the existing lifecycle barrier has frozen all inputs.
fn assemble_mapping_checkpoint_export(mut checkpoint: MappingBackendCheckpoint) -> Result<MappingBackendCheckpoint, String> {
    checkpoint.schema_version = 2;
    checkpoint.backend_payload_schema_version = crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION;
    let occupancy = checkpoint.occupancy_receipt.as_ref().expect("导出屏障冻结占据提交");
    let cloud_receipt = checkpoint.cloud_receipt.as_ref().expect("导出屏障冻结点云提交");
    let cloud = checkpoint.canonical_cloud_checkpoint.as_mut().expect("导出拥有规范点云");
    checkpoint.backend_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::BackendState, occupancy, cloud_receipt, &cloud.frame_id, None));
    checkpoint.occupancy_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::CanonicalOccupancy, occupancy, cloud_receipt, &cloud.frame_id, None));
    cloud.payload_schema_version = 2;
    cloud.member_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::CanonicalCloud, occupancy, cloud_receipt, &cloud.frame_id, None));
    validate_mapping_checkpoint_envelope_size("mapping backend checkpoint response", checkpoint.encoded_len())?;
    Ok(checkpoint)
}


fn validate_mapping_checkpoint_bundle(
    checkpoint: &MappingBackendCheckpoint,
) -> Result<(), String> {
    let occupancy_receipt = checkpoint
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint occupancy_receipt is required".to_owned())?;
    let cloud_receipt = checkpoint
        .cloud_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint cloud_receipt is required".to_owned())?;
    let occupancy_checkpoint=checkpoint.canonical_occupancy_checkpoint.as_ref()
        .ok_or("mapping backend checkpoint canonical_occupancy_checkpoint is required")?;
    let occupancy_meta = occupancy_checkpoint
        .meta
        .as_ref()
        .ok_or_else(|| "checkpoint.meta is required".to_owned())?;
    if occupancy_meta.revision != occupancy_receipt.occupancy_revision
        || occupancy_meta.timestamp_ns != occupancy_receipt.source_timestamp_ns
    {
        return Err(
            "mapping backend checkpoint occupancy receipt does not match checkpoint metadata"
                .to_owned(),
        );
    }
    let cloud_checkpoint = checkpoint.canonical_cloud_checkpoint.as_ref().ok_or_else(|| {
        "mapping backend checkpoint requires its correlated cloud checkpoint payload".to_owned()
    })?;
    if cloud_checkpoint.source_revision != cloud_receipt.cloud_revision
        || cloud_checkpoint.point_count != cloud_receipt.point_count
    {
        return Err(
            "mapping backend checkpoint cloud receipt does not match cloud checkpoint metadata"
                .to_owned(),
        );
    }
    Ok(())
}

fn exploration_state_from_snapshot(
    snapshot: crate::explore_runtime::ExploreRuntimeSnapshot,
) -> ExplorationState {
    let child_lease = snapshot.child_lease.map(|lease| ExplorationChildLease {
        session_id: lease.session_id,
        owner_kind: lease.owner_kind,
        parent_request_id: lease.parent_request_id,
        goal_generation: lease.goal_generation,
        producer_runtime_generation: lease.producer_runtime_generation,

    });
    ExplorationState {
        parking_pending: snapshot.parking_pending,
        status: snapshot.status.to_string(),
        message: snapshot.message,
        request_id: snapshot.request_id,
        selected_source: snapshot.selected_source,
        last_stop_reason: snapshot.last_stop_reason,
        frontier_count: snapshot.frontier_count,
        revision: snapshot.revision,
        timestamp_ns: now_ns(),
        child_task_id: snapshot.child_task_id,
        exploration_generation: snapshot.exploration_generation,
        child_lease,


        mission_progress_json: snapshot.mission_progress_json,
    }
}

fn explore_config_from_request(
    request: &ExploreRequest,
    expected_map_frame_id: &str,
) -> Result<crate::explore_runtime::ExploreStartConfig, String> {
    let boundary = if request.boundary_line_segments.is_empty() {
        if !request.boundary_frame_id.trim().is_empty() {
            return Err("empty exploration boundary must have an empty frame".to_string());
        }
        None
    } else {
        if request.boundary_frame_id.trim() != expected_map_frame_id {
            return Err(format!(
                "exploration boundary_frame_id must be {expected_map_frame_id}"
            ));
        }
        let segments = request
            .boundary_line_segments
            .iter()
            .map(|segment| BoundarySegment {
                start: Point2 {
                    x: segment.start_x,
                    y: segment.start_y,
                },
                end: Point2 {
                    x: segment.end_x,
                    y: segment.end_y,
                },
            })
            .collect();
        Some(ExplorationBoundary::from_segments(segments).map_err(|error| error.to_string())?)
    };
    let stop_conditions = request
        .stop_conditions
        .as_ref()
        .map(|conditions| crate::explore_runtime::ExploreStopConditions {
            max_coverage_pct: conditions.max_coverage_pct,
            no_frontier_timeout_ms: conditions.no_frontier_timeout_ms,
            max_duration_ms: conditions.max_duration_ms,
            max_goals: conditions.max_goals,
        })
        .unwrap_or_default();
    let arbitration = if request.arbitration.trim().is_empty() {
        std::env::var("NUWAX_EXPLORE_ARBITRATION")
            .unwrap_or_else(|_| "prefer_wavefront".to_string())
    } else {
        request.arbitration.clone()
    };
    Ok(crate::explore_runtime::ExploreStartConfig {
        strategy: if request.strategy.trim().is_empty() {
            "wavefront".to_string()
        } else {
            request.strategy.clone()
        },
        arbitration,
        motion_admission: MotionAdmissionProof::try_from_wire(
            &request.motion_guard_boot_nonce,
            request.motion_stop_generation,
        )?,
        boundary,
        stop_conditions,
        max_navigation_retries: request.max_navigation_retries,
        blacklist_radius_m: request.blacklist_radius_m,
        resume_progress_json: request.resume_progress_json.clone(),
        return_policy: request.return_policy.as_ref().map(|p| nuwax_exploration::mission::ReturnPolicy {
            maximum_distance_m:p.maximum_distance_m,reserve_distance_m:p.reserve_distance_m,return_on_completion:p.return_on_completion,
        }).unwrap_or_default(),
    })
}

async fn forward_map_revision(
    tx: &tokio::sync::mpsc::Sender<Result<MapUpdateNotification, Status>>,
    event: MapRevisionEvent,
    occupancy_shm_current: &Arc<RwLock<Option<crate::lcm_runtime::OccupancyShmSnapshot>>>,
    expected_map_frame_id: &str,
) -> bool {
    if event.session_id.trim().is_empty()
        || event.map_name.trim().is_empty()
        || event.session_generation == 0
    {
        return tx
            .send(Err(Status::failed_precondition(
                "map revision is missing mapping session identity",
            )))
            .await
            .is_ok();
    }
    if event.frame_id.trim() != expected_map_frame_id {
        return tx
            .send(Err(Status::failed_precondition(format!(
                "global map subscriber requires frame={expected_map_frame_id}, received {}",
                event.frame_id,
            ))))
            .await
            .is_ok();
    }
    if event.size != event.data.len() as u64 {
        return tx
            .send(Err(Status::data_loss(format!(
                "map revision {} size mismatch: declared={}, actual={}",
                event.revision,
                event.size,
                event.data.len()
            ))))
            .await
            .is_ok();
    }
    if let Err(error) = event.validate_provenance() {
        return tx.send(Err(Status::data_loss(error))).await.is_ok();
    }
    let occupancy_receipt = occupancy_commit_receipt_proto(&event.receipt);
    let shm_ref = match occupancy_shm_current.read().await.clone() {
        Some(snapshot) if snapshot.revision == event.revision
            && snapshot.identity.matches_event(&event) => {
            match occupancy_shm_ref(snapshot) {
                Ok(reference) => Some(reference),
                Err(status) => return tx.send(Err(*status)).await.is_ok(),
            }
        }
        Some(_) => None,
        None => None,
    };

    tx.send(Ok(MapUpdateNotification {
        meta: Some(OccupancyGridMeta {
            width: event.width,
            height: event.height,
            resolution: event.resolution,
            origin_x: event.origin_x,
            origin_y: event.origin_y,
            frame_id: event.frame_id,
            revision: event.revision,
            timestamp_ns: event.timestamp_ns,
        }),
        shm_ref,
        data: event.data.as_ref().to_vec(),
        size: event.size,
        schema_version: event.schema_version,
        occupancy_receipt: Some(occupancy_receipt),
        raw_representation: event.raw_representation as i32,
        last_seen_ms: event.last_seen_ms,
        session_id: event.session_id,
        session_generation: event.session_generation,
        raw_data: event.raw_data.as_ref().to_vec(),
        map_name: event.map_name,
    }))
    .await
    .is_ok()
}

async fn map_revision_claim_is_current(
    event: &MapRevisionEvent,
    mapping_adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
) -> bool {
    let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
    crate::lcm_runtime::map_event_matches_current_session(event, mapping_adapter)
}

async fn await_occupancy_shm_revision(
    bridge: &GeometryRuntimeHandle,
    target_revision: u64,
) -> Result<crate::lcm_runtime::OccupancyShmSnapshot, Status> {
    if bridge.occupancy_shm.is_none() {
        return Err(Status::unavailable("occupancy SHM writer is unavailable"));
    }

    let mut committed_revision_rx = bridge.subscribe_map_revisions();
    let deadline = tokio::time::Instant::now() + OCCUPANCY_SHM_COMMIT_TIMEOUT;
    loop {
        if let Some(snapshot) = bridge.occupancy_shm_current.read().await.clone() {
            if snapshot.revision >= target_revision {
                return Ok(snapshot);
            }
        }
        if committed_revision_rx
            .borrow()
            .as_ref()
            .is_some_and(|event| event.revision >= target_revision)
        {
            return Err(Status::internal(format!(
                "map revision {target_revision} is visible without a matching occupancy SHM descriptor"
            )));
        }

        match tokio::time::timeout_at(deadline, committed_revision_rx.changed()).await {
            Ok(Ok(())) => {}
            Ok(Err(_)) => {
                return Err(Status::unavailable(format!(
                    "occupancy SHM commit channel closed before revision {target_revision}"
                )));
            }
            Err(_) => {
                return Err(Status::deadline_exceeded(format!(
                    "occupancy SHM revision {target_revision} did not commit within {} ms",
                    OCCUPANCY_SHM_COMMIT_TIMEOUT.as_millis()
                )));
            }
        }
    }
}

fn occupancy_shm_ref(
    snapshot: crate::lcm_runtime::OccupancyShmSnapshot,
) -> Result<ShmRef, Box<Status>> {
    if snapshot.key.trim().is_empty()
        || snapshot.file_size == 0
        || snapshot.revision == 0
        || snapshot.layout_version != crate::occupancy_shm::OCCUPANCY_SHM_LAYOUT_VERSION
        || snapshot.payload_schema_version != 1
        || snapshot.generation == 0
        || snapshot.active_slot >= 2
        || snapshot.payload_size == 0
        || snapshot.payload_size >= snapshot.file_size
    {
        return Err(Box::new(Status::failed_precondition(format!(
            "invalid occupancy SHM v3 reference: layout={} schema={} generation={} slot={} file_size={} payload_size={} revision={}",
            snapshot.layout_version,
            snapshot.payload_schema_version,
            snapshot.generation,
            snapshot.active_slot,
            snapshot.file_size,
            snapshot.payload_size,
            snapshot.revision,
        ))));
    }
    let active_slot = u32::try_from(snapshot.active_slot).map_err(|_| {
        Box::new(Status::failed_precondition(
            "occupancy SHM active slot exceeds uint32",
        ))
    })?;
    Ok(ShmRef {
        key: snapshot.key,
        size: snapshot.file_size,
        revision: snapshot.revision,
        layout_version: snapshot.layout_version,
        payload_schema_version: snapshot.payload_schema_version,
        generation: snapshot.generation,
        active_slot,
        payload_size: snapshot.payload_size,
    })
}

fn cloud_map_shm_ref(notification: &CloudMapNotification) -> Result<ShmRef, Box<Status>> {
    if notification.revision == 0 || notification.session_generation == 0 {
        return Err(Box::new(Status::failed_precondition(format!(
            "invalid cloud-map SHM v3 reference: revision={} generation={}",
            notification.revision, notification.session_generation
        ))));
    }
    let file_size = u64::try_from(crate::pointcloud::CM_TOTAL_SIZE)
        .map_err(|_| Box::new(Status::internal("cloud-map SHM size exceeds uint64")))?;
    let payload_size = u64::from(notification.point_count)
        .checked_mul(crate::pointcloud::CM_POINT_SIZE_BYTES as u64)
        .ok_or_else(|| Box::new(Status::internal("cloud-map payload size overflow")))?;
    let max_payload_size = u64::try_from(crate::pointcloud::CM_SINGLE_BUFFER_SIZE)
        .map_err(|_| Box::new(Status::internal("cloud-map slot size exceeds uint64")))?;
    if payload_size > max_payload_size {
        return Err(Box::new(Status::failed_precondition(format!(
            "cloud-map payload exceeds one SHM slot: payload_size={} max={}",
            payload_size, max_payload_size
        ))));
    }
    let active_slot =
        crate::pointcloud::cloud_map_active_slot(notification.revision).ok_or_else(|| {
            Box::new(Status::failed_precondition(
                "cloud-map revision must be positive",
            ))
        })?;
    Ok(ShmRef {
        key: crate::pointcloud::cloud_map_shm_path()
            .to_string_lossy()
            .into_owned(),
        size: file_size,
        revision: notification.revision,
        layout_version: crate::pointcloud::CM_SHM_VERSION,
        payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
        generation: notification.session_generation,
        active_slot,
        payload_size,
    })
}

fn canonical_xyz_fields() -> Vec<schema::PointField> {
    vec![
        schema::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: 7,
            count: 1,
        },
    ]
}

fn point_field_size(datatype: u32) -> Option<u32> {
    match datatype {
        1 | 2 => Some(1),
        3 | 4 => Some(2),
        5..=7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

fn validate_runtime_point_fields(fields: &[PointCloudField], point_step: u32) -> CodecResult<()> {
    for field in fields {
        let size = point_field_size(field.datatype).ok_or_else(|| {
            CodecError::LcmEncode(format!(
                "unsupported point field datatype {} for {:?}",
                field.datatype, field.name
            ))
        })?;
        let field_end = field
            .offset
            .checked_add(size.checked_mul(field.count).ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} size overflow", field.name))
            })?)
            .ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} offset overflow", field.name))
            })?;
        if field.name.is_empty() || field.count == 0 || field_end > point_step {
            return Err(CodecError::LcmEncode(format!(
                "invalid point field name={:?} offset={} datatype={} count={} point_step={}",
                field.name, field.offset, field.datatype, field.count, point_step
            )));
        }
    }

    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        let field = fields.iter().find(|field| field.name == name);
        if !matches!(
            field,
            Some(field) if field.offset == offset && field.datatype == 7 && field.count == 1
        ) {
            return Err(CodecError::LcmEncode(format!(
                "runtime requires canonical {name} float32 field at offset {offset}"
            )));
        }
    }
    Ok(())
}

fn validate_native_raw_pointcloud_fields(request: &PublishPointCloudRequest) -> CodecResult<()> {
    if request.fields.is_empty() {
        return Err(CodecError::LcmEncode(
            "native raw PointCloud2 requires explicit fields; fields cannot be inferred".into(),
        ));
    }
    validate_runtime_point_fields(&request.fields, request.point_step)?;
    let offset = request
        .fields
        .iter()
        .find(|field| field.name == "offset_ns");
    if !matches!(offset, Some(field) if field.datatype == 6 && field.count == 1) {
        return Err(CodecError::LcmEncode(
            "native raw PointCloud2 requires offset_ns UINT32 field".into(),
        ));
    }
    Ok(())
}

fn encode_native_imu_payload(sample: &LioImuSample) -> CodecResult<Vec<u8>> {
    let values = [
        sample.accel_mps2_x,
        sample.accel_mps2_y,
        sample.accel_mps2_z,
        sample.gyro_rps_x,
        sample.gyro_rps_y,
        sample.gyro_rps_z,
    ];
    if sample.frame_id.is_empty() || sample.frame_id.trim() != sample.frame_id {
        return Err(CodecError::LcmEncode(
            "native raw IMU frame_id must be non-empty without surrounding whitespace".into(),
        ));
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(CodecError::LcmEncode(
            "native raw IMU acceleration and gyro values must be finite".into(),
        ));
    }
    let timestamp_ns = sample.timestamp_ns;
    let stamp_sec = i32::try_from(timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("IMU timestamp seconds exceed int32".into()))?;
    let orientation = if sample.orientation_valid {
        let q = [
            sample.orientation_x,
            sample.orientation_y,
            sample.orientation_z,
            sample.orientation_w,
        ];
        if q.iter().any(|value| !value.is_finite()) {
            return Err(CodecError::LcmEncode(
                "native raw IMU orientation must be finite when orientation_valid".into(),
            ));
        }
        (q, [0.0; 9])
    } else {
        // sensor_msgs/Imu convention: orientation_covariance[0] == -1 means unknown.
        ([0.0; 4], [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    };
    Ok(schema::encode_imu_body(&schema::Imu {
        header: schema::Header {
            seq: 0,
            stamp_sec,
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: sample.frame_id.clone(),
        },
        orientation_xyzw: orientation.0,
        orientation_covariance: orientation.1,
        angular_velocity: [sample.gyro_rps_x, sample.gyro_rps_y, sample.gyro_rps_z],
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: [
            sample.accel_mps2_x,
            sample.accel_mps2_y,
            sample.accel_mps2_z,
        ],
        linear_acceleration_covariance: [0.0; 9],
    }))
}

/// 发布一个 native raw measurement 的 typed 输入对。
///
/// LIO child 在收到 cloud callback 时会立即检查 IMU 是否覆盖整个扫描窗口，
/// 因而必须先发完本次 measurement 的全部 IMU，再发 cloud。LCM 没有跨消息
/// 事务；中途失败时保留已经成功发布的 IMU 前缀，但绝不提前发布 cloud。
/// 整帧只向原通信线程提交一次并等待一次确认，避免逐样本累计空队列等待。
pub(crate) async fn publish_native_raw_pair(
    transport: &nuwax_transport::LcmTransport,
    cloud_binding: &(LcmTopic, LcmMessageType),
    imu_binding: &(LcmTopic, LcmMessageType),
    cloud_payload: Vec<u8>,
    imu_payloads: Vec<Vec<u8>>,
) -> Result<(), Status> {
    let mut frames = Vec::with_capacity(imu_payloads.len() + 1);
    for imu_payload in imu_payloads {
        frames.push(typed_codec::build_typed_frame(
            imu_binding.0.clone(),
            imu_binding.1.clone(),
            imu_payload,
            0,
        ));
    }
    frames.push(typed_codec::build_typed_frame(
        cloud_binding.0.clone(),
        cloud_binding.1.clone(),
        cloud_payload,
        0,
    ));
    transport
        .publish_batch(frames)
        .await
        .map_err(|error| Status::unavailable(format!("native input batch publish failed: {error}")))
}

fn encode_pointcloud_request_payload(request: &PublishPointCloudRequest) -> CodecResult<Vec<u8>> {
    encode_pointcloud_request_payload_with_sequence(request, 0)
}

pub(crate) fn encode_pointcloud_request_payload_with_sequence(request: &PublishPointCloudRequest, sequence: i32) -> CodecResult<Vec<u8>> {
    if request.frame_id.is_empty() || request.frame_id.trim() != request.frame_id {
        return Err(CodecError::LcmEncode(
            "point cloud frame_id must be non-empty without surrounding whitespace".into(),
        ));
    }
    let point_step = usize::try_from(request.point_step)
        .map_err(|_| CodecError::LcmEncode("point_step exceeds usize".into()))?;
    if point_step < 12 || request.points.len() % point_step != 0 {
        return Err(CodecError::LcmEncode(format!(
            "point cloud data length {} is not aligned to point_step {}",
            request.points.len(),
            request.point_step
        )));
    }

    let derived_point_count = request.points.len() / point_step;
    if request.point_count != 0 && request.point_count as usize != derived_point_count {
        return Err(CodecError::LcmEncode(format!(
            "point_count {} does not match data-derived count {}",
            request.point_count, derived_point_count
        )));
    }

    let fields = if request.fields.is_empty() {
        canonical_xyz_fields()
    } else {
        validate_runtime_point_fields(&request.fields, request.point_step)?;
        request
            .fields
            .iter()
            .map(|field| {
                Ok(schema::PointField {
                    name: field.name.clone(),
                    offset: i32::try_from(field.offset).map_err(|_| {
                        CodecError::LcmEncode("point field offset exceeds int32".into())
                    })?,
                    datatype: u8::try_from(field.datatype).map_err(|_| {
                        CodecError::LcmEncode("point field datatype exceeds uint8".into())
                    })?,
                    count: i32::try_from(field.count).map_err(|_| {
                        CodecError::LcmEncode("point field count exceeds int32".into())
                    })?,
                })
            })
            .collect::<CodecResult<Vec<_>>>()?
    };

    let stamp_sec = i32::try_from(request.timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("point cloud timestamp seconds exceed int32".into()))?;
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema::Header {
            seq: sequence,
            stamp_sec,
            stamp_nsec: (request.timestamp_ns % 1_000_000_000) as i32,
            frame_id: request.frame_id.clone(),
        },
        height: 1,
        width: i32::try_from(derived_point_count)
            .map_err(|_| CodecError::LcmEncode("point cloud width exceeds int32".into()))?,
        fields,
        is_bigendian: false,
        point_step: i32::try_from(request.point_step)
            .map_err(|_| CodecError::LcmEncode("point_step exceeds int32".into()))?,
        row_step: i32::try_from(request.points.len())
            .map_err(|_| CodecError::LcmEncode("point cloud row_step exceeds int32".into()))?,
        data: request.points.clone(),
        is_dense: request.is_dense,
    })
}

/// 把内部 `NavState`(crate::lcm_runtime::NavState)转成 gRPC
/// `proto::NavigationState`,避免 server.rs 直接依赖 internal type。
fn nav_state_to_proto(update: NavigationUpdate) -> NavigationState {
    let NavigationUpdate {
        state: nav,
        feedback,
        producer_runtime_generation,
    } = update;
    let local_path = if nav.status == "RUNNING" { feedback.local_path } else { Vec::new() };
    NavigationState {
        status: nav.status,
        message: nav.message,
        goal_id: nav.goal_id,
        distance_remaining: nav.distance_remaining,
        timestamp_ns: nav.timestamp_ns,
        cancel_phase: feedback.cancel_phase,
        cancel_generation: feedback.cancel_generation,
        actuator_zeroed: feedback.actuator_zeroed,
        reason_code: feedback.reason_code,
        runtime_generation: feedback.runtime_generation,

        sink_ready: feedback.sink_ready,
        planner_map_frame: feedback.planner_map_frame,
        planner_pose_frame: feedback.planner_pose_frame,
        zero_ack_id: feedback.zero_ack_id,
        zero_applied_monotonic_ns: feedback.zero_applied_monotonic_ns,
        owner_kind: feedback.owner_kind,
        parent_request_id: feedback.parent_request_id,
        goal_generation: feedback.goal_generation,
        path_revision: feedback.path_revision,
        planner_runtime_generation: feedback.planner_runtime_generation,
        producer_runtime_generation,
        planned_path: feedback.planned_path.into_iter().map(|p| Vec3 { x:p[0], y:p[1], z:p[2] }).collect(),
        local_path: local_path.into_iter().map(|p| Vec3 { x:p[0], y:p[1], z:p[2] }).collect(),
    }
}

impl GeometryRuntimeService {
    pub(crate) async fn perform_process_stop(&self) -> Result<(), Status> {
        let shutdown = self.shutdown.as_ref().ok_or_else(|| {
            Status::failed_precondition("runtime 未配置进程级 shutdown coordinator")
        })?;
        let process_stop_attempt = shutdown.begin_process_stop_phase_a();
        if let Some(attempt) = process_stop_attempt
            .as_ref()
            .filter(|attempt| !attempt.owns_transition())
        {
            return match attempt.wait_for_owner_result().await {
                Ok(()) => Ok(()),
                Err(()) => Err(Status::failed_precondition(
                    "the first Process Stop attempt failed or correction admission closed internally",
                )),
            };
        }
        let _stop_single_flight = shutdown.stop_single_flight.lock().await;
        let already_stopped = {
            let mut state = self.state.write().await;
            if state.phase == "stopped" {
                true
            } else {
                state.phase = "stopping".into();
                state.message = "正在等待探索执行器安全停车".into();
                false
            }
        };
        if already_stopped {
            if let Some(process_stop_attempt) = process_stop_attempt {
                process_stop_attempt.complete();
            }
            return Ok(());
        }
        let mapping_transition = match tokio::time::timeout(
            STOP_LIFECYCLE_TIMEOUT,
            self.mapping_transition.lock(),
        )
        .await
        {
            Ok(guard) => guard,
            Err(_) => {
                let error = Status::deadline_exceeded(
                    "timed out settling the Mapping Start/Stop transition during process Stop",
                );
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
        };
        let (projection_guard, lifecycle_guard) = match tokio::time::timeout(STOP_LIFECYCLE_TIMEOUT, async {
                let projection = self.bridge.lock_explore_projection_owned().await;
                let lifecycle = self
                    .bridge
                    .mapping_lifecycle_lock
                    .clone()
                    .lock_owned()
                    .await;
                (projection, lifecycle)
            })
            .await
            {
                Ok((projection, lifecycle)) => (projection, lifecycle),
                Err(_) => {
                    let error = Status::deadline_exceeded(
                        "timed out acquiring mapping lifecycle fence during process Stop",
                    );
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
            };
        drop(lifecycle_guard);
        macro_rules! fail_before_motion_terminal {
            ($error:expr) => {{
                let error = $error;
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }};
        }
        if tokio::time::timeout(
            STOP_LIFECYCLE_TIMEOUT,
            shutdown.wait_for_correction_settlement(),
        )
        .await
        .is_err()
        {
            let error =
                Status::deadline_exceeded("timed out settling correction during process Stop");
            fail_before_motion_terminal!(error);
        }
        // Process Stop 阶段 A 此时已成为不可逆 owner。取消 await 期间只持 P、不持 L；
        // retained 元数据仅在下方精确运动 ownership 终态后消费。

        let request_id = self.bridge.explore.snapshot().request_id;
        let stop_snapshot = match tokio::time::timeout(
            STOP_EXPLORATION_DISPATCH_TIMEOUT,
            self.bridge.explore.stop(&request_id),
        )
        .await
        {
            Ok(snapshot) => snapshot,
            Err(_) => {
                // The scheduler can hold ExploreRuntimeInner while a navigation
                // submit awaits transport completion. Abort it, which drops that
                // future and releases the mutex, then make one correlated retry.
                let scheduler_shutdown = tokio::time::timeout(
                    STOP_EXPLORATION_SCHEDULER_TIMEOUT,
                    self.bridge.shutdown_explore_scheduler(),
                )
                .await;
                let scheduler_detail = match scheduler_shutdown {
                    Ok(Ok(())) => "scheduler was shut down".to_owned(),
                    Ok(Err(error)) => {
                        let error = Status::internal(format!(
                            "exploration scheduler shutdown failed during stop recovery: {error}"
                        ));
                        fail_before_motion_terminal!(error);
                    }
                    Err(_) => {
                        let error = Status::deadline_exceeded(
                            "timed out shutting down exploration scheduler during stop recovery",
                        );
                        fail_before_motion_terminal!(error);
                    }
                };
                match tokio::time::timeout(
                    STOP_EXPLORATION_DISPATCH_TIMEOUT,
                    self.bridge.explore.stop(&request_id),
                )
                .await
                {
                    Ok(snapshot) => snapshot,
                    Err(_) => {
                        let snapshot = self.bridge.explore.snapshot();
                        let error = Status::deadline_exceeded(format!(
                            "timed out dispatching exploration safety stop after scheduler recovery ({scheduler_detail}): request_id={} child_task_id={} status={}",
                            snapshot.request_id, snapshot.child_task_id, snapshot.status
                        ));
                        fail_before_motion_terminal!(error);
                    }
                }
            }
        };
        let safety_wait = tokio::time::timeout(STOP_EXPLORATION_SAFETY_TIMEOUT, async {
            let mut snapshot = stop_snapshot;
            loop {
                self.bridge
                    .explore
                    .reconcile_orphan_cancellations()
                    .await;
                if !self.bridge.explore.blocks_mapping_mutation().await {
                    return Ok(snapshot);
                }
                if matches!(
                    snapshot.status,
                    crate::explore_runtime::STATUS_FAILED
                        | crate::explore_runtime::STATUS_REJECTED
                ) {
                    return Err(Status::failed_precondition(format!(
                        "exploration safety ownership retained: request_id={} child_task_id={} message={}",
                        snapshot.request_id, snapshot.child_task_id, snapshot.message
                    )));
                }
                tokio::time::sleep(STOP_EXPLORATION_SAFETY_POLL).await;
                snapshot = self.bridge.explore.snapshot();
            }
        })
        .await;
        match safety_wait {
            Ok(Ok(_)) => {}
            Ok(Err(error)) => {
                fail_before_motion_terminal!(error);
            }
            Err(_) => {
                let snapshot = self.bridge.explore.snapshot();
                let error = Status::deadline_exceeded(format!(
                    "timed out waiting for exploration safety ownership release: request_id={} child_task_id={} status={}",
                    snapshot.request_id, snapshot.child_task_id, snapshot.status
                ));
                fail_before_motion_terminal!(error);
            }
        }

        if let Some(active_lease) = self.bridge.nav.active_goal_lease().await {
            {
                let mut state = self.state.write().await;
                state.message = "正在等待导航执行器安全停车".into();
            }
            let CancelOutcome { state: nav, lease } = match tokio::time::timeout(
                STOP_NAVIGATION_DISPATCH_TIMEOUT,
                nav_cancel(
                    &self.bridge,
                    &self.bridge.nav,
                    crate::nav_runtime::CancelGoalInput {
                        goal_id: active_lease.session_id.clone(),
                        owner_kind: active_lease.owner_kind,
                        parent_request_id: active_lease.parent_request_id.clone(),
                        goal_generation: active_lease.goal_generation,
                        producer_runtime_generation: active_lease.producer_runtime_generation,
                    },
                ),
            )
            .await
            {
                Ok(outcome) => outcome,
                Err(_) => {
                    let cancel = self.bridge.nav.cancel_snapshot();
                    let error = Status::deadline_exceeded(format!(
                        "timed out dispatching navigation safety stop: goal_id={} cancel_generation={} phase={:?}",
                        active_lease.session_id, cancel.generation, cancel.phase
                    ));
                    fail_before_motion_terminal!(error);
                }
            };
            {
                let mut shared = self.bridge.state.write().await;
                shared.nav_state = nav.clone();
            }
            let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
            let _ = self.bridge.nav_state_tx.send(update);

            let cancel_after_dispatch = self.bridge.nav.cancel_snapshot();
            let expected_cancel_lease = cancel_after_dispatch.lease.clone();
            if expected_cancel_lease.is_none()
                && self.bridge.nav.active_goal_lease().await.is_some()
            {
                let error = Status::failed_precondition(format!(
                    "navigation cancel did not establish an exact safety ticket: goal_id={} phase={:?} message={}",
                    active_lease.session_id,
                    cancel_after_dispatch.phase,
                    cancel_after_dispatch.message
                ));
                fail_before_motion_terminal!(error);
            }

            let navigation_wait =
                tokio::time::timeout(STOP_EXPLORATION_SAFETY_TIMEOUT, async {
                    loop {
                        let cancel = self.bridge.nav.cancel_snapshot();
                        let active = self.bridge.nav.active_goal_lease().await;
                        if expected_cancel_lease.is_none() && active.is_none() {
                            return Ok(());
                        }
                        if active.is_none()
                            && cancel.phase == crate::nav_runtime::CancelPhase::Zeroed
                            && cancel.lease == expected_cancel_lease
                        {
                            return Ok(());
                        }
                        if matches!(
                            cancel.phase,
                            crate::nav_runtime::CancelPhase::Failed
                                | crate::nav_runtime::CancelPhase::TimedOut
                        ) {
                            return Err(Status::failed_precondition(format!(
                                "navigation safety ownership retained: goal_id={} cancel_generation={} phase={:?} message={}",
                                active_lease.session_id,
                                cancel.generation,
                                cancel.phase,
                                cancel.message
                            )));
                        }
                        tokio::time::sleep(STOP_EXPLORATION_SAFETY_POLL).await;
                    }
                })
                .await;
            match navigation_wait {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    fail_before_motion_terminal!(error);
                }
                Err(_) => {
                    let cancel = self.bridge.nav.cancel_snapshot();
                    let error = Status::deadline_exceeded(format!(
                        "timed out waiting for navigation safety ownership release: goal_id={} cancel_generation={} phase={:?}",
                        active_lease.session_id, cancel.generation, cancel.phase
                    ));
                    fail_before_motion_terminal!(error);
                }
            }
        }

        #[cfg(test)]
        {
            let barrier = self.stop_before_post_zero_lifecycle.lock().take();
            if let Some(barrier) = barrier {
                barrier.wait().await;
                barrier.wait().await;
            }
        }
        let lifecycle_guard = match tokio::time::timeout(
            STOP_LIFECYCLE_TIMEOUT,
            self.bridge.mapping_lifecycle_lock.clone().lock_owned(),
        )
        .await
        {
            Ok(guard) => guard,
            Err(_) => {
                let error = Status::deadline_exceeded(
                    "timed out reacquiring mapping lifecycle fence after exact Process Stop motion terminal",
                );
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
        };
        match tokio::time::timeout(
            STOP_EXPLORATION_SCHEDULER_TIMEOUT,
            self.bridge.shutdown_explore_scheduler(),
        )
        .await
        {
            Ok(Ok(())) => {}
            Ok(Err(detail)) => {
                let error =
                    Status::internal(format!("exploration scheduler shutdown failed: {detail}"));
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
            Err(_) => {
                let error =
                    Status::deadline_exceeded("timed out shutting down exploration scheduler");
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
        }

        {
            let mut state = self.state.write().await;
            state.message = "正在关闭 gRPC listener".into();
        }

        if let Err(error) = shutdown.request_and_wait_for_listener().await {
            let mut state = self.state.write().await;
            state.phase = "failed".into();
            state.message = error.message().to_string();
            return Err(error);
        }

        {
            let mut state = self.state.write().await;
            state.phase = "stopped".into();
            state.message = "gRPC listener 已关闭".into();
        }
        drop(lifecycle_guard);
        drop(projection_guard);
        drop(mapping_transition);
        #[cfg(test)]
        {
            let barrier = self.stop_before_graceful.lock().take();
            if let Some(barrier) = barrier {
                barrier.wait().await;
                barrier.wait().await;
            }
        }
        shutdown.publish_graceful_shutdown();
        if let Some(process_stop_attempt) = process_stop_attempt {
            process_stop_attempt.complete();
        }
        Ok(())
    }
}

impl GeometryRuntimeService {
    async fn export_planar_checkpoint(&self)->Result<MappingBackendCheckpoint,String> {
        use nuwax_slam_2d_runtime::{protocol::Operation,archive};
        let _transition=self.mapping_transition.lock().await;
        self.ensure_running_admission("ExportMappingBackendCheckpoint").await.map_err(|e|e.to_string())?;
        let adapter=self.bridge.mapping_adapter.as_ref().ok_or("二维适配器不可用")?.clone();
        let binding=adapter.lock().planar.clone().ok_or("二维拥有方不可用")?;
        let _operation=binding.operation.lock().await;
        let epoch=binding.fence.snapshot();
        if epoch.active {return Err("重定位期间不允许导出图".into());}
        let session=adapter.lock().snapshot();
        let status=binding.client.status().await?;
        if status.session.as_ref().is_none_or(|s|s.session_id!=session.session_id || s.session_generation!=session.session_generation) {
            return Err("二维导出会话身份不一致".into());
        }
        let pause=Operation::SetPaused{session_id:session.session_id.clone(),session_generation:session.session_generation,paused:true};
        binding.control_until_settled(pause).await?;
        let result=async {
            let response=binding.control_until_settled(Operation::ExportGraph{session_id:session.session_id.clone(),
                session_generation:session.session_generation,unix_offset_ns:i64::try_from(binding.settings.unix_offset_ns).map_err(|_|"时钟偏移越界")?}).await?;
            let artifact=response.artifact.ok_or("二维导出未返回制品")?;
            let client=binding.client.current();
            let exported=async {
                let mut file=client.open_artifact(&artifact).await?;
                let (file,metadata,cells)=tokio::task::spawn_blocking(move|| {
                    let (metadata,cells)=archive::grid_structure(&mut file)?;
                    Ok::<_,String>((file,metadata,cells))
                }).await.map_err(|e|e.to_string())??;
                if metadata.identity() != artifact.identity { return Err("二维导出档案与worker身份不符".into()); }
                let archive::ArchiveMetadata::Assigned(assigned) = &metadata else { return Err("二维导出必须返回编号档案".into()); };
                let checkpoint_id = assigned.checkpoint_id.clone();
                let metadata = &assigned.source;
                let mut snapshot=nuwax_slam_2d_runtime::protocol::Snapshot {
                    snapshot_id:1,session:metadata.session.clone(),estimate:metadata.estimate.clone(),width:metadata.width,height:metadata.height,
                    resolution:metadata.resolution,origin:metadata.origin,payload_size:metadata.members[2].size,
                    file_name:String::new(),
                };
                // 定位静态图保留档案来源，但归属当前实际会话。
                snapshot.session=binding.session(&session.session_id,&session.map_name,session.session_generation);
                if adapter.lock().planar_revision<snapshot.estimate.graph_revision {
                    let preparation=adapter.lock().prepare_planar_map(snapshot,cells.clone(),metadata.source_unix_timestamp_ns);
                    let prepared=tokio::task::spawn_blocking(move||preparation.compute()).await.map_err(|e|e.to_string())??;
                    let _projection=self.bridge.lock_explore_projection_owned().await;
                    let _lifecycle=self.bridge.mapping_lifecycle_lock.lock().await;
                    if binding.fence.snapshot()!=epoch {return Err("二维导出跨越重定位代际".into());}
                    let mut state=self.bridge.state.write().await;
                    let (map,event)=adapter.lock().commit_planar_export(prepared,state.map.revision)?;
                    state.map=map;drop(state);let _=self.bridge.map_revision_tx.send(event);
                }
                let (current,map,receipt)={
                    let a=adapter.lock();
                    (a.snapshot(),a.raw_map_snapshot().ok_or("二维导出缺少规范物理栅格")?,a.latest_occupancy_commit_receipt().ok_or("二维导出缺少规范提交收据")?)
                };
                if current.session_id!=session.session_id || current.session_generation!=session.session_generation
                    || map.width!=metadata.width || map.height!=metadata.height || map.resolution!=metadata.resolution as f32
                    || [map.origin_x,map.origin_y]!=metadata.origin || map.data!=cells || map.timestamp_ns!=metadata.source_unix_timestamp_ns
                    || receipt.occupancy_revision!=map.revision || receipt.source_timestamp_ns!=map.timestamp_ns {
                    return Err("二维图档案与规范物理栅格或收据不一致".into());
                }
                let canonical=occupancy_checkpoint_proto(&map);
                let occupancy_receipt = occupancy_commit_receipt_proto(&receipt);
                let member = CheckpointMemberIdentity { checkpoint_id: checkpoint_id.clone(), role: CheckpointMemberRole::PlanarGraph as i32,
                    occupancy_receipt: Some(occupancy_receipt.clone()), cloud_receipt: None,
                    frame_id: binding.settings.template.map_frame_id.clone(), frame_generation: 0, producer_generation: 0 };
                let mut occupancy_identity = member.clone(); occupancy_identity.role = CheckpointMemberRole::CanonicalOccupancy as i32;
                let store=self.graph_artifacts.clone();let size=artifact.payload_size;
                let reference=tokio::task::spawn_blocking(move||store.lock().insert(file,size,member)).await.map_err(|e|e.to_string())??;
                let checkpoint = MappingBackendCheckpoint{schema_version:2,backend_kind:"slam_toolbox".into(),source_session_generation:session.session_generation,
                    source_session_id:session.session_id.clone(),payload_size:artifact.payload_size, checkpoint_id, backend_payload_schema_version: 2,
                    created_at_ns:now_ns(),occupancy_receipt:Some(occupancy_receipt),canonical_occupancy_checkpoint:Some(canonical),
                    occupancy_identity: Some(occupancy_identity), graph_artifact:Some(reference),target_kind:"planar_graph".into(),..Default::default()};
                        Ok(checkpoint)
            }.await;
            let released=binding.control_until_settled(Operation::ReleaseArtifact{artifact}).await;
            match (exported,released) {(Ok(v),Ok(_))=>Ok(v),(Err(e),_)=>Err(e),(Ok(_),Err(e))=>Err(e)}
        }.await;
        if status.active {
            if let Err(e)=binding.control_until_settled(Operation::SetPaused{session_id:session.session_id,session_generation:session.session_generation,paused:false}).await {
                adapter.lock().planar_ready=false;return Err(e);
            }
        }
        result
    }
    async fn start_planar_mapping(
        &self,
        request: StartMappingRequest,
        cancellation: watch::Receiver<bool>,
    ) -> Result<Response<MappingSessionState>, Status> {
        use nuwax_slam_2d_runtime::protocol::Operation;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("建图适配器不可用"))?
            .clone();
        let binding = adapter
            .lock()
            .planar
            .clone()
            .ok_or_else(|| Status::unavailable("二维拥有方不可用"))?;
        let worker_status=binding.client.status().await.map_err(Status::unavailable)?;
        if worker_status.localization && adapter.lock().snapshot().map_name==request.map_name {
            return self.continue_planar_mapping(request,cancellation).await;
        }
        let _operation=binding.operation.lock().await;
        let start_epoch = binding.fence.snapshot();
        if start_epoch.active {
            return Err(Status::failed_precondition("重定位期间不允许开始二维会话"));
        }
        let snapshot = {
            let mut projection = self.bridge.lock_explore_projection_owned().await;
            let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            self.ensure_running_admission("StartMapping").await?;
            if self.bridge.explore.blocks_mapping_mutation().await {
                return Err(Status::failed_precondition("探索仍持有地图会话"));
            }
            let admission = adapter
                .lock()
                .prepare_session_start_admission(&request.session_id, &request.map_name)
                .map_err(mapping_lifecycle_status)?;
            if let crate::mapping_adapter::MappingSessionStartAdmission::Replay(snapshot) =
                admission
            {
                if !adapter.lock().planar_ready {
                    return Err(Status::unavailable("二维会话尚未就绪"));
                }
                return Ok(Response::new(
                    self.mapping_state_with_receipt(snapshot, "二维会话已启动"),
                ));
            }
            self.activation_receipts
                .retire_for_mapping_change()
                .map_err(Status::resource_exhausted)?;
            let snapshot = adapter
                .lock()
                .start_session(&request.session_id, &request.map_name)
                .map_err(mapping_lifecycle_status)?;
            self.bridge
                .revoke_committed_map_projection_locked(&mut projection)
                .map_err(Status::unavailable)?;
            let mut state = self.bridge.state.write().await;
            state.map = Default::default();
            crate::lcm_runtime::reset_canonical_localization_locked(
                &mut state, binding.sensor_edge().as_ref());
            drop(state);
            *self.bridge.occupancy_shm_current.write().await = None;
            snapshot
        };
        let session = binding.session(
            &snapshot.session_id,
            &snapshot.map_name,
            snapshot.session_generation,
        );
        let result = binding.execute(Operation::Start { session }).await;
        let mut projection = self.bridge.lock_explore_projection_owned().await;
        let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
        let admission = self.ensure_running_admission("StartMapping").await;
        let failure = if *cancellation.borrow() {
            Some("二维会话请求已取消".to_string())
        } else if binding.fence.snapshot() != start_epoch {
            Some("二维会话初始化跨越重定位代际".into())
        } else {
            result
                .err()
                .or_else(|| admission.err().map(|e| e.to_string()))
        };
        if let Some(error) = failure {
            let _ = adapter.lock().force_stop_started_session_detached(
                &snapshot.session_id,
                snapshot.session_generation,
            );
            let _ = self
                .bridge
                .revoke_committed_map_projection_locked(&mut projection);
            drop(_lifecycle);
            drop(projection);
            let stopped = binding
                .execute(Operation::Stop {
                    session_id: snapshot.session_id,
                    session_generation: snapshot.session_generation,
                })
                .await;
            return Err(Status::unavailable(match stopped {
                Ok(_) => error,
                Err(stop) => format!("{error}；二维清理失败：{stop}"),
            }));
        }
        {
            let mut adapter = adapter.lock();
            adapter.planar_epoch = start_epoch.epoch;
            adapter.planar_ready = true;
        }
        Ok(Response::new(self.mapping_state_with_receipt(
            snapshot,
            "二维会话已启动，等待扫描",
        )))
    }
}

#[tonic::async_trait]
impl GeometryRuntime for GeometryRuntimeService {
    // ========================================================================
    // 控制面
    // ========================================================================

    async fn start(&self, _req: Request<StartRequest>) -> Result<Response<StartResult>, Status> {
        Ok(Response::new(StartResult {
            success: true,
            message: "已在运行".into(),
        }))
    }

    async fn stop(&self, _req: Request<StopRequest>) -> Result<Response<StopResult>, Status> {
        info!("收到 Stop 指令");
        self.perform_process_stop().await?;
        Ok(Response::new(StopResult { success: true }))
    }

    async fn get_status(&self, _req: Request<Empty>) -> Result<Response<RuntimeStatus>, Status> {
        let s = self.state.read().await;
        let diag = RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns);
        Ok(Response::new(RuntimeStatus {
            phase: diag.phase,
            message: s.message.clone(),
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            lcm_backend: diag.lcm_backend,
            supported_assigned_fine_archive_versions: vec![1, 2, 3, 4, 5],
        }))
    }

    async fn health_check(
        &self,
        _req: Request<HealthCheckRequest>,
    ) -> Result<Response<HealthCheckResponse>, Status> {
        let diag = self.diagnostics().await;
        // Phase 1 fail-fast + Phase 2 diagnostics 收敛:health_check 统一
        // 从 RuntimeDiagnostics 读 healthy + lcm_backend。
        // HealthCheck 调用方可从 `lcm_backend` 字段 grep
        // "LCM_BACKEND_UNAVAILABLE" 关键字做 CI gate。
        Ok(Response::new(HealthCheckResponse {
            healthy: diag.healthy,
            phase: diag.phase,
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            video_frame_sequence: 0,
            video_fps_estimate: 0,
            lcm_backend: diag.lcm_backend,
            geometry_pipeline: if self.bridge.mapping_adapter.as_ref().is_some_and(|a|a.lock().planar.is_some()) { "slam_2d" } else { "lio_3d" }.into(),
            runtime_generation: self.bridge.mapping_adapter.as_ref().and_then(|a|a.lock().planar.clone())
                .map(|b|b.client.identity().runtime_generation).unwrap_or_else(|| self.activation_receipts.runtime_generation()),
            planar_input_config: self.bridge.mapping_adapter.as_ref().and_then(|a|a.lock().planar.clone())
                .map(|b|serde_json::to_string(&b.settings.template).expect("已验证二维配置")).unwrap_or_default(),
            planar_unix_offset_ns: self.bridge.mapping_adapter.as_ref().and_then(|a|a.lock().planar.clone())
                .map(|b|b.settings.unix_offset_ns as i64).unwrap_or(0),
        }))
    }

    // ========================================================================
    // 定位
    // ========================================================================

    async fn get_current_pose(&self, _req: Request<PoseRequest>) -> Result<Response<Pose>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(Pose {
            position: Some(Vec3 {
                x: s.pose.x,
                y: s.pose.y,
                z: s.pose.z,
            }),
            orientation: Some(Quat {
                x: s.pose.qx,
                y: s.pose.qy,
                z: s.pose.qz,
                w: s.pose.qw,
            }),
            frame_id: s.pose.frame_id.clone(),
            timestamp_ns: s.pose.timestamp_ns,
        }))
    }

    type SubscribePoseStream = SubscriptionStream<PoseUpdate>;
    async fn subscribe_pose(
        &self,
        _req: Request<PoseRequest>,
    ) -> Result<Response<Self::SubscribePoseStream>, Status> {
        let state = self.bridge.state.clone();
        let graceful_rx = self
            .shutdown
            .as_ref()
            .map(|coordinator| coordinator.graceful_tx.subscribe());
        // 直接按消费需求读取最新规范状态，不增加轮询等待或历史帧发送队列。
        // 通知与首次快照之间的更新由接收器保留，关闭信号沿用其它订阅的来源。
        let updates = futures::stream::unfold(
            (None::<watch::Receiver<()>>, state, None, 0u64),
            |(mut changes, state, mut last_stamp, mut seq)| async move {
                loop {
                    if let Some(receiver) = changes.as_mut() {
                        if receiver.changed().await.is_err() {
                            return None;
                        }
                    }
                    let update = {
                        let s = state.read().await;
                        // 首次订阅和快照共用状态锁；锁等待本身也由外层关闭信号取消。
                        if changes.is_none() {
                            changes = Some(s.pose_changed.subscribe());
                        }
                        if Some(s.pose.timestamp_ns) == last_stamp {
                            continue;
                        }
                        last_stamp = Some(s.pose.timestamp_ns);
                        seq += 1;
                        PoseUpdate {
                            pose: Some(Pose {
                                position: Some(Vec3 {
                                    x: s.pose.x,
                                    y: s.pose.y,
                                    z: s.pose.z,
                                }),
                                orientation: Some(Quat {
                                    x: s.pose.qx,
                                    y: s.pose.qy,
                                    z: s.pose.qz,
                                    w: s.pose.qw,
                                }),
                                frame_id: s.pose.frame_id.clone(),
                                timestamp_ns: s.pose.timestamp_ns,
                            }),
                            sequence: seq,
                        }
                    };
                    return Some((Ok(update), (changes, state, last_stamp, seq)));
                }
            },
        );
        let stream: Self::SubscribePoseStream = Box::pin(futures::StreamExt::take_until(
            updates,
            wait_for_subscription_shutdown(graceful_rx),
        ));
        Ok(Response::new(stream))
    }

    async fn get_frame_tree(&self, _req: Request<Empty>) -> Result<Response<FrameTree>, Status> {
        let s = self.bridge.state.read().await;
        let transforms = s
            .latest_transform_entries()
            .into_iter()
            .map(|t| Transform {
                parent_frame: t.parent,
                child_frame: t.child,
                translation: Some(Vec3 {
                    x: t.tx,
                    y: t.ty,
                    z: t.tz,
                }),
                rotation: Some(Quat {
                    x: t.rx,
                    y: t.ry,
                    z: t.rz,
                    w: t.rw,
                }),
                timestamp_ns: t.stamp_ns,
            })
            .collect();
        Ok(Response::new(FrameTree { transforms }))
    }

    // ========================================================================
    // 地图
    // ========================================================================

    async fn start_mapping(
        &self,
        req: Request<StartMappingRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let request = req.into_inner();
        if request.session_id.trim().is_empty() || request.map_name.trim().is_empty() {
            return Err(Status::invalid_argument(
                "session_id and map_name must not be empty",
            ));
        }
        if !self.bridge.lcm_backend_available {
            return Err(Status::unavailable("mapping LCM backend is unavailable"));
        }

        let mapping_transition = tokio::time::timeout(
            MAPPING_TRANSITION_TIMEOUT,
            self.mapping_transition.clone().lock_owned(),
        )
        .await
        .map_err(|_| {
            Status::deadline_exceeded(
                "timed out joining the in-flight Mapping lifecycle transition",
            )
        })?;
        self.ensure_running_admission("StartMapping").await?;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        if adapter.lock().planar.is_some() {
            let service=self.clone();
            let (cancel_tx,cancel_rx)=watch::channel(false);
            let mut waiter=StartMappingRpcWaiter::new(cancel_tx);
            let task=self.bridge.register_planar_operation(async move {
                let _transition=mapping_transition;
                Ok(service.start_planar_mapping(request,cancel_rx).await)
            }).map_err(Status::unavailable)?;
            let result=task.await.map_err(|e|Status::internal(format!("二维会话任务异常：{e}")))?.map_err(Status::unavailable)?;
            waiter.complete();
                return result;
        }
        let current = adapter.lock().snapshot();
        if current.active
            || (current.phase == crate::mapping_adapter::MappingSessionPhase::Stopped
                && current.session_id == request.session_id)
        {
            let snapshot = adapter
                .lock()
                .start_session(&request.session_id, &request.map_name)
                .map_err(mapping_lifecycle_status)?;

            return Ok(Response::new(self.mapping_state_with_receipt(
                snapshot,
                "mapping session already active",
            )));
        }

        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before starting a new mapping session: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }

        #[cfg(test)]
        {
            let barrier = self.start_mapping_before_cloud_clear.lock().take();
            if let Some(barrier) = barrier {
                barrier.wait().await;
                barrier.wait().await;
            }
        }

        let projection_guard = self.bridge.lock_explore_projection_owned().await;
        let lifecycle_guard = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;

        let cloud_map_reservation = match self.bridge.reserve_cloud_map_session(true) {
            CloudMapSessionRegistration::Admitted(reservation) => reservation,
            CloudMapSessionRegistration::Rejected(admission) => {
                let detail = match admission {
                    CloudMapSessionAdmission::AdmissionClosed(detail)
                    | CloudMapSessionAdmission::ClearRejected(detail) => detail,
                    CloudMapSessionAdmission::WriterUnavailable => {
                        "cloud-map writer is unavailable".to_owned()
                    }
                    CloudMapSessionAdmission::Registered(_) => {
                        "cloud-map clear reservation returned an invalid admission".to_owned()
                    }
                };

                return Err(Status::unavailable(format!(
                    "StartMapping cloud-map clear registration failed: {detail}"
                )));
            }
        };

        if let Err(error) = self.ensure_running_admission("StartMapping").await {

            return Err(error);
        }

        if let Err(error) = self.activation_receipts.retire_for_mapping_change() {

            return Err(Status::resource_exhausted(error));
        }

        let snapshot = match adapter
            .lock()
            .start_session(&request.session_id, &request.map_name)
            .map_err(mapping_lifecycle_status)
        {
            Ok(snapshot) => snapshot,
            Err(error) => {

                return Err(error);
            }
        };
        let transition_owner = self.clone();
        let transition_task = tokio::spawn(async move {
            transition_owner
                .finish_started_mapping_transition(
                    adapter,
                    snapshot,
                    cloud_map_reservation,
                    projection_guard,
                    lifecycle_guard,
                    mapping_transition,
                )
                .await
        });
        let result = transition_task.await.map_err(|error| {
            Status::internal(format!(
                "owned StartMapping transition task failed: {error}"
            ))
        });
        result?
    }

    async fn stop_mapping(
        &self,
        req: Request<StopMappingRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let request = req.into_inner();
        let _mapping_transition =
            tokio::time::timeout(MAPPING_TRANSITION_TIMEOUT, self.mapping_transition.lock())
                .await
                .map_err(|_| {
                    Status::deadline_exceeded(
                        "timed out joining the in-flight Mapping lifecycle transition",
                    )
                })?;
        self.ensure_running_admission("StopMapping").await?;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let mut projection_guard = self.bridge.lock_explore_projection_owned().await;
        let lifecycle_guard = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before StopMapping: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        if let Err(error) = self.ensure_running_admission("StopMapping").await {
            return Err(error);
        }
        let transition = match adapter.lock().stop_session_detached(&request.session_id) {
            Ok(transition) => transition,
            Err(error) => {
                return Err(mapping_lifecycle_status(error));
            }
        };
        self.bridge.revoke_committed_map_projection_locked(&mut projection_guard)
            .map_err(Status::unavailable)?;
        #[cfg(test)]
        if self.stop_phase_a_after_detach.swap(false, Ordering::AcqRel) {
            if let Some(phase_a) = self
                .shutdown
                .as_ref()
                .and_then(|shutdown| shutdown.correction_phase_a.as_ref())
            {
                phase_a.begin();
            }
        }
        let receipt_retirement = if transition.snapshot.idempotent_replay {
            Ok(())
        } else {
            self.activation_receipts
                .retire_for_mapping_change()
                .map(|_| ())
        };
        let planar=adapter.lock().planar.clone();
        adapter.lock().planar_ready=false;
        drop(lifecycle_guard);
        drop(projection_guard);
        if let Some(binding)=planar {
            crate::planar_navigation::revoke(&self.bridge,&binding,&binding.session(
                &transition.snapshot.session_id,&transition.snapshot.map_name,transition.snapshot.session_generation)).await.map_err(Status::unavailable)?;
            binding.execute(nuwax_slam_2d_runtime::protocol::Operation::Stop {
                session_id:transition.snapshot.session_id.clone(),session_generation:transition.snapshot.session_generation,
            }).await.map_err(Status::unavailable)?;
        }
        let backend_retirement = finish_backend_retirement(
            &adapter,
            &self.bridge.mapping_lifecycle_lock,
            transition.retirement,
        )
        .await;
        if let Err(error) = receipt_retirement {
            let backend_detail = backend_retirement
                .err()
                .map(|backend| format!("; backend retirement also failed: {backend}"))
                .unwrap_or_default();
            return Err(Status::failed_precondition(format!(
                "activation receipt retirement failed after mapping detach: {error}{backend_detail}"
            )));
        }
        backend_retirement.map_err(mapping_lifecycle_status)?;
        let snapshot = transition.snapshot;
        let reason = request.reason.trim();
        Ok(Response::new(self.mapping_state_with_receipt(
            snapshot,
            if reason.is_empty() {
                "mapping session stopped".to_string()
            } else {
                format!("mapping session stopped: {reason}")
            },
        )))
    }

    async fn get_mapping_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        let snapshot = adapter.lock().snapshot();
        Ok(Response::new(self.mapping_state_with_receipt(
            snapshot,
            "mapping session state",
        )))
    }

    async fn transfer_graph_artifact(&self,request:Request<GraphArtifactTransferRequest>)->Result<Response<GraphArtifactTransferResponse>,Status> {
        self.ensure_running_admission("TransferGraphArtifact").await?;
        let store=self.graph_artifacts.clone();
        let request=request.into_inner();
        let receiver=self.bridge.register_planar_operation(async move {
            tokio::task::spawn_blocking(move||store.lock().transfer(request)).await.map_err(|e|e.to_string())?
        }).map_err(Status::unavailable)?;
        receiver.await.map_err(|_|Status::unavailable("二维制品保留任务未返回"))?
            .map(Response::new).map_err(Status::failed_precondition)
    }
    async fn export_mapping_backend_checkpoint(
        &self,
        _req: Request<ExportMappingBackendCheckpointRequest>,
    ) -> Result<Response<MappingBackendCheckpoint>, Status> {

        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        if adapter.lock().planar.is_some() {
            let service=self.clone();
            let receiver=self.bridge.register_planar_operation(async move {service.export_planar_checkpoint().await}).map_err(Status::unavailable)?;
            return receiver.await.map_err(|_|Status::unavailable("二维图导出保留任务未返回"))?.map(Response::new).map_err(Status::failed_precondition);
        }
        let owned_service = self.clone();
        tokio::spawn(async move {
        let bridge = owned_service.bridge.clone();
        let lease = {
            let _lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
            owned_service.ensure_running_admission("ExportMappingBackendCheckpoint")
                .await?;
            adapter
                .lock()
                .begin_backend_checkpoint_export()
                .map_err(Status::failed_precondition)?
        };
        let mut export_guard = MappingCheckpointExportGuard::new(adapter.clone(), lease);

        let deadline = tokio::time::Instant::now() + MAPPING_BACKEND_CHECKPOINT_TIMEOUT;
        let export_result = async {
            loop {
                if tokio::time::Instant::now() >= deadline {
                    return Err(Status::deadline_exceeded(
                        "mapping backend compute did not drain before checkpoint flush",
                    ));
                }
                let lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                owned_service.ensure_running_admission("ExportMappingBackendCheckpoint")
                    .await?;
                let drained = {
                    adapter
                        .lock()
                        .backend_checkpoint_client_if_drained(export_guard.lease())
                        .map_err(Status::failed_precondition)?
                        .is_some()
                };
                if !drained {
                    drop(lifecycle_guard);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    continue;
                }
                let prepared_at_ns = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_err(|_| Status::internal("system clock is before Unix epoch"))?
                    .as_nanos();
                let prepared_at_ns = u64::try_from(prepared_at_ns)
                    .map_err(|_| Status::internal("checkpoint flush time exceeds u64"))?;
                let prepared = {
                    adapter
                        .lock()
                        .prepare_backend_checkpoint_flush(export_guard.lease(), prepared_at_ns)
                        .map_err(Status::failed_precondition)?
                };
                drop(lifecycle_guard);
                let Some(prepared) = prepared else {
                    break;
                };
                let token = prepared.token.clone();
                let receiver = match prepared.worker.flush(prepared.token) {
                    Ok(receiver) => receiver,
                    Err(error) => {
                        let _lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                        adapter.lock().fail_backend_compute(&token);
                        return Err(Status::unavailable(format!(
                            "mapping backend checkpoint flush submission failed: {error:?}"
                        )));
                    }
                };
                let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                let completion = match tokio::time::timeout(remaining, receiver).await {
                    Ok(Ok(completion)) => completion,
                    Ok(Err(_)) | Err(_) => {
                        let _lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                        adapter.lock().fail_backend_compute(&token);
                        return Err(Status::deadline_exceeded(
                            "mapping backend checkpoint flush timed out or closed",
                        ));
                    }
                };
                let preparation = adapter.lock().prepare_backend_completion(completion);
                let completion = tokio::task::spawn_blocking(move || preparation.compute()).await
                    .map_err(|error| Status::internal(format!("mapping checkpoint preparation failed: {error}")))?;
                let mut projection_guard = bridge.lock_explore_projection_owned().await;
                let lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                let mut shared = bridge.state.write().await;
                let committed_result = {
                    let mut adapter_guard = adapter.lock();
                    adapter_guard.commit_prepared_backend_completion(completion, &mut shared)
                };
                drop(shared);
                let committed = committed_result
                    .map_err(Status::failed_precondition)?
                    .ok_or_else(|| {
                        Status::failed_precondition(
                            "mapping backend checkpoint flush completion became stale",
                        )
                    })?;
                let crate::mapping_adapter::CommittedMappingCandidate {
                    token,
                    resolution,
                    accepted_map_points,
                    event,
                    session_generation,
                    source_timestamp_ns,
                    occupancy_receipt,
                    ..
                } = committed;
                let (current_session, current_frame) = {
                    let adapter = adapter.lock();
                    (adapter.snapshot(), adapter.global_frame_generation())
                };
                let owner_bound = {
                    let mut writer = bridge.cloud_map_writer.lock();
                    match writer.as_mut() {
                        Some(writer) => writer.bind_fine_map_owner(current_frame, bridge.nav.producer_runtime_generation()),
                        None => Ok(()),
                    }
                };
                let cloud_result = owner_bound.and_then(|()|
                    accept_mapping_cloud_map_for_backend_candidate(
                        &accepted_map_points,
                        &event,
                        &occupancy_receipt,
                        session_generation,
                        source_timestamp_ns,
                        &current_session,
                        &bridge.cloud_map_writer,
                    ));
                let resolution_result = adapter
                    .lock()
                    .resolve_backend_completion(&token, resolution, cloud_result.is_ok());
                if cloud_result.is_err() || resolution_result.is_err() {
                    let mut reasons = Vec::new();
                    if let Err(error) = cloud_result { reasons.push(error); }
                    if let Err(error) = resolution_result {
                        reasons.push(format!("mapping backend checkpoint flush resolution failed: {error}"));
                    }
                    let mut reason = reasons.join("; ");
                    let retirement = adapter
                        .lock()
                        .stop_session_detached(&token.session_id)
                        .ok()
                        .and_then(|transition| transition.retirement);
                    bridge
                        .revoke_committed_map_projection_locked(&mut projection_guard)
                        .map_err(Status::unavailable)?;
                    drop(lifecycle_guard);
                    drop(projection_guard);
                    if let Err(error) = finish_backend_retirement(
                        &adapter,
                        &bridge.mapping_lifecycle_lock,
                        retirement,
                    )
                    .await
                    {
                        reason = format!("{reason}; retirement failed: {error}");
                    }
                    return Err(Status::failed_precondition(reason));
                }
                let cloud_commit = cloud_result.expect("point-cloud commit failure already retired");
                if let Some(event) = event {
                    let _ = bridge.map_revision_tx.send(event);
                }
                if let Some(cloud_commit) = cloud_commit {
                    {
                        let mut shared = bridge.state.write().await;
                        if shared.cloud_map_revision < cloud_commit.notification.revision {
                            shared.cloud_map_revision = cloud_commit.notification.revision;
                        }
                    }
                    let _ = bridge.cloud_map_tx.send(cloud_commit.notification.clone());
                    let map_frame_id = adapter.lock().configured_map_frame_id().to_owned();
                    publish_committed_global_map_in_frame(
                        &bridge,
                        &cloud_commit,
                        &map_frame_id,
                    )
                    .await
                    .map_err(|error| {
                        Status::internal(format!(
                            "mapping backend checkpoint flush typed /global_map publication failed: {error}"
                        ))
                    })?;
                }
                break;
            }

            loop {
                if tokio::time::Instant::now() >= deadline {
                    return Err(Status::deadline_exceeded(
                        "mapping backend compute did not drain before checkpoint deadline",
                    ));
                }
                let lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                owned_service.ensure_running_admission("ExportMappingBackendCheckpoint")
                    .await?;
                let drained = {
                    let adapter = adapter.lock();
                    let client = adapter
                        .backend_checkpoint_client_if_drained(export_guard.lease())
                        .map_err(Status::failed_precondition)?;
                    if let Some(client) = client {
                        let snapshot = adapter.snapshot();
                        let receipt =
                            adapter.latest_occupancy_commit_receipt().ok_or_else(|| {
                                Status::failed_precondition(
                                "mapping backend checkpoint requires a canonical occupancy commit",
                            )
                            })?;
                        let raw_map = adapter.raw_map_snapshot().ok_or_else(|| {
                            Status::failed_precondition(
                                "mapping backend checkpoint requires canonical raw occupancy",
                            )
                        })?;
                        Some((snapshot, adapter.backend_kind(), client, receipt, raw_map))
                    } else {
                        None
                    }
                };
                let Some((snapshot, backend_kind, client, occupancy_receipt, raw_map)) = drained
                else {
                    drop(lifecycle_guard);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    continue;
                };
                let (canonical_occupancy_revision, canonical_occupancy_timestamp_ns) = {
                    let shared = bridge.state.read().await;
                    (shared.map.revision, shared.map.timestamp_ns)
                };
                let (cloud_snapshot, cloud_receipt, fine_snapshot, dense_snapshot) = {
                    let writer = bridge.cloud_map_writer.lock();
                    let writer = writer.as_ref().ok_or_else(|| {
                        Status::failed_precondition(
                            "mapping backend checkpoint requires the canonical cloud-map writer",
                        )
                    })?;
                    let snapshot = writer.committed_snapshot().ok_or_else(|| {
                        Status::failed_precondition(
                            "mapping backend checkpoint requires a canonical cloud-map snapshot",
                        )
                    })?;
                    let receipt = writer.last_cloud_receipt().ok_or_else(|| {
                        Status::failed_precondition(
                            "mapping backend checkpoint requires a canonical cloud commit",
                        )
                    })?;
                    (snapshot, receipt, writer.committed_fine_cloud(), writer.committed_dense_cloud().map_err(Status::failed_precondition)?)
                };
                if !snapshot.active
                    || snapshot.session_id.trim().is_empty()
                    || snapshot.session_generation == 0
                    || snapshot.revision == 0
                    || snapshot.revision != canonical_occupancy_revision
                    || raw_map.revision != snapshot.revision
                    || raw_map.timestamp_ns != canonical_occupancy_timestamp_ns
                    || occupancy_receipt.session_id != snapshot.session_id
                    || occupancy_receipt.session_generation != snapshot.session_generation
                    || occupancy_receipt.occupancy_revision != snapshot.revision
                    || occupancy_receipt.source_timestamp_ns != canonical_occupancy_timestamp_ns
                    || occupancy_receipt.backend_kind != backend_kind
                    || occupancy_receipt.backend_generation != client.backend_generation()
                {
                    return Err(Status::failed_precondition(
                        "mapping backend checkpoint canonical occupancy receipt is inconsistent",
                    ));
                }
                validate_occupancy_grid(&raw_map)
                    .map_err(OccupancyGridValidationError::into_status)?;
                if cloud_snapshot.session_generation != snapshot.session_generation
                    || cloud_snapshot.revision != cloud_receipt.cloud_revision
                    || cloud_snapshot.point_count != cloud_receipt.point_count
                    || cloud_receipt.session_generation != snapshot.session_generation
                {
                    return Err(Status::failed_precondition(
                        "mapping backend checkpoint canonical cloud receipt is inconsistent",
                    ));
                }
                let occupancy_receipt = occupancy_commit_receipt_proto(&occupancy_receipt);
                let cloud_receipt = cloud_commit_receipt_proto(&cloud_receipt);
                validate_mapping_checkpoint_receipts(
                    &snapshot.session_id,
                    snapshot.session_generation,
                    backend_kind.stable_name(),
                    Some(&occupancy_receipt),
                    Some(&cloud_receipt),
                )
                .map_err(Status::failed_precondition)?;
                let canonical_occupancy_checkpoint = occupancy_checkpoint_proto(&raw_map);
                let canonical_cloud_checkpoint =
                    cloud_checkpoint_proto(cloud_snapshot, raw_map.frame_id.clone());
                let receiver = client.export_checkpoint().map_err(|error| {
                    Status::unavailable(format!(
                        "mapping backend checkpoint submission failed: {error:?}"
                    ))
                })?;
                let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
                let payload = tokio::time::timeout(remaining, receiver)
                    .await
                    .map_err(|_| {
                        Status::deadline_exceeded("mapping backend checkpoint export timed out")
                    })?
                    .map_err(|_| {
                        Status::internal("mapping backend checkpoint worker closed its response")
                    })?
                    .map_err(Status::failed_precondition)?;
                let created_at_ns = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_err(|_| Status::internal("system clock is before Unix epoch"))?
                    .as_nanos();
                let created_at_ns = u64::try_from(created_at_ns)
                    .map_err(|_| Status::internal("checkpoint creation time exceeds u64"))?;
                let fine_root = fine_snapshot.as_ref().map(|fine| Arc::clone(fine.manifest()));
                if let Some(root) = &fine_root {
                    let domain = &root.domain;
                    if domain.session != snapshot.session_id || domain.generation != snapshot.session_generation
                        || domain.frame_generation != adapter.lock().global_frame_generation()
                        || domain.producer_generation != bridge.nav.producer_runtime_generation()
                        || root.revision != cloud_receipt.cloud_revision || root.source_timestamp_ns != cloud_receipt.source_timestamp_ns {
                        return Err(Status::failed_precondition("fine checkpoint root differs from frozen canonical source"));
                    }
                }
                // Assign once after the existing barrier froze receipts, payload and immutable byte owners.
                let checkpoint_id = uuid::Uuid::new_v4().simple().to_string();

                // 已拿到同一屏障内的不可变快照，归档不持生命周期锁。
                drop(lifecycle_guard);
                let dense_cloud_artifact = if let Some(dense) = dense_snapshot {
                    let store = owned_service.graph_artifacts.clone();
                    let frame_id = raw_map.frame_id.clone();
                    let member = checkpoint_member_identity(&checkpoint_id, CheckpointMemberRole::DenseCloud,
                        &occupancy_receipt, &cloud_receipt, &frame_id, None);
                    Some(tokio::task::spawn_blocking(move || {
                        let (file, size) = dense.archive(&frame_id)?;
                        store.lock().insert(file, size, member)
                    }).await.map_err(|e| Status::internal(e.to_string()))?.map_err(Status::internal)?)
                } else { None };
                let fine_map_artifact = if let (Some(fine), Some(root)) = (fine_snapshot, fine_root) {
                    let store = owned_service.graph_artifacts.clone();
                    let member = checkpoint_member_identity(&checkpoint_id, CheckpointMemberRole::FineMap,
                        &occupancy_receipt, &cloud_receipt, &raw_map.frame_id, Some(&root.domain));
                    Some(tokio::task::spawn_blocking(move || {
                        let (file, size) = nuwax_map_chunks::archive_file(&root, &fine.repository)?;
                        store.lock().insert(file, size, member)
                    }).await.map_err(|e| Status::internal(e.to_string()))?.map_err(Status::internal)?)
                } else { None };
                let checkpoint = MappingBackendCheckpoint {
                    schema_version:
                        crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
                    backend_kind: backend_kind.stable_name().to_owned(),
                    source_session_generation: snapshot.session_generation,

                    payload_size: u64::try_from(payload.len()).map_err(|_| {
                        Status::internal("mapping backend checkpoint size exceeds u64")
                    })?,
                    created_at_ns,
                    payload,
                    source_session_id: snapshot.session_id,
                    occupancy_receipt: Some(occupancy_receipt),
                    cloud_receipt: Some(cloud_receipt),
                    canonical_occupancy_checkpoint: Some(canonical_occupancy_checkpoint),
                    canonical_cloud_checkpoint: Some(canonical_cloud_checkpoint),
                    graph_artifact: None,
                    target_kind: String::new(),
                    fine_map_artifact,
                    dense_cloud_artifact,
                    checkpoint_id,
                    backend_payload_schema_version: 0,
                    backend_identity: None, occupancy_identity: None,
                };
                return assemble_mapping_checkpoint_export(checkpoint).map_err(Status::internal);
            }
        }
        .await;

        {
            let _lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
            export_guard.finish();
        }
        export_result.map(Response::new)
        })
        .await
        .map_err(|error| Status::internal(format!("mapping checkpoint export task failed: {error}")))?
    }

    async fn activate_map(
        &self,
        req: Request<ActivateMapRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let service = self.clone();
        let request = req.into_inner();
        if self.bridge.mapping_adapter.as_ref().is_some_and(|a|a.lock().planar.is_some()) {
            let receiver=self.bridge.register_planar_operation(async move {Ok(activate_map_r1(service,request).await)})
                .map_err(Status::unavailable)?;
            return receiver.await.map_err(|_|Status::unavailable("二维激活保留任务未返回"))?.map_err(Status::unavailable)?;
        }
        tokio::spawn(async move { activate_map_r1(service, request).await })
            .await
            .map_err(|error| {
                Status::internal(format!(
                    "mapping checkpoint activation task failed: {error}"
                ))
            })?
    }
    async fn relocalize(
        &self,
        req: Request<RelocalizeRequest>,
    ) -> Result<Response<RelocalizeResult>, Status> {
        // Schema 2 is admitted by the exact typed actor, never legacy hashing.
        crate::relocalization_identity::validate_live_request_boundary(req.get_ref())
            .map_err(Status::failed_precondition)?;
        let ingress = self
            .relocalization_ingress
            .as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        if matches!(req.get_ref().assigned_target, Some(relocalize_request::AssignedTarget::AssignedPlanarV2(_))) {
            let lifecycle_guard = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            let request=req.into_inner();let task_request=request.clone();let service=self.clone();
            let work:crate::correction_coordinator::PlanarCommandWork=Box::new(move|context|Box::pin(async move {
                let bridge=service.bridge.clone();
                let receiver=bridge.register_planar_operation(async move {Ok(service.run_planar_relocalization(task_request,context).await)});
                match receiver {
                    Ok(receiver)=>match receiver.await {
                        Ok(Ok(outcome))=>outcome,
                        _=>crate::correction_coordinator::PlanarCommandOutcome{disposition:RelocalizeDisposition::OwnerFailed,detail:"二维重定位保留任务未返回".into(),correction_revision:0,evidence:None},
                    },
                    Err(e)=>crate::correction_coordinator::PlanarCommandOutcome{disposition:RelocalizeDisposition::OwnerStopped,detail:e,correction_revision:0,evidence:None},
                }
            }));
            let waiter=ingress.admit_planar(request,lifecycle_guard,work).await.map_err(relocalization_command_status)?;
            return waiter.wait().await.map(Response::new).map_err(relocalization_command_status);
        }
        let (lifecycle_guard, compute_barrier) = self.drain_mapping_before_relocalization().await?;
        let waiter: crate::correction_coordinator::RelocalizationCommandWaiter = ingress
            .admit_with_lifecycle_guard(req.into_inner(), lifecycle_guard)
            .await
            .map_err(relocalization_command_status)?;
        // 原准入拥有方已在生命周期锁内切换模式，旧批次已经提交。
        drop(compute_barrier);
        waiter
            .wait()
            .await
            .map(Response::new)
            .map_err(relocalization_command_status)
    }

    async fn get_assigned_localization_sample(
        &self, req: Request<RelocalizeRequest>,
    ) -> Result<Response<AssignedLocalizationSampleV2>, Status> {
        self.ensure_running_admission("GetAssignedLocalizationSample").await?;
        let request = crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2::admit_retained(req.into_inner())
            .map_err(Status::invalid_argument)?;
        let ingress = self.relocalization_ingress.as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let query = crate::relocalization_terminal_v2::AssignedRelocalizationTerminalQueryV2::admit(
            GetRelocalizationTerminalRequest {schema_version:2, request_id:request.request_id().into(),
                assigned_request_v2:Some(request.to_wire()), ..Default::default()})
            .map_err(Status::invalid_argument)?;
        // The retained actor query must finish before taking its lifecycle fence.
        let terminal = match ingress.terminal_snapshot_assigned(&query).await
            .map_err(Status::failed_precondition)?.terminal {
            crate::correction_coordinator::RelocalizationTerminalSnapshot::AssignedTerminal(terminal) => terminal,
            _ => return Err(Status::failed_precondition("ASSIGNED_SAMPLE_TERMINAL_UNAVAILABLE")),
        };
        let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("GetAssignedLocalizationSample").await?;
        if request.is_planar() {
            return self.assigned_planar_localization_sample(&terminal).await.map(Response::new).map_err(Status::failed_precondition);
        }
        let applier = self.canonical_correction.as_ref()
            .ok_or_else(|| Status::failed_precondition("ASSIGNED_SAMPLE_CANONICAL_OWNER_ABSENT"))?;
        let target = self.activation_receipts.current_sealed_assigned_target_snapshot()
            .filter(|snapshot| request.matches_target(&snapshot.target_receipt)
                && self.activation_receipts.assigned_actor_available())
            .ok_or_else(|| Status::failed_precondition("ASSIGNED_SAMPLE_ACTIVATION_REPLACED"))?;
        let sample = applier.assigned_localization_sample(&terminal).await
            .map_err(Status::failed_precondition)?;
        // Phase A can close independently of the mapping lifecycle lock.
        if !self.activation_receipts.assigned_actor_available()
            || !self.activation_receipts.current_sealed_assigned_target_snapshot()
                .is_some_and(|current| current.activation_identity == target.activation_identity
                    && current.target_receipt == target.target_receipt
                    && current.checkpoint_owner.shares_source_with(&target.checkpoint_owner)) {
            return Err(Status::failed_precondition("ASSIGNED_SAMPLE_ACTIVATION_REPLACED"));
        }
        Ok(Response::new(sample))
    }

    async fn get_relocalization_terminal(&self, req: Request<GetRelocalizationTerminalRequest>)
        -> Result<Response<GetRelocalizationTerminalResponse>, Status> {
        let query = crate::relocalization_terminal_v2::AssignedRelocalizationTerminalQueryV2::admit(req.into_inner())
            .map_err(Status::invalid_argument)?;
        let ingress = self.relocalization_ingress.as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let snapshot = ingress.terminal_snapshot_assigned(&query).await.map_err(Status::failed_precondition)?;
        let (status, terminal) = match snapshot.terminal {
            crate::correction_coordinator::RelocalizationTerminalSnapshot::AssignedTerminal(terminal) =>
                (RelocalizationTerminalStatus::Terminal, Some(terminal)),
            crate::correction_coordinator::RelocalizationTerminalSnapshot::Pending => (RelocalizationTerminalStatus::Pending, None),
            crate::correction_coordinator::RelocalizationTerminalSnapshot::NotFound => (RelocalizationTerminalStatus::NotFound, None),
            crate::correction_coordinator::RelocalizationTerminalSnapshot::IdentityConflict => (RelocalizationTerminalStatus::IdentityConflict, None),
        };
        Ok(Response::new(query.response(status, terminal.as_ref(), Some(snapshot.ledger_observation))))
    }

    async fn get_occupancy_grid(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridMeta>, Status> {
        let map = self.bridge.state.read().await.map.clone();
        validate_occupancy_grid(&map).map_err(OccupancyGridValidationError::into_status)?;
        if self.bridge.occupancy_shm.is_some() {
            await_occupancy_shm_revision(&self.bridge, map.revision).await?;
        }
        Ok(Response::new(occupancy_grid_meta(&map)))
    }

    async fn get_occupancy_grid_data(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridData>, Status> {
        let map = self.bridge.state.read().await.map.clone();
        validate_occupancy_grid(&map).map_err(OccupancyGridValidationError::into_status)?;
        if self.bridge.occupancy_shm.is_some() {
            await_occupancy_shm_revision(&self.bridge, map.revision).await?;
        }
        Ok(Response::new(OccupancyGridData {
            meta: Some(occupancy_grid_meta(&map)),
            data: owned_occupancy_bytes(map.data),
        }))
    }

    async fn get_raw_occupancy_grid_data(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridData>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?;
        let raw_map = adapter
            .lock()
            .raw_map_snapshot()
            .ok_or_else(|| Status::failed_precondition("physical occupancy grid is unavailable"))?;
        validate_occupancy_grid(&raw_map).map_err(OccupancyGridValidationError::into_status)?;
        Ok(Response::new(OccupancyGridData {
            meta: Some(occupancy_grid_meta(&raw_map)),
            data: owned_occupancy_bytes(raw_map.data),
        }))
    }

    async fn get_occupancy_grid_shm(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ShmRef>, Status> {
        // SHM sidecar 只有 map-revision fanout task 可写。本 RPC 只返回最新一次
        // 成功发布的不可变引用，size 是 mmap 文件长度而非 occupancy payload 长度。
        if self.bridge.occupancy_shm.is_none() {
            return Err(Status::unavailable(
                "occupancy_shm 未初始化 (LCM backend 不可用,导致 SHM 写入被禁用)",
            ));
        }
        let map = self.bridge.state.read().await.map.clone();
        validate_occupancy_grid(&map).map_err(OccupancyGridValidationError::into_status)?;
        let snapshot = await_occupancy_shm_revision(&self.bridge, map.revision).await?;
        Ok(Response::new(
            occupancy_shm_ref(snapshot).map_err(|status| *status)?,
        ))
    }

    type SubscribeMapUpdatesStream = SubscriptionStream<MapUpdateNotification>;
    async fn subscribe_map_updates(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeMapUpdatesStream>, Status> {
        let mapping_adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let expected_map_frame_id = mapping_adapter.lock().configured_map_frame_id().to_string();
        let mapping_lifecycle_lock = self.bridge.mapping_lifecycle_lock.clone();
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut map_revision_rx = self.bridge.subscribe_map_revisions();
        let occupancy_shm_current = self.bridge.occupancy_shm_current.clone();
        let stream =
            spawn_subscription_stream(self.shutdown.as_ref(), tx.clone(), rx, async move {
                let mut last_generation = 0u64;
                let mut last_rev = 0u64;
                let initial = map_revision_rx.borrow_and_update().clone();
                if let Some(event) = initial {
                    if !map_revision_claim_is_current(
                        &event,
                        &mapping_adapter,
                        &mapping_lifecycle_lock,
                    )
                    .await
                    {
                        return;
                    }
                    last_generation = event.session_generation;
                    last_rev = event.revision;
                    if !forward_map_revision(
                        &tx,
                        event,
                        &occupancy_shm_current,
                        &expected_map_frame_id,
                    )
                    .await
                    {
                        return;
                    }
                }

                while map_revision_rx.changed().await.is_ok() {
                    let Some(event) = map_revision_rx.borrow_and_update().clone() else {
                        continue;
                    };
                    if event.session_generation < last_generation
                        || (event.session_generation == last_generation
                            && event.revision <= last_rev)
                    {
                        continue;
                    }
                    if !map_revision_claim_is_current(
                        &event,
                        &mapping_adapter,
                        &mapping_lifecycle_lock,
                    )
                    .await
                    {
                        continue;
                    }
                    last_generation = event.session_generation;
                    last_rev = event.revision;
                    if !forward_map_revision(
                        &tx,
                        event,
                        &occupancy_shm_current,
                        &expected_map_frame_id,
                    )
                    .await
                    {
                        break;
                    }
                }
            });
        Ok(Response::new(stream))
    }

    type SubscribeCloudMapUpdatesStream = SubscriptionStream<CloudMapShmRef>;
    async fn subscribe_cloud_map_updates(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeCloudMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.cloud_map_tx.subscribe();
        let stream =
            spawn_subscription_stream(self.shutdown.as_ref(), tx.clone(), rx, async move {
                while let Ok(notif) = broadcast_rx.recv().await {
                    let shm_ref = match cloud_map_shm_ref(&notif) {
                        Ok(reference) => reference,
                        Err(status) => {
                            let _ = tx.send(Err(*status)).await;
                            break;
                        }
                    };
                    let msg = CloudMapShmRef {
                        shm_ref: Some(shm_ref),
                        point_count: notif.point_count,
                        voxel_size: notif.voxel_size,
                    };
                    if tx.send(Ok(msg)).await.is_err() {
                        break;
                    }
                }
            });
        Ok(Response::new(stream))
    }

    // ========================================================================
    // 导航兼容
    // ========================================================================
    //
    // Phase 4:`/nav/goal` 与 `/nav/cancel` publisher 已由 nav_runtime.rs 接管
    // (`publish_goal_frame` / `publish_cancel_frame`),通过 LcmTransport.publish
    // 推到 LCM typed channel。本节保留 setter RPC 入口供 Python/前端调用。
    // 删除 Phase 3 的 "UNIMPLEMENTED" placeholder 注释 —— readiness 与状态机
    // 都在 nav_runtime.rs 内真实执行(`set_goal` / `cancel`)。
    //
    // Phase 7:`SetNavigationGoal` 在 real production profile 且 cmd_vel_sink=false
    // 时被 nav_runtime::set_goal 拒绝;readiness 全量字段由
    // `mapping_adapter::MappingReadiness` / `nav_runtime::NavigationReadinessDetail`
    // / `explore_runtime::ExplorationReadinessDetail` 暴露,见
    // `docs/.../11_readiness_收敛.md`。

    async fn set_navigation_goal(
        &self,
        req: Request<NavigationGoal>,
    ) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime。流程:
        // 1) 归一化 NavigationGoal -> SetGoalInput(自动生成 UUID 当 goal_id 空)
        // 2) 调 set_goal():readiness 校验 -> 仅 publish /nav/goal typed frame
        //    -> 返回 ACCEPTED；规划/跟随/cmd_vel 全部归真实 stack 所有
        // 3) 把 NavState 同步到 SharedState.nav_state + broadcast 通道
        let ng = req.get_ref();
        let pose = ng.pose.as_ref();
        let pos = pose.and_then(|p| p.position.as_ref());
        let orient = pose.and_then(|p| p.orientation.as_ref());
        let target = PoseState {
            x: pos.map(|v| v.x).unwrap_or(0.0),
            y: pos.map(|v| v.y).unwrap_or(0.0),
            z: pos.map(|v| v.z).unwrap_or(0.0),
            qx: orient.map(|v| v.x).unwrap_or(0.0),
            qy: orient.map(|v| v.y).unwrap_or(0.0),
            qz: orient.map(|v| v.z).unwrap_or(0.0),
            qw: orient.map(|v| v.w).unwrap_or(1.0),
            frame_id: pose.map(|p| p.frame_id.clone()).unwrap_or_default(),
            timestamp_ns: pose.map(|p| p.timestamp_ns).unwrap_or(now_ns()),
        };
        let goal_id = ensure_goal_id(&ng.goal_id);
        let owner_kind = match ng.owner_kind {
            0 | 1 => nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            _ => {
                return Err(Status::invalid_argument(
                    "public SetNavigationGoal cannot claim exploration ownership",
                ));
            }
        };
        let parent_request_id = if ng.parent_request_id.trim().is_empty() {
            goal_id.clone()
        } else {
            ng.parent_request_id.trim().to_string()
        };
        let frame_id = if ng.frame_id.is_empty() {
            self.bridge
                .mapping_adapter
                .as_ref()
                .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
                .lock()
                .configured_map_frame_id()
                .to_string()
        } else {
            ng.frame_id.clone()
        };
        let motion_admission = MotionAdmissionProof::try_from_wire(
            &ng.motion_guard_boot_nonce,
            ng.motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let allow_reverse = nuwax_nav_types_messages::GoalOptions::resolve_reverse_request(
            ng.allow_reverse, ng.use_profile_reverse_default,
            self.bridge.nav.configured_profile() == crate::nav_runtime::PROFILE_MLS3D,
        ).map_err(Status::invalid_argument)?;
        let input = SetGoalInput {
            options: nuwax_nav_types_messages::GoalOptions {allow_reverse,position_tolerance_m:ng.position_tolerance_m.unwrap_or(0.35),yaw_tolerance_rad:ng.yaw_tolerance_rad.unwrap_or(0.3)},
            goal_id: goal_id.clone(),
            owner_kind,
            parent_request_id,
            goal_generation: ng.goal_generation,
            motion_admission,
            frame_id,
            target,
        };
        self.ensure_running_admission("SetNavigationGoal").await?;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "public navigation cannot start while exploration owns admission: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        let outcome = set_goal(&self.bridge, &self.bridge.nav, input).await;
        let update = NavigationUpdate::publish_local_set_goal_outcome(
            &outcome, &self.bridge.nav, &self.bridge.state, &self.bridge.nav_state_tx,
        ).await;
        Ok(Response::new(nav_state_to_proto(update)))
    }

    async fn get_navigation_map_block(
        &self, request: Request<NavigationMapBlockRequest>,
    ) -> Result<Response<NavigationMapBlockResponse>, Status> {
        self.ensure_running_admission("GetNavigationMapBlock").await?;
        let request = request.into_inner();
        if !matches!(request.schema_version, 2 | 3) {
            return Err(Status::invalid_argument("unsupported fine-map block protocol version"));
        }
        if (request.schema_version == 2 && (request.offset != 0 || request.max_bytes != 0))
            || (request.schema_version == 3 && (request.max_bytes == 0
                || request.max_bytes as usize > nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES
                || request.offset as usize >= nuwax_map_chunks::MAX_BLOCK_BYTES)) {
            return Err(Status::invalid_argument("invalid fine-map block range"));
        }
        let wire_root = request.root.as_ref().ok_or_else(|| Status::invalid_argument("missing retained root"))?;
        let wire_domain = wire_root.domain.as_ref().ok_or_else(|| Status::invalid_argument("missing root domain"))?;
        let root = nuwax_map_chunks::RootIdentity {
            domain: nuwax_map_chunks::Domain {
                session: wire_domain.session.clone(), generation: wire_domain.generation,
                frame_generation: wire_domain.frame_generation, producer_generation: wire_domain.producer_generation,
            }, revision: wire_root.revision,
        };
        root.domain.validate().map_err(Status::invalid_argument)?;
        if root.revision == 0 { return Err(Status::invalid_argument("zero root revision")); }
        let wire_block = request.block.as_ref().ok_or_else(|| Status::invalid_argument("missing block reference"))?;
        let identity = wire_block.object.as_ref().ok_or_else(|| Status::invalid_argument("missing block object identity"))?;
        let object = match identity.kind {
            1 if identity.legacy_alias.is_empty() => nuwax_map_chunks::ObjectIdentity::Assigned(
                nuwax_map_chunks::ObjectId(identity.object_id.as_slice().try_into()
                    .map_err(|_| Status::invalid_argument("assigned object identity must contain 16 bytes"))?)),
            2 if identity.object_id.is_empty() => nuwax_map_chunks::ObjectIdentity::Legacy(
                nuwax_map_chunks::LegacyAlias(identity.legacy_alias.as_slice().try_into()
                    .map_err(|_| Status::invalid_argument("legacy object alias must contain 32 bytes"))?)),
            _ => return Err(Status::invalid_argument("invalid tagged object identity")),
        };
        object.validate().map_err(Status::invalid_argument)?;
        let reference = nuwax_map_chunks::BlockRef {
            key: wire_block.key.as_slice().try_into().map_err(|_| Status::invalid_argument("block key requires three coordinates"))?,
            object, block_revision: wire_block.block_revision, geometry_revision: wire_block.geometry_revision,
            points: wire_block.points,
        };
        // 每段仍由原任务拥有方固定同一保留根，只读取所需范围，不积累下载会话。
        let range = (request.schema_version == 3)
            .then_some((request.offset as usize, request.max_bytes as usize));
        let receiver = self.bridge.read_navigation_map_block(&root, &reference, range).map_err(Status::failed_precondition)?;
        let read = receiver.await.map_err(|e| Status::unavailable(e.to_string()))?.map_err(Status::not_found)?;
        let nuwax_map_chunks::ObjectRangeRead::Data { bytes: data, total_bytes } = read else {
            return Err(Status::invalid_argument("fine-map block offset exceeds object"));
        };
        Ok(Response::new(NavigationMapBlockResponse {
            schema_version: request.schema_version, root: request.root, block: request.block, data,
            offset: request.offset, total_bytes: if request.schema_version == 3 { total_bytes as u32 } else { 0 },
        }))
    }

    async fn get_dense_cloud_chunk(
        &self, request: Request<DenseCloudChunkRequest>,
    ) -> Result<Response<DenseCloudChunk>, Status> {
        self.ensure_running_admission("GetDenseCloudChunk").await?;
        let request = request.into_inner();
        if request.max_points == 0 || request.max_points > crate::dense_cloud::CHUNK_POINTS
            || request.expected_epoch.len() > 128 || (request.expected_epoch.is_empty() && request.offset_points != 0) {
            return Err(Status::invalid_argument("完整点云分块请求非法"));
        }
        let (snapshot, base, frame_id) = {
            let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            self.ensure_running_admission("GetDenseCloudChunk").await?;
            let frame_id = self.bridge.state.read().await.map.frame_id.clone();
            let mut writer = self.bridge.cloud_map_writer.lock();
            let writer = writer.as_mut().ok_or_else(|| Status::failed_precondition("三维点云拥有方未就绪"))?;
            let captured = if request.allow_delta {
                writer.dense_cloud_update_for_read(&request.expected_epoch, request.offset_points)
            } else {
                writer.dense_cloud_for_read(&request.expected_epoch, request.offset_points)
                    .map(|value| value.map(|snapshot| (snapshot, None)))
            }.map_err(Status::failed_precondition)?
                .ok_or_else(|| Status::not_found("当前地图没有完整扫描数据；旧体素地图无法还原全部扫描点"))?;
            (captured.0, captured.1, frame_id)
        };
        let base_epoch = base.as_ref().map(|old| old.epoch.clone()).unwrap_or_default();
        let read_snapshot = snapshot.clone();
        let receiver = self.bridge.register_navigation_map_operation(async move {
            tokio::task::spawn_blocking(move || {
                let removed = match base {
                    Some(old) => read_snapshot.removed_prefix_ranges(&old, request.offset_points)?,
                    None => Vec::new(),
                };
                let removed_count: u64 = removed.chunks_exact(2).map(|range| range[1]).sum();
                let offset = request.offset_points - removed_count;
                let data = read_snapshot.read(offset, request.max_points)?;
                Ok((data, removed, offset))
            }).await.map_err(|e| e.to_string()).and_then(|v| v)
        }).map_err(Status::unavailable)?;
        let (data, removed_ranges, offset) = receiver.await.map_err(|e| Status::unavailable(e.to_string()))?
            .map_err(Status::invalid_argument)?;
        self.ensure_running_admission("GetDenseCloudChunk").await?;
        let readable = self.bridge.cloud_map_writer.lock().as_ref()
            .is_some_and(|writer| writer.dense_read_epoch_available(&snapshot.epoch));
        if !readable {
            return Err(Status::failed_precondition("DENSE_CLOUD_SOURCE_CHANGED"));
        }
        Ok(Response::new(DenseCloudChunk {
            epoch: snapshot.epoch, revision: snapshot.receipt.cloud_revision,
            source_timestamp_ns: snapshot.receipt.source_timestamp_ns, frame_id,
            session_id: snapshot.session_id, session_generation: snapshot.receipt.session_generation,
            total_points: snapshot.point_count, offset_points: offset, xyz_f32_le: data,
            base_epoch, base_offset_points: request.offset_points, removed_ranges,
            bounds_min: snapshot.bounds_min.to_vec(), bounds_max: snapshot.bounds_max.to_vec(),
        }))
    }

    async fn get_mapping_cloud_authority(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingCloudAuthority>, Status> {
        self.ensure_running_admission("GetMappingCloudAuthority").await?;
        let sealed = self.activation_receipts.current_sealed_assigned_target_snapshot()
            .ok_or_else(|| Status::failed_precondition("sealed assigned map authority is unavailable"))?;
        let activation = &sealed.activation_identity;
        let committed = self.bridge.committed_map_authority.lock()
            .map_err(|_| Status::unavailable("committed map authority is unavailable"))?
            .clone().ok_or_else(|| Status::failed_precondition("committed map authority is unavailable"))?;
        if activation.request.session_id() != committed.session_id
            || i64::try_from(activation.session_generation).ok() != Some(committed.session_generation)
            || sealed.target_receipt.source_member.frame_id() != committed.frame_id
        {
            return Err(Status::failed_precondition("sealed assigned map does not match committed map authority"));
        }
        Ok(Response::new(MappingCloudAuthority {
            schema_version: 3,
            map_name: activation.request.map_name().to_owned(),
            map_frame_id: committed.frame_id,
            activation_request_id: activation.request.activation_request_id().to_owned(),
            runtime_generation: activation.runtime_generation,
            lifecycle_epoch: activation.lifecycle_epoch,
            mapping_session_id: activation.request.session_id().to_owned(),
            mapping_session_generation: activation.session_generation,
            backend_kind: format!("{:?}", activation.backend_kind).to_ascii_lowercase(),
            backend_generation: activation.backend_generation,
            committed_occupancy_revision: u64::try_from(committed.revision)
                .map_err(|_| Status::internal("committed revision is invalid"))?,
            committed_occupancy_commit_id: Some(MappingCommitIdentity {
                boot_nonce: committed.commit_boot_nonce.to_vec(),
                sequence: u64::try_from(committed.commit_sequence)
                    .map_err(|_| Status::internal("map commit sequence is invalid"))?,
            }),
            committed_occupancy_source_timestamp_ns: u64::try_from(committed.source_timestamp_ns)
                .map_err(|_| Status::internal("map source timestamp is invalid"))?,
            owner_epoch: self.bridge.nav.producer_runtime_generation(),
            activation_receipt_v2: Some(mapping_cloud_assigned_receipt(activation, &sealed.target_receipt)),
            ..Default::default()
        }))
    }

    async fn submit_mapping_cloud_goal(
        &self,
        req: Request<MappingCloudGoalRequest>,
    ) -> Result<Response<MappingCloudGoalTerminal>, Status> {
        self.ensure_running_admission("SubmitMappingCloudGoal").await?;
        let request = req.into_inner();
        if request.schema_version != 3 {
            return Err(Status::invalid_argument("mapping-cloud requires assigned schema_version 3"));
        }
        let request_commit = validate_mapping_commit_identity(
            request.committed_occupancy_commit_id.as_ref(), "mapping-cloud occupancy")
            .map_err(Status::invalid_argument)?;
        let sealed = self.activation_receipts.current_sealed_assigned_target_snapshot()
            .ok_or_else(|| Status::failed_precondition("sealed assigned map authority is unavailable"))?;
        let activation = &sealed.activation_identity;
        let admitted_receipt = mapping_cloud_assigned_receipt(activation, &sealed.target_receipt);
        if request.activation_receipt_v2.as_ref() != Some(&admitted_receipt)
            || activation.request.map_name() != request.map_name
            || sealed.target_receipt.source_member.frame_id() != request.map_frame_id
            || activation.runtime_generation != request.runtime_generation
            || activation.request.session_id() != request.mapping_session_id
            || activation.session_generation != request.mapping_session_generation
        {
            return Err(Status::failed_precondition("mapping-cloud request does not match current assigned map authority"));
        }
        let committed_map = self.bridge.committed_map_authority.lock()
            .map_err(|_| Status::unavailable("committed map authority is unavailable"))?
            .clone().ok_or_else(|| Status::failed_precondition("committed map authority is unavailable"))?;
        if committed_map.session_id != request.mapping_session_id
            || u64::try_from(committed_map.session_generation).ok() != Some(request.mapping_session_generation)
            || u64::try_from(committed_map.revision).ok() != Some(request.committed_occupancy_revision)
            || committed_map.commit_boot_nonce.as_slice() != request_commit.boot_nonce
            || u64::try_from(committed_map.commit_sequence).ok() != Some(request_commit.sequence)
            || u64::try_from(committed_map.source_timestamp_ns).ok() != Some(request.committed_occupancy_source_timestamp_ns)
            || committed_map.frame_id != request.map_frame_id
        {
            return Err(Status::failed_precondition("mapping-cloud request does not match current committed map authority"));
        }
        let asset_id = activation.request.asset_identity().to_wire().asset_id;
        let georeference = Arc::new(AssignedGeoreferenceV3::from_json(&request.georeference_json, &asset_id)
            .map_err(|error| Status::invalid_argument(error.to_string()))?);
        if georeference.contract().map_name != request.map_name
            || georeference.contract().map_frame_id != request.map_frame_id
        {
            return Err(Status::failed_precondition("mapping-cloud georeference does not match current assigned map authority"));
        }
        let owner_epoch = self.bridge.nav.producer_runtime_generation();
        if request.runtime_generation != owner_epoch {
            return Err(Status::failed_precondition("mapping-cloud request runtime generation is stale"));
        }
        let admission_context = crate::nav_runtime::NavAdmissionContext::with_expected_map_name(
            self.bridge.mapping_lifecycle_lock.clone(),
            self.bridge.committed_map_authority.clone(),
            self.bridge.map_revision_latest_tx.subscribe(),
            Some(request.map_name.clone()),
        ).with_mapping_adapter(self.bridge.mapping_adapter.clone());
        let active_map = AssignedMapIdentityV3 {
            asset_id,
            activation_request_id: activation.request.activation_request_id().to_owned(),
            checkpoint_id: activation.request.checkpoint_identity().to_wire_id(),
            map_name: request.map_name,
            map_frame_id: request.map_frame_id,
            runtime_generation: activation.runtime_generation,
            lifecycle_epoch: activation.lifecycle_epoch,
            mapping_session_id: activation.request.session_id().to_owned(),
            mapping_session_generation: activation.session_generation,
            backend_kind: format!("{:?}", activation.backend_kind).to_ascii_lowercase(),
            backend_generation: activation.backend_generation,
            committed_occupancy_revision: request.committed_occupancy_revision,
            committed_occupancy_boot_nonce: request_commit.boot_nonce.as_slice().try_into()
                .map_err(|_| Status::invalid_argument("map commit nonce must be 16 bytes"))?,
            committed_occupancy_commit_sequence: request_commit.sequence,
            committed_occupancy_source_timestamp_ns: request.committed_occupancy_source_timestamp_ns,
        };
        // Only bounded original descriptors survive the network wait, not the checkpoint owner.
        drop(sealed);
        let query_request_id = request.query_request_id.trim().to_owned();
        let cloud_request = MappingCloudRequest {
            query_identity: CloudQueryIdentityV1 {
                query_request_id: query_request_id.clone(),
                producer: "mapping-cloud".to_owned(),
                producer_epoch: owner_epoch,
                source_timestamp_ns: i64::try_from(now_ns())
                    .map_err(|_| Status::internal("source timestamp exceeds wire range"))?,
            },
            query_text: request.query_text,
            active_map,
            georeference,
            caller_timeout: Duration::from_millis(request.caller_timeout_ms),
            provider_timeout: Duration::from_millis(request.provider_timeout_ms),
        };
        self.mapping_cloud_owner
            .submit(cloud_request)
            .await
            .map_err(|error| Status::failed_precondition(error.to_string()))?;
        let terminal = self
            .mapping_cloud_owner
            .await_terminal(&query_request_id)
            .await
            .map_err(|error| Status::internal(error.to_string()))?;
        let (status, message, navigation) = match terminal {
            MappingCloudTerminal::Succeeded(intent) => {
                // The local owner returns an immutable, constructor-validated assigned goal.
                // Query latency may cross a map replacement even when a name/frame is reused.
                let current = self.activation_receipts.current_sealed_assigned_target_snapshot()
                    .ok_or_else(|| Status::failed_precondition("assigned map authority changed during cloud query"))?;
                if mapping_cloud_assigned_receipt(&current.activation_identity, &current.target_receipt)
                    != admitted_receipt
                {
                    return Err(Status::failed_precondition("assigned map authority changed during cloud query"));
                }
                drop(current);
                let goal = mapping_cloud_assigned_goal_pose(&intent)?;
                let admission_io = crate::nav_runtime::NavAdmissionIo::new(
                    self.bridge.state.clone(),
                    self.bridge.transport.clone(),
                    self.bridge.transport_backend_name,
                    self.bridge.lcm_backend_available, self.bridge.body_reference_frame_id.clone(),
);
                let outcome = crate::nav_runtime::admit_goal_intent(
                    &admission_context,
                    &admission_io,
                    &self.bridge.nav,
                    schema::GoalIntent {
                        map_frame_generation: admission_context.map_frame_generation(),
                        options: Default::default(),
                        producer: "mapping-cloud".to_owned(),
                        producer_epoch: i64::try_from(owner_epoch)
                            .map_err(|_| Status::internal("owner epoch exceeds wire range"))?,
                        priority: 0,
                        session_id: query_request_id.clone(),
                        owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                        parent_request_id: query_request_id.clone(),
                        goal_generation: 0,
                        cancel_generation: i64::try_from(
                            self.bridge.nav.cancel_snapshot().generation,
                        )
                        .map_err(|_| Status::internal("cancel generation exceeds wire range"))?,
                        committed_map,
                        boundary_id: String::new(),
                        body_to_support_z_m: None,
                        boundary_points: Vec::new(),

                        motion_guard_boot_nonce: request
                            .motion_guard_boot_nonce
                            .as_slice()
                            .try_into()
                            .map_err(|_| {
                                Status::invalid_argument(
                                    "motion_guard_boot_nonce must contain 16 bytes",
                                )
                            })?,
                        motion_stop_generation: i64::try_from(request.motion_stop_generation)
                            .map_err(|_| {
                                Status::invalid_argument(
                                    "motion stop generation exceeds wire range",
                                )
                            })?,
                        goal,
                    },
                    crate::nav_runtime::GoalProducerEpochAuthority::MappingCloud,
                )
                .await;
                let update = NavigationUpdate::from_set_goal_outcome(&outcome, &self.bridge.nav).await;
                let navigation = nav_state_to_proto(update);
                // A resolved receipt is evaluated independently of accepted=false.
                let status = if outcome.resolved_receipt.is_some() {
                    navigation.status.as_str()
                } else if outcome.accepted() {
                    "SUCCEEDED"
                } else if matches!(outcome.lease_origin,
                    crate::nav_runtime::GoalLeaseOrigin::InstalledUnresolved
                        | crate::nav_runtime::GoalLeaseOrigin::ExistingUnresolved) {
                    "ADMISSION_UNCONFIRMED"
                } else {
                    "ADMISSION_REJECTED"
                };
                (
                    status.to_owned(),
                    navigation.message.clone(),
                    Some(navigation),
                )
            }
            MappingCloudTerminal::ProviderRejected => (
                "PROVIDER_REJECTED".to_owned(),
                "provider returned no result".to_owned(),
                None,
            ),
            MappingCloudTerminal::ProviderFailed(error) => {
                ("PROVIDER_FAILED".to_owned(), error, None)
            }
            MappingCloudTerminal::ProviderTimeout => (
                "PROVIDER_TIMEOUT".to_owned(),
                "provider deadline elapsed".to_owned(),
                None,
            ),
            MappingCloudTerminal::CallerDeadline => (
                "CALLER_DEADLINE".to_owned(),
                "caller deadline elapsed".to_owned(),
                None,
            ),
            MappingCloudTerminal::Cancelled => {
                ("CANCELLED".to_owned(), "query cancelled".to_owned(), None)
            }
            MappingCloudTerminal::AdaptationFailed(error) => {
                ("ADAPTATION_FAILED".to_owned(), error.to_string(), None)
            }
            MappingCloudTerminal::FinalizerFailed(error) => {
                ("FINALIZER_FAILED".to_owned(), error, None)
            }
        };
        Ok(Response::new(MappingCloudGoalTerminal {
            status,
            message,
            query_request_id,
            navigation,
        }))
    }

    async fn cancel_navigation(
        &self,
        req: Request<CancelNavigationRequest>,
    ) -> Result<Response<NavigationState>, Status> {
        let request = req.get_ref();
        let input = crate::nav_runtime::CancelGoalInput {
            goal_id: request.goal_id.trim().to_string(), owner_kind: request.owner_kind,
            parent_request_id: request.parent_request_id.trim().to_string(),
            goal_generation: request.goal_generation,
            producer_runtime_generation: request.producer_runtime_generation,
        };
        if let Some(confirmation) = request.global_stop_confirmation.as_ref() {
            let (update, retired_current) = self.bridge.nav.retire_goal_after_global_stop(&input, confirmation)
                .await.map_err(Status::failed_precondition)?;
            if retired_current {
                self.bridge.nav.publish_global_stop_retirement(
                    &update, &self.bridge.state, &self.bridge.nav_state_tx,
                ).await;
            }
            return Ok(Response::new(nav_state_to_proto(update)));
        }
        if let Some(update)=self.bridge.nav.terminal_receipt_for(&input).await {
            return Ok(Response::new(nav_state_to_proto(update)));
        }
        let CancelOutcome { state: nav, lease } = nav_cancel(
            &self.bridge,
            &self.bridge.nav,
            input.clone(),
        )
        .await;
        // 空请求在原生拥有方内才绑定目标；按该实际租约找发布期间到达的终态。
        let completed_input = if let Some(lease) = lease.as_ref() {
            crate::nav_runtime::CancelGoalInput {
                goal_id: lease.session_id.clone(), owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(), goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
            }
        } else { input };
        if let Some(update)=self.bridge.nav.terminal_receipt_for(&completed_input).await {
            return Ok(Response::new(nav_state_to_proto(update)));
        }
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
        let _ = self.bridge.nav_state_tx.send(update.clone());
        Ok(Response::new(nav_state_to_proto(update)))
    }

    type SubscribeNavigationStateStream = SubscriptionStream<NavigationState>;
    async fn subscribe_navigation_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeNavigationStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut broadcast_rx = self.bridge.nav_state_tx.subscribe();
        let nav = self.bridge.nav.clone();
        let stream =
            spawn_subscription_stream(self.shutdown.as_ref(), tx.clone(), rx, async move {
                for update in nav.terminal_receipts().await {
                    if tx.send(Ok(nav_state_to_proto(update))).await.is_err() {return;}
                }
                loop {
                    let update = match broadcast_rx.recv().await {
                        Ok(update)=>update,
                        Err(tokio::sync::broadcast::error::RecvError::Closed)=>break,
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(_))=>{
                            // 展示消费者落后不能吞掉已归零的目标终态。
                            for update in nav.terminal_receipts().await {
                                if tx.send(Ok(nav_state_to_proto(update))).await.is_err() {return;}
                            }
                            continue;
                        }
                    };
                    let state = nav_state_to_proto(update);
                    if tx.send(Ok(state)).await.is_err() {
                        break;
                    }
                }
            });
        Ok(Response::new(stream))
    }

    async fn get_navigation_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<NavigationReadiness>, Status> {
        // Phase 4:细分 readiness 子项(LCM backend + tf + costmap +
        // planner profile + cmd_vel sink)由 nav_runtime::compute_readiness
        // 计算,JSON 序列化到 `reason` 字段。Python 侧
        // `get_navigation_readiness` 解析 JSON 返回 dict(满足"costmap、
        // tf、planner、cmd_vel sink、LCM backend 子项"要求)。
        // 顶层 ready/reason 仍走 runtime_diagnostics 的粗略判定,保证
        // CI/CLI 现有 grep "nav ready" 的守门不会失效。
        let r = runtime_diagnostics::navigation_readiness(&self.bridge);
        let detail = compute_readiness(&self.bridge, &self.bridge.nav).await;
        let forbidden_zone_ack = self
            .bridge
            .mapping_adapter
            .as_ref()
            .map(|adapter| adapter.lock().forbidden_zone_ack())
            .unwrap_or_default();
        let json = readiness_to_reason(&detail);
        Ok(Response::new(NavigationReadiness {
            ready: r.ready && detail.ready,
            reason: format!("{}; details={}", r.reason, json),
            reason_code: detail.reason_code.as_str().to_string(),
            blockers: detail
                .blockers
                .iter()
                .map(|blocker| blocker.as_str().to_string())
                .collect(),
            tf_generation: detail.tf_generation,
            planner_generation: detail.planner_generation,
            motion_sink_generation: detail.motion_sink_generation,
            map_frame: detail.map_frame,
            pose_frame: detail.pose_frame,
            forbidden_zone_map_name: forbidden_zone_ack.map_name,
            forbidden_zone_generation: forbidden_zone_ack.generation,
            forbidden_zone_signature: forbidden_zone_ack.signature,
            forbidden_zone_applied_revision: forbidden_zone_ack.applied_revision,
            planner_costmap_revision: detail.costmap_revision,
        }))
    }

    /// Phase 7:mapping 板块 readiness 综合快照。
    async fn get_mapping_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingReadiness>, Status> {
        use crate::mapping_adapter::MappingReadiness as MappingReadinessDetail;
        // parking_lot Mutex guard 不能直接跨 await 持有,这里把锁解开再算。
        let (snapshot, lifecycle, planar) = {
            let lcm_backend = self.bridge.lcm_backend_available;
            match &self.bridge.mapping_adapter {
                Some(arc) => {
                    let g = arc.lock();
                    (
                        MappingReadinessDetail::compute(Some(&*g), lcm_backend),
                        Some(g.snapshot()),
                        g.planar.as_ref().and_then(|binding|binding.diagnostics(&g.snapshot())),
                    )
                }
                None => (MappingReadinessDetail::compute(None, lcm_backend), None, None),
            }
        };
        let message_json = serde_json::to_string(&serde_json::json!({
            "planar": planar,
            "lcm_backend": snapshot.lcm_backend,
            "adapter_present": snapshot.adapter_present,
            "odom_count": snapshot.odom_count,
            "pointcloud_count": snapshot.pointcloud_count,
            "last_occupancy_revision": snapshot.last_occupancy_revision,
            "accumulated_point_count": snapshot.accumulated_point_count,
            "global_consumer_ready": snapshot.global_consumer_ready,
            "frame_generation": snapshot.frame_generation,
            "last_error": snapshot.last_error,
            "mapping_backend_kind": lifecycle.as_ref().map(|state| state.mapping_backend_kind.as_str()),
            "mapping_phase": lifecycle.as_ref().map(|state| state.phase.as_str()),
            "mapping_session_id": lifecycle.as_ref().map(|state| state.session_id.as_str()),
            "map_name": lifecycle.as_ref().map(|state| state.map_name.as_str()),
            "session_generation": lifecycle.as_ref().map(|state| state.session_generation),
            "mapping_active": lifecycle.as_ref().map(|state| state.active).unwrap_or(false),
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(MappingReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    /// Phase 7:exploration 板块 readiness 综合快照。
    async fn get_exploration_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationReadiness>, Status> {
        let backend = self.bridge.lcm_backend_available;
        let snapshot = self.bridge.explore.readiness(backend).await;
        let message_json = serde_json::to_string(&serde_json::json!({
            "backend_available": snapshot.backend_available,
            "controller_active": snapshot.controller_active,
            "source_count": snapshot.source_count,
            "frontier_source_ready": snapshot.frontier_source_ready,
            "costmap_input_ready": snapshot.costmap_input_ready,
            "active_source": snapshot.active_source,
            "arbitration": snapshot.arbitration,
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(ExplorationReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    // ========================================================================
    // cmd_vel sink readiness 写回（rework17）
    // ========================================================================
    //
    // Python `LcmMotionCommandBridge` 在 start/stop 时通过本 RPC 把 sink ready
    // 状态推到 Rust handle。`compute_readiness` 的 cmd_vel_sink 子项读这个 flag;
    // 之前 spawn 默认 false + 跨进程无通道 → readiness 永远 false → 真实
    // production 路径走不通。
    async fn set_cmd_vel_sink_ready(
        &self,
        request: Request<CmdVelSinkReadyRequest>,
    ) -> Result<Response<CmdVelSinkReadyResponse>, Status> {
        let request = request.into_inner();
        if !request.ready {
            self.bridge.set_cmd_vel_sink_ready(false);
            return Ok(Response::new(CmdVelSinkReadyResponse { success: true }));
        }
        if request.generation == 0 {
            return Ok(Response::new(CmdVelSinkReadyResponse { success: false }));
        }
        let detail = compute_readiness(&self.bridge, &self.bridge.nav).await;
        let accepted = detail.motion_sink_fresh
            && detail.cmd_vel_sink_ready
            && detail.motion_sink_generation == request.generation;
        if accepted {
            self.bridge.set_cmd_vel_sink_ready(true);
        }
        Ok(Response::new(CmdVelSinkReadyResponse { success: accepted }))
    }

    // ========================================================================
    // 探索
    // ========================================================================

    async fn get_frontiers(&self, _req: Request<Empty>) -> Result<Response<FrontierList>, Status> {
        let s = self.bridge.state.read().await;
        let frontiers = s
            .frontiers
            .iter()
            .map(|f| Frontier {
                pose: Some(Pose {
                    position: Some(Vec3 {
                        x: f.x,
                        y: f.y,
                        z: f.z,
                    }),
                    orientation: Some(Quat {
                        x: f.qx,
                        y: f.qy,
                        z: f.qz,
                        w: f.qw,
                    }),
                    frame_id: f.frame_id.clone(),
                    timestamp_ns: 0,
                }),
                size_cells: f.size_cells,
                heading: f.heading,
            })
            .collect();
        Ok(Response::new(FrontierList {
            frontiers,
            revision: s.frontiers_revision,
        }))
    }

    type SubscribeFrontiersStream = SubscriptionStream<FrontierList>;
    async fn subscribe_frontiers(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeFrontiersStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.frontiers_tx.subscribe();
        let state_handle = self.bridge.state.clone();
        let stream =
            spawn_subscription_stream(self.shutdown.as_ref(), tx.clone(), rx, async move {
                while let Ok(entries) = broadcast_rx.recv().await {
                    let revision = {
                        let s = state_handle.read().await;
                        s.frontiers_revision
                    };
                    let frontiers = entries
                        .iter()
                        .map(|f| Frontier {
                            pose: Some(Pose {
                                position: Some(Vec3 {
                                    x: f.x,
                                    y: f.y,
                                    z: f.z,
                                }),
                                orientation: Some(Quat {
                                    x: f.qx,
                                    y: f.qy,
                                    z: f.qz,
                                    w: f.qw,
                                }),
                                frame_id: f.frame_id.clone(),
                                timestamp_ns: 0,
                            }),
                            size_cells: f.size_cells,
                            heading: f.heading,
                        })
                        .collect();
                    if tx
                        .send(Ok(FrontierList {
                            frontiers,
                            revision,
                        }))
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            });
        Ok(Response::new(stream))
    }

    // ========================================================================
    // 探索 start / stop / state / diagnostics（Phase 5 additive）
    // ========================================================================
    //
    // 这些 RPC 把 `nuwax-exploration::ExplorationController` 暴露给 Python
    // 平台层；上游 `nuwax-exploration` crate 的 source adapter（wavefront）
    // 在 `explore_runtime.rs` 内部组装；本 facade 只做字段转换 + 锁定 dimos
    // 行为,不再自造隐式 fallback。
    //
    // 状态契约（与 proto 注释一致）：
    // - `ACCEPTED` / `REJECTED` / `PENDING` / `STOPPED` / `ACTIVE` / `UNIMPLEMENTED`
    // - `last_stop_reason` 字符串: `no_information_gain` / `no_reachable_free_space` /
    //   `consecutive_failures_exceeded` / `user_cancelled` / 空串

    async fn start_exploration(
        &self,
        req: Request<ExploreRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let expected_map_frame_id = adapter.lock().configured_map_frame_id().to_string();
        let config = explore_config_from_request(r, &expected_map_frame_id)
            .map_err(Status::invalid_argument)?;
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("StartExploration").await?;
        let mapping = adapter.lock().snapshot();
        if !mapping.active {
            return Err(Status::failed_precondition(
                "StartExploration requires an ACTIVE mapping session",
            ));
        }
        if let Some(active_navigation) = self.bridge.nav.active_goal_lease().await {
            return Err(Status::failed_precondition(format!(
                "StartExploration cannot replace active navigation ownership: goal_id={} parent_request_id={}",
                active_navigation.session_id, active_navigation.parent_request_id
            )));
        }
        let snapshot = self
            .bridge
            .explore
            .start_with_config(&r.request_id, config)
            .await;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn get_exploration_progress(&self,request:Request<ExplorationProgressRequest>)->Result<Response<ExplorationProgressResponse>,Status> {
        let checkpoint_json=self.bridge.explore.checkpoint_progress(&request.get_ref().request_id).await.map_err(Status::failed_precondition)?;
        Ok(Response::new(ExplorationProgressResponse {checkpoint_json}))
    }

    async fn stop_exploration(
        &self,
        req: Request<StopExplorationRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let snapshot = if let Some(confirmation) = r.global_stop_confirmation.as_ref() {
            let (snapshot, navigation_update) = self.bridge.explore.stop_after_global_confirmation(
                &r.request_id, r.expected_exploration_generation, confirmation, &self.bridge.nav,
            ).await.map_err(Status::failed_precondition)?;
            if let Some(update) = navigation_update {
                self.bridge.nav.publish_global_stop_retirement(
                    &update, &self.bridge.state, &self.bridge.nav_state_tx,
                ).await;
            }
            snapshot
        } else {
            self.bridge.explore.stop(&r.request_id).await
        };
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }


    async fn get_exploration_state(&self, _req: Request<Empty>) -> Result<Response<ExplorationState>, Status> {
        Ok(Response::new(exploration_state_from_snapshot(self.bridge.explore.snapshot())))
    }

    async fn get_exploration_diagnostics(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationDiagnostics>, Status> {
        let diag = self.bridge.explore.diagnostics().await;
        let sources = diag
            .sources
            .into_iter()
            .map(|s| ExplorationSourceDiag {
                source_name: s.source_name,
                is_active: s.is_active,
                last_goal_id: s.last_goal_id,
                last_error: s.last_error,
            })
            .collect();
        Ok(Response::new(ExplorationDiagnostics {
            active: diag.active,
            arbitration: diag.arbitration,
            selected_source: diag.selected_source,
            last_stop_reason: diag.last_stop_reason,
            sources,
            frontier_count: diag.frontier_count,
            revision: diag.revision,
            timestamp_ns: now_ns(),
        }))
    }

    // ========================================================================
    // 输入发布兼容（gRPC 端用于调试；走 LCM 后端发布）
    // ========================================================================
    //
    // Phase 4 重写：`publish_odom` / `publish_point_cloud` 不再硬编码 channel；
    // 改走 `channel_registry::channel_for(Odom / PointCloud)`，由 env 或
    // default 拼 typed suffix。删除 stale "B6 blocker" / "Phase 5 才完成"
    // 注释 —— typed payload 编码与 LcmTransport.publish 调用已是当前正确路径。

    async fn publish_laser_scan(
        &self,
        req: Request<PublishLaserScanRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        self.ensure_running_admission("PublishLaserScan").await?;
        let request = req.into_inner();
        let scan = nuwax_codec::lcm::planar::decode(&request.payload)
            .map_err(|e| Status::invalid_argument(e.to_string()))?;
        let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("建图适配器不可用"))?;
        let (binding, session) = {
            let adapter = adapter.lock();
            if !adapter.is_active() || !adapter.planar_ready {
                return Err(Status::failed_precondition("二维会话未就绪"));
            }
            (
                adapter
                    .planar
                    .clone()
                    .ok_or_else(|| Status::failed_precondition("当前为三维执行链"))?,
                adapter.snapshot(),
            )
        };
        if request.session_id != session.session_id
            || request.session_generation != session.session_generation
            || scan.runtime_generation as u64 != binding.client.identity().runtime_generation
        {
            return Err(Status::failed_precondition("二维扫描运行或会话身份已过期"));
        }
        if binding.fence.snapshot().active {
            binding.route_relocalization_scan(&scan).await.map_err(Status::failed_precondition)?;
            return Ok(Response::new(PublishResult{success:true,message:"二维扫描已交付本次重定位候选".into()}));
        }
        binding
            .client
            .publish_scan(session.session_generation, &scan)
            .await
            .map_err(Status::unavailable)?;
        binding.navigation.capture(
            session.session_generation,
            binding.fence.snapshot().epoch,
            scan,
        );
        Ok(Response::new(PublishResult {
            success: true,
            message: "二维扫描已发送".into(),
        }))
    }

    async fn publish_odom(
        &self,
        req: Request<PublishOdomRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        // Phase 4:typed payload 编码走 typed_codec;channel 走 channel_registry;
        // 禁止本文件再写硬编码 `/odom` 或 ad-hoc bytes。
        use nuwax_protocol::LcmMessageType;
        let pose = req.get_ref().pose.as_ref();
        let (frame_id, ts_ns) = match pose {
            Some(p) => (p.frame_id.clone(), p.timestamp_ns),
            None => (String::new(), now_ns()),
        };
        let (x, y, z, qx, qy, qz, qw) = match pose {
            Some(p) => {
                let pos = p.position.as_ref();
                let q = p.orientation.as_ref();
                (
                    pos.map(|v| v.x).unwrap_or(0.0),
                    pos.map(|v| v.y).unwrap_or(0.0),
                    pos.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.x).unwrap_or(0.0),
                    q.map(|v| v.y).unwrap_or(0.0),
                    q.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.w).unwrap_or(0.0),
                )
            }
            None => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        };
        let pose_state = crate::lcm_runtime::PoseState {
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            frame_id,
            timestamp_ns: ts_ns,
        };
        let planar = self
            .bridge
            .mapping_adapter
            .as_ref()
            .and_then(|a| a.lock().planar.clone());
        if let Some(binding) = planar {
            self.ensure_running_admission("PublishOdom").await?;
            if binding.settings.template.odometry_source
                != nuwax_slam_2d_runtime::protocol::OdometrySource::Device
            {
                return Err(Status::failed_precondition(
                    "激光里程计模式不接纳设备里程计",
                ));
            }
            let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            let adapter = self.bridge.mapping_adapter.as_ref().expect("二维适配器");
            let session = adapter.lock().snapshot();
            let request = req.get_ref();
            if !session.active
                || !adapter.lock().planar_ready
                || request.session_id != session.session_id
                || request.session_generation != session.session_generation
                || request.runtime_generation != binding.client.identity().runtime_generation
            {
                return Err(Status::failed_precondition("二维里程计会话或运行身份无效"));
            }
            if [x, y, z, qx, qy, qz, qw].iter().any(|v| !v.is_finite())
                || qx.abs() > 1e-5
                || qy.abs() > 1e-5
                || (qz * qz + qw * qw - 1.0).abs() > 1e-3
                || ts_ns == 0
            {
                return Err(Status::invalid_argument(
                    "二维里程计要求有效采样时间和归一化平面姿态",
                ));
            }
            let sample = nuwax_slam_2d_runtime::input::TimedPose {
                timestamp_ns: ts_ns,
                clock_domain: request.clock_domain.clone(),
                clock_id: request.clock_id.clone(),
                odom_frame_id: pose_state.frame_id,
                body_frame_id: binding.settings.template.body_frame_id.clone(),
                pose: [x, y, 2.0 * qz.atan2(qw)],
            };
            if binding.fence.snapshot().active {
                binding.route_relocalization_odom(sample.clone()).await.map_err(Status::failed_precondition)?;
                binding.navigation.capture_odometry(session.session_generation,sample).map_err(Status::failed_precondition)?;
                return Ok(Response::new(PublishResult{success:true,message:"二维里程计已交付本次重定位候选".into()}));
            }
            let response=binding
                .execute(nuwax_slam_2d_runtime::protocol::Operation::Odometry { sample:sample.clone() })
                .await
                .map_err(Status::unavailable)?;
            binding.navigation.capture_odometry(session.session_generation,sample.clone()).map_err(Status::failed_precondition)?;
            let published=crate::slam2d_owner::publish_device_odometry(&self.bridge,&binding,&session,&sample,&response.status)
                .await.map_err(Status::failed_precondition)?;
            let epoch=binding.fence.snapshot().epoch;
            drop(_lifecycle);
            // 有效设备位姿下的真实新扫描独立派生；计算/发布仍由根保留任务和原提交锁拥有。
            if let (true,Some(native_session),Some(estimate))=(published,response.status.session,response.status.last_estimate) {
                let observer_bridge=self.bridge.clone();let observer_binding=binding.clone();
                let receiver=self.bridge.register_planar_operation(async move {
                    crate::planar_navigation::observe_device_latest(&observer_bridge,&observer_binding,&native_session,&estimate,&sample,epoch).await
                }).map_err(Status::unavailable)?;
                receiver.await.map_err(|_|Status::unavailable("二维观测保留任务未交付终态"))?
                    .map_err(Status::unavailable)?;
            }
            return Ok(Response::new(PublishResult {
                success: true,
                message: "二维里程计已接纳".into(),
            }));
        }
        let payload = match typed_codec::encode_odometry_payload(&pose_state, &self.base_frame_id) {
            Ok(payload) => payload,
            Err(error) => {
                tracing::warn!("publish_odom: payload encode failed: {}", error);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("Odometry payload encode failed: {}", error),
                }));
            }
        };

        // Phase 4:走 channel_registry::channel_for(Odom) 公共 helper。
        let topic = crate::channel_registry::channel_for(crate::channel_registry::ChannelKey::Odom);
        let msg_type = match LcmMessageType::new("nav_msgs/Odometry") {
            Ok(m) => m,
            Err(e) => {
                tracing::error!("publish_odom: LcmMessageType 构造失败: {}", e);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LcmMessageType::new failed: {}", e),
                }));
            }
        };
        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        let lcm_transport = self.bridge.transport.clone();
        match lcm_transport.publish(frame).await {
            Ok(()) => Ok(Response::new(PublishResult {
                success: true,
                message: format!(
                    "odom LCM publish 已接受（backend={}）",
                    self.bridge.transport_backend_name
                ),
            })),
            Err(e) => {
                tracing::warn!("publish_odom: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }

    async fn report_native_input_failure(
        &self, req: Request<NativeInputFailureRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        if self.native_raw_input_binding.is_none() {
            return Err(Status::failed_precondition("当前不是原生传感器输入"));
        }
        let request = req.into_inner();
        if request.reason.trim().is_empty() || request.reason.len() > 1024 {
            return Err(Status::invalid_argument("输入故障原因不能为空或超过1024字节"));
        }
        let reason = format!("source_ns={} {}", request.first_missing_source_timestamp_ns, request.reason);
        let mut failure = self.native_input_failure.lock();
        let first = failure.get_or_insert(reason);
        tracing::error!(reason = %first, "原生输入连续性失败，锁定本代后续扫描");
        Ok(Response::new(PublishResult { success: true, message: first.clone() }))
    }

    async fn publish_point_cloud(
        &self,
        req: Request<PublishPointCloudRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        if self.bridge.mapping_adapter.as_ref().is_some_and(|a|a.lock().planar.is_some()) {
            return Err(Status::failed_precondition("二维执行链不接纳三维点云"));
        }
        if req.get_ref().mapping_only {
            self.ensure_running_admission("PublishPointCloud").await?;
            let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            return crate::lcm_runtime::mapping_observation::enqueue(&self.bridge, req.into_inner(), &self.base_frame_id)
                .map(Response::new).map_err(Status::invalid_argument);
        }
        let Some((topic, msg_type)) = self.point_cloud_binding.clone() else {
            return Ok(Response::new(PublishResult {
                success: false,
                message: "PointCloud2 RPC ingress is unavailable for the selected native LIO owner"
                    .to_string(),
            }));
        };
        let pcr = req.get_ref();
        let lcm_transport = self.bridge.transport.clone();
        if let Some((cloud_binding, imu_binding)) = self.native_raw_input_binding.clone() {
            if let Some(reason) = self.native_input_failure.lock().as_ref() {
                return Err(Status::failed_precondition(format!("原生输入已锁定失败，需重启几何运行时：{reason}")));
            }
            if pcr.imu_samples.is_empty() {
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: "native raw ingress requires at least one IMU sample".to_string(),
                }));
            }
            if let Some(expected_frame) = self.native_raw_sensor_frame.as_deref() {
                if pcr.frame_id != expected_frame
                    || pcr
                        .imu_samples
                        .iter()
                        .any(|sample| sample.frame_id != expected_frame)
                {
                    return Err(Status::invalid_argument(format!(
                        "native raw cloud and IMU frame_id must equal configured sensor frame {expected_frame:?}"
                    )));
                }
            }
            validate_native_raw_pointcloud_fields(pcr).map_err(|error| {
                Status::invalid_argument(format!("native raw PointCloud2 rejected: {error}"))
            })?;
            let masked = self.native_self_filter.as_ref().as_ref()
                .map_err(|error| Status::failed_precondition(error.clone()))?
                .as_ref().map(|filter| filter.mask_request(pcr)).transpose()
                .map_err(Status::invalid_argument)?;
            let pcr = masked.as_ref().unwrap_or(pcr);
            let imu_payloads = pcr
                .imu_samples
                .iter()
                .map(encode_native_imu_payload)
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| {
                    Status::invalid_argument(format!("native raw IMU rejected: {error}"))
                })?;
            if let Some(delivery) = &self.native_input_delivery {
                let sequence = delivery.publish(&lcm_transport, pcr, imu_payloads).await?;
                return Ok(Response::new(PublishResult { success: true,
                    message: format!("原生扫描已处理 sequence={sequence} source_ns={}", pcr.timestamp_ns) }));
            }
            let cloud_payload = encode_pointcloud_request_payload(pcr).map_err(|error| {
                Status::invalid_argument(format!("native raw PointCloud2 rejected: {error}"))
            })?;
            publish_native_raw_pair(
                &lcm_transport,
                &cloud_binding,
                &imu_binding,
                cloud_payload,
                imu_payloads,
            )
            .await?;
            return Ok(Response::new(PublishResult {
                success: true,
                message: format!(
                    "native raw PointCloud2+Imu LCM publish 已接受（backend={}）",
                    self.bridge.transport_backend_name
                ),
            }));
        }

        // 带惯性的原始输入只能交给已绑定的真实原生算法，不能在缺少绑定时改算简化模型。
        if !pcr.imu_samples.is_empty() {
            return Err(Status::failed_precondition(
                "带惯性的点云需要已绑定的原生LIO输入通道",
            ));
        }
        let payload = encode_pointcloud_request_payload(pcr);
        let payload = match payload {
            Ok(payload) => payload,
            Err(error) => {
                tracing::warn!("publish_point_cloud: payload encode failed: {}", error);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("PointCloud2 payload encode failed: {}", error),
                }));
            }
        };

        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        match lcm_transport.publish(frame).await {
            Ok(()) => Ok(Response::new(PublishResult {
                success: true,
                message: format!(
                    "PointCloud2 LCM publish 已接受（backend={}）",
                    self.bridge.transport_backend_name
                ),
            })),
            Err(e) => {
                tracing::warn!("publish_point_cloud: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }
}


#[cfg(test)]
mod health_identity_tests {
    use super::*;

    #[tokio::test]
    async fn health_check_exposes_existing_3d_root_generation() {
        let handle = crate::lcm_runtime::build_handle_for_tests().await.expect("LCM测试拥有方");
        let service = GeometryRuntimeService::new(handle);
        let expected = service.activation_receipts.runtime_generation();
        assert!(expected > 0);
        for _ in 0..2 {
            let reply = GeometryRuntime::health_check(&service, Request::new(HealthCheckRequest {}))
                .await.expect("只读健康查询").into_inner();
            assert_eq!(reply.geometry_pipeline, "lio_3d");
            assert_eq!(reply.runtime_generation, expected);
        }
    }
}

#[cfg(test)]
mod base_frame_config_tests {
    use super::*;

    #[test]
    fn canonical_base_frame_value_precedes_legacy_alias() {
        assert_eq!(
            resolve_base_frame_alias(Some(" body "), Some("base_footprint")),
            "body"
        );
        assert_eq!(
            resolve_base_frame_alias(Some(" "), Some(" base_footprint ")),
            "base_footprint"
        );
        assert_eq!(resolve_base_frame_alias(None, None), "base_link");
    }
}

#[cfg(test)]
mod occupancy_rpc_tests {
    use super::*;

    async fn start_occupancy_test_session(service: &GeometryRuntimeService, session_id: &str) {
        GeometryRuntime::start_mapping(
            service,
            Request::new(StartMappingRequest {
                session_id: session_id.to_string(),
                map_name: "occupancy-test-map".to_string(),
            }),
        )
        .await
        .expect("StartMapping for occupancy RPC test");
    }

    #[tokio::test]
    async fn empty_grid_is_reported_as_not_ready_by_all_occupancy_getters() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);

        let meta_error = GeometryRuntime::get_occupancy_grid(&service, Request::new(Empty {}))
            .await
            .expect_err("empty occupancy metadata must not be returned");
        let data_error = GeometryRuntime::get_occupancy_grid_data(&service, Request::new(Empty {}))
            .await
            .expect_err("empty occupancy payload must not be returned");
        let raw_error =
            GeometryRuntime::get_raw_occupancy_grid_data(&service, Request::new(Empty {}))
                .await
                .expect_err("missing physical occupancy must not be returned");

        assert_eq!(meta_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(data_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(raw_error.code(), tonic::Code::FailedPrecondition);
    }

    #[tokio::test]
    async fn malformed_ready_grid_preserves_payload_integrity_error() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle.state.write().await.map = crate::lcm_runtime::MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            frame_id: "map".into(),
            revision: 1,
            data: vec![0, 100, -1],
            ..crate::lcm_runtime::MapState::default()
        };
        let service = GeometryRuntimeService::new(handle);

        let error = GeometryRuntime::get_occupancy_grid_data(&service, Request::new(Empty {}))
            .await
            .expect_err("mismatched occupancy payload must be rejected");

        assert_eq!(error.code(), tonic::Code::Internal);
        assert!(error.message().contains("payload size mismatch"));
    }

    #[tokio::test]
    async fn committed_invalid_grid_metadata_is_reported_as_integrity_error() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);

        for (width, height, resolution, frame_id, expected_reason) in [
            (0, 1, 0.05, "map", "dimensions"),
            (1, 0, 0.05, "map", "dimensions"),
            (1, 1, f32::NAN, "map", "resolution"),
            (1, 1, 0.0, "map", "resolution"),
            (1, 1, 0.05, "  ", "frame_id"),
        ] {
            service.bridge.state.write().await.map = crate::lcm_runtime::MapState {
                width,
                height,
                resolution,
                frame_id: frame_id.into(),
                revision: 1,
                data: vec![0],
                ..crate::lcm_runtime::MapState::default()
            };

            let error = GeometryRuntime::get_occupancy_grid(&service, Request::new(Empty {}))
                .await
                .expect_err("committed invalid occupancy metadata must not be returned");

            assert_eq!(error.code(), tonic::Code::Internal);
            assert!(error.message().contains(expected_reason));
        }
    }

    #[tokio::test]
    async fn canonical_data_and_shm_getters_wait_for_the_same_committed_revision() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_occupancy_test_session(&service, "getter-wait").await;
        let map = crate::lcm_runtime::MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 123,
            data: vec![0, 100, -1, 42],
            ..crate::lcm_runtime::MapState::default()
        };
        let event = crate::lcm_runtime::test_owner_map_revision(
            &mut handle.mapping_adapter.as_ref().expect("mapping adapter").lock(),
            &map,
        );
        handle.state.write().await.map = map.clone();

        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        handle
            .map_revision_tx
            .send(event)
            .expect("coalescer receiver");
        let data_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::get_occupancy_grid_data(&service, Request::new(Empty {})).await
            })
        };
        let shm_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::get_occupancy_grid_shm(&service, Request::new(Empty {})).await
            })
        };
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(!data_task.is_finished());
        assert!(!shm_task.is_finished());

        drop(lifecycle_guard);
        let data = tokio::time::timeout(Duration::from_secs(1), data_task)
            .await
            .expect("data getter timeout")
            .expect("data getter join")
            .expect("data getter status")
            .into_inner();
        let shm = tokio::time::timeout(Duration::from_secs(1), shm_task)
            .await
            .expect("SHM getter timeout")
            .expect("SHM getter join")
            .expect("SHM getter status")
            .into_inner();
        assert_eq!(data.meta.as_ref().map(|meta| meta.revision), Some(1));
        assert_eq!(data.data, vec![0, 100, 255, 42]);
        assert_eq!(shm.revision, 1);
        let descriptor = handle
            .occupancy_shm_current
            .read()
            .await
            .clone()
            .expect("committed descriptor");
        assert_eq!(descriptor.revision, 1);
        assert_eq!(descriptor.generation, shm.generation);
        assert_eq!(descriptor.active_slot as u32, shm.active_slot);
    }

    #[tokio::test]
    async fn failed_shm_write_makes_data_and_descriptor_getters_fail_boundedly() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_occupancy_test_session(&service, "getter-write-failure").await;
        let canonical = crate::lcm_runtime::MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 456,
            data: vec![0, 0, 0, 0],
            ..crate::lcm_runtime::MapState::default()
        };
        handle.state.write().await.map = canonical.clone();
        let mut event = crate::lcm_runtime::test_owner_map_revision(
            &mut handle.mapping_adapter.as_ref().expect("mapping adapter").lock(),
            &canonical,
        );
        event.data = event.data[..event.data.len() - 1].to_vec().into();
        event.size = event.data.len() as u64;
        assert!(event.validate_provenance().is_err(), "malformed body must not pass its receipt guard");
        handle
            .map_revision_tx
            .send(event)
            .expect("coalescer receiver");

        let (data, shm) = tokio::join!(
            GeometryRuntime::get_occupancy_grid_data(&service, Request::new(Empty {})),
            GeometryRuntime::get_occupancy_grid_shm(&service, Request::new(Empty {})),
        );
        let data = data.expect_err("failed write must withhold canonical data");
        let shm = shm.expect_err("failed write must withhold SHM descriptor");
        assert_eq!(data.code(), tonic::Code::DeadlineExceeded);
        assert_eq!(shm.code(), tonic::Code::DeadlineExceeded);
        assert!(data.message().contains("revision 1"));
        assert!(shm.message().contains("revision 1"));
        assert!(handle.map_revision_latest_tx.borrow().is_none());
        assert!(handle.occupancy_shm_current.read().await.is_none());
        assert_eq!(
            handle
                .occupancy_shm
                .as_ref()
                .expect("occupancy writer")
                .lock()
                .published_generation(),
            0
        );
    }
}

#[cfg(test)]
mod pointcloud_request_tests {
    use super::*;
    use crate::mapping_adapter::MappingReadiness;
    use nuwax_transport::{TransportError, TransportResult};
    use std::sync::Mutex as StdMutex;
    use tokio_stream::StreamExt;

    fn request() -> PublishPointCloudRequest {
        PublishPointCloudRequest {
            points: vec![
                0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192,
                64,
            ],
            point_step: 12,
            frame_id: "utlidar_lidar".to_string(),
            timestamp_ns: 1_752_345_678_901_234_000,
            fields: vec![
                PointCloudField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                PointCloudField {
                    name: "y".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                PointCloudField {
                    name: "z".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            point_count: 2,
            is_dense: true,
            imu_samples: Vec::new(),
            ..Default::default()
        }
    }

    #[derive(Clone)]
    struct NativeRawPublishBackend {
        frames: Arc<StdMutex<Vec<RawLcmFrame>>>,
        attempts: Arc<std::sync::atomic::AtomicUsize>,
        fail_at: Option<usize>,
    }

    impl NativeRawPublishBackend {
        fn new(fail_at: Option<usize>) -> Self {
            Self {
                frames: Arc::new(StdMutex::new(Vec::new())),
                attempts: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
                fail_at,
            }
        }
    }

    #[async_trait::async_trait]
    impl nuwax_transport::LcmBackend for NativeRawPublishBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            let attempt = self
                .attempts
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            if self.fail_at == Some(attempt) {
                return Err(TransportError::LcmBackend(format!(
                    "injected native raw publish failure at attempt {attempt}"
                )));
            }
            self.frames
                .lock()
                .expect("native raw published frames")
                .push(frame);
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> nuwax_transport::LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "native-raw-publish-test"
        }
    }

    fn native_raw_bindings() -> ((LcmTopic, LcmMessageType), (LcmTopic, LcmMessageType)) {
        (
            (
                LcmTopic::new("/test/native/cloud").expect("cloud topic"),
                LcmMessageType::new("sensor_msgs.PointCloud2").expect("cloud type"),
            ),
            (
                LcmTopic::new("/test/native/imu").expect("IMU topic"),
                LcmMessageType::new("sensor_msgs.Imu").expect("IMU type"),
            ),
        )
    }

    #[tokio::test]
    async fn producer_input_failure_is_shared_and_keeps_first_gap() {
        let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        let mut service = GeometryRuntimeService::new(handle);
        service.native_raw_input_binding = Some(native_raw_bindings());
        let other = service.clone();
        for (reason, stamp) in [("队列溢出", 123), ("后续故障", 456)] {
            let result = service.report_native_input_failure(Request::new(NativeInputFailureRequest {
                reason: reason.into(), first_missing_source_timestamp_ns: stamp,
            })).await.unwrap().into_inner();
            assert!(result.success && result.message.contains("123") && result.message.contains("队列溢出"));
        }
        let error = other.publish_point_cloud(Request::new(request())).await.unwrap_err();
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("123"));
    }

    #[tokio::test]
    async fn self_cloud_raw_publication_reaches_native_preprocessing_field() {
        let backend = NativeRawPublishBackend::new(None);
        let mut handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        handle.transport = nuwax_transport::LcmTransport::new(Arc::new(backend.clone()));
        let mut service = GeometryRuntimeService::new(handle);
        service.native_raw_input_binding = Some(native_raw_bindings());
        service.native_self_filter = Arc::new(Ok(Some(crate::self_cloud::tests::native_filter())));
        let mut input = request();
        input.point_step = 16;
        input.fields.push(PointCloudField { name: "offset_ns".into(), offset: 12, datatype: 6, count: 1 });
        input.points.clear();
        for xyz in [[0_f32, 0.1, -0.425], [0.0, 0.1, -0.2]] {
            for value in xyz { input.points.extend_from_slice(&value.to_le_bytes()); }
            input.points.extend_from_slice(&10_u32.to_le_bytes());
        }
        input.imu_samples.push(LioImuSample { frame_id: input.frame_id.clone(),
            timestamp_ns: input.timestamp_ns, accel_mps2_z: 9.81, ..Default::default() });
        assert!(service.publish_point_cloud(Request::new(input)).await.unwrap().into_inner().success);
        let frames = backend.frames.lock().unwrap();
        assert_eq!(frames.len(), 2);
        let mut wire = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
        wire.extend_from_slice(&frames[1].payload);
        let cloud = schema::decode_point_cloud2(&wire).unwrap();
        assert_eq!(cloud.point_step, 17);
        assert_eq!(cloud.width, 2);
        assert_eq!(cloud.data[16], 0);
        assert_eq!(cloud.data[33], 1);
    }

    #[tokio::test]
    async fn native_raw_pair_publishes_all_imu_before_cloud() {
        let backend = NativeRawPublishBackend::new(None);
        let transport = nuwax_transport::LcmTransport::new(Arc::new(backend.clone()));
        let (cloud_binding, imu_binding) = native_raw_bindings();

        publish_native_raw_pair(
            &transport,
            &cloud_binding,
            &imu_binding,
            vec![0xc0, 0xde],
            vec![vec![0x01], vec![0x02]],
        )
        .await
        .expect("native raw pair should publish");

        let frames = backend.frames.lock().expect("native raw published frames");
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].msg_type, imu_binding.1);
        assert_eq!(frames[1].msg_type, imu_binding.1);
        assert_eq!(frames[2].msg_type, cloud_binding.1);
        assert_eq!(frames[2].payload, vec![0xc0, 0xde]);
    }

    #[tokio::test]
    async fn native_raw_pair_failure_keeps_cloud_unpublished() {
        let backend = NativeRawPublishBackend::new(Some(1));
        let transport = nuwax_transport::LcmTransport::new(Arc::new(backend.clone()));
        let (cloud_binding, imu_binding) = native_raw_bindings();

        let error = publish_native_raw_pair(
            &transport,
            &cloud_binding,
            &imu_binding,
            vec![0xc0, 0xde],
            vec![vec![0x01], vec![0x02]],
        )
        .await
        .expect_err("second IMU publication should fail");

        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(error.message().contains("sample 1"));
        let frames = backend.frames.lock().expect("native raw published frames");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].msg_type, imu_binding.1);
        assert_ne!(frames[0].msg_type, cloud_binding.1);
    }

    #[test]
    fn structured_request_preserves_pointcloud2_metadata_and_data() {
        let request = request();
        let payload = encode_pointcloud_request_payload(&request).expect("encode request");
        let mut generated = Vec::with_capacity(8 + payload.len());
        generated.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        generated.extend_from_slice(&payload);
        let decoded = schema::decode_point_cloud2(&generated).expect("decode PointCloud2");

        assert_eq!(decoded.header.frame_id, request.frame_id);
        assert_eq!(decoded.header.stamp_sec, 1_752_345_678);
        assert_eq!(decoded.header.stamp_nsec, 901_234_000);
        assert_eq!(decoded.width, 2);
        assert_eq!(decoded.point_step, 12);
        assert_eq!(decoded.data, request.points);
        assert_eq!(
            decoded
                .fields
                .iter()
                .map(|field| field.name.as_str())
                .collect::<Vec<_>>(),
            vec!["x", "y", "z"]
        );
    }

    #[tokio::test]
    async fn imu_pointcloud_without_native_binding_is_rejected_without_publish() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("测试几何运行时");
        let provider = handle
            .transport
            .shared_lcm_context()
            .expect("测试共享通信上下文")
            .provider()
            .to_string();
        let observer = nuwax_transport::LcmTransport::new_real(&provider)
            .expect("独立观察者");
        let unique = uuid::Uuid::new_v4().simple().to_string();
        let binding = (
            LcmTopic::new(&format!("/t/no-lio/{}", &unique[..16])).expect("测试话题"),
            LcmMessageType::new("sensor_msgs/PointCloud2").expect("点云类型"),
        );
        let mut frames = observer
            .subscribe(binding.0.clone(), Some(binding.1.clone()))
            .await;
        let service = GeometryRuntimeService::new_with_point_cloud_binding(handle, binding);
        let mut input = request();
        input.imu_samples.push(LioImuSample {
            timestamp_ns: input.timestamp_ns - 1_000,
            frame_id: input.frame_id.clone(),
            accel_mps2_z: 9.81,
            ..LioImuSample::default()
        });
        let error = GeometryRuntime::publish_point_cloud(&service, Request::new(input))
            .await
            .expect_err("未绑定原生通道不能消费惯性生成替代位姿");
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("原生LIO输入通道"));
        assert!(tokio::time::timeout(Duration::from_millis(80), frames.next())
            .await
            .is_err(), "拒绝输入后不能发布替代点云");
    }

    #[test]
    fn structured_request_rejects_noncanonical_xyz_layout() {
        let mut request = request();
        request.fields[0].offset = 4;

        let error = encode_pointcloud_request_payload(&request).expect_err("x offset must fail");
        assert!(error.to_string().contains("canonical x float32"));
    }

    #[test]
    fn request_rejects_missing_or_padded_frame_identity() {
        for frame_id in ["", "   ", " odom", "odom "] {
            let request = PublishPointCloudRequest {
                points: request().points,
                point_step: 12,
                frame_id: frame_id.to_string(),
                ..PublishPointCloudRequest::default()
            };
            let error = encode_pointcloud_request_payload(&request)
                .expect_err("missing or padded frame must fail closed");
            assert!(error.to_string().contains("frame_id"));
        }
    }

    #[tokio::test]
    async fn publish_pointcloud_rpc_rejects_empty_frame_before_transport_publish() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);
        let request = PublishPointCloudRequest {
            points: request().points,
            point_step: 12,
            ..PublishPointCloudRequest::default()
        };

        let response = GeometryRuntime::publish_point_cloud(&service, Request::new(request))
            .await
            .expect("PublishPointCloud response")
            .into_inner();

        assert!(!response.success);
        assert!(response.message.contains("frame_id"));
    }

    #[tokio::test]
    async fn publish_pointcloud_rpc_rejects_native_owner_without_legacy_binding() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new_without_point_cloud_binding(handle);

        let response = GeometryRuntime::publish_point_cloud(&service, Request::new(request()))
            .await
            .expect("PublishPointCloud response")
            .into_inner();

        assert!(!response.success);
        assert!(response.message.contains("native LIO owner"));
    }

    #[tokio::test]
    async fn publish_pointcloud_uses_startup_binding_instead_of_request_registry() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let provider = handle
            .transport
            .shared_lcm_context()
            .expect("shared real LCM context")
            .provider()
            .to_string();
        let observer = nuwax_transport::LcmTransport::new_real(&provider)
            .expect("independent pointcloud observer");
        let unique = uuid::Uuid::new_v4().simple().to_string();
        let binding = (
            LcmTopic::new(&format!("/t/pc/{}", &unique[..16])).expect("frozen pointcloud topic"),
            LcmMessageType::new("sensor_msgs/PointCloud2").expect("PointCloud2 type"),
        );
        assert_ne!(
            binding.0,
            crate::channel_registry::channel_for(crate::channel_registry::ChannelKey::PointCloud)
        );
        let mut frames = observer
            .subscribe(binding.0.clone(), Some(binding.1.clone()))
            .await;
        let service = GeometryRuntimeService::new_with_point_cloud_binding(handle, binding.clone());
        tokio::time::sleep(Duration::from_millis(30)).await;

        let response = GeometryRuntime::publish_point_cloud(&service, Request::new(request()))
            .await
            .expect("PublishPointCloud response")
            .into_inner();
        assert!(response.success, "{}", response.message);
        let frame = tokio::time::timeout(Duration::from_secs(2), frames.next())
            .await
            .expect("frozen pointcloud binding receive timeout")
            .expect("pointcloud observer closed")
            .expect("pointcloud transport error");
        assert_eq!(frame.topic, binding.0);
        assert_eq!(frame.msg_type, binding.1);
    }

    #[tokio::test]
    async fn publish_pointcloud_rpc_does_not_claim_ingestion_before_subscriber_accepts() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());

        let response = GeometryRuntime::publish_point_cloud(&service, Request::new(request()))
            .await
            .expect("PublishPointCloud RPC")
            .into_inner();

        assert!(response.success, "{}", response.message);
        let adapter = handle.mapping_adapter.expect("mapping adapter");
        let adapter = adapter.lock();
        assert_eq!(adapter.pointcloud_count(), 0);
        assert_eq!(adapter.current_revision(), 0);
        assert!(!MappingReadiness::compute(Some(&adapter), true).ready);
    }

    #[tokio::test]
    async fn mapping_readiness_exposes_global_frame_contract() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);
        let response = GeometryRuntime::get_mapping_readiness(&service, Request::new(Empty {}))
            .await
            .expect("GetMappingReadiness")
            .into_inner();
        let detail: serde_json::Value =
            serde_json::from_str(&response.message).expect("mapping readiness JSON");
        assert_eq!(detail["global_consumer_ready"], false);
        assert_eq!(detail["frame_generation"], 0);
        assert!(!response.ready);
    }
}

#[cfg(test)]
mod frame_tree_tests {
    use super::*;
    use nuwax_tf::make_transform;

    #[tokio::test]
    async fn get_frame_tree_reads_latest_edges_from_canonical_tf_history() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        {
            let state = handle.state.write().await;
            let mut older =
                make_transform("map", "base_link", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
            older.stamp_secs = 1;
            older.stamp_nanos = 2;
            state.tf_tree.set(older);
            let mut latest =
                make_transform("map", "base_link", [3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
            latest.stamp_secs = 4;
            latest.stamp_nanos = 5;
            state.tf_tree.set(latest);
        }
        let service = GeometryRuntimeService::new(handle);

        let response = GeometryRuntime::get_frame_tree(&service, Request::new(Empty {}))
            .await
            .expect("GetFrameTree RPC")
            .into_inner();

        assert_eq!(response.transforms.len(), 1);
        let transform = &response.transforms[0];
        assert_eq!(transform.parent_frame, "map");
        assert_eq!(transform.child_frame, "base_link");
        assert_eq!(transform.translation.as_ref().unwrap().x, 3.0);
        assert_eq!(transform.timestamp_ns, 4_000_000_005);
    }
}

#[cfg(test)]
mod subscription_shutdown_tests {
    use super::*;
    use crate::lcm_runtime::MapState;
    use tokio_stream::StreamExt;

    fn erase_item<T: Send + 'static>(stream: SubscriptionStream<T>) -> SubscriptionStream<()> {
        Box::pin(stream.map(|item| item.map(|_| ())))
    }

    async fn open_all(service: &GeometryRuntimeService) -> Vec<SubscriptionStream<()>> {
        vec![
            erase_item(
                service
                    .subscribe_pose(Request::new(PoseRequest::default()))
                    .await
                    .unwrap()
                    .into_inner(),
            ),
            erase_item(
                service
                    .subscribe_map_updates(Request::new(Empty {}))
                    .await
                    .unwrap()
                    .into_inner(),
            ),
            erase_item(
                service
                    .subscribe_cloud_map_updates(Request::new(Empty {}))
                    .await
                    .unwrap()
                    .into_inner(),
            ),
            erase_item(
                service
                    .subscribe_navigation_state(Request::new(Empty {}))
                    .await
                    .unwrap()
                    .into_inner(),
            ),
            erase_item(
                service
                    .subscribe_frontiers(Request::new(Empty {}))
                    .await
                    .unwrap()
                    .into_inner(),
            ),
        ]
    }

    fn retained_inputs(handle: &GeometryRuntimeHandle) -> [usize; 5] {
        [
            Arc::strong_count(&handle.state),
            handle.map_revision_latest_tx.receiver_count(),
            handle.cloud_map_tx.receiver_count(),
            handle.nav_state_tx.receiver_count(),
            handle.frontiers_tx.receiver_count(),
        ]
    }

    async fn commit_pose(handle: &GeometryRuntimeHandle, stamp: u64) {
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let pose = PoseState {
            x: stamp as f64,
            qw: 1.0,
            frame_id: "odom".into(),
            timestamp_ns: stamp,
            ..PoseState::default()
        };
        let binding = (
            LcmTopic::new("/test/geometry/pose-stream-tf").unwrap(),
            LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
        );
        assert!(crate::lcm_runtime::handoff_lio_odometry(handle, &pose, "base_link", &binding)
            .await
            .unwrap());
    }

    #[tokio::test]
    async fn pose_subscription_is_immediate_coalesces_slow_consumers_and_preserves_source_time() {
        let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        handle.mapping_adapter.as_ref().unwrap().lock()
            .start_session("pose-stream-session", "warehouse").unwrap();
        let service = GeometryRuntimeService::new(handle.clone());
        let mut stream = service.subscribe_pose(Request::new(PoseRequest::default()))
            .await.unwrap().into_inner();
        let first = tokio::time::timeout(Duration::from_millis(50), stream.next())
            .await.expect("首个快照不能等待原来的100毫秒轮询")
            .unwrap().unwrap();
        assert_eq!(first.pose.unwrap().timestamp_ns, 0);
        assert_eq!(first.sequence, 1);

        // 更新先于下一次轮询流到达，必须保留唤醒并直接返回源时间。
        commit_pose(&handle, 10).await;
        let changed = tokio::time::timeout(Duration::from_millis(50), stream.next())
            .await.expect("规范位姿提交应立即唤醒订阅")
            .unwrap().unwrap();
        assert_eq!(changed.pose.unwrap().timestamp_ns, 10);
        assert_eq!(changed.sequence, 2);

        // 客户端暂停消费时不积累历史帧；恢复后交付最新规范状态。
        for stamp in 11..=50 {
            commit_pose(&handle, stamp).await;
        }
        let latest = tokio::time::timeout(Duration::from_millis(50), stream.next())
            .await.unwrap().unwrap().unwrap();
        assert_eq!(latest.pose.unwrap().timestamp_ns, 50);
        assert_eq!(latest.sequence, 3);
        commit_pose(&handle, 50).await;
        assert!(tokio::time::timeout(Duration::from_millis(20), stream.next()).await.is_err(),
            "相同源时间的规范通知不能生成重复位姿帧");
        commit_pose(&handle, 51).await;
        let changed = tokio::time::timeout(Duration::from_millis(50), stream.next())
            .await.unwrap().unwrap().unwrap();
        assert_eq!(changed.pose.unwrap().timestamp_ns, 51);
        assert_eq!(changed.sequence, 4);
    }

    #[tokio::test]
    async fn pose_subscription_shutdown_cancels_latest_state_lock_wait() {
        let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        let (shutdown, _listener) = shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());
        let _state_guard = handle.state.write().await;
        let mut stream = tokio::time::timeout(
            Duration::from_millis(50),
            service.subscribe_pose(Request::new(PoseRequest::default())),
        ).await.expect("建立订阅不能等待共享状态锁").unwrap().into_inner();
        assert!(tokio::time::timeout(Duration::from_millis(10), stream.next()).await.is_err());
        shutdown.publish_graceful_shutdown();
        assert!(tokio::time::timeout(Duration::from_millis(50), stream.next())
            .await.expect("关闭必须取消位姿快照锁等待").is_none());
    }

    #[tokio::test]
    async fn subscription_streams_release_all_five_producers_without_new_input() {
        for mode in [
            "client_without_coordinator",
            "client_with_coordinator",
            "graceful",
            "already_graceful",
        ] {
            let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
            let (shutdown, _listener) = shutdown_channel();
            let service = if mode == "client_without_coordinator" {
                GeometryRuntimeService::new(handle.clone())
            } else {
                GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone())
            };
            let baseline = retained_inputs(&handle);
            if mode == "already_graceful" {
                shutdown.publish_graceful_shutdown();
            }
            let mut streams = open_all(&service).await;
            if mode == "graceful" {
                shutdown.request_shutdown();
                tokio::task::yield_now().await;
                assert!(retained_inputs(&handle)[1] > baseline[1]);
                assert!(
                    !*shutdown.graceful_tx.borrow(),
                    "接纳关闭不能提前结束响应流"
                );
                shutdown.publish_graceful_shutdown();
            }
            if mode == "graceful" || mode == "already_graceful" {
                for stream in &mut streams {
                    assert!(tokio::time::timeout(Duration::from_secs(1), stream.next())
                        .await
                        .expect("关闭信号必须结束每条流")
                        .is_none());
                }
            }
            drop(streams);
            tokio::time::timeout(Duration::from_secs(1), async {
                while retained_inputs(&handle) != baseline {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .unwrap_or_else(|_| panic!("{mode}: 所有生产者必须释放状态和上游订阅"));
        }
    }

    struct ProducerDrop(Option<tokio::sync::oneshot::Sender<()>>);

    impl Drop for ProducerDrop {
        fn drop(&mut self) {
            if let Some(sender) = self.0.take() {
                let _ = sender.send(());
            }
        }
    }

    #[tokio::test]
    async fn subscription_shutdown_cancels_backpressured_send_and_discards_buffered_payload() {
        let (shutdown, _listener) = shutdown_channel();
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        let (queued_tx, queued_rx) = tokio::sync::oneshot::channel();
        let (dropped_tx, dropped_rx) = tokio::sync::oneshot::channel();
        let guard = ProducerDrop(Some(dropped_tx));
        let mut stream = spawn_subscription_stream(Some(&shutdown), tx.clone(), rx, async move {
            let _guard = guard;
            tx.send(Ok(vec![1u8; 1024 * 1024])).await.unwrap();
            queued_tx.send(()).unwrap();
            tx.send(Ok(vec![2u8; 1024 * 1024])).await.unwrap();
            panic!("无接收方时第二帧应保持背压等待");
        });
        queued_rx.await.unwrap();
        shutdown.publish_graceful_shutdown();
        tokio::time::timeout(Duration::from_secs(1), dropped_rx)
            .await
            .expect("未轮询响应流也必须取消生产者")
            .unwrap();
        assert!(
            stream.next().await.is_none(),
            "关闭后不能继续交付队列旧地图"
        );
    }

    #[tokio::test]
    async fn subscription_shutdown_cancels_mapping_lifecycle_lock_wait() {
        let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        handle
            .mapping_adapter
            .as_ref()
            .unwrap()
            .lock()
            .start_session("stream-session", "warehouse")
            .unwrap();
        let event = crate::lcm_runtime::test_owner_map_revision(
            &mut handle.mapping_adapter.as_ref().expect("mapping adapter").lock(),
            &MapState {
                width: 1,
                height: 1,
                resolution: 0.05,
                origin_x: 0.0,
                origin_y: 0.0,
                frame_id: "map".into(),
                revision: 1,
                timestamp_ns: 1,
                data: vec![0],
            },
        );
        handle.map_revision_latest_tx.send_replace(Some(event));
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let (shutdown, _listener) = shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());
        let baseline = handle.map_revision_latest_tx.receiver_count();
        let mut stream = service
            .subscribe_map_updates(Request::new(Empty {}))
            .await
            .unwrap()
            .into_inner();
        tokio::task::yield_now().await;
        shutdown.publish_graceful_shutdown();
        assert!(tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .unwrap()
            .is_none());
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.map_revision_latest_tx.receiver_count() != baseline {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("取消不应等待生命周期锁释放");
        drop(lifecycle_guard);
    }
}

#[cfg(test)]
mod map_update_tests {
    use super::*;
    use crate::lcm_runtime::MapState;
    use tokio_stream::StreamExt;

    fn test_mapping_event(map: MapState) -> MapRevisionEvent {
        let mut adapter = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "map",
            "odom",
        );
        let session = adapter
            .start_session("stream-session", "warehouse")
            .expect("test mapping session");
        let committed = crate::lcm_runtime::test_owner_map_revision(&mut adapter, &map);
        crate::lcm_runtime::PreparedMapRevisionEvent::from_map(&map, &session)
            .seal(committed.receipt).expect("fixture uses actual mapping owner receipt")
    }

    #[tokio::test]
    async fn subscribe_map_updates_is_event_driven_and_carries_real_payload() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .start_session("stream-session", "warehouse")
            .expect("activate stream session");
        let service = GeometryRuntimeService::new(handle.clone());
        let response = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("SubscribeMapUpdates RPC");
        let mut stream = response.into_inner();

        let event = crate::lcm_runtime::test_owner_map_revision(
            &mut handle.mapping_adapter.as_ref().expect("mapping adapter").lock(),
            &MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            origin_x: -1.0,
            origin_y: -2.0,
            frame_id: "map".into(),
            revision: 7,
            timestamp_ns: 1_750_000_123_456_000_000,
            data: vec![0, 100, -1, 0],
        });
        handle.state.write().await.map = event.to_map_state();
        handle
            .map_revision_tx
            .send(event.clone())
            .expect("subscriber must be registered before RPC returns");

        let update = tokio::time::timeout(std::time::Duration::from_millis(400), stream.next())
            .await
            .expect("event-driven stream must not wait for the former 500ms poll")
            .expect("stream item")
            .expect("map update");

        let meta = update.meta.expect("metadata");
        assert_eq!(meta.revision, event.revision);
        assert_eq!(meta.width, event.width);
        assert_eq!(meta.height, event.height);
        assert_eq!(update.data.as_slice(), event.data.as_ref());
        assert_eq!(update.size, event.size);
        assert_eq!(update.schema_version, 2);
        assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        assert_eq!(update.raw_representation, event.raw_representation as i32);
        assert!(update.raw_data.is_empty());
        assert_eq!(update.raw_representation, crate::lcm_runtime::RawMapRepresentation::SameAsEffective as i32);
        assert_eq!(update.last_seen_ms, event.last_seen_ms);
        assert_eq!(update.session_id, "stream-session");
        assert_eq!(update.map_name, "warehouse");
        assert_eq!(update.session_generation, 1);
        let shm_ref = tokio::time::timeout(std::time::Duration::from_secs(1), async {
            loop {
                match GeometryRuntime::get_occupancy_grid_shm(&service, Request::new(Empty {}))
                    .await
                {
                    Ok(response) => break response.into_inner(),
                    Err(status) if status.code() == tonic::Code::FailedPrecondition => {
                        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
                    }
                    Err(status) => panic!("unexpected SHM status: {status}"),
                }
            }
        })
        .await
        .expect("sole SHM writer must publish current reference");
        assert_eq!(shm_ref.revision, event.revision);
        assert!(shm_ref.size > event.size, "ShmRef.size is mmap file size");
        assert_eq!(shm_ref.layout_version, 3);
        assert_eq!(shm_ref.payload_schema_version, 1);
        assert!(shm_ref.generation > 0);
        assert!(shm_ref.active_slot < 2);
        assert_eq!(shm_ref.payload_size, event.size);
        let internal = handle
            .occupancy_shm_current
            .read()
            .await
            .clone()
            .expect("v3 SHM metadata");
        assert_eq!(internal.layout_version, 3);
        assert_eq!(internal.payload_schema_version, 1);
        assert_eq!(internal.payload_size, event.size);
        assert!(internal.generation > 0);
        assert!(internal.active_slot < 2);
        assert_eq!(
            shm_ref.key,
            handle
                .occupancy_shm_path
                .as_deref()
                .expect("authoritative SHM path")
        );
    }

    #[tokio::test]
    async fn map_update_carries_atomic_raw_grid_when_effective_cells_differ() {
        let mut adapter = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "map",
            "odom",
        );
        let session = adapter
            .start_session("raw-stream-session", "warehouse")
            .expect("test mapping session");
        let raw = MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            frame_id: "map".into(),
            revision: 8,
            timestamp_ns: 8,
            data: vec![-1, 0, 50, 100],
            ..MapState::default()
        };
        let effective = MapState {
            data: vec![100, 100, 50, 100],
            ..raw.clone()
        };
        let event = MapRevisionEvent::from_effective_and_raw_for_tests(&effective, &raw, &session);
        let current = Arc::new(RwLock::new(None));
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);

        assert!(forward_map_revision(&tx, event.clone(), &current, "map").await);
        let update = rx
            .recv()
            .await
            .expect("stream update")
            .expect("valid atomic map update");

        assert_eq!(update.data, vec![100, 100, 50, 100]);
        assert_eq!(update.raw_data, vec![255, 0, 50, 100]);
        assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        assert_eq!(update.raw_representation, event.raw_representation as i32);
        assert_eq!(event.raw_representation, crate::lcm_runtime::RawMapRepresentation::SeparateRaw);
        assert_ne!(update.raw_data, update.data);
    }

    #[tokio::test]
    async fn map_update_rejects_receipt_and_raw_representation_mismatches() {
        let event = test_mapping_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_owned(),
            revision: 9,
            timestamp_ns: 9,
            data: vec![0, 100],
            ..MapState::default()
        });
        event.validate_provenance().expect("valid receipt-bound fixture");
        assert_eq!(event.raw_representation, crate::lcm_runtime::RawMapRepresentation::EffectiveFallback);
        let mutations: [fn(&mut MapRevisionEvent); 7] = [
            |event| event.receipt.session_id.push_str("-foreign"),
            |event| event.receipt.session_generation += 1,
            |event| event.receipt.occupancy_revision += 1,
            |event| event.receipt.source_timestamp_ns += 1,
            |event| event.receipt.commit_id.sequence = 0,
            |event| event.raw_representation = crate::lcm_runtime::RawMapRepresentation::SeparateRaw,
            |event| event.raw_data = vec![0, 100].into(),
        ];
        for mutate in mutations {
            let mut invalid = event.clone();
            mutate(&mut invalid);
            let current = Arc::new(RwLock::new(None));
            let (tx, mut rx) = tokio::sync::mpsc::channel(1);
            assert!(forward_map_revision(&tx, invalid, &current, "map").await);
            let error = rx.recv().await.expect("stream result").expect_err("invalid receipt or representation must fail closed");
            assert_eq!(error.code(), tonic::Code::DataLoss);
        }
    }

    #[tokio::test]
    async fn map_update_omits_shm_when_current_revision_differs() {
        let event = test_mapping_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 41,
            timestamp_ns: 41,
            data: vec![0, 100],
            ..MapState::default()
        });
        for current_revision in [40, 42] {
            let current = Arc::new(RwLock::new(Some(
                crate::lcm_runtime::OccupancyShmSnapshot {
                    identity: crate::occupancy_shm::MapWriteIdentity::from_event(&event).unwrap(),
                    key: "/tmp/current-v3.mmap".to_string(),
                    file_size: 8192,
                    payload_size: event.size,
                    revision: current_revision,
                    layout_version: 3,
                    payload_schema_version: 1,
                    generation: 2,
                    active_slot: 1,
                },
            )));
            let (tx, mut rx) = tokio::sync::mpsc::channel(1);

            assert!(forward_map_revision(&tx, event.clone(), &current, "map").await);
            let update = rx
                .recv()
                .await
                .expect("stream update")
                .expect("no INTERNAL");
            assert_eq!(update.meta.as_ref().unwrap().revision, event.revision);
            assert_eq!(update.data.as_slice(), event.data.as_ref());
            assert_eq!(update.size, event.size);
            assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        assert_eq!(update.raw_representation, event.raw_representation as i32);
            assert!(
                update.shm_ref.is_none(),
                "revision {current_revision} must not be attached to event revision {}",
                event.revision
            );
        }
    }

    #[tokio::test]
    async fn late_map_update_does_not_attach_another_commit_or_geometry() {
        let event = test_mapping_event(MapState { width: 2, height: 1, resolution: 0.1,
            frame_id: "map".into(), revision: 41, timestamp_ns: 41,
            data: vec![0, 100], ..MapState::default() });
        let other_commit = test_mapping_event(event.to_map_state());
        assert_ne!(other_commit.receipt.commit_id, event.receipt.commit_id);
        let mut other_geometry = event.clone();
        other_geometry.origin_x += 1.0;
        for current_event in [other_commit, other_geometry] {
            let current = Arc::new(RwLock::new(Some(crate::lcm_runtime::OccupancyShmSnapshot {
                identity: crate::occupancy_shm::MapWriteIdentity::from_event(&current_event).unwrap(),
                key: "/tmp/newer-assigned-commit.mmap".into(), file_size: 8192,
                payload_size: current_event.size, revision: current_event.revision,
                layout_version: 3, payload_schema_version: 1, generation: 3, active_slot: 0,
            })));
            let (tx, mut rx) = tokio::sync::mpsc::channel(1);
            assert!(forward_map_revision(&tx, event.clone(), &current, "map").await);
            let update = rx.recv().await.unwrap().unwrap();
            assert!(update.shm_ref.is_none());
            assert_eq!(update.data.as_slice(), event.data.as_ref());
            assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        }
    }

    #[tokio::test]
    async fn map_update_carries_shm_when_revision_matches() {
        let event = test_mapping_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 41,
            timestamp_ns: 41,
            data: vec![0, 100],
            ..MapState::default()
        });
        let current = Arc::new(RwLock::new(Some(
            crate::lcm_runtime::OccupancyShmSnapshot {
                identity: crate::occupancy_shm::MapWriteIdentity::from_event(&event).unwrap(),
                key: "/tmp/current-v3.mmap".to_string(),
                file_size: 8192,
                payload_size: event.size,
                revision: event.revision,
                layout_version: 3,
                payload_schema_version: 1,
                generation: 2,
                active_slot: 1,
            },
        )));
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);

        assert!(forward_map_revision(&tx, event.clone(), &current, "map").await);
        let update = rx
            .recv()
            .await
            .expect("stream update")
            .expect("no INTERNAL");
        assert_eq!(update.meta.as_ref().unwrap().revision, event.revision);
        assert_eq!(update.data.as_slice(), event.data.as_ref());
        assert_eq!(update.size, event.size);
        assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        assert_eq!(update.raw_representation, event.raw_representation as i32);
        let shm_ref = update.shm_ref.expect("matching SHM reference");
        assert_eq!(shm_ref.revision, event.revision);
        assert_eq!(shm_ref.size, 8192);
        assert_eq!(shm_ref.layout_version, 3);
        assert_eq!(shm_ref.payload_schema_version, 1);
        assert_eq!(shm_ref.generation, 2);
        assert_eq!(shm_ref.active_slot, 1);
        assert_eq!(shm_ref.payload_size, event.size);
    }

    #[tokio::test]
    async fn stop_after_shm_commit_before_watch_claim_suppresses_subscription_output() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .start_session("post-commit-stop", "warehouse")
            .expect("active mapping session");
        let event = crate::lcm_runtime::test_owner_map_revision(
            &mut handle.mapping_adapter.as_ref().expect("mapping adapter").lock(),
            &MapState {
                width: 1,
                height: 1,
                resolution: 0.1,
                frame_id: "map".to_string(),
                revision: 1,
                timestamp_ns: 1,
                data: vec![100],
                ..MapState::default()
            },
        );
        *handle.occupancy_shm_current.write().await =
            Some(crate::lcm_runtime::OccupancyShmSnapshot {
                identity: crate::occupancy_shm::MapWriteIdentity::from_event(&event).unwrap(),
                key: "/tmp/post-commit-stop.mmap".to_string(),
                file_size: 8192,
                payload_size: event.size,
                revision: event.revision,
                layout_version: 3,
                payload_schema_version: 1,
                generation: 1,
                active_slot: 0,
            });
        handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .stop_session("post-commit-stop")
            .expect("Stop after SHM commit");
        handle
            .map_revision_latest_tx
            .send_replace(Some(event.clone()));

        assert!(
            !map_revision_claim_is_current(
                &event,
                handle.mapping_adapter.as_ref().expect("mapping adapter"),
                &handle.mapping_lifecycle_lock,
            )
            .await
        );
        let service = GeometryRuntimeService::new(handle);
        let mut stream = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("SubscribeMapUpdates RPC")
            .into_inner();
        assert!(
            tokio::time::timeout(Duration::from_millis(100), stream.next())
                .await
                .expect("stale initial stream must close")
                .is_none(),
            "post-Stop watch value must not be forwarded"
        );
    }

    #[test]
    fn occupancy_shm_v3_projection_rejects_old_or_inconsistent_references() {
        let event = crate::occupancy_shm::committed_event_for_tests(&MapState {
            width: 16, height: 16, resolution: 0.1, frame_id: "map".into(),
            revision: 7, timestamp_ns: 7, data: vec![0; 256], ..MapState::default()
        }, 1);
        let valid = crate::lcm_runtime::OccupancyShmSnapshot {
            identity: crate::occupancy_shm::MapWriteIdentity::from_event(&event).unwrap(),
            key: "/tmp/occupancy-v3.mmap".to_string(),
            file_size: 8192,
            payload_size: 256,
            revision: 7,
            layout_version: 3,
            payload_schema_version: 1,
            generation: 3,
            active_slot: 0,
        };
        assert!(occupancy_shm_ref(valid.clone()).is_ok());
        for invalid in [
            crate::lcm_runtime::OccupancyShmSnapshot {
                layout_version: 0,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                layout_version: 2,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                generation: 0,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                active_slot: 2,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                payload_size: 8192,
                ..valid
            },
        ] {
            assert_eq!(
                occupancy_shm_ref(invalid).unwrap_err().code(),
                tonic::Code::FailedPrecondition
            );
        }
    }
}

#[cfg(test)]
mod cloud_map_update_tests {
    use super::*;
    use crate::lcm_runtime::CloudMapNotification;
    use tokio_stream::StreamExt;

    async fn next_cloud_map_update(
        service: &GeometryRuntimeService,
        handle: &GeometryRuntimeHandle,
        notification: CloudMapNotification,
    ) -> CloudMapShmRef {
        let response =
            GeometryRuntime::subscribe_cloud_map_updates(service, Request::new(Empty {}))
                .await
                .expect("SubscribeCloudMapUpdates RPC");
        let mut stream = response.into_inner();
        handle
            .cloud_map_tx
            .send(notification)
            .expect("cloud-map subscriber registered before RPC returns");
        tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .expect("cloud-map update timeout")
            .expect("cloud-map stream item")
            .expect("cloud-map update")
    }

    fn assert_common_v3_reference(reference: &ShmRef, revision: u64, generation: u64) {
        assert_eq!(
            reference.key,
            crate::pointcloud::cloud_map_shm_path()
                .to_string_lossy()
                .into_owned()
        );
        assert_eq!(
            reference.size,
            u64::try_from(crate::pointcloud::CM_TOTAL_SIZE).unwrap()
        );
        assert_eq!(reference.revision, revision);
        assert_eq!(reference.layout_version, crate::pointcloud::CM_SHM_VERSION);
        assert_eq!(
            reference.payload_schema_version,
            crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION
        );
        assert_eq!(reference.generation, generation);
        assert_eq!(
            reference.active_slot,
            crate::pointcloud::cloud_map_active_slot(revision).unwrap()
        );
    }

    #[test]
    fn cloud_map_v3_metadata_reference_uses_slot_zero_and_u64_payload_size() {
        let reference = cloud_map_shm_ref(&CloudMapNotification {
            revision: 2,
            point_count: 3,
            voxel_size: 0.10,
            session_generation: 8,
        })
        .expect("content-free v3 cloud reference");
        assert_common_v3_reference(&reference, 2, 8);
        assert_eq!(reference.layout_version, 3);
        assert_eq!(reference.active_slot, 0);
        let payload_size: u64 = reference.payload_size;
        assert_eq!(payload_size, 36_u64);
    }

    #[tokio::test]
    async fn cloud_map_clear_update_carries_complete_v3_shm_reference() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let update = next_cloud_map_update(
            &service,
            &handle,
            CloudMapNotification {
                revision: 1,
                point_count: 0,
                voxel_size: 0.05,
                session_generation: 7,
            },
        )
        .await;

        assert_eq!(update.point_count, 0);
        assert_eq!(update.voxel_size, 0.05);
        let reference = update.shm_ref.expect("v3 SHM reference");
        assert_common_v3_reference(&reference, 1, 7);
        assert_eq!(reference.active_slot, 0);
        assert_eq!(reference.payload_size, 0);
    }

    #[tokio::test]
    async fn cloud_map_data_update_carries_payload_size_and_active_slot() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let update = next_cloud_map_update(
            &service,
            &handle,
            CloudMapNotification {
                revision: 2,
                point_count: 3,
                voxel_size: 0.10,
                session_generation: 8,
            },
        )
        .await;

        assert_eq!(update.point_count, 3);
        assert_eq!(update.voxel_size, 0.10);
        let reference = update.shm_ref.expect("v3 SHM reference");
        assert_common_v3_reference(&reference, 2, 8);
        assert_eq!(reference.active_slot, 0);
        assert_eq!(reference.payload_size, 36);
    }
}

#[cfg(test)]
fn server_test_committed_map_identity(
    revision: u64,
    frame_id: &str,
) -> schema::CommittedMapIdentity {
    crate::lcm_runtime::committed_map_identity_from_revision(
        &crate::lcm_runtime::test_map_revision_authority("server-test-map", "server-test-map", frame_id, revision),
    ).expect("server test committed receipt identity")
}

#[cfg(test)]
fn install_server_test_committed_map_in_frame(
    handle: &GeometryRuntimeHandle,
    revision: u64,
    frame_id: &str,
) {
    *handle
        .committed_map_authority
        .lock()
        .expect("server test committed map authority") =
        Some(server_test_committed_map_identity(revision, frame_id));
    handle.map_revision_latest_tx.send_replace(Some(
        crate::lcm_runtime::test_map_revision_authority("server-test-map", "server-test-map", frame_id, revision),
    ));
}

#[cfg(test)]
fn install_server_test_committed_map(handle: &GeometryRuntimeHandle, revision: u64) {
    install_server_test_committed_map_in_frame(handle, revision, "map");
}

#[cfg(test)]
mod lifecycle_tests {
// Include inside server::lifecycle_tests. This extends the existing real
// assigned export/transfer/activation fixture; it does not replace its owners.
// Run only with the validation-tree business-hash instrumentation documented
// in README.md. No public navigation endpoint is called.

fn assigned_cloud_request(authority: &MappingCloudAuthority) -> MappingCloudGoalRequest {
    MappingCloudGoalRequest {
        schema_version: 3,
        query_request_id: "assigned-cloud-server-boundary".into(),
        query_text: "fixture destination".into(),
        caller_timeout_ms: 1_000,
        provider_timeout_ms: 500,
        map_name: authority.map_name.clone(),
        map_frame_id: authority.map_frame_id.clone(),
        runtime_generation: authority.runtime_generation,
        mapping_session_id: authority.mapping_session_id.clone(),
        mapping_session_generation: authority.mapping_session_generation,
        committed_occupancy_revision: authority.committed_occupancy_revision,
        committed_occupancy_commit_id: authority.committed_occupancy_commit_id.clone(),
        committed_occupancy_source_timestamp_ns: authority.committed_occupancy_source_timestamp_ns,
        activation_receipt_v2: authority.activation_receipt_v2.clone(),
        georeference_json: serde_json::to_vec(&serde_json::json!({
            "schema_version": 1,
            "map_name": authority.map_name,
            "map_frame_id": authority.map_frame_id,
            "source_crs": "EPSG:4326",
            "projected_crs": "EPSG:3857",
            "axis_convention": "projected_x_east_y_north_z_up",
            "anchor_lat_deg": 0.0,
            "anchor_lng_deg": 0.0,
            "anchor_easting_m": 0.0,
            "anchor_northing_m": 0.0,
            "map_origin_x_m": 0.0,
            "map_origin_y_m": 0.0,
            "map_from_projected_yaw_rad": 0.0,
            "altitude_policy": "map_plane_constant",
            "map_plane_z_m": 0.0
        })).unwrap(),
        ..Default::default()
    }
}

fn assert_assigned_cloud_projection(
    actual: &MappingCloudAuthority,
    activated: &MappingSessionState,
    checkpoint: &MappingBackendCheckpoint,
    current: &schema::CommittedMapIdentity,
    owner_epoch: u64,
) {
    let receipt = activated.activation_receipt_v2.as_ref().unwrap();
    let request = receipt.request.as_ref().unwrap();
    let activation = receipt.activation.as_ref().unwrap();
    let target = receipt.target.as_ref().unwrap();
    let source = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
    assert_eq!(actual.schema_version, 3);
    // Exact whole-receipt equality covers the source checkpoint/member tuple
    // and both installation receipts; neither is reconstructed from current.
    assert_eq!(actual.activation_receipt_v2.as_ref(), Some(receipt));
    assert_eq!(target.source_cloud_member, source.member_identity);
    assert_ne!(target.occupancy_receipt.as_ref().unwrap().commit_id,
        checkpoint.occupancy_receipt.as_ref().unwrap().commit_id);
    assert_eq!(actual.map_name, request.map_name);
    assert_eq!(actual.activation_request_id, request.activation_request_id);
    assert_eq!(actual.runtime_generation, activation.runtime_generation);
    assert_eq!(actual.lifecycle_epoch, activation.lifecycle_epoch);
    assert_eq!(actual.mapping_session_id, request.session_id);
    assert_eq!(actual.mapping_session_generation, activation.session_generation);
    assert_eq!(actual.backend_kind, activation.backend_kind);
    assert_eq!(actual.backend_generation, activation.backend_generation);
    assert_eq!(actual.owner_epoch, owner_epoch);
    assert_eq!(actual.map_frame_id, current.frame_id);
    assert_eq!(actual.committed_occupancy_revision, u64::try_from(current.revision).unwrap());
    let commit = actual.committed_occupancy_commit_id.as_ref().unwrap();
    assert_eq!(commit.boot_nonce.as_slice(), current.commit_boot_nonce.as_slice());
    assert_eq!(commit.sequence, u64::try_from(current.commit_sequence).unwrap());
    assert_eq!(actual.committed_occupancy_source_timestamp_ns,
        u64::try_from(current.source_timestamp_ns).unwrap());
}

async fn assert_assigned_cloud_rpc_rejection(
    client: &mut crate::proto::geometry_runtime_client::GeometryRuntimeClient<tonic::transport::Channel>,
    request: MappingCloudGoalRequest,
    code: tonic::Code,
    message: &str,
) {
    let error = client.submit_mapping_cloud_goal(request).await.unwrap_err();
    assert_eq!(error.code(), code, "{error}");
    assert_eq!(error.message(), message);
}

async fn assigned_mapping_cloud_boundary_slice(
    fixture: &CanonicalResetFixture,
    activated: &MappingSessionState,
    checkpoint: &MappingBackendCheckpoint,
) {
    use crate::proto::geometry_runtime_client::GeometryRuntimeClient;
    use crate::proto::geometry_runtime_server::GeometryRuntimeServer;

    // Fence the actual owner before starting the RPC listener. Even a faulty
    // server admission path cannot issue a Nominatim request in this slice.
    assert_eq!(fixture.service.mapping_cloud_owner.shutdown(Duration::from_secs(1)).await,
        nuwax_mapping_cloud::MappingCloudShutdown::Settled);
    let original = fixture.handle.committed_map_authority.lock().unwrap().clone()
        .expect("real assigned fixture owns committed authority");
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap();
    let incoming = futures::stream::unfold(listener, |listener| async {
        Some((listener.accept().await.map(|(stream, _)| stream), listener))
    });
    let (stop, stopped) = tokio::sync::oneshot::channel();
    let service = fixture.service.clone();
    let server = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(GeometryRuntimeServer::new(service))
            .serve_with_incoming_shutdown(incoming, async { let _ = stopped.await; }).await
    });
    let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
        let mut client = GeometryRuntimeClient::connect(format!("http://{address}")).await.unwrap();
        let authority = client.get_mapping_cloud_authority(Empty {}).await.unwrap().into_inner();
        let epoch = fixture.handle.nav.producer_runtime_generation();
        assert_assigned_cloud_projection(&authority, activated, checkpoint, &original, epoch);
        let valid = assigned_cloud_request(&authority);

        let mut schema_two = valid.clone();
        schema_two.schema_version = 2;
        assert_assigned_cloud_rpc_rejection(&mut client, schema_two, tonic::Code::InvalidArgument,
            "mapping-cloud requires assigned schema_version 3").await;

        let mut wrong_receipt = valid.clone();
        wrong_receipt.activation_receipt_v2.as_mut().unwrap().target.as_mut().unwrap()
            .source_cloud_member.as_mut().unwrap().checkpoint_id = "b".repeat(32);
        assert_assigned_cloud_rpc_rejection(&mut client, wrong_receipt, tonic::Code::FailedPrecondition,
            "mapping-cloud request does not match current assigned map authority").await;

        // One case for each current commit field, rather than repeating legacy
        // field rejection cases. Exact error text distinguishes the server
        // boundary from the deliberately stopped owner's rejection.
        for component in 0..4 {
            let mut wrong_current = valid.clone();
            match component {
                0 => wrong_current.committed_occupancy_revision += 1,
                1 => wrong_current.committed_occupancy_commit_id.as_mut().unwrap().boot_nonce[0] ^= 1,
                2 => wrong_current.committed_occupancy_commit_id.as_mut().unwrap().sequence += 1,
                _ => wrong_current.committed_occupancy_source_timestamp_ns += 1,
            }
            assert_assigned_cloud_rpc_rejection(&mut client, wrong_current, tonic::Code::FailedPrecondition,
                "mapping-cloud request does not match current committed map authority").await;
        }

        assert_assigned_cloud_rpc_rejection(&mut client, valid.clone(), tonic::Code::FailedPrecondition,
            "mapping-cloud owner is shutting down").await;
        assert_eq!(fixture.service.mapping_cloud_owner.cancel(&valid.query_request_id).await.unwrap_err(),
            nuwax_mapping_cloud::MappingCloudOwnerError::UnknownRequest);

        // Controlled advancement of the existing current-authority slot tests
        // projection only. This is not claimed as a real live mapping commit.
        // The source owner and installed receipt remain the real fixture's.
        let mut advanced = original.clone();
        advanced.revision = advanced.revision.checked_add(1).unwrap();
        advanced.commit_sequence = advanced.commit_sequence.checked_add(1).unwrap();
        advanced.source_timestamp_ns = advanced.source_timestamp_ns.checked_add(1).unwrap();
        *fixture.handle.committed_map_authority.lock().unwrap() = Some(advanced.clone());
        let next = client.get_mapping_cloud_authority(Empty {}).await.unwrap().into_inner();
        assert_assigned_cloud_projection(&next, activated, checkpoint, &advanced, epoch);
        assert_eq!(next.activation_receipt_v2, authority.activation_receipt_v2);
        assert_ne!(next.committed_occupancy_commit_id, authority.committed_occupancy_commit_id);
        assert_assigned_cloud_rpc_rejection(&mut client, valid, tonic::Code::FailedPrecondition,
            "mapping-cloud request does not match current committed map authority").await;
        assert_assigned_cloud_rpc_rejection(&mut client, assigned_cloud_request(&next), tonic::Code::FailedPrecondition,
            "mapping-cloud owner is shutting down").await;
    })).await;
    *fixture.handle.committed_map_authority.lock().unwrap() = Some(original);
    let _ = stop.send(());
    server.await.unwrap().unwrap();
    if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
}

    mod assigned_navigation_composition {
        include!(concat!(env!("CARGO_MANIFEST_DIR"), "/src/assigned_navigation_composition.rs"));
    }
    use super::*;
    use crate::explore_runtime::{
        now_monotonic_nanos, now_unix_nanos, ChildNavigationCancelOutcome, ChildNavigationCancelTicket, ChildNavigationLease,
        ChildNavigationRequest, ChildNavigationResult, ChildNavigationSubmitError,
        ChildNavigationTerminalProof, ChildNavigationTicket, ChildNavigationZeroProof,
        ExploreCostmapSnapshot, ExploreNavigationSink, ExplorePoseProvider, ExploreRobotPose,
        STATUS_ACCEPTED, STATUS_ACTIVE, STATUS_FAILED, STATUS_STOPPING,
    };
    use crate::lcm_runtime::MapState;
    use crate::pointcloud::CloudMapShmWriter;
    use nalgebra::Point3;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
    use nuwax_protocol::{LcmMessageType, LcmTopic};
#[cfg(test)]
use nuwax_protocol::RawLcmFrame;
    use nuwax_transport::{
        LcmBackend, LcmFrameStream, LcmTransport, TransportError, TransportResult,
    };
    use std::sync::Mutex as StdMutex;
    use tokio_stream::StreamExt;


    type RelocalizeTransformFingerprint = (String, String, [u64; 3], [u64; 4], u64);
    type RelocalizeFrontierFingerprint = ([u64; 3], [u64; 4], String, u32, u32);

    #[derive(Debug, PartialEq, Eq)]
    struct RelocalizeSharedStateFingerprint {
        pose: ([u64; 7], String, u64),
        map: ([u32; 3], [u64; 2], String, u64, u64, Vec<i8>),
        tf_tree_identity: usize,
        tf_tree_latest: Vec<RelocalizeTransformFingerprint>,
        tf_tree_history_lengths: Vec<(String, usize)>,
        transforms: Vec<RelocalizeTransformFingerprint>,
        nav: ([String; 3], u32, u64),
        cloud_map_revision: u64,
        frontiers: Vec<RelocalizeFrontierFingerprint>,
        frontiers_revision: u64,
    }

    fn relocalize_transform_fingerprint(
        entry: &crate::lcm_runtime::TransformEntry,
    ) -> RelocalizeTransformFingerprint {
        (
            entry.parent.clone(),
            entry.child.clone(),
            [entry.tx.to_bits(), entry.ty.to_bits(), entry.tz.to_bits()],
            [
                entry.rx.to_bits(),
                entry.ry.to_bits(),
                entry.rz.to_bits(),
                entry.rw.to_bits(),
            ],
            entry.stamp_ns,
        )
    }

    fn relocalize_shared_state_fingerprint(
        state: &crate::lcm_runtime::SharedState,
    ) -> RelocalizeSharedStateFingerprint {
        let mut tf_tree_latest = state
            .latest_transform_entries()
            .iter()
            .map(relocalize_transform_fingerprint)
            .collect::<Vec<_>>();
        tf_tree_latest.sort();
        let mut tf_tree_history_lengths = state
            .tf_tree
            .frames()
            .into_iter()
            .map(|frame| {
                let history_len = state.tf_tree.history_len(&frame);
                (frame, history_len)
            })
            .collect::<Vec<_>>();
        tf_tree_history_lengths.sort();
        RelocalizeSharedStateFingerprint {
            pose: (
                [
                    state.pose.x.to_bits(),
                    state.pose.y.to_bits(),
                    state.pose.z.to_bits(),
                    state.pose.qx.to_bits(),
                    state.pose.qy.to_bits(),
                    state.pose.qz.to_bits(),
                    state.pose.qw.to_bits(),
                ],
                state.pose.frame_id.clone(),
                state.pose.timestamp_ns,
            ),
            map: (
                [
                    state.map.width,
                    state.map.height,
                    state.map.resolution.to_bits(),
                ],
                [state.map.origin_x.to_bits(), state.map.origin_y.to_bits()],
                state.map.frame_id.clone(),
                state.map.revision,
                state.map.timestamp_ns,
                state.map.data.clone(),
            ),
            tf_tree_identity: Arc::as_ptr(&state.tf_tree) as usize,
            tf_tree_latest,
            tf_tree_history_lengths,
            transforms: state
                .transforms
                .iter()
                .map(relocalize_transform_fingerprint)
                .collect(),
            nav: (
                [
                    state.nav_state.status.clone(),
                    state.nav_state.message.clone(),
                    state.nav_state.goal_id.clone(),
                ],
                state.nav_state.distance_remaining.to_bits(),
                state.nav_state.timestamp_ns,
            ),
            cloud_map_revision: state.cloud_map_revision,
            frontiers: state
                .frontiers
                .iter()
                .map(|frontier| {
                    (
                        [
                            frontier.x.to_bits(),
                            frontier.y.to_bits(),
                            frontier.z.to_bits(),
                        ],
                        [
                            frontier.qx.to_bits(),
                            frontier.qy.to_bits(),
                            frontier.qz.to_bits(),
                            frontier.qw.to_bits(),
                        ],
                        frontier.frame_id.clone(),
                        frontier.size_cells,
                        frontier.heading.to_bits(),
                    )
                })
                .collect(),
            frontiers_revision: state.frontiers_revision,
        }
    }

    #[tokio::test]
    async fn relocalize_without_selected_owner_has_zero_side_effects() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                x: 1.25,
                y: -2.5,
                z: 0.75,
                qz: 0.5,
                qw: 0.5,
                frame_id: "map".to_owned(),
                timestamp_ns: 17,
                ..PoseState::default()
            };
            shared.map = MapState {
                width: 2,
                height: 2,
                resolution: 0.05,
                origin_x: -1.0,
                origin_y: 2.0,
                frame_id: "map".to_owned(),
                revision: 19,
                timestamp_ns: 23,
                data: vec![0, 100, -1, 42],
            };
            shared.nav_state = crate::lcm_runtime::NavState {
                status: "ACTIVE".to_owned(),
                message: "relocalize side-effect sentinel".to_owned(),
                goal_id: "goal-before-relocalize".to_owned(),
                distance_remaining: 3.25,
                timestamp_ns: 29,
            };
            shared.cloud_map_revision = 31;
            shared.frontiers = vec![crate::lcm_runtime::FrontierEntry {
                x: 4.0,
                y: 5.0,
                z: 6.0,
                qz: 0.25,
                qw: 0.75,
                frame_id: "map".to_owned(),
                size_cells: 7,
                heading: 1.5,
                ..crate::lcm_runtime::FrontierEntry::default()
            }];
            shared.frontiers_revision = 37;
        }
        let service = GeometryRuntimeService::new(handle.clone());
        let target_xyz_f32_le = [0.25_f32, 0.5, 0.75, 1.25, 1.5, 1.75]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let wire=cloud_activation_request(target_xyz_f32_le);
        let admitted=crate::activation_request::ValidatedActivationRequestV2::admit(wire,1,
            crate::mapping_backend::MappingBackendKind::Voxel,"map",|_|Err("无制品".into()),&service.activation_checkpoint_owners).unwrap();
        let retained_bytes=admitted.checkpoint().cloud().xyz_f32_le().len();
        let token=match service.activation_receipts.stage_assigned_or_replay(admitted,MappingSessionState::default(),true) {
            crate::correction_coordinator::ActivationLedgerAdmission::Execute(token)=>token,
            _=>panic!("编号请求应进入账本"),
        };
        let receipt_epoch_before = service.activation_receipts.lifecycle_epoch();
        assert_eq!(
            service.activation_receipts.checkpoint_owner(&token).unwrap().cloud().xyz_f32_le().len(),
            retained_bytes
        );
        let mapping_before = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        let shared_before = {
            let shared = handle.state.read().await;
            relocalize_shared_state_fingerprint(&shared)
        };
        let mut map_revision_rx = handle.map_revision_tx.subscribe();
        let mut cloud_revision_rx = handle.cloud_map_tx.subscribe();

        let error =
            GeometryRuntime::relocalize(&service, Request::new(crate::relocalization_identity::tests::wire()))
                .await
                .expect_err("Relocalize requires a selected owner");

        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(error.message(), "RELOCALIZATION_OWNER_NOT_SELECTED");
        assert_eq!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot(),
            mapping_before
        );
        let shared_after = {
            let shared = handle.state.read().await;
            relocalize_shared_state_fingerprint(&shared)
        };
        assert_eq!(shared_after, shared_before);
        assert_eq!(
            service.activation_receipts.lifecycle_epoch(),
            receipt_epoch_before
        );
        assert_eq!(
            service.activation_receipts.checkpoint_owner(&token).unwrap().cloud().xyz_f32_le().len(),
            retained_bytes
        );
        assert!(service.activation_receipts.validate_token(&token));
        assert!(matches!(
            map_revision_rx.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));
        assert!(matches!(
            cloud_revision_rx.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));

        assert!(handle
            .mapping_lifecycle_lock
            .clone()
            .try_lock_owned()
            .is_ok());
    }


    #[derive(Default)]
    struct LifecycleExploreSink {
        last_lease: StdMutex<Option<ChildNavigationLease>>,
        cancel_error: StdMutex<Option<String>>,
        cancel_never_returns: std::sync::atomic::AtomicBool,
        cancel_receipt_before_block: std::sync::atomic::AtomicBool,
        retained_cancel_ticket: StdMutex<Option<ChildNavigationCancelTicket>>,
        cancel_entered: tokio::sync::Notify,
    }

    struct SubmitLeaseThenBlockSink {
        facade: GeometryRuntimeHandle,
        entered: tokio::sync::Notify,
    }

    struct LifecycleStaticPoseProvider;

    #[derive(Default)]
    struct PhaseALioSink {
        closed: std::sync::atomic::AtomicBool,
    }

    #[async_trait::async_trait]
    impl crate::lio_owner::LioIngressSink for PhaseALioSink {
        fn close(&self) {
            self.closed
                .store(true, std::sync::atomic::Ordering::Release);
        }

        async fn accept_public_odometry(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_public_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_odometry(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }
    }


    static MOTION_GUARD_SOCKET_ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

    #[derive(Default)]
    struct CheckpointPublishBackend {
        fail: std::sync::atomic::AtomicBool,
        fail_after_observe: std::sync::atomic::AtomicBool,
        fail_after_pointcloud_observe: std::sync::atomic::AtomicBool,
        published: StdMutex<Vec<RawLcmFrame>>,
    }

    #[derive(Default)]
    struct BlockingCancelPublishBackend {
        published: StdMutex<Vec<RawLcmFrame>>,
    }

    impl BlockingCancelPublishBackend {
        fn published_cancel_count(&self) -> usize {
            self.published
                .lock()
                .expect("blocking cancel published frames")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation")
                .count()
        }
    }

    #[async_trait::async_trait]
    impl LcmBackend for BlockingCancelPublishBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            let blocks = frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation";
            self.published
                .lock()
                .expect("blocking cancel published frames")
                .push(frame);
            if blocks {
                return std::future::pending().await;
            }
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "blocking-cancel-publish-test"
        }
    }

    #[async_trait::async_trait]
    impl LcmBackend for CheckpointPublishBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            if self.fail.load(std::sync::atomic::Ordering::Acquire) {
                return Err(TransportError::LcmBackend(
                    "injected checkpoint publication failure".to_string(),
                ));
            }
            let fail_pointcloud = frame.msg_type.fqn() == "sensor_msgs.PointCloud2"
                && self.fail_after_pointcloud_observe.load(std::sync::atomic::Ordering::Acquire);
            self.published
                .lock()
                .expect("checkpoint published frames")
                .push(frame);
            if self
                .fail_after_observe
                .load(std::sync::atomic::Ordering::Acquire) || fail_pointcloud
            {
                return Err(TransportError::LcmBackend(
                    "injected post-observation checkpoint publication failure".to_owned(),
                ));
            }
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "checkpoint-publish-test"
        }
    }

    impl LifecycleExploreSink {
        fn last_lease(&self) -> ChildNavigationLease {
            self.last_lease
                .lock()
                .expect("lifecycle exploration sink lock")
                .clone()
                .expect("submitted child lease")
        }
    }

    #[async_trait::async_trait]
    impl ExploreNavigationSink for LifecycleExploreSink {
        fn retain_child_dispatch(
            &self,
            operation: crate::explore_runtime::ChildNavigationDispatch,
        ) -> Result<crate::explore_runtime::ChildNavigationDispatchObserver, String> {
            // Legacy fake-sink contract; this fixture does not prove root ownership.
            Ok(crate::explore_runtime::ChildNavigationDispatchObserver::inline(operation))
        }

        fn current_map_name(&self) -> Result<String, String> {
            Ok("map".to_string())
        }

        async fn submit(
            &self,
            request: ChildNavigationRequest,
        ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError> {
            let lease = ChildNavigationLease {
                session_id: request.request_id,
                owner_kind: 2,
                parent_request_id: request.parent_request_id,
                goal_generation: 1,
                producer_runtime_generation: 1,

            };
            *self
                .last_lease
                .lock()
                .expect("lifecycle exploration sink lock") = Some(lease.clone());
            Ok(ChildNavigationTicket { lease })
        }

        async fn cancel(
            &self,
            lease: &ChildNavigationLease,
        ) -> Result<ChildNavigationCancelOutcome, String> {
            let ticket = ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: 1,
            };
            if self
                .cancel_receipt_before_block
                .load(std::sync::atomic::Ordering::Acquire)
            {
                *self
                    .retained_cancel_ticket
                    .lock()
                    .expect("lifecycle exploration cancel ticket lock") = Some(ticket);
                self.cancel_entered.notify_one();
                return std::future::pending().await;
            }
            if self
                .cancel_never_returns
                .load(std::sync::atomic::Ordering::Acquire)
            {
                return std::future::pending().await;
            }
            if let Some(error) = self
                .cancel_error
                .lock()
                .expect("lifecycle exploration sink lock")
                .clone()
            {
                return Err(error);
            }
            Ok(ChildNavigationCancelOutcome::Pending(ticket))
        }

        fn recover_cancel_receipt(
            &self,
            lease: &ChildNavigationLease,
        ) -> Option<crate::explore_runtime::ChildNavigationCancelRecovery> {
            self.retained_cancel_ticket
                .lock()
                .expect("lifecycle exploration cancel ticket lock")
                .clone()
                .filter(|ticket| ticket.lease.matches(lease))
                .map(
                    |ticket| crate::explore_runtime::ChildNavigationCancelRecovery {
                        ticket,
                        failure: None,
                    },
                )
        }
    }

    #[async_trait::async_trait]
    impl ExploreNavigationSink for SubmitLeaseThenBlockSink {
        fn retain_child_dispatch(
            &self,
            operation: crate::explore_runtime::ChildNavigationDispatch,
        ) -> Result<crate::explore_runtime::ChildNavigationDispatchObserver, String> {
            // Legacy fake-sink contract; this fixture does not prove root ownership.
            Ok(crate::explore_runtime::ChildNavigationDispatchObserver::inline(operation))
        }

        fn current_map_name(&self) -> Result<String, String> {
            Ok("map".to_string())
        }

        async fn submit(
            &self,
            request: ChildNavigationRequest,
        ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError> {
            let target = request.goal.target;
            let pose = target.pose;
            let producer_epoch_projection = self.facade.explore.producer_epoch_projection();
            let producer_epoch = producer_epoch_projection.load(Ordering::Acquire);
            let committed_map = self
                .facade
                .committed_map_authority
                .lock()
                .expect("response-loss committed map authority")
                .clone()
                .ok_or_else(|| {
                    ChildNavigationSubmitError::Rejected(
                        "response-loss child is missing committed map authority".to_string(),
                    )
                })?;
            let timestamp_ns = target.stamp.nanos.max(0) as u64;
            let goal = schema::PoseStamped {
                header: schema::Header {
                    seq: 0,
                    stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000).map_err(|_| {
                        ChildNavigationSubmitError::Rejected(
                            "response-loss child timestamp exceeds wire range".to_string(),
                        )
                    })?,
                    stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
                    frame_id: committed_map.frame_id.clone(),
                },
                pose: schema::Pose {
                    position: [pose.position.x, pose.position.y, pose.position.z],
                    orientation_xyzw: [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ],
                },
            };
            let admission_context = crate::nav_runtime::NavAdmissionContext::new(
                self.facade.mapping_lifecycle_lock.clone(),
                self.facade.committed_map_authority.clone(),
                self.facade.map_revision_latest_tx.subscribe(),
            );
            let admission_io = crate::nav_runtime::NavAdmissionIo::new(
                self.facade.state.clone(),
                self.facade.transport.clone(),
                self.facade.transport_backend_name,
                self.facade.lcm_backend_available, Arc::from("base_link"),
);
            let outcome = crate::nav_runtime::admit_goal_intent(
                &admission_context,
                &admission_io,
                &self.facade.nav,
                schema::GoalIntent {
                    map_frame_generation: 0,
                        options: Default::default(),
                    producer: "wavefront".to_string(),
                    producer_epoch: i64::try_from(producer_epoch).map_err(|_| {
                        ChildNavigationSubmitError::Rejected(
                            "response-loss producer epoch exceeds wire range".to_string(),
                        )
                    })?,
                    priority: 100,
                    session_id: request.request_id,
                    owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                    parent_request_id: request.parent_request_id,
                    goal_generation: 0,
                    cancel_generation: i64::try_from(self.facade.nav.cancel_snapshot().generation)
                        .map_err(|_| {
                            ChildNavigationSubmitError::Rejected(
                                "response-loss cancel generation exceeds wire range".to_string(),
                            )
                        })?,
                    committed_map,
                    boundary_id: String::new(),
                    body_to_support_z_m: None,
                    boundary_points: Vec::new(),

                    motion_guard_boot_nonce: request.motion_admission.guard_boot_nonce,
                    motion_stop_generation: request.motion_admission.motion_stop_generation as i64,
                    goal,
                },
                crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
                boundary_owner: Arc::new(crate::explore_runtime::ExploreStartConfig::default()),
                    current_session_generation: producer_epoch_projection,
                },
            )
            .await;
            if !matches!(
                outcome.state.status.as_str(),
                "ACCEPTED" | "PLANNING" | "RUNNING"
            ) {
                return Err(ChildNavigationSubmitError::Rejected(outcome.state.message));
            }
            let lease = outcome.lease.ok_or_else(|| {
                ChildNavigationSubmitError::Rejected(
                    "accepted response-loss child is missing its lease".to_string(),
                )
            })?;
            self.entered.notify_one();
            let _retained_lease = lease;
            std::future::pending().await
        }

        async fn cancel(
            &self,
            _lease: &ChildNavigationLease,
        ) -> Result<ChildNavigationCancelOutcome, String> {
            Err("response-loss child is cancelled through NavRuntime ownership".to_string())
        }
    }

    #[async_trait::async_trait]
    impl ExplorePoseProvider for LifecycleStaticPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            Ok(ExploreRobotPose {
                x: 2.0,
                y: 2.0,
                z: 0.0,
                orientation: nuwax_nav_types_messages::geometry::Quaternion::IDENTITY,
                frame_id: "map".to_string(),
                source_timestamp_ns: now_unix_nanos(),
                received_monotonic_ns: now_monotonic_nanos(),
            })
        }
    }

    async fn start_mapping_session(
        service: &GeometryRuntimeService,
        session_id: &str,
        map_name: &str,
    ) -> MappingSessionState {
        GeometryRuntime::start_mapping(
            service,
            Request::new(StartMappingRequest {
                session_id: session_id.to_string(),
                map_name: map_name.to_string(),
            }),
        )
        .await
        .expect("StartMapping")
        .into_inner()
    }

    fn commit_test_map(handle: &GeometryRuntimeHandle, x: f64) -> u64 {
        let mut adapter = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock();
        adapter.feed_pointcloud(&[Point3::new(x, 0.0, 2.0)]);
        assert!(adapter.build_and_publish(&handle.state, Some(&handle.map_revision_tx)));
        adapter.current_revision()
    }

    #[tokio::test]
    async fn start_mapping_deferred_style_commit_returns_aligned_data_and_descriptor() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "deferred-alignment", "warehouse").await;

        // Deferred startup replay commits canonical Mapping output before its
        // caller can issue immediate unary reads. Hold the lifecycle boundary
        // here to make that canonical-before-SHM interval deterministic.
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let revision = commit_test_map(&handle, 0.0);
        assert_eq!(revision, 1);
        let data_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::get_occupancy_grid_data(&service, Request::new(Empty {})).await
            })
        };
        let shm_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::get_occupancy_grid_shm(&service, Request::new(Empty {})).await
            })
        };
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(!data_task.is_finished());
        assert!(!shm_task.is_finished());

        drop(lifecycle_guard);
        let data = data_task
            .await
            .expect("data getter join")
            .expect("data getter status")
            .into_inner();
        let shm = shm_task
            .await
            .expect("SHM getter join")
            .expect("SHM getter status")
            .into_inner();
        assert_eq!(data.meta.as_ref().map(|meta| meta.revision), Some(revision));
        assert_eq!(shm.revision, revision);
        assert_eq!(
            handle
                .occupancy_shm_current
                .read()
                .await
                .as_ref()
                .map(|snapshot| snapshot.revision),
            Some(revision)
        );
    }

    fn frontier_costmap(_fixture: u64) -> ExploreCostmapSnapshot {
        let mut grid = OccupancyGrid2D {
            resolution: 0.5,
            origin: Point2 { x: 0.0, y: 0.0 },
            width: 12,
            height: 12,
            cells: vec![OccupancyCell::Unknown; 144],
        };
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        ExploreCostmapSnapshot {
            grid,
            frame_id: "map".to_string(),
        }
    }

    async fn ingest_server_test_committed_costmap(handle: &GeometryRuntimeHandle, revision: u64) {
        handle
            .explore
            .ingest_committed_costmap_snapshot(
                frontier_costmap(revision),
                server_test_committed_map_identity(revision, "map"),
            )
            .await
            .expect("committed frontier costmap");
    }


    #[tokio::test]
    async fn process_stop_post_zero_lifecycle_reacquire_timeout_is_bounded() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let barrier = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_stop_before_post_zero_lifecycle_barrier(barrier.clone());
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        barrier.wait().await;
        let held_lifecycle = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        barrier.wait().await;
        let error = stop_task
            .await
            .expect("post-ZERO Stop join")
            .expect_err("post-ZERO lifecycle reacquire must time out");
        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(error
            .message()
            .contains("after exact Process Stop motion terminal"));
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err()
        );
        drop(held_lifecycle);
    }

    // These fixtures construct only the injected in-process transport. The
    // capability flag admits its implemented publish path, not a real LCM link.
    struct CanonicalResetFixture {
        backend: Arc<CheckpointPublishBackend>,
        service: GeometryRuntimeService,
        handle: GeometryRuntimeHandle,
        tasks: crate::lcm_runtime::GeometryTestTaskOwner,
        task_owner: crate::lcm_runtime::GeometryRuntimeTaskOwner,
    }

    impl CanonicalResetFixture {
        async fn new() -> Self {
            Self::with_backend(crate::mapping_backend::MappingBackendSpec::Voxel).await
        }

        async fn with_backend(spec: crate::mapping_backend::MappingBackendSpec) -> Self {
            let backend = Arc::new(CheckpointPublishBackend::default());
            let transport = LcmTransport::new(backend.clone());
            let (mut handle, tasks) = crate::lcm_runtime::build_handle_for_tests_with_unit_transport(
                true, Some(spec), transport,
            ).await.expect("in-process reset fixture");
            handle.lcm_backend_available = true;
            let owned = crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
            let service = GeometryRuntimeService::new(owned.handle.clone());
            Self { backend, service, handle: owned.handle, tasks, task_owner: owned.task_owner }
        }

        async fn finish(mut self) {
            // The retained transition owns this mutex until all its work exits.
            let transition = self.service.mapping_transition.clone().lock_owned().await;
            // A failed publication may intentionally retain L. Release this
            // fixture's exact slot before shutting down its remaining tasks.
            let outcome = self.task_owner.shutdown().await;
            assert!(outcome.producers_settled(), "reset fixture registered producers joined");
            outcome.result.expect("reset fixture producer shutdown");
            let mapping = self.handle.mapping_adapter.as_ref().unwrap().clone();
            let retirement = {
                let _lifecycle = self.handle.mapping_lifecycle_lock.lock().await;
                let mut adapter = mapping.lock();
                let current = adapter.snapshot();
                if current.active {
                    adapter.stop_session_detached(&current.session_id)
                        .expect("reset fixture stop").retirement
                } else { None }
            };
            finish_backend_retirement(&mapping, &self.handle.mapping_lifecycle_lock, retirement)
                .await.expect("reset fixture worker joined");
            drop(transition);
            drop(mapping);
            drop(self.service);
            drop(self.handle);
            drop(self.task_owner);
            self.tasks.finish().await.expect("reset fixture fanout joined and files released");
        }
    }

    async fn seed_canonical_reset_pose(handle: &GeometryRuntimeHandle, timestamp_ns: u64) {
        let mut shared = handle.state.write().await;
        shared.pose = crate::lcm_runtime::PoseState {
            x: 8.0, qw: 1.0, frame_id: "map".into(), timestamp_ns,
            ..Default::default()
        };
        crate::lcm_runtime::apply_sensor_extrinsic_locked(&mut shared, &crate::lcm_runtime::TransformEntry {
            parent: "map".into(), child: "odom".into(), tx: 8.0, rw: 1.0,
            stamp_ns: timestamp_ns, ..Default::default()
        });
        shared.pose_changed.send_replace(());
    }

    async fn assert_canonical_reset_visible(service: &GeometryRuntimeService) {
        let pose = GeometryRuntime::get_current_pose(service, Request::new(PoseRequest::default()))
            .await.unwrap().into_inner();
        assert!(pose.frame_id.is_empty());
        assert_eq!(pose.timestamp_ns, 0);
        let tree = GeometryRuntime::get_frame_tree(service, Request::new(Empty {}))
            .await.unwrap().into_inner();
        assert!(tree.transforms.is_empty());
        assert!(service.bridge.state.read().await.transforms.is_empty());
    }

    #[tokio::test]
    async fn canonical_reset_new_start_clears_pose_but_replay_and_rejection_preserve_it() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            seed_canonical_reset_pose(&fixture.handle, 41).await;
            let mut changed = fixture.handle.state.read().await.pose_changed.subscribe();
            let started = start_mapping_session(&fixture.service, "reset-new", "room").await;
            assert!(started.active);
            assert!(changed.has_changed().unwrap());
            changed.borrow_and_update();
            assert_canonical_reset_visible(&fixture.service).await;
            seed_canonical_reset_pose(&fixture.handle, 42).await;
            changed.borrow_and_update();
            let replay = start_mapping_session(&fixture.service, "reset-new", "room").await;
            assert!(replay.idempotent_replay);
            let rejected = GeometryRuntime::start_mapping(&fixture.service, Request::new(StartMappingRequest {
                session_id: "other".into(), map_name: "other".into(),
            })).await;
            assert!(rejected.is_err());
            assert_eq!(fixture.handle.state.read().await.pose.timestamp_ns, 42);
            assert!(!fixture.handle.state.read().await.latest_transform_entries().is_empty());
            assert!(!changed.has_changed().unwrap());
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    #[tokio::test]
    async fn canonical_reset_cloud_activation_clears_pose_but_replay_and_rejection_preserve_it() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            seed_canonical_reset_pose(&fixture.handle, 51).await;
            let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
                .into_iter().flat_map(f32::to_le_bytes).collect::<Vec<_>>();
            let request = modern_activation_request(
                cloud_activation_request(xyz), "reset-modern",
                fixture.service.activation_receipts.runtime_generation(),
            );
            let activated = GeometryRuntime::activate_map(&fixture.service, Request::new(request.clone()))
                .await.expect("in-process cloud activation").into_inner();
            assert!(activated.success, "in-process activation outcome: {activated:?}");
            assert_canonical_reset_visible(&fixture.service).await;
            seed_canonical_reset_pose(&fixture.handle, 52).await;
            let replay = GeometryRuntime::activate_map(&fixture.service, Request::new(request.clone()))
                .await.unwrap().into_inner();
            assert!(replay.idempotent_replay);
            let mut malformed = request;
            malformed.session_id.clear();
            assert!(GeometryRuntime::activate_map(&fixture.service, Request::new(malformed)).await.is_err());
            assert_eq!(fixture.handle.state.read().await.pose.timestamp_ns, 52);
            assert!(!fixture.handle.state.read().await.latest_transform_entries().is_empty());
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    fn assigned_wire_roundtrip<T: Message + Default>(message: &T) -> T {
        T::decode(message.encode_to_vec().as_slice()).expect("assigned endpoint protobuf roundtrip")
    }

    async fn assigned_artifact_request(
        service: &GeometryRuntimeService,
        reference: &GraphArtifactRef,
        action: u32,
        offset: u64,
        data: Vec<u8>,
    ) -> Result<GraphArtifactTransferResponse, Status> {
        let request = GraphArtifactTransferRequest {
            action, artifact: Some(reference.clone()), offset, data,
        };
        GeometryRuntime::transfer_graph_artifact(service, Request::new(assigned_wire_roundtrip(&request)))
            .await.map(|response| assigned_wire_roundtrip(&response.into_inner()))
    }

    async fn transfer_assigned_checkpoint_member(
        source: &GeometryRuntimeService,
        destination: &GeometryRuntimeService,
        reference: &GraphArtifactRef,
        magic: &[u8; 8],
    ) {
        assert_eq!(reference.identity_schema_version, 2);

        let begun = assigned_artifact_request(destination, reference, 1, 0, Vec::new()).await.unwrap();
        assert_eq!(begun.artifact.as_ref(), Some(reference));
        assert!(!begun.sealed);
        let mut offset = 0;
        while offset < reference.size {
            let chunk = assigned_artifact_request(source, reference, 4, offset, Vec::new()).await.unwrap();
            assert_eq!(chunk.artifact.as_ref(), Some(reference));
            assert!(!chunk.data.is_empty());
            if offset == 0 { assert_eq!(&chunk.data[..8], magic); }
            let length = chunk.data.len() as u64;
            let written = assigned_artifact_request(destination, reference, 2, offset, chunk.data.clone()).await.unwrap();
            assert_eq!(written.offset, offset + length);
            // Exact chunk retry is idempotent, while a different body under the
            // same member/offset cannot overwrite the accepted bytes.
            let replay = assigned_artifact_request(destination, reference, 2, offset, chunk.data.clone()).await.unwrap();
            assert_eq!(replay.offset, written.offset);
            let mut changed = chunk.data;
            changed[0] ^= 1;
            let error = assigned_artifact_request(destination, reference, 2, offset, changed).await.unwrap_err();
            assert_eq!(error.code(), tonic::Code::FailedPrecondition);
            offset += length;
        }
        assert_eq!(offset, reference.size);
        let sealed = assigned_artifact_request(destination, reference, 3, 0, Vec::new()).await.unwrap();
        assert!(sealed.sealed);
        assert_eq!(sealed.artifact.as_ref(), Some(reference));
        // The destination must own its own sealed bytes, independent of the exporter.
        assigned_artifact_request(source, reference, 5, 0, Vec::new()).await.unwrap();
    }

    async fn assigned_navigation_readiness(handle: &GeometryRuntimeHandle) {
        let revision = handle.state.read().await.map.revision;
        handle.nav.observe_stack_state(crate::lcm_runtime::NavState {
            status:"IDLE".into(), timestamp_ns:now_ns(), ..Default::default()
        }).await;
        handle.nav.observe_tf_chain(crate::nav_runtime::TfChainObservation {
            generation:1, map_frame:"map".into(), odom_frame:"odom".into(), base_frame:"base_link".into(),
            map_to_odom:true, odom_to_base:true,
        }).unwrap();
        handle.nav.observe_planner_input(crate::nav_runtime::PlannerInputObservation {
            generation:1, map_revision:revision, map_frame:"map".into(), pose_frame:"map".into(), ready:true,
        }).unwrap();
        handle.nav.observe_motion_sink(crate::nav_runtime::MotionSinkObservation {generation:1,ready:true}).unwrap();
    }

    async fn assigned_relocalization_navigation_slice(
        fixture: &CanonicalResetFixture, activated: &MappingSessionState,
        owner: &crate::correction_coordinator::CorrectionTransportOwner,
        applier: &Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>,
    ) {
        use crate::proto::geometry_runtime_client::GeometryRuntimeClient;
        use crate::proto::geometry_runtime_server::GeometryRuntimeServer;
        // Exercise the generated client and tonic server over loopback. Only
        // the LCM publisher and readiness observations are injected; the actor,
        // source admission, sink, registration, canonical applier and goal
        // admission are the actual production implementations.
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let incoming = futures::stream::unfold(listener, |listener| async {
            Some((listener.accept().await.map(|(stream,_)|stream), listener))
        });
        let (stop, stopped) = tokio::sync::oneshot::channel();
        let service = fixture.service.clone();
        let server = tokio::spawn(async move {
            tonic::transport::Server::builder().add_service(GeometryRuntimeServer::new(service))
                .serve_with_incoming_shutdown(incoming, async {let _ = stopped.await;}).await
        });
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let mut client = GeometryRuntimeClient::connect(format!("http://{address}")).await.unwrap();
            let request = RelocalizeRequest {schema_version:2, request_id:"assigned-live-first".into(),
                assigned_target:Some(relocalize_request::AssignedTarget::AssignedCloudV2(AssignedCloudRelocalizeRequestV2 {
                    target:activated.activation_receipt_v2.as_ref().unwrap().target.clone(),
                    policy:activated.relocalization_policy_v2.clone(), logical_timeout_ms:30_000,
                })), ..Default::default()};
            let common = crate::lio_ingress::CommonLioIngress::new(
                crate::lio_ingress::DeviceLioIngressSpec::new(1,"sensor","base_link","odom","map").unwrap());
            common.bind_relocalization_mode_fence(owner.ingress().relocalization_mode_fence()).unwrap();
            assert_eq!(client.get_assigned_localization_sample(request.clone()).await.unwrap_err().code(),
                tonic::Code::FailedPrecondition, "same-frame pre-apply pose cannot establish ownership");
            assigned_navigation_readiness(&fixture.handle).await;
            let coordinate_goal = |id: &str, pose: Pose| NavigationGoal {
                pose:Some(pose), frame_id:"map".into(), goal_id:id.into(),
                 motion_guard_boot_nonce:vec![7;16], motion_stop_generation:1,
                ..Default::default()
            };
            let unlocalized_pose = client.get_current_pose(PoseRequest::default()).await.unwrap().into_inner();
            let blocked = client.set_navigation_goal(coordinate_goal("assigned-before-apply", unlocalized_pose)).await.unwrap().into_inner();
            assert_eq!(blocked.status,"FAILED");
            assert!(blocked.message.contains("等待真实定位"), "{blocked:?}");

            let retained = assigned_rpc_relocalize(&mut client, fixture, owner, applier, &common, request.clone(), 1).await;
            assert_eq!(retained.assigned_cloud_v2.as_ref().unwrap().correction_revision,1);
            let sample = client.get_assigned_localization_sample(request.clone()).await.unwrap().into_inner();
            assert_eq!(sample.terminal.as_ref(),Some(&retained));
            assert_eq!(sample.schema_version,2);
            assert_eq!(sample.correction_revision,1);
            assert!(!fixture.handle.mapping_adapter.as_ref().unwrap().lock().checkpoint_localization_required);
            // A lost response is settled by exact query/replay, without another apply.
            let replay = client.relocalize(request.clone()).await.unwrap().into_inner();
            assert_eq!(replay.encode_to_vec(),retained.encode_to_vec());
            assert_eq!(applier.snapshot().unwrap().correction_revision,1);

            let sample_pose = sample.pose.as_ref().unwrap();
            let next_stamp = sample_pose.timestamp_ns + 1_000_000_000;
            let moved = PoseState{x:5.0,y:2.0,z:1.0,qw:1.0,frame_id:"odom".into(),timestamp_ns:next_stamp,..Default::default()};
            {
                let _lifecycle = fixture.handle.mapping_lifecycle_lock.lock().await;
                assert!(crate::lcm_runtime::handoff_lio_odometry_with_correction(&fixture.handle,&moved,"base_link",
                    &(LcmTopic::new("/test/assigned/moving-tf").unwrap(),LcmMessageType::new("tf2_msgs.TFMessage").unwrap()),applier).await.unwrap());
            }
            let moving = client.get_assigned_localization_sample(request.clone()).await.unwrap().into_inner();
            assert_eq!(moving.pose.as_ref().unwrap().timestamp_ns,next_stamp);
            assert!((moving.pose.as_ref().unwrap().position.as_ref().unwrap().x - sample_pose.position.as_ref().unwrap().x - 1.0).abs()<1e-8);
            assert_eq!(moving.map_from_odom.as_ref().unwrap().timestamp_ns,next_stamp);
            assert_eq!(moving.odom_from_body.as_ref().unwrap().timestamp_ns,next_stamp);
            // Same frame and a fresh timestamp cannot excuse inconsistent TF.
            let saved_pose = fixture.handle.state.read().await.pose.clone();
            fixture.handle.state.write().await.pose.x += 7.0;
            assert_eq!(client.get_assigned_localization_sample(request.clone()).await.unwrap_err().code(),tonic::Code::FailedPrecondition);
            fixture.handle.state.write().await.pose = saved_pose.clone();
            fixture.handle.state.write().await.pose.timestamp_ns = sample_pose.timestamp_ns - 1;
            assert_eq!(client.get_assigned_localization_sample(request.clone()).await.unwrap_err().code(),tonic::Code::FailedPrecondition);
            fixture.handle.state.write().await.pose = saved_pose;
            let saved_edge = {
                let mut state = fixture.handle.state.write().await;
                let saved = state.transforms.iter().find(|edge|edge.child=="odom").unwrap().clone();
                let mut corrupt = saved.clone();corrupt.tx += 3.0;
                crate::lcm_runtime::apply_sensor_extrinsic_locked(&mut state,&corrupt);
                saved
            };
            assert_eq!(client.get_assigned_localization_sample(request.clone()).await.unwrap_err().code(),tonic::Code::FailedPrecondition);
            {
                let mut state = fixture.handle.state.write().await;
                crate::lcm_runtime::apply_sensor_extrinsic_locked(&mut state,&saved_edge);
            }
            assert!(client.get_assigned_localization_sample(request.clone()).await.is_ok());

            let mut second = request.clone();second.request_id="assigned-live-second".into();
            let second_terminal = assigned_rpc_relocalize(&mut client, fixture, owner, applier, &common, second.clone(), 2).await;
            assert_eq!(second_terminal.assigned_cloud_v2.as_ref().unwrap().correction_revision,2);
            assert_eq!(client.get_assigned_localization_sample(request.clone()).await.unwrap_err().code(),tonic::Code::FailedPrecondition);
            let correction_receipt = applier.snapshot().unwrap();
            assert!(crate::lcm_runtime::handoff_registered_lio_cloud(&fixture.handle,
                &[Point3::new(2.25, 1.25, 0.8)],12,next_stamp+1_000_000_000,next_stamp+1_000_000_000,"map")
                .await.unwrap(), "ordinary mapping resumes after the real registration commit");
            assert!(fixture.handle.state.read().await.map.revision > correction_receipt.occupancy_revision);
            let latest = client.get_assigned_localization_sample(second.clone()).await.unwrap().into_inner();
            assert_eq!(latest.correction_revision,2,"map advancement retains the same correction owner");
            assigned_navigation_readiness(&fixture.handle).await;
            let admitted = client.set_navigation_goal(coordinate_goal("assigned-coordinate-goal",latest.pose.unwrap())).await.unwrap().into_inner();
            assert_eq!(admitted.status,"ACCEPTED","{admitted:?}");
            {
                let published=fixture.backend.published.lock().unwrap();
                let frame=published.iter().find(|frame|frame.msg_type.fqn()=="nuwax_msgs.GoalEnvelope")
                    .expect("coordinate admission publishes the existing typed goal envelope");
                let goal=schema::decode_goal_envelope_body(&frame.payload).unwrap();
                assert_eq!(goal.motion_guard_boot_nonce,[7;16]);
                assert_eq!(goal.motion_stop_generation,1);
                assert_eq!(goal.goal.header.frame_id,"map");
                assert!(!published.iter().any(|frame|frame.msg_type.fqn()=="geometry_msgs.Twist"),
                    "geometry admission cannot bypass the motion guard with a direct velocity");
            }
            // Stop retires the installed activation. Historical settlement is
            // still readable but cannot authorize a new live sample.
            fixture.service.activation_receipts.retire_for_mapping_change().unwrap();
            assert_eq!(client.get_assigned_localization_sample(second).await.unwrap_err().code(),tonic::Code::FailedPrecondition);
            let state = client.get_mapping_state(Empty {}).await.unwrap().into_inner();
            assert!(!state.relocalization_target_available);
        })).await;
        let _ = stop.send(());
        server.await.unwrap().unwrap();
        if let Err(panic)=outcome {std::panic::resume_unwind(panic);}
    }

    async fn assigned_rpc_relocalize(
        client: &mut crate::proto::geometry_runtime_client::GeometryRuntimeClient<tonic::transport::Channel>,
        fixture: &CanonicalResetFixture, owner: &crate::correction_coordinator::CorrectionTransportOwner,
        applier: &Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>,
        common: &crate::lio_ingress::CommonLioIngress, request:RelocalizeRequest, sequence:u64,
    ) -> RelocalizeResult {
        let mut request_client=client.clone();let command=request.clone();
        let reply=tokio::spawn(async move {request_client.relocalize(command).await});
        tokio::time::timeout(Duration::from_secs(3),async {
            while !owner.ingress().relocalization_mode_snapshot().active {tokio::task::yield_now().await;}
        }).await.expect("real RPC admits into actor");
        let pose=fixture.handle.state.read().await.pose.clone();
        let odom_stamp=pose.timestamp_ns;
        let source_stamp=odom_stamp+1;
        let mut odometry=schema::Odometry::default();
        odometry.header.frame_id="odom".into();odometry.child_frame_id="base_link".into();
        odometry.pose.position=if sequence==1 {[4.0,2.0,1.0]} else {[5.0,2.0,1.0]};
        odometry.pose.orientation_xyzw=[0.0,0.0,0.0,1.0];
        let envelope=common.prepare_legacy_odometry(odom_stamp.try_into().unwrap(),odometry).unwrap();
        let identity=crate::lio_ingress::SourceIdentity {backend:envelope.backend,runtime_generation:envelope.runtime_generation,
            source_sequence:(sequence+100).try_into().unwrap(),source_timestamp_ns:source_stamp.try_into().unwrap()};
        let correlated=crate::lio_ingress::CorrelatedOdometry {source_sequence:envelope.source_sequence,
            source_timestamp_ns:envelope.source_timestamp_ns};
        common.accept_public_odometry(envelope,sequence as i64*10).unwrap();
        let target=fixture.handle.cloud_map_writer.lock().as_ref().unwrap().committed_snapshot().unwrap();
        let pattern=target.xyz_f32_le.chunks_exact(12).map(|p|Point3::new(
            f32::from_le_bytes(p[0..4].try_into().unwrap()) as f64+4.0,
            f32::from_le_bytes(p[4..8].try_into().unwrap()) as f64+2.0,
            f32::from_le_bytes(p[8..12].try_into().unwrap()) as f64+1.0)).collect::<Vec<_>>();
        let points=pattern.iter().cycle().take(50_000).copied().collect::<Vec<_>>();
        let prepared=common.register_odom_points(identity,correlated,sequence as i64*10+1,points.len(),points,
            &fixture.handle.mapping_adapter.as_ref().unwrap().lock()).unwrap();
        let accepted=common.accept_public_cloud(prepared.encoded_for_test().as_ref(),sequence as i64*10+2).unwrap();
        crate::lcm_runtime::handoff_registered_lio_cloud_with_correction(&fixture.handle,&accepted,&owner.ingress())
            .await.expect("actual admitted scan reaches real sink and canonical applier");
        // Intentionally discard the RPC response to model an unknown client
        // outcome; recovery below only uses the exact retained terminal query.
        drop(tokio::time::timeout(Duration::from_secs(20),reply).await.expect("real RPC settles").unwrap().unwrap());
        let query=GetRelocalizationTerminalRequest {schema_version:2,request_id:request.request_id.clone(),
            assigned_request_v2:Some(request),..Default::default()};
        let recovered=client.get_relocalization_terminal(query).await.unwrap().into_inner();
        assert_eq!(recovered.status,RelocalizationTerminalStatus::Terminal as i32,"{recovered:?}");
        let terminal=recovered.terminal.unwrap();
        assert_eq!(terminal.assigned_cloud_v2.as_ref().unwrap().disposition,RelocalizeDisposition::Applied as i32,"{terminal:?}");
        assert_eq!(applier.snapshot().unwrap().correction_revision,sequence);
        terminal
    }

    async fn assigned_export_transfer_activation(spec: crate::mapping_backend::MappingBackendSpec, paired: bool) {
        assigned_export_transfer_activation_source(spec, paired, false).await;
    }

    // Keep the four compatibility-array controls unchanged. The native-source
    // case separately exercises actual Common admission and canonical handoff.
    async fn assigned_export_transfer_activation_source(spec: crate::mapping_backend::MappingBackendSpec, paired: bool, native_source: bool) {
        const SOURCE_NS: u64 = 1_700_000_000_123_456_789;
        let source = CanonicalResetFixture::with_backend(spec).await;
        let mut destination = CanonicalResetFixture::with_backend(spec).await;
        let correction = if paired {
            let identity = crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
                runtime_generation: 1, backend_generation: 1,
                correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                    boot_nonce: uuid::Uuid::new_v4().as_u128(), generation: 1,
                },
            };
            let spec = crate::correction_coordinator::CorrectionOwnerSpec::Relocalization {
                map_frame_id:"map".into(), odom_frame_id:"odom".into(), body_frame_id:"base_link".into(),
            };
            let applier = Arc::new(crate::lcm_runtime::GeometryCanonicalCorrectionApplier::new(
                identity.clone(), "map", "odom", "base_link", destination.handle.state.clone(),
                destination.handle.transport.clone(),
                (LcmTopic::new("/test/assigned/canonical-tf").unwrap(), LcmMessageType::new("tf2_msgs.TFMessage").unwrap()),
            ).unwrap());
            applier.bind_relocalization_mapping(&destination.handle).unwrap();
            let sink = Arc::new(crate::correction_coordinator::GeometryCorrectionSink::new_for_spec(
                identity.clone(), &spec, "map", "odom", "base_link", applier.clone(),
            ).unwrap());
            let owner = crate::correction_coordinator::CorrectionTransportOwner::start(spec, identity, "", sink)
                .await.unwrap();
            destination.service.activation_receipts = owner.phase_a_control().activated_target_receipts();
            destination.service.relocalization_ingress = owner.relocalization_ingress();
            destination.service.canonical_correction = Some(applier.clone());
            Some((owner, applier))
        } else { None };
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            assert!(start_mapping_session(&source.service, "assigned-source", "warehouse").await.active);
            commit_checkpoint_frame_test_pose(&source.handle, SOURCE_NS - 1).await;
            let points = [
                Point3::new(0.25, 0.25, 0.5),
                Point3::new(0.26, 0.26, 0.51),
                Point3::new(1.25, 0.5, 1.0),
                Point3::new(0.5, 1.75, 0.75),
                Point3::new(1.5, 1.5, 1.5),
                Point3::new(-0.75, 0.5, 0.6),
            ];
            if native_source {
                let status=source.service.get_status(Request::new(Empty {})).await.unwrap().into_inner();
                assert_eq!(status.supported_assigned_fine_archive_versions,vec![1,2,3,4,5]);
            }
            let common = native_source.then(|| crate::lio_ingress::CommonLioIngress::new(
                crate::lio_ingress::DeviceLioIngressSpec::new(1, "sensor", "base_link", "odom", "map").unwrap()));
            for sequence in 0..3 {
                let timestamp = SOURCE_NS + sequence * 1_000_000_000;
                if let Some(common) = common.as_ref() {
                    let mut odometry = schema::Odometry::default();
                    odometry.header.frame_id = "odom".into();
                    odometry.child_frame_id = "base_link".into();
                    odometry.pose.orientation_xyzw = [0.0, 0.0, 0.0, 1.0];
                    let envelope = common.prepare_legacy_odometry(timestamp as i64, odometry).unwrap();
                    let identity = crate::lio_ingress::SourceIdentity {
                        backend: envelope.backend, runtime_generation: envelope.runtime_generation,
                        source_sequence: sequence as i64 + 1, source_timestamp_ns: timestamp as i64,
                    };
                    let correlated = crate::lio_ingress::CorrelatedOdometry {
                        source_sequence: envelope.source_sequence, source_timestamp_ns: envelope.source_timestamp_ns,
                    };
                    common.accept_public_odometry(envelope, timestamp as i64).unwrap();
                    let prepared = common.register_odom_points(identity, correlated, timestamp as i64,
                        points.len(), points.to_vec(), &source.handle.mapping_adapter.as_ref().unwrap().lock()).unwrap();
                    let admitted = common.accept_public_cloud(prepared.encoded_for_test().as_ref(), timestamp as i64).unwrap();
                    assert!(crate::lcm_runtime::handoff_admitted_registered_cloud(&source.handle, &admitted)
                        .await.expect("canonical admitted source cloud commit"));
                } else {
                    assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
                        &source.handle, &points, 12, timestamp, timestamp, "map",
                    ).await.expect("compatibility-array cloud commit"));
                }
            }
            if native_source {
                let root = source.handle.cloud_map_writer.lock().as_ref().unwrap().committed_fine_cloud().unwrap();
                let mut reader = nuwax_map_chunks::RegionReader::new(root.repository.clone(), 4 * 1024 * 1024);
                assert!(root.manifest().blocks.iter().any(|block| reader.ground_parts(block).unwrap().1.has_heightfield()),
                    "actual source-owned capture must precede the new-format export");
            }
            // This is the actual runtime exporter and backend worker, not a
            // hand-authored checkpoint or an assembly/classification fixture.
            let checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
                &source.service,
                Request::new(assigned_wire_roundtrip(&ExportMappingBackendCheckpointRequest {

                })),
            ).await.expect("native schema-2 export").into_inner();
            let checkpoint = assigned_wire_roundtrip(&checkpoint);
            assert_eq!(checkpoint.schema_version, 2);
            assert!(!checkpoint.payload.is_empty());
            let cloud = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
            assert_eq!(cloud.payload_schema_version, 2);
            let fine_reference = checkpoint.fine_map_artifact.as_ref().expect("actual source fine export");
            let dense_reference = checkpoint.dense_cloud_artifact.as_ref().expect("native dense export");
            let source_dense = source.handle.cloud_map_writer.lock().as_ref().unwrap()
                .committed_dense_cloud().unwrap().unwrap();
            assert!(source_dense.point_count > 0);

            // Switching maps requires an explicit caller-owned stop. The loader
            // itself only admits an idle/stopped owner or an exact sealed replay.
            assert!(start_mapping_session(&destination.service, "assigned-predecessor", "old-map").await.active);
            commit_checkpoint_frame_test_pose(&destination.handle, 90).await;
            seed_canonical_reset_pose(&destination.handle, 91).await;
            let stopped = GeometryRuntime::stop_mapping(&destination.service, Request::new(StopMappingRequest {
                session_id: "assigned-predecessor".into(), reason: "explicit map switch".into(),
            })).await.expect("caller explicitly stops the predecessor").into_inner();
            assert!(stopped.success && !stopped.active);
            assert_eq!(stopped.phase, "STOPPED");

            let request = ActivateMapRequest {
                activation_contract_version: 2,
                activation_request_id: "assigned-export-load".into(),
                session_id: "assigned-loaded".into(),
                map_name: "warehouse".into(),
                expected_runtime_generation: destination.service.activation_receipts.runtime_generation(),
                asset_identity: Some(AssetIdentityV2 { asset_id: "a".repeat(32) }),
                mapping_backend_checkpoint: Some(checkpoint.clone()),
                ..Default::default()
            };
            let untransferred = GeometryRuntime::activate_map(
                &destination.service, Request::new(assigned_wire_roundtrip(&request)),
            ).await.expect_err("missing archive owners reject before lifecycle mutation");
            assert_eq!(untransferred.code(), tonic::Code::InvalidArgument);
            assert!(!destination.handle.mapping_adapter.as_ref().unwrap().lock().is_active());
            assert_eq!(destination.handle.state.read().await.pose.timestamp_ns, 91);

            transfer_assigned_checkpoint_member(
                &source.service, &destination.service, fine_reference, if native_source { b"NUWAXAR5" } else { b"NUWAXAR4" },
            ).await;
            transfer_assigned_checkpoint_member(
                &source.service, &destination.service, dense_reference, b"NUWAXDC1",
            ).await;
            let mut map_events = destination.handle.map_revision_tx.subscribe();
            let mut cloud_events = destination.handle.cloud_map_tx.subscribe();
            let publications_before_activation = recorded_legacy_clouds(&destination.backend).len();
            let activated = GeometryRuntime::activate_map(
                &destination.service, Request::new(assigned_wire_roundtrip(&request)),
            ).await.expect("existing ActivateMap endpoint accepts explicit version 2").into_inner();
            let activated = assigned_wire_roundtrip(&activated);
            assert!(activated.success && activated.active, "assigned activation: {activated:?}");
            assert!(!activated.idempotent_replay);
            assert_eq!(activated.activation_receipt_schema_version, 2);
            assert_eq!(activated.activation_receipt_disposition, ActivationReceiptDisposition::Sealed as i32);
            assert_eq!(activated.relocalization_target_available, paired,
                "availability requires the actual cloud relocalization actor");
            let current = GeometryRuntime::get_mapping_state(&destination.service, Request::new(Empty {}))
                .await.unwrap().into_inner();
            assert_eq!(current.relocalization_target_available, paired);
            let receipt = activated.activation_receipt_v2.as_ref().unwrap();
            assert_eq!(receipt.request.as_ref().unwrap().checkpoint_id, checkpoint.checkpoint_id);
            assert_eq!(receipt.request.as_ref().unwrap().asset_identity, request.asset_identity);
            assert_eq!(receipt.target.as_ref().unwrap().source_cloud_member, cloud.member_identity);
            assert_ne!(receipt.target.as_ref().unwrap().occupancy_receipt.as_ref().unwrap().commit_id,
                checkpoint.occupancy_receipt.as_ref().unwrap().commit_id,
                "the installation allocates its own commit domain");
            assert_canonical_reset_visible(&destination.service).await;
            {
                let adapter = destination.handle.mapping_adapter.as_ref().unwrap().lock();
                assert!(adapter.checkpoint_localization_required);
                assert!(adapter.map_odom_transform_snapshot().is_none(), "predecessor pose must not anchor the loaded map");
            }
            assert_eq!(map_events.try_recv().unwrap().revision, activated.revision);
            assert_eq!(cloud_events.try_recv().unwrap().session_generation, activated.session_generation);
            while let Ok(event) = cloud_events.try_recv() {
                assert_eq!(event.session_generation, activated.session_generation);
            }
            assert_eq!(recorded_legacy_clouds(&destination.backend).len(), publications_before_activation + 1);

            let (installed_cloud_receipt, installed_dense) = {
                let writer = destination.handle.cloud_map_writer.lock();
                let writer = writer.as_ref().unwrap();
                let installed = writer.committed_snapshot().unwrap();
                // 存储槽位不是点身份：安装从体素集合重建排序，增量导出可采用不同槽序。
                // 逐个 12 字节记录精确比较多重集；不近似坐标，也不改变请求原字节。
                assert_eq!(installed.xyz_f32_le.len(), cloud.xyz_f32_le.len());
                assert_eq!(installed.xyz_f32_le.len() % 12, 0);
                let mut installed_points = installed.xyz_f32_le.chunks_exact(12).collect::<Vec<_>>();
                let mut source_points = cloud.xyz_f32_le.chunks_exact(12).collect::<Vec<_>>();
                installed_points.sort_unstable();
                source_points.sort_unstable();
                assert_eq!(installed_points, source_points);
                assert_eq!(installed.point_count, cloud.point_count);
                assert_eq!(installed.voxel_size.to_bits(), cloud.voxel_size.to_bits());
                let dense = writer.committed_dense_cloud().unwrap().unwrap();
                assert_eq!(dense.point_count, source_dense.point_count);
                assert_eq!(dense.read(0, dense.point_count.try_into().unwrap()).unwrap(),
                    source_dense.read(0, source_dense.point_count.try_into().unwrap()).unwrap());
                (writer.last_cloud_receipt().unwrap(), dense)
            };
            // A later real pose belongs to the new session. Replays and conflicts
            // must preserve it, without installing another backend/map/cloud.
            commit_checkpoint_frame_test_pose(&destination.handle, SOURCE_NS + 4_000_000_000).await;
            let installed_session = destination.handle.mapping_adapter.as_ref().unwrap().lock().snapshot();
            let installed_occupancy = destination.handle.mapping_adapter.as_ref().unwrap().lock()
                .latest_occupancy_commit_receipt().unwrap();
            let installed_fine = destination.handle.cloud_map_writer.lock().as_ref().unwrap()
                .committed_fine_cloud().unwrap();
            let current_pose = GeometryRuntime::get_current_pose(&destination.service, Request::new(PoseRequest::default()))
                .await.unwrap().into_inner();
            assert_eq!(current_pose.timestamp_ns, SOURCE_NS + 4_000_000_000);
            assert_ne!(current_pose.position.as_ref().unwrap().x, 8.0, "the predecessor's map pose is never reused");
            // Establishing the first real frame may republish the same cloud
            // with its rehomed fine root. It is not another map installation.
            let published_clouds = recorded_legacy_clouds(&destination.backend).len();
            while let Ok(event) = cloud_events.try_recv() {
                assert_eq!(event.session_generation, activated.session_generation);
                assert_eq!(event.revision, installed_cloud_receipt.cloud_revision);
            }

            // Live immutable owners, rather than temporary transfer leases, own replay.
            for reference in [fine_reference, dense_reference] {
                assigned_artifact_request(&destination.service, reference, 5, 0, Vec::new()).await.unwrap();
            }
            for _ in 0..2 {
                let replay = GeometryRuntime::activate_map(
                    &destination.service, Request::new(assigned_wire_roundtrip(&request)),
                ).await.unwrap().into_inner();
                assert!(replay.success && replay.idempotent_replay);
                assert_eq!(replay.relocalization_target_available, paired);
                assert_eq!(replay.activation_receipt_disposition, ActivationReceiptDisposition::SealedReplay as i32);
                assert_eq!(replay.activation_receipt_v2, activated.activation_receipt_v2);
                assert_eq!((replay.session_generation, replay.backend_generation, replay.revision),
                    (activated.session_generation, activated.backend_generation, activated.revision));
            }
            for mutation in 0..3 {
                let mut changed = request.clone();
                match mutation {
                    0 => changed.asset_identity.as_mut().unwrap().asset_id = "b".repeat(32),
                    1 => changed.session_id = "different-owner".into(),
                    _ => changed.mapping_backend_checkpoint.as_mut().unwrap().checkpoint_id = "c".repeat(32),
                }
                let conflict = GeometryRuntime::activate_map(
                    &destination.service, Request::new(assigned_wire_roundtrip(&changed)),
                ).await.unwrap().into_inner();
                assert!(!conflict.success, "changed owner {mutation} must reject");
                assert_eq!(conflict.activation_receipt_disposition, ActivationReceiptDisposition::Conflict as i32);
            }
            for mutation in 0..4 {
                let mut changed = request.clone();
                let checkpoint = changed.mapping_backend_checkpoint.as_mut().unwrap();
                match mutation {
                    0 => *checkpoint.payload.last_mut().unwrap() ^= 1,
                    1 => {
                        let cell = &mut checkpoint.canonical_occupancy_checkpoint.as_mut().unwrap().data[0];
                        *cell = if *cell == 100 { 0 } else { 100 };
                    }
                    2 => {
                        let xyz = &mut checkpoint.canonical_cloud_checkpoint.as_mut().unwrap().xyz_f32_le;
                        let changed_x = f32::from_le_bytes(xyz[..4].try_into().unwrap()) + 1.0;
                        xyz[..4].copy_from_slice(&changed_x.to_le_bytes());
                    }
                    _ => checkpoint.fine_map_artifact.as_mut().unwrap().id = "d".repeat(32),
                }
                let replay = GeometryRuntime::activate_map(
                    &destination.service, Request::new(assigned_wire_roundtrip(&changed)),
                ).await.expect("同编号重放复用已准入拥有方").into_inner();
                assert!(replay.success && replay.idempotent_replay);
                assert_eq!(replay.activation_receipt_v2,activated.activation_receipt_v2);
            }
            let after_conflicts = GeometryRuntime::activate_map(
                &destination.service, Request::new(assigned_wire_roundtrip(&request)),
            ).await.unwrap().into_inner();
            assert!(after_conflicts.success && after_conflicts.idempotent_replay);
            assert_eq!(after_conflicts.activation_receipt_v2, activated.activation_receipt_v2);
            assert_eq!(destination.handle.mapping_adapter.as_ref().unwrap().lock().snapshot(), installed_session);
            assert_eq!(destination.handle.mapping_adapter.as_ref().unwrap().lock()
                .latest_occupancy_commit_receipt().unwrap(), installed_occupancy);
            {
                let writer = destination.handle.cloud_map_writer.lock();
                let writer = writer.as_ref().unwrap();
                assert_eq!(writer.last_cloud_receipt().unwrap(), installed_cloud_receipt);
                assert!(Arc::ptr_eq(&writer.committed_fine_cloud().unwrap(), &installed_fine));
                assert_eq!(writer.committed_dense_cloud().unwrap().unwrap().epoch, installed_dense.epoch);
            }
            let pose = GeometryRuntime::get_current_pose(&destination.service, Request::new(PoseRequest::default()))
                .await.unwrap().into_inner();
            assert_eq!(pose, current_pose);
            assert!(!destination.handle.state.read().await.latest_transform_entries().is_empty());
            assert_eq!(recorded_legacy_clouds(&destination.backend).len(), published_clouds);
            assert!(matches!(map_events.try_recv(), Err(tokio::sync::broadcast::error::TryRecvError::Empty)));
            assert!(matches!(cloud_events.try_recv(), Err(tokio::sync::broadcast::error::TryRecvError::Empty)));
            if !paired {
                assigned_mapping_cloud_boundary_slice(&destination, &activated, &checkpoint).await;
            }
            if let Some((owner, applier)) = correction.as_ref() {
                assigned_relocalization_navigation_slice(&destination, &activated, owner, applier).await;
            }
        })).await;
        if let Some((owner, _)) = correction { owner.shutdown().await.expect("real correction owner joins"); }
        destination.finish().await;
        source.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    // Native server/backend integration uses injected LCM/readiness. Paired
    // cases additionally exercise the real generated tonic client over loopback;
    // no real LCM context, sensor process or actuator resource is constructed.
    #[tokio::test]
    async fn assigned_v2_native_heightfield_export_transfer_activate_replay_and_conflicts() {
        assigned_export_transfer_activation_source(crate::mapping_backend::MappingBackendSpec::Voxel, false, true).await;
    }

    #[tokio::test]
    async fn assigned_v2_voxel_export_transfer_activate_replay_and_conflicts() {
        assigned_export_transfer_activation(crate::mapping_backend::MappingBackendSpec::Voxel, false).await;
    }

    #[tokio::test]
    async fn assigned_v2_ray_export_transfer_activate_replay_and_conflicts() {
        assigned_export_transfer_activation(crate::mapping_backend::MappingBackendSpec::raytracing_defaults(), false).await;
    }

    #[tokio::test]
    async fn assigned_v2_voxel_real_rpc_relocalizes_samples_and_admits_coordinate_goal() {
        assigned_export_transfer_activation(crate::mapping_backend::MappingBackendSpec::Voxel, true).await;
    }

    #[tokio::test]
    async fn assigned_v2_ray_real_rpc_relocalizes_samples_and_admits_coordinate_goal() {
        assigned_export_transfer_activation(crate::mapping_backend::MappingBackendSpec::raytracing_defaults(), true).await;
    }


    fn recorded_legacy_clouds(backend: &CheckpointPublishBackend) -> Vec<RawLcmFrame> {
        backend.published.lock().unwrap().iter()
            .filter(|frame| frame.msg_type.fqn() == "sensor_msgs.PointCloud2")
            .cloned().collect()
    }


    async fn exercise_canonical_reset_retained_after_caller_cancel(cloud_activation: bool) {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            seed_canonical_reset_pose(&fixture.handle, 61).await;
            let shared = fixture.handle.state.write().await;
            let service = fixture.service.clone();
            let caller = tokio::spawn(async move {
                if cloud_activation {
                    let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
                        .into_iter().flat_map(f32::to_le_bytes).collect::<Vec<_>>();
                    GeometryRuntime::activate_map(&service, Request::new(cloud_activation_request(xyz))).await
                } else {
                    GeometryRuntime::start_mapping(&service, Request::new(StartMappingRequest {
                        session_id: "reset-held".into(), map_name: "room".into(),
                    })).await
                }
            });
            // Holding SharedState blocks the actual post-start reset. Adapter
            // admission proves the call has passed the no-effect/replay paths.
            tokio::time::timeout(Duration::from_secs(5), async {
                loop {
                    if fixture.handle.mapping_adapter.as_ref().unwrap().lock().is_active() { break; }
                    tokio::task::yield_now().await;
                }
            }).await.expect("new session reaches retained reset");
            assert!(!caller.is_finished());
            assert!(fixture.handle.mapping_lifecycle_lock.try_lock().is_err());
            assert_eq!(shared.pose.timestamp_ns, 61); // allowed unfinished-transition observation
            caller.abort();
            assert!(caller.await.unwrap_err().is_cancelled());
            assert!(fixture.handle.mapping_lifecycle_lock.try_lock().is_err());
            drop(shared);
            let transition = tokio::time::timeout(Duration::from_secs(5),
                fixture.service.mapping_transition.clone().lock_owned()).await
                .expect("retained transition completes after reset");
            drop(transition);
            assert_canonical_reset_visible(&fixture.service).await;
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    #[tokio::test]
    async fn canonical_reset_start_retains_lifecycle_until_reset_after_caller_cancel() {
        exercise_canonical_reset_retained_after_caller_cancel(false).await;
    }

    #[tokio::test]
    async fn canonical_reset_activation_retains_lifecycle_until_reset_after_caller_cancel() {
        exercise_canonical_reset_retained_after_caller_cancel(true).await;
    }

    fn activation_request(session_id: &str, map_name: &str) -> ActivateMapRequest {
        let mut request = cloud_activation_request([1.0_f32,2.0,3.0,4.0,5.0,6.0].into_iter().flat_map(f32::to_le_bytes).collect());
        request.session_id=session_id.into(); request.map_name=map_name.into(); request.activation_request_id=session_id.into();
        request
    }

    fn cloud_activation_request(xyz_f32_le: Vec<u8>) -> ActivateMapRequest {
        let mut checkpoint = super::assigned_checkpoint_assembly_tests::frozen();
        checkpoint.checkpoint_id=uuid::Uuid::new_v4().simple().to_string();
        checkpoint.payload = crate::mapping_backend::export_mapping_kernel_checkpoint(
            &crate::mapping_backend::MappingBackendSpec::Voxel.into_kernel(nuwax_mapping::DEFAULT_BOUNDED_CAPACITY)).unwrap();
        checkpoint.payload_size = checkpoint.payload.len() as u64;
        let occupancy = checkpoint.canonical_occupancy_checkpoint.as_mut().unwrap();
        occupancy.meta = Some(OccupancyGridMeta { width:2,height:2,resolution:0.05,origin_x:-1.0,origin_y:2.0,
            frame_id:"map".into(),revision:119,timestamp_ns:1_700_000_000_000_000_000 });
        occupancy.data = vec![u8::MAX,0,100,50];
        let source_time = occupancy.meta.as_ref().unwrap().timestamp_ns;
        let occupancy_receipt = checkpoint.occupancy_receipt.as_mut().unwrap();
        occupancy_receipt.occupancy_revision=119; occupancy_receipt.source_timestamp_ns=source_time;
        let cloud = checkpoint.canonical_cloud_checkpoint.as_mut().unwrap();
        cloud.source_revision=119; cloud.point_count=(xyz_f32_le.len()/12) as u32; cloud.xyz_f32_le=xyz_f32_le;
        let cloud_receipt=checkpoint.cloud_receipt.as_mut().unwrap();
        cloud_receipt.occupancy_revision=119; cloud_receipt.cloud_revision=119;
        cloud_receipt.source_timestamp_ns=source_time; cloud_receipt.point_count=cloud.point_count;
        activation_request_with_backend_checkpoint("mapload:one","warehouse",assemble_mapping_checkpoint_export(checkpoint).unwrap())
    }

    fn modern_activation_request(mut request: ActivateMapRequest, activation_request_id: &str, runtime_generation: u64) -> ActivateMapRequest {
        request.activation_request_id = activation_request_id.into();
        request.expected_runtime_generation = runtime_generation;
        request
    }

    fn ray_mapping_adapter() -> crate::mapping_adapter::MappingAdapter {
        crate::mapping_adapter::MappingAdapter::new_with_backend_for_tests(
            crate::mapping_adapter::MappingAdapterConfig {
                resolution: 0.1,
                initial_safe_radius_meters: 0.0,
            },
            crate::mapping_backend::MappingBackendSpec::raytracing_defaults(),
            "map",
            "odom",
        )
    }

    async fn ray_runtime_with_cloud_writer(
        label: &str,
    ) -> (
        GeometryRuntimeHandle,
        GeometryRuntimeService,
        std::path::PathBuf,
    ) {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("ray real-lcm test handle");
        *handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock() = ray_mapping_adapter();
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-ray-checkpoint-{label}-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new_for_test(&cloud_path).expect("ray checkpoint cloud-map writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        (handle, service, cloud_path)
    }

    async fn export_voxel_backend_checkpoint() -> (MappingBackendCheckpoint, Arc<parking_lot::Mutex<crate::planar_artifacts::Store>>) {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-backend-checkpoint-source-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new_for_test(&cloud_path).expect("source cloud-map writer"),
        )));
        let service = GeometryRuntimeService::new(handle.clone());
        let started =
            start_mapping_session(&service, "checkpoint-source", "checkpoint-source-map").await;
        assert!(started.active);
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 0.0, 2.0)],
            12,
            1,
            1_700_000_000_123_456_789,
            "map",
        )
        .await
        .expect("canonical source cloud handoff"));
        let checkpoint =
            GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(ExportMappingBackendCheckpointRequest::default()))
                .await
                .expect("ExportMappingBackendCheckpoint")
                .into_inner();
        let store=service.graph_artifacts.clone();
        drop(service);
        drop(handle);
        let _ = std::fs::remove_file(cloud_path);
        (checkpoint,store)
    }

    fn canonical_backend_checkpoint_fixture() -> MappingBackendCheckpoint {
        let payload = crate::mapping_backend::export_mapping_kernel_checkpoint(&crate::mapping_backend::MappingBackendSpec::Voxel.into_kernel(nuwax_mapping::DEFAULT_BOUNDED_CAPACITY)).unwrap();
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let canonical_occupancy_checkpoint = OccupancyGridData {
            meta: Some(OccupancyGridMeta {
                width: 1,
                height: 1,
                resolution: 0.05,
                origin_x: 0.0,
                origin_y: 0.0,
                frame_id: "map".to_owned(),
                revision: 20,
                timestamp_ns: 100,
            }),
            data: vec![0],
        };
        let canonical_cloud_checkpoint = CloudMapCheckpoint {
            frame_id: "map".to_owned(),
            source_revision: 4,
            point_count: 2,
            voxel_size: crate::pointcloud::VOXEL_SIZE,

            xyz_f32_le: cloud_xyz,
            payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            member_identity: None,
        };
        let checkpoint=MappingBackendCheckpoint {
            fine_map_artifact: None,
            dense_cloud_artifact: None,
            schema_version: crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
            backend_kind: "voxel".to_owned(),
            source_session_generation: 7,

            payload_size: u64::try_from(payload.len()).unwrap(),
            created_at_ns: 1,
            payload,
            source_session_id: "source-session".to_owned(),
            occupancy_receipt: Some(CanonicalOccupancyCommitReceipt {
                commit_id: Some(MappingCommitIdentity {
                    boot_nonce: vec![1; 16],
                    sequence: 9,
                }),
                session_id: "source-session".to_owned(),
                session_generation: 7,
                occupancy_revision: 20,
                source_timestamp_ns: 100,
                backend_kind: "voxel".to_owned(),
                backend_generation: 3,
            }),
            cloud_receipt: Some(CanonicalCloudCommitReceipt {
                commit_id: Some(MappingCommitIdentity {
                    boot_nonce: vec![1; 16],
                    sequence: 7,
                }),
                session_generation: 7,
                occupancy_revision: 18,
                cloud_revision: 4,
                source_timestamp_ns: 500,
                point_count: 2,
            }),
            canonical_occupancy_checkpoint: Some(canonical_occupancy_checkpoint),
            canonical_cloud_checkpoint: Some(canonical_cloud_checkpoint),
            graph_artifact: None,
            target_kind: String::new(),
            checkpoint_id: String::new(),
            backend_payload_schema_version: 0,
            backend_identity: None, occupancy_identity: None,
        };
        let mut checkpoint=checkpoint;checkpoint.checkpoint_id="1".repeat(32);
        assemble_mapping_checkpoint_export(checkpoint).unwrap()
    }

    fn activation_request_with_backend_checkpoint(session_id: &str, map_name: &str, mut checkpoint: MappingBackendCheckpoint) -> ActivateMapRequest {
        if checkpoint.schema_version != 2 {
            checkpoint.checkpoint_id=uuid::Uuid::new_v4().simple().to_string();
            checkpoint=assemble_mapping_checkpoint_export(checkpoint).unwrap();
        }
        ActivateMapRequest {session_id:session_id.into(),map_name:map_name.into(),mapping_backend_checkpoint:Some(checkpoint),
            activation_request_id:session_id.into(),expected_runtime_generation:1,activation_contract_version:2,
            asset_identity:Some(AssetIdentityV2 {asset_id:"2".repeat(32)})}
    }

    fn exploration_request(request_id: &str) -> ExploreRequest {
        ExploreRequest {
            request_id: request_id.to_string(),
            strategy: "wavefront".to_string(),
            arbitration: "prefer_wavefront".to_string(),

            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
            ..ExploreRequest::default()
        }
    }

    async fn prime_navigation_runtime(handle: &GeometryRuntimeHandle) {
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        install_server_test_committed_map(handle, 1);
    }

    async fn start_public_navigation(
        service: &GeometryRuntimeService,
        handle: &GeometryRuntimeHandle,
        goal_id: &str,
    ) -> NavigationState {
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        install_server_test_committed_map(handle, 1);
        GeometryRuntime::set_navigation_goal(
            service,
            Request::new(NavigationGoal {
                pose: Some(Pose {
                    orientation: Some(Quat {
                        w: 1.0,
                        ..Quat::default()
                    }),
                    frame_id: "map".to_string(),
                    timestamp_ns: now_ns(),
                    ..Pose::default()
                }),
                frame_id: "map".to_string(),
                goal_id: goal_id.to_string(),

                motion_guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 1,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect("public navigation goal")
        .into_inner()
    }

    async fn start_active_exploration(
        handle: &GeometryRuntimeHandle,
        sink: Arc<LifecycleExploreSink>,
        request_id: &str,
    ) -> (String, ChildNavigationLease) {
        handle.explore.set_navigation_sink(sink.clone());
        ingest_server_test_committed_costmap(handle, 1).await;
        assert_eq!(
            handle
                .explore
                .start(request_id, "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let active = handle.explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        let lease = sink.last_lease();
        assert_eq!(active.child_task_id, lease.session_id);
        (active.child_task_id, lease)
    }


    #[tokio::test]
    async fn process_stop_waits_for_exploration_zero_before_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let accepted = start_public_navigation(&service, &handle, "process-stop-zero-public").await;
        assert_eq!(accepted.status, "ACCEPTED");
        let public_lease = handle
            .nav
            .active_goal_lease()
            .await
            .expect("active public navigation lease");
        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_task_id, lease) =
            start_active_exploration(&handle, sink, "process-stop-zero").await;
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_millis(100), async {
            while handle.explore.snapshot().status != STATUS_STOPPING {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("process Stop must dispatch exploration cancellation");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must remain gated on ZEROED proof"
        );

        handle
            .explore
            .report_navigation_result(
                &child_task_id,
                ChildNavigationResult::Cancelled(ChildNavigationZeroProof {
                    lease,
                    cancel_generation: 1,
                    zero_ack_id: "process-stop-zero-1".to_string(),
                    zero_applied_monotonic_ns: now_monotonic_nanos(),
                }),
            )
            .await
            .expect("process Stop ZEROED proof");

        tokio::time::timeout(Duration::from_millis(100), async {
            while handle.nav.cancel_snapshot().phase != crate::nav_runtime::CancelPhase::Pending {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("process Stop must wait for public navigation ZERO");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "retained receipt and listener must remain gated on public Nav ZERO"
        );
        let public_cancel = handle.nav.cancel_snapshot();
        assert_eq!(public_cancel.lease.as_ref(), Some(&public_lease));
        handle
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: public_cancel.generation,
                goal_id: public_lease.session_id.clone(),
                owner_kind: public_lease.owner_kind,
                parent_request_id: public_lease.parent_request_id.clone(),
                goal_generation: public_lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "process Stop public navigation ZEROED".to_string(),
            })
            .await;

        let response = stop_task
            .await
            .expect("process Stop task")
            .expect("process Stop response")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");

        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "stopped");
    }

    #[tokio::test]
    async fn process_stop_failure_keeps_scheduler_and_listener_alive() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink.clone(), "process-stop-failed").await;
        *sink
            .cancel_error
            .lock()
            .expect("lifecycle exploration sink lock") = Some("guard unavailable".to_string());
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());


        let error = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect_err("retained ownership must reject process Stop");

        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("safety ownership retained"));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");

        tokio::time::timeout(
            Duration::from_millis(40),
            shutdown.wait_for_readiness_revocation(),
        )
        .await
        .expect("failed process Stop must revoke readiness");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "failed process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_bounds_a_blocked_exploration_cancel_and_retains_ownership() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        let (_, child_lease) = start_active_exploration(&handle, sink.clone(), "process-stop-cancel-timeout").await;
        sink.cancel_never_returns
            .store(true, std::sync::atomic::Ordering::Release);
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);

        let error = tokio::time::timeout(
            Duration::from_secs(1),
            GeometryRuntime::stop(&service, Request::new(StopRequest {})),
        )
        .await
        .expect("process Stop exploration deadline")
        .expect_err("blocked exploration cancel must fail process Stop");

        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("safety ownership retained"));
        let retained = handle.explore.snapshot();
        assert_eq!(retained.status, STATUS_FAILED);
        assert!(retained.message.contains("dispatch timed out"));
        assert!(retained.child_task_id.is_empty());
        assert!(handle.explore.orphan_cancellation_registry().contains(&child_lease));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");
        assert!(handle.mapping_lifecycle_lock.try_lock().is_ok());
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "failed process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_recovers_cancel_receipt_after_response_loss() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_task_id, lease) =
            start_active_exploration(&handle, sink.clone(), "process-stop-response-loss").await;
        sink.cancel_receipt_before_block
            .store(true, std::sync::atomic::Ordering::Release);
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_millis(100), sink.cancel_entered.notified())
            .await
            .expect("navigation cancel must record its receipt before losing the response");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must remain gated while the receipt response is lost"
        );
        let buffered = handle
            .explore
            .report_navigation_result(
                &child_task_id,
                ChildNavigationResult::Cancelled(ChildNavigationZeroProof {
                    lease,
                    cancel_generation: 1,
                    zero_ack_id: "process-stop-response-loss-zero".to_string(),
                    zero_applied_monotonic_ns: now_monotonic_nanos(),
                }),
            )
            .await
            .expect("early exact ZEROED must be buffered until receipt recovery");
        assert_eq!(buffered.status, STATUS_STOPPING);
        assert!(buffered.message.contains("ZEROED_EARLY_BUFFERED"));

        let response = tokio::time::timeout(Duration::from_secs(1), stop_task)
            .await
            .expect("process Stop response-loss recovery deadline")
            .expect("process Stop task")
            .expect("process Stop must recover the exact receipt")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "stopped");
    }

    #[tokio::test]
    async fn process_stop_recovers_submit_response_loss_through_nav_owner() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        prime_navigation_runtime(&handle).await;
        let mut facade = handle.clone();
        facade.explore_scheduler = Arc::new(tokio::sync::Mutex::new(None));
        let sink = Arc::new(SubmitLeaseThenBlockSink {
            facade,
            entered: tokio::sync::Notify::new(),
        });
        handle.explore.set_navigation_sink(sink.clone());
        ingest_server_test_committed_costmap(&handle, 1).await;
        assert_eq!(
            handle
                .explore
                .start("submit-response-loss", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let scheduler = handle.explore.spawn_scheduler(
            Arc::new(LifecycleStaticPoseProvider),
            Duration::from_millis(5),
        );
        *handle.explore_scheduler.lock().await = Some(scheduler);
        tokio::time::timeout(Duration::from_secs(1), sink.entered.notified())
            .await
            .expect("child submit must establish a nav lease before losing its response");
        let orphaned_lease = handle
            .nav
            .active_goal_lease()
            .await
            .expect("response-loss navigation lease");
        assert_eq!(
            orphaned_lease.owner_kind,
            nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        );

        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if handle.nav.cancel_snapshot().phase == crate::nav_runtime::CancelPhase::Pending {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("Process Stop must reclaim the orphaned NavRuntime lease");
        let pending = handle.nav.cancel_snapshot();
        assert!(pending.generation > 0);
        assert_eq!(pending.lease.as_ref(), Some(&orphaned_lease));
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must wait for orphaned child ZEROED proof"
        );
        handle
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: orphaned_lease.session_id.clone(),
                owner_kind: orphaned_lease.owner_kind,
                parent_request_id: orphaned_lease.parent_request_id.clone(),
                goal_generation: orphaned_lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "submit response-loss process stop zeroed".to_string(),
            })
            .await;

        let response = tokio::time::timeout(Duration::from_secs(2), stop_task)
            .await
            .expect("submit response-loss Process Stop deadline")
            .expect("Process Stop task")
            .expect("Process Stop response")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");
        assert!(handle.nav.active_goal_lease().await.is_none());
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "stopped");
    }


    #[tokio::test]
    async fn process_stop_timeout_does_not_request_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink, "process-stop-timeout").await;
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);

        let error = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect_err("missing ZEROED proof must time out process Stop");

        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(error.message().contains("safety ownership release"));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "timed-out process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_waits_for_public_navigation_zero_before_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let accepted = start_public_navigation(&service, &handle, "public-stop-zero").await;
        assert_eq!(accepted.status, "ACCEPTED");
        let active_lease = handle
            .nav
            .active_goal_lease()
            .await
            .expect("active public lease");
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.nav.cancel_snapshot().phase != crate::nav_runtime::CancelPhase::Pending {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("process Stop must dispatch public navigation cancellation");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must wait for public navigation ZEROED proof"
        );
        let pending = handle.nav.cancel_snapshot();
        assert_eq!(pending.lease.as_ref(), Some(&active_lease));
        handle
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: active_lease.session_id.clone(),
                owner_kind: active_lease.owner_kind,
                parent_request_id: active_lease.parent_request_id.clone(),
                goal_generation: active_lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "process stop public zero".to_string(),
            })
            .await;

        let stopped = stop_task
            .await
            .expect("process Stop join")
            .expect("process Stop response")
            .into_inner();
        assert!(stopped.success);
        listener_task.await.expect("listener shutdown task");
        assert!(handle.nav.active_goal_lease().await.is_none());
        assert_eq!(service.state.read().await.phase, "stopped");
    }

    #[tokio::test]
    async fn process_stop_bounds_a_blocked_navigation_cancel_and_retains_ownership() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let backend = Arc::new(BlockingCancelPublishBackend::default());
        handle.transport = LcmTransport::new(backend.clone());
        prime_navigation_runtime(&handle).await;
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let accepted =
            start_public_navigation(&service, &handle, "public-stop-cancel-timeout").await;
        assert_eq!(accepted.status, "ACCEPTED", "{}", accepted.message);
        let active_lease = handle
            .nav
            .active_goal_lease()
            .await
            .expect("active public lease");


        let error = tokio::time::timeout(
            Duration::from_secs(1),
            GeometryRuntime::stop(&service, Request::new(StopRequest {})),
        )
        .await
        .expect("process Stop navigation dispatch deadline")
        .expect_err("blocked navigation cancel must fail process Stop");

        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(error.message().contains("navigation safety stop"));
        assert_eq!(backend.published_cancel_count(), 1);
        assert_eq!(
            handle.nav.active_goal_lease().await.as_ref(),
            Some(&active_lease)
        );
        let retained = handle.nav.cancel_snapshot();
        assert_eq!(retained.phase, crate::nav_runtime::CancelPhase::Pending);
        assert_eq!(retained.lease.as_ref(), Some(&active_lease));
        assert_eq!(retained.generation, 1);
        let state = service.state.read().await;
        assert_eq!(state.phase, "failed");
        assert_eq!(state.message, error.message());
        drop(state);

        tokio::time::timeout(
            Duration::from_secs(1),
            readiness.wait_for_readiness_revocation(),
        )
        .await
        .expect("failed Stop must revoke readiness");
        assert!(handle.mapping_lifecycle_lock.try_lock().is_ok());
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "failed process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_closes_exploration_and_public_navigation_admission_atomically() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        start_mapping_session(&service, "stop-admission", "warehouse").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        let admission_guard = service.state.write().await;
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        tokio::time::timeout(
            Duration::from_millis(100),
            readiness.wait_for_readiness_revocation(),
        )
        .await
        .expect("Stop must close admission before queuing for service state");
        let explore_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_exploration(
                    &service,
                    Request::new(exploration_request("queued-after-stop")),
                )
                .await
            })
        };
        let navigation_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::set_navigation_goal(
                    &service,
                    Request::new(NavigationGoal {
                        goal_id: "queued-public-after-stop".to_string(),

                        motion_guard_boot_nonce: vec![7; 16],
                        motion_stop_generation: 1,
                        ..NavigationGoal::default()
                    }),
                )
                .await
            })
        };
        let listener_release = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_release = Arc::clone(&listener_release);
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_release.notified().await;
                listener.mark_listener_closed();
            })
        };
        drop(admission_guard);

        let explore_error = explore_task
            .await
            .unwrap()
            .expect_err("explore admission closed");
        let navigation_error = navigation_task
            .await
            .unwrap()
            .expect_err("public navigation admission closed");
        assert_eq!(explore_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(navigation_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(
            explore_error.message(),
            "StartExploration requires runtime phase=running, current=stopping"
        );
        assert_eq!(
            navigation_error.message(),
            "SetNavigationGoal requires runtime phase=running, current=stopping"
        );
        listener_release.notify_one();
        assert!(stop_task.await.unwrap().unwrap().into_inner().success);
        listener_task.await.unwrap();
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert!(handle.nav.active_goal_lease().await.is_none());
    }

    #[tokio::test]
    async fn start_exploration_requires_active_mapping() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));

        let inactive = GeometryRuntime::start_exploration(
            &service,
            Request::new(exploration_request("inactive-map")),
        )
        .await
        .expect_err("inactive mapping must reject exploration");
        assert_eq!(inactive.code(), tonic::Code::FailedPrecondition);

        let activated = GeometryRuntime::activate_map(
            &service,
            Request::new(activation_request("exact-map", "warehouse")),
        )
        .await
        .expect("activate exact test map")
        .into_inner();
        assert!(activated.revision > 0);
        let accepted = GeometryRuntime::start_exploration(
            &service,
            Request::new(exploration_request("active-map")),
        )
        .await
        .expect("active mapping must admit exploration")
        .into_inner();
        assert_eq!(accepted.status, STATUS_ACCEPTED);
        assert_eq!(accepted.request_id, "active-map");
    }

    #[tokio::test]
    async fn exploration_start_and_mapping_mutations_share_one_transition_barrier() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "barrier-map", "warehouse").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let start_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_exploration(
                    &service,
                    Request::new(exploration_request("barrier-explore")),
                )
                .await
            })
        };
        tokio::time::sleep(Duration::from_millis(20)).await;
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop_mapping(
                    &service,
                    Request::new(StopMappingRequest {
                        session_id: "barrier-map".to_string(),
                        reason: "must not overtake start".to_string(),
                    }),
                )
                .await
            })
        };
        let activate_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::activate_map(
                    &service,
                    Request::new(activation_request(
                        "barrier-activate",
                        "warehouse-after-barrier",
                    )),
                )
                .await
            })
        };
        drop(lifecycle_guard);

        let started = start_task
            .await
            .unwrap()
            .expect("queued exploration start")
            .into_inner();
        assert_eq!(started.status, STATUS_ACCEPTED);
        let stop_error = stop_task
            .await
            .unwrap()
            .expect_err("StopMapping must be fenced");
        let activate_error = activate_task
            .await
            .unwrap()
            .expect_err("ActivateMap must be fenced");
        assert_eq!(stop_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(activate_error.code(), tonic::Code::FailedPrecondition);
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        assert!(mapping.active);
        assert_eq!(mapping.map_name, "warehouse");
    }

    #[tokio::test]
    async fn mapping_mutation_allows_failed_exploration_without_safety_ownership() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "failed-session", "warehouse-failed").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        assert_eq!(
            handle
                .explore
                .start("failed-without-child", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );

        let failed = handle
            .explore
            .tick_with_pose(ExploreRobotPose {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                orientation: nuwax_nav_types_messages::geometry::Quaternion::IDENTITY,
                frame_id: "odom".to_string(),
                source_timestamp_ns: now_unix_nanos(),
                received_monotonic_ns: now_monotonic_nanos(),
            })
            .await;
        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.child_task_id.is_empty());
        assert!(!handle.explore.blocks_mapping_mutation().await);

        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "failed-session".to_string(),
                reason: "terminal exploration failure".to_string(),
            }),
        )
        .await
        .expect("child-free FAILED must allow StopMapping")
        .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert_eq!(handle.explore.snapshot().status, STATUS_FAILED);
        assert_eq!(
            handle.explore.snapshot().last_stop_reason,
            failed.last_stop_reason
        );

        let activated = GeometryRuntime::activate_map(
            &service,
            Request::new(activation_request("mapload:failed", "warehouse-recovered")),
        )
        .await
        .expect("child-free FAILED must allow ActivateMap")
        .into_inner();
        assert_eq!(activated.phase, "ACTIVE");
        assert_eq!(handle.explore.snapshot().status, STATUS_FAILED);
        assert_eq!(
            handle.explore.snapshot().last_stop_reason,
            failed.last_stop_reason
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn ordinary_real_guard_receipt_reaches_original_cancel_and_exploration_handlers() {
        use crate::lcm_runtime::vertical_v2_canary::{GuardChild, ORDINARY_STOP_TEST_CAPABILITIES};
        let guard = GuardChild::spawn_with_capabilities(&format!("b5-handlers-{}", uuid::Uuid::new_v4()),
            Some(ORDINARY_STOP_TEST_CAPABILITIES)).await;
        let mut client = nuwax_motion_safety_guard::MotionGuardClient::connect(&guard.socket_path).await.unwrap();
        let handle = crate::lcm_runtime::build_handle_for_tests().await.unwrap();
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "ordinary-handlers-map", "ordinary-handlers-map").await;
        let request_for = |state: &NavigationState, proof: Option<OrdinaryGlobalStopConfirmation>| CancelNavigationRequest {
            goal_id: state.goal_id.clone(), owner_kind: state.owner_kind,
            parent_request_id: state.parent_request_id.clone(), goal_generation: state.goal_generation,
            producer_runtime_generation: state.producer_runtime_generation, global_stop_confirmation: proof,
        };
        let receipt_for = |id: &str, proof: &nuwax_motion_safety_guard::GlobalStopConfirmation| OrdinaryGlobalStopConfirmation {
            operation_id: id.into(), zero_ack_id: proof.zero.zero_ack_id.clone(),
            applied_monotonic_ns: proof.zero.applied_monotonic_ns,
            stationary_confirmed: proof.zero.stationary_confirmed,
        };
        // 隔离LCM没有NAV回执消费者；真实守卫租约由此测试客户端持有。
        let old = start_public_navigation(&service, &handle, "ordinary-unanswered-nav").await;
        assert_eq!(old.status, "ACCEPTED");
        let ready = client.get_readiness().await.unwrap();
        client.acquire_lease("ordinary-unanswered-acquire", &nuwax_motion_safety_guard::MotionAdmissionProof {
            boot_nonce: ready.boot_nonce.try_into().unwrap(), motion_stop_generation: ready.motion_stop_generation,
        }).await.unwrap();
        client.submit_twist("ordinary-unanswered-motion", nuwax_motion_safety_guard::wire::TwistCommand {
            linear_x_mps: 0.1, linear_y_mps: 0.0, angular_z_rps: 0.0,
        }).await.unwrap();
        let pending = GeometryRuntime::cancel_navigation(&service, Request::new(request_for(&old, None))).await.unwrap().into_inner();
        assert_eq!(pending.cancel_phase, "PENDING");
        handle.nav.expire_cancel_for_test();
        assert!(handle.nav.poll_cancel_timeout().await.is_some());
        let confirmation = client.stop_all_motion("ordinary-handlers-stop", "本机真实回执串接").await.unwrap();
        let proof = receipt_for("ordinary-handlers-stop", &confirmation);
        let events_after_zero = guard.actuator_events();
        let stopped = GeometryRuntime::cancel_navigation(&service, Request::new(request_for(&old, Some(proof.clone())))).await.unwrap().into_inner();
        assert_eq!(stopped.status, "FAILED");
        assert_eq!(stopped.reason_code, "GLOBAL_STOP_CONFIRMED");
        assert_eq!(stopped.zero_ack_id, confirmation.zero.zero_ack_id);
        assert_eq!(stopped.zero_applied_monotonic_ns, confirmation.zero.applied_monotonic_ns);
        assert_eq!(guard.actuator_events(), events_after_zero, "几何收尾不发第二次停车");
        let successor = start_public_navigation(&service, &handle, "ordinary-handler-successor").await;
        assert_eq!(successor.status, "ACCEPTED");
        GeometryRuntime::cancel_navigation(&service, Request::new(request_for(&old, Some(proof.clone())))).await.unwrap();
        assert_eq!(handle.nav.active_goal_lease().await.unwrap().session_id, successor.goal_id);
        assert!(GeometryRuntime::cancel_navigation(&service, Request::new(request_for(&successor, Some(proof)))).await.is_err());
        // 下一独立操作清理本测试新目标，仍只沿同次回执处理一次。
        let cleanup = client.stop_all_motion("ordinary-handlers-next", "清理后继测试目标").await.unwrap();
        GeometryRuntime::cancel_navigation(&service, Request::new(request_for(&successor,
            Some(receipt_for("ordinary-handlers-next", &cleanup))))).await.unwrap();

        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_id, child) = start_active_exploration(&handle, sink.clone(), "ordinary-unanswered-exploration").await;
        let exploration = handle.explore.snapshot();
        // 明确无响应的子入口保留与生产相同的精确NAV身份；没有声称启动或杀死NAV进程。
        handle.nav.replace_active_goal_for_test(crate::nav_runtime::ActiveGoal {
            admitted_guard_clock_ns: nuwax_motion_safety_guard::measured_motion::monotonic_ns(),
            id: child_id.clone(), frame_id: "map".into(), target: PoseState::default(),
            options: Default::default(), producer: "wavefront".into(), producer_epoch: exploration.exploration_generation as i64,
            body_to_support_z_m: None, boundary_owner: None,
            lease: crate::nav_runtime::GoalLease {
                session_id: child_id, owner_kind: child.owner_kind, parent_request_id: child.parent_request_id.clone(),
                goal_generation: child.goal_generation, producer_runtime_generation: child.producer_runtime_generation,
                path_revision: 0, planner_runtime_generation: 0,
            },
        }).await;
        *sink.cancel_error.lock().unwrap() = Some("明确无响应子导航".into());
        assert!(handle.explore.stop(&exploration.request_id).await.parking_pending);
        let confirmation = client.stop_all_motion("ordinary-handlers-explore", "本机探索普通停车").await.unwrap();
        let proof = receipt_for("ordinary-handlers-explore", &confirmation);
        let events_after_zero = guard.actuator_events();
        GeometryRuntime::cancel_navigation(&service, Request::new(CancelNavigationRequest {
            goal_id: child.session_id, owner_kind: child.owner_kind, parent_request_id: child.parent_request_id,
            goal_generation: child.goal_generation, producer_runtime_generation: child.producer_runtime_generation,
            global_stop_confirmation: Some(proof.clone()),
        })).await.unwrap();
        let stopped = GeometryRuntime::stop_exploration(&service, Request::new(StopExplorationRequest {
            request_id: exploration.request_id, expected_exploration_generation: exploration.exploration_generation,
            global_stop_confirmation: Some(proof),
        })).await.unwrap().into_inner();
        assert_eq!(stopped.status, "FAILED");
        assert!(!stopped.parking_pending);
        assert_eq!(guard.actuator_events(), events_after_zero, "两个拥有方消费同一回执不重复归零");
        let next = GeometryRuntime::start_exploration(&service, Request::new(ExploreRequest {
            request_id: "ordinary-explore-successor".into(), motion_guard_boot_nonce: vec![7;16],
            ..ExploreRequest::default()
        })).await.unwrap().into_inner();
        assert_eq!(next.status, "ACCEPTED", "{}", next.message);
        handle.explore.stop(&next.request_id).await;
        drop(client);
        guard.shutdown().await;
    }

    #[tokio::test]
    async fn mapping_mutation_waits_for_late_zeroed_after_child_cancel_failure() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "parked-session", "warehouse-parked").await;
        let sink = Arc::new(LifecycleExploreSink::default());
        handle.explore.set_navigation_sink(sink.clone());
        ingest_server_test_committed_costmap(&handle, 1).await;
        assert_eq!(
            handle
                .explore
                .start("parked-exploration", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let active = handle.explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        let lease = sink.last_lease();
        assert_eq!(active.child_task_id, lease.session_id);
        assert_eq!(
            handle.explore.stop("parked-exploration").await.status,
            STATUS_STOPPING
        );

        let failed = handle
            .explore
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: ChildNavigationTerminalProof {
                        lease: lease.clone(),
                    },
                    cancel_generation: 1,
                    reason: "CANCEL_ZERO_ACK_TIMEOUT".to_string(),
                },
            )
            .await
            .expect("cancel failure terminal");
        assert_eq!(failed.status, STATUS_FAILED);
        assert_eq!(failed.last_stop_reason, "child_cancel_failed");
        assert!(failed.child_task_id.is_empty());
        assert!(failed.parking_pending);
        assert!(handle.explore.blocks_mapping_mutation().await);

        let stop_error = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "parked-session".to_string(),
                reason: "must remain parked".to_string(),
            }),
        )
        .await
        .expect_err("retained child ownership must block StopMapping");
        assert_eq!(stop_error.code(), tonic::Code::FailedPrecondition);
        let activate_error = GeometryRuntime::activate_map(
            &service,
            Request::new(activation_request("mapload:blocked", "warehouse-next")),
        )
        .await
        .expect_err("retained child ownership must block ActivateMap");
        assert_eq!(activate_error.code(), tonic::Code::FailedPrecondition);
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());

        let released = handle
            .explore
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(ChildNavigationZeroProof {
                    lease,
                    cancel_generation: 1,
                    zero_ack_id: "late-zero-1".to_string(),
                    zero_applied_monotonic_ns: now_monotonic_nanos(),
                }),
            )
            .await
            .expect("late ZEROED proof");
        assert_eq!(released.status, STATUS_FAILED);
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        assert!(released.child_task_id.is_empty());
        assert!(!handle.explore.blocks_mapping_mutation().await);

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "parked-session".to_string(),
                reason: "late ZEROED released ownership".to_string(),
            }),
        )
        .await
        .expect("late ZEROED must release StopMapping");
        let activated = GeometryRuntime::activate_map(
            &service,
            Request::new(activation_request("mapload:released", "warehouse-next")),
        )
        .await
        .expect("late ZEROED must release ActivateMap")
        .into_inner();
        assert_eq!(activated.phase, "ACTIVE");
        let final_exploration = handle.explore.snapshot();
        assert_eq!(final_exploration.status, STATUS_FAILED);
        assert_eq!(final_exploration.last_stop_reason, "child_cancel_failed");
    }

    #[tokio::test]
    async fn start_mapping_publishes_one_monotonic_zero_cloud_frame_per_new_session() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let path = std::env::temp_dir().join(format!(
            "nuwax-cloudmap-start-mapping-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new_for_test(&path).expect("cloud map writer"),
        )));
        let mut cloud_updates = handle.cloud_map_tx.subscribe();
        let service = GeometryRuntimeService::new(handle.clone());

        let first = start_mapping_session(&service, "cloud-session-1", "warehouse-one").await;
        assert_eq!(first.session_generation, 1);
        let first_clear = tokio::time::timeout(Duration::from_secs(1), cloud_updates.recv())
            .await
            .expect("first clear timeout")
            .expect("first clear notification");
        assert_eq!(first_clear.point_count, 0);
        assert!(first_clear.revision > 0);
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            first_clear.revision
        );

        let replay = start_mapping_session(&service, "cloud-session-1", "warehouse-one").await;
        assert!(replay.idempotent_replay);
        assert!(
            tokio::time::timeout(Duration::from_millis(100), cloud_updates.recv())
                .await
                .is_err(),
            "idempotent StartMapping must not publish another clear revision"
        );
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            first_clear.revision
        );

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "cloud-session-1".to_string(),
                reason: "new session test".to_string(),
            }),
        )
        .await
        .expect("StopMapping");
        let second = start_mapping_session(&service, "cloud-session-2", "warehouse-two").await;
        assert_eq!(second.session_generation, 2);
        let second_clear = tokio::time::timeout(Duration::from_secs(1), cloud_updates.recv())
            .await
            .expect("second clear timeout")
            .expect("second clear notification");
        assert_eq!(second_clear.point_count, 0);
        assert!(second_clear.revision > first_clear.revision);
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            second_clear.revision
        );

        drop(service);
        drop(handle);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test]
    async fn mapping_rpc_fences_sessions_and_keeps_wire_revision_globally_monotonic() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());

        let first = start_mapping_session(&service, "session-1", "warehouse-one").await;
        assert_eq!(first.phase, "ACTIVE");
        assert_eq!(first.session_generation, 1);
        assert!(!first.idempotent_replay);
        let replay = start_mapping_session(&service, "session-1", "warehouse-one").await;
        assert!(replay.idempotent_replay);

        let response = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("SubscribeMapUpdates");
        let mut stream = response.into_inner();
        let first_revision = commit_test_map(&handle, 0.0);
        assert_eq!(first_revision, 1);
        let first_update = tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .expect("first session update timeout")
            .expect("first session stream")
            .expect("first session update");
        assert_eq!(first_update.meta.as_ref().unwrap().revision, first_revision);
        assert_eq!(first_update.session_generation, 1);

        let queued_event = handle.map_revision_latest_tx.borrow().clone().expect("actual committed first-session event");
        assert_eq!(queued_event.revision, first_revision);
        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "session-1".to_string(),
                reason: "test".to_string(),
            }),
        )
        .await
        .expect("StopMapping")
        .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert!(!stopped.active);
        handle.map_revision_tx.send(queued_event.clone()).expect("coalescer remains registered");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), stream.next())
                .await
                .is_err(),
            "stopped session must not publish queued map events"
        );

        {
            let mut shared = handle.state.write().await;
            shared.map.revision = 99;
        }
        *handle.occupancy_shm_current.write().await =
            Some(crate::lcm_runtime::OccupancyShmSnapshot {
                identity: crate::occupancy_shm::MapWriteIdentity::from_event(&queued_event).unwrap(),
                key: "/tmp/stale.mmap".to_string(),
                file_size: 8192,
                payload_size: 1,
                revision: 99,
                layout_version: 3,
                payload_schema_version: 1,
                generation: 1,
                active_slot: 0,
            });

        let second = start_mapping_session(&service, "session-2", "warehouse-two").await;
        assert_eq!(second.session_generation, 2);
        assert_eq!(second.revision, 0);
        assert_eq!(handle.state.read().await.map.revision, 0);
        assert_eq!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .global_revision(),
            first_revision
        );
        assert!(handle.map_revision_latest_tx.borrow().is_none());
        assert!(handle.occupancy_shm_current.read().await.is_none());

        let stale_stop = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "session-1".to_string(),
                reason: "late retry".to_string(),
            }),
        )
        .await
        .expect_err("old session stop must fail closed");
        assert_eq!(stale_stop.code(), tonic::Code::FailedPrecondition);
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());

        let second_revision = commit_test_map(&handle, 1.0);
        assert!(second_revision > first_revision);
        let second_update = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let update = stream
                    .next()
                    .await
                    .expect("second session stream")
                    .expect("second session update");
                if update.session_generation == 2 {
                    break update;
                }
            }
        })
        .await
        .expect("new session globally newer revision must not be deduplicated");
        assert_eq!(
            second_update.meta.as_ref().unwrap().revision,
            second_revision
        );
        assert_eq!(second_update.map_name, "warehouse-two");
    }

    #[tokio::test]
    async fn stop_mapping_waits_for_inflight_fanout_barrier() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "barrier-session", "warehouse").await;

        // Fanout sinks hold this lock from lifecycle validation through their
        // bounded sink operation. Holding it here deterministically pauses the
        // same critical section and proves StopMapping cannot overtake it.
        let fanout_guard = handle.mapping_lifecycle_lock.lock().await;
        let stop_service = service.clone();
        let stop_task = tokio::spawn(async move {
            GeometryRuntime::stop_mapping(
                &stop_service,
                Request::new(StopMappingRequest {
                    session_id: "barrier-session".to_string(),
                    reason: "barrier-test".to_string(),
                }),
            )
            .await
        });
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(
            !stop_task.is_finished(),
            "StopMapping must wait for an in-flight fanout critical section"
        );

        drop(fanout_guard);
        let stopped = tokio::time::timeout(Duration::from_secs(1), stop_task)
            .await
            .expect("StopMapping must finish after fanout barrier release")
            .expect("StopMapping join")
            .expect("StopMapping RPC")
            .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert!(!stopped.active);
    }

    #[tokio::test]
    async fn start_exploration_uses_adapter_configured_map_frame() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        *handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock() = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "robot/map",
            "odom",
        );
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        let service = GeometryRuntimeService::new(handle);
        start_mapping_session(&service, "robot-map-explore", "robot-map").await;
        let mut request = exploration_request("robot-map-boundary");
        request.boundary_frame_id = "robot/map".to_string();
        request.boundary_line_segments = vec![
            ExplorationBoundarySegment {
                start_x: 0.0,
                start_y: 0.0,
                end_x: 2.0,
                end_y: 0.0,
            },
            ExplorationBoundarySegment {
                start_x: 2.0,
                start_y: 0.0,
                end_x: 0.0,
                end_y: 2.0,
            },
            ExplorationBoundarySegment {
                start_x: 0.0,
                start_y: 2.0,
                end_x: 0.0,
                end_y: 0.0,
            },
        ];

        let started = GeometryRuntime::start_exploration(&service, Request::new(request))
            .await
            .expect("configured robot/map boundary")
            .into_inner();
        assert_eq!(started.status, STATUS_ACCEPTED);
        assert_eq!(started.request_id, "robot-map-boundary");
    }

    #[tokio::test]
    async fn configured_robot_map_checkpoint_publish_and_subscription_preserve_frame() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        *handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock() = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "robot/map",
            "odom",
        );
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activate-robot-map-test-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new_for_test(&cloud_path).expect("configured-frame cloud-map writer"));
        let backend = Arc::new(CheckpointPublishBackend::default());
        handle.transport = LcmTransport::new(backend.clone());
        let service = GeometryRuntimeService::new(handle.clone());
        let mut committed_revisions = handle.map_revision_tx.subscribe();
        let mut updates = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("configured-frame map subscription")
            .into_inner();
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();

        let wrong = cloud_activation_request(cloud_xyz.clone());
        let error = GeometryRuntime::activate_map(&service, Request::new(wrong))
            .await
            .expect_err("literal map checkpoint must fail for robot/map runtime");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert!(error.message().contains("frame"), "{error}");

        let mut wrong_cloud = cloud_activation_request(cloud_xyz.clone());
        wrong_cloud
            .mapping_backend_checkpoint.as_mut().unwrap().canonical_occupancy_checkpoint.as_mut()
            .and_then(|checkpoint| checkpoint.meta.as_mut())
            .expect("occupancy checkpoint metadata")
            .frame_id = "robot/map".to_string();
        let error = GeometryRuntime::activate_map(&service, Request::new(wrong_cloud))
            .await
            .expect_err("literal map cloud checkpoint must fail for robot/map runtime");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert!(error.message().contains("frame"), "{error}");
        assert!(error.message().contains("frame"), "{error}");

        let mut request = cloud_activation_request(cloud_xyz.clone());
        request
            .mapping_backend_checkpoint.as_mut().unwrap().canonical_occupancy_checkpoint.as_mut()
            .and_then(|checkpoint| checkpoint.meta.as_mut())
            .expect("occupancy checkpoint metadata")
            .frame_id = "robot/map".to_string();
        request
            .mapping_backend_checkpoint.as_mut().unwrap().canonical_cloud_checkpoint.as_mut()
            .expect("cloud checkpoint")
            .frame_id = "robot/map".to_string();
        request.mapping_backend_checkpoint=Some(assemble_mapping_checkpoint_export(request.mapping_backend_checkpoint.take().unwrap()).unwrap());
        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("robot/map checkpoint activation")
            .into_inner();
        assert!(activated.active);
        let committed_event =
            tokio::time::timeout(Duration::from_secs(1), committed_revisions.recv())
                .await
                .expect("robot/map checkpoint event timeout")
                .expect("robot/map checkpoint event");
        assert_eq!(committed_event.frame_id, "robot/map");
        handle
            .map_revision_latest_tx
            .send_replace(Some(committed_event));

        let update = tokio::time::timeout(Duration::from_secs(1), updates.next())
            .await
            .expect("robot/map subscription timeout")
            .expect("robot/map subscription item")
            .expect("robot/map update");
        assert_eq!(
            update.meta.expect("robot/map metadata").frame_id,
            "robot/map"
        );
        assert_eq!(handle.state.read().await.map.frame_id, "robot/map");

        let published = backend
            .published
            .lock()
            .expect("configured-frame published frames");
        let published: Vec<_> = published.iter()
            .filter(|frame| frame.msg_type.fqn() == "sensor_msgs.PointCloud2").collect();
        assert_eq!(published.len(), 1);
        let mut encoded = Vec::with_capacity(8 + published[0].payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&published[0].payload);
        let cloud = schema::decode_point_cloud2(&encoded).expect("configured-frame PointCloud2");
        assert_eq!(cloud.header.frame_id, "robot/map");
        assert_eq!(cloud.data, cloud_xyz);
        drop(published);
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn mapping_state_exposes_backend_kind_and_checkpoint_provenance() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let idle = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .expect("idle voxel state")
            .into_inner();
        assert_eq!(idle.mapping_backend_kind, "voxel");

        let started = start_mapping_session(&service, "typed-state", "warehouse").await;
        assert_eq!(started.mapping_backend_kind, "voxel");
        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .expect("active voxel state")
            .into_inner();
        assert_eq!(queried.mapping_backend_kind, "voxel");

        let mut ray_handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("ray real-lcm test handle");
        ray_handle.mapping_adapter = Some(Arc::new(parking_lot::Mutex::new(ray_mapping_adapter())));
        let ray_service = GeometryRuntimeService::new(ray_handle);
        let ray_idle = GeometryRuntime::get_mapping_state(&ray_service, Request::new(Empty {}))
            .await
            .expect("idle ray state")
            .into_inner();
        assert_eq!(ray_idle.mapping_backend_kind, "raytracing");

        let legacy_handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("legacy real-lcm test handle");
        let legacy_service = GeometryRuntimeService::new(legacy_handle);
        let legacy = GeometryRuntime::activate_map(
            &legacy_service,
            Request::new(activation_request("mapload:legacy", "warehouse-legacy")),
        )
        .await
        .expect("legacy voxel activation")
        .into_inner();
        assert_eq!(legacy.mapping_backend_kind, "voxel");
        let legacy_queried =
            GeometryRuntime::get_mapping_state(&legacy_service, Request::new(Empty {}))
                .await
                .expect("legacy GetMappingState")
                .into_inner();
        assert_eq!(legacy_queried.mapping_backend_kind, "voxel");
    }

    #[test]
    fn backend_checkpoint_receipt_validation_covers_independent_lag_and_mismatches() {
        let checkpoint = canonical_backend_checkpoint_fixture();
        validate_assigned_mapping_checkpoint(
            &checkpoint,
            crate::mapping_backend::MappingBackendKind::Voxel,
        )
        .expect("independently lagging cloud receipt must remain valid");

        let mut zero_timestamp = canonical_backend_checkpoint_fixture();
        zero_timestamp
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .source_timestamp_ns = 0;
        zero_timestamp
            .canonical_occupancy_checkpoint
            .as_mut()
            .unwrap()
            .meta
            .as_mut()
            .unwrap()
            .timestamp_ns = 0;
        zero_timestamp
            .cloud_receipt
            .as_mut()
            .unwrap()
            .source_timestamp_ns = 0;
        validate_assigned_mapping_checkpoint(
            &zero_timestamp,
            crate::mapping_backend::MappingBackendKind::Voxel,
        )
        .expect_err("现行编号检查点首次准入要求已观测源时间");

        macro_rules! assert_invalid {
            ($label:literal, $mutate:expr) => {{
                let mut invalid = canonical_backend_checkpoint_fixture();
                ($mutate)(&mut invalid);
                assert!(
                    validate_assigned_mapping_checkpoint(
                        &invalid,
                        crate::mapping_backend::MappingBackendKind::Voxel,
                    )
                    .is_err(),
                    "{}",
                    $label
                );
            }};
        }

        assert_invalid!("schema", |value: &mut MappingBackendCheckpoint| value
            .schema_version =
            0);
        assert_invalid!("kind", |value: &mut MappingBackendCheckpoint| value
            .backend_kind =
            "raytracing".to_owned());
        assert_invalid!("source session", |value: &mut MappingBackendCheckpoint| {
            value.source_session_id = String::new()
        });
        assert_invalid!(
            "source generation",
            |value: &mut MappingBackendCheckpoint| value.source_session_generation = 0
        );
        assert_invalid!("creation time", |value: &mut MappingBackendCheckpoint| {
            value.created_at_ns = 0
        });
        assert_invalid!("payload size", |value: &mut MappingBackendCheckpoint| {
            value.payload_size += 1
        });
        assert_invalid!(
            "occupancy receipt",
            |value: &mut MappingBackendCheckpoint| value.occupancy_receipt = None
        );
        assert_invalid!("cloud receipt", |value: &mut MappingBackendCheckpoint| {
            value.cloud_receipt = None
        });
        assert_invalid!(
            "occupancy commit id",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .commit_id = None
        );
        assert_invalid!("cloud commit id", |value: &mut MappingBackendCheckpoint| {
            value.cloud_receipt.as_mut().unwrap().commit_id = None
        });
        assert_invalid!(
            "occupancy nonce length",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .boot_nonce = vec![1; 15]
        );
        assert_invalid!(
            "cloud nonce zero",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .boot_nonce = vec![0; 16]
        );
        assert_invalid!(
            "receipt nonce mismatch",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .boot_nonce = vec![2; 16]
        );
        assert_invalid!(
            "occupancy sequence",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .sequence = 0
        );
        assert_invalid!("cloud sequence", |value: &mut MappingBackendCheckpoint| {
            value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .sequence = 0
        });
        assert_invalid!(
            "cloud sequence lead",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .commit_id
                .as_mut()
                .unwrap()
                .sequence = 10
        );
        assert_invalid!(
            "occupancy session",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .session_id = "other".to_owned()
        );
        assert_invalid!(
            "occupancy generation",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .session_generation = 8
        );
        assert_invalid!(
            "cloud generation",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .session_generation = 8
        );
        assert_invalid!(
            "occupancy revision",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .occupancy_revision = 0
        );
        assert_invalid!(
            "cloud occupancy revision",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .occupancy_revision = 0
        );
        assert_invalid!(
            "cloud occupancy lead",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .occupancy_revision = 21
        );
        assert_invalid!(
            "sequence revision correlation",
            |value: &mut MappingBackendCheckpoint| {
                value
                    .cloud_receipt
                    .as_mut()
                    .unwrap()
                    .commit_id
                    .as_mut()
                    .unwrap()
                    .sequence = 9;
            }
        );
        assert_invalid!(
            "same commit timestamp",
            |value: &mut MappingBackendCheckpoint| {
                let occupancy = value.occupancy_receipt.as_ref().unwrap();
                let cloud = value.cloud_receipt.as_mut().unwrap();
                cloud.commit_id.as_mut().unwrap().sequence =
                    occupancy.commit_id.as_ref().unwrap().sequence;
                cloud.occupancy_revision = occupancy.occupancy_revision;
                cloud.source_timestamp_ns = occupancy.source_timestamp_ns + 1;
            }
        );
        assert_invalid!("cloud revision", |value: &mut MappingBackendCheckpoint| {
            value.cloud_receipt.as_mut().unwrap().cloud_revision = 0
        });
        assert_invalid!(
            "cloud point count",
            |value: &mut MappingBackendCheckpoint| value
                .cloud_receipt
                .as_mut()
                .unwrap()
                .point_count = 0
        );
        assert_invalid!(
            "receipt backend kind",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .backend_kind = "raytracing".to_owned()
        );
        assert_invalid!(
            "receipt backend generation",
            |value: &mut MappingBackendCheckpoint| value
                .occupancy_receipt
                .as_mut()
                .unwrap()
                .backend_generation = 0
        );
        assert_invalid!(
            "occupancy metadata revision",
            |value: &mut MappingBackendCheckpoint| value
                .canonical_occupancy_checkpoint
                .as_mut()
                .unwrap()
                .meta
                .as_mut()
                .unwrap()
                .revision += 1
        );
        assert_invalid!(
            "occupancy metadata timestamp",
            |value: &mut MappingBackendCheckpoint| value
                .canonical_occupancy_checkpoint
                .as_mut()
                .unwrap()
                .meta
                .as_mut()
                .unwrap()
                .timestamp_ns += 1
        );
        assert_invalid!(
            "cloud metadata revision",
            |value: &mut MappingBackendCheckpoint| value
                .canonical_cloud_checkpoint
                .as_mut()
                .unwrap()
                .source_revision += 1
        );
        assert_invalid!(
            "cloud metadata point count",
            |value: &mut MappingBackendCheckpoint| value
                .canonical_cloud_checkpoint
                .as_mut()
                .unwrap()
                .point_count += 1
        );
        assert_invalid!(
            "canonical occupancy",
            |value: &mut MappingBackendCheckpoint| value.canonical_occupancy_checkpoint = None
        );
        assert_invalid!("canonical cloud", |value: &mut MappingBackendCheckpoint| {
            value.canonical_cloud_checkpoint = None
        });
    }

    #[test]
    fn activation_bundle_binds_receipts_and_bounds_envelope() {
        let checkpoint=canonical_backend_checkpoint_fixture();
        let mut bad=checkpoint.clone();
        bad.canonical_occupancy_checkpoint.as_mut().unwrap().meta.as_mut().unwrap().timestamp_ns+=1;
        assert!(validate_mapping_checkpoint_bundle(&bad).is_err());
        let mut bad=checkpoint.clone();
        bad.canonical_cloud_checkpoint.as_mut().unwrap().point_count+=1;
        assert!(validate_mapping_checkpoint_bundle(&bad).is_err());
        let mut bad=checkpoint;
        bad.canonical_cloud_checkpoint=None;
        assert!(validate_mapping_checkpoint_bundle(&bad).is_err());

        validate_mapping_checkpoint_envelope_size(
            "test checkpoint",
            MAPPING_CHECKPOINT_PROTO_ENVELOPE_MAX_BYTES,
        )
        .expect("exact envelope limit");
        assert!(validate_mapping_checkpoint_envelope_size(
            "test checkpoint",
            MAPPING_CHECKPOINT_PROTO_ENVELOPE_MAX_BYTES + 1,
        )
        .is_err());
    }

    #[tokio::test]
    async fn activation_rejects_receipt_mismatch_before_any_runtime_mutation() {
        let mut checkpoint = canonical_backend_checkpoint_fixture();
        checkpoint.source_session_id = "mismatched-source".to_owned();
        let request = activation_request_with_backend_checkpoint(
            "mapload:metadata-mismatch",
            "warehouse-metadata-mismatch",
            checkpoint,
        );
        let handle = crate::lcm_runtime::build_handle_for_tests_without_cloud_writer()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let error = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect_err("receipt mismatch must fail before activation");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        let snapshot = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        assert_eq!(
            snapshot.phase,
            crate::mapping_adapter::MappingSessionPhase::Idle
        );
        assert_eq!(snapshot.session_generation, 0);
        assert_eq!(snapshot.revision, 0);
        assert!(!snapshot.active);
        assert_eq!(handle.state.read().await.map.revision, 0);
        assert!(handle.cloud_map_writer.lock().is_none());
    }

    #[tokio::test]
    async fn activation_preserves_exact_checkpoint_timestamp_bits() {
        for (index, timestamp_ns) in [0_u64, 1_700_000_000_123_456_789].into_iter().enumerate() {
            let handle = crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let service = GeometryRuntimeService::new(handle.clone());
            let mut request = activation_request(
                &format!("mapload:exact-timestamp-{index}"),
                &format!("warehouse-exact-timestamp-{index}"),
            );
            request
                .mapping_backend_checkpoint.as_mut().unwrap().canonical_occupancy_checkpoint.as_mut()
                .unwrap()
                .meta
                .as_mut()
                .unwrap()
                .timestamp_ns = timestamp_ns;
            let checkpoint=request.mapping_backend_checkpoint.as_mut().unwrap();
            checkpoint.occupancy_receipt.as_mut().unwrap().source_timestamp_ns=timestamp_ns;
            checkpoint.cloud_receipt.as_mut().unwrap().source_timestamp_ns=timestamp_ns;
            request.mapping_backend_checkpoint=Some(assemble_mapping_checkpoint_export(request.mapping_backend_checkpoint.take().unwrap()).unwrap());
            if timestamp_ns==0 {
                let error=GeometryRuntime::activate_map(&service,Request::new(request)).await.unwrap_err();
                assert_eq!(error.code(),tonic::Code::InvalidArgument);
                assert!(!handle.mapping_adapter.as_ref().unwrap().lock().is_active());
                continue;
            }
            let activated = GeometryRuntime::activate_map(&service, Request::new(request))
                .await
                .expect("activate exact-timestamp checkpoint")
                .into_inner();
            assert!(activated.active);
            assert_eq!(handle.state.read().await.map.timestamp_ns, timestamp_ns);
            let adapter = handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock();
            assert_eq!(
                adapter
                    .raw_map_snapshot()
                    .expect("raw activated checkpoint")
                    .timestamp_ns,
                timestamp_ns
            );
            assert_eq!(
                adapter
                    .latest_occupancy_commit_receipt()
                    .expect("activation occupancy receipt")
                    .source_timestamp_ns,
                timestamp_ns
            );
        }
    }

    // These server integration fixtures require liblcm (the existing isolated
    // test URL), writable temporary storage, and no live robot or sensor.
    async fn checkpoint_frame_test_handle() -> GeometryRuntimeHandle {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("checkpoint frame real-lcm test handle");
        *handle.mapping_adapter.as_ref().unwrap().lock() =
            crate::mapping_adapter::MappingAdapter::new_with_backend_for_tests(
                crate::mapping_adapter::MappingAdapterConfig {
                    resolution: 0.1,
                    initial_safe_radius_meters: 0.0,
                },
                crate::mapping_backend::MappingBackendSpec::Voxel,
                "map",
                "odom",
            );
        handle
    }

    async fn commit_checkpoint_frame_test_pose(handle: &GeometryRuntimeHandle, timestamp_ns: u64) {
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let pose = PoseState {
            x: 4.0,
            y: 2.0,
            z: 1.0,
            qw: 1.0,
            frame_id: "odom".to_owned(),
            timestamp_ns,
            ..PoseState::default()
        };
        let binding = (
            LcmTopic::new("/test/geometry/checkpoint-frame-tf").unwrap(),
            LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
        );
        assert!(crate::lcm_runtime::handoff_lio_odometry(handle, &pose, "base_link", &binding)
            .await
            .expect("commit real first-pose/anchor handoff"));
    }

    #[tokio::test]
    async fn activation_first_real_pose_exports_v2_without_another_scan() {
        const SOURCE_NS: u64 = 1_700_000_000_123_456_789;
        let handle = checkpoint_frame_test_handle().await;
        let mut owner = crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
        let handle = owner.handle.clone();
        let service = GeometryRuntimeService::new(handle.clone());
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .into_iter()
            .flatten()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let mut request = cloud_activation_request(xyz);
        request.mapping_backend_checkpoint.as_mut().unwrap().canonical_occupancy_checkpoint.as_mut().unwrap().meta.as_mut().unwrap().timestamp_ns = SOURCE_NS;
        let checkpoint=request.mapping_backend_checkpoint.as_mut().unwrap();
        checkpoint.occupancy_receipt.as_mut().unwrap().source_timestamp_ns=SOURCE_NS;
        checkpoint.cloud_receipt.as_mut().unwrap().source_timestamp_ns=SOURCE_NS;
        request.mapping_backend_checkpoint=Some(assemble_mapping_checkpoint_export(request.mapping_backend_checkpoint.take().unwrap()).unwrap());
        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("supported cloud activation")
            .into_inner();
        assert!(activated.success && activated.active);
        let (occupancy_before, raw_before, frame_before) = {
            let adapter = handle.mapping_adapter.as_ref().unwrap().lock();
            assert!(adapter.map_odom_transform_snapshot().is_none());
            (
                adapter.latest_occupancy_commit_receipt().unwrap(),
                occupancy_checkpoint_proto(&adapter.raw_map_snapshot().unwrap()),
                adapter.global_frame_generation(),
            )
        };
        let (cloud_before, snapshot_before, old_fine) = {
            let writer = handle.cloud_map_writer.lock();
            let writer = writer.as_ref().unwrap();
            (
                writer.last_cloud_receipt().unwrap(),
                writer.committed_snapshot().unwrap(),
                writer.committed_fine_cloud().unwrap(),
            )
        };
        assert_eq!(old_fine.manifest().domain.frame_generation, frame_before);
        assert!(!old_fine.manifest().blocks.is_empty());
        assert_eq!(old_fine.manifest().source_timestamp_ns, SOURCE_NS);
        assert_eq!(occupancy_before.source_timestamp_ns, SOURCE_NS);
        assert_eq!(cloud_before.source_timestamp_ns, SOURCE_NS);

        // No cloud handoff occurs between activation and either export.
        commit_checkpoint_frame_test_pose(&handle, SOURCE_NS + 17_000_001).await;
        let frame_after = handle.mapping_adapter.as_ref().unwrap().lock().global_frame_generation();
        assert_eq!(frame_after, frame_before + 1);
        assert_eq!(handle.state.read().await.pose.timestamp_ns, SOURCE_NS + 17_000_001);
        let anchored_fine = handle.cloud_map_writer.lock().as_ref().unwrap()
            .committed_fine_cloud().unwrap();
        assert!(!Arc::ptr_eq(&old_fine, &anchored_fine));
        let mut expected_root = old_fine.manifest().as_ref().clone();
        expected_root.domain.frame_generation = frame_after;
        assert_eq!(anchored_fine.manifest().as_ref(), &expected_root,
            "first pose may change only the root frame domain, never IDs, evidence or revisions");

        let checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &service,
            Request::new(ExportMappingBackendCheckpointRequest { }),
        ).await.expect("v2 export immediately after the first real pose").into_inner();
        assert_eq!(checkpoint.schema_version, 2);
        assert_eq!(checkpoint.occupancy_receipt, Some(occupancy_commit_receipt_proto(&occupancy_before)));
        assert_eq!(checkpoint.cloud_receipt, Some(cloud_commit_receipt_proto(&cloud_before)));
        assert_eq!(checkpoint.canonical_occupancy_checkpoint.as_ref(), Some(&raw_before));
        let exported_cloud = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
        assert_eq!(exported_cloud.payload_schema_version, 2);
        assert_eq!(exported_cloud.xyz_f32_le, snapshot_before.xyz_f32_le);
        assert_eq!(exported_cloud.source_revision, snapshot_before.revision);
        assert_eq!(exported_cloud.point_count, snapshot_before.point_count);
        assert_eq!(exported_cloud.voxel_size.to_bits(), snapshot_before.voxel_size.to_bits());
        let fine_ref = checkpoint.fine_map_artifact.as_ref().expect("AR4 fine checkpoint member");
        assert_eq!(fine_ref.identity_schema_version, 2);

        let member = fine_ref.member_identity.as_ref().unwrap();
        assert_eq!(member.frame_generation, frame_after);
        assert_eq!(member.producer_generation, handle.nav.producer_runtime_generation());
        let file = service.graph_artifacts.lock().open(fine_ref).expect("open exported fine member");
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file).expect("read AR4 fine member");
        assert_eq!(archive.root(), &nuwax_map_chunks::ArchivedRoot::Current(expected_root));
        for reference in &old_fine.manifest().blocks {
            assert!(matches!(reference.object, nuwax_map_chunks::ObjectIdentity::Assigned(_)));
            let bytes_before = old_fine.repository
                .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
            assert_eq!(archive.block(reference).unwrap(), bytes_before);
            let pinned = handle.cloud_map_writer.lock().as_ref().unwrap()
                .pin_fine_map_root(&old_fine.manifest().identity(), reference)
                .expect("old published root remains pinnable after frame rehome/export");
            assert!(Arc::ptr_eq(&pinned, &old_fine));
            let fetched = handle.read_navigation_map_block(&old_fine.manifest().identity(), reference, None)
                .expect("admit fetch under retained old root")
                .await.expect("retained fetch completion").expect("retained old object bytes");
            let nuwax_map_chunks::ObjectRangeRead::Data { bytes: fetched, total_bytes: total } = fetched else {
                panic!("full retained read cannot be out of range");
            };
            assert_eq!(fetched, bytes_before);
            assert_eq!(total, bytes_before.len());
        }

        // A later odometry observation must not reseal the already anchored root.
        commit_checkpoint_frame_test_pose(&handle, SOURCE_NS + 34_000_002).await;
        let repeated = GeometryRuntime::export_mapping_backend_checkpoint(
            &service,
            Request::new(ExportMappingBackendCheckpointRequest { }),
        ).await.expect("repeat v2 export without a scan").into_inner();
        assert_eq!(repeated.occupancy_receipt, checkpoint.occupancy_receipt);
        assert_eq!(repeated.cloud_receipt, checkpoint.cloud_receipt);
        assert_eq!(repeated.canonical_occupancy_checkpoint, checkpoint.canonical_occupancy_checkpoint);
        assert_eq!(repeated.canonical_cloud_checkpoint.as_ref().unwrap().xyz_f32_le,
            snapshot_before.xyz_f32_le);
        {
            let adapter = handle.mapping_adapter.as_ref().unwrap().lock();
            assert_eq!(adapter.global_frame_generation(), frame_after);
            assert_eq!(adapter.latest_occupancy_commit_receipt(), Some(occupancy_before));
        }
        {
            let writer = handle.cloud_map_writer.lock();
            let writer = writer.as_ref().unwrap();
            assert_eq!(writer.last_cloud_receipt(), Some(cloud_before));
            assert!(Arc::ptr_eq(&anchored_fine, &writer.committed_fine_cloud().unwrap()));
            assert_eq!(writer.committed_snapshot().unwrap().xyz_f32_le, snapshot_before.xyz_f32_le);
        }
        assert_eq!(handle.state.read().await.cloud_map_revision, snapshot_before.revision);
        GeometryRuntime::stop_mapping(&service, Request::new(StopMappingRequest {
            session_id: activated.session_id,
            reason: "first-pose checkpoint regression complete".to_owned(),
        })).await.expect("stop activated checkpoint session");
        assert!(owner.task_owner.shutdown().await.producers_settled());
    }

    #[tokio::test]
    async fn retained_navigation_block_serves_both_wire_versions() {
        let handle = checkpoint_frame_test_handle().await;
        let mut owner = crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
        let handle = owner.handle.clone();
        let service = GeometryRuntimeService::new(handle.clone());
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .into_iter().flatten().flat_map(f32::to_le_bytes).collect();
        let activated = GeometryRuntime::activate_map(&service, Request::new(cloud_activation_request(xyz)))
            .await.unwrap().into_inner();
        assert!(activated.success && activated.active);
        let fine = handle.cloud_map_writer.lock().as_ref().unwrap().committed_fine_cloud().unwrap();
        let root = fine.manifest();
        let reference = root.blocks.first().unwrap();
        let expected = fine.repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
        let object = match reference.object {
            nuwax_map_chunks::ObjectIdentity::Assigned(id) => NavigationMapObjectIdentity {
                kind: 1, object_id: id.0.to_vec(), legacy_alias: Vec::new(),
            },
            nuwax_map_chunks::ObjectIdentity::Legacy(id) => NavigationMapObjectIdentity {
                kind: 2, object_id: Vec::new(), legacy_alias: id.0.to_vec(),
            },
        };
        let wire_root = NavigationMapRootIdentity { revision: root.revision, domain: Some(NavigationMapDomain {
            session: root.domain.session.clone(), generation: root.domain.generation,
            frame_generation: root.domain.frame_generation, producer_generation: root.domain.producer_generation,
        }) };
        let wire_block = NavigationMapBlockRef { key: reference.key.to_vec(), object: Some(object),
            block_revision: reference.block_revision, geometry_revision: reference.geometry_revision, points: reference.points };
        for schema_version in [2, 3] {
            let request = NavigationMapBlockRequest { schema_version,
                root: Some(wire_root.clone()), block: Some(wire_block.clone()), offset:0,
                max_bytes:if schema_version==3 {nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES as u32} else {0} };
            let response = GeometryRuntime::get_navigation_map_block(&service, Request::new(request.clone()))
                .await.expect("两版编码共用真实保留根与读取拥有方").into_inner();
            assert_eq!(response.schema_version, schema_version);
            assert_eq!(response.root, request.root);
            assert_eq!(response.block, request.block);
            assert_eq!(response.data, expected);
            assert_eq!(response.total_bytes,if schema_version==3 {expected.len() as u32} else {0});
        }
        let mut joined=Vec::new();
        while joined.len()<expected.len() {
            let response=GeometryRuntime::get_navigation_map_block(&service,Request::new(NavigationMapBlockRequest {
                schema_version:3,root:Some(wire_root.clone()),block:Some(wire_block.clone()),
                offset:joined.len() as u32,max_bytes:7,
            })).await.unwrap().into_inner();
            assert_eq!(response.offset as usize,joined.len());
            assert_eq!(response.total_bytes as usize,expected.len());
            assert_eq!(response.data.len(),7.min(expected.len()-joined.len()));
            joined.extend_from_slice(&response.data);
        }
        assert_eq!(joined,expected);
        for (offset,max_bytes) in [(0,0),(expected.len() as u32,7),(0,nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES as u32+1)] {
            let error = GeometryRuntime::get_navigation_map_block(&service,Request::new(NavigationMapBlockRequest {
                schema_version:3,root:Some(wire_root.clone()),block:Some(wire_block.clone()),offset,max_bytes,
            })).await.unwrap_err();
            assert_eq!(error.code(), tonic::Code::InvalidArgument);
        }
        for schema_version in [0, 1, 4] {
            assert!(GeometryRuntime::get_navigation_map_block(&service, Request::new(NavigationMapBlockRequest {
                schema_version, root: Some(wire_root.clone()), block: Some(wire_block.clone()),offset:0,max_bytes:0,
            })).await.is_err());
        }
        GeometryRuntime::stop_mapping(&service, Request::new(StopMappingRequest {
            session_id: activated.session_id, reason: "块编码版本回归结束".into(),
        })).await.unwrap();
        assert!(owner.task_owner.shutdown().await.producers_settled());
    }

    include!("navigation_block_unit_transport_tests.rs");

    #[tokio::test]
    async fn checkpoint_v2_flush_keeps_the_actual_first_pose_frame() {
        const FIRST_RECEIVED_NS: u64 = 2_000_000_000;
        const FIRST_SOURCE_NS: u64 = 1_700_000_000_123_456_789;
        let handle = checkpoint_frame_test_handle().await;
        let service = GeometryRuntimeService::new(handle.clone());
        let session_id = "checkpoint-current-frame-flush";
        assert!(start_mapping_session(&service, session_id, "warehouse-flush").await.active);
        let frame_before = handle.mapping_adapter.as_ref().unwrap().lock().global_frame_generation();
        commit_checkpoint_frame_test_pose(&handle, FIRST_SOURCE_NS - 1).await;
        let current_frame = handle.mapping_adapter.as_ref().unwrap().lock().global_frame_generation();
        assert_eq!(current_frame, frame_before + 1);

        let points = [Point3::new(0.25, 0.0, 1.0), Point3::new(0.75, 0.5, 1.0)];
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &handle, &points, 12, FIRST_RECEIVED_NS, FIRST_SOURCE_NS, "map",
        ).await.expect("first source scan"));
        let first_receipt = handle.mapping_adapter.as_ref().unwrap().lock()
            .latest_occupancy_commit_receipt().expect("first source occupancy commit");
        let first_cloud_receipt = handle.cloud_map_writer.lock().as_ref().unwrap()
            .last_cloud_receipt().unwrap();
        let first_fine = handle.cloud_map_writer.lock().as_ref().unwrap()
            .committed_fine_cloud().unwrap();
        assert_eq!(first_fine.manifest().domain.frame_generation, current_frame);

        // This second, real source scan is the pending work the checkpoint must
        // flush. There is no third scan or artificial frame initialization.
        let deferred_received_ns = FIRST_RECEIVED_NS
            + crate::mapping_backend::MAPPING_CANDIDATE_INTERVAL_NS / 2;
        let deferred_source_ns = FIRST_SOURCE_NS + 7_000_003;
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &handle, &points, 12, deferred_received_ns, deferred_source_ns, "map",
        ).await.expect("cadence-deferred source scan"));
        assert_eq!(handle.mapping_adapter.as_ref().unwrap().lock()
            .latest_occupancy_commit_receipt(), Some(first_receipt.clone()));
        {
            let writer = handle.cloud_map_writer.lock();
            let writer = writer.as_ref().unwrap();
            assert_eq!(writer.last_cloud_receipt(), Some(first_cloud_receipt.clone()));
            assert!(Arc::ptr_eq(&first_fine, &writer.committed_fine_cloud().unwrap()));
        }

        let checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &service,
            Request::new(ExportMappingBackendCheckpointRequest { }),
        ).await.expect("explicit v2 export flushes pending source work in the current frame").into_inner();
        let occupancy = checkpoint.occupancy_receipt.as_ref().unwrap();
        let cloud = checkpoint.cloud_receipt.as_ref().unwrap();
        assert!(occupancy.occupancy_revision > first_receipt.occupancy_revision);
        assert!(cloud.cloud_revision > first_cloud_receipt.cloud_revision);
        assert_eq!(occupancy.source_timestamp_ns, deferred_source_ns);
        assert_eq!(cloud.source_timestamp_ns, deferred_source_ns);
        assert_eq!(cloud.commit_id, occupancy.commit_id);
        let fine = handle.cloud_map_writer.lock().as_ref().unwrap().committed_fine_cloud().unwrap();
        assert_eq!(fine.manifest().domain.frame_generation, current_frame);
        assert_eq!(fine.manifest().domain.producer_generation, handle.nav.producer_runtime_generation());
        assert_eq!(fine.manifest().revision, cloud.cloud_revision);
        assert_eq!(fine.manifest().source_timestamp_ns, deferred_source_ns);
        assert_eq!(handle.mapping_adapter.as_ref().unwrap().lock().global_frame_generation(), current_frame);
        let fine_ref = checkpoint.fine_map_artifact.as_ref().unwrap();
        assert_eq!(fine_ref.member_identity.as_ref().unwrap().frame_generation, current_frame);
        let file = service.graph_artifacts.lock().open(fine_ref).unwrap();
        let archive = nuwax_map_chunks::ArchiveReader::open(file).unwrap();
        assert_eq!(archive.root(), &nuwax_map_chunks::ArchivedRoot::Current(fine.manifest().as_ref().clone()));
        GeometryRuntime::stop_mapping(&service, Request::new(StopMappingRequest {
            session_id: session_id.to_owned(),
            reason: "current-frame checkpoint flush regression complete".to_owned(),
        })).await.expect("stop checkpoint flush session");
    }

    #[tokio::test]
    async fn backend_checkpoint_export_and_voxel_activation_round_trip() {
        let (checkpoint,store) = export_voxel_backend_checkpoint().await;
        assert_eq!(
            checkpoint.schema_version,
            2
        );
        assert_eq!(checkpoint.backend_kind, "voxel");
        assert_eq!(checkpoint.source_session_id, "checkpoint-source");
        assert!(checkpoint.source_session_generation > 0);
        assert_eq!(
            checkpoint.payload_size,
            u64::try_from(checkpoint.payload.len()).unwrap()
        );

        let occupancy_receipt = checkpoint
            .occupancy_receipt
            .as_ref()
            .expect("canonical occupancy receipt");
        let cloud_receipt = checkpoint
            .cloud_receipt
            .as_ref()
            .expect("canonical cloud receipt");
        assert_eq!(occupancy_receipt.source_timestamp_ns, 1_700_000_000_123_456_789);
        assert_eq!(cloud_receipt.source_timestamp_ns, 1_700_000_000_123_456_789);
        assert_eq!(
            occupancy_receipt
                .commit_id
                .as_ref()
                .expect("occupancy commit identity")
                .boot_nonce,
            cloud_receipt
                .commit_id
                .as_ref()
                .expect("cloud commit identity")
                .boot_nonce
        );


        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("restored real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-backend-checkpoint-target-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new_for_test(&cloud_path).expect("target cloud-map writer"),
        )));
        let mut root_owner=crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
        let handle=root_owner.handle.clone();
        let mut service = GeometryRuntimeService::new(handle.clone());
        service.graph_artifacts=store.clone();
        let mut request = activation_request("mapload:restored", "warehouse-restored");
        request.mapping_backend_checkpoint = Some(checkpoint);
        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("activate voxel backend checkpoint")
            .into_inner();
        assert!(activated.active);

        assert_eq!(activated.mapping_backend_kind, "voxel");
        assert_eq!(handle.state.read().await.map.revision, activated.revision);
        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .expect("GetMappingState after supplied checkpoint")
            .into_inner();
        assert_eq!(queried.mapping_backend_kind, "voxel");
        drop(service);
        drop(handle);
        assert!(root_owner.task_owner.shutdown().await.producers_settled());
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn dense_chunk_finishes_pinned_version_during_dynamic_removal_and_rejects_new_owner() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        use crate::mapping_commit::{MappingCommitId, OccupancyCommitReceipt};
        let (handle, _, path) = ray_runtime_with_cloud_writer("动态分块").await;
        let mut owner = crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
        let handle = owner.handle.clone();
        let service = GeometryRuntimeService::new(handle.clone());
        assert!(start_mapping_session(&service, "动态分块", "动态分块").await.active);
        let mut pinned: Option<DenseCloudChunk> = None;
        let mut expected_tail = Vec::new();
        for frame in 1..=22 {
            let person = (7..=16).contains(&frame).then(|| -0.5 + (frame - 7) as f64 * 0.12);
            let (points, _) = corridor(person, (frame % 3) as f64);
            {
                let mut guard = handle.cloud_map_writer.lock();
                let writer = guard.as_mut().unwrap();
                assert!(writer.accumulate_mapping_scan(1, &points, Some(ORIGIN), frame * 100_000_000));
                let receipt = OccupancyCommitReceipt {
                    commit_id: MappingCommitId { boot_nonce: 800, sequence: frame },
                    session_id: "动态分块".into(), session_generation: 1, occupancy_revision: frame,
                    source_timestamp_ns: frame * 100_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Raytracing,
                    backend_generation: 1,
                };
                writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap();
                if frame == 10 {
                    expected_tail = writer.committed_dense_cloud().unwrap().unwrap().read(10, 100).unwrap();
                }
            }
            if frame == 10 {
                pinned = Some(GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest { allow_delta: false,
                    expected_epoch: String::new(), offset_points: 0, max_points: 10,
                })).await.unwrap().into_inner());
            }
        }
        let pinned = pinned.unwrap();
        let tail = GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest { allow_delta: false,
            expected_epoch: pinned.epoch.clone(), offset_points: 10, max_points: 100,
        })).await.unwrap().into_inner();
        assert_eq!(tail.epoch, pinned.epoch);
        assert_eq!(tail.total_points, pinned.total_points);
        assert_eq!(tail.xyz_f32_le, expected_tail);
        let exhausted = GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest { allow_delta: false,
            expected_epoch: pinned.epoch.clone(), offset_points: pinned.total_points, max_points: 10,
        })).await.unwrap_err();
        assert!(exhausted.message().contains("DENSE_CLOUD_SOURCE_CHANGED"));
        let latest = GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest { allow_delta: false,
            expected_epoch: String::new(), offset_points: 0, max_points: 10,
        })).await.unwrap().into_inner();
        assert_ne!(latest.epoch, pinned.epoch);
        let delta = GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest {
            allow_delta: true, expected_epoch: pinned.epoch.clone(), offset_points: pinned.total_points, max_points: 100,
        })).await.unwrap().into_inner();
        assert_eq!(delta.base_epoch, pinned.epoch);
        assert_eq!(delta.epoch, latest.epoch);
        assert_eq!(delta.base_offset_points, pinned.total_points);
        assert!(!delta.removed_ranges.is_empty());
        let removed: u64 = delta.removed_ranges.chunks_exact(2).map(|range| range[1]).sum();
        assert_eq!(delta.offset_points, pinned.total_points - removed);
        let expected = handle.cloud_map_writer.lock().as_ref().unwrap().committed_dense_cloud().unwrap().unwrap()
            .read(delta.offset_points, 100).unwrap();
        assert_eq!(delta.xyz_f32_le, expected);
        // 替换拥有方后不能用其前任的分块代际继续读取。
        *handle.cloud_map_writer.lock() = Some(CloudMapShmWriter::new_private_for_test().unwrap());
        assert!(GeometryRuntime::get_dense_cloud_chunk(&service, Request::new(DenseCloudChunkRequest { allow_delta: false,
            expected_epoch: pinned.epoch, offset_points: 10, max_points: 100,
        })).await.is_err());
        assert!(owner.task_owner.shutdown().await.producers_settled());
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn ray_backend_checkpoint_activation_preserves_continuation_and_receipts() {
        let (source_handle, _, source_cloud_path) =
            ray_runtime_with_cloud_writer("source").await;
        let mut source_owner = crate::lcm_runtime::attach_root_task_owner_for_tests(source_handle);
        let source_handle = source_owner.handle.clone();
        let source_service = GeometryRuntimeService::new(source_handle.clone());
        let source_session_id = "ray-checkpoint-source";
        let started =
            start_mapping_session(&source_service, source_session_id, "warehouse-ray").await;
        assert!(started.active);
        let seed_points = [
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.5, 0.5, 1.0),
            Point3::new(2.0, -0.5, 1.0),
        ];
        for sequence in 1_u64..=3 {
            let timestamp_ns = sequence * 1_000_000_000;
            let committed = crate::lcm_runtime::handoff_registered_lio_cloud(
                &source_handle,
                &seed_points,
                12,
                timestamp_ns,
                timestamp_ns,
                "map",
            )
            .await
            .expect("seed registered ray handoff");
            assert!(committed, "每帧点必须接纳，栅格发布另行判断");
            // 相邻输入间隔1秒，均超过当前在线构图周期，应从首帧开始有真实修订。
            assert!(source_handle.state.read().await.map.revision > 0);
        }
        let mut source_checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &source_service,
            Request::new(ExportMappingBackendCheckpointRequest::default()),
        )
        .await
        .expect("export source ray checkpoint")
        .into_inner();
        assert_eq!(source_checkpoint.backend_kind, "raytracing");
        let source_commit_id = source_checkpoint
            .occupancy_receipt
            .as_ref()
            .and_then(|receipt| receipt.commit_id.clone())
            .expect("source checkpoint occupancy commit id");

        let (restored_handle, _, restored_cloud_path) =
            ray_runtime_with_cloud_writer("restored").await;
        let mut restored_owner = crate::lcm_runtime::attach_root_task_owner_for_tests(restored_handle);
        let restored_handle = restored_owner.handle.clone();
        let restored_service = GeometryRuntimeService::new(restored_handle.clone());
        for (is_dense, original) in [(false, source_checkpoint.fine_map_artifact.clone()), (true, source_checkpoint.dense_cloud_artifact.clone())] {
            let Some(original) = original else { continue; };
            let copied = GraphArtifactRef { id: uuid::Uuid::new_v4().simple().to_string(), ..original.clone() };
            GeometryRuntime::transfer_graph_artifact(&restored_service, Request::new(GraphArtifactTransferRequest {
                action: 1, artifact: Some(copied.clone()), offset: 0, data: Vec::new(),
            })).await.unwrap();
            let mut offset = 0;
            while offset < original.size {
                let chunk = GeometryRuntime::transfer_graph_artifact(&source_service, Request::new(GraphArtifactTransferRequest {
                    action: 4, artifact: Some(original.clone()), offset, data: Vec::new(),
                })).await.unwrap().into_inner();
                assert!(!chunk.data.is_empty());
                let length = chunk.data.len() as u64;
                GeometryRuntime::transfer_graph_artifact(&restored_service, Request::new(GraphArtifactTransferRequest {
                    action: 2, artifact: Some(copied.clone()), offset, data: chunk.data,
                })).await.unwrap();
                offset += length;
            }
            let sealed = GeometryRuntime::transfer_graph_artifact(&restored_service, Request::new(GraphArtifactTransferRequest {
                action: 3, artifact: Some(copied.clone()), offset: 0, data: Vec::new(),
            })).await.unwrap().into_inner();
            assert!(sealed.sealed);
            GeometryRuntime::transfer_graph_artifact(&source_service, Request::new(GraphArtifactTransferRequest {
                action: 5, artifact: Some(original), offset: 0, data: Vec::new(),
            })).await.unwrap();
            if is_dense { source_checkpoint.dense_cloud_artifact = Some(copied); }
            else { source_checkpoint.fine_map_artifact = Some(copied); }
        }
        let restored_session_id = "ray-checkpoint-restored";
        let activated = GeometryRuntime::activate_map(
            &restored_service,
            Request::new(activation_request_with_backend_checkpoint(restored_session_id,"warehouse-ray",source_checkpoint.clone())),
        )
        .await
        .expect("activate exported ray checkpoint")
        .into_inner();
        assert!(activated.active);
        let activation_receipt = restored_handle
            .mapping_adapter
            .as_ref()
            .expect("restored mapping adapter")
            .lock()
            .latest_occupancy_commit_receipt()
            .expect("activation occupancy receipt");
        let activation_boot_nonce = activation_receipt.commit_id.boot_nonce.to_be_bytes();
        assert_ne!(
            source_commit_id.boot_nonce.as_slice(),
            activation_boot_nonce.as_slice(),
            "activation must allocate a fresh occupancy receipt domain"
        );

        {
            let mut adapter = restored_handle.mapping_adapter.as_ref().unwrap().lock();
            assert!(adapter.checkpoint_localization_required);
            // 本测试对照同一坐标系的检查点序列化；显式建立已对齐夹具。
            // 真实校正事务及重启定位由注册提交测试和完整进程场景覆盖。
            adapter.feed_pose_in_frame(0.0,0.0,0.0,0.0,"odom").unwrap();
            let mut aligned = adapter.map_odom_transform_snapshot().unwrap();
            aligned.correction_revision = 1;
            adapter.commit_relocalization_registration(aligned);
        }

        let continuation_points = [
            Point3::new(2.5, 0.0, 1.0),
            Point3::new(2.5, 0.5, 1.0),
            Point3::new(3.0, -0.5, 1.0),
        ];
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &source_handle,
            &continuation_points,
            12,
            4_000_000_000,
            4_000_000_000,
            "map",
        )
        .await
        .expect("source ray continuation"));

        let global_map_topic = LcmTopic::new("/global_map").expect("global-map topic");
        let global_map_type =
            LcmMessageType::new("sensor_msgs.PointCloud2").expect("PointCloud2 type");
        let mut restored_global_map_frames = restored_handle
            .transport
            .subscribe(global_map_topic, Some(global_map_type))
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &restored_handle,
            &continuation_points,
            12,
            4_000_000_000,
            4_000_000_000,
            "map",
        )
        .await
        .expect("restored ray continuation"));
        let restored_global_map_frame =
            tokio::time::timeout(Duration::from_secs(1), restored_global_map_frames.next())
                .await
                .expect("restored global-map frame timeout")
                .expect("restored global-map stream")
                .expect("restored global-map transport");

        let source_map = source_handle.state.read().await.map.clone();
        let restored_map = restored_handle.state.read().await.map.clone();
        // 恢复会话独立分配修订；比较实际几何和源时间，原点允许协议分辨率的浮点量化误差。
        assert_eq!(restored_map.data, source_map.data);
        assert_eq!((restored_map.width, restored_map.height), (source_map.width, source_map.height));
        assert_eq!(restored_map.resolution, source_map.resolution);
        assert_eq!(restored_map.frame_id, source_map.frame_id);
        assert_eq!(restored_map.timestamp_ns, source_map.timestamp_ns);
        for (restored, source) in [(restored_map.origin_x, source_map.origin_x), (restored_map.origin_y, source_map.origin_y)] {
            let tolerance = f64::from(f32::EPSILON) * source.abs().max(f64::from(source_map.resolution));
            assert!((restored - source).abs() <= tolerance, "恢复原点必须在协议浮点精度内一致");
        }
        let source_cloud = source_handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(CloudMapShmWriter::committed_snapshot)
            .expect("source continued cloud snapshot");
        let restored_cloud = restored_handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(CloudMapShmWriter::committed_snapshot)
            .expect("restored continued cloud snapshot");
        assert_eq!(restored_cloud.point_count, source_cloud.point_count);
        let mut restored_points=restored_cloud.xyz_f32_le.chunks_exact(12).collect::<Vec<_>>();
        let mut source_points=source_cloud.xyz_f32_le.chunks_exact(12).collect::<Vec<_>>();
        restored_points.sort_unstable();source_points.sort_unstable();
        assert_eq!(restored_points,source_points);

        let request = || Request::new(DenseCloudChunkRequest { allow_delta: false, expected_epoch: String::new(), offset_points: 0, max_points: 65536 });
        let source_dense = GeometryRuntime::get_dense_cloud_chunk(&source_service, request()).await.unwrap().into_inner();
        let restored_dense = GeometryRuntime::get_dense_cloud_chunk(&restored_service, request()).await.unwrap().into_inner();
        assert_ne!(source_dense.epoch, restored_dense.epoch);
        assert_eq!(source_dense.total_points, 12, "四帧扫描的重复观测均须保留");
        assert_eq!(source_dense.xyz_f32_le, restored_dense.xyz_f32_le);
        assert_eq!(restored_dense.total_points, 12);
        let error = GeometryRuntime::get_dense_cloud_chunk(&restored_service, Request::new(DenseCloudChunkRequest { allow_delta: false,
            expected_epoch: source_dense.epoch, offset_points: 9, max_points: 65536,
        })).await.unwrap_err();
        assert!(error.message().contains("DENSE_CLOUD_SOURCE_CHANGED"));

        let continued_occupancy_receipt = restored_handle
            .mapping_adapter
            .as_ref()
            .expect("restored mapping adapter")
            .lock()
            .latest_occupancy_commit_receipt()
            .expect("continued occupancy receipt");
        let continued_cloud_receipt = restored_handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(CloudMapShmWriter::last_cloud_receipt)
            .expect("continued cloud receipt");
        assert_eq!(
            continued_occupancy_receipt.commit_id,
            continued_cloud_receipt.commit_id
        );
        assert_eq!(
            continued_occupancy_receipt.session_generation,
            continued_cloud_receipt.session_generation
        );
        assert_eq!(
            continued_occupancy_receipt.occupancy_revision,
            continued_cloud_receipt.occupancy_revision
        );
        assert_eq!(
            continued_occupancy_receipt.source_timestamp_ns,
            continued_cloud_receipt.source_timestamp_ns
        );
        assert_eq!(
            continued_occupancy_receipt.commit_id.boot_nonce,
            activation_receipt.commit_id.boot_nonce
        );
        assert!(
            continued_occupancy_receipt.commit_id.sequence > activation_receipt.commit_id.sequence
        );
        assert_eq!(
            continued_cloud_receipt.cloud_revision,
            restored_cloud.revision
        );
        assert_eq!(
            continued_cloud_receipt.point_count,
            restored_cloud.point_count
        );
        assert_eq!(
            restored_handle.state.read().await.cloud_map_revision,
            continued_cloud_receipt.cloud_revision
        );

        let mut encoded = Vec::with_capacity(8 + restored_global_map_frame.payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&restored_global_map_frame.payload);
        let restored_global_map =
            schema::decode_point_cloud2(&encoded).expect("continued global-map PointCloud2");
        assert_eq!(
            u64::try_from(restored_global_map.header.seq).expect("positive cloud header sequence"),
            continued_cloud_receipt.cloud_revision
        );
        assert_eq!(restored_global_map.header.frame_id, "map");
        assert_eq!(restored_global_map.header.stamp_sec, 4);
        assert_eq!(restored_global_map.header.stamp_nsec, 0);
        assert_eq!(restored_global_map.data, restored_cloud.xyz_f32_le);

        let source_continued_checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &source_service,
            Request::new(ExportMappingBackendCheckpointRequest::default()),
        )
        .await
        .expect("export uninterrupted continued checkpoint")
        .into_inner();
        let restored_continued_checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &restored_service,
            Request::new(ExportMappingBackendCheckpointRequest::default()),
        )
        .await
        .expect("export restored continued checkpoint")
        .into_inner();
        assert_eq!(
            restored_continued_checkpoint.payload, source_continued_checkpoint.payload,
            "restored and uninterrupted ray workers must serialize identically"
        );

        GeometryRuntime::stop_mapping(
            &source_service,
            Request::new(StopMappingRequest {
                session_id: source_session_id.to_owned(),
                reason: "ray checkpoint continuation test".to_owned(),
            }),
        )
        .await
        .expect("stop source ray session");
        GeometryRuntime::stop_mapping(
            &restored_service,
            Request::new(StopMappingRequest {
                session_id: restored_session_id.to_owned(),
                reason: "ray checkpoint continuation test".to_owned(),
            }),
        )
        .await
        .expect("stop restored ray session");
        assert!(source_owner.task_owner.shutdown().await.producers_settled());
        assert!(restored_owner.task_owner.shutdown().await.producers_settled());
        let _ = std::fs::remove_file(source_cloud_path);
        let _ = std::fs::remove_file(restored_cloud_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn ray_backend_checkpoint_export_waits_for_compute_and_fences_handoff() {
        let (handle, service, cloud_path) = ray_runtime_with_cloud_writer("export-race").await;
        let session_id = "ray-checkpoint-export-race";
        let started =
            start_mapping_session(&service, session_id, "warehouse-ray-export-race").await;
        assert!(started.active);
        let seed_points = [Point3::new(1.0, 0.0, 1.0), Point3::new(1.5, 0.5, 1.0)];
        for sequence in 1_u64..=3 {
            let timestamp_ns = sequence * 1_000_000_000;
            let committed = crate::lcm_runtime::handoff_registered_lio_cloud(
                &handle,
                &seed_points,
                12,
                timestamp_ns,
                timestamp_ns,
                "map",
            )
            .await
            .expect("seed export-race ray state");
            assert!(committed, "每帧点必须接纳，栅格发布另行判断");
            assert!(handle.state.read().await.map.revision > 0);
        }
        let seeded_checkpoint =
            GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(ExportMappingBackendCheckpointRequest::default()))
                .await
                .expect("export seed ray checkpoint")
                .into_inner();
        let gate = crate::mapping_backend::MappingBackendTestGate::default();
        handle
            .mapping_adapter
            .as_ref()
            .expect("ray mapping adapter")
            .lock()
            .replace_worker_with_test_gate(gate.clone())
            .expect("install gated ray worker");
        let restore_receiver = handle
            .mapping_adapter
            .as_ref()
            .expect("ray mapping adapter")
            .lock()
            .backend_checkpoint_client()
            .expect("gated ray worker client")
            .restore_checkpoint(seeded_checkpoint.payload)
            .expect("submit gated ray restore");
        let restore = restore_receiver
            .await
            .expect("gated ray restore response")
            .expect("gated ray restore");
        restore.commit().expect("commit gated ray restore");
        let points = [Point3::new(2.0, 0.0, 1.0), Point3::new(2.5, 0.5, 1.0)];
        let handoff_task = {
            let handle = handle.clone();
            tokio::spawn(async move {
                crate::lcm_runtime::handoff_registered_lio_cloud(
                    &handle,
                    &points,
                    12,
                    4_000_000_000,
                    4_000_000_000,
                    "map",
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            while !gate.entered() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("ray worker entered in-flight compute");

        let export_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(ExportMappingBackendCheckpointRequest::default()))
                    .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let export_waiting = {
                    let adapter = handle
                        .mapping_adapter
                        .as_ref()
                        .expect("ray mapping adapter")
                        .lock();
                    adapter.checkpoint_export_pending_for_tests()
                };
                if export_waiting {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("checkpoint export barrier became active");
        assert!(!export_task.is_finished());
        assert!(
            !crate::lcm_runtime::handoff_registered_lio_cloud(
                &handle,
                &[Point3::new(3.0, 0.0, 1.0)],
                12,
                5_000_000_000,
                5_000_000_000,
                "map",
            )
            .await
            .expect("handoff while checkpoint export is pending"),
            "checkpoint barrier must coalesce a later handoff"
        );

        gate.release();
        assert!(tokio::time::timeout(Duration::from_secs(1), handoff_task)
            .await
            .expect("in-flight handoff timeout")
            .expect("in-flight handoff join")
            .expect("in-flight handoff result"));
        let checkpoint = tokio::time::timeout(Duration::from_secs(2), export_task)
            .await
            .expect("checkpoint export timeout")
            .expect("checkpoint export join")
            .expect("checkpoint export response")
            .into_inner();
        assert_eq!(checkpoint.backend_kind, "raytracing");
        assert_eq!(
            checkpoint
                .occupancy_receipt
                .as_ref()
                .expect("checkpoint occupancy receipt")
                .source_timestamp_ns,
            4_000_000_000
        );
        assert_eq!(
            checkpoint
                .cloud_receipt
                .as_ref()
                .expect("checkpoint cloud receipt")
                .source_timestamp_ns,
            4_000_000_000
        );

        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(4.0, 0.0, 1.0)],
            12,
            6_000_000_000,
            6_000_000_000,
            "map",
        )
        .await
        .expect("handoff after checkpoint export"));
        assert_eq!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("ray mapping adapter")
                .lock()
                .latest_occupancy_commit_receipt()
                .expect("post-export occupancy receipt")
                .source_timestamp_ns,
            6_000_000_000
        );

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: session_id.to_owned(),
                reason: "ray checkpoint export race test".to_owned(),
            }),
        )
        .await
        .expect("stop export-race ray session");
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn checkpoint_export_cancellation_finishes_owned_flush_before_releasing_lease() {
        let (handle, service, cloud_path) =
            ray_runtime_with_cloud_writer("export-cancellation").await;
        let session_id = "ray-checkpoint-export-cancellation";
        start_mapping_session(&service, session_id, "warehouse-ray-export-cancellation").await;
        let gate = crate::mapping_backend::MappingBackendTestGate::default();
        handle
            .mapping_adapter
            .as_ref()
            .expect("ray mapping adapter")
            .lock()
            .replace_worker_with_test_gate(gate.clone())
            .expect("install cancellation gate");
        let handoff_task = {
            let handle = handle.clone();
            tokio::spawn(async move {
                crate::lcm_runtime::handoff_registered_lio_cloud(
                    &handle,
                    &[Point3::new(1.0, 0.0, 1.0)],
                    12,
                    1_000_000_000,
                    1_000_000_000,
                    "map",
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            while !gate.entered() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("cancellation test compute entered");

        let export_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(ExportMappingBackendCheckpointRequest::default()))
                    .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle
                    .mapping_adapter
                    .as_ref()
                    .expect("ray mapping adapter")
                    .lock()
                    .checkpoint_export_pending_for_tests()
                {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("cancellation test export barrier became active");
        export_task.abort();
        assert!(export_task
            .await
            .expect_err("aborted export must not complete")
            .is_cancelled());
        {
            let adapter = handle
                .mapping_adapter
                .as_ref()
                .expect("ray mapping adapter")
                .lock();
            assert!(adapter.checkpoint_export_pending_for_tests());
            assert!(adapter
                .backend_checkpoint_client()
                .expect_err("owned export must retain the live compute fence")
                .contains("compute to drain"));
        }
        assert!(!handoff_task.is_finished());

        gate.release();
        assert!(tokio::time::timeout(Duration::from_secs(1), handoff_task)
            .await
            .expect("cancelled-export handoff timeout")
            .expect("cancelled-export handoff join")
            .expect("cancelled-export handoff result"));
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if !handle
                    .mapping_adapter
                    .as_ref()
                    .expect("ray mapping adapter")
                    .lock()
                    .checkpoint_export_pending_for_tests()
                {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("owned checkpoint export must finish after compute drains");
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("ray mapping adapter")
            .lock()
            .backend_checkpoint_client()
            .is_ok());

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: session_id.to_owned(),
                reason: "ray checkpoint cancellation test".to_owned(),
            }),
        )
        .await
        .expect("stop cancellation ray session");
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn relocalization_drain_commits_in_flight_and_releases_cancelled_barriers() {
        for ending in ["admitted", "cancelled", "timeout"] {
            let (handle, service, cloud_path) = ray_runtime_with_cloud_writer(ending).await;
            start_mapping_session(&service, ending, ending).await;
            let adapter = handle.mapping_adapter.as_ref().unwrap().clone();
            let gate = crate::mapping_backend::MappingBackendTestGate::default();
            adapter.lock().replace_worker_with_test_gate(gate.clone()).unwrap();
            let handoff = {
                let handle = handle.clone();
                tokio::spawn(async move {
                    crate::lcm_runtime::handoff_registered_lio_cloud(
                        &handle, &[Point3::new(1.0, 0.0, 1.0), Point3::new(1.5, 0.5, 1.0)], 12,
                        1_000_000_000, 1_000_000_000, "map",
                    ).await
                })
            };
            tokio::time::timeout(Duration::from_secs(1), async {
                while !gate.entered() { tokio::task::yield_now().await; }
            }).await.unwrap();
            let drain = {
                let service = service.clone();
                tokio::spawn(async move { service.drain_mapping_before_relocalization().await })
            };
            tokio::time::timeout(Duration::from_secs(1), async {
                while !adapter.lock().checkpoint_export_pending_for_tests() {
                    tokio::task::yield_now().await;
                }
            }).await.unwrap();
            assert!(!drain.is_finished());
            // 排空时提交端仍能取得生命周期锁，不能把等待变成死锁。
            drop(tokio::time::timeout(Duration::from_millis(200), handle.mapping_lifecycle_lock.lock())
                .await.unwrap());
            match ending {
                "cancelled" => {
                    drain.abort();
                    assert!(drain.await.err().unwrap().is_cancelled());
                    assert!(!adapter.lock().checkpoint_export_pending_for_tests());
                    gate.release();
                }
                "timeout" => {
                    let result = drain.await.unwrap();
                    assert_eq!(result.err().unwrap().code(), tonic::Code::DeadlineExceeded);
                    assert!(!adapter.lock().checkpoint_export_pending_for_tests());
                    gate.release();
                }
                _ => {
                    gate.release();
                    let (lifecycle, barrier) = drain.await.unwrap().unwrap();
                    assert!(adapter.lock().backend_checkpoint_client().is_ok());
                    assert!(handle.mapping_lifecycle_lock.try_lock().is_err());
                    assert!(adapter.lock().checkpoint_export_pending_for_tests());
                    // 生产在此将同一锁交给原命令准入，模式切换后释放屏障。
                    drop(barrier);
                    drop(lifecycle);
                }
            }
            assert!(handoff.await.unwrap().unwrap());
            for sequence in 2..=3 {
                assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
                    &handle, &[Point3::new(1.0, 0.0, 1.0), Point3::new(1.5, 0.5, 1.0)], 12,
                    sequence * 1_000_000_000, sequence * 1_000_000_000, "map",
                ).await.unwrap(), "{ending}: 同一后端必须继续接纳下一批次");
            }
            assert_eq!(adapter.lock().latest_occupancy_commit_receipt().unwrap().source_timestamp_ns,
                3_000_000_000);
            GeometryRuntime::stop_mapping(&service, Request::new(StopMappingRequest {
                session_id: ending.into(), reason: "重定位排空验证结束".into(),
            })).await.unwrap();
            let _ = std::fs::remove_file(cloud_path);
        }
    }

    #[tokio::test]
    async fn corrupted_backend_checkpoint_fails_before_activation() {
        let (mut checkpoint,store) = export_voxel_backend_checkpoint().await;
        checkpoint.payload.push(0);
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let mut service = GeometryRuntimeService::new(handle.clone());
        service.graph_artifacts=store.clone();
        let mut request = activation_request("mapload:corrupt", "warehouse-corrupt");
        request.mapping_backend_checkpoint = Some(checkpoint);
        let error = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect_err("corrupt backend checkpoint must fail");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert!(error.message().contains("payload_size mismatch"));
        assert!(!handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());
        assert_eq!(handle.state.read().await.map.revision, 0);
    }

    #[tokio::test]
    async fn invalid_inner_backend_checkpoint_leaves_same_session_id_retryable() {
        let fixture=CanonicalResetFixture::new().await;
        let outcome=futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let valid=cloud_activation_request([1.0_f32,2.0,3.0,4.0,5.0,6.0].into_iter().flat_map(f32::to_le_bytes).collect());
            let mut invalid=valid.clone();invalid.mapping_backend_checkpoint.as_mut().unwrap().payload[8..12].copy_from_slice(&2_u32.to_le_bytes());
            for _ in 0..2 {
                let error=GeometryRuntime::activate_map(&fixture.service,Request::new(invalid.clone())).await.unwrap_err();
                assert_eq!(error.code(),tonic::Code::InvalidArgument);
                assert!(error.message().contains("backend payload format/schema"));
                assert!(!fixture.handle.mapping_adapter.as_ref().unwrap().lock().snapshot().active);
                assert_eq!(fixture.handle.state.read().await.map.revision,0);
            }
            let activated=GeometryRuntime::activate_map(&fixture.service,Request::new(valid)).await.unwrap().into_inner();
            assert!(activated.success && activated.active);
        })).await;
        fixture.finish().await;if let Err(panic)=outcome{std::panic::resume_unwind(panic);}
    }

    #[tokio::test]
    async fn ray_activation_without_backend_checkpoint_fails_before_mutation() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle.mapping_adapter = Some(Arc::new(parking_lot::Mutex::new(ray_mapping_adapter())));
        let service = GeometryRuntimeService::new(handle.clone());
        let error = GeometryRuntime::activate_map(
            &service,
            Request::new(ActivateMapRequest { mapping_backend_checkpoint:None, ..activation_request("mapload:ray", "warehouse-ray") }),
        )
        .await
        .expect_err("ray activation without backend state must fail");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert!(error.message().contains("mapping_backend_checkpoint is required"),"{error}");
        assert!(!handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());
        assert_eq!(handle.state.read().await.map.revision, 0);
    }

    #[tokio::test]
    async fn mixed_activation_identity_fields_reject_before_any_mutation() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activation-mixed-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        let epoch = service.activation_receipts.lifecycle_epoch();
        let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        let mut request = cloud_activation_request(xyz);
        request.activation_contract_version = 1;
        request.activation_request_id = "act-mixed".to_owned();
        request.expected_runtime_generation = 1;

        let error = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect_err("mixed activation identity must reject");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert_eq!(error.message(), "ACTIVATE_MAP_V2_IDENTITY_FIELDS_MIXED_OR_STALE");
        assert_eq!(service.activation_receipts.lifecycle_epoch(), epoch);
        assert!(!handle.mapping_adapter.as_ref().unwrap().lock().is_active());
        assert_eq!(handle.state.read().await.map.revision, 0);
        let _ = std::fs::remove_file(cloud_path);
    }


    #[tokio::test]
    async fn modern_activation_receipt_seals_replays_and_retires_on_first_stop() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activation-modern-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request =
            modern_activation_request(cloud_activation_request(xyz), "act-modern-1", 1);
        let replay_request = request.clone();

        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("modern activation")
            .into_inner();
        assert!(activated.success);
        assert_eq!(activated.activation_receipt_schema_version, 2);
        assert_eq!(activated.runtime_generation, 1);
        assert!(activated.backend_generation > 0);
        assert_eq!(activated.activation_request_id, "act-modern-1");
        assert!(!activated.relocalization_target_available);
        assert!(activated.activation_receipt_v2.as_ref().unwrap().target.is_some());
        assert_eq!(
            activated.activation_receipt_disposition,
            ActivationReceiptDisposition::Sealed as i32
        );

        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .unwrap()
            .into_inner();
        assert!(!queried.relocalization_target_available);
        assert!(queried.activation_receipt_v2.as_ref().unwrap().target.is_some());


        let replay = GeometryRuntime::activate_map(&service, Request::new(replay_request.clone()))
            .await
            .unwrap()
            .into_inner();
        assert!(replay.success);
        assert!(replay.idempotent_replay);
        assert_eq!(
            replay.activation_receipt_disposition,
            ActivationReceiptDisposition::SealedReplay as i32
        );

        let epoch = service.activation_receipts.lifecycle_epoch();
        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "mapload:one".to_owned(),
                reason: "test".to_owned(),
            }),
        )
        .await
        .unwrap()
        .into_inner();
        assert!(!stopped.relocalization_target_available);
        assert_eq!(
            stopped.activation_receipt_disposition,
            ActivationReceiptDisposition::Retired as i32
        );
        assert_eq!(service.activation_receipts.lifecycle_epoch(), epoch + 1);

        let duplicate = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "mapload:one".to_owned(),
                reason: "duplicate".to_owned(),
            }),
        )
        .await
        .unwrap()
        .into_inner();
        assert!(duplicate.idempotent_replay);
        assert_eq!(service.activation_receipts.lifecycle_epoch(), epoch + 1);

        let retired = GeometryRuntime::activate_map(&service, Request::new(replay_request))
            .await
            .unwrap()
            .into_inner();
        assert!(!retired.success);
        assert!(retired.idempotent_replay);
        assert_eq!(
            retired.activation_receipt_disposition,
            ActivationReceiptDisposition::Retired as i32
        );
        let _ = std::fs::remove_file(cloud_path);
    }


    #[tokio::test]
    async fn stop_mapping_settles_detached_backend_when_phase_a_wins_epoch_race() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-stop-phase-a-race-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
        let phase_a = crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let (shutdown, _listener) = shutdown_channel_inner(None, Some(phase_a), None);
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        GeometryRuntime::activate_map(&service, Request::new(cloud_activation_request(xyz)))
            .await
            .unwrap();
        let epoch = service.activation_receipts.lifecycle_epoch();
        service.inject_stop_phase_a_after_detach();

        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "mapload:one".to_owned(),
                reason: "phase-a-race".to_owned(),
            }),
        )
        .await
        .expect("StopMapping must settle detached backend")
        .into_inner();
        assert!(!stopped.active);
        assert_eq!(service.activation_receipts.lifecycle_epoch(), epoch + 1);

        let restarted = start_mapping_session(&service, "after-phase-a", "warehouse-next").await;
        assert!(
            restarted.active,
            "pending backend retirement must be cleared"
        );
        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "after-phase-a".to_owned(),
                reason: "cleanup".to_owned(),
            }),
        )
        .await
        .unwrap();
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn activation_fault_windows_are_typed_retired_and_backend_settled() {
        use ActivationFaultPoint;

        let cases = [
            (
                ActivationFaultPoint::SessionStart,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationFaultPoint::OccupancyShmBegin,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationFaultPoint::OccupancyInstall,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationFaultPoint::CloudInstall,
                ActivationReceiptDisposition::OutcomeUnknown,
            ),
            (
                ActivationFaultPoint::ActivationRecord,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationFaultPoint::SharedStateCommit,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationFaultPoint::CacheClears,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationFaultPoint::MapEvent,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationFaultPoint::CloudEvent,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationFaultPoint::TypedGlobalMap,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
        ];

        for (index, (step, expected_disposition)) in cases.into_iter().enumerate() {
            let handle = crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let cloud_path = std::env::temp_dir().join(format!(
                "nuwax-activation-fault-{index}-{}-{}.mmap",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            ));
            *handle.cloud_map_writer.lock() =
                Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
            let service = GeometryRuntimeService::new(handle.clone());
            let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
                .iter()
                .flat_map(|point| point.iter())
                .flat_map(|value| value.to_le_bytes())
                .collect::<Vec<_>>();
            let request = modern_activation_request(
                cloud_activation_request(xyz),
                &format!("act-fault-{index}"),
                1,
            );
            let replay_request = request.clone();
            service.inject_activation_fault_after(step);

            let terminal = GeometryRuntime::activate_map(&service, Request::new(request))
                .await
                .expect("injected transaction returns typed terminal")
                .into_inner();
            assert!(!terminal.success, "step={step:?}");
            assert_eq!(
                terminal.activation_receipt_disposition, expected_disposition as i32,
                "step={step:?}"
            );
            assert!(!handle.mapping_adapter.as_ref().unwrap().lock().is_active());
            assert!(
                handle
                    .mapping_lifecycle_lock
                    .clone()
                    .try_lock_owned()
                    .is_ok(),
                "lifecycle leaked for step={step:?}"
            );

            let replay = GeometryRuntime::activate_map(&service, Request::new(replay_request))
                .await
                .unwrap()
                .into_inner();
            assert!(!replay.success);
            assert_eq!(
                replay.activation_receipt_disposition,
                ActivationReceiptDisposition::Retired as i32
            );
            let _ = std::fs::remove_file(cloud_path);
        }
    }

    #[tokio::test]
    async fn activation_response_loss_after_seal_replays_without_side_effects() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activation-seal-loss-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle);
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request = modern_activation_request(cloud_activation_request(xyz), "act-seal-loss", 1);
        service.inject_activation_fault_after(
            ActivationFaultPoint::Seal,
        );
        let error = GeometryRuntime::activate_map(&service, Request::new(request.clone()))
            .await
            .expect_err("injected response loss after seal");
        assert_eq!(error.code(), tonic::Code::Unavailable);

        let replay = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .unwrap()
            .into_inner();
        assert!(replay.success);
        assert!(replay.idempotent_replay);
        assert_eq!(
            replay.activation_receipt_disposition,
            ActivationReceiptDisposition::SealedReplay as i32
        );
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn backend_restore_and_commit_fault_windows_fail_closed() {
        let (checkpoint,store) = export_voxel_backend_checkpoint().await;
        let cases = [
            (
                ActivationFaultPoint::BackendRestoreSubmission,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationFaultPoint::BackendCommit,
                ActivationReceiptDisposition::OutcomeUnknown,
            ),
        ];
        for (index, (step, expected_disposition)) in cases.into_iter().enumerate() {
            let handle = crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let cloud_path = std::env::temp_dir().join(format!(
                "nuwax-backend-activation-fault-{index}-{}-{}.mmap",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            ));
            *handle.cloud_map_writer.lock() =
                Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
            let mut root_owner=crate::lcm_runtime::attach_root_task_owner_for_tests(handle);
        let handle=root_owner.handle.clone();
        let mut service = GeometryRuntimeService::new(handle.clone());
        service.graph_artifacts=store.clone();
            service.inject_activation_fault_after(step);
            let request = activation_request_with_backend_checkpoint(
                &format!("backend-fault-{index}"),
                "warehouse-backend-fault",
                checkpoint.clone(),
            );
            let terminal = GeometryRuntime::activate_map(&service, Request::new(request))
                .await
                .expect("backend fault returns typed terminal")
                .into_inner();
            assert!(!terminal.success);
            assert_eq!(
                terminal.activation_receipt_disposition, expected_disposition as i32,
                "step={step:?}"
            );
            assert!(!handle.mapping_adapter.as_ref().unwrap().lock().is_active());
            assert!(handle
                .mapping_lifecycle_lock
                .clone()
                .try_lock_owned()
                .is_ok());
            assert!(root_owner.task_owner.shutdown().await.producers_settled());
        let _ = std::fs::remove_file(cloud_path);
        }
    }

    #[tokio::test]
    async fn activate_map_commits_checkpoint_and_start_mapping_reuses_session() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activate-cloud-test-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() = Some(
            crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("test cloud-map writer"),
        );
        let service = GeometryRuntimeService::new(handle.clone());
        let mut revision_rx = handle.map_revision_tx.subscribe();
        let mut cloud_revision_rx = handle.cloud_map_tx.subscribe();
        let global_map_topic = LcmTopic::new("/global_map").expect("global-map topic");
        let global_map_type =
            LcmMessageType::new("sensor_msgs.PointCloud2").expect("PointCloud2 type");
        let mut global_map_frames = handle
            .transport
            .subscribe(global_map_topic.clone(), Some(global_map_type.clone()))
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request = cloud_activation_request(cloud_xyz.clone());
        let replay_request = request.clone();

        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("ActivateMap")
            .into_inner();

        assert!(activated.success);
        assert!(activated.active);
        assert_eq!(activated.phase, "ACTIVE");
        assert_eq!(activated.map_name, "warehouse");
        assert!(activated.revision > 0);
        let event = revision_rx.recv().await.expect("checkpoint revision event");
        assert_eq!(event.session_id, "mapload:one");
        assert_eq!(event.map_name, "warehouse");
        assert_eq!(event.data.as_ref(), &[u8::MAX, 0, 100, 50]);
        let cloud_event = cloud_revision_rx
            .recv()
            .await
            .expect("cloud checkpoint revision event");
        assert_eq!(cloud_event.point_count, 2);
        assert_eq!(cloud_event.session_generation, activated.session_generation);
        let shared = handle.state.read().await;
        assert_eq!(shared.map.data, vec![-1, 0, 100, 50]);
        assert_eq!(shared.cloud_map_revision, cloud_event.revision);
        drop(shared);

        let first_frame = tokio::time::timeout(Duration::from_secs(1), global_map_frames.next())
            .await
            .expect("first activated global-map frame")
            .expect("first activated global-map stream")
            .expect("first activated global-map transport");
        let mut encoded = Vec::with_capacity(8 + first_frame.payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&first_frame.payload);
        let first_cloud = schema::decode_point_cloud2(&encoded).expect("activated PointCloud2");
        assert_eq!(first_cloud.header.seq, cloud_event.revision as i32);
        assert_eq!(first_cloud.header.frame_id, "map");
        assert_eq!(first_cloud.header.stamp_sec, 1_700_000_000);
        assert_eq!(first_cloud.header.stamp_nsec, 0);
        let canonical_cloud_xyz = handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(|writer| writer.committed_snapshot())
            .expect("canonical checkpoint SHM snapshot")
            .xyz_f32_le;
        assert_eq!(first_cloud.data, canonical_cloud_xyz);

        let mut late_global_map_frames = handle
            .transport
            .subscribe(global_map_topic, Some(global_map_type))
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let divergent_cloud_xyz = [[8.025_f32, 0.025, 0.025], [9.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        assert_ne!(divergent_cloud_xyz, cloud_xyz);
        let divergent = GeometryRuntime::activate_map(
            &service,
            Request::new(cloud_activation_request(divergent_cloud_xyz)),
        )
        .await
        .expect("conflicting activation returns a typed terminal")
        .into_inner();
        assert!(!divergent.success);
        assert!(divergent.idempotent_replay);
        assert_eq!(
            divergent.activation_receipt_disposition,
            ActivationReceiptDisposition::Conflict as i32
        );
        let replay = GeometryRuntime::activate_map(&service, Request::new(replay_request))
            .await
            .expect("exact ActivateMap replay")
            .into_inner();
        assert!(replay.idempotent_replay);
        assert_eq!(
            replay.activation_receipt_disposition,
            ActivationReceiptDisposition::SealedReplay as i32
        );
        assert!(
            tokio::time::timeout(Duration::from_millis(200), late_global_map_frames.next())
                .await
                .is_err(),
            "sealed replay must not republish /global_map"
        );

        let start_replay = start_mapping_session(&service, "mapload:one", "warehouse").await;
        assert!(start_replay.idempotent_replay);
        assert_eq!(start_replay.revision, activated.revision);
        assert_eq!(handle.state.read().await.map.data, vec![-1, 0, 100, 50]);
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn activate_map_after_runtime_restart_republishes_checkpoint() {
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request = cloud_activation_request(cloud_xyz.clone());

        for runtime_generation in 1..=2 {
            let mut handle = crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let cloud_path = std::env::temp_dir().join(format!(
                "nuwax-activate-cloud-restart-test-{}-{}-{}.mmap",
                std::process::id(),
                runtime_generation,
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            ));
            *handle.cloud_map_writer.lock() = Some(
                crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path)
                    .expect("test cloud-map writer"),
            );
            let backend = Arc::new(CheckpointPublishBackend::default());
            handle.transport = LcmTransport::new(backend.clone());
            let service = GeometryRuntimeService::new(handle.clone());

            GeometryRuntime::activate_map(&service, Request::new(request.clone()))
                .await
                .expect("checkpoint activation after runtime start");
            let revision = handle.state.read().await.cloud_map_revision;
            assert_eq!(revision, 1);
            let published = backend
                .published
                .lock()
                .expect("checkpoint published frames");
            let published: Vec<_> = published.iter()
            .filter(|frame| frame.msg_type.fqn() == "sensor_msgs.PointCloud2").collect();
        assert_eq!(published.len(), 1);
            let mut encoded = Vec::with_capacity(8 + published[0].payload.len());
            encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
            encoded.extend_from_slice(&published[0].payload);
            let cloud =
                schema::decode_point_cloud2(&encoded).expect("restart checkpoint PointCloud2");
            assert_eq!(cloud.header.seq, revision as i32);
            assert_eq!(cloud.header.frame_id, "map");
            assert_eq!(cloud.data, cloud_xyz);
            drop(published);
            let _ = std::fs::remove_file(cloud_path);
        }
    }

    #[tokio::test]
    async fn activate_map_publish_failure_releases_locks_and_retries_current_writer() {
        exercise_failed_activation_publication(false).await;
    }

    #[tokio::test]
    async fn activation_publish_failure_followed_by_mapping_change_retires_old_request() {
        exercise_failed_activation_publication(true).await;
    }

    async fn exercise_failed_activation_publication(replace_session: bool) {
        let fixture=CanonicalResetFixture::new().await;
        let outcome=futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let mut handle=fixture.handle.clone();
            let backend=Arc::new(CheckpointPublishBackend::default());
            backend.fail_after_observe.store(true,Ordering::Release);
            handle.transport=LcmTransport::new(backend.clone());
            let service=GeometryRuntimeService::new(handle.clone());
            let request=cloud_activation_request([[0.025_f32,0.025,0.025],[1.025,0.025,0.025]].into_iter()
                .flatten().flat_map(f32::to_le_bytes).collect());
            let error=GeometryRuntime::activate_map(&service,Request::new(request.clone())).await.unwrap_err();
            assert_eq!(error.code(),tonic::Code::Unavailable);
            assert!(error.message().contains("publication failed"));
            assert!(handle.mapping_lifecycle_lock.try_lock().is_ok(),"发布失败必须释放生命周期锁");
            let projection=tokio::time::timeout(Duration::from_millis(100),handle.lock_explore_projection_owned()).await.unwrap();
            drop(projection);
            let adapter=handle.mapping_adapter.as_ref().unwrap();
            let installed=adapter.lock().snapshot();
            assert!(installed.active);
            let before=handle.cloud_map_writer.lock().as_ref().unwrap().committed_metadata().unwrap();
            backend.fail_after_observe.store(false,Ordering::Release);
            if replace_session {
                GeometryRuntime::stop_mapping(&service,Request::new(StopMappingRequest{session_id:installed.session_id.clone(),reason:"换图".into()})).await.unwrap();
                let next=start_mapping_session(&service,"next-session","next-map").await;
                assert!(next.active && next.session_generation>installed.session_generation);
                let replay=GeometryRuntime::activate_map(&service,Request::new(request)).await.unwrap().into_inner();
                assert!(!replay.success);
                assert_eq!(replay.activation_receipt_disposition,ActivationReceiptDisposition::Retired as i32);
                assert!(adapter.lock().snapshot().active);
                assert_eq!(adapter.lock().snapshot().session_id,next.session_id,"旧重试不得撤销新会话");
                assert_eq!(adapter.lock().snapshot().session_generation,next.session_generation);
            } else {
                let current={
                    let _lifecycle=handle.mapping_lifecycle_lock.lock().await;
                    let occupancy=request.mapping_backend_checkpoint.as_ref().unwrap().canonical_occupancy_checkpoint.as_ref().unwrap();
                    let grid=decode_checkpoint_grid_borrowed(occupancy,"map").unwrap();
                    let floor=handle.state.read().await.map.revision;
                    let (map,_event)=adapter.lock().install_checkpoint(grid,occupancy.meta.as_ref().unwrap().timestamp_ns+1,floor).unwrap();
                    let receipt=adapter.lock().latest_occupancy_commit_receipt().unwrap();
                    let current={
                        let mut writer=handle.cloud_map_writer.lock();let writer=writer.as_mut().unwrap();
                        assert!(writer.accumulate_mapping_frame(installed.session_generation,&[Point3::new(12.025,0.025,0.025)]));
                        writer.commit_accumulated_snapshot(&receipt).unwrap().unwrap().snapshot
                    };
                    let mut shared=handle.state.write().await;shared.map=map;shared.cloud_map_revision=current.revision;
                    current
                };
                assert!(current.revision>before.revision);
                handle.state.write().await.cloud_map_revision=current.revision;
                let replay=GeometryRuntime::activate_map(&service,Request::new(request.clone())).await.unwrap().into_inner();
                assert!(replay.success && replay.idempotent_replay);
                let state=handle.cloud_map_writer.lock().as_ref().unwrap().typed_publish_state();
                assert_eq!(state.published_identity(),Some((current.session_generation,current.revision)));
                let observed=backend.published.lock().unwrap().len();
                let sealed=GeometryRuntime::activate_map(&service,Request::new(request)).await.unwrap().into_inner();
                assert!(sealed.success && sealed.idempotent_replay);
                assert_eq!(backend.published.lock().unwrap().len(),observed,"封存重放不重复发布");
            }
        })).await;
        fixture.finish().await;
        if let Err(panic)=outcome {std::panic::resume_unwind(panic);}
    }

    #[tokio::test]
    async fn stop_waits_until_the_listener_is_closed_before_reporting_success() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown);
        let shutdown_paths = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let listener_task = {
            let shutdown_paths = Arc::clone(&shutdown_paths);
            tokio::spawn(async move {
                listener.wait_for_request().await;
                shutdown_paths.fetch_add(1, std::sync::atomic::Ordering::AcqRel);
                listener.mark_listener_closed();
            })
        };

        let first = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        let second = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        let (first, second) = tokio::join!(first, second);
        let first = first.expect("first Stop join").expect("first Stop RPC");
        let second = second.expect("second Stop join").expect("second Stop RPC");

        assert!(first.into_inner().success);
        assert!(second.into_inner().success);
        listener_task.await.expect("listener acknowledgement task");
        assert_eq!(
            shutdown_paths.load(std::sync::atomic::Ordering::Acquire),
            1,
            "concurrent Stops must share one listener shutdown path"
        );
        assert_eq!(service.state.read().await.phase, "stopped");

        let repeated = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect("repeated Stop RPC")
            .into_inner();
        assert!(repeated.success, "Stop must remain idempotent");
    }

    #[tokio::test]
    async fn production_concurrent_stops_share_the_first_correction_transition() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let probe = Arc::new(PhaseALioSink::default());
        let lio_gate = Arc::new(crate::lio_ingress::LioIngressRoutingGate::new(probe));
        let correction_phase_a =
            crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let failure_probe = correction_phase_a.clone();
        let (shutdown, mut listener) =
            crate::server::production_shutdown_channel(lio_gate, correction_phase_a);
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown);
        let listener_task = tokio::spawn(async move {
            listener.wait_for_request().await;
            listener.mark_listener_closed();
        });
        let first = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        let second = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        let (first, second) = tokio::join!(first, second);
        assert!(
            first
                .expect("first Stop join")
                .expect("first Stop response")
                .into_inner()
                .success
        );
        assert!(
            second
                .expect("second Stop join")
                .expect("second Stop response")
                .into_inner()
                .success
        );
        listener_task.await.expect("listener ACK task");
        assert!(
            GeometryRuntime::stop(&service, Request::new(StopRequest {}))
                .await
                .expect("repeated Stop response")
                .into_inner()
                .success
        );
        assert!(
            tokio::time::timeout(
                Duration::from_millis(25),
                failure_probe.wait_process_stop_failed()
            )
            .await
            .is_err(),
            "duplicates cannot fail or relabel the first successful Stop"
        );
    }

    #[tokio::test]
    async fn cancelled_process_stop_before_listener_ack_reports_failure() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let probe = Arc::new(PhaseALioSink::default());
        let lio_gate = Arc::new(crate::lio_ingress::LioIngressRoutingGate::new(probe));
        let correction_phase_a =
            crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let failure_probe = correction_phase_a.clone();
        let (shutdown, mut listener) =
            crate::server::production_shutdown_channel(lio_gate, correction_phase_a);
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown);
        let stop_service = service.clone();
        let stop_task = tokio::spawn(async move {
            GeometryRuntime::stop(&stop_service, Request::new(StopRequest {})).await
        });
        tokio::time::timeout(Duration::from_millis(100), listener.wait_for_request())
            .await
            .expect("Stop reached listener-close wait");
        stop_task.abort();
        assert!(stop_task
            .await
            .expect_err("aborted Stop task")
            .is_cancelled());
        tokio::time::timeout(
            Duration::from_millis(100),
            failure_probe.wait_process_stop_failed(),
        )
        .await
        .expect("cancelled Stop publishes failure through its RAII attempt");
        assert!(service
            .shutdown
            .as_ref()
            .expect("production shutdown coordinator")
            .has_shutdown_failed());
        let duplicate_error = tokio::time::timeout(
            Duration::from_millis(100),
            GeometryRuntime::stop(&service, Request::new(StopRequest {})),
        )
        .await
        .expect("duplicate Stop failure replay deadline")
        .expect_err("duplicate Stop must replay first-owner failure");
        assert_eq!(duplicate_error.code(), tonic::Code::FailedPrecondition);
        assert!(duplicate_error
            .message()
            .contains("first Process Stop attempt failed"));
        let (_request_rx, _listener_closed_tx, graceful_rx) = listener.into_parts();
        assert!(!*graceful_rx.borrow());
    }

    #[tokio::test]
    async fn listener_ack_timeout_does_not_enable_graceful_shutdown() {
        let (shutdown, listener) = crate::server::shutdown_channel();
        let (request_rx, listener_closed_tx, graceful_rx) = listener.into_parts();

        let error = shutdown
            .request_and_wait_for_listener()
            .await
            .expect_err("missing listener ACK must fail");

        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(*request_rx.borrow());
        assert!(!*listener_closed_tx.borrow());
        assert!(!*graceful_rx.borrow());
        assert!(shutdown.has_shutdown_failed());
        tokio::time::timeout(
            Duration::from_millis(40),
            shutdown.wait_for_readiness_revocation(),
        )
        .await
        .expect("listener ACK failure must revoke readiness");
    }

    #[tokio::test]
    async fn listener_ack_does_not_publish_graceful_before_explicit_release() {
        let (shutdown, listener) = crate::server::shutdown_channel();
        let (mut request_rx, listener_closed_tx, graceful_rx) = listener.into_parts();
        let listener_task = tokio::spawn(async move {
            if !*request_rx.borrow() {
                request_rx
                    .wait_for(|requested| *requested)
                    .await
                    .expect("listener request channel");
            }
            listener_closed_tx.send_replace(true);
        });

        shutdown
            .request_and_wait_for_listener()
            .await
            .expect("listener close acknowledgement");
        listener_task.await.expect("listener task");
        assert!(shutdown.is_shutdown_requested());
        assert!(
            !*graceful_rx.borrow(),
            "listener ACK alone cannot publish graceful shutdown"
        );

        shutdown.publish_graceful_shutdown();
        assert!(*graceful_rx.borrow());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn ready_action_and_stopping_phase_are_linearly_ordered() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);
        let entered = Arc::new(std::sync::Barrier::new(2));
        let release = Arc::new(std::sync::Barrier::new(2));
        let ready_service = service.clone();
        let ready_entered = entered.clone();
        let ready_release = release.clone();
        let ready_task = tokio::spawn(async move {
            ready_service
                .run_if_running(move || {
                    ready_entered.wait();
                    ready_release.wait();
                })
                .await
        });
        tokio::task::spawn_blocking(move || {
            entered.wait();
        })
        .await
        .expect("ready entry join");

        let state = service.state.clone();
        let mut stopping = Box::pin(state.write());
        assert!(matches!(
            futures::poll!(stopping.as_mut()),
            std::task::Poll::Pending
        ));

        tokio::task::spawn_blocking(move || {
            release.wait();
        })
        .await
        .expect("ready release join");
        ready_task
            .await
            .expect("ready action join")
            .expect("running phase");
        let mut state = stopping.await;
        state.phase = "stopping".to_string();
        drop(state);
        let error = service
            .run_if_running(|| panic!("stopping phase must reject ready action"))
            .await
            .expect_err("stopping phase must reject ready action");
        assert_eq!(error, "stopping");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn ready_acquire_and_revocation_are_linearly_ordered() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, _listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown.clone());
        let ready_path = std::env::temp_dir().join(format!(
            "nuwax-ready-linearization-{}-{}",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let entered = Arc::new(std::sync::Barrier::new(2));
        let release = Arc::new(std::sync::Barrier::new(2));
        let acquire_service = service.clone();
        let acquire_path = ready_path.clone();
        let acquire_entered = entered.clone();
        let acquire_release = release.clone();
        let acquire_task = tokio::spawn(async move {
            acquire_service
                .run_if_running(move || {
                    acquire_entered.wait();
                    acquire_release.wait();
                    std::fs::write(&acquire_path, b"50051")
                })
                .await
        });
        tokio::task::spawn_blocking(move || entered.wait())
            .await
            .expect("ready acquisition entered shared authority");

        let revoke_task = tokio::task::spawn_blocking(move || shutdown.revoke_readiness());
        tokio::task::yield_now().await;
        assert!(
            !revoke_task.is_finished(),
            "revocation must serialize behind an acquisition already in its authority section"
        );
        tokio::task::spawn_blocking(move || release.wait())
            .await
            .expect("release ready acquisition");
        acquire_task
            .await
            .expect("ready acquisition task")
            .expect("ready authority active")
            .expect("ready file write");
        revoke_task.await.expect("readiness revocation task");
        assert!(ready_path.exists());
        std::fs::remove_file(&ready_path).expect("remove first linearized ready lease");

        let error = service
            .run_if_running(|| {
                std::fs::write(&ready_path, b"must-not-run")
                    .expect("revoked ready action must never execute")
            })
            .await
            .expect_err("revocation visible before acquire must reject the action");
        assert_eq!(error, "readiness_revoked");
        assert!(!ready_path.exists());
    }

    #[test]
    fn deferred_cloud_map_clear_requires_configured_voxel_identity() {
        let generation = 7;
        let exact = CloudMapNotification {
            revision: 1,
            point_count: 0,
            voxel_size: crate::pointcloud::VOXEL_SIZE,
            session_generation: generation,
        };
        validate_deferred_cloud_map_clear(
            &CloudMapSessionAdmission::Registered(exact.clone()),
            generation,
        )
        .expect("configured voxel identity");

        let positive_mismatch = CloudMapNotification {
            voxel_size: f32::from_bits(crate::pointcloud::VOXEL_SIZE.to_bits() + 1),
            ..exact
        };
        assert!(validate_deferred_cloud_map_clear(
            &CloudMapSessionAdmission::Registered(positive_mismatch),
            generation,
        )
        .is_err());
    }

    #[tokio::test]
    async fn nondeferred_start_mapping_writer_unavailable_fails_closed() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("Geometry test handle");
        *handle.cloud_map_writer.lock() = None;
        let service = GeometryRuntimeService::new(handle.clone());
        let error = GeometryRuntime::start_mapping(
            &service,
            Request::new(StartMappingRequest {
                session_id: "writer-unavailable".to_owned(),
                map_name: "writer-unavailable-map".to_owned(),
            }),
        )
        .await
        .expect_err("WriterUnavailable must reject non-deferred StartMapping");
        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(error.message().contains("writer is unavailable"));
        assert!(
            !handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot()
                .active,
            "WriterUnavailable rollback must leave Mapping inactive"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn phase_a_rejects_nondeferred_start_mapping_cloud_clear_registration() {
        let mut spawned = crate::lcm_runtime::attach_root_task_owner_for_tests(
            crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("Geometry test handle"),
        );
        let handle = spawned.handle.clone();
        let task_control = handle.task_control().expect("Geometry task control");
        let (shutdown, _listener) = shutdown_channel_inner(None, None, Some(task_control));
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());
        let barrier = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_start_mapping_before_cloud_clear_barrier(barrier.clone());
        let call_service = service.clone();
        let call = tokio::spawn(async move {
            GeometryRuntime::start_mapping(
                &call_service,
                Request::new(StartMappingRequest {
                    session_id: "phase-a-start".to_owned(),
                    map_name: "phase-a-map".to_owned(),
                }),
            )
            .await
        });

        barrier.wait().await;
        shutdown.begin_phase_a();
        barrier.wait().await;
        let error = call
            .await
            .expect("StartMapping task join")
            .expect_err("Phase A must reject cloud clear registration");
        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(error.message().contains("task admission is closed"));
        assert!(
            !handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot()
                .active
        );

        let outcome = spawned.task_owner.shutdown().await;
        assert!(outcome.producers_settled());
        if let Some(writer) = &handle.occupancy_shm {
            writer
                .lock()
                .cleanup_product_owned_file()
                .expect("occupancy mmap cleanup");
        }
    }


    #[tokio::test]
    async fn process_stop_holds_lifecycle_lock_until_listener_ack() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let lifecycle_lock = handle.mapping_lifecycle_lock.clone();
        let probe = Arc::new(PhaseALioSink::default());
        let active = Arc::new(PhaseALioSink::default());
        let lio_gate = Arc::new(crate::lio_ingress::LioIngressRoutingGate::new(probe));
        lio_gate
            .bind_and_open(active.clone())
            .expect("active production LIO route");
        let correction_phase_a =
            crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let (shutdown, mut listener) = crate::server::production_shutdown_channel(
            lio_gate.clone(),
            correction_phase_a.clone(),
        );
        let graceful_rx = listener.graceful_rx.clone();
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown);
        let mapping_transition = service.mapping_transition.clone();
        let before_graceful = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_stop_before_graceful_barrier(before_graceful.clone());
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_ack_release = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            let listener_ack_release = listener_ack_release.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener_ack_release.notified().await;
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_millis(100), async {
            loop {
                if service.state.read().await.phase == "stopping" {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("Stop phase must linearize before the lifecycle lock wait");
        tokio::time::timeout(
            Duration::from_millis(40),
            readiness.wait_for_readiness_revocation(),
        )
        .await
        .expect("Stop must revoke readiness before the lifecycle lock wait");
        assert!(!lio_gate.is_open());
        assert!(active.closed.load(std::sync::atomic::Ordering::Acquire));
        assert!(!correction_phase_a.is_open());
        assert_eq!(
            service
                .run_if_running(|| panic!("READY action must be closed by Stop"))
                .await
                .expect_err("Stop must close READY admission"),
            "stopping"
        );

        tokio::time::timeout(Duration::from_secs(1), listener_requested.notified())
            .await
            .expect("Stop reached listener-close ACK boundary");
        assert!(
            lifecycle_lock.try_lock().is_err(),
            "Process Stop must retain the lifecycle lock until listener ACK"
        );
        assert!(!*graceful_rx.borrow());
        assert!(!stop_task.is_finished());
        listener_ack_release.notify_one();
        before_graceful.wait().await;
        assert!(
            lifecycle_lock.try_lock().is_ok(),
            "response construction must release the lifecycle guard before graceful publication"
        );
        assert!(
            mapping_transition.try_lock().is_ok(),
            "response construction must release the transition guard before graceful publication"
        );
        assert!(
            !*graceful_rx.borrow(),
            "graceful must remain unpublished throughout the released-guard interval"
        );
        assert!(!stop_task.is_finished());
        before_graceful.wait().await;
        let response = stop_task
            .await
            .expect("process Stop task")
            .expect("process Stop response")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");
        assert_eq!(service.state.read().await.phase, "stopped");
        assert!(
            lifecycle_lock.try_lock().is_ok(),
            "Stop response releases the lifecycle lock for graceful drain"
        );
        assert!(*graceful_rx.borrow());
    }

    #[tokio::test]
    async fn process_stop_leaves_mapping_and_lio_teardown_to_main() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (_detached_lio_control, lio_request_rx, _lio_completion_tx) =
            crate::lio_owner::LioShutdownControl::test_channel();
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        start_mapping_session(&service, "process-stop-lio", "process-stop-lio-map").await;
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stopped = tokio::time::timeout(
            Duration::from_secs(1),
            GeometryRuntime::stop(&service, Request::new(StopRequest {})),
        )
        .await
        .expect("Stop completion timeout")
        .expect("Stop response")
        .into_inner();
        assert!(stopped.success);
        tokio::time::timeout(Duration::from_secs(1), listener_requested.notified())
            .await
            .expect("listener close ACK precedes Stop response");
        listener_task.await.expect("listener acknowledgement task");
        assert!(
            !*lio_request_rx.borrow(),
            "RPC Stop must not request LIO teardown"
        );
        assert!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot()
                .active,
            "RPC Stop must leave Mapping active for main-owned Phase B"
        );
        assert_eq!(service.state.read().await.phase, "stopped");
        assert!(handle.mapping_lifecycle_lock.try_lock().is_ok());
    }

    #[tokio::test]
    async fn process_stop_bounds_lifecycle_lock_without_requesting_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;

        let error = tokio::time::timeout(
            Duration::from_secs(1),
            GeometryRuntime::stop(&service, Request::new(StopRequest {})),
        )
        .await
        .expect("Stop lifecycle-lock deadline")
        .expect_err("held lifecycle lock must fail Stop");

        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(error.message().contains("mapping lifecycle fence"));
        assert_eq!(service.state.read().await.phase, "failed");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "a failed Stop must not request listener shutdown"
        );
    }
}

#[cfg(test)]
mod exploration_contract_tests {
    use super::*;

    #[test]
    fn start_request_parses_boundary_and_stop_conditions() {
        let request = ExploreRequest {
            return_policy: None,
            resume_progress_json: String::new(),
            strategy: "wavefront".into(),
            arbitration: "prefer_wavefront".into(),
            request_id: "explore-1".into(),
            boundary_line_segments: vec![
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 0.0,
                    end_x: 2.0,
                    end_y: 0.0,
                },
                ExplorationBoundarySegment {
                    start_x: 2.0,
                    start_y: 0.0,
                    end_x: 0.0,
                    end_y: 2.0,
                },
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 2.0,
                    end_x: 0.0,
                    end_y: 0.0,
                },
            ],
            stop_conditions: Some(ExplorationStopConditions {
                max_coverage_pct: Some(85.0),
                no_frontier_timeout_ms: Some(4_000),
                max_duration_ms: Some(60_000),
                max_goals: Some(12),
            }),
            max_navigation_retries: 3,
            blacklist_radius_m: 0.75,
            boundary_frame_id: "map".into(),

            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
        };

        let config = explore_config_from_request(&request, "map").expect("valid explore config");
        assert_eq!(config.max_navigation_retries, 3);
        assert_eq!(config.blacklist_radius_m, 0.75);
        assert_eq!(
            config.motion_admission,
            crate::nav_runtime::test_motion_admission()
        );
        assert!(config.boundary.is_some());
        assert_eq!(config.stop_conditions.max_goals, Some(12));
        assert_eq!(config.stop_conditions.no_frontier_timeout_ms, Some(4_000));
    }

    #[test]
    fn start_request_uses_configured_robot_map_boundary_frame() {
        let mut request = ExploreRequest {
            boundary_line_segments: vec![
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 0.0,
                    end_x: 2.0,
                    end_y: 0.0,
                },
                ExplorationBoundarySegment {
                    start_x: 2.0,
                    start_y: 0.0,
                    end_x: 0.0,
                    end_y: 2.0,
                },
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 2.0,
                    end_x: 0.0,
                    end_y: 0.0,
                },
            ],
            boundary_frame_id: "robot/map".to_string(),

            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
            ..ExploreRequest::default()
        };

        assert!(explore_config_from_request(&request, "robot/map").is_ok());
        request.boundary_frame_id = "/robot/map".to_string();
        let slash_error = explore_config_from_request(&request, "robot/map")
            .expect_err("slash-prefixed frame must not bypass raw-exact boundary identity");
        assert!(slash_error.contains("robot/map"));
        request.boundary_frame_id = "map".to_string();
        let error = explore_config_from_request(&request, "robot/map")
            .expect_err("literal map must not bypass the configured frame");
        assert!(error.contains("robot/map"));
    }

    #[test]
    fn exploration_state_preserves_child_identity() {
        let proto =
            exploration_state_from_snapshot(crate::explore_runtime::ExploreRuntimeSnapshot {
            parking_pending: false,
                mission_progress_json: String::new(),
                status: crate::explore_runtime::STATUS_ACTIVE,
                message: "moving".into(),
                request_id: "parent".into(),
                selected_source: "wavefront".into(),
                last_stop_reason: String::new(),
                frontier_count: 2,
                revision: 8,
                child_task_id: "child-3".into(),
                exploration_generation: 12,
                child_lease: Some(crate::explore_runtime::ChildNavigationLease {
                    session_id: "child-3".into(),
                    owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                    parent_request_id: "parent".into(),
                    goal_generation: 4,
                    producer_runtime_generation: 5,

                }),


            });
        assert_eq!(proto.child_task_id, "child-3");
        assert_eq!(proto.exploration_generation, 12);
        let child = proto.child_lease.expect("child lease");
        assert_eq!(child.producer_runtime_generation, 5);
    }
}

#[cfg(test)]
mod navigation_feedback_contract_tests {
    use super::*;
    use crate::lcm_runtime::{NavigationFeedback, NavigationUpdate};

    #[test]
    fn navigation_proto_projection_preserves_typed_feedback() {
        let proto = nav_state_to_proto(NavigationUpdate {
            state: crate::lcm_runtime::NavState {
                status: "CANCELLED".into(),
                message: "motion guard zeroed".into(),
                goal_id: "goal-7".into(),
                distance_remaining: 0.0,
                timestamp_ns: 99,
            },
            producer_runtime_generation: 41,
            feedback: NavigationFeedback {
                planned_path: vec![[1.0,2.0,0.4],[2.0,2.0,0.4]],
                local_path: vec![[1.0,2.0,0.4],[1.4,2.1,0.5]],
                cancel_phase: "ZEROED".into(),
                cancel_generation: 3,
                actuator_zeroed: true,
                reason_code: "CANCEL_ZEROED".into(),
                runtime_generation: 7,

                sink_ready: true,
                planner_map_frame: "map".into(),
                planner_pose_frame: "map".into(),
                zero_ack_id: "zero-3".into(),
                zero_applied_monotonic_ns: 123,
                owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                parent_request_id: "request-7".into(),
                goal_generation: 5,
                path_revision: 2,
                producer_runtime_generation: 7,
                planner_runtime_generation: 8,
            },
        });

        assert_eq!(proto.cancel_phase, "ZEROED");
        assert_eq!(proto.cancel_generation, 3);
        assert!(proto.actuator_zeroed);
        assert_eq!(proto.reason_code, "CANCEL_ZEROED");
        assert_eq!(proto.runtime_generation, 7);
        assert!(proto.sink_ready);
        assert_eq!(proto.planner_map_frame, "map");
        assert_eq!(proto.planner_pose_frame, "map");
        assert_eq!(proto.zero_ack_id, "zero-3");
        assert_eq!(proto.zero_applied_monotonic_ns, 123);
        assert_eq!(
            proto.owner_kind,
            nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );
        assert_eq!(proto.parent_request_id, "request-7");
        assert_eq!(proto.goal_generation, 5);
        assert_eq!(proto.path_revision, 2);
        assert_eq!(proto.planner_runtime_generation, 8);
        assert_eq!(proto.producer_runtime_generation, 41);
        assert_eq!(proto.planned_path.len(),2);
        assert_eq!(proto.planned_path[1].z,0.4);
        assert!(proto.local_path.is_empty());
        let running = nav_state_to_proto(NavigationUpdate {
            state: crate::lcm_runtime::NavState { status: "RUNNING".into(), ..Default::default() },
            feedback: NavigationFeedback { local_path: vec![[1.0,2.0,0.4],[1.4,2.1,0.5]], ..Default::default() },
            producer_runtime_generation: 41,
        });
        assert_eq!(running.local_path.len(),2);
        assert_eq!(running.local_path[1].z,0.5);
    }

    #[tokio::test]
    async fn deprecated_sink_rpc_cannot_create_ready_without_generation() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle.set_cmd_vel_sink_ready(false);
        let service = GeometryRuntimeService::new(handle.clone());

        let response = GeometryRuntime::set_cmd_vel_sink_ready(
            &service,
            Request::new(CmdVelSinkReadyRequest {
                ready: true,
                generation: 0,
            }),
        )
        .await
        .expect("deprecated RPC response")
        .into_inner();

        assert!(!response.success);
        assert!(!handle.cmd_vel_sink_ready());
    }

    #[tokio::test]
    async fn public_goal_empty_frame_uses_adapter_configured_robot_map() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        *handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock() = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "robot/map",
            "odom",
        );
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "robot/map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "robot/map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "robot/map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("robot/map TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "robot/map".to_string(),
                pose_frame: "robot/map".to_string(),
                ready: true,
            })
            .expect("robot/map planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        install_server_test_committed_map_in_frame(&handle, 1, "robot/map");
        let service = GeometryRuntimeService::new(handle);

        let goal = GeometryRuntime::set_navigation_goal(
            &service,
            Request::new(NavigationGoal {
                pose: Some(Pose {
                    orientation: Some(Quat {
                        w: 1.0,
                        ..Quat::default()
                    }),
                    timestamp_ns: now_ns(),
                    ..Pose::default()
                }),
                frame_id: String::new(),
                goal_id: "robot-map-default-goal".to_string(),

                motion_guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 1,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect("configured default goal")
        .into_inner();
        assert_eq!(goal.status, "ACCEPTED", "{}", goal.message);
    }

    #[tokio::test]
    async fn public_goal_normalizes_once_and_cancel_preserves_complete_lease() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        install_server_test_committed_map(&handle, 1);
        let service = GeometryRuntimeService::new(handle);
        let goal = GeometryRuntime::set_navigation_goal(
            &service,
            Request::new(NavigationGoal {
                pose: Some(Pose {
                    orientation: Some(Quat {
                        w: 1.0,
                        ..Quat::default()
                    }),
                    frame_id: "map".to_string(),
                    timestamp_ns: now_ns(),
                    ..Pose::default()
                }),
                frame_id: "map".to_string(),
                goal_id: "public-goal".to_string(),

                motion_guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 1,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect("public goal")
        .into_inner();
        assert_eq!(goal.status, "ACCEPTED");
        assert_eq!(
            goal.owner_kind,
            nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );
        assert_eq!(goal.parent_request_id, "public-goal");
        assert!(goal.goal_generation > 0);
        assert!(goal.producer_runtime_generation > 0);
        assert!(goal.producer_runtime_generation > 0);

        let wrong_producer_runtime_generation = if goal.producer_runtime_generation == 1 {
            2
        } else {
            goal.producer_runtime_generation - 1
        };
        let stale_cancel = GeometryRuntime::cancel_navigation(
            &service,
            Request::new(CancelNavigationRequest {
global_stop_confirmation: None,

                goal_id: goal.goal_id.clone(),
                owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(),
                goal_generation: goal.goal_generation,
                producer_runtime_generation: wrong_producer_runtime_generation,
            }),
        )
        .await
        .expect("stale producer cancel response")
        .into_inner();
        assert_eq!(stale_cancel.status, "FAILED");
        assert!(stale_cancel.message.contains("GOAL_LEASE_MISMATCH"));
        assert_eq!(service.bridge.nav.cancel_snapshot().generation, 0);

        let cancelled = GeometryRuntime::cancel_navigation(
            &service,
            Request::new(CancelNavigationRequest {
global_stop_confirmation: None,

                goal_id: goal.goal_id.clone(),
                owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(),
                goal_generation: goal.goal_generation,
                producer_runtime_generation: goal.producer_runtime_generation,
            }),
        )
        .await
        .expect("correlated cancel")
        .into_inner();
        assert_eq!(cancelled.status, "PENDING");
        assert_eq!(cancelled.owner_kind, goal.owner_kind);
        assert_eq!(cancelled.parent_request_id, goal.parent_request_id);
        assert_eq!(cancelled.goal_generation, goal.goal_generation);
        assert_eq!(
            cancelled.producer_runtime_generation,
            goal.producer_runtime_generation
        );
        assert_eq!(cancelled.producer_runtime_generation, goal.producer_runtime_generation);

        let rejected = GeometryRuntime::set_navigation_goal(
            &service,
            Request::new(NavigationGoal {
                owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect_err("public RPC cannot forge exploration owner");
        assert_eq!(rejected.code(), tonic::Code::InvalidArgument);
    }
}

#[cfg(test)]
mod relocalization_terminal_query_tests {
    use super::*;

    async fn query_service(
        ingress: crate::correction_coordinator::RelocalizationCommandIngress,
    ) -> GeometryRuntimeService {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let phase_a = crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let (shutdown, _listener) = shutdown_channel_inner(None, Some(phase_a), None);
        GeometryRuntimeService::new_with_shutdown_and_point_cloud_binding(
            handle,
            shutdown,
            None,
            None,
            None,
            None,
            Some(ingress),
        )
    }

    fn query_request(request_id:&str, changed:bool) -> GetRelocalizationTerminalRequest {
        let mut request=crate::relocalization_terminal_v2::planar_tests::wire(); request.request_id=request_id.into();
        if changed { let Some(relocalize_request::AssignedTarget::AssignedPlanarV2(body))=request.assigned_target.as_mut() else{unreachable!()}; body.seed.as_mut().unwrap().x=9.0; }
        GetRelocalizationTerminalRequest{schema_version:2,request_id:request_id.into(),assigned_request_v2:Some(request)}
    }

    fn query_root(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "nuwax-terminal-query-{name}-{}-{}",
            std::process::id(),
            now_ns()
        ))
    }

    #[tokio::test]
    async fn terminal_query_rejects_invalid_id_and_identity_lengths() {
        let root=query_root("invalid");
        let ingress=crate::correction_coordinator::RelocalizationCommandIngress::terminal_query_fixture_for_tests(root.clone());
        let service=query_service(ingress).await;
        let mut missing=query_request("req-missing",false);missing.assigned_request_v2=None;
        let mut old=query_request("req-old",false);old.schema_version=1;
        for request in [query_request(" invalid",false),missing,old] {
            assert_eq!(GeometryRuntime::get_relocalization_terminal(&service,Request::new(request)).await.unwrap_err().code(),tonic::Code::InvalidArgument);
        }
        std::fs::remove_dir_all(root).unwrap();
    }

    #[tokio::test]
    async fn terminal_query_projects_four_read_only_states_without_mutating_runtime() {
        let root=query_root("states");
        let ingress=crate::correction_coordinator::RelocalizationCommandIngress::terminal_query_fixture_for_tests(root.clone());
        ingress.seed_terminal_query_for_tests(query_request("req-pending",false).assigned_request_v2.unwrap(),None);
        let terminal_request=query_request("req-terminal",false).assigned_request_v2.unwrap();
        let mut result=crate::relocalization_terminal_v2::planar_tests::terminal(false);
        result.request_id=terminal_request.request_id.clone();result.assigned_planar_v2.as_mut().unwrap().request=Some(terminal_request.clone());
        ingress.seed_terminal_query_for_tests(terminal_request,Some(result));
        let service=query_service(ingress.clone()).await; let before=service.bridge.state.read().await.map.revision;
        for (id,changed,status,has_terminal) in [("req-terminal",false,RelocalizationTerminalStatus::Terminal,true),
            ("req-pending",false,RelocalizationTerminalStatus::Pending,false),("req-missing",false,RelocalizationTerminalStatus::NotFound,false),
            ("req-terminal",true,RelocalizationTerminalStatus::IdentityConflict,false)] {
            let response=GeometryRuntime::get_relocalization_terminal(&service,Request::new(query_request(id,changed))).await.unwrap().into_inner();
            assert_eq!(response.status,status as i32);assert_eq!(response.terminal.is_some(),has_terminal);
            assert_eq!(response.ledger_observation.unwrap().admitted_command_count,0);
        }
        assert_eq!(service.bridge.state.read().await.map.revision,before);
        let query=crate::relocalization_terminal_v2::AssignedRelocalizationTerminalQueryV2::admit(query_request("req-pending",false)).unwrap();
        assert!(matches!(ingress.terminal_snapshot_assigned(&query).await.unwrap().terminal,crate::correction_coordinator::RelocalizationTerminalSnapshot::Pending));
        std::fs::remove_dir_all(root).unwrap();
    }
}

#[cfg(test)]
mod assigned_checkpoint_assembly_tests {
use super::*;
pub(super) fn frozen() -> MappingBackendCheckpoint {
    let id=MappingCommitIdentity {boot_nonce:vec![1;16],sequence:5};
    let occupancy=CanonicalOccupancyCommitReceipt { commit_id:Some(id.clone()),session_id:"source".into(),session_generation:2,
        occupancy_revision:7,source_timestamp_ns:100,backend_kind:"voxel".into(),backend_generation:3 };
    let cloud=CanonicalCloudCommitReceipt {commit_id:Some(id),session_generation:2,occupancy_revision:7,cloud_revision:9,source_timestamp_ns:100,point_count:1};
    let payload=[b"NWMBCKP\0".as_slice(), &1u32.to_le_bytes(), &[0,0,0,0], &[0;40]].concat();
    MappingBackendCheckpoint {schema_version:0,backend_kind:"voxel".into(),source_session_generation:2,created_at_ns:101,
        source_session_id:"source".into(),payload_size:payload.len() as u64,payload,checkpoint_id:"11111111111111111111111111111111".into(),
        occupancy_receipt:Some(occupancy),cloud_receipt:Some(cloud),
        canonical_occupancy_checkpoint:Some(OccupancyGridData {meta:Some(OccupancyGridMeta{width:1,height:1,resolution:0.05,
            origin_x:0.0,origin_y:0.0,frame_id:"map".into(),revision:7,timestamp_ns:100}),data:vec![0]}),
        canonical_cloud_checkpoint:Some(CloudMapCheckpoint{frame_id:"map".into(),source_revision:9,point_count:1,voxel_size:0.05,
            xyz_f32_le:[1f32,2.,3.].into_iter().flat_map(f32::to_le_bytes).collect(),..Default::default()}),..Default::default()}
}
#[test]
fn assigned_v2_native_wire_and_frozen_assembly_keep_exact_ids_bytes_and_payload_schema() {
    let request=ExportMappingBackendCheckpointRequest{};
    assert_eq!(ExportMappingBackendCheckpointRequest::decode(request.encode_to_vec().as_slice()).unwrap(),request);
    let original=frozen();let checkpoint=assemble_mapping_checkpoint_export(original.clone()).unwrap();
    assert_eq!(checkpoint.schema_version,2);assert_eq!(checkpoint.backend_payload_schema_version,1);
    assert_eq!(checkpoint.checkpoint_id,original.checkpoint_id);assert_eq!(checkpoint.payload,original.payload);
    let cloud=checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
    assert_eq!(cloud.xyz_f32_le,original.canonical_cloud_checkpoint.unwrap().xyz_f32_le);assert_eq!(cloud.payload_schema_version,2);
    let encoded=checkpoint.encode_to_vec();assert_eq!(MappingBackendCheckpoint::decode(encoded.as_slice()).unwrap(),checkpoint);
    validate_assigned_mapping_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
}
#[test]
fn assigned_v2_backend_and_occupancy_members_require_exact_parent_binding() {
    let valid = assemble_mapping_checkpoint_export(frozen()).unwrap();
    for backend in [true, false] {
        for mutation in 0..7 {
            let mut bad = valid.clone();
            let field = if backend { &mut bad.backend_identity } else { &mut bad.occupancy_identity };
            if mutation == 0 { *field = None; } else {
                let member = field.as_mut().unwrap();
                match mutation {
                    1 => member.role = CheckpointMemberRole::CanonicalCloud as i32,
                    2 => member.checkpoint_id = "b".repeat(32),
                    3 => member.frame_id = "odom".into(),
                    4 => member.producer_generation = 8,
                    5 => member.occupancy_receipt.as_mut().unwrap().occupancy_revision += 1,
                    _ => member.cloud_receipt.as_mut().unwrap().cloud_revision += 1,
                }
            }
            assert!(validate_assigned_mapping_checkpoint(&bad, crate::mapping_backend::MappingBackendKind::Voxel).is_err(), "backend={backend} mutation={mutation}");
        }
    }
}
#[test]
fn assigned_v2_assembly_rejects_mixed_fields_source_geometry_and_backend_format() {
    let valid=assemble_mapping_checkpoint_export(frozen()).unwrap();
    for field in [1,2,3,4,5,7,8,9,10,11,12] {
        let mut bad=valid.clone();match field {
            1=>bad.checkpoint_id="A".repeat(32),2=>bad.backend_payload_schema_version=2,
            3=>bad.payload_size+=1,4=>bad.payload[8]=2,5=>bad.payload[12]=1,
            7=>bad.canonical_cloud_checkpoint.as_mut().unwrap().member_identity.as_mut().unwrap().checkpoint_id="2".repeat(32),
            8=>bad.canonical_cloud_checkpoint.as_mut().unwrap().xyz_f32_le[..4].copy_from_slice(&f32::NAN.to_le_bytes()),
            9=>bad.canonical_occupancy_checkpoint.as_mut().unwrap().data[0]=254,
            10=>bad.canonical_occupancy_checkpoint.as_mut().unwrap().meta.as_mut().unwrap().origin_x=f64::NAN,
            11=>bad.cloud_receipt.as_mut().unwrap().cloud_revision+=1,_=>bad.canonical_cloud_checkpoint.as_mut().unwrap().frame_id="odom".into() }
        assert!(validate_assigned_mapping_checkpoint(&bad,crate::mapping_backend::MappingBackendKind::Voxel).is_err(),"field {field}");
    }
}
#[test]
fn assigned_v2_assembly_checks_full_artifact_parent_descriptor() {
    let mut checkpoint=assemble_mapping_checkpoint_export(frozen()).unwrap();
    let member=checkpoint_member_identity(&checkpoint.checkpoint_id,CheckpointMemberRole::DenseCloud,checkpoint.occupancy_receipt.as_ref().unwrap(),checkpoint.cloud_receipt.as_ref().unwrap(),"map",None);
    checkpoint.dense_cloud_artifact=Some(GraphArtifactRef{id:"2".repeat(32),size:100,identity_schema_version:2,member_identity:Some(member),..Default::default()});
    validate_assigned_mapping_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
    let mut collision=checkpoint.clone();let mut fine=collision.dense_cloud_artifact.clone().unwrap();
    fine.member_identity.as_mut().unwrap().role=CheckpointMemberRole::FineMap as i32;fine.member_identity.as_mut().unwrap().producer_generation=6;
    collision.fine_map_artifact=Some(fine);
    assert!(validate_assigned_mapping_checkpoint(&collision,crate::mapping_backend::MappingBackendKind::Voxel).unwrap_err().contains("cannot share one artifact ID"));
    checkpoint.dense_cloud_artifact.as_mut().unwrap().member_identity.as_mut().unwrap().cloud_receipt.as_mut().unwrap().cloud_revision+=1;
    validate_assigned_mapping_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).expect_err("外部制品描述必须绑定原占据与点云收据");
}
}

#[cfg(test)]
mod assigned_activation_boundary_tests {
    use super::*;

    fn metadata_request() -> ActivateMapRequest {
        ActivateMapRequest {
            activation_contract_version: 2,
            activation_request_id: "assigned-boundary:one".into(),
            session_id: "loaded:one".into(), map_name: "warehouse".into(),
            expected_runtime_generation: 7,
            asset_identity: Some(crate::proto::AssetIdentityV2 { asset_id: "a".repeat(32) }),
            mapping_backend_checkpoint: Some(MappingBackendCheckpoint {
                schema_version: 2, checkpoint_id: "b".repeat(32), ..Default::default()
            }),
            ..Default::default()
        }
    }

    #[test]
    fn assigned_metadata_never_enters_legacy_request_classification() {
        let request = metadata_request();
        assert_eq!(classify_activation_request_identity(&request, 7).unwrap().activation_request_id(), "assigned-boundary:one");
        assert!(classify_activation_request_identity(&request, 8).is_err());
        for version in [0, 1, 3] {
            let mut skew = request.clone(); skew.activation_contract_version = version;
            assert!(classify_activation_request_identity(&skew, 7).is_err());
        }
    }


}

#[cfg(test)]
#[path = "owned_occupancy_bytes_tests.rs"]
mod owned_occupancy_bytes_tests;

#[cfg(test)]
mod planar_assigned_checkpoint_pair_tests {
    use super::*;
    #[test]
    fn planar_assigned_pair_checkpoint_binds_real_occupancy_without_cloud_or_hash() {
        let previous = crate::map_checksum::FORBID_HASH.with(|v|v.replace(true));
        let (checkpoint, _) = crate::planar_artifacts::planar_assigned_pair_tests::fixture();
        validate_planar_checkpoint(&checkpoint).unwrap();
        assert!(checkpoint.backend_identity.is_none()); assert!(checkpoint.cloud_receipt.is_none());
        for change in 0..5 {
            let mut bad=checkpoint.clone();
            match change {0=>bad.occupancy_receipt.as_mut().unwrap().occupancy_revision+=1,
                1=>bad.graph_artifact.as_mut().unwrap().member_identity.as_mut().unwrap().checkpoint_id="3".repeat(32),
                2=>bad.occupancy_identity.as_mut().unwrap().role=CheckpointMemberRole::PlanarGraph as i32,
                3=>bad.canonical_occupancy_checkpoint.as_mut().unwrap().data[0]=101,
                _=>bad.graph_artifact.as_mut().unwrap().size+=1}
            assert!(validate_planar_checkpoint(&bad).is_err(),"field {change}");
        }
        crate::map_checksum::FORBID_HASH.with(|v|v.set(previous));
    }
}


#[cfg(all(test, unix))]
mod shared_nav_v8_recovery_tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    struct UnusedJournalTransport;
    #[async_trait::async_trait]
    impl nuwax_transport::LcmBackend for UnusedJournalTransport {
        async fn publish(&self, _frame: RawLcmFrame) -> nuwax_transport::TransportResult<()> {
            panic!("journal GET/ACK/replay must not emit an LCM action")
        }
        async fn subscribe(&self, _topic: LcmTopic, _kind: Option<LcmMessageType>) -> nuwax_transport::LcmFrameStream {
            Box::pin(futures::stream::pending())
        }
        fn name(&self) -> &str { "journal-in-process-unit" }
    }
    pub(super) async fn unit_handle() -> (GeometryRuntimeHandle, crate::lcm_runtime::GeometryTestTaskOwner) {
        crate::lcm_runtime::build_handle_for_tests_with_unit_transport(
            true, None, nuwax_transport::LcmTransport::new(Arc::new(UnusedJournalTransport)),
        ).await.unwrap()
    }


}
