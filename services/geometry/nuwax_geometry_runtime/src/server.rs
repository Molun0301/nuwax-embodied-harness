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

use nuwax_codec::{lcm::schema, CodecError, CodecResult};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use prost::Message;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Duration;
use tokio::sync::{watch, RwLock};
use tonic::{Request, Response, Status};
use tracing::info;

use crate::lcm_runtime::{
    accept_mapping_cloud_map_for_backend_candidate, build_committed_global_map_frame_in_frame,
    publish_committed_global_map_in_frame, publish_retained_global_map_frame, CloudMapNotification,
    CloudMapSessionAdmission, CloudMapSessionRegistration, CloudMapSessionReservation,
    CommittedMappingCloudMap, GeometryRuntimeHandle, MapRevisionEvent, NavigationUpdate, PoseState,
};
use crate::mapping_adapter::{
    finish_backend_retirement, MappingAdapter, MappingBackendRetirement,
    MappingCheckpointExportLease, MappingSessionSnapshot,
};
use crate::nav_runtime::{
    cancel as nav_cancel, compute_readiness, ensure_goal_id, readiness_to_reason, set_goal,
    CancelOutcome, MotionAdmissionProof, SetGoalInput, SetGoalOutcome,
};
use crate::occupancy_shm::{OccupancyGridShmWriter, OccupancySessionFenceTransaction};
use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::runtime_diagnostics::{self, RuntimeDiagnostics, ServiceState, SharedServiceState};
use crate::typed_codec;
use nuwax_exploration::{BoundarySegment, ExplorationBoundary};
use nuwax_motion_safety_guard::{
    HeldRecoveryFenceProof, MotionGuardClient, ZeroConfirmation, DEFAULT_UDS_PATH,
};
use nuwax_nav_types_messages::geometry::Point2;

#[derive(Clone)]
pub struct GeometryRuntimeService {
    state: SharedServiceState,
    bridge: GeometryRuntimeHandle,
    base_frame_id: String,
    point_cloud_binding: Option<(LcmTopic, LcmMessageType)>,
    shutdown: Option<ShutdownCoordinator>,
    mapping_transition: Arc<tokio::sync::Mutex<()>>,
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
    activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl,
    relocalization_ingress: Option<crate::correction_coordinator::RelocalizationCommandIngress>,
    activation_lifecycle_fence: Arc<parking_lot::Mutex<Option<ActivationLifecycleFence>>>,
    #[cfg(test)]
    activation_fault_after:
        Arc<parking_lot::Mutex<Option<crate::correction_coordinator::ActivationJournalStep>>>,
    #[cfg(test)]
    stop_phase_a_after_detach: Arc<AtomicBool>,
    #[cfg(test)]
    fail_start_mapping_stop_transition: Arc<AtomicBool>,
    #[cfg(test)]
    start_mapping_before_cloud_clear: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    stop_before_graceful: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
}

struct ActivationLifecycleFence {
    activation_request_id: String,
    activation_request_identity_sha256: [u8; 32],
    retained_global_map: Option<RetainedActivationGlobalMap>,
    lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
}

struct RetainedActivationGlobalMap {
    activation_request_identity_sha256: [u8; 32],
    map_frame_id: String,
    installed_cloud_revision: u64,
    cloud_receipt: crate::mapping_commit::CloudCommitReceipt,
    frame: RawLcmFrame,
}

impl RetainedActivationGlobalMap {
    fn from_committed(
        activation_request_identity_sha256: [u8; 32],
        map_frame_id: &str,
        committed: &CommittedMappingCloudMap,
    ) -> Result<Self, String> {
        Ok(Self {
            activation_request_identity_sha256,
            map_frame_id: map_frame_id.to_owned(),
            installed_cloud_revision: committed.notification.revision,
            cloud_receipt: committed.receipt.clone(),
            frame: build_committed_global_map_frame_in_frame(committed, map_frame_id)?,
        })
    }

    fn validate_replay(
        &self,
        activation_request_identity_sha256: [u8; 32],
        map_frame_id: &str,
        committed: &CommittedMappingCloudMap,
    ) -> Result<(), String> {
        let expected =
            Self::from_committed(activation_request_identity_sha256, map_frame_id, committed)?;
        if self.activation_request_identity_sha256 != activation_request_identity_sha256
            || self.map_frame_id != map_frame_id
            || self.installed_cloud_revision != committed.notification.revision
            || self.cloud_receipt != committed.receipt
            || self.frame != expected.frame
        {
            return Err(
                "retained activation global-map publication identity does not match replay"
                    .to_owned(),
            );
        }
        Ok(())
    }
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

async fn await_start_mapping_step<T>(
    cancellation_rx: &mut watch::Receiver<bool>,
    step: impl std::future::Future<Output = T>,
) -> Option<T> {
    if *cancellation_rx.borrow() {
        return None;
    }
    tokio::select! {
        biased;
        _ = async {
            let _ = cancellation_rx.wait_for(|cancelled| *cancelled).await;
        } => None,
        result = step => Some(result),
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
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
    geometry_tasks: Option<crate::lcm_runtime::GeometryRuntimeTaskControl>,
    stop_single_flight: Arc<tokio::sync::Mutex<()>>,
    mapping_transition: Arc<tokio::sync::Mutex<()>>,
}

pub struct ShutdownListener {
    request_rx: watch::Receiver<bool>,
    listener_closed_tx: watch::Sender<bool>,
    graceful_rx: watch::Receiver<bool>,
}

#[cfg(test)]
pub(crate) fn shutdown_channel() -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(None, None, None, None)
}

#[cfg(test)]
pub(crate) fn production_shutdown_channel(
    lio_gate: Arc<crate::lio_ingress::LioIngressRoutingGate>,
    correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
) -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(
        Some(lio_gate),
        Some(correction_phase_a),
        deferred_replay,
        None,
    )
}

pub(crate) fn production_shutdown_channel_with_task_control(
    lio_gate: Arc<crate::lio_ingress::LioIngressRoutingGate>,
    correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
    geometry_tasks: crate::lcm_runtime::GeometryRuntimeTaskControl,
) -> (ShutdownCoordinator, ShutdownListener) {
    shutdown_channel_inner(
        Some(lio_gate),
        Some(correction_phase_a),
        deferred_replay,
        Some(geometry_tasks),
    )
}

fn shutdown_channel_inner(
    lio_gate: Option<Arc<crate::lio_ingress::LioIngressRoutingGate>>,
    correction_phase_a: Option<crate::correction_coordinator::CorrectionPhaseAControl>,
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
    geometry_tasks: Option<crate::lcm_runtime::GeometryRuntimeTaskControl>,
) -> (ShutdownCoordinator, ShutdownListener) {
    let (request_tx, request_rx) = watch::channel(false);
    let (listener_closed_tx, listener_closed_rx) = watch::channel(false);
    let (graceful_tx, graceful_rx) = watch::channel(false);
    let readiness = deferred_replay
        .as_ref()
        .map(|replay| replay.readiness_authority())
        .unwrap_or_default();
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
            deferred_replay,
            geometry_tasks,
            stop_single_flight: Arc::new(tokio::sync::Mutex::new(())),
            mapping_transition,
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
        if let Some(deferred_replay) = &self.deferred_replay {
            deferred_replay.begin_phase_a();
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
        if let Some(deferred_replay) = &self.deferred_replay {
            deferred_replay.begin_phase_a();
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
            || self
                .deferred_replay
                .as_ref()
                .is_some_and(|replay| replay.poison_detail().is_some())
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
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
            base_frame_id: configured_base_frame(),
            point_cloud_binding: Some(point_cloud_binding),
            shutdown: None,
            mapping_transition: Arc::new(tokio::sync::Mutex::new(())),
            deferred_replay: None,
            activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl::new(
                1,
            ),
            relocalization_ingress: None,
            activation_lifecycle_fence: Arc::new(parking_lot::Mutex::new(None)),
            activation_fault_after: Arc::new(parking_lot::Mutex::new(None)),
            stop_phase_a_after_detach: Arc::new(AtomicBool::new(false)),
            fail_start_mapping_stop_transition: Arc::new(AtomicBool::new(false)),
            start_mapping_before_cloud_clear: Arc::new(parking_lot::Mutex::new(None)),
            stop_before_graceful: Arc::new(parking_lot::Mutex::new(None)),
        }
    }

    #[cfg(test)]
    pub fn new_with_shutdown(bridge: GeometryRuntimeHandle, shutdown: ShutdownCoordinator) -> Self {
        let mut service = Self::new(bridge);
        service.mapping_transition = shutdown.mapping_transition.clone();
        service.deferred_replay = shutdown.deferred_replay.clone();
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

    pub(crate) fn new_with_shutdown_and_point_cloud_binding(
        bridge: GeometryRuntimeHandle,
        shutdown: ShutdownCoordinator,
        point_cloud_binding: Option<(LcmTopic, LcmMessageType)>,
        relocalization_ingress: Option<crate::correction_coordinator::RelocalizationCommandIngress>,
    ) -> Self {
        let activation_receipts = shutdown
            .correction_phase_a
            .as_ref()
            .map(crate::correction_coordinator::CorrectionPhaseAControl::activated_target_receipts)
            .unwrap_or_else(|| {
                crate::correction_coordinator::ActivatedTargetReceiptControl::new(1)
            });
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
            base_frame_id: configured_base_frame(),
            point_cloud_binding,
            mapping_transition: shutdown.mapping_transition.clone(),
            deferred_replay: shutdown.deferred_replay.clone(),
            shutdown: Some(shutdown),
            activation_receipts,
            relocalization_ingress,
            activation_lifecycle_fence: Arc::new(parking_lot::Mutex::new(None)),
            #[cfg(test)]
            activation_fault_after: Arc::new(parking_lot::Mutex::new(None)),
            #[cfg(test)]
            stop_phase_a_after_detach: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            fail_start_mapping_stop_transition: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            start_mapping_before_cloud_clear: Arc::new(parking_lot::Mutex::new(None)),
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
        state
    }

    fn take_matching_activation_lifecycle_fence(
        &self,
        activation_request_id: &str,
        activation_request_identity_sha256: [u8; 32],
    ) -> Result<Option<ActivationLifecycleFence>, ()> {
        let mut fence = self.activation_lifecycle_fence.lock();
        match fence.as_ref() {
            None => Ok(None),
            Some(existing)
                if existing.activation_request_id == activation_request_id
                    && existing.activation_request_identity_sha256
                        == activation_request_identity_sha256 =>
            {
                Ok(fence.take())
            }
            Some(_) => Err(()),
        }
    }

    fn retain_activation_lifecycle_fence(
        &self,
        activation_request_id: &str,
        activation_request_identity_sha256: [u8; 32],
        retained_global_map: Option<RetainedActivationGlobalMap>,
        lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
    ) -> Result<(), tokio::sync::OwnedMutexGuard<()>> {
        let mut fence = self.activation_lifecycle_fence.lock();
        if fence.is_some() {
            return Err(lifecycle_guard);
        }
        *fence = Some(ActivationLifecycleFence {
            activation_request_id: activation_request_id.to_owned(),
            activation_request_identity_sha256,
            retained_global_map,
            lifecycle_guard,
        });
        Ok(())
    }

    fn take_any_activation_lifecycle_fence(&self) -> Option<tokio::sync::OwnedMutexGuard<()>> {
        self.activation_lifecycle_fence
            .lock()
            .take()
            .map(|fence| fence.lifecycle_guard)
    }

    fn take_activation_lifecycle_fence(&self) -> Option<ActivationLifecycleFence> {
        self.activation_lifecycle_fence.lock().take()
    }

    fn restore_activation_lifecycle_fence(&self, fence: ActivationLifecycleFence) {
        let mut slot = self.activation_lifecycle_fence.lock();
        assert!(
            slot.is_none(),
            "activation lifecycle fence slot changed while Mapping transition was held"
        );
        *slot = Some(fence);
    }

    fn has_activation_lifecycle_fence(&self) -> bool {
        self.activation_lifecycle_fence.lock().is_some()
    }

    #[allow(clippy::too_many_arguments)]
    async fn finish_started_mapping_transition(
        &self,
        request: StartMappingRequest,
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        snapshot: MappingSessionSnapshot,
        cloud_map_reservation: CloudMapSessionReservation,
        lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        _mapping_transition: tokio::sync::OwnedMutexGuard<()>,
        mut cancellation_rx: watch::Receiver<bool>,
    ) -> Result<Response<MappingSessionState>, Status> {
        #[cfg(test)]
        if self
            .fail_start_mapping_stop_transition
            .swap(false, Ordering::AcqRel)
        {
            adapter.lock().exhaust_backend_generation_for_tests();
        }

        let deferred_replay = self.deferred_replay.clone();
        let mut occupancy_fence = None;
        if let Some(replay) = deferred_replay.as_deref() {
            if *cancellation_rx.borrow() {
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    lifecycle_guard,
                    &snapshot,
                    None,
                    "deferred StartMapping RPC waiter cancelled before occupancy fencing",
                )
                .await);
            }
        }

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
                    if let Some(replay) = deferred_replay.as_deref() {
                        return Err(finish_deferred_mapping_start_failure(
                            replay,
                            &adapter,
                            &self.bridge.mapping_lifecycle_lock,
                            lifecycle_guard,
                            &snapshot,
                            None,
                            format!(
                                "occupancy SHM session fence failed after startup odometry: {error}"
                            ),
                        )
                        .await);
                    }
                    let outcome = finish_failed_mapping_start_rollback(
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        lifecycle_guard,
                        &snapshot,
                        None,
                        format!("occupancy SHM session fence failed: {error}"),
                    )
                    .await;
                    return Err(Status::internal(outcome.detail_with_diagnostics()));
                }
            }
        }

        if let Some(replay) = deferred_replay.as_deref() {
            let map_frame_generation = adapter.lock().global_frame_generation();
            let claim = {
                let adapter = adapter.lock();
                replay.prepare_session(&snapshot, map_frame_generation, &adapter)
            };
            let claim = match claim {
                Ok(claim) => claim,
                Err(error) => {
                    return Err(finish_deferred_mapping_start_failure(
                        replay,
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        lifecycle_guard,
                        &snapshot,
                        occupancy_fence.take(),
                        format!("deferred Mapping claim failed: {error}"),
                    )
                    .await)
                }
            };

            let clear = match await_start_mapping_step(
                &mut cancellation_rx,
                cloud_map_reservation.activate(snapshot.session_generation),
            )
            .await
            {
                Some(clear) => clear,
                None => {
                    return Err(finish_deferred_mapping_start_failure(
                        replay,
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        lifecycle_guard,
                        &snapshot,
                        occupancy_fence.take(),
                        "deferred StartMapping RPC waiter cancelled during cloud-map clear",
                    )
                    .await)
                }
            };
            if let Err(clear_error) =
                validate_deferred_cloud_map_clear(&clear, snapshot.session_generation)
            {
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    lifecycle_guard,
                    &snapshot,
                    occupancy_fence.take(),
                    format!("deferred StartMapping cloud-map clear failed: {clear_error}"),
                )
                .await);
            }
            drop(lifecycle_guard);

            if matches!(claim, crate::lio_ingress::DeferredSessionClaim::Claimed) {
                let replay_result =
                    await_start_mapping_step(&mut cancellation_rx, replay.replay_claimed()).await;
                let failure_detail = match replay_result {
                    Some(Ok(())) => None,
                    Some(Err(error)) => Some(format!("deferred Mapping replay failed: {error}")),
                    None => Some(
                        "deferred StartMapping RPC waiter cancelled during retained replay"
                            .to_owned(),
                    ),
                };
                if let Some(failure_detail) = failure_detail {
                    let lifecycle_guard = self
                        .bridge
                        .mapping_lifecycle_lock
                        .clone()
                        .lock_owned()
                        .await;
                    return Err(finish_deferred_mapping_start_failure(
                        replay,
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        lifecycle_guard,
                        &snapshot,
                        occupancy_fence.take(),
                        failure_detail,
                    )
                    .await);
                }
            }

            let admission = await_start_mapping_step(
                &mut cancellation_rx,
                self.ensure_running_admission("StartMapping"),
            )
            .await;
            let admission_failure = match admission {
                Some(Ok(())) => None,
                Some(Err(error)) => Some(format!(
                    "deferred Mapping lost runtime admission before completion: {error}"
                )),
                None => Some(
                    "deferred StartMapping RPC waiter cancelled during runtime admission check"
                        .to_owned(),
                ),
            };
            if let Some(admission_failure) = admission_failure {
                let lifecycle_guard = self
                    .bridge
                    .mapping_lifecycle_lock
                    .clone()
                    .lock_owned()
                    .await;
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    lifecycle_guard,
                    &snapshot,
                    occupancy_fence.take(),
                    admission_failure,
                )
                .await);
            }

            let lifecycle_guard = match await_start_mapping_step(
                &mut cancellation_rx,
                self.bridge.mapping_lifecycle_lock.clone().lock_owned(),
            )
            .await
            {
                Some(lifecycle_guard) => lifecycle_guard,
                None => {
                    let lifecycle_guard = self
                        .bridge
                        .mapping_lifecycle_lock
                        .clone()
                        .lock_owned()
                        .await;
                    return Err(finish_deferred_mapping_start_failure(
                        replay,
                        &adapter,
                        &self.bridge.mapping_lifecycle_lock,
                        lifecycle_guard,
                        &snapshot,
                        occupancy_fence.take(),
                        "deferred StartMapping RPC waiter cancelled waiting for final lifecycle verification",
                    )
                    .await);
                }
            };
            if *cancellation_rx.borrow() {
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    lifecycle_guard,
                    &snapshot,
                    occupancy_fence.take(),
                    "deferred StartMapping RPC waiter cancelled before final lifecycle verification",
                )
                .await);
            }
            let (final_snapshot, final_map_frame_generation) = {
                let adapter = adapter.lock();
                (adapter.snapshot(), adapter.global_frame_generation())
            };
            if !final_snapshot.active
                || final_snapshot.session_id != request.session_id
                || final_snapshot.map_name != request.map_name
                || final_snapshot.session_generation != snapshot.session_generation
                || !replay.is_complete_for(
                    &request.session_id,
                    &request.map_name,
                    final_snapshot.session_generation,
                    final_map_frame_generation,
                )
            {
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    lifecycle_guard,
                    &snapshot,
                    occupancy_fence.take(),
                    "Mapping identity changed before deferred StartMapping completion",
                )
                .await);
            }
            drop(lifecycle_guard);
            drop(occupancy_fence.take());
            return Ok(Response::new(self.mapping_state_with_receipt(
                final_snapshot,
                "mapping session active; deferred startup replay complete",
            )));
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
                lifecycle_guard,
                &snapshot,
                occupancy_fence.take(),
                detail,
            )
            .await;
            return Err(Status::unavailable(outcome.detail_with_diagnostics()));
        }
        drop(occupancy_fence.take());
        drop(lifecycle_guard);
        Ok(Response::new(self.mapping_state_with_receipt(
            snapshot,
            "mapping session active",
        )))
    }

    #[cfg(test)]
    fn inject_activation_fault_after(
        &self,
        step: crate::correction_coordinator::ActivationJournalStep,
    ) {
        *self.activation_fault_after.lock() = Some(step);
    }

    #[cfg(test)]
    fn inject_stop_phase_a_after_detach(&self) {
        self.stop_phase_a_after_detach
            .store(true, Ordering::Release);
    }

    #[cfg(test)]
    fn inject_start_mapping_stop_transition_failure(&self) {
        self.fail_start_mapping_stop_transition
            .store(true, Ordering::Release);
    }

    #[cfg(test)]
    fn inject_start_mapping_before_cloud_clear_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.start_mapping_before_cloud_clear.lock() = Some(barrier);
    }

    #[cfg(test)]
    fn inject_activate_map_before_cloud_clear_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.start_mapping_before_cloud_clear.lock() = Some(barrier);
    }

    #[cfg(test)]
    fn inject_stop_before_graceful_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.stop_before_graceful.lock() = Some(barrier);
    }
}

#[cfg(test)]
fn take_activation_fault_after(
    service: &GeometryRuntimeService,
    step: crate::correction_coordinator::ActivationJournalStep,
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
        RelocalizationCommandError::Stopped => "RELOCALIZATION_OWNER_STOPPED",
        RelocalizationCommandError::OwnerUnavailable => "RELOCALIZATION_OWNER_UNAVAILABLE",
        RelocalizationCommandError::ResponseLost => "RELOCALIZATION_RESPONSE_LOST",
    };
    Status::unavailable(detail)
}

#[cfg(not(test))]
fn take_activation_fault_after(
    _service: &GeometryRuntimeService,
    _step: crate::correction_coordinator::ActivationJournalStep,
) -> bool {
    false
}

fn now_ns() -> u64 {
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
        success: true,
        phase: snapshot.phase.as_str().to_string(),
        message: message.into(),
        session_id: snapshot.session_id,
        map_name: snapshot.map_name,
        session_generation: snapshot.session_generation,
        revision: snapshot.revision,
        active: snapshot.active,
        idempotent_replay: snapshot.idempotent_replay,
        legacy_checkpoint_reconstructed: snapshot.legacy_checkpoint_reconstructed,
        mapping_backend_kind: snapshot.mapping_backend_kind,
        activation_receipt_schema_version: 0,
        activation_receipt_disposition: ActivationReceiptDisposition::Unspecified as i32,
        runtime_generation: 0,
        backend_generation: 0,
        activation_request_id: String::new(),
        activation_request_identity_sha256: Vec::new(),
        activation_identity_sha256: Vec::new(),
        target_receipt_sha256: Vec::new(),
        asset_manifest_sha256: Vec::new(),
        asset_bundle_sha256: Vec::new(),
        relocalization_target_available: false,
        source_policy_version: 0,
        source_policy_sha256: Vec::new(),
        deadline_policy_version: 0,
        deadline_policy_sha256: Vec::new(),
        algorithm_options_version: 0,
        algorithm_options_sha256: Vec::new(),
    }
}

#[derive(Clone, Debug)]
enum ActivationRequestIdentityClass {
    Modern {
        request_id: String,
        asset_manifest_sha256: [u8; 32],
        asset_bundle_sha256: [u8; 32],
    },
    Legacy {
        request_id: String,
    },
}

impl ActivationRequestIdentityClass {
    fn request_id(&self) -> &str {
        match self {
            Self::Modern { request_id, .. } | Self::Legacy { request_id } => request_id,
        }
    }

    const fn is_modern(&self) -> bool {
        matches!(self, Self::Modern { .. })
    }

    const fn asset_manifest_sha256(&self) -> Option<[u8; 32]> {
        match self {
            Self::Modern {
                asset_manifest_sha256,
                ..
            } => Some(*asset_manifest_sha256),
            Self::Legacy { .. } => None,
        }
    }

    const fn asset_bundle_sha256(&self) -> Option<[u8; 32]> {
        match self {
            Self::Modern {
                asset_bundle_sha256,
                ..
            } => Some(*asset_bundle_sha256),
            Self::Legacy { .. } => None,
        }
    }
}

fn canonical_push_string(out: &mut Vec<u8>, value: &str) -> Result<(), String> {
    let length =
        u32::try_from(value.len()).map_err(|_| "canonical string length exceeds u32".to_owned())?;
    out.extend_from_slice(&length.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn canonical_push_optional_digest(out: &mut Vec<u8>, digest: Option<[u8; 32]>) {
    match digest {
        Some(digest) => {
            out.push(1);
            out.extend_from_slice(&digest);
        }
        None => out.push(0),
    }
}

fn canonical_hash32(value: &[u8], label: &str) -> Result<[u8; 32], String> {
    value
        .try_into()
        .map_err(|_| format!("{label} must contain exactly 32 bytes"))
}

fn validate_canonical_protocol_id(value: &str, label: &str) -> Result<(), String> {
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

fn validate_canonical_text(value: &str, label: &str) -> Result<(), String> {
    if value.is_empty() || value.len() > 256 || value != value.trim() {
        return Err(format!("{label} must be 1..=256 bytes and already trimmed"));
    }
    Ok(())
}

fn classify_activation_request_identity(
    request: &ActivateMapRequest,
    runtime_generation: u64,
) -> Result<ActivationRequestIdentityClass, String> {
    let legacy = request.activation_contract_version == 0
        && request.activation_request_id.is_empty()
        && request.expected_runtime_generation == 0
        && request.asset_manifest_sha256.is_empty()
        && request.asset_bundle_sha256.is_empty();
    if legacy {
        let request_id = request.session_id.trim().to_owned();
        if request_id.is_empty() {
            return Err("session_id and map_name must not be empty".to_owned());
        }
        return Ok(ActivationRequestIdentityClass::Legacy { request_id });
    }

    let modern = request.activation_contract_version == 1
        && validate_canonical_protocol_id(&request.activation_request_id, "activation_request_id")
            .is_ok()
        && request.expected_runtime_generation != 0
        && request.expected_runtime_generation == runtime_generation
        && request.asset_manifest_sha256.len() == 32
        && request.asset_bundle_sha256.len() == 32;
    if !modern {
        return Err("ACTIVATE_MAP_IDENTITY_FIELDS_MIXED".to_owned());
    }
    validate_canonical_protocol_id(&request.session_id, "session_id")?;
    validate_canonical_text(&request.map_name, "map_name")?;
    Ok(ActivationRequestIdentityClass::Modern {
        request_id: request.activation_request_id.clone(),
        asset_manifest_sha256: canonical_hash32(
            &request.asset_manifest_sha256,
            "asset_manifest_sha256",
        )?,
        asset_bundle_sha256: canonical_hash32(&request.asset_bundle_sha256, "asset_bundle_sha256")?,
    })
}

fn canonical_occupancy_preimage(checkpoint: &OccupancyGridData) -> Result<Vec<u8>, String> {
    let meta = checkpoint
        .meta
        .as_ref()
        .ok_or_else(|| "checkpoint.meta is required".to_owned())?;
    if meta.width == 0 || meta.height == 0 || meta.revision == 0 {
        return Err("checkpoint dimensions and revision must be positive".to_owned());
    }
    if !meta.resolution.is_finite() || meta.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_owned());
    }
    if !meta.origin_x.is_finite() || !meta.origin_y.is_finite() {
        return Err("checkpoint origins must be finite".to_owned());
    }
    validate_canonical_text(&meta.frame_id, "checkpoint frame_id")?;
    let expected = usize::try_from(meta.width)
        .ok()
        .and_then(|width| {
            usize::try_from(meta.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| "checkpoint dimensions overflow".to_owned())?;
    if expected != checkpoint.data.len() {
        return Err(format!(
            "checkpoint data size mismatch: expected={expected} actual={}",
            checkpoint.data.len()
        ));
    }

    let mut out = b"nuwax.geometry.occupancy-checkpoint.v1\0".to_vec();
    out.extend_from_slice(&1_u32.to_be_bytes());
    out.extend_from_slice(&meta.width.to_be_bytes());
    out.extend_from_slice(&meta.height.to_be_bytes());
    out.extend_from_slice(&meta.resolution.to_be_bytes());
    let origin_x = if meta.origin_x == 0.0 {
        0.0
    } else {
        meta.origin_x
    };
    let origin_y = if meta.origin_y == 0.0 {
        0.0
    } else {
        meta.origin_y
    };
    out.extend_from_slice(&origin_x.to_be_bytes());
    out.extend_from_slice(&origin_y.to_be_bytes());
    canonical_push_string(&mut out, &meta.frame_id)?;
    out.extend_from_slice(&meta.revision.to_be_bytes());
    out.extend_from_slice(&meta.timestamp_ns.to_be_bytes());
    out.extend_from_slice(
        &u64::try_from(checkpoint.data.len())
            .map_err(|_| "checkpoint data length exceeds u64".to_owned())?
            .to_be_bytes(),
    );
    out.extend_from_slice(&crate::map_checksum::sha256_bytes(&checkpoint.data));
    Ok(out)
}

fn canonical_occupancy_sha256(checkpoint: &OccupancyGridData) -> Result<[u8; 32], String> {
    canonical_occupancy_preimage(checkpoint)
        .map(|preimage| crate::map_checksum::sha256_bytes(&preimage))
}

fn canonical_cloud_preimage(
    target: &crate::pointcloud::ImmutableActivatedTarget,
) -> Result<Vec<u8>, String> {
    validate_canonical_text(&target.frame_id, "cloud checkpoint frame_id")?;
    let mut out = b"nuwax.geometry.cloud-checkpoint.v1\0".to_vec();
    canonical_push_string(&mut out, &target.frame_id)?;
    out.extend_from_slice(&target.source_revision.to_be_bytes());
    out.extend_from_slice(&target.point_count.to_be_bytes());
    out.extend_from_slice(&target.voxel_size.to_be_bytes());
    out.extend_from_slice(
        &u64::try_from(target.xyz_f32_le.len())
            .map_err(|_| "cloud checkpoint data length exceeds u64".to_owned())?
            .to_be_bytes(),
    );
    out.extend_from_slice(&target.xyz_sha256);
    out.extend_from_slice(&target.payload_schema_version.to_be_bytes());
    Ok(out)
}

fn canonical_cloud_sha256(
    target: &crate::pointcloud::ImmutableActivatedTarget,
) -> Result<[u8; 32], String> {
    canonical_cloud_preimage(target).map(|preimage| crate::map_checksum::sha256_bytes(&preimage))
}

fn canonical_append_commit_identity(
    out: &mut Vec<u8>,
    identity: Option<&MappingCommitIdentity>,
    label: &str,
) -> Result<(), String> {
    let identity = validate_mapping_commit_identity(identity, label)?;
    out.extend_from_slice(&identity.boot_nonce);
    out.extend_from_slice(&identity.sequence.to_be_bytes());
    Ok(())
}

fn canonical_append_occupancy_receipt(
    out: &mut Vec<u8>,
    receipt: &CanonicalOccupancyCommitReceipt,
) -> Result<(), String> {
    canonical_append_commit_identity(out, receipt.commit_id.as_ref(), "occupancy receipt")?;
    canonical_push_string(out, &receipt.session_id)?;
    out.extend_from_slice(&receipt.session_generation.to_be_bytes());
    out.extend_from_slice(&receipt.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&receipt.source_timestamp_ns.to_be_bytes());
    canonical_push_string(out, &receipt.backend_kind)?;
    out.extend_from_slice(&receipt.backend_generation.to_be_bytes());
    Ok(())
}

fn canonical_append_cloud_receipt(
    out: &mut Vec<u8>,
    receipt: &CanonicalCloudCommitReceipt,
) -> Result<(), String> {
    canonical_append_commit_identity(out, receipt.commit_id.as_ref(), "cloud receipt")?;
    out.extend_from_slice(&receipt.session_generation.to_be_bytes());
    out.extend_from_slice(&receipt.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&receipt.cloud_revision.to_be_bytes());
    out.extend_from_slice(&receipt.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&receipt.point_count.to_be_bytes());
    Ok(())
}

fn immutable_target_from_checkpoint(
    checkpoint: &CloudMapCheckpoint,
    expected_map_frame_id: &str,
    modern: bool,
) -> Result<crate::pointcloud::ImmutableActivatedTarget, String> {
    let canonical_frame = if modern {
        checkpoint.frame_id.as_str()
    } else {
        checkpoint.frame_id.trim().trim_start_matches('/')
    };
    crate::pointcloud::decode_immutable_activated_target(
        canonical_frame,
        expected_map_frame_id,
        checkpoint.source_revision,
        checkpoint.point_count,
        checkpoint.voxel_size,
        &checkpoint.xyz_f32_le,
        &checkpoint.xyz_sha256,
        checkpoint.payload_schema_version,
    )
}

fn canonical_backend_sha256(
    checkpoint: &MappingBackendCheckpoint,
    expected_map_frame_id: &str,
) -> Result<[u8; 32], String> {
    let content_sha256 = canonical_hash32(&checkpoint.content_sha256, "content_sha256")?;
    let occupancy_receipt = checkpoint
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint occupancy_receipt is required".to_owned())?;
    let cloud_receipt = checkpoint
        .cloud_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint cloud_receipt is required".to_owned())?;
    let occupancy_checkpoint = checkpoint
        .canonical_occupancy_checkpoint
        .as_ref()
        .ok_or_else(|| {
            "mapping backend checkpoint canonical_occupancy_checkpoint is required".to_owned()
        })?;
    let cloud_checkpoint = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .ok_or_else(|| {
            "mapping backend checkpoint canonical_cloud_checkpoint is required".to_owned()
        })?;
    let cloud_target =
        immutable_target_from_checkpoint(cloud_checkpoint, expected_map_frame_id, false)?;

    let mut out = b"nuwax.geometry.backend-checkpoint.v1\0".to_vec();
    out.extend_from_slice(&checkpoint.schema_version.to_be_bytes());
    canonical_push_string(&mut out, &checkpoint.backend_kind)?;
    out.extend_from_slice(&checkpoint.source_session_generation.to_be_bytes());
    out.extend_from_slice(&content_sha256);
    out.extend_from_slice(&checkpoint.payload_size.to_be_bytes());
    out.extend_from_slice(&checkpoint.created_at_ns.to_be_bytes());
    out.extend_from_slice(&crate::map_checksum::sha256_bytes(&checkpoint.payload));
    canonical_push_string(&mut out, &checkpoint.source_session_id)?;
    canonical_append_occupancy_receipt(&mut out, occupancy_receipt)?;
    canonical_append_cloud_receipt(&mut out, cloud_receipt)?;
    out.extend_from_slice(&canonical_occupancy_sha256(occupancy_checkpoint)?);
    out.extend_from_slice(&canonical_cloud_sha256(&cloud_target)?);
    Ok(crate::map_checksum::sha256_bytes(&out))
}

fn canonical_asset_bundle_preimage(
    asset_manifest_sha256: [u8; 32],
    occupancy_sha256: [u8; 32],
    cloud_sha256: Option<[u8; 32]>,
    backend_sha256: Option<[u8; 32]>,
) -> Vec<u8> {
    let mut out = b"nuwax.geometry.asset-bundle.v1\0".to_vec();
    out.extend_from_slice(&1_u32.to_be_bytes());
    out.extend_from_slice(&asset_manifest_sha256);
    out.extend_from_slice(&occupancy_sha256);
    canonical_push_optional_digest(&mut out, cloud_sha256);
    canonical_push_optional_digest(&mut out, backend_sha256);
    out
}

#[allow(clippy::too_many_arguments)]
fn canonical_activation_request_preimage(
    class: &ActivationRequestIdentityClass,
    runtime_generation: u64,
    session_id: &str,
    map_name: &str,
    occupancy_sha256: [u8; 32],
    cloud_sha256: Option<[u8; 32]>,
    backend_sha256: Option<[u8; 32]>,
) -> Result<Vec<u8>, String> {
    let mut out = if class.is_modern() {
        b"nuwax.geometry.activate-map.request.v1\0".to_vec()
    } else {
        b"nuwax.geometry.activate-map.legacy-request.v1\0".to_vec()
    };
    if let ActivationRequestIdentityClass::Modern {
        asset_manifest_sha256,
        asset_bundle_sha256,
        ..
    } = class
    {
        out.extend_from_slice(&1_u32.to_be_bytes());
        canonical_push_string(&mut out, class.request_id())?;
        out.extend_from_slice(&runtime_generation.to_be_bytes());
        canonical_push_string(&mut out, session_id)?;
        canonical_push_string(&mut out, map_name)?;
        out.extend_from_slice(asset_manifest_sha256);
        out.extend_from_slice(asset_bundle_sha256);
    } else {
        out.extend_from_slice(&runtime_generation.to_be_bytes());
        canonical_push_string(&mut out, session_id)?;
        canonical_push_string(&mut out, map_name)?;
    }
    out.extend_from_slice(&occupancy_sha256);
    canonical_push_optional_digest(&mut out, cloud_sha256);
    canonical_push_optional_digest(&mut out, backend_sha256);
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn canonical_activation_identity_preimage(
    activation_request_identity_sha256: [u8; 32],
    runtime_generation: u64,
    session_generation: u64,
    backend_generation: u64,
    backend_kind: &str,
    backend_sha256: Option<[u8; 32]>,
    asset_manifest_sha256: [u8; 32],
    asset_bundle_sha256: [u8; 32],
) -> Result<Vec<u8>, String> {
    let mut out = b"nuwax.geometry.activate-map.identity.v1\0".to_vec();
    out.extend_from_slice(&activation_request_identity_sha256);
    out.extend_from_slice(&runtime_generation.to_be_bytes());
    out.extend_from_slice(&session_generation.to_be_bytes());
    out.extend_from_slice(&backend_generation.to_be_bytes());
    canonical_push_string(&mut out, backend_kind)?;
    canonical_push_optional_digest(&mut out, backend_sha256);
    out.extend_from_slice(&asset_manifest_sha256);
    out.extend_from_slice(&asset_bundle_sha256);
    out.extend_from_slice(
        &crate::correction_coordinator::RELOCALIZATION_SOURCE_POLICY_VERSION_V1.to_be_bytes(),
    );
    out.extend_from_slice(&crate::correction_coordinator::relocalization_source_policy_sha256_v1());
    out.extend_from_slice(
        &crate::correction_coordinator::RELOCALIZATION_DEADLINE_POLICY_VERSION_V1.to_be_bytes(),
    );
    out.extend_from_slice(
        &crate::correction_coordinator::relocalization_deadline_policy_sha256_v1(),
    );
    out.extend_from_slice(
        &crate::correction_coordinator::RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1.to_be_bytes(),
    );
    out.extend_from_slice(
        &crate::correction_coordinator::relocalization_algorithm_options_sha256_v1(),
    );
    Ok(out)
}

fn canonical_target_receipt_preimage(
    activation_identity_sha256: [u8; 32],
    cloud_sha256: [u8; 32],
    source_revision: u64,
    installed_cloud_revision: u64,
    activated_session_generation: u64,
    activated_backend_generation: u64,
    receipt: &crate::mapping_commit::CloudCommitReceipt,
) -> Result<Vec<u8>, String> {
    if receipt.session_generation != activated_session_generation
        || receipt.cloud_revision != installed_cloud_revision
    {
        return Err("activated target cloud receipt identity mismatch".to_owned());
    }
    let mut out = b"nuwax.geometry.activated-target-receipt.v1\0".to_vec();
    out.extend_from_slice(&activation_identity_sha256);
    out.extend_from_slice(&cloud_sha256);
    out.extend_from_slice(&source_revision.to_be_bytes());
    out.extend_from_slice(&installed_cloud_revision.to_be_bytes());
    out.extend_from_slice(&activated_session_generation.to_be_bytes());
    out.extend_from_slice(&activated_backend_generation.to_be_bytes());
    out.extend_from_slice(&receipt.commit_id.boot_nonce.to_be_bytes());
    out.extend_from_slice(&receipt.commit_id.sequence.to_be_bytes());
    out.extend_from_slice(&receipt.session_generation.to_be_bytes());
    out.extend_from_slice(&receipt.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&receipt.cloud_revision.to_be_bytes());
    out.extend_from_slice(&receipt.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&receipt.point_count.to_be_bytes());
    Ok(out)
}

fn activation_journal_key(
    activation_request_identity_sha256: [u8; 32],
    step: crate::correction_coordinator::ActivationJournalStep,
) -> [u8; 32] {
    let mut bytes = b"nuwax.geometry.activation-journal-key.v1\0".to_vec();
    bytes.extend_from_slice(&activation_request_identity_sha256);
    bytes.extend_from_slice(&(step as u32).to_be_bytes());
    crate::map_checksum::sha256_bytes(&bytes)
}

fn record_activation_journal(
    control: &crate::correction_coordinator::ActivatedTargetReceiptControl,
    token: &crate::correction_coordinator::ActivationToken,
    request_identity: [u8; 32],
    step: crate::correction_coordinator::ActivationJournalStep,
    status: crate::correction_coordinator::ActivationJournalStatus,
    verifier_receipt_sha256: Option<[u8; 32]>,
    typed_failure: Option<String>,
) -> bool {
    control.journal(
        token,
        crate::correction_coordinator::ActivationJournalEntry {
            step,
            idempotency_key: activation_journal_key(request_identity, step),
            status,
            verifier_receipt_sha256,
            typed_failure,
        },
    )
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
    request_identity: [u8; 32],
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
        let retirement_generation = retirement.generation();
        let retirement_proof =
            crate::map_checksum::sha256_bytes(&retirement_generation.to_be_bytes());
        if !record_activation_journal(
            &service.activation_receipts,
            token,
            request_identity,
            crate::correction_coordinator::ActivationJournalStep::RetirementPrepared,
            crate::correction_coordinator::ActivationJournalStatus::Prepared,
            Some(retirement_proof),
            None,
        ) || !service.activation_receipts.validate_token(token)
        {
            disposition = ActivationReceiptDisposition::OutcomeUnknown;
            detail.push_str("; retirement token could not be proven before join");
        }
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

#[allow(clippy::too_many_lines)]
async fn activate_map_r1(
    service: GeometryRuntimeService,
    mut request: ActivateMapRequest,
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

    let occupancy_checkpoint = request
        .checkpoint
        .take()
        .ok_or_else(|| Status::invalid_argument("checkpoint is required"))?;
    let checkpoint_timestamp_ns = occupancy_checkpoint
        .meta
        .as_ref()
        .map(|meta| meta.timestamp_ns)
        .ok_or_else(|| Status::invalid_argument("checkpoint.meta is required"))?;
    let occupancy_sha256 =
        canonical_occupancy_sha256(&occupancy_checkpoint).map_err(Status::invalid_argument)?;
    let checkpoint = decode_checkpoint_grid(occupancy_checkpoint.clone(), &expected_map_frame_id)
        .map_err(Status::invalid_argument)?;

    let cloud_checkpoint = request.cloud_checkpoint.take();
    if identity_class.is_modern() && cloud_checkpoint.is_none() {
        return Err(Status::invalid_argument(
            "ACTIVATE_MAP_CLOUD_CHECKPOINT_REQUIRED",
        ));
    }
    let immutable_target = cloud_checkpoint
        .as_ref()
        .map(|checkpoint| {
            validate_cloud_checkpoint_metadata(checkpoint, &expected_map_frame_id)?;
            immutable_target_from_checkpoint(
                checkpoint,
                &expected_map_frame_id,
                identity_class.is_modern(),
            )
        })
        .transpose()
        .map_err(Status::invalid_argument)?;
    let cloud_sha256 = immutable_target
        .as_ref()
        .map(canonical_cloud_sha256)
        .transpose()
        .map_err(Status::invalid_argument)?;
    let mapping_backend_checkpoint = request.mapping_backend_checkpoint.take();
    if let Some(checkpoint) = mapping_backend_checkpoint.as_ref() {
        validate_mapping_backend_checkpoint(checkpoint, selected_backend_kind)
            .map_err(Status::invalid_argument)?;
        validate_mapping_checkpoint_bundle(
            checkpoint,
            &occupancy_checkpoint,
            cloud_checkpoint.as_ref(),
        )
        .map_err(Status::invalid_argument)?;
    } else if selected_backend_kind == crate::mapping_backend::MappingBackendKind::Raytracing {
        return Err(Status::failed_precondition(
            "BACKEND_CHECKPOINT_REQUIRED: raytracing map activation requires a matching mapping backend checkpoint",
        ));
    }
    let backend_sha256 = mapping_backend_checkpoint
        .as_ref()
        .map(|checkpoint| canonical_backend_sha256(checkpoint, &expected_map_frame_id))
        .transpose()
        .map_err(Status::invalid_argument)?;

    if let ActivationRequestIdentityClass::Modern {
        asset_manifest_sha256,
        asset_bundle_sha256,
        ..
    } = &identity_class
    {
        let expected_bundle = crate::map_checksum::sha256_bytes(&canonical_asset_bundle_preimage(
            *asset_manifest_sha256,
            occupancy_sha256,
            cloud_sha256,
            backend_sha256,
        ));
        if expected_bundle != *asset_bundle_sha256 {
            return Err(Status::invalid_argument(
                "ACTIVATE_MAP_ASSET_BUNDLE_SHA256_MISMATCH",
            ));
        }
    }

    if let Some(target) = immutable_target.as_ref() {
        let writer = service.bridge.cloud_map_writer.lock();
        let writer = writer
            .as_ref()
            .ok_or_else(|| Status::unavailable("cloud-map SHM writer is unavailable"))?;
        writer
            .validate_checkpoint(target.voxel_size, target.point_count, &target.xyz_f32_le)
            .map_err(Status::invalid_argument)?;
    }

    let session_id = if identity_class.is_modern() {
        request.session_id.clone()
    } else {
        request.session_id.trim().to_owned()
    };
    let map_name = if identity_class.is_modern() {
        request.map_name.clone()
    } else {
        request.map_name.trim().to_owned()
    };
    let request_preimage = canonical_activation_request_preimage(
        &identity_class,
        service.activation_receipts.runtime_generation(),
        &session_id,
        &map_name,
        occupancy_sha256,
        cloud_sha256,
        backend_sha256,
    )
    .map_err(Status::invalid_argument)?;
    let activation_request_identity_sha256 = crate::map_checksum::sha256_bytes(&request_preimage);
    let seed = crate::correction_coordinator::ActivationReceiptSeed {
        activation_request_id: identity_class.request_id().to_owned(),
        activation_request_identity_sha256,
        asset_manifest_sha256: identity_class.asset_manifest_sha256(),
        asset_bundle_sha256: identity_class.asset_bundle_sha256(),
        modern: identity_class.is_modern(),
        immutable_target: immutable_target.clone(),
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
    let (mut retained_global_map, mut lifecycle_guard, restore_fence_on_early_return) =
        match service.take_matching_activation_lifecycle_fence(
            identity_class.request_id(),
            activation_request_identity_sha256,
        ) {
            Ok(Some(fence)) => (fence.retained_global_map, Some(fence.lifecycle_guard), true),
            Ok(None) => (
                None,
                Some(bridge.mapping_lifecycle_lock.clone().lock_owned().await),
                false,
            ),
            Err(()) => {
                let current = adapter.lock().snapshot();
                let terminal = match service.activation_receipts.stage_or_replay(
                    seed.clone(),
                    mapping_session_state(current, "activation lifecycle fence conflict"),
                    false,
                ) {
                    crate::correction_coordinator::ActivationLedgerAdmission::Terminal(state) => {
                        *state
                    }
                    crate::correction_coordinator::ActivationLedgerAdmission::Execute(_) => {
                        MappingSessionState {
                            success: false,
                            message: "ACTIVATE_MAP_LIFECYCLE_FENCE_BUSY".to_owned(),
                            activation_receipt_disposition: ActivationReceiptDisposition::Conflict
                                as i32,
                            ..MappingSessionState::default()
                        }
                    }
                };
                return Ok(Response::new(terminal));
            }
        };
    if let Err(status) = service.ensure_running_admission("ActivateMap").await {
        if restore_fence_on_early_return {
            service.restore_activation_lifecycle_fence(ActivationLifecycleFence {
                activation_request_id: identity_class.request_id().to_owned(),
                activation_request_identity_sha256,
                retained_global_map,
                lifecycle_guard: lifecycle_guard
                    .take()
                    .expect("ActivateMap early return owns lifecycle guard"),
            });
        }
        return Err(status);
    }
    if bridge.explore.blocks_mapping_mutation().await {
        let exploration = bridge.explore.snapshot();
        if restore_fence_on_early_return {
            service.restore_activation_lifecycle_fence(ActivationLifecycleFence {
                activation_request_id: identity_class.request_id().to_owned(),
                activation_request_identity_sha256,
                retained_global_map,
                lifecycle_guard: lifecycle_guard
                    .take()
                    .expect("ActivateMap early return owns lifecycle guard"),
            });
        }
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
            if restore_fence_on_early_return {
                service.restore_activation_lifecycle_fence(ActivationLifecycleFence {
                    activation_request_id: identity_class.request_id().to_owned(),
                    activation_request_identity_sha256,
                    retained_global_map,
                    lifecycle_guard: lifecycle_guard
                        .take()
                        .expect("ActivateMap early return owns lifecycle guard"),
                });
            }
            return Ok(Response::new(*state));
        }
        crate::correction_coordinator::ActivationLedgerAdmission::Execute(token) => token,
    };

    if current.active {
        let mut lifecycle_guard = lifecycle_guard
            .take()
            .expect("active ActivateMap owns lifecycle guard");
        macro_rules! settle_current_failure {
            ($disposition:expr, $detail:expr) => {{
                let detail = $detail;
                let failure_state =
                    mapping_session_state(adapter.lock().snapshot(), detail.clone());
                let retirement = adapter.lock().abort_started_session_detached(
                    &current.session_id,
                    current.session_generation,
                );
                drop(lifecycle_guard);
                let terminal = settle_failed_activation_retirement(
                    &service,
                    &adapter,
                    &bridge,
                    &token,
                    activation_request_identity_sha256,
                    retirement,
                    $disposition,
                    detail,
                    failure_state,
                )
                .await;
                return Ok(Response::new(terminal));
            }};
        }
        if current.session_id != session_id
            || current.map_name != map_name
            || !adapter
                .lock()
                .checkpoint_activation_matches(&activation_request_identity_sha256)
        {
            settle_current_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_STAGED_ACTIVE_IDENTITY_UNVERIFIABLE".to_owned()
            );
        }
        let proof = match service.activation_receipts.publication_proof(&token) {
            Some(proof) => proof,
            None => {
                settle_current_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    "ACTIVATE_MAP_STAGED_PUBLICATION_PROOF_UNAVAILABLE".to_owned()
                );
            }
        };
        let target = match service.activation_receipts.immutable_target(&token) {
            Some(target) => target,
            None => {
                settle_current_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    "ACTIVATE_MAP_STAGED_TARGET_UNAVAILABLE".to_owned()
                );
            }
        };
        if !service.activation_receipts.journal_step_applied(
            &token,
            crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
        ) {
            let committed = CommittedMappingCloudMap {
                notification: CloudMapNotification {
                    revision: proof.cloud_revision,
                    point_count: proof.point_count,
                    voxel_size: proof.voxel_size,
                    session_generation: current.session_generation,
                },
                source_timestamp_ns: proof.source_timestamp_ns,
                xyz_f32_le: target.xyz_f32_le.clone(),
                receipt: proof.cloud_receipt.clone(),
            };
            let replaying_retained_publication = retained_global_map.is_some();
            let retained_publication = match retained_global_map.take() {
                Some(retained) => {
                    if let Err(error) = retained.validate_replay(
                        activation_request_identity_sha256,
                        &expected_map_frame_id,
                        &committed,
                    ) {
                        let detail = format!(
                            "cloud-map checkpoint retained publication identity failed: {error}"
                        );
                        match service.retain_activation_lifecycle_fence(
                            identity_class.request_id(),
                            activation_request_identity_sha256,
                            Some(retained),
                            lifecycle_guard,
                        ) {
                            Ok(()) => return Err(Status::internal(detail)),
                            Err(guard) => {
                                lifecycle_guard = guard;
                                settle_current_failure!(
                                    ActivationReceiptDisposition::OutcomeUnknown,
                                    format!("{detail}; lifecycle fence retention failed")
                                );
                            }
                        }
                    }
                    retained
                }
                None => match RetainedActivationGlobalMap::from_committed(
                    activation_request_identity_sha256,
                    &expected_map_frame_id,
                    &committed,
                ) {
                    Ok(retained) => retained,
                    Err(error) => {
                        settle_current_failure!(
                            ActivationReceiptDisposition::OutcomeUnknown,
                            format!("ACTIVATE_MAP_GLOBAL_MAP_FRAME_BUILD_FAILED: {error}")
                        );
                    }
                },
            };
            let publish_result = if replaying_retained_publication {
                publish_retained_global_map_frame(&bridge, &retained_publication.frame).await
            } else {
                publish_committed_global_map_in_frame(&bridge, &committed, &expected_map_frame_id)
                    .await
            };
            if let Err(error) = publish_result {
                let detail = format!(
                    "cloud-map checkpoint installed but typed /global_map publication failed: {error}"
                );
                match service.retain_activation_lifecycle_fence(
                    identity_class.request_id(),
                    activation_request_identity_sha256,
                    Some(retained_publication),
                    lifecycle_guard,
                ) {
                    Ok(()) => return Err(Status::internal(detail)),
                    Err(guard) => {
                        lifecycle_guard = guard;
                        settle_current_failure!(
                            ActivationReceiptDisposition::OutcomeUnknown,
                            format!("{detail}; lifecycle fence retention failed")
                        );
                    }
                }
            }
            if take_activation_fault_after(
                &service,
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
            ) {
                settle_current_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_TYPED_GLOBAL_MAP_RETRY".to_owned()
                );
            }
            if !record_activation_journal(
                &service.activation_receipts,
                &token,
                activation_request_identity_sha256,
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                Some(crate::map_checksum::sha256_bytes(&target.xyz_f32_le)),
                None,
            ) {
                settle_current_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "ACTIVATE_MAP_PUBLICATION_JOURNAL_CAS_FAILED".to_owned()
                );
            }
        }
        let mut replay = current.clone();
        replay.idempotent_replay = true;
        let state = mapping_session_state(replay, "map checkpoint publication resumed");
        let target_receipt = identity_class
            .is_modern()
            .then_some(proof.target_receipt_sha256);
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
        if take_activation_fault_after(
            &service,
            crate::correction_coordinator::ActivationJournalStep::Seal,
        ) {
            return Err(Status::unavailable("INJECTED_AFTER_SEAL_RESPONSE_LOSS"));
        }
        return Ok(Response::new(sealed));
    }

    macro_rules! return_precommit_failure_with_fence {
        ($disposition:expr, $detail:expr) => {{
            let detail = $detail;
            let state = activation_failure_state(&service, &adapter, &token, $disposition, &detail);
            if restore_fence_on_early_return {
                service.restore_activation_lifecycle_fence(ActivationLifecycleFence {
                    activation_request_id: identity_class.request_id().to_owned(),
                    activation_request_identity_sha256,
                    retained_global_map,
                    lifecycle_guard: lifecycle_guard
                        .take()
                        .expect("ActivateMap early return owns lifecycle guard"),
                });
            }
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

    let mut restore_resolution = if let Some(checkpoint) = mapping_backend_checkpoint.as_ref() {
        if !record_activation_journal(
            &service.activation_receipts,
            &token,
            activation_request_identity_sha256,
            crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
            crate::correction_coordinator::ActivationJournalStatus::Prepared,
            backend_sha256,
            None,
        ) {
            let _ = staged_worker.stop_and_join();
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_RESTORE_PREPARE_CAS_FAILED"
            );
        }
        let receiver = match staged_worker
            .client()
            .restore_checkpoint(checkpoint.payload.clone())
        {
            Ok(receiver) => receiver,
            Err(error) => {
                let _ = staged_worker.stop_and_join();
                return_precommit_failure_with_fence!(
                    ActivationReceiptDisposition::InvalidatedPrecommit,
                    format!("ACTIVATE_MAP_RESTORE_SUBMISSION_FAILED: {error:?}")
                );
            }
        };
        if take_activation_fault_after(
            &service,
            crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
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
            lifecycle_guard = Some(bridge.mapping_lifecycle_lock.clone().lock_owned().await);
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
        if !record_activation_journal(
            &service.activation_receipts,
            &token,
            activation_request_identity_sha256,
            crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            backend_sha256,
            None,
        ) {
            let _ = resolution.abort();
            let _ = staged_worker.stop_and_join();
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_RESTORE_JOURNAL_CAS_FAILED"
            );
        }
        Some(resolution)
    } else {
        None
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
    macro_rules! settle_started_failure {
        ($disposition:expr, $detail:expr) => {{
            let detail = $detail;
            let failure_state = mapping_session_state(adapter.lock().snapshot(), detail.clone());
            let retirement = adapter
                .lock()
                .abort_started_session_detached(&started.session_id, started.session_generation);
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
                activation_request_identity_sha256,
                retirement,
                $disposition,
                detail,
                failure_state,
            )
            .await;
            return Ok(Response::new(terminal));
        }};
    }
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::SessionStart,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_SESSION_START".to_owned()
        );
    }
    let final_activation_identity = if let ActivationRequestIdentityClass::Modern {
        asset_manifest_sha256,
        asset_bundle_sha256,
        ..
    } = &identity_class
    {
        crate::map_checksum::sha256_bytes(
            &canonical_activation_identity_preimage(
                activation_request_identity_sha256,
                service.activation_receipts.runtime_generation(),
                started.session_generation,
                reserved_backend_generation,
                selected_backend_kind.stable_name(),
                backend_sha256,
                *asset_manifest_sha256,
                *asset_bundle_sha256,
            )
            .map_err(Status::internal)?,
        )
    } else {
        let mut bytes = b"nuwax.geometry.activate-map.legacy-identity.v1\0".to_vec();
        bytes.extend_from_slice(&activation_request_identity_sha256);
        bytes.extend_from_slice(&started.session_generation.to_be_bytes());
        bytes.extend_from_slice(&reserved_backend_generation.to_be_bytes());
        crate::map_checksum::sha256_bytes(&bytes)
    };
    token = match service.activation_receipts.bind_activation_identity(
        &token,
        final_activation_identity,
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
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::SessionStart,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(final_activation_identity),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "ACTIVATE_MAP_SESSION_START_JOURNAL_CAS_FAILED".to_owned()
        );
    }

    if let Some(writer) = &bridge.occupancy_shm {
        let begin_result = { writer.lock().begin_session(started.session_generation) };
        if let Err(error) = begin_result {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                format!("ACTIVATE_MAP_OCCUPANCY_FENCE_FAILED: {error}")
            );
        }
        if take_activation_fault_after(
            &service,
            crate::correction_coordinator::ActivationJournalStep::OccupancyShmBegin,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "INJECTED_AFTER_OCCUPANCY_SHM_BEGIN".to_owned()
            );
        }
        if !record_activation_journal(
            &service.activation_receipts,
            &token,
            activation_request_identity_sha256,
            crate::correction_coordinator::ActivationJournalStep::OccupancyShmBegin,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            Some(crate::map_checksum::sha256_bytes(
                &started.session_generation.to_be_bytes(),
            )),
            None,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_OCCUPANCY_FENCE_JOURNAL_CAS_FAILED".to_owned()
            );
        }
    }

    let revision_floor = bridge.state.read().await.map.revision;
    let install_result = {
        adapter
            .lock()
            .install_checkpoint(checkpoint, checkpoint_timestamp_ns, revision_floor)
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
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::OccupancyInstall,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_OCCUPANCY_INSTALL".to_owned()
        );
    }
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::OccupancyInstall,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_sha256),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "ACTIVATE_MAP_OCCUPANCY_INSTALL_JOURNAL_CAS_FAILED".to_owned()
        );
    }

    let source_timestamp_ns = map.timestamp_ns;
    let cloud_commit = if let Some(target) = immutable_target.as_ref() {
        let installed = {
            let mut writer = bridge.cloud_map_writer.lock();
            match writer.as_mut() {
                Some(writer) => writer.install_checkpoint(
                    &occupancy_receipt,
                    target.voxel_size,
                    target.point_count,
                    &target.xyz_f32_le,
                ),
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
            || correlated.snapshot.point_count != target.point_count
            || correlated.snapshot.voxel_size.to_bits() != target.voxel_size.to_bits()
            || correlated.receipt.session_generation != started.session_generation
            || correlated.receipt.point_count != target.point_count
        {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_CLOUD_EFFECT_UNVERIFIABLE".to_owned()
            );
        }
        #[cfg(test)]
        let correlated = {
            let mut correlated = correlated;
            if take_activation_fault_after(
                &service,
                crate::correction_coordinator::ActivationJournalStep::TargetReceiptVerification,
            ) {
                correlated.receipt.cloud_revision = correlated
                    .receipt
                    .cloud_revision
                    .checked_add(1)
                    .expect("test cloud revision must have headroom");
            }
            correlated
        };
        if take_activation_fault_after(
            &service,
            crate::correction_coordinator::ActivationJournalStep::CloudInstall,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "INJECTED_AFTER_CLOUD_INSTALL".to_owned()
            );
        }
        let target_receipt_preimage = match canonical_target_receipt_preimage(
            final_activation_identity,
            cloud_sha256.expect("cloud target digest exists"),
            target.source_revision,
            correlated.snapshot.revision,
            started.session_generation,
            reserved_backend_generation,
            &correlated.receipt,
        ) {
            Ok(preimage) => preimage,
            Err(error) => {
                settle_started_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    format!("ACTIVATE_MAP_TARGET_RECEIPT_OUTCOME_UNKNOWN: {error}")
                );
            }
        };
        let target_receipt_sha256 = crate::map_checksum::sha256_bytes(&target_receipt_preimage);
        let proof = crate::correction_coordinator::ActivationPublicationProof {
            source_timestamp_ns,
            cloud_revision: correlated.snapshot.revision,
            point_count: target.point_count,
            voxel_size: target.voxel_size,
            cloud_receipt: correlated.receipt.clone(),
            target_receipt_sha256,
        };
        if !service
            .activation_receipts
            .store_publication_proof(&token, proof)
            || !record_activation_journal(
                &service.activation_receipts,
                &token,
                activation_request_identity_sha256,
                crate::correction_coordinator::ActivationJournalStep::CloudInstall,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_sha256,
                None,
            )
        {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_CLOUD_PROOF_JOURNAL_CAS_FAILED".to_owned()
            );
        }
        let notification = CloudMapNotification {
            revision: correlated.snapshot.revision,
            point_count: target.point_count,
            voxel_size: target.voxel_size,
            session_generation: started.session_generation,
        };
        Some((
            notification.clone(),
            CommittedMappingCloudMap {
                notification,
                source_timestamp_ns,
                xyz_f32_le: target.xyz_f32_le.clone(),
                receipt: correlated.receipt,
            },
            target_receipt_sha256,
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
        if take_activation_fault_after(
            &service,
            crate::correction_coordinator::ActivationJournalStep::BackendCommit,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "INJECTED_AFTER_BACKEND_COMMIT".to_owned()
            );
        }
        if !record_activation_journal(
            &service.activation_receipts,
            &token,
            activation_request_identity_sha256,
            crate::correction_coordinator::ActivationJournalStep::BackendCommit,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            backend_sha256,
            None,
        ) {
            settle_started_failure!(
                ActivationReceiptDisposition::OutcomeUnknown,
                "ACTIVATE_MAP_BACKEND_COMMIT_JOURNAL_CAS_FAILED".to_owned()
            );
        }
    }

    let activation_record_result = {
        let mut adapter = adapter.lock();
        adapter.set_legacy_checkpoint_reconstructed(mapping_backend_checkpoint.is_none());
        adapter.record_checkpoint_activation(activation_request_identity_sha256)
    };
    if let Err(error) = activation_record_result {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            format!("ACTIVATE_MAP_RECORD_FAILED: {error}")
        );
    }
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::ActivationRecord,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_ACTIVATION_RECORD".to_owned()
        );
    }
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::ActivationRecord,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(final_activation_identity),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_RECORD_JOURNAL_CAS_FAILED".to_owned()
        );
    }

    {
        let mut shared = bridge.state.write().await;
        shared.map = map;
        if let Some((notification, _, _)) = cloud_commit.as_ref() {
            shared.cloud_map_revision = notification.revision;
        }
    }
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::SharedStateCommit,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_SHARED_STATE_COMMIT".to_owned()
        );
    }
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::SharedStateCommit,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_sha256),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_SHARED_STATE_JOURNAL_CAS_FAILED".to_owned()
        );
    }
    bridge.map_revision_latest_tx.send_replace(None);
    *bridge.occupancy_shm_current.write().await = None;
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::CacheClears,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_CACHE_CLEARS".to_owned()
        );
    }
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::CacheClears,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_sha256),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_CACHE_CLEAR_JOURNAL_CAS_FAILED".to_owned()
        );
    }
    let _ = bridge.map_revision_tx.send(event);
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::MapEvent,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "INJECTED_AFTER_MAP_EVENT".to_owned()
        );
    }
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::MapEvent,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_sha256),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_MAP_EVENT_JOURNAL_CAS_FAILED".to_owned()
        );
    }

    let target_receipt_sha256 =
        if let Some((notification, committed, target_receipt)) = cloud_commit {
            let _ = bridge.cloud_map_tx.send(notification);
            if take_activation_fault_after(
                &service,
                crate::correction_coordinator::ActivationJournalStep::CloudEvent,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_CLOUD_EVENT".to_owned()
                );
            }
            if !record_activation_journal(
                &service.activation_receipts,
                &token,
                activation_request_identity_sha256,
                crate::correction_coordinator::ActivationJournalStep::CloudEvent,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_sha256,
                None,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "ACTIVATE_MAP_CLOUD_EVENT_JOURNAL_CAS_FAILED".to_owned()
                );
            }
            let retained_publication = match RetainedActivationGlobalMap::from_committed(
                activation_request_identity_sha256,
                &expected_map_frame_id,
                &committed,
            ) {
                Ok(retained) => retained,
                Err(error) => {
                    settle_started_failure!(
                        ActivationReceiptDisposition::OutcomeUnknown,
                        format!("ACTIVATE_MAP_GLOBAL_MAP_FRAME_BUILD_FAILED: {error}")
                    );
                }
            };
            let publish_result =
                publish_committed_global_map_in_frame(&bridge, &committed, &expected_map_frame_id)
                    .await;
            if let Err(error) = publish_result {
                let detail = format!(
                "cloud-map checkpoint installed but typed /global_map publication failed: {error}"
            );
                match service.retain_activation_lifecycle_fence(
                    identity_class.request_id(),
                    activation_request_identity_sha256,
                    Some(retained_publication),
                    lifecycle_guard
                        .take()
                        .expect("ActivateMap publication retention owns lifecycle guard"),
                ) {
                    Ok(()) => return Err(Status::internal(detail)),
                    Err(guard) => {
                        lifecycle_guard = Some(guard);
                        settle_started_failure!(
                            ActivationReceiptDisposition::OutcomeUnknown,
                            format!("{detail}; lifecycle fence retention failed")
                        );
                    }
                }
            }
            if take_activation_fault_after(
                &service,
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "INJECTED_AFTER_TYPED_GLOBAL_MAP".to_owned()
                );
            }
            if !record_activation_journal(
                &service.activation_receipts,
                &token,
                activation_request_identity_sha256,
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_sha256,
                None,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "ACTIVATE_MAP_PUBLICATION_JOURNAL_CAS_FAILED".to_owned()
                );
            }
            Some(target_receipt)
        } else {
            #[cfg(test)]
            {
                let barrier = service.start_mapping_before_cloud_clear.lock().take();
                if let Some(barrier) = barrier {
                    barrier.wait().await;
                    barrier.wait().await;
                }
            }
            let clear = bridge
                .begin_cloud_map_session(started.session_generation)
                .await;
            if let CloudMapSessionAdmission::ClearRejected(detail)
            | CloudMapSessionAdmission::AdmissionClosed(detail) = clear
            {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    format!("ACTIVATE_MAP_CLOUD_CLEAR_ADMISSION_FAILED: {detail}")
                );
            }
            None
        };

    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_identity_sha256,
        crate::correction_coordinator::ActivationJournalStep::Seal,
        crate::correction_coordinator::ActivationJournalStatus::Prepared,
        Some(final_activation_identity),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_SEAL_PREPARE_CAS_FAILED".to_owned()
        );
    }
    let snapshot = adapter.lock().snapshot();
    let activation_message = if mapping_backend_checkpoint.is_some() {
        "map checkpoint activated; legacy_checkpoint_reconstructed=false"
    } else {
        "map checkpoint activated; legacy_checkpoint_reconstructed=true"
    };
    let state = mapping_session_state(snapshot, activation_message);
    let target_receipt = identity_class
        .is_modern()
        .then_some(target_receipt_sha256)
        .flatten();
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
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::Seal,
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

async fn finish_deferred_mapping_start_failure(
    replay: &crate::lio_ingress::NativeDeferredReplayControl,
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
    started: &MappingSessionSnapshot,
    occupancy_fence: Option<OccupancySessionFenceRollback>,
    detail: impl Into<String>,
) -> Status {
    let canonical_detail = replay.fail_generation(detail);
    let outcome = finish_failed_mapping_start_rollback(
        adapter,
        lifecycle_lock,
        lifecycle_guard,
        started,
        occupancy_fence,
        canonical_detail,
    )
    .await;
    for diagnostic in &outcome.diagnostics {
        tracing::error!(%diagnostic, "deferred StartMapping rollback diagnostic");
    }
    replay.record_poison_diagnostics(outcome.diagnostics);
    Status::unavailable(outcome.canonical_detail)
}

fn decode_checkpoint_grid(
    checkpoint: OccupancyGridData,
    expected_map_frame_id: &str,
) -> Result<nuwax_mapping::OccupancyGrid, String> {
    let meta = checkpoint
        .meta
        .ok_or_else(|| "checkpoint.meta is required".to_string())?;
    if meta.width == 0 || meta.height == 0 {
        return Err("checkpoint width/height must be positive".to_string());
    }
    let expected_size = (meta.width as usize)
        .checked_mul(meta.height as usize)
        .ok_or_else(|| "checkpoint dimensions overflow".to_string())?;
    if checkpoint.data.len() != expected_size {
        return Err(format!(
            "checkpoint data size mismatch: expected={} actual={}",
            expected_size,
            checkpoint.data.len()
        ));
    }
    if !meta.resolution.is_finite() || meta.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_string());
    }
    let frame_id = meta.frame_id.trim().trim_start_matches('/');
    if frame_id != expected_map_frame_id {
        return Err(format!(
            "checkpoint frame must be {expected_map_frame_id}, received {}",
            meta.frame_id,
        ));
    }
    let data = checkpoint
        .data
        .into_iter()
        .map(|value| value as i8)
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

fn cloud_checkpoint_proto(
    snapshot: crate::pointcloud::CommittedCloudMapSnapshot,
    frame_id: String,
) -> CloudMapCheckpoint {
    CloudMapCheckpoint {
        frame_id,
        source_revision: snapshot.revision,
        point_count: snapshot.point_count,
        voxel_size: snapshot.voxel_size,
        xyz_sha256: crate::map_checksum::sha256_hex(&snapshot.xyz_f32_le),
        xyz_f32_le: snapshot.xyz_f32_le,
        payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
    }
}

fn validate_mapping_checkpoint_envelope_size(
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

fn validate_cloud_checkpoint_metadata(
    checkpoint: &CloudMapCheckpoint,
    expected_map_frame_id: &str,
) -> Result<(), String> {
    let frame_id = checkpoint.frame_id.trim().trim_start_matches('/');
    if frame_id != expected_map_frame_id {
        return Err(format!(
            "cloud-map checkpoint frame must be {expected_map_frame_id}, received {}",
            checkpoint.frame_id,
        ));
    }
    if checkpoint.payload_schema_version != crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION {
        return Err(format!(
            "unsupported cloud-map checkpoint schema: expected={} actual={}",
            crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            checkpoint.payload_schema_version
        ));
    }
    let checksum = crate::pointcloud::decode_checkpoint_sha256(&checkpoint.xyz_sha256)?;
    if crate::map_checksum::sha256_bytes(&checkpoint.xyz_f32_le) != checksum {
        return Err("cloud-map checkpoint xyz_sha256 mismatch".to_string());
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

fn validate_mapping_commit_identity<'a>(
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

fn validate_mapping_checkpoint_receipts(
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

fn validate_mapping_backend_checkpoint(
    checkpoint: &MappingBackendCheckpoint,
    expected_backend: crate::mapping_backend::MappingBackendKind,
) -> Result<(), String> {
    if checkpoint.schema_version
        != crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION
    {
        return Err(format!(
            "unsupported mapping backend checkpoint schema: expected={} actual={}",
            crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
            checkpoint.schema_version
        ));
    }
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
    if checkpoint.content_sha256.len() != 32
        || checkpoint.content_sha256 != crate::map_checksum::sha256_bytes(&checkpoint.payload)
    {
        return Err("mapping backend checkpoint content_sha256 mismatch".to_owned());
    }
    let canonical_occupancy_checkpoint = checkpoint
        .canonical_occupancy_checkpoint
        .as_ref()
        .ok_or_else(|| {
            "mapping backend checkpoint canonical_occupancy_checkpoint is required".to_owned()
        })?;
    let canonical_cloud_checkpoint =
        checkpoint
            .canonical_cloud_checkpoint
            .as_ref()
            .ok_or_else(|| {
                "mapping backend checkpoint canonical_cloud_checkpoint is required".to_owned()
            })?;
    validate_mapping_checkpoint_bundle(
        checkpoint,
        canonical_occupancy_checkpoint,
        Some(canonical_cloud_checkpoint),
    )?;
    Ok(())
}

fn validate_mapping_checkpoint_bundle(
    checkpoint: &MappingBackendCheckpoint,
    occupancy_checkpoint: &OccupancyGridData,
    cloud_checkpoint: Option<&CloudMapCheckpoint>,
) -> Result<(), String> {
    let occupancy_receipt = checkpoint
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint occupancy_receipt is required".to_owned())?;
    let cloud_receipt = checkpoint
        .cloud_receipt
        .as_ref()
        .ok_or_else(|| "mapping backend checkpoint cloud_receipt is required".to_owned())?;
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
    let cloud_checkpoint = cloud_checkpoint.ok_or_else(|| {
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
    let canonical_occupancy_checkpoint = checkpoint
        .canonical_occupancy_checkpoint
        .as_ref()
        .ok_or_else(|| {
            "mapping backend checkpoint canonical_occupancy_checkpoint is required".to_owned()
        })?;
    let canonical_cloud_checkpoint =
        checkpoint
            .canonical_cloud_checkpoint
            .as_ref()
            .ok_or_else(|| {
                "mapping backend checkpoint canonical_cloud_checkpoint is required".to_owned()
            })?;
    if occupancy_checkpoint != canonical_occupancy_checkpoint
        || cloud_checkpoint != canonical_cloud_checkpoint
    {
        return Err(
            "mapping backend checkpoint canonical assets differ from activation payloads"
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
        goal_payload_sha256: lease.goal_payload_sha256.to_vec(),
    });
    let global_stop_reconciliation = snapshot.global_stop_reconciliation;
    let global_stop_reconciliation = Some(ExplorationGlobalStopReconciliation {
        eligible: global_stop_reconciliation.eligible,
        cancel_generation: global_stop_reconciliation.cancel_generation,
        recovery_correlation: global_stop_reconciliation.recovery_correlation,
        admission_guard_generation: global_stop_reconciliation
            .motion_admission
            .map_or(0, |proof| proof.guard_generation),
        admission_guard_boot_nonce: global_stop_reconciliation
            .motion_admission
            .map_or_else(Vec::new, |proof| proof.guard_boot_nonce.to_vec()),
        admission_motion_stop_generation: global_stop_reconciliation
            .motion_admission
            .map_or(0, |proof| proof.motion_stop_generation),
    });
    let ownership_reconciliation_receipt =
        snapshot.ownership_reconciliation_receipt.map(|receipt| {
            ExplorationOwnershipReconciliationReceipt {
                cause: receipt.cause,
                recovery_correlation: receipt.recovery_correlation,
                parent_request_id: receipt.parent_request_id,
                exploration_generation: receipt.exploration_generation,
                child_lease_fingerprint: receipt.child_lease_fingerprint,
                cancel_generation: receipt.cancel_generation,
                commit_disposition: receipt.commit_disposition,
            }
        });
    ExplorationState {
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
        rollover_recovery_eligible: snapshot.rollover_recovery_eligible,
        global_stop_reconciliation,
        ownership_reconciliation_receipt,
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
        if request.boundary_frame_id.trim().trim_start_matches('/') != expected_map_frame_id {
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
            request.motion_guard_generation,
            &request.motion_guard_boot_nonce,
            request.motion_stop_generation,
        )?,
        boundary,
        stop_conditions,
        max_navigation_retries: request.max_navigation_retries,
        blacklist_radius_m: request.blacklist_radius_m,
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
    if event.frame_id.trim().trim_start_matches('/') != expected_map_frame_id {
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
    if crate::map_checksum::sha256_hex(event.data.as_ref()) != event.checksum {
        return tx
            .send(Err(Status::data_loss(format!(
                "map revision {} checksum mismatch",
                event.revision
            ))))
            .await
            .is_ok();
    }
    if event.raw_data.is_empty() && event.raw_checksum != event.checksum {
        return tx
            .send(Err(Status::data_loss(format!(
                "raw map revision {} omitted data without an equivalent checksum",
                event.revision
            ))))
            .await
            .is_ok();
    }
    if !event.raw_data.is_empty() && event.raw_data.len() as u64 != event.size {
        return tx
            .send(Err(Status::data_loss(format!(
                "raw map revision {} size mismatch: expected={}, actual={}",
                event.revision,
                event.size,
                event.raw_data.len()
            ))))
            .await
            .is_ok();
    }
    if !event.raw_data.is_empty()
        && crate::map_checksum::sha256_hex(event.raw_data.as_ref()) != event.raw_checksum
    {
        return tx
            .send(Err(Status::data_loss(format!(
                "raw map revision {} checksum mismatch",
                event.revision
            ))))
            .await
            .is_ok();
    }

    let shm_ref = match occupancy_shm_current.read().await.clone() {
        Some(snapshot) if snapshot.revision == event.revision => {
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
        checksum: event.checksum,
        last_seen_ms: event.last_seen_ms,
        session_id: event.session_id,
        session_generation: event.session_generation,
        raw_data: event.raw_data.as_ref().to_vec(),
        raw_checksum: event.raw_checksum,
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
    let snapshot = mapping_adapter.lock().snapshot();
    snapshot.active
        && event.session_generation > 0
        && event.session_generation == snapshot.session_generation
        && event.session_id == snapshot.session_id
        && event.map_name == snapshot.map_name
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
        || snapshot.layout_version != 2
        || snapshot.payload_schema_version != 1
        || snapshot.generation == 0
        || snapshot.active_slot >= 2
        || snapshot.payload_size == 0
        || snapshot.payload_size >= snapshot.file_size
    {
        return Err(Box::new(Status::failed_precondition(format!(
            "invalid occupancy SHM v2 reference: layout={} schema={} generation={} slot={} file_size={} payload_size={} revision={}",
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
            "invalid cloud-map SHM v2 reference: revision={} generation={}",
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

fn encode_pointcloud_request_payload(request: &PublishPointCloudRequest) -> CodecResult<Vec<u8>> {
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
            seq: 0,
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
        guard_generation: feedback.guard_generation,
        sink_ready: feedback.sink_ready,
        planner_map_frame: feedback.planner_map_frame,
        planner_pose_frame: feedback.planner_pose_frame,
        zero_ack_id: feedback.zero_ack_id,
        zero_applied_monotonic_ns: feedback.zero_applied_monotonic_ns,
        owner_kind: feedback.owner_kind,
        parent_request_id: feedback.parent_request_id,
        goal_generation: feedback.goal_generation,
        path_revision: feedback.path_revision,
        goal_payload_sha256: feedback.goal_payload_sha256.to_vec(),
        path_payload_sha256: feedback.path_payload_sha256.to_vec(),
        producer_runtime_generation,
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
        let lifecycle_guard = match tokio::time::timeout(STOP_LIFECYCLE_TIMEOUT, async {
            let guard = match self.take_any_activation_lifecycle_fence() {
                Some(guard) => guard,
                None => {
                    self.bridge
                        .mapping_lifecycle_lock
                        .clone()
                        .lock_owned()
                        .await
                }
            };
            shutdown.wait_for_correction_settlement().await;
            guard
        })
        .await
        {
            Ok(guard) => guard,
            Err(_) => {
                let error = Status::deadline_exceeded(
                    "timed out acquiring mapping lifecycle lock or settling correction during process Stop",
                );
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
        };

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
                        let mut state = self.state.write().await;
                        state.phase = "failed".into();
                        state.message = error.message().to_string();
                        return Err(error);
                    }
                    Err(_) => {
                        let error = Status::deadline_exceeded(
                            "timed out shutting down exploration scheduler during stop recovery",
                        );
                        let mut state = self.state.write().await;
                        state.phase = "failed".into();
                        state.message = error.message().to_string();
                        return Err(error);
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
                        let mut state = self.state.write().await;
                        state.phase = "failed".into();
                        state.message = error.message().to_string();
                        return Err(error);
                    }
                }
            }
        };
        let safety_wait = tokio::time::timeout(STOP_EXPLORATION_SAFETY_TIMEOUT, async {
            let mut snapshot = stop_snapshot;
            loop {
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
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
            Err(_) => {
                let snapshot = self.bridge.explore.snapshot();
                let error = Status::deadline_exceeded(format!(
                    "timed out waiting for exploration safety ownership release: request_id={} child_task_id={} status={}",
                    snapshot.request_id, snapshot.child_task_id, snapshot.status
                ));
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
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
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
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
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
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
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
                Err(_) => {
                    let cancel = self.bridge.nav.cancel_snapshot();
                    let error = Status::deadline_exceeded(format!(
                        "timed out waiting for navigation safety ownership release: goal_id={} cancel_generation={} phase={:?}",
                        active_lease.session_id, cancel.generation, cancel.phase
                    ));
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
            }
        }

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

    type SubscribePoseStream = tokio_stream::wrappers::ReceiverStream<Result<PoseUpdate, Status>>;
    async fn subscribe_pose(
        &self,
        _req: Request<PoseRequest>,
    ) -> Result<Response<Self::SubscribePoseStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(32);
        let state = self.bridge.state.clone();
        tokio::spawn(async move {
            let mut seq = 0u64;
            // 变化驱动：仅当位姿实际更新（timestamp_ns 变化）时才推送帧；
            // 避免静止时以 100ms 周期推送大量重复帧。last_stamp 初值 u64::MAX
            // 确保首个有效帧（含 stamp==0 的早期态）至少推送一次后再去重。
            let mut last_stamp = u64::MAX;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                let update = {
                    let s = state.read().await;
                    if s.pose.timestamp_ns == last_stamp {
                        continue;
                    }
                    last_stamp = s.pose.timestamp_ns;
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
                if tx.send(Ok(update)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
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
        if self.has_activation_lifecycle_fence() {
            return Err(Status::unavailable("ACTIVATE_MAP_STAGED_LIFECYCLE_FENCE"));
        }
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        if self.deferred_replay.is_some() {
            self.shutdown.as_ref().ok_or_else(|| {
                Status::failed_precondition(
                    "deferred Mapping requires a process shutdown coordinator",
                )
            })?;
        }
        if let Some(detail) = self
            .deferred_replay
            .as_ref()
            .and_then(|deferred_replay| deferred_replay.poison_detail())
        {
            return Err(Status::unavailable(detail));
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
            if let Some(deferred_replay) = &self.deferred_replay {
                let map_frame_generation = adapter.lock().global_frame_generation();
                if deferred_replay.is_complete_for(
                    &request.session_id,
                    &request.map_name,
                    snapshot.session_generation,
                    map_frame_generation,
                ) {
                    return Ok(Response::new(self.mapping_state_with_receipt(
                        snapshot,
                        "mapping session already active; deferred startup replay complete",
                    )));
                }
                let detail = deferred_replay.poison_detail().unwrap_or_else(|| {
                    "active Mapping identity has no sealed deferred replay completion".to_owned()
                });
                return Err(Status::unavailable(detail));
            }
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

        let lifecycle_guard = match self.take_any_activation_lifecycle_fence() {
            Some(guard) => guard,
            None => {
                self.bridge
                    .mapping_lifecycle_lock
                    .clone()
                    .lock_owned()
                    .await
            }
        };
        self.ensure_running_admission("StartMapping").await?;

        self.activation_receipts
            .retire_for_mapping_change()
            .map_err(Status::resource_exhausted)?;

        let snapshot = adapter
            .lock()
            .start_session(&request.session_id, &request.map_name)
            .map_err(mapping_lifecycle_status)?;
        let (cancellation_tx, cancellation_rx) = watch::channel(false);
        let mut waiter = StartMappingRpcWaiter::new(cancellation_tx);
        let transition_owner = self.clone();
        let transition_task = tokio::spawn(async move {
            transition_owner
                .finish_started_mapping_transition(
                    request,
                    adapter,
                    snapshot,
                    cloud_map_reservation,
                    lifecycle_guard,
                    mapping_transition,
                    cancellation_rx,
                )
                .await
        });
        let result = transition_task.await.map_err(|error| {
            Status::internal(format!(
                "owned StartMapping transition task failed: {error}"
            ))
        });
        waiter.complete();
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
        let has_staged_fence = self.has_activation_lifecycle_fence();
        if has_staged_fence {
            let current = adapter.lock().snapshot();
            let requested_session_id = request.session_id.trim();
            let validation = if requested_session_id.is_empty() {
                Err(
                    crate::mapping_adapter::MappingLifecycleError::InvalidArgument(
                        "session_id must not be empty".to_owned(),
                    ),
                )
            } else if current.session_id != requested_session_id {
                Err(crate::mapping_adapter::MappingLifecycleError::StaleSession(
                    format!(
                        "requested={requested_session_id} current={}",
                        current.session_id
                    ),
                ))
            } else if !current.active {
                Err(crate::mapping_adapter::MappingLifecycleError::StaleSession(
                    format!(
                        "requested={requested_session_id} current_phase={}",
                        current.phase.as_str()
                    ),
                ))
            } else {
                Ok(())
            };
            validation.map_err(mapping_lifecycle_status)?;
            if self.bridge.explore.blocks_mapping_mutation().await {
                let exploration = self.bridge.explore.snapshot();
                return Err(Status::failed_precondition(format!(
                    "exploration safety ownership must be released before StopMapping: status={} request_id={} child_task_id={}",
                    exploration.status, exploration.request_id, exploration.child_task_id
                )));
            }
        }
        let mut staged_fence = self.take_activation_lifecycle_fence();
        let acquired_lifecycle_guard = if staged_fence.is_none() {
            Some(
                self.bridge
                    .mapping_lifecycle_lock
                    .clone()
                    .lock_owned()
                    .await,
            )
        } else {
            None
        };
        if staged_fence.is_none() && self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before StopMapping: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        if let Err(error) = self.ensure_running_admission("StopMapping").await {
            if let Some(fence) = staged_fence.take() {
                self.restore_activation_lifecycle_fence(fence);
            }
            return Err(error);
        }
        let transition = match adapter.lock().stop_session_detached(&request.session_id) {
            Ok(transition) => transition,
            Err(error) => {
                if let Some(fence) = staged_fence.take() {
                    self.restore_activation_lifecycle_fence(fence);
                }
                return Err(mapping_lifecycle_status(error));
            }
        };
        let lifecycle_guard = staged_fence
            .take()
            .map(|fence| fence.lifecycle_guard)
            .or(acquired_lifecycle_guard)
            .expect("StopMapping owns one lifecycle guard after preflight");
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
        drop(lifecycle_guard);
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

    async fn export_mapping_backend_checkpoint(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingBackendCheckpoint>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
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
                let lifecycle_guard = bridge.mapping_lifecycle_lock.lock().await;
                let committed = adapter
                    .lock()
                    .commit_backend_completion(completion, &bridge.state)
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
                let current_session = adapter.lock().snapshot();
                let (cloud_accepted, cloud_commit) =
                    accept_mapping_cloud_map_for_backend_candidate(
                        &accepted_map_points,
                        &event,
                        &occupancy_receipt,
                        session_generation,
                        source_timestamp_ns,
                        &current_session,
                        &bridge.cloud_map_writer,
                    );
                let resolution_result = adapter
                    .lock()
                    .resolve_backend_completion(&token, resolution, cloud_accepted);
                if !cloud_accepted || resolution_result.is_err() {
                    let mut reason = if !cloud_accepted {
                        "mapping backend checkpoint flush cloud-map acceptance failed".to_owned()
                    } else {
                        format!(
                            "mapping backend checkpoint flush resolution failed: {}",
                            resolution_result.expect_err("resolution failure checked above")
                        )
                    };
                    let retirement = adapter
                        .lock()
                        .stop_session_detached(&token.session_id)
                        .ok()
                        .and_then(|transition| transition.retirement);
                    drop(lifecycle_guard);
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
                let (cloud_snapshot, cloud_receipt) = {
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
                    (snapshot, receipt)
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
                validate_cloud_checkpoint_metadata(
                    &canonical_cloud_checkpoint,
                    raw_map.frame_id.trim().trim_start_matches('/'),
                )
                .map_err(Status::internal)?;
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
                let checkpoint = MappingBackendCheckpoint {
                    schema_version:
                        crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
                    backend_kind: backend_kind.stable_name().to_owned(),
                    source_session_generation: snapshot.session_generation,
                    content_sha256: crate::map_checksum::sha256_bytes(&payload).to_vec(),
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
                };
                validate_mapping_backend_checkpoint(&checkpoint, backend_kind)
                    .map_err(Status::internal)?;
                validate_mapping_checkpoint_envelope_size(
                    "mapping backend checkpoint response",
                    checkpoint.encoded_len(),
                )
                .map_err(Status::resource_exhausted)?;
                return Ok(checkpoint);
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
        let ingress = self
            .relocalization_ingress
            .as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let lifecycle_guard = self
            .bridge
            .mapping_lifecycle_lock
            .clone()
            .lock_owned()
            .await;
        let waiter: crate::correction_coordinator::RelocalizationCommandWaiter = ingress
            .admit_with_lifecycle_guard(req.into_inner(), lifecycle_guard)
            .await
            .map_err(relocalization_command_status)?;
        waiter
            .wait()
            .await
            .map(Response::new)
            .map_err(relocalization_command_status)
    }

    async fn get_relocalization_terminal(
        &self,
        req: Request<GetRelocalizationTerminalRequest>,
    ) -> Result<Response<GetRelocalizationTerminalResponse>, Status> {
        let ingress = self
            .relocalization_ingress
            .as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let request = req.into_inner();
        if request.schema_version != 1
            || crate::correction_coordinator::validate_relocalization_request_id(
                &request.request_id,
            )
            .is_err()
        {
            return Err(Status::invalid_argument(
                "RELOCALIZATION_TERMINAL_QUERY_INVALID",
            ));
        }
        let request_identity_sha256: [u8; 32] = request
            .request_identity_sha256
            .as_slice()
            .try_into()
            .map_err(|_| Status::invalid_argument("RELOCALIZATION_REQUEST_IDENTITY_INVALID"))?;
        let activation_identity_sha256: [u8; 32] = request
            .activation_identity_sha256
            .as_slice()
            .try_into()
            .map_err(|_| Status::invalid_argument("RELOCALIZATION_ACTIVATION_IDENTITY_INVALID"))?;
        let snapshot = ingress
            .terminal_snapshot(
                &request.request_id,
                request_identity_sha256,
                activation_identity_sha256,
            )
            .await
            .map_err(Status::failed_precondition)?;
        let ledger_observation = Some(snapshot.ledger_observation);
        let (status, terminal) = match snapshot.terminal {
            crate::correction_coordinator::RelocalizationTerminalSnapshot::Terminal(terminal) => {
                (RelocalizationTerminalStatus::Terminal, Some(terminal))
            }
            crate::correction_coordinator::RelocalizationTerminalSnapshot::Pending => {
                (RelocalizationTerminalStatus::Pending, None)
            }
            crate::correction_coordinator::RelocalizationTerminalSnapshot::NotFound => {
                (RelocalizationTerminalStatus::NotFound, None)
            }
            crate::correction_coordinator::RelocalizationTerminalSnapshot::IdentityConflict => {
                (RelocalizationTerminalStatus::IdentityConflict, None)
            }
        };
        Ok(Response::new(GetRelocalizationTerminalResponse {
            schema_version: 1,
            request_id: request.request_id,
            request_identity_sha256: request.request_identity_sha256,
            activation_identity_sha256: request.activation_identity_sha256,
            status: status as i32,
            terminal,
            ledger_observation,
        }))
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
            data: map.data.iter().map(|&v| v as u8).collect(),
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
            data: raw_map.data.iter().map(|&value| value as u8).collect(),
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

    type SubscribeMapUpdatesStream =
        tokio_stream::wrappers::ReceiverStream<Result<MapUpdateNotification, Status>>;
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
        tokio::spawn(async move {
            let mut last_generation = 0u64;
            let mut last_rev = 0u64;
            let initial = map_revision_rx.borrow_and_update().clone();
            if let Some(event) = initial {
                if !map_revision_claim_is_current(&event, &mapping_adapter, &mapping_lifecycle_lock)
                    .await
                {
                    return;
                }
                last_generation = event.session_generation;
                last_rev = event.revision;
                if !forward_map_revision(&tx, event, &occupancy_shm_current, &expected_map_frame_id)
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
                    || (event.session_generation == last_generation && event.revision <= last_rev)
                {
                    continue;
                }
                if !map_revision_claim_is_current(&event, &mapping_adapter, &mapping_lifecycle_lock)
                    .await
                {
                    continue;
                }
                last_generation = event.session_generation;
                last_rev = event.revision;
                if !forward_map_revision(&tx, event, &occupancy_shm_current, &expected_map_frame_id)
                    .await
                {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    type SubscribeCloudMapUpdatesStream =
        tokio_stream::wrappers::ReceiverStream<Result<CloudMapShmRef, Status>>;
    async fn subscribe_cloud_map_updates(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeCloudMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.cloud_map_tx.subscribe();
        tokio::spawn(async move {
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
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
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
            ng.motion_guard_generation,
            &ng.motion_guard_boot_nonce,
            ng.motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let input = SetGoalInput {
            goal_id: goal_id.clone(),
            owner_kind,
            parent_request_id,
            goal_generation: ng.goal_generation,
            motion_admission,
            frame_id,
            target,
        };
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("SetNavigationGoal").await?;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "public navigation cannot start while exploration owns admission: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        let SetGoalOutcome {
            state: nav, lease, ..
        } = set_goal(&self.bridge, &self.bridge.nav, input).await;
        // 同步到 SharedState.nav_state,供 SubscribeNavigationState 推送。
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
        let _ = self.bridge.nav_state_tx.send(update.clone());
        Ok(Response::new(nav_state_to_proto(update)))
    }

    async fn cancel_navigation(
        &self,
        req: Request<CancelNavigationRequest>,
    ) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime.cancel。流程:
        // 1) publish /nav/cancel typed frame
        // 2) best-effort publish zero cmd_vel
        // 3) 返回 PENDING；只有观测到 stack /nav/state 才进入 terminal CANCELLED
        let request = req.get_ref();
        let CancelOutcome { state: nav, lease } = nav_cancel(
            &self.bridge,
            &self.bridge.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: request.goal_id.trim().to_string(),
                owner_kind: request.owner_kind,
                parent_request_id: request.parent_request_id.trim().to_string(),
                goal_generation: request.goal_generation,
                producer_runtime_generation: request.producer_runtime_generation,
            },
        )
        .await;
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
        let _ = self.bridge.nav_state_tx.send(update.clone());
        Ok(Response::new(nav_state_to_proto(update)))
    }

    type SubscribeNavigationStateStream =
        tokio_stream::wrappers::ReceiverStream<Result<NavigationState, Status>>;
    async fn subscribe_navigation_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeNavigationStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut broadcast_rx = self.bridge.nav_state_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(update) = broadcast_rx.recv().await {
                let state = nav_state_to_proto(update);
                if tx.send(Ok(state)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
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
        let (snapshot, lifecycle) = {
            let lcm_backend = self.bridge.lcm_backend_available;
            match &self.bridge.mapping_adapter {
                Some(arc) => {
                    let g = arc.lock();
                    (
                        MappingReadinessDetail::compute(Some(&*g), lcm_backend),
                        Some(g.snapshot()),
                    )
                }
                None => (MappingReadinessDetail::compute(None, lcm_backend), None),
            }
        };
        let message_json = serde_json::to_string(&serde_json::json!({
            "lcm_backend": snapshot.lcm_backend,
            "adapter_present": snapshot.adapter_present,
            "odom_count": snapshot.odom_count,
            "pointcloud_count": snapshot.pointcloud_count,
            "last_occupancy_revision": snapshot.last_occupancy_revision,
            "accumulated_point_count": snapshot.accumulated_point_count,
            "global_consumer_ready": snapshot.global_consumer_ready,
            "frame_generation": snapshot.frame_generation,
            "last_error": snapshot.last_error,
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

    type SubscribeFrontiersStream =
        tokio_stream::wrappers::ReceiverStream<Result<FrontierList, Status>>;
    async fn subscribe_frontiers(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeFrontiersStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.frontiers_tx.subscribe();
        let state_handle = self.bridge.state.clone();
        tokio::spawn(async move {
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
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
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

    async fn stop_exploration(
        &self,
        req: Request<StopExplorationRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let snapshot = self.bridge.explore.stop(&r.request_id).await;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn recover_exploration_after_navigation_rollover(
        &self,
        req: Request<RecoverExplorationAfterNavigationRolloverRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.into_inner();
        if r.observed_navigation_runtime_generation <= r.expected_navigation_runtime_generation {
            return Err(Status::invalid_argument(
                "observed navigation runtime generation must advance",
            ));
        }
        let child = r
            .child_lease
            .ok_or_else(|| Status::invalid_argument("child_lease is required"))?;
        let goal_payload_sha256: [u8; 32] = child
            .goal_payload_sha256
            .as_slice()
            .try_into()
            .map_err(|_| {
                Status::invalid_argument("child goal_payload_sha256 must contain 32 bytes")
            })?;
        let guard = r
            .guard_proof
            .ok_or_else(|| Status::invalid_argument("guard_proof is required"))?;
        let motion_admission = MotionAdmissionProof::try_from_wire(
            guard.guard_generation,
            &guard.guard_boot_nonce,
            guard.motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let request = crate::explore_runtime::ExplorationRolloverRecoveryRequest {
            request_id: r.request_id,
            exploration_generation: r.exploration_generation,
            child_lease: crate::explore_runtime::ChildNavigationLease {
                session_id: child.session_id,
                owner_kind: child.owner_kind,
                parent_request_id: child.parent_request_id,
                goal_generation: child.goal_generation,
                producer_runtime_generation: child.producer_runtime_generation,
                goal_payload_sha256,
            },
            expected_navigation_runtime_generation: r.expected_navigation_runtime_generation,
            observed_navigation_runtime_generation: r.observed_navigation_runtime_generation,
            guard_proof: crate::explore_runtime::ExplorationRecoveryGuardProof {
                stop_operation_id: guard.stop_operation_id.clone(),
                recovery_correlation: guard.recovery_correlation.clone(),
                motion_admission,
                zero_ack_id: guard.zero_ack_id.clone(),
                zero_applied_monotonic_ns: guard.zero_applied_monotonic_ns,
                recovery_fence_token: crate::explore_runtime::RecoveryFenceToken::from_wire(
                    guard.recovery_fence_token.clone(),
                ),
            },
        };
        if let Some(snapshot) = self
            .bridge
            .explore
            .replay_completed_rollover_recovery(&request)
            .await
            .map_err(|error| match error {
                crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                    Status::invalid_argument(message)
                }
                crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(message) => {
                    Status::failed_precondition(message)
                }
            })?
        {
            return Ok(Response::new(exploration_state_from_snapshot(snapshot)));
        }

        let readiness = compute_readiness(&self.bridge, &self.bridge.nav).await;
        if readiness.planner_generation != request.observed_navigation_runtime_generation {
            return Err(Status::failed_precondition(format!(
                "navigation runtime generation mismatch: current={} observed={}",
                readiness.planner_generation, request.observed_navigation_runtime_generation
            )));
        }

        let recovery_fence_token: [u8; 32] = guard
            .recovery_fence_token
            .as_slice()
            .try_into()
            .map_err(|_| {
                Status::invalid_argument("recovery_fence_token must contain exactly 32 bytes")
            })?;
        let held_proof = HeldRecoveryFenceProof {
            operation_id: guard.stop_operation_id,
            recovery_correlation: guard.recovery_correlation,
            generation: motion_admission.guard_generation,
            boot_nonce: motion_admission.guard_boot_nonce,
            motion_stop_generation: motion_admission.motion_stop_generation,
            zero: ZeroConfirmation {
                zero_ack_id: guard.zero_ack_id,
                applied_monotonic_ns: guard.zero_applied_monotonic_ns,
            },
            recovery_fence_token,
        };
        let socket_path = std::env::var("NUWAX_MOTION_GUARD_SOCKET")
            .ok()
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty())
            .unwrap_or_else(|| DEFAULT_UDS_PATH.to_string());
        let mut guard_client = MotionGuardClient::connect(&socket_path)
            .await
            .map_err(|error| {
                Status::unavailable(format!(
                    "motion guard recovery fence verification unavailable: {error}"
                ))
            })?;
        guard_client
            .verify_held_recovery_fence(&held_proof)
            .await
            .map_err(|error| {
                Status::failed_precondition(format!(
                    "motion guard recovery fence verification failed: {error}"
                ))
            })?;
        let snapshot = self
            .bridge
            .explore
            .recover_after_navigation_rollover(request)
            .await
            .map_err(|error| match error {
                crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                    Status::invalid_argument(message)
                }
                crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(message) => {
                    Status::failed_precondition(message)
                }
            })?;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn reconcile_exploration_after_global_stop(
        &self,
        req: Request<ReconcileExplorationAfterGlobalStopRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.into_inner();
        let child = r
            .child_lease
            .ok_or_else(|| Status::invalid_argument("child_lease is required"))?;
        let goal_payload_sha256: [u8; 32] = child
            .goal_payload_sha256
            .as_slice()
            .try_into()
            .map_err(|_| {
                Status::invalid_argument("child goal_payload_sha256 must contain 32 bytes")
            })?;
        let original_motion_admission = MotionAdmissionProof::try_from_wire(
            r.admission_guard_generation,
            &r.admission_guard_boot_nonce,
            r.admission_motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let held = r
            .held_guard_proof
            .ok_or_else(|| Status::invalid_argument("held_guard_proof is required"))?;
        let held_guard_boot_nonce: [u8; 16] =
            held.guard_boot_nonce.as_slice().try_into().map_err(|_| {
                Status::invalid_argument("held guard_boot_nonce must contain exactly 16 bytes")
            })?;
        let request = crate::explore_runtime::ExplorationGlobalStopReconciliationRequest {
            request_id: r.request_id,
            exploration_generation: r.exploration_generation,
            child_lease: crate::explore_runtime::ChildNavigationLease {
                session_id: child.session_id,
                owner_kind: child.owner_kind,
                parent_request_id: child.parent_request_id,
                goal_generation: child.goal_generation,
                producer_runtime_generation: child.producer_runtime_generation,
                goal_payload_sha256,
            },
            cancel_generation: r.cancel_generation,
            recovery_correlation: r.recovery_correlation,
            guard_proof: crate::explore_runtime::ExplorationGlobalStopGuardProof {
                stop_operation_id: held.stop_operation_id.clone(),
                recovery_correlation: held.recovery_correlation.clone(),
                original_motion_admission,
                held_guard_generation: held.guard_generation,
                held_guard_boot_nonce,
                held_motion_stop_generation: held.motion_stop_generation,
                zero_ack_id: held.zero_ack_id.clone(),
                zero_applied_monotonic_ns: held.zero_applied_monotonic_ns,
                recovery_fence_token: crate::explore_runtime::RecoveryFenceToken::from_wire(
                    held.recovery_fence_token.clone(),
                ),
            },
        };
        if let Some(snapshot) = self
            .bridge
            .explore
            .replay_completed_global_stop_reconciliation(&request)
            .await
            .map_err(|error| match error {
                crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                    Status::invalid_argument(message)
                }
                crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(message) => {
                    Status::failed_precondition(message)
                }
            })?
        {
            return Ok(Response::new(exploration_state_from_snapshot(snapshot)));
        }

        let recovery_fence_token: [u8; 32] = held
            .recovery_fence_token
            .as_slice()
            .try_into()
            .map_err(|_| {
                Status::invalid_argument("recovery_fence_token must contain exactly 32 bytes")
            })?;
        let held_proof = HeldRecoveryFenceProof {
            operation_id: held.stop_operation_id,
            recovery_correlation: held.recovery_correlation,
            generation: held.guard_generation,
            boot_nonce: held_guard_boot_nonce,
            motion_stop_generation: held.motion_stop_generation,
            zero: ZeroConfirmation {
                zero_ack_id: held.zero_ack_id,
                applied_monotonic_ns: held.zero_applied_monotonic_ns,
            },
            recovery_fence_token,
        };
        let socket_path = std::env::var("NUWAX_MOTION_GUARD_SOCKET")
            .ok()
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty())
            .unwrap_or_else(|| DEFAULT_UDS_PATH.to_string());
        let mut guard_client = MotionGuardClient::connect(&socket_path)
            .await
            .map_err(|error| {
                Status::unavailable(format!(
                    "motion guard recovery fence verification unavailable: {error}"
                ))
            })?;
        guard_client
            .verify_held_recovery_fence(&held_proof)
            .await
            .map_err(|error| {
                Status::failed_precondition(format!(
                    "motion guard recovery fence verification failed: {error}"
                ))
            })?;
        let snapshot = self
            .bridge
            .explore
            .reconcile_after_global_stop(request)
            .await
            .map_err(|error| match error {
                crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                    Status::invalid_argument(message)
                }
                crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(message) => {
                    Status::failed_precondition(message)
                }
            })?;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn acknowledge_exploration_ownership_reconciliation(
        &self,
        req: Request<AcknowledgeExplorationOwnershipReconciliationRequest>,
    ) -> Result<Response<AcknowledgeExplorationOwnershipReconciliationResult>, Status> {
        let request = req.into_inner();
        let acknowledged = self
            .bridge
            .explore
            .acknowledge_durable_reconciliation(
                &request.recovery_correlation,
                &request.child_lease_fingerprint,
            )
            .map_err(|error| match error {
                crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                    Status::invalid_argument(message)
                }
                crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(message) => {
                    Status::failed_precondition(message)
                }
            })?;
        Ok(Response::new(
            AcknowledgeExplorationOwnershipReconciliationResult { acknowledged },
        ))
    }

    async fn get_exploration_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationState>, Status> {
        let live_snapshot = self.bridge.explore.snapshot();
        let snapshot = if live_snapshot.request_id.is_empty() {
            self.bridge
                .explore
                .latest_durable_reconciliation_snapshot()
                .map_err(|error| match error {
                    crate::explore_runtime::ExplorationRecoveryError::InvalidArgument(message) => {
                        Status::invalid_argument(message)
                    }
                    crate::explore_runtime::ExplorationRecoveryError::FailedPrecondition(
                        message,
                    ) => Status::failed_precondition(message),
                })?
                .unwrap_or(live_snapshot)
        } else {
            live_snapshot
        };
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
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

    async fn publish_point_cloud(
        &self,
        req: Request<PublishPointCloudRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        let Some((topic, msg_type)) = self.point_cloud_binding.clone() else {
            return Ok(Response::new(PublishResult {
                success: false,
                message: "PointCloud2 RPC ingress is unavailable for the selected native LIO owner"
                    .to_string(),
            }));
        };
        // Phase 4:channel 走 channel_registry::channel_for(PointCloud)，
        // 不再硬编码 `/go2/lidar/points`。
        let pcr = req.get_ref();
        let payload = match encode_pointcloud_request_payload(pcr) {
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
        let lcm_transport = self.bridge.transport.clone();
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
        let event = MapRevisionEvent::from_map(
            &map,
            &handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot(),
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
        let mut malformed = canonical;
        malformed.data.pop();
        let event = MapRevisionEvent::from_map(
            &malformed,
            &handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot(),
        );
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
        }
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
mod map_update_tests {
    use super::*;
    use crate::lcm_runtime::{MapRevisionEvent, MapState};
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
        MapRevisionEvent::from_map(&map, &session)
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

        let event = test_mapping_event(MapState {
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
        assert_eq!(update.schema_version, 1);
        assert_eq!(update.checksum, event.checksum);
        assert!(update.raw_data.is_empty());
        assert_eq!(update.raw_checksum, event.checksum);
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
        assert_eq!(shm_ref.layout_version, 2);
        assert_eq!(shm_ref.payload_schema_version, 1);
        assert!(shm_ref.generation > 0);
        assert!(shm_ref.active_slot < 2);
        assert_eq!(shm_ref.payload_size, event.size);
        let internal = handle
            .occupancy_shm_current
            .read()
            .await
            .clone()
            .expect("v2 SHM metadata");
        assert_eq!(internal.layout_version, 2);
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
        let event = MapRevisionEvent::from_effective_and_raw(&effective, &raw, &session);
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
        assert_eq!(update.checksum, event.checksum);
        assert_eq!(update.raw_checksum, event.raw_checksum);
        assert_ne!(update.raw_checksum, update.checksum);
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
                    key: "/tmp/current-v2.mmap".to_string(),
                    file_size: 8192,
                    payload_size: event.size,
                    revision: current_revision,
                    layout_version: 2,
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
            assert_eq!(update.checksum, event.checksum);
            assert!(
                update.shm_ref.is_none(),
                "revision {current_revision} must not be attached to event revision {}",
                event.revision
            );
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
                key: "/tmp/current-v2.mmap".to_string(),
                file_size: 8192,
                payload_size: event.size,
                revision: event.revision,
                layout_version: 2,
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
        assert_eq!(update.checksum, event.checksum);
        let shm_ref = update.shm_ref.expect("matching SHM reference");
        assert_eq!(shm_ref.revision, event.revision);
        assert_eq!(shm_ref.size, 8192);
        assert_eq!(shm_ref.layout_version, 2);
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
        let event = MapRevisionEvent::from_map(
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
            &handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .snapshot(),
        );
        *handle.occupancy_shm_current.write().await =
            Some(crate::lcm_runtime::OccupancyShmSnapshot {
                key: "/tmp/post-commit-stop.mmap".to_string(),
                file_size: 8192,
                payload_size: event.size,
                revision: event.revision,
                layout_version: 2,
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
    fn occupancy_shm_v2_projection_rejects_old_or_inconsistent_references() {
        let valid = crate::lcm_runtime::OccupancyShmSnapshot {
            key: "/tmp/occupancy-v2.mmap".to_string(),
            file_size: 8192,
            payload_size: 256,
            revision: 7,
            layout_version: 2,
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

    fn assert_common_v2_reference(reference: &ShmRef, revision: u64, generation: u64) {
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

    #[tokio::test]
    async fn cloud_map_clear_update_carries_complete_v2_shm_reference() {
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
        let reference = update.shm_ref.expect("v2 SHM reference");
        assert_common_v2_reference(&reference, 1, 7);
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
        let reference = update.shm_ref.expect("v2 SHM reference");
        assert_common_v2_reference(&reference, 2, 8);
        assert_eq!(reference.active_slot, 1);
        assert_eq!(reference.payload_size, 36);
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use super::*;
    use crate::explore_runtime::{
        now_monotonic_nanos, now_unix_nanos, ChildNavigationCancelTicket, ChildNavigationLease,
        ChildNavigationRequest, ChildNavigationResult, ChildNavigationSubmitError,
        ChildNavigationTerminalProof, ChildNavigationTicket, ChildNavigationZeroProof,
        ExploreCostmapSnapshot, ExploreNavigationSink, ExplorePoseProvider, ExploreRobotPose,
        STATUS_ACCEPTED, STATUS_ACTIVE, STATUS_FAILED, STATUS_STOPPING,
    };
    use crate::lcm_runtime::{MapRevisionEvent, MapState};
    use crate::pointcloud::CloudMapShmWriter;
    use nalgebra::Point3;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
    use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
    use nuwax_transport::{
        LcmBackend, LcmFrameStream, LcmTransport, TransportError, TransportResult,
    };
    use std::sync::Mutex as StdMutex;
    use tokio_stream::StreamExt;

    fn exact_hex(bytes: &[u8]) -> String {
        bytes.iter().map(|byte| format!("{byte:02x}")).collect()
    }

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
        let retained_bytes = target_xyz_f32_le.len();
        let activation_request_identity_sha256 = [0x31; 32];
        let token = match service.activation_receipts.stage_or_replay(
            crate::correction_coordinator::ActivationReceiptSeed {
                activation_request_id: "relocalize-r1-gate".to_owned(),
                activation_request_identity_sha256,
                asset_manifest_sha256: Some([0x32; 32]),
                asset_bundle_sha256: Some([0x33; 32]),
                modern: true,
                immutable_target: Some(crate::pointcloud::ImmutableActivatedTarget {
                    frame_id: "map".to_owned(),
                    source_revision: 41,
                    point_count: 2,
                    voxel_size: crate::pointcloud::VOXEL_SIZE,
                    xyz_sha256: crate::map_checksum::sha256_bytes(&target_xyz_f32_le),
                    xyz_f32_le: target_xyz_f32_le,
                    payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
                }),
            },
            MappingSessionState::default(),
            true,
        ) {
            crate::correction_coordinator::ActivationLedgerAdmission::Execute(token) => token,
            crate::correction_coordinator::ActivationLedgerAdmission::Terminal(_) => {
                panic!("relocalize test receipt must stage")
            }
        };
        let receipt_epoch_before = service.activation_receipts.lifecycle_epoch();
        assert_eq!(
            service
                .activation_receipts
                .retained_immutable_target_bytes(),
            retained_bytes
        );
        let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        assert!(service
            .retain_activation_lifecycle_fence(
                "relocalize-r1-gate",
                activation_request_identity_sha256,
                None,
                lifecycle_guard,
            )
            .is_ok());

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
            GeometryRuntime::relocalize(&service, Request::new(RelocalizeRequest::default()))
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
            service
                .activation_receipts
                .retained_immutable_target_bytes(),
            retained_bytes
        );
        assert!(service.activation_receipts.validate_token(&token));
        assert!(service.has_activation_lifecycle_fence());
        assert!(handle
            .mapping_lifecycle_lock
            .clone()
            .try_lock_owned()
            .is_err());
        assert!(matches!(
            map_revision_rx.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));
        assert!(matches!(
            cloud_revision_rx.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));

        drop(service.take_any_activation_lifecycle_fence());
        assert!(handle
            .mapping_lifecycle_lock
            .clone()
            .try_lock_owned()
            .is_ok());
    }

    #[test]
    fn activation_identity_v1_preimages_and_hashes_match_approved_literals() {
        let occupancy = OccupancyGridData {
            meta: Some(OccupancyGridMeta {
                width: 2,
                height: 2,
                resolution: 0.05,
                origin_x: -1.25,
                origin_y: 2.5,
                frame_id: "map".to_owned(),
                revision: 7,
                timestamp_ns: 11,
            }),
            data: vec![0, 100, 255, 42],
        };
        let occupancy_preimage = canonical_occupancy_preimage(&occupancy).unwrap();
        assert_eq!(occupancy_preimage.len(), 134);
        assert_eq!(
            exact_hex(&occupancy_preimage),
            "6e757761782e67656f6d657472792e6f63637570616e63792d636865636b706f696e742e7631000000000100000002000000023d4ccccdbff40000000000004004000000000000000000036d61700000000000000007000000000000000b0000000000000004e7e4905f703bdc569cc35bebf90c9a4e03c990b290606a3c74e6317bc8a409f5"
        );
        let occupancy_sha256 = crate::map_checksum::sha256_bytes(&occupancy_preimage);
        assert_eq!(
            exact_hex(&occupancy_sha256),
            "1b2d37a8a728987c12ffd397a22db83ea59787e374594e27ec13d4af5397461f"
        );

        let xyz_f32_le = [1.0_f32, -2.0, 3.5]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        assert_eq!(
            crate::map_checksum::sha256_hex(&xyz_f32_le),
            "89e4869f4e44c5ae044a22108f669af1c86db9944003c8f2c07fdab3980d5938"
        );
        let padded_uppercase_digest = format!(
            " \t{}\r\n",
            crate::map_checksum::sha256_hex(&xyz_f32_le).to_ascii_uppercase()
        );
        let target = crate::pointcloud::decode_immutable_activated_target(
            "map",
            "map",
            9,
            1,
            0.1,
            &xyz_f32_le,
            &padded_uppercase_digest,
            crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
        )
        .unwrap();
        let cloud_preimage = canonical_cloud_preimage(&target).unwrap();
        assert_eq!(cloud_preimage.len(), 102);
        assert_eq!(
            exact_hex(&cloud_preimage),
            "6e757761782e67656f6d657472792e636c6f75642d636865636b706f696e742e763100000000036d61700000000000000009000000013dcccccd000000000000000c89e4869f4e44c5ae044a22108f669af1c86db9944003c8f2c07fdab3980d593800000001"
        );
        let cloud_sha256 = crate::map_checksum::sha256_bytes(&cloud_preimage);
        assert_eq!(
            exact_hex(&cloud_sha256),
            "28ab6334466329df9ccda3366bc49d68cd4af133617acc931160aef14514d54a"
        );

        let manifest = <[u8; 32]>::try_from((0_u8..32).collect::<Vec<_>>()).unwrap();
        let bundle_preimage =
            canonical_asset_bundle_preimage(manifest, occupancy_sha256, Some(cloud_sha256), None);
        assert_eq!(bundle_preimage.len(), 133);
        assert_eq!(
            exact_hex(&bundle_preimage),
            "6e757761782e67656f6d657472792e61737365742d62756e646c652e76310000000001000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f1b2d37a8a728987c12ffd397a22db83ea59787e374594e27ec13d4af5397461f0128ab6334466329df9ccda3366bc49d68cd4af133617acc931160aef14514d54a00"
        );
        let bundle_sha256 = crate::map_checksum::sha256_bytes(&bundle_preimage);
        assert_eq!(
            exact_hex(&bundle_sha256),
            "fad41779ffa7488a761e741427bab46c9eaf82875cdcb96951a1980b6e1256f4"
        );

        let class = ActivationRequestIdentityClass::Modern {
            request_id: "act-0001".to_owned(),
            asset_manifest_sha256: manifest,
            asset_bundle_sha256: bundle_sha256,
        };
        let request_preimage = canonical_activation_request_preimage(
            &class,
            7,
            "session-42",
            "地图-A",
            occupancy_sha256,
            Some(cloud_sha256),
            None,
        )
        .unwrap();
        assert_eq!(request_preimage.len(), 219);
        assert_eq!(
            exact_hex(&request_preimage),
            "6e757761782e67656f6d657472792e61637469766174652d6d61702e726571756573742e76310000000001000000086163742d3030303100000000000000070000000a73657373696f6e2d343200000008e59cb0e59bbe2d41000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1ffad41779ffa7488a761e741427bab46c9eaf82875cdcb96951a1980b6e1256f41b2d37a8a728987c12ffd397a22db83ea59787e374594e27ec13d4af5397461f0128ab6334466329df9ccda3366bc49d68cd4af133617acc931160aef14514d54a00"
        );
        let request_sha256 = crate::map_checksum::sha256_bytes(&request_preimage);
        assert_eq!(
            exact_hex(&request_sha256),
            "caabfd76b99dc93d85ec03607eb309f2397e6f7aff0a806ff82d0f0b4272a3d8"
        );

        let activation_preimage = canonical_activation_identity_preimage(
            request_sha256,
            7,
            11,
            13,
            "voxel",
            None,
            manifest,
            bundle_sha256,
        )
        .unwrap();
        assert_eq!(activation_preimage.len(), 278);
        assert_eq!(
            exact_hex(&activation_preimage),
            "6e757761782e67656f6d657472792e61637469766174652d6d61702e6964656e746974792e763100caabfd76b99dc93d85ec03607eb309f2397e6f7aff0a806ff82d0f0b4272a3d80000000000000007000000000000000b000000000000000d00000005766f78656c00000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1ffad41779ffa7488a761e741427bab46c9eaf82875cdcb96951a1980b6e1256f400000001b0e3c49266e73179cee77207a64680ee719e497886db18ea2267093c94a76aa800000001cf2793085656ef81fadf12a80fb1147973ec71a77735a4a3e600ac1345b59c7000000001f8b5ef8b359eeb0129f6d523244ea8bca35102143f3bce28e619598749276090"
        );
        let activation_sha256 = crate::map_checksum::sha256_bytes(&activation_preimage);
        assert_eq!(
            exact_hex(&activation_sha256),
            "64584d3a3b0f099f970e6fec1faf19053634383a400975126157667944c20a1f"
        );

        let receipt = crate::mapping_commit::CloudCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 0x0011_2233_4455_6677_8899_aabb_ccdd_eeff,
                sequence: 17,
            },
            session_generation: 11,
            occupancy_revision: 19,
            cloud_revision: 23,
            source_timestamp_ns: 29,
            point_count: 1,
        };
        let receipt_preimage = canonical_target_receipt_preimage(
            activation_sha256,
            cloud_sha256,
            9,
            23,
            11,
            13,
            &receipt,
        )
        .unwrap();
        assert_eq!(receipt_preimage.len(), 199);
        assert_eq!(
            exact_hex(&receipt_preimage),
            "6e757761782e67656f6d657472792e6163746976617465642d7461726765742d726563656970742e76310064584d3a3b0f099f970e6fec1faf19053634383a400975126157667944c20a1f28ab6334466329df9ccda3366bc49d68cd4af133617acc931160aef14514d54a00000000000000090000000000000017000000000000000b000000000000000d00112233445566778899aabbccddeeff0000000000000011000000000000000b00000000000000130000000000000017000000000000001d00000001"
        );
        assert_eq!(
            crate::map_checksum::sha256_hex(&receipt_preimage),
            "7fcaf2e50cc465960f59b4c8be143d9cae1441f30b60d30827a41372da176dce"
        );

        let mut zero_origin = occupancy.clone();
        zero_origin.meta.as_mut().unwrap().origin_x = 0.0;
        let mut negative_zero_origin = zero_origin.clone();
        negative_zero_origin.meta.as_mut().unwrap().origin_x = -0.0;
        assert_eq!(
            canonical_occupancy_preimage(&zero_origin).unwrap(),
            canonical_occupancy_preimage(&negative_zero_origin).unwrap()
        );
        let mut reordered = occupancy;
        reordered.data.swap(0, 1);
        assert_ne!(
            canonical_occupancy_sha256(&reordered).unwrap(),
            occupancy_sha256
        );
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

    #[derive(Clone)]
    struct DeferredClearPublishBackend {
        frames: Arc<StdMutex<Vec<RawLcmFrame>>>,
        frame_tx: tokio::sync::mpsc::UnboundedSender<RawLcmFrame>,
    }

    impl DeferredClearPublishBackend {
        fn new() -> (Self, tokio::sync::mpsc::UnboundedReceiver<RawLcmFrame>) {
            let (frame_tx, frame_rx) = tokio::sync::mpsc::unbounded_channel();
            (
                Self {
                    frames: Arc::new(StdMutex::new(Vec::new())),
                    frame_tx,
                },
                frame_rx,
            )
        }

        fn published_count(&self, message_type: &str) -> usize {
            self.frames
                .lock()
                .expect("deferred clear published frames")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == message_type)
                .count()
        }
    }

    #[async_trait::async_trait]
    impl LcmBackend for DeferredClearPublishBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.frames
                .lock()
                .expect("deferred clear published frames")
                .push(frame.clone());
            self.frame_tx.send(frame).map_err(|_| {
                TransportError::LcmBackend("deferred clear frame receiver closed".into())
            })
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "deferred-clear-publish-test"
        }
    }

    #[derive(Default)]
    struct DeferredClearCorrectionSink {
        apply_count: std::sync::atomic::AtomicU64,
    }

    #[async_trait::async_trait]
    impl crate::correction_coordinator::CorrectionCandidateSink for DeferredClearCorrectionSink {
        async fn apply_candidate(
            &self,
            _candidate: crate::correction_coordinator::CorrectionCandidate,
            _cancellation: crate::correction_coordinator::CorrectionRequestCancellation,
        ) -> Result<crate::correction_coordinator::CorrectionDisposition, String> {
            self.apply_count
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            Ok(crate::correction_coordinator::CorrectionDisposition::Applied)
        }

        async fn stop(&self) {}
    }

    struct DeferredClearFixture {
        handle: GeometryRuntimeHandle,
        backend: DeferredClearPublishBackend,
        frame_rx: tokio::sync::mpsc::UnboundedReceiver<RawLcmFrame>,
        public_odometry: (LcmTopic, LcmMessageType),
        gate: Arc<crate::lio_ingress::LioIngressRoutingGate>,
        replay: Arc<crate::lio_ingress::NativeDeferredReplayControl>,
        correction_sink: Arc<DeferredClearCorrectionSink>,
        correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
        correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
        correction_owner: Option<crate::correction_coordinator::CorrectionTransportOwner>,
        occupancy_path: Option<std::path::PathBuf>,
    }

    impl DeferredClearFixture {
        async fn new() -> Self {
            let native_spec = crate::lio_ingress::DeviceLioIngressSpec::new_for_backend(
                nuwax_codec::lcm::lio::LIO_BACKEND_FASTLIO2,
                71,
                "lidar",
                "base_link",
                "odom",
                "map",
            )
            .expect("deferred clear native spec");
            let body_from_sensor = crate::native_lio_owner::BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("deferred clear identity body-from-sensor");
            let relocalization_mode_fence =
                Arc::new(crate::correction_coordinator::RelocalizationModeFence::default());
            let probe = Arc::new(
                crate::lio_ingress::NativeLioIngress::without_runtime(
                    native_spec.clone(),
                    body_from_sensor.clone(),
                    Duration::from_millis(1),
                )
                .expect("deferred clear runtime-free probe"),
            );
            probe
                .bind_relocalization_mode_fence(Arc::clone(&relocalization_mode_fence))
                .expect("bind deferred clear probe fence before raw capture");

            let header = |frame_id: &str| nuwax_codec::lcm::schema::Header {
                seq: 1,
                stamp_sec: 1,
                stamp_nsec: 0,
                frame_id: frame_id.to_owned(),
            };
            let mut source_odometry = nuwax_codec::lcm::schema::Odometry {
                header: header("odom"),
                child_frame_id: "base_link".to_owned(),
                ..Default::default()
            };
            source_odometry.pose.position = [0.5, -0.25, 0.0];
            source_odometry.pose.orientation_xyzw[3] = 1.0;
            let points = [[0.25_f32, 0.25_f32, 0.0_f32], [0.75, 0.5, 0.0]];
            let mut cloud_data = Vec::with_capacity(points.len() * 12);
            for point in points {
                for coordinate in point {
                    cloud_data.extend_from_slice(&coordinate.to_le_bytes());
                }
            }
            let source_cloud = nuwax_codec::lcm::schema::PointCloud2 {
                header: header("lidar"),
                height: 1,
                width: 2,
                fields: [("x", 0), ("y", 4), ("z", 8)]
                    .into_iter()
                    .map(|(name, offset)| nuwax_codec::lcm::schema::PointField {
                        name: name.to_owned(),
                        offset,
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                is_bigendian: false,
                point_step: 12,
                row_step: 24,
                data: cloud_data,
                is_dense: true,
            };
            let encoded_odometry = nuwax_codec::lcm::schema::encode_odometry(&source_odometry)
                .expect("encode deferred clear odometry");
            let encoded_cloud = nuwax_codec::lcm::schema::encode_point_cloud2(&source_cloud)
                .expect("encode deferred clear cloud");
            let source_odometry_binding = (
                LcmTopic::new("/test/lifecycle/deferred/source-odometry")
                    .expect("deferred source odometry topic"),
                LcmMessageType::new("nav_msgs.Odometry").expect("deferred source odometry type"),
            );
            let source_cloud_binding = (
                LcmTopic::new("/test/lifecycle/deferred/source-cloud")
                    .expect("deferred source cloud topic"),
                LcmMessageType::new("sensor_msgs.PointCloud2").expect("deferred source cloud type"),
            );
            let source_frame = |binding: &(LcmTopic, LcmMessageType), payload: &[u8]| {
                nuwax_protocol::RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
                    .seqno(1)
                    .payload(payload.get(8..).expect("generated fingerprint").to_vec())
                    .build()
            };
            let (_probe_shutdown_tx, mut probe_shutdown_rx) = watch::channel(false);
            crate::lio_owner::LioIngressSink::accept_legacy_cloud(
                probe.as_ref(),
                source_frame(&source_cloud_binding, &encoded_cloud),
                &mut probe_shutdown_rx,
            )
            .await
            .expect("capture deferred clear raw cloud");
            crate::lio_owner::LioIngressSink::accept_legacy_odometry(
                probe.as_ref(),
                source_frame(&source_odometry_binding, &encoded_odometry),
                &mut probe_shutdown_rx,
            )
            .await
            .expect("seal deferred clear startup pair");
            let pair = probe
                .take_validated_startup_pair()
                .expect("transfer deferred clear startup pair");

            let mut handle = crate::lcm_runtime::build_handle_for_tests_without_cloud_writer()
                .await
                .expect("deferred clear Geometry handle");
            let occupancy_path = handle.occupancy_shm_path.clone().map(Into::into);
            let (backend, frame_rx) = DeferredClearPublishBackend::new();
            handle.transport = LcmTransport::new(Arc::new(backend.clone()));
            let public_odometry = (
                LcmTopic::new("/test/lifecycle/deferred/odometry")
                    .expect("deferred public odometry topic"),
                LcmMessageType::new("nuwax_msgs.LioOdometryEnvelope")
                    .expect("deferred public odometry type"),
            );
            let public_cloud = (
                LcmTopic::new("/test/lifecycle/deferred/cloud")
                    .expect("deferred public cloud topic"),
                LcmMessageType::new("nuwax_msgs.LioCloudEnvelope")
                    .expect("deferred public cloud type"),
            );
            let tf = (
                LcmTopic::new("/test/lifecycle/deferred/tf").expect("deferred TF topic"),
                LcmMessageType::new("tf2_msgs.TFMessage").expect("deferred TF type"),
            );
            let ingress = Arc::new(
                crate::lio_ingress::NativeLioIngress::with_runtime(
                    native_spec,
                    handle.clone(),
                    public_odometry.clone(),
                    public_cloud,
                    tf.clone(),
                    body_from_sensor,
                    Duration::from_millis(1),
                )
                .expect("deferred clear runtime ingress"),
            );
            let correction_identity =
                crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
                    runtime_generation: 71,
                    backend_generation: 71,
                    correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                        boot_nonce: 0x7171,
                        generation: 71,
                    },
                };
            let correction_applier = Arc::new(
                crate::lcm_runtime::GeometryCanonicalCorrectionApplier::new(
                    correction_identity.clone(),
                    "map",
                    "odom",
                    "base_link",
                    handle.state.clone(),
                    handle.transport.clone(),
                    tf,
                )
                .expect("deferred clear correction applier"),
            );
            let correction_sink = Arc::new(DeferredClearCorrectionSink::default());
            let correction_owner = crate::correction_coordinator::CorrectionTransportOwner::start_with_relocalization_mode_fence(
                crate::correction_coordinator::CorrectionOwnerSpec::None,
                correction_identity,
                "unused-by-none",
                correction_sink.clone(),
                relocalization_mode_fence,
            )
            .await
            .expect("deferred clear Correction owner");
            let correction_ingress = correction_owner.ingress();
            let correction_phase_a = correction_owner.phase_a_control();
            let active: Arc<dyn crate::lio_owner::LioIngressSink> =
                Arc::new(crate::lio_ingress::CorrectionAwareLioIngress::native(
                    ingress.clone(),
                    correction_ingress.clone(),
                    correction_applier.clone(),
                ));
            let probe_sink: Arc<dyn crate::lio_owner::LioIngressSink> = probe;
            let gate = Arc::new(crate::lio_ingress::LioIngressRoutingGate::new(probe_sink));
            let token = gate
                .bind_deferred(active)
                .expect("bind deferred clear replay gate");
            let replay = crate::lio_ingress::NativeDeferredReplayControl::new(
                ingress,
                gate.clone(),
                correction_ingress.clone(),
                correction_applier,
                correction_phase_a.clone(),
                token,
                pair,
            );

            Self {
                handle,
                backend,
                frame_rx,
                public_odometry,
                gate,
                replay,
                correction_sink,
                correction_ingress,
                correction_phase_a,
                correction_owner: Some(correction_owner),
                occupancy_path,
            }
        }

        async fn complete_startup_odometry(&mut self) {
            let replay = self.replay.clone();
            let publication = tokio::spawn(async move { replay.publish_startup_odometry().await });
            let frame = tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    let frame = self
                        .frame_rx
                        .recv()
                        .await
                        .expect("deferred clear publication frame");
                    if frame.topic == self.public_odometry.0
                        && frame.msg_type == self.public_odometry.1
                    {
                        return frame;
                    }
                }
            })
            .await
            .expect("deferred clear startup odometry frame deadline");
            let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
            crate::lio_owner::LioIngressSink::accept_public_odometry(
                self.gate.as_ref(),
                frame,
                &mut shutdown_rx,
            )
            .await
            .expect("deferred clear startup odometry callback");
            publication
                .await
                .expect("deferred clear startup odometry task")
                .expect("deferred clear startup odometry ACK");
        }

        async fn shutdown(mut self) {
            if let Some(owner) = self.correction_owner.take() {
                owner
                    .shutdown()
                    .await
                    .expect("deferred clear Correction shutdown");
            }
            if let Some(writer) = &self.handle.occupancy_shm {
                writer
                    .lock()
                    .cleanup_product_owned_file()
                    .expect("deferred clear occupancy cleanup");
            }
            if let Some(path) = self.occupancy_path {
                assert!(!path.exists());
            }
        }
    }

    fn install_deferred_cloud_writer(
        handle: &GeometryRuntimeHandle,
        label: &str,
    ) -> (std::path::PathBuf, watch::Receiver<u64>) {
        let path = std::env::temp_dir().join(format!(
            "nuwax-deferred-{label}-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let writer = crate::pointcloud::CloudMapShmWriter::new(&path)
            .expect("deferred test cloud-map writer");
        let generation_rx = writer.typed_publish_state().session_generation_rx();
        *handle.cloud_map_writer.lock() = Some(writer);
        (path, generation_rx)
    }

    fn cleanup_deferred_cloud_writer(handle: &GeometryRuntimeHandle, path: &std::path::Path) {
        let mut writer = handle
            .cloud_map_writer
            .lock()
            .take()
            .expect("installed deferred test cloud-map writer");
        writer
            .cleanup_product_owned_file()
            .expect("deferred test cloud-map cleanup");
        assert!(!path.exists());
    }

    static MOTION_GUARD_SOCKET_ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

    #[derive(Default)]
    struct CheckpointPublishBackend {
        fail: std::sync::atomic::AtomicBool,
        fail_after_observe: std::sync::atomic::AtomicBool,
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
            self.published
                .lock()
                .expect("checkpoint published frames")
                .push(frame);
            if self
                .fail_after_observe
                .load(std::sync::atomic::Ordering::Acquire)
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
                goal_payload_sha256: [7; 32],
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
        ) -> Result<ChildNavigationCancelTicket, String> {
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
            Ok(ticket)
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
        async fn submit(
            &self,
            request: ChildNavigationRequest,
        ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError> {
            let target = request.goal.target;
            let pose = target.pose;
            let outcome = crate::nav_runtime::set_goal(
                &self.facade,
                &self.facade.nav,
                crate::nav_runtime::SetGoalInput {
                    goal_id: request.request_id,
                    owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                    parent_request_id: request.parent_request_id,
                    goal_generation: 0,
                    motion_admission: request.motion_admission,
                    frame_id: "map".to_string(),
                    target: PoseState {
                        x: pose.position.x,
                        y: pose.position.y,
                        z: pose.position.z,
                        qx: pose.orientation.x,
                        qy: pose.orientation.y,
                        qz: pose.orientation.z,
                        qw: pose.orientation.w,
                        frame_id: "map".to_string(),
                        timestamp_ns: target.stamp.nanos.max(0) as u64,
                    },
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
        ) -> Result<ChildNavigationCancelTicket, String> {
            Err("response-loss child is cancelled through NavRuntime ownership".to_string())
        }
    }

    #[async_trait::async_trait]
    impl ExplorePoseProvider for LifecycleStaticPoseProvider {
        async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
            Ok(ExploreRobotPose {
                x: 2.0,
                y: 2.0,
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

    fn activation_request(session_id: &str, map_name: &str) -> ActivateMapRequest {
        ActivateMapRequest {
            session_id: session_id.to_string(),
            map_name: map_name.to_string(),
            checkpoint: Some(OccupancyGridData {
                meta: Some(OccupancyGridMeta {
                    width: 2,
                    height: 2,
                    resolution: 0.05,
                    origin_x: -1.0,
                    origin_y: 2.0,
                    frame_id: "map".to_string(),
                    revision: 1,
                    timestamp_ns: 1_700_000_000_000_000_000,
                }),
                data: vec![u8::MAX, 0, 100, 50],
            }),
            cloud_checkpoint: None,
            mapping_backend_checkpoint: None,
            activation_request_id: String::new(),
            asset_manifest_sha256: Vec::new(),
            asset_bundle_sha256: Vec::new(),
            expected_runtime_generation: 0,
            activation_contract_version: 0,
        }
    }

    fn cloud_activation_request(xyz_f32_le: Vec<u8>) -> ActivateMapRequest {
        ActivateMapRequest {
            session_id: "mapload:one".to_string(),
            map_name: "warehouse".to_string(),
            checkpoint: Some(OccupancyGridData {
                meta: Some(OccupancyGridMeta {
                    width: 2,
                    height: 2,
                    resolution: 0.05,
                    origin_x: -1.0,
                    origin_y: 2.0,
                    frame_id: "map".to_string(),
                    revision: 119,
                    timestamp_ns: 1_700_000_000_000_000_000,
                }),
                data: vec![u8::MAX, 0, 100, 50],
            }),
            cloud_checkpoint: Some(CloudMapCheckpoint {
                frame_id: "map".to_string(),
                source_revision: 119,
                point_count: 2,
                voxel_size: crate::pointcloud::VOXEL_SIZE,
                xyz_sha256: crate::map_checksum::sha256_hex(&xyz_f32_le),
                xyz_f32_le,
                payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            }),
            mapping_backend_checkpoint: None,
            activation_request_id: String::new(),
            asset_manifest_sha256: Vec::new(),
            asset_bundle_sha256: Vec::new(),
            expected_runtime_generation: 0,
            activation_contract_version: 0,
        }
    }

    fn modern_activation_request(
        mut request: ActivateMapRequest,
        activation_request_id: &str,
        runtime_generation: u64,
    ) -> ActivateMapRequest {
        let occupancy_sha256 =
            canonical_occupancy_sha256(request.checkpoint.as_ref().unwrap()).unwrap();
        let cloud = request.cloud_checkpoint.as_ref().unwrap();
        let target = immutable_target_from_checkpoint(cloud, "map", true).unwrap();
        let cloud_sha256 = canonical_cloud_sha256(&target).unwrap();
        let asset_manifest_sha256 = [0x5a; 32];
        let asset_bundle_sha256 =
            crate::map_checksum::sha256_bytes(&canonical_asset_bundle_preimage(
                asset_manifest_sha256,
                occupancy_sha256,
                Some(cloud_sha256),
                None,
            ));
        request.activation_request_id = activation_request_id.to_owned();
        request.asset_manifest_sha256 = asset_manifest_sha256.to_vec();
        request.asset_bundle_sha256 = asset_bundle_sha256.to_vec();
        request.expected_runtime_generation = runtime_generation;
        request.activation_contract_version = 1;
        request
    }

    fn ray_mapping_adapter() -> crate::mapping_adapter::MappingAdapter {
        crate::mapping_adapter::MappingAdapter::new_with_backend_for_tests(
            crate::mapping_adapter::MappingAdapterConfig {
                resolution: 0.1,
                width_cells: 20,
                height_cells: 20,
                origin_xy: (-1.0, -1.0),
                voxel_size: 0.1,
                occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
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
            Some(CloudMapShmWriter::new(&cloud_path).expect("ray checkpoint cloud-map writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        (handle, service, cloud_path)
    }

    async fn export_voxel_backend_checkpoint() -> MappingBackendCheckpoint {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-backend-checkpoint-source-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&cloud_path).expect("source cloud-map writer"),
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
            0,
            "map",
        )
        .await
        .expect("canonical source cloud handoff"));
        let checkpoint =
            GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(Empty {}))
                .await
                .expect("ExportMappingBackendCheckpoint")
                .into_inner();
        drop(service);
        drop(handle);
        let _ = std::fs::remove_file(cloud_path);
        checkpoint
    }

    fn canonical_backend_checkpoint_fixture() -> MappingBackendCheckpoint {
        let payload = br#"{"schema_version":1,"backend_kind":"voxel"}"#.to_vec();
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
            xyz_sha256: crate::map_checksum::sha256_hex(&cloud_xyz),
            xyz_f32_le: cloud_xyz,
            payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
        };
        MappingBackendCheckpoint {
            schema_version: crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
            backend_kind: "voxel".to_owned(),
            source_session_generation: 7,
            content_sha256: crate::map_checksum::sha256_bytes(&payload).to_vec(),
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
        }
    }

    fn activation_request_with_backend_checkpoint(
        session_id: &str,
        map_name: &str,
        checkpoint: MappingBackendCheckpoint,
    ) -> ActivateMapRequest {
        ActivateMapRequest {
            session_id: session_id.to_owned(),
            map_name: map_name.to_owned(),
            checkpoint: checkpoint.canonical_occupancy_checkpoint.clone(),
            cloud_checkpoint: checkpoint.canonical_cloud_checkpoint.clone(),
            mapping_backend_checkpoint: Some(checkpoint),
            activation_request_id: String::new(),
            asset_manifest_sha256: Vec::new(),
            asset_bundle_sha256: Vec::new(),
            expected_runtime_generation: 0,
            activation_contract_version: 0,
        }
    }

    fn exploration_request(request_id: &str) -> ExploreRequest {
        ExploreRequest {
            request_id: request_id.to_string(),
            strategy: "wavefront".to_string(),
            arbitration: "prefer_wavefront".to_string(),
            motion_guard_generation: 7,
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
                motion_guard_generation: 7,
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
        handle
            .explore
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .expect("frontier costmap");
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

    fn rollover_recovery_proto_request(
        snapshot: &crate::explore_runtime::ExploreRuntimeSnapshot,
    ) -> RecoverExplorationAfterNavigationRolloverRequest {
        let lease = snapshot
            .child_lease
            .as_ref()
            .expect("retained exploration child lease");
        RecoverExplorationAfterNavigationRolloverRequest {
            request_id: snapshot.request_id.clone(),
            exploration_generation: snapshot.exploration_generation,
            child_lease: Some(ExplorationChildLease {
                session_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
                goal_payload_sha256: lease.goal_payload_sha256.to_vec(),
            }),
            expected_navigation_runtime_generation: 41,
            observed_navigation_runtime_generation: 42,
            guard_proof: Some(crate::proto::ExplorationRecoveryGuardProof {
                stop_operation_id: "hold:geometry-rollover-test".to_string(),
                guard_generation: 7,
                guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 9,
                zero_ack_id: "zero:geometry-rollover-test".to_string(),
                zero_applied_monotonic_ns: now_monotonic_nanos(),
                recovery_fence_token: vec![9; 32],
                recovery_correlation: "geometry-rollover:test".to_string(),
            }),
        }
    }

    fn rollover_recovery_internal_request(
        request: &RecoverExplorationAfterNavigationRolloverRequest,
    ) -> crate::explore_runtime::ExplorationRolloverRecoveryRequest {
        let child = request.child_lease.as_ref().expect("child lease");
        let guard = request.guard_proof.as_ref().expect("guard proof");
        crate::explore_runtime::ExplorationRolloverRecoveryRequest {
            request_id: request.request_id.clone(),
            exploration_generation: request.exploration_generation,
            child_lease: ChildNavigationLease {
                session_id: child.session_id.clone(),
                owner_kind: child.owner_kind,
                parent_request_id: child.parent_request_id.clone(),
                goal_generation: child.goal_generation,
                producer_runtime_generation: child.producer_runtime_generation,
                goal_payload_sha256: child
                    .goal_payload_sha256
                    .as_slice()
                    .try_into()
                    .expect("32-byte goal hash"),
            },
            expected_navigation_runtime_generation: request.expected_navigation_runtime_generation,
            observed_navigation_runtime_generation: request.observed_navigation_runtime_generation,
            guard_proof: crate::explore_runtime::ExplorationRecoveryGuardProof {
                stop_operation_id: guard.stop_operation_id.clone(),
                recovery_correlation: guard.recovery_correlation.clone(),
                motion_admission: MotionAdmissionProof::try_from_wire(
                    guard.guard_generation,
                    &guard.guard_boot_nonce,
                    guard.motion_stop_generation,
                )
                .expect("motion admission"),
                zero_ack_id: guard.zero_ack_id.clone(),
                zero_applied_monotonic_ns: guard.zero_applied_monotonic_ns,
                recovery_fence_token: crate::explore_runtime::RecoveryFenceToken::from_wire(
                    guard.recovery_fence_token.clone(),
                ),
            },
        }
    }

    fn global_stop_reconciliation_proto_request(
        snapshot: &crate::explore_runtime::ExploreRuntimeSnapshot,
    ) -> ReconcileExplorationAfterGlobalStopRequest {
        let lease = snapshot.child_lease.as_ref().expect("retained child lease");
        let reconciliation = &snapshot.global_stop_reconciliation;
        let admission = reconciliation
            .motion_admission
            .expect("original motion admission");
        ReconcileExplorationAfterGlobalStopRequest {
            request_id: snapshot.request_id.clone(),
            exploration_generation: snapshot.exploration_generation,
            child_lease: Some(ExplorationChildLease {
                session_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
                goal_payload_sha256: lease.goal_payload_sha256.to_vec(),
            }),
            cancel_generation: reconciliation.cancel_generation,
            recovery_correlation: reconciliation.recovery_correlation.clone(),
            admission_guard_generation: admission.guard_generation,
            admission_guard_boot_nonce: admission.guard_boot_nonce.to_vec(),
            admission_motion_stop_generation: admission.motion_stop_generation,
            held_guard_proof: Some(crate::proto::ExplorationRecoveryGuardProof {
                stop_operation_id: "hold:geometry-global-stop-test".to_string(),
                guard_generation: admission.guard_generation,
                guard_boot_nonce: admission.guard_boot_nonce.to_vec(),
                motion_stop_generation: admission.motion_stop_generation + 1,
                zero_ack_id: "zero:geometry-global-stop-test".to_string(),
                zero_applied_monotonic_ns: now_monotonic_nanos(),
                recovery_fence_token: vec![9; 32],
                recovery_correlation: reconciliation.recovery_correlation.clone(),
            }),
        }
    }

    fn global_stop_reconciliation_internal_request(
        request: &ReconcileExplorationAfterGlobalStopRequest,
    ) -> crate::explore_runtime::ExplorationGlobalStopReconciliationRequest {
        let child = request.child_lease.as_ref().expect("child lease");
        let held = request.held_guard_proof.as_ref().expect("held proof");
        let original_motion_admission = MotionAdmissionProof::try_from_wire(
            request.admission_guard_generation,
            &request.admission_guard_boot_nonce,
            request.admission_motion_stop_generation,
        )
        .expect("original motion admission");
        crate::explore_runtime::ExplorationGlobalStopReconciliationRequest {
            request_id: request.request_id.clone(),
            exploration_generation: request.exploration_generation,
            child_lease: ChildNavigationLease {
                session_id: child.session_id.clone(),
                owner_kind: child.owner_kind,
                parent_request_id: child.parent_request_id.clone(),
                goal_generation: child.goal_generation,
                producer_runtime_generation: child.producer_runtime_generation,
                goal_payload_sha256: child
                    .goal_payload_sha256
                    .as_slice()
                    .try_into()
                    .expect("32-byte goal hash"),
            },
            cancel_generation: request.cancel_generation,
            recovery_correlation: request.recovery_correlation.clone(),
            guard_proof: crate::explore_runtime::ExplorationGlobalStopGuardProof {
                stop_operation_id: held.stop_operation_id.clone(),
                recovery_correlation: held.recovery_correlation.clone(),
                original_motion_admission,
                held_guard_generation: held.guard_generation,
                held_guard_boot_nonce: held
                    .guard_boot_nonce
                    .as_slice()
                    .try_into()
                    .expect("16-byte held boot nonce"),
                held_motion_stop_generation: held.motion_stop_generation,
                zero_ack_id: held.zero_ack_id.clone(),
                zero_applied_monotonic_ns: held.zero_applied_monotonic_ns,
                recovery_fence_token: crate::explore_runtime::RecoveryFenceToken::from_wire(
                    held.recovery_fence_token.clone(),
                ),
            },
        }
    }

    #[tokio::test]
    async fn process_stop_waits_for_exploration_zero_before_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_task_id, lease) =
            start_active_exploration(&handle, sink, "process-stop-zero").await;
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
        start_active_exploration(&handle, sink.clone(), "process-stop-cancel-timeout").await;
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
        assert!(!retained.child_task_id.is_empty());
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
        facade.explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
        facade.explore_scheduler = Arc::new(tokio::sync::Mutex::new(None));
        let sink = Arc::new(SubmitLeaseThenBlockSink {
            facade,
            entered: tokio::sync::Notify::new(),
        });
        handle.explore.set_navigation_sink(sink.clone());
        handle
            .explore
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .expect("frontier costmap");
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
    async fn rollover_recovery_keeps_child_when_guard_verification_is_unavailable() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink.clone(), "guard-proof-unavailable").await;
        *sink
            .cancel_error
            .lock()
            .expect("lifecycle exploration sink lock") = Some("old runtime lost child".to_string());
        let retained = handle.explore.stop("guard-proof-unavailable").await;
        assert_eq!(retained.status, STATUS_FAILED);
        assert!(retained.rollover_recovery_eligible);
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 42,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner generation");
        let service = GeometryRuntimeService::new(handle.clone());
        let request = rollover_recovery_proto_request(&retained);

        let _env_lock = MOTION_GUARD_SOCKET_ENV_LOCK.lock().await;
        let previous_socket = std::env::var_os("NUWAX_MOTION_GUARD_SOCKET");
        let missing_socket = std::env::temp_dir().join(format!(
            "nuwax-motion-guard-missing-{}.sock",
            uuid::Uuid::new_v4()
        ));
        std::env::set_var("NUWAX_MOTION_GUARD_SOCKET", &missing_socket);
        let result = GeometryRuntime::recover_exploration_after_navigation_rollover(
            &service,
            Request::new(request),
        )
        .await;
        if let Some(value) = previous_socket {
            std::env::set_var("NUWAX_MOTION_GUARD_SOCKET", value);
        } else {
            std::env::remove_var("NUWAX_MOTION_GUARD_SOCKET");
        }

        let error = result.expect_err("unverified recovery fence must fail closed");
        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert_eq!(
            handle.explore.snapshot().child_task_id,
            retained.child_task_id
        );
        assert!(handle.explore.blocks_mapping_mutation().await);
    }

    #[tokio::test]
    async fn exact_rollover_replay_precedes_planner_and_guard_checks() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink.clone(), "exact-rollover-replay").await;
        *sink
            .cancel_error
            .lock()
            .expect("lifecycle exploration sink lock") = Some("old runtime lost child".to_string());
        let retained = handle.explore.stop("exact-rollover-replay").await;
        let proto_request = rollover_recovery_proto_request(&retained);
        let internal_request = rollover_recovery_internal_request(&proto_request);
        let committed = handle
            .explore
            .recover_after_navigation_rollover(internal_request)
            .await
            .expect("seed completed recovery");
        let service = GeometryRuntimeService::new(handle);

        let replay = GeometryRuntime::recover_exploration_after_navigation_rollover(
            &service,
            Request::new(proto_request),
        )
        .await
        .expect("exact replay must not require live planner or guard")
        .into_inner();

        assert_eq!(replay.status, committed.status);
        assert_eq!(replay.last_stop_reason, "runtime_rollover_aborted_safe");
        assert!(replay.child_lease.is_none());
    }

    #[tokio::test]
    async fn exact_global_stop_replay_precedes_live_guard_verification() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle.explore.set_reconciliation_journal_path_for_tests(
            std::env::temp_dir()
                .join(format!(
                    "nuwax-server-global-stop-journal-{}",
                    uuid::Uuid::new_v4()
                ))
                .join("journal.json"),
        );
        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_task_id, lease) =
            start_active_exploration(&handle, sink, "exact-global-stop-replay").await;
        assert_eq!(
            handle.explore.stop("exact-global-stop-replay").await.status,
            STATUS_STOPPING
        );
        let retained = handle
            .explore
            .report_navigation_result(
                &child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: ChildNavigationTerminalProof {
                        lease: lease.clone(),
                    },
                    cancel_generation: 1,
                    reason: "exact cancel failed".to_string(),
                },
            )
            .await
            .expect("retained failed cancellation");
        assert!(retained.global_stop_reconciliation.eligible);
        let proto_request = global_stop_reconciliation_proto_request(&retained);
        let internal_request = global_stop_reconciliation_internal_request(&proto_request);
        let committed = handle
            .explore
            .reconcile_after_global_stop(internal_request)
            .await
            .expect("seed completed global-stop reconciliation");
        assert!(committed.ownership_reconciliation_receipt.is_some());
        let service = GeometryRuntimeService::new(handle);

        let _env_lock = MOTION_GUARD_SOCKET_ENV_LOCK.lock().await;
        let previous_socket = std::env::var_os("NUWAX_MOTION_GUARD_SOCKET");
        let missing_socket = std::env::temp_dir().join(format!(
            "nuwax-motion-guard-missing-{}.sock",
            uuid::Uuid::new_v4()
        ));
        std::env::set_var("NUWAX_MOTION_GUARD_SOCKET", &missing_socket);
        let result = GeometryRuntime::reconcile_exploration_after_global_stop(
            &service,
            Request::new(proto_request),
        )
        .await;
        if let Some(value) = previous_socket {
            std::env::set_var("NUWAX_MOTION_GUARD_SOCKET", value);
        } else {
            std::env::remove_var("NUWAX_MOTION_GUARD_SOCKET");
        }

        let replay = result
            .expect("exact replay must not require live Guard")
            .into_inner();
        assert!(replay.child_lease.is_none());
        let receipt = replay
            .ownership_reconciliation_receipt
            .expect("wire reconciliation receipt");
        assert_eq!(receipt.parent_request_id, "exact-global-stop-replay");
        assert_eq!(receipt.cancel_generation, 1);
        let acknowledged = GeometryRuntime::acknowledge_exploration_ownership_reconciliation(
            &service,
            Request::new(AcknowledgeExplorationOwnershipReconciliationRequest {
                recovery_correlation: receipt.recovery_correlation,
                child_lease_fingerprint: receipt.child_lease_fingerprint,
            }),
        )
        .await
        .expect("reconciliation acknowledgement RPC")
        .into_inner();
        assert!(acknowledged.acknowledged);
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
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let accepted =
            start_public_navigation(&service, &handle, "public-stop-cancel-timeout").await;
        assert_eq!(accepted.status, "ACCEPTED");
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
                        motion_guard_generation: 7,
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

    #[tokio::test]
    async fn mapping_mutation_waits_for_late_zeroed_after_child_cancel_failure() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "parked-session", "warehouse-parked").await;
        let sink = Arc::new(LifecycleExploreSink::default());
        handle.explore.set_navigation_sink(sink.clone());
        handle
            .explore
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .expect("frontier costmap");
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
        assert_eq!(failed.child_task_id, active.child_task_id);
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
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
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
        let stopped_snapshot = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        handle
            .map_revision_tx
            .send(MapRevisionEvent::from_map(
                &MapState {
                    width: 1,
                    height: 1,
                    resolution: 0.1,
                    frame_id: "map".to_string(),
                    revision: first_revision + 1,
                    timestamp_ns: first_revision + 1,
                    data: vec![100],
                    ..MapState::default()
                },
                &stopped_snapshot,
            ))
            .expect("coalescer remains registered");
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
                key: "/tmp/stale.mmap".to_string(),
                file_size: 8192,
                payload_size: 1,
                revision: 99,
                layout_version: 2,
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
            Some(CloudMapShmWriter::new(&cloud_path).expect("configured-frame cloud-map writer"));
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
        assert!(error.message().contains("robot/map"));

        let mut wrong_cloud = cloud_activation_request(cloud_xyz.clone());
        wrong_cloud
            .checkpoint
            .as_mut()
            .and_then(|checkpoint| checkpoint.meta.as_mut())
            .expect("occupancy checkpoint metadata")
            .frame_id = "robot/map".to_string();
        let error = GeometryRuntime::activate_map(&service, Request::new(wrong_cloud))
            .await
            .expect_err("literal map cloud checkpoint must fail for robot/map runtime");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert!(error.message().contains("cloud-map checkpoint frame"));
        assert!(error.message().contains("robot/map"));

        let mut request = cloud_activation_request(cloud_xyz.clone());
        request
            .checkpoint
            .as_mut()
            .and_then(|checkpoint| checkpoint.meta.as_mut())
            .expect("occupancy checkpoint metadata")
            .frame_id = "robot/map".to_string();
        request
            .cloud_checkpoint
            .as_mut()
            .expect("cloud checkpoint")
            .frame_id = "robot/map".to_string();
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
        assert!(!idle.legacy_checkpoint_reconstructed);

        let started = start_mapping_session(&service, "typed-state", "warehouse").await;
        assert_eq!(started.mapping_backend_kind, "voxel");
        assert!(!started.legacy_checkpoint_reconstructed);
        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .expect("active voxel state")
            .into_inner();
        assert_eq!(queried.mapping_backend_kind, "voxel");
        assert!(!queried.legacy_checkpoint_reconstructed);

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
        assert!(!ray_idle.legacy_checkpoint_reconstructed);

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
        assert!(legacy.legacy_checkpoint_reconstructed);
        assert_eq!(legacy.mapping_backend_kind, "voxel");
        let legacy_queried =
            GeometryRuntime::get_mapping_state(&legacy_service, Request::new(Empty {}))
                .await
                .expect("legacy GetMappingState")
                .into_inner();
        assert!(legacy_queried.legacy_checkpoint_reconstructed);
        assert_eq!(legacy_queried.mapping_backend_kind, "voxel");
    }

    #[test]
    fn backend_checkpoint_receipt_validation_covers_independent_lag_and_mismatches() {
        let checkpoint = canonical_backend_checkpoint_fixture();
        validate_mapping_backend_checkpoint(
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
        validate_mapping_backend_checkpoint(
            &zero_timestamp,
            crate::mapping_backend::MappingBackendKind::Voxel,
        )
        .expect("zero source timestamps are valid identities");

        macro_rules! assert_invalid {
            ($label:literal, $mutate:expr) => {{
                let mut invalid = canonical_backend_checkpoint_fixture();
                ($mutate)(&mut invalid);
                assert!(
                    validate_mapping_backend_checkpoint(
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
        assert_invalid!("payload hash", |value: &mut MappingBackendCheckpoint| {
            value.content_sha256[0] ^= 1
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
    fn activation_bundle_requires_byte_identical_canonical_assets_and_bounded_envelope() {
        let checkpoint = canonical_backend_checkpoint_fixture();
        let mut occupancy = checkpoint.canonical_occupancy_checkpoint.clone().unwrap();
        let cloud = checkpoint.canonical_cloud_checkpoint.clone().unwrap();
        occupancy.data[0] = 100;
        assert!(validate_mapping_checkpoint_bundle(&checkpoint, &occupancy, Some(&cloud)).is_err());

        let occupancy = checkpoint.canonical_occupancy_checkpoint.clone().unwrap();
        let mut cloud = cloud;
        cloud.xyz_f32_le[0] ^= 1;
        assert!(validate_mapping_checkpoint_bundle(&checkpoint, &occupancy, Some(&cloud)).is_err());
        assert!(validate_mapping_checkpoint_bundle(&checkpoint, &occupancy, None).is_err());

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
                .checkpoint
                .as_mut()
                .unwrap()
                .meta
                .as_mut()
                .unwrap()
                .timestamp_ns = timestamp_ns;
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

    #[tokio::test]
    async fn backend_checkpoint_export_and_voxel_activation_round_trip() {
        let checkpoint = export_voxel_backend_checkpoint().await;
        assert_eq!(
            checkpoint.schema_version,
            crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION
        );
        assert_eq!(checkpoint.backend_kind, "voxel");
        assert_eq!(checkpoint.source_session_id, "checkpoint-source");
        assert!(checkpoint.source_session_generation > 0);
        assert_eq!(
            checkpoint.payload_size,
            u64::try_from(checkpoint.payload.len()).unwrap()
        );
        assert_eq!(
            checkpoint.content_sha256,
            crate::map_checksum::sha256_bytes(&checkpoint.payload)
        );
        let occupancy_receipt = checkpoint
            .occupancy_receipt
            .as_ref()
            .expect("canonical occupancy receipt");
        let cloud_receipt = checkpoint
            .cloud_receipt
            .as_ref()
            .expect("canonical cloud receipt");
        assert_eq!(occupancy_receipt.source_timestamp_ns, 0);
        assert_eq!(cloud_receipt.source_timestamp_ns, 0);
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
        let canonical_occupancy = checkpoint
            .canonical_occupancy_checkpoint
            .clone()
            .expect("canonical raw occupancy");
        let canonical_cloud = checkpoint
            .canonical_cloud_checkpoint
            .clone()
            .expect("canonical cloud checkpoint");

        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("restored real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-backend-checkpoint-target-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&cloud_path).expect("target cloud-map writer"),
        )));
        let service = GeometryRuntimeService::new(handle.clone());
        let mut request = activation_request("mapload:restored", "warehouse-restored");
        request.checkpoint = Some(canonical_occupancy);
        request.cloud_checkpoint = Some(canonical_cloud);
        request.mapping_backend_checkpoint = Some(checkpoint);
        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("activate voxel backend checkpoint")
            .into_inner();
        assert!(activated.active);
        assert!(activated
            .message
            .contains("legacy_checkpoint_reconstructed=false"));
        assert!(!activated.legacy_checkpoint_reconstructed);
        assert_eq!(activated.mapping_backend_kind, "voxel");
        assert_eq!(handle.state.read().await.map.revision, activated.revision);
        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .expect("GetMappingState after supplied checkpoint")
            .into_inner();
        assert!(!queried.legacy_checkpoint_reconstructed);
        assert_eq!(queried.mapping_backend_kind, "voxel");
        drop(service);
        drop(handle);
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn ray_backend_checkpoint_activation_preserves_continuation_and_receipts() {
        let (source_handle, source_service, source_cloud_path) =
            ray_runtime_with_cloud_writer("source").await;
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
            assert_eq!(committed, sequence == 3);
        }
        let source_checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &source_service,
            Request::new(Empty {}),
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

        let (restored_handle, restored_service, restored_cloud_path) =
            ray_runtime_with_cloud_writer("restored").await;
        let restored_session_id = "ray-checkpoint-restored";
        let activated = GeometryRuntime::activate_map(
            &restored_service,
            Request::new(ActivateMapRequest {
                session_id: restored_session_id.to_owned(),
                map_name: "warehouse-ray".to_owned(),
                checkpoint: source_checkpoint.canonical_occupancy_checkpoint.clone(),
                cloud_checkpoint: source_checkpoint.canonical_cloud_checkpoint.clone(),
                mapping_backend_checkpoint: Some(source_checkpoint.clone()),
                activation_request_id: String::new(),
                asset_manifest_sha256: Vec::new(),
                asset_bundle_sha256: Vec::new(),
                expected_runtime_generation: 0,
                activation_contract_version: 0,
            }),
        )
        .await
        .expect("activate exported ray checkpoint")
        .into_inner();
        assert!(activated.active);
        assert!(!activated.legacy_checkpoint_reconstructed);
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
        assert_eq!(
            occupancy_checkpoint_proto(&restored_map),
            occupancy_checkpoint_proto(&source_map),
            "restored and uninterrupted canonical maps must match"
        );
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
        assert_eq!(restored_cloud.xyz_f32_le, source_cloud.xyz_f32_le);

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
            Request::new(Empty {}),
        )
        .await
        .expect("export uninterrupted continued checkpoint")
        .into_inner();
        let restored_continued_checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
            &restored_service,
            Request::new(Empty {}),
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
            assert_eq!(committed, sequence == 3);
        }
        let seeded_checkpoint =
            GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(Empty {}))
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
                GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(Empty {}))
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
                GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(Empty {}))
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
        assert!(!tokio::time::timeout(Duration::from_secs(1), handoff_task)
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

    #[tokio::test]
    async fn corrupted_backend_checkpoint_fails_before_activation() {
        let mut checkpoint = export_voxel_backend_checkpoint().await;
        checkpoint.payload.push(0);
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
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
        let checkpoint = export_voxel_backend_checkpoint().await;
        let mut invalid = checkpoint.clone();
        invalid.payload[8..12].copy_from_slice(&2_u32.to_le_bytes());
        invalid.payload_size = u64::try_from(invalid.payload.len()).unwrap();
        invalid.content_sha256 = crate::map_checksum::sha256_bytes(&invalid.payload).to_vec();

        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-inner-checkpoint-retry-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&cloud_path).expect("checkpoint retry cloud-map writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        let session_id = "mapload:inner-schema-retry";
        let map_name = "warehouse-inner-schema-retry";
        let runtime_generation = service.activation_receipts.runtime_generation();
        let modern_backend_request =
            |checkpoint: MappingBackendCheckpoint, activation_request_id: &str| {
                let mut request =
                    activation_request_with_backend_checkpoint(session_id, map_name, checkpoint);
                let occupancy_sha256 =
                    canonical_occupancy_sha256(request.checkpoint.as_ref().unwrap()).unwrap();
                let target = immutable_target_from_checkpoint(
                    request.cloud_checkpoint.as_ref().unwrap(),
                    "map",
                    true,
                )
                .unwrap();
                let cloud_sha256 = canonical_cloud_sha256(&target).unwrap();
                let backend_sha256 = canonical_backend_sha256(
                    request.mapping_backend_checkpoint.as_ref().unwrap(),
                    "map",
                )
                .unwrap();
                let asset_manifest_sha256 = [0x5a; 32];
                request.activation_request_id = activation_request_id.to_owned();
                request.asset_manifest_sha256 = asset_manifest_sha256.to_vec();
                request.asset_bundle_sha256 =
                    crate::map_checksum::sha256_bytes(&canonical_asset_bundle_preimage(
                        asset_manifest_sha256,
                        occupancy_sha256,
                        Some(cloud_sha256),
                        Some(backend_sha256),
                    ))
                    .to_vec();
                request.expected_runtime_generation = runtime_generation;
                request.activation_contract_version = 1;
                request
            };
        let invalid_request = modern_backend_request(invalid, "activation:inner-schema-invalid");
        let invalid_replay_request = invalid_request.clone();
        let failed = GeometryRuntime::activate_map(&service, Request::new(invalid_request))
            .await
            .expect("invalid staged activation returns a typed terminal")
            .into_inner();
        assert!(!failed.success);
        assert_eq!(
            failed.activation_receipt_disposition,
            ActivationReceiptDisposition::InvalidatedPrecommit as i32
        );
        assert!(failed.message.contains("unsupported schema version 2"));
        let retired = GeometryRuntime::activate_map(&service, Request::new(invalid_replay_request))
            .await
            .expect("exact invalidated replay returns the permanent terminal")
            .into_inner();
        assert!(!retired.success);
        assert!(retired.idempotent_replay);
        assert_eq!(
            retired.activation_receipt_disposition,
            ActivationReceiptDisposition::Retired as i32
        );
        assert_eq!(retired.message, "ACTIVATION_IDENTITY_RETIRED");
        let before_retry = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        assert_eq!(
            before_retry.phase,
            crate::mapping_adapter::MappingSessionPhase::Idle
        );
        assert_eq!(before_retry.session_generation, 0);
        assert_eq!(before_retry.revision, 0);
        assert!(!before_retry.active);
        assert_eq!(handle.state.read().await.map.revision, 0);
        assert!(handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .expect("cloud-map writer")
            .committed_snapshot()
            .is_none());

        let activated = GeometryRuntime::activate_map(
            &service,
            Request::new(modern_backend_request(
                checkpoint,
                "activation:inner-schema-retry",
            )),
        )
        .await
        .expect("corrected checkpoint retry with same session id")
        .into_inner();
        assert!(activated.active);
        assert_eq!(activated.session_id, session_id);
        assert!(activated.session_generation > 0);
        assert!(activated.revision > 0);
        let _ = std::fs::remove_file(cloud_path);
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
            Request::new(activation_request("mapload:ray", "warehouse-ray")),
        )
        .await
        .expect_err("ray activation without backend state must fail");
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("BACKEND_CHECKPOINT_REQUIRED"));
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
            Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
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
        request.asset_manifest_sha256 = vec![1; 32];

        let error = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect_err("mixed activation identity must reject");
        assert_eq!(error.code(), tonic::Code::InvalidArgument);
        assert_eq!(error.message(), "ACTIVATE_MAP_IDENTITY_FIELDS_MIXED");
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
            Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle.clone());
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let mut request =
            modern_activation_request(cloud_activation_request(xyz), "act-modern-1", 1);
        let cloud = request.cloud_checkpoint.as_mut().unwrap();
        cloud.xyz_sha256 = format!(" \t{}\r\n", cloud.xyz_sha256.to_ascii_uppercase());
        validate_cloud_checkpoint_metadata(cloud, "map").unwrap();
        let replay_request = request.clone();

        let activated = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("modern activation")
            .into_inner();
        assert!(activated.success);
        assert_eq!(activated.activation_receipt_schema_version, 1);
        assert_eq!(activated.runtime_generation, 1);
        assert!(activated.backend_generation > 0);
        assert_eq!(activated.activation_request_id, "act-modern-1");
        assert_eq!(activated.activation_request_identity_sha256.len(), 32);
        assert_eq!(activated.activation_identity_sha256.len(), 32);
        assert_eq!(activated.target_receipt_sha256.len(), 32);
        assert!(activated.relocalization_target_available);
        assert_eq!(
            activated.activation_receipt_disposition,
            ActivationReceiptDisposition::Sealed as i32
        );

        let queried = GeometryRuntime::get_mapping_state(&service, Request::new(Empty {}))
            .await
            .unwrap()
            .into_inner();
        assert!(queried.relocalization_target_available);
        assert_eq!(
            queried.target_receipt_sha256,
            activated.target_receipt_sha256
        );

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
    async fn legacy_activation_is_permanently_relocalization_ineligible() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activation-legacy-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle);
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let activated =
            GeometryRuntime::activate_map(&service, Request::new(cloud_activation_request(xyz)))
                .await
                .unwrap()
                .into_inner();
        assert!(activated.success);
        assert_eq!(
            activated.activation_receipt_disposition,
            ActivationReceiptDisposition::LegacyUnbound as i32
        );
        assert!(!activated.relocalization_target_available);
        assert!(activated.activation_identity_sha256.is_empty());
        assert!(activated.target_receipt_sha256.is_empty());
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
            Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
        let phase_a = crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let (shutdown, _listener) = shutdown_channel_inner(None, Some(phase_a), None, None);
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
        use crate::correction_coordinator::ActivationJournalStep;

        let cases = [
            (
                ActivationJournalStep::SessionStart,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationJournalStep::OccupancyShmBegin,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationJournalStep::OccupancyInstall,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                ActivationJournalStep::CloudInstall,
                ActivationReceiptDisposition::OutcomeUnknown,
            ),
            (
                ActivationJournalStep::TargetReceiptVerification,
                ActivationReceiptDisposition::OutcomeUnknown,
            ),
            (
                ActivationJournalStep::ActivationRecord,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationJournalStep::SharedStateCommit,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationJournalStep::CacheClears,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationJournalStep::MapEvent,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationJournalStep::CloudEvent,
                ActivationReceiptDisposition::InvalidatedPostcommit,
            ),
            (
                ActivationJournalStep::TypedGlobalMap,
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
                Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
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
            if step == ActivationJournalStep::TargetReceiptVerification {
                assert!(terminal
                    .message
                    .contains("ACTIVATE_MAP_TARGET_RECEIPT_OUTCOME_UNKNOWN"));
            }
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
            Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
        let service = GeometryRuntimeService::new(handle);
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request = modern_activation_request(cloud_activation_request(xyz), "act-seal-loss", 1);
        service.inject_activation_fault_after(
            crate::correction_coordinator::ActivationJournalStep::Seal,
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
        let checkpoint = export_voxel_backend_checkpoint().await;
        let cases = [
            (
                crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
                ActivationReceiptDisposition::InvalidatedPrecommit,
            ),
            (
                crate::correction_coordinator::ActivationJournalStep::BackendCommit,
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
                Some(crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("cloud writer"));
            let service = GeometryRuntimeService::new(handle.clone());
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
            crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("test cloud-map writer"),
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
                crate::pointcloud::CloudMapShmWriter::new(&cloud_path)
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
    async fn activate_map_publish_failure_retains_checkpoint_for_replay() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activate-cloud-failure-test-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() = Some(
            crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("test cloud-map writer"),
        );
        let backend = Arc::new(CheckpointPublishBackend::default());
        backend
            .fail_after_observe
            .store(true, std::sync::atomic::Ordering::Release);
        handle.transport = LcmTransport::new(backend.clone());
        let service = GeometryRuntimeService::new(handle.clone());
        let mut map_revision_rx = handle.map_revision_tx.subscribe();
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let request = cloud_activation_request(cloud_xyz.clone());

        let error = GeometryRuntime::activate_map(&service, Request::new(request.clone()))
            .await
            .expect_err("typed global-map publication failure must fail the RPC");
        assert_eq!(error.code(), tonic::Code::Internal);
        assert!(error.message().contains("installed"));
        let installed_revision = handle.state.read().await.cloud_map_revision;
        assert!(installed_revision > 0);
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());
        let map_event = map_revision_rx
            .recv()
            .await
            .expect("installed occupancy checkpoint event");
        assert_eq!(map_event.map_name, "warehouse");
        assert_eq!(map_event.data.as_ref(), &[u8::MAX, 0, 100, 50]);
        let shm = std::fs::read(&cloud_path).expect("installed cloud-map SHM");
        assert_eq!(
            u64::from_le_bytes(shm[28..36].try_into().expect("SHM revision bytes")),
            installed_revision
        );
        let active_buffer =
            u32::from_le_bytes(shm[12..16].try_into().expect("SHM active buffer bytes"));
        let payload_offset = 64
            + usize::try_from(active_buffer).expect("active buffer index")
                * crate::pointcloud::CM_SINGLE_BUFFER_SIZE;
        let mut installed_points = shm[payload_offset..payload_offset + cloud_xyz.len()]
            .chunks_exact(crate::pointcloud::CM_POINT_SIZE_BYTES)
            .map(<[u8; crate::pointcloud::CM_POINT_SIZE_BYTES]>::try_from)
            .collect::<Result<Vec<_>, _>>()
            .expect("installed SHM XYZ points");
        let mut expected_points = cloud_xyz
            .chunks_exact(crate::pointcloud::CM_POINT_SIZE_BYTES)
            .map(<[u8; crate::pointcloud::CM_POINT_SIZE_BYTES]>::try_from)
            .collect::<Result<Vec<_>, _>>()
            .expect("checkpoint XYZ points");
        installed_points.sort_unstable();
        expected_points.sort_unstable();
        assert_eq!(installed_points, expected_points);
        assert_eq!(
            backend
                .published
                .lock()
                .expect("observed failed publication")
                .len(),
            1,
            "publish error may arrive after one observable delivery"
        );

        assert!(service.has_activation_lifecycle_fence());
        assert!(
            handle
                .mapping_lifecycle_lock
                .clone()
                .try_lock_owned()
                .is_err(),
            "staged publication failure must retain lifecycle serialization"
        );

        {
            let mut state = service.state.write().await;
            state.phase = "stopping".to_owned();
        }
        let transient_rejection =
            GeometryRuntime::activate_map(&service, Request::new(request.clone()))
                .await
                .expect_err("same-request runtime rejection must remain retryable");
        assert_eq!(transient_rejection.code(), tonic::Code::FailedPrecondition);
        assert!(service.has_activation_lifecycle_fence());
        assert_eq!(
            backend
                .published
                .lock()
                .expect("transient rejection publication count")
                .len(),
            1,
            "ordinary retry rejection must not republish the retained frame"
        );
        {
            let mut state = service.state.write().await;
            state.phase = "running".to_owned();
        }

        let empty_stop = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: String::new(),
                reason: "must-not-consume-fence".to_owned(),
            }),
        )
        .await
        .expect_err("empty StopMapping must fail before consuming the staged fence");
        assert_eq!(empty_stop.code(), tonic::Code::InvalidArgument);
        let stale_stop = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "stale-session".to_owned(),
                reason: "must-not-consume-fence".to_owned(),
            }),
        )
        .await
        .expect_err("stale StopMapping must fail before consuming the staged fence");
        assert_eq!(stale_stop.code(), tonic::Code::FailedPrecondition);
        assert!(service.has_activation_lifecycle_fence());
        assert!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .is_active(),
            "invalid StopMapping cannot detach the staged active session"
        );
        assert!(
            handle
                .mapping_lifecycle_lock
                .clone()
                .try_lock_owned()
                .is_err(),
            "invalid StopMapping cannot release staged lifecycle serialization"
        );

        let divergent_xyz = [[8.025_f32, 0.025, 0.025], [9.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();
        let mut conflicting_request = request.clone();
        let conflicting_cloud = conflicting_request.cloud_checkpoint.as_mut().unwrap();
        conflicting_cloud.xyz_f32_le = divergent_xyz;
        conflicting_cloud.xyz_sha256 =
            crate::map_checksum::sha256_hex(&conflicting_cloud.xyz_f32_le);
        let conflict = GeometryRuntime::activate_map(&service, Request::new(conflicting_request))
            .await
            .expect("conflicting retained target returns typed terminal")
            .into_inner();
        assert!(!conflict.success);
        assert_eq!(
            conflict.activation_receipt_disposition,
            ActivationReceiptDisposition::Conflict as i32
        );
        assert!(service.has_activation_lifecycle_fence());

        let session_generation = handle
            .mapping_adapter
            .as_ref()
            .unwrap()
            .lock()
            .snapshot()
            .session_generation;
        let contaminated_revision = {
            let mut writer = handle.cloud_map_writer.lock();
            let writer = writer.as_mut().unwrap();
            assert!(writer.accumulate_mapping_frame(
                session_generation,
                &[Point3::new(12.025, 0.025, 0.025)]
            ));
            writer
                .commit_accumulated()
                .expect("contaminate mutable SHM after staged publication")
                .0
        };
        assert!(contaminated_revision > installed_revision);

        backend
            .fail_after_observe
            .store(false, std::sync::atomic::Ordering::Release);
        let replay = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("checkpoint publication replay")
            .into_inner();
        assert!(replay.idempotent_replay);
        assert!(!service.has_activation_lifecycle_fence());
        assert!(
            handle
                .mapping_lifecycle_lock
                .clone()
                .try_lock_owned()
                .is_ok(),
            "sealed retry must release lifecycle serialization"
        );
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            installed_revision
        );
        let published = backend
            .published
            .lock()
            .expect("checkpoint published frames");
        assert_eq!(published.len(), 2);
        assert_eq!(u64::from(published[0].header.seqno), installed_revision);
        assert_eq!(published[0].topic, published[1].topic);
        assert_eq!(published[0].msg_type, published[1].msg_type);
        assert_eq!(published[0].header, published[1].header);
        assert_eq!(published[0].payload, published[1].payload);
        let mut encoded = Vec::with_capacity(8 + published[0].payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&published[0].payload);
        let cloud = schema::decode_point_cloud2(&encoded).expect("replayed checkpoint PointCloud2");
        assert_eq!(cloud.data, cloud_xyz);
        drop(published);
        let _ = std::fs::remove_file(cloud_path);
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
            crate::server::production_shutdown_channel(lio_gate, correction_phase_a, None);
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
            crate::server::production_shutdown_channel(lio_gate, correction_phase_a, None);
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
        let (shutdown, _listener) = shutdown_channel_inner(None, None, None, Some(task_control));
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

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn phase_a_rejects_activate_map_cloud_clear_registration_without_sealing_success() {
        let mut spawned = crate::lcm_runtime::attach_root_task_owner_for_tests(
            crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("Geometry test handle"),
        );
        let handle = spawned.handle.clone();
        let task_control = handle.task_control().expect("Geometry task control");
        let (shutdown, _listener) = shutdown_channel_inner(None, None, None, Some(task_control));
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown.clone());
        let barrier = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_activate_map_before_cloud_clear_barrier(barrier.clone());
        let call_service = service.clone();
        let call = tokio::spawn(async move {
            GeometryRuntime::activate_map(
                &call_service,
                Request::new(activation_request("phase-a-activate", "phase-a-map")),
            )
            .await
        });

        barrier.wait().await;
        shutdown.begin_phase_a();
        barrier.wait().await;
        let terminal = call
            .await
            .expect("ActivateMap task join")
            .expect("activation failure is a stable terminal response")
            .into_inner();
        assert!(!terminal.success);
        assert_eq!(
            terminal.activation_receipt_disposition,
            ActivationReceiptDisposition::InvalidatedPostcommit as i32
        );
        assert!(terminal.message.contains("CLOUD_CLEAR_ADMISSION_FAILED"));
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

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_fails_closed_when_cloud_map_clear_is_unavailable() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        assert!(fixture.handle.cloud_map_writer.lock().is_none());
        let occupancy_generation_before = fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .session_generation();
        let mut cloud_updates = fixture.handle.cloud_map_tx.subscribe();
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let poison_order = {
            let replay = fixture.replay.clone();
            let readiness = readiness.clone();
            tokio::spawn(async move {
                replay.wait_poison().await;
                readiness.is_readiness_revoked()
            })
        };
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);
        let request = || StartMappingRequest {
            session_id: "deferred-clear-unavailable".to_owned(),
            map_name: "deferred-clear-map".to_owned(),
        };

        let first_error = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("missing deferred cloud-map clear must fail closed");
        assert_eq!(first_error.code(), tonic::Code::Unavailable);
        assert!(first_error.message().contains("cloud-map writer"));
        assert!(
            poison_order.await.expect("poison-order observer"),
            "readiness revocation must be visible before deferred poison"
        );
        assert!(readiness.is_readiness_revoked());
        assert!(
            readiness.has_shutdown_failed(),
            "deferred poison must remain sticky for main terminal arbitration"
        );
        assert_eq!(
            fixture.replay.poison_detail().as_deref(),
            Some(first_error.message())
        );
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_ingress.is_open());
        assert!(!fixture.correction_phase_a.is_open());
        let first_snapshot = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred clear mapping adapter")
            .lock()
            .snapshot();
        assert!(!first_snapshot.active);
        assert_eq!(
            first_snapshot.phase,
            crate::mapping_adapter::MappingSessionPhase::Stopped
        );
        assert!(first_snapshot.session_generation > 0);
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred clear mapping adapter")
                .lock();
            assert_eq!(adapter.odom_count(), 0);
            assert_eq!(adapter.pointcloud_count(), 0);
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            occupancy_generation_before,
            "failed deferred start must restore the occupancy session fence"
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        assert_eq!(fixture.backend.published_count("tf2_msgs.TFMessage"), 0);
        assert_eq!(
            fixture
                .correction_sink
                .apply_count
                .load(std::sync::atomic::Ordering::Acquire),
            0
        );
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.map.revision, 0);
        assert_eq!(shared.cloud_map_revision, 0);
        drop(shared);
        assert!(matches!(
            cloud_updates.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));

        let retry_error = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("poisoned exact retry must remain failed");
        assert_eq!(retry_error.code(), tonic::Code::Unavailable);
        assert_eq!(retry_error.message(), first_error.message());
        let retry_snapshot = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred clear mapping adapter")
            .lock()
            .snapshot();
        assert_eq!(
            retry_snapshot.session_generation,
            first_snapshot.session_generation
        );
        assert_eq!(retry_snapshot.phase, first_snapshot.phase);
        assert!(!retry_snapshot.active);
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        assert!(matches!(
            cloud_updates.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));

        let conflict_error = GeometryRuntime::start_mapping(
            &service,
            Request::new(StartMappingRequest {
                session_id: "deferred-clear-conflict".to_owned(),
                map_name: "deferred-clear-conflict-map".to_owned(),
            }),
        )
        .await
        .expect_err("poisoned conflicting retry must fail before a new Mapping session");
        assert_eq!(conflict_error.code(), tonic::Code::Unavailable);
        assert_eq!(conflict_error.message(), first_error.message());
        let conflict_snapshot = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred clear mapping adapter")
            .lock()
            .snapshot();
        assert_eq!(
            conflict_snapshot.session_generation,
            first_snapshot.session_generation
        );
        assert_eq!(conflict_snapshot.session_id, first_snapshot.session_id);
        assert!(!conflict_snapshot.active);

        let mismatch = CloudMapNotification {
            revision: 1,
            point_count: 0,
            voxel_size: 0.05,
            session_generation: first_snapshot.session_generation + 1,
        };
        assert!(validate_deferred_cloud_map_clear(
            &CloudMapSessionAdmission::Registered(mismatch),
            first_snapshot.session_generation
        )
        .is_err());

        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_stale_epoch_revokes_readiness_before_poison_and_retires() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        let (cloud_path, _generation_rx) =
            install_deferred_cloud_writer(&fixture.handle, "stale-epoch");
        let occupancy_generation_before = fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .session_generation();
        let captured = fixture.correction_ingress.relocalization_mode_snapshot();
        assert_eq!(captured.epoch, 0);
        assert!(!captured.active);
        let replay_epoch = fixture
            .correction_ingress
            .relocalization_mode_fence()
            .activate_for_tests();
        assert!(replay_epoch.active);
        assert_ne!(replay_epoch.epoch, captured.epoch);

        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let poison_order = {
            let replay = fixture.replay.clone();
            let readiness = readiness.clone();
            tokio::spawn(async move {
                let detail = replay.wait_poison().await;
                (detail, readiness.is_readiness_revoked())
            })
        };
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);
        let request = || StartMappingRequest {
            session_id: "deferred-stale-epoch".to_owned(),
            map_name: "deferred-stale-map".to_owned(),
        };

        let first_error = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("E0 retained cloud must fail the full StartMapping path at E1");
        assert_eq!(first_error.code(), tonic::Code::Unavailable);
        assert!(first_error
            .message()
            .contains("stale cloud mode provenance"));
        let (poison_detail, readiness_visible) = poison_order.await.expect("poison-order observer");
        assert_eq!(poison_detail, first_error.message());
        assert!(readiness_visible);
        assert!(readiness.is_readiness_revoked());
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_ingress.is_open());
        assert!(!fixture.correction_phase_a.is_open());
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        assert_eq!(
            fixture
                .correction_sink
                .apply_count
                .load(std::sync::atomic::Ordering::Acquire),
            0
        );

        let retry_error = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("stale-epoch exact retry must return the canonical poison");
        assert_eq!(retry_error.code(), first_error.code());
        assert_eq!(retry_error.message(), first_error.message());
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred mapping adapter")
                .lock();
            let snapshot = adapter.snapshot();
            assert!(!snapshot.active);
            assert_eq!(
                snapshot.phase,
                crate::mapping_adapter::MappingSessionPhase::Stopped
            );
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            occupancy_generation_before
        );

        cleanup_deferred_cloud_writer(&fixture.handle, &cloud_path);
        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_cancellation_during_cloud_clear_retires_total_state() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        let (cloud_path, mut generation_rx) =
            install_deferred_cloud_writer(&fixture.handle, "cancel-cloud-clear");
        let occupancy_generation_before = fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .session_generation();
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);
        let barrier = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_start_mapping_before_cloud_clear_barrier(barrier.clone());
        let request = || StartMappingRequest {
            session_id: "deferred-cancel-cloud-clear".to_owned(),
            map_name: "deferred-cancel-cloud-map".to_owned(),
        };
        let rpc_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_mapping(
                    &service,
                    Request::new(StartMappingRequest {
                        session_id: "deferred-cancel-cloud-clear".to_owned(),
                        map_name: "deferred-cancel-cloud-map".to_owned(),
                    }),
                )
                .await
            })
        };

        barrier.wait().await;
        let shared_state_guard = fixture.handle.state.write().await;
        barrier.wait().await;
        let started_generation = *tokio::time::timeout(
            Duration::from_secs(1),
            generation_rx.wait_for(|generation| *generation > 0),
        )
        .await
        .expect("cloud writer generation must advance before waiter cancellation")
        .expect("cloud writer generation watch");
        assert!(started_generation > 0);

        rpc_task.abort();
        assert!(rpc_task
            .await
            .expect_err("aborted StartMapping RPC waiter")
            .is_cancelled());
        drop(shared_state_guard);
        let poison_detail =
            tokio::time::timeout(Duration::from_secs(1), fixture.replay.wait_poison())
                .await
                .expect("cloud-clear cancellation poison deadline");
        assert!(poison_detail.contains("cancelled during cloud-map clear"));
        assert!(readiness.is_readiness_revoked());

        let retry = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("cancelled cloud-clear retry must expose settled poison");
        assert_eq!(retry.code(), tonic::Code::Unavailable);
        assert_eq!(retry.message(), poison_detail);
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred mapping adapter")
                .lock();
            let snapshot = adapter.snapshot();
            assert!(!snapshot.active);
            assert_eq!(
                snapshot.phase,
                crate::mapping_adapter::MappingSessionPhase::Stopped
            );
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            occupancy_generation_before
        );
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_ingress.is_open());

        cleanup_deferred_cloud_writer(&fixture.handle, &cloud_path);
        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_cancellation_during_replay_retires_total_state() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        let (cloud_path, _generation_rx) =
            install_deferred_cloud_writer(&fixture.handle, "cancel-replay");
        let occupancy_generation_before = fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .session_generation();
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);
        let request = || StartMappingRequest {
            session_id: "deferred-cancel-replay".to_owned(),
            map_name: "deferred-cancel-replay-map".to_owned(),
        };
        let rpc_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_mapping(
                    &service,
                    Request::new(StartMappingRequest {
                        session_id: "deferred-cancel-replay".to_owned(),
                        map_name: "deferred-cancel-replay-map".to_owned(),
                    }),
                )
                .await
            })
        };

        let cloud = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = fixture
                    .frame_rx
                    .recv()
                    .await
                    .expect("deferred replay publication frame");
                if frame.msg_type.fqn() == "nuwax_msgs.LioCloudEnvelope" {
                    return frame;
                }
            }
        })
        .await
        .expect("public cloud replay wait window");
        assert_eq!(cloud.msg_type.fqn(), "nuwax_msgs.LioCloudEnvelope");

        rpc_task.abort();
        assert!(rpc_task
            .await
            .expect_err("aborted replay RPC waiter")
            .is_cancelled());
        let poison_detail =
            tokio::time::timeout(Duration::from_secs(1), fixture.replay.wait_poison())
                .await
                .expect("replay cancellation poison deadline");
        assert!(poison_detail.contains("cancelled during retained replay"));
        assert!(readiness.is_readiness_revoked());

        let retry = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("cancelled replay retry must expose settled poison");
        assert_eq!(retry.code(), tonic::Code::Unavailable);
        assert_eq!(retry.message(), poison_detail);
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred mapping adapter")
                .lock();
            let snapshot = adapter.snapshot();
            assert!(!snapshot.active);
            assert_eq!(
                snapshot.phase,
                crate::mapping_adapter::MappingSessionPhase::Stopped
            );
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            occupancy_generation_before
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            1
        );
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_ingress.is_open());

        cleanup_deferred_cloud_writer(&fixture.handle, &cloud_path);
        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_retires_backend_when_ordinary_stop_transition_fails() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        let occupancy_generation_before = fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .session_generation();
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);
        service.inject_start_mapping_stop_transition_failure();
        let request = || StartMappingRequest {
            session_id: "deferred-stop-fallback".to_owned(),
            map_name: "deferred-stop-fallback-map".to_owned(),
        };

        let error = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("missing clear plus ordinary stop failure must remain total");

        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(error.message().contains("cloud-map writer"));
        assert!(!error
            .message()
            .contains("ordinary Mapping stop transition failed"));
        let retry = GeometryRuntime::start_mapping(&service, Request::new(request()))
            .await
            .expect_err("ordinary-stop fallback retry must remain canonical");
        assert_eq!(retry.code(), error.code());
        assert_eq!(retry.message(), error.message());
        assert!(fixture
            .replay
            .poison_diagnostics_for_tests()
            .iter()
            .any(|diagnostic| diagnostic.contains("ordinary Mapping stop transition failed")));
        assert!(readiness.is_readiness_revoked());
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred mapping adapter")
                .lock();
            let snapshot = adapter.snapshot();
            assert!(!snapshot.active);
            assert_eq!(
                snapshot.phase,
                crate::mapping_adapter::MappingSessionPhase::Stopped
            );
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            occupancy_generation_before
        );

        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_occupancy_fence_failure_revokes_readiness_and_retires_mapping() {
        let mut fixture = DeferredClearFixture::new().await;
        fixture.complete_startup_odometry().await;
        fixture
            .handle
            .occupancy_shm
            .as_ref()
            .expect("deferred fixture occupancy writer")
            .lock()
            .begin_session(u64::MAX)
            .expect("install future occupancy generation");
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let readiness = shutdown.clone();
        let service = GeometryRuntimeService::new_with_shutdown(fixture.handle.clone(), shutdown);

        let error = GeometryRuntime::start_mapping(
            &service,
            Request::new(StartMappingRequest {
                session_id: "deferred-occupancy-fence-failure".to_owned(),
                map_name: "deferred-occupancy-fence-map".to_owned(),
            }),
        )
        .await
        .expect_err("regressed occupancy generation must fail closed");

        assert_eq!(error.code(), tonic::Code::Unavailable);
        assert!(error
            .message()
            .contains("occupancy SHM session fence failed"));
        assert!(readiness.is_readiness_revoked());
        assert!(fixture.replay.poison_detail().is_some());
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred mapping adapter")
                .lock();
            assert!(!adapter.snapshot().active);
            assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
        }
        assert_eq!(
            fixture
                .handle
                .occupancy_shm
                .as_ref()
                .expect("deferred fixture occupancy writer")
                .lock()
                .session_generation(),
            u64::MAX
        );

        drop(service);
        drop(listener);
        fixture.shutdown().await;
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
            None,
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
        assert!(error.message().contains("mapping lifecycle lock"));
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
            motion_guard_generation: 7,
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
            motion_guard_generation: 7,
            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
            ..ExploreRequest::default()
        };

        assert!(explore_config_from_request(&request, "robot/map").is_ok());
        request.boundary_frame_id = "map".to_string();
        let error = explore_config_from_request(&request, "robot/map")
            .expect_err("literal map must not bypass the configured frame");
        assert!(error.contains("robot/map"));
    }

    #[test]
    fn exploration_state_preserves_child_identity() {
        let proto =
            exploration_state_from_snapshot(crate::explore_runtime::ExploreRuntimeSnapshot {
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
                    goal_payload_sha256: [7; 32],
                }),
                rollover_recovery_eligible: true,
                global_stop_reconciliation:
                    crate::explore_runtime::ExplorationGlobalStopReconciliationSnapshot::default(),
                ownership_reconciliation_receipt: None,
            });
        assert_eq!(proto.child_task_id, "child-3");
        assert_eq!(proto.exploration_generation, 12);
        assert!(proto.rollover_recovery_eligible);
        let child = proto.child_lease.expect("child lease");
        assert_eq!(child.producer_runtime_generation, 5);
        assert_eq!(child.goal_payload_sha256, vec![7; 32]);
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
                cancel_phase: "ZEROED".into(),
                cancel_generation: 3,
                actuator_zeroed: true,
                reason_code: "CANCEL_ZEROED".into(),
                runtime_generation: 7,
                guard_generation: 11,
                sink_ready: true,
                planner_map_frame: "map".into(),
                planner_pose_frame: "map".into(),
                zero_ack_id: "zero-3".into(),
                zero_applied_monotonic_ns: 123,
                owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                parent_request_id: "request-7".into(),
                goal_generation: 5,
                path_revision: 2,
                goal_payload_sha256: [7; 32],
                path_payload_sha256: [8; 32],
            },
        });

        assert_eq!(proto.cancel_phase, "ZEROED");
        assert_eq!(proto.cancel_generation, 3);
        assert!(proto.actuator_zeroed);
        assert_eq!(proto.reason_code, "CANCEL_ZEROED");
        assert_eq!(proto.runtime_generation, 7);
        assert_eq!(proto.guard_generation, 11);
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
        assert_eq!(proto.goal_payload_sha256, vec![7; 32]);
        assert_eq!(proto.path_payload_sha256, vec![8; 32]);
        assert_eq!(proto.producer_runtime_generation, 41);
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
                motion_guard_generation: 7,
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
                motion_guard_generation: 7,
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
        assert_eq!(goal.goal_payload_sha256.len(), 32);

        let wrong_producer_runtime_generation = if goal.producer_runtime_generation == 1 {
            2
        } else {
            goal.producer_runtime_generation - 1
        };
        let stale_cancel = GeometryRuntime::cancel_navigation(
            &service,
            Request::new(CancelNavigationRequest {
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
        assert_eq!(cancelled.goal_payload_sha256, goal.goal_payload_sha256);

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
        let (shutdown, _listener) = shutdown_channel_inner(None, Some(phase_a), None, None);
        GeometryRuntimeService::new_with_shutdown_and_point_cloud_binding(
            handle,
            shutdown,
            None,
            Some(ingress),
        )
    }

    fn query_request(
        request_id: &str,
        request_identity_sha256: Vec<u8>,
        activation_identity_sha256: Vec<u8>,
    ) -> GetRelocalizationTerminalRequest {
        GetRelocalizationTerminalRequest {
            schema_version: 1,
            request_id: request_id.to_owned(),
            request_identity_sha256,
            activation_identity_sha256,
        }
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
        let root = query_root("invalid");
        let ingress = crate::correction_coordinator::RelocalizationCommandIngress::terminal_query_fixture_for_tests(root.clone());
        let service = query_service(ingress).await;
        for request in [
            query_request(" invalid", vec![3; 32], vec![5; 32]),
            query_request("req-invalid-request-digest", vec![3; 31], vec![5; 32]),
            query_request("req-invalid-activation-digest", vec![3; 32], vec![5; 33]),
        ] {
            let error =
                GeometryRuntime::get_relocalization_terminal(&service, Request::new(request))
                    .await
                    .expect_err("invalid terminal query must fail closed");
            assert_eq!(error.code(), tonic::Code::InvalidArgument);
        }
        std::fs::remove_dir_all(root).expect("remove terminal query store");
    }

    #[tokio::test]
    async fn terminal_query_projects_four_read_only_states_without_mutating_runtime() {
        let root = query_root("states");
        let ingress = crate::correction_coordinator::RelocalizationCommandIngress::terminal_query_fixture_for_tests(root.clone());
        let request_identity = [3_u8; 32];
        let activation_identity = [5_u8; 32];
        ingress.mark_terminal_query_pending_for_tests(
            "req-pending",
            request_identity,
            activation_identity,
        );
        ingress.persist_terminal_query_result_for_tests(RelocalizeResult {
            schema_version: 1,
            request_id: "req-terminal".to_owned(),
            request_identity_sha256: request_identity.to_vec(),
            activation_identity_sha256: activation_identity.to_vec(),
            disposition: RelocalizeDisposition::Applied as i32,
            terminal_proof_sha256: vec![7; 32],
            ..RelocalizeResult::default()
        });
        let service = query_service(ingress.clone()).await;
        let map_revision_before = service.bridge.state.read().await.map.revision;

        for (request_id, request_digest, expected_status, has_terminal) in [
            (
                "req-terminal",
                request_identity,
                RelocalizationTerminalStatus::Terminal,
                true,
            ),
            (
                "req-pending",
                request_identity,
                RelocalizationTerminalStatus::Pending,
                false,
            ),
            (
                "req-missing",
                request_identity,
                RelocalizationTerminalStatus::NotFound,
                false,
            ),
            (
                "req-terminal",
                [9; 32],
                RelocalizationTerminalStatus::IdentityConflict,
                false,
            ),
        ] {
            let response = GeometryRuntime::get_relocalization_terminal(
                &service,
                Request::new(query_request(
                    request_id,
                    request_digest.to_vec(),
                    activation_identity.to_vec(),
                )),
            )
            .await
            .expect("terminal query response")
            .into_inner();
            assert_eq!(response.status, expected_status as i32);
            assert_eq!(response.terminal.is_some(), has_terminal);
            let observation = response
                .ledger_observation
                .expect("successful query must include ledger observation");
            assert_eq!(observation.observation_schema_version, 1);
            assert_ne!(observation.observation_epoch, 0);
            assert_eq!(observation.admitted_command_count, 0);
            assert_eq!(observation.claimed_command_count, 0);
            assert_eq!(observation.terminal_record_count, 0);
            assert_eq!(observation.pending_record_count, 0);
            assert_eq!(observation.active_waiter_count, 0);
        }

        assert_eq!(
            service.bridge.state.read().await.map.revision,
            map_revision_before
        );
        assert!(matches!(
            ingress
                .terminal_snapshot("req-pending", request_identity, activation_identity)
                .await
                .expect("terminal snapshot")
                .terminal,
            crate::correction_coordinator::RelocalizationTerminalSnapshot::Pending
        ));
        std::fs::remove_dir_all(root).expect("remove terminal query store");
    }
}
