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

use crate::activation_contract::{ActivationRequestKey, ActivationIdentity, TargetReceipt, ActivationJournalKey, ActivationJournalVerifier, ActivationJournalVerifierV2, ActivationTargetSource, InstalledActivationIdentityV2, InstalledTargetReceiptV2};
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
use nuwax_mapping_cloud::{
    ActiveMapIdentityV2, CloudQueryIdentityV1, GeoreferenceBindingV1, MappingCloudOwner,
    MappingCloudProvider, MappingCloudRequest, MappingCloudTerminal, OsmClient, Sha256Digest,
    VerifiedGeoreferenceV1,
};
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
    /// Native mode publishes raw cloud and IMU to generation-private channels.
    native_raw_input_binding: Option<((LcmTopic, LcmMessageType), (LcmTopic, LcmMessageType))>,
    native_raw_sensor_frame: Option<String>,
    native_input_failure: Arc<parking_lot::Mutex<Option<String>>>,
    native_input_delivery: Option<Arc<crate::native_input_delivery::NativeInputDelivery>>,
    native_self_filter: Arc<Result<Option<crate::self_cloud::NativeSelfFilter>, String>>,
    shutdown: Option<ShutdownCoordinator>,
    mapping_transition: Arc<tokio::sync::Mutex<()>>,
    graph_artifacts: Arc<parking_lot::Mutex<crate::planar_artifacts::Store>>,
    deferred_replay: Option<Arc<crate::lio_ingress::NativeDeferredReplayControl>>,
    activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl,
    relocalization_ingress: Option<crate::correction_coordinator::RelocalizationCommandIngress>,
    canonical_correction: Option<Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>>,
    mapping_cloud_owner: Arc<MappingCloudOwner>,
    activation_lifecycle_fence: Arc<parking_lot::Mutex<Option<ActivationLifecycleFenceSlot>>>,
    activation_checkpoint_owners: Arc<crate::activation_request::CheckpointOwnerInternerV2>,
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
    stop_before_post_zero_lifecycle: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    retained_receipt_consume_count: Arc<std::sync::atomic::AtomicUsize>,
    #[cfg(test)]
    retained_receipt_consumed: Arc<tokio::sync::Notify>,
    #[cfg(test)]
    stop_after_retained_receipt_consume: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    stop_before_graceful: Arc<parking_lot::Mutex<Option<Arc<tokio::sync::Barrier>>>>,
}

struct ActivationLifecycleFence {
    activation_request_id: String,
    activation_request_key: ActivationRequestKey,
    retained_global_map: Option<RetainedActivationGlobalMap>,
    projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
    lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
}

enum ActivationLifecycleFenceSlot {
    Held(ActivationLifecycleFence),
    RestorePending(RestorePendingActivationLifecycleFence),
}

struct RestorePendingActivationLifecycleFence {
    activation_request_id: String,
    activation_request_key: ActivationRequestKey,
    retained_global_map: Option<RetainedActivationGlobalMap>,
    projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
    reason: String,
}

struct RetainedActivationGlobalMap {
    activation_request_key: ActivationRequestKey,
    map_frame_id: String,
    installed_cloud_revision: u64,
    cloud_metadata: crate::pointcloud::CommittedCloudMapMetadata,
    cloud_receipt: crate::mapping_commit::CloudCommitReceipt,
    legacy_frame: Option<RawLcmFrame>,
}

impl RetainedActivationGlobalMap {
    fn from_committed(
        activation_request_key: impl Into<ActivationRequestKey>,
        map_frame_id: &str,
        committed: &CommittedMappingCloudMap,
    ) -> Result<Self, String> {
        let activation_request_key = activation_request_key.into();
        Ok(Self {
            activation_request_key: activation_request_key.clone(),
            map_frame_id: map_frame_id.to_owned(),
            installed_cloud_revision: committed.notification.revision,
            cloud_metadata: committed.metadata,
            cloud_receipt: committed.receipt.clone(),
            // The request bytes may be non-center coordinates. Retain only
            // metadata until a failed publication exposes its canonical cache.
            legacy_frame: None,
        })
    }

    fn capture_materialized_legacy_frame(
        &mut self,
        writer: &Arc<parking_lot::Mutex<Option<crate::pointcloud::CloudMapShmWriter>>>,
        committed: &CommittedMappingCloudMap,
    ) -> Result<(), String> {
        if self.legacy_frame.is_some()
            || !matches!(self.activation_request_key, ActivationRequestKey::LegacyDigest(_))
        {
            return Ok(());
        }
        self.validate_replay(self.activation_request_key.clone(), &self.map_frame_id, committed)?;
        let snapshot = {
            let guard = writer.lock();
            let owner = guard.as_ref().ok_or("retained publication writer is unavailable")?;
            let metadata = owner.committed_metadata()
                .ok_or("retained publication metadata is unavailable")?;
            if !crate::lcm_runtime::same_cloud_metadata(&metadata, &self.cloud_metadata)
                || owner.last_cloud_receipt().as_ref() != Some(&self.cloud_receipt)
            {
                return Err("retained publication current metadata/receipt changed".into());
            }
            // Never initialize the full export cache here. When no legacy
            // payload was exported, retry through the canonical sequencer.
            let Some(snapshot) = owner.materialized_committed_snapshot() else {
                return Ok(());
            };
            let cached_metadata = crate::pointcloud::CommittedCloudMapMetadata {
                revision: snapshot.revision, point_count: snapshot.point_count,
                voxel_size: snapshot.voxel_size, session_generation: snapshot.session_generation,
                producer_epoch: snapshot.producer_epoch, layout_generation: snapshot.layout_generation,
            };
            if !crate::lcm_runtime::same_cloud_metadata(&cached_metadata, &self.cloud_metadata) {
                return Err("retained publication cached metadata changed".into());
            }
            snapshot
        };
        // The existing encoded retention buffer is built from the already
        // shared actual publication bytes, outside the writer lock, only once.
        self.legacy_frame = Some(build_committed_global_map_frame_in_frame(
            committed, &snapshot.xyz_f32_le, &self.map_frame_id,
        )?);
        Ok(())
    }

    fn validate_replay(
        &self,
        activation_request_key: impl Into<ActivationRequestKey>,
        map_frame_id: &str,
        committed: &CommittedMappingCloudMap,
    ) -> Result<(), String> {
        let activation_request_key = activation_request_key.into();
        if self.activation_request_key != activation_request_key
            || self.map_frame_id != map_frame_id
            || self.installed_cloud_revision != committed.notification.revision
            || self.cloud_receipt != committed.receipt
            || !crate::lcm_runtime::same_cloud_metadata(&self.cloud_metadata, &committed.metadata)
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

    #[cfg(test)]
    fn inject_correction_settlement_blocker(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.correction_settlement_blocker.lock() = Some(barrier);
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
            deferred_replay: None,
            activation_receipts: crate::correction_coordinator::ActivatedTargetReceiptControl::new(
                1,
            ),
            relocalization_ingress: None,
            canonical_correction: None,
            mapping_cloud_owner,
            activation_lifecycle_fence: Arc::new(parking_lot::Mutex::new(None)),
            activation_checkpoint_owners: Arc::new(crate::activation_request::CheckpointOwnerInternerV2::default()),
            activation_fault_after: Arc::new(parking_lot::Mutex::new(None)),
            stop_phase_a_after_detach: Arc::new(AtomicBool::new(false)),
            fail_start_mapping_stop_transition: Arc::new(AtomicBool::new(false)),
            start_mapping_before_cloud_clear: Arc::new(parking_lot::Mutex::new(None)),
            stop_before_post_zero_lifecycle: Arc::new(parking_lot::Mutex::new(None)),
            retained_receipt_consume_count: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            retained_receipt_consumed: Arc::new(tokio::sync::Notify::new()),
            stop_after_retained_receipt_consume: Arc::new(parking_lot::Mutex::new(None)),
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
            native_self_filter: Arc::new(crate::self_cloud::NativeSelfFilter::from_env()),
            mapping_transition: shutdown.mapping_transition.clone(),
            graph_artifacts: Arc::new(parking_lot::Mutex::new(crate::planar_artifacts::Store::default())),
            deferred_replay: shutdown.deferred_replay.clone(),
            shutdown: Some(shutdown),
            activation_receipts,
            relocalization_ingress,
            canonical_correction: None,
            mapping_cloud_owner,
            activation_lifecycle_fence: Arc::new(parking_lot::Mutex::new(None)),
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
            retained_receipt_consume_count: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            #[cfg(test)]
            retained_receipt_consumed: Arc::new(tokio::sync::Notify::new()),
            #[cfg(test)]
            stop_after_retained_receipt_consume: Arc::new(parking_lot::Mutex::new(None)),
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

    fn take_matching_activation_lifecycle_fence(
        &self,
        activation_request_id: &str,
        activation_request_key: impl Into<ActivationRequestKey>,
    ) -> Result<Option<ActivationLifecycleFence>, &'static str> {
        let activation_request_key = activation_request_key.into();
        let mut fence = self.activation_lifecycle_fence.lock();
        match fence.as_ref() {
            None => Ok(None),
            Some(ActivationLifecycleFenceSlot::Held(existing))
                if existing.activation_request_id == activation_request_id
                    && existing.activation_request_key
                        == activation_request_key =>
            {
                match fence.take() {
                    Some(ActivationLifecycleFenceSlot::Held(fence)) => Ok(Some(fence)),
                    _ => unreachable!("matched activation lifecycle fence changed while locked"),
                }
            }
            Some(ActivationLifecycleFenceSlot::Held(_)) => Err("ACTIVATE_MAP_LIFECYCLE_FENCE_BUSY"),
            Some(ActivationLifecycleFenceSlot::RestorePending(_)) => {
                Err("ACTIVATION_LIFECYCLE_FENCE_RESTORE_PENDING")
            }
        }
    }

    fn retain_activation_lifecycle_fence(
        &self,
        activation_request_id: &str,
        activation_request_key: impl Into<ActivationRequestKey>,
        retained_global_map: Option<RetainedActivationGlobalMap>,
        projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
        lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
    ) -> Result<
        (),
        (
            crate::lcm_runtime::ExploreProjectionOwnedGuard,
            tokio::sync::OwnedMutexGuard<()>,
        ),
    > {
        let activation_request_key = activation_request_key.into();
        let mut fence = self.activation_lifecycle_fence.lock();
        if fence.is_some() {
            return Err((projection_guard, lifecycle_guard));
        }
        *fence = Some(ActivationLifecycleFenceSlot::Held(
            ActivationLifecycleFence {
                activation_request_id: activation_request_id.to_owned(),
                activation_request_key,
                retained_global_map,
                projection_guard,
                lifecycle_guard,
            },
        ));
        Ok(())
    }

    #[allow(clippy::result_large_err)]
    fn take_activation_lifecycle_fence(&self) -> Result<Option<ActivationLifecycleFence>, Status> {
        let mut slot = self.activation_lifecycle_fence.lock();
        match slot.as_ref() {
            None => Ok(None),
            Some(ActivationLifecycleFenceSlot::Held(_)) => match slot.take() {
                Some(ActivationLifecycleFenceSlot::Held(fence)) => Ok(Some(fence)),
                _ => unreachable!("matched activation lifecycle fence changed while locked"),
            },
            Some(ActivationLifecycleFenceSlot::RestorePending(pending)) => {
                let _retained_projection_guard = &pending.projection_guard;
                let _activation_request_key =
                    &pending.activation_request_key;
                Err(Status::failed_precondition(format!(
                    "ACTIVATION_LIFECYCLE_FENCE_RESTORE_PENDING: activation_request_id={} retained_global_map={} reason={}",
                    pending.activation_request_id,
                    pending.retained_global_map.is_some(),
                    pending.reason
                )))
            }
        }
    }

    fn restore_activation_lifecycle_fence(&self, fence: ActivationLifecycleFence) {
        let mut slot = self.activation_lifecycle_fence.lock();
        assert!(
            slot.is_none(),
            "activation lifecycle fence slot changed while Mapping transition was held"
        );
        *slot = Some(ActivationLifecycleFenceSlot::Held(fence));
    }

    fn restore_pending_activation_lifecycle_fence(
        &self,
        fence: RestorePendingActivationLifecycleFence,
    ) {
        let mut slot = self.activation_lifecycle_fence.lock();
        assert!(
            slot.is_none(),
            "activation lifecycle fence slot changed while Mapping transition was held"
        );
        *slot = Some(ActivationLifecycleFenceSlot::RestorePending(fence));
    }

    fn has_activation_lifecycle_fence(&self) -> bool {
        self.activation_lifecycle_fence.lock().is_some()
    }

    fn consume_retained_activation_receipt_after_motion_terminal(
        &self,
        retained_metadata: Option<(String, ActivationRequestKey, Option<RetainedActivationGlobalMap>)>,
    ) {
        #[cfg(test)]
        let consumed_retained_receipt = retained_metadata.is_some();
        drop(retained_metadata);
        #[cfg(test)]
        if consumed_retained_receipt {
            self.retained_receipt_consume_count
                .fetch_add(1, Ordering::AcqRel);
            self.retained_receipt_consumed.notify_one();
        }
    }

    async fn settle_process_stop_failure(
        &self,
        retained_metadata: Option<(String, ActivationRequestKey, Option<RetainedActivationGlobalMap>)>,
        projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
        error: Status,
    ) -> Status {
        let Some((activation_request_id, activation_request_key, retained_global_map)) =
            retained_metadata
        else {
            return error;
        };
        match tokio::time::timeout(
            STOP_LIFECYCLE_TIMEOUT,
            self.bridge.mapping_lifecycle_lock.clone().lock_owned(),
        )
        .await
        {
            Ok(lifecycle_guard) => {
                self.restore_activation_lifecycle_fence(ActivationLifecycleFence {
                    activation_request_id,
                    activation_request_key,
                    retained_global_map,
                    projection_guard,
                    lifecycle_guard,
                });
                error
            }
            Err(_) => {
                let original_detail = error.message().to_owned();
                self.restore_pending_activation_lifecycle_fence(
                    RestorePendingActivationLifecycleFence {
                        activation_request_id,
                        activation_request_key,
                        retained_global_map,
                        projection_guard,
                        reason: original_detail.clone(),
                    },
                );
                Status::deadline_exceeded(format!(
                    "ACTIVATION_LIFECYCLE_FENCE_RESTORE_PENDING: timed out restoring mapping lifecycle fence after Process Stop failure; original={original_detail}"
                ))
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    async fn finish_started_mapping_transition(
        &self,
        request: StartMappingRequest,
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        snapshot: MappingSessionSnapshot,
        cloud_map_reservation: CloudMapSessionReservation,
        mut projection_guard: crate::lcm_runtime::ExploreProjectionOwnedGuard,
        lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        _mapping_transition: tokio::sync::OwnedMutexGuard<()>,
        mut cancellation_rx: watch::Receiver<bool>,
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

        let deferred_replay = self.deferred_replay.clone();
        let mut occupancy_fence = None;
        if let Some(replay) = deferred_replay.as_deref() {
            if *cancellation_rx.borrow() {
                return Err(finish_deferred_mapping_start_failure(
                    replay,
                    &adapter,
                    &self.bridge.mapping_lifecycle_lock,
                    self.bridge
                        .mapping_lifecycle_lock
                        .clone()
                        .lock_owned()
                        .await,
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
                            self.bridge
                                .mapping_lifecycle_lock
                                .clone()
                                .lock_owned()
                                .await,
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
                        self.bridge
                            .mapping_lifecycle_lock
                            .clone()
                            .lock_owned()
                            .await,
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
                        self.bridge
                            .mapping_lifecycle_lock
                            .clone()
                            .lock_owned()
                            .await,
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
                    self.bridge
                        .mapping_lifecycle_lock
                        .clone()
                        .lock_owned()
                        .await,
                    &snapshot,
                    occupancy_fence.take(),
                    format!("deferred StartMapping cloud-map clear failed: {clear_error}"),
                )
                .await);
            }
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
    fn inject_stop_before_post_zero_lifecycle_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.stop_before_post_zero_lifecycle.lock() = Some(barrier);
    }

    #[cfg(test)]
    fn inject_stop_after_retained_receipt_consume_barrier(
        &self,
        barrier: Arc<tokio::sync::Barrier>,
    ) {
        *self.stop_after_retained_receipt_consume.lock() = Some(barrier);
    }

    #[cfg(test)]
    fn retained_receipt_consume_count(&self) -> usize {
        self.retained_receipt_consume_count.load(Ordering::Acquire)
    }

    #[cfg(test)]
    async fn wait_for_retained_receipt_consume(&self) {
        self.retained_receipt_consumed.notified().await;
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
        RelocalizationCommandError::InvalidRequest(detail) => return Status::failed_precondition(detail),
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
        target_kind:String::new(),target_graph_sha256:Vec::new(),planar_mode:String::new(),
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
        activation_receipt_v2: None,
        relocalization_policy_v2: None,
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
    Assigned(crate::activation_request::ActivationRequestMetadataV2),
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
            Self::Assigned(metadata) => metadata.activation_request_id(),
        }
    }

    const fn is_modern(&self) -> bool {
        matches!(self, Self::Modern { .. } | Self::Assigned(_))
    }

    const fn asset_manifest_sha256(&self) -> Option<[u8; 32]> {
        match self {
            Self::Modern {
                asset_manifest_sha256,
                ..
            } => Some(*asset_manifest_sha256),
            Self::Legacy { .. } | Self::Assigned(_) => None,
        }
    }

    const fn asset_bundle_sha256(&self) -> Option<[u8; 32]> {
        match self {
            Self::Modern {
                asset_bundle_sha256,
                ..
            } => Some(*asset_bundle_sha256),
            Self::Legacy { .. } | Self::Assigned(_) => None,
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
    if request.activation_contract_version == 2 {
        return crate::activation_request::ActivationRequestMetadataV2::from_wire(request, runtime_generation)
            .map(ActivationRequestIdentityClass::Assigned);
    }
    let legacy = request.activation_contract_version == 0
        && request.asset_identity.is_none()
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
        && request.asset_identity.is_none()
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
        checkpoint.frame_id.trim()
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
    if checkpoint.backend_kind=="slam_toolbox" {
        validate_planar_checkpoint(checkpoint)?;
        let mut out=b"nuwax.geometry.planar-backend-checkpoint.v1\0".to_vec();
        out.extend_from_slice(&checkpoint.schema_version.to_be_bytes());
        canonical_push_string(&mut out,&checkpoint.backend_kind)?;
        canonical_push_string(&mut out,&checkpoint.source_session_id)?;
        for n in [checkpoint.source_session_generation,checkpoint.payload_size,checkpoint.created_at_ns] {out.extend_from_slice(&n.to_be_bytes());}
        out.extend_from_slice(&checkpoint.content_sha256);
        canonical_append_occupancy_receipt(&mut out,checkpoint.occupancy_receipt.as_ref().unwrap())?;
        out.extend_from_slice(&canonical_occupancy_sha256(checkpoint.canonical_occupancy_checkpoint.as_ref().unwrap())?);
        return Ok(crate::map_checksum::sha256_bytes(&out));
    }
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
    if let Some(fine) = &checkpoint.fine_map_artifact {
        out.extend_from_slice(b"nuwax.geometry.fine-map-archive.v1\0");
        out.extend_from_slice(&fine.size.to_be_bytes());
        out.extend_from_slice(&canonical_hash32(&fine.sha256, "fine_map_artifact.sha256")?);
    }
    if let Some(dense) = &checkpoint.dense_cloud_artifact {
        out.extend_from_slice(b"nuwax.geometry.dense-cloud-archive.v1\0");
        out.extend_from_slice(&dense.size.to_be_bytes());
        out.extend_from_slice(&canonical_hash32(&dense.sha256, "dense_cloud_artifact.sha256")?);
    }
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
        &crate::correction_coordinator::RELOCALIZATION_SOURCE_POLICY_VERSION.to_be_bytes(),
    );
    out.extend_from_slice(&if backend_kind=="slam_toolbox" {crate::correction_coordinator::planar_source_policy_sha256()}else{crate::correction_coordinator::relocalization_source_policy_sha256_v1()});
    out.extend_from_slice(
        &crate::correction_coordinator::RELOCALIZATION_DEADLINE_POLICY_VERSION_V1.to_be_bytes(),
    );
    out.extend_from_slice(
        &crate::correction_coordinator::relocalization_deadline_policy_sha256_v1(),
    );
    out.extend_from_slice(
        &crate::correction_coordinator::RELOCALIZATION_ALGORITHM_OPTIONS_VERSION.to_be_bytes(),
    );
    out.extend_from_slice(
        &if backend_kind=="slam_toolbox" {crate::correction_coordinator::planar_algorithm_policy_sha256()}else{crate::correction_coordinator::relocalization_algorithm_options_sha256_v1()},
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
    request_identity: impl Into<ActivationRequestKey>,
    step: crate::correction_coordinator::ActivationJournalStep,
    status: crate::correction_coordinator::ActivationJournalStatus,
    verifier_receipt: Option<ActivationJournalVerifier>,
    typed_failure: Option<String>,
) -> bool {
    let idempotency_key = match request_identity.into() {
        ActivationRequestKey::LegacyDigest(digest) => activation_journal_key(digest, step).into(),
        ActivationRequestKey::Assigned(request) => ActivationJournalKey::Assigned { request, step },
    };
    control.journal(token, crate::correction_coordinator::ActivationJournalEntry {
        step, idempotency_key, status, verifier_receipt, typed_failure,
    })
}

fn activation_verifier(identity: &ActivationIdentity) -> ActivationJournalVerifier {
    match identity {
        ActivationIdentity::LegacyDigest(digest) => (*digest).into(),
        ActivationIdentity::Assigned(identity) => ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::Activation(identity.clone())),
    }
}

fn cloud_activation_verifier(key: &ActivationRequestKey, legacy: Option<[u8; 32]>, receipt: &crate::mapping_commit::CloudCommitReceipt) -> Option<ActivationJournalVerifier> {
    match key {
        ActivationRequestKey::LegacyDigest(_) => legacy.map(Into::into),
        ActivationRequestKey::Assigned(_) => Some(ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::Cloud(receipt.clone()))),
    }
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
    request_identity: ActivationRequestKey,
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
        let retirement_proof = match &request_identity {
            ActivationRequestKey::LegacyDigest(_) => crate::map_checksum::sha256_bytes(&retirement_generation.to_be_bytes()).into(),
            ActivationRequestKey::Assigned(_) => ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::BackendGeneration(retirement_generation)),
        };
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

    if selected_backend_kind==crate::mapping_backend::MappingBackendKind::SlamToolbox {
        if matches!(identity_class, ActivationRequestIdentityClass::Assigned(_)) {
            return Err(Status::failed_precondition("PLANAR_ASSIGNED_ACTIVATION_NOT_PAIRED"));
        }
        return service.activate_planar_map(request,identity_class).await;
    }

    let (session_id, map_name, checkpoint, checkpoint_timestamp_ns, mapping_backend_checkpoint,
         assigned_checkpoint, immutable_target, occupancy_sha256, cloud_sha256, backend_sha256,
         prepared_fine, prepared_dense, seed, activation_request_key) =
    if let ActivationRequestIdentityClass::Assigned(metadata) = &identity_class {
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
        let key = ActivationRequestKey::Assigned(metadata.clone());
        let seed = crate::correction_coordinator::ActivationReceiptSeed {
            planar_target: None, activation_request_id: metadata.activation_request_id().to_owned(),
            request_key: key.clone(), checkpoint_owner: Some(owner.clone()),
            asset_manifest_sha256: None, asset_bundle_sha256: None, modern: true, immutable_target: None,
        };
        (metadata.session_id().to_owned(), metadata.map_name().to_owned(), checkpoint, timestamp, None,
         Some(owner.clone()), Some(ActivationTargetSource::Assigned(owner)), None, None, None,
         fine, dense, seed, key)
    } else {
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

    let prepared_fine = if let Some(backend) = mapping_backend_checkpoint.as_ref().filter(|b| b.fine_map_artifact.is_some()) {
        let file = service.graph_artifacts.lock().open(backend.fine_map_artifact.as_ref().unwrap()).map_err(Status::invalid_argument)?;
        let source_session = backend.source_session_id.clone();
        let source_generation = backend.source_session_generation;
        let receipt = backend.cloud_receipt.as_ref().ok_or_else(|| Status::invalid_argument("精细资产缺少原点云收据"))?.clone();
        let receiver = service.bridge.register_navigation_map_operation(async move {
            tokio::task::spawn_blocking(move || crate::pointcloud::prepare_fine_cloud_archive(file, &source_session,
                source_generation, receipt.cloud_revision, receipt.source_timestamp_ns))
                .await.map_err(|e| e.to_string()).and_then(|v| v)
        }).map_err(Status::unavailable)?;
        Some(receiver.await.map_err(|e| Status::unavailable(e.to_string()))?.map_err(Status::invalid_argument)?)
    } else { None };

    let prepared_dense = if let Some(backend) = mapping_backend_checkpoint.as_ref().filter(|b| b.dense_cloud_artifact.is_some()) {
        let dense_voxel_size = prepared_fine.as_ref().map(|fine| fine.voxel_size()).or_else(|| immutable_target.as_ref().map(|target| target.voxel_size))
            .ok_or_else(|| Status::invalid_argument("完整点云缺少匹配的体素地图"))?;
        let file = service.graph_artifacts.lock().open(backend.dense_cloud_artifact.as_ref().unwrap()).map_err(Status::invalid_argument)?;
        let receipt = backend.cloud_receipt.as_ref().ok_or_else(|| Status::invalid_argument("完整点云缺少原提交收据"))?;
        let expected = crate::dense_cloud::Metadata {
            version: 1, session_id: backend.source_session_id.clone(), session_generation: backend.source_session_generation,
            cloud_revision: receipt.cloud_revision, source_timestamp_ns: receipt.source_timestamp_ns,
            frame_id: expected_map_frame_id.clone(), point_count: 0, bounds_min: [0.; 3], bounds_max: [0.; 3],
        };
        let receiver = service.bridge.register_navigation_map_operation(async move {
            tokio::task::spawn_blocking(move || crate::dense_cloud::DenseCloud::from_archive_with_voxel_size(file, &expected, dense_voxel_size))
                .await.map_err(|e| e.to_string()).and_then(|v| v)
        }).map_err(Status::unavailable)?;
        Some(receiver.await.map_err(|e| Status::unavailable(e.to_string()))?.map_err(Status::invalid_argument)?)
    } else { None };

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
        planar_target: None,
        activation_request_id: identity_class.request_id().to_owned(),
        request_key: activation_request_identity_sha256.into(),
        checkpoint_owner: None,
        asset_manifest_sha256: identity_class.asset_manifest_sha256(),
        asset_bundle_sha256: identity_class.asset_bundle_sha256(),
        modern: identity_class.is_modern(),
        immutable_target: immutable_target.clone(),
    };

    let target = immutable_target.map(|value| ActivationTargetSource::Legacy(Arc::new(value)));
    (session_id, map_name, checkpoint, checkpoint_timestamp_ns, mapping_backend_checkpoint, None,
     target, Some(occupancy_sha256), cloud_sha256, backend_sha256, prepared_fine, prepared_dense,
     seed, ActivationRequestKey::LegacyDigest(activation_request_identity_sha256))
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
    let (
        mut retained_global_map,
        mut projection_guard,
        mut lifecycle_guard,
        restore_fence_on_early_return,
    ) = match service.take_matching_activation_lifecycle_fence(
        identity_class.request_id(),
        activation_request_key.clone(),
    ) {
        Ok(Some(fence)) => (
            fence.retained_global_map,
            Some(fence.projection_guard),
            Some(fence.lifecycle_guard),
            true,
        ),
        Ok(None) => {
            let projection = bridge.lock_explore_projection_owned().await;
            let lifecycle = bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            (None, Some(projection), Some(lifecycle), false)
        }
        Err(fence_error_code) => {
            let current = adapter.lock().snapshot();
            let terminal = match service.activation_receipts.stage_or_replay(
                seed.clone(),
                mapping_session_state(current, fence_error_code),
                false,
            ) {
                crate::correction_coordinator::ActivationLedgerAdmission::Terminal(state) => *state,
                crate::correction_coordinator::ActivationLedgerAdmission::Execute(_) => {
                    MappingSessionState {
                        success: false,
                        message: fence_error_code.to_owned(),
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
                activation_request_key: activation_request_key.clone(),
                retained_global_map,
                projection_guard: projection_guard
                    .take()
                    .expect("ActivateMap early return owns projection guard"),
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
                activation_request_key: activation_request_key.clone(),
                retained_global_map,
                projection_guard: projection_guard
                    .take()
                    .expect("ActivateMap early return owns projection guard"),
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
                    activation_request_key: activation_request_key.clone(),
                    retained_global_map,
                    projection_guard: projection_guard
                        .take()
                        .expect("ActivateMap early return owns projection guard"),
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
        let mut projection_guard = projection_guard
            .take()
            .expect("active ActivateMap owns projection guard");
        let mut lifecycle_guard = lifecycle_guard
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
        if current.session_id != session_id
            || current.map_name != map_name
            || !adapter
                .lock()
                .checkpoint_activation_matches(&activation_request_key)
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
        let target = match service.activation_receipts.target_source(&token) {
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
            let cloud_metadata = bridge.cloud_map_writer.lock().as_ref()
                .and_then(crate::pointcloud::CloudMapShmWriter::committed_metadata);
            let Some(cloud_metadata) = cloud_metadata else {
                settle_current_failure!(
                    ActivationReceiptDisposition::OutcomeUnknown,
                    "ACTIVATE_MAP_STAGED_CLOUD_METADATA_UNAVAILABLE".to_owned()
                );
            };
            let committed = CommittedMappingCloudMap {
                notification: CloudMapNotification {
                    revision: proof.cloud_revision,
                    point_count: proof.point_count,
                    voxel_size: proof.voxel_size,
                    session_generation: current.session_generation,
                },
                source_timestamp_ns: proof.source_timestamp_ns,
                metadata: cloud_metadata,
                receipt: proof.cloud_receipt.clone(),
            };
            let replaying_retained_publication = retained_global_map.is_some();
            let mut retained_publication = match retained_global_map.take() {
                Some(retained) => {
                    if let Err(error) = retained.validate_replay(
                        activation_request_key.clone(),
                        &expected_map_frame_id,
                        &committed,
                    ) {
                        let detail = format!(
                            "cloud-map checkpoint retained publication identity failed: {error}"
                        );
                        match service.retain_activation_lifecycle_fence(
                            identity_class.request_id(),
                            activation_request_key.clone(),
                            Some(retained),
                            projection_guard,
                            lifecycle_guard,
                        ) {
                            Ok(()) => return Err(Status::internal(detail)),
                            Err((returned_projection, returned_lifecycle)) => {
                                projection_guard = returned_projection;
                                lifecycle_guard = returned_lifecycle;
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
                    activation_request_key.clone(),
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
            let publish_result = match (replaying_retained_publication, retained_publication.legacy_frame.as_ref()) {
                (true, Some(frame)) => publish_retained_global_map_frame(&bridge, frame).await,
                _ => publish_committed_global_map_in_frame(&bridge, &committed, &expected_map_frame_id).await,
            };
            if let Err(error) = publish_result {
                let mut detail = format!(
                    "cloud-map checkpoint installed but typed /global_map publication failed: {error}"
                );
                if let Err(capture_error) = retained_publication.capture_materialized_legacy_frame(
                    &bridge.cloud_map_writer, &committed,
                ) {
                    detail.push_str(&format!("; canonical retention failed: {capture_error}"));
                }
                match service.retain_activation_lifecycle_fence(
                    identity_class.request_id(),
                    activation_request_key.clone(),
                    Some(retained_publication),
                    projection_guard,
                    lifecycle_guard,
                ) {
                    Ok(()) => return Err(Status::internal(detail)),
                    Err((returned_projection, returned_lifecycle)) => {
                        projection_guard = returned_projection;
                        lifecycle_guard = returned_lifecycle;
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
                activation_request_key.clone(),
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                Some(match &activation_request_key {
                    ActivationRequestKey::LegacyDigest(_) => crate::map_checksum::sha256_bytes(target.xyz_f32_le()).into(),
                    ActivationRequestKey::Assigned(_) => ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::Cloud(proof.cloud_receipt.clone())),
                }),
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
        // Seal against the actual publication proof even for an unbound
        // compatibility request. Public eligibility is projected by the ledger.
        let target_receipt = Some(proof.target_receipt.clone());
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
                    activation_request_key: activation_request_key.clone(),
                    retained_global_map,
                    projection_guard: projection_guard
                        .take()
                        .expect("ActivateMap early return owns projection guard"),
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

    let backend_verifier = match &activation_request_key {
        ActivationRequestKey::LegacyDigest(_) => backend_sha256.map(ActivationJournalVerifier::LegacyDigest),
        ActivationRequestKey::Assigned(_) => Some(ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::BackendGeneration(reserved_backend_generation))),
    };
    let mut restore_resolution = if mapping_backend_checkpoint.is_some() || assigned_checkpoint.is_some() {
        if !record_activation_journal(
            &service.activation_receipts,
            &token,
            activation_request_key.clone(),
            crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
            crate::correction_coordinator::ActivationJournalStatus::Prepared,
            backend_verifier.clone(),
            None,
        ) {
            let _ = staged_worker.stop_and_join();
            return_precommit_failure_with_fence!(
                ActivationReceiptDisposition::InvalidatedPrecommit,
                "ACTIVATE_MAP_RESTORE_PREPARE_CAS_FAILED"
            );
        }
        let restore = if let Some(owner) = assigned_checkpoint.as_ref() {
            staged_worker.client().restore_owned_checkpoint(owner.clone())
        } else {
            staged_worker.client().restore_checkpoint(mapping_backend_checkpoint.as_ref().expect("legacy restore owns backend checkpoint").payload.clone())
        };
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
            activation_request_key.clone(),
            crate::correction_coordinator::ActivationJournalStep::BackendRestoreSubmission,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            backend_verifier.clone(),
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
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::SessionStart,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_SESSION_START".to_owned()
        );
    }
    let final_activation_identity = if let ActivationRequestIdentityClass::Assigned(metadata) = &identity_class {
        ActivationIdentity::Assigned(InstalledActivationIdentityV2 {
            request: metadata.clone(), runtime_generation: service.activation_receipts.runtime_generation(),
            lifecycle_epoch: service.activation_receipts.lifecycle_epoch(), session_generation: started.session_generation,
            backend_kind: selected_backend_kind, backend_generation: reserved_backend_generation,
        })
    } else if let ActivationRequestIdentityClass::Modern {
        asset_manifest_sha256,
        asset_bundle_sha256,
        ..
    } = &identity_class
    {
        ActivationIdentity::LegacyDigest(crate::map_checksum::sha256_bytes(
            &canonical_activation_identity_preimage(
                activation_request_key.legacy_digest().expect("legacy activation preimage"),
                service.activation_receipts.runtime_generation(),
                started.session_generation,
                reserved_backend_generation,
                selected_backend_kind.stable_name(),
                backend_sha256,
                *asset_manifest_sha256,
                *asset_bundle_sha256,
            )
            .map_err(Status::internal)?,
        ))
    } else {
        let mut bytes = b"nuwax.geometry.activate-map.legacy-identity.v1\0".to_vec();
        bytes.extend_from_slice(&activation_request_key.legacy_digest().expect("legacy activation identity"));
        bytes.extend_from_slice(&started.session_generation.to_be_bytes());
        bytes.extend_from_slice(&reserved_backend_generation.to_be_bytes());
        ActivationIdentity::LegacyDigest(crate::map_checksum::sha256_bytes(&bytes))
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
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::SessionStart,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(activation_verifier(&final_activation_identity)),
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
            activation_request_key.clone(),
            crate::correction_coordinator::ActivationJournalStep::OccupancyShmBegin,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            Some(match &activation_request_key {
                ActivationRequestKey::LegacyDigest(_) => crate::map_checksum::sha256_bytes(&started.session_generation.to_be_bytes()).into(),
                ActivationRequestKey::Assigned(_) => ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::SessionGeneration(started.session_generation)),
            }),
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
    if take_activation_fault_after(
        &service,
        crate::correction_coordinator::ActivationJournalStep::OccupancyInstall,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "INJECTED_AFTER_OCCUPANCY_INSTALL".to_owned()
        );
    }
    let occupancy_verifier = match &activation_request_key {
        ActivationRequestKey::LegacyDigest(_) => ActivationJournalVerifier::LegacyDigest(occupancy_sha256.expect("legacy occupancy digest")),
        ActivationRequestKey::Assigned(_) => ActivationJournalVerifier::Assigned(ActivationJournalVerifierV2::Occupancy(occupancy_receipt.clone())),
    };
    if !record_activation_journal(
        &service.activation_receipts,
        &token,
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::OccupancyInstall,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_verifier.clone()),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPrecommit,
            "ACTIVATE_MAP_OCCUPANCY_INSTALL_JOURNAL_CAS_FAILED".to_owned()
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
        let target_receipt = match &final_activation_identity {
            ActivationIdentity::LegacyDigest(identity) => {
                let preimage = match canonical_target_receipt_preimage(*identity,
                    cloud_sha256.expect("legacy cloud target digest exists"), target.source_revision(),
                    correlated.snapshot.revision, started.session_generation, reserved_backend_generation, &correlated.receipt) {
                    Ok(preimage) => preimage,
                    Err(error) => { settle_started_failure!(ActivationReceiptDisposition::OutcomeUnknown,
                        format!("ACTIVATE_MAP_TARGET_RECEIPT_OUTCOME_UNKNOWN: {error}")); }
                };
                TargetReceipt::LegacyDigest(crate::map_checksum::sha256_bytes(&preimage))
            }
            ActivationIdentity::Assigned(identity) => {
                let owner = assigned_checkpoint.as_ref().expect("assigned activation owns source");
                let receipt = InstalledTargetReceiptV2 { activation: identity.clone(),
                    source_cloud_member: owner.cloud().member().clone(), occupancy_receipt: occupancy_receipt.clone(),
                    cloud_receipt: correlated.receipt.clone() };
                if !receipt.validate(owner) {
                    settle_started_failure!(ActivationReceiptDisposition::OutcomeUnknown,
                        "ACTIVATE_MAP_TYPED_TARGET_RECEIPT_MISMATCH".to_owned());
                }
                TargetReceipt::Assigned(receipt)
            }
        };
        let proof = crate::correction_coordinator::ActivationPublicationProof {
            source_timestamp_ns,
            cloud_revision: correlated.snapshot.revision,
            point_count: target.point_count(),
            voxel_size: target.voxel_size(),
            cloud_receipt: correlated.receipt.clone(),
            target_receipt: target_receipt.clone(),
        };
        if !service
            .activation_receipts
            .store_publication_proof(&token, proof)
            || !record_activation_journal(
                &service.activation_receipts,
                &token,
                activation_request_key.clone(),
                crate::correction_coordinator::ActivationJournalStep::CloudInstall,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_activation_verifier(&activation_request_key, cloud_sha256, &correlated.receipt),
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
            activation_request_key.clone(),
            crate::correction_coordinator::ActivationJournalStep::BackendCommit,
            crate::correction_coordinator::ActivationJournalStatus::Applied,
            backend_verifier.clone(),
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
        adapter.set_legacy_checkpoint_reconstructed(mapping_backend_checkpoint.is_none() && assigned_checkpoint.is_none());
        adapter.record_checkpoint_activation(activation_request_key.clone())
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
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::ActivationRecord,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(activation_verifier(&final_activation_identity)),
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
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::SharedStateCommit,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_verifier.clone()),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_SHARED_STATE_JOURNAL_CAS_FAILED".to_owned()
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
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::CacheClears,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_verifier.clone()),
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
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::MapEvent,
        crate::correction_coordinator::ActivationJournalStatus::Applied,
        Some(occupancy_verifier.clone()),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_MAP_EVENT_JOURNAL_CAS_FAILED".to_owned()
        );
    }

    let target_receipt =
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
                activation_request_key.clone(),
                crate::correction_coordinator::ActivationJournalStep::CloudEvent,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_activation_verifier(&activation_request_key, cloud_sha256, &committed.receipt),
                None,
            ) {
                settle_started_failure!(
                    ActivationReceiptDisposition::InvalidatedPostcommit,
                    "ACTIVATE_MAP_CLOUD_EVENT_JOURNAL_CAS_FAILED".to_owned()
                );
            }
            let mut retained_publication = match RetainedActivationGlobalMap::from_committed(
                activation_request_key.clone(),
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
                let mut detail = format!(
                    "cloud-map checkpoint installed but typed /global_map publication failed: {error}"
                );
                if let Err(capture_error) = retained_publication.capture_materialized_legacy_frame(
                    &bridge.cloud_map_writer, &committed,
                ) {
                    detail.push_str(&format!("; canonical retention failed: {capture_error}"));
                }
                match service.retain_activation_lifecycle_fence(
                    identity_class.request_id(),
                    activation_request_key.clone(),
                    Some(retained_publication),
                    projection_guard
                        .take()
                        .expect("ActivateMap publication retention owns projection guard"),
                    lifecycle_guard
                        .take()
                        .expect("ActivateMap publication retention owns lifecycle guard"),
                ) {
                    Ok(()) => return Err(Status::internal(detail)),
                    Err((returned_projection, returned_lifecycle)) => {
                        projection_guard = Some(returned_projection);
                        lifecycle_guard = Some(returned_lifecycle);
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
                activation_request_key.clone(),
                crate::correction_coordinator::ActivationJournalStep::TypedGlobalMap,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                cloud_activation_verifier(&activation_request_key, cloud_sha256, &committed.receipt),
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
        activation_request_key.clone(),
        crate::correction_coordinator::ActivationJournalStep::Seal,
        crate::correction_coordinator::ActivationJournalStatus::Prepared,
        Some(activation_verifier(&final_activation_identity)),
        None,
    ) {
        settle_started_failure!(
            ActivationReceiptDisposition::InvalidatedPostcommit,
            "ACTIVATE_MAP_SEAL_PREPARE_CAS_FAILED".to_owned()
        );
    }
    let snapshot = adapter.lock().snapshot();
    let activation_message = if mapping_backend_checkpoint.is_some() || assigned_checkpoint.is_some() {
        "map checkpoint activated; legacy_checkpoint_reconstructed=false"
    } else {
        "map checkpoint activated; legacy_checkpoint_reconstructed=true"
    };
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
    let meta = checkpoint.meta.ok_or_else(|| "checkpoint.meta is required".to_string())?;
    decode_checkpoint_grid_parts(&meta, checkpoint.data.len(), checkpoint.data.into_iter(), expected_map_frame_id)
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
        xyz_sha256: String::new(),
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

fn validate_cloud_checkpoint_metadata(
    checkpoint: &CloudMapCheckpoint,
    expected_map_frame_id: &str,
) -> Result<(), String> {
    if checkpoint.member_identity.is_some() {
        return Err("assigned cloud member cannot enter legacy checkpoint validation".into());
    }
    let frame_id = checkpoint.frame_id.trim();
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

fn validate_planar_checkpoint(checkpoint:&MappingBackendCheckpoint)->Result<(),String> {
    if checkpoint.dense_cloud_artifact.is_some() { return Err("二维检查点不能携带完整三维点云".into()); }
    if checkpoint.fine_map_artifact.is_some() { return Err("二维检查点不能携带三维精细资产".into()); }
    let reference=checkpoint.graph_artifact.as_ref().ok_or("二维图必须通过分块制品引用加载")?;
    if checkpoint.schema_version!=1 || checkpoint.backend_kind!="slam_toolbox" || checkpoint.target_kind!="planar_graph"
        || checkpoint.created_at_ns==0 || checkpoint.source_session_id.is_empty() || checkpoint.source_session_generation==0
        || !checkpoint.payload.is_empty() || checkpoint.cloud_receipt.is_some() || checkpoint.canonical_cloud_checkpoint.is_some()
        || reference.size!=checkpoint.payload_size || reference.sha256!=checkpoint.content_sha256
        || reference.size==0 || reference.size>nuwax_slam_2d_runtime::archive::MAX_ARCHIVE || reference.sha256.len()!=32 {
        return Err("二维完整图检查点类型、身份或预算非法".into());
    }
    let receipt=checkpoint.occupancy_receipt.as_ref().ok_or("二维图缺少规范占据收据")?;
    validate_mapping_commit_identity(receipt.commit_id.as_ref(),"二维图")?;
    let grid=checkpoint.canonical_occupancy_checkpoint.as_ref().ok_or("二维图缺少物理栅格")?;
    canonical_occupancy_sha256(grid)?;
    let meta=grid.meta.as_ref().ok_or("二维栅格缺少几何")?;
    if receipt.session_id!=checkpoint.source_session_id || receipt.session_generation!=checkpoint.source_session_generation
        || receipt.backend_kind!="slam_toolbox" || receipt.backend_generation==0 || receipt.occupancy_revision!=meta.revision
        || receipt.source_timestamp_ns!=meta.timestamp_ns || meta.timestamp_ns==0 || meta.revision==0 {
        return Err("二维图物理栅格与规范收据身份冲突".into());
    }
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
fn checkpoint_envelope_version(requested: u32) -> Result<u32, String> {
    match requested { 0 | 1 => Ok(1), 2 => Ok(2), _ => Err(format!("unsupported checkpoint envelope schema: {requested}")) }
}
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
    validate_mapping_checkpoint_bundle(checkpoint, occupancy, Some(cloud))
}
fn validate_checkpoint_member_binding(checkpoint: &MappingBackendCheckpoint, member: &CheckpointMemberIdentity, role: CheckpointMemberRole) -> Result<(), String> {
    crate::planar_artifacts::validate_member_identity(member)?;
    let frame = &checkpoint.canonical_cloud_checkpoint.as_ref().ok_or("canonical cloud required")?.frame_id;
    if member.checkpoint_id != checkpoint.checkpoint_id || member.role != role as i32
        || member.occupancy_receipt != checkpoint.occupancy_receipt || member.cloud_receipt != checkpoint.cloud_receipt || member.frame_id != *frame {
        return Err("checkpoint member descriptor differs from its frozen parent".into());
    }
    Ok(())
}
pub(crate) fn validate_assigned_mapping_checkpoint(checkpoint: &MappingBackendCheckpoint, expected_backend: crate::mapping_backend::MappingBackendKind) -> Result<(), String> {
    if checkpoint.schema_version != ASSIGNED_CHECKPOINT_ENVELOPE_VERSION
        || !crate::planar_artifacts::assigned_id(&checkpoint.checkpoint_id) || !checkpoint.content_sha256.is_empty()
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
    if cloud.payload_schema_version != 2 || !cloud.xyz_sha256.is_empty() { return Err("mixed canonical cloud identity generation".into()); }
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
fn assemble_mapping_checkpoint_export(mut checkpoint: MappingBackendCheckpoint, envelope_version: u32,
    expected_backend: crate::mapping_backend::MappingBackendKind) -> Result<MappingBackendCheckpoint, String> {
    checkpoint.schema_version = checkpoint_envelope_version(envelope_version)?;
    if checkpoint.schema_version == 2 {
        checkpoint.backend_payload_schema_version = crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION;
        if !checkpoint.content_sha256.is_empty() { return Err("v2 assembly received an active legacy digest".into()); }
        let occupancy = checkpoint.occupancy_receipt.as_ref().ok_or("occupancy receipt required")?;
        let cloud_receipt = checkpoint.cloud_receipt.as_ref().ok_or("cloud receipt required")?;
        let cloud = checkpoint.canonical_cloud_checkpoint.as_mut().ok_or("canonical cloud required")?;
        if !cloud.xyz_sha256.is_empty() || cloud.member_identity.is_some() { return Err("cloud assembly requires unfingerprinted canonical bytes".into()); }
        if checkpoint.backend_identity.is_some() || checkpoint.occupancy_identity.is_some() { return Err("assembly requires unassigned backend/occupancy members".into()); }
        checkpoint.backend_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::BackendState, occupancy, cloud_receipt, &cloud.frame_id, None));
        checkpoint.occupancy_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::CanonicalOccupancy, occupancy, cloud_receipt, &cloud.frame_id, None));
        cloud.payload_schema_version = 2;
        cloud.member_identity = Some(checkpoint_member_identity(&checkpoint.checkpoint_id, CheckpointMemberRole::CanonicalCloud, occupancy, cloud_receipt, &cloud.frame_id, None));
        validate_assigned_mapping_checkpoint(&checkpoint, expected_backend)?;
    } else {
        if !checkpoint.checkpoint_id.is_empty() || checkpoint.backend_payload_schema_version != 0 || checkpoint.backend_identity.is_some() || checkpoint.occupancy_identity.is_some() { return Err("legacy assembly received assigned identity".into()); }
        checkpoint.content_sha256 = crate::map_checksum::sha256_bytes(&checkpoint.payload).to_vec();
        let cloud = checkpoint.canonical_cloud_checkpoint.as_mut().ok_or("canonical cloud required")?;
        cloud.payload_schema_version = crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION;
        cloud.xyz_sha256 = crate::map_checksum::sha256_hex(&cloud.xyz_f32_le);
        validate_cloud_checkpoint_metadata(cloud, &cloud.frame_id)?;
        validate_mapping_backend_checkpoint(&checkpoint, expected_backend)?;
    }
    validate_mapping_checkpoint_envelope_size("mapping backend checkpoint response", checkpoint.encoded_len())?;
    Ok(checkpoint)
}

fn validate_mapping_backend_checkpoint(
    checkpoint: &MappingBackendCheckpoint,
    expected_backend: crate::mapping_backend::MappingBackendKind,
) -> Result<(), String> {
    if !checkpoint.checkpoint_id.is_empty() || checkpoint.backend_payload_schema_version != 0 || checkpoint.backend_identity.is_some() || checkpoint.occupancy_identity.is_some() {
        return Err("assigned checkpoint requires its paired v2 consumer".into());
    }
    if let Some(dense) = &checkpoint.dense_cloud_artifact {
        crate::planar_artifacts::Store::validate_reference(dense)?;
        if dense.identity_schema_version == 2 { return Err("v2 artifact cannot enter legacy activation".into()); }
        if dense.id.is_empty() || dense.id.len() > 128 || dense.size == 0 || dense.size > crate::dense_cloud::MAX_ARCHIVE_BYTES
            || dense.sha256.len() != 32 || dense.sha256.iter().all(|b| *b == 0) { return Err("完整点云制品引用无效".into()); }
    }
    if let Some(fine) = &checkpoint.fine_map_artifact {
        crate::planar_artifacts::Store::validate_reference(fine)?;
        if fine.identity_schema_version == 2 { return Err("v2 artifact cannot enter legacy activation".into()); }
        if fine.id.is_empty() || fine.id.len() > 128 || fine.size == 0 || fine.size > nuwax_map_chunks::MAX_ARCHIVE_BYTES
            || fine.sha256.len() != 32 || fine.sha256.iter().all(|b| *b == 0) { return Err("精细地图制品引用无效".into()); }
    }
    if expected_backend==crate::mapping_backend::MappingBackendKind::SlamToolbox {return validate_planar_checkpoint(checkpoint);}
    if checkpoint.schema_version
        != crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION
    {
        return Err(format!(
            "unsupported mapping backend checkpoint schema: expected={} actual={}",
            crate::mapping_backend::MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
            checkpoint.schema_version
        ));
    }
    validate_mapping_checkpoint_structure(checkpoint, expected_backend)?;
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
            request.motion_guard_generation,
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
        let retained_fence = self.take_activation_lifecycle_fence()?;
        let (projection_guard, lifecycle_guard, retained_metadata) = match retained_fence {
            Some(fence) => (
                fence.projection_guard,
                fence.lifecycle_guard,
                Some((
                    fence.activation_request_id,
                    fence.activation_request_key,
                    fence.retained_global_map,
                )),
            ),
            None => match tokio::time::timeout(STOP_LIFECYCLE_TIMEOUT, async {
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
                Ok((projection, lifecycle)) => (projection, lifecycle, None),
                Err(_) => {
                    let error = Status::deadline_exceeded(
                        "timed out acquiring mapping lifecycle fence during process Stop",
                    );
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
            },
        };
        drop(lifecycle_guard);
        macro_rules! fail_before_motion_terminal {
            ($error:expr) => {{
                let error = self
                    .settle_process_stop_failure(retained_metadata, projection_guard, $error)
                    .await;
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

        self.consume_retained_activation_receipt_after_motion_terminal(retained_metadata);
        #[cfg(test)]
        {
            let barrier = self.stop_after_retained_receipt_consume.lock().take();
            if let Some(barrier) = barrier {
                barrier.wait().await;
                barrier.wait().await;
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
                    let metadata=archive::verify(&mut file)?;
                    let (_,cells)=archive::grid(&mut file)?;
                    Ok::<_,String>((file,metadata,cells))
                }).await.map_err(|e|e.to_string())??;
                let mut snapshot=nuwax_slam_2d_runtime::protocol::Snapshot {
                    snapshot_id:1,session:metadata.session.clone(),estimate:metadata.estimate.clone(),width:metadata.width,height:metadata.height,
                    resolution:metadata.resolution,origin:metadata.origin,payload_size:metadata.members[2].size,
                    data_sha256:metadata.members[2].sha256,file_name:String::new(),
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
                let store=self.graph_artifacts.clone();let size=artifact.payload_size;let sha=artifact.sha256;
                let reference=tokio::task::spawn_blocking(move||store.lock().insert_legacy(file,size,sha)).await.map_err(|e|e.to_string())??;
                Ok(MappingBackendCheckpoint{schema_version:1,backend_kind:"slam_toolbox".into(),source_session_generation:session.session_generation,
                    source_session_id:session.session_id.clone(),content_sha256:artifact.sha256.to_vec(),payload_size:artifact.payload_size,
                    created_at_ns:now_ns(),occupancy_receipt:Some(occupancy_commit_receipt_proto(&receipt)),canonical_occupancy_checkpoint:Some(canonical),
                    graph_artifact:Some(reference),target_kind:"planar_graph".into(),..Default::default()})
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
                .map(|b|b.client.identity().runtime_generation).unwrap_or(0),
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

        let retained_fence = self.take_activation_lifecycle_fence()?;
        let fresh_guards = if retained_fence.is_none() {
            let projection = self.bridge.lock_explore_projection_owned().await;
            let lifecycle = self
                .bridge
                .mapping_lifecycle_lock
                .clone()
                .lock_owned()
                .await;
            Some((projection, lifecycle))
        } else {
            None
        };

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
                if let Some(fence) = retained_fence {
                    self.restore_activation_lifecycle_fence(fence);
                }
                return Err(Status::unavailable(format!(
                    "StartMapping cloud-map clear registration failed: {detail}"
                )));
            }
        };

        if let Err(error) = self.ensure_running_admission("StartMapping").await {
            if let Some(fence) = retained_fence {
                self.restore_activation_lifecycle_fence(fence);
            }
            return Err(error);
        }

        if let Err(error) = self.activation_receipts.retire_for_mapping_change() {
            if let Some(fence) = retained_fence {
                self.restore_activation_lifecycle_fence(fence);
            }
            return Err(Status::resource_exhausted(error));
        }

        let snapshot = match adapter
            .lock()
            .start_session(&request.session_id, &request.map_name)
            .map_err(mapping_lifecycle_status)
        {
            Ok(snapshot) => snapshot,
            Err(error) => {
                if let Some(fence) = retained_fence {
                    self.restore_activation_lifecycle_fence(fence);
                }
                return Err(error);
            }
        };
        let (projection_guard, lifecycle_guard) = match retained_fence {
            Some(fence) => (fence.projection_guard, fence.lifecycle_guard),
            None => fresh_guards.expect("fresh StartMapping guards must be present"),
        };
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
                    projection_guard,
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
        let mut staged_fence = self.take_activation_lifecycle_fence()?;
        let (mut acquired_projection_guard, acquired_lifecycle_guard) = if staged_fence.is_none() {
            let projection = self.bridge.lock_explore_projection_owned().await;
            let lifecycle = self
                .bridge
                .mapping_lifecycle_lock
                .clone()
                .lock_owned()
                .await;
            (Some(projection), Some(lifecycle))
        } else {
            (None, None)
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
        let projection_guard = staged_fence
            .as_mut()
            .map(|fence| &mut fence.projection_guard)
            .or(acquired_projection_guard.as_mut())
            .expect("StopMapping owns one projection guard before mutation");
        self.bridge
            .revoke_committed_map_projection_locked(projection_guard)
            .map_err(Status::unavailable)?;
        let (projection_guard, lifecycle_guard) = match staged_fence.take() {
            Some(fence) => (fence.projection_guard, fence.lifecycle_guard),
            None => (
                acquired_projection_guard
                    .take()
                    .expect("StopMapping owns acquired projection guard"),
                acquired_lifecycle_guard.expect("StopMapping owns acquired lifecycle guard"),
            ),
        };
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
        req: Request<ExportMappingBackendCheckpointRequest>,
    ) -> Result<Response<MappingBackendCheckpoint>, Status> {
        let envelope_version = checkpoint_envelope_version(req.into_inner().checkpoint_schema_version).map_err(Status::invalid_argument)?;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        if adapter.lock().planar.is_some() {
            if envelope_version == 2 { return Err(Status::failed_precondition("planar checkpoint v2 requires paired inner archive/protocol migration")); }
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
                let checkpoint_id = if envelope_version == 2 { uuid::Uuid::new_v4().simple().to_string() } else { String::new() };
                if envelope_version == 1 && fine_snapshot.is_some() {
                    return Err(Status::failed_precondition("staged AR2 fine export requires envelope v2; default finalized-map/activation remains v1 until paired consumers land"));
                }
                // 已拿到同一屏障内的不可变快照，归档不持生命周期锁。
                drop(lifecycle_guard);
                let dense_cloud_artifact = if let Some(dense) = dense_snapshot {
                    let store = owned_service.graph_artifacts.clone();
                    let frame_id = raw_map.frame_id.clone();
                    let member = checkpoint_member_identity(&checkpoint_id, CheckpointMemberRole::DenseCloud,
                        &occupancy_receipt, &cloud_receipt, &frame_id, None);
                    Some(tokio::task::spawn_blocking(move || {
                        let (mut file, size) = dense.archive(&frame_id)?;
                        if envelope_version == 2 { store.lock().insert(file, size, member) }
                        else {
                            // Explicit legacy DC1 compatibility only; never applied to AR2.
                            use std::io::{Seek, SeekFrom};
                            file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
                            let (_, sha) = crate::dense_cloud::hash_reader(&mut file)?;
                            store.lock().insert_legacy(file, size, sha)
                        }
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
                    content_sha256: Vec::new(),
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
                return assemble_mapping_checkpoint_export(checkpoint, envelope_version, backend_kind).map_err(Status::internal);
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
        if req.get_ref().target_kind=="planar_graph" {
            let lifecycle_guard = self.bridge.mapping_lifecycle_lock.clone().lock_owned().await;
            let request=req.into_inner();let task_request=request.clone();let service=self.clone();
            let work:crate::correction_coordinator::PlanarCommandWork=Box::new(move|context|Box::pin(async move {
                let bridge=service.bridge.clone();
                let receiver=bridge.register_planar_operation(async move {Ok(service.run_planar_relocalization(task_request,context).await)});
                match receiver {
                    Ok(receiver)=>match receiver.await {
                        Ok(Ok(outcome))=>outcome,
                        _=>crate::correction_coordinator::PlanarCommandOutcome{disposition:RelocalizeDisposition::OwnerFailed,detail:"二维重定位保留任务未返回".into(),correction_revision:0,evidence:Vec::new()},
                    },
                    Err(e)=>crate::correction_coordinator::PlanarCommandOutcome{disposition:RelocalizeDisposition::OwnerStopped,detail:e,correction_revision:0,evidence:Vec::new()},
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
        let request = crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2::admit(req.into_inner())
            .map_err(Status::invalid_argument)?;
        let ingress = self.relocalization_ingress.as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let applier = self.canonical_correction.as_ref()
            .ok_or_else(|| Status::failed_precondition("ASSIGNED_SAMPLE_CANONICAL_OWNER_ABSENT"))?;
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

    async fn get_relocalization_terminal(
        &self,
        req: Request<GetRelocalizationTerminalRequest>,
    ) -> Result<Response<GetRelocalizationTerminalResponse>, Status> {
        let ingress = self
            .relocalization_ingress
            .as_ref()
            .ok_or_else(|| Status::failed_precondition("RELOCALIZATION_OWNER_NOT_SELECTED"))?;
        let request = req.into_inner();
        if request.schema_version == 2 {
            let query = crate::relocalization_terminal_v2::AssignedRelocalizationTerminalQueryV2::admit(request)
                .map_err(Status::invalid_argument)?;
            let snapshot = ingress.terminal_snapshot_assigned(&query).await
                .map_err(Status::failed_precondition)?;
            let (status, terminal) = match snapshot.terminal {
                crate::correction_coordinator::RelocalizationTerminalSnapshot::AssignedTerminal(terminal) =>
                    (RelocalizationTerminalStatus::Terminal, Some(terminal)),
                crate::correction_coordinator::RelocalizationTerminalSnapshot::Pending =>
                    (RelocalizationTerminalStatus::Pending, None),
                crate::correction_coordinator::RelocalizationTerminalSnapshot::NotFound =>
                    (RelocalizationTerminalStatus::NotFound, None),
                crate::correction_coordinator::RelocalizationTerminalSnapshot::IdentityConflict
                | crate::correction_coordinator::RelocalizationTerminalSnapshot::Terminal(_) =>
                    (RelocalizationTerminalStatus::IdentityConflict, None),
            };
            return query.response(status, terminal.as_ref(), Some(snapshot.ledger_observation))
                .map(Response::new).map_err(Status::internal);
        }
        if request.schema_version != 1
            || request.assigned_request_v2.is_some()
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
            crate::correction_coordinator::RelocalizationTerminalSnapshot::AssignedTerminal(_)
            | crate::correction_coordinator::RelocalizationTerminalSnapshot::IdentityConflict => {
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
            assigned_request_v2: None,
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
            ng.motion_guard_generation,
            &ng.motion_guard_boot_nonce,
            ng.motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let input = SetGoalInput {
            options: nuwax_nav_types_messages::GoalOptions {allow_reverse:ng.allow_reverse,position_tolerance_m:ng.position_tolerance_m.unwrap_or(0.35),yaw_tolerance_rad:ng.yaw_tolerance_rad.unwrap_or(0.3)},
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

    async fn get_navigation_map_block(
        &self, request: Request<NavigationMapBlockRequest>,
    ) -> Result<Response<NavigationMapBlockResponse>, Status> {
        self.ensure_running_admission("GetNavigationMapBlock").await?;
        let request = request.into_inner();
        if request.schema_version != 2 {
            return Err(Status::invalid_argument("unsupported fine-map block protocol version"));
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
        let receiver = self.bridge.read_navigation_map_block(&root, &reference).map_err(Status::failed_precondition)?;
        let data = receiver.await.map_err(|e| Status::unavailable(e.to_string()))?.map_err(Status::not_found)?;
        Ok(Response::new(NavigationMapBlockResponse {
            schema_version: 2, root: request.root, block: request.block, data,
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
        self.ensure_running_admission("GetMappingCloudAuthority")
            .await?;
        let sealed = self
            .activation_receipts
            .current_sealed_target_snapshot()
            .ok_or_else(|| {
                Status::failed_precondition("sealed finalized map authority is unavailable")
            })?;
        let committed = self
            .bridge
            .committed_map_authority
            .lock()
            .map_err(|_| Status::unavailable("committed map authority is unavailable"))?
            .clone()
            .ok_or_else(|| Status::failed_precondition("committed map authority is unavailable"))?;
        if sealed.session_id != committed.session_id
            || i64::try_from(sealed.session_generation).ok() != Some(committed.session_generation)
            || sealed.immutable_target.frame_id != committed.frame_id
        {
            return Err(Status::failed_precondition(
                "sealed finalized map does not match committed map authority",
            ));
        }
        let asset_manifest_sha256 = sealed
            .asset_manifest_sha256
            .ok_or_else(|| Status::failed_precondition("asset manifest identity is unavailable"))?;
        let asset_bundle_sha256 = sealed
            .asset_bundle_sha256
            .ok_or_else(|| Status::failed_precondition("asset bundle identity is unavailable"))?;
        let backend_kind = self
            .bridge
            .mapping_adapter
            .as_ref()
            .map(|adapter| format!("{:?}", adapter.lock().backend_kind()).to_ascii_lowercase())
            .unwrap_or_default();
        Ok(Response::new(MappingCloudAuthority {
            map_name: sealed.map_name,
            map_frame_id: committed.frame_id,
            asset_manifest_sha256: asset_manifest_sha256.to_vec(),
            asset_bundle_sha256: asset_bundle_sha256.to_vec(),
            activation_request_id: sealed.activation_request_id,
            activation_request_identity_sha256: sealed.activation_request_identity_sha256.to_vec(),
            activation_identity_sha256: sealed.activation_identity_sha256.to_vec(),
            target_receipt_sha256: sealed.target_receipt_sha256.to_vec(),
            runtime_generation: sealed.runtime_generation,
            lifecycle_epoch: sealed.lifecycle_epoch,
            mapping_session_id: sealed.session_id,
            mapping_session_generation: sealed.session_generation,
            backend_kind,
            backend_generation: sealed.backend_generation,
            committed_occupancy_revision: u64::try_from(committed.revision)
                .map_err(|_| Status::internal("committed revision is invalid"))?,
            schema_version: 2,
            committed_occupancy_commit_id: Some(MappingCommitIdentity {
                boot_nonce: committed.commit_boot_nonce.to_vec(),
                sequence: u64::try_from(committed.commit_sequence).map_err(|_| Status::internal("map commit sequence is invalid"))?,
            }),
            committed_occupancy_source_timestamp_ns: u64::try_from(committed.source_timestamp_ns)
                .map_err(|_| Status::internal("map source timestamp is invalid"))?,
            owner_epoch: self.bridge.nav.producer_runtime_generation(),
        }))
    }

    async fn submit_mapping_cloud_goal(
        &self,
        req: Request<MappingCloudGoalRequest>,
    ) -> Result<Response<MappingCloudGoalTerminal>, Status> {
        self.ensure_running_admission("SubmitMappingCloudGoal")
            .await?;
        let request = req.into_inner();
        if request.schema_version != 2 {
            return Err(Status::invalid_argument("unsupported MappingCloudGoalRequest schema_version"));
        }
        let request_commit = validate_mapping_commit_identity(request.committed_occupancy_commit_id.as_ref(), "mapping-cloud occupancy")
            .map_err(Status::invalid_argument)?;
        let digest = |field: &'static str, bytes: &[u8]| -> Result<Sha256Digest, Status> {
            let value: [u8; 32] = bytes
                .try_into()
                .map_err(|_| Status::invalid_argument(format!("{field} must contain 32 bytes")))?;
            Sha256Digest::from_bytes(value)
                .map_err(|error| Status::invalid_argument(error.to_string()))
        };
        let sealed = self
            .activation_receipts
            .current_sealed_target_snapshot()
            .ok_or_else(|| {
                Status::failed_precondition("sealed finalized map authority is unavailable")
            })?;
        let asset_manifest_sha256 = sealed
            .asset_manifest_sha256
            .ok_or_else(|| Status::failed_precondition("asset manifest identity is unavailable"))?;
        let asset_bundle_sha256 = sealed
            .asset_bundle_sha256
            .ok_or_else(|| Status::failed_precondition("asset bundle identity is unavailable"))?;
        if sealed.map_name != request.map_name
            || sealed.immutable_target.frame_id != request.map_frame_id
            || asset_manifest_sha256.as_slice() != request.asset_manifest_sha256
            || asset_bundle_sha256.as_slice() != request.asset_bundle_sha256
            || sealed.activation_identity_sha256.as_slice() != request.activation_identity_sha256
            || sealed.runtime_generation != request.runtime_generation
            || sealed.session_id != request.mapping_session_id
            || sealed.session_generation != request.mapping_session_generation
        {
            return Err(Status::failed_precondition(
                "mapping-cloud request does not match current sealed map authority",
            ));
        }
        let committed_map = self
            .bridge
            .committed_map_authority
            .lock()
            .map_err(|_| Status::unavailable("committed map authority is unavailable"))?
            .clone()
            .ok_or_else(|| Status::failed_precondition("committed map authority is unavailable"))?;
        if committed_map.session_id != request.mapping_session_id
            || u64::try_from(committed_map.session_generation).ok()
                != Some(request.mapping_session_generation)
            || u64::try_from(committed_map.revision).ok()
                != Some(request.committed_occupancy_revision)
            || committed_map.commit_boot_nonce.as_slice() != request_commit.boot_nonce
            || u64::try_from(committed_map.commit_sequence).ok() != Some(request_commit.sequence)
            || u64::try_from(committed_map.source_timestamp_ns).ok() != Some(request.committed_occupancy_source_timestamp_ns)
            || committed_map.frame_id != request.map_frame_id
        {
            return Err(Status::failed_precondition(
                "mapping-cloud request does not match current committed map authority",
            ));
        }
        let georeference = Arc::new(
            VerifiedGeoreferenceV1::from_json(
                &request.georeference_json,
                &GeoreferenceBindingV1::for_document(&request.georeference_json),
            )
            .map_err(|error| Status::invalid_argument(error.to_string()))?,
        );
        if georeference.contract().map_name != sealed.map_name
            || georeference.contract().map_frame_id != sealed.immutable_target.frame_id
        {
            return Err(Status::failed_precondition(
                "mapping-cloud georeference does not match current sealed map authority",
            ));
        }
        let owner_epoch = self.bridge.nav.producer_runtime_generation();
        if request.runtime_generation != owner_epoch {
            return Err(Status::failed_precondition(
                "mapping-cloud request runtime generation is stale",
            ));
        }
        let admission_context = crate::nav_runtime::NavAdmissionContext::with_expected_map_name(
            self.bridge.mapping_lifecycle_lock.clone(),
            self.bridge.committed_map_authority.clone(),
            self.bridge.map_revision_latest_tx.subscribe(),
            Some(sealed.map_name.clone()),
        ).with_mapping_adapter(self.bridge.mapping_adapter.clone());
        let active_map = ActiveMapIdentityV2 {
            map_name: request.map_name,
            map_frame_id: request.map_frame_id,
            asset_manifest_sha256: digest("asset_manifest_sha256", &request.asset_manifest_sha256)?,
            asset_bundle_sha256: digest("asset_bundle_sha256", &request.asset_bundle_sha256)?,
            georeference_sha256: georeference.raw_sha256(),
            activation_identity_sha256: digest(
                "activation_identity_sha256",
                &request.activation_identity_sha256,
            )?,
            runtime_generation: request.runtime_generation,
            mapping_session_generation: request.mapping_session_generation,
            committed_occupancy_revision: request.committed_occupancy_revision,
            committed_occupancy_boot_nonce: request_commit.boot_nonce.as_slice().try_into()
                .map_err(|_| Status::invalid_argument("map commit nonce must be 16 bytes"))?,
            committed_occupancy_commit_sequence: request_commit.sequence,
            committed_occupancy_source_timestamp_ns: request.committed_occupancy_source_timestamp_ns,
        };
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
                intent
                    .verify()
                    .map_err(|error| Status::internal(error.to_string()))?;
                let map_position = intent.map_position();
                let timestamp_ns = u64::try_from(intent.query().source_timestamp_ns)
                    .map_err(|_| Status::internal("mapping-cloud timestamp is invalid"))?;
                let goal = schema::PoseStamped {
                    header: schema::Header {
                        seq: 0,
                        stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000).map_err(|_| {
                            Status::internal("mapping-cloud timestamp exceeds wire range")
                        })?,
                        stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
                        frame_id: committed_map.frame_id.clone(),
                    },
                    pose: schema::Pose {
                        position: [map_position.x_m, map_position.y_m, map_position.z_m],
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                };
                let admission_io = crate::nav_runtime::NavAdmissionIo::new(
                    self.bridge.state.clone(),
                    self.bridge.transport.clone(),
                    self.bridge.transport_backend_name,
                    self.bridge.lcm_backend_available,
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
                        boundary_sha256: [0; 32],
                        boundary_points: Vec::new(),
                        motion_guard_generation: i64::try_from(request.motion_guard_generation)
                            .map_err(|_| {
                                Status::invalid_argument(
                                    "motion guard generation exceeds wire range",
                                )
                            })?,
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
                        goal_sha256: schema::goal_intent_goal_sha256(&goal)
                            .map_err(|error| Status::internal(error.to_string()))?,
                        goal,
                    },
                    crate::nav_runtime::GoalProducerEpochAuthority::MappingCloud,
                )
                .await;
                let update = NavigationUpdate::from_local(
                    outcome.state.clone(),
                    &self.bridge.nav,
                    outcome.lease,
                )
                .await;
                let navigation = nav_state_to_proto(update);
                let status = if matches!(
                    navigation.status.as_str(),
                    "ACCEPTED" | "PLANNING" | "RUNNING"
                ) {
                    "SUCCEEDED"
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
        if let Some(update)=self.bridge.nav.terminal_receipt_for(&input).await {
            return Ok(Response::new(nav_state_to_proto(update)));
        }
        let CancelOutcome { state: nav, lease } = nav_cancel(
            &self.bridge,
            &self.bridge.nav,
            input.clone(),
        )
        .await;
        if let Some(update)=self.bridge.nav.terminal_receipt_for(&input).await {
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
                ..Default::default()
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
                ..Default::default()
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
    use crate::lcm_runtime::{MapRevisionEvent, MapState};
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
        MapRevisionEvent::from_map_for_tests(&map, &session)
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
        assert_eq!(update.occupancy_receipt.as_ref(), Some(&occupancy_commit_receipt_proto(&event.receipt)));
        assert_eq!(update.raw_representation, event.raw_representation as i32);
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
    mod assigned_navigation_composition {
        include!(concat!(env!("CARGO_MANIFEST_DIR"), "/src/assigned_navigation_composition.rs"));
    }
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
                planar_target: None,
                activation_request_id: "relocalize-r1-gate".to_owned(),
                request_key: activation_request_identity_sha256.into(),
                checkpoint_owner: None,
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
        let projection_guard = handle.lock_explore_projection_owned().await;
        let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        assert!(service
            .retain_activation_lifecycle_fence(
                "relocalize-r1-gate",
                activation_request_identity_sha256,
                None,
                projection_guard,
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

        drop(service.take_activation_lifecycle_fence());
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
        let writer = crate::pointcloud::CloudMapShmWriter::new_for_test(&path)
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
            let goal_sha256 = schema::goal_intent_goal_sha256(&goal)
                .map_err(|error| ChildNavigationSubmitError::Rejected(error.to_string()))?;
            let admission_context = crate::nav_runtime::NavAdmissionContext::new(
                self.facade.mapping_lifecycle_lock.clone(),
                self.facade.committed_map_authority.clone(),
                self.facade.map_revision_latest_tx.subscribe(),
            );
            let admission_io = crate::nav_runtime::NavAdmissionIo::new(
                self.facade.state.clone(),
                self.facade.transport.clone(),
                self.facade.transport_backend_name,
                self.facade.lcm_backend_available,
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
                    boundary_sha256: [0; 32],
                    boundary_points: Vec::new(),
                    motion_guard_generation: request.motion_admission.guard_generation as i64,
                    motion_guard_boot_nonce: request.motion_admission.guard_boot_nonce,
                    motion_stop_generation: request.motion_admission.motion_stop_generation as i64,
                    goal_sha256,
                    goal,
                },
                crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
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

    async fn retain_process_stop_activation_fence(
        service: &GeometryRuntimeService,
        handle: &GeometryRuntimeHandle,
        activation_request_id: &str,
        identity: [u8; 32],
    ) {
        let retained_global_map = RetainedActivationGlobalMap {
            activation_request_key: identity.into(),
            map_frame_id: "map".to_string(),
            installed_cloud_revision: 7,
            cloud_metadata: crate::pointcloud::CommittedCloudMapMetadata {
                revision: 7, point_count: 1, voxel_size: crate::pointcloud::VOXEL_SIZE,
                session_generation: 1, producer_epoch: [1; 16], layout_generation: 1,
            },
            cloud_receipt: crate::mapping_commit::CloudCommitReceipt {
                commit_id: crate::mapping_commit::MappingCommitId {
                    boot_nonce: u128::from_be_bytes(identity[..16].try_into().unwrap()),
                    sequence: 7,
                },
                session_generation: 1,
                occupancy_revision: 6,
                cloud_revision: 7,
                source_timestamp_ns: 8,
                point_count: 1,
            },
            legacy_frame: Some(nuwax_protocol::RawLcmFrameBuilder::new(
                LcmTopic::new("/global_map").expect("retained map topic"),
                LcmMessageType::new("sensor_msgs.PointCloud2").expect("retained map message type"),
            )
            .payload(vec![identity[0]])
            .seqno(7)
            .build()),
        };
        let projection_guard = handle.lock_explore_projection_owned().await;
        let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        assert!(
            service
                .retain_activation_lifecycle_fence(
                    activation_request_id,
                    identity,
                    Some(retained_global_map),
                    projection_guard,
                    lifecycle_guard,
                )
                .is_ok(),
            "retain process Stop activation fence"
        );
    }

    fn assert_held_process_stop_activation_fence(
        service: &GeometryRuntimeService,
        activation_request_id: &str,
        identity: [u8; 32],
    ) {
        let slot = service.activation_lifecycle_fence.lock();
        let ActivationLifecycleFenceSlot::Held(fence) =
            slot.as_ref().expect("retained activation lifecycle fence")
        else {
            panic!("activation lifecycle fence must be restored as Held");
        };
        assert_eq!(fence.activation_request_id, activation_request_id);
        assert_eq!(fence.activation_request_key.legacy_digest(), Some(identity));
        let retained = fence
            .retained_global_map
            .as_ref()
            .expect("retained global-map receipt");
        assert_eq!(retained.activation_request_key.legacy_digest(), Some(identity));
        assert_eq!(retained.installed_cloud_revision, 7);
        assert_eq!(retained.cloud_receipt.cloud_revision, 7);
    }

    #[tokio::test]
    async fn process_stop_retained_correction_timeout_replays_restore_pending_failure() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let lio_gate = Arc::new(crate::lio_ingress::LioIngressRoutingGate::new(Arc::new(
            PhaseALioSink::default(),
        )));
        let correction_phase_a =
            crate::correction_coordinator::CorrectionPhaseAControl::for_tests();
        let (shutdown, mut listener) =
            crate::server::production_shutdown_channel(lio_gate, correction_phase_a, None);
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let identity = [0x51; 32];
        retain_process_stop_activation_fence(
            &service,
            &handle,
            "process-stop-restore-pending",
            identity,
        )
        .await;
        let correction_entered = Arc::new(tokio::sync::Barrier::new(2));
        service
            .shutdown
            .as_ref()
            .expect("production shutdown coordinator")
            .inject_correction_settlement_blocker(correction_entered.clone());
        let first = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), correction_entered.wait())
            .await
            .expect("first Stop must enter correction settlement");
        let held_lifecycle = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        let duplicate = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        let first_error = tokio::time::timeout(Duration::from_secs(1), first)
            .await
            .expect("first Stop correction/restore deadline")
            .expect("first Stop join")
            .expect_err("correction and restore timeout must fail first Stop");
        let duplicate_error = tokio::time::timeout(Duration::from_secs(1), duplicate)
            .await
            .expect("duplicate Stop failure replay deadline")
            .expect("duplicate Stop join")
            .expect_err("duplicate Stop must replay first-owner failure");
        assert_eq!(first_error.code(), tonic::Code::DeadlineExceeded);
        assert!(first_error
            .message()
            .contains("ACTIVATION_LIFECYCLE_FENCE_RESTORE_PENDING"));
        assert_eq!(duplicate_error.code(), tonic::Code::FailedPrecondition);
        assert!(duplicate_error
            .message()
            .contains("first Process Stop attempt failed"));
        {
            let slot = service.activation_lifecycle_fence.lock();
            let ActivationLifecycleFenceSlot::RestorePending(pending) =
                slot.as_ref().expect("restore pending slot")
            else {
                panic!("restore timeout must preserve a RestorePending fence");
            };
            assert_eq!(
                pending.activation_request_id,
                "process-stop-restore-pending"
            );
            assert_eq!(pending.activation_request_key.legacy_digest(), Some(identity));
            assert!(pending.retained_global_map.is_some());
        }
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err()
        );
        drop(held_lifecycle);
    }

    #[tokio::test]
    async fn process_stop_post_zero_lifecycle_reacquire_timeout_is_bounded() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        retain_process_stop_activation_fence(
            &service,
            &handle,
            "process-stop-post-zero-timeout",
            [0x53; 32],
        )
        .await;
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
        assert!(!service.has_activation_lifecycle_fence());
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
            drop(self.service.take_activation_lifecycle_fence().expect("fixture retained fence"));
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
        assert!(reference.sha256.is_empty());
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
                motion_guard_generation:7, motion_guard_boot_nonce:vec![7;16], motion_stop_generation:1,
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
                assert_eq!(goal.motion_guard_generation,7);
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
            for sequence in 0..3 {
                let timestamp = SOURCE_NS + sequence * 1_000_000_000;
                assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
                    &source.handle, &points, 12, timestamp, timestamp, "map",
                ).await.expect("native source cloud commit"));
            }
            // This is the actual runtime exporter and backend worker, not a
            // hand-authored checkpoint or an assembly/classification fixture.
            let checkpoint = GeometryRuntime::export_mapping_backend_checkpoint(
                &source.service,
                Request::new(assigned_wire_roundtrip(&ExportMappingBackendCheckpointRequest {
                    checkpoint_schema_version: 2,
                })),
            ).await.expect("native schema-2 export").into_inner();
            let checkpoint = assigned_wire_roundtrip(&checkpoint);
            assert_eq!(checkpoint.schema_version, 2);
            assert!(!checkpoint.payload.is_empty());
            assert!(checkpoint.content_sha256.is_empty());
            let cloud = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
            assert_eq!(cloud.payload_schema_version, 2);
            assert!(cloud.xyz_sha256.is_empty());
            let fine_reference = checkpoint.fine_map_artifact.as_ref().expect("native AR2 fine export");
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
                &source.service, &destination.service, fine_reference, b"NUWAXAR2",
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
            assert!(!activated.legacy_checkpoint_reconstructed);
            assert_eq!(activated.activation_receipt_schema_version, 2);
            assert_eq!(activated.activation_receipt_disposition, ActivationReceiptDisposition::Sealed as i32);
            assert_eq!(activated.relocalization_target_available, paired,
                "availability requires the actual cloud relocalization actor");
            let current = GeometryRuntime::get_mapping_state(&destination.service, Request::new(Empty {}))
                .await.unwrap().into_inner();
            assert_eq!(current.relocalization_target_available, paired);
            assert!(destination.service.activation_receipts.current_sealed_target_snapshot().is_none());
            assert!(activated.activation_request_identity_sha256.is_empty());
            assert!(activated.activation_identity_sha256.is_empty());
            assert!(activated.target_receipt_sha256.is_empty());
            assert!(activated.asset_manifest_sha256.is_empty());
            assert!(activated.asset_bundle_sha256.is_empty());
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
                let error = GeometryRuntime::activate_map(
                    &destination.service, Request::new(assigned_wire_roundtrip(&changed)),
                ).await.expect_err("changed checkpoint owner/body cannot reuse its assigned identity");
                assert_eq!(error.code(), tonic::Code::InvalidArgument);
                assert!(error.message().contains("replay conflict"), "mutation {mutation}: {error}");
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

    fn assert_legacy_cloud_unbound(state: &MappingSessionState, disposition: ActivationReceiptDisposition) {
        assert!(state.success && state.active, "legacy activation: {state:?}");
        assert_eq!(state.activation_receipt_disposition, disposition as i32);
        assert!(!state.relocalization_target_available);
        assert!(state.activation_identity_sha256.is_empty());
        assert!(state.target_receipt_sha256.is_empty());
        assert!(state.asset_manifest_sha256.is_empty());
        assert!(state.asset_bundle_sha256.is_empty());
        assert!(state.activation_receipt_v2.is_none());
    }

    fn recorded_legacy_clouds(backend: &CheckpointPublishBackend) -> Vec<RawLcmFrame> {
        backend.published.lock().unwrap().iter()
            .filter(|frame| frame.msg_type.fqn() == "sensor_msgs.PointCloud2")
            .cloned().collect()
    }

    #[tokio::test]
    async fn legacy_cloud_seals_unbound_and_replays_without_republication() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
                .into_iter().flat_map(f32::to_le_bytes).collect::<Vec<_>>();
            let request = cloud_activation_request(xyz);
            let activated = GeometryRuntime::activate_map(&fixture.service, Request::new(request.clone()))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&activated, ActivationReceiptDisposition::LegacyUnbound);
            assert_eq!(fixture.service.activation_receipts.retained_immutable_target_bytes(), 0);
            assert!(fixture.service.activation_receipts.current_sealed_target_snapshot().is_none());
            let clouds = recorded_legacy_clouds(&fixture.backend);
            assert_eq!(clouds.len(), 1);
            let replay = GeometryRuntime::activate_map(&fixture.service, Request::new(request))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&replay, ActivationReceiptDisposition::SealedReplay);
            assert!(replay.idempotent_replay);
            assert_eq!((replay.session_generation, replay.backend_generation, replay.revision),
                (activated.session_generation, activated.backend_generation, activated.revision));
            assert_eq!(recorded_legacy_clouds(&fixture.backend).len(), 1);
            let queried = GeometryRuntime::get_mapping_state(&fixture.service, Request::new(Empty {}))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&queried, ActivationReceiptDisposition::LegacyUnbound);
            assert_eq!(fixture.service.activation_receipts.retained_immutable_target_bytes(), 0);
            assert!(fixture.service.activation_receipts.current_sealed_target_snapshot().is_none());
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    #[tokio::test]
    async fn legacy_cloud_resumes_exact_retained_publication_unbound() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
                .into_iter().flat_map(f32::to_le_bytes).collect::<Vec<_>>();
            let source_bytes = xyz.len();
            let request = cloud_activation_request(xyz);
            fixture.backend.fail_after_pointcloud_observe.store(true, Ordering::Release);
            let failure = GeometryRuntime::activate_map(&fixture.service, Request::new(request.clone()))
                .await.expect_err("observed publication failure must remain pending");
            assert!(failure.message().contains("publication failed"));
            assert!(fixture.service.has_activation_lifecycle_fence());
            assert!(fixture.handle.mapping_lifecycle_lock.try_lock().is_err());
            assert_eq!(fixture.service.activation_receipts.retained_immutable_target_bytes(), source_bytes);
            let before = fixture.handle.mapping_adapter.as_ref().unwrap().lock().snapshot();
            let first_clouds = recorded_legacy_clouds(&fixture.backend);
            assert_eq!(first_clouds.len(), 1);
            fixture.backend.fail_after_pointcloud_observe.store(false, Ordering::Release);
            let resumed = GeometryRuntime::activate_map(&fixture.service, Request::new(request.clone()))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&resumed, ActivationReceiptDisposition::LegacyUnbound);
            assert!(resumed.idempotent_replay);
            assert_eq!((resumed.session_generation, resumed.revision),
                (before.session_generation, before.revision));
            assert!(!fixture.service.has_activation_lifecycle_fence());
            assert!(fixture.handle.mapping_lifecycle_lock.try_lock().is_ok());
            let clouds = recorded_legacy_clouds(&fixture.backend);
            assert_eq!(clouds.len(), 2);
            assert_eq!(clouds[0].header, clouds[1].header);
            assert_eq!(clouds[0].topic, clouds[1].topic);
            assert_eq!(clouds[0].msg_type, clouds[1].msg_type);
            assert_eq!(clouds[0].payload, clouds[1].payload);
            assert_eq!(fixture.service.activation_receipts.retained_immutable_target_bytes(), 0);
            let queried = GeometryRuntime::get_mapping_state(&fixture.service, Request::new(Empty {}))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&queried, ActivationReceiptDisposition::LegacyUnbound);
            let replay = GeometryRuntime::activate_map(&fixture.service, Request::new(request))
                .await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&replay, ActivationReceiptDisposition::SealedReplay);
            assert_eq!(recorded_legacy_clouds(&fixture.backend).len(), 2);
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
    }

    #[tokio::test]
    async fn retained_legacy_frame_checks_current_receipt_and_never_replaces_capture() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            let xyz = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0]
                .into_iter().flat_map(f32::to_le_bytes).collect::<Vec<_>>();
            let activated = GeometryRuntime::activate_map(&fixture.service,
                Request::new(cloud_activation_request(xyz))).await.unwrap().into_inner();
            assert_legacy_cloud_unbound(&activated, ActivationReceiptDisposition::LegacyUnbound);
            let key: [u8; 32] = activated.activation_request_identity_sha256.try_into().unwrap();
            let _lifecycle = fixture.handle.mapping_lifecycle_lock.lock().await;
            let (metadata, receipt, cached) = {
                let writer = fixture.handle.cloud_map_writer.lock();
                let writer = writer.as_ref().unwrap();
                (writer.committed_metadata().unwrap(), writer.last_cloud_receipt().unwrap(),
                    writer.materialized_committed_snapshot().unwrap())
            };
            let committed = CommittedMappingCloudMap {
                notification: CloudMapNotification { revision: metadata.revision,
                    point_count: metadata.point_count, voxel_size: metadata.voxel_size,
                    session_generation: metadata.session_generation },
                source_timestamp_ns: receipt.source_timestamp_ns, metadata, receipt: receipt.clone(),
            };
            let mut foreign = CommittedMappingCloudMap {
                notification: committed.notification.clone(), source_timestamp_ns: committed.source_timestamp_ns,
                metadata, receipt: receipt.clone(),
            };
            foreign.receipt.commit_id.sequence += 1;
            let mut rejected = RetainedActivationGlobalMap::from_committed(key, "map", &foreign).unwrap();
            let error = rejected.capture_materialized_legacy_frame(&fixture.handle.cloud_map_writer, &foreign)
                .expect_err("matching metadata cannot substitute a different current receipt");
            assert!(error.contains("metadata/receipt changed"));
            assert!(rejected.legacy_frame.is_none());
            let after = fixture.handle.cloud_map_writer.lock().as_ref().unwrap()
                .materialized_committed_snapshot().unwrap();
            assert!(Arc::ptr_eq(&cached.xyz_f32_le, &after.xyz_f32_le));
            let mut retained = RetainedActivationGlobalMap::from_committed(key, "map", &committed).unwrap();
            retained.capture_materialized_legacy_frame(&fixture.handle.cloud_map_writer, &committed).unwrap();
            let first = recorded_legacy_clouds(&fixture.backend);
            assert_eq!(retained.legacy_frame.as_ref().unwrap().payload, first[0].payload);
            let original_payload = retained.legacy_frame.as_ref().unwrap().payload.as_ptr();
            fixture.handle.cloud_map_writer.lock().as_mut().unwrap()
                .begin_session(metadata.session_generation + 1).unwrap();
            assert!(fixture.handle.cloud_map_writer.lock().as_ref().unwrap()
                .materialized_committed_snapshot().is_none());
            retained.capture_materialized_legacy_frame(&fixture.handle.cloud_map_writer, &committed).unwrap();
            assert_eq!(retained.legacy_frame.as_ref().unwrap().payload.as_ptr(), original_payload);
            assert_eq!(retained.legacy_frame.as_ref().unwrap().payload, first[0].payload);
            assert_eq!(recorded_legacy_clouds(&fixture.backend).len(), 1);
        })).await;
        fixture.finish().await;
        if let Err(panic) = outcome { std::panic::resume_unwind(panic); }
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
            asset_identity: None,
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
                member_identity: None,
            }),
            mapping_backend_checkpoint: None,
            activation_request_id: String::new(),
            asset_manifest_sha256: Vec::new(),
            asset_bundle_sha256: Vec::new(),
            expected_runtime_generation: 0,
            activation_contract_version: 0,
            asset_identity: None,
        }
    }

    fn modern_activation_request(
        mut request: ActivateMapRequest,
        activation_request_id: &str,
        runtime_generation: u64,
    ) -> ActivateMapRequest {
        let occupancy_sha256 =
            canonical_occupancy_sha256(request.checkpoint.as_ref().unwrap()).unwrap();
        let cloud_sha256 = request.cloud_checkpoint.as_ref().map(|cloud| {
            let target = immutable_target_from_checkpoint(cloud, "map", true).unwrap();
            canonical_cloud_sha256(&target).unwrap()
        });
        let asset_manifest_sha256 = [0x5a; 32];
        let asset_bundle_sha256 =
            crate::map_checksum::sha256_bytes(&canonical_asset_bundle_preimage(
                asset_manifest_sha256,
                occupancy_sha256,
                cloud_sha256,
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
            0,
            "map",
        )
        .await
        .expect("canonical source cloud handoff"));
        let checkpoint =
            GeometryRuntime::export_mapping_backend_checkpoint(&service, Request::new(ExportMappingBackendCheckpointRequest::default()))
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
            member_identity: None,
        };
        MappingBackendCheckpoint {
            fine_map_artifact: None,
            dense_cloud_artifact: None,
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
            graph_artifact: None,
            target_kind: String::new(),
            checkpoint_id: String::new(),
            backend_payload_schema_version: 0,
            backend_identity: None, occupancy_identity: None,
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
            asset_identity: None,
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
        let retained_identity = [0x52; 32];
        retain_process_stop_activation_fence(
            &service,
            &handle,
            "process-stop-exact-terminal-retained",
            retained_identity,
        )
        .await;
        let receipt_consume_barrier = Arc::new(tokio::sync::Barrier::new(2));
        service.inject_stop_after_retained_receipt_consume_barrier(receipt_consume_barrier.clone());
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
        assert_eq!(service.retained_receipt_consume_count(), 0);
        assert!(
            tokio::time::timeout(
                Duration::from_millis(40),
                service.wait_for_retained_receipt_consume(),
            )
            .await
            .is_err(),
            "retained receipt must not be consumed before public Nav ZERO"
        );
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

        tokio::time::timeout(
            Duration::from_secs(1),
            service.wait_for_retained_receipt_consume(),
        )
        .await
        .expect("retained receipt consume must follow public Nav ZERO");
        tokio::time::timeout(Duration::from_secs(1), receipt_consume_barrier.wait())
            .await
            .expect("Process Stop must pause immediately after retained receipt consume");
        assert_eq!(service.retained_receipt_consume_count(), 1);
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must not begin before the consume observation is verified"
        );
        tokio::time::timeout(Duration::from_secs(1), receipt_consume_barrier.wait())
            .await
            .expect("the test must release Process Stop after consume verification");

        let response = stop_task
            .await
            .expect("process Stop task")
            .expect("process Stop response")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");
        assert_eq!(service.retained_receipt_consume_count(), 1);
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert!(!service.has_activation_lifecycle_fence());
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
        let retained_identity = [0x41; 32];
        retain_process_stop_activation_fence(
            &service,
            &handle,
            "process-stop-explore-retained",
            retained_identity,
        )
        .await;

        let error = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect_err("retained ownership must reject process Stop");

        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("safety ownership retained"));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");
        assert_held_process_stop_activation_fence(
            &service,
            "process-stop-explore-retained",
            retained_identity,
        );
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
        let retained_identity = [0x42; 32];
        retain_process_stop_activation_fence(
            &service,
            &handle,
            "process-stop-public-nav-retained",
            retained_identity,
        )
        .await;

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
        assert_held_process_stop_activation_fence(
            &service,
            "process-stop-public-nav-retained",
            retained_identity,
        );
        tokio::time::timeout(
            Duration::from_secs(1),
            readiness.wait_for_readiness_revocation(),
        )
        .await
        .expect("failed Stop must revoke readiness");
        assert!(handle.mapping_lifecycle_lock.try_lock().is_err());
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
        handle.map_revision_tx.send(queued_event).expect("coalescer remains registered");
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
                    width_cells: 40,
                    height_cells: 40,
                    origin_xy: (-2.0, -2.0),
                    voxel_size: 0.1,
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
        // Activation remains on its supported legacy input contract. The v2
        // opt-in below is an export request, not an activation-gate bypass.
        let mut request = cloud_activation_request(xyz);
        request.checkpoint.as_mut().unwrap().meta.as_mut().unwrap().timestamp_ns = SOURCE_NS;
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
            Request::new(ExportMappingBackendCheckpointRequest { checkpoint_schema_version: 2 }),
        ).await.expect("v2 export immediately after the first real pose").into_inner();
        assert_eq!(checkpoint.schema_version, 2);
        assert!(checkpoint.content_sha256.is_empty());
        assert_eq!(checkpoint.occupancy_receipt, Some(occupancy_commit_receipt_proto(&occupancy_before)));
        assert_eq!(checkpoint.cloud_receipt, Some(cloud_commit_receipt_proto(&cloud_before)));
        assert_eq!(checkpoint.canonical_occupancy_checkpoint.as_ref(), Some(&raw_before));
        let exported_cloud = checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();
        assert_eq!(exported_cloud.payload_schema_version, 2);
        assert!(exported_cloud.xyz_sha256.is_empty());
        assert_eq!(exported_cloud.xyz_f32_le, snapshot_before.xyz_f32_le);
        assert_eq!(exported_cloud.source_revision, snapshot_before.revision);
        assert_eq!(exported_cloud.point_count, snapshot_before.point_count);
        assert_eq!(exported_cloud.voxel_size.to_bits(), snapshot_before.voxel_size.to_bits());
        let fine_ref = checkpoint.fine_map_artifact.as_ref().expect("AR2 fine checkpoint member");
        assert_eq!(fine_ref.identity_schema_version, 2);
        assert!(fine_ref.sha256.is_empty());
        let member = fine_ref.member_identity.as_ref().unwrap();
        assert_eq!(member.frame_generation, frame_after);
        assert_eq!(member.producer_generation, handle.nav.producer_runtime_generation());
        let file = service.graph_artifacts.lock().open(fine_ref).expect("open exported fine member");
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file).expect("read AR2 fine member");
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
            let fetched = handle.read_navigation_map_block(&old_fine.manifest().identity(), reference)
                .expect("admit fetch under retained old root")
                .await.expect("retained fetch completion").expect("retained old object bytes");
            assert_eq!(fetched, bytes_before);
        }

        // A later odometry observation must not reseal the already anchored root.
        commit_checkpoint_frame_test_pose(&handle, SOURCE_NS + 34_000_002).await;
        let repeated = GeometryRuntime::export_mapping_backend_checkpoint(
            &service,
            Request::new(ExportMappingBackendCheckpointRequest { checkpoint_schema_version: 2 }),
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
            Request::new(ExportMappingBackendCheckpointRequest { checkpoint_schema_version: 2 }),
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
            CloudMapShmWriter::new_for_test(&cloud_path).expect("target cloud-map writer"),
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
            asset_identity: None,
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
        assert_eq!(restored_cloud.xyz_f32_le, source_cloud.xyz_f32_le);

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
            assert_eq!(handle.state.read().await.map.revision > 0, sequence == 3);
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
            Some(CloudMapShmWriter::new_for_test(&cloud_path).expect("checkpoint retry cloud-map writer"));
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
    async fn modern_2d_activation_seals_and_replays_without_relocalization_target() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activation-2d-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() = Some(
            crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("empty cloud writer"),
        );
        let service = GeometryRuntimeService::new(handle.clone());
        let request = modern_activation_request(
            activation_request("mapload:two-d", "warehouse-2d"),
            "act-modern-2d",
            1,
        );
        assert!(request.cloud_checkpoint.is_none());
        assert!(request.mapping_backend_checkpoint.is_none());
        let activated = GeometryRuntime::activate_map(&service, Request::new(request.clone()))
            .await
            .expect("ordinary 2D activation")
            .into_inner();
        assert!(activated.success && activated.active);
        assert_eq!(activated.activation_receipt_disposition, ActivationReceiptDisposition::Sealed as i32);
        assert_eq!(activated.activation_identity_sha256.len(), 32);
        assert!(activated.revision > 0);
        assert!(!activated.relocalization_target_available);
        assert!(activated.target_receipt_sha256.is_empty());
        assert!(service.activation_receipts.current_sealed_target_snapshot().is_none());
        assert_eq!(handle.state.read().await.map.data, vec![-1, 0, 100, 50]);
        let cleared = handle.cloud_map_writer.lock().as_ref().unwrap()
            .committed_snapshot().expect("2D activation clears the previous cloud session");
        assert_eq!(cleared.point_count, 0);
        assert!(cleared.xyz_f32_le.is_empty());
        let replay = GeometryRuntime::activate_map(&service, Request::new(request))
            .await
            .expect("2D activation replay")
            .into_inner();
        assert!(replay.success && replay.idempotent_replay);
        assert_eq!(replay.revision, activated.revision);
        assert!(!replay.relocalization_target_available);
        assert!(replay.target_receipt_sha256.is_empty());
        GeometryRuntime::stop_mapping(&service, Request::new(StopMappingRequest {
            session_id: "mapload:two-d".to_owned(),
            reason: "2D test complete".to_owned(),
        })).await.expect("2D stop");
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
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
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
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
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
            Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
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
                Some(crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("cloud writer"));
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
            crate::pointcloud::CloudMapShmWriter::new_for_test(&cloud_path).expect("test cloud-map writer"),
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
                planned_path: vec![[1.0,2.0,0.4],[2.0,2.0,0.4]],
                local_path: vec![[1.0,2.0,0.4],[1.4,2.1,0.5]],
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
            None,
            None,
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
            assigned_request_v2: None,
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

#[cfg(test)]
mod assigned_checkpoint_assembly_tests {
use super::*;
fn frozen() -> MappingBackendCheckpoint {
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
    assert_eq!(ExportMappingBackendCheckpointRequest::decode(&[][..]).unwrap().checkpoint_schema_version,0);
    assert_eq!(checkpoint_envelope_version(0).unwrap(),1);assert_eq!(checkpoint_envelope_version(1).unwrap(),1);assert!(checkpoint_envelope_version(3).is_err());
    let request=ExportMappingBackendCheckpointRequest{checkpoint_schema_version:2};
    assert_eq!(ExportMappingBackendCheckpointRequest::decode(request.encode_to_vec().as_slice()).unwrap(),request);
    let original=frozen();let checkpoint=assemble_mapping_checkpoint_export(original.clone(),2,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
    assert_eq!(checkpoint.schema_version,2);assert_eq!(checkpoint.backend_payload_schema_version,1);
    assert_eq!(checkpoint.checkpoint_id,original.checkpoint_id);assert_eq!(checkpoint.payload,original.payload);
    assert!(checkpoint.content_sha256.is_empty());assert_eq!(checkpoint.canonical_occupancy_checkpoint,original.canonical_occupancy_checkpoint);
    let cloud=checkpoint.canonical_cloud_checkpoint.as_ref().unwrap();assert!(cloud.xyz_sha256.is_empty());
    assert_eq!(cloud.xyz_f32_le,original.canonical_cloud_checkpoint.unwrap().xyz_f32_le);assert_eq!(cloud.payload_schema_version,2);
    let encoded=checkpoint.encode_to_vec();assert_eq!(MappingBackendCheckpoint::decode(encoded.as_slice()).unwrap(),checkpoint);
    assert!(validate_mapping_backend_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).is_err());
}
#[test]
fn assigned_v2_backend_and_occupancy_members_require_exact_parent_binding() {
    let valid = assemble_mapping_checkpoint_export(frozen(), 2, crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
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
    let valid=assemble_mapping_checkpoint_export(frozen(),2,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
    for field in 0..13 {
        let mut bad=valid.clone();match field {
            0=>bad.content_sha256=vec![7;32],1=>bad.checkpoint_id="A".repeat(32),2=>bad.backend_payload_schema_version=2,
            3=>bad.payload_size+=1,4=>bad.payload[8]=2,5=>bad.payload[12]=1,
            6=>bad.canonical_cloud_checkpoint.as_mut().unwrap().xyz_sha256="01".repeat(32),
            7=>bad.canonical_cloud_checkpoint.as_mut().unwrap().member_identity.as_mut().unwrap().checkpoint_id="2".repeat(32),
            8=>bad.canonical_cloud_checkpoint.as_mut().unwrap().xyz_f32_le[..4].copy_from_slice(&f32::NAN.to_le_bytes()),
            9=>bad.canonical_occupancy_checkpoint.as_mut().unwrap().data[0]=254,
            10=>bad.canonical_occupancy_checkpoint.as_mut().unwrap().meta.as_mut().unwrap().origin_x=f64::NAN,
            11=>bad.cloud_receipt.as_mut().unwrap().cloud_revision+=1,_=>bad.canonical_cloud_checkpoint.as_mut().unwrap().frame_id="odom".into() }
        assert!(validate_assigned_mapping_checkpoint(&bad,crate::mapping_backend::MappingBackendKind::Voxel).is_err(),"field {field}");
    }
    let mut bad=frozen();bad.content_sha256=vec![1;32];assert!(assemble_mapping_checkpoint_export(bad,2,crate::mapping_backend::MappingBackendKind::Voxel).is_err());
}
#[test]
fn assigned_v2_assembly_checks_full_artifact_parent_descriptor() {
    let mut checkpoint=assemble_mapping_checkpoint_export(frozen(),2,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
    let member=checkpoint_member_identity(&checkpoint.checkpoint_id,CheckpointMemberRole::DenseCloud,checkpoint.occupancy_receipt.as_ref().unwrap(),checkpoint.cloud_receipt.as_ref().unwrap(),"map",None);
    checkpoint.dense_cloud_artifact=Some(GraphArtifactRef{id:"2".repeat(32),size:100,identity_schema_version:2,member_identity:Some(member),..Default::default()});
    validate_assigned_mapping_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).unwrap();
    let mut collision=checkpoint.clone();let mut fine=collision.dense_cloud_artifact.clone().unwrap();
    fine.member_identity.as_mut().unwrap().role=CheckpointMemberRole::FineMap as i32;fine.member_identity.as_mut().unwrap().producer_generation=6;
    collision.fine_map_artifact=Some(fine);
    assert!(validate_assigned_mapping_checkpoint(&collision,crate::mapping_backend::MappingBackendKind::Voxel).unwrap_err().contains("cannot share one artifact ID"));
    checkpoint.dense_cloud_artifact.as_mut().unwrap().member_identity.as_mut().unwrap().cloud_receipt.as_mut().unwrap().cloud_revision+=1;
    assert!(validate_assigned_mapping_checkpoint(&checkpoint,crate::mapping_backend::MappingBackendKind::Voxel).is_err());
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
        assert!(matches!(classify_activation_request_identity(&request, 7).unwrap(), ActivationRequestIdentityClass::Assigned(_)));
        assert!(classify_activation_request_identity(&request, 8).is_err());
        for version in [0, 1, 3] {
            let mut skew = request.clone(); skew.activation_contract_version = version;
            assert!(classify_activation_request_identity(&skew, 7).is_err());
        }
        let mut duplicate = request; duplicate.cloud_checkpoint = Some(CloudMapCheckpoint::default());
        assert!(classify_activation_request_identity(&duplicate, 7).is_err());
    }

    #[test]
    fn assigned_publication_fence_retains_no_legacy_cloud_frame() {
        let metadata = crate::activation_request::ActivationRequestMetadataV2::from_wire(&metadata_request(), 7).unwrap();
        let key = ActivationRequestKey::Assigned(metadata);
        let receipt = crate::mapping_commit::CloudCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 9, sequence: 10 },
            session_generation: 2, occupancy_revision: 3, cloud_revision: 4,
            source_timestamp_ns: 100, point_count: 1,
        };
        let committed = CommittedMappingCloudMap {
            notification: CloudMapNotification { revision: 4, point_count: 1, voxel_size: 0.1, session_generation: 2 },
            source_timestamp_ns: 100,
            metadata: crate::pointcloud::CommittedCloudMapMetadata {
                revision: 4, point_count: 1, voxel_size: 0.1, session_generation: 2,
                producer_epoch: [1; 16], layout_generation: 1,
            },
            receipt,
        };
        // No XYZ body is supplied: a typed fence must not eagerly encode one.
        let mut retained = RetainedActivationGlobalMap::from_committed(key.clone(), "map", &committed).unwrap();
        assert!(retained.legacy_frame.is_none());
        retained.capture_materialized_legacy_frame(&Arc::new(parking_lot::Mutex::new(None)), &committed).unwrap();
        assert!(retained.legacy_frame.is_none());
        retained.validate_replay(key.clone(), "map", &committed).unwrap();
        assert!(retained.validate_replay(key, "odom", &committed).is_err());
    }
}

#[cfg(test)]
#[path = "owned_occupancy_bytes_tests.rs"]
mod owned_occupancy_bytes_tests;
