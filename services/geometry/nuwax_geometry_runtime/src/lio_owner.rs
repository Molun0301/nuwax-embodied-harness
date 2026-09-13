//! Device LIO ingress ownership and acknowledged real-LCM shutdown.

use std::collections::HashSet;
use std::os::unix::process::ExitStatusExt;
use std::process::ExitStatus;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use async_trait::async_trait;
use nuwax_codec::lcm::{lio, schema};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_transport::{LcmTransport, OwnedLcmSubscription};
use tokio::sync::{oneshot, watch};
use tokio::task::{JoinHandle, JoinSet};
use tracing::{info, warn};

use crate::channel_registry::{ChannelKey, ValidatedChannelBindings};
use crate::lcm_runtime::{
    GeometryRuntimeHandle, GeometryRuntimeTaskOwner, GeometryTaskShutdownOutcome,
    SpawnedGeometryRuntime, ValidatedFrameBindings,
};
use crate::mapping_adapter::{
    MappingFinalAttempt, MappingFinalOwner, MappingPendingCause, ShutdownFailures,
};
use crate::native_lio_owner::{BoundNativeLioRuntime, NativeProcessOwner};

pub(crate) const DEFAULT_LIO_ODOMETRY_CHANNEL: &str =
    "/geometry/lio/odometry#nuwax_msgs.LioOdometryEnvelope";
pub(crate) const DEFAULT_LIO_CLOUD_CHANNEL: &str =
    "/geometry/lio/cloud#nuwax_msgs.LioCloudEnvelope";
pub(crate) const DEFAULT_LIO_STATUS_CHANNEL: &str = "/geometry/lio/status#nuwax_msgs.LioStatus";

const LIO_ODOMETRY_CHANNEL_ENV: &str = "NUWAX_GEOMETRY_RUNTIME_LIO_ODOMETRY_CHANNEL";
const LIO_ODOMETRY_CHANNEL_PLATFORM_ENV: &str = "NUWAX_LCM_CHANNEL_LIO_ODOMETRY";
const LIO_CLOUD_CHANNEL_ENV: &str = "NUWAX_GEOMETRY_RUNTIME_LIO_CLOUD_CHANNEL";
const LIO_CLOUD_CHANNEL_PLATFORM_ENV: &str = "NUWAX_LCM_CHANNEL_LIO_CLOUD";
const LIO_STATUS_CHANNEL_ENV: &str = "NUWAX_GEOMETRY_RUNTIME_LIO_STATUS_CHANNEL";
const LIO_STATUS_CHANNEL_PLATFORM_ENV: &str = "NUWAX_LCM_CHANNEL_LIO_STATUS";
const LIO_SENSOR_FRAME_ENV: &str = "NUWAX_GEOMETRY_LIDAR_FRAME";
const DEFAULT_LIO_SENSOR_FRAME: &str = "utlidar_lidar";

const LIO_ODOMETRY_TYPE: &str = "nuwax_msgs.LioOdometryEnvelope";
const LIO_CLOUD_TYPE: &str = "nuwax_msgs.LioCloudEnvelope";
const LIO_STATUS_TYPE: &str = "nuwax_msgs.LioStatus";
const LEGACY_ODOMETRY_TYPE: &str = "nav_msgs.Odometry";
const LEGACY_CLOUD_TYPE: &str = "sensor_msgs.PointCloud2";
const LIO_STARTUP_IO_TIMEOUT: Duration = Duration::from_secs(2);
const LIO_CONTROL_IO_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(not(test))]
const NATIVE_LIO_TERM_GRACE: Duration = Duration::from_millis(1_500);
#[cfg(test)]
const NATIVE_LIO_TERM_GRACE: Duration = Duration::from_millis(100);
#[cfg(not(test))]
const NATIVE_LIO_FORCE_REAP_TIMEOUT: Duration = Duration::from_millis(1_500);
#[cfg(test)]
const NATIVE_LIO_FORCE_REAP_TIMEOUT: Duration = Duration::from_millis(500);
#[cfg(not(test))]
const LIO_WORKER_JOIN_TIMEOUT: Duration = Duration::from_secs(4);
#[cfg(test)]
const LIO_WORKER_JOIN_TIMEOUT: Duration = Duration::from_secs(1);
#[cfg(not(test))]
const LIO_SUPERVISOR_SHUTDOWN_TIMEOUT: Duration = Duration::from_millis(4_750);
#[cfg(test)]
const LIO_SUPERVISOR_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);
const GEOMETRY_SHUTDOWN_EVENT_SCHEMA_VERSION: u64 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GeometryShutdownStage {
    GeometryTasksSettled,
    CorrectionSettled,
    MappingRetired,
    LioJoined,
    ResourceCleanupComplete,
}

impl GeometryShutdownStage {
    fn event_name(self) -> &'static str {
        match self {
            Self::GeometryTasksSettled => {
                "nuwax.geometry.runtime.shutdown.geometry_tasks_settled.v1"
            }
            Self::CorrectionSettled => "nuwax.geometry.runtime.shutdown.correction_settled.v1",
            Self::MappingRetired => "nuwax.geometry.runtime.shutdown.mapping_retired.v1",
            Self::LioJoined => "nuwax.geometry.runtime.shutdown.lio_joined.v1",
            Self::ResourceCleanupComplete => {
                "nuwax.geometry.runtime.shutdown.resource_cleanup_complete.v1"
            }
        }
    }

    fn stable_name(self) -> &'static str {
        match self {
            Self::GeometryTasksSettled => "geometry_tasks_settled",
            Self::CorrectionSettled => "correction_settled",
            Self::MappingRetired => "mapping_retired",
            Self::LioJoined => "lio_joined",
            Self::ResourceCleanupComplete => "resource_cleanup_complete",
        }
    }

    fn ordinal(self) -> u64 {
        match self {
            Self::GeometryTasksSettled => 1,
            Self::CorrectionSettled => 2,
            Self::MappingRetired => 3,
            Self::LioJoined => 4,
            Self::ResourceCleanupComplete => 5,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct GeometryShutdownEventEmitter {
    runtime_generation: u64,
    #[cfg(test)]
    completed: Option<Arc<parking_lot::Mutex<Vec<GeometryShutdownStage>>>>,
}

impl GeometryShutdownEventEmitter {
    pub(crate) fn new(runtime_generation: u64) -> Self {
        Self {
            runtime_generation,
            #[cfg(test)]
            completed: None,
        }
    }

    pub(crate) fn emit_completed(&self, stage: GeometryShutdownStage) {
        info!(
            event_name = stage.event_name(),
            event_schema_version = GEOMETRY_SHUTDOWN_EVENT_SCHEMA_VERSION,
            runtime_generation = self.runtime_generation,
            stage_ordinal = stage.ordinal(),
            stage = stage.stable_name(),
            status = "completed",
            "geometry runtime shutdown stage completed"
        );
        #[cfg(test)]
        if let Some(completed) = &self.completed {
            completed.lock().push(stage);
        }
    }

    #[cfg(test)]
    pub(crate) fn recording(runtime_generation: u64) -> (Self, GeometryShutdownEventRecorder) {
        let completed = Arc::new(parking_lot::Mutex::new(Vec::new()));
        (
            Self {
                runtime_generation,
                completed: Some(completed.clone()),
            },
            GeometryShutdownEventRecorder { completed },
        )
    }
}

#[cfg(test)]
pub(crate) struct GeometryShutdownEventRecorder {
    completed: Arc<parking_lot::Mutex<Vec<GeometryShutdownStage>>>,
}

#[cfg(test)]
impl GeometryShutdownEventRecorder {
    pub(crate) fn stages(&self) -> Vec<GeometryShutdownStage> {
        self.completed.lock().clone()
    }
}

/// Immutable Device-LIO bindings validated before any transport side effect.
#[derive(Clone, Debug)]
pub(crate) struct DeviceLioOwnerSpec {
    pub(crate) public_odometry: (LcmTopic, LcmMessageType),
    pub(crate) public_cloud: (LcmTopic, LcmMessageType),
    pub(crate) status: (LcmTopic, LcmMessageType),
    pub(crate) legacy_odometry: (LcmTopic, LcmMessageType),
    pub(crate) legacy_cloud: (LcmTopic, LcmMessageType),
    pub(crate) sensor_frame: String,
    pub(crate) body_frame: String,
    pub(crate) odom_frame: String,
    pub(crate) map_frame: String,
}

impl DeviceLioOwnerSpec {
    /// Resolve and validate the complete Device owner contract without I/O.
    pub(crate) fn from_env(
        registry: &ValidatedChannelBindings,
        frames: &ValidatedFrameBindings,
    ) -> Result<Self, String> {
        let public_odometry_pattern = strict_env_or_default(
            LIO_ODOMETRY_CHANNEL_ENV,
            LIO_ODOMETRY_CHANNEL_PLATFORM_ENV,
            DEFAULT_LIO_ODOMETRY_CHANNEL,
        )?;
        let public_cloud_pattern = strict_env_or_default(
            LIO_CLOUD_CHANNEL_ENV,
            LIO_CLOUD_CHANNEL_PLATFORM_ENV,
            DEFAULT_LIO_CLOUD_CHANNEL,
        )?;
        let status_pattern = strict_env_or_default(
            LIO_STATUS_CHANNEL_ENV,
            LIO_STATUS_CHANNEL_PLATFORM_ENV,
            DEFAULT_LIO_STATUS_CHANNEL,
        )?;
        let sensor_frame = strict_env_or_default(
            LIO_SENSOR_FRAME_ENV,
            LIO_SENSOR_FRAME_ENV,
            DEFAULT_LIO_SENSOR_FRAME,
        )?;
        let (map_frame, odom_frame, body_frame) = frames.as_parts();
        let reserved = ChannelKey::ALL
            .into_iter()
            .map(|key| registry.binding(key).0)
            .collect::<Vec<_>>();
        Self::from_values(
            &public_odometry_pattern,
            &public_cloud_pattern,
            &status_pattern,
            registry.binding(ChannelKey::Odom),
            registry.binding(ChannelKey::PointCloud),
            reserved,
            &sensor_frame,
            body_frame,
            odom_frame,
            map_frame,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_values(
        public_odometry_pattern: &str,
        public_cloud_pattern: &str,
        status_pattern: &str,
        legacy_odometry: (LcmTopic, LcmMessageType),
        legacy_cloud: (LcmTopic, LcmMessageType),
        reserved_topics: impl IntoIterator<Item = LcmTopic>,
        sensor_frame: &str,
        body_frame: &str,
        odom_frame: &str,
        map_frame: &str,
    ) -> Result<Self, String> {
        let public_odometry = strict_typed_binding(
            "public_odometry",
            public_odometry_pattern,
            LIO_ODOMETRY_TYPE,
            lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
        )?;
        let public_cloud = strict_typed_binding(
            "public_cloud",
            public_cloud_pattern,
            LIO_CLOUD_TYPE,
            lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1,
        )?;
        let status = strict_typed_binding(
            "status",
            status_pattern,
            LIO_STATUS_TYPE,
            lio::LIO_STATUS_FINGERPRINT_V1,
        )?;
        validate_binding(
            "legacy_odometry",
            &legacy_odometry,
            LEGACY_ODOMETRY_TYPE,
            schema::ODOMETRY_FINGERPRINT,
        )?;
        validate_binding(
            "legacy_cloud",
            &legacy_cloud,
            LEGACY_CLOUD_TYPE,
            schema::POINT_CLOUD2_FINGERPRINT,
        )?;

        let public_channels = [
            public_odometry.0.channel(),
            public_cloud.0.channel(),
            status.0.channel(),
        ];
        if public_channels.into_iter().collect::<HashSet<_>>().len() != public_channels.len() {
            return Err(
                "LIO_PREFLIGHT_FAILED public topic channels must be pairwise distinct".into(),
            );
        }
        let reserved = reserved_topics
            .into_iter()
            .map(|topic| topic.channel().to_owned())
            .collect::<HashSet<_>>();
        for (label, binding) in [
            ("public_odometry", &public_odometry),
            ("public_cloud", &public_cloud),
            ("status", &status),
        ] {
            if reserved.contains(binding.0.channel()) {
                return Err(format!(
                    "LIO_PREFLIGHT_FAILED {label} channel={} conflicts with registry/legacy topic",
                    binding.0.channel()
                ));
            }
        }

        let sensor_frame = validate_frame("sensor", sensor_frame)?;
        let body_frame = validate_frame("body", body_frame)?;
        let odom_frame = validate_frame("odom", odom_frame)?;
        let map_frame = validate_frame("map", map_frame)?;
        let frame_values = [
            sensor_frame.as_str(),
            body_frame.as_str(),
            odom_frame.as_str(),
            map_frame.as_str(),
        ];
        if frame_values.into_iter().collect::<HashSet<_>>().len() != frame_values.len() {
            return Err(
                "LIO_PREFLIGHT_FAILED sensor/body/odom/map frames must be pairwise distinct".into(),
            );
        }

        Ok(Self {
            public_odometry,
            public_cloud,
            status,
            legacy_odometry,
            legacy_cloud,
            sensor_frame,
            body_frame,
            odom_frame,
            map_frame,
        })
    }
}

/// Fully bound source-owner selection for one Geometry runtime generation.
#[derive(Clone, Debug)]
pub(crate) enum LioOwnerSpec {
    Device(DeviceLioOwnerSpec),
    Native {
        shared: DeviceLioOwnerSpec,
        runtime: BoundNativeLioRuntime,
    },
}

impl From<DeviceLioOwnerSpec> for LioOwnerSpec {
    fn from(spec: DeviceLioOwnerSpec) -> Self {
        Self::Device(spec)
    }
}

impl LioOwnerSpec {
    pub(crate) fn native(shared: DeviceLioOwnerSpec, runtime: BoundNativeLioRuntime) -> Self {
        Self::Native { shared, runtime }
    }

    fn shared(&self) -> &DeviceLioOwnerSpec {
        match self {
            Self::Device(spec) => spec,
            Self::Native { shared, .. } => shared,
        }
    }

    fn backend_code(&self) -> i32 {
        match self {
            Self::Device(_) => lio::LIO_BACKEND_DEVICE,
            Self::Native { runtime, .. } => runtime.backend_code,
        }
    }

    fn source_subscriptions(&self) -> Vec<(WorkerKind, (LcmTopic, LcmMessageType))> {
        match self {
            Self::Device(spec) => vec![
                (WorkerKind::SourceOdometry, spec.legacy_odometry.clone()),
                (WorkerKind::SourceCloud, spec.legacy_cloud.clone()),
            ],
            Self::Native { runtime, .. } => vec![
                (
                    WorkerKind::SourceOdometry,
                    runtime.channels.source_odometry.clone(),
                ),
                (
                    WorkerKind::SourceCloud,
                    runtime.channels.source_cloud.clone(),
                ),
            ],
        }
    }

    fn first_pair_timeout(&self) -> Duration {
        match self {
            Self::Device(_) => LIO_STARTUP_IO_TIMEOUT,
            Self::Native { runtime, .. } => runtime.first_pair_timeout,
        }
    }

    fn startup_ack_timeout(&self) -> Duration {
        self.first_pair_timeout()
            .checked_add(LIO_SUPERVISOR_SHUTDOWN_TIMEOUT)
            .and_then(|timeout| timeout.checked_add(LIO_STARTUP_IO_TIMEOUT))
            .unwrap_or(Duration::MAX)
    }

    fn native_runtime(&self) -> Option<&BoundNativeLioRuntime> {
        match self {
            Self::Device(_) => None,
            Self::Native { runtime, .. } => Some(runtime),
        }
    }
}

/// The ingress methods invoked by the four acknowledged subscription workers.
///
/// A returned error rejects only the current source frame. Transport closure,
/// unsubscribe failure, or worker panic is what fails the generation owner.
#[async_trait]
pub(crate) trait LioIngressSink: Send + Sync + 'static {
    /// Close the generation fence synchronously before shutdown fanout.
    fn close(&self);
    /// Device ingress is ready after subscription ACKs. Native ingress
    /// overrides this to wait for one dry-validated private source pair.
    async fn wait_startup_ready(
        &self,
        _shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        Ok(())
    }
    async fn accept_public_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String>;
    async fn accept_public_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String>;
    async fn accept_legacy_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String>;
    async fn accept_legacy_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String>;
}

/// Cloneable stop control shared with the gRPC shutdown coordinator.
#[derive(Clone, Debug)]
pub(crate) struct LioShutdownControl {
    request_tx: watch::Sender<bool>,
    #[cfg(test)]
    completion_rx: watch::Receiver<Option<Result<(), String>>>,
}

#[cfg(test)]
type LioShutdownTestChannel = (
    LioShutdownControl,
    watch::Receiver<bool>,
    watch::Sender<Option<Result<(), String>>>,
);

impl LioShutdownControl {
    #[cfg(test)]
    pub(crate) fn test_channel() -> LioShutdownTestChannel {
        let (request_tx, request_rx) = watch::channel(false);
        let (completion_tx, completion_rx) = watch::channel(None);
        (
            Self {
                request_tx,
                completion_rx,
            },
            request_rx,
            completion_tx,
        )
    }

    pub(crate) fn request_shutdown(&self) {
        self.request_tx.send_replace(true);
    }

    #[cfg(test)]
    pub(crate) fn is_shutdown_requested(&self) -> bool {
        *self.request_tx.borrow()
    }

    #[cfg(test)]
    pub(crate) async fn shutdown_and_wait(&self) -> Result<(), String> {
        self.request_shutdown();
        let mut completion_rx = self.completion_rx.clone();
        if let Some(result) = completion_rx.borrow().clone() {
            return result;
        }
        let result = completion_rx
            .wait_for(Option::is_some)
            .await
            .map_err(|_| "LIO shutdown completion channel closed before STOPPED".to_string())?
            .clone()
            .expect("wait_for accepted only Some completion");
        result
    }
}

/// Non-clone generation owner. Exactly one root retains the supervisor join.
pub(crate) struct LioOwnerHandle {
    shutdown: LioShutdownControl,
    supervisor: Option<JoinHandle<Result<(), String>>>,
    terminal: Option<Result<(), String>>,
}

impl std::fmt::Debug for LioOwnerHandle {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("LioOwnerHandle")
            .field("shutdown", &self.shutdown)
            .field(
                "supervisor_finished",
                &self.supervisor.as_ref().is_none_or(JoinHandle::is_finished),
            )
            .finish()
    }
}

impl LioOwnerHandle {
    /// 根拥有方保留二维监督任务，关闭回执在真实子进程回收后完成。
    pub(crate) fn supervised_planar<F, Fut>(run: F) -> Self
    where
        F: FnOnce(watch::Receiver<bool>) -> Fut + Send + 'static,
        Fut: std::future::Future<Output = Result<(), String>> + Send + 'static,
    {
        let (request_tx, request_rx) = watch::channel(false);
        #[cfg(test)]
        let (completion_tx, completion_rx) = watch::channel(None);
        let supervisor = tokio::spawn(async move {
            let result = run(request_rx).await;
            #[cfg(test)]
            completion_tx.send_replace(Some(result.clone()));
            result
        });
        Self {
            shutdown: LioShutdownControl { request_tx, #[cfg(test)] completion_rx },
            supervisor: Some(supervisor), terminal: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn shutdown_control_for_tests(&self) -> LioShutdownControl {
        self.shutdown.clone()
    }

    /// Acquire every subscription acknowledgement and return without waiting
    /// for the first sensor message.
    pub(crate) async fn start(
        transport: LcmTransport,
        spec: impl Into<LioOwnerSpec>,
        runtime_generation: i64,
        ingress: Arc<dyn LioIngressSink>,
    ) -> Result<Self, String> {
        let spec = spec.into();
        let startup_ack_timeout = spec.startup_ack_timeout();
        if runtime_generation <= 0 {
            ingress.close();
            return Err("LIO_PREFLIGHT_FAILED runtime_generation must be positive".into());
        }
        if transport.shared_lcm_context().is_none() {
            ingress.close();
            return Err(
                "LIO_PREFLIGHT_FAILED owned subscriptions require real shared LCM context".into(),
            );
        }

        let mut status_sequence = 0_i64;
        if let Err(error) = publish_status(
            &transport,
            &spec,
            runtime_generation,
            &mut status_sequence,
            lio::LIO_LIFECYCLE_STARTING,
            lio::LIO_REASON_NONE,
            "",
        )
        .await
        {
            ingress.close();
            return Err(error);
        }

        let mut subscription_specs = vec![
            (
                WorkerKind::PublicOdometry,
                spec.shared().public_odometry.clone(),
            ),
            (WorkerKind::PublicCloud, spec.shared().public_cloud.clone()),
        ];
        subscription_specs.extend(spec.source_subscriptions());
        let mut subscriptions = Vec::with_capacity(subscription_specs.len());
        for (kind, (topic, message_type)) in subscription_specs {
            let subscription = match tokio::time::timeout(
                LIO_STARTUP_IO_TIMEOUT,
                transport.subscribe_owned(topic, Some(message_type)),
            )
            .await
            {
                Ok(Ok(subscription)) => subscription,
                Ok(Err(error)) => {
                    let failure = format!("{} subscribe failed: {error}", kind.label());
                    return rollback_startup(
                        &transport,
                        &spec,
                        runtime_generation,
                        &mut status_sequence,
                        ingress.as_ref(),
                        subscriptions,
                        TerminalFailure::new(lio::LIO_REASON_TRANSPORT_ERROR, failure),
                    )
                    .await;
                }
                Err(_) => {
                    let failure = format!("{} subscribe timed out", kind.label());
                    return rollback_startup(
                        &transport,
                        &spec,
                        runtime_generation,
                        &mut status_sequence,
                        ingress.as_ref(),
                        subscriptions,
                        TerminalFailure::new(lio::LIO_REASON_TRANSPORT_ERROR, failure),
                    )
                    .await;
                }
            };
            subscriptions.push((kind, subscription));
        }

        // Owned subscriptions are acknowledged before a native child can emit
        // its first generation-private odometry/cloud pair.
        let native_process = match spec.native_runtime() {
            Some(runtime) => match NativeProcessOwner::spawn(&runtime.runtime) {
                Ok(process) => Some(process),
                Err(error) => {
                    return rollback_startup(
                        &transport,
                        &spec,
                        runtime_generation,
                        &mut status_sequence,
                        ingress.as_ref(),
                        subscriptions,
                        TerminalFailure::new(lio::LIO_REASON_CHILD_CRASHED, error),
                    )
                    .await;
                }
            },
            None => None,
        };

        let (request_tx, request_rx) = watch::channel(false);
        let (worker_shutdown_tx, worker_shutdown_rx) = watch::channel(false);
        #[cfg(test)]
        let (completion_tx, completion_rx) = watch::channel(None);
        #[cfg(not(test))]
        let (completion_tx, _) = watch::channel::<Option<Result<(), String>>>(None);
        let shutdown = LioShutdownControl {
            request_tx: request_tx.clone(),
            #[cfg(test)]
            completion_rx,
        };
        let (ready_tx, ready_rx) = oneshot::channel();
        let mut supervisor = tokio::spawn(run_supervisor(
            transport,
            spec,
            runtime_generation,
            ingress,
            subscriptions,
            native_process,
            request_rx,
            worker_shutdown_tx,
            worker_shutdown_rx,
            completion_tx,
            ready_tx,
            status_sequence,
        ));

        match tokio::time::timeout(startup_ack_timeout, ready_rx).await {
            Ok(Ok(Ok(()))) => Ok(Self {
                shutdown,
                supervisor: Some(supervisor),
                terminal: None,
            }),
            Ok(Ok(Err(error))) => {
                let _ = supervisor.await;
                Err(error)
            }
            Ok(Err(_)) => match supervisor.await {
                Ok(Ok(())) => Err("LIO supervisor exited before READY acknowledgement".into()),
                Ok(Err(error)) => Err(error),
                Err(error) => Err(format!("LIO supervisor join failed before READY: {error}")),
            },
            Err(_) => {
                request_tx.send_replace(true);
                let failure = "LIO supervisor READY acknowledgement timed out".to_string();
                match tokio::time::timeout(LIO_SUPERVISOR_SHUTDOWN_TIMEOUT, &mut supervisor).await {
                    Ok(_) => Err(failure),
                    Err(_) => {
                        supervisor.abort();
                        let _ = supervisor.await;
                        Err(format!(
                            "{failure}; supervisor teardown timed out and was aborted"
                        ))
                    }
                }
            }
        }
    }

    pub(crate) async fn wait_terminal(&mut self) -> Result<(), String> {
        if let Some(result) = self.terminal.clone() {
            return result;
        }
        let result = match self
            .supervisor
            .as_mut()
            .ok_or_else(|| "LIO supervisor join already consumed".to_string())?
            .await
        {
            Ok(result) => result,
            Err(error) => Err(format!("LIO supervisor join failed: {error}")),
        };
        self.supervisor.take();
        self.terminal = Some(result.clone());
        result
    }

    fn is_finished(&self) -> bool {
        self.terminal.is_some()
            || self
                .supervisor
                .as_ref()
                .is_none_or(tokio::task::JoinHandle::is_finished)
    }

    async fn shutdown_retained(&mut self) -> Result<(), String> {
        self.shutdown.request_shutdown();
        match tokio::time::timeout(LIO_SUPERVISOR_SHUTDOWN_TIMEOUT, self.wait_terminal()).await {
            Ok(result) => result,
            Err(_) => {
                if let Some(supervisor) = self.supervisor.take() {
                    supervisor.abort();
                    let _ = supervisor.await;
                }
                let error = "LIO supervisor shutdown timed out and was aborted".to_string();
                self.terminal = Some(Err(error.clone()));
                Err(error)
            }
        }
    }

    pub(crate) async fn shutdown(mut self) -> Result<(), String> {
        self.shutdown_retained().await
    }
}

impl Drop for LioOwnerHandle {
    fn drop(&mut self) {
        self.shutdown.request_shutdown();
    }
}

/// Root lifetime token retained by `main` until all Geometry I/O is quiescent.
pub(crate) struct GeometryRuntimeOwner {
    handle: GeometryRuntimeHandle,
    task_owner: Option<GeometryRuntimeTaskOwner>,
    lio: LioOwnerHandle,
    geometry_task_settlement: Option<crate::lcm_runtime::GeometryTaskSettlement>,
    geometry_task_failure: Option<String>,
    mapping_final_owner: Option<MappingFinalOwner>,
    mapping_final_sticky: ShutdownFailures,
}

pub(crate) enum GeometryOwnedTerminal {
    Tasks(Result<(), String>),
    Lio(Result<(), String>),
}

pub(crate) enum GeometryRuntimeAfterTasksAttempt {
    Pending {
        cause: MappingPendingCause,
        sticky: ShutdownFailures,
    },
    Settled {
        terminal: Result<(), String>,
    },
}

pub(crate) struct GeometryRuntimeOwnerShutdownFailure {
    detail: String,
    pending_owner: Option<Box<GeometryRuntimeOwner>>,
}

impl std::fmt::Debug for GeometryRuntimeOwnerShutdownFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("GeometryRuntimeOwnerShutdownFailure")
            .field("detail", &self.detail)
            .field("has_pending_owner", &self.pending_owner.is_some())
            .finish()
    }
}

impl std::fmt::Display for GeometryRuntimeOwnerShutdownFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.detail)
    }
}

impl GeometryRuntimeOwnerShutdownFailure {
    fn terminal(detail: impl Into<String>) -> Self {
        Self {
            detail: detail.into(),
            pending_owner: None,
        }
    }

    fn unsettled(detail: impl Into<String>, owner: GeometryRuntimeOwner) -> Self {
        Self {
            detail: detail.into(),
            pending_owner: Some(Box::new(owner)),
        }
    }

    pub(crate) fn into_pending_owner(self) -> Option<Box<GeometryRuntimeOwner>> {
        self.pending_owner
    }
}

impl GeometryRuntimeOwner {
    pub(crate) fn new(spawned: SpawnedGeometryRuntime, lio: LioOwnerHandle) -> Self {
        Self {
            handle: spawned.handle,
            task_owner: Some(spawned.task_owner),
            lio,
            geometry_task_settlement: None,
            geometry_task_failure: None,
            mapping_final_owner: None,
            mapping_final_sticky: ShutdownFailures::default(),
        }
    }

    pub(crate) fn handle(&self) -> GeometryRuntimeHandle {
        self.handle.clone()
    }

    pub(crate) async fn wait_lio_terminal(&mut self) -> Result<(), String> {
        self.lio.wait_terminal().await
    }

    pub(crate) fn close_task_admission(&self) {
        if let Some(task_owner) = &self.task_owner {
            task_owner.close_admission();
        }
    }

    pub(crate) async fn wait_geometry_terminal(&mut self) -> Result<(), String> {
        match self.task_owner.as_mut() {
            Some(task_owner) => task_owner.wait_terminal().await,
            None => std::future::pending().await,
        }
    }

    pub(crate) async fn wait_terminal(&mut self) -> GeometryOwnedTerminal {
        match self.task_owner.as_mut() {
            Some(task_owner) => tokio::select! {
                biased;
                result = task_owner.wait_terminal() => GeometryOwnedTerminal::Tasks(result),
                result = self.lio.wait_terminal() => GeometryOwnedTerminal::Lio(result),
            },
            None => GeometryOwnedTerminal::Lio(self.lio.wait_terminal().await),
        }
    }

    pub(crate) fn geometry_completed_failure(&self) -> Option<String> {
        self.task_owner
            .as_ref()
            .and_then(GeometryRuntimeTaskOwner::completed_failure)
    }

    pub(crate) fn is_geometry_finished(&self) -> bool {
        self.task_owner
            .as_ref()
            .is_some_and(GeometryRuntimeTaskOwner::is_finished)
    }

    pub(crate) fn is_lio_finished(&self) -> bool {
        self.lio.is_finished()
    }

    #[cfg(test)]
    pub(crate) async fn settle_lio_then_inject_terminal_failure_for_shutdown_tests(
        &mut self,
        detail: impl Into<String>,
    ) {
        self.lio
            .shutdown_retained()
            .await
            .expect("test LIO supervisor must settle cleanly before terminal failure injection");
        assert!(
            self.lio.supervisor.is_none(),
            "test LIO failure injection requires the final supervisor handle to be joined"
        );
        assert!(
            self.lio
                .terminal
                .as_ref()
                .is_some_and(|terminal| terminal.is_ok()),
            "test LIO failure injection requires an already settled terminal"
        );
        self.lio.terminal = Some(Err(detail.into()));
    }

    pub(crate) async fn shutdown(self) -> Result<(), GeometryRuntimeOwnerShutdownFailure> {
        self.shutdown_with_events(None).await
    }

    async fn shutdown_with_events(
        mut self,
        events: Option<&GeometryShutdownEventEmitter>,
    ) -> Result<(), GeometryRuntimeOwnerShutdownFailure> {
        let (task_result, task_settlement) = match self.geometry_task_settlement {
            Some(settlement) => (
                self.geometry_task_failure.clone().map_or(Ok(()), Err),
                settlement,
            ),
            None => self.shutdown_geometry_tasks(events).await.into_parts(),
        };
        if !task_settlement.is_settled() {
            let detail = task_result.clone().err().unwrap_or_else(|| {
                "Geometry task producers remained unsettled during runtime shutdown".to_owned()
            });
            return Err(GeometryRuntimeOwnerShutdownFailure::unsettled(detail, self));
        }
        let runtime_attempt = self
            .shutdown_after_geometry_tasks(task_settlement, events)
            .await;
        let mut errors = task_result.err().into_iter().collect::<Vec<_>>();
        match runtime_attempt {
            GeometryRuntimeAfterTasksAttempt::Pending { cause, sticky } => {
                if let Some(detail) = sticky.detail() {
                    errors.push(detail);
                }
                errors.push(cause.to_string());
                Err(GeometryRuntimeOwnerShutdownFailure::unsettled(
                    errors.join("; "),
                    self,
                ))
            }
            GeometryRuntimeAfterTasksAttempt::Settled { terminal } => {
                if let Err(error) = terminal {
                    errors.push(error);
                }
                if errors.is_empty() {
                    Ok(())
                } else {
                    Err(GeometryRuntimeOwnerShutdownFailure::terminal(
                        errors.join("; "),
                    ))
                }
            }
        }
    }

    pub(crate) async fn shutdown_geometry_tasks(
        &mut self,
        events: Option<&GeometryShutdownEventEmitter>,
    ) -> GeometryTaskShutdownOutcome {
        self.close_task_admission();
        let outcome = match self.task_owner.as_mut() {
            Some(task_owner) => task_owner.shutdown().await,
            None => GeometryTaskShutdownOutcome::unsettled(
                "Geometry runtime root task owner is missing",
            ),
        };
        if outcome.producers_settled() {
            let (result, settlement) = outcome.clone().into_parts();
            self.geometry_task_settlement = Some(settlement);
            self.geometry_task_failure = result.err();
            self.task_owner.take();
            if let Some(events) = events {
                events.emit_completed(GeometryShutdownStage::GeometryTasksSettled);
            }
        }
        outcome
    }

    pub(crate) async fn shutdown_after_geometry_tasks(
        &mut self,
        geometry_task_settlement: crate::lcm_runtime::GeometryTaskSettlement,
        events: Option<&GeometryShutdownEventEmitter>,
    ) -> GeometryRuntimeAfterTasksAttempt {
        if !geometry_task_settlement.is_settled() {
            return GeometryRuntimeAfterTasksAttempt::Pending {
                cause: MappingPendingCause::RetirementUnproven(
                    "Geometry task producers remained unsettled; downstream teardown was withheld"
                        .to_owned(),
                ),
                sticky: ShutdownFailures::default(),
            };
        }
        let mapping_owner = self.mapping_final_owner.take().unwrap_or_else(|| {
            MappingFinalOwner::waiting_for_lifecycle_lock(
                self.handle.mapping_adapter.clone(),
                Arc::clone(&self.handle.mapping_lifecycle_lock),
            )
        });
        let mapping_attempt = mapping_owner
            .attempt(std::mem::take(&mut self.mapping_final_sticky))
            .await;
        let mapping_terminal = match mapping_attempt {
            MappingFinalAttempt::Pending {
                owner,
                cause,
                sticky,
            } => {
                self.mapping_final_owner = Some(owner);
                self.mapping_final_sticky = sticky.clone();
                return GeometryRuntimeAfterTasksAttempt::Pending { cause, sticky };
            }
            MappingFinalAttempt::Settled {
                capability: _capability,
                terminal,
            } => {
                if let Some(events) = events {
                    events.emit_completed(GeometryShutdownStage::MappingRetired);
                }
                terminal.map_err(|error| error.to_string())
            }
        };

        let lio_result = self.lio.shutdown_retained().await;
        if let Some(events) = events {
            events.emit_completed(GeometryShutdownStage::LioJoined);
        }

        let cleanup_result = cleanup_product_owned_runtime_files(&self.handle, events);

        let mut errors = Vec::new();
        if let Err(error) = mapping_terminal {
            errors.push(error);
        }
        if let Err(error) = lio_result {
            errors.push(format!("failed to shut down LIO owner: {error}"));
        }
        if let Err(error) = cleanup_result {
            errors.push(error);
        }
        if errors.is_empty() {
            GeometryRuntimeAfterTasksAttempt::Settled { terminal: Ok(()) }
        } else {
            GeometryRuntimeAfterTasksAttempt::Settled {
                terminal: Err(errors.join("; ")),
            }
        }
    }
}

fn cleanup_product_owned_runtime_files(
    handle: &GeometryRuntimeHandle,
    events: Option<&GeometryShutdownEventEmitter>,
) -> Result<(), String> {
    let occupancy_result = match handle.occupancy_shm.as_ref() {
        Some(writer) => writer
            .lock()
            .cleanup_product_owned_file()
            .map_err(|error| format!("product-owned occupancy mmap cleanup failed: {error}")),
        None => Ok(()),
    };
    let cloud_result = {
        let mut cloud_writer = handle.cloud_map_writer.lock();
        match cloud_writer.as_mut() {
            Some(writer) => writer
                .cleanup_product_owned_file()
                .map_err(|error| format!("product-owned cloud mmap cleanup failed: {error}")),
            None => Ok(()),
        }
    };
    let result = match (occupancy_result, cloud_result) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
        (Err(occupancy), Err(cloud)) => Err(format!("{occupancy}; {cloud}")),
    };
    if result.is_ok() {
        if let Some(events) = events {
            events.emit_completed(GeometryShutdownStage::ResourceCleanupComplete);
        }
    }
    result
}

#[derive(Clone, Copy, Debug)]
enum WorkerKind {
    PublicOdometry,
    PublicCloud,
    SourceOdometry,
    SourceCloud,
}

impl WorkerKind {
    const fn label(self) -> &'static str {
        match self {
            Self::PublicOdometry => "public LIO odometry",
            Self::PublicCloud => "public LIO cloud",
            Self::SourceOdometry => "source LIO odometry",
            Self::SourceCloud => "source LIO cloud",
        }
    }

    async fn accept(
        self,
        ingress: &dyn LioIngressSink,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match self {
            Self::PublicOdometry => ingress.accept_public_odometry(frame, shutdown_rx).await,
            Self::PublicCloud => ingress.accept_public_cloud(frame, shutdown_rx).await,
            Self::SourceOdometry => ingress.accept_legacy_odometry(frame, shutdown_rx).await,
            Self::SourceCloud => ingress.accept_legacy_cloud(frame, shutdown_rx).await,
        }
    }
}

#[derive(Debug)]
struct TerminalFailure {
    reason_code: i32,
    message: String,
}

impl TerminalFailure {
    fn new(reason_code: i32, message: impl Into<String>) -> Self {
        Self {
            reason_code,
            message: message.into(),
        }
    }

    fn push_teardown(&mut self, error: impl AsRef<str>) {
        self.message.push_str("; teardown: ");
        self.message.push_str(error.as_ref());
    }
}

fn classify_teardown_failure(
    terminal_failure: Option<TerminalFailure>,
    stopping_error: Option<String>,
    join_error: Option<String>,
    native_shutdown_error: Option<String>,
) -> Option<TerminalFailure> {
    let (mut failure, native_teardown_error) = match (terminal_failure, native_shutdown_error) {
        (Some(failure), native_shutdown_error) => (failure, native_shutdown_error),
        (None, Some(native_shutdown_error)) => (
            TerminalFailure::new(lio::LIO_REASON_CHILD_CRASHED, native_shutdown_error),
            None,
        ),
        (None, None) if stopping_error.is_none() && join_error.is_none() => return None,
        (None, None) => (
            TerminalFailure::new(lio::LIO_REASON_TRANSPORT_ERROR, "LIO owner teardown failed"),
            None,
        ),
    };
    for error in [stopping_error, join_error, native_teardown_error]
        .into_iter()
        .flatten()
    {
        failure.push_teardown(error);
    }
    Some(failure)
}

fn worker_terminal_error(
    worker: Option<Result<Result<(), String>, tokio::task::JoinError>>,
) -> String {
    match worker {
        Some(Ok(Ok(()))) => "LIO subscription worker exited before shutdown".to_string(),
        Some(Ok(Err(error))) => error,
        Some(Err(error)) => format!("LIO subscription worker join failed: {error}"),
        None => "all LIO subscription workers exited before shutdown".to_string(),
    }
}

fn child_exit_failure(status: ExitStatus) -> TerminalFailure {
    if let Some(signal) = status.signal() {
        TerminalFailure::new(
            lio::LIO_REASON_CHILD_CRASHED,
            format!("native LIO child exited from signal {signal}"),
        )
    } else if status.success() {
        TerminalFailure::new(
            lio::LIO_REASON_CHILD_EXITED,
            "native LIO child exited unexpectedly with status 0",
        )
    } else {
        TerminalFailure::new(
            lio::LIO_REASON_CHILD_CRASHED,
            format!(
                "native LIO child exited unexpectedly with status {}",
                status.code().unwrap_or(-1)
            ),
        )
    }
}

async fn wait_native_unexpected_exit(
    process: &mut Option<NativeProcessOwner>,
) -> Result<ExitStatus, String> {
    match process {
        Some(process) => process.wait_unexpected_exit().await,
        None => std::future::pending().await,
    }
}

async fn shutdown_native_process(process: &mut Option<NativeProcessOwner>) -> Result<(), String> {
    let Some(process) = process.as_mut() else {
        return Ok(());
    };
    process
        .shutdown(NATIVE_LIO_TERM_GRACE, NATIVE_LIO_FORCE_REAP_TIMEOUT)
        .await
        .map(|_| ())
}

fn poll_owner_terminal(
    workers: &mut JoinSet<Result<(), String>>,
    process: &mut Option<NativeProcessOwner>,
) -> Option<TerminalFailure> {
    if let Some(worker) = workers.try_join_next() {
        return Some(TerminalFailure::new(
            lio::LIO_REASON_TRANSPORT_ERROR,
            worker_terminal_error(Some(worker)),
        ));
    }
    let process = process.as_mut()?;
    match process.poll_unexpected_exit() {
        Ok(Some(status)) => Some(child_exit_failure(status)),
        Ok(None) => None,
        Err(error) => Some(TerminalFailure::new(lio::LIO_REASON_CHILD_CRASHED, error)),
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_supervisor(
    transport: LcmTransport,
    spec: LioOwnerSpec,
    runtime_generation: i64,
    ingress: Arc<dyn LioIngressSink>,
    subscriptions: Vec<(WorkerKind, OwnedLcmSubscription)>,
    mut native_process: Option<NativeProcessOwner>,
    mut request_rx: watch::Receiver<bool>,
    worker_shutdown_tx: watch::Sender<bool>,
    worker_shutdown_rx: watch::Receiver<bool>,
    completion_tx: watch::Sender<Option<Result<(), String>>>,
    ready_tx: oneshot::Sender<Result<(), String>>,
    mut status_sequence: i64,
) -> Result<(), String> {
    let mut workers = JoinSet::new();
    for (kind, subscription) in subscriptions {
        workers.spawn(run_subscription_worker(
            kind,
            subscription,
            worker_shutdown_rx.clone(),
            Arc::clone(&ingress),
        ));
    }
    let mut startup_shutdown_rx = worker_shutdown_rx.clone();

    let mut startup_failure = tokio::select! {
        biased;
        worker = workers.join_next() => Some(TerminalFailure::new(
            lio::LIO_REASON_TRANSPORT_ERROR,
            worker_terminal_error(worker),
        )),
        child = wait_native_unexpected_exit(&mut native_process) => Some(match child {
            Ok(status) => child_exit_failure(status),
            Err(error) => TerminalFailure::new(lio::LIO_REASON_CHILD_CRASHED, error),
        }),
        changed = request_rx.changed() => Some(TerminalFailure::new(
            lio::LIO_REASON_TRANSPORT_ERROR,
            if changed.is_err() {
                "LIO startup owner was cancelled before READY".to_string()
            } else {
                "LIO startup shutdown was requested before READY".to_string()
            },
        )),
        ready = tokio::time::timeout(
            spec.first_pair_timeout(),
            ingress.wait_startup_ready(&mut startup_shutdown_rx),
        ) => match ready {
            Ok(Ok(())) => None,
            Ok(Err(error)) => Some(TerminalFailure::new(
                lio::LIO_REASON_TRANSPORT_ERROR,
                error,
            )),
            Err(_) => Some(TerminalFailure::new(
                lio::LIO_REASON_STARTUP_TIMEOUT,
                format!(
                    "native LIO first valid source pair timed out after {} ms",
                    spec.first_pair_timeout().as_millis()
                ),
            )),
        },
    };
    if startup_failure.is_none() {
        startup_failure = poll_owner_terminal(&mut workers, &mut native_process);
    }

    if let Some(mut failure) = startup_failure {
        ingress.close();
        worker_shutdown_tx.send_replace(true);
        let (worker_result, native_result) = tokio::join!(
            join_workers_bounded(&mut workers),
            shutdown_native_process(&mut native_process),
        );
        if let Err(error) = worker_result {
            failure.push_teardown(error);
        }
        if let Err(error) = native_result {
            failure.push_teardown(error);
        }
        let _ = publish_status(
            &transport,
            &spec,
            runtime_generation,
            &mut status_sequence,
            lio::LIO_LIFECYCLE_FAILED,
            failure.reason_code,
            &failure.message,
        )
        .await;
        let _ = ready_tx.send(Err(failure.message.clone()));
        completion_tx.send_replace(Some(Err(failure.message.clone())));
        return Err(failure.message);
    }

    if let Err(error) = publish_status(
        &transport,
        &spec,
        runtime_generation,
        &mut status_sequence,
        lio::LIO_LIFECYCLE_READY,
        lio::LIO_REASON_NONE,
        "",
    )
    .await
    {
        let mut failure = TerminalFailure::new(lio::LIO_REASON_TRANSPORT_ERROR, error);
        ingress.close();
        worker_shutdown_tx.send_replace(true);
        let (worker_result, native_result) = tokio::join!(
            join_workers_bounded(&mut workers),
            shutdown_native_process(&mut native_process),
        );
        if let Err(error) = worker_result {
            failure.push_teardown(error);
        }
        if let Err(error) = native_result {
            failure.push_teardown(error);
        }
        let _ = publish_status(
            &transport,
            &spec,
            runtime_generation,
            &mut status_sequence,
            lio::LIO_LIFECYCLE_FAILED,
            failure.reason_code,
            &failure.message,
        )
        .await;
        let _ = ready_tx.send(Err(failure.message.clone()));
        completion_tx.send_replace(Some(Err(failure.message.clone())));
        return Err(failure.message);
    }
    let _ = ready_tx.send(Ok(()));
    info!(
        runtime_generation,
        backend = spec.backend_code(),
        "LIO owner READY after subscription ACKs and ingress admission"
    );

    let terminal_failure = loop {
        if let Some(failure) = poll_owner_terminal(&mut workers, &mut native_process) {
            break Some(failure);
        }
        if *request_rx.borrow() {
            break None;
        }
        tokio::select! {
            biased;
            worker = workers.join_next() => {
                break Some(TerminalFailure::new(
                    lio::LIO_REASON_TRANSPORT_ERROR,
                    worker_terminal_error(worker),
                ));
            }
            child = wait_native_unexpected_exit(&mut native_process) => {
                break Some(match child {
                    Ok(status) => child_exit_failure(status),
                    Err(error) => TerminalFailure::new(lio::LIO_REASON_CHILD_CRASHED, error),
                });
            }
            changed = request_rx.changed() => {
                if changed.is_err() || *request_rx.borrow() {
                    break None;
                }
            }
        }
    };

    ingress.close();
    worker_shutdown_tx.send_replace(true);
    let stopping_reason = match terminal_failure.as_ref().map(|failure| failure.reason_code) {
        None => "shutdown requested",
        Some(lio::LIO_REASON_CHILD_EXITED | lio::LIO_REASON_CHILD_CRASHED) => {
            "native child failure; stopping"
        }
        Some(_) => "subscription worker failure; stopping",
    };
    let stopping_error = publish_status(
        &transport,
        &spec,
        runtime_generation,
        &mut status_sequence,
        lio::LIO_LIFECYCLE_STOPPING,
        lio::LIO_REASON_SHUTDOWN_REQUESTED,
        stopping_reason,
    )
    .await
    .err();

    let (worker_result, native_shutdown_result) = tokio::join!(
        join_workers_bounded(&mut workers),
        shutdown_native_process(&mut native_process),
    );
    let join_error = worker_result.err();
    let teardown_failure = classify_teardown_failure(
        terminal_failure,
        stopping_error,
        join_error,
        native_shutdown_result.err(),
    );
    let result = match teardown_failure {
        None => {
            publish_status(
                &transport,
                &spec,
                runtime_generation,
                &mut status_sequence,
                lio::LIO_LIFECYCLE_STOPPED,
                lio::LIO_REASON_SHUTDOWN_REQUESTED,
                "stopped",
            )
            .await
        }
        Some(failure) => {
            let _ = publish_status(
                &transport,
                &spec,
                runtime_generation,
                &mut status_sequence,
                lio::LIO_LIFECYCLE_FAILED,
                failure.reason_code,
                &failure.message,
            )
            .await;
            Err(failure.message)
        }
    };
    completion_tx.send_replace(Some(result.clone()));
    result
}

async fn run_subscription_worker(
    kind: WorkerKind,
    mut subscription: OwnedLcmSubscription,
    mut shutdown_rx: watch::Receiver<bool>,
    ingress: Arc<dyn LioIngressSink>,
) -> Result<(), String> {
    loop {
        if *shutdown_rx.borrow() {
            return unsubscribe_bounded(kind, subscription).await;
        }
        tokio::select! {
            biased;
            changed = shutdown_rx.changed() => {
                if changed.is_err() || *shutdown_rx.borrow() {
                    return unsubscribe_bounded(kind, subscription).await;
                }
            }
            frame = subscription.recv() => {
                match frame {
                    Ok(frame) => {
                        if let Err(error) = kind
                            .accept(ingress.as_ref(), frame, &mut shutdown_rx)
                            .await
                        {
                            warn!(source = kind.label(), %error, "LIO ingress rejected source frame");
                        }
                    }
                    Err(error) => {
                        let receive_error = format!("{} receive failed: {error}", kind.label());
                        return match unsubscribe_bounded(kind, subscription).await {
                            Ok(()) => Err(receive_error),
                            Err(unsubscribe_error) => Err(format!(
                                "{receive_error}; {unsubscribe_error}"
                            )),
                        };
                    }
                }
            }
        }
    }
}

async fn join_workers(workers: &mut JoinSet<Result<(), String>>) -> Result<(), String> {
    let mut errors = Vec::new();
    while let Some(result) = workers.join_next().await {
        match result {
            Ok(Ok(())) => {}
            Ok(Err(error)) => errors.push(error),
            Err(error) => errors.push(format!("worker join failed: {error}")),
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

async fn join_workers_bounded(workers: &mut JoinSet<Result<(), String>>) -> Result<(), String> {
    match tokio::time::timeout(LIO_WORKER_JOIN_TIMEOUT, join_workers(workers)).await {
        Ok(result) => result,
        Err(_) => {
            workers.abort_all();
            Err("LIO worker shutdown timed out; remaining workers were aborted".into())
        }
    }
}

async fn unsubscribe_bounded(
    kind: WorkerKind,
    subscription: OwnedLcmSubscription,
) -> Result<(), String> {
    match tokio::time::timeout(LIO_CONTROL_IO_TIMEOUT, subscription.unsubscribe()).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(format!("{} unsubscribe failed: {error}", kind.label())),
        Err(_) => Err(format!("{} unsubscribe timed out", kind.label())),
    }
}

async fn unsubscribe_all(
    subscriptions: Vec<(WorkerKind, OwnedLcmSubscription)>,
) -> Result<(), String> {
    let mut errors = Vec::new();
    for (kind, subscription) in subscriptions {
        if let Err(error) = unsubscribe_bounded(kind, subscription).await {
            errors.push(error);
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

#[allow(clippy::too_many_arguments)]
async fn rollback_startup<T>(
    transport: &LcmTransport,
    spec: &LioOwnerSpec,
    runtime_generation: i64,
    status_sequence: &mut i64,
    ingress: &dyn LioIngressSink,
    subscriptions: Vec<(WorkerKind, OwnedLcmSubscription)>,
    mut failure: TerminalFailure,
) -> Result<T, String> {
    ingress.close();
    if let Err(error) = unsubscribe_all(subscriptions).await {
        failure.push_teardown(error);
    }
    let _ = publish_status(
        transport,
        spec,
        runtime_generation,
        status_sequence,
        lio::LIO_LIFECYCLE_FAILED,
        failure.reason_code,
        &failure.message,
    )
    .await;
    Err(failure.message)
}

#[allow(clippy::too_many_arguments)]
async fn publish_status(
    transport: &LcmTransport,
    spec: &LioOwnerSpec,
    runtime_generation: i64,
    status_sequence: &mut i64,
    lifecycle_state: i32,
    reason_code: i32,
    reason: &str,
) -> Result<(), String> {
    let shared = spec.shared();
    *status_sequence = status_sequence
        .checked_add(1)
        .ok_or_else(|| "LIO status sequence overflow".to_string())?;
    let message = lio::LioStatus {
        schema_version: lio::LIO_SCHEMA_VERSION_V1,
        backend: spec.backend_code(),
        runtime_generation,
        status_sequence: *status_sequence,
        status_timestamp_ns: wall_clock_ns(),
        last_odometry_source_sequence: 0,
        last_odometry_source_timestamp_ns: 0,
        last_cloud_source_sequence: 0,
        last_cloud_source_timestamp_ns: 0,
        sensor_frame_id: shared.sensor_frame.clone(),
        body_frame_id: shared.body_frame.clone(),
        odom_frame_id: shared.odom_frame.clone(),
        map_frame_id: shared.map_frame.clone(),
        lifecycle_state,
        reason_code,
        reason: truncate_utf8(reason, 1_024),
    };
    let encoded = lio::encode_lio_status(&message)
        .map_err(|error| format!("LIO status encode failed: {error}"))?;
    let body = encoded
        .strip_prefix(&lio::LIO_STATUS_FINGERPRINT_V1)
        .ok_or_else(|| "LIO status codec returned an unlocked fingerprint".to_string())?;
    let frame = RawLcmFrameBuilder::new(shared.status.0.clone(), shared.status.1.clone())
        .seqno(u32::try_from(*status_sequence).unwrap_or(u32::MAX))
        .payload(body.to_vec())
        .build();
    match tokio::time::timeout(LIO_CONTROL_IO_TIMEOUT, transport.publish(frame)).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(format!("LIO status publish failed: {error}")),
        Err(_) => Err("LIO status publish timed out".into()),
    }
}

fn wall_clock_ns() -> i64 {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    i64::try_from(nanos).unwrap_or(i64::MAX)
}

fn truncate_utf8(value: &str, max_bytes: usize) -> String {
    if value.len() <= max_bytes {
        return value.to_owned();
    }
    let mut end = max_bytes;
    while !value.is_char_boundary(end) {
        end -= 1;
    }
    value[..end].to_owned()
}

fn strict_env_or_default(preferred: &str, fallback: &str, default: &str) -> Result<String, String> {
    for key in [preferred, fallback] {
        match std::env::var(key) {
            Ok(value) => {
                if value.is_empty() {
                    return Err(format!("LIO_PREFLIGHT_FAILED env={key} value_is_empty"));
                }
                if value != value.trim() {
                    return Err(format!(
                        "LIO_PREFLIGHT_FAILED env={key} value_has_surrounding_whitespace"
                    ));
                }
                return Ok(value);
            }
            Err(std::env::VarError::NotPresent) => {}
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(format!(
                    "LIO_PREFLIGHT_FAILED env={key} value_is_not_unicode"
                ));
            }
        }
    }
    Ok(default.to_owned())
}

fn strict_typed_binding(
    label: &str,
    pattern: &str,
    expected_type: &str,
    expected_fingerprint: [u8; 8],
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = LcmTopic::parse_subscription_pattern(pattern)
        .map_err(|error| format!("LIO_PREFLIGHT_FAILED {label} topic: {error}"))?;
    let suffix = suffix
        .ok_or_else(|| format!("LIO_PREFLIGHT_FAILED {label} missing typed message suffix"))?;
    let message_type = LcmMessageType::new(&suffix)
        .map_err(|error| format!("LIO_PREFLIGHT_FAILED {label} type: {error}"))?;
    let binding = (topic, message_type);
    validate_binding(label, &binding, expected_type, expected_fingerprint)?;
    Ok(binding)
}

fn validate_binding(
    label: &str,
    binding: &(LcmTopic, LcmMessageType),
    expected_type: &str,
    expected_fingerprint: [u8; 8],
) -> Result<(), String> {
    if binding.1.fqn() != expected_type {
        return Err(format!(
            "LIO_PREFLIGHT_FAILED {label} message_type={} expected={expected_type}",
            binding.1.fqn()
        ));
    }
    let actual_fingerprint = binding.1.fingerprint();
    if actual_fingerprint != expected_fingerprint || actual_fingerprint == [0; 8] {
        return Err(format!(
            "LIO_PREFLIGHT_FAILED {label} unlocked fingerprint actual={:02x?} expected={:02x?}",
            actual_fingerprint, expected_fingerprint
        ));
    }
    Ok(())
}

fn validate_frame(role: &str, value: &str) -> Result<String, String> {
    let valid = value == value.trim()
        && !value.is_empty()
        && value.len() <= 255
        && value.split('/').all(|token| {
            let mut chars = token.chars();
            chars
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && chars.all(|character| character.is_ascii_alphanumeric() || character == '_')
        });
    if !valid {
        return Err(format!(
            "LIO_PREFLIGHT_FAILED role={role} invalid_frame={value:?}"
        ));
    }
    Ok(value.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lio_ingress::{CommonLioIngress, CorrelatedOdometry, DeviceLioIngressSpec};
    use crate::native_lio_owner::{BodyFromSensorTransform, NativePrivateChannels};
    use crate::pointcloud::CloudMapShmWriter;
    use nuwax_lio_backends::native::{NativeBackend, NativeRuntimeSpec};
    use std::collections::BTreeMap;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::AtomicI64;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::time::Duration;
    use tokio_stream::StreamExt;

    #[test]
    fn shutdown_event_contract_has_stable_names_fields_and_order() {
        let stages = [
            GeometryShutdownStage::GeometryTasksSettled,
            GeometryShutdownStage::CorrectionSettled,
            GeometryShutdownStage::MappingRetired,
            GeometryShutdownStage::LioJoined,
            GeometryShutdownStage::ResourceCleanupComplete,
        ];
        assert_eq!(
            stages[0].event_name(),
            "nuwax.geometry.runtime.shutdown.geometry_tasks_settled.v1"
        );
        assert_eq!(
            stages[1].event_name(),
            "nuwax.geometry.runtime.shutdown.correction_settled.v1"
        );
        assert_eq!(
            stages[2].event_name(),
            "nuwax.geometry.runtime.shutdown.mapping_retired.v1"
        );
        assert_eq!(
            stages[3].event_name(),
            "nuwax.geometry.runtime.shutdown.lio_joined.v1"
        );
        assert_eq!(
            stages[4].event_name(),
            "nuwax.geometry.runtime.shutdown.resource_cleanup_complete.v1"
        );
        assert_eq!(GEOMETRY_SHUTDOWN_EVENT_SCHEMA_VERSION, 2);
        assert_eq!(stages.map(GeometryShutdownStage::ordinal), [1, 2, 3, 4, 5]);

        let (emitter, recorder) = GeometryShutdownEventEmitter::recording(77);
        for stage in stages {
            emitter.emit_completed(stage);
        }
        assert_eq!(recorder.stages(), stages);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn cleanup_failure_preserves_replacement_and_emits_no_false_completion() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-LCM Geometry handle");
        let path = std::env::temp_dir().join(format!(
            "nuwax-cleanup-failure-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&path).expect("product cloud writer"));
        std::fs::remove_file(&path).expect("remove owned cloud name");
        std::fs::write(&path, b"foreign replacement").expect("install replacement");

        let (events, recorder) = GeometryShutdownEventEmitter::recording(78);
        let error = cleanup_product_owned_runtime_files(&handle, Some(&events))
            .expect_err("replacement identity mismatch must fail cleanup");
        assert!(error.contains("product-owned cloud mmap cleanup failed"));
        assert!(!recorder
            .stages()
            .contains(&GeometryShutdownStage::ResourceCleanupComplete));
        assert_eq!(
            std::fs::read(&path).expect("replacement remains"),
            b"foreign replacement"
        );
        std::fs::remove_file(path).expect("remove replacement");
    }

    #[derive(Default)]
    struct TestIngressSink {
        accepted: AtomicUsize,
        block_public_odometry: AtomicBool,
        drain_public_odometry: AtomicBool,
        closed: AtomicBool,
        public_odometry_entered: tokio::sync::Notify,
        public_odometry_release: tokio::sync::Notify,
        public_odometry_completed: AtomicBool,
    }

    #[async_trait]
    impl LioIngressSink for TestIngressSink {
        fn close(&self) {
            self.closed.store(true, Ordering::Release);
        }

        async fn accept_public_odometry(
            &self,
            _frame: RawLcmFrame,
            shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.accepted.fetch_add(1, Ordering::AcqRel);
            if self.block_public_odometry.load(Ordering::Acquire) {
                self.public_odometry_entered.notify_one();
                if !*shutdown_rx.borrow() {
                    let _ = shutdown_rx.wait_for(|shutdown| *shutdown).await;
                }
                return Ok(());
            }
            if self.drain_public_odometry.load(Ordering::Acquire) {
                self.public_odometry_entered.notify_one();
                self.public_odometry_release.notified().await;
                self.public_odometry_completed
                    .store(true, Ordering::Release);
            }
            Ok(())
        }

        async fn accept_public_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.accepted.fetch_add(1, Ordering::AcqRel);
            Ok(())
        }

        async fn accept_legacy_odometry(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.accepted.fetch_add(1, Ordering::AcqRel);
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.accepted.fetch_add(1, Ordering::AcqRel);
            Ok(())
        }
    }

    #[derive(Clone, Copy)]
    enum NativeStartupBehavior {
        Immediate,
        Pending,
        SourcePair,
    }

    struct NativeSupervisorTestSink {
        startup_behavior: NativeStartupBehavior,
        startup_entered: tokio::sync::Notify,
        source_changed: tokio::sync::Notify,
        source_odometry_accepted: AtomicUsize,
        source_cloud_accepted: AtomicUsize,
        closed: AtomicBool,
    }

    impl NativeSupervisorTestSink {
        fn ready() -> Self {
            Self::with_behavior(NativeStartupBehavior::Immediate)
        }

        fn pending() -> Self {
            Self::with_behavior(NativeStartupBehavior::Pending)
        }

        fn ready_after_source_pair() -> Self {
            Self::with_behavior(NativeStartupBehavior::SourcePair)
        }

        fn with_behavior(startup_behavior: NativeStartupBehavior) -> Self {
            Self {
                startup_behavior,
                startup_entered: tokio::sync::Notify::new(),
                source_changed: tokio::sync::Notify::new(),
                source_odometry_accepted: AtomicUsize::new(0),
                source_cloud_accepted: AtomicUsize::new(0),
                closed: AtomicBool::new(false),
            }
        }

        fn has_source_pair(&self) -> bool {
            self.source_odometry_accepted.load(Ordering::Acquire) > 0
                && self.source_cloud_accepted.load(Ordering::Acquire) > 0
        }
    }

    #[async_trait]
    impl LioIngressSink for NativeSupervisorTestSink {
        fn close(&self) {
            self.closed.store(true, Ordering::Release);
        }

        async fn wait_startup_ready(
            &self,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.startup_entered.notify_one();
            match self.startup_behavior {
                NativeStartupBehavior::Immediate => Ok(()),
                NativeStartupBehavior::Pending => std::future::pending().await,
                NativeStartupBehavior::SourcePair => loop {
                    let changed = self.source_changed.notified();
                    if self.has_source_pair() {
                        return Ok(());
                    }
                    changed.await;
                },
            }
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
            self.source_odometry_accepted.fetch_add(1, Ordering::AcqRel);
            self.source_changed.notify_waiters();
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.source_cloud_accepted.fetch_add(1, Ordering::AcqRel);
            self.source_changed.notify_waiters();
            Ok(())
        }
    }

    fn next_native_test_generation() -> i64 {
        static NEXT_GENERATION: AtomicI64 = AtomicI64::new(10_000);
        NEXT_GENERATION.fetch_add(2, Ordering::Relaxed)
    }

    fn native_owner_spec(
        script: &str,
        first_pair_timeout: Duration,
        runtime_env: BTreeMap<String, String>,
    ) -> (LioOwnerSpec, i64) {
        let generation = next_native_test_generation();
        let shared = spec_with(
            &format!("/geometry/test/lio/{generation}/odometry#{LIO_ODOMETRY_TYPE}"),
            &format!("/geometry/test/lio/{generation}/cloud#{LIO_CLOUD_TYPE}"),
            &format!("/geometry/test/lio/{generation}/status#{LIO_STATUS_TYPE}"),
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("native shared owner spec");
        let channel_root = format!("/geometry/test/lio/{generation}/private");
        let runtime = BoundNativeLioRuntime {
            backend_code: lio::LIO_BACKEND_FASTLIO2,
            channels: NativePrivateChannels {
                source_odometry: binding(
                    &format!("{channel_root}/odometry"),
                    LEGACY_ODOMETRY_TYPE,
                ),
                source_cloud: binding(&format!("{channel_root}/cloud"), LEGACY_CLOUD_TYPE),
                input_cloud: binding(
                    &format!("{channel_root}/input_cloud"),
                    LEGACY_CLOUD_TYPE,
                ),
                input_imu: binding(
                    &format!("{channel_root}/input_imu"),
                    "sensor_msgs.Imu",
                ),
            },
            runtime: Box::new(NativeRuntimeSpec {
                backend: NativeBackend::FastLio2,
                cwd: PathBuf::from("/"),
                executable: PathBuf::from("/bin/sh"),
                args: vec!["-c".to_owned(), script.to_owned()],
                env: runtime_env,
            }),
            body_from_sensor: BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("identity body-from-sensor transform"),
            first_pair_timeout,
            pair_max_skew: Duration::from_millis(1),
        };
        (LioOwnerSpec::native(shared, runtime), generation)
    }

    fn native_child_probe() -> (PathBuf, PathBuf, BTreeMap<String, String>) {
        let generation = next_native_test_generation();
        let root = std::env::temp_dir();
        let pid_path = root.join(format!(
            "nuwax-native-supervisor-{}-{generation}.pid",
            std::process::id()
        ));
        let term_path = root.join(format!(
            "nuwax-native-supervisor-{}-{generation}.term",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&pid_path);
        let _ = std::fs::remove_file(&term_path);
        let mut env = BTreeMap::new();
        env.insert(
            "NUWAX_TEST_NATIVE_PID_FILE".to_owned(),
            pid_path.to_string_lossy().into_owned(),
        );
        env.insert(
            "NUWAX_TEST_NATIVE_TERM_FILE".to_owned(),
            term_path.to_string_lossy().into_owned(),
        );
        (pid_path, term_path, env)
    }

    async fn wait_for_file(path: &Path) -> String {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        loop {
            if let Ok(contents) = std::fs::read_to_string(path) {
                return contents;
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "timed out waiting for {}",
                path.display()
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }

    async fn wait_until_pid_is_reaped(pid: u32) {
        let pid = nix::unistd::Pid::from_raw(i32::try_from(pid).expect("test child pid range"));
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        loop {
            match nix::sys::signal::kill(pid, None) {
                Err(nix::errno::Errno::ESRCH) => return,
                Ok(()) | Err(nix::errno::Errno::EPERM) => {}
                Err(error) => panic!("unexpected child liveness probe failure: {error}"),
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "native child pid={pid} was not reaped"
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }

    fn binding(channel: &str, message_type: &str) -> (LcmTopic, LcmMessageType) {
        (
            LcmTopic::new(channel).unwrap(),
            LcmMessageType::new(message_type).unwrap(),
        )
    }

    fn spec_with(
        odometry: &str,
        cloud: &str,
        status: &str,
        sensor_frame: &str,
    ) -> Result<DeviceLioOwnerSpec, String> {
        DeviceLioOwnerSpec::from_values(
            odometry,
            cloud,
            status,
            binding("/odom", LEGACY_ODOMETRY_TYPE),
            binding("/go2/lidar/points", LEGACY_CLOUD_TYPE),
            [
                LcmTopic::new("/odom").unwrap(),
                LcmTopic::new("/go2/lidar/points").unwrap(),
                LcmTopic::new("/tf").unwrap(),
            ],
            sensor_frame,
            "base_link",
            "odom",
            "map",
        )
    }

    fn legacy_frame(
        binding: &(LcmTopic, LcmMessageType),
        sequence: u32,
        payload: Vec<u8>,
    ) -> RawLcmFrame {
        RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
            .seqno(sequence)
            .payload(payload)
            .build()
    }

    fn source_header(timestamp_ns: i64, frame_id: &str) -> schema::Header {
        schema::Header {
            seq: 0,
            stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000).expect("test timestamp seconds"),
            stamp_nsec: i32::try_from(timestamp_ns % 1_000_000_000)
                .expect("test timestamp nanoseconds"),
            frame_id: frame_id.to_string(),
        }
    }

    fn legacy_odometry(timestamp_ns: i64) -> schema::Odometry {
        schema::Odometry {
            header: source_header(timestamp_ns, "odom"),
            child_frame_id: "base_link".to_string(),
            pose: schema::Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        }
    }

    fn legacy_cloud(timestamp_ns: i64, points: &[[f32; 3]]) -> schema::PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * 12);
        for point in points {
            for coordinate in point {
                data.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        schema::PointCloud2 {
            header: source_header(timestamp_ns, "odom"),
            height: 1,
            width: i32::try_from(points.len()).expect("test cloud width"),
            fields: [("x", 0), ("y", 4), ("z", 8)]
                .into_iter()
                .map(|(name, offset)| schema::PointField {
                    name: name.to_string(),
                    offset,
                    datatype: 7,
                    count: 1,
                })
                .collect(),
            is_bigendian: false,
            point_step: 12,
            row_step: i32::try_from(points.len() * 12).expect("test cloud row step"),
            data,
            is_dense: true,
        }
    }

    async fn next_real_frame(stream: &mut nuwax_transport::LcmFrameStream) -> RawLcmFrame {
        tokio::time::timeout(Duration::from_secs(2), stream.next())
            .await
            .expect("real LCM frame timeout")
            .expect("real LCM frame stream closed")
            .expect("real LCM frame transport")
    }

    async fn next_lio_status(stream: &mut nuwax_transport::LcmFrameStream) -> lio::LioStatus {
        let frame = next_real_frame(stream).await;
        let mut encoded = lio::LIO_STATUS_FINGERPRINT_V1.to_vec();
        encoded.extend_from_slice(&frame.payload);
        lio::decode_lio_status(&encoded).expect("versioned LIO status")
    }

    #[test]
    fn defaults_are_strict_locked_and_isolated() {
        let spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap();
        assert_eq!(
            spec.public_odometry.1.fingerprint(),
            lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1
        );
        assert_eq!(
            spec.public_cloud.1.fingerprint(),
            lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1
        );
        assert_eq!(spec.status.1.fingerprint(), lio::LIO_STATUS_FINGERPRINT_V1);
    }

    #[test]
    fn every_device_lifecycle_status_satisfies_codec_contract() {
        for (index, (lifecycle_state, reason_code, reason)) in [
            (lio::LIO_LIFECYCLE_STARTING, lio::LIO_REASON_NONE, ""),
            (lio::LIO_LIFECYCLE_READY, lio::LIO_REASON_NONE, ""),
            (
                lio::LIO_LIFECYCLE_STOPPING,
                lio::LIO_REASON_SHUTDOWN_REQUESTED,
                "shutdown requested",
            ),
            (
                lio::LIO_LIFECYCLE_STOPPED,
                lio::LIO_REASON_SHUTDOWN_REQUESTED,
                "stopped",
            ),
            (
                lio::LIO_LIFECYCLE_FAILED,
                lio::LIO_REASON_TRANSPORT_ERROR,
                "transport failed",
            ),
        ]
        .into_iter()
        .enumerate()
        {
            let message = lio::LioStatus {
                schema_version: lio::LIO_SCHEMA_VERSION_V1,
                backend: lio::LIO_BACKEND_DEVICE,
                runtime_generation: 7,
                status_sequence: i64::try_from(index + 1).unwrap(),
                status_timestamp_ns: 0,
                last_odometry_source_sequence: 0,
                last_odometry_source_timestamp_ns: 0,
                last_cloud_source_sequence: 0,
                last_cloud_source_timestamp_ns: 0,
                sensor_frame_id: DEFAULT_LIO_SENSOR_FRAME.into(),
                body_frame_id: "base_link".into(),
                odom_frame_id: "odom".into(),
                map_frame_id: "map".into(),
                lifecycle_state,
                reason_code,
                reason: reason.into(),
            };
            assert!(lio::encode_lio_status(&message).is_ok());
        }
    }

    #[test]
    fn native_shutdown_failure_keeps_child_reason_with_transport_teardown_errors() {
        let status_failure = classify_teardown_failure(
            None,
            Some("STOPPING status publish failed".to_owned()),
            None,
            Some("native child reap failed".to_owned()),
        )
        .expect("combined teardown failure");
        assert_eq!(status_failure.reason_code, lio::LIO_REASON_CHILD_CRASHED);
        assert_eq!(
            status_failure.message,
            "native child reap failed; teardown: STOPPING status publish failed"
        );

        let join_failure = classify_teardown_failure(
            None,
            None,
            Some("worker join failed".to_owned()),
            Some("native child kill timed out".to_owned()),
        )
        .expect("combined teardown failure");
        assert_eq!(join_failure.reason_code, lio::LIO_REASON_CHILD_CRASHED);
        assert_eq!(
            join_failure.message,
            "native child kill timed out; teardown: worker join failed"
        );

        let transport_only = classify_teardown_failure(
            None,
            Some("STOPPING status publish failed".to_owned()),
            None,
            None,
        )
        .expect("transport teardown failure");
        assert_eq!(transport_only.reason_code, lio::LIO_REASON_TRANSPORT_ERROR);
        assert_eq!(
            transport_only.message,
            "LIO owner teardown failed; teardown: STOPPING status publish failed"
        );

        let existing_terminal = classify_teardown_failure(
            Some(TerminalFailure::new(
                lio::LIO_REASON_STARTUP_TIMEOUT,
                "first source pair timed out",
            )),
            None,
            Some("worker join failed".to_owned()),
            Some("native child reap failed".to_owned()),
        )
        .expect("existing terminal failure");
        assert_eq!(
            existing_terminal.reason_code,
            lio::LIO_REASON_STARTUP_TIMEOUT
        );
        assert_eq!(
            existing_terminal.message,
            "first source pair timed out; teardown: worker join failed; teardown: native child reap failed"
        );
    }

    #[test]
    fn rejects_missing_or_wrong_typed_suffix() {
        let missing = spec_with(
            "/geometry/lio/odometry",
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap_err();
        assert!(missing.contains("missing typed message suffix"));

        let wrong = spec_with(
            "/geometry/lio/odometry#nav_msgs.Odometry",
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap_err();
        assert!(wrong.contains("message_type=nav_msgs.Odometry"));
    }

    #[test]
    fn rejects_public_topic_collision() {
        let duplicate = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            "/geometry/lio/odometry#nuwax_msgs.LioCloudEnvelope",
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap_err();
        assert!(duplicate.contains("pairwise distinct"));

        let legacy = spec_with(
            "/odom#nuwax_msgs.LioOdometryEnvelope",
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap_err();
        assert!(legacy.contains("conflicts with registry/legacy topic"));
    }

    #[test]
    fn rejects_invalid_or_aliased_frames() {
        let invalid = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            "/lidar",
        )
        .unwrap_err();
        assert!(invalid.contains("invalid_frame"));

        let aliased = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            "odom",
        )
        .unwrap_err();
        assert!(aliased.contains("frames must be pairwise distinct"));
    }

    #[test]
    fn rejects_surrounding_whitespace_in_topics_env_and_frames() {
        let padded_topic = spec_with(
            &format!(" {DEFAULT_LIO_ODOMETRY_CHANNEL}"),
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .unwrap_err();
        assert!(padded_topic.contains("topic"));

        let padded_frame = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            "utlidar_lidar ",
        )
        .unwrap_err();
        assert!(padded_frame.contains("invalid_frame"));

        let preferred = format!("NUWAX_GEOMETRY_TEST_LIO_PADDED_{}", std::process::id());
        let fallback = format!("{preferred}_FALLBACK");
        std::env::set_var(&preferred, " padded ");
        let padded_env = strict_env_or_default(&preferred, &fallback, "default").unwrap_err();
        std::env::remove_var(&preferred);
        assert!(padded_env.contains("value_has_surrounding_whitespace"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn device_owner_reaches_ready_without_sensor_messages() {
        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("device owner spec");
        let sink = Arc::new(TestIngressSink::default());

        let owner = tokio::time::timeout(
            Duration::from_secs(2),
            LioOwnerHandle::start(transport, spec, 1, sink.clone()),
        )
        .await
        .expect("owner READY timeout")
        .expect("owner READY");
        assert_eq!(sink.accepted.load(Ordering::Acquire), 0);

        tokio::time::timeout(Duration::from_secs(2), owner.shutdown())
            .await
            .expect("owner shutdown timeout")
            .expect("owner shutdown");
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_child_exit_zero_during_starting_is_exact_terminal_failure() {
        let provider = crate::lcm_runtime::test_lcm_url();
        let transport = LcmTransport::new_real(&provider).expect("real LCM owner context");
        let observer = LcmTransport::new_real(&provider).expect("real LCM status observer");
        let sink = Arc::new(NativeSupervisorTestSink::pending());
        let (spec, generation) =
            native_owner_spec("exit 0", Duration::from_secs(2), BTreeMap::new());
        let status_binding = spec.shared().status.clone();
        let mut status_frames = observer
            .subscribe(status_binding.0, Some(status_binding.1))
            .await;

        let error = tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native exit-zero owner timeout")
        .expect_err("exit zero during STARTING must fail");
        assert_eq!(error, "native LIO child exited unexpectedly with status 0");
        let starting = next_lio_status(&mut status_frames).await;
        assert_eq!(starting.lifecycle_state, lio::LIO_LIFECYCLE_STARTING);
        let failed = next_lio_status(&mut status_frames).await;
        assert_eq!(failed.backend, lio::LIO_BACKEND_FASTLIO2);
        assert_eq!(failed.runtime_generation, generation);
        assert_eq!(failed.lifecycle_state, lio::LIO_LIFECYCLE_FAILED);
        assert_eq!(failed.reason_code, lio::LIO_REASON_CHILD_EXITED);
        assert_eq!(failed.reason, error);
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_child_exit_nonzero_during_starting_is_exact_terminal_failure() {
        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::pending());
        let (spec, generation) =
            native_owner_spec("exit 23", Duration::from_secs(2), BTreeMap::new());

        let error = tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native nonzero owner timeout")
        .expect_err("nonzero exit during STARTING must fail");
        assert_eq!(error, "native LIO child exited unexpectedly with status 23");
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_child_signal_during_starting_is_exact_terminal_failure() {
        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::pending());
        let (spec, generation) =
            native_owner_spec("kill -KILL $$", Duration::from_secs(2), BTreeMap::new());

        let error = tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native signal owner timeout")
        .expect_err("signal exit during STARTING must fail");
        assert_eq!(
            error,
            format!("native LIO child exited from signal {}", nix::libc::SIGKILL)
        );
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_first_pair_timeout_is_exact_terminal_failure() {
        const FIRST_PAIR_TIMEOUT: Duration = Duration::from_millis(30);

        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::pending());
        let (spec, generation) = native_owner_spec(
            "trap 'exit 0' TERM; while :; do :; done",
            FIRST_PAIR_TIMEOUT,
            BTreeMap::new(),
        );

        let error = tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native first-pair owner timeout")
        .expect_err("missing first pair must fail STARTING");
        assert_eq!(
            error,
            format!(
                "native LIO first valid source pair timed out after {} ms",
                FIRST_PAIR_TIMEOUT.as_millis()
            )
        );
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_explicit_shutdown_sends_term_and_reaps_child() {
        const SCRIPT: &str = "trap 'printf term > \"$NUWAX_TEST_NATIVE_TERM_FILE\"; exit 0' TERM; printf '%s' \"$$\" > \"$NUWAX_TEST_NATIVE_PID_FILE\"; while :; do :; done";

        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::ready());
        let (pid_path, term_path, runtime_env) = native_child_probe();
        let (spec, generation) = native_owner_spec(SCRIPT, Duration::from_secs(2), runtime_env);
        let owner = tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native READY timeout")
        .expect("native READY");
        let pid = wait_for_file(&pid_path)
            .await
            .parse::<u32>()
            .expect("native child pid");

        tokio::time::timeout(Duration::from_secs(3), owner.shutdown())
            .await
            .expect("native shutdown timeout")
            .expect("native shutdown");
        assert_eq!(wait_for_file(&term_path).await, "term");
        wait_until_pid_is_reaped(pid).await;
        assert!(sink.closed.load(Ordering::Acquire));
        let _ = std::fs::remove_file(pid_path);
        let _ = std::fs::remove_file(term_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn cancelling_native_startup_future_still_terms_and_reaps_child() {
        const SCRIPT: &str = "trap 'printf term > \"$NUWAX_TEST_NATIVE_TERM_FILE\"; exit 0' TERM; printf '%s' \"$$\" > \"$NUWAX_TEST_NATIVE_PID_FILE\"; while :; do :; done";

        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::pending());
        let startup_entered = sink.startup_entered.notified();
        let (pid_path, term_path, runtime_env) = native_child_probe();
        let (spec, generation) = native_owner_spec(SCRIPT, Duration::from_secs(5), runtime_env);
        let start_task = tokio::spawn({
            let sink = sink.clone();
            async move { LioOwnerHandle::start(transport, spec, generation, sink).await }
        });

        tokio::time::timeout(Duration::from_secs(3), startup_entered)
            .await
            .expect("native startup wait was never entered");
        let pid = wait_for_file(&pid_path)
            .await
            .parse::<u32>()
            .expect("native child pid");
        start_task.abort();
        assert!(start_task
            .await
            .expect_err("aborted startup task must not return")
            .is_cancelled());

        assert_eq!(wait_for_file(&term_path).await, "term");
        wait_until_pid_is_reaped(pid).await;
        assert!(sink.closed.load(Ordering::Acquire));
        let _ = std::fs::remove_file(pid_path);
        let _ = std::fs::remove_file(term_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn old_generation_private_topics_cannot_reach_new_owner_or_ready() {
        const SCRIPT: &str = "trap 'printf term > \"$NUWAX_TEST_NATIVE_TERM_FILE\"; exit 0' TERM; printf '%s' \"$$\" > \"$NUWAX_TEST_NATIVE_PID_FILE\"; while :; do :; done";

        let provider = crate::lcm_runtime::test_lcm_url();
        let owner_transport = LcmTransport::new_real(&provider).expect("real LCM owner context");
        let publisher =
            LcmTransport::new_real(&provider).expect("independent real LCM publisher context");
        let sink = Arc::new(NativeSupervisorTestSink::ready_after_source_pair());
        let startup_entered = sink.startup_entered.notified();
        let (pid_path, term_path, runtime_env) = native_child_probe();
        let (spec, generation) = native_owner_spec(SCRIPT, Duration::from_secs(5), runtime_env);
        let old_generation = generation.checked_sub(1).expect("older generation");
        let old_channel_root = format!("/geometry/test/lio/{old_generation}/private");
        let old_odometry = binding(
            &format!("{old_channel_root}/odometry"),
            LEGACY_ODOMETRY_TYPE,
        );
        let old_cloud = binding(&format!("{old_channel_root}/cloud"), LEGACY_CLOUD_TYPE);
        let start_task = tokio::spawn({
            let sink = sink.clone();
            async move { LioOwnerHandle::start(owner_transport, spec, generation, sink).await }
        });

        tokio::time::timeout(Duration::from_secs(3), startup_entered)
            .await
            .expect("native startup wait was never entered");
        let pid = wait_for_file(&pid_path)
            .await
            .parse::<u32>()
            .expect("native child pid");
        publisher
            .publish(
                RawLcmFrameBuilder::new(old_odometry.0, old_odometry.1)
                    .seqno(1)
                    .payload(vec![1])
                    .build(),
            )
            .await
            .expect("publish old-generation odometry");
        publisher
            .publish(
                RawLcmFrameBuilder::new(old_cloud.0, old_cloud.1)
                    .seqno(1)
                    .payload(vec![1])
                    .build(),
            )
            .await
            .expect("publish old-generation cloud");
        tokio::time::sleep(Duration::from_millis(150)).await;

        assert_eq!(sink.source_odometry_accepted.load(Ordering::Acquire), 0);
        assert_eq!(sink.source_cloud_accepted.load(Ordering::Acquire), 0);
        assert!(
            !start_task.is_finished(),
            "old-generation private frames must not acknowledge READY"
        );

        start_task.abort();
        assert!(start_task
            .await
            .expect_err("aborted startup task must not return")
            .is_cancelled());
        assert_eq!(wait_for_file(&term_path).await, "term");
        wait_until_pid_is_reaped(pid).await;
        assert!(sink.closed.load(Ordering::Acquire));
        let _ = std::fs::remove_file(pid_path);
        let _ = std::fs::remove_file(term_path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn native_first_pair_and_child_exit_race_is_always_terminal() {
        let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
            .expect("real LCM owner context");
        let sink = Arc::new(NativeSupervisorTestSink::ready());
        let (spec, generation) =
            native_owner_spec("exit 0", Duration::from_secs(2), BTreeMap::new());

        match tokio::time::timeout(
            Duration::from_secs(3),
            LioOwnerHandle::start(transport, spec, generation, sink.clone()),
        )
        .await
        .expect("native first-pair/exit race timeout")
        {
            Err(error) => {
                assert_eq!(error, "native LIO child exited unexpectedly with status 0");
            }
            Ok(mut owner) => {
                let error = tokio::time::timeout(Duration::from_secs(3), owner.wait_terminal())
                    .await
                    .expect("post-READY child exit was not observed")
                    .expect_err("post-READY child exit must be terminal");
                assert_eq!(error, "native LIO child exited unexpectedly with status 0");
            }
        }
        assert!(sink.closed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn independent_real_lcm_context_preempts_accept_before_commit_admission() {
        let provider = crate::lcm_runtime::test_lcm_url();
        let owner_transport = LcmTransport::new_real(&provider).expect("real LCM owner context");
        let publisher =
            LcmTransport::new_real(&provider).expect("independent real LCM publisher context");
        let spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("device owner spec");
        let bindings = [
            spec.public_odometry.clone(),
            spec.public_cloud.clone(),
            spec.legacy_odometry.clone(),
            spec.legacy_cloud.clone(),
        ];
        let sink = Arc::new(TestIngressSink {
            block_public_odometry: AtomicBool::new(true),
            ..TestIngressSink::default()
        });
        let owner = tokio::time::timeout(
            Duration::from_secs(2),
            LioOwnerHandle::start(owner_transport, spec, 2, sink.clone()),
        )
        .await
        .expect("owner READY timeout")
        .expect("owner READY");

        let entered = sink.public_odometry_entered.notified();
        publisher
            .publish(
                RawLcmFrameBuilder::new(bindings[0].0.clone(), bindings[0].1.clone())
                    .payload(vec![1])
                    .build(),
            )
            .await
            .expect("publish in-flight odometry");
        tokio::time::timeout(Duration::from_secs(2), entered)
            .await
            .expect("public odometry accept did not start");

        tokio::time::timeout(Duration::from_secs(2), owner.shutdown())
            .await
            .expect("shutdown must preempt in-flight accept")
            .expect("all workers acknowledged unsubscribe");
        assert!(sink.closed.load(Ordering::Acquire));
        assert_eq!(sink.accepted.load(Ordering::Acquire), 1);

        for (topic, message_type) in bindings {
            publisher
                .publish(
                    RawLcmFrameBuilder::new(topic, message_type)
                        .payload(vec![2])
                        .build(),
                )
                .await
                .expect("post-shutdown publish");
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(
            sink.accepted.load(Ordering::Acquire),
            1,
            "no worker may deliver after acknowledged shutdown"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn shutdown_waits_for_started_commit_before_acknowledgement() {
        let provider = crate::lcm_runtime::test_lcm_url();
        let owner_transport = LcmTransport::new_real(&provider).expect("real LCM owner context");
        let publisher =
            LcmTransport::new_real(&provider).expect("independent real LCM publisher context");
        let spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("device owner spec");
        let public_odometry = spec.public_odometry.clone();
        let sink = Arc::new(TestIngressSink {
            drain_public_odometry: AtomicBool::new(true),
            ..TestIngressSink::default()
        });
        let owner = LioOwnerHandle::start(owner_transport, spec, 3, sink.clone())
            .await
            .expect("owner READY");

        let entered = sink.public_odometry_entered.notified();
        publisher
            .publish(
                RawLcmFrameBuilder::new(public_odometry.0, public_odometry.1)
                    .payload(vec![1])
                    .build(),
            )
            .await
            .expect("publish commit candidate");
        tokio::time::timeout(Duration::from_secs(2), entered)
            .await
            .expect("commit did not start");

        let shutdown_task = tokio::spawn(async move { owner.shutdown().await });
        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(sink.closed.load(Ordering::Acquire));
        assert!(!shutdown_task.is_finished());
        assert!(!sink.public_odometry_completed.load(Ordering::Acquire));

        sink.public_odometry_release.notify_one();
        tokio::time::timeout(Duration::from_secs(2), shutdown_task)
            .await
            .expect("shutdown did not drain started commit")
            .expect("shutdown task join")
            .expect("owner shutdown");
        assert!(sink.public_odometry_completed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn shutdown_rejects_acknowledgement_when_started_commit_never_drains() {
        let provider = crate::lcm_runtime::test_lcm_url();
        let owner_transport = LcmTransport::new_real(&provider).expect("real LCM owner context");
        let publisher =
            LcmTransport::new_real(&provider).expect("independent real LCM publisher context");
        let spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("device owner spec");
        let public_odometry = spec.public_odometry.clone();
        let sink = Arc::new(TestIngressSink {
            drain_public_odometry: AtomicBool::new(true),
            ..TestIngressSink::default()
        });
        let owner = LioOwnerHandle::start(owner_transport, spec, 4, sink.clone())
            .await
            .expect("owner READY");

        let entered = sink.public_odometry_entered.notified();
        publisher
            .publish(
                RawLcmFrameBuilder::new(public_odometry.0, public_odometry.1)
                    .payload(vec![1])
                    .build(),
            )
            .await
            .expect("publish non-draining commit candidate");
        tokio::time::timeout(Duration::from_secs(2), entered)
            .await
            .expect("non-draining commit did not start");

        let error = tokio::time::timeout(Duration::from_secs(3), owner.shutdown())
            .await
            .expect("owner shutdown exceeded its bounded deadline")
            .expect_err("non-draining commit must never receive successful shutdown ACK");
        assert!(
            (error.contains("worker shutdown timed out") && error.contains("aborted"))
                || error.contains("supervisor shutdown timed out and was aborted"),
            "unexpected bounded shutdown failure: {error}"
        );
        assert!(sink.closed.load(Ordering::Acquire));
        assert!(!sink.public_odometry_completed.load(Ordering::Acquire));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn real_lcm_legacy_frames_reach_versioned_mapping_handoff_and_quiesce_on_shutdown() {
        const RUNTIME_GENERATION: i64 = 41;
        const ODOM_TIMESTAMP_NS: i64 = 1_700_000_000_100_000_000;
        const CLOUD_TIMESTAMP_NS: i64 = 1_700_000_000_200_000_000;

        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real LCM Geometry handle");
        let provider = handle
            .transport
            .shared_lcm_context()
            .expect("Geometry handle real LCM context")
            .provider()
            .to_owned();
        let publisher =
            LcmTransport::new_real(&provider).expect("independent real LCM publisher context");
        let owner_spec = spec_with(
            DEFAULT_LIO_ODOMETRY_CHANNEL,
            DEFAULT_LIO_CLOUD_CHANNEL,
            DEFAULT_LIO_STATUS_CHANNEL,
            DEFAULT_LIO_SENSOR_FRAME,
        )
        .expect("device owner spec");
        let ingress_spec = DeviceLioIngressSpec::new(
            RUNTIME_GENERATION,
            owner_spec.sensor_frame.clone(),
            owner_spec.body_frame.clone(),
            owner_spec.odom_frame.clone(),
            owner_spec.map_frame.clone(),
        )
        .expect("device ingress spec");
        let tf_binding = binding("/tf", "tf2_msgs.TFMessage");
        let ingress = Arc::new(CommonLioIngress::with_runtime(
            ingress_spec,
            handle.clone(),
            owner_spec.public_odometry.clone(),
            owner_spec.public_cloud.clone(),
            tf_binding.clone(),
        ));

        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        let session_generation = mapping
            .lock()
            .start_session("lio-owner-v2", "lio-owner-map")
            .expect("active mapping session")
            .session_generation;
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-lio-owner-v2-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&cloud_path).expect("test cloud-map writer"));

        let mut public_odometry_frames = publisher
            .subscribe(
                owner_spec.public_odometry.0.clone(),
                Some(owner_spec.public_odometry.1.clone()),
            )
            .await;
        let mut public_cloud_frames = publisher
            .subscribe(
                owner_spec.public_cloud.0.clone(),
                Some(owner_spec.public_cloud.1.clone()),
            )
            .await;
        let mut tf_frames = publisher
            .subscribe(tf_binding.0.clone(), Some(tf_binding.1.clone()))
            .await;
        let global_map_binding = binding("/global_map", "sensor_msgs.PointCloud2");
        let mut global_map_frames = publisher
            .subscribe(
                global_map_binding.0.clone(),
                Some(global_map_binding.1.clone()),
            )
            .await;
        let mut map_revisions = handle.map_revision_tx.subscribe();
        let mut cloud_updates = handle.cloud_map_tx.subscribe();

        let owner = tokio::time::timeout(
            Duration::from_secs(2),
            LioOwnerHandle::start(
                handle.transport.clone(),
                owner_spec.clone(),
                RUNTIME_GENERATION,
                ingress.clone(),
            ),
        )
        .await
        .expect("owner READY timeout")
        .expect("owner READY");
        tokio::time::sleep(Duration::from_millis(50)).await;

        publisher
            .publish(legacy_frame(
                &owner_spec.legacy_odometry,
                1,
                schema::encode_odometry_body(&legacy_odometry(ODOM_TIMESTAMP_NS))
                    .expect("legacy Odometry body"),
            ))
            .await
            .expect("publish legacy Odometry");
        let public_odometry_frame = next_real_frame(&mut public_odometry_frames).await;
        let mut encoded_odometry = lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1.to_vec();
        encoded_odometry.extend_from_slice(&public_odometry_frame.payload);
        let public_odometry = lio::decode_lio_odometry_envelope(&encoded_odometry)
            .expect("versioned LIO odometry envelope");
        assert_eq!(public_odometry.runtime_generation, RUNTIME_GENERATION);
        assert_eq!(public_odometry.source_sequence, 1);
        assert_eq!(public_odometry.source_timestamp_ns, ODOM_TIMESTAMP_NS);
        assert_eq!(public_odometry.odom_frame_id, "odom");
        assert_eq!(public_odometry.body_frame_id, "base_link");
        assert_eq!(public_odometry.map_frame_id, "map");
        assert_eq!(public_odometry.odometry.pose.position, [1.0, 2.0, 0.0]);
        let _tf_frame = next_real_frame(&mut tf_frames).await;
        assert_eq!(mapping.lock().odom_count(), 1);
        assert_eq!(
            ingress.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: ODOM_TIMESTAMP_NS,
            })
        );

        let source_points = [[2.0_f32, 2.0, 0.0], [3.0, 2.0, 0.0]];
        publisher
            .publish(legacy_frame(
                &owner_spec.legacy_cloud,
                1,
                schema::encode_point_cloud2_body(&legacy_cloud(CLOUD_TIMESTAMP_NS, &source_points))
                    .expect("legacy PointCloud2 body"),
            ))
            .await
            .expect("publish legacy PointCloud2");
        let public_cloud_frame = next_real_frame(&mut public_cloud_frames).await;
        let mut encoded_cloud = lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1.to_vec();
        encoded_cloud.extend_from_slice(&public_cloud_frame.payload);
        let public_cloud =
            lio::decode_lio_cloud_envelope(&encoded_cloud).expect("versioned LIO cloud envelope");
        assert_eq!(public_cloud.runtime_generation, RUNTIME_GENERATION);
        assert_eq!(public_cloud.source_sequence, 1);
        assert_eq!(public_cloud.source_timestamp_ns, CLOUD_TIMESTAMP_NS);
        assert_eq!(public_cloud.correlated_odometry_source_sequence, 1);
        assert_eq!(
            public_cloud.correlated_odometry_source_timestamp_ns,
            ODOM_TIMESTAMP_NS
        );
        assert_eq!(
            public_cloud.registration_state,
            lio::LIO_REGISTRATION_REGISTERED_MAP
        );
        assert_eq!(public_cloud.cloud.header.frame_id, "map");
        assert_eq!(public_cloud.source_point_count, 2);
        assert_eq!(public_cloud.emitted_point_count, 2);

        let map_revision = tokio::time::timeout(Duration::from_secs(2), map_revisions.recv())
            .await
            .expect("mapping revision timeout")
            .expect("mapping revision");
        let cloud_update = tokio::time::timeout(Duration::from_secs(2), cloud_updates.recv())
            .await
            .expect("cloud update timeout")
            .expect("cloud update");
        let global_map_frame = next_real_frame(&mut global_map_frames).await;
        let mut encoded_global_map = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
        encoded_global_map.extend_from_slice(&global_map_frame.payload);
        let global_map =
            schema::decode_point_cloud2(&encoded_global_map).expect("typed global map");
        assert_eq!(map_revision.frame_id, "map");
        assert_eq!(map_revision.timestamp_ns, CLOUD_TIMESTAMP_NS as u64);
        assert!(map_revision.revision > 0);
        assert_eq!(cloud_update.session_generation, session_generation);
        assert_eq!(global_map.header.frame_id, "map");
        assert_eq!(global_map.header.stamp_sec, 1_700_000_000);
        assert_eq!(global_map.header.stamp_nsec, 200_000_000);
        assert!(!global_map.data.is_empty());
        {
            let adapter = mapping.lock();
            assert_eq!(adapter.pointcloud_count(), 1);
            assert_eq!(adapter.current_revision(), map_revision.revision);
        }
        let committed_cloud = handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(CloudMapShmWriter::committed_snapshot)
            .expect("committed cloud-map SHM snapshot");
        assert_eq!(committed_cloud.session_generation, session_generation);
        assert_eq!(committed_cloud.revision, cloud_update.revision);
        assert_eq!(committed_cloud.point_count, cloud_update.point_count);
        let state = handle.state.read().await;
        let committed_map = &state.map;
        assert!(committed_map.revision > 0);
        assert_eq!(committed_map.timestamp_ns, CLOUD_TIMESTAMP_NS as u64);
        drop(state);

        tokio::time::timeout(Duration::from_secs(2), owner.shutdown())
            .await
            .expect("owner shutdown timeout")
            .expect("owner acknowledged shutdown");
        let stopped_ingress = ingress.snapshot();
        assert!(stopped_ingress.closed);
        let stopped_revision = mapping.lock().current_revision();
        let stopped_odom_count = mapping.lock().odom_count();
        let stopped_cloud_count = mapping.lock().pointcloud_count();

        publisher
            .publish(legacy_frame(
                &owner_spec.legacy_odometry,
                2,
                schema::encode_odometry_body(&legacy_odometry(ODOM_TIMESTAMP_NS + 1_000_000))
                    .expect("post-shutdown Odometry body"),
            ))
            .await
            .expect("post-shutdown legacy Odometry publish");
        publisher
            .publish(legacy_frame(
                &owner_spec.legacy_cloud,
                2,
                schema::encode_point_cloud2_body(&legacy_cloud(
                    CLOUD_TIMESTAMP_NS + 1_000_000,
                    &source_points,
                ))
                .expect("post-shutdown PointCloud2 body"),
            ))
            .await
            .expect("post-shutdown legacy PointCloud2 publish");
        for stream in [
            &mut public_odometry_frames,
            &mut public_cloud_frames,
            &mut tf_frames,
            &mut global_map_frames,
        ] {
            assert!(
                tokio::time::timeout(Duration::from_millis(150), stream.next())
                    .await
                    .is_err(),
                "acknowledged shutdown must leave every derived output silent"
            );
        }
        assert!(
            tokio::time::timeout(Duration::from_millis(150), map_revisions.recv())
                .await
                .is_err(),
            "acknowledged shutdown must prevent mapping revisions"
        );
        assert_eq!(ingress.snapshot(), stopped_ingress);
        assert_eq!(mapping.lock().current_revision(), stopped_revision);
        assert_eq!(mapping.lock().odom_count(), stopped_odom_count);
        assert_eq!(mapping.lock().pointcloud_count(), stopped_cloud_count);

        *handle.cloud_map_writer.lock() = None;
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn shutdown_control_observes_terminal_result() {
        let (control, _, completion_tx) = LioShutdownControl::test_channel();
        let waiter = tokio::spawn({
            let control = control.clone();
            async move { control.shutdown_and_wait().await }
        });
        tokio::task::yield_now().await;
        assert!(*control.request_tx.borrow());
        completion_tx.send_replace(Some(Ok(())));
        assert_eq!(waiter.await.unwrap(), Ok(()));
    }

    #[tokio::test]
    async fn terminal_wait_is_cancel_safe_and_cached_for_shutdown() {
        let (shutdown, _, _) = LioShutdownControl::test_channel();
        let (release_tx, release_rx) = oneshot::channel();
        let supervisor = tokio::spawn(async move {
            release_rx.await.expect("terminal release");
            Ok(())
        });
        let mut owner = LioOwnerHandle {
            shutdown,
            supervisor: Some(supervisor),
            terminal: None,
        };

        assert!(
            tokio::time::timeout(Duration::from_millis(20), owner.wait_terminal())
                .await
                .is_err()
        );
        release_tx.send(()).expect("release supervisor");
        assert_eq!(owner.wait_terminal().await, Ok(()));
        assert_eq!(owner.shutdown().await, Ok(()));
    }

    #[tokio::test]
    async fn terminal_wait_reports_and_caches_supervisor_panic() {
        let (shutdown, _, _) = LioShutdownControl::test_channel();
        let supervisor = tokio::spawn(async move {
            panic!("injected post-READY supervisor panic");
            #[allow(unreachable_code)]
            Ok(())
        });
        let mut owner = LioOwnerHandle {
            shutdown,
            supervisor: Some(supervisor),
            terminal: None,
        };

        let error = owner
            .wait_terminal()
            .await
            .expect_err("supervisor panic must be terminal");
        assert!(error.contains("LIO supervisor join failed"));
        assert_eq!(owner.shutdown().await, Err(error));
    }
}
