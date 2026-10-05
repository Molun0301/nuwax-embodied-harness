//! Generation-fenced online native-PGO owner.
//!
//! The owner in this module only returns validated correction candidates. It
//! never publishes canonical TF, maps, or legacy `pgo_tf` topics; Geometry
//! remains the sole canonical correction writer.

use std::collections::BTreeMap;
use std::path::PathBuf;
#[cfg(test)]
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use bytes::Bytes;
use nuwax_codec::lcm::native_pgo as codec;
pub use nuwax_codec::lcm::native_pgo::{NativePgoRequest, NativePgoResult, NativePgoStatus};
use nuwax_lcm::{LcmContext, LcmReceivedMessage, LcmSubscription};
use nuwax_nav_native_runner::{
    spawn_supervised, SupervisedLogs, SupervisedNativeChild, SupervisedNativeCommand,
    SupervisedNativeConfig, SupervisedReadyError, SupervisedShutdown, SupervisedStartError,
    SupervisedTerminal,
};
use thiserror::Error;
use tokio::sync::{broadcast, mpsc, oneshot};
use tokio::task::JoinHandle;

const REQUEST_TYPE: &str = "nuwax_msgs.NativePgoRequest";
const RESULT_TYPE: &str = "nuwax_msgs.NativePgoResult";
const STATUS_TYPE: &str = "nuwax_msgs.NativePgoStatus";
const OWNER_COMMAND_CAPACITY: usize = 4;
const ADMISSION_CAPACITY: usize = 1;
const CHILD_POLL_INTERVAL: Duration = Duration::from_millis(5);

/// Generation-private typed LCM topics owned by one online PGO generation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OnlinePgoTopics {
    /// Complete typed request channel.
    pub request: String,
    /// Complete typed result channel.
    pub result: String,
    /// Complete typed owner/status channel.
    pub status: String,
    /// Complete typed cancellation channel.
    pub cancel: String,
}

impl OnlinePgoTopics {
    /// Derive the only admitted channel set for one runtime/backend generation.
    pub fn for_runtime(
        runtime_generation: i64,
        backend_generation: i64,
    ) -> Result<Self, OnlinePgoStartError> {
        if runtime_generation <= 0 {
            return Err(OnlinePgoStartError::Preflight(
                "runtime_generation must be positive".into(),
            ));
        }
        if backend_generation <= 0 {
            return Err(OnlinePgoStartError::Preflight(
                "backend_generation must be positive".into(),
            ));
        }
        let prefix = format!("/c/n/{runtime_generation}/b/{backend_generation}");
        Ok(Self {
            request: format!("{prefix}/request#{REQUEST_TYPE}"),
            result: format!("{prefix}/result#{RESULT_TYPE}"),
            status: format!("{prefix}/status#{STATUS_TYPE}"),
            cancel: format!("{prefix}/cancel#{STATUS_TYPE}"),
        })
    }
}

/// Startup and bounded-lifecycle configuration for one native PGO owner.
#[derive(Clone, Debug)]
pub struct OnlinePgoConfig {
    /// Built native PGO executable.
    pub executable: PathBuf,
    /// Optional child working directory.
    pub cwd: Option<PathBuf>,
    /// Additional child environment.
    pub env: BTreeMap<String, String>,
    /// Whether the child starts from an empty environment before `env` is
    /// applied.
    pub clear_environment: bool,
    /// Real libLCM provider shared with the child, but opened through
    /// independent publisher and subscriber contexts in this process.
    pub lcm_url: String,
    /// Geometry runtime generation used in every private topic and message.
    pub runtime_generation: i64,
    /// Native backend generation used to reject stale restart traffic.
    pub backend_generation: i64,
    /// Previous backend generation, when this start is a restart. The new
    /// generation must be strictly greater.
    pub previous_backend_generation: Option<i64>,
    /// Absolute READY wait is derived from this duration.
    pub ready_timeout: Duration,
    /// Exact result plus terminal-status deadline for each request.
    pub result_timeout: Duration,
    /// Exact cancellation acknowledgement plus terminal deadline.
    pub cancel_timeout: Duration,
    /// SIGTERM grace period before the supervisor escalates to SIGKILL.
    pub shutdown_term_grace: Duration,
    /// Diagnostic bounds and startup rollback/reap policy.
    pub supervisor: SupervisedNativeConfig,
}

impl OnlinePgoConfig {
    /// Construct a production configuration with bounded default deadlines.
    #[must_use]
    pub fn new(
        lcm_url: impl Into<String>,
        runtime_generation: i64,
        backend_generation: i64,
    ) -> Self {
        Self {
            executable: crate::native_executable(),
            cwd: None,
            env: BTreeMap::new(),
            clear_environment: false,
            lcm_url: lcm_url.into(),
            runtime_generation,
            backend_generation,
            previous_backend_generation: None,
            ready_timeout: Duration::from_secs(5),
            result_timeout: Duration::from_secs(5),
            cancel_timeout: Duration::from_secs(2),
            shutdown_term_grace: Duration::from_millis(500),
            supervisor: SupervisedNativeConfig::default(),
        }
    }

    fn validate(&self) -> Result<OnlinePgoTopics, OnlinePgoStartError> {
        let topics =
            OnlinePgoTopics::for_runtime(self.runtime_generation, self.backend_generation)?;
        if self
            .previous_backend_generation
            .is_some_and(|previous| previous <= 0)
        {
            return Err(OnlinePgoStartError::Preflight(
                "previous_backend_generation must be positive when present".into(),
            ));
        }
        if self
            .previous_backend_generation
            .is_some_and(|previous| self.backend_generation <= previous)
        {
            return Err(OnlinePgoStartError::Preflight(
                "backend_generation must advance across restart".into(),
            ));
        }
        if self.lcm_url.is_empty()
            || self.lcm_url.len() > 1_024
            || self.lcm_url.chars().any(char::is_whitespace)
            || self.lcm_url.chars().any(char::is_control)
        {
            return Err(OnlinePgoStartError::Preflight(
                "lcm_url must be nonempty, bounded, and contain no whitespace/control bytes".into(),
            ));
        }
        let metadata = std::fs::metadata(&self.executable).map_err(|error| {
            OnlinePgoStartError::Preflight(format!(
                "native PGO executable {} is unavailable: {error}",
                self.executable.display()
            ))
        })?;
        if !metadata.is_file() {
            return Err(OnlinePgoStartError::Preflight(format!(
                "native PGO executable {} is not a file",
                self.executable.display()
            )));
        }
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            if metadata.permissions().mode() & 0o111 == 0 {
                return Err(OnlinePgoStartError::Preflight(format!(
                    "native PGO artifact {} has no executable bit",
                    self.executable.display()
                )));
            }
        }
        if let Some(cwd) = &self.cwd {
            let metadata = std::fs::metadata(cwd).map_err(|error| {
                OnlinePgoStartError::Preflight(format!(
                    "native PGO cwd {} is unavailable: {error}",
                    cwd.display()
                ))
            })?;
            if !metadata.is_dir() {
                return Err(OnlinePgoStartError::Preflight(format!(
                    "native PGO cwd {} is not a directory",
                    cwd.display()
                )));
            }
        }
        for (name, value) in &self.env {
            if name.is_empty() || name.contains('=') || name.contains('\0') || value.contains('\0')
            {
                return Err(OnlinePgoStartError::Preflight(
                    "native PGO environment contains an invalid name or NUL".into(),
                ));
            }
        }
        for (name, duration) in [
            ("ready_timeout", self.ready_timeout),
            ("result_timeout", self.result_timeout),
            ("cancel_timeout", self.cancel_timeout),
            ("shutdown_term_grace", self.shutdown_term_grace),
        ] {
            if duration.is_zero() || Instant::now().checked_add(duration).is_none() {
                return Err(OnlinePgoStartError::Preflight(format!(
                    "{name} must be positive and bounded"
                )));
            }
        }
        Ok(topics)
    }
}

/// Render the frozen online child argument vector, excluding argv[0].
#[must_use]
pub fn online_pgo_argv(config: &OnlinePgoConfig, topics: &OnlinePgoTopics) -> Vec<String> {
    vec![
        "--mode".into(),
        "online".into(),
        "--lcm-url".into(),
        config.lcm_url.clone(),
        "--runtime-generation".into(),
        config.runtime_generation.to_string(),
        "--backend-generation".into(),
        config.backend_generation.to_string(),
        "--request-channel".into(),
        topics.request.clone(),
        "--result-channel".into(),
        topics.result.clone(),
        "--status-channel".into(),
        topics.status.clone(),
        "--cancel-channel".into(),
        topics.cancel.clone(),
    ]
}

/// Failure before a retained READY owner can be returned.
#[derive(Debug, Error)]
pub enum OnlinePgoStartError {
    /// Artifact, generation, provider, path, or deadline preflight failed.
    #[error("native PGO online preflight failed: {0}")]
    Preflight(String),
    /// Real LCM context/subscription setup failed.
    #[error("native PGO online transport setup failed: {0}")]
    Transport(String),
    /// Child spawn/readiness failed; the supervisor has already attempted
    /// bounded process-group rollback.
    #[error("native PGO online supervised startup failed: {source}{cleanup}")]
    Supervision {
        /// Typed supervisor failure and bounded diagnostics.
        #[source]
        source: Box<SupervisedStartError>,
        /// Any additional acknowledged LCM cleanup failure.
        cleanup: String,
    },
}

/// Fail-closed runtime failure for one backend generation.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum OnlinePgoError {
    /// One request already owns the capacity-one admission slot.
    #[error("native PGO capacity-one admission is busy")]
    Busy,
    /// The owner no longer accepts work.
    #[error("native PGO owner is stopped")]
    Stopped,
    /// Geometry supplied an internally inconsistent complete request.
    #[error("native PGO request rejected before publish: {0}")]
    InvalidRequest(String),
    /// Real typed LCM failed.
    #[error("native PGO transport failed: {0}")]
    Transport(String),
    /// A typed status/result violated identity, receipt, ordering, or matrix rules.
    #[error("native PGO protocol failed: {0}")]
    Protocol(String),
    /// A typed post-READY health sample closed this backend generation.
    #[error(
        "native PGO owner health closed the generation: state={health_state}, reason={reason_code}: {reason}"
    )]
    OwnerHealth {
        /// `NATIVE_PGO_HEALTH_DEGRADED` or `NATIVE_PGO_HEALTH_UNHEALTHY`.
        health_state: i32,
        /// Typed native-PGO reason code.
        reason_code: i32,
        /// Bounded typed native-PGO reason text.
        reason: String,
    },
    /// Exact result plus terminal status did not arrive in time.
    #[error("native PGO request {request_sequence} timed out")]
    RequestTimedOut {
        /// Owner-assigned request sequence.
        request_sequence: i64,
    },
    /// Exact cancellation acknowledgement plus terminal did not arrive.
    #[error("native PGO cancellation for request {request_sequence} timed out")]
    CancelTimedOut {
        /// Owner-assigned request sequence.
        request_sequence: i64,
    },
    /// Child exited or crashed while the generation was admitted.
    #[error("native PGO child terminated: {0:?}")]
    ChildTerminated(SupervisedTerminal),
    /// Process observation or bounded TERM/KILL/reap failed.
    #[error("native PGO supervision failed: {0}")]
    Supervision(String),
    /// Subscription/context teardown could not be acknowledged.
    #[error("native PGO cleanup failed: {0}")]
    Cleanup(String),
    /// The retained background owner task panicked or disappeared.
    #[error("native PGO owner task failed: {0}")]
    OwnerTask(String),
}

/// Read-only full-SE(3) transform validated from a correction result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValidatedOnlinePgoTransform {
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
}

impl ValidatedOnlinePgoTransform {
    /// Absolute-left correction translation in metres.
    #[must_use]
    pub fn translation_m(&self) -> [f64; 3] {
        self.translation_m
    }

    /// Absolute-left correction quaternion in XYZW order.
    #[must_use]
    pub fn rotation_xyzw(&self) -> [f64; 4] {
        self.rotation_xyzw
    }
}

/// Read-only disposition of a bridge-validated online PGO result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidatedOnlinePgoDisposition {
    /// Successful request with no admissible loop candidate.
    NoCandidate,
    /// Valid full-SE(3) correction candidate.
    Correction,
    /// Typed request rejection, including cancellation rejection.
    Rejected,
}

/// Exact typed result returned to Geometry after protocol validation.
///
/// Its representation is private: only the bridge can construct this proof.
/// Consumers may inspect it through read-only accessors or consume it to take
/// ownership of the already-validated native result.
#[derive(Clone, Debug, PartialEq)]
pub struct ValidatedOnlinePgoResult {
    result: NativePgoResult,
    transform: Option<ValidatedOnlinePgoTransform>,
}

impl ValidatedOnlinePgoResult {
    fn from_result(
        request: &NativePgoRequest,
        result: NativePgoResult,
    ) -> Result<Self, OnlinePgoError> {
        codec::validate_native_pgo_result_echo(request, &result)
            .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
        let transform = match result.result_disposition {
            codec::NATIVE_PGO_DISPOSITION_NO_CANDIDATE | codec::NATIVE_PGO_DISPOSITION_REJECTED => {
                None
            }
            codec::NATIVE_PGO_DISPOSITION_CORRECTION => Some(ValidatedOnlinePgoTransform {
                translation_m: result.translation_m,
                rotation_xyzw: result.rotation_xyzw,
            }),
            disposition => Err(OnlinePgoError::Protocol(format!(
                "unknown decoded result disposition {disposition}"
            )))?,
        };
        Ok(Self { result, transform })
    }

    /// Return the validated result disposition.
    #[must_use]
    pub fn disposition(&self) -> ValidatedOnlinePgoDisposition {
        match self.result.result_disposition {
            codec::NATIVE_PGO_DISPOSITION_NO_CANDIDATE => {
                ValidatedOnlinePgoDisposition::NoCandidate
            }
            codec::NATIVE_PGO_DISPOSITION_CORRECTION => ValidatedOnlinePgoDisposition::Correction,
            codec::NATIVE_PGO_DISPOSITION_REJECTED => ValidatedOnlinePgoDisposition::Rejected,
            _ => unreachable!("private validated result contains a checked disposition"),
        }
    }

    /// Borrow the complete validated result.
    #[must_use]
    pub fn result(&self) -> &NativePgoResult {
        &self.result
    }

    /// Return the correction transform when the disposition is CORRECTION.
    #[must_use]
    pub fn transform(&self) -> Option<ValidatedOnlinePgoTransform> {
        self.transform
    }

    /// Consume this bridge proof and return the validated native result.
    #[must_use]
    pub fn into_result(self) -> NativePgoResult {
        self.result
    }
}

/// Exact outcome of a published cancellation request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OnlinePgoCancelOutcome {
    /// The active request acknowledged cancellation and terminated cancelled.
    Cancelled,
    /// The request became terminal before cancellation could take effect.
    TooLate,
    /// No request was in flight.
    NoInFlight,
}

/// Why the retained online owner stopped.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OnlinePgoStopCause {
    /// Explicit shutdown or drop of the retained owner handle.
    Requested,
    /// A protocol, transport, timeout, or child failure terminated generation.
    Terminal(OnlinePgoError),
}

/// Proven shutdown result after process reap and acknowledged LCM teardown.
#[derive(Clone, Debug)]
pub struct OnlinePgoShutdown {
    /// Requested versus fail-closed terminal shutdown.
    pub cause: OnlinePgoStopCause,
    /// Direct-child and process-group cleanup proof.
    pub process: SupervisedShutdown,
    /// Bounded stdout/stderr diagnostics; stdout never establishes readiness.
    pub logs: SupervisedLogs,
}

/// Capacity-one Geometry ingress for complete native-PGO requests.
///
/// This handle deliberately does not implement `Clone`. Dropping an in-flight
/// submit future does not reopen admission; the retained owner releases the
/// slot only after an exact terminal result or generation shutdown.
#[derive(Debug)]
pub struct OnlinePgoIngress {
    tx: mpsc::Sender<Admission>,
    admitted: Arc<AtomicBool>,
}

impl OnlinePgoIngress {
    /// Submit one complete Geometry request and await an exact typed result.
    pub async fn submit(
        &mut self,
        request: NativePgoRequest,
    ) -> Result<ValidatedOnlinePgoResult, OnlinePgoError> {
        self.admitted
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| OnlinePgoError::Busy)?;
        let (reply_tx, reply_rx) = oneshot::channel();
        if self.tx.send(Admission { request, reply_tx }).await.is_err() {
            self.admitted.store(false, Ordering::Release);
            return Err(OnlinePgoError::Stopped);
        }
        reply_rx.await.unwrap_or(Err(OnlinePgoError::Stopped))
    }

    /// Whether the retained owner still has an admission receiver.
    #[must_use]
    pub fn is_open(&self) -> bool {
        !self.tx.is_closed()
    }
}

/// Retained, non-cloneable owner of the child, subscriptions, and LCM contexts.
#[derive(Debug)]
#[must_use = "retain the online PGO owner and call shutdown for cleanup proof"]
pub struct OnlinePgoOwner {
    control_tx: Option<mpsc::Sender<OwnerCommand>>,
    task: Option<JoinHandle<Result<OnlinePgoShutdown, OnlinePgoError>>>,
    topics: OnlinePgoTopics,
}

impl OnlinePgoOwner {
    /// Preflight, subscribe, spawn, and wait for an exact typed READY status.
    ///
    /// Result and status subscriptions are acknowledged by the real libLCM
    /// owner thread before `spawn_supervised` is called. Admission becomes
    /// reachable only after READY has been decoded and generation-checked.
    pub async fn start(
        config: OnlinePgoConfig,
    ) -> Result<(Self, OnlinePgoIngress), OnlinePgoStartError> {
        let topics = config.validate()?;
        let publisher = LcmContext::new(&config.lcm_url)
            .map_err(|error| OnlinePgoStartError::Transport(error.to_string()))?;
        let subscriber = match LcmContext::new(&config.lcm_url) {
            Ok(context) => context,
            Err(error) => {
                let cleanup = publisher.shutdown().await.err();
                return Err(OnlinePgoStartError::Transport(format!(
                    "subscriber context: {error}{}",
                    cleanup
                        .map(|failure| format!("; publisher cleanup: {failure}"))
                        .unwrap_or_default()
                )));
            }
        };
        let mut result_subscription = match subscriber.subscribe(&topics.result).await {
            Ok(subscription) => subscription,
            Err(error) => {
                let cleanup = shutdown_context_pair(&publisher, &subscriber).await;
                return Err(OnlinePgoStartError::Transport(format!(
                    "result subscription: {error}{}",
                    cleanup_suffix(cleanup)
                )));
            }
        };
        let mut status_subscription = match subscriber.subscribe(&topics.status).await {
            Ok(subscription) => subscription,
            Err(error) => {
                let unsubscribe = result_subscription.unsubscribe().await.err();
                let mut cleanup = shutdown_context_pair(&publisher, &subscriber).await;
                if let Some(error) = unsubscribe {
                    cleanup.insert(0, format!("result unsubscribe: {error}"));
                }
                return Err(OnlinePgoStartError::Transport(format!(
                    "status subscription: {error}{}",
                    cleanup_suffix(cleanup)
                )));
            }
        };

        let mut command = SupervisedNativeCommand::new(&config.executable);
        if config.clear_environment {
            command.env_clear();
        }
        command.args(online_pgo_argv(&config, &topics));
        if let Some(cwd) = &config.cwd {
            command.current_dir(cwd);
        }
        command.envs(&config.env);
        let ready_deadline = Instant::now()
            .checked_add(config.ready_timeout)
            .expect("validated READY timeout");
        let runtime_generation = config.runtime_generation;
        let backend_generation = config.backend_generation;
        let status_channel = topics.status.clone();
        let readiness = wait_for_ready(
            &mut status_subscription,
            &status_channel,
            runtime_generation,
            backend_generation,
        );
        let started = spawn_supervised(command, config.supervisor.clone(), ready_deadline, |_| {
            readiness
        })
        .await;
        let started = match started {
            Ok(started) => started,
            Err(source) => {
                let mut cleanup = Vec::new();
                if let Err(error) = result_subscription.unsubscribe().await {
                    cleanup.push(format!("result unsubscribe: {error}"));
                }
                if let Err(error) = status_subscription.unsubscribe().await {
                    cleanup.push(format!("status unsubscribe: {error}"));
                }
                cleanup.extend(shutdown_context_pair(&publisher, &subscriber).await);
                return Err(OnlinePgoStartError::Supervision {
                    source: Box::new(source),
                    cleanup: cleanup_suffix(cleanup),
                });
            }
        };

        let (admission_tx, admission_rx) = mpsc::channel(ADMISSION_CAPACITY);
        let (control_tx, control_rx) = mpsc::channel(OWNER_COMMAND_CAPACITY);
        let admitted = Arc::new(AtomicBool::new(false));
        let runtime = OwnerRuntime {
            publisher,
            subscriber,
            result_subscription,
            status_subscription,
            child: started.child,
            topics: topics.clone(),
            runtime_generation,
            backend_generation,
            result_timeout: config.result_timeout,
            cancel_timeout: config.cancel_timeout,
            shutdown_term_grace: config.shutdown_term_grace,
            last_status_sequence: started.ready.status_sequence,
            next_request_sequence: 0,
            next_cancel_sequence: 0,
            admitted: Arc::clone(&admitted),
        };
        let task = tokio::spawn(run_owner(runtime, admission_rx, control_rx));
        Ok((
            Self {
                control_tx: Some(control_tx),
                task: Some(task),
                topics,
            },
            OnlinePgoIngress {
                tx: admission_tx,
                admitted,
            },
        ))
    }

    /// Generation-private typed topics used by this retained owner.
    #[must_use]
    pub fn topics(&self) -> &OnlinePgoTopics {
        &self.topics
    }

    /// Publish exact typed cancellation and await ACK plus request terminal.
    pub async fn cancel_in_flight(&mut self) -> Result<OnlinePgoCancelOutcome, OnlinePgoError> {
        let tx = self.control_tx.as_ref().ok_or(OnlinePgoError::Stopped)?;
        let (reply_tx, reply_rx) = oneshot::channel();
        tx.send(OwnerCommand::Cancel { reply_tx })
            .await
            .map_err(|_| OnlinePgoError::Stopped)?;
        reply_rx.await.unwrap_or(Err(OnlinePgoError::Stopped))
    }

    /// Whether the retained background owner has already terminated.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.task.as_ref().is_none_or(JoinHandle::is_finished)
    }

    /// Close admission and prove cancel, TERM/KILL/reap, unsubscribe, and join.
    pub async fn shutdown(mut self) -> Result<OnlinePgoShutdown, OnlinePgoError> {
        if let Some(tx) = self.control_tx.take() {
            let _ = tx.send(OwnerCommand::Shutdown).await;
        }
        let task = self.task.take().ok_or_else(|| {
            OnlinePgoError::OwnerTask("retained owner task was already consumed".into())
        })?;
        task.await
            .map_err(|error| OnlinePgoError::OwnerTask(error.to_string()))?
    }
}

impl Drop for OnlinePgoOwner {
    fn drop(&mut self) {
        if self.task.is_some() {
            self.control_tx.take();
            tracing::error!(
                component = "nuwax-nav-pgo-bridge",
                "online PGO owner dropped without shutdown proof; background cleanup requested"
            );
        }
    }
}

#[derive(Debug)]
struct Admission {
    request: NativePgoRequest,
    reply_tx: oneshot::Sender<Result<ValidatedOnlinePgoResult, OnlinePgoError>>,
}

#[derive(Debug)]
enum OwnerCommand {
    Cancel {
        reply_tx: oneshot::Sender<Result<OnlinePgoCancelOutcome, OnlinePgoError>>,
    },
    Shutdown,
}

#[derive(Debug)]
struct ReadyObservation {
    status_sequence: i64,
}

struct OwnerRuntime {
    publisher: LcmContext,
    subscriber: LcmContext,
    result_subscription: LcmSubscription,
    status_subscription: LcmSubscription,
    child: SupervisedNativeChild,
    topics: OnlinePgoTopics,
    runtime_generation: i64,
    backend_generation: i64,
    result_timeout: Duration,
    cancel_timeout: Duration,
    shutdown_term_grace: Duration,
    last_status_sequence: i64,
    next_request_sequence: i64,
    next_cancel_sequence: i64,
    admitted: Arc<AtomicBool>,
}

struct ActiveRequest {
    request: NativePgoRequest,
    reply_tx: oneshot::Sender<Result<ValidatedOnlinePgoResult, OnlinePgoError>>,
    deadline: Instant,
    accepted: bool,
    running: bool,
    result: Option<ValidatedOnlinePgoResult>,
    terminal: Option<NativePgoStatus>,
    cancel_requested: bool,
    cancel_deadline: Option<Instant>,
    cancel_ack: Option<OnlinePgoCancelOutcome>,
    cancel_reply: Option<oneshot::Sender<Result<OnlinePgoCancelOutcome, OnlinePgoError>>>,
}

async fn wait_for_ready(
    subscription: &mut LcmSubscription,
    expected_channel: &str,
    runtime_generation: i64,
    backend_generation: i64,
) -> Result<ReadyObservation, SupervisedReadyError> {
    let mut last_sequence = 0_i64;
    let mut saw_starting = false;
    loop {
        let received = match subscription.receiver.recv().await {
            Ok(received) => received,
            Err(broadcast::error::RecvError::Closed) => {
                return Err(SupervisedReadyError::Eof(
                    "typed status subscription closed before READY".into(),
                ));
            }
            Err(broadcast::error::RecvError::Lagged(count)) => {
                return Err(SupervisedReadyError::Protocol(format!(
                    "typed status subscription dropped {count} messages before READY"
                )));
            }
        };
        if received.channel != expected_channel {
            return Err(SupervisedReadyError::Protocol(format!(
                "READY callback channel mismatch: got {}, expected {expected_channel}",
                received.channel
            )));
        }
        let status = codec::decode_native_pgo_status_wire(&received.payload)
            .map_err(|error| SupervisedReadyError::Protocol(error.to_string()))?;
        validate_owner_identity(&status, runtime_generation, backend_generation)
            .map_err(|error| SupervisedReadyError::Protocol(error.to_string()))?;
        let expected_sequence = last_sequence.checked_add(1).ok_or_else(|| {
            SupervisedReadyError::Protocol("READY status sequence overflow".into())
        })?;
        if status.status_sequence != expected_sequence {
            return Err(SupervisedReadyError::Protocol(format!(
                "READY status sequence mismatch: got {}, expected {expected_sequence}",
                status.status_sequence
            )));
        }
        last_sequence = status.status_sequence;
        match status.event_kind {
            codec::NATIVE_PGO_EVENT_STARTING if !saw_starting && status.request.is_none() => {
                saw_starting = true;
            }
            codec::NATIVE_PGO_EVENT_READY if saw_starting && status.request.is_none() => {
                return Ok(ReadyObservation {
                    status_sequence: status.status_sequence,
                });
            }
            event => {
                return Err(SupervisedReadyError::Protocol(format!(
                    "unexpected typed status event {event} before READY"
                )));
            }
        }
    }
}

async fn run_owner(
    mut runtime: OwnerRuntime,
    mut admission_rx: mpsc::Receiver<Admission>,
    mut control_rx: mpsc::Receiver<OwnerCommand>,
) -> Result<OnlinePgoShutdown, OnlinePgoError> {
    let mut active: Option<ActiveRequest> = None;
    let mut shutting_down = false;
    let mut admission_open = true;
    let mut cause: Option<OnlinePgoStopCause> = None;
    let mut poll = tokio::time::interval(CHILD_POLL_INTERVAL);
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
        if shutting_down && active.is_none() {
            cause.get_or_insert(OnlinePgoStopCause::Requested);
            break;
        }
        let active_deadline = active.as_ref().map(|current| {
            current
                .cancel_deadline
                .map_or(current.deadline, |cancel| cancel.min(current.deadline))
        });
        tokio::select! {
            biased;
            _ = wait_for_deadline(active_deadline) => {
                let current = active
                    .as_ref()
                    .expect("active deadline exists only for an active request");
                let error = active_deadline_error(current, Instant::now()).unwrap_or(
                    OnlinePgoError::RequestTimedOut {
                        request_sequence: current.request.identity.request_sequence,
                    },
                );
                cause = Some(OnlinePgoStopCause::Terminal(error));
                break;
            }
            received = runtime.status_subscription.receiver.recv() => {
                if let Some(error) = active
                    .as_ref()
                    .and_then(|current| active_deadline_error(current, Instant::now()))
                {
                    cause = Some(OnlinePgoStopCause::Terminal(error));
                    break;
                }
                if let Err(error) = handle_status_receive(
                    received,
                    &runtime.topics.status,
                    runtime.runtime_generation,
                    runtime.backend_generation,
                    &mut runtime.last_status_sequence,
                    &mut active,
                ) {
                    cause = Some(OnlinePgoStopCause::Terminal(error));
                    break;
                }
            }
            command = control_rx.recv() => {
                match command {
                    Some(OwnerCommand::Cancel { reply_tx }) if !shutting_down => {
                        if let Some(current) = active.as_mut() {
                            if current.cancel_requested {
                                let _ = reply_tx.send(Err(OnlinePgoError::Busy));
                            } else {
                                current.cancel_reply = Some(reply_tx);
                                if let Err(error) = publish_cancel(&mut runtime, current).await {
                                    cause = Some(OnlinePgoStopCause::Terminal(error));
                                    break;
                                }
                            }
                        } else {
                            let _ = reply_tx.send(Ok(OnlinePgoCancelOutcome::NoInFlight));
                        }
                    }
                    Some(OwnerCommand::Cancel { reply_tx }) => {
                        let _ = reply_tx.send(Err(OnlinePgoError::Stopped));
                    }
                    Some(OwnerCommand::Shutdown) | None => {
                        shutting_down = true;
                        admission_rx.close();
                        reject_queued_admissions(&mut admission_rx, &runtime.admitted);
                        if let Some(current) = active.as_mut() {
                            if !current.cancel_requested {
                                if let Err(error) = publish_cancel(&mut runtime, current).await {
                                    cause = Some(OnlinePgoStopCause::Terminal(error));
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            admission = admission_rx.recv(), if !shutting_down && admission_open => {
                match admission {
                    Some(admission) => {
                        if active.is_some() {
                            runtime.admitted.store(false, Ordering::Release);
                            let _ = admission.reply_tx.send(Err(OnlinePgoError::Busy));
                            continue;
                        }
                        match admit_request(&mut runtime, admission).await {
                            Ok(current) => active = Some(current),
                            Err((error, reply_tx)) => {
                                runtime.admitted.store(false, Ordering::Release);
                                let terminal = matches!(
                                    error,
                                    OnlinePgoError::Transport(_)
                                        | OnlinePgoError::RequestTimedOut { .. }
                                );
                                let _ = reply_tx.send(Err(error.clone()));
                                if terminal {
                                    cause = Some(OnlinePgoStopCause::Terminal(error));
                                    break;
                                }
                            }
                        }
                    }
                    None => {
                        admission_open = false;
                    }
                }
            }
            received = runtime.result_subscription.receiver.recv() => {
                if let Some(error) = active
                    .as_ref()
                    .and_then(|current| active_deadline_error(current, Instant::now()))
                {
                    cause = Some(OnlinePgoStopCause::Terminal(error));
                    break;
                }
                if let Err(error) = handle_result_receive(
                    received,
                    &runtime.topics.result,
                    &mut active,
                ) {
                    cause = Some(OnlinePgoStopCause::Terminal(error));
                    break;
                }
            }
            _ = poll.tick() => {
                match runtime.child.is_finished() {
                    Ok(true) => {
                        cause = Some(OnlinePgoStopCause::Terminal(match runtime.child.wait().await {
                            Ok(terminal) => OnlinePgoError::ChildTerminated(terminal),
                            Err(error) => OnlinePgoError::Supervision(error.to_string()),
                        }));
                        break;
                    }
                    Ok(false) => {}
                    Err(error) => {
                        cause = Some(OnlinePgoStopCause::Terminal(
                            OnlinePgoError::Supervision(error.to_string()),
                        ));
                        break;
                    }
                }
            }
        }

        match try_finish_active(&mut active, &runtime.admitted) {
            Ok(true) if shutting_down => {
                cause.get_or_insert(OnlinePgoStopCause::Requested);
                break;
            }
            Ok(_) => {}
            Err(error) => {
                cause = Some(OnlinePgoStopCause::Terminal(error));
                break;
            }
        }
    }

    admission_rx.close();
    reject_queued_admissions(&mut admission_rx, &runtime.admitted);
    let cause = cause.unwrap_or(OnlinePgoStopCause::Requested);
    if let Some(current) = active.take() {
        let error = match &cause {
            OnlinePgoStopCause::Terminal(error) => error.clone(),
            OnlinePgoStopCause::Requested => OnlinePgoError::Stopped,
        };
        fail_active(current, error, &runtime.admitted);
    }
    let term_deadline = Instant::now()
        .checked_add(runtime.shutdown_term_grace)
        .expect("validated shutdown grace");
    let process = runtime.child.terminate(term_deadline).await;
    let logs = runtime.child.logs();
    let cleanup = cleanup_runtime_transport(&mut runtime).await;
    let process = match process {
        Ok(process) => process,
        Err(error) => {
            let cleanup_detail = cleanup_suffix(cleanup);
            return Err(OnlinePgoError::Supervision(format!(
                "{error}{cleanup_detail}"
            )));
        }
    };
    if !cleanup.is_empty() {
        return Err(OnlinePgoError::Cleanup(cleanup.join("; ")));
    }
    Ok(OnlinePgoShutdown {
        cause,
        process,
        logs,
    })
}

async fn wait_for_deadline(deadline: Option<Instant>) {
    match deadline {
        Some(deadline) => tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)).await,
        None => std::future::pending::<()>().await,
    }
}

fn active_deadline_error(current: &ActiveRequest, now: Instant) -> Option<OnlinePgoError> {
    if current
        .cancel_deadline
        .is_some_and(|deadline| now >= deadline)
    {
        Some(OnlinePgoError::CancelTimedOut {
            request_sequence: current.request.identity.request_sequence,
        })
    } else if now >= current.deadline {
        Some(OnlinePgoError::RequestTimedOut {
            request_sequence: current.request.identity.request_sequence,
        })
    } else {
        None
    }
}

async fn admit_request(
    runtime: &mut OwnerRuntime,
    admission: Admission,
) -> Result<
    ActiveRequest,
    (
        OnlinePgoError,
        oneshot::Sender<Result<ValidatedOnlinePgoResult, OnlinePgoError>>,
    ),
> {
    let next_sequence = match runtime.next_request_sequence.checked_add(1) {
        Some(sequence) => sequence,
        None => {
            return Err((
                OnlinePgoError::InvalidRequest("request sequence exhausted".into()),
                admission.reply_tx,
            ));
        }
    };
    let prepared = match prepare_request(
        admission.request,
        runtime.runtime_generation,
        runtime.backend_generation,
        next_sequence,
    ) {
        Ok(prepared) => prepared,
        Err(error) => return Err((error, admission.reply_tx)),
    };
    let wire = match codec::encode_native_pgo_request_wire(&prepared) {
        Ok(wire) => wire,
        Err(error) => {
            return Err((
                OnlinePgoError::InvalidRequest(error.to_string()),
                admission.reply_tx,
            ));
        }
    };
    let deadline = Instant::now()
        .checked_add(runtime.result_timeout)
        .expect("validated result timeout");
    match tokio::time::timeout_at(
        tokio::time::Instant::from_std(deadline),
        runtime
            .publisher
            .publish(&runtime.topics.request, Bytes::from(wire)),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(error)) => {
            return Err((
                OnlinePgoError::Transport(format!("request publish: {error}")),
                admission.reply_tx,
            ));
        }
        Err(_) => {
            return Err((
                OnlinePgoError::RequestTimedOut {
                    request_sequence: next_sequence,
                },
                admission.reply_tx,
            ));
        }
    }
    runtime.next_request_sequence = next_sequence;
    Ok(ActiveRequest {
        request: prepared,
        reply_tx: admission.reply_tx,
        deadline,
        accepted: false,
        running: false,
        result: None,
        terminal: None,
        cancel_requested: false,
        cancel_deadline: None,
        cancel_ack: None,
        cancel_reply: None,
    })
}

fn prepare_request(
    mut request: NativePgoRequest,
    runtime_generation: i64,
    backend_generation: i64,
    request_sequence: i64,
) -> Result<NativePgoRequest, OnlinePgoError> {
    if request.schema_version != codec::NATIVE_PGO_SCHEMA_VERSION_V3
        || request.identity.backend != codec::NATIVE_PGO_BACKEND
        || request.identity.runtime_generation != runtime_generation
        || request.identity.backend_generation != backend_generation
    {
        return Err(OnlinePgoError::InvalidRequest(
            "schema, backend, or owner generation mismatch".into(),
        ));
    }
    let expected_coordinate_count = request
        .identity
        .registered_cloud
        .point_count
        .checked_mul(3)
        .and_then(|value| i32::try_from(value).ok());
    if expected_coordinate_count != Some(request.registered_body_coordinate_count)
        || usize::try_from(request.registered_body_coordinate_count).ok()
            != Some(request.registered_body_points_xyz.len())
    {
        return Err(OnlinePgoError::InvalidRequest(
            "registered body point/count receipt mismatch".into(),
        ));
    }
    request.identity.request_sequence = request_sequence;
    codec::seal_native_pgo_request(request)
        .map_err(|error| OnlinePgoError::InvalidRequest(error.to_string()))
}

async fn publish_cancel(
    runtime: &mut OwnerRuntime,
    current: &mut ActiveRequest,
) -> Result<(), OnlinePgoError> {
    runtime.next_cancel_sequence = runtime
        .next_cancel_sequence
        .checked_add(1)
        .ok_or_else(|| OnlinePgoError::Protocol("cancel status sequence exhausted".into()))?;
    let status = NativePgoStatus {
        schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
        backend: codec::NATIVE_PGO_BACKEND,
        runtime_generation: runtime.runtime_generation,
        backend_generation: runtime.backend_generation,
        status_sequence: runtime.next_cancel_sequence,
        status_timestamp_ns: timestamp_now_ns(),
        request: Some(current.request.echo()),
        event_kind: codec::NATIVE_PGO_EVENT_CANCEL_REQUESTED,
        health_state: codec::NATIVE_PGO_HEALTH_UNKNOWN,
        terminal_state: codec::NATIVE_PGO_TERMINAL_NONE,
        cancel_state: codec::NATIVE_PGO_CANCEL_REQUESTED,
        reason_code: codec::NATIVE_PGO_REASON_CANCEL_REQUESTED,
        reason: "cancel_requested".into(),
    };
    let status = codec::seal_native_pgo_status(status)
        .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
    let wire = codec::encode_native_pgo_status_wire(&status)
        .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
    let cancel_deadline = Instant::now()
        .checked_add(runtime.cancel_timeout)
        .expect("validated cancel timeout");
    match tokio::time::timeout_at(
        tokio::time::Instant::from_std(cancel_deadline),
        runtime
            .publisher
            .publish(&runtime.topics.cancel, Bytes::from(wire)),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(error)) => {
            return Err(OnlinePgoError::Transport(format!(
                "cancel publish: {error}"
            )));
        }
        Err(_) => {
            return Err(OnlinePgoError::CancelTimedOut {
                request_sequence: current.request.identity.request_sequence,
            });
        }
    }
    current.cancel_requested = true;
    current.cancel_deadline = Some(cancel_deadline);
    Ok(())
}

fn handle_result_receive(
    received: Result<LcmReceivedMessage, broadcast::error::RecvError>,
    expected_channel: &str,
    active: &mut Option<ActiveRequest>,
) -> Result<(), OnlinePgoError> {
    let received = map_receive(received, "result")?;
    if received.channel != expected_channel {
        return Err(OnlinePgoError::Protocol(format!(
            "result callback channel mismatch: got {}, expected {expected_channel}",
            received.channel
        )));
    }
    let result = codec::decode_native_pgo_result_wire(&received.payload)
        .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
    let current = active
        .as_mut()
        .ok_or_else(|| OnlinePgoError::Protocol("late result without active request".into()))?;
    if !current.accepted || !current.running {
        return Err(OnlinePgoError::Protocol(
            "result arrived before REQUEST_ACCEPTED and RUNNING".into(),
        ));
    }
    if current.result.is_some() {
        return Err(OnlinePgoError::Protocol(
            "duplicate result for active request".into(),
        ));
    }
    current.result = Some(ValidatedOnlinePgoResult::from_result(
        &current.request,
        result,
    )?);
    Ok(())
}

fn handle_status_receive(
    received: Result<LcmReceivedMessage, broadcast::error::RecvError>,
    expected_channel: &str,
    runtime_generation: i64,
    backend_generation: i64,
    last_status_sequence: &mut i64,
    active: &mut Option<ActiveRequest>,
) -> Result<(), OnlinePgoError> {
    let received = map_receive(received, "status")?;
    if received.channel != expected_channel {
        return Err(OnlinePgoError::Protocol(format!(
            "status callback channel mismatch: got {}, expected {expected_channel}",
            received.channel
        )));
    }
    let status = codec::decode_native_pgo_status_wire(&received.payload)
        .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
    validate_owner_identity(&status, runtime_generation, backend_generation)?;
    let expected_sequence = last_status_sequence
        .checked_add(1)
        .ok_or_else(|| OnlinePgoError::Protocol("status sequence exhausted".into()))?;
    if status.status_sequence != expected_sequence {
        return Err(OnlinePgoError::Protocol(format!(
            "duplicate, stale, or skipped status sequence: got {}, expected {expected_sequence}",
            status.status_sequence
        )));
    }
    *last_status_sequence = status.status_sequence;
    if status.event_kind == codec::NATIVE_PGO_EVENT_PROTOCOL_ERROR {
        return Err(OnlinePgoError::Protocol(format!(
            "native owner protocol error {}: {}",
            status.reason_code, status.reason
        )));
    }
    if status.request.is_none() {
        return match (status.event_kind, status.health_state) {
            (codec::NATIVE_PGO_EVENT_HEALTH, codec::NATIVE_PGO_HEALTH_HEALTHY) => Ok(()),
            (
                codec::NATIVE_PGO_EVENT_HEALTH,
                health_state @ (codec::NATIVE_PGO_HEALTH_DEGRADED
                | codec::NATIVE_PGO_HEALTH_UNHEALTHY),
            ) => Err(OnlinePgoError::OwnerHealth {
                health_state,
                reason_code: status.reason_code,
                reason: status.reason,
            }),
            event => Err(OnlinePgoError::Protocol(format!(
                "unexpected owner-scoped status event/health {event:?} after READY"
            ))),
        };
    }
    let current = active.as_mut().ok_or_else(|| {
        OnlinePgoError::Protocol("late request-scoped status without active request".into())
    })?;
    codec::validate_native_pgo_status_echo(&current.request, &status)
        .map_err(|error| OnlinePgoError::Protocol(error.to_string()))?;
    match status.event_kind {
        codec::NATIVE_PGO_EVENT_REQUEST_ACCEPTED
            if !current.accepted && !current.running && current.terminal.is_none() =>
        {
            current.accepted = true;
            Ok(())
        }
        codec::NATIVE_PGO_EVENT_RUNNING
            if current.accepted && !current.running && current.terminal.is_none() =>
        {
            current.running = true;
            Ok(())
        }
        codec::NATIVE_PGO_EVENT_TERMINAL
            if current.accepted && current.running && current.terminal.is_none() =>
        {
            current.terminal = Some(status);
            Ok(())
        }
        codec::NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED
            if current.cancel_requested
                && current.accepted
                && current.running
                && current.cancel_ack.is_none() =>
        {
            let outcome = match status.cancel_state {
                codec::NATIVE_PGO_CANCEL_ACKNOWLEDGED => OnlinePgoCancelOutcome::Cancelled,
                codec::NATIVE_PGO_CANCEL_TOO_LATE => OnlinePgoCancelOutcome::TooLate,
                other => {
                    return Err(OnlinePgoError::Protocol(format!(
                        "invalid decoded cancel ACK state {other}"
                    )));
                }
            };
            if outcome == OnlinePgoCancelOutcome::Cancelled {
                if current.terminal.is_some() {
                    return Err(OnlinePgoError::Protocol(
                        "cancel ACK arrived after a separate terminal status".into(),
                    ));
                }
                current.terminal = Some(status);
            }
            current.cancel_ack = Some(outcome);
            Ok(())
        }
        event => Err(OnlinePgoError::Protocol(format!(
            "duplicate or out-of-order request status event {event}"
        ))),
    }
}

fn try_finish_active(
    active: &mut Option<ActiveRequest>,
    admitted: &AtomicBool,
) -> Result<bool, OnlinePgoError> {
    let Some(current) = active.as_ref() else {
        return Ok(false);
    };
    let (Some(result), Some(terminal)) = (&current.result, &current.terminal) else {
        return Ok(false);
    };
    if current.cancel_requested && current.cancel_ack.is_none() {
        return Ok(false);
    }
    validate_result_terminal_matrix(result, terminal, current.cancel_ack)?;
    let mut current = active.take().expect("active request checked above");
    let result = current.result.take().expect("result checked above");
    if let (Some(reply), Some(outcome)) = (current.cancel_reply.take(), current.cancel_ack) {
        let _ = reply.send(Ok(outcome));
    }
    let _ = current.reply_tx.send(Ok(result));
    admitted.store(false, Ordering::Release);
    Ok(true)
}

fn validate_result_terminal_matrix(
    result: &ValidatedOnlinePgoResult,
    terminal: &NativePgoStatus,
    cancel_ack: Option<OnlinePgoCancelOutcome>,
) -> Result<(), OnlinePgoError> {
    let expected_terminal = match result.disposition() {
        ValidatedOnlinePgoDisposition::NoCandidate | ValidatedOnlinePgoDisposition::Correction => {
            codec::NATIVE_PGO_TERMINAL_SUCCEEDED
        }
        ValidatedOnlinePgoDisposition::Rejected
            if result.result().rejection_reason_code == codec::NATIVE_PGO_REJECTION_CANCELLED =>
        {
            codec::NATIVE_PGO_TERMINAL_CANCELLED
        }
        ValidatedOnlinePgoDisposition::Rejected => codec::NATIVE_PGO_TERMINAL_REJECTED,
    };
    if terminal.terminal_state != expected_terminal {
        return Err(OnlinePgoError::Protocol(format!(
            "result/terminal mismatch: disposition={}, terminal={}, expected={expected_terminal}",
            result.result().result_disposition,
            terminal.terminal_state
        )));
    }
    match cancel_ack {
        Some(OnlinePgoCancelOutcome::Cancelled) => {
            if expected_terminal != codec::NATIVE_PGO_TERMINAL_CANCELLED
                || terminal.event_kind != codec::NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED
            {
                return Err(OnlinePgoError::Protocol(
                    "cancel ACKNOWLEDGED must be the sole cancelled terminal proof".into(),
                ));
            }
            Ok(())
        }
        Some(OnlinePgoCancelOutcome::TooLate) => {
            if expected_terminal == codec::NATIVE_PGO_TERMINAL_CANCELLED
                || terminal.event_kind != codec::NATIVE_PGO_EVENT_TERMINAL
            {
                return Err(OnlinePgoError::Protocol(
                    "cancel TOO_LATE must retain the normal result/terminal pair".into(),
                ));
            }
            Ok(())
        }
        Some(OnlinePgoCancelOutcome::NoInFlight) => Err(OnlinePgoError::Protocol(
            "NoInFlight cannot be attached to an active request".into(),
        )),
        None => {
            if expected_terminal == codec::NATIVE_PGO_TERMINAL_CANCELLED
                || terminal.event_kind != codec::NATIVE_PGO_EVENT_TERMINAL
            {
                return Err(OnlinePgoError::Protocol(
                    "normal completion requires one non-cancelled EVENT_TERMINAL".into(),
                ));
            }
            Ok(())
        }
    }
}

fn validate_owner_identity(
    status: &NativePgoStatus,
    runtime_generation: i64,
    backend_generation: i64,
) -> Result<(), OnlinePgoError> {
    if status.schema_version != codec::NATIVE_PGO_SCHEMA_VERSION_V3
        || status.backend != codec::NATIVE_PGO_BACKEND
        || status.runtime_generation != runtime_generation
        || status.backend_generation != backend_generation
    {
        return Err(OnlinePgoError::Protocol(
            "status schema/backend/generation identity mismatch".into(),
        ));
    }
    Ok(())
}

fn fail_active(current: ActiveRequest, error: OnlinePgoError, admitted: &AtomicBool) {
    if let Some(reply) = current.cancel_reply {
        let _ = reply.send(Err(error.clone()));
    }
    let _ = current.reply_tx.send(Err(error));
    admitted.store(false, Ordering::Release);
}

fn reject_queued_admissions(admission_rx: &mut mpsc::Receiver<Admission>, admitted: &AtomicBool) {
    while let Ok(admission) = admission_rx.try_recv() {
        let _ = admission.reply_tx.send(Err(OnlinePgoError::Stopped));
        admitted.store(false, Ordering::Release);
    }
}

fn map_receive(
    received: Result<LcmReceivedMessage, broadcast::error::RecvError>,
    phase: &str,
) -> Result<LcmReceivedMessage, OnlinePgoError> {
    match received {
        Ok(message) => Ok(message),
        Err(broadcast::error::RecvError::Closed) => Err(OnlinePgoError::Transport(format!(
            "{phase} subscription reached EOF"
        ))),
        Err(broadcast::error::RecvError::Lagged(count)) => Err(OnlinePgoError::Protocol(format!(
            "{phase} subscription dropped {count} typed messages"
        ))),
    }
}

async fn cleanup_runtime_transport(runtime: &mut OwnerRuntime) -> Vec<String> {
    let mut errors = Vec::new();
    if let Err(error) = runtime.result_subscription.unsubscribe().await {
        errors.push(format!("result unsubscribe: {error}"));
    }
    if let Err(error) = runtime.status_subscription.unsubscribe().await {
        errors.push(format!("status unsubscribe: {error}"));
    }
    errors.extend(shutdown_context_pair(&runtime.publisher, &runtime.subscriber).await);
    errors
}

async fn shutdown_context_pair(publisher: &LcmContext, subscriber: &LcmContext) -> Vec<String> {
    let mut errors = Vec::new();
    if let Err(error) = subscriber.shutdown().await {
        errors.push(format!("subscriber context shutdown: {error}"));
    }
    if let Err(error) = publisher.shutdown().await {
        errors.push(format!("publisher context shutdown: {error}"));
    }
    errors
}

fn cleanup_suffix(errors: Vec<String>) -> String {
    if errors.is_empty() {
        String::new()
    } else {
        format!("; cleanup failed: {}", errors.join("; "))
    }
}

fn timestamp_now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .ok()
        .and_then(|duration| i64::try_from(duration.as_nanos()).ok())
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU32;

    static NEXT_TEST_PORT: AtomicU32 = AtomicU32::new(0);

    fn test_provider() -> String {
        let offset = NEXT_TEST_PORT.fetch_add(1, Ordering::Relaxed);
        let process = std::process::id();
        let port = 20_000 + ((process + offset) % 20_000);
        format!("udpm://239.255.76.67:{port}?ttl=1")
    }

    fn test_config(runtime_generation: i64, backend_generation: i64) -> OnlinePgoConfig {
        let mut config =
            OnlinePgoConfig::new(test_provider(), runtime_generation, backend_generation);
        config.executable = crate::native_executable();
        config.ready_timeout = Duration::from_secs(5);
        config.result_timeout = Duration::from_secs(5);
        config.cancel_timeout = Duration::from_secs(2);
        config.shutdown_term_grace = Duration::from_millis(500);
        config
    }

    fn complete_request(
        runtime_generation: i64,
        backend_generation: i64,
        mapping_sequence: i64,
        source_timestamp_ns: i64,
    ) -> NativePgoRequest {
        let mut points = Vec::new();
        for index in 0..24 {
            let index = f64::from(index);
            points.extend_from_slice(&[index * 0.2, (index % 5.0) * 0.21, index * 0.03]);
        }
        let point_count = i64::try_from(points.len() / 3).unwrap();
        let request = NativePgoRequest {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            identity: codec::NativePgoRequestIdentity {
                backend: codec::NATIVE_PGO_BACKEND,
                runtime_generation,
                backend_generation,
                request_sequence: 91,
                mapping_session_id: "mapping-session-a".into(),
                mapping_session_generation: 3,
                mapping_backend_kind: "voxel".into(),
                mapping_backend_generation: 17,
                map_frame_generation: 4,
                mapping_commit_id: codec::NativePgoMappingCommitId {
                    boot_nonce: [0x11; 16],
                    sequence: mapping_sequence,
                },
                occupancy_revision: mapping_sequence,
                cloud_revision: mapping_sequence,
                registered_cloud: codec::NativePgoRegisteredCloudProof {
                    backend: 2,
                    runtime_generation: 23,
                    source_point_count: point_count,
                    source_sequence: mapping_sequence,
                    source_timestamp_ns,
                    point_count,
                },
                correlated_odom: codec::NativePgoCorrelatedOdomProof {
                    translation_m: [0.0, 0.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    source_sequence: mapping_sequence,
                    source_timestamp_ns,
                },
                committed_map: codec::NativePgoCommittedMapProof {
                    source_timestamp_ns,
                    width: 2,
                    height: 2,
                    resolution_m: 0.1,
                    origin_x_m: -0.1,
                    origin_y_m: -0.1,
                },
                source_timestamp_ns,
                map_frame_id: "map".into(),
                odom_frame_id: "odom".into(),
                body_frame_id: "base_link".into(),
                points_frame_id: "base_link".into(),
                correction_lease: codec::NativePgoCorrectionLeaseId {
                    boot_nonce: [0x33; 16],
                    generation: 5,
                },
            },
            registered_body_coordinate_count: i32::try_from(points.len()).unwrap(),
            registered_body_points_xyz: points,
        };
        let request = codec::seal_native_pgo_request(request).unwrap();
        codec::encode_native_pgo_request_wire(&request).unwrap();
        request
    }

    fn no_candidate(request: &NativePgoRequest) -> NativePgoResult {
        codec::seal_native_pgo_result(NativePgoResult {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            request: request.echo(),
            result_disposition: codec::NATIVE_PGO_DISPOSITION_NO_CANDIDATE,
            result_timestamp_ns: request.identity.source_timestamp_ns,
            rejection_reason_code: codec::NATIVE_PGO_REJECTION_NONE,
            ..Default::default()
        })
        .unwrap()
    }

    fn with_odom_translation(
        mut request: NativePgoRequest,
        translation_m: [f64; 3],
    ) -> NativePgoRequest {
        request.identity.correlated_odom.translation_m = translation_m;
        codec::seal_native_pgo_request(request).unwrap()
    }

    fn with_dense_cloud(mut request: NativePgoRequest, point_count: usize) -> NativePgoRequest {
        let side = (point_count as f64).sqrt().ceil() as usize;
        let mut points = Vec::with_capacity(point_count * 3);
        for index in 0..point_count {
            let x = (index % side) as f64 * 0.11;
            let y = (index / side) as f64 * 0.11;
            let z = ((index * 17) % 29) as f64 * 0.013;
            points.extend_from_slice(&[x, y, z]);
        }
        request.identity.registered_cloud.point_count = i64::try_from(point_count).unwrap();
        request.identity.registered_cloud.source_point_count = i64::try_from(point_count).unwrap();
        request.registered_body_coordinate_count = i32::try_from(points.len()).unwrap();
        request.registered_body_points_xyz = points;
        codec::seal_native_pgo_request(request).unwrap()
    }

    fn correction(request: &NativePgoRequest) -> NativePgoResult {
        codec::seal_native_pgo_result(NativePgoResult {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            request: request.echo(),
            result_disposition: codec::NATIVE_PGO_DISPOSITION_CORRECTION,
            result_timestamp_ns: request.identity.source_timestamp_ns,
            transform_semantics: codec::NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3,
            translation_m: [0.1, -0.2, 0.3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            fitness_score: 0.01,
            inlier_rmse_m: 0.02,
            loop_source_node_id: 2,
            loop_target_node_id: 0,
            loop_inlier_count: 24,
            trajectory: (0..3)
                .map(|i| codec::OptimizedKeyframe {
                    source_timestamp_ns: request.identity.source_timestamp_ns - 2 + i,
                    translation_m: [i as f64, 0.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                })
                .collect(),
            optimized_graph_node_count: 3,
            optimized_graph_edge_count: 3,
            optimized_graph_loop_edge_count: 1,
            optimization_iterations: 2,
            optimization_converged: 1,
            optimization_initial_cost: 2.0,
            optimization_final_cost: 1.0,
            rejection_reason_code: codec::NATIVE_PGO_REJECTION_NONE,
            ..Default::default()
        })
        .unwrap()
    }

    fn rejected(request: &NativePgoRequest, cancelled: bool) -> NativePgoResult {
        codec::seal_native_pgo_result(NativePgoResult {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            request: request.echo(),
            result_disposition: codec::NATIVE_PGO_DISPOSITION_REJECTED,
            result_timestamp_ns: request.identity.source_timestamp_ns,
            rejection_reason_code: if cancelled {
                codec::NATIVE_PGO_REJECTION_CANCELLED
            } else {
                codec::NATIVE_PGO_REJECTION_INVALID_GRAPH
            },
            rejection_reason: if cancelled {
                "cancelled"
            } else {
                "invalid graph"
            }
            .into(),
            ..Default::default()
        })
        .unwrap()
    }

    fn request_status(
        request: &NativePgoRequest,
        status_sequence: i64,
        event_kind: i32,
        terminal_state: i32,
        cancel_state: i32,
        reason_code: i32,
        reason: &str,
    ) -> NativePgoStatus {
        codec::seal_native_pgo_status(NativePgoStatus {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            backend: codec::NATIVE_PGO_BACKEND,
            runtime_generation: request.identity.runtime_generation,
            backend_generation: request.identity.backend_generation,
            status_sequence,
            status_timestamp_ns: request.identity.source_timestamp_ns,
            request: Some(request.echo()),
            event_kind,
            health_state: codec::NATIVE_PGO_HEALTH_HEALTHY,
            terminal_state,
            cancel_state,
            reason_code,
            reason: reason.into(),
        })
        .unwrap()
    }

    fn owner_health_status(
        runtime_generation: i64,
        backend_generation: i64,
        status_sequence: i64,
        health_state: i32,
    ) -> NativePgoStatus {
        codec::seal_native_pgo_status(NativePgoStatus {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            backend: codec::NATIVE_PGO_BACKEND,
            runtime_generation,
            backend_generation,
            status_sequence,
            status_timestamp_ns: 1,
            request: None,
            event_kind: codec::NATIVE_PGO_EVENT_HEALTH,
            health_state,
            terminal_state: codec::NATIVE_PGO_TERMINAL_NONE,
            cancel_state: codec::NATIVE_PGO_CANCEL_NONE,
            reason_code: codec::NATIVE_PGO_REASON_TRANSPORT_ERROR,
            reason: "injected_owner_health".into(),
        })
        .unwrap()
    }

    fn cancel_requested_status(
        request: &NativePgoRequest,
        status_sequence: i64,
    ) -> NativePgoStatus {
        codec::seal_native_pgo_status(NativePgoStatus {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            backend: codec::NATIVE_PGO_BACKEND,
            runtime_generation: request.identity.runtime_generation,
            backend_generation: request.identity.backend_generation,
            status_sequence,
            status_timestamp_ns: request.identity.source_timestamp_ns,
            request: Some(request.echo()),
            event_kind: codec::NATIVE_PGO_EVENT_CANCEL_REQUESTED,
            health_state: codec::NATIVE_PGO_HEALTH_UNKNOWN,
            terminal_state: codec::NATIVE_PGO_TERMINAL_NONE,
            cancel_state: codec::NATIVE_PGO_CANCEL_REQUESTED,
            reason_code: codec::NATIVE_PGO_REASON_CANCEL_REQUESTED,
            reason: "cancel_requested".into(),
        })
        .unwrap()
    }

    async fn next_audit_message(
        subscription: &mut LcmSubscription,
        label: &str,
    ) -> LcmReceivedMessage {
        tokio::time::timeout(Duration::from_secs(3), subscription.receiver.recv())
            .await
            .unwrap_or_else(|_| panic!("timed out auditing {label}"))
            .unwrap_or_else(|error| panic!("audit subscription for {label} failed: {error}"))
    }

    fn received_status(channel: &str, status: &NativePgoStatus) -> LcmReceivedMessage {
        LcmReceivedMessage {
            channel: channel.into(),
            payload: Bytes::from(codec::encode_native_pgo_status_wire(status).unwrap()),
        }
    }

    fn received_result(channel: &str, result: &NativePgoResult) -> LcmReceivedMessage {
        LcmReceivedMessage {
            channel: channel.into(),
            payload: Bytes::from(codec::encode_native_pgo_result_wire(result).unwrap()),
        }
    }

    fn active_request(
        request: NativePgoRequest,
    ) -> (
        ActiveRequest,
        oneshot::Receiver<Result<ValidatedOnlinePgoResult, OnlinePgoError>>,
    ) {
        let (reply_tx, reply_rx) = oneshot::channel();
        (
            ActiveRequest {
                request,
                reply_tx,
                deadline: Instant::now() + Duration::from_secs(1),
                accepted: false,
                running: false,
                result: None,
                terminal: None,
                cancel_requested: false,
                cancel_deadline: None,
                cancel_ack: None,
                cancel_reply: None,
            },
            reply_rx,
        )
    }

    #[test]
    fn topics_and_frozen_argv_are_generation_private_and_noncanonical() {
        let config = test_config(41, 7);
        let topics = config.validate().unwrap();
        assert_eq!(
            topics.request,
            "/c/n/41/b/7/request#nuwax_msgs.NativePgoRequest"
        );
        assert_eq!(
            topics.result,
            "/c/n/41/b/7/result#nuwax_msgs.NativePgoResult"
        );
        assert_eq!(
            topics.status,
            "/c/n/41/b/7/status#nuwax_msgs.NativePgoStatus"
        );
        assert_eq!(
            topics.cancel,
            "/c/n/41/b/7/cancel#nuwax_msgs.NativePgoStatus"
        );
        let restarted_topics = OnlinePgoTopics::for_runtime(41, 8).unwrap();
        assert_ne!(topics, restarted_topics);
        let argv = online_pgo_argv(&config, &topics);
        assert_eq!(argv.len(), 16);
        assert_eq!(&argv[0..2], ["--mode", "online"]);
        assert!(argv
            .windows(2)
            .any(|pair| { pair == ["--request-channel", topics.request.as_str()] }));
        assert!(!argv.iter().any(|arg| {
            matches!(arg.as_str(), "/tf" | "/pgo_tf" | "/global_map") || arg.contains("canonical")
        }));
    }

    #[test]
    fn native_argv_rejects_cross_generation_topic_sets() {
        let config = test_config(42, 7);
        for foreign_topics in [
            OnlinePgoTopics::for_runtime(42, 6).unwrap(),
            OnlinePgoTopics::for_runtime(43, 7).unwrap(),
        ] {
            let output = Command::new("timeout")
                .args(["--signal=TERM", "--kill-after=1s", "2s"])
                .arg(&config.executable)
                .args(online_pgo_argv(&config, &foreign_topics))
                .output()
                .unwrap();
            assert_eq!(output.status.code(), Some(2));
            assert!(String::from_utf8_lossy(&output.stderr)
                .contains("runtime/backend generation-private contract"));
        }
    }

    #[test]
    fn preflight_fences_restart_generation_and_invalid_deadlines() {
        let mut config = test_config(9, 4);
        config.previous_backend_generation = Some(0);
        assert!(matches!(
            config.validate(),
            Err(OnlinePgoStartError::Preflight(message))
                if message.contains("previous_backend_generation")
        ));
        config.previous_backend_generation = Some(4);
        assert!(matches!(
            config.validate(),
            Err(OnlinePgoStartError::Preflight(message))
                if message.contains("advance across restart")
        ));
        config.previous_backend_generation = Some(3);
        config.ready_timeout = Duration::ZERO;
        assert!(matches!(
            config.validate(),
            Err(OnlinePgoStartError::Preflight(message))
                if message.contains("ready_timeout")
        ));
        config.ready_timeout = Duration::from_secs(1);
        config.lcm_url = "udpm://bad provider".into();
        assert!(matches!(
            config.validate(),
            Err(OnlinePgoStartError::Preflight(message))
                if message.contains("whitespace/control")
        ));
    }

    #[tokio::test]
    async fn capacity_one_gate_survives_a_dropped_submit_future() {
        let (tx, mut rx) = mpsc::channel(ADMISSION_CAPACITY);
        let admitted = Arc::new(AtomicBool::new(false));
        let mut ingress = OnlinePgoIngress {
            tx,
            admitted: Arc::clone(&admitted),
        };
        let mut first = Box::pin(ingress.submit(complete_request(10, 1, 1, 1)));
        let admission = tokio::select! {
            admission = rx.recv() => admission.expect("first admission"),
            result = &mut first => panic!("submit completed before owner reply: {result:?}"),
        };
        drop(first);
        assert!(admitted.load(Ordering::Acquire));
        assert_eq!(
            ingress
                .submit(complete_request(10, 1, 2, 2))
                .await
                .unwrap_err(),
            OnlinePgoError::Busy
        );
        let _ = admission.reply_tx.send(Err(OnlinePgoError::Stopped));
    }

    #[test]
    fn request_and_cancel_deadlines_are_exact_and_cancel_takes_precedence() {
        let request = prepare_request(complete_request(10, 2, 1, 1), 10, 2, 17).unwrap();
        let (mut current, _reply_rx) = active_request(request);
        let now = Instant::now();
        current.deadline = now;
        assert_eq!(
            active_deadline_error(&current, now),
            Some(OnlinePgoError::RequestTimedOut {
                request_sequence: 17,
            })
        );
        current.cancel_deadline = Some(now);
        assert_eq!(
            active_deadline_error(&current, now),
            Some(OnlinePgoError::CancelTimedOut {
                request_sequence: 17,
            })
        );
    }

    #[test]
    fn owner_assigns_monotonic_sequence_and_validates_complete_request() {
        let request = complete_request(11, 2, 1, 1_000_000_000);
        let prepared = prepare_request(request.clone(), 11, 2, 1).unwrap();
        assert_eq!(prepared.identity.request_sequence, 1);
        assert_ne!(
            prepared.identity.request_sequence,
            request.identity.request_sequence
        );
        assert_eq!(
            prepared.registered_body_points_xyz,
            request.registered_body_points_xyz
        );
        let wire = codec::encode_native_pgo_request_wire(&prepared).unwrap();
        assert_eq!(
            codec::decode_native_pgo_request_wire(&wire).unwrap(),
            prepared
        );

        let mut hostile = request;
        hostile.registered_body_coordinate_count += 3;
        assert!(matches!(
            prepare_request(hostile, 11, 2, 2),
            Err(OnlinePgoError::InvalidRequest(message))
                if message.contains("point/count receipt mismatch")
        ));
    }

    #[test]
    fn result_classification_validates_all_three_dispositions_and_finite_proofs() {
        let request = prepare_request(complete_request(12, 3, 1, 2_000_000_000), 12, 3, 1).unwrap();
        assert_eq!(
            ValidatedOnlinePgoResult::from_result(&request, no_candidate(&request))
                .unwrap()
                .disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        let validated_correction =
            ValidatedOnlinePgoResult::from_result(&request, correction(&request)).unwrap();
        assert_eq!(
            validated_correction.transform().unwrap().translation_m(),
            [0.1, -0.2, 0.3]
        );
        assert_eq!(
            validated_correction.disposition(),
            ValidatedOnlinePgoDisposition::Correction
        );
        assert_eq!(
            validated_correction.clone().into_result(),
            correction(&request)
        );
        assert_eq!(
            ValidatedOnlinePgoResult::from_result(&request, rejected(&request, false))
                .unwrap()
                .disposition(),
            ValidatedOnlinePgoDisposition::Rejected
        );

        let mut nonfinite = correction(&request);
        nonfinite.translation_m[0] = f64::NAN;
        assert!(codec::seal_native_pgo_result(nonfinite).is_err());
        assert!(codec::decode_native_pgo_result_wire(&[0_u8; 24]).is_err());
    }

    #[tokio::test]
    async fn status_and_result_require_exact_order_identity_receipt_and_single_terminal() {
        let topics = OnlinePgoTopics::for_runtime(13, 4).unwrap();
        let request = prepare_request(complete_request(13, 4, 1, 3_000_000_000), 13, 4, 1).unwrap();
        let (current, reply_rx) = active_request(request.clone());
        let mut active = Some(current);
        let mut sequence = 2;

        let early_result = no_candidate(&request);
        let early_error = handle_result_receive(
            Ok(received_result(&topics.result, &early_result)),
            &topics.result,
            &mut active,
        )
        .unwrap_err();
        assert!(early_error.to_string().contains("before REQUEST_ACCEPTED"));

        let accepted = request_status(
            &request,
            3,
            codec::NATIVE_PGO_EVENT_REQUEST_ACCEPTED,
            codec::NATIVE_PGO_TERMINAL_NONE,
            codec::NATIVE_PGO_CANCEL_NONE,
            codec::NATIVE_PGO_REASON_NONE,
            "",
        );
        handle_status_receive(
            Ok(received_status(&topics.status, &accepted)),
            &topics.status,
            13,
            4,
            &mut sequence,
            &mut active,
        )
        .unwrap();
        let duplicate = handle_status_receive(
            Ok(received_status(&topics.status, &accepted)),
            &topics.status,
            13,
            4,
            &mut sequence,
            &mut active,
        )
        .unwrap_err();
        assert!(duplicate
            .to_string()
            .contains("duplicate, stale, or skipped"));

        let running = request_status(
            &request,
            4,
            codec::NATIVE_PGO_EVENT_RUNNING,
            codec::NATIVE_PGO_TERMINAL_NONE,
            codec::NATIVE_PGO_CANCEL_NONE,
            codec::NATIVE_PGO_REASON_NONE,
            "",
        );
        handle_status_receive(
            Ok(received_status(&topics.status, &running)),
            &topics.status,
            13,
            4,
            &mut sequence,
            &mut active,
        )
        .unwrap();
        handle_result_receive(
            Ok(received_result(&topics.result, &early_result)),
            &topics.result,
            &mut active,
        )
        .unwrap();
        let terminal = request_status(
            &request,
            5,
            codec::NATIVE_PGO_EVENT_TERMINAL,
            codec::NATIVE_PGO_TERMINAL_SUCCEEDED,
            codec::NATIVE_PGO_CANCEL_NONE,
            codec::NATIVE_PGO_REASON_NONE,
            "",
        );
        handle_status_receive(
            Ok(received_status(&topics.status, &terminal)),
            &topics.status,
            13,
            4,
            &mut sequence,
            &mut active,
        )
        .unwrap();
        let admitted = AtomicBool::new(true);
        assert!(try_finish_active(&mut active, &admitted).unwrap());
        assert_eq!(
            reply_rx.await.unwrap().unwrap().disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        assert!(!admitted.load(Ordering::Acquire));
    }

    #[test]
    fn wrong_generation_late_malformed_and_terminal_matrix_fail_closed() {
        let topics = OnlinePgoTopics::for_runtime(14, 5).unwrap();
        let request = prepare_request(complete_request(14, 5, 1, 4_000_000_000), 14, 5, 1).unwrap();
        let wrong_generation = codec::seal_native_pgo_status(NativePgoStatus {
            schema_version: codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            backend: codec::NATIVE_PGO_BACKEND,
            runtime_generation: 14,
            backend_generation: 6,
            status_sequence: 3,
            status_timestamp_ns: 0,
            request: None,
            event_kind: codec::NATIVE_PGO_EVENT_HEALTH,
            health_state: codec::NATIVE_PGO_HEALTH_HEALTHY,
            terminal_state: codec::NATIVE_PGO_TERMINAL_NONE,
            cancel_state: codec::NATIVE_PGO_CANCEL_NONE,
            reason_code: codec::NATIVE_PGO_REASON_NONE,
            reason: String::new(),
        })
        .unwrap();
        let mut sequence = 2;
        assert!(handle_status_receive(
            Ok(received_status(&topics.status, &wrong_generation)),
            &topics.status,
            14,
            5,
            &mut sequence,
            &mut None,
        )
        .is_err());

        for (runtime_generation, backend_generation) in [(14, 4), (15, 5)] {
            let foreign_generation = codec::seal_native_pgo_status(NativePgoStatus {
                runtime_generation,
                backend_generation,
                ..wrong_generation.clone()
            })
            .unwrap();
            let mut foreign_sequence = 2;
            assert!(handle_status_receive(
                Ok(received_status(&topics.status, &foreign_generation)),
                &topics.status,
                14,
                5,
                &mut foreign_sequence,
                &mut None,
            )
            .is_err());
            assert_eq!(foreign_sequence, 2);
        }

        let current_generation = codec::seal_native_pgo_status(NativePgoStatus {
            backend_generation: 5,
            ..wrong_generation.clone()
        })
        .unwrap();
        let mut hostile_wire = codec::encode_native_pgo_status_wire(&current_generation).unwrap();
        let last = hostile_wire.len() - 1;
        hostile_wire[last] ^= 0x01;
        assert!(codec::decode_native_pgo_status_wire(&hostile_wire).is_err());
        let mut hostile_sequence = 2;
        assert!(handle_status_receive(
            Ok(LcmReceivedMessage {
                channel: topics.status.clone(),
                payload: Bytes::from(hostile_wire),
            }),
            &topics.status,
            14,
            5,
            &mut hostile_sequence,
            &mut None,
        )
        .is_err());
        assert_eq!(hostile_sequence, 2);
        assert!(handle_result_receive(
            Ok(LcmReceivedMessage {
                channel: topics.result.clone(),
                payload: Bytes::from_static(b"malformed"),
            }),
            &topics.result,
            &mut None,
        )
        .is_err());

        let no_candidate =
            ValidatedOnlinePgoResult::from_result(&request, no_candidate(&request)).unwrap();
        let rejected_terminal = request_status(
            &request,
            5,
            codec::NATIVE_PGO_EVENT_TERMINAL,
            codec::NATIVE_PGO_TERMINAL_REJECTED,
            codec::NATIVE_PGO_CANCEL_NONE,
            codec::NATIVE_PGO_REASON_INVALID_REQUEST,
            "invalid",
        );
        assert!(validate_result_terminal_matrix(&no_candidate, &rejected_terminal, None,).is_err());
    }

    #[tokio::test]
    async fn cancelled_result_uses_the_single_exact_ack_as_terminal_proof() {
        let request = prepare_request(complete_request(15, 6, 1, 5_000_000_000), 15, 6, 1).unwrap();
        let result =
            ValidatedOnlinePgoResult::from_result(&request, rejected(&request, true)).unwrap();
        let terminal = request_status(
            &request,
            5,
            codec::NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED,
            codec::NATIVE_PGO_TERMINAL_CANCELLED,
            codec::NATIVE_PGO_CANCEL_ACKNOWLEDGED,
            codec::NATIVE_PGO_REASON_CANCELLED,
            "cancelled",
        );
        assert!(validate_result_terminal_matrix(
            &result,
            &terminal,
            Some(OnlinePgoCancelOutcome::Cancelled),
        )
        .is_ok());
        assert!(validate_result_terminal_matrix(
            &ValidatedOnlinePgoResult::from_result(&request, no_candidate(&request)).unwrap(),
            &terminal,
            Some(OnlinePgoCancelOutcome::Cancelled),
        )
        .is_err());

        let topics = OnlinePgoTopics::for_runtime(15, 6).unwrap();
        let (mut current, reply_rx) = active_request(request.clone());
        current.accepted = true;
        current.running = true;
        current.cancel_requested = true;
        current.cancel_deadline = Some(Instant::now() + Duration::from_secs(1));
        let mut active = Some(current);
        let mut sequence = 4;
        handle_status_receive(
            Ok(received_status(&topics.status, &terminal)),
            &topics.status,
            15,
            6,
            &mut sequence,
            &mut active,
        )
        .unwrap();
        assert_eq!(
            active.as_ref().unwrap().cancel_ack,
            Some(OnlinePgoCancelOutcome::Cancelled)
        );
        assert_eq!(
            active
                .as_ref()
                .unwrap()
                .terminal
                .as_ref()
                .unwrap()
                .event_kind,
            codec::NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED
        );
        handle_result_receive(
            Ok(received_result(&topics.result, &rejected(&request, true))),
            &topics.result,
            &mut active,
        )
        .unwrap();
        let admitted = AtomicBool::new(true);
        assert!(try_finish_active(&mut active, &admitted).unwrap());
        let result = reply_rx.await.unwrap().unwrap();
        assert_eq!(
            result.disposition(),
            ValidatedOnlinePgoDisposition::Rejected
        );
        assert_eq!(
            result.result().rejection_reason_code,
            codec::NATIVE_PGO_REJECTION_CANCELLED
        );
    }

    async fn assert_owner_health_closes_generation(runtime_generation: i64, health_state: i32) {
        let config = test_config(runtime_generation, 1);
        let health_publisher = LcmContext::new(&config.lcm_url).unwrap();
        let (owner, mut ingress) = OnlinePgoOwner::start(config).await.unwrap();
        let status = owner_health_status(runtime_generation, 1, 3, health_state);
        health_publisher
            .publish(
                &owner.topics().status,
                Bytes::from(codec::encode_native_pgo_status_wire(&status).unwrap()),
            )
            .await
            .unwrap();
        tokio::time::timeout(Duration::from_secs(3), async {
            while !owner.is_finished() {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("owner did not close after non-healthy status");
        assert!(!ingress.is_open());
        assert_eq!(
            ingress
                .submit(complete_request(runtime_generation, 1, 1, 1))
                .await
                .unwrap_err(),
            OnlinePgoError::Stopped
        );
        let shutdown = owner.shutdown().await.unwrap();
        assert!(matches!(
            shutdown.cause,
            OnlinePgoStopCause::Terminal(OnlinePgoError::OwnerHealth {
                health_state: observed,
                reason_code: codec::NATIVE_PGO_REASON_TRANSPORT_ERROR,
                ref reason,
            }) if observed == health_state && reason == "injected_owner_health"
        ));
        health_publisher.shutdown().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn degraded_and_unhealthy_owner_health_close_generation_and_ingress() {
        assert_owner_health_closes_generation(20_005, codec::NATIVE_PGO_HEALTH_DEGRADED).await;
        assert_owner_health_closes_generation(20_006, codec::NATIVE_PGO_HEALTH_UNHEALTHY).await;
    }

    #[tokio::test]
    async fn degraded_and_unhealthy_owner_health_fail_the_active_request() {
        for health_state in [
            codec::NATIVE_PGO_HEALTH_DEGRADED,
            codec::NATIVE_PGO_HEALTH_UNHEALTHY,
        ] {
            let topics = OnlinePgoTopics::for_runtime(20_007, 1).unwrap();
            let request = prepare_request(complete_request(20_007, 1, 1, 1), 20_007, 1, 1).unwrap();
            let (current, reply_rx) = active_request(request);
            let mut active = Some(current);
            let mut sequence = 2;
            let error = handle_status_receive(
                Ok(received_status(
                    &topics.status,
                    &owner_health_status(20_007, 1, 3, health_state),
                )),
                &topics.status,
                20_007,
                1,
                &mut sequence,
                &mut active,
            )
            .unwrap_err();
            assert!(matches!(
                error,
                OnlinePgoError::OwnerHealth {
                    health_state: observed,
                    reason_code: codec::NATIVE_PGO_REASON_TRANSPORT_ERROR,
                    ..
                } if observed == health_state
            ));
            let admitted = AtomicBool::new(true);
            fail_active(active.take().unwrap(), error.clone(), &admitted);
            assert_eq!(reply_rx.await.unwrap().unwrap_err(), error);
            assert!(!admitted.load(Ordering::Acquire));
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn real_lcm_subscriptions_precede_spawn_ready_and_no_candidate_terminal() {
        let config = test_config(20_001, 1);
        let (owner, mut ingress) = OnlinePgoOwner::start(config).await.unwrap();
        assert!(owner.topics().request.ends_with(REQUEST_TYPE));
        let result = ingress
            .submit(complete_request(20_001, 1, 1, 6_000_000_000))
            .await
            .unwrap();
        assert_eq!(
            result.disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        let shutdown = owner.shutdown().await.unwrap();
        assert_eq!(shutdown.cause, OnlinePgoStopCause::Requested);
        assert!(shutdown.logs.stdout.total_bytes < 64 * 1024);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn real_binary_three_keyframes_emit_validated_full_se3_correction() {
        let config = test_config(20_003, 1);
        let (owner, mut ingress) = OnlinePgoOwner::start(config).await.unwrap();
        let first = with_odom_translation(
            complete_request(20_003, 1, 1, 1_000_000_000),
            [0.0, 0.0, 0.0],
        );
        let second = with_odom_translation(
            complete_request(20_003, 1, 2, 62_000_000_000),
            [2.0, 0.0, 0.0],
        );
        let third = with_odom_translation(
            complete_request(20_003, 1, 3, 123_000_000_000),
            [0.2, 0.0, 0.0],
        );
        assert_eq!(
            ingress.submit(first).await.unwrap().disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        assert_eq!(
            ingress.submit(second).await.unwrap().disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        let correction = ingress.submit(third).await.unwrap();
        assert_eq!(
            correction.disposition(),
            ValidatedOnlinePgoDisposition::Correction,
            "third keyframe did not produce a correction"
        );
        let result = correction.result();
        let transform = correction.transform().unwrap();
        assert_eq!(
            result.transform_semantics,
            codec::NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3
        );
        assert!(result.fitness_score.is_finite());
        assert!(result.inlier_rmse_m.is_finite());
        assert!(result.loop_inlier_count >= 20);
        assert!(result.optimized_graph_node_count >= 3);
        assert!(result.optimized_graph_loop_edge_count >= 1);
        assert!(transform
            .translation_m()
            .iter()
            .all(|value| value.is_finite()));
        assert!(transform
            .rotation_xyzw()
            .iter()
            .all(|value| value.is_finite()));
        owner.shutdown().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn real_binary_active_cancel_waits_for_ack_rejected_result_and_terminal() {
        let mut config = test_config(20_004, 1);
        config.result_timeout = Duration::from_secs(10);
        config.cancel_timeout = Duration::from_secs(5);
        let topics = OnlinePgoTopics::for_runtime(20_004, 1).unwrap();
        let audit_context = LcmContext::new(&config.lcm_url).unwrap();
        let mut audit_subscription = audit_context.subscribe(&topics.status).await.unwrap();
        let (mut owner, mut ingress) = OnlinePgoOwner::start(config).await.unwrap();
        let first = with_odom_translation(
            complete_request(20_004, 1, 1, 1_000_000_000),
            [0.0, 0.0, 0.0],
        );
        let second = with_odom_translation(
            complete_request(20_004, 1, 2, 62_000_000_000),
            [2.0, 0.0, 0.0],
        );
        assert_eq!(
            ingress.submit(first).await.unwrap().disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        assert_eq!(
            ingress.submit(second).await.unwrap().disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        let request = with_odom_translation(
            with_dense_cloud(complete_request(20_004, 1, 3, 123_000_000_000), 20_000),
            [0.2, 0.0, 0.0],
        );
        let submitted = tokio::spawn(async move { ingress.submit(request).await });
        let cancel = loop {
            match owner.cancel_in_flight().await.unwrap() {
                OnlinePgoCancelOutcome::NoInFlight => tokio::task::yield_now().await,
                outcome => break outcome,
            }
        };
        assert_eq!(cancel, OnlinePgoCancelOutcome::Cancelled);
        let result = submitted.await.unwrap().unwrap();
        assert_eq!(
            result.disposition(),
            ValidatedOnlinePgoDisposition::Rejected,
            "cancelled request did not return typed REJECTED"
        );
        assert_eq!(
            result.result().rejection_reason_code,
            codec::NATIVE_PGO_REJECTION_CANCELLED
        );
        let expected_echo = result.result().request.clone();
        let audit_deadline = Instant::now() + Duration::from_millis(250);
        let mut request_events = Vec::new();
        loop {
            let received = tokio::time::timeout_at(
                tokio::time::Instant::from_std(audit_deadline),
                audit_subscription.receiver.recv(),
            )
            .await;
            let Ok(Ok(received)) = received else {
                break;
            };
            let status = codec::decode_native_pgo_status_wire(&received.payload).unwrap();
            if status.request.as_ref() == Some(&expected_echo) {
                request_events.push(status.event_kind);
            }
        }
        assert_eq!(
            request_events,
            [
                codec::NATIVE_PGO_EVENT_REQUEST_ACCEPTED,
                codec::NATIVE_PGO_EVENT_RUNNING,
                codec::NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED,
            ]
        );
        owner.shutdown().await.unwrap();
        audit_subscription.unsubscribe().await.unwrap();
        audit_context.shutdown().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn real_binary_restart_isolates_all_old_topics_and_serves_new_generation() {
        const RUNTIME_GENERATION: i64 = 20_008;

        let mut old_config = test_config(RUNTIME_GENERATION, 1);
        old_config.result_timeout = Duration::from_secs(15);
        let mut new_config = old_config.clone();
        new_config.backend_generation = 2;
        new_config.previous_backend_generation = Some(1);
        let old_topics = old_config.validate().unwrap();
        let new_topics = new_config.validate().unwrap();
        assert_ne!(old_topics.request, new_topics.request);
        assert_ne!(old_topics.result, new_topics.result);
        assert_ne!(old_topics.status, new_topics.status);
        assert_ne!(old_topics.cancel, new_topics.cancel);

        let old_source = complete_request(RUNTIME_GENERATION, 1, 1, 1_000_000_000);
        let old_request = prepare_request(old_source.clone(), RUNTIME_GENERATION, 1, 1).unwrap();
        let (old_owner, mut old_ingress) = OnlinePgoOwner::start(old_config).await.unwrap();
        let old_result = old_ingress.submit(old_source).await.unwrap().into_result();
        assert_eq!(old_result.request, old_request.echo());
        drop(old_ingress);
        let old_shutdown = tokio::time::timeout(Duration::from_secs(3), old_owner.shutdown())
            .await
            .expect("generation 1 shutdown exceeded its bound")
            .unwrap();
        assert_eq!(old_shutdown.cause, OnlinePgoStopCause::Requested);
        assert!(!old_shutdown.process.escalated_to_kill);
        assert!(matches!(
            old_shutdown.process.terminal,
            SupervisedTerminal::Exited { code } if code.is_none_or(|value| value == 0)
        ));

        let old_cancel = cancel_requested_status(&old_request, 1);
        let old_status =
            owner_health_status(RUNTIME_GENERATION, 1, 3, codec::NATIVE_PGO_HEALTH_DEGRADED);
        let old_request_wire =
            Bytes::from(codec::encode_native_pgo_request_wire(&old_request).unwrap());
        let old_cancel_wire =
            Bytes::from(codec::encode_native_pgo_status_wire(&old_cancel).unwrap());
        let old_result_wire =
            Bytes::from(codec::encode_native_pgo_result_wire(&old_result).unwrap());
        let old_status_wire =
            Bytes::from(codec::encode_native_pgo_status_wire(&old_status).unwrap());

        let replay_context = LcmContext::new(&new_config.lcm_url).unwrap();
        let old_audit_context = LcmContext::new(&new_config.lcm_url).unwrap();
        let mut old_request_audit = old_audit_context
            .subscribe(&old_topics.request)
            .await
            .unwrap();
        let mut old_cancel_audit = old_audit_context
            .subscribe(&old_topics.cancel)
            .await
            .unwrap();
        let mut old_result_audit = old_audit_context
            .subscribe(&old_topics.result)
            .await
            .unwrap();
        let mut old_status_audit = old_audit_context
            .subscribe(&old_topics.status)
            .await
            .unwrap();

        let new_audit_context = LcmContext::new(&new_config.lcm_url).unwrap();
        let mut new_request_audit = new_audit_context
            .subscribe(&new_topics.request)
            .await
            .unwrap();
        let mut new_result_audit = new_audit_context
            .subscribe(&new_topics.result)
            .await
            .unwrap();
        let mut new_status_audit = new_audit_context
            .subscribe(&new_topics.status)
            .await
            .unwrap();

        let (started, startup_replay) = tokio::join!(OnlinePgoOwner::start(new_config), async {
            tokio::task::yield_now().await;
            replay_context
                .publish(&old_topics.status, old_status_wire.clone())
                .await
        });
        startup_replay.unwrap();
        let (new_owner, new_ingress) = started.unwrap();
        let audited_startup_status =
            next_audit_message(&mut old_status_audit, "old startup status").await;
        assert_eq!(audited_startup_status.channel, old_topics.status);
        assert_eq!(audited_startup_status.payload, old_status_wire);
        assert!(!new_owner.is_finished());
        assert!(new_ingress.is_open());

        let new_source = with_dense_cloud(
            complete_request(RUNTIME_GENERATION, 2, 1, 2_000_000_000),
            100_000,
        );
        let expected_new_request =
            prepare_request(new_source.clone(), RUNTIME_GENERATION, 2, 1).unwrap();
        let submitted = tokio::spawn(async move {
            let mut ingress = new_ingress;
            let result = ingress.submit(new_source).await;
            (ingress, result)
        });
        let audited_new_request =
            next_audit_message(&mut new_request_audit, "generation 2 request").await;
        assert_eq!(audited_new_request.channel, new_topics.request);
        assert!(
            !submitted.is_finished(),
            "generation 2 request was not active during replay"
        );
        assert_eq!(
            codec::decode_native_pgo_request_wire(&audited_new_request.payload).unwrap(),
            expected_new_request
        );

        let (request_replay, cancel_replay, result_replay, status_replay) = tokio::join!(
            replay_context.publish(&old_topics.request, old_request_wire.clone()),
            replay_context.publish(&old_topics.cancel, old_cancel_wire.clone()),
            replay_context.publish(&old_topics.result, old_result_wire.clone()),
            replay_context.publish(&old_topics.status, old_status_wire.clone()),
        );
        request_replay.unwrap();
        cancel_replay.unwrap();
        result_replay.unwrap();
        status_replay.unwrap();

        for (audited, channel, wire) in [
            (
                next_audit_message(&mut old_request_audit, "old request replay").await,
                old_topics.request.as_str(),
                &old_request_wire,
            ),
            (
                next_audit_message(&mut old_cancel_audit, "old cancel replay").await,
                old_topics.cancel.as_str(),
                &old_cancel_wire,
            ),
            (
                next_audit_message(&mut old_result_audit, "old result replay").await,
                old_topics.result.as_str(),
                &old_result_wire,
            ),
            (
                next_audit_message(&mut old_status_audit, "old active status replay").await,
                old_topics.status.as_str(),
                &old_status_wire,
            ),
        ] {
            assert_eq!(audited.channel, channel);
            assert_eq!(&audited.payload, wire);
        }

        let (mut new_ingress, new_result) =
            tokio::time::timeout(Duration::from_secs(15), submitted)
                .await
                .expect("generation 2 request timed out after old traffic")
                .unwrap();
        let new_result = new_result.unwrap();
        assert_eq!(
            new_result.disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate
        );
        assert_eq!(new_result.result().request, expected_new_request.echo());
        assert!(!new_owner.is_finished());
        assert!(new_ingress.is_open());

        let audited_new_result =
            next_audit_message(&mut new_result_audit, "generation 2 result").await;
        assert_eq!(audited_new_result.channel, new_topics.result);
        assert_eq!(
            codec::decode_native_pgo_result_wire(&audited_new_result.payload).unwrap(),
            *new_result.result()
        );

        let expected_echo = expected_new_request.echo();
        let mut last_status_sequence = 0;
        let mut request_events = Vec::new();
        loop {
            let audited = next_audit_message(&mut new_status_audit, "generation 2 status").await;
            assert_eq!(audited.channel, new_topics.status);
            let status = codec::decode_native_pgo_status_wire(&audited.payload).unwrap();
            assert_eq!(status.runtime_generation, RUNTIME_GENERATION);
            assert_eq!(status.backend_generation, 2);
            assert_eq!(status.status_sequence, last_status_sequence + 1);
            last_status_sequence = status.status_sequence;
            assert_ne!(status.event_kind, codec::NATIVE_PGO_EVENT_PROTOCOL_ERROR);
            assert!(!matches!(
                status.health_state,
                codec::NATIVE_PGO_HEALTH_DEGRADED | codec::NATIVE_PGO_HEALTH_UNHEALTHY
            ));
            if status.request.as_ref() == Some(&expected_echo) {
                request_events.push(status.event_kind);
                if status.event_kind == codec::NATIVE_PGO_EVENT_TERMINAL {
                    break;
                }
            }
        }
        assert_eq!(
            request_events,
            [
                codec::NATIVE_PGO_EVENT_REQUEST_ACCEPTED,
                codec::NATIVE_PGO_EVENT_RUNNING,
                codec::NATIVE_PGO_EVENT_TERMINAL,
            ]
        );

        let follow_up = new_ingress
            .submit(complete_request(RUNTIME_GENERATION, 2, 2, 3_000_000_000))
            .await
            .expect("generation 2 did not remain serviceable after old traffic");
        assert!(matches!(
            follow_up.disposition(),
            ValidatedOnlinePgoDisposition::NoCandidate | ValidatedOnlinePgoDisposition::Correction
        ));
        drop(new_ingress);

        let new_shutdown = tokio::time::timeout(Duration::from_secs(3), new_owner.shutdown())
            .await
            .expect("generation 2 shutdown exceeded its bound")
            .unwrap();
        assert_eq!(new_shutdown.cause, OnlinePgoStopCause::Requested);
        assert!(!new_shutdown.process.escalated_to_kill);
        assert!(matches!(
            new_shutdown.process.terminal,
            SupervisedTerminal::Exited { code } if code.is_none_or(|value| value == 0)
        ));

        old_request_audit.unsubscribe().await.unwrap();
        old_cancel_audit.unsubscribe().await.unwrap();
        old_result_audit.unsubscribe().await.unwrap();
        old_status_audit.unsubscribe().await.unwrap();
        new_request_audit.unsubscribe().await.unwrap();
        new_result_audit.unsubscribe().await.unwrap();
        new_status_audit.unsubscribe().await.unwrap();
        replay_context.shutdown().await.unwrap();
        old_audit_context.shutdown().await.unwrap();
        new_audit_context.shutdown().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn child_exit_before_typed_ready_rolls_back_after_subscription_ack() {
        let mut config = test_config(20_002, 1);
        config.executable = PathBuf::from("/bin/true");
        let error = OnlinePgoOwner::start(config).await.unwrap_err();
        assert!(matches!(
            error,
            OnlinePgoStartError::Supervision { source, .. }
                if matches!(source.failure, nuwax_nav_native_runner::SupervisedStartFailure::TerminatedBeforeReady(_))
        ));
    }
}
