//! Retained native-child supervision with typed external readiness.
//!
//! This module is additive to [`crate::NativeRunner`]. It owns one process
//! group, drains diagnostics without an unbounded allocation, and makes
//! startup conditional on a caller-supplied readiness future. The readiness
//! future is expected to cross the real typed protocol boundary after the
//! caller has acknowledged its subscriptions; stdout is diagnostics only.

use std::collections::{BTreeMap, VecDeque};
use std::ffi::{OsStr, OsString};
use std::fmt;
use std::fs;
use std::future::Future;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{ExitStatus, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

#[cfg(target_os = "linux")]
use rustix::process::{
    getpgid, getpid, kill_process, kill_process_group, waitid, waitpid, Pid, Signal, WaitId,
    WaitIdOptions, WaitIdStatus, WaitOptions, WaitStatus,
};
use thiserror::Error;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWriteExt};
use tokio::process::{Child, ChildStdin, ChildStdout, Command};
use tokio::task::JoinHandle;

const READ_BUFFER_BYTES: usize = 8 * 1024;
const PROCESS_POLL_INTERVAL: Duration = Duration::from_millis(5);
const MAX_DIAGNOSTIC_CAPACITY: usize = 16 * 1024 * 1024;
const GUARDIAN_PROTOCOL_BYTES: usize = 48;
const GUARDIAN_PROTOCOL_MAGIC: &[u8; 8] = b"NUWAXGG1";
const GUARDIAN_PROTOCOL_VERSION: u32 = 1;
const GUARDIAN_MESSAGE_SPAWNED: u32 = 1;
const GUARDIAN_MESSAGE_READY: u32 = 2;
const GUARDIAN_MESSAGE_ERROR: u32 = 3;
const GUARDIAN_DISARM: u8 = b'D';
const GROUP_GUARDIAN_PATH: &str = env!("NUWAX_GROUP_GUARDIAN_PATH");
#[cfg(test)]
const TEST_GROUP_GUARDIAN_PATH: &str = env!("NUWAX_GROUP_GUARDIAN_TEST_PATH");

/// Built dedicated guardian executable used by this supervisor.
#[must_use]
pub fn group_guardian_executable() -> &'static Path {
    Path::new(GROUP_GUARDIAN_PATH)
}

/// Explicit executable, argument, working-directory, and environment contract.
///
/// The group guardian must become the process spawned by Rust, so an opaque
/// [`std::process::Command`] cannot be forwarded without losing whether the
/// environment was cleared. This value records every exec-affecting choice
/// before the guardian is introduced.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedNativeCommand {
    program: OsString,
    args: Vec<OsString>,
    current_dir: Option<PathBuf>,
    clear_environment: bool,
    environment: BTreeMap<OsString, Option<OsString>>,
    #[cfg(test)]
    guardian_test_checkpoint: Option<(GuardianTestCheckpoint, PathBuf)>,
    #[cfg(test)]
    guardian_test_exit: Option<GuardianTestExit>,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GuardianTestCheckpoint {
    Cloned = 1,
    BeforeSetpgid = 2,
    AfterSetpgid = 3,
    BeforeExec = 4,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GuardianTestExit {
    CleanupProofFailed = 73,
    ObservationFailed = 74,
    Signaled = 75,
}

impl SupervisedNativeCommand {
    /// Construct a command contract for `program`.
    pub fn new(program: impl AsRef<OsStr>) -> Self {
        Self {
            program: program.as_ref().to_os_string(),
            args: Vec::new(),
            current_dir: None,
            clear_environment: false,
            environment: BTreeMap::new(),
            #[cfg(test)]
            guardian_test_checkpoint: None,
            #[cfg(test)]
            guardian_test_exit: None,
        }
    }

    /// Append one argument without shell interpretation.
    pub fn arg(&mut self, argument: impl AsRef<OsStr>) -> &mut Self {
        self.args.push(argument.as_ref().to_os_string());
        self
    }

    /// Append arguments without shell interpretation.
    pub fn args<I, S>(&mut self, arguments: I) -> &mut Self
    where
        I: IntoIterator<Item = S>,
        S: AsRef<OsStr>,
    {
        self.args.extend(
            arguments
                .into_iter()
                .map(|value| value.as_ref().to_os_string()),
        );
        self
    }

    /// Set the payload working directory.
    pub fn current_dir(&mut self, directory: impl AsRef<Path>) -> &mut Self {
        self.current_dir = Some(directory.as_ref().to_path_buf());
        self
    }

    /// Set one payload environment entry.
    pub fn env(&mut self, name: impl AsRef<OsStr>, value: impl AsRef<OsStr>) -> &mut Self {
        self.environment.insert(
            name.as_ref().to_os_string(),
            Some(value.as_ref().to_os_string()),
        );
        self
    }

    /// Set payload environment entries.
    pub fn envs<I, K, V>(&mut self, entries: I) -> &mut Self
    where
        I: IntoIterator<Item = (K, V)>,
        K: AsRef<OsStr>,
        V: AsRef<OsStr>,
    {
        for (name, value) in entries {
            self.env(name, value);
        }
        self
    }

    /// Remove one inherited payload environment entry.
    pub fn env_remove(&mut self, name: impl AsRef<OsStr>) -> &mut Self {
        self.environment.insert(name.as_ref().to_os_string(), None);
        self
    }

    /// Start the payload from an empty environment.
    pub fn env_clear(&mut self) -> &mut Self {
        self.clear_environment = true;
        self.environment.clear();
        self
    }

    #[cfg(test)]
    fn guardian_test_checkpoint(
        &mut self,
        checkpoint: GuardianTestCheckpoint,
        state_path: impl AsRef<Path>,
    ) -> &mut Self {
        self.guardian_test_exit = None;
        self.guardian_test_checkpoint = Some((checkpoint, state_path.as_ref().to_path_buf()));
        self
    }

    #[cfg(test)]
    fn guardian_test_exit(&mut self, exit: GuardianTestExit) -> &mut Self {
        self.guardian_test_checkpoint = None;
        self.guardian_test_exit = Some(exit);
        self
    }

    fn into_guardian(self, parent_pid: u32, cleanup_timeout: Duration) -> Command {
        #[cfg(test)]
        let guardian_path =
            if self.guardian_test_checkpoint.is_some() || self.guardian_test_exit.is_some() {
                TEST_GROUP_GUARDIAN_PATH
            } else {
                GROUP_GUARDIAN_PATH
            };
        #[cfg(not(test))]
        let guardian_path = GROUP_GUARDIAN_PATH;
        let mut command = Command::new(guardian_path);
        command
            .arg("--parent-pid")
            .arg(parent_pid.to_string())
            .arg("--cleanup-timeout-ms")
            .arg(cleanup_timeout.as_millis().to_string());
        #[cfg(test)]
        if let Some((checkpoint, state_path)) = self.guardian_test_checkpoint {
            command
                .arg("--test-stage")
                .arg((checkpoint as u8).to_string())
                .arg("--test-state-path")
                .arg(state_path);
        }
        #[cfg(test)]
        if let Some(exit) = self.guardian_test_exit {
            command
                .arg("--test-exit-code")
                .arg((exit as u8).to_string());
        }
        command.arg("--").arg(self.program).args(self.args);
        if let Some(directory) = self.current_dir {
            command.current_dir(directory);
        }
        if self.clear_environment {
            command.env_clear();
        }
        for (name, value) in self.environment {
            match value {
                Some(value) => {
                    command.env(name, value);
                }
                None => {
                    command.env_remove(name);
                }
            }
        }
        command
    }
}

/// Bounds and cleanup policy for one supervised process group.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedNativeConfig {
    /// Maximum stdout bytes retained in memory. The pipe continues draining
    /// after the bound is reached.
    pub stdout_capacity: usize,
    /// Maximum stderr bytes retained in memory. The pipe continues draining
    /// after the bound is reached.
    pub stderr_capacity: usize,
    /// Grace period used to roll back a child whose startup did not become
    /// ready.
    pub startup_rollback_grace: Duration,
    /// Additional bounded interval used after SIGKILL to reap the direct
    /// child, observe an empty process group, and finish diagnostic drains.
    pub kill_reap_timeout: Duration,
}

impl Default for SupervisedNativeConfig {
    fn default() -> Self {
        Self {
            stdout_capacity: 64 * 1024,
            stderr_capacity: 64 * 1024,
            startup_rollback_grace: Duration::from_millis(250),
            kill_reap_timeout: Duration::from_secs(2),
        }
    }
}

impl SupervisedNativeConfig {
    fn validate(&self) -> Result<(), SupervisedStartFailure> {
        if self.stdout_capacity == 0 || self.stderr_capacity == 0 {
            return Err(SupervisedStartFailure::InvalidConfig(
                "stdout and stderr capacities must be positive".to_string(),
            ));
        }
        if self.stdout_capacity > MAX_DIAGNOSTIC_CAPACITY
            || self.stderr_capacity > MAX_DIAGNOSTIC_CAPACITY
        {
            return Err(SupervisedStartFailure::InvalidConfig(format!(
                "stdout and stderr capacities must not exceed {MAX_DIAGNOSTIC_CAPACITY} bytes per stream"
            )));
        }
        if self.startup_rollback_grace.is_zero() || self.kill_reap_timeout.is_zero() {
            return Err(SupervisedStartFailure::InvalidConfig(
                "rollback grace and kill/reap timeout must be positive".to_string(),
            ));
        }
        let now = Instant::now();
        if now.checked_add(self.startup_rollback_grace).is_none()
            || now.checked_add(self.kill_reap_timeout).is_none()
        {
            return Err(SupervisedStartFailure::InvalidConfig(
                "cleanup duration is too large".to_string(),
            ));
        }
        if self.kill_reap_timeout.as_millis() > i32::MAX as u128 {
            return Err(SupervisedStartFailure::InvalidConfig(
                "kill/reap timeout exceeds the guardian protocol bound".to_string(),
            ));
        }
        Ok(())
    }
}

/// Stable process identity supplied to the external readiness callback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SupervisedProcessIdentity {
    pid: u32,
    process_group: u32,
    guardian_pid: u32,
}

impl SupervisedProcessIdentity {
    /// Direct child process ID.
    #[must_use]
    pub const fn pid(self) -> u32 {
        self.pid
    }

    /// Process group ID. The child is group leader, so this equals [`Self::pid`].
    #[must_use]
    pub const fn process_group(self) -> u32 {
        self.process_group
    }

    /// Dedicated guardian process ID.
    #[must_use]
    pub const fn guardian_pid(self) -> u32 {
        self.guardian_pid
    }
}

/// Failure reported by the caller's typed readiness boundary.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SupervisedReadyError {
    /// A typed message was received but failed schema or identity validation.
    #[error("readiness protocol rejected: {0}")]
    Protocol(String),
    /// The typed readiness source closed before producing a valid message.
    #[error("readiness source reached EOF: {0}")]
    Eof(String),
    /// The typed transport failed before readiness could be established.
    #[error("readiness transport failed: {0}")]
    Transport(String),
}

/// Terminal state of the direct child.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SupervisedTerminal {
    /// The child exited successfully.
    Exited {
        /// Platform exit code, when available.
        code: Option<i32>,
    },
    /// The child exited unsuccessfully or was terminated by a signal.
    Crashed {
        /// Platform exit code, when available.
        code: Option<i32>,
        /// Unix terminating signal, when available.
        signal: Option<i32>,
    },
}

impl SupervisedTerminal {
    #[cfg(target_os = "linux")]
    fn from_wait_status(status: WaitStatus) -> Result<Self, SupervisedLifecycleError> {
        if status.exited() {
            let code = status.exit_status();
            Ok(if code == Some(0) {
                Self::Exited { code }
            } else {
                Self::Crashed { code, signal: None }
            })
        } else if status.signaled() {
            Ok(Self::Crashed {
                code: None,
                signal: status.terminating_signal(),
            })
        } else {
            Err(SupervisedLifecycleError::Observation(format!(
                "waitpid returned a non-terminal status: {status:?}"
            )))
        }
    }

    #[cfg(target_os = "linux")]
    fn from_waitid(status: &WaitIdStatus) -> Result<Self, SupervisedLifecycleError> {
        if status.exited() {
            let code = status.exit_status();
            return Ok(if code == Some(0) {
                Self::Exited { code }
            } else {
                Self::Crashed { code, signal: None }
            });
        }
        if status.killed() || status.dumped() {
            return Ok(Self::Crashed {
                code: None,
                signal: status.terminating_signal(),
            });
        }
        Err(SupervisedLifecycleError::Observation(format!(
            "waitid returned a non-terminal status: {status:?}"
        )))
    }
}

/// One bounded diagnostic stream snapshot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedStreamSnapshot {
    /// Retained tail bytes, never larger than the configured capacity.
    pub bytes: Vec<u8>,
    /// Total bytes drained from the pipe.
    pub total_bytes: u64,
    /// Whether earlier bytes were discarded to maintain the bound.
    pub truncated: bool,
    /// Whether the stream reached EOF.
    pub eof: bool,
    /// Read failure, if the drain ended abnormally.
    pub error: Option<String>,
}

/// Bounded stdout and stderr diagnostics.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedLogs {
    /// Stdout diagnostics. Stdout never establishes readiness.
    pub stdout: SupervisedStreamSnapshot,
    /// Stderr diagnostics.
    pub stderr: SupervisedStreamSnapshot,
}

/// Startup failure before a typed READY value was accepted.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SupervisedStartFailure {
    /// The identity-preserving cleanup implementation requires Linux procfs
    /// and `waitid(WNOWAIT)`.
    #[error("supervised process groups require Linux with procfs")]
    UnsupportedPlatform,
    /// Supervisor configuration is internally inconsistent.
    #[error("invalid supervisor configuration: {0}")]
    InvalidConfig(String),
    /// The absolute readiness deadline had already expired or elapsed.
    #[error("typed readiness deadline elapsed")]
    ReadyTimedOut,
    /// Process creation failed.
    #[error("native child spawn failed: {0}")]
    Spawn(String),
    /// The dedicated guardian failed its identity or startup protocol.
    #[error("native group guardian startup failed: {0}")]
    Guardian(String),
    /// Bounded diagnostic storage could not be reserved before spawn.
    #[error("failed to reserve bounded diagnostic storage: {0}")]
    DiagnosticAllocation(String),
    /// The spawned child did not expose a valid process ID.
    #[error("spawned native child has no process ID")]
    MissingPid,
    /// A required diagnostic pipe was unexpectedly unavailable.
    #[error("spawned native child is missing its {0} pipe")]
    MissingPipe(&'static str),
    /// The direct child terminated before readiness was accepted.
    #[error("native child terminated before typed readiness: {0:?}")]
    TerminatedBeforeReady(SupervisedTerminal),
    /// The external typed readiness boundary rejected startup.
    #[error(transparent)]
    Readiness(#[from] SupervisedReadyError),
    /// Waiting for the direct child failed during startup.
    #[error("failed to observe native child during startup: {0}")]
    Observation(String),
}

/// Startup failure plus bounded diagnostics and rollback evidence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedStartError {
    /// Primary startup failure.
    pub failure: SupervisedStartFailure,
    /// Diagnostics captured through the end of rollback.
    pub logs: SupervisedLogs,
    /// Rollback failure, if TERM/KILL/reap could not prove cleanup.
    pub rollback_error: Option<SupervisedLifecycleError>,
}

impl fmt::Display for SupervisedStartError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}", self.failure)?;
        if let Some(cleanup) = &self.rollback_error {
            write!(formatter, "; startup rollback failed: {cleanup}")?;
        }
        Ok(())
    }
}

impl std::error::Error for SupervisedStartError {}

/// Runtime observation or cleanup failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SupervisedLifecycleError {
    /// The retained direct child was unexpectedly absent.
    #[error("supervised native child is missing")]
    MissingChild,
    /// Polling or waiting for the direct child failed.
    #[error("failed to observe supervised native child: {0}")]
    Observation(String),
    /// Process-group existence could not be checked.
    #[error("failed to inspect native process group: {0}")]
    GroupObservation(String),
    /// The dedicated guardian exited or failed its normal disarm handshake.
    #[error("native group guardian failed: {0}")]
    Guardian(String),
    /// SIGKILL was sent, but cleanup could not be proven within the bounded
    /// reap interval.
    #[error("native process group did not quiesce after SIGKILL (leader_terminal={leader_terminal}, nonleader_members={nonleader_members})")]
    ReapTimedOut {
        /// Whether WNOWAIT observed the leader terminal state.
        leader_terminal: bool,
        /// Whether procfs still showed non-leader members in the pinned PGID.
        nonleader_members: bool,
    },
    /// A diagnostic drain task panicked or exceeded the cleanup deadline.
    #[error("diagnostic drain failed: {0}")]
    DiagnosticDrain(String),
}

/// Proven process-group shutdown result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SupervisedShutdown {
    /// Direct-child terminal observation.
    pub terminal: SupervisedTerminal,
    /// Whether the TERM deadline elapsed and SIGKILL was required.
    pub escalated_to_kill: bool,
    /// Non-fatal signal errors for which subsequent observation still proved
    /// the entire group gone.
    pub signal_warnings: Vec<String>,
}

/// Successful typed startup result.
#[derive(Debug)]
pub struct SupervisedNativeStart<R> {
    /// Retained, non-cloneable process owner.
    pub child: SupervisedNativeChild,
    /// Value decoded and validated by the caller's typed readiness boundary.
    pub ready: R,
}

#[derive(Debug)]
struct BoundedStreamState {
    capacity: usize,
    bytes: VecDeque<u8>,
    total_bytes: u64,
    eof: bool,
    error: Option<String>,
}

impl BoundedStreamState {
    fn try_new(capacity: usize) -> Result<Self, SupervisedStartFailure> {
        let mut bytes = VecDeque::new();
        bytes.try_reserve_exact(capacity).map_err(|error| {
            SupervisedStartFailure::DiagnosticAllocation(format!("{capacity} bytes: {error}"))
        })?;
        Ok(Self {
            capacity,
            bytes,
            total_bytes: 0,
            eof: false,
            error: None,
        })
    }

    fn empty() -> Self {
        Self {
            capacity: 1,
            bytes: VecDeque::new(),
            total_bytes: 0,
            eof: false,
            error: None,
        }
    }

    fn push(&mut self, chunk: &[u8]) {
        self.total_bytes = self
            .total_bytes
            .saturating_add(u64::try_from(chunk.len()).unwrap_or(u64::MAX));
        if chunk.len() >= self.capacity {
            self.bytes.clear();
            self.bytes.extend(
                chunk[chunk.len().saturating_sub(self.capacity)..]
                    .iter()
                    .copied(),
            );
            return;
        }
        let overflow = self
            .bytes
            .len()
            .saturating_add(chunk.len())
            .saturating_sub(self.capacity);
        if overflow > 0 {
            self.bytes.drain(..overflow);
        }
        self.bytes.extend(chunk.iter().copied());
    }

    fn snapshot(&self) -> SupervisedStreamSnapshot {
        SupervisedStreamSnapshot {
            bytes: self.bytes.iter().copied().collect(),
            total_bytes: self.total_bytes,
            truncated: self.total_bytes > self.bytes.len() as u64,
            eof: self.eof,
            error: self.error.clone(),
        }
    }
}

type SharedStream = Arc<Mutex<BoundedStreamState>>;

fn snapshot_stream(stream: &SharedStream) -> SupervisedStreamSnapshot {
    stream
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .snapshot()
}

fn spawn_bounded_drain<R>(mut reader: R, state: SharedStream) -> JoinHandle<()>
where
    R: AsyncRead + Unpin + Send + 'static,
{
    tokio::spawn(async move {
        let mut buffer = [0_u8; READ_BUFFER_BYTES];
        loop {
            match reader.read(&mut buffer).await {
                Ok(0) => {
                    state
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .eof = true;
                    break;
                }
                Ok(read) => {
                    state
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .push(&buffer[..read]);
                }
                Err(error) => {
                    state
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .error = Some(error.to_string());
                    break;
                }
            }
        }
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct GuardianHandshake {
    kind: u32,
    error_number: u32,
    guardian_pid: u32,
    parent_pid: u32,
    payload_pid: u32,
    process_group: u32,
    parent_pidfd_ready: bool,
    payload_pidfd_ready: bool,
}

fn decode_guardian_u32(message: &[u8; GUARDIAN_PROTOCOL_BYTES], offset: usize) -> u32 {
    u32::from_be_bytes(
        message[offset..offset + 4]
            .try_into()
            .expect("guardian field is four bytes"),
    )
}

fn decode_guardian_handshake(
    message: &[u8; GUARDIAN_PROTOCOL_BYTES],
) -> Result<GuardianHandshake, SupervisedStartFailure> {
    if &message[..GUARDIAN_PROTOCOL_MAGIC.len()] != GUARDIAN_PROTOCOL_MAGIC {
        return Err(SupervisedStartFailure::Guardian(
            "invalid protocol magic".to_string(),
        ));
    }
    let version = decode_guardian_u32(message, 8);
    if version != GUARDIAN_PROTOCOL_VERSION {
        return Err(SupervisedStartFailure::Guardian(format!(
            "unsupported protocol version {version}"
        )));
    }
    let parent_pidfd_ready = decode_guardian_u32(message, 36);
    let payload_pidfd_ready = decode_guardian_u32(message, 40);
    if parent_pidfd_ready > 1 || payload_pidfd_ready > 1 {
        return Err(SupervisedStartFailure::Guardian(
            "invalid pidfd readiness flag".to_string(),
        ));
    }
    Ok(GuardianHandshake {
        kind: decode_guardian_u32(message, 12),
        error_number: decode_guardian_u32(message, 16),
        guardian_pid: decode_guardian_u32(message, 20),
        parent_pid: decode_guardian_u32(message, 24),
        payload_pid: decode_guardian_u32(message, 28),
        process_group: decode_guardian_u32(message, 32),
        parent_pidfd_ready: parent_pidfd_ready == 1,
        payload_pidfd_ready: payload_pidfd_ready == 1,
    })
}

async fn read_guardian_handshake(
    stdout: &mut ChildStdout,
    deadline: Instant,
) -> Result<GuardianHandshake, SupervisedStartFailure> {
    if Instant::now() >= deadline {
        return Err(SupervisedStartFailure::ReadyTimedOut);
    }
    let mut message = [0_u8; GUARDIAN_PROTOCOL_BYTES];
    tokio::time::timeout_at(
        tokio::time::Instant::from_std(deadline),
        stdout.read_exact(&mut message),
    )
    .await
    .map_err(|_| SupervisedStartFailure::ReadyTimedOut)?
    .map_err(|error| {
        SupervisedStartFailure::Guardian(format!("read startup handshake: {error}"))
    })?;
    decode_guardian_handshake(&message)
}

fn validate_guardian_identity(
    handshake: GuardianHandshake,
    expected_kind: u32,
    guardian_pid: u32,
    parent_pid: u32,
    payload_pid: Option<u32>,
) -> Result<GuardianHandshake, SupervisedStartFailure> {
    if handshake.kind == GUARDIAN_MESSAGE_ERROR {
        return Err(SupervisedStartFailure::Guardian(format!(
            "guardian reported errno {} for payload {}",
            handshake.error_number, handshake.payload_pid
        )));
    }
    if handshake.kind != expected_kind
        || handshake.error_number != 0
        || handshake.guardian_pid != guardian_pid
        || handshake.parent_pid != parent_pid
        || !handshake.parent_pidfd_ready
        || handshake.payload_pid == 0
        || handshake.process_group != handshake.payload_pid
        || payload_pid.is_some_and(|expected| handshake.payload_pid != expected)
        || (expected_kind == GUARDIAN_MESSAGE_SPAWNED && handshake.payload_pidfd_ready)
        || (expected_kind == GUARDIAN_MESSAGE_READY && !handshake.payload_pidfd_ready)
    {
        return Err(SupervisedStartFailure::Guardian(format!(
            "identity mismatch in guardian message: {handshake:?}"
        )));
    }
    Ok(handshake)
}

/// Spawn a retained process group and wait for an external typed READY value.
///
/// `readiness` is invoked only after the child has been spawned and receives
/// its stable PID/PGID. It should await a typed transport message from an
/// already acknowledged subscription and validate schema, generation,
/// channels, and backend identity. The readiness future races the direct
/// child terminal state and the absolute `ready_deadline`.
///
/// On every startup failure after spawn, this function performs bounded
/// TERM-to-KILL rollback before returning. Stdout and stderr are never parsed
/// for readiness.
pub async fn spawn_supervised<R, MakeReady, ReadyFuture>(
    command: SupervisedNativeCommand,
    config: SupervisedNativeConfig,
    ready_deadline: Instant,
    readiness: MakeReady,
) -> Result<SupervisedNativeStart<R>, SupervisedStartError>
where
    MakeReady: FnOnce(SupervisedProcessIdentity) -> ReadyFuture,
    ReadyFuture: Future<Output = Result<R, SupervisedReadyError>>,
{
    if !cfg!(target_os = "linux") {
        return Err(empty_start_error(
            SupervisedStartFailure::UnsupportedPlatform,
        ));
    }
    if let Err(failure) = config.validate() {
        return Err(empty_start_error(failure));
    }
    if ready_deadline <= Instant::now() {
        return Err(empty_start_error(SupervisedStartFailure::ReadyTimedOut));
    }

    let stdout_state = match BoundedStreamState::try_new(config.stdout_capacity) {
        Ok(state) => Arc::new(Mutex::new(state)),
        Err(failure) => return Err(empty_start_error(failure)),
    };
    let stderr_state = match BoundedStreamState::try_new(config.stderr_capacity) {
        Ok(state) => Arc::new(Mutex::new(state)),
        Err(failure) => return Err(empty_start_error(failure)),
    };
    #[cfg(target_os = "linux")]
    let expected_parent = u32::try_from(getpid().as_raw_pid()).expect("positive process ID");
    #[cfg(not(target_os = "linux"))]
    let expected_parent = 0_u32;
    let mut command = command.into_guardian(expected_parent, config.kill_reap_timeout);
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true);
    if Instant::now() >= ready_deadline {
        return Err(SupervisedStartError {
            failure: SupervisedStartFailure::ReadyTimedOut,
            logs: logs_from(&stdout_state, &stderr_state),
            rollback_error: None,
        });
    }

    let mut guardian = command.spawn().map_err(|error| SupervisedStartError {
        failure: SupervisedStartFailure::Spawn(error.to_string()),
        logs: logs_from(&stdout_state, &stderr_state),
        rollback_error: None,
    })?;
    let Some(guardian_pid) = guardian.id() else {
        let _ = guardian.start_kill();
        let _ = guardian.wait().await;
        return Err(SupervisedStartError {
            failure: SupervisedStartFailure::MissingPid,
            logs: logs_from(&stdout_state, &stderr_state),
            rollback_error: None,
        });
    };
    let control = guardian.stdin.take();
    let stdout = guardian.stdout.take();
    let stderr = guardian.stderr.take();
    let missing_pipe = if control.is_none() {
        Some("control")
    } else if stdout.is_none() {
        Some("stdout")
    } else if stderr.is_none() {
        Some("stderr")
    } else {
        None
    };
    let stderr_drain = stderr
        .map(|stderr| DrainState::Running(spawn_bounded_drain(stderr, stderr_state.clone())))
        .unwrap_or(DrainState::Complete);
    if let Some(pipe) = missing_pipe {
        drop(control);
        let _ = guardian.start_kill();
        let _ = guardian.wait().await;
        return Err(SupervisedStartError {
            failure: SupervisedStartFailure::MissingPipe(pipe),
            logs: logs_from(&stdout_state, &stderr_state),
            rollback_error: None,
        });
    }
    let control = control.expect("control pipe checked");
    let mut stdout = stdout.expect("stdout pipe checked");

    let spawned = match read_guardian_handshake(&mut stdout, ready_deadline).await {
        Ok(handshake) => match validate_guardian_identity(
            handshake,
            GUARDIAN_MESSAGE_SPAWNED,
            guardian_pid,
            expected_parent,
            None,
        ) {
            Ok(handshake) => handshake,
            Err(failure) => {
                return Err(fail_pending_guardian_startup(
                    failure,
                    guardian,
                    Some(control),
                    None,
                    stdout,
                    stderr_drain,
                    &stdout_state,
                    &stderr_state,
                    config.kill_reap_timeout,
                )
                .await);
            }
        },
        Err(failure) => {
            return Err(fail_pending_guardian_startup(
                failure,
                guardian,
                Some(control),
                None,
                stdout,
                stderr_drain,
                &stdout_state,
                &stderr_state,
                config.kill_reap_timeout,
            )
            .await);
        }
    };

    let ready_handshake = match read_guardian_handshake(&mut stdout, ready_deadline).await {
        Ok(handshake) => validate_guardian_identity(
            handshake,
            GUARDIAN_MESSAGE_READY,
            guardian_pid,
            expected_parent,
            Some(spawned.payload_pid),
        ),
        Err(failure) => Err(failure),
    };
    if let Err(failure) = ready_handshake {
        return Err(fail_pending_guardian_startup(
            failure,
            guardian,
            Some(control),
            Some(spawned.payload_pid),
            stdout,
            stderr_drain,
            &stdout_state,
            &stderr_state,
            config.kill_reap_timeout,
        )
        .await);
    }

    #[cfg(target_os = "linux")]
    let process_group_valid = process_group_pid(spawned.payload_pid)
        .is_ok_and(|payload| getpgid(Some(payload)) == Ok(payload));
    #[cfg(not(target_os = "linux"))]
    let process_group_valid = false;
    if !process_group_valid {
        return Err(fail_pending_guardian_startup(
            SupervisedStartFailure::Guardian(
                "payload did not retain its handshake process group".to_string(),
            ),
            guardian,
            Some(control),
            Some(spawned.payload_pid),
            stdout,
            stderr_drain,
            &stdout_state,
            &stderr_state,
            config.kill_reap_timeout,
        )
        .await);
    }

    let stdout_drain = DrainState::Running(spawn_bounded_drain(stdout, stdout_state.clone()));
    let identity = SupervisedProcessIdentity {
        pid: spawned.payload_pid,
        process_group: spawned.process_group,
        guardian_pid,
    };
    let mut supervised = SupervisedNativeChild {
        guardian: Some(guardian),
        control: Some(control),
        identity,
        config,
        observed_terminal: None,
        reaped_terminal: None,
        guardian_failure: None,
        group_signal_active: true,
        group_cleanup_confirmed: false,
        cleanup_complete: false,
        shutdown: None,
        stdout_state,
        stderr_state,
        stdout_drain,
        stderr_drain,
    };

    if Instant::now() >= ready_deadline {
        return Err(supervised
            .rollback_startup(SupervisedStartFailure::ReadyTimedOut)
            .await);
    }

    let startup = {
        let ready_future = readiness(identity);
        tokio::pin!(ready_future);
        loop {
            let ready = tokio::select! {
                biased;
                () = tokio::time::sleep_until(tokio::time::Instant::from_std(ready_deadline)) => {
                    break Err(SupervisedStartFailure::ReadyTimedOut);
                },
                ready = &mut ready_future => Some(ready),
                () = tokio::time::sleep(PROCESS_POLL_INTERVAL) => None,
            };
            match supervised.observe_guardian_startup() {
                Ok(Some(status)) => {
                    break Err(SupervisedStartFailure::Guardian(format!(
                        "guardian {guardian_pid} exited before typed readiness: {status}"
                    )));
                }
                Err(error) => {
                    break Err(SupervisedStartFailure::Guardian(error.to_string()));
                }
                Ok(None) => {}
            }
            match observe_terminal_without_reap(identity.pid) {
                Ok(Some(terminal)) => {
                    break Err(SupervisedStartFailure::TerminatedBeforeReady(terminal));
                }
                Err(error) => {
                    break Err(SupervisedStartFailure::Observation(error.to_string()));
                }
                Ok(None) => {}
            }
            if let Some(ready) = ready {
                break if Instant::now() >= ready_deadline {
                    Err(SupervisedStartFailure::ReadyTimedOut)
                } else {
                    ready.map_err(SupervisedStartFailure::Readiness)
                };
            }
        }
    };

    match startup {
        Ok(ready) => Ok(SupervisedNativeStart {
            child: supervised,
            ready,
        }),
        Err(failure) => {
            if let SupervisedStartFailure::TerminatedBeforeReady(terminal) = &failure {
                supervised.observed_terminal = Some(terminal.clone());
            }
            Err(supervised.rollback_startup(failure).await)
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn fail_pending_guardian_startup(
    failure: SupervisedStartFailure,
    mut guardian: Child,
    control: Option<ChildStdin>,
    payload_pid: Option<u32>,
    stdout: ChildStdout,
    mut stderr_drain: DrainState,
    stdout_state: &SharedStream,
    stderr_state: &SharedStream,
    cleanup_timeout: Duration,
) -> SupervisedStartError {
    let mut stdout_drain = DrainState::Running(spawn_bounded_drain(stdout, stdout_state.clone()));
    drop(control);
    let rollback_error = cleanup_pending_guardian(&mut guardian, payload_pid, cleanup_timeout)
        .await
        .err();
    let deadline = cleanup_deadline(cleanup_timeout).unwrap_or_else(|_| Instant::now());
    let stdout_error = finish_drain_state(&mut stdout_drain, stdout_state, deadline, "stdout")
        .await
        .err();
    let stderr_error = finish_drain_state(&mut stderr_drain, stderr_state, deadline, "stderr")
        .await
        .err();
    let drain_error = [stdout_error, stderr_error]
        .into_iter()
        .flatten()
        .map(|error| error.to_string())
        .collect::<Vec<_>>();
    let rollback_error = rollback_error.or_else(|| {
        (!drain_error.is_empty())
            .then(|| SupervisedLifecycleError::DiagnosticDrain(drain_error.join("; ")))
    });
    SupervisedStartError {
        failure,
        logs: logs_from(stdout_state, stderr_state),
        rollback_error,
    }
}

async fn cleanup_pending_guardian(
    guardian: &mut Child,
    payload_pid: Option<u32>,
    cleanup_timeout: Duration,
) -> Result<(), SupervisedLifecycleError> {
    let deadline = cleanup_deadline(cleanup_timeout)?;
    if let Some(payload_pid) = payload_pid {
        let _ = signal_process_group(payload_pid, GroupSignal::Stop);
        let _ = signal_process_group(payload_pid, GroupSignal::Kill);
        #[cfg(target_os = "linux")]
        {
            let payload = process_group_pid(payload_pid)?;
            let _ = kill_process(payload, Signal::KILL);
        }
        loop {
            let leader_terminal = observe_terminal_without_reap(payload_pid)?.is_some();
            let nonleader_members = leader_terminal
                && group_has_nonleader_members_async(payload_pid, payload_pid).await?;
            if leader_terminal && !nonleader_members {
                break;
            }
            if Instant::now() >= deadline {
                return Err(SupervisedLifecycleError::ReapTimedOut {
                    leader_terminal,
                    nonleader_members: !leader_terminal || nonleader_members,
                });
            }
            tokio::time::sleep(PROCESS_POLL_INTERVAL).await;
        }
        let guardian_result = wait_guardian_exit(guardian, deadline).await;
        let reap_result = reap_payload_pid(payload_pid).map(|_| ());
        return guardian_result.and(reap_result);
    }
    wait_guardian_exit(guardian, deadline).await
}

fn empty_start_error(failure: SupervisedStartFailure) -> SupervisedStartError {
    let stdout = Arc::new(Mutex::new(BoundedStreamState::empty()));
    let stderr = Arc::new(Mutex::new(BoundedStreamState::empty()));
    SupervisedStartError {
        failure,
        logs: logs_from(&stdout, &stderr),
        rollback_error: None,
    }
}

fn logs_from(stdout: &SharedStream, stderr: &SharedStream) -> SupervisedLogs {
    SupervisedLogs {
        stdout: snapshot_stream(stdout),
        stderr: snapshot_stream(stderr),
    }
}

/// Retained owner of one native process group.
///
/// This type is deliberately non-`Clone`. Call [`Self::terminate`] or
/// [`Self::wait`] to obtain terminal proof. Because Rust has no async `Drop`,
/// dropping a live handle synchronously sends SIGKILL while the unreaped leader
/// still pins the PGID, then transfers the child to an OS reaper thread that
/// proves all non-leader members disappeared before reaping the leader.
#[derive(Debug)]
#[must_use = "retain the supervisor and call terminate or wait to observe cleanup"]
pub struct SupervisedNativeChild {
    guardian: Option<Child>,
    control: Option<ChildStdin>,
    identity: SupervisedProcessIdentity,
    config: SupervisedNativeConfig,
    observed_terminal: Option<SupervisedTerminal>,
    reaped_terminal: Option<SupervisedTerminal>,
    guardian_failure: Option<String>,
    group_signal_active: bool,
    group_cleanup_confirmed: bool,
    cleanup_complete: bool,
    shutdown: Option<SupervisedShutdown>,
    stdout_state: SharedStream,
    stderr_state: SharedStream,
    stdout_drain: DrainState,
    stderr_drain: DrainState,
}

impl SupervisedNativeChild {
    /// Stable child and process-group identity.
    #[must_use]
    pub const fn identity(&self) -> SupervisedProcessIdentity {
        self.identity
    }

    /// Snapshot bounded stdout/stderr diagnostics without stopping drains.
    #[must_use]
    pub fn logs(&self) -> SupervisedLogs {
        logs_from(&self.stdout_state, &self.stderr_state)
    }

    /// Observe whether the direct child has reached a terminal state.
    ///
    /// A `true` result is a `waitid(WNOWAIT)` observation. It deliberately
    /// leaves the leader unreaped so its PID continues to pin the numeric PGID.
    pub fn is_finished(&mut self) -> Result<bool, SupervisedLifecycleError> {
        if let Some(status) = self.observe_guardian_startup()? {
            let failure = format!(
                "guardian {} exited while payload was active: {status}",
                self.identity.guardian_pid
            );
            self.guardian_failure = Some(failure.clone());
            let _ = signal_process_group(self.identity.process_group, GroupSignal::Stop);
            let _ = signal_process_group(self.identity.process_group, GroupSignal::Kill);
            #[cfg(target_os = "linux")]
            if let Ok(payload) = process_group_pid(self.identity.pid) {
                let _ = kill_process(payload, Signal::KILL);
            }
            return Err(SupervisedLifecycleError::Guardian(failure));
        }
        self.observe_terminal().map(|terminal| terminal.is_some())
    }

    /// Wait for the leader terminal state, fail-closed clean any descendants,
    /// prove the pinned group contains no non-leader member, and reap the
    /// leader last.
    pub async fn wait(&mut self) -> Result<SupervisedTerminal, SupervisedLifecycleError> {
        if self.cleanup_complete {
            return self.cached_terminal();
        }
        if self.group_cleanup_confirmed {
            self.finish_remaining_drains().await?;
            self.cleanup_complete = true;
            return self.cached_terminal();
        }
        if !self.group_signal_active {
            return Err(SupervisedLifecycleError::Observation(
                "leader identity is no longer safe for numeric PGID operations".to_string(),
            ));
        }
        if self.observed_terminal.is_none() {
            self.observed_terminal = Some(wait_for_terminal_without_reap(self.identity.pid).await?);
        }
        let deadline = cleanup_deadline(self.config.kill_reap_timeout)?;
        let mut signal_warnings = Vec::new();
        let escalated_to_kill = self
            .freeze_and_clean_descendants(deadline, &mut signal_warnings)
            .await?;
        self.disarm_guardian(deadline).await?;
        if let Some(failure) = &self.guardian_failure {
            signal_warnings.push(format!("guardian: {failure}"));
        }
        self.reap_leader()?;
        self.shutdown = Some(SupervisedShutdown {
            terminal: self.cached_terminal()?,
            escalated_to_kill,
            signal_warnings,
        });
        self.finish_remaining_drains().await?;
        self.cleanup_complete = true;
        self.cached_terminal()
    }

    /// Send SIGTERM to the entire group, wait only until the absolute
    /// `term_deadline`, then send SIGKILL and prove direct-child reap plus
    /// process-group disappearance within the configured kill/reap timeout.
    pub async fn terminate(
        &mut self,
        term_deadline: Instant,
    ) -> Result<SupervisedShutdown, SupervisedLifecycleError> {
        if self.cleanup_complete {
            return self.cached_shutdown();
        }
        if self.group_cleanup_confirmed {
            self.finish_remaining_drains().await?;
            self.cleanup_complete = true;
            return self.cached_shutdown();
        }
        if !self.group_signal_active {
            return Err(SupervisedLifecycleError::Observation(
                "leader identity is no longer safe for numeric PGID operations".to_string(),
            ));
        }

        let mut signal_warnings = Vec::new();
        if let Err(error) = signal_process_group(self.identity.process_group, GroupSignal::Term) {
            signal_warnings.push(format!("SIGTERM: {error}"));
        }
        loop {
            if self.observe_terminal()?.is_some() {
                let deadline = cleanup_deadline(self.config.kill_reap_timeout)?;
                let escalated_to_kill = self
                    .freeze_and_clean_descendants(deadline, &mut signal_warnings)
                    .await?;
                self.disarm_guardian(deadline).await?;
                if let Some(failure) = &self.guardian_failure {
                    signal_warnings.push(format!("guardian: {failure}"));
                }
                self.reap_leader()?;
                self.shutdown = Some(SupervisedShutdown {
                    terminal: self.cached_terminal()?,
                    escalated_to_kill,
                    signal_warnings,
                });
                self.finish_remaining_drains().await?;
                self.cleanup_complete = true;
                return self.cached_shutdown();
            }
            let now = Instant::now();
            if now >= term_deadline {
                break;
            }
            sleep_to_next_poll(now, term_deadline).await;
        }

        let kill_barrier = signal_process_group(self.identity.process_group, GroupSignal::Kill);
        if let Err(error) = &kill_barrier {
            signal_warnings.push(format!("SIGKILL: {error}"));
        }
        #[cfg(target_os = "linux")]
        if let Ok(payload) = process_group_pid(self.identity.pid) {
            if let Err(error) = kill_process(payload, Signal::KILL) {
                signal_warnings.push(format!("direct-child SIGKILL fallback: {error}"));
            }
        }
        if kill_barrier.is_err() {
            return Err(kill_barrier.expect_err("checked error"));
        }
        let kill_deadline = cleanup_deadline(self.config.kill_reap_timeout)?;
        if !self.wait_for_kill_barrier_quiescence(kill_deadline).await? {
            return Err(SupervisedLifecycleError::ReapTimedOut {
                leader_terminal: self.observed_terminal.is_some(),
                nonleader_members: self.observed_terminal.is_none()
                    || group_has_nonleader_members(self.identity.process_group, self.identity.pid)?,
            });
        }
        self.disarm_guardian(kill_deadline).await?;
        if let Some(failure) = &self.guardian_failure {
            signal_warnings.push(format!("guardian: {failure}"));
        }
        self.reap_leader()?;
        self.shutdown = Some(SupervisedShutdown {
            terminal: self.cached_terminal()?,
            escalated_to_kill: true,
            signal_warnings,
        });
        self.finish_remaining_drains().await?;
        self.cleanup_complete = true;
        self.cached_shutdown()
    }

    fn observe_guardian_startup(&mut self) -> Result<Option<ExitStatus>, SupervisedLifecycleError> {
        let Some(guardian) = self.guardian.as_mut() else {
            return Ok(None);
        };
        match guardian.try_wait() {
            Ok(Some(status)) => {
                self.guardian = None;
                self.control = None;
                Ok(Some(status))
            }
            Ok(None) => Ok(None),
            Err(error) => Err(SupervisedLifecycleError::Guardian(format!(
                "inspect guardian {}: {error}",
                self.identity.guardian_pid
            ))),
        }
    }

    async fn disarm_guardian(&mut self, deadline: Instant) -> Result<(), SupervisedLifecycleError> {
        if self.guardian.is_none() {
            self.control = None;
            return Ok(());
        }
        let Some(mut control) = self.control.take() else {
            return self
                .kill_failed_guardian(
                    deadline,
                    "normal cleanup lost the unique control-pipe writer".to_string(),
                )
                .await;
        };
        let disarm = async {
            control.write_all(&[GUARDIAN_DISARM]).await?;
            control.shutdown().await
        };
        match tokio::time::timeout_at(tokio::time::Instant::from_std(deadline), disarm).await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => {
                return self
                    .kill_failed_guardian(deadline, format!("write disarm command: {error}"))
                    .await;
            }
            Err(_) => {
                return self
                    .kill_failed_guardian(deadline, "disarm command timed out".to_string())
                    .await;
            }
        }
        let Some(guardian) = self.guardian.as_mut() else {
            return Ok(());
        };
        let status =
            tokio::time::timeout_at(tokio::time::Instant::from_std(deadline), guardian.wait())
                .await
                .map_err(|_| {
                    SupervisedLifecycleError::Guardian(format!(
                        "guardian {} did not acknowledge disarm",
                        self.identity.guardian_pid
                    ))
                })?
                .map_err(|error| {
                    SupervisedLifecycleError::Guardian(format!(
                        "wait for guardian {}: {error}",
                        self.identity.guardian_pid
                    ))
                })?;
        self.guardian = None;
        if !status.success() {
            self.guardian_failure = Some(format!(
                "guardian {} rejected normal disarm with {status}",
                self.identity.guardian_pid
            ));
        }
        Ok(())
    }

    async fn kill_failed_guardian(
        &mut self,
        deadline: Instant,
        failure: String,
    ) -> Result<(), SupervisedLifecycleError> {
        self.guardian_failure = Some(failure);
        let Some(guardian) = self.guardian.as_mut() else {
            return Ok(());
        };
        let _ = guardian.start_kill();
        wait_guardian_exit(guardian, deadline).await?;
        self.guardian = None;
        Ok(())
    }

    fn observe_terminal(&mut self) -> Result<Option<SupervisedTerminal>, SupervisedLifecycleError> {
        if let Some(terminal) = &self.observed_terminal {
            return Ok(Some(terminal.clone()));
        }
        if let Some(terminal) = observe_terminal_without_reap(self.identity.pid)? {
            self.observed_terminal = Some(terminal.clone());
            return Ok(Some(terminal));
        }
        Ok(None)
    }

    async fn freeze_and_clean_descendants(
        &mut self,
        deadline: Instant,
        signal_warnings: &mut Vec<String>,
    ) -> Result<bool, SupervisedLifecycleError> {
        let stop_barrier = signal_process_group(self.identity.process_group, GroupSignal::Stop);
        if let Err(error) = &stop_barrier {
            signal_warnings.push(format!("SIGSTOP: {error}"));
        }
        if stop_barrier.is_ok()
            && !group_has_nonleader_members_async(self.identity.process_group, self.identity.pid)
                .await?
        {
            return Ok(false);
        }
        let kill_barrier = signal_process_group(self.identity.process_group, GroupSignal::Kill);
        if let Err(error) = &kill_barrier {
            signal_warnings.push(format!("SIGKILL: {error}"));
        }
        if kill_barrier.is_err() {
            return Err(kill_barrier.expect_err("checked error"));
        }
        if !self.wait_for_kill_barrier_quiescence(deadline).await? {
            return Err(SupervisedLifecycleError::ReapTimedOut {
                leader_terminal: self.observed_terminal.is_some(),
                nonleader_members: true,
            });
        }
        Ok(true)
    }

    async fn wait_for_kill_barrier_quiescence(
        &mut self,
        deadline: Instant,
    ) -> Result<bool, SupervisedLifecycleError> {
        loop {
            let leader_terminal = self.observe_terminal()?.is_some();
            if leader_terminal
                && !group_has_nonleader_members_async(
                    self.identity.process_group,
                    self.identity.pid,
                )
                .await?
            {
                return Ok(true);
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(false);
            }
            sleep_to_next_poll(now, deadline).await;
        }
    }

    fn reap_leader(&mut self) -> Result<(), SupervisedLifecycleError> {
        let observed = self.observed_terminal.clone().ok_or_else(|| {
            SupervisedLifecycleError::Observation(
                "leader must be WNOWAIT-observed before reap".to_string(),
            )
        })?;
        self.group_signal_active = false;
        let reaped = match reap_payload_pid(self.identity.pid) {
            Ok(terminal) => terminal,
            Err(error) => {
                self.group_signal_active = true;
                return Err(error);
            }
        };
        self.reaped_terminal = Some(reaped.clone());
        // Group membership was proven before the leader was reaped. From this
        // point onward no numeric PGID operation is permitted, even if the two
        // terminal representations unexpectedly disagree.
        self.group_cleanup_confirmed = true;
        if reaped != observed {
            return Err(SupervisedLifecycleError::Observation(format!(
                "WNOWAIT/reaped terminal mismatch: observed={observed:?} reaped={reaped:?}"
            )));
        }
        Ok(())
    }

    async fn finish_remaining_drains(&mut self) -> Result<(), SupervisedLifecycleError> {
        let deadline = cleanup_deadline(self.config.kill_reap_timeout)?;
        let stdout = finish_drain_state(
            &mut self.stdout_drain,
            &self.stdout_state,
            deadline,
            "stdout",
        )
        .await
        .err();
        let stderr = finish_drain_state(
            &mut self.stderr_drain,
            &self.stderr_state,
            deadline,
            "stderr",
        )
        .await
        .err();
        let errors = [stdout, stderr]
            .into_iter()
            .flatten()
            .map(|error| error.to_string())
            .collect::<Vec<_>>();
        if errors.is_empty() {
            Ok(())
        } else {
            Err(SupervisedLifecycleError::DiagnosticDrain(errors.join("; ")))
        }
    }

    fn cached_terminal(&self) -> Result<SupervisedTerminal, SupervisedLifecycleError> {
        self.reaped_terminal
            .clone()
            .or_else(|| self.observed_terminal.clone())
            .ok_or(SupervisedLifecycleError::MissingChild)
    }

    fn cached_shutdown(&self) -> Result<SupervisedShutdown, SupervisedLifecycleError> {
        self.shutdown
            .clone()
            .ok_or(SupervisedLifecycleError::MissingChild)
    }

    async fn rollback_startup(&mut self, failure: SupervisedStartFailure) -> SupervisedStartError {
        let deadline = Instant::now()
            .checked_add(self.config.startup_rollback_grace)
            .unwrap_or_else(Instant::now);
        let rollback_error = self.terminate(deadline).await.err();
        SupervisedStartError {
            failure,
            logs: self.logs(),
            rollback_error,
        }
    }
}

impl Drop for SupervisedNativeChild {
    fn drop(&mut self) {
        self.stdout_drain.abort();
        self.stderr_drain.abort();
        self.control = None;
        let guardian = self.guardian.take();
        if guardian.is_none() && self.reaped_terminal.is_some() {
            return;
        }
        if !self.group_signal_active {
            if let Some(mut guardian) = guardian {
                let _ = guardian.start_kill();
            }
            tracing::error!(
                component = "nuwax-nav-native-runner",
                pid = self.identity.pid,
                "supervised child dropped after leader identity became uncertain; numeric PGID cleanup suppressed"
            );
            spawn_detached_reaper(self.identity, true, self.config.kill_reap_timeout);
            return;
        }
        let stop_result = signal_process_group(self.identity.process_group, GroupSignal::Stop);
        let group_result = signal_process_group(self.identity.process_group, GroupSignal::Kill);
        #[cfg(target_os = "linux")]
        let child_result = process_group_pid(self.identity.pid).and_then(|payload| {
            kill_process(payload, Signal::KILL)
                .map_err(|error| SupervisedLifecycleError::Observation(error.to_string()))
        });
        #[cfg(not(target_os = "linux"))]
        let child_result = Err(SupervisedLifecycleError::GroupObservation(
            "direct payload signaling requires Linux".to_string(),
        ));
        self.group_signal_active = false;
        tracing::error!(
            component = "nuwax-nav-native-runner",
            pid = self.identity.pid,
            process_group = self.identity.process_group,
            stop_signal = ?stop_result,
            group_signal = ?group_result,
            child_signal = ?child_result,
            "supervised child dropped; transferred to detached fail-closed reaper"
        );
        if let Some(mut guardian) = guardian {
            let _ = guardian.start_kill();
        }
        spawn_detached_reaper(
            self.identity,
            group_result.is_ok(),
            self.config.kill_reap_timeout,
        );
    }
}

#[derive(Debug)]
enum DrainState {
    Running(JoinHandle<()>),
    Complete,
    Failed(String),
}

impl DrainState {
    fn abort(&mut self) {
        if let Self::Running(handle) = self {
            handle.abort();
        }
    }
}

async fn finish_drain_state(
    drain: &mut DrainState,
    stream_state: &SharedStream,
    deadline: Instant,
    stream: &'static str,
) -> Result<(), SupervisedLifecycleError> {
    match drain {
        DrainState::Complete => return Ok(()),
        DrainState::Failed(error) => {
            return Err(SupervisedLifecycleError::DiagnosticDrain(error.clone()));
        }
        DrainState::Running(_) => {}
    }
    let DrainState::Running(mut handle) = std::mem::replace(drain, DrainState::Complete) else {
        unreachable!("drain state checked above");
    };
    if Instant::now() >= deadline {
        if handle.is_finished() {
            return complete_finished_drain(drain, stream_state, stream, handle.await);
        }
        handle.abort();
        let error = format!("{stream} drain exceeded cleanup deadline");
        *drain = DrainState::Failed(error.clone());
        return Err(SupervisedLifecycleError::DiagnosticDrain(error));
    }
    match tokio::time::timeout_at(tokio::time::Instant::from_std(deadline), &mut handle).await {
        Ok(result) => complete_finished_drain(drain, stream_state, stream, result),
        Err(_) => {
            handle.abort();
            let error = format!("{stream} drain exceeded cleanup deadline");
            *drain = DrainState::Failed(error.clone());
            Err(SupervisedLifecycleError::DiagnosticDrain(error))
        }
    }
}

fn complete_finished_drain(
    drain: &mut DrainState,
    stream_state: &SharedStream,
    stream: &'static str,
    result: Result<(), tokio::task::JoinError>,
) -> Result<(), SupervisedLifecycleError> {
    if let Err(error) = result {
        let error = format!("{stream} drain task failed: {error}");
        *drain = DrainState::Failed(error.clone());
        return Err(SupervisedLifecycleError::DiagnosticDrain(error));
    }
    if let Some(read_error) = snapshot_stream(stream_state).error {
        let error = format!("{stream} read failed: {read_error}");
        *drain = DrainState::Failed(error.clone());
        return Err(SupervisedLifecycleError::DiagnosticDrain(error));
    }
    *drain = DrainState::Complete;
    Ok(())
}

#[derive(Clone, Copy, Debug)]
enum GroupSignal {
    Term,
    Stop,
    Kill,
}

#[cfg(target_os = "linux")]
fn process_group_pid(process_group: u32) -> Result<Pid, SupervisedLifecycleError> {
    let raw = i32::try_from(process_group).map_err(|_| {
        SupervisedLifecycleError::GroupObservation(format!(
            "process group {process_group} exceeds pid_t"
        ))
    })?;
    Pid::from_raw(raw).ok_or_else(|| {
        SupervisedLifecycleError::GroupObservation(
            "process group must be a positive pid_t".to_string(),
        )
    })
}

#[cfg(target_os = "linux")]
fn signal_process_group(
    process_group: u32,
    signal: GroupSignal,
) -> Result<(), SupervisedLifecycleError> {
    let pid = process_group_pid(process_group)?;
    let signal = match signal {
        GroupSignal::Term => Signal::TERM,
        GroupSignal::Stop => Signal::STOP,
        GroupSignal::Kill => Signal::KILL,
    };
    match kill_process_group(pid, signal) {
        Ok(()) => Ok(()),
        Err(error) => Err(SupervisedLifecycleError::GroupObservation(
            error.to_string(),
        )),
    }
}

#[cfg(target_os = "linux")]
fn observe_terminal_without_reap(
    pid: u32,
) -> Result<Option<SupervisedTerminal>, SupervisedLifecycleError> {
    let pid = process_group_pid(pid)?;
    let options = WaitIdOptions::EXITED | WaitIdOptions::NOHANG | WaitIdOptions::NOWAIT;
    waitid(WaitId::Pid(pid), options)
        .map_err(|error| SupervisedLifecycleError::Observation(error.to_string()))?
        .map(|status| SupervisedTerminal::from_waitid(&status))
        .transpose()
}

#[cfg(target_os = "linux")]
fn reap_payload_pid(pid: u32) -> Result<SupervisedTerminal, SupervisedLifecycleError> {
    let pid = process_group_pid(pid)?;
    match waitpid(Some(pid), WaitOptions::NOHANG)
        .map_err(|error| SupervisedLifecycleError::Observation(error.to_string()))?
    {
        Some((reaped_pid, status)) if reaped_pid == pid => {
            SupervisedTerminal::from_wait_status(status)
        }
        Some((reaped_pid, _)) => Err(SupervisedLifecycleError::Observation(format!(
            "waitpid reaped unexpected payload {reaped_pid:?}"
        ))),
        None => Err(SupervisedLifecycleError::Observation(
            "waitid observed terminal but waitpid did not reap".to_string(),
        )),
    }
}

async fn wait_guardian_exit(
    guardian: &mut Child,
    deadline: Instant,
) -> Result<(), SupervisedLifecycleError> {
    if Instant::now() >= deadline {
        let _ = guardian.start_kill();
        let _ = guardian.wait().await;
        return Err(SupervisedLifecycleError::Guardian(
            "guardian cleanup deadline elapsed".to_string(),
        ));
    }
    match tokio::time::timeout_at(tokio::time::Instant::from_std(deadline), guardian.wait()).await {
        Ok(Ok(status)) if status.success() => Ok(()),
        Ok(Ok(status)) => Err(SupervisedLifecycleError::Guardian(format!(
            "guardian exited unsuccessfully: {status}"
        ))),
        Ok(Err(error)) => Err(SupervisedLifecycleError::Guardian(format!(
            "wait for guardian: {error}"
        ))),
        Err(_) => {
            let _ = guardian.start_kill();
            let _ = guardian.wait().await;
            Err(SupervisedLifecycleError::Guardian(
                "guardian cleanup deadline elapsed".to_string(),
            ))
        }
    }
}

#[cfg(target_os = "linux")]
fn group_has_nonleader_members(
    process_group: u32,
    leader_pid: u32,
) -> Result<bool, SupervisedLifecycleError> {
    let mut leader_state = None;
    let entries = fs::read_dir("/proc").map_err(|error| {
        SupervisedLifecycleError::GroupObservation(format!("read /proc: {error}"))
    })?;
    for entry in entries {
        let entry = entry.map_err(|error| {
            SupervisedLifecycleError::GroupObservation(format!("iterate /proc: {error}"))
        })?;
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse::<u32>().ok())
        else {
            continue;
        };
        let stat = match fs::read_to_string(entry.path().join("stat")) {
            Ok(stat) => stat,
            Err(error) if error.kind() == io::ErrorKind::NotFound => continue,
            Err(error) => {
                return Err(SupervisedLifecycleError::GroupObservation(format!(
                    "read /proc/{pid}/stat: {error}"
                )));
            }
        };
        let (state, pgrp) = parse_proc_stat(pid, &stat)?;
        if pgrp != process_group {
            continue;
        }
        if pid == leader_pid {
            leader_state = Some(state);
        } else {
            return Ok(true);
        }
    }
    match leader_state {
        Some('Z' | 'X') => Ok(false),
        Some(state) => Err(SupervisedLifecycleError::GroupObservation(format!(
            "WNOWAIT leader {leader_pid} was not zombie (state={state})"
        ))),
        None => Err(SupervisedLifecycleError::GroupObservation(format!(
            "WNOWAIT leader {leader_pid} was absent from pinned process group {process_group}"
        ))),
    }
}

#[cfg(target_os = "linux")]
fn parse_proc_stat(pid: u32, stat: &str) -> Result<(char, u32), SupervisedLifecycleError> {
    let suffix = stat
        .rsplit_once(") ")
        .map(|(_, suffix)| suffix)
        .ok_or_else(|| {
            SupervisedLifecycleError::GroupObservation(format!(
                "malformed /proc/{pid}/stat command field"
            ))
        })?;
    let mut fields = suffix.split_whitespace();
    let state = fields
        .next()
        .and_then(|value| value.chars().next())
        .ok_or_else(|| {
            SupervisedLifecycleError::GroupObservation(format!("missing state in /proc/{pid}/stat"))
        })?;
    let _parent_pid = fields.next().ok_or_else(|| {
        SupervisedLifecycleError::GroupObservation(format!(
            "missing parent pid in /proc/{pid}/stat"
        ))
    })?;
    let process_group = fields
        .next()
        .ok_or_else(|| {
            SupervisedLifecycleError::GroupObservation(format!(
                "missing process group in /proc/{pid}/stat"
            ))
        })?
        .parse::<u32>()
        .map_err(|error| {
            SupervisedLifecycleError::GroupObservation(format!(
                "invalid process group in /proc/{pid}/stat: {error}"
            ))
        })?;
    Ok((state, process_group))
}

async fn wait_for_terminal_without_reap(
    pid: u32,
) -> Result<SupervisedTerminal, SupervisedLifecycleError> {
    loop {
        if let Some(terminal) = observe_terminal_without_reap(pid)? {
            return Ok(terminal);
        }
        tokio::time::sleep(PROCESS_POLL_INTERVAL).await;
    }
}

async fn group_has_nonleader_members_async(
    process_group: u32,
    leader_pid: u32,
) -> Result<bool, SupervisedLifecycleError> {
    tokio::task::spawn_blocking(move || group_has_nonleader_members(process_group, leader_pid))
        .await
        .map_err(|error| {
            SupervisedLifecycleError::GroupObservation(format!("procfs scan task failed: {error}"))
        })?
}

fn cleanup_deadline(timeout: Duration) -> Result<Instant, SupervisedLifecycleError> {
    Instant::now().checked_add(timeout).ok_or_else(|| {
        SupervisedLifecycleError::Observation("cleanup deadline overflowed".to_string())
    })
}

async fn sleep_to_next_poll(now: Instant, deadline: Instant) {
    let next_poll = now
        .checked_add(PROCESS_POLL_INTERVAL)
        .unwrap_or(deadline)
        .min(deadline);
    tokio::time::sleep_until(tokio::time::Instant::from_std(next_poll)).await;
}

fn spawn_detached_reaper(
    identity: SupervisedProcessIdentity,
    kill_barrier_established: bool,
    retry_interval: Duration,
) {
    let retry_interval = retry_interval
        .min(Duration::from_millis(50))
        .max(PROCESS_POLL_INTERVAL);
    let spawn = thread::Builder::new()
        .name(format!("nuwax-native-reaper-{}", identity.pid))
        .spawn(move || run_detached_reaper(identity, kill_barrier_established, retry_interval));
    if let Err(error) = spawn {
        tracing::error!(
            component = "nuwax-nav-native-runner",
            pid = identity.pid,
            %error,
            "failed to start detached supervisor reaper; running bounded synchronous fallback"
        );
        run_detached_reaper(identity, kill_barrier_established, retry_interval);
    }
}

fn run_detached_reaper(
    identity: SupervisedProcessIdentity,
    mut kill_barrier_established: bool,
    retry_interval: Duration,
) {
    loop {
        if !kill_barrier_established {
            kill_barrier_established =
                signal_process_group(identity.process_group, GroupSignal::Kill).is_ok();
            #[cfg(target_os = "linux")]
            if let Ok(payload) = process_group_pid(identity.pid) {
                let _ = kill_process(payload, Signal::KILL);
            }
        }
        let observed = observe_terminal_without_reap(identity.pid);
        let quiescent = match observed {
            Ok(Some(_)) if kill_barrier_established => {
                group_has_nonleader_members(identity.process_group, identity.pid)
                    .map(|members| !members)
            }
            Ok(_) => Ok(false),
            Err(error) => Err(error),
        };
        match quiescent {
            Ok(true) => match reap_payload_pid(identity.pid) {
                Ok(terminal) => {
                    tracing::debug!(
                        component = "nuwax-nav-native-runner",
                        pid = identity.pid,
                        ?terminal,
                        "detached supervisor reaped leader after group proof"
                    );
                    break;
                }
                Err(error) => tracing::error!(
                    component = "nuwax-nav-native-runner",
                    pid = identity.pid,
                    %error,
                    "detached supervisor failed to reap leader"
                ),
            },
            Ok(false) => {}
            Err(error) => tracing::warn!(
                component = "nuwax-nav-native-runner",
                pid = identity.pid,
                %error,
                "detached supervisor is retrying group proof"
            ),
        }
        thread::sleep(retry_interval);
    }
}

#[cfg(not(target_os = "linux"))]
fn signal_process_group(
    _process_group: u32,
    _signal: GroupSignal,
) -> Result<(), SupervisedLifecycleError> {
    Err(SupervisedLifecycleError::GroupObservation(
        "process-group supervision requires Unix".to_string(),
    ))
}

#[cfg(not(target_os = "linux"))]
fn observe_terminal_without_reap(
    _pid: u32,
) -> Result<Option<SupervisedTerminal>, SupervisedLifecycleError> {
    Err(SupervisedLifecycleError::GroupObservation(
        "identity-preserving process supervision requires Linux".to_string(),
    ))
}

#[cfg(not(target_os = "linux"))]
fn reap_payload_pid(_pid: u32) -> Result<SupervisedTerminal, SupervisedLifecycleError> {
    Err(SupervisedLifecycleError::Observation(
        "payload reaping requires Linux".to_string(),
    ))
}

#[cfg(not(target_os = "linux"))]
fn group_has_nonleader_members(
    _process_group: u32,
    _leader_pid: u32,
) -> Result<bool, SupervisedLifecycleError> {
    Err(SupervisedLifecycleError::GroupObservation(
        "process-group membership proof requires Linux procfs".to_string(),
    ))
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use rustix::io::Errno;
    use rustix::process::{getpgid, kill_process, test_kill_process, test_kill_process_group};
    use std::fs;
    #[cfg(target_os = "linux")]
    use std::process::{Child as StdChild, Command as StdCommand};
    use tempfile::tempdir;

    #[cfg(target_os = "linux")]
    const PARENT_DEATH_HELPER_STATE_ENV: &str = "NUWAX_PARENT_DEATH_HELPER_STATE";
    #[cfg(target_os = "linux")]
    const PARENT_DEATH_HELPER_TOKEN_ENV: &str = "NUWAX_PARENT_DEATH_HELPER_TOKEN";
    #[cfg(target_os = "linux")]
    const PARENT_DEATH_CHILD_TOKEN_ENV: &str = "NUWAX_PARENT_DEATH_CHILD_TOKEN";
    #[cfg(target_os = "linux")]
    const PARENT_DEATH_CHECKPOINT_ENV: &str = "NUWAX_PARENT_DEATH_CHECKPOINT";
    #[cfg(target_os = "linux")]
    const PARENT_DEATH_CHECKPOINT_STATE_ENV: &str = "NUWAX_PARENT_DEATH_CHECKPOINT_STATE";

    fn shell(script: &str) -> SupervisedNativeCommand {
        let mut command = SupervisedNativeCommand::new("/bin/sh");
        command.arg("-c").arg(script);
        command
    }

    fn short_config() -> SupervisedNativeConfig {
        SupervisedNativeConfig {
            stdout_capacity: 1024,
            stderr_capacity: 1024,
            startup_rollback_grace: Duration::from_millis(50),
            kill_reap_timeout: Duration::from_secs(1),
        }
    }

    fn absolute_deadline(after: Duration) -> Instant {
        Instant::now().checked_add(after).expect("test deadline")
    }

    fn raw_pid(pid: u32) -> Pid {
        Pid::from_raw(i32::try_from(pid).expect("pid_t range")).expect("positive pid")
    }

    fn process_exists(pid: u32) -> bool {
        match test_kill_process(raw_pid(pid)) {
            Ok(()) => true,
            Err(Errno::SRCH) => false,
            Err(error) => panic!("failed to inspect pid {pid}: {error}"),
        }
    }

    fn process_group_exists(process_group: u32) -> bool {
        match test_kill_process_group(raw_pid(process_group)) {
            Ok(()) => true,
            Err(Errno::SRCH) => false,
            Err(error) => panic!("failed to inspect process group {process_group}: {error}"),
        }
    }

    async fn wait_until_process_gone(pid: u32) {
        let deadline = absolute_deadline(Duration::from_secs(2));
        while process_exists(pid) && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(
            !process_exists(pid),
            "process {pid} remained after cleanup: {}",
            fs::read_to_string(format!("/proc/{pid}/stat"))
                .unwrap_or_else(|error| format!("unreadable stat: {error}"))
        );
    }

    async fn wait_until_process_group_gone(process_group: u32) {
        let deadline = absolute_deadline(Duration::from_secs(2));
        while process_group_exists(process_group) && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(
            !process_group_exists(process_group),
            "process group {process_group} remained after cleanup"
        );
    }

    async fn wait_for_pid_file(path: &std::path::Path) -> Result<u32, SupervisedReadyError> {
        let deadline = absolute_deadline(Duration::from_secs(1));
        loop {
            if let Ok(contents) = fs::read_to_string(path) {
                return contents.trim().parse::<u32>().map_err(|error| {
                    SupervisedReadyError::Protocol(format!("invalid descendant pid: {error}"))
                });
            }
            if Instant::now() >= deadline {
                return Err(SupervisedReadyError::Transport(
                    "pid file was not published".to_string(),
                ));
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    #[cfg(target_os = "linux")]
    struct TestParentProcess {
        child: Option<StdChild>,
    }

    #[cfg(target_os = "linux")]
    impl TestParentProcess {
        fn new(child: StdChild) -> Self {
            Self { child: Some(child) }
        }

        fn try_wait(&mut self) -> io::Result<Option<ExitStatus>> {
            self.child
                .as_mut()
                .expect("test parent already consumed")
                .try_wait()
        }

        fn kill_and_wait(&mut self) -> ExitStatus {
            let mut child = self.child.take().expect("test parent already consumed");
            child.kill().expect("SIGKILL test parent");
            child.wait().expect("reap SIGKILLed test parent")
        }
    }

    #[cfg(target_os = "linux")]
    impl Drop for TestParentProcess {
        fn drop(&mut self) {
            if let Some(mut child) = self.child.take() {
                let _ = child.kill();
                let _ = child.wait();
            }
        }
    }

    #[cfg(target_os = "linux")]
    struct TestProcessGroupFallback {
        pid: u32,
        process_group: u32,
        armed: bool,
    }

    #[cfg(target_os = "linux")]
    impl TestProcessGroupFallback {
        fn new(pid: u32, process_group: u32) -> Self {
            Self {
                pid,
                process_group,
                armed: true,
            }
        }

        fn disarm(&mut self) {
            self.armed = false;
        }
    }

    #[cfg(target_os = "linux")]
    impl Drop for TestProcessGroupFallback {
        fn drop(&mut self) {
            if self.armed {
                let _ = kill_process_group(raw_pid(self.process_group), Signal::KILL);
                let _ = kill_process(raw_pid(self.pid), Signal::KILL);
            }
        }
    }

    #[cfg(target_os = "linux")]
    struct TestTokenFallback {
        token: String,
        armed: bool,
    }

    #[cfg(target_os = "linux")]
    impl TestTokenFallback {
        fn new(token: String) -> Self {
            Self { token, armed: true }
        }

        fn disarm(&mut self) {
            self.armed = false;
        }
    }

    #[cfg(target_os = "linux")]
    impl Drop for TestTokenFallback {
        fn drop(&mut self) {
            if !self.armed {
                return;
            }
            for _ in 0..20 {
                let matches = processes_with_test_token(&self.token);
                if matches.is_empty() {
                    break;
                }
                for pid in matches {
                    let _ = kill_process(raw_pid(pid), Signal::KILL);
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
    }

    #[cfg(target_os = "linux")]
    async fn wait_for_checkpoint(path: &Path) -> (u8, u32, u32) {
        let deadline = absolute_deadline(Duration::from_secs(2));
        loop {
            if let Ok(contents) = fs::read_to_string(path) {
                let mut fields = contents.split_whitespace();
                let complete = fields
                    .next()
                    .and_then(|checkpoint| checkpoint.parse::<u8>().ok())
                    .zip(fields.next().and_then(|pid| pid.parse::<u32>().ok()))
                    .zip(
                        fields
                            .next()
                            .and_then(|process_group| process_group.parse::<u32>().ok()),
                    )
                    .map(|((checkpoint, pid), process_group)| (checkpoint, pid, process_group));
                if let Some(complete) = complete {
                    assert!(fields.next().is_none(), "unexpected checkpoint fields");
                    return complete;
                }
            }
            assert!(
                Instant::now() < deadline,
                "guardian checkpoint was not published"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    #[cfg(target_os = "linux")]
    async fn wait_for_helper_identity(
        path: &std::path::Path,
        helper: &mut TestParentProcess,
    ) -> (u32, u32, u32, u32) {
        let deadline = absolute_deadline(Duration::from_secs(2));
        loop {
            if let Ok(contents) = fs::read_to_string(path) {
                let mut fields = contents.split_whitespace();
                let pid = fields
                    .next()
                    .expect("helper child pid")
                    .parse::<u32>()
                    .expect("numeric helper child pid");
                let process_group = fields
                    .next()
                    .expect("helper child process group")
                    .parse::<u32>()
                    .expect("numeric helper child process group");
                let guardian_pid = fields
                    .next()
                    .expect("helper guardian pid")
                    .parse::<u32>()
                    .expect("numeric helper guardian pid");
                let descendant_pid = fields
                    .next()
                    .expect("helper descendant pid")
                    .parse::<u32>()
                    .expect("numeric helper descendant pid");
                assert!(fields.next().is_none(), "unexpected helper state fields");
                return (pid, process_group, guardian_pid, descendant_pid);
            }
            if let Some(status) = helper.try_wait().expect("inspect test parent") {
                panic!("test parent exited before publishing child identity: {status}");
            }
            assert!(
                Instant::now() < deadline,
                "test parent did not publish child identity"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    #[cfg(target_os = "linux")]
    fn processes_with_test_token(token: &str) -> Vec<u32> {
        let expected = format!("{PARENT_DEATH_CHILD_TOKEN_ENV}={token}").into_bytes();
        let mut matches = Vec::new();
        let Ok(entries) = fs::read_dir("/proc") else {
            return matches;
        };
        for entry in entries.flatten() {
            let Ok(pid) = entry.file_name().to_string_lossy().parse::<u32>() else {
                continue;
            };
            let Ok(environment) = fs::read(entry.path().join("environ")) else {
                continue;
            };
            if environment
                .split(|byte| *byte == 0)
                .any(|field| field == expected)
            {
                matches.push(pid);
            }
        }
        matches.sort_unstable();
        matches
    }

    #[cfg(target_os = "linux")]
    async fn wait_until_test_token_gone(token: &str) {
        let deadline = absolute_deadline(Duration::from_secs(2));
        loop {
            let matches = processes_with_test_token(token);
            if matches.is_empty() {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "test processes remained after cleanup: {matches:?}"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn parent_death_helper_process() {
        let Some(state_path) = std::env::var_os(PARENT_DEATH_HELPER_STATE_ENV) else {
            return;
        };
        let token =
            std::env::var(PARENT_DEATH_HELPER_TOKEN_ENV).expect("parent-death helper token");
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("parent-death helper runtime");
        runtime.block_on(async move {
            if let Some(checkpoint) = std::env::var_os(PARENT_DEATH_CHECKPOINT_ENV) {
                let checkpoint = checkpoint
                    .to_string_lossy()
                    .parse::<u8>()
                    .expect("numeric guardian checkpoint");
                let checkpoint = match checkpoint {
                    1 => GuardianTestCheckpoint::Cloned,
                    2 => GuardianTestCheckpoint::BeforeSetpgid,
                    3 => GuardianTestCheckpoint::AfterSetpgid,
                    4 => GuardianTestCheckpoint::BeforeExec,
                    _ => panic!("unsupported guardian checkpoint {checkpoint}"),
                };
                let checkpoint_state = std::env::var_os(PARENT_DEATH_CHECKPOINT_STATE_ENV)
                    .expect("guardian checkpoint state path");
                let mut command = shell("exec sleep 30");
                command
                    .env(PARENT_DEATH_CHILD_TOKEN_ENV, token)
                    .guardian_test_checkpoint(checkpoint, checkpoint_state);
                let started = spawn_supervised(
                    command,
                    short_config(),
                    absolute_deadline(Duration::from_secs(2)),
                    |_| async { Ok::<_, SupervisedReadyError>(()) },
                )
                .await
                .expect("checkpoint supervised start");
                let mut child = started.child;
                loop {
                    tokio::time::sleep(Duration::from_millis(50)).await;
                    assert!(
                        !child.is_finished().expect("observe checkpoint child"),
                        "checkpoint child exited before its parent was killed"
                    );
                }
            }
            let descendant_path = std::path::PathBuf::from(&state_path).with_extension("descendant");
            let mut command = shell(
                "sh -c 'trap \"\" HUP TERM; sh -c '\"'\"'trap \"\" HUP TERM; sleep 30 & wait'\"'\"' & printf %s $$ > \"$DESCENDANT_STATE\"; wait' & wait",
            );
            command.env(PARENT_DEATH_CHILD_TOKEN_ENV, token);
            command.env("DESCENDANT_STATE", &descendant_path);
            let ready_path = descendant_path.clone();
            let started = spawn_supervised(
                command,
                short_config(),
                absolute_deadline(Duration::from_secs(1)),
                move |_| async move { wait_for_pid_file(&ready_path).await },
            )
            .await
            .expect("parent-death helper supervised start");
            let identity = started.child.identity();
            fs::write(
                state_path,
                format!(
                    "{} {} {} {}",
                    identity.pid(),
                    identity.process_group(),
                    identity.guardian_pid(),
                    started.ready
                ),
            )
            .expect("publish parent-death helper identity");
            let mut child = started.child;
            loop {
                tokio::time::sleep(Duration::from_millis(50)).await;
                assert!(
                    !child.is_finished().expect("observe helper child"),
                    "supervised child exited before its parent was killed"
                );
            }
        });
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn parent_sigkill_kills_child_and_restart_reaps_term_ignoring_descendant() {
        let directory = tempdir().unwrap();
        let helper_state = directory.path().join("helper.state");
        let token = directory.path().to_string_lossy().into_owned();
        let mut helper_command = StdCommand::new(std::env::current_exe().unwrap());
        helper_command
            .arg("supervised::tests::parent_death_helper_process")
            .arg("--exact")
            .arg("--nocapture")
            .env(PARENT_DEATH_HELPER_STATE_ENV, &helper_state)
            .env(PARENT_DEATH_HELPER_TOKEN_ENV, &token)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit());
        let mut helper = TestParentProcess::new(
            helper_command
                .spawn()
                .expect("spawn retained-supervisor test parent"),
        );
        let (first_pid, first_group, first_guardian, first_descendant) =
            wait_for_helper_identity(&helper_state, &mut helper).await;
        let mut first_fallback = TestProcessGroupFallback::new(first_pid, first_group);
        assert_eq!(first_pid, first_group);
        assert!(process_exists(first_pid));
        assert!(process_exists(first_guardian));
        assert!(process_exists(first_descendant));
        assert!(
            processes_with_test_token(&token).len() >= 4,
            "deep descendant tree was not established"
        );

        let mut sentinels = Vec::new();
        for _ in 0..4 {
            let sentinel = StdCommand::new("/usr/bin/setsid")
                .arg("/bin/sleep")
                .arg("5")
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .expect("spawn unrelated process-group sentinel");
            let sentinel_pid = sentinel.id();
            sentinels.push((sentinel_pid, TestParentProcess::new(sentinel)));
        }
        let sentinel_deadline = absolute_deadline(Duration::from_secs(1));
        for (sentinel_pid, _) in &sentinels {
            loop {
                if getpgid(Some(raw_pid(*sentinel_pid))) == Ok(raw_pid(*sentinel_pid)) {
                    break;
                }
                assert!(
                    Instant::now() < sentinel_deadline,
                    "unrelated sentinel {sentinel_pid} did not establish its PGID"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }

        let helper_status = helper.kill_and_wait();
        assert!(!helper_status.success(), "SIGKILLed test parent succeeded");
        for _ in 0..128 {
            let mut churn = StdCommand::new("/bin/true")
                .spawn()
                .expect("spawn concurrent PID churn");
            let churn_pid = churn.id();
            churn.wait().expect("reap concurrent PID churn");
            assert_ne!(churn_pid, first_pid, "stopped leader PID was reused");
        }
        wait_until_process_gone(first_pid).await;
        wait_until_process_gone(first_descendant).await;
        wait_until_process_gone(first_guardian).await;
        wait_until_process_group_gone(first_group).await;
        wait_until_test_token_gone(&token).await;
        first_fallback.disarm();
        for (sentinel_pid, sentinel) in &mut sentinels {
            assert_ne!(*sentinel_pid, first_group);
            assert!(
                sentinel
                    .try_wait()
                    .expect("inspect unrelated process-group sentinel")
                    .is_none(),
                "guardian cleanup killed unrelated process group {sentinel_pid}"
            );
        }

        let descendant_file = directory.path().join("restart-descendant.pid");
        let mut restart_command = shell(
            "sh -c 'trap \"\" TERM; printf %s $$ > \"$DESC_PID_FILE\"; exec sleep 30' & wait",
        );
        restart_command
            .env("DESC_PID_FILE", &descendant_file)
            .env(PARENT_DEATH_CHILD_TOKEN_ENV, &token);
        let ready_file = descendant_file.clone();
        let restarted = spawn_supervised(
            restart_command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move { wait_for_pid_file(&ready_file).await },
        )
        .await
        .expect("restart after parent-death cleanup");
        let descendant_pid = restarted.ready;
        let mut restarted_child = restarted.child;
        let restart_pid = restarted_child.identity().pid();
        let shutdown = restarted_child
            .terminate(absolute_deadline(Duration::from_millis(50)))
            .await
            .expect("TERM-to-KILL cleanup of restarted process group");
        assert!(
            shutdown.escalated_to_kill,
            "TERM-ignoring descendant did not require SIGKILL"
        );
        assert!(!process_exists(restart_pid));
        assert!(!process_exists(descendant_pid));
        wait_until_test_token_gone(&token).await;
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn parent_death_is_fail_closed_across_clone_handshake_setpgid_and_exec_windows() {
        let checkpoints = [
            GuardianTestCheckpoint::Cloned,
            GuardianTestCheckpoint::BeforeSetpgid,
            GuardianTestCheckpoint::AfterSetpgid,
            GuardianTestCheckpoint::BeforeExec,
        ];
        for (round, checkpoint) in checkpoints.into_iter().enumerate() {
            let directory = tempdir().unwrap();
            let helper_state = directory.path().join("unused-helper.state");
            let checkpoint_state = directory.path().join("checkpoint.state");
            let token = format!("{}-{round}", directory.path().display());
            let mut fallback = TestTokenFallback::new(token.clone());
            let mut helper_command = StdCommand::new(std::env::current_exe().unwrap());
            helper_command
                .arg("supervised::tests::parent_death_helper_process")
                .arg("--exact")
                .arg("--nocapture")
                .env(PARENT_DEATH_HELPER_STATE_ENV, &helper_state)
                .env(PARENT_DEATH_HELPER_TOKEN_ENV, &token)
                .env(PARENT_DEATH_CHECKPOINT_ENV, (checkpoint as u8).to_string())
                .env(PARENT_DEATH_CHECKPOINT_STATE_ENV, &checkpoint_state)
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null());
            let mut helper = TestParentProcess::new(
                helper_command
                    .spawn()
                    .expect("spawn checkpoint test parent"),
            );
            let (reported_checkpoint, pid, process_group) =
                wait_for_checkpoint(&checkpoint_state).await;
            assert_eq!(reported_checkpoint, checkpoint as u8);
            assert!(process_exists(pid));
            let helper_status = helper.kill_and_wait();
            assert!(
                !helper_status.success(),
                "SIGKILLed checkpoint parent succeeded"
            );
            wait_until_process_gone(pid).await;
            if process_group == pid {
                wait_until_process_group_gone(process_group).await;
            }
            wait_until_test_token_gone(&token).await;
            fallback.disarm();
        }
    }

    #[cfg(target_os = "linux")]
    async fn assert_invalid_guardian_control_is_fail_closed(command: Option<u8>) {
        let started = spawn_supervised(
            shell("exec sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .expect("guardian control test startup");
        let mut child = started.child;
        let identity = child.identity();
        if let Some(command) = command {
            let mut control = child.control.take().expect("guardian control pipe");
            control
                .write_all(&[command])
                .await
                .expect("write malformed guardian command");
            control.shutdown().await.expect("close malformed control");
        } else {
            child.control = None;
        }
        let deadline = absolute_deadline(Duration::from_secs(2));
        loop {
            match child.is_finished() {
                Err(SupervisedLifecycleError::Guardian(_)) => break,
                Err(error) => panic!("unexpected guardian control failure: {error}"),
                Ok(_) if Instant::now() < deadline => {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Ok(state) => panic!("guardian control failure was not observed ({state})"),
            }
        }
        child
            .terminate(absolute_deadline(Duration::from_millis(50)))
            .await
            .expect("finish fail-closed guardian control cleanup");
        assert!(!process_exists(identity.pid()));
        assert!(!process_exists(identity.guardian_pid()));
        wait_until_process_group_gone(identity.process_group()).await;
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn guardian_control_eof_and_malformed_frame_are_fail_closed() {
        assert_invalid_guardian_control_is_fail_closed(None).await;
        assert_invalid_guardian_control_is_fail_closed(Some(b'X')).await;
    }

    #[tokio::test]
    async fn typed_ready_returns_retained_process_group() {
        let started = spawn_supervised(
            shell("sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |identity| async move { Ok::<_, SupervisedReadyError>((identity, 41_u64)) },
        )
        .await
        .expect("typed startup");
        let SupervisedNativeStart {
            mut child,
            ready: (identity, generation),
        } = started;
        assert_eq!(generation, 41);
        assert_eq!(identity, child.identity());
        assert_eq!(
            getpgid(Some(raw_pid(identity.pid()))).unwrap(),
            raw_pid(identity.pid())
        );
        child
            .terminate(absolute_deadline(Duration::from_millis(100)))
            .await
            .expect("terminate process group");
    }

    #[tokio::test]
    async fn explicit_command_preserves_argv_environment_clear_and_cwd() {
        let directory = tempdir().unwrap();
        let mut command = shell("printf '%s' \"$PWD|$ONLY|${REMOVED-unset}|$1\"; sleep 0.1");
        command
            .arg("payload-argv0")
            .arg("argument value")
            .env_clear()
            .env("ONLY", "retained")
            .env("REMOVED", "must-not-survive")
            .env_remove("REMOVED")
            .current_dir(directory.path());
        let started = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .expect("explicit command startup");
        let guardian_pid = started.child.identity().guardian_pid();
        let mut child = started.child;
        let terminal = child.wait().await.expect("explicit command exit");
        assert_eq!(terminal, SupervisedTerminal::Exited { code: Some(0) });
        let output = String::from_utf8(child.logs().stdout.bytes).unwrap();
        assert_eq!(
            output,
            format!(
                "{}|retained|unset|argument value",
                directory.path().display()
            )
        );
        assert!(!process_exists(guardian_pid));
    }

    #[tokio::test]
    async fn guardian_crash_fail_closes_the_payload_group() {
        let started = spawn_supervised(
            shell("exec sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .expect("guardian-fault startup");
        let mut child = started.child;
        let identity = child.identity();
        kill_process(raw_pid(identity.guardian_pid()), Signal::KILL)
            .expect("kill dedicated guardian");
        let deadline = absolute_deadline(Duration::from_secs(1));
        let failure = loop {
            match child.is_finished() {
                Err(error) => break error,
                Ok(_) if Instant::now() < deadline => {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Ok(state) => panic!("guardian failure was not observed (payload_finished={state})"),
            }
        };
        assert!(matches!(failure, SupervisedLifecycleError::Guardian(_)));
        let shutdown = child
            .terminate(absolute_deadline(Duration::from_millis(50)))
            .await
            .expect("cleanup after guardian failure");
        assert!(shutdown
            .signal_warnings
            .iter()
            .any(|warning| warning.contains("guardian")));
        assert!(!process_exists(identity.pid()));
        assert!(!process_exists(identity.guardian_pid()));
        wait_until_process_group_gone(identity.process_group()).await;
    }

    #[cfg(target_os = "linux")]
    async fn assert_forced_guardian_startup_exit_is_reported(exit: GuardianTestExit) {
        let directory = tempdir().unwrap();
        let token = format!(
            "forced-guardian-{}-{}",
            exit as u8,
            directory.path().display()
        );
        let mut fallback = TestTokenFallback::new(token.clone());
        let mut command = shell("exec sleep 30");
        command
            .env(PARENT_DEATH_CHILD_TOKEN_ENV, &token)
            .guardian_test_exit(exit);
        let error = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| std::future::pending::<Result<(), SupervisedReadyError>>(),
        )
        .await
        .expect_err("forced guardian exit must fail startup");
        assert!(matches!(error.failure, SupervisedStartFailure::Guardian(_)));
        let rollback = error
            .rollback_error
            .expect("nonzero guardian exit must be rollback evidence");
        assert!(matches!(rollback, SupervisedLifecycleError::Guardian(_)));
        if exit == GuardianTestExit::Signaled {
            assert!(
                rollback.to_string().contains("signal: 9"),
                "guardian signal was not preserved: {rollback}"
            );
        } else {
            assert!(
                rollback.to_string().contains(&(exit as u8).to_string()),
                "guardian exit code was not preserved: {rollback}"
            );
        }
        wait_until_test_token_gone(&token).await;
        fallback.disarm();
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn guardian_cleanup_and_observation_failures_are_startup_rollback_errors() {
        assert_forced_guardian_startup_exit_is_reported(GuardianTestExit::CleanupProofFailed).await;
        assert_forced_guardian_startup_exit_is_reported(GuardianTestExit::ObservationFailed).await;
    }

    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn signaled_guardian_is_startup_rollback_error_after_group_proof() {
        assert_forced_guardian_startup_exit_is_reported(GuardianTestExit::Signaled).await;
    }

    #[tokio::test]
    async fn malformed_typed_ready_is_protocol_failure_and_rolls_back() {
        let error = spawn_supervised(
            shell("sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async {
                Err::<(), _>(SupervisedReadyError::Protocol(
                    "generation mismatch".to_string(),
                ))
            },
        )
        .await
        .expect_err("malformed readiness must fail");
        assert!(matches!(
            error.failure,
            SupervisedStartFailure::Readiness(SupervisedReadyError::Protocol(_))
        ));
        assert!(error.rollback_error.is_none(), "{error}");
    }

    #[tokio::test]
    async fn readiness_eof_is_distinct_from_protocol_and_crash() {
        let error = spawn_supervised(
            shell("sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async {
                Err::<(), _>(SupervisedReadyError::Eof(
                    "typed subscription closed".to_string(),
                ))
            },
        )
        .await
        .expect_err("EOF must fail startup");
        assert!(matches!(
            error.failure,
            SupervisedStartFailure::Readiness(SupervisedReadyError::Eof(_))
        ));
        assert!(error.rollback_error.is_none(), "{error}");
    }

    #[tokio::test]
    async fn crash_before_ready_is_terminal_failure() {
        let error = spawn_supervised(
            shell("exit 23"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| std::future::pending::<Result<(), SupervisedReadyError>>(),
        )
        .await
        .expect_err("crash must win readiness race");
        assert_eq!(
            error.failure,
            SupervisedStartFailure::TerminatedBeforeReady(SupervisedTerminal::Crashed {
                code: Some(23),
                signal: None,
            })
        );
        assert!(error.rollback_error.is_none(), "{error}");
    }

    #[tokio::test]
    async fn readiness_timeout_rolls_back_spawned_group() {
        let directory = tempdir().unwrap();
        let pid_file = directory.path().join("startup.pid");
        let mut command = shell("echo $$ > \"$START_PID_FILE\"; sleep 30");
        command.env("START_PID_FILE", &pid_file);
        let error = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_millis(80)),
            |_| std::future::pending::<Result<(), SupervisedReadyError>>(),
        )
        .await
        .expect_err("readiness timeout");
        assert_eq!(error.failure, SupervisedStartFailure::ReadyTimedOut);
        assert!(error.rollback_error.is_none(), "{error}");
        let pid = fs::read_to_string(&pid_file)
            .expect("startup pid file")
            .trim()
            .parse::<u32>()
            .unwrap();
        assert!(!process_exists(pid), "startup rollback left pid {pid}");
    }

    #[tokio::test]
    async fn readiness_at_absolute_deadline_loses_to_timeout() {
        let ready_deadline = absolute_deadline(Duration::from_millis(50));
        let error = spawn_supervised(
            shell("sleep 30"),
            short_config(),
            ready_deadline,
            move |_| async move {
                tokio::time::sleep_until(tokio::time::Instant::from_std(ready_deadline)).await;
                Ok::<_, SupervisedReadyError>(())
            },
        )
        .await
        .expect_err("deadline must win simultaneous READY");
        assert_eq!(error.failure, SupervisedStartFailure::ReadyTimedOut);
        assert!(error.rollback_error.is_none(), "{error}");
    }

    #[tokio::test]
    async fn post_spawn_setup_crossing_deadline_rejects_immediate_ready() {
        let directory = tempdir().unwrap();
        let pid_file = directory.path().join("spawned.pid");
        let mut command = shell("echo $$ > \"$SPAWNED_PID_FILE\"; sleep 30");
        command.env("SPAWNED_PID_FILE", &pid_file);
        let ready_deadline = absolute_deadline(Duration::from_millis(30));
        let error = spawn_supervised(command, short_config(), ready_deadline, |_| {
            thread::sleep(Duration::from_millis(60));
            std::future::ready(Ok::<_, SupervisedReadyError>(()))
        })
        .await
        .expect_err("post-spawn overrun must not accept immediate READY");
        assert_eq!(error.failure, SupervisedStartFailure::ReadyTimedOut);
        assert!(error.rollback_error.is_none(), "{error}");
        let pid = fs::read_to_string(pid_file)
            .expect("process proved it was spawned")
            .trim()
            .parse::<u32>()
            .unwrap();
        assert!(!process_exists(pid), "overrun rollback left pid {pid}");
    }

    #[tokio::test]
    async fn oversized_diagnostic_capacity_is_rejected_before_spawn() {
        let directory = tempdir().unwrap();
        let sentinel = directory.path().join("spawned");
        let mut command = shell("printf spawned > \"$SPAWN_SENTINEL\"");
        command.env("SPAWN_SENTINEL", &sentinel);
        let mut config = short_config();
        config.stdout_capacity = usize::MAX;
        let error = spawn_supervised(
            command,
            config,
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .expect_err("oversized capacity must be typed invalid config");
        assert!(matches!(
            error.failure,
            SupervisedStartFailure::InvalidConfig(_)
        ));
        assert!(!sentinel.exists(), "invalid config spawned the process");
    }

    #[tokio::test]
    async fn ignored_term_escalates_to_group_kill() {
        let directory = tempdir().unwrap();
        let ready_file = directory.path().join("term-handler.ready");
        let mut command =
            shell("trap '' TERM; printf ready > \"$TERM_READY_FILE\"; while :; do sleep 1; done");
        command.env("TERM_READY_FILE", &ready_file);
        let readiness_file = ready_file.clone();
        let started = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move {
                let deadline = absolute_deadline(Duration::from_secs(1));
                while !readiness_file.exists() {
                    if Instant::now() >= deadline {
                        return Err(SupervisedReadyError::Transport(
                            "TERM handler readiness file was not published".to_string(),
                        ));
                    }
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Ok(())
            },
        )
        .await
        .unwrap();
        let mut child = started.child;
        let pid = child.identity().pid();
        let shutdown = child
            .terminate(absolute_deadline(Duration::from_millis(40)))
            .await
            .expect("KILL fallback");
        assert!(shutdown.escalated_to_kill);
        assert!(matches!(
            shutdown.terminal,
            SupervisedTerminal::Crashed {
                signal: Some(9),
                ..
            }
        ));
        assert!(!process_exists(pid));
    }

    #[tokio::test]
    async fn terminate_reaps_direct_child_and_descendant_group() {
        let directory = tempdir().unwrap();
        let descendant_file = directory.path().join("descendant.pid");
        let mut command = shell("sleep 30 & echo $! > \"$DESC_PID_FILE\"; wait");
        command.env("DESC_PID_FILE", &descendant_file);
        let ready_file = descendant_file.clone();
        let started = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move { wait_for_pid_file(&ready_file).await },
        )
        .await
        .expect("descendant ready");
        let descendant_pid = started.ready;
        let mut child = started.child;
        let direct_pid = child.identity().pid();
        let first = child
            .terminate(absolute_deadline(Duration::from_millis(200)))
            .await
            .expect("whole group termination");
        let repeated = child
            .terminate(absolute_deadline(Duration::from_millis(1)))
            .await
            .expect("cached repeated termination");
        assert_eq!(repeated, first);
        assert!(!process_exists(direct_pid));
        assert!(!process_exists(descendant_pid));
    }

    #[tokio::test]
    async fn wait_keeps_exited_leader_pinned_until_descendant_cleanup() {
        let directory = tempdir().unwrap();
        let descendant_file = directory.path().join("lingering.pid");
        let exit_file = directory.path().join("leader.exit");
        let mut command = shell(
            "sh -c 'trap \"\" HUP TERM; while :; do sleep 1; done' & echo $! > \"$DESC_PID_FILE\"; while [ ! -f \"$EXIT_FILE\" ]; do sleep 0.01; done; sleep 0.1; exit 0",
        );
        command.env("DESC_PID_FILE", &descendant_file);
        command.env("EXIT_FILE", &exit_file);
        let ready_descendant = descendant_file.clone();
        let release_leader = exit_file.clone();
        let started = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move {
                let descendant = wait_for_pid_file(&ready_descendant).await?;
                fs::write(&release_leader, b"exit").map_err(|error| {
                    SupervisedReadyError::Transport(format!("release leader: {error}"))
                })?;
                Ok(descendant)
            },
        )
        .await
        .expect("leader/descendant startup");
        let descendant_pid = started.ready;
        let mut child = started.child;
        let leader_pid = child.identity().pid();
        let observation_deadline = absolute_deadline(Duration::from_secs(1));
        while !child.is_finished().unwrap() && Instant::now() < observation_deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(child.is_finished().unwrap(), "leader did not exit");
        assert!(
            std::path::Path::new(&format!("/proc/{leader_pid}")).exists(),
            "WNOWAIT observation reaped the leader before group cleanup"
        );
        assert!(process_exists(descendant_pid), "descendant did not linger");
        for _ in 0..128 {
            let mut churn = StdCommand::new("/bin/true")
                .spawn()
                .expect("spawn PID reuse churn");
            let churn_pid = churn.id();
            churn.wait().expect("reap PID reuse churn");
            assert_ne!(churn_pid, leader_pid, "pinned leader PID was reused");
        }
        assert_eq!(
            getpgid(Some(raw_pid(leader_pid))).unwrap(),
            raw_pid(leader_pid),
            "pinned leader no longer identified the retained process group"
        );
        let terminal = child.wait().await.expect("wait cleanup and reap");
        assert_eq!(terminal, SupervisedTerminal::Exited { code: Some(0) });
        assert!(!process_exists(leader_pid));
        assert!(!process_exists(descendant_pid));
    }

    #[tokio::test]
    async fn diagnostic_pipes_are_bounded_while_continuing_to_drain() {
        let started = spawn_supervised(
            shell(
                "i=0; while [ $i -lt 4096 ]; do printf 0123456789abcdef; printf fedcba9876543210 >&2; i=$((i + 1)); done; sleep 30",
            ),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .unwrap();
        let mut child = started.child;
        let observation_deadline = absolute_deadline(Duration::from_secs(1));
        loop {
            let logs = child.logs();
            if logs.stdout.total_bytes >= 64 * 1024 && logs.stderr.total_bytes >= 64 * 1024 {
                assert_eq!(logs.stdout.bytes.len(), 1024);
                assert_eq!(logs.stderr.bytes.len(), 1024);
                assert!(logs.stdout.truncated);
                assert!(logs.stderr.truncated);
                break;
            }
            assert!(Instant::now() < observation_deadline, "logs did not drain");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        child
            .terminate(absolute_deadline(Duration::from_millis(100)))
            .await
            .expect("terminate noisy child");
    }

    #[tokio::test]
    async fn wait_and_is_finished_distinguish_normal_exit() {
        let started = spawn_supervised(
            shell("sleep 0.1; exit 0"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .unwrap();
        let mut child = started.child;
        assert!(!child.is_finished().unwrap());
        let terminal = child.wait().await.unwrap();
        assert_eq!(terminal, SupervisedTerminal::Exited { code: Some(0) });
        assert!(child.is_finished().unwrap());
    }

    #[tokio::test]
    async fn drop_is_fail_closed_for_a_live_group() {
        let started = spawn_supervised(
            shell("sleep 30"),
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            |_| async { Ok::<_, SupervisedReadyError>(()) },
        )
        .await
        .unwrap();
        let pid = started.child.identity().pid();
        drop(started.child);
        let deadline = absolute_deadline(Duration::from_secs(1));
        while process_exists(pid) && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(!process_exists(pid), "Drop left live pid {pid}");
    }

    #[tokio::test]
    async fn drop_detached_reaper_cleans_descendant_and_leader() {
        let directory = tempdir().unwrap();
        let descendant_file = directory.path().join("drop-descendant.pid");
        let mut command = shell("sleep 30 & echo $! > \"$DROP_DESC_PID_FILE\"; wait");
        command.env("DROP_DESC_PID_FILE", &descendant_file);
        let ready_file = descendant_file.clone();
        let started = spawn_supervised(
            command,
            short_config(),
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move { wait_for_pid_file(&ready_file).await },
        )
        .await
        .expect("drop descendant startup");
        let descendant_pid = started.ready;
        let leader_pid = started.child.identity().pid();
        drop(started.child);
        wait_until_process_gone(leader_pid).await;
        wait_until_process_gone(descendant_pid).await;
    }

    #[tokio::test]
    async fn repeated_terminate_preserves_diagnostic_drain_failure() {
        assert!(std::path::Path::new("/usr/bin/setsid").exists());
        let directory = tempdir().unwrap();
        let escaped_file = directory.path().join("escaped.pid");
        let mut command = shell(
            "/usr/bin/setsid /bin/sh -c 'echo $$ > \"$ESCAPED_PID_FILE\"; exec sleep 30' & while [ ! -f \"$ESCAPED_PID_FILE\" ]; do sleep 0.01; done; while :; do sleep 1; done",
        );
        command.env("ESCAPED_PID_FILE", &escaped_file);
        let ready_file = escaped_file.clone();
        let mut config = short_config();
        config.kill_reap_timeout = Duration::from_millis(50);
        let started = spawn_supervised(
            command,
            config,
            absolute_deadline(Duration::from_secs(1)),
            move |_| async move { wait_for_pid_file(&ready_file).await },
        )
        .await
        .expect("escaped diagnostic holder startup");
        let escaped_pid = started.ready;
        let mut child = started.child;
        let first = child
            .terminate(absolute_deadline(Duration::from_millis(50)))
            .await
            .expect_err("escaped pipe holder must fail drain proof");
        assert!(matches!(
            first,
            SupervisedLifecycleError::DiagnosticDrain(_)
        ));
        let repeated = child
            .terminate(absolute_deadline(Duration::from_millis(1)))
            .await
            .expect_err("repeat must retain drain failure");
        assert_eq!(repeated, first);
        kill_process(raw_pid(escaped_pid), Signal::KILL).expect("kill escaped test process");
        wait_until_process_gone(escaped_pid).await;
    }
}
