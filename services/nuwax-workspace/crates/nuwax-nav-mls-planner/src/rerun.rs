// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Optional Rerun SDK process adapter for MLS planner debug events.

use crate::debug_events::{debug_events, DebugEvent};
use crate::transformer::PlanObservation;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};
use thiserror::Error;

const POLL_INTERVAL: Duration = Duration::from_millis(5);
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(30);
const DEFAULT_STDOUT_LIMIT: usize = 64 * 1024;
const DEFAULT_STDERR_LIMIT: usize = 64 * 1024;
const DEFAULT_REQUEST_LIMIT: usize = 16 * 1024 * 1024;
const REQUEST_FIXED_BUDGET: usize = 2 * 1024;
const REQUEST_BYTES_PER_POINT_BUDGET: usize = 96;
const WORKER_EVENT_POINT_LIMIT: usize = 100_000;

/// Process boundary used to send planner artifacts to the optional Python SDK.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RerunProcessConfig {
    /// Python interpreter used to execute the worker.
    pub python: PathBuf,
    /// Worker implementing the Rerun 0.32 API boundary.
    pub worker: PathBuf,
    /// Optional `.rrd` output. When absent, the worker starts a viewer.
    pub output: Option<PathBuf>,
    /// Rerun application identifier.
    pub application_id: String,
    /// Rerun recording identifier.
    pub recording_id: String,
    /// Also start a viewer when writing an output file.
    pub spawn_viewer: bool,
    /// Additional worker environment, primarily for controlled SDK discovery.
    pub environment: BTreeMap<String, String>,
    /// Hard deadline covering worker execution and all pipe I/O.
    pub timeout: Duration,
    /// Maximum accepted worker stdout bytes.
    pub max_stdout_bytes: usize,
    /// Maximum accepted worker stderr bytes.
    pub max_stderr_bytes: usize,
    /// Maximum NDJSON request bytes sent to the worker.
    pub max_request_bytes: usize,
}

impl Default for RerunProcessConfig {
    fn default() -> Self {
        Self {
            python: std::env::var_os("NUWAX_MLS_RERUN_PYTHON")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from("python3")),
            worker: PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("python")
                .join("mls_rerun.py"),
            output: None,
            application_id: "nuwax_mls_planner".to_string(),
            recording_id: "mls_plan".to_string(),
            spawn_viewer: false,
            environment: BTreeMap::new(),
            timeout: DEFAULT_TIMEOUT,
            max_stdout_bytes: DEFAULT_STDOUT_LIMIT,
            max_stderr_bytes: DEFAULT_STDERR_LIMIT,
            max_request_bytes: DEFAULT_REQUEST_LIMIT,
        }
    }
}

/// Successful process export summary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RerunExportSummary {
    /// Number of ordered planner events sent to the worker.
    pub events: usize,
    /// Output selected by the caller, if any.
    pub output: Option<PathBuf>,
}

/// MLS Rerun process-boundary failure.
#[derive(Debug, Error)]
pub enum RerunExportError {
    /// Required configuration was empty or inconsistent.
    #[error("invalid MLS Rerun configuration: {0}")]
    InvalidConfig(String),
    /// Worker process or pipe I/O failed.
    #[error("MLS Rerun worker I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// An event could not be encoded as JSON.
    #[error("MLS Rerun event encoding failed: {0}")]
    Json(#[from] serde_json::Error),
    /// Worker rejected the request or could not load the optional SDK.
    #[error("MLS Rerun worker failed with status {status}: {stderr}")]
    WorkerFailed {
        /// Process exit code, or `-1` when terminated by a signal.
        status: i32,
        /// Bounded diagnostic text emitted by the worker.
        stderr: String,
    },
    /// The worker or a descendant did not close all process pipes in time.
    #[error("MLS Rerun worker timed out after {0:?}")]
    TimedOut(Duration),
    /// A diagnostic stream exceeded its configured bound.
    #[error("MLS Rerun worker {stream} exceeded {limit} bytes")]
    OutputLimit {
        /// Name of the stream that exceeded its limit.
        stream: &'static str,
        /// Configured maximum number of bytes.
        limit: usize,
    },
    /// The encoded planner event request exceeded its configured bound.
    #[error("MLS Rerun request exceeded {limit} bytes")]
    RequestLimit {
        /// Configured maximum request bytes.
        limit: usize,
    },
    /// Termination or reaping failed while preserving the primary failure.
    #[error("{primary}; cleanup also failed: {cleanup}")]
    CleanupFailed {
        /// Timeout, output-limit, or pipe failure that triggered cleanup.
        primary: Box<RerunExportError>,
        /// Termination and/or reap diagnostics.
        cleanup: String,
    },
}

/// Export one real planner observation through the optional Rerun SDK worker.
pub fn export_plan_to_rerun(
    observation: &PlanObservation,
    config: &RerunProcessConfig,
) -> Result<RerunExportSummary, RerunExportError> {
    validate_config(config)?;
    validate_observation(observation, config.max_request_bytes)?;

    let mut command = Command::new(&config.python);
    command
        .arg(&config.worker)
        .arg("--application-id")
        .arg(&config.application_id)
        .arg("--recording-id")
        .arg(&config.recording_id)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .envs(&config.environment);
    if let Some(output) = &config.output {
        command.arg("--output").arg(output);
    }
    if config.spawn_viewer || config.output.is_none() {
        command.arg("--spawn");
    }

    let events = debug_events(observation);
    let mut request = LimitedRequest::new(config.max_request_bytes);
    for event in &events {
        let result = serde_json::to_writer(
            &mut request,
            &json!({
                "timestamp": observation.timestamp,
                "event": event_json(event),
            }),
        );
        if let Err(error) = result {
            if request.exceeded() {
                return Err(RerunExportError::RequestLimit {
                    limit: config.max_request_bytes,
                });
            }
            return Err(RerunExportError::Json(error));
        }
        request.push_byte(b'\n')?;
    }
    let request = request.into_inner();
    if request.len() > config.max_request_bytes {
        return Err(RerunExportError::RequestLimit {
            limit: config.max_request_bytes,
        });
    }

    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    let mut child = command.spawn()?;
    let stdin = child
        .stdin
        .take()
        .ok_or_else(|| RerunExportError::InvalidConfig("worker stdin was not piped".to_string()))?;
    let stdout = child.stdout.take().ok_or_else(|| {
        RerunExportError::InvalidConfig("worker stdout was not piped".to_string())
    })?;
    let stderr = child.stderr.take().ok_or_else(|| {
        RerunExportError::InvalidConfig("worker stderr was not piped".to_string())
    })?;

    let writer_rx = spawn_writer(stdin, request);
    let stdout_rx = spawn_reader(stdout, config.max_stdout_bytes);
    let stderr_rx = spawn_reader(stderr, config.max_stderr_bytes);
    let deadline = Instant::now()
        .checked_add(config.timeout)
        .ok_or_else(|| RerunExportError::InvalidConfig("timeout is too large".to_string()))?;
    let ProcessCompletion { status, stderr } = wait_for_completion(
        &mut child, deadline, config, &writer_rx, &stdout_rx, &stderr_rx,
    )?;

    if !status.success() {
        return Err(RerunExportError::WorkerFailed {
            status: status.code().unwrap_or(-1),
            stderr: String::from_utf8_lossy(&stderr).trim().to_string(),
        });
    }

    Ok(RerunExportSummary {
        events: events.len(),
        output: config.output.clone(),
    })
}

fn validate_config(config: &RerunProcessConfig) -> Result<(), RerunExportError> {
    if config.python.as_os_str().is_empty() {
        return Err(RerunExportError::InvalidConfig(
            "python interpreter is empty".to_string(),
        ));
    }
    if config.worker.as_os_str().is_empty() {
        return Err(RerunExportError::InvalidConfig(
            "worker path is empty".to_string(),
        ));
    }
    if config.application_id.trim().is_empty() || config.recording_id.trim().is_empty() {
        return Err(RerunExportError::InvalidConfig(
            "application_id and recording_id must be non-empty".to_string(),
        ));
    }
    if config.timeout.is_zero() {
        return Err(RerunExportError::InvalidConfig(
            "timeout must be positive".to_string(),
        ));
    }
    if Instant::now().checked_add(config.timeout).is_none() {
        return Err(RerunExportError::InvalidConfig(
            "timeout is too large".to_string(),
        ));
    }
    if config.max_stdout_bytes == 0 || config.max_stderr_bytes == 0 || config.max_request_bytes == 0
    {
        return Err(RerunExportError::InvalidConfig(
            "request, stdout and stderr limits must be positive".to_string(),
        ));
    }
    if config.max_stdout_bytes == usize::MAX
        || config.max_stderr_bytes == usize::MAX
        || config.max_request_bytes == usize::MAX
    {
        return Err(RerunExportError::InvalidConfig(
            "request, stdout and stderr limits are too large".to_string(),
        ));
    }
    Ok(())
}

fn validate_observation(
    observation: &PlanObservation,
    request_limit: usize,
) -> Result<(), RerunExportError> {
    for (label, count) in [
        ("path", observation.path.len()),
        ("voxel_map", observation.artifacts.voxel_map.len()),
        (
            "surface_clearance",
            observation.artifacts.surface_clearance.len(),
        ),
        ("nodes", observation.artifacts.nodes.len()),
    ] {
        if count > WORKER_EVENT_POINT_LIMIT {
            return Err(invalid_observation(&format!(
                "{label} exceeds the worker {WORKER_EVENT_POINT_LIMIT}-point event limit"
            )));
        }
    }
    let edge_points = observation
        .artifacts
        .node_edges
        .len()
        .checked_mul(2)
        .ok_or_else(|| invalid_observation("node_edges point count overflowed"))?;
    if edge_points > WORKER_EVENT_POINT_LIMIT {
        return Err(invalid_observation(&format!(
            "node_edges exceed the worker {WORKER_EVENT_POINT_LIMIT}-point event limit"
        )));
    }

    if !observation.timestamp.is_finite() {
        return Err(invalid_observation("timestamp must be finite"));
    }
    if !point_is_finite(observation.start) {
        return Err(invalid_observation("start must contain finite coordinates"));
    }
    if !observation.path.iter().copied().all(point_is_finite)
        || !observation
            .artifacts
            .voxel_map
            .iter()
            .copied()
            .all(point_is_finite)
        || !observation
            .artifacts
            .nodes
            .iter()
            .copied()
            .all(point_is_finite)
        || !observation
            .artifacts
            .surface_clearance
            .iter()
            .all(|(point, clearance)| point_is_finite(*point) && clearance.is_finite())
        || !observation
            .artifacts
            .node_edges
            .iter()
            .all(|edge| point_is_finite(edge.from) && point_is_finite(edge.to))
    {
        return Err(invalid_observation(
            "planner artifacts must contain only finite coordinates and clearance",
        ));
    }
    if !observation.timings.update_ms.is_finite()
        || !observation.timings.plan_ms.is_finite()
        || !observation.timings.total_ms.is_finite()
    {
        return Err(invalid_observation("timings must be finite"));
    }

    let points = 1usize
        .checked_add(observation.path.len())
        .and_then(|count| count.checked_add(observation.artifacts.voxel_map.len()))
        .and_then(|count| count.checked_add(observation.artifacts.surface_clearance.len()))
        .and_then(|count| count.checked_add(observation.artifacts.nodes.len()))
        .and_then(|count| {
            observation
                .artifacts
                .node_edges
                .len()
                .checked_mul(2)
                .and_then(|edges| count.checked_add(edges))
        })
        .ok_or(RerunExportError::RequestLimit {
            limit: request_limit,
        })?;
    let estimated_bytes = points
        .checked_mul(REQUEST_BYTES_PER_POINT_BUDGET)
        .and_then(|bytes| bytes.checked_add(REQUEST_FIXED_BUDGET))
        .ok_or(RerunExportError::RequestLimit {
            limit: request_limit,
        })?;
    if estimated_bytes > request_limit {
        return Err(RerunExportError::RequestLimit {
            limit: request_limit,
        });
    }
    Ok(())
}

fn invalid_observation(message: &str) -> RerunExportError {
    RerunExportError::InvalidConfig(format!("observation {message}"))
}

fn point_is_finite((x, y, z): (f32, f32, f32)) -> bool {
    x.is_finite() && y.is_finite() && z.is_finite()
}

struct LimitedRequest {
    bytes: Vec<u8>,
    limit: usize,
    exceeded: bool,
}

impl LimitedRequest {
    fn new(limit: usize) -> Self {
        Self {
            bytes: Vec::with_capacity(limit.min(64 * 1024)),
            limit,
            exceeded: false,
        }
    }

    fn push_byte(&mut self, byte: u8) -> Result<(), RerunExportError> {
        if self.bytes.len() >= self.limit {
            self.exceeded = true;
            return Err(RerunExportError::RequestLimit { limit: self.limit });
        }
        self.bytes.push(byte);
        Ok(())
    }

    fn exceeded(&self) -> bool {
        self.exceeded
    }

    fn into_inner(self) -> Vec<u8> {
        self.bytes
    }
}

impl Write for LimitedRequest {
    fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
        let Some(next_len) = self.bytes.len().checked_add(buffer.len()) else {
            self.exceeded = true;
            return Err(std::io::Error::new(
                std::io::ErrorKind::FileTooLarge,
                "MLS Rerun request size overflow",
            ));
        };
        if next_len > self.limit {
            self.exceeded = true;
            return Err(std::io::Error::new(
                std::io::ErrorKind::FileTooLarge,
                "MLS Rerun request limit exceeded",
            ));
        }
        self.bytes.extend_from_slice(buffer);
        Ok(buffer.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

struct ProcessCompletion {
    status: ExitStatus,
    stderr: Vec<u8>,
}

fn spawn_writer(
    mut stdin: impl Write + Send + 'static,
    request: Vec<u8>,
) -> Receiver<std::io::Result<()>> {
    let (sender, receiver) = mpsc::channel();
    let _ = thread::spawn(move || {
        let result = stdin.write_all(&request);
        drop(stdin);
        let _ = sender.send(result);
    });
    receiver
}

fn spawn_reader(
    reader: impl Read + Send + 'static,
    limit: usize,
) -> Receiver<std::io::Result<Vec<u8>>> {
    let (sender, receiver) = mpsc::channel();
    let _ = thread::spawn(move || {
        let mut bytes = Vec::new();
        let result = reader
            .take(limit as u64 + 1)
            .read_to_end(&mut bytes)
            .map(|_| bytes);
        let _ = sender.send(result);
    });
    receiver
}

fn wait_for_completion(
    child: &mut Child,
    deadline: Instant,
    config: &RerunProcessConfig,
    writer_rx: &Receiver<std::io::Result<()>>,
    stdout_rx: &Receiver<std::io::Result<Vec<u8>>>,
    stderr_rx: &Receiver<std::io::Result<Vec<u8>>>,
) -> Result<ProcessCompletion, RerunExportError> {
    let mut status = None;
    let mut writer = None;
    let mut stdout = None;
    let mut stderr = None;

    loop {
        if let Err(error) = receive_once(writer_rx, &mut writer, "stdin writer") {
            return Err(with_cleanup(child, error));
        }
        if let Err(error) = receive_once(stdout_rx, &mut stdout, "stdout reader") {
            return Err(with_cleanup(child, error));
        }
        if let Err(error) = receive_once(stderr_rx, &mut stderr, "stderr reader") {
            return Err(with_cleanup(child, error));
        }

        if stdout
            .as_ref()
            .is_some_and(|bytes| bytes.len() > config.max_stdout_bytes)
        {
            return Err(with_cleanup(
                child,
                RerunExportError::OutputLimit {
                    stream: "stdout",
                    limit: config.max_stdout_bytes,
                },
            ));
        }
        if stderr
            .as_ref()
            .is_some_and(|bytes| bytes.len() > config.max_stderr_bytes)
        {
            return Err(with_cleanup(
                child,
                RerunExportError::OutputLimit {
                    stream: "stderr",
                    limit: config.max_stderr_bytes,
                },
            ));
        }

        if status.is_none() {
            match child.try_wait() {
                Ok(Some(exit_status)) => status = Some(exit_status),
                Ok(None) => {}
                Err(error) => {
                    return Err(with_cleanup(child, error.into()));
                }
            }
        }

        if writer.is_some() && stdout.is_some() && stderr.is_some() && status.is_some() {
            let status = child.wait()?;
            let stderr = match stderr.take() {
                Some(stderr) => stderr,
                None => unreachable!("stderr presence checked"),
            };
            return Ok(ProcessCompletion { status, stderr });
        }
        if Instant::now() >= deadline {
            return Err(with_cleanup(
                child,
                RerunExportError::TimedOut(config.timeout),
            ));
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn receive_once<T>(
    receiver: &Receiver<std::io::Result<T>>,
    slot: &mut Option<T>,
    operation: &'static str,
) -> Result<(), RerunExportError> {
    if slot.is_some() {
        return Ok(());
    }
    match receiver.try_recv() {
        Ok(Ok(value)) => *slot = Some(value),
        Ok(Err(error)) => return Err(error.into()),
        Err(TryRecvError::Empty) => {}
        Err(TryRecvError::Disconnected) => {
            return Err(std::io::Error::other(format!("{operation} panicked")).into());
        }
    }
    Ok(())
}

fn with_cleanup(child: &mut Child, primary: RerunExportError) -> RerunExportError {
    match terminate_process_tree(child) {
        Ok(()) => primary,
        Err(cleanup) => RerunExportError::CleanupFailed {
            primary: Box::new(primary),
            cleanup: cleanup.to_string(),
        },
    }
}

#[cfg(unix)]
fn terminate_process_tree(child: &mut Child) -> std::io::Result<()> {
    let mut failures = Vec::new();
    if let Err(error) = kill_process_group(child.id()) {
        failures.push(format!("process-group kill: {error}"));
        if let Err(error) = child.kill() {
            failures.push(format!("child kill fallback: {error}"));
        }
    }
    if let Err(error) = child.wait() {
        failures.push(format!("child reap: {error}"));
    }
    cleanup_result(failures)
}

#[cfg(not(unix))]
fn terminate_process_tree(child: &mut Child) -> std::io::Result<()> {
    let mut failures = Vec::new();
    if let Err(error) = child.kill() {
        failures.push(format!("child kill: {error}"));
    }
    if let Err(error) = child.wait() {
        failures.push(format!("child reap: {error}"));
    }
    cleanup_result(failures)
}

fn cleanup_result(failures: Vec<String>) -> std::io::Result<()> {
    if failures.is_empty() {
        Ok(())
    } else {
        Err(std::io::Error::other(failures.join("; ")))
    }
}

#[cfg(unix)]
#[allow(unsafe_code)]
fn kill_process_group(process_group_id: u32) -> std::io::Result<()> {
    use std::os::raw::c_int;

    const SIGKILL: c_int = 9;
    unsafe extern "C" {
        #[link_name = "kill"]
        fn posix_kill(pid: c_int, signal: c_int) -> c_int;
    }

    let process_group_id = c_int::try_from(process_group_id)
        .map_err(|_| std::io::Error::other("process-group id exceeds pid_t"))?;
    // SAFETY: POSIX kill accepts any integer pid/signal pair. The negative pid targets the
    // dedicated process group created by CommandExt::process_group(0); no pointers cross FFI.
    let result = unsafe { posix_kill(-process_group_id, SIGKILL) };
    if result == 0 {
        return Ok(());
    }
    let error = std::io::Error::last_os_error();
    if error.kind() == std::io::ErrorKind::NotFound {
        Ok(())
    } else {
        Err(error)
    }
}

fn event_json(event: &DebugEvent) -> Value {
    match event {
        DebugEvent::Start(point) => json!({
            "kind": "points3d",
            "entity": "world/start",
            "positions": [point],
            "colors": [[0, 255, 0]],
            "radii": [0.1],
        }),
        DebugEvent::VoxelMap(points) => json!({
            "kind": "points3d",
            "entity": "world/voxel_map",
            "positions": points,
            "colors": [[180, 125, 125]],
            "radii": [0.025],
        }),
        DebugEvent::SurfaceClearance(points) => {
            let positions: Vec<_> = points.iter().map(|(point, _)| point).collect();
            let colors: Vec<_> = points
                .iter()
                .map(|(_, clearance)| clearance_color(*clearance))
                .collect();
            json!({
                "kind": "points3d",
                "entity": "world/surface_map",
                "positions": positions,
                "colors": colors,
                "radii": [0.025],
            })
        }
        DebugEvent::Nodes(points) => json!({
            "kind": "points3d",
            "entity": "world/nodes",
            "positions": points,
            "colors": [[255, 200, 0]],
            "radii": [0.05],
        }),
        DebugEvent::NodeEdges(edges) => {
            let strips: Vec<_> = edges.iter().map(|edge| [edge.from, edge.to]).collect();
            json!({
                "kind": "line_strips3d",
                "entity": "world/node_edges",
                "strips": strips,
            })
        }
        DebugEvent::Path(points) => json!({
            "kind": "line_strips3d",
            "entity": "world/path",
            "strips": [points],
            "colors": [[0, 255, 0]],
            "radii": [0.05],
        }),
        DebugEvent::Metrics {
            update_ms,
            plan_ms,
            total_ms,
            voxels,
            planned,
        } => json!({
            "kind": "metrics",
            "values": {
                "update_ms": update_ms,
                "plan_ms": plan_ms,
                "total_ms": total_ms,
                "voxels": voxels,
                "planned": usize::from(*planned),
            },
        }),
    }
}

fn clearance_color(clearance: f32) -> [u8; 3] {
    if !clearance.is_finite() {
        return [150, 200, 255];
    }
    let ratio = (clearance / 2.0).clamp(0.0, 1.0);
    [
        (4.0 + ratio * 146.0) as u8,
        (8.0 + ratio * 192.0) as u8,
        (48.0 + ratio * 207.0) as u8,
    ]
}
