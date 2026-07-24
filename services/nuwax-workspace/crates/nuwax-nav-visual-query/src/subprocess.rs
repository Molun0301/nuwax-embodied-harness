//! Local subprocess implementation of the visual-language bbox model contract.

use std::ffi::OsString;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::{ImageView, VisualQueryError, VlBBoxModel};

const POLL_INTERVAL: Duration = Duration::from_millis(5);
pub(crate) const STDERR_LIMIT: usize = 8 * 1024;
const REQUEST_FIXED_OVERHEAD: usize = 128;

/// Configuration for one-process-per-query local model execution.
#[derive(Debug, Clone)]
pub struct SubprocessModelConfig {
    /// Executable path. It is invoked directly, never through a shell.
    pub program: PathBuf,
    /// Fixed executable arguments.
    pub args: Vec<OsString>,
    /// Hard query deadline after which the child is killed and reaped.
    pub timeout: Duration,
    /// Maximum accepted stdout bytes.
    pub max_response_bytes: usize,
    /// Maximum raw image bytes accepted before JSON serialization.
    pub max_image_bytes: usize,
    /// Maximum decoded image pixels accepted before JSON serialization.
    pub max_image_pixels: u64,
    /// Maximum serialized JSON request bytes.
    pub max_request_bytes: usize,
}

impl SubprocessModelConfig {
    /// Validate a subprocess configuration.
    pub fn validate(&self) -> Result<(), VisualQueryError> {
        if self.program.as_os_str().is_empty() {
            return Err(VisualQueryError::ModelFailed(
                "model program path is empty".into(),
            ));
        }
        if self.timeout.is_zero() {
            return Err(VisualQueryError::ModelFailed(
                "model timeout must be positive".into(),
            ));
        }
        if self.max_response_bytes == 0 {
            return Err(VisualQueryError::ModelFailed(
                "max_response_bytes must be positive".into(),
            ));
        }
        if self.max_response_bytes == usize::MAX {
            return Err(VisualQueryError::ModelFailed(
                "max_response_bytes is too large".into(),
            ));
        }
        if self.max_image_bytes == 0 || self.max_request_bytes == 0 || self.max_image_pixels == 0 {
            return Err(VisualQueryError::ModelFailed(
                "image and request limits must be positive".into(),
            ));
        }
        if self.max_image_bytes == usize::MAX
            || self.max_request_bytes == usize::MAX
            || self.max_image_pixels == u64::MAX
        {
            return Err(VisualQueryError::ModelFailed(
                "image and request limits are too large".into(),
            ));
        }
        Ok(())
    }
}

/// Real local model owner that exchanges one JSON request and raw text response.
///
/// Request schema: `{"width":u32,"height":u32,"prompt":string,"image":[u8...]}`.
/// The child writes the same free-form response accepted by the Dimos parser to stdout.
pub struct SubprocessVlBBoxModel {
    config: SubprocessModelConfig,
}

impl SubprocessVlBBoxModel {
    /// Construct a validated backend.
    pub fn new(config: SubprocessModelConfig) -> Result<Self, VisualQueryError> {
        config.validate()?;
        Ok(Self { config })
    }

    /// Current immutable process configuration.
    #[must_use]
    pub fn config(&self) -> &SubprocessModelConfig {
        &self.config
    }
}

#[derive(Serialize)]
struct QueryRequest<'a> {
    width: u32,
    height: u32,
    prompt: &'a str,
    image: &'a [u8],
}

impl VlBBoxModel for SubprocessVlBBoxModel {
    fn query(&self, image: ImageView<'_>, prompt: &str) -> Result<String, VisualQueryError> {
        if image.width == 0 || image.height == 0 {
            return Err(VisualQueryError::ModelFailed(
                "image dimensions must be positive".into(),
            ));
        }
        let pixels = u64::from(image.width)
            .checked_mul(u64::from(image.height))
            .ok_or_else(|| VisualQueryError::ModelFailed("image dimensions overflow".into()))?;
        if pixels > self.config.max_image_pixels {
            return Err(VisualQueryError::ModelFailed(format!(
                "image has {pixels} pixels, limit is {}",
                self.config.max_image_pixels
            )));
        }
        if image.data.is_empty() {
            return Err(VisualQueryError::ModelFailed(
                "image data must be non-empty".into(),
            ));
        }
        if image.data.len() > self.config.max_image_bytes {
            return Err(VisualQueryError::ModelFailed(format!(
                "image has {} bytes, limit is {}",
                image.data.len(),
                self.config.max_image_bytes
            )));
        }
        let estimated_request_bytes = image
            .data
            .len()
            .checked_mul(4)
            .and_then(|size| {
                prompt
                    .len()
                    .checked_mul(6)
                    .and_then(|prompt| size.checked_add(prompt))
            })
            .and_then(|size| size.checked_add(REQUEST_FIXED_OVERHEAD))
            .ok_or_else(|| VisualQueryError::ModelFailed("request size overflow".into()))?;
        if estimated_request_bytes > self.config.max_request_bytes {
            return Err(VisualQueryError::ModelFailed(format!(
                "request may require {estimated_request_bytes} bytes, limit is {}",
                self.config.max_request_bytes
            )));
        }
        let request = serde_json::to_vec(&QueryRequest {
            width: image.width,
            height: image.height,
            prompt,
            image: image.data,
        })
        .map_err(|error| VisualQueryError::ModelFailed(error.to_string()))?;
        if request.len() > self.config.max_request_bytes {
            return Err(VisualQueryError::ModelFailed(format!(
                "request has {} bytes, limit is {}",
                request.len(),
                self.config.max_request_bytes
            )));
        }

        let mut command = Command::new(&self.config.program);
        command.args(&self.config.args);
        let output = run_bounded_process(
            &mut command,
            request,
            self.config.timeout,
            self.config.max_response_bytes,
            STDERR_LIMIT,
            "model process",
        )?;
        if !output.status.success() {
            return Err(VisualQueryError::ModelFailed(format!(
                "model process exited with {}: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        String::from_utf8(output.stdout)
            .map_err(|error| VisualQueryError::ModelFailed(format!("model stdout: {error}")))
    }
}

pub(crate) struct BoundedProcessOutput {
    pub(crate) status: ExitStatus,
    pub(crate) stdout: Vec<u8>,
    pub(crate) stderr: Vec<u8>,
}

pub(crate) fn run_bounded_process(
    command: &mut Command,
    input: Vec<u8>,
    timeout_duration: Duration,
    stdout_limit: usize,
    stderr_limit: usize,
    label: &str,
) -> Result<BoundedProcessOutput, VisualQueryError> {
    if timeout_duration.is_zero() {
        return Err(VisualQueryError::ModelFailed(format!(
            "{label} timeout must be positive"
        )));
    }
    if stdout_limit == 0 || stderr_limit == 0 {
        return Err(VisualQueryError::ModelFailed(format!(
            "{label} output limits must be positive"
        )));
    }
    if stdout_limit == usize::MAX || stderr_limit == usize::MAX {
        return Err(VisualQueryError::ModelFailed(format!(
            "{label} output limits are too large"
        )));
    }
    let deadline = Instant::now()
        .checked_add(timeout_duration)
        .ok_or_else(|| VisualQueryError::ModelFailed(format!("{label} timeout is too large")))?;
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    let mut child = command
        .spawn()
        .map_err(|error| VisualQueryError::ModelFailed(format!("{label} spawn failed: {error}")))?;
    let mut stdin = child.stdin.take().expect("piped child stdin");
    let stdout = child.stdout.take().expect("piped child stdout");
    let stderr = child.stderr.take().expect("piped child stderr");

    let (writer_tx, writer_rx) = mpsc::channel();
    let _ = thread::spawn(move || {
        let result = stdin.write_all(&input);
        drop(stdin);
        let _ = writer_tx.send(result);
    });
    let stdout_rx = read_bounded(stdout, stdout_limit);
    let stderr_rx = read_bounded(stderr, stderr_limit);

    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) if Instant::now() >= deadline => {
                return timeout(&mut child, timeout_duration, label);
            }
            Ok(None) => thread::sleep(POLL_INTERVAL),
            Err(error) => {
                return fail_with_cleanup(
                    &mut child,
                    format!("{label} status check failed: {error}"),
                );
            }
        }
    };
    let writer = receive_before_deadline(&writer_rx, deadline, "stdin writer panicked");
    let stdout = receive_before_deadline(&stdout_rx, deadline, "stdout reader panicked");
    let stderr = receive_before_deadline(&stderr_rx, deadline, "stderr reader panicked");
    let (writer, stdout, stderr) = match (writer, stdout, stderr) {
        (Ok(writer), Ok(stdout), Ok(stderr)) => (writer, stdout, stderr),
        (Err(CompletionError::Failed(message)), _, _)
        | (_, Err(CompletionError::Failed(message)), _)
        | (_, _, Err(CompletionError::Failed(message))) => {
            return fail_with_cleanup(&mut child, format!("{label} {message}"));
        }
        _ => return timeout(&mut child, timeout_duration, label),
    };
    writer.map_err(|error| VisualQueryError::ModelFailed(format!("{label} stdin: {error}")))?;
    let stdout = stdout
        .map_err(|error| VisualQueryError::ModelFailed(format!("{label} stdout: {error}")))?;
    let stderr = stderr
        .map_err(|error| VisualQueryError::ModelFailed(format!("{label} stderr: {error}")))?;
    if stdout.len() > stdout_limit {
        return Err(VisualQueryError::ModelFailed(format!(
            "{label} stdout exceeded {stdout_limit} bytes"
        )));
    }
    if stderr.len() > stderr_limit {
        return Err(VisualQueryError::ModelFailed(format!(
            "{label} stderr exceeded {stderr_limit} bytes"
        )));
    }
    Ok(BoundedProcessOutput {
        status,
        stdout,
        stderr,
    })
}

fn read_bounded(
    reader: impl Read + Send + 'static,
    limit: usize,
) -> Receiver<std::io::Result<Vec<u8>>> {
    let (sender, receiver) = mpsc::channel();
    let _ = thread::spawn(move || {
        let mut bytes = Vec::new();
        let result = reader
            .take(limit.saturating_add(1) as u64)
            .read_to_end(&mut bytes)
            .map(|_| bytes);
        let _ = sender.send(result);
    });
    receiver
}

enum CompletionError {
    TimedOut,
    Failed(&'static str),
}

fn receive_before_deadline<T>(
    receiver: &Receiver<T>,
    deadline: Instant,
    disconnected_message: &'static str,
) -> Result<T, CompletionError> {
    let remaining = deadline.saturating_duration_since(Instant::now());
    match receiver.recv_timeout(remaining) {
        Ok(value) => Ok(value),
        Err(RecvTimeoutError::Timeout) => Err(CompletionError::TimedOut),
        Err(RecvTimeoutError::Disconnected) => Err(CompletionError::Failed(disconnected_message)),
    }
}

fn timeout<T>(child: &mut Child, duration: Duration, label: &str) -> Result<T, VisualQueryError> {
    fail_with_cleanup(child, format!("{label} timed out after {duration:?}"))
}

fn fail_with_cleanup<T>(child: &mut Child, primary: String) -> Result<T, VisualQueryError> {
    let message = match terminate_process_tree(child) {
        Ok(()) => primary,
        Err(error) => format!("{primary}; cleanup failed: {error}"),
    };
    Err(VisualQueryError::ModelFailed(message))
}

#[cfg(unix)]
#[allow(unsafe_code)]
fn terminate_process_tree(child: &mut Child) -> std::io::Result<()> {
    use std::io::{Error, ErrorKind};
    use std::os::raw::c_int;

    const SIGKILL: c_int = 9;
    unsafe extern "C" {
        fn kill(pid: c_int, signal: c_int) -> c_int;
    }

    let process_group = c_int::try_from(child.id())
        .map_err(|_| Error::new(ErrorKind::InvalidInput, "child PID exceeds c_int"))?;
    // SAFETY: `kill` receives scalar values only. The child was placed in a process group whose
    // ID equals its PID, and negating it targets exactly that owned group.
    let signal_result = unsafe { kill(-process_group, SIGKILL) };
    let signal_error = if signal_result == 0 {
        None
    } else {
        let error = Error::last_os_error();
        (error.kind() != ErrorKind::NotFound).then_some(error)
    };
    let wait_result = child.wait().map(|_| ());
    match (signal_error, wait_result) {
        (None, result) => result,
        (Some(signal_error), Ok(())) => Err(signal_error),
        (Some(signal_error), Err(wait_error)) => Err(Error::other(format!(
            "process-group termination failed: {signal_error}; reap failed: {wait_error}"
        ))),
    }
}

#[cfg(not(unix))]
fn terminate_process_tree(child: &mut Child) -> std::io::Result<()> {
    let kill_result = child.kill();
    let wait_result = child.wait().map(|_| ());
    match (kill_result, wait_result) {
        (Ok(()), result) => result,
        (Err(kill_error), Ok(())) if kill_error.kind() == std::io::ErrorKind::InvalidInput => {
            Ok(())
        }
        (Err(kill_error), Ok(())) => Err(kill_error),
        (Err(kill_error), Err(wait_error)) => Err(std::io::Error::other(format!(
            "termination failed: {kill_error}; reap failed: {wait_error}"
        ))),
    }
}
