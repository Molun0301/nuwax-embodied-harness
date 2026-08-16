//! 公共视觉查询 JSONL（逐行 JSON）生产入口与 worker 生命周期 owner。

use crate::perception::PerceptionWorkerConfig;
use crate::subprocess::SubprocessVlBBoxModel;
use crate::{build_prompt, extract_bbox_from_response, BoundingBox2D, ImageView, VisualQueryError};
use serde::{Deserialize, Serialize};
use std::io::{self, BufRead, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::{self, JoinHandle};
use std::time::Duration;

const COMMAND_QUEUE_CAPACITY: usize = 8;
const COMMAND_POLL_INTERVAL: Duration = Duration::from_millis(5);

/// 公共入口配置。
#[derive(Debug, Clone)]
pub struct VisualQueryServiceConfig {
    /// 真实感知 worker 配置。
    pub worker: PerceptionWorkerConfig,
    /// 单条 JSONL 命令的硬字节上限。
    pub max_command_bytes: usize,
}

impl VisualQueryServiceConfig {
    /// 由 worker 请求上限构造默认服务配置。
    #[must_use]
    pub fn new(worker: PerceptionWorkerConfig) -> Self {
        let max_command_bytes = worker.max_request_bytes;
        Self {
            worker,
            max_command_bytes,
        }
    }

    fn validate(&self) -> Result<(), VisualQueryError> {
        if self.max_command_bytes == 0 {
            return Err(VisualQueryError::ModelFailed(
                "service max_command_bytes must be positive".into(),
            ));
        }
        self.worker.preflight()
    }
}

/// 公共 JSONL 命令。
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ServiceCommand {
    Query(QueryRequest),
    Cancel { request_id: String },
    Shutdown,
}

/// 一次查询携带的完整来源身份。
#[derive(Debug, Deserialize)]
struct QueryRequest {
    request_id: String,
    session_id: String,
    source_timestamp_ns: u64,
    frame_id: String,
    width: u32,
    height: u32,
    object_description: String,
    image: Vec<u8>,
}

#[derive(Debug, Serialize)]
struct ServiceEvent<'a> {
    event: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    request_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    session_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    source_timestamp_ns: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    frame_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    bbox: Option<&'a BoundingBox2D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<&'a str>,
}

impl<'a> ServiceEvent<'a> {
    fn lifecycle(event: &'a str) -> Self {
        Self {
            event,
            request_id: None,
            session_id: None,
            source_timestamp_ns: None,
            frame_id: None,
            bbox: None,
            error: None,
        }
    }
}

enum ReaderEvent {
    Command(Result<ServiceCommand, String>),
    Eof,
}

struct QueryResult {
    request: QueryRequest,
    result: Result<Option<BoundingBox2D>, VisualQueryError>,
    cancelled: bool,
}

struct ActiveQuery {
    request_id: String,
    cancelled: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// 启动公共 stdin/stdout JSONL 服务。
///
/// 启动先执行真实 backend 预检；任一时刻只接受一个查询。`cancel`、`shutdown` 或
/// stdin EOF 都会设置取消标志，查询线程随后终止并回收其完整 worker 进程组。
pub fn run_stdio_service(config: VisualQueryServiceConfig) -> Result<(), VisualQueryError> {
    config.validate()?;
    let preflight = config.worker.run_process_preflight()?;
    if preflight.trim().is_empty() {
        return Err(VisualQueryError::ModelFailed(
            "worker preflight returned an empty response".into(),
        ));
    }
    let model = Arc::new(config.worker.build_model()?);

    let stdout = io::stdout();
    let mut output = stdout.lock();
    write_event(&mut output, &ServiceEvent::lifecycle("ready"))?;
    output.flush().map_err(io_error)?;

    let (command_tx, command_rx) = mpsc::sync_channel(COMMAND_QUEUE_CAPACITY);
    let max_command_bytes = config.max_command_bytes;
    let reader_handle = thread::spawn(move || {
        let stdin = io::stdin();
        let mut input = stdin.lock();
        loop {
            match read_bounded_line(&mut input, max_command_bytes) {
                Ok(Some(line)) => {
                    let parsed = serde_json::from_slice::<ServiceCommand>(&line)
                        .map_err(|error| format!("invalid command: {error}"));
                    let stop = matches!(parsed, Ok(ServiceCommand::Shutdown));
                    if command_tx.send(ReaderEvent::Command(parsed)).is_err() || stop {
                        break;
                    }
                }
                Ok(None) => {
                    let _ = command_tx.send(ReaderEvent::Eof);
                    break;
                }
                Err(error) => {
                    if command_tx
                        .send(ReaderEvent::Command(Err(error.to_string())))
                        .is_err()
                    {
                        break;
                    }
                }
            }
        }
    });

    let (result_tx, result_rx) = mpsc::sync_channel(1);
    let mut active: Option<ActiveQuery> = None;
    let mut stopping = false;

    while !stopping {
        match command_rx.recv_timeout(COMMAND_POLL_INTERVAL) {
            Ok(ReaderEvent::Command(Ok(ServiceCommand::Query(request)))) => {
                if let Some(current) = active.as_ref() {
                    let event = ServiceEvent {
                        event: "busy",
                        request_id: Some(&request.request_id),
                        session_id: Some(&request.session_id),
                        source_timestamp_ns: Some(request.source_timestamp_ns),
                        frame_id: Some(&request.frame_id),
                        bbox: None,
                        error: Some(&current.request_id),
                    };
                    write_event(&mut output, &event)?;
                    output.flush().map_err(io_error)?;
                    continue;
                }
                if let Err(message) = validate_query(&request) {
                    write_query_event(&mut output, "error", &request, None, Some(&message))?;
                    output.flush().map_err(io_error)?;
                    continue;
                }
                let request_id = request.request_id.clone();
                let cancelled = Arc::new(AtomicBool::new(false));
                let query_cancelled = Arc::clone(&cancelled);
                let query_model = Arc::clone(&model);
                let query_tx = result_tx.clone();
                let handle = thread::spawn(move || {
                    let result = run_query(query_model.as_ref(), &request, &query_cancelled);
                    let was_cancelled = query_cancelled.load(Ordering::Acquire);
                    let _ = query_tx.send(QueryResult {
                        request,
                        result,
                        cancelled: was_cancelled,
                    });
                });
                active = Some(ActiveQuery {
                    request_id,
                    cancelled,
                    handle,
                });
            }
            Ok(ReaderEvent::Command(Ok(ServiceCommand::Cancel { request_id }))) => {
                match active.as_ref() {
                    Some(current) if current.request_id == request_id => {
                        current.cancelled.store(true, Ordering::Release);
                    }
                    _ => {
                        let event = ServiceEvent {
                            event: "error",
                            request_id: Some(&request_id),
                            session_id: None,
                            source_timestamp_ns: None,
                            frame_id: None,
                            bbox: None,
                            error: Some("no matching active query"),
                        };
                        write_event(&mut output, &event)?;
                        output.flush().map_err(io_error)?;
                    }
                }
            }
            Ok(ReaderEvent::Command(Ok(ServiceCommand::Shutdown))) | Ok(ReaderEvent::Eof) => {
                stopping = true;
                if let Some(current) = active.as_ref() {
                    current.cancelled.store(true, Ordering::Release);
                }
            }
            Ok(ReaderEvent::Command(Err(message))) => {
                let event = ServiceEvent {
                    event: "error",
                    request_id: None,
                    session_id: None,
                    source_timestamp_ns: None,
                    frame_id: None,
                    bbox: None,
                    error: Some(&message),
                };
                write_event(&mut output, &event)?;
                output.flush().map_err(io_error)?;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => stopping = true,
        }

        if !stopping {
            drain_query_result(&result_rx, &mut active, &mut output)?;
        }
    }

    if let Some(current) = active.take() {
        current.cancelled.store(true, Ordering::Release);
        current
            .handle
            .join()
            .map_err(|_| VisualQueryError::ModelFailed("query thread panicked".into()))?;
        let _ = result_rx.try_recv();
    }
    reader_handle
        .join()
        .map_err(|_| VisualQueryError::ModelFailed("command reader thread panicked".into()))?;
    write_event(&mut output, &ServiceEvent::lifecycle("stopped"))?;
    output.flush().map_err(io_error)
}

fn validate_query(request: &QueryRequest) -> Result<(), String> {
    if request.request_id.trim().is_empty() {
        return Err("request_id must be non-empty".into());
    }
    if request.session_id.trim().is_empty() {
        return Err("session_id must be non-empty".into());
    }
    if request.frame_id.trim().is_empty() {
        return Err("frame_id must be non-empty".into());
    }
    if request.object_description.trim().is_empty() {
        return Err("object_description must be non-empty".into());
    }
    Ok(())
}

fn run_query(
    model: &SubprocessVlBBoxModel,
    request: &QueryRequest,
    cancelled: &AtomicBool,
) -> Result<Option<BoundingBox2D>, VisualQueryError> {
    let prompt = build_prompt(&request.object_description);
    let response = model.query_with_cancel(
        ImageView {
            width: request.width,
            height: request.height,
            data: &request.image,
        },
        &prompt,
        cancelled,
    )?;
    if response.trim().is_empty() {
        return Ok(None);
    }
    Ok(extract_bbox_from_response(&response))
}

fn drain_query_result(
    result_rx: &mpsc::Receiver<QueryResult>,
    active: &mut Option<ActiveQuery>,
    output: &mut impl Write,
) -> Result<(), VisualQueryError> {
    let result = match result_rx.try_recv() {
        Ok(result) => result,
        Err(mpsc::TryRecvError::Empty) => return Ok(()),
        Err(mpsc::TryRecvError::Disconnected) => {
            return Err(VisualQueryError::ModelFailed(
                "query result channel disconnected".into(),
            ));
        }
    };
    let current = active
        .take()
        .ok_or_else(|| VisualQueryError::ModelFailed("orphan query result".into()))?;
    current
        .handle
        .join()
        .map_err(|_| VisualQueryError::ModelFailed("query thread panicked".into()))?;

    if result.cancelled {
        write_query_event(output, "cancelled", &result.request, None, None)?;
    } else {
        match result.result {
            Ok(Some(ref bbox)) => {
                write_query_event(output, "tracked", &result.request, Some(bbox), None)?;
            }
            Ok(None) => write_query_event(output, "lost", &result.request, None, None)?,
            Err(ref error) => {
                let message = error.to_string();
                write_query_event(output, "error", &result.request, None, Some(&message))?;
            }
        }
    }
    output.flush().map_err(io_error)
}

fn write_query_event(
    output: &mut impl Write,
    event: &str,
    request: &QueryRequest,
    bbox: Option<&BoundingBox2D>,
    error: Option<&str>,
) -> Result<(), VisualQueryError> {
    write_event(
        output,
        &ServiceEvent {
            event,
            request_id: Some(&request.request_id),
            session_id: Some(&request.session_id),
            source_timestamp_ns: Some(request.source_timestamp_ns),
            frame_id: Some(&request.frame_id),
            bbox,
            error,
        },
    )
}

fn write_event(output: &mut impl Write, event: &ServiceEvent<'_>) -> Result<(), VisualQueryError> {
    serde_json::to_writer(&mut *output, event)
        .map_err(|error| VisualQueryError::ModelFailed(format!("service stdout: {error}")))?;
    output.write_all(b"\n").map_err(io_error)
}

fn read_bounded_line(input: &mut impl BufRead, limit: usize) -> io::Result<Option<Vec<u8>>> {
    let mut line = Vec::new();
    let mut overflow = false;
    loop {
        let available = input.fill_buf()?;
        if available.is_empty() {
            if overflow {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("command exceeds {limit} bytes"),
                ));
            }
            return if line.is_empty() {
                Ok(None)
            } else {
                Ok(Some(line))
            };
        }
        let newline = available.iter().position(|byte| *byte == b'\n');
        let consumed = newline.map_or(available.len(), |position| position + 1);
        let payload_len = newline.unwrap_or(consumed);
        if !overflow {
            if line.len().saturating_add(payload_len) > limit {
                overflow = true;
                line.clear();
            } else {
                line.extend_from_slice(&available[..payload_len]);
            }
        }
        input.consume(consumed);
        if newline.is_some() {
            return if overflow {
                Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("command exceeds {limit} bytes"),
                ))
            } else {
                Ok(Some(line))
            };
        }
    }
}

fn io_error(error: io::Error) -> VisualQueryError {
    VisualQueryError::ModelFailed(format!("service I/O: {error}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn bounded_reader_recovers_after_oversized_command() {
        let mut input = Cursor::new(b"12345\n{}\n".to_vec());
        assert!(read_bounded_line(&mut input, 4).is_err());
        assert_eq!(
            read_bounded_line(&mut input, 4).unwrap(),
            Some(b"{}".to_vec())
        );
    }

    #[test]
    fn query_identity_is_required_before_worker_spawn() {
        let request = QueryRequest {
            request_id: "request-1".into(),
            session_id: "session-1".into(),
            source_timestamp_ns: 42,
            frame_id: "camera_optical_frame".into(),
            width: 1,
            height: 1,
            object_description: "chair".into(),
            image: vec![0],
        };
        assert!(validate_query(&request).is_ok());
        let mut missing_frame = request;
        missing_frame.frame_id.clear();
        assert_eq!(
            validate_query(&missing_frame).unwrap_err(),
            "frame_id must be non-empty"
        );
    }
}
