//! 公共视觉查询 JSONL（逐行 JSON）生产入口与 worker 生命周期 owner。

use crate::goal_transport::{
    AdmittedGoalLease, GeometryGoalTransport, GoalTransportConfig, NavigationAdmission,
    NavigationOutcome,
};
use crate::perception::PerceptionWorkerConfig;
use crate::session::{
    TargetObservation, TargetObservationState, VisualQueryFrameIdentity, VisualQuerySession,
    VisualQuerySessionConfig,
};
use crate::subprocess::SubprocessVlBBoxModel;
use crate::{build_prompt, extract_bbox_from_response, BoundingBox2D, ImageView, VisualQueryError};
use nuwax_nav_bbox_tracker::{
    BBoxTracker, BBoxTrackerConfig, BBoxTrackerStatus, CameraInfo, Detection2D, Detection2DArray,
};
use nuwax_nav_types_messages::{NavFrameId, Timestamp};
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
    /// 查询会话的连续丢失上限。
    pub session: VisualQuerySessionConfig,
    /// BBox 到相机姿态的投影配置。
    pub tracker: BBoxTrackerConfig,
    /// Batch02 唯一 GoalIntent 准入 transport。
    pub goal_transport: GoalTransportConfig,
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
            session: VisualQuerySessionConfig::default(),
            tracker: BBoxTrackerConfig::default(),
            goal_transport: GoalTransportConfig::default(),
            max_command_bytes,
        }
    }

    fn validate(&self) -> Result<(), VisualQueryError> {
        if self.max_command_bytes == 0 {
            return Err(service_error("service max_command_bytes must be positive"));
        }
        if self.session.max_lost_frames == 0 {
            return Err(service_error("max_lost_frames must be positive"));
        }
        BBoxTracker::new(self.tracker.clone())
            .map_err(|error| service_error(format!("bbox tracker: {error}")))?;
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
    #[serde(default)]
    target_id: Option<String>,
    source_timestamp_ns: u64,
    frame_id: String,
    #[serde(default)]
    calibration_id: Option<String>,
    #[serde(default)]
    camera_info: Option<CameraInfo>,
    width: u32,
    height: u32,
    object_description: String,
    image: Vec<u8>,
    #[serde(default)]
    navigation: Option<NavigationAdmission>,
}

impl QueryRequest {
    fn identity(&self) -> Result<Option<VisualQueryFrameIdentity>, String> {
        match (&self.target_id, &self.calibration_id, self.camera_info) {
            (None, None, None) => Ok(None),
            (Some(target_id), Some(calibration_id), Some(camera_info)) => {
                Ok(Some(VisualQueryFrameIdentity {
                    request_id: self.request_id.clone(),
                    session_id: self.session_id.clone(),
                    target_id: target_id.clone(),
                    object_description: self.object_description.clone(),
                    source_timestamp_ns: self.source_timestamp_ns,
                    camera_frame: NavFrameId::from_static(&self.frame_id),
                    calibration_id: calibration_id.clone(),
                    camera_info,
                    image_width: self.width,
                    image_height: self.height,
                }))
            }
            _ => Err("target_id, calibration_id, and camera_info must be supplied together".into()),
        }
    }
}

#[derive(Debug, Serialize)]
struct ServiceEvent<'a> {
    event: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    request_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    session_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    target_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    source_timestamp_ns: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    frame_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    calibration_id: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    bbox: Option<&'a BoundingBox2D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    observation: Option<&'a TargetObservation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    navigation: Option<&'a NavigationOutcome>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<&'a str>,
}

impl<'a> ServiceEvent<'a> {
    fn lifecycle(event: &'a str) -> Self {
        Self {
            event,
            request_id: None,
            session_id: None,
            target_id: None,
            source_timestamp_ns: None,
            frame_id: None,
            calibration_id: None,
            bbox: None,
            observation: None,
            navigation: None,
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
    handle: Option<JoinHandle<()>>,
}

impl ActiveQuery {
    fn cancel_and_join(&mut self) -> Result<(), VisualQueryError> {
        self.cancelled.store(true, Ordering::Release);
        if let Some(handle) = self.handle.take() {
            handle
                .join()
                .map_err(|_| service_error("query thread panicked"))?;
        }
        Ok(())
    }

    fn join(&mut self) -> Result<(), VisualQueryError> {
        if let Some(handle) = self.handle.take() {
            handle
                .join()
                .map_err(|_| service_error("query thread panicked"))?;
        }
        Ok(())
    }
}

impl Drop for ActiveQuery {
    fn drop(&mut self) {
        let _ = self.cancel_and_join();
    }
}

/// 启动公共 stdin/stdout JSONL 服务。
///
/// 启动先执行真实 backend 预检；任一时刻只接受一个查询。`cancel`、`shutdown` 或
/// stdin EOF 都会设置取消标志，查询线程随后终止并回收其完整 worker 进程组。
pub fn run_stdio_service(config: VisualQueryServiceConfig) -> Result<(), VisualQueryError> {
    config.validate()?;
    let preflight = config.worker.run_process_preflight()?;
    if preflight.trim().is_empty() {
        return Err(service_error("worker preflight returned an empty response"));
    }
    let model = Arc::new(config.worker.build_model()?);
    let session_model = Box::new(config.worker.build_model()?);
    let mut session = VisualQuerySession::new(session_model, config.session);
    let mut tracker = BBoxTracker::new(config.tracker.clone())
        .map_err(|error| service_error(format!("bbox tracker: {error}")))?;
    let goal_transport = GeometryGoalTransport::new(config.goal_transport.clone())
        .map_err(|error| service_error(format!("goal transport: {error}")))?;
    let tracker_swaps_axes = config.tracker.swap_axes;
    let mut active_goal: Option<AdmittedGoalLease> = None;

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
                    write_request_event(
                        &mut output,
                        "busy",
                        &request,
                        None,
                        Some(&current.request_id),
                    )?;
                    output.flush().map_err(io_error)?;
                    continue;
                }
                if let Err(message) = validate_query(&request) {
                    write_request_event(&mut output, "error", &request, None, Some(&message))?;
                    output.flush().map_err(io_error)?;
                    continue;
                }
                let identity = request.identity().map_err(service_error)?;
                if let Some(identity) = identity {
                    match session.begin_request(identity) {
                        Ok(Some(stopped)) => {
                            write_observation_event(&mut output, &stopped, None)?;
                            output.flush().map_err(io_error)?;
                        }
                        Ok(None) => {}
                        Err(error) => {
                            let message = error.to_string();
                            write_request_event(
                                &mut output,
                                "error",
                                &request,
                                None,
                                Some(&message),
                            )?;
                            output.flush().map_err(io_error)?;
                            continue;
                        }
                    }
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
                    handle: Some(handle),
                });
            }
            Ok(ReaderEvent::Command(Ok(ServiceCommand::Cancel { request_id }))) => {
                match active.as_ref() {
                    Some(current) if current.request_id == request_id => {
                        current.cancelled.store(true, Ordering::Release);
                    }
                    _ if active_goal
                        .as_ref()
                        .is_some_and(|lease| lease.goal_id == request_id) =>
                    {
                        let lease = active_goal.take().expect("matched active goal lease");
                        match goal_transport.cancel(&lease) {
                            Ok(terminal) => {
                                let stopped = session.stop_matching(&lease.parent_request_id);
                                if let Some(observation) = stopped.as_ref() {
                                    write_observation_event(&mut output, observation, None)?;
                                    write_navigation_event(
                                        &mut output,
                                        "navigation_cancelled",
                                        observation,
                                        &terminal,
                                        None,
                                    )?;
                                } else {
                                    write_navigation_terminal(
                                        &mut output,
                                        "navigation_cancelled",
                                        &terminal,
                                        None,
                                    )?;
                                }
                            }
                            Err(error) => {
                                active_goal = Some(lease.clone());
                                write_navigation_terminal(
                                    &mut output,
                                    "navigation_cancel_failed",
                                    &NavigationOutcome {
                                        status: "FAILED".into(),
                                        reason_code: "TRANSPORT_ERROR".into(),
                                        message: error,
                                        goal_id: lease.goal_id.clone(),
                                        parent_request_id: lease.parent_request_id.clone(),
                                        goal_generation: lease.goal_generation,
                                        producer_runtime_generation: lease
                                            .producer_runtime_generation,
                                        cancel_phase: String::new(),
                                        actuator_zeroed: false,
                                    },
                                    Some("navigation cancel transport failed"),
                                )?;
                            }
                        }
                        output.flush().map_err(io_error)?;
                    }
                    _ => {
                        let event = ServiceEvent {
                            event: "error",
                            request_id: Some(&request_id),
                            session_id: None,
                            target_id: None,
                            source_timestamp_ns: None,
                            frame_id: None,
                            calibration_id: None,
                            bbox: None,
                            observation: None,
                            navigation: None,
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
                    target_id: None,
                    source_timestamp_ns: None,
                    frame_id: None,
                    calibration_id: None,
                    bbox: None,
                    observation: None,
                    navigation: None,
                    error: Some(&message),
                };
                write_event(&mut output, &event)?;
                output.flush().map_err(io_error)?;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => stopping = true,
        }

        if !stopping {
            drain_query_result(
                &result_rx,
                &mut active,
                &mut session,
                &mut tracker,
                &goal_transport,
                &mut active_goal,
                tracker_swaps_axes,
                &mut output,
            )?;
        }
    }

    if let Some(mut current) = active.take() {
        current.cancel_and_join()?;
        let _ = result_rx.try_recv();
    }
    if let Some(lease) = active_goal.take() {
        match goal_transport.cancel(&lease) {
            Ok(terminal) => {
                write_navigation_terminal(&mut output, "navigation_cancelled", &terminal, None)?
            }
            Err(error) => {
                return Err(service_error(format!(
                    "failed to cancel active navigation during shutdown: {error}"
                )));
            }
        }
    }
    if let Some(stopped) = session.stop_active() {
        write_observation_event(&mut output, &stopped, None)?;
    }
    reader_handle
        .join()
        .map_err(|_| service_error("command reader thread panicked"))?;
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
    if let Some(target_id) = request.target_id.as_deref() {
        if target_id.trim().is_empty() {
            return Err("target_id must be non-empty".into());
        }
    }
    if let Some(calibration_id) = request.calibration_id.as_deref() {
        if calibration_id.trim().is_empty() {
            return Err("calibration_id must be non-empty".into());
        }
    }
    if request.object_description.trim().is_empty() {
        return Err("object_description must be non-empty".into());
    }
    if request.source_timestamp_ns == 0 || request.source_timestamp_ns > i64::MAX as u64 {
        return Err("source_timestamp_ns must be within the positive sensor clock range".into());
    }
    if let Some(camera_info) = request.camera_info {
        if !camera_info.is_valid()
            || camera_info.width != request.width
            || camera_info.height != request.height
        {
            return Err("camera_info must be valid and match image dimensions".into());
        }
    }
    let identity = request.identity()?;
    if request.navigation.is_some() && identity.is_none() {
        return Err(
            "navigation requires target_id, calibration_id, and camera_info strict identity".into(),
        );
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
    session: &mut VisualQuerySession,
    tracker: &mut BBoxTracker,
    goal_transport: &GeometryGoalTransport,
    active_goal: &mut Option<AdmittedGoalLease>,
    tracker_swaps_axes: bool,
    output: &mut impl Write,
) -> Result<(), VisualQueryError> {
    let result = match result_rx.try_recv() {
        Ok(result) => result,
        Err(mpsc::TryRecvError::Empty) => return Ok(()),
        Err(mpsc::TryRecvError::Disconnected) => {
            return Err(service_error("query result channel disconnected"));
        }
    };
    let mut current = active
        .take()
        .ok_or_else(|| service_error("orphan query result"))?;
    let cancelled = result.cancelled || current.cancelled.load(Ordering::Acquire);
    current.join()?;

    let identity = result.request.identity().map_err(service_error)?;
    match identity {
        Some(identity) if cancelled => {
            let stopped = session
                .stop_matching(&identity.session_id)
                .ok_or_else(|| service_error("cancelled query lost its session"))?;
            write_observation_event(output, &stopped, None)?;
        }
        None if cancelled => {
            write_legacy_event(output, "cancelled", &result.request, None, None)?;
        }
        Some(identity) => match result.result {
            Ok(Some(bbox)) => match project_bbox(tracker, &identity, &bbox)
                .and_then(|pose| session.complete_request(&identity, Some(bbox), Some(pose)))
            {
                Ok(observation) => {
                    write_observation_event(output, &observation, None)?;
                    if let Some(admission) = result.request.navigation.as_ref() {
                        if let Some(previous) = active_goal.take() {
                            match goal_transport.cancel(&previous) {
                                Ok(terminal) => write_navigation_event(
                                    output,
                                    "navigation_replaced",
                                    &observation,
                                    &terminal,
                                    None,
                                )?,
                                Err(error) => {
                                    *active_goal = Some(previous);
                                    write_navigation_error(output, &observation, &error)?;
                                    output.flush().map_err(io_error)?;
                                    return Ok(());
                                }
                            }
                        }
                        match goal_transport.submit(&observation, admission, tracker_swaps_axes) {
                            Ok((lease, admitted)) => {
                                *active_goal = Some(lease);
                                write_navigation_event(
                                    output,
                                    "goal_admitted",
                                    &observation,
                                    &admitted,
                                    None,
                                )?;
                            }
                            Err(error) => write_navigation_error(output, &observation, &error)?,
                        }
                    }
                }
                Err(error) => write_failed_result(output, session, &result.request, error)?,
            },
            Ok(None) => match session.complete_request(&identity, None, None) {
                Ok(observation) => write_observation_event(output, &observation, None)?,
                Err(error) => write_failed_result(output, session, &result.request, error)?,
            },
            Err(error) => write_failed_result(output, session, &result.request, error)?,
        },
        None => match result.result {
            Ok(Some(ref bbox)) => {
                write_legacy_event(output, "tracked", &result.request, Some(bbox), None)?;
            }
            Ok(None) => write_legacy_event(output, "lost", &result.request, None, None)?,
            Err(ref error) => {
                let message = error.to_string();
                write_legacy_event(output, "error", &result.request, None, Some(&message))?;
            }
        },
    }
    output.flush().map_err(io_error)
}

fn write_failed_result(
    output: &mut impl Write,
    session: &mut VisualQuerySession,
    request: &QueryRequest,
    error: VisualQueryError,
) -> Result<(), VisualQueryError> {
    let stopped = session.stop_matching(&request.session_id);
    let message = error.to_string();
    write_request_event(output, "error", request, stopped.as_ref(), Some(&message))
}

fn project_bbox(
    tracker: &mut BBoxTracker,
    identity: &VisualQueryFrameIdentity,
    bbox: &BoundingBox2D,
) -> Result<nuwax_nav_bbox_tracker::PoseStamped, VisualQueryError> {
    tracker.update_camera_info(identity.camera_info);
    let (center_x, center_y) = bbox.center();
    let detections = Detection2DArray {
        frame_id: identity.camera_frame.clone(),
        stamp: Timestamp::from_sensor(identity.source_timestamp_ns as i64),
        detections: vec![Detection2D {
            bbox_center: (f64::from(center_x), f64::from(center_y)),
            bbox_size: (f64::from(bbox.width()), f64::from(bbox.height())),
            score: bbox.score,
            class_name: bbox.class_name.clone(),
        }],
    };
    match tracker.process(&detections) {
        BBoxTrackerStatus::Published(mut pose) => {
            pose.stamp = detections.stamp;
            Ok(pose)
        }
        BBoxTrackerStatus::NoDetection => Err(service_error(
            "bbox tracker rejected a tracked worker result",
        )),
        BBoxTrackerStatus::WaitingForIntrinsics => {
            Err(service_error("bbox tracker lost calibration snapshot"))
        }
    }
}

fn write_observation_event(
    output: &mut impl Write,
    observation: &TargetObservation,
    error: Option<&str>,
) -> Result<(), VisualQueryError> {
    let event = match observation.state {
        TargetObservationState::Tracked => "tracked",
        TargetObservationState::Lost { .. } => "lost",
        TargetObservationState::Stopped => "stopped",
    };
    write_event(
        output,
        &ServiceEvent {
            event,
            request_id: Some(&observation.request_id),
            session_id: Some(&observation.session_id),
            target_id: Some(&observation.target_id),
            source_timestamp_ns: Some(observation.source_timestamp_ns),
            frame_id: Some(observation.camera_frame.as_str()),
            calibration_id: Some(&observation.calibration_id),
            bbox: observation.bbox.as_ref(),
            observation: Some(observation),
            navigation: None,
            error,
        },
    )
}

fn write_navigation_event(
    output: &mut impl Write,
    event: &str,
    observation: &TargetObservation,
    navigation: &NavigationOutcome,
    error: Option<&str>,
) -> Result<(), VisualQueryError> {
    write_event(
        output,
        &ServiceEvent {
            event,
            request_id: Some(&observation.request_id),
            session_id: Some(&observation.session_id),
            target_id: Some(&observation.target_id),
            source_timestamp_ns: Some(observation.source_timestamp_ns),
            frame_id: Some(observation.camera_frame.as_str()),
            calibration_id: Some(&observation.calibration_id),
            bbox: observation.bbox.as_ref(),
            observation: Some(observation),
            navigation: Some(navigation),
            error,
        },
    )
}

fn write_navigation_error(
    output: &mut impl Write,
    observation: &TargetObservation,
    error: &str,
) -> Result<(), VisualQueryError> {
    write_event(
        output,
        &ServiceEvent {
            event: "goal_rejected",
            request_id: Some(&observation.request_id),
            session_id: Some(&observation.session_id),
            target_id: Some(&observation.target_id),
            source_timestamp_ns: Some(observation.source_timestamp_ns),
            frame_id: Some(observation.camera_frame.as_str()),
            calibration_id: Some(&observation.calibration_id),
            bbox: observation.bbox.as_ref(),
            observation: Some(observation),
            navigation: None,
            error: Some(error),
        },
    )
}

fn write_navigation_terminal(
    output: &mut impl Write,
    event: &str,
    navigation: &NavigationOutcome,
    error: Option<&str>,
) -> Result<(), VisualQueryError> {
    write_event(
        output,
        &ServiceEvent {
            event,
            request_id: Some(&navigation.goal_id),
            session_id: Some(&navigation.parent_request_id),
            target_id: None,
            source_timestamp_ns: None,
            frame_id: None,
            calibration_id: None,
            bbox: None,
            observation: None,
            navigation: Some(navigation),
            error,
        },
    )
}

fn write_request_event(
    output: &mut impl Write,
    event: &str,
    request: &QueryRequest,
    observation: Option<&TargetObservation>,
    error: Option<&str>,
) -> Result<(), VisualQueryError> {
    write_event(
        output,
        &ServiceEvent {
            event,
            request_id: Some(&request.request_id),
            session_id: Some(&request.session_id),
            target_id: request.target_id.as_deref(),
            source_timestamp_ns: Some(request.source_timestamp_ns),
            frame_id: Some(&request.frame_id),
            calibration_id: request.calibration_id.as_deref(),
            bbox: observation.and_then(|value| value.bbox.as_ref()),
            observation,
            navigation: None,
            error,
        },
    )
}

fn write_legacy_event(
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
            target_id: None,
            source_timestamp_ns: Some(request.source_timestamp_ns),
            frame_id: Some(&request.frame_id),
            calibration_id: None,
            bbox,
            observation: None,
            navigation: None,
            error,
        },
    )
}

fn write_event(output: &mut impl Write, event: &ServiceEvent<'_>) -> Result<(), VisualQueryError> {
    serde_json::to_writer(&mut *output, event)
        .map_err(|error| service_error(format!("service stdout: {error}")))?;
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
    service_error(format!("service I/O: {error}"))
}

fn service_error(message: impl Into<String>) -> VisualQueryError {
    VisualQueryError::ModelFailed(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn query_request() -> QueryRequest {
        QueryRequest {
            request_id: "request-1".into(),
            session_id: "session-1".into(),
            target_id: Some("target-chair".into()),
            source_timestamp_ns: 42,
            frame_id: "camera_optical_frame".into(),
            calibration_id: Some("front-camera-v3".into()),
            camera_info: Some(CameraInfo::new(1, 1, 1.0, 1.0, 0.5, 0.5)),
            width: 1,
            height: 1,
            object_description: "chair".into(),
            image: vec![0],
            navigation: None,
        }
    }

    fn strict_identity(request: &QueryRequest) -> VisualQueryFrameIdentity {
        request.identity().unwrap().unwrap()
    }

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
        let request = query_request();
        assert!(validate_query(&request).is_ok());
        let mut missing_frame = request;
        missing_frame.frame_id.clear();
        assert_eq!(
            validate_query(&missing_frame).unwrap_err(),
            "frame_id must be non-empty"
        );
    }

    #[test]
    fn v1_query_and_top_level_bbox_event_remain_compatible() {
        let command: ServiceCommand = serde_json::from_str(
            r#"{"type":"query","request_id":"request-v1","session_id":"session-v1","source_timestamp_ns":42,"frame_id":"legacy_camera","width":1,"height":1,"object_description":"chair","image":[0]}"#,
        )
        .unwrap();
        let ServiceCommand::Query(request) = command else {
            panic!("expected query command");
        };
        assert!(validate_query(&request).is_ok());
        assert!(request.identity().unwrap().is_none());

        let bbox = BoundingBox2D::from_xyxy(0.0, 0.0, 1.0, 1.0).unwrap();
        let mut output = Vec::new();
        write_legacy_event(&mut output, "tracked", &request, Some(&bbox), None).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&output).unwrap();
        assert_eq!(value["event"], "tracked");
        assert_eq!(value["bbox"]["x2"], 1.0);
        assert!(value.get("observation").is_none());
        assert!(value.get("target_id").is_none());
    }

    #[test]
    fn v1_query_cannot_enable_navigation() {
        let command: ServiceCommand = serde_json::from_str(
            r#"{"type":"query","request_id":"request-v1","session_id":"session-v1","source_timestamp_ns":42,"frame_id":"legacy_camera","width":1,"height":1,"object_description":"chair","image":[0],"navigation":{"transform":{"source_frame_id":"camera_optical_frame","target_frame_id":"map","source_timestamp_ns":42,"calibration_id":"calibration-1","translation_xyz":[0.0,0.0,0.0],"rotation_xyzw":[0.0,0.0,0.0,1.0]},"motion_guard_boot_nonce_hex":"00112233445566778899aabbccddeeff","motion_stop_generation":0}}"#,
        )
        .unwrap();
        let ServiceCommand::Query(request) = command else {
            panic!("expected query command");
        };
        assert_eq!(
            validate_query(&request).unwrap_err(),
            "navigation requires target_id, calibration_id, and camera_info strict identity"
        );
    }

    #[test]
    fn strict_query_rejects_non_camera_custom_frame() {
        let mut request = query_request();
        request.frame_id = "laser".into();
        let identity = strict_identity(&request);
        let mut session = VisualQuerySession::new(
            Box::new(configured_unused_model()),
            VisualQuerySessionConfig::default(),
        );
        assert!(session.begin_request(identity).is_err());
        assert!(!session.is_active());
    }

    #[test]
    fn bbox_tracker_pose_keeps_frame_and_source_timestamp() {
        let request = query_request();
        let identity = strict_identity(&request);
        let bbox = BoundingBox2D::from_xyxy(0.0, 0.0, 1.0, 1.0).unwrap();
        let mut tracker = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        let pose = project_bbox(&mut tracker, &identity, &bbox).unwrap();
        assert_eq!(pose.frame_id, identity.camera_frame);
        assert_eq!(
            pose.stamp,
            Timestamp::from_sensor(identity.source_timestamp_ns as i64)
        );
    }

    #[test]
    fn observation_event_serializes_strict_identity_and_bbox() {
        let request = query_request();
        let identity = strict_identity(&request);
        let bbox = extract_bbox_from_response(r#"{"name":"chair","bbox":[0,0,1,1],"score":0.75}"#)
            .unwrap();
        let mut tracker = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        let pose = project_bbox(&mut tracker, &identity, &bbox).unwrap();
        let mut session = VisualQuerySession::new(
            Box::new(configured_unused_model()),
            VisualQuerySessionConfig::default(),
        );
        session.begin_request(identity.clone()).unwrap();
        let observation = session
            .complete_request(&identity, Some(bbox), Some(pose))
            .unwrap();
        let mut output = Vec::new();
        write_observation_event(&mut output, &observation, None).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&output).unwrap();
        assert_eq!(value["event"], "tracked");
        assert_eq!(value["target_id"], "target-chair");
        assert_eq!(value["calibration_id"], "front-camera-v3");
        assert_eq!(value["observation"]["source_timestamp_ns"], 42);
        assert_eq!(value["observation"]["bbox"]["x2"], 1.0);
        assert_eq!(value["bbox"]["score"], 0.75);
        assert_eq!(value["observation"]["bbox"]["score"], 0.75);
    }

    #[test]
    fn accepted_cancel_wins_over_already_queued_worker_result() {
        let request = query_request();
        let identity = strict_identity(&request);
        let mut session = VisualQuerySession::new(
            Box::new(configured_unused_model()),
            VisualQuerySessionConfig::default(),
        );
        session.begin_request(identity).unwrap();
        let (result_tx, result_rx) = mpsc::sync_channel(1);
        result_tx
            .send(QueryResult {
                request,
                result: Ok(BoundingBox2D::from_xyxy(0.0, 0.0, 1.0, 1.0)),
                cancelled: false,
            })
            .unwrap();
        let cancelled = Arc::new(AtomicBool::new(true));
        let mut active = Some(ActiveQuery {
            request_id: "request-1".into(),
            cancelled,
            handle: Some(thread::spawn(|| {})),
        });
        let mut tracker = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        let goal_transport = GeometryGoalTransport::new(GoalTransportConfig::default()).unwrap();
        let mut active_goal = None;
        let mut output = Vec::new();
        drain_query_result(
            &result_rx,
            &mut active,
            &mut session,
            &mut tracker,
            &goal_transport,
            &mut active_goal,
            true,
            &mut output,
        )
        .unwrap();
        let value: serde_json::Value = serde_json::from_slice(&output).unwrap();
        assert_eq!(value["event"], "stopped");
        assert_eq!(value["observation"]["state"], "stopped");
        assert!(active.is_none());
        assert!(!session.is_active());
    }

    #[cfg(unix)]
    #[test]
    fn output_error_drops_active_query_and_reaps_slow_worker() {
        use crate::subprocess::SubprocessModelConfig;
        use std::ffi::OsString;
        use std::fs;
        use std::path::PathBuf;
        use std::sync::atomic::AtomicU64;
        use std::time::{Duration, Instant};

        static TEMP_ID: AtomicU64 = AtomicU64::new(0);
        struct BrokenWriter;
        impl Write for BrokenWriter {
            fn write(&mut self, _buffer: &[u8]) -> io::Result<usize> {
                Err(io::Error::new(io::ErrorKind::BrokenPipe, "closed output"))
            }

            fn flush(&mut self) -> io::Result<()> {
                Err(io::Error::new(io::ErrorKind::BrokenPipe, "closed output"))
            }
        }

        let id = TEMP_ID.fetch_add(1, Ordering::Relaxed);
        let root = std::env::temp_dir().join(format!(
            "nuwax-vq-output-cleanup-{}-{id}",
            std::process::id()
        ));
        fs::create_dir_all(&root).unwrap();
        let script = root.join("slow-worker.sh");
        let pid_file = root.join("worker.pid");
        fs::write(
            &script,
            format!(
                "#!/bin/sh\nread request\necho $$ > '{}'\nsleep 30\n",
                pid_file.display()
            ),
        )
        .unwrap();
        let model = SubprocessVlBBoxModel::new(SubprocessModelConfig {
            program: PathBuf::from("/bin/sh"),
            args: vec![OsString::from(script.as_os_str())],
            timeout: Duration::from_secs(30),
            max_response_bytes: 1024,
            max_image_bytes: 1024,
            max_image_pixels: 1024,
            max_request_bytes: 4096,
        })
        .unwrap();
        let cancelled = Arc::new(AtomicBool::new(false));
        let query_cancelled = Arc::clone(&cancelled);
        let handle = thread::spawn(move || {
            let _ = model.query_with_cancel(
                ImageView {
                    width: 1,
                    height: 1,
                    data: &[0],
                },
                "find object",
                &query_cancelled,
            );
        });
        let active = ActiveQuery {
            request_id: "request-slow".into(),
            cancelled,
            handle: Some(handle),
        };
        let deadline = Instant::now() + Duration::from_secs(2);
        while !pid_file.is_file() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(5));
        }
        let pid = fs::read_to_string(&pid_file)
            .unwrap()
            .trim()
            .parse::<u32>()
            .unwrap();

        let failure = (|| -> Result<(), VisualQueryError> {
            let _active = active;
            write_event(&mut BrokenWriter, &ServiceEvent::lifecycle("ready"))?;
            Ok(())
        })();
        assert!(failure.is_err());
        assert!(!PathBuf::from(format!("/proc/{pid}")).exists());
        fs::remove_dir_all(root).unwrap();
    }

    fn configured_unused_model() -> impl crate::VlBBoxModel {
        struct Unused;
        impl crate::VlBBoxModel for Unused {
            fn query(
                &self,
                _image: ImageView<'_>,
                _prompt: &str,
            ) -> Result<String, VisualQueryError> {
                panic!("serialization test must not invoke a model")
            }
        }
        Unused
    }
}
