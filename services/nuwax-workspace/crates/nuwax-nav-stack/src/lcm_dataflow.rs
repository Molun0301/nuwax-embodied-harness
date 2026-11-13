//! 固定导航任务的真实通信适配。

#[cfg(any(feature = "cmu-native", feature = "explore"))]
use std::collections::{HashMap, HashSet};
use std::future::Future;
use std::pin::Pin;
#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::time::Duration;
#[cfg(feature = "guard-runtime")]
use std::time::Instant;

use futures::{FutureExt, Stream, StreamExt};
use nuwax_codec::lcm::schema;
#[cfg(feature = "patrolling")]
use nuwax_nav_types_messages::GridSnapshot;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore",
    feature = "patrolling"
))]
use nuwax_nav_types_messages::Path2D;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore"
))]
use nuwax_nav_types_messages::Point2;
#[cfg(any(
    feature = "simple-2d",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
use nuwax_nav_types_messages::PointCloud;
#[cfg(feature = "simple-2d")]
use nuwax_nav_types_messages::PointStamped;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling",
    feature = "click-goal",
    feature = "explore"
))]
use nuwax_nav_types_messages::PoseStamped;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling"
))]
use nuwax_nav_types_messages::Twist2;
use nuwax_nav_types_messages::{GoalKey, NavFrameId, Point3, Pose3, Quaternion, Timestamp};
#[cfg(feature = "explore")]
use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_transport::{LcmFrameStream, LcmTransport, TransportResult};
use tokio::sync::oneshot;

#[cfg(feature = "mls")]
#[path = "surface_dataflow.rs"]
mod surface_dataflow;
#[cfg(feature = "mls")]
pub(crate) use surface_dataflow::start_surface_tasks;

use crate::error::{NavStackError, NavStackResult};
use crate::topics::TaskChannels;
use crate::task::{spawn_task, ReadySender, TaskThread};

#[cfg(any(feature = "cmu-native", feature = "explore"))]
static FAR_READY_GENERATION: AtomicI64 = AtomicI64::new(0);
#[cfg(feature = "patrolling")]
const PLANNER_CONTROL_TIMEOUT: Duration = Duration::from_secs(2);

fn lcm_transport() -> NavStackResult<LcmTransport> {
    let provider = nuwax_config::transport()
        .map_err(NavStackError::InvalidConfig)?
        .lcm_url
        .clone();
    LcmTransport::new_real(&provider)
        .map_err(|error| NavStackError::Internal(format!("real LCM startup failed: {error}")))
}

async fn typed_stream(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
) -> NavStackResult<LcmFrameStream> {
    let topic =
        protocol_topic(channel).map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(type_name)
        .map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    if type_name == "nav_msgs.OccupancyGrid" && planar_navigation_enabled() {
        return planar_navigation_stream(transport,topic).await;
    }
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

fn planar_navigation_enabled() -> bool {
    nuwax_config::geometry()
        .expect("启动已捕获 geometry 配置")
        .pipeline
        == "slam_2d"
}

/// 每个消费者独立核对运行、会话和观测顺序，清空与切域先向执行者交付撤销。
async fn planar_navigation_stream(transport:&LcmTransport,topic:LcmTopic)->NavStackResult<LcmFrameStream> {
    use nuwax_codec::lcm::planar_navigation::{self,Admission};
    let message_type=LcmMessageType::new(planar_navigation::TYPE_NAME)
        .map_err(|e|NavStackError::InvalidConfig(e.to_string()))?;
    let stream=transport.subscribe(topic,Some(message_type)).await;
    let mapped=stream.scan((Admission::default(),std::time::Instant::now()),|(admission,last_diagnostic),item| {
        let mut output=Vec::new();
        match item {
            Err(error)=>output.push(Err(error)),
            Ok(frame)=>{
                match planar_navigation::decode_body(&frame.payload)
                    .map_err(|e|e.to_string())
                    .and_then(|map|admission.admit(&map).map(|changed|(map,changed))) {
                    Ok((map,changed))=>{
                        let convert=|grid:&schema::OccupancyGrid| -> TransportResult<RawLcmFrame> {
                            let mut translated=frame.clone();
                            translated.payload=schema::encode_occupancy_grid_body(grid)?;
                            translated.msg_type=LcmMessageType::new("nav_msgs.OccupancyGrid")
                                .map_err(|e|nuwax_transport::TransportError::MessageType(e.to_string()))?;
                            Ok(translated)
                        };
                        if changed || map.cleared {
                            let cleared=schema::OccupancyGrid {header:map.grid.header.clone(),..Default::default()};
                            output.push(convert(&cleared));
                        }
                        if !map.cleared {output.push(convert(&map.grid));}
                    },
                    Err(error)=>{
                        if last_diagnostic.elapsed()>=Duration::from_secs(1) {
                            eprintln!("二维导航观测拒收：{error}");
                            *last_diagnostic=std::time::Instant::now();
                        }
                    },
                }
            },
        }
        futures::future::ready(Some(futures::stream::iter(output)))
    }).flatten();
    Ok(Box::pin(mapped))
}

fn build_body_frame(channel: &str, type_name: &str, body: Vec<u8>) -> NavStackResult<RawLcmFrame> {
    let topic =
        protocol_topic(channel).map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(type_name)
        .map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    Ok(RawLcmFrameBuilder::new(topic, message_type)
        .payload(body)
        .build())
}

async fn publish_body(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
    body: Vec<u8>,
) -> NavStackResult<()> {
    transport
        .publish(build_body_frame(channel, type_name, body)?)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish {channel}: {error}")))
}

fn protocol_topic(channel: &str) -> Result<LcmTopic, nuwax_protocol::LcmTopicParseError> {
    if channel.starts_with('/') {
        LcmTopic::new(channel)
    } else {
        LcmTopic::new(&format!("/{channel}"))
    }
}

fn required_frame(
    item: Option<TransportResult<RawLcmFrame>>,
    component: &str,
) -> NavStackResult<RawLcmFrame> {
    item.ok_or_else(|| NavStackError::Internal(format!("{component} LCM stream closed")))?
        .map_err(|error| NavStackError::Internal(format!("{component} LCM stream: {error}")))
}

#[cfg(feature = "guard-runtime")]
const INPUT_DIAGNOSTIC_INTERVAL: Duration = Duration::from_secs(1);

#[cfg(feature = "guard-runtime")]
#[derive(Debug, PartialEq)]
enum OwnerInput<T> {
    Accepted(T),
    Rejected(String),
}

#[cfg(feature = "guard-runtime")]
type OwnerInputResult<T> = NavStackResult<OwnerInput<T>>;

#[cfg(feature = "guard-runtime")]
#[derive(Default)]
struct InputDiagnosticLimiter {
    next_emit: Option<Instant>,
    suppressed: u64,
}

#[cfg(feature = "guard-runtime")]
impl InputDiagnosticLimiter {
    fn reject(&mut self, component: &str, reason: &str) {
        if let Some(suppressed) = self.record_at(Instant::now()) {
            tracing::warn!(input=component, reason, suppressed, "导航输入被拒绝");
        }
    }

    fn record_at(&mut self, now: Instant) -> Option<u64> {
        if self.next_emit.is_none_or(|deadline| now >= deadline) {
            let suppressed = std::mem::take(&mut self.suppressed);
            self.next_emit = Some(now + INPUT_DIAGNOSTIC_INTERVAL);
            Some(suppressed)
        } else {
            self.suppressed = self.suppressed.saturating_add(1);
            None
        }
    }
}

#[cfg(feature = "guard-runtime")]
fn runtime_input_error<T>(
    component: &str,
    error: nuwax_nav_runtime::RuntimeError,
) -> OwnerInputResult<T> {
    use nuwax_nav_runtime::RuntimeError;

    let fatal = matches!(
        &error,
        RuntimeError::SchedulerStopped
    );
    if fatal {
        Err(module_error(component, error))
    } else {
        Ok(OwnerInput::Rejected(error.to_string()))
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn module_input_error<T>(component: &str, error: nuwax_nav_base::NavError) -> OwnerInputResult<T> {
    use nuwax_nav_base::NavError;

    let external = matches!(
        &error,
        NavError::InvalidInput { .. }
            | NavError::MapUnavailable
            | NavError::GoalOutOfBounds
            | NavError::GoalOccupied
            | NavError::NoPath { .. }
    );
    if external {
        Ok(OwnerInput::Rejected(error.to_string()))
    } else {
        Err(module_error(component, error))
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn module_tick_error(component: &str, error: nuwax_nav_base::NavError) -> NavStackError {
    module_error(component, error)
}

fn signal_ready(ready: ReadySender) -> NavStackResult<()> {
    ready
        .send(Ok(()))
        .map_err(|_| NavStackError::Internal("startup handshake receiver dropped".into()))
}



#[cfg(feature = "guard-runtime")]
pub(crate) fn map_pose_owner(node: &TaskChannels, body_reference_frame_id: Arc<str>) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("map-pose-adapter", Box::new(move |shutdown, ready| {
        Box::pin(run_map_pose_adapter(node, body_reference_frame_id, shutdown, ready))
    }))
}

#[cfg(feature = "guard-runtime")]
fn map_pose_input_error<T>(
    component: &str,
    error: crate::map_pose::MapPoseError,
) -> OwnerInputResult<T> {
    if matches!(&error, crate::map_pose::MapPoseError::InvalidConfig(_)) {
        Err(module_error(component, error))
    } else {
        Ok(OwnerInput::Rejected(error.to_string()))
    }
}

#[cfg(feature = "guard-runtime")]
fn ingest_map_pose_tf(
    adapter: &mut crate::map_pose::MapPoseAdapter,
    payload: &[u8],
) -> OwnerInputResult<Option<crate::map_pose::MapPoseObservation>> {
    let message = match schema::decode_tf_message_body(payload) {
        Ok(message) => message,
        Err(error) => return Ok(OwnerInput::Rejected(format!("TFMessage decode: {error}"))),
    };
    match adapter.observe_tf_message(&message) {
        Ok(observed) if observed.accepted => Ok(OwnerInput::Accepted(observed.map_pose)),
        Ok(_) => Ok(OwnerInput::Rejected(
            "TFMessage has no newer canonical map->odom transform".into(),
        )),
        Err(error) => map_pose_input_error("map_pose_adapter.tf", error),
    }
}

#[cfg(feature = "guard-runtime")]
enum MapPoseOdometryIngress {
    Accepted(crate::map_pose::MapPoseObservation),
    AwaitingTransform(Box<schema::Odometry>, String),
    Rejected(String),
}

#[cfg(feature = "guard-runtime")]
fn transform_map_pose_odometry(
    adapter: &crate::map_pose::MapPoseAdapter,
    odometry: schema::Odometry,
) -> NavStackResult<MapPoseOdometryIngress> {
    match adapter.transform_odometry(&odometry) {
        Ok(Some(observed)) => Ok(MapPoseOdometryIngress::Accepted(observed)),
        Ok(None) => Ok(MapPoseOdometryIngress::AwaitingTransform(
            Box::new(odometry),
            "odometry is waiting for a canonical map->odom transform".into(),
        )),
        Err(error @ crate::map_pose::MapPoseError::MisalignedTransform { .. }) => {
            Ok(MapPoseOdometryIngress::AwaitingTransform(
                Box::new(odometry),
                error.to_string(),
            ))
        }
        Err(error @ crate::map_pose::MapPoseError::InvalidConfig(_)) => {
            Err(module_error("map_pose_adapter.odometry", error))
        }
        Err(error) => Ok(MapPoseOdometryIngress::Rejected(error.to_string())),
    }
}

#[cfg(feature = "guard-runtime")]
fn ingest_map_pose_odometry(
    adapter: &crate::map_pose::MapPoseAdapter,
    payload: &[u8],
) -> NavStackResult<MapPoseOdometryIngress> {
    let odometry = match schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, payload)) {
        Ok(odometry) => odometry,
        Err(error) => {
            return Ok(MapPoseOdometryIngress::Rejected(format!(
                "Odometry decode: {error}"
            )))
        }
    };
    transform_map_pose_odometry(adapter, odometry)
}

#[cfg(feature = "guard-runtime")]
async fn publish_map_pose_observation(
    transport: &LcmTransport,
    channel: &str,
    observed: crate::map_pose::MapPoseObservation,
) -> NavStackResult<()> {
    let body = schema::encode_pose_stamped_body(&observed.pose)
        .map_err(|e| NavStackError::Internal(e.to_string()))?;
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await
}

#[cfg(feature = "guard-runtime")]
fn bound_map_pose_adapter(
    map_frame: String,
    odom_frame: String,
    body_reference_frame_id: &str,
) -> NavStackResult<crate::map_pose::MapPoseAdapter> {
    crate::map_pose::MapPoseAdapter::new(map_frame, odom_frame, body_reference_frame_id)
        .map(|adapter| adapter.require_body_reference())
        .map_err(|error| module_error("map_pose_adapter", error))
}

#[cfg(feature = "guard-runtime")]
async fn run_map_pose_adapter(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
    let mut transforms = typed_stream(&transport, &node.inputs[1], "tf2_msgs.TFMessage").await?;
    let mut adapter = bound_map_pose_adapter(
        nuwax_config::robot()
            .map_err(NavStackError::InvalidConfig)?
            .map_frame
            .clone(),
        nuwax_config::robot()
            .map_err(NavStackError::InvalidConfig)?
            .odom_frame
            .clone(),
        &body_reference_frame_id,
    )?;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut pending_odometry: Option<Box<schema::Odometry>> = None;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => return Ok(()),
            message = transforms.next() => {
                let mut next = message;
                let mut newest = None;
                loop {
                    let frame = required_frame(next, "map_pose_adapter.tf")?;
                    match ingest_map_pose_tf(&mut adapter, &frame.payload)? {
                        OwnerInput::Accepted(Some(observed)) => {
                            pending_odometry=None;
                            newest=Some(observed);
                        }
                        OwnerInput::Accepted(None) => {
                            if let Some(odometry) = pending_odometry.take() {
                                match transform_map_pose_odometry(&adapter, *odometry)? {
                                    MapPoseOdometryIngress::Accepted(observed) => {
                                        newest=Some(observed);
                                    }
                                    MapPoseOdometryIngress::AwaitingTransform(odometry, reason) => {
                                        pending_odometry = Some(odometry);
                                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                                    }
                                    MapPoseOdometryIngress::Rejected(reason) => {
                                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                                    }
                                }
                            }
                        }
                        OwnerInput::Rejected(reason) => diagnostics.reject("map_pose_adapter.tf", &reason),
                    }
                    match transforms.next().now_or_never() {
                        Some(message) => next = message,
                        None => break,
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        observed).await?;
                }
            }
            message = odometry.next() => {
                let mut newest = None;
                {
                    let frame = latest_pose_frame(message, &mut odometry, "map_pose_adapter.odometry")?;
                    match ingest_map_pose_odometry(&adapter, &frame.payload)? {
                        MapPoseOdometryIngress::Accepted(observed) => {
                            pending_odometry=None;
                            newest=Some(observed);
                        }
                        MapPoseOdometryIngress::AwaitingTransform(candidate, reason) => {
                            pending_odometry=Some(candidate);
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                        MapPoseOdometryIngress::Rejected(reason) => {
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        observed).await?;
                }
            }
        }
    }
}

#[cfg(feature = "guard-runtime")]
pub(crate) fn guarded_nav_scheduler_owner(node: &TaskChannels, body_reference_frame_id: Arc<str>) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("guarded-nav-scheduler", Box::new(move |shutdown, ready| {
        Box::pin(run_guarded_nav_scheduler(node, body_reference_frame_id, shutdown, ready))
    }))
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone, Debug, Default, PartialEq)]
struct GuardedNavTelemetry {
    runtime_generation: u64,
    public_session_id: String,
    motion_generation: i64,
    map_frame: String,
    pose_frame: String,
    goal: [f64; 3],
    current: [f64; 3],
    // 旧线格式的第三项仍为yaw；三维高度在内部独立保留用于真实进度。
    current_z: f64,
    goal_z: f64,
    surface_navigation: bool,
    initial_distance: f64,
    progress: f64,
    path_revision: i64,
    planner_runtime_generation: i64,
}

#[cfg(feature = "guard-runtime")]
fn guarded_goal_execution_identity(
    goal: &schema::NavigationGoal,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity { goal_key: goal.goal_key(), session_id: goal.session_id.clone(), planner_runtime_generation: 0, path_revision: 0 }
}

#[cfg(feature = "guard-runtime")]
fn guarded_path_execution_identity(
    path: &schema::PathEnvelope,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity {
        goal_key: path.goal_key(),
        session_id: path.session_id.clone(),

        planner_runtime_generation: path.planner_runtime_generation,
        path_revision: u64::try_from(path.path_revision).expect("validated positive path revision"),
    }
}

#[cfg(feature = "guard-runtime")]
struct GuardedPoseInput {
    pose: PoseStamped,
    frame_id: String,
    current: [f64; 3],
}

#[cfg(feature = "guard-runtime")]
fn latest_pose_frame(
    first: Option<TransportResult<RawLcmFrame>>,
    poses: &mut LcmFrameStream,
    name: &str,
) -> NavStackResult<RawLcmFrame> {
    let mut latest = required_frame(first, name)?;
    while let Some(next) = poses.next().now_or_never() {
        latest = required_frame(next, name)?;
    }
    Ok(latest)
}

#[cfg(feature = "guard-runtime")]
fn drain_guarded_pose_input(
    first: Option<TransportResult<RawLcmFrame>>,
    poses: &mut LcmFrameStream,
    diagnostics: &mut InputDiagnosticLimiter,
) -> NavStackResult<Option<GuardedPoseInput>> {
    let frame = latest_pose_frame(first, poses, "guarded_nav_scheduler.map_pose")?;
    match decode_guarded_pose_input(&frame.payload)? {
        OwnerInput::Accepted(candidate) => Ok(Some(candidate)),
        OwnerInput::Rejected(reason) => {
            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason);
            Ok(None)
        }
    }
}

#[cfg(feature = "guard-runtime")]
fn configured_pose_source_max_age() -> NavStackResult<Duration> {
    let seconds = nuwax_config::geometry()
        .map_err(NavStackError::InvalidConfig)?
        .odom_max_age_sec;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|value| !value.is_zero())
        .ok_or_else(|| {
            NavStackError::InvalidConfig("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC必须为正有限秒数".into())
        })
}

#[cfg(any(feature="guard-runtime",feature="simple-2d",feature="replanning-astar",feature="cmu-native",feature="explore",feature="mls"))]
fn decode_navigation_goal_input(payload: &[u8]) -> Result<schema::NavigationGoal, String> {
    let goal = schema::decode_navigation_goal_body(payload).map_err(|e| e.to_string())?;
    if goal.goal.header.frame_id != "map" {
        return Err(format!(
            "目标坐标系必须为map，收到{}",
            goal.goal.header.frame_id
        ));
    }
    Ok(goal)
}

#[cfg(feature="guard-runtime")]
fn decode_guarded_pose_input(payload: &[u8]) -> OwnerInputResult<GuardedPoseInput> {
    let pose_message =
        match schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, payload)) {
            Ok(message) => message,
            Err(error) => {
                return Ok(OwnerInput::Rejected(format!("map pose decode: {error}")));
            }
        };
    if pose_message.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "scheduler map pose frame must be map, got {}",
            pose_message.header.frame_id,
        )));
    }
    if let Err(error) =
        lcm_header_timestamp_ns(&pose_message.header, "guarded_nav_scheduler.map_pose")
    {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    if !schema_pose_is_finite_and_normalized(&pose_message.pose) {
        return Ok(OwnerInput::Rejected(
            "scheduler map pose contains non-finite values or a zero-norm quaternion".into(),
        ));
    }
    let pose = match pose_from_schema(&pose_message) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let current = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.orientation.yaw(),
    ];
    Ok(OwnerInput::Accepted(GuardedPoseInput {
        pose,
        frame_id: pose_message.header.frame_id,
        current,
    }))
}

#[cfg(feature = "guard-runtime")]
struct GuardedCostmapInput {
    observation: nuwax_nav_runtime::CostmapObservation,
    map_frame: String,
}

#[cfg(feature = "guard-runtime")]
fn commit_guarded_pose_telemetry(telemetry: &mut GuardedNavTelemetry, input: &GuardedPoseInput) {
    telemetry.map_frame = input.frame_id.clone();
    telemetry.pose_frame = input.frame_id.clone();
    telemetry.current = input.current;
    telemetry.current_z = input.pose.pose.position.z;
    update_progress(telemetry);
}

#[cfg(feature = "guard-runtime")]
fn commit_guarded_costmap_telemetry(
    telemetry: &mut GuardedNavTelemetry,
    input: &GuardedCostmapInput,
) {
    telemetry.map_frame = input.map_frame.clone();
}

#[cfg(feature = "guard-runtime")]
fn decode_guarded_costmap_input(payload: &[u8]) -> OwnerInputResult<GuardedCostmapInput> {
    let grid = match schema::decode_occupancy_grid_body(payload) {
        Ok(grid) => grid,
        Err(error) => {
            return Ok(OwnerInput::Rejected(format!(
                "OccupancyGrid decode: {error}"
            )));
        }
    };
    if grid.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "scheduler occupancy requires map frame, got frame={}",
            grid.header.frame_id,
        )));
    }
    if !grid.info.resolution.is_finite()
        || grid
            .info
            .origin
            .position
            .iter()
            .chain(grid.info.origin.orientation_xyzw.iter())
            .any(|value| !value.is_finite())
    {
        return Ok(OwnerInput::Rejected(
            "guarded_nav_scheduler.costmap metadata contains a non-finite value".into(),
        ));
    }
    let source_timestamp_ns =
        match lcm_header_timestamp_ns(&grid.header, "guarded_nav_scheduler.costmap") {
            Ok(timestamp) => timestamp,
            Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
        };
    let costmap = match decode_astar_costmap(payload) {
        Ok(costmap) => costmap,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    // 保留未知类别；全局规划拒绝未知区域，局部碰撞只判定真实占据障碍。
    Ok(OwnerInput::Accepted(GuardedCostmapInput {
        observation: nuwax_nav_runtime::CostmapObservation {
            costmap: nuwax_nav_runtime::ExecutionMap::Planar(costmap),
            source_timestamp_ns,
        },
        map_frame: grid.header.frame_id,
    }))
}

#[cfg(feature = "guard-runtime")]
fn schema_pose_is_finite_and_normalized(pose: &schema::Pose) -> bool {
    let finite = pose
        .position
        .iter()
        .chain(pose.orientation_xyzw.iter())
        .all(|value| value.is_finite());
    let norm_squared = pose
        .orientation_xyzw
        .iter()
        .map(|value| value * value)
        .sum::<f64>();
    finite && norm_squared.is_finite() && norm_squared > f64::EPSILON
}

#[cfg(feature = "guard-runtime")]
fn guarded_scheduler_result(
    scheduler_exited_first: bool,
    loop_result: NavStackResult<()>,
    shutdown_result: NavStackResult<()>,
    task_result: NavStackResult<()>,
) -> NavStackResult<()> {
    if scheduler_exited_first && task_result.is_err() {
        return task_result;
    }
    loop_result?;
    shutdown_result?;
    task_result
}

#[cfg(feature = "guard-runtime")]
async fn coordinate_guarded_scheduler<LoopFuture, TaskFuture, Shutdown, ShutdownFuture>(
    loop_future: LoopFuture,
    task_future: TaskFuture,
    shutdown: Shutdown,
) -> NavStackResult<()>
where
    LoopFuture: Future<Output = NavStackResult<()>>,
    TaskFuture: Future<Output = NavStackResult<()>>,
    Shutdown: FnOnce() -> ShutdownFuture,
    ShutdownFuture: Future<Output = NavStackResult<()>>,
{
    tokio::pin!(loop_future);
    tokio::pin!(task_future);

    tokio::select! {
        biased;
        task_result = &mut task_future => {
            let loop_result = loop_future.await;
            guarded_scheduler_result(true, loop_result, Ok(()), task_result)
        }
        loop_result = &mut loop_future => {
            let shutdown_result = shutdown().await;
            let task_result = task_future.await;
            guarded_scheduler_result(false, loop_result, shutdown_result, task_result)
        }
    }
}

#[cfg(feature = "guard-runtime")]
async fn run_guarded_nav_scheduler(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let source_max_age = configured_pose_source_max_age()?;
    let transport = lcm_transport()?;
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.NavigationGoal").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let costmap_type = if planar_navigation_enabled() {nuwax_codec::lcm::planar_navigation::TYPE_NAME} else {"nav_msgs.OccupancyGrid"};
    let costmaps = typed_stream(&transport, &node.inputs[3], costmap_type).await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let socket_path = nuwax_nav_runtime::motion_guard_socket_path();
    let scheduler_config = nuwax_nav_runtime::SchedulerConfig::default();
    let (scheduler, task) = nuwax_nav_runtime::spawn_production_scheduler(
        socket_path,
        scheduler_config,
    )
    .await
    .map_err(|error| module_error("guarded_nav_scheduler.connect", error))?;
    let status = scheduler.subscribe_status();
    signal_ready(ready)?;

    let loop_future = run_guarded_nav_loop(&node, &transport, &scheduler, shutdown, goals, poses, paths, costmaps, cancels, status, Some(source_max_age), planar_navigation_enabled(), &body_reference_frame_id, None);
    let task_future = async move {
        task.await
            .map_err(|error| NavStackError::Internal(format!("guard scheduler join: {error}")))?
            .map_err(|error| module_error("guarded_nav_scheduler.task", error))
    };
    coordinate_guarded_scheduler(loop_future, task_future, || async {
        scheduler
            .shutdown()
            .await
            .map_err(|error| module_error("guarded_nav_scheduler.shutdown", error))
    })
    .await
}

#[cfg(feature = "guard-runtime")]
enum GuardedMapFrame {
    Planar(RawLcmFrame),
    #[cfg(feature = "mls")]
    Surface(surface_dataflow::PreparedSurfaceMap),
    #[cfg(feature = "mls")]
    RecentCloud(RawLcmFrame),
}

#[cfg(feature = "guard-runtime")]
impl From<RawLcmFrame> for GuardedMapFrame {
    fn from(frame: RawLcmFrame) -> Self {
        Self::Planar(frame)
    }
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone, Debug, PartialEq, Eq)]
struct GuardedMapAuthority {
    session_id: String,
    revision: i64,
    frame_id: String,
}

#[cfg(feature = "guard-runtime")]
impl GuardedMapAuthority {
    fn same_domain(&self, other: &Self) -> bool {
        self.session_id == other.session_id
    }

    fn permits_goal(&self, goal: &schema::NavigationGoal) -> bool {
        self.frame_id == goal.goal.header.frame_id
    }
}

#[cfg(feature = "guard-runtime")]
enum GuardedNavEvent {
    Pose(Option<TransportResult<RawLcmFrame>>),
    Costmap(Option<TransportResult<GuardedMapFrame>>),
    Status(Result<(), tokio::sync::watch::error::RecvError>),
    Goal(Option<TransportResult<RawLcmFrame>>),
    Path(Option<TransportResult<RawLcmFrame>>),
    Cancel(Option<TransportResult<RawLcmFrame>>),
}

#[cfg(feature = "guard-runtime")]
async fn wait_guarded_nav_event<M, T>(
    goals: &mut LcmFrameStream,
    poses: &mut LcmFrameStream,
    paths: &mut LcmFrameStream,
    costmaps: &mut M,
    cancels: &mut LcmFrameStream,
    status: &mut tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
) -> GuardedNavEvent
where
    M: Stream<Item = TransportResult<T>> + Unpin,
    T: Into<GuardedMapFrame>,
{
    tokio::select! {
        message = poses.next() => GuardedNavEvent::Pose(message),
        message = costmaps.next() => GuardedNavEvent::Costmap(message.map(|frame| frame.map(Into::into))),
        changed = status.changed() => GuardedNavEvent::Status(changed),
        message = goals.next() => GuardedNavEvent::Goal(message),
        message = paths.next() => GuardedNavEvent::Path(message),
        message = cancels.next() => GuardedNavEvent::Cancel(message),
    }
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn run_guarded_nav_loop<M, T>(
    node: &TaskChannels,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    mut shutdown: oneshot::Receiver<()>,
    mut goals: LcmFrameStream,
    mut poses: LcmFrameStream,
    mut paths: LcmFrameStream,
    mut costmaps: M,
    mut cancels: LcmFrameStream,
    mut status: tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
    source_max_age: Option<Duration>,
    planar_mode: bool,
    _body_reference_frame_id: &str,
    recent_source_max_age: Option<Duration>,
) -> NavStackResult<()>
where
    M: Stream<Item = TransportResult<T>> + Unpin + Send,
    T: Into<GuardedMapFrame> + Send,
{
    let process_runtime_generation = runtime_generation();
    let mut telemetry = GuardedNavTelemetry {
        runtime_generation: process_runtime_generation,
        surface_navigation: node.id == "surface_nav_scheduler",
        ..GuardedNavTelemetry::default()
    };
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut surface_authority: Option<GuardedMapAuthority> = None;
    #[cfg(feature="mls")]
    let mut recent_source=surface_dataflow::RecentSourceState::new(recent_source_max_age);
    if recent_source_max_age.is_some() {
        scheduler.update_obstacle_source(None,false).await
            .map_err(|error|module_error("recent source initial health",error))?;
    }
    let mut planar_admission=nuwax_codec::lcm::planar_navigation::Admission::default();
    let mut publications = GuardedStatusPublications::default();
    let mut diagnostics = InputDiagnosticLimiter::default();

    let initial_status = status.borrow().clone();
    publish_guarded_nav_observation(
        transport,
        node,
        initial_status,
        &telemetry,
        None,
        &mut publications,
    )
    .await?;

    loop {
        let event = tokio::select! {
            biased;
            _ = &mut shutdown => return Ok(()),
            event = wait_guarded_nav_event(
                &mut goals, &mut poses, &mut paths, &mut costmaps, &mut cancels, &mut status,
            ) => event,
        };
        match event {
            GuardedNavEvent::Status(changed) => {
                changed.map_err(|_| {
                    NavStackError::Internal("guard scheduler status stream closed".into())
                })?;
                let snapshot = status.borrow().clone();
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, None, &mut publications).await?;
            }
            GuardedNavEvent::Pose(message) => {
                let Some(input) = drain_guarded_pose_input(
                    message,
                    &mut poses,
                    &mut diagnostics,
                )?
                else {
                    continue;
                };
                let source_age_limit = source_max_age;
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_pose_telemetry(&mut candidate_telemetry, &input);
                let settlement = (async {
                    match source_age_limit {
                        Some(maximum) => scheduler
                            .update_odometry_with_source_age(input.pose, maximum).await,
                        None => scheduler.update_odometry(input.pose).await,
                    }
                }).await;
                let accepted = settlement.is_ok();
                let ordinary = match settlement {
                    Ok(()) => { telemetry = candidate_telemetry; Ok(()) }
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.map_pose", error) {
                        Ok(OwnerInput::Rejected(reason)) => {
                            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason); Ok(())
                        }
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                };
                ordinary?;
                if !accepted { continue; }

                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, Some(planner_inputs_fresh), &mut publications).await?;
            }
            GuardedNavEvent::Costmap(message) => {
                let frame = message
                    .ok_or_else(|| NavStackError::Internal("guarded map stream closed".into()))?
                    .map_err(|error| module_error("guarded map input", error))?;
                let (decoded, received_at) = match frame {
                    #[cfg(feature="mls")]
                    GuardedMapFrame::RecentCloud(frame) => {
                        let settlement = (recent_source.consume(&frame.payload, surface_authority.as_ref(), scheduler)).await;
                        if let Err(reason) = settlement {
                            diagnostics.reject("recent obstacle source", &reason);
                        }

                        continue;
                    }
                    GuardedMapFrame::Planar(frame) => {
                        let source_received_at = None;
                        let body = if planar_mode {
                            let value=match nuwax_codec::lcm::planar_navigation::decode_body(&frame.payload) {
                                Ok(value)=>value,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error.to_string());continue;}
                            };
                            let changed=match planar_admission.admit(&value) {
                                Ok(changed)=>changed,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error);continue;}
                            };

                            let authority = GuardedMapAuthority {
                                session_id: value.session_id.clone(),
                                revision: value.map_revision,
                                frame_id: value.grid.header.frame_id.clone(),
                            };
                            if changed || value.cleared {
                                // 清空与切域由会话和观测修订隔离，不以时间戳建立水位。
                                scheduler.invalidate_map_source("二维导航来源已撤销或切换",
                                    nuwax_nav_runtime::MapInvalidationKind::Invalidated).await
                                    .map_err(|e|module_error("planar map invalidation",e))?;

                                let snapshot=scheduler.status();
                                publish_guarded_nav_observation(transport,node,snapshot.clone(),&telemetry,
                                    Some(false),&mut publications).await?;
                            }
                            surface_authority=if value.cleared {None} else {Some(authority)};
                            if value.cleared {continue;}
                            schema::encode_occupancy_grid_body(&value.grid)
                                .map_err(|e|NavStackError::Internal(e.to_string()))?
                        } else {frame.payload.to_vec()};
                        (decode_guarded_costmap_input(&body)?, source_received_at)
                    }
                    #[cfg(feature = "mls")]
                    GuardedMapFrame::Surface(update) => {
                        let changed_domain = surface_authority
                            .as_ref()
                            .is_some_and(|previous| !previous.same_domain(&update.authority));
                        if changed_domain || update.snapshot.is_err() {
                            scheduler
                                .invalidate_map_source(
                                    update
                                        .snapshot
                                        .as_ref()
                                        .err()
                                        .cloned()
                                        .unwrap_or_else(|| "surface map domain changed".into()),
                                    if changed_domain || !update.awaiting_build {
                                        nuwax_nav_runtime::MapInvalidationKind::Invalidated
                                    } else {
                                        nuwax_nav_runtime::MapInvalidationKind::TemporaryUnavailable
                                    },
                                )
                                .await
                                .map_err(|error| module_error("surface map invalidation", error))?;

                            // 精细源失败或换域时保留待执行目标，之后的取消仍须
                            // 通过原状态通道回报该目标。

                            let snapshot = scheduler.status();
                            publish_guarded_nav_observation(
                                transport,
                                node,
                                snapshot.clone(),
                                &telemetry,
                                Some(false),
                                &mut publications,
                            )
                            .await?;
                        }
                        if changed_domain || update.snapshot.is_err() {
                            let settlement = (recent_source.reset(scheduler)).await;
                            let ordinary = settlement.map_err(|error|module_error("recent source domain reset",error));
                            ordinary?;
                        }
                        surface_authority = Some(update.authority.clone());
                        let surface = match update.snapshot {
                            Ok(surface) => surface,
                            Err(reason) => {
                                diagnostics.reject("surface map", &reason);
                                continue;
                            }
                        };
                        let surface=recent_source.attach_terrain(surface);
                        (
                            OwnerInput::Accepted(GuardedCostmapInput {
                                map_frame: update.authority.frame_id.clone(),
                                observation: nuwax_nav_runtime::CostmapObservation {
                                    costmap: nuwax_nav_runtime::ExecutionMap::SurfaceUpdate(nuwax_nav_runtime::surface_changes::SurfaceMapUpdate {
                                        snapshot: surface, change_basis: update.change_basis, changes: update.geometry_changes,
                                    }),
                                    source_timestamp_ns: update.source_timestamp_ns,
                                },
                            }),
                            Some(update.received_at),
                        )
                    }
                };
                let input = match decoded {
                    OwnerInput::Accepted(input) => input,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("guarded_nav_scheduler.costmap", &reason);
                        continue;
                    }
                };
                // 保留活动会话；只有显式替换目标才能抢占。
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_costmap_telemetry(&mut candidate_telemetry, &input);
                let settlement = (scheduler.update_costmap_received(input.observation,
                        tokio::time::Instant::from_std(received_at.unwrap_or_else(Instant::now)))).await;
                let accepted = settlement.is_ok();
                let ordinary = match settlement {
                    Ok(()) => {
                        telemetry = candidate_telemetry;
                        // Observe the accepted map without taking/activating a
                        // buffered goal before an early cancellation settles.

                        Ok(())
                    }
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.costmap", error) {
                        Ok(OwnerInput::Rejected(reason)) => {
                            diagnostics.reject("guarded_nav_scheduler.costmap", &reason); Ok(())
                        }
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                };
                ordinary?;
                if !accepted { continue; }

                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(transport, node, snapshot, &mut telemetry, Some(planner_inputs_fresh), &mut publications).await?;
            }
            GuardedNavEvent::Goal(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.goal")?;
                let navigation_goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "goal",
                            &format!("NavigationGoal rejected: {error}"),
                        )
                        .await?;
                        continue;
                    }
                };
                if latest_goal.as_ref()==Some(&navigation_goal){continue;}
                activate_guarded_goal(
                    node,
                    transport,
                    scheduler,
                    &mut telemetry,
                    &mut latest_goal,
                    navigation_goal,
                )
                .await?;
            }
            GuardedNavEvent::Path(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.path")?;
                let Some(active_goal) = latest_goal.as_ref() else {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "no active versioned goal lease",
                    )
                    .await?;
                    continue;
                };

                let path_envelope = match schema::decode_path_envelope_body(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "path",
                            &format!("PathEnvelope rejected: {error}"),
                        )
                        .await?;
                        continue;
                    }
                };
                if let Err(reason) = validate_path_goal_correlation(&path_envelope, active_goal) {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        &reason,
                    )
                    .await?;
                    continue;
                }
                let path = match execution_path_from_schema(
                    &path_envelope.path,
                    node.id == "surface_nav_scheduler",
                ) {
                    Ok(path) => path,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "path",
                            &error.to_string(),
                        )
                        .await?;
                        continue;
                    }
                };
                let settlement = (scheduler
                    .submit_path(nuwax_nav_runtime::VersionedPath {
                        route_kind: if path_envelope.route_kind == 1 {nuwax_nav_runtime::RouteKind::Approach}
                            else {nuwax_nav_runtime::RouteKind::Complete},
                        session_id: path_envelope.session_id.clone(),
                        execution_identity: guarded_path_execution_identity(&path_envelope),
                        path_revision: u64::try_from(path_envelope.path_revision)
                            .expect("validated positive path revision"),
                        path_yaws: if node.id == "surface_nav_scheduler" {
                            None
                        } else {
                            Some(
                                path_envelope
                                    .path
                                    .poses
                                    .iter()
                                    .map(|pose| {
                                        let [x, y, z, w] = pose.pose.orientation_xyzw;
                                        Quaternion { x, y, z, w }.yaw()
                                    })
                                    .collect(),
                            )
                        },
                        path,
                    })).await;
                let ordinary = match settlement {
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.path", error) {
                        Ok(OwnerInput::Rejected(reason)) => publish_guarded_rejection(
                            transport, &node.outputs[1], scheduler.status(), &telemetry, "path", &reason,
                        ).await,
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                    Ok(()) => {
                        {
                            telemetry.path_revision = path_envelope.path_revision;
                            telemetry.planner_runtime_generation = path_envelope.planner_runtime_generation;
                        }
                        Ok(())
                    }
                };
                ordinary?;
            }
            GuardedNavEvent::Cancel(message) => {
                let frame = required_frame(message, "导航取消")?;
                let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
                    Ok(cancel) => cancel,
                    Err(error) => { diagnostics.reject("导航取消", &error.to_string()); continue; }
                };
                if latest_goal.as_ref().is_some_and(|goal| validate_cancel_goal_correlation(&cancel, goal).is_ok()) {
                    if let Err(error) = scheduler.cancel(&cancel.session_id).await {
                        if let OwnerInput::Rejected(reason) = runtime_input_error::<()>("导航取消", error)? {
                            diagnostics.reject("导航取消", &reason);
                        }
                    }
                    let snapshot = scheduler.status();
                    publish_guarded_nav_observation(transport, node, snapshot.clone(), &telemetry, None, &mut publications).await?;
                }

            }
        }
    }
}

#[cfg(feature = "guard-runtime")]
async fn activate_guarded_goal(
    node: &TaskChannels,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    latest_goal: &mut Option<schema::NavigationGoal>,
    navigation_goal: schema::NavigationGoal,
) -> NavStackResult<()> {
    let goal = pose_from_schema(&navigation_goal.goal)?;
    telemetry.goal = [
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.orientation.yaw(),
    ];
    telemetry.goal_z = goal.pose.position.z;
    telemetry.public_session_id = navigation_goal.session_id.clone();
    telemetry.motion_generation = navigation_goal.motion_generation as i64;
    telemetry.path_revision = 0;
    telemetry.planner_runtime_generation = 0;
    telemetry.initial_distance = telemetry_distance(telemetry);
    telemetry.progress = 0.;
    *latest_goal = Some(navigation_goal.clone());
    let result = scheduler
        .start_session(nuwax_nav_runtime::NavSession {
            boundary: navigation_goal
                .boundary_points
                .iter()
                .map(|p| [p[0], p[1]])
                .collect(),
            options: navigation_goal.options(),
            goal: nuwax_nav_runtime::ExecutionGoal::Body(goal),
            session_id: navigation_goal.session_id.clone(),
            execution_identity: guarded_goal_execution_identity(&navigation_goal),
            motion_generation: navigation_goal.motion_generation,
        })
        .await;
    if let Err(error) = result {
        match runtime_input_error::<()>("guarded_nav_scheduler.goal", error)? {
            OwnerInput::Rejected(reason) => {
                publish_guarded_rejection(
                    transport,
                    &node.outputs[1],
                    scheduler.status(),
                    telemetry,
                    "goal",
                    &reason,
                )
                .await?
            }
            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
        }
    } else {
        publish_guarded_nav_state(
            transport,
            &node.outputs[1],
            scheduler.status(),
            telemetry,
            None,
        )
        .await?;
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn runtime_generation() -> u64 {
    static PROCESS_RUNTIME_GENERATION: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *PROCESS_RUNTIME_GENERATION.get_or_init(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
            .min(u64::MAX as u128) as u64
    })
}

#[cfg(feature = "guard-runtime")]
fn clear_guarded_session_telemetry(telemetry: &mut GuardedNavTelemetry) {
    telemetry.public_session_id.clear();
    telemetry.motion_generation = 0;
    telemetry.goal = [0.0; 3];
    telemetry.goal_z = 0.0;
    telemetry.path_revision = 0;
    telemetry.planner_runtime_generation = 0;
    telemetry.initial_distance = 0.0;
    telemetry.progress = 0.0;
}

#[cfg(feature = "guard-runtime")]
fn validate_path_goal_correlation(path: &schema::PathEnvelope, goal: &schema::NavigationGoal) -> Result<(), String> {
    if path.session_id != goal.session_id {
        return Err("PathEnvelope goal identity does not match the active NavigationGoal".into());
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn validate_cancel_goal_correlation(
    cancel: &schema::CancelNavigation,
    goal: &schema::NavigationGoal,
) -> Result<(), String> {
    if cancel.session_id.is_empty() || cancel.session_id == goal.session_id { Ok(()) }
    else { Err("取消目标号不匹配".into()) }
}

#[cfg(any(feature = "guard-runtime", feature = "click-goal"))]
fn lcm_header_timestamp_ns(header: &schema::Header, component: &str) -> NavStackResult<i64> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err(NavStackError::InvalidConfig(format!(
            "{component} has an invalid source timestamp",
        )));
    }
    i64::from(header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(header.stamp_nsec)))
        .filter(|timestamp| *timestamp > 0)
        .ok_or_else(|| {
            NavStackError::InvalidConfig(format!("{component} source timestamp must be positive",))
        })
}

#[cfg(feature = "guard-runtime")]
fn telemetry_distance(telemetry: &GuardedNavTelemetry) -> f64 {
    let xy = distance_2d(telemetry.goal, telemetry.current);
    if telemetry.surface_navigation {
        xy.hypot(telemetry.goal_z - telemetry.current_z)
    } else {
        xy
    }
}

#[cfg(feature = "guard-runtime")]
fn update_progress(telemetry: &mut GuardedNavTelemetry) {
    if telemetry.initial_distance > f64::EPSILON {
        let remaining = telemetry_distance(telemetry);
        telemetry.progress = (1.0 - remaining / telemetry.initial_distance).clamp(0.0, 1.0);
    }
}

#[cfg(feature = "guard-runtime")]
#[derive(Default)]
struct GuardedStatusPublications {
    goal: Option<(String, i64)>,
    applied_commands: u64,
    last_terminal: Option<nuwax_nav_runtime::TerminalOutcome>,
    reached_published: bool,
}

#[cfg(feature = "guard-runtime")]
impl GuardedStatusPublications {
    fn frames(
        &mut self,
        node: &TaskChannels,
        status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry,
        planner_observation: Option<bool>,
    ) -> NavStackResult<Vec<RawLcmFrame>> {
        let mut frames = Vec::with_capacity(4);
        if !telemetry.public_session_id.is_empty() {
            let goal = (
                telemetry.public_session_id.clone(),
                telemetry.motion_generation,
            );
            if self.goal.as_ref() != Some(&goal) {
                self.goal = Some(goal);
                self.applied_commands = 0;
                self.last_terminal = None;
                self.reached_published = false;
            }
            if status.applied_commands > self.applied_commands {
                if let Some(twist) = status.last_applied_twist {
                    frames.push(build_twist2_frame(&node.outputs[0], twist)?);
                }
                self.applied_commands = status.applied_commands;
            }
            if status.last_terminal.is_some() && status.last_terminal != self.last_terminal {
                frames.push(build_twist2_frame(&node.outputs[0], Twist2::ZERO)?);
            }
            self.last_terminal = status.last_terminal;
            if status.last_terminal == Some(nuwax_nav_runtime::TerminalOutcome::Succeeded)
                && !self.reached_published
            {
                frames.push(build_body_frame(
                    &node.outputs[2],
                    "std_msgs.Bool",
                    schema::encode_bool_body(&schema::Bool { data: true }),
                )?);
                self.reached_published = true;
            }
        }
        frames.push(build_guarded_nav_state_frame(
            &node.outputs[1],
            status,
            telemetry,
            planner_observation,
        )?);
        Ok(frames)
    }
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_tail_observation(
    transport: &LcmTransport, node: &TaskChannels, snapshot: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &mut GuardedNavTelemetry, planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    publish_guarded_nav_observation(transport, node, snapshot.clone(), telemetry, planner_observation, publications).await?;
    clear_terminal_telemetry_after_publication(telemetry, &snapshot);
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn clear_terminal_telemetry_after_publication(
    telemetry: &mut GuardedNavTelemetry,
    status: &nuwax_nav_runtime::SchedulerStatus,
) {
    if status.last_terminal.is_some() {
        clear_guarded_session_telemetry(telemetry);
    }
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_nav_observation(
    transport: &LcmTransport,
    node: &TaskChannels,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    let frames=publications.frames(node, &status, telemetry, planner_observation)?;

    transport
        .publish_batch(frames)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish guard status batch: {error}")))?;

    Ok(())
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_nav_state(
    transport: &LcmTransport,
    channel: &str,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
) -> NavStackResult<()> {
    transport
        .publish(build_guarded_nav_state_frame(
            channel,
            &status,
            telemetry,
            planner_observation,
        )?)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish {channel}: {error}")))
}

#[cfg(feature = "guard-runtime")]
fn build_guarded_nav_state_frame(
    channel: &str,
    status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
) -> NavStackResult<RawLcmFrame> {
    let message = build_guarded_navigation_status(
        &status,
        telemetry,
        planner_observation,
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default(),
    );
    let body = schema::encode_navigation_status_body(&message)
        .map_err(|error| NavStackError::Internal(format!("NavigationStatus encode: {error}")))?;
    build_body_frame(channel, "nuwax_msgs.NavigationStatus", body)
}

#[cfg(feature = "guard-runtime")]
fn build_guarded_navigation_status(
    status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    now: Duration,
) -> schema::NavigationStatus {
    use nuwax_nav_runtime::{SchedulerPhase, TerminalOutcome};

    let has_public_session = !telemetry.public_session_id.is_empty();
    let state = if !has_public_session {
        0
    } else {
        match (status.phase, status.last_terminal) {
            (_, Some(TerminalOutcome::Stopped)) => 4,
            (_, Some(TerminalOutcome::Succeeded)) => 3,
            (_, Some(TerminalOutcome::Cancelled | TerminalOutcome::Shutdown)) => 0,
            (SchedulerPhase::Failed, _) => 4,
            (SchedulerPhase::Controlling, _) => 2,
            (SchedulerPhase::Acquiring | SchedulerPhase::AwaitingPath, _) => 1,
            _ => 0,
        }
    };
    let goal_status = if !has_public_session {
        schema::NAVIGATION_GOAL_STATUS_NONE
    } else {
        match (status.phase, status.last_terminal) {
            (_, Some(TerminalOutcome::Stopped)) => schema::NAVIGATION_GOAL_STATUS_STOPPED,
            (_, Some(TerminalOutcome::Succeeded)) => schema::NAVIGATION_GOAL_STATUS_SUCCEEDED,
            (_, Some(TerminalOutcome::Cancelled | TerminalOutcome::Shutdown)) => schema::NAVIGATION_GOAL_STATUS_CANCELLED,
            (SchedulerPhase::Failed, _) => schema::NAVIGATION_GOAL_STATUS_FAILED,
            (SchedulerPhase::Controlling, _) => schema::NAVIGATION_GOAL_STATUS_EXECUTING,
            (SchedulerPhase::Acquiring | SchedulerPhase::AwaitingPath, _) => schema::NAVIGATION_GOAL_STATUS_PLANNING,
            _ => schema::NAVIGATION_GOAL_STATUS_NONE,
        }
    };
    let input_ready = status.motion_sink_ready && status.planner_inputs_fresh
        && !telemetry.map_frame.is_empty() && telemetry.map_frame == telemetry.pose_frame;
    let message = if !input_ready && !matches!(goal_status, schema::NAVIGATION_GOAL_STATUS_SUCCEEDED | schema::NAVIGATION_GOAL_STATUS_FAILED | schema::NAVIGATION_GOAL_STATUS_CANCELLED | schema::NAVIGATION_GOAL_STATUS_STOPPED) {
        let age = |stamp: i64| {
            if stamp <= 0 { return "尚未收到，年龄未知".to_string(); }
            match nuwax_nav_types_messages::source_time::data_age_ns_at(stamp as u64, now.as_nanos() as u64) {
                Ok(age) => format!("来源年龄={}毫秒", age / 1_000_000),
                Err(reason) => format!("来源年龄不可用：{reason}"),
            }
        };
        let mut missing = Vec::new();
        if !status.motion_sink_ready { missing.push("运动守卫未就绪".to_string()); }
        if !status.planner_inputs_fresh {
            missing.push(format!("{}；位姿{}，接收年龄={}毫秒；栅格/曲面{}，接收年龄={}毫秒",
                status.input_wait_reason.as_deref().unwrap_or("规划输入尚未就绪"),
                age(status.pose_source_timestamp_ns), status.pose_receipt_age_ms,
                age(status.costmap_source_timestamp_ns), status.costmap_receipt_age_ms));
        }
        if telemetry.map_frame.is_empty() || telemetry.map_frame != telemetry.pose_frame {
            missing.push(format!("地图/位姿坐标系未就绪：地图={}，位姿={}", telemetry.map_frame, telemetry.pose_frame));
        }
        format!("等待导航输入：{}", missing.join("；"))
    } else if has_public_session { status.message.clone() } else { "导航栈空闲".into() };
    let include_planner_fields = planner_observation == Some(true);
    schema::NavigationStatus {
        session_id: telemetry.public_session_id.clone(),
        robot_frame: "base_link".into(),
        stamp_sec: now.as_secs().min(i64::MAX as u64) as i64,
        stamp_nsec: now.subsec_nanos() as i32,
        state,
        goal_status,
        reason: message.clone(),
        remaining_distance_m: status.remaining_distance_m,
        input_ready,
        no_feasible_path: status.no_feasible_path && goal_status == schema::NAVIGATION_GOAL_STATUS_PLANNING,
        goal: telemetry.goal,
        current: telemetry.current,
        progress: telemetry.progress,
        message,
        runtime_generation: saturating_i64(telemetry.runtime_generation),

        pose_source_timestamp_ns: status.pose_source_timestamp_ns,
        costmap_source_timestamp_ns: status.costmap_source_timestamp_ns,
        sink_ready: i8::from(status.motion_sink_ready),
        planner_map_frame: if include_planner_fields {
            telemetry.map_frame.clone()
        } else {
            String::new()
        },
        planner_pose_frame: if include_planner_fields {
            telemetry.pose_frame.clone()
        } else {
            String::new()
        },
        motion_generation: telemetry.motion_generation,
        path_revision: telemetry.path_revision,
        planner_runtime_generation: telemetry.planner_runtime_generation,
        local_path: if state == 2
            && status.session_id.as_deref() == Some(telemetry.public_session_id.as_str())
            && i64::try_from(status.path_revision).ok() == Some(telemetry.path_revision)
        { status.local_path.clone() } else { Vec::new() },
    }
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_rejection(
    transport: &LcmTransport,
    channel: &str,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    input: &str,
    reason: &str,
) -> NavStackResult<()> {
    tracing::warn!(input, reason, session_id = %telemetry.public_session_id, "导航输入被拒绝");
    let status = guarded_rejection_status(status, input, reason);
    publish_guarded_nav_state(transport, channel, status, telemetry, None).await
}

#[cfg(feature = "guard-runtime")]
fn guarded_rejection_status(
    mut status: nuwax_nav_runtime::SchedulerStatus,
    input: &str,
    reason: &str,
) -> nuwax_nav_runtime::SchedulerStatus {
    // 停车中的原因和已确认终态不能被迟到路径的协议拒绝覆盖。
    if input=="path" && (status.last_terminal.is_some() || matches!(status.phase,
        nuwax_nav_runtime::SchedulerPhase::Cancelling|nuwax_nav_runtime::SchedulerPhase::Failed)) {
        tracing::debug!(reason,session_id=?status.session_id,"迟到路径被拒绝，保留原停车或终态原因");
        return status;
    }
    status.message = format!("REJECTED {input}: {reason}");
    status
}

#[cfg(feature = "guard-runtime")]
fn saturating_i64(value: u64) -> i64 {
    value.min(i64::MAX as u64) as i64
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime"
))]
fn distance_2d(left: [f64; 3], right: [f64; 3]) -> f64 {
    (left[0] - right[0]).hypot(left[1] - right[1])
}



#[cfg(feature = "click-goal")]
async fn run_click_goal(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut clicks =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PointStamped").await?;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = clicks.next() => {
                let frame = required_frame(message, "click_goal_router.clicked_point")?;
                let click = schema::decode_point_stamped_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("clicked_point decode: {error}")))?;
                let goal = clicked_goal_pose(&click)?;
                publish_pose(&transport, &node.outputs[0], &goal).await?;
            }
        }
    }
    Ok(())
}

#[cfg(feature = "click-goal")]
fn clicked_goal_pose(click: &schema::PointStamped) -> NavStackResult<PoseStamped> {
    if click.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "clicked goal frame must be map, got {}",
            click.header.frame_id,
        )));
    }
    lcm_header_timestamp_ns(&click.header, "click_goal_router.clicked_point")?;
    let mut goal = PoseStamped::from_pose(
        Pose3::from_pose2(nuwax_nav_types_messages::Pose2::new(
            click.point[0],
            click.point[1],
            0.0,
        )),
        NavFrameId::Map,
    );
    goal.stamp = Timestamp::from_unix(
        i64::from(click.header.stamp_sec),
        click.header.stamp_nsec as u32,
    );
    Ok(goal)
}



#[cfg(feature = "explore")]
async fn run_wavefront(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_explore_wavefront::runtime::{
        RuntimeInputs, RuntimeTickOutcome, WavefrontExplorerRuntime,
    };
    use nuwax_explore_wavefront::WavefrontGoalSelector;

    let transport = lcm_transport()?;
    let mut maps = typed_stream(&transport, &node.inputs[0], "nav_msgs.OccupancyGrid").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut reached = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let mut failed = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut stop = typed_stream(&transport, &node.inputs[4], "std_msgs.Bool").await?;

    let config = wavefront_config_from_runtime()?;
    let mut runtime = WavefrontExplorerRuntime::new(
        WavefrontGoalSelector::new(config.clone()),
        LcmWavefrontGoalSink,
        WavefrontGoalIds::default(),
        config,
    );
    runtime
        .begin()
        .map_err(|error| module_error("wavefront.begin", format!("{error:?}")))?;
    let mut latest_map = None;
    let mut latest_odometry = None;
    let mut ticker = tokio::time::interval(nuwax_explore_wavefront::runtime::NO_INPUT_TICK);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = maps.next() => {
                let frame = required_frame(message, "wavefront.occupancy_grid")?;
                latest_map = Some(decode_wavefront_grid(&frame.payload)?);
            }
            message = odometry.next() => {
                let frame = required_frame(message, "wavefront.odometry")?;
                let pose = decode_odometry_pose_body(&frame.payload)?;
                if pose.frame_id != NavFrameId::Map {
                    return Err(NavStackError::InvalidConfig(format!(
                        "wavefront odometry frame must be map, got {}",
                        pose.frame_id.as_str()
                    )));
                }
                latest_odometry = Some(pose);
            }
            message = reached.next() => {
                let frame = required_frame(message, "wavefront.goal_reached")?;
                let feedback = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront goal_reached decode: {error}")))?;
                if feedback.data {
                    runtime.report_goal_reached(true);
                }
            }
            message = failed.next() => {
                let frame = required_frame(message, "wavefront.goal_failed")?;
                let feedback = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront goal_failed decode: {error}")))?;
                if feedback.data {
                    runtime.report_goal_reached(false);
                }
            }
            message = stop.next() => {
                let frame = required_frame(message, "wavefront.stop_movement")?;
                let requested = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("wavefront stop_movement decode: {error}")))?;
                if requested.data {
                    runtime.request_stop_movement()
                        .map_err(|error| module_error("wavefront.stop_movement", format!("{error:?}")))?;
                }
            }
            _ = ticker.tick() => {
                let outcome = runtime
                    .tick(RuntimeInputs {
                        costmap: latest_map.as_ref(),
                        odom: latest_odometry.as_ref(),
                        now: unix_timestamp(),
                    })
                    .map_err(|error| module_error("wavefront.tick", format!("{error:?}")))?;
                if let RuntimeTickOutcome::Goal(goal) = outcome {
                    publish_pose(&transport, &node.outputs[0], &goal.pose).await?;
                }
            }
        }
    }

    if runtime.is_active() {
        runtime
            .request_stop_movement()
            .map_err(|error| module_error("wavefront.shutdown", format!("{error:?}")))?;
        let _ = runtime
            .tick(RuntimeInputs {
                costmap: latest_map.as_ref(),
                odom: latest_odometry.as_ref(),
                now: unix_timestamp(),
            })
            .map_err(|error| module_error("wavefront.shutdown", format!("{error:?}")))?;
    }
    if runtime.is_active() {
        runtime
            .end()
            .map_err(|error| module_error("wavefront.end", format!("{error:?}")))?;
    }
    Ok(())
}

#[cfg(feature = "explore")]
fn wavefront_config_from_runtime() -> NavStackResult<nuwax_explore_wavefront::WavefrontConfig> {
    Ok(nuwax_explore_wavefront::WavefrontConfig {
        goal_timeout_ms: nuwax_config::navigation()
            .map_err(NavStackError::InvalidConfig)?
            .wavefront_goal_timeout_ms as u64,
        ..Default::default()
    })
}

#[cfg(feature = "explore")]
struct LcmWavefrontGoalSink;

#[cfg(feature = "explore")]
impl nuwax_explore_wavefront::runtime::WavefrontGoalSink for LcmWavefrontGoalSink {
    fn publish(&mut self, _goal: nuwax_nav_types_messages::FrontierGoal) {}
}

#[cfg(feature = "explore")]
#[derive(Default)]
struct WavefrontGoalIds(u64);

#[cfg(feature = "explore")]
impl nuwax_explore_wavefront::FrontierGoalIdSource for WavefrontGoalIds {
    fn next_frontier_goal_id(
        &mut self,
        seed: &nuwax_explore_wavefront::FrontierGoalIdSeed,
    ) -> String {
        self.0 = self.0.saturating_add(1);
        format!("wavefront-{}-{}", seed.stamp.nanos, self.0)
    }
}

#[cfg(feature = "explore")]
fn decode_wavefront_grid(body: &[u8]) -> NavStackResult<OccupancyGrid2D> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "wavefront OccupancyGrid frame must be map, got {}",
            grid.header.frame_id
        )));
    }
    let orientation = grid.info.origin.orientation_xyzw;
    let identity_rotation = orientation[0].abs() <= 1e-9
        && orientation[1].abs() <= 1e-9
        && orientation[2].abs() <= 1e-9
        && (orientation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity_rotation || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "wavefront grid cannot represent a rotated or non-planar origin".into(),
        ));
    }
    let width = u32::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = u32::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    let origin = Point2::new(grid.info.origin.position[0], grid.info.origin.position[1]);
    if width == 0
        || height == 0
        || !resolution.is_finite()
        || resolution <= 0.0
        || !origin.x.is_finite()
        || !origin.y.is_finite()
    {
        return Err(NavStackError::InvalidConfig(
            "wavefront grid dimensions, resolution, and origin must be finite and valid".into(),
        ));
    }
    let expected = (width as usize).saturating_mul(height as usize);
    if grid.data.len() != expected {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {expected}",
            grid.data.len()
        )));
    }
    let cells = grid
        .data
        .into_iter()
        .map(|value| match value {
            value if value < 0 => OccupancyCell::Unknown,
            0 => OccupancyCell::Free,
            value if value >= 100 => OccupancyCell::Occupied,
            value => OccupancyCell::Cost(value as u8),
        })
        .collect();
    Ok(OccupancyGrid2D::new(
        origin, resolution, width, height, cells,
    ))
}

#[cfg(feature = "explore")]
fn unix_timestamp() -> Timestamp {
    let elapsed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    Timestamp::from_unix(
        elapsed.as_secs().min(i64::MAX as u64) as i64,
        elapsed.subsec_nanos(),
    )
}



#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_terrain_map_ext(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_terrain_map_ext::module::{
        Module, ModuleOutput, TerrainMapExtModule, TICK_PERIOD_MS,
    };

    let transport = lcm_transport()?;
    let mut scans = typed_stream(&transport, &node.inputs[0], "sensor_msgs.PointCloud2").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut module = TerrainMapExtModule::new();
    Module::start(&mut module).map_err(|error| module_error("terrain_map_ext.start", error))?;
    let mut has_odometry = false;
    let mut scan_before_odometry = None;
    let mut ticker = tokio::time::interval(Duration::from_millis(TICK_PERIOD_MS));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = odometry.next() => {
                let frame = required_frame(message, "terrain_map_ext.odometry")?;
                Module::on_odometry(&mut module, decode_terrain_odometry_body(&frame.payload)?)
                    .map_err(|error| module_error("terrain_map_ext.odometry", error))?;
                has_odometry = true;
                if let Some(cloud) = scan_before_odometry.take() {
                    Module::on_registered_scan(&mut module, cloud)
                        .map_err(|error| module_error("terrain_map_ext.registered_scan", error))?;
                }
            }
            message = scans.next() => {
                let frame = required_frame(message, "terrain_map_ext.registered_scan")?;
                let cloud = decode_cloud_body(&frame.payload)?;
                if cloud.frame_id != NavFrameId::Map {
                    return Err(NavStackError::InvalidConfig(format!(
                        "terrain_map_ext registered scan frame must be map, got {}",
                        cloud.frame_id.as_str()
                    )));
                }
                if has_odometry {
                    Module::on_registered_scan(&mut module, cloud)
                        .map_err(|error| module_error("terrain_map_ext.registered_scan", error))?;
                } else {
                    scan_before_odometry = Some(cloud);
                }
            }
            _ = ticker.tick() => {
                match Module::tick(&mut module)
                    .map_err(|error| module_error("terrain_map_ext.tick", error))?
                {
                    ModuleOutput::TerrainMap(cloud) => {
                        if !cloud.is_empty() {
                            publish_point_cloud(&transport, &node.outputs[0], &cloud).await?;
                        }
                    }
                    ModuleOutput::Idle => {}
                }
            }
        }
    }

    Module::stop(&mut module).map_err(|error| module_error("terrain_map_ext.stop", error))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn decode_terrain_odometry_body(body: &[u8]) -> NavStackResult<nuwax_nav_types_messages::Odometry> {
    let odometry = schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Odometry decode: {error}")))?;
    Ok(nuwax_nav_types_messages::Odometry {
        frame_id: NavFrameId::from_static(&odometry.header.frame_id),
        stamp: timestamp(&odometry.header)?,
        pose: Pose3 {
            position: Point3::new(
                odometry.pose.position[0],
                odometry.pose.position[1],
                odometry.pose.position[2],
            ),
            orientation: Quaternion {
                x: odometry.pose.orientation_xyzw[0],
                y: odometry.pose.orientation_xyzw[1],
                z: odometry.pose.orientation_xyzw[2],
                w: odometry.pose.orientation_xyzw[3],
            },
        },
        child_frame_id: NavFrameId::from_static(&odometry.child_frame_id),
        linear: Point3::new(
            odometry.twist_linear[0],
            odometry.twist_linear[1],
            odometry.twist_linear[2],
        ),
        angular: Point3::new(
            odometry.twist_angular[0],
            odometry.twist_angular[1],
            odometry.twist_angular[2],
        ),
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn publish_point_cloud(
    transport: &LcmTransport,
    channel: &str,
    cloud: &PointCloud,
) -> NavStackResult<()> {
    let body = encode_point_cloud_body(cloud)?;
    publish_body(transport, channel, "sensor_msgs.PointCloud2", body).await
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn encode_point_cloud_body(cloud: &PointCloud) -> NavStackResult<Vec<u8>> {
    cloud
        .validate()
        .map_err(|reason| NavStackError::InvalidConfig(reason.into()))?;
    let width = i32::try_from(cloud.points.len())
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 width exceeds int32".into()))?;
    let row_step = width
        .checked_mul(28)
        .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 row_step overflow".into()))?;
    let mut data = Vec::with_capacity(row_step as usize);
    for (index, point) in cloud.points.iter().enumerate() {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(NavStackError::InvalidConfig(
                "PointCloud2 output contains a non-finite coordinate".into(),
            ));
        }
        data.extend_from_slice(&point.x.to_le_bytes());
        data.extend_from_slice(&point.y.to_le_bytes());
        data.extend_from_slice(&point.z.to_le_bytes());
        let intensity = cloud.intensities.get(index).copied().unwrap_or(0.0);
        if !intensity.is_finite() {
            return Err(NavStackError::InvalidConfig(
                "PointCloud2 output contains a non-finite intensity".into(),
            ));
        }
        data.extend_from_slice(&intensity.to_le_bytes());
    }
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema_header(cloud.stamp, cloud.frame_id.as_str())?,
        height: 1,
        width,
        fields: vec![
            schema::PointField {
                name: "x".into(),
                offset: 0,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "y".into(),
                offset: 8,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "z".into(),
                offset: 16,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "intensity".into(),
                offset: 24,
                datatype: 7,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step: 28,
        row_step,
        data,
        is_dense: true,
    })
    .map_err(|error| NavStackError::Internal(format!("PointCloud2 encode: {error}")))
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
const BOUNDARY_EPSILON: f64 = 1e-9;

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
#[derive(Debug, Clone, PartialEq)]
struct NavigationBoundary {
    vertices: Vec<Point2>,
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
impl NavigationBoundary {
    fn new(points: &[[f64; 3]]) -> NavStackResult<Self> {
        let mut vertices: Vec<Point2> = points
            .iter()
            .map(|point| Point2::new(point[0], point[1]))
            .collect();
        if vertices
            .first()
            .zip(vertices.last())
            .is_some_and(|(first, last)| first.distance(*last) <= BOUNDARY_EPSILON)
        {
            vertices.pop();
        }
        if vertices.len() < 3 {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary requires at least three distinct vertices".into(),
            ));
        }
        if vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .any(|(left, right)| left.distance(*right) <= BOUNDARY_EPSILON)
        {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary contains duplicate adjacent vertices".into(),
            ));
        }
        let signed_area_twice: f64 = vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .map(|(left, right)| left.x * right.y - right.x * left.y)
            .sum();
        if signed_area_twice.abs() <= BOUNDARY_EPSILON {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary polygon area must be non-zero".into(),
            ));
        }
        for left_index in 0..vertices.len() {
            let left_next = (left_index + 1) % vertices.len();
            for right_index in (left_index + 1)..vertices.len() {
                let right_next = (right_index + 1) % vertices.len();
                if left_index == right_index || left_next == right_index || right_next == left_index
                {
                    continue;
                }
                if segments_intersect(
                    vertices[left_index],
                    vertices[left_next],
                    vertices[right_index],
                    vertices[right_next],
                ) {
                    return Err(NavStackError::InvalidConfig(
                        "navigation boundary polygon must not self-intersect".into(),
                    ));
                }
            }
        }
        Ok(Self { vertices })
    }

    fn contains(&self, point: Point2) -> bool {
        if self
            .edges()
            .any(|(start, end)| point_on_segment(point, start, end))
        {
            return true;
        }
        let mut inside = false;
        for (start, end) in self.edges() {
            if (start.y > point.y) != (end.y > point.y)
                && point.x < (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x
            {
                inside = !inside;
            }
        }
        inside
    }

    fn segment_is_inside(&self, start: Point2, end: Point2, sample_step: f64) -> bool {
        if !self.contains(start) || !self.contains(end) {
            return false;
        }
        let mut parameters = vec![0.0, 1.0];
        for (edge_start, edge_end) in self.edges() {
            if let Some(parameter) =
                segment_intersection_parameter(start, end, edge_start, edge_end)
            {
                parameters.push(parameter.clamp(0.0, 1.0));
            }
        }
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|left, right| (*left - *right).abs() <= BOUNDARY_EPSILON);
        if parameters.windows(2).any(|window| {
            let midpoint = (window[0] + window[1]) * 0.5;
            !self.contains(start.lerp(end, midpoint))
        }) {
            return false;
        }

        let step = if sample_step.is_finite() && sample_step > 0.0 {
            sample_step
        } else {
            0.05
        };
        let samples = (start.distance(end) / step).ceil().max(1.0) as usize;
        (0..=samples).all(|index| self.contains(start.lerp(end, index as f64 / samples as f64)))
    }

    fn edges(&self) -> impl Iterator<Item = (Point2, Point2)> + '_ {
        self.vertices
            .iter()
            .copied()
            .zip(self.vertices.iter().copied().cycle().skip(1))
            .take(self.vertices.len())
    }
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn navigation_boundary_from_goal(
    goal: &schema::NavigationGoal,
) -> NavStackResult<Option<NavigationBoundary>> {
    if goal.boundary_points.is_empty() {
        return Ok(None);
    }
    let boundary = NavigationBoundary::new(&goal.boundary_points)?;
    let point = Point2::new(goal.goal.pose.position[0], goal.goal.pose.position[1]);
    if !boundary.contains(point) {
        return Err(NavStackError::InvalidConfig("探索目标不在边界内".into()));
    }
    Ok(Some(boundary))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar", feature = "cmu-native",
    feature = "explore", feature = "mls"))]
fn path_respects_navigation_boundary(
    path: &Path2D,
    boundary: Option<&NavigationBoundary>,
    sample_step: f64,
) -> bool {
    let Some(boundary) = boundary else {
        return true;
    };
    !path.points.is_empty()
        && path
            .points
            .iter()
            .copied()
            .all(|point| boundary.contains(point))
        && path
            .points
            .windows(2)
            .all(|segment| boundary.segment_is_inside(segment[0], segment[1], sample_step))
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    let cross = cross_product(start, end, point).abs();
    let scale = start.distance(end).max(1.0);
    if cross > BOUNDARY_EPSILON * scale {
        return false;
    }
    point.x >= start.x.min(end.x) - BOUNDARY_EPSILON
        && point.x <= start.x.max(end.x) + BOUNDARY_EPSILON
        && point.y >= start.y.min(end.y) - BOUNDARY_EPSILON
        && point.y <= start.y.max(end.y) + BOUNDARY_EPSILON
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn cross_product(start: Point2, end: Point2, point: Point2) -> f64 {
    (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn segments_intersect(
    left_start: Point2,
    left_end: Point2,
    right_start: Point2,
    right_end: Point2,
) -> bool {
    let left_a = cross_product(left_start, left_end, right_start);
    let left_b = cross_product(left_start, left_end, right_end);
    let right_a = cross_product(right_start, right_end, left_start);
    let right_b = cross_product(right_start, right_end, left_end);
    if ((left_a > BOUNDARY_EPSILON && left_b < -BOUNDARY_EPSILON)
        || (left_a < -BOUNDARY_EPSILON && left_b > BOUNDARY_EPSILON))
        && ((right_a > BOUNDARY_EPSILON && right_b < -BOUNDARY_EPSILON)
            || (right_a < -BOUNDARY_EPSILON && right_b > BOUNDARY_EPSILON))
    {
        return true;
    }
    (left_a.abs() <= BOUNDARY_EPSILON && point_on_segment(right_start, left_start, left_end))
        || (left_b.abs() <= BOUNDARY_EPSILON && point_on_segment(right_end, left_start, left_end))
        || (right_a.abs() <= BOUNDARY_EPSILON
            && point_on_segment(left_start, right_start, right_end))
        || (right_b.abs() <= BOUNDARY_EPSILON && point_on_segment(left_end, right_start, right_end))
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn segment_intersection_parameter(
    start: Point2,
    end: Point2,
    edge_start: Point2,
    edge_end: Point2,
) -> Option<f64> {
    let segment = Point2::new(end.x - start.x, end.y - start.y);
    let edge = Point2::new(edge_end.x - edge_start.x, edge_end.y - edge_start.y);
    let denominator = segment.x * edge.y - segment.y * edge.x;
    if denominator.abs() <= BOUNDARY_EPSILON {
        return None;
    }
    let offset = Point2::new(edge_start.x - start.x, edge_start.y - start.y);
    let parameter = (offset.x * edge.y - offset.y * edge.x) / denominator;
    let edge_parameter = (offset.x * segment.y - offset.y * segment.x) / denominator;
    let segment_range = -BOUNDARY_EPSILON..=1.0 + BOUNDARY_EPSILON;
    (segment_range.contains(&parameter) && segment_range.contains(&edge_parameter))
        .then_some(parameter)
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn decode_planner_pose_input(payload: &[u8], component: &str) -> OwnerInputResult<PoseStamped> {
    let pose_message =
        match schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, payload)) {
            Ok(message) => message,
            Err(error) => {
                return Ok(OwnerInput::Rejected(format!(
                    "{component} PoseStamped decode: {error}"
                )));
            }
        };
    if pose_message.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "{component} frame must be map, got {}",
            pose_message.header.frame_id,
        )));
    }
    if let Err(error) = lcm_header_timestamp_ns(&pose_message.header, component) {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    if !schema_pose_is_finite_and_normalized(&pose_message.pose) {
        return Ok(OwnerInput::Rejected(format!(
            "{component} contains non-finite values or a zero-norm quaternion"
        )));
    }
    match pose_from_schema(&pose_message) {
        Ok(pose) => Ok(OwnerInput::Accepted(pose)),
        Err(error) => Ok(OwnerInput::Rejected(error.to_string())),
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn decode_planner_grid_input(
    payload: &[u8],
    component: &str,
) -> OwnerInputResult<schema::OccupancyGrid> {
    let grid = match schema::decode_occupancy_grid_body(payload) {
        Ok(grid) => grid,
        Err(error) => {
            return Ok(OwnerInput::Rejected(format!(
                "{component} OccupancyGrid decode: {error}"
            )));
        }
    };
    if grid.header.frame_id != "map" {
        return Ok(OwnerInput::Rejected(format!(
            "{component} requires map frame, got frame={}",
            grid.header.frame_id,
        )));
    }
    if !grid.info.resolution.is_finite()
        || grid
            .info
            .origin
            .position
            .iter()
            .chain(grid.info.origin.orientation_xyzw.iter())
            .any(|value| !value.is_finite())
    {
        return Ok(OwnerInput::Rejected(format!(
            "{component} metadata contains a non-finite value"
        )));
    }
    if let Err(error) = lcm_header_timestamp_ns(&grid.header, component) {
        return Ok(OwnerInput::Rejected(error.to_string()));
    }
    Ok(OwnerInput::Accepted(grid))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn decode_planner_stop_input(payload: &[u8], component: &str) -> OwnerInputResult<bool> {
    match schema::decode_bool_body(payload) {
        Ok(stop) => Ok(OwnerInput::Accepted(stop.data)),
        Err(error) => Ok(OwnerInput::Rejected(format!(
            "{component} Bool decode: {error}"
        ))),
    }
}



#[cfg(feature = "simple-2d")]
fn apply_simple_planner_goal(
    module: &mut nuwax_nav_simple_planner::module::SimplePlannerModule,
    envelope: schema::NavigationGoal,
    occupancy_body: &[u8],
) -> OwnerInputResult<(schema::NavigationGoal, Option<NavigationBoundary>)> {
    use nuwax_nav_simple_planner::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let pose = match pose_from_schema(&envelope.goal) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let occupancy = match boundary.as_ref() {
        Some(boundary) => {
            match decode_simple_occupancy_with_boundary(occupancy_body, Some(boundary)) {
                Ok(occupancy) => occupancy,
                Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
            }
        }
        None => match decode_simple_occupancy(occupancy_body) {
            Ok(occupancy) => occupancy,
            Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
        },
    };
    module
        .on_occupancy_grid(occupancy)
        .map_err(|error| module_error("simple_planner.goal_boundary", error))?;
    if let Err(error) = Module::on_goal(
        module,
        PointStamped::new(pose.frame_id, pose.stamp, pose.pose.position),
    ) {
        return module_input_error("simple_planner.goal_pose", error);
    }
    module.set_goal_yaw(pose.pose.orientation.yaw());
    // 目标身份保留原信封，自主导航只收窄其允许的运动方向。
    module.set_goal_options(nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..envelope.options()})
        .map_err(|error|module_error("simple_planner.goal_options",error))?;
    Ok(OwnerInput::Accepted((envelope, boundary)))
}

#[cfg(feature = "replanning-astar")]
fn apply_astar_planner_goal(
    module: &mut nuwax_nav_astar::module::ReplanningAStarPlannerModule,
    envelope: schema::NavigationGoal,
    costmap_body: &[u8],
) -> OwnerInputResult<(schema::NavigationGoal, Option<NavigationBoundary>, f64)> {
    use nuwax_nav_astar::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let goal_pose = match pose_from_schema(&envelope.goal) {
        Ok(pose) => pose,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let costmap = match decode_astar_costmap_with_boundary(costmap_body, boundary.as_ref()) {
        Ok(costmap) => costmap,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let map_resolution = costmap.resolution();
    Module::on_costmap(module, costmap)
        .map_err(|error| module_error("astar_planner.goal_boundary", error))?;
    if let Err(error) = Module::on_goal(module, goal_pose) {
        return module_input_error("astar_planner.goal", error);
    }
    Ok(OwnerInput::Accepted((envelope, boundary, map_resolution)))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(crate) fn invalidate_native_planner_generation(node_id: &str) {
    if node_id == "far_planner" { FAR_READY_GENERATION.store(0, Ordering::Release); }
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn native_ready_generation() -> i64 { FAR_READY_GENERATION.load(Ordering::Acquire) }

#[cfg(any(feature = "cmu-native", feature = "explore"))]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct FarJoinKey {
    backend: String,
    runtime_generation: i64,
    session_id: String,
    motion_generation: u64,
    path_revision: i64,
    planner_runtime_generation: i64,
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn far_join_key(state: &schema::NavigationStatus) -> Option<FarJoinKey> {
    if state.state != nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED
        || state.runtime_generation <= 0
        || state.path_revision <= 0
        || state.planner_runtime_generation != state.runtime_generation
    {
        return None;
    }
    Some(FarJoinKey {
        backend: state.robot_frame.clone(),
        runtime_generation: state.runtime_generation,
        session_id: state.session_id.clone(),
        motion_generation: state.motion_generation as u64,
        path_revision: state.path_revision,
        planner_runtime_generation: state.planner_runtime_generation,
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn far_key_matches_goal(
    key: &FarJoinKey,
    goal: &schema::NavigationGoal,
    accepted_generation: i64,
) -> bool {
    accepted_generation > 0
        && key.backend == "far-visibility-graph"
        && key.runtime_generation == accepted_generation
        && key.session_id == goal.session_id
        && key.motion_generation == goal.motion_generation
        && key.planner_runtime_generation == accepted_generation
}

#[cfg(all(any(feature = "cmu-native", feature = "explore"), test))]
fn far_terminal_matches_goal(
    state: &schema::NavigationStatus,
    goal: &schema::NavigationGoal,
    accepted_generation: i64,
) -> bool {
    far_join_key(state)
        .as_ref()
        .is_some_and(|key| far_key_matches_goal(key, goal, accepted_generation))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn take_far_join(
    key: &FarJoinKey,
    pending_results: &mut HashMap<FarJoinKey, Path2D>,
    pending_terminals: &mut HashSet<FarJoinKey>,
    consumed: &mut HashSet<FarJoinKey>,
) -> Option<Path2D> {
    if consumed.contains(key)
        || !pending_results.contains_key(key)
        || !pending_terminals.contains(key)
    {
        return None;
    }
    let path = pending_results.remove(key)?;
    pending_terminals.remove(key);
    consumed.insert(key.clone());
    Some(path)
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn native_path_respects_goal(path: &Path2D, goal: &schema::NavigationGoal) -> bool {
    path.frame_id.as_str() == goal.goal.header.frame_id
        && navigation_boundary_from_goal(goal)
            .ok()
            .is_some_and(|boundary| {
                path_respects_navigation_boundary(path, boundary.as_ref(), 0.025)
            })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn admit_native_adapter_goal(
    latest_goal: &mut Option<schema::NavigationGoal>,
    candidate: schema::NavigationGoal,
) -> bool {
    let Some(current) = latest_goal.as_ref() else {
        *latest_goal = Some(candidate);
        return true;
    };
    if candidate == *current { return false; }
    *latest_goal = Some(candidate);
    true
}



#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_native_planner_path_adapter(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_native_planner_path_adapter_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_native_planner_path_adapter_with_transport(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let mut goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.NavigationGoal").await?;
    let mut events =
        typed_stream(&transport, &node.inputs[1], "nuwax_msgs.NavigationStatus").await?;
    let mut far_results = if node.inputs.len() == 3 {
        Some(typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationStatus").await?)
    } else {
        None
    };
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut pending_results: HashMap<FarJoinKey, Path2D> = HashMap::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut pending_terminals: HashSet<FarJoinKey> = HashSet::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut consumed_far_results: HashSet<FarJoinKey> = HashSet::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut observed_far_generation = 0_i64;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            frame = goals.next() => {
                let frame = required_frame(frame, "native_planner_path_adapter.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(goal) => goal,
                    Err(_) => continue,
                };
                if !admit_native_adapter_goal(&mut latest_goal, goal) {
                    continue;
                }
                #[cfg(any(feature = "cmu-native", feature = "explore"))]
                {
                    pending_results.clear();
                    pending_terminals.clear();
                    consumed_far_results.clear();
                }
            }
            _frame = async {
                match far_results.as_mut() {
                    Some(results) => results.next().await,
                    None => futures::future::pending().await,
                }
            }, if far_results.is_some() => {
                #[cfg(any(feature = "cmu-native", feature = "explore"))]
                {
                    let frame = required_frame(_frame, "native_planner_path_adapter.far_result")?;
                    let accepted_generation = native_ready_generation();
                    if accepted_generation != observed_far_generation {
                        pending_results.clear();
                        pending_terminals.clear();
                        consumed_far_results.clear();
                        observed_far_generation = accepted_generation;
                    }
                    let state = match schema::decode_navigation_status_body(&frame.payload) {
                        Ok(state) => state,
                        Err(_) => continue,
                    };
                    let Some(goal) = latest_goal.as_ref() else { continue; };
                    let Some(key) = far_join_key(&state) else { continue; };
                    if !far_key_matches_goal(&key, goal, accepted_generation)
                        || state.planner_map_frame != goal.goal.header.frame_id
                        || state.planner_pose_frame != goal.goal.header.frame_id
                        || consumed_far_results.contains(&key)
                    {
                        continue;
                    }
                    let path = match nuwax_nav_far_planner_bridge::worker::decode_private_result_v3(&state) {
                        Ok(path) => path,
                        Err(_) => continue,
                    };
                    if path.header != goal.goal.header {
                        continue;
                    }
                    let path = match path_from_schema(&path) {
                        Ok(path) => path,
                        Err(_) => continue,
                    };
                    if pending_results.len() >= 64 && !pending_results.contains_key(&key) {
                        continue;
                    }
                    pending_results.insert(key.clone(), path);
                    let Some(path) = take_far_join(
                        &key,
                        &mut pending_results,
                        &mut pending_terminals,
                        &mut consumed_far_results,
                    ) else { continue; };
                    let _ = publish_versioned_path(
                        &transport,
                        &node.outputs[0],
                        VersionedPathPublication {
                    path_yaws: None,                            goal,
                            planner_runtime_generation: accepted_generation,
                            path_revision: key.path_revision,
                            path: &path,
                            boundary_safe: native_path_respects_goal(&path, goal),
                        },
                    ).await?;
                }
            }
            frame = events.next() => {
                let frame = required_frame(frame, "native_planner_path_adapter.event")?;
                if far_results.is_some() {
                    #[cfg(any(feature = "cmu-native", feature = "explore"))]
                    {
                        let state = match schema::decode_navigation_status_body(&frame.payload) {
                            Ok(state) => state,
                            Err(_) => continue,
                        };
                        let accepted_generation = native_ready_generation();
                        if accepted_generation != observed_far_generation {
                            pending_results.clear();
                            pending_terminals.clear();
                            consumed_far_results.clear();
                            observed_far_generation = accepted_generation;
                        }
                        if state.state == nuwax_nav_far_planner_bridge::worker::FAR_EVENT_READY {
                            continue;
                        }
                        let Some(goal) = latest_goal.as_ref() else { continue; };
                        let Some(key) = far_join_key(&state) else { continue; };
                        if !far_key_matches_goal(&key, goal, accepted_generation)
                            || state.planner_map_frame != goal.goal.header.frame_id
                            || state.planner_pose_frame != goal.goal.header.frame_id
                            || consumed_far_results.contains(&key)
                        {
                            continue;
                        }
                        if pending_terminals.len() >= 64 && !pending_terminals.contains(&key) {
                            continue;
                        }
                        pending_terminals.insert(key.clone());
                        let Some(path) = take_far_join(
                            &key,
                            &mut pending_results,
                            &mut pending_terminals,
                            &mut consumed_far_results,
                        ) else { continue; };
                        let _ = publish_versioned_path(
                            &transport,
                            &node.outputs[0],
                            VersionedPathPublication {
                    path_yaws: None,                                goal,
                                planner_runtime_generation: accepted_generation,
                                path_revision: key.path_revision,
                                path: &path,
                                boundary_safe: native_path_respects_goal(&path, goal),
                            },
                        ).await?;
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(feature = "simple-2d")]
pub(crate) fn simple_planner_owner(node: &TaskChannels, body_reference_frame_id: Arc<str>) -> NavStackResult<TaskThread> {
    let node = node.clone();
    spawn_task("simple-planner", Box::new(move |shutdown, ready| {
        Box::pin(run_simple_planner(node, body_reference_frame_id, shutdown, ready))
    }))
}

#[cfg(feature = "simple-2d")]
async fn run_simple_planner(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::{Module, SimplePlannerModule};

    let transport = lcm_transport()?;
    let mut occupancy = typed_stream(&transport, &node.inputs[0], "nav_msgs.OccupancyGrid").await?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let mut goal_pose =
        typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationGoal").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let robot_geometry = Some(
        nuwax_config::derived()
            .map_err(NavStackError::InvalidConfig)?
            .robot_geometry_json
            .clone(),
    )
    .filter(|value| !value.trim().is_empty())
    .map(|value| nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value))
    .transpose()
    .map_err(NavStackError::InvalidConfig)?;
    let motion_limits = nuwax_nav_types_messages::NavigationCapabilities::from_runtime()
        .map_err(NavStackError::InvalidConfig)?
        .map(|profile| {
            let mut limits = profile.motion;
            let navigation = nuwax_nav_runtime::surface_follower::forward_navigation_kinematics(
                limits.kinematics(),
            );
            limits.model = navigation.model;
            limits.max_reverse_mps = navigation.max_reverse_mps;
            limits.max_lateral_mps = navigation.max_lateral_mps;
            limits
        });
    let mut module =
        SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
            robot_geometry,
            motion_limits,
            ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
        })
        .map_err(|error| module_error("simple_planner.robot_geometry", error))?;
    module.enable_background_planning();
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut latest_occupancy_body: Option<Vec<u8>> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05;
    let mut published_path_is_boundary_safe = false;
    let mut path_revision = 0;
    let mut diagnostics = InputDiagnosticLimiter::default();
    Module::start(&mut module).map_err(|e| module_error("simple_planner", e))?;
    let mut ticker = tokio::time::interval(Duration::from_millis(
        nuwax_nav_simple_planner::module::TICK_PERIOD_MS,
    ));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;
    loop {
        tokio::select! {
            _=&mut shutdown=>break,
            message = occupancy.next() => {
                let frame = required_frame(message, "simple_planner.occupancy")?;
                let grid = match decode_planner_grid_input(&frame.payload, "simple_planner.occupancy")? {
                    OwnerInput::Accepted(v) => v,
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("simple_planner.occupancy", &e);
                        continue;
                    }
                };
                let first = latest_occupancy_body.is_none();
                latest_occupancy_body = Some(frame.payload.to_vec());
                active_map_resolution = f64::from(grid.info.resolution);
                published_path_is_boundary_safe = false;
                if first && latest_goal.is_some() {
                    match apply_simple_planner_goal(&mut module, latest_goal.clone().unwrap(), &frame.payload)?
                    {
                        OwnerInput::Accepted((_, boundary)) => {
                            active_boundary = boundary;
                            path_revision = 0;
                        }
                        OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.goal", &e),
                    }
                } else {
                    let occupancy =
                        decode_simple_occupancy_with_boundary(&frame.payload, active_boundary.as_ref())?;
                    module
                        .on_occupancy_grid(occupancy)
                        .map_err(|e| module_error("simple_planner.occupancy", e))?;
                }
            },
            message = map_pose.next() => {
                let frame = latest_pose_frame(message, &mut map_pose, "simple_planner.map_pose")?;
                match decode_planner_pose_input(&frame.payload, "simple_planner.map_pose")? {
                    OwnerInput::Accepted(pose) => Module::on_odometry(&mut module, pose)
                        .map_err(|e| module_error("simple_planner.map_pose", e))?,
                    OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.map_pose", &e),
                }
            },
            message = goal_pose.next() => {
                let frame = required_frame(message, "simple_planner.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(v) => v,
                    Err(e) => {
                        diagnostics.reject("simple_planner.goal", &e);
                        continue;
                    }
                };
                if latest_goal.as_ref() == Some(&goal) {
                    continue;
                }
                Module::on_stop_movement(&mut module, true)
                    .map_err(|e| module_error("simple_planner.replace", e))?;
                latest_goal = Some(goal.clone());
                active_boundary = None;
                path_revision = 0;
                published_path_is_boundary_safe = false;
                if let Some(body) = &latest_occupancy_body {
                    match apply_simple_planner_goal(&mut module, goal, body)? {
                        OwnerInput::Accepted((_, boundary)) => active_boundary = boundary,
                        OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.goal", &e),
                    }
                }
            },
            message = stop.next() => {
                let frame = required_frame(message, "simple_planner.stop")?;
                match decode_planner_stop_input(&frame.payload, "simple_planner.stop")? {
                    OwnerInput::Accepted(value) => {
                        Module::on_stop_movement(&mut module, value)
                            .map_err(|e| module_error("simple_planner.stop", e))?;
                        if value {
                            latest_goal = None;
                            active_boundary = None;
                            path_revision = 0;
                            published_path_is_boundary_safe = false;
                        }
                    }
                    OwnerInput::Rejected(e) => diagnostics.reject("simple_planner.stop", &e),
                }
            },
            _ = ticker.tick(), if latest_goal.is_some()&&latest_occupancy_body.is_some() => {
                let output =
                    Module::tick(&mut module).map_err(|e| module_tick_error("simple_planner.tick", e))?;
                let mut publication = SimpleOutputPublication {
                    path_yaws: module.planned_yaws(),
                    active_goal: latest_goal.as_ref(),
                    active_boundary: active_boundary.as_ref(),
                    map_resolution: active_map_resolution,
                    planner_runtime_generation,
                    path_revision: &mut path_revision,
                    published_path_is_boundary_safe: &mut published_path_is_boundary_safe,
                };
                publish_simple_output(&transport, &node, output, &mut publication).await?;
            }
        }
    }
    Module::stop(&mut module).map_err(|e| module_error("simple_planner.stop", e))?;
    Ok(())
}

#[cfg(feature = "simple-2d")]
struct SimpleOutputPublication<'a> {
    path_yaws: Option<&'a [f64]>,
    active_goal: Option<&'a schema::NavigationGoal>,
    active_boundary: Option<&'a NavigationBoundary>,
    map_resolution: f64,
    planner_runtime_generation: i64,
    path_revision: &'a mut i64,
    published_path_is_boundary_safe: &'a mut bool,
}

#[cfg(feature = "simple-2d")]
fn simple_hold_path_is_executable(reason: nuwax_nav_simple_planner::module::HoldReason) -> bool {
    reason != nuwax_nav_simple_planner::module::HoldReason::PlanningFailed
}

#[cfg(feature = "simple-2d")]
async fn publish_simple_output(
    transport: &LcmTransport,
    node: &TaskChannels,
    output: nuwax_nav_simple_planner::module::ModuleOutput,
    publication: &mut SimpleOutputPublication<'_>,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::ModuleOutput;

    match output {
        ModuleOutput::Plan { path, waypoint } => {
            let next_revision = publication.path_revision.checked_add(1).ok_or_else(|| {
                NavStackError::Internal("simple planner path revision exhausted".into())
            })?;
            *publication.published_path_is_boundary_safe = publish_versioned_path(
                transport,
                &node.outputs[1],
                VersionedPathPublication {
                    path_yaws: publication.path_yaws,                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner path has no active NavigationGoal".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary_safe: path_respects_navigation_boundary(
                        &path,
                        publication.active_boundary,
                        publication.map_resolution * 0.5,
                    ),
                },
            )
            .await?;
            if *publication.published_path_is_boundary_safe {
                *publication.path_revision = next_revision;
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::Waypoint(waypoint) => {
            let point_is_safe = publication
                .active_boundary
                .is_none_or(|boundary| boundary.contains(waypoint.point.to_2d()));
            if *publication.published_path_is_boundary_safe && point_is_safe {
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::SafetyHold { reason, .. } if !simple_hold_path_is_executable(reason) => {
            *publication.published_path_is_boundary_safe = false;
            Ok(())
        }
        ModuleOutput::SafetyHold { waypoint, path, .. } => {
            let next_revision = publication.path_revision.checked_add(1).ok_or_else(|| {
                NavStackError::Internal("simple planner path revision exhausted".into())
            })?;
            *publication.published_path_is_boundary_safe = publish_versioned_path(
                transport,
                &node.outputs[1],
                VersionedPathPublication {
                    path_yaws: None,                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner hold has no active NavigationGoal".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary_safe: path_respects_navigation_boundary(
                        &path,
                        publication.active_boundary,
                        publication.map_resolution * 0.5,
                    ),
                },
            )
            .await?;
            if *publication.published_path_is_boundary_safe {
                *publication.path_revision = next_revision;
                publish_point(transport, &node.outputs[0], &waypoint).await?;
            }
            Ok(())
        }
        ModuleOutput::Idle => Ok(()),
    }
}



#[cfg(feature = "replanning-astar")]
async fn run_astar_planner(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_astar_planner_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(feature = "replanning-astar")]
async fn run_astar_planner_with_transport(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_astar::module::{
        Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
    };

    let mut map_pose =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationGoal").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let value = nuwax_config::derived()
        .map_err(NavStackError::InvalidConfig)?
        .robot_geometry_json
        .clone();
    let robot_geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value)
        .map_err(NavStackError::InvalidConfig)?;
    let config = nuwax_nav_astar::GlobalPlannerConfig::new(robot_geometry);
    let mut module = ReplanningAStarPlannerModule::new(config);
    NavigationInterface::start(&mut module)
        .map_err(|error| module_error("astar_planner", error))?;
    let mut has_odometry = false;
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    let mut latest_costmap_body: Option<Vec<u8>> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05;
    let mut path_revision = 0_i64;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(100));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;
    loop {
        tokio::select! {
            _=&mut shutdown=>break,
            message = map_pose.next() => {
                let frame = latest_pose_frame(message, &mut map_pose, "astar_planner.map_pose")?;
                let pose = match decode_planner_pose_input(&frame.payload, "astar_planner.map_pose")? {
                    OwnerInput::Accepted(v) => v,
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("astar_planner.map_pose", &e);
                        continue;
                    }
                };
                Module::on_odom(&mut module, pose).map_err(|e| module_error("astar_planner.map_pose", e))?;
                let first = !has_odometry;
                has_odometry = true;
                if first {
                    if let (Some(goal), Some(body)) = (latest_goal.clone(), latest_costmap_body.as_ref()) {
                        match apply_astar_planner_goal(&mut module, goal, body)? {
                            OwnerInput::Accepted((_, boundary, resolution)) => {
                                active_boundary = boundary;
                                active_map_resolution = resolution;
                                path_revision = 0;
                            }
                            OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                        }
                    }
                }
            },
            message = costmap.next() => {
                let frame = required_frame(message, "astar_planner.costmap")?;
                match decode_planner_grid_input(&frame.payload, "astar_planner.costmap")? {
                    OwnerInput::Accepted(_) => {}
                    OwnerInput::Rejected(e) => {
                        diagnostics.reject("astar_planner.costmap", &e);
                        continue;
                    }
                }
                let first = latest_costmap_body.is_none();
                latest_costmap_body = Some(frame.payload.to_vec());
                if first && has_odometry && latest_goal.is_some() {
                    let goal = latest_goal.clone().unwrap();
                    let body = &frame.payload;
                    match apply_astar_planner_goal(&mut module, goal, body)? {
                        OwnerInput::Accepted((_, boundary, resolution)) => {
                            active_boundary = boundary;
                            active_map_resolution = resolution;
                            path_revision = 0;
                        }
                        OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                    }
                } else {
                    let map = decode_astar_costmap_with_boundary(&frame.payload, active_boundary.as_ref())?;
                    active_map_resolution = map.resolution();
                    Module::on_costmap(&mut module, map)
                        .map_err(|e| module_error("astar_planner.costmap", e))?;
                }
            },
            message = goal.next() => {
                let frame = required_frame(message, "astar_planner.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(v) => v,
                    Err(e) => {
                        diagnostics.reject("astar_planner.goal", &e);
                        continue;
                    }
                };
                if latest_goal.as_ref() == Some(&goal) {
                    continue;
                }
                Module::on_stop_movement(&mut module, true)
                    .map_err(|e| module_error("astar_planner.replace", e))?;
                latest_goal = Some(goal.clone());
                active_boundary = None;
                path_revision = 0;
                if has_odometry {
                    if let Some(body) = latest_costmap_body.as_ref() {
                        match apply_astar_planner_goal(&mut module, goal, body)? {
                            OwnerInput::Accepted((_, boundary, resolution)) => {
                                active_boundary = boundary;
                                active_map_resolution = resolution;
                                path_revision = 0;
                            }
                            OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.goal", &e),
                        }
                    }
                }
            },
            message = stop.next() => {
                let frame = required_frame(message, "astar_planner.stop")?;
                match decode_planner_stop_input(&frame.payload, "astar_planner.stop")? {
                    OwnerInput::Accepted(value) => {
                        Module::on_stop_movement(&mut module, value)
                            .map_err(|e| module_error("astar_planner.stop", e))?;
                        if value {
                            latest_goal = None;
                            active_boundary = None;
                            path_revision = 0;
                        }
                    }
                    OwnerInput::Rejected(e) => diagnostics.reject("astar_planner.stop", &e),
                }
            },
            _ = ticker.tick(), if has_odometry&&latest_costmap_body.is_some()&&latest_goal.is_some() => {
                let output =
                    Module::tick(&mut module).map_err(|e| module_tick_error("astar_planner.tick", e))?;
                let update = match output {
                    ModuleOutput::Navigation(update) | ModuleOutput::SafetyHold(update) => update,
                };
                let terminal_failure = matches!(
                    update.event,
                    Some(
                        nuwax_nav_astar::global_planner::GlobalPlannerEvent::PlanningFailed
                            | nuwax_nav_astar::global_planner::GlobalPlannerEvent::Blocked
                    )
                );
                let mut boundary_failure = false;
                if let Some(path) = update.path {
                    let next_revision = path_revision
                        .checked_add(1)
                        .ok_or_else(|| NavStackError::Internal("A* planner path revision exhausted".into()))?;
                    let published = publish_versioned_path(
                        &transport,
                        &node.outputs[1],
                        VersionedPathPublication {
                            path_yaws: None,
                            goal: latest_goal.as_ref().ok_or_else(|| {
                                NavStackError::Internal("A* path has no active NavigationGoal".into())
                            })?,
                            planner_runtime_generation,
                            path_revision: next_revision,
                            path: &path,
                            boundary_safe: path_respects_navigation_boundary(
                                &path,
                                active_boundary.as_ref(),
                                active_map_resolution * 0.5,
                            ),
                        },
                    )
                    .await?;
                    if published {
                        path_revision = next_revision;
                    } else {
                        boundary_failure = true;
                    }
                }
                publish_bool(&transport, &node.outputs[2], update.goal_reached).await?;
                if terminal_failure || boundary_failure {
                    publish_bool(&transport, &node.outputs[3], true).await?;
                }
            }
        }
    }
    NavigationInterface::stop(&mut module).map_err(|e| module_error("astar_planner.stop", e))?;
    Ok(())
}



#[cfg(feature = "patrolling")]
async fn run_patrolling(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_patrolling::{
        create_patrol_router, Module, PatrolRouterName, PatrollingModule, PatrollingModuleConfig,
    };

    let transport = lcm_transport()?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal_reached = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let mut activation = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut goal_failed = typed_stream(&transport, &node.inputs[4], "std_msgs.Bool").await?;
    let mut planner_ack = typed_stream(&transport, &node.inputs[5], "std_msgs.Bool").await?;
    let mut module = PatrollingModule::new(
        create_patrol_router(PatrolRouterName::Coverage, 0.3, 32, Some(0)),
        PatrollingModuleConfig::default(),
    );
    let mut goals = module.subscribe_goals();
    let mut holds = module.subscribe_hold();
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = activation.next() => {
                let frame = required_frame(message, "patrolling.activation")?;
                let enabled = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling activation decode: {error}")))?
                    .data;
                if enabled {
                    if !Module::running(&module) {
                        request_patrol_planner_control(
                            &transport,
                            &node.outputs[3],
                            &mut planner_ack,
                            true,
                        ).await?;
                        Module::start(&mut module);
                        if !Module::running(&module) {
                            let _ = request_patrol_planner_control(
                                &transport,
                                &node.outputs[3],
                                &mut planner_ack,
                                false,
                            ).await;
                            return Err(NavStackError::Internal(
                                "patrolling lifecycle failed to start".into(),
                            ));
                        }
                    }
                } else {
                    let restore = request_patrol_planner_control(
                        &transport,
                        &node.outputs[3],
                        &mut planner_ack,
                        false,
                    ).await;
                    let stopped = stop_patrolling(
                        &transport,
                        &node,
                        &mut module,
                        &mut goals,
                        &mut holds,
                    ).await;
                    restore?;
                    stopped?;
                }
            }
            message = map_pose.next() => {
                let frame = required_frame(message, "patrolling.map_pose")?;
                let map_pose = decode_pose_body(&frame.payload)?;
                require_map_pose(&map_pose, "patrolling.map_pose")?;
                module.update_odometry(map_pose);
            }
            message = costmap.next() => {
                let frame = required_frame(message, "patrolling.global_costmap")?;
                module.update_grid(decode_patrol_grid(&frame.payload)?);
            }
            message = goal_reached.next() => {
                let frame = required_frame(message, "patrolling.goal_reached")?;
                let reached = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling goal_reached decode: {error}")))?;
                if reached.data {
                    Module::report_goal_reached(&mut module, true);
                }
            }
            message = goal_failed.next() => {
                let frame = required_frame(message, "patrolling.goal_failed")?;
                let failed = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrolling goal_failed decode: {error}")))?;
                if failed.data && Module::running(&module) {
                    Module::report_goal_failed(&mut module);
                    if !Module::running(&module) {
                        let restore = request_patrol_planner_control(
                            &transport,
                            &node.outputs[3],
                            &mut planner_ack,
                            false,
                        ).await;
                        let stopped = publish_patrolling_stop_outputs(
                            &transport,
                            &node,
                            &mut goals,
                            &mut holds,
                        ).await;
                        restore?;
                        stopped?;
                    }
                }
            }
            goal = goals.recv() => {
                let goal = goal.map_err(|error| NavStackError::Internal(format!(
                    "patrolling goal port: {error}"
                )))?;
                publish_pose(&transport, &node.outputs[0], &goal).await?;
            }
        }
    }

    let _ = publish_bool(&transport, &node.outputs[3], false).await;
    stop_patrolling(&transport, &node, &mut module, &mut goals, &mut holds).await
}

#[cfg(feature = "patrolling")]
async fn stop_patrolling(
    transport: &LcmTransport,
    node: &TaskChannels,
    module: &mut nuwax_nav_patrolling::PatrollingModule,
    goals: &mut tokio::sync::broadcast::Receiver<PoseStamped>,
    holds: &mut tokio::sync::broadcast::Receiver<Twist2>,
) -> NavStackResult<()> {
    use nuwax_nav_patrolling::Module;

    let was_running = Module::running(module);
    Module::stop(module);
    if !was_running {
        return Ok(());
    }

    publish_patrolling_stop_outputs(transport, node, goals, holds).await
}

#[cfg(feature = "patrolling")]
async fn publish_patrolling_stop_outputs(
    transport: &LcmTransport,
    node: &TaskChannels,
    goals: &mut tokio::sync::broadcast::Receiver<PoseStamped>,
    holds: &mut tokio::sync::broadcast::Receiver<Twist2>,
) -> NavStackResult<()> {
    let hold = latest_broadcast(holds).unwrap_or(Twist2::ZERO);
    publish_twist2(transport, &node.outputs[1], hold).await?;
    publish_bool(transport, &node.outputs[2], true).await?;
    if let Some(goal) = latest_broadcast(goals) {
        publish_pose(transport, &node.outputs[0], &goal).await?;
    }
    Ok(())
}

#[cfg(feature = "patrolling")]
async fn request_patrol_planner_control(
    transport: &LcmTransport,
    request_channel: &str,
    acknowledgements: &mut LcmFrameStream,
    active: bool,
) -> NavStackResult<()> {
    publish_bool(transport, request_channel, active).await?;
    let result = match tokio::time::timeout(PLANNER_CONTROL_TIMEOUT, async {
        loop {
            let frame = required_frame(
                acknowledgements.next().await,
                "patrolling.planner_control_ack",
            )?;
            let acknowledged = schema::decode_bool_body(&frame.payload).map_err(|error| {
                NavStackError::Internal(format!("patrolling planner control ack decode: {error}"))
            })?;
            if acknowledged.data == active {
                return Ok(());
            }
        }
    })
    .await
    {
        Ok(result) => result,
        Err(_) => Err(NavStackError::Internal(format!(
            "patrolling planner control ack timed out for active={active}"
        ))),
    };
    if result.is_err() && active {
        let _ = publish_bool(transport, request_channel, false).await;
    }
    result
}

#[cfg(feature = "patrolling")]
fn latest_broadcast<T>(receiver: &mut tokio::sync::broadcast::Receiver<T>) -> Option<T>
where
    T: Clone,
{
    let mut latest = None;
    loop {
        match receiver.try_recv() {
            Ok(value) => latest = Some(value),
            Err(tokio::sync::broadcast::error::TryRecvError::Lagged(_)) => continue,
            Err(
                tokio::sync::broadcast::error::TryRecvError::Empty
                | tokio::sync::broadcast::error::TryRecvError::Closed,
            ) => return latest,
        }
    }
}

#[cfg(feature = "patrolling")]
fn decode_patrol_grid(body: &[u8]) -> NavStackResult<GridSnapshot> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "patrolling OccupancyGrid frame must be map, got {}",
            grid.header.frame_id
        )));
    }
    let orientation = grid.info.origin.orientation_xyzw;
    let identity_rotation = orientation[0].abs() <= 1e-9
        && orientation[1].abs() <= 1e-9
        && orientation[2].abs() <= 1e-9
        && (orientation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity_rotation || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "patrolling grid cannot represent a rotated or non-planar origin".into(),
        ));
    }
    let width = u32::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = u32::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let snapshot = GridSnapshot {
        width,
        height,
        resolution: f64::from(grid.info.resolution),
        origin_x: grid.info.origin.position[0],
        origin_y: grid.info.origin.position[1],
        data: grid.data,
    };
    snapshot
        .validate()
        .map_err(|reason| NavStackError::InvalidConfig(reason.into()))?;
    if !(snapshot.resolution.is_finite()
        && snapshot.origin_x.is_finite()
        && snapshot.origin_y.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "patrolling grid metadata must be finite".into(),
        ));
    }
    Ok(snapshot)
}



#[cfg(feature = "basic-path-follower")]
async fn run_basic_path_follower(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_basic_path_follower::module::{
        BasicPathFollowerModule, BasicPathFollowerModuleConfig, Module,
    };

    let transport = lcm_transport()?;
    let mut path = typed_stream(&transport, &node.inputs[0], "nav_msgs.Path").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut stop = typed_stream(&transport, &node.inputs[2], "std_msgs.Bool").await?;
    let config = BasicPathFollowerModuleConfig {
        cmd_vel_topic: node.outputs[0].clone(),
        path_topic: node.inputs[0].clone(),
        odom_topic: node.inputs[1].clone(),
        stop_topic: node.inputs[2].clone(),
        ..BasicPathFollowerModuleConfig::default()
    };
    let mut module = BasicPathFollowerModule::new(config)
        .map_err(|error| module_error("basic_path_follower", error))?;
    let stop_input = module.stop_input();
    let mut commands = module.subscribe_cmd_vel();
    let mut goal_reached = module.subscribe_goal_reached();
    Module::start(&mut module).map_err(|error| module_error("basic_path_follower", error))?;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = path.next() => {
                let frame = required_frame(message, "basic_path_follower.path")?;
                Module::on_path(&mut module, decode_path_body(&frame.payload)?);
            }
            message = odometry.next() => {
                let frame = required_frame(message, "basic_path_follower.odometry")?;
                Module::on_odom(&mut module, decode_odometry_pose_body(&frame.payload)?);
            }
            message = stop.next() => {
                let frame = required_frame(message, "basic_path_follower.stop_movement")?;
                let stop = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("stop_movement decode: {error}")))?;
                stop_input.send(stop.data)
                    .map_err(|_| NavStackError::Internal("basic_path_follower stop port closed".into()))?;
            }
            command = commands.recv() => {
                let command = command.ok_or_else(|| NavStackError::Internal(
                    "basic_path_follower command port closed".into()
                ))?;
                publish_twist2(&transport, &node.outputs[0], command).await?;
            }
            reached = goal_reached.recv() => {
                let reached = reached.map_err(|error| NavStackError::Internal(format!(
                    "basic_path_follower goal port: {error}"
                )))?;
                if node.outputs.len() == 2 {
                    publish_bool(&transport, &node.outputs[1], reached).await?;
                }
            }
        }
    }
    Module::stop(&mut module).map_err(|error| module_error("basic_path_follower.stop", error))?;
    publish_twist2(&transport, &node.outputs[0], Twist2::ZERO).await
}

fn module_error(component: &str, error: impl std::fmt::Display) -> NavStackError {
    NavStackError::Internal(format!("{component}: {error}"))
}

#[cfg(feature = "patrolling")]
fn decode_pose_body(body: &[u8]) -> NavStackResult<PoseStamped> {
    let pose = schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PoseStamped decode: {error}")))?;
    pose_from_schema(&pose)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime"
))]
fn pose_from_schema(pose: &schema::PoseStamped) -> NavStackResult<PoseStamped> {
    Ok(PoseStamped {
        frame_id: NavFrameId::from_static(&pose.header.frame_id),
        stamp: timestamp(&pose.header)?,
        pose: Pose3 {
            position: Point3::new(
                pose.pose.position[0],
                pose.pose.position[1],
                pose.pose.position[2],
            ),
            orientation: Quaternion {
                x: pose.pose.orientation_xyzw[0],
                y: pose.pose.orientation_xyzw[1],
                z: pose.pose.orientation_xyzw[2],
                w: pose.pose.orientation_xyzw[3],
            },
        },
    })
}

#[cfg(any(feature = "basic-path-follower", feature = "explore"))]
fn decode_odometry_pose_body(body: &[u8]) -> NavStackResult<PoseStamped> {
    let odometry = schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Odometry decode: {error}")))?;
    Ok(PoseStamped {
        frame_id: NavFrameId::from_static(&odometry.header.frame_id),
        stamp: timestamp(&odometry.header)?,
        pose: Pose3 {
            position: Point3::new(
                odometry.pose.position[0],
                odometry.pose.position[1],
                odometry.pose.position[2],
            ),
            orientation: Quaternion {
                x: odometry.pose.orientation_xyzw[0],
                y: odometry.pose.orientation_xyzw[1],
                z: odometry.pose.orientation_xyzw[2],
                w: odometry.pose.orientation_xyzw[3],
            },
        },
    })
}

#[cfg(any(
    feature = "basic-path-follower",
    all(test, feature = "replanning-astar")
))]
fn decode_path_body(body: &[u8]) -> NavStackResult<Path2D> {
    let path = schema::decode_path(&prefixed(schema::PATH_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Path decode: {error}")))?;
    path_from_schema(&path)
}

#[cfg(feature = "guard-runtime")]
fn execution_path_from_schema(
    path: &schema::Path,
    surface: bool,
) -> NavStackResult<nuwax_nav_runtime::ExecutionPath> {
    if path.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "scheduler path frame must be map, got {}",
            path.header.frame_id
        )));
    }
    if surface {
        if path
            .poses
            .iter()
            .any(|pose| pose.header.frame_id != path.header.frame_id)
        {
            return Err(NavStackError::InvalidConfig(
                "Path pose frame does not match path frame".into(),
            ));
        }
        Ok(nuwax_nav_runtime::ExecutionPath::Surface(
            nuwax_nav_types_messages::Path3D {
                poses: path
                    .poses
                    .iter()
                    .map(pose_from_schema)
                    .collect::<NavStackResult<Vec<_>>>()?,
            },
        ))
    } else {
        path_from_schema(path).map(nuwax_nav_runtime::ExecutionPath::Planar)
    }
}

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore"
))]
fn path_from_schema(path: &schema::Path) -> NavStackResult<Path2D> {
    if path.poses.iter().any(|pose| {
        !pose.header.frame_id.is_empty() && pose.header.frame_id != path.header.frame_id
    }) {
        return Err(NavStackError::InvalidConfig(
            "Path pose frame does not match the path header frame".into(),
        ));
    }
    Ok(Path2D::stamped(
        NavFrameId::from_static(&path.header.frame_id),
        timestamp(&path.header)?,
        path.poses
            .iter()
            .map(|pose| Point2::new(pose.pose.position[0], pose.pose.position[1]))
            .collect(),
    ))
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
fn decode_cloud_body(body: &[u8]) -> NavStackResult<PointCloud> {
    let cloud = schema::decode_point_cloud2(&prefixed(schema::POINT_CLOUD2_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PointCloud2 decode: {error}")))?;
    point_cloud_from_schema(cloud)
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
fn point_cloud_from_schema(cloud: schema::PointCloud2) -> NavStackResult<PointCloud> {
    let width = usize::try_from(cloud.width)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 width is negative".into()))?;
    let height = usize::try_from(cloud.height)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 height is negative".into()))?;
    let point_step = usize::try_from(cloud.point_step)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 point_step is negative".into()))?;
    let row_step = usize::try_from(cloud.row_step)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 row_step is negative".into()))?;
    if width > 0 && height > 0 && (point_step == 0 || row_step < width * point_step) {
        return Err(NavStackError::InvalidConfig(
            "PointCloud2 stride metadata is inconsistent".into(),
        ));
    }
    let needed = height
        .checked_sub(1)
        .and_then(|rows| rows.checked_mul(row_step))
        .and_then(|bytes| bytes.checked_add(width.checked_mul(point_step)?))
        .unwrap_or(0);
    if cloud.data.len() < needed {
        return Err(NavStackError::InvalidConfig(format!(
            "PointCloud2 data has {} bytes, needs {needed}",
            cloud.data.len()
        )));
    }
    let x = cloud_field(&cloud.fields, "x")?;
    let y = cloud_field(&cloud.fields, "y")?;
    let z = cloud_field(&cloud.fields, "z")?;
    let mut points = Vec::with_capacity(width.saturating_mul(height));
    for row in 0..height {
        for column in 0..width {
            let base = row * row_step + column * point_step;
            points.push(Point3::new(
                read_cloud_scalar(&cloud.data, base, x, cloud.is_bigendian)?,
                read_cloud_scalar(&cloud.data, base, y, cloud.is_bigendian)?,
                read_cloud_scalar(&cloud.data, base, z, cloud.is_bigendian)?,
            ));
        }
    }
    Ok(PointCloud {
        stamp: timestamp(&cloud.header)?,
        frame_id: NavFrameId::from_static(&cloud.header.frame_id),
        points,
        intensities: Vec::new(),
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
fn cloud_field<'a>(
    fields: &'a [schema::PointField],
    name: &str,
) -> NavStackResult<&'a schema::PointField> {
    fields
        .iter()
        .find(|field| field.name == name && field.count >= 1)
        .ok_or_else(|| NavStackError::InvalidConfig(format!("PointCloud2 missing {name} field")))
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
fn read_cloud_scalar(
    data: &[u8],
    base: usize,
    field: &schema::PointField,
    big_endian: bool,
) -> NavStackResult<f64> {
    let offset = usize::try_from(field.offset)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 field offset is negative".into()))?;
    let start = base
        .checked_add(offset)
        .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 field offset overflow".into()))?;
    match field.datatype {
        7 => {
            let bytes: [u8; 4] = data
                .get(start..start + 4)
                .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 f32 is truncated".into()))?
                .try_into()
                .expect("slice length checked");
            Ok(if big_endian {
                f32::from_be_bytes(bytes)
            } else {
                f32::from_le_bytes(bytes)
            } as f64)
        }
        8 => {
            let bytes: [u8; 8] = data
                .get(start..start + 8)
                .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 f64 is truncated".into()))?
                .try_into()
                .expect("slice length checked");
            Ok(if big_endian {
                f64::from_be_bytes(bytes)
            } else {
                f64::from_le_bytes(bytes)
            })
        }
        datatype => Err(NavStackError::InvalidConfig(format!(
            "PointCloud2 field {} uses unsupported datatype {datatype}",
            field.name
        ))),
    }
}

#[cfg(feature = "simple-2d")]
fn decode_simple_occupancy(
    body: &[u8],
) -> NavStackResult<nuwax_nav_types_messages::OccupancyGrid2D> {
    decode_simple_occupancy_with_boundary(body, None)
}

#[cfg(feature = "simple-2d")]
fn decode_simple_occupancy_with_boundary(
    body: &[u8],
    boundary: Option<&NavigationBoundary>,
) -> NavStackResult<nuwax_nav_types_messages::OccupancyGrid2D> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "simple planner OccupancyGrid frame must be map, got {}",
            grid.header.frame_id,
        )));
    }
    let width = usize::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = usize::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    if width == 0 || height == 0 || !resolution.is_finite() || resolution <= 0.0 {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid dimensions and resolution must be positive".into(),
        ));
    }
    if grid.data.len() != width.saturating_mul(height) {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {}",
            grid.data.len(),
            width * height,
        )));
    }
    if grid
        .info
        .origin
        .position
        .iter()
        .chain(grid.info.origin.orientation_xyzw.iter())
        .any(|value| !value.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid origin contains a non-finite value".into(),
        ));
    }
    let rotation = grid.info.origin.orientation_xyzw;
    let identity = rotation[0].abs() <= 1e-9
        && rotation[1].abs() <= 1e-9
        && rotation[2].abs() <= 1e-9
        && (rotation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "simple planner cannot consume a rotated/non-planar occupancy origin".into(),
        ));
    }
    use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};

    let mut cells = Vec::with_capacity(grid.data.len());
    for y in 0..height {
        for x in 0..width {
            let value = grid.data[y * width + x];
            let center = Point2::new(
                grid.info.origin.position[0] + (x as f64 + 0.5) * resolution,
                grid.info.origin.position[1] + (y as f64 + 0.5) * resolution,
            );
            cells.push(
                if boundary.is_some_and(|boundary| !boundary.contains(center)) || value >= 50 {
                    OccupancyCell::Occupied
                } else if value < 0 {
                    OccupancyCell::Unknown
                } else {
                    OccupancyCell::Free
                },
            );
        }
    }
    timestamp(&grid.header)?;
    Ok(OccupancyGrid2D::new(
        Point2::new(grid.info.origin.position[0], grid.info.origin.position[1]),
        resolution,
        width as u32,
        height as u32,
        cells,
    ))
}

#[cfg(any(feature = "replanning-astar", feature = "guard-runtime"))]
fn decode_astar_costmap(body: &[u8]) -> NavStackResult<nuwax_nav_astar::navigation_map::CostMap2D> {
    use nuwax_nav_astar::navigation_map::{CostMap2D, OCCUPIED, UNKNOWN};

    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "A* OccupancyGrid frame must be map, got {}",
            grid.header.frame_id
        )));
    }
    if grid
        .info
        .origin
        .position
        .iter()
        .chain(grid.info.origin.orientation_xyzw.iter())
        .any(|value| !value.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid origin contains a non-finite value".into(),
        ));
    }
    let orientation = grid.info.origin.orientation_xyzw;
    let identity_rotation = orientation[0].abs() <= 1e-9
        && orientation[1].abs() <= 1e-9
        && orientation[2].abs() <= 1e-9
        && (orientation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity_rotation || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "A* CostMap2D cannot represent a rotated or non-planar OccupancyGrid origin".into(),
        ));
    }
    let width = usize::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = usize::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    if width == 0 || height == 0 || !resolution.is_finite() || resolution <= 0.0 {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid dimensions and resolution must be positive".into(),
        ));
    }
    if grid.data.len() != width.saturating_mul(height) {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {}",
            grid.data.len(),
            width * height
        )));
    }
    let mut map = CostMap2D::new("map", width, height, resolution);
    map.set_origin(Point2::new(
        grid.info.origin.position[0],
        grid.info.origin.position[1],
    ));
    for y in 0..height {
        for x in 0..width {
            let value = grid.data[y * width + x];
            map.set(
                x,
                y,
                if value < 0 {
                    UNKNOWN
                } else if value >= 50 {
                    OCCUPIED
                } else {
                    value as u8
                },
            );
        }
    }
    Ok(map)
}

#[cfg(feature = "replanning-astar")]
fn decode_astar_costmap_with_boundary(
    body: &[u8],
    boundary: Option<&NavigationBoundary>,
) -> NavStackResult<nuwax_nav_astar::navigation_map::CostMap2D> {
    use nuwax_nav_astar::navigation_map::OCCUPIED;

    let mut map = decode_astar_costmap(body)?;
    let Some(boundary) = boundary else {
        return Ok(map);
    };
    for y in 0..map.height() {
        for x in 0..map.width() {
            if !boundary.contains(map.cell_center_world(x, y)) {
                map.set(x, y, OCCUPIED);
            }
        }
    }
    Ok(map)
}

#[cfg(any(feature = "replanning-astar", feature = "guard-runtime"))]
fn require_map_pose(pose: &PoseStamped, component: &str) -> NavStackResult<()> {
    if pose.frame_id == NavFrameId::Map {
        Ok(())
    } else {
        Err(NavStackError::InvalidConfig(format!(
            "{component} frame must be map, got {}",
            pose.frame_id.as_str()
        )))
    }
}

#[cfg(any(feature = "click-goal", feature = "patrolling", feature = "explore"))]
async fn publish_pose(
    transport: &LcmTransport,
    channel: &str,
    pose: &PoseStamped,
) -> NavStackResult<()> {
    let message = schema_pose(pose)?;
    let body = schema::encode_pose_stamped_body(&message)
        .map_err(|error| NavStackError::Internal(format!("PoseStamped encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await
}

#[cfg(feature = "simple-2d")]
async fn publish_point(
    transport: &LcmTransport,
    channel: &str,
    point: &PointStamped,
) -> NavStackResult<()> {
    let body = schema::encode_point_stamped_body(&schema::PointStamped {
        header: schema_header(point.stamp, point.frame_id.as_str())?,
        point: [point.point.x, point.point.y, point.point.z],
    })
    .map_err(|error| NavStackError::Internal(format!("PointStamped encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PointStamped", body).await
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
struct VersionedPathPublication<'a> {
    path_yaws: Option<&'a [f64]>,
    goal: &'a schema::NavigationGoal,
    planner_runtime_generation: i64,
    path_revision: i64,
    path: &'a Path2D,
    boundary_safe: bool,
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn build_versioned_path_envelope(
    publication: VersionedPathPublication<'_>,
    source_timestamp_ns: i64,
) -> NavStackResult<Option<schema::PathEnvelope>> {
    if !publication.boundary_safe || publication.path.points.is_empty() {
        return Ok(None);
    }
    if publication.path_yaws.is_some_and(|yaws| yaws.len()!=publication.path.points.len() || yaws.iter().any(|yaw|!yaw.is_finite())) {
        return Err(NavStackError::InvalidConfig("oriented path dimensions or yaw are invalid".into()));
    }
    let stamp = Timestamp::from_unix(
        source_timestamp_ns / 1_000_000_000,
        (source_timestamp_ns % 1_000_000_000) as u32,
    );
    let header = schema_header(stamp, publication.goal.goal.header.frame_id.as_str())?;
    let path = schema::Path {
        header: header.clone(),
        poses: publication
            .path
            .points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                // 中途沿下一段方向前进；最后一点恢复用户要求的目标朝向。
                let mut orientation_xyzw = publication.path.points.get(index + 1).map_or(
                    publication.goal.goal.pose.orientation_xyzw,
                    |next| {
                        let half_yaw = (next.y - point.y).atan2(next.x - point.x) * 0.5;
                        [0.0, 0.0, half_yaw.sin(), half_yaw.cos()]
                    },
                );
                if let Some(yaw)=publication.path_yaws.and_then(|yaws|yaws.get(index)) {
                    orientation_xyzw=[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()];
                }
                schema::PoseStamped {
                    header: header.clone(),
                    pose: schema::Pose {
                        position: [point.x, point.y, publication.goal.goal.pose.position[2]],
                        orientation_xyzw,
                    },
                }
            })
            .collect(),
    };
    build_versioned_path_from_schema(
        publication.goal,
        publication.planner_runtime_generation,
        publication.path_revision,
        source_timestamp_ns,
        path,
    )
    .map(Some)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn build_versioned_path_from_schema(
    goal: &schema::NavigationGoal,
    planner_runtime_generation: i64,
    path_revision: i64,
    source_timestamp_ns: i64,
    path: schema::Path,
) -> NavStackResult<schema::PathEnvelope> {
    let envelope = schema::PathEnvelope {
route_kind: 0,
schema_version: schema::PATH_ENVELOPE_SCHEMA_VERSION,
session_id: goal.session_id.clone(),
motion_generation: goal.motion_generation,
planner_runtime_generation: planner_runtime_generation,
path_revision: path_revision,
path: path
};
    Ok(envelope)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
async fn publish_versioned_path(
    transport: &LcmTransport,
    channel: &str,
    publication: VersionedPathPublication<'_>,
) -> NavStackResult<bool> {
    let source_timestamp_ns = current_unix_timestamp_ns()?;
    let Some(envelope) = build_versioned_path_envelope(publication, source_timestamp_ns)? else {
        return Ok(false);
    };
    let body = schema::encode_path_envelope_body(&envelope)
        .map_err(|error| NavStackError::Internal(format!("PathEnvelope encode: {error}")))?;
    publish_body(transport, channel, "nuwax_msgs.PathEnvelope", body).await?;
    Ok(true)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn current_unix_timestamp_ns() -> NavStackResult<i64> {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|error| {
            NavStackError::Internal(format!("system clock before Unix epoch: {error}"))
        })?
        .as_nanos();
    i64::try_from(now)
        .map_err(|_| NavStackError::Internal("Unix timestamp exceeds int64 nanoseconds".into()))
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "replanning-astar",
    feature = "patrolling",
    feature = "basic-path-follower"
))]
async fn publish_bool(transport: &LcmTransport, channel: &str, value: bool) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "std_msgs.Bool",
        schema::encode_bool_body(&schema::Bool { data: value }),
    )
    .await
}

#[cfg(feature = "guard-runtime")]
fn build_twist2_frame(channel: &str, twist: Twist2) -> NavStackResult<RawLcmFrame> {
    build_body_frame(
        channel,
        "geometry_msgs.Twist",
        schema::encode_twist_body(&schema::Twist {
            linear: [twist.linear.x, twist.linear.y, 0.0],
            angular: [0.0, 0.0, twist.angular_z],
        }),
    )
}

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling"
))]
async fn publish_twist2(
    transport: &LcmTransport,
    channel: &str,
    twist: Twist2,
) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "geometry_msgs.Twist",
        schema::encode_twist_body(&schema::Twist {
            linear: [twist.linear.x, twist.linear.y, 0.0],
            angular: [0.0, 0.0, twist.angular_z],
        }),
    )
    .await
}

#[cfg(any(feature = "click-goal", feature = "explore"))]
fn schema_pose(pose: &PoseStamped) -> NavStackResult<schema::PoseStamped> {
    Ok(schema::PoseStamped {
        header: schema_header(pose.stamp, pose.frame_id.as_str())?,
        pose: schema::Pose {
            position: [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            orientation_xyzw: [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
        },
    })
}

#[cfg(any(
    feature = "cmu-native",
    feature = "explore",
    feature = "click-goal",
    feature = "patrolling",
    feature = "simple-2d",
    feature = "movement-manager",
    feature = "replanning-astar"
))]
fn schema_header(stamp: Timestamp, frame_id: &str) -> NavStackResult<schema::Header> {
    Ok(schema::Header {
        seq: 0,
        stamp_sec: i32::try_from(stamp.unix_seconds()).map_err(|_| {
            NavStackError::InvalidConfig("timestamp seconds exceed LCM int32 range".into())
        })?,
        stamp_nsec: i32::try_from(stamp.unix_subsec_nanos()).expect("nanoseconds fit int32"),
        frame_id: frame_id.to_string(),
    })
}

fn timestamp(header: &schema::Header) -> NavStackResult<Timestamp> {
    if !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err(NavStackError::InvalidConfig(format!(
            "invalid nanosecond field {}",
            header.stamp_nsec
        )));
    }
    Ok(Timestamp::from_unix(
        i64::from(header.stamp_sec),
        header.stamp_nsec as u32,
    ))
}

fn prefixed(fingerprint: [u8; 8], body: &[u8]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(body);
    encoded
}

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
fn planner_test_goal(session_id: &str, goal_generation: i64) -> schema::NavigationGoal {
    let mut goal = schema::NavigationGoal {
session_id: session_id.into(),
boundary_points: vec![
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0],
            [0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
motion_generation: 1,
goal: schema::PoseStamped {
            header: schema::Header {
                seq: i32::try_from(goal_generation).expect("test generation fits i32"),
                stamp_sec: i32::try_from(goal_generation).expect("test generation fits i32"),
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [4.5, 4.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        },
..schema::NavigationGoal::default()
};

    schema::validate_navigation_goal(&goal).expect("seal planner test goal");
    goal
}

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
fn planner_test_grid_body(revision: i32) -> Vec<u8> {
    schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
        header: schema::Header {
            seq: revision,
            stamp_sec: revision,
            frame_id: "map".into(),
            ..schema::Header::default()
        },
        info: schema::MapMetaData {
            resolution: 1.0,
            width: 6,
            height: 6,
            origin: schema::Pose {
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                ..schema::Pose::default()
            },
            ..schema::MapMetaData::default()
        },
        data: vec![0; 36],
    })
    .expect("encode planner test grid")
}

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
mod planner_input_tests {
    use super::*;

    #[test]
    fn module_internal_native_transport_timeout_and_all_tick_errors_are_fatal() {
        use nuwax_nav_base::{NativeProcessError, NavError};

        for error in [
            NavError::Internal("planner invariant".into()),
            NavError::Transport("planner transport closed".into()),
            NavError::Timeout {
                operation: "planner-input",
                millis: 10,
            },
            NavError::NativeProcess(NativeProcessError::ExecutableMissing {
                path: std::path::PathBuf::from("/missing/planner"),
            }),
        ] {
            assert!(module_input_error::<()>("planner-input", error).is_err());
        }
        assert!(matches!(
            module_input_error::<()>(
                "planner-input",
                NavError::InvalidInput {
                    field: "pose",
                    reason: "non-finite".into(),
                },
            )
            .expect("external module input is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let tick_error = module_tick_error(
            "planner.tick",
            NavError::InvalidInput {
                field: "module",
                reason: "unexpected tick state".into(),
            },
        );
        assert!(matches!(tick_error, NavStackError::Internal(_)));
    }

    #[test]
    fn planner_sensor_validation_rejects_frame_and_nonfinite() {
        let mut pose = schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 40,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        };
        let foreign_frame = schema::encode_pose_stamped_body(&pose).expect("encode foreign pose");
        assert!(matches!(
            decode_planner_pose_input(&foreign_frame, "planner.map_pose")
                .expect("foreign frame is external"),
            OwnerInput::Rejected(_)
        ));

        pose.header.frame_id = "map".into();
        pose.pose.position[1] = f64::INFINITY;
        let non_finite = schema::encode_pose_stamped_body(&pose).expect("encode non-finite pose");
        assert!(matches!(
            decode_planner_pose_input(&non_finite, "planner.map_pose")
                .expect("non-finite pose is external"),
            OwnerInput::Rejected(_)
        ));

    }
}

#[cfg(all(test, feature = "simple-2d"))]
mod simple_tests {
    use nuwax_nav_simple_planner::module::{Module, ModuleOutput, SimplePlannerModule};

    use super::*;

    #[test]
    fn published_path_preserves_goal_heading_and_uses_segment_headings() {
        let mut goal = planner_test_goal("heading", 1);
        goal.goal.pose.position[2] = 3.0;
        let half_yaw = std::f64::consts::FRAC_PI_4;
        goal.goal.pose.orientation_xyzw = [0.0, 0.0, half_yaw.sin(), half_yaw.cos()];
        schema::validate_navigation_goal(&goal).unwrap();
        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::from_unix(12, 0),
            vec![Point2::new(0.5, 0.5), Point2::new(2.5, 0.5)],
        );
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: &goal,
                planner_runtime_generation: 1,
                path_revision: 1,
                path: &path,
                boundary_safe: true,
            },
            12_000_000_000,
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            envelope.path.poses[0].pose.orientation_xyzw,
            [0.0, 0.0, 0.0, 1.0]
        );
        assert_eq!(
            envelope.path.poses[1].pose.orientation_xyzw,
            goal.goal.pose.orientation_xyzw
        );
        assert!(envelope
            .path
            .poses
            .iter()
            .all(|pose| pose.pose.position[2] == 3.0));
    }

    #[test]
    fn simple_occupancy_uses_persistent_map_cells_not_cloud_history() {
        let body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 4,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.5,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    position: [-1.0, -2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0, -1, 50, 49],
        })
        .expect("encode occupancy fixture");

        use nuwax_nav_types_messages::OccupancyCell;
        let grid = decode_simple_occupancy(&body).expect("decode persistent occupancy");
        assert_eq!(grid.origin, Point2::new(-1.0, -2.0));
        assert_eq!(grid.resolution, 0.5);
        assert_eq!(
            grid.cells,
            vec![
                OccupancyCell::Free,
                OccupancyCell::Unknown,
                OccupancyCell::Occupied,
                OccupancyCell::Free
            ]
        );
    }

    #[test]
    fn planning_failure_diagnostic_path_is_not_executable() {
        use nuwax_nav_simple_planner::module::HoldReason;

        assert!(!simple_hold_path_is_executable(HoldReason::PlanningFailed));
        assert!(simple_hold_path_is_executable(HoldReason::StopRequested));
        assert!(simple_hold_path_is_executable(HoldReason::GoalReached));
        assert!(simple_hold_path_is_executable(HoldReason::LifecycleStopped));
    }

    #[test]
    fn simple_planner_rejects_routes_through_unknown_cells() {
        let mut data = vec![-1; 25];
        data[12] = 0;
        data[14] = 0; // 起终点均为空闲，但中间未知带没有可通行证据。
        let body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 5,
                height: 5,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data,
        })
        .expect("encode frontier-style occupancy fixture");
        let grid = decode_simple_occupancy(&body).expect("decode unknown cells");

        let mut planner =
            SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
                cell_size: 1.0,
                inflation_radius: 0.0,
                ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
            })
            .expect("frontier planner config");
        Module::start(&mut planner).expect("start simple planner");
        planner.on_occupancy_grid(grid).expect("frontier occupancy");
        Module::on_odometry(
            &mut planner,
            PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(2.5, 2.5, 0.0),
                    ..Pose3::default()
                },
                NavFrameId::Map,
            ),
        )
        .expect("map pose");
        Module::on_goal(
            &mut planner,
            PointStamped::from_xyz(NavFrameId::Map, 4.5, 2.5, 0.0),
        )
        .expect("frontier goal");

        assert!(matches!(
            Module::tick(&mut planner).expect("planner tick"),
            ModuleOutput::SafetyHold {
                reason: nuwax_nav_simple_planner::module::HoldReason::PlanningFailed,
                ..
            }
        ));
    }

    #[test]
    fn clicked_goal_does_not_replace_authoritative_planner_pose() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(1.0, 2.0, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(7, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("authoritative map pose");

        let clicked = clicked_goal_pose(&schema::PointStamped {
            header: schema::Header {
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            point: [5.0, 2.0, 0.0],
        })
        .expect("clicked goal");
        Module::on_goal(
            &mut planner,
            PointStamped::new(clicked.frame_id, clicked.stamp, clicked.pose.position),
        )
        .expect("goal only");

        let first = match Module::tick(&mut planner).expect("planner tick") {
            ModuleOutput::Plan { path, .. } | ModuleOutput::SafetyHold { path, .. } => {
                path.points[0]
            }
            output => panic!("expected path-bearing output, got {output:?}"),
        };
        assert!(first.distance(Point2::new(1.0, 2.0)) < 0.1);
        assert!(first.distance(Point2::new(5.0, 2.0)) > 3.0);
    }

    #[test]
    fn simple_goal_arriving_before_costmap_is_applied_once_with_full_envelope() {
        let goal = planner_test_goal("simple-goal", 4);
        let expected_key = goal.goal_key();
        let expected_boundary = goal.boundary_points.clone();
        let ready = decode_navigation_goal_input(&schema::encode_navigation_goal_body(&goal).unwrap()).unwrap();
        assert_eq!(ready.goal_key(), expected_key);
        assert_eq!(ready.boundary_points, expected_boundary);

        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(21, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("simple map pose");
        let (applied, boundary) =
            match apply_simple_planner_goal(&mut planner, ready, &planner_test_grid_body(21))
                .expect("apply simple buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid simple goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert!(matches!(
            Module::tick(&mut planner).expect("simple planner tick"),
            ModuleOutput::Plan { .. } | ModuleOutput::SafetyHold { .. }
        ));
    }

    #[test]
    fn simple_stop_holds_until_a_replacement_goal() {
        let mut module = SimplePlannerModule::new();
        Module::start(&mut module).unwrap();
        let mut pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        pose.stamp = Timestamp::from_unix(30, 0);
        Module::on_odometry(&mut module, pose).unwrap();
        for _ in 0..2 {
            Module::on_stop_movement(&mut module, true).unwrap();
            assert!(matches!(
                Module::tick(&mut module).unwrap(),
                ModuleOutput::SafetyHold {
                    reason: nuwax_nav_simple_planner::module::HoldReason::StopRequested,
                    ..
                }
            ));
        }
        let next = planner_test_goal("replacement", 2);
        assert!(matches!(
            apply_simple_planner_goal(&mut module, next, &planner_test_grid_body(30)).unwrap(),
            OwnerInput::Accepted(_)
        ));
        assert!(matches!(
            Module::tick(&mut module).unwrap(),
            ModuleOutput::Plan { .. } | ModuleOutput::SafetyHold { .. }
        ));
    }

    #[test]
    fn simple_new_goal_ignores_map_metadata_and_builds_correlated_path() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(12, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("simple map pose");

        planner
            .on_occupancy_grid(
                decode_simple_occupancy(&planner_test_grid_body(12)).expect("map 12"),
            )
            .expect("apply map 12");
        let goal_12 = planner_test_goal("simple-12", 1);
        let ready_12 = goal_12.clone();
        let (applied_12, boundary_12) =
            match apply_simple_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert_eq!(applied_12, goal_12);
        assert!(boundary_12.is_some());
        let goal_13 = planner_test_goal("simple-13", 2);
        let ready_13 = goal_13.clone();
        let (applied_13, boundary_13) =
            match apply_simple_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        let active_goal = Some(applied_13);
        let active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("simple-13")
        );
        assert!(active_boundary.is_some());
        let path = match Module::tick(&mut planner).expect("simple map-b tick") {
            ModuleOutput::Plan { path, .. } | ModuleOutput::SafetyHold { path, .. } => path,
            output => panic!("expected map-b path, got {output:?}"),
        };
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 23,
                path_revision: 1,
                path: &path,
                boundary_safe: path_respects_navigation_boundary(
                    &path,
                    active_boundary.as_ref(),
                    0.5,
                ),
            },
            13_000_000_000,
        )
        .expect("build map-b PathEnvelope")
        .expect("map-b path stays inside boundary");
        assert_eq!(
            envelope.goal_key(),
            active_goal.as_ref().unwrap().goal_key()
        );
    }

    #[test]
    fn simple_owner_drops_bad_pose_and_grid_then_accepts_new_valid_inputs() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        assert!(matches!(
            decode_planner_pose_input(&[0x44], "simple_planner.map_pose")
                .expect("bad pose is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 30,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid simple pose");
        let pose = match decode_planner_pose_input(&pose_body, "simple_planner.map_pose")
            .expect("valid simple pose")
        {
            OwnerInput::Accepted(pose) => pose,
            OwnerInput::Rejected(reason) => panic!("valid simple pose rejected: {reason}"),
        };
        Module::on_odometry(&mut planner, pose).expect("simple planner accepts valid pose");

        assert!(matches!(
            decode_planner_grid_input(&[0x44], "simple_planner.occupancy")
                .expect("bad grid is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let grid_body = planner_test_grid_body(30);
        assert!(matches!(
            decode_planner_grid_input(&grid_body, "simple_planner.occupancy")
                .expect("valid simple grid"),
            OwnerInput::Accepted(_)
        ));
        let occupancy = decode_simple_occupancy(&grid_body).expect("decode valid simple grid");
        planner
            .on_occupancy_grid(occupancy)
            .expect("simple planner accepts valid grid");
    }
}

#[cfg(all(test, feature = "guard-runtime"))]
mod guarded_tests {
    include!("guard_protocol_tests.rs");
    fn planar_test_costmap(
        observation: &nuwax_nav_runtime::CostmapObservation,
    ) -> &nuwax_nav_astar::navigation_map::CostMap2D {
        match &observation.costmap {
            nuwax_nav_runtime::ExecutionMap::Planar(map) => map,
            _ => panic!("expected planar map"),
        }
    }

    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum GuardedNavSource {
        Pose,
        Costmap,
        Status,
        Goal,
        Path,
        Cancel,
    }

    impl GuardedNavSource {
        const ORDER: [Self; 6] = [
            Self::Pose,
            Self::Costmap,
            Self::Status,
            Self::Goal,
            Self::Path,
            Self::Cancel,
        ];

        fn index(self) -> usize {
            Self::ORDER
                .iter()
                .position(|source| *source == self)
                .expect("guarded navigation source must belong to the source ring")
        }

    }

    fn event_source(event: &GuardedNavEvent) -> GuardedNavSource {
        match event {
            GuardedNavEvent::Pose(_) => GuardedNavSource::Pose,
            GuardedNavEvent::Costmap(_) => GuardedNavSource::Costmap,
            GuardedNavEvent::Status(_) => GuardedNavSource::Status,
            GuardedNavEvent::Goal(_) => GuardedNavSource::Goal,
            GuardedNavEvent::Path(_) => GuardedNavSource::Path,
            GuardedNavEvent::Cancel(_) => GuardedNavSource::Cancel,
        }
    }
    use futures::channel::mpsc::{unbounded, UnboundedSender};
    use nuwax_nav_runtime::{NavMotionPort, NavSchedulerHandle, RuntimeError, SchedulerPhase, TerminalOutcome};
    use tokio::task::JoinHandle;

    #[tokio::test]
    async fn status_twist_and_zero_stay_live_under_continuous_inputs() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        harness.send_path(&sealed_path(&goal));
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = harness.scheduler.subscribe_status();
            while status.borrow().path_revision != 1 {
                status.changed().await.expect("active path status");
            }
        })
        .await
        .expect("active path");

        while harness.commands.next().now_or_never().flatten().is_some() {}
        while harness.states.next().now_or_never().flatten().is_some() {}
        while harness.reached.next().now_or_never().flatten().is_some() {}
        let applied_baseline = harness.scheduler.status().applied_commands;
        let expected_twist = Twist2 {
            linear: Point2::new(0.731, 0.0),
            angular_z: -0.417,
        };
        *harness.planner_twist.lock().unwrap() = expected_twist;
        let mut status = harness.scheduler.subscribe_status();
        let feeder =
            GuardedContentionFeeder::spawn(harness.poses.clone(), harness.costmaps.clone(), goal.goal.pose.position);

        tokio::time::timeout(Duration::from_secs(1), async {
            while status.borrow().applied_commands <= applied_baseline {
                status
                    .changed()
                    .await
                    .expect("post-baseline command status");
            }
        })
        .await
        .expect("post-baseline applied command");
        let applied = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .commands
                    .next()
                    .await
                    .expect("command stream open")
                    .expect("command frame");
                let twist =
                    schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                        .expect("real status Twist");
                if twist.linear[0] == expected_twist.linear.x
                    && twist.linear[1] == expected_twist.linear.y
                    && twist.angular[2] == expected_twist.angular_z
                {
                    return twist;
                }
            }
        })
        .await
        .expect("bounded status Twist publication");
        feeder.assert_ready_evidence("Twist");
        assert_eq!(applied.linear[0], expected_twist.linear.x);
        assert_eq!(applied.linear[1], expected_twist.linear.y);
        assert_eq!(applied.angular[2], expected_twist.angular_z);
        assert!(harness.scheduler.status().applied_commands > applied_baseline);

        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while status.borrow().last_terminal != Some(TerminalOutcome::Succeeded) {
                status.changed().await.expect("Succeeded status");
            }
        })
        .await
        .expect("Succeeded terminal status");

        let zero = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .commands
                    .next()
                    .await
                    .expect("command stream open")
                    .expect("command frame");
                let twist =
                    schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, &frame.payload))
                        .expect("real status ZERO");
                if twist.linear == [0.0; 3] && twist.angular == [0.0; 3] {
                    return twist;
                }
            }
        })
        .await
        .expect("bounded status ZERO publication");
        feeder.assert_ready_evidence("ZERO");
        assert_eq!(zero.linear, [0.0; 3]);
        assert_eq!(zero.angular, [0.0; 3]);

        let reached = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .reached
                    .next()
                    .await
                    .expect("reached stream open")
                    .expect("reached frame");
                let reached = schema::decode_bool_body(&frame.payload).expect("Succeeded bool");
                if reached.data {
                    return reached;
                }
            }
        })
        .await
        .expect("bounded Succeeded bool publication");
        feeder.assert_ready_evidence("Succeeded Bool");
        assert!(reached.data);

        let _terminal = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .states
                    .next()
                    .await
                    .expect("state stream open")
                    .expect("state frame");
                let state = schema::decode_navigation_status_body(&frame.payload)
                    .expect("real status NavigationStatus");
                if state.session_id == goal.session_id
                    && state.state == 3
                {
                    return state;
                }
            }
        })
        .await
        .expect("bounded terminal NavigationStatus publication");
        feeder.assert_ready_evidence("terminal NavigationStatus");
        assert_eq!(
            harness.scheduler.status().last_terminal,
            Some(TerminalOutcome::Succeeded)
        );
        assert!(
            !harness.loop_task.is_finished(),
            "single owner loop remains live"
        );
        feeder.stop().await.expect("contention feeder cleanup");
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn input_wait_propagates_required_stream_close() {
        for source in GuardedNavSource::ORDER {
            let (goals_tx, mut goals) = guarded_test_stream();
            let (poses_tx, mut poses) = guarded_test_stream();
            let (paths_tx, mut paths) = guarded_test_stream();
            let (costmaps_tx, mut costmaps) = guarded_test_stream();
            let (cancels_tx, mut cancels) = guarded_test_stream();
            let (status_tx, mut status) =
                tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
            let payload = vec![source.index() as u8 + 1];
            match source {
                GuardedNavSource::Pose => {
                    send_guarded_test_body(&poses_tx, "geometry_msgs.PoseStamped", payload.clone())
                }
                GuardedNavSource::Costmap => {
                    send_guarded_test_body(&costmaps_tx, "nav_msgs.OccupancyGrid", payload.clone())
                }
                GuardedNavSource::Status => status_tx
                    .send(nuwax_nav_runtime::SchedulerStatus {
                        message: "status-payload".into(),
                        ..nuwax_nav_runtime::SchedulerStatus::default()
                    })
                    .expect("status receiver"),
                GuardedNavSource::Goal => {
                    send_guarded_test_body(&goals_tx, "nuwax_msgs.NavigationGoal", payload.clone())
                }
                GuardedNavSource::Path => {
                    send_guarded_test_body(&paths_tx, "nuwax_msgs.PathEnvelope", payload.clone())
                }
                GuardedNavSource::Cancel => send_guarded_test_body(
                    &cancels_tx,
                    "nuwax_msgs.CancelNavigation",
                    payload.clone(),
                ),
            }
            let event = wait_guarded_nav_event(
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .now_or_never().expect("ready source mapping");
            assert_eq!(event_source(&event), source);
            match event {
                GuardedNavEvent::Pose(Some(Ok(frame)))
                | GuardedNavEvent::Goal(Some(Ok(frame)))
                | GuardedNavEvent::Path(Some(Ok(frame)))
                | GuardedNavEvent::Cancel(Some(Ok(frame))) => assert_eq!(frame.payload, payload),
                GuardedNavEvent::Costmap(Some(Ok(GuardedMapFrame::Planar(frame)))) => {
                    assert_eq!(frame.payload, payload)
                }
                GuardedNavEvent::Status(Ok(())) => {
                    assert_eq!(status.borrow().message, "status-payload")
                }
                _ => panic!("unexpected ready mapping for {source:?}"),
            }
        }

        for source in [
            GuardedNavSource::Pose,
            GuardedNavSource::Costmap,
            GuardedNavSource::Goal,
            GuardedNavSource::Path,
            GuardedNavSource::Cancel,
        ] {
            let (goals_tx, mut goals) = guarded_test_stream();
            let (poses_tx, mut poses) = guarded_test_stream();
            let (paths_tx, mut paths) = guarded_test_stream();
            let (costmaps_tx, mut costmaps) = guarded_test_stream();
            let (cancels_tx, mut cancels) = guarded_test_stream();
            let (_status_tx, mut status) =
                tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
            match source {
                GuardedNavSource::Pose => poses_tx.close_channel(),
                GuardedNavSource::Costmap => costmaps_tx.close_channel(),
                GuardedNavSource::Goal => goals_tx.close_channel(),
                GuardedNavSource::Path => paths_tx.close_channel(),
                GuardedNavSource::Cancel => cancels_tx.close_channel(),
                GuardedNavSource::Status => unreachable!("watch is covered separately"),
            }
            let event = wait_guarded_nav_event(
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .now_or_never().expect("closed stream is Ready");
            assert_eq!(event_source(&event), source);
            assert!(matches!(
                event,
                GuardedNavEvent::Pose(None)
                    | GuardedNavEvent::Costmap(None)
                    | GuardedNavEvent::Goal(None)
                    | GuardedNavEvent::Path(None)
                    | GuardedNavEvent::Cancel(None)
            ));
        }

        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (_paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        assert!(
            tokio::time::timeout(
                Duration::from_millis(10),
                wait_guarded_nav_event(
                    &mut goals,
                    &mut poses,
                    &mut paths,
                    &mut costmaps,
                    &mut cancels,
                    &mut status,
                ),
            )
            .await
            .is_err(),
            "all sources must remain pending"
        );
        status_tx
            .send(nuwax_nav_runtime::SchedulerStatus {
                message: "rebuilt-watch".into(),
                ..nuwax_nav_runtime::SchedulerStatus::default()
            })
            .expect("rebuilt watch receiver");
        let rebuilt = wait_guarded_nav_event(
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        )
        .await;
        assert!(matches!(rebuilt, GuardedNavEvent::Status(Ok(()))));
        assert_eq!(status.borrow().message, "rebuilt-watch");
    }

    #[tokio::test]
    async fn input_wait_propagates_status_close() {
        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (_paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        drop(status_tx);
        let event = wait_guarded_nav_event(
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        )
        .now_or_never().expect("closed watch is Ready");
        assert!(matches!(event, GuardedNavEvent::Status(Err(_))));

        for source in [
            GuardedCloseSource::Pose,
            GuardedCloseSource::Costmap,
            GuardedCloseSource::Status,
            GuardedCloseSource::Goal,
            GuardedCloseSource::Path,
            GuardedCloseSource::Cancel,
        ] {
            assert_guarded_close_exits(GuardedLoopHarness::spawn().await, source).await;
        }
    }

    #[test]
    fn input_errors_preserve_reject_and_fail_closed_paths() {
        assert!(matches!(
            runtime_input_error::<()>(
                "guarded arbitration",
                RuntimeError::StaleInput("old pose".into()),
            )
            .expect("stale external input remains rejectable"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(runtime_input_error::<()>(
            "guarded arbitration", RuntimeError::Motion("传输失败".into()),
        ).unwrap(), OwnerInput::Rejected(reason) if reason.contains("传输失败")));
        assert!(matches!(
            decode_guarded_pose_input(&[0xff]).expect("bad pose remains external input"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(
            decode_guarded_costmap_input(&[0xff]).expect("bad costmap remains external input"),
            OwnerInput::Rejected(_)
        ));
    }

    fn active_goal(session_id: &str) -> schema::NavigationGoal {
        schema::NavigationGoal {
session_id: session_id.into(),
..schema::NavigationGoal::default()
}
    }

    fn sealed_goal(session_id: &str) -> schema::NavigationGoal {
        let mut goal = schema::NavigationGoal {
session_id: session_id.into(),
motion_generation: 1,
goal: schema::PoseStamped {
                header: schema::Header {
                    stamp_sec: 1,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
};
        schema::validate_navigation_goal(&goal).expect("seal goal");
        goal
    }

    fn sealed_path(goal: &schema::NavigationGoal) -> schema::PathEnvelope {
        let mut path = schema::PathEnvelope {
route_kind: 0,
schema_version: schema::PATH_ENVELOPE_SCHEMA_VERSION,
session_id: goal.session_id.clone(),
motion_generation: goal.motion_generation as u64,
planner_runtime_generation: saturating_i64(runtime_generation()),
path_revision: 1,
path: schema::Path {
                header: schema::Header {
                    stamp_sec: 2,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                poses: vec![schema::PoseStamped {
                    header: schema::Header {
                        stamp_sec: 2,
                        frame_id: "map".into(),
                        ..schema::Header::default()
                    },
                    pose: goal.goal.pose.clone(),
                }],
            },
..schema::PathEnvelope::default()
};
        schema::validate_path_envelope(&path).expect("seal path");
        path
    }

    fn rollover_goal(
        previous: &schema::NavigationGoal,
        session_id: &str,
        goal_generation: i64,
    ) -> schema::NavigationGoal {
        let mut goal = previous.clone();
        goal.session_id = session_id.into();
        goal.motion_generation = goal_generation as u64;

        schema::validate_navigation_goal(&goal).expect("seal rollover goal");
        goal
    }

    fn guarded_pose_body(stamp_sec: i32) -> Vec<u8> {
        guarded_pose_body_at(stamp_sec, [0.0; 3])
    }

    fn guarded_pose_body_at(stamp_sec: i32, position: [f64; 3]) -> Vec<u8> {
        schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                seq: stamp_sec,
                stamp_sec,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position,
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode guarded test pose")
    }

    fn guarded_costmap_body(revision: i32) -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: revision,
                stamp_sec: revision,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode guarded costmap")
    }

    #[derive(Clone)]
    pub(super) struct RecordingMotion(pub(super) Arc<Mutex<Vec<String>>>, Twist2);

    impl RecordingMotion {
        pub(super) fn new(calls: Arc<Mutex<Vec<String>>>) -> Self {
            Self(calls, Twist2::ZERO)
        }
    }

    struct FirstAcquireMotion {
        inner: RecordingMotion,
        reject_first: bool,
    }

    #[async_trait::async_trait]
    impl nuwax_nav_runtime::NavMotionPort for FirstAcquireMotion {
        fn measured_motion(&self) -> Option<nuwax_nav_runtime::MeasuredMotion> {
            nuwax_nav_runtime::NavMotionPort::measured_motion(&self.inner)
        }
        async fn acquire(&mut self,generation:u64)->Result<u64,nuwax_nav_runtime::MotionPortError> {
            if self.reject_first {
                self.reject_first=false;
                self.inner.0.lock().unwrap().push("rejected_acquire:".into());
                return Err(nuwax_nav_runtime::MotionPortError {message:"代次过期".into(),code:Some(nuwax_motion_safety_guard::wire::ErrorCode::StaleGeneration)});
            }
            self.inner.acquire(generation).await
        }
        async fn velocity(&mut self,twist:Twist2)->Result<(),nuwax_nav_runtime::MotionPortError> { self.inner.velocity(twist).await }
        async fn release(&mut self)->Result<(),nuwax_nav_runtime::MotionPortError> { self.inner.release().await }

}

    #[async_trait::async_trait]
    impl nuwax_nav_runtime::NavMotionPort for RecordingMotion {
        fn measured_motion(&self) -> Option<nuwax_nav_runtime::MeasuredMotion> {
            // 夹具即时采用已接受命令；读取时给出当前执行速度的测量快照。
            let now = nuwax_nav_runtime::motion_feedback::monotonic_ns();
            Some(nuwax_nav_runtime::MeasuredMotion {
                source_id: "test:recording-motion".into(),
                source_timestamp_ns: current_unix_timestamp_ns().unwrap() as u64,
                received_monotonic_ns: now,
                valid_until_monotonic_ns: u64::MAX,
                frame_id: "base_link".into(),
                body_linear_xyz: [self.1.linear.x, self.1.linear.y, 0.],
                body_angular_xyz: [0., 0., self.1.angular_z],
                orientation_wxyz: Some([1., 0., 0., 0.]),
            })
        }
        async fn acquire(&mut self,generation:u64)->Result<u64,nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push(format!("acquire:{generation}")); Ok(generation)
        }
        async fn velocity(&mut self,twist:Twist2)->Result<(),nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push("twist:".into());
            self.1=twist; Ok(())
        }
        async fn release(&mut self)->Result<(),nuwax_nav_runtime::MotionPortError> {
            self.0.lock().unwrap().push("release:".into()); Ok(())
        }

}

    struct CommandingPlanner {
        succeeds: Arc<std::sync::atomic::AtomicBool>,
        twist: Arc<Mutex<Twist2>>,
    }

    impl nuwax_nav_runtime::NavPlanner for CommandingPlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {}

        fn update_costmap(&mut self, _costmap: nuwax_nav_astar::navigation_map::CostMap2D) {}

        fn step(&mut self) -> nuwax_nav_runtime::PlannerStep {
            if self
                .succeeds
                .swap(false, std::sync::atomic::Ordering::SeqCst)
            {
                return nuwax_nav_runtime::PlannerStep::Succeeded;
            }
            nuwax_nav_runtime::PlannerStep::Command(*self.twist.lock().unwrap())
        }

        fn stop(&mut self) {}
    }

    #[derive(Default)]
    struct GuardedDequeueLedger {
        pending: std::sync::atomic::AtomicUsize,
        dequeued: std::sync::atomic::AtomicUsize,
    }

    type GuardedTestInput = (
        TransportResult<RawLcmFrame>,
        Option<oneshot::Sender<()>>,
        Option<Arc<GuardedDequeueLedger>>,
    );
    type GuardedTestSender = UnboundedSender<GuardedTestInput>;

    struct GuardedContentionFeeder {
        stop: tokio::sync::watch::Sender<bool>,
        tasks: Vec<JoinHandle<()>>,
        pose_ledger: Arc<GuardedDequeueLedger>,
        costmap_ledger: Arc<GuardedDequeueLedger>,
    }

    impl GuardedContentionFeeder {
        fn spawn(poses: GuardedTestSender, costmaps: GuardedTestSender, position: [f64; 3]) -> Self {
            const WINDOW: usize = 8;
            let (stop, stop_rx) = tokio::sync::watch::channel(false);
            let pose_ledger = Arc::new(GuardedDequeueLedger::default());
            let costmap_ledger = Arc::new(GuardedDequeueLedger::default());

            let pose_task = {
                let ledger = Arc::clone(&pose_ledger);
                let mut stop = stop_rx.clone();
                tokio::spawn(async move {
                    let mut stamp = 100_i32;
                    let mut receipts = futures::stream::FuturesUnordered::new();
                    for _ in 0..WINDOW {
                        receipts.push(send_guarded_test_body_observed_with_ledger(
                            &poses,
                            "geometry_msgs.PoseStamped",
                            guarded_pose_body_at(stamp, position),
                            Some(Arc::clone(&ledger)),
                        ));
                        stamp += 1;
                    }
                    loop {
                        if *stop.borrow() {
                            break;
                        }
                        tokio::select! {
                            biased;
                            changed = stop.changed() => {
                                if changed.is_err() || *stop.borrow() {
                                    break;
                                }
                            }
                            receipt = receipts.next() => {
                                receipt.expect("pose receipt stream").expect("pose dequeued");
                                if *stop.borrow() {
                                    break;
                                }
                                receipts.push(send_guarded_test_body_observed_with_ledger(
                                    &poses,
                                    "geometry_msgs.PoseStamped",
                                    guarded_pose_body_at(stamp, position),
                                    Some(Arc::clone(&ledger)),
                                ));
                                stamp += 1;
                            }
                        }
                    }
                })
            };
            let costmap_task = {
                let ledger = Arc::clone(&costmap_ledger);
                let mut stop = stop_rx;
                tokio::spawn(async move {
                    let mut stamp = 200_i32;
                    let mut receipts = futures::stream::FuturesUnordered::new();
                    for _ in 0..WINDOW {
                        receipts.push(send_guarded_test_body_observed_with_ledger(
                            &costmaps,
                            "nav_msgs.OccupancyGrid",
                            guarded_costmap_body(stamp),
                            Some(Arc::clone(&ledger)),
                        ));
                        stamp += 1;
                    }
                    loop {
                        if *stop.borrow() {
                            break;
                        }
                        tokio::select! {
                            biased;
                            changed = stop.changed() => {
                                if changed.is_err() || *stop.borrow() {
                                    break;
                                }
                            }
                            receipt = receipts.next() => {
                                receipt.expect("costmap receipt stream").expect("costmap dequeued");
                                if *stop.borrow() {
                                    break;
                                }
                                receipts.push(send_guarded_test_body_observed_with_ledger(
                                    &costmaps,
                                    "nav_msgs.OccupancyGrid",
                                    guarded_costmap_body(stamp),
                                    Some(Arc::clone(&ledger)),
                                ));
                                stamp += 1;
                            }
                        }
                    }
                })
            };

            Self {
                stop,
                tasks: vec![pose_task, costmap_task],
                pose_ledger,
                costmap_ledger,
            }
        }

        fn assert_ready_evidence(&self, observation: &str) {
            let pose = self
                .pose_ledger
                .pending
                .load(std::sync::atomic::Ordering::SeqCst);
            let costmap = self
                .costmap_ledger
                .pending
                .load(std::sync::atomic::Ordering::SeqCst);
            assert!(
                pose > 0 || costmap > 0,
                "{observation} lacked unresolved Pose/Costmap dequeue evidence"
            );
        }

        async fn stop(self) -> Result<(), String> {
            let stop_error = self
                .stop
                .send(true)
                .err()
                .map(|_| "contention feeder stop receivers closed".to_string());
            let reap_error = reap_guarded_tasks(self.tasks, Duration::from_secs(1))
                .await
                .err();
            match (stop_error, reap_error) {
                (None, None) => Ok(()),
                (Some(error), None) | (None, Some(error)) => Err(error),
                (Some(stop_error), Some(reap_error)) => Err(format!("{stop_error}; {reap_error}")),
            }
        }
    }

    async fn reap_guarded_tasks(
        tasks: Vec<JoinHandle<()>>,
        join_timeout: Duration,
    ) -> Result<(), String> {
        let mut errors = Vec::new();
        for (index, mut task) in tasks.into_iter().enumerate() {
            match tokio::time::timeout(join_timeout, &mut task).await {
                Ok(Ok(())) => {}
                Ok(Err(error)) => errors.push(format!("task {index} join failed: {error}")),
                Err(_) => {
                    task.abort();
                    match task.await {
                        Ok(()) => errors.push(format!(
                            "task {index} timed out after {join_timeout:?} but abort joined cleanly"
                        )),
                        Err(error) => errors.push(format!(
                            "task {index} timed out after {join_timeout:?}; abort join: {error}"
                        )),
                    }
                }
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("; "))
        }
    }

    struct GuardedTaskExitProbe(Arc<std::sync::atomic::AtomicBool>);

    impl Drop for GuardedTaskExitProbe {
        fn drop(&mut self) {
            self.0.store(true, std::sync::atomic::Ordering::SeqCst);
        }
    }

    #[tokio::test]
    async fn contention_feeder_reaps_every_task_before_reporting_failures() {
        let first_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let later_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let panic_reaped = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let completed = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let tasks = vec![
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&first_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    std::future::pending::<()>().await;
                })
            },
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&later_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    std::future::pending::<()>().await;
                })
            },
            {
                let probe = GuardedTaskExitProbe(Arc::clone(&panic_reaped));
                tokio::spawn(async move {
                    let _probe = probe;
                    panic!("intentional feeder join failure");
                })
            },
            {
                let completed = Arc::clone(&completed);
                tokio::spawn(async move {
                    completed.store(true, std::sync::atomic::Ordering::SeqCst);
                })
            },
        ];

        let error = reap_guarded_tasks(tasks, Duration::from_millis(10))
            .await
            .expect_err("timeout and JoinError must be reported after full cleanup");
        assert!(error.contains("task 0 timed out"), "{error}");
        assert!(error.contains("task 1 timed out"), "{error}");
        assert!(error.contains("task 2 join failed"), "{error}");
        assert!(first_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(later_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(panic_reaped.load(std::sync::atomic::Ordering::SeqCst));
        assert!(completed.load(std::sync::atomic::Ordering::SeqCst));

        let normal_a = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let normal_b = Arc::new(std::sync::atomic::AtomicBool::new(false));
        reap_guarded_tasks(
            vec![
                {
                    let completed = Arc::clone(&normal_a);
                    tokio::spawn(async move {
                        completed.store(true, std::sync::atomic::Ordering::SeqCst);
                    })
                },
                {
                    let completed = Arc::clone(&normal_b);
                    tokio::spawn(async move {
                        completed.store(true, std::sync::atomic::Ordering::SeqCst);
                    })
                },
            ],
            Duration::from_secs(1),
        )
        .await
        .expect("normal feeder cleanup");
        assert!(normal_a.load(std::sync::atomic::Ordering::SeqCst));
        assert!(normal_b.load(std::sync::atomic::Ordering::SeqCst));
    }

    struct GuardedLoopHarness {
        goals: GuardedTestSender,
        poses: GuardedTestSender,
        paths: GuardedTestSender,
        costmaps: GuardedTestSender,
        cancels: GuardedTestSender,
        commands: LcmFrameStream,
        states: LcmFrameStream,
        reached: LcmFrameStream,
        shutdown: Option<oneshot::Sender<()>>,
        loop_task: JoinHandle<NavStackResult<()>>,
        scheduler: NavSchedulerHandle,
        scheduler_task: JoinHandle<Result<(), RuntimeError>>,
        motion_calls: Arc<Mutex<Vec<String>>>,
        planner_succeeds: Arc<std::sync::atomic::AtomicBool>,
        planner_twist: Arc<Mutex<Twist2>>,
    }

    impl GuardedLoopHarness {
        async fn spawn() -> Self {Self::spawn_with_planar_mode(false).await}

        async fn spawn_with_planar_mode(planar_mode:bool) -> Self {
            Self::spawn_with_reference(planar_mode, "base_link").await
        }

        async fn spawn_with_reference(planar_mode:bool, body_reference: &'static str) -> Self {
            Self::spawn_with_first_acquire(planar_mode, body_reference, false).await
        }

        async fn spawn_with_first_acquire(planar_mode:bool, body_reference: &'static str, reject_first: bool) -> Self {
            let calls=Arc::new(Mutex::new(Vec::new()));
            Self::spawn_with_motion(planar_mode,body_reference,
                FirstAcquireMotion {inner:RecordingMotion::new(calls.clone()),reject_first},calls).await
        }

        async fn spawn_with_motion<M:nuwax_nav_runtime::NavMotionPort>(planar_mode:bool, body_reference: &'static str, motion:M, motion_calls:Arc<Mutex<Vec<String>>>) -> Self {
            static NEXT_LOOP_ID: std::sync::atomic::AtomicU64 =
                std::sync::atomic::AtomicU64::new(0);
            let loop_id = NEXT_LOOP_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // One real liblcm context is shared by this in-process fixture.
            // Functional owner/codec checks do not require a host multicast route.
            let transport = LcmTransport::new_real("memq://")
                .expect("guarded loop test LCM memq");
            let planner_succeeds = Arc::new(std::sync::atomic::AtomicBool::new(false));
            let planner_twist = Arc::new(Mutex::new(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.1,
            }));
            let scheduler = nuwax_nav_runtime::NavScheduler::new(
                motion,
                CommandingPlanner {
                    succeeds: Arc::clone(&planner_succeeds),
                    twist: Arc::clone(&planner_twist),
                },
                nuwax_nav_runtime::SchedulerConfig {
                    tick_interval: Duration::from_millis(5),

                    path_wait_timeout: Duration::from_secs(5),

                },
            )
            .expect("guarded loop scheduler");
            let (scheduler, scheduler_task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
            tokio::time::timeout(Duration::from_secs(2), async {
                while !scheduler.status().motion_sink_ready {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("scheduler motion readiness");

            let (goals, goal_stream) = guarded_test_stream();
            let (poses, pose_stream) = guarded_test_stream();
            let (paths, path_stream) = guarded_test_stream();
            let (costmaps, costmap_stream) = guarded_test_stream();
            let (cancels, cancel_stream) = guarded_test_stream();
            let (shutdown, shutdown_rx) = oneshot::channel();
            let status = scheduler.subscribe_status();
            let loop_scheduler = scheduler.clone();
            let mut node = TaskChannels { id: format!("guarded-loop-test-{loop_id}").into(), inputs: Vec::new(), outputs: Vec::new() };
            node.outputs = vec![
                format!("/test/guarded/{loop_id}/cmd"),
                format!("/test/guarded/{loop_id}/state"),
                format!("/test/guarded/{loop_id}/reached"),
                format!("/test/guarded/{loop_id}/cancelled"),
            ];
            let commands = typed_stream(&transport, &node.outputs[0], "geometry_msgs.Twist")
                .await
                .expect("guarded command test stream");
            let states = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.NavigationStatus")
                .await
                .expect("guarded state test stream");
            let reached = typed_stream(&transport, &node.outputs[2], "std_msgs.Bool")
                .await
                .expect("guarded reached test stream");
            let loop_transport = transport.clone();
            let loop_task = tokio::spawn(async move {
                run_guarded_nav_loop(&node, &loop_transport, &loop_scheduler, shutdown_rx, goal_stream, pose_stream, path_stream, costmap_stream, cancel_stream, status, None, planar_mode, body_reference, None)
                .await
            });

            Self {
                goals,
                poses,
                paths,
                costmaps,
                cancels,
                commands,
                states,
                reached,
                shutdown: Some(shutdown),
                loop_task,
                scheduler,
                scheduler_task,
                motion_calls,
                planner_succeeds,
                planner_twist,
            }
        }

        fn send_goal(&self, goal: &schema::NavigationGoal) {
            send_guarded_test_body(
                &self.goals,
                "nuwax_msgs.NavigationGoal",
                schema::encode_navigation_goal_body(goal).expect("encode guarded test goal"),
            );
        }

        async fn send_goal_and_wait_dequeued(&self, goal: &schema::NavigationGoal) {
            let dequeued = send_guarded_test_body_observed(
                &self.goals,
                "nuwax_msgs.NavigationGoal",
                schema::encode_navigation_goal_body(goal).expect("encode guarded test goal"),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded goal was not dequeued")
                .expect("guarded goal dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn send_pose(&self, stamp_sec: i32) {
            send_guarded_test_body(
                &self.poses,
                "geometry_msgs.PoseStamped",
                guarded_pose_body(stamp_sec),
            );
        }

        fn send_costmap(&self, revision: i32) {
            send_guarded_test_body(
                &self.costmaps,
                "nav_msgs.OccupancyGrid",
                guarded_costmap_body(revision),
            );
        }

        fn send_path(&self, path: &schema::PathEnvelope) {
            send_guarded_test_body(
                &self.paths,
                "nuwax_msgs.PathEnvelope",
                schema::encode_path_envelope_body(path).expect("encode guarded test path"),
            );
        }

        fn send_cancel(&self, goal: &schema::NavigationGoal) {
            send_guarded_test_body(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal),
            );
        }

        async fn send_cancel_and_wait_dequeued(
            &self,
            goal: &schema::NavigationGoal,
        ) {
            let dequeued = send_guarded_test_body_observed(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded cancellation was not dequeued")
                .expect("guarded cancellation dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn cancel_body(goal: &schema::NavigationGoal) -> Vec<u8> {
            schema::encode_cancel_navigation_body(&schema::CancelNavigation {
                session_id: goal.session_id.clone(),
                reason: 1,
            })
            .expect("encode guarded test cancellation")
        }

        fn motion_call_count(&self, prefix: &str) -> usize {
            self.motion_calls
                .lock()
                .unwrap()
                .iter()
                .filter(|call| call.starts_with(prefix))
                .count()
        }

        async fn wait_for_session(&self, session_id: &str) {
            tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    let snapshot=self.scheduler.status();
                    if snapshot.session_id.as_deref() == Some(session_id) && snapshot.applied_commands>0 {
                        break;
                    }
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap_or_else(|_| {
                panic!(
                    "session {session_id} was not activated; status={:?}",
                    self.scheduler.status()
                )
            });
        }

        async fn wait_for_pose(&self, stamp_sec: i32) {
            let timestamp_ns = i64::from(stamp_sec) * 1_000_000_000;
            tokio::time::timeout(Duration::from_secs(1), async {
                while self.scheduler.status().pose_source_timestamp_ns != timestamp_ns {
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("pose was not consumed");
        }

        async fn wait_for_costmap(&self, stamp_sec: i32) {
            let timestamp_ns = i64::from(stamp_sec) * 1_000_000_000;
            tokio::time::timeout(Duration::from_secs(1), async {
                while self.scheduler.status().costmap_source_timestamp_ns != timestamp_ns {
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("costmap was not consumed");
        }

        async fn wait_for_release(&self) {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    if self
                        .motion_calls
                        .lock()
                        .unwrap()
                        .iter()
                        .any(|call| call.starts_with("release:"))
                    {
                        break;
                    }
                    assert!(!self.loop_task.is_finished(), "guarded loop exited early");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap_or_else(|_| {
                panic!(
                    "未观察到租约释放请求; status={:?} calls={:?}",
                    self.scheduler.status(),
                    self.motion_calls.lock().unwrap().clone()
                )
            });
        }

        async fn wait_for_cancelled_status(
            &mut self,
            goal: &schema::NavigationGoal,
        ) -> schema::NavigationStatus {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let frame = self
                        .states
                        .next()
                        .await
                        .expect("guarded state stream open")
                        .expect("guarded state transport frame");
                    let state = schema::decode_navigation_status_body(&frame.payload)
                        .expect("guarded NavigationStatus body");
                    if state.session_id == goal.session_id
                        && state.motion_generation == goal.motion_generation as i64
                        && state.goal_status == schema::NAVIGATION_GOAL_STATUS_CANCELLED
                    {
                        return state;
                    }
                }
            })
            .await
            .expect("关联目标的取消状态")
        }

        async fn assert_no_session(&self) {
            tokio::time::sleep(Duration::from_millis(30)).await;
            assert!(
                self.scheduler.status().session_id.is_none(),
                "goal activated before all required inputs"
            );
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        async fn shutdown(mut self) {
            if let Some(shutdown) = self.shutdown.take() {
                shutdown.send(()).expect("guarded loop shutdown receiver");
            }
            tokio::time::timeout(Duration::from_secs(1), self.loop_task)
                .await
                .expect("guarded loop shutdown timeout")
                .expect("guarded loop join")
                .expect("guarded loop result");
            self.scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command");
            self.scheduler_task
                .await
                .expect("scheduler join")
                .expect("scheduler result");
        }
    }

    #[derive(Clone, Copy)]
    enum GuardedCloseSource {
        Pose,
        Costmap,
        Status,
        Goal,
        Path,
        Cancel,
    }

    async fn assert_guarded_close_exits(
        mut harness: GuardedLoopHarness,
        source: GuardedCloseSource,
    ) {
        match source {
            GuardedCloseSource::Pose => harness.poses.close_channel(),
            GuardedCloseSource::Costmap => harness.costmaps.close_channel(),
            GuardedCloseSource::Goal => harness.goals.close_channel(),
            GuardedCloseSource::Path => harness.paths.close_channel(),
            GuardedCloseSource::Cancel => harness.cancels.close_channel(),
            GuardedCloseSource::Status => harness
                .scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command"),
        }
        let error = tokio::time::timeout(Duration::from_secs(1), &mut harness.loop_task)
            .await
            .expect("closed source must terminate without busy-loop")
            .expect("guarded loop join")
            .expect_err("closed required source must fail closed");
        assert!(
            error.to_string().contains("closed"),
            "unexpected close error: {error}"
        );
        if !matches!(source, GuardedCloseSource::Status) {
            harness
                .scheduler
                .shutdown()
                .await
                .expect("scheduler shutdown command");
        }
        harness
            .scheduler_task
            .await
            .expect("scheduler join")
            .expect("scheduler result");
    }

    fn guarded_test_stream() -> (GuardedTestSender, LcmFrameStream) {
        let (sender, receiver) = unbounded::<GuardedTestInput>();
        let receiver = receiver.map(|(frame, dequeued, ledger)| {
            if let Some(ledger) = ledger {
                ledger
                    .pending
                    .fetch_sub(1, std::sync::atomic::Ordering::SeqCst);
                ledger
                    .dequeued
                    .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            }
            if let Some(dequeued) = dequeued {
                let _ = dequeued.send(());
            }
            frame
        });
        (sender, Box::pin(receiver))
    }

    fn send_guarded_test_body(sender: &GuardedTestSender, type_name: &str, body: Vec<u8>) {
        send_guarded_test_body_with_receipt(sender, type_name, body, None, None);
    }

    fn send_guarded_test_body_observed(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
    ) -> oneshot::Receiver<()> {
        send_guarded_test_body_observed_with_ledger(sender, type_name, body, None)
    }

    fn send_guarded_test_body_observed_with_ledger(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
        ledger: Option<Arc<GuardedDequeueLedger>>,
    ) -> oneshot::Receiver<()> {
        let (dequeued, receipt) = oneshot::channel();
        if let Some(ledger) = ledger.as_ref() {
            ledger
                .pending
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }
        send_guarded_test_body_with_receipt(sender, type_name, body, Some(dequeued), ledger);
        receipt
    }

    fn send_guarded_test_body_with_receipt(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
        dequeued: Option<oneshot::Sender<()>>,
        ledger: Option<Arc<GuardedDequeueLedger>>,
    ) {
        let frame = RawLcmFrameBuilder::new(
            LcmTopic::new("/test/guarded/input").expect("guarded test topic"),
            LcmMessageType::new(type_name).expect("guarded test message type"),
        )
        .payload(body)
        .build();
        sender
            .unbounded_send((Ok(frame), dequeued, ledger))
            .expect("guarded test input stream open");
    }

    #[derive(Clone, Copy)]
    enum ColdInput {
        Goal,
        Pose,
        Costmap,
    }

    #[tokio::test]
    async fn guarded_latest_goal_waits_and_replaces_without_input_gates() {
        let harness = GuardedLoopHarness::spawn().await;
        let mut first = sealed_goal("first");
        first.motion_generation=7;
        let mut latest = sealed_goal("latest");
        // 来源代次/运动代次较小也不再由桥的目标水位拒绝。
        latest.motion_generation = 1;
        harness.send_goal_and_wait_dequeued(&first).await;
        harness.send_goal_and_wait_dequeued(&latest).await;
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.scheduler.status().session_id.as_deref()!=Some("latest"){
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        assert!(harness.scheduler.status().message.contains("等待"));
        assert_eq!(harness.motion_call_count("acquire:"), 0);
        harness.send_pose(2);
        harness.send_costmap(12);
        harness.wait_for_pose(2).await;
        harness.wait_for_costmap(12).await;
        harness.send_path(&sealed_path(&latest));
        tokio::time::timeout(Duration::from_secs(1), async {
            while !harness
                .scheduler
                .status()
                .last_applied_twist
                .is_some_and(|v| v != Twist2::ZERO)
            {
                assert!(!harness.loop_task.is_finished());
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("latest")
        );
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        assert_eq!(harness.motion_call_count("release:"), 0);
        harness.shutdown().await;
    }

    #[test]
    fn goal_options_and_map_frame_are_checked_at_decode() {
        let goal = sealed_goal("decode");
        let body = schema::encode_navigation_goal_body(&goal).unwrap();
        assert_eq!(decode_navigation_goal_input(&body).unwrap(), goal);
        let mut foreign = goal.clone();
        foreign.goal.header.frame_id = "odom".into();
        assert!(decode_navigation_goal_input(
            &schema::encode_navigation_goal_body(&foreign).unwrap()
        )
        .unwrap_err()
        .contains("map"));
        let mut options = goal.clone();
        options.position_tolerance_m = Some(0.31415926535);
        let mut invalid = schema::encode_navigation_goal_body(&options).unwrap();
        let at = invalid
            .windows(8)
            .position(|b| b == 0.31415926535_f64.to_be_bytes())
            .unwrap();
        invalid[at..at + 8].copy_from_slice(&(-1_f64).to_be_bytes());
        assert!(decode_navigation_goal_input(&invalid).is_err());
    }

    #[tokio::test]
    async fn guarded_real_loop_accepts_all_cold_goal_pose_costmap_orders() {
        for order in [
            [ColdInput::Goal, ColdInput::Costmap, ColdInput::Pose],
            [ColdInput::Goal, ColdInput::Pose, ColdInput::Costmap],
            [ColdInput::Pose, ColdInput::Goal, ColdInput::Costmap],
        ] {
            let harness = GuardedLoopHarness::spawn().await;
            let goal = sealed_goal("cold-order-goal");
            for (index, input) in order.into_iter().enumerate() {
                match input {
                    ColdInput::Goal => {
                        harness.send_goal(&goal);
                        tokio::time::sleep(Duration::from_millis(20)).await;
                    }
                    ColdInput::Pose => {
                        harness.send_pose(1);
                        harness.wait_for_pose(1).await;
                    }
                    ColdInput::Costmap => {
                        harness.send_costmap(11);
                        harness.wait_for_costmap(11).await;
                    }
                }
                if index < 2 {
                    assert_eq!(harness.motion_call_count("acquire:"),0,"输入未齐时只保存目标");
                    assert!(harness.scheduler.status().last_terminal.is_none());
                }
            }
            harness.wait_for_session("cold-order-goal").await;
            harness.shutdown().await;
        }
    }

    #[tokio::test]
    async fn planar_observation_clear_fences_active_scheduler_and_rejects_replay() {
        use nuwax_codec::lcm::planar_navigation::{self,PlanarNavigationMap};
        let harness=GuardedLoopHarness::spawn_with_planar_mode(true).await;
        let goal=sealed_goal("planar-goal");
        let timestamp=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as i64-100_000_000;
        let mut grid=schema::decode_occupancy_grid_body(&guarded_costmap_body(1)).unwrap();
        grid.header.stamp_sec=(timestamp/1_000_000_000) as i32;
        grid.header.stamp_nsec=(timestamp%1_000_000_000) as i32;
        let value=PlanarNavigationMap {
            map_frame_generation: 0,
            runtime_generation:7,session_id:"guarded-test-map".into(),
            session_generation:2,map_revision:4,
            map_boot_nonce:[0x42;16],map_commit_sequence:1,
            map_source_timestamp_ns:1,observation_revision:1,source_sequence:0,
            source_timestamp_ns:timestamp,cleared:false,grid,
        };
        let body=planar_navigation::encode_body(&value).unwrap();
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,body.clone());
        harness.send_pose(1);harness.wait_for_pose(1).await;
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.scheduler.status().costmap_source_timestamp_ns!=timestamp {
                assert!(!harness.loop_task.is_finished());
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        harness.send_goal(&goal);harness.wait_for_session("planar-goal").await;
        harness.send_path(&sealed_path(&goal));
        tokio::time::timeout(Duration::from_secs(1),async {
            while harness.motion_call_count("twist:")==0 {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.unwrap();
        let cleared=PlanarNavigationMap {cleared:true,observation_revision:2,
            grid:Default::default(),..value};
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,
            planar_navigation::encode_body(&cleared).unwrap());
        harness.wait_for_release().await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,body);
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn rejected_lease_publishes_candidate_failure_and_owner_accepts_next_goal() {
        let mut harness = GuardedLoopHarness::spawn_with_first_acquire(false, "base_link", true).await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        let rejected = sealed_goal("rejected-lease");
        harness.send_goal_and_wait_dequeued(&rejected).await;
        let failure = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_status_body(&frame.payload).unwrap();
                if state.session_id == rejected.session_id && state.state == 4 { break state; }
            }
        }).await.expect("必须实际发布被拒绝候选目标的失败");
        assert_eq!(failure.goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);

        assert_eq!(failure.motion_generation, rejected.motion_generation as i64);

        assert_eq!(failure.path_revision, 0);
        assert_eq!(failure.planner_runtime_generation, 0);
        assert_ne!(failure.sink_ready, 0);
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.assert_no_session().await;
        assert_eq!(harness.motion_call_count("rejected_acquire:"), 1);
        assert_eq!(harness.motion_call_count("acquire:"), 0, "失败目标不能被新输入自动复活");
        assert_eq!(harness.motion_call_count("release:"), 0, "未取得租约不能伪造归零");
        let mut next = sealed_goal("lease-recovered");
        next.motion_generation = rejected.motion_generation + 1;

        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session(&next.session_id).await;
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        harness.send_cancel(&next);
        harness.wait_for_cancelled_status(&next).await;
        harness.shutdown().await;
    }

    async fn activate_map_a(harness: &GuardedLoopHarness) -> schema::NavigationGoal {
        let goal_a = sealed_goal("session-a");
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.send_goal(&goal_a);
        harness.wait_for_session("session-a").await;
        goal_a
    }

    async fn finish_map_a_after_lease_release(
        harness: &GuardedLoopHarness,
        goal_a: &schema::NavigationGoal,
    ) {
        harness.send_path(&sealed_path(goal_a));
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("map-a path did not enter controlling phase");
        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().session_id.is_some()

            {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("租约释放后目标未结束");
    }

    #[tokio::test]
    async fn guarded_terminal_goal_reuses_fresh_inputs_before_costmap_update() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-b")
        );
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_terminal_costmap_then_goal_reuses_fresh_pose() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_new_goal_uses_available_fresh_inputs() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.send_pose(3);
        harness.wait_for_pose(3).await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-b")
        );
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_fresh_pose_before_new_goal_is_reused() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_lease_release(&harness, &goal_a).await;

        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_changed_goal_with_same_id_replaces_and_accepts_the_next_goal() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let first = activate_map_a(&harness).await;
        let mut changed = first.clone();
        changed.goal.pose.position[0] += 0.5;
        harness.send_goal_and_wait_dequeued(&changed).await;
        tokio::time::timeout(Duration::from_millis(50), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_status_body(&frame.payload).unwrap();
                if state.goal[0] == changed.goal.pose.position[0] {
                    break;
                }
            }
        })
        .await
        .expect("变化后的同号目标应直接替换");
        assert_eq!(harness.motion_call_count("release:"), 1);
        let next = rollover_goal(&first, "session-b", 1);
        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_goal_replacement_releases_previous_before_new_acquire() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        assert_eq!(harness.motion_call_count("release:"),1);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_costmap_update_keeps_the_active_session_and_path() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        let path_a = sealed_path(&goal_a);
        harness.send_path(&path_a);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("map-a path was not accepted");

        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-a")
        );
        assert_eq!(harness.scheduler.status().path_revision, 1);

        assert!(!harness
            .motion_calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("release:")));
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_succeeded_terminal_rejects_late_path_without_losing_release_result() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        let path = sealed_path(&goal);
        harness.send_path(&path);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("path accepted before success");

        harness
            .planner_succeeds
            .store(true, std::sync::atomic::Ordering::SeqCst);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().last_terminal != Some(TerminalOutcome::Succeeded) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("scheduler success terminal");
        let terminal = harness.scheduler.status();
        assert_eq!(terminal.phase, SchedulerPhase::Idle);

        harness.send_path(&path);
        let mut wrong_identity = path.clone();
        wrong_identity.session_id.push_str("-wrong");
        schema::validate_path_envelope(&wrong_identity).expect("seal wrong identity path");
        harness.send_path(&wrong_identity);
        tokio::time::sleep(Duration::from_millis(50)).await;
        let after_late = harness.scheduler.status();
        assert_eq!(after_late.phase, SchedulerPhase::Idle);
        assert_eq!(after_late.last_terminal, Some(TerminalOutcome::Succeeded));

        harness.shutdown().await;
    }

    #[test]
    fn guarded_local_display_survives_heartbeats_and_clears_at_terminal() {
        let telemetry = GuardedNavTelemetry { public_session_id: "display-goal".into(), ..Default::default() };
        let mut status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Controlling,
            session_id: Some("display-goal".into()),
            local_path: vec![[0.0,0.0,0.4],[1.0,0.5,0.6]],
            ..Default::default()
        };
        for observation in [None, Some(false), Some(true)] {
            let wire = build_guarded_navigation_status(&status, &telemetry, observation, Duration::from_secs(1));
            let decoded = schema::decode_navigation_status(&schema::encode_navigation_status(&wire).unwrap()).unwrap();
            assert_eq!(decoded.local_path, status.local_path);
        }
        status.last_terminal = Some(nuwax_nav_runtime::TerminalOutcome::Succeeded);
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(2)).local_path.is_empty());
        status.last_terminal = None;
        status.session_id = Some("old-goal".into());
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(3)).local_path.is_empty());
        status.session_id = Some("display-goal".into());
        status.path_revision = 1;
        assert!(build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(4)).local_path.is_empty());
    }

    #[test]
    fn guarded_navigation_status_receipt_projection_is_read_only() {
        let telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            ..GuardedNavTelemetry::default()
        };
        let baseline_status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-a".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let baseline = build_guarded_navigation_status(
            &baseline_status,
            &telemetry,
            None,
            Duration::from_secs(1),
        );
        assert_eq!(baseline.pose_source_timestamp_ns, 0);
        assert_eq!(baseline.costmap_source_timestamp_ns, 0);

        let status = nuwax_nav_runtime::SchedulerStatus {
            pose_source_timestamp_ns: 11,
            costmap_source_timestamp_ns: 17,
            ..baseline_status
        };
        let projected =
            build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(1));
        assert_eq!(projected.pose_source_timestamp_ns, 11);
        assert_eq!(projected.costmap_source_timestamp_ns, 17);
        assert_eq!(
            build_guarded_navigation_status(&status, &telemetry, None, Duration::from_secs(1)),
            projected,
            "replaying the same scheduler snapshot must not advance receipt identity",
        );

        let mut without_receipts = projected.clone();
        without_receipts.pose_source_timestamp_ns = 0;
        without_receipts.costmap_source_timestamp_ns = 0;
        assert!(baseline.reason.contains("尚未收到，年龄未知"));
        assert!(projected.reason.contains("来源年龄="));
        assert_eq!(projected.message, projected.reason);
        // 来源时间也用于等待诊断；除回执和诊断文字外，状态投影保持一致。
        without_receipts.reason = baseline.reason.clone();
        without_receipts.message = baseline.message.clone();
        assert_eq!(
            without_receipts, baseline,
            "回执投影只影响来源时间与对应的输入年龄诊断"
        );

        let rejected = guarded_rejection_status(status, "path", "out-of-order candidate");
        assert_eq!(rejected.pose_source_timestamp_ns, 11);
        assert_eq!(rejected.costmap_source_timestamp_ns, 17);
    }

    #[test]
    fn guarded_input_rejection_preserves_the_real_scheduler_phase() {
        let status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-a".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let rejected = guarded_rejection_status(status, "path", "out-of-order candidate");
        assert_eq!(rejected.phase, SchedulerPhase::AwaitingPath);
        assert_eq!(rejected.message, "REJECTED path: out-of-order candidate");

        let telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            ..GuardedNavTelemetry::default()
        };
        let published =
            build_guarded_navigation_status(&rejected, &telemetry, None, Duration::from_secs(1));
        assert_eq!(published.state, 1, "rejection must not fabricate FAILED");
    }

    #[test]
    fn late_path_rejection_keeps_terminal_and_terminal_cause_and_receipts() {
        for (phase,terminal) in [
            (SchedulerPhase::Cancelling,None),(SchedulerPhase::Cancelling,None),
            (SchedulerPhase::Failed,Some(nuwax_nav_runtime::TerminalOutcome::SafetyFailed)),
            (SchedulerPhase::Idle,Some(nuwax_nav_runtime::TerminalOutcome::Succeeded)),
            (SchedulerPhase::Idle,Some(nuwax_nav_runtime::TerminalOutcome::Cancelled)),
        ] {
            let status=nuwax_nav_runtime::SchedulerStatus{phase,last_terminal:terminal,
                message:"原始停车原因及确认结果".into(),session_id:Some("original".into()),
                pose_source_timestamp_ns:17,costmap_source_timestamp_ns:23,..Default::default()};
            assert_eq!(guarded_rejection_status(status.clone(),"path","no active navigation session"),status);
        }
    }

    #[test]
    fn terminal_publication_transitions_to_unowned_guard_heartbeat() {
        let mut active = Some(active_goal("goal-a"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Idle,
            motion_sink_ready: true,

            last_terminal: Some(TerminalOutcome::Cancelled),

            message: "租约已释放".into(),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let mut telemetry = GuardedNavTelemetry {
            runtime_generation: 42,
            public_session_id: "goal-a".into(),
            motion_generation: 1,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            goal: [1.0, 2.0, 0.0],
            current: [0.5, 0.25, 0.0],
            current_z: 0.0,
            goal_z: 0.0,
            surface_navigation: false,
            initial_distance: 3.0,
            progress: 0.5,
            path_revision: 3,
            planner_runtime_generation: 22,
        };

        let terminal_message =
            build_guarded_navigation_status(&terminal, &telemetry, None, Duration::from_secs(1));
        assert_eq!(terminal_message.session_id, "goal-a");
        assert_eq!(terminal_message.planner_runtime_generation, 22);
        assert_eq!(terminal_message.runtime_generation, 42);
        assert_eq!(terminal_message.reason, terminal.message);

        clear_terminal_telemetry_after_publication(&mut telemetry, &terminal);
        assert!(active.is_some());

        let heartbeat = build_guarded_navigation_status(
            &terminal,
            &telemetry,
            Some(true),
            Duration::from_secs(2),
        );
        assert!(heartbeat.session_id.is_empty());
        assert_eq!(heartbeat.state, 0);
        assert_eq!(heartbeat.goal_status, schema::NAVIGATION_GOAL_STATUS_NONE);
        assert!(!heartbeat.input_ready);
        assert!(heartbeat.reason.contains("等待导航输入"));
        assert!(heartbeat.reason.contains("尚未收到，年龄未知"));
        assert_eq!(heartbeat.sink_ready, 1);
        assert_eq!(heartbeat.motion_generation, 0);
        assert_eq!(heartbeat.path_revision, 0);
        assert_eq!(heartbeat.planner_runtime_generation, 0);
        assert_eq!(heartbeat.runtime_generation, 42);
        assert_eq!(heartbeat.planner_map_frame, "map");
        assert_eq!(heartbeat.planner_pose_frame, "map");
        assert_eq!(heartbeat.current, [0.5, 0.25, 0.0]);
    }

    #[test]
    fn path_and_cancel_match_active_goal_identity() {
        let goal = sealed_goal("public-goal-1");
        let path = sealed_path(&goal);
        validate_path_goal_correlation(&path, &goal).expect("correlated path");

        let mut foreign_path = path.clone();
        foreign_path.session_id.push_str("-other");
        assert!(validate_path_goal_correlation(&foreign_path, &goal).is_err());

        let cancel = schema::CancelNavigation {
            session_id: goal.session_id.clone(),
            reason: 1,
        };
        validate_cancel_goal_correlation(&cancel, &goal).expect("correlated cancel");
        let mut foreign_cancel = cancel;
        foreign_cancel.session_id.push_str("-other");
        assert!(validate_cancel_goal_correlation(&foreign_cancel, &goal).is_err());
    }

    #[test]
    fn map_pose_owner_drops_bad_tf_and_processes_following_valid_odometry() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let tf = schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [1.0, 2.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        };
        let tf_body = schema::encode_tf_message_body(&tf).expect("encode TF");
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("valid TF input"),
            OwnerInput::Accepted(None)
        ));
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("重复来源时间的TF按到达顺序接收"),
            OwnerInput::Accepted(None)
        ));
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &[0x01, 0x02]).expect("bad TF is external"),
            OwnerInput::Rejected(_)
        ));

        let odometry = schema::Odometry {
            header: schema::Header {
                stamp_sec: 7,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            pose: schema::Pose {
                position: [3.0, 4.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        };
        let odometry_body = schema::encode_odometry_body(&odometry).expect("encode odometry body");
        let observed = match ingest_map_pose_odometry(&adapter, &odometry_body)
            .expect("valid odometry after bad TF")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("valid odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("valid odometry rejected: {reason}")
            }
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }

    #[test]
    fn queued_pose_uses_only_last_frame_and_preserves_stream_failure() {
        let (sender, mut poses) = guarded_test_stream();
        let make = |stamp| {
            build_body_frame(
                "/test/pose",
                "geometry_msgs.PoseStamped",
                guarded_pose_body(stamp),
            )
            .unwrap()
        };
        for stamp in 3..103 {
            sender.unbounded_send((Ok(make(stamp)), None, None)).unwrap();
        }
        sender.unbounded_send((Ok(make(2)), None, None)).unwrap();
        let mut diagnostics = InputDiagnosticLimiter::default();
        let input = drain_guarded_pose_input(Some(Ok(make(1))), &mut poses, &mut diagnostics)
            .unwrap()
            .unwrap();
        assert_eq!(input.pose.stamp.nanos, 2_000_000_000);
        drop(sender);
        assert!(
            drain_guarded_pose_input(Some(Ok(make(4))), &mut poses, &mut diagnostics)
                .is_err()
        );
    }

    #[test]
    fn queued_pose_defers_source_age_to_the_control_tick() {
        let (sender, mut poses) = guarded_test_stream();
        let current = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap();
        let mut pose = schema::PoseStamped {
            header: schema::Header {
                stamp_sec: current.as_secs() as i32,
                stamp_nsec: current.subsec_nanos() as i32,
                frame_id: "map".into(),
                ..Default::default()
            },
            pose: schema::Pose {
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                ..Default::default()
            },
        };
        let body = |pose: &schema::PoseStamped| {
            build_body_frame(
                "/test/pose",
                "geometry_msgs.PoseStamped",
                schema::encode_pose_stamped_body(pose).unwrap(),
            )
            .unwrap()
        };
        let valid = body(&pose);
        pose.header.stamp_sec += 3600;
        sender
            .unbounded_send((Ok(body(&pose)), None, None))
            .unwrap();
        let input = drain_guarded_pose_input(
            Some(Ok(valid)),
            &mut poses,
            &mut InputDiagnosticLimiter::default(),
        )
        .unwrap().unwrap();
        assert_eq!(input.pose.stamp.nanos / 1_000_000_000, i64::from(pose.header.stamp_sec));
    }

    #[test]
    fn status_publications_reset_per_goal_and_publish_success_once() {
        let mut node = TaskChannels { id: "guard-test".into(), inputs: Vec::new(), outputs: Vec::new() };
        node.outputs = vec![
            "/test/cmd".into(),
            "/test/state".into(),
            "/test/reached".into(),
        ];
        let mut publications = GuardedStatusPublications::default();
        let mut telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            motion_generation: 1,
            ..Default::default()
        };
        let mut status = nuwax_nav_runtime::SchedulerStatus {
            applied_commands: 12,
            last_applied_twist: Some(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.0,
            }),
            ..Default::default()
        };
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            2
        );
        status.last_terminal = Some(TerminalOutcome::Succeeded);
        let terminal = publications
            .frames(&node, &status, &telemetry, Some(true))
            .unwrap();
        assert_eq!(terminal.len(), 3);
        assert_eq!(terminal[1].msg_type.fqn(), "std_msgs.Bool");
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            1
        );
        clear_guarded_session_telemetry(&mut telemetry);
        assert_eq!(
            publications
                .frames(&node, &status, &telemetry, None)
                .unwrap()
                .len(),
            1
        );
        telemetry.public_session_id = "goal-b".into();
        telemetry.motion_generation = 2;
        status.last_terminal = None;

        status.applied_commands = 1;
        let next = publications
            .frames(&node, &status, &telemetry, None)
            .unwrap();
        assert_eq!(
            next.len(),
            2,
            "第二个目标首个执行回执不能被上一目标累计计数吞掉"
        );
        assert_eq!(next[0].msg_type.fqn(), "geometry_msgs.Twist");
    }

    #[test]
    fn map_pose_owner_retries_pending_odometry_when_matching_tf_arrives() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let odometry = schema::Odometry {
            header: schema::Header {
                stamp_sec: 7,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            pose: schema::Pose {
                position: [3.0, 4.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        };
        let odometry_body = schema::encode_odometry_body(&odometry).expect("encode odometry body");
        let pending = match ingest_map_pose_odometry(&adapter, &odometry_body)
            .expect("odometry before TF is retryable")
        {
            MapPoseOdometryIngress::AwaitingTransform(odometry, _) => odometry,
            MapPoseOdometryIngress::Accepted(_) => panic!("odometry unexpectedly accepted"),
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("retryable odometry rejected: {reason}")
            }
        };

        let tf_body = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [1.0, 2.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode TF");
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("matching TF"),
            OwnerInput::Accepted(None)
        ));

        let observed = match transform_map_pose_odometry(&adapter, *pending)
            .expect("retry pending odometry")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("aligned odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("aligned odometry rejected: {reason}")
            }
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }



    #[test]
    fn map_pose_owner_accepts_a_coherent_tf_chain_without_odometry_join() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let tf_body = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![
                schema::TransformStamped {
                    header: schema::Header {
                        stamp_sec: 7,
                        frame_id: "map".into(),
                        ..schema::Header::default()
                    },
                    child_frame_id: "odom".into(),
                    transform: schema::Transform {
                        translation: [1.0, 2.0, 0.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
                schema::TransformStamped {
                    header: schema::Header {
                        stamp_sec: 7,
                        frame_id: "odom".into(),
                        ..schema::Header::default()
                    },
                    child_frame_id: "base_link".into(),
                    transform: schema::Transform {
                        translation: [3.0, 4.0, 0.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
            ],
        })
        .expect("encode coherent TF");

        let observed = match ingest_map_pose_tf(&mut adapter, &tf_body).expect("coherent TF input")
        {
            OwnerInput::Accepted(Some(observed)) => observed,
            OwnerInput::Accepted(None) => panic!("coherent TF did not produce map pose"),
            OwnerInput::Rejected(reason) => panic!("coherent TF rejected: {reason}"),
        };
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.0]);
    }

    #[test]
    fn guarded_owner_drops_bad_sensor_frames_and_ignores_costmap_sequence() {
        let mut telemetry = GuardedNavTelemetry {
            map_frame: "map".into(),
            pose_frame: "map".into(),
            current: [9.0, 8.0, 0.0],
            ..GuardedNavTelemetry::default()
        };
        let before_bad_pose = telemetry.clone();
        assert!(matches!(
            decode_guarded_pose_input(&[0x7f]).expect("bad pose is external"),
            OwnerInput::Rejected(_)
        ));
        assert_eq!(telemetry, before_bad_pose);
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid pose");
        let pose_input = match decode_guarded_pose_input(&pose_body).expect("valid pose") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("valid pose rejected: {reason}"),
        };
        commit_guarded_pose_telemetry(&mut telemetry, &pose_input);
        assert_eq!(telemetry.current, [1.0, 2.0, 0.0]);

        let mut non_finite_pose =
            schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, &pose_body))
                .expect("decode pose fixture for mutation");
        non_finite_pose.pose.position[0] = f64::NAN;
        let non_finite_body =
            schema::encode_pose_stamped_body(&non_finite_pose).expect("encode non-finite pose");
        assert!(matches!(
            decode_guarded_pose_input(&non_finite_body).expect("non-finite pose is external"),
            OwnerInput::Rejected(_)
        ));

        let before_bad_costmap = telemetry.clone();
        assert!(matches!(
            decode_guarded_costmap_input(&[0x7f]).expect("bad costmap is external"),
            OwnerInput::Rejected(_)
        ));
        assert_eq!(telemetry, before_bad_costmap);
        let grid_body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 8,
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode valid costmap");
        let input = match decode_guarded_costmap_input(&grid_body).expect("valid costmap") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("valid costmap rejected: {reason}"),
        };
        commit_guarded_costmap_telemetry(&mut telemetry, &input);
        assert_eq!(telemetry.map_frame, "map");
    }

    #[test]
    fn guarded_controller_preserves_unknown_and_stops_for_observed_obstacle() {
        use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerConfig};
        use nuwax_nav_astar::navigation_map::{OCCUPIED, UNKNOWN};

        let width = 60_usize;
        let height = 20_usize;
        let mut data = vec![-1; width * height];
        data[width * height - 1] = 100;
        let grid_body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                stamp_sec: 9,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.1,
                width: width as i32,
                height: height as i32,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data,
        })
        .expect("encode unknown exploration costmap");
        let input = match decode_guarded_costmap_input(&grid_body).expect("decode costmap") {
            OwnerInput::Accepted(input) => input,
            OwnerInput::Rejected(reason) => panic!("unknown costmap rejected: {reason}"),
        };
        assert_eq!(planar_test_costmap(&input.observation).get(10, 5), UNKNOWN);
        assert_eq!(
            planar_test_costmap(&input.observation).get(width - 1, height - 1),
            OCCUPIED
        );

        let mut planner = LocalPlanner::with_config(LocalPlannerConfig::default_config(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
        planner.update_costmap(match input.observation.costmap {
            nuwax_nav_runtime::ExecutionMap::Planar(map) => map,
            _ => panic!("expected planar map"),
        });
        planner.handle_odom(PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                orientation: Quaternion::IDENTITY,
                ..Pose3::default()
            },
            NavFrameId::Map,
        ));
        planner.start_planning(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::from_unix(9, 0),
            vec![Point2::new(1.0, 0.5), Point2::new(2.0, 0.5)],
        ));

        let (_, stop) = planner.step();
        assert_eq!(stop, None, "未知格保留未知语义，不能冒充已观测障碍");
        let mut observed=schema::decode_occupancy_grid_body(&grid_body).unwrap();
        observed.data[5*width+15]=100;
        let body=schema::encode_occupancy_grid_body(&observed).unwrap();
        let input=match decode_guarded_costmap_input(&body).unwrap() {
            OwnerInput::Accepted(input)=>input,
            OwnerInput::Rejected(reason)=>panic!("实测障碍拒收：{reason}"),
        };
        if let nuwax_nav_runtime::ExecutionMap::Planar(map)=input.observation.costmap {
            planner.update_costmap(map);
        } else {panic!("二维障碍被转换为其它地图");}
        let (twist,stop)=planner.step();
        assert_eq!(stop,Some(nuwax_nav_astar::local_planner::StopMessage::ObstacleFound));
        assert_eq!(twist,Twist2::ZERO);
    }

    #[test]
    fn scheduler_task_error_wins_when_scheduler_exits_first() {
        let result = guarded_scheduler_result(
            true,
            Err(NavStackError::Internal(
                "guard scheduler status stream closed".into(),
            )),
            Ok(()),
            Err(NavStackError::Internal("motion guard root cause".into())),
        );

        assert!(result
            .expect_err("fatal scheduler task must fail")
            .to_string()
            .contains("motion guard root cause"));
    }

    #[test]
    fn scheduler_loop_error_survives_normal_task_and_shutdown() {
        let result = guarded_scheduler_result(
            true,
            Err(NavStackError::Internal("original loop error".into())),
            Ok(()),
            Ok(()),
        );

        assert!(result
            .expect_err("loop failure must survive")
            .to_string()
            .contains("original loop error"));
    }

    #[test]
    fn scheduler_loop_error_wins_when_loop_exits_first() {
        let result = guarded_scheduler_result(
            false,
            Err(NavStackError::Internal("original loop error".into())),
            Ok(()),
            Err(NavStackError::Internal("later task error".into())),
        );

        assert!(result
            .expect_err("first loop failure must survive")
            .to_string()
            .contains("original loop error"));
    }

    #[tokio::test]
    async fn scheduler_task_completion_is_recorded_before_loop_cleanup() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let (release_loop, loop_released) = oneshot::channel();
        let shutdown_called = Arc::new(AtomicBool::new(false));
        let shutdown_observer = Arc::clone(&shutdown_called);

        let result = coordinate_guarded_scheduler(
            async {
                loop_released.await.expect("task releases loop cleanup");
                Err(NavStackError::Internal(
                    "guard scheduler status stream closed".into(),
                ))
            },
            async {
                release_loop.send(()).expect("loop cleanup receiver alive");
                Err(NavStackError::Internal("motion guard root cause".into()))
            },
            move || async move {
                shutdown_observer.store(true, Ordering::SeqCst);
                Ok(())
            },
        )
        .await;

        assert!(result
            .expect_err("scheduler task failure must win")
            .to_string()
            .contains("motion guard root cause"));
        assert!(!shutdown_called.load(Ordering::SeqCst));
    }

    #[tokio::test]
    async fn scheduler_loop_completion_triggers_shutdown_and_stays_primary() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let (release_task, task_released) = oneshot::channel();
        let shutdown_called = Arc::new(AtomicBool::new(false));
        let shutdown_observer = Arc::clone(&shutdown_called);

        let result = coordinate_guarded_scheduler(
            async {
                release_task.send(()).expect("task receiver alive");
                Err(NavStackError::Internal("original loop error".into()))
            },
            async {
                task_released.await.expect("loop releases task cleanup");
                Err(NavStackError::Internal("later task error".into()))
            },
            move || async move {
                shutdown_observer.store(true, Ordering::SeqCst);
                Ok(())
            },
        )
        .await;

        assert!(result
            .expect_err("first loop failure must survive")
            .to_string()
            .contains("original loop error"));
        assert!(shutdown_called.load(Ordering::SeqCst));
    }

    #[test]
    fn retired_motion_failures_are_recoverable_but_scheduler_closure_is_fatal() {
        use nuwax_nav_runtime::RuntimeError;

        for error in [RuntimeError::SchedulerStopped] {
            assert!(runtime_input_error::<()>("guarded-test", error).is_err());
        }
        assert!(matches!(runtime_input_error::<()>("guarded-test", RuntimeError::Motion("guard unavailable".into())).unwrap(),
            OwnerInput::Rejected(reason) if reason.contains("guard unavailable")));
        assert!(matches!(
            runtime_input_error::<()>("guarded-test", RuntimeError::StaleInput("old pose".into()),)
                .expect("stale external input is recoverable"),
            OwnerInput::Rejected(_)
        ));

        for component in [
            "map_pose_adapter.tf",
            "guarded_nav_scheduler.goal",
            "simple_planner.map_pose",
            "astar_planner.costmap",
        ] {
            assert!(required_frame(None, component).is_err(), "{component}");
        }
    }

    #[test]
    fn external_input_diagnostics_are_rate_limited() {
        let start = Instant::now();
        let mut limiter = InputDiagnosticLimiter::default();
        assert_eq!(limiter.record_at(start), Some(0));
        assert_eq!(limiter.record_at(start + Duration::from_millis(10)), None);
        assert_eq!(limiter.record_at(start + Duration::from_millis(20)), None);
        assert_eq!(
            limiter.record_at(start + INPUT_DIAGNOSTIC_INTERVAL),
            Some(2)
        );
    }
}

#[cfg(all(test, feature = "replanning-astar"))]
mod tests {
    use super::*;

    #[tokio::test]
    async fn astar_real_owner_retains_goal_costmap_until_pose_and_publishes_exact_path() {
        static NEXT_ASTAR_OWNER_ID: std::sync::atomic::AtomicU64 =
            std::sync::atomic::AtomicU64::new(0);
        let id = NEXT_ASTAR_OWNER_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let prefix = format!("/test/astar-cold/{}/{id}", std::process::id());
        let node = TaskChannels { id: format!("astar-cold-owner-{id}"), inputs: vec![format!("{prefix}/pose"),format!("{prefix}/costmap"),format!("{prefix}/goal"),format!("{prefix}/stop")], outputs: vec![format!("{prefix}/waypoint"),format!("{prefix}/path"),format!("{prefix}/reached"),format!("{prefix}/failed")] };
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let mut paths = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.PathEnvelope")
            .await
            .expect("path subscription");
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let owner = tokio::spawn(run_astar_planner_with_transport(node.clone(), Arc::from("base_link"), transport.clone(), shutdown_rx, ready_tx));
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                match ready_rx.try_recv() {
                    Ok(Ok(())) => break,
                    Ok(Err(reason)) => panic!("A* owner startup failed: {reason}"),
                    Err(std_mpsc::TryRecvError::Empty) => {
                        assert!(!owner.is_finished(), "A* owner exited before ready");
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    Err(std_mpsc::TryRecvError::Disconnected) => {
                        panic!("A* owner dropped readiness channel")
                    }
                }
            }
        })
        .await
        .expect("A* owner readiness timeout");

        let goal = planner_test_goal("astar-cold-22", 1);
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationGoal",
            schema::encode_navigation_goal_body(&goal).expect("encode goal"),
        )
        .await
        .expect("publish goal first");
        publish_body(
            &transport,
            &node.inputs[1],
            "nav_msgs.OccupancyGrid",
            planner_test_grid_body(22),
        )
        .await
        .expect("publish costmap second");
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(
            !owner.is_finished(),
            "recoverable missing pose must not terminate the A* owner"
        );

        let pose = schema::PoseStamped {
            header: schema::Header {
                seq: 22,
                stamp_sec: 22,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        };
        publish_body(
            &transport,
            &node.inputs[0],
            "geometry_msgs.PoseStamped",
            schema::encode_pose_stamped_body(&pose).expect("encode pose"),
        )
        .await
        .expect("publish pose third");

        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("A* owner did not publish after the late pose")
            .expect("path stream closed")
            .expect("path transport error");
        let path = schema::decode_path_envelope_body(&frame.payload).expect("decode path envelope");
        assert_eq!(path.session_id, goal.session_id);
        assert_eq!(path.path_revision, 1);
        assert_eq!(path.goal_key(), goal.goal_key());

        shutdown_tx.send(()).expect("A* owner shutdown receiver");
        tokio::time::timeout(Duration::from_secs(3), owner)
            .await
            .expect("A* owner shutdown timeout")
            .expect("A* owner join")
            .expect("A* owner result");
    }

    fn grid_body(frame: &str, orientation_xyzw: [f64; 4]) -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                frame_id: frame.into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.1,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw,
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode test grid")
    }

    fn path_body(path_frame: &str, pose_frame: &str) -> Vec<u8> {
        let header = schema::Header {
            stamp_sec: 7,
            stamp_nsec: 8,
            frame_id: path_frame.into(),
            ..schema::Header::default()
        };
        schema::encode_path_body(&schema::Path {
            header,
            poses: vec![schema::PoseStamped {
                header: schema::Header {
                    frame_id: pose_frame.into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode test path")
    }

    #[test]
    fn astar_costmap_rejects_non_map_frame() {
        let error = decode_astar_costmap(&grid_body("odom", [0.0, 0.0, 0.0, 1.0]))
            .expect_err("frame mismatch must fail");
        assert!(error.to_string().contains("frame must be map"));
    }

    #[test]
    fn astar_costmap_rejects_unrepresentable_origin_rotation() {
        let error = decode_astar_costmap(&grid_body("map", [0.0, 0.0, 0.1, 0.995]))
            .expect_err("rotated origin must fail");
        assert!(error.to_string().contains("rotated or non-planar"));
    }

    #[test]
    fn path_decode_preserves_header_frame_and_stamp() {
        let path = decode_path_body(&path_body("warehouse", "warehouse")).expect("decode path");
        assert_eq!(path.frame_id, NavFrameId::Custom("warehouse".into()));
        assert_eq!(path.stamp, Timestamp::from_unix(7, 8));
        assert_eq!(path.points, vec![Point2::new(1.0, 2.0)]);
    }

    #[test]
    fn path_decode_rejects_pose_frame_mismatch() {
        let error = decode_path_body(&path_body("map", "odom"))
            .expect_err("path pose/header mismatch must fail");
        assert!(error.to_string().contains("Path pose frame does not match"));
    }

    #[test]
    fn astar_goal_arriving_before_costmap_is_applied_once_with_full_envelope() {
        use nuwax_nav_astar::module::{Module, NavigationInterface, ReplanningAStarPlannerModule};

        let goal = planner_test_goal("astar-goal", 5);
        let expected_key = goal.goal_key();
        let expected_boundary = goal.boundary_points.clone();
        let ready = decode_navigation_goal_input(&schema::encode_navigation_goal_body(&goal).unwrap()).unwrap();
        assert_eq!(ready.goal_key(), expected_key);
        assert_eq!(ready.boundary_points, expected_boundary);

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(22, 0);
        Module::on_odom(&mut planner, robot_pose).expect("A* map pose");
        let (applied, boundary, resolution) =
            match apply_astar_planner_goal(&mut planner, ready, &planner_test_grid_body(22))
                .expect("apply A* buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid A* goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert_eq!(resolution, 1.0);
        let _ = Module::tick(&mut planner).expect("A* planner tick");
    }

    #[test]
    fn astar_new_goal_ignores_map_metadata_and_builds_correlated_path() {
        use nuwax_nav_astar::module::{
            Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
        };

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                orientation: nuwax_nav_types_messages::Quaternion::IDENTITY,
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(12, 0);
        Module::on_odom(&mut planner, robot_pose).expect("A* map pose");

        Module::on_costmap(
            &mut planner,
            decode_astar_costmap(&planner_test_grid_body(12)).expect("map 12"),
        )
        .expect("apply map 12");
        let goal_12 = planner_test_goal("astar-12", 1);
        let ready_12 = goal_12.clone();
        let (applied_12, boundary_12, _) =
            match apply_astar_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert_eq!(applied_12, goal_12);
        assert!(boundary_12.is_some());
        let goal_13 = planner_test_goal("astar-13", 2);
        let ready_13 = goal_13.clone();
        let (applied_13, boundary_13, _) =
            match apply_astar_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        let active_goal = Some(applied_13);
        let active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("astar-13")
        );
        assert!(active_boundary.is_some());
        let update = match Module::tick(&mut planner).expect("A* map-b tick") {
            ModuleOutput::Navigation(update) | ModuleOutput::SafetyHold(update) => update,
        };
        let path = update.path.expect("A* map-b path");
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 29,
                path_revision: 1,
                path: &path,
                boundary_safe: path_respects_navigation_boundary(
                    &path,
                    active_boundary.as_ref(),
                    0.5,
                ),
            },
            13_000_000_000,
        )
        .expect("build A* map-b PathEnvelope")
        .unwrap_or_else(|| {
            let map = decode_astar_costmap_with_boundary(&planner_test_grid_body(12), active_boundary.as_ref()).unwrap();
            let raw = nuwax_nav_astar::astar::min_cost_astar(&map, Point2::new(0.5, 0.5), Point2::new(4.5, 4.5),
                &nuwax_nav_astar::astar::AStarOptions::default());
            panic!("A* map-b path stays inside boundary: path={path:?}, boundary={active_boundary:?}, raw_astar={raw:?}");
        });
        assert_eq!(
            envelope.goal_key(),
            active_goal.as_ref().unwrap().goal_key()
        );
    }

    #[test]
    fn astar_owner_drops_bad_pose_and_grid_then_accepts_new_valid_inputs() {
        use nuwax_nav_astar::module::{Module, NavigationInterface, ReplanningAStarPlannerModule};

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        assert!(matches!(
            decode_planner_pose_input(&[0x55], "astar_planner.map_pose")
                .expect("bad pose is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 31,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid A* pose");
        let pose = match decode_planner_pose_input(&pose_body, "astar_planner.map_pose")
            .expect("valid A* pose")
        {
            OwnerInput::Accepted(pose) => pose,
            OwnerInput::Rejected(reason) => panic!("valid A* pose rejected: {reason}"),
        };
        Module::on_odom(&mut planner, pose).expect("A* planner accepts valid pose");

        assert!(matches!(
            decode_planner_grid_input(&[0x55], "astar_planner.costmap")
                .expect("bad grid is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let grid_body = planner_test_grid_body(31);
        assert!(matches!(
            decode_planner_grid_input(&grid_body, "astar_planner.costmap").expect("valid A* grid"),
            OwnerInput::Accepted(_)
        ));
        let costmap = decode_astar_costmap(&grid_body).expect("decode valid A* grid");
        Module::on_costmap(&mut planner, costmap).expect("A* planner accepts valid grid");
    }
}

#[cfg(all(test, feature = "simple-2d", feature = "replanning-astar"))]
mod boundary_tests {
    use super::*;
    use nuwax_nav_astar::navigation_map::OCCUPIED;

    fn free_grid_body() -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 17,
                stamp_sec: 1,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 7,
                height: 7,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 49],
        })
        .expect("encode free grid")
    }

    fn concave_boundary_points() -> Vec<[f64; 3]> {
        vec![
            [0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0],
            [6.0, 6.0, 0.0],
            [4.0, 6.0, 0.0],
            [4.0, 2.0, 0.0],
            [2.0, 2.0, 0.0],
            [2.0, 6.0, 0.0],
            [0.0, 6.0, 0.0],
            [0.0, 0.0, 0.0],
        ]
    }

    fn goal(owner_kind: i32, boundary_points: Vec<[f64; 3]>) -> schema::NavigationGoal {
        schema::NavigationGoal {
boundary_points: boundary_points,
goal: schema::PoseStamped {
                header: schema::Header {
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [5.5, 5.5, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
}
    }

    #[test]
    fn concave_boundary_rejects_shortcut_path_with_inside_endpoints() {
        let goal = goal(
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            concave_boundary_points(),
        );
        let boundary = navigation_boundary_from_goal(&goal)
            .expect("valid boundary")
            .expect("non-empty boundary");
        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(1.5, 5.5), Point2::new(5.5, 5.5)],
        );

        assert!(boundary.contains(Point2::new(1.5, 5.5)));
        assert!(boundary.contains(Point2::new(5.5, 5.5)));
        assert!(!path_respects_navigation_boundary(
            &path,
            Some(&boundary),
            0.5,
        ));

        let map = decode_astar_costmap_with_boundary(&free_grid_body(), Some(&boundary))
            .expect("masked A* map");
        assert_eq!(map.get(3, 5), OCCUPIED, "concave notch must be masked");
        assert_eq!(map.get(1, 5), 0, "left arm remains navigable");

        let grid = decode_simple_occupancy_with_boundary(&free_grid_body(), Some(&boundary))
            .expect("masked simple occupancy");
        assert_eq!(
            grid.get(3, 5),
            Some(nuwax_nav_types_messages::OccupancyCell::Occupied)
        );
    }

    #[test]
    fn boundary_does_not_cross_goal_lease_reuse() {
        let exploration = goal(
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            concave_boundary_points(),
        );
        let exploration_boundary = navigation_boundary_from_goal(&exploration)
            .expect("valid exploration boundary")
            .expect("non-empty exploration boundary");
        let masked =
            decode_astar_costmap_with_boundary(&free_grid_body(), Some(&exploration_boundary))
                .expect("masked exploration map");
        assert_eq!(masked.get(3, 5), OCCUPIED);

        let public = goal(schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV, Vec::new());
        let public_boundary = navigation_boundary_from_goal(&public).expect("public goal");
        assert!(public_boundary.is_none());
        let restored =
            decode_astar_costmap_with_boundary(&free_grid_body(), public_boundary.as_ref())
                .expect("unmasked public map");
        assert_eq!(restored.get(3, 5), 0, "old exploration mask must not leak");
    }

    #[test]
    fn public_nav_is_not_constrained_by_a_boundary_mask() {
        let public = goal(schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV, Vec::new());
        let boundary = navigation_boundary_from_goal(&public).expect("public goal");
        assert!(boundary.is_none());
        let map = decode_astar_costmap_with_boundary(&free_grid_body(), boundary.as_ref())
            .expect("public map");
        assert!((0..map.height()).all(|y| (0..map.width()).all(|x| map.get(x, y) == 0)));

        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(0.5, 0.5), Point2::new(6.5, 6.5)],
        );
        assert!(path_respects_navigation_boundary(
            &path,
            boundary.as_ref(),
            0.5,
        ));
    }
}

#[cfg(all(
    test,
    any(feature = "cmu-native", feature = "explore")
))]
mod native_adapter_tests {
    static NATIVE_GENERATION_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    use super::*;

    fn goal() -> schema::NavigationGoal {
        let mut goal = schema::NavigationGoal {
session_id: "native-session".into(),
boundary_points: vec![
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 5.0, 0.0],
                [0.0, 5.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
motion_generation: 8,
goal: schema::PoseStamped {
                header: schema::Header {
                    seq: 9,
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [4.0, 4.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
};

        schema::validate_navigation_goal(&goal).expect("seal native goal");
        goal
    }

    fn path(points: Vec<Point2>) -> Path2D {
        Path2D::stamped(NavFrameId::Map, Timestamp::from_unix(7, 0), points)
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn goal_generation(generation: i64) -> schema::NavigationGoal {
        let mut value = goal();
        value.motion_generation = generation as u64;
        value.goal.header.seq = i32::try_from(generation).expect("test generation fits i32");
        value.goal.header.stamp_sec = i32::try_from(generation).expect("test generation fits i32");
        schema::validate_navigation_goal(&value).expect("reseal native goal generation");
        value
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn far_key(goal: &schema::NavigationGoal) -> FarJoinKey {
        FarJoinKey {
            backend: "far-visibility-graph".into(),
            runtime_generation: 12,
            session_id: goal.session_id.clone(),
            motion_generation: goal.motion_generation,
            path_revision: 4,
            planner_runtime_generation: 12,
        }
    }

    #[cfg(all(
        feature = "native-runner",
        any(feature = "cmu-native", feature = "explore")
    ))]



    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn duplicate_goal_preserves_far_join_and_consumed_state() {
        let goal = goal_generation(9);
        let key = far_key(&goal);
        let mut latest = None;
        assert!(admit_native_adapter_goal(&mut latest, goal.clone()));
        let mut pending_results = HashMap::from([(
            key.clone(),
            path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]),
        )]);
        let mut pending_terminals = HashSet::new();
        let mut consumed = HashSet::new();

        assert!(!admit_native_adapter_goal(&mut latest, goal));
        pending_terminals.insert(key.clone());
        let mut published = usize::from(
            take_far_join(
                &key,
                &mut pending_results,
                &mut pending_terminals,
                &mut consumed,
            )
            .is_some(),
        );
        pending_terminals.insert(key.clone());
        published += usize::from(
            take_far_join(
                &key,
                &mut pending_results,
                &mut pending_terminals,
                &mut consumed,
            )
            .is_some(),
        );
        assert_eq!(published, 1);
        assert!(consumed.contains(&key));
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn same_motion_generation_keeps_distinct_far_goal_identity() {
        let first=goal_generation(9);
        let mut current=first.clone();current.session_id="next-child".into();
        let old_key=far_key(&first);let key=far_key(&current);
        let mut latest=None;
        assert!(admit_native_adapter_goal(&mut latest,first));
        assert!(admit_native_adapter_goal(&mut latest,current.clone()));
        assert!(!far_key_matches_goal(&old_key,&current,12));
        assert!(far_key_matches_goal(&key,&current,12));
        let mut results=HashMap::from([(key.clone(),path(vec![Point2::new(1.,1.),Point2::new(4.,4.)]))]);
        let mut terminals=HashSet::from([old_key.clone()]);let mut consumed=HashSet::new();
        assert!(take_far_join(&old_key,&mut results,&mut terminals,&mut consumed).is_none());
        terminals.insert(key.clone());
        assert!(take_far_join(&key,&mut results,&mut terminals,&mut consumed).is_some());
        assert_eq!(latest,Some(current));
    }

    #[test]
    fn native_path_gate_rejects_wrong_map_boundary_and_frame() {
        let goal = goal();
        let inside = path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]);
        assert!(native_path_respects_goal(&inside, &goal));

        let outside = path(vec![Point2::new(1.0, 1.0), Point2::new(6.0, 4.0)]);
        assert!(!native_path_respects_goal(&outside, &goal));

        let wrong_path_frame = Path2D::stamped(
            NavFrameId::Odom,
            Timestamp::from_unix(7, 0),
            inside.points.clone(),
        );
        assert!(!native_path_respects_goal(&wrong_path_frame, &goal));

        let mut wrong_map = goal.clone();
        wrong_map.goal.header.frame_id = "odom".into();
        assert!(!native_path_respects_goal(&inside, &wrong_map));

    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_terminal_gate_rejects_old_future_and_wrong_goal_identity() {
        let goal = goal();
        let state = schema::NavigationStatus {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            motion_generation: goal.motion_generation as i64,
            path_revision: 4,
            planner_runtime_generation: 12,
            ..schema::NavigationStatus::default()
        };
        assert!(far_terminal_matches_goal(&state, &goal, 12));
        assert!(!far_terminal_matches_goal(&state, &goal, 11));
        assert!(!far_terminal_matches_goal(&state, &goal, 13));

        let mut wrong_goal = goal;
        wrong_goal.motion_generation += 1;
        schema::validate_navigation_goal(&wrong_goal).expect("reseal wrong goal");
        assert!(!far_terminal_matches_goal(&state, &wrong_goal, 12));
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_join_wrong_key_does_not_consume_matching_pending_result() {
        let goal = goal();
        let good = FarJoinKey {
            backend: "far-visibility-graph".into(),
            runtime_generation: 12,
            session_id: goal.session_id.clone(),
            motion_generation: goal.motion_generation,
            path_revision: 4,
            planner_runtime_generation: 12,
        };
        let mut pending_results = HashMap::from([(
            good.clone(),
            path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]),
        )]);
        let mut pending_terminals = HashSet::new();
        let mut consumed = HashSet::new();

        let mut wrong_revision = good.clone();
        wrong_revision.path_revision += 1;
        pending_terminals.insert(wrong_revision.clone());
        assert!(take_far_join(
            &wrong_revision,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
        assert!(pending_results.contains_key(&good));

        let mut wrong_backend = good.clone();
        wrong_backend.backend = "another-backend".into();
        pending_terminals.insert(wrong_backend.clone());
        assert!(take_far_join(
            &wrong_backend,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
        assert!(pending_results.contains_key(&good));

        pending_terminals.insert(good.clone());
        assert!(take_far_join(
            &good,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_some());
        assert!(take_far_join(
            &good,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[tokio::test]
    async fn far_real_lcm_adapter_pairs_ready_path_and_terminal_without_crossing_headers() {
        static NEXT_ADAPTER_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let id = NEXT_ADAPTER_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let prefix = format!("/t/fa/{}/{id}", std::process::id());
        let node = TaskChannels { id: format!("far-adapter-{id}"), inputs: vec![format!("{prefix}/g"),format!("{prefix}/e"),format!("{prefix}/p")], outputs: vec![format!("{prefix}/o")] };
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let mut paths = typed_stream(&transport, &node.outputs[0], "nuwax_msgs.PathEnvelope")
            .await
            .expect("canonical path subscription");
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let owner = tokio::spawn(run_native_planner_path_adapter_with_transport(
            node.clone(),
            Arc::from("base_link"),
            transport.clone(),
            shutdown_rx,
            ready_tx,
        ));
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                match ready_rx.try_recv() {
                    Ok(Ok(())) => break,
                    Ok(Err(reason)) => panic!("FAR adapter startup failed: {reason}"),
                    Err(std_mpsc::TryRecvError::Empty) => {
                        assert!(!owner.is_finished(), "FAR adapter exited before ready");
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    Err(std_mpsc::TryRecvError::Disconnected) => {
                        panic!("FAR adapter dropped readiness channel")
                    }
                }
            }
        })
        .await
        .expect("FAR adapter readiness timeout");

        let mut goal = goal();
        goal.motion_generation=u64::MAX;
        publish_body(
            &transport,
            &node.inputs[0],
            "nuwax_msgs.NavigationGoal",
            schema::encode_navigation_goal_body(&goal).expect("encode FAR goal"),
        )
        .await
        .expect("publish FAR goal");
        FAR_READY_GENERATION.store(12, Ordering::Release);

        let state = |revision| schema::NavigationStatus {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            planner_map_frame: goal.goal.header.frame_id.clone(),
            planner_pose_frame: goal.goal.header.frame_id.clone(),
            motion_generation: goal.motion_generation as i64,
            path_revision: revision,
            planner_runtime_generation: 12,
            ..schema::NavigationStatus::default()
        };

        publish_body(
            &transport,
            &node.inputs[2],
            "nav_msgs.Path",
            schema::encode_path_body(&schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            })
            .expect("encode legacy FAR path"),
        )
        .await
        .expect("publish legacy FAR path");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "legacy raw FAR path must not become canonical"
        );

        let mut wrong_header = goal.goal.header.clone();
        wrong_header.frame_id = "odom".into();
        let wrong_result = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(1),
            &schema::Path {
                header: wrong_header.clone(),
                poses: vec![schema::PoseStamped {
                    header: wrong_header,
                    pose: goal.goal.pose.clone(),
                }],
            },
        )
        .expect("seal wrong-header FAR result");
        let mut wrong_terminal = wrong_result.clone();
        wrong_terminal.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&wrong_result).expect("encode wrong FAR result"),
        )
        .await
        .expect("publish wrong FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&wrong_terminal)
                .expect("encode wrong FAR terminal"),
        )
        .await
        .expect("publish wrong FAR terminal");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "wrong-header FAR result must not become canonical"
        );

        let accepted_result = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(2),
            &schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            },
        )
        .expect("seal accepted FAR result");
        let mut accepted_terminal = accepted_result.clone();
        accepted_terminal.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&accepted_terminal)
                .expect("encode accepted FAR terminal"),
        )
        .await
        .expect("publish accepted FAR terminal first");
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&accepted_result)
                .expect("encode accepted FAR result"),
        )
        .await
        .expect("publish accepted FAR result second");
        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("FAR canonical path timeout")
            .expect("FAR canonical path stream closed")
            .expect("FAR canonical path transport");
        let envelope = schema::decode_path_envelope_body(&frame.payload)
            .expect("decode FAR canonical PathEnvelope");
        assert_eq!(envelope.goal_key(), goal.goal_key());
        assert_eq!(envelope.planner_runtime_generation, 12);
        assert_eq!(envelope.path_revision, 2);

        let result_first = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(3),
            &schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            },
        )
        .expect("seal result-first FAR result");
        let mut terminal_second = result_first.clone();
        terminal_second.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&result_first)
                .expect("encode result-first FAR result"),
        )
        .await
        .expect("publish FAR result first");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&terminal_second)
                .expect("encode terminal-second FAR event"),
        )
        .await
        .expect("publish FAR terminal second");
        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("result-first FAR canonical path timeout")
            .expect("FAR canonical path stream closed")
            .expect("FAR canonical path transport");
        let envelope = schema::decode_path_envelope_body(&frame.payload)
            .expect("decode result-first FAR canonical PathEnvelope");
        assert_eq!(envelope.path_revision, 3);

        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&result_first).expect("encode replay FAR result"),
        )
        .await
        .expect("publish replay FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&terminal_second)
                .expect("encode replay FAR terminal"),
        )
        .await
        .expect("publish replay FAR terminal");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "replayed FAR pair must publish at most once"
        );

        shutdown_tx.send(()).expect("shutdown FAR adapter");
        owner
            .await
            .expect("join FAR adapter")
            .expect("FAR adapter shutdown");
    }

    #[test]
    fn restart_generation_is_cleared_before_a_new_ready_generation() {
        let _generation_guard=NATIVE_GENERATION_TEST_LOCK.lock().unwrap();
        #[cfg(any(feature = "cmu-native", feature = "explore"))]
        {
            FAR_READY_GENERATION.store(12, Ordering::Release);
            assert_eq!(native_ready_generation(), 12);
            invalidate_native_planner_generation("far_planner");
            assert_eq!(native_ready_generation(), 0);
            FAR_READY_GENERATION.store(13, Ordering::Release);
            assert_eq!(native_ready_generation(), 13);
        }

    }
}

#[cfg(all(test, feature = "guard-runtime"))]
#[test]
fn navigation_status_reports_one_input_ready_signal_and_typed_goal_reason() {
    let mut status = nuwax_nav_runtime::SchedulerStatus {
        phase: nuwax_nav_runtime::SchedulerPhase::AwaitingPath,
        motion_sink_ready: true, planner_inputs_fresh: true,
        message: "正在等待可通行路线".into(), no_feasible_path: true,
        remaining_distance_m: 8.0, ..Default::default()
    };
    let telemetry = GuardedNavTelemetry { public_session_id: "目标".into(), map_frame: "map".into(), pose_frame: "map".into(), ..Default::default() };
    let ready = build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO);
    assert!(ready.input_ready);
    assert_eq!(ready.goal_status, schema::NAVIGATION_GOAL_STATUS_PLANNING);
    assert_eq!(ready.reason, "正在等待可通行路线");
    assert!(ready.no_feasible_path);
    assert_eq!(ready.remaining_distance_m, 8.0);
    status.planner_inputs_fresh = false;
    status.input_wait_reason = Some("recent obstacle source revoked".into());
    let waiting = build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO);
    assert!(!waiting.input_ready);
    assert!(waiting.reason.contains("recent obstacle source revoked"));
    assert!(waiting.reason.contains("尚未收到，年龄未知"));
    status.last_terminal = Some(nuwax_nav_runtime::TerminalOutcome::Stopped);
    assert_eq!(build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO).goal_status, schema::NAVIGATION_GOAL_STATUS_STOPPED);
    assert_eq!(build_guarded_navigation_status(&status, &telemetry, None, Duration::ZERO).reason, status.message);
}
