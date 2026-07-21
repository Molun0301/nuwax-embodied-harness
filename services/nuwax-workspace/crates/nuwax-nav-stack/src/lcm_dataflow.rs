//! Real-LCM adapters for in-process navigation graph nodes.

use std::future::Future;
use std::pin::Pin;
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;
#[cfg(feature = "guard-runtime")]
use std::time::Instant;

#[cfg(feature = "movement-manager")]
use futures::future::pending;
use futures::StreamExt;
use nuwax_codec::lcm::schema;
#[cfg(feature = "patrolling")]
use nuwax_nav_types_messages::GridSnapshot;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "explore"
))]
use nuwax_nav_types_messages::Point2;
#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
use nuwax_nav_types_messages::PointCloud;
#[cfg(any(feature = "simple-2d", feature = "movement-manager"))]
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
#[cfg(feature = "movement-manager")]
use nuwax_nav_types_messages::Twist3;
use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, Quaternion, Timestamp};
#[cfg(feature = "explore")]
use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling"
))]
use nuwax_nav_types_messages::{Path2D, Twist2};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_transport::{LcmFrameStream, LcmTransport, TransportResult};
use tokio::sync::oneshot;

use crate::error::{NavStackError, NavStackResult};
use crate::graph::ModuleNode;
use crate::runner::InProcessNode;

const DEFAULT_LCM_URL: &str = "udpm://239.255.76.67:7667?ttl=1";
const STARTUP_TIMEOUT: Duration = Duration::from_secs(10);
#[cfg(feature = "patrolling")]
const PLANNER_CONTROL_TIMEOUT: Duration = Duration::from_secs(2);

type ReadySender = std_mpsc::SyncSender<Result<(), String>>;
type OwnerFuture = Pin<Box<dyn Future<Output = NavStackResult<()>> + Send>>;
type OwnerRun = Box<dyn FnOnce(oneshot::Receiver<()>, ReadySender) -> OwnerFuture + Send + 'static>;

struct LcmThreadOwner {
    label: String,
    run: Option<OwnerRun>,
    shutdown: Option<oneshot::Sender<()>>,
    thread: Option<JoinHandle<()>>,
    failure: Arc<Mutex<Option<String>>>,
}

impl LcmThreadOwner {
    fn new(label: impl Into<String>, run: OwnerRun) -> Self {
        Self {
            label: label.into(),
            run: Some(run),
            shutdown: None,
            thread: None,
            failure: Arc::new(Mutex::new(None)),
        }
    }
}

impl InProcessNode for LcmThreadOwner {
    fn start(&mut self) -> NavStackResult<()> {
        if self.thread.is_some() {
            return Ok(());
        }
        let run = self.run.take().ok_or_else(|| {
            NavStackError::Internal(format!("{} cannot be restarted", self.label))
        })?;
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let ready_fallback = ready_tx.clone();
        let failure = Arc::clone(&self.failure);
        let label = self.label.clone();
        let thread = std::thread::Builder::new()
            .name(format!("nav-stack-{label}"))
            .spawn(move || {
                let result = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .map_err(|error| {
                        NavStackError::Internal(format!(
                            "{label} failed to create Tokio runtime: {error}"
                        ))
                    })
                    .and_then(|runtime| runtime.block_on(run(shutdown_rx, ready_tx)));
                if let Err(error) = result {
                    let detail = error.to_string();
                    let _ = ready_fallback.send(Err(detail.clone()));
                    if let Ok(mut slot) = failure.lock() {
                        *slot = Some(detail);
                    }
                }
            })
            .map_err(|error| {
                NavStackError::Internal(format!("{} thread spawn failed: {error}", self.label))
            })?;
        self.shutdown = Some(shutdown_tx);
        self.thread = Some(thread);

        match ready_rx.recv_timeout(STARTUP_TIMEOUT) {
            Ok(Ok(())) => Ok(()),
            Ok(Err(reason)) => {
                self.join_after_failed_start();
                Err(NavStackError::Internal(reason))
            }
            Err(error) => {
                self.join_after_failed_start();
                Err(NavStackError::Internal(format!(
                    "{} startup handshake failed: {error}",
                    self.label
                )))
            }
        }
    }

    fn is_running(&mut self) -> NavStackResult<bool> {
        if let Some(reason) = self
            .failure
            .lock()
            .map_err(|_| NavStackError::Internal("LCM owner health mutex poisoned".into()))?
            .clone()
        {
            return Err(NavStackError::Internal(reason));
        }
        Ok(self
            .thread
            .as_ref()
            .is_some_and(|thread| !thread.is_finished()))
    }

    fn shutdown(&mut self) -> NavStackResult<()> {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.thread.take() {
            thread.join().map_err(|_| {
                NavStackError::Internal(format!("{} runtime thread panicked", self.label))
            })?;
        }
        Ok(())
    }
}

impl LcmThreadOwner {
    fn join_after_failed_start(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn boxed_owner(label: &'static str, run: OwnerRun) -> Box<dyn InProcessNode> {
    Box::new(LcmThreadOwner::new(label, run))
}

fn lcm_transport() -> NavStackResult<LcmTransport> {
    let provider = std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| DEFAULT_LCM_URL.into());
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
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

async fn publish_body(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
    body: Vec<u8>,
) -> NavStackResult<()> {
    let topic =
        protocol_topic(channel).map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(type_name)
        .map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    transport
        .publish(
            RawLcmFrameBuilder::new(topic, message_type)
                .payload(body)
                .build(),
        )
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
            eprintln!(
                "[nuwax-nav-stack] {component}: rejected external input: {reason} (suppressed={suppressed})"
            );
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
        RuntimeError::Motion(_)
            | RuntimeError::InvalidApplied(_)
            | RuntimeError::InvalidFence(_)
            | RuntimeError::SchedulerStopped
            | RuntimeError::SequenceExhausted
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

#[cfg(feature = "movement-manager")]
async fn optional_next(
    stream: &mut Option<LcmFrameStream>,
) -> Option<TransportResult<RawLcmFrame>> {
    match stream {
        Some(stream) => stream.next().await,
        None => pending().await,
    }
}

fn signal_ready(ready: ReadySender) -> NavStackResult<()> {
    ready
        .send(Ok(()))
        .map_err(|_| NavStackError::Internal("startup handshake receiver dropped".into()))
}

fn validate_ports(node: &ModuleNode, inputs: usize, outputs: &[usize]) -> NavStackResult<()> {
    if node.inputs.len() != inputs || !outputs.contains(&node.outputs.len()) {
        return Err(NavStackError::InvalidConfig(format!(
            "{} requires {inputs} inputs and one of {outputs:?} outputs, got {} and {}",
            node.id,
            node.inputs.len(),
            node.outputs.len()
        )));
    }
    if node
        .inputs
        .iter()
        .chain(&node.outputs)
        .any(String::is_empty)
    {
        return Err(NavStackError::InvalidConfig(format!(
            "{} contains an empty LCM topic",
            node.id
        )));
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
pub(crate) fn map_pose_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 2, &[1])?;
    let node = node.clone();
    Ok(boxed_owner(
        "map-pose-adapter",
        Box::new(move |shutdown, ready| Box::pin(run_map_pose_adapter(node, shutdown, ready))),
    ))
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
) -> OwnerInputResult<()> {
    let message = match schema::decode_tf_message_body(payload) {
        Ok(message) => message,
        Err(error) => return Ok(OwnerInput::Rejected(format!("TFMessage decode: {error}"))),
    };
    match adapter.observe_tf(&message) {
        Ok(true) => Ok(OwnerInput::Accepted(())),
        Ok(false) => Ok(OwnerInput::Rejected(
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
        Err(
            error @ (crate::map_pose::MapPoseError::StaleTransform { .. }
            | crate::map_pose::MapPoseError::MisalignedTransform { .. }),
        ) => Ok(MapPoseOdometryIngress::AwaitingTransform(
            Box::new(odometry),
            error.to_string(),
        )),
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
        .map_err(|error| NavStackError::Internal(format!("map pose encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await
}

#[cfg(feature = "guard-runtime")]
async fn run_map_pose_adapter(
    node: ModuleNode,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
    let mut transforms = typed_stream(&transport, &node.inputs[1], "tf2_msgs.TFMessage").await?;
    let mut adapter = crate::map_pose::MapPoseAdapter::new(
        configured_frame("NUWAX_GEOMETRY_MAP_FRAME", "map")?,
        configured_frame("NUWAX_GEOMETRY_ODOM_FRAME", "odom")?,
        configured_frame("NUWAX_GEOMETRY_BASE_FRAME", "base_link")?,
    )
    .map_err(|error| module_error("map_pose_adapter", error))?;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut pending_odometry: Option<Box<schema::Odometry>> = None;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => return Ok(()),
            message = transforms.next() => {
                let frame = required_frame(message, "map_pose_adapter.tf")?;
                match ingest_map_pose_tf(&mut adapter, &frame.payload)? {
                    OwnerInput::Accepted(()) => {
                        if let Some(odometry) = pending_odometry.take() {
                            match transform_map_pose_odometry(&adapter, *odometry)? {
                                MapPoseOdometryIngress::Accepted(observed) => {
                                    publish_map_pose_observation(
                                        &transport,
                                        &node.outputs[0],
                                        observed,
                                    ).await?;
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
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("map_pose_adapter.tf", &reason);
                    }
                }
            }
            message = odometry.next() => {
                let frame = required_frame(message, "map_pose_adapter.odometry")?;
                match ingest_map_pose_odometry(&adapter, &frame.payload)? {
                    MapPoseOdometryIngress::Accepted(observed) => {
                        pending_odometry = None;
                        publish_map_pose_observation(
                            &transport,
                            &node.outputs[0],
                            observed,
                        ).await?;
                    }
                    MapPoseOdometryIngress::AwaitingTransform(odometry, reason) => {
                        pending_odometry = Some(odometry);
                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                    }
                    MapPoseOdometryIngress::Rejected(reason) => {
                        pending_odometry = None;
                        diagnostics.reject("map_pose_adapter.odometry", &reason);
                    }
                }
            }
        }
    }
}

#[cfg(feature = "guard-runtime")]
fn configured_frame(key: &str, fallback: &str) -> NavStackResult<String> {
    let value = std::env::var(key).unwrap_or_else(|_| fallback.to_string());
    let value = value.trim();
    if value.is_empty() {
        Err(NavStackError::InvalidConfig(format!(
            "{key} must not be empty"
        )))
    } else {
        Ok(value.to_string())
    }
}

#[cfg(feature = "guard-runtime")]
pub(crate) fn guarded_nav_scheduler_owner(
    node: &ModuleNode,
) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 5, &[4])?;
    let node = node.clone();
    Ok(boxed_owner(
        "guarded-nav-scheduler",
        Box::new(move |shutdown, ready| Box::pin(run_guarded_nav_scheduler(node, shutdown, ready))),
    ))
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone, Debug, Default, PartialEq)]
struct GuardedNavTelemetry {
    runtime_generation: u64,
    public_session_id: String,
    owner_kind: i32,
    parent_request_id: String,
    goal_generation: i64,
    cancel_generation: u64,
    map_frame: String,
    pose_frame: String,
    goal: [f64; 3],
    current: [f64; 3],
    initial_distance: f64,
    progress: f64,
    path_revision: i64,
    goal_payload_sha256: [u8; 32],
    path_payload_sha256: [u8; 32],
}

#[cfg(feature = "guard-runtime")]
struct GuardedPoseInput {
    pose: PoseStamped,
    frame_id: String,
    current: [f64; 3],
}

#[cfg(feature = "guard-runtime")]
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
    Ok(OwnerInput::Accepted(GuardedCostmapInput {
        observation: nuwax_nav_runtime::CostmapObservation {
            costmap,
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
async fn run_guarded_nav_scheduler(
    node: ModuleNode,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.GoalEnvelope").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let costmaps = typed_stream(&transport, &node.inputs[3], "nav_msgs.OccupancyGrid").await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let socket_path = std::env::var("NUWAX_MOTION_GUARD_SOCKET")
        .unwrap_or_else(|_| "/run/nuwax/motion-guard.sock".to_string());
    let (scheduler, task) = nuwax_nav_runtime::spawn_production_scheduler(
        std::path::PathBuf::from(socket_path),
        nuwax_nav_runtime::SchedulerConfig::default(),
    )
    .await
    .map_err(|error| module_error("guarded_nav_scheduler.connect", error))?;
    let status = scheduler.subscribe_status();
    signal_ready(ready)?;

    let loop_result = run_guarded_nav_loop(
        &node, &transport, &scheduler, shutdown, goals, poses, paths, costmaps, cancels, status,
    )
    .await;
    let shutdown_result = if task.is_finished() {
        Ok(())
    } else {
        scheduler
            .shutdown()
            .await
            .map_err(|error| module_error("guarded_nav_scheduler.shutdown", error))
    };
    let task_result = task
        .await
        .map_err(|error| NavStackError::Internal(format!("guard scheduler join: {error}")))?
        .map_err(|error| module_error("guarded_nav_scheduler.task", error));
    loop_result?;
    shutdown_result?;
    task_result
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn run_guarded_nav_loop(
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    mut shutdown: oneshot::Receiver<()>,
    mut goals: LcmFrameStream,
    mut poses: LcmFrameStream,
    mut paths: LcmFrameStream,
    mut costmaps: LcmFrameStream,
    mut cancels: LcmFrameStream,
    mut status: tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
) -> NavStackResult<()> {
    let mut telemetry = GuardedNavTelemetry {
        runtime_generation: runtime_generation(),
        ..GuardedNavTelemetry::default()
    };
    let mut active_session: Option<schema::GoalEnvelope> = None;
    let mut goal_buffer = PlannerGoalBuffer::default();
    let mut applied_commands = 0_u64;
    let mut last_zero_ack_id: Option<String> = None;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let initial_status = status.borrow().clone();
    publish_guarded_nav_state(
        transport,
        &node.outputs[1],
        initial_status,
        &telemetry,
        None,
    )
    .await?;

    loop {
        tokio::select! {
            _ = &mut shutdown => return Ok(()),
            changed = status.changed() => {
                changed.map_err(|_| NavStackError::Internal("guard scheduler status stream closed".into()))?;
                let snapshot = status.borrow().clone();
                if snapshot.applied_commands > applied_commands {
                    if let Some(twist) = snapshot.last_applied_twist {
                        publish_twist2(transport, &node.outputs[0], twist).await?;
                    }
                    applied_commands = snapshot.applied_commands;
                }
                if snapshot.last_zero_ack_id != last_zero_ack_id {
                    if snapshot.last_zero_ack_id.is_some() {
                        publish_twist2(transport, &node.outputs[0], Twist2::ZERO).await?;
                    }
                    last_zero_ack_id = snapshot.last_zero_ack_id.clone();
                }
                if snapshot.last_terminal == Some(nuwax_nav_runtime::TerminalOutcome::Succeeded) {
                    publish_bool(transport, &node.outputs[2], true).await?;
                }
                publish_guarded_nav_state(
                    transport,
                    &node.outputs[1],
                    snapshot.clone(),
                    &telemetry,
                    None,
                ).await?;
                clear_terminal_session_after_publication(
                    &mut active_session,
                    &mut telemetry,
                    &snapshot,
                );
            }
            message = poses.next() => {
                let frame = required_frame(message, "guarded_nav_scheduler.map_pose")?;
                let input = match decode_guarded_pose_input(&frame.payload)? {
                    OwnerInput::Accepted(input) => input,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("guarded_nav_scheduler.map_pose", &reason);
                        continue;
                    }
                };
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_pose_telemetry(&mut candidate_telemetry, &input);
                if let Err(error) = scheduler.update_odometry(input.pose).await {
                    match runtime_input_error::<()>("guarded_nav_scheduler.map_pose", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
                telemetry = candidate_telemetry;
                if let Some(goal) = goal_buffer.take_ready_goal() {
                    activate_or_defer_guarded_goal(
                        node,
                        transport,
                        scheduler,
                        &mut telemetry,
                        &mut active_session,
                        &mut goal_buffer,
                        *goal,
                    )
                    .await?;
                }
                let snapshot = scheduler.status();
                publish_guarded_nav_state(
                    transport,
                    &node.outputs[1],
                    snapshot.clone(),
                    &telemetry,
                    Some(snapshot.planner_inputs_fresh),
                ).await?;
                clear_terminal_session_after_publication(
                    &mut active_session,
                    &mut telemetry,
                    &snapshot,
                );
            }
            message = costmaps.next() => {
                let frame = required_frame(message, "guarded_nav_scheduler.costmap")?;
                let input = match decode_guarded_costmap_input(&frame.payload)? {
                    OwnerInput::Accepted(input) => input,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("guarded_nav_scheduler.costmap", &reason);
                        continue;
                    }
                };
                // 保留活动会话；只有显式替换目标才能抢占。
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_costmap_telemetry(&mut candidate_telemetry, &input);
                if let Err(error) = scheduler.update_costmap(input.observation).await {
                    match runtime_input_error::<()>("guarded_nav_scheduler.costmap", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("guarded_nav_scheduler.costmap", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
                telemetry = candidate_telemetry;
                let ready_goal = goal_buffer.observe_costmap();
                if let Some(goal) = ready_goal {
                    activate_or_defer_guarded_goal(
                        node,
                        transport,
                        scheduler,
                        &mut telemetry,
                        &mut active_session,
                        &mut goal_buffer,
                        *goal,
                    )
                    .await?;
                }
                let snapshot = scheduler.status();
                publish_guarded_nav_state(
                    transport,
                    &node.outputs[1],
                    snapshot.clone(),
                    &telemetry,
                    Some(snapshot.planner_inputs_fresh),
                ).await?;
                clear_terminal_session_after_publication(
                    &mut active_session,
                    &mut telemetry,
                    &snapshot,
                );
            }
            message = goals.next() => {
                let frame = required_frame(message, "guarded_nav_scheduler.goal")?;
                let goal_envelope = match schema::decode_goal_envelope_body(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "goal",
                            &format!("GoalEnvelope rejected: {error}"),
                        ).await?;
                        continue;
                    }
                };
                if let Err(error) = pose_from_schema(&goal_envelope.goal).and_then(|goal| {
                    require_map_pose(&goal, "guarded_nav_scheduler.goal")
                }) {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "goal",
                        &error.to_string(),
                    ).await?;
                    continue;
                }
                let goal_envelope = match goal_buffer.observe_goal(goal_envelope) {
                    PlannerGoalIngress::Ready(goal) => *goal,
                    PlannerGoalIngress::Buffered => {
                        if preempt_guarded_active_session(
                            node,
                            transport,
                            scheduler,
                            &mut telemetry,
                            &mut active_session,
                        )
                        .await?
                        {
                            publish_guarded_nav_state(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                None,
                            )
                            .await?;
                        }
                        continue;
                    }
                    PlannerGoalIngress::Rejected(reason) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "goal",
                            &format!("GoalEnvelope lease rejected: {reason:?}"),
                        ).await?;
                        continue;
                    }
                };
                activate_or_defer_guarded_goal(
                    node,
                    transport,
                    scheduler,
                    &mut telemetry,
                    &mut active_session,
                    &mut goal_buffer,
                    goal_envelope,
                )
                .await?;
            }
            message = paths.next() => {
                let frame = required_frame(message, "guarded_nav_scheduler.path")?;
                let Some(active_goal) = active_session.as_ref() else {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "no active versioned goal lease",
                    ).await?;
                    continue;
                };
                if !goal_buffer.has_applied_goal() {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "active goal lease is fenced by a newer goal",
                    ).await?;
                    continue;
                }
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
                        ).await?;
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
                    ).await?;
                    continue;
                }
                let path = match path_from_schema(&path_envelope.path) {
                    Ok(path) => path,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "path",
                            &error.to_string(),
                        ).await?;
                        continue;
                    }
                };
                if path.frame_id != NavFrameId::Map {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        &format!("scheduler path frame must be map, got {}", path.frame_id.as_str()),
                    ).await?;
                    continue;
                }
                let result = scheduler.submit_path(nuwax_nav_runtime::VersionedPath {
                    session_id: path_envelope.session_id.clone(),
                    path_revision: u64::try_from(path_envelope.path_revision)
                        .expect("validated positive path revision"),
                    payload_hash: path_envelope.path_payload_sha256,
                    path,
                }).await;
                if let Err(error) = result {
                    match runtime_input_error::<()>("guarded_nav_scheduler.path", error)? {
                        OwnerInput::Rejected(reason) => {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "path",
                                &reason,
                            ).await?;
                            let snapshot = scheduler.status();
                            clear_terminal_session_after_publication(
                                &mut active_session,
                                &mut telemetry,
                                &snapshot,
                            );
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                } else {
                    telemetry.path_revision = path_envelope.path_revision;
                    telemetry.path_payload_sha256 = path_envelope.path_payload_sha256;
                }
            }
            message = cancels.next() => {
                let frame = required_frame(message, "guarded_nav_scheduler.cancel")?;
                let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
                    Ok(cancel) => cancel,
                    Err(error) => {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "cancel",
                            &format!("CancelNavigation rejected: {error}"),
                        ).await?;
                        continue;
                    }
                };
                let Some(cancel_generation) = u64::try_from(cancel.cancel_generation)
                    .ok()
                    .filter(|generation| *generation > 0)
                else {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "cancel",
                        "CancelNavigation.cancel_generation must be positive",
                    ).await?;
                    continue;
                };
                let mut candidate_telemetry = telemetry.clone();
                candidate_telemetry.cancel_generation = cancel_generation;
                candidate_telemetry.public_session_id = cancel.session_id.clone();
                candidate_telemetry.owner_kind = cancel.owner_kind;
                candidate_telemetry.parent_request_id = cancel.parent_request_id.clone();
                candidate_telemetry.goal_generation = cancel.goal_generation;
                if let Some(active_goal) = active_session.clone() {
                    if let Err(reason) = validate_cancel_goal_correlation(&cancel, &active_goal) {
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &telemetry,
                            "cancel",
                            &reason,
                        ).await?;
                        continue;
                    }
                    let session_id = active_goal.session_id;
                    if let Err(error) = scheduler.cancel(&session_id).await {
                        match runtime_input_error::<()>("guarded_nav_scheduler.cancel", error)? {
                            OwnerInput::Rejected(reason) => {
                                publish_guarded_rejection(
                                    transport,
                                    &node.outputs[1],
                                    scheduler.status(),
                                    &telemetry,
                                    "cancel",
                                    &reason,
                                ).await?;
                                let snapshot = scheduler.status();
                                clear_terminal_session_after_publication(
                                    &mut active_session,
                                    &mut telemetry,
                                    &snapshot,
                                );
                                continue;
                            }
                            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                        }
                    }
                    telemetry = candidate_telemetry;
                    publish_bool(transport, &node.outputs[3], true).await?;
                    let snapshot = scheduler.status();
                    publish_guarded_nav_state(
                        transport,
                        &node.outputs[1],
                        snapshot.clone(),
                        &telemetry,
                        None,
                    ).await?;
                    clear_terminal_session_after_publication(
                        &mut active_session,
                        &mut telemetry,
                        &snapshot,
                    );
                } else {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "cancel",
                        "no active scheduler session",
                    ).await?;
                }
            }
        }
    }
}

#[cfg(feature = "guard-runtime")]
async fn fence_guarded_active_session(
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
) -> Result<(), nuwax_nav_runtime::RuntimeError> {
    let Some(active_goal) = active_session.clone() else {
        return Ok(());
    };
    let session_id = active_goal.session_id;
    let snapshot = scheduler.status();
    if scheduler_session_is_active(&snapshot, &session_id) {
        scheduler.cancel(&session_id).await?;
    }
    *active_session = None;
    clear_guarded_session_telemetry(telemetry);
    Ok(())
}

#[cfg(feature = "guard-runtime")]
async fn preempt_guarded_active_session(
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
) -> NavStackResult<bool> {
    match fence_guarded_active_session(scheduler, telemetry, active_session).await {
        Ok(()) => Ok(true),
        Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.preempt", error)? {
            OwnerInput::Rejected(reason) => {
                publish_guarded_rejection(
                    transport,
                    &node.outputs[1],
                    scheduler.status(),
                    telemetry,
                    "preempt",
                    &reason,
                )
                .await?;
                let snapshot = scheduler.status();
                clear_terminal_session_after_publication(active_session, telemetry, &snapshot);
                Ok(active_session.is_none())
            }
            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
        },
    }
}

#[cfg(feature = "guard-runtime")]
async fn activate_or_defer_guarded_goal(
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_buffer: &mut PlannerGoalBuffer,
    goal_envelope: schema::GoalEnvelope,
) -> NavStackResult<()> {
    if !guarded_goal_inputs_ready(telemetry, scheduler, &goal_envelope) {
        if !goal_buffer.defer_ready_goal(goal_envelope) {
            return Err(NavStackError::Internal(
                "guarded scheduler could not retain a not-yet-ready GoalEnvelope".into(),
            ));
        }
        return Ok(());
    }

    let applied_goal = goal_envelope.clone();
    if activate_guarded_goal(
        node,
        transport,
        scheduler,
        telemetry,
        active_session,
        goal_envelope,
    )
    .await?
    {
        if !goal_buffer.mark_applied(&applied_goal) {
            return Err(NavStackError::Internal(
                "guarded scheduler applied a superseded GoalEnvelope".into(),
            ));
        }
    } else if !goal_buffer.defer_ready_goal(applied_goal) {
        return Err(NavStackError::Internal(
            "guarded scheduler lost a GoalEnvelope after deferred activation".into(),
        ));
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn guarded_goal_inputs_ready(
    telemetry: &GuardedNavTelemetry,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    _goal_envelope: &schema::GoalEnvelope,
) -> bool {
    !telemetry.map_frame.is_empty()
        && telemetry.pose_frame == telemetry.map_frame
        && scheduler.status().planner_inputs_fresh
}

#[cfg(feature = "guard-runtime")]
async fn activate_guarded_goal(
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_envelope: schema::GoalEnvelope,
) -> NavStackResult<bool> {
    let goal = match pose_from_schema(&goal_envelope.goal).and_then(|goal| {
        require_map_pose(&goal, "guarded_nav_scheduler.goal")?;
        Ok(goal)
    }) {
        Ok(goal) => goal,
        Err(error) => {
            publish_guarded_rejection(
                transport,
                &node.outputs[1],
                scheduler.status(),
                telemetry,
                "goal",
                &error.to_string(),
            )
            .await?;
            return Ok(false);
        }
    };
    if telemetry.map_frame.is_empty()
        || telemetry.pose_frame != telemetry.map_frame
        || !scheduler.status().planner_inputs_fresh
    {
        publish_guarded_rejection(
            transport,
            &node.outputs[1],
            scheduler.status(),
            telemetry,
            "goal",
            "planner pose/costmap inputs are not ready or use different frames",
        )
        .await?;
        return Ok(false);
    }

    if !preempt_guarded_active_session(node, transport, scheduler, telemetry, active_session)
        .await?
    {
        return Ok(false);
    }

    let mut candidate_telemetry = telemetry.clone();
    candidate_telemetry.goal = [
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.orientation.yaw(),
    ];
    candidate_telemetry.public_session_id = goal_envelope.session_id.clone();
    candidate_telemetry.owner_kind = goal_envelope.owner_kind;
    candidate_telemetry.parent_request_id = goal_envelope.parent_request_id.clone();
    candidate_telemetry.goal_generation = goal_envelope.goal_generation;
    candidate_telemetry.goal_payload_sha256 = goal_envelope.payload_sha256;
    candidate_telemetry.path_revision = 0;
    candidate_telemetry.path_payload_sha256 = [0; 32];
    candidate_telemetry.initial_distance =
        distance_2d(candidate_telemetry.goal, candidate_telemetry.current);
    candidate_telemetry.progress = 0.0;
    if let Err(error) = scheduler
        .start_session(nuwax_nav_runtime::NavSession {
            session_id: goal_envelope.session_id.clone(),
            motion_admission: nuwax_nav_runtime::NavMotionAdmission {
                guard_generation: u64::try_from(goal_envelope.motion_guard_generation)
                    .expect("validated positive motion_guard_generation"),
                guard_boot_nonce: goal_envelope.motion_guard_boot_nonce,
                motion_stop_generation: u64::try_from(goal_envelope.motion_stop_generation)
                    .expect("validated non-negative motion_stop_generation"),
            },
        })
        .await
    {
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
                .await?;
                return Ok(false);
            }
            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
        }
    }
    *telemetry = candidate_telemetry;
    *active_session = Some(goal_envelope);
    Ok(true)
}

#[cfg(feature = "guard-runtime")]
fn runtime_generation() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(u64::MAX as u128) as u64
}

#[cfg(feature = "guard-runtime")]
fn clear_terminal_active_session(
    active_session: &mut Option<schema::GoalEnvelope>,
    status: &nuwax_nav_runtime::SchedulerStatus,
) -> bool {
    if active_session.is_some() && status.last_terminal.is_some() {
        active_session.take();
        true
    } else {
        false
    }
}

#[cfg(feature = "guard-runtime")]
fn clear_guarded_session_telemetry(telemetry: &mut GuardedNavTelemetry) {
    telemetry.public_session_id.clear();
    telemetry.owner_kind = 0;
    telemetry.parent_request_id.clear();
    telemetry.goal_generation = 0;
    telemetry.cancel_generation = 0;
    telemetry.goal = [0.0; 3];
    telemetry.goal_payload_sha256 = [0; 32];
    telemetry.path_revision = 0;
    telemetry.path_payload_sha256 = [0; 32];
    telemetry.initial_distance = 0.0;
    telemetry.progress = 0.0;
}

#[cfg(feature = "guard-runtime")]
fn clear_terminal_session_after_publication(
    active_session: &mut Option<schema::GoalEnvelope>,
    telemetry: &mut GuardedNavTelemetry,
    status: &nuwax_nav_runtime::SchedulerStatus,
) -> bool {
    if !clear_terminal_active_session(active_session, status) {
        return false;
    }
    clear_guarded_session_telemetry(telemetry);
    true
}

#[cfg(feature = "guard-runtime")]
fn scheduler_session_is_active(
    status: &nuwax_nav_runtime::SchedulerStatus,
    session_id: &str,
) -> bool {
    use nuwax_nav_runtime::SchedulerPhase;

    status.session_id.as_deref() == Some(session_id)
        && matches!(
            status.phase,
            SchedulerPhase::Acquiring
                | SchedulerPhase::AwaitingPath
                | SchedulerPhase::Controlling
                | SchedulerPhase::Cancelling
                | SchedulerPhase::Fencing
        )
}

#[cfg(feature = "guard-runtime")]
fn validate_path_goal_correlation(
    path: &schema::PathEnvelope,
    goal: &schema::GoalEnvelope,
) -> Result<(), String> {
    if path.session_id != goal.session_id
        || path.owner_kind != goal.owner_kind
        || path.parent_request_id != goal.parent_request_id
        || path.goal_generation != goal.goal_generation
        || path.goal_payload_sha256 != goal.payload_sha256
    {
        return Err(
            "PathEnvelope identity/goal hash does not match the active GoalEnvelope".into(),
        );
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn validate_cancel_goal_correlation(
    cancel: &schema::CancelNavigation,
    goal: &schema::GoalEnvelope,
) -> Result<(), String> {
    if cancel.session_id != goal.session_id
        || cancel.owner_kind != goal.owner_kind
        || cancel.parent_request_id != goal.parent_request_id
        || cancel.goal_generation != goal.goal_generation
        || cancel.producer_runtime_generation != goal.producer_runtime_generation
    {
        return Err("CancelNavigation identity does not match the active GoalEnvelope".into());
    }
    Ok(())
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
fn update_progress(telemetry: &mut GuardedNavTelemetry) {
    if telemetry.initial_distance > f64::EPSILON {
        let remaining = distance_2d(telemetry.goal, telemetry.current);
        telemetry.progress = (1.0 - remaining / telemetry.initial_distance).clamp(0.0, 1.0);
    }
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_nav_state(
    transport: &LcmTransport,
    channel: &str,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
) -> NavStackResult<()> {
    let message = build_guarded_navigation_state(
        &status,
        telemetry,
        planner_observation,
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default(),
    );
    let body = schema::encode_navigation_state_body(&message)
        .map_err(|error| NavStackError::Internal(format!("NavigationState encode: {error}")))?;
    publish_body(transport, channel, "nuwax_msgs.NavigationState", body).await
}

#[cfg(feature = "guard-runtime")]
fn build_guarded_navigation_state(
    status: &nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    now: Duration,
) -> schema::NavigationState {
    use nuwax_nav_runtime::{SchedulerPhase, TerminalOutcome};

    let has_public_session = !telemetry.public_session_id.is_empty();
    let state = if !has_public_session {
        0
    } else {
        match (status.phase, status.last_terminal) {
            (_, Some(TerminalOutcome::Succeeded)) => 3,
            (_, Some(TerminalOutcome::Cancelled | TerminalOutcome::Shutdown)) => 0,
            (SchedulerPhase::Failed, _) => 4,
            (SchedulerPhase::Controlling, _) => 2,
            (SchedulerPhase::Acquiring | SchedulerPhase::AwaitingPath, _) => 1,
            _ => 0,
        }
    };
    let message = if !has_public_session {
        "IDLE: guard client heartbeat".to_string()
    } else {
        match status.last_terminal {
            Some(TerminalOutcome::Cancelled) => format!("CANCELLED: {}", status.message),
            _ => status.message.clone(),
        }
    };
    let include_planner_fields = planner_observation == Some(true);
    schema::NavigationState {
        session_id: telemetry.public_session_id.clone(),
        robot_frame: "base_link".into(),
        stamp_sec: now.as_secs().min(i64::MAX as u64) as i64,
        stamp_nsec: now.subsec_nanos() as i32,
        state,
        goal: telemetry.goal,
        current: telemetry.current,
        progress: telemetry.progress,
        message,
        runtime_generation: planner_observation
            .map_or(0, |_| saturating_i64(telemetry.runtime_generation)),
        guard_generation: if has_public_session {
            saturating_i64(status.guard_generation)
        } else {
            0
        },
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
        cancel_generation: saturating_i64(telemetry.cancel_generation),
        zero_ack_id: if has_public_session {
            status.last_zero_ack_id.clone().unwrap_or_default()
        } else {
            String::new()
        },
        zero_applied_monotonic_ns: if has_public_session {
            saturating_i64(status.last_fence_monotonic_ns)
        } else {
            0
        },
        owner_kind: telemetry.owner_kind,
        parent_request_id: telemetry.parent_request_id.clone(),
        goal_generation: telemetry.goal_generation,
        path_revision: telemetry.path_revision,
        goal_payload_sha256: telemetry.goal_payload_sha256,
        path_payload_sha256: telemetry.path_payload_sha256,
    }
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_rejection(
    transport: &LcmTransport,
    channel: &str,
    mut status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    input: &str,
    reason: &str,
) -> NavStackResult<()> {
    status.phase = nuwax_nav_runtime::SchedulerPhase::Failed;
    status.message = format!("REJECTED {input}: {reason}");
    publish_guarded_nav_state(transport, channel, status, telemetry, None).await
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
pub(crate) fn click_goal_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 1, &[1])?;
    let node = node.clone();
    Ok(boxed_owner(
        "click-goal-router",
        Box::new(move |shutdown, ready| Box::pin(run_click_goal(node, shutdown, ready))),
    ))
}

#[cfg(feature = "click-goal")]
async fn run_click_goal(
    node: ModuleNode,
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
pub(crate) fn wavefront_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 5, &[1])?;
    let node = node.clone();
    Ok(boxed_owner(
        "wavefront-explorer",
        Box::new(move |shutdown, ready| Box::pin(run_wavefront(node, shutdown, ready))),
    ))
}

#[cfg(feature = "explore")]
async fn run_wavefront(
    node: ModuleNode,
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

    let config = wavefront_config_from_env()?;
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
fn wavefront_config_from_env() -> NavStackResult<nuwax_explore_wavefront::WavefrontConfig> {
    let mut config = nuwax_explore_wavefront::WavefrontConfig::default();
    if let Ok(value) = std::env::var("NUWAX_WAVEFRONT_GOAL_TIMEOUT_MS") {
        config.goal_timeout_ms = value.parse::<u64>().map_err(|error| {
            NavStackError::InvalidConfig(format!(
                "NUWAX_WAVEFRONT_GOAL_TIMEOUT_MS must be an unsigned integer: {error}"
            ))
        })?;
        if config.goal_timeout_ms == 0 {
            return Err(NavStackError::InvalidConfig(
                "NUWAX_WAVEFRONT_GOAL_TIMEOUT_MS must be greater than zero".into(),
            ));
        }
    }
    Ok(config)
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
pub(crate) fn terrain_map_ext_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 2, &[1])?;
    let node = node.clone();
    Ok(boxed_owner(
        "terrain-map-ext",
        Box::new(move |shutdown, ready| Box::pin(run_terrain_map_ext(node, shutdown, ready))),
    ))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_terrain_map_ext(
    node: ModuleNode,
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
const BOUNDARY_EPSILON: f64 = 1e-9;

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
#[derive(Debug, Clone, PartialEq)]
struct NavigationBoundary {
    vertices: Vec<Point2>,
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn navigation_boundary_from_goal(
    goal: &schema::GoalEnvelope,
) -> NavStackResult<Option<NavigationBoundary>> {
    match goal.owner_kind {
        schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV => {
            if goal.boundary_frame_id.is_empty() && goal.boundary_points.is_empty() {
                Ok(None)
            } else {
                Err(NavStackError::InvalidConfig(
                    "PUBLIC_NAV goal must not carry a navigation boundary".into(),
                ))
            }
        }
        schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD => {
            if goal.boundary_points.is_empty() {
                return if goal.boundary_frame_id.is_empty() {
                    Ok(None)
                } else {
                    Err(NavStackError::InvalidConfig(
                        "empty exploration boundary must not carry a frame".into(),
                    ))
                };
            }
            if goal.boundary_frame_id != "map" {
                return Err(NavStackError::InvalidConfig(
                    "exploration navigation boundary frame must be map".into(),
                ));
            }
            let boundary = NavigationBoundary::new(&goal.boundary_points)?;
            let goal_point = Point2::new(goal.goal.pose.position[0], goal.goal.pose.position[1]);
            if !boundary.contains(goal_point) {
                return Err(NavStackError::InvalidConfig(
                    "exploration goal must lie inside its navigation boundary".into(),
                ));
            }
            Ok(Some(boundary))
        }
        owner => Err(NavStackError::InvalidConfig(format!(
            "unsupported navigation boundary owner kind {owner}"
        ))),
    }
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)]
enum PlannerGoalRejectReason {
    DuplicateLease,
    SupersededLease,
    ConflictingLease,
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
#[derive(Debug)]
enum PlannerGoalIngress {
    Ready(Box<schema::GoalEnvelope>),
    Buffered,
    Rejected(PlannerGoalRejectReason),
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
#[derive(Default)]
struct PlannerGoalBuffer {
    costmap_observed: bool,
    pending_goal: Option<schema::GoalEnvelope>,
    newest_goal: Option<schema::GoalEnvelope>,
    active_goal_sha256: Option<[u8; 32]>,
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
impl PlannerGoalBuffer {
    fn observe_goal(&mut self, goal: schema::GoalEnvelope) -> PlannerGoalIngress {
        if let Some(newest) = self.newest_goal.as_ref() {
            if let Err(reason) = validate_goal_envelope_order(&goal, newest) {
                return PlannerGoalIngress::Rejected(reason);
            }
        }
        self.active_goal_sha256 = None;
        self.newest_goal = Some(goal.clone());
        if self.costmap_observed {
            self.pending_goal = None;
            PlannerGoalIngress::Ready(Box::new(goal))
        } else {
            self.pending_goal = Some(goal);
            PlannerGoalIngress::Buffered
        }
    }

    fn observe_costmap(&mut self) -> Option<Box<schema::GoalEnvelope>> {
        self.costmap_observed = true;
        // Occupancy header.seq is compatibility telemetry, not a planner
        // synchronization primitive. Timestamp validation and receipt TTL own
        // freshness; any valid costmap can release the newest pending lease.
        self.pending_goal.take().map(Box::new)
    }

    fn mark_applied(&mut self, goal: &schema::GoalEnvelope) -> bool {
        let is_current = self.newest_goal.as_ref().is_some_and(|newest| {
            planner_goal_identity_matches(goal, newest)
                && goal.payload_sha256 == newest.payload_sha256
        });
        if is_current {
            self.active_goal_sha256 = Some(goal.payload_sha256);
            if self.pending_goal.as_ref().is_some_and(|pending| {
                planner_goal_identity_matches(pending, goal)
                    && pending.payload_sha256 == goal.payload_sha256
            }) {
                self.pending_goal = None;
            }
        }
        is_current
    }

    fn defer_ready_goal(&mut self, goal: schema::GoalEnvelope) -> bool {
        let is_current = self.newest_goal.as_ref().is_some_and(|newest| {
            planner_goal_identity_matches(&goal, newest)
                && goal.payload_sha256 == newest.payload_sha256
        });
        if is_current {
            self.active_goal_sha256 = None;
            self.pending_goal = Some(goal);
        }
        is_current
    }

    fn take_ready_goal(&mut self) -> Option<Box<schema::GoalEnvelope>> {
        if self.costmap_observed {
            self.pending_goal.take().map(Box::new)
        } else {
            None
        }
    }

    fn has_applied_goal(&self) -> bool {
        self.active_goal_sha256.is_some()
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn clear_planner_goal_for_costmap_advance(
    active_goal: &mut Option<schema::GoalEnvelope>,
    active_boundary: &mut Option<NavigationBoundary>,
    path_revision: &mut i64,
) {
    active_goal.take();
    active_boundary.take();
    *path_revision = 0;
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
fn planner_goal_order(goal: &schema::GoalEnvelope) -> (i64, i64) {
    (goal.producer_runtime_generation, goal.goal_generation)
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
fn planner_goal_identity_matches(
    left: &schema::GoalEnvelope,
    right: &schema::GoalEnvelope,
) -> bool {
    left.session_id == right.session_id
        && left.owner_kind == right.owner_kind
        && left.parent_request_id == right.parent_request_id
        && left.goal_generation == right.goal_generation
        && left.producer_runtime_generation == right.producer_runtime_generation
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "simple-2d",
    feature = "replanning-astar"
))]
fn validate_goal_envelope_order(
    candidate: &schema::GoalEnvelope,
    newest: &schema::GoalEnvelope,
) -> Result<(), PlannerGoalRejectReason> {
    match planner_goal_order(candidate).cmp(&planner_goal_order(newest)) {
        std::cmp::Ordering::Less => Err(PlannerGoalRejectReason::SupersededLease),
        std::cmp::Ordering::Equal => {
            if planner_goal_identity_matches(candidate, newest)
                && candidate.payload_sha256 == newest.payload_sha256
            {
                Err(PlannerGoalRejectReason::DuplicateLease)
            } else {
                Err(PlannerGoalRejectReason::ConflictingLease)
            }
        }
        std::cmp::Ordering::Greater => Ok(()),
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn cross_product(start: Point2, end: Point2, point: Point2) -> f64 {
    (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x)
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn validate_newer_source_timestamp(
    accepted_timestamp_ns: i64,
    candidate_timestamp_ns: i64,
    component: &str,
) -> OwnerInputResult<()> {
    if candidate_timestamp_ns <= accepted_timestamp_ns {
        Ok(OwnerInput::Rejected(format!(
            "{component} timestamp {candidate_timestamp_ns} is duplicate or older than accepted timestamp {accepted_timestamp_ns}"
        )))
    } else {
        Ok(OwnerInput::Accepted(()))
    }
}

#[cfg(feature = "simple-2d")]
fn apply_simple_planner_goal(
    module: &mut nuwax_nav_simple_planner::module::SimplePlannerModule,
    envelope: schema::GoalEnvelope,
    occupancy_body: &[u8],
) -> OwnerInputResult<(schema::GoalEnvelope, Option<NavigationBoundary>)> {
    use nuwax_nav_simple_planner::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let pose = match pose_from_schema(&envelope.goal) {
        Ok(pose) if require_map_pose(&pose, "simple_planner.goal_pose").is_ok() => pose,
        Ok(_) => {
            return Ok(OwnerInput::Rejected(
                "simple planner goal pose must use map frame".into(),
            ));
        }
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
    Module::on_terrain_map_ext(module, occupancy)
        .map_err(|error| module_error("simple_planner.goal_boundary", error))?;
    if let Err(error) = Module::on_goal(
        module,
        PointStamped::new(pose.frame_id, pose.stamp, pose.pose.position),
    ) {
        return module_input_error("simple_planner.goal_pose", error);
    }
    Ok(OwnerInput::Accepted((envelope, boundary)))
}

#[cfg(feature = "replanning-astar")]
fn apply_astar_planner_goal(
    module: &mut nuwax_nav_astar::module::ReplanningAStarPlannerModule,
    envelope: schema::GoalEnvelope,
    costmap_body: &[u8],
) -> OwnerInputResult<(schema::GoalEnvelope, Option<NavigationBoundary>, f64)> {
    use nuwax_nav_astar::module::Module;

    let boundary = match navigation_boundary_from_goal(&envelope) {
        Ok(boundary) => boundary,
        Err(error) => return Ok(OwnerInput::Rejected(error.to_string())),
    };
    let goal_pose = match pose_from_schema(&envelope.goal) {
        Ok(goal_pose) if require_map_pose(&goal_pose, "astar_planner.goal").is_ok() => goal_pose,
        Ok(_) => {
            return Ok(OwnerInput::Rejected(
                "A* planner goal pose must use map frame".into(),
            ));
        }
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

#[cfg(feature = "simple-2d")]
pub(crate) fn simple_planner_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 4, &[2])?;
    let node = node.clone();
    Ok(boxed_owner(
        "simple-planner",
        Box::new(move |shutdown, ready| Box::pin(run_simple_planner(node, shutdown, ready))),
    ))
}

#[cfg(feature = "simple-2d")]
async fn run_simple_planner(
    node: ModuleNode,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::{Module, SimplePlannerModule};

    let transport = lcm_transport()?;
    let mut occupancy = typed_stream(&transport, &node.inputs[0], "nav_msgs.OccupancyGrid").await?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let mut goal_pose =
        typed_stream(&transport, &node.inputs[2], "nuwax_msgs.GoalEnvelope").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut module = SimplePlannerModule::new();
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut goal_buffer = PlannerGoalBuffer::default();
    let mut latest_occupancy_body: Option<Vec<u8>> = None;
    let mut active_goal: Option<schema::GoalEnvelope> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05_f64;
    let mut published_path_is_boundary_safe = false;
    let mut path_revision = 0_i64;
    let mut last_pose_timestamp_ns = 0_i64;
    let mut diagnostics = InputDiagnosticLimiter::default();
    Module::start(&mut module).map_err(|error| module_error("simple_planner", error))?;
    let mut ticker = tokio::time::interval(Duration::from_millis(
        nuwax_nav_simple_planner::module::TICK_PERIOD_MS,
    ));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = occupancy.next() => {
                let frame = required_frame(message, "simple_planner.occupancy")?;
                let grid = match decode_planner_grid_input(
                    &frame.payload,
                    "simple_planner.occupancy",
                )? {
                    OwnerInput::Accepted(grid) => grid,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("simple_planner.occupancy", &reason);
                        continue;
                    }
                };
                let raw_occupancy = match decode_simple_occupancy(&frame.payload) {
                    Ok(occupancy) => occupancy,
                    Err(error) => {
                        diagnostics.reject("simple_planner.occupancy", &error.to_string());
                        continue;
                    }
                };
                let ready_goal = goal_buffer.observe_costmap();
                published_path_is_boundary_safe = false;
                if let Some(envelope) = ready_goal {
                    clear_planner_goal_for_costmap_advance(
                        &mut active_goal,
                        &mut active_boundary,
                        &mut path_revision,
                    );
                    match apply_simple_planner_goal(&mut module, *envelope, &frame.payload)? {
                        OwnerInput::Accepted((envelope, boundary)) => {
                            if !goal_buffer.mark_applied(&envelope) {
                                return Err(NavStackError::Internal(
                                    "simple planner applied a superseded GoalEnvelope".into(),
                                ));
                            }
                            path_revision = 0;
                            published_path_is_boundary_safe = false;
                            active_boundary = boundary;
                            active_goal = Some(envelope);
                        }
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("simple_planner.goal_pose", &reason);
                            Module::on_terrain_map_ext(&mut module, raw_occupancy)
                                .map_err(|error| module_error("simple_planner.occupancy", error))?;
                        }
                    }
                } else {
                    let boundary = if goal_buffer.has_applied_goal() {
                        active_boundary.as_ref()
                    } else {
                        None
                    };
                    let occupancy = match boundary {
                        Some(boundary) => match decode_simple_occupancy_with_boundary(
                            &frame.payload,
                            Some(boundary),
                        ) {
                            Ok(occupancy) => occupancy,
                            Err(error) => {
                                diagnostics.reject("simple_planner.occupancy", &error.to_string());
                                continue;
                            }
                        },
                        None => raw_occupancy,
                    };
                    Module::on_terrain_map_ext(&mut module, occupancy)
                        .map_err(|error| module_error("simple_planner.occupancy", error))?;
                }
                active_map_resolution = f64::from(grid.info.resolution);
                latest_occupancy_body = Some(frame.payload.clone());
            }
            message = map_pose.next() => {
                let frame = required_frame(message, "simple_planner.map_pose")?;
                let pose = match decode_planner_pose_input(
                    &frame.payload,
                    "simple_planner.map_pose",
                )? {
                    OwnerInput::Accepted(pose) => pose,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("simple_planner.map_pose", &reason);
                        continue;
                    }
                };
                if let OwnerInput::Rejected(reason) = validate_newer_source_timestamp(
                    last_pose_timestamp_ns,
                    pose.stamp.nanos,
                    "simple_planner.map_pose",
                )? {
                    diagnostics.reject("simple_planner.map_pose", &reason);
                    continue;
                }
                let pose_timestamp_ns = pose.stamp.nanos;
                if let Err(error) = Module::on_odometry(&mut module, pose) {
                    match module_input_error::<()>("simple_planner.map_pose", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("simple_planner.map_pose", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
                last_pose_timestamp_ns = pose_timestamp_ns;
            }
            message = goal_pose.next() => {
                let frame = required_frame(message, "simple_planner.goal_pose")?;
                let envelope = match schema::decode_goal_envelope_body(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        diagnostics.reject("simple_planner.goal_pose", &error.to_string());
                        continue;
                    }
                };
                let envelope = match goal_buffer.observe_goal(envelope) {
                    PlannerGoalIngress::Ready(envelope) => *envelope,
                    PlannerGoalIngress::Buffered => continue,
                    PlannerGoalIngress::Rejected(reason) => {
                        diagnostics.reject(
                            "simple_planner.goal_pose",
                            &format!("GoalEnvelope rejected: {reason:?}"),
                        );
                        continue;
                    }
                };
                let Some(occupancy_body) = latest_occupancy_body.as_ref() else {
                    continue;
                };
                let (envelope, boundary) = match apply_simple_planner_goal(
                    &mut module,
                    envelope,
                    occupancy_body,
                )? {
                    OwnerInput::Accepted(applied) => applied,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("simple_planner.goal_pose", &reason);
                        continue;
                    }
                };
                if !goal_buffer.mark_applied(&envelope) {
                    return Err(NavStackError::Internal(
                        "simple planner applied a superseded GoalEnvelope".into(),
                    ));
                }
                path_revision = 0;
                published_path_is_boundary_safe = false;
                active_boundary = boundary;
                active_goal = Some(envelope);
            }
            message = stop.next() => {
                let frame = required_frame(message, "simple_planner.stop_movement")?;
                let stop = match decode_planner_stop_input(
                    &frame.payload,
                    "simple_planner.stop_movement",
                )? {
                    OwnerInput::Accepted(stop) => stop,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("simple_planner.stop_movement", &reason);
                        continue;
                    }
                };
                if let Err(error) = Module::on_stop_movement(&mut module, stop) {
                    match module_input_error::<()>("simple_planner.stop_movement", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("simple_planner.stop_movement", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
            }
            _ = ticker.tick(), if goal_buffer.has_applied_goal() => {
                let output = Module::tick(&mut module)
                    .map_err(|error| module_tick_error("simple_planner.tick", error))?;
                let mut publication = SimpleOutputPublication {
                    active_goal: active_goal.as_ref(),
                    active_boundary: active_boundary.as_ref(),
                    map_resolution: active_map_resolution,
                    planner_runtime_generation,
                    path_revision: &mut path_revision,
                    published_path_is_boundary_safe: &mut published_path_is_boundary_safe,
                };
                publish_simple_output(
                    &transport,
                    &node,
                    output,
                    &mut publication,
                ).await?;
            }
        }
    }
    Module::stop(&mut module).map_err(|error| module_error("simple_planner.stop", error))
}

#[cfg(feature = "simple-2d")]
struct SimpleOutputPublication<'a> {
    active_goal: Option<&'a schema::GoalEnvelope>,
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
    node: &ModuleNode,
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
                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner path has no active GoalEnvelope".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary: publication.active_boundary,
                    map_resolution: publication.map_resolution,
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
                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner hold has no active GoalEnvelope".into(),
                        )
                    })?,
                    planner_runtime_generation: publication.planner_runtime_generation,
                    path_revision: next_revision,
                    path: &path,
                    boundary: publication.active_boundary,
                    map_resolution: publication.map_resolution,
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
pub(crate) fn astar_planner_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 4, &[4])?;
    let node = node.clone();
    Ok(boxed_owner(
        "astar-planner",
        Box::new(move |shutdown, ready| Box::pin(run_astar_planner(node, shutdown, ready))),
    ))
}

#[cfg(feature = "replanning-astar")]
async fn run_astar_planner(
    node: ModuleNode,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_astar::module::{
        Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
    };

    let transport = lcm_transport()?;
    let mut map_pose =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.GoalEnvelope").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let config = nuwax_nav_astar::GlobalPlannerConfig::default();
    let mut module = ReplanningAStarPlannerModule::new(config);
    NavigationInterface::start(&mut module)
        .map_err(|error| module_error("astar_planner", error))?;
    let mut has_odometry = false;
    let mut has_costmap = false;
    let planner_runtime_generation = saturating_i64(runtime_generation());
    let mut goal_buffer = PlannerGoalBuffer::default();
    let mut latest_costmap_body: Option<Vec<u8>> = None;
    let mut active_goal: Option<schema::GoalEnvelope> = None;
    let mut active_boundary: Option<NavigationBoundary> = None;
    let mut active_map_resolution = 0.05_f64;
    let mut path_revision = 0_i64;
    let mut last_pose_timestamp_ns = 0_i64;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut ticker = tokio::time::interval(Duration::from_millis(100));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = map_pose.next() => {
                let frame = required_frame(message, "astar_planner.map_pose")?;
                let map_pose = match decode_planner_pose_input(
                    &frame.payload,
                    "astar_planner.map_pose",
                )? {
                    OwnerInput::Accepted(pose) => pose,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("astar_planner.map_pose", &reason);
                        continue;
                    }
                };
                if let OwnerInput::Rejected(reason) = validate_newer_source_timestamp(
                    last_pose_timestamp_ns,
                    map_pose.stamp.nanos,
                    "astar_planner.map_pose",
                )? {
                    diagnostics.reject("astar_planner.map_pose", &reason);
                    continue;
                }
                let pose_timestamp_ns = map_pose.stamp.nanos;
                if let Err(error) = Module::on_odom(&mut module, map_pose) {
                    match module_input_error::<()>("astar_planner.map_pose", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("astar_planner.map_pose", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
                last_pose_timestamp_ns = pose_timestamp_ns;
                has_odometry = true;
                if has_costmap {
                    if let Some(envelope) = goal_buffer.take_ready_goal() {
                        let costmap_body = latest_costmap_body.as_ref().ok_or_else(|| {
                            NavStackError::Internal(
                                "A* planner marked a costmap ready without retaining its body"
                                    .into(),
                            )
                        })?;
                        let (envelope, boundary, map_resolution) =
                            match apply_astar_planner_goal(
                                &mut module,
                                *envelope,
                                costmap_body,
                            )? {
                                OwnerInput::Accepted(applied) => applied,
                                OwnerInput::Rejected(reason) => {
                                    diagnostics.reject("astar_planner.goal", &reason);
                                    continue;
                                }
                            };
                        if !goal_buffer.mark_applied(&envelope) {
                            return Err(NavStackError::Internal(
                                "A* planner applied a superseded GoalEnvelope".into(),
                            ));
                        }
                        active_map_resolution = map_resolution;
                        path_revision = 0;
                        active_boundary = boundary;
                        active_goal = Some(envelope);
                    }
                }
            }
            message = costmap.next() => {
                let frame = required_frame(message, "astar_planner.costmap")?;
                match decode_planner_grid_input(
                    &frame.payload,
                    "astar_planner.costmap",
                )? {
                    OwnerInput::Accepted(_) => {}
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("astar_planner.costmap", &reason);
                        continue;
                    }
                };
                let raw_costmap = match decode_astar_costmap(&frame.payload) {
                    Ok(costmap) => costmap,
                    Err(error) => {
                        diagnostics.reject("astar_planner.costmap", &error.to_string());
                        continue;
                    }
                };
                let ready_goal = goal_buffer.observe_costmap();
                if let Some(envelope) = ready_goal {
                    clear_planner_goal_for_costmap_advance(
                        &mut active_goal,
                        &mut active_boundary,
                        &mut path_revision,
                    );
                    if !has_odometry {
                        let raw_resolution = raw_costmap.resolution();
                        Module::on_costmap(&mut module, raw_costmap)
                            .map_err(|error| module_error("astar_planner.costmap", error))?;
                        active_map_resolution = raw_resolution;
                        if !goal_buffer.defer_ready_goal(*envelope) {
                            return Err(NavStackError::Internal(
                                "A* planner lost a GoalEnvelope while waiting for odometry"
                                    .into(),
                            ));
                        }
                    } else {
                        match apply_astar_planner_goal(&mut module, *envelope, &frame.payload)? {
                        OwnerInput::Accepted((envelope, boundary, map_resolution)) => {
                            if !goal_buffer.mark_applied(&envelope) {
                                return Err(NavStackError::Internal(
                                    "A* planner applied a superseded GoalEnvelope".into(),
                                ));
                            }
                            active_map_resolution = map_resolution;
                            path_revision = 0;
                            active_boundary = boundary;
                            active_goal = Some(envelope);
                        }
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("astar_planner.goal", &reason);
                            let raw_resolution = raw_costmap.resolution();
                            Module::on_costmap(&mut module, raw_costmap)
                                .map_err(|error| module_error("astar_planner.costmap", error))?;
                            active_map_resolution = raw_resolution;
                        }
                    }
                    }
                } else {
                    let boundary = if goal_buffer.has_applied_goal() {
                        active_boundary.as_ref()
                    } else {
                        None
                    };
                    let costmap = match boundary {
                        Some(boundary) => match decode_astar_costmap_with_boundary(
                            &frame.payload,
                            Some(boundary),
                        ) {
                            Ok(costmap) => costmap,
                            Err(error) => {
                                diagnostics.reject("astar_planner.costmap", &error.to_string());
                                continue;
                            }
                        },
                        None => raw_costmap,
                    };
                    active_map_resolution = costmap.resolution();
                    Module::on_costmap(&mut module, costmap)
                        .map_err(|error| module_error("astar_planner.costmap", error))?;
                }
                latest_costmap_body = Some(frame.payload.clone());
                has_costmap = true;
            }
            message = goal.next() => {
                let frame = required_frame(message, "astar_planner.goal")?;
                let envelope = match schema::decode_goal_envelope_body(&frame.payload) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        diagnostics.reject("astar_planner.goal", &error.to_string());
                        continue;
                    }
                };
                let envelope = match goal_buffer.observe_goal(envelope) {
                    PlannerGoalIngress::Ready(envelope) => *envelope,
                    PlannerGoalIngress::Buffered => continue,
                    PlannerGoalIngress::Rejected(reason) => {
                        diagnostics.reject(
                            "astar_planner.goal",
                            &format!("GoalEnvelope rejected: {reason:?}"),
                        );
                        continue;
                    }
                };
                if !has_odometry {
                    if !goal_buffer.defer_ready_goal(envelope) {
                        return Err(NavStackError::Internal(
                            "A* planner lost a GoalEnvelope while waiting for odometry".into(),
                        ));
                    }
                    continue;
                }
                let Some(costmap_body) = latest_costmap_body.as_ref() else {
                    if !goal_buffer.defer_ready_goal(envelope) {
                        return Err(NavStackError::Internal(
                            "A* planner lost a GoalEnvelope while waiting for its costmap body"
                                .into(),
                        ));
                    }
                    continue;
                };
                let (envelope, boundary, map_resolution) = match apply_astar_planner_goal(
                    &mut module,
                    envelope,
                    costmap_body,
                )? {
                    OwnerInput::Accepted(applied) => applied,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("astar_planner.goal", &reason);
                        continue;
                    }
                };
                if !goal_buffer.mark_applied(&envelope) {
                    return Err(NavStackError::Internal(
                        "A* planner applied a superseded GoalEnvelope".into(),
                    ));
                }
                active_map_resolution = map_resolution;
                path_revision = 0;
                active_boundary = boundary;
                active_goal = Some(envelope);
            }
            message = stop.next() => {
                let frame = required_frame(message, "astar_planner.stop_movement")?;
                let stop = match decode_planner_stop_input(
                    &frame.payload,
                    "astar_planner.stop_movement",
                )? {
                    OwnerInput::Accepted(stop) => stop,
                    OwnerInput::Rejected(reason) => {
                        diagnostics.reject("astar_planner.stop_movement", &reason);
                        continue;
                    }
                };
                if let Err(error) = Module::on_stop_movement(&mut module, stop) {
                    match module_input_error::<()>("astar_planner.stop_movement", error)? {
                        OwnerInput::Rejected(reason) => {
                            diagnostics.reject("astar_planner.stop_movement", &reason);
                            continue;
                        }
                        OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
                    }
                }
            }
            _ = ticker.tick(), if has_odometry && has_costmap && goal_buffer.has_applied_goal() => {
                let output = Module::tick(&mut module)
                    .map_err(|error| module_tick_error("astar_planner.tick", error))?;
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
                    let next_revision = path_revision.checked_add(1).ok_or_else(|| {
                        NavStackError::Internal("A* planner path revision exhausted".into())
                    })?;
                    let published = publish_versioned_path(
                        &transport,
                        &node.outputs[1],
                        VersionedPathPublication {
                            goal: active_goal.as_ref().ok_or_else(|| {
                                NavStackError::Internal(
                                    "A* path has no active GoalEnvelope".into(),
                                )
                            })?,
                            planner_runtime_generation,
                            path_revision: next_revision,
                            path: &path,
                            boundary: active_boundary.as_ref(),
                            map_resolution: active_map_resolution,
                        },
                    ).await?;
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
    NavigationInterface::set_replanning_enabled(&mut module, true)
        .map_err(|error| module_error("astar_planner.restore_replanning", error))?;
    NavigationInterface::reset_safe_goal_clearance(&mut module)
        .map_err(|error| module_error("astar_planner.reset_safe_goal_clearance", error))?;
    NavigationInterface::stop(&mut module)
        .map_err(|error| module_error("astar_planner.stop", error))
}

#[cfg(feature = "patrolling")]
pub(crate) fn patrolling_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 6, &[4])?;
    let node = node.clone();
    Ok(boxed_owner(
        "patrolling",
        Box::new(move |shutdown, ready| Box::pin(run_patrolling(node, shutdown, ready))),
    ))
}

#[cfg(feature = "patrolling")]
async fn run_patrolling(
    node: ModuleNode,
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
    node: &ModuleNode,
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
    node: &ModuleNode,
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
pub(crate) fn basic_path_follower_owner(
    node: &ModuleNode,
) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 3, &[1, 2])?;
    let node = node.clone();
    Ok(boxed_owner(
        "basic-path-follower",
        Box::new(move |shutdown, ready| Box::pin(run_basic_path_follower(node, shutdown, ready))),
    ))
}

#[cfg(feature = "basic-path-follower")]
async fn run_basic_path_follower(
    node: ModuleNode,
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

#[cfg(feature = "movement-manager")]
pub(crate) fn movement_manager_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, node.inputs.len(), &[2, 4])?;
    if !matches!(node.inputs.len(), 2 | 3) {
        return Err(NavStackError::InvalidConfig(format!(
            "movement_manager requires two or three inputs, got {}",
            node.inputs.len()
        )));
    }
    let node = node.clone();
    Ok(boxed_owner(
        "movement-manager",
        Box::new(move |shutdown, ready| Box::pin(run_movement_manager(node, shutdown, ready))),
    ))
}

#[cfg(feature = "movement-manager")]
async fn run_movement_manager(
    node: ModuleNode,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_movement_manager::module::{MovementManagerModule, MovementManagerModuleConfig};

    let transport = lcm_transport()?;
    let mut nav = typed_stream(&transport, &node.inputs[0], "geometry_msgs.Twist").await?;
    let mut joy = typed_stream(&transport, &node.inputs[1], "geometry_msgs.Twist").await?;
    let mut clicked = if node.inputs.len() == 3 {
        Some(typed_stream(&transport, &node.inputs[2], "geometry_msgs.PointStamped").await?)
    } else {
        None
    };
    let mut module = MovementManagerModule::new(MovementManagerModuleConfig::default())
        .map_err(|error| module_error("movement_manager", error))?;
    let nav_input = module.nav_cmd_vel_input();
    let teleop_input = module.tele_cmd_vel_input();
    let click_input = module.clicked_point_input();
    let mut commands = module.subscribe_cmd_vel();
    let mut goals = module.subscribe_goal();
    let mut waypoints = module.subscribe_way_point();
    let mut stops = module.subscribe_stop_movement();
    module
        .start()
        .map_err(|error| module_error("movement_manager", error))?;
    signal_ready(ready)?;

    let stop_topic = if node.outputs.len() == 4 {
        &node.outputs[3]
    } else {
        &node.outputs[1]
    };
    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = nav.next() => {
                let frame = required_frame(message, "movement_manager.nav_cmd_vel")?;
                nav_input.send(decode_twist3_body(&frame.payload)?)
                    .map_err(|_| NavStackError::Internal("movement_manager nav port closed".into()))?;
            }
            message = joy.next() => {
                let frame = required_frame(message, "movement_manager.joy_cmd")?;
                teleop_input.send(decode_twist3_body(&frame.payload)?)
                    .map_err(|_| NavStackError::Internal("movement_manager teleop port closed".into()))?;
            }
            message = optional_next(&mut clicked) => {
                let frame = required_frame(message, "movement_manager.clicked_point")?;
                click_input.send(decode_point_body(&frame.payload)?)
                    .map_err(|_| NavStackError::Internal("movement_manager click port closed".into()))?;
            }
            command = commands.recv() => {
                let command = command.map_err(|error| NavStackError::Internal(format!(
                    "movement_manager command port: {error}"
                )))?;
                publish_twist3(&transport, &node.outputs[0], command).await?;
            }
            goal = goals.recv() => {
                let goal = goal.map_err(|error| NavStackError::Internal(format!(
                    "movement_manager goal port: {error}"
                )))?;
                if node.outputs.len() == 4 {
                    publish_point(&transport, &node.outputs[1], &goal).await?;
                }
            }
            waypoint = waypoints.recv() => {
                let waypoint = waypoint.map_err(|error| NavStackError::Internal(format!(
                    "movement_manager waypoint port: {error}"
                )))?;
                if node.outputs.len() == 4 {
                    publish_point(&transport, &node.outputs[2], &waypoint).await?;
                }
            }
            stop = stops.recv() => {
                let stop = stop.map_err(|error| NavStackError::Internal(format!(
                    "movement_manager stop port: {error}"
                )))?;
                publish_bool(&transport, stop_topic, stop).await?;
            }
        }
    }
    module
        .stop()
        .map_err(|error| module_error("movement_manager.stop", error))?;
    publish_twist3(&transport, &node.outputs[0], Twist3::ZERO).await
}

fn module_error(component: &str, error: impl std::fmt::Display) -> NavStackError {
    NavStackError::Internal(format!("{component}: {error}"))
}

#[cfg(feature = "movement-manager")]
fn decode_point_body(body: &[u8]) -> NavStackResult<PointStamped> {
    let point = schema::decode_point_stamped_body(body)
        .map_err(|error| NavStackError::Internal(format!("PointStamped decode: {error}")))?;
    Ok(PointStamped::new(
        NavFrameId::from_static(&point.header.frame_id),
        timestamp(&point.header)?,
        Point3::new(point.point[0], point.point[1], point.point[2]),
    ))
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
    feature = "mls",
    all(test, feature = "replanning-astar")
))]
fn decode_path_body(body: &[u8]) -> NavStackResult<Path2D> {
    let path = schema::decode_path(&prefixed(schema::PATH_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Path decode: {error}")))?;
    path_from_schema(&path)
}

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "guard-runtime"
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

#[cfg(feature = "movement-manager")]
fn decode_twist3_body(body: &[u8]) -> NavStackResult<Twist3> {
    let twist = schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Twist decode: {error}")))?;
    Ok(Twist3 {
        linear: Point3::new(twist.linear[0], twist.linear[1], twist.linear[2]),
        angular: Point3::new(twist.angular[0], twist.angular[1], twist.angular[2]),
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn decode_cloud_body(body: &[u8]) -> NavStackResult<PointCloud> {
    let cloud = schema::decode_point_cloud2(&prefixed(schema::POINT_CLOUD2_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PointCloud2 decode: {error}")))?;
    point_cloud_from_schema(cloud)
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
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

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn cloud_field<'a>(
    fields: &'a [schema::PointField],
    name: &str,
) -> NavStackResult<&'a schema::PointField> {
    fields
        .iter()
        .find(|field| field.name == name && field.count >= 1)
        .ok_or_else(|| NavStackError::InvalidConfig(format!("PointCloud2 missing {name} field")))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
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
fn decode_simple_occupancy(body: &[u8]) -> NavStackResult<PointCloud> {
    decode_simple_occupancy_with_boundary(body, None)
}

#[cfg(feature = "simple-2d")]
fn decode_simple_occupancy_with_boundary(
    body: &[u8],
    boundary: Option<&NavigationBoundary>,
) -> NavStackResult<PointCloud> {
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
    let mut points = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let value = grid.data[y * width + x];
            let center = Point2::new(
                grid.info.origin.position[0] + (x as f64 + 0.5) * resolution,
                grid.info.origin.position[1] + (y as f64 + 0.5) * resolution,
            );
            if value >= 50 || boundary.is_some_and(|boundary| !boundary.contains(center)) {
                points.push(Point3::new(center.x, center.y, 0.0));
            }
        }
    }
    Ok(PointCloud {
        stamp: timestamp(&grid.header)?,
        frame_id: NavFrameId::Map,
        points,
        intensities: Vec::new(),
    })
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
                } else if value >= 100 {
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

#[cfg(any(feature = "simple-2d", feature = "movement-manager"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
struct VersionedPathPublication<'a> {
    goal: &'a schema::GoalEnvelope,
    planner_runtime_generation: i64,
    path_revision: i64,
    path: &'a Path2D,
    boundary: Option<&'a NavigationBoundary>,
    map_resolution: f64,
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn build_versioned_path_envelope(
    publication: VersionedPathPublication<'_>,
    source_timestamp_ns: i64,
) -> NavStackResult<Option<schema::PathEnvelope>> {
    if !path_respects_navigation_boundary(
        publication.path,
        publication.boundary,
        publication.map_resolution * 0.5,
    ) {
        return Ok(None);
    }
    let stamp = Timestamp::from_unix(
        source_timestamp_ns / 1_000_000_000,
        (source_timestamp_ns % 1_000_000_000) as u32,
    );
    let header = schema_header(stamp, NavFrameId::Map.as_str())?;
    let mut envelope = schema::PathEnvelope {
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: publication.goal.session_id.clone(),
        owner_kind: publication.goal.owner_kind,
        parent_request_id: publication.goal.parent_request_id.clone(),
        goal_generation: publication.goal.goal_generation,
        planner_runtime_generation: publication.planner_runtime_generation,
        path_revision: publication.path_revision,
        source_timestamp_ns,
        goal_payload_sha256: publication.goal.payload_sha256,
        path_payload_sha256: [0; 32],
        path: schema::Path {
            header: header.clone(),
            poses: publication
                .path
                .points
                .iter()
                .map(|point| schema::PoseStamped {
                    header: header.clone(),
                    pose: schema::Pose {
                        position: [point.x, point.y, 0.0],
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                })
                .collect(),
        },
    };
    schema::seal_path_envelope(&mut envelope)
        .map_err(|error| NavStackError::InvalidConfig(format!("PathEnvelope seal: {error}")))?;
    Ok(Some(envelope))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
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

async fn publish_bool(transport: &LcmTransport, channel: &str, value: bool) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "std_msgs.Bool",
        schema::encode_bool_body(&schema::Bool { data: value }),
    )
    .await
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

#[cfg(feature = "movement-manager")]
async fn publish_twist3(
    transport: &LcmTransport,
    channel: &str,
    twist: Twist3,
) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "geometry_msgs.Twist",
        schema::encode_twist_body(&schema::Twist {
            linear: [twist.linear.x, twist.linear.y, twist.linear.z],
            angular: [twist.angular.x, twist.angular.y, twist.angular.z],
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
fn planner_test_goal(session_id: &str, goal_generation: i64) -> schema::GoalEnvelope {
    let mut goal = schema::GoalEnvelope {
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: session_id.into(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: "explore-parent".into(),
        goal_generation,
        producer_runtime_generation: 17,
        source_timestamp_ns: goal_generation * 1_000_000_000,
        boundary_frame_id: "map".into(),
        boundary_points: vec![
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0],
            [0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        motion_guard_generation: 7,
        motion_guard_boot_nonce: [7; 16],
        motion_stop_generation: 1,
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
        ..schema::GoalEnvelope::default()
    };
    schema::seal_goal_envelope(&mut goal).expect("seal planner test goal");
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
mod planner_goal_buffer_tests {
    use super::*;

    struct GeometryProducerFixture {
        producer_runtime_generation: i64,
    }

    impl GeometryProducerFixture {
        fn goal(&self, session_id: &str, goal_generation: i64) -> schema::GoalEnvelope {
            let mut goal = planner_test_goal(session_id, goal_generation);
            goal.producer_runtime_generation = self.producer_runtime_generation;
            goal.payload_sha256 = [0; 32];
            schema::seal_goal_envelope(&mut goal).expect("seal producer fixture goal");
            goal
        }
    }

    #[test]
    fn retained_nav_owner_accepts_new_goal_after_geometry_producer_restart() {
        let mut nav_owner = PlannerGoalBuffer::default();
        let first_goal = {
            let first_geometry = GeometryProducerFixture {
                producer_runtime_generation: 41,
            };
            assert!(nav_owner.observe_costmap().is_none());
            let first_goal = first_geometry.goal("before-restart", 7);
            let ready_first = match nav_owner.observe_goal(first_goal.clone()) {
                PlannerGoalIngress::Ready(goal) => *goal,
                outcome => panic!("first producer goal must be ready: {outcome:?}"),
            };
            assert!(nav_owner.mark_applied(&ready_first));
            first_goal
        };

        // The nav owner and its ordering memory remain alive while the geometry
        // producer is recreated. Goal generation may restart at one because the
        // durably allocated producer epoch is strictly newer.
        let restarted_geometry = GeometryProducerFixture {
            producer_runtime_generation: 42,
        };
        let restarted_goal = restarted_geometry.goal("after-restart", 1);
        let ready_restarted = match nav_owner.observe_goal(restarted_goal.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("restarted producer goal must activate: {outcome:?}"),
        };
        assert!(!nav_owner.has_applied_goal());
        assert_eq!(ready_restarted, restarted_goal);
        assert!(nav_owner.mark_applied(&ready_restarted));
        assert!(nav_owner.has_applied_goal());
        assert!(matches!(
            nav_owner.observe_goal(first_goal),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::SupersededLease)
        ));
    }

    #[test]
    fn goal_generation_orders_the_newest_goal_lease() {
        let mut buffer = PlannerGoalBuffer::default();
        assert!(buffer.observe_costmap().is_none());

        let first = planner_test_goal("goal-1", 1);
        let ready_first = match buffer.observe_goal(first.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("first lease must use the observed costmap: {outcome:?}"),
        };
        assert!(buffer.mark_applied(&ready_first));

        let second = planner_test_goal("goal-2", 2);
        let ready_second = match buffer.observe_goal(second.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("newer goal generation must not be buffered: {outcome:?}"),
        };
        assert!(!buffer.has_applied_goal());
        assert!(buffer.mark_applied(&ready_second));
        assert!(matches!(
            buffer.observe_goal(first),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::SupersededLease)
        ));
        assert!(buffer.observe_costmap().is_none());
        assert!(buffer.observe_costmap().is_none());
        assert!(buffer.has_applied_goal());

        let mut cold = PlannerGoalBuffer::default();
        let pending = planner_test_goal("pending", 1);
        assert!(matches!(
            cold.observe_goal(pending.clone()),
            PlannerGoalIngress::Buffered
        ));
        let released = *cold
            .observe_costmap()
            .expect("any valid costmap must release the pending lease");
        assert_eq!(released, pending);
    }

    #[test]
    fn applied_goal_remains_active_when_the_costmap_updates() {
        let mut buffer = PlannerGoalBuffer::default();
        assert!(buffer.observe_costmap().is_none());
        let goal = planner_test_goal("active-goal", 1);
        let ready = match buffer.observe_goal(goal) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("goal must be ready: {outcome:?}"),
        };
        assert!(buffer.mark_applied(&ready));

        assert!(buffer.observe_costmap().is_none());
        assert!(buffer.has_applied_goal());
        assert!(buffer.pending_goal.is_none());
    }

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
    fn planner_sensor_validation_rejects_frame_nonfinite_and_timestamp_replay() {
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

        assert!(matches!(
            validate_newer_source_timestamp(40, 41, "planner.map_pose").expect("new timestamp"),
            OwnerInput::Accepted(())
        ));
        assert!(matches!(
            validate_newer_source_timestamp(41, 41, "planner.map_pose")
                .expect("duplicate timestamp is external"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(
            validate_newer_source_timestamp(41, 40, "planner.map_pose")
                .expect("regressive timestamp is external"),
            OwnerInput::Rejected(_)
        ));
    }
}

#[cfg(all(test, feature = "simple-2d"))]
mod simple_tests {
    use nuwax_nav_simple_planner::module::{Module, ModuleOutput, SimplePlannerModule};

    use super::*;

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

        let cloud = decode_simple_occupancy(&body).expect("decode persistent occupancy");
        assert_eq!(cloud.frame_id, NavFrameId::Map);
        assert_eq!(cloud.points, vec![Point3::new(-0.75, -1.25, 0.0)]);
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
    fn simple_planner_can_leave_a_free_island_through_unknown_cells() {
        let mut data = vec![-1; 25];
        data[12] = 0;
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
        let cloud = decode_simple_occupancy(&body).expect("decode unknown cells as traversable");
        assert!(cloud.points.is_empty());

        let mut planner =
            SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
                cell_size: 1.0,
                inflation_radius: 0.0,
                ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
            })
            .expect("frontier planner config");
        Module::start(&mut planner).expect("start simple planner");
        Module::on_terrain_map_ext(&mut planner, cloud).expect("frontier occupancy");
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

        let ModuleOutput::Plan { path, .. } = Module::tick(&mut planner).expect("planner tick")
        else {
            panic!("unknown cells must not isolate a reachable frontier goal");
        };
        assert!(path.points.len() >= 3);
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
        let expected_hash = goal.payload_sha256;
        let expected_boundary = goal.boundary_points.clone();
        let mut buffer = PlannerGoalBuffer::default();
        assert!(matches!(
            buffer.observe_goal(goal.clone()),
            PlannerGoalIngress::Buffered
        ));
        assert_eq!(buffer.pending_goal.as_ref(), Some(&goal));

        let ready = buffer
            .observe_costmap()
            .expect("first valid costmap must release the buffered simple goal");
        assert_eq!(ready.payload_sha256, expected_hash);
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
            match apply_simple_planner_goal(&mut planner, *ready, &planner_test_grid_body(21))
                .expect("apply simple buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid simple goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert!(buffer.mark_applied(&applied));
        assert!(buffer.has_applied_goal());
        assert!(matches!(
            Module::tick(&mut planner).expect("simple planner tick"),
            ModuleOutput::Plan { .. } | ModuleOutput::SafetyHold { .. }
        ));
        assert!(buffer.observe_costmap().is_none());
        assert!(buffer.pending_goal.is_none());
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

        let mut buffer = PlannerGoalBuffer::default();
        assert!(buffer.observe_costmap().is_none());
        Module::on_terrain_map_ext(
            &mut planner,
            decode_simple_occupancy(&planner_test_grid_body(12)).expect("map 12"),
        )
        .expect("apply map 12");
        let goal_12 = planner_test_goal("simple-12", 1);
        let ready_12 = match buffer.observe_goal(goal_12.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("goal 12 must be ready, got {outcome:?}"),
        };
        let (applied_12, boundary_12) =
            match apply_simple_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert!(buffer.mark_applied(&applied_12));
        let mut active_goal = Some(applied_12);
        let mut active_boundary = boundary_12;
        let mut path_revision = 4_i64;
        let mut published_path_is_boundary_safe = true;
        assert!(published_path_is_boundary_safe);

        let goal_13 = planner_test_goal("simple-13", 2);
        let ready_13 = match buffer.observe_goal(goal_13.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("newer lease must be ready: {outcome:?}"),
        };
        assert!(!buffer.has_applied_goal(), "superseded path must be gated");
        assert!(
            active_goal.is_some(),
            "outer map-a lease is not replaced early"
        );
        assert!(active_boundary.is_some());
        assert_eq!(path_revision, 4);
        assert!(published_path_is_boundary_safe);

        clear_planner_goal_for_costmap_advance(
            &mut active_goal,
            &mut active_boundary,
            &mut path_revision,
        );
        published_path_is_boundary_safe = false;
        assert!(
            !buffer.has_applied_goal(),
            "tick/path publication stays gated"
        );
        assert!(active_goal.is_none());
        assert!(active_boundary.is_none());
        assert_eq!(path_revision, 0);
        assert!(!published_path_is_boundary_safe);

        let (applied_13, boundary_13) =
            match apply_simple_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        assert!(buffer.mark_applied(&applied_13));
        active_goal = Some(applied_13);
        active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("simple-13")
        );
        assert!(active_boundary.is_some());
        assert!(buffer.has_applied_goal());
        let path = match Module::tick(&mut planner).expect("simple map-b tick") {
            ModuleOutput::Plan { path, .. } | ModuleOutput::SafetyHold { path, .. } => path,
            output => panic!("expected map-b path, got {output:?}"),
        };
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 23,
                path_revision: 1,
                path: &path,
                boundary: active_boundary.as_ref(),
                map_resolution: 1.0,
            },
            13_000_000_000,
        )
        .expect("build map-b PathEnvelope")
        .expect("map-b path stays inside boundary");
        assert_eq!(
            envelope.goal_payload_sha256,
            active_goal.as_ref().unwrap().payload_sha256
        );
        assert!(matches!(
            buffer.observe_goal(goal_12),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::SupersededLease)
        ));
        assert!(buffer.observe_costmap().is_none());
        assert!(matches!(
            buffer.observe_goal(goal_13),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::DuplicateLease)
        ));
        assert!(buffer.observe_costmap().is_none());
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
        Module::on_terrain_map_ext(&mut planner, occupancy)
            .expect("simple planner accepts valid grid");
    }
}

#[cfg(all(test, feature = "guard-runtime"))]
mod guarded_tests {
    use super::*;
    use futures::channel::mpsc::{unbounded, UnboundedSender};
    use nuwax_nav_runtime::{NavSchedulerHandle, RuntimeError, SchedulerPhase, TerminalOutcome};
    use tokio::task::JoinHandle;

    fn active_goal(session_id: &str) -> schema::GoalEnvelope {
        schema::GoalEnvelope {
            session_id: session_id.into(),
            ..schema::GoalEnvelope::default()
        }
    }

    fn sealed_goal(session_id: &str) -> schema::GoalEnvelope {
        let mut goal = schema::GoalEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: session_id.into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "public-api-request-1".into(),
            goal_generation: 3,
            producer_runtime_generation: 7,
            source_timestamp_ns: 1_000_000_000,
            motion_guard_generation: 7,
            motion_guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
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
            ..schema::GoalEnvelope::default()
        };
        schema::seal_goal_envelope(&mut goal).expect("seal goal");
        goal
    }

    fn sealed_path(goal: &schema::GoalEnvelope) -> schema::PathEnvelope {
        let mut path = schema::PathEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            planner_runtime_generation: 9,
            path_revision: 1,
            source_timestamp_ns: 2_000_000_000,
            goal_payload_sha256: goal.payload_sha256,
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
        schema::seal_path_envelope(&mut path).expect("seal path");
        path
    }

    fn rollover_goal(
        previous: &schema::GoalEnvelope,
        session_id: &str,
        producer_runtime_generation: i64,
        goal_generation: i64,
    ) -> schema::GoalEnvelope {
        let mut goal = previous.clone();
        goal.session_id = session_id.into();
        goal.producer_runtime_generation = producer_runtime_generation;
        goal.goal_generation = goal_generation;
        goal.source_timestamp_ns = goal.source_timestamp_ns.saturating_add(1_000_000_000);
        goal.payload_sha256 = [0; 32];
        schema::seal_goal_envelope(&mut goal).expect("seal rollover goal");
        goal
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
    struct RecordingMotion(Arc<Mutex<Vec<String>>>);

    impl nuwax_nav_runtime::NavMotionPort for RecordingMotion {
        fn acquire<'life0, 'life1, 'life2, 'async_trait>(
            &'life0 mut self,
            operation_id: &'life1 str,
            admission: &'life2 nuwax_nav_runtime::NavMotionAdmission,
        ) -> Pin<
            Box<
                dyn Future<
                        Output = Result<
                            nuwax_nav_runtime::LeaseProof,
                            nuwax_nav_runtime::MotionPortError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            'life2: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                self.0
                    .lock()
                    .unwrap()
                    .push(format!("acquire:{operation_id}"));
                Ok(nuwax_nav_runtime::LeaseProof {
                    lease_id: 11,
                    guard_generation: admission.guard_generation,
                })
            })
        }

        fn renew<'life0, 'life1, 'async_trait>(
            &'life0 mut self,
            operation_id: &'life1 str,
        ) -> Pin<
            Box<
                dyn Future<Output = Result<(), nuwax_nav_runtime::MotionPortError>>
                    + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                self.0.lock().unwrap().push(format!("renew:{operation_id}"));
                Ok(())
            })
        }

        fn submit_twist<'life0, 'life1, 'async_trait>(
            &'life0 mut self,
            operation_id: &'life1 str,
            _twist: Twist2,
        ) -> Pin<
            Box<
                dyn Future<
                        Output = Result<
                            nuwax_nav_runtime::AppliedProof,
                            nuwax_nav_runtime::MotionPortError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                self.0.lock().unwrap().push(format!("twist:{operation_id}"));
                Ok(nuwax_nav_runtime::AppliedProof {
                    operation_id: operation_id.into(),
                    applied_monotonic_ns: 99,
                })
            })
        }

        fn revoke_and_zero<'life0, 'life1, 'async_trait>(
            &'life0 mut self,
            operation_id: &'life1 str,
        ) -> Pin<
            Box<
                dyn Future<
                        Output = Result<
                            nuwax_nav_runtime::FenceProof,
                            nuwax_nav_runtime::MotionPortError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                self.0.lock().unwrap().push(format!("zero:{operation_id}"));
                Ok(nuwax_nav_runtime::FenceProof {
                    zero_ack_id: "zero-pending-goal".into(),
                    applied_monotonic_ns: 100,
                    lease: nuwax_nav_runtime::FenceLeaseState::Idle,
                })
            })
        }
    }

    struct CommandingPlanner(Arc<std::sync::atomic::AtomicBool>);

    impl nuwax_nav_runtime::NavPlanner for CommandingPlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {}

        fn update_costmap(&mut self, _costmap: nuwax_nav_astar::navigation_map::CostMap2D) {}

        fn step(&mut self, _robot_width_m: f64) -> nuwax_nav_runtime::PlannerStep {
            if self.0.swap(false, std::sync::atomic::Ordering::SeqCst) {
                return nuwax_nav_runtime::PlannerStep::Succeeded;
            }
            nuwax_nav_runtime::PlannerStep::Command(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.1,
            })
        }

        fn stop(&mut self) {}
    }

    type GuardedTestInput = (TransportResult<RawLcmFrame>, Option<oneshot::Sender<()>>);
    type GuardedTestSender = UnboundedSender<GuardedTestInput>;

    struct GuardedLoopHarness {
        goals: GuardedTestSender,
        poses: GuardedTestSender,
        paths: GuardedTestSender,
        costmaps: GuardedTestSender,
        cancels: GuardedTestSender,
        shutdown: Option<oneshot::Sender<()>>,
        loop_task: JoinHandle<NavStackResult<()>>,
        scheduler: NavSchedulerHandle,
        scheduler_task: JoinHandle<Result<(), RuntimeError>>,
        motion_calls: Arc<Mutex<Vec<String>>>,
        planner_succeeds: Arc<std::sync::atomic::AtomicBool>,
    }

    impl GuardedLoopHarness {
        async fn spawn() -> Self {
            static NEXT_LOOP_ID: std::sync::atomic::AtomicU64 =
                std::sync::atomic::AtomicU64::new(0);
            let loop_id = NEXT_LOOP_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let port = 20_000 + ((u64::from(std::process::id()) * 97 + loop_id) % 40_000) as u16;
            let transport = LcmTransport::new_real(&format!("udpm://239.255.76.67:{port}?ttl=1"))
                .expect("guarded loop test LCM");
            let motion_calls = Arc::new(Mutex::new(Vec::<String>::new()));
            let planner_succeeds = Arc::new(std::sync::atomic::AtomicBool::new(false));
            let scheduler = nuwax_nav_runtime::NavScheduler::new(
                RecordingMotion(Arc::clone(&motion_calls)),
                CommandingPlanner(Arc::clone(&planner_succeeds)),
                nuwax_nav_runtime::SchedulerConfig {
                    tick_interval: Duration::from_millis(5),
                    renew_interval: Duration::from_millis(100),
                    robot_width_m: 0.3,
                    pose_ttl: Duration::from_secs(5),
                    costmap_ttl: Duration::from_secs(5),
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
            let mut node = ModuleNode::new(
                format!("guarded-loop-test-{loop_id}"),
                "guarded loop test",
                crate::graph::ModuleKind::Coordinator,
            );
            node.outputs = vec![
                format!("/test/guarded/{loop_id}/cmd"),
                format!("/test/guarded/{loop_id}/state"),
                format!("/test/guarded/{loop_id}/reached"),
                format!("/test/guarded/{loop_id}/cancelled"),
            ];
            let loop_task = tokio::spawn(async move {
                run_guarded_nav_loop(
                    &node,
                    &transport,
                    &loop_scheduler,
                    shutdown_rx,
                    goal_stream,
                    pose_stream,
                    path_stream,
                    costmap_stream,
                    cancel_stream,
                    status,
                )
                .await
            });

            Self {
                goals,
                poses,
                paths,
                costmaps,
                cancels,
                shutdown: Some(shutdown),
                loop_task,
                scheduler,
                scheduler_task,
                motion_calls,
                planner_succeeds,
            }
        }

        fn send_goal(&self, goal: &schema::GoalEnvelope) {
            send_guarded_test_body(
                &self.goals,
                "nuwax_msgs.GoalEnvelope",
                schema::encode_goal_envelope_body(goal).expect("encode guarded test goal"),
            );
        }

        async fn send_goal_and_wait_dequeued(&self, goal: &schema::GoalEnvelope) {
            let dequeued = send_guarded_test_body_observed(
                &self.goals,
                "nuwax_msgs.GoalEnvelope",
                schema::encode_goal_envelope_body(goal).expect("encode guarded test goal"),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded goal was not dequeued")
                .expect("guarded goal dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn send_pose(&self, stamp_sec: i32) {
            let body = schema::encode_pose_stamped_body(&schema::PoseStamped {
                header: schema::Header {
                    seq: stamp_sec,
                    stamp_sec,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [0.0, 0.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            })
            .expect("encode guarded test pose");
            send_guarded_test_body(&self.poses, "geometry_msgs.PoseStamped", body);
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

        fn send_cancel(&self, goal: &schema::GoalEnvelope, cancel_generation: i64) {
            send_guarded_test_body(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                schema::encode_cancel_navigation_body(&schema::CancelNavigation {
                    session_id: goal.session_id.clone(),
                    reason: 1,
                    cancel_generation,
                    owner_kind: goal.owner_kind,
                    parent_request_id: goal.parent_request_id.clone(),
                    goal_generation: goal.goal_generation,
                    producer_runtime_generation: goal.producer_runtime_generation,
                })
                .expect("encode guarded test cancellation"),
            );
        }

        async fn wait_for_session(&self, session_id: &str) {
            tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    if self.scheduler.status().session_id.as_deref() == Some(session_id) {
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

        async fn wait_for_zero(&self) {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    if self
                        .motion_calls
                        .lock()
                        .unwrap()
                        .iter()
                        .any(|call| call.starts_with("zero:"))
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
                    "verified ZERO fence was not observed; status={:?} calls={:?}",
                    self.scheduler.status(),
                    self.motion_calls.lock().unwrap().clone()
                )
            });
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

    fn guarded_test_stream() -> (GuardedTestSender, LcmFrameStream) {
        let (sender, receiver) = unbounded::<GuardedTestInput>();
        let receiver = receiver.map(|(frame, dequeued)| {
            if let Some(dequeued) = dequeued {
                let _ = dequeued.send(());
            }
            frame
        });
        (sender, Box::pin(receiver))
    }

    fn send_guarded_test_body(sender: &GuardedTestSender, type_name: &str, body: Vec<u8>) {
        send_guarded_test_body_with_receipt(sender, type_name, body, None);
    }

    fn send_guarded_test_body_observed(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
    ) -> oneshot::Receiver<()> {
        let (dequeued, receipt) = oneshot::channel();
        send_guarded_test_body_with_receipt(sender, type_name, body, Some(dequeued));
        receipt
    }

    fn send_guarded_test_body_with_receipt(
        sender: &GuardedTestSender,
        type_name: &str,
        body: Vec<u8>,
        dequeued: Option<oneshot::Sender<()>>,
    ) {
        let frame = RawLcmFrameBuilder::new(
            LcmTopic::new("/test/guarded/input").expect("guarded test topic"),
            LcmMessageType::new(type_name).expect("guarded test message type"),
        )
        .payload(body)
        .build();
        sender
            .unbounded_send((Ok(frame), dequeued))
            .expect("guarded test input stream open");
    }

    #[derive(Clone, Copy)]
    enum ColdInput {
        Goal,
        Pose,
        Costmap,
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
                    harness.assert_no_session().await;
                }
            }
            harness.wait_for_session("cold-order-goal").await;
            harness.shutdown().await;
        }
    }

    async fn activate_map_a(harness: &GuardedLoopHarness) -> schema::GoalEnvelope {
        let goal_a = sealed_goal("session-a");
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.send_goal(&goal_a);
        harness.wait_for_session("session-a").await;
        goal_a
    }

    async fn finish_map_a_after_verified_zero(
        harness: &GuardedLoopHarness,
        goal_a: &schema::GoalEnvelope,
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
                || harness.scheduler.status().last_zero_ack_id.is_none()
            {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("verified ZERO did not make map-a terminal");
        tokio::time::sleep(Duration::from_millis(30)).await;
    }

    #[tokio::test]
    async fn guarded_terminal_goal_reuses_fresh_inputs_before_costmap_update() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        finish_map_a_after_verified_zero(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);

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
        finish_map_a_after_verified_zero(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);

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
        finish_map_a_after_verified_zero(&harness, &goal_a).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);

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
        finish_map_a_after_verified_zero(&harness, &goal_a).await;

        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_conflicting_goal_does_not_block_a_later_valid_lease() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;

        let conflicting = rollover_goal(
            &goal_a,
            "conflicting-session",
            goal_a.producer_runtime_generation,
            goal_a.goal_generation,
        );
        harness.send_goal(&conflicting);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some("session-a"),
            "the conflicting lease must be rejected without preempting map A"
        );

        harness.send_pose(2);
        harness.wait_for_pose(2).await;

        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_zero().await;
        harness.wait_for_session("session-b").await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_goal_replacement_waits_for_zero() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal_a = activate_map_a(&harness).await;
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);

        harness.send_goal_and_wait_dequeued(&goal_b).await;
        harness.wait_for_zero().await;
        harness.wait_for_session("session-b").await;
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
        assert!(harness.scheduler.status().last_zero_ack_id.is_none());
        assert!(!harness
            .motion_calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("zero:")));
        harness.shutdown().await;
    }

    #[test]
    fn terminal_cleanup_allows_continuous_a_then_b_without_busy_preempt() {
        let mut active = Some(active_goal("goal-a"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Idle,
            last_terminal: Some(TerminalOutcome::Succeeded),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        assert!(clear_terminal_active_session(&mut active, &terminal));
        assert!(active.is_none());

        let next = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-b".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        assert!(scheduler_session_is_active(&next, "goal-b"));
    }

    #[test]
    fn terminal_publication_transitions_to_unowned_guard_heartbeat() {
        let mut active = Some(active_goal("goal-a"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Idle,
            motion_sink_ready: true,
            guard_generation: 98,
            last_terminal: Some(TerminalOutcome::Cancelled),
            last_zero_ack_id: Some("zero-ack-a".into()),
            last_fence_monotonic_ns: 123,
            message: "verified fence".into(),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let mut telemetry = GuardedNavTelemetry {
            runtime_generation: 42,
            public_session_id: "goal-a".into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: "explore-a".into(),
            goal_generation: 1,
            cancel_generation: 2,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            goal: [1.0, 2.0, 0.0],
            current: [0.5, 0.25, 0.0],
            initial_distance: 3.0,
            progress: 0.5,
            path_revision: 3,
            goal_payload_sha256: [0x11; 32],
            path_payload_sha256: [0x22; 32],
        };

        let terminal_message =
            build_guarded_navigation_state(&terminal, &telemetry, None, Duration::from_secs(1));
        assert_eq!(terminal_message.session_id, "goal-a");
        assert_eq!(terminal_message.guard_generation, 98);
        assert_eq!(terminal_message.cancel_generation, 2);
        assert_eq!(terminal_message.zero_ack_id, "zero-ack-a");
        assert_eq!(terminal_message.zero_applied_monotonic_ns, 123);

        assert!(clear_terminal_session_after_publication(
            &mut active,
            &mut telemetry,
            &terminal,
        ));
        assert!(active.is_none());

        let heartbeat = build_guarded_navigation_state(
            &terminal,
            &telemetry,
            Some(true),
            Duration::from_secs(2),
        );
        assert!(heartbeat.session_id.is_empty());
        assert_eq!(heartbeat.state, 0);
        assert_eq!(heartbeat.message, "IDLE: guard client heartbeat");
        assert_eq!(heartbeat.guard_generation, 0);
        assert_eq!(heartbeat.sink_ready, 1);
        assert_eq!(heartbeat.cancel_generation, 0);
        assert!(heartbeat.zero_ack_id.is_empty());
        assert_eq!(heartbeat.zero_applied_monotonic_ns, 0);
        assert_eq!(heartbeat.owner_kind, 0);
        assert!(heartbeat.parent_request_id.is_empty());
        assert_eq!(heartbeat.goal_generation, 0);
        assert_eq!(heartbeat.path_revision, 0);
        assert_eq!(heartbeat.goal_payload_sha256, [0; 32]);
        assert_eq!(heartbeat.path_payload_sha256, [0; 32]);
        assert_eq!(heartbeat.runtime_generation, 42);
        assert_eq!(heartbeat.planner_map_frame, "map");
        assert_eq!(heartbeat.planner_pose_frame, "map");
        assert_eq!(heartbeat.current, [0.5, 0.25, 0.0]);
    }

    #[test]
    fn exploration_second_frontier_reuses_owner_after_first_child_terminal() {
        let mut active = Some(active_goal("explore-parent:frontier-1"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Failed,
            last_terminal: Some(TerminalOutcome::PlannerFailed),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        assert!(clear_terminal_active_session(&mut active, &terminal));

        let second = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Controlling,
            session_id: Some("explore-parent:frontier-2".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        assert!(scheduler_session_is_active(
            &second,
            "explore-parent:frontier-2"
        ));
    }

    #[test]
    fn path_and_cancel_match_active_goal_identity() {
        let goal = sealed_goal("public-goal-1");
        let path = sealed_path(&goal);
        validate_path_goal_correlation(&path, &goal).expect("correlated path");

        let mut foreign_path = path.clone();
        foreign_path.goal_payload_sha256[0] ^= 0xff;
        assert!(validate_path_goal_correlation(&foreign_path, &goal).is_err());

        let cancel = schema::CancelNavigation {
            session_id: goal.session_id.clone(),
            reason: 1,
            cancel_generation: 5,
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
        };
        validate_cancel_goal_correlation(&cancel, &goal).expect("correlated cancel");
        let mut foreign_cancel = cancel;
        foreign_cancel.goal_generation += 1;
        assert!(validate_cancel_goal_correlation(&foreign_cancel, &goal).is_err());
    }

    #[test]
    fn delayed_cancel_cannot_cross_a_geometry_producer_restart() {
        let mut goal_a = sealed_goal("same-public-goal");
        goal_a.goal_generation = 1;
        goal_a.producer_runtime_generation = 41;
        goal_a.payload_sha256 = [0; 32];
        schema::seal_goal_envelope(&mut goal_a).expect("seal producer 41 goal");

        let mut goal_b = goal_a.clone();
        goal_b.producer_runtime_generation = 42;
        goal_b.source_timestamp_ns += 1_000_000_000;
        goal_b.payload_sha256 = [0; 32];
        schema::seal_goal_envelope(&mut goal_b).expect("seal producer 42 goal");

        let delayed_a = schema::CancelNavigation {
            session_id: goal_a.session_id.clone(),
            reason: 1,
            cancel_generation: 7,
            owner_kind: goal_a.owner_kind,
            parent_request_id: goal_a.parent_request_id.clone(),
            goal_generation: goal_a.goal_generation,
            producer_runtime_generation: goal_a.producer_runtime_generation,
        };
        validate_cancel_goal_correlation(&delayed_a, &goal_a).expect("exact A cancel");
        assert!(
            validate_cancel_goal_correlation(&delayed_a, &goal_b).is_err(),
            "producer-41 cancellation must not match producer-42 goal generation 1"
        );
        let exact_b = schema::CancelNavigation {
            producer_runtime_generation: goal_b.producer_runtime_generation,
            ..delayed_a
        };
        validate_cancel_goal_correlation(&exact_b, &goal_b).expect("exact B cancel");
    }

    #[tokio::test]
    async fn delayed_cancel_does_not_cancel_or_zero_the_new_producer_session() {
        let harness = GuardedLoopHarness::spawn().await;
        let mut goal_a = sealed_goal("same-public-goal");
        goal_a.goal_generation = 1;
        goal_a.producer_runtime_generation = 41;
        goal_a.payload_sha256 = [0; 32];
        schema::seal_goal_envelope(&mut goal_a).expect("seal producer 41 goal");
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.send_goal(&goal_a);
        harness.wait_for_session(&goal_a.session_id).await;

        let mut goal_b = goal_a.clone();
        goal_b.producer_runtime_generation = 42;
        goal_b.source_timestamp_ns += 1_000_000_000;
        goal_b.payload_sha256 = [0; 32];
        schema::seal_goal_envelope(&mut goal_b).expect("seal producer 42 goal");
        harness.send_goal(&goal_b);
        harness.wait_for_zero().await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        harness.wait_for_session(&goal_b.session_id).await;
        let zero_count = harness
            .motion_calls
            .lock()
            .unwrap()
            .iter()
            .filter(|call| call.starts_with("zero:"))
            .count();

        harness.send_cancel(&goal_a, 2);
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some(goal_b.session_id.as_str()),
            "producer-41 cancellation must leave producer-42 session active"
        );
        assert_eq!(
            harness
                .motion_calls
                .lock()
                .unwrap()
                .iter()
                .filter(|call| call.starts_with("zero:"))
                .count(),
            zero_count,
            "rejected delayed cancellation must not request another ZERO"
        );

        harness.send_cancel(&goal_b, 3);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().session_id.is_some()
                || harness
                    .motion_calls
                    .lock()
                    .unwrap()
                    .iter()
                    .filter(|call| call.starts_with("zero:"))
                    .count()
                    == zero_count
            {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("exact producer-42 cancellation did not reach verified ZERO");
        harness.shutdown().await;
    }

    #[test]
    fn guarded_owner_orders_leases_by_producer_and_goal_generation() {
        let goal_a = sealed_goal("session-a");
        let mut goal_buffer = PlannerGoalBuffer::default();
        assert!(goal_buffer.observe_costmap().is_none());
        let ready_a = match goal_buffer.observe_goal(goal_a.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("first goal must be ready: {outcome:?}"),
        };
        assert!(goal_buffer.mark_applied(&ready_a));

        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);
        let ready_b = match goal_buffer.observe_goal(goal_b.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("newer lease must be ready: {outcome:?}"),
        };
        assert!(!goal_buffer.has_applied_goal());
        assert!(matches!(
            goal_buffer.observe_goal(goal_a.clone()),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::SupersededLease)
        ));
        assert!(goal_buffer.mark_applied(&ready_b));
        validate_path_goal_correlation(&sealed_path(&ready_b), &ready_b)
            .expect("new lease path remains correlated");
        assert!(goal_buffer.observe_costmap().is_none());

        let goal_a_again = rollover_goal(&goal_b, "session-a2", 9, 1);
        let ready_a_again = match goal_buffer.observe_goal(goal_a_again.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("new producer lease must activate: {outcome:?}"),
        };
        assert!(!goal_buffer.has_applied_goal());
        assert_eq!(ready_a_again, goal_a_again);
        assert!(goal_buffer.mark_applied(&ready_a_again));
    }

    #[tokio::test]
    async fn guarded_new_goal_fences_active_motion() {
        let motion_calls = Arc::new(Mutex::new(Vec::<String>::new()));
        let scheduler = nuwax_nav_runtime::NavScheduler::new(
            RecordingMotion(Arc::clone(&motion_calls)),
            CommandingPlanner(Arc::new(std::sync::atomic::AtomicBool::new(false))),
            nuwax_nav_runtime::SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                renew_interval: Duration::from_millis(100),
                robot_width_m: 0.3,
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
            },
        )
        .expect("guarded test scheduler");
        let (handle, task) = nuwax_nav_runtime::spawn_scheduler(scheduler);
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if status.borrow().motion_sink_ready {
                    break;
                }
                status.changed().await.expect("scheduler status open");
            }
        })
        .await
        .expect("motion readiness");

        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        pose.stamp = Timestamp::from_unix(1, 0);
        handle.update_odometry(pose).await.expect("map pose");
        handle
            .update_costmap(nuwax_nav_runtime::CostmapObservation {
                costmap: nuwax_nav_astar::navigation_map::CostMap2D::new_all_free("map", 4, 4, 0.1),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("map-a costmap");
        handle
            .start_session(nuwax_nav_runtime::NavSession {
                session_id: "session-a".into(),
                motion_admission: nuwax_nav_runtime::NavMotionAdmission {
                    guard_generation: 7,
                    guard_boot_nonce: [7; 16],
                    motion_stop_generation: 1,
                },
            })
            .await
            .expect("map-a session");
        handle
            .submit_path(nuwax_nav_runtime::VersionedPath {
                session_id: "session-a".into(),
                path_revision: 1,
                payload_hash: [0x11; 32],
                path: Path2D::stamped(
                    NavFrameId::Map,
                    Timestamp::from_unix(1, 0),
                    vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                ),
            })
            .await
            .expect("map-a path");
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if motion_calls
                    .lock()
                    .unwrap()
                    .iter()
                    .any(|call| call.starts_with("twist:"))
                {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("map-a APPLIED command");

        let goal_a = sealed_goal("session-a");
        let goal_b = rollover_goal(&goal_a, "session-b", 8, 1);
        let mut goal_buffer = PlannerGoalBuffer::default();
        assert!(goal_buffer.observe_costmap().is_none());
        let ready_a = match goal_buffer.observe_goal(goal_a.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("map-a goal ready: {outcome:?}"),
        };
        assert!(goal_buffer.mark_applied(&ready_a));
        let ready_b = match goal_buffer.observe_goal(goal_b.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => {
                panic!("new goal must be ready with fresh planner inputs: {outcome:?}")
            }
        };
        assert_eq!(ready_b, goal_b);
        assert!(!goal_buffer.has_applied_goal());

        let mut active_session = Some(goal_a);
        let mut telemetry = GuardedNavTelemetry {
            public_session_id: "session-a".into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            goal_generation: 3,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            path_revision: 1,
            goal_payload_sha256: [0x22; 32],
            path_payload_sha256: [0x33; 32],
            ..GuardedNavTelemetry::default()
        };
        fence_guarded_active_session(&handle, &mut telemetry, &mut active_session)
            .await
            .expect("pending map-b goal fences map-a");
        assert!(active_session.is_none());
        assert_eq!(telemetry.path_revision, 0);
        assert_eq!(telemetry.goal_payload_sha256, [0; 32]);
        assert_eq!(telemetry.path_payload_sha256, [0; 32]);
        assert!(goal_buffer.pending_goal.is_none());
        assert!(motion_calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("zero:")));

        handle.shutdown().await.expect("shutdown scheduler");
        task.await
            .expect("scheduler task")
            .expect("scheduler shutdown");
    }

    #[test]
    fn legacy_raw_goal_and_path_bodies_are_rejected_by_versioned_decoders() {
        let raw_goal =
            schema::encode_pose_stamped_body(&sealed_goal("goal").goal).expect("legacy goal body");
        assert!(schema::decode_goal_envelope_body(&raw_goal).is_err());

        let goal = sealed_goal("goal");
        let raw_path =
            schema::encode_path_body(&sealed_path(&goal).path).expect("legacy path body");
        assert!(schema::decode_path_envelope_body(&raw_path).is_err());
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
            OwnerInput::Accepted(())
        ));
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &tf_body).expect("duplicate TF is external"),
            OwnerInput::Rejected(_)
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
            OwnerInput::Accepted(())
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
    fn safety_runtime_errors_and_stream_closure_remain_fatal() {
        use nuwax_nav_runtime::RuntimeError;

        for error in [
            RuntimeError::Motion("guard unavailable".into()),
            RuntimeError::InvalidApplied("wrong operation".into()),
            RuntimeError::InvalidFence("missing zero proof".into()),
            RuntimeError::SchedulerStopped,
            RuntimeError::SequenceExhausted,
        ] {
            assert!(runtime_input_error::<()>("guarded-test", error).is_err());
        }
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
        let node = ModuleNode::new(
            format!("astar-cold-owner-{id}"),
            "A* cold owner regression",
            crate::graph::ModuleKind::RustAlgorithm,
        )
        .add_input(format!("{prefix}/pose"))
        .add_input(format!("{prefix}/costmap"))
        .add_input(format!("{prefix}/goal"))
        .add_input(format!("{prefix}/stop"))
        .add_output(format!("{prefix}/waypoint"))
        .add_output(format!("{prefix}/path"))
        .add_output(format!("{prefix}/reached"))
        .add_output(format!("{prefix}/failed"));
        let transport = lcm_transport().expect("real LCM transport");
        let mut paths = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.PathEnvelope")
            .await
            .expect("path subscription");
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let owner = tokio::spawn(run_astar_planner(node.clone(), shutdown_rx, ready_tx));
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
            "nuwax_msgs.GoalEnvelope",
            schema::encode_goal_envelope_body(&goal).expect("encode goal"),
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
        assert_eq!(path.goal_payload_sha256, goal.payload_sha256);

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
        let expected_hash = goal.payload_sha256;
        let expected_boundary = goal.boundary_points.clone();
        let mut buffer = PlannerGoalBuffer::default();
        assert!(matches!(
            buffer.observe_goal(goal.clone()),
            PlannerGoalIngress::Buffered
        ));
        assert_eq!(buffer.pending_goal.as_ref(), Some(&goal));

        let ready = buffer
            .observe_costmap()
            .expect("first valid costmap must release the buffered A* goal");
        assert_eq!(ready.payload_sha256, expected_hash);
        assert_eq!(ready.boundary_points, expected_boundary);

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::default());
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
            match apply_astar_planner_goal(&mut planner, *ready, &planner_test_grid_body(22))
                .expect("apply A* buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid A* goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert_eq!(resolution, 1.0);
        assert!(buffer.mark_applied(&applied));
        assert!(buffer.has_applied_goal());
        let _ = Module::tick(&mut planner).expect("A* planner tick");
        assert!(buffer.observe_costmap().is_none());
        assert!(buffer.pending_goal.is_none());
    }

    #[test]
    fn astar_new_goal_ignores_map_metadata_and_builds_correlated_path() {
        use nuwax_nav_astar::module::{
            Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
        };

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::default());
        NavigationInterface::start(&mut planner).expect("start A* planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(12, 0);
        Module::on_odom(&mut planner, robot_pose).expect("A* map pose");

        let mut buffer = PlannerGoalBuffer::default();
        assert!(buffer.observe_costmap().is_none());
        Module::on_costmap(
            &mut planner,
            decode_astar_costmap(&planner_test_grid_body(12)).expect("map 12"),
        )
        .expect("apply map 12");
        let goal_12 = planner_test_goal("astar-12", 1);
        let ready_12 = match buffer.observe_goal(goal_12.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("goal 12 must be ready, got {outcome:?}"),
        };
        let (applied_12, boundary_12, _) =
            match apply_astar_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert!(buffer.mark_applied(&applied_12));
        let mut active_goal = Some(applied_12);
        let mut active_boundary = boundary_12;
        let mut path_revision = 5_i64;

        let goal_13 = planner_test_goal("astar-13", 2);
        let ready_13 = match buffer.observe_goal(goal_13.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("newer lease must be ready: {outcome:?}"),
        };
        assert!(!buffer.has_applied_goal(), "superseded path must be gated");
        assert!(
            active_goal.is_some(),
            "outer map-a lease is not replaced early"
        );
        assert!(active_boundary.is_some());
        assert_eq!(path_revision, 5);

        clear_planner_goal_for_costmap_advance(
            &mut active_goal,
            &mut active_boundary,
            &mut path_revision,
        );
        assert!(
            !buffer.has_applied_goal(),
            "tick/path publication stays gated"
        );
        assert!(active_goal.is_none());
        assert!(active_boundary.is_none());
        assert_eq!(path_revision, 0);

        let (applied_13, boundary_13, _) =
            match apply_astar_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        assert!(buffer.mark_applied(&applied_13));
        active_goal = Some(applied_13);
        active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("astar-13")
        );
        assert!(active_boundary.is_some());
        assert!(buffer.has_applied_goal());
        let update = match Module::tick(&mut planner).expect("A* map-b tick") {
            ModuleOutput::Navigation(update) | ModuleOutput::SafetyHold(update) => update,
        };
        let path = update.path.expect("A* map-b path");
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 29,
                path_revision: 1,
                path: &path,
                boundary: active_boundary.as_ref(),
                map_resolution: 1.0,
            },
            13_000_000_000,
        )
        .expect("build A* map-b PathEnvelope")
        .expect("A* map-b path stays inside boundary");
        assert_eq!(
            envelope.goal_payload_sha256,
            active_goal.as_ref().unwrap().payload_sha256
        );
        assert!(matches!(
            buffer.observe_goal(goal_12),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::SupersededLease)
        ));
        assert!(buffer.observe_costmap().is_none());
        assert!(matches!(
            buffer.observe_goal(goal_13),
            PlannerGoalIngress::Rejected(PlannerGoalRejectReason::DuplicateLease)
        ));
        assert!(buffer.observe_costmap().is_none());
    }

    #[test]
    fn astar_owner_drops_bad_pose_and_grid_then_accepts_new_valid_inputs() {
        use nuwax_nav_astar::module::{Module, NavigationInterface, ReplanningAStarPlannerModule};

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::default());
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

    fn goal(owner_kind: i32, boundary_points: Vec<[f64; 3]>) -> schema::GoalEnvelope {
        schema::GoalEnvelope {
            owner_kind,
            boundary_frame_id: if boundary_points.is_empty() {
                String::new()
            } else {
                "map".into()
            },
            boundary_points,
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
            ..schema::GoalEnvelope::default()
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

        let cloud = decode_simple_occupancy_with_boundary(&free_grid_body(), Some(&boundary))
            .expect("masked simple occupancy");
        assert!(cloud
            .points
            .iter()
            .any(|point| point.to_2d().distance(Point2::new(3.5, 5.5)) < 1e-9));
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
